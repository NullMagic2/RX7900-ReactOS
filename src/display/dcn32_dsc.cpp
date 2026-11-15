/*
 * Copyright 2012-2019 Advanced Micro Devices, Inc.
 * Copyright (c) 2018 Intel Corp
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Authors: AMD
 */
/*
Module: rx7900.display.dcn32_dsc
Purpose: DSC sink caps, the DC slice/bpp policy, rate control, PPS and DSC engine programming for DCN 3.2.
Created: 2026-10-24 (adaptation; the upstream notices above are preserved)
Architecture: Implementation of src/display/dcn32_dsc.hpp. Statement order and
integer types follow the pinned Linux v6.12 sources named there
(work/upstream-dcn-dsc-odm, SHA-256 in its SHA256SUMS), so each function can be
compared with its upstream text. Deliberate differences: the float/double
expressions of _do_calc_rc_params are evaluated exactly (rationals with a /16
denominator, and the binary64 products of 8/3.0, 4/3.0 and 2/3.0 rounded to
nearest-even as an x86-64 build does), only the RGB 8 bpc rows of the qp tables
are present, an out-of-table bpp is refused instead of leaving the qp sets
uninitialized, and the logging is removed.
*/
#include "dcn32_dsc.hpp"

// Keeps the large DSC locals in frames of their own (kernel stack).
#if defined(_MSC_VER)
#define DCN32_DSC_NOINLINE __declspec(noinline)
#else
#define DCN32_DSC_NOINLINE __attribute__((noinline))
#endif

namespace rx7900::display {

namespace r = dcn32_stream_reg;

namespace {

// ---- dc/basics/fixpt31_32.c and include/fixed31_32.h ----

constexpr uint32_t fixed_fraction_bits = 32;
constexpr uint64_t fixed_fraction_mask = (1ull << fixed_fraction_bits) - 1;

struct Fixed final {
    int64_t value;
};

constexpr Fixed fixed_half{0x80000000ll};
constexpr Fixed fixed_one{0x100000000ll};
constexpr Fixed fixed_epsilon{1};

Fixed fixpt_from_int(int arg) noexcept {
    // (long long)arg << 32, written as a multiplication (defined for negatives).
    return {static_cast<int64_t>(arg) * static_cast<int64_t>(1ll << fixed_fraction_bits)};
}

uint64_t magnitude(int64_t value) noexcept {
    return value < 0 ? 0 - static_cast<uint64_t>(value) : static_cast<uint64_t>(value);
}

// dc_fixpt_from_fraction.
Fixed fixpt_from_fraction(int64_t numerator, int64_t denominator) noexcept {
    const bool arg1_negative = numerator < 0;
    const bool arg2_negative = denominator < 0;
    const uint64_t arg1_value = magnitude(numerator);
    const uint64_t arg2_value = magnitude(denominator);
    if (arg2_value == 0) {
        return {0};
    }
    uint64_t remainder = arg1_value % arg2_value;
    uint64_t res_value = arg1_value / arg2_value;
    for (uint32_t i = fixed_fraction_bits; i != 0; --i) {
        remainder <<= 1;
        res_value <<= 1;
        if (remainder >= arg2_value) {
            res_value |= 1;
            remainder -= arg2_value;
        }
    }
    const uint64_t summand = (remainder << 1) >= arg2_value ? 1u : 0u;
    res_value += summand;
    int64_t value = static_cast<int64_t>(res_value);
    if (arg1_negative != arg2_negative) {
        value = -value;
    }
    return {value};
}

// dc_fixpt_mul.
Fixed fixpt_mul(Fixed arg1, Fixed arg2) noexcept {
    const bool arg1_negative = arg1.value < 0;
    const bool arg2_negative = arg2.value < 0;
    const uint64_t arg1_value = magnitude(arg1.value);
    const uint64_t arg2_value = magnitude(arg2.value);
    const uint64_t arg1_int = arg1_value >> fixed_fraction_bits;
    const uint64_t arg2_int = arg2_value >> fixed_fraction_bits;
    const uint64_t arg1_fra = arg1_value & fixed_fraction_mask;
    const uint64_t arg2_fra = arg2_value & fixed_fraction_mask;
    uint64_t value = arg1_int * arg2_int;
    value <<= fixed_fraction_bits;
    value += arg1_int * arg2_fra;
    value += arg2_int * arg1_fra;
    uint64_t tmp = arg1_fra * arg2_fra;
    tmp = (tmp >> fixed_fraction_bits) + (tmp >= static_cast<uint64_t>(fixed_half.value) ? 1u : 0u);
    value += tmp;
    int64_t result = static_cast<int64_t>(value);
    if (arg1_negative != arg2_negative) {
        result = -result;
    }
    return {result};
}

Fixed fixpt_mul_int(Fixed arg1, int arg2) noexcept { return fixpt_mul(arg1, fixpt_from_int(arg2)); }
Fixed fixpt_div_int(Fixed arg1, int64_t arg2) noexcept {
    return fixpt_from_fraction(arg1.value, fixpt_from_int(static_cast<int>(arg2)).value);
}
Fixed fixpt_add_int(Fixed arg1, int arg2) noexcept { return {arg1.value + fixpt_from_int(arg2).value}; }
Fixed fixpt_sub_int(Fixed arg1, int arg2) noexcept { return {arg1.value - fixpt_from_int(arg2).value}; }

int fixpt_floor(Fixed arg) noexcept {
    const uint64_t arg_value = magnitude(arg.value);
    const int whole = static_cast<int>(arg_value >> fixed_fraction_bits);
    return arg.value >= 0 ? whole : -whole;
}

int fixpt_ceil(Fixed arg) noexcept {
    uint64_t arg_value = magnitude(arg.value);
    arg_value += static_cast<uint64_t>(fixed_one.value - fixed_epsilon.value);
    const int whole = static_cast<int>(arg_value >> fixed_fraction_bits);
    return arg.value >= 0 ? whole : -whole;
}

// ---- dc/dsc/dc_dsc.c ----

// dsc_policy_max_target_bpp_limit and the default options of
// dc_dsc_get_default_config_option (no debug overrides).
constexpr uint32_t policy_max_target_bpp_limit = 16;
constexpr int min_available_slices_size = 6;

int min_int(int a, int b) noexcept { return a < b ? a : b; }

bool dsc_buff_block_size_from_dpcd(int dpcd_buff_block_size, int& buff_block_size) noexcept {
    switch (dpcd_buff_block_size) {
    case 0x0: buff_block_size = 1024; break;
    case 0x1: buff_block_size = 4 * 1024; break;
    case 0x2: buff_block_size = 16 * 1024; break;
    case 0x3: buff_block_size = 64 * 1024; break;
    default: return false;
    }
    return true;
}

bool dsc_line_buff_depth_from_dpcd(int dpcd_line_buff_bit_depth, int32_t& line_buff_bit_depth) noexcept {
    if (0 <= dpcd_line_buff_bit_depth && dpcd_line_buff_bit_depth <= 7) {
        line_buff_bit_depth = dpcd_line_buff_bit_depth + 9;
    } else if (dpcd_line_buff_bit_depth == 8) {
        line_buff_bit_depth = 8;
    } else {
        return false;
    }
    return true;
}

bool dsc_throughput_from_dpcd(int dpcd_throughput, int32_t& throughput) noexcept {
    // DP_DSC_THROUGHPUT_MODE_0_UNSUPPORTED, 340, 400, ... 1000, then 170 (1.4a).
    static const int32_t mps[16] = {0, 340, 400, 450, 500, 550, 600, 650, 700, 750, 800, 850, 900, 950, 1000, 170};
    if (dpcd_throughput < 0 || dpcd_throughput > 15) {
        return false;
    }
    throughput = mps[dpcd_throughput];
    return true;
}

bool dsc_bpp_increment_div_from_dpcd(uint8_t bpp_increment_dpcd, uint32_t& bpp_increment_div) noexcept {
    switch (bpp_increment_dpcd & 0x7u) {
    case 0: bpp_increment_div = 16; break;
    case 1: bpp_increment_div = 8; break;
    case 2: bpp_increment_div = 4; break;
    case 3: bpp_increment_div = 2; break;
    case 4: bpp_increment_div = 1; break;
    default: return false;
    }
    return true;
}

// intersect_dsc_caps.
bool intersect_dsc_caps(const DscSinkCaps& sink, const DscEncoderCaps& enc, DscEncoderCaps& common) noexcept {
    common = DscEncoderCaps{};
    common.dsc_version = sink.dsc_version < enc.dsc_version ? sink.dsc_version : enc.dsc_version;
    if (common.dsc_version == 0) {
        return false;
    }
    uint8_t slices = 0;
    slices |= ((sink.slice_caps1 & 0x01u) && (enc.slice_caps & dsc_slices_1)) ? dsc_slices_1 : 0u;
    slices |= ((sink.slice_caps1 & 0x02u) && (enc.slice_caps & dsc_slices_2)) ? dsc_slices_2 : 0u;
    slices |= ((sink.slice_caps1 & 0x08u) && (enc.slice_caps & dsc_slices_4)) ? dsc_slices_4 : 0u;
    slices |= ((sink.slice_caps1 & 0x20u) && (enc.slice_caps & dsc_slices_8)) ? dsc_slices_8 : 0u;
    slices |= ((sink.slice_caps1 & 0x80u) && (enc.slice_caps & dsc_slices_12)) ? dsc_slices_12 : 0u;
    slices |= ((sink.slice_caps2 & 0x01u) && (enc.slice_caps & dsc_slices_16)) ? dsc_slices_16 : 0u;
    common.slice_caps = slices;
    if (common.slice_caps == 0) {
        return false;
    }
    common.lb_bit_depth = sink.lb_bit_depth < enc.lb_bit_depth ? sink.lb_bit_depth : enc.lb_bit_depth;
    if (common.lb_bit_depth == 0) {
        return false;
    }
    common.is_block_pred_supported = sink.is_block_pred_supported && enc.is_block_pred_supported;
    common.color_formats = static_cast<uint8_t>(sink.color_formats & enc.color_formats);
    if (common.color_formats == 0) {
        return false;
    }
    common.color_depth = static_cast<uint8_t>(sink.color_depth & enc.color_depth);
    if (common.color_depth == 0) {
        return false;
    }
    int32_t max_slices = 0;
    if (common.slice_caps & dsc_slices_1) {
        max_slices = 1;
    }
    if (common.slice_caps & dsc_slices_2) {
        max_slices = 2;
    }
    if (common.slice_caps & dsc_slices_4) {
        max_slices = 4;
    }
    // PIXEL_ENCODING_RGB: throughput mode 0.
    const int32_t total_sink_throughput = max_slices * sink.throughput_mode_0_mps;
    common.max_total_throughput_mps = total_sink_throughput < enc.max_total_throughput_mps ?
                                      total_sink_throughput : enc.max_total_throughput_mps;
    common.max_slice_width = sink.max_slice_width < enc.max_slice_width ? sink.max_slice_width : enc.max_slice_width;
    if (common.max_slice_width == 0) {
        return false;
    }
    common.bpp_increment_div = sink.bpp_increment_div < enc.bpp_increment_div ? sink.bpp_increment_div
                                                                              : enc.bpp_increment_div;
    common.edp_sink_max_bits_per_pixel = static_cast<uint32_t>(sink.edp_max_bits_per_pixel);
    common.is_dp = sink.is_dp;
    return true;
}

uint32_t dsc_div_by_10_round_up(uint32_t value) noexcept { return (value + 9) / 10; }

// dc_dsc_stream_bandwidth_overhead_in_kbps (dsc_policy_disable_dsc_stream_overhead false).
uint32_t stream_overhead_kbps(const DscTiming& timing, int num_slices_h, bool is_dp) noexcept {
    if (!is_dp || timing.h_total == 0 || timing.v_total == 0) {
        return 0;
    }
    Fixed refresh_rate = fixpt_from_int(static_cast<int>(timing.pixel_clock_100hz));
    refresh_rate = fixpt_div_int(refresh_rate, timing.h_total);
    refresh_rate = fixpt_div_int(refresh_rate, timing.v_total);
    refresh_rate = fixpt_mul_int(refresh_rate, 100);
    Fixed max_dsc_overhead = fixpt_from_int(num_slices_h);
    max_dsc_overhead = fixpt_mul_int(max_dsc_overhead, static_cast<int>(timing.v_total));
    max_dsc_overhead = fixpt_mul_int(max_dsc_overhead, 256);
    max_dsc_overhead = fixpt_div_int(max_dsc_overhead, 1000);
    max_dsc_overhead = fixpt_mul(max_dsc_overhead, refresh_rate);
    return static_cast<uint32_t>(fixpt_ceil(max_dsc_overhead));
}

// compute_bpp_x16_from_target_bandwidth.
uint32_t compute_bpp_x16_from_target_bandwidth(uint32_t bandwidth_in_kbps, const DscTiming& timing,
                                               uint32_t num_slices_h, uint32_t bpp_increment_div,
                                               bool is_dp) noexcept {
    const uint32_t overhead_in_kbps = stream_overhead_kbps(timing, static_cast<int>(num_slices_h), is_dp);
    Fixed effective = fixpt_from_int(static_cast<int>(bandwidth_in_kbps));
    effective = fixpt_sub_int(effective, static_cast<int>(overhead_in_kbps));
    Fixed bpp_x16 = fixpt_mul_int(effective, 10);
    bpp_x16 = fixpt_div_int(bpp_x16, timing.pixel_clock_100hz);
    bpp_x16 = fixpt_from_int(fixpt_floor(fixpt_mul_int(bpp_x16, static_cast<int>(bpp_increment_div))));
    bpp_x16 = fixpt_div_int(bpp_x16, bpp_increment_div);
    bpp_x16 = fixpt_mul_int(bpp_x16, 16);
    return static_cast<uint32_t>(fixpt_floor(bpp_x16));
}

struct BandwidthRange final {
    uint32_t min_target_bpp_x16 = 0;
    uint32_t max_target_bpp_x16 = 0;
    uint32_t min_kbps = 0;
    uint32_t max_kbps = 0;
    uint32_t stream_kbps = 0;
};

// decide_dsc_bandwidth_range (no dsc_fixed_bits_per_pixel_x16).
bool decide_dsc_bandwidth_range(uint32_t min_bpp_x16, uint32_t max_bpp_x16, uint32_t num_slices_h,
                                const DscEncoderCaps& caps, const DscTiming& timing,
                                BandwidthRange& range) noexcept {
    range = BandwidthRange{};
    if (caps.edp_sink_max_bits_per_pixel) {
        range.max_target_bpp_x16 = caps.edp_sink_max_bits_per_pixel < max_bpp_x16 ?
                                   caps.edp_sink_max_bits_per_pixel : max_bpp_x16;
        range.min_target_bpp_x16 = min_bpp_x16;
    } else {
        range.max_target_bpp_x16 = max_bpp_x16;
        range.min_target_bpp_x16 = min_bpp_x16;
    }
    if (range.max_target_bpp_x16 >= range.min_target_bpp_x16 && range.min_target_bpp_x16 > 0) {
        range.stream_kbps = uncompressed_bandwidth_kbps(timing);
        range.max_kbps = dsc_stream_bandwidth_kbps(timing, range.max_target_bpp_x16, num_slices_h, caps.is_dp);
        range.min_kbps = dsc_stream_bandwidth_kbps(timing, range.min_target_bpp_x16, num_slices_h, caps.is_dp);
    }
    return range.max_kbps >= range.min_kbps && range.min_kbps > 0;
}

// dc_dsc_get_policy_for_timing for COLOR_DEPTH_888 RGB, no override.
struct DscPolicy final {
    bool use_min_slices_h = true;
    int max_slices_h = 0;
    int min_slice_height = 108;
    uint32_t min_target_bpp = 8;
    uint32_t max_target_bpp = 24;
    bool enable_dsc_when_not_needed = false;
};

DscPolicy dsc_policy() noexcept {
    DscPolicy policy{};
    policy.min_target_bpp = 8;
    policy.max_target_bpp = 3 * 8;
    if (policy.max_target_bpp > policy_max_target_bpp_limit) {
        policy.max_target_bpp = policy_max_target_bpp_limit;
    }
    return policy;
}

// decide_dsc_target_bpp_x16 (force_dsc_when_not_needed false).
bool decide_dsc_target_bpp_x16(const DscPolicy& policy, const DscEncoderCaps& common, int target_bandwidth_kbps,
                               const DscTiming& timing, int num_slices_h, int& target_bpp_x16) noexcept {
    target_bpp_x16 = 0;
    BandwidthRange range{};
    if (decide_dsc_bandwidth_range(policy.min_target_bpp * 16, policy.max_target_bpp * 16,
                                   static_cast<uint32_t>(num_slices_h), common, timing, range)) {
        if (target_bandwidth_kbps >= static_cast<int>(range.stream_kbps)) {
            if (policy.enable_dsc_when_not_needed) {
                target_bpp_x16 = static_cast<int>(range.max_target_bpp_x16);
            }
        } else if (target_bandwidth_kbps >= static_cast<int>(range.max_kbps)) {
            target_bpp_x16 = static_cast<int>(range.max_target_bpp_x16);
        } else if (target_bandwidth_kbps >= static_cast<int>(range.min_kbps)) {
            target_bpp_x16 = static_cast<int>(compute_bpp_x16_from_target_bandwidth(
                static_cast<uint32_t>(target_bandwidth_kbps), timing, static_cast<uint32_t>(num_slices_h),
                common.bpp_increment_div, common.is_dp));
        }
    }
    return target_bpp_x16 != 0;
}

int get_available_dsc_slices(uint8_t slice_caps, int (&available_slices)[min_available_slices_size]) noexcept {
    int idx = 0;
    if (slice_caps & dsc_slices_1) {
        available_slices[idx++] = 1;
    }
    if (slice_caps & dsc_slices_2) {
        available_slices[idx++] = 2;
    }
    if (slice_caps & dsc_slices_4) {
        available_slices[idx++] = 4;
    }
    if (slice_caps & dsc_slices_8) {
        available_slices[idx++] = 8;
    }
    if (slice_caps & dsc_slices_12) {
        available_slices[idx++] = 12;
    }
    if (slice_caps & dsc_slices_16) {
        available_slices[idx++] = 16;
    }
    return idx;
}

int get_max_dsc_slices(uint8_t slice_caps) noexcept {
    int available_slices[min_available_slices_size]{};
    const int end_idx = get_available_dsc_slices(slice_caps, available_slices);
    return end_idx > 0 ? available_slices[end_idx - 1] : 0;
}

int inc_num_slices(uint8_t slice_caps, int num_slices) noexcept {
    int available_slices[min_available_slices_size]{};
    const int end_idx = get_available_dsc_slices(slice_caps, available_slices);
    int new_num_slices = num_slices;
    if (end_idx == 0) {
        return new_num_slices + 1;
    }
    for (int i = 0; i < end_idx; i++) {
        if (new_num_slices < available_slices[i]) {
            new_num_slices = available_slices[i];
            break;
        }
    }
    if (new_num_slices == num_slices) {
        new_num_slices++;
    }
    return new_num_slices;
}

int dec_num_slices(uint8_t slice_caps, int num_slices) noexcept {
    int available_slices[min_available_slices_size]{};
    const int end_idx = get_available_dsc_slices(slice_caps, available_slices);
    int new_num_slices = num_slices;
    if (end_idx == 0 && new_num_slices > 0) {
        return new_num_slices + 1;
    }
    for (int i = end_idx - 1; i >= 0; i--) {
        if (new_num_slices > available_slices[i]) {
            new_num_slices = available_slices[i];
            break;
        }
    }
    if (new_num_slices == num_slices) {
        new_num_slices--;
        if (new_num_slices < 0) {
            new_num_slices = 0;
        }
    }
    return new_num_slices;
}

int fit_num_slices_up(uint8_t slice_caps, int num_slices) noexcept {
    int available_slices[min_available_slices_size]{};
    const int end_idx = get_available_dsc_slices(slice_caps, available_slices);
    int new_num_slices = num_slices;
    if (end_idx == 0) {
        return new_num_slices + 1;
    }
    for (int i = 0; i < end_idx; i++) {
        if (new_num_slices <= available_slices[i]) {
            new_num_slices = available_slices[i];
            break;
        }
    }
    return new_num_slices;
}

// setup_dsc_config for PIXEL_ENCODING_RGB, COLOR_DEPTH_888 and the default
// options (no slice height override, no forced ODM slicing, granularity 1).
bool setup_dsc_config(const DscSinkCaps& sink, const DscEncoderCaps& enc, int target_bandwidth_kbps,
                      const DscTiming& timing, DscConfig& config) noexcept {
    config = DscConfig{};
    const DscPolicy policy = dsc_policy();
    const int pic_width = static_cast<int>(timing.h_active);
    const int pic_height = static_cast<int>(timing.v_active);
    DscEncoderCaps common{};
    bool is_dsc_possible = false;

    if (!sink.is_dsc_supported) {
        return false;
    }
    if (sink.branch_max_line_width && static_cast<int>(sink.branch_max_line_width) < pic_width) {
        return false;
    }
    is_dsc_possible = intersect_dsc_caps(sink, enc, common);
    if (!is_dsc_possible) {
        return false;
    }
    // PIXEL_ENCODING_RGB.
    is_dsc_possible = (common.color_formats & 0x01u) != 0;
    const int sink_per_slice_throughput_mps = sink.throughput_mode_0_mps;
    const uint32_t branch_max_throughput_mps = sink.branch_overall_throughput_0_mps;
    if (branch_max_throughput_mps &&
        dsc_div_by_10_round_up(timing.pixel_clock_100hz) > branch_max_throughput_mps * 1000) {
        is_dsc_possible = false;
    }
    if (!is_dsc_possible) {
        return false;
    }
    // COLOR_DEPTH_888.
    if ((common.color_depth & 0x02u) == 0) {
        return false;
    }

    int max_slices_h = get_max_dsc_slices(common.slice_caps);
    while (max_slices_h > 0) {
        if (pic_width % max_slices_h == 0) {
            break;
        }
        max_slices_h = dec_num_slices(common.slice_caps, max_slices_h);
    }
    if (common.max_slice_width <= 0) {
        return false;
    }
    int min_slices_h = pic_width / common.max_slice_width;
    if (pic_width % common.max_slice_width) {
        min_slices_h++;
    }
    min_slices_h = fit_num_slices_up(common.slice_caps, min_slices_h);
    while (min_slices_h <= max_slices_h) {
        const int pix_clk_per_slice_khz =
            static_cast<int>(dsc_div_by_10_round_up(timing.pixel_clock_100hz)) / min_slices_h;
        if (pix_clk_per_slice_khz <= sink_per_slice_throughput_mps * 1000) {
            break;
        }
        min_slices_h = inc_num_slices(common.slice_caps, min_slices_h);
    }
    is_dsc_possible = min_slices_h <= max_slices_h;
    if (min_slices_h == 0 || pic_width % min_slices_h != 0) {
        min_slices_h = 0;
    }
    if (min_slices_h == 0 && max_slices_h == 0) {
        is_dsc_possible = false;
    }
    if (!is_dsc_possible) {
        return false;
    }

    int num_slices_h = 0;
    if (policy.use_min_slices_h) {
        if (min_slices_h > 0) {
            num_slices_h = min_slices_h;
        } else if (max_slices_h > 0) {
            num_slices_h = policy.max_slices_h ? min_int(policy.max_slices_h, max_slices_h) : max_slices_h;
        } else {
            is_dsc_possible = false;
        }
    }
    if (num_slices_h == 0) {
        is_dsc_possible = false;
    }
    if (!is_dsc_possible) {
        return false;
    }
    config.num_slices_h = static_cast<uint32_t>(num_slices_h);
    const int slice_width = pic_width / num_slices_h;
    if (slice_width > common.max_slice_width) {
        config = DscConfig{};
        return false;
    }

    int slice_height = min_int(policy.min_slice_height, pic_height);
    while (slice_height < pic_height && pic_height % slice_height != 0) {
        slice_height++;
    }
    if (slice_height <= 0) {
        config = DscConfig{};
        return false;
    }
    config.num_slices_v = static_cast<uint32_t>(pic_height / slice_height);

    if (target_bandwidth_kbps > 0) {
        int target_bpp = 0;
        is_dsc_possible = decide_dsc_target_bpp_x16(policy, common, target_bandwidth_kbps, timing, num_slices_h,
                                                    target_bpp);
        config.bits_per_pixel = static_cast<uint32_t>(target_bpp);
    }
    if (!is_dsc_possible) {
        config = DscConfig{};
        return false;
    }
    config.block_pred_enable = common.is_block_pred_supported;
    config.linebuf_depth = static_cast<uint32_t>(common.lb_bit_depth);
    config.version_minor = (common.dsc_version & 0xf0u) >> 4;
    config.is_dp = sink.is_dp;
    return true;
}

// ---- dc/dml/dsc/rc_calc_fpu.c (exact evaluation) and qp_tables.h ----

constexpr uint32_t qp_set_size = 15;
constexpr uint32_t qp_table_rows = 37;  // bpp 6.0 .. 24.0 in steps of 0.5

// qp_table_444_8bpc_max / qp_table_444_8bpc_min.
const uint8_t qp_table_444_8bpc_max[qp_table_rows][qp_set_size] = {
    {4, 6, 8, 8, 9, 9, 9, 10, 11, 12, 12, 12, 12, 13, 15}, {4, 6, 7, 8, 8, 8, 9, 10, 11, 11, 12, 12, 12, 13, 15},
    {4, 5, 7, 7, 8, 8, 8, 9, 10, 11, 11, 12, 12, 13, 14}, {4, 5, 6, 7, 7, 8, 8, 9, 10, 10, 11, 11, 12, 13, 14},
    {4, 4, 5, 6, 7, 7, 7, 8, 9, 10, 10, 11, 11, 12, 13}, {4, 4, 5, 6, 7, 7, 7, 8, 9, 10, 10, 11, 11, 12, 13},
    {3, 4, 5, 6, 7, 7, 7, 8, 9, 9, 10, 10, 11, 11, 13}, {3, 4, 5, 6, 7, 7, 7, 8, 9, 9, 10, 10, 11, 11, 13},
    {3, 4, 5, 6, 7, 7, 7, 8, 9, 9, 10, 10, 11, 11, 12}, {3, 4, 5, 6, 7, 7, 7, 8, 9, 9, 10, 10, 10, 11, 12},
    {2, 4, 5, 6, 7, 7, 7, 8, 8, 9, 9, 9, 10, 10, 11}, {2, 4, 5, 6, 7, 7, 7, 8, 8, 9, 9, 9, 9, 10, 11},
    {2, 3, 4, 5, 6, 6, 7, 8, 8, 9, 9, 9, 9, 10, 11}, {2, 3, 4, 5, 6, 6, 6, 7, 7, 8, 8, 9, 9, 10, 11},
    {1, 3, 4, 5, 5, 6, 6, 7, 7, 8, 8, 8, 8, 9, 10}, {1, 2, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 10},
    {1, 2, 2, 3, 4, 4, 4, 5, 6, 6, 7, 8, 8, 8, 10}, {0, 1, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 9},
    {0, 1, 1, 2, 2, 3, 3, 4, 5, 5, 6, 7, 7, 8, 9}, {0, 1, 1, 2, 2, 3, 3, 4, 5, 5, 6, 7, 7, 8, 9},
    {0, 1, 1, 2, 2, 3, 3, 4, 5, 5, 5, 6, 6, 7, 8}, {0, 1, 1, 2, 2, 3, 3, 4, 5, 5, 5, 6, 6, 7, 8},
    {0, 1, 1, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 6, 8}, {0, 1, 1, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 6, 8},
    {0, 1, 1, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 6, 7}, {0, 1, 1, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 6, 7},
    {0, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 4, 4, 5, 6}, {0, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 4, 4, 5, 6},
    {0, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 3, 3, 4, 6}, {0, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 3, 3, 4, 6},
    {0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 4, 5}, {0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 4, 5},
    {0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 5}, {0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 4},
    {0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 4}, {0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 4},
    {0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 2, 4}};
const uint8_t qp_table_444_8bpc_min[qp_table_rows][qp_set_size] = {
    {0, 1, 3, 3, 5, 5, 5, 5, 5, 6, 6, 6, 6, 9, 14}, {0, 1, 2, 3, 4, 4, 5, 5, 5, 5, 6, 6, 6, 9, 14},
    {0, 0, 2, 2, 4, 4, 4, 4, 4, 5, 5, 6, 6, 9, 13}, {0, 0, 2, 2, 3, 4, 4, 4, 4, 4, 5, 5, 6, 9, 13},
    {0, 0, 1, 1, 3, 3, 3, 3, 3, 4, 5, 5, 5, 8, 12}, {0, 0, 1, 1, 3, 3, 3, 3, 3, 4, 5, 5, 5, 8, 12},
    {0, 0, 1, 1, 3, 3, 3, 3, 3, 3, 5, 5, 5, 7, 12}, {0, 0, 1, 1, 3, 3, 3, 3, 3, 3, 5, 5, 5, 7, 12},
    {0, 0, 1, 1, 3, 3, 3, 3, 3, 3, 5, 5, 5, 7, 11}, {0, 0, 1, 1, 3, 3, 3, 3, 3, 3, 5, 5, 5, 7, 11},
    {0, 0, 1, 1, 3, 3, 3, 3, 3, 3, 5, 5, 5, 7, 10}, {0, 0, 1, 1, 3, 3, 3, 3, 3, 3, 5, 5, 5, 7, 10},
    {0, 0, 1, 1, 3, 3, 3, 3, 3, 3, 5, 5, 5, 7, 10}, {0, 0, 1, 1, 3, 3, 3, 3, 3, 3, 4, 5, 5, 7, 10},
    {0, 0, 1, 1, 2, 3, 3, 3, 3, 3, 4, 5, 5, 7, 9}, {0, 0, 0, 1, 1, 2, 2, 2, 2, 3, 4, 5, 5, 7, 9},
    {0, 0, 0, 0, 1, 1, 2, 2, 2, 3, 4, 5, 5, 7, 9}, {0, 0, 0, 0, 1, 1, 2, 2, 2, 3, 4, 4, 5, 7, 8},
    {0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 4, 4, 5, 7, 8}, {0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 4, 4, 5, 7, 8},
    {0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 4, 4, 5, 6, 7}, {0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 4, 4, 5, 6, 7},
    {0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 3, 4, 4, 5, 7}, {0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 3, 4, 4, 5, 7},
    {0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 3, 4, 4, 5, 6}, {0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 3, 4, 4, 5, 6},
    {0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 3, 3, 4, 5}, {0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 3, 3, 4, 5},
    {0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 3, 5}, {0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 3, 5},
    {0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 3, 4}, {0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 3, 4},
    {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 4}, {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 3},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 3}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 3},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 3}};

// struct rc_params.
struct RcParams final {
    int rc_quant_incr_limit0;
    int rc_quant_incr_limit1;
    int initial_fullness_offset;
    int initial_xmit_delay;
    int first_line_bpg_offset;
    int second_line_bpg_offset;
    int flatness_min_qp;
    int flatness_max_qp;
    int flatness_det_thresh;
    int qp_min[qp_set_size];
    int qp_max[qp_set_size];
    int ofs[qp_set_size];
    int rc_model_size;
    int rc_edge_factor;
    int rc_tgt_offset_hi;
    int rc_tgt_offset_lo;
    int rc_buf_thresh[qp_set_size - 1];
};

// A positive binary64 value mantissa * 2^exponent, mantissa in [2^52, 2^53).
struct Binary64 final {
    uint64_t mantissa;
    int32_t exponent;
};

// 8 / 3.0, 4 / 3.0 and 2 / 3.0 as the compiler stores them (0x4005555555555555,
// 0x3ff5555555555555, 0x3fe5555555555555).
constexpr Binary64 eight_thirds{0x15555555555555ull, -51};
constexpr Binary64 four_thirds{0x15555555555555ull, -52};
constexpr Binary64 two_thirds{0x15555555555555ull, -53};

uint32_t bit_length(uint64_t value) noexcept {
    uint32_t bits = 0;
    while (value != 0) {
        value >>= 1;
        ++bits;
    }
    return bits;
}

// Rounds value * 2^exponent to 53 significant bits, ties to even.
void round_to_binary64(uint64_t& value, int32_t& exponent) noexcept {
    const uint32_t bits = bit_length(value);
    if (bits <= 53) {
        return;
    }
    const uint32_t shift = bits - 53;
    uint64_t quotient = value >> shift;
    const uint64_t remainder = value & ((1ull << shift) - 1);
    const uint64_t half = 1ull << (shift - 1);
    uint32_t applied = shift;
    if (remainder > half || (remainder == half && (quotient & 1u) != 0)) {
        ++quotient;
        if (quotient == (1ull << 53)) {
            quotient >>= 1;
            ++applied;
        }
    }
    value = quotient;
    exponent += static_cast<int32_t>(applied);
}

// dsc_roundf((x16 / 16.0f) * c) for one of the constants above: the product
// and the +-0.5 addition are rounded to binary64 as an SSE2 build does, then
// truncated toward zero.
int dsc_round_product(int32_t x16, const Binary64& c) noexcept {
    if (x16 == 0) {
        return 0;
    }
    const bool negative = x16 < 0;
    uint64_t value = static_cast<uint64_t>(negative ? -static_cast<int64_t>(x16) : x16) * c.mantissa;
    int32_t exponent = c.exponent - 4;
    round_to_binary64(value, exponent);
    // + 0.5 (sign-symmetric: num - 0.5 for a negative num).
    if (exponent <= -1) {
        value += 1ull << static_cast<uint32_t>(-1 - exponent);
    } else {
        value = (value << static_cast<uint32_t>(exponent + 1)) + 1;
        exponent = -1;
    }
    round_to_binary64(value, exponent);
    const int64_t truncated = exponent >= 0 ? static_cast<int64_t>(value << static_cast<uint32_t>(exponent))
                                            : static_cast<int64_t>(value >> static_cast<uint32_t>(-exponent));
    return static_cast<int>(negative ? -truncated : truncated);
}

// dsc_roundf(x16 / 16.0) for an exact /16 rational: num + 0.5 truncated.
int dsc_round_sixteenths(int32_t x16) noexcept {
    return x16 >= 0 ? (x16 + 8) / 16 : -((-x16 + 8) / 16);
}

int median3(int a, int b, int c) noexcept {
    // Upstream text, including its third swap of b and c.
    int t = 0;
    if (a > b) {
        t = a;
        a = b;
        b = t;
    }
    if (b > c) {
        t = b;
        b = c;
        c = t;
    }
    if (a > b) {
        t = b;
        b = c;
        c = t;
    }
    return b;
}

// _do_calc_rc_params for CM_RGB, BPC_8, not native 422/420.
bool calc_rc_params_rgb8(uint16_t drm_bpp, int slice_width, int slice_height, int minor_version,
                         RcParams& rc) noexcept {
    const int32_t n = drm_bpp;  // bpp * 16
    (void)minor_version;        // CM_RGB is never CM_444: no DSC 1.1 adjustment.
    rc = RcParams{};
    rc.rc_quant_incr_limit0 = 11;
    rc.rc_quant_incr_limit1 = 11;

    // initial_fullness_offset (CM_444/CM_RGB row).
    if (n >= 12 * 16) {
        rc.initial_fullness_offset = 2048;
    } else if (n <= 8 * 16) {
        rc.initial_fullness_offset = 6144;
    } else if (n > 8 * 16 && n <= 10 * 16) {
        rc.initial_fullness_offset = 6144 - (n - 8 * 16) * 16;     // dsc_roundf((bpp - 8) * 256)
    } else {
        rc.initial_fullness_offset = 5632 - (n - 10 * 16) * 112;   // dsc_roundf((bpp - 10) * 1792)
    }
    // (int)(0.09 * min(34, slice_height - 8)): 9 k / 100 truncated; no |k| <= 34
    // product lies within binary64 rounding of an integer.
    const int k = min_int(34, slice_height - 8);
    const int scaled = k >= 0 ? (9 * k) / 100 : -((9 * -k) / 100);
    // (int)(((3 * bpc + 2) * 3) - (3 * bpp_group)), exact in float.
    const int limit = (1248 - 3 * n) / 16;
    rc.first_line_bpg_offset = median3(0, 12 + scaled, limit);
    rc.second_line_bpg_offset = 0;

    // dsc_roundf(8192.0 / 2.0 / bpp / 1.0) = round(65536 / n), never a tie.
    rc.initial_xmit_delay = (131072 + n) / (2 * n);
    if (slice_width <= 0) {
        return false;
    }
    const int padding_pixels = (slice_width % 3) != 0 ?
                               (3 - (slice_width % 3)) * (rc.initial_xmit_delay / slice_width) : 0;
    // 3 * bpp_group >= ((initial_xmit_delay + 2) / 3) * 3
    if (3 * n >= 16 * (((rc.initial_xmit_delay + 2) / 3) * 3)) {
        if ((rc.initial_xmit_delay + padding_pixels) % 3 == 1) {
            rc.initial_xmit_delay++;
        }
    }
    rc.flatness_min_qp = 3;
    rc.flatness_max_qp = 12;
    rc.flatness_det_thresh = 2 << (8 - 8);

    // get_qp_set: index = (bpp - 6) * 2, truncated.
    if (n < 6 * 16) {
        return false;
    }
    const uint32_t index = static_cast<uint32_t>((n - 6 * 16) / 8);
    if (index >= qp_table_rows) {
        return false;
    }
    for (uint32_t i = 0; i < qp_set_size; ++i) {
        rc.qp_min[i] = qp_table_444_8bpc_min[index][i];
        rc.qp_max[i] = qp_table_444_8bpc_max[index][i];
    }

    // get_ofs_set, CM_444/CM_RGB rows.
    const int32_t d6 = n - 6 * 16;    // (bpp - 6) * 16
    const int32_t d12 = n - 12 * 16;  // (bpp - 12) * 16
    const bool le6 = n <= 6 * 16;
    const bool in8_12 = n >= 8 * 16 && n <= 12 * 16;
    const bool ge15 = n >= 15 * 16;
    const bool in6_8 = n > 6 * 16 && n < 8 * 16;
    const bool le12 = n <= 12 * 16;
    const bool ge8 = n >= 8 * 16;
    int* p = rc.ofs;
    p[0] = le6 ? 0 : (in8_12 ? 2 : (ge15 ? 10 : (in6_8 ? 0 + dsc_round_sixteenths(d6)
                                                         : 2 + dsc_round_product(d12, eight_thirds))));
    p[1] = le6 ? -2 : (in8_12 ? 0 : (ge15 ? 8 : (in6_8 ? -2 + dsc_round_sixteenths(d6)
                                                         : 0 + dsc_round_product(d12, eight_thirds))));
    p[2] = le6 ? -2 : (in8_12 ? 0 : (ge15 ? 6 : (in6_8 ? -2 + dsc_round_sixteenths(d6)
                                                         : 0 + dsc_round_sixteenths(2 * d12))));
    p[3] = le6 ? -4 : (in8_12 ? -2 : (ge15 ? 4 : (in6_8 ? -4 + dsc_round_sixteenths(d6)
                                                          : -2 + dsc_round_sixteenths(2 * d12))));
    p[4] = le6 ? -6 : (in8_12 ? -4 : (ge15 ? 2 : (in6_8 ? -6 + dsc_round_sixteenths(d6)
                                                          : -4 + dsc_round_sixteenths(2 * d12))));
    p[5] = le12 ? -6 : (ge15 ? 0 : -6 + dsc_round_sixteenths(2 * d12));
    p[6] = le12 ? -8 : (ge15 ? -2 : -8 + dsc_round_sixteenths(2 * d12));
    p[7] = le12 ? -8 : (ge15 ? -4 : -8 + dsc_round_product(d12, four_thirds));
    p[8] = le12 ? -8 : (ge15 ? -6 : -8 + dsc_round_product(d12, two_thirds));
    p[9] = le12 ? -10 : (ge15 ? -8 : -10 + dsc_round_product(d12, two_thirds));
    p[10] = -10;
    p[11] = le6 ? -12 : (ge8 ? -10 : -12 + dsc_round_sixteenths(d6));
    p[12] = -12;
    p[13] = -12;
    p[14] = -12;

    rc.rc_model_size = 8192;
    rc.rc_edge_factor = 6;
    rc.rc_tgt_offset_hi = 3;
    rc.rc_tgt_offset_lo = 3;
    static const int thresholds[qp_set_size - 1] = {896, 1792, 2688, 3584, 4480, 5376, 6272,
                                                    6720, 7168, 7616, 7744, 7872, 8000, 8064};
    for (uint32_t i = 0; i < qp_set_size - 1; ++i) {
        rc.rc_buf_thresh[i] = thresholds[i];
    }
    return true;
}

// ---- drm/display/drm_dsc_helper.c ----

uint64_t div_round_up(uint64_t numerator, uint64_t denominator) noexcept {
    return (numerator + denominator - 1) / denominator;
}

// drm_dsc_compute_rc_parameters (unsigned long is 64-bit on x86-64 Linux).
// mux_word_size is passed separately (copy_pps_fields does not carry it).
bool compute_rc_parameters(DscPps& cfg, uint8_t mux_word_size, uint16_t& rc_bits) noexcept {
    uint64_t groups_per_line = 0;
    uint64_t num_extra_mux_bits = 0;
    if (cfg.slice_width == 0 || cfg.slice_height == 0 || cfg.bits_per_pixel == 0 || mux_word_size == 0) {
        return false;
    }
    if (cfg.native_420 || cfg.native_422) {
        groups_per_line = div_round_up(cfg.slice_width / 2u, 3);
        cfg.slice_chunk_size = static_cast<uint16_t>(div_round_up(
            static_cast<uint64_t>(cfg.slice_width / 2u) * cfg.bits_per_pixel, 8 * 16));
    } else {
        groups_per_line = div_round_up(cfg.slice_width, 3);
        cfg.slice_chunk_size = static_cast<uint16_t>(div_round_up(
            static_cast<uint64_t>(cfg.slice_width) * cfg.bits_per_pixel, 8 * 16));
    }
    if (cfg.convert_rgb) {
        num_extra_mux_bits = 3u * (mux_word_size + (4u * cfg.bits_per_component + 4u) - 2u);
    } else if (cfg.native_422) {
        num_extra_mux_bits = 4u * mux_word_size + (4u * cfg.bits_per_component + 4u) +
                             3u * (4u * cfg.bits_per_component) - 2u;
    } else {
        num_extra_mux_bits = 3u * mux_word_size + (4u * cfg.bits_per_component + 4u) +
                             2u * (4u * cfg.bits_per_component) - 2u;
    }
    const uint64_t slice_bits = 8ull * cfg.slice_chunk_size * cfg.slice_height;
    while (num_extra_mux_bits > 0 && ((slice_bits - num_extra_mux_bits) % mux_word_size)) {
        num_extra_mux_bits--;
    }
    // groups_per_line < initial_scale_value - 8 (int compared as unsigned long).
    const int64_t scale_minus_8 = static_cast<int64_t>(cfg.initial_scale_value) - 8;
    if (groups_per_line < static_cast<uint64_t>(scale_minus_8)) {
        cfg.initial_scale_value = static_cast<uint8_t>(groups_per_line + 8);
    }
    if (cfg.initial_scale_value > 8) {
        cfg.scale_decrement_interval = static_cast<uint16_t>(groups_per_line / (cfg.initial_scale_value - 8u));
    } else {
        cfg.scale_decrement_interval = 4095;  // DSC_SCALE_DECREMENT_INTERVAL_MAX
    }
    cfg.final_offset = static_cast<uint16_t>(
        static_cast<uint64_t>(cfg.rc_model_size -
                              (static_cast<int>(cfg.initial_xmit_delay) * cfg.bits_per_pixel + 8) / 16) +
        num_extra_mux_bits);
    if (cfg.final_offset >= cfg.rc_model_size) {
        return false;  // -ERANGE
    }
    const uint64_t final_scale = static_cast<uint64_t>(
        (static_cast<int>(cfg.rc_model_size) * 8) / (static_cast<int>(cfg.rc_model_size) - cfg.final_offset));
    if (cfg.slice_height > 1) {
        cfg.nfl_bpg_offset = static_cast<uint16_t>(div_round_up(
            static_cast<uint64_t>(cfg.first_line_bpg_offset) << 11, static_cast<uint64_t>(cfg.slice_height - 1)));
    } else {
        cfg.nfl_bpg_offset = 0;
    }
    const uint64_t groups_total = groups_per_line * cfg.slice_height;
    cfg.slice_bpg_offset = static_cast<uint16_t>(div_round_up(
        (static_cast<uint64_t>(cfg.rc_model_size - cfg.initial_offset) + num_extra_mux_bits) << 11, groups_total));
    if (final_scale > 9) {
        const uint64_t denominator = static_cast<uint64_t>(cfg.nfl_bpg_offset + cfg.slice_bpg_offset) *
                                     (final_scale - 9);
        cfg.scale_increment_interval = static_cast<uint16_t>(
            denominator == 0 ? 0 : static_cast<uint64_t>(cfg.final_offset * (1 << 11)) / denominator);
    } else {
        cfg.scale_increment_interval = 0;
    }
    const uint64_t rbs_min = static_cast<uint64_t>(cfg.rc_model_size - cfg.initial_offset) +
                             div_round_up(static_cast<uint64_t>(cfg.initial_xmit_delay) * cfg.bits_per_pixel, 16) +
                             groups_per_line * cfg.first_line_bpg_offset;
    const uint64_t hrd_delay = div_round_up(rbs_min * 16, cfg.bits_per_pixel);
    rc_bits = static_cast<uint16_t>((hrd_delay * cfg.bits_per_pixel) / 16);
    cfg.initial_dec_delay = static_cast<uint16_t>(hrd_delay - cfg.initial_xmit_delay);
    return true;
}

// dscc_compute_dsc_parameters (rc_calc_dpi.c): copy_rc_to_cfg, mux word size,
// drm_dsc_compute_rc_parameters and the u3.28 bytes per pixel.
bool dscc_compute_dsc_parameters(const DscPps& pps, const RcParams& rc, DscPps& out, uint32_t& bytes_per_pixel,
                                 uint32_t& rc_buffer_model_size) noexcept {
    DscPps cfg = pps;
    if (rc.rc_model_size - rc.initial_fullness_offset <= 0) {
        return false;
    }
    cfg.initial_scale_value =
        static_cast<uint8_t>(8 * rc.rc_model_size / (rc.rc_model_size - rc.initial_fullness_offset));
    cfg.rc_quant_incr_limit0 = static_cast<uint8_t>(rc.rc_quant_incr_limit0);
    cfg.rc_quant_incr_limit1 = static_cast<uint8_t>(rc.rc_quant_incr_limit1);
    cfg.initial_offset = static_cast<uint16_t>(rc.initial_fullness_offset);
    cfg.initial_xmit_delay = static_cast<uint16_t>(rc.initial_xmit_delay);
    cfg.first_line_bpg_offset = static_cast<uint8_t>(rc.first_line_bpg_offset);
    cfg.second_line_bpg_offset = static_cast<uint8_t>(rc.second_line_bpg_offset);
    cfg.flatness_min_qp = static_cast<uint8_t>(rc.flatness_min_qp);
    cfg.flatness_max_qp = static_cast<uint8_t>(rc.flatness_max_qp);
    for (uint32_t i = 0; i < qp_set_size; ++i) {
        cfg.rc_range_params[i].range_min_qp = static_cast<uint8_t>(rc.qp_min[i]);
        cfg.rc_range_params[i].range_max_qp = static_cast<uint8_t>(rc.qp_max[i]);
        // Truncate 8-bit signed value to 6-bit signed value.
        cfg.rc_range_params[i].range_bpg_offset = static_cast<uint8_t>(0x3f & rc.ofs[i]);
    }
    cfg.rc_model_size = static_cast<uint16_t>(rc.rc_model_size);
    cfg.rc_edge_factor = static_cast<uint8_t>(rc.rc_edge_factor);
    cfg.rc_tgt_offset_high = static_cast<uint8_t>(rc.rc_tgt_offset_hi);
    cfg.rc_tgt_offset_low = static_cast<uint8_t>(rc.rc_tgt_offset_lo);
    for (uint32_t i = 0; i < qp_set_size - 1; ++i) {
        cfg.rc_buf_thresh[i] = static_cast<uint16_t>(rc.rc_buf_thresh[i]);
    }
    const uint8_t mux_word_size = cfg.bits_per_component <= 10 ? 48 : 64;
    uint16_t rc_bits = 0;
    if (!compute_rc_parameters(cfg, mux_word_size, rc_bits)) {
        return false;
    }
    bytes_per_pixel = static_cast<uint32_t>((static_cast<uint64_t>(cfg.slice_chunk_size) * 0x10000000ull +
                                             (cfg.slice_width - 1u)) / cfg.slice_width);
    rc_buffer_model_size = rc_bits;
    out = cfg;
    return true;
}

void put_be16(uint8_t* bytes, uint32_t value) noexcept {
    bytes[0] = static_cast<uint8_t>(value >> 8);
    bytes[1] = static_cast<uint8_t>(value);
}

} // namespace

// ---- public: dc_dsc.c ----

bool parse_dsc_dpcd(const uint8_t (&basic)[dsc_dpcd_caps_bytes], const uint8_t* branch,
                    DscSinkCaps& caps) noexcept {
    caps = DscSinkCaps{};
    caps.is_dsc_supported = (basic[0x00] & 0x01u) != 0;  // DP_DSC_DECOMPRESSION_IS_SUPPORTED
    if (!caps.is_dsc_supported) {
        return false;
    }
    caps.dsc_version = basic[0x01];
    int buff_block_size = 0;
    if (!dsc_buff_block_size_from_dpcd(basic[0x02] & 0x03, buff_block_size)) {
        return false;
    }
    caps.rc_buffer_size = (basic[0x03] + 1) * buff_block_size;
    caps.slice_caps1 = basic[0x04];
    if (!dsc_line_buff_depth_from_dpcd(basic[0x05], caps.lb_bit_depth)) {
        return false;
    }
    caps.is_block_pred_supported = (basic[0x06] & 0x01u) != 0;
    caps.edp_max_bits_per_pixel = basic[0x07] | (basic[0x08] << 8);
    caps.color_formats = basic[0x09];
    caps.color_depth = basic[0x0a];
    const int dpcd_throughput = basic[0x0b];
    const int granular_delta = (basic[0x02] >> 3) * 2;
    if (!dsc_throughput_from_dpcd(dpcd_throughput & 0x0f, caps.throughput_mode_0_mps)) {
        return false;
    }
    caps.throughput_mode_0_mps += granular_delta;
    if (!dsc_throughput_from_dpcd((dpcd_throughput & 0xf0) >> 4, caps.throughput_mode_1_mps)) {
        return false;
    }
    caps.max_slice_width = basic[0x0c] * 320;
    caps.slice_caps2 = basic[0x0d];
    if (!dsc_bpp_increment_div_from_dpcd(basic[0x0f], caps.bpp_increment_div)) {
        return false;
    }
    if (branch == nullptr) {
        return true;
    }
    const auto branch_mps = [](uint32_t raw) noexcept -> uint32_t {
        if (raw == 0) {
            return 0;
        }
        return raw == 1 ? 680u : raw * 50u + 600u;
    };
    caps.branch_overall_throughput_0_mps = branch_mps(branch[0]);
    caps.branch_overall_throughput_1_mps = branch_mps(branch[1]);
    caps.branch_max_line_width = branch[2] * 320u;
    caps.is_dp = true;
    return true;
}

void dsc_encoder_caps(uint32_t pixel_clock_100hz, DscEncoderCaps& caps) noexcept {
    // dsc2_get_enc_caps (DCN20_MAX_PIXEL_CLOCK_Mhz 1188, DCN20_MAX_DISPLAY_CLOCK_Mhz 1200).
    caps = DscEncoderCaps{};
    caps.dsc_version = 0x21;
    caps.slice_caps = dsc_slices_1 | dsc_slices_2 | dsc_slices_3 | dsc_slices_4;
    caps.lb_bit_depth = 13;
    caps.is_block_pred_supported = true;
    caps.color_formats = 0x01u | 0x02u | 0x04u | 0x10u;  // RGB, 444, simple 422, native 420
    caps.color_depth = 0x02u | 0x04u | 0x08u;            // 8, 10, 12 bpc
    caps.max_total_throughput_mps = 1200;
    if (pixel_clock_100hz >= 1188u * 10000u) {
        caps.slice_caps = static_cast<uint8_t>(caps.slice_caps & ~dsc_slices_1);
        caps.slice_caps |= dsc_slices_8;
        caps.max_total_throughput_mps = 1200 * 2;
    }
    if (pixel_clock_100hz > 1188u * 10000u * 2u) {
        caps.slice_caps |= dsc_slices_12 | dsc_slices_16;
        caps.max_total_throughput_mps = 1200 * 4;
    }
    caps.max_slice_width = 5184;
    caps.bpp_increment_div = 16;
}

bool dsc_compute_config(const DscSinkCaps& sink, uint32_t target_bandwidth_kbps, const DscTiming& timing,
                        DscConfig& config) noexcept {
    config = DscConfig{};
    if (timing.pixel_clock_100hz == 0 || timing.h_active == 0 || timing.v_active == 0 ||
        timing.h_total < timing.h_active || timing.v_total < timing.v_active || target_bandwidth_kbps > 0x7fffffffu) {
        return false;
    }
    DscEncoderCaps enc{};
    dsc_encoder_caps(timing.pixel_clock_100hz, enc);
    return setup_dsc_config(sink, enc, static_cast<int>(target_bandwidth_kbps), timing, config);
}

uint32_t dsc_stream_bandwidth_kbps(const DscTiming& timing, uint32_t bpp_x16, uint32_t num_slices_h,
                                   bool is_dp) noexcept {
    const uint32_t overhead_in_kbps = stream_overhead_kbps(timing, static_cast<int>(num_slices_h), is_dp);
    const Fixed bpp = fixpt_from_fraction(bpp_x16, 16);
    Fixed actual = fixpt_from_fraction(timing.pixel_clock_100hz, 10);
    actual = fixpt_mul(actual, bpp);
    actual = fixpt_add_int(actual, static_cast<int>(overhead_in_kbps));
    return static_cast<uint32_t>(fixpt_ceil(actual));
}

uint32_t uncompressed_bandwidth_kbps(const DscTiming& timing) noexcept {
    // dc_bandwidth_in_kbps_from_timing: pix_clk_100hz / 10 * 8 bpc * 3.
    uint32_t kbps = timing.pixel_clock_100hz / 10;
    kbps *= 8;
    kbps *= 3;
    return kbps;
}

uint32_t dp_link_bandwidth_kbps(uint8_t rate_code, uint8_t lanes, bool fec) noexcept {
    // LINK_RATE_REF_FREQ_IN_KHZ 27000, BITS_PER_DP_BYTE 10,
    // DATA_EFFICIENCY_8b_10b_x10000 8000, FEC_EFFICIENCY_x100 97.
    const uint32_t link_rate_per_lane_kbps = static_cast<uint32_t>(rate_code) * 27000u * 10u;
    uint32_t efficiency_x10000 = 8000;
    if (fec) {
        efficiency_x10000 /= 100;
        efficiency_x10000 *= 97;
    }
    return link_rate_per_lane_kbps * lanes / 10000u * efficiency_x10000;
}

// ---- public: dcn20_dsc.c ----

namespace {

// dsc_prepare_config after dsc_init_reg_values and the copied input: the rc
// parameters, dscc_compute_dsc_parameters and dsc_update_from_dsc_parameters.
// Split into frames of their own (one register image, the rc parameters and
// the computed PPS never share a kernel frame).
DCN32_DSC_NOINLINE bool update_from_parameters(DscRegisterValues& values, const RcParams& rc,
                                               uint32_t& bytes_per_pixel) noexcept {
    DscPps computed{};
    uint32_t rc_buffer_model_size = 0;
    if (!dscc_compute_dsc_parameters(values.pps, rc, computed, bytes_per_pixel, rc_buffer_model_size)) {
        return false;
    }
    values.pps = computed;
    for (uint32_t i = 0; i < dsc_buf_ranges - 1; ++i) {
        values.pps.rc_buf_thresh[i] = static_cast<uint16_t>(values.pps.rc_buf_thresh[i] >> 6);
    }
    values.rc_buffer_model_size = rc_buffer_model_size;
    return true;
}

DCN32_DSC_NOINLINE bool finish_prepare(DscRegisterValues& values, DscRegisterValues& registers,
                                       DscOptcConfig& optc) noexcept {
    RcParams rc{};
    if (!calc_rc_params_rgb8(values.pps.bits_per_pixel, values.pps.slice_width, values.pps.slice_height,
                             values.pps.dsc_version_minor, rc)) {
        return false;
    }
    uint32_t bytes_per_pixel = 0;
    if (!update_from_parameters(values, rc, bytes_per_pixel)) {
        return false;
    }
    registers = values;
    optc.bytes_per_pixel = bytes_per_pixel;
    optc.slice_width = values.pps.slice_width;
    optc.is_pixel_format_444 = true;
    return true;
}

} // namespace

bool dsc_prepare_config(uint32_t pic_width, uint32_t pic_height, bool is_odm, const DscConfig& config,
                        DscRegisterValues& registers, DscOptcConfig& optc) noexcept {
    // Validate input parameters (RGB, COLOR_DEPTH_888).
    if (!config.num_slices_v || !config.num_slices_h ||
        !(config.version_minor == 1 || config.version_minor == 2) || !pic_width || !pic_height ||
        !((config.version_minor == 1 && 8 <= config.linebuf_depth && config.linebuf_depth <= 13) ||
          (config.version_minor == 2 &&
           ((8 <= config.linebuf_depth && config.linebuf_depth <= 15) || config.linebuf_depth == 0))) ||
        !(96 <= config.bits_per_pixel && config.bits_per_pixel <= 0x3ff) || pic_width > 0xffffu ||
        pic_height > 0xffffu) {
        return false;
    }
    // dsc_init_reg_values.
    DscRegisterValues values{};
    values.pps.dsc_version_minor = 2;
    values.pps.dsc_version_major = 1;
    values.pps.line_buf_depth = 9;
    values.pps.bits_per_component = 8;
    values.pps.block_pred_enable = true;
    values.pps.initial_xmit_delay = 170;
    values.pps.initial_offset = 6144;
    values.pps.flatness_min_qp = 3;
    values.pps.flatness_max_qp = 12;
    values.pps.rc_model_size = 8192;
    values.pps.rc_edge_factor = 6;
    values.pps.rc_quant_incr_limit0 = 11;
    values.pps.rc_quant_incr_limit1 = 11;
    values.pps.rc_tgt_offset_low = 3;
    values.pps.rc_tgt_offset_high = 3;

    // Copy input config.
    values.pixel_format = 0;  // DSC_PIXFMT_RGB
    values.num_slices_h = config.num_slices_h;
    values.num_slices_v = config.num_slices_v;
    values.pps.dsc_version_minor = static_cast<uint8_t>(config.version_minor);
    values.pps.pic_width = static_cast<uint16_t>(pic_width);
    values.pps.pic_height = static_cast<uint16_t>(pic_height);
    values.pps.bits_per_component = 8;  // DSC_BPC_8
    values.pps.block_pred_enable = config.block_pred_enable;
    values.pps.line_buf_depth = static_cast<uint8_t>(config.linebuf_depth);
    values.alternate_ich_encoding_en = values.pps.dsc_version_minor == 1 ? 0 : 1;
    values.ich_reset_at_eol = (is_odm || values.num_slices_h > 1) ? 0xf : 0;
    values.pps.slice_width = static_cast<uint16_t>(pic_width / config.num_slices_h);
    values.pps.slice_height = static_cast<uint16_t>(pic_height / config.num_slices_v);
    if (values.pps.slice_height * config.num_slices_v != pic_height) {
        return false;
    }
    values.bpp_x32 = config.bits_per_pixel << 1;
    values.pps.bits_per_pixel = static_cast<uint16_t>(values.bpp_x32 >> 1);
    values.pps.convert_rgb = true;
    values.pps.native_422 = false;
    values.pps.native_420 = false;
    values.pps.simple_422 = false;

    return finish_prepare(values, registers, optc);
}

void dsc_pack_pps(const DscPps& pps, uint8_t (&payload)[dsc_pps_bytes]) noexcept {
    for (uint8_t& byte : payload) {
        byte = 0;
    }
    payload[0] = static_cast<uint8_t>(pps.dsc_version_minor | (pps.dsc_version_major << 4));
    payload[3] = static_cast<uint8_t>(pps.line_buf_depth | (pps.bits_per_component << 4));
    payload[4] = static_cast<uint8_t>(((pps.bits_per_pixel & 0x300u) >> 8) | (pps.vbr_enable ? 1u << 2 : 0u) |
                                      (pps.simple_422 ? 1u << 3 : 0u) | (pps.convert_rgb ? 1u << 4 : 0u) |
                                      (pps.block_pred_enable ? 1u << 5 : 0u));
    payload[5] = static_cast<uint8_t>(pps.bits_per_pixel & 0xffu);
    put_be16(&payload[6], pps.pic_height);
    put_be16(&payload[8], pps.pic_width);
    put_be16(&payload[10], pps.slice_height);
    put_be16(&payload[12], pps.slice_width);
    put_be16(&payload[14], pps.slice_chunk_size);
    payload[16] = static_cast<uint8_t>((pps.initial_xmit_delay & 0x300u) >> 8);
    payload[17] = static_cast<uint8_t>(pps.initial_xmit_delay & 0xffu);
    put_be16(&payload[18], pps.initial_dec_delay);
    payload[21] = pps.initial_scale_value;
    put_be16(&payload[22], pps.scale_increment_interval);
    payload[24] = static_cast<uint8_t>((pps.scale_decrement_interval & 0xf00u) >> 8);
    payload[25] = static_cast<uint8_t>(pps.scale_decrement_interval & 0xffu);
    payload[27] = pps.first_line_bpg_offset;
    put_be16(&payload[28], pps.nfl_bpg_offset);
    put_be16(&payload[30], pps.slice_bpg_offset);
    put_be16(&payload[32], pps.initial_offset);
    put_be16(&payload[34], pps.final_offset);
    payload[36] = pps.flatness_min_qp;
    payload[37] = pps.flatness_max_qp;
    put_be16(&payload[38], pps.rc_model_size);
    payload[40] = 6;   // DSC_RC_EDGE_FACTOR_CONST
    payload[41] = pps.rc_quant_incr_limit0;
    payload[42] = pps.rc_quant_incr_limit1;
    payload[43] = static_cast<uint8_t>(3u | (3u << 4));  // DSC_RC_TGT_OFFSET_LO/HI_CONST
    for (uint32_t i = 0; i < dsc_buf_ranges - 1; ++i) {
        payload[44 + i] = static_cast<uint8_t>(pps.rc_buf_thresh[i]);
    }
    for (uint32_t i = 0; i < dsc_buf_ranges; ++i) {
        const DscRcRange& range = pps.rc_range_params[i];
        put_be16(&payload[58 + 2 * i], static_cast<uint32_t>((range.range_min_qp << 11) |
                                                             (range.range_max_qp << 6) | range.range_bpg_offset));
    }
    payload[88] = static_cast<uint8_t>((pps.native_422 ? 1u : 0u) | (pps.native_420 ? 1u << 1 : 0u));
    payload[89] = pps.second_line_bpg_offset;
    put_be16(&payload[90], pps.nsl_bpg_offset);
    put_be16(&payload[92], pps.second_line_offset_adj);
}

// ---- DSC engine registers ----

namespace dcn32_dsc {

namespace {

bool valid_engine(uint32_t dsc) noexcept { return dsc < engine_count; }

} // namespace

bool read_state(Dcn32StreamIo& io, uint32_t dsc, bool& forward_enabled) noexcept {
    // dsc2_read_state: REG_GET per field (each a register read), then
    // REG_GET_2 of DSCRM_DSC_FORWARD_CONFIG (one read).
    forward_enabled = false;
    if (!valid_engine(dsc)) {
        return false;
    }
    uint32_t value = 0;
    if (!io.get(r::DSC_TOP_CONTROL::reg, dsc, r::DSC_TOP_CONTROL::DSC_CLOCK_EN, value) ||
        !io.get(r::DSCC_PPS_CONFIG3::reg, dsc, r::DSCC_PPS_CONFIG3::SLICE_WIDTH, value) ||
        !io.get(r::DSCC_PPS_CONFIG1::reg, dsc, r::DSCC_PPS_CONFIG1::BITS_PER_PIXEL, value) ||
        !io.get(r::DSCC_PPS_CONFIG3::reg, dsc, r::DSCC_PPS_CONFIG3::SLICE_HEIGHT, value) ||
        !io.get(r::DSCC_PPS_CONFIG1::reg, dsc, r::DSCC_PPS_CONFIG1::CHUNK_SIZE, value) ||
        !io.get(r::DSCC_PPS_CONFIG2::reg, dsc, r::DSCC_PPS_CONFIG2::PIC_WIDTH, value) ||
        !io.get(r::DSCC_PPS_CONFIG2::reg, dsc, r::DSCC_PPS_CONFIG2::PIC_HEIGHT, value) ||
        !io.get(r::DSCC_PPS_CONFIG7::reg, dsc, r::DSCC_PPS_CONFIG7::SLICE_BPG_OFFSET, value)) {
        return false;
    }
    uint32_t forward = 0;
    if (!io.read(r::DSCRM_DSC_FORWARD_CONFIG::reg, dsc, forward)) {
        return false;
    }
    forward_enabled = field_of(forward, r::DSCRM_DSC_FORWARD_CONFIG::DSCRM_DSC_FORWARD_EN) != 0;
    return true;
}

bool write_registers(Dcn32StreamIo& io, uint32_t dsc, const DscRegisterValues& v) noexcept {
    if (!valid_engine(dsc)) {
        return false;
    }
    const DscPps& p = v.pps;
    namespace C0 = r::DSCC_CONFIG0;
    namespace I = r::DSCC_INTERRUPT_CONTROL_STATUS;
    // bpp: bpp_x32 >> 1 for RGB.
    const uint32_t bpp = v.bpp_x32 >> 1;
    const DscRcRange* rr = p.rc_range_params;
    // One statement per register: each field list is a temporary that ends with
    // its statement (a single && chain kept every list alive: kernel frame).
    if (!io.set(r::DSC_DEBUG_CONTROL::reg, dsc, {{r::DSC_DEBUG_CONTROL::DSC_DBG_EN, 0}})) {
        return false;
    }
    if (!io.set(r::DSCCIF_CONFIG0::reg, dsc,
                {{r::DSCCIF_CONFIG0::INPUT_INTERFACE_UNDERFLOW_RECOVERY_EN, 0},
                 {r::DSCCIF_CONFIG0::INPUT_INTERFACE_UNDERFLOW_OCCURRED_INT_EN, 0},
                 {r::DSCCIF_CONFIG0::INPUT_INTERFACE_UNDERFLOW_OCCURRED_STATUS, 0},
                 {r::DSCCIF_CONFIG0::INPUT_PIXEL_FORMAT, v.pixel_format},
                 {r::DSCCIF_CONFIG0::BITS_PER_COMPONENT, p.bits_per_component}})) {
        return false;
    }
    if (!io.set(r::DSCCIF_CONFIG1::reg, dsc,
                {{r::DSCCIF_CONFIG1::PIC_WIDTH, p.pic_width}, {r::DSCCIF_CONFIG1::PIC_HEIGHT, p.pic_height}})) {
        return false;
    }
    // ICH_RESET_AT_END_OF_LINE is in the DCN 2.0 mask list: REG_SET_4.
    if (!io.set(C0::reg, dsc, {{C0::ICH_RESET_AT_END_OF_LINE, v.ich_reset_at_eol},
                               {C0::NUMBER_OF_SLICES_PER_LINE, v.num_slices_h - 1},
                               {C0::ALTERNATE_ICH_ENCODING_EN, v.alternate_ich_encoding_en},
                               {C0::NUMBER_OF_SLICES_IN_VERTICAL_DIRECTION, v.num_slices_v - 1}})) {
        return false;
    }
    if (!io.set(r::DSCC_CONFIG1::reg, dsc,
                {{r::DSCC_CONFIG1::DSCC_RATE_CONTROL_BUFFER_MODEL_SIZE, v.rc_buffer_model_size}})) {
        return false;
    }
    if (!io.set(I::reg, dsc, {{I::DSCC_RATE_CONTROL_BUFFER_MODEL0_OVERFLOW_OCCURRED_INT_EN, 0},
                              {I::DSCC_RATE_CONTROL_BUFFER_MODEL1_OVERFLOW_OCCURRED_INT_EN, 0},
                              {I::DSCC_RATE_CONTROL_BUFFER_MODEL2_OVERFLOW_OCCURRED_INT_EN, 0},
                              {I::DSCC_RATE_CONTROL_BUFFER_MODEL3_OVERFLOW_OCCURRED_INT_EN, 0}})) {
        return false;
    }
    if (!io.set(r::DSCC_PPS_CONFIG0::reg, dsc,
                {{r::DSCC_PPS_CONFIG0::DSC_VERSION_MINOR, p.dsc_version_minor},
                 {r::DSCC_PPS_CONFIG0::LINEBUF_DEPTH, p.line_buf_depth},
                 {r::DSCC_PPS_CONFIG0::BITS_PER_COMPONENT, p.bits_per_component}})) {
        return false;
    }
    if (!io.set(r::DSCC_PPS_CONFIG1::reg, dsc,
                {{r::DSCC_PPS_CONFIG1::BITS_PER_PIXEL, bpp}, {r::DSCC_PPS_CONFIG1::SIMPLE_422, 0},
                 {r::DSCC_PPS_CONFIG1::CONVERT_RGB, v.pixel_format == 0 ? 1u : 0u},
                 {r::DSCC_PPS_CONFIG1::BLOCK_PRED_ENABLE, p.block_pred_enable ? 1u : 0u},
                 {r::DSCC_PPS_CONFIG1::NATIVE_422, 0}, {r::DSCC_PPS_CONFIG1::NATIVE_420, 0},
                 {r::DSCC_PPS_CONFIG1::CHUNK_SIZE, p.slice_chunk_size}})) {
        return false;
    }
    if (!io.set(r::DSCC_PPS_CONFIG2::reg, dsc,
                {{r::DSCC_PPS_CONFIG2::PIC_WIDTH, p.pic_width}, {r::DSCC_PPS_CONFIG2::PIC_HEIGHT, p.pic_height}})) {
        return false;
    }
    if (!io.set(r::DSCC_PPS_CONFIG3::reg, dsc, {{r::DSCC_PPS_CONFIG3::SLICE_WIDTH, p.slice_width},
                                                {r::DSCC_PPS_CONFIG3::SLICE_HEIGHT, p.slice_height}})) {
        return false;
    }
    if (!io.set(r::DSCC_PPS_CONFIG4::reg, dsc, {{r::DSCC_PPS_CONFIG4::INITIAL_XMIT_DELAY, p.initial_xmit_delay}})) {
        return false;
    }
    if (!io.set(r::DSCC_PPS_CONFIG5::reg, dsc,
                {{r::DSCC_PPS_CONFIG5::INITIAL_SCALE_VALUE, p.initial_scale_value},
                 {r::DSCC_PPS_CONFIG5::SCALE_INCREMENT_INTERVAL, p.scale_increment_interval}})) {
        return false;
    }
    if (!io.set(r::DSCC_PPS_CONFIG6::reg, dsc,
                {{r::DSCC_PPS_CONFIG6::SCALE_DECREMENT_INTERVAL, p.scale_decrement_interval},
                 {r::DSCC_PPS_CONFIG6::FIRST_LINE_BPG_OFFSET, p.first_line_bpg_offset},
                 {r::DSCC_PPS_CONFIG6::SECOND_LINE_BPG_OFFSET, p.second_line_bpg_offset}})) {
        return false;
    }
    if (!io.set(r::DSCC_PPS_CONFIG7::reg, dsc, {{r::DSCC_PPS_CONFIG7::NFL_BPG_OFFSET, p.nfl_bpg_offset},
                                                {r::DSCC_PPS_CONFIG7::SLICE_BPG_OFFSET, p.slice_bpg_offset}})) {
        return false;
    }
    if (!io.set(r::DSCC_PPS_CONFIG8::reg, dsc,
                {{r::DSCC_PPS_CONFIG8::NSL_BPG_OFFSET, p.nsl_bpg_offset},
                 {r::DSCC_PPS_CONFIG8::SECOND_LINE_OFFSET_ADJ, p.second_line_offset_adj}})) {
        return false;
    }
    if (!io.set(r::DSCC_PPS_CONFIG9::reg, dsc, {{r::DSCC_PPS_CONFIG9::INITIAL_OFFSET, p.initial_offset},
                                                {r::DSCC_PPS_CONFIG9::FINAL_OFFSET, p.final_offset}})) {
        return false;
    }
    if (!io.set(r::DSCC_PPS_CONFIG10::reg, dsc, {{r::DSCC_PPS_CONFIG10::FLATNESS_MIN_QP, p.flatness_min_qp},
                                                 {r::DSCC_PPS_CONFIG10::FLATNESS_MAX_QP, p.flatness_max_qp},
                                                 {r::DSCC_PPS_CONFIG10::RC_MODEL_SIZE, p.rc_model_size}})) {
        return false;
    }
    if (!io.set(r::DSCC_PPS_CONFIG11::reg, dsc,
                {{r::DSCC_PPS_CONFIG11::RC_EDGE_FACTOR, p.rc_edge_factor},
                 {r::DSCC_PPS_CONFIG11::RC_QUANT_INCR_LIMIT0, p.rc_quant_incr_limit0},
                 {r::DSCC_PPS_CONFIG11::RC_QUANT_INCR_LIMIT1, p.rc_quant_incr_limit1},
                 {r::DSCC_PPS_CONFIG11::RC_TGT_OFFSET_LO, p.rc_tgt_offset_low},
                 {r::DSCC_PPS_CONFIG11::RC_TGT_OFFSET_HI, p.rc_tgt_offset_high}})) {
        return false;
    }
    if (!io.set(r::DSCC_PPS_CONFIG12::reg, dsc, {{r::DSCC_PPS_CONFIG12::RC_BUF_THRESH0, p.rc_buf_thresh[0]},
                                                 {r::DSCC_PPS_CONFIG12::RC_BUF_THRESH1, p.rc_buf_thresh[1]},
                                                 {r::DSCC_PPS_CONFIG12::RC_BUF_THRESH2, p.rc_buf_thresh[2]},
                                                 {r::DSCC_PPS_CONFIG12::RC_BUF_THRESH3, p.rc_buf_thresh[3]}})) {
        return false;
    }
    if (!io.set(r::DSCC_PPS_CONFIG13::reg, dsc, {{r::DSCC_PPS_CONFIG13::RC_BUF_THRESH4, p.rc_buf_thresh[4]},
                                                 {r::DSCC_PPS_CONFIG13::RC_BUF_THRESH5, p.rc_buf_thresh[5]},
                                                 {r::DSCC_PPS_CONFIG13::RC_BUF_THRESH6, p.rc_buf_thresh[6]},
                                                 {r::DSCC_PPS_CONFIG13::RC_BUF_THRESH7, p.rc_buf_thresh[7]}})) {
        return false;
    }
    if (!io.set(r::DSCC_PPS_CONFIG14::reg, dsc, {{r::DSCC_PPS_CONFIG14::RC_BUF_THRESH8, p.rc_buf_thresh[8]},
                                                 {r::DSCC_PPS_CONFIG14::RC_BUF_THRESH9, p.rc_buf_thresh[9]},
                                                 {r::DSCC_PPS_CONFIG14::RC_BUF_THRESH10, p.rc_buf_thresh[10]},
                                                 {r::DSCC_PPS_CONFIG14::RC_BUF_THRESH11, p.rc_buf_thresh[11]}})) {
        return false;
    }
    if (!io.set(r::DSCC_PPS_CONFIG15::reg, dsc, {{r::DSCC_PPS_CONFIG15::RC_BUF_THRESH12, p.rc_buf_thresh[12]},
                                                 {r::DSCC_PPS_CONFIG15::RC_BUF_THRESH13, p.rc_buf_thresh[13]},
                                                 {r::DSCC_PPS_CONFIG15::RANGE_MIN_QP0, rr[0].range_min_qp},
                                                 {r::DSCC_PPS_CONFIG15::RANGE_MAX_QP0, rr[0].range_max_qp},
                                                 {r::DSCC_PPS_CONFIG15::RANGE_BPG_OFFSET0, rr[0].range_bpg_offset}})) {
        return false;
    }
    if (!io.set(r::DSCC_PPS_CONFIG16::reg, dsc, {{r::DSCC_PPS_CONFIG16::RANGE_MIN_QP1, rr[1].range_min_qp},
                                                 {r::DSCC_PPS_CONFIG16::RANGE_MAX_QP1, rr[1].range_max_qp},
                                                 {r::DSCC_PPS_CONFIG16::RANGE_BPG_OFFSET1, rr[1].range_bpg_offset},
                                                 {r::DSCC_PPS_CONFIG16::RANGE_MIN_QP2, rr[2].range_min_qp},
                                                 {r::DSCC_PPS_CONFIG16::RANGE_MAX_QP2, rr[2].range_max_qp},
                                                 {r::DSCC_PPS_CONFIG16::RANGE_BPG_OFFSET2, rr[2].range_bpg_offset}})) {
        return false;
    }
    if (!io.set(r::DSCC_PPS_CONFIG17::reg, dsc, {{r::DSCC_PPS_CONFIG17::RANGE_MIN_QP3, rr[3].range_min_qp},
                                                 {r::DSCC_PPS_CONFIG17::RANGE_MAX_QP3, rr[3].range_max_qp},
                                                 {r::DSCC_PPS_CONFIG17::RANGE_BPG_OFFSET3, rr[3].range_bpg_offset},
                                                 {r::DSCC_PPS_CONFIG17::RANGE_MIN_QP4, rr[4].range_min_qp},
                                                 {r::DSCC_PPS_CONFIG17::RANGE_MAX_QP4, rr[4].range_max_qp},
                                                 {r::DSCC_PPS_CONFIG17::RANGE_BPG_OFFSET4, rr[4].range_bpg_offset}})) {
        return false;
    }
    if (!io.set(r::DSCC_PPS_CONFIG18::reg, dsc, {{r::DSCC_PPS_CONFIG18::RANGE_MIN_QP5, rr[5].range_min_qp},
                                                 {r::DSCC_PPS_CONFIG18::RANGE_MAX_QP5, rr[5].range_max_qp},
                                                 {r::DSCC_PPS_CONFIG18::RANGE_BPG_OFFSET5, rr[5].range_bpg_offset},
                                                 {r::DSCC_PPS_CONFIG18::RANGE_MIN_QP6, rr[6].range_min_qp},
                                                 {r::DSCC_PPS_CONFIG18::RANGE_MAX_QP6, rr[6].range_max_qp},
                                                 {r::DSCC_PPS_CONFIG18::RANGE_BPG_OFFSET6, rr[6].range_bpg_offset}})) {
        return false;
    }
    if (!io.set(r::DSCC_PPS_CONFIG19::reg, dsc, {{r::DSCC_PPS_CONFIG19::RANGE_MIN_QP7, rr[7].range_min_qp},
                                                 {r::DSCC_PPS_CONFIG19::RANGE_MAX_QP7, rr[7].range_max_qp},
                                                 {r::DSCC_PPS_CONFIG19::RANGE_BPG_OFFSET7, rr[7].range_bpg_offset},
                                                 {r::DSCC_PPS_CONFIG19::RANGE_MIN_QP8, rr[8].range_min_qp},
                                                 {r::DSCC_PPS_CONFIG19::RANGE_MAX_QP8, rr[8].range_max_qp},
                                                 {r::DSCC_PPS_CONFIG19::RANGE_BPG_OFFSET8, rr[8].range_bpg_offset}})) {
        return false;
    }
    if (!io.set(r::DSCC_PPS_CONFIG20::reg, dsc,
                {{r::DSCC_PPS_CONFIG20::RANGE_MIN_QP9, rr[9].range_min_qp},
                 {r::DSCC_PPS_CONFIG20::RANGE_MAX_QP9, rr[9].range_max_qp},
                 {r::DSCC_PPS_CONFIG20::RANGE_BPG_OFFSET9, rr[9].range_bpg_offset},
                 {r::DSCC_PPS_CONFIG20::RANGE_MIN_QP10, rr[10].range_min_qp},
                 {r::DSCC_PPS_CONFIG20::RANGE_MAX_QP10, rr[10].range_max_qp},
                 {r::DSCC_PPS_CONFIG20::RANGE_BPG_OFFSET10, rr[10].range_bpg_offset}})) {
        return false;
    }
    if (!io.set(r::DSCC_PPS_CONFIG21::reg, dsc,
                {{r::DSCC_PPS_CONFIG21::RANGE_MIN_QP11, rr[11].range_min_qp},
                 {r::DSCC_PPS_CONFIG21::RANGE_MAX_QP11, rr[11].range_max_qp},
                 {r::DSCC_PPS_CONFIG21::RANGE_BPG_OFFSET11, rr[11].range_bpg_offset},
                 {r::DSCC_PPS_CONFIG21::RANGE_MIN_QP12, rr[12].range_min_qp},
                 {r::DSCC_PPS_CONFIG21::RANGE_MAX_QP12, rr[12].range_max_qp},
                 {r::DSCC_PPS_CONFIG21::RANGE_BPG_OFFSET12, rr[12].range_bpg_offset}})) {
        return false;
    }
    if (!io.set(r::DSCC_PPS_CONFIG22::reg, dsc,
                {{r::DSCC_PPS_CONFIG22::RANGE_MIN_QP13, rr[13].range_min_qp},
                 {r::DSCC_PPS_CONFIG22::RANGE_MAX_QP13, rr[13].range_max_qp},
                 {r::DSCC_PPS_CONFIG22::RANGE_BPG_OFFSET13, rr[13].range_bpg_offset},
                 {r::DSCC_PPS_CONFIG22::RANGE_MIN_QP14, rr[14].range_min_qp},
                 {r::DSCC_PPS_CONFIG22::RANGE_MAX_QP14, rr[14].range_max_qp},
                 {r::DSCC_PPS_CONFIG22::RANGE_BPG_OFFSET14, rr[14].range_bpg_offset}})) {
        return false;
    }
    return true;
}

bool enable(Dcn32StreamIo& io, uint32_t dsc, uint32_t opp) noexcept {
    // dsc2_enable: the "already enabled" check only logs; both reads stay.
    if (!valid_engine(dsc)) {
        return false;
    }
    uint32_t clock_en = 0;
    uint32_t forward = 0;
    namespace F = r::DSCRM_DSC_FORWARD_CONFIG;
    return io.get(r::DSC_TOP_CONTROL::reg, dsc, r::DSC_TOP_CONTROL::DSC_CLOCK_EN, clock_en) &&
           io.read(F::reg, dsc, forward) &&
           io.update(r::DSC_TOP_CONTROL::reg, dsc, {{r::DSC_TOP_CONTROL::DSC_CLOCK_EN, 1}}) &&
           io.update(F::reg, dsc, {{F::DSCRM_DSC_FORWARD_EN, 1}, {F::DSCRM_DSC_OPP_PIPE_SOURCE, opp}});
}

bool disable(Dcn32StreamIo& io, uint32_t dsc) noexcept {
    if (!valid_engine(dsc)) {
        return false;
    }
    uint32_t clock_en = 0;
    namespace F = r::DSCRM_DSC_FORWARD_CONFIG;
    return io.get(r::DSC_TOP_CONTROL::reg, dsc, r::DSC_TOP_CONTROL::DSC_CLOCK_EN, clock_en) &&
           io.update(F::reg, dsc, {{F::DSCRM_DSC_FORWARD_EN, 0}}) &&
           io.update(r::DSC_TOP_CONTROL::reg, dsc, {{r::DSC_TOP_CONTROL::DSC_CLOCK_EN, 0}});
}

bool disconnect(Dcn32StreamIo& io, uint32_t dsc) noexcept {
    return valid_engine(dsc) &&
           io.update(r::DSCRM_DSC_FORWARD_CONFIG::reg, dsc, {{r::DSCRM_DSC_FORWARD_CONFIG::DSCRM_DSC_FORWARD_EN, 0}});
}

bool wait_disconnect_pending_clear(Dcn32StreamIo& io, uint32_t dsc) noexcept {
    uint32_t value = 0;
    return valid_engine(dsc) && io.read(r::DSCRM_DSC_FORWARD_CONFIG::reg, dsc, value);
}

} // namespace dcn32_dsc

} // namespace rx7900::display

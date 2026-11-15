/*
Module: rx7900.display.dcn32_stream
Purpose: Linux-ordered DCN 3.2 stream and plane commit, disable, DPMS, readback and recovery.
Created: 2026-10-24
Architecture: Every block function is the instance of the Linux v6.12
function named in its comment for the pipes of the plan's topology (one pipe,
ODM slices, a second plane), in Linux statement order, including the
reads Linux performs (REG_GET/REG_READ) so a register trace compares access
for access. Link, PHY, PLL/DTO and display clocks are Dcn32StreamHooks calls at
the place Linux calls program_pix_clk, enable_link, disable_link,
cs_power_down, update_clocks, dc_dmub_srv_p_state_delegate and the
link_set_dpms_off write_scdc_data. Adapted from AMD DC dcn10/20/30/31/32 hwseq,
dcn31 dccg (dccg31_init),
optc, opp, mpc, dpp, hubp and hubbub sources (Copyright 2012-2023 Advanced
Micro Devices, Inc., MIT notice in licenses/AMDGPU-DISPLAY.txt).
*/
#include "dcn32_stream.hpp"

namespace rx7900::display {

namespace r = dcn32_stream_reg;

// Keeps helpers with a register-image local out of their callers' frames.
#if defined(_MSC_VER)
#define DCN32_STREAM_NOINLINE __declspec(noinline)
#else
#define DCN32_STREAM_NOINLINE __attribute__((noinline))
#endif

namespace {

constexpr uint32_t mpcc_none = 0xf;
constexpr uint32_t det_segment_kb = 64;        // DCN32_CRB_SEGMENT_SIZE_KB
constexpr uint32_t pixel_chunk_bytes = 8 * 1024; // dcn3_2_ip.pixel_chunk_size_kbytes
constexpr uint32_t hdr_mult_one = 0x1f000;     // dcn10_set_hdr_multiplier default
constexpr uint32_t gain_one = 0x1f000;         // dcn20_update_mpcc top/bottom gains
constexpr uint32_t dp_downspread_ctrl = 0x107; // DP_DOWNSPREAD_CTRL
constexpr uint32_t mall_mblk_bytes = 64 * 1024; // DCN3_2_MALL_MBLK_SIZE_BYTES (cursor only)
// The bandwidth state of a context without streams (static: too large for a frame).
constexpr Dcn32BandwidthPlan no_bandwidth{};

// PIXEL_RATE_DIV_BY_1/2/4 (dccg.h enum pixel_rate_div).
constexpr uint32_t pixel_rate_div_by_1 = 0;
constexpr uint32_t pixel_rate_div_by_2 = 1;
constexpr uint32_t pixel_rate_div_by_4 = 3;

// TEST_PATTERN_MODE_HORIZONTALBARS, MPCC_BLEND_MODE_TOP_LAYER_PASSTHROUGH /
// TOP_LAYER_ONLY / TOP_BOT_BLENDING, MPCC_ALPHA_BLEND_MODE_PER_PIXEL_ALPHA /
// PER_PIXEL_ALPHA_COMBINED_GLOBAL_GAIN / GLOBAL_ALPHA, MPC_OUTPUT_CSC_COEF_A,
// EXPANSION_MODE_ZERO, OPTC_DSC_ENABLED_444.
constexpr uint32_t dpg_mode_horizontal_bars = 4;
constexpr uint32_t mpcc_mode_top_layer_passthrough = 1;
constexpr uint32_t mpcc_mode_top_layer_only = 2;
constexpr uint32_t mpcc_mode_top_bot_blending = 3;
constexpr uint32_t mpcc_alpha_per_pixel = 0;
constexpr uint32_t mpcc_alpha_per_pixel_global_gain = 1;
constexpr uint32_t mpcc_alpha_global = 2;
constexpr uint32_t optc_dsc_enabled_444 = 1;
constexpr uint32_t ocsc_coef_a = 1;
constexpr uint32_t expansion_mode_zero = 1;

uint32_t log_2(uint32_t value) noexcept {
    uint32_t result = 0;
    while (value > 1) {
        value >>= 1;
        ++result;
    }
    return result;
}

uint32_t ceil_segments(uint32_t kb) noexcept { return (kb + det_segment_kb - 1) / det_segment_kb; }

// convert_and_clamp (dcn32_hubbub.c).
uint32_t convert_and_clamp(uint32_t wm_ns, uint32_t refclk_mhz, uint32_t clamp) noexcept {
    uint32_t value = wm_ns * refclk_mhz;
    value /= 1000;
    return value > clamp ? clamp : value;
}

// apply_front_porch_workaround (progressive).
uint32_t patched_v_front_porch(const Timing& timing) noexcept {
    return timing.v_front_porch < 1 ? 1 : timing.v_front_porch;
}

// dcn10_get_vupdate_offset_from_vsync.
int32_t vupdate_offset_from_vsync(const Dcn32StreamPlan& plan) noexcept {
    const Timing& t = plan.timing;
    const int32_t vesa_sync_start = static_cast<int32_t>(t.height + patched_v_front_porch(t));
    const int32_t asic_blank_end = static_cast<int32_t>(t.v_total) - vesa_sync_start;
    return asic_blank_end - static_cast<int32_t>(plan.bandwidth.vstartup_start) + 1;
}

bool is_dp(StreamSignal signal) noexcept { return signal == StreamSignal::display_port; }
bool is_hdmi(StreamSignal signal) noexcept { return signal == StreamSignal::hdmi; }

uint32_t frame_microseconds(const Timing& timing) noexcept {
    if (timing.pixel_clock_khz == 0) {
        return 0;
    }
    const uint64_t pixels = static_cast<uint64_t>(timing.h_total) * timing.v_total;
    return static_cast<uint32_t>((pixels * 1000u + timing.pixel_clock_khz - 1) / timing.pixel_clock_khz);
}

// The DCHUBBUB DETn_CTRL registers are distinct registers with distinct field names.
struct DetRegister final {
    const r::Register* reg;
    r::Field size;
    r::Field current;
};

constexpr DetRegister det_registers[dcn32_stream_pipes] = {
    {&r::DCHUBBUB_DET0_CTRL::reg, r::DCHUBBUB_DET0_CTRL::DET0_SIZE, r::DCHUBBUB_DET0_CTRL::DET0_SIZE_CURRENT},
    {&r::DCHUBBUB_DET1_CTRL::reg, r::DCHUBBUB_DET1_CTRL::DET1_SIZE, r::DCHUBBUB_DET1_CTRL::DET1_SIZE_CURRENT},
    {&r::DCHUBBUB_DET2_CTRL::reg, r::DCHUBBUB_DET2_CTRL::DET2_SIZE, r::DCHUBBUB_DET2_CTRL::DET2_SIZE_CURRENT},
    {&r::DCHUBBUB_DET3_CTRL::reg, r::DCHUBBUB_DET3_CTRL::DET3_SIZE, r::DCHUBBUB_DET3_CTRL::DET3_SIZE_CURRENT},
};

} // namespace

namespace {

// The DML output of one pipe of a plan (DML pipes are in pipe index order).
const Dcn32PipeBandwidth* find_pipe_bandwidth(const Dcn32BandwidthPlan& bandwidth, uint32_t pipe) noexcept {
    for (uint32_t i = 0; i < bandwidth.pipe_count && i < dcn32_stream_pipes; ++i) {
        if (bandwidth.pipes[i].pipe == pipe) {
            return &bandwidth.pipes[i];
        }
    }
    return nullptr;
}

const Dcn32PipeBandwidth& pipe_bandwidth(const Dcn32StreamPlan& plan, uint32_t pipe) noexcept {
    static const Dcn32PipeBandwidth none{};
    const Dcn32PipeBandwidth* found = find_pipe_bandwidth(plan.bandwidth, pipe);
    return found != nullptr ? *found : none;
}

const Dcn32TopologyPipe& topo(const Dcn32StreamPlan& plan, uint32_t pipe) noexcept {
    return plan.bandwidth.topology.pipes[pipe];
}

const Dcn32PlaneScanout& plane_of(const Dcn32StreamPlan& plan, uint32_t pipe) noexcept {
    const uint32_t plane = topo(plan, pipe).plane;
    return plan.planes[plane < dcn32_max_planes ? plane : 0];
}

// resource_get_odm_slice_src_rect (RGB: no left edge extra pixel) of an OPP head.
void odm_slice(const Dcn32Topology& t, uint32_t h_active, uint32_t opp_head, uint32_t& x, uint32_t& width) noexcept {
    const uint32_t count = topology_odm_slice_count(t);
    const uint32_t slice_width = count != 0 ? h_active / count : h_active;
    x = slice_width * topology_odm_slice_index(t, opp_head);
    width = t.pipes[opp_head].next_odm == dcn32_no_pipe && count != 0 ? h_active - slice_width * (count - 1)
                                                                       : slice_width;
}

// memcmp of two register images (byte access, any layout).
template <typename T>
bool same_bytes(const T& a, const T& b) noexcept {
    const unsigned char* pa = reinterpret_cast<const unsigned char*>(&a);
    const unsigned char* pb = reinterpret_cast<const unsigned char*>(&b);
    for (size_t i = 0; i < sizeof(T); ++i) {
        if (pa[i] != pb[i]) {
            return false;
        }
    }
    return true;
}

bool same_dlg(const Dcn32DlgRegs& a, const Dcn32DlgRegs& b) noexcept { return same_bytes(a, b); }
bool same_ttu(const Dcn32TtuRegs& a, const Dcn32TtuRegs& b) noexcept { return same_bytes(a, b); }
bool same_rq(const Dcn32RqRegs& a, const Dcn32RqRegs& b) noexcept { return same_bytes(a, b); }

// Number of DPP pipes (planes) of a context.
uint32_t hubp_count(const Dcn32Topology& t) noexcept {
    uint32_t count = 0;
    for (const Dcn32TopologyPipe& pipe : t.pipes) {
        count += pipe.has_plane ? 1u : 0u;
    }
    return count;
}

// dc_state_rem_all_planes_for_stream -> resource_remove_dpp_pipes_for_plane_composition
// for every plane: OPP heads keep their stream without a plane, secondary DPP
// pipes become free.
void remove_all_planes(Dcn32Topology& t) noexcept {
    for (uint32_t i = dcn32_stream_pipes; i-- > 0;) {
        Dcn32TopologyPipe& pipe = t.pipes[i];
        if (!pipe.has_plane) {
            continue;
        }
        if (pipe.top != dcn32_no_pipe) {
            t.pipes[pipe.top].bottom = pipe.bottom;
        }
        if (pipe.bottom != dcn32_no_pipe && pipe.top != dcn32_no_pipe) {
            t.pipes[pipe.bottom].top = pipe.top;
        }
        if (pipe.top == dcn32_no_pipe) {
            pipe.has_plane = false;
            pipe.plane = 0;
        } else {
            pipe = Dcn32TopologyPipe{};
        }
    }
    t.plane_count = 0;
}

} // namespace

bool dcn32_dp_dig_pixel_rate_div_policy(const Timing& timing) noexcept {
    // is_h_timing_divisible_by_2 (uint16_t arithmetic as in dc_resource.c).
    const uint16_t h_blank_start = static_cast<uint16_t>(timing.h_total - timing.h_front_porch);
    const uint16_t h_blank_end = static_cast<uint16_t>(h_blank_start - timing.width);
    return (timing.h_total % 2 == 0) && (h_blank_start % 2 == 0) && (h_blank_end % 2 == 0) &&
           (timing.h_sync_width % 2 == 0);
}

uint32_t dcn32_mall_ways_from_bytes(uint32_t total_bytes, uint32_t max_cab_allocation_bytes) noexcept {
    if (total_bytes == 0) {
        return 0;
    }
    const uint32_t cache_lines_used = total_bytes / dcn32_mall_cache_line_bytes + 2;
    const uint32_t total_cache_lines = max_cab_allocation_bytes / dcn32_mall_cache_line_bytes;
    const uint32_t lines_per_way = total_cache_lines / dcn32_mall_cache_ways;
    if (lines_per_way == 0) {
        return UINT32_MAX;
    }
    uint32_t ways = cache_lines_used / lines_per_way;
    if (cache_lines_used % lines_per_way > 0) {
        ++ways;
    }
    return ways;
}

namespace {

// Linear 32 bpp scanout: pitch in pixels, 256-byte multiple; 256-byte surface base.
bool usable_surface(uint64_t address, uint32_t pitch_pixels, uint32_t width) noexcept {
    return pitch_pixels >= width && pitch_pixels <= 0x3fff && (pitch_pixels % 64u) == 0 && address != 0 &&
           (address & 0xffu) == 0 && (address >> 48) == 0;
}

// The DSC configurations a plan needs, validated before the plan is written
// (one scratch register image, kept out of the builder's frame).
DCN32_STREAM_NOINLINE bool dsc_plan_valid(const Timing& timing, uint32_t odm, const DscConfig& config) noexcept {
    DscRegisterValues scratch{};
    DscOptcConfig optc{};
    DscConfig per_engine = config;
    per_engine.num_slices_h = config.num_slices_h / odm;
    return dsc_prepare_config(timing.width / odm, timing.height, odm > 1, per_engine, scratch, optc) &&
           dsc_prepare_config(timing.width, timing.height, odm > 1, config, scratch, optc);
}

} // namespace

StreamStatus build_dcn32_stream_plan(const Dcn32StreamRequest& request, const Dcn32BandwidthPlan& bandwidth,
                                     Dcn32StreamPlan& output) noexcept {
    const Timing& timing = request.timing;
    OptcTiming optc{};
    if (request.pipe >= dcn32_stream_pipes || request.engine >= dcn32_encoder::engine_count ||
        request.dchub_refclk_khz < 1000 || request.max_cab_allocation_bytes == 0 ||
        request.plane_count > dcn32_max_planes) {
        return StreamStatus::invalid_argument;
    }
    if (prepare_optc_timing(timing, optc) != Status::success || bandwidth.vstartup_start == 0 ||
        bandwidth.vstartup_start > timing.v_total) {
        return StreamStatus::unsupported;
    }
    // The planes (a plan without the dcn-dsc-odm fields has one full-screen plane).
    if (request.plane_count == 0) {
        if (!usable_surface(request.surface_address, request.pitch_pixels, timing.width)) {
            return StreamStatus::invalid_surface;
        }
    } else {
        for (uint32_t i = 0; i < request.plane_count; ++i) {
            const Dcn32PlaneScanout& plane = request.planes[i];
            const Dcn32PlaneGeometry& g = plane.geometry;
            if (!usable_surface(plane.surface_address, plane.pitch_pixels, g.surface_width) ||
                g.dst.x < 0 || g.dst.y < 0 || g.dst.width != static_cast<int32_t>(g.surface_width) ||
                g.dst.height != static_cast<int32_t>(g.surface_height) || g.surface_width == 0 ||
                g.surface_height == 0 || static_cast<uint32_t>(g.dst.x) + g.surface_width > timing.width ||
                static_cast<uint32_t>(g.dst.y) + g.surface_height > timing.height ||
                plane.global_alpha_value > 0xff) {
                return StreamStatus::invalid_surface;
            }
        }
    }
    if (request.tiling.num_pipes == 0 || request.tiling.max_compressed_frags == 0 || request.tiling.num_pkrs == 0) {
        return StreamStatus::invalid_argument;
    }
    // Every refusal precedes the first write to output.
    if (is_dp(request.signal) ? (request.dp_rate_code == 0 || request.dp_lanes == 0)
                              : ((request.tmds.signal == TmdsSignal::hdmi) != is_hdmi(request.signal) ||
                                 request.tmds.tmds_khz == 0)) {
        return StreamStatus::invalid_argument;
    }
    // The validated pipe topology must be the request's (OTG master on pipe).
    const bool topology = bandwidth.pipe_count != 0;
    const uint32_t planes = request.plane_count == 0 ? 1u : request.plane_count;
    const uint32_t odm = topology ? bandwidth.odm_slices : 1u;
    if (topology && (bandwidth.topology.otg_master != request.pipe || bandwidth.topology.plane_count != planes ||
                     odm == 0 || odm > dcn32_stream_pipes)) {
        return StreamStatus::invalid_argument;
    }
    // DSC: DP only, FEC required, every ODM slice's engine configuration and
    // the PPS of the whole picture must pass dsc_prepare_config.
    const bool dsc = request.dsc.num_slices_h != 0;
    if (dsc && (!is_dp(request.signal) || !request.fec || request.dsc.num_slices_h % odm != 0 ||
                timing.width % odm != 0 || !topology ||
                !dsc_plan_valid(timing, odm, request.dsc))) {
        return StreamStatus::unsupported;
    }
    if (!dsc && request.fec) {
        return StreamStatus::unsupported;
    }

    // Field by field: a whole-plan temporary would not fit a kernel frame.
    Dcn32StreamPlan& plan = output;
    if (dsc) {
        // dsc2_get_packed_pps of the whole picture, then dsc_prepare_config of
        // one ODM slice's engine (pic_width / opp_cnt, slices / opp_cnt).
        DscOptcConfig whole_optc{};
        DscConfig per_engine = request.dsc;
        per_engine.num_slices_h = request.dsc.num_slices_h / odm;
        (void)dsc_prepare_config(timing.width, timing.height, odm > 1, request.dsc, plan.dsc_registers, whole_optc);
        dsc_pack_pps(plan.dsc_registers.pps, plan.dsc_pps);
        (void)dsc_prepare_config(timing.width / odm, timing.height, odm > 1, per_engine, plan.dsc_registers,
                                 plan.dsc_optc);
    }
    plan.timing = timing;
    plan.signal = request.signal;
    plan.pipe = request.pipe;
    plan.engine = request.engine;
    plan.tiling = request.tiling;
    plan.bandwidth = bandwidth;
    plan.dchub_refclk_mhz = request.dchub_refclk_khz / 1000u;
    plan.vblank_end = timing.v_total - timing.v_front_porch - timing.height;
    plan.requested_pixel_100hz = timing.pixel_clock_khz * 10u;
    plan.pixel_rate_k1 = pixel_rate_div_by_1;
    plan.pixel_rate_k2 = pixel_rate_div_by_4;
    plan.dio_pix_per_cycle = 1;
    plan.dp_rate_code = 0;
    plan.dp_lanes = 0;
    plan.dp_use_vsc = false;
    plan.dp_ignore_msa = false;
    plan.tmds_khz = 0;
    plan.lte_340_scramble = false;
    plan.avi = InfoPacket{};
    plan.vendor = InfoPacket{};
    plan.vsc = InfoPacket{};
    plan.dsc = dsc;
    plan.dsc_config = dsc ? request.dsc : DscConfig{};
    if (!dsc) {
        plan.dsc_registers = DscRegisterValues{};
        plan.dsc_optc = DscOptcConfig{};
        for (uint8_t& byte : plan.dsc_pps) {
            byte = 0;
        }
    }
    plan.fec = request.fec;

    // Planes in dc order and the pipe topology. A bandwidth plan without one
    // (pipe_count 0) describes the single pipe of the one-plane owner.
    plan.plane_count = planes;
    if (request.plane_count == 0) {
        plan.planes[0] = Dcn32PlaneScanout{};
        plan.planes[0].surface_address = request.surface_address;
        plan.planes[0].pitch_pixels = request.pitch_pixels;
        plan.planes[0].geometry.surface_width = timing.width;
        plan.planes[0].geometry.surface_height = timing.height;
        plan.planes[0].geometry.dst = Dcn32Rect{0, 0, static_cast<int32_t>(timing.width),
                                                static_cast<int32_t>(timing.height)};
        plan.planes[1] = Dcn32PlaneScanout{};
    } else {
        for (uint32_t i = 0; i < dcn32_max_planes; ++i) {
            plan.planes[i] = i < request.plane_count ? request.planes[i] : Dcn32PlaneScanout{};
        }
    }
    plan.surface_address = plan.planes[planes - 1].surface_address;
    plan.pitch_pixels = plan.planes[planes - 1].pitch_pixels;
    if (!topology) {
        Dcn32BandwidthPlan& bw = plan.bandwidth;
        topology_start_stream(bw.topology, request.pipe, false);
        (void)topology_append_plane(bw.topology, Dcn32Topology{}, 0);
        (void)topology_build_scaling(bw.topology, timing.width, timing.height, &plan.planes[0].geometry, 1);
        bw.pipe_count = 1;
        bw.odm_slices = 1;
        bw.pipes[0] = Dcn32PipeBandwidth{};
        bw.pipes[0].pipe = request.pipe;
        bw.pipes[0].vstartup_start = bw.vstartup_start;
        bw.pipes[0].vupdate_offset = bw.vupdate_offset;
        bw.pipes[0].vupdate_width = bw.vupdate_width;
        bw.pipes[0].vready_offset = bw.vready_offset;
        bw.pipes[0].pstate_keepout = bw.pstate_keepout;
        bw.pipes[0].det_buffer_size_kb = bw.det_buffer_size_kb;
        bw.pipes[0].dppclk_khz = bw.dpp_dppclk_khz;
        bw.pipes[0].surface_size_in_mall = bw.mall_ss_size_bytes;
        bw.pipes[0].dlg = bw.dlg;
        bw.pipes[0].ttu = bw.ttu;
        bw.pipes[0].rq = bw.rq;
    }
    plan.odm_slices = odm;
    plan.odm_slice_width = topology_odm_slice_width(plan.bandwidth.topology, timing.width, false);
    plan.odm_last_slice_width = topology_odm_slice_width(plan.bandwidth.topology, timing.width, true);

    // dcn20 get_pixel_clock_parameters and dcn32_calculate_dccg_k1_k2_values.
    if (is_dp(request.signal)) {
        const bool policy = dcn32_dp_dig_pixel_rate_div_policy(timing);
        if (odm == 4) {
            plan.requested_pixel_100hz /= 4;
        } else if (odm == 2 || policy) {
            plan.requested_pixel_100hz /= 2;
        }
        plan.pixel_rate_k2 = (odm == 2 || policy) ? pixel_rate_div_by_2 : pixel_rate_div_by_4;
        plan.dio_pix_per_cycle = (policy || odm > 1) ? 2 : 1;
        plan.dp_rate_code = request.dp_rate_code;
        plan.dp_lanes = request.dp_lanes;
        plan.dp_use_vsc = request.dp_use_vsc;
        build_dp_vsc(request.dp_use_vsc, plan.vsc);
    } else {
        const bool hdmi = request.tmds.signal == TmdsSignal::hdmi;
        plan.pixel_rate_k2 = pixel_rate_div_by_4;
        plan.dio_pix_per_cycle = 1;
        plan.tmds_khz = request.tmds.tmds_khz;
        plan.lte_340_scramble = request.tmds.lte_340_scramble;
        if (hdmi) {
            plan.avi = request.tmds.avi;
            plan.vendor = request.tmds.vsif;
        }
    }

    // dcn32_calculate_cab_allocation (no cursor beyond the 16 KiB DCN buffer) and
    // dcn32_update_mall_sel: MALL for static screen when it fits.
    const uint32_t ways = dcn32_mall_ways_from_bytes(plan.bandwidth.mall_ss_size_bytes,
                                                     request.max_cab_allocation_bytes);
    plan.mall_sel = ways <= dcn32_mall_cache_ways ? 2 : 0;
    (void)mall_mblk_bytes;
    return StreamStatus::success;
}

// ---- controller plumbing ----

bool Dcn32StreamController::bind(const hardware::RegisterIo& io, uint32_t segment1, uint32_t segment2,
                                 uint32_t segment3, const Dcn32StreamHooks& hooks) noexcept {
    if (hooks.update_clocks == nullptr || hooks.set_pixel_clock == nullptr ||
        hooks.power_down_pixel_clock == nullptr || hooks.enable_link == nullptr || hooks.disable_link == nullptr) {
        return false;
    }
    if (!io_.bind(io, segment1, segment2, segment3)) {
        return false;
    }
    hooks_ = hooks;
    return true;
}

void Dcn32StreamController::unbind() noexcept {
    io_.unbind();
    hooks_ = {};
}

bool Dcn32StreamController::adopt_firmware(const Dcn32FirmwarePipe& firmware) noexcept {
    if (!io_.bound() || current_stream_ || firmware.pipe >= dcn32_stream_pipes ||
        firmware.engine >= dcn32_encoder::engine_count) {
        return false;
    }
    current_.timing = firmware.timing;
    current_.signal = firmware.signal;
    current_.pipe = firmware.pipe;
    current_.engine = firmware.engine;
    current_.bandwidth.vstartup_start = 0;
    current_.surface_address = 0;
    current_.dsc = false;
    current_.fec = false;
    current_.plane_count = 1;
    current_.odm_slices = 1;
    // One pipe with one plane, as preos_scanout validated it.
    topology_start_stream(live_, firmware.pipe, false);
    live_.pipes[firmware.pipe].has_plane = true;
    live_.plane_count = 1;
    current_.bandwidth.topology = live_;
    current_.bandwidth.pipe_count = 1;
    current_.bandwidth.odm_slices = 1;
    current_.bandwidth.pipes[0] = Dcn32PipeBandwidth{};
    current_.bandwidth.pipes[0].pipe = firmware.pipe;
    current_stream_ = true;
    current_plane_ = true;
    current_firmware_ = true;
    dpms_off_ = false;
    mpc_tree_top_[firmware.pipe] = firmware.pipe;
    mpcc_bot_[firmware.pipe] = mpcc_none;
    mpcc_dpp_[firmware.pipe] = firmware.pipe;
    hubp_power_gated_[firmware.pipe] = false;
    takeover_pending_ = true;
    return true;
}

Dcn32StreamController::Step Dcn32StreamController::fail(StreamFailure failure) noexcept {
    failure_ = failure;
    return Step::failed;
}

Dcn32StreamController::Step Dcn32StreamController::link(LinkCall call, StreamFailure refused) noexcept {
    switch (call) {
    case LinkCall::success: return Step::ok;
    case LinkCall::refused: failure_ = refused; return Step::failed;
    default: failure_ = StreamFailure::hook_uncertain; return Step::uncertain;
    }
}

Dcn32StreamController::Step Dcn32StreamController::ignored_result(LinkCall call) noexcept {
    if (call == LinkCall::uncertain) {
        failure_ = StreamFailure::hook_uncertain;
        return Step::uncertain;
    }
    return Step::ok;
}

// Maps the IO latch after a refused register helper.
#define DCN32_IO(expression)                                                     \
    do {                                                                         \
        if (!(expression)) {                                                     \
            if (io_.timed_out() && !io_.faulted()) {                             \
                failure_ = StreamFailure::register_timeout;                      \
                return Step::failed;                                             \
            }                                                                    \
            failure_ = StreamFailure::register_fault;                            \
            return Step::uncertain;                                              \
        }                                                                        \
    } while (false)

#define DCN32_STEP(expression)                                                   \
    do {                                                                         \
        const Step step_result = (expression);                                   \
        if (step_result != Step::ok) {                                           \
            return step_result;                                                  \
        }                                                                        \
    } while (false)

// ---- HUBBUB (dcn32_hubbub.c, dcn10_hubbub.c, dcn30_hubbub.c) ----

namespace {

struct WatermarkWrite final {
    uint32_t Dcn32Watermarks::*member;
    const r::Register* reg;
    r::Field field;
    // 0: raw value; otherwise the convert_and_clamp limit.
    uint32_t clamp;
};

} // namespace

Dcn32StreamController::Step Dcn32StreamController::program_watermarks(const Dcn32BandwidthPlan& plan,
                                                                      bool safe_to_lower) noexcept {
    const uint32_t refclk = dchub_refclk_mhz_;
    using W = Dcn32Watermarks;
    // hubbub32_program_urgent/stutter/pstate/usr_watermarks, sets A..D each.
    static constexpr WatermarkWrite urgent[4][4] = {
        {{&W::urgent_ns, &r::DCHUBBUB_ARB_DATA_URGENCY_WATERMARK_A::reg,
          r::DCHUBBUB_ARB_DATA_URGENCY_WATERMARK_A::DCHUBBUB_ARB_DATA_URGENCY_WATERMARK_A, 0x3fff},
         {&W::frac_urg_bw_flip, &r::DCHUBBUB_ARB_FRAC_URG_BW_FLIP_A::reg,
          r::DCHUBBUB_ARB_FRAC_URG_BW_FLIP_A::DCHUBBUB_ARB_FRAC_URG_BW_FLIP_A, 0},
         {&W::frac_urg_bw_nom, &r::DCHUBBUB_ARB_FRAC_URG_BW_NOM_A::reg,
          r::DCHUBBUB_ARB_FRAC_URG_BW_NOM_A::DCHUBBUB_ARB_FRAC_URG_BW_NOM_A, 0},
         {&W::urgent_latency_ns, &r::DCHUBBUB_ARB_REFCYC_PER_TRIP_TO_MEMORY_A::reg,
          r::DCHUBBUB_ARB_REFCYC_PER_TRIP_TO_MEMORY_A::DCHUBBUB_ARB_REFCYC_PER_TRIP_TO_MEMORY_A, 0x3fff}},
        {{&W::urgent_ns, &r::DCHUBBUB_ARB_DATA_URGENCY_WATERMARK_B::reg,
          r::DCHUBBUB_ARB_DATA_URGENCY_WATERMARK_B::DCHUBBUB_ARB_DATA_URGENCY_WATERMARK_B, 0x3fff},
         {&W::frac_urg_bw_flip, &r::DCHUBBUB_ARB_FRAC_URG_BW_FLIP_B::reg,
          r::DCHUBBUB_ARB_FRAC_URG_BW_FLIP_B::DCHUBBUB_ARB_FRAC_URG_BW_FLIP_B, 0},
         {&W::frac_urg_bw_nom, &r::DCHUBBUB_ARB_FRAC_URG_BW_NOM_B::reg,
          r::DCHUBBUB_ARB_FRAC_URG_BW_NOM_B::DCHUBBUB_ARB_FRAC_URG_BW_NOM_B, 0},
         {&W::urgent_latency_ns, &r::DCHUBBUB_ARB_REFCYC_PER_TRIP_TO_MEMORY_B::reg,
          r::DCHUBBUB_ARB_REFCYC_PER_TRIP_TO_MEMORY_B::DCHUBBUB_ARB_REFCYC_PER_TRIP_TO_MEMORY_B, 0x3fff}},
        {{&W::urgent_ns, &r::DCHUBBUB_ARB_DATA_URGENCY_WATERMARK_C::reg,
          r::DCHUBBUB_ARB_DATA_URGENCY_WATERMARK_C::DCHUBBUB_ARB_DATA_URGENCY_WATERMARK_C, 0x3fff},
         {&W::frac_urg_bw_flip, &r::DCHUBBUB_ARB_FRAC_URG_BW_FLIP_C::reg,
          r::DCHUBBUB_ARB_FRAC_URG_BW_FLIP_C::DCHUBBUB_ARB_FRAC_URG_BW_FLIP_C, 0},
         {&W::frac_urg_bw_nom, &r::DCHUBBUB_ARB_FRAC_URG_BW_NOM_C::reg,
          r::DCHUBBUB_ARB_FRAC_URG_BW_NOM_C::DCHUBBUB_ARB_FRAC_URG_BW_NOM_C, 0},
         {&W::urgent_latency_ns, &r::DCHUBBUB_ARB_REFCYC_PER_TRIP_TO_MEMORY_C::reg,
          r::DCHUBBUB_ARB_REFCYC_PER_TRIP_TO_MEMORY_C::DCHUBBUB_ARB_REFCYC_PER_TRIP_TO_MEMORY_C, 0x3fff}},
        {{&W::urgent_ns, &r::DCHUBBUB_ARB_DATA_URGENCY_WATERMARK_D::reg,
          r::DCHUBBUB_ARB_DATA_URGENCY_WATERMARK_D::DCHUBBUB_ARB_DATA_URGENCY_WATERMARK_D, 0x3fff},
         {&W::frac_urg_bw_flip, &r::DCHUBBUB_ARB_FRAC_URG_BW_FLIP_D::reg,
          r::DCHUBBUB_ARB_FRAC_URG_BW_FLIP_D::DCHUBBUB_ARB_FRAC_URG_BW_FLIP_D, 0},
         {&W::frac_urg_bw_nom, &r::DCHUBBUB_ARB_FRAC_URG_BW_NOM_D::reg,
          r::DCHUBBUB_ARB_FRAC_URG_BW_NOM_D::DCHUBBUB_ARB_FRAC_URG_BW_NOM_D, 0},
         {&W::urgent_latency_ns, &r::DCHUBBUB_ARB_REFCYC_PER_TRIP_TO_MEMORY_D::reg,
          r::DCHUBBUB_ARB_REFCYC_PER_TRIP_TO_MEMORY_D::DCHUBBUB_ARB_REFCYC_PER_TRIP_TO_MEMORY_D, 0x3fff}}};
    static constexpr WatermarkWrite stutter[4][2] = {
        {{&W::cstate_enter_plus_exit_ns, &r::DCHUBBUB_ARB_ALLOW_SR_ENTER_WATERMARK_A::reg,
          r::DCHUBBUB_ARB_ALLOW_SR_ENTER_WATERMARK_A::DCHUBBUB_ARB_ALLOW_SR_ENTER_WATERMARK_A, 0xffff},
         {&W::cstate_exit_ns, &r::DCHUBBUB_ARB_ALLOW_SR_EXIT_WATERMARK_A::reg,
          r::DCHUBBUB_ARB_ALLOW_SR_EXIT_WATERMARK_A::DCHUBBUB_ARB_ALLOW_SR_EXIT_WATERMARK_A, 0xffff}},
        {{&W::cstate_enter_plus_exit_ns, &r::DCHUBBUB_ARB_ALLOW_SR_ENTER_WATERMARK_B::reg,
          r::DCHUBBUB_ARB_ALLOW_SR_ENTER_WATERMARK_B::DCHUBBUB_ARB_ALLOW_SR_ENTER_WATERMARK_B, 0xffff},
         {&W::cstate_exit_ns, &r::DCHUBBUB_ARB_ALLOW_SR_EXIT_WATERMARK_B::reg,
          r::DCHUBBUB_ARB_ALLOW_SR_EXIT_WATERMARK_B::DCHUBBUB_ARB_ALLOW_SR_EXIT_WATERMARK_B, 0xffff}},
        {{&W::cstate_enter_plus_exit_ns, &r::DCHUBBUB_ARB_ALLOW_SR_ENTER_WATERMARK_C::reg,
          r::DCHUBBUB_ARB_ALLOW_SR_ENTER_WATERMARK_C::DCHUBBUB_ARB_ALLOW_SR_ENTER_WATERMARK_C, 0xffff},
         {&W::cstate_exit_ns, &r::DCHUBBUB_ARB_ALLOW_SR_EXIT_WATERMARK_C::reg,
          r::DCHUBBUB_ARB_ALLOW_SR_EXIT_WATERMARK_C::DCHUBBUB_ARB_ALLOW_SR_EXIT_WATERMARK_C, 0xffff}},
        {{&W::cstate_enter_plus_exit_ns, &r::DCHUBBUB_ARB_ALLOW_SR_ENTER_WATERMARK_D::reg,
          r::DCHUBBUB_ARB_ALLOW_SR_ENTER_WATERMARK_D::DCHUBBUB_ARB_ALLOW_SR_ENTER_WATERMARK_D, 0xffff},
         {&W::cstate_exit_ns, &r::DCHUBBUB_ARB_ALLOW_SR_EXIT_WATERMARK_D::reg,
          r::DCHUBBUB_ARB_ALLOW_SR_EXIT_WATERMARK_D::DCHUBBUB_ARB_ALLOW_SR_EXIT_WATERMARK_D, 0xffff}}};
    static constexpr WatermarkWrite uclk[4] = {
        {&W::pstate_change_ns, &r::DCHUBBUB_ARB_UCLK_PSTATE_CHANGE_WATERMARK_A::reg,
         r::DCHUBBUB_ARB_UCLK_PSTATE_CHANGE_WATERMARK_A::DCHUBBUB_ARB_UCLK_PSTATE_CHANGE_WATERMARK_A, 0xffff},
        {&W::pstate_change_ns, &r::DCHUBBUB_ARB_UCLK_PSTATE_CHANGE_WATERMARK_B::reg,
         r::DCHUBBUB_ARB_UCLK_PSTATE_CHANGE_WATERMARK_B::DCHUBBUB_ARB_UCLK_PSTATE_CHANGE_WATERMARK_B, 0xffff},
        {&W::pstate_change_ns, &r::DCHUBBUB_ARB_UCLK_PSTATE_CHANGE_WATERMARK_C::reg,
         r::DCHUBBUB_ARB_UCLK_PSTATE_CHANGE_WATERMARK_C::DCHUBBUB_ARB_UCLK_PSTATE_CHANGE_WATERMARK_C, 0xffff},
        {&W::pstate_change_ns, &r::DCHUBBUB_ARB_UCLK_PSTATE_CHANGE_WATERMARK_D::reg,
         r::DCHUBBUB_ARB_UCLK_PSTATE_CHANGE_WATERMARK_D::DCHUBBUB_ARB_UCLK_PSTATE_CHANGE_WATERMARK_D, 0xffff}};
    static constexpr WatermarkWrite fclk[4] = {
        {&W::fclk_pstate_change_ns, &r::DCHUBBUB_ARB_FCLK_PSTATE_CHANGE_WATERMARK_A::reg,
         r::DCHUBBUB_ARB_FCLK_PSTATE_CHANGE_WATERMARK_A::DCHUBBUB_ARB_FCLK_PSTATE_CHANGE_WATERMARK_A, 0xffff},
        {&W::fclk_pstate_change_ns, &r::DCHUBBUB_ARB_FCLK_PSTATE_CHANGE_WATERMARK_B::reg,
         r::DCHUBBUB_ARB_FCLK_PSTATE_CHANGE_WATERMARK_B::DCHUBBUB_ARB_FCLK_PSTATE_CHANGE_WATERMARK_B, 0xffff},
        {&W::fclk_pstate_change_ns, &r::DCHUBBUB_ARB_FCLK_PSTATE_CHANGE_WATERMARK_C::reg,
         r::DCHUBBUB_ARB_FCLK_PSTATE_CHANGE_WATERMARK_C::DCHUBBUB_ARB_FCLK_PSTATE_CHANGE_WATERMARK_C, 0xffff},
        {&W::fclk_pstate_change_ns, &r::DCHUBBUB_ARB_FCLK_PSTATE_CHANGE_WATERMARK_D::reg,
         r::DCHUBBUB_ARB_FCLK_PSTATE_CHANGE_WATERMARK_D::DCHUBBUB_ARB_FCLK_PSTATE_CHANGE_WATERMARK_D, 0xffff}};
    static constexpr WatermarkWrite usr[4] = {
        {&W::usr_retraining_ns, &r::DCHUBBUB_ARB_USR_RETRAINING_WATERMARK_A::reg,
         r::DCHUBBUB_ARB_USR_RETRAINING_WATERMARK_A::DCHUBBUB_ARB_USR_RETRAINING_WATERMARK_A, 0x3fff},
        {&W::usr_retraining_ns, &r::DCHUBBUB_ARB_USR_RETRAINING_WATERMARK_B::reg,
         r::DCHUBBUB_ARB_USR_RETRAINING_WATERMARK_B::DCHUBBUB_ARB_USR_RETRAINING_WATERMARK_B, 0x3fff},
        {&W::usr_retraining_ns, &r::DCHUBBUB_ARB_USR_RETRAINING_WATERMARK_C::reg,
         r::DCHUBBUB_ARB_USR_RETRAINING_WATERMARK_C::DCHUBBUB_ARB_USR_RETRAINING_WATERMARK_C, 0x3fff},
        {&W::usr_retraining_ns, &r::DCHUBBUB_ARB_USR_RETRAINING_WATERMARK_D::reg,
         r::DCHUBBUB_ARB_USR_RETRAINING_WATERMARK_D::DCHUBBUB_ARB_USR_RETRAINING_WATERMARK_D, 0x3fff}};

    const Dcn32Watermarks* requested[4] = {&plan.a, &plan.b, &plan.c, &plan.d};
    Dcn32Watermarks* cached[4] = {&hubbub_.a, &hubbub_.b, &hubbub_.c, &hubbub_.d};

    auto program = [&](const WatermarkWrite& write, uint32_t set) noexcept -> bool {
        const uint32_t value = requested[set]->*write.member;
        uint32_t& cache = cached[set]->*write.member;
        if (!safe_to_lower && value <= cache) {
            return true;
        }
        cache = value;
        const uint32_t programmed = write.clamp != 0 ? convert_and_clamp(value, refclk, write.clamp) : value;
        return io_.set(*write.reg, 0, {{write.field, programmed}});
    };

    for (uint32_t set = 0; set < 4; ++set) {
        for (const WatermarkWrite& write : urgent[set]) {
            DCN32_IO(program(write, set));
        }
    }
    for (uint32_t set = 0; set < 4; ++set) {
        for (const WatermarkWrite& write : stutter[set]) {
            DCN32_IO(program(write, set));
        }
    }
    for (uint32_t set = 0; set < 4; ++set) {
        DCN32_IO(program(uclk[set], set));
    }
    for (uint32_t set = 0; set < 4; ++set) {
        DCN32_IO(program(fclk[set], set));
    }
    for (uint32_t set = 0; set < 4; ++set) {
        DCN32_IO(program(usr[set], set));
    }
    // hubbub1_allow_self_refresh_control(!disable_stutter), then
    // hubbub32_force_usr_retraining_allow(force_usr_allow = false).
    namespace D = r::DCHUBBUB_ARB_DRAM_STATE_CNTL;
    namespace U = r::DCHUBBUB_ARB_USR_RETRAINING_CNTL;
    DCN32_IO(io_.update(D::reg, 0, {{D::DCHUBBUB_ARB_ALLOW_SELF_REFRESH_FORCE_VALUE, 0},
                                    {D::DCHUBBUB_ARB_ALLOW_SELF_REFRESH_FORCE_ENABLE, 0}}));
    DCN32_IO(io_.update(U::reg, 0, {{U::DCHUBBUB_ARB_ALLOW_USR_RETRAINING_FORCE_VALUE, 0},
                                    {U::DCHUBBUB_ARB_ALLOW_USR_RETRAINING_FORCE_ENABLE, 0}}));
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::program_compbuf_size(uint32_t compbuf_kb,
                                                                        bool safe_to_increase) noexcept {
    // dcn32_program_compbuf_size.
    const uint32_t segments = ceil_segments(compbuf_kb);
    if (!safe_to_increase && segments > hubbub_.compbuf_size_segments) {
        return Step::ok;
    }
    if (segments > hubbub_.compbuf_size_segments) {
        for (uint32_t index = 0; index < dcn32_stream_pipes; ++index) {
            DCN32_IO(io_.wait(*det_registers[index].reg, 0, det_registers[index].current, hubbub_.det_size[index],
                              1, 100));
        }
    }
    namespace C = r::DCHUBBUB_COMPBUF_CTRL;
    DCN32_IO(io_.update(C::reg, 0, {{C::COMPBUF_SIZE, segments}}));
    hubbub_.compbuf_size_segments = segments;
    uint32_t config_error = 0;
    DCN32_IO(io_.get(C::reg, 0, C::CONFIG_ERROR, config_error));
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::program_det_size(uint32_t hubp, uint32_t det_kb) noexcept {
    // dcn32_program_det_size.
    const uint32_t segments = ceil_segments(det_kb);
    DCN32_IO(io_.update(*det_registers[hubp].reg, 0, {{det_registers[hubp].size, segments}}));
    hubbub_.det_size[hubp] = segments;
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::force_pstate_change_control(bool force, bool allow) noexcept {
    // hubbub3_force_pstate_change_control.
    namespace D = r::DCHUBBUB_ARB_DRAM_STATE_CNTL;
    DCN32_IO(io_.update(D::reg, 0, {{D::DCHUBBUB_ARB_ALLOW_PSTATE_CHANGE_FORCE_VALUE, allow ? 1u : 0u},
                                    {D::DCHUBBUB_ARB_ALLOW_PSTATE_CHANGE_FORCE_ENABLE, force ? 1u : 0u}}));
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::update_force_pstate(const Dcn32Topology& context) noexcept {
    // dcn32_update_force_pstate: no SubVP/FPO, so every pipe with a HUBP (a
    // pipe of the context with a stream) is unforced; the second loop
    // (SubVP main / FPO pipes) never applies.
    namespace P = r::UCLK_PSTATE_FORCE;
    for (uint32_t pipe = 0; pipe < dcn32_stream_pipes; ++pipe) {
        if (!context.pipes[pipe].stream) {
            continue;
        }
        DCN32_IO(io_.update(P::reg, pipe, {{P::DATA_UCLK_PSTATE_FORCE_EN, 0}, {P::DATA_UCLK_PSTATE_FORCE_VALUE, 0}}));
        DCN32_IO(io_.update(P::reg, pipe,
                            {{P::CURSOR_UCLK_PSTATE_FORCE_EN, 0}, {P::CURSOR_UCLK_PSTATE_FORCE_VALUE, 0}}));
    }
    return Step::ok;
}

// ---- power gating (dcn32_hwseq.c) ----

Dcn32StreamController::Step Dcn32StreamController::dsc_pg_control(uint32_t dsc, bool power_on) noexcept {
    namespace I = r::DC_IP_REQUEST_CNTL;
    uint32_t original = 0;
    DCN32_IO(io_.get(I::reg, 0, I::IP_REQUEST_EN, original));
    if (original == 0) {
        DCN32_IO(io_.set(I::reg, 0, {{I::IP_REQUEST_EN, 1}}));
    }
    // DOMAIN16..19 are catalog instances 4..7.
    DCN32_IO(io_.update(r::DOMAIN_PG_CONFIG::reg, 4 + dsc, {{r::DOMAIN_PG_CONFIG::DOMAIN_POWER_GATE, power_on ? 0u : 1u}}));
    DCN32_IO(io_.wait(r::DOMAIN_PG_STATUS::reg, 4 + dsc, r::DOMAIN_PG_STATUS::DOMAIN_PGFSM_PWR_STATUS,
                      power_on ? 0u : 2u, 1, 1000));
    if (original == 0) {
        DCN32_IO(io_.set(I::reg, 0, {{I::IP_REQUEST_EN, 0}}));
    }
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::dsc_pg_status(uint32_t dsc, bool& ungated) noexcept {
    // dcn32_dsc_pg_status (DOMAIN16..19 are catalog instances 4..7).
    uint32_t status = 0;
    DCN32_IO(io_.get(r::DOMAIN_PG_STATUS::reg, 4 + dsc, r::DOMAIN_PG_STATUS::DOMAIN_PGFSM_PWR_STATUS, status));
    ungated = status == 0;
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::update_dsc_pg(const Dcn32StreamPlan* target,
                                                                 bool safe_to_disable) noexcept {
    // dcn32_update_dsc_pg: res_ctx.is_dsc_acquired of the context (one DSC per
    // OPP head of a DSC stream).
    for (uint32_t dsc = 0; dsc < dcn32_stream_dscs; ++dsc) {
        bool ungated = false;
        DCN32_STEP(dsc_pg_status(dsc, ungated));
        const bool acquired = target != nullptr && topo(*target, dsc).stream && topo(*target, dsc).dsc;
        if (acquired) {
            if (!ungated) {
                DCN32_STEP(dsc_pg_control(dsc, true));
            }
        } else if (safe_to_disable && ungated) {
            DCN32_STEP(dsc_pg_control(dsc, false));
        }
    }
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::hubp_pg_control(uint32_t hubp, bool power_on) noexcept {
    // dcn32_hubp_pg_control (REG_SET: the whole DOMAINn_PG_CONFIG).
    DCN32_IO(io_.set(r::DOMAIN_PG_CONFIG::reg, hubp, {{r::DOMAIN_PG_CONFIG::DOMAIN_POWER_GATE, power_on ? 0u : 1u}}));
    DCN32_IO(io_.wait(r::DOMAIN_PG_STATUS::reg, hubp, r::DOMAIN_PG_STATUS::DOMAIN_PGFSM_PWR_STATUS,
                      power_on ? 0u : 2u, 1, 1000));
    return Step::ok;
}

// ---- dcn32_init_hw subset owned here ----

Dcn32StreamController::Step Dcn32StreamController::dccg_init() noexcept {
    // dccg31_init: every HPO stream encoder SYMCLK32 back to refclk
    // (dccg31_disable_symclk32_se, REG_UPDATE_2 per instance). dcn32 sets no
    // root_clock_optimization bit, so the SE/LE root gating, DPSTREAMCLK and
    // PHYSYMCLK steps of dccg31_init do nothing.
    namespace S = r::SYMCLK32_SE_CNTL;
    DCN32_IO(io_.update(S::reg, 0, {{S::SYMCLK32_SE0_SRC_SEL, 0}, {S::SYMCLK32_SE0_EN, 0}}));
    DCN32_IO(io_.update(S::reg, 0, {{S::SYMCLK32_SE1_SRC_SEL, 0}, {S::SYMCLK32_SE1_EN, 0}}));
    DCN32_IO(io_.update(S::reg, 0, {{S::SYMCLK32_SE2_SRC_SEL, 0}, {S::SYMCLK32_SE2_EN, 0}}));
    DCN32_IO(io_.update(S::reg, 0, {{S::SYMCLK32_SE3_SRC_SEL, 0}, {S::SYMCLK32_SE3_EN, 0}}));
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::takeover() noexcept {
    // dcn32_init_hw: init_clocks belongs to the link owner (bind), then the dccg init.
    DCN32_STEP(dccg_init());

    // dcn32_enable_power_gating_plane(hws, true): FORCEON off for HUBP0..3 and DSC0..3.
    namespace I = r::DC_IP_REQUEST_CNTL;
    uint32_t original = 0;
    DCN32_IO(io_.get(I::reg, 0, I::IP_REQUEST_EN, original));
    if (original == 0) {
        DCN32_IO(io_.set(I::reg, 0, {{I::IP_REQUEST_EN, 1}}));
    }
    for (uint32_t domain = 0; domain < 8; ++domain) {
        DCN32_IO(io_.update(r::DOMAIN_PG_CONFIG::reg, domain, {{r::DOMAIN_PG_CONFIG::DOMAIN_POWER_FORCEON, 0}}));
    }
    if (original == 0) {
        DCN32_IO(io_.set(I::reg, 0, {{I::IP_REQUEST_EN, 0}}));
    }

    // hubbub1_allow_self_refresh_control(true) after init_pipes.
    namespace D = r::DCHUBBUB_ARB_DRAM_STATE_CNTL;
    DCN32_IO(io_.update(D::reg, 0, {{D::DCHUBBUB_ARB_ALLOW_SELF_REFRESH_FORCE_VALUE, 0},
                                    {D::DCHUBBUB_ARB_ALLOW_SELF_REFRESH_FORCE_ENABLE, 0}}));

    // REG_WRITE(DIO_MEM_PWR_CTRL, 0): power the AFMT/HDMI stream memories.
    DCN32_IO(io_.write(r::DIO_MEM_PWR_CTRL::reg, 0, 0));

    // !dc->debug.disable_clock_gate (dcn32 default): enable all DCN clock gating.
    DCN32_IO(io_.write(r::DCCG_GATE_DISABLE_CNTL::reg, 0, 0));
    DCN32_IO(io_.write(r::DCCG_GATE_DISABLE_CNTL2::reg, 0, 0));
    DCN32_IO(io_.update(r::DCFCLK_CNTL::reg, 0, {{r::DCFCLK_CNTL::DCFCLK_GATE_DIS, 0}}));

    // hubbub32_init_watermarks (VBIOS-lit, not accelerated mode): A copied to B..D.
    struct Copy final {
        const r::Register* a;
        const r::Register* b;
        const r::Register* c;
        const r::Register* d;
    };
    static constexpr Copy copies[8] = {
        {&r::DCHUBBUB_ARB_DATA_URGENCY_WATERMARK_A::reg, &r::DCHUBBUB_ARB_DATA_URGENCY_WATERMARK_B::reg,
         &r::DCHUBBUB_ARB_DATA_URGENCY_WATERMARK_C::reg, &r::DCHUBBUB_ARB_DATA_URGENCY_WATERMARK_D::reg},
        {&r::DCHUBBUB_ARB_FRAC_URG_BW_FLIP_A::reg, &r::DCHUBBUB_ARB_FRAC_URG_BW_FLIP_B::reg,
         &r::DCHUBBUB_ARB_FRAC_URG_BW_FLIP_C::reg, &r::DCHUBBUB_ARB_FRAC_URG_BW_FLIP_D::reg},
        {&r::DCHUBBUB_ARB_FRAC_URG_BW_NOM_A::reg, &r::DCHUBBUB_ARB_FRAC_URG_BW_NOM_B::reg,
         &r::DCHUBBUB_ARB_FRAC_URG_BW_NOM_C::reg, &r::DCHUBBUB_ARB_FRAC_URG_BW_NOM_D::reg},
        {&r::DCHUBBUB_ARB_REFCYC_PER_TRIP_TO_MEMORY_A::reg, &r::DCHUBBUB_ARB_REFCYC_PER_TRIP_TO_MEMORY_B::reg,
         &r::DCHUBBUB_ARB_REFCYC_PER_TRIP_TO_MEMORY_C::reg, &r::DCHUBBUB_ARB_REFCYC_PER_TRIP_TO_MEMORY_D::reg},
        {&r::DCHUBBUB_ARB_ALLOW_SR_ENTER_WATERMARK_A::reg, &r::DCHUBBUB_ARB_ALLOW_SR_ENTER_WATERMARK_B::reg,
         &r::DCHUBBUB_ARB_ALLOW_SR_ENTER_WATERMARK_C::reg, &r::DCHUBBUB_ARB_ALLOW_SR_ENTER_WATERMARK_D::reg},
        {&r::DCHUBBUB_ARB_ALLOW_SR_EXIT_WATERMARK_A::reg, &r::DCHUBBUB_ARB_ALLOW_SR_EXIT_WATERMARK_B::reg,
         &r::DCHUBBUB_ARB_ALLOW_SR_EXIT_WATERMARK_C::reg, &r::DCHUBBUB_ARB_ALLOW_SR_EXIT_WATERMARK_D::reg},
        {&r::DCHUBBUB_ARB_USR_RETRAINING_WATERMARK_A::reg, &r::DCHUBBUB_ARB_USR_RETRAINING_WATERMARK_B::reg,
         &r::DCHUBBUB_ARB_USR_RETRAINING_WATERMARK_C::reg, &r::DCHUBBUB_ARB_USR_RETRAINING_WATERMARK_D::reg},
        {&r::DCHUBBUB_ARB_UCLK_PSTATE_CHANGE_WATERMARK_A::reg, &r::DCHUBBUB_ARB_UCLK_PSTATE_CHANGE_WATERMARK_B::reg,
         &r::DCHUBBUB_ARB_UCLK_PSTATE_CHANGE_WATERMARK_C::reg, &r::DCHUBBUB_ARB_UCLK_PSTATE_CHANGE_WATERMARK_D::reg},
    };
    for (const Copy& copy : copies) {
        uint32_t value = 0;
        DCN32_IO(io_.read(*copy.a, 0, value));
        DCN32_IO(io_.write(*copy.b, 0, value));
        DCN32_IO(io_.write(*copy.c, 0, value));
        DCN32_IO(io_.write(*copy.d, 0, value));
    }
    {
        uint32_t value = 0;
        DCN32_IO(io_.read(r::DCHUBBUB_ARB_FCLK_PSTATE_CHANGE_WATERMARK_A::reg, 0, value));
        DCN32_IO(io_.write(r::DCHUBBUB_ARB_FCLK_PSTATE_CHANGE_WATERMARK_B::reg, 0, value));
        DCN32_IO(io_.write(r::DCHUBBUB_ARB_FCLK_PSTATE_CHANGE_WATERMARK_C::reg, 0, value));
        DCN32_IO(io_.write(r::DCHUBBUB_ARB_FCLK_PSTATE_CHANGE_WATERMARK_D::reg, 0, value));
    }

    DCN32_STEP(force_pstate_change_control(false, false));

    // dcn32_init_crb.
    for (uint32_t index = 0; index < dcn32_stream_pipes; ++index) {
        DCN32_IO(io_.get(*det_registers[index].reg, 0, det_registers[index].current, hubbub_.det_size[index]));
    }
    DCN32_IO(io_.get(r::DCHUBBUB_COMPBUF_CTRL::reg, 0, r::DCHUBBUB_COMPBUF_CTRL::COMPBUF_SIZE_CURRENT,
                     hubbub_.compbuf_size_segments));
    namespace C = r::COMPBUF_RESERVED_SPACE;
    DCN32_IO(io_.set(C::reg, 0, {{C::COMPBUF_RESERVED_SPACE_64B, pixel_chunk_bytes / 32},
                                 {C::COMPBUF_RESERVED_SPACE_ZS, pixel_chunk_bytes / 128}}));
    DCN32_IO(io_.update(r::DCHUBBUB_DEBUG_CTRL_0::reg, 0, {{r::DCHUBBUB_DEBUG_CTRL_0::DET_DEPTH, 0x47f}}));
    hubbub_.a = {};
    hubbub_.b = {};
    hubbub_.c = {};
    hubbub_.d = {};
    takeover_pending_ = false;
    return Step::ok;
}

// ---- OPTC (dcn10/30/32 optc) ----

Dcn32StreamController::Step Dcn32StreamController::pipe_lock(uint32_t otg, bool lock, bool& locked) noexcept {
    // dcn32_interdependent_update_lock: only an enabled OTG master is locked.
    uint32_t enabled = 0;
    DCN32_IO(io_.get(r::OTG_CONTROL::reg, otg, r::OTG_CONTROL::OTG_MASTER_EN, enabled));
    locked = enabled != 0;
    if (!locked) {
        return Step::ok;
    }
    if (lock) {
        // optc3_lock.
        DCN32_IO(io_.update(r::OTG_GLOBAL_CONTROL2::reg, otg, {{r::OTG_GLOBAL_CONTROL2::OTG_MASTER_UPDATE_LOCK_SEL, otg}}));
        DCN32_IO(io_.set(r::OTG_MASTER_UPDATE_LOCK::reg, otg, {{r::OTG_MASTER_UPDATE_LOCK::OTG_MASTER_UPDATE_LOCK, 1}}));
        DCN32_IO(io_.wait(r::OTG_MASTER_UPDATE_LOCK::reg, otg, r::OTG_MASTER_UPDATE_LOCK::UPDATE_LOCK_STATUS, 1, 1, 10));
    } else {
        // optc1_unlock.
        DCN32_IO(io_.set(r::OTG_MASTER_UPDATE_LOCK::reg, otg, {{r::OTG_MASTER_UPDATE_LOCK::OTG_MASTER_UPDATE_LOCK, 0}}));
    }
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::interdependent_lock(const Dcn32Topology& context,
                                                                      bool lock) noexcept {
    // dcn32_interdependent_update_lock -> dcn20_pipe_control_lock (no GSL, no
    // DMUB lock, no triple buffering): every enabled OTG master of the context.
    for (uint32_t pipe = 0; pipe < dcn32_stream_pipes; ++pipe) {
        if (!topology_is_otg_master(context, pipe)) {
            continue;
        }
        bool locked = false;
        DCN32_STEP(pipe_lock(pipe, lock, locked));
    }
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::disable_crtc(uint32_t otg) noexcept {
    // optc32_disable_crtc.
    namespace S = r::OPTC_DATA_SOURCE_SELECT;
    DCN32_IO(io_.update(S::reg, otg, {{S::OPTC_SEG0_SRC_SEL, 0xf}, {S::OPTC_SEG1_SRC_SEL, 0xf},
                                      {S::OPTC_SEG2_SRC_SEL, 0xf}, {S::OPTC_SEG3_SRC_SEL, 0xf},
                                      {S::OPTC_NUM_OF_INPUT_SEGMENT, 0}}));
    DCN32_IO(io_.update(r::OPTC_MEMORY_CONFIG::reg, otg, {{r::OPTC_MEMORY_CONFIG::OPTC_MEM_SEL, 0}}));
    DCN32_IO(io_.update(r::OTG_CONTROL::reg, otg, {{r::OTG_CONTROL::OTG_MASTER_EN, 0}}));
    DCN32_IO(io_.update(r::VTG_CONTROL::reg, otg, {{r::VTG_CONTROL::VTG0_ENABLE, 0}}));
    DCN32_IO(io_.wait(r::OTG_CLOCK_CONTROL::reg, otg, r::OTG_CLOCK_CONTROL::OTG_BUSY, 0, 1, 150000));
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::enable_optc_clock(uint32_t otg, bool enable) noexcept {
    // optc1_enable_optc_clock.
    namespace I = r::OPTC_INPUT_CLOCK_CONTROL;
    namespace C = r::OTG_CLOCK_CONTROL;
    if (enable) {
        DCN32_IO(io_.update(I::reg, otg, {{I::OPTC_INPUT_CLK_EN, 1}, {I::OPTC_INPUT_CLK_GATE_DIS, 1}}));
        DCN32_IO(io_.wait(I::reg, otg, I::OPTC_INPUT_CLK_ON, 1, 1, 1000));
        DCN32_IO(io_.update(C::reg, otg, {{C::OTG_CLOCK_EN, 1}, {C::OTG_CLOCK_GATE_DIS, 1}}));
        DCN32_IO(io_.wait(C::reg, otg, C::OTG_CLOCK_ON, 1, 1, 1000));
        return Step::ok;
    }
    uint32_t underflow = 0;
    namespace G = r::OPTC_INPUT_GLOBAL_CONTROL;
    DCN32_IO(io_.get(G::reg, otg, G::OPTC_UNDERFLOW_OCCURRED_STATUS, underflow));
    if (underflow == 1) {
        DCN32_IO(io_.update(G::reg, otg, {{G::OPTC_UNDERFLOW_CLEAR, 1}}));
    }
    DCN32_IO(io_.update(C::reg, otg, {{C::OTG_CLOCK_GATE_DIS, 0}, {C::OTG_CLOCK_EN, 0}}));
    DCN32_IO(io_.update(I::reg, otg, {{I::OPTC_INPUT_CLK_GATE_DIS, 0}, {I::OPTC_INPUT_CLK_EN, 0}}));
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::set_odm_bypass(uint32_t otg) noexcept {
    // optc32_set_odm_bypass (RGB: no two pixels per container).
    namespace S = r::OPTC_DATA_SOURCE_SELECT;
    DCN32_IO(io_.set(S::reg, otg, {{S::OPTC_NUM_OF_INPUT_SEGMENT, 0}, {S::OPTC_SEG0_SRC_SEL, otg},
                                   {S::OPTC_SEG1_SRC_SEL, 0xf}, {S::OPTC_SEG2_SRC_SEL, 0xf},
                                   {S::OPTC_SEG3_SRC_SEL, 0xf}}));
    DCN32_IO(io_.update(r::OTG_H_TIMING_CNTL::reg, otg, {{r::OTG_H_TIMING_CNTL::OTG_H_TIMING_DIV_MODE, 0}}));
    DCN32_IO(io_.set(r::OPTC_MEMORY_CONFIG::reg, otg, {{r::OPTC_MEMORY_CONFIG::OPTC_MEM_SEL, 0}}));
    opp_count_[otg] = 1;
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::set_odm_combine(const Dcn32StreamPlan& plan) noexcept {
    // optc32_set_odm_combine with the OPP heads of the OTG master.
    uint8_t heads[dcn32_topology_pipes]{};
    const uint32_t opp_cnt = topology_opp_heads(plan.bandwidth.topology, heads);
    if (opp_cnt != 2 && opp_cnt != 4) {
        return fail(StreamFailure::invalid_plan);
    }
    const uint32_t otg = plan.pipe;
    const uint32_t segment_width = plan.odm_slice_width;
    const uint32_t h_active = segment_width * opp_cnt;
    // Each memory instance is 2048x(32x2) bits to support half line of 4096.
    const uint32_t odm_mem_count = (h_active + 2047) / 2048;
    uint32_t memory_mask = 0;
    const uint32_t opp0 = topo(plan, heads[0]).opp;
    const uint32_t opp1 = topo(plan, heads[1]).opp;
    if (opp_cnt == 4) {
        memory_mask = odm_mem_count <= 2 ? 0x3u : (odm_mem_count <= 4 ? 0xfu : 0x3fu);
    } else if (odm_mem_count <= 2) {
        memory_mask = (0x1u << (opp0 * 2)) | (0x1u << (opp1 * 2));
    } else if (odm_mem_count <= 4) {
        memory_mask = (0x3u << (opp0 * 2)) | (0x3u << (opp1 * 2));
    } else {
        memory_mask = 0x77;
    }
    DCN32_IO(io_.set(r::OPTC_MEMORY_CONFIG::reg, otg, {{r::OPTC_MEMORY_CONFIG::OPTC_MEM_SEL, memory_mask}}));
    namespace S = r::OPTC_DATA_SOURCE_SELECT;
    if (opp_cnt == 2) {
        DCN32_IO(io_.set(S::reg, otg, {{S::OPTC_NUM_OF_INPUT_SEGMENT, 1}, {S::OPTC_SEG0_SRC_SEL, opp0},
                                       {S::OPTC_SEG1_SRC_SEL, opp1}}));
    } else {
        DCN32_IO(io_.set(S::reg, otg, {{S::OPTC_NUM_OF_INPUT_SEGMENT, 3}, {S::OPTC_SEG0_SRC_SEL, opp0},
                                       {S::OPTC_SEG1_SRC_SEL, opp1},
                                       {S::OPTC_SEG2_SRC_SEL, topo(plan, heads[2]).opp},
                                       {S::OPTC_SEG3_SRC_SEL, topo(plan, heads[3]).opp}}));
    }
    DCN32_IO(io_.update(r::OPTC_WIDTH_CONTROL::reg, otg,
                        {{r::OPTC_WIDTH_CONTROL::OPTC_SEGMENT_WIDTH, segment_width}}));
    DCN32_IO(io_.update(r::OTG_H_TIMING_CNTL::reg, otg,
                        {{r::OTG_H_TIMING_CNTL::OTG_H_TIMING_DIV_MODE, opp_cnt - 1}}));
    opp_count_[otg] = opp_cnt;
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::setup_manual_trigger(uint32_t otg) noexcept {
    // optc32_set_drr (no DRR) -> optc32_setup_manual_trigger (no FAMS).
    namespace V = r::OTG_V_TOTAL_CONTROL;
    DCN32_IO(io_.update(V::reg, otg, {{V::OTG_V_TOTAL_MIN_SEL, 1}, {V::OTG_V_TOTAL_MAX_SEL, 1},
                                      {V::OTG_FORCE_LOCK_ON_EVENT, 0}, {V::OTG_SET_V_TOTAL_MIN_MASK, 1u << 1}}));
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::program_global_sync(uint32_t otg, uint32_t vready_offset,
                                                                      const Dcn32PipeBandwidth& head) noexcept {
    // optc1_program_global_sync.
    if (head.vstartup_start == 0) {
        return fail(StreamFailure::invalid_plan);
    }
    DCN32_IO(io_.set(r::OTG_VSTARTUP_PARAM::reg, otg, {{r::OTG_VSTARTUP_PARAM::VSTARTUP_START, head.vstartup_start}}));
    DCN32_IO(io_.set(r::OTG_VUPDATE_PARAM::reg, otg, {{r::OTG_VUPDATE_PARAM::VUPDATE_OFFSET, head.vupdate_offset},
                                                      {r::OTG_VUPDATE_PARAM::VUPDATE_WIDTH, head.vupdate_width}}));
    DCN32_IO(io_.set(r::OTG_VREADY_PARAM::reg, otg, {{r::OTG_VREADY_PARAM::VREADY_OFFSET, vready_offset}}));
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::set_vtg_params(const Dcn32StreamPlan& plan,
                                                                  bool program_fp2) noexcept {
    // optc1_set_vtg_params.
    const Timing& t = plan.timing;
    const uint32_t v_init = t.v_total - patched_v_front_porch(t);
    const uint32_t asic_blank_end = v_init - t.height;
    const int32_t vertical_line_start =
        static_cast<int32_t>(asic_blank_end) - static_cast<int32_t>(plan.bandwidth.vstartup_start) + 1;
    const uint32_t v_fp2 = vertical_line_start < 0 ? static_cast<uint32_t>(-vertical_line_start) : 0u;
    if (program_fp2) {
        DCN32_IO(io_.update(r::VTG_CONTROL::reg, plan.pipe,
                            {{r::VTG_CONTROL::VTG0_FP2, v_fp2}, {r::VTG_CONTROL::VTG0_VCOUNT_INIT, v_init}}));
    } else {
        DCN32_IO(io_.update(r::VTG_CONTROL::reg, plan.pipe, {{r::VTG_CONTROL::VTG0_VCOUNT_INIT, v_init}}));
    }
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::program_timing(const Dcn32StreamPlan& plan) noexcept {
    // optc1_program_timing for a progressive RGB timing.
    const Timing& t = plan.timing;
    const uint32_t otg = plan.pipe;
    const uint32_t v_fp = patched_v_front_porch(t);
    DCN32_IO(io_.set(r::OTG_H_TOTAL::reg, otg, {{r::OTG_H_TOTAL::OTG_H_TOTAL, t.h_total - 1}}));
    DCN32_IO(io_.update(r::OTG_H_SYNC_A::reg, otg,
                        {{r::OTG_H_SYNC_A::OTG_H_SYNC_A_START, 0}, {r::OTG_H_SYNC_A::OTG_H_SYNC_A_END, t.h_sync_width}}));
    const uint32_t h_blank_start = t.h_total - t.h_front_porch;
    const uint32_t h_blank_end = h_blank_start - t.width;
    DCN32_IO(io_.update(r::OTG_H_BLANK_START_END::reg, otg,
                        {{r::OTG_H_BLANK_START_END::OTG_H_BLANK_START, h_blank_start},
                         {r::OTG_H_BLANK_START_END::OTG_H_BLANK_END, h_blank_end}}));
    DCN32_IO(io_.update(r::OTG_H_SYNC_A_CNTL::reg, otg,
                        {{r::OTG_H_SYNC_A_CNTL::OTG_H_SYNC_A_POL, (t.flags & positive_hsync) ? 0u : 1u}}));
    const uint32_t v_total = t.v_total - 1;
    DCN32_IO(io_.set(r::OTG_V_TOTAL::reg, otg, {{r::OTG_V_TOTAL::OTG_V_TOTAL, v_total}}));
    // optc3_set_vtotal_min_max without FAMS: optc1_set_vtotal_min_max.
    DCN32_IO(io_.set(r::OTG_V_TOTAL_MAX::reg, otg, {{r::OTG_V_TOTAL_MAX::OTG_V_TOTAL_MAX, v_total}}));
    DCN32_IO(io_.set(r::OTG_V_TOTAL_MIN::reg, otg, {{r::OTG_V_TOTAL_MIN::OTG_V_TOTAL_MIN, v_total}}));
    DCN32_IO(io_.update(r::OTG_V_SYNC_A::reg, otg,
                        {{r::OTG_V_SYNC_A::OTG_V_SYNC_A_START, 0}, {r::OTG_V_SYNC_A::OTG_V_SYNC_A_END, t.v_sync_width}}));
    const uint32_t v_blank_start = t.v_total - v_fp;
    const uint32_t v_blank_end = v_blank_start - t.height;
    DCN32_IO(io_.update(r::OTG_V_BLANK_START_END::reg, otg,
                        {{r::OTG_V_BLANK_START_END::OTG_V_BLANK_START, v_blank_start},
                         {r::OTG_V_BLANK_START_END::OTG_V_BLANK_END, v_blank_end}}));
    DCN32_IO(io_.update(r::OTG_V_SYNC_A_CNTL::reg, otg,
                        {{r::OTG_V_SYNC_A_CNTL::OTG_V_SYNC_A_POL, (t.flags & positive_vsync) ? 0u : 1u}}));
    const uint32_t start_point = is_dp(plan.signal) ? 1u : 0u;
    DCN32_IO(io_.update(r::OTG_INTERLACE_CONTROL::reg, otg, {{r::OTG_INTERLACE_CONTROL::OTG_INTERLACE_ENABLE, 0}}));
    DCN32_IO(io_.update(r::VTG_CONTROL::reg, otg, {{r::VTG_CONTROL::VTG0_ENABLE, 0}}));
    DCN32_IO(io_.update(r::OTG_CONTROL::reg, otg, {{r::OTG_CONTROL::OTG_START_POINT_CNTL, start_point},
                                                   {r::OTG_CONTROL::OTG_FIELD_NUMBER_CNTL, 0}}));
    // program_timing passes the OTG master's own pipe_dlg_param.
    DCN32_STEP(program_global_sync(otg, plan.bandwidth.vready_offset, pipe_bandwidth(plan, otg)));
    DCN32_STEP(set_vtg_params(plan, true));
    DCN32_IO(io_.update(r::OPTC_DATA_FORMAT_CONTROL::reg, otg, {{r::OPTC_DATA_FORMAT_CONTROL::OPTC_DATA_FORMAT, 0}}));
    // H_TIMING_NO_DIV / DIV_BY2 / DIV_BY4 from optc1->opp_count.
    const uint32_t h_div = opp_count_[otg] == 4 ? 3u : (opp_count_[otg] == 2 ? 1u : 0u);
    DCN32_IO(io_.update(r::OTG_H_TIMING_CNTL::reg, otg, {{r::OTG_H_TIMING_CNTL::OTG_H_TIMING_DIV_MODE, h_div}}));
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::enable_crtc(uint32_t otg) noexcept {
    // optc32_enable_crtc.
    DCN32_IO(io_.update(r::OPTC_DATA_SOURCE_SELECT::reg, otg, {{r::OPTC_DATA_SOURCE_SELECT::OPTC_SEG0_SRC_SEL, otg}}));
    DCN32_IO(io_.update(r::VTG_CONTROL::reg, otg, {{r::VTG_CONTROL::VTG0_ENABLE, 1}}));
    DCN32_IO(io_.update(r::OTG_CONTROL::reg, otg,
                        {{r::OTG_CONTROL::OTG_DISABLE_POINT_CNTL, 2}, {r::OTG_CONTROL::OTG_MASTER_EN, 1}}));
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::wait_for_state(uint32_t otg, bool vblank) noexcept {
    // optc1_wait_for_state.
    if (vblank) {
        DCN32_IO(io_.wait(r::OTG_STATUS::reg, otg, r::OTG_STATUS::OTG_V_BLANK, 1, 1, 100000));
    } else {
        DCN32_IO(io_.wait(r::OTG_STATUS::reg, otg, r::OTG_STATUS::OTG_V_ACTIVE_DISP, 1, 1, 100000));
    }
    return Step::ok;
}

// ---- OPP (dcn10_opp.c, dcn20_opp.c) ----

Dcn32StreamController::Step Dcn32StreamController::dpg_pattern(uint32_t opp, uint32_t width, uint32_t height,
                                                               uint32_t offset, bool blank) noexcept {
    // dcn30_set_disp_pattern_generator -> opp2_set_disp_pattern_generator;
    // black is RGB full range (0, 0, 0).
    DCN32_IO(io_.set(r::DPG_DIMENSIONS::reg, opp, {{r::DPG_DIMENSIONS::DPG_ACTIVE_WIDTH, width},
                                                   {r::DPG_DIMENSIONS::DPG_ACTIVE_HEIGHT, height}}));
    DCN32_IO(io_.set(r::DPG_OFFSET_SEGMENT::reg, opp, {{r::DPG_OFFSET_SEGMENT::DPG_X_OFFSET, offset},
                                                       {r::DPG_OFFSET_SEGMENT::DPG_SEGMENT_WIDTH, 0}}));
    if (blank) {
        // CONTROLLER_DP_TEST_PATTERN_SOLID_COLOR: opp2_dpg_set_blank_color.
        DCN32_IO(io_.set(r::DPG_COLOUR_B_CB::reg, opp, {{r::DPG_COLOUR_B_CB::DPG_COLOUR1_B_CB, 0},
                                                        {r::DPG_COLOUR_B_CB::DPG_COLOUR0_B_CB, 0}}));
        DCN32_IO(io_.set(r::DPG_COLOUR_G_Y::reg, opp, {{r::DPG_COLOUR_G_Y::DPG_COLOUR1_G_Y, 0},
                                                       {r::DPG_COLOUR_G_Y::DPG_COLOUR0_G_Y, 0}}));
        DCN32_IO(io_.set(r::DPG_COLOUR_R_CR::reg, opp, {{r::DPG_COLOUR_R_CR::DPG_COLOUR1_R_CR, 0},
                                                        {r::DPG_COLOUR_R_CR::DPG_COLOUR0_R_CR, 0}}));
        DCN32_IO(io_.update(r::DPG_CONTROL::reg, opp,
                            {{r::DPG_CONTROL::DPG_EN, 1}, {r::DPG_CONTROL::DPG_MODE, dpg_mode_horizontal_bars}}));
        DCN32_IO(io_.set(r::DPG_DIMENSIONS::reg, opp, {{r::DPG_DIMENSIONS::DPG_ACTIVE_WIDTH, width},
                                                       {r::DPG_DIMENSIONS::DPG_ACTIVE_HEIGHT, height}}));
        return Step::ok;
    }
    // CONTROLLER_DP_TEST_PATTERN_VIDEOMODE.
    DCN32_IO(io_.write(r::DPG_CONTROL::reg, opp, 0));
    DCN32_IO(io_.write(r::DPG_COLOUR_R_CR::reg, opp, 0));
    DCN32_IO(io_.write(r::DPG_COLOUR_G_Y::reg, opp, 0));
    DCN32_IO(io_.write(r::DPG_COLOUR_B_CB::reg, opp, 0));
    DCN32_IO(io_.write(r::DPG_RAMP_CONTROL::reg, opp, 0));
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::blank_pixel_data(const Dcn32Topology& context,
                                                                    const Timing& timing, bool blank) noexcept {
    // dcn20_blank_pixel_data of the context's OTG master: the pattern of every
    // OPP head in ODM order over its slice (set_abm_immediate_disable has no
    // ABM/panel on an external connector).
    uint8_t heads[dcn32_topology_pipes]{};
    const uint32_t count = topology_opp_heads(context, heads);
    for (uint32_t i = 0; i < count; ++i) {
        uint32_t x = 0;
        uint32_t width = 0;
        odm_slice(context, timing.width, heads[i], x, width);
        DCN32_STEP(dpg_pattern(context.pipes[heads[i]].opp, width, timing.height, x, blank));
    }
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::wait_for_blank_complete(uint32_t opp) noexcept {
    // dcn20_wait_for_blank_complete: opp2_dpg_is_pending x 1000, 100 us apart,
    // then opp2_dpg_is_blanked (its result is not used by the callers here).
    uint32_t counter = 0;
    for (; counter < 1000; ++counter) {
        uint32_t enabled = 0;
        uint32_t pending = 0;
        DCN32_IO(io_.get(r::DPG_CONTROL::reg, opp, r::DPG_CONTROL::DPG_EN, enabled));
        DCN32_IO(io_.get(r::DPG_STATUS::reg, opp, r::DPG_STATUS::DPG_DOUBLE_BUFFER_PENDING, pending));
        if (!(enabled == 1 && pending == 1)) {
            break;
        }
        io_.delay_us(100);
    }
    if (counter == 1000) {
        return fail(StreamFailure::register_timeout);
    }
    uint32_t control = 0;
    uint32_t status = 0;
    DCN32_IO(io_.read(r::DPG_CONTROL::reg, opp, control));
    DCN32_IO(io_.get(r::DPG_STATUS::reg, opp, r::DPG_STATUS::DPG_DOUBLE_BUFFER_PENDING, status));
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::set_dyn_expansion(uint32_t opp, StreamSignal signal) noexcept {
    // opp1_set_dyn_expansion(COLOR_SPACE_YCBCR601, COLOR_DEPTH_888, signal), DYN_EXPANSION_AUTO.
    namespace E = r::FMT_DYNAMIC_EXP_CNTL;
    DCN32_IO(io_.update(E::reg, opp, {{E::FMT_DYNAMIC_EXP_EN, 0}, {E::FMT_DYNAMIC_EXP_MODE, 0}}));
    if (is_hdmi(signal) || is_dp(signal)) {
        DCN32_IO(io_.update(E::reg, opp, {{E::FMT_DYNAMIC_EXP_EN, 1}, {E::FMT_DYNAMIC_EXP_MODE, 1}}));
    }
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::program_fmt(uint32_t opp) noexcept {
    // opp1_program_fmt with resource_build_bit_depth_reduction_params for
    // DITHER_OPTION_SPATIAL8 (COLOR_DEPTH_888, RGB) and CLAMPING_FULL_RANGE.
    namespace B = r::FMT_BIT_DEPTH_CONTROL;
    namespace C = r::FMT_CONTROL;
    DCN32_IO(io_.update(B::reg, opp, {{B::FMT_TRUNCATE_EN, 0}, {B::FMT_TRUNCATE_DEPTH, 0}, {B::FMT_TRUNCATE_MODE, 0}}));
    DCN32_IO(io_.update(B::reg, opp, {{B::FMT_SPATIAL_DITHER_EN, 0}, {B::FMT_SPATIAL_DITHER_MODE, 0},
                                      {B::FMT_SPATIAL_DITHER_DEPTH, 0}, {B::FMT_TEMPORAL_DITHER_EN, 0},
                                      {B::FMT_HIGHPASS_RANDOM_ENABLE, 0}, {B::FMT_FRAME_RANDOM_ENABLE, 0},
                                      {B::FMT_RGB_RANDOM_ENABLE, 0}}));
    DCN32_IO(io_.update(C::reg, opp, {{C::FMT_SPATIAL_DITHER_FRAME_COUNTER_MAX, 0},
                                      {C::FMT_SPATIAL_DITHER_FRAME_COUNTER_BIT_SWAP, 0}}));
    DCN32_IO(io_.set(r::FMT_DITHER_RAND_R_SEED::reg, opp, {{r::FMT_DITHER_RAND_R_SEED::FMT_RAND_R_SEED, 0}}));
    DCN32_IO(io_.set(r::FMT_DITHER_RAND_G_SEED::reg, opp, {{r::FMT_DITHER_RAND_G_SEED::FMT_RAND_G_SEED, 0}}));
    DCN32_IO(io_.set(r::FMT_DITHER_RAND_B_SEED::reg, opp, {{r::FMT_DITHER_RAND_B_SEED::FMT_RAND_B_SEED, 0}}));
    DCN32_IO(io_.update(B::reg, opp, {{B::FMT_SPATIAL_DITHER_EN, 1}, {B::FMT_SPATIAL_DITHER_MODE, 0},
                                      {B::FMT_SPATIAL_DITHER_DEPTH, 1}, {B::FMT_HIGHPASS_RANDOM_ENABLE, 1},
                                      {B::FMT_FRAME_RANDOM_ENABLE, 0}, {B::FMT_RGB_RANDOM_ENABLE, 1}}));
    namespace K = r::FMT_CLAMP_CNTL;
    DCN32_IO(io_.update(K::reg, opp, {{K::FMT_CLAMP_DATA_EN, 0}, {K::FMT_CLAMP_COLOR_FORMAT, 0}}));
    DCN32_IO(io_.update(K::reg, opp, {{K::FMT_CLAMP_DATA_EN, 1}, {K::FMT_CLAMP_COLOR_FORMAT, 0}}));
    DCN32_IO(io_.update(C::reg, opp, {{C::FMT_PIXEL_ENCODING, 0}, {C::FMT_SUBSAMPLING_MODE, 0},
                                      {C::FMT_CBCR_BIT_REDUCTION_BYPASS, 0}}));
    DCN32_IO(io_.update(C::reg, opp, {{C::FMT_PIXEL_ENCODING, 0}}));
    return Step::ok;
}

// ---- MPC / HUBP disconnect and plane disable (dcn10/20 hwseq, mpc1/mpc2, hubp2) ----

Dcn32StreamController::Step Dcn32StreamController::remove_mpcc(uint32_t opp, uint32_t mpcc) noexcept {
    // mpc1_remove_mpcc.
    bool found = false;
    if (mpc_tree_top_[opp] == mpcc) {
        found = true;
        // Remove the MPCC from the top of the tree.
        if (mpcc_bot_[mpcc] != mpcc_none) {
            mpc_tree_top_[opp] = mpcc_bot_[mpcc];
            DCN32_IO(io_.update(r::MPC_OUT_MUX::reg, opp, {{r::MPC_OUT_MUX::MPC_OUT_MUX, mpc_tree_top_[opp]}}));
        } else {
            mpc_tree_top_[opp] = mpcc_none;
            DCN32_IO(io_.update(r::MPC_OUT_MUX::reg, opp, {{r::MPC_OUT_MUX::MPC_OUT_MUX, mpcc_none}}));
        }
    } else {
        uint32_t temp = mpc_tree_top_[opp];
        uint32_t guard = 0;
        while (temp != mpcc_none && mpcc_bot_[temp] != mpcc && guard++ < dcn32_stream_pipes) {
            temp = mpcc_bot_[temp];
        }
        if (temp != mpcc_none && mpcc_bot_[temp] == mpcc) {
            found = true;
            mpcc_bot_[temp] = mpcc_bot_[mpcc];
            if (mpcc_bot_[mpcc] != mpcc_none) {
                // Remove the MPCC in the middle of the list.
                DCN32_IO(io_.set(r::MPCC_BOT_SEL::reg, temp, {{r::MPCC_BOT_SEL::MPCC_BOT_SEL, mpcc_bot_[mpcc]}}));
            } else {
                // Remove the MPCC from the bottom of the list.
                DCN32_IO(io_.set(r::MPCC_BOT_SEL::reg, temp, {{r::MPCC_BOT_SEL::MPCC_BOT_SEL, mpcc_none}}));
                DCN32_IO(io_.update(r::MPCC_CONTROL::reg, temp,
                                    {{r::MPCC_CONTROL::MPCC_MODE, mpcc_mode_top_layer_passthrough}}));
            }
        }
    }
    // Turn off the MPCC mux registers (also for a BIOS leftover not in the tree).
    DCN32_IO(io_.set(r::MPCC_TOP_SEL::reg, mpcc, {{r::MPCC_TOP_SEL::MPCC_TOP_SEL, mpcc_none}}));
    DCN32_IO(io_.set(r::MPCC_BOT_SEL::reg, mpcc, {{r::MPCC_BOT_SEL::MPCC_BOT_SEL, mpcc_none}}));
    DCN32_IO(io_.set(r::MPCC_OPP_ID::reg, mpcc, {{r::MPCC_OPP_ID::MPCC_OPP_ID, mpcc_none}}));
    DCN32_IO(io_.set(r::MPCC_UPDATE_LOCK_SEL::reg, mpcc, {{r::MPCC_UPDATE_LOCK_SEL::MPCC_UPDATE_LOCK_SEL, mpcc_none}}));
    if (found) {
        mpcc_dpp_[mpcc] = mpcc_none;
        mpcc_bot_[mpcc] = mpcc_none;
    }
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::plane_atomic_disconnect(uint32_t pipe, uint32_t opp) noexcept {
    // dcn10_plane_atomic_disconnect: mpc1_get_mpcc_for_dpp in the OPP's tree,
    // mpc1_remove_mpcc, opp->mpcc_disconnect_pending (no hubp_disconnect on dcn32).
    uint32_t mpcc = mpc_tree_top_[opp];
    uint32_t guard = 0;
    while (mpcc != mpcc_none && mpcc_dpp_[mpcc] != pipe && guard++ < dcn32_stream_pipes) {
        mpcc = mpcc_bot_[mpcc] == mpcc ? mpcc_none : mpcc_bot_[mpcc];
    }
    if (mpcc == mpcc_none) {
        return Step::ok; // already reset
    }
    DCN32_STEP(remove_mpcc(opp, mpcc));
    mpcc_disconnect_pending_[opp][pipe] = true;
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::wait_for_mpcc_disconnect(uint32_t opp, uint32_t otg) noexcept {
    // dcn10_wait_for_mpcc_disconnect for a pipe on opp and otg.
    for (uint32_t mpcc = 0; mpcc < dcn32_stream_pipes; ++mpcc) {
        if (!mpcc_disconnect_pending_[opp][mpcc]) {
            continue;
        }
        uint32_t enabled = 0;
        DCN32_IO(io_.get(r::OTG_CONTROL::reg, otg, r::OTG_CONTROL::OTG_MASTER_EN, enabled));
        if (enabled != 0) {
            // mpc2_assert_idle_mpcc.
            uint32_t disabled = 0;
            DCN32_IO(io_.get(r::MPCC_STATUS::reg, mpcc, r::MPCC_STATUS::MPCC_DISABLED, disabled));
            if (disabled == 0) {
                DCN32_IO(io_.wait(r::MPCC_STATUS::reg, mpcc, r::MPCC_STATUS::MPCC_IDLE, 1, 1, 100000));
            }
        }
        mpcc_disconnect_pending_[opp][mpcc] = false;
        // hubp2_set_blank(true) of HUBP mpcc.
        uint32_t control = 0;
        DCN32_IO(io_.read(r::DCHUBP_CNTL::reg, mpcc, control));
        if (control != 0) {
            DCN32_IO(io_.wait(r::DCHUBP_CNTL::reg, mpcc, r::DCHUBP_CNTL::HUBP_NO_OUTSTANDING_REQ, 1, 1, 100000));
        }
        DCN32_IO(io_.update(r::DCHUBP_CNTL::reg, mpcc,
                            {{r::DCHUBP_CNTL::HUBP_BLANK_EN, 1}, {r::DCHUBP_CNTL::HUBP_TTU_DISABLE, 0}}));
    }
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::plane_atomic_disable(uint32_t pipe, uint32_t opp,
                                                                        uint32_t otg) noexcept {
    // dcn20_plane_atomic_disable; first dcn10_wait_for_mpcc_disconnect.
    DCN32_STEP(wait_for_mpcc_disconnect(opp, otg));
    DCN32_IO(io_.update(r::DCHUBP_MALL_CONFIG::reg, pipe,
                        {{r::DCHUBP_MALL_CONFIG::USE_MALL_SEL, 0}, {r::DCHUBP_MALL_CONFIG::USE_MALL_FOR_CURSOR, 0}}));
    DCN32_IO(io_.update(r::DCSURF_FLIP_CONTROL2::reg, pipe, {{r::DCSURF_FLIP_CONTROL2::SURFACE_GSL_ENABLE, 0}}));
    DCN32_IO(io_.update(r::HUBP_CLK_CNTL::reg, pipe, {{r::HUBP_CLK_CNTL::HUBP_CLOCK_ENABLE, 0}}));
    DCN32_IO(io_.update(r::DPP_CONTROL::reg, pipe, {{r::DPP_CONTROL::DPP_CLOCK_ENABLE, 0}}));
    hubp_power_gated_[pipe] = true;
    // dcn10_plane_atomic_power_down (no DPP power gate or root clock on dcn32).
    DCN32_IO(io_.set(r::DC_IP_REQUEST_CNTL::reg, 0, {{r::DC_IP_REQUEST_CNTL::IP_REQUEST_EN, 1}}));
    DCN32_STEP(hubp_pg_control(pipe, false));
    scl_[pipe] = ScalerCache{}; // dpp_reset
    DCN32_IO(io_.set(r::DC_IP_REQUEST_CNTL::reg, 0, {{r::DC_IP_REQUEST_CNTL::IP_REQUEST_EN, 0}}));
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::disable_dangling_plane() noexcept {
    // disable_dangling_plane for the removed stream: program_front_end_for_ctx
    // of the plane-less copy of the current state under the interdependent
    // lock of the current state, then post unlock; the copy becomes current.
    Dcn32Topology dangling = live_;
    remove_all_planes(dangling);
    DCN32_STEP(interdependent_lock(live_, true));
    DCN32_STEP(program_front_end(nullptr, dangling, UpdateKind{}));
    DCN32_STEP(interdependent_lock(live_, false));
    DCN32_STEP(post_unlock_front_end(nullptr, dangling));
    live_ = dangling;
    current_plane_ = false;
    return Step::ok;
}

// ---- stream back end ----

Dcn32StreamController::Step Dcn32StreamController::set_avmute(const Dcn32StreamPlan& stream, bool enable) noexcept {
    // dcn30_set_avmute (HDMI only).
    if (!is_hdmi(stream.signal)) {
        return Step::ok;
    }
    DCN32_IO(dcn32_encoder::set_avmute(io_, stream.engine, enable));
    if (!enable) {
        return Step::ok;
    }
    uint32_t enabled = 0;
    DCN32_IO(io_.get(r::OTG_CONTROL::reg, stream.pipe, r::OTG_CONTROL::OTG_MASTER_EN, enabled));
    if (enabled != 0) {
        DCN32_STEP(wait_for_state(stream.pipe, false));
        DCN32_STEP(wait_for_state(stream.pipe, true));
        DCN32_STEP(wait_for_state(stream.pipe, false));
        DCN32_STEP(wait_for_state(stream.pipe, true));
        DCN32_STEP(wait_for_state(stream.pipe, false));
    }
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::disable_stream(const Dcn32StreamPlan& stream) noexcept {
    // dce110_disable_stream.
    if (is_hdmi(stream.signal)) {
        DCN32_IO(dcn32_encoder::stop_hdmi_info_packets(io_, stream.engine));
        DCN32_IO(dcn32_encoder::hdmi_reset_stream_attribute(io_, stream.engine));
    }
    if (is_dp(stream.signal)) {
        DCN32_IO(dcn32_encoder::stop_dp_info_packets(io_, stream.engine));
    }
    // dce110_disable_audio_stream without an audio endpoint.
    DCN32_IO(dcn32_encoder::audio_mute(io_, stream.engine));
    // reset_dio_stream_encoder: set_input_mode(0); the FE->BE disconnect is
    // the last step of the link owner's disable_link.
    DCN32_IO(dcn32_encoder::set_input_mode(io_, stream.engine, 0));
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::link_set_dpms_off(const Dcn32StreamPlan& stream) noexcept {
    DCN32_STEP(set_avmute(stream, true));
    DCN32_IO(dcn32_encoder::audio_mute(io_, stream.engine));
    // dce110_blank_stream.
    if (is_dp(stream.signal)) {
        DCN32_IO(dcn32_encoder::dp_blank(io_, stream.engine));
        io_.delay_us(60000);
    }
    // "Need to inform that sink is going to use legacy HDMI mode":
    // write_scdc_data(ddc, 165000, false); its result is not checked.
    if (is_hdmi(stream.signal) && hooks_.hdmi_scdc_legacy != nullptr) {
        DCN32_STEP(ignored_result(hooks_.hdmi_scdc_legacy(hooks_.context, stream)));
    }
    if (is_dp(stream.signal)) {
        DCN32_STEP(link(hooks_.disable_link(hooks_.context, stream), StreamFailure::link_refused));
        DCN32_STEP(disable_stream(stream));
    } else {
        DCN32_STEP(disable_stream(stream));
        DCN32_STEP(link(hooks_.disable_link(hooks_.context, stream), StreamFailure::link_refused));
    }
    // link_set_dsc_enable(false): dp_set_dsc_on_rx(false) (result ignored),
    // then link_set_dsc_on_stream(false).
    if (stream.dsc && is_dp(stream.signal)) {
        DCN32_STEP(set_dsc_on_rx(stream, false));
        DCN32_STEP(set_dsc_on_stream(stream, false));
    }
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::reset_back_end_for_pipe() noexcept {
    // dcn20_reset_hw_ctx_wrap -> dcn20_reset_back_end_for_pipe of the OTG
    // master (its ODM pipes share the back end), then cs_power_down.
    const Dcn32StreamPlan& old = current_;
    if (!dpms_off_) {
        DCN32_STEP(link_set_dpms_off(old));
    }
    dpms_off_ = false;
    DCN32_STEP(disable_crtc(old.pipe));
    DCN32_STEP(enable_optc_clock(old.pipe, false));
    DCN32_STEP(set_odm_bypass(old.pipe));
    DCN32_STEP(setup_manual_trigger(old.pipe));
    DCN32_STEP(link(hooks_.power_down_pixel_clock(hooks_.context, old), StreamFailure::clock_refused));
    // pipe_ctx->stream = NULL on the current state's OTG master only.
    live_.pipes[old.pipe].stream = false;
    current_stream_ = false;
    current_firmware_ = false;
    return Step::ok;
}

// ---- stream enable ----

Dcn32StreamController::Step Dcn32StreamController::update_info_frame(const Dcn32StreamPlan& plan) noexcept {
    // dcn31_update_info_frame.
    if (is_hdmi(plan.signal)) {
        DCN32_IO(dcn32_encoder::update_hdmi_info_packets(io_, plan.engine, plan.avi, plan.vendor));
    } else if (is_dp(plan.signal)) {
        DCN32_IO(dcn32_encoder::update_dp_info_packets(io_, plan.engine, plan.vsc));
    }
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::enable_stream_features(const Dcn32StreamPlan& plan) noexcept {
    // link_dpms.c enable_stream_features (SST): DP_DOWNSPREAD_CTRL bit 7.
    if (hooks_.dpcd_read == nullptr || hooks_.dpcd_write == nullptr) {
        return fail(StreamFailure::link_refused);
    }
    uint8_t old_downspread = 0;
    DCN32_STEP(link(hooks_.dpcd_read(hooks_.context, dp_downspread_ctrl, old_downspread),
                    StreamFailure::link_refused));
    const uint8_t new_downspread = static_cast<uint8_t>((old_downspread & 0x7fu) | (plan.dp_ignore_msa ? 0x80u : 0u));
    if (new_downspread != old_downspread) {
        DCN32_STEP(link(hooks_.dpcd_write(hooks_.context, dp_downspread_ctrl, new_downspread),
                        StreamFailure::link_refused));
    }
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::link_set_dpms_on(const Dcn32StreamPlan& plan) noexcept {
    const uint32_t engine = plan.engine;
    // set_out_mux(OUT_MUX_DIO).
    DCN32_IO(io_.update(r::OTG_CONTROL::reg, plan.pipe, {{r::OTG_CONTROL::OTG_OUT_MUX, 0}}));
    // setup_dio_stream_attribute.
    DCN32_IO(dcn32_encoder::setup_stereo_sync(io_, engine, plan.pipe, false));
    switch (plan.signal) {
    case StreamSignal::display_port:
        DCN32_IO(dcn32_encoder::dp_set_stream_attribute(io_, engine, plan.timing, plan.dp_use_vsc));
        break;
    case StreamSignal::hdmi:
        DCN32_IO(dcn32_encoder::hdmi_set_stream_attribute(io_, engine, plan.tmds_khz));
        break;
    default:
        DCN32_IO(dcn32_encoder::dvi_set_stream_attribute(io_, engine));
        break;
    }
    DCN32_STEP(update_info_frame(plan));
    // DSC before the DIG FE/BE connect and link training (link_set_dsc_enable).
    if (plan.dsc && is_dp(plan.signal)) {
        DCN32_STEP(set_dsc_on_stream(plan, true));
    }
    // enable_link: link_enc setup, FE->BE connect, training (FEC ready/enable
    // when plan.fec) / SCDC + TMDS.
    DCN32_STEP(link(hooks_.enable_link(hooks_.context, plan), StreamFailure::link_refused));
    // dcn20_enable_stream: setup_dio_stream_encoder (input mode, FIFO), the
    // dynamic metadata engine off, update_info_frame again.
    DCN32_IO(dcn32_encoder::set_input_mode(io_, engine, plan.dio_pix_per_cycle));
    DCN32_IO(dcn32_encoder::enable_fifo(io_, engine));
    DCN32_IO(dcn32_encoder::disable_dynamic_metadata(io_, engine, is_dp(plan.signal)));
    DCN32_STEP(update_info_frame(plan));
    // DSC on the sink, then the PPS SDP (after the FE is connected to the BE).
    if (plan.dsc && is_dp(plan.signal)) {
        DCN32_STEP(set_dsc_on_rx(plan, true));
        DCN32_STEP(set_dsc_pps_packet(plan, true, true));
    }
    if (is_dp(plan.signal)) {
        // dcn32_unblank_stream: with ODM the encoder sees half the pixel rate
        // at two pixels per cycle (pix_clk_100hz / 2, then / 10 as kHz).
        const bool odm = plan.odm_slices > 1;
        const uint32_t pixel_khz = odm ? (plan.timing.pixel_clock_khz * 10u / 2u) / 10u : plan.timing.pixel_clock_khz;
        DCN32_IO(dcn32_encoder::dp_unblank(io_, engine, pixel_khz, plan.dp_rate_code,
                                           odm ? 2u : plan.dio_pix_per_cycle));
        DCN32_STEP(enable_stream_features(plan));
    }
    if (is_hdmi(plan.signal)) {
        DCN32_STEP(set_avmute(plan, false));
    }
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::enable_stream_timing(const Dcn32StreamPlan& plan) noexcept {
    // dcn20_enable_stream_timing (set_pixel_rate_div runs inside set_pixel_clock).
    const uint32_t otg = plan.pipe;
    const Dcn32Topology& t = plan.bandwidth.topology;
    uint8_t heads[dcn32_topology_pipes]{};
    const uint32_t opp_cnt = topology_opp_heads(t, heads);
    if (opp_cnt > 1) {
        DCN32_STEP(set_odm_combine(plan));
    }
    DCN32_STEP(enable_optc_clock(otg, true));
    DCN32_STEP(link(hooks_.set_pixel_clock(hooks_.context, plan), StreamFailure::clock_refused));
    DCN32_STEP(program_timing(plan));
    // No set_out_rate_control on DCN 3.2's MPC. Each OPP head: pipe clock and
    // opp2_program_left_edge_extra_pixel (0 for RGB).
    for (uint32_t i = 0; i < opp_cnt; ++i) {
        const uint32_t opp = t.pipes[heads[i]].opp;
        DCN32_IO(io_.update(r::OPP_PIPE_CONTROL::reg, opp, {{r::OPP_PIPE_CONTROL::OPP_PIPE_CLOCK_EN, 1}}));
        DCN32_IO(io_.update(r::FMT_422_CONTROL::reg, opp, {{r::FMT_422_CONTROL::FMT_LEFT_EDGE_EXTRA_PIXEL_COUNT, 0}}));
    }
    DCN32_STEP(blank_pixel_data(t, plan.timing, true));
    DCN32_STEP(enable_crtc(otg));
    DCN32_STEP(wait_for_blank_complete(t.pipes[otg].opp));
    DCN32_STEP(setup_manual_trigger(otg));
    DCN32_IO(io_.set(r::OTG_STATIC_SCREEN_CONTROL::reg, otg,
                     {{r::OTG_STATIC_SCREEN_CONTROL::OTG_STATIC_SCREEN_EVENT_MASK, 0},
                      {r::OTG_STATIC_SCREEN_CONTROL::OTG_STATIC_SCREEN_FRAME_COUNT, 2}}));
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::setup_vupdate_interrupt(const Dcn32StreamPlan& plan) noexcept {
    // dcn20_setup_vupdate_interrupt.
    int32_t start_line = vupdate_offset_from_vsync(plan);
    if (start_line < 0) {
        start_line = 0;
    }
    DCN32_IO(io_.set(r::OTG_VERTICAL_INTERRUPT2_POSITION::reg, plan.pipe,
                     {{r::OTG_VERTICAL_INTERRUPT2_POSITION::OTG_VERTICAL_INTERRUPT2_LINE_START,
                       static_cast<uint32_t>(start_line)}}));
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::apply_single_controller(const Dcn32StreamPlan& plan) noexcept {
    // dce110_apply_single_controller_ctx_to_hw. From here the pipe belongs to plan.
    current_ = plan;
    current_stream_ = true;
    current_plane_ = false;
    current_firmware_ = false;
    dpms_off_ = false;
    const uint32_t otg = plan.pipe;
    const Dcn32Topology& t = plan.bandwidth.topology;
    DCN32_STEP(program_fmt(t.pipes[otg].opp));
    DCN32_STEP(set_dyn_expansion(t.pipes[otg].opp, plan.signal));
    for (uint32_t odm = t.pipes[otg].next_odm; odm != dcn32_no_pipe; odm = t.pipes[odm].next_odm) {
        DCN32_STEP(set_dyn_expansion(t.pipes[odm].opp, plan.signal));
        DCN32_STEP(program_fmt(t.pipes[odm].opp));
    }
    DCN32_STEP(enable_stream_timing(plan));
    DCN32_STEP(setup_vupdate_interrupt(plan));
    DCN32_STEP(setup_manual_trigger(otg));
    DCN32_IO(io_.set(r::OTG_STATIC_SCREEN_CONTROL::reg, otg,
                     {{r::OTG_STATIC_SCREEN_CONTROL::OTG_STATIC_SCREEN_EVENT_MASK, 0},
                      {r::OTG_STATIC_SCREEN_CONTROL::OTG_STATIC_SCREEN_FRAME_COUNT, 2}}));
    DCN32_IO(dcn32_encoder::dig_connect_to_otg(io_, plan.engine, otg));
    DCN32_STEP(link_set_dpms_on(plan));
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::resync_fifo_dccg_dio() noexcept {
    // dcn32_resync_fifo_dccg_dio: no DPMS-off/virtual OTG to stop, then
    // dccg32_trigger_dio_fifo_resync (WDIVIDER = RDIVIDER, frequency unchanged).
    namespace D = r::DENTIST_DISPCLK_CNTL;
    uint32_t rdivider = 0;
    DCN32_IO(io_.get(D::reg, 0, D::DENTIST_DISPCLK_RDIVIDER, rdivider));
    // "Not valid for the WDIVIDER to be set to 0".
    if (rdivider != 0) {
        DCN32_IO(io_.update(D::reg, 0, {{D::DENTIST_DISPCLK_WDIVIDER, rdivider}}));
    }
    return Step::ok;
}

// ---- DSC (link_dpms.c, dcn32_hwseq.c, dcn20_dsc.c, dcn30/32 optc and encoder) ----

Dcn32StreamController::Step Dcn32StreamController::optc_set_dsc_config(uint32_t otg, uint32_t mode,
                                                                       uint32_t bytes_per_pixel,
                                                                       uint32_t slice_width) noexcept {
    // optc3_set_dsc_config -> optc2_set_dsc_config, then OTG_V_SYNC_MODE 0.
    DCN32_IO(io_.update(r::OPTC_DATA_FORMAT_CONTROL::reg, otg, {{r::OPTC_DATA_FORMAT_CONTROL::OPTC_DSC_MODE, mode}}));
    DCN32_IO(io_.set(r::OPTC_BYTES_PER_PIXEL::reg, otg,
                     {{r::OPTC_BYTES_PER_PIXEL::OPTC_DSC_BYTES_PER_PIXEL, bytes_per_pixel}}));
    DCN32_IO(io_.update(r::OPTC_WIDTH_CONTROL::reg, otg,
                        {{r::OPTC_WIDTH_CONTROL::OPTC_DSC_SLICE_WIDTH, slice_width}}));
    DCN32_IO(io_.update(r::OTG_V_SYNC_A_CNTL::reg, otg, {{r::OTG_V_SYNC_A_CNTL::OTG_V_SYNC_MODE, 0}}));
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::set_dsc_on_stream(const Dcn32StreamPlan& plan,
                                                                     bool enable) noexcept {
    // link_set_dsc_on_stream. dccg32 has no set_dto_dscclk/set_ref_dscclk, so
    // DSCCLK stays on refclk. Each ODM slice's DSC engine is the OPP head's.
    const Dcn32Topology& t = plan.bandwidth.topology;
    const uint32_t otg = plan.pipe;
    if (enable) {
        for (uint32_t pipe = otg; pipe != dcn32_no_pipe; pipe = t.pipes[pipe].next_odm) {
            // dsc2_set_config (dsc_prepare_config + dsc_write_to_registers), dsc2_enable.
            DCN32_IO(dcn32_dsc::write_registers(io_, pipe, plan.dsc_registers));
            DCN32_IO(dcn32_dsc::enable(io_, pipe, t.pipes[pipe].opp));
        }
        // Encoder (DP 8b/10b), then OPTC.
        DCN32_IO(dcn32_encoder::dp_set_dsc_config(io_, plan.engine, true));
        DCN32_STEP(optc_set_dsc_config(otg, optc_dsc_enabled_444, plan.dsc_optc.bytes_per_pixel,
                                       plan.dsc_optc.slice_width));
        return Step::ok;
    }
    DCN32_STEP(optc_set_dsc_config(otg, 0, 0, 0));
    DCN32_IO(dcn32_encoder::dp_set_dsc_config(io_, plan.engine, false));
    DCN32_IO(dcn32_encoder::dp_set_dsc_pps_info_packet(io_, plan.engine, false, nullptr, true));
    for (uint32_t pipe = otg; pipe != dcn32_no_pipe; pipe = t.pipes[pipe].next_odm) {
        DCN32_IO(dcn32_dsc::disconnect(io_, pipe));
        DCN32_IO(dcn32_dsc::disable(io_, pipe));
    }
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::update_dsc_on_stream(const Dcn32StreamPlan& plan) noexcept {
    // dcn32_update_dsc_on_stream(pipe_ctx, true): only while the DSC was left
    // forwarding (dsc2_read_state dsc_fw_en), reprogram every ODM slice's DSC
    // and the OPTC (the encoder keeps its configuration).
    const Dcn32Topology& t = plan.bandwidth.topology;
    const uint32_t otg = plan.pipe;
    bool forward_enabled = false;
    DCN32_IO(dcn32_dsc::read_state(io_, otg, forward_enabled));
    if (!forward_enabled) {
        return Step::ok;
    }
    for (uint32_t pipe = otg; pipe != dcn32_no_pipe; pipe = t.pipes[pipe].next_odm) {
        DCN32_IO(dcn32_dsc::write_registers(io_, pipe, plan.dsc_registers));
        DCN32_IO(dcn32_dsc::enable(io_, pipe, t.pipes[pipe].opp));
    }
    DCN32_STEP(optc_set_dsc_config(otg, optc_dsc_enabled_444, plan.dsc_optc.bytes_per_pixel,
                                   plan.dsc_optc.slice_width));
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::set_dsc_pps_packet(const Dcn32StreamPlan& plan, bool enable,
                                                                      bool immediate_update) noexcept {
    // link_set_dsc_pps_packet (dsc2_get_packed_pps of the whole picture).
    DCN32_IO(dcn32_encoder::dp_set_dsc_pps_info_packet(io_, plan.engine, enable, enable ? plan.dsc_pps : nullptr,
                                                       immediate_update));
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::set_dsc_on_rx(const Dcn32StreamPlan& plan, bool enable) noexcept {
    // dp_set_dsc_on_rx -> dm_helpers_dp_write_dsc_enable (SST: DP_DSC_ENABLE).
    // Linux ignores the result; DEVIATION for enable: a sink that did not
    // take DSC on would get an undecodable stream, so a refusal fails here.
    (void)plan;
    if (hooks_.dpcd_write == nullptr) {
        return enable ? fail(StreamFailure::link_refused) : Step::ok;
    }
    const LinkCall call = hooks_.dpcd_write(hooks_.context, dpcd_dsc_enable, enable ? 1u : 0u);
    return enable ? link(call, StreamFailure::link_refused) : ignored_result(call);
}

Dcn32StreamController::Step Dcn32StreamController::update_odm(const Dcn32StreamPlan& plan) noexcept {
    // dcn32_update_odm for the OTG master of plan.
    const Dcn32Topology& t = plan.bandwidth.topology;
    const uint32_t otg = plan.pipe;
    if (plan.odm_slices > 1) {
        DCN32_STEP(set_odm_combine(plan));
    } else {
        DCN32_STEP(set_odm_bypass(otg));
    }
    for (uint32_t odm = t.pipes[otg].next_odm; odm != dcn32_no_pipe; odm = t.pipes[odm].next_odm) {
        DCN32_IO(io_.update(r::OPP_PIPE_CONTROL::reg, t.pipes[odm].opp, {{r::OPP_PIPE_CONTROL::OPP_PIPE_CLOCK_EN, 1}}));
        DCN32_IO(io_.update(r::FMT_422_CONTROL::reg, t.pipes[odm].opp,
                            {{r::FMT_422_CONTROL::FMT_LEFT_EDGE_EXTRA_PIXEL_COUNT, 0}}));
    }
    if (t.pipes[otg].dsc) {
        if (plan.dsc) {
            DCN32_STEP(update_dsc_on_stream(plan));
        }
        // No longer using a pipe for ODM: disconnect that pipe's DSC (the
        // current state's OTG master links, which the back end reset kept).
        const uint8_t old_next = live_.pipes[otg].next_odm;
        if (t.pipes[otg].next_odm == dcn32_no_pipe && old_next != dcn32_no_pipe && live_.pipes[old_next].dsc) {
            DCN32_IO(dcn32_dsc::disconnect(io_, old_next));
        }
    }
    // The blank pattern is generated by the OPP: reprogram it for an OTG
    // master without a plane.
    if (!t.pipes[otg].has_plane) {
        DCN32_STEP(blank_pixel_data(t, plan.timing, true));
    }
    return Step::ok;
}

// ---- front end (dcn20_program_front_end_for_ctx / dcn20_program_pipe) ----

Dcn32StreamController::Step Dcn32StreamController::hubp_setup(const Dcn32StreamPlan& plan, uint32_t pipe) noexcept {
    // hubp3_setup: hubp2_vready_at_or_After_vsync, hubp21_program_requestor,
    // hubp3_program_deadline (the pipe's pipe_dlg_param, dlg, ttu and rq).
    const uint32_t hubp = pipe;
    const Dcn32PipeBandwidth& bw = pipe_bandwidth(plan, pipe);
    DCN32_IO(io_.write(r::HUBPREQ_DEBUG_DB::reg, hubp, 1u << 8));
    uint32_t vready = 0;
    const uint32_t htotal = plan.timing.h_total;
    if (htotal != 0) {
        vready = (bw.vstartup_start - (bw.vready_offset + bw.vupdate_width + bw.vupdate_offset) / htotal) <=
                 plan.vblank_end ? 1u : 0u;
    }
    DCN32_IO(io_.update(r::DCHUBP_CNTL::reg, hubp, {{r::DCHUBP_CNTL::HUBP_VREADY_AT_OR_AFTER_VSYNC, vready}}));

    const Dcn32RqRegs& rq = bw.rq;
    DCN32_IO(io_.update(r::HUBPRET_CONTROL::reg, hubp,
                        {{r::HUBPRET_CONTROL::DET_BUF_PLANE1_BASE_ADDRESS, rq.plane1_base_address}}));
    namespace X = r::DCN_EXPANSION_MODE;
    DCN32_IO(io_.set(X::reg, hubp, {{X::DRQ_EXPANSION_MODE, rq.drq_expansion_mode},
                                    {X::PRQ_EXPANSION_MODE, rq.prq_expansion_mode},
                                    {X::MRQ_EXPANSION_MODE, rq.mrq_expansion_mode},
                                    {X::CRQ_EXPANSION_MODE, rq.crq_expansion_mode}}));
    namespace L = r::DCHUBP_REQ_SIZE_CONFIG;
    DCN32_IO(io_.set(L::reg, hubp, {{L::CHUNK_SIZE, rq.rq_regs_l.chunk_size},
                                    {L::MIN_CHUNK_SIZE, rq.rq_regs_l.min_chunk_size},
                                    {L::META_CHUNK_SIZE, rq.rq_regs_l.meta_chunk_size},
                                    {L::MIN_META_CHUNK_SIZE, rq.rq_regs_l.min_meta_chunk_size},
                                    {L::DPTE_GROUP_SIZE, rq.rq_regs_l.dpte_group_size},
                                    {L::VM_GROUP_SIZE, rq.rq_regs_l.mpte_group_size},
                                    {L::SWATH_HEIGHT, rq.rq_regs_l.swath_height},
                                    {L::PTE_ROW_HEIGHT_LINEAR, rq.rq_regs_l.pte_row_height_linear}}));
    namespace LC = r::DCHUBP_REQ_SIZE_CONFIG_C;
    DCN32_IO(io_.set(LC::reg, hubp, {{LC::CHUNK_SIZE_C, rq.rq_regs_c.chunk_size},
                                     {LC::MIN_CHUNK_SIZE_C, rq.rq_regs_c.min_chunk_size},
                                     {LC::META_CHUNK_SIZE_C, rq.rq_regs_c.meta_chunk_size},
                                     {LC::MIN_META_CHUNK_SIZE_C, rq.rq_regs_c.min_meta_chunk_size},
                                     {LC::DPTE_GROUP_SIZE_C, rq.rq_regs_c.dpte_group_size},
                                     {LC::SWATH_HEIGHT_C, rq.rq_regs_c.swath_height},
                                     {LC::PTE_ROW_HEIGHT_LINEAR_C, rq.rq_regs_c.pte_row_height_linear}}));

    // hubp2_program_deadline.
    const Dcn32DlgRegs& dlg = bw.dlg;
    const Dcn32TtuRegs& ttu = bw.ttu;
    DCN32_IO(io_.set(r::BLANK_OFFSET_0::reg, hubp, {{r::BLANK_OFFSET_0::REFCYC_H_BLANK_END, dlg.refcyc_h_blank_end},
                                                    {r::BLANK_OFFSET_0::DLG_V_BLANK_END, dlg.dlg_vblank_end}}));
    DCN32_IO(io_.set(r::BLANK_OFFSET_1::reg, hubp, {{r::BLANK_OFFSET_1::MIN_DST_Y_NEXT_START, dlg.min_dst_y_next_start}}));
    DCN32_IO(io_.set(r::DST_DIMENSIONS::reg, hubp, {{r::DST_DIMENSIONS::REFCYC_PER_HTOTAL, dlg.refcyc_per_htotal}}));
    DCN32_IO(io_.set(r::DST_AFTER_SCALER::reg, hubp,
                     {{r::DST_AFTER_SCALER::REFCYC_X_AFTER_SCALER, dlg.refcyc_x_after_scaler},
                      {r::DST_AFTER_SCALER::DST_Y_AFTER_SCALER, dlg.dst_y_after_scaler}}));
    DCN32_IO(io_.set(r::REF_FREQ_TO_PIX_FREQ::reg, hubp,
                     {{r::REF_FREQ_TO_PIX_FREQ::REF_FREQ_TO_PIX_FREQ, dlg.ref_freq_to_pix_freq}}));
    DCN32_IO(io_.set(r::VBLANK_PARAMETERS_1::reg, hubp,
                     {{r::VBLANK_PARAMETERS_1::REFCYC_PER_PTE_GROUP_VBLANK_L, dlg.refcyc_per_pte_group_vblank_l}}));
    DCN32_IO(io_.set(r::NOM_PARAMETERS_0::reg, hubp,
                     {{r::NOM_PARAMETERS_0::DST_Y_PER_PTE_ROW_NOM_L, dlg.dst_y_per_pte_row_nom_l}}));
    DCN32_IO(io_.set(r::NOM_PARAMETERS_1::reg, hubp,
                     {{r::NOM_PARAMETERS_1::REFCYC_PER_PTE_GROUP_NOM_L, dlg.refcyc_per_pte_group_nom_l}}));
    DCN32_IO(io_.set(r::NOM_PARAMETERS_4::reg, hubp,
                     {{r::NOM_PARAMETERS_4::DST_Y_PER_META_ROW_NOM_L, dlg.dst_y_per_meta_row_nom_l}}));
    DCN32_IO(io_.set(r::NOM_PARAMETERS_5::reg, hubp,
                     {{r::NOM_PARAMETERS_5::REFCYC_PER_META_CHUNK_NOM_L, dlg.refcyc_per_meta_chunk_nom_l}}));
    DCN32_IO(io_.set(r::PER_LINE_DELIVERY::reg, hubp,
                     {{r::PER_LINE_DELIVERY::REFCYC_PER_LINE_DELIVERY_L, dlg.refcyc_per_line_delivery_l},
                      {r::PER_LINE_DELIVERY::REFCYC_PER_LINE_DELIVERY_C, dlg.refcyc_per_line_delivery_c}}));
    DCN32_IO(io_.set(r::VBLANK_PARAMETERS_2::reg, hubp,
                     {{r::VBLANK_PARAMETERS_2::REFCYC_PER_PTE_GROUP_VBLANK_C, dlg.refcyc_per_pte_group_vblank_c}}));
    DCN32_IO(io_.set(r::NOM_PARAMETERS_2::reg, hubp,
                     {{r::NOM_PARAMETERS_2::DST_Y_PER_PTE_ROW_NOM_C, dlg.dst_y_per_pte_row_nom_c}}));
    DCN32_IO(io_.set(r::NOM_PARAMETERS_3::reg, hubp,
                     {{r::NOM_PARAMETERS_3::REFCYC_PER_PTE_GROUP_NOM_C, dlg.refcyc_per_pte_group_nom_c}}));
    DCN32_IO(io_.set(r::NOM_PARAMETERS_6::reg, hubp,
                     {{r::NOM_PARAMETERS_6::DST_Y_PER_META_ROW_NOM_C, dlg.dst_y_per_meta_row_nom_c}}));
    DCN32_IO(io_.set(r::NOM_PARAMETERS_7::reg, hubp,
                     {{r::NOM_PARAMETERS_7::REFCYC_PER_META_CHUNK_NOM_C, dlg.refcyc_per_meta_chunk_nom_c}}));
    DCN32_IO(io_.set(r::DCN_TTU_QOS_WM::reg, hubp, {{r::DCN_TTU_QOS_WM::QoS_LEVEL_LOW_WM, ttu.qos_level_low_wm},
                                                    {r::DCN_TTU_QOS_WM::QoS_LEVEL_HIGH_WM, ttu.qos_level_high_wm}}));
    namespace T0 = r::DCN_SURF0_TTU_CNTL0;
    DCN32_IO(io_.set(T0::reg, hubp, {{T0::REFCYC_PER_REQ_DELIVERY, ttu.refcyc_per_req_delivery_l},
                                     {T0::QoS_LEVEL_FIXED, ttu.qos_level_fixed_l},
                                     {T0::QoS_RAMP_DISABLE, ttu.qos_ramp_disable_l}}));
    namespace T1 = r::DCN_SURF1_TTU_CNTL0;
    DCN32_IO(io_.set(T1::reg, hubp, {{T1::REFCYC_PER_REQ_DELIVERY, ttu.refcyc_per_req_delivery_c},
                                     {T1::QoS_LEVEL_FIXED, ttu.qos_level_fixed_c},
                                     {T1::QoS_RAMP_DISABLE, ttu.qos_ramp_disable_c}}));
    namespace C0 = r::DCN_CUR0_TTU_CNTL0;
    DCN32_IO(io_.set(C0::reg, hubp, {{C0::REFCYC_PER_REQ_DELIVERY, ttu.refcyc_per_req_delivery_cur0},
                                     {C0::QoS_LEVEL_FIXED, ttu.qos_level_fixed_cur0},
                                     {C0::QoS_RAMP_DISABLE, ttu.qos_ramp_disable_cur0}}));
    DCN32_IO(io_.set(r::FLIP_PARAMETERS_1::reg, hubp,
                     {{r::FLIP_PARAMETERS_1::REFCYC_PER_PTE_GROUP_FLIP_L, dlg.refcyc_per_pte_group_flip_l}}));
    DCN32_IO(io_.update(r::DCN_DMDATA_VM_CNTL::reg, hubp,
                        {{r::DCN_DMDATA_VM_CNTL::REFCYC_PER_VM_DMDATA, dlg.refcyc_per_vm_dmdata}}));
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::hubp_setup_interdependent(const Dcn32StreamPlan& plan,
                                                                            uint32_t pipe) noexcept {
    // hubp2_setup_interdependent.
    const uint32_t hubp = pipe;
    const Dcn32DlgRegs& dlg = pipe_bandwidth(plan, pipe).dlg;
    const Dcn32TtuRegs& ttu = pipe_bandwidth(plan, pipe).ttu;
    DCN32_IO(io_.set(r::PREFETCH_SETTINGS::reg, hubp, {{r::PREFETCH_SETTINGS::DST_Y_PREFETCH, dlg.dst_y_prefetch},
                                                       {r::PREFETCH_SETTINGS::VRATIO_PREFETCH, dlg.vratio_prefetch}}));
    DCN32_IO(io_.set(r::PREFETCH_SETTINGS_C::reg, hubp,
                     {{r::PREFETCH_SETTINGS_C::VRATIO_PREFETCH_C, dlg.vratio_prefetch_c}}));
    DCN32_IO(io_.set(r::VBLANK_PARAMETERS_0::reg, hubp,
                     {{r::VBLANK_PARAMETERS_0::DST_Y_PER_VM_VBLANK, dlg.dst_y_per_vm_vblank},
                      {r::VBLANK_PARAMETERS_0::DST_Y_PER_ROW_VBLANK, dlg.dst_y_per_row_vblank}}));
    DCN32_IO(io_.set(r::FLIP_PARAMETERS_0::reg, hubp, {{r::FLIP_PARAMETERS_0::DST_Y_PER_VM_FLIP, dlg.dst_y_per_vm_flip},
                                                       {r::FLIP_PARAMETERS_0::DST_Y_PER_ROW_FLIP, dlg.dst_y_per_row_flip}}));
    DCN32_IO(io_.set(r::VBLANK_PARAMETERS_3::reg, hubp,
                     {{r::VBLANK_PARAMETERS_3::REFCYC_PER_META_CHUNK_VBLANK_L, dlg.refcyc_per_meta_chunk_vblank_l}}));
    DCN32_IO(io_.set(r::VBLANK_PARAMETERS_4::reg, hubp,
                     {{r::VBLANK_PARAMETERS_4::REFCYC_PER_META_CHUNK_VBLANK_C, dlg.refcyc_per_meta_chunk_vblank_c}}));
    DCN32_IO(io_.set(r::FLIP_PARAMETERS_2::reg, hubp,
                     {{r::FLIP_PARAMETERS_2::REFCYC_PER_META_CHUNK_FLIP_L, dlg.refcyc_per_meta_chunk_flip_l}}));
    DCN32_IO(io_.set(r::PER_LINE_DELIVERY_PRE::reg, hubp,
                     {{r::PER_LINE_DELIVERY_PRE::REFCYC_PER_LINE_DELIVERY_PRE_L, dlg.refcyc_per_line_delivery_pre_l},
                      {r::PER_LINE_DELIVERY_PRE::REFCYC_PER_LINE_DELIVERY_PRE_C, dlg.refcyc_per_line_delivery_pre_c}}));
    DCN32_IO(io_.set(r::DCN_SURF0_TTU_CNTL1::reg, hubp,
                     {{r::DCN_SURF0_TTU_CNTL1::REFCYC_PER_REQ_DELIVERY_PRE, ttu.refcyc_per_req_delivery_pre_l}}));
    DCN32_IO(io_.set(r::DCN_SURF1_TTU_CNTL1::reg, hubp,
                     {{r::DCN_SURF1_TTU_CNTL1::REFCYC_PER_REQ_DELIVERY_PRE, ttu.refcyc_per_req_delivery_pre_c}}));
    DCN32_IO(io_.set(r::DCN_CUR0_TTU_CNTL1::reg, hubp,
                     {{r::DCN_CUR0_TTU_CNTL1::REFCYC_PER_REQ_DELIVERY_PRE, ttu.refcyc_per_req_delivery_pre_cur0}}));
    DCN32_IO(io_.set(r::DCN_CUR1_TTU_CNTL1::reg, hubp,
                     {{r::DCN_CUR1_TTU_CNTL1::REFCYC_PER_REQ_DELIVERY_PRE, ttu.refcyc_per_req_delivery_pre_cur1}}));
    DCN32_IO(io_.set(r::DCN_GLOBAL_TTU_CNTL::reg, hubp, {{r::DCN_GLOBAL_TTU_CNTL::MIN_TTU_VBLANK, ttu.min_ttu_vblank},
                                                         {r::DCN_GLOBAL_TTU_CNTL::QoS_LEVEL_FLIP, ttu.qos_level_flip}}));
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::dpp_setup(uint32_t dpp) noexcept {
    // dpp3_cnv_setup(SURFACE_PIXEL_FORMAT_GRPH_ARGB8888, EXPANSION_MODE_ZERO,
    // no input CSC adjustment, COLOR_SPACE_SRGB, no 2-bit alpha LUT).
    namespace F = r::FORMAT_CONTROL;
    DCN32_IO(io_.set(F::reg, dpp, {{F::CNVC_BYPASS, 0}, {F::FORMAT_EXPANSION_MODE, expansion_mode_zero}}));
    DCN32_IO(io_.update(F::reg, dpp, {{F::FORMAT_CNV16, 0}}));
    DCN32_IO(io_.update(F::reg, dpp, {{F::CNVC_BYPASS_MSB_ALIGN, 0}}));
    DCN32_IO(io_.update(F::reg, dpp, {{F::CLAMP_POSITIVE, 0}}));
    DCN32_IO(io_.update(F::reg, dpp, {{F::CLAMP_POSITIVE_C, 0}}));
    DCN32_IO(io_.update(F::reg, dpp, {{F::FORMAT_CROSSBAR_R, 0}}));
    DCN32_IO(io_.update(F::reg, dpp, {{F::FORMAT_CROSSBAR_G, 1}}));
    DCN32_IO(io_.update(F::reg, dpp, {{F::FORMAT_CROSSBAR_B, 2}}));
    namespace P = r::CNVC_SURFACE_PIXEL_FORMAT;
    DCN32_IO(io_.set(P::reg, dpp, {{P::CNVC_SURFACE_PIXEL_FORMAT, 8}, {P::CNVC_ALPHA_PLANE_ENABLE, 0}}));
    DCN32_IO(io_.update(F::reg, dpp, {{F::ALPHA_EN, 1}}));
    DCN32_IO(io_.set(r::PRE_DEALPHA::reg, dpp, {{r::PRE_DEALPHA::PRE_DEALPHA_EN, 0},
                                                {r::PRE_DEALPHA::PRE_DEALPHA_ABLND_EN, 0}}));
    DCN32_IO(io_.set(r::PRE_REALPHA::reg, dpp, {{r::PRE_REALPHA::PRE_REALPHA_EN, 0},
                                                {r::PRE_REALPHA::PRE_REALPHA_ABLND_EN, 0}}));
    // dpp3_program_post_csc(INPUT_CSC_SELECT_BYPASS).
    DCN32_IO(io_.set(r::CM_POST_CSC_CONTROL::reg, dpp, {{r::CM_POST_CSC_CONTROL::CM_POST_CSC_MODE, 0}}));
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::update_blending(const Dcn32StreamPlan& plan,
                                                                  uint32_t pipe) noexcept {
    // mpc2_update_blending with dcn20_update_mpcc's blnd_cfg for the plane,
    // then dcn10_update_visual_confirm_color -> mpc1_set_bg_color(black),
    // which programs the bottommost MPCC below this one.
    const Dcn32PlaneScanout& plane = plane_of(plan, pipe);
    const uint32_t mpcc = pipe;
    uint32_t alpha_mode = mpcc_alpha_global;
    uint32_t pre_multiplied = 0;
    uint32_t global_gain = 0xff;
    if (plane.per_pixel_alpha) {
        pre_multiplied = plane.pre_multiplied_alpha ? 1u : 0u;
        if (plane.global_alpha) {
            alpha_mode = mpcc_alpha_per_pixel_global_gain;
            global_gain = plane.global_alpha_value;
        } else {
            alpha_mode = mpcc_alpha_per_pixel;
        }
    }
    const uint32_t global_alpha = plane.global_alpha ? plane.global_alpha_value : 0xffu;
    namespace C = r::MPCC_CONTROL;
    DCN32_IO(io_.update(C::reg, mpcc, {{C::MPCC_ALPHA_BLND_MODE, alpha_mode}, {C::MPCC_ALPHA_MULTIPLIED_MODE, pre_multiplied},
                                       {C::MPCC_BLND_ACTIVE_OVERLAP_ONLY, 0}, {C::MPCC_GLOBAL_ALPHA, global_alpha},
                                       {C::MPCC_GLOBAL_GAIN, global_gain}, {C::MPCC_BG_BPC, 4}, {C::MPCC_BOT_GAIN_MODE, 0}}));
    DCN32_IO(io_.set(r::MPCC_TOP_GAIN::reg, mpcc, {{r::MPCC_TOP_GAIN::MPCC_TOP_GAIN, gain_one}}));
    DCN32_IO(io_.set(r::MPCC_BOT_GAIN_INSIDE::reg, mpcc, {{r::MPCC_BOT_GAIN_INSIDE::MPCC_BOT_GAIN_INSIDE, gain_one}}));
    DCN32_IO(io_.set(r::MPCC_BOT_GAIN_OUTSIDE::reg, mpcc,
                     {{r::MPCC_BOT_GAIN_OUTSIDE::MPCC_BOT_GAIN_OUTSIDE, gain_one}}));
    uint32_t bottommost = mpcc;
    uint32_t guard = 0;
    while (mpcc_bot_[bottommost] != mpcc_none && mpcc_bot_[bottommost] != bottommost && guard++ < dcn32_stream_pipes) {
        bottommost = mpcc_bot_[bottommost];
    }
    DCN32_IO(io_.set(r::MPCC_BG_R_CR::reg, bottommost, {{r::MPCC_BG_R_CR::MPCC_BG_R_CR, 0}}));
    DCN32_IO(io_.set(r::MPCC_BG_G_Y::reg, bottommost, {{r::MPCC_BG_G_Y::MPCC_BG_G_Y, 0}}));
    DCN32_IO(io_.set(r::MPCC_BG_B_CB::reg, bottommost, {{r::MPCC_BG_B_CB::MPCC_BG_B_CB, 0}}));
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::update_mpcc(const Dcn32StreamPlan& plan, uint32_t pipe,
                                                               const PipeFlags& flags, UpdateKind kind) noexcept {
    // dcn20_update_mpcc: without a full plane update or an MPCC change only the
    // blending; otherwise remove the DPP's MPCC from the OPP tree if it is in
    // use and mpc1_insert_plane it at the bottom (insert_above_mpcc NULL).
    const uint32_t mpcc = pipe;
    const uint32_t opp = topo(plan, pipe).opp;
    if (!kind.plane_all && !flags.mpcc) {
        return update_blending(plan, pipe);
    }
    uint32_t found = mpc_tree_top_[opp];
    uint32_t guard = 0;
    while (found != mpcc_none && mpcc_dpp_[found] != mpcc && guard++ < dcn32_stream_pipes) {
        found = mpcc_bot_[found] == found ? mpcc_none : mpcc_bot_[found];
    }
    if (found != mpcc_none) {
        DCN32_STEP(remove_mpcc(opp, found));
    }
    // mpc1_insert_plane.
    mpcc_dpp_[mpcc] = pipe;
    mpcc_bot_[mpcc] = mpcc_none;
    DCN32_IO(io_.set(r::MPCC_BOT_SEL::reg, mpcc, {{r::MPCC_BOT_SEL::MPCC_BOT_SEL, mpcc_none}}));
    DCN32_IO(io_.update(r::MPCC_CONTROL::reg, mpcc, {{r::MPCC_CONTROL::MPCC_MODE, mpcc_mode_top_layer_only}}));
    DCN32_IO(io_.set(r::MPCC_TOP_SEL::reg, mpcc, {{r::MPCC_TOP_SEL::MPCC_TOP_SEL, pipe}}));
    DCN32_IO(io_.set(r::MPCC_OPP_ID::reg, mpcc, {{r::MPCC_OPP_ID::MPCC_OPP_ID, opp}}));
    DCN32_IO(io_.set(r::MPCC_UPDATE_LOCK_SEL::reg, mpcc, {{r::MPCC_UPDATE_LOCK_SEL::MPCC_UPDATE_LOCK_SEL, opp}}));
    if (mpc_tree_top_[opp] == mpcc_none) {
        // Insert the toppest MPCC.
        mpc_tree_top_[opp] = mpcc;
        DCN32_IO(io_.update(r::MPC_OUT_MUX::reg, opp, {{r::MPC_OUT_MUX::MPC_OUT_MUX, mpcc}}));
    } else {
        uint32_t tail = mpc_tree_top_[opp];
        guard = 0;
        while (mpcc_bot_[tail] != mpcc_none && guard++ < dcn32_stream_pipes) {
            tail = mpcc_bot_[tail];
        }
        DCN32_IO(io_.set(r::MPCC_BOT_SEL::reg, tail, {{r::MPCC_BOT_SEL::MPCC_BOT_SEL, mpcc}}));
        mpcc_bot_[tail] = mpcc;
        DCN32_IO(io_.update(r::MPCC_CONTROL::reg, tail, {{r::MPCC_CONTROL::MPCC_MODE, mpcc_mode_top_bot_blending}}));
    }
    return update_blending(plan, pipe);
}

Dcn32StreamController::Step Dcn32StreamController::dpp_set_scaler(const Dcn32StreamPlan& plan, uint32_t pipe) noexcept {
    // dpp1_dscl_set_scaler_manual_scale at 1:1: DSCL_MODE_SCALING_444_BYPASS,
    // use_max_lb -> LB_MEMORY_CONFIG_0, float format LB, 31 partitions;
    // lb alpha_en is the plane's per_pixel_alpha. The memcmp against the
    // cached scaler data skips an identical reprogram.
    const uint32_t dpp = pipe;
    const Dcn32TopologyPipe& t = topo(plan, pipe);
    uint32_t slice_x = 0;
    uint32_t slice_width = 0;
    odm_slice(plan.bandwidth.topology, plan.timing.width, topology_opp_head(plan.bandwidth.topology, pipe), slice_x,
              slice_width);
    ScalerCache next{};
    next.valid = true;
    next.recout = t.recout;
    next.viewport = t.viewport;
    next.h_active = slice_width;
    next.v_active = plan.timing.height;
    next.alpha_en = plane_of(plan, pipe).per_pixel_alpha;
    const ScalerCache& cached = scl_[dpp];
    if (cached.valid && cached.recout.x == next.recout.x && cached.recout.y == next.recout.y &&
        cached.recout.width == next.recout.width && cached.recout.height == next.recout.height &&
        cached.viewport.x == next.viewport.x && cached.viewport.y == next.viewport.y &&
        cached.viewport.width == next.viewport.width && cached.viewport.height == next.viewport.height &&
        cached.h_active == next.h_active && cached.v_active == next.v_active && cached.alpha_en == next.alpha_en) {
        return Step::ok;
    }
    scl_[dpp] = next;
    DCN32_IO(io_.set(r::DSCL_AUTOCAL::reg, dpp, {{r::DSCL_AUTOCAL::AUTOCAL_MODE, 0},
                                                 {r::DSCL_AUTOCAL::AUTOCAL_NUM_PIPE, 0},
                                                 {r::DSCL_AUTOCAL::AUTOCAL_PIPE_ID, 0}}));
    DCN32_IO(io_.set(r::DSCL_CONTROL::reg, dpp, {{r::DSCL_CONTROL::SCL_BOUNDARY_MODE, 0}}));
    DCN32_IO(io_.set(r::RECOUT_START::reg, dpp, {{r::RECOUT_START::RECOUT_START_X, static_cast<uint32_t>(t.recout.x)},
                                                 {r::RECOUT_START::RECOUT_START_Y, static_cast<uint32_t>(t.recout.y)}}));
    DCN32_IO(io_.set(r::RECOUT_SIZE::reg, dpp,
                     {{r::RECOUT_SIZE::RECOUT_WIDTH, static_cast<uint32_t>(t.recout.width)},
                      {r::RECOUT_SIZE::RECOUT_HEIGHT, static_cast<uint32_t>(t.recout.height)}}));
    DCN32_IO(io_.set(r::MPC_SIZE::reg, dpp, {{r::MPC_SIZE::MPC_WIDTH, next.h_active},
                                             {r::MPC_SIZE::MPC_HEIGHT, next.v_active}}));
    DCN32_IO(io_.update(r::SCL_MODE::reg, dpp, {{r::SCL_MODE::DSCL_MODE, 0}}));
    DCN32_IO(io_.set(r::LB_DATA_FORMAT::reg, dpp, {{r::LB_DATA_FORMAT::INTERLEAVE_EN, 0},
                                                   {r::LB_DATA_FORMAT::ALPHA_EN, next.alpha_en ? 1u : 0u}}));
    DCN32_IO(io_.set(r::LB_MEMORY_CTRL::reg, dpp, {{r::LB_MEMORY_CTRL::MEMORY_CONFIG, 0},
                                                   {r::LB_MEMORY_CTRL::LB_MAX_PARTITIONS, 31}}));
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::program_viewport(const Dcn32StreamPlan& plan,
                                                                   uint32_t pipe) noexcept {
    // min_set_viewport, viewport = viewport_c (RGB).
    const uint32_t hubp = pipe;
    const Dcn32Rect& vp = topo(plan, pipe).viewport;
    const uint32_t x = static_cast<uint32_t>(vp.x);
    const uint32_t y = static_cast<uint32_t>(vp.y);
    const uint32_t w = static_cast<uint32_t>(vp.width);
    const uint32_t h = static_cast<uint32_t>(vp.height);
    DCN32_IO(io_.set(r::DCSURF_PRI_VIEWPORT_DIMENSION::reg, hubp,
                     {{r::DCSURF_PRI_VIEWPORT_DIMENSION::PRI_VIEWPORT_WIDTH, w},
                      {r::DCSURF_PRI_VIEWPORT_DIMENSION::PRI_VIEWPORT_HEIGHT, h}}));
    DCN32_IO(io_.set(r::DCSURF_PRI_VIEWPORT_START::reg, hubp,
                     {{r::DCSURF_PRI_VIEWPORT_START::PRI_VIEWPORT_X_START, x},
                      {r::DCSURF_PRI_VIEWPORT_START::PRI_VIEWPORT_Y_START, y}}));
    DCN32_IO(io_.set(r::DCSURF_SEC_VIEWPORT_DIMENSION::reg, hubp,
                     {{r::DCSURF_SEC_VIEWPORT_DIMENSION::SEC_VIEWPORT_WIDTH, w},
                      {r::DCSURF_SEC_VIEWPORT_DIMENSION::SEC_VIEWPORT_HEIGHT, h}}));
    DCN32_IO(io_.set(r::DCSURF_SEC_VIEWPORT_START::reg, hubp,
                     {{r::DCSURF_SEC_VIEWPORT_START::SEC_VIEWPORT_X_START, x},
                      {r::DCSURF_SEC_VIEWPORT_START::SEC_VIEWPORT_Y_START, y}}));
    DCN32_IO(io_.set(r::DCSURF_PRI_VIEWPORT_DIMENSION_C::reg, hubp,
                     {{r::DCSURF_PRI_VIEWPORT_DIMENSION_C::PRI_VIEWPORT_WIDTH_C, w},
                      {r::DCSURF_PRI_VIEWPORT_DIMENSION_C::PRI_VIEWPORT_HEIGHT_C, h}}));
    DCN32_IO(io_.set(r::DCSURF_PRI_VIEWPORT_START_C::reg, hubp,
                     {{r::DCSURF_PRI_VIEWPORT_START_C::PRI_VIEWPORT_X_START_C, x},
                      {r::DCSURF_PRI_VIEWPORT_START_C::PRI_VIEWPORT_Y_START_C, y}}));
    DCN32_IO(io_.set(r::DCSURF_SEC_VIEWPORT_DIMENSION_C::reg, hubp,
                     {{r::DCSURF_SEC_VIEWPORT_DIMENSION_C::SEC_VIEWPORT_WIDTH_C, w},
                      {r::DCSURF_SEC_VIEWPORT_DIMENSION_C::SEC_VIEWPORT_HEIGHT_C, h}}));
    DCN32_IO(io_.set(r::DCSURF_SEC_VIEWPORT_START_C::reg, hubp,
                     {{r::DCSURF_SEC_VIEWPORT_START_C::SEC_VIEWPORT_X_START_C, x},
                      {r::DCSURF_SEC_VIEWPORT_START_C::SEC_VIEWPORT_Y_START_C, y}}));
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::program_gamut_and_output_csc(const Dcn32StreamPlan& plan,
                                                                               uint32_t pipe) noexcept {
    // dcn30_program_gamut_remap (DPP and MPC bypass), then
    // dcn20_program_output_csc -> mpc3_power_on_ogam_lut (the pipe's MPCC) +
    // mpc3_set_ocsc_default on hubp->opp_id (COLOR_SPACE_SRGB ->
    // COLOR_SPACE_RGB_TYPE identity, MPC_OUTPUT_CSC_COEF_A).
    const uint32_t mpcc = pipe;
    const uint32_t opp = topo(plan, pipe).opp;
    DCN32_IO(io_.set(r::CM_GAMUT_REMAP_CONTROL::reg, pipe, {{r::CM_GAMUT_REMAP_CONTROL::CM_GAMUT_REMAP_MODE, 0}}));
    DCN32_IO(io_.set(r::MPCC_GAMUT_REMAP_MODE::reg, mpcc, {{r::MPCC_GAMUT_REMAP_MODE::MPCC_GAMUT_REMAP_MODE, 0}}));
    DCN32_IO(io_.update(r::MPCC_MEM_PWR_CTRL::reg, mpcc, {{r::MPCC_MEM_PWR_CTRL::MPCC_OGAM_MEM_PWR_DIS, 1}}));
    DCN32_IO(io_.wait(r::MPCC_MEM_PWR_CTRL::reg, mpcc, r::MPCC_MEM_PWR_CTRL::MPCC_OGAM_MEM_PWR_STATE, 0, 10, 10));
    DCN32_IO(io_.write(r::MPC_OUT_CSC_COEF_FORMAT::reg, 0, 0));
    DCN32_IO(io_.set(r::MPC_OUT_CSC_MODE::reg, opp, {{r::MPC_OUT_CSC_MODE::MPC_OCSC_MODE, ocsc_coef_a}}));
    // cm_helper_program_color_matrices: six consecutive registers, each with
    // the C11/C12 field layout, values {0x2000,0,0,0, 0,0x2000,0,0, 0,0,0x2000,0}.
    static const uint16_t identity[12] = {0x2000, 0, 0, 0, 0, 0x2000, 0, 0, 0, 0, 0x2000, 0};
    const r::Register* csc[6] = {&r::MPC_OUT_CSC_C11_C12_A::reg, &r::MPC_OUT_CSC_C13_C14_A::reg,
                                 &r::MPC_OUT_CSC_C21_C22_A::reg, &r::MPC_OUT_CSC_C23_C24_A::reg,
                                 &r::MPC_OUT_CSC_C31_C32_A::reg, &r::MPC_OUT_CSC_C33_C34_A::reg};
    for (uint32_t index = 0; index < 6; ++index) {
        DCN32_IO(io_.set(*csc[index], opp, {{r::MPC_OUT_CSC_C11_C12_A::MPC_OCSC_C11_A, identity[2 * index]},
                                            {r::MPC_OUT_CSC_C11_C12_A::MPC_OCSC_C12_A, identity[2 * index + 1]}}));
    }
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::program_surface_config(const Dcn32StreamPlan& plan,
                                                                         uint32_t pipe) noexcept {
    // hubp3_program_surface_config(ARGB8888, linear, no DCC, ROTATION_ANGLE_0).
    const uint32_t hubp = pipe;
    namespace S = r::DCSURF_SURFACE_CONTROL;
    DCN32_IO(io_.update(S::reg, hubp, {{S::PRIMARY_SURFACE_DCC_EN, 0}, {S::PRIMARY_SURFACE_DCC_IND_BLK, 0},
                                       {S::PRIMARY_SURFACE_DCC_IND_BLK_C, 0}, {S::SECONDARY_SURFACE_DCC_EN, 0},
                                       {S::SECONDARY_SURFACE_DCC_IND_BLK, 0}, {S::SECONDARY_SURFACE_DCC_IND_BLK_C, 0}}));
    namespace A = r::DCSURF_ADDR_CONFIG;
    DCN32_IO(io_.update(A::reg, hubp, {{A::NUM_PIPES, log_2(plan.tiling.num_pipes)},
                                       {A::PIPE_INTERLEAVE, plan.tiling.pipe_interleave_bytes},
                                       {A::MAX_COMPRESSED_FRAGS, log_2(plan.tiling.max_compressed_frags)},
                                       {A::NUM_PKRS, log_2(plan.tiling.num_pkrs)}}));
    namespace T = r::DCSURF_TILING_CONFIG;
    DCN32_IO(io_.update(T::reg, hubp, {{T::SW_MODE, 0}, {T::META_LINEAR, 0}, {T::PIPE_ALIGNED, 0}}));
    DCN32_IO(io_.update(r::DCSURF_SURFACE_PITCH::reg, hubp,
                        {{r::DCSURF_SURFACE_PITCH::PITCH, plane_of(plan, pipe).pitch_pixels - 1},
                         {r::DCSURF_SURFACE_PITCH::META_PITCH, 0}}));
    namespace C = r::DCSURF_SURFACE_CONFIG;
    DCN32_IO(io_.update(C::reg, hubp, {{C::ROTATION_ANGLE, 0}, {C::H_MIRROR_EN, 0}}));
    DCN32_IO(io_.update(r::HUBPRET_CONTROL::reg, hubp,
                        {{r::HUBPRET_CONTROL::CROSSBAR_SRC_CB_B, 2}, {r::HUBPRET_CONTROL::CROSSBAR_SRC_CR_R, 3}}));
    DCN32_IO(io_.update(C::reg, hubp, {{C::SURFACE_PIXEL_FORMAT, 8}}));
    hubp_power_gated_[hubp] = false;
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::program_surface_address(const Dcn32StreamPlan& plan,
                                                                          uint32_t pipe) noexcept {
    // dcn20_update_plane_addr -> hubp3_program_surface_flip_and_addr (vsync
    // flip, VMID 0, PLN_ADDR_TYPE_GRAPHICS, no meta, no TMZ).
    const uint32_t hubp = pipe;
    const uint64_t address = plane_of(plan, pipe).surface_address;
    namespace F = r::DCSURF_FLIP_CONTROL;
    DCN32_IO(io_.update(F::reg, hubp, {{F::SURFACE_FLIP_TYPE, 0}}));
    DCN32_IO(io_.update(r::VMID_SETTINGS_0::reg, hubp, {{r::VMID_SETTINGS_0::VMID, 0}}));
    DCN32_IO(io_.update(F::reg, hubp, {{F::SURFACE_FLIP_MODE_FOR_STEREOSYNC, 0}}));
    DCN32_IO(io_.update(F::reg, hubp, {{F::SURFACE_FLIP_IN_STEREOSYNC, 0}}));
    DCN32_IO(io_.update(r::DCSURF_SURFACE_CONTROL::reg, hubp, {{r::DCSURF_SURFACE_CONTROL::PRIMARY_SURFACE_TMZ, 0},
                                                               {r::DCSURF_SURFACE_CONTROL::PRIMARY_META_SURFACE_TMZ, 0}}));
    DCN32_IO(io_.set(r::DCSURF_PRIMARY_SURFACE_ADDRESS_HIGH::reg, hubp,
                     {{r::DCSURF_PRIMARY_SURFACE_ADDRESS_HIGH::PRIMARY_SURFACE_ADDRESS_HIGH,
                       static_cast<uint32_t>(address >> 32)}}));
    DCN32_IO(io_.set(r::DCSURF_PRIMARY_SURFACE_ADDRESS::reg, hubp,
                     {{r::DCSURF_PRIMARY_SURFACE_ADDRESS::PRIMARY_SURFACE_ADDRESS, static_cast<uint32_t>(address)}}));
    hubp_request_address_[hubp] = address;
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::set_input_transfer_func(uint32_t pipe) noexcept {
    // dcn32_set_input_transfer_func with TF_TYPE_BYPASS: dpp3_set_pre_degam
    // (TRANSFER_FUNCTION_UNITY), dpp3_program_gamcor_lut(NULL), dcn32_set_mcm_luts
    // (1D LUT, shaper and 3D LUT bypass).
    DCN32_IO(io_.set(r::PRE_DEGAM::reg, pipe, {{r::PRE_DEGAM::PRE_DEGAM_MODE, 0}, {r::PRE_DEGAM::PRE_DEGAM_SELECT, 0}}));
    DCN32_IO(io_.update(r::CM_CONTROL::reg, pipe, {{r::CM_CONTROL::CM_BYPASS, 0}}));
    DCN32_IO(io_.set(r::CM_GAMCOR_CONTROL::reg, pipe, {{r::CM_GAMCOR_CONTROL::CM_GAMCOR_MODE, 0}}));
    const uint32_t mpcc = pipe;
    DCN32_IO(io_.set(r::MPCC_MCM_1DLUT_CONTROL::reg, mpcc, {{r::MPCC_MCM_1DLUT_CONTROL::MPCC_MCM_1DLUT_MODE, 0}}));
    DCN32_IO(io_.set(r::MPCC_MCM_SHAPER_CONTROL::reg, mpcc,
                     {{r::MPCC_MCM_SHAPER_CONTROL::MPCC_MCM_SHAPER_LUT_MODE, 0}}));
    DCN32_IO(io_.update(r::MPCC_MOVABLE_CM_LOCATION_CONTROL::reg, mpcc,
                        {{r::MPCC_MOVABLE_CM_LOCATION_CONTROL::MPCC_MOVABLE_CM_LOCATION_CNTL, 0}}));
    DCN32_IO(io_.update(r::MPCC_MCM_3DLUT_MODE::reg, mpcc, {{r::MPCC_MCM_3DLUT_MODE::MPCC_MCM_3DLUT_MODE, 0},
                                                            {r::MPCC_MCM_3DLUT_MODE::MPCC_MCM_3DLUT_SIZE, 1}}));
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::set_output_transfer_func(uint32_t pipe) noexcept {
    // dcn32_set_output_transfer_func: no stream 3D LUT, OGAM bypass.
    DCN32_IO(io_.set(r::MPCC_OGAM_CONTROL::reg, pipe, {{r::MPCC_OGAM_CONTROL::MPCC_OGAM_MODE, 0}}));
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::enable_plane(uint32_t pipe, uint32_t opp) noexcept {
    // dcn20_enable_plane: dcn20_power_on_plane_resources, HUBP clock, hubp3_init,
    // the pipe's OPP clock (no system aperture on a dGPU).
    DCN32_IO(io_.set(r::DC_IP_REQUEST_CNTL::reg, 0, {{r::DC_IP_REQUEST_CNTL::IP_REQUEST_EN, 1}}));
    DCN32_STEP(hubp_pg_control(pipe, true));
    DCN32_IO(io_.set(r::DC_IP_REQUEST_CNTL::reg, 0, {{r::DC_IP_REQUEST_CNTL::IP_REQUEST_EN, 0}}));
    DCN32_IO(io_.update(r::HUBP_CLK_CNTL::reg, pipe, {{r::HUBP_CLK_CNTL::HUBP_CLOCK_ENABLE, 1}}));
    DCN32_IO(io_.write(r::HUBPREQ_DEBUG::reg, pipe, 1u << 26));
    DCN32_IO(io_.update(r::OPP_PIPE_CONTROL::reg, opp, {{r::OPP_PIPE_CONTROL::OPP_PIPE_CLOCK_EN, 1}}));
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::update_dchubp_dpp(const Dcn32StreamPlan& plan, uint32_t pipe,
                                                                    const PipeFlags& flags, UpdateKind kind) noexcept {
    // dcn20_update_dchubp_dpp; kind.plane_all is a FULL surface update (every
    // plane update flag), kind.stream_all dc_commit_state_no_check (every
    // stream update flag).
    const bool plane_all = kind.plane_all;
    // dpp1_dppclk_control(false, true): dcn32 has no DPPCLK_RATE_CONTROL mask.
    if (flags.dppclk) {
        DCN32_IO(io_.update(r::DPP_CONTROL::reg, pipe, {{r::DPP_CONTROL::DPP_CLOCK_ENABLE, 1}}));
    }
    // update_dpp_dto on enable runs in the link owner's update_clocks (prepare_bandwidth).
    if (flags.hubp_rq_dlg_ttu) {
        DCN32_IO(io_.update(r::DCHUBP_CNTL::reg, pipe, {{r::DCHUBP_CNTL::HUBP_VTG_SEL, plan.pipe}}));
        DCN32_STEP(hubp_setup(plan, pipe));
    }
    if (flags.unbounded_req) {
        // hubp31_set_unbounded_requesting.
        const uint32_t unbounded = plan.bandwidth.unbounded_req ? 1u : 0u;
        DCN32_IO(io_.update(r::DCHUBP_CNTL::reg, pipe, {{r::DCHUBP_CNTL::HUBP_UNBOUNDED_REQ_MODE, unbounded}}));
        DCN32_IO(io_.update(r::CURSOR_CONTROL::reg, pipe, {{r::CURSOR_CONTROL::CURSOR_REQ_MODE, unbounded}}));
    }
    if (flags.hubp_interdependent) {
        DCN32_STEP(hubp_setup_interdependent(plan, pipe));
    }
    if (flags.enable || flags.plane_changed || plane_all) {
        DCN32_STEP(dpp_setup(pipe));
    }
    if (flags.mpcc || flags.plane_changed || plane_all) {
        DCN32_STEP(update_mpcc(plan, pipe, flags, kind));
    }
    if (flags.scaler || plane_all || kind.stream_all) {
        DCN32_STEP(dpp_set_scaler(plan, pipe));
    }
    // The context is never dc->current_state here: only the viewport flag.
    if (flags.viewport) {
        DCN32_STEP(program_viewport(plan, pipe));
    }
    // No cursor attributes on the stream (the cursor owner programs HUBP cursors).
    if (flags.enable || flags.opp_changed || flags.plane_changed || kind.stream_all || plane_all) {
        DCN32_STEP(program_gamut_and_output_csc(plan, pipe));
    }
    if (flags.enable || flags.plane_changed || flags.opp_changed || plane_all) {
        DCN32_STEP(program_surface_config(plan, pipe));
    }
    if (flags.enable || flags.plane_changed || plane_all) {
        DCN32_STEP(program_surface_address(plan, pipe));
    }
    if (flags.enable) {
        // hubp2_set_blank(false).
        DCN32_IO(io_.update(r::DCHUBP_CNTL::reg, pipe,
                            {{r::DCHUBP_CNTL::HUBP_BLANK_EN, 0}, {r::DCHUBP_CNTL::HUBP_TTU_DISABLE, 0}}));
    }
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::program_pipe(const Dcn32StreamPlan& plan, uint32_t pipe,
                                                                const PipeFlags& flags, UpdateKind kind) noexcept {
    // dcn20_program_pipe.
    const Dcn32Topology& t = plan.bandwidth.topology;
    const bool otg_master = topology_is_otg_master(t, pipe);
    if (otg_master && (flags.enable || flags.odm || kind.stream_all)) {
        DCN32_STEP(blank_pixel_data(t, plan.timing, false));
    }
    if (flags.global_sync && otg_master) {
        // calculate_vready_offset_for_group: the largest vready_offset of the
        // pipes connected to the OTG master.
        uint32_t vready = pipe_bandwidth(plan, pipe).vready_offset;
        for (uint32_t other = 0; other < dcn32_stream_pipes; ++other) {
            if (other != pipe && t.pipes[other].stream && pipe_bandwidth(plan, other).vready_offset > vready) {
                vready = pipe_bandwidth(plan, other).vready_offset;
            }
        }
        DCN32_STEP(program_global_sync(plan.pipe, vready, pipe_bandwidth(plan, pipe)));
        DCN32_STEP(wait_for_state(plan.pipe, false));
        DCN32_STEP(set_vtg_params(plan, true));
        DCN32_STEP(setup_vupdate_interrupt(plan));
    }
    if (flags.odm) {
        DCN32_STEP(update_odm(plan));
    }
    if (flags.enable) {
        DCN32_STEP(enable_plane(pipe, t.pipes[pipe].opp));
        // hubbub32_force_wm_propagate_to_pipes.
        DCN32_IO(io_.set(r::DCHUBBUB_ARB_DATA_URGENCY_WATERMARK_A::reg, 0,
                         {{r::DCHUBBUB_ARB_DATA_URGENCY_WATERMARK_A::DCHUBBUB_ARB_DATA_URGENCY_WATERMARK_A,
                           convert_and_clamp(hubbub_.a.urgent_ns, plan.dchub_refclk_mhz, 0x3fff)}}));
    }
    if (flags.det_size) {
        DCN32_STEP(program_det_size(pipe, pipe_bandwidth(plan, pipe).det_buffer_size_kb));
    }
    if (flags.any() || kind.plane_all || kind.stream_all) {
        DCN32_STEP(update_dchubp_dpp(plan, pipe, flags, kind));
    }
    if (flags.enable || kind.plane_all) {
        DCN32_IO(io_.update(r::CM_HDR_MULT_COEF::reg, pipe, {{r::CM_HDR_MULT_COEF::CM_HDR_MULT_COEF, hdr_mult_one}}));
    }
    if (kind.plane_all || flags.enable) {
        DCN32_STEP(set_input_transfer_func(pipe));
    }
    if (flags.enable || flags.plane_changed || kind.stream_all || kind.plane_all) {
        DCN32_STEP(set_output_transfer_func(pipe));
    }
    if (flags.enable || flags.opp_changed) {
        DCN32_STEP(set_dyn_expansion(t.pipes[pipe].opp, plan.signal));
        DCN32_STEP(program_fmt(t.pipes[pipe].opp));
    }
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::is_flip_pending(uint32_t hubp, bool& pending) noexcept {
    // hubp2_is_flip_pending (against hubp->request_address).
    uint32_t flip = 0;
    uint32_t low = 0;
    uint32_t high = 0;
    DCN32_IO(io_.get(r::DCSURF_FLIP_CONTROL::reg, hubp, r::DCSURF_FLIP_CONTROL::SURFACE_FLIP_PENDING, flip));
    DCN32_IO(io_.get(r::DCSURF_SURFACE_EARLIEST_INUSE::reg, hubp,
                     r::DCSURF_SURFACE_EARLIEST_INUSE::SURFACE_EARLIEST_INUSE_ADDRESS, low));
    DCN32_IO(io_.get(r::DCSURF_SURFACE_EARLIEST_INUSE_HIGH::reg, hubp,
                     r::DCSURF_SURFACE_EARLIEST_INUSE_HIGH::SURFACE_EARLIEST_INUSE_ADDRESS_HIGH, high));
    const uint64_t earliest = (static_cast<uint64_t>(high) << 32) | low;
    pending = flip != 0 || earliest != hubp_request_address_[hubp];
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::wait_for_flip(uint32_t hubp) noexcept {
    // TIMEOUT_FOR_PIPE_ENABLE_US / 1 us polls (also hwss_wait_for_no_pipes_pending).
    for (uint32_t index = 0; index < 100000; ++index) {
        bool pending = false;
        DCN32_STEP(is_flip_pending(hubp, pending));
        if (!pending) {
            break;
        }
        io_.delay_us(1);
    }
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::wait_for_no_pipes_pending(const Dcn32Topology& context) noexcept {
    // hwss_wait_for_no_pipes_pending: every pipe with a plane.
    for (uint32_t pipe = 0; pipe < dcn32_stream_pipes; ++pipe) {
        if (context.pipes[pipe].has_plane) {
            DCN32_STEP(wait_for_flip(pipe));
        }
    }
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::wait_for_odm_update_pending_complete(
    const Dcn32Topology& context) noexcept {
    // hwss_wait_for_odm_update_pending_complete: optc32_wait_odm_doublebuffer_pending_clear
    // of each OTG master (OTG_H_TIMING_DIV_MODE_DB_UPDATE_PENDING is not in
    // dcn32's mask list, so the REG_WAIT is satisfied by its first read), then
    // hwss_wait_for_all_blank_complete over the OPP heads.
    for (uint32_t pipe = 0; pipe < dcn32_stream_pipes; ++pipe) {
        if (topology_is_otg_master(context, pipe)) {
            uint32_t double_buffer = 0;
            DCN32_IO(io_.read(r::OTG_DOUBLE_BUFFER_CONTROL::reg, pipe, double_buffer));
        }
    }
    for (uint32_t pipe = 0; pipe < dcn32_stream_pipes; ++pipe) {
        if (topology_is_opp_head(context, pipe)) {
            DCN32_STEP(wait_for_blank_complete(context.pipes[pipe].opp));
        }
    }
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::post_unlock_reset_opp(uint32_t opp_head) noexcept {
    // post_unlock_reset_opp of a current-state OPP head that is not one in
    // the new state: wait for its blending tree to disconnect, then a DSC left
    // ungated waits for its double-buffered disconnect and is disabled.
    const Dcn32TopologyPipe& pipe = live_.pipes[opp_head];
    DCN32_STEP(wait_for_mpcc_disconnect(pipe.opp, live_.otg_master));
    if (pipe.dsc) {
        bool ungated = false;
        DCN32_STEP(dsc_pg_status(opp_head, ungated));
        if (ungated) {
            DCN32_IO(dcn32_dsc::wait_disconnect_pending_clear(io_, opp_head));
            DCN32_IO(dcn32_dsc::disable(io_, opp_head));
        }
    }
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::post_unlock_front_end(const Dcn32StreamPlan* next_plan,
                                                                        const Dcn32Topology& next) noexcept {
    // dcn20_post_unlock_program_front_end (dc->current_state = live_).
    for (uint32_t pipe = 0; pipe < dcn32_stream_pipes; ++pipe) {
        if (topology_is_opp_head(live_, pipe) && !topology_is_opp_head(next, pipe)) {
            DCN32_STEP(post_unlock_reset_opp(pipe));
        }
    }
    for (uint32_t pipe = 0; pipe < dcn32_stream_pipes; ++pipe) {
        // dcn20_disable_plane of the current state's pipe.
        if (flags_[pipe].disable && !hubp_power_gated_[pipe]) {
            DCN32_STEP(plane_atomic_disable(pipe, live_.pipes[pipe].opp, live_.otg_master));
        }
    }
    // Enabling a pipe waits for its pending flip to clear.
    for (uint32_t pipe = 0; pipe < dcn32_stream_pipes; ++pipe) {
        if (next.pipes[pipe].has_plane && next.pipes[pipe].top == dcn32_no_pipe && flags_[pipe].enable) {
            DCN32_STEP(wait_for_flip(pipe));
        }
    }
    // Going to more ODM slices: the double buffer update completes first.
    for (uint32_t pipe = 0; pipe < dcn32_stream_pipes; ++pipe) {
        if (topology_is_otg_master(live_, pipe) && topology_is_otg_master(next, pipe) &&
            topology_odm_slice_count(live_) < topology_odm_slice_count(next)) {
            for (uint32_t index = 0; index < 100000; ++index) {
                uint32_t pending = 0;
                DCN32_IO(io_.get(r::OPTC_INPUT_GLOBAL_CONTROL::reg, pipe,
                                 r::OPTC_INPUT_GLOBAL_CONTROL::OPTC_DOUBLE_BUFFER_PENDING, pending));
                if (pending != 1) {
                    break;
                }
                io_.delay_us(1);
            }
        }
    }
    DCN32_STEP(force_pstate_change_control(false, false));
    DCN32_STEP(update_force_pstate(next));
    // dcn32_program_mall_pipe_config -> dcn32_update_mall_sel (no SubVP buffering).
    if (next_plan != nullptr) {
        for (uint32_t pipe = 0; pipe < dcn32_stream_pipes; ++pipe) {
            if (next.pipes[pipe].stream && next.pipes[pipe].has_plane) {
                DCN32_IO(io_.update(r::DCHUBP_MALL_CONFIG::reg, pipe,
                                    {{r::DCHUBP_MALL_CONFIG::USE_MALL_SEL, next_plan->mall_sel},
                                     {r::DCHUBP_MALL_CONFIG::USE_MALL_FOR_CURSOR, 0}}));
            }
        }
    }
    return Step::ok;
}

void Dcn32StreamController::detect_pipe_changes(const Dcn32StreamPlan* next_plan, const Dcn32Topology& next,
                                                uint32_t pipe, PipeFlags& flags) const noexcept {
    // dcn20_detect_pipe_changes(dc->current_state = live_, next). Every
    // commit creates new plane states, so a pipe keeping a plane always has
    // plane_changed.
    flags = PipeFlags{};
    const Dcn32TopologyPipe& old_pipe = live_.pipes[pipe];
    const Dcn32TopologyPipe& new_pipe = next.pipes[pipe];
    if (topology_is_otg_master(next, pipe)) {
        // resource_is_odm_topology_changed.
        bool changed = !topology_is_otg_master(live_, pipe);
        uint32_t a = pipe;
        uint32_t b = pipe;
        while (!changed && a != dcn32_no_pipe && b != dcn32_no_pipe) {
            if (next.pipes[a].opp != live_.pipes[b].opp ||
                (next.pipes[a].next_odm == dcn32_no_pipe) != (live_.pipes[b].next_odm == dcn32_no_pipe)) {
                changed = true;
            }
            a = next.pipes[a].next_odm;
            b = live_.pipes[b].next_odm;
        }
        flags.odm = changed;
    }
    const bool old_plane = old_pipe.has_plane;
    if (!old_plane && !new_pipe.has_plane) {
        return;
    }
    if (!old_plane && new_pipe.has_plane) {
        flags.enable = true;
        flags.mpcc = true;
        flags.dppclk = true;
        flags.hubp_interdependent = true;
        flags.hubp_rq_dlg_ttu = true;
        flags.unbounded_req = true;
        flags.gamut_remap = true;
        flags.scaler = true;
        flags.viewport = true;
        flags.det_size = true;
        if (new_pipe.top == dcn32_no_pipe && new_pipe.prev_odm == dcn32_no_pipe) {
            flags.odm = true;
            flags.global_sync = true;
        }
        return;
    }
    if (old_plane && !new_pipe.has_plane) {
        flags.disable = true;
        return;
    }
    if (next_plan == nullptr) {
        return;
    }
    flags.plane_changed = true;
    const Dcn32PipeBandwidth& old_bw = pipe_bandwidth(current_, pipe);
    const Dcn32PipeBandwidth& new_bw = pipe_bandwidth(*next_plan, pipe);
    if (topology_is_otg_master(next, pipe) &&
        (old_bw.vready_offset != new_bw.vready_offset || old_bw.vstartup_start != new_bw.vstartup_start ||
         old_bw.vupdate_offset != new_bw.vupdate_offset || old_bw.vupdate_width != new_bw.vupdate_width)) {
        flags.global_sync = true;
    }
    flags.det_size = old_bw.det_buffer_size_kb != new_bw.det_buffer_size_kb;
    flags.opp_changed = old_pipe.opp != new_pipe.opp;
    flags.mpcc = flags.opp_changed;
    flags.dppclk = old_bw.dppclk_khz != new_bw.dppclk_khz;
    // scl_data and viewport.
    const Dcn32TopologyPipe& old_scl = current_.bandwidth.topology.pipes[pipe];
    uint32_t old_x = 0;
    uint32_t old_width = 0;
    uint32_t new_x = 0;
    uint32_t new_width = 0;
    odm_slice(current_.bandwidth.topology, current_.timing.width,
              topology_opp_head(current_.bandwidth.topology, pipe), old_x, old_width);
    odm_slice(next, next_plan->timing.width, topology_opp_head(next, pipe), new_x, new_width);
    const bool viewport_changed = old_scl.viewport.x != new_pipe.viewport.x ||
                                  old_scl.viewport.y != new_pipe.viewport.y ||
                                  old_scl.viewport.width != new_pipe.viewport.width ||
                                  old_scl.viewport.height != new_pipe.viewport.height;
    flags.viewport = viewport_changed;
    flags.scaler = viewport_changed || old_scl.recout.x != new_pipe.recout.x || old_scl.recout.y != new_pipe.recout.y ||
                   old_scl.recout.width != new_pipe.recout.width || old_scl.recout.height != new_pipe.recout.height ||
                   old_width != new_width || current_.timing.height != next_plan->timing.height ||
                   plane_of(current_, pipe).per_pixel_alpha != plane_of(*next_plan, pipe).per_pixel_alpha;
    // DLG/TTU/RQ.
    const Dcn32DlgRegs& od = old_bw.dlg;
    const Dcn32DlgRegs& nd = new_bw.dlg;
    const Dcn32TtuRegs& ot = old_bw.ttu;
    const Dcn32TtuRegs& nt = new_bw.ttu;
    flags.hubp_interdependent =
        od.dst_y_prefetch != nd.dst_y_prefetch || od.vratio_prefetch != nd.vratio_prefetch ||
        od.vratio_prefetch_c != nd.vratio_prefetch_c || od.dst_y_per_vm_vblank != nd.dst_y_per_vm_vblank ||
        od.dst_y_per_row_vblank != nd.dst_y_per_row_vblank || od.dst_y_per_vm_flip != nd.dst_y_per_vm_flip ||
        od.dst_y_per_row_flip != nd.dst_y_per_row_flip ||
        od.refcyc_per_meta_chunk_vblank_l != nd.refcyc_per_meta_chunk_vblank_l ||
        od.refcyc_per_meta_chunk_vblank_c != nd.refcyc_per_meta_chunk_vblank_c ||
        od.refcyc_per_meta_chunk_flip_l != nd.refcyc_per_meta_chunk_flip_l ||
        od.refcyc_per_line_delivery_pre_l != nd.refcyc_per_line_delivery_pre_l ||
        od.refcyc_per_line_delivery_pre_c != nd.refcyc_per_line_delivery_pre_c ||
        ot.refcyc_per_req_delivery_pre_l != nt.refcyc_per_req_delivery_pre_l ||
        ot.refcyc_per_req_delivery_pre_c != nt.refcyc_per_req_delivery_pre_c ||
        ot.refcyc_per_req_delivery_pre_cur0 != nt.refcyc_per_req_delivery_pre_cur0 ||
        ot.refcyc_per_req_delivery_pre_cur1 != nt.refcyc_per_req_delivery_pre_cur1 ||
        ot.min_ttu_vblank != nt.min_ttu_vblank || ot.qos_level_flip != nt.qos_level_flip;
    // Any other DLG/TTU/RQ difference (the interdependent fields compare equal
    // after Linux copies them into its old copy).
    Dcn32DlgRegs od_copy = od;
    Dcn32TtuRegs ot_copy = ot;
    od_copy.dst_y_prefetch = nd.dst_y_prefetch;
    od_copy.vratio_prefetch = nd.vratio_prefetch;
    od_copy.vratio_prefetch_c = nd.vratio_prefetch_c;
    od_copy.dst_y_per_vm_vblank = nd.dst_y_per_vm_vblank;
    od_copy.dst_y_per_row_vblank = nd.dst_y_per_row_vblank;
    od_copy.dst_y_per_vm_flip = nd.dst_y_per_vm_flip;
    od_copy.dst_y_per_row_flip = nd.dst_y_per_row_flip;
    od_copy.refcyc_per_meta_chunk_vblank_l = nd.refcyc_per_meta_chunk_vblank_l;
    od_copy.refcyc_per_meta_chunk_vblank_c = nd.refcyc_per_meta_chunk_vblank_c;
    od_copy.refcyc_per_meta_chunk_flip_l = nd.refcyc_per_meta_chunk_flip_l;
    od_copy.refcyc_per_line_delivery_pre_l = nd.refcyc_per_line_delivery_pre_l;
    od_copy.refcyc_per_line_delivery_pre_c = nd.refcyc_per_line_delivery_pre_c;
    ot_copy.refcyc_per_req_delivery_pre_l = nt.refcyc_per_req_delivery_pre_l;
    ot_copy.refcyc_per_req_delivery_pre_c = nt.refcyc_per_req_delivery_pre_c;
    ot_copy.refcyc_per_req_delivery_pre_cur0 = nt.refcyc_per_req_delivery_pre_cur0;
    ot_copy.refcyc_per_req_delivery_pre_cur1 = nt.refcyc_per_req_delivery_pre_cur1;
    ot_copy.min_ttu_vblank = nt.min_ttu_vblank;
    ot_copy.qos_level_flip = nt.qos_level_flip;
    flags.hubp_rq_dlg_ttu = !same_dlg(od_copy, nd) || !same_ttu(ot_copy, nt) || !same_rq(old_bw.rq, new_bw.rq);
    flags.unbounded_req = current_.bandwidth.unbounded_req != next_plan->bandwidth.unbounded_req;
}

Dcn32StreamController::Step Dcn32StreamController::program_front_end(const Dcn32StreamPlan* next_plan,
                                                                    const Dcn32Topology& next,
                                                                    UpdateKind kind) noexcept {
    // dcn20_program_front_end_for_ctx (dc->current_state = live_, current_).
    if (hubp_count(live_) == 0 && hubp_count(next) > 0) {
        DCN32_STEP(force_pstate_change_control(true, false));
        io_.delay_us(500);
    }
    for (uint32_t pipe = 0; pipe < dcn32_stream_pipes; ++pipe) {
        detect_pipe_changes(next_plan, next, pipe, flags_[pipe]);
    }
    const Timing& timing = next_plan != nullptr ? next_plan->timing : current_.timing;
    // OTG blank before disabling all front ends.
    for (uint32_t pipe = 0; pipe < dcn32_stream_pipes; ++pipe) {
        if (flags_[pipe].disable && next.pipes[pipe].top == dcn32_no_pipe &&
            next.pipes[pipe].prev_odm == dcn32_no_pipe && next.pipes[pipe].stream) {
            DCN32_STEP(blank_pixel_data(next, timing, true));
        }
    }
    // Disconnect MPCCs (the current state's pipes).
    for (uint32_t pipe = 0; pipe < dcn32_stream_pipes; ++pipe) {
        if (flags_[pipe].disable || flags_[pipe].opp_changed) {
            if (flags_[pipe].disable) {
                DCN32_STEP(program_det_size(pipe, 0));
            }
            DCN32_STEP(plane_atomic_disconnect(pipe, live_.pipes[pipe].opp));
        }
    }
    // Update ODM for blanked OTG master pipes.
    if (next_plan != nullptr) {
        for (uint32_t pipe = 0; pipe < dcn32_stream_pipes; ++pipe) {
            if (topology_is_otg_master(next, pipe) && !topology_is_dpp_pipe(next, pipe) && flags_[pipe].odm) {
                DCN32_STEP(update_odm(*next_plan));
            }
        }
        // Program every updated pipe, top pipes first, then their bottom pipes.
        for (uint32_t pipe = 0; pipe < dcn32_stream_pipes; ++pipe) {
            if (!next.pipes[pipe].has_plane || next.pipes[pipe].top != dcn32_no_pipe) {
                continue;
            }
            uint32_t walk = pipe;
            uint32_t guard = 0;
            while (walk != dcn32_no_pipe && guard++ < dcn32_stream_pipes) {
                DCN32_STEP(program_pipe(*next_plan, walk, flags_[walk], kind));
                walk = next.pipes[walk].bottom;
            }
        }
    }
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::setup_stereo(uint32_t opp, uint32_t otg) noexcept {
    // dcn10_setup_stereo without stereo: opp1_program_stereo(false) and
    // optc1_program_stereo -> optc1_disable_stereo (the generic GPIO stereo
    // mux is not touched).
    DCN32_IO(io_.update(r::FMT_CONTROL::reg, opp, {{r::FMT_CONTROL::FMT_STEREOSYNC_OVERRIDE, 0}}));
    DCN32_IO(io_.update(r::OPPBUF_CONTROL::reg, opp, {{r::OPPBUF_CONTROL::OPPBUF_ACTIVE_WIDTH, 0}}));
    DCN32_IO(io_.update(r::OPPBUF_3D_PARAMETERS_0::reg, opp,
                        {{r::OPPBUF_3D_PARAMETERS_0::OPPBUF_3D_VACT_SPACE1_SIZE, 0}}));
    DCN32_IO(io_.set(r::OTG_STEREO_CONTROL::reg, otg, {{r::OTG_STEREO_CONTROL::OTG_STEREO_EN, 0}}));
    DCN32_IO(io_.set(r::OTG_3D_STRUCTURE_CONTROL::reg, otg,
                     {{r::OTG_3D_STRUCTURE_CONTROL::OTG_3D_STRUCTURE_EN, 0},
                      {r::OTG_3D_STRUCTURE_CONTROL::OTG_3D_STRUCTURE_STEREO_SEL_OVR, 0}}));
    return Step::ok;
}

// ---- bandwidth (dcn32_prepare_bandwidth, dcn20_optimize_bandwidth) ----

Dcn32StreamController::Step Dcn32StreamController::prepare_bandwidth(const Dcn32BandwidthPlan* bandwidth,
                                                                    const Dcn32StreamClocks& clocks) noexcept {
    // dcn32_prepare_bandwidth -> dcn20_prepare_bandwidth (no FAMS, no SubVP).
    DCN32_STEP(link(hooks_.update_clocks(hooks_.context, clocks, false), StreamFailure::clock_refused));
    DCN32_STEP(program_watermarks(bandwidth != nullptr ? *bandwidth : no_bandwidth, false));
    DCN32_STEP(program_compbuf_size(bandwidth != nullptr ? bandwidth->compbuf_size_kb : 0, false));
    // No FPO (fw_based_mclk_switching is never planned): dc_dmub_srv_p_state_delegate(dc, false, context).
    if (hooks_.p_state_delegate != nullptr) {
        DCN32_STEP(ignored_result(hooks_.p_state_delegate(hooks_.context, false)));
    }
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::optimize_bandwidth(const Dcn32StreamPlan* target,
                                                                     const Dcn32BandwidthPlan* bandwidth,
                                                                     const Dcn32StreamClocks& clocks) noexcept {
    if (target != nullptr) {
        DCN32_STEP(wait_for_no_pipes_pending(target->bandwidth.topology));
        DCN32_STEP(wait_for_odm_update_pending_complete(target->bandwidth.topology));
    }
    // dcn20_optimize_bandwidth.
    DCN32_STEP(program_watermarks(bandwidth != nullptr ? *bandwidth : no_bandwidth, true));
    DCN32_STEP(program_compbuf_size(bandwidth != nullptr ? bandwidth->compbuf_size_kb : 0, true));
    DCN32_STEP(link(hooks_.update_clocks(hooks_.context, clocks, true), StreamFailure::clock_refused));
    return Step::ok;
}

Dcn32StreamClocks Dcn32StreamController::stream_clocks(const Dcn32StreamPlan& plan,
                                                       uint32_t display_count) const noexcept {
    Dcn32StreamClocks clocks{plan.bandwidth.clocks, plan.pipe, plan.bandwidth.dpp_dppclk_khz, display_count};
    if (plan.bandwidth.pipe_count > 1) {
        for (uint32_t i = 0; i < plan.bandwidth.pipe_count && i < dcn32_stream_pipes; ++i) {
            const Dcn32PipeBandwidth& pipe = plan.bandwidth.pipes[i];
            if (pipe.pipe < dcn32_stream_pipes) {
                clocks.dpp_pipe_khz[pipe.pipe] = pipe.dppclk_khz;
            }
        }
    }
    return clocks;
}

// ---- whole sequences ----

Dcn32StreamController::Step Dcn32StreamController::apply(const Dcn32StreamPlan* target) noexcept {
    // dc_commit_state_no_check.
    if (takeover_pending_) {
        DCN32_STEP(takeover());
    }
    // A context without streams keeps the copied bandwidth state (clocks,
    // watermarks, compbuf) of the current one; the firmware state has none.
    const bool own_current = current_stream_ && !current_firmware_;
    const Dcn32BandwidthPlan* bandwidth = nullptr;
    Dcn32StreamClocks clocks{};
    if (target != nullptr) {
        dchub_refclk_mhz_ = target->dchub_refclk_mhz;
        bandwidth = &target->bandwidth;
        clocks = stream_clocks(*target, 1);
    } else if (own_current) {
        bandwidth = &current_.bandwidth;
        clocks = {current_.bandwidth.clocks, current_.pipe, 0, 0};
    }
    DCN32_STEP(prepare_bandwidth(bandwidth, clocks));
    DCN32_STEP(update_dsc_pg(target, false));
    if (current_stream_ && current_plane_) {
        DCN32_STEP(disable_dangling_plane());
    }
    // Program hardware: wait_for_mpcc_disconnect of the new pipes (every
    // disconnect was already waited for by the dangling commit).
    // dce110_apply_ctx_to_hw.
    if (current_stream_) {
        DCN32_STEP(reset_back_end_for_pipe());
    }
    if (target == nullptr) {
        // program_front_end_for_ctx/post_unlock of an empty context.
        const Dcn32Topology empty{};
        DCN32_STEP(program_front_end(nullptr, empty, UpdateKind{true, false}));
        DCN32_STEP(post_unlock_front_end(nullptr, empty));
        live_ = empty;
        DCN32_STEP(optimize_bandwidth(nullptr, bandwidth, clocks));
        DCN32_STEP(update_dsc_pg(nullptr, true));
        return Step::ok;
    }
    const Dcn32Topology& next = target->bandwidth.topology;
    DCN32_STEP(apply_single_controller(*target));
    DCN32_STEP(resync_fifo_dccg_dio());
    // Program all planes within the new context under the interdependent lock.
    current_plane_ = true;
    DCN32_STEP(interdependent_lock(next, true));
    DCN32_STEP(program_front_end(target, next, UpdateKind{true, false}));
    DCN32_STEP(interdependent_lock(next, false));
    DCN32_STEP(post_unlock_front_end(target, next));
    live_ = next;
    // Stereo for the mode-changed stream, then dc_enable_stereo: every pipe of
    // the stream in pipe order, twice.
    for (uint32_t round = 0; round < 2; ++round) {
        for (uint32_t pipe = 0; pipe < dcn32_stream_pipes; ++pipe) {
            if (next.pipes[pipe].stream) {
                DCN32_STEP(setup_stereo(next.pipes[pipe].opp, target->pipe));
            }
        }
    }
    DCN32_STEP(optimize_bandwidth(target, bandwidth, clocks));
    DCN32_STEP(update_dsc_pg(target, true));
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::apply_planes(const Dcn32StreamPlan& plan) noexcept {
    // commit_planes_for_stream(UPDATE_TYPE_FULL) on the new context.
    const Dcn32Topology& next = plan.bandwidth.topology;
    // hwss_process_outstanding_hw_updates(dc->current_state): pending MPCC
    // disconnects of the current pipes, the ODM/blank waits, then the compbuf.
    for (uint32_t pipe = 0; pipe < dcn32_stream_pipes; ++pipe) {
        if (!live_.pipes[pipe].stream) {
            continue;
        }
        for (uint32_t opp = 0; opp < dcn32_stream_pipes; ++opp) {
            if (mpcc_disconnect_pending_[opp][pipe]) {
                uint32_t disabled = 0;
                DCN32_IO(io_.get(r::MPCC_STATUS::reg, pipe, r::MPCC_STATUS::MPCC_DISABLED, disabled));
                if (disabled == 0) {
                    DCN32_IO(io_.wait(r::MPCC_STATUS::reg, pipe, r::MPCC_STATUS::MPCC_IDLE, 1, 1, 100000));
                }
                mpcc_disconnect_pending_[opp][pipe] = false;
                break;
            }
        }
    }
    DCN32_STEP(wait_for_odm_update_pending_complete(live_));
    DCN32_STEP(program_compbuf_size(current_.bandwidth.compbuf_size_kb, true));
    dchub_refclk_mhz_ = plan.dchub_refclk_mhz;
    const Dcn32StreamClocks clocks = stream_clocks(plan, 1);
    DCN32_STEP(prepare_bandwidth(&plan.bandwidth, clocks));
    DCN32_STEP(update_dsc_pg(&plan, false));
    DCN32_STEP(interdependent_lock(next, true));
    DCN32_STEP(program_front_end(&plan, next, UpdateKind{false, true}));
    DCN32_STEP(interdependent_lock(next, false));
    DCN32_STEP(post_unlock_front_end(&plan, next));
    // Fire the manual trigger once per blending tree (its bottom pipe).
    for (uint32_t pipe = 0; pipe < dcn32_stream_pipes; ++pipe) {
        if (next.pipes[pipe].has_plane && next.pipes[pipe].bottom == dcn32_no_pipe &&
            next.pipes[pipe].next_odm == dcn32_no_pipe) {
            // optc2_program_manual_trigger.
            DCN32_IO(io_.set(r::OTG_TRIGA_MANUAL_TRIG::reg, plan.pipe,
                             {{r::OTG_TRIGA_MANUAL_TRIG::OTG_TRIGA_MANUAL_TRIG, 1}}));
        }
    }
    current_ = plan;
    live_ = next;
    current_plane_ = true;
    // dc_post_update_surfaces_to_stream (DEVIATION: at once, after waiting for
    // the pending flips instead of returning until the next flip). Free pipes
    // have no HUBP (disable_plane returns), mem low power is off.
    DCN32_STEP(wait_for_no_pipes_pending(next));
    DCN32_STEP(optimize_bandwidth(nullptr, &plan.bandwidth, clocks));
    DCN32_STEP(update_dsc_pg(&plan, true));
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::expect(const r::Register& reg, uint32_t instance, r::Field field,
                                                          uint32_t value) noexcept {
    uint32_t current = 0;
    DCN32_IO(io_.get(reg, instance, field, current));
    return current == value ? Step::ok : fail(StreamFailure::readback_mismatch);
}

Dcn32StreamController::Step Dcn32StreamController::verify(const Dcn32StreamPlan& plan) noexcept {
    // Independent readback of the state the commit established (double-
    // buffered registers are read after the post-unlock flip wait).
    const uint32_t otg = plan.pipe;
    const Timing& t = plan.timing;
    const Dcn32Topology& topology = plan.bandwidth.topology;
    const uint32_t v_fp = patched_v_front_porch(t);
    DCN32_STEP(expect(r::OTG_CONTROL::reg, otg, r::OTG_CONTROL::OTG_MASTER_EN, 1));
    DCN32_STEP(expect(r::OTG_H_TOTAL::reg, otg, r::OTG_H_TOTAL::OTG_H_TOTAL, t.h_total - 1));
    DCN32_STEP(expect(r::OTG_V_TOTAL::reg, otg, r::OTG_V_TOTAL::OTG_V_TOTAL, t.v_total - 1));
    DCN32_STEP(expect(r::OTG_H_BLANK_START_END::reg, otg, r::OTG_H_BLANK_START_END::OTG_H_BLANK_START,
                      t.h_total - t.h_front_porch));
    DCN32_STEP(expect(r::OTG_H_BLANK_START_END::reg, otg, r::OTG_H_BLANK_START_END::OTG_H_BLANK_END,
                      t.h_total - t.h_front_porch - t.width));
    DCN32_STEP(expect(r::OTG_V_BLANK_START_END::reg, otg, r::OTG_V_BLANK_START_END::OTG_V_BLANK_START,
                      t.v_total - v_fp));
    DCN32_STEP(expect(r::OTG_V_BLANK_START_END::reg, otg, r::OTG_V_BLANK_START_END::OTG_V_BLANK_END,
                      t.v_total - v_fp - t.height));
    DCN32_STEP(expect(r::OTG_VSTARTUP_PARAM::reg, otg, r::OTG_VSTARTUP_PARAM::VSTARTUP_START,
                      pipe_bandwidth(plan, otg).vstartup_start));
    // OTG data sources: one segment per ODM slice (OPP of each OPP head).
    uint8_t heads[dcn32_topology_pipes]{};
    const uint32_t opp_cnt = topology_opp_heads(topology, heads);
    const r::Field segments[4] = {r::OPTC_DATA_SOURCE_SELECT::OPTC_SEG0_SRC_SEL,
                                  r::OPTC_DATA_SOURCE_SELECT::OPTC_SEG1_SRC_SEL,
                                  r::OPTC_DATA_SOURCE_SELECT::OPTC_SEG2_SRC_SEL,
                                  r::OPTC_DATA_SOURCE_SELECT::OPTC_SEG3_SRC_SEL};
    for (uint32_t i = 0; i < opp_cnt; ++i) {
        DCN32_STEP(expect(r::OPTC_DATA_SOURCE_SELECT::reg, otg, segments[i], topology.pipes[heads[i]].opp));
    }
    if (opp_cnt > 1) {
        DCN32_STEP(expect(r::OPTC_DATA_SOURCE_SELECT::reg, otg, r::OPTC_DATA_SOURCE_SELECT::OPTC_NUM_OF_INPUT_SEGMENT,
                          opp_cnt - 1));
        DCN32_STEP(expect(r::OTG_H_TIMING_CNTL::reg, otg, r::OTG_H_TIMING_CNTL::OTG_H_TIMING_DIV_MODE, opp_cnt - 1));
    }
    // Each OPP head: video mode and its blending tree's top MPCC.
    for (uint32_t i = 0; i < opp_cnt; ++i) {
        const uint32_t opp = topology.pipes[heads[i]].opp;
        DCN32_STEP(expect(r::DPG_CONTROL::reg, opp, r::DPG_CONTROL::DPG_EN, 0));
        DCN32_STEP(expect(r::MPC_OUT_MUX::reg, opp, r::MPC_OUT_MUX::MPC_OUT_MUX, heads[i]));
    }
    // Each pipe with a plane: MPCC, HUBP and its surface.
    for (uint32_t pipe = 0; pipe < dcn32_stream_pipes; ++pipe) {
        const Dcn32TopologyPipe& p = topology.pipes[pipe];
        if (!p.has_plane) {
            continue;
        }
        const Dcn32PlaneScanout& plane = plane_of(plan, pipe);
        DCN32_STEP(expect(r::MPCC_TOP_SEL::reg, pipe, r::MPCC_TOP_SEL::MPCC_TOP_SEL, pipe));
        DCN32_STEP(expect(r::MPCC_OPP_ID::reg, pipe, r::MPCC_OPP_ID::MPCC_OPP_ID, p.opp));
        DCN32_STEP(expect(r::MPCC_BOT_SEL::reg, pipe, r::MPCC_BOT_SEL::MPCC_BOT_SEL,
                          p.bottom == dcn32_no_pipe ? mpcc_none : p.bottom));
        DCN32_STEP(expect(r::HUBP_CLK_CNTL::reg, pipe, r::HUBP_CLK_CNTL::HUBP_CLOCK_ENABLE, 1));
        DCN32_STEP(expect(r::DCHUBP_CNTL::reg, pipe, r::DCHUBP_CNTL::HUBP_BLANK_EN, 0));
        DCN32_STEP(expect(r::DCHUBP_CNTL::reg, pipe, r::DCHUBP_CNTL::HUBP_VTG_SEL, otg));
        DCN32_STEP(expect(r::DCSURF_SURFACE_PITCH::reg, pipe, r::DCSURF_SURFACE_PITCH::PITCH, plane.pitch_pixels - 1));
        DCN32_STEP(expect(r::DCSURF_PRI_VIEWPORT_DIMENSION::reg, pipe,
                          r::DCSURF_PRI_VIEWPORT_DIMENSION::PRI_VIEWPORT_WIDTH,
                          static_cast<uint32_t>(p.viewport.width)));
        DCN32_STEP(expect(r::DCSURF_PRI_VIEWPORT_DIMENSION::reg, pipe,
                          r::DCSURF_PRI_VIEWPORT_DIMENSION::PRI_VIEWPORT_HEIGHT,
                          static_cast<uint32_t>(p.viewport.height)));
        DCN32_STEP(expect(r::DCSURF_PRI_VIEWPORT_START::reg, pipe, r::DCSURF_PRI_VIEWPORT_START::PRI_VIEWPORT_X_START,
                          static_cast<uint32_t>(p.viewport.x)));
        DCN32_STEP(expect(r::DCSURF_PRIMARY_SURFACE_ADDRESS::reg, pipe,
                          r::DCSURF_PRIMARY_SURFACE_ADDRESS::PRIMARY_SURFACE_ADDRESS,
                          static_cast<uint32_t>(plane.surface_address)));
        DCN32_STEP(expect(r::DCSURF_PRIMARY_SURFACE_ADDRESS_HIGH::reg, pipe,
                          r::DCSURF_PRIMARY_SURFACE_ADDRESS_HIGH::PRIMARY_SURFACE_ADDRESS_HIGH,
                          static_cast<uint32_t>(plane.surface_address >> 32)));
        DCN32_STEP(expect(r::DCSURF_SURFACE_CONFIG::reg, pipe, r::DCSURF_SURFACE_CONFIG::SURFACE_PIXEL_FORMAT, 8));
    }
    DCN32_STEP(expect(r::DIG_FE_CNTL::reg, plan.engine, r::DIG_FE_CNTL::DIG_SOURCE_SELECT, otg));
    if (is_dp(plan.signal)) {
        DCN32_STEP(expect(r::DP_VID_STREAM_CNTL::reg, plan.engine, r::DP_VID_STREAM_CNTL::DP_VID_STREAM_ENABLE, 1));
    }
    if (is_hdmi(plan.signal)) {
        DCN32_STEP(expect(r::HDMI_GC::reg, plan.engine, r::HDMI_GC::HDMI_GC_AVMUTE, 0));
    }
    if (plan.dsc) {
        DCN32_STEP(expect(r::DP_DSC_CNTL::reg, plan.engine, r::DP_DSC_CNTL::DP_DSC_MODE, 1));
        DCN32_STEP(expect(r::OPTC_DATA_FORMAT_CONTROL::reg, otg, r::OPTC_DATA_FORMAT_CONTROL::OPTC_DSC_MODE,
                          optc_dsc_enabled_444));
        for (uint32_t i = 0; i < opp_cnt; ++i) {
            DCN32_STEP(expect(r::DSCRM_DSC_FORWARD_CONFIG::reg, heads[i],
                              r::DSCRM_DSC_FORWARD_CONFIG::DSCRM_DSC_FORWARD_EN, 1));
            DCN32_STEP(expect(r::DSCRM_DSC_FORWARD_CONFIG::reg, heads[i],
                              r::DSCRM_DSC_FORWARD_CONFIG::DSCRM_DSC_OPP_PIPE_SOURCE, topology.pipes[heads[i]].opp));
        }
    }
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::check_frame(const Dcn32StreamPlan& plan,
                                                               StreamHealth& health) noexcept {
    const uint32_t otg = plan.pipe;
    const Dcn32Topology& topology = plan.bandwidth.topology;
    health = {};
    uint32_t value = 0;
    DCN32_IO(io_.get(r::OTG_CONTROL::reg, otg, r::OTG_CONTROL::OTG_MASTER_EN, value));
    health.active = value != 0;
    DCN32_IO(io_.get(r::DPG_CONTROL::reg, topology.pipes[otg].opp, r::DPG_CONTROL::DPG_EN, value));
    health.dpg_blanked = value != 0;
    uint32_t first = 0;
    uint32_t second = 0;
    DCN32_IO(io_.get(r::OTG_STATUS_FRAME_COUNT::reg, otg, r::OTG_STATUS_FRAME_COUNT::OTG_FRAME_COUNT, first));
    // One frame and a margin; bounded by the 100 ms of a 10 Hz frame.
    uint32_t wait = frame_microseconds(plan.timing) + 1000u;
    if (wait > 101000u) {
        wait = 101000u;
    }
    io_.delay_us(wait);
    DCN32_IO(io_.get(r::OTG_STATUS_FRAME_COUNT::reg, otg, r::OTG_STATUS_FRAME_COUNT::OTG_FRAME_COUNT, second));
    health.frame_count = second;
    health.frame_counter_moving = first != second;
    DCN32_IO(io_.get(r::OPTC_INPUT_GLOBAL_CONTROL::reg, otg,
                     r::OPTC_INPUT_GLOBAL_CONTROL::OPTC_UNDERFLOW_OCCURRED_STATUS, value));
    health.optc_underflow = value != 0;
    for (uint32_t pipe = 0; pipe < dcn32_stream_pipes; ++pipe) {
        if (!topology.pipes[pipe].has_plane) {
            continue;
        }
        DCN32_IO(io_.get(r::DCHUBP_CNTL::reg, pipe, r::DCHUBP_CNTL::HUBP_UNDERFLOW_STATUS, value));
        health.hubp_underflow = health.hubp_underflow || value != 0;
    }
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::settle_and_check(const Dcn32StreamPlan& plan) noexcept {
    // The first frame after an enable may legitimately report the transition;
    // clear the sticky bits (optc1_clear_optc_underflow, hubp2_clear_underflow
    // of every pipe with a plane) and require a clean, moving frame.
    DCN32_IO(io_.update(r::OPTC_INPUT_GLOBAL_CONTROL::reg, plan.pipe,
                        {{r::OPTC_INPUT_GLOBAL_CONTROL::OPTC_UNDERFLOW_CLEAR, 1}}));
    for (uint32_t pipe = 0; pipe < dcn32_stream_pipes; ++pipe) {
        if (topo(plan, pipe).has_plane) {
            DCN32_IO(io_.update(r::DCHUBP_CNTL::reg, pipe, {{r::DCHUBP_CNTL::HUBP_UNDERFLOW_CLEAR, 1}}));
        }
    }
    StreamHealth health{};
    DCN32_STEP(check_frame(plan, health));
    if (!health.active || !health.frame_counter_moving) {
        return fail(StreamFailure::timing_stalled);
    }
    if (health.optc_underflow || health.hubp_underflow) {
        return fail(StreamFailure::underflow);
    }
    return Step::ok;
}

Dcn32StreamController::Step Dcn32StreamController::attempt(const Dcn32StreamPlan& plan) noexcept {
    ++report_.attempts;
    DCN32_STEP(apply(&plan));
    DCN32_STEP(verify(plan));
    return settle_and_check(plan);
}

Dcn32StreamController::Step Dcn32StreamController::disable_current() noexcept {
    // Recovery cleanup: whatever part of the stream/plane was enabled is
    // treated as enabled and taken down by the Linux disable path.
    io_.clear_timeout();
    if (current_plane_) {
        for (uint32_t pipe = 0; pipe < dcn32_stream_pipes; ++pipe) {
            if (live_.pipes[pipe].has_plane) {
                hubp_power_gated_[pipe] = false;
            }
        }
    }
    return apply(nullptr);
}

StreamResult Dcn32StreamController::finish(StreamResult result, StreamFailure failure) noexcept {
    report_.result = result;
    report_.failure = failure;
    report_.writes = io_.write_count();
    if (result == StreamResult::uncertain) {
        quarantined_ = true;
    }
    return result;
}

namespace {

// The pipes of a plan are consistent with its OTG master and planes.
bool plan_topology_valid(const Dcn32StreamPlan& plan) noexcept {
    const Dcn32Topology& t = plan.bandwidth.topology;
    if (plan.bandwidth.pipe_count == 0 || plan.bandwidth.pipe_count > dcn32_stream_pipes ||
        t.otg_master != plan.pipe || !topology_is_otg_master(t, plan.pipe) || plan.plane_count == 0 ||
        plan.plane_count > dcn32_max_planes || topology_odm_slice_count(t) != plan.odm_slices ||
        topology_used_pipes(t) != plan.bandwidth.pipe_count) {
        return false;
    }
    for (uint32_t pipe = 0; pipe < dcn32_stream_pipes; ++pipe) {
        const Dcn32TopologyPipe& p = t.pipes[pipe];
        if (!p.stream) {
            continue;
        }
        if (!p.has_plane || p.plane >= plan.plane_count || find_pipe_bandwidth(plan.bandwidth, pipe) == nullptr ||
            plan.planes[p.plane].surface_address == 0 || plan.planes[p.plane].pitch_pixels == 0 ||
            p.opp >= dcn32_stream_pipes || (p.dsc && !plan.dsc)) {
            return false;
        }
    }
    return true;
}

// Two plans drive the same stream (timing, signal, link, DSC/FEC, ODM pipes).
bool same_stream(const Dcn32StreamPlan& a, const Dcn32StreamPlan& b) noexcept {
    const Timing& x = a.timing;
    const Timing& y = b.timing;
    if (x.pixel_clock_khz != y.pixel_clock_khz || x.width != y.width || x.h_total != y.h_total ||
        x.h_front_porch != y.h_front_porch || x.h_sync_width != y.h_sync_width || x.height != y.height ||
        x.v_total != y.v_total || x.v_front_porch != y.v_front_porch || x.v_sync_width != y.v_sync_width ||
        x.flags != y.flags || a.signal != b.signal || a.pipe != b.pipe || a.engine != b.engine ||
        a.dp_rate_code != b.dp_rate_code || a.dp_lanes != b.dp_lanes || a.dp_use_vsc != b.dp_use_vsc ||
        a.tmds_khz != b.tmds_khz || a.dsc != b.dsc || a.fec != b.fec || a.odm_slices != b.odm_slices ||
        a.dchub_refclk_mhz != b.dchub_refclk_mhz) {
        return false;
    }
    if (a.dsc && (a.dsc_config.num_slices_h != b.dsc_config.num_slices_h ||
                  a.dsc_config.bits_per_pixel != b.dsc_config.bits_per_pixel)) {
        return false;
    }
    uint8_t heads_a[dcn32_topology_pipes]{};
    uint8_t heads_b[dcn32_topology_pipes]{};
    const uint32_t count = topology_opp_heads(a.bandwidth.topology, heads_a);
    if (count != topology_opp_heads(b.bandwidth.topology, heads_b)) {
        return false;
    }
    for (uint32_t i = 0; i < count; ++i) {
        if (heads_a[i] != heads_b[i]) {
            return false;
        }
    }
    return true;
}

} // namespace

StreamResult Dcn32StreamController::retry_or_rollback(const Dcn32StreamPlan& plan) noexcept {
    report_.first_failure = failure_;
    ++report_.recoveries;
    // Bounded retry from a disabled pipe.
    if (disable_current() != Step::ok) {
        return finish(StreamResult::uncertain, failure_);
    }
    Step step = attempt(plan);
    if (step == Step::ok) {
        return finish(StreamResult::committed, StreamFailure::none);
    }
    if (step == Step::uncertain) {
        return finish(StreamResult::uncertain, failure_);
    }
    const StreamFailure second = failure_;
    if (rollback_valid_) {
        if (disable_current() != Step::ok) {
            return finish(StreamResult::uncertain, failure_);
        }
        step = attempt(rollback_);
        if (step == Step::ok) {
            rollback_valid_ = false;
            return finish(StreamResult::restored, second);
        }
        if (step == Step::uncertain) {
            return finish(StreamResult::uncertain, failure_);
        }
    }
    rollback_valid_ = false;
    if (disable_current() != Step::ok) {
        return finish(StreamResult::uncertain, failure_);
    }
    return finish(StreamResult::blanked, second);
}

StreamResult Dcn32StreamController::commit(const Dcn32StreamPlan& plan) noexcept {
    report_.failure = StreamFailure::none;
    report_.first_failure = StreamFailure::none;
    report_.attempts = 0;
    failure_ = StreamFailure::none;
    if (!io_.bound() || quarantined_) {
        return finish(quarantined_ ? StreamResult::uncertain : StreamResult::unchanged, StreamFailure::quarantined);
    }
    if (plan.pipe >= dcn32_stream_pipes || plan.engine >= dcn32_encoder::engine_count ||
        plan.bandwidth.vstartup_start == 0 || (plan.plane_count <= 1 && plan.pitch_pixels < plan.timing.width) ||
        plan.surface_address == 0 ||
        plan.dchub_refclk_mhz == 0 || (current_stream_ && plan.pipe != current_.pipe) || &plan == &rollback_ ||
        !plan_topology_valid(plan)) {
        return finish(StreamResult::unchanged, StreamFailure::invalid_plan);
    }
    ++report_.commits;
    // The rollback target: the stream this owner committed last (the
    // firmware state cannot be re-created; its mode is committed as a plan).
    rollback_valid_ = current_stream_ && !current_firmware_;
    if (rollback_valid_ && &plan != &current_) {
        rollback_ = current_;
    } else if (rollback_valid_) {
        rollback_valid_ = false;
    }
    const Step step = attempt(plan);
    if (step == Step::ok) {
        return finish(StreamResult::committed, StreamFailure::none);
    }
    if (step == Step::uncertain) {
        return finish(StreamResult::uncertain, failure_);
    }
    return retry_or_rollback(plan);
}

StreamResult Dcn32StreamController::update_planes(const Dcn32StreamPlan& plan) noexcept {
    report_.failure = StreamFailure::none;
    report_.first_failure = StreamFailure::none;
    report_.attempts = 0;
    failure_ = StreamFailure::none;
    if (!io_.bound() || quarantined_) {
        return finish(quarantined_ ? StreamResult::uncertain : StreamResult::unchanged, StreamFailure::quarantined);
    }
    // A plane update keeps the stream (update_planes_and_stream_v2 needs no
    // minimal transition: the topology change must be seamless).
    if (!current_stream_ || current_firmware_ || dpms_off_ || &plan == &rollback_ || &plan == &current_ ||
        !plan_topology_valid(plan) || !same_stream(plan, current_) ||
        !topology_transition_seamless(live_, plan.bandwidth.topology)) {
        return finish(StreamResult::unchanged, StreamFailure::invalid_plan);
    }
    ++report_.commits;
    rollback_ = current_;
    rollback_valid_ = true;
    ++report_.attempts;
    Step step = apply_planes(plan);
    if (step == Step::ok) {
        step = verify(plan);
    }
    if (step == Step::ok) {
        step = settle_and_check(plan);
    }
    if (step == Step::ok) {
        return finish(StreamResult::committed, StreamFailure::none);
    }
    if (step == Step::uncertain) {
        return finish(StreamResult::uncertain, failure_);
    }
    // The planes may be partly programmed: the disable path takes every pipe
    // of the plan down before the retry as a full commit.
    current_ = plan;
    live_ = plan.bandwidth.topology;
    current_stream_ = true;
    current_plane_ = true;
    return retry_or_rollback(plan);
}

StreamResult Dcn32StreamController::disable() noexcept {
    report_.failure = StreamFailure::none;
    failure_ = StreamFailure::none;
    if (!io_.bound() || quarantined_) {
        return finish(quarantined_ ? StreamResult::uncertain : StreamResult::unchanged, StreamFailure::quarantined);
    }
    if (!current_stream_) {
        return finish(StreamResult::committed, StreamFailure::none);
    }
    // A disable that stops part way has nothing safer to fall back to.
    if (apply(nullptr) == Step::ok) {
        return finish(StreamResult::committed, StreamFailure::none);
    }
    return finish(StreamResult::uncertain, failure_);
}

StreamResult Dcn32StreamController::set_dpms(bool on) noexcept {
    failure_ = StreamFailure::none;
    if (!io_.bound() || quarantined_ || !current_stream_ || current_firmware_) {
        return finish(quarantined_ ? StreamResult::uncertain : StreamResult::unchanged, StreamFailure::invalid_plan);
    }
    if (on == !dpms_off_) {
        return finish(StreamResult::committed, StreamFailure::none);
    }
    Step step = Step::ok;
    if (!on) {
        // commit_planes_do_stream_update(dpms_off): link_set_dpms_off.
        step = link_set_dpms_off(current_);
        if (step == Step::ok) {
            dpms_off_ = true;
        }
    } else {
        // prepare_bandwidth(current_state) then link_set_dpms_on.
        const Dcn32StreamClocks clocks = stream_clocks(current_, 1);
        step = prepare_bandwidth(&current_.bandwidth, clocks);
        if (step == Step::ok) {
            step = link_set_dpms_on(current_);
        }
        if (step == Step::ok) {
            dpms_off_ = false;
        }
    }
    if (step == Step::ok) {
        return finish(StreamResult::committed, StreamFailure::none);
    }
    return finish(StreamResult::uncertain, failure_);
}

bool Dcn32StreamController::check_health(StreamHealth& health) noexcept {
    health = {};
    if (!io_.bound() || quarantined_ || !current_stream_ || current_firmware_ || dpms_off_) {
        return false;
    }
    if (check_frame(current_, health) != Step::ok) {
        return false;
    }
    if (health.optc_underflow &&
        !io_.update(r::OPTC_INPUT_GLOBAL_CONTROL::reg, current_.pipe,
                    {{r::OPTC_INPUT_GLOBAL_CONTROL::OPTC_UNDERFLOW_CLEAR, 1}})) {
        return false;
    }
    if (health.hubp_underflow) {
        for (uint32_t pipe = 0; pipe < dcn32_stream_pipes; ++pipe) {
            if (topo(current_, pipe).has_plane &&
                !io_.update(r::DCHUBP_CNTL::reg, pipe, {{r::DCHUBP_CNTL::HUBP_UNDERFLOW_CLEAR, 1}})) {
                return false;
            }
        }
    }
    return true;
}

StreamResult Dcn32StreamController::recover() noexcept {
    if (!io_.bound() || quarantined_ || !current_stream_ || current_firmware_) {
        return finish(quarantined_ ? StreamResult::uncertain : StreamResult::unchanged, StreamFailure::invalid_plan);
    }
    // Re-commit the same plan: the disable path runs first (the plan is the
    // previous state too, so a failure ends restored-or-blanked).
    scratch_ = current_;
    return commit(scratch_);
}

void Dcn32StreamController::reset_proven() noexcept {
    io_.reset_proven();
    current_stream_ = false;
    current_plane_ = false;
    current_firmware_ = false;
    dpms_off_ = false;
    rollback_valid_ = false;
    live_ = Dcn32Topology{};
    for (uint32_t index = 0; index < dcn32_stream_pipes; ++index) {
        mpc_tree_top_[index] = mpcc_none;
        mpcc_bot_[index] = mpcc_none;
        mpcc_dpp_[index] = mpcc_none;
        for (uint32_t mpcc = 0; mpcc < dcn32_stream_pipes; ++mpcc) {
            mpcc_disconnect_pending_[index][mpcc] = false;
        }
        opp_count_[index] = 1;
        hubp_power_gated_[index] = false;
        hubp_request_address_[index] = 0;
        scl_[index] = ScalerCache{};
        flags_[index] = PipeFlags{};
    }
    hubbub_ = {};
    dchub_refclk_mhz_ = 0;
    takeover_pending_ = true;
    quarantined_ = false;
    failure_ = StreamFailure::none;
}

#undef DCN32_IO
#undef DCN32_STEP

} // namespace rx7900::display

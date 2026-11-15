/*
 * Copyright 2022 Advanced Micro Devices, Inc. All rights reserved.
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
Module: rx7900.display.dcn32_bandwidth
Purpose: The dcn32 glue around the DML32 port for one stream with one linear plane.
Created: 2026-10-24 (adaptation; the upstream notice above is preserved)
Architecture: Public side of src/display/dcn32_bandwidth.hpp. Transcribes, in Linux
v6.12 call order, dcn32_init_clocks (DISPCLK/DPPCLK caps), dcn32_update_bw_bounding_box_fpu
and build_synthetic_soc_states with their helpers, dcn32_build_wm_range_table_fpu,
dcn20/dcn32_populate_dml_pipes_from_context, dcn30_fpu_update_soc_for_wm_a,
dcn32_internal_validate_bw with dcn32_full_validate_bw_helper (SubVP, FPO and ODM power
optimization never apply in the reference configuration), dcn20_validate_apply_pipe_split_flags
(MPC_SPLIT_AVOID), dcn32_calculate_wm_and_dlg_fpu (sets B, C, A and D = A) and
dcn32_calculate_dlg_params (dcn32_fpu.c, dcn32_resource.c, dcn32_resource_helpers.c,
dcn20_fpu.c, dcn30_fpu.c, dcn32_clk_mgr.c). The DML core itself is src/display/dcn32_dml*.
*/
#include "dcn32_bandwidth.hpp"

#include "dcn32_dml_util.hpp"
#include "../reactos/lifetime.hpp"

namespace rx7900::display {

using namespace dml32;

namespace {

// clk_mgr.h WM_A..WM_D.
constexpr uint32_t wm_a = 0;
constexpr uint32_t wm_b = 1;
constexpr uint32_t wm_c = 2;
constexpr uint32_t wm_d = 3;
// dcn32_resource.h.
constexpr uint32_t dcn3_2_max_det_size = 1152;
// DCN3_2_VMIN_DISPCLK_HZ / 1000: above it Linux applies ODM 2:1 power optimization
// to a single DP stream (should_apply_odm_power_optimization), which needs two pipes.
constexpr uint32_t dcn3_2_vmin_dispclk_khz = 717000;
// dcn32_init_clocks: "HW recommends limit of 1950 MHz in display clock for all DCN3.2.x".
constexpr uint32_t dcn32_dispclk_cap_mhz = 1950;
// Linux's synthetic state table holds DC__VOLTAGE_STATES entries; the build never
// needs more than 5 STA + 1 + 8 UCLK + 8 FCLK + 1 = 23 of them.
constexpr uint32_t synthetic_table_capacity = DML32_NUM_STATES;

struct WmRangeInput final {
    bool valid;
    double pstate_latency_us;
    double sr_exit_time_us;
    double sr_enter_plus_exit_time_us;
    double fclk_change_latency_us;
};

struct DummyPstate final {
    uint32_t dram_speed_mts;
    uint32_t dummy_pstate_latency_us;
};

} // namespace

// Device-owned model state; see dcn32_bandwidth.hpp.
struct Dcn32BandwidthWorkspace final {
    // context->bw_ctx.dml; soc and ip are reset from the configured box per validate.
    display_mode_lib dml;
    // dcn3_2_soc and dcn3_2_ip after update_bw_bounding_box.
    soc_bounding_box_st soc;
    ip_params_st ip;
    // clk_mgr bw_params.
    Dcn32ClockTable clocks;
    uint32_t num_memclk_levels;
    WmRangeInput wm[4];
    DummyPstate dummy[4];
    uint32_t dchub_refclk_khz;
    display_e2e_pipe_params_st pipes[DML32_NUM_PLANES];
    display_dlg_regs_st dlg;
    display_ttu_regs_st ttu;
    display_rq_regs_st rq;
    // validate's result, kept off the kernel stack until it is copied out.
    Dcn32BandwidthPlan plan;
    // The context being validated (new) and dc->current_state's pipes (cur),
    // the planes, and the split/merge flags of dcn32_internal_validate_bw.
    Dcn32Topology topology;
    Dcn32Topology current;
    Dcn32PlaneGeometry planes[dcn32_max_planes];
    uint32_t plane_pitch[dcn32_max_planes];
    uint32_t plane_count;
    int split[dcn32_topology_pipes];
    bool merge[dcn32_topology_pipes];
    unsigned int pipe_cnt;
};

namespace {

void dcn3_2_defaults(soc_bounding_box_st& soc, ip_params_st& ip) noexcept {
    dml_zero(soc);
    dml_zero(ip);
    // dcn3_2_ip (dcn32_fpu.c), DCN3_2_DEFAULT_DET_SIZE = 256.
    ip.gpuvm_enable = 0;
    ip.gpuvm_max_page_table_levels = 4;
    ip.hostvm_enable = 0;
    ip.rob_buffer_size_kbytes = 128;
    ip.det_buffer_size_kbytes = 256;
    ip.config_return_buffer_size_in_kbytes = 1280;
    ip.compressed_buffer_segment_size_in_kbytes = 64;
    ip.meta_fifo_size_in_kentries = 22;
    ip.zero_size_buffer_entries = 512;
    ip.compbuf_reserved_space_64b = 256;
    ip.compbuf_reserved_space_zs = 64;
    ip.dpp_output_buffer_pixels = 2560;
    ip.opp_output_buffer_lines = 1;
    ip.pixel_chunk_size_kbytes = 8;
    ip.alpha_pixel_chunk_size_kbytes = 4;
    ip.min_pixel_chunk_size_bytes = 1024;
    ip.dcc_meta_buffer_size_bytes = 6272;
    ip.meta_chunk_size_kbytes = 2;
    ip.min_meta_chunk_size_bytes = 256;
    ip.writeback_chunk_size_kbytes = 8;
    ip.ptoi_supported = false;
    ip.num_dsc = 4;
    ip.maximum_dsc_bits_per_component = 12;
    ip.maximum_pixels_per_line_per_dsc_unit = 6016;
    ip.dsc422_native_support = true;
    ip.is_line_buffer_bpp_fixed = true;
    ip.line_buffer_fixed_bpp = 57;
    ip.line_buffer_size_bits = 1171920;
    ip.max_line_buffer_lines = 32;
    ip.writeback_interface_buffer_size_kbytes = 90;
    ip.max_num_dpp = 4;
    ip.max_num_otg = 4;
    ip.max_num_hdmi_frl_outputs = 1;
    ip.max_num_wb = 1;
    ip.max_dchub_pscl_bw_pix_per_clk = 4;
    ip.max_pscl_lb_bw_pix_per_clk = 2;
    ip.max_lb_vscl_bw_pix_per_clk = 4;
    ip.max_vscl_hscl_bw_pix_per_clk = 4;
    ip.max_hscl_ratio = 6;
    ip.max_vscl_ratio = 6;
    ip.max_hscl_taps = 8;
    ip.max_vscl_taps = 8;
    ip.dpte_buffer_size_in_pte_reqs_luma = 64;
    ip.dpte_buffer_size_in_pte_reqs_chroma = 34;
    ip.dispclk_ramp_margin_percent = 1;
    ip.max_inter_dcn_tile_repeaters = 8;
    ip.cursor_buffer_size = 16;
    ip.cursor_chunk_size = 2;
    ip.writeback_line_buffer_buffer_size = 0;
    ip.writeback_min_hscl_ratio = 1;
    ip.writeback_min_vscl_ratio = 1;
    ip.writeback_max_hscl_ratio = 1;
    ip.writeback_max_vscl_ratio = 1;
    ip.writeback_max_hscl_taps = 1;
    ip.writeback_max_vscl_taps = 1;
    ip.dppclk_delay_subtotal = 47;
    ip.dppclk_delay_scl = 50;
    ip.dppclk_delay_scl_lb_only = 16;
    ip.dppclk_delay_cnvc_formatter = 28;
    ip.dppclk_delay_cnvc_cursor = 6;
    ip.dispclk_delay_subtotal = 125;
    ip.dynamic_metadata_vm_enabled = false;
    ip.odm_combine_4to1_supported = false;
    ip.dcc_supported = true;
    ip.max_num_dp2p0_outputs = 2;
    ip.max_num_dp2p0_streams = 4;
    // dcn3_2_soc (dcn32_fpu.c).
    soc.clock_limits[0].state = 0;
    soc.clock_limits[0].dcfclk_mhz = 1564.0;
    soc.clock_limits[0].fabricclk_mhz = 2500.0;
    soc.clock_limits[0].dispclk_mhz = 2150.0;
    soc.clock_limits[0].dppclk_mhz = 2150.0;
    soc.clock_limits[0].phyclk_mhz = 810.0;
    soc.clock_limits[0].phyclk_d18_mhz = 667.0;
    soc.clock_limits[0].phyclk_d32_mhz = 625.0;
    soc.clock_limits[0].socclk_mhz = 1200.0;
    soc.clock_limits[0].dscclk_mhz = 716.667;
    soc.clock_limits[0].dram_speed_mts = 18000.0;
    soc.clock_limits[0].dtbclk_mhz = 1564.0;
    soc.num_states = 1;
    soc.sr_exit_time_us = 42.97;
    soc.sr_enter_plus_exit_time_us = 49.94;
    soc.sr_exit_z8_time_us = 285.0;
    soc.sr_enter_plus_exit_z8_time_us = 320;
    soc.writeback_latency_us = 12.0;
    soc.round_trip_ping_latency_dcfclk_cycles = 263;
    soc.urgent_latency_pixel_data_only_us = 4.0;
    soc.urgent_latency_pixel_mixed_with_vm_data_us = 4.0;
    soc.urgent_latency_vm_data_only_us = 4.0;
    soc.fclk_change_latency_us = 25;
    soc.usr_retraining_latency_us = 2;
    soc.smn_latency_us = 2;
    soc.mall_allocated_for_dcn_mbytes = 64;
    soc.urgent_out_of_order_return_per_channel_pixel_only_bytes = 4096;
    soc.urgent_out_of_order_return_per_channel_pixel_and_vm_bytes = 4096;
    soc.urgent_out_of_order_return_per_channel_vm_only_bytes = 4096;
    soc.pct_ideal_sdp_bw_after_urgent = 90.0;
    soc.pct_ideal_fabric_bw_after_urgent = 67.0;
    soc.pct_ideal_dram_sdp_bw_after_urgent_pixel_only = 20.0;
    soc.pct_ideal_dram_sdp_bw_after_urgent_pixel_and_vm = 60.0;
    soc.pct_ideal_dram_sdp_bw_after_urgent_vm_only = 30.0;
    soc.pct_ideal_dram_bw_after_urgent_strobe = 67.0;
    soc.max_avg_sdp_bw_use_normal_percent = 80.0;
    soc.max_avg_fabric_bw_use_normal_percent = 60.0;
    soc.max_avg_dram_bw_use_normal_strobe_percent = 50.0;
    soc.max_avg_dram_bw_use_normal_percent = 15.0;
    soc.num_chans = 24;
    soc.dram_channel_width_bytes = 2;
    soc.fabric_datapath_to_dcn_data_return_bytes = 64;
    soc.return_bus_width_bytes = 64;
    soc.downspread_percent = 0.38;
    soc.dcn_downspread_percent = 0.5;
    soc.dram_clock_change_latency_us = 400;
    soc.dispclk_dppclk_vco_speed_mhz = 4300.0;
    soc.do_urgent_latency_adjustment = true;
    soc.urgent_latency_adjustment_fabric_clock_component_us = 1.0;
    soc.urgent_latency_adjustment_fabric_clock_reference_mhz = 3000;
}

// calculate_net_bw_in_kbytes_sec (dcn32_fpu.c), float arithmetic as upstream.
float calculate_net_bw_in_kbytes_sec(const soc_bounding_box_st& soc, const voltage_scaling_st& entry) noexcept {
    float memory_bw_kbytes_sec;
    float fabric_bw_kbytes_sec;
    float sdp_bw_kbytes_sec;
    float limiting_bw_kbytes_sec;

    memory_bw_kbytes_sec = entry.dram_speed_mts * soc.num_chans * soc.dram_channel_width_bytes *
                           (static_cast<float>(soc.pct_ideal_dram_sdp_bw_after_urgent_pixel_only) / 100);
    fabric_bw_kbytes_sec = entry.fabricclk_mhz * soc.return_bus_width_bytes *
                           (static_cast<float>(soc.pct_ideal_fabric_bw_after_urgent) / 100);
    sdp_bw_kbytes_sec = entry.dcfclk_mhz * soc.return_bus_width_bytes *
                        (static_cast<float>(soc.pct_ideal_sdp_bw_after_urgent) / 100);
    limiting_bw_kbytes_sec = memory_bw_kbytes_sec;

    if (fabric_bw_kbytes_sec < limiting_bw_kbytes_sec) {
        limiting_bw_kbytes_sec = fabric_bw_kbytes_sec;
    }

    if (sdp_bw_kbytes_sec < limiting_bw_kbytes_sec) {
        limiting_bw_kbytes_sec = sdp_bw_kbytes_sec;
    }

    return limiting_bw_kbytes_sec;
}

// get_optimal_ntuple (dcn32_fpu.c). Expressions are kept exactly as upstream writes
// them: on an x87 target the excess-precision behaviour depends on their shape.
void get_optimal_ntuple(const soc_bounding_box_st& soc, voltage_scaling_st& entry) noexcept {
    if (entry.dcfclk_mhz > 0) {
        float bw_on_sdp = entry.dcfclk_mhz * soc.return_bus_width_bytes *
                          (static_cast<float>(soc.pct_ideal_sdp_bw_after_urgent) / 100);

        entry.fabricclk_mhz = bw_on_sdp / (soc.return_bus_width_bytes *
                                           (static_cast<float>(soc.pct_ideal_fabric_bw_after_urgent) / 100));
        entry.dram_speed_mts = bw_on_sdp / (soc.num_chans * soc.dram_channel_width_bytes *
                                            (static_cast<float>(soc.pct_ideal_dram_sdp_bw_after_urgent_pixel_only) / 100));
    } else if (entry.fabricclk_mhz > 0) {
        float bw_on_fabric = entry.fabricclk_mhz * soc.return_bus_width_bytes *
                             (static_cast<float>(soc.pct_ideal_fabric_bw_after_urgent) / 100);

        entry.dcfclk_mhz = bw_on_fabric / (soc.return_bus_width_bytes *
                                           (static_cast<float>(soc.pct_ideal_sdp_bw_after_urgent) / 100));
        entry.dram_speed_mts = bw_on_fabric / (soc.num_chans * soc.dram_channel_width_bytes *
                                               (static_cast<float>(soc.pct_ideal_dram_sdp_bw_after_urgent_pixel_only) / 100));
    } else if (entry.dram_speed_mts > 0) {
        float bw_on_dram = entry.dram_speed_mts * soc.num_chans * soc.dram_channel_width_bytes *
                           (static_cast<float>(soc.pct_ideal_dram_sdp_bw_after_urgent_pixel_only) / 100);

        entry.fabricclk_mhz = bw_on_dram / (soc.return_bus_width_bytes *
                                            (static_cast<float>(soc.pct_ideal_fabric_bw_after_urgent) / 100));
        entry.dcfclk_mhz = bw_on_dram / (soc.return_bus_width_bytes *
                                         (static_cast<float>(soc.pct_ideal_sdp_bw_after_urgent) / 100));
    }
}

// insert_entry_into_table_sorted. The caller guarantees room for one more entry.
void insert_entry_into_table_sorted(voltage_scaling_st* table, unsigned int& num_entries,
                                    const voltage_scaling_st& entry) noexcept {
    if (num_entries == 0) {
        table[0] = entry;
        ++num_entries;
        return;
    }

    unsigned int index = 0;

    while (entry.net_bw_in_kbytes_sec > table[index].net_bw_in_kbytes_sec) {
        ++index;

        if (index >= num_entries) {
            break;
        }
    }

    for (unsigned int i = num_entries; i > index; --i) {
        table[i] = table[i - 1];
    }

    table[index] = entry;
    ++num_entries;
}

// remove_entry_from_table_at_index.
void remove_entry_from_table_at_index(voltage_scaling_st* table, unsigned int& num_entries,
                                      unsigned int index) noexcept {
    if (num_entries == 0) {
        return;
    }

    for (unsigned int i = index; i + 1 < num_entries; ++i) {
        table[i] = table[i + 1];
    }

    --num_entries;
    dml_zero(table[num_entries]);
}

// sort_entries_with_same_bw (the upstream index arithmetic is kept, including the
// int/unsigned mixture of its loop bounds).
void sort_entries_with_same_bw(voltage_scaling_st* table, unsigned int& num_entries) noexcept {
    unsigned int start_index = 0;
    unsigned int end_index = 0;
    unsigned int current_bw = 0;

    for (int i = 0; i < static_cast<int>(num_entries - 1); i++) {
        if (table[i].net_bw_in_kbytes_sec == table[i + 1].net_bw_in_kbytes_sec) {
            current_bw = static_cast<unsigned int>(table[i].net_bw_in_kbytes_sec);
            start_index = static_cast<unsigned int>(i);
            end_index = static_cast<unsigned int>(++i);

            while ((i < static_cast<int>(num_entries - 1)) &&
                   (table[i + 1].net_bw_in_kbytes_sec == current_bw)) {
                end_index = static_cast<unsigned int>(++i);
            }
        }

        if (start_index != end_index) {
            for (unsigned int j = start_index; j < end_index; j++) {
                for (unsigned int k = start_index; k < end_index; k++) {
                    if (table[k].dcfclk_mhz > table[k + 1].dcfclk_mhz) {
                        const voltage_scaling_st swap = table[k];
                        table[k] = table[k + 1];
                        table[k + 1] = swap;
                    }
                }
            }
        }

        start_index = 0;
        end_index = 0;
    }
}

// remove_inconsistent_entries.
void remove_inconsistent_entries(voltage_scaling_st* table, unsigned int& num_entries) noexcept {
    for (int i = 0; i < static_cast<int>(num_entries - 1); i++) {
        if (table[i].net_bw_in_kbytes_sec == table[i + 1].net_bw_in_kbytes_sec) {
            if ((table[i].dram_speed_mts > table[i + 1].dram_speed_mts) ||
                (table[i].fabricclk_mhz > table[i + 1].fabricclk_mhz)) {
                remove_entry_from_table_at_index(table, num_entries, static_cast<unsigned int>(i));
            }
        }
    }
}

// override_max_clk_values.
void override_max_clk_values(const Dcn32ClockLevel& max_clk_limit, Dcn32ClockLevel& curr_clk_limit) noexcept {
    if (max_clk_limit.dcfclk_mhz != 0) {
        curr_clk_limit.dcfclk_mhz = max_clk_limit.dcfclk_mhz;
    }

    if (max_clk_limit.fclk_mhz != 0) {
        curr_clk_limit.fclk_mhz = max_clk_limit.fclk_mhz;
    }

    if (max_clk_limit.memclk_mhz != 0) {
        curr_clk_limit.memclk_mhz = max_clk_limit.memclk_mhz;
    }

    if (max_clk_limit.socclk_mhz != 0) {
        curr_clk_limit.socclk_mhz = max_clk_limit.socclk_mhz;
    }

    if (max_clk_limit.dtbclk_mhz != 0) {
        curr_clk_limit.dtbclk_mhz = max_clk_limit.dtbclk_mhz;
    }

    if (max_clk_limit.dispclk_mhz != 0) {
        curr_clk_limit.dispclk_mhz = max_clk_limit.dispclk_mhz;
    }
}

void insert_ntuple(soc_bounding_box_st& soc, unsigned int& num_entries, voltage_scaling_st& entry) noexcept {
    get_optimal_ntuple(soc, entry);
    entry.net_bw_in_kbytes_sec = calculate_net_bw_in_kbytes_sec(soc, entry);
    insert_entry_into_table_sorted(soc.clock_limits, num_entries, entry);
}

// build_synthetic_soc_states with disable_dc_mode_overwrite = false (the dcn32
// default), building in place into soc.clock_limits as upstream does. Returns false
// where upstream returns -1 (and keeps the previous bounding box).
bool build_synthetic_soc_states(Dcn32BandwidthWorkspace& w) noexcept {
    soc_bounding_box_st& soc = w.soc;
    Dcn32ClockTable& table = w.clocks;
    voltage_scaling_st entry{};
    Dcn32ClockLevel max_clk_data{};
    unsigned int min_dcfclk_mhz = 199;
    unsigned int min_fclk_mhz = 299;
    constexpr unsigned int num_dcfclk_stas = 5;
    const unsigned int dcfclk_sta_targets[num_dcfclk_stas]{199, 615, 906, 1324, 1564};
    unsigned int num_uclk_dpms = 0;
    unsigned int num_fclk_dpms = 0;
    unsigned int num_dcfclk_dpms = 0;
    unsigned int num_dc_uclk_dpms = 0;
    unsigned int num_dc_fclk_dpms = 0;
    unsigned int num_dc_dcfclk_dpms = 0;

    for (uint32_t i = 0; i < dcn32_max_dpm_levels; i++) {
        const Dcn32ClockLevel& level = table.levels[i];

        if (level.dcfclk_mhz > max_clk_data.dcfclk_mhz) {
            max_clk_data.dcfclk_mhz = level.dcfclk_mhz;
        }

        if (level.fclk_mhz > max_clk_data.fclk_mhz) {
            max_clk_data.fclk_mhz = level.fclk_mhz;
        }

        if (level.memclk_mhz > max_clk_data.memclk_mhz) {
            max_clk_data.memclk_mhz = level.memclk_mhz;
        }

        if (level.dispclk_mhz > max_clk_data.dispclk_mhz) {
            max_clk_data.dispclk_mhz = level.dispclk_mhz;
        }

        if (level.dppclk_mhz > max_clk_data.dppclk_mhz) {
            max_clk_data.dppclk_mhz = level.dppclk_mhz;
        }

        if (level.phyclk_mhz > max_clk_data.phyclk_mhz) {
            max_clk_data.phyclk_mhz = level.phyclk_mhz;
        }

        if (level.dtbclk_mhz > max_clk_data.dtbclk_mhz) {
            max_clk_data.dtbclk_mhz = level.dtbclk_mhz;
        }

        if (level.memclk_mhz > 0) {
            num_uclk_dpms++;

            if (level.memclk_mhz <= table.dc_mode_limit.memclk_mhz) {
                num_dc_uclk_dpms++;
            }
        }

        if (level.fclk_mhz > 0) {
            num_fclk_dpms++;

            if (level.fclk_mhz <= table.dc_mode_limit.fclk_mhz) {
                num_dc_fclk_dpms++;
            }
        }

        if (level.dcfclk_mhz > 0) {
            num_dcfclk_dpms++;

            if (level.dcfclk_mhz <= table.dc_mode_limit.dcfclk_mhz) {
                num_dc_dcfclk_dpms++;
            }
        }
    }

    // !disable_dc_mode_overwrite
    override_max_clk_values(table.dc_mode_limit, max_clk_data);
    num_uclk_dpms = num_dc_uclk_dpms;
    num_fclk_dpms = num_dc_fclk_dpms;
    num_dcfclk_dpms = num_dc_dcfclk_dpms;
    w.num_memclk_levels = num_uclk_dpms;

    if (num_dcfclk_dpms > 0 && table.levels[0].fclk_mhz > min_fclk_mhz) {
        min_fclk_mhz = table.levels[0].fclk_mhz;
    }

    if (max_clk_data.dcfclk_mhz == 0 || max_clk_data.dispclk_mhz == 0 || max_clk_data.dtbclk_mhz == 0) {
        return false;
    }

    if (max_clk_data.dppclk_mhz == 0) {
        max_clk_data.dppclk_mhz = max_clk_data.dispclk_mhz;
    }

    if (max_clk_data.fclk_mhz == 0) {
        max_clk_data.fclk_mhz = static_cast<unsigned int>(max_clk_data.dcfclk_mhz *
                                                          soc.pct_ideal_sdp_bw_after_urgent /
                                                          soc.pct_ideal_fabric_bw_after_urgent);
    }

    if (max_clk_data.phyclk_mhz == 0) {
        max_clk_data.phyclk_mhz = static_cast<unsigned int>(soc.clock_limits[0].phyclk_mhz);
    }

    unsigned int num_entries = 0;
    entry.dispclk_mhz = max_clk_data.dispclk_mhz;
    entry.dscclk_mhz = max_clk_data.dispclk_mhz / 3;
    entry.dppclk_mhz = max_clk_data.dppclk_mhz;
    entry.dtbclk_mhz = max_clk_data.dtbclk_mhz;
    entry.phyclk_mhz = max_clk_data.phyclk_mhz;
    entry.phyclk_d18_mhz = soc.clock_limits[0].phyclk_d18_mhz;
    entry.phyclk_d32_mhz = soc.clock_limits[0].phyclk_d32_mhz;

    for (unsigned int i = 0; i < num_dcfclk_stas; i++) {
        entry.dcfclk_mhz = dcfclk_sta_targets[i];
        entry.fabricclk_mhz = 0;
        entry.dram_speed_mts = 0;
        insert_ntuple(soc, num_entries, entry);
    }

    entry.dcfclk_mhz = max_clk_data.dcfclk_mhz;
    entry.fabricclk_mhz = 0;
    entry.dram_speed_mts = 0;
    insert_ntuple(soc, num_entries, entry);

    for (unsigned int i = 0; i < num_uclk_dpms; i++) {
        entry.dcfclk_mhz = 0;
        entry.fabricclk_mhz = 0;
        entry.dram_speed_mts = table.levels[i].memclk_mhz * 16;
        insert_ntuple(soc, num_entries, entry);
    }

    if (num_fclk_dpms > 2) {
        for (unsigned int i = 0; i < num_fclk_dpms; i++) {
            entry.dcfclk_mhz = 0;
            entry.fabricclk_mhz = table.levels[i].fclk_mhz;
            entry.dram_speed_mts = 0;
            insert_ntuple(soc, num_entries, entry);
        }
    } else {
        entry.dcfclk_mhz = 0;
        entry.fabricclk_mhz = max_clk_data.fclk_mhz;
        entry.dram_speed_mts = 0;
        insert_ntuple(soc, num_entries, entry);
    }

    for (int i = static_cast<int>(num_entries) - 1; i >= 0; i--) {
        if (soc.clock_limits[i].dcfclk_mhz > max_clk_data.dcfclk_mhz ||
            soc.clock_limits[i].fabricclk_mhz > max_clk_data.fclk_mhz ||
            soc.clock_limits[i].dram_speed_mts > max_clk_data.memclk_mhz * 16) {
            remove_entry_from_table_at_index(soc.clock_limits, num_entries, static_cast<unsigned int>(i));
        }
    }

    {
        voltage_scaling_st max_dc_limits_entry = entry;

        max_dc_limits_entry.dcfclk_mhz = max_clk_data.dcfclk_mhz;
        max_dc_limits_entry.fabricclk_mhz = max_clk_data.fclk_mhz;
        max_dc_limits_entry.dram_speed_mts = max_clk_data.memclk_mhz * 16;
        max_dc_limits_entry.net_bw_in_kbytes_sec = calculate_net_bw_in_kbytes_sec(soc, max_dc_limits_entry);
        insert_entry_into_table_sorted(soc.clock_limits, num_entries, max_dc_limits_entry);
        sort_entries_with_same_bw(soc.clock_limits, num_entries);
        remove_inconsistent_entries(soc.clock_limits, num_entries);
    }

    for (int i = static_cast<int>(num_entries) - 1; i >= 0; i--) {
        for (unsigned int j = 0; j < num_uclk_dpms; j++) {
            if (table.levels[j].memclk_mhz * 16 >= soc.clock_limits[i].dram_speed_mts) {
                soc.clock_limits[i].dram_speed_mts = table.levels[j].memclk_mhz * 16;
                break;
            }
        }
    }

    if (num_fclk_dpms > 2) {
        for (int i = static_cast<int>(num_entries) - 1; i >= 0; i--) {
            for (unsigned int j = 0; j < num_fclk_dpms; j++) {
                if (table.levels[j].fclk_mhz >= soc.clock_limits[i].fabricclk_mhz) {
                    soc.clock_limits[i].fabricclk_mhz = table.levels[j].fclk_mhz;
                    break;
                }
            }
        }
    } else {
        for (int i = static_cast<int>(num_entries) - 1; i >= 0; i--) {
            if (soc.clock_limits[i].fabricclk_mhz < min_fclk_mhz) {
                soc.clock_limits[i].fabricclk_mhz = min_fclk_mhz;
            }
        }
    }

    for (int i = static_cast<int>(num_entries) - 1; i >= 0; i--) {
        if (soc.clock_limits[i].dcfclk_mhz < min_dcfclk_mhz) {
            soc.clock_limits[i].dcfclk_mhz = min_dcfclk_mhz;
        }
    }

    unsigned int i = 0;

    while (i + 1 < num_entries) {
        if (soc.clock_limits[i].dcfclk_mhz == soc.clock_limits[i + 1].dcfclk_mhz &&
            soc.clock_limits[i].fabricclk_mhz == soc.clock_limits[i + 1].fabricclk_mhz &&
            soc.clock_limits[i].dram_speed_mts == soc.clock_limits[i + 1].dram_speed_mts) {
            remove_entry_from_table_at_index(soc.clock_limits, num_entries, i + 1);
        } else {
            i++;
        }
    }

    for (unsigned int state = 0; state < num_entries; state++) {
        soc.clock_limits[state].state = state;
    }

    soc.num_states = num_entries;
    return true;
}

// dcn32_build_wm_range_table_fpu: latencies come from dc->dml.soc after the
// bounding box update; WM C uses the dummy p-state latencies of the first four
// UCLK levels (bb_overrides.dummy_clock_change_latency_ns is not 0x7FFFFFFF).
void build_wm_range_table(Dcn32BandwidthWorkspace& w) noexcept {
    const double pstate_latency_us = w.soc.dram_clock_change_latency_us;
    const double fclk_change_latency_us = w.soc.fclk_change_latency_us;
    const double sr_exit_time_us = w.soc.sr_exit_time_us;
    const double sr_enter_plus_exit_time_us = w.soc.sr_enter_plus_exit_time_us;

    w.wm[wm_a] = {true, pstate_latency_us, sr_exit_time_us, sr_enter_plus_exit_time_us, fclk_change_latency_us};
    w.wm[wm_b] = w.wm[wm_a];
    w.wm[wm_c] = {true, 50, sr_exit_time_us, sr_enter_plus_exit_time_us, fclk_change_latency_us};
    w.dummy[0] = {w.clocks.levels[0].memclk_mhz * 16, 50};
    w.dummy[1] = {w.clocks.levels[1].memclk_mhz * 16, 9};
    w.dummy[2] = {w.clocks.levels[2].memclk_mhz * 16, 8};
    w.dummy[3] = {w.clocks.levels[3].memclk_mhz * 16, 5};
    w.wm[wm_d] = {true, static_cast<double>(w.dummy[3].dummy_pstate_latency_us), sr_exit_time_us / 2,
                  sr_enter_plus_exit_time_us / 2, fclk_change_latency_us};
}

// dcn20_populate_dml_pipes_from_context + dcn32_populate_dml_pipes_from_context for
// one stream and the pipes of w.topology in pipe index order (DML pipe n is the
// n-th pipe with a stream): scaler data of resource_build_scaling_params at 1:1
// (dpp3_get_optimal_number_of_taps gives 1 tap), DC_SW_LINEAR,
// SURFACE_PIXEL_FORMAT_GRPH_ARGB8888, RGB 8 bpc, no audio/DRR/writeback, DSC
// from the request, then dcn32_set_det_allocations (one linear pipe:
// DCN3_2_MAX_DET_SIZE and bounded requests; more pipes:
// dcn32_determine_det_override). Returns pipe_cnt.
DML32_NOINLINE void populate_pipe(Dcn32BandwidthWorkspace& w, const Dcn32PlaneRequest& request, uint32_t index,
                                  display_e2e_pipe_params_st& p) noexcept {
    const Timing& timing = request.timing;
    const Dcn32TopologyPipe& pipe = w.topology.pipes[index];
    const uint32_t odm = topology_odm_slice_count(w.topology);
    const Dcn32PlaneGeometry& plane = w.planes[pipe.plane];
    const bool dsc = request.dsc_slices_h != 0;

    dml_zero(p);
    p.clks_cfg.refclk_mhz = w.dchub_refclk_khz / 1000.0;
    p.pipe.dest.use_maximum_vstartup = false;
    p.dout.dsc_enable = dsc;
    p.dout.dsc_slices = static_cast<int>(request.dsc_slices_h);
    p.pipe.src.dcc = false;
    p.pipe.src.dcc_rate = 1;
    p.pipe.dest.synchronized_vblank_all_planes = true;
    p.pipe.dest.synchronize_timings = true;
    p.pipe.dest.hblank_start = timing.h_total - timing.h_front_porch;
    p.pipe.dest.hblank_end = p.pipe.dest.hblank_start - timing.width;
    p.pipe.dest.vblank_start = timing.v_total - timing.v_front_porch;
    p.pipe.dest.vblank_end = p.pipe.dest.vblank_start - timing.height;
    p.pipe.dest.htotal = timing.h_total;
    p.pipe.dest.vtotal = timing.v_total;
    p.pipe.dest.hactive = timing.width;
    p.pipe.dest.vactive = timing.height;
    p.pipe.dest.interlaced = 0;
    // pix_clk_100hz / 10000.0 with pix_clk_100hz = kHz * 10.
    p.pipe.dest.pixel_rate_mhz = (static_cast<double>(timing.pixel_clock_khz) * 10) / 10000.0;
    p.pipe.dest.otg_inst = static_cast<unsigned char>(request.otg_instance);
    p.dout.dp_lanes = 4;
    p.dout.dp_rate = dm_dp_rate_na;
    p.dout.is_virtual = 0;
    p.pipe.dest.vtotal_min = 0;
    p.pipe.dest.vtotal_max = 0;
    p.pipe.dest.odm_combine = odm == 2 ? dm_odm_combine_mode_2to1 :
                              odm == 4 ? dm_odm_combine_mode_4to1 : dm_odm_combine_mode_disabled;
    p.pipe.src.hsplit_grp = static_cast<int>(index);
    if (pipe.prev_odm != dcn32_no_pipe) {
        uint32_t first = pipe.prev_odm;
        uint32_t guard = 0;
        while (w.topology.pipes[first].prev_odm != dcn32_no_pipe && guard++ < dcn32_topology_pipes) {
            first = w.topology.pipes[first].prev_odm;
        }
        p.pipe.src.hsplit_grp = static_cast<int>(first);
    }
    p.dout.output_type = request.signal == Dcn32OutputSignal::tmds ? dm_hdmi : dm_dp;
    p.dout.output_format = dm_444;
    p.dout.output_bpp = 8 * 3;
    if (dsc) {
        p.dout.output_bpp = request.dsc_bits_per_pixel_x16 / 16.0;
    }
    p.dout.max_audio_sample_rate = 0;
    p.pipe.src.num_cursors = w.ip.number_of_cursors;
    p.pipe.src.cur0_src_width = 256;
    p.pipe.src.cur0_bpp = dm_cur_32bit;
    p.pipe.src.immediate_flip = false;
    // MPC combine never happens (MPC_SPLIT_AVOID), so only ODM splits a plane.
    p.pipe.src.is_hsplit = p.pipe.dest.odm_combine != dm_odm_combine_mode_disabled;
    p.pipe.src.source_scan = dm_horz;
    p.pipe.src.source_rotation = dm_rotation_0;
    p.pipe.src.viewport_y_y = pipe.viewport.y;
    p.pipe.src.viewport_y_c = pipe.viewport.y;
    p.pipe.src.viewport_x_y = pipe.viewport.x;
    p.pipe.src.viewport_x_c = pipe.viewport.x;
    p.pipe.src.viewport_width = pipe.viewport.width;
    p.pipe.src.viewport_width_c = pipe.viewport.width;
    p.pipe.src.viewport_height = pipe.viewport.height;
    p.pipe.src.viewport_height_c = pipe.viewport.height;
    p.pipe.src.viewport_width_max = static_cast<int>(plane.surface_width);
    p.pipe.src.viewport_height_max = static_cast<int>(plane.surface_height);
    p.pipe.src.surface_width_y = plane.surface_width;
    p.pipe.src.surface_height_y = plane.surface_height;
    p.pipe.src.data_pitch = w.plane_pitch[pipe.plane];
    p.pipe.dest.recout_width = pipe.recout.width;
    p.pipe.dest.recout_height = pipe.recout.height;
    p.pipe.dest.full_recout_width = pipe.recout.width;
    p.pipe.dest.full_recout_height = pipe.recout.height;
    if (p.pipe.dest.odm_combine == dm_odm_combine_mode_2to1) {
        p.pipe.dest.full_recout_width *= 2;
    } else if (p.pipe.dest.odm_combine == dm_odm_combine_mode_4to1) {
        p.pipe.dest.full_recout_width *= 4;
    }
    p.pipe.scale_ratio_depth.hscl_ratio = 1.0;
    p.pipe.scale_ratio_depth.hscl_ratio_c = 1.0;
    p.pipe.scale_ratio_depth.vscl_ratio = 1.0;
    p.pipe.scale_ratio_depth.vscl_ratio_c = 1.0;
    p.pipe.scale_ratio_depth.scl_enable = 0;
    p.pipe.scale_taps.htaps = 1;
    p.pipe.scale_taps.htaps_c = 1;
    p.pipe.scale_taps.vtaps = 1;
    p.pipe.scale_taps.vtaps_c = 1;
    p.pipe.src.macro_tile_size = dm_4k_tile;
    p.pipe.src.sw_mode = dm_sw_linear;
    p.pipe.src.source_format = dm_444_32;
    // dcn32_populate_dml_pipes_from_context (enable_windowed_mpo_odm and
    // enable_single_display_2to1_odm_policy: the policy follows the slice count).
    p.pipe.src.gpuvm = true;
    p.pipe.src.dcc_fraction_of_zs_req_luma = 0;
    p.pipe.src.dcc_fraction_of_zs_req_chroma = 0;
    p.pipe.dest.vfront_porch = timing.v_front_porch;
    p.pipe.dest.odm_combine_policy = odm == 2 ? dm_odm_combine_policy_2to1 :
                                     odm == 4 ? dm_odm_combine_policy_4to1 : dm_odm_combine_policy_dal;
    p.pipe.src.gpuvm_min_page_size_kbytes = 256;
    p.pipe.src.unbounded_req_mode = false;
    p.pipe.scale_ratio_depth.lb_depth = dm_lb_19;
    p.pipe.src.use_mall_for_pstate_change = dm_use_mall_pstate_change_disable;
    p.pipe.src.use_mall_for_static_screen = dm_use_mall_static_screen_disable;
    // COLOR_DEPTH_888.
    p.dout.dsc_input_bpc = dsc ? 8 : 0;
}

unsigned int populate_pipes(Dcn32BandwidthWorkspace& w, const Dcn32PlaneRequest& request) noexcept {
    unsigned int pipe_cnt = 0;

    for (uint32_t i = 0; i < dcn32_topology_pipes; i++) {
        if (!w.topology.pipes[i].stream) {
            continue;
        }
        populate_pipe(w, request, i, w.pipes[pipe_cnt]);
        pipe_cnt++;
    }

    if (pipe_cnt == 1) {
        w.pipes[0].pipe.src.det_size_override = dcn3_2_max_det_size;
        return pipe_cnt;
    }

    // dcn32_determine_det_override: 18 segments for the stream, split equally
    // among its planes, then among the pipes of each plane.
    uint8_t pipe_segments[dcn32_topology_pipes] = {0, 0, 0, 0};
    uint8_t pipe_counted[dcn32_topology_pipes] = {0, 0, 0, 0};
    const uint8_t stream_segments = 18;
    const uint8_t plane_segments = static_cast<uint8_t>(
        w.topology.plane_count > 0 ? stream_segments / w.topology.plane_count : stream_segments);
    const Dcn32Topology& t = w.topology;

    for (uint32_t j = 0; j < dcn32_topology_pipes; j++) {
        uint8_t pipe_plane_count = 0;

        if (t.pipes[j].stream && pipe_counted[j] != 1) {
            pipe_plane_count++;
            pipe_counted[j] = 1;
            for (uint32_t k = 0; k < dcn32_topology_pipes; k++) {
                if (k != j && t.pipes[k].stream && t.pipes[k].has_plane == t.pipes[j].has_plane &&
                    t.pipes[k].plane == t.pipes[j].plane) {
                    pipe_plane_count++;
                    pipe_counted[k] = 1;
                }
            }
            pipe_segments[j] = static_cast<uint8_t>(plane_segments / pipe_plane_count);
            for (uint32_t k = 0; k < dcn32_topology_pipes; k++) {
                if (k != j && t.pipes[k].stream && t.pipes[k].has_plane == t.pipes[j].has_plane &&
                    t.pipes[k].plane == t.pipes[j].plane) {
                    pipe_segments[k] = static_cast<uint8_t>(plane_segments / pipe_plane_count);
                }
            }
        }
    }

    for (uint32_t i = 0, n = 0; i < dcn32_topology_pipes; i++) {
        if (!t.pipes[i].stream) {
            continue;
        }
        w.pipes[n].pipe.src.det_size_override = pipe_segments[i] * 64u; // DCN3_2_DET_SEG_SIZE
        n++;
    }
    return pipe_cnt;
}

// double * 1000 stored into a uint32_t dcn_watermarks field. The conversion follows
// the x86 target (truncate to int64, keep the low 32 bits), which is what the
// Linux build performs and is defined in C++.
uint32_t to_u32(double value) noexcept {
    return static_cast<uint32_t>(static_cast<int64_t>(value));
}

void watermark_set(Dcn32BandwidthWorkspace& w, Dcn32Watermarks& set) noexcept {
    display_mode_lib* lib = &w.dml;
    const display_e2e_pipe_params_st* pipes = w.pipes;
    const int pipe_cnt = static_cast<int>(w.pipe_cnt);

    set.urgent_ns = to_u32(get_wm_urgent(lib, pipes, pipe_cnt) * 1000);
    set.cstate_enter_plus_exit_ns = to_u32(get_wm_stutter_enter_exit(lib, pipes, pipe_cnt) * 1000);
    set.cstate_exit_ns = to_u32(get_wm_stutter_exit(lib, pipes, pipe_cnt) * 1000);
    set.pstate_change_ns = to_u32(get_wm_dram_clock_change(lib, pipes, pipe_cnt) * 1000);
    set.pte_meta_urgent_ns = to_u32(get_wm_memory_trip(lib, pipes, pipe_cnt) * 1000);
    set.frac_urg_bw_nom = to_u32(get_fraction_of_urgent_bandwidth(lib, pipes, pipe_cnt) * 1000);
    set.frac_urg_bw_flip = to_u32(get_fraction_of_urgent_bandwidth_imm_flip(lib, pipes, pipe_cnt) * 1000);
    set.urgent_latency_ns = to_u32(get_urgent_latency(lib, pipes, pipe_cnt) * 1000);
    set.fclk_pstate_change_ns = to_u32(get_fclk_watermark(lib, pipes, pipe_cnt) * 1000);
    set.usr_retraining_ns = to_u32(get_usr_retraining_watermark(lib, pipes, pipe_cnt) * 1000);
}

void copy_dlg(const display_dlg_regs_st& in, Dcn32DlgRegs& out) noexcept {
    out.refcyc_h_blank_end = in.refcyc_h_blank_end;
    out.dlg_vblank_end = in.dlg_vblank_end;
    out.min_dst_y_next_start = in.min_dst_y_next_start;
    out.min_dst_y_next_start_us = in.min_dst_y_next_start_us;
    out.refcyc_per_htotal = in.refcyc_per_htotal;
    out.refcyc_x_after_scaler = in.refcyc_x_after_scaler;
    out.dst_y_after_scaler = in.dst_y_after_scaler;
    out.dst_y_prefetch = in.dst_y_prefetch;
    out.dst_y_per_vm_vblank = in.dst_y_per_vm_vblank;
    out.dst_y_per_row_vblank = in.dst_y_per_row_vblank;
    out.dst_y_per_vm_flip = in.dst_y_per_vm_flip;
    out.dst_y_per_row_flip = in.dst_y_per_row_flip;
    out.ref_freq_to_pix_freq = in.ref_freq_to_pix_freq;
    out.vratio_prefetch = in.vratio_prefetch;
    out.vratio_prefetch_c = in.vratio_prefetch_c;
    out.refcyc_per_tdlut_group = in.refcyc_per_tdlut_group;
    out.refcyc_per_pte_group_vblank_l = in.refcyc_per_pte_group_vblank_l;
    out.refcyc_per_pte_group_vblank_c = in.refcyc_per_pte_group_vblank_c;
    out.refcyc_per_meta_chunk_vblank_l = in.refcyc_per_meta_chunk_vblank_l;
    out.refcyc_per_meta_chunk_vblank_c = in.refcyc_per_meta_chunk_vblank_c;
    out.refcyc_per_pte_group_flip_l = in.refcyc_per_pte_group_flip_l;
    out.refcyc_per_pte_group_flip_c = in.refcyc_per_pte_group_flip_c;
    out.refcyc_per_meta_chunk_flip_l = in.refcyc_per_meta_chunk_flip_l;
    out.refcyc_per_meta_chunk_flip_c = in.refcyc_per_meta_chunk_flip_c;
    out.dst_y_per_pte_row_nom_l = in.dst_y_per_pte_row_nom_l;
    out.dst_y_per_pte_row_nom_c = in.dst_y_per_pte_row_nom_c;
    out.refcyc_per_pte_group_nom_l = in.refcyc_per_pte_group_nom_l;
    out.refcyc_per_pte_group_nom_c = in.refcyc_per_pte_group_nom_c;
    out.dst_y_per_meta_row_nom_l = in.dst_y_per_meta_row_nom_l;
    out.dst_y_per_meta_row_nom_c = in.dst_y_per_meta_row_nom_c;
    out.refcyc_per_meta_chunk_nom_l = in.refcyc_per_meta_chunk_nom_l;
    out.refcyc_per_meta_chunk_nom_c = in.refcyc_per_meta_chunk_nom_c;
    out.refcyc_per_line_delivery_pre_l = in.refcyc_per_line_delivery_pre_l;
    out.refcyc_per_line_delivery_pre_c = in.refcyc_per_line_delivery_pre_c;
    out.refcyc_per_line_delivery_l = in.refcyc_per_line_delivery_l;
    out.refcyc_per_line_delivery_c = in.refcyc_per_line_delivery_c;
    out.chunk_hdl_adjust_cur0 = in.chunk_hdl_adjust_cur0;
    out.chunk_hdl_adjust_cur1 = in.chunk_hdl_adjust_cur1;
    out.vready_after_vcount0 = in.vready_after_vcount0;
    out.dst_y_offset_cur0 = in.dst_y_offset_cur0;
    out.dst_y_offset_cur1 = in.dst_y_offset_cur1;
    out.dst_y_delta_drq_limit = in.dst_y_delta_drq_limit;
    out.refcyc_per_vm_group_vblank = in.refcyc_per_vm_group_vblank;
    out.refcyc_per_vm_group_flip = in.refcyc_per_vm_group_flip;
    out.refcyc_per_vm_req_vblank = in.refcyc_per_vm_req_vblank;
    out.refcyc_per_vm_req_flip = in.refcyc_per_vm_req_flip;
    out.refcyc_per_vm_dmdata = in.refcyc_per_vm_dmdata;
    out.dmdata_dl_delta = in.dmdata_dl_delta;
}

void copy_ttu(const display_ttu_regs_st& in, Dcn32TtuRegs& out) noexcept {
    out.qos_level_low_wm = in.qos_level_low_wm;
    out.qos_level_high_wm = in.qos_level_high_wm;
    out.min_ttu_vblank = in.min_ttu_vblank;
    out.qos_level_flip = in.qos_level_flip;
    out.refcyc_per_req_delivery_l = in.refcyc_per_req_delivery_l;
    out.refcyc_per_req_delivery_c = in.refcyc_per_req_delivery_c;
    out.refcyc_per_req_delivery_cur0 = in.refcyc_per_req_delivery_cur0;
    out.refcyc_per_req_delivery_cur1 = in.refcyc_per_req_delivery_cur1;
    out.refcyc_per_req_delivery_pre_l = in.refcyc_per_req_delivery_pre_l;
    out.refcyc_per_req_delivery_pre_c = in.refcyc_per_req_delivery_pre_c;
    out.refcyc_per_req_delivery_pre_cur0 = in.refcyc_per_req_delivery_pre_cur0;
    out.refcyc_per_req_delivery_pre_cur1 = in.refcyc_per_req_delivery_pre_cur1;
    out.qos_level_fixed_l = in.qos_level_fixed_l;
    out.qos_level_fixed_c = in.qos_level_fixed_c;
    out.qos_level_fixed_cur0 = in.qos_level_fixed_cur0;
    out.qos_level_fixed_cur1 = in.qos_level_fixed_cur1;
    out.qos_ramp_disable_l = in.qos_ramp_disable_l;
    out.qos_ramp_disable_c = in.qos_ramp_disable_c;
    out.qos_ramp_disable_cur0 = in.qos_ramp_disable_cur0;
    out.qos_ramp_disable_cur1 = in.qos_ramp_disable_cur1;
}

void copy_data_rq(const display_data_rq_regs_st& in, Dcn32DataRqRegs& out) noexcept {
    out.chunk_size = in.chunk_size;
    out.min_chunk_size = in.min_chunk_size;
    out.meta_chunk_size = in.meta_chunk_size;
    out.min_meta_chunk_size = in.min_meta_chunk_size;
    out.dpte_group_size = in.dpte_group_size;
    out.mpte_group_size = in.mpte_group_size;
    out.swath_height = in.swath_height;
    out.pte_row_height_linear = in.pte_row_height_linear;
}

void copy_rq(const display_rq_regs_st& in, Dcn32RqRegs& out) noexcept {
    copy_data_rq(in.rq_regs_l, out.rq_regs_l);
    copy_data_rq(in.rq_regs_c, out.rq_regs_c);
    out.drq_expansion_mode = in.drq_expansion_mode;
    out.prq_expansion_mode = in.prq_expansion_mode;
    out.mrq_expansion_mode = in.mrq_expansion_mode;
    out.crq_expansion_mode = in.crq_expansion_mode;
    out.plane1_base_address = in.plane1_base_address;
}

bool valid_request(const Dcn32PlaneRequest& request) noexcept {
    OptcTiming fields{};

    if (prepare_optc_timing(request.timing, fields) != Status::success || request.otg_instance >= 4 ||
        !(request.signal == Dcn32OutputSignal::display_port || request.signal == Dcn32OutputSignal::tmds)) {
        return false;
    }

    // DSC only on DP 8b/10b here (HDMI FRL DSC is not supported by this owner).
    if ((request.dsc_slices_h != 0) != (request.dsc_bits_per_pixel_x16 != 0) ||
        (request.dsc_slices_h != 0 && request.signal != Dcn32OutputSignal::display_port)) {
        return false;
    }

    if (request.plane_count == 0) {
        return request.pitch_pixels >= request.timing.width;
    }

    if (request.plane_count > dcn32_max_planes) {
        return false;
    }

    // Unscaled planes inside the active area, pitch covering the surface.
    for (uint32_t i = 0; i < request.plane_count; i++) {
        const Dcn32PlaneGeometry& plane = request.planes[i];

        if (plane.surface_width == 0 || plane.surface_height == 0 || plane.dst.x < 0 || plane.dst.y < 0 ||
            plane.dst.width != static_cast<int32_t>(plane.surface_width) ||
            plane.dst.height != static_cast<int32_t>(plane.surface_height) ||
            static_cast<uint32_t>(plane.dst.x) + plane.surface_width > request.timing.width ||
            static_cast<uint32_t>(plane.dst.y) + plane.surface_height > request.timing.height ||
            request.plane_pitch_pixels[i] < plane.surface_width) {
            return false;
        }
    }

    return true;
}

// is_h_timing_divisible_by_2 (uint16_t arithmetic as in dc_resource.c).
bool h_timing_divisible_by_2(const Timing& timing) noexcept {
    const uint16_t h_blank_start = static_cast<uint16_t>(timing.h_total - timing.h_front_porch);
    const uint16_t h_blank_end = static_cast<uint16_t>(h_blank_start - timing.width);

    return (timing.h_total % 2 == 0) && (h_blank_start % 2 == 0) && (h_blank_end % 2 == 0) &&
           (timing.h_sync_width % 2 == 0);
}

// dcn20_validate_apply_pipe_split_flags for one stream with MPC_SPLIT_AVOID
// (avoid_split is always set), no forced split, 3D or forced ODM.
unsigned int apply_pipe_split_flags(Dcn32BandwidthWorkspace& w, unsigned int vlevel) noexcept {
    vba_vars_st& v = w.dml.vba;
    const Dcn32Topology& t = w.topology;
    int max_mpc_comb = v.maxMpcComb;

    // Avoid split loop: lowest voltage level that allows most unsplit pipes.
    for (uint32_t i = 0, pipe_idx = 0; i < dcn32_topology_pipes; i++) {
        if (!t.pipes[i].stream) {
            continue;
        }

        const unsigned int vlevel_split = vlevel;

        for (; vlevel <= w.dml.soc.num_states; vlevel++) {
            if (v.NoOfDPP[vlevel][0][pipe_idx] == 1 && v.ModeSupport[vlevel][0]) {
                break;
            }
        }

        // Impossible to not split this pipe.
        if (vlevel > w.dml.soc.num_states) {
            vlevel = vlevel_split;
        } else {
            max_mpc_comb = 0;
        }

        pipe_idx++;
    }

    v.maxMpcComb = max_mpc_comb;

    // Split loop.
    for (uint32_t i = 0, pipe_idx = 0; i < dcn32_topology_pipes; i++) {
        if (!t.pipes[i].stream) {
            continue;
        }

        const unsigned int pipe_plane = v.pipe_plane[pipe_idx];
        const uint32_t odm_slices = topology_odm_slice_count(t);
        const Dcn32TopologyPipe& pipe = t.pipes[i];

        if (v.NoOfDPP[vlevel][max_mpc_comb][pipe_plane] == 4) {
            w.split[i] = 4;
        } else if (v.NoOfDPP[vlevel][max_mpc_comb][pipe_plane] == 2) {
            w.split[i] = 2;
        }

        v.ODMCombineEnabled[pipe_plane] = v.ODMCombineEnablePerState[vlevel][pipe_plane];

        if (v.ODMCombineEnabled[pipe_plane] == dm_odm_combine_mode_disabled) {
            // ODM -> MPC transition (MPC slice counts are always one here).
            if (odm_slices > 1 && pipe.prev_odm != dcn32_no_pipe) {
                w.split[i] = 0;
                w.merge[i] = true;
            }
        } else if (odm_slices == 2) {
            if (w.split[i] == 4) {
                w.split[i] = 2;
            } else if (w.split[i] == 2) {
                w.split[i] = 0;
            } else if (pipe.prev_odm != dcn32_no_pipe) {
                w.merge[i] = true;
            }
        } else if (odm_slices == 4) {
            if (w.split[i] == 2 &&
                ((pipe.prev_odm != dcn32_no_pipe && t.pipes[pipe.prev_odm].prev_odm == dcn32_no_pipe) ||
                 pipe.next_odm == dcn32_no_pipe)) {
                w.merge[i] = true;
            } else if (w.split[i] == 0 && pipe.prev_odm != dcn32_no_pipe) {
                w.merge[i] = true;
            }
            w.split[i] = 0;
        }

        // A forced split never happens here (dcn20_fpu_adjust_dppclk is not needed).
        pipe_idx++;
    }

    return vlevel;
}

enum class SplitResult : uint8_t {
    unchanged,
    updated,
    // Merges, MPC combine, ODM without allow_odm or with two planes, or no
    // free pipe: this owner stops here.
    needs_more_pipes,
    invalid
};

// dcn32_apply_merge_split_flags_helper -> update_pipes_with_split_flags with
// the slice table (enable_windowed_mpo_odm): ODM slices are appended at the
// end through resource_update_pipes_for_stream_with_slice_count, then the
// scaler data of every pipe is rebuilt.
SplitResult apply_split_flags(Dcn32BandwidthWorkspace& w, const Dcn32PlaneRequest& request) noexcept {
    vba_vars_st& v = w.dml.vba;
    const uint32_t current_slices = topology_odm_slice_count(w.topology);
    uint32_t slices = current_slices;
    bool updated = false;

    for (uint32_t i = 0, dml_pipe_idx = 0; i < dcn32_topology_pipes; i++) {
        if (topology_is_free_pipe(w.topology, i)) {
            continue;
        }

        if (w.merge[i]) {
            return SplitResult::needs_more_pipes;
        }

        if (w.split[i] != 0) {
            const bool odm = v.ODMCombineEnabled[v.pipe_plane[dml_pipe_idx]] != dm_odm_combine_mode_disabled;

            if (odm && topology_is_opp_head(w.topology, i)) {
                slices += static_cast<uint32_t>(w.split[i] - 1);
            } else if (!odm && topology_is_dpp_pipe(w.topology, i)) {
                return SplitResult::needs_more_pipes;
            }

            updated = true;
        }

        dml_pipe_idx++;
    }

    if (slices != current_slices) {
        if (!request.allow_odm || w.topology.plane_count > 1 || slices > dcn32_topology_pipes ||
            !topology_add_odm_slices(w.topology, w.current, slices)) {
            return SplitResult::needs_more_pipes;
        }

        if (!topology_build_scaling(w.topology, request.timing.width, request.timing.height, w.planes,
                                    w.plane_count)) {
            return SplitResult::invalid;
        }
    }

    return updated ? SplitResult::updated : SplitResult::unchanged;
}

void clear_split_flags(Dcn32BandwidthWorkspace& w) noexcept {
    for (uint32_t i = 0; i < dcn32_topology_pipes; i++) {
        w.split[i] = 0;
        w.merge[i] = false;
    }
}

// dcn32_calculate_wm_and_dlg_fpu for the reference configuration (no SubVP, FPO
// impossible because the DMCUB reports no mclk_sw), followed by
// dcn32_calculate_dlg_params. vlevel is the validated voltage level.
void calculate_wm_and_dlg(Dcn32BandwidthWorkspace& w, unsigned int vlevel, Dcn32BandwidthPlan& plan) noexcept {
    display_mode_lib& dml = w.dml;
    vba_vars_st& v = dml.vba;
    display_e2e_pipe_params_st* pipes = w.pipes;
    const int pipe_cnt = static_cast<int>(w.pipe_cnt);
    const double dcfclk_from_validation = v.DCFCLKState[vlevel][v.maxMpcComb];
    const double dram_speed_from_validation = v.DRAMSpeed;
    const bool pstate_en = v.DRAMClockChangeSupport[vlevel][v.maxMpcComb] != dm_dram_clock_change_unsupported;
    unsigned int dummy_latency_index = 0;
    const int max_mpc_comb = v.maxMpcComb;
    unsigned int min_dram_speed_mts = static_cast<unsigned int>(v.DRAMSpeed);
    unsigned int vlevel_temp = 0;
    double dcfclk = 0;

    // Set B: clock_limits[2] when the box has more than two states, else 615 MHz.
    if (w.soc.num_states > 2) {
        vlevel_temp = 2;
        dcfclk = w.soc.clock_limits[2].dcfclk_mhz;
    } else {
        dcfclk = 615;
    }

    pipes[0].clks_cfg.voltage = static_cast<int>(vlevel_temp);
    pipes[0].clks_cfg.dcfclk_mhz = dcfclk;
    pipes[0].clks_cfg.socclk_mhz = dml.soc.clock_limits[vlevel_temp].socclk_mhz;

    if (w.wm[wm_b].valid) {
        dml.soc.dram_clock_change_latency_us = w.wm[wm_b].pstate_latency_us;
        dml.soc.fclk_change_latency_us = w.wm[wm_b].fclk_change_latency_us;
        dml.soc.sr_enter_plus_exit_time_us = w.wm[wm_b].sr_enter_plus_exit_time_us;
        dml.soc.sr_exit_time_us = w.wm[wm_b].sr_exit_time_us;
    }

    watermark_set(w, plan.b);

    // Sets A and C use the validation clocks.
    pipes[0].clks_cfg.voltage = static_cast<int>(vlevel);
    pipes[0].clks_cfg.dcfclk_mhz = dcfclk_from_validation;
    pipes[0].clks_cfg.socclk_mhz = dml.soc.clock_limits[vlevel].socclk_mhz;

    if (w.wm[wm_c].valid) {
        constexpr unsigned int min_dram_speed_mts_margin = 160;

        min_dram_speed_mts = static_cast<unsigned int>(dram_speed_from_validation);
        dml.soc.dram_clock_change_latency_us = w.dummy[0].dummy_pstate_latency_us;

        if (v.DRAMClockChangeSupport[vlevel][max_mpc_comb] == dm_dram_clock_change_unsupported) {
            // num_memclk_levels - 1 as an int index (at least 1 level is reported).
            const int offset = static_cast<int>(w.num_memclk_levels) - 1;

            min_dram_speed_mts = offset >= 0 && offset < static_cast<int>(dcn32_max_dpm_levels) ?
                                 w.clocks.levels[offset].memclk_mhz * 16 : 0;
        }

        for (dummy_latency_index = 3; dummy_latency_index > 0; dummy_latency_index--) {
            if (min_dram_speed_mts + min_dram_speed_mts_margin > w.dummy[dummy_latency_index].dram_speed_mts) {
                break;
            }
        }

        dml.soc.dram_clock_change_latency_us = w.dummy[dummy_latency_index].dummy_pstate_latency_us;
        dml.soc.fclk_change_latency_us = w.wm[wm_c].fclk_change_latency_us;
        dml.soc.sr_enter_plus_exit_time_us = w.wm[wm_c].sr_enter_plus_exit_time_us;
        dml.soc.sr_exit_time_us = w.wm[wm_c].sr_exit_time_us;
    }

    watermark_set(w, plan.c);
    // PMFW reports the UCLK dummy p-state as an FCLK p-state change on DCN32.
    plan.c.fclk_pstate_change_ns = to_u32(get_wm_dram_clock_change(&dml, pipes, pipe_cnt) * 1000);

    if (!pstate_en && w.wm[wm_c].valid) {
        plan.a = plan.c;
        plan.a.pstate_change_ns = 0;
        plan.a.fclk_pstate_change_ns = to_u32(get_fclk_watermark(&dml, pipes, pipe_cnt) * 1000);
    } else {
        dml.soc.dram_clock_change_latency_us = w.wm[wm_a].pstate_latency_us;
        dml.soc.sr_enter_plus_exit_time_us = w.wm[wm_a].sr_enter_plus_exit_time_us;
        dml.soc.sr_exit_time_us = w.wm[wm_a].sr_exit_time_us;
        watermark_set(w, plan.a);
    }

    plan.d = plan.a;

    // min_disp_clk_khz and min_dpp_clk_khz are zero and forced_clocks is false on dcn32.
    for (int pipe_idx = 0; pipe_idx < pipe_cnt; pipe_idx++) {
        pipes[pipe_idx].clks_cfg.dispclk_mhz = get_dispclk_calculated(&dml, pipes, pipe_cnt);
        pipes[pipe_idx].clks_cfg.dppclk_mhz = get_dppclk_calculated(&dml, pipes, pipe_cnt, pipe_idx);
    }

    // dcn32_calculate_dlg_params.
    Dcn32Clocks& clk = plan.clocks;
    clk.dispclk_khz = to_u32(v.DISPCLK * 1000);
    clk.dcfclk_khz = to_u32(v.DCFCLK * 1000);
    clk.socclk_khz = to_u32(v.SOCCLK * 1000);
    clk.dramclk_khz = to_u32(v.DRAMSpeed * 1000 / 16);
    clk.dcfclk_deep_sleep_khz = to_u32(v.DCFCLKDeepSleep * 1000);
    clk.fclk_khz = to_u32(v.FabricClock * 1000);
    clk.p_state_change_support = v.DRAMClockChangeSupport[vlevel][v.maxMpcComb] != dm_dram_clock_change_unsupported;
    clk.dppclk_khz = 0;
    // 8b/10b DP and TMDS never need DTBCLK (is_dtbclk_required: 128b/132b only).
    clk.dtbclk_en = false;
    clk.ref_dtbclk_khz = to_u32(v.DTBCLKPerState[vlevel] * 1000);
    clk.fclk_p_state_change_support = v.FCLKChangeSupport[vlevel][v.maxMpcComb] != dm_fclock_change_unsupported;

    bool unbounded_req_enabled = get_unbounded_request_enabled(&dml, pipes, pipe_cnt) != 0;

    // "Unbounded requesting should not ever be used when more than 1 pipe is enabled."
    if (unbounded_req_enabled && pipe_cnt > 1) {
        unbounded_req_enabled = false;
    }

    plan.unbounded_req = unbounded_req_enabled;
    plan.mall_ss_size_bytes = 0;
    plan.compbuf_size_kb = dml.ip.config_return_buffer_size_in_kbytes;
    plan.pipe_count = w.pipe_cnt;
    plan.odm_slices = topology_odm_slice_count(w.topology);
    plan.topology = w.topology;

    const Dcn32Topology& t = w.topology;
    for (uint32_t i = 0, pipe_idx = 0; i < dcn32_topology_pipes; i++) {
        if (!t.pipes[i].stream) {
            continue;
        }

        const int idx = static_cast<int>(pipe_idx);
        Dcn32PipeBandwidth& out = plan.pipes[pipe_idx];

        out.pipe = i;
        out.vstartup_start = static_cast<uint32_t>(get_vstartup(&dml, pipes, pipe_cnt, idx));
        out.vupdate_offset = static_cast<uint32_t>(get_vupdate_offset(&dml, pipes, pipe_cnt, idx));
        out.vupdate_width = static_cast<uint32_t>(get_vupdate_width(&dml, pipes, pipe_cnt, idx));
        out.vready_offset = static_cast<uint32_t>(get_vready_offset(&dml, pipes, pipe_cnt, idx));
        pipes[pipe_idx].pipe.dest.vstartup_start = out.vstartup_start;
        pipes[pipe_idx].pipe.dest.vupdate_offset = out.vupdate_offset;
        pipes[pipe_idx].pipe.dest.vupdate_width = out.vupdate_width;
        pipes[pipe_idx].pipe.dest.vready_offset = out.vready_offset;
        out.pstate_keepout = pipes[pipe_idx].pipe.dest.pstate_keepout;
        out.det_buffer_size_kb = static_cast<uint32_t>(get_det_buffer_size_kbytes(&dml, pipes, pipe_cnt, idx));

        if (clk.dppclk_khz < pipes[pipe_idx].clks_cfg.dppclk_mhz * 1000) {
            clk.dppclk_khz = to_u32(pipes[pipe_idx].clks_cfg.dppclk_mhz * 1000);
        }

        out.dppclk_khz = t.pipes[i].has_plane ? to_u32(pipes[pipe_idx].clks_cfg.dppclk_mhz * 1000) : 0u;
        // get_surface_size_in_mall (display_mode_vba.c dml_get_pipe_attr_func).
        out.surface_size_in_mall = static_cast<uint32_t>(get_surface_size_in_mall(&dml, pipes, pipe_cnt, idx));

        // MALL allocation sizes count active top pipes per plane only.
        const uint8_t top = t.pipes[i].top;
        if (t.pipes[i].has_plane &&
            (top == dcn32_no_pipe || !t.pipes[top].has_plane || t.pipes[top].plane != t.pipes[i].plane) &&
            t.pipes[i].prev_odm == dcn32_no_pipe) {
            plan.mall_ss_size_bytes += out.surface_size_in_mall;
        }

        pipe_idx++;
    }

    clk.max_supported_dppclk_khz = to_u32(dml.soc.clock_limits[vlevel].dppclk_mhz * 1000);
    clk.max_supported_dispclk_khz = to_u32(dml.soc.clock_limits[vlevel].dispclk_mhz * 1000);

    for (uint32_t pipe_idx = 0; pipe_idx < w.pipe_cnt; pipe_idx++) {
        plan.compbuf_size_kb -= plan.pipes[pipe_idx].det_buffer_size_kb;
    }

    for (uint32_t pipe_idx = 0; pipe_idx < w.pipe_cnt; pipe_idx++) {
        Dcn32PipeBandwidth& out = plan.pipes[pipe_idx];

        dml32_rq_dlg_get_dlg_reg(&dml, &w.dlg, &w.ttu, pipes, pipe_cnt, static_cast<int>(pipe_idx));
        dml32_rq_dlg_get_rq_reg(&w.rq, &dml, pipes, pipe_cnt, static_cast<int>(pipe_idx));
        copy_dlg(w.dlg, out.dlg);
        copy_ttu(w.ttu, out.ttu);
        copy_rq(w.rq, out.rq);
    }

    // The single-pipe fields are the OTG master pipe's.
    for (uint32_t pipe_idx = 0; pipe_idx < w.pipe_cnt; pipe_idx++) {
        const Dcn32PipeBandwidth& head = plan.pipes[pipe_idx];

        if (head.pipe != t.otg_master) {
            continue;
        }

        plan.vstartup_start = head.vstartup_start;
        plan.vupdate_offset = head.vupdate_offset;
        plan.vupdate_width = head.vupdate_width;
        plan.vready_offset = head.vready_offset;
        plan.pstate_keepout = head.pstate_keepout;
        plan.det_buffer_size_kb = head.det_buffer_size_kb;
        plan.dpp_dppclk_khz = head.dppclk_khz;
        plan.dlg = head.dlg;
        plan.ttu = head.ttu;
        plan.rq = head.rq;
    }
}

} // namespace

uint32_t dcn32_bandwidth_workspace_bytes() noexcept {
    return static_cast<uint32_t>(sizeof(Dcn32BandwidthWorkspace));
}

Dcn32BandwidthWorkspace* construct_dcn32_bandwidth_workspace(void* storage, uint32_t bytes) noexcept {
    if (storage == nullptr || bytes < sizeof(Dcn32BandwidthWorkspace) ||
        (reinterpret_cast<uintptr_t>(storage) % alignof(Dcn32BandwidthWorkspace)) != 0) {
        return nullptr;
    }

    // Nonallocating placement construction (value-initialized, trivially destructible).
    return reactos::construct_device_state<Dcn32BandwidthWorkspace>(storage);
}

Dcn32BandwidthStatus Dcn32Bandwidth::configure(Dcn32BandwidthWorkspace& workspace,
                                               const Dcn32SocInputs& inputs) noexcept {
    workspace_ = nullptr;
    Dcn32BandwidthWorkspace& w = workspace;
    const Dcn32ClockTable& clocks = inputs.clocks;

    if (inputs.dchub_refclk_khz == 0 || clocks.num_dcfclk_levels > dcn32_max_dpm_levels ||
        clocks.num_dppclk_levels > dcn32_max_dpm_levels || clocks.num_memclk_levels > dcn32_max_dpm_levels) {
        return Dcn32BandwidthStatus::invalid_argument;
    }

    // dpm_present (dcn32_init_clocks) and the clk_table.entries[0].memclk_mhz gate of
    // update_bw_bounding_box: without them Linux keeps the one-state default box,
    // which describes no real board, so the owner refuses to commit instead.
    if (clocks.num_dcfclk_levels == 0 || clocks.num_dtbclk_levels == 0 || clocks.num_dispclk_levels == 0 ||
        clocks.levels[0].memclk_mhz == 0) {
        return Dcn32BandwidthStatus::clock_table_unavailable;
    }

    dcn3_2_defaults(w.soc, w.ip);
    w.dchub_refclk_khz = inputs.dchub_refclk_khz;

    // dc_create's update_bw_bounding_box (VBIOS bb info, vram_info, debug options).
    if (inputs.dram_clock_change_latency_100ns > 0) {
        w.soc.dram_clock_change_latency_us = inputs.dram_clock_change_latency_100ns * 10;
    }

    if (inputs.sr_enter_exit_latency_100ns > 0) {
        w.soc.sr_enter_plus_exit_time_us = inputs.sr_enter_exit_latency_100ns * 10;
    }

    if (inputs.sr_exit_latency_100ns > 0) {
        w.soc.sr_exit_time_us = inputs.sr_exit_latency_100ns * 10;
    }

    if (inputs.num_chans != 0) {
        w.soc.num_chans = inputs.num_chans;
        // dcn32_calc_num_avail_chans_for_mall (GC 11.0.0) * mall_size_per_mem_channel (4).
        w.soc.mall_allocated_for_dcn_mbytes = static_cast<double>((inputs.num_chans == 48 ? 32u : 16u) * 4u);
    }

    if (inputs.dram_channel_width_bytes != 0) {
        w.soc.dram_channel_width_bytes = inputs.dram_channel_width_bytes;
    }

    w.ip.dsc_delay_factor_wa = 0 / 1000.0;
    w.ip.min_prefetch_in_strobe_us = 60000 / 1000.0;
    w.soc.dispclk_dppclk_vco_speed_mhz = (inputs.dentist_vco_khz != 0 ? inputs.dentist_vco_khz : 4300000u) / 1000.0;

    // dcn32_init_clocks: the DC-mode caps, and the 1950 MHz caps of the DISPCLK and
    // DPPCLK entries (both loops run over num_dppclk_levels, as upstream reuses
    // num_levels after reading the DPPCLK levels).
    w.clocks = clocks;

    if (w.clocks.dc_mode_limit.dispclk_mhz > dcn32_dispclk_cap_mhz) {
        w.clocks.dc_mode_limit.dispclk_mhz = dcn32_dispclk_cap_mhz;
    }

    if (w.clocks.dc_mode_limit.dppclk_mhz > dcn32_dispclk_cap_mhz) {
        w.clocks.dc_mode_limit.dppclk_mhz = dcn32_dispclk_cap_mhz;
    }

    for (uint32_t i = 0; i < w.clocks.num_dppclk_levels; i++) {
        if (w.clocks.levels[i].dispclk_mhz > dcn32_dispclk_cap_mhz) {
            w.clocks.levels[i].dispclk_mhz = dcn32_dispclk_cap_mhz;
        }
    }

    for (uint32_t i = 0; i < w.clocks.num_dppclk_levels; i++) {
        if (w.clocks.levels[i].dppclk_mhz > dcn32_dispclk_cap_mhz) {
            w.clocks.levels[i].dppclk_mhz = dcn32_dispclk_cap_mhz;
        }
    }

    // At most 5 STA + 1 + memclk levels + fclk levels (or 1) + 1 entries are inserted.
    uint32_t inserts = 5 + 1 + 1;

    for (uint32_t i = 0; i < dcn32_max_dpm_levels; i++) {
        inserts += (w.clocks.levels[i].memclk_mhz > 0 ? 1u : 0u) + (w.clocks.levels[i].fclk_mhz > 0 ? 1u : 0u);
    }

    if (inserts + 1 > synthetic_table_capacity) {
        return Dcn32BandwidthStatus::too_many_states;
    }

    if (!build_synthetic_soc_states(w)) {
        return Dcn32BandwidthStatus::clock_table_unavailable;
    }

    if (w.soc.num_states == 0 || w.soc.num_states > dcn32_max_soc_states) {
        return Dcn32BandwidthStatus::too_many_states;
    }

    build_wm_range_table(w);
    workspace_ = &w;
    return Dcn32BandwidthStatus::ready;
}

Dcn32BandwidthStatus Dcn32Bandwidth::validate(const Dcn32PlaneRequest& request,
                                              Dcn32BandwidthPlan& output) noexcept {
    if (workspace_ == nullptr) {
        return Dcn32BandwidthStatus::clock_table_unavailable;
    }

    if (!valid_request(request)) {
        return Dcn32BandwidthStatus::invalid_argument;
    }

    if (!request.allow_odm && request.timing.pixel_clock_khz > dcn3_2_vmin_dispclk_khz) {
        return Dcn32BandwidthStatus::needs_more_pipes;
    }

    Dcn32BandwidthWorkspace& w = *workspace_;
    display_mode_lib& dml = w.dml;
    vba_vars_st& v = dml.vba;

    // The planes of the context (one full-screen plane by default).
    if (request.plane_count == 0) {
        w.plane_count = 1;
        w.planes[0] = Dcn32PlaneGeometry{};
        w.planes[0].surface_width = request.timing.width;
        w.planes[0].surface_height = request.timing.height;
        w.planes[0].dst = Dcn32Rect{0, 0, static_cast<int32_t>(request.timing.width),
                                    static_cast<int32_t>(request.timing.height)};
        w.plane_pitch[0] = request.pitch_pixels;
    } else {
        w.plane_count = request.plane_count;
        for (uint32_t i = 0; i < request.plane_count; i++) {
            w.planes[i] = request.planes[i];
            w.plane_pitch[i] = request.plane_pitch_pixels[i];
        }
    }

    // The new dc_state: the stream's OTG master with no plane and one ODM slice
    // (dc_state_add_plane resets the slice count for a stream without planes),
    // then dc_state_add_plane in amdgpu_dm's descending zpos order.
    if (request.current != nullptr) {
        w.current = *request.current;
    } else {
        w.current = Dcn32Topology{};
    }
    topology_start_stream(w.topology, request.otg_instance, request.dsc_slices_h != 0);
    for (uint32_t i = 0; i < w.plane_count; i++) {
        if (!topology_append_plane(w.topology, w.current, static_cast<uint8_t>(i))) {
            return Dcn32BandwidthStatus::needs_more_pipes;
        }
    }
    if (!topology_build_scaling(w.topology, request.timing.width, request.timing.height, w.planes, w.plane_count)) {
        return Dcn32BandwidthStatus::invalid_argument;
    }

    // A fresh model per validation (the first validation after dc_create).
    dml_zero(dml.vba);
    dml.soc = w.soc;
    dml.ip = w.ip;
    dml.validate_max_state = false;
    clear_split_flags(w);

    // dcn30_fpu_update_soc_for_wm_a (no firmware-based MCLK switching).
    if (w.wm[wm_a].valid) {
        dml.soc.dram_clock_change_latency_us = w.wm[wm_a].pstate_latency_us;
        dml.soc.sr_enter_plus_exit_time_us = w.wm[wm_a].sr_enter_plus_exit_time_us;
        dml.soc.sr_exit_time_us = w.wm[wm_a].sr_exit_time_us;
    }

    w.pipe_cnt = populate_pipes(w, request);
    dml.soc.dram_clock_change_requirement_final = true;
    // dcn32_determine_max_vratio_prefetch: no YUV420 plane.
    dml.soc.max_vratio_pre = __DML_MAX_BW_RATIO_PRE__;

    // dcn32_full_validate_bw_helper (dml_disallow_alternate_prefetch_modes = false).
    dml.soc.allow_for_pstate_or_stutter_in_vblank_final = dm_prefetch_support_uclk_fclk_and_stutter_if_possible;
    unsigned int vlevel = dml_get_voltage_level(&dml, w.pipes, static_cast<int>(w.pipe_cnt));

    // This may adjust vlevel and maxMpcComb.
    if (vlevel < dml.soc.num_states) {
        vlevel = apply_pipe_split_flags(w, vlevel);
        v.VoltageLevel = vlevel;
    }

    bool repopulate = false;
    SplitResult split = apply_split_flags(w, request);

    if (split == SplitResult::needs_more_pipes) {
        return Dcn32BandwidthStatus::needs_more_pipes;
    }
    if (split == SplitResult::invalid) {
        return Dcn32BandwidthStatus::invalid_argument;
    }
    repopulate = split == SplitResult::updated;
    clear_split_flags(w);

    // should_apply_odm_power_optimization (single stream, windowed MPO ODM: the
    // state is not split) then try_odm_power_optimization_and_revalidate.
    // DEVIATION: not for a stream with two planes (Linux combines MPO and ODM).
    if (request.allow_odm && w.plane_count == 1 && request.signal == Dcn32OutputSignal::display_port &&
        h_timing_divisible_by_2(request.timing) &&
        request.timing.pixel_clock_khz * 10u * 100u > dcn3_2_vmin_dispclk_khz * 1000u &&
        topology_odm_slice_count(w.topology) == 1) {
        int cur_policy[DML32_NUM_PLANES] = {0, 0, 0, 0};

        for (uint32_t i = 0; i < w.pipe_cnt; i++) {
            cur_policy[i] = w.pipes[i].pipe.dest.odm_combine_policy;
            w.pipes[i].pipe.dest.odm_combine_policy = dm_odm_combine_policy_2to1;
        }

        const unsigned int new_vlevel = dml_get_voltage_level(&dml, w.pipes, static_cast<int>(w.pipe_cnt));

        if (new_vlevel < dml.soc.num_states) {
            clear_split_flags(w);
            vlevel = apply_pipe_split_flags(w, new_vlevel);
            v.VoltageLevel = vlevel;
        } else {
            for (uint32_t i = 0; i < w.pipe_cnt; i++) {
                w.pipes[i].pipe.dest.odm_combine_policy = static_cast<enum odm_combine_policy>(cur_policy[i]);
            }
        }
    }

    if (vlevel >= dml.soc.num_states) {
        return Dcn32BandwidthStatus::unsupported_mode;
    }

    split = apply_split_flags(w, request);
    if (split == SplitResult::needs_more_pipes) {
        return Dcn32BandwidthStatus::needs_more_pipes;
    }
    if (split == SplitResult::invalid) {
        return Dcn32BandwidthStatus::invalid_argument;
    }
    repopulate = repopulate || split == SplitResult::updated;

    if (repopulate) {
        // The pipes were split: recalculate the DET allocation and run DML once
        // more; a mode found with maxMpcComb 0 keeps a level supporting it.
        const int flag_max_mpc_comb = v.maxMpcComb;
        const unsigned int flag_vlevel = vlevel;

        w.pipe_cnt = populate_pipes(w, request);
        dml.soc.dram_clock_change_requirement_final = true;
        dml.soc.allow_for_pstate_or_stutter_in_vblank_final = dm_prefetch_support_uclk_fclk_and_stutter_if_possible;
        vlevel = dml_get_voltage_level(&dml, w.pipes, static_cast<int>(w.pipe_cnt));

        if (vlevel == dml.soc.num_states) {
            return Dcn32BandwidthStatus::unsupported_mode;
        }

        if (flag_max_mpc_comb == 0 && flag_max_mpc_comb != v.maxMpcComb) {
            bool flags_valid = false;

            for (unsigned int i = flag_vlevel; i < dml.soc.num_states; i++) {
                if (v.ModeSupport[i][flag_max_mpc_comb]) {
                    v.maxMpcComb = flag_max_mpc_comb;
                    v.VoltageLevel = i;
                    vlevel = i;
                    flags_valid = true;
                    break;
                }
            }

            if (!flags_valid) {
                return Dcn32BandwidthStatus::unsupported_mode;
            }
        }
    }

    // A level that only works with DSC is refused unless DSC was requested.
    if (vlevel >= dml.soc.num_states ||
        (request.dsc_slices_h == 0 && v.RequiresDSC[vlevel][v.pipe_plane[0]])) {
        return Dcn32BandwidthStatus::unsupported_mode;
    }

    dml_zero(w.plan);
    w.plan.voltage_level = vlevel;
    calculate_wm_and_dlg(w, vlevel, w.plan);
    output = w.plan;
    return Dcn32BandwidthStatus::ready;
}

} // namespace rx7900::display

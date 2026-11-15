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
Module: rx7900.display.dcn32_dml
Purpose: DML32 model types, enumerations, variables and arithmetic helpers.
Created: 2026-10-24 (adaptation; the upstream notice above is preserved)
Architecture: Part of the bounded C++17 port of the Linux v6.12 DML32 model behind
src/display/dcn32_bandwidth (one stream, up to four pipes). Identifiers, statement order and
arithmetic (including the float-typed dml_min/max/floor/ceil helpers and every
int/unsigned conversion) deliberately follow the pinned upstream text so each
function can be compared line by line with work/upstream-dcn-stream; this is a
documented exception to the snake_case rule, as in src/opengl/gfx11_registers.hpp.
Per-plane and per-pipe arrays hold the four DCN 3.2 pipes (DML32_NUM_PLANES; Linux
sizes them DC__NUM_DPP__MAX = 8) and per-state arrays DML32_NUM_STATES entries; debug printing is removed.
Deliberate differences from the upstream text, none of which changes a result in the
configurations this owner validates: writeback branches (dout.wb_enable is 0) and
dml32_UseMinimumDCFCLK (ip.use_min_dcfclk is 0) are removed together with unused
getters and helpers; the large passes are split into DML32_NOINLINE sections so no
i386 kernel frame exceeds 512 bytes; C casts are static_cast; int arrays passed as
unsigned pointers use dml_uint; dml_log2 reads the exponent arithmetically; the
memcmp/memcpy cache of recalculate_params uses dml_same/dml_copy; Linux's memset of
sizeof(DmlPipe) bytes over the ModeSupport dummy block is reproduced as the
entries it actually clears; four always-written local arrays are value-initialized.
Arithmetic follows the compiler's floating-point model for the target exactly as the
upstream C does: the x86-64 build matches Linux's x86-64 results, and the i386 x87
build matches the same Linux code compiled for i386 x87.
Upstream: dml/display_mode_enums.h, display_mode_structs.h, display_mode_vba.h, display_mode_vba_32.h, dml_inline_defs.h and dcn_calc_math.c.
*/
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace rx7900::display::dml32 {

// Keeps the split DML32 passes out of their callers so each i386 kernel frame
// stays small (the convention of src/opengl/gfx11_state.cpp).
#if defined(_MSC_VER)
#define DML32_NOINLINE __declspec(noinline)
#else
#define DML32_NOINLINE __attribute__((noinline))
#endif
constexpr unsigned int DML32_NUM_PLANES = 4;
constexpr unsigned int DML32_NUM_STATES = 25;
#define DC__NUM_DPP 4
#define DC__NUM_CURSOR__MAX 2
enum output_encoder_class {
    dm_dp = 0,
    dm_hdmi = 1,
    dm_wb = 2,
    dm_edp = 3,
    dm_dp2p0 = 5,
};
enum output_format_class {
    dm_444 = 0, dm_420 = 1, dm_n422, dm_s422
};
enum source_format_class {
    dm_444_16 = 0,
    dm_444_32 = 1,
    dm_444_64 = 2,
    dm_420_8 = 3,
    dm_420_10 = 4,
    dm_420_12 = 5,
    dm_422_8 = 6,
    dm_422_10 = 7,
    dm_444_8 = 8,
    dm_mono_8 = dm_444_8,
    dm_mono_16 = dm_444_16,
    dm_rgbe = 9,
    dm_rgbe_alpha = 10,
};
enum output_bpc_class {
    dm_out_6 = 0, dm_out_8 = 1, dm_out_10 = 2, dm_out_12 = 3, dm_out_16 = 4
};
enum scan_direction_class {
    dm_horz = 0, dm_vert = 1
};
enum dm_swizzle_mode {
    dm_sw_linear = 0,
    dm_sw_256b_s = 1,
    dm_sw_256b_d = 2,
    dm_sw_SPARE_0 = 3,
    dm_sw_SPARE_1 = 4,
    dm_sw_4kb_s = 5,
    dm_sw_4kb_d = 6,
    dm_sw_SPARE_2 = 7,
    dm_sw_SPARE_3 = 8,
    dm_sw_64kb_s = 9,
    dm_sw_64kb_d = 10,
    dm_sw_SPARE_4 = 11,
    dm_sw_SPARE_5 = 12,
    dm_sw_var_s = 13,
    dm_sw_var_d = 14,
    dm_sw_SPARE_6 = 15,
    dm_sw_SPARE_7 = 16,
    dm_sw_64kb_s_t = 17,
    dm_sw_64kb_d_t = 18,
    dm_sw_SPARE_10 = 19,
    dm_sw_SPARE_11 = 20,
    dm_sw_4kb_s_x = 21,
    dm_sw_4kb_d_x = 22,
    dm_sw_SPARE_12 = 23,
    dm_sw_SPARE_13 = 24,
    dm_sw_64kb_s_x = 25,
    dm_sw_64kb_d_x = 26,
    dm_sw_64kb_r_x = 27,
    dm_sw_SPARE_15 = 28,
    dm_sw_var_s_x = 29,
    dm_sw_var_d_x = 30,
    dm_sw_var_r_x = 31,
    dm_sw_gfx7_2d_thin_l_vp,
    dm_sw_gfx7_2d_thin_gl,
};
enum lb_depth {
    dm_lb_10 = 0, dm_lb_8 = 1, dm_lb_6 = 2, dm_lb_12 = 3, dm_lb_16 = 4,
    dm_lb_19 = 5
};
enum voltage_state {
    dm_vmin = 0, dm_vmid = 1, dm_vnom = 2, dm_vmax = 3
};
enum source_macro_tile_size {
    dm_4k_tile = 0, dm_64k_tile = 1, dm_256k_tile = 2
};
enum cursor_bpp {
    dm_cur_2bit = 0, dm_cur_32bit = 1, dm_cur_64bit = 2
};

/**
 * @enum clock_change_support - It represents possible reasons to change the DRAM clock.
 *
 * DC may change the DRAM clock during its execution, and this enum tracks all
 * the available methods. Note that every ASIC has their specific way to deal
 * with these clock switch.
 */
enum clock_change_support {
    /**
     * @dm_dram_clock_change_uninitialized: If you see this, we might have
     * a code initialization issue
     */
    dm_dram_clock_change_uninitialized = 0,

    /**
     * @dm_dram_clock_change_vactive: Support DRAM switch in VActive
     */
    dm_dram_clock_change_vactive,

    /**
     * @dm_dram_clock_change_vblank: Support DRAM switch in VBlank
     */
    dm_dram_clock_change_vblank,

    dm_dram_clock_change_vactive_w_mall_full_frame,
    dm_dram_clock_change_vactive_w_mall_sub_vp,
    dm_dram_clock_change_vblank_w_mall_full_frame,
    dm_dram_clock_change_vblank_w_mall_sub_vp,

    /**
     * @dm_dram_clock_change_unsupported: Do not support DRAM switch
     */
    dm_dram_clock_change_unsupported
};

enum output_standard {
    dm_std_uninitialized = 0,
    dm_std_cvtr2,
    dm_std_cvt
};

enum mpc_combine_affinity {
    dm_mpc_always_when_possible,
    dm_mpc_reduce_voltage,
    dm_mpc_reduce_voltage_and_clocks,
    dm_mpc_never
};

enum RequestType {
    REQ_256Bytes, REQ_128BytesNonContiguous, REQ_128BytesContiguous, REQ_NA
};

enum self_refresh_affinity {
    dm_try_to_allow_self_refresh_and_mclk_switch,
    dm_allow_self_refresh_and_mclk_switch,
    dm_allow_self_refresh,
    dm_neither_self_refresh_nor_mclk_switch
};

enum dm_validation_status {
    DML_VALIDATION_OK,
    DML_FAIL_SCALE_RATIO_TAP,
    DML_FAIL_SOURCE_PIXEL_FORMAT,
    DML_FAIL_VIEWPORT_SIZE,
    DML_FAIL_TOTAL_V_ACTIVE_BW,
    DML_FAIL_DIO_SUPPORT,
    DML_FAIL_NOT_ENOUGH_DSC,
    DML_FAIL_DSC_CLK_REQUIRED,
    DML_FAIL_DSC_VALIDATION_FAILURE,
    DML_FAIL_URGENT_LATENCY,
    DML_FAIL_REORDERING_BUFFER,
    DML_FAIL_DISPCLK_DPPCLK,
    DML_FAIL_TOTAL_AVAILABLE_PIPES,
    DML_FAIL_NUM_OTG,
    DML_FAIL_WRITEBACK_MODE,
    DML_FAIL_WRITEBACK_LATENCY,
    DML_FAIL_WRITEBACK_SCALE_RATIO_TAP,
    DML_FAIL_CURSOR_SUPPORT,
    DML_FAIL_PITCH_SUPPORT,
    DML_FAIL_PTE_BUFFER_SIZE,
    DML_FAIL_HOST_VM_IMMEDIATE_FLIP,
    DML_FAIL_DSC_INPUT_BPC,
    DML_FAIL_PREFETCH_SUPPORT,
    DML_FAIL_V_RATIO_PREFETCH,
    DML_FAIL_P2I_WITH_420,
    DML_FAIL_DSC_ONLY_IF_NECESSARY_WITH_BPP,
    DML_FAIL_NOT_DSC422_NATIVE,
    DML_FAIL_ODM_COMBINE4TO1,
    DML_FAIL_ENOUGH_WRITEBACK_UNITS,
    DML_FAIL_VIEWPORT_EXCEEDS_SURFACE,
    DML_FAIL_DYNAMIC_METADATA,
    DML_FAIL_FMT_BUFFER_EXCEEDED,
};

enum writeback_config {
    dm_normal,
    dm_whole_buffer_for_single_stream_no_interleave,
    dm_whole_buffer_for_single_stream_interleave,
};

enum odm_combine_mode {
    dm_odm_combine_mode_disabled,
    dm_odm_combine_mode_2to1,
    dm_odm_combine_mode_4to1,
    dm_odm_split_mode_1to2,
    dm_odm_mode_mso_1to2,
    dm_odm_mode_mso_1to4
};

enum odm_combine_policy {
    dm_odm_combine_policy_dal,
    dm_odm_combine_policy_none,
    dm_odm_combine_policy_2to1,
    dm_odm_combine_policy_4to1,
    dm_odm_split_policy_1to2,
    dm_odm_mso_policy_1to2,
    dm_odm_mso_policy_1to4,
};

enum immediate_flip_requirement {
    dm_immediate_flip_not_required,
    dm_immediate_flip_required,
    dm_immediate_flip_opportunistic,
};

enum unbounded_requesting_policy {
    dm_unbounded_requesting,
    dm_unbounded_requesting_edp_only,
    dm_unbounded_requesting_disable
};

enum dm_rotation_angle {
    dm_rotation_0,
    dm_rotation_90,
    dm_rotation_180,
    dm_rotation_270,
    dm_rotation_0m,
    dm_rotation_90m,
    dm_rotation_180m,
    dm_rotation_270m,
};

enum dm_use_mall_for_pstate_change_mode {
    dm_use_mall_pstate_change_disable,
    dm_use_mall_pstate_change_full_frame,
    dm_use_mall_pstate_change_sub_viewport,
    dm_use_mall_pstate_change_phantom_pipe
};

enum dm_use_mall_for_static_screen_mode {
    dm_use_mall_static_screen_disable,
    dm_use_mall_static_screen_optimize,
    dm_use_mall_static_screen_enable,
};

enum dm_output_link_dp_rate {
    dm_dp_rate_na,
    dm_dp_rate_hbr,
    dm_dp_rate_hbr2,
    dm_dp_rate_hbr3,
    dm_dp_rate_uhbr10,
    dm_dp_rate_uhbr13p5,
    dm_dp_rate_uhbr20,
};

enum dm_fclock_change_support {
    dm_fclock_change_vactive,
    dm_fclock_change_vblank,
    dm_fclock_change_unsupported,
};

enum dm_prefetch_modes {
    dm_prefetch_support_uclk_fclk_and_stutter_if_possible,
    dm_prefetch_support_uclk_fclk_and_stutter,
    dm_prefetch_support_fclk_and_stutter,
    dm_prefetch_support_stutter,
    dm_prefetch_support_none,
};
enum dm_output_type {
    dm_output_type_unknown,
    dm_output_type_dp,
    dm_output_type_edp,
    dm_output_type_dp2p0,
    dm_output_type_hdmi,
    dm_output_type_hdmifrl,
};

enum dm_output_rate {
    dm_output_rate_unknown,
    dm_output_rate_dp_rate_hbr,
    dm_output_rate_dp_rate_hbr2,
    dm_output_rate_dp_rate_hbr3,
    dm_output_rate_dp_rate_uhbr10,
    dm_output_rate_dp_rate_uhbr13p5,
    dm_output_rate_dp_rate_uhbr20,
    dm_output_rate_hdmi_rate_3x3,
    dm_output_rate_hdmi_rate_6x3,
    dm_output_rate_hdmi_rate_6x4,
    dm_output_rate_hdmi_rate_8x4,
    dm_output_rate_hdmi_rate_10x4,
    dm_output_rate_hdmi_rate_12x4,
};

/**
 * DOC: overview
 *
 * Most of the DML code is automatically generated and tested via hardware
 * description language. Usually, we use the reference _vcs_dpi in the code
 * where VCS means "Verilog Compiled Simulator" and DPI stands for "Direct
 * Programmer Interface". In other words, those structs can be used to
 * interface with Verilog with other languages such as C.
 */

typedef struct _vcs_dpi_voltage_scaling_st voltage_scaling_st;
typedef struct _vcs_dpi_soc_bounding_box_st soc_bounding_box_st;
typedef struct _vcs_dpi_ip_params_st ip_params_st;
typedef struct _vcs_dpi_display_pipe_source_params_st display_pipe_source_params_st;
typedef struct _vcs_dpi_display_output_params_st display_output_params_st;
typedef struct _vcs_dpi_scaler_ratio_depth_st scaler_ratio_depth_st;
typedef struct _vcs_dpi_scaler_taps_st scaler_taps_st;
typedef struct _vcs_dpi_display_pipe_dest_params_st display_pipe_dest_params_st;
typedef struct _vcs_dpi_display_pipe_params_st display_pipe_params_st;
typedef struct _vcs_dpi_display_clocks_and_cfg_st display_clocks_and_cfg_st;
typedef struct _vcs_dpi_display_e2e_pipe_params_st display_e2e_pipe_params_st;
typedef struct _vcs_dpi_display_data_rq_misc_params_st display_data_rq_misc_params_st;
typedef struct _vcs_dpi_display_data_rq_sizing_params_st display_data_rq_sizing_params_st;
typedef struct _vcs_dpi_display_data_rq_dlg_params_st display_data_rq_dlg_params_st;
typedef struct _vcs_dpi_display_rq_dlg_params_st display_rq_dlg_params_st;
typedef struct _vcs_dpi_display_rq_sizing_params_st display_rq_sizing_params_st;
typedef struct _vcs_dpi_display_rq_misc_params_st display_rq_misc_params_st;
typedef struct _vcs_dpi_display_rq_params_st display_rq_params_st;
typedef struct _vcs_dpi_display_dlg_regs_st display_dlg_regs_st;
typedef struct _vcs_dpi_display_ttu_regs_st display_ttu_regs_st;
typedef struct _vcs_dpi_display_data_rq_regs_st display_data_rq_regs_st;
typedef struct _vcs_dpi_display_rq_regs_st display_rq_regs_st;
typedef struct _vcs_dpi_display_dlg_sys_params_st display_dlg_sys_params_st;
typedef struct _vcs_dpi_display_arb_params_st display_arb_params_st;

typedef struct {
    double UrgentWatermark;
    double WritebackUrgentWatermark;
    double DRAMClockChangeWatermark;
    double FCLKChangeWatermark;
    double WritebackDRAMClockChangeWatermark;
    double WritebackFCLKChangeWatermark;
    double StutterExitWatermark;
    double StutterEnterPlusExitWatermark;
    double Z8StutterExitWatermark;
    double Z8StutterEnterPlusExitWatermark;
    double USRRetrainingWatermark;
} Watermarks;

typedef struct {
    double UrgentLatency;
    double ExtraLatency;
    double WritebackLatency;
    double DRAMClockChangeLatency;
    double FCLKChangeLatency;
    double SRExitTime;
    double SREnterPlusExitTime;
    double SRExitZ8Time;
    double SREnterPlusExitZ8Time;
    double USRRetrainingLatencyPlusSMNLatency;
} Latencies;

typedef struct {
    double Dppclk;
    double Dispclk;
    double PixelClock;
    double DCFClkDeepSleep;
    unsigned int DPPPerSurface;
    bool ScalerEnabled;
    enum dm_rotation_angle SourceRotation;
    unsigned int ViewportHeight;
    unsigned int ViewportHeightChroma;
    unsigned int BlockWidth256BytesY;
    unsigned int BlockHeight256BytesY;
    unsigned int BlockWidth256BytesC;
    unsigned int BlockHeight256BytesC;
    unsigned int BlockWidthY;
    unsigned int BlockHeightY;
    unsigned int BlockWidthC;
    unsigned int BlockHeightC;
    unsigned int InterlaceEnable;
    unsigned int NumberOfCursors;
    unsigned int VBlank;
    unsigned int HTotal;
    unsigned int HActive;
    bool DCCEnable;
    enum odm_combine_mode ODMMode;
    enum source_format_class SourcePixelFormat;
    enum dm_swizzle_mode SurfaceTiling;
    unsigned int BytePerPixelY;
    unsigned int BytePerPixelC;
    bool ProgressiveToInterlaceUnitInOPP;
    double VRatio;
    double VRatioChroma;
    unsigned int VTaps;
    unsigned int VTapsChroma;
    unsigned int PitchY;
    unsigned int DCCMetaPitchY;
    unsigned int PitchC;
    unsigned int DCCMetaPitchC;
    bool ViewportStationary;
    unsigned int ViewportXStart;
    unsigned int ViewportYStart;
    unsigned int ViewportXStartC;
    unsigned int ViewportYStartC;
    bool FORCE_ONE_ROW_FOR_FRAME;
    unsigned int SwathHeightY;
    unsigned int SwathHeightC;
} DmlPipe;

typedef struct {
    double UrgentLatency;
    double ExtraLatency;
    double WritebackLatency;
    double DRAMClockChangeLatency;
    double FCLKChangeLatency;
    double SRExitTime;
    double SREnterPlusExitTime;
    double SRExitZ8Time;
    double SREnterPlusExitZ8Time;
    double USRRetrainingLatency;
    double SMNLatency;
} SOCParametersList;

struct _vcs_dpi_voltage_scaling_st {
    int state;
    double dscclk_mhz;
    double dcfclk_mhz;
    double socclk_mhz;
    double phyclk_d18_mhz;
    double phyclk_d32_mhz;
    double dram_speed_mts;
    double fabricclk_mhz;
    double dispclk_mhz;
    double dram_bw_per_chan_gbps;
    double phyclk_mhz;
    double dppclk_mhz;
    double dtbclk_mhz;
    float net_bw_in_kbytes_sec;
};

/**
 * _vcs_dpi_soc_bounding_box_st: SOC definitions
 *
 * This struct maintains the SOC Bounding Box information for the ASIC; it
 * defines things such as clock, voltage, performance, etc. Usually, we load
 * these values from VBIOS; if something goes wrong, we use some hard-coded
 * values, which will enable the ASIC to light up with limitations.
 */
struct _vcs_dpi_soc_bounding_box_st {
    struct _vcs_dpi_voltage_scaling_st clock_limits[DML32_NUM_STATES];
    /**
     * @num_states: It represents the total of Display Power Management
     * (DPM) supported by the specific ASIC.
     */
    unsigned int num_states;
    double sr_exit_time_us;
    double sr_enter_plus_exit_time_us;
    double sr_exit_z8_time_us;
    double sr_enter_plus_exit_z8_time_us;
    double urgent_latency_us;
    double urgent_latency_pixel_data_only_us;
    double urgent_latency_pixel_mixed_with_vm_data_us;
    double urgent_latency_vm_data_only_us;
    double usr_retraining_latency_us;
    double smn_latency_us;
    double fclk_change_latency_us;
    double mall_allocated_for_dcn_mbytes;
    double pct_ideal_fabric_bw_after_urgent;
    double pct_ideal_dram_bw_after_urgent_strobe;
    double max_avg_fabric_bw_use_normal_percent;
    double max_avg_dram_bw_use_normal_strobe_percent;
    enum dm_prefetch_modes allow_for_pstate_or_stutter_in_vblank_final;
    bool dram_clock_change_requirement_final;
    double writeback_latency_us;
    double ideal_dram_bw_after_urgent_percent;
    double pct_ideal_dram_sdp_bw_after_urgent_pixel_only; // PercentOfIdealDRAMFabricAndSDPPortBWReceivedAfterUrgLatencyPixelDataOnly
    double pct_ideal_dram_sdp_bw_after_urgent_pixel_and_vm;
    double pct_ideal_dram_sdp_bw_after_urgent_vm_only;
    double pct_ideal_sdp_bw_after_urgent;
    double max_avg_sdp_bw_use_normal_percent;
    double max_avg_dram_bw_use_normal_percent;
    unsigned int max_request_size_bytes;
    double downspread_percent;
    double dram_page_open_time_ns;
    double dram_rw_turnaround_time_ns;
    double dram_return_buffer_per_channel_bytes;
    double dram_channel_width_bytes;
    double fabric_datapath_to_dcn_data_return_bytes;
    double dcn_downspread_percent;
    double dispclk_dppclk_vco_speed_mhz;
    double dfs_vco_period_ps;
    unsigned int urgent_out_of_order_return_per_channel_pixel_only_bytes;
    unsigned int urgent_out_of_order_return_per_channel_pixel_and_vm_bytes;
    unsigned int urgent_out_of_order_return_per_channel_vm_only_bytes;
    unsigned int round_trip_ping_latency_dcfclk_cycles;
    unsigned int urgent_out_of_order_return_per_channel_bytes;
    unsigned int channel_interleave_bytes;
    unsigned int num_banks;
    unsigned int num_chans;
    unsigned int vmm_page_size_bytes;
    unsigned int hostvm_min_page_size_bytes;
    unsigned int gpuvm_min_page_size_bytes;
    double dram_clock_change_latency_us;
    double dummy_pstate_latency_us;
    double writeback_dram_clock_change_latency_us;
    unsigned int return_bus_width_bytes;
    unsigned int voltage_override;
    double xfc_bus_transport_time_us;
    double xfc_xbuf_latency_tolerance_us;
    int use_urgent_burst_bw;
    double min_dcfclk;
    bool do_urgent_latency_adjustment;
    double urgent_latency_adjustment_fabric_clock_component_us;
    double urgent_latency_adjustment_fabric_clock_reference_mhz;
    bool disable_dram_clock_change_vactive_support;
    bool allow_dram_clock_one_display_vactive;
    enum self_refresh_affinity allow_dram_self_refresh_or_dram_clock_change_in_vblank;
    double max_vratio_pre;
};

/**
 * @_vcs_dpi_ip_params_st: IP configuraion for DCN blocks
 *
 * In this struct you can find the DCN configuration associated to the specific
 * ASIC. For example, here we can save how many DPPs the ASIC is using and it
 * is available.
 *
 */
struct _vcs_dpi_ip_params_st {
    bool use_min_dcfclk;
    bool clamp_min_dcfclk;
    bool gpuvm_enable;
    bool hostvm_enable;
    bool dsc422_native_support;
    unsigned int gpuvm_max_page_table_levels;
    unsigned int hostvm_max_page_table_levels;
    unsigned int hostvm_cached_page_table_levels;
    unsigned int pte_group_size_bytes;
    unsigned int max_inter_dcn_tile_repeaters;
    unsigned int num_dsc;
    unsigned int odm_capable;
    unsigned int rob_buffer_size_kbytes;
    unsigned int det_buffer_size_kbytes;
    unsigned int min_comp_buffer_size_kbytes;
    unsigned int dpte_buffer_size_in_pte_reqs_luma;
    unsigned int dpte_buffer_size_in_pte_reqs_chroma;
    unsigned int pde_proc_buffer_size_64k_reqs;
    unsigned int dpp_output_buffer_pixels;
    unsigned int opp_output_buffer_lines;
    unsigned int pixel_chunk_size_kbytes;
    unsigned int alpha_pixel_chunk_size_kbytes;
    unsigned int min_pixel_chunk_size_bytes;
    unsigned int dcc_meta_buffer_size_bytes;
    unsigned char pte_enable;
    unsigned int pte_chunk_size_kbytes;
    unsigned int meta_chunk_size_kbytes;
    unsigned int min_meta_chunk_size_bytes;
    unsigned int writeback_chunk_size_kbytes;
    unsigned int line_buffer_size_bits;
    unsigned int max_line_buffer_lines;
    unsigned int writeback_luma_buffer_size_kbytes;
    unsigned int writeback_chroma_buffer_size_kbytes;
    unsigned int writeback_chroma_line_buffer_width_pixels;

    unsigned int writeback_interface_buffer_size_kbytes;
    unsigned int writeback_line_buffer_buffer_size;

    unsigned int writeback_10bpc420_supported;
    double writeback_max_hscl_ratio;
    double writeback_max_vscl_ratio;
    double writeback_min_hscl_ratio;
    double writeback_min_vscl_ratio;
    unsigned int maximum_dsc_bits_per_component;
    unsigned int maximum_pixels_per_line_per_dsc_unit;
    unsigned int writeback_max_hscl_taps;
    unsigned int writeback_max_vscl_taps;
    unsigned int writeback_line_buffer_luma_buffer_size;
    unsigned int writeback_line_buffer_chroma_buffer_size;

    unsigned int max_page_table_levels;
    /**
     * @max_num_dpp: Maximum number of DPP supported in the target ASIC.
     */
    unsigned int max_num_dpp;
    unsigned int max_num_otg;
    unsigned int cursor_chunk_size;
    unsigned int cursor_buffer_size;
    unsigned int max_num_wb;
    unsigned int max_dchub_pscl_bw_pix_per_clk;
    unsigned int max_pscl_lb_bw_pix_per_clk;
    unsigned int max_lb_vscl_bw_pix_per_clk;
    unsigned int max_vscl_hscl_bw_pix_per_clk;
    double max_hscl_ratio;
    double max_vscl_ratio;
    unsigned int hscl_mults;
    unsigned int vscl_mults;
    unsigned int max_hscl_taps;
    unsigned int max_vscl_taps;
    unsigned int xfc_supported;
    unsigned int ptoi_supported;
    unsigned int gfx7_compat_tiling_supported;

    bool odm_combine_4to1_supported;
    bool dynamic_metadata_vm_enabled;
    unsigned int max_num_hdmi_frl_outputs;

    unsigned int xfc_fill_constant_bytes;
    double dispclk_ramp_margin_percent;
    double xfc_fill_bw_overhead_percent;
    double underscan_factor;
    unsigned int min_vblank_lines;
    unsigned int dppclk_delay_subtotal;
    unsigned int dispclk_delay_subtotal;
    double dcfclk_cstate_latency;
    unsigned int dppclk_delay_scl;
    unsigned int dppclk_delay_scl_lb_only;
    unsigned int dppclk_delay_cnvc_formatter;
    unsigned int dppclk_delay_cnvc_cursor;
    unsigned int is_line_buffer_bpp_fixed;
    unsigned int line_buffer_fixed_bpp;
    unsigned int dcc_supported;
    unsigned int config_return_buffer_size_in_kbytes;
    unsigned int compressed_buffer_segment_size_in_kbytes;
    unsigned int meta_fifo_size_in_kentries;
    unsigned int zero_size_buffer_entries;
    unsigned int compbuf_reserved_space_64b;
    unsigned int compbuf_reserved_space_zs;

    unsigned int IsLineBufferBppFixed;
    unsigned int LineBufferFixedBpp;
    unsigned int can_vstartup_lines_exceed_vsync_plus_back_porch_lines_minus_one;
    unsigned int bug_forcing_LC_req_same_size_fixed;
    unsigned int number_of_cursors;
    unsigned int max_num_dp2p0_outputs;
    unsigned int max_num_dp2p0_streams;
    unsigned int VBlankNomDefaultUS;

    /* DM workarounds */
    double dsc_delay_factor_wa; // TODO: Remove after implementing root cause fix
    double min_prefetch_in_strobe_us;
};

struct _vcs_dpi_display_xfc_params_st {
    double xfc_tslv_vready_offset_us;
    double xfc_tslv_vupdate_width_us;
    double xfc_tslv_vupdate_offset_us;
    int xfc_slv_chunk_size_bytes;
};

struct _vcs_dpi_display_pipe_source_params_st {
    int source_format;
    double dcc_fraction_of_zs_req_luma;
    double dcc_fraction_of_zs_req_chroma;
    unsigned char dcc;
    unsigned int dcc_rate;
    unsigned int dcc_rate_chroma;
    unsigned char dcc_use_global;
    unsigned char vm;
    bool unbounded_req_mode;
    bool gpuvm;    // gpuvm enabled
    bool hostvm;    // hostvm enabled
    bool gpuvm_levels_force_en;
    unsigned int gpuvm_levels_force;
    bool hostvm_levels_force_en;
    unsigned int hostvm_levels_force;
    int source_scan;
    int source_rotation; // new in dml32
    unsigned int det_size_override; // use to populate DETSizeOverride in vba struct
    int sw_mode;
    int macro_tile_size;
    unsigned int surface_width_y;
    unsigned int surface_height_y;
    unsigned int surface_width_c;
    unsigned int surface_height_c;
    unsigned int viewport_width;
    unsigned int viewport_height;
    unsigned int viewport_y_y;
    unsigned int viewport_y_c;
    unsigned int viewport_width_c;
    unsigned int viewport_height_c;
    unsigned int viewport_width_max;
    unsigned int viewport_height_max;
    unsigned int viewport_x_y;
    unsigned int viewport_x_c;
    bool viewport_stationary;
    unsigned int dcc_rate_luma;
    unsigned int gpuvm_min_page_size_kbytes;
    unsigned int use_mall_for_pstate_change;
    unsigned int use_mall_for_static_screen;
    bool force_one_row_for_frame;
    bool pte_buffer_mode;
    unsigned int data_pitch;
    unsigned int data_pitch_c;
    unsigned int meta_pitch;
    unsigned int meta_pitch_c;
    unsigned int cur0_src_width;
    int cur0_bpp;
    unsigned int cur1_src_width;
    int cur1_bpp;
    int num_cursors;
    unsigned char is_hsplit;
    unsigned char dynamic_metadata_enable;
    unsigned int dynamic_metadata_lines_before_active;
    unsigned int dynamic_metadata_xmit_bytes;
    unsigned int hsplit_grp;
    unsigned char xfc_enable;
    unsigned char xfc_slave;
    unsigned char immediate_flip;
    struct _vcs_dpi_display_xfc_params_st xfc_params;
    //for vstartuplines calculation freesync
    unsigned char v_total_min;
    unsigned char v_total_max;
};
struct writeback_st {
    int wb_src_height;
    int wb_src_width;
    int wb_dst_width;
    int wb_dst_height;
    int wb_pixel_format;
    int wb_htaps_luma;
    int wb_vtaps_luma;
    int wb_htaps_chroma;
    int wb_vtaps_chroma;
    unsigned int wb_htaps;
    unsigned int wb_vtaps;
    double wb_hratio;
    double wb_vratio;
};

struct display_audio_params_st {
    unsigned int   audio_sample_rate_khz;
    int            audio_sample_layout;
};

struct _vcs_dpi_display_output_params_st {
    int dp_lanes;
    double output_bpp;
    unsigned int dsc_input_bpc;
    int dsc_enable;
    int wb_enable;
    int num_active_wb;
    int output_type;
    int is_virtual;
    int output_format;
    int dsc_slices;
    int max_audio_sample_rate;
    struct writeback_st wb;
    struct display_audio_params_st audio;
    unsigned int output_bpc;
    int dp_rate;
    unsigned int dp_multistream_id;
    bool dp_multistream_en;
};

struct _vcs_dpi_scaler_ratio_depth_st {
    double hscl_ratio;
    double vscl_ratio;
    double hscl_ratio_c;
    double vscl_ratio_c;
    double vinit;
    double vinit_c;
    double vinit_bot;
    double vinit_bot_c;
    int lb_depth;
    int scl_enable;
};

struct _vcs_dpi_scaler_taps_st {
    unsigned int htaps;
    unsigned int vtaps;
    unsigned int htaps_c;
    unsigned int vtaps_c;
};

struct _vcs_dpi_display_pipe_dest_params_st {
    unsigned int recout_width;
    unsigned int recout_height;
    unsigned int full_recout_width;
    unsigned int full_recout_height;
    unsigned int hblank_start;
    unsigned int hblank_end;
    unsigned int vblank_start;
    unsigned int vblank_end;
    unsigned int htotal;
    unsigned int vtotal;
    unsigned int vfront_porch;
    unsigned int vblank_nom;
    unsigned int vactive;
    unsigned int hactive;
    unsigned int vstartup_start;
    unsigned int vupdate_offset;
    unsigned int vupdate_width;
    unsigned int vready_offset;
    unsigned int pstate_keepout;
    unsigned char interlaced;
    double pixel_rate_mhz;
    unsigned char synchronized_vblank_all_planes;
    unsigned char otg_inst;
    unsigned int odm_combine;
    unsigned char use_maximum_vstartup;
    unsigned int vtotal_max;
    unsigned int vtotal_min;
    unsigned int refresh_rate;
    bool synchronize_timings;
    unsigned int odm_combine_policy;
    bool drr_display;
};

struct _vcs_dpi_display_pipe_params_st {
    display_pipe_source_params_st src;
    display_pipe_dest_params_st dest;
    scaler_ratio_depth_st scale_ratio_depth;
    scaler_taps_st scale_taps;
};

struct _vcs_dpi_display_clocks_and_cfg_st {
    int voltage;
    double dppclk_mhz;
    double refclk_mhz;
    double dispclk_mhz;
    double dcfclk_mhz;
    double socclk_mhz;
};

struct _vcs_dpi_display_e2e_pipe_params_st {
    display_pipe_params_st pipe;
    display_output_params_st dout;
    display_clocks_and_cfg_st clks_cfg;
};

struct _vcs_dpi_display_data_rq_misc_params_st {
    unsigned int full_swath_bytes;
    unsigned int stored_swath_bytes;
    unsigned int blk256_height;
    unsigned int blk256_width;
    unsigned int req_height;
    unsigned int req_width;
};

struct _vcs_dpi_display_data_rq_sizing_params_st {
    unsigned int chunk_bytes;
    unsigned int min_chunk_bytes;
    unsigned int meta_chunk_bytes;
    unsigned int min_meta_chunk_bytes;
    unsigned int mpte_group_bytes;
    unsigned int dpte_group_bytes;
};

struct _vcs_dpi_display_data_rq_dlg_params_st {
    unsigned int swath_width_ub;
    unsigned int swath_height;
    unsigned int req_per_swath_ub;
    unsigned int meta_pte_bytes_per_frame_ub;
    unsigned int dpte_req_per_row_ub;
    unsigned int dpte_groups_per_row_ub;
    unsigned int dpte_row_height;
    unsigned int dpte_bytes_per_row_ub;
    unsigned int meta_chunks_per_row_ub;
    unsigned int meta_req_per_row_ub;
    unsigned int meta_row_height;
    unsigned int meta_bytes_per_row_ub;
};

struct _vcs_dpi_display_rq_dlg_params_st {
    display_data_rq_dlg_params_st rq_l;
    display_data_rq_dlg_params_st rq_c;
};

struct _vcs_dpi_display_rq_sizing_params_st {
    display_data_rq_sizing_params_st rq_l;
    display_data_rq_sizing_params_st rq_c;
};

struct _vcs_dpi_display_rq_misc_params_st {
    display_data_rq_misc_params_st rq_l;
    display_data_rq_misc_params_st rq_c;
};

struct _vcs_dpi_display_rq_params_st {
    unsigned char yuv420;
    unsigned char yuv420_10bpc;
    unsigned char rgbe_alpha;
    display_rq_misc_params_st misc;
    display_rq_sizing_params_st sizing;
    display_rq_dlg_params_st dlg;
};

struct _vcs_dpi_display_dlg_regs_st {
    unsigned int refcyc_h_blank_end;
    unsigned int dlg_vblank_end;
    unsigned int min_dst_y_next_start;
    unsigned int min_dst_y_next_start_us;
    unsigned int refcyc_per_htotal;
    unsigned int refcyc_x_after_scaler;
    unsigned int dst_y_after_scaler;
    unsigned int dst_y_prefetch;
    unsigned int dst_y_per_vm_vblank;
    unsigned int dst_y_per_row_vblank;
    unsigned int dst_y_per_vm_flip;
    unsigned int dst_y_per_row_flip;
    unsigned int ref_freq_to_pix_freq;
    unsigned int vratio_prefetch;
    unsigned int vratio_prefetch_c;
    unsigned int refcyc_per_tdlut_group;
    unsigned int refcyc_per_pte_group_vblank_l;
    unsigned int refcyc_per_pte_group_vblank_c;
    unsigned int refcyc_per_meta_chunk_vblank_l;
    unsigned int refcyc_per_meta_chunk_vblank_c;
    unsigned int refcyc_per_pte_group_flip_l;
    unsigned int refcyc_per_pte_group_flip_c;
    unsigned int refcyc_per_meta_chunk_flip_l;
    unsigned int refcyc_per_meta_chunk_flip_c;
    unsigned int dst_y_per_pte_row_nom_l;
    unsigned int dst_y_per_pte_row_nom_c;
    unsigned int refcyc_per_pte_group_nom_l;
    unsigned int refcyc_per_pte_group_nom_c;
    unsigned int dst_y_per_meta_row_nom_l;
    unsigned int dst_y_per_meta_row_nom_c;
    unsigned int refcyc_per_meta_chunk_nom_l;
    unsigned int refcyc_per_meta_chunk_nom_c;
    unsigned int refcyc_per_line_delivery_pre_l;
    unsigned int refcyc_per_line_delivery_pre_c;
    unsigned int refcyc_per_line_delivery_l;
    unsigned int refcyc_per_line_delivery_c;
    unsigned int chunk_hdl_adjust_cur0;
    unsigned int chunk_hdl_adjust_cur1;
    unsigned int vready_after_vcount0;
    unsigned int dst_y_offset_cur0;
    unsigned int dst_y_offset_cur1;
    unsigned int xfc_reg_transfer_delay;
    unsigned int xfc_reg_precharge_delay;
    unsigned int xfc_reg_remote_surface_flip_latency;
    unsigned int xfc_reg_prefetch_margin;
    unsigned int dst_y_delta_drq_limit;
    unsigned int refcyc_per_vm_group_vblank;
    unsigned int refcyc_per_vm_group_flip;
    unsigned int refcyc_per_vm_req_vblank;
    unsigned int refcyc_per_vm_req_flip;
    unsigned int refcyc_per_vm_dmdata;
    unsigned int dmdata_dl_delta;
};

struct _vcs_dpi_display_ttu_regs_st {
    unsigned int qos_level_low_wm;
    unsigned int qos_level_high_wm;
    unsigned int min_ttu_vblank;
    unsigned int qos_level_flip;
    unsigned int refcyc_per_req_delivery_l;
    unsigned int refcyc_per_req_delivery_c;
    unsigned int refcyc_per_req_delivery_cur0;
    unsigned int refcyc_per_req_delivery_cur1;
    unsigned int refcyc_per_req_delivery_pre_l;
    unsigned int refcyc_per_req_delivery_pre_c;
    unsigned int refcyc_per_req_delivery_pre_cur0;
    unsigned int refcyc_per_req_delivery_pre_cur1;
    unsigned int qos_level_fixed_l;
    unsigned int qos_level_fixed_c;
    unsigned int qos_level_fixed_cur0;
    unsigned int qos_level_fixed_cur1;
    unsigned int qos_ramp_disable_l;
    unsigned int qos_ramp_disable_c;
    unsigned int qos_ramp_disable_cur0;
    unsigned int qos_ramp_disable_cur1;
};

struct _vcs_dpi_display_data_rq_regs_st {
    unsigned int chunk_size;
    unsigned int min_chunk_size;
    unsigned int meta_chunk_size;
    unsigned int min_meta_chunk_size;
    unsigned int dpte_group_size;
    unsigned int mpte_group_size;
    unsigned int swath_height;
    unsigned int pte_row_height_linear;
};

struct _vcs_dpi_display_rq_regs_st {
    display_data_rq_regs_st rq_regs_l;
    display_data_rq_regs_st rq_regs_c;
    unsigned int drq_expansion_mode;
    unsigned int prq_expansion_mode;
    unsigned int mrq_expansion_mode;
    unsigned int crq_expansion_mode;
    unsigned int plane1_base_address;
    unsigned int aperture_low_addr;   // bits [47:18]
    unsigned int aperture_high_addr;  // bits [47:18]
};

struct _vcs_dpi_display_dlg_sys_params_st {
    double t_mclk_wm_us;
    double t_urg_wm_us;
    double t_sr_wm_us;
    double t_extra_us;
    double mem_trip_us;
    double deepsleep_dcfclk_mhz;
    double total_flip_bw;
    unsigned int total_flip_bytes;
};

struct _vcs_dpi_display_arb_params_st {
    int max_req_outstanding;
    int min_req_outstanding;
    int sat_level_us;
    int hvm_min_req_outstand_commit_threshold;
    int hvm_max_qos_commit_threshold;
    int compbuf_reserved_space_kbytes;
};

struct display_mode_lib;

void ModeSupportAndSystemConfiguration(struct display_mode_lib *mode_lib);

#define dml_get_attr_decl(attr) double get_##attr(struct display_mode_lib *mode_lib, const display_e2e_pipe_params_st *pipes, unsigned int num_pipes)

dml_get_attr_decl(wm_urgent);
dml_get_attr_decl(wm_memory_trip);
dml_get_attr_decl(wm_stutter_exit);
dml_get_attr_decl(wm_stutter_enter_exit);
dml_get_attr_decl(wm_dram_clock_change);
dml_get_attr_decl(urgent_latency);
dml_get_attr_decl(dispclk_calculated);
dml_get_attr_decl(fraction_of_urgent_bandwidth);
dml_get_attr_decl(fraction_of_urgent_bandwidth_imm_flip);
dml_get_attr_decl(pixel_chunk_size_in_kbyte);
dml_get_attr_decl(alpha_pixel_chunk_size_in_kbyte);
dml_get_attr_decl(meta_chunk_size_in_kbyte);
dml_get_attr_decl(min_pixel_chunk_size_in_byte);
dml_get_attr_decl(min_meta_chunk_size_in_byte);
dml_get_attr_decl(fclk_watermark);
dml_get_attr_decl(usr_retraining_watermark);
dml_get_attr_decl(unbounded_request_enabled);

#define dml_get_pipe_attr_decl(attr) double get_##attr(struct display_mode_lib *mode_lib, const display_e2e_pipe_params_st *pipes, unsigned int num_pipes, unsigned int which_pipe)

dml_get_pipe_attr_decl(dppclk_calculated);
dml_get_pipe_attr_decl(surface_size_in_mall);
dml_get_pipe_attr_decl(min_ttu_vblank_in_us);
dml_get_pipe_attr_decl(vratio_prefetch_l);
dml_get_pipe_attr_decl(vratio_prefetch_c);
dml_get_pipe_attr_decl(dst_x_after_scaler);
dml_get_pipe_attr_decl(dst_y_after_scaler);
dml_get_pipe_attr_decl(dst_y_per_vm_vblank);
dml_get_pipe_attr_decl(dst_y_per_row_vblank);
dml_get_pipe_attr_decl(dst_y_prefetch);
dml_get_pipe_attr_decl(dst_y_per_vm_flip);
dml_get_pipe_attr_decl(dst_y_per_row_flip);
dml_get_pipe_attr_decl(dst_y_per_pte_row_nom_l);
dml_get_pipe_attr_decl(dst_y_per_pte_row_nom_c);
dml_get_pipe_attr_decl(dst_y_per_meta_row_nom_l);
dml_get_pipe_attr_decl(dst_y_per_meta_row_nom_c);
dml_get_pipe_attr_decl(dpte_row_height_linear_c);
dml_get_pipe_attr_decl(swath_height_l);
dml_get_pipe_attr_decl(swath_height_c);
dml_get_pipe_attr_decl(det_stored_buffer_size_l_bytes);
dml_get_pipe_attr_decl(det_stored_buffer_size_c_bytes);
dml_get_pipe_attr_decl(dpte_group_size_in_bytes);
dml_get_pipe_attr_decl(vm_group_size_in_bytes);
dml_get_pipe_attr_decl(det_buffer_size_kbytes);
dml_get_pipe_attr_decl(dpte_row_height_linear_l);
dml_get_pipe_attr_decl(refcyc_per_pte_group_nom_l_in_us);
dml_get_pipe_attr_decl(refcyc_per_pte_group_nom_c_in_us);
dml_get_pipe_attr_decl(refcyc_per_pte_group_vblank_l_in_us);
dml_get_pipe_attr_decl(refcyc_per_pte_group_vblank_c_in_us);
dml_get_pipe_attr_decl(refcyc_per_pte_group_flip_l_in_us);
dml_get_pipe_attr_decl(refcyc_per_pte_group_flip_c_in_us);
dml_get_pipe_attr_decl(refcyc_per_vm_group_vblank_in_us);
dml_get_pipe_attr_decl(refcyc_per_vm_group_flip_in_us);
dml_get_pipe_attr_decl(refcyc_per_vm_req_vblank_in_us);
dml_get_pipe_attr_decl(refcyc_per_vm_req_flip_in_us);
dml_get_pipe_attr_decl(refcyc_per_vm_dmdata_in_us);
dml_get_pipe_attr_decl(dmdata_dl_delta_in_us);
dml_get_pipe_attr_decl(refcyc_per_line_delivery_l_in_us);
dml_get_pipe_attr_decl(refcyc_per_line_delivery_c_in_us);
dml_get_pipe_attr_decl(refcyc_per_line_delivery_pre_l_in_us);
dml_get_pipe_attr_decl(refcyc_per_line_delivery_pre_c_in_us);
dml_get_pipe_attr_decl(refcyc_per_req_delivery_l_in_us);
dml_get_pipe_attr_decl(refcyc_per_req_delivery_c_in_us);
dml_get_pipe_attr_decl(refcyc_per_req_delivery_pre_l_in_us);
dml_get_pipe_attr_decl(refcyc_per_req_delivery_pre_c_in_us);
dml_get_pipe_attr_decl(refcyc_per_cursor_req_delivery_in_us);
dml_get_pipe_attr_decl(refcyc_per_cursor_req_delivery_pre_in_us);
dml_get_pipe_attr_decl(refcyc_per_meta_chunk_nom_l_in_us);
dml_get_pipe_attr_decl(refcyc_per_meta_chunk_nom_c_in_us);
dml_get_pipe_attr_decl(refcyc_per_meta_chunk_vblank_l_in_us);
dml_get_pipe_attr_decl(refcyc_per_meta_chunk_vblank_c_in_us);
dml_get_pipe_attr_decl(refcyc_per_meta_chunk_flip_l_in_us);
dml_get_pipe_attr_decl(refcyc_per_meta_chunk_flip_c_in_us);

dml_get_pipe_attr_decl(vstartup);
dml_get_pipe_attr_decl(vupdate_offset);
dml_get_pipe_attr_decl(vupdate_width);
dml_get_pipe_attr_decl(vready_offset);
dml_get_pipe_attr_decl(vready_at_or_after_vsync);
dml_get_pipe_attr_decl(min_dst_y_next_start);

unsigned int dml_get_voltage_level(
        struct display_mode_lib *mode_lib,
        const display_e2e_pipe_params_st *pipes,
        unsigned int num_pipes);


bool get_is_phantom_pipe(struct display_mode_lib *mode_lib,
        const display_e2e_pipe_params_st *pipes,
        unsigned int num_pipes,
        unsigned int pipe_idx);
void PixelClockAdjustmentForProgressiveToInterlaceUnit(struct display_mode_lib *mode_lib);


struct DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation {
    unsigned int dummy_integer_array[2][DML32_NUM_PLANES];
    double dummy_single_array[2][DML32_NUM_PLANES];
    unsigned int dummy_long_array[2][DML32_NUM_PLANES];
    double dummy_double_array[2][DML32_NUM_PLANES];
    bool dummy_boolean_array[DML32_NUM_PLANES];
    bool dummy_boolean;
    bool dummy_boolean2;
    enum output_encoder_class dummy_output_encoder_array[DML32_NUM_PLANES];
    DmlPipe SurfaceParameters[DML32_NUM_PLANES];
    bool dummy_boolean_array2[2][DML32_NUM_PLANES];
    unsigned int ReorderBytes;
    unsigned int VMDataOnlyReturnBW;
    double HostVMInefficiencyFactor;
    DmlPipe myPipe;
    SOCParametersList mmSOCParameters;
    double dummy_unit_vector[DML32_NUM_PLANES];
    double dummy_single[2];
    enum clock_change_support dummy_dramchange_support;
    enum dm_fclock_change_support dummy_fclkchange_support;
    bool dummy_USRRetrainingSupport;
};

struct dml32_ModeSupportAndSystemConfigurationFull {
    unsigned int dummy_integer_array[22][DML32_NUM_PLANES];
    double dummy_double_array[2][DML32_NUM_PLANES];
    DmlPipe SurfParameters[DML32_NUM_PLANES];
    double dummy_single[5];
    double dummy_single2[5];
    SOCParametersList mSOCParameters;
    unsigned int MaximumSwathWidthSupportLuma;
    unsigned int MaximumSwathWidthSupportChroma;
    double DSTYAfterScaler[DML32_NUM_PLANES];
    double DSTXAfterScaler[DML32_NUM_PLANES];
    double MaxTotalVActiveRDBandwidth;
    bool dummy_boolean_array[2][DML32_NUM_PLANES];
    enum odm_combine_mode dummy_odm_mode[DML32_NUM_PLANES];
    DmlPipe myPipe;
    unsigned int dummy_integer[4];
    unsigned int TotalNumberOfActiveOTG;
    unsigned int TotalNumberOfActiveHDMIFRL;
    unsigned int TotalNumberOfActiveDP2p0;
    unsigned int TotalNumberOfActiveDP2p0Outputs;
    unsigned int TotalDSCUnitsRequired;
    unsigned int ReorderingBytes;
    unsigned int TotalSlots;
    unsigned int NumberOfDPPDSC;
    unsigned int NumberOfDPPNoDSC;
    unsigned int NextPrefetchModeState;
    bool MPCCombineMethodAsNeededForPStateChangeAndVoltage;
    bool MPCCombineMethodAsPossible;
    bool FullFrameMALLPStateMethod;
    bool SubViewportMALLPStateMethod;
    bool PhantomPipeMALLPStateMethod;
    bool NoChroma;
    bool TotalAvailablePipesSupportNoDSC;
    bool TotalAvailablePipesSupportDSC;
    enum odm_combine_mode ODMModeNoDSC;
    enum odm_combine_mode ODMModeDSC;
    double RequiredDISPCLKPerSurfaceNoDSC;
    double RequiredDISPCLKPerSurfaceDSC;
    double BWOfNonCombinedSurfaceOfMaximumBandwidth;
    double VMDataOnlyReturnBWPerState;
    double HostVMInefficiencyFactor;
    bool dummy_boolean[2];
    // The upstream function-local CompBufReservedSpaceNeedAdjustmentSingleDPP,
    // kept here because the port splits the function into sections.
    bool CompBufReservedSpaceNeedAdjustmentSingleDPP;
};

struct dummy_vars {
    struct DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation
    DISPCLKDPPCLKDCFCLKDeepSleepPrefetchParametersWatermarksAndPerformanceCalculation;
    struct dml32_ModeSupportAndSystemConfigurationFull dml32_ModeSupportAndSystemConfigurationFull;
};

struct vba_vars_st {
    ip_params_st ip;
    soc_bounding_box_st soc;

    int maxMpcComb;
    bool UseMaximumVStartup;

    double MaxVRatioPre;
    double WritebackDISPCLK;
    double DPPCLKUsingSingleDPPLuma;
    double DPPCLKUsingSingleDPPChroma;
    double DISPCLKWithRamping;
    double DISPCLKWithoutRamping;
    double GlobalDPPCLK;
    double DISPCLKWithRampingRoundedToDFSGranularity;
    double DISPCLKWithoutRampingRoundedToDFSGranularity;
    double MaxDispclkRoundedToDFSGranularity;
    bool DCCEnabledAnyPlane;
    double ReturnBandwidthToDCN;
    unsigned int TotalActiveDPP;
    unsigned int TotalDCCActiveDPP;
    double UrgentRoundTripAndOutOfOrderLatency;
    double StutterPeriod;
    double FrameTimeForMinFullDETBufferingTime;
    double AverageReadBandwidth;
    double TotalRowReadBandwidth;
    double PartOfBurstThatFitsInROB;
    double StutterBurstTime;
    unsigned int NextPrefetchMode;
    double NextMaxVStartup;
    double VBlankTime;
    double SmallestVBlank;
    enum dm_prefetch_modes AllowForPStateChangeOrStutterInVBlankFinal; // Mode Support only
    double DCFCLKDeepSleepPerPlane[DML32_NUM_PLANES];
    double EffectiveDETPlusLBLinesLuma;
    double EffectiveDETPlusLBLinesChroma;
    double UrgentLatencySupportUsLuma;
    double UrgentLatencySupportUsChroma;
    unsigned int DSCFormatFactor;

    bool DummyPStateCheck;
    bool DRAMClockChangeSupportsVActive;
    bool PrefetchModeSupported;
    bool PrefetchAndImmediateFlipSupported;
    enum self_refresh_affinity AllowDRAMSelfRefreshOrDRAMClockChangeInVblank; // Mode Support only
    double XFCRemoteSurfaceFlipDelay;
    double TInitXFill;
    double TslvChk;
    double SrcActiveDrainRate;
    bool ImmediateFlipSupported;
    enum mpc_combine_affinity WhenToDoMPCCombine; // Mode Support only

    bool PrefetchERROR;

    unsigned int VStartupLines;
    unsigned int ActiveDPPs;
    unsigned int LBLatencyHidingSourceLinesY;
    unsigned int LBLatencyHidingSourceLinesC;
    double ActiveDRAMClockChangeLatencyMarginPerState[DML32_NUM_STATES][2][DML32_NUM_PLANES];// DML doesn't save active margin per state
    double ActiveDRAMClockChangeLatencyMargin[DML32_NUM_PLANES];
    double CachedActiveDRAMClockChangeLatencyMargin[DML32_NUM_PLANES]; // Cache in dml_get_voltage_level for debug purposes only
    double MinActiveDRAMClockChangeMargin;
    double InitFillLevel;
    double FinalFillMargin;
    double FinalFillLevel;
    double RemainingFillLevel;
    double TFinalxFill;

    //
    // SOC Bounding Box Parameters
    //
    double SRExitTime;
    double SREnterPlusExitTime;
    double UrgentLatencyPixelDataOnly;
    double UrgentLatencyPixelMixedWithVMData;
    double UrgentLatencyVMDataOnly;
    double UrgentLatency; // max of the above three
    double USRRetrainingLatency;
    double SMNLatency;
    double FCLKChangeLatency;
    unsigned int MALLAllocatedForDCNFinal;
    double MaxAveragePercentOfIdealFabricBWDisplayCanUseInNormalSystemOperation;
    double MaxAveragePercentOfIdealDRAMBWDisplayCanUseInNormalSystemOperationSTROBE;
    double PercentOfIdealDRAMBWReceivedAfterUrgLatencySTROBE;
    double WritebackLatency;
    double PercentOfIdealDRAMFabricAndSDPPortBWReceivedAfterUrgLatencyPixelDataOnly; // Mode Support
    double PercentOfIdealDRAMFabricAndSDPPortBWReceivedAfterUrgLatencyPixelMixedWithVMData; // Mode Support
    double PercentOfIdealDRAMFabricAndSDPPortBWReceivedAfterUrgLatencyVMDataOnly; // Mode Support
    double MaxAveragePercentOfIdealSDPPortBWDisplayCanUseInNormalSystemOperation; // Mode Support
    double MaxAveragePercentOfIdealDRAMBWDisplayCanUseInNormalSystemOperation; // Mode Support
    double NumberOfChannels;
    double DRAMChannelWidth;
    double FabricDatapathToDCNDataReturn;
    double ReturnBusWidth;
    double Downspreading;
    double DISPCLKDPPCLKDSCCLKDownSpreading;
    double DISPCLKDPPCLKVCOSpeed;
    double RoundTripPingLatencyCycles;
    double UrgentOutOfOrderReturnPerChannel;
    double UrgentOutOfOrderReturnPerChannelPixelDataOnly;
    double UrgentOutOfOrderReturnPerChannelPixelMixedWithVMData;
    double UrgentOutOfOrderReturnPerChannelVMDataOnly;
    unsigned int VMMPageSize;
    double DRAMClockChangeLatency;
    double XFCBusTransportTime;
    bool UseUrgentBurstBandwidth;
    double XFCXBUFLatencyTolerance;

    //
    // IP Parameters
    //
    unsigned int ROBBufferSizeInKByte;
    unsigned int DETBufferSizeInKByte[DML32_NUM_PLANES];
    double DETBufferSizeInTime;
    unsigned int DPPOutputBufferPixels;
    unsigned int OPPOutputBufferLines;
    unsigned int PixelChunkSizeInKByte;
    double ReturnBW;
    bool GPUVMEnable;
    bool HostVMEnable;
    unsigned int GPUVMMaxPageTableLevels;
    unsigned int HostVMMaxPageTableLevels;
    unsigned int HostVMCachedPageTableLevels;
    unsigned int OverrideGPUVMPageTableLevels;
    unsigned int OverrideHostVMPageTableLevels;
    unsigned int MetaChunkSize;
    unsigned int MinMetaChunkSizeBytes;
    unsigned int WritebackChunkSize;
    bool ODMCapability;
    unsigned int NumberOfDSC;
    unsigned int LineBufferSize;
    unsigned int MaxLineBufferLines;
    unsigned int WritebackInterfaceLumaBufferSize;
    unsigned int WritebackInterfaceChromaBufferSize;
    unsigned int WritebackChromaLineBufferWidth;
    enum writeback_config WritebackConfiguration;
    double MaxDCHUBToPSCLThroughput;
    double MaxPSCLToLBThroughput;
    unsigned int PTEBufferSizeInRequestsLuma;
    unsigned int PTEBufferSizeInRequestsChroma;
    double DISPCLKRampingMargin;
    unsigned int MaxInterDCNTileRepeaters;
    bool XFCSupported;
    double XFCSlvChunkSize;
    double XFCFillBWOverhead;
    double XFCFillConstant;
    double XFCTSlvVupdateOffset;
    double XFCTSlvVupdateWidth;
    double XFCTSlvVreadyOffset;
    double DPPCLKDelaySubtotal;
    double DPPCLKDelaySCL;
    double DPPCLKDelaySCLLBOnly;
    double DPPCLKDelayCNVCFormater;
    double DPPCLKDelayCNVCCursor;
    double DISPCLKDelaySubtotal;
    bool ProgressiveToInterlaceUnitInOPP;
    unsigned int CompressedBufferSegmentSizeInkByteFinal;
    unsigned int CompbufReservedSpace64B;
    unsigned int CompbufReservedSpaceZs;
    unsigned int LineBufferSizeFinal;
    unsigned int MaximumPixelsPerLinePerDSCUnit;
    unsigned int AlphaPixelChunkSizeInKByte;
    double MinPixelChunkSizeBytes;
    unsigned int DCCMetaBufferSizeBytes;
    // Pipe/Plane Parameters

    /** @VoltageLevel:
     * Every ASIC has a fixed number of DPM states, and some devices might
     * have some particular voltage configuration that does not map
     * directly to the DPM states. This field tells how many states the
     * target device supports; even though this field combines the DPM and
     * special SOC voltages, it mostly matches the total number of DPM
     * states.
     */
    int VoltageLevel;
    double FabricClock;
    double DRAMSpeed;
    double DISPCLK;
    double SOCCLK;
    double DCFCLK;
    unsigned int MaxTotalDETInKByte;
    unsigned int MinCompressedBufferSizeInKByte;
    unsigned int NumberOfActiveSurfaces;
    bool ViewportStationary[DML32_NUM_PLANES];
    unsigned int RefreshRate[DML32_NUM_PLANES];
    double       OutputBPP[DML32_NUM_PLANES];
    unsigned int GPUVMMinPageSizeKBytes[DML32_NUM_PLANES];
    bool SynchronizeTimingsFinal;
    bool SynchronizeDRRDisplaysForUCLKPStateChangeFinal;
    bool ForceOneRowForFrame[DML32_NUM_PLANES];
    unsigned int ViewportXStartY[DML32_NUM_PLANES];
    unsigned int ViewportXStartC[DML32_NUM_PLANES];
    enum dm_rotation_angle SourceRotation[DML32_NUM_PLANES];
    bool DRRDisplay[DML32_NUM_PLANES];
    bool PteBufferMode[DML32_NUM_PLANES];
    enum dm_output_type OutputType[DML32_NUM_PLANES];
    enum dm_output_rate OutputRate[DML32_NUM_PLANES];

    unsigned int NumberOfActivePlanes;
    unsigned int NumberOfDSCSlices[DML32_NUM_PLANES];
    unsigned int ViewportWidth[DML32_NUM_PLANES];
    unsigned int ViewportHeight[DML32_NUM_PLANES];
    unsigned int ViewportYStartY[DML32_NUM_PLANES];
    unsigned int ViewportYStartC[DML32_NUM_PLANES];
    unsigned int PitchY[DML32_NUM_PLANES];
    unsigned int PitchC[DML32_NUM_PLANES];
    double HRatio[DML32_NUM_PLANES];
    double VRatio[DML32_NUM_PLANES];
    unsigned int htaps[DML32_NUM_PLANES];
    unsigned int vtaps[DML32_NUM_PLANES];
    unsigned int HTAPsChroma[DML32_NUM_PLANES];
    unsigned int VTAPsChroma[DML32_NUM_PLANES];
    unsigned int HTotal[DML32_NUM_PLANES];
    unsigned int VTotal[DML32_NUM_PLANES];
    unsigned int VTotal_Max[DML32_NUM_PLANES];
    unsigned int VTotal_Min[DML32_NUM_PLANES];
    int DPPPerPlane[DML32_NUM_PLANES];
    double PixelClock[DML32_NUM_PLANES];
    double PixelClockBackEnd[DML32_NUM_PLANES];
    bool DCCEnable[DML32_NUM_PLANES];
    bool FECEnable[DML32_NUM_PLANES];
    unsigned int DCCMetaPitchY[DML32_NUM_PLANES];
    unsigned int DCCMetaPitchC[DML32_NUM_PLANES];
    enum scan_direction_class SourceScan[DML32_NUM_PLANES];
    enum source_format_class SourcePixelFormat[DML32_NUM_PLANES];
    bool WritebackEnable[DML32_NUM_PLANES];
    unsigned int ActiveWritebacksPerPlane[DML32_NUM_PLANES];
    double WritebackDestinationWidth[DML32_NUM_PLANES];
    double WritebackDestinationHeight[DML32_NUM_PLANES];
    double WritebackSourceHeight[DML32_NUM_PLANES];
    enum source_format_class WritebackPixelFormat[DML32_NUM_PLANES];
    unsigned int WritebackLumaHTaps[DML32_NUM_PLANES];
    unsigned int WritebackLumaVTaps[DML32_NUM_PLANES];
    unsigned int WritebackChromaHTaps[DML32_NUM_PLANES];
    unsigned int WritebackChromaVTaps[DML32_NUM_PLANES];
    double WritebackHRatio[DML32_NUM_PLANES];
    double WritebackVRatio[DML32_NUM_PLANES];
    unsigned int HActive[DML32_NUM_PLANES];
    unsigned int VActive[DML32_NUM_PLANES];
    bool Interlace[DML32_NUM_PLANES];
    enum dm_swizzle_mode SurfaceTiling[DML32_NUM_PLANES];
    unsigned int ScalerRecoutWidth[DML32_NUM_PLANES];
    bool DynamicMetadataEnable[DML32_NUM_PLANES];
    int DynamicMetadataLinesBeforeActiveRequired[DML32_NUM_PLANES];
    unsigned int DynamicMetadataTransmittedBytes[DML32_NUM_PLANES];
    double DCCRate[DML32_NUM_PLANES];
    double AverageDCCCompressionRate;
    enum odm_combine_mode ODMCombineEnabled[DML32_NUM_PLANES];
    double OutputBpp[DML32_NUM_PLANES];
    bool DSCEnabled[DML32_NUM_PLANES];
    unsigned int DSCInputBitPerComponent[DML32_NUM_PLANES];
    enum output_format_class OutputFormat[DML32_NUM_PLANES];
    enum output_encoder_class Output[DML32_NUM_PLANES];
    bool skip_dio_check[DML32_NUM_PLANES];
    unsigned int BlendingAndTiming[DML32_NUM_PLANES];
    bool SynchronizedVBlank;
    unsigned int NumberOfCursors[DML32_NUM_PLANES];
    unsigned int CursorWidth[DML32_NUM_PLANES][DC__NUM_CURSOR__MAX];
    unsigned int CursorBPP[DML32_NUM_PLANES][DC__NUM_CURSOR__MAX];
    bool XFCEnabled[DML32_NUM_PLANES];
    bool ScalerEnabled[DML32_NUM_PLANES];
    unsigned int VBlankNom[DML32_NUM_PLANES];
    bool DisableUnboundRequestIfCompBufReservedSpaceNeedAdjustment;

    // Intermediates/Informational
    bool ImmediateFlipSupport;
    unsigned int DETBufferSizeY[DML32_NUM_PLANES];
    unsigned int DETBufferSizeC[DML32_NUM_PLANES];
    unsigned int SwathHeightY[DML32_NUM_PLANES];
    unsigned int SwathHeightC[DML32_NUM_PLANES];
    unsigned int LBBitPerPixel[DML32_NUM_PLANES];
    double LastPixelOfLineExtraWatermark;
    double TotalDataReadBandwidth;
    unsigned int TotalActiveWriteback;
    unsigned int EffectiveLBLatencyHidingSourceLinesLuma;
    unsigned int EffectiveLBLatencyHidingSourceLinesChroma;
    double BandwidthAvailableForImmediateFlip;
    unsigned int PrefetchMode[DML32_NUM_STATES][2];
    unsigned int PrefetchModePerState[DML32_NUM_STATES][2];
    unsigned int MinPrefetchMode;
    unsigned int MaxPrefetchMode;
    bool AnyLinesForVMOrRowTooLarge;
    double MaxVStartup;
    bool IgnoreViewportPositioning;
    bool ErrorResult[DML32_NUM_PLANES];
    //
    // Calculated dml_ml->vba.Outputs
    //
    double DCFCLKDeepSleep;
    double UrgentWatermark;
    double UrgentExtraLatency;
    double WritebackUrgentWatermark;
    double StutterExitWatermark;
    double StutterEnterPlusExitWatermark;
    double DRAMClockChangeWatermark;
    double WritebackDRAMClockChangeWatermark;
    double StutterEfficiency;
    double StutterEfficiencyNotIncludingVBlank;
    double NonUrgentLatencyTolerance;
    double MinActiveDRAMClockChangeLatencySupported;
    double Z8StutterEfficiencyBestCase;
    unsigned int Z8NumberOfStutterBurstsPerFrameBestCase;
    double Z8StutterEfficiencyNotIncludingVBlankBestCase;
    double StutterPeriodBestCase;
    Watermarks      Watermark;
    bool DCHUBBUB_ARB_CSTATE_MAX_CAP_MODE;
    unsigned int CompBufReservedSpaceKBytes;
    unsigned int CompBufReservedSpace64B;
    unsigned int CompBufReservedSpaceZs;
    bool CompBufReservedSpaceNeedAdjustment;

    // These are the clocks calcuated by the library but they are not actually
    // used explicitly. They are fetched by tests and then possibly used. The
    // ultimate values to use are the ones specified by the parameters to DML
    double DISPCLK_calculated;
    double DPPCLK_calculated[DML32_NUM_PLANES];

    bool ImmediateFlipSupportedSurface[DML32_NUM_PLANES];

    bool Use_One_Row_For_Frame[DML32_NUM_PLANES];
    bool Use_One_Row_For_Frame_Flip[DML32_NUM_PLANES];
    unsigned int VUpdateOffsetPix[DML32_NUM_PLANES];
    double VUpdateWidthPix[DML32_NUM_PLANES];
    double VReadyOffsetPix[DML32_NUM_PLANES];

    unsigned int TotImmediateFlipBytes;
    double TCalc;

    display_e2e_pipe_params_st cache_pipes[DML32_NUM_PLANES];
    unsigned int cache_num_pipes;
    unsigned int pipe_plane[DML32_NUM_PLANES];

    /* vba mode support */
    /*inputs*/
    bool SupportGFX7CompatibleTilingIn32bppAnd64bpp;
    double MaxHSCLRatio;
    double MaxVSCLRatio;
    unsigned int MaxNumWriteback;
    bool WritebackLumaAndChromaScalingSupported;
    bool Cursor64BppSupport;
    double DCFCLKPerState[DML32_NUM_STATES];
    double DCFCLKState[DML32_NUM_STATES][2];
    double FabricClockPerState[DML32_NUM_STATES];
    double SOCCLKPerState[DML32_NUM_STATES];
    double PHYCLKPerState[DML32_NUM_STATES];
    double DTBCLKPerState[DML32_NUM_STATES];
    double MaxDppclk[DML32_NUM_STATES];
    double MaxDSCCLK[DML32_NUM_STATES];
    double DRAMSpeedPerState[DML32_NUM_STATES];
    double MaxDispclk[DML32_NUM_STATES];
    int VoltageOverrideLevel;
    double PHYCLKD32PerState[DML32_NUM_STATES];

    /*outputs*/
    bool ScaleRatioAndTapsSupport;
    bool SourceFormatPixelAndScanSupport;
    double TotalBandwidthConsumedGBytePerSecond;
    bool DCCEnabledInAnyPlane;
    bool WritebackLatencySupport;
    bool WritebackModeSupport;
    bool Writeback10bpc420Supported;
    bool BandwidthSupport[DML32_NUM_STATES];
    unsigned int TotalNumberOfActiveWriteback;
    double CriticalPoint;
    double ReturnBWToDCNPerState;
    bool IsErrorResult[DML32_NUM_STATES][2][DML32_NUM_PLANES];
    bool prefetch_vm_bw_valid;
    bool prefetch_row_bw_valid;
    bool NumberOfOTGSupport;
    bool NonsupportedDSCInputBPC;
    bool WritebackScaleRatioAndTapsSupport;
    bool CursorSupport;
    bool PitchSupport;
    enum dm_validation_status ValidationStatus[DML32_NUM_STATES];

    /* Mode Support Reason */
    bool P2IWith420;
    bool DSCOnlyIfNecessaryWithBPP;
    bool DSC422NativeNotSupported;
    bool LinkRateDoesNotMatchDPVersion;
    bool LinkRateForMultistreamNotIndicated;
    bool BPPForMultistreamNotIndicated;
    bool MultistreamWithHDMIOreDP;
    bool MSOOrODMSplitWithNonDPLink;
    bool NotEnoughLanesForMSO;
    bool ViewportExceedsSurface;

    bool ImmediateFlipRequiredButTheRequirementForEachSurfaceIsNotSpecified;
    bool ImmediateFlipOrHostVMAndPStateWithMALLFullFrameOrPhantomPipe;
    bool InvalidCombinationOfMALLUseForPStateAndStaticScreen;
    bool InvalidCombinationOfMALLUseForPState;

    enum dm_output_link_dp_rate OutputLinkDPRate[DML32_NUM_PLANES];
    double PrefetchLinesYThisState[DML32_NUM_PLANES];
    double PrefetchLinesCThisState[DML32_NUM_PLANES];
    double meta_row_bandwidth_this_state[DML32_NUM_PLANES];
    double dpte_row_bandwidth_this_state[DML32_NUM_PLANES];
    double DPTEBytesPerRowThisState[DML32_NUM_PLANES];
    double PDEAndMetaPTEBytesPerFrameThisState[DML32_NUM_PLANES];
    double MetaRowBytesThisState[DML32_NUM_PLANES];
    bool use_one_row_for_frame[DML32_NUM_STATES][2][DML32_NUM_PLANES];
    bool use_one_row_for_frame_flip[DML32_NUM_STATES][2][DML32_NUM_PLANES];
    bool use_one_row_for_frame_this_state[DML32_NUM_PLANES];
    bool use_one_row_for_frame_flip_this_state[DML32_NUM_PLANES];

    unsigned int OutputTypeAndRatePerState[DML32_NUM_STATES][DML32_NUM_PLANES];
    double RequiredDISPCLKPerSurface[DML32_NUM_STATES][2][DML32_NUM_PLANES];
    unsigned int MacroTileHeightY[DML32_NUM_PLANES];
    unsigned int MacroTileHeightC[DML32_NUM_PLANES];
    unsigned int MacroTileWidthY[DML32_NUM_PLANES];
    unsigned int MacroTileWidthC[DML32_NUM_PLANES];
    bool ImmediateFlipRequiredFinal;
    bool DCCProgrammingAssumesScanDirectionUnknownFinal;
    bool EnoughWritebackUnits;
    bool ODMCombine2To1SupportCheckOK[DML32_NUM_STATES];
    bool NumberOfDP2p0Support;
    unsigned int MaxNumDP2p0Streams;
    unsigned int MaxNumDP2p0Outputs;
    enum dm_output_type OutputTypePerState[DML32_NUM_STATES][DML32_NUM_PLANES];
    enum dm_output_rate OutputRatePerState[DML32_NUM_STATES][DML32_NUM_PLANES];
    double WritebackLineBufferLumaBufferSize;
    double WritebackLineBufferChromaBufferSize;
    double WritebackMinHSCLRatio;
    double WritebackMinVSCLRatio;
    double WritebackMaxHSCLRatio;
    double WritebackMaxVSCLRatio;
    double WritebackMaxHSCLTaps;
    double WritebackMaxVSCLTaps;
    unsigned int MaxNumDPP;
    unsigned int MaxNumOTG;
    double CursorBufferSize;
    double CursorChunkSize;
    unsigned int Mode;
    double OutputLinkDPLanes[DML32_NUM_PLANES];
    double ForcedOutputLinkBPP[DML32_NUM_PLANES]; // Mode Support only
    double ImmediateFlipBW[DML32_NUM_PLANES];
    double MaxMaxVStartup[DML32_NUM_STATES][2];

    double WritebackLumaVExtra;
    double WritebackChromaVExtra;
    double WritebackRequiredDISPCLK;
    double MaximumSwathWidthSupport;
    double MaximumSwathWidthInDETBuffer;
    double MaximumSwathWidthInLineBuffer;
    double MaxDispclkRoundedDownToDFSGranularity;
    double MaxDppclkRoundedDownToDFSGranularity;
    double PlaneRequiredDISPCLKWithoutODMCombine;
    double PlaneRequiredDISPCLKWithODMCombine;
    double PlaneRequiredDISPCLK;
    double TotalNumberOfActiveOTG;
    double FECOverhead;
    double EffectiveFECOverhead;
    double Outbpp;
    unsigned int OutbppDSC;
    double TotalDSCUnitsRequired;
    double bpp;
    unsigned int slices;
    double SwathWidthGranularityY;
    double RoundedUpMaxSwathSizeBytesY;
    double SwathWidthGranularityC;
    double RoundedUpMaxSwathSizeBytesC;
    double EffectiveDETLBLinesLuma;
    double EffectiveDETLBLinesChroma;
    double ProjectedDCFCLKDeepSleep[DML32_NUM_STATES][2];
    double PDEAndMetaPTEBytesPerFrameY;
    double PDEAndMetaPTEBytesPerFrameC;
    unsigned int MetaRowBytesY;
    unsigned int MetaRowBytesC;
    unsigned int DPTEBytesPerRowC;
    unsigned int DPTEBytesPerRowY;
    double ExtraLatency;
    double TimeCalc;
    double TWait;
    double MaximumReadBandwidthWithPrefetch;
    double MaximumReadBandwidthWithoutPrefetch;
    double total_dcn_read_bw_with_flip;
    double total_dcn_read_bw_with_flip_no_urgent_burst;
    double FractionOfUrgentBandwidth;
    double FractionOfUrgentBandwidthImmediateFlip; // Mode Support debugging output

    /* ms locals */
    double IdealSDPPortBandwidthPerState[DML32_NUM_STATES][2];
    unsigned int NoOfDPP[DML32_NUM_STATES][2][DML32_NUM_PLANES];
    int NoOfDPPThisState[DML32_NUM_PLANES];
    enum odm_combine_mode ODMCombineEnablePerState[DML32_NUM_STATES][DML32_NUM_PLANES];
    double SwathWidthYThisState[DML32_NUM_PLANES];
    unsigned int SwathHeightCPerState[DML32_NUM_STATES][2][DML32_NUM_PLANES];
    unsigned int SwathHeightYThisState[DML32_NUM_PLANES];
    unsigned int SwathHeightCThisState[DML32_NUM_PLANES];
    double VRatioPreY[DML32_NUM_STATES][2][DML32_NUM_PLANES];
    double VRatioPreC[DML32_NUM_STATES][2][DML32_NUM_PLANES];
    double RequiredPrefetchPixelDataBWLuma[DML32_NUM_STATES][2][DML32_NUM_PLANES];
    double RequiredPrefetchPixelDataBWChroma[DML32_NUM_STATES][2][DML32_NUM_PLANES];
    double RequiredDPPCLK[DML32_NUM_STATES][2][DML32_NUM_PLANES];
    double RequiredDPPCLKThisState[DML32_NUM_PLANES];
    bool PTEBufferSizeNotExceededY[DML32_NUM_STATES][2][DML32_NUM_PLANES];
    bool PTEBufferSizeNotExceededC[DML32_NUM_STATES][2][DML32_NUM_PLANES];
    bool BandwidthWithoutPrefetchSupported[DML32_NUM_STATES][2];
    bool PrefetchSupported[DML32_NUM_STATES][2];
    bool VRatioInPrefetchSupported[DML32_NUM_STATES][2];
    double RequiredDISPCLK[DML32_NUM_STATES][2];
    bool DISPCLK_DPPCLK_Support[DML32_NUM_STATES][2];
    bool TotalAvailablePipesSupport[DML32_NUM_STATES][2];
    unsigned int TotalNumberOfActiveDPP[DML32_NUM_STATES][2];
    unsigned int TotalNumberOfDCCActiveDPP[DML32_NUM_STATES][2];
    bool ModeSupport[DML32_NUM_STATES][2];
    double ReturnBWPerState[DML32_NUM_STATES][2];
    bool DIOSupport[DML32_NUM_STATES];
    bool NotEnoughDSCUnits[DML32_NUM_STATES];
    bool DSCCLKRequiredMoreThanSupported[DML32_NUM_STATES];
    bool DTBCLKRequiredMoreThanSupported[DML32_NUM_STATES];
    double UrgentRoundTripAndOutOfOrderLatencyPerState[DML32_NUM_STATES];
    bool ROBSupport[DML32_NUM_STATES][2];
    //based on rev 99: Dim DCCMetaBufferSizeSupport(NumberOfStates, 1) As Boolean
    bool DCCMetaBufferSizeSupport[DML32_NUM_STATES][2];
    bool PTEBufferSizeNotExceeded[DML32_NUM_STATES][2];
    bool TotalVerticalActiveBandwidthSupport[DML32_NUM_STATES][2];
    double MaxTotalVerticalActiveAvailableBandwidth[DML32_NUM_STATES][2];
    double PrefetchBW[DML32_NUM_PLANES];
    double PDEAndMetaPTEBytesPerFrame[DML32_NUM_STATES][2][DML32_NUM_PLANES];
    double MetaRowBytes[DML32_NUM_STATES][2][DML32_NUM_PLANES];
    double DPTEBytesPerRow[DML32_NUM_STATES][2][DML32_NUM_PLANES];
    double PrefetchLinesY[DML32_NUM_STATES][2][DML32_NUM_PLANES];
    double PrefetchLinesC[DML32_NUM_STATES][2][DML32_NUM_PLANES];
    unsigned int MaxNumSwY[DML32_NUM_PLANES];
    unsigned int MaxNumSwC[DML32_NUM_PLANES];
    double PrefillY[DML32_NUM_PLANES];
    double PrefillC[DML32_NUM_PLANES];
    double LineTimesForPrefetch[DML32_NUM_PLANES];
    double LinesForMetaPTE[DML32_NUM_PLANES];
    double LinesForMetaAndDPTERow[DML32_NUM_PLANES];
    double MinDPPCLKUsingSingleDPP[DML32_NUM_PLANES];
    double SwathWidthYSingleDPP[DML32_NUM_PLANES];
    double BytePerPixelInDETY[DML32_NUM_PLANES];
    double BytePerPixelInDETC[DML32_NUM_PLANES];
    bool RequiresDSC[DML32_NUM_STATES][DML32_NUM_PLANES];
    unsigned int NumberOfDSCSlice[DML32_NUM_STATES][DML32_NUM_PLANES];
    double RequiresFEC[DML32_NUM_STATES][DML32_NUM_PLANES];
    double OutputBppPerState[DML32_NUM_STATES][DML32_NUM_PLANES];
    double DSCDelayPerState[DML32_NUM_STATES][DML32_NUM_PLANES];
    bool ViewportSizeSupport[DML32_NUM_STATES][2];
    unsigned int Read256BlockHeightY[DML32_NUM_PLANES];
    unsigned int Read256BlockWidthY[DML32_NUM_PLANES];
    unsigned int Read256BlockHeightC[DML32_NUM_PLANES];
    unsigned int Read256BlockWidthC[DML32_NUM_PLANES];
    double MaxSwathHeightY[DML32_NUM_PLANES];
    double MaxSwathHeightC[DML32_NUM_PLANES];
    double MinSwathHeightY[DML32_NUM_PLANES];
    double MinSwathHeightC[DML32_NUM_PLANES];
    double ReadBandwidthLuma[DML32_NUM_PLANES];
    double ReadBandwidthChroma[DML32_NUM_PLANES];
    double ReadBandwidth[DML32_NUM_PLANES];
    double WriteBandwidth[DML32_NUM_PLANES];
    double PSCL_FACTOR[DML32_NUM_PLANES];
    double PSCL_FACTOR_CHROMA[DML32_NUM_PLANES];
    double MaximumVStartup[DML32_NUM_STATES][2][DML32_NUM_PLANES];
    double AlignedDCCMetaPitch[DML32_NUM_PLANES];
    double AlignedYPitch[DML32_NUM_PLANES];
    double AlignedCPitch[DML32_NUM_PLANES];
    double MaximumSwathWidth[DML32_NUM_PLANES];
    double cursor_bw[DML32_NUM_PLANES];
    double cursor_bw_pre[DML32_NUM_PLANES];
    double Tno_bw[DML32_NUM_PLANES];
    double prefetch_vmrow_bw[DML32_NUM_PLANES];
    double DestinationLinesToRequestVMInImmediateFlip[DML32_NUM_PLANES];
    double DestinationLinesToRequestRowInImmediateFlip[DML32_NUM_PLANES];
    double final_flip_bw[DML32_NUM_PLANES];
    bool ImmediateFlipSupportedForState[DML32_NUM_STATES][2];
    double WritebackDelay[DML32_NUM_STATES][DML32_NUM_PLANES];
    unsigned int vm_group_bytes[DML32_NUM_PLANES];
    unsigned int dpte_group_bytes[DML32_NUM_PLANES];
    unsigned int dpte_row_height[DML32_NUM_PLANES];
    unsigned int meta_req_height[DML32_NUM_PLANES];
    unsigned int meta_req_width[DML32_NUM_PLANES];
    unsigned int meta_row_height[DML32_NUM_PLANES];
    unsigned int meta_row_width[DML32_NUM_PLANES];
    unsigned int dpte_row_height_chroma[DML32_NUM_PLANES];
    unsigned int meta_req_height_chroma[DML32_NUM_PLANES];
    unsigned int meta_req_width_chroma[DML32_NUM_PLANES];
    unsigned int meta_row_height_chroma[DML32_NUM_PLANES];
    unsigned int meta_row_width_chroma[DML32_NUM_PLANES];
    bool ImmediateFlipSupportedForPipe[DML32_NUM_PLANES];
    double meta_row_bw[DML32_NUM_PLANES];
    double dpte_row_bw[DML32_NUM_PLANES];
    double DisplayPipeLineDeliveryTimeLuma[DML32_NUM_PLANES];                     // WM
    double DisplayPipeLineDeliveryTimeChroma[DML32_NUM_PLANES];                     // WM
    double DisplayPipeRequestDeliveryTimeLuma[DML32_NUM_PLANES];
    double DisplayPipeRequestDeliveryTimeChroma[DML32_NUM_PLANES];
    enum clock_change_support DRAMClockChangeSupport[DML32_NUM_STATES][2];
    double UrgentBurstFactorCursor[DML32_NUM_PLANES];
    double UrgentBurstFactorCursorPre[DML32_NUM_PLANES];
    double UrgentBurstFactorLuma[DML32_NUM_PLANES];
    double UrgentBurstFactorLumaPre[DML32_NUM_PLANES];
    double UrgentBurstFactorChroma[DML32_NUM_PLANES];
    double UrgentBurstFactorChromaPre[DML32_NUM_PLANES];

    bool           MPCCombine[DML32_NUM_STATES][2][DML32_NUM_PLANES];
    double         SwathWidthCSingleDPP[DML32_NUM_PLANES];
    double         MaximumSwathWidthInLineBufferLuma;
    double         MaximumSwathWidthInLineBufferChroma;
    double         MaximumSwathWidthLuma[DML32_NUM_PLANES];
    double         MaximumSwathWidthChroma[DML32_NUM_PLANES];
    enum odm_combine_mode odm_combine_dummy[DML32_NUM_PLANES];
    double         dummy1[DML32_NUM_PLANES];
    double         dummy2[DML32_NUM_PLANES];
    unsigned int   dummy3[DML32_NUM_PLANES];
    unsigned int   dummy4[DML32_NUM_PLANES];
    double         dummy5;
    double         dummy6;
    double         dummy7[DML32_NUM_PLANES];
    double         dummy8[DML32_NUM_PLANES];
    double         dummy13[DML32_NUM_PLANES];
    double         dummy_double_array[2][DML32_NUM_PLANES];
    unsigned int        dummyinteger3[DML32_NUM_PLANES];
    unsigned int        dummyinteger4[DML32_NUM_PLANES];
    unsigned int        dummyinteger5;
    unsigned int        dummyinteger6;
    unsigned int        dummyinteger7;
    unsigned int        dummyinteger8;
    unsigned int        dummyinteger9;
    unsigned int        dummyinteger10;
    unsigned int        dummyinteger11;
    unsigned int        dummy_integer_array[8][DML32_NUM_PLANES];

    bool           dummysinglestring;
    bool           SingleDPPViewportSizeSupportPerPlane[DML32_NUM_PLANES];
    double         PlaneRequiredDISPCLKWithODMCombine2To1;
    double         PlaneRequiredDISPCLKWithODMCombine4To1;
    unsigned int   TotalNumberOfSingleDPPPlanes[DML32_NUM_STATES][2];
    bool           LinkDSCEnable;
    bool           ODMCombine4To1SupportCheckOK[DML32_NUM_STATES];
    enum odm_combine_mode ODMCombineEnableThisState[DML32_NUM_PLANES];
    double   SwathWidthCThisState[DML32_NUM_PLANES];
    bool           ViewportSizeSupportPerPlane[DML32_NUM_PLANES];
    double         AlignedDCCMetaPitchY[DML32_NUM_PLANES];
    double         AlignedDCCMetaPitchC[DML32_NUM_PLANES];

    unsigned int NotEnoughUrgentLatencyHiding[DML32_NUM_STATES][2];
    unsigned int NotEnoughUrgentLatencyHidingPre;
    int PTEBufferSizeInRequestsForLuma;
    int PTEBufferSizeInRequestsForChroma;

    // Missing from VBA
    int dpte_group_bytes_chroma;
    unsigned int vm_group_bytes_chroma;
    double dst_x_after_scaler;
    double dst_y_after_scaler;
    unsigned int VStartupRequiredWhenNotEnoughTimeForDynamicMetadata;

    /* perf locals*/
    double PrefetchBandwidth[DML32_NUM_PLANES];
    double VInitPreFillY[DML32_NUM_PLANES];
    double VInitPreFillC[DML32_NUM_PLANES];
    unsigned int MaxNumSwathY[DML32_NUM_PLANES];
    unsigned int MaxNumSwathC[DML32_NUM_PLANES];
    unsigned int VStartup[DML32_NUM_PLANES];
    double DSTYAfterScaler[DML32_NUM_PLANES];
    double DSTXAfterScaler[DML32_NUM_PLANES];
    bool AllowDRAMClockChangeDuringVBlank[DML32_NUM_PLANES];
    bool AllowDRAMSelfRefreshDuringVBlank[DML32_NUM_PLANES];
    double VRatioPrefetchY[DML32_NUM_PLANES];
    double VRatioPrefetchC[DML32_NUM_PLANES];
    double DestinationLinesForPrefetch[DML32_NUM_PLANES];
    double DestinationLinesToRequestVMInVBlank[DML32_NUM_PLANES];
    double DestinationLinesToRequestRowInVBlank[DML32_NUM_PLANES];
    double MinTTUVBlank[DML32_NUM_PLANES];
    double BytePerPixelDETY[DML32_NUM_PLANES];
    double BytePerPixelDETC[DML32_NUM_PLANES];
    double SwathWidthY[DML32_NUM_PLANES];
    double SwathWidthSingleDPPY[DML32_NUM_PLANES];
    double CursorRequestDeliveryTime[DML32_NUM_PLANES];
    double CursorRequestDeliveryTimePrefetch[DML32_NUM_PLANES];
    double ReadBandwidthPlaneLuma[DML32_NUM_PLANES];
    double ReadBandwidthPlaneChroma[DML32_NUM_PLANES];
    double DisplayPipeLineDeliveryTimeLumaPrefetch[DML32_NUM_PLANES];
    double DisplayPipeLineDeliveryTimeChromaPrefetch[DML32_NUM_PLANES];
    double DisplayPipeRequestDeliveryTimeLumaPrefetch[DML32_NUM_PLANES];
    double DisplayPipeRequestDeliveryTimeChromaPrefetch[DML32_NUM_PLANES];
    double PixelPTEBytesPerRow[DML32_NUM_PLANES];
    double PDEAndMetaPTEBytesFrame[DML32_NUM_PLANES];
    double MetaRowByte[DML32_NUM_PLANES];
    double PrefetchSourceLinesY[DML32_NUM_PLANES];
    double RequiredPrefetchPixDataBWLuma[DML32_NUM_PLANES];
    double RequiredPrefetchPixDataBWChroma[DML32_NUM_PLANES];
    double PrefetchSourceLinesC[DML32_NUM_PLANES];
    double PSCL_THROUGHPUT_LUMA[DML32_NUM_PLANES];
    double PSCL_THROUGHPUT_CHROMA[DML32_NUM_PLANES];
    double DSCCLK_calculated[DML32_NUM_PLANES];
    unsigned int DSCDelay[DML32_NUM_PLANES];
    unsigned int MaxVStartupLines[DML32_NUM_PLANES];
    double DPPCLKUsingSingleDPP[DML32_NUM_PLANES];
    double DPPCLK[DML32_NUM_PLANES];
    unsigned int DCCYMaxUncompressedBlock[DML32_NUM_PLANES];
    unsigned int DCCYMaxCompressedBlock[DML32_NUM_PLANES];
    unsigned int DCCYIndependent64ByteBlock[DML32_NUM_PLANES];
    double MaximumDCCCompressionYSurface[DML32_NUM_PLANES];
    unsigned int BlockHeight256BytesY[DML32_NUM_PLANES];
    unsigned int BlockHeight256BytesC[DML32_NUM_PLANES];
    unsigned int BlockWidth256BytesY[DML32_NUM_PLANES];
    unsigned int BlockWidth256BytesC[DML32_NUM_PLANES];
    double XFCSlaveVUpdateOffset[DML32_NUM_PLANES];
    double XFCSlaveVupdateWidth[DML32_NUM_PLANES];
    double XFCSlaveVReadyOffset[DML32_NUM_PLANES];
    double XFCTransferDelay[DML32_NUM_PLANES];
    double XFCPrechargeDelay[DML32_NUM_PLANES];
    double XFCRemoteSurfaceFlipLatency[DML32_NUM_PLANES];
    double XFCPrefetchMargin[DML32_NUM_PLANES];
    unsigned int dpte_row_width_luma_ub[DML32_NUM_PLANES];
    unsigned int dpte_row_width_chroma_ub[DML32_NUM_PLANES];
    double FullDETBufferingTimeY[DML32_NUM_PLANES];                     // WM
    double FullDETBufferingTimeC[DML32_NUM_PLANES];                     // WM
    double DST_Y_PER_PTE_ROW_NOM_L[DML32_NUM_PLANES];
    double DST_Y_PER_PTE_ROW_NOM_C[DML32_NUM_PLANES];
    double DST_Y_PER_META_ROW_NOM_L[DML32_NUM_PLANES];
    double TimePerMetaChunkNominal[DML32_NUM_PLANES];
    double TimePerMetaChunkVBlank[DML32_NUM_PLANES];
    double TimePerMetaChunkFlip[DML32_NUM_PLANES];
    unsigned int swath_width_luma_ub[DML32_NUM_PLANES];
    unsigned int swath_width_chroma_ub[DML32_NUM_PLANES];
    unsigned int PixelPTEReqWidthY[DML32_NUM_PLANES];
    unsigned int PixelPTEReqHeightY[DML32_NUM_PLANES];
    unsigned int PTERequestSizeY[DML32_NUM_PLANES];
    unsigned int PixelPTEReqWidthC[DML32_NUM_PLANES];
    unsigned int PixelPTEReqHeightC[DML32_NUM_PLANES];
    unsigned int PTERequestSizeC[DML32_NUM_PLANES];
    double time_per_pte_group_nom_luma[DML32_NUM_PLANES];
    double time_per_pte_group_nom_chroma[DML32_NUM_PLANES];
    double time_per_pte_group_vblank_luma[DML32_NUM_PLANES];
    double time_per_pte_group_vblank_chroma[DML32_NUM_PLANES];
    double time_per_pte_group_flip_luma[DML32_NUM_PLANES];
    double time_per_pte_group_flip_chroma[DML32_NUM_PLANES];
    double TimePerVMGroupVBlank[DML32_NUM_PLANES];
    double TimePerVMGroupFlip[DML32_NUM_PLANES];
    double TimePerVMRequestVBlank[DML32_NUM_PLANES];
    double TimePerVMRequestFlip[DML32_NUM_PLANES];
    unsigned int dpde0_bytes_per_frame_ub_l[DML32_NUM_PLANES];
    unsigned int meta_pte_bytes_per_frame_ub_l[DML32_NUM_PLANES];
    unsigned int dpde0_bytes_per_frame_ub_c[DML32_NUM_PLANES];
    unsigned int meta_pte_bytes_per_frame_ub_c[DML32_NUM_PLANES];
    double LinesToFinishSwathTransferStutterCriticalPlane;
    unsigned int BytePerPixelYCriticalPlane;
    double SwathWidthYCriticalPlane;
    double LinesInDETY[DML32_NUM_PLANES];
    double LinesInDETYRoundedDownToSwath[DML32_NUM_PLANES];

    double SwathWidthSingleDPPC[DML32_NUM_PLANES];
    double SwathWidthC[DML32_NUM_PLANES];
    unsigned int BytePerPixelY[DML32_NUM_PLANES];
    unsigned int BytePerPixelC[DML32_NUM_PLANES];
    unsigned int dummyinteger1;
    unsigned int dummyinteger2;
    double FinalDRAMClockChangeLatency;
    double Tdmdl_vm[DML32_NUM_PLANES];
    double Tdmdl[DML32_NUM_PLANES];
    double TSetup[DML32_NUM_PLANES];
    unsigned int ThisVStartup;
    bool WritebackAllowDRAMClockChangeEndPosition[DML32_NUM_PLANES];
    double DST_Y_PER_META_ROW_NOM_C[DML32_NUM_PLANES];
    double TimePerChromaMetaChunkNominal[DML32_NUM_PLANES];
    double TimePerChromaMetaChunkVBlank[DML32_NUM_PLANES];
    double TimePerChromaMetaChunkFlip[DML32_NUM_PLANES];
    unsigned int DCCCMaxUncompressedBlock[DML32_NUM_PLANES];
    unsigned int DCCCMaxCompressedBlock[DML32_NUM_PLANES];
    double VStartupMargin;
    bool NotEnoughTimeForDynamicMetadata[DML32_NUM_PLANES];

    /* Missing from VBA */
    unsigned int MaximumMaxVStartupLines;
    double FabricAndDRAMBandwidth;
    double LinesInDETLuma;
    double LinesInDETChroma;
    unsigned int ImmediateFlipBytes[DML32_NUM_PLANES];
    unsigned int LinesInDETC[DML32_NUM_PLANES];
    unsigned int LinesInDETCRoundedDownToSwath[DML32_NUM_PLANES];
    double UrgentLatencySupportUsPerState[DML32_NUM_STATES][2][DML32_NUM_PLANES];
    double UrgentLatencySupportUs[DML32_NUM_PLANES];
    double FabricAndDRAMBandwidthPerState[DML32_NUM_STATES];
    bool UrgentLatencySupport[DML32_NUM_STATES][2];
    unsigned int SwathWidthYPerState[DML32_NUM_STATES][2][DML32_NUM_PLANES];
    unsigned int SwathHeightYPerState[DML32_NUM_STATES][2][DML32_NUM_PLANES];
    double qual_row_bw[DML32_NUM_PLANES];
    double prefetch_row_bw[DML32_NUM_PLANES];
    double prefetch_vm_bw[DML32_NUM_PLANES];

    double PTEGroupSize;
    unsigned int PDEProcessingBufIn64KBReqs;

    double MaxTotalVActiveRDBandwidth;
    bool DoUrgentLatencyAdjustment;
    double UrgentLatencyAdjustmentFabricClockComponent;
    double UrgentLatencyAdjustmentFabricClockReference;
    double MinUrgentLatencySupportUs;
    double MinFullDETBufferingTime;
    double AverageReadBandwidthGBytePerSecond;
    bool   FirstMainPlane;
    bool NotEnoughDETSwathFillLatencyHiding;

    unsigned int ViewportWidthChroma[DML32_NUM_PLANES];
    unsigned int ViewportHeightChroma[DML32_NUM_PLANES];
    double HRatioChroma[DML32_NUM_PLANES];
    double VRatioChroma[DML32_NUM_PLANES];
    int WritebackSourceWidth[DML32_NUM_PLANES];

    bool ModeIsSupported;
    bool ODMCombine4To1Supported;

    unsigned int SurfaceWidthY[DML32_NUM_PLANES];
    unsigned int SurfaceWidthC[DML32_NUM_PLANES];
    unsigned int SurfaceHeightY[DML32_NUM_PLANES];
    unsigned int SurfaceHeightC[DML32_NUM_PLANES];
    unsigned int WritebackHTaps[DML32_NUM_PLANES];
    unsigned int WritebackVTaps[DML32_NUM_PLANES];
    bool DSCEnable[DML32_NUM_PLANES];

    double DRAMClockChangeLatencyOverride;

    double GPUVMMinPageSize;
    double HostVMMinPageSize;

    bool   MPCCombineEnable[DML32_NUM_PLANES];
    unsigned int HostVMMaxNonCachedPageTableLevels;
    bool   DynamicMetadataVMEnabled;
    double       WritebackInterfaceBufferSize;
    double       WritebackLineBufferSize;

    double DCCRateLuma[DML32_NUM_PLANES];
    double DCCRateChroma[DML32_NUM_PLANES];

    double PHYCLKD18PerState[DML32_NUM_STATES];

    bool WritebackSupportInterleaveAndUsingWholeBufferForASingleStream;
    bool NumberOfHDMIFRLSupport;
    unsigned int MaxNumHDMIFRLOutputs;
    int    AudioSampleRate[DML32_NUM_PLANES];
    int    AudioSampleLayout[DML32_NUM_PLANES];

    int PercentMarginOverMinimumRequiredDCFCLK;
    bool DynamicMetadataSupported[DML32_NUM_STATES][2];
    enum immediate_flip_requirement ImmediateFlipRequirement[DML32_NUM_PLANES];
    unsigned int DETBufferSizeYThisState[DML32_NUM_PLANES];
    unsigned int DETBufferSizeCThisState[DML32_NUM_PLANES];
    bool NoUrgentLatencyHiding[DML32_NUM_PLANES];
    bool NoUrgentLatencyHidingPre[DML32_NUM_PLANES];
    int swath_width_luma_ub_this_state[DML32_NUM_PLANES];
    int swath_width_chroma_ub_this_state[DML32_NUM_PLANES];
    double UrgLatency[DML32_NUM_STATES];
    double VActiveCursorBandwidth[DML32_NUM_STATES][2][DML32_NUM_PLANES];
    double VActivePixelBandwidth[DML32_NUM_STATES][2][DML32_NUM_PLANES];
    bool NoTimeForPrefetch[DML32_NUM_STATES][2][DML32_NUM_PLANES];
    bool NoTimeForDynamicMetadata[DML32_NUM_STATES][2][DML32_NUM_PLANES];
    double dpte_row_bandwidth[DML32_NUM_STATES][2][DML32_NUM_PLANES];
    double meta_row_bandwidth[DML32_NUM_STATES][2][DML32_NUM_PLANES];
    double DETBufferSizeYAllStates[DML32_NUM_STATES][2][DML32_NUM_PLANES];
    double DETBufferSizeCAllStates[DML32_NUM_STATES][2][DML32_NUM_PLANES];
    unsigned int swath_width_luma_ub_all_states[DML32_NUM_STATES][2][DML32_NUM_PLANES];
    unsigned int swath_width_chroma_ub_all_states[DML32_NUM_STATES][2][DML32_NUM_PLANES];
    bool NotUrgentLatencyHiding[DML32_NUM_STATES][2];
    unsigned int SwathHeightYAllStates[DML32_NUM_STATES][2][DML32_NUM_PLANES];
    unsigned int SwathHeightCAllStates[DML32_NUM_STATES][2][DML32_NUM_PLANES];
    unsigned int SwathWidthYAllStates[DML32_NUM_STATES][2][DML32_NUM_PLANES];
    unsigned int SwathWidthCAllStates[DML32_NUM_STATES][2][DML32_NUM_PLANES];
    double TotalDPTERowBandwidth[DML32_NUM_STATES][2];
    double TotalMetaRowBandwidth[DML32_NUM_STATES][2];
    double TotalVActiveCursorBandwidth[DML32_NUM_STATES][2];
    double TotalVActivePixelBandwidth[DML32_NUM_STATES][2];
    double WritebackDelayTime[DML32_NUM_PLANES];
    unsigned int DCCYIndependentBlock[DML32_NUM_PLANES];
    unsigned int DCCCIndependentBlock[DML32_NUM_PLANES];
    unsigned int dummyinteger17;
    unsigned int dummyinteger18;
    unsigned int dummyinteger19;
    unsigned int dummyinteger20;
    unsigned int dummyinteger21;
    unsigned int dummyinteger22;
    unsigned int dummyinteger23;
    unsigned int dummyinteger24;
    unsigned int dummyinteger25;
    unsigned int dummyinteger26;
    unsigned int dummyinteger27;
    unsigned int dummyinteger28;
    unsigned int dummyinteger29;
    bool dummystring[DML32_NUM_PLANES];
    double BPP;
    enum odm_combine_policy ODMCombinePolicy;
    bool UseMinimumRequiredDCFCLK;
    bool ClampMinDCFCLK;
    bool AllowDramClockChangeOneDisplayVactive;

    double MaxAveragePercentOfIdealFabricAndSDPPortBWDisplayCanUseInNormalSystemOperation;
    double PercentOfIdealFabricAndSDPPortBWReceivedAfterUrgLatency;
    double PercentOfIdealDRAMBWReceivedAfterUrgLatencyPixelMixedWithVMData;
    double PercentOfIdealDRAMBWReceivedAfterUrgLatencyVMDataOnly;
    double PercentOfIdealDRAMBWReceivedAfterUrgLatencyPixelDataOnly;
    double SRExitZ8Time;
    double SREnterPlusExitZ8Time;
    double Z8StutterExitWatermark;
    double Z8StutterEnterPlusExitWatermark;
    double Z8StutterEfficiencyNotIncludingVBlank;
    double Z8StutterEfficiency;
    double DCCFractionOfZeroSizeRequestsLuma[DML32_NUM_PLANES];
    double DCCFractionOfZeroSizeRequestsChroma[DML32_NUM_PLANES];
    double UrgBurstFactorCursor[DML32_NUM_PLANES];
    double UrgBurstFactorLuma[DML32_NUM_PLANES];
    double UrgBurstFactorChroma[DML32_NUM_PLANES];
    double UrgBurstFactorCursorPre[DML32_NUM_PLANES];
    double UrgBurstFactorLumaPre[DML32_NUM_PLANES];
    double UrgBurstFactorChromaPre[DML32_NUM_PLANES];
    bool NotUrgentLatencyHidingPre[DML32_NUM_PLANES];
    bool LinkCapacitySupport[DML32_NUM_STATES];
    bool VREADY_AT_OR_AFTER_VSYNC[DML32_NUM_PLANES];
    unsigned int MIN_DST_Y_NEXT_START[DML32_NUM_PLANES];
    unsigned int VFrontPorch[DML32_NUM_PLANES];
    int ConfigReturnBufferSizeInKByte;
    enum unbounded_requesting_policy UseUnboundedRequesting;
    int CompressedBufferSegmentSizeInkByte;
    int CompressedBufferSizeInkByte;
    int MetaFIFOSizeInKEntries;
    int ZeroSizeBufferEntries;
    int COMPBUF_RESERVED_SPACE_64B;
    int COMPBUF_RESERVED_SPACE_ZS;
    bool UnboundedRequestEnabled;
    bool DSC422NativeSupport;
    bool NoEnoughUrgentLatencyHiding;
    bool NoEnoughUrgentLatencyHidingPre;
    int NumberOfStutterBurstsPerFrame;
    int Z8NumberOfStutterBurstsPerFrame;
    unsigned int MaximumDSCBitsPerComponent;
    unsigned int NotEnoughUrgentLatencyHidingA[DML32_NUM_STATES][2];
    double ReadBandwidthSurfaceLuma[DML32_NUM_PLANES];
    double ReadBandwidthSurfaceChroma[DML32_NUM_PLANES];
    double SurfaceRequiredDISPCLKWithoutODMCombine;
    double SurfaceRequiredDISPCLK;
    double MinActiveFCLKChangeLatencySupported;
    int MinVoltageLevel;
    int MaxVoltageLevel;
    unsigned int TotalNumberOfSingleDPPSurfaces[DML32_NUM_STATES][2];
    unsigned int CompressedBufferSizeInkByteAllStates[DML32_NUM_STATES][2];
    unsigned int DETBufferSizeInKByteAllStates[DML32_NUM_STATES][2][DML32_NUM_PLANES];
    unsigned int DETBufferSizeInKByteThisState[DML32_NUM_PLANES];
    unsigned int SurfaceSizeInMALL[DML32_NUM_PLANES];
    bool ExceededMALLSize;
    bool PTE_BUFFER_MODE[DML32_NUM_PLANES];
    unsigned int BIGK_FRAGMENT_SIZE[DML32_NUM_PLANES];
    unsigned int CompressedBufferSizeInkByteThisState;
    enum dm_fclock_change_support FCLKChangeSupport[DML32_NUM_STATES][2];
    bool USRRetrainingSupport[DML32_NUM_STATES][2];
    enum dm_use_mall_for_pstate_change_mode UsesMALLForPStateChange[DML32_NUM_PLANES];
    bool UnboundedRequestEnabledAllStates[DML32_NUM_STATES][2];
    bool SingleDPPViewportSizeSupportPerSurface[DML32_NUM_PLANES];
    enum dm_use_mall_for_static_screen_mode UseMALLForStaticScreen[DML32_NUM_PLANES];
    bool UnboundedRequestEnabledThisState;
    bool DRAMClockChangeRequirementFinal;
    bool FCLKChangeRequirementFinal;
    bool USRRetrainingRequiredFinal;
    unsigned int DETSizeOverride[DML32_NUM_PLANES];
    unsigned int nomDETInKByte;
    enum mpc_combine_affinity  MPCCombineUse[DML32_NUM_PLANES];
    bool MPCCombineMethodIncompatible;
    unsigned int RequiredSlots[DML32_NUM_STATES][DML32_NUM_PLANES];
    bool ExceededMultistreamSlots[DML32_NUM_STATES];
    enum odm_combine_policy ODMUse[DML32_NUM_PLANES];
    unsigned int OutputMultistreamId[DML32_NUM_PLANES];
    bool OutputMultistreamEn[DML32_NUM_PLANES];
    bool UsesMALLForStaticScreen[DML32_NUM_PLANES];
    double MaxActiveDRAMClockChangeLatencySupported[DML32_NUM_PLANES];
    double WritebackAllowFCLKChangeEndPosition[DML32_NUM_PLANES];
    bool PTEBufferSizeNotExceededPerState[DML32_NUM_PLANES]; // new in DML32
    bool DCCMetaBufferSizeNotExceededPerState[DML32_NUM_PLANES]; // new in DML32
    bool NotEnoughDSCSlices[DML32_NUM_STATES];
    bool PixelsPerLinePerDSCUnitSupport[DML32_NUM_STATES];
    bool DCCMetaBufferSizeNotExceeded[DML32_NUM_STATES][2];
    unsigned int dpte_row_height_linear[DML32_NUM_PLANES];
    unsigned int dpte_row_height_linear_chroma[DML32_NUM_PLANES];
    unsigned int BlockHeightY[DML32_NUM_PLANES];
    unsigned int BlockHeightC[DML32_NUM_PLANES];
    unsigned int BlockWidthY[DML32_NUM_PLANES];
    unsigned int BlockWidthC[DML32_NUM_PLANES];
    unsigned int SubViewportLinesNeededInMALL[DML32_NUM_PLANES];
    bool VActiveBandwithSupport[DML32_NUM_STATES][2];
    bool NotEnoughDETSwathFillLatencyHidingPerState[DML32_NUM_STATES][2];
    struct dummy_vars dummy_vars;
};



inline bool dml_is_nan(float v) { return v != v; }
inline float dcn_bw_mod(const float arg1, const float arg2) { if (dml_is_nan(arg1)) return arg2; if (dml_is_nan(arg2)) return arg1; return arg1 - arg1 * static_cast<int>(arg1 / arg2); }
inline float dcn_bw_min2(const float arg1, const float arg2) { if (dml_is_nan(arg1)) return arg2; if (dml_is_nan(arg2)) return arg1; return arg1 < arg2 ? arg1 : arg2; }
inline float dcn_bw_max2(const float arg1, const float arg2) { if (dml_is_nan(arg1)) return arg2; if (dml_is_nan(arg2)) return arg1; return arg1 > arg2 ? arg1 : arg2; }
inline float dcn_bw_floor2(const float arg, const float significance) { return static_cast<int>(arg / significance) * significance; }
inline float dcn_bw_ceil2(const float arg, const float significance) { return static_cast<int>(arg / significance + 0.99999) * significance; }
inline float dcn_bw_pow(float a, float exp) { float temp; if (static_cast<int>(exp) == 0) return 1; temp = dcn_bw_pow(a, static_cast<int>(exp / 2)); if ((static_cast<int>(exp) % 2) == 0) { return temp * temp; } else { if (static_cast<int>(exp) > 0) return a * temp * temp; else return (temp * temp) / a; } }
inline double dml_min(double a, double b) { return static_cast<double>(dcn_bw_min2(a, b)); }
inline double dml_min3(double a, double b, double c) { return dml_min(dml_min(a, b), c); }
inline double dml_min4(double a, double b, double c, double d) { return dml_min(dml_min(a, b), dml_min(c, d)); }
inline double dml_max(double a, double b) { return static_cast<double>(dcn_bw_max2(a, b)); }
inline double dml_max3(double a, double b, double c) { return dml_max(dml_max(a, b), c); }
inline double dml_max4(double a, double b, double c, double d) { return dml_max(dml_max(a, b), dml_max(c, d)); }
inline double dml_max5(double a, double b, double c, double d, double e) { return dml_max(dml_max4(a, b, c, d), e); }
inline double dml_ceil(double a, double granularity) { return static_cast<double>(dcn_bw_ceil2(a, granularity)); }
inline double dml_floor(double a, double granularity) { return static_cast<double>(dcn_bw_floor2(a, granularity)); }
inline double dml_round(double a) { const double round_pt = 0.5; return dml_floor(a + round_pt, 1); }

inline double dml_pow(double a, int exp) { return static_cast<double>(dcn_bw_pow(a, exp)); }
inline double dml_fmod(double f, int val) { return static_cast<double>(dcn_bw_mod(f, val)); }
inline double dml_ceil_2(double f) { return static_cast<double>(dcn_bw_ceil2(f, 2)); }
inline double dml_ceil_ex(double x, double granularity) { return static_cast<double>(dcn_bw_ceil2(x, granularity)); }
inline double dml_floor_ex(double x, double granularity) { return static_cast<double>(dcn_bw_floor2(x, granularity)); }
inline unsigned int dml_round_to_multiple(unsigned int num, unsigned int multiple, unsigned char up) { unsigned int remainder; if (multiple == 0) return num; remainder = num % multiple; if (remainder == 0) return num; if (up) return (num + multiple - remainder); else return (num - remainder); }
inline double dml_abs(double a) { if (a > 0) return a; else return (a*(-1)); }
// dml_inline_defs.h dml_log2: the IEEE-754 biased exponent field minus 1023 (sign
// ignored), computed without reading the object representation.
inline int dml_log2(double x)
{
    double a = x < 0 ? -x : x;
    if (!(a == a)) return 1024;            // NaN: exponent field all ones
    if (a > 1.7976931348623157e308) return 1024; // infinity
    if (a < 2.2250738585072014e-308) return -1023; // zero and subnormals
    int e = 0;
    while (a >= 2.0) { a *= 0.5; ++e; }
    while (a < 1.0) { a *= 2.0; --e; }
    return e;
}
struct display_mode_lib {
    struct _vcs_dpi_ip_params_st ip;
    struct _vcs_dpi_soc_bounding_box_st soc;
    struct vba_vars_st vba;
    bool validate_max_state;
};
#define __DML_VBA_MIN_VSTARTUP__    9
#define __DML_ARB_TO_RET_DELAY__    7 + 95
#define __DML_MIN_DCFCLK_FACTOR__   1.15
#define __DML_MAX_VRATIO_PRE__ 7.9
#define __DML_MAX_BW_RATIO_PRE__ 4.0
#define __DML_VBA_MAX_DST_Y_PRE__    63.75
#define BPP_INVALID 0
#define BPP_BLENDED_PIPE 0xffffffff
#define MEM_STROBE_FREQ_MHZ 1600
#define DCFCLK_FREQ_EXTRA_PREFETCH_REQ_MHZ 300
#define MEM_STROBE_MAX_DELIVERY_TIME_US 60.0
void dml32_recalculate(struct display_mode_lib *mode_lib);
void dml32_ModeSupportAndSystemConfigurationFull(struct display_mode_lib *mode_lib);

// Linux recalculate_params and dml_get_voltage_level compare the cached soc, ip and
// pipe parameters with memcmp and copy them with assignment/memcpy. The port keeps
// byte comparison and copies the bytes too, so padding is always identical and a
// comparison differs exactly when a value differs.
template <typename T> void dml_copy(T& to, const T& from) {
    unsigned char* x = reinterpret_cast<unsigned char*>(&to);
    const unsigned char* y = reinterpret_cast<const unsigned char*>(&from);
    for (size_t i = 0; i < sizeof(T); ++i)
        x[i] = y[i];
}
template <typename T> bool dml_same(const T& a, const T& b) {
    const unsigned char* x = reinterpret_cast<const unsigned char*>(&a);
    const unsigned char* y = reinterpret_cast<const unsigned char*>(&b);
    for (size_t i = 0; i < sizeof(T); ++i) if (x[i] != y[i]) return false;
    return true;
}
inline bool dml_same_pipes(const display_e2e_pipe_params_st* a, const display_e2e_pipe_params_st* b, unsigned int n) {
    for (unsigned int i = 0; i < n; ++i) if (!dml_same(a[i], b[i])) return false;
    return true;
}
// Linux passes int arrays where unsigned int pointers are declared (C accepts
// it with a warning); the callee stores through the same object representation.
inline unsigned int* dml_uint(int* p) { return reinterpret_cast<unsigned int*>(p); }
// Zero a trivially copyable model object in place (Linux memset/kzalloc), without a
// value-initialized temporary on the kernel stack.
template <typename T> void dml_zero(T& object) {
    unsigned char* bytes = reinterpret_cast<unsigned char*>(&object);
    for (size_t i = 0; i < sizeof(T); ++i)
        bytes[i] = 0;
}

} // namespace rx7900::display::dml32

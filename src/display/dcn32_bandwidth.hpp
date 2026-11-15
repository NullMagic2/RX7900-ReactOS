/*
Module: rx7900.display.dcn32_bandwidth
Purpose: DCN 3.2 bandwidth, clock and arbitration requirements of one owned stream (one or more pipes).
Created: 2026-10-24
Architecture: Portable display core beside src/display/dcn32_stream, which programs
the registers this module computes. A bounded C++ port of the Linux v6.12 DML32
subset that one stream with one unscaled, unrotated, linear ARGB8888 plane exercises
(dml/display_mode_vba.c, dml/dcn32/display_mode_vba_32.c, display_mode_vba_util_32.c,
display_rq_dlg_calc_32.c) plus the dcn32 glue that feeds it: dcn32_init_clocks
(DISPCLK/DPPCLK 1950 MHz caps), dcn32_update_bw_bounding_box_fpu with
build_synthetic_soc_states, dcn32_build_wm_range_table_fpu,
dcn20/dcn32_populate_dml_pipes_from_context, dcn30_fpu_update_soc_for_wm_a,
dcn32_internal_validate_bw, dcn20_validate_apply_pipe_split_flags (MPC_SPLIT_AVOID),
dcn32_calculate_wm_and_dlg_fpu and dcn32_calculate_dlg_params (dcn32_fpu.c,
dcn32_resource.c, dcn32_resource_helpers.c, dcn20_fpu.c, dcn30_fpu.c).
The reference configuration is the one Linux runs when the DMCUB firmware reports
no FAMS capability (mclk_sw = 0) and SubVP and FPO are off (dcn32_hwseq.c sets
force_disable_subvp and disable_fpo_optimizations for DMCUB older than 7.0.35):
no SubVP phantom pipes, no firmware-assisted MCLK switching, no MALL static screen,
no DCC, no MPC split. By default one plane and no ODM combine; the dcn-dsc-odm
opt-ins (Dcn32PlaneRequest allow_odm, DSC fields, a second plane) run the rest
of dcn32_internal_validate_bw: dcn32_apply_merge_split_flags_helper with the
windowed-MPO slice table, should_apply_odm_power_optimization /
try_odm_power_optimization_and_revalidate and the repopulated second pass, with
pipes acquired by src/display/dcn32_topology. DEVIATIONS: ODM is refused for a
stream with two planes (Linux would combine windowed MPO with ODM over four
pipes) and the ODM power policy is not applied to such a stream.
Copyright 2022 Advanced Micro Devices, Inc. (MIT permission notice preserved in the
.cpp files and licenses/AMDGPU-DISPLAY.txt). Pinned sources: work/upstream-dcn-stream.
*/
#pragma once

#include "dcn32_topology.hpp"
#include "modes.hpp"

namespace rx7900::display {

// dc/inc/hw/clk_mgr.h MAX_NUM_DPM_LVL.
constexpr uint32_t dcn32_max_dpm_levels = 8;
// Bound on synthetic SoC states (Linux sizes them DC__VOLTAGE_STATES = 40; the
// synthetic table holds at most 5 STA + 1 + 8 UCLK + 8 FCLK + 1 entries).
constexpr uint32_t dcn32_max_soc_states = 24;

// One SMU13 DPM level in MHz (clk_limit_table_entry). Zero means not reported.
struct Dcn32ClockLevel final {
    uint32_t dcfclk_mhz = 0;
    uint32_t fclk_mhz = 0;
    uint32_t memclk_mhz = 0;
    uint32_t socclk_mhz = 0;
    uint32_t dtbclk_mhz = 0;
    uint32_t dispclk_mhz = 0;
    uint32_t dppclk_mhz = 0;
    uint32_t phyclk_mhz = 0;
};

// clk_bw_params.clk_table and dc_mode_limit as dcn32_init_clocks and
// dcn32_get_memclk_states_from_smu fill them from PMFW (supplied by the link
// owner's SMU13 clock service; never invented here).
struct Dcn32ClockTable final {
    Dcn32ClockLevel levels[dcn32_max_dpm_levels]{};
    uint32_t num_dcfclk_levels = 0;
    uint32_t num_fclk_levels = 0;
    uint32_t num_memclk_levels = 0;
    uint32_t num_socclk_levels = 0;
    uint32_t num_dtbclk_levels = 0;
    uint32_t num_dispclk_levels = 0;
    uint32_t num_dppclk_levels = 0;
    // dcn30_smu_get_dc_mode_max_dpm_freq per clock (phyclk unused).
    Dcn32ClockLevel dc_mode_limit{};
};

// Board facts the bounding box needs besides the clock table.
struct Dcn32SocInputs final {
    Dcn32ClockTable clocks{};
    // dc_bios vram_info (ATOM vram_info): channel count and width; zero keeps
    // the dcn3_2_soc defaults (24 channels, 2 bytes) exactly as Linux does.
    uint32_t num_chans = 0;
    uint32_t dram_channel_width_bytes = 0;
    // dcn32_get_vco_frequency_from_reg, 4300000 when the register reads zero.
    uint32_t dentist_vco_khz = 0;
    // hubbub2_get_dchub_ref_freq (pipes[].clks_cfg.refclk_mhz).
    uint32_t dchub_refclk_khz = 0;
    // bios_parser get_soc_bb_info, 100 ns units; zero means not provided.
    uint32_t dram_clock_change_latency_100ns = 0;
    uint32_t sr_enter_exit_latency_100ns = 0;
    uint32_t sr_exit_latency_100ns = 0;
};

// dc signal of the stream: DisplayPort SST 8b/10b (dm_dp) or TMDS (dm_hdmi, which
// dcn20_populate_dml_pipes_from_context also uses for DVI).
enum class Dcn32OutputSignal : uint32_t {
    display_port,
    tmds
};

// One stream with one plane covering it: viewport = recout = active area at (0, 0).
struct Dcn32PlaneRequest final {
    Timing timing{};
    Dcn32OutputSignal signal = Dcn32OutputSignal::display_port;
    // Surface pitch in pixels (plane_size.surface_pitch), at least timing.width.
    uint32_t pitch_pixels = 0;
    uint32_t otg_instance = 0;
    // dcn-dsc-odm opt-ins; the defaults keep the one-pipe owner unchanged.
    // ODM combine as Linux decides it: a DML pass that needs it, or the
    // single-display 2:1 power policy above DCN3_2_VMIN_DISPCLK_HZ. Without
    // it every multi-pipe result is needs_more_pipes, as before.
    bool allow_odm = false;
    // DSC (stream timing flags.DSC and dsc_cfg): total horizontal slices and
    // bits per pixel in 1/16 units; zero slices means no DSC.
    uint32_t dsc_slices_h = 0;
    uint32_t dsc_bits_per_pixel_x16 = 0;
    // Planes of the stream, plane 0 the top layer (dc_state_add_plane order of
    // amdgpu_dm: descending zpos). 0 means one plane covering the active area
    // with pitch_pixels.
    uint32_t plane_count = 0;
    Dcn32PlaneGeometry planes[dcn32_max_planes]{};
    uint32_t plane_pitch_pixels[dcn32_max_planes]{};
    // dc->current_state's pipes for the dcn32 free-pipe policies (nullptr: all
    // pipes free). Read during validate only.
    const Dcn32Topology* current = nullptr;
};

// dcn_watermarks of one set, in the units hubbub32_program_watermarks consumes.
struct Dcn32Watermarks final {
    uint32_t urgent_ns = 0;
    uint32_t cstate_enter_plus_exit_ns = 0;
    uint32_t cstate_exit_ns = 0;
    uint32_t pstate_change_ns = 0;
    uint32_t pte_meta_urgent_ns = 0;
    uint32_t frac_urg_bw_nom = 0;
    uint32_t frac_urg_bw_flip = 0;
    uint32_t urgent_latency_ns = 0;
    uint32_t fclk_pstate_change_ns = 0;
    uint32_t usr_retraining_ns = 0;
};

// _vcs_dpi_display_dlg_regs_st, field for field (values exactly as DML32 stores
// them; hubp programming masks each into its register field).
struct Dcn32DlgRegs final {
    uint32_t refcyc_h_blank_end = 0;
    uint32_t dlg_vblank_end = 0;
    uint32_t min_dst_y_next_start = 0;
    uint32_t min_dst_y_next_start_us = 0;
    uint32_t refcyc_per_htotal = 0;
    uint32_t refcyc_x_after_scaler = 0;
    uint32_t dst_y_after_scaler = 0;
    uint32_t dst_y_prefetch = 0;
    uint32_t dst_y_per_vm_vblank = 0;
    uint32_t dst_y_per_row_vblank = 0;
    uint32_t dst_y_per_vm_flip = 0;
    uint32_t dst_y_per_row_flip = 0;
    uint32_t ref_freq_to_pix_freq = 0;
    uint32_t vratio_prefetch = 0;
    uint32_t vratio_prefetch_c = 0;
    uint32_t refcyc_per_tdlut_group = 0;
    uint32_t refcyc_per_pte_group_vblank_l = 0;
    uint32_t refcyc_per_pte_group_vblank_c = 0;
    uint32_t refcyc_per_meta_chunk_vblank_l = 0;
    uint32_t refcyc_per_meta_chunk_vblank_c = 0;
    uint32_t refcyc_per_pte_group_flip_l = 0;
    uint32_t refcyc_per_pte_group_flip_c = 0;
    uint32_t refcyc_per_meta_chunk_flip_l = 0;
    uint32_t refcyc_per_meta_chunk_flip_c = 0;
    uint32_t dst_y_per_pte_row_nom_l = 0;
    uint32_t dst_y_per_pte_row_nom_c = 0;
    uint32_t refcyc_per_pte_group_nom_l = 0;
    uint32_t refcyc_per_pte_group_nom_c = 0;
    uint32_t dst_y_per_meta_row_nom_l = 0;
    uint32_t dst_y_per_meta_row_nom_c = 0;
    uint32_t refcyc_per_meta_chunk_nom_l = 0;
    uint32_t refcyc_per_meta_chunk_nom_c = 0;
    uint32_t refcyc_per_line_delivery_pre_l = 0;
    uint32_t refcyc_per_line_delivery_pre_c = 0;
    uint32_t refcyc_per_line_delivery_l = 0;
    uint32_t refcyc_per_line_delivery_c = 0;
    uint32_t chunk_hdl_adjust_cur0 = 0;
    uint32_t chunk_hdl_adjust_cur1 = 0;
    uint32_t vready_after_vcount0 = 0;
    uint32_t dst_y_offset_cur0 = 0;
    uint32_t dst_y_offset_cur1 = 0;
    uint32_t dst_y_delta_drq_limit = 0;
    uint32_t refcyc_per_vm_group_vblank = 0;
    uint32_t refcyc_per_vm_group_flip = 0;
    uint32_t refcyc_per_vm_req_vblank = 0;
    uint32_t refcyc_per_vm_req_flip = 0;
    uint32_t refcyc_per_vm_dmdata = 0;
    uint32_t dmdata_dl_delta = 0;
};

// _vcs_dpi_display_ttu_regs_st.
struct Dcn32TtuRegs final {
    uint32_t qos_level_low_wm = 0;
    uint32_t qos_level_high_wm = 0;
    uint32_t min_ttu_vblank = 0;
    uint32_t qos_level_flip = 0;
    uint32_t refcyc_per_req_delivery_l = 0;
    uint32_t refcyc_per_req_delivery_c = 0;
    uint32_t refcyc_per_req_delivery_cur0 = 0;
    uint32_t refcyc_per_req_delivery_cur1 = 0;
    uint32_t refcyc_per_req_delivery_pre_l = 0;
    uint32_t refcyc_per_req_delivery_pre_c = 0;
    uint32_t refcyc_per_req_delivery_pre_cur0 = 0;
    uint32_t refcyc_per_req_delivery_pre_cur1 = 0;
    uint32_t qos_level_fixed_l = 0;
    uint32_t qos_level_fixed_c = 0;
    uint32_t qos_level_fixed_cur0 = 0;
    uint32_t qos_level_fixed_cur1 = 0;
    uint32_t qos_ramp_disable_l = 0;
    uint32_t qos_ramp_disable_c = 0;
    uint32_t qos_ramp_disable_cur0 = 0;
    uint32_t qos_ramp_disable_cur1 = 0;
};

// _vcs_dpi_display_data_rq_regs_st. Values are unsigned wraparounds of DML's
// int results (for example dml_log2(0) - 6 for an absent PTE group).
struct Dcn32DataRqRegs final {
    uint32_t chunk_size = 0;
    uint32_t min_chunk_size = 0;
    uint32_t meta_chunk_size = 0;
    uint32_t min_meta_chunk_size = 0;
    uint32_t dpte_group_size = 0;
    uint32_t mpte_group_size = 0;
    uint32_t swath_height = 0;
    uint32_t pte_row_height_linear = 0;
};

// _vcs_dpi_display_rq_regs_st.
struct Dcn32RqRegs final {
    Dcn32DataRqRegs rq_regs_l{};
    Dcn32DataRqRegs rq_regs_c{};
    uint32_t drq_expansion_mode = 0;
    uint32_t prq_expansion_mode = 0;
    uint32_t mrq_expansion_mode = 0;
    uint32_t crq_expansion_mode = 0;
    uint32_t plane1_base_address = 0;
};

// bw_ctx.bw.dcn.clk as dcn32_calculate_dlg_params leaves it (dc_clocks subset).
struct Dcn32Clocks final {
    uint32_t dispclk_khz = 0;
    uint32_t dppclk_khz = 0;
    uint32_t dcfclk_khz = 0;
    uint32_t dcfclk_deep_sleep_khz = 0;
    uint32_t socclk_khz = 0;
    uint32_t fclk_khz = 0;
    uint32_t dramclk_khz = 0;
    uint32_t ref_dtbclk_khz = 0;
    uint32_t max_supported_dppclk_khz = 0;
    uint32_t max_supported_dispclk_khz = 0;
    bool p_state_change_support = false;
    bool fclk_p_state_change_support = false;
    bool dtbclk_en = false;
};

// dcn32_calculate_dlg_params output of one pipe (pipe_ctx fields).
struct Dcn32PipeBandwidth final {
    // Hardware pipe (pipe_ctx index); DML pipes are in pipe index order.
    uint32_t pipe = 0;
    uint32_t vstartup_start = 0;
    uint32_t vupdate_offset = 0;
    uint32_t vupdate_width = 0;
    uint32_t vready_offset = 0;
    uint32_t pstate_keepout = 0;
    uint32_t det_buffer_size_kb = 0;
    // plane_res.bw.dppclk_khz.
    uint32_t dppclk_khz = 0;
    uint32_t surface_size_in_mall = 0;
    Dcn32DlgRegs dlg{};
    Dcn32TtuRegs ttu{};
    Dcn32RqRegs rq{};
};

// Everything the stream programmer needs for one validated plane.
struct Dcn32BandwidthPlan final {
    uint32_t voltage_level = 0;
    Dcn32Clocks clocks{};
    // plane_res.bw.dppclk_khz of the DPP (pipes[0].clks_cfg.dppclk_mhz * 1000).
    uint32_t dpp_dppclk_khz = 0;
    Dcn32Watermarks a{};
    Dcn32Watermarks b{};
    Dcn32Watermarks c{};
    Dcn32Watermarks d{};
    // pipe_dlg_param (display_pipe_dest_params_st) globals for OTG/HUBP.
    uint32_t vstartup_start = 0;
    uint32_t vupdate_offset = 0;
    uint32_t vupdate_width = 0;
    uint32_t vready_offset = 0;
    uint32_t pstate_keepout = 0;
    uint32_t det_buffer_size_kb = 0;
    // bw.dcn.compbuf_size_kb (config_return_buffer_size_in_kbytes minus DET).
    uint32_t compbuf_size_kb = 0;
    // bw.dcn.mall_ss_size_bytes: get_surface_size_in_mall of the plane
    // (dcn32_calculate_dlg_params), the input of dcn32_calculate_cab_allocation.
    uint32_t mall_ss_size_bytes = 0;
    bool unbounded_req = false;
    Dcn32DlgRegs dlg{};
    Dcn32TtuRegs ttu{};
    Dcn32RqRegs rq{};
    // dcn-dsc-odm: every pipe of the validated context (the fields above are
    // the OTG master pipe's), the ODM slice count and the pipe topology the
    // validation acquired.
    uint32_t pipe_count = 0;
    uint32_t odm_slices = 0;
    Dcn32PipeBandwidth pipes[dcn32_topology_pipes]{};
    Dcn32Topology topology{};
};

enum class Dcn32BandwidthStatus {
    ready,
    invalid_argument,
    // No usable DCFCLK, DISPCLK or DTBCLK level, or not configured yet.
    clock_table_unavailable,
    // build_synthetic_soc_states produced more than dcn32_max_soc_states states.
    too_many_states,
    // DML32 found no voltage level supporting the plane.
    unsupported_mode,
    // Supported only with MPC split, or ODM combine when allow_odm is off (or
    // with more than one plane, which this owner does not combine with ODM).
    needs_more_pipes
};

struct Dcn32BandwidthWorkspace;

// Device-owned model state (several tens of KiB; the native owner allocates it
// nonpaged once). Not thread-safe: the display owner serializes every call at
// PASSIVE_LEVEL inside a saved floating-point state (KeSaveFloatingPointState),
// the NT counterpart of Linux DC_FP_START. Arithmetic is IEEE double/float exactly
// as DML32 performs it, including its float-typed dml_min/max/floor/ceil helpers.
class Dcn32Bandwidth final {
public:
    Dcn32Bandwidth() noexcept = default;
    Dcn32Bandwidth(const Dcn32Bandwidth&) = delete;
    Dcn32Bandwidth& operator=(const Dcn32Bandwidth&) = delete;

    // dcn32_init_clocks + update_bw_bounding_box + build_wm_range_table. The
    // workspace must outlive this object and is owned by the caller.
    Dcn32BandwidthStatus configure(Dcn32BandwidthWorkspace& workspace,
                                   const Dcn32SocInputs& inputs) noexcept;
    bool configured() const noexcept { return workspace_ != nullptr; }

    // dcn32_validate_bandwidth for one stream and one plane: validation, then
    // watermark sets B, C, A (D = A) and DLG/TTU/RQ for the chosen level. Output is
    // written only on ready.
    Dcn32BandwidthStatus validate(const Dcn32PlaneRequest& request,
                                  Dcn32BandwidthPlan& output) noexcept;

private:
    Dcn32BandwidthWorkspace* workspace_ = nullptr;
};

// Bytes the caller must provide for Dcn32BandwidthWorkspace (sizeof, exposed so a
// native owner can allocate without including the model internals).
uint32_t dcn32_bandwidth_workspace_bytes() noexcept;

// Nonallocating placement construction of a zeroed workspace in caller-owned
// storage of at least dcn32_bandwidth_workspace_bytes() bytes, 8-byte aligned.
// Returns nullptr (storage untouched) for a null, short or misaligned buffer. The
// workspace is trivially destructible; the caller frees the storage after the
// last Dcn32Bandwidth using it.
Dcn32BandwidthWorkspace* construct_dcn32_bandwidth_workspace(void* storage, uint32_t bytes) noexcept;

} // namespace rx7900::display

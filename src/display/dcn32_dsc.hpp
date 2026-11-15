/*
Module: rx7900.display.dcn32_dsc
Purpose: VESA DSC 1.2a configuration for DCN 3.2: sink capabilities, the DC slice/bpp policy, rate control, PPS and DSC engine registers.
Created: 2026-10-24
Architecture: Portable display core used by src/display/dcn32_stream (which
programs the DSC engines, OPTC and stream encoder from a stream plan) and by the
native mode service (which reads the sink DPCD and decides the link). Every
computation is a C++ transcription of Linux v6.12: dc/dsc/dc_dsc.c
(dc_dsc_parse_dsc_dpcd, dc_dsc_compute_config/setup_dsc_config with the default
amdgpu_dm SST options, dc_dsc_stream_bandwidth_in_kbps and its overhead),
dc/dsc/dcn20/dcn20_dsc.c (dsc2_get_enc_caps, dsc_prepare_config,
dsc_write_to_registers, dsc2_enable/disable/disconnect/read_state),
dc/dsc/rc_calc.c, dc/dml/dsc/rc_calc_fpu.c with qp_tables.h (444 8 bpc rows),
dc/dsc/rc_calc_dpi.c (dscc_compute_dsc_parameters), drm/display/drm_dsc_helper.c
(drm_dsc_compute_rc_parameters, drm_dsc_pps_payload_pack), dc/basics/fixpt31_32.c
(the fixed31_32 arithmetic the policy uses) and link/link_validation.c
(dp_link_bandwidth_kbps). The rate-control arithmetic Linux performs in IEEE
float/double is evaluated exactly in integers here (including the round-half-up
of products with 8/3.0, 4/3.0 and 2/3.0 at their binary64 values), so results do
not depend on the target's floating-point unit and no FPU state is needed. The
scope is the stream this driver emits: RGB 8 bpc progressive DisplayPort SST
8b/10b; YCbCr, 10/12 bpc, eDP, MST, branch devices and HDMI FRL DSC are refused.
Linux AMD and DRM notices and provenance: licenses/AMDGPU-DISPLAY.txt.
*/
#pragma once

#include "dcn32_stream_encoder.hpp"

namespace rx7900::display {

constexpr uint32_t dsc_pps_bytes = 128;
constexpr uint32_t dsc_buf_ranges = 15;   // DSC_NUM_BUF_RANGES
constexpr uint32_t dsc_dpcd_caps_bytes = 16;   // DP_DSC_SUPPORT .. DP_DSC_BITS_PER_PIXEL_INC
constexpr uint32_t dsc_branch_caps_bytes = 3;  // DP_DSC_BRANCH_OVERALL_THROUGHPUT_0 .. MAX_LINE_WIDTH
constexpr uint32_t dpcd_dsc_support = 0x060;
constexpr uint32_t dpcd_dsc_branch_caps = 0x0a0;
constexpr uint32_t dpcd_fec_capability = 0x090;
constexpr uint32_t dpcd_fec_configuration = 0x120;
constexpr uint32_t dpcd_dsc_enable = 0x160;
constexpr uint8_t dpcd_fec_capable = 0x01;     // DP_FEC_CAPABLE

// dsc_enc_slice_caps bit positions (dc/inc/hw/dsc.h, not the DPCD layout).
constexpr uint8_t dsc_slices_1 = 1u << 0;
constexpr uint8_t dsc_slices_2 = 1u << 1;
constexpr uint8_t dsc_slices_3 = 1u << 2;
constexpr uint8_t dsc_slices_4 = 1u << 3;
constexpr uint8_t dsc_slices_8 = 1u << 4;
constexpr uint8_t dsc_slices_12 = 1u << 5;
constexpr uint8_t dsc_slices_16 = 1u << 6;

// struct dsc_dec_dpcd_caps, decoded from DPCD 0x060..0x06f (+ 0x0a0..0x0a2).
struct DscSinkCaps final {
    bool is_dsc_supported = false;
    uint8_t dsc_version = 0;
    int32_t rc_buffer_size = 0;
    uint8_t slice_caps1 = 0;     // DPCD layout: 1, 2, -, 4, 6, 8, 10, 12
    uint8_t slice_caps2 = 0;     // DPCD layout: 16, 20, 24
    int32_t lb_bit_depth = 0;
    bool is_block_pred_supported = false;
    int32_t edp_max_bits_per_pixel = 0;
    uint8_t color_formats = 0;   // RGB, 444, simple 422, native 422, native 420
    uint8_t color_depth = 0;     // bit 1: 8 bpc, bit 2: 10 bpc, bit 3: 12 bpc
    int32_t throughput_mode_0_mps = 0;
    int32_t throughput_mode_1_mps = 0;
    int32_t max_slice_width = 0;
    uint32_t bpp_increment_div = 0;
    uint32_t branch_overall_throughput_0_mps = 0;
    uint32_t branch_overall_throughput_1_mps = 0;
    uint32_t branch_max_line_width = 0;
    bool is_dp = false;
};

// dc_dsc_parse_dsc_dpcd. branch may be nullptr (no extended caps, is_dp stays
// false); amdgpu_dm always passes the (zeroed for a non-branch sink) branch
// bytes, which sets is_dp. Returns false (caps unusable) on an unsupported or
// malformed field; caps is fully rewritten either way.
bool parse_dsc_dpcd(const uint8_t (&basic)[dsc_dpcd_caps_bytes], const uint8_t* branch,
                    DscSinkCaps& caps) noexcept;

// struct dsc_enc_caps.
struct DscEncoderCaps final {
    uint8_t dsc_version = 0;
    uint8_t slice_caps = 0;      // dsc_slices_* bits
    int32_t lb_bit_depth = 0;
    bool is_block_pred_supported = false;
    uint8_t color_formats = 0;
    uint8_t color_depth = 0;
    int32_t max_total_throughput_mps = 0;
    int32_t max_slice_width = 0;
    uint32_t bpp_increment_div = 0;
    uint32_t edp_sink_max_bits_per_pixel = 0;
    bool is_dp = false;
};

// dsc2_get_enc_caps for one DCN 2.x/3.2 DSC engine.
void dsc_encoder_caps(uint32_t pixel_clock_100hz, DscEncoderCaps& caps) noexcept;

// The stream timing facts the policy reads (dc_crtc_timing of a borderless
// progressive RGB 8 bpc stream).
struct DscTiming final {
    uint32_t pixel_clock_100hz = 0;
    uint32_t h_active = 0;
    uint32_t v_active = 0;
    uint32_t h_total = 0;
    uint32_t v_total = 0;
};

// struct dc_dsc_config (rc_params_ovrd is never used: nullptr in Linux SST).
struct DscConfig final {
    uint32_t num_slices_h = 0;
    uint32_t num_slices_v = 0;
    uint32_t bits_per_pixel = 0;   // 1/16 bpp
    bool block_pred_enable = false;
    uint32_t linebuf_depth = 0;
    uint32_t version_minor = 0;
    bool is_dp = false;
};

// dc_dsc_compute_config(dscs[0], caps, default options, target, timing,
// DC_LINK_ENCODING_DP_8b_10b) for PIXEL_ENCODING_RGB / COLOR_DEPTH_888 with
// max_target_bpp_limit_override 0 (amdgpu_dm apply_dsc_policy_for_stream, no
// dongle). target_bandwidth_kbps is the link bandwidth; true when DSC is
// needed and possible (config written), false otherwise (config zeroed).
bool dsc_compute_config(const DscSinkCaps& sink, uint32_t target_bandwidth_kbps, const DscTiming& timing,
                        DscConfig& config) noexcept;

// dc_dsc_stream_bandwidth_in_kbps (overhead included for is_dp).
uint32_t dsc_stream_bandwidth_kbps(const DscTiming& timing, uint32_t bpp_x16, uint32_t num_slices_h,
                                   bool is_dp) noexcept;
// dc_bandwidth_in_kbps_from_timing of an uncompressed RGB 8 bpc stream.
uint32_t uncompressed_bandwidth_kbps(const DscTiming& timing) noexcept;
// dp_link_bandwidth_kbps for 8b/10b; fec applies the 97 % FEC efficiency.
uint32_t dp_link_bandwidth_kbps(uint8_t rate_code, uint8_t lanes, bool fec) noexcept;

// struct drm_dsc_rc_range_parameters / drm_dsc_config (the fields DC uses).
struct DscRcRange final {
    uint8_t range_min_qp = 0;
    uint8_t range_max_qp = 0;
    uint8_t range_bpg_offset = 0;
};

struct DscPps final {
    uint8_t line_buf_depth = 0;
    uint8_t bits_per_component = 0;
    bool convert_rgb = false;
    uint16_t slice_width = 0;
    uint16_t slice_height = 0;
    bool simple_422 = false;
    uint16_t pic_width = 0;
    uint16_t pic_height = 0;
    uint8_t rc_tgt_offset_high = 0;
    uint8_t rc_tgt_offset_low = 0;
    uint16_t bits_per_pixel = 0;
    uint8_t rc_edge_factor = 0;
    uint8_t rc_quant_incr_limit1 = 0;
    uint8_t rc_quant_incr_limit0 = 0;
    uint16_t initial_xmit_delay = 0;
    uint16_t initial_dec_delay = 0;
    bool block_pred_enable = false;
    uint8_t first_line_bpg_offset = 0;
    uint16_t initial_offset = 0;
    uint16_t rc_buf_thresh[dsc_buf_ranges - 1]{};
    DscRcRange rc_range_params[dsc_buf_ranges]{};
    uint16_t rc_model_size = 0;
    uint8_t flatness_min_qp = 0;
    uint8_t flatness_max_qp = 0;
    uint8_t initial_scale_value = 0;
    uint16_t scale_decrement_interval = 0;
    uint16_t scale_increment_interval = 0;
    uint16_t nfl_bpg_offset = 0;
    uint16_t slice_bpg_offset = 0;
    uint16_t final_offset = 0;
    bool vbr_enable = false;
    uint16_t slice_chunk_size = 0;
    uint8_t dsc_version_minor = 0;
    uint8_t dsc_version_major = 0;
    bool native_422 = false;
    bool native_420 = false;
    uint8_t second_line_bpg_offset = 0;
    uint16_t nsl_bpg_offset = 0;
    uint16_t second_line_offset_adj = 0;
};

// struct dsc_reg_values (non-PPS fields at their dsc_init_reg_values values
// except those dsc_prepare_config sets).
struct DscRegisterValues final {
    DscPps pps{};
    uint32_t pixel_format = 0;          // DSC_PIXFMT_RGB = 0
    uint32_t num_slices_h = 0;
    uint32_t num_slices_v = 0;
    uint32_t bpp_x32 = 0;
    uint32_t ich_reset_at_eol = 0;
    uint32_t alternate_ich_encoding_en = 0;
    uint32_t rc_buffer_model_size = 0;
};

// struct dsc_optc_config.
struct DscOptcConfig final {
    uint32_t bytes_per_pixel = 0;   // u3.28
    uint32_t slice_width = 0;
    bool is_pixel_format_444 = false;
};

// dsc_prepare_config for one DSC engine (struct dsc_config: pic_width of the
// engine's ODM slice, slices per engine, is_odm). Fails (outputs untouched) on
// the validation dsc_prepare_config performs or an rc parameter range error.
bool dsc_prepare_config(uint32_t pic_width, uint32_t pic_height, bool is_odm, const DscConfig& config,
                        DscRegisterValues& registers, DscOptcConfig& optc) noexcept;

// drm_dsc_pps_payload_pack: the 128-byte VESA DSC 1.2a PPS, big-endian fields.
void dsc_pack_pps(const DscPps& pps, uint8_t (&payload)[dsc_pps_bytes]) noexcept;

// The DSC engine registers (dcn20_dsc.c with the DCN 3.2 register and mask
// lists of dcn32_resource.c). One engine per OPP: instance == pipe.
namespace dcn32_dsc {

constexpr uint32_t engine_count = 4;

// dsc2_read_state (every REG_GET it performs); forward_enabled is dsc_fw_en.
bool read_state(Dcn32StreamIo& io, uint32_t dsc, bool& forward_enabled) noexcept;
// dsc_write_to_registers.
bool write_registers(Dcn32StreamIo& io, uint32_t dsc, const DscRegisterValues& values) noexcept;
// dsc2_enable / dsc2_disable / dsc2_disconnect.
bool enable(Dcn32StreamIo& io, uint32_t dsc, uint32_t opp) noexcept;
bool disable(Dcn32StreamIo& io, uint32_t dsc) noexcept;
bool disconnect(Dcn32StreamIo& io, uint32_t dsc) noexcept;
// dsc2_wait_disconnect_pending_clear: DSCRM_DSC_DOUBLE_BUFFER_REG_UPDATE_PENDING
// is absent from the DCN 2.0 mask list DCN 3.2 uses (mask 0), so the REG_WAIT
// reads once and is satisfied.
bool wait_disconnect_pending_clear(Dcn32StreamIo& io, uint32_t dsc) noexcept;

} // namespace dcn32_dsc

} // namespace rx7900::display

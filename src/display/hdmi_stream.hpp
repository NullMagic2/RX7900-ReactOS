/*
Module: rx7900.display.hdmi_stream
Purpose: Stream-side HDMI/DVI/DP signalling facts: sink HDMI capability, VIC matching, TMDS limits and info packets.
Created: 2026-10-24
Architecture: Portable, allocation-free policy beside src/display/dcn32_stream,
which writes the packets this module builds into the DIG FE/VPG registers. Sink
parsing follows Linux v6.12 drm_edid.c (drm_parse_cea_ext CTA data block walk,
drm_parse_hdmi_vsdb_video, drm_parse_hdmi_forum_scds, drm_match_cea_mode,
drm_match_hdmi_mode, cea_mode_alternate_clock, fixup_detailed_cea_mode_clock,
edid_cea_modes_1/_193 and edid_4k_modes data). Packet layouts follow DC
core/dc_resource.c set_avi_info_frame (union hdmi_info_packet of
include/set_mode_types.h, enum values of dc/inc/hw/transform.h),
modules/info_packet/info_packet.c mod_build_hf_vsif_infopacket and
mod_build_vsc_infopacket, and the amdgpu_dm v6.12 stream defaults (RGB 8 bpc,
COLOR_SPACE_SRGB, content type "no data", qs_bit 0). HDMI-versus-DVI selection
follows link_detection.c (no HDMI VSDB on an HDMI connector -> DVI single link);
TMDS limits follow dcn10_link_encoder_validate_output_with_stream with the
dcn32 link_enc_feature (max_hdmi_pixel_clock 600 MHz) and TMDS_MAX_PIXEL_CLOCK
165 MHz for single-link DVI. Copyright notices and pinned sources:
licenses/AMDGPU-DISPLAY.txt, THIRD_PARTY.txt and work/upstream-dcn-stream.
*/
#pragma once

#include "byte_view.hpp"
#include "modes.hpp"

namespace rx7900::display {

// dc_info_packet (dc/dc_types.h): header bytes and 32 payload bytes.
struct InfoPacket final {
    bool valid = false;
    uint8_t hb0 = 0;
    uint8_t hb1 = 0;
    uint8_t hb2 = 0;
    uint8_t hb3 = 0;
    uint8_t sb[32]{};
};

// drm_display_info facts the stream side consumes, parsed from CTA extensions.
struct HdmiSinkInfo final {
    // HDMI 1.x VSDB (OUI 00-0C-03) present: drm is_hdmi / dc edid_hdmi.
    bool is_hdmi = false;
    // HDMI_Video_present in the VSDB, or any HF-VSDB/HF-SCDB.
    bool has_hdmi_infoframe = false;
    bool dvi_dual = false;
    // drm max_tmds_clock in kHz: VSDB byte 7 * 5000, raised by HF max TMDS
    // character rate when above 340 MHz; 0 when not declared.
    uint32_t max_tmds_clock_khz = 0;
    bool scdc_supported = false;
    bool scrambling_supported = false;
    bool scrambling_low_rates = false;
    // DRM_COLOR_FORMAT_YCBCR420 from a Y420VDB VIC or a non-empty Y420CMDB map.
    bool ycbcr420 = false;
    uint8_t cea_revision = 0;
    // Valid CEA VICs listed in CTA Video Data Blocks (drm info->vics).
    uint32_t vdb_vics[8]{};
};

// is_hdmi2_sink: SCDC support or YCbCr 4:2:0 capability.
bool hdmi2_sink(const HdmiSinkInfo& sink) noexcept;

enum class HdmiStatus {
    success,
    invalid_argument,
    truncated_edid,
    // A TMDS stream cannot carry the timing on this connector or sink.
    exceeds_tmds_limit,
    // > 340 MHz without HF scrambling + SCDC on the sink.
    needs_scrambling,
    unsupported_timing
};

// Walks every 128-byte CTA (tag 0x02) extension of a complete EDID blob in the
// drm_parse_cea_ext order. Checksums are the caller's (sink.cpp parse_edid)
// responsibility. A data block whose length runs past the DTD offset ends the
// walk of that extension (__cea_db_iter_current_block), as in Linux; only a
// blob that is not a whole number of 128-byte blocks is invalid_argument.
HdmiStatus parse_hdmi_sink(ByteView edid, HdmiSinkInfo& output) noexcept;

// Short video descriptors of every CTA Video Data Block (VDB order, the
// native flag of SVDs 129..192 removed) and the HDMI_VIC list of the HDMI 1.4b
// VSDB (CTA-861-G 7.5.1, HDMI 1.4b 8.3.2). YCbCr 4:2:0-only VICs (Y420VDB)
// are not listed: the scanout is RGB.
constexpr uint32_t cta_video_capacity = 64;
constexpr uint32_t cta_hdmi_vic_capacity = 8;
struct CtaVideoList final {
    uint8_t vics[cta_video_capacity]{};
    uint32_t vic_count = 0;
    uint8_t hdmi_vics[cta_hdmi_vic_capacity]{};
    uint32_t hdmi_vic_count = 0;
    // Reserved/unknown codes and entries beyond either capacity.
    uint32_t dropped = 0;
};

// Same walk and blob rules as parse_hdmi_sink.
HdmiStatus parse_cta_video(ByteView edid, CtaVideoList& output) noexcept;

// The CEA/CTA-861 timing of a VIC (edid_cea_modes data above) and of an HDMI
// 1.4b HDMI_VIC 1..4 (edid_4k_modes). false for unknown, interlaced and
// pixel-repeated (DBLCLK) VICs; output is unchanged then.
bool cea_vic_timing(uint8_t vic, Timing& output) noexcept;
bool hdmi_vic_timing(uint8_t hdmi_vic, Timing& output) noexcept;

// drm_mode_vrefresh for a progressive, non-doublescan timing (Hz, rounded).
uint32_t drm_vrefresh_hz(const Timing& timing) noexcept;

// drm_match_cea_mode for a mode without picture aspect ratio (EDID DTD modes):
// first VIC in table order whose timings and sync flags match and whose clock
// equals the CEA or alternate (59.94/60) clock in KHZ2PICOS units. 0 = none.
uint8_t match_cea_vic(const Timing& timing) noexcept;
// drm_match_hdmi_mode (HDMI 1.4 4K VICs 1..4). 0 = none.
uint8_t match_hdmi_vic(const Timing& timing) noexcept;

// fixup_detailed_cea_mode_clock: a DTD within 5 kHz of a CEA (else HDMI 4K)
// mode takes whichever of its clocks is closer. Returns the clock to use.
uint32_t fixup_detailed_cea_clock(const Timing& timing) noexcept;

enum class TmdsSignal : uint32_t {
    hdmi,
    dvi_single_link
};

// What dcn-stream programs for a TMDS stream and what it asks the link owner
// for (enable_tmds_link kind, character rate and lte_340 scrambling).
struct TmdsStreamPlan final {
    TmdsSignal signal = TmdsSignal::dvi_single_link;
    // TMDS character rate = pixel clock at 8 bpc RGB (no deep colour).
    uint32_t tmds_khz = 0;
    // HDMI_DATA_SCRAMBLE_EN/HDMI_CLOCK_CHANNEL_RATE (enc32 hdmi attribute).
    bool scramble_above_340 = false;
    // dc_crtc_timing flags.LTE_340MCSC_SCRAMBLE: drm sets it only for 4:2:0
    // forced scrambling; always false for RGB.
    bool lte_340_scramble = false;
    uint8_t vic = 0;
    uint8_t hdmi_vic = 0;
    InfoPacket avi{};
    InfoPacket vsif{};
};

// connector_hdmi: the route is an HDMI connector (DVI routes are not bound by
// this owner). The pixel clock in timing must already be the clock to program.
HdmiStatus plan_tmds_stream(const Timing& timing, const HdmiSinkInfo& sink,
                            TmdsStreamPlan& output) noexcept;

// set_avi_info_frame for RGB, COLOR_SPACE_SRGB, no borders, content type "no
// data", qs_bit 0, aspect ratio "no data" (DTD modes), hdmi_vic overriding vic.
void build_avi_infoframe(const Timing& timing, uint8_t vic, uint8_t hdmi_vic,
                         InfoPacket& output) noexcept;
// mod_build_hf_vsif_infopacket for a 2D stream (HDMI VIC 4K modes only).
void build_hdmi_vsif(const Timing& timing, uint8_t hdmi_vic, InfoPacket& output) noexcept;
// mod_build_vsc_infopacket for SST without PSR/replay: revision 5 colorimetry
// SDP when use_vsc_sdp_for_colorimetry (DPCD 1.4 and DPRX VSC_SDP_COLORIMETRY),
// otherwise invalid.
void build_dp_vsc(bool use_vsc_sdp_for_colorimetry, InfoPacket& output) noexcept;

} // namespace rx7900::display

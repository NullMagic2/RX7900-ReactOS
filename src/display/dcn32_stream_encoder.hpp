/*
Module: rx7900.display.dcn32_stream_encoder
Purpose: DCN 3.2 register access for the stream side and the DIO stream encoder (DIG FE, DP/HDMI, VPG, AFMT, DME).
Created: 2026-10-24
Architecture: Dcn32StreamIo is the bounded REG_SET/REG_UPDATE/REG_WAIT layer
of src/display/dcn32_stream over the shared hardware::RegisterIo and the
discovery-validated DMU 3.2.0 segments; every access names a register of the
header-validated catalog dcn32_stream_registers.hpp. The encoder functions are
one-to-one ports of the Linux v6.12 DCN 3.2 stream encoder
(dcn32_dio_stream_encoder.c, dcn30_dio_stream_encoder.c, dcn20_stream_encoder.c,
dcn10_stream_encoder.c), dcn30_vpg.c and dcn30_afmt.c for one DIO engine with
RGB 8 bpc, no audio, no DSC and the avoid_vbios_exec_table TMDS attribute path.
The DIG back end, PHY and DIG FE-to-BE connection belong to output::DisplayLinkOwner.
Linux AMD notice and provenance: licenses/AMDGPU-DISPLAY.txt.
*/
#pragma once

#include "dcn32_stream_registers.hpp"
#include "hdmi_stream.hpp"
#include "register_io.hpp"

namespace rx7900::display {

struct FieldValue final {
    dcn32_stream_reg::Field field;
    uint32_t value;
};

// Fault latch over one RegisterIo. A refused read/write (unmapped offset, an
// unconfirmed posted store) latches faulted(): every later call fails without
// touching hardware, so a sequence can never continue on unproven state. A
// REG_WAIT that never sees its value latches timed_out() (Linux only logs it;
// this owner stops the sequence and runs its recovery instead). PASSIVE_LEVEL,
// serialized by the display owner.
class Dcn32StreamIo final {
public:
    Dcn32StreamIo() noexcept = default;
    Dcn32StreamIo(const Dcn32StreamIo&) = delete;
    Dcn32StreamIo& operator=(const Dcn32StreamIo&) = delete;

    // Segment dword bases by BASE_IDX (1 = DCCG, 2 = DCN, 3 = MPC); 0 = unused.
    bool bind(const hardware::RegisterIo& io, uint32_t segment1, uint32_t segment2,
              uint32_t segment3) noexcept;
    void unbind() noexcept;
    bool bound() const noexcept { return bound_; }

    bool read(const dcn32_stream_reg::Register& reg, uint32_t instance, uint32_t& value) noexcept;
    // REG_WRITE.
    bool write(const dcn32_stream_reg::Register& reg, uint32_t instance, uint32_t value) noexcept;
    // REG_SET_N with initial value 0: unnamed fields are written as zero.
    template <size_t N>
    bool set(const dcn32_stream_reg::Register& reg, uint32_t instance, const FieldValue (&fields)[N]) noexcept {
        return set_fields(reg, instance, fields, N);
    }
    // REG_UPDATE_N: read, replace the named fields, write (always written).
    template <size_t N>
    bool update(const dcn32_stream_reg::Register& reg, uint32_t instance, const FieldValue (&fields)[N]) noexcept {
        return update_fields(reg, instance, fields, N);
    }
    bool get(const dcn32_stream_reg::Register& reg, uint32_t instance, dcn32_stream_reg::Field field,
             uint32_t& value) noexcept;
    // generic_reg_wait: up to tries + 1 reads, delay_us between them.
    bool wait(const dcn32_stream_reg::Register& reg, uint32_t instance, dcn32_stream_reg::Field field,
              uint32_t value, uint32_t delay_us, uint32_t tries) noexcept;
    // udelay/msleep.
    void delay_us(uint32_t microseconds) noexcept;

    bool faulted() const noexcept { return faulted_; }
    bool timed_out() const noexcept { return timed_out_; }
    // A REG_WAIT timeout leaves the state known (every write landed); the
    // controller clears it before its recovery path. A fault is only cleared
    // by an owner-proven reset (reset_proven).
    void clear_timeout() noexcept { timed_out_ = false; }
    void reset_proven() noexcept {
        faulted_ = false;
        timed_out_ = false;
    }
    uint32_t write_count() const noexcept { return writes_; }

private:
    bool offset_of(const dcn32_stream_reg::Register& reg, uint32_t instance, uint32_t& offset) noexcept;
    bool set_fields(const dcn32_stream_reg::Register& reg, uint32_t instance, const FieldValue* fields,
                    size_t count) noexcept;
    bool update_fields(const dcn32_stream_reg::Register& reg, uint32_t instance, const FieldValue* fields,
                       size_t count) noexcept;

    hardware::RegisterIo io_{};
    uint32_t segment_[4]{};
    uint32_t writes_ = 0;
    bool bound_ = false;
    bool faulted_ = false;
    bool timed_out_ = false;
};

// set_reg_field_value_ex: (value << shift) & mask, other bits kept.
constexpr uint32_t place_field(uint32_t word, dcn32_stream_reg::Field field, uint32_t value) noexcept {
    return (word & ~field.mask) | ((value << field.shift) & field.mask);
}
constexpr uint32_t field_of(uint32_t word, dcn32_stream_reg::Field field) noexcept {
    return (word & field.mask) >> field.shift;
}

// The stream timing in the dc_crtc_timing terms the encoder uses: progressive,
// borderless RGB 8 bpc (COLOR_DEPTH_888, PIXEL_ENCODING_RGB, COLOR_SPACE_SRGB).
namespace dcn32_encoder {

constexpr uint32_t engine_count = 5;     // DIG A..E (ENGINE_ID_DIGA..DIGE)
constexpr uint32_t vpg_packets = 15;     // VPG GSP 0..14

// enc1_dig_connect_to_otg.
bool dig_connect_to_otg(Dcn32StreamIo& io, uint32_t engine, uint32_t otg) noexcept;
// enc1_setup_stereo_sync (stereo off: STEREOSYNC_GATE_EN = 1).
bool setup_stereo_sync(Dcn32StreamIo& io, uint32_t engine, uint32_t otg, bool enable) noexcept;
// enc2_stream_encoder_dp_set_stream_attribute (SDP splitting off).
bool dp_set_stream_attribute(Dcn32StreamIo& io, uint32_t engine, const Timing& timing,
                             bool use_vsc_sdp_for_colorimetry) noexcept;
// enc32_stream_encoder_hdmi_set_stream_attribute, avoid_vbios_exec_table path
// (DIG_CLOCK_PATTERN 0x1F), no audio.
bool hdmi_set_stream_attribute(Dcn32StreamIo& io, uint32_t engine, uint32_t actual_pix_clk_khz) noexcept;
// enc32_stream_encoder_dvi_set_stream_attribute, avoid_vbios_exec_table path.
bool dvi_set_stream_attribute(Dcn32StreamIo& io, uint32_t engine) noexcept;
// enc3_stream_encoder_update_hdmi_info_packets with AVI (0) and vendor (1);
// gamut, SPD, HDR SMD, HF-VSIF and VTEM are invalid (their slots are stopped).
bool update_hdmi_info_packets(Dcn32StreamIo& io, uint32_t engine, const InfoPacket& avi,
                              const InfoPacket& vendor) noexcept;
bool stop_hdmi_info_packets(Dcn32StreamIo& io, uint32_t engine) noexcept;
bool hdmi_reset_stream_attribute(Dcn32StreamIo& io, uint32_t engine) noexcept;
// enc3_stream_encoder_update_dp_info_packets with VSC only (no SPD, HDR SMD,
// adaptive sync); update_dp_info_packets_sdp_line_num writes nothing for it.
bool update_dp_info_packets(Dcn32StreamIo& io, uint32_t engine, const InfoPacket& vsc) noexcept;
bool stop_dp_info_packets(Dcn32StreamIo& io, uint32_t engine) noexcept;
// enc1_stream_encoder_dp_blank (the caller then sleeps 60 ms, dce110_blank_stream).
bool dp_blank(Dcn32StreamIo& io, uint32_t engine) noexcept;
// enc32_dp_set_odm_combine then enc32_stream_encoder_dp_unblank (dcn32_unblank_stream).
bool dp_unblank(Dcn32StreamIo& io, uint32_t engine, uint32_t pixel_clock_khz, uint32_t link_rate_code,
                uint32_t pix_per_cycle) noexcept;
// enc1_stream_encoder_set_avmute.
bool set_avmute(Dcn32StreamIo& io, uint32_t engine, bool enable) noexcept;
// enc32_set_dig_input_mode.
bool set_input_mode(Dcn32StreamIo& io, uint32_t engine, uint32_t pix_per_container) noexcept;
// enc32_enable_fifo (enc32_reset_fifo true then false).
bool enable_fifo(Dcn32StreamIo& io, uint32_t engine) noexcept;
// enc2_set_dynamic_metadata(enable_dme = false).
bool disable_dynamic_metadata(Dcn32StreamIo& io, uint32_t engine, bool display_port) noexcept;
// disable_dio_audio_packet without an audio endpoint: enc3_audio_mute_control
// -> afmt3_audio_mute_control(true).
bool audio_mute(Dcn32StreamIo& io, uint32_t engine) noexcept;
// vpg3_update_generic_info_packet.
bool vpg_update_generic_info_packet(Dcn32StreamIo& io, uint32_t vpg, uint32_t packet_index,
                                    const InfoPacket& packet, bool immediate_update) noexcept;
// dcn-dsc-odm. enc32_dp_set_dsc_config (DCN 3.2 keeps only DP_DSC_MODE).
bool dp_set_dsc_config(Dcn32StreamIo& io, uint32_t engine, bool enable) noexcept;
// enc3_dp_set_dsc_pps_info_packet: the 128-byte PPS in GSP11..14 (SDP type
// DC_DP_INFOFRAME_TYPE_PPS, hb2 127), line 2, VBID[6] at line 3; disable
// turns GSP11 and its PPS size off.
bool dp_set_dsc_pps_info_packet(Dcn32StreamIo& io, uint32_t engine, bool enable, const uint8_t* pps,
                                bool immediate_update) noexcept;

} // namespace dcn32_encoder

} // namespace rx7900::display

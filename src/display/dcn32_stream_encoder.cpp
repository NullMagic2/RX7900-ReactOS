/*
Module: rx7900.display.dcn32_stream_encoder
Purpose: Bounded register access and the DCN 3.2 DIO stream encoder sequences.
Created: 2026-10-24
Architecture: Each function mirrors one Linux v6.12 function (named in its
comment) statement for statement for RGB 8 bpc, COLOR_SPACE_SRGB, no audio, no
DSC, no PSR and no dynamic metadata. REG_GET/REG_READ reads are kept where Linux
reads, so a register trace is comparable access for access. Adapted from AMD DC
dcn10/20/30/32 stream encoder, dcn30_vpg.c and dcn30_afmt.c (Copyright 2012-2022
Advanced Micro Devices, Inc., MIT notice in licenses/AMDGPU-DISPLAY.txt).
*/
#include "dcn32_stream_encoder.hpp"

namespace rx7900::display {

namespace r = dcn32_stream_reg;

bool Dcn32StreamIo::bind(const hardware::RegisterIo& io, uint32_t segment1, uint32_t segment2,
                         uint32_t segment3) noexcept {
    if (!hardware::register_io_valid(io) || segment2 == 0 || segment3 == 0) {
        return false;
    }
    io_ = io;
    segment_[0] = 0;
    segment_[1] = segment1;
    segment_[2] = segment2;
    segment_[3] = segment3;
    writes_ = 0;
    bound_ = true;
    faulted_ = false;
    timed_out_ = false;
    return true;
}

void Dcn32StreamIo::unbind() noexcept {
    io_ = {};
    for (uint32_t& segment : segment_) {
        segment = 0;
    }
    bound_ = false;
}

bool Dcn32StreamIo::offset_of(const r::Register& reg, uint32_t instance, uint32_t& offset) noexcept {
    if (!bound_ || faulted_ || timed_out_) {
        return false;
    }
    if (instance >= reg.count || reg.base_index >= 4 || segment_[reg.base_index] == 0) {
        // A catalog or plan mismatch is a programming error, never a guess.
        faulted_ = true;
        return false;
    }
    const uint64_t bytes = (static_cast<uint64_t>(segment_[reg.base_index]) + reg.offset[instance]) * 4u;
    if (bytes + 4u > io_.aperture_bytes) {
        faulted_ = true;
        return false;
    }
    offset = static_cast<uint32_t>(bytes);
    return true;
}

bool Dcn32StreamIo::read(const r::Register& reg, uint32_t instance, uint32_t& value) noexcept {
    uint32_t offset = 0;
    if (!offset_of(reg, instance, offset)) {
        return false;
    }
    if (!io_.read32(io_.context, offset, value)) {
        faulted_ = true;
        return false;
    }
    return true;
}

bool Dcn32StreamIo::write(const r::Register& reg, uint32_t instance, uint32_t value) noexcept {
    uint32_t offset = 0;
    if (!offset_of(reg, instance, offset)) {
        return false;
    }
    if (!io_.write32(io_.context, offset, value)) {
        faulted_ = true;
        return false;
    }
    ++writes_;
    return true;
}

bool Dcn32StreamIo::set_fields(const r::Register& reg, uint32_t instance, const FieldValue* fields,
                               size_t count) noexcept {
    uint32_t value = 0;
    for (size_t index = 0; index < count; ++index) {
        value = place_field(value, fields[index].field, fields[index].value);
    }
    return write(reg, instance, value);
}

bool Dcn32StreamIo::update_fields(const r::Register& reg, uint32_t instance, const FieldValue* fields,
                                  size_t count) noexcept {
    uint32_t value = 0;
    if (!read(reg, instance, value)) {
        return false;
    }
    for (size_t index = 0; index < count; ++index) {
        value = place_field(value, fields[index].field, fields[index].value);
    }
    return write(reg, instance, value);
}

bool Dcn32StreamIo::get(const r::Register& reg, uint32_t instance, r::Field field, uint32_t& value) noexcept {
    uint32_t word = 0;
    if (!read(reg, instance, word)) {
        return false;
    }
    value = field_of(word, field);
    return true;
}

bool Dcn32StreamIo::wait(const r::Register& reg, uint32_t instance, r::Field field, uint32_t value,
                         uint32_t delay, uint32_t tries) noexcept {
    for (uint32_t attempt = 0; attempt <= tries; ++attempt) {
        if (attempt != 0 && delay != 0) {
            delay_us(delay);
        }
        uint32_t current = 0;
        if (!get(reg, instance, field, current)) {
            return false;
        }
        if (current == value) {
            return true;
        }
    }
    timed_out_ = true;
    return false;
}

void Dcn32StreamIo::delay_us(uint32_t microseconds) noexcept {
    if (bound_ && microseconds != 0) {
        io_.delay_microseconds(io_.context, microseconds);
    }
}

namespace dcn32_encoder {
namespace {

constexpr uint32_t dp_blank_max_retry = 20;          // DP_BLANK_MAX_RETRY
constexpr uint32_t hdmi_more_340m_khz = 340000;      // HDMI_CLOCK_CHANNEL_RATE_MORE_340M
constexpr uint32_t link_rate_ref_khz = 27000;        // LINK_RATE_REF_FREQ_IN_KHZ
constexpr uint32_t vbi_line_0 = 0;                   // VBI_LINE_0

bool valid_engine(uint32_t engine) noexcept { return engine < engine_count; }

// enc1_stream_encoder_set_stream_attribute_helper, PIXEL_ENCODING_RGB.
bool set_stream_attribute_helper(Dcn32StreamIo& io, uint32_t engine) noexcept {
    return io.update(r::DIG_FE_CNTL::reg, engine, {{r::DIG_FE_CNTL::TMDS_PIXEL_ENCODING, 0}}) &&
           io.update(r::DIG_FE_CNTL::reg, engine, {{r::DIG_FE_CNTL::TMDS_COLOR_FORMAT, 0}});
}

// enc3_update_hdmi_info_packet for one packet index.
bool update_hdmi_info_packet(Dcn32StreamIo& io, uint32_t engine, uint32_t index,
                             const InfoPacket& packet) noexcept {
    uint32_t cont = 0;
    uint32_t send = 0;
    uint32_t line = 0;

    if (packet.valid) {
        if (!vpg_update_generic_info_packet(io, engine, index, packet, true)) {
            return false;
        }
        cont = 1;
        send = 1;
        line = 2;
    }

    switch (index) {
    case 0:
        return io.update(r::HDMI_GENERIC_PACKET_CONTROL0::reg, engine,
                         {{r::HDMI_GENERIC_PACKET_CONTROL0::HDMI_GENERIC0_CONT, cont},
                          {r::HDMI_GENERIC_PACKET_CONTROL0::HDMI_GENERIC0_SEND, send}}) &&
               io.update(r::HDMI_GENERIC_PACKET_CONTROL1::reg, engine,
                         {{r::HDMI_GENERIC_PACKET_CONTROL1::HDMI_GENERIC0_LINE, line}});
    case 1:
        return io.update(r::HDMI_GENERIC_PACKET_CONTROL0::reg, engine,
                         {{r::HDMI_GENERIC_PACKET_CONTROL0::HDMI_GENERIC1_CONT, cont},
                          {r::HDMI_GENERIC_PACKET_CONTROL0::HDMI_GENERIC1_SEND, send}}) &&
               io.update(r::HDMI_GENERIC_PACKET_CONTROL1::reg, engine,
                         {{r::HDMI_GENERIC_PACKET_CONTROL1::HDMI_GENERIC1_LINE, line}});
    case 2:
        return io.update(r::HDMI_GENERIC_PACKET_CONTROL0::reg, engine,
                         {{r::HDMI_GENERIC_PACKET_CONTROL0::HDMI_GENERIC2_CONT, cont},
                          {r::HDMI_GENERIC_PACKET_CONTROL0::HDMI_GENERIC2_SEND, send}}) &&
               io.update(r::HDMI_GENERIC_PACKET_CONTROL2::reg, engine,
                         {{r::HDMI_GENERIC_PACKET_CONTROL2::HDMI_GENERIC2_LINE, line}});
    case 3:
        return io.update(r::HDMI_GENERIC_PACKET_CONTROL0::reg, engine,
                         {{r::HDMI_GENERIC_PACKET_CONTROL0::HDMI_GENERIC3_CONT, cont},
                          {r::HDMI_GENERIC_PACKET_CONTROL0::HDMI_GENERIC3_SEND, send}}) &&
               io.update(r::HDMI_GENERIC_PACKET_CONTROL2::reg, engine,
                         {{r::HDMI_GENERIC_PACKET_CONTROL2::HDMI_GENERIC3_LINE, line}});
    case 4:
        return io.update(r::HDMI_GENERIC_PACKET_CONTROL0::reg, engine,
                         {{r::HDMI_GENERIC_PACKET_CONTROL0::HDMI_GENERIC4_CONT, cont},
                          {r::HDMI_GENERIC_PACKET_CONTROL0::HDMI_GENERIC4_SEND, send}}) &&
               io.update(r::HDMI_GENERIC_PACKET_CONTROL3::reg, engine,
                         {{r::HDMI_GENERIC_PACKET_CONTROL3::HDMI_GENERIC4_LINE, line}});
    case 5:
        return io.update(r::HDMI_GENERIC_PACKET_CONTROL0::reg, engine,
                         {{r::HDMI_GENERIC_PACKET_CONTROL0::HDMI_GENERIC5_CONT, cont},
                          {r::HDMI_GENERIC_PACKET_CONTROL0::HDMI_GENERIC5_SEND, send}}) &&
               io.update(r::HDMI_GENERIC_PACKET_CONTROL3::reg, engine,
                         {{r::HDMI_GENERIC_PACKET_CONTROL3::HDMI_GENERIC5_LINE, line}});
    case 6:
        return io.update(r::HDMI_GENERIC_PACKET_CONTROL0::reg, engine,
                         {{r::HDMI_GENERIC_PACKET_CONTROL0::HDMI_GENERIC6_CONT, cont},
                          {r::HDMI_GENERIC_PACKET_CONTROL0::HDMI_GENERIC6_SEND, send}}) &&
               io.update(r::HDMI_GENERIC_PACKET_CONTROL4::reg, engine,
                         {{r::HDMI_GENERIC_PACKET_CONTROL4::HDMI_GENERIC6_LINE, line}});
    default:
        // Only the seven DCN 3.2 encoder_info_frame slots are ever used.
        return false;
    }
}

} // namespace

bool dig_connect_to_otg(Dcn32StreamIo& io, uint32_t engine, uint32_t otg) noexcept {
    return valid_engine(engine) &&
           io.update(r::DIG_FE_CNTL::reg, engine, {{r::DIG_FE_CNTL::DIG_SOURCE_SELECT, otg}});
}

bool setup_stereo_sync(Dcn32StreamIo& io, uint32_t engine, uint32_t otg, bool enable) noexcept {
    return valid_engine(engine) &&
           io.update(r::DIG_FE_CNTL::reg, engine, {{r::DIG_FE_CNTL::DIG_STEREOSYNC_SELECT, otg}}) &&
           io.update(r::DIG_FE_CNTL::reg, engine, {{r::DIG_FE_CNTL::DIG_STEREOSYNC_GATE_EN, enable ? 0u : 1u}});
}

bool dp_set_stream_attribute(Dcn32StreamIo& io, uint32_t engine, const Timing& timing,
                             bool use_vsc_sdp_for_colorimetry) noexcept {
    if (!valid_engine(engine)) {
        return false;
    }
    // enc1_stream_encoder_dp_set_stream_attribute.
    uint32_t misc1 = 0;
    if (!io.read(r::DP_MSA_MISC::reg, engine, misc1)) {
        return false;
    }
    misc1 = use_vsc_sdp_for_colorimetry ? (misc1 | 0x40u) : (misc1 & ~0x40u);

    // DP_PIXEL_ENCODING_TYPE_RGB444, DP_COMPONENT_PIXEL_DEPTH_8BPC.
    if (!io.update(r::DP_PIXEL_FORMAT::reg, engine,
                   {{r::DP_PIXEL_FORMAT::DP_PIXEL_ENCODING, 0}, {r::DP_PIXEL_FORMAT::DP_COMPONENT_DEPTH, 1}})) {
        return false;
    }

    // colorimetry_bpc 1 (COLOR_DEPTH_888) << 5; COLOR_SPACE_SRGB clears MISC1 bit 7.
    const uint32_t misc0 = 1u << 5;
    misc1 &= ~0x80u;

    const uint32_t h_blank = timing.h_total - timing.width;
    const uint32_t h_back_porch = h_blank - timing.h_front_porch - timing.h_sync_width;
    const uint32_t h_active_start = timing.h_sync_width + h_back_porch;
    const uint32_t v_active_start = timing.v_total - timing.height - timing.v_front_porch;

    return io.set(r::DP_MSA_COLORIMETRY::reg, engine, {{r::DP_MSA_COLORIMETRY::DP_MSA_MISC0, misc0}}) &&
           io.write(r::DP_MSA_MISC::reg, engine, misc1) &&
           io.set(r::DP_MSA_TIMING_PARAM1::reg, engine,
                  {{r::DP_MSA_TIMING_PARAM1::DP_MSA_HTOTAL, timing.h_total},
                   {r::DP_MSA_TIMING_PARAM1::DP_MSA_VTOTAL, timing.v_total}}) &&
           io.set(r::DP_MSA_TIMING_PARAM2::reg, engine,
                  {{r::DP_MSA_TIMING_PARAM2::DP_MSA_HSTART, h_active_start},
                   {r::DP_MSA_TIMING_PARAM2::DP_MSA_VSTART, v_active_start}}) &&
           io.set(r::DP_MSA_TIMING_PARAM3::reg, engine,
                  {{r::DP_MSA_TIMING_PARAM3::DP_MSA_HSYNCWIDTH, timing.h_sync_width},
                   {r::DP_MSA_TIMING_PARAM3::DP_MSA_HSYNCPOLARITY, (timing.flags & positive_hsync) ? 0u : 1u},
                   {r::DP_MSA_TIMING_PARAM3::DP_MSA_VSYNCWIDTH, timing.v_sync_width},
                   {r::DP_MSA_TIMING_PARAM3::DP_MSA_VSYNCPOLARITY, (timing.flags & positive_vsync) ? 0u : 1u}}) &&
           io.set(r::DP_MSA_TIMING_PARAM4::reg, engine,
                  {{r::DP_MSA_TIMING_PARAM4::DP_MSA_HWIDTH, timing.width},
                   {r::DP_MSA_TIMING_PARAM4::DP_MSA_VHEIGHT, timing.height}}) &&
           // enc2: DP_SST_SDP_SPLITTING = dprx SST_SPLIT_SDP_CAP (never set for SST here).
           io.update(r::DP_SEC_FRAMING4::reg, engine, {{r::DP_SEC_FRAMING4::DP_SST_SDP_SPLITTING, 0}});
}

bool hdmi_set_stream_attribute(Dcn32StreamIo& io, uint32_t engine, uint32_t actual_pix_clk_khz) noexcept {
    if (!valid_engine(engine) ||
        !io.update(r::DIG_CLOCK_PATTERN::reg, engine, {{r::DIG_CLOCK_PATTERN::DIG_CLOCK_PATTERN, 0x1f}}) ||
        !set_stream_attribute_helper(io, engine) ||
        !io.update(r::HDMI_CONTROL::reg, engine,
                   {{r::HDMI_CONTROL::HDMI_PACKET_GEN_VERSION, 1},
                    {r::HDMI_CONTROL::HDMI_KEEPOUT_MODE, 1},
                    {r::HDMI_CONTROL::HDMI_DEEP_COLOR_ENABLE, 0},
                    {r::HDMI_CONTROL::HDMI_DATA_SCRAMBLE_EN, 0},
                    {r::HDMI_CONTROL::HDMI_NO_EXTRA_NULL_PACKET_FILLED, 1},
                    {r::HDMI_CONTROL::HDMI_CLOCK_CHANNEL_RATE, 0}}) ||
        // COLOR_DEPTH_888.
        !io.update(r::HDMI_CONTROL::reg, engine, {{r::HDMI_CONTROL::HDMI_DEEP_COLOR_DEPTH, 0}})) {
        return false;
    }

    // LTE_340MCSC_SCRAMBLE is never set for RGB streams (see hdmi_stream).
    if (actual_pix_clk_khz >= hdmi_more_340m_khz &&
        !io.update(r::HDMI_CONTROL::reg, engine,
                   {{r::HDMI_CONTROL::HDMI_DATA_SCRAMBLE_EN, 1}, {r::HDMI_CONTROL::HDMI_CLOCK_CHANNEL_RATE, 1}})) {
        return false;
    }

    return io.update(r::HDMI_VBI_PACKET_CONTROL::reg, engine,
                     {{r::HDMI_VBI_PACKET_CONTROL::HDMI_GC_CONT, 1},
                      {r::HDMI_VBI_PACKET_CONTROL::HDMI_GC_SEND, 1},
                      {r::HDMI_VBI_PACKET_CONTROL::HDMI_NULL_SEND, 1}}) &&
           io.update(r::HDMI_VBI_PACKET_CONTROL::reg, engine, {{r::HDMI_VBI_PACKET_CONTROL::HDMI_ACP_SEND, 0}}) &&
           io.update(r::HDMI_INFOFRAME_CONTROL0::reg, engine,
                     {{r::HDMI_INFOFRAME_CONTROL0::HDMI_AUDIO_INFO_SEND, 1}}) &&
           // afmt3_audio_info_immediate_update.
           io.update(r::AFMT_INFOFRAME_CONTROL0::reg, engine,
                     {{r::AFMT_INFOFRAME_CONTROL0::AFMT_AUDIO_INFO_UPDATE, 1}}) &&
           io.update(r::HDMI_INFOFRAME_CONTROL1::reg, engine,
                     {{r::HDMI_INFOFRAME_CONTROL1::HDMI_AUDIO_INFO_LINE, vbi_line_0 + 2}}) &&
           io.update(r::HDMI_GC::reg, engine, {{r::HDMI_GC::HDMI_GC_AVMUTE, 0}});
}

bool dvi_set_stream_attribute(Dcn32StreamIo& io, uint32_t engine) noexcept {
    return valid_engine(engine) &&
           io.update(r::DIG_CLOCK_PATTERN::reg, engine, {{r::DIG_CLOCK_PATTERN::DIG_CLOCK_PATTERN, 0x1f}}) &&
           set_stream_attribute_helper(io, engine);
}

bool update_hdmi_info_packets(Dcn32StreamIo& io, uint32_t engine, const InfoPacket& avi,
                              const InfoPacket& vendor) noexcept {
    const InfoPacket none{};
    return valid_engine(engine) &&
           io.update(r::HDMI_DB_CONTROL::reg, engine, {{r::HDMI_DB_CONTROL::HDMI_DB_DISABLE, 1}}) &&
           io.update(r::AFMT_CNTL::reg, engine, {{r::AFMT_CNTL::AFMT_AUDIO_CLOCK_EN, 1}}) &&
           update_hdmi_info_packet(io, engine, 0, avi) &&      // avi
           update_hdmi_info_packet(io, engine, 5, none) &&     // hfvsif
           update_hdmi_info_packet(io, engine, 2, none) &&     // gamut
           update_hdmi_info_packet(io, engine, 1, vendor) &&   // vendor
           update_hdmi_info_packet(io, engine, 3, none) &&     // spd
           update_hdmi_info_packet(io, engine, 4, none) &&     // hdrsmd
           update_hdmi_info_packet(io, engine, 6, none);       // vtem
}

bool stop_hdmi_info_packets(Dcn32StreamIo& io, uint32_t engine) noexcept {
    namespace C0 = r::HDMI_GENERIC_PACKET_CONTROL0;
    namespace C6 = r::HDMI_GENERIC_PACKET_CONTROL6;
    if (!valid_engine(engine) ||
        !io.set(C0::reg, engine, {{C0::HDMI_GENERIC0_CONT, 0}, {C0::HDMI_GENERIC0_SEND, 0},
                                  {C0::HDMI_GENERIC1_CONT, 0}, {C0::HDMI_GENERIC1_SEND, 0}}) ||
        !io.set(r::HDMI_GENERIC_PACKET_CONTROL1::reg, engine,
                {{r::HDMI_GENERIC_PACKET_CONTROL1::HDMI_GENERIC0_LINE, 0},
                 {r::HDMI_GENERIC_PACKET_CONTROL1::HDMI_GENERIC1_LINE, 0}})) {
        return false;
    }
    if (!io.set(C0::reg, engine, {{C0::HDMI_GENERIC2_CONT, 0}, {C0::HDMI_GENERIC2_SEND, 0},
                                  {C0::HDMI_GENERIC3_CONT, 0}, {C0::HDMI_GENERIC3_SEND, 0}}) ||
        !io.set(r::HDMI_GENERIC_PACKET_CONTROL2::reg, engine,
                {{r::HDMI_GENERIC_PACKET_CONTROL2::HDMI_GENERIC2_LINE, 0},
                 {r::HDMI_GENERIC_PACKET_CONTROL2::HDMI_GENERIC3_LINE, 0}})) {
        return false;
    }
    if (!io.set(C0::reg, engine, {{C0::HDMI_GENERIC4_CONT, 0}, {C0::HDMI_GENERIC4_SEND, 0},
                                  {C0::HDMI_GENERIC5_CONT, 0}, {C0::HDMI_GENERIC5_SEND, 0}}) ||
        !io.set(r::HDMI_GENERIC_PACKET_CONTROL3::reg, engine,
                {{r::HDMI_GENERIC_PACKET_CONTROL3::HDMI_GENERIC4_LINE, 0},
                 {r::HDMI_GENERIC_PACKET_CONTROL3::HDMI_GENERIC5_LINE, 0}})) {
        return false;
    }
    if (!io.set(C0::reg, engine, {{C0::HDMI_GENERIC6_CONT, 0}, {C0::HDMI_GENERIC6_SEND, 0},
                                  {C0::HDMI_GENERIC7_CONT, 0}, {C0::HDMI_GENERIC7_SEND, 0}}) ||
        !io.set(r::HDMI_GENERIC_PACKET_CONTROL4::reg, engine,
                {{r::HDMI_GENERIC_PACKET_CONTROL4::HDMI_GENERIC6_LINE, 0},
                 {r::HDMI_GENERIC_PACKET_CONTROL4::HDMI_GENERIC7_LINE, 0}})) {
        return false;
    }
    if (!io.set(C6::reg, engine, {{C6::HDMI_GENERIC8_CONT, 0}, {C6::HDMI_GENERIC8_SEND, 0},
                                  {C6::HDMI_GENERIC9_CONT, 0}, {C6::HDMI_GENERIC9_SEND, 0}}) ||
        !io.set(r::HDMI_GENERIC_PACKET_CONTROL7::reg, engine,
                {{r::HDMI_GENERIC_PACKET_CONTROL7::HDMI_GENERIC8_LINE, 0},
                 {r::HDMI_GENERIC_PACKET_CONTROL7::HDMI_GENERIC9_LINE, 0}})) {
        return false;
    }
    if (!io.set(C6::reg, engine, {{C6::HDMI_GENERIC10_CONT, 0}, {C6::HDMI_GENERIC10_SEND, 0},
                                  {C6::HDMI_GENERIC11_CONT, 0}, {C6::HDMI_GENERIC11_SEND, 0}}) ||
        !io.set(r::HDMI_GENERIC_PACKET_CONTROL8::reg, engine,
                {{r::HDMI_GENERIC_PACKET_CONTROL8::HDMI_GENERIC10_LINE, 0},
                 {r::HDMI_GENERIC_PACKET_CONTROL8::HDMI_GENERIC11_LINE, 0}})) {
        return false;
    }
    if (!io.set(C6::reg, engine, {{C6::HDMI_GENERIC12_CONT, 0}, {C6::HDMI_GENERIC12_SEND, 0},
                                  {C6::HDMI_GENERIC13_CONT, 0}, {C6::HDMI_GENERIC13_SEND, 0}}) ||
        !io.set(r::HDMI_GENERIC_PACKET_CONTROL9::reg, engine,
                {{r::HDMI_GENERIC_PACKET_CONTROL9::HDMI_GENERIC12_LINE, 0},
                 {r::HDMI_GENERIC_PACKET_CONTROL9::HDMI_GENERIC13_LINE, 0}})) {
        return false;
    }
    return io.set(C6::reg, engine, {{C6::HDMI_GENERIC14_CONT, 0}, {C6::HDMI_GENERIC14_SEND, 0}}) &&
           io.update(r::HDMI_GENERIC_PACKET_CONTROL10::reg, engine,
                     {{r::HDMI_GENERIC_PACKET_CONTROL10::HDMI_GENERIC14_LINE, 0}});
}

bool hdmi_reset_stream_attribute(Dcn32StreamIo& io, uint32_t engine) noexcept {
    return valid_engine(engine) &&
           io.update(r::HDMI_CONTROL::reg, engine,
                     {{r::HDMI_CONTROL::HDMI_PACKET_GEN_VERSION, 1},
                      {r::HDMI_CONTROL::HDMI_KEEPOUT_MODE, 1},
                      {r::HDMI_CONTROL::HDMI_DEEP_COLOR_ENABLE, 0},
                      {r::HDMI_CONTROL::HDMI_DATA_SCRAMBLE_EN, 0},
                      {r::HDMI_CONTROL::HDMI_CLOCK_CHANNEL_RATE, 0}});
}

bool update_dp_info_packets(Dcn32StreamIo& io, uint32_t engine, const InfoPacket& vsc) noexcept {
    if (!valid_engine(engine)) {
        return false;
    }
    // VSC at packet index 0, then twice at index 1 (the Linux sequence).
    if (vsc.valid &&
        (!vpg_update_generic_info_packet(io, engine, 0, vsc, true) ||
         !vpg_update_generic_info_packet(io, engine, 1, vsc, true) ||
         !vpg_update_generic_info_packet(io, engine, 1, vsc, true))) {
        return false;
    }
    namespace S = r::DP_SEC_CNTL;
    if (!io.update(S::reg, engine, {{S::DP_SEC_GSP0_ENABLE, vsc.valid ? 1u : 0u}}) ||
        !io.update(S::reg, engine, {{S::DP_SEC_GSP2_ENABLE, 0}}) ||
        !io.update(S::reg, engine, {{S::DP_SEC_GSP3_ENABLE, 0}}) ||
        !io.update(S::reg, engine, {{S::DP_SEC_GSP5_ENABLE, 0}})) {
        return false;
    }
    uint32_t value = 0;
    if (!io.read(S::reg, engine, value) ||
        (value != 0 && !io.update(S::reg, engine, {{S::DP_SEC_STREAM_ENABLE, 1}}))) {
        return false;
    }
    uint32_t dmdata = 0;
    if (!io.get(r::DP_SEC_METADATA_TRANSMISSION::reg, engine,
                r::DP_SEC_METADATA_TRANSMISSION::DP_SEC_METADATA_PACKET_ENABLE, dmdata)) {
        return false;
    }
    return dmdata == 0 || io.update(S::reg, engine, {{S::DP_SEC_STREAM_ENABLE, 1}});
}

bool stop_dp_info_packets(Dcn32StreamIo& io, uint32_t engine) noexcept {
    namespace S = r::DP_SEC_CNTL;
    if (!valid_engine(engine) ||
        !io.set(S::reg, engine,
                {{S::DP_SEC_GSP0_ENABLE, 0}, {S::DP_SEC_GSP1_ENABLE, 0}, {S::DP_SEC_GSP2_ENABLE, 0},
                 {S::DP_SEC_GSP3_ENABLE, 0}, {S::DP_SEC_GSP4_ENABLE, 0}, {S::DP_SEC_GSP5_ENABLE, 0},
                 {S::DP_SEC_GSP6_ENABLE, 0}, {S::DP_SEC_GSP7_ENABLE, 0}, {S::DP_SEC_MPG_ENABLE, 0},
                 {S::DP_SEC_STREAM_ENABLE, 0}})) {
        return false;
    }
    uint32_t value = 0;
    if (!io.read(S::reg, engine, value)) {
        return false;
    }
    return value == 0 || io.update(S::reg, engine, {{S::DP_SEC_STREAM_ENABLE, 1}});
}

bool dp_blank(Dcn32StreamIo& io, uint32_t engine) noexcept {
    namespace V = r::DP_VID_STREAM_CNTL;
    uint32_t enabled = 0;
    if (!valid_engine(engine) || !io.get(V::reg, engine, V::DP_VID_STREAM_ENABLE, enabled)) {
        return false;
    }
    if ((enabled & 1u) == 0) {
        return true; // stream not enabled
    }
    return io.update(V::reg, engine, {{V::DP_VID_STREAM_DIS_DEFER, 2}}) &&
           io.update(V::reg, engine, {{V::DP_VID_STREAM_ENABLE, 0}}) &&
           io.wait(V::reg, engine, V::DP_VID_STREAM_STATUS, 0, 10, dp_blank_max_retry * 501) &&
           io.update(r::DP_STEER_FIFO::reg, engine, {{r::DP_STEER_FIFO::DP_STEER_FIFO_RESET, 1}});
}

bool dp_unblank(Dcn32StreamIo& io, uint32_t engine, uint32_t pixel_clock_khz, uint32_t link_rate_code,
                uint32_t pix_per_cycle) noexcept {
    if (!valid_engine(engine) || link_rate_code == 0) {
        return false;
    }
    // enc32_dp_set_odm_combine(pix_per_cycle > 1).
    if (!io.update(r::DP_PIXEL_FORMAT::reg, engine,
                   {{r::DP_PIXEL_FORMAT::DP_PIXEL_PER_CYCLE_PROCESSING_MODE, pix_per_cycle > 1 ? 1u : 0u}})) {
        return false;
    }

    const uint32_t n_vid = 0x8000;
    uint32_t n_multiply = 0;
    uint32_t pixel_per_cycle = 0;
    if (pix_per_cycle > 1) {
        n_multiply = 1;
        pixel_per_cycle = 1;
    }
    const uint64_t m_vid_l = static_cast<uint64_t>(n_vid) * pixel_clock_khz /
                             (static_cast<uint64_t>(link_rate_code) * link_rate_ref_khz);
    const uint32_t m_vid = static_cast<uint32_t>(m_vid_l);

    namespace V = r::DP_VID_STREAM_CNTL;
    namespace F = r::DIG_FIFO_CTRL0;
    return io.update(r::DP_VID_TIMING::reg, engine, {{r::DP_VID_TIMING::DP_VID_M_N_GEN_EN, 0}}) &&
           io.update(r::DP_VID_N::reg, engine, {{r::DP_VID_N::DP_VID_N, n_vid}}) &&
           io.update(r::DP_VID_M::reg, engine, {{r::DP_VID_M::DP_VID_M, m_vid}}) &&
           io.update(r::DP_VID_TIMING::reg, engine,
                     {{r::DP_VID_TIMING::DP_VID_M_N_GEN_EN, 1}, {r::DP_VID_TIMING::DP_VID_N_MUL, n_multiply}}) &&
           io.update(r::DP_PIXEL_FORMAT::reg, engine,
                     {{r::DP_PIXEL_FORMAT::DP_PIXEL_PER_CYCLE_PROCESSING_MODE, pixel_per_cycle}}) &&
           io.update(V::reg, engine, {{V::DP_VID_STREAM_ENABLE, 0}}) &&
           io.wait(V::reg, engine, V::DP_VID_STREAM_STATUS, 0, 10, 5000) &&
           io.update(r::DP_STEER_FIFO::reg, engine, {{r::DP_STEER_FIFO::DP_STEER_FIFO_RESET, 1}}) &&
           (io.delay_us(10), true) &&
           io.update(r::DP_STEER_FIFO::reg, engine, {{r::DP_STEER_FIFO::DP_STEER_FIFO_RESET, 0}}) &&
           io.wait(r::DIG_FE_CNTL::reg, engine, r::DIG_FE_CNTL::DIG_SYMCLK_FE_ON, 1, 10, 5000) &&
           io.update(F::reg, engine, {{F::DIG_FIFO_READ_START_LEVEL, 0x7}}) &&
           io.update(F::reg, engine, {{F::DIG_FIFO_RESET, 1}}) &&
           io.wait(F::reg, engine, F::DIG_FIFO_RESET_DONE, 1, 10, 5000) &&
           io.update(F::reg, engine, {{F::DIG_FIFO_RESET, 0}}) &&
           io.wait(F::reg, engine, F::DIG_FIFO_RESET_DONE, 0, 10, 5000) &&
           io.update(F::reg, engine, {{F::DIG_FIFO_ENABLE, 1}}) &&
           (io.delay_us(100), true) &&
           io.update(V::reg, engine, {{V::DP_VID_STREAM_ENABLE, 1}});
}

bool set_avmute(Dcn32StreamIo& io, uint32_t engine, bool enable) noexcept {
    return valid_engine(engine) &&
           io.update(r::HDMI_GC::reg, engine, {{r::HDMI_GC::HDMI_GC_AVMUTE, enable ? 1u : 0u}});
}

bool set_input_mode(Dcn32StreamIo& io, uint32_t engine, uint32_t pix_per_container) noexcept {
    return valid_engine(engine) &&
           io.update(r::DIG_FIFO_CTRL0::reg, engine,
                     {{r::DIG_FIFO_CTRL0::DIG_FIFO_OUTPUT_PIXEL_MODE, pix_per_container == 2 ? 1u : 0u}});
}

namespace {

// enc32_reset_fifo.
bool reset_fifo(Dcn32StreamIo& io, uint32_t engine, bool reset) noexcept {
    const uint32_t value = reset ? 1u : 0u;
    uint32_t symclk_on = 0;
    if (!io.update(r::DIG_FIFO_CTRL0::reg, engine, {{r::DIG_FIFO_CTRL0::DIG_FIFO_RESET, value}}) ||
        !io.get(r::DIG_FE_CNTL::reg, engine, r::DIG_FE_CNTL::DIG_SYMCLK_FE_ON, symclk_on)) {
        return false;
    }
    if (symclk_on != 0) {
        return io.wait(r::DIG_FIFO_CTRL0::reg, engine, r::DIG_FIFO_CTRL0::DIG_FIFO_RESET_DONE, value, 10, 5000);
    }
    io.delay_us(10);
    return true;
}

} // namespace

bool enable_fifo(Dcn32StreamIo& io, uint32_t engine) noexcept {
    return valid_engine(engine) &&
           io.update(r::DIG_FIFO_CTRL0::reg, engine, {{r::DIG_FIFO_CTRL0::DIG_FIFO_READ_START_LEVEL, 0x7}}) &&
           reset_fifo(io, engine, true) && reset_fifo(io, engine, false) &&
           io.update(r::DIG_FIFO_CTRL0::reg, engine, {{r::DIG_FIFO_CTRL0::DIG_FIFO_ENABLE, 1}});
}

bool disable_dynamic_metadata(Dcn32StreamIo& io, uint32_t engine, bool display_port) noexcept {
    if (!valid_engine(engine) ||
        !io.update(r::DME_CONTROL::reg, engine, {{r::DME_CONTROL::METADATA_ENGINE_EN, 0}})) {
        return false;
    }
    if (display_port) {
        return io.update(r::DP_SEC_METADATA_TRANSMISSION::reg, engine,
                         {{r::DP_SEC_METADATA_TRANSMISSION::DP_SEC_METADATA_PACKET_ENABLE, 0}});
    }
    return io.update(r::HDMI_METADATA_PACKET_CONTROL::reg, engine,
                     {{r::HDMI_METADATA_PACKET_CONTROL::HDMI_METADATA_PACKET_ENABLE, 0}}) &&
           io.update(r::DIG_FE_CNTL::reg, engine, {{r::DIG_FE_CNTL::DOLBY_VISION_EN, 0}});
}

bool audio_mute(Dcn32StreamIo& io, uint32_t engine) noexcept {
    return valid_engine(engine) &&
           io.update(r::AFMT_AUDIO_PACKET_CONTROL::reg, engine,
                     {{r::AFMT_AUDIO_PACKET_CONTROL::AFMT_AUDIO_SAMPLE_SEND, 0}});
}

bool vpg_update_generic_info_packet(Dcn32StreamIo& io, uint32_t vpg, uint32_t packet_index,
                                    const InfoPacket& packet, bool immediate_update) noexcept {
    if (packet_index >= vpg_packets || vpg >= engine_count) {
        return false;
    }
    namespace S = r::VPG_GENERIC_STATUS;
    namespace D = r::VPG_GENERIC_PACKET_DATA;
    if (!io.wait(S::reg, vpg, S::VPG_GENERIC_CONFLICT_OCCURED, 0, 10, 50) ||
        !io.update(S::reg, vpg, {{S::VPG_GENERIC_CONFLICT_CLR, 1}}) ||
        !io.update(r::VPG_GENERIC_PACKET_ACCESS_CTRL::reg, vpg,
                   {{r::VPG_GENERIC_PACKET_ACCESS_CTRL::VPG_GENERIC_DATA_INDEX, packet_index * 9}}) ||
        !io.set(D::reg, vpg,
                {{D::VPG_GENERIC_DATA_BYTE0, packet.hb0}, {D::VPG_GENERIC_DATA_BYTE1, packet.hb1},
                 {D::VPG_GENERIC_DATA_BYTE2, packet.hb2}, {D::VPG_GENERIC_DATA_BYTE3, packet.hb3}})) {
        return false;
    }
    // Eight little-endian dwords of sb[0..31] (the uint32_t view Linux writes).
    for (uint32_t index = 0; index < 8; ++index) {
        const uint8_t* bytes = &packet.sb[index * 4];
        const uint32_t word = static_cast<uint32_t>(bytes[0]) | (static_cast<uint32_t>(bytes[1]) << 8) |
                              (static_cast<uint32_t>(bytes[2]) << 16) | (static_cast<uint32_t>(bytes[3]) << 24);
        if (!io.write(D::reg, vpg, word)) {
            return false;
        }
    }
    namespace I = r::VPG_GSP_IMMEDIATE_UPDATE_CTRL;
    namespace U = r::VPG_GSP_FRAME_UPDATE_CTRL;
    static constexpr r::Field immediate[vpg_packets] = {
        I::VPG_GENERIC0_IMMEDIATE_UPDATE, I::VPG_GENERIC1_IMMEDIATE_UPDATE, I::VPG_GENERIC2_IMMEDIATE_UPDATE,
        I::VPG_GENERIC3_IMMEDIATE_UPDATE, I::VPG_GENERIC4_IMMEDIATE_UPDATE, I::VPG_GENERIC5_IMMEDIATE_UPDATE,
        I::VPG_GENERIC6_IMMEDIATE_UPDATE, I::VPG_GENERIC7_IMMEDIATE_UPDATE, I::VPG_GENERIC8_IMMEDIATE_UPDATE,
        I::VPG_GENERIC9_IMMEDIATE_UPDATE, I::VPG_GENERIC10_IMMEDIATE_UPDATE, I::VPG_GENERIC11_IMMEDIATE_UPDATE,
        I::VPG_GENERIC12_IMMEDIATE_UPDATE, I::VPG_GENERIC13_IMMEDIATE_UPDATE, I::VPG_GENERIC14_IMMEDIATE_UPDATE};
    static constexpr r::Field frame[vpg_packets] = {
        U::VPG_GENERIC0_FRAME_UPDATE, U::VPG_GENERIC1_FRAME_UPDATE, U::VPG_GENERIC2_FRAME_UPDATE,
        U::VPG_GENERIC3_FRAME_UPDATE, U::VPG_GENERIC4_FRAME_UPDATE, U::VPG_GENERIC5_FRAME_UPDATE,
        U::VPG_GENERIC6_FRAME_UPDATE, U::VPG_GENERIC7_FRAME_UPDATE, U::VPG_GENERIC8_FRAME_UPDATE,
        U::VPG_GENERIC9_FRAME_UPDATE, U::VPG_GENERIC10_FRAME_UPDATE, U::VPG_GENERIC11_FRAME_UPDATE,
        U::VPG_GENERIC12_FRAME_UPDATE, U::VPG_GENERIC13_FRAME_UPDATE, U::VPG_GENERIC14_FRAME_UPDATE};
    if (immediate_update) {
        return io.update(I::reg, vpg, {{immediate[packet_index], 1}});
    }
    return io.update(U::reg, vpg, {{frame[packet_index], 1}});
}

bool dp_set_dsc_config(Dcn32StreamIo& io, uint32_t engine, bool enable) noexcept {
    return valid_engine(engine) &&
           io.update(r::DP_DSC_CNTL::reg, engine, {{r::DP_DSC_CNTL::DP_DSC_MODE, enable ? 1u : 0u}});
}

bool dp_set_dsc_pps_info_packet(Dcn32StreamIo& io, uint32_t engine, bool enable, const uint8_t* pps,
                                bool immediate_update) noexcept {
    if (!valid_engine(engine)) {
        return false;
    }
    if (!enable) {
        // Disable Generic Stream Packet 11 (GSP) transmission.
        return io.update(r::DP_GSP11_CNTL::reg, engine, {{r::DP_GSP11_CNTL::DP_SEC_GSP11_ENABLE, 0}}) &&
               io.update(r::DP_SEC_CNTL2::reg, engine, {{r::DP_SEC_CNTL2::DP_SEC_GSP11_PPS, 0}});
    }
    if (pps == nullptr) {
        return false;
    }
    // PPS packet size (128 bytes), then the AFMT clock before the SDP registers.
    if (!io.update(r::DP_SEC_CNTL2::reg, engine, {{r::DP_SEC_CNTL2::DP_SEC_GSP11_PPS, 1}}) ||
        !io.update(r::AFMT_CNTL::reg, engine, {{r::AFMT_CNTL::AFMT_AUDIO_CLOCK_EN, 1}})) {
        return false;
    }
    InfoPacket sdp{};
    sdp.valid = true;
    sdp.hb0 = 0;
    sdp.hb1 = 0x10; // DC_DP_INFOFRAME_TYPE_PPS
    sdp.hb2 = 127;
    sdp.hb3 = 0;
    for (uint32_t i = 0; i < 4; ++i) {
        for (uint32_t byte = 0; byte < 32; ++byte) {
            sdp.sb[byte] = pps[i * 32 + byte];
        }
        if (!vpg_update_generic_info_packet(io, engine, 11 + i, sdp, immediate_update)) {
            return false;
        }
    }
    // VBID[6] after the PPS line; PPS sent every frame.
    namespace G = r::DP_GSP11_CNTL;
    namespace M = r::DP_MSA_VBID_MISC;
    return io.update(G::reg, engine, {{G::DP_SEC_GSP11_LINE_NUM, 2}}) &&
           io.update(M::reg, engine, {{M::DP_VBID6_LINE_REFERENCE, 0}, {M::DP_VBID6_LINE_NUM, 3}}) &&
           io.update(G::reg, engine, {{G::DP_SEC_GSP11_ENABLE, 1}}) &&
           io.update(r::DP_SEC_CNTL::reg, engine, {{r::DP_SEC_CNTL::DP_SEC_STREAM_ENABLE, 1}});
}

} // namespace dcn32_encoder

} // namespace rx7900::display

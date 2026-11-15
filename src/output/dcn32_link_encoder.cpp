/*
Module: rx7900.output.dcn32_link_encoder
Purpose: DCN3.2 DIO link-encoder registers and VBIOS PHY command tables.
Created: 2026-10-24
Architecture: Exact v6.12 dcn_3_2_0_offset.h/_sh_mask.h values (BASE_IDX 2;
DIG/DP blocks stride 0x100 per transmitter, AUX 0x1c per channel, HPD 8) and
atomfirmware.h parameter layouts; sequences from the Linux files named in the
header.
Copyright 2012-2023 Advanced Micro Devices, Inc.
Linux notice and provenance: licenses/AMDGPU-OUTPUT.txt.
*/
#include "dcn32_link_encoder.hpp"

namespace rx7900::output {
namespace {

constexpr uint32_t block_stride = 0x100u;
constexpr uint32_t dig_be_cntl = 0x20b1u;
constexpr uint32_t dig_be_en_cntl = 0x20b2u;
constexpr uint32_t tmds_ctl_bits = 0x20dfu;
constexpr uint32_t dp_link_cntl = 0x2108u;
constexpr uint32_t dp_config = 0x210bu;
constexpr uint32_t dp_dphy_internal_ctrl = 0x210fu;
constexpr uint32_t dp_link_framing_cntl = 0x2113u;
constexpr uint32_t dp_dphy_cntl = 0x2117u;
constexpr uint32_t dp_dphy_training_pattern_sel = 0x2118u;
constexpr uint32_t dp_dphy_prbs_cntl = 0x211du;
constexpr uint32_t dp_dphy_scram_cntl = 0x211eu;
constexpr uint32_t aux_stride = 0x1cu;
constexpr uint32_t aux_control = 0x1f50u;
constexpr uint32_t aux_dphy_tx_control = 0x1f59u;
constexpr uint32_t aux_dphy_rx_control0 = 0x1f5au;
constexpr uint32_t hpd_stride = 8u;
constexpr uint32_t hpd_control = 0x1f16u;

constexpr uint32_t dig_fe_source_select = 0x00007f00u;
constexpr uint32_t dig_fe_source_shift = 8u;
constexpr uint32_t dig_mode = 0x00070000u;
constexpr uint32_t dig_mode_shift = 16u;
constexpr uint32_t dig_enable = 0x00000001u;
constexpr uint32_t tmds_ctl0 = 0x00000001u;
constexpr uint32_t dp_udi_lanes = 0x00000003u;
constexpr uint32_t scrambler_advance = 0x00000010u;
constexpr uint32_t scrambler_bs_count = 0x0003ff00u;
constexpr uint32_t scrambler_bs_count_shift = 8u;
constexpr uint32_t training_complete = 0x00000010u;
constexpr uint32_t dphy_bypass = 0x00010000u;
constexpr uint32_t dphy_fec_en = 0x00000010u;
constexpr uint32_t dphy_fec_ready_shadow = 0x00000020u;
constexpr uint32_t dphy_fec_active_status = 0x00000040u;
constexpr uint32_t prbs_enable = 0x00000001u;
constexpr uint32_t idle_bs_interval = 0x0003ffffu;
constexpr uint32_t vbid_disable = 0x01000000u;
constexpr uint32_t enhanced_frame_mode = 0x10000000u;
constexpr uint32_t aux_hpd_sel = 0x00700000u;
constexpr uint32_t aux_hpd_sel_shift = 20u;
constexpr uint32_t aux_ls_read_en = 0x00000100u;
constexpr uint32_t aux_rx_receive_window = 0x00000700u;
constexpr uint32_t hpd_enable = 0x10000000u;

// enc32_hw_init literal register values.
constexpr uint32_t enc32_aux_rx_control0 = 0x103d1110u;
constexpr uint32_t enc32_aux_tx_control = 0x21c7au;

// LINK_RATE_REF_FREQ_IN_KHZ.
constexpr uint32_t link_rate_reference_khz = 27000u;
// atomfirmware.h ATOM_TRANSMITTER_DIGMODE_V6_* and encoder modes.
constexpr uint8_t digmode_dp = 0;
constexpr uint8_t digmode_dvi = 2;
constexpr uint8_t digmode_hdmi = 3;
constexpr uint8_t atom_combophy_pll0 = 20;
constexpr uint8_t atom_crtc_invalid = 0xff;
constexpr uint8_t misc_force_program_ppll = 0x01;
constexpr uint8_t misc_dvi_dual_link = 0x08;
constexpr uint8_t misc_ref_xtalin = 0x00;
// ObjectID.h ENCODER_OBJECT_ID_INTERNAL_UNIPHY/1/2 by transmitter pair.
constexpr uint8_t uniphy_objects[3]{0x1e, 0x20, 0x21};
// TMDS above 6 GHz-class HDMI 2.0 is out of range for SetPixelClock (dcn31 check).
constexpr uint32_t maximum_pll_100hz = 6000000u;

uint8_t digmode_of(LinkSignal signal) noexcept {
    switch (signal) {
    case LinkSignal::display_port: return digmode_dp;
    case LinkSignal::hdmi: return digmode_hdmi;
    default: return digmode_dvi;
    }
}

bool valid_rate(uint8_t rate) noexcept {
    return rate == 0x06 || rate == 0x0a || rate == 0x14 || rate == 0x1e;
}

bool valid_lanes(uint8_t lanes) noexcept { return lanes == 1 || lanes == 2 || lanes == 4; }

bool tmds_signal(LinkSignal signal) noexcept {
    return signal == LinkSignal::hdmi || signal == LinkSignal::dvi_single ||
           signal == LinkSignal::dvi_dual;
}

// The sink decides HDMI versus DVI (EDID HDMI VSDB): an HDMI connector may
// carry single-link DVI, a DVI connector never carries HDMI.
bool signal_allowed(LinkSignal connector, LinkSignal signal) noexcept {
    switch (connector) {
    case LinkSignal::display_port: return signal == LinkSignal::display_port;
    case LinkSignal::hdmi: return signal == LinkSignal::hdmi || signal == LinkSignal::dvi_single;
    case LinkSignal::dvi_single: return signal == LinkSignal::dvi_single;
    case LinkSignal::dvi_dual:
        return signal == LinkSignal::dvi_single || signal == LinkSignal::dvi_dual;
    default: return false;
    }
}

} // namespace

void pack_transmitter_control(const TransmitterControl& control, uint32_t (&dwords)[8]) noexcept {
    for (uint32_t& value : dwords) {
        value = 0;
    }
    dwords[0] = static_cast<uint32_t>(control.phy_id) |
                (static_cast<uint32_t>(control.action) << 8) |
                (static_cast<uint32_t>(control.mode_laneset) << 16) |
                (static_cast<uint32_t>(control.lanes) << 24);
    dwords[1] = control.symclk_10khz;
    dwords[2] = static_cast<uint32_t>(control.hpd_select) |
                (static_cast<uint32_t>(control.dig_fe_select) << 8) |
                (static_cast<uint32_t>(control.connector_id) << 16);
}

void pack_pixel_clock_control(const PixelClockControl& control, uint32_t (&dwords)[4]) noexcept {
    dwords[0] = control.pixel_100hz;
    dwords[1] = static_cast<uint32_t>(control.pll_id) |
                (static_cast<uint32_t>(control.encoder_object) << 8) |
                (static_cast<uint32_t>(control.encoder_mode) << 16) |
                (static_cast<uint32_t>(control.misc) << 24);
    dwords[2] = static_cast<uint32_t>(control.crtc) |
                (static_cast<uint32_t>(control.deep_color_ratio) << 8);
    dwords[3] = 0;
}

EncoderResult Dcn32LinkEncoder::bind(const display::Dcn32RegisterSpace& space,
                                     const Dcn32ConnectorRoute& route, AtomRunner atom) noexcept {
    if (bound_ || !route.validated || route.transmitter >= dcn32_link_count ||
        route.aux_engine >= dcn32_link_count || route.hpd_source >= dcn32_link_count ||
        !space.discovery_verified || space.dmu_major != 3 || space.dmu_minor != 2 ||
        space.dmu_revision != 0 || atom.interpreter == nullptr || !atom.interpreter->bound() ||
        !hardware::register_io_valid(atom.io)) {
        return EncoderResult::invalid_argument;
    }
    AtomTable table{};
    if (atom.interpreter->command_table(atom_command_dig1_transmitter_control, table) !=
            AtomTableStatus::success ||
        table.revision != 6) {
        return EncoderResult::unsupported_revision;
    }
    if (tmds_signal(route.signal) &&
        (atom.interpreter->command_table(atom_command_set_pixel_clock, table) !=
             AtomTableStatus::success ||
         table.revision != 7)) {
        return EncoderResult::unsupported_revision;
    }
    space_ = space;
    route_ = route;
    atom_ = atom;
    bound_ = true;
    quarantined_ = false;
    active_signal_ = LinkSignal::none;
    return EncoderResult::success;
}

void Dcn32LinkEncoder::unbind() noexcept {
    // A quarantined encoder stays bound so nothing can rebind its PHY.
    if (!quarantined_) {
        reset_proven();
    }
}

void Dcn32LinkEncoder::reset_proven() noexcept {
    space_ = {};
    route_ = {};
    atom_ = {};
    atom_report_ = {};
    active_signal_ = LinkSignal::none;
    rate_code_ = 0;
    lanes_ = 0;
    bound_ = false;
    quarantined_ = false;
}

EncoderResult Dcn32LinkEncoder::write(uint32_t reg, uint32_t value) noexcept {
    if (!hardware::write_ip_register(atom_.io, space_.segment2_dword_base, reg, value)) {
        quarantined_ = true;
        return EncoderResult::quarantined;
    }
    return EncoderResult::success;
}

EncoderResult Dcn32LinkEncoder::update(uint32_t reg, uint32_t field, uint32_t value) noexcept {
    uint32_t current = 0;
    if (!hardware::read_ip_register(atom_.io, space_.segment2_dword_base, reg, current) ||
        current == UINT32_MAX) {
        return EncoderResult::refused;
    }
    return write(reg, (current & ~field) | (value & field));
}

TransmitterControl Dcn32LinkEncoder::base_control(TransmitterAction action) const noexcept {
    TransmitterControl control{};
    control.phy_id = route_.transmitter;
    control.action = action;
    // hpd_sel_to_atom: HPD_SOURCEIDn -> n + 1. dig_encoder_sel_to_atom is 0:
    // the FE is connected by connect_fe, not by the VBIOS.
    control.hpd_select = static_cast<uint8_t>(route_.hpd_source + 1u);
    control.dig_fe_select = 0;
    return control;
}

EncoderResult Dcn32LinkEncoder::transmitter(const TransmitterControl& control) noexcept {
    uint32_t dwords[8]{};
    pack_transmitter_control(control, dwords);
    atom_.interpreter->clear_parameters();
    for (uint32_t index = 0; index < 8; ++index) {
        atom_.interpreter->set_parameter(index, dwords[index]);
    }
    const hardware::AtomStatus status = atom_.interpreter->execute(
        atom_.io, atom_command_dig1_transmitter_control, 8, atom_.policy);
    atom_report_ = atom_.interpreter->report();
    if (status == hardware::AtomStatus::completed) {
        return EncoderResult::success;
    }
    // Any started table may have written PHY registers before it stopped.
    if (atom_report_.register_writes != 0 || atom_report_.steps != 0) {
        quarantined_ = true;
        return EncoderResult::quarantined;
    }
    return EncoderResult::refused;
}

EncoderResult Dcn32LinkEncoder::hw_init() noexcept {
    if (!usable()) {
        return quarantined_ ? EncoderResult::quarantined : EncoderResult::not_bound;
    }
    const uint32_t aux = route_.aux_engine * aux_stride;
    const uint32_t dig = route_.transmitter * block_stride;
    EncoderResult result = write(aux_dphy_rx_control0 + aux, enc32_aux_rx_control0);
    if (result == EncoderResult::success) {
        result = write(aux_dphy_tx_control + aux, enc32_aux_tx_control);
    }
    if (result == EncoderResult::success) {
        result = update(tmds_ctl_bits + dig, tmds_ctl0, tmds_ctl0);
    }
    if (result == EncoderResult::success) {
        result = update(aux_control + aux, aux_hpd_sel | aux_ls_read_en,
                        static_cast<uint32_t>(route_.hpd_source) << aux_hpd_sel_shift);
    }
    if (result == EncoderResult::success) {
        result = update(aux_dphy_rx_control0 + aux, aux_rx_receive_window, 0);
    }
    if (result == EncoderResult::success) {
        result = update(hpd_control + route_.hpd_source * hpd_stride, hpd_enable, hpd_enable);
    }
    // After the first write a refused read leaves a partly initialized block.
    if (result == EncoderResult::refused) {
        quarantined_ = true;
        result = EncoderResult::quarantined;
    }
    return result;
}

EncoderResult Dcn32LinkEncoder::setup(LinkSignal signal) noexcept {
    if (!usable()) {
        return quarantined_ ? EncoderResult::quarantined : EncoderResult::not_bound;
    }
    if (!signal_allowed(route_.signal, signal)) {
        return EncoderResult::invalid_argument;
    }
    return update(dig_be_cntl + route_.transmitter * block_stride, dig_mode,
                  static_cast<uint32_t>(digmode_of(signal)) << dig_mode_shift);
}

EncoderResult Dcn32LinkEncoder::connect_fe(uint8_t dig_fe, bool connect) noexcept {
    if (!usable()) {
        return quarantined_ ? EncoderResult::quarantined : EncoderResult::not_bound;
    }
    if (dig_fe >= dcn32_link_count) {
        return EncoderResult::invalid_argument;
    }
    const uint32_t reg = dig_be_cntl + route_.transmitter * block_stride;
    uint32_t current = 0;
    if (!hardware::read_ip_register(atom_.io, space_.segment2_dword_base, reg, current) ||
        current == UINT32_MAX) {
        return EncoderResult::refused;
    }
    uint32_t field = (current & dig_fe_source_select) >> dig_fe_source_shift;
    const uint32_t mask = 1u << dig_fe;
    field = connect ? (field | mask) : (field & ~mask);
    return write(reg, (current & ~dig_fe_source_select) | (field << dig_fe_source_shift));
}

EncoderResult Dcn32LinkEncoder::enable_dp(uint8_t rate_code, uint8_t lanes) noexcept {
    if (!usable()) {
        return quarantined_ ? EncoderResult::quarantined : EncoderResult::not_bound;
    }
    if (route_.signal != LinkSignal::display_port || !valid_rate(rate_code) || !valid_lanes(lanes)) {
        return EncoderResult::invalid_argument;
    }
    const uint32_t dp = route_.transmitter * block_stride;
    // enc1_configure_encoder: REG_SET(DP_CONFIG, 0, DP_UDI_LANES, lanes - 1).
    EncoderResult result = write(dp_config + dp, (lanes - 1u) & dp_udi_lanes);
    if (result == EncoderResult::success) {
        result = update(dp_dphy_scram_cntl + dp, scrambler_advance, scrambler_advance);
    }
    if (result == EncoderResult::success) {
        TransmitterControl control = base_control(TransmitterAction::enable);
        control.mode_laneset = digmode_dp;
        control.lanes = lanes;
        control.symclk_10khz = rate_code * link_rate_reference_khz / 10u;
        // dcn10 leaves connector_obj_id zero for DP enable.
        result = transmitter(control);
    }
    if (result == EncoderResult::refused) {
        quarantined_ = true;
        return EncoderResult::quarantined;
    }
    if (result == EncoderResult::success) {
        active_signal_ = LinkSignal::display_port;
        rate_code_ = rate_code;
        lanes_ = lanes;
    }
    return result;
}

EncoderResult Dcn32LinkEncoder::set_lane_settings(uint8_t rate_code, const uint8_t* lane_settings,
                                                  uint8_t lanes) noexcept {
    if (!usable()) {
        return quarantined_ ? EncoderResult::quarantined : EncoderResult::not_bound;
    }
    if (lane_settings == nullptr || !valid_lanes(lanes) || !valid_rate(rate_code) ||
        active_signal_ != LinkSignal::display_port) {
        return EncoderResult::invalid_argument;
    }
    EncoderResult result = EncoderResult::success;
    for (uint8_t lane = 0; lane < lanes; ++lane) {
        // dpcd_training_lane_set: VOLTAGE_SWING_SET 1:0, PRE_EMPHASIS_SET 4:3;
        // the MAX_*_REACHED bits stay clear and POST_CURSOR2 is HBR2-only and
        // never requested by dp_training.
        const uint8_t swing = lane_settings[lane] & 0x3u;
        const uint8_t emphasis = (lane_settings[lane] >> 3) & 0x3u;
        TransmitterControl control = base_control(TransmitterAction::setup_vsemph);
        control.dig_fe_select = 0;
        control.mode_laneset = static_cast<uint8_t>(swing | (emphasis << 3));
        control.lanes = lanes;
        control.connector_id = route_.connector_id;
        control.symclk_10khz = rate_code * link_rate_reference_khz / 10u;
        // lane_select is not part of the v1.6 parameter block: Linux passes it
        // in bp_transmitter_control only. Each call carries the lane's setting.
        result = transmitter(control);
        if (result != EncoderResult::success) {
            return result;
        }
    }
    return result;
}

EncoderResult Dcn32LinkEncoder::set_dp_pattern(uint8_t pattern) noexcept {
    if (!usable()) {
        return quarantined_ ? EncoderResult::quarantined : EncoderResult::not_bound;
    }
    if (pattern > 4 || active_signal_ != LinkSignal::display_port) {
        return EncoderResult::invalid_argument;
    }
    const uint32_t dp = route_.transmitter * block_stride;
    EncoderResult result = EncoderResult::success;
    if (pattern != 0) {
        // dcn10_link_encoder_set_dp_phy_pattern_training_pattern(index).
        result = write(dp_dphy_training_pattern_sel + dp, pattern - 1u);
        if (result == EncoderResult::success) {
            result = update(dp_link_cntl + dp, training_complete, 0);
        }
    } else {
        // set_dp_phy_pattern_passthrough_mode(DP_PANEL_MODE_DEFAULT).
        result = write(dp_dphy_internal_ctrl + dp, 0);
        if (result == EncoderResult::success) {
            result = update(dp_link_framing_cntl + dp,
                            idle_bs_interval | vbid_disable | enhanced_frame_mode,
                            0x2000u | enhanced_frame_mode);
        }
        if (result == EncoderResult::success) {
            result = update(dp_dphy_scram_cntl + dp, scrambler_bs_count,
                            0x1ffu << scrambler_bs_count_shift);
        }
        if (result == EncoderResult::success) {
            result = update(dp_link_cntl + dp, training_complete, training_complete);
        }
    }
    if (result == EncoderResult::success) {
        result = update(dp_dphy_cntl + dp, dphy_bypass, 0);
    }
    if (result == EncoderResult::success) {
        result = update(dp_dphy_prbs_cntl + dp, prbs_enable, 0);
    }
    if (result == EncoderResult::refused) {
        quarantined_ = true;
        result = EncoderResult::quarantined;
    }
    return result;
}

EncoderResult Dcn32LinkEncoder::enable_tmds(LinkSignal signal, uint32_t tmds_khz) noexcept {
    if (!usable()) {
        return quarantined_ ? EncoderResult::quarantined : EncoderResult::not_bound;
    }
    if (!tmds_signal(signal) || !signal_allowed(route_.signal, signal) || tmds_khz == 0 ||
        tmds_khz > maximum_pll_100hz / 10u) {
        return EncoderResult::invalid_argument;
    }
    TransmitterControl control = base_control(TransmitterAction::enable);
    control.mode_laneset = digmode_of(signal);
    control.lanes = signal == LinkSignal::dvi_dual ? 8u : 4u;
    control.symclk_10khz = tmds_khz / 10u;
    const EncoderResult result = transmitter(control);
    if (result == EncoderResult::success) {
        active_signal_ = signal;
    } else if (result == EncoderResult::refused) {
        quarantined_ = true;
        return EncoderResult::quarantined;
    }
    return result;
}

bool Dcn32LinkEncoder::dig_enabled(bool& enabled) noexcept {
    uint32_t value = 0;
    if (!bound_ || !hardware::read_ip_register(atom_.io, space_.segment2_dword_base,
                                               dig_be_en_cntl + route_.transmitter * block_stride,
                                               value) ||
        value == UINT32_MAX) {
        return false;
    }
    enabled = (value & dig_enable) != 0;
    return true;
}

EncoderResult Dcn32LinkEncoder::disable(LinkSignal signal) noexcept {
    if (!usable()) {
        return quarantined_ ? EncoderResult::quarantined : EncoderResult::not_bound;
    }
    if (!signal_allowed(route_.signal, signal)) {
        return EncoderResult::invalid_argument;
    }
    bool enabled = false;
    if (!dig_enabled(enabled)) {
        return EncoderResult::refused;
    }
    if (!enabled) {
        active_signal_ = LinkSignal::none;
        return EncoderResult::success;
    }
    TransmitterControl control = base_control(TransmitterAction::disable);
    control.mode_laneset = digmode_of(signal);
    control.connector_id = route_.connector_id;
    EncoderResult result = transmitter(control);
    if (result == EncoderResult::success && signal == LinkSignal::display_port) {
        // link_encoder_disable: pattern 0, training incomplete, default panel mode.
        const uint32_t dp = route_.transmitter * block_stride;
        result = write(dp_dphy_training_pattern_sel + dp, 0);
        if (result == EncoderResult::success) {
            result = update(dp_link_cntl + dp, training_complete, 0);
        }
        if (result == EncoderResult::success) {
            result = write(dp_dphy_internal_ctrl + dp, 0);
        }
    }
    if (result == EncoderResult::refused) {
        quarantined_ = true;
        return EncoderResult::quarantined;
    }
    if (result == EncoderResult::success) {
        active_signal_ = LinkSignal::none;
        rate_code_ = 0;
        lanes_ = 0;
    }
    return result;
}

EncoderResult Dcn32LinkEncoder::program_tmds_pll(uint32_t otg, uint32_t pll, LinkSignal signal,
                                                 uint32_t pixel_100hz, TmdsDepth depth) noexcept {
    if (!usable()) {
        return quarantined_ ? EncoderResult::quarantined : EncoderResult::not_bound;
    }
    if (otg >= 4 || pll >= dcn32_link_count || !tmds_signal(signal) ||
        !signal_allowed(route_.signal, signal) || pixel_100hz == 0 ||
        (signal != LinkSignal::hdmi && depth != TmdsDepth::bpc8)) {
        return EncoderResult::invalid_argument;
    }
    // dcn3_get_pix_clk_dividers: HDMI scales by 5/4, 6/4 or 2 for deep colour.
    uint64_t actual = pixel_100hz;
    switch (depth) {
    case TmdsDepth::bpc10: actual = actual * 5u / 4u; break;
    case TmdsDepth::bpc12: actual = actual * 6u / 4u; break;
    case TmdsDepth::bpc16: actual = actual * 2u; break;
    default: break;
    }
    if (actual > maximum_pll_100hz) {
        return EncoderResult::invalid_argument;
    }
    PixelClockControl control{};
    control.pixel_100hz = static_cast<uint32_t>(actual);
    control.pll_id = static_cast<uint8_t>(atom_combophy_pll0 + pll);
    control.encoder_object = uniphy_objects[route_.transmitter / 2u];
    control.encoder_mode = digmode_of(signal);
    control.misc = static_cast<uint8_t>(misc_ref_xtalin |
                                        (signal == LinkSignal::dvi_dual ? misc_dvi_dual_link : 0));
    control.crtc = static_cast<uint8_t>(otg);
    control.deep_color_ratio = static_cast<uint8_t>(depth);
    uint32_t dwords[4]{};
    pack_pixel_clock_control(control, dwords);
    atom_.interpreter->clear_parameters();
    for (uint32_t index = 0; index < 4; ++index) {
        atom_.interpreter->set_parameter(index, dwords[index]);
    }
    const hardware::AtomStatus status =
        atom_.interpreter->execute(atom_.io, atom_command_set_pixel_clock, 4, atom_.policy);
    atom_report_ = atom_.interpreter->report();
    if (status == hardware::AtomStatus::completed) {
        return EncoderResult::success;
    }
    if (atom_report_.register_writes != 0 || atom_report_.steps != 0) {
        quarantined_ = true;
        return EncoderResult::quarantined;
    }
    return EncoderResult::refused;
}

EncoderResult Dcn32LinkEncoder::power_down_pll(uint32_t pll) noexcept {
    if (!usable()) {
        return quarantined_ ? EncoderResult::quarantined : EncoderResult::not_bound;
    }
    if (pll >= dcn32_link_count) {
        return EncoderResult::invalid_argument;
    }
    // CONTROLLER_ID_UNDEFINED -> ATOM_CRTC_INVALID; no encoder object (NONE),
    // SIGNAL_TYPE_NONE -> ATOM_ENCODER_MODE_CRT (15), target clock 0.
    PixelClockControl control{};
    control.pll_id = static_cast<uint8_t>(atom_combophy_pll0 + pll);
    control.encoder_mode = 15;
    control.misc = misc_force_program_ppll;
    control.crtc = atom_crtc_invalid;
    uint32_t dwords[4]{};
    pack_pixel_clock_control(control, dwords);
    atom_.interpreter->clear_parameters();
    for (uint32_t index = 0; index < 4; ++index) {
        atom_.interpreter->set_parameter(index, dwords[index]);
    }
    const hardware::AtomStatus status =
        atom_.interpreter->execute(atom_.io, atom_command_set_pixel_clock, 4, atom_.policy);
    atom_report_ = atom_.interpreter->report();
    if (status == hardware::AtomStatus::completed) {
        return EncoderResult::success;
    }
    quarantined_ = atom_report_.register_writes != 0 || atom_report_.steps != 0;
    return quarantined_ ? EncoderResult::quarantined : EncoderResult::refused;
}

bool Dcn32LinkEncoder::phy_configure(void* context, uint8_t rate, uint8_t lanes,
                                     bool) noexcept {
    // Enhanced framing is a DIG FE/MSA attribute owned by the stream side.
    return static_cast<Dcn32LinkEncoder*>(context)->enable_dp(rate, lanes) ==
           EncoderResult::success;
}

bool Dcn32LinkEncoder::phy_pattern(void* context, uint8_t pattern) noexcept {
    // dp_training passes DPCD TRAINING_PATTERN_SET codes: TPS1..3 are 1..3,
    // TPS4 is 7 (DP_TRAINING_PATTERN_4); the encoder takes TPS1..4 as 1..4.
    const uint8_t tps = pattern == 7 ? 4 : pattern;
    return tps >= 1 && tps <= 4 && pattern != 4 &&
           static_cast<Dcn32LinkEncoder*>(context)->set_dp_pattern(tps) ==
               EncoderResult::success;
}

bool Dcn32LinkEncoder::phy_drive(void* context, const uint8_t* lane_settings,
                                 uint8_t lanes) noexcept {
    auto& encoder = *static_cast<Dcn32LinkEncoder*>(context);
    return encoder.set_lane_settings(encoder.rate_code_, lane_settings, lanes) ==
           EncoderResult::success;
}

bool Dcn32LinkEncoder::phy_finish(void* context, bool trained) noexcept {
    auto& encoder = *static_cast<Dcn32LinkEncoder*>(context);
    if (trained) {
        return encoder.set_dp_pattern(0) == EncoderResult::success;
    }
    return encoder.disable(LinkSignal::display_port) == EncoderResult::success;
}

bool Dcn32LinkEncoder::phy_fec_ready(void* context, bool ready) noexcept {
    return static_cast<Dcn32LinkEncoder*>(context)->fec_set_ready(ready) == EncoderResult::success;
}

DpPhyIo Dcn32LinkEncoder::make_phy_io() noexcept {
    if (!bound_ || route_.signal != LinkSignal::display_port) {
        return {};
    }
    return {this, phy_configure, phy_pattern, phy_drive, phy_finish, phy_fec_ready};
}

EncoderResult Dcn32LinkEncoder::fec_set_ready(bool ready) noexcept {
    if (!usable()) {
        return quarantined_ ? EncoderResult::quarantined : EncoderResult::not_bound;
    }
    if (route_.signal != LinkSignal::display_port) {
        return EncoderResult::invalid_argument;
    }
    const uint32_t dp = route_.transmitter * block_stride;
    return update(dp_dphy_cntl + dp, dphy_fec_ready_shadow, ready ? dphy_fec_ready_shadow : 0u);
}

EncoderResult Dcn32LinkEncoder::fec_set_enable(bool enable) noexcept {
    if (!usable()) {
        return quarantined_ ? EncoderResult::quarantined : EncoderResult::not_bound;
    }
    if (route_.signal != LinkSignal::display_port) {
        return EncoderResult::invalid_argument;
    }
    const uint32_t dp = route_.transmitter * block_stride;
    return update(dp_dphy_cntl + dp, dphy_fec_en, enable ? dphy_fec_en : 0u);
}

EncoderResult Dcn32LinkEncoder::fec_active(bool& active) noexcept {
    active = false;
    if (!usable()) {
        return quarantined_ ? EncoderResult::quarantined : EncoderResult::not_bound;
    }
    if (route_.signal != LinkSignal::display_port) {
        return EncoderResult::invalid_argument;
    }
    uint32_t value = 0;
    const uint32_t dp = route_.transmitter * block_stride;
    if (!hardware::read_ip_register(atom_.io, space_.segment2_dword_base, dp_dphy_cntl + dp, value) ||
        value == UINT32_MAX) {
        return EncoderResult::refused;
    }
    active = (value & dphy_fec_active_status) != 0;
    return EncoderResult::success;
}

} // namespace rx7900::output

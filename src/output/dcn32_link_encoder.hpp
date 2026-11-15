/*
Module: rx7900.output.dcn32_link_encoder
Purpose: Own one Navi31 DIO link encoder and its PHY for DP SST and HDMI/DVI TMDS.
Created: 2026-10-24
Architecture: Binds one validated dcn32_route to its DIG BE/DP/AUX/HPD register
blocks and runs the VBIOS DIG1TransmitterControl (v1.6) and SetPixelClock (v1.7)
command tables through the bounded hardware::AtomInterpreter on the CPU. It
backs the existing dp_training DpPhyIo contract. Follows Linux v6.12
dio/dcn32/dcn32_dio_link_encoder.c, dio/dcn10/dcn10_link_encoder.c,
bios/command_table2.c, bios/dce112/command_table_helper2_dce112.c and
dce/dce_clock_source.c (dcn31_program_pix_clk, dce110_clock_source_power_down).
Linux defaults to dmub_command_table on DCN 3.2, so DMCUB executes the same
tables there; this CPU path is the non-DMUB EXEC_BIOS_CMD_TABLE branch.
Linux AMD register provenance and notice: licenses/AMDGPU-OUTPUT.txt.
*/
#pragma once

#include "atom_interpreter.hpp"
#include "dcn32_route.hpp"
#include "dp_training.hpp"

namespace rx7900::output {

// atom_master_list_of_command_functions_v2_1 indices.
constexpr uint8_t atom_command_set_pixel_clock = 12;
constexpr uint8_t atom_command_dig1_transmitter_control = 76;

// atom_dig_transmitter_control_action.
enum class TransmitterAction : uint8_t {
    disable = 0,
    enable = 1,
    init = 7,
    setup_vsemph = 11
};

// struct dig_transmitter_control_parameters_v1_6 (16 bytes) inside the 32-byte
// ps allocation; laid out little-endian into ATOM parameter dwords.
struct TransmitterControl final {
    uint8_t phy_id = 0;
    TransmitterAction action = TransmitterAction::disable;
    uint8_t mode_laneset = 0;
    uint8_t lanes = 0;
    uint32_t symclk_10khz = 0;
    uint8_t hpd_select = 0;
    uint8_t dig_fe_select = 0;
    uint8_t connector_id = 0;
};

// struct set_pixel_clock_parameter_v1_7 (16 bytes).
struct PixelClockControl final {
    uint32_t pixel_100hz = 0;
    uint8_t pll_id = 0;
    uint8_t encoder_object = 0;
    uint8_t encoder_mode = 0;
    uint8_t misc = 0;
    uint8_t crtc = 0;
    uint8_t deep_color_ratio = 0;
};

void pack_transmitter_control(const TransmitterControl& control, uint32_t (&dwords)[8]) noexcept;
void pack_pixel_clock_control(const PixelClockControl& control, uint32_t (&dwords)[4]) noexcept;

struct AtomRunner final {
    hardware::AtomInterpreter* interpreter = nullptr;
    hardware::RegisterIo io{};
    hardware::AtomPolicy policy = hardware::atom_default_policy;
};

enum class EncoderResult : uint8_t {
    success,
    invalid_argument,
    not_bound,
    unsupported_revision,
    // The command table or a register read refused before any write.
    refused,
    // A write or a started command table may have changed hardware.
    quarantined
};

// TMDS colour depth for the PHY PLL (transmitter_color_depth_to_atom).
enum class TmdsDepth : uint8_t { bpc8 = 0, bpc10 = 1, bpc12 = 2, bpc16 = 3 };

// Device-owned link encoder of one route; PASSIVE_LEVEL, serialized by the
// display owner with AUX, HPD, DMUB, mode changes and reset. Any uncertain
// write or command table quarantines the encoder until a proven reset.
class Dcn32LinkEncoder final {
public:
    Dcn32LinkEncoder() noexcept = default;
    Dcn32LinkEncoder(const Dcn32LinkEncoder&) = delete;
    Dcn32LinkEncoder& operator=(const Dcn32LinkEncoder&) = delete;

    // Requires a validated route, a DMU 3.2.0 register space and an ATOM ROM
    // whose DIG1TransmitterControl is content revision 6 and SetPixelClock 7.
    EncoderResult bind(const display::Dcn32RegisterSpace& space,
                       const Dcn32ConnectorRoute& route, AtomRunner atom) noexcept;
    void unbind() noexcept;
    // Only after the device owner proved an ASIC reset: drops a quarantine.
    void reset_proven() noexcept;

    // enc32_hw_init: AUX_DPHY_RX_CONTROL0=0x103d1110, AUX_DPHY_TX_CONTROL=
    // 0x21c7a, TMDS_CTL_BITS.TMDS_CTL0=1, then dcn10_aux_initialize (AUX_HPD_SEL,
    // AUX_LS_READ_EN=0, AUX_RX_RECEIVE_WINDOW=0) and amdgpu_dm_hpd_init's
    // enable_hpd (DC_HPD_EN=1). No PHY is enabled.
    EncoderResult hw_init() noexcept;
    // dcn10_link_encoder_setup: DIG_BE_CNTL.DIG_MODE (DP 0, DVI 2, HDMI 3).
    EncoderResult setup(LinkSignal signal) noexcept;
    // dcn10_link_encoder_connect_dig_be_to_fe: DIG_FE_SOURCE_SELECT bit fe.
    EncoderResult connect_fe(uint8_t dig_fe, bool connect) noexcept;
    // dcn10_link_encoder_enable_dp_output: DP_UDI_LANES, DPHY_SCRAMBLER_ADVANCE,
    // then ENABLE with symclk = rate_code * 27 MHz.
    EncoderResult enable_dp(uint8_t rate_code, uint8_t lanes) noexcept;
    // dcn10_link_encoder_dp_set_lane_settings: one SETUP_VSEMPH per lane.
    EncoderResult set_lane_settings(uint8_t rate_code, const uint8_t* lane_settings,
                                    uint8_t lanes) noexcept;
    // TPS1..4 (1..4) or 0 for video/passthrough mode.
    EncoderResult set_dp_pattern(uint8_t pattern) noexcept;
    // dcn10_link_encoder_enable_tmds_output (4 lanes, 8 for dual-link DVI).
    EncoderResult enable_tmds(LinkSignal signal, uint32_t tmds_khz) noexcept;
    // dcn10_link_encoder_disable_output, then link_encoder_disable for DP.
    EncoderResult disable(LinkSignal signal) noexcept;
    // DIG_BE_EN_CNTL.DIG_ENABLE; false on a read failure.
    bool dig_enabled(bool& enabled) noexcept;

    // dcn31_program_pix_clk TMDS: SetPixelClock v1.7 on COMBO PHY PLL pll with
    // XTALIN reference and the HDMI deep-colour adjusted rate.
    EncoderResult program_tmds_pll(uint32_t otg, uint32_t pll, LinkSignal signal,
                                   uint32_t pixel_100hz, TmdsDepth depth) noexcept;
    // dce110_clock_source_power_down: SetPixelClock with target 0 and FORCE_PROG_PPLL.
    EncoderResult power_down_pll(uint32_t pll) noexcept;

    // dcn-dsc-odm: enc2_fec_set_ready (DP_DPHY_CNTL.DPHY_FEC_READY_SHADOW),
    // enc2_fec_set_enable (DPHY_FEC_EN), one REG_UPDATE each, and
    // enc2_fec_is_active (DPHY_FEC_ACTIVE_STATUS) of this route's DP block.
    EncoderResult fec_set_ready(bool ready) noexcept;
    EncoderResult fec_set_enable(bool enable) noexcept;
    EncoderResult fec_active(bool& active) noexcept;

    // DpPhyIo over this encoder for train_dp_sst; finish(true) selects video
    // mode, finish(false) disables the PHY.
    DpPhyIo make_phy_io() noexcept;

    bool bound() const noexcept { return bound_; }
    bool quarantined() const noexcept { return quarantined_; }
    const Dcn32ConnectorRoute& route() const noexcept { return route_; }
    const hardware::AtomExecReport& last_atom_report() const noexcept { return atom_report_; }

private:
    static bool phy_configure(void*, uint8_t, uint8_t, bool) noexcept;
    static bool phy_pattern(void*, uint8_t) noexcept;
    static bool phy_drive(void*, const uint8_t*, uint8_t) noexcept;
    static bool phy_finish(void*, bool) noexcept;
    static bool phy_fec_ready(void*, bool) noexcept;

    EncoderResult transmitter(const TransmitterControl& control) noexcept;
    EncoderResult update(uint32_t reg, uint32_t field, uint32_t value) noexcept;
    EncoderResult write(uint32_t reg, uint32_t value) noexcept;
    TransmitterControl base_control(TransmitterAction action) const noexcept;
    bool usable() const noexcept { return bound_ && !quarantined_; }

    display::Dcn32RegisterSpace space_{};
    Dcn32ConnectorRoute route_{};
    AtomRunner atom_{};
    hardware::AtomExecReport atom_report_{};
    LinkSignal active_signal_ = LinkSignal::none;
    uint8_t rate_code_ = 0;
    uint8_t lanes_ = 0;
    bool bound_ = false;
    bool quarantined_ = false;
};

} // namespace rx7900::output

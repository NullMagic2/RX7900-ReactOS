/*
Module: rx7900.tests.dcn32_link_encoder
Purpose: Check DIO link-encoder registers and the VBIOS PHY command-table parameters.
Created: 2026-10-24
Architecture: Synthetic ATOM images from tests/atom_test_builder.hpp whose
DIG1TransmitterControl / SetPixelClock tables copy their parameter space into
probe registers, run by the real AtomInterpreter over the shared DCN register
model. Expected values follow Linux v6.12 dcn32/dcn10 link encoder and
command_table2 (v1.6 / v1.7) field rules. No physical GPU access.
*/
#include <memory>

#include "atom_test_builder.hpp"
#include "dcn32_display_model.hpp"
#include "dcn32_link_encoder.hpp"

namespace {
using namespace rx7900::output;
using rx7900_test::Checks;
namespace dcn = rx7900_test::dcn;
namespace atom = rx7900_test::atom;

constexpr uint16_t probe_transmitter = 0x7000;
constexpr uint16_t probe_pixel = 0x7010;
constexpr uint32_t seg2 = dcn::segment2;

std::vector<uint8_t> build_rom(uint8_t transmitter_revision, uint8_t pixel_revision,
                               bool long_table = false, bool vsemph_aborts = false) {
    atom::ImageBuilder builder;
    auto& transmitter = builder.command(atom_command_dig1_transmitter_control, 0, 32);
    transmitter.content = transmitter_revision;
    atom::Code::Label abort_label{};
    if (vsemph_aborts) {
        // PS2 == hpdsel 1 | connobj 0x13 (only SETUP_VSEMPH/DISABLE carry the
        // connector id): write a probe, then jump out of the table.
        abort_label = transmitter.code.label();
        transmitter.code.compare(atom::ps(2), atom::imm32(0x00130001u));
        transmitter.code.jump(atom::Cond::equal, abort_label);
    }
    for (uint16_t index = 0; index < 3; ++index) {
        transmitter.code.move(atom::reg(static_cast<uint16_t>(probe_transmitter + index)),
                              atom::ps(static_cast<uint8_t>(index)));
    }
    if (long_table) {
        for (int index = 0; index < 8; ++index) {
            transmitter.code.nop();
        }
    }
    transmitter.code.eot();
    if (vsemph_aborts) {
        transmitter.code.place(abort_label);
        transmitter.code.move(atom::reg(static_cast<uint16_t>(probe_transmitter + 3)), atom::ps(0));
        transmitter.code.jump_raw(atom::Cond::always, 0xfff0);
    }
    auto& pixel = builder.command(atom_command_set_pixel_clock, 0, 16);
    pixel.content = pixel_revision;
    for (uint16_t index = 0; index < 3; ++index) {
        pixel.code.move(atom::reg(static_cast<uint16_t>(probe_pixel + index)),
                        atom::ps(static_cast<uint8_t>(index)));
    }
    pixel.code.eot();
    return builder.build().bytes;
}

Dcn32ConnectorRoute route(LinkSignal signal, uint8_t transmitter, uint8_t ddc, uint8_t hpd) {
    Dcn32ConnectorRoute value{};
    value.signal = signal;
    value.transmitter = transmitter;
    value.dig_engine = transmitter;
    value.ddc_line = ddc;
    value.aux_engine = ddc;
    value.hpd_source = hpd;
    value.connector_id = signal == LinkSignal::display_port ? 0x13 : 0x0c;
    value.validated = true;
    return value;
}

struct Fixture final {
    dcn::Model model;
    std::vector<uint8_t> rom;
    std::unique_ptr<rx7900::hardware::AtomInterpreter> interpreter =
        std::make_unique<rx7900::hardware::AtomInterpreter>();
    Dcn32LinkEncoder encoder;

    explicit Fixture(std::vector<uint8_t> image) : rom(std::move(image)) {
        interpreter->bind(rx7900::ByteView(rom.data(), rom.size()), 0);
    }
    AtomRunner runner(rx7900::hardware::AtomPolicy policy = rx7900::hardware::atom_default_policy) {
        return {interpreter.get(), model.io(), policy};
    }
    uint32_t probe(uint16_t base, uint32_t index) const {
        const auto found = model.values.find((base + index) * 4u);
        return found == model.values.end() ? 0xdeadbeefu : found->second;
    }
    uint32_t executions(uint16_t base) const {
        uint32_t count = 0;
        for (const auto& entry : model.writes) {
            count += entry.first == base * 4u ? 1u : 0u;
        }
        return count;
    }
};

void check_bind(Checks& checks) {
    {
        Fixture fixture(build_rom(6, 7));
        checks.expect(fixture.encoder.bind(dcn::space(), route(LinkSignal::display_port, 1, 1, 1),
                                           fixture.runner()) == EncoderResult::success,
                      "encoder binds to a validated route");
        checks.expect(fixture.encoder.bind(dcn::space(), route(LinkSignal::display_port, 1, 1, 1),
                                           fixture.runner()) == EncoderResult::invalid_argument,
                      "bind once");
    }
    {
        Fixture fixture(build_rom(7, 7));
        checks.expect(fixture.encoder.bind(dcn::space(), route(LinkSignal::display_port, 0, 0, 0),
                                           fixture.runner()) == EncoderResult::unsupported_revision,
                      "DIG1TransmitterControl v1.7 refused (HPO layout)");
    }
    {
        Fixture fixture(build_rom(6, 6));
        checks.expect(fixture.encoder.bind(dcn::space(), route(LinkSignal::hdmi, 2, 2, 2),
                                           fixture.runner()) == EncoderResult::unsupported_revision,
                      "TMDS route needs SetPixelClock v1.7");
        Dcn32LinkEncoder dp;
        checks.expect(dp.bind(dcn::space(), route(LinkSignal::display_port, 0, 0, 0),
                              fixture.runner()) == EncoderResult::success,
                      "DP route does not need SetPixelClock");
    }
    Fixture fixture(build_rom(6, 7));
    Dcn32ConnectorRoute quarantined = route(LinkSignal::display_port, 0, 0, 0);
    quarantined.validated = false;
    checks.expect(fixture.encoder.bind(dcn::space(), quarantined, fixture.runner()) ==
                      EncoderResult::invalid_argument,
                  "quarantined route refused");
    Dcn32ConnectorRoute range = route(LinkSignal::display_port, 5, 0, 0);
    checks.expect(fixture.encoder.bind(dcn::space(), range, fixture.runner()) ==
                      EncoderResult::invalid_argument,
                  "transmitter 5 refused");
    checks.expect(fixture.encoder.hw_init() == EncoderResult::not_bound, "unbound encoder refuses");
}

void check_hw_init(Checks& checks) {
    Fixture fixture(build_rom(6, 7));
    // AUX engine 1 (stride 0x1c), HPD 3 (stride 8), transmitter 2 (stride 0x100).
    fixture.encoder.bind(dcn::space(), route(LinkSignal::display_port, 2, 1, 3), fixture.runner());
    fixture.model.set(seg2, 0x1f50u + 0x1c, 0x00000101u); // AUX_CONTROL: LS_READ_EN set
    checks.expect(fixture.encoder.hw_init() == EncoderResult::success, "hw_init succeeds");
    checks.expect(fixture.model.written(seg2, 0x1f5au + 0x1c) ==
                      std::vector<uint32_t>({0x103d1110u, 0x103d1010u}),
                  "AUX_DPHY_RX_CONTROL0 literal then 1/4 receive window");
    checks.expect(fixture.model.written(seg2, 0x1f59u + 0x1c) == std::vector<uint32_t>{0x21c7au},
                  "AUX_DPHY_TX_CONTROL literal");
    checks.expect(fixture.model.get(seg2, 0x20dfu + 0x200) == 1u, "TMDS_CTL0 of DIG2");
    checks.expect(fixture.model.get(seg2, 0x1f50u + 0x1c) == (0x00000001u | (3u << 20)),
                  "AUX_HPD_SEL = 3, AUX_LS_READ_EN = 0");
    checks.expect(fixture.model.get(seg2, 0x1f16u + 3u * 8u) == 0x10000000u, "DC_HPD_EN of HPD3");
    const size_t rx = fixture.model.write_index(seg2, 0x1f5au + 0x1c);
    const size_t tx = fixture.model.write_index(seg2, 0x1f59u + 0x1c);
    const size_t tmds = fixture.model.write_index(seg2, 0x20dfu + 0x200);
    const size_t control = fixture.model.write_index(seg2, 0x1f50u + 0x1c);
    checks.expect(rx < tx && tx < tmds && tmds < control, "enc32_hw_init order");

    Fixture failing(build_rom(6, 7));
    failing.encoder.bind(dcn::space(), route(LinkSignal::display_port, 0, 0, 0), failing.runner());
    failing.model.fail_write = 1;
    checks.expect(failing.encoder.hw_init() == EncoderResult::quarantined &&
                      failing.encoder.quarantined() &&
                      failing.encoder.setup(LinkSignal::display_port) == EncoderResult::quarantined,
                  "failed init write quarantines");
}

void check_dp(Checks& checks) {
    Fixture fixture(build_rom(6, 7));
    fixture.encoder.bind(dcn::space(), route(LinkSignal::display_port, 3, 3, 2), fixture.runner());
    const uint32_t dig = 0x300u;
    fixture.model.set(seg2, 0x20b1u + dig, 0x00030000u | 0x0200u); // HDMI mode, FE1 connected
    checks.expect(fixture.encoder.setup(LinkSignal::display_port) == EncoderResult::success &&
                      fixture.model.get(seg2, 0x20b1u + dig) == 0x0200u,
                  "DIG_MODE 0 for DP, other fields kept");
    checks.expect(fixture.encoder.setup(LinkSignal::hdmi) == EncoderResult::invalid_argument,
                  "HDMI on a DP connector refused");
    checks.expect(fixture.encoder.connect_fe(3, true) == EncoderResult::success &&
                      fixture.model.get(seg2, 0x20b1u + dig) == (0x0200u | 0x0800u),
                  "connect DIG FE 3");
    checks.expect(fixture.encoder.connect_fe(1, false) == EncoderResult::success &&
                      fixture.model.get(seg2, 0x20b1u + dig) == 0x0800u,
                  "disconnect DIG FE 1");
    checks.expect(fixture.encoder.connect_fe(5, true) == EncoderResult::invalid_argument,
                  "FE 5 refused");

    fixture.model.set(seg2, 0x211eu + dig, 0x0003ff00u);
    checks.expect(fixture.encoder.enable_dp(0x14, 4) == EncoderResult::success, "HBR2 x4 enable");
    checks.expect(fixture.model.written(seg2, 0x210bu + dig) == std::vector<uint32_t>{3u},
                  "REG_SET DP_UDI_LANES = lanes - 1");
    checks.expect(fixture.model.get(seg2, 0x211eu + dig) == (0x0003ff00u | 0x10u),
                  "DPHY_SCRAMBLER_ADVANCE");
    checks.expect(fixture.probe(probe_transmitter, 0) == (3u | (1u << 8) | (0u << 16) | (4u << 24)),
                  "phyid D, ENABLE, DIGMODE DP, four lanes");
    checks.expect(fixture.probe(probe_transmitter, 1) == 54000u, "symclk 540 MHz in 10 kHz");
    checks.expect(fixture.probe(probe_transmitter, 2) == 3u, "hpdsel HPD3, digfe 0, connobj 0");
    checks.expect(fixture.model.write_index(seg2, 0x210bu + dig) <
                      fixture.model.write_index(0, probe_transmitter),
                  "configure_encoder before the VBIOS table");
    checks.expect(fixture.encoder.enable_dp(0x0b, 4) == EncoderResult::invalid_argument &&
                      fixture.encoder.enable_dp(0x14, 3) == EncoderResult::invalid_argument,
                  "invalid rate and lane count refused");

    const uint8_t lanes[4]{0x0e, 0x00, 0x1b, 0xc3};
    const uint32_t before = fixture.executions(probe_transmitter);
    checks.expect(fixture.encoder.set_lane_settings(0x14, lanes, 4) == EncoderResult::success,
                  "lane settings");
    checks.expect(fixture.executions(probe_transmitter) == before + 4u, "one SETUP_VSEMPH per lane");
    // Last lane 0xc3: swing 3, pre-emphasis 0, PC2 bits dropped.
    checks.expect(fixture.probe(probe_transmitter, 0) == (3u | (11u << 8) | (0x03u << 16) | (4u << 24)) &&
                      fixture.probe(probe_transmitter, 2) == (3u | (0x13u << 16)),
                  "SETUP_VSEMPH dplaneset, connector id");

    checks.expect(fixture.encoder.set_dp_pattern(2) == EncoderResult::success &&
                      fixture.model.get(seg2, 0x2118u + dig) == 1u &&
                      (fixture.model.get(seg2, 0x2108u + dig) & 0x10u) == 0,
                  "TPS2 selects index 1, training incomplete");
    fixture.model.set(seg2, 0x2117u + dig, 0x00010000u);
    fixture.model.set(seg2, 0x211du + dig, 1u);
    checks.expect(fixture.encoder.set_dp_pattern(0) == EncoderResult::success, "video mode");
    checks.expect(fixture.model.get(seg2, 0x2113u + dig) == (0x2000u | 0x10000000u) &&
                      (fixture.model.get(seg2, 0x211eu + dig) & 0x3ff00u) == 0x1ff00u &&
                      (fixture.model.get(seg2, 0x2108u + dig) & 0x10u) == 0x10u &&
                      fixture.model.get(seg2, 0x2117u + dig) == 0 &&
                      fixture.model.get(seg2, 0x211du + dig) == 0 &&
                      fixture.model.written(seg2, 0x210fu + dig) == std::vector<uint32_t>{0u},
                  "passthrough mode registers");
    checks.expect(fixture.encoder.set_dp_pattern(5) == EncoderResult::invalid_argument, "TPS5 refused");

    // disable: DIG_ENABLE clear -> no table; set -> DISABLE and link_encoder_disable.
    const uint32_t executed = fixture.executions(probe_transmitter);
    checks.expect(fixture.encoder.disable(LinkSignal::display_port) == EncoderResult::success &&
                      fixture.executions(probe_transmitter) == executed,
                  "disabled DIG needs no table");
    fixture.encoder.enable_dp(0x0a, 2);
    fixture.model.set(seg2, 0x20b2u + dig, 1u);
    checks.expect(fixture.encoder.disable(LinkSignal::display_port) == EncoderResult::success,
                  "disable output");
    checks.expect(fixture.probe(probe_transmitter, 0) == 3u && fixture.probe(probe_transmitter, 1) == 0 &&
                      fixture.probe(probe_transmitter, 2) == (3u | (0x13u << 16)),
                  "DISABLE parameters with connector id");
    checks.expect(fixture.model.get(seg2, 0x2118u + dig) == 0 &&
                      (fixture.model.get(seg2, 0x2108u + dig) & 0x10u) == 0,
                  "link_encoder_disable registers");
    checks.expect(fixture.encoder.make_phy_io().configure != nullptr, "DP route has PHY callbacks");
}

void check_tmds(Checks& checks) {
    Fixture fixture(build_rom(6, 7));
    fixture.encoder.bind(dcn::space(), route(LinkSignal::hdmi, 1, 2, 4), fixture.runner());
    checks.expect(fixture.encoder.make_phy_io().configure == nullptr, "HDMI route has no DP PHY io");
    checks.expect(fixture.encoder.setup(LinkSignal::hdmi) == EncoderResult::success &&
                      fixture.model.get(seg2, 0x20b1u + 0x100u) == 0x00030000u,
                  "DIG_MODE 3 for HDMI");
    checks.expect(fixture.encoder.setup(LinkSignal::dvi_single) == EncoderResult::success &&
                      fixture.model.get(seg2, 0x20b1u + 0x100u) == 0x00020000u,
                  "DVI sink on an HDMI connector");
    checks.expect(fixture.encoder.enable_tmds(LinkSignal::hdmi, 594000) == EncoderResult::success,
                  "HDMI 2.0 TMDS enable");
    checks.expect(fixture.probe(probe_transmitter, 0) == (1u | (1u << 8) | (3u << 16) | (4u << 24)) &&
                      fixture.probe(probe_transmitter, 1) == 59400u &&
                      fixture.probe(probe_transmitter, 2) == 5u,
                  "TMDS ENABLE parameters");
    checks.expect(fixture.encoder.enable_tmds(LinkSignal::dvi_dual, 300000) ==
                      EncoderResult::invalid_argument,
                  "dual-link DVI not on an HDMI connector");
    checks.expect(fixture.encoder.enable_tmds(LinkSignal::hdmi, 600001) == EncoderResult::invalid_argument,
                  "TMDS above 600 MHz refused");

    checks.expect(fixture.encoder.program_tmds_pll(2, 0, LinkSignal::hdmi, 1485000, TmdsDepth::bpc10) ==
                      EncoderResult::success,
                  "SetPixelClock for HDMI 10 bpc");
    checks.expect(fixture.probe(probe_pixel, 0) == 1856250u, "deep colour 5/4 pixel rate");
    checks.expect(fixture.probe(probe_pixel, 1) == (20u | (0x1eu << 8) | (3u << 16)),
                  "COMBOPHY_PLL0, UNIPHY object, HDMI mode, XTALIN reference");
    checks.expect(fixture.probe(probe_pixel, 2) == (2u | (1u << 8)), "CRTC3, deep colour 5:4");
    checks.expect(fixture.encoder.program_tmds_pll(0, 0, LinkSignal::dvi_single, 1485000,
                                                   TmdsDepth::bpc10) == EncoderResult::invalid_argument,
                  "DVI is 8 bpc only");
    checks.expect(fixture.encoder.program_tmds_pll(0, 0, LinkSignal::hdmi, 5000000, TmdsDepth::bpc16) ==
                      EncoderResult::invalid_argument,
                  "PLL above 600 MHz refused");
    checks.expect(fixture.encoder.power_down_pll(1) == EncoderResult::success &&
                      fixture.probe(probe_pixel, 0) == 0 &&
                      fixture.probe(probe_pixel, 1) == (21u | (15u << 16) | (1u << 24)) &&
                      fixture.probe(probe_pixel, 2) == 0xffu,
                  "dce110_clock_source_power_down parameters");

    Fixture dvi(build_rom(6, 7));
    Dcn32ConnectorRoute dual = route(LinkSignal::dvi_dual, 4, 4, 0);
    dual.connector_id = 0x04;
    dvi.encoder.bind(dcn::space(), dual, dvi.runner());
    checks.expect(dvi.encoder.enable_tmds(LinkSignal::dvi_dual, 270000) == EncoderResult::success &&
                      (dvi.probe(probe_transmitter, 0) >> 24) == 8u &&
                      ((dvi.probe(probe_transmitter, 0) >> 16) & 0xffu) == 2u,
                  "dual-link DVI: eight lanes, DIGMODE DVI");
    checks.expect(dvi.encoder.program_tmds_pll(0, 3, LinkSignal::dvi_dual, 2700000, TmdsDepth::bpc8) ==
                      EncoderResult::success &&
                      (dvi.probe(probe_pixel, 1) >> 24) == 0x08u &&
                      ((dvi.probe(probe_pixel, 1) >> 8) & 0xffu) == 0x21u,
                  "DVI dual-link misc bit and UNIPHY2 object");
}

void check_atom_faults(Checks& checks) {
    Fixture fixture(build_rom(6, 7, true));
    rx7900::hardware::AtomPolicy policy = rx7900::hardware::atom_default_policy;
    policy.max_steps = 2; // the table writes registers, then exceeds its budget
    fixture.encoder.bind(dcn::space(), route(LinkSignal::display_port, 0, 0, 0), fixture.runner(policy));
    checks.expect(fixture.encoder.enable_dp(0x0a, 1) == EncoderResult::quarantined &&
                      fixture.encoder.quarantined(),
                  "aborted table after register writes quarantines");
    checks.expect(fixture.encoder.last_atom_report().status == rx7900::hardware::AtomStatus::step_budget,
                  "ATOM report kept");
    {
        Fixture tmds(build_rom(6, 7, true));
        tmds.encoder.bind(dcn::space(), route(LinkSignal::hdmi, 1, 1, 1), tmds.runner(policy));
        checks.expect(tmds.encoder.enable_tmds(LinkSignal::hdmi, 297000) == EncoderResult::quarantined &&
                          tmds.encoder.quarantined(),
                      "aborted TMDS enable table quarantines");
    }
    checks.expect(fixture.encoder.disable(LinkSignal::display_port) == EncoderResult::quarantined,
                  "quarantined encoder refuses");
    {
        // SETUP_VSEMPH writes a register, then leaves its table: quarantine.
        Fixture lanes(build_rom(6, 7, false, true));
        lanes.encoder.bind(dcn::space(), route(LinkSignal::display_port, 0, 0, 0), lanes.runner());
        const uint8_t setting[1]{0};
        checks.expect(lanes.encoder.enable_dp(0x0a, 1) == EncoderResult::success &&
                          lanes.encoder.set_lane_settings(0x0a, setting, 1) == EncoderResult::quarantined &&
                          lanes.encoder.quarantined() && lanes.probe(probe_transmitter, 3) != 0xdeadbeefu,
                      "aborted SETUP_VSEMPH after a register write quarantines");
    }
    fixture.encoder.unbind();
    checks.expect(fixture.encoder.bound(), "quarantined encoder cannot be unbound");
    fixture.encoder.reset_proven();
    checks.expect(!fixture.encoder.bound() && !fixture.encoder.quarantined(), "reset proof clears it");

    uint32_t transmitter[8]{};
    TransmitterControl control{};
    control.phy_id = 4;
    control.action = TransmitterAction::setup_vsemph;
    control.mode_laneset = 0x1b;
    control.lanes = 2;
    control.symclk_10khz = 0x12345678u;
    control.hpd_select = 6;
    control.dig_fe_select = 0x40;
    control.connector_id = 0x13;
    pack_transmitter_control(control, transmitter);
    checks.expect(transmitter[0] == 0x021b0b04u && transmitter[1] == 0x12345678u &&
                      transmitter[2] == 0x00134006u && transmitter[3] == 0 && transmitter[7] == 0,
                  "dig_transmitter_control_parameters_v1_6 byte layout");
    uint32_t pixel[4]{1, 1, 1, 1};
    PixelClockControl clock{};
    clock.pixel_100hz = 0xabcdef01u;
    clock.pll_id = 24;
    clock.encoder_object = 0x21;
    clock.encoder_mode = 2;
    clock.misc = 0x09;
    clock.crtc = 3;
    clock.deep_color_ratio = 3;
    pack_pixel_clock_control(clock, pixel);
    checks.expect(pixel[0] == 0xabcdef01u && pixel[1] == 0x09022118u && pixel[2] == 0x0303u &&
                      pixel[3] == 0,
                  "set_pixel_clock_parameter_v1_7 byte layout");
}

// dcn-dsc-odm: enc2_fec_set_ready / enc2_fec_set_enable / enc2_fec_is_active
// on DP_DPHY_CNTL (dcn_3_2_0: regDP0_DP_DPHY_CNTL 0x2117, 0x100 per DP block;
// DPHY_FEC_EN 0x10, DPHY_FEC_READY_SHADOW 0x20, DPHY_FEC_ACTIVE_STATUS 0x40).
void check_fec(Checks& checks) {
    Fixture fixture(build_rom(6, 7));
    fixture.encoder.bind(dcn::space(), route(LinkSignal::display_port, 2, 2, 1), fixture.runner());
    const uint32_t dphy = 0x2117u + 0x200u;
    fixture.model.set(seg2, dphy, 0x00010000u);
    checks.expect(fixture.encoder.fec_set_ready(true) == EncoderResult::success &&
                      fixture.model.written(seg2, dphy) == std::vector<uint32_t>{0x00010020u},
                  "FEC ready: one REG_UPDATE of DPHY_FEC_READY_SHADOW, other fields kept");
    checks.expect(fixture.encoder.fec_set_enable(true) == EncoderResult::success &&
                      fixture.model.get(seg2, dphy) == 0x00010030u,
                  "FEC enable: DPHY_FEC_EN");
    bool active = true;
    checks.expect(fixture.encoder.fec_active(active) == EncoderResult::success && !active,
                  "FEC not active until the hardware reports it");
    fixture.model.set(seg2, dphy, 0x00010070u);
    checks.expect(fixture.encoder.fec_active(active) == EncoderResult::success && active,
                  "DPHY_FEC_ACTIVE_STATUS reads back");
    checks.expect(fixture.encoder.fec_set_enable(false) == EncoderResult::success &&
                      fixture.model.get(seg2, dphy) == 0x00010060u &&
                      fixture.encoder.fec_set_ready(false) == EncoderResult::success &&
                      fixture.model.get(seg2, dphy) == 0x00010040u,
                  "FEC disable then not ready clear only their bits");
    const DpPhyIo io = fixture.encoder.make_phy_io();
    checks.expect(io.fec_ready != nullptr && io.fec_ready(io.context, true) &&
                      (fixture.model.get(seg2, dphy) & 0x20u) == 0x20u,
                  "the training PHY callback sets FEC_READY on this route's DP block");
    fixture.model.fail_read_offset = dcn::byte_offset(seg2, dphy);
    const size_t writes = fixture.model.writes.size();
    checks.expect(fixture.encoder.fec_set_enable(true) == EncoderResult::refused &&
                      fixture.encoder.fec_active(active) == EncoderResult::refused && !active &&
                      fixture.model.writes.size() == writes,
                  "an unreadable DP_DPHY_CNTL refuses without a write");
    fixture.model.fail_read_offset = UINT32_MAX;

    Fixture hdmi(build_rom(6, 7));
    hdmi.encoder.bind(dcn::space(), route(LinkSignal::hdmi, 1, 1, 1), hdmi.runner());
    checks.expect(hdmi.encoder.fec_set_ready(true) == EncoderResult::invalid_argument &&
                      hdmi.encoder.fec_set_enable(true) == EncoderResult::invalid_argument &&
                      hdmi.encoder.make_phy_io().fec_ready == nullptr && hdmi.model.writes.empty(),
                  "FEC is DP only");
    Fixture unbound(build_rom(6, 7));
    checks.expect(unbound.encoder.fec_set_ready(true) == EncoderResult::not_bound &&
                      unbound.encoder.fec_active(active) == EncoderResult::not_bound,
                  "unbound encoder refuses FEC");
}

} // namespace

int main() {
    Checks checks;
    check_bind(checks);
    check_hw_init(checks);
    check_dp(checks);
    check_tmds(checks);
    check_atom_faults(checks);
    check_fec(checks);
    return checks.finish("dcn32_link_encoder");
}

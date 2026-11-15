/*
Module: rx7900.tests.display_link
Purpose: Check the DCN3.2 link owner: leases, proofs, clocks, DP training and TMDS order.
Created: 2026-10-24
Architecture: The real route/encoder/clock/I2C modules and AtomInterpreter over
the shared DCN register model, a synthetic PMFW DAL mailbox, probe-register
VBIOS tables and a DPCD sink; Linux v6.12 link_set_dpms_on/off order. No
physical GPU access.
*/
#include <memory>

#include "atom_test_builder.hpp"
#include "dcn32_display_model.hpp"
#include "display_link.hpp"

namespace {
using namespace rx7900::output;
using rx7900_test::Checks;
namespace dcn = rx7900_test::dcn;
namespace atom = rx7900_test::atom;
namespace display = rx7900::display;

constexpr uint32_t seg2 = dcn::segment2;
constexpr uint16_t probe_transmitter = 0x7000;
constexpr uint16_t probe_pixel = 0x7010;
constexpr uint64_t epoch = 7;

std::vector<uint8_t> build_rom() {
    atom::ImageBuilder builder;
    auto& transmitter = builder.command(atom_command_dig1_transmitter_control, 0, 32);
    transmitter.content = 6;
    auto& pixel = builder.command(atom_command_set_pixel_clock, 0, 16);
    pixel.content = 7;
    for (uint16_t index = 0; index < 3; ++index) {
        transmitter.code.move(atom::reg(static_cast<uint16_t>(probe_transmitter + index)),
                              atom::ps(static_cast<uint8_t>(index)));
        pixel.code.move(atom::reg(static_cast<uint16_t>(probe_pixel + index)),
                        atom::ps(static_cast<uint8_t>(index)));
    }
    transmitter.code.eot();
    pixel.code.eot();
    return builder.build().bytes;
}

Connector make(uint16_t connector, uint16_t encoder, uint8_t ddc, uint8_t hpd_shift, uint8_t path) {
    Connector value{};
    value.connector_object_id = connector;
    value.encoder_object_id = encoder;
    value.firmware_path_index = path;
    value.has_i2c_record = true;
    value.ddc_gpio = {seg2 + 0x28d1u + 4u * ddc, 0, 0, true};
    value.has_hpd_record = true;
    value.hpd_gpio = {seg2 + 0x28f5u, hpd_shift, hpd_shift, true};
    return value;
}

// PMFW DAL mailbox: every message succeeds; DPM queries answer two levels.
void install_smu(dcn::Model& model) {
    model.set(dcn::mp1_segment0, 0x274u, 1);
    model.on_write = [](dcn::Model& m, uint32_t offset, uint32_t value) {
        if (offset != dcn::byte_offset(dcn::mp1_segment0, 0x28au)) {
            return;
        }
        const uint32_t argument = m.get(dcn::mp1_segment0, 0x273u);
        uint32_t reply = 0;
        if (value == display::dalsmc::get_message_header_version) {
            reply = 1;
        } else if (value == display::dalsmc::get_dpm_frequency_by_index) {
            const uint32_t index = argument & 0xffffu;
            reply = index == 0xffu ? 2u : (index == 0 ? 400u : 1200u);
        } else if (value == display::dalsmc::set_hard_min_by_frequency) {
            reply = argument & 0xffffu;
        }
        m.set(dcn::mp1_segment0, 0x273u, reply);
        m.set(dcn::mp1_segment0, 0x274u, 1);
    };
}

struct Sink final {
    bool hpd = true;
    bool clock_fails = false;
    uint8_t pattern = 0;
    uint8_t rate = 0;
    uint8_t lanes = 0;
    uint32_t reads = 0;
    // dcn-dsc-odm: DP_FEC_CONFIGURATION writes.
    uint8_t fec = 0;
    uint32_t fec_writes = 0;
};

bool sink_transfer(void* context, const AuxRequest& request, AuxReply& reply) noexcept {
    auto& sink = *static_cast<Sink*>(context);
    if (request.operation == AuxOperation::native_write) {
        if (request.address == 0x100 && request.bytes == 2) {
            sink.rate = request.write_data[0];
            sink.lanes = request.write_data[1];
        } else if (request.address == 0x102 && request.bytes == 1) {
            sink.pattern = request.write_data[0];
        } else if (request.address == 0x120 && request.bytes == 1) {
            sink.fec = request.write_data[0];
            ++sink.fec_writes;
        } else if (request.address != 0x103) {
            return false;
        }
        reply = {AuxResult::ack, request.bytes};
        return true;
    }
    if (request.operation == AuxOperation::native_read && request.address == 0x202 &&
        request.bytes == 6) {
        ++sink.reads;
        const bool clock = (sink.pattern & 0x0fu) == 1u;
        for (uint8_t index = 0; index < 6; ++index) {
            request.read_data[index] = 0;
        }
        if (!sink.clock_fails) {
            request.read_data[0] = clock ? 0x11 : 0x77;
            request.read_data[1] = clock ? 0x11 : 0x77;
            request.read_data[2] = clock ? 0 : 1;
        }
        reply = {AuxResult::ack, 6};
        return true;
    }
    return false;
}

bool sink_wait(void*, uint32_t) noexcept { return true; }
bool sink_connected(void* context) noexcept { return static_cast<Sink*>(context)->hpd; }

DpPortSnapshot port_snapshot() {
    DpPortSnapshot port{};
    port.receiver = {0x14, 0x14, 4, true, true, 0, false};
    port.sink.digital_input = true;
    port.sink.bits_per_channel = 8;
    const rx7900::display::Timing timing{148500, 1920, 2200, 88, 44, 1080, 1125, 4, 5,
                                         rx7900::display::positive_hsync |
                                             rx7900::display::positive_vsync};
    uint32_t index = 0;
    port.sink.detailed_modes.append(timing, index);
    dp_link_budget(0x14, 4, true, port.link);
    return port;
}

struct Fixture final {
    dcn::Model model;
    std::vector<uint8_t> rom = build_rom();
    std::unique_ptr<rx7900::hardware::AtomInterpreter> interpreter =
        std::make_unique<rx7900::hardware::AtomInterpreter>();
    ConnectorCatalog catalog{};
    std::unique_ptr<DisplayLinkOwner> owner = std::make_unique<DisplayLinkOwner>();

    Fixture() {
        interpreter->bind(rx7900::ByteView(rom.data(), rom.size()), 0);
        install_smu(model);
        model.set(seg2, 0x0200u, 1u);      // DMCUB held in reset (never started)
        model.set(dcn::segment1, 0x007bu, 0x264u);
        catalog.connectors[0] = make(0x3113, 0x211e, 0, 0, 0);   // DP, UNIPHY A, DDC1, HPD1
        catalog.connectors[1] = make(0x310c, 0x2120, 2, 16, 2);  // HDMI, UNIPHY C, DDC3, HPD3
        catalog.connectors[2] = make(0x3117, 0x2221, 3, 24, 5);  // USB-C: quarantined
        catalog.connectors[3] = make(0x320c, 0x2220, 4, 26, 6);  // HDMI, UNIPHY D, DDC5, HPD5
        catalog.count = 4;
    }
    DisplayLinkConfig config() {
        DisplayLinkConfig value{};
        value.io = model.io();
        value.dcn = dcn::space();
        value.dccg_segment = dcn::segment1;
        value.mp1_segment0 = dcn::mp1_segment0;
        value.xtalin_khz = 100000;
        value.catalog = &catalog;
        value.atom = interpreter.get();
        value.dp_source = dcn32_default_dp_source();
        value.epoch = epoch;
        return value;
    }
    uint32_t probe(uint16_t base, uint32_t index) const {
        const auto found = model.values.find((base + index) * 4u);
        return found == model.values.end() ? 0xdeadbeefu : found->second;
    }
};

void check_leases(Checks& checks) {
    Fixture f;
    DisplayLinkOwner& owner = *f.owner;
    RouteLease lease{};
    checks.expect(owner.acquire_route(0, epoch, lease) == LinkResult::not_bound, "unbound owner refuses");
    checks.expect(owner.bind(f.config()) == LinkResult::success, "owner binds");
    checks.expect(owner.routes().count == 4 && owner.routes().validated_count == 3,
                  "three validated routes, USB-C quarantined");
    checks.expect(owner.clock_table() != nullptr && owner.clock_table()->dcfclk.count == 2 &&
                      !owner.clock_table()->memory_info_valid,
                  "clock table published, memory info pending");
    owner.set_memory_info(24, 2, 9);
    checks.expect(owner.clock_table()->memory_info_valid && owner.clock_table()->num_channels == 24,
                  "memory info recorded");
    checks.expect(f.model.written(seg2, 0x1f5au).empty(), "bind runs no encoder init");
    checks.expect(owner.acquire_route(5, epoch, lease) == LinkResult::route_unavailable,
                  "quarantined route cannot be leased");
    checks.expect(owner.acquire_route(0, epoch + 1, lease) == LinkResult::stale_lease, "wrong epoch");
    checks.expect(owner.acquire_route(0, epoch, lease) == LinkResult::success && lease.route == 0,
                  "DP route leased");
    checks.expect(f.model.written(seg2, 0x1f5au).size() == 2, "first lease runs enc32_hw_init");
    RouteLease second{};
    checks.expect(owner.acquire_route(0, epoch, second) == LinkResult::route_unavailable,
                  "route leased once");

    checks.expect(owner.hpd_proof(epoch, 0, seg2, dcn::aperture), "HPD proof for the leased pin");
    checks.expect(!owner.hpd_proof(epoch, 2, seg2, dcn::aperture), "no proof for an unleased pin");
    checks.expect(!owner.hpd_proof(epoch + 1, 0, seg2, dcn::aperture), "no proof for another epoch");
    checks.expect(!owner.hpd_proof(epoch, 0, seg2 + 4, dcn::aperture), "no proof for another segment");
    checks.expect(!owner.aux_proof(epoch, 0, 0, seg2, dcn::aperture), "no AUX proof before the pad opens");
    checks.expect(owner.prepare_aux(lease) == LinkResult::success, "AUX pad opens");
    checks.expect(owner.aux_proof(epoch, 0, 0, seg2, dcn::aperture), "AUX proof after the pad opens");
    checks.expect(!owner.aux_proof(epoch, 1, 0, seg2, dcn::aperture), "wrong AUX engine refused");
    f.model.set(seg2, 0x0200u, 0u);
    checks.expect(!owner.aux_proof(epoch, 0, 0, seg2, dcn::aperture),
                  "unowned running DMCUB defeats the AUX proof");
    f.model.set(seg2, 0x0200u, 1u);
    f.model.set(seg2, 0x28d0u, f.model.get(seg2, 0x28d0u) | 0x100u);
    checks.expect(!owner.aux_proof(epoch, 0, 0, seg2, dcn::aperture),
                  "pad changed behind the owner defeats the AUX proof");
    f.model.set(seg2, 0x28d0u, f.model.get(seg2, 0x28d0u) & ~0x100u);

    RouteLease copy = lease;
    checks.expect(owner.release_route(lease) == LinkResult::success, "lease released");
    checks.expect(owner.release_route(copy) == LinkResult::stale_lease, "released lease is stale");
    checks.expect(!owner.hpd_proof(epoch, 0, seg2, dcn::aperture), "released route loses its proof");
    checks.expect((f.model.get(seg2, 0x28d0u) & 0x101u) == 0, "pad restored on release (boot masks 0)");
    checks.expect(owner.acquire_route(0, epoch, lease) == LinkResult::success &&
                      lease.generation == copy.generation + 1u &&
                      f.model.written(seg2, 0x1f5au).size() == 2,
                  "re-lease bumps the generation, no second hw_init");

    owner.reset_proven(epoch);
    checks.expect(!owner.bound() && owner.release_route(lease) == LinkResult::stale_lease,
                  "reset proof retires every lease");
    checks.expect(owner.bind(f.config()) == LinkResult::success, "rebinds after the reset proof");
}

void check_dp(Checks& checks) {
    Fixture f;
    DisplayLinkOwner& owner = *f.owner;
    owner.bind(f.config());
    RouteLease lease{};
    owner.acquire_route(0, epoch, lease);
    display::DisplayClocks clocks{};
    clocks.dispclk_khz = 600000;
    clocks.dppclk_khz = 600000;
    clocks.dpp_pipe_khz[0] = 600000;
    clocks.dcfclk_khz = 400000;
    clocks.display_count = 1;
    display::DisplayClocks applied{};
    checks.expect(owner.set_display_clocks(clocks, false, applied) == LinkResult::success &&
                      applied.dispclk_khz != 0,
                  "display clocks raised");
    checks.expect(owner.set_pixel_clock(lease, 0, 1485000, TmdsKind::hdmi, TmdsDepth::bpc8, 0, 0) ==
                      LinkResult::success &&
                      f.model.get(dcn::segment1, 0x0081u) == 148500000u,
                  "DP pixel clock through the DP DTO");

    Sink sink;
    const AuxIo aux{&sink, sink_transfer, sink_wait, sink_connected};
    const DpPortSnapshot port = port_snapshot();
    DpTrainedLink trained{};
    const DpTrainingRequest request{0x14, 4, 0};
    checks.expect(owner.enable_dp_link(lease, aux, port, request, 1, trained) == LinkResult::not_ready,
                  "training needs the AUX pad");
    owner.prepare_aux(lease);
    checks.expect(owner.enable_dp_link(lease, aux, port, request, 1, trained) == LinkResult::success,
                  "DP link trains");
    checks.expect(trained.rate_code == 0x14 && trained.lanes == 4 && trained.enhanced_framing,
                  "trained link settings returned");
    checks.expect(sink.rate == 0x14 && sink.lanes == (4u | 0x80u) && sink.pattern == 0,
                  "sink link settings and pattern off");
    const size_t mode = f.model.write_index(seg2, 0x20b1u);
    const size_t transmitter = f.model.write_index(0, probe_transmitter);
    checks.expect(mode < transmitter && (f.model.get(seg2, 0x20b1u) & 0x7f00u) == 0x0200u,
                  "DIG_MODE and FE1 connect before the PHY enable");
    checks.expect((f.model.get(seg2, 0x2108u) & 0x10u) == 0x10u, "link left in video mode");
    checks.expect(owner.enable_dp_link(lease, aux, port, request, 1, trained) ==
                      LinkResult::invalid_argument,
                  "an enabled link is not re-enabled");

    f.model.set(seg2, 0x20b2u, 1u); // DIG_ENABLE reads back set
    checks.expect(owner.disable_link(lease) == LinkResult::success, "DP link disabled");
    checks.expect((f.probe(probe_transmitter, 0) & 0xff00u) == 0, "PHY DISABLE table ran");
    checks.expect((f.model.get(seg2, 0x20b1u) & 0x7f00u) == 0, "DIG FE disconnected");
    checks.expect(!owner.aux_proof(epoch, 0, 0, seg2, dcn::aperture), "AUX pad closed on disable");

    // A sink that never recovers clock: training fails, FE disconnected, no quarantine.
    f.model.set(seg2, 0x20b2u, 0u);
    Sink failing;
    failing.clock_fails = true;
    const AuxIo bad{&failing, sink_transfer, sink_wait, sink_connected};
    owner.prepare_aux(lease);
    checks.expect(owner.enable_dp_link(lease, bad, port, request, 2, trained) ==
                      LinkResult::training_failed &&
                      !owner.quarantined() && (f.model.get(seg2, 0x20b1u) & 0x7f00u) == 0 &&
                      trained.lanes == 0,
                  "clock recovery failure cleans up");
    checks.expect(owner.clear_pixel_clock(lease, 0) == LinkResult::success &&
                      (f.model.get(dcn::segment1, 0x0080u) & 0x10u) == 0,
                  "DP DTO cleared");

    // HBR3 needs TPS4: DPCD code 7 reaches the encoder as pattern index 3.
    DpPortSnapshot hbr3 = port_snapshot();
    hbr3.receiver.maximum_rate_code = 0x1e;
    hbr3.receiver.training_pattern_4 = true;
    dp_link_budget(0x1e, 4, true, hbr3.link);
    Sink fast;
    const AuxIo fast_aux{&fast, sink_transfer, sink_wait, sink_connected};
    owner.prepare_aux(lease);
    f.model.writes.clear();
    checks.expect(owner.enable_dp_link(lease, fast_aux, hbr3, {0x1e, 4, 0}, 1, trained) ==
                      LinkResult::success &&
                      trained.rate_code == 0x1e &&
                      f.model.written(seg2, 0x2118u) == std::vector<uint32_t>({0u, 3u}),
                  "HBR3 trains with TPS1 then TPS4");
    f.model.set(seg2, 0x20b2u, 1u);
    owner.disable_link(lease);
    checks.expect(owner.release_route(lease) == LinkResult::success, "DP route released");
    checks.expect(owner.unbind() == LinkResult::success, "owner unbinds");
}

// dcn-dsc-odm: FEC for a DSC stream (enable_link_dp -> dp_set_fec_enable,
// disable_link_dp -> dp_set_fec_enable(false), dp_set_fec_ready(false)).
void check_fec(Checks& checks) {
    Fixture f;
    DisplayLinkOwner& owner = *f.owner;
    owner.bind(f.config());
    RouteLease lease{};
    owner.acquire_route(0, epoch, lease);
    DpPortSnapshot hbr3 = port_snapshot();
    hbr3.receiver.maximum_rate_code = 0x1e;
    hbr3.receiver.training_pattern_4 = true;
    dp_link_budget(0x1e, 4, true, hbr3.link);
    DpTrainingRequest request{0x1e, 4, 0};
    request.fec_ready = true;
    request.dsc_bits_per_pixel_x16 = 256;
    Sink sink;
    const AuxIo aux{&sink, sink_transfer, sink_wait, sink_connected};
    DpTrainedLink trained{};
    owner.prepare_aux(lease);
    f.model.writes.clear();
    const uint32_t delays = f.model.delays;
    checks.expect(owner.enable_dp_link(lease, aux, hbr3, request, 1, trained) == LinkResult::success &&
                      trained.rate_code == 0x1e,
                  "HBR3 trains with FEC for a DSC stream");
    const std::vector<uint32_t> dphy = f.model.written(seg2, 0x2117u);
    const size_t ready_at = f.model.write_index(seg2, 0x2117u);
    const size_t bw_at = f.model.write_index(seg2, 0x2118u);
    size_t enable_at = SIZE_MAX;
    for (size_t i = 0; i < f.model.writes.size(); ++i) {
        if (f.model.writes[i].first == dcn::byte_offset(seg2, 0x2117u) && (f.model.writes[i].second & 0x10u) != 0) {
            enable_at = i;
            break;
        }
    }
    size_t last_pattern = 0;
    for (size_t i = 0; i < f.model.writes.size(); ++i) {
        if (f.model.writes[i].first == dcn::byte_offset(seg2, 0x2118u)) {
            last_pattern = i;
        }
    }
    checks.expect(sink.fec == 1 && sink.fec_writes == 1 && !dphy.empty() && (dphy.front() & 0x30u) == 0x20u &&
                      ready_at < bw_at && enable_at != SIZE_MAX && enable_at > last_pattern &&
                      (f.model.get(seg2, 0x2117u) & 0x30u) == 0x30u && f.model.delays > delays,
                  "FEC_READY on sink and encoder before training; FEC_EN after it (7 us of LL codes)");
    f.model.set(seg2, 0x20b2u, 1u);
    f.model.writes.clear();
    checks.expect(owner.disable_link(lease) == LinkResult::success && sink.fec == 0 && sink.fec_writes == 2 &&
                      (f.model.get(seg2, 0x2117u) & 0x30u) == 0,
                  "disable clears FEC_EN, then FEC_READY on sink and encoder");
    const std::vector<uint32_t> cleared = f.model.written(seg2, 0x2117u);
    const size_t phy_off = f.model.write_index(0, probe_transmitter);
    const size_t fec_off = f.model.write_index(seg2, 0x2117u);
    const size_t fe_off = f.model.write_index(seg2, 0x20b1u);
    checks.expect(cleared.size() >= 2 && (cleared[cleared.size() - 2] & 0x30u) == 0x20u &&
                      (cleared.back() & 0x30u) == 0 && phy_off < fec_off && fec_off < fe_off,
                  "disable_link_dp order: PHY off, FEC disable, FEC not ready, FE disconnect");

    // Training that fails with FEC ready leaves neither end FEC-ready.
    f.model.set(seg2, 0x20b2u, 0u);
    Sink failing;
    failing.clock_fails = true;
    const AuxIo bad{&failing, sink_transfer, sink_wait, sink_connected};
    owner.prepare_aux(lease);
    checks.expect(owner.enable_dp_link(lease, bad, hbr3, request, 2, trained) == LinkResult::training_failed &&
                      failing.fec == 0 && failing.fec_writes == 2 && (f.model.get(seg2, 0x2117u) & 0x30u) == 0 &&
                      !owner.quarantined(),
                  "failed FEC training clears FEC_READY on both ends");
    // A request without FEC never touches FEC_CONFIGURATION or the encoder FEC.
    Sink plain;
    const AuxIo plain_aux{&plain, sink_transfer, sink_wait, sink_connected};
    owner.prepare_aux(lease);
    f.model.writes.clear();
    const bool plain_trained = owner.enable_dp_link(lease, plain_aux, hbr3, {0x1e, 4, 0}, 1, trained) ==
                               LinkResult::success;
    bool fec_bits = false;
    for (const uint32_t value : f.model.written(seg2, 0x2117u)) {
        fec_bits = fec_bits || (value & 0x30u) != 0;
    }
    checks.expect(plain_trained && plain.fec_writes == 0 && !fec_bits,
                  "no FEC without a DSC request");
    f.model.set(seg2, 0x20b2u, 1u);
    checks.expect(owner.disable_link(lease) == LinkResult::success && plain.fec_writes == 0, "plain disable");
    owner.release_route(lease);
    owner.unbind();
}

void check_tmds(Checks& checks) {
    Fixture f;
    DisplayLinkOwner& owner = *f.owner;
    owner.bind(f.config());
    RouteLease lease{};
    checks.expect(owner.acquire_route(2, epoch, lease) == LinkResult::success, "HDMI route leased");
    ScdcReport scdc{};
    checks.expect(owner.enable_tmds_link(lease, 148500, 3, TmdsKind::dvi_single, false, scdc) ==
                      LinkResult::not_ready,
                  "TMDS needs the PHY PLL first");
    checks.expect(owner.set_pixel_clock(lease, 1, 1485000, TmdsKind::hdmi, TmdsDepth::bpc12, 0, 1) ==
                      LinkResult::success,
                  "HDMI pixel clock");
    checks.expect(f.probe(probe_pixel, 0) == 2227500u &&
                      (f.probe(probe_pixel, 1) & 0xffu) == 20u &&
                      ((f.probe(probe_pixel, 1) >> 8) & 0xffu) == 0x20u &&
                      (f.probe(probe_pixel, 2) & 0xffu) == 1u,
                  "SetPixelClock on the first free PLL for UNIPHY C, CRTC2, 12 bpc");
    checks.expect(f.model.get(dcn::segment1, 0x0040u) == 0x20u &&
                      f.model.get(dcn::segment1, 0x006fu) == (2u << 3),
                  "PHYPLLA resync and OTG1 divider");

    // HDMI SCDC through an engine owned by hardware: Linux continues below 340 MHz.
    f.model.set(seg2, 0x1e9cu + 2u, 2u);
    checks.expect(owner.enable_tmds_link(lease, 594000, 3, TmdsKind::hdmi, false, scdc) ==
                      LinkResult::sink_failed &&
                      (f.model.get(seg2, 0x20b1u + 0x200u) & 0x7f00u) == 0,
                  "HDMI 2.0 without SCDC is refused and FE undone");
    checks.expect(owner.enable_tmds_link(lease, 297000, 3, TmdsKind::hdmi, false, scdc) ==
                      LinkResult::success,
                  "HDMI 1.4 rate proceeds without SCDC");
    checks.expect(((f.probe(probe_transmitter, 0) >> 16) & 0xffu) == 3u &&
                      f.probe(probe_transmitter, 1) == 29700u &&
                      (f.probe(probe_transmitter, 2) & 0xffu) == 3u,
                  "TMDS enable on HPD3");
    checks.expect((f.model.get(seg2, 0x20b1u + 0x200u) & 0x77f00u) == (0x30000u | 0x0800u),
                  "DIG_MODE HDMI and FE3");
    f.model.set(seg2, 0x20b2u + 0x200u, 1u);
    checks.expect(owner.disable_link(lease) == LinkResult::success, "TMDS disabled");
    checks.expect(owner.clear_pixel_clock(lease, 1) == LinkResult::success &&
                      f.probe(probe_pixel, 0) == 0 && (f.probe(probe_pixel, 1) >> 24) == 1u,
                  "PLL powered down and freed");
    checks.expect(owner.enable_tmds_link(lease, 200000, 3, TmdsKind::dvi_single, false, scdc) ==
                      LinkResult::invalid_argument,
                  "single-link DVI above 165 MHz refused");

    // Two TMDS streams: dc_resource_find_first_free_pll gives the second PLL1.
    owner.set_pixel_clock(lease, 1, 1485000, TmdsKind::hdmi, TmdsDepth::bpc8, 0, 0);
    RouteLease other{};
    checks.expect(owner.acquire_route(6, epoch, other) == LinkResult::success &&
                      owner.set_pixel_clock(other, 2, 1485000, TmdsKind::hdmi, TmdsDepth::bpc8, 0, 0) ==
                          LinkResult::success &&
                      (f.probe(probe_pixel, 1) & 0xffu) == 21u &&
                      ((f.probe(probe_pixel, 1) >> 8) & 0xffu) == 0x20u,
                  "second TMDS stream on COMBOPHY_PLL1");
}

// dcn-stream (2026-10-24, additive): adopt_firmware_link records a DIG the GOP
// left lit so that the stream owner's first disable_link turns that PHY off and
// disconnects the firmware FE, as Linux link_set_dpms_off does for a
// link_active link found by dcn32_init_hw.
void check_adopt(Checks& checks) {
    Fixture f;
    DisplayLinkOwner& owner = *f.owner;
    owner.bind(f.config());
    RouteLease lease{};
    checks.expect(owner.acquire_route(2, epoch, lease) == LinkResult::success, "route for adoption");
    bool adopted = true;
    checks.expect(owner.adopt_firmware_link(lease, TmdsKind::hdmi, 2, adopted) == LinkResult::success && !adopted,
                  "a dark DIG back end is not adopted");
    checks.expect(owner.adopt_firmware_link(lease, TmdsKind::hdmi, 5, adopted) == LinkResult::invalid_argument &&
                      !adopted,
                  "an FE beyond the five DIG engines is refused");
    f.model.set(seg2, 0x20b2u + 0x200u, 1u);           // DIG_BE_EN_CNTL: enabled by the GOP
    f.model.set(seg2, 0x20b1u + 0x200u, 0x30000u | 0x0400u); // HDMI mode, FE2 connected
    checks.expect(owner.adopt_firmware_link(lease, TmdsKind::hdmi, 2, adopted) == LinkResult::success && adopted,
                  "a lit DIG back end is adopted");
    checks.expect(owner.adopt_firmware_link(lease, TmdsKind::hdmi, 2, adopted) == LinkResult::invalid_argument &&
                      !adopted,
                  "a link is adopted at most once");
    RouteLease stale = lease;
    stale.epoch = epoch + 1;
    checks.expect(owner.adopt_firmware_link(stale, TmdsKind::hdmi, 2, adopted) == LinkResult::stale_lease,
                  "a stale lease cannot adopt");
    checks.expect(owner.disable_link(lease) == LinkResult::success &&
                      (f.model.get(seg2, 0x20b1u + 0x200u) & 0x7f00u) == 0,
                  "disable_link of the adopted link disconnects the firmware FE");
}

// dcn-restore-modes (2026-10-24, additive): the Linux DMCUB p-state delegate
// and DPMS-off SCDC write, and the post-reset re-bind whose epoch the
// re-armed AUX adapter proves against.
void check_restore_steps(Checks& checks) {
    Fixture f;
    DisplayLinkOwner& owner = *f.owner;
    checks.expect(owner.p_state_delegate(false) == LinkResult::not_bound, "the delegate needs a bound owner");
    owner.bind(f.config());
    const size_t writes = f.model.writes.size();
    checks.expect(owner.p_state_delegate(false) == LinkResult::success && f.model.writes.size() == writes,
                  "no DMCUB started by this owner: the delegate sends nothing (Linux has no dmub_srv)");
    checks.expect(owner.p_state_delegate(true) == LinkResult::invalid_argument,
                  "FPO p-state management (pipe data) is refused");
    RouteLease dp{};
    RouteLease hdmi{};
    checks.expect(owner.acquire_route(0, epoch, dp) == LinkResult::success &&
                      owner.acquire_route(2, epoch, hdmi) == LinkResult::success,
                  "DP and HDMI routes leased");
    checks.expect(owner.hdmi_scdc_legacy(dp) == LinkResult::invalid_argument, "no SCDC write on a DP route");
    f.model.set(seg2, 0x1e9cu + 2u, 2u); // DC_I2C engine of DDC3 owned by hardware
    checks.expect(owner.hdmi_scdc_legacy(hdmi) == LinkResult::sink_failed && !owner.quarantined(),
                  "an SCDC write that cannot reach the sink is reported, not quarantined");
    RouteLease stale = hdmi;
    stale.generation += 1;
    checks.expect(owner.hdmi_scdc_legacy(stale) == LinkResult::stale_lease, "the SCDC write needs the current lease");
    {
        Fixture g;
        DisplayLinkOwner& other = *g.owner;
        other.bind(g.config());
        RouteLease route{};
        other.acquire_route(2, epoch, route);
        g.model.fail_write = g.model.writes.size() + 5; // a DC_I2C engine write is lost
        const LinkResult lost = other.hdmi_scdc_legacy(route);
        checks.expect(lost == LinkResult::quarantined && other.quarantined(),
                      "an SCDC write with unknown I2C engine state quarantines the owner");
    }

    checks.expect(owner.prepare_aux(dp) == LinkResult::success && owner.aux_proof(epoch, 0, 0, seg2, dcn::aperture),
                  "AUX proof before the reset");
    owner.reset_proven(epoch);
    checks.expect(!owner.bound() && !owner.aux_proof(epoch, 0, 0, seg2, dcn::aperture) &&
                      !owner.hpd_proof(epoch, 0, seg2, dcn::aperture),
                  "a proven reset drops every proof");
    DisplayLinkConfig next = f.config();
    next.epoch = epoch + 1;
    checks.expect(owner.bind(next) == LinkResult::success, "re-bind for the next epoch");
    RouteLease old{};
    RouteLease fresh{};
    checks.expect(owner.acquire_route(0, epoch, old) == LinkResult::stale_lease &&
                      owner.acquire_route(0, epoch + 1, fresh) == LinkResult::success,
                  "only the new epoch can lease the DP route");
    checks.expect(!owner.aux_proof(epoch + 1, 0, 0, seg2, dcn::aperture) && owner.prepare_aux(fresh) == LinkResult::success &&
                      owner.aux_proof(epoch + 1, 0, 0, seg2, dcn::aperture) &&
                      !owner.aux_proof(epoch, 0, 0, seg2, dcn::aperture),
                  "the re-armed AUX proof needs the reopened pad and holds for the new epoch only");
    f.model.set(seg2, 0x0200u, 0u); // DMCUB_CNTL2: out of reset, not started by this owner
    checks.expect(!owner.aux_proof(epoch + 1, 0, 0, seg2, dcn::aperture),
                  "a DMCUB that left reset without this owner blocks AUX after the reset");
}

} // namespace

int main() {
    Checks checks;
    check_leases(checks);
    check_dp(checks);
    check_fec(checks);
    check_tmds(checks);
    check_adopt(checks);
    check_restore_steps(checks);
    return checks.finish("display_link");
}

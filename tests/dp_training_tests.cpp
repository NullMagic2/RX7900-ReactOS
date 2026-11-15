/*
Module: rx7900.tests.dp_training
Purpose: Verify bounded DP clock recovery, equalization and failure cleanup.
Created: 2026-10-22
Architecture: Synthetic AUX and PHY callbacks; no MMIO or physical GPU access.
*/
#include <cstdio>

#include "dp_training.hpp"

namespace {

using namespace rx7900::output;

unsigned checks = 0;
unsigned failures = 0;

void expect(bool condition, const char* contract) {
    ++checks;

    if (!condition) {
        ++failures;
        std::fprintf(stderr, "FAIL: %s\n", contract);
    }
}

struct Port final {
    uint8_t sink_rate = 0;
    uint8_t sink_lanes = 0;
    uint8_t sink_pattern = 0;
    uint8_t source_pattern = 0;
    uint8_t last_sink_training_pattern = 0;
    uint8_t expected_rate = 0x14;
    uint8_t lane_settings[4]{};
    uint32_t clock_reads = 0;
    uint32_t equalization_reads = 0;
    uint32_t waits = 0;
    uint32_t configure_calls = 0;
    uint32_t source_pattern_calls = 0;
    uint32_t sink_pattern_calls = 0;
    uint32_t drive_calls = 0;
    uint32_t finish_calls = 0;
    bool last_finish_trained = false;
    bool configured = false;
    bool hpd = true;
    bool mst_active = false;
    bool clock_fails = false;
    bool equalization_fails = false;
    bool configure_fails = false;
    bool finish_fails = false;
    bool fail_disable = false;
    bool short_setup = false;
    // dcn-dsc-odm FEC: DPCD FEC_CONFIGURATION and the encoder FEC_READY, with
    // the order of the PHY configure, both FEC steps and LINK_BW_SET.
    uint8_t fec_configuration = 0;
    uint32_t fec_dpcd_writes = 0;
    uint32_t fec_phy_calls = 0;
    bool fec_phy_ready = false;
    bool fec_write_fails = false;
    bool fec_phy_fails = false;
    uint32_t events = 0;
    uint32_t configure_at = 0;
    uint32_t fec_dpcd_at = 0;
    uint32_t fec_phy_at = 0;
    uint32_t link_bw_at = 0;
};

bool transfer(void* context, const AuxRequest& request, AuxReply& reply) noexcept {
    auto& port = *static_cast<Port*>(context);

    if (request.operation == AuxOperation::native_write) {
        if (request.address == 0x120 && request.bytes == 1) {
            if (port.fec_write_fails) {
                return false;
            }
            port.fec_configuration = request.write_data[0];
            ++port.fec_dpcd_writes;
            port.fec_dpcd_at = ++port.events;
            reply = {AuxResult::ack, 1};
            return true;
        }

        if (request.address == 0x100 && request.bytes == 2) {
            port.link_bw_at = ++port.events;
            port.sink_rate = request.write_data[0];
            port.sink_lanes = request.write_data[1];
            reply = {AuxResult::ack,
                     static_cast<uint8_t>(port.short_setup ? 1 : 2)};
            port.short_setup = false;
            return true;
        }

        if (request.address == 0x102 && request.bytes == 1) {
            if (port.fail_disable && request.write_data[0] == 0) {
                return false;
            }

            port.sink_pattern = request.write_data[0];
            if (port.sink_pattern != 0) {
                ++port.sink_pattern_calls;
                port.last_sink_training_pattern = port.sink_pattern;
            }
            reply = {AuxResult::ack, 1};
            return true;
        }

        if (request.address == 0x103 && request.bytes <= 4) {
            for (uint8_t index = 0; index < request.bytes; ++index) {
                port.lane_settings[index] = request.write_data[index];
            }

            reply = {AuxResult::ack, request.bytes};
            return true;
        }

        return false;
    }

    if (request.operation == AuxOperation::native_read && request.bytes == 1 &&
        request.address == 0x111) {
        request.read_data[0] = port.mst_active ? 1 : 0;
        reply = {AuxResult::ack, 1};
        return true;
    }

    if (request.operation == AuxOperation::native_read && request.bytes == 6 &&
        request.address == 0x202) {
        for (uint8_t index = 0; index < 6; ++index) {
            request.read_data[index] = 0;
        }

        if (port.source_pattern == 1) {
            ++port.clock_reads;
            const bool recovered = !port.clock_fails && port.clock_reads >= 2;
            request.read_data[0] = recovered ? 0x11 : 0;
            request.read_data[1] = recovered ? 0x11 : 0;
        } else {
            ++port.equalization_reads;
            const bool equalized = !port.equalization_fails &&
                                   port.equalization_reads >= 2;
            request.read_data[0] = equalized ? 0x77 : 0x11;
            request.read_data[1] = equalized ? 0x77 : 0x11;
            request.read_data[2] = equalized ? 1 : 0;
        }

        request.read_data[4] = 0x11;
        request.read_data[5] = 0x11;
        reply = {AuxResult::ack, 6};
        return true;
    }

    return false;
}

bool wait_microseconds(void* context, uint32_t microseconds) noexcept {
    auto& port = *static_cast<Port*>(context);
    ++port.waits;
    return microseconds == 100 || microseconds == 400 ||
           microseconds == 4000;
}

bool connected(void* context) noexcept {
    return static_cast<Port*>(context)->hpd;
}

bool configure(void* context, uint8_t rate, uint8_t lanes, bool enhanced) noexcept {
    auto& port = *static_cast<Port*>(context);
    ++port.configure_calls;
    port.configure_at = ++port.events;
    port.configured = rate == port.expected_rate && lanes == 4 && enhanced;
    return port.configured && !port.configure_fails;
}

bool pattern(void* context, uint8_t value) noexcept {
    auto& port = *static_cast<Port*>(context);
    port.source_pattern = value;
    ++port.source_pattern_calls;

    return true;
}

bool drive(void* context, const uint8_t* settings, uint8_t lanes) noexcept {
    auto& port = *static_cast<Port*>(context);
    ++port.drive_calls;

    for (uint8_t index = 0; index < lanes; ++index) {
        if (settings[index] > 0x3f) {
            return false;
        }
    }

    return true;
}

bool finish(void* context, bool trained) noexcept {
    auto& port = *static_cast<Port*>(context);
    ++port.finish_calls;
    port.last_finish_trained = trained;
    return !port.finish_fails;
}

bool fec_ready(void* context, bool ready) noexcept {
    auto& port = *static_cast<Port*>(context);
    ++port.fec_phy_calls;
    port.fec_phy_ready = ready;
    port.fec_phy_at = ++port.events;
    return !port.fec_phy_fails;
}

AuxIo aux(Port& port) noexcept {
    return AuxIo{&port, transfer, wait_microseconds, connected};
}

DpPhyIo phy(Port& port) noexcept {
    return DpPhyIo{&port, configure, pattern, drive, finish};
}

DpPhyIo fec_phy(Port& port) noexcept {
    return DpPhyIo{&port, configure, pattern, drive, finish, fec_ready};
}

DpPortSnapshot snapshot() noexcept {
    DpPortSnapshot port{};
    port.receiver = {0x14, 0x14, 4, true, true, 0, false};
    port.sink.digital_input = true;
    port.sink.bits_per_channel = 8;
    const rx7900::display::Timing timing{
        148500, 1920, 2200, 88, 44, 1080, 1125, 4, 5,
        rx7900::display::positive_hsync | rx7900::display::positive_vsync
    };
    uint32_t index = 0;
    port.sink.detailed_modes.append(timing, index);
    dp_link_budget(0x14, 4, true, port.link);
    return port;
}

DpSourceCapabilities source() noexcept {
    return DpSourceCapabilities{0x14, 4, true, true, true, true};
}

void check_training() {
    Port port{};
    const auto result = train_dp_sst(aux(port), phy(port), source(), snapshot(),
                                     DpTrainingRequest{0x14, 4, 0});
    expect(result == DpTrainingStatus::trained && port.configured &&
           port.sink_rate == 0x14 && port.sink_lanes == 0x84,
           "verified source and sink negotiate HBR2 four-lane enhanced framing");
    expect(port.clock_reads == 2 && port.equalization_reads == 2 &&
           port.waits == 4 && port.source_pattern == 3,
           "clock recovery and TPS3 equalization use bounded status polling");
    expect(port.source_pattern_calls == 2 && port.sink_pattern_calls == 2 &&
           port.drive_calls == 4,
           "training pattern is set once per phase while lane drive is retried");
    expect(port.sink_pattern == 0 && port.finish_calls == 1 &&
           port.last_finish_trained,
           "successful training disables patterns before source completion");
    expect(port.lane_settings[0] == 1 && port.lane_settings[3] == 1,
           "sink voltage requests reach each source lane");
}

void check_rejections() {
    Port port{};
    auto probed = snapshot();
    expect(train_dp_sst(aux(port), phy(port), source(), probed,
                        DpTrainingRequest{0x06, 1, 0}) ==
           DpTrainingStatus::unsupported_mode && port.configure_calls == 0,
           "mode bandwidth is checked before touching the PHY");

    auto caps = source();
    caps.verified = false;
    expect(train_dp_sst(aux(port), phy(port), caps, probed,
                        DpTrainingRequest{0x14, 4, 0}) ==
           DpTrainingStatus::unsupported_link,
           "unverified source hardware cannot train");

    auto hbr3_source = source();
    hbr3_source.maximum_rate_code = 0x1e;
    auto hbr3_port = snapshot();
    hbr3_port.receiver.maximum_rate_code = 0x1e;
    dp_link_budget(0x1e, 4, true, hbr3_port.link);
    expect(train_dp_sst(aux(port), phy(port), hbr3_source, hbr3_port,
                        DpTrainingRequest{0x1e, 4, 0}) ==
           DpTrainingStatus::unsupported_link && port.configure_calls == 0,
           "HBR3 is refused without verified TPS4 on both ends");

    port.mst_active = true;
    probed.receiver.mst_capable = true;
    expect(train_dp_sst(aux(port), phy(port), source(), probed,
                        DpTrainingRequest{0x14, 4, 0}) ==
           DpTrainingStatus::unsupported_link && port.configure_calls == 0,
           "active MST is refused before any source change");

    port = {};
    port.hpd = false;
    expect(train_dp_sst(aux(port), phy(port), source(), snapshot(),
                        DpTrainingRequest{0x14, 4, 0}) ==
           DpTrainingStatus::disconnected && port.configure_calls == 0,
           "disconnected port cannot start training");
}

void check_hbr3() {
    Port port{};
    port.expected_rate = 0x1e;
    auto caps = source();
    caps.maximum_rate_code = 0x1e;
    caps.training_pattern_4 = true;
    auto sink = snapshot();
    sink.receiver.maximum_rate_code = 0x1e;
    sink.receiver.training_pattern_4 = true;
    dp_link_budget(0x1e, 4, true, sink.link);

    expect(train_dp_sst(aux(port), phy(port), caps, sink,
                        DpTrainingRequest{0x1e, 4, 0}) ==
           DpTrainingStatus::trained && port.source_pattern == 7 &&
           port.last_sink_training_pattern == 7 && port.sink_pattern == 0,
           "HBR3 TPS4 equalization keeps sink scrambling enabled and cleans up");
}

void check_failures() {
    Port port{};
    port.clock_fails = true;
    expect(train_dp_sst(aux(port), phy(port), source(), snapshot(),
                        DpTrainingRequest{0x14, 4, 0}) ==
           DpTrainingStatus::clock_recovery_failed && port.clock_reads == 5 &&
           port.sink_pattern == 0 && !port.last_finish_trained,
           "failed clock recovery is bounded and quiesces the link");

    port = {};
    port.equalization_fails = true;
    expect(train_dp_sst(aux(port), phy(port), source(), snapshot(),
                        DpTrainingRequest{0x14, 4, 0}) ==
           DpTrainingStatus::equalization_failed &&
           port.equalization_reads == 5 && port.sink_pattern == 0,
           "failed channel equalization is bounded and quiesces the link");

    port = {};
    port.configure_fails = true;
    expect(train_dp_sst(aux(port), phy(port), source(), snapshot(),
                        DpTrainingRequest{0x14, 4, 0}) ==
           DpTrainingStatus::source_failure && port.finish_calls == 1,
           "source configuration failure requests explicit recovery");

    port = {};
    port.short_setup = true;
    expect(train_dp_sst(aux(port), phy(port), source(), snapshot(),
                        DpTrainingRequest{0x14, 4, 0}) ==
           DpTrainingStatus::auxiliary_failure && port.sink_pattern == 0,
           "short DPCD setup cannot proceed to pattern training");

    port = {};
    port.fail_disable = true;
    expect(train_dp_sst(aux(port), phy(port), source(), snapshot(),
                        DpTrainingRequest{0x14, 4, 0}) ==
           DpTrainingStatus::state_unknown && !port.last_finish_trained,
           "failed sink pattern disable cannot claim a trained link");

    port = {};
    port.finish_fails = true;
    expect(train_dp_sst(aux(port), phy(port), source(), snapshot(),
                        DpTrainingRequest{0x14, 4, 0}) ==
           DpTrainingStatus::state_unknown,
           "failed PHY completion quarantines the uncertain source state");
}

// dcn-dsc-odm: 3840x2160@144 (CVT-RB2) on HBR3 x4 only fits compressed.
constexpr rx7900::display::Timing mode_4k144{
    1306200, 3840, 3920, 8, 32, 2160, 2314, 140, 8,
    rx7900::display::positive_hsync | rx7900::display::positive_vsync
};

DpPortSnapshot hbr3_dsc_snapshot() noexcept {
    DpPortSnapshot sink = snapshot();
    sink.receiver.maximum_rate_code = 0x1e;
    sink.receiver.training_pattern_4 = true;
    uint32_t index = 0;
    sink.sink.detailed_modes.append(mode_4k144, index);
    dp_link_budget(0x1e, 4, true, sink.link);
    return sink;
}

DpSourceCapabilities hbr3_source() noexcept {
    auto caps = source();
    caps.maximum_rate_code = 0x1e;
    caps.training_pattern_4 = true;
    return caps;
}

DpTrainingRequest dsc_request(uint32_t bpp_x16, bool fec) noexcept {
    DpTrainingRequest request{0x1e, 4, 1};
    request.fec_ready = fec;
    request.dsc_bits_per_pixel_x16 = bpp_x16;
    return request;
}

void check_fec() {
    Port port{};
    port.expected_rate = 0x1e;
    expect(train_dp_sst(aux(port), fec_phy(port), hbr3_source(), hbr3_dsc_snapshot(),
                        dsc_request(256, true)) == DpTrainingStatus::trained,
           "3840x2160@144 trains on HBR3 x4 with FEC at 16 bpp DSC");
    expect(port.fec_configuration == 1 && port.fec_dpcd_writes == 1 && port.fec_phy_calls == 1 &&
           port.fec_phy_ready && port.configure_at < port.fec_dpcd_at &&
           port.fec_dpcd_at < port.fec_phy_at && port.fec_phy_at < port.link_bw_at,
           "dp_set_fec_ready: FEC_CONFIGURATION.FEC_READY, then the encoder, after the PHY enable and before LINK_BW_SET");

    port = {};
    port.expected_rate = 0x1e;
    expect(train_dp_sst(aux(port), fec_phy(port), hbr3_source(), hbr3_dsc_snapshot(),
                        DpTrainingRequest{0x1e, 4, 1}) == DpTrainingStatus::unsupported_mode &&
           port.configure_calls == 0,
           "the same mode uncompressed exceeds HBR3 x4");

    port = {};
    port.expected_rate = 0x1e;
    expect(train_dp_sst(aux(port), fec_phy(port), hbr3_source(), hbr3_dsc_snapshot(),
                        DpTrainingRequest{0x1e, 4, 0}) == DpTrainingStatus::trained &&
           port.fec_dpcd_writes == 0 && port.fec_phy_calls == 0,
           "a request without FEC never touches FEC_CONFIGURATION or the encoder FEC");

    // The 97 percent FEC efficiency: the largest rate that fits the plain
    // budget no longer fits with FEC.
    const DpPortSnapshot sink = hbr3_dsc_snapshot();
    const uint64_t pixel_bits = static_cast<uint64_t>(mode_4k144.pixel_clock_khz) * 1000u;
    const uint32_t edge = static_cast<uint32_t>(sink.link.usable_payload_bits_per_second * 16u / pixel_bits);
    port = {};
    port.expected_rate = 0x1e;
    expect(edge > 16u * 16u && edge < 24u * 16u &&
           train_dp_sst(aux(port), phy(port), hbr3_source(), sink, dsc_request(edge, false)) ==
               DpTrainingStatus::trained,
           "a compressed rate at the plain budget is admitted without FEC");
    port = {};
    port.expected_rate = 0x1e;
    expect(train_dp_sst(aux(port), fec_phy(port), hbr3_source(), sink, dsc_request(edge, true)) ==
               DpTrainingStatus::unsupported_mode && port.configure_calls == 0,
           "with FEC the same rate is refused (97 percent payload)");

    port = {};
    port.expected_rate = 0x1e;
    expect(train_dp_sst(aux(port), fec_phy(port), hbr3_source(), sink, dsc_request(5u * 16u, true)) ==
               DpTrainingStatus::unsupported_mode &&
           train_dp_sst(aux(port), fec_phy(port), hbr3_source(), sink, dsc_request(25u * 16u, true)) ==
               DpTrainingStatus::unsupported_mode && port.configure_calls == 0,
           "DSC rates outside 6..24 bpp are refused");

    port = {};
    port.expected_rate = 0x1e;
    expect(train_dp_sst(aux(port), phy(port), hbr3_source(), sink, dsc_request(256, true)) ==
               DpTrainingStatus::unsupported_link && port.configure_calls == 0,
           "FEC without an encoder FEC_READY hook is refused before the PHY");

    port = {};
    port.expected_rate = 0x1e;
    port.fec_write_fails = true;
    expect(train_dp_sst(aux(port), fec_phy(port), hbr3_source(), sink, dsc_request(256, true)) ==
               DpTrainingStatus::auxiliary_failure && port.fec_phy_calls == 0 && port.finish_calls == 1 &&
           !port.last_finish_trained && port.link_bw_at == 0,
           "a failed FEC_CONFIGURATION write stops before the encoder and LINK_BW_SET and quiesces");

    port = {};
    port.expected_rate = 0x1e;
    port.fec_phy_fails = true;
    expect(train_dp_sst(aux(port), fec_phy(port), hbr3_source(), sink, dsc_request(256, true)) ==
               DpTrainingStatus::source_failure && port.finish_calls == 1 && port.link_bw_at == 0,
           "a failed encoder FEC_READY is a source failure before LINK_BW_SET");
}

} // namespace

int main() {
    check_training();
    check_hbr3();
    check_rejections();
    check_failures();
    check_fec();
    std::printf("dp_training: %u checks, %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

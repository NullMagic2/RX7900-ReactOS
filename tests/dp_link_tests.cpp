/*
Module: rx7900.tests.dp_link
Purpose: Exercise bounded AUX, DPCD and EDID discovery against a synthetic port.
Created: 2026-10-22
Architecture: Protocol and ownership failures only; no physical GPU is touched.
*/
#include <array>
#include <cstdio>

#include "dp_link.hpp"

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
    std::array<uint8_t, maximum_edid_blocks * edid_block_bytes> edid{};
    std::array<uint8_t, dp_receiver_capability_bytes> dpcd{};
    uint8_t mst = 0;
    uint8_t segment = 0;
    uint8_t offset = 0;
    uint32_t defer_left = 0;
    uint32_t waits = 0;
    uint32_t transfers = 0;
    uint32_t stops = 0;
    uint32_t connected_calls = 0;
    uint32_t disconnect_at_call = 0;
    bool fail_transfer = false;
    bool short_native = false;
    bool short_i2c = false;
};

void checksum(Port& port, uint32_t block) {
    uint32_t sum = 0;
    const uint32_t base = block * edid_block_bytes;

    for (uint32_t index = 0; index < 127; ++index) {
        sum += port.edid[base + index];
    }

    port.edid[base + 127] = static_cast<uint8_t>((256u - (sum & 255u)) & 255u);
}

Port fixture(bool extension = false) {
    Port port{};
    port.dpcd[0] = 0x14;
    port.dpcd[1] = 0x14;
    port.dpcd[2] = 0x84;
    port.dpcd[6] = 0x01;

    for (uint32_t index = 1; index < 7; ++index) {
        port.edid[index] = 0xff;
    }

    port.edid[18] = 1;
    port.edid[19] = 4;
    port.edid[20] = 0xa0;
    port.edid[24] = 0x02;
    port.edid[126] = extension ? 1 : 0;
    constexpr std::array<uint8_t, 18> full_hd_dtd{{
        0x02, 0x3a, 0x80, 0x18, 0x71, 0x38, 0x2d, 0x40, 0x58,
        0x2c, 0x45, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1e
    }};

    for (uint32_t index = 0; index < full_hd_dtd.size(); ++index) {
        port.edid[54 + index] = full_hd_dtd[index];
    }

    checksum(port, 0);

    if (extension) {
        port.edid[128] = 2;
        port.edid[129] = 3;
        port.edid[130] = 4;

        for (uint32_t index = 0; index < full_hd_dtd.size(); ++index) {
            port.edid[132 + index] = full_hd_dtd[index];
        }

        port.edid[132] = 0x01;
        checksum(port, 1);
    }

    return port;
}

bool transfer(void* context, const AuxRequest& request, AuxReply& reply) noexcept {
    auto& port = *static_cast<Port*>(context);
    ++port.transfers;

    if (port.fail_transfer) {
        return false;
    }

    if (port.defer_left != 0) {
        --port.defer_left;
        reply = {AuxResult::defer, 0};
        return true;
    }

    if (request.operation == AuxOperation::native_read) {
        if (request.address == 0 && request.bytes == dp_receiver_capability_bytes) {
            const uint8_t count = port.short_native ? 15 : request.bytes;

            for (uint8_t index = 0; index < count; ++index) {
                request.read_data[index] = port.dpcd[index];
            }

            reply = {AuxResult::ack, count};
            return true;
        }

        if (request.address == 0x21 && request.bytes == 1) {
            request.read_data[0] = port.mst;
            reply = {AuxResult::ack, 1};
            return true;
        }

        return false;
    }

    if (request.operation == AuxOperation::i2c_write && request.bytes == 1) {
        if (request.address == 0x30) {
            port.segment = request.write_data[0];
        } else if (request.address == 0x50) {
            port.offset = request.write_data[0];
        } else {
            return false;
        }

        reply = {AuxResult::ack, 1};
        return true;
    }

    if (request.operation == AuxOperation::i2c_read && request.address == 0x50) {
        if (request.bytes == 0) {
            ++port.stops;
            reply = {AuxResult::ack, 0};
            return true;
        }

        const uint8_t count = port.short_i2c ? 0 : request.bytes;
        const uint32_t base = static_cast<uint32_t>(port.segment) * 256u + port.offset;

        if (base + count > port.edid.size()) {
            return false;
        }

        for (uint8_t index = 0; index < count; ++index) {
            request.read_data[index] = port.edid[base + index];
        }

        port.offset = static_cast<uint8_t>(port.offset + count);
        reply = {AuxResult::ack, count};
        return true;
    }

    return false;
}

bool wait_microseconds(void* context, uint32_t microseconds) noexcept {
    auto& port = *static_cast<Port*>(context);
    ++port.waits;
    return microseconds == 400;
}

bool connected(void* context) noexcept {
    auto& port = *static_cast<Port*>(context);
    ++port.connected_calls;
    return port.disconnect_at_call == 0 || port.connected_calls < port.disconnect_at_call;
}

AuxIo io(Port& port) noexcept {
    return AuxIo{&port, transfer, wait_microseconds, connected};
}

DpSourceCapabilities source() noexcept {
    return DpSourceCapabilities{0x14, 4, true, true};
}

DpProbeStatus probe(Port& port, const DpSourceCapabilities& capabilities,
                    DpPortSnapshot& output) noexcept {
    DpProbeScratch scratch{};
    return probe_dp_port(io(port), capabilities, scratch, output);
}

void check_success() {
    Port port = fixture();
    DpPortSnapshot snapshot{};
    expect(probe(port, source(), snapshot) == DpProbeStatus::success,
           "real DPCD and complete EDID transactions create a snapshot");
    expect(snapshot.receiver.dpcd_revision == 0x14 &&
           snapshot.receiver.maximum_rate_code == 0x14 &&
           snapshot.receiver.maximum_lanes == 4 &&
           snapshot.receiver.enhanced_framing,
           "receiver properties derive only from DPCD");
    expect(snapshot.sink.detailed_modes.count() == 1 &&
           snapshot.sink.preferred_mode == 0 &&
           snapshot.link.transport == Transport::display_port_sst &&
           snapshot.link.rgb_8bpc_verified,
           "verified source and sink admit the reusable output mode");
    expect(snapshot.link.usable_payload_bits_per_second == UINT64_C(17107200000) &&
           snapshot.link.maximum_pixel_clock_khz == 712800,
           "8b/10b and framing reserve bound the pixel clock");
    expect(port.transfers == 11 && port.stops == 0,
           "base EDID uses one offset and eight bounded AUX reads");

    port = fixture(true);
    snapshot = {};
    expect(probe(port, source(), snapshot) == DpProbeStatus::success &&
           snapshot.sink.block_count == 2 && snapshot.sink.detailed_modes.count() == 2,
           "CTA extension is acquired and decoded through the existing parser");
}

void check_faults() {
    Port port = fixture();
    DpPortSnapshot snapshot{};
    snapshot.receiver.dpcd_revision = 77;
    port.defer_left = 7;
    expect(probe(port, source(), snapshot) == DpProbeStatus::success &&
           port.waits == 7, "AUX DEFER is retried seven times before ACK");

    port = fixture();
    port.defer_left = 8;
    snapshot.receiver.dpcd_revision = 77;
    expect(probe(port, source(), snapshot) ==
           DpProbeStatus::auxiliary_defer_exhausted &&
           snapshot.receiver.dpcd_revision == 77,
           "bounded DEFER exhaustion leaves caller state unchanged");

    port = fixture();
    port.short_native = true;
    expect(probe(port, source(), snapshot) ==
           DpProbeStatus::short_transfer && snapshot.receiver.dpcd_revision == 77,
           "short DPCD ACK cannot publish an incomplete receiver");

    port = fixture();
    port.short_i2c = true;
    expect(probe(port, source(), snapshot) ==
           DpProbeStatus::short_transfer && port.stops == 1 &&
           snapshot.receiver.dpcd_revision == 77,
           "short EDID packet closes I2C MOT and publishes no snapshot");

    port = fixture();
    port.edid[30] ^= 1;
    expect(probe(port, source(), snapshot) ==
           DpProbeStatus::invalid_edid && snapshot.receiver.dpcd_revision == 77,
           "EDID checksum failure is rejected atomically");

    port = fixture();
    port.edid[126] = 8;
    checksum(port, 0);
    expect(probe(port, source(), snapshot) ==
           DpProbeStatus::invalid_edid,
           "extension count cannot exceed the bounded EDID buffer");

    port = fixture();
    port.disconnect_at_call = 3;
    expect(probe(port, source(), snapshot) ==
           DpProbeStatus::disconnected && snapshot.receiver.dpcd_revision == 77,
           "HPD loss during discovery cannot publish stale connector state");

    port = fixture();
    port.fail_transfer = true;
    expect(probe(port, source(), snapshot) ==
           DpProbeStatus::auxiliary_failure,
           "transport failure is distinct from NACK or malformed EDID");
}

void check_capabilities() {
    Port port = fixture();
    DpPortSnapshot snapshot{};
    auto source_caps = source();
    source_caps.verified = false;
    expect(probe(port, source_caps, snapshot) ==
           DpProbeStatus::unsupported_source && port.transfers == 0,
           "unverified PHY capabilities cannot advertise a mode");

    port = fixture();
    port.dpcd[5] = 1;
    expect(probe(port, source(), snapshot) ==
           DpProbeStatus::unsupported_receiver,
           "branch and downstream adapters stay outside the SST-only contract");

    port = fixture();
    port.mst = 1;
    expect(probe(port, source(), snapshot) ==
           DpProbeStatus::success && snapshot.receiver.mst_capable,
           "MST capability is recorded without asserting MST operation");

    port = fixture();
    port.dpcd[6] = 2;
    expect(probe(port, source(), snapshot) ==
           DpProbeStatus::unsupported_receiver,
           "128b/132b-only receiver cannot use legacy rate calculations");

    port = fixture();
    source_caps = source();
    source_caps.maximum_rate_code = 0x06;
    source_caps.maximum_lanes = 1;
    expect(probe(port, source_caps, snapshot) ==
           DpProbeStatus::no_rgb8_mode,
           "one RBR lane cannot admit the 148.5 MHz RGB8 timing");

    port = fixture();
    source_caps = source();
    source_caps.rgb8_verified = false;
    expect(probe(port, source_caps, snapshot) ==
           DpProbeStatus::no_rgb8_mode,
           "unverified RGB8 source cannot become a false video mode");
}

} // namespace

int main() {
    check_success();
    check_faults();
    check_capabilities();
    std::printf("dp_link: %u checks, %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

/*
Module: rx7900.tests.dp_port_native
Purpose: Exercise ATOM route ownership and uncertain-link quarantine.
Created: 2026-10-22
Architecture: Synthetic owner/AUX/PHY only; no actual Navi31 registers or GPU.
*/
#include <cstdio>

#include "dp_port_native.hpp"

namespace {

using namespace rx7900;

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
    uint8_t edid[128]{};
    uint8_t edid_offset = 0;
    uint8_t source_pattern = 0;
    uint8_t sink_pattern = 0;
    uint8_t sink_rate = 0;
    uint32_t clock_reads = 0;
    uint32_t eq_reads = 0;
    uint32_t retire_calls = 0;
    uint64_t epoch = 7;
    bool hpd = true;
    bool quiet = true;
    bool can_quiesce = true;
    bool reset = false;
};

void make_edid(Port& port) {
    for (unsigned i = 1; i < 7; ++i) port.edid[i] = 0xff;
    port.edid[18] = 1;
    port.edid[19] = 4;
    port.edid[20] = 0xa0;
    port.edid[24] = 0x02;
    const uint8_t timing[18]{
        0x02, 0x3a, 0x80, 0x18, 0x71, 0x38, 0x2d, 0x40, 0x58,
        0x2c, 0x45, 0, 0, 0, 0, 0, 0, 0x1e
    };
    for (unsigned i = 0; i < 18; ++i) port.edid[54 + i] = timing[i];
    unsigned sum = 0;
    for (unsigned i = 0; i < 127; ++i) sum += port.edid[i];
    port.edid[127] = static_cast<uint8_t>(0u - sum);
}

bool transfer(void* context, const output::AuxRequest& request,
              output::AuxReply& reply) noexcept {
    auto& port = *static_cast<Port*>(context);
    using output::AuxOperation;
    using output::AuxResult;

    if (request.operation == AuxOperation::native_read) {
        if (request.address == 0 && request.bytes == 16) {
            for (uint8_t i = 0; i < 16; ++i) request.read_data[i] = 0;
            request.read_data[0] = 0x14;
            request.read_data[1] = 0x14;
            request.read_data[2] = 0xc4;
            request.read_data[6] = 1;
        } else if (request.address == 0x21 && request.bytes == 1) {
            request.read_data[0] = 0;
        } else if (request.address == 0x202 && request.bytes == 6) {
            for (uint8_t i = 0; i < 6; ++i) request.read_data[i] = 0;
            if (port.source_pattern == 1) {
                ++port.clock_reads;
                if (port.clock_reads >= 2) {
                    request.read_data[0] = 0x11;
                    request.read_data[1] = 0x11;
                }
            } else {
                ++port.eq_reads;
                request.read_data[0] = 0x77;
                request.read_data[1] = 0x77;
                request.read_data[2] = 1;
            }
            request.read_data[4] = 0x11;
            request.read_data[5] = 0x11;
        } else {
            return false;
        }
    } else if (request.operation == AuxOperation::native_write) {
        if (request.address == 0x100 && request.bytes == 2) {
            port.sink_rate = request.write_data[0];
        } else if (request.address == 0x102 && request.bytes == 1) {
            port.sink_pattern = request.write_data[0];
        } else if (request.address != 0x103 || request.bytes > 4) {
            return false;
        }
    } else if (request.operation == AuxOperation::i2c_write &&
               request.address == 0x50 && request.bytes == 1) {
        port.edid_offset = request.write_data[0];
    } else if (request.operation == AuxOperation::i2c_read &&
               request.address == 0x50) {
        if (static_cast<unsigned>(port.edid_offset) + request.bytes > 128u) {
            return false;
        }
        for (uint8_t i = 0; i < request.bytes; ++i) {
            request.read_data[i] = port.edid[port.edid_offset + i];
        }
        port.edid_offset = static_cast<uint8_t>(port.edid_offset + request.bytes);
    } else {
        return false;
    }

    reply = {AuxResult::ack, request.bytes};
    return true;
}

bool wait(void*, uint32_t microseconds) noexcept {
    return microseconds == 100 || microseconds == 400 || microseconds == 4000;
}
bool connected(void* context) noexcept { return static_cast<Port*>(context)->hpd; }
bool configure(void*, uint8_t rate, uint8_t lanes, bool enhanced) noexcept {
    return rate == 0x14 && lanes == 4 && enhanced;
}
bool pattern(void* context, uint8_t value) noexcept {
    static_cast<Port*>(context)->source_pattern = value;
    return true;
}
bool drive(void*, const uint8_t*, uint8_t lanes) noexcept { return lanes == 4; }
bool finish(void*, bool) noexcept { return true; }

bool proof(void* context, uint64_t epoch,
           const output::Connector& connector) noexcept {
    auto& port = *static_cast<Port*>(context);
    return epoch == port.epoch && connector.firmware_path_index == 2 &&
           connector.encoder_object_id == 0x10 && connector.hpd_pin_id == 1 &&
           connector.i2c_id == 1;
}
bool quiescent(void* context) noexcept { return static_cast<Port*>(context)->quiet; }
bool quiesce(void* context) noexcept {
    auto& port = *static_cast<Port*>(context);
    ++port.retire_calls;
    return port.can_quiesce;
}
bool reset_proven(void* context, uint64_t old_epoch) noexcept {
    auto& port = *static_cast<Port*>(context);
    return port.reset && old_epoch == 7 && port.epoch != old_epoch;
}

output::ConnectorCatalog catalog() noexcept {
    output::ConnectorCatalog result{};
    result.count = 1;
    auto& connector = result.connectors[0];
    connector.kind = output::ConnectorKind::display_port;
    connector.firmware_path_index = 2;
    connector.encoder_object_id = 0x10;
    connector.hpd_pin_id = 1;
    connector.i2c_id = 1;
    connector.has_hpd_record = true;
    connector.has_i2c_record = true;
    return result;
}

reactos::DpPortResult bind(reactos::DpNativePort& session, Port& port,
                           const output::ConnectorCatalog& routes) noexcept {
    const output::DpSourceCapabilities source{0x14, 4, true, true, true, true};
    return session.bind(routes, 2, 7, source,
                        output::AuxIo{&port, transfer, wait, connected},
                        output::DpPhyIo{&port, configure, pattern, drive, finish},
                        reactos::DpNativeOwner{&port, proof, quiescent,
                                               quiesce, reset_proven});
}

void check_lifecycle() {
    Port port{};
    make_edid(port);
    reactos::DpNativePort session{};
    auto routes = catalog();
    output::DpProbeScratch scratch{};
    expect(bind(session, port, routes) == reactos::DpPortResult::success &&
           session.state() == reactos::DpPortState::bound,
           "one verified ATOM route obtains an exclusive idle link lease");
    expect(session.probe(scratch) == reactos::DpPortResult::success &&
           session.snapshot() != nullptr &&
           session.snapshot()->sink.detailed_modes.count() == 1,
           "owned AUX path discovers a real checked EDID mode");
    expect(session.train(output::DpTrainingRequest{0x14, 4, 0}) ==
           reactos::DpPortResult::success &&
           session.state() == reactos::DpPortState::trained &&
           port.sink_rate == 0x14 && port.sink_pattern == 0,
           "owned PHY/AUX path trains without publishing scanout");
    port.epoch = 8;
    expect(session.snapshot() == nullptr &&
           session.retire() == reactos::DpPortResult::state_unknown &&
           session.state() == reactos::DpPortState::quarantined,
           "lost ownership hides stale snapshot and quarantines link resources");
    expect(session.recover_after_reset() == reactos::DpPortResult::state_unknown,
           "quarantine cannot be released without a physical reset proof");
    port.reset = true;
    expect(session.recover_after_reset() == reactos::DpPortResult::success &&
           session.state() == reactos::DpPortState::unbound,
           "proven reset retires the old epoch without touching stale MMIO");
}

void check_boundaries() {
    Port port{};
    make_edid(port);
    auto routes = catalog();
    reactos::DpNativePort session{};
    routes.connectors[0].kind = output::ConnectorKind::usb_type_c;
    expect(bind(session, port, routes) == reactos::DpPortResult::unsupported_route,
           "USB-C cannot borrow the plain DP path without an owned mux");
    routes = catalog();
    routes.count = 2;
    routes.connectors[1] = routes.connectors[0];
    expect(bind(session, port, routes) == reactos::DpPortResult::unsupported_route,
           "ambiguous firmware paths cannot select hardware resources");
    routes = catalog();
    port.quiet = false;
    expect(bind(session, port, routes) == reactos::DpPortResult::not_quiescent,
           "active scanout cannot be silently stolen");
    port.quiet = true;
    expect(bind(session, port, routes) == reactos::DpPortResult::success,
           "failed binding has no residual state");
    port.can_quiesce = false;
    expect(session.retire() == reactos::DpPortResult::state_unknown &&
           session.state() == reactos::DpPortState::quarantined &&
           port.retire_calls == 1,
           "unconfirmed hardware quiesce quarantines the lease");

    Port stolen{};
    make_edid(stolen);
    reactos::DpNativePort abandoned{};
    expect(bind(abandoned, stolen, routes) == reactos::DpPortResult::success,
           "second independent fixture obtains a lease");
    stolen.quiet = false;
    output::DpProbeScratch scratch{};
    expect(abandoned.probe(scratch) == reactos::DpPortResult::state_unknown &&
           abandoned.state() == reactos::DpPortState::quarantined &&
           abandoned.snapshot() == nullptr,
           "lost quiescence after binding cannot expose a stale candidate");
}

} // namespace

int main() {
    check_lifecycle();
    check_boundaries();
    std::printf("dp_port_native: %u checks, %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

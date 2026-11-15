/*
Module: rx7900.tests.interrupt
Purpose: Verify IH6 decoding and rejection of unrelated/MES queue events.
Created: 2026-10-21
Architecture: CPU-only vectors; no interrupts are installed or generated.
*/
#include <array>
#include <cstdio>
#include "interrupt.hpp"

namespace {

using namespace rx7900::sdma;
unsigned checks = 0;
unsigned failures = 0;

void expect(bool condition, const char* contract) {
    ++checks;

    if (!condition) {
        ++failures;
        std::fprintf(stderr, "FAIL: %s\n", contract);
    }
}

void check_vectors() {
    std::array<uint8_t, 40> bytes{};
    CommandBuffer output(bytes.data(), bytes.size());
    output.write_u32(4, 0x8010310a);
    output.write_u32(8, 0x12345678);
    output.write_u32(12, 0x8000abcd);
    output.write_u32(16, 0x00550000);
    output.write_u32(20, 0);
    output.write_u32(24, 0x11223344);
    output.write_u32(28, 0x55667788);
    output.write_u32(32, 0xaabbccdd);
    InterruptVector vector{};
    expect(decode_interrupt({bytes.data(), bytes.size()}, 4, vector),
           "decode complete unaligned-offset IH vector");
    expect(vector.client == 0xa && vector.source == 49 && vector.ring == 0x10 &&
           vector.vmid == 0 && vector.vmid_source && vector.timestamp_source &&
           vector.timestamp == 0xabcd12345678ULL && vector.node == 0x55 &&
           vector.source_data[3] == 0xaabbccdd,
           "IH fields match pinned Linux helper");
    auto route = classify_interrupt(vector);
    expect(route.action == InterruptAction::poll_fence && route.instance == 1,
           "route legacy SDMA instance1 queue0 trap");

    for (size_t size = 0; size < 36; ++size) {
        expect(!decode_interrupt({bytes.data(), size}, 4, vector) && vector.client == 0,
               "every incomplete IH entry fails without stale output");
    }

    expect(!decode_interrupt({bytes.data(), bytes.size()}, SIZE_MAX, vector),
           "IH offset overflow fails before arithmetic");
    expect(!decode_interrupt({nullptr, SIZE_MAX}, 0, vector), "null IH bytes are empty");
    vector = {};
    vector.client = 0xa;
    vector.source = 49;
    vector.ring = 0;
    expect(classify_interrupt(vector).action == InterruptAction::poll_fence,
           "route legacy SDMA instance0 trap");
    Ring unavailable;
    expect(service_interrupt(vector, 0, unavailable, 0) == RingStatus::unavailable,
           "interrupt cannot initialize unavailable hardware or claim completion");
    vector.source_data[0] = 0x80000001;
    expect(classify_interrupt(vector).action == InterruptAction::ignore,
           "MES context cannot complete legacy queue");
    vector.source_data[0] = 0;
    vector.ring = 1;
    expect(classify_interrupt(vector).action == InterruptAction::ignore,
           "unowned queue is ignored");
    vector.ring = 0x20;
    expect(classify_interrupt(vector).action == InterruptAction::malformed,
           "nonexistent SDMA instance rejected");
    vector.ring = 0;
    vector.vmid = 1;
    expect(classify_interrupt(vector).action == InterruptAction::ignore,
           "other process VMID cannot complete kernel queue");
    vector.vmid = 0;
    vector.pasid = 1;
    expect(classify_interrupt(vector).action == InterruptAction::ignore,
           "other process PASID cannot complete kernel queue");
    vector.pasid = 0;

    for (uint32_t source = 54; source <= 66; ++source) {
        vector.source = source;
        expect(classify_interrupt(vector).action == InterruptAction::queue_fault,
               "SDMA fault events require queue recovery");
    }

    vector.source = 50;
    expect(classify_interrupt(vector).action == InterruptAction::queue_fault,
           "protected SRBM write is an error");
    vector.source = 51;
    expect(classify_interrupt(vector).action == InterruptAction::ignore,
           "context-empty event is not a completion fence");
    vector.client = 8;
    vector.source = 49;
    expect(classify_interrupt(vector).action == InterruptAction::ignore,
           "pre-SOC21 SDMA client ID is not Navi31 GFX");
}

} // namespace

int main() {
    check_vectors();
    std::printf("%u interrupt checks; %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

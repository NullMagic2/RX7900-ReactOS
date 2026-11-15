/*
Module: rx7900.tests.dcn32_hpd
Purpose: Check bounded, read-only DCN3.2 HPD sense access.
Created: 2026-10-22
Architecture: Synthetic registers only; no physical GPU is touched.
*/
#include <cstdio>

#include "dcn32_hpd.hpp"

namespace {

using namespace rx7900::display;

unsigned checks = 0;
unsigned failures = 0;

void expect(bool condition, const char* contract) {
    ++checks;
    if (!condition) {
        ++failures;
        std::fprintf(stderr, "FAIL: %s\n", contract);
    }
}

struct Registers final {
    uint32_t expected_offset = 0;
    uint32_t value = 0;
    unsigned reads = 0;
    bool fail = false;
};

bool read(void* context, uint32_t offset, uint32_t& value) noexcept {
    auto& registers = *static_cast<Registers*>(context);
    ++registers.reads;
    if (registers.fail || offset != registers.expected_offset) {
        return false;
    }
    value = registers.value;
    return true;
}

void test_pins_and_sense() {
    Registers registers{};
    const RegisterIo io{&registers, read, nullptr};
    const Dcn32RegisterSpace space{0x34c0, 0x40000, 3, 2, 0, true};
    for (uint8_t pin = 0; pin < 5; ++pin) {
        registers.expected_offset =
            (space.segment2_dword_base + 0x1f14u + pin * 8u) * 4u;
        registers.value = 0x2u;
        expect(read_dcn32_hpd(io, space, pin) == HpdResult::connected,
               "verified HPD pin high");
        registers.value = 0x1u;
        expect(read_dcn32_hpd(io, space, pin) == HpdResult::disconnected,
               "IRQ status does not imply connected HPD");
    }
    expect(registers.reads == 10, "one read per sample");
}

void test_fail_closed() {
    Registers registers{};
    const RegisterIo io{&registers, read, nullptr};
    Dcn32RegisterSpace space{0, 0x8000, 3, 2, 0, true};
    registers.expected_offset = 0x1f14u * 4u;

    expect(read_dcn32_hpd(io, space, 5) == HpdResult::unavailable,
           "raw or out-of-range HPD ID refused");
    expect(registers.reads == 0, "invalid pin does not touch MMIO");
    space.discovery_verified = false;
    expect(read_dcn32_hpd(io, space, 0) == HpdResult::unavailable,
           "unverified discovery refused");
    space.discovery_verified = true;
    space.dmu_minor = 1;
    expect(read_dcn32_hpd(io, space, 0) == HpdResult::unavailable,
           "wrong DCN revision refused");
    space.dmu_minor = 2;
    space.aperture_bytes = registers.expected_offset + 3;
    expect(read_dcn32_hpd(io, space, 0) == HpdResult::unavailable,
           "truncated final DWORD refused");
    space.aperture_bytes = 0x8000;
    space.segment2_dword_base = UINT32_MAX;
    expect(read_dcn32_hpd(io, space, 0) == HpdResult::unavailable,
           "offset overflow refused");
    space.segment2_dword_base = 0;
    registers.fail = true;
    expect(read_dcn32_hpd(io, space, 0) == HpdResult::read_failure,
           "MMIO read failure is not disconnected");
    registers.fail = false;
    registers.value = UINT32_MAX;
    expect(read_dcn32_hpd(io, space, 0) == HpdResult::read_failure,
           "unresponsive all-ones register is not connected");
    expect(read_dcn32_hpd({}, space, 0) == HpdResult::unavailable,
           "missing callback refused");
}

} // namespace

int main() {
    test_pins_and_sense();
    test_fail_closed();
    std::printf("dcn32_hpd: %u checks, %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

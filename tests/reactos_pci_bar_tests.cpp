/*
Module: rx7900.tests.reactos_pci_bar
Purpose: Verify PCI sizing transaction ordering and rollback with failed config I/O.
Created: 2026-10-21
Architecture: Exercises the actual ReactOS C++ policy through a device model.
*/
#include "bar_probe_policy.hpp"
#include <cstdio>
#include <array>

namespace {
unsigned checks = 0;
unsigned failures = 0;
constexpr uint16_t decode_bits = 3;
constexpr uint16_t active_command = 0x107;
constexpr uint32_t sizing_write = UINT32_MAX;
constexpr pci::ProbeValue untouched{0xa5a5a5a5, 0x5a5a5a5a};

void check(bool value, const char* message) {
    ++checks;
    if (!value) {
        ++failures;
        std::fprintf(stderr, "FAIL %s\n", message);
    }
}

struct Device final {
    uint16_t command = active_command;
    uint32_t bar = 0;
    uint32_t mask = UINT32_MAX;
    unsigned calls = 0;
    unsigned fail_at = 0;
    unsigned command_writes = 0;
    bool partial_write = false;
    bool unsafe_probe = false;

    bool next() noexcept { return ++calls != fail_at; }
    static bool read_command(void* context, uint16_t& output) noexcept {
        auto& device = *static_cast<Device*>(context);
        if (!device.next()) { return false; }
        output = device.command;
        return true;
    }
    static bool write_command(void* context, uint16_t value) noexcept {
        auto& device = *static_cast<Device*>(context);
        const bool success = device.next();
        ++device.command_writes;
        if (success || device.partial_write) { device.command = value; }
        return success;
    }
    static bool read_bar(void* context, uint32_t& output) noexcept {
        auto& device = *static_cast<Device*>(context);
        if (!device.next()) { return false; }
        output = device.bar == sizing_write ? device.mask : device.bar;
        return true;
    }
    static bool write_bar(void* context, uint32_t value) noexcept {
        auto& device = *static_cast<Device*>(context);
        const bool success = device.next();
        if (success || device.partial_write) {
            device.unsafe_probe = device.unsafe_probe ||
                (value == sizing_write && (device.command & decode_bits) != 0);
            device.bar = value;
        }
        return success;
    }
    pci::ProbeAccess access() noexcept {
        return {this, read_command, write_command, read_bar, write_bar};
    }
};

void valid_transaction(uint32_t original, uint32_t mask, uint16_t command) {
    Device device{};
    device.command = command;
    device.bar = original;
    device.mask = mask;
    auto output = untouched;
    const auto result = pci::probe_bar(device.access(), output);
    check(result == pci::ProbeStatus::success, "sizing succeeds");
    check(output.original == original && output.mask == mask, "exact original and mask");
    check(device.bar == original, "original BAR restored");
    check(device.command == command, "all command bits restored");
    check(!device.unsafe_probe, "probe never decoded as a live address");
    if ((command & decode_bits) == 0) {
        check(device.command_writes == 0, "already disabled command is preserved");
    }
}

void fault_boundaries() {
    constexpr bool partial_cases[]{false, true};
    for (unsigned failure = 1; failure <= 10; ++failure) {
        for (const bool partial : partial_cases) {
            Device device{};
            device.bar = 0; // Upper half of an assigned 64-bit BAR below 4 GiB.
            device.fail_at = failure;
            device.partial_write = partial;
            auto output = untouched;
            const auto status = pci::probe_bar(device.access(), output);
            check(status != pci::ProbeStatus::success, "failed I/O is reported");
            check(output.original == untouched.original && output.mask == untouched.mask,
                  "failure does not publish output");
            check(!device.unsafe_probe, "failed probes never decode all-ones address");
            check(device.bar == 0 || (device.command & decode_bits) == 0,
                  "uncertain BAR retains disabled decoding");
        }
    }
}

struct SnapshotDevice final {
    std::array<pci::ProbeValue, pci::bar_capacity> values{};
    std::array<unsigned, pci::bar_capacity> visits{};
    unsigned calls = 0;
    unsigned fail_at = 0;

    static pci::ProbeStatus probe(void* context, uint32_t index,
                                  pci::ProbeValue& output) noexcept {
        auto& device = *static_cast<SnapshotDevice*>(context);
        ++device.calls;
        if (device.calls == device.fail_at || index >= device.values.size()) {
            return pci::ProbeStatus::read_failed;
        }
        ++device.visits.at(index);
        output = device.values.at(index);
        return pci::ProbeStatus::success;
    }
    pci::SnapshotAccess access() noexcept { return {this, probe}; }
};

SnapshotDevice lightning_device() {
    SnapshotDevice device{};
    device.values.at(0) = {0xfebf0000, 0xffffc000}; // BAR0 16-KiB control.
    device.values.at(2) = {0xfebf4000, 0xfffff000}; // BAR2 MSI-X, 4KiB.
    device.values.at(3) = {0x60000008, 0xe0000008}; // BAR3 512-MiB aperture.
    device.values.at(4) = {0x8000000c, 0xc000000c}; // BAR4/5 1-GiB 64-bit RAM.
    device.values.at(5) = {0, UINT32_MAX};
    return device;
}

void snapshot_boundaries() {
    auto device = lightning_device();
    pci::BarSnapshot snapshot{};
    check(pci::capture_bars(device.access(), 0, snapshot) == pci::SnapshotStatus::success &&
          snapshot.count == 4 && device.calls == 6,
          "Lightning resources captured exactly once, including zero-valued upper BAR");
    for (unsigned visits : device.visits) {
        check(visits == 1, "every BAR DWORD visited once, never reprobed for fill");
    }
    check(snapshot.ranges[3].base == 0x80000000ull &&
          snapshot.ranges[3].length == 0x40000000ull &&
          snapshot.ranges[3].first_bar == 4 && snapshot.ranges[3].flags == 12,
          "64-bit prefetchable BAR survives exact-width snapshot decoding");

    // This was the corruption trigger: counting stopped on a transient error,
    // then a second hardware pass recovered and filled a larger allocation.
    for (unsigned failure = 1; failure <= 6; ++failure) {
        auto failing = lightning_device();
        failing.fail_at = failure;
        snapshot.count = 77;
        snapshot.ranges[0].base = 0x1234;
        check(pci::capture_bars(failing.access(), 0, snapshot) ==
                  pci::SnapshotStatus::probe_failed && snapshot.count == 77 &&
              snapshot.ranges[0].base == 0x1234 && failing.calls == failure,
              "any transient capture failure publishes no prefix and never retries into fill");
    }

    device = {};
    device.values.at(0) = {0x0000c001, 0x0000ffe1};
    check(pci::capture_bars(device.access(), 0, snapshot) == pci::SnapshotStatus::success &&
          snapshot.count == 1 && snapshot.ranges[0].length == 32 &&
          snapshot.ranges[0].maximum_address == 0xffff,
          "legacy 16-bit I/O BAR masks remain supported");
    device = {};
    device.values.at(0) = {0x00080002, 0x000f0002};
    check(pci::capture_bars(device.access(), 0, snapshot) == pci::SnapshotStatus::success &&
          snapshot.ranges[0].length == 0x10000 &&
          snapshot.ranges[0].maximum_address == 0xfffff,
          "legacy below-one-MiB memory BAR keeps its actual address domain");
    device = {};
    device.values.at(0) = {0x80000004, 0x00000004};
    device.values.at(1) = {0, UINT32_MAX};
    snapshot.count = 77;
    check(pci::capture_bars(device.access(), 0, snapshot) ==
              pci::SnapshotStatus::length_unrepresentable && snapshot.count == 77,
          "4-GiB BAR length cannot silently narrow into ULONG zero");
    device.values.at(1).mask = 0xfffffffe;
    check(pci::capture_bars(device.access(), 0, snapshot) ==
              pci::SnapshotStatus::length_unrepresentable,
          "8-GiB aperture needs MemoryLarge support rather than truncated descriptors");
    device = {};
    device.values.at(5) = {0x80000004, 0xc0000004};
    check(pci::capture_bars(device.access(), 0, snapshot) == pci::SnapshotStatus::invalid_bar,
          "64-bit lower BAR cannot occupy the last legal DWORD");
    device = {};
    device.values.at(1) = {0x80000004, 0xc0000004};
    check(pci::capture_bars(device.access(), 1, snapshot) == pci::SnapshotStatus::invalid_bar &&
          device.calls == 2, "bridge BAR pair cannot escape its two-register layout");
    device = {};
    device.values.at(0) = {0x80000000, 0xffff5000};
    check(pci::capture_bars(device.access(), 0, snapshot) == pci::SnapshotStatus::invalid_bar,
          "non-contiguous mask cannot advertise a fabricated resource size");
    device.values.at(0) = {0x80000008, 0xffffc000};
    check(pci::capture_bars(device.access(), 0, snapshot) == pci::SnapshotStatus::invalid_bar,
          "sizing mask cannot silently alter BAR attributes");
    device = {};
    device.values.at(0) = {0x80001000, 0xffffc000};
    check(pci::capture_bars(device.access(), 0, snapshot) == pci::SnapshotStatus::invalid_bar,
          "misaligned assigned BAR cannot escape its sized alignment requirement");
    device = {};
    device.values.at(0) = {0, 0x80000000};
    check(pci::capture_bars(device.access(), 0, snapshot) == pci::SnapshotStatus::success &&
          snapshot.ranges[0].length == 0x80000000ull,
          "largest ULONG-representable power-of-two BAR remains valid");
    device = {};
    device.values.at(0) = {0xfffffffc, 0xfffff00c};
    device.values.at(1) = {UINT32_MAX, UINT32_MAX};
    check(pci::capture_bars(device.access(), 0, snapshot) == pci::SnapshotStatus::address_overflow,
          "inclusive 64-bit BAR end cannot wrap before CM descriptor publication");
    device = {};
    device.values.at(0) = {0xfffffff0, 0xffff0000};
    check(pci::capture_bars(device.access(), 0, snapshot) == pci::SnapshotStatus::address_overflow,
          "32-bit BAR end cannot exceed its address domain");
    device = {};
    for (uint32_t index = 0; index < pci::bar_capacity; ++index) {
        device.values.at(index) = {0x1000u * (index + 1u), 0xfffff000u};
    }
    check(pci::capture_bars(device.access(), 0, snapshot) == pci::SnapshotStatus::success &&
          snapshot.count == pci::bar_capacity && device.calls == pci::bar_capacity,
          "maximum six-resource capture establishes exact worst-case descriptor count");
    device = {};
    check(pci::capture_bars(device.access(), 1, snapshot) == pci::SnapshotStatus::success &&
          device.calls == 2 && snapshot.count == 0,
          "unused bridge BARs produce an empty stable capture");
    device.calls = 0;
    check(pci::capture_bars(device.access(), 2, snapshot) ==
              pci::SnapshotStatus::unsupported_header && device.calls == 0,
          "unsupported CardBus layout does not guess at type-zero BARs");
    auto invalid = device.access();
    invalid.probe = nullptr;
    check(pci::capture_bars(invalid, 0, snapshot) == pci::SnapshotStatus::invalid_access,
          "missing snapshot callback is rejected before I/O");
}
} // namespace

int main() {
    valid_transaction(0x8000000c, 0xc000000c, active_command); // 1 GiB 64-bit BAR low.
    valid_transaction(0, UINT32_MAX, active_command); // Same BAR upper dword.
    valid_transaction(0xfebf0000, 0xffffc000, active_command); // 16 KiB control BAR.
    valid_transaction(0xc001, 0xffffffe1, active_command); // I/O BAR.
    valid_transaction(0, 0, active_command); // Unimplemented BAR.
    valid_transaction(0x8000000c, 0xc000000c, 4); // Already disabled decode, bus master retained.
    fault_boundaries();
    snapshot_boundaries();
    Device device{};
    auto invalid = device.access();
    invalid.read_bar = nullptr;
    auto output = untouched;
    check(pci::probe_bar(invalid, output) == pci::ProbeStatus::invalid_access, "reject missing callback");
    check(device.calls == 0, "invalid access performs no configuration I/O");
    std::printf("ReactOS PCI BAR transaction: %u checks, %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

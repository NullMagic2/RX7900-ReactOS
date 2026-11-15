/*
Module: rx7900.tests.memory
Purpose: Verify GPUVA allocation, stale handles, leases and failed cleanup.
Created: 2026-10-21
Architecture: Explicit test backend only; no actual memory mapping or GPU access.
*/
#include <array>
#include <cstdio>

#include "manager.hpp"
#include "memory_test_backend.hpp"
#include "page_table.hpp"
#include "gfxhub.hpp"
#include "discovery_test_fixture.hpp"

namespace {

using namespace rx7900::memory;
using rx7900_test::TestBacking;
unsigned checks = 0;
unsigned failures = 0;

void expect(bool condition, const char* contract) {
    ++checks;

    if (!condition) {
        ++failures;
        std::fprintf(stderr, "FAIL: %s\n", contract);
    }
}

void check_allocation() {
    TestBacking backing{};
    Manager manager;
    Handle handle{};
    expect(manager.allocate(page_bytes, page_bytes, Access::read_write, true, handle) ==
           Status::unavailable, "allocation cannot succeed without backend");
    expect(manager.initialize({0x100000, 0x100000}, {}) == Status::unavailable,
           "absent mapping callbacks fail explicitly");
    expect(manager.initialize({UINT64_MAX - 4095, 8192}, backing.backend()) ==
           Status::invalid_argument, "reject overflowing GPU arena");
    expect(manager.initialize({0x100001, 0x100000}, backing.backend()) ==
           Status::invalid_argument, "arena requires page alignment");
    expect(manager.initialize({0x100000, 0x100000}, backing.backend()) == Status::success,
           "initialize checked virtual arena");
    expect(manager.allocate(4095, page_bytes, Access::read_write, true, handle) ==
           Status::invalid_argument, "reject partial-page backing request");
    expect(manager.allocate(page_bytes, 12288, Access::read_write, true, handle) ==
           Status::invalid_argument, "alignment must be power of two");
    expect(manager.allocate(page_bytes, page_bytes, static_cast<Access>(4), false, handle) ==
           Status::invalid_argument, "reject invalid permission bits");
    expect(manager.allocate(page_bytes, page_bytes, Access::read_write, true, handle) ==
           Status::success, "allocate a mapped CPU-visible page");
    const Handle first = handle;
    AllocationInfo info{};
    expect(manager.describe(first, info) == Status::success &&
           info.gpu.virtual_address == 0x100000 && info.backing.cpu_bytes != nullptr,
           "descriptor preserves distinct GPUVA and CPU mapping");
    expect(manager.initialize({0x200000, 0x100000}, backing.backend()) == Status::busy,
           "cannot replace backend with live resources");
    expect(manager.allocate(page_bytes, 0x10000, Access::read, false, handle) ==
           Status::success, "align next GPU allocation without assuming contiguous addresses");
    const Handle second = handle;
    expect(manager.describe(second, info) == Status::success &&
           info.gpu.virtual_address == 0x110000, "GPUVA alignment is honored");
    rx7900::sdma::GpuRange resolved{};
    expect(manager.resolve({second, 4095}, 1, Access::read, resolved) == Status::success &&
           resolved.byte_offset == 4095, "last mapped byte is readable");
    expect(manager.resolve({second, 4095}, 2, Access::read, resolved) ==
           Status::invalid_argument, "cannot resolve outside mapping");
    expect(manager.resolve({second, UINT64_MAX}, 1, Access::read, resolved) ==
           Status::invalid_argument, "offset arithmetic cannot wrap");
    expect(manager.resolve({second, 0}, 1, Access::write, resolved) == Status::access_denied,
           "GPU permissions cannot be widened by resolve");
    expect(manager.retain(second, Access::write) == Status::access_denied,
           "lease cannot widen access");
    expect(manager.release(first) == Status::success, "release idle mapping");
    expect(manager.allocate(page_bytes, page_bytes, Access::read_write, true, handle) ==
           Status::success, "reuse first-fit hole");
    expect(handle.slot == first.slot && handle.generation != first.generation,
           "reused slots have new generation");
    expect(manager.resolve({first, 0}, 1, Access::read, resolved) == Status::stale_handle,
           "stale handle cannot reach reused backing");
    expect(manager.shutdown() == Status::success && manager.empty() &&
           backing.creates == backing.destroys, "shutdown destroys all idle backing");
    expect(manager.initialize({0x100000, page_bytes}, backing.backend()) == Status::success,
           "empty manager may be reinitialized");
    expect(manager.allocate(page_bytes, page_bytes, Access::read, false, handle) ==
           Status::success && handle.generation > second.generation,
           "generations persist across shutdown/reinitialize");
    Handle extra{};
    expect(manager.allocate(page_bytes, page_bytes, Access::read, false, extra) ==
           Status::out_of_memory, "arena exhaustion rejects allocation");
    expect(manager.shutdown() == Status::success, "release reinitialized mapping");
}

void check_lifetime() {
    TestBacking backing{};
    Manager manager;
    Handle handle{};
    expect(manager.initialize({0x100000, 0x100000}, backing.backend()) == Status::success,
           "lifetime manager initialized");
    expect(manager.allocate(page_bytes, page_bytes, Access::read_write, true, handle) ==
           Status::success, "lifetime page allocated");
    expect(manager.retain(handle, Access::read) == Status::success &&
           manager.retain(handle, Access::write) == Status::success, "two hardware leases retained");
    expect(manager.shutdown() == Status::busy && backing.destroys == 0,
           "shutdown preserves pinned backing");
    expect(manager.release(handle) == Status::deferred && backing.destroys == 0,
           "release defers destruction while GPU can access backing");
    rx7900::sdma::GpuRange range{};
    expect(manager.resolve({handle, 0}, 4, Access::read, range) == Status::busy &&
           manager.retain(handle, Access::read) == Status::busy,
           "retirement forbids new work");
    expect(manager.unretain(handle) == Status::success && backing.destroys == 0,
           "one remaining lease keeps mapping alive");
    expect(manager.unretain(handle) == Status::success && backing.destroys == 1,
           "last completed hardware lease releases mapping");
    expect(manager.unretain(handle) == Status::stale_handle,
           "double release cannot touch freed backing");
    expect(manager.shutdown() == Status::success, "retired manager shuts down");
}

void check_failures() {
    TestBacking backing{};
    Manager manager;
    Handle handle{};
    expect(manager.initialize({0x100000, 0x100000}, backing.backend()) == Status::success,
           "failure manager initialized");
    backing.fail_create = true;
    expect(manager.allocate(page_bytes, page_bytes, Access::read_write, true, handle) ==
           Status::backend_failure && manager.empty(), "failed real mapping does not report allocation");
    backing.fail_create = false;
    backing.invalid_cpu_descriptor = true;
    expect(manager.allocate(page_bytes, page_bytes, Access::read_write, true, handle) ==
           Status::backend_failure && manager.empty() && backing.destroys == 1,
           "bad native descriptor is rolled back");
    backing.invalid_cpu_descriptor = false;
    expect(manager.allocate(page_bytes, page_bytes, Access::read_write, true, handle) ==
           Status::success, "retry after backend failure");
    backing.fail_destroy = true;
    expect(manager.release(handle) == Status::quarantined && !manager.empty(),
           "failed unmap retains resource ownership");
    Handle extra{};
    expect(manager.allocate(page_bytes, page_bytes, Access::read_write, true, extra) ==
           Status::quarantined, "quarantine blocks new work");
    expect(manager.shutdown() == Status::quarantined && !manager.empty(),
           "shutdown cannot fake successful cleanup");
    backing.fail_destroy = false;
    expect(manager.release(handle) == Status::success && manager.empty(),
           "confirmed cleanup clears quarantine");
    expect(manager.shutdown() == Status::success, "clean failure manager shutdown");

    expect(manager.initialize({0x100000, 0x100000}, backing.backend()) == Status::success,
           "restart manager to test failed-create quarantine");
    expect(manager.allocate(page_bytes, page_bytes, Access::read_write, true, handle) ==
           Status::success, "capture pre-quarantine generation");
    const uint64_t previous_generation = handle.generation;
    expect(manager.release(handle) == Status::success, "release pre-quarantine mapping");
    backing.quarantine_create = true;
    backing.fail_destroy = true;
    const auto previous_destroys = backing.destroys;
    expect(manager.allocate(page_bytes, page_bytes, Access::read_write, true, handle) ==
           Status::quarantined && handle.generation == 0 && !manager.empty(),
           "uncertain map publishes no handle and retains resource ownership");
    expect(manager.allocate(page_bytes, page_bytes, Access::read_write, true, extra) ==
           Status::quarantined, "uncertain create prevents GPUVA slot reuse");
    expect(manager.shutdown() == Status::quarantined && backing.destroys == previous_destroys,
           "failed TLB cleanup cannot free quarantined create backing");
    backing.fail_destroy = false;
    backing.quarantine_create = false;
    expect(manager.shutdown() == Status::success && manager.empty() &&
           backing.destroys == previous_destroys + 1,
           "confirmed recovery releases failed-create backing exactly once");
    expect(manager.initialize({0x100000, 0x100000}, backing.backend()) == Status::success &&
           manager.allocate(page_bytes, page_bytes, Access::read_write, true, handle) ==
           Status::success && handle.generation > previous_generation + 1,
           "quarantined generation is consumed and never reused");
    expect(manager.shutdown() == Status::success, "failed-create recovery complete");
}

void check_capacity() {
    TestBacking backing{};
    Manager manager;
    std::array<Handle, allocation_capacity> handles{};
    expect(manager.initialize({0x100000, page_bytes * allocation_capacity}, backing.backend()) ==
           Status::success, "capacity arena initialized");

    for (auto& handle : handles) {
        expect(manager.allocate(page_bytes, page_bytes, Access::read, false, handle) ==
               Status::success, "allocate every metadata slot");
    }

    Handle extra{};
    expect(manager.allocate(page_bytes, page_bytes, Access::read, false, extra) ==
           Status::out_of_memory, "metadata capacity is bounded");
    expect(manager.shutdown() == Status::success && backing.destroys == allocation_capacity,
           "capacity shutdown releases every resource once");
}

void check_page_tables() {
    uint64_t pte = 0;
    expect(prepare_system_pte(0x123456000ULL, Access::read_write, pte) == PteStatus::prepared &&
           pte == 0x0003000123456067ULL, "GFX11 PTE preserves above-4GiB DMA bus address");
    expect(prepare_system_pte(0x1000, Access::read, pte) == PteStatus::prepared &&
           pte == 0x0003000000001027ULL, "read-only PTE has no write/execute bits");
    expect(prepare_system_pte(0x1000, Access::write, pte) == PteStatus::prepared &&
           pte == 0x0003000000001047ULL, "write-only PTE cannot acquire read permission");
    expect(prepare_system_pte(0x1001, Access::read_write, pte) == PteStatus::invalid_address &&
           pte == 0, "unaligned DMA page cannot set reserved PTE bits");
    expect(prepare_system_pte(0x1000000000000ULL, Access::read, pte) ==
           PteStatus::invalid_address, "DMA address cannot escape low48 PTE address field");
    expect(prepare_system_pte(0x1000, static_cast<Access>(0), pte) ==
           PteStatus::invalid_access, "no-access page list cannot become valid PTE");
    std::array<uint8_t, 16> pages{};
    rx7900::sdma::CommandBuffer page_list(pages.data(), pages.size());
    page_list.write_u32(0, 0x23456000);
    page_list.write_u32(4, 1);
    page_list.write_u32(8, 0x8000);
    page_list.write_u32(12, 0);
    std::array<uint8_t, 16> output{};
    PageTableResult result{};
    expect(encode_system_ptes(0x100000, {pages.data(), pages.size()}, Access::read_write,
           {output.data(), output.size()}, result) == PteStatus::prepared &&
           result.virtual_byte_count == 8192 && result.bytes_written == 16,
           "prepare consecutive GPU leaf entries for noncontiguous DMA pages");
    uint32_t low = 0;
    uint32_t high = 0;
    const rx7900::ByteView encoded(output.data(), output.size());
    expect(encoded.read_u32(0, low) && encoded.read_u32(4, high) &&
           low == 0x23456067 && high == 0x30001,
           "PTE bytes are explicit little-endian with verified MTYPE_UC3");

    for (size_t capacity = 0; capacity < output.size(); ++capacity) {
        output.fill(0xa5);
        expect(encode_system_ptes(0x100000, {pages.data(), pages.size()}, Access::read_write,
               {output.data(), capacity}, result) == PteStatus::output_too_small &&
               result.bytes_written == 0 && result.bytes_required == output.size(),
               "short PTE output rejected before any write");

        for (const auto byte : output) {
            expect(byte == 0xa5, "PTE capacity failure preserves old table");
        }
    }

    page_list.write_u32(8, 0x8001);
    output.fill(0xa5);
    expect(encode_system_ptes(0x100000, {pages.data(), pages.size()}, Access::read_write,
           {output.data(), output.size()}, result) == PteStatus::invalid_address,
           "bad trailing DMA page rejects complete table update");

    for (const auto byte : output) {
        expect(byte == 0xa5, "invalid trailing PTE cannot leave a mapped prefix");
    }

    expect(encode_system_ptes(0x100000, {pages.data(), 15}, Access::read_write,
           {output.data(), output.size()}, result) == PteStatus::invalid_page_list,
           "truncated DMA address rejected");
    expect(encode_system_ptes(0xfffffffff000ULL, {pages.data(), pages.size()}, Access::read_write,
           {output.data(), output.size()}, result) == PteStatus::invalid_address,
           "GPUVA page span cannot overflow48 bits");
}

using rx7900_test::discovery_u16;
using rx7900_test::discovery_checksums;
using rx7900_test::discovery_binary;

void check_discovery() {
    auto bytes = discovery_binary();
    GcRegisterBase base;
    expect(validate_gc_discovery({bytes.data(), bytes.size()}, 0x40000, base) ==
           GfxhubStatus::discovery_validated && base.valid() && base.segment(0) == 0x1000 &&
           base.segment(1) == 0x2000, "validate checksummed GC11 discovery register segments");

    for (size_t count = 0; count < 160; ++count) {
        expect(validate_gc_discovery({bytes.data(), count}, 0x40000, base) ==
               GfxhubStatus::invalid_discovery && !base.valid(),
               "every truncated discovery binary fails without a valid token");
    }

    bytes.at(159) ^= 1;
    expect(validate_gc_discovery({bytes.data(), bytes.size()}, 0x40000, base) ==
           GfxhubStatus::invalid_discovery, "invalid binary checksum cannot authorize register writes");
    bytes = discovery_binary();
    bytes.at(14) ^= 1;
    discovery_checksums(bytes, false);
    expect(validate_gc_discovery({bytes.data(), bytes.size()}, 0x40000, base) ==
           GfxhubStatus::invalid_discovery, "valid outer checksum cannot bypass IP table checksum");
    bytes = discovery_binary(true);
    expect(validate_gc_discovery({bytes.data(), bytes.size()}, 0x40000, base) ==
           GfxhubStatus::discovery_validated, "support local 64-bit discovery base records without narrowing");
    bytes.at(156) = 1;
    discovery_checksums(bytes);
    expect(validate_gc_discovery({bytes.data(), bytes.size()}, 0x40000, base) ==
           GfxhubStatus::unsupported && !base.valid(), "remote/high discovery address cannot be truncated");
    bytes = discovery_binary();
    bytes.at(148) = 12;
    discovery_checksums(bytes);
    expect(validate_gc_discovery({bytes.data(), bytes.size()}, 0x40000, base) ==
           GfxhubStatus::unsupported, "other GC generation cannot use GFX11 register protocol");
    bytes = discovery_binary();
    bytes.at(147) = 255;
    discovery_checksums(bytes);
    expect(validate_gc_discovery({bytes.data(), bytes.size()}, 0x40000, base) ==
           GfxhubStatus::invalid_discovery, "base array cannot escape IP table extent");
    bytes = discovery_binary();
    discovery_u16(bytes, 76, 60);
    discovery_checksums(bytes);
    expect(validate_gc_discovery({bytes.data(), bytes.size()}, 0x40000, base) ==
           GfxhubStatus::invalid_discovery, "die cannot overlap discovery header");
    bytes = discovery_binary();
    expect(validate_gc_discovery({bytes.data(), bytes.size()}, 0x4000, base) ==
           GfxhubStatus::invalid_discovery, "GC register extent must fit assigned MMIO aperture");
}

struct TestGfxhub final {
    // Test-only register model. Its generated ACKs validate protocol sequencing,
    // not a physical GPU. Production read/write callbacks must perform MMIO.
    std::array<uint32_t, 0x10000> registers{};
    std::array<uint8_t, 4096> table{};
    bool reads_ok = true;
    bool writes_ok = true;
    bool publication_ok = true;
    bool auto_ack = true;
    bool ack_pending = false;
    uint32_t ack_delay = 0;
    uint32_t corrupt_read_register = UINT32_MAX;
    unsigned writes = 0;
    unsigned publications = 0;
    unsigned last_enable_publication = 0;
    unsigned stalls = 0;
    bool enabled_before_publication = false;

    TestGfxhub() {
        table.fill(0xa5);
        registers.at(0x1000 + 0x1678) = 0x100;
        registers.at(0x1000 + 0x1679) = 0x6ff;
        registers.at(0x1000 + 0x167d) = 0x4000;
        registers.at(0x1000 + 0x167e) = 0x1bfff;
        registers.at(0x1000 + 0x167b) = 0xffffff;
        registers.at(0x1000 + 0x167f) = 0x1859;
        registers.at(0x1000 + 0x15bc) = 1;
        registers.at(0x1000 + 0x15be) = 9 | (6 << 15);
        registers.at(0x1000 + 0x25) = 1;
        registers.at(0x1000 + 0x625) = 1;
        registers.at(0x2000 + 0x589a) = 1;
        registers.at(0x2000 + 0x58ba) = 1;
    }

    static bool read(void* context, uint32_t byte_offset, uint32_t& value) noexcept {
        auto& self = *static_cast<TestGfxhub*>(context);
        const size_t index = byte_offset / 4;

        if (!self.reads_ok || (byte_offset & 3) != 0 || index >= self.registers.size()) {
            return false;
        }

        if (index == 0x1000 + 0x16ce && self.ack_pending && self.auto_ack) {
            if (self.ack_delay == 0) {
                self.registers.at(index) = 1;
                self.ack_pending = false;
            } else {
                --self.ack_delay;
            }
        }

        value = self.registers.at(index);

        if (index == self.corrupt_read_register) {
            value ^= 1;
        }

        return true;
    }

    static bool write(void* context, uint32_t byte_offset, uint32_t value) noexcept {
        auto& self = *static_cast<TestGfxhub*>(context);
        const size_t index = byte_offset / 4;

        if (!self.writes_ok || (byte_offset & 3) != 0 || index >= self.registers.size()) {
            return false;
        }

        ++self.writes;
        self.registers.at(index) = value;

        if (index == 0x1000 + 0x1688 && (value & 1) != 0) {
            if (self.publications == self.last_enable_publication) {
                self.enabled_before_publication = true;
            }
            self.last_enable_publication = self.publications;
        }

        if (index == 0x1000 + 0x16bc) {
            self.ack_pending = true;
            self.registers.at(0x1000 + 0x16ce) = 0;
        }

        return true;
    }

    static bool publish(void* context) noexcept {
        auto& self = *static_cast<TestGfxhub*>(context);
        ++self.publications;
        return self.publication_ok;
    }

    static void stall(void* context) noexcept {
        ++static_cast<TestGfxhub*>(context)->stalls;
    }

    GfxhubIo io() noexcept { return {this, read, write, publish, stall}; }

    FlatGart configuration() noexcept {
        return {{0x1000000000ULL, 2 * 1024 * 1024}, table.data(), table.size(),
                0x300000000ULL, 0x100000000ULL, 0x600000000ULL, 3};
    }
};

GcRegisterBase test_gc_base() {
    const auto binary = discovery_binary();
    GcRegisterBase base;
    expect(validate_gc_discovery({binary.data(), binary.size()}, 0x40000, base) ==
           GfxhubStatus::discovery_validated, "test register token validated from discovery bytes");
    return base;
}

void check_gfxhub_mapping() {
    TestGfxhub hardware;
    Gfxhub hub;
    hardware.ack_delay = 2;
    expect(hub.initialize(test_gc_base(), hardware.configuration(), hardware.io()) ==
           GfxhubStatus::configured && hardware.stalls == 2,
           "VMID0 setup waits for actual backend ACK instead of assuming completion");
    expect(hardware.registers.at(0x1000 + 0x16f3) == 7 &&
           hardware.registers.at(0x1000 + 0x16f4) == 3,
           "system-memory table root stores low48 DMA address plus PDE flags");
    expect(hardware.registers.at(0x1000 + 0x1713) == 0x1000000 &&
           hardware.registers.at(0x1000 + 0x1733) == 0x10001ff &&
           hardware.registers.at(0x1000 + 0x16bc) == 0xf80001 &&
           hardware.registers.at(0x1000 + 0x16f1) == UINT32_MAX &&
           hardware.registers.at(0x1000 + 0x16f2) == 0x1f,
           "flat aperture and engine17 request match pinned Linux register encoding");

    for (const auto byte : hardware.table) {
        expect(byte == 0, "initial table cannot expose old physical mappings");
    }

    std::array<uint8_t, 8> pages{};
    rx7900::sdma::CommandBuffer addresses(pages.data(), pages.size());
    addresses.write_u32(4, 2);
    const uint64_t gpu = hardware.configuration().virtual_aperture.virtual_address;
    expect(hub.map(gpu, {pages.data(), pages.size()}, Access::read_write) == GfxhubStatus::mapped,
           "map prepares shared PTE and confirms hardware invalidation");
    uint32_t low = 0;
    uint32_t high = 0;
    const rx7900::ByteView table(hardware.table.data(), hardware.table.size());
    expect(table.read_u32(0, low) && table.read_u32(4, high) && low == 0x67 && high == 0x30002,
           "actual table bytes reuse checked GFX11 system PTE encoder");
    const auto writes = hardware.writes;
    expect(hub.map(gpu, {pages.data(), pages.size()}, Access::read_write) == GfxhubStatus::busy &&
           hardware.writes == writes, "mapped leaf cannot silently replace live backing");
    expect(hub.unmap(gpu, 4096) == GfxhubStatus::unmapped &&
           table.read_u32(0, low) && low == 0, "unmap clears leaf and confirms TLB ACK");
    expect(hub.unmap(gpu, 4096) == GfxhubStatus::invalid_argument,
           "double unmap cannot imply ownership of a missing page");
    addresses.write_u32(4, 3);
    expect(hub.map(gpu, {pages.data(), pages.size()}, Access::read_write) ==
           GfxhubStatus::invalid_argument, "client cannot map privileged page-table backing");
    addresses.write_u32(4, 2);
    hardware.registers.at(0x1000 + 0x0da4) = 0x80000000;
    expect(hub.map(gpu, {pages.data(), pages.size()}, Access::read) == GfxhubStatus::busy,
           "GPU-active state prevents page-table mutation");
    hardware.registers.at(0x1000 + 0x0da4) = 0;
    hardware.registers.at(0x2000 + 0x58ba) = 0;
    expect(hub.map(gpu, {pages.data(), pages.size()}, Access::read) == GfxhubStatus::busy,
           "idle but unhalted SDMA cannot race PTE updates");
    hardware.registers.at(0x2000 + 0x58ba) = 1;
    expect(!hardware.enabled_before_publication,
           "CPU/HDP publication occurs before translation is reenabled");
    expect(hub.disable() == GfxhubStatus::disabled &&
           hardware.registers.at(0x1000 + 0x1688) == 0,
           "confirmed disable retires translation before native table release");
}

void check_gfxhub_failure() {
    TestGfxhub hardware;
    Gfxhub hub;
    expect(hub.initialize({}, hardware.configuration(), hardware.io()) ==
           GfxhubStatus::unavailable && hardware.writes == 0,
           "unvalidated register base cannot access hardware");
    hardware.registers.at(0x1000 + 0x167f) = 0;
    expect(hub.initialize(test_gc_base(), hardware.configuration(), hardware.io()) ==
           GfxhubStatus::dependencies_unmet && hardware.writes == 0 &&
           hardware.table.front() == 0xa5, "missing native L1/cache setup fails before mutation");
    hardware.registers.at(0x1000 + 0x167f) = 0x1859;
    hardware.registers.at(0x1000 + 0x1688) = 1;
    expect(hub.initialize(test_gc_base(), hardware.configuration(), hardware.io()) ==
           GfxhubStatus::busy && hardware.writes == 0,
           "cannot take over an active VMID0 context");
    hardware.registers.at(0x1000 + 0x1688) = 0;
    expect(hub.initialize(test_gc_base(), hardware.configuration(), hardware.io()) ==
           GfxhubStatus::configured, "initialize after prerequisites become valid");
    hardware.auto_ack = false;
    std::array<uint8_t, 8> pages{};
    rx7900::sdma::CommandBuffer(pages.data(), pages.size()).write_u32(4, 2);
    expect(hub.map(0x1000000000ULL, {pages.data(), pages.size()}, Access::read) ==
           GfxhubStatus::quarantined && hardware.stalls == 3,
           "missing TLB ACK quarantines possible live mappings after bounded poll");
    expect(hub.map(0x1000001000ULL, {pages.data(), pages.size()}, Access::read) ==
           GfxhubStatus::quarantined && hub.disable() == GfxhubStatus::quarantined,
           "timeout forbids new mappings or fabricated resource release");
    hardware.auto_ack = true;
    expect(hub.disable() == GfxhubStatus::disabled,
           "only confirmed later disable ends table quarantine");

    hardware.corrupt_read_register = 0x1000 + 0x16f4;
    expect(hub.initialize(test_gc_base(), hardware.configuration(), hardware.io()) ==
           GfxhubStatus::quarantined, "failed root-register readback cannot claim configured translation");
    hardware.corrupt_read_register = UINT32_MAX;
    expect(hub.disable() == GfxhubStatus::disabled,
           "readback failure requires confirmed disable before table release");
    hardware.reads_ok = false;
    const auto writes = hardware.writes;
    expect(hub.initialize(test_gc_base(), hardware.configuration(), hardware.io()) ==
           GfxhubStatus::mmio_failure && hardware.writes == writes,
           "MMIO acquisition failure does not issue speculative setup writes");
    hardware.reads_ok = true;
    expect(hub.initialize(test_gc_base(), hardware.configuration(), hardware.io()) ==
           GfxhubStatus::configured, "restart after confirmed root cleanup");
    hardware.publication_ok = false;
    expect(hub.map(0x1000000000ULL, {pages.data(), pages.size()}, Access::read) ==
           GfxhubStatus::quarantined && hardware.registers.at(0x1000 + 0x1688) == 0,
           "failed CPU/HDP visibility cannot enable translation to new PTEs");
    hardware.publication_ok = true;
    expect(hub.disable() == GfxhubStatus::disabled,
           "visibility failure retains backing until confirmed later cleanup");
}

} // namespace

int main() {
    check_allocation();
    check_lifetime();
    check_failures();
    check_capacity();
    check_page_tables();
    check_discovery();
    check_gfxhub_mapping();
    check_gfxhub_failure();
    std::printf("%u memory checks; %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

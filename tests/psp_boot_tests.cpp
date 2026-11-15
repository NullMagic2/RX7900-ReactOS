/*
Module: rx7900.tests.psp_boot
Purpose: Verify PSP13 mailbox traces, exact statuses, timing and retained ownership.
Created: 2026-10-21
Architecture: Explicit synthetic register machine; no firmware is sent to hardware.
*/
#include <array>
#include <cstdio>
#include <vector>

#include "psp_boot.hpp"
#include "discovery_test_fixture.hpp"

namespace {

using namespace rx7900::hardware;
using rx7900::ByteView;
using rx7900::sdma::CommandBuffer;
unsigned checks = 0;
unsigned failures = 0;

void expect(bool condition, const char* contract) {
    ++checks;

    if (!condition) {
        ++failures;
        std::fprintf(stderr, "FAIL: %s\n", contract);
    }
}

PspRegisters psp_registers() {
    const auto binary = rx7900_test::discovery_binary(false, discovery_mp0_hardware_id, 13, 0x100);
    PspRegisters registers;
    expect(validate_psp_discovery({binary.data(), binary.size()}, 0x4000, registers) ==
           DiscoveryStatus::validated, "synthetic checksummed MP0 discovery authorizes test backend");
    return registers;
}

void write_u16(std::vector<uint8_t>& bytes, size_t offset, uint16_t value) {
    bytes.at(offset) = static_cast<uint8_t>(value);
    bytes.at(offset + 1) = static_cast<uint8_t>(value >> 8);
}

template<size_t count>
std::vector<uint8_t> firmware_fixture(const std::array<uint32_t, count>& identifiers,
                                    uint32_t part_bytes = 16) {
    std::vector<uint8_t> bytes(256 + count * part_bytes);
    CommandBuffer output(bytes.data(), bytes.size());
    output.write_u32(0, static_cast<uint32_t>(bytes.size()));
    output.write_u32(4, 36);
    write_u16(bytes, 8, 2);
    write_u16(bytes, 12, 13);
    output.write_u32(16, 0x310035);
    output.write_u32(20, static_cast<uint32_t>(count * part_bytes));
    output.write_u32(24, 256);
    output.write_u32(32, static_cast<uint32_t>(count));

    for (size_t index = 0; index < count; ++index) {
        output.write_u32(36 + index * 16, identifiers.at(index));
        output.write_u32(40 + index * 16, 0x310035);
        output.write_u32(44 + index * 16, static_cast<uint32_t>(index * part_bytes));
        output.write_u32(48 + index * 16, part_bytes);

        for (size_t word = 0; word < 4; ++word) {
            output.write_u32(256 + index * part_bytes + word * 4,
                0x11220000 + identifiers.at(index) * 0x100 + static_cast<uint32_t>(word));
        }
    }

    return bytes;
}

struct TestPsp final {
    enum class EventKind { read, write, publish, delay, reset };
    struct Event final { EventKind kind; uint32_t reg; uint32_t value; };

    // Fixed-size host-owned test storage uses RAII heap allocation to avoid a
    // 1 MiB stack frame on Win32. This is not a production backing allocator.
    std::vector<uint8_t> staging = std::vector<uint8_t>(psp_staging_bytes + 16, 0xa5);
    std::array<uint32_t, 0x1000> registers{};
    std::array<Event, 256> events{};
    size_t event_count = 0;
    bool used = false;
    bool destroy_ok = true;
    bool reads_ok = true;
    bool writes_ok = true;
    bool publish_ok = true;
    bool reset_ok = false;
    bool complete_components = true;
    bool start_sos = true;
    bool sos_settled = false;
    bool command_write_uncertain = false;
    bool corrupt_address = false;
    bool pending = false;
    uint32_t pending_command = 0;
    uint32_t reply_delay = 0;
    uint32_t component_reply = 0x80000000;
    unsigned destroys = 0;
    unsigned command_writes = 0;
    unsigned publications = 0;
    unsigned one_us_delays = 0;
    unsigned sos_delays = 0;
    bool command_before_publication = false;
    uint32_t staged_first_word = 0;
    uint64_t expected_mc_address = 0x40000000;

    TestPsp() { registers.at(0x163) = 0x80000000; }

    void record(EventKind kind, uint32_t reg, uint32_t value) noexcept {
        if (event_count < events.size()) {
            events.at(event_count++) = {kind, reg, value};
        }
    }

    static rx7900::memory::CreateResult create(void* context, rx7900::sdma::GpuAllocation gpu,
        rx7900::memory::Access, bool cpu_visible, rx7900::memory::Backing& backing) noexcept {
        auto& self = *static_cast<TestPsp*>(context);

        if (self.used || !cpu_visible || gpu.byte_length > psp_staging_bytes) {
            return rx7900::memory::CreateResult::no_resources;
        }

        self.used = true;
        backing = {&self, self.staging.data(), self.staging.size()};
        return rx7900::memory::CreateResult::created;
    }

    static bool destroy(void* context, const rx7900::memory::Backing& backing,
                         rx7900::sdma::GpuAllocation) noexcept {
        auto& self = *static_cast<TestPsp*>(context);

        if (backing.token != &self || !self.used || !self.destroy_ok) {
            return false;
        }

        self.used = false;
        ++self.destroys;
        return true;
    }

    static bool read(void* context, uint32_t offset, uint32_t& value) noexcept {
        auto& self = *static_cast<TestPsp*>(context);
        const size_t reg = offset / 4;

        if (!self.reads_ok || (offset & 3) != 0 || reg >= self.registers.size()) {
            return false;
        }

        if (self.pending && reg == 0x163 && self.pending_command != 0x20000 &&
            self.complete_components) {
            if (self.reply_delay == 0) {
                self.registers.at(reg) = self.component_reply;
                self.pending = false;
            } else {
                --self.reply_delay;
            }
        }

        if (self.pending && reg == 0x191 && self.pending_command == 0x20000 &&
            self.start_sos && self.sos_settled) {
            self.registers.at(reg) = 0x55aa33cc;
            self.registers.at(0x17a) = 0x310035;
            self.pending = false;
        }

        value = self.registers.at(reg);

        if (reg == 0x164 && self.corrupt_address) {
            value ^= 1;
        }

        self.record(EventKind::read, static_cast<uint32_t>(reg), value);
        return true;
    }

    static bool write(void* context, uint32_t offset, uint32_t value) noexcept {
        auto& self = *static_cast<TestPsp*>(context);
        const size_t reg = offset / 4;

        if (!self.writes_ok || (offset & 3) != 0 || reg >= self.registers.size()) {
            return false;
        }

        self.registers.at(reg) = value;
        self.record(EventKind::write, static_cast<uint32_t>(reg), value);

        if (reg == 0x163) {
            ++self.command_writes;
            self.pending_command = value;
            self.pending = true;
            self.sos_settled = false;
            self.command_before_publication = self.command_before_publication || self.publications == 0;
            ByteView(self.staging.data(), self.staging.size()).read_u32(0, self.staged_first_word);
            return !self.command_write_uncertain;
        }

        return true;
    }

    static bool publish(void* context, PspMcAddress address, size_t bytes) noexcept {
        auto& self = *static_cast<TestPsp*>(context);
        ++self.publications;
        self.record(EventKind::publish, 0, static_cast<uint32_t>(bytes));
        return self.publish_ok && address.value == self.expected_mc_address && bytes == psp_staging_bytes;
    }

    static void delay(void* context, uint32_t microseconds) noexcept {
        auto& self = *static_cast<TestPsp*>(context);
        self.record(EventKind::delay, 0, microseconds);

        if (microseconds == 20000) {
            ++self.sos_delays;
            self.sos_settled = true;
        } else if (microseconds == 1) {
            ++self.one_us_delays;
        }
    }

    static bool reset(void* context) noexcept {
        auto& self = *static_cast<TestPsp*>(context);
        self.record(EventKind::reset, 0, self.reset_ok ? 1 : 0);

        if (self.reset_ok) {
            self.pending = false;
            self.registers.at(0x163) = 0x80000000;
            self.registers.at(0x191) = 0;
        }

        return self.reset_ok;
    }

    PspBootIo io() noexcept { return {this, read, write, publish, delay, reset}; }
    rx7900::memory::Backend backing() noexcept { return {this, create, destroy}; }
};

struct Fixture final {
    TestPsp hardware;
    rx7900::memory::Manager manager;
    rx7900::memory::Handle staging{};
    PspBoot boot;
    std::vector<uint8_t> firmware = firmware_fixture(std::array<uint32_t, 6>{2, 1, 4, 3, 5, 6});

    void initialize(uint64_t size = psp_staging_bytes, uint64_t base = 0x40000000) {
        using rx7900::memory::Status;
        hardware.expected_mc_address = base;
        expect(manager.initialize({base, psp_staging_bytes * 4}, hardware.backing()) == Status::success,
               "test staging manager initialized");
        expect(manager.allocate(size, 4096, rx7900::memory::Access::read_write, true, staging) ==
               Status::success, "test MC staging backing allocated");
        expect(boot.initialize(psp_registers(), hardware.io(), {3, 3}) ==
               PspBootStatus::bootloader_ready, "actual modeled mailbox exact READY observed");
    }

    PspBootStatus prepare(uint64_t mc_address = 0x40000000) {
        return boot.prepare({firmware.data(), firmware.size()}, manager, staging, {mc_address});
    }

    void cleanup_idle() {
        if (boot.report().staging_retained) {
            expect(boot.release_idle_staging() == PspBootStatus::staging_released,
                   "idle staging release requires observed mailbox completion");
        }
        expect(manager.shutdown() == rx7900::memory::Status::success,
               "test backing shutdown after protocol ownership retired");
    }

    void cleanup_reset() {
        hardware.reset_ok = true;
        expect(boot.release_after_reset() == PspBootStatus::staging_released,
               "native confirmed reset releases uncertain staging lease");
        expect(manager.shutdown() == rx7900::memory::Status::success,
               "quarantined test backing shutdown after reset proof");
    }
};

void check_boot_order() {
    Fixture fixture;
    fixture.initialize();
    expect(fixture.prepare() == PspBootStatus::prepared && fixture.hardware.command_writes == 0,
           "container preparation never implies hardware upload");
    const std::array<uint32_t, 4> commands{0x80000, 0x10000000, 0x10000, 0x20000};
    const std::array<uint32_t, 4> payloads{3, 3, 2, 1};

    for (size_t step = 0; step < commands.size(); ++step) {
        expect(fixture.boot.submit_next() == PspBootStatus::submitted,
               "ordered command is submitted, not reported completed");
        const auto report = fixture.boot.report();
        expect(report.completed_steps == step && report.payload_identifier == payloads.at(step) &&
               report.staging_retained && fixture.hardware.registers.at(0x164) == 0x400 &&
               fixture.hardware.registers.at(0x163) == commands.at(step),
               "mailbox address units/command/payload match pinned Linux protocol");
        expect(fixture.hardware.staged_first_word == 0x11220000 + payloads.at(step) * 0x100,
               "SPL command stages KDB bytes exactly as pinned upstream");
        expect(fixture.boot.submit_next() == PspBootStatus::busy,
               "pending command cannot overwrite shared PSP staging");

        const bool sos = step + 1 == commands.size();

        if (sos) {
            expect(fixture.manager.release(fixture.staging) == rx7900::memory::Status::deferred &&
                   fixture.manager.shutdown() == rx7900::memory::Status::busy,
                   "client cannot free staging while secure OS may still read it");
        }

        const auto status = fixture.boot.wait();
        expect(status == (sos ? PspBootStatus::secure_os_running : PspBootStatus::component_completed),
               "completion status comes from exact READY or new secure-OS life");
    }

    const auto report = fixture.boot.report();
    expect(report.completed_steps == 4 && report.sos_life == 0x55aa33cc &&
           report.sos_version == 0x310035 && !report.staging_retained &&
           fixture.hardware.destroys == 1, "observed SOS completion releases retired staging exactly once");
    expect(fixture.hardware.sos_delays == 1 && !fixture.hardware.command_before_publication,
           "SOS settling delay and publication precede hardware completion observation");

    bool saw_sos_command = false;
    bool saw_delay = false;
    bool saw_sos_read = false;

    for (size_t index = 0; index < fixture.hardware.event_count; ++index) {
        const auto event = fixture.hardware.events.at(index);

        if (event.kind == TestPsp::EventKind::write && event.reg == 0x163 && event.value == 0x20000) {
            saw_sos_command = true;
        } else if (saw_sos_command && event.kind == TestPsp::EventKind::delay && event.value == 20000) {
            saw_delay = true;
        } else if (saw_sos_command && event.kind == TestPsp::EventKind::read && event.reg == 0x191) {
            saw_sos_read = true;
            expect(saw_delay, "no post-SOS life poll before mandatory20ms settling");
        }
    }

    expect(saw_sos_command && saw_delay && saw_sos_read, "trace contains complete real protocol ordering");

    bool zero_tail = true;

    for (size_t offset = 16; offset < psp_staging_bytes; ++offset) {
        zero_tail = zero_tail && fixture.hardware.staging.at(offset) == 0;
    }

    expect(zero_tail, "unused staging tail is initialized");

    for (size_t offset = psp_staging_bytes; offset < fixture.hardware.staging.size(); ++offset) {
        expect(fixture.hardware.staging.at(offset) == 0xa5, "staging writes stay within1MiB ownership bound");
    }

    expect(fixture.manager.shutdown() == rx7900::memory::Status::success,
           "completed boot has no retained manager leases");
}

void check_discovery_factory() {
    auto bytes = rx7900_test::discovery_binary(false, discovery_mp0_hardware_id, 13, 0x100);
    PspRegisters registers;
    expect(!registers.valid(), "PSP descriptor has no publicly asserted valid base");
    expect(validate_psp_discovery({bytes.data(), bytes.size()}, 0x648, registers) ==
           DiscoveryStatus::validated && registers.valid() &&
           registers.segment0_dword_base() == 0x100 && registers.aperture_bytes() == 0x648,
           "MP0 IP13.0.0 descriptor accepts exact final mailbox dword extent");

    for (size_t size = 0; size < 160; ++size) {
        expect(validate_psp_discovery({bytes.data(), size}, 0x4000, registers) ==
               DiscoveryStatus::invalid_binary && !registers.valid(),
               "truncated MP0 discovery clears valid PSP token");
    }

    for (const uint32_t aperture : {uint32_t{0}, uint32_t{3}, uint32_t{0x644}}) {
        expect(validate_psp_discovery({bytes.data(), bytes.size()}, aperture, registers) ==
               DiscoveryStatus::invalid_argument && !registers.valid(),
               "invalid or insufficient assigned BAR extent cannot authorize PSP");
    }

    auto other = rx7900_test::discovery_binary();
    expect(validate_psp_discovery({other.data(), other.size()}, 0x40000, registers) ==
           DiscoveryStatus::invalid_binary && !registers.valid(),
           "GC register base cannot substitute for MP0 discovery");
    bytes.at(146) = 1;
    rx7900_test::discovery_checksums(bytes);
    expect(validate_psp_discovery({bytes.data(), bytes.size()}, 0x4000, registers) ==
           DiscoveryStatus::invalid_binary, "different MP0 instance rejected");
    bytes = rx7900_test::discovery_binary(true, discovery_mp0_hardware_id, 13, 0x100);
    expect(validate_psp_discovery({bytes.data(), bytes.size()}, 0x4000, registers) ==
           DiscoveryStatus::validated, "PSP accepts checked local64-bit discovery records");
    bytes.at(156) = 1;
    rx7900_test::discovery_checksums(bytes);
    expect(validate_psp_discovery({bytes.data(), bytes.size()}, 0x4000, registers) ==
           DiscoveryStatus::unsupported && !registers.valid(), "PSP refuses remote base truncation");
    bytes = rx7900_test::discovery_binary(false, discovery_mp0_hardware_id, 13, 0x100);
    bytes.at(159) ^= 1;
    expect(validate_psp_discovery({bytes.data(), bytes.size()}, 0x4000, registers) ==
           DiscoveryStatus::invalid_binary && !registers.valid(),
           "PSP token requires a correct discovery checksum");
}

void check_initialization() {
    TestPsp hardware;
    PspBoot boot;
    expect(boot.initialize({}, hardware.io(), {3, 3}) ==
           PspBootStatus::unsupported_ip && hardware.event_count == 0,
           "unknown MP0 generation cannot access mailbox registers");
    auto binary = rx7900_test::discovery_binary(false, discovery_mp0_hardware_id, 13, 0x100);
    binary.at(150) = 6;
    rx7900_test::discovery_checksums(binary);
    PspRegisters invalid;
    expect(validate_psp_discovery({binary.data(), binary.size()}, 0x4000, invalid) ==
           DiscoveryStatus::unsupported && !invalid.valid() &&
           boot.initialize(invalid, hardware.io(), {3, 3}) == PspBootStatus::unsupported_ip,
           "PSP13.0.6-specific semantics are not guessed for target13.0.0");
    binary = rx7900_test::discovery_binary(false, discovery_mp0_hardware_id, 13, UINT32_MAX);
    expect(validate_psp_discovery({binary.data(), binary.size()}, 0x4000, invalid) ==
           DiscoveryStatus::unsupported && !invalid.valid() && hardware.event_count == 0,
           "invalid discovered base cannot authorize MMIO");
    expect(boot.initialize(psp_registers(), {}, {3, 3}) ==
           PspBootStatus::unavailable, "missing native backend has no success fallback");
    hardware.registers.at(0x163) = 0;
    expect(boot.initialize(psp_registers(), hardware.io(), {3, 3}) ==
           PspBootStatus::timeout && hardware.one_us_delays == 3,
           "bootloader readiness wait is bounded and does not write commands");
    hardware.registers.at(0x163) = 0x80000007;
    expect(boot.initialize(psp_registers(), hardware.io(), {3, 3}) ==
           PspBootStatus::unexpected_response && boot.report().raw_mailbox == 0x80000007,
           "unknown nonzero completion/error word never becomes READY");
    hardware.registers.at(0x191) = 5;
    hardware.registers.at(0x17a) = 0x1234;
    expect(boot.initialize(psp_registers(), hardware.io(), {3, 3}) ==
           PspBootStatus::already_running && boot.report().completed_steps == 0 &&
           boot.report().sos_version == 0x1234 && hardware.command_writes == 0,
           "preexisting SOS is observed without claiming this image was uploaded");
    hardware.registers.at(0x191) = UINT32_MAX;
    expect(boot.initialize(psp_registers(), hardware.io(), {3, 3}) ==
           PspBootStatus::mmio_failure, "unmapped/all-ones read cannot fake a running secure OS");
}

void check_validation() {
    Fixture fixture;
    fixture.initialize();
    expect(fixture.prepare(0x50000000) == PspBootStatus::invalid_staging &&
           fixture.hardware.command_writes == 0, "DMA/BAR-like address cannot replace owned MC mapping");
    auto original = fixture.firmware;

    for (size_t size = 0; size < fixture.firmware.size(); ++size) {
        expect(fixture.boot.prepare({fixture.firmware.data(), size}, fixture.manager,
                   fixture.staging, {0x40000000}) == PspBootStatus::invalid_firmware,
               "truncated container rejected before staging lease or command");
    }

    fixture.firmware = firmware_fixture(std::array<uint32_t, 3>{2, 1, 5});
    expect(fixture.prepare() == PspBootStatus::invalid_firmware,
           "SPL step cannot silently substitute missing KDB payload");
    fixture.firmware = firmware_fixture(std::array<uint32_t, 3>{2, 1, 11});
    expect(fixture.prepare() == PspBootStatus::invalid_firmware,
           "newer IPKEYMGR requirement cannot be silently skipped by target13.0.0");
    fixture.firmware = original;
    expect(fixture.prepare() == PspBootStatus::prepared, "valid image remains usable after validation failures");
    expect(fixture.boot.initialize(psp_registers(), fixture.hardware.io(), {3, 3}) ==
           PspBootStatus::busy, "active staging owner cannot be reinitialized away");
    fixture.cleanup_idle();

    Fixture small;
    small.initialize(4096);
    expect(small.prepare() == PspBootStatus::invalid_staging,
           "short CPU/GPU staging allocation is rejected");
    small.cleanup_idle();
    Fixture unaligned;
    unaligned.initialize(psp_staging_bytes, 0x40001000);
    expect(unaligned.prepare(0x40001000) == PspBootStatus::invalid_staging,
           "MC staging base must have1MiB alignment before address shift");
    unaligned.cleanup_idle();
}

void check_quarantine() {
    Fixture timeout;
    timeout.initialize();
    expect(timeout.prepare() == PspBootStatus::prepared, "timeout staging prepared");
    timeout.hardware.complete_components = false;
    expect(timeout.boot.submit_next() == PspBootStatus::submitted &&
           timeout.boot.wait() == PspBootStatus::timeout && timeout.hardware.one_us_delays == 3,
           "submitted component does not complete without exact hardware response");
    expect(timeout.boot.report().staging_retained && timeout.manager.shutdown() ==
           rx7900::memory::Status::busy && timeout.boot.release_idle_staging() ==
           PspBootStatus::quarantined, "timeout preserves buffer despite an unload request");
    expect(timeout.boot.release_after_reset() == PspBootStatus::quarantined,
           "planned/unconfirmed reset cannot free possibly active PSP memory");
    timeout.cleanup_reset();

    Fixture rejected;
    rejected.initialize();
    expect(rejected.prepare() == PspBootStatus::prepared, "rejection staging prepared");
    rejected.hardware.component_reply = 0x80000001;
    expect(rejected.boot.submit_next() == PspBootStatus::submitted && rejected.boot.wait() ==
           PspBootStatus::unexpected_response && rejected.boot.report().raw_mailbox == 0x80000001 &&
           rejected.boot.report().completed_steps == 0 && rejected.boot.report().staging_retained,
           "unknown error response is retained and never authenticated/completed");
    rejected.cleanup_reset();

    Fixture uncertain;
    uncertain.initialize();
    expect(uncertain.prepare() == PspBootStatus::prepared, "ambiguous-write staging prepared");
    uncertain.hardware.command_write_uncertain = true;
    expect(uncertain.boot.submit_next() == PspBootStatus::mmio_failure &&
           uncertain.boot.report().command_attempted && uncertain.boot.report().staging_retained &&
           uncertain.boot.wait() == PspBootStatus::quarantined,
           "ambiguous command store cannot accept a possibly stale READY as completion");
    uncertain.cleanup_reset();
}

void check_sos_timeout_and_publication() {
    Fixture fixture;
    fixture.firmware = firmware_fixture(std::array<uint32_t, 2>{2, 1});
    fixture.initialize();
    expect(fixture.prepare() == PspBootStatus::prepared &&
           fixture.boot.submit_next() == PspBootStatus::submitted &&
           fixture.boot.wait() == PspBootStatus::component_completed,
           "mandatory SYS executes before SOS");
    fixture.hardware.start_sos = false;
    expect(fixture.boot.submit_next() == PspBootStatus::submitted &&
           fixture.boot.wait() == PspBootStatus::timeout && fixture.hardware.sos_delays == 1 &&
           fixture.boot.report().sos_life == 0 && fixture.boot.report().staging_retained,
           "SOS command without observed life cannot claim running/authenticated state");
    fixture.cleanup_reset();

    Fixture publication;
    publication.initialize();
    expect(publication.prepare() == PspBootStatus::prepared, "visibility staging prepared");
    publication.hardware.publish_ok = false;
    expect(publication.boot.submit_next() == PspBootStatus::invalid_staging &&
           publication.hardware.command_writes == 0 && publication.boot.report().staging_retained,
           "failed PSP/HDP visibility cannot ring the mailbox");
    publication.cleanup_reset();

    Fixture readback;
    readback.initialize();
    expect(readback.prepare() == PspBootStatus::prepared, "address readback staging prepared");
    readback.hardware.corrupt_address = true;
    expect(readback.boot.submit_next() == PspBootStatus::mmio_failure &&
           readback.hardware.command_writes == 0 && readback.boot.report().staging_retained,
           "MC mailbox address must read back before firmware command is sent");
    readback.cleanup_reset();
}

void check_all_components_and_boundaries() {
    Fixture full;
    full.firmware = firmware_fixture(std::array<uint32_t, 8>{2, 1, 3, 5, 7, 8, 9, 10});
    full.initialize();
    expect(full.prepare() == PspBootStatus::prepared, "complete supported PSP boot plan prepared");
    const std::array<uint32_t, 8> commands{
        0x80000, 0x10000000, 0x10000, 0xb0000, 0xd0000, 0xc0000, 0xe0000, 0x20000};

    for (size_t index = 0; index < commands.size(); ++index) {
        expect(full.boot.submit_next() == PspBootStatus::submitted &&
               full.hardware.registers.at(0x163) == commands.at(index),
               "every optional component follows Linux boot order and opcode");
        expect(full.boot.wait() == (index + 1 == commands.size()
            ? PspBootStatus::secure_os_running : PspBootStatus::component_completed),
            "all supported components require observed completion");
    }

    full.cleanup_idle();
    Fixture high;
    high.initialize(psp_staging_bytes, 0x12340000000ULL);
    expect(high.prepare(0x12340000000ULL) == PspBootStatus::prepared &&
           high.boot.submit_next() == PspBootStatus::submitted &&
           high.hardware.registers.at(0x164) == 0x123400 &&
           high.boot.wait() == PspBootStatus::component_completed,
           "mailbox address preserves high MC bits before narrowing after20-bit shift");
    high.cleanup_idle();

    Fixture exact;
    exact.firmware = firmware_fixture(std::array<uint32_t, 2>{2, 1}, psp_staging_bytes);
    exact.initialize();
    expect(exact.prepare() == PspBootStatus::prepared &&
           exact.boot.submit_next() == PspBootStatus::submitted &&
           exact.boot.wait() == PspBootStatus::component_completed &&
           exact.hardware.staging.at(psp_staging_bytes) == 0xa5,
           "exact1MiB firmware component fits without overwriting its ownership boundary");
    exact.cleanup_idle();

    Fixture oversized;
    oversized.firmware = firmware_fixture(std::array<uint32_t, 2>{2, 1}, psp_staging_bytes + 4);
    oversized.initialize();
    expect(oversized.prepare() == PspBootStatus::invalid_firmware &&
           !oversized.boot.report().staging_retained && oversized.hardware.command_writes == 0,
           "oversized component rejected before staging ownership or hardware mutation");
    oversized.cleanup_idle();
}

void check_completed_resource_cleanup_failure() {
    Fixture fixture;
    fixture.firmware = firmware_fixture(std::array<uint32_t, 2>{2, 1});
    fixture.initialize();
    expect(fixture.prepare() == PspBootStatus::prepared &&
           fixture.boot.submit_next() == PspBootStatus::submitted &&
           fixture.boot.wait() == PspBootStatus::component_completed,
           "cleanup failure fixture executes mandatory SYS");
    expect(fixture.manager.release(fixture.staging) == rx7900::memory::Status::deferred,
           "retire staging before final SOS completion");
    fixture.hardware.destroy_ok = false;
    expect(fixture.boot.submit_next() == PspBootStatus::submitted &&
           fixture.boot.wait() == PspBootStatus::quarantined &&
           fixture.boot.report().sos_life != 0 && !fixture.boot.report().staging_retained &&
           fixture.manager.shutdown() == rx7900::memory::Status::quarantined &&
           fixture.hardware.used, "failed native cleanup remains manager-owned after PSP lease ends");
    fixture.hardware.reset_ok = true;
    expect(fixture.boot.release_after_reset() == PspBootStatus::staging_released,
           "confirmed reset can clear protocol fault after manager took quarantine ownership");
    fixture.hardware.destroy_ok = true;
    expect(fixture.manager.shutdown() == rx7900::memory::Status::success &&
           fixture.hardware.destroys == 1, "native manager cleanup destroys quarantined backing once");
}

} // namespace

int main() {
    check_discovery_factory();
    check_boot_order();
    check_initialization();
    check_validation();
    check_quarantine();
    check_sos_timeout_and_publication();
    check_all_components_and_boundaries();
    check_completed_resource_cleanup_failure();
    std::printf("%u PSP boot checks; %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

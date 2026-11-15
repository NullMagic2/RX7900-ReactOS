/*
Module: rx7900.tests.display
Purpose: Verify DCN timing boundaries, scanout spans and transactional failures.
Created: 2026-10-21
Architecture: Exercises the portable display core without MMIO or a physical GPU.
*/
#include <array>
#include <cstdio>

#include "modes.hpp"
#include "dcn32_otg.hpp"

namespace {

using namespace rx7900::display;
unsigned checks = 0;
unsigned failures = 0;
// A deliberately invented device scanout address, unrelated to any PCI BAR.
constexpr uint64_t test_scanout_base = UINT64_C(0x1200000000);

void expect(bool condition, const char* contract) {
    ++checks;

    if (!condition) {
        ++failures;
        std::fprintf(stderr, "FAIL: %s\n", contract);
    }
}

Timing full_hd() noexcept {
    return Timing{148500, 1920, 2200, 88, 44, 1080, 1125, 4, 5,
                  positive_hsync | positive_vsync};
}

Framebuffer full_hd_buffer() noexcept {
    return Framebuffer{{test_scanout_base}, 8294400, 0, 7680};
}

void check_timing() {
    Timing timing = full_hd();
    OptcTiming fields{};
    expect(prepare_optc_timing(timing, fields) == Status::success,
           "1080p60 borderless progressive timing validates");
    expect(fields.h_total_minus_one == 2199 && fields.h_blank_start == 2112 &&
           fields.h_blank_end == 192 && fields.h_sync_end == 44,
           "horizontal sync-relative fields match 1080p60");
    expect(fields.v_total_minus_one == 1124 && fields.v_blank_start == 1121 &&
           fields.v_blank_end == 41 && fields.v_sync_end == 5,
           "vertical sync-relative fields match 1080p60");
    expect(fields.h_sync_negative == 0 && fields.v_sync_negative == 0,
           "positive polarities have clear negative-polarity fields");
    uint32_t frequency = 0;
    expect(refresh_millihz(timing, frequency) == Status::success && frequency == 60000,
           "refresh uses pixel clock and complete blanking totals");

    timing.flags = 0;
    expect(prepare_optc_timing(timing, fields) == Status::success &&
           fields.h_sync_negative == 1 && fields.v_sync_negative == 1,
           "negative polarities are represented explicitly");
    timing.flags = interlaced;
    expect(prepare_optc_timing(timing, fields) == Status::unsupported_timing,
           "upstream unsupported interlace is rejected");
    timing.flags = 8;
    expect(prepare_optc_timing(timing, fields) == Status::unsupported_timing,
           "unknown timing features are rejected");

    const std::array<Timing, 14> invalid{{
        {0, 1920, 2200, 88, 44, 1080, 1125, 4, 5, 0},
        {148500, 0, 2200, 88, 44, 1080, 1125, 4, 5, 0},
        {148500, 1920, 2200, 88, 44, 0, 1125, 4, 5, 0},
        {148500, 1920, 1919, 88, 44, 1080, 1125, 4, 5, 0},
        {148500, 1920, 2200, 88, 44, 1080, 1079, 4, 5, 0},
        {148500, 1920, 32769, 88, 44, 1080, 1125, 4, 5, 0},
        {148500, 1920, 2200, 88, 44, 1080, 32769, 4, 5, 0},
        {148500, 1920, 1951, 1, 4, 1080, 1125, 4, 5, 0},
        {148500, 1920, 2200, 88, 44, 1080, 1082, 1, 1, 0},
        {148500, 1920, 2200, 0, 44, 1080, 1125, 4, 5, 0},
        {148500, 1920, 2200, 88, 44, 1080, 1125, 0, 5, 0},
        {148500, 1920, 2200, 88, 3, 1080, 1125, 4, 5, 0},
        {148500, 1920, 2200, 279, 4, 1080, 1125, 4, 5, 0},
        {148500, 1920, 2200, UINT32_MAX, UINT32_MAX, 1080, 1125, 4, 5, 0}
    }};

    for (const Timing& candidate : invalid) {
        fields.h_total_minus_one = 123;
        expect(prepare_optc_timing(candidate, fields) == Status::invalid_timing &&
               fields.h_total_minus_one == 123,
               "bad timing geometry leaves prepared output unchanged");
    }

    timing = Timing{1000000, 32700, 32768, 4, 4, 32700, 32768, 4, 1, 0};
    expect(prepare_optc_timing(timing, fields) == Status::success &&
           fields.h_total_minus_one == 32767 && fields.v_total_minus_one == 32767,
           "15-bit total-minus-one boundary is inclusive");
    timing = Timing{UINT32_MAX, 1, 33, 1, 4, 1, 4, 1, 1, 0};
    frequency = 77;
    expect(refresh_millihz(timing, frequency) == Status::arithmetic_overflow && frequency == 77,
           "unrepresentable millihertz does not narrow into a plausible refresh");
    timing = Timing{1, 32700, 32768, 4, 4, 32700, 32768, 4, 1, 0};
    expect(refresh_millihz(timing, frequency) == Status::arithmetic_overflow,
           "sub-millihertz mode cannot advertise zero refresh");
}

void check_framebuffer() {
    const Timing timing = full_hd();
    Framebuffer framebuffer = full_hd_buffer();
    ScanoutLayout layout{};
    expect(validate_framebuffer(timing, framebuffer, layout) == Status::success &&
           layout.surface_address.value == test_scanout_base &&
           layout.surface_bytes == 8294400 && layout.stride_bytes == 7680,
           "scanout preserves full 64-bit GPU address");
    --framebuffer.allocation_bytes;
    layout.surface_bytes = 55;
    expect(validate_framebuffer(timing, framebuffer, layout) == Status::invalid_framebuffer &&
           layout.surface_bytes == 55, "one byte short rejects without modifying output");
    framebuffer = full_hd_buffer();
    framebuffer.surface_offset = 4096;
    framebuffer.allocation_bytes += framebuffer.surface_offset;
    expect(validate_framebuffer(timing, framebuffer, layout) == Status::success &&
           layout.surface_address.value == test_scanout_base + 4096,
           "checked offset is relative to device allocation");
    framebuffer.surface_offset = UINT64_MAX - 3;
    expect(validate_framebuffer(timing, framebuffer, layout) == Status::invalid_framebuffer,
           "offset beyond allocation does not underflow remaining extent");
    framebuffer = full_hd_buffer();
    framebuffer.stride_bytes = 7676;
    expect(validate_framebuffer(timing, framebuffer, layout) == Status::invalid_framebuffer,
           "stride shorter than visible row is rejected");
    framebuffer.stride_bytes = 7681;
    expect(validate_framebuffer(timing, framebuffer, layout) == Status::invalid_framebuffer,
           "unaligned XRGB8888 stride is rejected");
    framebuffer = full_hd_buffer();
    framebuffer.allocation_bytes = UINT64_MAX;
    framebuffer.stride_bytes = 0x80000000u;
    expect(validate_framebuffer(timing, framebuffer, layout) == Status::invalid_framebuffer,
           "stride outside signed GDI range is rejected");
    framebuffer = full_hd_buffer();
    framebuffer.allocation_base.value = UINT64_MAX - 3;
    framebuffer.surface_offset = 4;
    framebuffer.allocation_bytes += 4;
    expect(validate_framebuffer(timing, framebuffer, layout) == Status::arithmetic_overflow,
           "base plus offset cannot wrap GPU address");
    framebuffer.surface_offset = 0;
    expect(validate_framebuffer(timing, framebuffer, layout) == Status::arithmetic_overflow,
           "last scanout byte cannot wrap GPU address");
    framebuffer = full_hd_buffer();
    framebuffer.allocation_base.value = UINT64_MAX - framebuffer.allocation_bytes + 1;
    expect(validate_framebuffer(timing, framebuffer, layout) == Status::success,
           "inclusive last-byte boundary does not require overflowing one-past address");
    framebuffer.allocation_bytes += 4;
    expect(validate_framebuffer(timing, framebuffer, layout) == Status::arithmetic_overflow,
           "reject overflowing allocation metadata even when its surface would fit");
    framebuffer.allocation_base.value = 1;
    expect(validate_framebuffer(timing, framebuffer, layout) == Status::invalid_framebuffer,
           "misaligned GPU address is rejected");
    framebuffer = full_hd_buffer();
    framebuffer.surface_offset = 1;
    expect(validate_framebuffer(timing, framebuffer, layout) == Status::invalid_framebuffer,
           "misaligned allocation offset is rejected");
}

void check_catalog_and_request() {
    ModeCatalog catalog;
    Timing timing = full_hd();
    uint32_t index = 777;
    expect(catalog.count() == 0 && catalog.get(0, timing) == Status::mode_not_found,
           "empty catalog invents no hardware modes");
    expect(catalog.append(timing, index) == Status::success && index == 0,
           "first verified timing gets index zero");
    index = 777;
    expect(catalog.append(timing, index) == Status::duplicate_mode && index == 777 &&
           catalog.count() == 1, "duplicate append is atomic");

    for (uint32_t mode = 1; mode < mode_capacity; ++mode) {
        timing.pixel_clock_khz = 148500 + mode;
        expect(catalog.append(timing, index) == Status::success && index == mode,
               "distinct clock timings have stable append-only indices");
    }

    timing.pixel_clock_khz += 1;
    index = 777;
    expect(catalog.append(timing, index) == Status::capacity_exceeded && index == 777 &&
           catalog.count() == mode_capacity, "full catalog rejects without partial append");
    expect(catalog.get(UINT32_MAX, timing) == Status::mode_not_found,
           "untrusted index cannot access catalog beyond capacity");
    expect(catalog.get(0, timing) == Status::success && timing.pixel_clock_khz == 148500,
           "later inserts preserve index zero");

    const std::array<uint8_t, 4> bytes{{7, 0, 0, 0xc0}};
    ModeRequest request{99, false};
    expect(decode_video_mode_request(rx7900::ByteView(bytes.data(), bytes.size()), request) ==
           Status::success && request.mode_index == 7 && request.preserve_contents,
           "VIDEO_MODE flags are separated from little-endian mode index");
    request.mode_index = 99;
    expect(decode_video_mode_request(rx7900::ByteView(bytes.data(), 3), request) ==
           Status::invalid_request && request.mode_index == 99,
           "short native mode request leaves output unchanged");
    expect(decode_video_mode_request(rx7900::ByteView(nullptr, 4), request) ==
           Status::invalid_request, "null request cannot pretend to have four readable bytes");
    const std::array<uint8_t, 4> clear_bytes{{0, 0, 0, 0x40}};
    expect(decode_video_mode_request(rx7900::ByteView(clear_bytes.data(), 4), request) ==
           Status::success && request.mode_index == 0 && !request.preserve_contents,
           "absence of NO_ZERO_MEMORY explicitly requires clearing on apply");
}

// A bounded script represents confirmed, unchanged or indeterminate hardware
// outcomes. This tests coordinator semantics; it does not emulate Navi31.
struct BackendScript final {
    std::array<ApplyResult, 4> results{{ApplyResult::committed, ApplyResult::committed,
                                       ApplyResult::committed, ApplyResult::committed}};
    Status prepare_result = Status::success;
    uint32_t apply_calls = 0;
    uint32_t prepare_calls = 0;
    ModePlan last_plan{};
    ModeController* controller = nullptr;
    bool check_reentry = false;
    bool fail_rollback_prepare = false;
};

Status prepare_script(void* context, const ModePlan&) noexcept {
    auto& script = *static_cast<BackendScript*>(context);
    ++script.prepare_calls;

    if (script.check_reentry && script.controller != nullptr) {
        expect(script.controller->set_mode({0, false}, full_hd_buffer()) == Status::busy,
               "backend reentry cannot interleave another mode transaction");
        ModePlan current{};
        expect(script.controller->current_mode(current) == Status::busy,
               "in-progress transition does not expose stale current mode");
        expect(script.controller->invalidate_current() == Status::busy,
               "in-progress transition cannot be invalidated reentrantly");
    }

    if (script.fail_rollback_prepare && script.prepare_calls == 3) {
        return Status::hardware_not_ready;
    }

    return script.prepare_result;
}

ApplyResult apply_script(void* context, const ModePlan& plan) noexcept {
    auto& script = *static_cast<BackendScript*>(context);
    script.last_plan = plan;

    if (script.apply_calls >= script.results.size()) {
        return ApplyResult::indeterminate;
    }

    const ApplyResult result = script.results.at(script.apply_calls);
    ++script.apply_calls;
    return result;
}

void populate(ModeController& controller) {
    uint32_t index = 0;
    Timing timing = full_hd();
    expect(controller.append_mode(timing, index) == Status::success && index == 0,
           "populate first transaction timing");
    timing.pixel_clock_khz = 148352;
    expect(controller.append_mode(timing, index) == Status::success && index == 1,
           "populate alternative refresh timing");
}

void check_transactions() {
    ModeController unavailable;
    populate(unavailable);
    ModePlan current{};
    expect(unavailable.current_mode(current) == Status::state_unknown,
           "no current mode before hardware confirmation");
    expect(unavailable.set_mode({0, false}, full_hd_buffer()) == Status::hardware_not_ready &&
           unavailable.current_mode(current) == Status::state_unknown,
           "missing backend never reports an in-memory resolution change as success");
    expect(unavailable.set_mode({mode_capacity, false}, full_hd_buffer()) == Status::mode_not_found,
           "untrusted requested index rejected before a hardware call");

    BackendScript script;
    ModeController controller({&script, prepare_script, apply_script});
    script.controller = &controller;
    script.check_reentry = true;
    populate(controller);
    script.prepare_result = Status::hardware_not_ready;
    expect(controller.set_mode({0, false}, full_hd_buffer()) == Status::hardware_not_ready &&
           script.apply_calls == 0, "failed pure validation makes no hardware attempt");
    script.prepare_result = Status::success;
    expect(controller.set_mode({0, false}, full_hd_buffer()) == Status::success &&
           controller.current_mode(current) == Status::success && current.mode_index == 0 &&
           !script.last_plan.preserve_contents, "confirmed initial mode carries clear requirement");
    script.results.at(1) = ApplyResult::unchanged;
    expect(controller.set_mode({1, true}, full_hd_buffer()) == Status::hardware_failure &&
           controller.current_mode(current) == Status::success && current.mode_index == 0,
           "proven unchanged failure preserves confirmed previous mode");
    script.results.at(2) = ApplyResult::indeterminate;
    script.results.at(3) = ApplyResult::committed;
    expect(controller.set_mode({1, false}, full_hd_buffer()) == Status::hardware_failure &&
           controller.current_mode(current) == Status::success && current.mode_index == 0 &&
           script.apply_calls == 4 && script.last_plan.mode_index == 0 &&
           script.last_plan.preserve_contents,
           "confirmed rollback restores old mode without clearing its contents");
    expect(controller.invalidate_current() == Status::success &&
           controller.current_mode(current) == Status::state_unknown,
           "reset or external ownership loss invalidates cached current mode");

    BackendScript broken;
    broken.results.at(1) = ApplyResult::indeterminate;
    broken.results.at(2) = ApplyResult::unchanged;
    ModeController failed({&broken, prepare_script, apply_script});
    populate(failed);
    expect(failed.set_mode({0, false}, full_hd_buffer()) == Status::success,
           "seed previous confirmed mode for rollback failure");
    expect(failed.set_mode({1, true}, full_hd_buffer()) == Status::state_unknown &&
           failed.current_mode(current) == Status::state_unknown && broken.apply_calls == 3,
           "failed rollback invalidates current instead of reporting a stale resolution");

    BackendScript no_previous;
    no_previous.results.at(0) = ApplyResult::indeterminate;
    ModeController first_failure({&no_previous, prepare_script, apply_script});
    populate(first_failure);
    expect(first_failure.set_mode({0, false}, full_hd_buffer()) == Status::state_unknown &&
           no_previous.apply_calls == 1, "first indeterminate attempt has no invented rollback mode");

    BackendScript rollback_validation;
    rollback_validation.results.at(1) = ApplyResult::indeterminate;
    rollback_validation.fail_rollback_prepare = true;
    ModeController rejected({&rollback_validation, prepare_script, apply_script});
    populate(rejected);
    expect(rejected.set_mode({0, false}, full_hd_buffer()) == Status::success,
           "seed previous mode for rejected rollback validation");
    expect(rejected.set_mode({1, true}, full_hd_buffer()) == Status::state_unknown &&
           rollback_validation.apply_calls == 2,
           "rollback capability failure makes no unvalidated hardware attempt");
}

// A bounded register callback model checks transaction/error semantics. This is
// not Navi31 emulation and cannot prove clocks, link training or physical output.
struct RegisterFile final {
    std::array<uint32_t, 0x2000> words{};
    uint32_t reads = 0;
    uint32_t writes = 0;
    uint32_t fail_read = 0;
    uint32_t fail_write = 0;
    uint32_t corrupt_write = 0;
    bool activate_after_write = false;
};

bool register_read(void* context, uint32_t offset, uint32_t& value) noexcept {
    auto& registers = *static_cast<RegisterFile*>(context);
    ++registers.reads;

    if (offset % 4 != 0 || offset / 4 >= registers.words.size() ||
        registers.reads == registers.fail_read) {
        return false;
    }

    value = registers.words.at(offset / 4);
    return true;
}

bool register_write(void* context, uint32_t offset, uint32_t value) noexcept {
    auto& registers = *static_cast<RegisterFile*>(context);
    ++registers.writes;

    if (offset % 4 != 0 || offset / 4 >= registers.words.size()) {
        return false;
    }

    // A failed callback may still reach MMIO; the implementation must quarantine
    // even if this is the very first failed write, rather than claiming unchanged.
    registers.words.at(offset / 4) = value ^
        (registers.writes == registers.corrupt_write ? 1u : 0u);

    if (registers.activate_after_write) {
        registers.words.at(0x1b41) = 1;
    }

    return registers.writes != registers.fail_write;
}

RegisterIo register_io(RegisterFile& registers) noexcept {
    return RegisterIo{&registers, register_read, register_write};
}

Dcn32RegisterSpace register_space() noexcept {
    // A deliberately invented IP-discovery segment base of zero for this model.
    return Dcn32RegisterSpace{0, 0x8000, 3, 2, 0, true};
}

DisabledOtgPrerequisites prerequisites() noexcept {
    return DisabledOtgPrerequisites{true, true, true, true, true};
}

void initialize_registers(RegisterFile& registers, uint32_t instance = 0) {
    constexpr std::array<uint32_t, 10> indices{{
        0x1b2a, 0x1b2c, 0x1b2b, 0x1b2d, 0x1b2f,
        0x1b30, 0x1b31, 0x1b37, 0x1b36, 0x1b38
    }};

    for (uint32_t index : indices) {
        registers.words.at(index + 0x80 * instance) = 0x80008000u;
    }

    registers.words.at(0x1b85 + 0x80 * instance) = 0x101;
}

void check_disabled_otg() {
    RegisterFile registers{};
    initialize_registers(registers);
    Dcn32Otg otg(register_io(registers), register_space(), 0);
    expect(otg.program_disabled(full_hd(), prerequisites()) ==
               OtgResult::disabled_fields_verified && registers.writes == 10 &&
               !otg.quarantined(),
           "disabled DCN3.2 timing writes require actual callback readback");
    expect(registers.words.at(0x1b2a) == (0x80008000u | 2199u) &&
           registers.words.at(0x1b2c) == (0x80008000u | (44u << 16)) &&
           registers.words.at(0x1b2b) == (0x80008000u | 2112u | (192u << 16)) &&
           registers.words.at(0x1b30) == (0x80008000u | 1124u) &&
           registers.words.at(0x1b31) == (0x80008000u | 1124u),
           "exact upstream timing fields preserve unrelated register bits");
    expect(registers.words.at(0x1b41) == 0 && registers.words.at(0x1b85) == 0x101 &&
           registers.words.at(0x1b5a) == 0 && registers.words.at(0x1b8a) == 0,
           "programmer never enables clocks/OTG, changes reset or takes hardware locks");
    const uint32_t healthy_reads = registers.reads;

    for (uint32_t failure = 1; failure <= healthy_reads; ++failure) {
        RegisterFile failing{};
        initialize_registers(failing);
        failing.fail_read = failure;
        Dcn32Otg broken(register_io(failing), register_space(), 0);
        const OtgResult result = broken.program_disabled(full_hd(), prerequisites());
        expect(failing.writes == 0 ? result == OtgResult::read_failure && !broken.quarantined() :
                                     result == OtgResult::quarantined && broken.quarantined(),
               "every failed preflight/readback/interlock read has honest write-state result");

        if (broken.quarantined()) {
            const uint32_t reads = failing.reads;
            const uint32_t writes = failing.writes;
            expect(broken.program_disabled(full_hd(), prerequisites()) == OtgResult::quarantined &&
                   failing.reads == reads && failing.writes == writes,
                   "quarantined OTG can never retry further register access");
        }
    }

    for (uint32_t failure = 1; failure <= 10; ++failure) {
        RegisterFile failing{};
        initialize_registers(failing);
        failing.fail_write = failure;
        Dcn32Otg broken(register_io(failing), register_space(), 0);
        expect(broken.program_disabled(full_hd(), prerequisites()) == OtgResult::quarantined &&
               failing.writes == failure && broken.quarantined(),
               "every partial or failed write permanently quarantines the pipeline");
    }

    constexpr std::array<uint32_t, 11> guard_indices{{
        0x1b41, 0x1b41, 0x1b85, 0x1b85, 0x1b2e, 0x1b44,
        0x1b54, 0x1b5a, 0x1b8a, 0x1b33, 0x1b5b
    }};
    constexpr std::array<uint32_t, 11> guard_values{{
        1, 0x10000, 0x111, 0x10101, 1, 1, 0x1000000, 1, 0x100, 1, 0x20
    }};

    for (size_t index = 0; index < guard_indices.size(); ++index) {
        RegisterFile unsafe{};
        initialize_registers(unsafe);
        unsafe.words.at(guard_indices.at(index)) = guard_values.at(index);
        Dcn32Otg blocked(register_io(unsafe), register_space(), 0);
        expect(blocked.program_disabled(full_hd(), prerequisites()) ==
                   OtgResult::unsafe_hardware_state && unsafe.writes == 0 && !blocked.quarantined(),
               "active, resetting, busy, divided, stereo, interlace or locked OTG is never written");
    }

    RegisterFile corrupt{};
    initialize_registers(corrupt);
    corrupt.corrupt_write = 3;
    Dcn32Otg mismatch(register_io(corrupt), register_space(), 0);
    expect(mismatch.program_disabled(full_hd(), prerequisites()) == OtgResult::quarantined &&
           corrupt.writes == 3, "readback mismatch stops at the first unconfirmed register");
    RegisterFile activated{};
    initialize_registers(activated);
    activated.activate_after_write = true;
    Dcn32Otg interrupted(register_io(activated), register_space(), 0);
    expect(interrupted.program_disabled(full_hd(), prerequisites()) == OtgResult::quarantined &&
           activated.writes == 1, "loss of disabled state mid-batch quarantines without further writes");

    RegisterFile unavailable{};
    initialize_registers(unavailable);
    auto limits = register_space();
    limits.segment2_dword_base = UINT32_MAX;
    Dcn32Otg overflow(register_io(unavailable), limits, 0);
    expect(overflow.program_disabled(full_hd(), prerequisites()) == OtgResult::unavailable &&
           unavailable.reads == 0 && unavailable.writes == 0,
           "IP-discovery DWORD base cannot wrap a BAR-relative byte offset");
    limits = register_space();
    limits.aperture_bytes = 16;
    Dcn32Otg too_small(register_io(unavailable), limits, 0);
    expect(too_small.program_disabled(full_hd(), prerequisites()) == OtgResult::unavailable &&
           unavailable.reads == 0, "all timing offsets must fit the owned register mapping before reads");
    limits = register_space();
    limits.discovery_verified = false;
    Dcn32Otg unverified(register_io(unavailable), limits, 0);
    expect(unverified.program_disabled(full_hd(), prerequisites()) == OtgResult::unavailable &&
           unavailable.reads == 0, "marketing PCI identity does not substitute for verified IP discovery");
    Dcn32Otg bad_instance(register_io(unavailable), register_space(), 4);
    expect(bad_instance.program_disabled(full_hd(), prerequisites()) == OtgResult::unavailable &&
           unavailable.reads == 0, "OTG instance cannot exceed the four pinned register banks");
    Dcn32Otg no_ownership(register_io(unavailable), register_space(), 0);
    auto facts = prerequisites();
    facts.resources_and_initialization_owned = false;
    expect(no_ownership.program_disabled(full_hd(), facts) == OtgResult::unavailable &&
           unavailable.reads == 0, "resource mapping alone never authorizes display writes");
    facts = prerequisites();
    facts.exclusive_register_lock_held = false;
    expect(no_ownership.program_disabled(full_hd(), facts) == OtgResult::unavailable &&
           unavailable.reads == 0, "external firmware/register serialization is required");

    RegisterFile instance_three{};
    initialize_registers(instance_three, 3);
    Dcn32Otg fourth(register_io(instance_three), register_space(), 3);
    expect(fourth.program_disabled(full_hd(), prerequisites()) ==
               OtgResult::disabled_fields_verified &&
           instance_three.words.at(0x1caa) == (0x80008000u | 2199u) &&
           instance_three.words.at(0x1b2a) == 0,
           "instance-three writes use exact pinned bank offsets without touching OTG0");
}

} // namespace

int main() {
    check_timing();
    check_framebuffer();
    check_catalog_and_request();
    check_transactions();
    check_disabled_otg();
    std::printf("display: %u checks, %u failures; no hardware operations\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

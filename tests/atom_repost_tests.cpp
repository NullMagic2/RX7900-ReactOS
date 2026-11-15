/*
Module: rx7900.tests.atom_repost
Purpose: Verify AsicInit::repost and the resume need-post path over builder-made VBIOS images.
Created: 2026-10-24
Architecture: The VBIOS (firmwareinfo with boot clocks, an ASIC_Init command table
format 2 and a helper table it calls) is assembled by tests/atom_test_builder.hpp;
the device is the shared simulated Navi31 (asic_test_device.hpp) behind a
recording RegisterIo. The end-to-end board mirrors the Fixture of
asic_init_tests.cpp (that file is owned by other agents, so its fixture is not
yet shared). Synthetic only: ordering and contract evidence, never hardware.
*/
#include <array>
#include <memory>
#include <vector>

#include "asic_test_device.hpp"
#include "atom_test_builder.hpp"
#include "mes_test_model.hpp"
#include "ucode_test_files.hpp"

namespace {

using namespace rx7900::hardware;
using namespace rx7900_test::ucode_files;
namespace atom = rx7900_test::atom;
using atom::Code;
using atom::Cond;
using atom::Image;
using atom::ImageBuilder;
using atom::imm32;
using atom::ps;
using atom::reg;
using atom::ws;
using rx7900_test::SimulatedNavi31;

rx7900_test::Checks checks;

void expect(bool condition, const char* contract) { checks.expect(condition, contract); }

// firmwareinfo bios_scratch_reg_startaddr (BIOS_SCRATCH_0) and BIOS_SCRATCH_7.
constexpr uint32_t scratch_reg = 0x5c9;
constexpr uint32_t scratch7 = scratch_reg + 7;
// atombios.h ATOM_S7_ASIC_INIT_COMPLETE_MASK.
constexpr uint32_t asic_init_complete = 0x200;
// MP0 C2PMSG_35 (psp_v13_0_wait_for_bootloader).
constexpr uint32_t c2pmsg_35 = rx7900_test::mp_base + 0x63;
// 10 retries x adev->usec_timeout polls.
constexpr uint32_t bootloader_polls = 10u * 100000u;
// Boot clocks wider than the 24-bit asic_init fields.
constexpr uint32_t boot_sclk = 0x0f123456u;
constexpr uint32_t boot_mclk = 0x0a654321u;
// Registers the synthetic ASIC_Init programs (unmodelled by the device).
constexpr uint16_t atom_window = 0x7000;
constexpr uint16_t ready_flag = 0x7010;

struct Vbios final {
    enum class Completion { always, never, when_ready, all_ones };
    uint8_t format = 2;
    uint8_t content = 1;
    bool asic_init = true;
    bool firmware_info = true;
    Completion completion = Completion::always;
    bool malformed = false;
    bool read_ps18 = false;
    bool delay_forever = false;
    bool recursive = false;
    // vram_usagebyfirmware v2.1 used_by_driver_in_kb (0: table absent).
    uint16_t driver_scratch_kb = 0;
    // Distinguishes otherwise identical images (another ROM fingerprint).
    uint8_t marker = 0;
};

// ASIC_Init (command 0, format 2): engine and memory parameters to two
// registers, the OR of PS[2..17] to a third, a helper table (command 1) that
// programs through a REG block and waits, then ATOM_S7_ASIC_INIT_COMPLETE in
// BIOS_SCRATCH_7 as a real VBIOS reports a completed POST.
Image vbios(const Vbios& options) {
    ImageBuilder builder;
    builder.command_entries(4);

    if (options.firmware_info) {
        builder.firmware_info({4, boot_sclk, boot_mclk, 0, scratch_reg});
    }

    if (options.driver_scratch_kb != 0) {
        // vram_usagebyfirmware_v2_1 after its header: start_address_in_kb,
        // used_by_firmware_in_kb, used_by_driver_in_kb.
        std::vector<uint8_t> payload(8, 0);
        atom::set_u16(payload, 6, options.driver_scratch_kb);
        builder.data(atom::data_vram_usage, 2, 1, payload);
    }

    auto& helper = builder.command(1, 0, 0).code;
    helper.set_reg_block(atom_window);
    helper.move(reg(0x8), imm32(0x0000a5a5u | (static_cast<uint32_t>(options.marker) << 16)));
    helper.delay_us(10);
    helper.eot();

    if (options.asic_init) {
        auto& table = builder.command(atom_command_asic_init, 1, 0);
        table.format = options.format;
        table.content = options.content;
        Code& code = table.code;
        code.move(reg(atom_window), ps(0));
        code.move(reg(atom_window + 1), ps(1));
        code.move(ws(0), imm32(0));

        for (uint8_t index = 2; index < 18; ++index) {
            code.or_(ws(0), ps(index));
        }

        code.move(reg(atom_window + 2), ws(0));

        if (options.read_ps18) {
            code.move(reg(atom_window + 3), ps(18));
        }

        code.call_table(1);
        code.set_reg_block(0);

        if (options.malformed) {
            code.jump_raw(Cond::always, 0x7fff);
        }

        if (options.delay_forever) {
            const Code::Label top = code.label();
            code.place(top);
            code.delay_ms(255);
            code.jump(Cond::always, top);
        }

        if (options.recursive) {
            code.call_table(atom_command_asic_init);
        }

        if (options.completion == Vbios::Completion::always) {
            code.or_(reg(scratch7), imm32(asic_init_complete));
        } else if (options.completion == Vbios::Completion::when_ready) {
            const Code::Label skip = code.label();
            code.compare(reg(ready_flag), imm32(0));
            code.jump(Cond::equal, skip);
            code.or_(reg(scratch7), imm32(asic_init_complete));
            code.place(skip);
        } else if (options.completion == Vbios::Completion::all_ones) {
            // A floating bus reads all ones, which proves nothing either way.
            code.move(reg(scratch7), imm32(0xffffffffu));
        }

        code.eot();
    }

    return builder.build();
}

// RegisterIo wrapper: counts C2PMSG_35 polls (optionally reporting the first
// not_ready of them as 0), logs the ATOM window and BIOS scratch accesses with
// the poll and PSP boot-command counts at that moment, and can fail one write.
struct Recorder final {
    struct Entry final {
        bool write;
        uint32_t index;
        uint32_t value;
        uint32_t polls;
        unsigned boots;
    };

    RegisterIo inner{nullptr, 0, nullptr, nullptr, nullptr};
    const unsigned* boots = nullptr;
    uint32_t polls = 0;
    uint32_t not_ready = 0;
    uint32_t fail_write = UINT32_MAX;
    uint32_t delays = 0;
    std::vector<Entry> log;

    static bool watched(uint32_t index) {
        return (index >= atom_window && index < atom_window + 0x100u) ||
               (index >= scratch_reg && index < scratch_reg + 16u);
    }

    void note(bool write, uint32_t index, uint32_t value) {
        if (watched(index)) {
            log.push_back({write, index, value, polls, boots != nullptr ? *boots : 0u});
        }
    }

    static bool read(void* context, uint32_t offset, uint32_t& value) noexcept {
        auto& self = *static_cast<Recorder*>(context);

        if (offset / 4u == c2pmsg_35) {
            ++self.polls;

            if (self.polls <= self.not_ready) {
                value = 0;
                return true;
            }
        }

        if (!self.inner.read32(self.inner.context, offset, value)) {
            return false;
        }

        self.note(false, offset / 4u, value);
        return true;
    }

    static bool write(void* context, uint32_t offset, uint32_t value) noexcept {
        auto& self = *static_cast<Recorder*>(context);

        if (offset / 4u == self.fail_write) {
            return false;
        }

        self.note(true, offset / 4u, value);
        return self.inner.write32(self.inner.context, offset, value);
    }

    static void delay(void* context, uint32_t microseconds) noexcept {
        auto& self = *static_cast<Recorder*>(context);
        ++self.delays;
        self.inner.delay_microseconds(self.inner.context, microseconds);
    }

    RegisterIo io() noexcept { return {this, inner.aperture_bytes, read, write, delay}; }

    std::vector<Entry> writes() const {
        std::vector<Entry> output;

        for (const auto& entry : log) {
            if (entry.write) {
                output.push_back(entry);
            }
        }

        return output;
    }
};

// A fresh AsicInit (no run yet) on a simulated card: repost is allowed.
struct Direct final {
    explicit Direct(const Vbios& options = {}) : image(vbios(options)), rom(image.bytes) {
        recorder.inner = device->io();
        recorder.boots = &device->boot_commands;
        parsed = parse_atom_firmware_info({rom.data(), rom.size()}, info);
    }

    std::unique_ptr<SimulatedNavi31> device = std::make_unique<SimulatedNavi31>();
    std::vector<uint8_t> discovery = rx7900_test::navi31_discovery();
    std::unique_ptr<AsicInit> asic = std::make_unique<AsicInit>();
    Image image;
    std::vector<uint8_t> rom;
    AtomFirmwareInfo info{};
    AtomFirmwareStatus parsed = AtomFirmwareStatus::absent;
    Recorder recorder;

    AsicInputs inputs(const AtomFirmwareInfo* atom_info, rx7900::ByteView atom_rom) {
        return {recorder.io(), {discovery.data(), discovery.size()}, {nullptr, 0}, nullptr, {nullptr, 0}, nullptr,
                atom_info, PspTrainingIo{}, {nullptr, 0}, SmuPcieLink{}, 0, Ih6Platform{}, false, 0, DoorbellIo{},
                GttPlatform{}, true, InterruptReady{}, atom_rom};
    }

    AsicInputs inputs() { return inputs(&info, {rom.data(), rom.size()}); }
    AsicStatus repost() { return asic->repost(inputs()); }
    uint32_t reg_value(uint32_t index) { return device->reg(index); }
};

Vbios with(Vbios::Completion completion) {
    Vbios options{};
    options.completion = completion;
    return options;
}

void firmware_and_success() {
    Direct d;
    expect(d.parsed == AtomFirmwareStatus::success && d.info.bootup_gfxclk_10khz == boot_sclk &&
               d.info.bootup_uclk_10khz == boot_mclk && d.info.bios_scratch_reg == scratch_reg,
           "the builder VBIOS firmwareinfo v3.4 parses to the boot clocks and BIOS_SCRATCH_0");
    expect(d.reg_value(scratch7) == 0, "the simulated card starts unposted");
    const AsicStatus status = d.repost();
    const AsicRepostReport report = d.asic->repost_report();
    expect(status == AsicStatus::completed && report.status == AsicStatus::completed &&
               report.step == AsicStep::repost_confirm && report.posted && report.bootloader_ready &&
               report.attempts == 1 && report.completed == 1,
           "repost confirms ASIC_INIT_COMPLETE after ASIC_Init");
    expect(report.scratch_bytes == atom_default_scratch_bytes && report.atom.status == AtomStatus::completed &&
               report.atom.root_table == atom_command_asic_init && report.atom.calls == 1 &&
               report.atom.tolerated_faults == 0 && report.atom.delay_us == 10,
           "ASIC_Init ran from table 0 with the 20 KiB default scratch, its helper call and delay");
    // asic_init_ps_allocation_v2_1: {sclk:24, engineflag:8}, {mclk:24, memflag:8},
    // reserved[16]; amdgpu_device_asic_init passes fb_reset = true (memflag 0).
    expect(d.reg_value(atom_window) == 0x00123456u && d.reg_value(atom_window + 1) == 0x00654321u &&
               d.reg_value(atom_window + 2) == 0,
           "parameters: 24-bit clocks, engineflag 0 (b3NORMAL_ENGINE_INIT), memflag 0, 16 zero dwords");
    expect(d.reg_value(atom_window + 8) == 0x0000a5a5u && (d.reg_value(scratch7) & asic_init_complete) != 0,
           "the helper table and the BIOS_SCRATCH_7 completion bit were programmed");

    bool after_bootloader = !d.recorder.log.empty() && d.recorder.polls == 1;

    for (const auto& entry : d.recorder.log) {
        after_bootloader = after_bootloader && entry.polls == d.recorder.polls;
    }

    expect(after_bootloader, "every ATOM access follows the single ready C2PMSG_35 poll");
    const size_t entries = d.recorder.log.size();
    expect(entries >= 2 && !d.recorder.log.at(entries - 1).write && d.recorder.log.at(entries - 1).index == scratch7 &&
               (d.recorder.log.at(entries - 1).value & asic_init_complete) != 0 &&
               d.recorder.log.at(entries - 2).write && d.recorder.log.at(entries - 2).index == scratch7,
           "the posted check (BIOS_SCRATCH_7 read) comes after ASIC_Init's own write");
    expect(d.device->boot_commands == 0 && d.asic->report().status == AsicStatus::invalid_input,
           "repost sends no PSP boot command and leaves the AsicReport untouched");

    Direct twice;
    twice.repost();
    expect(twice.repost() == AsicStatus::completed && twice.asic->repost_report().attempts == 2 &&
               twice.asic->repost_report().completed == 2,
           "repost can run again on the same resident ROM (it rebinds each call)");

    Vbios ps18{};
    ps18.read_ps18 = true;
    Direct boundary(ps18);
    expect(boundary.repost() == AsicStatus::completed && boundary.reg_value(atom_window + 3) == 0xcdcdcdcdu &&
               boundary.asic->repost_report().atom.tolerated_faults == 1 &&
               boundary.asic->repost_report().atom.first_fault == AtomFault::ps_index,
           "exactly 18 PS dwords: PS[18] reads atom.c's 0xCDCDCDCD (Linux passes sizeof in bytes)");

    Vbios crev3{};
    crev3.content = 3;
    Direct revision3(crev3);
    expect(revision3.repost() == AsicStatus::completed, "ASIC_Init content revision 3 is accepted (crev >= 1)");
}

void scratch_sizes() {
    Vbios small{};
    small.driver_scratch_kb = 8;
    Direct a(small);
    Vbios large{};
    large.driver_scratch_kb = 200;
    Direct b(large);
    expect(a.repost() == AsicStatus::completed && a.asic->repost_report().scratch_bytes == 8192 &&
               b.repost() == AsicStatus::completed &&
               b.asic->repost_report().scratch_bytes == atom_scratch_capacity_bytes,
           "vram_usagebyfirmware v2.1 used_by_driver_in_kb sizes the scratch, clamped to 64 KiB");
}

void memory_flag() {
    // fb_reset = false selects b3DRAM_SELF_REFRESH_EXIT (0x20); repost itself
    // always passes true, as amdgpu_device_asic_init does.
    Direct d;
    auto interpreter = std::make_unique<AtomInterpreter>();
    expect(interpreter->bind({d.rom.data(), d.rom.size()}, atom_default_scratch_bytes) == AtomStatus::completed &&
               interpreter->asic_init(d.device->io(), boot_sclk, boot_mclk, false, atom_default_policy) ==
                   AtomStatus::completed &&
               d.reg_value(atom_window) == 0x00123456u && d.reg_value(atom_window + 1) == 0x20654321u,
           "memflag 0x20 without fb_reset, engine parameter unchanged");
    expect(interpreter->asic_init(d.device->io(), boot_sclk, boot_mclk, true, atom_default_policy) ==
                   AtomStatus::completed && d.reg_value(atom_window + 1) == 0x00654321u,
           "memflag 0 with fb_reset");
}

void bootloader() {
    Direct never;
    never.device->reg(c2pmsg_35) = 0;
    const AsicStatus status = never.repost();
    expect(status == AsicStatus::completed && !never.asic->repost_report().bootloader_ready &&
               never.asic->repost_report().posted && never.recorder.polls == bootloader_polls &&
               never.recorder.delays >= bootloader_polls && never.reg_value(atom_window) == 0x00123456u,
           "a bootloader that never reports ready is recorded; ASIC_Init still runs (Linux ignores the result)");

    Direct late;
    late.recorder.not_ready = 50;
    expect(late.repost() == AsicStatus::completed && late.asic->repost_report().bootloader_ready &&
               late.recorder.polls == 51 && !late.recorder.log.empty() && late.recorder.log.front().polls == 51,
           "a bootloader ready after 50 polls: ASIC_Init starts only after the ready poll");
}

void refusals() {
    Vbios missing{};
    missing.asic_init = false;
    Direct m(missing);
    expect(m.repost() == AsicStatus::unsupported && m.asic->repost_report().step == AsicStep::repost_atom &&
               m.asic->repost_report().atom.status == AtomStatus::absent_table && m.recorder.writes().empty() &&
               !m.asic->repost_report().posted,
           "no ASIC_Init table: unsupported, nothing written");

    Vbios format1{};
    format1.format = 1;
    Vbios content0{};
    content0.content = 0;
    Direct frev1(format1);
    Direct crev0(content0);
    expect(frev1.repost() == AsicStatus::unsupported &&
               frev1.asic->repost_report().atom.status == AtomStatus::unsupported_revision &&
               crev0.repost() == AsicStatus::unsupported &&
               crev0.asic->repost_report().atom.status == AtomStatus::unsupported_revision &&
               frev1.recorder.writes().empty() && crev0.recorder.writes().empty(),
           "ASIC_Init format 1 or content 0: unsupported_revision, nothing written");

    Direct d;
    AtomFirmwareInfo no_scratch = d.info;
    no_scratch.bios_scratch_reg = 0;
    std::vector<uint8_t> corrupt = d.rom;
    corrupt.at(0) = 0;
    const AsicStatus no_info = d.asic->repost(d.inputs(nullptr, {d.rom.data(), d.rom.size()}));
    const AsicStatus no_register = d.asic->repost(d.inputs(&no_scratch, {d.rom.data(), d.rom.size()}));
    const AsicStatus no_rom = d.asic->repost(d.inputs(&d.info, {nullptr, 0}));
    const AsicStatus bad_rom = d.asic->repost(d.inputs(&d.info, {corrupt.data(), corrupt.size()}));
    expect(no_info == AsicStatus::unsupported && no_register == AsicStatus::unsupported &&
               no_rom == AsicStatus::unsupported && bad_rom == AsicStatus::unsupported && d.recorder.log.empty() &&
               d.recorder.polls == 0 && d.asic->repost_report().attempts == 4,
           "no firmwareinfo, no scratch register, no ROM or an invalid ROM: unsupported before any access");

    AsicInputs broken = d.inputs();
    broken.io.read32 = nullptr;
    AsicInputs no_discovery = d.inputs();
    no_discovery.discovery = {nullptr, 0};
    expect(d.asic->repost(broken) == AsicStatus::invalid_input &&
               d.asic->repost(no_discovery) == AsicStatus::invalid_input,
           "invalid RegisterIo or discovery is invalid_input");
}

void failures() {
    Direct n(with(Vbios::Completion::never));
    expect(n.repost() == AsicStatus::failed && n.asic->repost_report().step == AsicStep::repost_confirm &&
               n.asic->repost_report().atom.status == AtomStatus::completed && !n.asic->repost_report().posted &&
               n.reg_value(atom_window) == 0x00123456u,
           "ASIC_Init that never sets ASIC_INIT_COMPLETE: failed at repost_confirm (its writes are not undone)");
    expect(n.repost() == AsicStatus::failed && n.asic->repost_report().attempts == 2 &&
               n.asic->repost_report().completed == 0,
           "the unconfirmed repost stays retryable (not quarantined)");

    Direct w(with(Vbios::Completion::when_ready));
    const AsicStatus first = w.repost();
    w.device->reg(ready_flag) = 1;
    expect(first == AsicStatus::failed && w.repost() == AsicStatus::completed &&
               w.asic->repost_report().attempts == 2 && w.asic->repost_report().completed == 1,
           "a retry of the same ROM completes once the hardware condition holds");

    Direct ones(with(Vbios::Completion::all_ones));
    expect(ones.repost() == AsicStatus::failed && ones.asic->repost_report().step == AsicStep::repost_confirm &&
               !ones.asic->repost_report().posted,
           "an all-ones BIOS_SCRATCH_7 is no posted proof: failed");

    Vbios broken{};
    broken.malformed = true;
    Direct q(broken);
    expect(q.repost() == AsicStatus::quarantined && q.asic->repost_report().step == AsicStep::repost_atom &&
               q.asic->repost_report().atom.status == AtomStatus::malformed_table,
           "a jump outside ASIC_Init is malformed: quarantined");
    const size_t writes_before = q.recorder.writes().size();
    expect(q.repost() == AsicStatus::quarantined &&
               q.asic->repost_report().atom.status == AtomStatus::quarantined &&
               q.recorder.writes().size() == writes_before,
           "the quarantined ROM is refused on the next repost without any ATOM write");
    const Image fixed = vbios(Vbios{});
    expect(q.asic->repost(q.inputs(&q.info, {fixed.bytes.data(), fixed.bytes.size()})) == AsicStatus::completed,
           "a different (corrected) ROM is accepted after a quarantine");

    Vbios recursive{};
    recursive.recursive = true;
    Direct r(recursive);
    expect(r.repost() == AsicStatus::quarantined && r.asic->repost_report().atom.status == AtomStatus::call_depth &&
               r.asic->repost_report().atom.max_depth == atom_frame_capacity,
           "a self-calling ASIC_Init exhausts the 16 frames: call_depth, quarantined");

    Vbios slow{};
    slow.delay_forever = true;
    Direct s(slow);
    expect(s.repost() == AsicStatus::failed && s.asic->repost_report().atom.status == AtomStatus::delay_budget &&
               s.asic->repost_report().atom.delay_us <= atom_default_policy.max_delay_us,
           "an ASIC_Init that waits forever stops at the 20 s delay budget: failed, retryable");

    Direct io;
    io.recorder.fail_write = atom_window + 1u;
    expect(io.repost() == AsicStatus::failed && io.asic->repost_report().atom.status == AtomStatus::io_failure,
           "a failed register write during ASIC_Init is failed (io_failure), not quarantined");
    io.recorder.fail_write = UINT32_MAX;
    expect(io.repost() == AsicStatus::completed, "the same ROM reposts after the I/O failure cleared");
}

// ---------------------------------------------------------------------------
// End to end: run, suspend, power loss, resume through repost.
// ---------------------------------------------------------------------------

std::vector<uint8_t> sos_container() {
    const std::array<uint32_t, 6> identifiers{2, 1, 4, 3, 5, 6};
    constexpr uint32_t part_bytes = 64;
    std::vector<uint8_t> bytes(256 + identifiers.size() * part_bytes);
    const rx7900::sdma::CommandBuffer output(bytes.data(), bytes.size());
    output.write_u32(0, static_cast<uint32_t>(bytes.size()));
    output.write_u32(4, 36);
    bytes.at(8) = 2;
    bytes.at(12) = 13;
    output.write_u32(16, 0x310035);
    output.write_u32(20, static_cast<uint32_t>(identifiers.size() * part_bytes));
    output.write_u32(24, 256);
    output.write_u32(32, static_cast<uint32_t>(identifiers.size()));

    for (size_t index = 0; index < identifiers.size(); ++index) {
        output.write_u32(36 + index * 16, identifiers.at(index));
        output.write_u32(40 + index * 16, 0x310035);
        output.write_u32(44 + index * 16, static_cast<uint32_t>(index * part_bytes));
        output.write_u32(48 + index * 16, part_bytes);

        for (uint32_t word = 0; word < part_bytes / 4; ++word) {
            output.write_u32(256 + index * part_bytes + word * 4,
                             0x50000000u + identifiers.at(index) * 0x100 + word);
        }
    }

    return bytes;
}

struct PciLog final {
    unsigned calls = 0;
    static bool step(void* context) noexcept {
        ++static_cast<PciLog*>(context)->calls;
        return true;
    }
    ResetPlatform platform() noexcept { return {this, step, step, step}; }
};

// The asic_init_tests.cpp Fixture's untrained cold boot with the builder VBIOS
// as inputs.atom_rom and its parsed firmwareinfo as inputs.atom.
struct Board final {
    explicit Board(const Vbios& options = {}) : image(vbios(options)) {
        backend.device = device.get();
        mes_file.u32(56, 0xf0005000u);
        mes_file.u32(60, 0xffffffffu);
        kiq.u32(56, 0xf0005000u);
        kiq.u32(60, 0xffffffffu);
        manager->initialize({rx7900_test::fb_base + rx7900_test::arena_offset, rx7900_test::arena_bytes},
                            backend.backend());
        catalog->add(UcodeSource::pfp, pfp.view());
        catalog->add(UcodeSource::me, me.view());
        catalog->add(UcodeSource::mec, mec.view());
        catalog->add(UcodeSource::rlc, rlc_file.view());
        catalog->add(UcodeSource::mes, mes_file.view());
        catalog->add(UcodeSource::mes_kiq, kiq.view());
        catalog->add(UcodeSource::smu, smu_file.view());
        catalog->add(UcodeSource::sdma, sdma_file.view());
        catalog->add(UcodeSource::dmcub, dmcub_file.view());
        catalog->add(UcodeSource::imu, imu_file.view());
        recorder.inner = mes_firmware.io();
        recorder.boots = &device->boot_commands;
        rom = image.bytes;
        parse_atom_firmware_info({rom.data(), rom.size()}, atom_info);
        // The VBIOS posted the card at boot.
        device->reg(scratch7) = asic_init_complete;
    }

    std::unique_ptr<SimulatedNavi31> device = std::make_unique<SimulatedNavi31>();
    rx7900_test::mes_model::MesModel mes_firmware{device.get()};
    rx7900_test::VramBackend backend{};
    std::unique_ptr<rx7900::memory::Manager> manager = std::make_unique<rx7900::memory::Manager>();
    std::unique_ptr<UcodeCatalog> catalog = std::make_unique<UcodeCatalog>();
    std::unique_ptr<AsicInit> asic = std::make_unique<AsicInit>();
    std::vector<uint8_t> discovery = rx7900_test::navi31_discovery();
    std::vector<uint8_t> sos = sos_container();
    File pfp = gfx(0x40, 0x20);
    File me = gfx(0x40, 0x20);
    File mec = gfx(0x80, 0x40);
    File rlc_file = rlc(3);
    File mes_file = mes();
    File kiq = mes();
    File imu_file = imu();
    File smu_file = smu();
    File sdma_file = sdma();
    File dmcub_file = dmcub();
    File ta_file = ta_navi31();
    AsicInitPolicy policy{{1000, 1000}, {1000, 50, 10}, {100, 3}, {1000, 2100000}, true};
    std::vector<uint8_t> board = std::vector<uint8_t>(smu13_board_table_bytes, 0x5a);
    SmuPcieLink pcie{3, 6};
    Image image;
    std::vector<uint8_t> rom;
    AtomFirmwareInfo atom_info{};
    Recorder recorder;
    PciLog pci;

    AsicInputs inputs(rx7900::ByteView atom_rom) {
        return {recorder.io(), {discovery.data(), discovery.size()}, {sos.data(), sos.size()}, catalog.get(),
                ta_file.view(), manager.get(), &atom_info, PspTrainingIo{}, {board.data(), board.size()}, pcie,
                0x0000000012345000ULL, {device.get(), SimulatedNavi31::bus_master, nullptr, SimulatedNavi31::confirm},
                false, 0x000000e000000000ULL, mes_firmware.doorbells(), device->gtt(), true, InterruptReady{},
                atom_rom};
    }

    AsicInputs inputs() { return inputs({rom.data(), rom.size()}); }

    bool start() {
        return asic->run(inputs(), policy, {device.get(), SimulatedNavi31::confirm}) == AsicStatus::completed;
    }

    // suspend ends with the suspend_noirq mode-1 reset, which the MES/CP model
    // does not see on its own: its queues and HQD state are lost here.
    bool suspend() {
        const bool done = asic->suspend({pci.platform(), nullptr, nullptr}) == AsicStatus::completed;
        mes_firmware.reset();
        return done;
    }

    // S3: the card loses power, the BIOS_SCRATCH_7 POST flag included.
    void lose_power() {
        device->power_cycle();
        mes_firmware.reset();
    }
};

void resume_reposts() {
    Board b;
    expect(b.start(), "cold boot of the simulated Navi31 with the builder VBIOS");
    expect(b.asic->repost(b.inputs()) == AsicStatus::busy && b.asic->repost_report().attempts == 0,
           "repost is refused while run owns the live ASIC");

    // A suspend without power loss keeps the POST flag: resume does not re-POST.
    expect(b.suspend() && b.asic->resume(b.inputs()) == AsicStatus::completed &&
               b.asic->repost_report().attempts == 0 && b.asic->report().posted,
           "resume of a still-posted card skips repost (amdgpu_device_need_post false)");

    expect(b.suspend(), "second suspend");
    b.lose_power();
    expect(b.device->reg(scratch7) == 0, "power loss cleared ASIC_INIT_COMPLETE");
    const unsigned boots_before = b.device->boot_commands;
    b.recorder.log.clear();
    const AsicStatus resumed = b.asic->resume(b.inputs());
    const AsicReport report = b.asic->report();
    const AsicRepostReport repost = b.asic->repost_report();
    expect(resumed == AsicStatus::completed && report.step == AsicStep::complete && report.posted &&
               !report.suspended && report.resumes == 2,
           "resume of the unposted card completes through repost");
    expect(repost.status == AsicStatus::completed && repost.step == AsicStep::repost_confirm && repost.posted &&
               repost.bootloader_ready && repost.attempts == 1 && repost.atom.status == AtomStatus::completed,
           "the repost report records the bootloader wait, ASIC_Init and the posted confirmation");
    const auto writes = b.recorder.writes();
    expect(!writes.empty() && writes.front().index == atom_window && writes.front().value == 0x00123456u &&
               writes.front().boots == boots_before && b.device->reg(atom_window + 1) == 0x00654321u &&
               (b.device->reg(scratch7) & asic_init_complete) != 0,
           "ASIC_Init ran with the firmwareinfo clocks before any PSP boot command of resume");
    expect(b.device->boot_commands > boots_before, "resume then boots the PSP again (psp_resume)");
    expect(b.asic->shutdown() == AsicStatus::completed, "the re-POSTed device shuts down cleanly");
}

// Each failing resume leaves the ASIC suspended with its reset proof; a retry
// with fixed conditions completes.
void resume_failures() {
    {
        Vbios missing{};
        missing.asic_init = false;
        Board b(missing);
        expect(b.start() && b.suspend(), "boot and suspend with a VBIOS lacking ASIC_Init");
        b.lose_power();
        const unsigned boots = b.device->boot_commands;
        expect(b.asic->resume(b.inputs()) == AsicStatus::unsupported &&
                   b.asic->report().step == AsicStep::repost_atom && b.asic->report().suspended &&
                   b.asic->reset_proven() && b.device->boot_commands == boots && !b.asic->report().posted,
               "no ASIC_Init table: resume is unsupported at repost_atom, still suspended, nothing booted");
        const Image good = vbios(Vbios{});
        expect(b.asic->resume(b.inputs({good.bytes.data(), good.bytes.size()})) == AsicStatus::completed &&
                   b.asic->report().posted,
               "a resume retry with a ROM carrying ASIC_Init completes");
    }

    {
        Board b(with(Vbios::Completion::when_ready));
        expect(b.start() && b.suspend(), "boot and suspend with a conditional ASIC_Init");
        b.lose_power();
        const unsigned boots = b.device->boot_commands;
        expect(b.asic->resume(b.inputs()) == AsicStatus::failed &&
                   b.asic->report().step == AsicStep::repost_confirm && b.asic->report().suspended &&
                   b.asic->reset_proven() && b.device->boot_commands == boots &&
                   b.asic->repost_report().status == AsicStatus::failed,
               "ASIC_Init that does not set the scratch bit: failed at repost_confirm, card stays suspended");
        b.device->reg(ready_flag) = 1;
        expect(b.asic->resume(b.inputs()) == AsicStatus::completed && b.asic->repost_report().attempts == 2 &&
                   !b.asic->report().suspended,
               "the same ROM succeeds on retry once ASIC_Init can complete");
    }

    {
        Vbios broken{};
        broken.malformed = true;
        Board b(broken);
        expect(b.start() && b.suspend(), "boot and suspend with a malformed ASIC_Init");
        b.lose_power();
        expect(b.asic->resume(b.inputs()) == AsicStatus::quarantined &&
                   b.asic->report().step == AsicStep::repost_atom && b.asic->report().suspended &&
                   b.asic->reset_proven(),
               "a structurally malformed ASIC_Init quarantines; the card stays suspended");
        const size_t writes = b.recorder.writes().size();
        expect(b.asic->resume(b.inputs()) == AsicStatus::quarantined &&
                   b.asic->repost_report().atom.status == AtomStatus::quarantined &&
                   b.recorder.writes().size() == writes,
               "retrying the quarantined ROM is refused without executing it");
        Vbios fixed{};
        fixed.marker = 1;
        const Image good = vbios(fixed);
        expect(b.asic->resume(b.inputs({good.bytes.data(), good.bytes.size()})) == AsicStatus::completed &&
                   b.device->reg(atom_window + 8) == 0x0001a5a5u,
               "resume completes with a corrected ROM");
    }
}

} // namespace

int main() {
    firmware_and_success();
    scratch_sizes();
    memory_flag();
    bootloader();
    refusals();
    failures();
    resume_reposts();
    resume_failures();
    return checks.finish("atom_repost");
}

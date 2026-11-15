/*
Module: rx7900.tests.atom_firmware
Purpose: Verify firmwareinfo capability, reserved-FB and PPTable decoding bounds.
Created: 2026-10-21
Architecture: Synthetic ATOM images built field by field; no VBIOS is embedded.
*/
#include <cstdio>
#include <vector>

#include "atom_firmware.hpp"
#include "sdma.hpp"

namespace {

using namespace rx7900::hardware;

unsigned checks = 0;
unsigned failures = 0;

void expect(bool condition, const char* contract) {
    ++checks;

    if (!condition) {
        ++failures;
        std::fprintf(stderr, "FAIL: %s\n", contract);
    }
}

struct Rom final {
    std::vector<uint8_t> bytes = std::vector<uint8_t>(0x400, 0);

    void u16(size_t offset, uint16_t value) {
        bytes.at(offset) = static_cast<uint8_t>(value);
        bytes.at(offset + 1) = static_cast<uint8_t>(value >> 8);
    }

    void u32(size_t offset, uint32_t value) {
        rx7900::sdma::CommandBuffer(bytes.data(), bytes.size()).write_u32(offset, value);
    }

    void table(size_t offset, uint16_t size, uint8_t format, uint8_t revision) {
        u16(offset, size);
        bytes.at(offset + 2) = format;
        bytes.at(offset + 3) = revision;
    }

    // ROM header 2.2 at 0x100, master data 2.1 at 0x200, firmwareinfo at 0x300.
    explicit Rom(uint8_t revision = 4, uint16_t info_bytes = 0x64) {
        u16(0, 0xaa55);
        u16(0x48, 0x100);
        table(0x100, 0x30, 2, 2);
        u32(0x104, 0x4d4f5441);
        u16(0x100 + 32, 0x200);
        table(0x200, 4 + 35 * 2, 2, 1);
        u16(0x200 + 4 + 4 * 2, 0x300);
        table(0x300, info_bytes, 3, revision);
        u32(0x300 + 8, 250000);
        u32(0x300 + 12, 125000);
        u32(0x300 + 16, atom_capability_two_stage_training | atom_capability_firmware_posted);
        u32(0x300 + 24, 0x5c9);
        u32(0x300 + 60, 7);
        u32(0x300 + 84, 4096);
    }

    rx7900::ByteView view() const { return {bytes.data(), bytes.size()}; }
};

void decoding() {
    AtomFirmwareInfo info{};
    expect(parse_atom_firmware_info(Rom().view(), info) == AtomFirmwareStatus::success &&
           info.content_revision == 4 &&
           info.capability == (atom_capability_two_stage_training | atom_capability_firmware_posted) &&
           info.reserved_fb_bytes == 4u << 20 && info.pptable_id == 7 &&
           info.bootup_gfxclk_10khz == 250000 && info.bootup_uclk_10khz == 125000 &&
           info.bios_scratch_reg == 0x5c9,
           "firmwareinfo v3.4 capability, reserved size, PPTable id and boot clocks");

    expect(parse_atom_firmware_info(Rom(5).view(), info) == AtomFirmwareStatus::success &&
           info.reserved_fb_bytes == 4u << 20 && info.bootup_gfxclk_10khz == 250000,
           "v3.5 keeps reserved size; its reserved clock words are read as Linux does");

    Rom zero;
    zero.u32(0x300 + 84, 0);
    expect(parse_atom_firmware_info(zero.view(), info) == AtomFirmwareStatus::success &&
           info.reserved_fb_bytes == atom_default_reserved_fb_bytes,
           "zero reserved size falls back to the 64 KiB discovery reservation");

    expect(parse_atom_firmware_info(Rom(3, 0x40).view(), info) == AtomFirmwareStatus::success &&
           info.pptable_id == 7 && info.reserved_fb_bytes == atom_default_reserved_fb_bytes,
           "v3.3 has a PPTable id but no reserved size");
    expect(parse_atom_firmware_info(Rom(1, 0x48).view(), info) == AtomFirmwareStatus::success &&
           info.pptable_id == 0, "v3.1 reports capability only");

    expect(parse_atom_firmware_info(Rom().view(), info) == AtomFirmwareStatus::success &&
           info.bootup_socclk_10khz == 0 && info.smu_info_format == 0,
           "missing smu_info leaves the SMU boot clocks zero");
    Rom v4;
    v4.bytes.resize(0x600);
    v4.u16(0x200 + 4 + 8 * 2, 0x400);
    v4.table(0x400, 288, 4, 0);
    v4.u32(0x400 + 40, 120000);
    v4.u32(0x400 + 56, 60000);
    v4.u32(0x400 + 72, 140000);
    v4.u32(0x400 + 196, 80000);
    v4.u32(0x400 + 200, 90000);
    expect(parse_atom_firmware_info(v4.view(), info) == AtomFirmwareStatus::success &&
           info.bootup_socclk_10khz == 120000 && info.bootup_dcefclk_10khz == 60000 &&
           info.bootup_fclk_10khz == 140000 && info.bootup_dclk_10khz == 80000 &&
           info.bootup_vclk_10khz == 90000, "smu_info v4.0 boot clocks");
    Rom v36;
    v36.bytes.resize(0x600);
    v36.u16(0x200 + 4 + 8 * 2, 0x400);
    v36.table(0x400, 240, 3, 6);
    v36.u32(0x400 + 40, 110000);
    v36.u32(0x400 + 56, 60000);
    v36.u32(0x400 + 176, 70000);
    v36.u32(0x400 + 180, 75000);
    expect(parse_atom_firmware_info(v36.view(), info) == AtomFirmwareStatus::success &&
           info.bootup_socclk_10khz == 110000 && info.bootup_dcefclk_10khz == 0 &&
           info.bootup_vclk_10khz == 70000 && info.bootup_dclk_10khz == 75000,
           "smu_info v3.6 has no DCEF boot clock");
    Rom short_v4;
    short_v4.bytes.resize(0x600);
    short_v4.u16(0x200 + 4 + 8 * 2, 0x400);
    short_v4.table(0x400, 200, 4, 0);
    short_v4.u32(0x400 + 40, 120000);
    expect(parse_atom_firmware_info(short_v4.view(), info) == AtomFirmwareStatus::success &&
           info.bootup_socclk_10khz == 0 && info.smu_info_format == 4,
           "truncated smu_info ignored");
}

void refusals() {
    AtomFirmwareInfo info{};
    info.pptable_id = 99;
    expect(parse_atom_firmware_info(Rom(4, 0x50).view(), info) == AtomFirmwareStatus::invalid_rom &&
           info.pptable_id == 99, "v3.4 table too short for its fields; output unchanged");
    expect(parse_atom_firmware_info(Rom(6).view(), info) == AtomFirmwareStatus::unsupported_revision,
           "unknown content revision refused");

    Rom format;
    format.table(0x300, 0x64, 2, 4);
    expect(parse_atom_firmware_info(format.view(), info) == AtomFirmwareStatus::unsupported_revision,
           "firmwareinfo format other than 3 refused");

    Rom absent;
    absent.u16(0x200 + 4 + 4 * 2, 0);
    expect(parse_atom_firmware_info(absent.view(), info) == AtomFirmwareStatus::absent,
           "missing firmwareinfo entry reported as absent");

    Rom huge;
    huge.u32(0x300 + 84, 0x00400000u);
    expect(parse_atom_firmware_info(huge.view(), info) == AtomFirmwareStatus::invalid_rom,
           "reserved size that overflows bytes refused");

    Rom signature;
    signature.u16(0, 0);
    expect(parse_atom_firmware_info(signature.view(), info) == AtomFirmwareStatus::invalid_rom,
           "non-ROM input refused");

    Rom truncated;
    truncated.bytes.resize(0x320);
    expect(parse_atom_firmware_info(truncated.view(), info) == AtomFirmwareStatus::invalid_rom,
           "table extending past the ROM refused");
}

// vram_usagebyfirmware (master data entry 11) at 0x380 of the standard image.
Rom usage_rom(uint8_t format, uint8_t revision, uint16_t bytes) {
    Rom rom;
    rom.u16(0x200 + 4 + 11 * 2, 0x380);
    rom.table(0x380, bytes, format, revision);
    return rom;
}

void vram_usage() {
    AtomVramUsage usage{};
    expect(parse_atom_vram_usage(Rom().view(), usage) == AtomFirmwareStatus::absent &&
           usage.layout == AtomVramUsageLayout::none &&
           atom_allocate_fb_scratch(usage).scratch_bytes == 20 * 1024 &&
           atom_allocate_fb_scratch(usage).firmware_usage_bytes == 0,
           "absent table: no reservation, 20 KiB ATOM scratch");

    // v2.1 as a posted VBIOS fills it: top-of-FB firmware region, driver scratch request.
    Rom v21 = usage_rom(2, 1, 12);
    v21.u32(0x380 + 4, 24560u * 1024u - 28672u);
    v21.u16(0x380 + 8, 28672);
    v21.u16(0x380 + 10, 64);
    expect(parse_atom_vram_usage(v21.view(), usage) == AtomFirmwareStatus::success &&
           usage.layout == AtomVramUsageLayout::v2_1 && usage.firmware_start_kb == 24560u * 1024u - 28672u &&
           usage.firmware_kb == 28672 && usage.driver_kb == 64 && usage.driver_start_kb == 0,
           "v2.1 fields decoded");
    AtomFbScratch scratch = atom_allocate_fb_scratch(usage);
    expect(scratch.firmware_usage_bytes == 0 && scratch.driver_usage_bytes == 0 &&
           scratch.scratch_bytes == 64 * 1024, "v2.1 without the SR-IOV flag: only the ATOM scratch size");

    v21.u32(0x380 + 4, 0x80000000u | 0x100u);
    v21.u16(0x380 + 8, 4096);
    parse_atom_vram_usage(v21.view(), usage);
    scratch = atom_allocate_fb_scratch(usage);
    expect(scratch.firmware_usage_offset == 0x100u << 10 && scratch.firmware_usage_bytes == 4u << 20 &&
           scratch.scratch_bytes == 20 * 1024,
           "v2.1 SRIOV_MSG_SHARE_RESERVATION: fw_vram_usage at start << 10 (not SR-IOV gated)");
    v21.u32(0x380 + 4, 0x40000000u | 0x100u);
    parse_atom_vram_usage(v21.view(), usage);
    expect(atom_allocate_fb_scratch(usage).firmware_usage_bytes == 0 &&
           atom_allocate_fb_scratch(usage).scratch_bytes == 64 * 1024, "NEEDS_NO_RESERVATION flag ignored");

    Rom v22 = usage_rom(2, 2, 48);
    v22.u32(0x380 + 4, 0x80001000u);
    v22.u16(0x380 + 8, 512);
    v22.u32(0x380 + 12, 0x2000);
    v22.u32(0x380 + 16, 0x30000);
    expect(parse_atom_vram_usage(v22.view(), usage) == AtomFirmwareStatus::success &&
           usage.layout == AtomVramUsageLayout::v2_2 && usage.firmware_start_kb == 0x80001000u &&
           usage.firmware_kb == 512 && usage.driver_start_kb == 0x2000 && usage.driver_kb == 0x30000,
           "v2.2 fields decoded, 32-bit driver region0 size");
    scratch = atom_allocate_fb_scratch(usage);
    expect(scratch.firmware_usage_bytes == 0 && scratch.driver_usage_bytes == 0 &&
           scratch.scratch_bytes == 20 * 1024, "v2.2 on bare metal: no reservation, default scratch");

    expect(parse_atom_vram_usage(usage_rom(3, 4, 48).view(), usage) == AtomFirmwareStatus::success &&
           usage.layout == AtomVramUsageLayout::v2_2 && usage.format == 3 && usage.content_revision == 4,
           "frev >= 2 and crev >= 2 select the v2.2 layout as Linux does");
    expect(parse_atom_vram_usage(usage_rom(1, 3, 48).view(), usage) == AtomFirmwareStatus::success &&
           usage.layout == AtomVramUsageLayout::none &&
           parse_atom_vram_usage(usage_rom(2, 0, 48).view(), usage) == AtomFirmwareStatus::success &&
           usage.layout == AtomVramUsageLayout::none, "other revisions are ignored");

    usage.firmware_kb = 77;
    expect(parse_atom_vram_usage(usage_rom(2, 2, 20).view(), usage) == AtomFirmwareStatus::invalid_rom &&
           parse_atom_vram_usage(usage_rom(2, 1, 10).view(), usage) == AtomFirmwareStatus::invalid_rom &&
           usage.firmware_kb == 77, "tables shorter than their layout refused; output unchanged");
    Rom past = usage_rom(2, 2, 48);
    past.bytes.resize(0x3a0);
    expect(parse_atom_vram_usage(past.view(), usage) == AtomFirmwareStatus::invalid_rom,
           "table extending past the ROM refused");
}

} // namespace

int main() {
    decoding();
    refusals();
    vram_usage();
    std::printf("atom_firmware: %u checks, %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

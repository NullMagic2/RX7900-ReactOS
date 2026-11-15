/*
Module: rx7900.hardware.atom_firmware
Purpose: Read firmwareinfo capability, reserved-FB size and PPTable identifier.
Created: 2026-10-21
Architecture: Checked ByteView reads over the shared ATOM data-table lookup.
Copyright 2016 Advanced Micro Devices, Inc. Adapted Linux v6.12 atomfirmware.h,
amdgpu_atomfirmware.c (firmware flags, get_fw_reserved_fb_size, allocate_fb_scratch,
asic_init parameters),
atombios.h (VRAM operation flags) and smu_v13_0.c (get_vbios_bootup_values). Notice and provenance: licenses/AMDGPU-FIRMWARE.txt.
*/
#include "atom_firmware.hpp"

namespace rx7900::hardware {
namespace {

constexpr size_t smc_dpm_info_index = 2;
constexpr size_t firmwareinfo_index = 4;
constexpr size_t sclk_offset = 8;
constexpr size_t mclk_offset = 12;
constexpr size_t capability_offset = 16;
constexpr size_t pptable_id_offset = 60;
constexpr size_t reserved_kb_offset = 84;
constexpr size_t bios_scratch_offset = 24;
constexpr size_t smu_info_index = 8;
// atom_smu_info_v3_6 / v4_0 offsets measured by compiling the pinned atomfirmware.h.
constexpr size_t smu_socclk_offset = 40;
constexpr size_t smu_fclk_offset = 72;
constexpr size_t smu_v3_6_vclk_offset = 176;
constexpr size_t smu_v3_6_dclk_offset = 180;
constexpr size_t smu_v3_6_bytes = 240;
constexpr size_t smu_v4_0_dcefclk_offset = 56;
constexpr size_t smu_v4_0_dclk0_offset = 196;
constexpr size_t smu_v4_0_vclk0_offset = 200;
constexpr size_t smu_v4_0_bytes = 288;

void read_smu_boot_clocks(ByteView rom, AtomFirmwareInfo& info) noexcept {
    AtomTable table{};
    ByteView view(nullptr, 0);

    if (atom_data_table(rom, smu_info_index, table) != AtomTableStatus::success ||
        !rom.subview(table.base, table.bytes, view)) {
        return;
    }

    info.smu_info_format = table.format;
    info.smu_info_revision = table.revision;

    if (table.format == 3 && table.revision == 6 && table.bytes >= smu_v3_6_bytes) {
        view.read_u32(smu_socclk_offset, info.bootup_socclk_10khz);
        view.read_u32(smu_v3_6_vclk_offset, info.bootup_vclk_10khz);
        view.read_u32(smu_v3_6_dclk_offset, info.bootup_dclk_10khz);
        view.read_u32(smu_fclk_offset, info.bootup_fclk_10khz);
    } else if (table.format == 4 && table.revision == 0 && table.bytes >= smu_v4_0_bytes) {
        view.read_u32(smu_socclk_offset, info.bootup_socclk_10khz);
        view.read_u32(smu_v4_0_dcefclk_offset, info.bootup_dcefclk_10khz);
        view.read_u32(smu_v4_0_vclk0_offset, info.bootup_vclk_10khz);
        view.read_u32(smu_v4_0_dclk0_offset, info.bootup_dclk_10khz);
        view.read_u32(smu_fclk_offset, info.bootup_fclk_10khz);
    }
}

} // namespace

AtomFirmwareStatus parse_atom_firmware_info(ByteView rom, AtomFirmwareInfo& output) noexcept {
    AtomTable table{};
    const AtomTableStatus lookup = atom_data_table(rom, firmwareinfo_index, table);

    if (lookup == AtomTableStatus::absent) {
        return AtomFirmwareStatus::absent;
    }

    if (lookup != AtomTableStatus::success) {
        return lookup == AtomTableStatus::unsupported_revision ?
                   AtomFirmwareStatus::unsupported_revision : AtomFirmwareStatus::invalid_rom;
    }

    if (table.format != 3 || table.revision < 1 || table.revision > 5) {
        return AtomFirmwareStatus::unsupported_revision;
    }

    const size_t needed = table.revision >= 4 ? reserved_kb_offset + 4 :
                          table.revision == 3 ? pptable_id_offset + 4 : capability_offset + 4;
    ByteView info(nullptr, 0);

    if (table.bytes < needed || !rom.subview(table.base, table.bytes, info)) {
        return AtomFirmwareStatus::invalid_rom;
    }

    AtomFirmwareInfo candidate{};
    candidate.content_revision = table.revision;
    info.read_u32(capability_offset, candidate.capability);

    // v3.5 renamed these bootup_clk_reserved; Linux still reads them through the
    // v3.4 layout (smu_v13_0_get_vbios_bootup_values default case).
    info.read_u32(sclk_offset, candidate.bootup_gfxclk_10khz);
    info.read_u32(mclk_offset, candidate.bootup_uclk_10khz);

    if (table.bytes >= bios_scratch_offset + 4) {
        info.read_u32(bios_scratch_offset, candidate.bios_scratch_reg);
    }

    if (table.revision >= 3) {
        info.read_u32(pptable_id_offset, candidate.pptable_id);
    }

    if (table.revision >= 4) {
        uint32_t reserved_kb = 0;
        info.read_u32(reserved_kb_offset, reserved_kb);

        if (reserved_kb > (UINT32_MAX >> 10)) {
            return AtomFirmwareStatus::invalid_rom;
        }

        if (reserved_kb != 0) {
            candidate.reserved_fb_bytes = reserved_kb << 10;
        }
    }

    read_smu_boot_clocks(rom, candidate);
    output = candidate;
    return AtomFirmwareStatus::success;
}

AtomFirmwareStatus find_smu13_board_table(ByteView rom, ByteView& board) noexcept {
    constexpr size_t header_bytes = 4;
    AtomTable table{};
    const AtomTableStatus lookup = atom_data_table(rom, smc_dpm_info_index, table);

    if (lookup == AtomTableStatus::absent) {
        return AtomFirmwareStatus::absent;
    }

    if (lookup != AtomTableStatus::success) {
        return lookup == AtomTableStatus::unsupported_revision ?
                   AtomFirmwareStatus::unsupported_revision : AtomFirmwareStatus::invalid_rom;
    }

    ByteView candidate(nullptr, 0);

    if (table.bytes < header_bytes + smu13_board_table_bytes ||
        !rom.subview(table.base + header_bytes, smu13_board_table_bytes, candidate)) {
        return AtomFirmwareStatus::invalid_rom;
    }

    board = candidate;
    return AtomFirmwareStatus::success;
}

AtomFirmwareStatus parse_atom_vram_usage(ByteView rom, AtomVramUsage& output) noexcept {
    // atom_master_list_of_data_tables_v2_1 vram_usagebyfirmware and the
    // vram_usagebyfirmware_v2_1/_v2_2 offsets measured from the pinned atomfirmware.h.
    constexpr size_t vram_usage_index = 11;
    constexpr size_t start_offset = 4;
    constexpr size_t firmware_kb_offset = 8;
    constexpr size_t v2_1_driver_kb_offset = 10;
    constexpr size_t v2_1_bytes = 12;
    constexpr size_t v2_2_driver_start_offset = 12;
    constexpr size_t v2_2_driver_kb_offset = 16;
    constexpr size_t v2_2_bytes = 48;
    AtomTable table{};
    const AtomTableStatus lookup = atom_data_table(rom, vram_usage_index, table);

    if (lookup == AtomTableStatus::absent) {
        return AtomFirmwareStatus::absent;
    }

    if (lookup != AtomTableStatus::success) {
        return lookup == AtomTableStatus::unsupported_revision ?
                   AtomFirmwareStatus::unsupported_revision : AtomFirmwareStatus::invalid_rom;
    }

    AtomVramUsage candidate{};
    candidate.format = table.format;
    candidate.content_revision = table.revision;

    if (table.format == 2 && table.revision == 1) {
        candidate.layout = AtomVramUsageLayout::v2_1;
    } else if (table.format >= 2 && table.revision >= 2) {
        candidate.layout = AtomVramUsageLayout::v2_2;
    } else {
        output = candidate;
        return AtomFirmwareStatus::success;
    }

    const size_t needed = candidate.layout == AtomVramUsageLayout::v2_1 ? v2_1_bytes : v2_2_bytes;
    ByteView usage(nullptr, 0);

    if (table.bytes < needed || !rom.subview(table.base, needed, usage)) {
        return AtomFirmwareStatus::invalid_rom;
    }

    usage.read_u32(start_offset, candidate.firmware_start_kb);
    usage.read_u16(firmware_kb_offset, candidate.firmware_kb);

    if (candidate.layout == AtomVramUsageLayout::v2_1) {
        uint16_t driver_kb = 0;
        usage.read_u16(v2_1_driver_kb_offset, driver_kb);
        candidate.driver_kb = driver_kb;
    } else {
        usage.read_u32(v2_2_driver_start_offset, candidate.driver_start_kb);
        usage.read_u32(v2_2_driver_kb_offset, candidate.driver_kb);
    }

    output = candidate;
    return AtomFirmwareStatus::success;
}

AtomFbScratch atom_allocate_fb_scratch(const AtomVramUsage& usage) noexcept {
    AtomFbScratch result{};
    uint32_t usage_bytes = 0;

    if (usage.layout == AtomVramUsageLayout::v2_1) {
        if ((usage.firmware_start_kb & atom_vram_operation_flags_mask) ==
            (atom_vram_block_sriov_msg_share_reservation << atom_vram_operation_flags_shift)) {
            // Firmware requests a VRAM reservation; the scratch keeps its default.
            result.firmware_usage_offset =
                static_cast<uint64_t>(usage.firmware_start_kb & ~atom_vram_operation_flags_mask) << 10;
            result.firmware_usage_bytes = static_cast<uint64_t>(usage.firmware_kb) << 10;
        } else {
            // A 16-bit KiB count: at most 64 MiB - 1 KiB, no overflow.
            usage_bytes = usage.driver_kb << 10;
        }
    }

    // v2.2 reservations need amdgpu_sriov_vf and its usage_bytes is always zero.
    result.scratch_bytes = usage_bytes != 0 ? usage_bytes : atom_default_scratch_bytes;
    return result;
}

AtomAsicInitParameters atom_asic_init_parameters(uint32_t bootup_sclk_10khz, uint32_t bootup_mclk_10khz,
                                                 bool fb_reset) noexcept {
    constexpr uint32_t clock_mask = 0x00ffffffu;
    constexpr uint32_t flag_shift = 24;
    const uint32_t memory_flag = fb_reset ? 0u : atom_asic_init_self_refresh_exit;
    AtomAsicInitParameters result{};
    result.engine = (bootup_sclk_10khz & clock_mask) | (atom_asic_init_normal_engine << flag_shift);
    result.memory = (bootup_mclk_10khz & clock_mask) | (memory_flag << flag_shift);
    return result;
}

} // namespace rx7900::hardware

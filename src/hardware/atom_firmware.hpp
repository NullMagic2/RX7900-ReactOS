/*
Module: rx7900.hardware.atom_firmware
Purpose: Decode ATOM firmwareinfo values that steer Navi31 ASIC initialization.
Created: 2026-10-21
Architecture: Reuses the common ATOM data-table lookup; never executes ATOM code.
*/
#pragma once

#include "atom_bios.hpp"

namespace rx7900::hardware {

// atombios_firmware_capability bits used by the Linux v6.12 init path.
constexpr uint32_t atom_capability_firmware_posted = 0x00000001;
constexpr uint32_t atom_capability_gpu_virtualization = 0x00000002;
constexpr uint32_t atom_capability_two_stage_training = 0x00000400;
// Linux amdgpu_ttm_reserve_tmr falls back to DISCOVERY_TMR_OFFSET.
constexpr uint32_t atom_default_reserved_fb_bytes = 64 * 1024;

enum class AtomFirmwareStatus { success, absent, invalid_rom, unsupported_revision };

struct AtomFirmwareInfo final {
    uint8_t content_revision = 0;
    uint32_t capability = 0;
    // firmware_info v3.4/v3.5 fw_reserved_size_in_kb in bytes, else the default.
    uint32_t reserved_fb_bytes = atom_default_reserved_fb_bytes;
    // firmware_info v3.3+ pplib_pptable_id; zero selects the VBIOS table.
    uint32_t pptable_id = 0;
    uint32_t bootup_gfxclk_10khz = 0;
    uint32_t bootup_uclk_10khz = 0;
    // bios_scratch_reg_startaddr (amdgpu_atomfirmware_scratch_regs_init): dword
    // MMIO index of BIOS_SCRATCH_0; zero when the table is too short to hold it.
    uint32_t bios_scratch_reg = 0;
    // smu_info v3.6 / v4.0 boot clocks (smu_v13_0_get_vbios_bootup_values); zero
    // when smu_info is absent or another revision, exactly as Linux leaves them.
    uint32_t bootup_socclk_10khz = 0;
    uint32_t bootup_dcefclk_10khz = 0;
    uint32_t bootup_vclk_10khz = 0;
    uint32_t bootup_dclk_10khz = 0;
    uint32_t bootup_fclk_10khz = 0;
    uint8_t smu_info_format = 0;
    uint8_t smu_info_revision = 0;
};

// Any IRQL, resident immutable ROM bytes (the validated first ATOM image).
// Accepts firmwareinfo format 3, content revision 1..5, with every decoded
// field inside the declared table size. Output unchanged on failure. A missing,
// truncated or other-revision smu_info table leaves its boot clocks zero.
AtomFirmwareStatus parse_atom_firmware_info(ByteView rom, AtomFirmwareInfo& output) noexcept;

// smc_dpm_info for SMU13.0.0 (smu_v13_0_0_ppt.c atom_smc_dpm_info_table_13_0_0):
// a common header followed by BoardTable_t, measured at 488 bytes from the pinned
// smu13_driver_if_v13_0_0.h. Linux copies it without a revision check; this
// requires the whole board table inside the declared table. The output view
// borrows rom and is unchanged on failure.
constexpr uint32_t smu13_board_table_bytes = 488;
AtomFirmwareStatus find_smu13_board_table(ByteView rom, ByteView& board) noexcept;

// atombios.h ATOM_VRAM_OPERATION_FLAGS_MASK/SHIFT and its block values, carried
// in bits 31:30 of the vram_usagebyfirmware start addresses.
constexpr uint32_t atom_vram_operation_flags_mask = 0xc0000000u;
constexpr uint32_t atom_vram_operation_flags_shift = 30;
constexpr uint32_t atom_vram_block_needs_no_reservation = 0x1;
constexpr uint32_t atom_vram_block_sriov_msg_share_reservation = 0x2;
// amdgpu_atomfirmware_allocate_fb_scratch: ATOM interpreter scratch default.
constexpr uint32_t atom_default_scratch_bytes = 20u * 1024u;

// The layout amdgpu_atomfirmware_allocate_fb_scratch selects: format 2 content 1
// is vram_usagebyfirmware_v2_1 (12 bytes), any format >= 2 with content >= 2 is
// read as vram_usagebyfirmware_v2_2 (48 bytes), everything else is ignored.
enum class AtomVramUsageLayout : uint8_t { none, v2_1, v2_2 };

// Raw little-endian fields, flag bits included. v2.1 keeps used_by_driver_in_kb
// (16 bit) in driver_kb and has no driver start; v2.2 has both region0 fields.
struct AtomVramUsage final {
    AtomVramUsageLayout layout = AtomVramUsageLayout::none;
    uint8_t format = 0;
    uint8_t content_revision = 0;
    uint32_t firmware_start_kb = 0;
    uint16_t firmware_kb = 0;
    uint32_t driver_start_kb = 0;
    uint32_t driver_kb = 0;
};

// Any IRQL, resident immutable validated ROM. absent: master entry 11 is zero
// (Linux then only uses the 20 KiB scratch default). success with layout none:
// a revision Linux ignores. invalid_rom: a selected layout does not fit the
// declared table (Linux reads it without a size check; this refuses). Output
// unchanged on failure.
AtomFirmwareStatus parse_atom_vram_usage(ByteView rom, AtomVramUsage& output) noexcept;

// amdgpu_atomfirmware_allocate_fb_v2_1/_v2_2 on bare metal (amdgpu_sriov_vf
// false), as byte values: v2.1 with SRIOV_MSG_SHARE_RESERVATION in the start
// flags requests mman.fw_vram_usage (that test is not SR-IOV gated in v6.12);
// every v2.2 request is SR-IOV only and therefore empty here. scratch_bytes is
// the ATOM interpreter scratch (system memory: v2.1 used_by_driver_in_kb or the
// 20 KiB default), never a VRAM reservation.
struct AtomFbScratch final {
    uint64_t firmware_usage_offset = 0;
    uint64_t firmware_usage_bytes = 0;
    uint64_t driver_usage_offset = 0;
    uint64_t driver_usage_bytes = 0;
    uint32_t scratch_bytes = atom_default_scratch_bytes;
};

AtomFbScratch atom_allocate_fb_scratch(const AtomVramUsage& usage) noexcept;

// asic_init_ps_allocation_v2_1 (atomfirmware.h) as amdgpu_atomfirmware_asic_init
// fills it: param.engineparam {sclkfreqin10khz:24, engineflag:8}, then
// param.memparam {mclkfreqin10khz:24, memflag:8}, then reserved[16], all zero
// otherwise. engineflag is b3NORMAL_ENGINE_INIT; memflag is 0 when fb_reset
// (amdgpu_device_asic_init passes true) else b3DRAM_SELF_REFRESH_EXIT. Clocks
// keep their low 24 bits, as the C bitfield assignment does.
constexpr uint32_t atom_asic_init_parameter_dwords = 18;
constexpr uint32_t atom_asic_init_normal_engine = 0x00;
constexpr uint32_t atom_asic_init_self_refresh_exit = 0x20;

struct AtomAsicInitParameters final {
    uint32_t engine = 0;
    uint32_t memory = 0;
};

AtomAsicInitParameters atom_asic_init_parameters(uint32_t bootup_sclk_10khz, uint32_t bootup_mclk_10khz,
                                                 bool fb_reset) noexcept;

} // namespace rx7900::hardware

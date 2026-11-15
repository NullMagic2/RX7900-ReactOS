/*
Module: rx7900.reactos.vbios
Purpose: Acquire the Navi31 ATOM VBIOS from the card itself, Linux v6.12 order.
Created: 2026-10-22
Architecture: Native resource owner for the legacy shadow, PCI expansion-ROM BAR
and SMUIO ROM sources; src/hardware/vbios_rom owns every protocol and validity
rule, driver_blob owns the resulting nonpaged storage in HardwareState.
*/
#pragma once

#include "hardware.hpp"
#include "vbios_rom.hpp"
#include "vram_discovery.hpp"

namespace rx7900::reactos {

// Linux amdgpu_get_bios_dgpu sources in its order, then the administrator file.
enum class VbiosSource : uint8_t {
    none,
    atrm,
    vfct,
    vram,
    platform,
    rom_shadow,
    rom_bar,
    smuio_rom,
    administrator_file
};

constexpr size_t vbios_source_count = 9;

enum class VbiosAttempt : uint8_t {
    not_attempted,
    // ReactOS offers this miniport no interface for the source.
    unavailable,
    // Deliberately not tried here (precondition unmet or not applicable).
    skipped,
    // The source exists but holds no image (for example an unassigned ROM BAR).
    absent,
    // Bytes were read but failed validation; see VbiosReport::validation.
    rejected,
    // An OS mapping, config access or MMIO operation failed.
    failed,
    accepted
};

enum class VbiosPrecedence {
    // Card sources in Linux order, then rx7900.rom as the last resort.
    card_first,
    // Explicit administrator override: rx7900.rom only; card sources untouched.
    administrator_file_only
};

struct VbiosReport final {
    VbiosSource source = VbiosSource::none;
    VbiosAttempt attempts[vbios_source_count]{};
    hardware::VbiosStatus validation[vbios_source_count]{};
    // Expansion ROM BAR (config offset 0x30) as found, its sized decode window,
    // and whether the exact original value was confirmed after every access.
    uint32_t rom_bar_original = 0;
    uint32_t rom_bar_window_bytes = 0;
    bool rom_bar_restored = true;
    hardware::VramDiscoveryStatus discovery = hardware::VramDiscoveryStatus::invalid_argument;
    hardware::SmuioRomReport smuio{};
    NTSTATUS file_status = STATUS_SUCCESS;
    // Accepted candidate length and its validated first-image extent.
    uint32_t candidate_bytes = 0;
    uint32_t image_bytes = 0;
};

// PASSIVE_LEVEL with APCs enabled, in HwInitialize after acquire and before any
// interrupt, ASIC initialization or other BAR5 user: the shadow/ROM BAR copies,
// the MM_INDEX discovery read and SMUIO ROM_INDEX/ROM_DATA require exclusive,
// single-threaded ownership. Fills the storage acquire_atom_rom fills
// (state.rom, state.rom_image_bytes) with the first candidate that passes
// hardware::validate_vbios_image for the assigned vendor/device; the shadow must
// also match the config-space subsystem. discovery is caller-owned storage (for
// example AsicBringup::discovery): reused when already acquired, otherwise read
// from VRAM only when the SMUIO source is reached. The ROM BAR register is
// enabled only for the copy and restored to its exact original value; a restore
// that cannot be confirmed stops acquisition with STATUS_DEVICE_CONFIGURATION_ERROR.
// Never writes PCI COMMAND, BARs other than the ROM BAR, or any GPU register
// other than the MM_INDEX/MM_INDEX_HI, NBIO SMN index and SMUIO ROM_INDEX.
// STATUS_NOT_FOUND when no source produced a valid image; report says why.
NTSTATUS acquire_vbios(HardwareState& state, PVOID extension, VbiosPrecedence precedence,
                       hardware::DiscoveryBinary& discovery, VbiosReport& report) noexcept;

} // namespace rx7900::reactos

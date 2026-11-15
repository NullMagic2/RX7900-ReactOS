/*
Module: rx7900.reactos.hardware
Purpose: Own native VideoPort register mappings and bounded hardware access.
Created: 2026-10-21
Architecture: Called only by the device-owning miniport, never by diagnostic IOCTLs.
*/
#pragma once

#include "video_sdk.hpp"

#include "pci.hpp"
#include "driver_blob.hpp"

namespace rx7900::reactos {

// Native device extension owns this state. Construct before publication, release
// exactly once at PASSIVE_LEVEL after all callbacks and DMA users are stopped.
// Readiness for GPU firmware, GPUVM, rings and DCN is separate from MMIO mapping.
struct HardwareState final {
    Adapter adapter{};
    PULONG registers = nullptr;
    ULONG register_bytes = 0;
    ULONG pci_slot = 0;
    bool acquired = false;
    BlobStorage rom{};
    uint32_t rom_image_bytes = 0;
    // Assigned BAR0 (VRAM aperture) range recorded at acquisition; not mapped.
    PHYSICAL_ADDRESS vram_range{};
    ULONG vram_range_bytes = 0;
    // Assigned 64-bit BAR2 doorbell aperture; mapped only for physical init.
    PHYSICAL_ADDRESS doorbell_range{};
    ULONG doorbell_range_bytes = 0;
    PULONG doorbells = nullptr;
    ULONG doorbell_bytes = 0;
    // VideoPort-assigned interrupt recorded at acquisition (VIDEO_PORT_CONFIG_INFO
    // BusInterruptLevel/BusInterruptVector/InterruptMode). VideoPort connects
    // HwInterrupt to it after HwFindAdapter whenever either value is nonzero; PCI
    // devices get LevelSensitive, shared per the PnP resource descriptor.
    ULONG interrupt_level = 0;
    ULONG interrupt_vector = 0;
    KINTERRUPT_MODE interrupt_mode = LevelSensitive;
    bool interrupt_assigned = false;
};

// HwFindAdapter/PASSIVE_LEVEL. Claims VideoPort-assigned ranges, validates PCI
// identity and maps only the actual BAR5 register aperture. A 64-bit BAR0 range
// is recorded, never mapped here and never used to infer VRAM capacity. No register writes,
// resets, bus-master enabling or firmware operations are implicit in acquisition.
VP_STATUS acquire(HardwareState& state, PVOID extension,
                  PVIDEO_PORT_CONFIG_INFO config) noexcept;
void release(HardwareState& state, PVOID extension) noexcept;

// Resident mapping, serialized against release at caller's hardware IRQL. Byte
// offsets must be aligned and entirely in BAR5. The miniport alone may call these;
// they are not exposed through a user-supplied physical/register-address request.
bool read_register(HardwareState& state, uint32_t byte_offset, uint32_t& value) noexcept;
bool write_register(HardwareState& state, uint32_t byte_offset, uint32_t value) noexcept;

// Same bounds and uncached mapping for a short-lived BAR0 sub-range owned by the
// caller, who must call unmap_vram_temporary before release() and never retain it.
VP_STATUS map_vram_temporary(const HardwareState& state, PVOID extension, uint64_t offset,
                             ULONG bytes, PUCHAR& output) noexcept;
void unmap_vram_temporary(PVOID extension, PUCHAR mapping) noexcept;

// PASSIVE_LEVEL, physical initialization only: uncached mapping of the whole
// assigned doorbell BAR (at most 8 MiB), released by release().
VP_STATUS map_doorbells(HardwareState& state, PVOID extension) noexcept;
// Resident mapping, exclusive owner. Linux WDOORBELL32; WDOORBELL64 is two
// ordered 32-bit stores (low then high) because i386 has no 64-bit MMIO store.
bool write_doorbell32(HardwareState& state, uint32_t dword_index, uint32_t value) noexcept;
bool write_doorbell64(HardwareState& state, uint32_t dword_index, uint64_t value) noexcept;

// PASSIVE_LEVEL, physical initialization only (Linux pci_set_master from IH
// hw_init): sets PCI COMMAND bus-master enable on the owned function, preserving
// every other bit, and confirms it by readback.
bool enable_bus_master(const HardwareState& state, PVOID extension) noexcept;

// PASSIVE_LEVEL/APCs enabled. Fixed rx7900.rom path, matched PCIR/ATOM metadata.
// This file source alone touches no hardware; vbios.hpp acquire_vbios adds the
// card's own legacy shadow, ROM BAR and SMUIO sources and calls this last.
NTSTATUS acquire_atom_rom(HardwareState& state) noexcept;
// Borrowed immutable image, retained by the device until explicit release.
ByteView atom_rom(const HardwareState& state) noexcept;

} // namespace rx7900::reactos

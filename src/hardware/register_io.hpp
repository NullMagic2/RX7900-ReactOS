/*
Module: rx7900.hardware.register_io
Purpose: Share one bounded register-callback contract across physical-init protocols.
Created: 2026-10-21
Architecture: Protocol modules decode; the native device owner performs real MMIO.
*/
#pragma once

#include "discovery.hpp"

namespace rx7900::hardware {

// Supplied by the exclusive native device owner for the assigned BAR5 register
// aperture. read32 returns false for an unmapped or faulted access; write32 true
// confirms an ordered posted store, false means the device state is uncertain.
// delay_microseconds waits at least the requested time at PASSIVE_LEVEL. The
// context and mapping outlive every protocol object holding this descriptor.
struct RegisterIo final {
    void* context;
    uint32_t aperture_bytes;
    bool (*read32)(void*, uint32_t byte_offset, uint32_t&) noexcept;
    bool (*write32)(void*, uint32_t byte_offset, uint32_t) noexcept;
    void (*delay_microseconds)(void*, uint32_t) noexcept;
};

inline bool register_io_valid(const RegisterIo& io) noexcept {
    return io.read32 != nullptr && io.write32 != nullptr &&
           io.delay_microseconds != nullptr && io.aperture_bytes != 0 &&
           (io.aperture_bytes & 3u) == 0;
}

// Discovered dword segment base plus register index; base zero addresses the
// fixed pre-discovery SOC15 indices. Rejects any access outside the aperture.
inline bool read_ip_register(const RegisterIo& io, uint32_t base, uint32_t reg,
                             uint32_t& value) noexcept {
    uint32_t offset = 0;

    if (!register_io_valid(io) ||
        !discovery_register_offset(base, reg, io.aperture_bytes, offset)) {
        return false;
    }

    return io.read32(io.context, offset, value);
}

inline bool write_ip_register(const RegisterIo& io, uint32_t base, uint32_t reg,
                              uint32_t value) noexcept {
    uint32_t offset = 0;

    if (!register_io_valid(io) ||
        !discovery_register_offset(base, reg, io.aperture_bytes, offset)) {
        return false;
    }

    return io.write32(io.context, offset, value);
}

// Doorbell BAR (BAR2) access for ring write pointers (Linux WDOORBELL32/64).
// dword_index is the Linux doorbell index in dwords; a 64-bit doorbell occupies
// two consecutive dwords. Null callbacks mean no doorbell aperture is owned, so
// protocols fall back to Linux's non-doorbell register path or refuse.
struct DoorbellIo final {
    void* context;
    uint32_t aperture_dwords;
    bool (*write32)(void*, uint32_t dword_index, uint32_t value) noexcept;
    bool (*write64)(void*, uint32_t dword_index, uint64_t value) noexcept;
};

inline bool doorbell_io_valid(const DoorbellIo& io) noexcept {
    return io.write32 != nullptr && io.write64 != nullptr && io.aperture_dwords != 0;
}

// System memory for GTT buffers (Linux GTT kernel BOs bound through the GART).
// allocate returns one pinned, physically contiguous, cache-coherent buffer of
// at least bytes, with a 4 KiB-aligned DMA bus address and a CPU mapping; token
// identifies it to release. release is called only after the GART entries were
// unbound and a reset or retirement proved no engine still uses the pages.
struct GttPlatform final {
    void* context;
    bool (*allocate)(void*, uint32_t bytes, uint8_t*& cpu, uint64_t& bus, void*& token) noexcept;
    bool (*release)(void*, void* token) noexcept;
};

// PCI services the explicit mode-1 reset needs (amdgpu_device_mode1_reset):
// cache and later restore the function's config space, and clear bus mastering.
struct ResetPlatform final {
    void* context;
    bool (*cache_pci_state)(void*) noexcept;
    bool (*clear_bus_master)(void*) noexcept;
    bool (*restore_pci_state)(void*) noexcept;
};

inline bool gtt_platform_valid(const GttPlatform& platform) noexcept {
    return platform.allocate != nullptr && platform.release != nullptr;
}

} // namespace rx7900::hardware

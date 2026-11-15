/*
Module: rx7900.hardware.vram_discovery
Purpose: Acquire the IP-discovery binary that IFWI leaves near the top of VRAM.
Created: 2026-10-21
Architecture: Bounded indirect MM_INDEX/MM_DATA reads into device-owned storage;
discovery.hpp remains the only parser and checksum validator.
*/
#pragma once

#include "register_io.hpp"

namespace rx7900::hardware {

constexpr uint32_t discovery_tmr_bytes = 10 * 1024;
constexpr uint32_t discovery_tmr_offset = 64 * 1024;

enum class VramDiscoveryStatus { acquired, invalid_argument, mmio_failure, no_vram, invalid_binary };

struct VramDiscoveryReport final {
    VramDiscoveryStatus status = VramDiscoveryStatus::invalid_argument;
    bool ifwi_ready = false;
    uint32_t ifwi_status = 0;
    uint32_t memsize_mib = 0;
    uint64_t vram_bytes = 0;
    uint64_t source_offset = 0;
};

// Fixed-capacity, device-owned snapshot. bytes() is empty unless the most recent
// read completed and carried the discovery signature; checksums, extents and IP
// versions are validated only by discover_ip and its callers. Noncopyable so a
// large snapshot is never duplicated implicitly in kernel stack frames.
class DiscoveryBinary final {
public:
    DiscoveryBinary() noexcept = default;
    DiscoveryBinary(const DiscoveryBinary&) = delete;
    DiscoveryBinary& operator=(const DiscoveryBinary&) = delete;

    ByteView bytes() const noexcept {
        return acquired_ ? ByteView(bytes_, sizeof(bytes_)) : ByteView(nullptr, 0);
    }

private:
    friend VramDiscoveryStatus read_vram_discovery(const RegisterIo&, DiscoveryBinary&,
                                                   VramDiscoveryReport&) noexcept;
    uint8_t bytes_[discovery_tmr_bytes]{};
    bool acquired_ = false;
};

// PASSIVE_LEVEL, exclusive register owner. MM_INDEX/MM_INDEX_HI/MM_DATA are a
// shared indirect window: no ISR, DPC or other thread may use them during the
// call. Waits up to 1000 x 1 ms for IFWI (C2PMSG_33 bit 31) exactly as Linux,
// then continues even when not ready and reports it. VRAM size comes from
// RCC_CONFIG_MEMSIZE, never from a BAR length. Reads only; writes affect the
// indirect window index registers alone. Output is cleared first; on failure
// bytes() is empty. Any all-ones register read is treated as a lost device.
VramDiscoveryStatus read_vram_discovery(const RegisterIo& io, DiscoveryBinary& output,
                                        VramDiscoveryReport& report) noexcept;

// amdgpu_device_mm_access read of bytes (dword multiple, dword-aligned position)
// into caller-owned writable storage of at least bytes. window_high caches the
// last MM_INDEX_HI value; pass UINT32_MAX for a fresh sequence. Same exclusive
// indirect-window rule as read_vram_discovery. False on any MMIO failure.
bool read_vram_block(const RegisterIo& io, uint64_t position, uint8_t* output, uint32_t bytes,
                     uint32_t& window_high) noexcept;

// amdgpu_device_mm_access write (amdgpu_device_vram_access with write true and no
// visible-VRAM shortcut): input.size() bytes, a dword multiple, to a dword-aligned
// position through the same exclusive MM_INDEX/MM_INDEX_HI/MM_DATA window and the
// same window_high cache. False on any MMIO failure; earlier dwords may be written.
bool write_vram_block(const RegisterIo& io, uint64_t position, ByteView input,
                      uint32_t& window_high) noexcept;

// Linux psp_get_runtime_db_entry: cookie 0x0ed5 header 1 MiB below the VRAM top,
// then a 64-entry directory. An absent cookie or invalid count means "no
// database" exactly as Linux; each entry is validated by its declared size.
constexpr uint32_t psp_runtime_db_offset = 0x100000;
constexpr uint32_t psp_boot_config_two_stage_training = 0x2;

struct PspRuntimeDatabase final {
    bool present = false;
    bool boot_config_valid = false;
    uint32_t boot_config = 0;
    bool scpm_valid = false;
    uint32_t scpm_status = 0;
};

// Same exclusive-owner and indirect-window rules as read_vram_discovery. vram_bytes
// comes from RCC_CONFIG_MEMSIZE. acquired with present=false is a valid outcome.
VramDiscoveryStatus read_psp_runtime_database(const RegisterIo& io, uint64_t vram_bytes,
                                              PspRuntimeDatabase& output) noexcept;

} // namespace rx7900::hardware

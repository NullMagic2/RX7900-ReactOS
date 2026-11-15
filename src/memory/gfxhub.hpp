/*
Module: rx7900.memory.gfxhub
Purpose: Execute a narrow GFXHUB3.0 flat VMID0 GART register protocol.
Created: 2026-10-21
Architecture: Native-owned MMIO/common buffers; no physical acquisition or ASIC reset.
*/
#pragma once

#include "page_table.hpp"

namespace rx7900 {
namespace memory {

enum class GfxhubStatus {
    discovery_validated,
    configured,
    mapped,
    unmapped,
    disabled,
    invalid_argument,
    invalid_discovery,
    unsupported,
    dependencies_unmet,
    unavailable,
    busy,
    mmio_failure,
    timeout,
    quarantined
};

class GcRegisterBase final {
public:
    GcRegisterBase() noexcept = default;
    bool valid() const noexcept { return valid_; }
    uint32_t segment(size_t index) const noexcept { return index < 2 ? bases_[index] : 0; }
    uint32_t aperture_bytes() const noexcept { return aperture_bytes_; }

private:
    friend GfxhubStatus validate_gc_discovery(ByteView, uint32_t, GcRegisterBase&) noexcept;
    uint32_t bases_[2]{};
    uint32_t aperture_bytes_ = 0;
    bool valid_ = false;
};

// Validate the native-acquired immutable discovery binary's signatures, byte-sum
// checksums, die/IP extents and unique GC11.0.0 instance0 entry before accepting
// segments0/1. Supports IP table versions3/4 and local low30-bit dword bases;
// remote/XCC address extensions fail rather than being truncated. Output cleared
// on failure. Provenance is hardware discovery, never a PCI-ID/base guess.
GfxhubStatus validate_gc_discovery(ByteView binary, uint32_t register_aperture_bytes,
                                  GcRegisterBase& output) noexcept;

struct GfxhubIo final {
    void* context;
    bool (*read32)(void*, uint32_t byte_offset, uint32_t&) noexcept;
    bool (*write32)(void*, uint32_t byte_offset, uint32_t) noexcept;
    // Release CPU PTE stores, flush the native HDP visibility path and confirm
    // completion. A CPU barrier alone is insufficient for noncoherent mappings.
    bool (*publish_ptes)(void*) noexcept;
    void (*stall_one_microsecond)(void*) noexcept;
};

struct FlatGart final {
    sdma::GpuAllocation virtual_aperture;
    uint8_t* table_cpu_bytes;
    size_t table_cpu_byte_count;
    uint64_t table_dma_bus_address;
    uint64_t vram_mc_start;
    uint64_t vram_byte_count;
    uint32_t poll_iterations;
};

// Narrow bring-up protocol, serialized at PASSIVE_LEVEL by the exclusive native
// device owner. No ISR/DPC/reentrant calls. The caller holds GPU power/clock and
// reset exclusion throughout. It owns coherent pinned table backing (contiguous
// DMA bus span), discovery, MMIO and all mapped pages until disable confirms.
//
// Existing MC/system apertures, safe scratch/default pages, L1/L2 configuration
// and translate_further=false must already match the Linux GFXHUB3.0 baseline;
// this class checks the relevant hardware fields instead of declaring readiness.
// It requires every VMID disabled initially and verifies GFX idle plus both SDMA
// engines idle AND halted before changing PTEs. All later queue producers must
// share exclusion with map/unmap/disable. Other ASIC/IP versions are unsupported.
// It programs VMID0 only: no PSP/SMU firmware, GFX power-on, MC/cache global setup,
// bus-master enabling, MMHUB/display aperture or user VMID initialization occurs.
class Gfxhub final {
public:
    Gfxhub() noexcept = default;
    Gfxhub(const Gfxhub&) = delete;
    Gfxhub& operator=(const Gfxhub&) = delete;

    GfxhubStatus initialize(GcRegisterBase registers, FlatGart configuration,
                            GfxhubIo io) noexcept;
    GfxhubStatus map(uint64_t virtual_address, ByteView dma_page_addresses,
                     Access access) noexcept;
    GfxhubStatus unmap(uint64_t virtual_address, uint64_t byte_count) noexcept;
    // Failure after hardware/PTE mutation quarantines the entire table. Keep
    // table AND backing pages pinned; only a later confirmed disable allows
    // release. No implicit destructor can establish hardware quiescence.
    GfxhubStatus disable() noexcept;

private:
    bool read(size_t segment, uint32_t reg, uint32_t& value) const noexcept;
    bool write(uint32_t reg, uint32_t value) const noexcept;
    bool write_checked(uint32_t reg, uint32_t value) const noexcept;
    GfxhubStatus idle() const noexcept;
    GfxhubStatus dependencies() const noexcept;
    GfxhubStatus invalidate() const noexcept;
    GfxhubStatus resume_translation() noexcept;
    bool set_context_enabled(bool enabled) const noexcept;
    bool window(uint64_t address, uint64_t bytes, size_t& table_offset,
                size_t& table_bytes) const noexcept;

    GcRegisterBase registers_{};
    FlatGart configuration_{};
    GfxhubIo io_{};
    size_t used_table_bytes_ = 0;
    bool bound_ = false;
    bool quarantined_ = false;
};

} // namespace memory
} // namespace rx7900

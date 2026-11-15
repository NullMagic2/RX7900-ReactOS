/*
Module: rx7900.hardware.nbio
Purpose: Own NBIO4.3 HDP publication, SMN index access and MMHUB3.0 VRAM location.
Created: 2026-10-21
Architecture: Discovery-validated register token; native owner supplies MMIO.
*/
#pragma once

#include "register_io.hpp"
#include "manager.hpp"

namespace rx7900::hardware {

// Linux nbio_v4_3_set_reg_remap places the HDP flush registers in this BAR5 hole.
constexpr uint32_t hdp_remap_byte_offset = 0x80000 - 0x1000;

enum class NbioStatus {
    ready,
    unavailable,
    unsupported_ip,
    mmio_failure,
    readback_mismatch,
    invalid_location
};

// MC addresses are what PSP, SMU and GPU engines use for local VRAM. They are
// not BAR0 bus addresses, CPU physical addresses or GPUVM virtual addresses.
// physical_offset is MMHUB FB_OFFSET, used where firmware wants VRAM "PA".
struct VramLocation final {
    uint64_t mc_start;
    uint64_t mc_end;
    uint64_t bytes;
    uint64_t physical_offset;
};

// Retain one read/write lease on a Manager allocation whose GPU address is an MC
// address wholly inside local VRAM, aligned and at least minimum_bytes long;
// cpu_visible also requires that many CPU-mapped backing bytes. Output and
// leases are unchanged on failure. Shared by the PSP ring and SMU table owners.
bool retain_local_vram(memory::Manager& manager, memory::Handle handle,
                       const VramLocation& vram, uint64_t minimum_bytes, uint64_t alignment,
                       bool cpu_visible, memory::AllocationInfo& output) noexcept;

// Only validate_nbio_discovery creates a valid token: unique NBIF instance0 4.3.0
// with six local segments and MMHUB instance0 3.0.0. The HDP remap hole, RSMU
// pair, remap/memsize and MMHUB registers must fit BAR5; the function2 strap may
// lie beyond it and is then reached through RSMU. No PCI-ID inference.
class NbioRegisters final {
public:
    NbioRegisters() noexcept = default;
    bool valid() const noexcept { return nbif_.valid() && mmhub_.valid() && aperture_bytes_ != 0; }
    uint32_t nbif_base(size_t segment) const noexcept { return nbif_.segment(segment); }
    uint32_t mmhub_base() const noexcept { return mmhub_.segment(0); }
    uint32_t aperture_bytes() const noexcept { return aperture_bytes_; }

private:
    friend DiscoveryStatus validate_nbio_discovery(ByteView, uint32_t, NbioRegisters&) noexcept;
    DiscoveredIp nbif_{};
    DiscoveredIp mmhub_{};
    uint32_t aperture_bytes_ = 0;
};

DiscoveryStatus validate_nbio_discovery(ByteView binary, uint32_t register_aperture_bytes,
                                        NbioRegisters& output) noexcept;

// Serialized PASSIVE_LEVEL use by the exclusive device owner. The SMN index/data
// pair is shared state: no ISR/DPC may use it concurrently. A failed write leaves
// hardware state uncertain and the caller must stop dependent initialization.
class Nbio final {
public:
    Nbio() noexcept = default;

    NbioStatus initialize(NbioRegisters registers, RegisterIo io) noexcept;

    // Linux soc21_common_hw_init: NBIO4.3.0 clears STRAP_NO_SOFT_RESET for
    // function2, remaps HDP flush registers to the BAR5 hole (confirmed by
    // readback) and enables the doorbell BAR aperture. ASPM is policy and skipped.
    NbioStatus program_common() noexcept;

    // nbio_v4_3_enable_doorbell_aperture (RCC_DOORBELL_APER_EN).
    NbioStatus enable_doorbell_aperture(bool enable) noexcept;
    // soc21_common_late_init: nbio_v4_3_enable_doorbell_selfring_aperture with
    // the doorbell BAR bus base (adev->doorbell.base), mode 1, size 0.
    NbioStatus enable_doorbell_selfring(uint64_t doorbell_bus_base) noexcept;
    // soc21_common_hw_fini: SELFRING_GPA_APER_CNTL = 0 (enable false writes no base).
    NbioStatus disable_doorbell_selfring() noexcept;
    // nbio_v4_3_get_memsize: RCC_CONFIG_MEMSIZE (all ones while the ASIC is in reset).
    bool read_memsize(uint32_t& value) noexcept;
    // gfx_v11_0_hw_init: nbio_v4_3_gc_doorbell_init S2A entries 0 and 3.
    NbioStatus gc_doorbell_init() noexcept;

    // CPU writes through the BAR0 VRAM aperture become visible to GPU/PSP only
    // after this HDP flush (Linux hdp_v6_0_flush_hdp without a ring). HDP6 has
    // no invalidate operation. Requires a successful program_common.
    bool flush_hdp() noexcept;

    // MMHUB FB_LOCATION/FB_OFFSET with RCC_CONFIG_MEMSIZE, as gmc_v11_0. Fails
    // when the reported size does not fit the programmed FB window.
    NbioStatus vram_location(VramLocation& output) noexcept;

    // System Management Network access through BIF RSMU index/data (Linux
    // amdgpu_device_indirect_rreg/wreg), including the index posting readback.
    bool read_smn(uint32_t address, uint32_t& value) noexcept;
    bool write_smn(uint32_t address, uint32_t value) noexcept;

    // Linux amdgpu_device_rreg/wreg: a register inside BAR5 is direct MMIO; a
    // discovered register beyond the aperture uses the SMN pair at byte reg*4.
    bool read_register(uint32_t base, uint32_t reg, uint32_t& value) noexcept;
    bool write_register(uint32_t base, uint32_t reg, uint32_t value) noexcept;

private:
    NbioRegisters registers_{};
    RegisterIo io_{};
    bool initialized_ = false;
    bool remapped_ = false;
};

} // namespace rx7900::hardware

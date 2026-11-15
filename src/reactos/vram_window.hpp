/*
Module: rx7900.reactos.vram_window
Purpose: Map CPU-visible VRAM allocations through BAR0 on demand, uncached.
Created: 2026-10-22
Architecture: Native resource owner beside HardwareState; supplies the portable
hardware::VramCpuMapper used by VramBackend and maps direct VramAllocator BOs.

On-demand windows rather than one whole-BAR0 view: ReactOS assigns a 256 MiB
BAR0 (no resizable BAR), and a permanent 256 MiB i386 kernel mapping would take
65536 system PTEs from a small pool. Each mapping covers exactly one allocation;
a byte budget bounds the total. Linux maps visible VRAM write-combined
(ioremap_wc); this module keeps the existing uncached VideoPort mapping so CPU
stores reach VRAM in order before an HDP flush, as the bring-up path requires.
*/
#pragma once

#include "hardware.hpp"
#include "vram_backend.hpp"

namespace rx7900::reactos {

// One Manager pool (64 slots) plus direct users.
constexpr size_t vram_aperture_mapping_capacity = 128;
constexpr uint64_t vram_aperture_default_budget_bytes = 64ULL << 20;

// Construct once in the zeroed device extension; trivially destructible.
// Externally serialized; every call that maps or unmaps requires PASSIVE_LEVEL
// (VideoPortGetDeviceBase/VideoPortFreeDeviceBase). HardwareState and the
// extension must outlive this object; call release before hardware release().
// CPU views returned here are borrowed: valid until the matching unmap.
class VramAperture final {
public:
    VramAperture() noexcept = default;
    VramAperture(const VramAperture&) = delete;
    VramAperture& operator=(const VramAperture&) = delete;

    // Records visible_vram_size = min(assigned BAR0 bytes, vram_bytes)
    // (gmc_v11_0_mc_init), rounded down to whole pages. Maps nothing. Fails
    // without an acquired BAR0, outside PASSIVE_LEVEL or with a zero budget.
    VP_STATUS initialize(HardwareState& hardware, PVOID extension, uint64_t vram_bytes,
                         uint64_t budget_bytes) noexcept;
    uint64_t visible_bytes() const noexcept { return visible_bytes_; }
    uint64_t mapped_bytes() const noexcept { return mapped_bytes_; }

    // Uncached view of VRAM [vram_offset, vram_offset + bytes). Refuses
    // (ERROR_INVALID_PARAMETER) any byte beyond visible VRAM, a zero or
    // non-ULONG length; ERROR_NOT_ENOUGH_MEMORY when the budget, the mapping
    // table or VideoPort is exhausted. token identifies the mapping for unmap.
    VP_STATUS map(uint64_t vram_offset, uint64_t bytes, PUCHAR& cpu, void*& token) noexcept;
    // amdgpu_bo_kmap for a direct allocator BO: refuses NO_CPU_ACCESS BOs
    // (Linux -EPERM), non-contiguous block lists and any byte of invisible VRAM.
    VP_STATUS map_allocation(const hardware::VramAllocator& allocator,
                             hardware::VramAllocationHandle handle, PUCHAR& cpu,
                             void*& token) noexcept;
    // False for an unknown token or outside PASSIVE_LEVEL (the view stays valid).
    bool unmap(void* token) noexcept;
    // Unmaps every remaining view; PASSIVE_LEVEL, after all CPU users stopped.
    void release() noexcept;

    // Callbacks over this object for hardware::VramBackend cpu_visible pools.
    hardware::VramCpuMapper mapper() noexcept { return {this, map_callback, unmap_callback}; }

private:
    struct Mapping final {
        PUCHAR cpu = nullptr;
        uint64_t vram_offset = 0;
        uint64_t bytes = 0;
        bool used = false;
    };

    static bool map_callback(void* context, uint64_t vram_offset, uint64_t bytes, uint8_t*& cpu,
                             void*& mapping) noexcept;
    static bool unmap_callback(void* context, void* mapping) noexcept;

    Mapping mappings_[vram_aperture_mapping_capacity]{};
    const HardwareState* hardware_ = nullptr;
    PVOID extension_ = nullptr;
    uint64_t visible_bytes_ = 0;
    uint64_t budget_bytes_ = 0;
    uint64_t mapped_bytes_ = 0;
};

} // namespace rx7900::reactos

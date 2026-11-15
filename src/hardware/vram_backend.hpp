/*
Module: rx7900.hardware.vram_backend
Purpose: Back memory::Manager local-VRAM allocations with VramAllocator pools.
Created: 2026-10-22
Architecture: Adapter between the shared Manager Backend contract and the whole-VRAM
allocator; CPU access comes from a caller-supplied mapper (native vram_window).

Why one Manager per placement class: memory::Manager chooses each GPU address
itself (first fit inside one contiguous arena) and only then asks the Backend to
back exactly that span; the Backend cannot relocate it. Its consumers (PSP, SMU,
GART table, rings, MQDs) use that address directly as an MC address with no GPUVM
translation, so it must be physically contiguous VRAM. One Manager over all of
VRAM would therefore place objects over reservations and over other allocator
users, could not express TOPDOWN or the visible-window limit, and would put
CPU-less objects into scarce visible VRAM. Instead each VramBackend carves one
contiguous kernel BO from the allocator with the Linux placement for its class:
a cpu_visible pool is CPU_ACCESS_REQUIRED (bottom-up inside visible VRAM), a
gpu_only pool is NO_CPU_ACCESS (TOPDOWN over all VRAM). Its Manager sub-allocates
that arena; scattered (non-contiguous) VRAM stays available to direct
VramAllocator users and is never expressed through a Manager.
*/
#pragma once

#include "vram_allocator.hpp"

namespace rx7900::hardware {

// CPU access to visible VRAM. map establishes an uncached CPU view of the VRAM
// byte range [vram_offset, vram_offset + bytes), which the caller guarantees is
// inside visible VRAM, and returns the view and an opaque mapping token. unmap
// returns false when release is uncertain; the token must stay valid to retry.
struct VramCpuMapper final {
    void* context;
    bool (*map)(void*, uint64_t vram_offset, uint64_t bytes, uint8_t*& cpu, void*& mapping) noexcept;
    bool (*unmap)(void*, void* mapping) noexcept;
};

enum class VramPool {
    cpu_visible,
    gpu_only
};

// Device-owned and trivially destructible; must outlive its Manager. Calls are
// serialized with the Manager (<= APC_LEVEL; mapper callbacks may need
// PASSIVE_LEVEL, as the native mapper does). Order: initialize, then
// Manager::initialize(arena(), backend()); at teardown Manager::shutdown, then
// shutdown here, then the allocator.
class VramBackend final {
public:
    VramBackend() noexcept = default;
    VramBackend(const VramBackend&) = delete;
    VramBackend& operator=(const VramBackend&) = delete;

    // Carves arena_bytes (page multiple) at arena_alignment (0 or a power of
    // two) as a pinned contiguous kernel BO. mc_base is VramLocation.mc_start:
    // arena().virtual_address = mc_base + VRAM offset. cpu_visible pools need
    // both mapper callbacks; gpu_only pools ignore the mapper. Fails with the
    // allocator status (no_space, no_metadata, ...) and retains nothing.
    VramStatus initialize(VramAllocator& allocator, uint64_t mc_base, VramPool pool,
                          uint64_t arena_bytes, uint64_t arena_alignment,
                          VramCpuMapper mapper) noexcept;
    // Releases the carve-out; busy while any Manager backing is still live.
    VramStatus shutdown() noexcept;

    // MC-address arena for Manager::initialize and the Backend callbacks.
    sdma::GpuAllocation arena() const noexcept { return {mc_base_ + arena_offset_, arena_bytes_}; }
    memory::Backend backend() noexcept { return {this, create, destroy}; }
    uint64_t arena_vram_offset() const noexcept { return arena_offset_; }
    VramAllocationHandle carve_out() const noexcept { return carve_; }
    bool initialized() const noexcept { return initialized_; }

private:
    struct Record final {
        sdma::GpuAllocation gpu{};
        void* mapping = nullptr;
        bool used = false;
    };

    // memory::Backend contract. create refuses (no_resources, complete rollback)
    // a span outside the arena, a CPU view of a gpu_only pool or a failed map.
    // Access is not enforced: local VRAM has no per-object permission without
    // GPUVM, and Manager itself checks leases against the granted Access.
    static memory::CreateResult create(void* context, sdma::GpuAllocation gpu, memory::Access access,
                                       bool cpu_visible, memory::Backing& backing) noexcept;
    // Unmaps the CPU view (false keeps the record for a retry) and frees the
    // record; the VRAM itself stays in the carve-out.
    static bool destroy(void* context, const memory::Backing& backing,
                        sdma::GpuAllocation gpu) noexcept;

    Record records_[memory::allocation_capacity]{};
    VramAllocator* allocator_ = nullptr;
    VramCpuMapper mapper_{};
    VramAllocationHandle carve_{};
    uint64_t mc_base_ = 0;
    uint64_t arena_offset_ = 0;
    uint64_t arena_bytes_ = 0;
    VramPool pool_ = VramPool::gpu_only;
    bool initialized_ = false;
};

} // namespace rx7900::hardware

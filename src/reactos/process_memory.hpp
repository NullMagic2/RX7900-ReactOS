/*
Module: rx7900.reactos.process_memory
Purpose: Own process-authorized VRAM/GTT buffers and their GPUVM mappings.
Created: 2026-10-22
Architecture: Native process sessions use real Manager backing and UserQueues
VM operations; no address or backing token supplied by user mode is trusted.
*/
#pragma once

#include "../hardware/user_queue.hpp"

namespace rx7900::reactos {

constexpr uint32_t process_memory_capacity = 16;
// One buffer holds a 4096x4096 X24_8_32 depth-copy target; a session's live
// backing (VRAM buffers count their GTT backup too) stays within the budget.
// The GTT Manager and the session's GPUVM page-table pool bound it further.
constexpr uint64_t process_memory_max_bytes = 128ULL << 20;
constexpr uint64_t process_memory_session_bytes = 384ULL << 20;
constexpr uint32_t process_memory_copy_bytes = 4096;

enum class ProcessMemoryDomain : uint32_t { system = 0, vram = 1 };

struct ProcessMemoryConfig final {
    hardware::UserQueues* queues;
    uint32_t pasid;
    memory::Manager* vram;
    memory::Manager* gtt;
    void* context;
    // Resolves only an allocation owned by gtt to its actual contiguous DMA
    // bus extent, obtained from the native common-buffer adapter. Never a PFN
    // guessed from a CPU pointer or an address supplied through an IOCTL.
    bool (*dma_bus)(void*, memory::Handle, uint64_t&) noexcept;
};

struct ProcessBufferInfo final {
    uint64_t bytes = 0;
    uint64_t va = 0;
    memory::Access access = memory::Access::read;
    memory::Access mapped_access = memory::Access::read;
    ProcessMemoryDomain domain = ProcessMemoryDomain::system;
    uint32_t leases = 0;
    bool mapped = false;
    bool quarantined = false;
};

// One object per native process session. Externally serialized PASSIVE_LEVEL
// use; config services outlive every buffer. Object construction never owns OS
// memory. Handles are opaque, unique within this session and never recycle.
// The session authenticates the actual NT process before calling any method.
// User-copy buffers are already captured by the caller and remain owned there.
class ProcessMemory final {
public:
    ProcessMemory() noexcept = default;
    ProcessMemory(const ProcessMemory&) = delete;
    ProcessMemory& operator=(const ProcessMemory&) = delete;

    memory::Status initialize(ProcessMemoryConfig config) noexcept;
    memory::Status allocate(uint64_t bytes, ProcessMemoryDomain domain,
                            memory::Access access, uint64_t& handle) noexcept;
    memory::Status describe(uint64_t handle, ProcessBufferInfo& output) const noexcept;
    // Whole-buffer mappings only, permissions cannot exceed allocation grants.
    // Executable user mappings are deliberately unavailable. UserQueues also
    // rejects overlap with its private queue and context mappings.
    memory::Status map(uint64_t handle, uint64_t va, memory::Access access) noexcept;
    memory::Status unmap(uint64_t handle) noexcept;
    memory::Status release(uint64_t handle) noexcept;
    memory::Status read(uint64_t handle, uint64_t offset, uint8_t* destination,
                        uint32_t bytes) noexcept;
    memory::Status write(uint64_t handle, uint64_t offset, ByteView source) noexcept;
    // A validated command owner leases every referenced buffer before submit;
    // only actual fence completion or confirmed queue removal returns leases.
    // CPU access and mapping changes are refused while any such lease exists.
    memory::Status retain(uint64_t handle, memory::Access access) noexcept;
    memory::Status unretain(uint64_t handle) noexcept;

    // Caller proves all process queues suspended before saving; neither path
    // invents GPU-idle proof. VRAM contents have a retained GTT backup allocated
    // together with the buffer, so recovery never depends on a late allocation.
    memory::Status save_for_recovery() noexcept;
    memory::Status restore_after_recovery() noexcept;
    // Confirmed destructive (mode-1) reset retired the old VM and every engine
    // access; call inside the reset window, after every command lease returned
    // (busy otherwise, nothing changed). GTT backing survives the reset (Linux
    // amdgpu_ttm_recover_gart) and keeps its GPUVA record for the rebind. Local
    // VRAM contents are lost (amdgpu_device_check_vram_lost: a MODE1 reset always
    // loses VRAM) unless save_for_recovery captured them while proven idle (the
    // power path, like Linux eviction before suspend). A lost buffer, and one
    // whose backing could not be described, releases its backing and backup
    // now and leaves a tombstone: every method naming it fails (stale_handle)
    // and records it for take_lost_touch, and only retire_lost frees its handle,
    // so the owner answers it lost exactly once. quarantined: a release failed
    // and that slot keeps its allocations. CPU access stays refused until rebind.
    memory::Status retire_after_reset() noexcept;
    // Confirmed destructive reset has retired the old VM. The caller supplies
    // a newly created process/PASID over the same Manager and DMA owners; this
    // replays only previously authorized GPUVAs at their recorded addresses
    // (TLB flush after each), then restores contents saved by the power path.
    // Retry-safe: a failure changes no record; the caller destroys the new VM
    // and calls abandon_rebind before any retry.
    memory::Status rebind_after_reset(ProcessMemoryConfig config) noexcept;
    // After a failed rebind_after_reset whose new VM the caller destroyed
    // (destroy_process returned destroyed): forget the partial mappings.
    void abandon_rebind() noexcept;
    bool ready_for_reset_rebind() const noexcept;
    // First tombstone handle any method met since the previous call, or 0; the
    // owner turns it into its single lost answer and then calls retire_lost.
    uint64_t take_lost_touch() noexcept;
    // Retires a tombstone handle (never recycled). false: handle is not lost.
    bool retire_lost(uint64_t handle) noexcept;
    // Tombstones not yet answered.
    uint32_t lost_buffers() const noexcept;
    // Process exit, after every queue of the process was removed through MES
    // and before destroy_process: unmap every client GPUVA, then one
    // heavyweight TLB flush, before any backing may be released. busy with
    // leases; quarantined on a failed unmap/flush (affected slots keep backing).
    memory::Status unmap_all() noexcept;
    // Call only after destroy_process returned destroyed, or confirmed reset
    // retired the VM. This is the sole release path for uncertain mappings.
    memory::Status close_after_idle() noexcept;
    bool empty() const noexcept;
    // RAS page retirement: true when a live VRAM buffer's backing overlaps the
    // VMID0 MC range [mc_address, mc_address + bytes). Read-only.
    bool owns_vram(uint64_t mc_address, uint64_t bytes) const noexcept;

private:
    struct Slot final {
        uint64_t handle = 0;
        ProcessBufferInfo info{};
        memory::Handle backing{};
        memory::Handle backup{};
        bool backing_live = false;
        bool backing_retained = false;
        bool backup_live = false;
        bool backup_retained = false;
        // Tombstone of a buffer whose contents a confirmed reset destroyed.
        bool lost = false;
        // GPUVA record kept across a confirmed reset until the rebind maps it.
        bool reset_mapped = false;
        uint64_t reset_va = 0;
        memory::Access reset_access = memory::Access::read;
    };
    Slot* find(uint64_t handle) noexcept;
    const Slot* find(uint64_t handle) const noexcept;
    memory::Manager& manager(const Slot& slot) const noexcept;
    bool backing(Slot& slot, memory::AllocationInfo& output) noexcept;
    memory::Status map_slot(Slot& slot, uint64_t va, memory::Access access, bool rebinding) noexcept;
    memory::Status release_slot(Slot& slot) noexcept;
    // Bytes of live backing and backup allocations (process_memory_session_bytes).
    uint64_t committed_bytes() const noexcept;
    bool uncertain() const noexcept;

    ProcessMemoryConfig config_{};
    Slot slots_[process_memory_capacity]{};
    // Caller-owned DMA page list, read synchronously by GpuVm::map_system.
    uint8_t pages_[process_memory_max_bytes / memory::page_bytes * sizeof(uint64_t)]{};
    uint64_t generation_ = 0;
    // Written by const lookups: the first tombstone a request named.
    mutable uint64_t lost_touch_ = 0;
    bool initialized_ = false;
    bool suspended_ = false;
    // save_for_recovery copied every VRAM buffer to its backup while idle.
    bool saved_ = false;
};

} // namespace rx7900::reactos

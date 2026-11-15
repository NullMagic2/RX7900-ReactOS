/*
Module: rx7900.memory.manager
Purpose: Own bounded GPUVA reservations, backing resources and hardware leases.
Created: 2026-10-21
Architecture: Device-owned fixed-capacity manager; native callbacks own GPUVM/OS work.
*/
#pragma once

#include "sdma.hpp"

namespace rx7900 {
namespace memory {

constexpr size_t allocation_capacity = 64;
constexpr uint64_t page_bytes = 4096;

enum class Status {
    success,
    unavailable,
    invalid_argument,
    stale_handle,
    access_denied,
    out_of_memory,
    busy,
    deferred,
    backend_failure,
    quarantined,
    exhausted
};

enum class Access : uint32_t {
    read = 1,
    write = 2,
    read_write = 3
};

struct Handle final {
    uint32_t slot = UINT32_MAX;
    uint64_t generation = 0;
};

struct Slice final {
    Handle allocation;
    uint64_t byte_offset;
};

// A native opaque resource token owns its allocation, pin and mapping objects.
// CPU storage is optional; its validity lasts until successful destroy. This
// descriptor never crosses the user/kernel ABI or supplies a physical address.
struct Backing final {
    void* token;
    uint8_t* cpu_bytes;
    size_t cpu_byte_count;
};

enum class CreateResult {
    created,
    no_resources,
    quarantined
};

struct Backend final {
    void* context;
    // created means real backing exists, permissions match, PTEs are visible
    // and TLB invalidation completed. no_resources guarantees complete rollback
    // with an empty Backing. An uncertain map/unmap/ACK MUST return quarantined
    // with its full resource token: Manager reserves the GPUVA and ownership,
    // publishes no Handle and requires later confirmed destroy before reuse.
    CreateResult (*create)(void*, sdma::GpuAllocation, Access, bool, Backing&) noexcept;
    // Called only without leases. True confirms GPUVM unmap/TLB invalidation
    // and physical resource destruction. False retains the entire descriptor;
    // it must remain safe to retry, and the manager quarantines this allocation.
    bool (*destroy)(void*, const Backing&, sdma::GpuAllocation) noexcept;
};

struct AllocationInfo final {
    sdma::GpuAllocation gpu;
    Backing backing;
    Access access;
    uint32_t leases;
    bool retiring;
};

// Calls are externally serialized at <= APC_LEVEL; backend callbacks may block.
// A DPC must not call this manager: queue completion to an appropriate worker.
// The device owner must call shutdown after confirmed DMA quiescence; automatic
// destruction cannot safely infer whether a GPU still holds pinned resources.
// Manager/backend/context outlive all handles, leases and submitted GPU work.
class Manager final {
public:
    Manager() noexcept = default;
    Manager(const Manager&) = delete;
    Manager& operator=(const Manager&) = delete;

    Status initialize(sdma::GpuAllocation arena, Backend backend) noexcept;
    Status shutdown() noexcept;

    Status allocate(uint64_t byte_length, uint64_t alignment, Access access,
                    bool cpu_visible, Handle& handle) noexcept;
    Status resolve(Slice slice, uint64_t byte_count, Access access,
                   sdma::GpuRange& range) const noexcept;
    Status describe(Handle handle, AllocationInfo& info) const noexcept;
    Status retain(Handle handle, Access access) noexcept;
    Status unretain(Handle handle) noexcept;
    // A retained allocation retires now and is destroyed at its final unretain.
    // No future resolve/retain is allowed after retirement. Failure to destroy
    // quarantines rather than freeing a resource the backend may still access.
    Status release(Handle handle) noexcept;
    bool empty() const noexcept;

private:
    struct Slot final {
        AllocationInfo info{};
        uint64_t generation = 0;
        bool occupied = false;
        bool quarantined = false;
    };

    const Slot* find(Handle handle) const noexcept;
    Slot* find(Handle handle) noexcept;
    Status destroy_slot(Slot& slot) noexcept;
    bool quarantined() const noexcept;

    sdma::GpuAllocation arena_{};
    Backend backend_{};
    Slot slots_[allocation_capacity]{};
    uint64_t generation_ = 0;
    bool initialized_ = false;
};

} // namespace memory
} // namespace rx7900

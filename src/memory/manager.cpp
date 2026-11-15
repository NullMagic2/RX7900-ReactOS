/*
Module: rx7900.memory.manager
Purpose: Check and reserve GPUVA spans and defer destruction until leases end.
Created: 2026-10-21
Architecture: No host allocation, raw-memory access or speculative hardware success.
*/
#include "manager.hpp"

namespace rx7900 {
namespace memory {
namespace {

bool valid_access(Access access) noexcept {
    const uint32_t bits = static_cast<uint32_t>(access);
    return bits != 0 && (bits & ~3u) == 0;
}

bool permits(Access granted, Access requested) noexcept {
    return valid_access(requested) &&
        (static_cast<uint32_t>(granted) & static_cast<uint32_t>(requested)) ==
        static_cast<uint32_t>(requested);
}

bool valid_span(sdma::GpuAllocation span) noexcept {
    return span.byte_length != 0 &&
        span.virtual_address <= sdma::gpu_virtual_address_max &&
        span.byte_length - 1 <= sdma::gpu_virtual_address_max - span.virtual_address;
}

bool power_of_two(uint64_t value) noexcept {
    return value != 0 && (value & (value - 1)) == 0;
}

bool align_up(uint64_t value, uint64_t alignment, uint64_t& result) noexcept {
    const uint64_t padding = (alignment - (value & (alignment - 1))) & (alignment - 1);

    if (value > sdma::gpu_virtual_address_max ||
        padding > sdma::gpu_virtual_address_max - value) {
        return false;
    }

    result = value + padding;
    return true;
}

} // namespace

Status Manager::initialize(sdma::GpuAllocation arena, Backend backend) noexcept {
    if (!empty()) {
        return Status::busy;
    }

    if (!valid_span(arena) || (arena.virtual_address & (page_bytes - 1)) != 0 ||
        (arena.byte_length & (page_bytes - 1)) != 0) {
        return Status::invalid_argument;
    }

    if (backend.create == nullptr || backend.destroy == nullptr) {
        return Status::unavailable;
    }

    arena_ = arena;
    backend_ = backend;
    initialized_ = true;
    return Status::success;
}

bool Manager::empty() const noexcept {
    for (const auto& slot : slots_) {
        if (slot.occupied) {
            return false;
        }
    }

    return true;
}

bool Manager::quarantined() const noexcept {
    for (const auto& slot : slots_) {
        if (slot.occupied && slot.quarantined) {
            return true;
        }
    }

    return false;
}

const Manager::Slot* Manager::find(Handle handle) const noexcept {
    if (handle.slot >= allocation_capacity || handle.generation == 0) {
        return nullptr;
    }

    const auto& slot = slots_[handle.slot];
    return slot.occupied && slot.generation == handle.generation ? &slot : nullptr;
}

Manager::Slot* Manager::find(Handle handle) noexcept {
    if (handle.slot >= allocation_capacity || handle.generation == 0) {
        return nullptr;
    }

    auto& slot = slots_[handle.slot];
    return slot.occupied && slot.generation == handle.generation ? &slot : nullptr;
}

Status Manager::allocate(uint64_t byte_length, uint64_t alignment, Access access,
                         bool cpu_visible, Handle& handle) noexcept {
    handle = {};

    if (!initialized_) {
        return Status::unavailable;
    }

    if (quarantined()) {
        return Status::quarantined;
    }

    if (byte_length == 0 || (byte_length & (page_bytes - 1)) != 0 ||
        alignment < page_bytes || !power_of_two(alignment) || !valid_access(access) ||
        (cpu_visible && byte_length > SIZE_MAX)) {
        return Status::invalid_argument;
    }

    if (generation_ == UINT64_MAX) {
        return Status::exhausted;
    }

    size_t free_slot = allocation_capacity;

    for (size_t index = 0; index < allocation_capacity; ++index) {
        if (!slots_[index].occupied) {
            free_slot = index;
            break;
        }
    }

    if (free_slot == allocation_capacity) {
        return Status::out_of_memory;
    }

    uint64_t candidate = arena_.virtual_address;
    bool placed = false;

    // Advancing beyond one occupied allocation per pass bounds first-fit work
    // by the fixed slot capacity; no user-controlled unbounded search occurs.
    for (size_t pass = 0; pass <= allocation_capacity; ++pass) {
        if (!align_up(candidate, alignment, candidate) ||
            candidate < arena_.virtual_address ||
            candidate - arena_.virtual_address > arena_.byte_length ||
            byte_length > arena_.byte_length - (candidate - arena_.virtual_address)) {
            return Status::out_of_memory;
        }

        bool collision = false;

        for (const auto& slot : slots_) {
            if (!slot.occupied) {
                continue;
            }

            const auto span = slot.info.gpu;
            const bool overlap = candidate <= span.virtual_address
                ? span.virtual_address - candidate < byte_length
                : candidate - span.virtual_address < span.byte_length;

            if (overlap) {
                // A valid low-48-bit span has a representable one-past end.
                candidate = span.virtual_address + span.byte_length;
                collision = true;
                break;
            }
        }

        if (!collision) {
            placed = true;
            break;
        }
    }

    if (!placed) {
        return Status::out_of_memory;
    }

    auto& slot = slots_[free_slot];
    const sdma::GpuAllocation gpu{candidate, byte_length};
    Backing backing{};

    const auto creation = backend_.create(backend_.context, gpu, access, cpu_visible, backing);

    if (creation == CreateResult::no_resources && backing.token == nullptr &&
        backing.cpu_bytes == nullptr && backing.cpu_byte_count == 0) {
        return Status::backend_failure;
    }

    slot.info = {gpu, backing, access, 0, false};
    slot.generation = ++generation_;
    slot.occupied = true;
    slot.quarantined = false;

    if (creation != CreateResult::created) {
        slot.info.retiring = true;
        slot.quarantined = true;
        return Status::quarantined;
    }

    if (backing.token == nullptr || (cpu_visible &&
        (backing.cpu_bytes == nullptr || backing.cpu_byte_count < byte_length))) {
        // A broken native descriptor still owns a possible live GPU mapping.
        // Retain it until destroy confirms cleanup instead of losing the token.
        slot.info.retiring = true;
        const auto cleanup = destroy_slot(slot);
        return cleanup == Status::success ? Status::backend_failure : cleanup;
    }

    handle = {static_cast<uint32_t>(free_slot), slot.generation};
    return Status::success;
}

Status Manager::describe(Handle handle, AllocationInfo& info) const noexcept {
    info = {};
    const Slot* slot = find(handle);

    if (slot == nullptr) {
        return Status::stale_handle;
    }

    if (slot->quarantined) {
        return Status::quarantined;
    }

    info = slot->info;
    return Status::success;
}

Status Manager::resolve(Slice slice, uint64_t byte_count, Access access,
                        sdma::GpuRange& range) const noexcept {
    range = {};
    const Slot* slot = find(slice.allocation);

    if (slot == nullptr) {
        return Status::stale_handle;
    }

    if (slot->quarantined) {
        return Status::quarantined;
    }

    if (slot->info.retiring) {
        return Status::busy;
    }

    if (!permits(slot->info.access, access)) {
        return Status::access_denied;
    }

    if (byte_count == 0 || slice.byte_offset > slot->info.gpu.byte_length ||
        byte_count > slot->info.gpu.byte_length - slice.byte_offset) {
        return Status::invalid_argument;
    }

    range = {slot->info.gpu, slice.byte_offset};
    return Status::success;
}

Status Manager::retain(Handle handle, Access access) noexcept {
    Slot* slot = find(handle);

    if (slot == nullptr) {
        return Status::stale_handle;
    }

    if (slot->quarantined) {
        return Status::quarantined;
    }

    if (slot->info.retiring) {
        return Status::busy;
    }

    if (!permits(slot->info.access, access)) {
        return Status::access_denied;
    }

    if (slot->info.leases == UINT32_MAX) {
        return Status::exhausted;
    }

    ++slot->info.leases;
    return Status::success;
}

Status Manager::destroy_slot(Slot& slot) noexcept {
    if (!backend_.destroy(backend_.context, slot.info.backing, slot.info.gpu)) {
        slot.quarantined = true;
        return Status::quarantined;
    }

    slot = {};
    return Status::success;
}

Status Manager::unretain(Handle handle) noexcept {
    Slot* slot = find(handle);

    if (slot == nullptr) {
        return Status::stale_handle;
    }

    if (slot->info.leases == 0) {
        return Status::invalid_argument;
    }

    --slot->info.leases;
    return slot->info.leases == 0 && slot->info.retiring
        ? destroy_slot(*slot) : Status::success;
}

Status Manager::release(Handle handle) noexcept {
    Slot* slot = find(handle);

    if (slot == nullptr) {
        return Status::stale_handle;
    }

    slot->info.retiring = true;
    return slot->info.leases != 0 ? Status::deferred : destroy_slot(*slot);
}

Status Manager::shutdown() noexcept {
    // Do not partially shut down if any hardware lease remains.
    for (const auto& slot : slots_) {
        if (slot.occupied && slot.info.leases != 0) {
            return Status::busy;
        }
    }

    Status status = Status::success;

    for (auto& slot : slots_) {
        if (slot.occupied && destroy_slot(slot) != Status::success) {
            status = Status::quarantined;
        }
    }

    if (status == Status::success) {
        initialized_ = false;
        backend_ = {};
        arena_ = {};
    }

    return status;
}

} // namespace memory
} // namespace rx7900

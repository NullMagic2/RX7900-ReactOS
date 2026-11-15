/*
Module: rx7900.acceleration.ring
Purpose: Encode, commit, track and safely quiesce a real SDMA queue.
Created: 2026-10-21
Architecture: Uses shared encoder and memory leases; HW actions are native callbacks.
Adaptation: Linux v6.12 sdma_v6_0.c uses byte RPTR/WPTR, 16-dword ring alignment,
GCR cache synchronization and fence-before-TRAP completion; see AMDGPU-SDMA.txt.
Copyright 2020 Advanced Micro Devices, Inc.
*/
#include "ring.hpp"
#include "byte_view.hpp"

namespace rx7900 {
namespace sdma {
namespace {

bool same_handle(memory::Handle first, memory::Handle second) noexcept {
    return first.slot == second.slot && first.generation == second.generation;
}

bool valid_snapshot(const HardwareSnapshot& snapshot, uint32_t instance) noexcept {
    return snapshot.ready && !snapshot.faulted && snapshot.ip_version == 0x060000 &&
           snapshot.vmid == 0 && snapshot.instance == instance;
}

} // namespace

RingStatus Ring::initialize(memory::Manager& manager, RingConfiguration configuration,
                            RingBackend backend) noexcept {
    if (active_) {
        return RingStatus::busy;
    }

    if (generation_ == UINT64_MAX) {
        return RingStatus::sequence_exhausted;
    }

    if (backend.query == nullptr || backend.publish == nullptr || backend.quiesce == nullptr) {
        return RingStatus::unavailable;
    }

    const uint32_t capacity = configuration.ring_byte_count;

    if (capacity < 256 || (capacity & (capacity - 1)) != 0 ||
        configuration.scratch_bytes == nullptr || configuration.scratch_byte_count < capacity ||
        configuration.timeout_nanoseconds == 0 || configuration.instance > 1 ||
        same_handle(configuration.ring_allocation, configuration.fence_allocation)) {
        return RingStatus::invalid_argument;
    }

    memory::AllocationInfo ring{};
    memory::AllocationInfo fence{};

    if (manager.describe(configuration.ring_allocation, ring) != memory::Status::success ||
        manager.describe(configuration.fence_allocation, fence) != memory::Status::success ||
        ring.retiring || fence.retiring || ring.backing.cpu_bytes == nullptr ||
        ring.backing.cpu_byte_count < capacity || ring.gpu.byte_length < capacity ||
        (ring.gpu.virtual_address & 255) != 0 || fence.gpu.byte_length < 4 ||
        fence.backing.cpu_bytes == nullptr || fence.backing.cpu_byte_count < 4) {
        return RingStatus::invalid_memory;
    }

    HardwareSnapshot snapshot{};

    if (!backend.query(backend.context, snapshot) ||
        !valid_snapshot(snapshot, configuration.instance) ||
        snapshot.read_pointer_bytes != 0 || snapshot.write_pointer_bytes != 0 ||
        snapshot.completed_sequence != 0) {
        return RingStatus::unavailable;
    }

    if (manager.retain(configuration.ring_allocation, memory::Access::read_write) !=
        memory::Status::success) {
        return RingStatus::invalid_memory;
    }

    if (manager.retain(configuration.fence_allocation, memory::Access::read_write) !=
        memory::Status::success) {
        manager.unretain(configuration.ring_allocation);
        return RingStatus::invalid_memory;
    }

    manager_ = &manager;
    configuration_ = configuration;
    backend_ = backend;
    ring_ = ring;
    fence_ = fence;
    pending_begin_ = 0;
    pending_count_ = 0;
    write_pointer_ = 0;
    read_pointer_ = 0;
    last_time_ = 0;
    submitted_sequence_ = 0;
    completed_sequence_ = 0;
    ++generation_;
    active_ = true;
    faulted_ = false;
    return RingStatus::success;
}

bool Ring::reserved(memory::Handle handle) const noexcept {
    return same_handle(handle, configuration_.ring_allocation) ||
           same_handle(handle, configuration_.fence_allocation);
}

bool Ring::release_pending(Pending& pending) noexcept {
    bool released = true;

    if (pending.copy && manager_->unretain(pending.source) != memory::Status::success) {
        released = false;
    }

    if (manager_->unretain(pending.destination) != memory::Status::success) {
        released = false;
    }

    pending = {};
    return released;
}

RingStatus Ring::poll(uint64_t now_nanoseconds) noexcept {
    if (!active_) {
        return RingStatus::unavailable;
    }

    if (faulted_) {
        return RingStatus::hardware_fault;
    }

    if (now_nanoseconds < last_time_) {
        return RingStatus::invalid_argument;
    }

    last_time_ = now_nanoseconds;
    HardwareSnapshot snapshot{};

    if (!backend_.query(backend_.context, snapshot) ||
        !valid_snapshot(snapshot, configuration_.instance) ||
        (snapshot.read_pointer_bytes & 3) != 0 ||
        snapshot.read_pointer_bytes < read_pointer_ ||
        snapshot.read_pointer_bytes > write_pointer_ ||
        snapshot.write_pointer_bytes != write_pointer_ ||
        snapshot.completed_sequence < completed_sequence_ ||
        snapshot.completed_sequence > submitted_sequence_) {
        faulted_ = true;
        return RingStatus::hardware_fault;
    }

    read_pointer_ = snapshot.read_pointer_bytes;
    completed_sequence_ = snapshot.completed_sequence;

    while (pending_count_ != 0) {
        auto& pending = pending_[pending_begin_];

        if (pending.sequence > completed_sequence_) {
            if (now_nanoseconds >= pending.deadline) {
                faulted_ = true;
                return RingStatus::timeout;
            }

            break;
        }

        const bool released = release_pending(pending);
        pending_begin_ = (pending_begin_ + 1) % pending_capacity;
        --pending_count_;

        if (!released) {
            faulted_ = true;
            return RingStatus::hardware_fault;
        }
    }

    return pending_count_ == 0 ? RingStatus::success : RingStatus::pending;
}

RingStatus Ring::submit_copy(memory::Slice source, memory::Slice destination,
                             uint64_t byte_count, uint64_t now_nanoseconds,
                             Ticket& ticket) noexcept {
    return submit(source, destination, byte_count, 0, true, now_nanoseconds, ticket);
}

RingStatus Ring::submit_fill(memory::Slice destination, uint64_t byte_count,
                             uint32_t value, uint64_t now_nanoseconds,
                             Ticket& ticket) noexcept {
    return submit({}, destination, byte_count, value, false, now_nanoseconds, ticket);
}

RingStatus Ring::submit(memory::Slice source, memory::Slice destination,
                        uint64_t count, uint32_t value, bool copy, uint64_t now,
                        Ticket& ticket) noexcept {
    ticket = {};
    const auto status = poll(now);

    if (status != RingStatus::success && status != RingStatus::pending) {
        return status;
    }

    if (pending_count_ == pending_capacity) {
        return RingStatus::busy;
    }

    if (submitted_sequence_ == UINT32_MAX) {
        return RingStatus::sequence_exhausted;
    }

    if (configuration_.timeout_nanoseconds > UINT64_MAX - now) {
        return RingStatus::invalid_argument;
    }

    if (reserved(destination.allocation) || (copy && reserved(source.allocation))) {
        return RingStatus::invalid_memory;
    }

    GpuRange source_range{};
    GpuRange destination_range{};

    if ((copy && manager_->resolve(source, count, memory::Access::read, source_range) !=
        memory::Status::success) ||
        manager_->resolve(destination, count, memory::Access::write, destination_range) !=
        memory::Status::success) {
        return RingStatus::invalid_memory;
    }

    const CopyRequest copy_request{source_range, destination_range, count};
    const FillRequest fill_request{destination_range, count, value};
    EncodeResult transfer_result{};
    const auto encode_status = copy
        ? encode_copy(copy_request, {nullptr, 0}, transfer_result)
        : encode_fill(fill_request, {nullptr, 0}, transfer_result);

    if (encode_status != EncodeStatus::output_too_small) {
        return RingStatus::encoder_failure;
    }

    const size_t fence_bytes = fence32_packet_bytes +
                               (configuration_.interrupts ? trap_packet_bytes : 0);

    if (transfer_result.bytes_required > SIZE_MAX - cache_sync_packet_bytes -
        fence_bytes - (ring_alignment_bytes - 1)) {
        return RingStatus::invalid_argument;
    }

    const size_t unpadded = transfer_result.bytes_required + cache_sync_packet_bytes + fence_bytes;
    const size_t total = (unpadded + ring_alignment_bytes - 1) & ~(ring_alignment_bytes - 1);
    const uint64_t capacity = configuration_.ring_byte_count;
    const uint64_t used = write_pointer_ - read_pointer_;

    // Preserve one complete alignment unit as a gap; RPTR and WPTR must never
    // represent an ambiguous full ring. uint64 counters are never wrapped here.
    if (used > capacity - ring_alignment_bytes || total > capacity - ring_alignment_bytes - used) {
        return RingStatus::busy;
    }

    if (total > UINT64_MAX - write_pointer_) {
        return RingStatus::sequence_exhausted;
    }

    CommandBuffer scratch(configuration_.scratch_bytes, configuration_.scratch_byte_count);
    EncodeResult result{};
    const auto transfer_buffer = scratch.subbuffer(0, transfer_result.bytes_required);
    const auto transfer_status = copy ? encode_copy(copy_request, transfer_buffer, result)
                                      : encode_fill(fill_request, transfer_buffer, result);

    if (transfer_status != EncodeStatus::success ||
        encode_cache_sync(scratch.subbuffer(transfer_result.bytes_required,
                          cache_sync_packet_bytes), result) != EncodeStatus::success) {
        return RingStatus::encoder_failure;
    }

    const uint32_t sequence = submitted_sequence_ + 1;
    const FenceRequest fence_request{{fence_.gpu, 0}, sequence,
                                     FenceWidth::bits32, configuration_.interrupts};

    if (encode_fence(fence_request,
        scratch.subbuffer(transfer_result.bytes_required + cache_sync_packet_bytes,
                          fence_bytes), result) != EncodeStatus::success) {
        return RingStatus::encoder_failure;
    }

    for (size_t offset = unpadded; offset < total; offset += sizeof(uint32_t)) {
        scratch.write_u32(offset, 0);
    }

    if (copy && manager_->retain(source.allocation, memory::Access::read) !=
        memory::Status::success) {
        return RingStatus::invalid_memory;
    }

    if (manager_->retain(destination.allocation, memory::Access::write) != memory::Status::success) {
        if (copy) {
            manager_->unretain(source.allocation);
        }
        return RingStatus::invalid_memory;
    }

    const rx7900::ByteView commands(configuration_.scratch_bytes, total);
    CommandBuffer ring_bytes(ring_.backing.cpu_bytes, configuration_.ring_byte_count);

    for (size_t offset = 0; offset < total; offset += sizeof(uint32_t)) {
        uint32_t word = 0;
        commands.read_u32(offset, word);
        const size_t ring_offset = static_cast<size_t>((write_pointer_ + offset) & (capacity - 1));
        ring_bytes.write_u32(ring_offset, word);
    }

    const size_t tail = (pending_begin_ + pending_count_) % pending_capacity;
    auto& pending = pending_[tail];
    pending = {sequence, now + configuration_.timeout_nanoseconds,
               source.allocation, destination.allocation, copy};
    ++pending_count_;
    const uint64_t previous_pointer = write_pointer_;
    write_pointer_ += total;
    submitted_sequence_ = sequence;
    const auto publication = backend_.publish(backend_.context, write_pointer_);

    if (publication == PublishResult::not_published) {
        const bool released = release_pending(pending);
        --pending_count_;
        --submitted_sequence_;
        write_pointer_ = previous_pointer;

        if (!released) {
            faulted_ = true;
            return RingStatus::hardware_fault;
        }

        return RingStatus::unavailable;
    }

    ticket = {generation_, sequence};

    if (publication != PublishResult::published) {
        faulted_ = true;
        return RingStatus::hardware_fault;
    }

    return RingStatus::submitted;
}

RingStatus Ring::ticket_status(Ticket ticket) const noexcept {
    if (ticket.generation == 0 || ticket.sequence == 0) {
        return RingStatus::invalid_argument;
    }

    if (ticket.generation != generation_ || !active_) {
        return RingStatus::invalidated;
    }

    if (ticket.sequence > submitted_sequence_) {
        return RingStatus::invalid_argument;
    }

    if (ticket.sequence <= completed_sequence_) {
        return RingStatus::completed;
    }

    return faulted_ ? RingStatus::hardware_fault : RingStatus::pending;
}

void Ring::notify_fault() noexcept {
    if (active_) {
        faulted_ = true;
    }
}

RingStatus Ring::stop() noexcept {
    if (!active_) {
        return RingStatus::unavailable;
    }

    if (!backend_.quiesce(backend_.context)) {
        faulted_ = true;
        return RingStatus::hardware_fault;
    }

    bool released = true;

    while (pending_count_ != 0) {
        if (!release_pending(pending_[pending_begin_])) {
            released = false;
        }
        pending_begin_ = (pending_begin_ + 1) % pending_capacity;
        --pending_count_;
    }

    if (manager_->unretain(configuration_.ring_allocation) != memory::Status::success) {
        released = false;
    }

    if (manager_->unretain(configuration_.fence_allocation) != memory::Status::success) {
        released = false;
    }

    active_ = false;
    faulted_ = false;
    manager_ = nullptr;
    ring_ = {};
    fence_ = {};
    configuration_ = {};
    backend_ = {};
    return released ? RingStatus::stopped : RingStatus::hardware_fault;
}

} // namespace sdma
} // namespace rx7900

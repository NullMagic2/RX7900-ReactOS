/*
Module: lightning.reactos.ring
Purpose: Portable layout, ordering and bounds of the Venus shared request ring and timeline page.
Created: 2026-10-23
Architecture: Pure C++ over one caller-owned, nonpaged, device-registered control region.
NativeTransport owns the memory, registers, waits and policy decisions; this module
owns the wire layout from transport.h, the producer/consumer ordering rules, the
seqlocked timeline reads and the bounded per-device tables. No allocation, no OS calls.
*/
#pragma once

#include <stddef.h>
#include <stdint.h>
#include "byte_view.hpp"
#include "sdma.hpp"
#include "transport.h"

namespace lightning::reactos::ring {

inline constexpr uint32_t entries = static_cast<uint32_t>(QLV_RING_ENTRIES);
inline constexpr uint32_t slot_bytes = static_cast<uint32_t>(QLV_RING_SLOT_BYTES);
inline constexpr size_t control_bytes = static_cast<size_t>(QLV_RING_CONTROL_BYTES);
inline constexpr size_t data_bytes = size_t{entries} * slot_bytes;
inline constexpr uint32_t timeline_slots = static_cast<uint32_t>(QLV_RING_TIMELINE_SLOTS);
inline constexpr size_t context_capacity = static_cast<size_t>(QLV_KERNEL_MAX_CLIENTS);
// A seqlock reader gives up after this many torn snapshots and lets the caller
// fall back to an exchange; the writer is QEMU's main loop, never preempted
// for long while it holds a slot odd.
inline constexpr unsigned seqlock_attempts = 4;
// Local answers may be stale: every refresh_reads-th unchanged answer for the
// same slot (or unsatisfied wait) goes to the host instead. That bounds the
// damage of a lost publication to a constant factor of exchanges, never a hang.
inline constexpr uint32_t refresh_reads = 8;

static_assert(QLV_RING_DESCRIPTORS_OFFSET + QLV_RING_ENTRIES * QLV_RING_DESCRIPTOR_BYTES <= QLV_PAGE_BYTES,
    "descriptors fit the control page");
static_assert(QLV_RING_TIMELINE_OFFSET + QLV_RING_TIMELINE_SLOTS * QLV_RING_TIMELINE_SLOT_BYTES ==
    QLV_RING_CONTROL_BYTES, "the timeline page ends the control region");
static_assert(QLV_RING_POSTED_ERROR_CONTEXT_OFFSET % 8 == 0 && QLV_RING_TIMELINE_SLOT_BYTES % 8 == 0,
    "64-bit words are naturally aligned");
static_assert((entries & (entries - 1)) == 0 && slot_bytes % QLV_PAGE_BYTES == 0,
    "sequence modulo and slot pages are exact");

// a is at or after b in wrapping u32 sequence order.
constexpr bool reached(uint32_t a, uint32_t b) noexcept {
    return static_cast<uint32_t>(a - b) < 0x80000000u;
}

// The registered control region as naturally aligned 32-bit words. The owner
// keeps control_bytes of nonpaged memory alive and registered for the view's
// lifetime; every offset below is a checked layout constant or bounds-checked
// index, and each access is one volatile load or store shared with QEMU.
class ControlView final {
public:
    ControlView() noexcept = default;
    ControlView(volatile uint32_t* words, size_t bytes) noexcept
        : words_(bytes == control_bytes ? words : nullptr) {}

    bool valid() const noexcept { return words_ != nullptr; }
    uint32_t load(size_t offset) const noexcept {
        return offset % 4 == 0 && offset < control_bytes ? words_[offset / 4] : 0;
    }
    void store(size_t offset, uint32_t value) const noexcept {
        if (offset % 4 == 0 && offset < control_bytes) { words_[offset / 4] = value; }
    }

private:
    volatile uint32_t* words_ = nullptr;
};

enum class Health { active, fault, lost, off };

// Barriers is a policy with static acquire(), release() and full(). On x86
// (the only NT5 target) acquire/release are compiler barriers because TSO
// already orders load-load and store-store; full() must also order a store
// before a later load (a locked instruction or mfence).
//
// Ordering contract with QEMU (hw/misc/lightning-venus.c):
//   producer: request bytes, descriptor, release, TAIL, full, read IDLE;
//   consumer: IDLE=1, full, re-read TAIL; so a missed doorbell is impossible.
//   A batch may publish several entries without the IDLE probe and then probe
//   once (doorbell_needed): the probe follows every TAIL store it covers.
//   completion: QEMU writes the response, RESULT_*, then RETIRED, each after
//   smp_mb; the guest reads RETIRED with acquire before RESULT_* and the reply.
template <class Barriers>
class Ring final {
public:
    struct Published final { uint32_t sequence; bool doorbell; };

    // Zero the region and write the guest header. Registration adopts it.
    bool initialize(ControlView control) noexcept {
        if (!control.valid()) { return false; }
        control_ = control;
        tail_ = 0;

        for (size_t offset = 0; offset < control_bytes; offset += 4) { control_.store(offset, 0); }

        control_.store(QLV_RING_MAGIC_OFFSET, static_cast<uint32_t>(QLV_RING_MAGIC));
        control_.store(QLV_RING_VERSION_OFFSET, static_cast<uint32_t>(QLV_RING_VERSION));
        Barriers::full();
        return true;
    }
    void detach() noexcept { control_ = {}; tail_ = 0; }
    bool attached() const noexcept { return control_.valid(); }
    const ControlView& control() const noexcept { return control_; }

    Health health() const noexcept {
        if (!attached()) { return Health::off; }
        const uint32_t state = acquire_load(QLV_RING_STATE_OFFSET);
        return state == QLV_RING_ACTIVE ? Health::active : state == QLV_RING_FAULT ? Health::fault :
            state == QLV_RING_LOST ? Health::lost : Health::off;
    }
    uint32_t tail() const noexcept { return tail_; }
    uint32_t head() const noexcept { return acquire_load(QLV_RING_HEAD_OFFSET); }
    uint32_t retired() const noexcept { return acquire_load(QLV_RING_RETIRED_OFFSET); }
    // Every slot holds a request QEMU has not copied yet.
    bool full() const noexcept { return static_cast<uint32_t>(tail_ - head()) >= entries; }
    bool drained() const noexcept { return retired() == tail_; }
    bool completed(uint32_t sequence) const noexcept { return reached(retired(), sequence); }
    // The slot the next published entry uses; its bytes go there first.
    uint32_t next_slot() const noexcept { return tail_ % entries; }
    // Valid after completed() for the newest entry, which is the caller's.
    uint32_t result_status() const noexcept { return control_.load(QLV_RING_RESULT_STATUS_OFFSET); }
    uint32_t result_bytes() const noexcept { return control_.load(QLV_RING_RESULT_BYTES_OFFSET); }

    // Precondition: attached(), !full(), and the request bytes are in
    // next_slot(). Returns the entry's sequence and whether QEMU's consumer was
    // idle, in which case the caller must write the doorbell once. probe=false
    // skips that check (doorbell is false): the caller owes doorbell_needed()
    // before it waits for this entry or returns to its own caller.
    Published publish(uint32_t flags, uint32_t request_bytes, uint32_t response_capacity,
                      bool probe = true) noexcept {
        const uint32_t sequence = tail_ + 1;
        const size_t descriptor = QLV_RING_DESCRIPTORS_OFFSET + size_t{next_slot()} * QLV_RING_DESCRIPTOR_BYTES;
        Barriers::release(); // slot bytes before the descriptor
        control_.store(descriptor + QLV_RING_DESCRIPTOR_FLAGS_OFFSET, flags);
        control_.store(descriptor + QLV_RING_DESCRIPTOR_BYTES_OFFSET, request_bytes);
        control_.store(descriptor + QLV_RING_DESCRIPTOR_CAPACITY_OFFSET, response_capacity);
        control_.store(descriptor + QLV_RING_DESCRIPTOR_SEQUENCE_OFFSET, sequence);
        Barriers::release(); // descriptor before TAIL
        control_.store(QLV_RING_TAIL_OFFSET, sequence);
        tail_ = sequence;

        if (!probe) {
            return {sequence, false};
        }

        return {sequence, doorbell_needed()};
    }
    // The IDLE probe for every entry published so far: true when the caller
    // must write the doorbell once.
    bool doorbell_needed() const noexcept {
        Barriers::full(); // TAIL before reading IDLE
        return control_.load(QLV_RING_IDLE_OFFSET) != 0;
    }

    // Count and context of failed posted entries; a count that moved while the
    // context was read is reported as unattributed (context zero).
    void posted_errors(uint32_t& count, uint64_t& context) const noexcept {
        const uint32_t before = acquire_load(QLV_RING_POSTED_ERRORS_OFFSET);
        const uint32_t low = control_.load(QLV_RING_POSTED_ERROR_CONTEXT_OFFSET);
        const uint32_t high = control_.load(QLV_RING_POSTED_ERROR_CONTEXT_OFFSET + 4);
        Barriers::acquire();
        const uint32_t after = control_.load(QLV_RING_POSTED_ERRORS_OFFSET);
        count = after;
        context = before == after ? (uint64_t{high} << 32) | low : 0;
    }

private:
    uint32_t acquire_load(size_t offset) const noexcept {
        const uint32_t value = control_.load(offset);
        Barriers::acquire();
        return value;
    }

    ControlView control_{};
    uint32_t tail_ = 0;
};

enum class TimelineRead { value, absent, unstable };

// Seqlocked read of one host-published slot: QEMU makes the word odd, writes
// the native id and value, then makes it even. The 64-bit value is two 32-bit
// loads, so a torn read is detected, never returned (i386 has no plain 64-bit
// load). absent: never published, or the slot now names another sync.
template <class Barriers>
TimelineRead read_timeline(const ControlView& control, uint32_t slot, uint32_t native,
                           uint64_t& value) noexcept {
    if (!control.valid() || slot >= timeline_slots || native == 0) { return TimelineRead::absent; }
    const size_t base = QLV_RING_TIMELINE_OFFSET + size_t{slot} * QLV_RING_TIMELINE_SLOT_BYTES;

    for (unsigned attempt = 0; attempt < seqlock_attempts; ++attempt) {
        const uint32_t before = control.load(base + QLV_RING_TIMELINE_SEQUENCE_OFFSET);
        Barriers::acquire();
        if ((before & 1) != 0) { continue; }
        const uint32_t id = control.load(base + QLV_RING_TIMELINE_NATIVE_OFFSET);
        const uint32_t low = control.load(base + QLV_RING_TIMELINE_VALUE_OFFSET);
        const uint32_t high = control.load(base + QLV_RING_TIMELINE_VALUE_OFFSET + 4);
        Barriers::acquire();
        if (control.load(base + QLV_RING_TIMELINE_SEQUENCE_OFFSET) != before) { continue; }
        if (before == 0 || id != native) { return TimelineRead::absent; }
        value = (uint64_t{high} << 32) | low;
        return TimelineRead::value;
    }

    return TimelineRead::unstable;
}

// Published syncs of this device, indexed by their host-assigned slot. A slot
// answers only for its exact token and owning context, and not while a posted
// WRITE_SYNC of it has not retired: its page value may predate that write.
class SyncTable final {
public:
    bool insert(uint32_t slot, uint64_t token, uint64_t context) noexcept {
        if (slot >= timeline_slots || token == 0 || context == 0) { return false; }
        erase(token);
        entries_[slot] = Entry{token, context, 0, 0, 0, false, false};
        return true;
    }
    // A posted WRITE_SYNC of token is ring entry sequence; unpublished tokens
    // are never answered locally and need no record.
    void posted_write(uint64_t token, uint64_t context, uint32_t sequence) noexcept {
        uint32_t slot = 0;

        if (find(token, context, slot)) {
            entries_[slot].write_sequence = sequence;
            entries_[slot].write_pending = true;
        }
    }
    // Forget posted writes the consumer has retired (RETIRED reached them).
    void retire_writes(uint32_t retired) noexcept {
        for (auto& entry : entries_) {
            if (entry.write_pending && reached(retired, entry.write_sequence)) { entry.write_pending = false; }
        }
    }
    // The slot's page value may answer locally: no unretired posted write.
    bool settled(uint32_t slot) const noexcept {
        return slot < timeline_slots && !entries_[slot].write_pending;
    }
    void erase(uint64_t token) noexcept {
        for (auto& entry : entries_) {
            if (entry.token == token) { entry = Entry{}; }
        }
    }
    void erase_context(uint64_t context) noexcept {
        for (auto& entry : entries_) {
            if (entry.context == context) { entry = Entry{}; }
        }
    }
    void clear() noexcept {
        for (auto& entry : entries_) { entry = Entry{}; }
    }
    bool find(uint64_t token, uint64_t context, uint32_t& slot) const noexcept {
        if (token == 0) { return false; }
        for (uint32_t index = 0; index < timeline_slots; ++index) {
            if (entries_[index].token == token) {
                slot = index;
                return entries_[index].context == context;
            }
        }
        return false;
    }
    // Record a local answer; true when this one must refresh through the host.
    bool refresh(uint32_t slot, uint64_t value) noexcept {
        if (slot >= timeline_slots) { return true; }
        auto& entry = entries_[slot];
        if (!entry.seen || entry.last_value != value) {
            entry.seen = true;
            entry.last_value = value;
            entry.unchanged = 0;
            return false;
        }
        if (++entry.unchanged < refresh_reads) { return false; }
        entry.unchanged = 0;
        return true;
    }

private:
    struct Entry final {
        uint64_t token = 0;
        uint64_t context = 0;
        uint64_t last_value = 0;
        uint32_t unchanged = 0;
        uint32_t write_sequence = 0;
        bool seen = false;
        bool write_pending = false;
    };
    Entry entries_[timeline_slots]{};
};

// Contexts whose posted SUBMIT failed. Their later exchanges report device
// loss; an unattributable failure (or overflow) marks every context lost.
class PostedFailures final {
public:
    void clear() noexcept { *this = PostedFailures{}; }
    void observe(uint32_t count, uint64_t context) noexcept {
        const uint32_t added = count - seen_;
        if (added == 0) { return; }
        seen_ = count;
        if (added != 1 || context == 0 || count_ == context_capacity) { all_ = true; return; }
        if (!lost(context)) { contexts_[count_++] = context; }
    }
    bool lost(uint64_t context) const noexcept {
        if (all_) { return true; }
        for (size_t index = 0; index < count_; ++index) {
            if (contexts_[index] == context) { return true; }
        }
        return false;
    }
    void forget(uint64_t context) noexcept {
        for (size_t index = 0; index < count_; ++index) {
            if (contexts_[index] == context) {
                contexts_[index] = contexts_[--count_];
                contexts_[count_] = 0;
                return;
            }
        }
    }

private:
    uint64_t contexts_[context_capacity]{};
    size_t count_ = 0;
    uint32_t seen_ = 0;
    bool all_ = false;
};

// Consecutive unsatisfied local WAIT answers; every refresh_reads-th goes to
// the host so a missing publication cannot keep a waiter polling forever.
class WaitValve final {
public:
    bool refresh_after_unsatisfied() noexcept {
        if (++unsatisfied_ < refresh_reads) { return false; }
        unsatisfied_ = 0;
        return true;
    }
    void satisfied() noexcept { unsatisfied_ = 0; }

private:
    uint32_t unsatisfied_ = 0;
};

// Build the reply header the host would send for READ_SYNC (value) or
// WAIT_SYNCS (status only) from the validated request: same identity, no
// payload. output must hold QLV_HEADER_BYTES; every byte is written.
inline bool local_reply(const QLV_HEADER& request, rx7900::sdma::CommandBuffer output,
                        uint64_t value, int32_t status) noexcept {
    const auto put64 = [&output](size_t offset, uint64_t word) noexcept {
        return output.write_u32(offset, static_cast<uint32_t>(word)) &&
            output.write_u32(offset + 4, static_cast<uint32_t>(word >> 32));
    };

    return output.size() >= QLV_HEADER_BYTES &&
        output.write_u32(QLV_MAGIC_OFFSET, request.magic) &&
        output.write_u32(QLV_VERSION_OFFSET, static_cast<uint32_t>(request.version) |
            (static_cast<uint32_t>(request.operation) << 16)) &&
        put64(QLV_CONTEXT_OFFSET, request.context) && put64(QLV_RESOURCE_OFFSET, request.resource) &&
        put64(QLV_VALUE_OFFSET, value) && put64(QLV_BYTES_OFFSET, request.bytes) &&
        put64(QLV_OFFSET_OFFSET, request.offset) &&
        output.write_u32(QLV_STATUS_OFFSET, static_cast<uint32_t>(status)) &&
        output.write_u32(QLV_FLAGS_OFFSET, request.flags) &&
        output.write_u32(QLV_PAYLOAD_BYTES_OFFSET, 0) &&
        output.write_u32(QLV_RESERVED_OFFSET, 0);
}

// Decide a zero-timeout WAIT_SYNCS from published values. points is the
// validated payload of count QLV_SYNC_POINT records. Returns false when any
// sync is unpublished, foreign, unstable or has an unretired posted write (the
// caller then exchanges, behind that write in ring order).
template <class Barriers>
bool local_wait(const ControlView& control, const SyncTable& syncs, uint64_t context,
                rx7900::ByteView points, uint32_t count, bool any, bool& satisfied) noexcept {
    if (count == 0 || points.size() != size_t{count} * QLV_SYNC_POINT_BYTES) { return false; }
    uint32_t reached_points = 0;

    for (uint32_t index = 0; index < count; ++index) {
        uint64_t token = 0, wanted = 0, value = 0;
        uint32_t slot = 0;
        const size_t offset = size_t{index} * QLV_SYNC_POINT_BYTES;

        if (!points.read_u64(offset, token) || !points.read_u64(offset + QLV_SYNC_VALUE_OFFSET, wanted) ||
            !syncs.find(token, context, slot) || !syncs.settled(slot) ||
            read_timeline<Barriers>(control, slot, static_cast<uint32_t>(token), value) != TimelineRead::value) {
            return false;
        }
        if (value >= wanted) { ++reached_points; }
    }

    satisfied = reached_points == count || (any && reached_points != 0);
    return true;
}

} // namespace lightning::reactos::ring

/*
Module: lightning.tests.ring
Purpose: Exercise the shared request ring, idle doorbell, timeline seqlock and bounded tables.
Created: 2026-10-23
Architecture: Portable policy tests over the production lightning_ring.hpp. A host
model follows QEMU's documented consumer order; hw/misc/lightning-venus.c itself is
tested separately by the private qtest harness. Threaded stress runs on native hosts.
*/
#include "lightning_ring.hpp"
#include "lightning_exchange_policy.hpp"
#include <stdio.h>
#include <string.h>
#if !defined(_WIN32)
#include <atomic>
#include <thread>
#endif

namespace {
using namespace lightning::reactos;
unsigned checks = 0, failures = 0;

void check(bool value, const char* text) {
    ++checks;
    if (!value) { ++failures; printf("FAIL %s\n", text); }
}

// Fences for the test build. A hook lets a deterministic test run a host step
// exactly at a barrier, which is where real interleavings can happen.
void (*release_hook)() = nullptr;
void (*full_hook)() = nullptr;
void (*acquire_hook)() = nullptr;
struct TestBarriers final {
    static void acquire() noexcept {
        __atomic_thread_fence(__ATOMIC_ACQUIRE);
        if (acquire_hook != nullptr) { auto hook = acquire_hook; acquire_hook = nullptr; hook(); }
    }
    static void release() noexcept {
        if (release_hook != nullptr) { auto hook = release_hook; release_hook = nullptr; hook(); }
        __atomic_thread_fence(__ATOMIC_RELEASE);
    }
    static void full() noexcept {
        __atomic_thread_fence(__ATOMIC_SEQ_CST);
        if (full_hook != nullptr) { auto hook = full_hook; full_hook = nullptr; hook(); }
    }
};
using TestRing = ring::Ring<TestBarriers>;

alignas(8) uint32_t control_words[ring::control_bytes / 4];
uint8_t slots[ring::data_bytes];

uint32_t word(size_t offset) { return __atomic_load_n(&control_words[offset / 4], __ATOMIC_ACQUIRE); }
void put(size_t offset, uint32_t value) { __atomic_store_n(&control_words[offset / 4], value, __ATOMIC_RELEASE); }

// QEMU's consumer, step by step (venus_ring_consume / venus_ring_start /
// venus_ring_retire), on the same words the guest uses.
struct HostModel final {
    uint32_t head = 0, retired = 0;
    bool idle = true, faulted = false;
    uint32_t posted_errors = 0;
    uint32_t timeline_sequence[ring::timeline_slots]{};
    uint32_t last_flags = 0, last_bytes = 0;
    uint8_t last_request[QLV_HEADER_BYTES]{};

    void registered() {
        *this = HostModel{};
        put(QLV_RING_IDLE_OFFSET, 1);
        put(QLV_RING_STATE_OFFSET, QLV_RING_ACTIVE);
    }
    void fault() { faulted = true; put(QLV_RING_STATE_OFFSET, QLV_RING_FAULT); }
    // Copy the next entry (HEAD), as venus_ring_start does. False: none.
    bool take() {
        const uint32_t tail = word(QLV_RING_TAIL_OFFSET);
        if (faulted || tail == head) { return false; }
        if (static_cast<uint32_t>(tail - head) > ring::entries) { fault(); return false; }
        const uint32_t sequence = head + 1;
        const size_t descriptor = QLV_RING_DESCRIPTORS_OFFSET + ((sequence - 1) % ring::entries) * QLV_RING_DESCRIPTOR_BYTES;
        const uint32_t flags = word(descriptor + QLV_RING_DESCRIPTOR_FLAGS_OFFSET);
        const uint32_t bytes = word(descriptor + QLV_RING_DESCRIPTOR_BYTES_OFFSET);
        const uint32_t capacity = word(descriptor + QLV_RING_DESCRIPTOR_CAPACITY_OFFSET);
        const bool posted = (flags & QLV_RING_POSTED) != 0;
        if (word(descriptor + QLV_RING_DESCRIPTOR_SEQUENCE_OFFSET) != sequence ||
            (flags & ~static_cast<uint32_t>(QLV_RING_POSTED)) != 0 || bytes < QLV_HEADER_BYTES ||
            bytes > ring::slot_bytes || (posted ? capacity != 0 : capacity < QLV_HEADER_BYTES)) {
            fault();
            return false;
        }
        memcpy(last_request, slots + ((sequence - 1) % ring::entries) * ring::slot_bytes, QLV_HEADER_BYTES);
        last_flags = flags;
        last_bytes = bytes;
        head = sequence;
        put(QLV_RING_HEAD_OFFSET, head);
        return true;
    }
    void retire(uint32_t mmio_status, uint32_t response_bytes, bool failed_posted = false, uint64_t context = 0) {
        if (failed_posted) {
            put(QLV_RING_POSTED_ERROR_CONTEXT_OFFSET, static_cast<uint32_t>(context));
            put(QLV_RING_POSTED_ERROR_CONTEXT_OFFSET + 4, static_cast<uint32_t>(context >> 32));
            put(QLV_RING_POSTED_ERRORS_OFFSET, ++posted_errors);
        }
        put(QLV_RING_RESULT_STATUS_OFFSET, mmio_status);
        put(QLV_RING_RESULT_BYTES_OFFSET, response_bytes);
        retired = head;
        put(QLV_RING_RETIRED_OFFSET, retired);
    }
    // The consumer's empty-ring handshake: IDLE=1, full fence, re-read TAIL.
    bool go_idle_unless_work() {
        if (word(QLV_RING_TAIL_OFFSET) != head) { return false; }
        idle = true;
        put(QLV_RING_IDLE_OFFSET, 1);
        __atomic_thread_fence(__ATOMIC_SEQ_CST);
        return word(QLV_RING_TAIL_OFFSET) == head;
    }
    void busy() { idle = false; put(QLV_RING_IDLE_OFFSET, 0); }
    void publish(uint32_t slot, uint32_t native, uint64_t value) {
        const size_t base = QLV_RING_TIMELINE_OFFSET + size_t{slot} * QLV_RING_TIMELINE_SLOT_BYTES;
        put(base, ++timeline_sequence[slot]);
        put(base + QLV_RING_TIMELINE_NATIVE_OFFSET, native);
        put(base + QLV_RING_TIMELINE_VALUE_OFFSET, static_cast<uint32_t>(value));
        put(base + QLV_RING_TIMELINE_VALUE_OFFSET + 4, static_cast<uint32_t>(value >> 32));
        put(base, ++timeline_sequence[slot]);
    }
};
HostModel host;

ring::ControlView view() {
    return {reinterpret_cast<volatile uint32_t*>(control_words), sizeof(control_words)};
}

uint8_t* next_slot(const TestRing& ring) { return slots + size_t{ring.next_slot()} * ring::slot_bytes; }

void request(uint8_t* slot, uint16_t operation, uint64_t context) {
    memset(slot, 0, QLV_HEADER_BYTES);
    const uint32_t magic = static_cast<uint32_t>(QLV_MAGIC);
    const uint16_t version = static_cast<uint16_t>(QLV_VERSION);
    memcpy(slot, &magic, 4);
    memcpy(slot + 4, &version, 2);
    memcpy(slot + 6, &operation, 2);
    memcpy(slot + 8, &context, 8);
}

TestRing fresh() {
    memset(control_words, 0xa5, sizeof(control_words));
    TestRing ring;
    check(ring.initialize(view()), "the guest header initializes on an exact control region");
    host.registered();
    return ring;
}

void layout_and_registration() {
    TestRing ring = fresh();
    check(word(QLV_RING_MAGIC_OFFSET) == QLV_RING_MAGIC && word(QLV_RING_VERSION_OFFSET) == QLV_RING_VERSION &&
        word(QLV_RING_TAIL_OFFSET) == 0, "initialize writes magic, version and a zero TAIL for REGISTER");
    bool zeroed = true;
    for (size_t offset = QLV_RING_DESCRIPTORS_OFFSET; offset < ring::control_bytes; offset += 4) {
        zeroed = zeroed && word(offset) == 0;
    }
    check(zeroed, "descriptors and the whole timeline page start zeroed");
    check(ring.health() == ring::Health::active, "registered state is visible as active");
    TestRing absent;
    check(!absent.initialize({reinterpret_cast<volatile uint32_t*>(control_words), sizeof(control_words) - 4}),
        "a short control region is refused (no partial registration)");
    check(absent.health() == ring::Health::off, "an unregistered ring reports off: old QEMU keeps the mailbox");
    put(QLV_RING_STATE_OFFSET, QLV_RING_LOST);
    check(ring.health() == ring::Health::lost, "a lost host is reported, never retried silently");
    put(QLV_RING_STATE_OFFSET, 77);
    check(ring.health() == ring::Health::off, "unknown states are not active");
    check(ring::slot_bytes >= QLV_HEADER_BYTES + QLV_CAPS_BYTES,
        "every bounded reply request fits a slot; only larger requests use the mailbox");
}

void synchronous_and_posted_entries() {
    TestRing ring = fresh();
    request(next_slot(ring), static_cast<uint16_t>(QLV_READ_SYNC), 9);
    const auto first = ring.publish(0, QLV_HEADER_BYTES, QLV_HEADER_BYTES);
    check(first.sequence == 1 && first.doorbell, "first entry rings because the consumer is idle");
    check(!ring.completed(1), "an unconsumed entry is not complete");
    check(host.take() && host.last_bytes == QLV_HEADER_BYTES && host.last_flags == 0,
        "the host sees exactly the published descriptor");
    host.retire(QLV_MMIO_COMPLETE, QLV_HEADER_BYTES);
    check(ring.completed(1) && ring.result_status() == QLV_MMIO_COMPLETE &&
        ring.result_bytes() == QLV_HEADER_BYTES && ring.drained(), "retirement publishes the result");
    host.busy();
    request(next_slot(ring), static_cast<uint16_t>(QLV_SUBMIT), 9);
    const auto posted = ring.publish(QLV_RING_POSTED, QLV_HEADER_BYTES, 0);
    check(!posted.doorbell, "a busy consumer needs no doorbell");
    check(host.take() && (host.last_flags & QLV_RING_POSTED) != 0, "posted flag reaches the host");
    host.retire(QLV_MMIO_COMPLETE, 0, true, UINT64_C(0x100000009));
    uint32_t count = 0;
    uint64_t context = 0;
    ring.posted_errors(count, context);
    check(count == 1 && context == UINT64_C(0x100000009), "a failed posted entry names its context");
    // Another failure lands while the context words are read: the snapshot
    // must not attribute the new count to the old context.
    acquire_hook = [] { acquire_hook = [] { host.retire(QLV_MMIO_COMPLETE, 0, true, UINT64_C(0x10000000a)); }; };
    ring.posted_errors(count, context);
    check(count == 2 && context == 0, "a count that moves during the read is reported unattributed");
}

void wraparound_full_and_empty() {
    TestRing ring = fresh();
    bool sequences = true;
    for (uint32_t index = 0; index < 3 * ring::entries + 3; ++index) {
        request(next_slot(ring), static_cast<uint16_t>(QLV_SUBMIT), 5);
        const uint32_t slot = ring.next_slot();
        const auto published = ring.publish(QLV_RING_POSTED, QLV_HEADER_BYTES, 0);
        sequences = sequences && published.sequence == index + 1 && slot == index % ring::entries;
        sequences = sequences && host.take();
        host.retire(QLV_MMIO_COMPLETE, 0);
    }
    check(sequences && ring.drained(), "slots and descriptors wrap with matching sequences");
    for (uint32_t index = 0; index < ring::entries; ++index) {
        request(next_slot(ring), static_cast<uint16_t>(QLV_SUBMIT), 5);
        ring.publish(QLV_RING_POSTED, QLV_HEADER_BYTES, 0);
    }
    check(ring.full(), "entries unconsumed requests fill the ring");
    check(host.take() && !ring.full(), "one copied slot frees one entry before it retires");
    check(!ring.drained(), "drained waits for retirement, not only copying");
    while (host.take()) { host.retire(QLV_MMIO_COMPLETE, 0); }
    host.retire(QLV_MMIO_COMPLETE, 0);
    check(ring.drained() && !ring.full(), "an empty ring is drained");
    check(ring::reached(5, 5) && ring::reached(6, 5) && !ring::reached(4, 5) &&
        ring::reached(2, UINT32_MAX - 1) && !ring::reached(UINT32_MAX - 1, 2),
        "sequence comparison is wrap-aware across UINT32_MAX");
}

void forged_and_oversized_descriptors() {
    TestRing ring = fresh();
    request(next_slot(ring), static_cast<uint16_t>(QLV_SUBMIT), 5);
    ring.publish(0, QLV_HEADER_BYTES, QLV_HEADER_BYTES);
    put(QLV_RING_DESCRIPTORS_OFFSET + QLV_RING_DESCRIPTOR_SEQUENCE_OFFSET, 99);
    check(!host.take() && ring.health() == ring::Health::fault, "a stale or forged descriptor sequence faults the ring");
    ring = fresh();
    put(QLV_RING_TAIL_OFFSET, ring::entries + 1);
    check(!host.take() && ring.health() == ring::Health::fault, "a TAIL beyond the ring capacity faults it");
    ring = fresh();
    request(next_slot(ring), static_cast<uint16_t>(QLV_SUBMIT), 5);
    ring.publish(0, ring::slot_bytes + 4, QLV_HEADER_BYTES);
    check(!host.take() && ring.health() == ring::Health::fault, "an oversized request is refused by its consumer");
    ring = fresh();
    request(next_slot(ring), static_cast<uint16_t>(QLV_SUBMIT), 5);
    ring.publish(QLV_RING_POSTED, QLV_HEADER_BYTES, QLV_HEADER_BYTES);
    check(!host.take(), "a posted entry cannot name a reply buffer");
}

// The race the IDLE protocol closes: the consumer finds the ring empty and
// goes idle exactly while the producer publishes. Hooks run the host step at
// the guest's barriers. A guest that read IDLE before publishing TAIL, or
// without the full fence, would strand its entry here.
void idle_doorbell_races() {
    TestRing ring = fresh();
    host.busy();
    // Host goes idle just before the guest's TAIL store: the guest must see IDLE.
    request(next_slot(ring), static_cast<uint16_t>(QLV_SUBMIT), 5);
    static bool host_saw_work = false;
    host_saw_work = true;
    release_hook = nullptr;
    // publish calls release twice: slot bytes, then descriptor->TAIL. Fire on the second.
    release_hook = [] { release_hook = [] { host_saw_work = !host.go_idle_unless_work(); }; };
    const auto late = ring.publish(QLV_RING_POSTED, QLV_HEADER_BYTES, 0);
    check(!host_saw_work && late.doorbell, "consumer idles before TAIL: the producer rings");
    check(host.take(), "the rung entry is consumable");
    host.retire(QLV_MMIO_COMPLETE, 0);
    // Host checks for work after the TAIL store but before the IDLE read: it
    // sees the entry itself; the producer may skip the doorbell.
    host.busy();
    request(next_slot(ring), static_cast<uint16_t>(QLV_SUBMIT), 5);
    full_hook = [] { host_saw_work = !host.go_idle_unless_work(); };
    const auto early = ring.publish(QLV_RING_POSTED, QLV_HEADER_BYTES, 0);
    check(host_saw_work && !early.doorbell, "consumer re-reads TAIL after IDLE: no doorbell, no lost entry");
    check(host.take(), "the unrung entry is still consumed");
    host.retire(QLV_MMIO_COMPLETE, 0);
    check(release_hook == nullptr && full_hook == nullptr, "every race hook ran");
}

// Ring_batching: entries published without the IDLE probe ring nothing until
// one probe covers all of them, whichever side moves first.
void batched_publish_probes_once() {
    TestRing ring = fresh();
    static bool probed = false;
    probed = false;
    full_hook = [] { probed = true; };

    for (int index = 0; index < 3; ++index) {
        request(next_slot(ring), static_cast<uint16_t>(QLV_SUBMIT), 5);
        check(!ring.publish(QLV_RING_POSTED, QLV_HEADER_BYTES, 0, false).doorbell,
            "an unprobed entry never asks for the doorbell itself");
    }
    check(!probed && word(QLV_RING_TAIL_OFFSET) == 3, "no probe ran, yet every TAIL store is visible");
    check(ring.doorbell_needed() && probed, "one probe after the batch finds the idle consumer");
    check(host.take() && host.take() && host.take() && !host.take(), "all three entries are consumed in order");
    host.retire(QLV_MMIO_COMPLETE, 0);
    full_hook = nullptr;

    // The consumer finishes and idles just as the producer probes: it either
    // sees the new TAIL itself or the probe sees IDLE; never neither.
    host.busy();
    request(next_slot(ring), static_cast<uint16_t>(QLV_SUBMIT), 5);
    ring.publish(QLV_RING_POSTED, QLV_HEADER_BYTES, 0, false);
    static bool host_saw_work = false;
    host_saw_work = false;
    full_hook = [] { host_saw_work = !host.go_idle_unless_work(); };
    check(!ring.doorbell_needed() && host_saw_work, "a consumer that re-reads TAIL needs no doorbell");
    check(host.take(), "the entry it saw is consumed");
    host.retire(QLV_MMIO_COMPLETE, 0);
    check(host.go_idle_unless_work(), "the consumer then idles");
    request(next_slot(ring), static_cast<uint16_t>(QLV_SUBMIT), 5);
    ring.publish(QLV_RING_POSTED, QLV_HEADER_BYTES, 0, false);
    check(ring.doorbell_needed() && host.take(), "an idle consumer before the probe gets the doorbell");
    host.retire(QLV_MMIO_COMPLETE, 0);
    check(full_hook == nullptr, "every race hook ran");
}

void posted_writes_hide_stale_values() {
    fresh();
    ring::SyncTable table;
    const uint64_t context = UINT64_C(0x100000002);
    uint32_t slot = 0;
    table.insert(1, UINT64_C(0x100000011), context);
    table.insert(2, UINT64_C(0x100000012), context);
    check(table.settled(1) && table.settled(2) && !table.settled(ring::timeline_slots),
        "published syncs start settled; slot bounds hold");
    table.posted_write(UINT64_C(0x100000011), context + 1, 5);
    check(table.settled(1), "a foreign context records nothing");
    table.posted_write(UINT64_C(0x100000099), context, 5);
    table.posted_write(UINT64_C(0x100000011), context, 5);
    check(!table.settled(1) && table.settled(2), "a posted write unsettles only its own sync");
    table.retire_writes(4);
    check(!table.settled(1), "earlier retirements do not settle it");
    table.retire_writes(5);
    check(table.settled(1), "its own retirement settles it");
    table.posted_write(UINT64_C(0x100000012), context, UINT32_C(0xffffffff));
    table.retire_writes(1);
    check(table.settled(2), "retirement order survives sequence wraparound");
    table.posted_write(UINT64_C(0x100000012), context, 9);
    check(table.find(UINT64_C(0x100000012), context, slot) && slot == 2 && !table.settled(2),
        "find still names the slot of an unsettled sync");
    table.insert(2, UINT64_C(0x100000012), context);
    check(table.settled(2), "a fresh publication starts settled");

    uint8_t packet[QLV_HEADER_BYTES + 2 * QLV_SYNC_POINT_BYTES]{};
    const uint64_t points[4] = {UINT64_C(0x100000011), 1, UINT64_C(0x100000012), 1};
    memcpy(packet + QLV_HEADER_BYTES, points, sizeof(points));
    const rx7900::ByteView payload(packet + QLV_HEADER_BYTES, 2 * QLV_SYNC_POINT_BYTES);
    host.publish(1, 0x11, 1);
    host.publish(2, 0x12, 1);
    bool satisfied = false;
    check(ring::local_wait<TestBarriers>(view(), table, context, payload, 2, false, satisfied) && satisfied,
        "settled published values answer locally");
    table.posted_write(UINT64_C(0x100000012), context, 10);
    check(!ring::local_wait<TestBarriers>(view(), table, context, payload, 2, false, satisfied),
        "a wait naming a sync with an unretired posted write goes to the host");
    table.retire_writes(10);
    check(ring::local_wait<TestBarriers>(view(), table, context, payload, 2, false, satisfied),
        "after retirement it answers locally again");
}

void timeline_reads() {
    fresh();
    const auto control = view();
    uint64_t value = 0;
    check(ring::read_timeline<TestBarriers>(control, 3, 7, value) == ring::TimelineRead::absent,
        "a never-published slot is absent");
    put(QLV_RING_TIMELINE_OFFSET + 3 * QLV_RING_TIMELINE_SLOT_BYTES + QLV_RING_TIMELINE_NATIVE_OFFSET, 7);
    check(ring::read_timeline<TestBarriers>(control, 3, 7, value) == ring::TimelineRead::absent,
        "a slot whose sequence was never advanced is absent even if its id matches");
    host.publish(3, 7, UINT64_C(0x1ffffffff));
    check(ring::read_timeline<TestBarriers>(control, 3, 7, value) == ring::TimelineRead::value &&
        value == UINT64_C(0x1ffffffff), "a published 64-bit value reads back whole");
    check(ring::read_timeline<TestBarriers>(control, 3, 8, value) == ring::TimelineRead::absent,
        "a slot reused by another sync never answers for this one");
    check(ring::read_timeline<TestBarriers>(control, ring::timeline_slots, 7, value) == ring::TimelineRead::absent,
        "slot indices are bounded");
    put(QLV_RING_TIMELINE_OFFSET + 3 * QLV_RING_TIMELINE_SLOT_BYTES, host.timeline_sequence[3] + 1);
    check(ring::read_timeline<TestBarriers>(control, 3, 7, value) == ring::TimelineRead::unstable,
        "an odd (in-progress) slot gives up after bounded attempts");
    put(QLV_RING_TIMELINE_OFFSET + 3 * QLV_RING_TIMELINE_SLOT_BYTES, host.timeline_sequence[3]);
    // A writer finishing between the reader's two words: the torn snapshot
    // (new high word, old low word) must be discarded and the read retried.
    acquire_hook = [] { acquire_hook = [] { host.publish(3, 7, UINT64_C(0x200000000)); }; };
    check(ring::read_timeline<TestBarriers>(control, 3, 7, value) == ring::TimelineRead::value &&
        value == UINT64_C(0x200000000), "a concurrent update is retried, never torn");
}

void tables_and_valves() {
    ring::SyncTable table;
    uint32_t slot = 0;
    check(table.insert(4, 0x100000011, 0x100000002) && table.find(0x100000011, 0x100000002, slot) && slot == 4,
        "a published sync answers for its slot");
    check(!table.find(0x100000011, 0x100000003, slot), "a foreign context cannot read it");
    check(!table.insert(ring::timeline_slots, 1, 1) && !table.insert(0, 0, 1), "slot and token bounds");
    check(table.insert(5, 0x100000011, 0x100000002) && table.find(0x100000011, 0x100000002, slot) && slot == 5,
        "a reassigned token keeps one entry");
    bool refreshed = false;
    for (uint32_t index = 0; index <= ring::refresh_reads; ++index) { refreshed = table.refresh(5, 9); }
    check(refreshed, "repeated unchanged answers periodically refresh through the host");
    check(!table.refresh(5, 10), "a changed value resets the staleness count");
    table.erase_context(0x100000002);
    check(!table.find(0x100000011, 0x100000002, slot), "CLOSE forgets every sync of its context");

    ring::PostedFailures failed;
    failed.observe(0, 0);
    check(!failed.lost(1), "no failures, no losses");
    failed.observe(1, 7);
    check(failed.lost(7) && !failed.lost(8), "one failure poisons one context");
    failed.forget(7);
    check(!failed.lost(7), "closed contexts are forgotten");
    failed.observe(3, 8);
    check(failed.lost(8) && failed.lost(9), "an unattributable burst poisons every context until reset");
    failed.clear();
    check(!failed.lost(9), "reset clears the poison");

    ring::WaitValve valve;
    bool refresh = false;
    for (uint32_t index = 0; index < ring::refresh_reads; ++index) { refresh = valve.refresh_after_unsatisfied(); }
    check(refresh, "unsatisfied local waits periodically ask the host");
}

QLV_HEADER header(uint16_t operation, uint64_t context) {
    QLV_HEADER value{};
    value.magic = static_cast<uint32_t>(QLV_MAGIC);
    value.version = static_cast<uint16_t>(QLV_VERSION);
    value.operation = operation;
    value.context = context;
    value.status = static_cast<int32_t>(QLV_STATUS_UNSUPPORTED);
    return value;
}

bool lookup(void*, uint64_t token, ResourceInfo& info) noexcept {
    info = {};
    info.kind = ResourceKind::sync;
    return token == UINT64_C(0x100000011) || token == UINT64_C(0x100000012);
}

// Request bytes for the fixture: local_reply writes every header field, then
// the request's payload length is restored.
void encode(const QLV_HEADER& value, uint8_t* bytes) {
    rx7900::sdma::CommandBuffer output(bytes, QLV_HEADER_BYTES);
    ring::local_reply(value, output, value.value, value.status);
    output.write_u32(QLV_PAYLOAD_BYTES_OFFSET, value.payload_bytes);
}

void local_replies_pass_the_production_validator() {
    const uint64_t context = UINT64_C(0x100000002);
    const ExchangeSession session{context, QLV_APERTURE_BYTES, nullptr, lookup};
    auto read = header(static_cast<uint16_t>(QLV_READ_SYNC), context);
    read.resource = UINT64_C(0x100000011);
    uint8_t packet[QLV_HEADER_BYTES + 2 * QLV_SYNC_POINT_BYTES]{};
    encode(read, packet);
    ValidatedExchange validated;
    check(validate_exchange_request({packet, QLV_HEADER_BYTES}, QLV_HEADER_BYTES, session, validated) ==
        ExchangeStatus::validated, "fixture READ_SYNC request is valid");
    uint8_t reply[QLV_HEADER_BYTES]{};
    ExchangeReply output;
    check(ring::local_reply(validated.header(), {reply, sizeof(reply)}, UINT64_C(0x123456789), 0) &&
        validate_exchange_reply(validated, {reply, sizeof(reply)}, output) == ExchangeStatus::validated &&
        output.header.value == UINT64_C(0x123456789), "a local READ_SYNC reply is the host's reply");
    check(!ring::local_reply(validated.header(), {reply, sizeof(reply) - 1}, 0, 0), "short reply storage is refused");

    auto wait = header(static_cast<uint16_t>(QLV_WAIT_SYNCS), context);
    wait.bytes = 2;
    wait.payload_bytes = 2 * QLV_SYNC_POINT_BYTES;
    encode(wait, packet);
    const uint64_t points[4] = {UINT64_C(0x100000011), 5, UINT64_C(0x100000012), 9};
    memcpy(packet + QLV_HEADER_BYTES, points, sizeof(points));
    check(validate_exchange_request({packet, sizeof(packet)}, QLV_HEADER_BYTES, session, validated) ==
        ExchangeStatus::validated, "fixture WAIT_SYNCS request is valid");
    check(ring::local_reply(validated.header(), {reply, sizeof(reply)}, 0, static_cast<int32_t>(QLV_STATUS_TIMEOUT)) &&
        validate_exchange_reply(validated, {reply, sizeof(reply)}, output) == ExchangeStatus::host_error &&
        output.header.status == QLV_STATUS_TIMEOUT, "a local timeout is a valid host result");

    fresh();
    ring::SyncTable table;
    table.insert(1, UINT64_C(0x100000011), context);
    bool satisfied = true;
    const rx7900::ByteView payload(packet + QLV_HEADER_BYTES, 2 * QLV_SYNC_POINT_BYTES);
    check(!ring::local_wait<TestBarriers>(view(), table, context, payload, 2, false, satisfied),
        "an unpublished point keeps the exchange");
    table.insert(2, UINT64_C(0x100000012), context);
    host.publish(1, 0x11, 5);
    host.publish(2, 0x12, 8);
    check(ring::local_wait<TestBarriers>(view(), table, context, payload, 2, false, satisfied) && !satisfied,
        "all-of waits need every point");
    check(ring::local_wait<TestBarriers>(view(), table, context, payload, 2, true, satisfied) && satisfied,
        "any-of waits need one point");
    host.publish(2, 0x12, 9);
    check(ring::local_wait<TestBarriers>(view(), table, context, payload, 2, false, satisfied) && satisfied,
        "reaching the last point satisfies the wait");
    check(!ring::local_wait<TestBarriers>(view(), table, context + 1, payload, 2, false, satisfied),
        "another context cannot wait on these syncs locally");
    check(!ring::local_wait<TestBarriers>(view(), table, context, payload, 3, false, satisfied),
        "count and payload must agree");
}

#if !defined(_WIN32)
struct StressBarriers final {
    static void acquire() noexcept { __atomic_thread_fence(__ATOMIC_ACQUIRE); }
    static void release() noexcept { __atomic_thread_fence(__ATOMIC_RELEASE); }
    static void full() noexcept { __atomic_thread_fence(__ATOMIC_SEQ_CST); }
};

// Real threads: a producer posting through the IDLE protocol and a consumer
// that sleeps until a doorbell whenever it found the ring empty. A lost wakeup
// leaves entries unretired at the deadline.
void threaded_idle_doorbell() {
    memset(control_words, 0, sizeof(control_words));
    ring::Ring<StressBarriers> ring;
    ring.initialize(view());
    put(QLV_RING_IDLE_OFFSET, 1);
    put(QLV_RING_STATE_OFFSET, QLV_RING_ACTIVE);
    constexpr uint32_t total = 200000;
    std::atomic<uint32_t> doorbells{0};
    std::atomic<bool> stop{false};
    std::thread consumer([&] {
        uint32_t head = 0, seen = 0;
        bool idle = true;
        while (!stop.load()) {
            const uint32_t tail = word(QLV_RING_TAIL_OFFSET);
            if (tail == head) {
                if (!idle) {
                    idle = true;
                    put(QLV_RING_IDLE_OFFSET, 1);
                    __atomic_thread_fence(__ATOMIC_SEQ_CST);
                    if (word(QLV_RING_TAIL_OFFSET) != head) { continue; }
                }
                while (doorbells.load() == seen && !stop.load()) { std::this_thread::yield(); }
                seen = doorbells.load();
                continue;
            }
            if (idle) { idle = false; put(QLV_RING_IDLE_OFFSET, 0); }
            head = head + 1;
            put(QLV_RING_HEAD_OFFSET, head);
            put(QLV_RING_RETIRED_OFFSET, head);
        }
    });
    uint32_t rung = 0;
    for (uint32_t index = 0; index < total; ++index) {
        while (ring.full()) { std::this_thread::yield(); }
        const auto published = ring.publish(QLV_RING_POSTED, QLV_HEADER_BYTES, 0);
        if (published.doorbell) { ++rung; doorbells.fetch_add(1); }
    }
    bool drained = false;
    for (unsigned spin = 0; spin < 2000000 && !drained; ++spin) {
        drained = ring.drained();
        if (!drained) { std::this_thread::yield(); }
    }
    stop.store(true);
    doorbells.fetch_add(1);
    consumer.join();
    check(drained, "200000 posted entries all retire: no lost doorbell under real concurrency");
    check(rung > 0 && rung < total, "doorbells are rung only while the consumer is idle");
    printf("threaded: %u entries, %u doorbells\n", total, rung);
}

// Ring_batching under real threads: batches of 1..4 unprobed entries, one
// probe each. A batch whose probe missed an idle consumer stays unretired.
void threaded_batched_doorbell() {
    memset(control_words, 0, sizeof(control_words));
    ring::Ring<StressBarriers> ring;
    ring.initialize(view());
    put(QLV_RING_IDLE_OFFSET, 1);
    put(QLV_RING_STATE_OFFSET, QLV_RING_ACTIVE);
    constexpr uint32_t total = 200000;
    std::atomic<uint32_t> doorbells{0};
    std::atomic<bool> stop{false};
    std::thread consumer([&] {
        uint32_t head = 0, seen = 0;
        bool idle = true;
        while (!stop.load()) {
            const uint32_t tail = word(QLV_RING_TAIL_OFFSET);
            if (tail == head) {
                if (!idle) {
                    idle = true;
                    put(QLV_RING_IDLE_OFFSET, 1);
                    __atomic_thread_fence(__ATOMIC_SEQ_CST);
                    if (word(QLV_RING_TAIL_OFFSET) != head) { continue; }
                }
                while (doorbells.load() == seen && !stop.load()) { std::this_thread::yield(); }
                seen = doorbells.load();
                continue;
            }
            if (idle) { idle = false; put(QLV_RING_IDLE_OFFSET, 0); }
            head = head + 1;
            put(QLV_RING_HEAD_OFFSET, head);
            put(QLV_RING_RETIRED_OFFSET, head);
        }
    });
    uint32_t rung = 0, batches = 0;
    for (uint32_t index = 0; index < total; ++batches) {
        const uint32_t size = 1 + batches % 4;
        for (uint32_t entry = 0; entry < size && index < total; ++entry, ++index) {
            // A full ring is waited for only after the owed probe.
            if (ring.full()) {
                if (ring.doorbell_needed()) { ++rung; doorbells.fetch_add(1); }
                while (ring.full()) { std::this_thread::yield(); }
            }
            ring.publish(QLV_RING_POSTED, QLV_HEADER_BYTES, 0, false);
        }
        if (ring.doorbell_needed()) { ++rung; doorbells.fetch_add(1); }
    }
    bool drained = false;
    for (unsigned spin = 0; spin < 2000000 && !drained; ++spin) {
        drained = ring.drained();
        if (!drained) { std::this_thread::yield(); }
    }
    stop.store(true);
    doorbells.fetch_add(1);
    consumer.join();
    check(drained, "200000 batched posted entries all retire: one probe per batch loses no doorbell");
    check(rung > 0 && rung <= batches + total / ring::entries, "at most one doorbell per batch (or full-ring wait)");
    printf("threaded batches: %u entries, %u batches, %u doorbells\n", total, batches, rung);
}

// A writer publishing k * 0x100000001 (equal halves) while a reader checks
// that every value it accepts has equal halves and never goes backwards.
void threaded_tear_free_timeline() {
    memset(control_words, 0, sizeof(control_words));
    std::atomic<bool> stop{false};
    std::thread writer([&] {
        uint32_t sequence = 0;
        const size_t base = QLV_RING_TIMELINE_OFFSET;
        for (uint32_t k = 1; !stop.load(); ++k) {
            put(base, ++sequence);
            put(base + QLV_RING_TIMELINE_NATIVE_OFFSET, 7);
            put(base + QLV_RING_TIMELINE_VALUE_OFFSET, k);
            put(base + QLV_RING_TIMELINE_VALUE_OFFSET + 4, k);
            put(base, ++sequence);
        }
    });
    uint64_t last = 0, values = 0;
    bool torn = false, backwards = false;
    for (unsigned index = 0; index < 3000000; ++index) {
        uint64_t value = 0;
        if (ring::read_timeline<StressBarriers>(view(), 0, 7, value) != ring::TimelineRead::value) { continue; }
        ++values;
        torn = torn || static_cast<uint32_t>(value) != static_cast<uint32_t>(value >> 32);
        backwards = backwards || value < last;
        last = value;
    }
    stop.store(true);
    writer.join();
    check(values > 0 && !torn, "two-word reads under a concurrent writer are never torn");
    check(!backwards, "published values read monotonically");
    printf("threaded timeline: %llu consistent reads\n", static_cast<unsigned long long>(values));
}
#endif

} // namespace

int main() {
    layout_and_registration();
    synchronous_and_posted_entries();
    wraparound_full_and_empty();
    forged_and_oversized_descriptors();
    idle_doorbell_races();
    batched_publish_probes_once();
    timeline_reads();
    tables_and_valves();
    posted_writes_hide_stale_values();
    local_replies_pass_the_production_validator();
#if !defined(_WIN32)
    threaded_idle_doorbell();
    threaded_batched_doorbell();
    threaded_tear_free_timeline();
#endif
    printf("Lightning shared ring: %u checks, %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

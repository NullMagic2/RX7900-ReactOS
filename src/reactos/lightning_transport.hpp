/*
Module: lightning.reactos.transport
Purpose: Own native NT5 assigned BAR0 and common buffers for bounded QLV queries.
Created: 2026-10-21
Architecture: Exclusive PnP owner at PASSIVE_LEVEL; no WDDM or user DMA mappings.
With MMIO_SHARED_RING (lightning_ring.hpp) requests go through a registered
guest-RAM ring with one doorbell write, completion is polled from RAM, and
published sync values answer READ_SYNC/WAIT_SYNCS without an exchange. Inside
a renderer batch posted entries share one IDLE probe and doorbell.
*/
#pragma once

#include <ntddk.h>
#include "lightning_exchange_policy.hpp"
#include "lightning_ring.hpp"

namespace lightning::reactos {

namespace transport_wait {
inline constexpr ULONG short_polls = 8;
inline constexpr ULONG short_poll_microseconds = 25;
inline constexpr ULONG maximum_sleeps = 6000;
// Ring completion spins on guest RAM (no exit) for the same budget the
// register mailbox spent stalling, then sleeps like it. The spin is bounded by
// a TSC budget calibrated once per start, or this iteration count without one.
inline constexpr ULONG ring_spin_microseconds = short_polls * short_poll_microseconds;
inline constexpr ULONG ring_spin_iterations = 4096;
inline constexpr ULONG ring_calibration_microseconds = 50;
// A published-timeline wait is for GPU completion, not a host round trip, and
// its waiter (the WSI worker, a drain) runs beside the application thread on
// a single vCPU, so it spins for this fraction of the ring budget and then
// lets the caller sleep.
inline constexpr ULONG published_spin_divisor = 4;
enum class Pause { stall, sleep, expired };

// A reply is checked before consuming any pause. Short polls supplement the
// original sleep budget, never replace its real elapsed-time deadline.
class Budget final {
public:
    Pause next(uint64_t elapsed_100ns) noexcept {
        if (elapsed_100ns >= QLV_KERNEL_EXCHANGE_TIMEOUT_100NS) { return Pause::expired; }
        if (stalls_ < short_polls) { ++stalls_; return Pause::stall; }
        if (sleeps_ < maximum_sleeps) { ++sleeps_; return Pause::sleep; }
        return Pause::expired;
    }
private:
    ULONG stalls_ = 0, sleeps_ = 0;
};
} // namespace transport_wait

#if defined(RX7900_LIGHTNING_TRANSPORT_TIMING)
namespace transport_timing {
// Internal, device-owned diagnostics. These are not a wire ABI. The same FDO
// mutex that serializes transport calls protects counters and aggregate dumps.
// Counts only: the actual guest's PIT QPC was intrusive and sometimes regressed.
struct Row final {
    uint64_t calls = 0, succeeded = 0, failed = 0, submitted = 0;
    uint64_t immediate = 0, pending = 0, sleeps = 0, stalls = 0, short_complete = 0;
    uint64_t context = 0;
    ULONG last_status = 0;
    bool context_seen = false, mixed_contexts = false;
};

// Shared-ring path totals: doorbell writes (the only MMIO per ring request),
// entries, answers served from the timeline page and their host refreshes.
struct RingRow final {
    uint64_t doorbells = 0, synchronous = 0, posted = 0, mailbox = 0;
    uint64_t local_reads = 0, local_waits = 0, local_lost = 0, refreshes = 0;
    uint64_t spin_complete = 0, sleeps = 0, published_syncs = 0;
    // Renderer batches, posted entries published without their own IDLE probe,
    // and posted WRITE_SYNCs (a subset of posted).
    uint64_t batches = 0, deferred = 0, posted_writes = 0;
    // Submission-path waits for GPU completion inside the renderer: a busy
    // fence reset/destroy, a full completion ledger, and how many of either
    // ran inside a batch (that is, on the thread that sent the batch).
    uint64_t busy_waits = 0, ledger_waits = 0, batched_waits = 0;
};

struct Aggregates final {
    static constexpr size_t row_count = static_cast<size_t>(QLV_SYNC_EVENT) + 1;
    Row rows[row_count]{}; // Row zero includes invalid/unknown operations.
    RingRow ring{};
    uint64_t dumps = 0;
    bool saturated = false;

    void add(uint64_t& target, uint64_t value) noexcept {
        if (value > UINT64_MAX - target) { target = UINT64_MAX; saturated = true; }
        else { target += value; }
    }
};

// One native invocation; no request/response pointer is retained. Every return
// uses finish, while destruction records its result. No clock reads or logging
// occur in this scope, including when the counters are enabled.
class Scope final {
public:
    Scope(Aggregates& totals, uint16_t operation, uint64_t context) noexcept
        : totals_(totals), row_(totals.rows[operation < Aggregates::row_count ? operation : 0]) {
        totals_.add(row_.calls, 1);
        if (!row_.context_seen) { row_.context = context; row_.context_seen = true; }
        else if (row_.context != context) { row_.mixed_contexts = true; }
    }
    ~Scope() noexcept {
        totals_.add(NT_SUCCESS(status_) ? row_.succeeded : row_.failed, 1);
        row_.last_status = static_cast<ULONG>(status_);
    }
    Scope(const Scope&) = delete;
    Scope& operator=(const Scope&) = delete;

    NTSTATUS finish(NTSTATUS status) noexcept { status_ = status; return status; }
    void after_submit() noexcept { totals_.add(row_.submitted, 1); }
    void pending() noexcept { pending_ = true; totals_.add(row_.pending, 1); }
    void complete() noexcept {
        if (!pending_) { totals_.add(row_.immediate, 1); }
        else if (!slept_) { totals_.add(row_.short_complete, 1); }
    }
    void before_sleep() noexcept { slept_ = true; totals_.add(row_.sleeps, 1); }
    void stall() noexcept { totals_.add(row_.stalls, 1); }

private:
    Aggregates& totals_;
    Row& row_;
    NTSTATUS status_ = STATUS_UNSUCCESSFUL;
    bool pending_ = false, slept_ = false;
};
} // namespace transport_timing
#endif

// The FDO mutex serializes all calls at PASSIVE_LEVEL. The attached lower stack
// and PDO remain alive through stop. No buffer or register pointer escapes this
// owner. Explicit stop is mandatory; trivial destruction cannot cancel DMA.
class NativeTransport final {
public:
    struct PrivateSubmitFence final { uint64_t sync = 0; uint64_t point = 0; };
    NativeTransport() noexcept = default;
    NativeTransport(const NativeTransport&) = delete;
    NativeTransport& operator=(const NativeTransport&) = delete;

    NTSTATUS start(PDEVICE_OBJECT pdo,
                   PCM_RESOURCE_LIST raw, PCM_RESOURCE_LIST translated) noexcept;
    // allow_mmio=false after surprise removal. An uncertain RESET retains all
    // DMA/adapter ownership until reboot; callers must prohibit unload/restart.
    NTSTATUS stop(bool allow_mmio = true) noexcept;
    // Device-power transition: retain assigned mapping/DMA ownership while D3;
    // resume only after the lower stack confirms D0. No register access in D3.
    NTSTATUS suspend() noexcept;
    NTSTATUS resume() noexcept;
    bool ready() const noexcept { return ready_; }
    bool quarantined() const noexcept { return quarantined_; }
    const LightningCapabilities& capabilities() const noexcept { return capabilities_; }
    bool aperture(PHYSICAL_ADDRESS& base, uint64_t& bytes) const noexcept;
    // Serialized native renderer only. Valid wire errors remain completed replies;
    // callers must not convert WAIT timeout into a hardware reset.
    NTSTATUS renderer_exchange(rx7900::ByteView packet, size_t output_capacity,
                              const ExchangeSession& session, ExchangeReply& output,
                              const PrivateSubmitFence* fence = nullptr,
                              bool* fused = nullptr) noexcept;
    NTSTATUS claim_renderer() noexcept;
    NTSTATUS recover_renderer() noexcept;
    // Opt-in (the serialized renderer decides): with an active ring, SUBMIT
    // returns once queued; a later host failure reports DEVICE_LOST to every
    // later exchange of that context, as virtio-gpu EXECBUFFER would. A peer
    // with MMIO_POSTED_SYNC_WRITES posts WRITE_SYNC the same way; until it
    // retires, that sync is never answered from the timeline page.
    void post_submits(bool enable) noexcept { post_submits_ = enable; }
    // Serialized renderer only, around the exchanges of one batch: posted
    // entries skip their IDLE probe and end_batch probes once for all of them.
    // Every wait below (a synchronous entry, a full ring, the mailbox drain,
    // await_published) probes first, so no entry ever waits unannounced.
    // Nesting is not supported; end_batch is harmless after a reset.
    void begin_batch() noexcept;
    void end_batch() noexcept;
    // Serialized renderer only: count a submission-path completion wait
    // (timing builds; otherwise nothing).
    void note_submission_wait(bool ledger_full) noexcept;
    // Serialized renderer only: spin on guest RAM (no exit) within a quarter of
    // the ring spin budget until the published sync token of context exceeds value.
    // False: not published, ring inactive, or no progress; keep polling.
    bool await_published(uint64_t context, uint64_t token, uint64_t value) noexcept;
    // Serialized renderer only: the active ring publishes this sync token of
    // context in the timeline page (read_timeline can answer it locally).
    bool published(uint64_t context, uint64_t token) const noexcept;
    // Trusted serialized owner only. Compile-time opt-in; otherwise a no-op.
    // Cumulative counts only, no clock, MMIO, DMA, user ABI or per-call logs.
    void dump_timing() noexcept;

private:
    NTSTATUS acquire_bus(PDEVICE_OBJECT pdo) noexcept;
    NTSTATUS find_bar(PCM_RESOURCE_LIST raw, PCM_RESOURCE_LIST translated,
                      PHYSICAL_ADDRESS& base) noexcept;
    void find_aperture(PCM_RESOURCE_LIST raw, PCM_RESOURCE_LIST translated) noexcept;
    // posted: may return after queuing (posted then reports true); only an
    // eligible SUBMIT on an active ring is posted, anything else waits.
    NTSTATUS transact(PVOID request, PHYSICAL_ADDRESS address, ULONG bytes,
                      ULONG capacity, ULONG& returned, bool* posted = nullptr) noexcept;
    bool ring_usable() const noexcept;
    // Probe IDLE once for every entry published without its own probe and
    // write the doorbell if QEMU's consumer is idle.
    void announce() noexcept;
    bool register_ring() noexcept;
    void forget_ring() noexcept;
    void calibrate_spin() noexcept;
    // STATUS_SUCCESS, STATUS_IO_TIMEOUT, or STATUS_DEVICE_PROTOCOL_ERROR for a
    // ring that faulted or lost its host before the entry retired.
    NTSTATUS ring_wait(uint32_t sequence, bool drain, bool& waited, bool& slept) noexcept;
    bool local_answer(const ValidatedExchange& validated, rx7900::ByteView snapshot,
                      ExchangeReply& output) noexcept;
    bool synthesized_reply(const ValidatedExchange& validated, rx7900::ByteView snapshot,
                           uint64_t value, int32_t status, ExchangeReply& output) noexcept;
    void release_large_request() noexcept;
    bool bus_master(bool enable) noexcept;
    // The vendor ID reads 0xFFFF twice: the function left the bus.
    bool function_absent() const noexcept;
    bool read(ULONG offset, ULONG& value) const noexcept;
    bool write(ULONG offset, ULONG value) const noexcept;
    bool reset() noexcept;
    NTSTATUS exchange(uint16_t operation, uint64_t context, Reply& output) noexcept;
    NTSTATUS quiesce(bool allow_mmio) noexcept;
    void release_resources() noexcept;

    BUS_INTERFACE_STANDARD bus_{};
    PDMA_ADAPTER adapter_ = nullptr;
    PUCHAR registers_ = nullptr;
    PVOID request_ = nullptr;
    PVOID response_ = nullptr;
    PHYSICAL_ADDRESS request_address_{};
    PHYSICAL_ADDRESS response_address_{};
    PVOID large_request_ = nullptr;
    PHYSICAL_ADDRESS large_request_address_{};
    ULONG large_request_bytes_ = 0;
    PHYSICAL_ADDRESS aperture_base_{};
    uint64_t aperture_bytes_ = 0;
    uint64_t context_ = 0;
    bool bus_acquired_ = false;
    bool identity_valid_ = false;
    bool bus_master_owned_ = false;
    bool ever_submitted_ = false;
    bool in_flight_ = false;
    bool ready_ = false;
    bool quarantined_ = false;
    LightningCapabilities capabilities_{};
#if defined(RX7900_LIGHTNING_TRANSPORT_TIMING)
    transport_timing::Aggregates timing_{};
#endif
    // x86 TSO ordering for the shared ring (defined with the native code).
    struct KernelBarriers final {
        static void acquire() noexcept;
        static void release() noexcept;
        static void full() noexcept;
    };
    // Shared ring: two nonpaged common buffers registered with QEMU, the
    // control pages and the request slots; freed only with the other DMA.
    PVOID ring_control_ = nullptr;
    PHYSICAL_ADDRESS ring_control_address_{};
    PVOID ring_slots_ = nullptr;
    PHYSICAL_ADDRESS ring_slots_address_{};
    ring::Ring<KernelBarriers> ring_{};
    ring::SyncTable ring_syncs_{};
    ring::PostedFailures ring_failures_{};
    ring::WaitValve ring_waits_{};
    uint64_t spin_ticks_ = 0;
    bool ring_active_ = false;
    bool host_publish_ = false;
    bool post_submits_ = false;
    // The registered peer accepts posted WRITE_SYNC (MMIO_POSTED_SYNC_WRITES).
    bool posted_writes_ = false;
    bool batching_ = false;
    // Entries were published without an IDLE probe since the last one.
    bool announce_owed_ = false;
    // Sequence of the newest posted entry (valid after transact posted it).
    uint32_t posted_sequence_ = 0;
};

} // namespace lightning::reactos

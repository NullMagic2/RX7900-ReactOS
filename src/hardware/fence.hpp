/*
Module: rx7900.hardware.fence
Purpose: Kernel-ring fence completion (amdgpu_fence semantics), job lockup
detection and CP/VM fault routing for the Navi31 gfx, compute and SDMA rings.
Created: 2026-10-22
Architecture: Device-owned, portable state machine. Gfx11Queues and Sdma6 emit
fences through begin_submission/publish_submission; IrqService feeds EOP, SDMA
trap, CP fault and VM fault records through its drain consumer (DPC context);
the native owner supplies a lock, a monotonic clock, per-waiter events and log
sinks (src/reactos/fence_native). Nothing here resets hardware: a hung ring is
quarantined and an explicit AsicInit::mode1_reset is reported as required.
Copyright 2009 Jerome Glisse; Copyright 2015, 2016, 2021, 2022 Advanced Micro
Devices, Inc. Adapted Linux v6.12 amdgpu_fence.c (amdgpu_fence_emit,
amdgpu_fence_process, amdgpu_fence_fallback, amdgpu_fence_schedule_fallback,
amdgpu_fence_enable_signaling, amdgpu_fence_driver_init_ring/start_ring),
amdgpu.h (AMDGPU_FENCE_JIFFIES_TIMEOUT), amdgpu_drv.c (amdgpu_sched_hw_submission,
lockup_timeout, vm_fault_stop), amdgpu_device.c
(amdgpu_device_get_job_timeout_settings), amdgpu_job.c (amdgpu_job_timedout),
drivers/gpu/drm/scheduler/sched_main.c (drm_sched_job_begin,
drm_sched_start_timeout, drm_sched_get_finished_job, drm_sched_fault,
drm_sched_job_timedout), gfx_v11_0.c (gfx_v11_0_eop_irq,
gfx_v11_0_handle_priv_fault), sdma_v6_0.c (sdma_v6_0_process_trap_irq),
gmc_v11_0.c (gmc_v11_0_process_interrupt), gfxhub_v3_0.c (UTCL2 client ids),
kernel/printk/printk.c (printk_ratelimit_state) and lib/ratelimit.c
(___ratelimit). Notice and provenance: licenses/AMDGPU-SDMA.txt.
*/
#pragma once

#include "irq_service.hpp"

namespace rx7900::hardware {

// Fence ring index convention shared with Gfx11Queues and Sdma6: 0 is the gfx
// ring, 1 + i compute ring i (the Gfx11Queues module index), then
// fence_ring_sdma0 + n SDMA instance n.
constexpr uint32_t fence_gfx_rings = 1;
constexpr uint32_t fence_compute_rings = 8;
constexpr uint32_t fence_sdma_rings = 2;
constexpr uint32_t fence_ring_gfx = 0;
constexpr uint32_t fence_ring_compute0 = fence_gfx_rings;
constexpr uint32_t fence_ring_sdma0 = fence_gfx_rings + fence_compute_rings;
constexpr uint32_t fence_max_rings = fence_ring_sdma0 + fence_sdma_rings;

// amdgpu_fence_driver_init_ring: num_fences_mask = num_hw_submission * 2 - 1
// with amdgpu_sched_hw_submission 2, so four fences may be outstanding per
// ring. amdgpu_fence_emit waits for the fence occupying the new sequence's slot;
// begin_submission instead returns busy while four are outstanding.
constexpr uint32_t fence_hw_submission = 2;
constexpr uint32_t fence_slots = fence_hw_submission * 2;

// AMDGPU_FENCE_JIFFIES_TIMEOUT (HZ / 2): the fallback timer period.
constexpr uint64_t fence_fallback_timeout_us = 500000;

// amdgpu_device_get_job_timeout_settings defaults on bare metal (empty
// lockup_timeout): msecs_to_jiffies(10000) for gfx, SDMA and video, 60000 for
// compute (SR-IOV values do not apply).
constexpr uint32_t fence_gfx_timeout_ms = 10000;
constexpr uint32_t fence_compute_timeout_ms = 60000;
constexpr uint32_t fence_sdma_timeout_ms = 10000;
// Largest accepted override; Linux maps negative values to "infinite", which
// this driver refuses.
constexpr uint32_t fence_max_timeout_ms = 3600000;

// printk_ratelimit_state: DEFINE_RATELIMIT_STATE(printk_ratelimit_state,
// 5 * HZ, 10), applied by gmc_v11_0_process_interrupt to fault reports.
constexpr uint64_t fence_report_interval_us = 5000000;
constexpr uint32_t fence_report_burst = 10;

// Concurrent PASSIVE_LEVEL waiters (one native event each).
constexpr uint32_t fence_waiter_slots = 8;

// gfxhub_v3_0.c gfxhub_client_ids: GCVM_L2_PROTECTION_FAULT_STATUS.CID values.
constexpr uint32_t fence_utcl2_cid_cpf = 4;
constexpr uint32_t fence_utcl2_cid_cpc = 5;
constexpr uint32_t fence_utcl2_cid_cpg = 6;
constexpr uint32_t fence_utcl2_cid_sdma0 = 13;
constexpr uint32_t fence_utcl2_cid_sdma1 = 14;

// Atomic 64-bit store of a ring WPTR shadow (Linux atomic64_set on
// wptr_cpu_addr). address is an 8-byte aligned CPU pointer into a retained,
// CPU-visible writeback allocation; the engine may read it concurrently, so the
// native implementation must store all 8 bytes in one access (i386:
// InterlockedCompareExchange64 / cmpxchg8b). Returns false on refusal.
struct RingPointerStore final {
    void* context;
    bool (*store)(void* context, uint8_t* address, uint64_t value) noexcept;
};

enum class FenceStatus {
    ready,
    attached,
    detached,
    released,
    signalled,
    pending,
    // A wait ran out of time; the ring is not hung by that alone.
    timed_out,
    // Four fences outstanding (submission) or no free waiter slot (wait).
    busy,
    // The ring was quarantined (job timeout or fault); see FenceRingReport.
    ring_hung,
    unavailable,
    invalid_argument
};

enum class FenceEngine : uint32_t { gfx, compute, sdma };

enum class FenceRingState : uint32_t { detached, active, hung };

// Why a ring was quarantined. job_timeout and the three CP faults follow
// amdgpu_job_timedout (drm_sched_fault starts it immediately); the others are
// fail-closed decisions of this driver.
enum class FenceHangReason : uint32_t {
    none,
    job_timeout,
    cp_priv_reg,
    cp_priv_instr,
    cp_bad_opcode,
    // A VMID 0 GFXHUB fault attributed to this kernel ring by UTCL2 client id.
    vm_fault,
    // The fence writeback held a value outside (last signalled, last emitted].
    fence_out_of_window,
    writeback_unreadable,
    // A doorbell, WPTR or HDP store after publish_submission failed.
    submit_failure
};

// Native services, all noexcept and callable at the IRQL stated.
//  lock/unlock: one non-reentrant lock over all manager state, taken at
//    <= DISPATCH_LEVEL (native: KSPIN_LOCK). Never held across block.
//  now_us: monotonic microseconds, any IRQL <= DISPATCH_LEVEL.
//  signal/reset/block: waiter slot s owns one auto-reset event. signal runs
//    under the lock at <= DISPATCH_LEVEL; reset under the lock at PASSIVE_LEVEL;
//    block at PASSIVE_LEVEL without the lock, returning when the event is set or
//    after timeout_us (the result is advisory: the caller re-checks).
//  report_vm_fault, report_hang: optional log sinks (null skips them), called
//    under the lock at <= DISPATCH_LEVEL; suppressed is the printk_ratelimit
//    "callbacks suppressed" count flushed at a window change, else 0.
struct FencePlatform final {
    void* context;
    void (*lock)(void* context) noexcept;
    void (*unlock)(void* context) noexcept;
    uint64_t (*now_us)(void* context) noexcept;
    void (*signal)(void* context, uint32_t slot) noexcept;
    void (*reset)(void* context, uint32_t slot) noexcept;
    bool (*block)(void* context, uint32_t slot, uint64_t timeout_us) noexcept;
    void (*report_vm_fault)(void* context, const IrqVmFault& fault, uint32_t suppressed) noexcept;
    void (*report_hang)(void* context, uint32_t ring, FenceHangReason reason,
                        uint32_t signalled_seq, uint32_t emitted_seq) noexcept;
};

// One kernel ring. gfx: me 0, pipe/queue of the ring (EOP from gfx_eop[pipe]);
// compute: me 1 or 2 (compute_eop[me - 1][pipe][queue]); sdma: instance 0..1
// (sdma_trap[instance]). fence_writeback is the CPU view of the ring's fence
// dword (Linux fence_drv.cpu_addr), at least 4 bytes, owned by the queue module
// and kept mapped until detach_ring. initial_seq is the ring's sync_seq so far
// (amdgpu_fence_driver_start_ring): the writeback must already hold it.
// initial_position is the ring write pointer (dwords) all earlier frames end at.
// timeout_ms 0 selects the engine's Linux default.
struct FenceRingConfig final {
    FenceEngine engine;
    uint32_t me;
    uint32_t pipe;
    uint32_t queue;
    uint32_t instance;
    ByteView fence_writeback;
    uint32_t initial_seq;
    uint64_t initial_position;
    uint32_t timeout_ms;
};

// begin_submission result: the sequence the next fence must carry (sync_seq + 1)
// and the ring position up to which the engine has provably consumed the ring
// (the end of the newest signalled frame).
struct FenceReservation final {
    uint32_t sequence;
    uint64_t retired_position;
    uint32_t outstanding;
};

struct FenceRingReport final {
    FenceRingState state = FenceRingState::detached;
    FenceEngine engine = FenceEngine::gfx;
    uint32_t me = 0;
    uint32_t pipe = 0;
    uint32_t queue = 0;
    uint32_t instance = 0;
    uint32_t timeout_ms = 0;
    // fence_drv.sync_seq (last emitted) and fence_drv.last_seq (last signalled).
    uint32_t sync_seq = 0;
    uint32_t last_seq = 0;
    uint64_t retired_position = 0;
    // drm_sched work_tdr and fence_drv.fallback_timer state.
    bool timeout_armed = false;
    uint64_t timeout_deadline_us = 0;
    bool fallback_armed = false;
    uint64_t fallback_deadline_us = 0;
    uint64_t submissions = 0;
    uint64_t signalled = 0;
    // amdgpu_fence_process calls by origin, and fallback timer expiries that
    // found progress (Linux DRM_WARN "Fence fallback timer expired").
    uint32_t interrupt_processes = 0;
    uint32_t fallback_expiries = 0;
    uint32_t fallback_progress = 0;
    uint32_t cp_faults = 0;
    uint32_t vm_faults = 0;
    FenceHangReason hang_reason = FenceHangReason::none;
    // Linux "ring %s timeout, signaled seq=%u, emitted seq=%u".
    uint32_t hang_signalled_seq = 0;
    uint32_t hang_emitted_seq = 0;
    uint64_t hang_time_us = 0;
};

struct FenceReport final {
    FenceStatus status = FenceStatus::unavailable;
    // A ring is hung: only an explicit AsicInit::mode1_reset (then
    // release_after_reset of every owner) can return it to service.
    bool reset_required = false;
    uint32_t hung_rings = 0;
    // IrqService vectors lost: every active ring was re-read.
    uint32_t lost_event_rescans = 0;
    // CP faults whose ME/pipe/queue matched no attached kernel ring (Linux: no
    // scheduler matched, nothing happens) or carried ME 3 (Linux BUG()).
    uint32_t cp_faults_unattributed = 0;
    // VM faults: all, rate-limit printed and suppressed, overwritten in the
    // IrqService log before they were consumed, VMID 0 faults (kernel context),
    // VMID 0 faults on MMHUB or whose GFXHUB client could not be matched, and
    // faults of process VMIDs (reported only: vm_fault_stop NEVER).
    uint32_t vm_faults = 0;
    uint32_t vm_faults_printed = 0;
    uint32_t vm_faults_suppressed = 0;
    uint32_t vm_faults_lost = 0;
    uint32_t vm_faults_vmid0 = 0;
    uint32_t vm_faults_vmid0_unattributed = 0;
    uint32_t vm_faults_process = 0;
    IrqVmFault last_vm_fault{};
    // Waits that found every waiter slot in use.
    uint32_t waiter_exhaustion = 0;
    FenceRingReport rings[fence_max_rings]{};
};

// Confirms, after a reset, that no engine can still write any fence writeback
// or fetch any ring this manager tracked (never inferred from a failure).
struct FenceRecovery final {
    void* context;
    bool (*confirm_reset_quiescence)(void* context) noexcept;
};

// Device-owned, constructed once in nonpaged storage; trivially destructible.
// Every public method takes the platform lock itself; none may be called with
// it held. IRQL: initialize, attach_ring, detach_ring, wait, release_after_reset
// at PASSIVE_LEVEL; begin/publish_submission, mark_hung, query, poll,
// process_interrupts, report at <= DISPATCH_LEVEL. Submission for one ring is
// serialized by its queue module (one submitter per ring, as Linux's ring lock).
class FenceManager final {
public:
    FenceManager() noexcept = default;
    FenceManager(const FenceManager&) = delete;
    FenceManager& operator=(const FenceManager&) = delete;

    // Once. lock, unlock, now_us, signal, reset and block are required.
    FenceStatus initialize(FencePlatform platform) noexcept;

    // amdgpu_fence_driver_init_ring + start_ring for a detached ring index.
    // The engine identity must be unique among attached rings and the writeback
    // must read initial_seq. Returns attached.
    FenceStatus attach_ring(uint32_t ring, const FenceRingConfig& config) noexcept;
    // Detaches an idle active ring (every emitted fence signalled). A hung ring
    // only with reset_confirmed: its owner has just proven reset quiescence.
    FenceStatus detach_ring(uint32_t ring, bool reset_confirmed) noexcept;

    // amdgpu_fence_emit, first half: processes the ring writeback, then reserves
    // sync_seq + 1. busy while fence_slots fences are outstanding; ring_hung or
    // unavailable when the ring cannot take work. No state changes.
    FenceStatus begin_submission(uint32_t ring, FenceReservation& reservation) noexcept;
    // Second half, after the frame is written and before the WPTR/doorbell is
    // published: sync_seq = sequence; position is the ring write pointer (dwords)
    // after the frame; drm_sched_job_begin re-arms the job timeout and the job
    // fence's enable_signaling arms the fallback timer when idle. ring_hung or
    // invalid_argument (sequence is not sync_seq + 1) means: do not publish.
    FenceStatus publish_submission(uint32_t ring, uint32_t sequence, uint64_t position) noexcept;
    // Quarantines an attached ring (e.g. a failed doorbell after publish).
    FenceStatus mark_hung(uint32_t ring, FenceHangReason reason) noexcept;

    // signalled, pending, ring_hung (seq not signalled on a hung ring),
    // unavailable or invalid_argument (sequence was never emitted). 32-bit
    // wrap order: sequence is signalled when last_seq - sequence < 2^31.
    FenceStatus query(uint32_t ring, uint32_t sequence) noexcept;

    // dma_fence_wait_timeout at PASSIVE_LEVEL: signalled, timed_out, ring_hung,
    // busy (no waiter slot), unavailable or invalid_argument. Blocks in slices
    // of at most fence_fallback_timeout_us and re-reads the writeback after each
    // (amdgpu_fence_process), also checking the ring's job timeout.
    FenceStatus wait(uint32_t ring, uint32_t sequence, uint64_t timeout_us) noexcept;

    // The caller-driven fallback timer and TDR: for every active ring, a due
    // fallback timer runs amdgpu_fence_process once (Linux amdgpu_fence_fallback),
    // and a due job timeout re-reads the fence once and then quarantines the
    // ring (amdgpu_job_timedout without soft recovery, ring reset or GPU
    // recovery). Call at least every fence_fallback_timeout_us while
    // next_deadline_us reports work.
    void poll() noexcept;
    // Earliest armed fallback/timeout deadline, or UINT64_MAX when none.
    uint64_t next_deadline_us() noexcept;

    // IrqService drain consumer (DPC): EOP/trap counters of attached rings
    // (gfx_v11_0_eop_irq, sdma_v6_0_process_trap_irq), every ring after lost
    // vectors, CP faults (drm_sched_fault -> immediate timeout -> quarantine) and
    // VM faults (rate-limited report; VMID 0 GFXHUB faults quarantine the kernel
    // rings their UTCL2 client id names).
    void process_interrupts(const IrqDispatchView& view) noexcept;
    IrqConsumer consumer() noexcept { return {this, consume_trampoline}; }

    // After a confirmed reset: detaches every ring (hung or not), wakes all
    // waiters and clears reset_required. quarantined rings stay until then.
    FenceStatus release_after_reset(FenceRecovery recovery) noexcept;

    FenceReport report() noexcept;
    FenceRingReport ring_report(uint32_t ring) noexcept;

private:
    // Per ring state beyond its report_.rings entry: the fence dword view, the
    // ring position each outstanding sequence's frame ends at (indexed by
    // sequence % fence_slots, like fence_drv.fences) and an attach generation
    // that lets a waiter notice a detach/re-attach while it slept.
    struct Ring final {
        ByteView writeback{nullptr, 0};
        uint64_t end_position[fence_slots]{};
        uint32_t generation = 0;
    };

    struct Waiter final {
        bool used;
        uint32_t ring;
        uint32_t sequence;
    };

    static void consume_trampoline(void* context, const IrqDispatchView& view) noexcept;
    static bool valid_sequence(const FenceRingReport& ring, uint32_t sequence) noexcept;
    static bool is_signalled(const FenceRingReport& ring, uint32_t sequence) noexcept;
    bool valid_identity(uint32_t ring, const FenceRingConfig& config) const noexcept;
    bool process_ring(uint32_t ring, uint64_t now) noexcept;
    void check_timeout(uint32_t ring, uint64_t now) noexcept;
    void hang(uint32_t ring, FenceHangReason reason, uint64_t now) noexcept;
    void wake(uint32_t ring) noexcept;
    void arm_fallback(FenceRingReport& ring, uint64_t now) noexcept;
    void detach(uint32_t ring) noexcept;
    bool interrupt_seen(const IrqReport& report, const FenceRingReport& ring) const noexcept;
    void remember_interrupts(const IrqReport& report) noexcept;
    void consume_cp_faults(const IrqReport& report, uint64_t now) noexcept;
    void hang_vm_engine(FenceEngine engine, uint32_t instance, bool all, uint64_t now,
                        uint32_t& matched) noexcept;
    void consume_vm_fault(const IrqVmFault& fault, uint64_t now) noexcept;
    bool ratelimit(uint64_t now, uint32_t& suppressed) noexcept;
    void lock() noexcept { platform_.lock(platform_.context); }
    void unlock() noexcept { platform_.unlock(platform_.context); }

    FencePlatform platform_{};
    bool initialized_ = false;
    FenceReport report_{};
    Ring rings_[fence_max_rings]{};
    Waiter waiters_[fence_waiter_slots]{};
    // IrqService counters consumed so far (all sources, attached or not), so a
    // ring attached later reacts only to newer records.
    uint32_t gfx_eop_seen_[irq_gfx_pipes]{};
    uint32_t compute_eop_seen_[irq_mec_count][irq_mec_pipes][irq_mec_queues]{};
    uint32_t sdma_trap_seen_[irq_sdma_instances]{};
    uint32_t cp_fault_seen_[irq_cp_me_count][irq_mec_pipes][irq_mec_queues]{};
    uint32_t cp_invalid_me_seen_ = 0;
    uint32_t vm_fault_seen_ = 0;
    // ___ratelimit state of the fault report.
    bool ratelimit_started_ = false;
    uint64_t ratelimit_begin_us_ = 0;
    uint32_t ratelimit_printed_ = 0;
    uint32_t ratelimit_missed_ = 0;
};

} // namespace rx7900::hardware

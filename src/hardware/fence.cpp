/*
Module: rx7900.hardware.fence
Purpose: Track emitted and signalled kernel-ring fences, wake waiters, detect job
lockups and route CP/VM faults to ring quarantine.
Created: 2026-10-22
Architecture: Portable state under one native lock; writeback reads through
ByteView; time, events and logging through FencePlatform. No hardware writes.
Copyright 2009 Jerome Glisse; Copyright 2015, 2016, 2021, 2022 Advanced Micro
Devices, Inc. Adapted Linux v6.12 amdgpu_fence.c, amdgpu.h, amdgpu_drv.c,
amdgpu_device.c, amdgpu_job.c, drm scheduler sched_main.c, gfx_v11_0.c,
sdma_v6_0.c, gmc_v11_0.c, gfxhub_v3_0.c, gc_11_0_0_sh_mask.h, printk.c and
ratelimit.c. Notice and provenance: licenses/AMDGPU-SDMA.txt.
*/
#include "fence.hpp"

namespace rx7900::hardware {
namespace {

// Sequence ordering of amdgpu_fence_process / dma_fence seqno comparisons:
// a precedes b when b - a is below 2^31.
constexpr uint32_t sequence_half_range = 0x80000000u;

// gc_11_0_0_sh_mask.h GCVM_L2_PROTECTION_FAULT_STATUS CID (bits 17:9) and VMID
// (bits 23:20); MMVM_L2_PROTECTION_FAULT_STATUS uses the same positions.
constexpr uint32_t fault_status_cid_shift = 9;
constexpr uint32_t fault_status_cid_mask = 0x1ffu;
constexpr uint32_t fault_status_vmid_shift = 20;
constexpr uint32_t fault_status_vmid_mask = 0xfu;

uint64_t saturating_add(uint64_t a, uint64_t b) noexcept {
    return a > UINT64_MAX - b ? UINT64_MAX : a + b;
}

FenceEngine engine_of(uint32_t ring) noexcept {
    return ring == fence_ring_gfx ? FenceEngine::gfx :
           ring < fence_ring_sdma0 ? FenceEngine::compute : FenceEngine::sdma;
}

uint32_t default_timeout_ms(FenceEngine engine) noexcept {
    return engine == FenceEngine::compute ? fence_compute_timeout_ms :
           engine == FenceEngine::gfx ? fence_gfx_timeout_ms : fence_sdma_timeout_ms;
}

FenceHangReason cp_reason(IrqSource source) noexcept {
    return source == IrqSource::cp_priv_reg ? FenceHangReason::cp_priv_reg :
           source == IrqSource::cp_priv_instr ? FenceHangReason::cp_priv_instr :
                                                FenceHangReason::cp_bad_opcode;
}

} // namespace

FenceStatus FenceManager::initialize(FencePlatform platform) noexcept {
    if (initialized_) {
        return FenceStatus::invalid_argument;
    }

    if (platform.lock == nullptr || platform.unlock == nullptr || platform.now_us == nullptr ||
        platform.signal == nullptr || platform.reset == nullptr || platform.block == nullptr) {
        return FenceStatus::invalid_argument;
    }

    platform_ = platform;
    initialized_ = true;
    report_.status = FenceStatus::ready;
    return FenceStatus::ready;
}

bool FenceManager::valid_sequence(const FenceRingReport& ring, uint32_t sequence) noexcept {
    // Emitted: not after sync_seq in wrap order.
    return ring.sync_seq - sequence < sequence_half_range;
}

bool FenceManager::is_signalled(const FenceRingReport& ring, uint32_t sequence) noexcept {
    return ring.last_seq - sequence < sequence_half_range;
}

bool FenceManager::valid_identity(uint32_t ring, const FenceRingConfig& config) const noexcept {
    if (config.engine != engine_of(ring) || config.timeout_ms > fence_max_timeout_ms) {
        return false;
    }

    switch (config.engine) {
    case FenceEngine::gfx:
        if (config.me != 0 || config.pipe >= irq_gfx_pipes || config.queue >= irq_mec_queues) {
            return false;
        }
        break;
    case FenceEngine::compute:
        if (config.me == 0 || config.me > irq_mec_count || config.pipe >= irq_mec_pipes ||
            config.queue >= irq_mec_queues) {
            return false;
        }
        break;
    case FenceEngine::sdma:
        if (config.instance != ring - fence_ring_sdma0) {
            return false;
        }
        break;
    }

    for (uint32_t other = 0; other < fence_max_rings; ++other) {
        const FenceRingReport& info = report_.rings[other];

        if (other == ring || info.state == FenceRingState::detached || info.engine != config.engine) {
            continue;
        }

        if (config.engine == FenceEngine::sdma ? info.instance == config.instance :
            info.me == config.me && info.pipe == config.pipe && info.queue == config.queue) {
            return false;
        }
    }

    return true;
}

FenceStatus FenceManager::attach_ring(uint32_t ring, const FenceRingConfig& config) noexcept {
    if (!initialized_) {
        return FenceStatus::unavailable;
    }

    if (ring >= fence_max_rings) {
        return FenceStatus::invalid_argument;
    }

    lock();
    FenceRingReport& info = report_.rings[ring];
    uint32_t value = 0;
    FenceStatus status = FenceStatus::attached;

    if (info.state != FenceRingState::detached) {
        status = info.state == FenceRingState::hung ? FenceStatus::ring_hung : FenceStatus::busy;
    } else if (!valid_identity(ring, config) || config.fence_writeback.size() < sizeof(uint32_t) ||
               !config.fence_writeback.read_u32(0, value) || value != config.initial_seq) {
        // amdgpu_fence_driver_start_ring writes last_seq to the writeback; the
        // queue module did so, and anything else is not a known fence state.
        status = FenceStatus::invalid_argument;
    } else {
        Ring& state = rings_[ring];
        const uint32_t generation = state.generation + 1;
        info = {};
        info.state = FenceRingState::active;
        info.engine = config.engine;
        info.me = config.me;
        info.pipe = config.pipe;
        info.queue = config.queue;
        info.instance = config.instance;
        info.timeout_ms = config.timeout_ms != 0 ? config.timeout_ms :
                                                   default_timeout_ms(config.engine);
        info.sync_seq = config.initial_seq;
        info.last_seq = config.initial_seq;
        info.retired_position = config.initial_position;
        state = {};
        state.writeback = config.fence_writeback;
        state.generation = generation;

        for (auto& position : state.end_position) {
            position = config.initial_position;
        }
    }

    unlock();
    return status;
}

// Caller holds the lock.
void FenceManager::detach(uint32_t ring) noexcept {
    Ring& state = rings_[ring];
    const uint32_t generation = state.generation + 1;
    state = {};
    state.generation = generation;
    report_.rings[ring] = {};
    report_.hung_rings &= ~(1u << ring);
    report_.reset_required = report_.hung_rings != 0;
    wake(ring);
}

FenceStatus FenceManager::detach_ring(uint32_t ring, bool reset_confirmed) noexcept {
    if (!initialized_) {
        return FenceStatus::unavailable;
    }

    if (ring >= fence_max_rings) {
        return FenceStatus::invalid_argument;
    }

    lock();
    const uint64_t now = platform_.now_us(platform_.context);
    FenceStatus status = FenceStatus::detached;
    process_ring(ring, now);
    const FenceRingReport& info = report_.rings[ring];

    if (info.state == FenceRingState::detached) {
        status = FenceStatus::unavailable;
    } else if (info.state == FenceRingState::hung && !reset_confirmed) {
        status = FenceStatus::ring_hung;
    } else if (info.state == FenceRingState::active && info.last_seq != info.sync_seq &&
               !reset_confirmed) {
        // The engine may still write the fence dword or fetch the ring.
        status = FenceStatus::pending;
    } else {
        detach(ring);
    }

    unlock();
    return status;
}

void FenceManager::wake(uint32_t ring) noexcept {
    for (uint32_t slot = 0; slot < fence_waiter_slots; ++slot) {
        if (waiters_[slot].used && waiters_[slot].ring == ring) {
            platform_.signal(platform_.context, slot);
        }
    }
}

// amdgpu_fence_enable_signaling: start the fallback timer unless pending.
void FenceManager::arm_fallback(FenceRingReport& ring, uint64_t now) noexcept {
    if (!ring.fallback_armed) {
        ring.fallback_armed = true;
        ring.fallback_deadline_us = saturating_add(now, fence_fallback_timeout_us);
    }
}

// amdgpu_job_timedout for a kernel ring, without coredump, soft recovery,
// per-ring reset or amdgpu_device_gpu_recover: the ring is quarantined and the
// explicit mode-1 reset is reported as required. Caller holds the lock.
void FenceManager::hang(uint32_t ring, FenceHangReason reason, uint64_t now) noexcept {
    FenceRingReport& info = report_.rings[ring];

    if (info.state != FenceRingState::active) {
        return;
    }

    info.state = FenceRingState::hung;
    info.hang_reason = reason;
    info.hang_signalled_seq = info.last_seq;
    info.hang_emitted_seq = info.sync_seq;
    info.hang_time_us = now;
    info.timeout_armed = false;
    info.fallback_armed = false;
    report_.hung_rings |= 1u << ring;
    report_.reset_required = true;

    if (platform_.report_hang != nullptr) {
        platform_.report_hang(platform_.context, ring, reason, info.last_seq, info.sync_seq);
    }

    wake(ring);
}

// amdgpu_fence_process for one ring; returns true when last_seq advanced.
// Caller holds the lock.
bool FenceManager::process_ring(uint32_t ring, uint64_t now) noexcept {
    FenceRingReport& info = report_.rings[ring];

    if (info.state != FenceRingState::active) {
        return false;
    }

    uint32_t value = 0;

    if (!rings_[ring].writeback.read_u32(0, value)) {
        hang(ring, FenceHangReason::writeback_unreadable, now);
        return false;
    }

    // Linux stores whatever it reads. A value outside (last_seq, sync_seq] in
    // wrap order is no fence this ring emitted: the writeback or engine state is
    // unknown, so the ring is quarantined instead of signalling anything.
    if (value - info.last_seq > info.sync_seq - info.last_seq) {
        hang(ring, FenceHangReason::fence_out_of_window, now);
        return false;
    }

    // del_timer(&fallback_timer) && seq != sync_seq: reschedule the fallback.
    if (info.fallback_armed) {
        info.fallback_armed = false;

        if (value != info.sync_seq) {
            arm_fallback(info, now);
        }
    }

    if (value == info.last_seq) {
        return false;
    }

    info.signalled += value - info.last_seq;
    info.last_seq = value;
    // Outstanding sequences never exceed fence_slots, so this slot still holds
    // the end of value's frame: the engine consumed the ring up to there.
    info.retired_position = rings_[ring].end_position[value % fence_slots];

    // drm_sched_get_finished_job: cancel the job timeout and restart it for the
    // next pending job, or leave it cancelled when the ring is idle.
    if (value != info.sync_seq) {
        info.timeout_armed = true;
        info.timeout_deadline_us = saturating_add(now, uint64_t{info.timeout_ms} * 1000u);
    } else {
        info.timeout_armed = false;
    }

    wake(ring);
    return true;
}

// drm_sched_job_timedout once the work_tdr deadline passed; the fence is read
// once more first, so a job that just completed is not reported. Caller holds
// the lock.
void FenceManager::check_timeout(uint32_t ring, uint64_t now) noexcept {
    FenceRingReport& info = report_.rings[ring];

    if (info.state != FenceRingState::active || !info.timeout_armed ||
        now < info.timeout_deadline_us) {
        return;
    }

    process_ring(ring, now);

    if (info.state == FenceRingState::active && info.timeout_armed &&
        now >= info.timeout_deadline_us) {
        hang(ring, FenceHangReason::job_timeout, now);
    }
}

FenceStatus FenceManager::begin_submission(uint32_t ring, FenceReservation& reservation) noexcept {
    reservation = {};

    if (!initialized_) {
        return FenceStatus::unavailable;
    }

    if (ring >= fence_max_rings) {
        return FenceStatus::invalid_argument;
    }

    lock();
    const uint64_t now = platform_.now_us(platform_.context);
    process_ring(ring, now);
    check_timeout(ring, now);
    const FenceRingReport& info = report_.rings[ring];
    FenceStatus status = FenceStatus::ready;

    if (info.state != FenceRingState::active) {
        status = info.state == FenceRingState::hung ? FenceStatus::ring_hung :
                                                      FenceStatus::unavailable;
    } else if (info.sync_seq - info.last_seq >= fence_slots) {
        status = FenceStatus::busy;
    } else {
        reservation = {info.sync_seq + 1, info.retired_position, info.sync_seq - info.last_seq};
    }

    unlock();
    return status;
}

FenceStatus FenceManager::publish_submission(uint32_t ring, uint32_t sequence,
                                             uint64_t position) noexcept {
    if (!initialized_) {
        return FenceStatus::unavailable;
    }

    if (ring >= fence_max_rings) {
        return FenceStatus::invalid_argument;
    }

    lock();
    FenceRingReport& info = report_.rings[ring];
    Ring& state = rings_[ring];
    FenceStatus status = FenceStatus::pending;

    if (info.state != FenceRingState::active) {
        status = info.state == FenceRingState::hung ? FenceStatus::ring_hung :
                                                      FenceStatus::unavailable;
    } else if (sequence != info.sync_seq + 1 || info.sync_seq - info.last_seq >= fence_slots ||
               position <= state.end_position[info.sync_seq % fence_slots]) {
        status = FenceStatus::invalid_argument;
    } else {
        const uint64_t now = platform_.now_us(platform_.context);
        info.sync_seq = sequence;
        state.end_position[sequence % fence_slots] = position;
        ++info.submissions;
        // drm_sched_job_begin -> drm_sched_start_timeout: mod_delayed_work
        // re-arms the TDR for every job that starts.
        info.timeout_armed = true;
        info.timeout_deadline_us = saturating_add(now, uint64_t{info.timeout_ms} * 1000u);
        // The scheduler's completion callback enables signaling on the fence.
        arm_fallback(info, now);
    }

    unlock();
    return status;
}

FenceStatus FenceManager::mark_hung(uint32_t ring, FenceHangReason reason) noexcept {
    if (!initialized_) {
        return FenceStatus::unavailable;
    }

    if (ring >= fence_max_rings || reason == FenceHangReason::none) {
        return FenceStatus::invalid_argument;
    }

    lock();
    hang(ring, reason, platform_.now_us(platform_.context));
    const FenceStatus status = report_.rings[ring].state == FenceRingState::hung ?
        FenceStatus::ring_hung : FenceStatus::unavailable;
    unlock();
    return status;
}

FenceStatus FenceManager::query(uint32_t ring, uint32_t sequence) noexcept {
    if (!initialized_) {
        return FenceStatus::unavailable;
    }

    if (ring >= fence_max_rings) {
        return FenceStatus::invalid_argument;
    }

    lock();
    const uint64_t now = platform_.now_us(platform_.context);
    process_ring(ring, now);
    check_timeout(ring, now);
    const FenceRingReport& info = report_.rings[ring];
    FenceStatus status = FenceStatus::pending;

    if (info.state == FenceRingState::detached) {
        status = FenceStatus::unavailable;
    } else if (!valid_sequence(info, sequence)) {
        status = FenceStatus::invalid_argument;
    } else if (is_signalled(info, sequence)) {
        status = FenceStatus::signalled;
    } else if (info.state == FenceRingState::hung) {
        status = FenceStatus::ring_hung;
    }

    unlock();
    return status;
}

FenceStatus FenceManager::wait(uint32_t ring, uint32_t sequence, uint64_t timeout_us) noexcept {
    if (!initialized_) {
        return FenceStatus::unavailable;
    }

    if (ring >= fence_max_rings) {
        return FenceStatus::invalid_argument;
    }

    lock();
    const uint64_t start = platform_.now_us(platform_.context);
    const uint64_t deadline = saturating_add(start, timeout_us);
    process_ring(ring, start);
    check_timeout(ring, start);
    FenceRingReport& info = report_.rings[ring];
    const uint32_t generation = rings_[ring].generation;
    FenceStatus status = FenceStatus::pending;
    uint32_t slot = fence_waiter_slots;

    if (info.state == FenceRingState::detached) {
        status = FenceStatus::unavailable;
    } else if (!valid_sequence(info, sequence)) {
        status = FenceStatus::invalid_argument;
    } else if (is_signalled(info, sequence)) {
        status = FenceStatus::signalled;
    } else if (info.state == FenceRingState::hung) {
        status = FenceStatus::ring_hung;
    } else if (timeout_us == 0) {
        status = FenceStatus::timed_out;
    } else {
        for (slot = 0; slot < fence_waiter_slots && waiters_[slot].used; ++slot) {
        }

        if (slot == fence_waiter_slots) {
            ++report_.waiter_exhaustion;
            status = FenceStatus::busy;
        } else {
            waiters_[slot] = {true, ring, sequence};
            // A stale signal from the slot's previous waiter must not count.
            platform_.reset(platform_.context, slot);
            arm_fallback(info, start);
        }
    }

    unlock();

    while (status == FenceStatus::pending) {
        const uint64_t now = platform_.now_us(platform_.context);

        if (now >= deadline) {
            status = FenceStatus::timed_out;
            break;
        }

        const uint64_t remaining = deadline - now;
        platform_.block(platform_.context, slot,
                        remaining < fence_fallback_timeout_us ? remaining : fence_fallback_timeout_us);
        lock();
        const uint64_t woke = platform_.now_us(platform_.context);

        if (rings_[ring].generation != generation ||
            report_.rings[ring].state == FenceRingState::detached) {
            status = FenceStatus::unavailable;
        } else {
            process_ring(ring, woke);
            check_timeout(ring, woke);

            if (is_signalled(report_.rings[ring], sequence)) {
                status = FenceStatus::signalled;
            } else if (report_.rings[ring].state == FenceRingState::hung) {
                status = FenceStatus::ring_hung;
            }
        }

        unlock();
    }

    if (slot < fence_waiter_slots) {
        lock();
        waiters_[slot] = {};
        unlock();
    }

    return status;
}

void FenceManager::poll() noexcept {
    if (!initialized_) {
        return;
    }

    lock();
    const uint64_t now = platform_.now_us(platform_.context);

    for (uint32_t ring = 0; ring < fence_max_rings; ++ring) {
        FenceRingReport& info = report_.rings[ring];

        // amdgpu_fence_fallback: the timer fired (no longer pending, so
        // amdgpu_fence_process does not reschedule it).
        if (info.state == FenceRingState::active && info.fallback_armed &&
            now >= info.fallback_deadline_us) {
            info.fallback_armed = false;
            ++info.fallback_expiries;

            if (process_ring(ring, now)) {
                ++info.fallback_progress;
            }
        }

        check_timeout(ring, now);
    }

    unlock();
}

uint64_t FenceManager::next_deadline_us() noexcept {
    if (!initialized_) {
        return UINT64_MAX;
    }

    uint64_t next = UINT64_MAX;
    lock();

    for (const auto& info : report_.rings) {
        if (info.state != FenceRingState::active) {
            continue;
        }

        if (info.fallback_armed && info.fallback_deadline_us < next) {
            next = info.fallback_deadline_us;
        }

        if (info.timeout_armed && info.timeout_deadline_us < next) {
            next = info.timeout_deadline_us;
        }
    }

    unlock();
    return next;
}

bool FenceManager::interrupt_seen(const IrqReport& report, const FenceRingReport& ring) const noexcept {
    switch (ring.engine) {
    case FenceEngine::gfx: {
        // gfx_v11_0_eop_irq: pipe 0 -> gfx_ring[0], any other -> gfx_ring[1].
        const uint32_t pipe = ring.pipe == 0 ? 0 : 1;
        return report.gfx_eop[pipe] != gfx_eop_seen_[pipe];
    }
    case FenceEngine::compute:
        return report.compute_eop[ring.me - 1][ring.pipe][ring.queue] !=
               compute_eop_seen_[ring.me - 1][ring.pipe][ring.queue];
    case FenceEngine::sdma:
        return report.sdma_trap[ring.instance] != sdma_trap_seen_[ring.instance];
    }

    return false;
}

void FenceManager::remember_interrupts(const IrqReport& report) noexcept {
    for (uint32_t pipe = 0; pipe < irq_gfx_pipes; ++pipe) {
        gfx_eop_seen_[pipe] = report.gfx_eop[pipe];
    }

    for (uint32_t me = 0; me < irq_mec_count; ++me) {
        for (uint32_t pipe = 0; pipe < irq_mec_pipes; ++pipe) {
            for (uint32_t queue = 0; queue < irq_mec_queues; ++queue) {
                compute_eop_seen_[me][pipe][queue] = report.compute_eop[me][pipe][queue];
            }
        }
    }

    for (uint32_t instance = 0; instance < irq_sdma_instances; ++instance) {
        sdma_trap_seen_[instance] = report.sdma_trap[instance];
    }
}

// gfx_v11_0_handle_priv_fault: drm_sched_fault on the ring whose me/pipe/queue
// match, which runs the timeout handler at once. Caller holds the lock.
void FenceManager::consume_cp_faults(const IrqReport& report, uint64_t now) noexcept {
    for (uint32_t me = 0; me < irq_cp_me_count; ++me) {
        for (uint32_t pipe = 0; pipe < irq_mec_pipes; ++pipe) {
            for (uint32_t queue = 0; queue < irq_mec_queues; ++queue) {
                const uint32_t count = report.cp_fault_count[me][pipe][queue];
                const uint32_t fresh = count - cp_fault_seen_[me][pipe][queue];

                if (fresh == 0) {
                    continue;
                }

                cp_fault_seen_[me][pipe][queue] = count;
                const FenceEngine engine = me == 0 ? FenceEngine::gfx : FenceEngine::compute;
                bool matched = false;

                for (uint32_t ring = 0; ring < fence_max_rings; ++ring) {
                    FenceRingReport& info = report_.rings[ring];

                    if (info.state == FenceRingState::detached || info.engine != engine ||
                        info.me != me || info.pipe != pipe || info.queue != queue) {
                        continue;
                    }

                    matched = true;
                    info.cp_faults += fresh;
                    hang(ring, cp_reason(report.cp_fault_source[me][pipe][queue]), now);
                }

                if (!matched) {
                    report_.cp_faults_unattributed += fresh;
                }
            }
        }
    }

    report_.cp_faults_unattributed += report.cp_fault_invalid_me - cp_invalid_me_seen_;
    cp_invalid_me_seen_ = report.cp_fault_invalid_me;
}

// printk_ratelimit() with printk_ratelimit_state (5 s, burst 10), as ___ratelimit.
bool FenceManager::ratelimit(uint64_t now, uint32_t& suppressed) noexcept {
    suppressed = 0;

    if (!ratelimit_started_) {
        ratelimit_started_ = true;
        ratelimit_begin_us_ = now;
    }

    // time_is_before_jiffies(begin + interval).
    if (now > saturating_add(ratelimit_begin_us_, fence_report_interval_us)) {
        suppressed = ratelimit_missed_;
        ratelimit_missed_ = 0;
        ratelimit_begin_us_ = now;
        ratelimit_printed_ = 0;
    }

    if (ratelimit_printed_ < fence_report_burst) {
        ++ratelimit_printed_;
        return true;
    }

    ++ratelimit_missed_;
    return false;
}

// Quarantines every attached ring of the engine (all) or the one instance/ring.
void FenceManager::hang_vm_engine(FenceEngine engine, uint32_t instance, bool all, uint64_t now,
                                  uint32_t& matched) noexcept {
    for (uint32_t ring = 0; ring < fence_max_rings; ++ring) {
        FenceRingReport& info = report_.rings[ring];

        if (info.state == FenceRingState::detached || info.engine != engine ||
            (!all && engine == FenceEngine::sdma && info.instance != instance)) {
            continue;
        }

        ++matched;
        ++info.vm_faults;
        hang(ring, FenceHangReason::vm_fault, now);
    }
}

// gmc_v11_0_process_interrupt after IrqService read and cleared the status:
// a rate-limited report and nothing else for process VMIDs (vm_fault_stop
// NEVER). VMID 0 is the kernel context every ring here uses; a GFXHUB fault
// there quarantines the rings its UTCL2 client id names, or every attached
// kernel ring when the client is unknown. MMHUB VMID 0 clients (IH, display,
// multimedia, PSP) are no gfx/compute/SDMA kernel ring and are only counted.
void FenceManager::consume_vm_fault(const IrqVmFault& fault, uint64_t now) noexcept {
    ++report_.vm_faults;
    report_.last_vm_fault = fault;
    uint32_t suppressed = 0;

    if (ratelimit(now, suppressed)) {
        ++report_.vm_faults_printed;

        if (platform_.report_vm_fault != nullptr) {
            platform_.report_vm_fault(platform_.context, fault, suppressed);
        }
    } else {
        ++report_.vm_faults_suppressed;
    }

    // The status register latches the first fault since the last clear; a
    // latched VMID 0 makes the fault kernel context even if the IV says otherwise.
    const bool status_known = fault.status_read && fault.status != 0;
    const bool kernel = fault.vmid == 0 ||
        (status_known && ((fault.status >> fault_status_vmid_shift) & fault_status_vmid_mask) == 0);

    if (!kernel) {
        ++report_.vm_faults_process;
        return;
    }

    ++report_.vm_faults_vmid0;
    uint32_t matched = 0;

    if (fault.register_hub == VmHub::gfxhub) {
        const uint32_t cid = (fault.status >> fault_status_cid_shift) & fault_status_cid_mask;

        if (status_known && cid == fence_utcl2_cid_cpg) {
            hang_vm_engine(FenceEngine::gfx, 0, true, now, matched);
        } else if (status_known && cid == fence_utcl2_cid_cpc) {
            hang_vm_engine(FenceEngine::compute, 0, true, now, matched);
        } else if (status_known && cid == fence_utcl2_cid_cpf) {
            hang_vm_engine(FenceEngine::gfx, 0, true, now, matched);
            hang_vm_engine(FenceEngine::compute, 0, true, now, matched);
        } else if (status_known && (cid == fence_utcl2_cid_sdma0 || cid == fence_utcl2_cid_sdma1)) {
            hang_vm_engine(FenceEngine::sdma, cid - fence_utcl2_cid_sdma0, false, now, matched);
        } else {
            hang_vm_engine(FenceEngine::gfx, 0, true, now, matched);
            hang_vm_engine(FenceEngine::compute, 0, true, now, matched);
            hang_vm_engine(FenceEngine::sdma, 0, true, now, matched);
        }
    }

    if (matched == 0) {
        ++report_.vm_faults_vmid0_unattributed;
    }
}

void FenceManager::process_interrupts(const IrqDispatchView& view) noexcept {
    if (!initialized_ || view.report == nullptr || view.vm_faults == nullptr) {
        return;
    }

    const IrqReport& irq = *view.report;
    lock();
    const uint64_t now = platform_.now_us(platform_.context);

    // Fence progress first, as the EOPs of completed work precede a later fault.
    for (uint32_t ring = 0; ring < fence_max_rings; ++ring) {
        FenceRingReport& info = report_.rings[ring];

        if (info.state == FenceRingState::active && (view.events_lost || interrupt_seen(irq, info))) {
            ++info.interrupt_processes;
            process_ring(ring, now);
        }
    }

    remember_interrupts(irq);

    if (view.events_lost) {
        ++report_.lost_event_rescans;
    }

    consume_cp_faults(irq, now);
    const IrqVmFaultLog& log = *view.vm_faults;
    uint32_t fresh = log.total - vm_fault_seen_;

    if (fresh > irq_vm_fault_log_entries) {
        report_.vm_faults_lost += fresh - irq_vm_fault_log_entries;
        fresh = irq_vm_fault_log_entries;
    }

    for (uint32_t number = log.total - fresh; number != log.total; ++number) {
        consume_vm_fault(log.entries[number % irq_vm_fault_log_entries], now);
    }

    vm_fault_seen_ = log.total;

    for (uint32_t ring = 0; ring < fence_max_rings; ++ring) {
        check_timeout(ring, now);
    }

    unlock();
}

void FenceManager::consume_trampoline(void* context, const IrqDispatchView& view) noexcept {
    static_cast<FenceManager*>(context)->process_interrupts(view);
}

FenceStatus FenceManager::release_after_reset(FenceRecovery recovery) noexcept {
    if (!initialized_) {
        return FenceStatus::unavailable;
    }

    if (recovery.confirm_reset_quiescence == nullptr ||
        !recovery.confirm_reset_quiescence(recovery.context)) {
        lock();
        const bool required = report_.reset_required;
        unlock();
        return required ? FenceStatus::ring_hung : FenceStatus::busy;
    }

    lock();

    for (uint32_t ring = 0; ring < fence_max_rings; ++ring) {
        if (report_.rings[ring].state != FenceRingState::detached) {
            detach(ring);
        }
    }

    report_.hung_rings = 0;
    report_.reset_required = false;
    unlock();
    return FenceStatus::released;
}

FenceReport FenceManager::report() noexcept {
    if (!initialized_) {
        return report_;
    }

    lock();
    const FenceReport copy = report_;
    unlock();
    return copy;
}

FenceRingReport FenceManager::ring_report(uint32_t ring) noexcept {
    if (!initialized_ || ring >= fence_max_rings) {
        return {};
    }

    lock();
    const FenceRingReport copy = report_.rings[ring];
    unlock();
    return copy;
}

} // namespace rx7900::hardware

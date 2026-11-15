/*
Module: rx7900.tests.fence
Purpose: Verify FenceManager ordering and wrap, interrupt-driven and fallback
completion, waits, job timeouts, CP/VM fault routing, quarantine and release.
Created: 2026-10-22
Architecture: Test-local platform model (lock discipline, controllable clock,
waiter events with a block hook) and synthetic IrqDispatchView records; expected
values are hand-derived from Linux v6.12 amdgpu_fence.c, amdgpu_job.c,
sched_main.c, amdgpu_device.c, gfx_v11_0.c, sdma_v6_0.c, gmc_v11_0.c,
gfxhub_v3_0.c and lib/ratelimit.c. Not hardware evidence.
*/
#include <cstdio>
#include <functional>
#include <vector>

#include "asic_test_device.hpp"
#include "fence.hpp"

namespace {

using namespace rx7900::hardware;
using rx7900::ByteView;

rx7900_test::Checks checks;

void expect(bool condition, const char* contract) { checks.expect(condition, contract); }

constexpr uint64_t ms = 1000;
constexpr uint64_t second = 1000000;

// Platform model. block advances the clock by the requested slice (or by step
// when nonzero) and runs on_block without the lock held.
struct Platform final {
    uint64_t now = 1000;
    bool held = false;
    unsigned lock_errors = 0;
    unsigned locks = 0;
    std::vector<uint32_t> signals;
    std::vector<uint32_t> resets;
    bool event[fence_waiter_slots]{};
    uint64_t step = 0;
    unsigned blocks = 0;
    std::function<void()> on_block;
    std::vector<IrqVmFault> printed;
    std::vector<uint32_t> suppressed;
    std::vector<uint32_t> hung;
    std::vector<FenceHangReason> reasons;

    FencePlatform platform() {
        return {this, lock, unlock, clock, signal, reset, block, vm_fault, hang};
    }

    static Platform& self(void* context) { return *static_cast<Platform*>(context); }

    static void lock(void* context) noexcept {
        auto& p = self(context);
        p.lock_errors += p.held ? 1 : 0;
        p.held = true;
        ++p.locks;
    }

    static void unlock(void* context) noexcept {
        auto& p = self(context);
        p.lock_errors += p.held ? 0 : 1;
        p.held = false;
    }

    static uint64_t clock(void* context) noexcept { return self(context).now; }

    static void signal(void* context, uint32_t slot) noexcept {
        auto& p = self(context);
        p.lock_errors += p.held ? 0 : 1;
        p.signals.push_back(slot);
        p.event[slot] = true;
    }

    static void reset(void* context, uint32_t slot) noexcept {
        auto& p = self(context);
        p.resets.push_back(slot);
        p.event[slot] = false;
    }

    static bool block(void* context, uint32_t slot, uint64_t timeout) noexcept {
        auto& p = self(context);
        p.lock_errors += p.held ? 1 : 0;
        ++p.blocks;

        if (p.on_block) {
            p.on_block();
        }

        if (p.event[slot]) {
            p.event[slot] = false;
            return true;
        }

        p.now += p.step != 0 && p.step < timeout ? p.step : timeout;
        return false;
    }

    static void vm_fault(void* context, const IrqVmFault& fault, uint32_t suppressed) noexcept {
        self(context).printed.push_back(fault);
        self(context).suppressed.push_back(suppressed);
    }

    static void hang(void* context, uint32_t ring, FenceHangReason reason, uint32_t,
                     uint32_t) noexcept {
        self(context).hung.push_back(ring);
        self(context).reasons.push_back(reason);
    }
};

// One fence dword per ring.
struct Writeback final {
    uint8_t bytes[fence_max_rings][4]{};

    void set(uint32_t ring, uint32_t value) {
        rx7900::sdma::CommandBuffer(bytes[ring], 4).write_u32(0, value);
    }

    ByteView view(uint32_t ring) const { return {bytes[ring], 4}; }
};

struct Fixture final {
    Platform platform;
    Writeback writeback;
    FenceManager fences;
    IrqReport irq{};
    IrqVmFaultLog vm{};

    Fixture() { fences.initialize(platform.platform()); }

    // The gfx ring (me 0 pipe 0 queue 0), compute ring i (MEC1, pipe i % 4,
    // queue i / 4 as Gfx11Queues assigns them) or an SDMA instance.
    FenceStatus attach(uint32_t ring, uint32_t seq = 0, uint64_t position = 0,
                       uint32_t timeout_ms = 0) {
        writeback.set(ring, seq);
        FenceRingConfig config{FenceEngine::gfx, 0, 0, 0, 0, writeback.view(ring), seq, position,
                               timeout_ms};

        if (ring >= fence_ring_sdma0) {
            config.engine = FenceEngine::sdma;
            config.instance = ring - fence_ring_sdma0;
        } else if (ring >= fence_ring_compute0) {
            const uint32_t index = ring - fence_ring_compute0;
            config.engine = FenceEngine::compute;
            config.me = 1;
            config.pipe = index % 4;
            config.queue = index / 4;
        }

        return fences.attach_ring(ring, config);
    }

    // begin + publish with a 256-dword frame; returns the sequence or 0 on refusal.
    FenceStatus submit(uint32_t ring, uint32_t& seq) {
        FenceReservation reservation{};
        const FenceStatus begun = fences.begin_submission(ring, reservation);

        if (begun != FenceStatus::ready) {
            return begun;
        }

        seq = reservation.sequence;
        return fences.publish_submission(ring, seq, position_after(ring, reservation));
    }

    uint64_t position_after(uint32_t ring, const FenceReservation& reservation) {
        const FenceRingReport info = fences.ring_report(ring);
        (void)reservation;
        return 256 * (static_cast<uint64_t>(info.submissions) + 1);
    }

    void dispatch(bool lost = false) { fences.process_interrupts({&irq, &vm, lost}); }

    void add_vm_fault(const IrqVmFault& fault) {
        ++vm.total;
        vm.entries[(vm.total - 1) % irq_vm_fault_log_entries] = fault;
    }

    FenceRingReport ring(uint32_t index) { return fences.ring_report(index); }
};

void initialize_contract() {
    FenceManager fences;
    Platform platform;
    FencePlatform missing = platform.platform();
    missing.block = nullptr;
    expect(fences.initialize(missing) == FenceStatus::invalid_argument, "every wait primitive required");
    Writeback writeback;
    const FenceRingConfig config{FenceEngine::gfx, 0, 0, 0, 0, writeback.view(0), 0, 0, 0};
    expect(fences.attach_ring(0, config) == FenceStatus::unavailable &&
           fences.wait(0, 0, 1) == FenceStatus::unavailable &&
           fences.next_deadline_us() == UINT64_MAX, "uninitialized manager refuses");
    expect(fences.initialize(platform.platform()) == FenceStatus::ready &&
           fences.initialize(platform.platform()) == FenceStatus::invalid_argument,
           "initialize once");
    FencePlatform optional = platform.platform();
    optional.report_hang = nullptr;
    optional.report_vm_fault = nullptr;
    FenceManager quiet;
    expect(quiet.initialize(optional) == FenceStatus::ready, "log sinks are optional");
}

void attach_contract() {
    Fixture f;
    f.writeback.set(0, 5);
    FenceRingConfig gfx{FenceEngine::gfx, 0, 0, 0, 0, f.writeback.view(0), 4, 0, 0};
    expect(f.fences.attach_ring(0, gfx) == FenceStatus::invalid_argument,
           "writeback must hold initial_seq (amdgpu_fence_driver_start_ring)");
    gfx.initial_seq = 5;
    gfx.engine = FenceEngine::compute;
    expect(f.fences.attach_ring(0, gfx) == FenceStatus::invalid_argument, "ring 0 is the gfx ring");
    gfx.engine = FenceEngine::gfx;
    gfx.me = 1;
    expect(f.fences.attach_ring(0, gfx) == FenceStatus::invalid_argument, "gfx is ME0");
    gfx.me = 0;
    gfx.fence_writeback = ByteView(f.writeback.bytes[0], 3);
    expect(f.fences.attach_ring(0, gfx) == FenceStatus::invalid_argument, "fence dword must fit");
    gfx.fence_writeback = f.writeback.view(0);
    gfx.timeout_ms = fence_max_timeout_ms + 1;
    expect(f.fences.attach_ring(0, gfx) == FenceStatus::invalid_argument, "bounded lockup timeout");
    gfx.timeout_ms = 0;
    expect(f.fences.attach_ring(fence_max_rings, gfx) == FenceStatus::invalid_argument,
           "ring index bounded");
    expect(f.fences.attach_ring(0, gfx) == FenceStatus::attached &&
           f.fences.attach_ring(0, gfx) == FenceStatus::busy, "attach once");
    const FenceRingReport info = f.ring(0);
    expect(info.state == FenceRingState::active && info.sync_seq == 5 && info.last_seq == 5 &&
           info.timeout_ms == fence_gfx_timeout_ms, "gfx default lockup timeout 10000 ms");
    expect(f.attach(1) == FenceStatus::attached && f.ring(1).timeout_ms == fence_compute_timeout_ms,
           "compute default lockup timeout 60000 ms");
    expect(f.attach(fence_ring_sdma0) == FenceStatus::attached &&
           f.ring(fence_ring_sdma0).timeout_ms == fence_sdma_timeout_ms,
           "SDMA default lockup timeout 10000 ms");
    f.writeback.set(2, 0);
    FenceRingConfig duplicate{FenceEngine::compute, 1, 0, 0, 0, f.writeback.view(2), 0, 0, 0};
    expect(f.fences.attach_ring(2, duplicate) == FenceStatus::invalid_argument,
           "one ring per ME/pipe/queue");
    FenceRingConfig sdma{FenceEngine::sdma, 0, 0, 0, 0, f.writeback.view(fence_ring_sdma0 + 1), 0,
                         0, 0};
    expect(f.fences.attach_ring(fence_ring_sdma0 + 1, sdma) == FenceStatus::invalid_argument,
           "SDMA ring index matches its instance");
    expect(f.platform.lock_errors == 0, "lock discipline");
}

void ordering_and_wrap() {
    Fixture f;
    f.attach(0, 0xfffffffeu);
    uint32_t seq[5]{};
    bool ok = true;

    for (uint32_t index = 0; index < 4; ++index) {
        ok = ok && f.submit(0, seq[index]) == FenceStatus::pending;
    }

    expect(ok && seq[0] == 0xffffffffu && seq[1] == 0 && seq[2] == 1 && seq[3] == 2,
           "sync_seq increments through the 32-bit wrap");
    expect(f.submit(0, seq[4]) == FenceStatus::busy,
           "num_hw_submission * 2 = 4 outstanding fences per ring");
    expect(f.fences.query(0, 3) == FenceStatus::invalid_argument, "unemitted sequence rejected");
    expect(f.fences.query(0, 0xffffffffu) == FenceStatus::pending &&
           f.fences.query(0, 0xfffffff0u) == FenceStatus::signalled,
           "older sequences are signalled in wrap order");
    f.writeback.set(0, 0);
    expect(f.fences.query(0, 0xffffffffu) == FenceStatus::signalled &&
           f.fences.query(0, 0) == FenceStatus::signalled &&
           f.fences.query(0, 1) == FenceStatus::pending, "a wrapped value signals across zero");
    const FenceRingReport info = f.ring(0);
    expect(info.last_seq == 0 && info.signalled == 2 && info.retired_position == 512,
           "retired position is the end of the newest signalled frame");
    FenceReservation reservation{};
    expect(f.fences.begin_submission(0, reservation) == FenceStatus::ready &&
           reservation.sequence == 3 && reservation.outstanding == 2 &&
           reservation.retired_position == 512, "a slot frees once its fence signalled");
    expect(f.fences.publish_submission(0, 4, 2048) == FenceStatus::invalid_argument &&
           f.fences.publish_submission(0, 3, 1024) == FenceStatus::invalid_argument,
           "publish needs sync_seq + 1 and an advancing position");
    expect(f.fences.publish_submission(0, 3, 1280) == FenceStatus::pending, "publish next");
    f.writeback.set(0, 3);
    expect(f.fences.query(0, 3) == FenceStatus::signalled && f.ring(0).retired_position == 1280,
           "all signalled");
}

void out_of_window() {
    Fixture f;
    f.attach(1, 10);
    uint32_t seq = 0;
    f.submit(1, seq);
    f.writeback.set(1, 12);
    expect(f.fences.query(1, 11) == FenceStatus::ring_hung, "value beyond sync_seq quarantines");
    const FenceReport report = f.fences.report();
    expect(report.reset_required && report.hung_rings == 2 &&
           report.rings[1].hang_reason == FenceHangReason::fence_out_of_window &&
           report.rings[1].hang_signalled_seq == 10 && report.rings[1].hang_emitted_seq == 11 &&
           f.platform.hung.size() == 1 && f.platform.hung[0] == 1,
           "fail closed: hung, reported, mode-1 reset required");
    expect(f.submit(1, seq) == FenceStatus::ring_hung, "no submissions to a hung ring");
    Fixture g;
    g.attach(2, 10);
    g.submit(2, seq);
    g.writeback.set(2, 9);
    expect(g.fences.query(2, 11) == FenceStatus::ring_hung, "value before last_seq quarantines");
}

void interrupt_driven() {
    Fixture f;
    f.attach(0);
    f.attach(1 + 6);       // MEC1 pipe 2 queue 1
    f.attach(fence_ring_sdma0 + 1);
    uint32_t a = 0, b = 0, c = 0;
    f.submit(0, a);
    f.submit(7, b);
    f.submit(fence_ring_sdma0 + 1, c);
    f.writeback.set(0, a);
    f.writeback.set(7, b);
    f.writeback.set(fence_ring_sdma0 + 1, c);
    // No vector yet: process_interrupts does not read the rings.
    f.dispatch();
    expect(f.ring(0).last_seq == 0 && f.ring(7).last_seq == 0, "no EOP, no processing");
    ++f.irq.gfx_eop[0];
    ++f.irq.compute_eop[0][2][1];
    f.dispatch();
    expect(f.ring(0).last_seq == a && f.ring(0).interrupt_processes == 1 &&
           f.ring(7).last_seq == b && f.ring(fence_ring_sdma0 + 1).last_seq == 0,
           "gfx_v11_0_eop_irq routes ME0 pipe 0 and MEC1 pipe 2 queue 1");
    ++f.irq.sdma_trap[0];
    f.dispatch();
    expect(f.ring(fence_ring_sdma0 + 1).last_seq == 0, "SDMA0 trap does not process SDMA1");
    ++f.irq.sdma_trap[1];
    f.dispatch();
    expect(f.ring(fence_ring_sdma0 + 1).last_seq == c &&
           !f.ring(fence_ring_sdma0 + 1).timeout_armed,
           "sdma_v6_0_process_trap_irq processes its instance; idle ring cancels the TDR");
    // Lost vectors: every active ring is re-read.
    f.submit(0, a);
    f.writeback.set(0, a);
    f.dispatch(true);
    expect(f.ring(0).last_seq == a && f.fences.report().lost_event_rescans == 1,
           "events_lost rescans every ring");
    // A ring attached later reacts only to newer records.
    Fixture g;
    ++g.irq.gfx_eop[0];
    g.dispatch();
    g.attach(0);
    g.dispatch();
    expect(g.ring(0).interrupt_processes == 0, "baseline consumed before attach");
    expect(f.platform.lock_errors == 0 && g.platform.lock_errors == 0, "lock discipline");
}

void fallback_timer() {
    Fixture f;
    f.attach(1);
    uint32_t seq = 0, second_seq = 0;
    f.submit(1, seq);
    FenceRingReport info = f.ring(1);
    expect(info.fallback_armed && info.fallback_deadline_us == f.platform.now + 500 * ms &&
           f.fences.next_deadline_us() == info.fallback_deadline_us,
           "publish enables signaling: AMDGPU_FENCE_JIFFIES_TIMEOUT 500 ms");
    f.writeback.set(1, seq);
    f.platform.now += 499 * ms;
    f.fences.poll();
    expect(f.ring(1).last_seq == 0 && f.ring(1).fallback_expiries == 0, "not yet due");
    f.platform.now += 1 * ms;
    f.fences.poll();
    info = f.ring(1);
    expect(info.last_seq == seq && info.fallback_expiries == 1 && info.fallback_progress == 1 &&
           !info.fallback_armed, "fallback found the lost completion (DRM_WARN) and is not re-armed");
    // Interrupt processing with work left reschedules a pending timer.
    f.submit(1, seq);
    f.submit(1, second_seq);
    f.platform.now += 100 * ms;
    f.writeback.set(1, seq);
    ++f.irq.compute_eop[0][0][0];
    f.dispatch();
    info = f.ring(1);
    expect(info.fallback_armed && info.fallback_deadline_us == f.platform.now + 500 * ms,
           "del_timer && seq != sync_seq reschedules the fallback");
    f.writeback.set(1, second_seq);
    ++f.irq.compute_eop[0][0][0];
    f.dispatch();
    expect(!f.ring(1).fallback_armed, "idle ring: the fallback stays cancelled");
    expect(f.fences.next_deadline_us() == UINT64_MAX, "nothing armed when idle");
}

void waits() {
    Fixture f;
    f.attach(0);
    uint32_t seq = 0;
    f.submit(0, seq);
    f.writeback.set(0, seq);
    expect(f.fences.wait(0, seq, 0) == FenceStatus::signalled && f.platform.blocks == 0,
           "already signalled without blocking");
    expect(f.fences.wait(0, seq + 1, 10) == FenceStatus::invalid_argument, "unemitted sequence");
    f.submit(0, seq);
    expect(f.fences.wait(0, seq, 0) == FenceStatus::timed_out, "zero timeout polls once");
    // Interrupt-driven wake: the EOP arrives while the waiter blocks.
    f.platform.on_block = [&] {
        f.writeback.set(0, seq);
        ++f.irq.gfx_eop[0];
        f.dispatch();
        f.platform.on_block = nullptr;
    };
    const unsigned before = f.platform.blocks;
    expect(f.fences.wait(0, seq, 10 * second) == FenceStatus::signalled &&
           f.platform.blocks == before + 1 && !f.platform.signals.empty() &&
           f.platform.signals.back() == f.platform.resets.back(),
           "EOP signals the waiter's own event (DPC -> KeSetEvent)");
    // No interrupt at all: the waiter's 500 ms slices re-read the writeback.
    f.submit(0, seq);
    const uint64_t start = f.platform.now;
    f.platform.on_block = [&] {
        if (f.platform.now >= start + 1000 * ms) {
            f.writeback.set(0, seq);
        }
    };
    expect(f.fences.wait(0, seq, 5 * second) == FenceStatus::signalled &&
           f.platform.now == start + 1500 * ms, "fallback-poll completion without interrupts");
    f.platform.on_block = nullptr;
    // Timeout: the fence never signals within the wait.
    f.submit(0, seq);
    const uint64_t begin = f.platform.now;
    expect(f.fences.wait(0, seq, 1200 * ms) == FenceStatus::timed_out &&
           f.platform.now == begin + 1200 * ms && f.ring(0).state == FenceRingState::active,
           "wait timeout does not hang the ring");
    // Waiter slot exhaustion through nested waits on other rings.
    Fixture g;

    for (uint32_t ring = 1; ring <= 8; ++ring) {
        g.attach(ring);
    }

    g.attach(0);
    uint32_t seqs[9]{};

    for (uint32_t ring = 0; ring <= 8; ++ring) {
        g.submit(ring, seqs[ring]);
    }

    uint32_t depth = 0;
    FenceStatus innermost = FenceStatus::pending;
    g.platform.on_block = [&] {
        if (depth < 8) {
            const uint32_t mine = ++depth;
            const FenceStatus status = g.fences.wait(mine, seqs[mine], 1 * ms);

            if (mine == 8) {
                innermost = status;
            }
        }
    };
    g.fences.wait(0, seqs[0], 1 * ms);
    expect(innermost == FenceStatus::busy && g.fences.report().waiter_exhaustion == 1,
           "eight waiter slots; the ninth concurrent waiter is refused");
    expect(f.platform.lock_errors == 0 && g.platform.lock_errors == 0,
           "lock never held while blocking");
}

void job_timeouts() {
    Fixture f;
    f.attach(0);
    f.attach(1);
    f.attach(fence_ring_sdma0);
    uint32_t g1 = 0, c1 = 0, s1 = 0;
    const uint64_t t0 = f.platform.now;
    f.submit(0, g1);
    f.submit(1, c1);
    f.submit(fence_ring_sdma0, s1);
    f.platform.now = t0 + 10 * second - 1;
    f.fences.poll();
    expect(f.ring(0).state == FenceRingState::active, "gfx: not before 10 s");
    f.platform.now = t0 + 10 * second;
    f.fences.poll();
    FenceRingReport gfx = f.ring(0);
    expect(gfx.state == FenceRingState::hung && gfx.hang_reason == FenceHangReason::job_timeout &&
           gfx.hang_signalled_seq == 0 && gfx.hang_emitted_seq == g1 &&
           f.ring(fence_ring_sdma0).state == FenceRingState::hung &&
           f.ring(1).state == FenceRingState::active,
           "gfx and SDMA lock up at 10000 ms, compute does not");
    f.platform.now = t0 + 60 * second - 1;
    f.fences.poll();
    expect(f.ring(1).state == FenceRingState::active, "compute: not before 60 s");
    f.platform.now = t0 + 60 * second;
    f.fences.poll();
    expect(f.ring(1).state == FenceRingState::hung, "compute locks up at 60000 ms");
    const FenceReport report = f.fences.report();
    expect(report.reset_required && report.hung_rings == ((1u << 0) | (1u << 1) | (1u << fence_ring_sdma0)),
           "every hung ring is reported; a mode-1 reset is required");
    uint32_t seq = 0;
    expect(f.submit(0, seq) == FenceStatus::ring_hung && f.fences.query(0, g1) == FenceStatus::ring_hung &&
           f.fences.wait(0, g1, second) == FenceStatus::ring_hung, "hung ring refuses work and waits");
    expect(f.fences.detach_ring(0, false) == FenceStatus::ring_hung, "no detach without a reset proof");
    // A completion that lands at the deadline is found by the final re-read.
    Fixture g;
    g.attach(0);
    const uint64_t t1 = g.platform.now;
    g.submit(0, g1);
    g.platform.now = t1 + 10 * second;
    g.writeback.set(0, g1);
    g.fences.poll();
    expect(g.ring(0).state == FenceRingState::active && g.ring(0).last_seq == g1,
           "TDR re-reads the fence before declaring a hang");
    // drm_sched_job_begin re-arms per job; a finished job restarts it for the next.
    Fixture h;
    h.attach(0);
    const uint64_t t2 = h.platform.now;
    uint32_t first = 0, second_seq = 0;
    h.submit(0, first);
    h.platform.now = t2 + 5 * second;
    h.submit(0, second_seq);
    expect(h.ring(0).timeout_deadline_us == t2 + 15 * second, "mod_delayed_work on job begin");
    h.platform.now = t2 + 8 * second;
    h.writeback.set(0, first);
    ++h.irq.gfx_eop[0];
    h.dispatch();
    expect(h.ring(0).timeout_deadline_us == t2 + 18 * second,
           "drm_sched_get_finished_job restarts the TDR for the next job");
    h.platform.now = t2 + 18 * second - 1;
    h.fences.poll();
    expect(h.ring(0).state == FenceRingState::active, "still within the restarted timeout");
    h.platform.now = t2 + 18 * second;
    h.fences.poll();
    expect(h.ring(0).state == FenceRingState::hung && h.ring(0).hang_signalled_seq == first,
           "second job times out");
    // A waiter detects the lockup too, without poll.
    Fixture w;
    w.attach(fence_ring_sdma0);
    w.submit(fence_ring_sdma0, s1);
    expect(w.fences.wait(fence_ring_sdma0, s1, 20 * second) == FenceStatus::ring_hung &&
           w.ring(fence_ring_sdma0).hang_reason == FenceHangReason::job_timeout,
           "a blocked waiter wakes with ring_hung at the lockup timeout");
    // Configured timeout override.
    Fixture o;
    o.attach(1, 0, 0, 2000);
    const uint64_t t3 = o.platform.now;
    o.submit(1, c1);
    o.platform.now = t3 + 2 * second;
    o.fences.poll();
    expect(o.ring(1).state == FenceRingState::hung, "explicit lockup timeout");
}

void cp_faults() {
    Fixture f;
    f.attach(0);
    f.attach(1 + 5);    // MEC1 pipe 1 queue 1
    f.attach(1 + 2);    // MEC1 pipe 2 queue 0
    f.irq.cp_fault_count[1][1][1] = 1;
    f.irq.cp_fault_source[1][1][1] = IrqSource::cp_priv_reg;
    f.dispatch();
    expect(f.ring(6).state == FenceRingState::hung &&
           f.ring(6).hang_reason == FenceHangReason::cp_priv_reg && f.ring(6).cp_faults == 1 &&
           f.ring(3).state == FenceRingState::active && f.ring(0).state == FenceRingState::active,
           "drm_sched_fault on the matching compute ring only");
    f.irq.cp_fault_count[0][0][0] = 2;
    f.irq.cp_fault_source[0][0][0] = IrqSource::cp_bad_opcode;
    f.dispatch();
    expect(f.ring(0).hang_reason == FenceHangReason::cp_bad_opcode && f.ring(0).cp_faults == 2,
           "ME0 faults route to the gfx ring");
    f.irq.cp_fault_count[1][2][0] = 1;
    f.irq.cp_fault_source[1][2][0] = IrqSource::cp_priv_instr;
    f.dispatch();
    expect(f.ring(3).hang_reason == FenceHangReason::cp_priv_instr, "priv_instr");
    f.irq.cp_fault_count[2][3][7] = 3;
    f.irq.cp_fault_invalid_me = 1;
    f.dispatch();
    expect(f.fences.report().cp_faults_unattributed == 4,
           "MEC2 queue without a kernel ring and ME 3 are counted, not routed");
    f.dispatch();
    expect(f.fences.report().cp_faults_unattributed == 4, "each fault counted once");
}

IrqVmFault vm_fault(uint32_t vmid, VmHub hub, bool read, uint32_t status) {
    IrqVmFault fault{};
    fault.address = 0x123456000ULL;
    fault.vmid = vmid;
    fault.register_hub = hub;
    fault.cache_hub = hub;
    fault.client = hub == VmHub::mmhub ? 0x12 : 0x0a;
    fault.status_read = read;
    fault.status = status;
    return fault;
}

uint32_t cid_status(uint32_t cid, uint32_t vmid = 0) { return (cid << 9) | (vmid << 20) | 0x1u; }

void vm_faults() {
    Fixture f;
    f.attach(0);
    f.attach(1);
    f.attach(2);
    f.attach(fence_ring_sdma0);
    f.attach(fence_ring_sdma0 + 1);
    f.add_vm_fault(vm_fault(3, VmHub::gfxhub, true, cid_status(fence_utcl2_cid_cpc, 3)));
    f.dispatch();
    FenceReport report = f.fences.report();
    expect(report.vm_faults == 1 && report.vm_faults_process == 1 && report.hung_rings == 0 &&
           f.platform.printed.size() == 1, "process VMID fault: reported only (vm_fault_stop NEVER)");
    f.add_vm_fault(vm_fault(0, VmHub::gfxhub, true, cid_status(fence_utcl2_cid_sdma1)));
    f.dispatch();
    expect(f.ring(fence_ring_sdma0 + 1).hang_reason == FenceHangReason::vm_fault &&
           f.ring(fence_ring_sdma0).state == FenceRingState::active &&
           f.ring(0).state == FenceRingState::active, "VMID0 SDMA1 client quarantines sdma1 only");
    f.add_vm_fault(vm_fault(0, VmHub::gfxhub, true, cid_status(fence_utcl2_cid_cpc)));
    f.dispatch();
    expect(f.ring(1).state == FenceRingState::hung && f.ring(2).state == FenceRingState::hung &&
           f.ring(0).state == FenceRingState::active, "CPC: every compute ring");
    f.add_vm_fault(vm_fault(0, VmHub::mmhub, true, cid_status(3)));
    f.dispatch();
    report = f.fences.report();
    expect(report.vm_faults_vmid0_unattributed == 1 && f.ring(0).state == FenceRingState::active,
           "MMHUB VMID0 fault is no GC kernel ring");
    f.add_vm_fault(vm_fault(0, VmHub::gfxhub, false, 0));
    f.dispatch();
    expect(f.ring(0).state == FenceRingState::hung &&
           f.ring(fence_ring_sdma0).state == FenceRingState::hung,
           "unknown client: every kernel ring (fail closed)");
    // A process VMID in the IV but VMID 0 latched in the status is kernel context.
    Fixture g;
    g.attach(0);
    g.add_vm_fault(vm_fault(5, VmHub::gfxhub, true, cid_status(fence_utcl2_cid_cpg, 0)));
    g.dispatch();
    expect(g.ring(0).hang_reason == FenceHangReason::vm_fault, "CPG with latched VMID0: gfx ring");
    // printk_ratelimit: 10 per 5 s window, then the suppressed count.
    Fixture r;

    for (uint32_t index = 0; index < 25; ++index) {
        r.add_vm_fault(vm_fault(7, VmHub::gfxhub, true, cid_status(8, 7)));
    }

    r.dispatch();
    report = r.fences.report();
    expect(report.vm_faults == 25 && report.vm_faults_printed == 10 &&
           report.vm_faults_suppressed == 15 && r.platform.printed.size() == 10,
           "burst of 10 within the interval");
    r.platform.now += 5 * second;
    r.add_vm_fault(vm_fault(7, VmHub::gfxhub, true, cid_status(8, 7)));
    r.dispatch();
    expect(r.platform.printed.size() == 10, "interval not yet over at exactly 5 s");
    r.platform.now += 1;
    r.add_vm_fault(vm_fault(7, VmHub::gfxhub, true, cid_status(8, 7)));
    r.dispatch();
    expect(r.platform.printed.size() == 11 && r.platform.suppressed.back() == 16,
           "new window prints and reports the 16 suppressed callbacks");
    // The log holds two drain batches; older entries are counted as lost.
    Fixture l;
    l.vm.total = 70;
    l.dispatch();
    expect(l.fences.report().vm_faults_lost == 6 && l.fences.report().vm_faults == 64,
           "overwritten log entries are counted");
}

void detach_and_reset() {
    Fixture f;
    f.attach(0);
    f.attach(1);
    uint32_t seq = 0;
    f.submit(0, seq);
    expect(f.fences.detach_ring(0, false) == FenceStatus::pending, "busy ring stays attached");
    f.writeback.set(0, seq);
    expect(f.fences.detach_ring(0, false) == FenceStatus::detached &&
           f.ring(0).state == FenceRingState::detached, "idle ring detaches");
    expect(f.fences.query(0, seq) == FenceStatus::unavailable, "detached ring has no fences");
    f.submit(1, seq);
    f.fences.mark_hung(1, FenceHangReason::submit_failure);
    bool proof = false;
    const FenceRecovery recovery{&proof, [](void* context) noexcept {
        return *static_cast<bool*>(context);
    }};
    expect(f.fences.release_after_reset(recovery) == FenceStatus::ring_hung &&
           f.fences.report().reset_required, "no release without a reset proof");
    // A waiter blocked on the hung ring's peer sees the detach.
    f.attach(2);
    f.submit(2, seq);
    f.platform.on_block = [&] {
        proof = true;
        f.fences.release_after_reset(recovery);
        f.platform.on_block = nullptr;
    };
    expect(f.fences.wait(2, seq, 10 * second) == FenceStatus::unavailable,
           "release_after_reset wakes waiters, which see the detach");
    const FenceReport report = f.fences.report();
    expect(!report.reset_required && report.hung_rings == 0 &&
           report.rings[1].state == FenceRingState::detached &&
           report.rings[2].state == FenceRingState::detached, "every ring released after reset");
    expect(f.attach(1, 77) == FenceStatus::attached && f.ring(1).sync_seq == 77,
           "rings attach again after the reset");
    // A hung ring detaches with its owner's reset proof.
    Fixture g;
    g.attach(0);
    g.fences.mark_hung(0, FenceHangReason::submit_failure);
    expect(g.fences.detach_ring(0, true) == FenceStatus::detached && !g.fences.report().reset_required,
           "owner reset proof detaches a hung ring");
    expect(f.platform.lock_errors == 0 && g.platform.lock_errors == 0, "lock discipline");
}

} // namespace

int main() {
    initialize_contract();
    attach_contract();
    ordering_and_wrap();
    out_of_window();
    interrupt_driven();
    fallback_timer();
    waits();
    job_timeouts();
    cp_faults();
    vm_faults();
    detach_and_reset();
    return checks.finish("fence_tests");
}

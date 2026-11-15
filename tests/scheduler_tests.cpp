/*
Module: rx7900.tests.scheduler
Purpose: Verify the job scheduler: credit limit, entity FIFO and priorities, dependencies, timeouts, cancellation and its backends.
Created: 2026-10-22
Architecture: Test-local fake rings for the drm_sched rules, the real FenceManager
behind the kernel-ring adapter query, and real UserQueues over the process-queue
model (user_queue_fixture.hpp). It is not hardware evidence. Expected behaviour is
derived from Linux v6.12 sched_main.c, sched_entity.c, amdgpu_job.c and
amdgpu_drv.c.
*/
#include <vector>

#include "scheduler.hpp"
#include "user_queue_fixture.hpp"

namespace {

using namespace rx7900::hardware;
using namespace rx7900_test::user_queue_fixture;

rx7900_test::Checks checks;

void expect(bool condition, const char* contract) { checks.expect(condition, contract); }

// A ring whose fences the test signals. run hands out sequences 1, 2, ...
struct FakeRing final {
    uint32_t emitted = 0;
    uint32_t signalled = 0;
    bool hung = false;
    SchedulerRunResult next = SchedulerRunResult::submitted;
    std::vector<uint32_t> ib_dwords;
    unsigned polls = 0;

    static SchedulerRunResult run(void* context, const SchedulerIb& ib, uint32_t& sequence) noexcept {
        auto& self = *static_cast<FakeRing*>(context);

        if (self.next != SchedulerRunResult::submitted) {
            return self.next;
        }

        self.ib_dwords.push_back(ib.dwords);
        sequence = ++self.emitted;
        return SchedulerRunResult::submitted;
    }

    static SchedulerFenceResult query(void* context, uint32_t sequence) noexcept {
        const auto& self = *static_cast<FakeRing*>(context);

        if (sequence == 0 || sequence > self.emitted) {
            return SchedulerFenceResult::failed;
        }

        return sequence <= self.signalled ? SchedulerFenceResult::signalled :
               self.hung ? SchedulerFenceResult::hung : SchedulerFenceResult::pending;
    }

    static void poll(void* context) noexcept { ++static_cast<FakeRing*>(context)->polls; }

    SchedulerBackend backend() noexcept { return {this, run, query, poll}; }
};

SchedulerJob job(uint32_t entity, uint32_t dwords) {
    SchedulerJob result{};
    result.entity = entity;
    result.ib = {{}, 0, 0, dwords};
    return result;
}

SchedulerJob after(uint32_t entity, uint32_t dwords, uint64_t dependency) {
    SchedulerJob result = job(entity, dwords);
    result.dependencies[0] = {SchedulerDependencyKind::job, dependency, 0, 0};
    result.dependency_count = 1;
    return result;
}

SchedulerJobState state(const Scheduler& scheduler, uint64_t id) { return scheduler.job(id).state; }

void credit_limit_and_order() {
    Scheduler scheduler;
    FakeRing ring;
    uint32_t target = 0;
    uint32_t entity = 0;
    expect(scheduler.register_target({nullptr, nullptr, nullptr, nullptr}, target) ==
           SchedulerStatus::invalid_argument, "backend needs run and query");
    expect(scheduler.register_target(ring.backend(), target) == SchedulerStatus::registered &&
           scheduler.create_entity(target, SchedulerPriority::normal, entity) == SchedulerStatus::created,
           "target and entity");
    uint64_t ids[5]{};

    for (uint32_t index = 0; index < 5; ++index) {
        scheduler.push(job(entity, 10 + index), ids[index]);
    }

    expect(ids[0] == 1 && ids[4] == 5 && state(scheduler, 1) == SchedulerJobState::queued,
           "job ids increase in push order");
    scheduler.run();
    expect(ring.emitted == 2 && state(scheduler, 1) == SchedulerJobState::running &&
           state(scheduler, 2) == SchedulerJobState::running &&
           state(scheduler, 3) == SchedulerJobState::queued && ring.polls == 1,
           "amdgpu_sched_hw_submission 2: two jobs on the ring");
    scheduler.run();
    expect(ring.emitted == 2, "no credit, nothing more");
    ring.signalled = 1;
    scheduler.run();
    expect(state(scheduler, 1) == SchedulerJobState::completed && ring.emitted == 3 &&
           scheduler.job(3).sequence == 3, "a finished job returns its credit");
    ring.signalled = 5;
    scheduler.run();
    scheduler.run();
    expect(ring.ib_dwords == std::vector<uint32_t>{10, 11, 12, 13, 14} &&
           scheduler.report().finished == 5 && scheduler.report().targets[target].completed == 5,
           "per-entity FIFO order");
    expect(scheduler.release_job(1) == SchedulerStatus::released &&
           scheduler.job(1).state == SchedulerJobState::free &&
           scheduler.release_job(1) == SchedulerStatus::not_found, "finished job released");

    // Priorities and FIFO across entities.
    Scheduler ordered;
    FakeRing second;
    ordered.register_target(second.backend(), target);
    uint32_t low = 0;
    uint32_t normal = 0;
    uint32_t kernel = 0;
    ordered.create_entity(target, SchedulerPriority::low, low);
    ordered.create_entity(target, SchedulerPriority::normal, normal);
    ordered.create_entity(target, SchedulerPriority::kernel, kernel);
    uint64_t id = 0;
    ordered.push(job(low, 1), id);
    ordered.push(job(normal, 2), id);
    ordered.push(job(normal, 3), id);
    ordered.push(job(kernel, 4), id);
    ordered.run();
    second.signalled = 2;
    ordered.run();
    second.signalled = 4;
    ordered.run();
    expect(second.ib_dwords == std::vector<uint32_t>{4, 2, 3, 1},
           "KERNEL first, then NORMAL in submit order, LOW last");

    // full: retried; failed: that job fails, the next runs.
    Scheduler retry;
    FakeRing busy;
    retry.register_target(busy.backend(), target);
    retry.create_entity(target, SchedulerPriority::normal, entity);
    uint64_t a = 0;
    uint64_t b = 0;
    retry.push(job(entity, 1), a);
    retry.push(job(entity, 2), b);
    busy.next = SchedulerRunResult::full;
    retry.run();
    expect(state(retry, a) == SchedulerJobState::queued, "ring full: job stays queued");
    busy.next = SchedulerRunResult::failed;
    retry.run();
    expect(state(retry, a) == SchedulerJobState::failed && state(retry, b) == SchedulerJobState::failed &&
           retry.report().targets[target].failed == 2, "run_job error finishes the job");
    expect(retry.push(job(99, 1), a) == SchedulerStatus::invalid_argument &&
           retry.push(job(entity, 0), a) == SchedulerStatus::invalid_argument,
           "unknown entity and empty IB refused");
}

void dependencies() {
    Scheduler scheduler;
    FakeRing first;
    FakeRing second;
    uint32_t t1 = 0;
    uint32_t t2 = 0;
    uint32_t e1 = 0;
    uint32_t e2 = 0;
    scheduler.register_target(first.backend(), t1);
    scheduler.register_target(second.backend(), t2);
    scheduler.create_entity(t1, SchedulerPriority::normal, e1);
    scheduler.create_entity(t2, SchedulerPriority::normal, e2);
    uint64_t producer = 0;
    uint64_t consumer = 0;
    uint64_t later = 0;
    scheduler.push(job(e1, 1), producer);
    scheduler.push(after(e2, 2, producer), consumer);
    scheduler.push(job(e2, 3), later);
    scheduler.run();
    expect(state(scheduler, producer) == SchedulerJobState::running &&
           state(scheduler, consumer) == SchedulerJobState::queued &&
           state(scheduler, later) == SchedulerJobState::queued && second.emitted == 0,
           "consumer waits for the producer; its entity FIFO blocks the later job too");
    expect(scheduler.release_job(producer) == SchedulerStatus::invalid_argument,
           "a running job cannot be released");
    first.signalled = 1;
    scheduler.run();
    expect(state(scheduler, consumer) == SchedulerJobState::running &&
           state(scheduler, later) == SchedulerJobState::running,
           "producer finished: consumer and the next job run");
    uint64_t fenced = 0;
    SchedulerJob fence_job = job(e1, 4);
    fence_job.dependencies[0] = {SchedulerDependencyKind::target_fence, 0, t2, 2};
    fence_job.dependency_count = 1;
    expect(scheduler.push(fence_job, fenced) == SchedulerStatus::queued, "hardware-fence dependency");
    scheduler.run();
    expect(state(scheduler, fenced) == SchedulerJobState::queued, "waits for target 2 fence 2");
    second.signalled = 2;
    scheduler.run();
    expect(state(scheduler, fenced) == SchedulerJobState::running, "fence signalled: runs");
    SchedulerJob bad = job(e1, 1);
    bad.dependencies[0] = {SchedulerDependencyKind::job, 999, 0, 0};
    bad.dependency_count = 1;
    uint64_t id = 0;
    expect(scheduler.push(bad, id) == SchedulerStatus::invalid_argument, "unknown job dependency refused");

    // Failure propagation: a cancelled producer cancels its whole chain.
    Scheduler chain;
    FakeRing ring;
    uint32_t target = 0;
    uint32_t entity = 0;
    uint32_t other = 0;
    chain.register_target(ring.backend(), target);
    chain.create_entity(target, SchedulerPriority::normal, entity);
    chain.create_entity(target, SchedulerPriority::normal, other);
    uint64_t root = 0;
    uint64_t child = 0;
    uint64_t grandchild = 0;
    ring.next = SchedulerRunResult::full;
    chain.push(job(entity, 1), root);
    chain.push(after(other, 2, root), child);
    chain.push(after(other, 3, child), grandchild);
    expect(chain.release_job(root) == SchedulerStatus::invalid_argument, "queued job not releasable");
    chain.destroy_entity(entity);
    chain.run();
    expect(state(chain, root) == SchedulerJobState::cancelled &&
           state(chain, child) == SchedulerJobState::cancelled &&
           state(chain, grandchild) == SchedulerJobState::cancelled,
           "destroyed entity: -ESRCH; dependants cancelled (fail closed)");
    expect(chain.release_job(root) == SchedulerStatus::released,
           "finished root released after its dependants");
}

void timeouts_and_cancellation() {
    Scheduler scheduler;
    FakeRing ring;
    FakeRing other_ring;
    uint32_t target = 0;
    uint32_t other = 0;
    uint32_t entity = 0;
    uint32_t elsewhere = 0;
    scheduler.register_target(ring.backend(), target);
    scheduler.register_target(other_ring.backend(), other);
    scheduler.create_entity(target, SchedulerPriority::normal, entity);
    scheduler.create_entity(other, SchedulerPriority::normal, elsewhere);
    uint64_t ids[4]{};

    for (auto& id : ids) {
        scheduler.push(job(entity, 1), id);
    }

    uint64_t dependant = 0;
    scheduler.push(after(elsewhere, 1, ids[0]), dependant);
    scheduler.run();
    ring.hung = true;
    scheduler.run();
    const SchedulerReport report = scheduler.report();
    expect(state(scheduler, ids[0]) == SchedulerJobState::timed_out &&
           state(scheduler, ids[1]) == SchedulerJobState::cancelled &&
           state(scheduler, ids[2]) == SchedulerJobState::cancelled &&
           state(scheduler, ids[3]) == SchedulerJobState::cancelled &&
           state(scheduler, dependant) == SchedulerJobState::cancelled &&
           report.targets[target].hung && report.reset_required && report.targets[target].running == 0,
           "hung ring: oldest job -ETIME, the rest and dependants cancelled, no reset");
    uint64_t id = 0;
    expect(scheduler.push(job(entity, 1), id) == SchedulerStatus::hung &&
           scheduler.push(job(elsewhere, 1), id) == SchedulerStatus::queued,
           "hung target takes nothing; others continue");

    // Driver quarantine: queued cancelled, running tracked to completion.
    Scheduler stopped;
    FakeRing quiet;
    stopped.register_target(quiet.backend(), target);
    stopped.create_entity(target, SchedulerPriority::normal, entity);

    for (auto& job_id : ids) {
        stopped.push(job(entity, 1), job_id);
    }

    stopped.run();
    expect(stopped.cancel_target(target) == SchedulerStatus::ready &&
           state(stopped, ids[2]) == SchedulerJobState::cancelled &&
           state(stopped, ids[0]) == SchedulerJobState::running, "cancel_target keeps running jobs");
    quiet.signalled = 2;
    stopped.run();
    expect(state(stopped, ids[0]) == SchedulerJobState::completed &&
           state(stopped, ids[1]) == SchedulerJobState::completed &&
           stopped.push(job(entity, 1), id) == SchedulerStatus::hung && !stopped.report().reset_required,
           "running jobs complete; nothing new is accepted");

    // A table full of unreleased jobs.
    Scheduler table;
    FakeRing fill;
    table.register_target(fill.backend(), target);
    table.create_entity(target, SchedulerPriority::normal, entity);
    SchedulerStatus last = SchedulerStatus::queued;

    for (uint32_t index = 0; index <= scheduler_max_jobs; ++index) {
        last = table.push(job(entity, 1), id);
    }

    expect(last == SchedulerStatus::full, "bounded job table");

    // After a confirmed reset: unfinished jobs cancelled, targets dropped.
    table.run();
    fill.signalled = 1;
    table.run();
    expect(state(table, 1) == SchedulerJobState::completed &&
           state(table, 2) == SchedulerJobState::running &&
           table.release_after_reset() == SchedulerStatus::released &&
           state(table, 1) == SchedulerJobState::completed &&
           state(table, 2) == SchedulerJobState::cancelled &&
           state(table, 64) == SchedulerJobState::cancelled && table.report().running == 0 &&
           !table.report().targets[target].registered &&
           table.push(job(entity, 1), id) == SchedulerStatus::invalid_argument &&
           table.register_target(fill.backend(), target) == SchedulerStatus::registered,
           "release_after_reset: unfinished cancelled, finished kept, rings registered again");
}

// Real FenceManager behind the kernel-ring adapter's query; Gfx11Queues/Sdma6 not
// started, so run is refused (failed).
struct FencePlatformModel final {
    uint64_t now = 1000;

    FencePlatform platform() {
        return {this, [](void*) noexcept {}, [](void*) noexcept {},
                [](void* c) noexcept { return static_cast<FencePlatformModel*>(c)->now; },
                [](void*, uint32_t) noexcept {}, [](void*, uint32_t) noexcept {},
                [](void*, uint32_t, uint64_t) noexcept { return false; }, nullptr, nullptr};
    }
};

void kernel_adapters() {
    FencePlatformModel platform;
    std::unique_ptr<FenceManager> fences = std::make_unique<FenceManager>();
    fences->initialize(platform.platform());
    uint8_t writeback[4]{};
    expect(fences->attach_ring(fence_ring_compute0, {FenceEngine::compute, 1, 0, 0, 0,
                                                      {writeback, sizeof(writeback)}, 0, 0, 0}) ==
           FenceStatus::attached, "compute ring 0 attached to the FenceManager");
    FenceReservation reservation{};
    fences->begin_submission(fence_ring_compute0, reservation);
    fences->publish_submission(fence_ring_compute0, reservation.sequence, 256);
    const uint32_t reserved = reservation.sequence;
    std::unique_ptr<Gfx11Queues> queues = std::make_unique<Gfx11Queues>();
    std::unique_ptr<Sdma6> sdma = std::make_unique<Sdma6>();
    SchedulerGfx11Target gfx{queues.get(), fences.get(), fence_ring_compute0};
    SchedulerSdma6Target copy{sdma.get(), fences.get(), 0};
    Scheduler scheduler;
    uint32_t compute = 0;
    uint32_t dma = 0;
    uint32_t entity = 0;
    uint32_t compute_entity = 0;
    scheduler.register_target(scheduler_gfx11_backend(gfx), compute);
    scheduler.register_target(scheduler_sdma6_backend(copy), dma);
    scheduler.create_entity(dma, SchedulerPriority::normal, entity);
    scheduler.create_entity(compute, SchedulerPriority::normal, compute_entity);
    SchedulerJob waiting = job(entity, 8);
    waiting.dependencies[0] = {SchedulerDependencyKind::target_fence, 0, compute, reserved};
    waiting.dependency_count = 1;
    uint64_t id = 0;
    scheduler.push(waiting, id);
    scheduler.run();
    expect(state(scheduler, id) == SchedulerJobState::queued, "kernel fence pending: job waits");
    writeback[0] = static_cast<uint8_t>(reserved);
    scheduler.run();
    expect(state(scheduler, id) == SchedulerJobState::failed,
           "fence signalled: run_job on an SDMA engine without submission is refused (failed)");
    uint64_t direct = 0;
    scheduler.push(job(compute_entity, 5), direct);
    scheduler.run();
    expect(state(scheduler, direct) == SchedulerJobState::failed,
           "Gfx11Queues without submission refuses run_job (failed)");
    uint64_t hung = 0;
    fences->begin_submission(fence_ring_compute0, reservation);
    fences->publish_submission(fence_ring_compute0, reservation.sequence, 512);
    waiting.dependencies[0].sequence = reservation.sequence;
    scheduler.push(waiting, hung);
    fences->mark_hung(fence_ring_compute0, FenceHangReason::job_timeout);
    scheduler.run();
    expect(state(scheduler, hung) == SchedulerJobState::cancelled,
           "dependency on a hung kernel ring: cancelled");
}

void user_queue_backend() {
    Fixture fixture;
    fixture.up();
    UserQueues& queues = *fixture.queues;
    const auto process = fixture.process();
    const auto gang = fixture.gang(process.pasid);
    const auto gfx = fixture.queue(gang.id, Mes11QueueType::gfx, 0x100000, 8192, 50);
    const auto ib = fixture.map(process.pasid, 0x400000, 0x1000, read_only);
    const auto data = fixture.map(process.pasid, 0x500000, 0x1000);
    write_gfx_ib(fixture.cpu_of(ib.buffer), 0x500000);
    SchedulerUserQueueTarget target{&queues, gfx.id};
    Scheduler scheduler;
    uint32_t ring = 0;
    uint32_t entity = 0;
    scheduler.register_target(scheduler_user_queue_backend(target), ring);
    scheduler.create_entity(ring, SchedulerPriority::normal, entity);
    SchedulerJob work{};
    work.entity = entity;
    work.ib = {{}, 0, 0x400000, 5};
    uint64_t ids[3]{};

    for (auto& id : ids) {
        scheduler.push(work, id);
    }

    scheduler.run();
    expect(state(scheduler, ids[0]) == SchedulerJobState::running &&
           state(scheduler, ids[1]) == SchedulerJobState::running &&
           state(scheduler, ids[2]) == SchedulerJobState::queued &&
           u32(fixture.cpu_of(data.buffer), 0) == 0xDEADBEEFu, "two IBs on the user queue");
    scheduler.run();
    scheduler.run();
    expect(state(scheduler, ids[2]) == SchedulerJobState::completed &&
           queues.queue_report(gfx.id).sync_seq == 3, "all three completed through the queue fence");

    // Suspended gangs: the job waits (ring-full semantics), then runs.
    fixture.mes->suspend_all();
    uint64_t parked = 0;
    scheduler.push(work, parked);
    scheduler.run();
    expect(state(scheduler, parked) == SchedulerJobState::queued, "no submission while suspended");
    fixture.mes->resume_all();
    scheduler.run();
    expect(state(scheduler, parked) == SchedulerJobState::running, "runs after RESUME");
    scheduler.run();

    // Queue timeout through UserQueues::poll -> job timed out.
    fixture.engines.hang_doorbell = queues.queue_report(gfx.id).doorbell_index;
    uint64_t stuck = 0;
    uint64_t next = 0;
    scheduler.push(work, stuck);
    scheduler.push(work, next);
    scheduler.run();
    fixture.now += 50001;
    scheduler.run();
    expect(state(scheduler, stuck) == SchedulerJobState::timed_out &&
           state(scheduler, next) == SchedulerJobState::cancelled &&
           queues.queue_report(gfx.id).hang_reason == FenceHangReason::job_timeout &&
           scheduler.report().reset_required, "user-queue job timeout: -ETIME, queue hung, no reset");
}

} // namespace

int main() {
    credit_limit_and_order();
    dependencies();
    timeouts_and_cancellation();
    kernel_adapters();
    user_queue_backend();
    return checks.finish("scheduler_tests");
}

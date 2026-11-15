/*
Module: rx7900.hardware.scheduler
Purpose: Queue the driver's own GPU jobs per entity, resolve fence dependencies and feed kernel rings and MES user queues within the hardware submission limit.
Created: 2026-10-22
Architecture: A bounded, polled equivalent of drm_sched for internal use. Each
target is one ring behind a SchedulerBackend (adapters for Gfx11Queues and Sdma6
with FenceManager, and for UserQueues); entities feed exactly one target. Job
timeouts are the backends' own (FenceManager job timeout, UserQueues queue
timeout): a target reported hung turns its head job into timed_out and cancels
the rest. Nothing here resets hardware.
Copyright 2015 Advanced Micro Devices, Inc.; Copyright 2009 Jerome Glisse.
Adapted Linux v6.12 drivers/gpu/drm/scheduler/sched_main.c (drm_sched_can_queue,
credit accounting, drm_sched_select_entity FIFO policy, drm_sched_run_job_work,
drm_sched_job_done, drm_sched_job_timedout), sched_entity.c
(drm_sched_entity_push_job, pop_job dependency loop, drm_sched_entity_kill
-ESRCH), sched_fence.c, include/drm/gpu_scheduler.h (priorities),
amdgpu_job.c (amdgpu_job_run skipping errored jobs, amdgpu_job_timedout -ETIME),
amdgpu_device.c (drm_sched_init with ring->num_hw_submission) and amdgpu_drv.c
(amdgpu_sched_hw_submission 2). Notice and provenance: licenses/AMDGPU-SDMA.txt.
*/
#pragma once

#include "sdma6_init.hpp"
#include "user_queue.hpp"

namespace rx7900::hardware {

// amdgpu_sched_hw_submission (default 2) is the drm_sched credit_limit of every
// gfx/compute/SDMA ring; amdgpu jobs carry one credit each.
constexpr uint32_t scheduler_hw_submission = 2;
constexpr uint32_t scheduler_max_targets = 16;
constexpr uint32_t scheduler_max_entities = 32;
constexpr uint32_t scheduler_max_jobs = 64;
constexpr uint32_t scheduler_max_dependencies = 4;

enum class SchedulerStatus {
    ready,
    registered,
    created,
    queued,
    released,
    destroyed,
    // A job slot or dependency is still in use; nothing changed.
    busy,
    // Every job slot holds an unreleased job.
    full,
    not_found,
    unavailable,
    invalid_argument,
    // The entity's target is hung or stopped: no new job is accepted.
    hung
};

// drm_sched_priority (v6.12): KERNEL is served first, then HIGH, NORMAL, LOW.
enum class SchedulerPriority : uint32_t { kernel = 0, high = 1, normal = 2, low = 3 };

// The finished fence of a job: completed (error 0), cancelled (-ECANCELED: the
// entity was destroyed (-ESRCH in Linux), a dependency did not complete, or its
// target was hung/stopped before it ran), timed_out (-ETIME, the guilty job of a
// hung target), failed (run_job refused it, e.g. an invalid IB).
enum class SchedulerJobState : uint32_t {
    free,
    queued,
    running,
    completed,
    cancelled,
    timed_out,
    failed
};

enum class SchedulerRunResult : uint32_t { submitted, full, hung, failed };
enum class SchedulerFenceResult : uint32_t { signalled, pending, hung, failed };

// One IB: a Manager handle and byte offset for kernel rings (VMID0), or a
// process VA for user queues; the backend uses the half that applies.
struct SchedulerIb final {
    memory::Handle ib;
    uint64_t offset;
    uint64_t va;
    uint32_t dwords;
};

// One ring. run is amdgpu_job_run -> amdgpu_ib_schedule (sequence = the hardware
// fence); query reads that fence (amdgpu_fence_process); poll (optional) drives
// the backend's fallback timer and job timeout. All PASSIVE_LEVEL, serialized
// with the scheduler; context outlives the scheduler registration.
struct SchedulerBackend final {
    void* context;
    SchedulerRunResult (*run)(void* context, const SchedulerIb& ib, uint32_t& sequence) noexcept;
    SchedulerFenceResult (*query)(void* context, uint32_t sequence) noexcept;
    void (*poll)(void* context) noexcept;
};

enum class SchedulerDependencyKind : uint32_t { job, target_fence };

// drm_sched_job_add_dependency: another scheduler job's finished fence, or a
// hardware fence (sequence) of a registered target.
struct SchedulerDependency final {
    SchedulerDependencyKind kind;
    uint64_t job;
    uint32_t target;
    uint32_t sequence;
};

struct SchedulerJob final {
    uint32_t entity;
    SchedulerIb ib;
    SchedulerDependency dependencies[scheduler_max_dependencies];
    uint32_t dependency_count;
};

struct SchedulerJobReport final {
    SchedulerJobState state = SchedulerJobState::free;
    uint64_t id = 0;
    uint32_t entity = 0;
    uint32_t target = 0;
    uint32_t sequence = 0;
};

struct SchedulerTargetReport final {
    bool registered = false;
    // The backend reported the ring hung (FenceManager/UserQueues quarantine).
    bool hung = false;
    // cancel_target stopped it (driver quarantine decision).
    bool stopped = false;
    uint32_t running = 0;
    uint64_t submitted = 0;
    uint64_t completed = 0;
    uint64_t cancelled = 0;
    uint64_t timed_out = 0;
    uint64_t failed = 0;
};

struct SchedulerReport final {
    uint32_t queued = 0;
    uint32_t running = 0;
    uint32_t finished = 0;
    // A target is hung: only an explicit reset can return it (never implicit).
    bool reset_required = false;
    SchedulerTargetReport targets[scheduler_max_targets]{};
};

// Device-owned, trivially destructible. Serialized PASSIVE_LEVEL use by the
// device owner (the work item a fence or user-queue interrupt hint wakes, Linux
// sched->submit_wq). Jobs stay in their slot after they finished until
// release_job, so a finished fence stays queryable.
class Scheduler final {
public:
    Scheduler() noexcept = default;
    Scheduler(const Scheduler&) = delete;
    Scheduler& operator=(const Scheduler&) = delete;

    // drm_sched_init with credit_limit scheduler_hw_submission.
    SchedulerStatus register_target(const SchedulerBackend& backend, uint32_t& target) noexcept;
    // drm_sched_entity_init with one scheduler.
    SchedulerStatus create_entity(uint32_t target, SchedulerPriority priority,
                                  uint32_t& entity) noexcept;
    // drm_sched_entity_destroy -> drm_sched_entity_kill: queued jobs become
    // cancelled; running jobs finish on their target as usual.
    SchedulerStatus destroy_entity(uint32_t entity) noexcept;

    // drm_sched_job_init + add_dependency + drm_sched_entity_push_job. Job
    // dependencies must name existing jobs; target fences a registered target.
    // job_id is unique and increasing (the FIFO submit order).
    SchedulerStatus push(const SchedulerJob& job, uint64_t& job_id) noexcept;

    // One pass of the scheduler work: backend polls, finished jobs
    // (drm_sched_get_finished_job; credits returned), hung targets (head
    // running job timed_out, the rest cancelled), dependency resolution, then
    // per target while credits remain: the ready entity of highest priority and
    // oldest head job runs its head job (drm_sched_select_entity FIFO,
    // drm_sched_entity_pop_job, run_job). Returns ready.
    SchedulerStatus run() noexcept;

    // Driver quarantine of a target (no hardware access): queued jobs of its
    // entities are cancelled and nothing more is submitted; running jobs are
    // still tracked until their fences signal or the backend reports a hang.
    SchedulerStatus cancel_target(uint32_t target) noexcept;

    // After a confirmed reset (the owners' release_after_reset ran): every queued
    // or running job is cancelled (its hardware fence died with the reset),
    // targets and entities are dropped; finished records stay until release_job.
    // The integrator registers the rebuilt rings again.
    SchedulerStatus release_after_reset() noexcept;

    SchedulerJobReport job(uint64_t job_id) const noexcept;
    // drm_sched_job_cleanup/free_job of a finished job; busy while a queued job
    // depends on it, invalid_argument while it is queued or running.
    SchedulerStatus release_job(uint64_t job_id) noexcept;

    SchedulerReport report() const noexcept;

private:
    struct Target final {
        bool used;
        SchedulerBackend backend;
        SchedulerTargetReport report;
    };

    struct Entity final {
        bool used;
        uint32_t target;
        SchedulerPriority priority;
    };

    struct Slot final {
        SchedulerJobReport report;
        SchedulerJob job;
    };

    bool find(uint64_t job_id, uint32_t& slot) const noexcept;
    void finish(Slot& slot, SchedulerJobState state) noexcept;
    void retire(uint32_t target) noexcept;
    void cancel_queued(uint32_t target) noexcept;
    bool resolve_dependencies() noexcept;
    bool ready(const Slot& slot) noexcept;
    bool head(uint32_t entity, uint32_t& slot) const noexcept;
    void submit(uint32_t target) noexcept;

    Target targets_[scheduler_max_targets]{};
    Entity entities_[scheduler_max_entities]{};
    Slot slots_[scheduler_max_jobs]{};
    uint64_t next_id_ = 1;
};

// Adapters. Gfx11 kernel ring: Gfx11Queues::submit/retire on module ring index
// ring with fences (the FenceManager it was enabled with, same ring index).
struct SchedulerGfx11Target final {
    Gfx11Queues* queues;
    FenceManager* fences;
    uint32_t ring;
};
SchedulerBackend scheduler_gfx11_backend(SchedulerGfx11Target& target) noexcept;

// SDMA6 kernel ring of instance (fence ring fence_ring_sdma0 + instance).
struct SchedulerSdma6Target final {
    Sdma6* sdma;
    FenceManager* fences;
    uint32_t instance;
};
SchedulerBackend scheduler_sdma6_backend(SchedulerSdma6Target& target) noexcept;

// A MES user queue (IB at a process VA); poll runs UserQueues::poll.
struct SchedulerUserQueueTarget final {
    UserQueues* queues;
    uint32_t queue_id;
};
SchedulerBackend scheduler_user_queue_backend(SchedulerUserQueueTarget& target) noexcept;

} // namespace rx7900::hardware

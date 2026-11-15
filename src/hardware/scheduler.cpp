/*
Module: rx7900.hardware.scheduler
Purpose: Run the bounded drm_sched-style job scheduler and its kernel-ring and user-queue backends.
Created: 2026-10-22
Architecture: Fixed job/entity/target tables; backends own ring writes, fences and
timeouts (see scheduler.hpp).
Copyright 2015 Advanced Micro Devices, Inc.; Copyright 2009 Jerome Glisse.
Adapted Linux v6.12 sched_main.c, sched_entity.c, sched_fence.c, gpu_scheduler.h,
amdgpu_job.c, amdgpu_device.c and amdgpu_drv.c (see scheduler.hpp). Notice and
provenance: licenses/AMDGPU-SDMA.txt.
*/
#include "scheduler.hpp"

namespace rx7900::hardware {
namespace {

bool finished(SchedulerJobState state) noexcept {
    return state == SchedulerJobState::completed || state == SchedulerJobState::cancelled ||
           state == SchedulerJobState::timed_out || state == SchedulerJobState::failed;
}

// Kernel-ring and user-queue backend callbacks.
SchedulerRunResult gfx11_run(void* context, const SchedulerIb& ib, uint32_t& sequence) noexcept {
    const auto& target = *static_cast<const SchedulerGfx11Target*>(context);
    const Gfx11QueuesStatus status =
        target.queues->submit({target.ring, ib.ib, ib.offset, ib.dwords}, sequence);
    return status == Gfx11QueuesStatus::running ? SchedulerRunResult::submitted :
           status == Gfx11QueuesStatus::ring_full ? SchedulerRunResult::full :
           status == Gfx11QueuesStatus::ring_hung || status == Gfx11QueuesStatus::quarantined ?
               SchedulerRunResult::hung : SchedulerRunResult::failed;
}

SchedulerFenceResult fence_result(FenceStatus status) noexcept {
    return status == FenceStatus::signalled ? SchedulerFenceResult::signalled :
           status == FenceStatus::pending ? SchedulerFenceResult::pending :
           status == FenceStatus::ring_hung ? SchedulerFenceResult::hung :
                                              SchedulerFenceResult::failed;
}

SchedulerFenceResult gfx11_query(void* context, uint32_t sequence) noexcept {
    const auto& target = *static_cast<const SchedulerGfx11Target*>(context);
    return fence_result(target.fences->query(target.ring, sequence));
}

void gfx11_poll(void* context) noexcept {
    const auto& target = *static_cast<const SchedulerGfx11Target*>(context);
    target.fences->poll();
    target.queues->retire();
}

SchedulerRunResult sdma6_run(void* context, const SchedulerIb& ib, uint32_t& sequence) noexcept {
    const auto& target = *static_cast<const SchedulerSdma6Target*>(context);
    const Sdma6Status status =
        target.sdma->submit({target.instance, ib.ib, ib.offset, ib.dwords}, sequence);
    return status == Sdma6Status::started ? SchedulerRunResult::submitted :
           status == Sdma6Status::ring_full ? SchedulerRunResult::full :
           status == Sdma6Status::ring_hung || status == Sdma6Status::quarantined ?
               SchedulerRunResult::hung : SchedulerRunResult::failed;
}

SchedulerFenceResult sdma6_query(void* context, uint32_t sequence) noexcept {
    const auto& target = *static_cast<const SchedulerSdma6Target*>(context);
    return fence_result(target.fences->query(fence_ring_sdma0 + target.instance, sequence));
}

void sdma6_poll(void* context) noexcept {
    const auto& target = *static_cast<const SchedulerSdma6Target*>(context);
    target.fences->poll();
    target.sdma->retire();
}

SchedulerRunResult user_run(void* context, const SchedulerIb& ib, uint32_t& sequence) noexcept {
    const auto& target = *static_cast<const SchedulerUserQueueTarget*>(context);
    const UserQueueStatus status = target.queues->submit(target.queue_id, {ib.va, ib.dwords},
                                                         sequence);
    return status == UserQueueStatus::submitted ? SchedulerRunResult::submitted :
           status == UserQueueStatus::ring_full || status == UserQueueStatus::suspended ?
               SchedulerRunResult::full :
           status == UserQueueStatus::hung || status == UserQueueStatus::quarantined ?
               SchedulerRunResult::hung : SchedulerRunResult::failed;
}

SchedulerFenceResult user_query(void* context, uint32_t sequence) noexcept {
    const auto& target = *static_cast<const SchedulerUserQueueTarget*>(context);
    const UserQueueStatus status = target.queues->query(target.queue_id, sequence);
    return status == UserQueueStatus::signalled ? SchedulerFenceResult::signalled :
           status == UserQueueStatus::pending ? SchedulerFenceResult::pending :
           status == UserQueueStatus::hung || status == UserQueueStatus::quarantined ?
               SchedulerFenceResult::hung : SchedulerFenceResult::failed;
}

void user_poll(void* context) noexcept {
    static_cast<const SchedulerUserQueueTarget*>(context)->queues->poll();
}

} // namespace

SchedulerBackend scheduler_gfx11_backend(SchedulerGfx11Target& target) noexcept {
    return {&target, gfx11_run, gfx11_query, gfx11_poll};
}

SchedulerBackend scheduler_sdma6_backend(SchedulerSdma6Target& target) noexcept {
    return {&target, sdma6_run, sdma6_query, sdma6_poll};
}

SchedulerBackend scheduler_user_queue_backend(SchedulerUserQueueTarget& target) noexcept {
    return {&target, user_run, user_query, user_poll};
}

SchedulerStatus Scheduler::register_target(const SchedulerBackend& backend,
                                           uint32_t& target) noexcept {
    target = UINT32_MAX;

    if (backend.run == nullptr || backend.query == nullptr) {
        return SchedulerStatus::invalid_argument;
    }

    for (uint32_t index = 0; index < scheduler_max_targets; ++index) {
        if (!targets_[index].used) {
            targets_[index] = {true, backend, {}};
            targets_[index].report.registered = true;
            target = index;
            return SchedulerStatus::registered;
        }
    }

    return SchedulerStatus::full;
}

SchedulerStatus Scheduler::create_entity(uint32_t target, SchedulerPriority priority,
                                         uint32_t& entity) noexcept {
    entity = UINT32_MAX;

    if (target >= scheduler_max_targets || !targets_[target].used ||
        static_cast<uint32_t>(priority) > static_cast<uint32_t>(SchedulerPriority::low)) {
        return SchedulerStatus::invalid_argument;
    }

    for (uint32_t index = 0; index < scheduler_max_entities; ++index) {
        if (!entities_[index].used) {
            entities_[index] = {true, target, priority};
            entity = index;
            return SchedulerStatus::created;
        }
    }

    return SchedulerStatus::full;
}

bool Scheduler::find(uint64_t job_id, uint32_t& slot) const noexcept {
    for (slot = 0; slot < scheduler_max_jobs; ++slot) {
        if (slots_[slot].report.state != SchedulerJobState::free && slots_[slot].report.id == job_id &&
            job_id != 0) {
            return true;
        }
    }

    return false;
}

void Scheduler::finish(Slot& slot, SchedulerJobState state) noexcept {
    SchedulerTargetReport& target = targets_[slot.report.target].report;

    if (slot.report.state == SchedulerJobState::running && target.running > 0) {
        --target.running;
    }

    slot.report.state = state;
    target.completed += state == SchedulerJobState::completed ? 1 : 0;
    target.cancelled += state == SchedulerJobState::cancelled ? 1 : 0;
    target.timed_out += state == SchedulerJobState::timed_out ? 1 : 0;
    target.failed += state == SchedulerJobState::failed ? 1 : 0;
}

SchedulerStatus Scheduler::destroy_entity(uint32_t entity) noexcept {
    if (entity >= scheduler_max_entities || !entities_[entity].used) {
        return SchedulerStatus::not_found;
    }

    // drm_sched_entity_kill: drm_sched_fence_finished(-ESRCH) for queued jobs.
    for (auto& slot : slots_) {
        if (slot.report.state == SchedulerJobState::queued && slot.report.entity == entity) {
            finish(slot, SchedulerJobState::cancelled);
        }
    }

    entities_[entity] = {};
    return SchedulerStatus::destroyed;
}

SchedulerStatus Scheduler::push(const SchedulerJob& job, uint64_t& job_id) noexcept {
    job_id = 0;

    if (job.entity >= scheduler_max_entities || !entities_[job.entity].used ||
        job.ib.dwords == 0 || job.dependency_count > scheduler_max_dependencies) {
        return SchedulerStatus::invalid_argument;
    }

    const uint32_t target = entities_[job.entity].target;

    for (uint32_t index = 0; index < job.dependency_count; ++index) {
        const SchedulerDependency& dependency = job.dependencies[index];
        uint32_t slot = 0;
        const bool valid = dependency.kind == SchedulerDependencyKind::job ?
            find(dependency.job, slot) :
            dependency.kind == SchedulerDependencyKind::target_fence &&
                dependency.target < scheduler_max_targets && targets_[dependency.target].used;

        if (!valid) {
            return SchedulerStatus::invalid_argument;
        }
    }

    if (targets_[target].report.hung || targets_[target].report.stopped) {
        return SchedulerStatus::hung;
    }

    for (auto& slot : slots_) {
        if (slot.report.state == SchedulerJobState::free) {
            slot.job = job;
            slot.report = {SchedulerJobState::queued, next_id_, job.entity, target, 0};
            job_id = next_id_++;
            return SchedulerStatus::queued;
        }
    }

    return SchedulerStatus::full;
}

// Hung target: amdgpu_job_timedout sets -ETIME on the oldest pending job; the
// ring is quarantined, so later pending jobs never finish (cancelled here) and
// nothing queued may run (cancelled, as amdgpu_job_run skips errored jobs).
void Scheduler::cancel_queued(uint32_t target) noexcept {
    for (auto& slot : slots_) {
        if (slot.report.state == SchedulerJobState::queued && slot.report.target == target) {
            finish(slot, SchedulerJobState::cancelled);
        }
    }
}

// drm_sched_get_finished_job in pending-list (submission) order.
void Scheduler::retire(uint32_t target) noexcept {
    Target& owner = targets_[target];
    uint64_t last = 0;
    bool guilty_found = false;

    for (;;) {
        Slot* next = nullptr;

        for (auto& slot : slots_) {
            if (slot.report.state == SchedulerJobState::running && slot.report.target == target &&
                slot.report.id > last && (next == nullptr || slot.report.id < next->report.id)) {
                next = &slot;
            }
        }

        if (next == nullptr) {
            break;
        }

        last = next->report.id;

        if (owner.report.hung) {
            finish(*next, guilty_found ? SchedulerJobState::cancelled : SchedulerJobState::timed_out);
            guilty_found = true;
            continue;
        }

        const SchedulerFenceResult result = owner.backend.query(owner.backend.context,
                                                                next->report.sequence);

        if (result == SchedulerFenceResult::signalled) {
            finish(*next, SchedulerJobState::completed);
        } else if (result != SchedulerFenceResult::pending) {
            // The backend quarantined the ring (or the fence is unknown: fail closed).
            owner.report.hung = true;
            finish(*next, SchedulerJobState::timed_out);
            guilty_found = true;
        }
    }

    if (owner.report.hung || owner.report.stopped) {
        cancel_queued(target);
    }
}

bool Scheduler::ready(const Slot& slot) noexcept {
    for (uint32_t index = 0; index < slot.job.dependency_count; ++index) {
        const SchedulerDependency& dependency = slot.job.dependencies[index];

        if (dependency.kind == SchedulerDependencyKind::job) {
            uint32_t other = 0;

            if (!find(dependency.job, other) ||
                slots_[other].report.state != SchedulerJobState::completed) {
                return false;
            }
        } else {
            const Target& target = targets_[dependency.target];

            if (target.backend.query(target.backend.context, dependency.sequence) !=
                SchedulerFenceResult::signalled) {
                return false;
            }
        }
    }

    return true;
}

// Fail closed: a job whose dependency finished with an error, or whose hardware
// fence belongs to a hung ring, is cancelled instead of run (Linux v6.12 only
// skips jobs of a guilty entity; see the module report).
bool Scheduler::resolve_dependencies() noexcept {
    bool changed = false;

    for (auto& slot : slots_) {
        if (slot.report.state != SchedulerJobState::queued) {
            continue;
        }

        for (uint32_t index = 0; index < slot.job.dependency_count; ++index) {
            const SchedulerDependency& dependency = slot.job.dependencies[index];
            bool failed = false;

            if (dependency.kind == SchedulerDependencyKind::job) {
                uint32_t other = 0;
                const SchedulerJobState state = find(dependency.job, other) ?
                    slots_[other].report.state : SchedulerJobState::cancelled;
                failed = finished(state) && state != SchedulerJobState::completed;
            } else {
                const Target& target = targets_[dependency.target];
                const SchedulerFenceResult result =
                    target.backend.query(target.backend.context, dependency.sequence);
                failed = result == SchedulerFenceResult::hung ||
                         result == SchedulerFenceResult::failed;
            }

            if (failed) {
                finish(slot, SchedulerJobState::cancelled);
                changed = true;
                break;
            }
        }
    }

    return changed;
}

// The oldest queued job of an entity (its spsc job_queue head).
bool Scheduler::head(uint32_t entity, uint32_t& found) const noexcept {
    bool any = false;

    for (uint32_t slot = 0; slot < scheduler_max_jobs; ++slot) {
        const SchedulerJobReport& report = slots_[slot].report;

        if (report.state == SchedulerJobState::queued && report.entity == entity &&
            (!any || report.id < slots_[found].report.id)) {
            found = slot;
            any = true;
        }
    }

    return any;
}

void Scheduler::submit(uint32_t target) noexcept {
    Target& owner = targets_[target];

    // drm_sched_can_queue: one credit per job, credit_limit scheduler_hw_submission.
    while (!owner.report.hung && !owner.report.stopped &&
           owner.report.running < scheduler_hw_submission) {
        uint32_t chosen = UINT32_MAX;

        // drm_sched_select_entity: priorities KERNEL..LOW, FIFO by submit order
        // inside a priority; an entity whose head waits on a dependency is skipped.
        for (uint32_t entity = 0; entity < scheduler_max_entities; ++entity) {
            uint32_t slot = 0;

            if (!entities_[entity].used || entities_[entity].target != target ||
                !head(entity, slot) || !ready(slots_[slot])) {
                continue;
            }

            if (chosen == UINT32_MAX) {
                chosen = slot;
                continue;
            }

            const uint32_t priority = static_cast<uint32_t>(entities_[entity].priority);
            const uint32_t best = static_cast<uint32_t>(
                entities_[slots_[chosen].report.entity].priority);

            if (priority < best || (priority == best &&
                                    slots_[slot].report.id < slots_[chosen].report.id)) {
                chosen = slot;
            }
        }

        if (chosen == UINT32_MAX) {
            return;
        }

        Slot& slot = slots_[chosen];
        uint32_t sequence = 0;
        const SchedulerRunResult result = owner.backend.run(owner.backend.context, slot.job.ib,
                                                            sequence);

        if (result == SchedulerRunResult::full) {
            return;
        }

        if (result == SchedulerRunResult::submitted) {
            slot.report.state = SchedulerJobState::running;
            slot.report.sequence = sequence;
            ++owner.report.running;
            ++owner.report.submitted;
            continue;
        }

        if (result == SchedulerRunResult::failed) {
            finish(slot, SchedulerJobState::failed);
            continue;
        }

        // The ring went hung under this submission: nothing of this job reached it.
        owner.report.hung = true;
        finish(slot, SchedulerJobState::cancelled);
        retire(target);
        return;
    }
}

SchedulerStatus Scheduler::run() noexcept {
    for (const auto& target : targets_) {
        if (target.used && target.backend.poll != nullptr) {
            target.backend.poll(target.backend.context);
        }
    }

    for (uint32_t target = 0; target < scheduler_max_targets; ++target) {
        if (targets_[target].used) {
            retire(target);
        }
    }

    // Cancellation can cascade along dependency chains; each round finishes at
    // least one job, so scheduler_max_jobs rounds reach the fixpoint.
    for (uint32_t round = 0; round < scheduler_max_jobs && resolve_dependencies(); ++round) {
    }

    for (uint32_t target = 0; target < scheduler_max_targets; ++target) {
        if (targets_[target].used) {
            submit(target);
        }
    }

    return SchedulerStatus::ready;
}

SchedulerStatus Scheduler::cancel_target(uint32_t target) noexcept {
    if (target >= scheduler_max_targets || !targets_[target].used) {
        return SchedulerStatus::not_found;
    }

    targets_[target].report.stopped = true;
    cancel_queued(target);
    return SchedulerStatus::ready;
}

SchedulerStatus Scheduler::release_after_reset() noexcept {
    for (auto& slot : slots_) {
        if (slot.report.state == SchedulerJobState::queued ||
            slot.report.state == SchedulerJobState::running) {
            finish(slot, SchedulerJobState::cancelled);
        }
    }

    for (auto& target : targets_) {
        target = {};
    }

    for (auto& entity : entities_) {
        entity = {};
    }

    return SchedulerStatus::released;
}

SchedulerJobReport Scheduler::job(uint64_t job_id) const noexcept {
    uint32_t slot = 0;
    return find(job_id, slot) ? slots_[slot].report : SchedulerJobReport{};
}

SchedulerStatus Scheduler::release_job(uint64_t job_id) noexcept {
    uint32_t slot = 0;

    if (!find(job_id, slot)) {
        return SchedulerStatus::not_found;
    }

    if (!finished(slots_[slot].report.state)) {
        return SchedulerStatus::invalid_argument;
    }

    for (const auto& other : slots_) {
        if (other.report.state != SchedulerJobState::queued) {
            continue;
        }

        for (uint32_t index = 0; index < other.job.dependency_count; ++index) {
            if (other.job.dependencies[index].kind == SchedulerDependencyKind::job &&
                other.job.dependencies[index].job == job_id) {
                return SchedulerStatus::busy;
            }
        }
    }

    slots_[slot] = {};
    return SchedulerStatus::released;
}

SchedulerReport Scheduler::report() const noexcept {
    SchedulerReport result{};

    for (uint32_t index = 0; index < scheduler_max_targets; ++index) {
        result.targets[index] = targets_[index].report;
        result.reset_required = result.reset_required || targets_[index].report.hung;
    }

    for (const auto& slot : slots_) {
        result.queued += slot.report.state == SchedulerJobState::queued ? 1 : 0;
        result.running += slot.report.state == SchedulerJobState::running ? 1 : 0;
        result.finished += finished(slot.report.state) ? 1 : 0;
    }

    return result;
}

} // namespace rx7900::hardware

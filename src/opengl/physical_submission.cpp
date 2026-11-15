/*
Module: rx7900.opengl.physical_submission
Purpose: Bounded shader/vertex/target leases for trusted physical graphics work.
Created: 2026-10-23
Architecture: Reuses Manager lease semantics and GFX11/FenceManager receipts.
*/
#include "physical_submission.hpp"

namespace rx7900::opengl {
namespace {

bool same_handle(memory::Handle a, memory::Handle b) noexcept {
    return a.slot == b.slot && a.generation == b.generation;
}

bool valid_access(memory::Access access) noexcept {
    return access == memory::Access::read || access == memory::Access::write ||
           access == memory::Access::read_write;
}

} // namespace

GraphicsJobStatus PhysicalGraphicsSubmission::initialize(
    memory::Manager& manager, GraphicsSubmitter submitter) noexcept {
    if (manager_ != nullptr || pending()) {
        return GraphicsJobStatus::busy;
    }
    if (submitter.submit == nullptr || submitter.query == nullptr ||
        submitter.retire == nullptr || submitter.retired_after_reset == nullptr) {
        return GraphicsJobStatus::invalid_argument;
    }
    manager_ = &manager;
    submitter_ = submitter;
    return GraphicsJobStatus::ready;
}

GraphicsJobStatus PhysicalGraphicsSubmission::release_resources() noexcept {
    while (retained_count_ != 0) {
        GraphicsResource& resource = retained_[retained_count_ - 1];
        if (manager_->unretain(resource.slice.allocation) != memory::Status::success) {
            quarantined_ = true;
            return GraphicsJobStatus::quarantined;
        }
        resource = {};
        --retained_count_;
    }
    pending_ = false;
    sequence_ = 0;
    return GraphicsJobStatus::signalled;
}

GraphicsJobStatus PhysicalGraphicsSubmission::submit(const TrustedGraphicsJob& job,
                                                      uint32_t& sequence) noexcept {
    sequence = 0;
    if (manager_ == nullptr) {
        return GraphicsJobStatus::unavailable;
    }
    if (quarantined_) {
        return GraphicsJobStatus::quarantined;
    }
    if (pending_) {
        const GraphicsJobStatus previous = poll();
        if (previous != GraphicsJobStatus::signalled) {
            return previous;
        }
    }
    if (job.ib_dwords == 0 || (job.ib_offset & 3u) != 0 ||
        job.resource_count == 0 ||
        job.resource_count > graphics_job_resource_capacity) {
        return GraphicsJobStatus::invalid_argument;
    }
    for (uint32_t index = 0; index < job.resource_count; ++index) {
        const GraphicsResource& resource = job.resources[index];
        if (!valid_access(resource.access) || resource.byte_count == 0 ||
            same_handle(resource.slice.allocation, job.ib)) {
            return GraphicsJobStatus::invalid_argument;
        }
    }
    // CPU writes to an IB are owned by the trusted producer and serialized with
    // this object. Gfx11Queues::submit validates and retains the IB itself.
    for (uint32_t index = 0; index < job.resource_count; ++index) {
        const GraphicsResource resource = job.resources[index];
        sdma::GpuRange checked{};
        if (manager_->resolve(resource.slice, resource.byte_count,
                              resource.access, checked) != memory::Status::success ||
            manager_->retain(resource.slice.allocation, resource.access) !=
                memory::Status::success) {
            const auto released = release_resources();
            return released == GraphicsJobStatus::signalled ?
                GraphicsJobStatus::invalid_memory : released;
        }
        retained_[retained_count_++] = resource;
    }

    uint32_t submitted_sequence = 0;
    const hardware::Gfx11Submission frame{hardware::fence_ring_gfx, job.ib,
                                           job.ib_offset, job.ib_dwords,
                                           job.in_vm ? &job.vm : nullptr};
    const auto status = submitter_.submit(submitter_.context, frame, submitted_sequence);
    if (status == hardware::Gfx11QueuesStatus::running && submitted_sequence != 0) {
        pending_ = true;
        sequence_ = submitted_sequence;
        sequence = submitted_sequence;
        return GraphicsJobStatus::submitted;
    }
    if (status == hardware::Gfx11QueuesStatus::ring_full && submitted_sequence == 0) {
        const auto released = release_resources();
        return released == GraphicsJobStatus::signalled ? GraphicsJobStatus::busy : released;
    }
    // Gfx11Queues refuses these before the frame reaches the CP (submission
    // disabled/stopped, ring already hung, invalid frame or VMID inputs, or a
    // fence publication that was undone): nothing can execute the IB.
    if (submitted_sequence == 0 && (status == hardware::Gfx11QueuesStatus::unavailable ||
                                    status == hardware::Gfx11QueuesStatus::busy ||
                                    status == hardware::Gfx11QueuesStatus::ring_hung ||
                                    status == hardware::Gfx11QueuesStatus::invalid_argument)) {
        const auto released = release_resources();
        return released == GraphicsJobStatus::signalled ? GraphicsJobStatus::unavailable : released;
    }
    // An error after fence publication/doorbell attempt may still execute the
    // IB. A zero output sequence is not proof that the GPU never saw it.
    pending_ = true;
    quarantined_ = true;
    return GraphicsJobStatus::quarantined;
}

GraphicsJobStatus PhysicalGraphicsSubmission::poll() noexcept {
    if (manager_ == nullptr) {
        return GraphicsJobStatus::unavailable;
    }
    if (quarantined_) {
        return GraphicsJobStatus::quarantined;
    }
    if (!pending_) {
        return GraphicsJobStatus::ready;
    }
    const auto fence = submitter_.query(submitter_.context, sequence_);
    if (fence == hardware::FenceStatus::pending) {
        return GraphicsJobStatus::busy;
    }
    if (fence != hardware::FenceStatus::signalled ||
        submitter_.retire(submitter_.context) != hardware::Gfx11QueuesStatus::running) {
        quarantined_ = true;
        return GraphicsJobStatus::quarantined;
    }
    return release_resources();
}

GraphicsJobStatus PhysicalGraphicsSubmission::query(uint32_t sequence) noexcept {
    if (sequence == 0 || !pending_ || sequence != sequence_) {
        return GraphicsJobStatus::invalid_argument;
    }
    return poll();
}

GraphicsJobStatus PhysicalGraphicsSubmission::close_idle() noexcept {
    if (manager_ == nullptr) {
        return GraphicsJobStatus::unavailable;
    }
    const auto status = poll();
    if (status == GraphicsJobStatus::busy || status == GraphicsJobStatus::quarantined) {
        return status;
    }
    if (retained_count_ != 0) {
        return GraphicsJobStatus::quarantined;
    }
    manager_ = nullptr;
    submitter_ = {};
    return GraphicsJobStatus::ready;
}

GraphicsJobStatus PhysicalGraphicsSubmission::retire_after_reset() noexcept {
    if (manager_ == nullptr) {
        return GraphicsJobStatus::unavailable;
    }
    if (!submitter_.retired_after_reset(submitter_.context)) {
        return GraphicsJobStatus::busy;
    }
    quarantined_ = false;
    return release_resources();
}

} // namespace rx7900::opengl

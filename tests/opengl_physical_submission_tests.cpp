/*
Module: rx7900.tests.opengl_physical_submission
Purpose: Check physical graphics leases against fence and reset boundaries.
Created: 2026-10-23
Architecture: Synthetic queue receipts with the production memory Manager.
*/
#include "../src/opengl/physical_submission.hpp"
#include "memory_test_backend.hpp"

#include <cstdio>

using namespace rx7900;

namespace {
unsigned checks = 0;
unsigned failures = 0;

void expect(bool valid, const char* label) {
    ++checks;
    if (!valid) {
        ++failures;
        std::printf("FAIL: %s\n", label);
    }
}

struct Queue final {
    memory::Manager* manager = nullptr;
    memory::Handle ib{};
    hardware::Gfx11QueuesStatus submit_result = hardware::Gfx11QueuesStatus::running;
    hardware::FenceStatus fence_result = hardware::FenceStatus::pending;
    bool reset_retired = false;
    bool ib_held = false;
    uint32_t submissions = 0;

    bool vm_seen = false;
    hardware::Gfx11VmJob vm{};

    static hardware::Gfx11QueuesStatus submit(void* context,
                                              const hardware::Gfx11Submission& frame,
                                              uint32_t& sequence) noexcept {
        Queue& self = *static_cast<Queue*>(context);
        sequence = 0;
        self.vm_seen = frame.vm != nullptr;
        if (frame.vm != nullptr) self.vm = *frame.vm;
        if (self.submit_result != hardware::Gfx11QueuesStatus::running) {
            return self.submit_result;
        }
        if (frame.ring != hardware::fence_ring_gfx || frame.ib_dwords == 0 ||
            self.manager->retain(frame.ib, memory::Access::read) != memory::Status::success) {
            return hardware::Gfx11QueuesStatus::invalid_memory;
        }
        self.ib = frame.ib;
        self.ib_held = true;
        sequence = ++self.submissions;
        return hardware::Gfx11QueuesStatus::running;
    }

    static hardware::FenceStatus query(void* context, uint32_t) noexcept {
        return static_cast<Queue*>(context)->fence_result;
    }

    static hardware::Gfx11QueuesStatus retire(void* context) noexcept {
        Queue& self = *static_cast<Queue*>(context);
        if (self.ib_held) {
            if (self.manager->unretain(self.ib) != memory::Status::success) {
                return hardware::Gfx11QueuesStatus::quarantined;
            }
            self.ib_held = false;
        }
        return hardware::Gfx11QueuesStatus::running;
    }

    static bool reset(void* context) noexcept {
        return static_cast<Queue*>(context)->reset_retired;
    }

    opengl::GraphicsSubmitter callbacks() noexcept {
        return {this, submit, query, retire, reset};
    }
};

struct Fixture final {
    rx7900_test::TestBacking backing{};
    memory::Manager manager{};
    Queue queue{};
    opengl::PhysicalGraphicsSubmission owner{};
    memory::Handle ib{}, shader{}, vertex{}, target{};

    void initialize() {
        expect(manager.initialize({0x100000000ULL, 0x100000}, backing.backend()) ==
               memory::Status::success, "manager initialized");
        for (auto* handle : {&ib, &shader, &vertex, &target}) {
            expect(manager.allocate(memory::page_bytes, memory::page_bytes,
                memory::Access::read_write, true, *handle) == memory::Status::success,
                "GPU-backed allocation created");
        }
        queue.manager = &manager;
        expect(owner.initialize(manager, queue.callbacks()) ==
               opengl::GraphicsJobStatus::ready, "graphics owner initialized");
    }

    opengl::TrustedGraphicsJob job() const noexcept {
        opengl::TrustedGraphicsJob value{};
        value.ib = ib;
        value.ib_dwords = 32;
        value.resources[0] = {{shader, 0}, 256, memory::Access::read};
        value.resources[1] = {{vertex, 0}, 256, memory::Access::read};
        value.resources[2] = {{target, 0}, 1024, memory::Access::write};
        value.resource_count = 3;
        return value;
    }

    void finish() {
        expect(owner.close_idle() == opengl::GraphicsJobStatus::ready,
               "idle owner closes");
        for (auto handle : {ib, shader, vertex, target}) {
            const memory::Status status = manager.release(handle);
            expect(status == memory::Status::success ||
                   status == memory::Status::stale_handle,
                   "allocation retires after fence ownership");
        }
        expect(manager.shutdown() == memory::Status::success &&
               backing.creates == backing.destroys,
               "all backing and GPUVA leases returned");
    }
};

void completed_submission() {
    Fixture fixture{};
    fixture.initialize();
    uint32_t sequence = 0;
    expect(fixture.owner.submit(fixture.job(), sequence) ==
           opengl::GraphicsJobStatus::submitted && sequence == 1 &&
           fixture.queue.ib_held, "trusted work uses real-style queue IB lease");
    memory::AllocationInfo info{};
    expect(fixture.manager.describe(fixture.shader, info) == memory::Status::success &&
           info.leases == 1, "shader stays leased while fence is pending");
    expect(fixture.manager.describe(fixture.ib, info) == memory::Status::success &&
           info.leases == 1, "IB has only the queue's lease");
    expect(fixture.owner.query(sequence) == opengl::GraphicsJobStatus::busy &&
           fixture.owner.close_idle() == opengl::GraphicsJobStatus::busy,
           "pending fence blocks resource release");
    fixture.queue.fence_result = hardware::FenceStatus::signalled;
    expect(fixture.owner.query(sequence) == opengl::GraphicsJobStatus::signalled &&
           !fixture.queue.ib_held, "actual fence and queue retirement release leases");
    expect(fixture.manager.describe(fixture.shader, info) == memory::Status::success &&
           info.leases == 0, "shader lease returns after completion");
    fixture.finish();
}

void refused_and_uncertain_submission() {
    Fixture fixture{};
    fixture.initialize();
    uint32_t sequence = 99;
    auto work = fixture.job();
    work.resource_count = opengl::graphics_job_resource_capacity + 1;
    expect(fixture.owner.submit(work, sequence) ==
           opengl::GraphicsJobStatus::invalid_argument && sequence == 0,
           "oversized resource list rejected before indexing");
    work = fixture.job();
    work.resources[0].slice.allocation = fixture.ib;
    expect(fixture.owner.submit(work, sequence) ==
           opengl::GraphicsJobStatus::invalid_argument,
           "IB cannot alias a mutable shader/target resource");
    work = fixture.job();
    work.resources[0].slice.byte_offset = memory::page_bytes - 128;
    expect(fixture.owner.submit(work, sequence) ==
           opengl::GraphicsJobStatus::invalid_memory,
           "shader slice beyond live allocation rejected before queue");
    work = fixture.job();
    fixture.queue.submit_result = hardware::Gfx11QueuesStatus::ring_full;
    expect(fixture.owner.submit(work, sequence) == opengl::GraphicsJobStatus::busy,
           "known pre-publication ring-full refusal returns leases");
    memory::AllocationInfo info{};
    expect(fixture.manager.describe(fixture.shader, info) == memory::Status::success &&
           info.leases == 0, "refused job has no shader lease");

    fixture.queue.submit_result = hardware::Gfx11QueuesStatus::doorbell_failure;
    expect(fixture.owner.submit(work, sequence) ==
           opengl::GraphicsJobStatus::quarantined && fixture.owner.pending(),
           "uncertain doorbell publication retains resources");
    expect(fixture.manager.describe(fixture.shader, info) == memory::Status::success &&
           info.leases == 1, "uncertain job retains shader until reset proof");
    expect(fixture.owner.retire_after_reset() == opengl::GraphicsJobStatus::busy,
           "unproven reset cannot release resource");
    fixture.queue.reset_retired = true;
    expect(fixture.owner.retire_after_reset() ==
           opengl::GraphicsJobStatus::signalled,
           "confirmed queue/fence retirement releases resource");
    fixture.finish();
}

// A process-VM job reaches the queue with its VM; refusals the queue answers
// before anything reached the CP return every resource lease at once.
void vm_jobs_and_clean_refusals() {
    Fixture fixture{};
    fixture.initialize();
    uint32_t sequence = 0;
    auto work = fixture.job();
    work.in_vm = true;
    work.vm = {0x8123, 0x0000000400000001ULL, 0x0000200100000000ULL};
    expect(fixture.owner.submit(work, sequence) == opengl::GraphicsJobStatus::submitted &&
           fixture.queue.vm_seen && fixture.queue.vm.pasid == 0x8123 &&
           fixture.queue.vm.pd_address == 0x0000000400000001ULL && fixture.queue.vm.ib_va == 0x0000200100000000ULL,
           "the job's PASID, page directory and IB VA reach Gfx11Queues");
    fixture.queue.fence_result = hardware::FenceStatus::signalled;
    expect(fixture.owner.poll() == opengl::GraphicsJobStatus::signalled, "receipt releases");
    fixture.queue.fence_result = hardware::FenceStatus::pending;
    work = fixture.job();
    expect(fixture.owner.submit(work, sequence) == opengl::GraphicsJobStatus::submitted && !fixture.queue.vm_seen,
           "a VMID 0 job carries no VM");
    fixture.queue.fence_result = hardware::FenceStatus::signalled;
    fixture.owner.poll();
    memory::AllocationInfo info{};
    for (const auto refused : {hardware::Gfx11QueuesStatus::unavailable, hardware::Gfx11QueuesStatus::busy,
                               hardware::Gfx11QueuesStatus::ring_hung,
                               hardware::Gfx11QueuesStatus::invalid_argument}) {
        fixture.queue.submit_result = refused;
        expect(fixture.owner.submit(work, sequence) == opengl::GraphicsJobStatus::unavailable &&
               !fixture.owner.pending() &&
               fixture.manager.describe(fixture.shader, info) == memory::Status::success && info.leases == 0,
               "a refusal before the CP saw the frame is unavailable and keeps no lease");
    }
    fixture.queue.submit_result = hardware::Gfx11QueuesStatus::invalid_memory;
    expect(fixture.owner.submit(work, sequence) == opengl::GraphicsJobStatus::quarantined &&
           fixture.manager.describe(fixture.shader, info) == memory::Status::success && info.leases == 1,
           "an ambiguous invalid_memory (also a failed ring store) quarantines and keeps the lease");
    fixture.queue.reset_retired = true;
    fixture.owner.retire_after_reset();
    fixture.finish();

    // The production bridge: without queues it refuses cleanly; inside the
    // device owner's confirmed-reset window it reports retirement.
    bool window = false;
    opengl::KernelGraphicsBridge bridge{};
    bridge.reset_context = &window;
    bridge.reset_confirmed = [](void* c) noexcept { return *static_cast<bool*>(c); };
    const auto submitter = bridge.submitter();
    const hardware::Gfx11Submission frame{hardware::fence_ring_gfx, {}, 0, 4, &work.vm};
    expect(submitter.submit(submitter.context, frame, sequence) == hardware::Gfx11QueuesStatus::unavailable &&
           sequence == 0 && submitter.query(submitter.context, 1) == hardware::FenceStatus::unavailable &&
           submitter.retire(submitter.context) == hardware::Gfx11QueuesStatus::unavailable,
           "bridge without queues: unavailable, no sequence");
    expect(!submitter.retired_after_reset(submitter.context), "no reset proof, no released queues: not retired");
    window = true;
    expect(submitter.retired_after_reset(submitter.context), "the confirmed reset window retires");
}
}

int main() {
    completed_submission();
    refused_and_uncertain_submission();
    vm_jobs_and_clean_refusals();
    std::printf("Physical GL submission: %u checks, %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

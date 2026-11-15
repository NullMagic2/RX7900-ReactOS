/*
Module: rx7900.tests.gpu_process_core
Purpose: Fault-inject the production native process-session state machine: exit
reaping with pending work, MES removal failure and escalation, reaping racing a
reset, destructive-reset reconstruction with lost VRAM and lost work reported,
second faults during reconstruction, handle non-aliasing and lock order.
Created: 2026-10-24
Architecture: Production GpuProcessCore with UserQueues, ProcessMemory,
ProcessSubmissions and ProcessGraphics over the shared synthetic MES/KIQ and
process-engine model (user_queue_fixture.hpp). The test plays the native owner:
its locks, the confirmed-reset window and the ASIC restart. No NT, no hardware.
*/
#include <algorithm>
#include <memory>
#include <vector>

#include "user_queue_fixture.hpp"
#include "../src/reactos/gpu_process_core.hpp"
#include "../src/opengl/gl_shader_gfx11.hpp"

namespace {
using namespace rx7900::reactos;
using namespace rx7900_test::user_queue_fixture;
using process_abi::Operation;
using process_abi::Outcome;
using rx7900::memory::Access;

rx7900_test::Checks checks;
void expect(bool value, const char* description) { checks.expect(value, description); }

constexpr uint64_t gtt_va = 0x400000;
constexpr uint64_t vram_va = 0x800000;
constexpr uint64_t other_va = 0xc00000;
constexpr uint32_t sdma_engine = 2;

struct Harness final {
    std::unique_ptr<Fixture> device;
    std::unique_ptr<GpuProcessCore> core = std::make_unique<GpuProcessCore>();
    int process_a = 0;
    int process_b = 0;
    int references = 0;
    unsigned dma_failures = 0;
    unsigned hook_calls = 0;
    unsigned lock_depth = 0;
    unsigned hook_depth_violations = 0;
    unsigned lock_acquisitions = 0;

    static bool dma_bus(void* context, rx7900::memory::Handle handle, uint64_t& bus) noexcept {
        auto& self = *static_cast<Harness*>(context);
        if (self.dma_failures != 0) {
            --self.dma_failures;
            return false;
        }
        rx7900::memory::AllocationInfo info{};
        if (self.device->gtt->describe(handle, info) != rx7900::memory::Status::success) return false;
        bus = bus_base + info.gpu.virtual_address - self.device->gart.start;
        return true;
    }
    static bool queue_config(void* context, PasidAllocator& pasids, UserQueueConfig& config) noexcept {
        auto& self = *static_cast<Harness*>(context);
        if (!self.device->mes->report().scheduler_ready) return false;
        config = self.device->config();
        config.pasids = &pasids;
        return true;
    }
    static uint64_t now_us(void* context) noexcept { return static_cast<Harness*>(context)->device->now; }
    static void reference(void* context, void*) noexcept { ++static_cast<Harness*>(context)->references; }
    static void dereference(void* context, void*) noexcept { --static_cast<Harness*>(context)->references; }
    static void acquire(void* context) noexcept {
        auto& self = *static_cast<Harness*>(context);
        ++self.lock_depth;
        ++self.lock_acquisitions;
    }
    static void release(void* context) noexcept { --static_cast<Harness*>(context)->lock_depth; }
    static void hook(void* context) noexcept {
        auto& self = *static_cast<Harness*>(context);
        ++self.hook_calls;
        if (self.lock_depth != 0) ++self.hook_depth_violations;
    }

    uint32_t api_timeout = 2100000;

    explicit Harness(uint32_t timeout = 2100000)
        : device(std::make_unique<Fixture>(0x00001063u, timeout)), api_timeout(timeout) {
        expect(device->up(), "synthetic MES scheduler and process queues ready");
        ProcessPlatform platform{};
        platform.context = this;
        platform.gtt = device->gtt.get();
        platform.vram = device->vram.get();
        platform.dma_bus = dma_bus;
        platform.queue_config = queue_config;
        platform.now_us = now_us;
        platform.reference = reference;
        platform.dereference = dereference;
        expect(process_core_bind(*core, platform), "production core binds the ready ASIC services");
        core->initialized = true;
        core->registered = true;
        core->recovery = {this, hook};
    }

    ProcessLock lock() noexcept { return {this, acquire, release}; }

    Outcome call(void* process, Operation op, uint64_t session, uint64_t handle = 0, uint64_t bytes = 0,
                 uint64_t offset = 0, uint64_t address = 0, uint32_t domain = 0, uint32_t access = 0,
                 uint32_t version = process_abi::recovery_version) {
        auto& c = *core;
        c.request = {sizeof(process_abi::Request), version, static_cast<uint32_t>(op), 0, session, handle,
                     bytes, offset, address, domain, access};
        if (!process_abi::valid_request_fields(c.request)) return Outcome::invalid_request;
        c.reply = {sizeof(process_abi::Reply), version, 0, 0, 0, 0, 0, 0};
        return process_core_execute(c, process);
    }

    uint64_t open(void* process) {
        const auto result = call(process, Operation::open, 0);
        expect(result == Outcome::success && core->reply.session != 0, "session opens for the process token");
        return core->reply.session;
    }

    uint64_t buffer(void* process, uint64_t session, ProcessMemoryDomain domain, uint64_t va,
                    uint64_t bytes = 8192) {
        expect(call(process, Operation::allocate, session, 0, bytes, 0, 0, static_cast<uint32_t>(domain), 3) ==
               Outcome::success, "owned buffer allocates");
        const uint64_t handle = core->reply.handle;
        expect(call(process, Operation::map, session, handle, 0, 0, va, 0, 3) == Outcome::success,
               "owned buffer maps at its client GPUVA");
        return handle;
    }

    uint64_t queue(void* process, uint64_t session) {
        expect(call(process, Operation::queue_create, session, 0, 0, 0, 0, sdma_engine) == Outcome::success,
               "SDMA user queue added through MES");
        return core->reply.handle;
    }

    Outcome write(void* process, uint64_t session, uint64_t handle, uint64_t offset, uint8_t seed,
                  uint32_t bytes = 64) {
        for (uint32_t index = 0; index < bytes; ++index) core->payload[index] = static_cast<uint8_t>(seed + index);
        return call(process, Operation::write, session, handle, bytes, offset);
    }

    Outcome read(void* process, uint64_t session, uint64_t handle, uint64_t offset, uint32_t bytes = 64,
                 uint32_t version = process_abi::recovery_version) {
        return call(process, Operation::read, session, handle, bytes, offset, 0, 0, 0, version);
    }

    bool payload_is(uint8_t seed, uint32_t bytes = 64) const {
        for (uint32_t index = 0; index < bytes; ++index) {
            if (core->payload[index] != static_cast<uint8_t>(seed + index)) return false;
        }
        return true;
    }

    void transfer_payload(uint64_t source, uint64_t destination, uint64_t source_offset,
                          uint64_t destination_offset, uint64_t bytes, uint32_t value) {
        const rx7900::sdma::CommandBuffer out(core->payload, sizeof(process_abi::Transfer));
        out.write_u32(0, static_cast<uint32_t>(source)); out.write_u32(4, static_cast<uint32_t>(source >> 32));
        out.write_u32(8, static_cast<uint32_t>(destination));
        out.write_u32(12, static_cast<uint32_t>(destination >> 32));
        out.write_u32(16, static_cast<uint32_t>(source_offset));
        out.write_u32(20, static_cast<uint32_t>(source_offset >> 32));
        out.write_u32(24, static_cast<uint32_t>(destination_offset));
        out.write_u32(28, static_cast<uint32_t>(destination_offset >> 32));
        out.write_u32(32, static_cast<uint32_t>(bytes)); out.write_u32(36, static_cast<uint32_t>(bytes >> 32));
        out.write_u32(40, value); out.write_u32(44, 0);
    }

    Outcome fill(void* process, uint64_t session, uint64_t queue, uint64_t destination, uint32_t value,
                 uint32_t& sequence, uint64_t offset = 0, uint64_t bytes = 64) {
        transfer_payload(0, destination, 0, offset, bytes, value);
        const auto result = call(process, Operation::fill, session, queue);
        sequence = static_cast<uint32_t>(core->reply.bytes);
        return result;
    }

    Outcome copy(void* process, uint64_t session, uint64_t queue, uint64_t source, uint64_t destination,
                 uint32_t& sequence) {
        transfer_payload(source, destination, 0, 0, 64, 0);
        const auto result = call(process, Operation::copy, session, queue);
        sequence = static_cast<uint32_t>(core->reply.bytes);
        return result;
    }

    Outcome query(void* process, uint64_t session, uint64_t queue, uint32_t sequence,
                  uint32_t version = process_abi::recovery_version) {
        return call(process, Operation::submission_query, session, queue, 0, sequence, 0, 0, 0, version);
    }

    GpuProcessCore::Session* session_of(void* process) {
        for (auto& session : core->sessions) {
            if (session.process == process) return &session;
        }
        return nullptr;
    }

    uint32_t doorbell_of(void* process, uint64_t queue_handle) {
        auto* session = session_of(process);
        if (session == nullptr) return UINT32_MAX;
        for (const auto& queue : session->queues) {
            if (queue.handle == queue_handle) return core->queues.queue_report(queue.id).doorbell_index;
        }
        return UINT32_MAX;
    }

    void hang(void* process, uint64_t queue) { device->engines.hang_doorbell = doorbell_of(process, queue); }

    ProcessReaperPass reap_pass() {
        const bool stop = false;
        return process_core_reaper_pass(*core, lock(), stop);
    }

    // The native reset window: begin (service mutex held), AsicInit's reset
    // proof, release_after_reset retiring the process owner and then MES.
    bool reset_window() {
        process_core_begin_reset(*core);
        device->reset_confirmed = true;
        const bool retired = process_core_retire_reset(*core, Fixture::confirm, &device->reset_confirmed);
        device->mes->release_after_reset();
        device->reset_confirmed = false;
        core->reset_locked = false; // end_reset
        return retired;
    }

    // Mode-1 reset effects on the model, then a new ASIC incarnation's MES.
    bool restart(bool lose_vram = true) {
        device->model.reset();
        device->model.ignore_opcode = 0xff;
        device->engines.pending.clear();
        device->engines.hang_doorbell = UINT32_MAX;
        if (lose_vram) std::fill(device->model.device->vram.begin(), device->model.device->vram.end(), 0xdb);
        // AsicInit::run on the new incarnation: Mes11::initialize, then the KIQ.
        return device->mes->initialize(device->registers, device->model.io(),
                                       {device.get(), 0x10000, Fixture::doorbell32, Fixture::doorbell64},
                                       device->nbio, rx7900_test::navi31_vram(), {sched_uc, kiq_uc, 2},
                                       {1000, api_timeout}, {&device->reset_confirmed, Fixture::confirm}) ==
                   Mes11Status::ready &&
               device->mes->kiq_hw_init(*device->vram, device->memory) == Mes11Status::scheduler_ready;
    }

    bool destructive_reset(bool lose_vram = true) {
        const bool retired = reset_window();
        const bool restarted = restart(lose_vram);
        return retired && restarted && process_core_rearm(*core);
    }
};

void reap_without_pending_work() {
    Harness h;
    void* a = &h.process_a;
    const uint64_t session = h.open(a);
    const uint64_t gtt = h.buffer(a, session, ProcessMemoryDomain::system, gtt_va);
    h.buffer(a, session, ProcessMemoryDomain::vram, vram_va);
    const uint64_t queue = h.queue(a, session);
    uint32_t sequence = 0;
    expect(h.fill(a, session, queue, gtt, 0x01020304, sequence) == Outcome::success &&
           h.query(a, session, queue, sequence) == Outcome::success, "idle session's work completed");
    const auto* owner = h.session_of(a);
    const auto tables = owner->tables;
    expect(h.references == 1, "open referenced the process once");
    expect(process_core_mark_exit(*h.core, a), "exit notification matches the session");
    expect(owner->pasid != 0 && h.core->queues.report().queues == 1 && h.references == 1,
           "the notification body itself releases nothing");
    expect(h.call(a, Operation::poll, session) == Outcome::access_denied,
           "an exiting session accepts no further work");
    const auto pass = h.reap_pass();
    rx7900::memory::AllocationInfo info{};
    expect(pass.retry_us == 0 && h.session_of(a) == nullptr && h.references == 0,
           "an idle exited session is reaped on the first pass, its process reference dropped");
    expect(h.core->queues.report().processes == 0 && h.core->queues.report().queues == 0 &&
           h.device->gtt->empty() &&
           h.device->vram->describe(tables, info) == rx7900::memory::Status::stale_handle,
           "queues, VM, PASID, page tables and every GTT allocation released");
    expect(h.core->counters.reaped == 1 && h.core->counters.reap_preempted == 0, "reap counted, no preemption");
}

void reap_with_pending_work() {
    Harness h;
    void* a = &h.process_a;
    void* b = &h.process_b;
    const uint64_t session = h.open(a);
    const uint64_t other = h.open(b);
    const uint64_t gtt = h.buffer(a, session, ProcessMemoryDomain::system, gtt_va);
    const uint64_t queue = h.queue(a, session);
    const uint64_t other_queue = h.queue(b, other);
    const uint64_t other_buffer = h.buffer(b, other, ProcessMemoryDomain::system, gtt_va);
    h.hang(a, queue);
    uint32_t sequence = 0;
    expect(h.fill(a, session, queue, gtt, 0x0badf00d, sequence) == Outcome::success &&
           h.query(a, session, queue, sequence) == Outcome::busy, "submitted work stays pending");
    const size_t removes = h.device->frames(3).size();
    expect(process_core_mark_exit(*h.core, a), "process with pending work exits");
    auto pass = h.reap_pass();
    expect(pass.retry_us == process_reap_retry_us && h.session_of(a) != nullptr &&
           h.core->queues.report().queues == 2, "within the idle bound the reaper waits and retries");
    h.device->now += process_reap_idle_timeout_us / 2;
    pass = h.reap_pass();
    expect(pass.retry_us != 0 && h.session_of(a) != nullptr, "still bounded waiting");
    h.device->now += process_reap_idle_timeout_us;
    pass = h.reap_pass();
    expect(pass.retry_us == 0 && h.session_of(a) == nullptr && h.references == 1,
           "past the bound the hung queue is preempted and the session reaped without shutdown");
    expect(h.device->frames(3).size() == removes + 1 && h.core->counters.reap_preempted == 1 &&
           h.core->queues.report().queues == 1 && h.core->queues.report().processes == 1,
           "exactly the exited process's queue was removed through MES REMOVE_QUEUE");
    expect(h.hook_calls == 0 && h.core->counters.escalations == 0, "a completed removal never escalates");
    uint32_t other_sequence = 0;
    expect(h.fill(b, other, other_queue, other_buffer, 0x5a5a5a5a, other_sequence) == Outcome::success &&
           h.query(b, other, other_queue, other_sequence) == Outcome::success,
           "the other process keeps working on its own queue");
}

void removal_failure_escalates() {
    Harness h(200);
    void* a = &h.process_a;
    const uint64_t session = h.open(a);
    const uint64_t gtt = h.buffer(a, session, ProcessMemoryDomain::system, gtt_va);
    const uint64_t queue = h.queue(a, session);
    h.hang(a, queue);
    uint32_t sequence = 0;
    expect(h.fill(a, session, queue, gtt, 0x77, sequence) == Outcome::success, "pending work");
    process_core_mark_exit(*h.core, a);
    h.device->now += process_reap_idle_timeout_us + 1;
    h.device->model.ignore_opcode = 3;
    h.reap_pass();
    auto* owner = h.session_of(a);
    expect(owner != nullptr && owner->quarantined && h.references == 1 &&
           h.core->queues.report().reset_required,
           "REMOVE_QUEUE that MES never completes quarantines the session with every resource");
    expect(h.hook_calls == 1 && h.hook_depth_violations == 0 && h.core->counters.escalations == 1,
           "the hung session requests destructive recovery once, with no process lock held");
    h.reap_pass();
    h.reap_pass();
    expect(h.hook_calls == 1, "escalation is not repeated while the request stands");
    expect(h.destructive_reset(), "the requested destructive recovery retires and rearms the service");
    expect(h.session_of(a) == nullptr && h.references == 0 && h.device->gtt->empty() &&
           h.core->queues.report().processes == 0,
           "the reset proof releases the quarantined exited session inside the reset window");
}

// The exit path unmaps every client GPUVA and waits for the TLB invalidation
// before any backing returns: a failed flush keeps everything and escalates.
void exit_unmap_flush_failure() {
    Harness h;
    void* a = &h.process_a;
    const uint64_t session = h.open(a);
    h.buffer(a, session, ProcessMemoryDomain::system, gtt_va);
    process_core_mark_exit(*h.core, a);
    // IH_VMID_1_LUT reads as all ones: gmc_v11_0_flush_gpu_tlb_pasid cannot run.
    auto& lut = h.device->model.device->reg(h.device->osssys.segment0_dword_base() + 1);
    lut = UINT32_MAX;
    h.reap_pass();
    auto* owner = h.session_of(a);
    expect(owner != nullptr && owner->quarantined && !h.device->gtt->empty() && h.references == 1 &&
           h.hook_calls == 1, "an unacknowledged TLB flush keeps the backing and requests recovery");
    lut = 0;
    expect(h.destructive_reset() && h.session_of(a) == nullptr && h.device->gtt->empty() && h.references == 0,
           "the reset proof then releases it");
}

void reap_racing_reset() {
    Harness h;
    void* a = &h.process_a;
    void* b = &h.process_b;
    const uint64_t session = h.open(a);
    const uint64_t other = h.open(b);
    const uint64_t gtt = h.buffer(a, session, ProcessMemoryDomain::system, gtt_va);
    const uint64_t queue = h.queue(a, session);
    const uint64_t other_buffer = h.buffer(b, other, ProcessMemoryDomain::system, gtt_va);
    const uint64_t other_queue = h.queue(b, other);
    h.hang(a, queue);
    uint32_t sequence = 0;
    h.fill(a, session, queue, gtt, 0x99, sequence);
    process_core_mark_exit(*h.core, a);
    expect(h.reap_pass().retry_us != 0 && h.session_of(a) != nullptr, "the reap is in its bounded wait");
    process_core_begin_reset(*h.core);
    const auto skipped = process_core_reap(*h.core);
    expect(skipped.retry_us == 0 && h.session_of(a) != nullptr && h.core->counters.reaped == 0,
           "a reap that races the reset window touches nothing");
    h.device->reset_confirmed = true;
    expect(process_core_retire_reset(*h.core, Fixture::confirm, &h.device->reset_confirmed),
           "retirement inside the confirmed window");
    h.device->mes->release_after_reset();
    h.core->reset_locked = false;
    expect(h.session_of(a) == nullptr && h.references == 1,
           "the exited session is released by the reset proof, not by a later shutdown");
    // A process that exits after the retirement but before the restart.
    expect(h.session_of(b) != nullptr, "the live session was kept for reconstruction");
    process_core_mark_exit(*h.core, b);
    h.reap_pass();
    expect(h.session_of(b) != nullptr, "while no ASIC runs the reaper defers the exited session");
    expect(h.restart() && process_core_rearm(*h.core) && h.session_of(b) == nullptr && h.references == 0,
           "rearm closes it before reconstructing anything");
    (void)other_buffer;
    (void)other_queue;
}

void reconstruction() {
    Harness h;
    void* a = &h.process_a;
    const uint64_t session = h.open(a);
    const uint64_t gtt = h.buffer(a, session, ProcessMemoryDomain::system, gtt_va);
    const uint64_t vram = h.buffer(a, session, ProcessMemoryDomain::vram, vram_va);
    const uint64_t vram_two = h.buffer(a, session, ProcessMemoryDomain::vram, other_va);
    const uint64_t queue = h.queue(a, session);
    expect(h.write(a, session, gtt, 128, 0x30) == Outcome::success &&
           h.write(a, session, vram, 0, 0x60) == Outcome::success, "client contents written");
    uint32_t done = 0;
    expect(h.fill(a, session, queue, gtt, 0x11223344, done, 4096) == Outcome::success &&
           h.query(a, session, queue, done) == Outcome::success, "pre-reset work completed");
    h.hang(a, queue);
    uint32_t lost = 0;
    expect(h.fill(a, session, queue, gtt, 0x55667788, lost, 8192 - 64) == Outcome::success &&
           h.query(a, session, queue, lost) == Outcome::busy && lost == done + 1,
           "work in flight when the device hangs");
    const auto doorbell = h.doorbell_of(a, queue);
    const size_t adds = h.device->frames(2).size();
    const uint32_t mappings = h.session_of(a) != nullptr ?
        h.core->queues.vm_report(h.session_of(a)->pasid).mappings : 0;
    expect(h.destructive_reset(), "confirmed mode-1 reset, restart and reconstruction");
    auto* owner = h.session_of(a);
    expect(owner != nullptr && owner->handle == session && !owner->poisoned && owner->pasid != 0 &&
           h.references == 1 && h.core->counters.rebuilt == 1,
           "the live session survives with the same opaque handle and process reference");
    if (owner == nullptr) return;
    expect(h.call(a, Operation::query, 0) == Outcome::success && h.core->reply.bytes == 1,
           "version 4 query reports the reset generation (vram_lost_counter)");
    ProcessBufferInfo info{};
    expect(owner->buffers.describe(gtt, info) == rx7900::memory::Status::success && info.mapped &&
           info.va == gtt_va, "the GTT buffer is mapped again at the same GPU virtual address");
    expect(h.core->queues.vm_report(owner->pasid).mappings + 2 == mappings,
           "every surviving mapping is rebuilt; the two lost VRAM buffers are not mapped");
    expect(h.read(a, session, gtt, 128) == Outcome::success && h.payload_is(0x30),
           "GTT contents survive the reset");
    expect(h.read(a, session, vram, 0) == Outcome::buffer_lost && h.core->reply.lost_buffer == vram &&
           h.core->reply.payload_bytes == 0, "the VRAM buffer is reported lost, never with stale data");
    expect(h.read(a, session, vram, 0) == Outcome::invalid_request && h.core->reply.lost_buffer == 0,
           "a lost buffer is answered exactly once, then its handle is dead");
    uint32_t sequence = 0;
    expect(h.copy(a, session, queue, vram_two, gtt, sequence) == Outcome::buffer_lost &&
           h.core->reply.lost_buffer == vram_two && sequence == 0,
           "a lost buffer named as a transfer operand refuses the work and names that buffer");
    expect(h.query(a, session, queue, lost) == Outcome::work_lost &&
           h.query(a, session, queue, lost) == Outcome::work_lost &&
           h.query(a, session, queue, lost, process_abi::graphics_version) == Outcome::quarantined,
           "the in-flight transfer is reported lost (stably), never replayed or claimed complete");
    const auto report = h.core->queues.queue_report(owner->queues[0].id);
    expect(h.device->frames(2).size() == adds + 1 && report.doorbell_index != UINT32_MAX &&
           report.state == UserQueueState::active && report.sync_seq == lost && report.last_seq == lost,
           "the MES queue is re-added with a doorbell and continues the pre-reset fence values");
    (void)doorbell;
    const unsigned faults = h.device->engines.translation_faults;
    expect(h.fill(a, session, queue, gtt, 0x0f0f0f0f, sequence, 0, 64) == Outcome::success &&
           sequence == lost + 1 && h.query(a, session, queue, sequence) == Outcome::success,
           "new work runs on the rebuilt queue with a fence value that cannot alias lost work");
    expect(h.read(a, session, gtt, 0, 4) == Outcome::success && h.core->payload[0] == 0x0f &&
           h.device->engines.translation_faults == faults,
           "the engine walks the rebuilt page tables at the recorded GPUVA");
    expect(h.call(a, Operation::allocate, session, 0, 4096, 0, 0, 1, 3) == Outcome::success &&
           h.core->reply.handle > vram_two, "a new buffer handle never aliases a retired lost one");
    expect(h.core->counters.lost_buffers == 2 && h.core->counters.lost_work == 1 &&
           h.core->counters.reset_generation == 1, "lost resources counted");
}

void version3_lost_answers() {
    Harness h;
    void* a = &h.process_a;
    const uint64_t session = h.open(a);
    const uint64_t vram = h.buffer(a, session, ProcessMemoryDomain::vram, vram_va);
    expect(h.destructive_reset(), "reset and reconstruction");
    expect(h.read(a, session, vram, 0, 64, process_abi::graphics_version) == Outcome::quarantined &&
           h.core->reply.lost_buffer == 0, "a version-3 client receives quarantined for the lost buffer");
    expect(h.read(a, session, vram, 0, 64, process_abi::graphics_version) == Outcome::invalid_request,
           "and the same exactly-once retirement");
}

void second_fault_transient() {
    Harness h;
    void* a = &h.process_a;
    const uint64_t session = h.open(a);
    const uint64_t gtt = h.buffer(a, session, ProcessMemoryDomain::system, gtt_va);
    const uint64_t queue = h.queue(a, session);
    expect(h.reset_window() && h.restart(), "reset retired, ASIC restarted");
    h.dma_failures = 1;
    expect(process_core_rearm(*h.core) && h.core->counters.rebuild_retries == 1 &&
           h.core->counters.rebuilt == 1 && h.session_of(a) != nullptr && !h.session_of(a)->poisoned,
           "a transient fault in the first attempt is undone and the bounded retry succeeds");
    uint32_t sequence = 0;
    expect(h.fill(a, session, queue, gtt, 0x1234, sequence) == Outcome::success &&
           h.query(a, session, queue, sequence) == Outcome::success, "the retried session works");
}

void second_fault_clean_poison() {
    Harness h;
    void* a = &h.process_a;
    const uint64_t session = h.open(a);
    h.buffer(a, session, ProcessMemoryDomain::system, gtt_va);
    const uint64_t queue = h.queue(a, session);
    expect(h.reset_window() && h.restart(), "reset retired, ASIC restarted");
    h.dma_failures = 1000;
    expect(process_core_rearm(*h.core), "the service itself rearms");
    h.dma_failures = 0;
    auto* owner = h.session_of(a);
    expect(owner != nullptr && owner->poisoned && !owner->quarantined && owner->pasid == 0 &&
           h.core->counters.rebuild_retries == process_rebuild_attempts - 1 && h.device->gtt->empty() &&
           h.core->queues.report().processes == 0,
           "bounded attempts exhausted: every resource released, only the tombstone remains");
    expect(h.call(a, Operation::poll, session) == Outcome::session_lost &&
           h.call(a, Operation::queue_destroy, session, queue) == Outcome::session_lost &&
           h.call(a, Operation::poll, session, 0, 0, 0, 0, 0, 0, process_abi::graphics_version) ==
               Outcome::quarantined, "every request of the poisoned session answers a defined error");
    expect(h.hook_calls == 0, "a cleanly retired attempt needs no further recovery");
    expect(h.call(a, Operation::close, session) == Outcome::success && h.session_of(a) == nullptr &&
           h.references == 0, "close retires the tombstone and its process reference");
    expect(h.call(a, Operation::poll, session) == Outcome::access_denied, "the dead handle never revives");
}

void second_fault_uncertain() {
    Harness h(200);
    void* a = &h.process_a;
    const uint64_t session = h.open(a);
    h.buffer(a, session, ProcessMemoryDomain::system, gtt_va);
    h.queue(a, session);
    expect(h.reset_window() && h.restart(), "reset retired, ASIC restarted");
    h.device->model.ignore_opcode = 2;
    expect(process_core_rearm(*h.core), "the service rearms around the faulting session");
    auto* owner = h.session_of(a);
    ProcessRecoveryHook hook{};
    expect(owner != nullptr && owner->poisoned && owner->quarantined && h.hook_calls == 0 &&
           process_core_take_escalation(*h.core, hook) && hook.request != nullptr,
           "ADD_QUEUE lost during reconstruction: uncertain MES ownership retained and escalated");
    expect(h.call(a, Operation::poll, session) == Outcome::session_lost &&
           h.call(a, Operation::close, session) == Outcome::quarantined && h.references == 1,
           "requests answer session_lost; close cannot release what MES may own");
    expect(h.destructive_reset(), "the escalated recovery");
    owner = h.session_of(a);
    expect(owner != nullptr && owner->poisoned && !owner->quarantined && h.device->gtt->empty() &&
           h.core->queues.report().processes == 0,
           "the next reset proof releases the retained resources, never freed by inference");
    expect(h.call(a, Operation::poll, session) == Outcome::session_lost &&
           h.call(a, Operation::close, session) == Outcome::success && h.references == 0,
           "the tombstone stays defined until closed");
}

void poisoned_exit_reaped() {
    Harness h;
    void* a = &h.process_a;
    const uint64_t session = h.open(a);
    h.buffer(a, session, ProcessMemoryDomain::system, gtt_va);
    expect(h.reset_window() && h.restart(), "reset retired, ASIC restarted");
    h.dma_failures = 1000;
    process_core_rearm(*h.core);
    h.dma_failures = 0;
    expect(h.session_of(a) != nullptr && h.session_of(a)->poisoned, "poisoned session");
    if (h.session_of(a) == nullptr) return;
    process_core_mark_exit(*h.core, a);
    h.reap_pass();
    expect(h.session_of(a) == nullptr && h.references == 0, "its process exit retires the tombstone");
}

// A tombstone needs no ASIC: its exit retires it even while no incarnation runs.
void poisoned_exit_while_parked() {
    Harness h;
    void* a = &h.process_a;
    const uint64_t session = h.open(a);
    h.buffer(a, session, ProcessMemoryDomain::system, gtt_va);
    expect(h.reset_window() && h.restart(), "reset retired, ASIC restarted");
    h.dma_failures = 1000;
    process_core_rearm(*h.core);
    h.dma_failures = 0;
    expect(h.reset_window(), "a second reset leaves the service parked");
    process_core_mark_exit(*h.core, a);
    h.reap_pass();
    expect(h.session_of(a) == nullptr && h.references == 0,
           "the poisoned tombstone retires on exit without waiting for a rearm");
}

// An uncertain mapping (unacknowledged TLB flush) is resolved by the reset
// proof: the GTT buffer is remapped and usable again after reconstruction.
void quarantined_mapping_resolved_by_reset() {
    Harness h;
    void* a = &h.process_a;
    const uint64_t session = h.open(a);
    expect(h.call(a, Operation::allocate, session, 0, 8192, 0, 0, 0, 3) == Outcome::success, "GTT buffer");
    const uint64_t gtt = h.core->reply.handle;
    expect(h.write(a, session, gtt, 0, 0x70) == Outcome::success, "contents written");
    expect(h.call(a, Operation::map, session, gtt, 0, 0, gtt_va, 0, 3) == Outcome::success, "mapped");
    auto& lut = h.device->model.device->reg(h.device->osssys.segment0_dword_base() + 1);
    lut = UINT32_MAX;
    expect(h.call(a, Operation::unmap, session, gtt) == Outcome::quarantined &&
           h.read(a, session, gtt, 0) == Outcome::quarantined,
           "an unmap whose TLB invalidation was not acknowledged quarantines the session's buffers");
    lut = 0;
    expect(h.destructive_reset(), "reset and reconstruction");
    ProcessBufferInfo info{};
    auto* owner = h.session_of(a);
    expect(owner != nullptr && !owner->poisoned &&
           owner->buffers.describe(gtt, info) == rx7900::memory::Status::success && info.mapped &&
           info.va == gtt_va && !info.quarantined && h.read(a, session, gtt, 0) == Outcome::success &&
           h.payload_is(0x70), "the reset proof resolves the uncertain unmap; the still-recorded GPUVA is rebuilt");
}

void handle_non_aliasing() {
    Harness h;
    void* a = &h.process_a;
    void* b = &h.process_b;
    const uint64_t session = h.open(a);
    const uint64_t queue = h.queue(a, session);
    const uint64_t first = h.buffer(a, session, ProcessMemoryDomain::vram, vram_va);
    expect(h.destructive_reset(), "reset and reconstruction");
    expect(h.read(a, session, first, 0) == Outcome::buffer_lost, "the lost buffer answered once");
    const uint64_t second = h.buffer(a, session, ProcessMemoryDomain::vram, vram_va);
    expect(second > first && h.read(a, session, first, 0) == Outcome::invalid_request,
           "the slot is reused under a new handle; the old handle stays invalid");
    const uint64_t other = h.open(b);
    expect(other > queue && other > session, "device handles stay monotonic across the reset");
    expect(h.call(b, Operation::poll, session) == Outcome::access_denied &&
           h.call(a, Operation::poll, other) == Outcome::access_denied,
           "a session handle is never usable by another process");
    uint32_t sequence = 0;
    h.core->payload[0] = 0;
    expect(h.fill(b, other, queue, second, 1, sequence) == Outcome::invalid_request,
           "another process cannot name this session's queue or buffer handles");
}

void lock_order() {
    Harness h(200);
    void* a = &h.process_a;
    const uint64_t session = h.open(a);
    const uint64_t gtt = h.buffer(a, session, ProcessMemoryDomain::system, gtt_va);
    const uint64_t queue = h.queue(a, session);
    h.hang(a, queue);
    uint32_t sequence = 0;
    h.fill(a, session, queue, gtt, 1, sequence);
    process_core_mark_exit(*h.core, a);
    h.device->now += process_reap_idle_timeout_us + 1;
    h.device->model.ignore_opcode = 3;
    process_core_reap(*h.core);
    expect(h.hook_calls == 0 && h.core->escalation_pending,
           "core functions only record an escalation; they never call the owner");
    h.lock_acquisitions = 0;
    const auto pass = h.reap_pass();
    expect(h.hook_calls == 1 && h.hook_depth_violations == 0 && h.lock_depth == 0 &&
           h.lock_acquisitions == 1 && !pass.stopped,
           "the reaper pass releases the service lock before the escalation hook");
    const bool stop = true;
    h.lock_acquisitions = 0;
    const auto stopped = process_core_reaper_pass(*h.core, h.lock(), stop);
    expect(stopped.stopped && h.lock_acquisitions == 1 && h.lock_depth == 0 && h.hook_calls == 1,
           "stop is observed under the lock and nothing else runs");
}

void suspend_saved_survives_reset() {
    Harness h;
    void* a = &h.process_a;
    const uint64_t session = h.open(a);
    const uint64_t vram = h.buffer(a, session, ProcessMemoryDomain::vram, vram_va);
    h.queue(a, session);
    expect(h.write(a, session, vram, 0, 0x40) == Outcome::success, "VRAM contents written");
    expect(process_core_suspend(*h.core), "power path proves idle and saves VRAM to its GTT backup");
    expect(h.destructive_reset(), "a reset while parked (failed suspend/resume) retires and rebuilds");
    expect(h.read(a, session, vram, 0) == Outcome::success && h.payload_is(0x40) &&
           h.core->counters.lost_buffers == 0,
           "contents saved while proven idle survive, as Linux eviction preserves them");
}

// The power path (no destructive loss): suspend proves idle, saves VRAM and
// detaches the MES queues; resume restores both. An exit while parked waits.
void suspend_resume_and_parked_exit() {
    Harness h;
    void* a = &h.process_a;
    void* b = &h.process_b;
    const uint64_t session = h.open(a);
    const uint64_t other = h.open(b);
    const uint64_t vram = h.buffer(a, session, ProcessMemoryDomain::vram, vram_va);
    const uint64_t gtt = h.buffer(a, session, ProcessMemoryDomain::system, gtt_va);
    const uint64_t queue = h.queue(a, session);
    h.queue(b, other);
    expect(h.write(a, session, vram, 0, 0x21) == Outcome::success, "VRAM contents written");
    expect(process_core_suspend(*h.core) && h.core->parked && h.device->mes->report().process_queues == 0,
           "suspend detaches every MES queue");
    expect(h.call(a, Operation::poll, session) == Outcome::unavailable, "no request while parked");
    process_core_mark_exit(*h.core, b);
    h.reap_pass();
    expect(h.session_of(b) != nullptr, "an exit while parked waits for resume");
    expect(process_core_resume(*h.core) && !h.core->parked && h.device->mes->report().process_queues == 2,
           "resume re-adds the queues through MES");
    h.reap_pass();
    expect(h.session_of(b) == nullptr && h.references == 1, "the parked exit is reaped after resume");
    uint32_t sequence = 0;
    expect(h.read(a, session, vram, 0) == Outcome::success && h.payload_is(0x21) &&
           h.fill(a, session, queue, gtt, 0x42424242, sequence) == Outcome::success &&
           h.query(a, session, queue, sequence) == Outcome::success,
           "contents and the queue survive the power cycle");
}

bool generate_vertex(void* context, const rx7900::opengl::gfx11_abi::ShaderKey& key,
                     rx7900::sdma::CommandBuffer out, rx7900::opengl::gfx11_abi::ProgramInfo& info) noexcept {
    auto& workspace = *static_cast<rx7900::opengl::shader::GeneratorWorkspace*>(context);
    return rx7900::opengl::shader::generate_vertex_shader(key, out, info, workspace) ==
           rx7900::opengl::shader::Status::success;
}
bool generate_pixel(void* context, const rx7900::opengl::gfx11_abi::ShaderKey& key,
                    rx7900::sdma::CommandBuffer out, rx7900::opengl::gfx11_abi::ProgramInfo& info) noexcept {
    auto& workspace = *static_cast<rx7900::opengl::shader::GeneratorWorkspace*>(context);
    return rx7900::opengl::shader::generate_pixel_shader(key, out, info, workspace) ==
           rx7900::opengl::shader::Status::success;
}
bool run_direct(void*, bool (*body)(void*) noexcept, void* argument) noexcept { return body(argument); }
void no_wait(void*, uint32_t) noexcept {}

// The version-3 graphics context: its gfx queue and private GPUVA areas
// (IB/vertex jobs, shaders, attribute ring) are re-established after a reset.
void graphics_reconstruction() {
    Harness h;
    auto workspace = std::make_unique<rx7900::opengl::shader::GeneratorWorkspace>();
    auto& core = *h.core;
    core.graphics.generator = {workspace.get(), generate_vertex, generate_pixel};
    core.graphics.fpu = {nullptr, run_direct};
    core.graphics.wait_us = no_wait;
    // A small device attribute ring (the fixture arena cannot hold Navi31's).
    rx7900::memory::Handle ring{};
    expect(h.device->vram->allocate(65536, 65536, Access::read_write, true, ring) ==
               rx7900::memory::Status::success &&
           h.device->vram->retain(ring, Access::read_write) == rx7900::memory::Status::success,
           "attribute ring allocated");
    core.attribute_ring = ring;
    core.attribute_ring_live = true;
    core.graphics.attribute_ring = true;
    core.graphics.attribute_ring_mc = h.device->info(*h.device->vram, ring).gpu.virtual_address;
    core.graphics.attribute_ring_per_se_bytes = 65536;
    core.graphics.shader_engines = 1;
    void* a = &h.process_a;
    const uint64_t session = h.open(a);
    expect(h.call(a, Operation::queue_create, session, 0, 0, 0, 0, 0) == Outcome::success, "gfx queue");
    const uint64_t gfx = core.reply.handle;
    expect(h.call(a, Operation::graphics_open, session, gfx) == Outcome::success, "graphics context opened");
    // One typed triangle into owned colour/depth targets, left in flight.
    namespace wire = graphics_abi;
    constexpr uint32_t width = 32;
    constexpr uint32_t height = 32;
    const uint32_t pitch = rx7900::opengl::gfx11::linear_pitch_pixels(width, rx7900::opengl::gfx11::TargetKind::bgra8);
    rx7900::opengl::gfx11::DepthLayout layout{};
    rx7900::opengl::gfx11::depth_layout(width, height, layout);
    const uint64_t color = h.buffer(a, session, ProcessMemoryDomain::system, 0x10000000ULL,
                                    (uint64_t{pitch} * 4 * height + 4095) & ~uint64_t{4095});
    const uint64_t depth = h.buffer(a, session, ProcessMemoryDomain::system, 0x20000000ULL,
                                    (uint64_t{layout.total_bytes} + 4095) & ~uint64_t{4095});
    const uint32_t vertex_bytes = 3 * 80;
    std::vector<uint8_t> draw(wire::submit_header_bytes + wire::state_bytes + wire::command_bytes + vertex_bytes);
    const wire::SubmitHeader header{wire::submit_header_bytes, 1, 1, vertex_bytes, color, depth, width, height, 0, 0};
    wire::GraphicsState state{};
    state.viewport_width = width;
    state.viewport_height = height;
    state.depth_far = 0x3f800000u;
    state.point_size = 0x3f800000u;
    state.line_width = 0x3f800000u;
    state.depth_func = 1;
    state.stencil_func = 7;
    state.stencil_value_mask = 0xff;
    state.stencil_write_mask = 0xff;
    state.blend_src = 1;
    state.logic_op_code = 3;
    state.color_mask = 15;
    const wire::GraphicsCommand command{0, 3, 0, 3, 0, 0};
    expect(wire::encode_header(header, draw.data(), draw.size()) &&
           wire::encode_state(state, draw.data() + wire::submit_header_bytes, wire::state_bytes) &&
           wire::encode_command(command, draw.data() + wire::submit_header_bytes + wire::state_bytes,
                                wire::command_bytes), "typed draw description encoded");
    const uint32_t clip[3][4] = {{0xbf800000u, 0xbf800000u, 0, 0x3f800000u},
                                 {0x3f800000u, 0xbf800000u, 0, 0x3f800000u}, {0, 0x3f800000u, 0, 0x3f800000u}};
    uint8_t* vertices = draw.data() + wire::submit_header_bytes + wire::state_bytes + wire::command_bytes;
    for (uint32_t vertex = 0; vertex < 3; ++vertex) {
        for (uint32_t lane = 0; lane < 4; ++lane) {
            rx7900::sdma::CommandBuffer(vertices + vertex * 80 + lane * 4, 4).write_u32(0, clip[vertex][lane]);
        }
    }
    h.hang(a, gfx);
    core.bulk_input = rx7900::ByteView(draw.data(), draw.size());
    const auto submitted = h.call(a, Operation::graphics_submit, session, gfx, draw.size());
    core.bulk_input = rx7900::ByteView(nullptr, 0);
    const uint32_t in_flight = static_cast<uint32_t>(core.reply.bytes);
    expect(submitted == Outcome::success && in_flight != 0 &&
           h.call(a, Operation::graphics_query, session, gfx, 0, in_flight) == Outcome::busy,
           "the kernel-built draw is in flight when the device hangs");
    expect(h.read(a, session, color, 0, 16) == Outcome::busy, "its target stays leased");
    auto* owner = h.session_of(a);
    if (owner == nullptr) return;
    const uint32_t mappings = core.queues.vm_report(owner->pasid).mappings;
    expect(h.destructive_reset(), "reset and reconstruction");
    owner = h.session_of(a);
    expect(h.call(a, Operation::graphics_query, session, gfx, 0, in_flight) == Outcome::work_lost &&
           h.read(a, session, color, 0, 16) == Outcome::success,
           "the in-flight draw is reported lost and its target lease returned by the reset proof");
    expect(owner != nullptr && !owner->poisoned && owner->graphics.opened() &&
           owner->graphics_queue < process_service_queues &&
           owner->graphics.queue() == owner->queues[owner->graphics_queue].id && owner->graphics.queue() != 0 &&
           core.queues.queue_report(owner->graphics.queue()).type == Mes11QueueType::gfx,
           "the graphics context is bound to the re-added gfx queue");
    if (owner == nullptr) return;
    expect(core.queues.vm_report(owner->pasid).mappings == mappings,
           "every private graphics and queue mapping is re-established in the new VM");
    expect(h.call(a, Operation::graphics_open, session, gfx) == Outcome::busy,
           "the reconstructed context is the open one; no duplicate context");
    expect(process_core_close_all(core) && h.references == 0 && h.device->gtt->empty(),
           "shutdown releases the reconstructed graphics owners and the attribute ring");
}

// The kernel gfx ring of the KernelGraphicsBridge contract (test stand-in for
// Gfx11Queues/FenceManager): leases the IB backing, holds fences until signal.
struct TestKernelRing final {
    rx7900::memory::Manager* gtt = nullptr;
    uint32_t emitted = 0, signalled = 0;
    bool reset_confirmed = false;
    std::vector<std::pair<uint32_t, rx7900::memory::Handle>> leases;
    std::vector<rx7900::hardware::Gfx11VmJob> jobs;

    static rx7900::hardware::Gfx11QueuesStatus submit(void* c, const rx7900::hardware::Gfx11Submission& frame,
                                                      uint32_t& sequence) noexcept {
        auto& self = *static_cast<TestKernelRing*>(c);
        sequence = 0;
        if (frame.vm == nullptr || self.gtt->retain(frame.ib, Access::read) != rx7900::memory::Status::success)
            return rx7900::hardware::Gfx11QueuesStatus::invalid_argument;
        sequence = ++self.emitted;
        self.leases.emplace_back(sequence, frame.ib);
        self.jobs.push_back(*frame.vm);
        return rx7900::hardware::Gfx11QueuesStatus::running;
    }
    static rx7900::hardware::FenceStatus query(void* c, uint32_t sequence) noexcept {
        auto& self = *static_cast<TestKernelRing*>(c);
        return sequence <= self.signalled ? rx7900::hardware::FenceStatus::signalled :
                                            rx7900::hardware::FenceStatus::pending;
    }
    static rx7900::hardware::Gfx11QueuesStatus retire(void* c) noexcept {
        auto& self = *static_cast<TestKernelRing*>(c);
        for (size_t i = 0; i < self.leases.size();) {
            if (self.leases[i].first <= self.signalled) {
                self.gtt->unretain(self.leases[i].second);
                self.leases.erase(self.leases.begin() + static_cast<long>(i));
            } else {
                ++i;
            }
        }
        return rx7900::hardware::Gfx11QueuesStatus::running;
    }
    static bool reset(void* c) noexcept {
        auto& self = *static_cast<TestKernelRing*>(c);
        if (!self.reset_confirmed) return false;
        for (const auto& lease : self.leases) self.gtt->unretain(lease.second);
        self.leases.clear();
        return true;
    }
    rx7900::opengl::GraphicsSubmitter submitter() noexcept { return {this, submit, query, retire, reset}; }
    void signal() noexcept { signalled = emitted; }
};

// Graphics device services for a harness (generator, FPU scope, attribute ring).
void graphics_device(Harness& h, rx7900::opengl::shader::GeneratorWorkspace& workspace) {
    auto& core = *h.core;
    core.graphics.generator = {&workspace, generate_vertex, generate_pixel};
    core.graphics.fpu = {nullptr, run_direct};
    core.graphics.wait_us = no_wait;
    rx7900::memory::Handle ring{};
    expect(h.device->vram->allocate(65536, 65536, Access::read_write, true, ring) ==
               rx7900::memory::Status::success &&
           h.device->vram->retain(ring, Access::read_write) == rx7900::memory::Status::success,
           "attribute ring allocated");
    core.attribute_ring = ring;
    core.attribute_ring_live = true;
    core.graphics.attribute_ring = true;
    core.graphics.attribute_ring_mc = h.device->info(*h.device->vram, ring).gpu.virtual_address;
    core.graphics.attribute_ring_per_se_bytes = 65536;
    core.graphics.shader_engines = 1;
}

// One typed 32x32 triangle into colour/depth targets the session owns.
std::vector<uint8_t> triangle_draw(Harness& h, void* process, uint64_t session, uint64_t& color) {
    namespace wire = graphics_abi;
    constexpr uint32_t width = 32;
    constexpr uint32_t height = 32;
    const uint32_t pitch = rx7900::opengl::gfx11::linear_pitch_pixels(width, rx7900::opengl::gfx11::TargetKind::bgra8);
    rx7900::opengl::gfx11::DepthLayout layout{};
    rx7900::opengl::gfx11::depth_layout(width, height, layout);
    color = h.buffer(process, session, ProcessMemoryDomain::system, 0x10000000ULL,
                     (uint64_t{pitch} * 4 * height + 4095) & ~uint64_t{4095});
    const uint64_t depth = h.buffer(process, session, ProcessMemoryDomain::system, 0x20000000ULL,
                                    (uint64_t{layout.total_bytes} + 4095) & ~uint64_t{4095});
    const uint32_t vertex_bytes = 3 * 80;
    std::vector<uint8_t> draw(wire::submit_header_bytes + wire::state_bytes + wire::command_bytes + vertex_bytes);
    const wire::SubmitHeader header{wire::submit_header_bytes, 1, 1, vertex_bytes, color, depth, width, height, 0, 0};
    wire::GraphicsState state{};
    state.viewport_width = width;
    state.viewport_height = height;
    state.depth_far = 0x3f800000u;
    state.point_size = 0x3f800000u;
    state.line_width = 0x3f800000u;
    state.depth_func = 1;
    state.stencil_func = 7;
    state.stencil_value_mask = 0xff;
    state.stencil_write_mask = 0xff;
    state.blend_src = 1;
    state.logic_op_code = 3;
    state.color_mask = 15;
    const wire::GraphicsCommand command{0, 3, 0, 3, 0, 0};
    wire::encode_header(header, draw.data(), draw.size());
    wire::encode_state(state, draw.data() + wire::submit_header_bytes, wire::state_bytes);
    wire::encode_command(command, draw.data() + wire::submit_header_bytes + wire::state_bytes, wire::command_bytes);
    const uint32_t clip[3][4] = {{0xbf800000u, 0xbf800000u, 0, 0x3f800000u},
                                 {0x3f800000u, 0xbf800000u, 0, 0x3f800000u}, {0, 0x3f800000u, 0, 0x3f800000u}};
    uint8_t* vertices = draw.data() + wire::submit_header_bytes + wire::state_bytes + wire::command_bytes;
    for (uint32_t vertex = 0; vertex < 3; ++vertex) {
        for (uint32_t lane = 0; lane < 4; ++lane) {
            rx7900::sdma::CommandBuffer(vertices + vertex * 80 + lane * 4, 4).write_u32(0, clip[vertex][lane]);
        }
    }
    return draw;
}

Outcome submit_draw(Harness& h, void* process, uint64_t session, uint64_t handle, const std::vector<uint8_t>& draw,
                    uint32_t& sequence) {
    h.core->bulk_input = rx7900::ByteView(draw.data(), draw.size());
    const auto result = h.call(process, Operation::graphics_submit, session, handle, draw.size());
    h.core->bulk_input = rx7900::ByteView(nullptr, 0);
    sequence = static_cast<uint32_t>(h.core->reply.bytes);
    return result;
}

// graphics_open with handle 0: typed draws on the kernel gfx ring, the session
// VM kept until the ring's receipt, lost work after a reset, VRAM quarantine.
void graphics_kernel_ring() {
    Harness none;
    void* a = &none.process_a;
    auto workspace = std::make_unique<rx7900::opengl::shader::GeneratorWorkspace>();
    graphics_device(none, *workspace);
    uint64_t session = none.open(a);
    expect(none.call(a, Operation::graphics_open, session, 0) == Outcome::unavailable,
           "without a kernel-ring submitter graphics_open(handle 0) is unavailable");

    Harness h;
    TestKernelRing ring;
    ring.gtt = h.device->gtt.get();
    h.core->platform.kernel_ring = ring.submitter();
    graphics_device(h, *workspace);
    a = &h.process_a;
    session = h.open(a);
    expect(h.call(a, Operation::graphics_open, session, 0) == Outcome::success && h.session_of(a) != nullptr &&
           h.session_of(a)->graphics_queue == process_graphics_kernel_ring && h.session_of(a)->graphics.kernel_route(),
           "graphics_open(handle 0) binds the context to the kernel gfx ring, no MES queue");
    uint64_t color = 0;
    const auto draw = triangle_draw(h, a, session, color);
    uint32_t sequence = 0;
    auto* owner = h.session_of(a);
    expect(submit_draw(h, a, session, 0, draw, sequence) == Outcome::success && sequence == 1 && owner != nullptr &&
           ring.jobs.size() == 1 && ring.jobs[0].pasid == owner->pasid &&
           ring.jobs[0].pd_address == h.core->queues.vm_report(owner->pasid).pd_address,
           "the typed draw runs as a job in this session's VM on the kernel ring");
    expect(h.call(a, Operation::graphics_query, session, 0, 0, sequence) == Outcome::busy &&
           h.read(a, session, color, 0, 16) == Outcome::busy &&
           h.call(a, Operation::graphics_submit, session, 7, 4) == Outcome::invalid_request,
           "pending: query busy, target leased; a queue handle names no context here");
    expect(h.call(a, Operation::close, session) == Outcome::busy && h.session_of(a) != nullptr,
           "close waits for the kernel ring: the VM outlives the job");
    expect(!process_core_suspend(*h.core) && !h.core->failed && !h.core->parked,
           "a running kernel job refuses the power transition after a bounded drain (retryable, not failed)");

    // Process exit with the job in flight: the reaper waits past the idle
    // deadline (a kernel job cannot be preempted), then reaps after the receipt.
    const uint32_t pasid = owner->pasid;
    process_core_mark_exit(*h.core, a);
    h.device->now += process_reap_idle_timeout_us + 1;
    const auto pass = h.reap_pass();
    expect(pass.retry_us != 0 && h.session_of(a) != nullptr &&
           h.core->queues.vm_report(pasid).pasid == pasid && h.core->counters.reaped == 0,
           "the exited session and its VM are kept while the kernel job is pending");
    ring.signal();
    h.reap_pass();
    expect(h.session_of(a) == nullptr && h.core->counters.reaped == 1 && h.references == 0 && ring.leases.empty(),
           "after the ring's receipt the session is reaped and every lease returned");

    // A destructive reset with kernel-ring work in flight.
    void* b = &h.process_b;
    session = h.open(b);
    expect(h.call(b, Operation::graphics_open, session, 0) == Outcome::success, "second context on the ring");
    const auto second = triangle_draw(h, b, session, color);
    expect(submit_draw(h, b, session, 0, second, sequence) == Outcome::success && sequence == 1,
           "its first draw is in flight");
    ring.reset_confirmed = true;
    expect(h.destructive_reset(), "reset retires and rebuilds the session");
    ring.reset_confirmed = false;
    owner = h.session_of(b);
    expect(owner != nullptr && !owner->poisoned && owner->graphics.opened() && owner->graphics.kernel_route() &&
           h.call(b, Operation::graphics_query, session, 0, 0, 1) == Outcome::work_lost &&
           h.read(b, session, color, 0, 16) == Outcome::success && ring.leases.empty(),
           "the lost kernel job is reported work_lost; its leases returned; the context follows the new VM");
    const size_t jobs = ring.jobs.size();
    expect(submit_draw(h, b, session, 0, second, sequence) == Outcome::success && sequence == 2 &&
           ring.jobs.size() == jobs + 1 && owner != nullptr &&
           ring.jobs.back().pd_address == h.core->queues.vm_report(owner->pasid).pd_address &&
           ring.jobs.back().pasid == owner->pasid,
           "new work continues the session sequence in the rebuilt VM");
    ring.signal();
    expect(h.call(b, Operation::graphics_query, session, 0, 0, 2) == Outcome::success &&
           h.call(b, Operation::graphics_query, session, 0, 0, 1) == Outcome::work_lost,
           "receipts and lost answers stay distinct");

    // An exited session whose kernel job is still in flight at a reset: the
    // reset proof retires the job, the session closes, the service rearms.
    void* c = &h.process_a;
    const uint64_t exiting = h.open(c);
    expect(h.call(c, Operation::graphics_open, exiting, 0) == Outcome::success, "third context");
    uint64_t target = 0;
    const auto third = triangle_draw(h, c, exiting, target);
    expect(submit_draw(h, c, exiting, 0, third, sequence) == Outcome::success, "its draw is in flight");
    process_core_mark_exit(*h.core, c);
    ring.reset_confirmed = true;
    expect(h.destructive_reset() && h.session_of(c) == nullptr && ring.leases.empty(),
           "an exiting session's kernel job is retired by the reset proof; nothing blocks the rearm");
    ring.reset_confirmed = false;

    // RAS page retirement: the session owning the affected VRAM is quarantined.
    const uint64_t vram = h.buffer(b, session, ProcessMemoryDomain::vram, vram_va);
    (void)vram;
    // Find one page of it in the fixture's VRAM arena (MC addresses).
    const auto& slot_owner = *h.session_of(b);
    const uint64_t arena = rx7900_test::fb_base + rx7900_test::arena_offset;
    uint64_t mc = 0;
    for (uint64_t probe = arena; probe < arena + rx7900_test::arena_bytes && mc == 0; probe += 4096) {
        if (slot_owner.buffers.owns_vram(probe, 4096)) mc = probe;
    }
    expect(!h.session_of(a) && !slot_owner.buffers.owns_vram(mc + (1ULL << 40), 4096) &&
           !slot_owner.buffers.owns_vram(UINT64_MAX - 10, 4096), "VRAM ownership is exact and overflow-safe");
    expect(mc != 0 && process_core_quarantine_vram(*h.core, mc, 4096) == 1 &&
           process_core_quarantine_vram(*h.core, mc, 4096) == 0 &&
           h.call(b, Operation::graphics_query, session, 0, 0, 2) == Outcome::access_denied,
           "the VRAM owner is quarantined once and admits no further request");
    uint32_t owners = 7;
    uint32_t nobody = 7;
    expect(process_core_quarantine_vram(*h.core, mc, 4096, &owners) == 0 && owners == 1 &&
           process_core_quarantine_vram(*h.core, mc + (1ULL << 40), 4096, &nobody) == 0 && nobody == 0,
           "owners counts an already quarantined owner (RAS containment); no owner gives 0");
}

// The rearm gate inspects actual retirement; an unproven reset releases nothing.
void unproven_reset_and_rearm_gate() {
    Harness h;
    void* a = &h.process_a;
    const uint64_t session = h.open(a);
    h.buffer(a, session, ProcessMemoryDomain::system, gtt_va);
    h.queue(a, session);
    if (h.session_of(a) == nullptr) return;
    const uint32_t pasid = h.session_of(a)->pasid;
    process_core_begin_reset(*h.core);
    h.device->reset_confirmed = false;
    expect(!process_core_retire_reset(*h.core, Fixture::confirm, &h.device->reset_confirmed) &&
           h.session_of(a) != nullptr && h.session_of(a)->pasid == pasid &&
           h.core->queues.report().processes == 1 &&
           h.core->counters.reset_generation == 0 && !h.core->reset_retired,
           "without the reset proof every queue, VM and session is kept as it was");
    h.core->reset_locked = false;
    expect(!process_core_rearm(*h.core) && h.core->failed,
           "a service whose reset was never retired cannot be rearmed");
    expect(h.reset_window() && h.restart(), "the proven reset retires it");
    h.core->reset_retired = false;
    expect(!process_core_rearm(*h.core) && h.session_of(a) != nullptr && h.session_of(a)->pasid == 0 &&
           h.core->counters.rebuilt == 0,
           "rearm refuses (and rebuilds nothing) unless retirement was complete");
    h.core->reset_retired = true;
    expect(process_core_rearm(*h.core) && h.core->counters.rebuilt == 1, "then the rebuild proceeds");
}

// A queue that lost more work than its list holds still never reports success.
void lost_list_overflow() {
    Harness h;
    void* a = &h.process_a;
    const uint64_t session = h.open(a);
    const uint64_t gtt = h.buffer(a, session, ProcessMemoryDomain::system, gtt_va);
    const uint64_t queue = h.queue(a, session);
    uint32_t lost[process_lost_sequences + 2]{};
    for (auto& sequence : lost) {
        h.hang(a, queue);
        expect(h.fill(a, session, queue, gtt, 0x33, sequence) == Outcome::success, "work in flight");
        expect(h.destructive_reset(), "reset with work in flight");
    }
    bool all_lost = true;
    for (const auto sequence : lost) all_lost = all_lost && h.query(a, session, queue, sequence) == Outcome::work_lost;
    expect(all_lost && h.core->counters.lost_work == process_lost_sequences + 2,
           "every lost transfer, including those past the bounded list, answers work_lost");
    uint32_t sequence = 0;
    expect(h.fill(a, session, queue, gtt, 0x44, sequence) == Outcome::success &&
           sequence == lost[process_lost_sequences + 1] + 1 &&
           h.query(a, session, queue, sequence) == Outcome::success,
           "new work after them completes normally");
}

void restart_admission() {
    Harness h;
    auto& core = *h.core;
    expect(!core.can_restart_after_reset(), "a running service has no reset proof");
    core.failed = true;
    core.parked = true;
    expect(!core.can_restart_after_reset(), "failed parked state alone never establishes retirement");
    core.reset_retired = true;
    expect(core.can_restart_after_reset(), "retired service with no owners admits a restart check");
    auto& session = core.sessions[1];
    session.process = &h.process_a;
    expect(!core.can_restart_after_reset(), "a referenced process that is neither preserved nor a tombstone");
    session.handle = 7;
    session.poisoned = true;
    expect(core.can_restart_after_reset(), "a resource-free poisoned tombstone does not block restart");
    session.quarantined = true;
    expect(!core.can_restart_after_reset(), "a tombstone with retained resources blocks restart");
    session.quarantined = false;
    session.queues[3].handle = 9;
    expect(!core.can_restart_after_reset(), "a tombstone holding a queue capability blocks restart");
    session.queues[3].handle = 0;
    session.poisoned = false;
    session.handle = 0;
    session.process = nullptr;
    expect(core.can_restart_after_reset(), "retiring exact owners restores admission");
}

} // namespace

int main() {
    reap_without_pending_work();
    reap_with_pending_work();
    removal_failure_escalates();
    exit_unmap_flush_failure();
    reap_racing_reset();
    reconstruction();
    version3_lost_answers();
    second_fault_transient();
    second_fault_clean_poison();
    second_fault_uncertain();
    poisoned_exit_reaped();
    poisoned_exit_while_parked();
    quarantined_mapping_resolved_by_reset();
    handle_non_aliasing();
    lock_order();
    suspend_saved_survives_reset();
    graphics_reconstruction();
    graphics_kernel_ring();
    unproven_reset_and_rearm_gate();
    lost_list_overflow();
    suspend_resume_and_parked_exit();
    restart_admission();
    return checks.finish("gpu_process_core_tests");
}

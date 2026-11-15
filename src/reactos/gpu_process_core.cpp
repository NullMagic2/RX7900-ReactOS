/*
Module: rx7900.reactos.gpu_process_core
Purpose: Execute process requests, reap exited sessions and retire/rebuild
sessions across a confirmed destructive reset, without any NT dependency.
Created: 2026-10-24
Architecture: See gpu_process_core.hpp. The request dispatcher, session
open/close, queue creation and power paths moved here unchanged from
gpu_process.cpp (2026-10-22/23); exit reaping, reset retirement, lost-resource
reporting and reconstruction are new. Linux references as in the header.
*/
#include "gpu_process_core.hpp"

namespace rx7900::reactos {
namespace {
using hardware::UserQueueStatus;
using process_abi::Operation;
using process_abi::Outcome;
using Session = GpuProcessCore::Session;
using Queue = GpuProcessCore::Queue;

#if defined(_MSC_VER)
#define RX7900_PROCESS_NOINLINE __declspec(noinline)
#else
#define RX7900_PROCESS_NOINLINE __attribute__((noinline))
#endif

constexpr uint32_t half_range = 0x80000000u;
// Manager allocations are whole pages; every engine's MQD (gfx/compute 2048,
// SDMA 512 bytes) fits one page. A 2048-byte request is refused outright.
constexpr uint64_t queue_mqd_bytes =
    (uint64_t{hardware::mes11_mqd_bytes} + memory::page_bytes - 1u) & ~uint64_t{memory::page_bytes - 1u};
static_assert(hardware::mes11_sdma_mqd_bytes <= queue_mqd_bytes, "SDMA MQD fits the queue MQD page");

uint64_t now(const GpuProcessCore& core) noexcept {
    return core.platform.now_us(core.platform.context);
}

bool dma_bus(const GpuProcessCore& core, memory::Handle handle, uint64_t& bus) noexcept {
    return core.platform.dma_bus(core.platform.context, handle, bus);
}

ProcessMemoryConfig memory_config(GpuProcessCore& core, const Session& session) noexcept {
    return {&core.queues, session.pasid, core.platform.vram, core.platform.gtt,
            core.platform.context, core.platform.dma_bus};
}

ProcessSubmissionConfig submission_config(GpuProcessCore& core, Session& session) noexcept {
    return {&core.queues, &session.buffers, core.platform.gtt, session.pasid,
            core.platform.context, core.platform.dma_bus};
}

ProcessGraphicsConfig graphics_config(GpuProcessCore& core, Session& session) noexcept {
    ProcessGraphicsConfig config{};
    config.queues = &core.queues;
    config.buffers = &session.buffers;
    config.gtt = core.platform.gtt;
    config.pasid = session.pasid;
    config.context = core.platform.context;
    config.dma_bus = core.platform.dma_bus;
    config.device = &core.graphics;
    config.workspace = &core.graphics_workspace;
    config.kernel_ring = core.platform.kernel_ring;
    return config;
}

// Version-4 requests receive the recovery outcomes; older versions receive
// quarantined in their place (gpu_process_abi.hpp recovery_version).
bool recovery_abi(const GpuProcessCore& core) noexcept {
    return core.request.abi_version == process_abi::recovery_version;
}

Outcome versioned(const GpuProcessCore& core, Outcome recovery) noexcept {
    return recovery_abi(core) ? recovery : Outcome::quarantined;
}

// One attribute ring per device (Mesa ac_gpu_info.c, chips with L3: 1400 KiB
// per SE aligned to 64 KiB; Navi31 has six SEs), VRAM, kept for the service life.
bool allocate_attribute_ring(GpuProcessCore& core) noexcept {
    if (core.attribute_ring_live) return true;
    auto& vram = *core.platform.vram;
    const uint32_t per_se = opengl::gfx11::attribute_ring_per_se_bytes_l3;
    const uint32_t engines = opengl::gfx11::navi31_shader_engines;
    memory::Handle ring{};
    if (vram.allocate(static_cast<uint64_t>(per_se) * engines, 65536, memory::Access::read_write,
                      true, ring) != memory::Status::success) return false;
    memory::AllocationInfo info{};
    if (vram.retain(ring, memory::Access::read_write) != memory::Status::success) {
        vram.release(ring);
        return false;
    }
    if (vram.describe(ring, info) != memory::Status::success ||
        (info.gpu.virtual_address & 0xffffu) != 0) {
        vram.unretain(ring);
        vram.release(ring);
        return false;
    }
    core.attribute_ring = ring;
    core.attribute_ring_live = true;
    core.graphics.attribute_ring = true;
    core.graphics.attribute_ring_mc = info.gpu.virtual_address;
    core.graphics.attribute_ring_per_se_bytes = per_se;
    core.graphics.shader_engines = engines;
    return true;
}

bool release_attribute_ring(GpuProcessCore& core) noexcept {
    if (!core.attribute_ring_live) return true;
    auto& vram = *core.platform.vram;
    if (vram.unretain(core.attribute_ring) != memory::Status::success ||
        vram.release(core.attribute_ring) != memory::Status::success) return false;
    core.attribute_ring = {};
    core.attribute_ring_live = false;
    core.graphics.attribute_ring = false;
    return true;
}

Outcome outcome(memory::Status status) noexcept {
    switch (status) {
    case memory::Status::success: return Outcome::success;
    case memory::Status::access_denied: return Outcome::access_denied;
    case memory::Status::busy:
    case memory::Status::deferred: return Outcome::busy;
    case memory::Status::out_of_memory:
    case memory::Status::exhausted: return Outcome::exhausted;
    case memory::Status::quarantined:
    case memory::Status::backend_failure: return Outcome::quarantined;
    case memory::Status::unavailable: return Outcome::unavailable;
    default: return Outcome::invalid_request;
    }
}

bool allocation(memory::Manager& manager, uint64_t bytes, memory::Handle& handle) noexcept {
    return manager.allocate(bytes, memory::page_bytes, memory::Access::read_write,
                            true, handle) == memory::Status::success;
}

bool drop(memory::Manager& manager, memory::Handle& handle) noexcept {
    if (handle.slot == UINT32_MAX) {
        return true;
    }
    if (manager.release(handle) != memory::Status::success) {
        return false;
    }
    handle = {};
    return true;
}

bool release_owned(GpuProcessCore& core, Session& session) noexcept {
    auto& gtt = *core.platform.gtt;
    auto& vram = *core.platform.vram;
    bool released = true;
    for (auto& queue : session.queues) {
        // This helper is called only after the entire VM/queues were retired.
        if (!drop(gtt, queue.ring) || !drop(gtt, queue.mqd)) {
            released = false;
        } else {
            queue = {};
        }
    }
    if (!drop(gtt, session.gang_context)) released = false;
    if (!drop(gtt, session.context)) released = false;
    if (!drop(gtt, session.table_backup)) released = false;
    if (!drop(vram, session.tables)) released = false;
    return released;
}

// Every GPU-side owner of a session whose VM is gone (destroyed or retired by
// a confirmed reset). Graphics leases on session buffers return first.
bool release_resources(GpuProcessCore& core, Session& session) noexcept {
    return session.submissions.close_after_idle() == memory::Status::success &&
           session.graphics.close_after_idle() == memory::Status::success &&
           session.buffers.close_after_idle() == memory::Status::success &&
           release_owned(core, session);
}

// The session's identity ends: the process reference is dropped exactly once.
// ProcessMemory has no OS destructor, but its monotonic handles must not
// recycle within a reused session. initialize() retains its counter.
void retire_identity(GpuProcessCore& core, Session& session) noexcept {
    if (session.process != nullptr && core.platform.dereference != nullptr) {
        core.platform.dereference(core.platform.context, session.process);
    }
    session.process = nullptr;
    session.handle = 0;
    session.gang = 0;
    session.graphics_queue = UINT32_MAX;
    session.kernel_graphics = {};
    session.quarantined = false;
    session.exiting = false;
    session.reset_preserved = false;
    session.poisoned = false;
    session.escalated = false;
    session.exit_deadline_us = 0;
}

// Out of line: keeps every caller frame within the kernel stack budget.
RX7900_PROCESS_NOINLINE bool close_session(GpuProcessCore& core, Session& session) noexcept {
    // A kernel gfx-ring job reads this VM until its fence: the VM outlives it.
    if (session.graphics.kernel_pending()) {
        session.graphics.poll();
        if (session.graphics.kernel_pending()) {
            if (session.graphics.kernel_quarantined()) session.quarantined = true;
            return false;
        }
    }
    if (session.pasid != 0) {
        const auto status = core.queues.destroy_process(session.pasid);
        if (status != UserQueueStatus::destroyed) {
            // A live fence is not an uncertain ownership failure. The reaper
            // retries after the queue's actual completion receipt arrives.
            if (status != UserQueueStatus::busy) session.quarantined = true;
            return false;
        }
        session.pasid = 0;
    }
    if (!release_resources(core, session)) {
        session.quarantined = true;
        return false;
    }
    retire_identity(core, session);
    return true;
}

// Ask the device owner for destructive recovery, once per uncertain session.
void escalate(GpuProcessCore& core, Session& session) noexcept {
    if (session.escalated) return;
    session.escalated = true;
    core.escalation_pending = true;
    ++core.counters.escalations;
}

// Reconstruction failed with a provably retired VM: keep only the tombstone
// (process reference and handle) that answers session_lost until close/exit.
void poison_session(GpuProcessCore& core, Session& session) noexcept {
    if (!session.poisoned) ++core.counters.poisoned;
    session.poisoned = true;
    session.reset_preserved = false;
    if (!release_resources(core, session)) {
        session.quarantined = true;
        escalate(core, session);
    }
}

// Out of line: keeps every caller frame within the kernel stack budget.
RX7900_PROCESS_NOINLINE bool initialize_queues(GpuProcessCore& core) noexcept {
    hardware::UserQueueConfig config{};
    if (core.platform.queue_config == nullptr ||
        !core.platform.queue_config(core.platform.context, core.pasids, config)) return false;
    config.clock = {core.platform.context, core.platform.now_us};
    return core.queues.initialize(config) == UserQueueStatus::ready;
}

RX7900_PROCESS_NOINLINE UserQueueStatus attach_queue(GpuProcessCore& core, Session& session, Queue& queue,
                             uint32_t index) noexcept {
    constexpr uint32_t ring_bytes = hardware::user_queue_min_ring_bytes;
    constexpr uint32_t buffer_bytes = ring_bytes + hardware::user_queue_control_bytes;
    uint64_t bus = 0;
    if (!dma_bus(core, queue.ring, bus) || bus > UINT64_MAX - buffer_bytes)
        return UserQueueStatus::invalid_memory;
    for (uint32_t page = 0; page < buffer_bytes / memory::page_bytes; ++page) {
        const uint64_t address = bus + page * memory::page_bytes;
        for (uint32_t byte = 0; byte < 8; ++byte)
            core.queue_pages[page * 8 + byte] = static_cast<uint8_t>(address >> (byte * 8));
    }
    const uint64_t queue_va = (1ULL << 44) + index * (2ULL << 20);
    const hardware::UserQueueProperties properties{queue.type, ring_bytes, queue_va,
        queue.ring, ByteView(core.queue_pages, sizeof(core.queue_pages)), queue.mqd, 0};
    return core.queues.add_queue(session.gang, properties, queue.id);
}

Session* authorize(GpuProcessCore& core, void* process, uint64_t handle) noexcept {
    for (auto& session : core.sessions) {
        if (session.handle == handle && handle != 0 && session.process == process &&
            process != nullptr && !session.exiting &&
            (!session.quarantined || session.poisoned)) return &session;
    }
    return nullptr;
}

uint64_t next_handle(GpuProcessCore& core) noexcept {
    // Refuse exhaustion rather than recycling a capability in this device.
    if (core.next_handle == UINT64_MAX) return 0;
    return core.next_handle++;
}

RX7900_PROCESS_NOINLINE Outcome open_session(GpuProcessCore& core, void* process) noexcept {
    if (process == nullptr) return Outcome::access_denied;
    for (const auto& session : core.sessions) {
        if (session.process == process) return Outcome::busy;
    }
    Session* available = nullptr;
    for (auto& session : core.sessions) {
        if (session.process == nullptr && !session.quarantined) { available = &session; break; }
    }
    if (available == nullptr) return Outcome::exhausted;
    auto& session = *available;
    session.handle = next_handle(core);
    if (session.handle == 0) return Outcome::exhausted;
    session.process = process;
    if (core.platform.reference != nullptr) core.platform.reference(core.platform.context, process);
    auto& gtt = *core.platform.gtt;
    auto& vram = *core.platform.vram;
    constexpr uint64_t table_bytes = process_service_tables * hardware::gpuvm_table_bytes;
    if (!allocation(vram, table_bytes, session.tables) ||
        !allocation(gtt, table_bytes, session.table_backup) ||
        !allocation(gtt, hardware::mes11_process_context_bytes, session.context) ||
        !allocation(gtt, hardware::mes11_gang_context_bytes, session.gang_context)) {
        close_session(core, session);
        return Outcome::exhausted;
    }
    const hardware::UserProcessMemory memory{session.tables, process_service_tables,
                                             session.context, session.table_backup};
    const auto created = core.queues.create_process(memory, session.pasid);
    if (created != UserQueueStatus::created) {
        close_session(core, session);
        return created == UserQueueStatus::quarantined ? Outcome::quarantined : Outcome::unavailable;
    }
    const hardware::UserGangProperties gang{hardware::mes11_priority_normal, 0, 0,
                                            session.gang_context};
    if (core.queues.add_gang(session.pasid, gang, session.gang) != UserQueueStatus::added ||
        session.buffers.initialize(memory_config(core, session)) != memory::Status::success ||
        session.submissions.initialize(submission_config(core, session)) != memory::Status::success) {
        close_session(core, session);
        return Outcome::unavailable;
    }
    core.reply.session = session.handle;
    return Outcome::success;
}

RX7900_PROCESS_NOINLINE Outcome create_queue(GpuProcessCore& core, Session& session, uint32_t engine) noexcept {
    Queue* available = nullptr;
    uint32_t index = 0;
    for (; index < process_service_queues; ++index) {
        auto& slot = session.queues[index];
        if (slot.handle == 0 && !slot.quarantined) { available = &slot; break; }
    }
    if (available == nullptr) return Outcome::exhausted;
    auto& queue = *available;
    queue.handle = next_handle(core);
    if (queue.handle == 0) return Outcome::exhausted;
    queue.type = static_cast<hardware::Mes11QueueType>(engine);
    auto& gtt = *core.platform.gtt;
    constexpr uint32_t ring_bytes = hardware::user_queue_min_ring_bytes;
    constexpr uint32_t buffer_bytes = ring_bytes + hardware::user_queue_control_bytes;
    if (!allocation(gtt, buffer_bytes, queue.ring) ||
        !allocation(gtt, queue_mqd_bytes, queue.mqd)) {
        const bool ring_released = drop(gtt, queue.ring);
        const bool mqd_released = drop(gtt, queue.mqd);
        queue.quarantined = !ring_released || !mqd_released;
        queue.handle = 0;
        return Outcome::exhausted;
    }
    const auto status = attach_queue(core, session, queue, index);
    if (status != UserQueueStatus::added) {
        if (queue.id != 0 || status == UserQueueStatus::quarantined) {
            queue.quarantined = true;
            return Outcome::quarantined;
        }
        const bool ring_released = drop(gtt, queue.ring);
        const bool mqd_released = drop(gtt, queue.mqd);
        queue.quarantined = !ring_released || !mqd_released;
        queue.handle = 0;
        return Outcome::unavailable;
    }
    core.reply.handle = queue.handle;
    return Outcome::success;
}

// Wrap-safe: value is at or below floor in the monotonic 32-bit fence space.
bool at_or_below(uint32_t value, uint32_t floor) noexcept {
    return floor - value < half_range;
}

bool work_lost(const Queue& queue, uint32_t sequence) noexcept {
    if (sequence == 0) return false;
    for (const uint32_t lost : queue.lost) {
        if (lost == sequence) return true;
    }
    return queue.lost_floor != 0 && at_or_below(sequence, queue.lost_floor);
}

// The oldest remembered value moves into lost_floor when the list is full:
// anything at or below it is reported lost or uncertain, never success.
void record_lost_in(GpuProcessCore& core, Queue& queue, uint32_t sequence) noexcept {
    uint32_t slot = 0;
    while (slot < process_lost_sequences && queue.lost[slot] != 0) ++slot;
    if (slot == process_lost_sequences) {
        if (queue.lost_floor == 0 || !at_or_below(queue.lost[0], queue.lost_floor)) {
            queue.lost_floor = queue.lost[0];
        }
        for (uint32_t index = 1; index < process_lost_sequences; ++index) {
            queue.lost[index - 1] = queue.lost[index];
        }
        slot = process_lost_sequences - 1;
    }
    queue.lost[slot] = sequence;
    ++core.counters.lost_work;
}

void record_lost(GpuProcessCore& core, Session& session, uint32_t queue_id, uint32_t sequence) noexcept {
    if (queue_id == 0 || sequence == 0) return;
    for (auto& queue : session.queues) {
        if (queue.id != queue_id) continue;
        record_lost_in(core, queue, sequence);
        return;
    }
}

// Version 3 graphics on the kernel gfx ring (Request.handle 0): Linux's
// amdgpu_cs path, for a session whose MES gfx queue is unavailable.
RX7900_PROCESS_NOINLINE Outcome execute_kernel_graphics(GpuProcessCore& core, Session& session,
                                                        Operation op) noexcept {
    const auto& r = core.request;
    if (op == Operation::graphics_open) {
        if (session.graphics.opened()) return Outcome::busy;
        if (core.platform.kernel_ring.submit == nullptr) return Outcome::unavailable;
        if (!allocate_attribute_ring(core)) return Outcome::exhausted;
        const auto status = session.graphics.open(graphics_config(core, session), 0);
        if (status == memory::Status::success) session.graphics_queue = process_graphics_kernel_ring;
        return outcome(status);
    }
    if (session.graphics_queue != process_graphics_kernel_ring) return Outcome::invalid_request;
    if (op == Operation::graphics_query) {
        core.reply.bytes = r.offset;
        if (work_lost(session.kernel_graphics, static_cast<uint32_t>(r.offset)))
            return versioned(core, Outcome::work_lost);
        return outcome(session.graphics.query(0, static_cast<uint32_t>(r.offset)));
    }
    uint32_t sequence = 0;
    const auto status = session.graphics.submit(0, core.bulk_input, sequence);
    core.reply.bytes = sequence;
    return outcome(status);
}

// Version 3 graphics: Request.handle names a gfx queue of this session, or is
// 0 for the kernel gfx-ring route.
RX7900_PROCESS_NOINLINE Outcome execute_graphics(GpuProcessCore& core, Session& session, Operation op) noexcept {
    const auto& r = core.request;
    if (r.handle == 0) return execute_kernel_graphics(core, session, op);
    uint32_t index = 0;
    for (; index < process_service_queues; ++index) {
        const auto& queue = session.queues[index];
        if (queue.handle == r.handle && !queue.quarantined &&
            queue.type == hardware::Mes11QueueType::gfx) break;
    }
    if (index == process_service_queues) return Outcome::invalid_request;
    const uint32_t queue_id = session.queues[index].id;
    if (op == Operation::graphics_open) {
        if (session.graphics.opened()) return Outcome::busy;
        if (!allocate_attribute_ring(core)) return Outcome::exhausted;
        const auto status = session.graphics.open(graphics_config(core, session), queue_id);
        if (status == memory::Status::success) session.graphics_queue = index;
        return outcome(status);
    }
    if (session.graphics_queue != index) return Outcome::invalid_request;
    if (op == Operation::graphics_query) {
        core.reply.bytes = r.offset;
        if (work_lost(session.queues[index], static_cast<uint32_t>(r.offset)))
            return versioned(core, Outcome::work_lost);
        core.queues.poll();
        return outcome(session.graphics.query(queue_id, static_cast<uint32_t>(r.offset)));
    }
    uint32_t sequence = 0;
    const auto status = session.graphics.submit(queue_id, core.bulk_input, sequence);
    core.reply.bytes = sequence;
    return outcome(status);
}

// Version 3 bulk transfers through ProcessMemory's bounded 4 KiB CPU copies.
RX7900_PROCESS_NOINLINE Outcome execute_bulk(GpuProcessCore& core, Session& session, Operation op) noexcept {
    const auto& r = core.request;
    const uint32_t total = static_cast<uint32_t>(r.bytes);
    for (uint32_t done = 0; done < total;) {
        const uint32_t chunk = total - done < process_abi::copy_bytes ? total - done : process_abi::copy_bytes;
        if (r.offset > UINT64_MAX - done) return Outcome::invalid_request;
        memory::Status status = memory::Status::invalid_argument;
        if (op == Operation::buffer_upload) {
            ByteView part(nullptr, 0);
            if (!core.bulk_input.subview(done, chunk, part)) return Outcome::invalid_request;
            status = session.buffers.write(r.handle, r.offset + done, part);
        } else {
            if (core.bulk_output == nullptr || core.bulk_output_bytes < total) return Outcome::invalid_request;
            status = session.buffers.read(r.handle, r.offset + done, core.bulk_output + done, chunk);
        }
        if (status != memory::Status::success) return outcome(status);
        done += chunk;
    }
    if (op == Operation::buffer_readback) core.reply.payload_bytes = total;
    return Outcome::success;
}

Outcome destroy_queue(GpuProcessCore& core, Session& session, uint64_t handle) noexcept {
    for (auto& queue : session.queues) {
        if (queue.handle != handle || handle == 0) continue;
        if (core.queues.remove_queue(queue.id) != UserQueueStatus::removed) {
            queue.quarantined = true; return Outcome::quarantined;
        }
        if (session.submissions.retire_removed_queue(queue.id) != memory::Status::success ||
            session.graphics.retire_removed_queue(queue.id) != memory::Status::success) {
            queue.quarantined = true; return Outcome::quarantined;
        }
        if (session.graphics_queue < process_service_queues &&
            &session.queues[session.graphics_queue] == &queue) session.graphics_queue = UINT32_MAX;
        auto& gtt = *core.platform.gtt;
        if (!drop(gtt, queue.ring) || !drop(gtt, queue.mqd)) {
            queue.quarantined = true; return Outcome::quarantined;
        }
        queue = {};
        return Outcome::success;
    }
    return Outcome::invalid_request;
}

RX7900_PROCESS_NOINLINE Outcome transfer(GpuProcessCore& core, Session& session, Operation op) noexcept {
    const auto& r = core.request;
    for (const auto& queue : session.queues) {
        if (queue.handle != r.handle || queue.quarantined || r.handle == 0) continue;
        if (op == Operation::submission_query) {
            core.reply.bytes = r.offset;
            if (work_lost(queue, static_cast<uint32_t>(r.offset)))
                return versioned(core, Outcome::work_lost);
            return outcome(session.submissions.query(queue.id, static_cast<uint32_t>(r.offset)));
        }
        process_abi::Transfer command{};
        const bool fill = op == Operation::fill;
        if (!process_abi::decode_transfer(ByteView(core.payload, sizeof(command)), command, fill))
            return Outcome::invalid_request;
        const ProcessTransfer work{command.source, command.destination, command.source_offset,
            command.destination_offset, command.bytes, command.value};
        uint32_t sequence = 0;
        const auto status = fill ? session.submissions.fill(queue.id, work, sequence) :
            session.submissions.copy(queue.id, work, sequence);
        core.reply.bytes = sequence;
        return outcome(status);
    }
    return Outcome::invalid_request;
}

Outcome dispatch(GpuProcessCore& core, Session& session, Operation op) noexcept {
    const auto& r = core.request;
    switch (op) {
    case Operation::close:
        return close_session(core, session) ? Outcome::success :
            session.quarantined ? Outcome::quarantined : Outcome::busy;
    case Operation::allocate:
        return outcome(session.buffers.allocate(r.bytes, static_cast<ProcessMemoryDomain>(r.domain),
            static_cast<memory::Access>(r.access), core.reply.handle));
    case Operation::release: return outcome(session.buffers.release(r.handle));
    case Operation::map: return outcome(session.buffers.map(r.handle, r.address,
                                                            static_cast<memory::Access>(r.access)));
    case Operation::unmap: return outcome(session.buffers.unmap(r.handle));
    case Operation::read: {
        const auto status = session.buffers.read(r.handle, r.offset, core.payload,
                                                  static_cast<uint32_t>(r.bytes));
        if (status == memory::Status::success) core.reply.payload_bytes = static_cast<uint32_t>(r.bytes);
        return outcome(status);
    }
    case Operation::write:
        return outcome(session.buffers.write(r.handle, r.offset,
                                              ByteView(core.payload, static_cast<size_t>(r.bytes))));
    case Operation::queue_create: return create_queue(core, session, r.domain);
    case Operation::queue_destroy: return destroy_queue(core, session, r.handle);
    case Operation::poll:
        core.queues.poll();
        return core.queues.report().reset_required ? Outcome::quarantined :
            outcome(session.submissions.poll());
    case Operation::copy:
    case Operation::fill:
    case Operation::submission_query:
        return transfer(core, session, op);
    case Operation::graphics_open:
    case Operation::graphics_submit:
    case Operation::graphics_query:
        return execute_graphics(core, session, op);
    case Operation::buffer_upload:
    case Operation::buffer_readback:
        return execute_bulk(core, session, op);
    default: return Outcome::invalid_request;
    }
}

// ------------------------------------------------------------------ reaping

bool outstanding(const GpuProcessCore& core, const Session& session) noexcept {
    for (const auto& queue : session.queues) {
        if (queue.id == 0) continue;
        const auto report = core.queues.queue_report(queue.id);
        if (report.queue_id == queue.id && report.state == hardware::UserQueueState::active &&
            report.sync_seq != report.last_seq) return true;
    }
    return false;
}

// Work leases of removed queues return (REMOVE_QUEUE completion is the proof).
bool retire_removed_queues(Session& session) noexcept {
    for (auto& queue : session.queues) {
        if (queue.id == 0) continue;
        if (session.submissions.retire_removed_queue(queue.id) != memory::Status::success ||
            session.graphics.retire_removed_queue(queue.id) != memory::Status::success) return false;
        queue.id = 0;
    }
    session.graphics_queue = UINT32_MAX;
    return true;
}

void quarantine_exit(GpuProcessCore& core, Session& session) noexcept {
    session.quarantined = true;
    ++core.counters.reap_quarantined;
    escalate(core, session);
}

// Linux amdgpu_userq_mgr_fini/amdgpu_userq_cleanup order for one exited
// session: wait (bounded) for the last fences, unmap the queues through MES
// (hung ones preempted), unmap the client GPUVAs with a TLB flush, finish the
// VM and free the PASID, then release every buffer. Unlike v6.12
// amdgpu_mes_remove_hw_queue, which frees the queue even when REMOVE_QUEUE
// failed, a failed removal keeps every lease and asks for the destructive
// recovery Linux amdgpu_userq_unmap_helper schedules. true: retry later.
RX7900_PROCESS_NOINLINE bool reap_session(GpuProcessCore& core, Session& session, uint64_t time) noexcept {
    if (session.quarantined) {
        escalate(core, session);
        return false;
    }
    // A poisoned tombstone has no PASID and no resources: close_session below
    // only drops its process reference.
    if (session.pasid != 0) {
        session.submissions.poll();
        session.graphics.poll();
        // Kernel gfx-ring jobs cannot be preempted: the reaper waits for their
        // receipt, or for the ring's job timeout, which quarantines them.
        if (session.graphics.kernel_quarantined()) {
            quarantine_exit(core, session);
            return false;
        }
        if (session.graphics.kernel_pending()) return true;
        const bool busy = outstanding(core, session);
        if (busy && time < session.exit_deadline_us) return true;
        if (busy) ++core.counters.reap_preempted;
        const auto evicted = core.queues.evict_process_queues(session.pasid);
        if (evicted == UserQueueStatus::suspended || evicted == UserQueueStatus::unavailable) return false;
        if (evicted != UserQueueStatus::removed && evicted != UserQueueStatus::not_found) {
            quarantine_exit(core, session);
            return false;
        }
        if (!retire_removed_queues(session) ||
            session.buffers.unmap_all() != memory::Status::success) {
            quarantine_exit(core, session);
            return false;
        }
        const auto destroyed = core.queues.destroy_process(session.pasid);
        if (destroyed != UserQueueStatus::destroyed) {
            quarantine_exit(core, session);
            return false;
        }
        session.pasid = 0;
    }
    if (!close_session(core, session)) return false;
    ++core.counters.reaped;
    return false;
}

// ------------------------------------------------------- reset and rebuild

RX7900_PROCESS_NOINLINE bool retire_preserved(GpuProcessCore& core, Session& session) noexcept {
    uint32_t queue = 0;
    uint32_t sequence = 0;
    if (session.submissions.retire_after_reset(queue, sequence) != memory::Status::success) return false;
    record_lost(core, session, queue, sequence);
    uint32_t sequences[graphics_jobs]{};
    const bool kernel = session.graphics.kernel_route();
    if (session.graphics.retire_after_reset(queue, sequences) != memory::Status::success) return false;
    for (const uint32_t lost : sequences) {
        if (kernel && lost != 0) {
            record_lost_in(core, session.kernel_graphics, lost);
        } else {
            record_lost(core, session, queue, lost);
        }
    }
    auto& gtt = *core.platform.gtt;
    for (auto& entry : session.queues) {
        entry.id = 0;
        // A queue whose MES add/remove was uncertain is resolved by the proof;
        // its client never received a usable handle state, so it retires here.
        if (entry.quarantined) {
            if (!drop(gtt, entry.ring) || !drop(gtt, entry.mqd)) return false;
            if (session.graphics_queue < process_service_queues &&
                &session.queues[session.graphics_queue] == &entry) session.graphics_queue = UINT32_MAX;
            entry = {};
        }
    }
    return session.buffers.retire_after_reset() == memory::Status::success;
}

// The graphics context follows its re-added gfx queue (0 when none is bound).
RX7900_PROCESS_NOINLINE bool rebind_graphics(GpuProcessCore& core, Session& session) noexcept {
    const uint32_t graphics_queue = session.graphics_queue < process_service_queues ?
        session.queues[session.graphics_queue].id : 0;
    return session.graphics.rebind_after_reset(graphics_config(core, session), graphics_queue) ==
        memory::Status::success;
}

RX7900_PROCESS_NOINLINE bool rebuild_session(GpuProcessCore& core, Session& session) noexcept {
    const hardware::UserProcessMemory memory{session.tables, process_service_tables,
                                             session.context, session.table_backup};
    // amdgpu_vm_init over the retained page-table allocation: its VRAM
    // contents were lost, the root is cleared and every PTE rebuilt below.
    if (core.queues.create_process(memory, session.pasid) != UserQueueStatus::created) {
        session.pasid = 0;
        return false;
    }
    const hardware::UserGangProperties gang{hardware::mes11_priority_normal, 0, 0,
                                            session.gang_context};
    if (core.queues.add_gang(session.pasid, gang, session.gang) != UserQueueStatus::added ||
        session.buffers.rebind_after_reset(memory_config(core, session)) != memory::Status::success ||
        session.submissions.rebind_after_reset(submission_config(core, session)) != memory::Status::success)
        return false;
    for (uint32_t index = 0; index < process_service_queues; ++index) {
        auto& queue = session.queues[index];
        if (queue.handle == 0) continue;
        if (attach_queue(core, session, queue, index) != UserQueueStatus::added ||
            core.queues.continue_fences(queue.id, queue.fence_floor) != UserQueueStatus::ready)
            return false;
    }
    return rebind_graphics(core, session);
}

// After a failed attempt: retire that attempt's VM through MES. false when its
// teardown is itself uncertain (a queue MES may still own).
RX7900_PROCESS_NOINLINE bool undo_rebuild(GpuProcessCore& core, Session& session) noexcept {
    if (session.pasid != 0) {
        const auto status = core.queues.destroy_process(session.pasid);
        if (status != UserQueueStatus::destroyed && status != UserQueueStatus::not_found) return false;
    }
    session.pasid = 0;
    session.gang = 0;
    for (auto& queue : session.queues) queue.id = 0;
    session.buffers.abandon_rebind();
    session.submissions.abandon_rebind();
    session.graphics.abandon_rebind();
    return true;
}

void rebuild_or_poison(GpuProcessCore& core, Session& session) noexcept {
    for (uint32_t attempt = 0; attempt < process_rebuild_attempts; ++attempt) {
        if (attempt != 0) ++core.counters.rebuild_retries;
        if (rebuild_session(core, session)) {
            session.reset_preserved = false;
            ++core.counters.rebuilt;
            return;
        }
        if (!undo_rebuild(core, session)) {
            // A second fault left MES ownership uncertain: keep everything,
            // answer session_lost and ask for another destructive recovery.
            session.reset_preserved = false;
            session.poisoned = true;
            session.quarantined = true;
            ++core.counters.poisoned;
            escalate(core, session);
            return;
        }
    }
    poison_session(core, session);
}

} // namespace

bool GpuProcessCore::can_restart_after_reset() const noexcept {
    if (!initialized || !registered || !failed || !parked || reset_locked ||
        !reset_retired) return false;
    const auto retained = queues.report();
    if (retained.processes != 0 || retained.gangs != 0 || retained.queues != 0 ||
        retained.reset_required) return false;
    for (const auto& session : sessions) {
        if (session.reset_preserved) {
            if (session.process == nullptr || session.handle == 0 || session.pasid != 0 ||
                session.gang != 0 || session.quarantined || session.exiting || session.poisoned ||
                !session.buffers.ready_for_reset_rebind() ||
                !session.submissions.ready_for_reset_rebind() ||
                !session.graphics.ready_for_reset_rebind() ||
                session.tables.generation == 0 || session.table_backup.generation == 0 ||
                session.context.generation == 0 || session.gang_context.generation == 0)
                return false;
            for (const auto& queue : session.queues) {
                if (queue.id != 0 || queue.quarantined ||
                    (queue.handle != 0 && (queue.ring.generation == 0 || queue.mqd.generation == 0)) ||
                    (queue.handle == 0 && (queue.ring.generation != 0 || queue.mqd.generation != 0)))
                    return false;
            }
            continue;
        }
        // A poisoned tombstone keeps only its process reference and handle.
        const bool tombstone = session.poisoned && session.process != nullptr && session.handle != 0;
        if ((!tombstone && (session.process != nullptr || session.handle != 0)) ||
            session.pasid != 0 || session.gang != 0 || session.quarantined || session.exiting ||
            !session.buffers.empty() || !session.submissions.empty() || !session.graphics.empty() ||
            session.tables.generation != 0 ||
            session.table_backup.generation != 0 || session.context.generation != 0 ||
            session.gang_context.generation != 0) return false;
        for (const auto& queue : session.queues) {
            if (queue.handle != 0 || queue.id != 0 || queue.quarantined ||
                queue.ring.generation != 0 || queue.mqd.generation != 0) return false;
        }
    }
    return true;
}

bool process_core_bind(GpuProcessCore& core, const ProcessPlatform& platform) noexcept {
    if (platform.gtt == nullptr || platform.vram == nullptr || platform.dma_bus == nullptr ||
        platform.queue_config == nullptr || platform.now_us == nullptr) return false;
    core.platform = platform;
    return initialize_queues(core);
}

Outcome process_core_execute(GpuProcessCore& core, void* process) noexcept {
    const auto& r = core.request;
    const auto op = static_cast<Operation>(r.operation);
    core.reply.lost_buffer = 0;
    if (op == Operation::query) {
        if (recovery_abi(core)) core.reply.bytes = core.counters.reset_generation;
        return core.initialized && !core.failed ? Outcome::success : Outcome::unavailable;
    }
    if (!core.initialized || core.failed || core.parked) return Outcome::unavailable;
    if (op == Operation::open) return open_session(core, process);
    auto* session = authorize(core, process, r.session);
    if (session == nullptr) return Outcome::access_denied;
    core.reply.session = session->handle;
    core.reply.handle = r.handle;
    if (session->poisoned) {
        if (op != Operation::close) return versioned(core, Outcome::session_lost);
        if (session->quarantined) return Outcome::quarantined;
        retire_identity(core, *session);
        return Outcome::success;
    }
    session->buffers.take_lost_touch();
    const Outcome result = dispatch(core, *session, op);
    const uint64_t lost = session->buffers.take_lost_touch();
    // A refused request that named a buffer whose VRAM a reset destroyed: that
    // buffer is answered lost exactly once and its handle retires now.
    if (lost != 0 && result != Outcome::success && session->process != nullptr &&
        session->buffers.retire_lost(lost)) {
        ++core.counters.lost_buffers;
        core.reply.payload_bytes = 0;
        if (recovery_abi(core)) {
            core.reply.lost_buffer = lost;
            return Outcome::buffer_lost;
        }
        return Outcome::quarantined;
    }
    return result;
}

bool process_core_mark_exit(GpuProcessCore& core, void* process) noexcept {
    bool matched = false;
    for (auto& session : core.sessions) {
        if (session.process != process || process == nullptr || session.exiting) continue;
        session.exiting = true;
        const uint64_t time = core.platform.now_us != nullptr ? now(core) : 0;
        session.exit_deadline_us = time > UINT64_MAX - process_reap_idle_timeout_us ? UINT64_MAX :
            time + process_reap_idle_timeout_us;
        matched = true;
    }
    return matched;
}

ProcessReapResult process_core_reap(GpuProcessCore& core) noexcept {
    ProcessReapResult result{};
    if (!core.initialized) return result;
    const bool running = !core.parked && !core.failed && !core.reset_locked;
    if (running) core.queues.poll();
    const uint64_t time = now(core);
    for (auto& session : core.sessions) {
        if (!session.exiting || session.process == nullptr) continue;
        // Without a running incarnation only a resource-free tombstone retires;
        // everything else waits for resume, rearm or the reset retirement.
        if (!running && !(session.poisoned && !session.quarantined)) continue;
        if (reap_session(core, session, time)) result.retry_us = process_reap_retry_us;
    }
    return result;
}

bool process_core_take_escalation(GpuProcessCore& core, ProcessRecoveryHook& hook) noexcept {
    hook = {};
    if (!core.escalation_pending) return false;
    core.escalation_pending = false;
    hook = core.recovery;
    return hook.request != nullptr;
}

ProcessReaperPass process_core_reaper_pass(GpuProcessCore& core, const ProcessLock& service_lock,
                                           const bool& stop) noexcept {
    ProcessReaperPass pass{};
    ProcessRecoveryHook hook{};
    bool escalate_now = false;
    service_lock.acquire(service_lock.context);
    if (stop) {
        pass.stopped = true;
    } else {
        pass.retry_us = process_core_reap(core).retry_us;
        escalate_now = process_core_take_escalation(core, hook);
    }
    service_lock.release(service_lock.context);
    // The device owner takes its own outer lock: never call it under ours.
    if (escalate_now) hook.request(hook.context);
    return pass;
}

void process_core_begin_reset(GpuProcessCore& core) noexcept {
    core.reset_locked = true;
    core.reset_retired = false;
    core.failed = true;
    for (auto& session : core.sessions) {
        // amdgpu_userq_pre_reset keeps every live queue's owner; an exiting,
        // closing or uncertain session is released by the reset instead.
        session.reset_preserved = session.process != nullptr && !session.exiting &&
            !session.quarantined && !session.poisoned &&
            (session.pasid != 0 || session.reset_preserved);
    }
}

bool process_core_retire_reset(GpuProcessCore& core, bool (*confirm)(void*) noexcept,
                               void* confirm_context) noexcept {
    if (!core.reset_locked) return false;
    // The last fence each queue emitted: rebuilt queues continue after it.
    for (auto& session : core.sessions) {
        for (auto& queue : session.queues) {
            if (queue.id == 0) continue;
            const auto report = core.queues.queue_report(queue.id);
            if (report.queue_id == queue.id) queue.fence_floor = report.sync_seq;
        }
    }
    if (core.queues.release_after_reset(confirm, confirm_context) != UserQueueStatus::released)
        return false;
    bool released = true;
    for (auto& session : core.sessions) {
        session.pasid = 0;
        session.gang = 0;
        if (session.reset_preserved && retire_preserved(core, session)) continue;
        // The reset proof also retires kernel gfx-ring jobs (no MES queue).
        if (session.graphics.kernel_pending()) {
            uint32_t queue = 0;
            uint32_t lost[graphics_jobs]{};
            session.graphics.retire_after_reset(queue, lost);
        }
        for (auto& queue : session.queues) queue.id = 0;
        if (session.process == nullptr) continue;
        if (session.reset_preserved || session.poisoned) {
            // Not reconstructible (or already poisoned): resources go now,
            // reset-safe, the tombstone keeps answering session_lost.
            if (!session.poisoned) ++core.counters.poisoned;
            session.poisoned = true;
            session.reset_preserved = false;
            if (release_resources(core, session)) {
                session.quarantined = false;
                session.escalated = false;
            } else {
                session.quarantined = true;
                released = false;
            }
            continue;
        }
        if (!close_session(core, session)) released = false;
    }
    ++core.counters.reset_generation;
    core.failed = true; // Explicit reset invalidates this service incarnation.
    core.parked = true;
    core.reset_retired = released;
    return released;
}

bool process_core_rearm(GpuProcessCore& core) noexcept {
    if (!core.initialized) return false;
    if (!core.failed && !core.parked && !core.reset_locked) {
        hardware::UserQueueConfig config{};
        return core.platform.queue_config(core.platform.context, core.pasids, config);
    }
    // A failed or merely suspended service never becomes ready by an
    // idempotent call. Only retired reset ownership plus a real new ASIC
    // configuration permits rearming. Sessions that exited after the reset
    // retirement are closed first (no VM remains to wait for).
    for (auto& session : core.sessions) {
        if (!session.exiting || session.quarantined || session.pasid != 0 || session.process == nullptr)
            continue;
        if (session.poisoned) {
            retire_identity(core, session);
            continue;
        }
        session.reset_preserved = false;
        if (!close_session(core, session)) return false;
    }
    if (!core.can_restart_after_reset() || !initialize_queues(core)) return false;
    core.reset_retired = false;
    for (auto& session : core.sessions) {
        if (session.reset_preserved) rebuild_or_poison(core, session);
    }
    core.failed = false;
    core.parked = false;
    return true;
}

bool process_core_suspend(GpuProcessCore& core) noexcept {
    if (!core.initialized) return true;
    if (core.failed) return false;
    if (core.parked) return true;
    // Kernel gfx-ring jobs drain first (amdgpu_fence_driver_hw_fini waits on
    // the rings); a job still running refuses this transition, retryable.
    for (auto& session : core.sessions) {
        if (session.graphics.kernel_pending() &&
            session.graphics.wait_kernel_idle(graphics_kernel_drain_timeout_us) != memory::Status::success)
            return false;
    }
    const auto status = core.queues.suspend_for_recovery();
    if (status != UserQueueStatus::suspended) {
        // Before any queue detached, a busy refusal can restore D0 admission.
        if (core.queues.cancel_recovery() != UserQueueStatus::ready) core.failed = true;
        return false;
    }
    core.parked = true;
    for (auto& session : core.sessions) {
        if (session.pasid != 0 && (session.submissions.poll() != memory::Status::success ||
            session.graphics.poll() != memory::Status::success ||
            session.buffers.save_for_recovery() != memory::Status::success)) {
            core.failed = true; return false;
        }
    }
    return true;
}

bool process_core_resume(GpuProcessCore& core) noexcept {
    if (!core.initialized) return true;
    if (core.failed) return false;
    if (!core.parked) return true;
    for (auto& session : core.sessions) {
        if (session.pasid != 0 && session.buffers.restore_after_recovery() != memory::Status::success) {
            core.failed = true; return false;
        }
    }
    if (core.queues.resume_after_recovery() != UserQueueStatus::ready) {
        core.failed = true; return false;
    }
    // Sessions that exited while parked are reaped by the reaper thread.
    core.parked = false;
    return true;
}

bool process_core_close_all(GpuProcessCore& core) noexcept {
    bool released = true;
    for (auto& session : core.sessions) {
        if (session.process == nullptr) continue;
        if (session.graphics.kernel_pending()) {
            session.graphics.wait_kernel_idle(graphics_kernel_drain_timeout_us);
        }
        if (session.poisoned && !session.quarantined) {
            retire_identity(core, session);
            continue;
        }
        if (!close_session(core, session)) {
            released = false;
            if (session.quarantined) core.failed = true;
        }
    }
    // Every process VM is gone, so no queue can still read the shared ring.
    return released && release_attribute_ring(core);
}

uint32_t process_core_quarantine_vram(GpuProcessCore& core, uint64_t mc_address, uint64_t bytes,
                                      uint32_t* owners) noexcept {
    uint32_t quarantined = 0;
    uint32_t owning = 0;
    for (auto& session : core.sessions) {
        if (session.process == nullptr || !session.buffers.owns_vram(mc_address, bytes)) continue;
        ++owning;
        if (session.quarantined || session.poisoned) continue;
        session.quarantined = true;
        ++quarantined;
    }
    if (owners != nullptr) *owners = owning;
    return quarantined;
}

bool process_core_configure_scanout(GpuProcessCore& core, const GraphicsScanout* scanout) noexcept {
    for (const auto& session : core.sessions) {
        // Existing graphics mappings were built without (or with another) scanout.
        if (session.graphics.opened()) return false;
    }
    if (scanout == nullptr) {
        core.graphics.scanout = false;
        return true;
    }
    if (scanout->width == 0 || scanout->height == 0 || scanout->width > 16384 ||
        scanout->height > 16384 || scanout->pitch_bytes < scanout->width * 4u ||
        (scanout->pitch_bytes & 3u) != 0 || (scanout->mc_address & 3u) != 0 ||
        static_cast<uint64_t>(scanout->pitch_bytes) * scanout->height + memory::page_bytes >
            graphics_scanout_max_bytes) return false;
    core.graphics.scanout = true;
    core.graphics.scanout_mc = scanout->mc_address;
    core.graphics.scanout_width = scanout->width;
    core.graphics.scanout_height = scanout->height;
    core.graphics.scanout_pitch_bytes = scanout->pitch_bytes;
    return true;
}

Outcome process_core_present(GpuProcessCore& core, void* process, uint32_t& copied) noexcept {
    copied = 0;
    if (core.failed || core.parked) return Outcome::unavailable;
    auto* session = authorize(core, process, core.present_request.input.session);
    if (session == nullptr) return Outcome::access_denied;
    if (session->poisoned) return Outcome::quarantined;
    session->buffers.take_lost_touch();
    const Outcome result = outcome(session->graphics.present(core.present_request, copied));
    // The present reply carries no handle field: a lost source answers
    // quarantined and retires, exactly once, like a version-3 request.
    const uint64_t lost = session->buffers.take_lost_touch();
    if (lost != 0 && result != Outcome::success && session->buffers.retire_lost(lost)) {
        ++core.counters.lost_buffers;
        copied = 0;
        return Outcome::quarantined;
    }
    return result;
}

} // namespace rx7900::reactos

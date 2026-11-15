/*
Module: rx7900.hardware.user_queue
Purpose: Create, feed, observe and tear down MES-managed process queues in their own GPUVM.
Created: 2026-10-22
Architecture: Bounded fixed storage over Mes11 (frames, doorbells, queue ids,
MQD init), GpuVm (process page tables) and the caller's GTT Manager; ring frames
from gfx11_queues and the Mes11 SDMA builders; fences on each queue's control page.
Copyright 2019 Advanced Micro Devices, Inc.; Copyright 2009 Jerome Glisse;
Copyright 2015 Advanced Micro Devices, Inc.
Adapted Linux v6.12 amdgpu_mes.c, amdgpu_mes_ctx.h, mes_v11_0.c, amdgpu_ring.c,
amdgpu_ib.c, amdgpu_fence.c, gfx_v11_0.c, sdma_v6_0.c, amdgpu_vm.c, amdgpu_ids.c
and drm sched_main.c (see user_queue.hpp). Notice and provenance:
licenses/AMDGPU-FIRMWARE.txt.
*/
#include "user_queue.hpp"
#include "sdma6_init.hpp"

namespace rx7900::hardware {
namespace {

constexpr uint32_t half_range = 0x80000000u;
constexpr uint32_t sdma_nop = 0;
// gfx11_kiq style pipe priorities of amdgpu_mes_ring_to_queue_props:
// AMDGPU_GFX_PIPE_PRIO_NORMAL and AMDGPU_GFX_QUEUE_PRIORITY_MINIMUM.
constexpr uint32_t pipe_priority_normal = 1;
constexpr uint32_t queue_priority_minimum = 0;
// KFD/amdgpu_amdkfd_flush_gpu_tlb_pasid types after map and unmap.
constexpr uint32_t tlb_flush_legacy = 0;
constexpr uint32_t tlb_flush_heavyweight = 2;
// amdgpu_ctx_map_meta_data flags: READABLE | WRITEABLE | EXECUTABLE, MTYPE default.
constexpr GpuVmMapFlags queue_map_flags{memory::Access::read_write, true, GpuVmMtype::nc, false};
// AMDGPU_PTE_VALID and AMDGPU_PTE_READABLE.
constexpr uint64_t pte_valid = 1;
constexpr uint64_t pte_readable = 1ULL << 5;
constexpr uint64_t pte_huge = 1ULL << 54;
constexpr uint64_t gpu_page = 4096;

uint64_t saturating_add(uint64_t a, uint64_t b) noexcept {
    return a > UINT64_MAX - b ? UINT64_MAX : a + b;
}

bool power_of_two(uint32_t value) noexcept { return value != 0 && (value & (value - 1)) == 0; }

uint32_t mqd_bytes(Mes11QueueType type) noexcept {
    return type == Mes11QueueType::sdma ? mes11_sdma_mqd_bytes : mes11_mqd_bytes;
}

uint32_t default_timeout_ms(Mes11QueueType type) noexcept {
    return type == Mes11QueueType::gfx ? fence_gfx_timeout_ms :
           type == Mes11QueueType::compute ? fence_compute_timeout_ms : fence_sdma_timeout_ms;
}

bool zero(uint8_t* cpu, uint64_t bytes) noexcept {
    const sdma::CommandBuffer out(cpu, static_cast<size_t>(bytes));
    bool ok = out.size() == bytes;

    for (uint64_t offset = 0; ok && offset < bytes; offset += 4) {
        ok = out.write_u32(static_cast<size_t>(offset), 0);
    }

    return ok;
}

bool valid_type(Mes11QueueType type) noexcept {
    return type == Mes11QueueType::gfx || type == Mes11QueueType::compute ||
           type == Mes11QueueType::sdma;
}

} // namespace

bool UserQueueInterruptHint::observe(const IrqDispatchView& view) noexcept {
    if (view.report == nullptr) {
        return false;
    }

    const uint32_t eop = view.report->sequence[static_cast<uint32_t>(IrqSource::mes_eop)];
    const uint32_t trap = view.report->sequence[static_cast<uint32_t>(IrqSource::sdma_mes_trap)];
    const bool progress = eop != mes_eop_seen_ || trap != sdma_mes_seen_ || view.events_lost;
    mes_eop_seen_ = eop;
    sdma_mes_seen_ = trap;
    return progress;
}

UserQueueStatus UserQueues::finish(UserQueueStatus status) noexcept {
    status_ = status;
    return status;
}

bool UserQueues::confirm_process_idle(void* context) noexcept {
    return static_cast<const Process*>(context)->idle;
}

UserQueueStatus UserQueues::initialize(const UserQueueConfig& config) noexcept {
    for (const auto& process : processes_) {
        if (process.used) {
            return UserQueueStatus::busy;
        }
    }

    initialized_ = false;
    recovery_blocked_ = false;
    recovery_saved_ = false;
    recovery_detached_ = false;
    uint32_t offset = 0;
    const uint32_t nbif = config.nbio_registers.valid() ?
        config.nbio_registers.nbif_base(gfx11_hdp_flush_nbif_segment) : 0;

    if (config.mes == nullptr || config.pasids == nullptr || config.invalidator == nullptr ||
        config.nbio == nullptr || config.vram_manager == nullptr || config.gtt_manager == nullptr ||
        config.clock.now_us == nullptr || config.wptr_store.store == nullptr ||
        !doorbell_io_valid(config.doorbells) || config.vram.bytes == 0 ||
        config.gart.end <= config.gart.start || !config.nbio_registers.valid() ||
        !discovery_register_offset(nbif, gfx11_hdp_flush_done_register,
                                   config.nbio_registers.aperture_bytes(), offset)) {
        return finish(UserQueueStatus::invalid_argument);
    }

    config_ = config;
    hdp_flush_req_ = nbif + gfx11_hdp_flush_req_register;
    hdp_flush_done_ = nbif + gfx11_hdp_flush_done_register;
    initialized_ = true;
    return finish(UserQueueStatus::ready);
}

bool UserQueues::find_process(uint32_t pasid, uint32_t& index) const noexcept {
    for (index = 0; index < user_queue_max_processes; ++index) {
        if (processes_[index].used && processes_[index].pasid == pasid && pasid != 0) {
            return true;
        }
    }

    return false;
}

bool UserQueues::find_gang(uint32_t gang_id, uint32_t& index) const noexcept {
    for (index = 0; index < user_queue_max_gangs; ++index) {
        if (gangs_[index].used && gangs_[index].gang_id == gang_id && gang_id != 0) {
            return true;
        }
    }

    return false;
}

bool UserQueues::find_queue(uint32_t queue_id, uint32_t& index) const noexcept {
    for (index = 0; index < user_queue_max_queues; ++index) {
        const UserQueueReport& report = queues_[index].report;

        if (report.state != UserQueueState::free && report.queue_id == queue_id && queue_id != 0) {
            return true;
        }
    }

    return false;
}

bool UserQueues::address_in_use(uint64_t gart) const noexcept {
    for (const auto& process : processes_) {
        if (process.used && (process.context_gart == gart ||
            (process.backup_cpu != nullptr && process.backup_gart == gart))) {
            return true;
        }
    }

    for (const auto& gang : gangs_) {
        if (gang.used && gang.context_gart == gart) {
            return true;
        }
    }

    for (const auto& queue : queues_) {
        if (queue.report.state != UserQueueState::free &&
            (queue.report.buffer_gart == gart || queue.report.mqd_gart == gart)) {
            return true;
        }
    }

    return false;
}

// amdgpu_bo_create_kernel(AMDGPU_GEM_DOMAIN_GTT): a page-aligned, CPU-mapped
// allocation inside the VMID0 GART window, leased read/write here.
bool UserQueues::retain_gtt(memory::Handle handle, uint64_t bytes,
                            memory::AllocationInfo& info) noexcept {
    memory::Manager& manager = *config_.gtt_manager;
    sdma::GpuRange range{};
    info = {};

    if (manager.describe(handle, info) != memory::Status::success ||
        manager.resolve({handle, 0}, bytes, memory::Access::read_write, range) !=
            memory::Status::success) {
        return false;
    }

    const uint64_t start = info.gpu.virtual_address;
    return (start & (gpu_page - 1)) == 0 && info.gpu.byte_length >= bytes &&
           start >= config_.gart.start && start <= config_.gart.end &&
           bytes - 1 <= config_.gart.end - start && info.backing.cpu_bytes != nullptr &&
           info.backing.cpu_byte_count >= bytes && !address_in_use(start) &&
           manager.retain(handle, memory::Access::read_write) == memory::Status::success;
}

UserQueueStatus UserQueues::create_process(const UserProcessMemory& memory,
                                           uint32_t& pasid) noexcept {
    pasid = 0;

    if (!initialized_) {
        return UserQueueStatus::unavailable;
    }

    if (recovery_blocked_) {
        return UserQueueStatus::suspended;
    }

    uint32_t index = 0;

    while (index < user_queue_max_processes && processes_[index].used) {
        ++index;
    }

    if (index == user_queue_max_processes) {
        return finish(UserQueueStatus::exhausted);
    }

    memory::AllocationInfo context{};

    if (memory.table_count == 0 || memory.table_count > gpuvm_max_tables ||
        !retain_gtt(memory.process_context, mes11_process_context_bytes, context)) {
        return finish(UserQueueStatus::invalid_memory);
    }

    // amdgpu_mes_create_process: memset(proc_ctx_cpu_ptr, 0, AMDGPU_MES_PROC_CTX_SIZE).
    if (!zero(context.backing.cpu_bytes, mes11_process_context_bytes)) {
        config_.gtt_manager->unretain(memory.process_context);
        return finish(UserQueueStatus::invalid_memory);
    }

    uint32_t id = 0;

    if (config_.pasids->allocate(gpuvm_pasid_bits, id) != GpuVmStatus::ready) {
        config_.gtt_manager->unretain(memory.process_context);
        return finish(UserQueueStatus::exhausted);
    }

    Process& process = processes_[index];
    process = {false, true, false, id, memory.process_context, context.gpu.virtual_address,
               {}, nullptr, 0, 0};
    GpuVm& vm = vms_[index];
    const GpuVmStatus initialized = vm.init(*config_.vram_manager,
        {config_.vram, memory.page_tables, memory.table_count, {&process, confirm_process_idle}},
        *config_.nbio);

    const bool pasid_set = initialized == GpuVmStatus::ready &&
                           vm.set_pasid(id) == GpuVmStatus::ready;

    if (!pasid_set) {
        // No queue references the VM yet: amdgpu_vm_fini is safe (idle).
        if (vm.report().memory_retained) {
            vm.fini();
        }

        config_.pasids->release(id);
        config_.gtt_manager->unretain(memory.process_context);
        process = {};
        return finish(initialized == GpuVmStatus::invalid_argument ||
                      initialized == GpuVmStatus::invalid_memory ?
                      UserQueueStatus::invalid_memory : UserQueueStatus::vm_failure);
    }

    if (memory.table_backup.generation != 0) {
        memory::AllocationInfo backup{};
        const uint64_t bytes = uint64_t{memory.table_count} * gpuvm_table_bytes;
        const bool retained = retain_gtt(memory.table_backup, bytes, backup);
        if (!retained || backup.gpu.virtual_address == context.gpu.virtual_address) {
            if (retained) {
                config_.gtt_manager->unretain(memory.table_backup);
            }
            vm.fini();
            config_.pasids->release(id);
            config_.gtt_manager->unretain(memory.process_context);
            process = {};
            return finish(UserQueueStatus::invalid_memory);
        }
        process.backup = memory.table_backup;
        process.backup_cpu = backup.backing.cpu_bytes;
        process.backup_gart = backup.gpu.virtual_address;
        process.backup_bytes = bytes;
    }

    process.used = true;
    process.idle = false;
    pasid = id;
    return finish(UserQueueStatus::created);
}

GpuVmStatus UserQueues::map_system(uint32_t pasid, uint64_t va, ByteView dma_pages,
                                   bool cpu_cached, GpuVmMapFlags flags) noexcept {
    uint32_t index = 0;

    if (recovery_blocked_ || !find_process(pasid, index) || processes_[index].quarantined) {
        return GpuVmStatus::unavailable;
    }

    return vms_[index].map_system(va, dma_pages, cpu_cached, flags);
}

GpuVmStatus UserQueues::map_vram(uint32_t pasid, uint64_t va, uint64_t vram_mc, uint64_t bytes,
                                 GpuVmMapFlags flags) noexcept {
    uint32_t index = 0;

    if (recovery_blocked_ || !find_process(pasid, index) || processes_[index].quarantined) {
        return GpuVmStatus::unavailable;
    }

    return vms_[index].map_vram(va, vram_mc, bytes, flags);
}

GpuVmStatus UserQueues::unmap(uint32_t pasid, uint64_t va) noexcept {
    uint32_t index = 0;

    if (recovery_blocked_ || !find_process(pasid, index) || processes_[index].quarantined) {
        return GpuVmStatus::unavailable;
    }

    // Queue buffers are unmapped only by remove_queue.
    for (const auto& queue : queues_) {
        if (queue.report.state != UserQueueState::free && queue.process == index &&
            queue.report.queue_va == va) {
            return GpuVmStatus::invalid_argument;
        }
    }

    return vms_[index].unmap(va);
}

GpuVmStatus UserQueues::flush_tlb(uint32_t pasid, uint32_t flush_type) noexcept {
    uint32_t index = 0;

    if (recovery_blocked_ || !find_process(pasid, index) || processes_[index].quarantined) {
        return GpuVmStatus::unavailable;
    }

    return vms_[index].flush_tlb(*config_.invalidator, flush_type);
}

GpuVmReport UserQueues::vm_report(uint32_t pasid) const noexcept {
    uint32_t index = 0;
    return find_process(pasid, index) ? vms_[index].report() : GpuVmReport{};
}

UserQueueStatus UserQueues::add_gang(uint32_t pasid, const UserGangProperties& properties,
                                     uint32_t& gang_id) noexcept {
    gang_id = 0;
    uint32_t process = 0;

    if (!initialized_) {
        return UserQueueStatus::unavailable;
    }

    if (recovery_blocked_) {
        return UserQueueStatus::suspended;
    }

    if (!find_process(pasid, process)) {
        return finish(UserQueueStatus::not_found);
    }

    if (processes_[process].quarantined) {
        return finish(UserQueueStatus::quarantined);
    }

    if (properties.global_priority_level >= mes11_priority_levels ||
        properties.inprocess_gang_priority >= mes11_priority_levels) {
        return finish(UserQueueStatus::invalid_argument);
    }

    uint32_t slot = 0;

    while (slot < user_queue_max_gangs && gangs_[slot].used) {
        ++slot;
    }

    if (slot == user_queue_max_gangs) {
        return finish(UserQueueStatus::exhausted);
    }

    memory::AllocationInfo context{};

    if (!retain_gtt(properties.gang_context, mes11_gang_context_bytes, context)) {
        return finish(UserQueueStatus::invalid_memory);
    }

    if (!zero(context.backing.cpu_bytes, mes11_gang_context_bytes)) {
        config_.gtt_manager->unretain(properties.gang_context);
        return finish(UserQueueStatus::invalid_memory);
    }

    // idr_alloc(&gang_id_idr, gang, 1, 0): lowest free id from 1.
    uint32_t id = 1;

    for (uint32_t probe = 0; find_gang(id, probe); ++id) {
    }

    gangs_[slot] = {true, false, id, process, properties.gang_context, context.gpu.virtual_address,
                    properties.gang_quantum, properties.global_priority_level,
                    properties.inprocess_gang_priority};
    gang_id = id;
    return finish(UserQueueStatus::added);
}

// amdgpu_ring_init for a MES queue on a fresh buffer: the queue_alloc memset,
// amdgpu_ring_clear_ring with the engine NOP, *cond_exe_cpu_addr = 1 and the
// fence driver's last_seq 0 (zero already).
bool UserQueues::prepare_buffer(Queue& queue) noexcept {
    const uint32_t ring = queue.report.ring_bytes;
    const uint32_t nop = queue.report.type == Mes11QueueType::sdma ? sdma_nop : gfx11_ring_nop;
    const sdma::CommandBuffer out(queue.cpu, ring + user_queue_control_bytes);
    bool ok = zero(queue.cpu, uint64_t{ring} + user_queue_control_bytes);

    for (uint32_t offset = 0; ok && offset < ring; offset += 4) {
        ok = out.write_u32(offset, nop);
    }

    return ok && out.write_u32(ring + user_queue_cond_exec_offset, 1);
}

// Undo a queue that MES never saw: ids, mapping and leases.
UserQueueStatus UserQueues::rollback_queue(uint32_t index, bool free_ids,
                                           UserQueueStatus status) noexcept {
    Queue& queue = queues_[index];
    GpuVm& vm = vms_[queue.process];

    if (free_ids) {
        config_.mes->free_doorbell(queue.report.doorbell_index);
        config_.mes->free_queue_id(queue.report.queue_id);
        queue.resources_registered = false;
    }

    if (queue.mapped) {
        // Nothing ran on the queue: the PTEs were never used by any engine.
        if (vm.unmap(queue.report.queue_va) != GpuVmStatus::unmapped ||
            (vm.tlb_flush_needed() &&
             vm.flush_tlb(*config_.invalidator, tlb_flush_heavyweight) != GpuVmStatus::flushed)) {
            queue.report.state = UserQueueState::quarantined;
            processes_[queue.process].quarantined = true;
            return finish(UserQueueStatus::quarantined);
        }
    }

    config_.gtt_manager->unretain(queue.buffer);
    config_.gtt_manager->unretain(queue.mqd);
    queue = {};
    return finish(status);
}

UserQueueStatus UserQueues::add_queue(uint32_t gang_id, const UserQueueProperties& properties,
                                      uint32_t& queue_id) noexcept {
    queue_id = 0;
    uint32_t gang = 0;

    if (!initialized_) {
        return UserQueueStatus::unavailable;
    }

    if (recovery_blocked_) {
        return UserQueueStatus::suspended;
    }

    if (!find_gang(gang_id, gang)) {
        return finish(UserQueueStatus::not_found);
    }

    const uint32_t process = gangs_[gang].process;

    if (gangs_[gang].quarantined || processes_[process].quarantined) {
        return finish(UserQueueStatus::quarantined);
    }

    const uint32_t ring = properties.ring_bytes;
    const uint64_t bytes = uint64_t{ring} + user_queue_control_bytes;
    const uint64_t va_limit = gpuvm_max_pfn << 12;

    if (!valid_type(properties.type) || !power_of_two(ring) || ring < user_queue_min_ring_bytes ||
        ring > user_queue_max_ring_bytes || properties.queue_va == 0 ||
        (properties.queue_va & (gpu_page - 1)) != 0 || properties.queue_va >= va_limit ||
        bytes > va_limit - properties.queue_va ||
        properties.buffer_dma_pages.size() != bytes / gpu_page * 8 ||
        properties.timeout_ms > fence_max_timeout_ms) {
        return finish(UserQueueStatus::invalid_argument);
    }

    if (!config_.mes->report().scheduler_ready) {
        return finish(UserQueueStatus::unavailable);
    }

    uint32_t index = 0;

    while (index < user_queue_max_queues && queues_[index].report.state != UserQueueState::free) {
        ++index;
    }

    if (index == user_queue_max_queues) {
        return finish(UserQueueStatus::exhausted);
    }

    memory::AllocationInfo buffer{};
    memory::AllocationInfo mqd{};

    if (!retain_gtt(properties.buffer, bytes, buffer)) {
        return finish(UserQueueStatus::invalid_memory);
    }

    // The two handles must name different allocations (address_in_use does not
    // see the buffer before the slot is published).
    if (config_.gtt_manager->describe(properties.mqd, mqd) != memory::Status::success ||
        mqd.gpu.virtual_address == buffer.gpu.virtual_address ||
        !retain_gtt(properties.mqd, mqd_bytes(properties.type), mqd)) {
        config_.gtt_manager->unretain(properties.buffer);
        return finish(UserQueueStatus::invalid_memory);
    }

    Queue& queue = queues_[index];
    queue = {};
    queue.gang = gang;
    queue.process = process;
    queue.buffer = properties.buffer;
    queue.mqd = properties.mqd;
    queue.cpu = buffer.backing.cpu_bytes;
    UserQueueReport& report = queue.report;
    report.state = UserQueueState::active;
    report.type = properties.type;
    report.pasid = processes_[process].pasid;
    report.gang_id = gang_id;
    report.ring_bytes = ring;
    report.queue_va = properties.queue_va;
    report.buffer_gart = buffer.gpu.virtual_address;
    report.mqd_gart = mqd.gpu.virtual_address;
    report.timeout_ms = properties.timeout_ms != 0 ? properties.timeout_ms :
                                                     default_timeout_ms(properties.type);

    if (!prepare_buffer(queue) || !zero(mqd.backing.cpu_bytes, mqd_bytes(properties.type))) {
        return rollback_queue(index, false, UserQueueStatus::invalid_memory);
    }

    // amdgpu_mes_ctx_map_meta_data: amdgpu_vm_bo_map + amdgpu_vm_bo_update +
    // amdgpu_vm_update_pdes; Linux waits for the update fence before ADD_QUEUE.
    GpuVm& vm = vms_[process];
    const GpuVmStatus mapped = vm.map_system(properties.queue_va, properties.buffer_dma_pages, true,
                                             queue_map_flags);

    if (mapped != GpuVmStatus::mapped) {
        return rollback_queue(index, false, mapped == GpuVmStatus::invalid_argument ||
                                            mapped == GpuVmStatus::overlap ?
                                            UserQueueStatus::invalid_argument :
                                            UserQueueStatus::vm_failure);
    }

    queue.mapped = true;

    if (vm.tlb_flush_needed() &&
        vm.flush_tlb(*config_.invalidator, tlb_flush_legacy) != GpuVmStatus::flushed) {
        return rollback_queue(index, false, UserQueueStatus::vm_failure);
    }

    // amdgpu_mes_add_hw_queue: queue id, then the MES doorbell.
    Mes11& mes = *config_.mes;

    if (mes.allocate_queue_id(report.queue_id) != Mes11Status::completed) {
        return rollback_queue(index, false, UserQueueStatus::exhausted);
    }

    if (mes.allocate_doorbell(properties.type, report.doorbell_index) != Mes11Status::completed) {
        mes.free_queue_id(report.queue_id);
        report.queue_id = 0;
        return rollback_queue(index, false, UserQueueStatus::exhausted);
    }
    queue.resources_registered = true;

    const uint64_t control_va = properties.queue_va + ring;
    const uint64_t control_gart = report.buffer_gart + ring;
    const Gfx11MqdProperties mqd_properties{report.mqd_gart, properties.queue_va,
        control_va + user_queue_rptr_offset, control_va + user_queue_wptr_offset,
        properties.type == Mes11QueueType::compute ? control_va + user_queue_eop_offset : 0, ring,
        report.doorbell_index, pipe_priority_normal, queue_priority_minimum, false, false};

    if (mes.init_queue_mqd(properties.type, mqd_properties,
                           sdma::CommandBuffer(mqd.backing.cpu_bytes, mqd_bytes(properties.type))) !=
        Mes11Status::completed) {
        // No frame referenced the queue: its buffers are safe to return.
        return rollback_queue(index, true, UserQueueStatus::mes_failure);
    }

    const Gang& owner = gangs_[gang];
    processes_[process].idle = false;
    report.add_status = mes.add_hw_queue({properties.type, report.pasid,
        vm.page_table_base_addr(), user_queue_process_va_end, 0,
        processes_[process].context_gart, owner.quantum, owner.context_gart,
        owner.inprocess_gang_priority, owner.global_priority_level, report.doorbell_index,
        report.mqd_gart, control_va + user_queue_wptr_offset,
        control_gart + user_queue_wptr_offset});

    if (report.add_status != Mes11Status::completed) {
        // Refused before or without a frame: MES never saw the queue.
        if (mes.report().scheduler_ready) {
            return rollback_queue(index, true, UserQueueStatus::mes_failure);
        }

        // ADD_QUEUE may still complete inside MES: keep everything; queue_id
        // still names the slot for queue_report.
        queue_id = report.queue_id;
        report.state = UserQueueState::quarantined;
        gangs_[gang].quarantined = true;
        processes_[process].quarantined = true;
        return finish(UserQueueStatus::quarantined);
    }

    queue_id = report.queue_id;
    return finish(UserQueueStatus::added);
}

void UserQueues::hang(Queue& queue, FenceHangReason reason) noexcept {
    UserQueueReport& report = queue.report;

    if (report.state != UserQueueState::active) {
        return;
    }

    report.state = UserQueueState::hung;
    report.hang_reason = reason;
    report.hang_signalled_seq = report.last_seq;
    report.hang_emitted_seq = report.sync_seq;
    report.timeout_armed = false;
}

// amdgpu_fence_process on the queue's fence slot, with FenceManager's window rule
// and drm_sched_get_finished_job's timeout restart.
void UserQueues::process_fence(Queue& queue, uint64_t now) noexcept {
    UserQueueReport& report = queue.report;

    if (report.state != UserQueueState::active) {
        return;
    }

    uint32_t value = 0;

    if (!ByteView(queue.cpu, report.ring_bytes + user_queue_control_bytes)
             .read_u32(report.ring_bytes + user_queue_fence_offset, value)) {
        hang(queue, FenceHangReason::writeback_unreadable);
        return;
    }

    if (value - report.last_seq > report.sync_seq - report.last_seq) {
        hang(queue, FenceHangReason::fence_out_of_window);
        return;
    }

    if (value == report.last_seq) {
        return;
    }

    report.last_seq = value;
    report.retired_position = queue.end_position[value % user_queue_fence_slots];
    report.timeout_armed = value != report.sync_seq;
    report.timeout_deadline_us = report.timeout_armed ?
        saturating_add(now, uint64_t{report.timeout_ms} * 1000u) : 0;
}

// Every page of the IB is a valid, readable PTE (or huge PDE) of the process VM.
bool UserQueues::ib_mapped(uint32_t index, const UserQueueIb& ib) const noexcept {
    const uint64_t bytes = uint64_t{ib.dwords} * 4;
    const uint64_t first = ib.va & ~(gpu_page - 1);
    const uint64_t last = (ib.va + bytes - 1) & ~(gpu_page - 1);

    for (uint64_t page = first; page <= last; page += gpu_page) {
        GpuVmWalk walk{};

        if (vms_[index].walk(page, walk) != GpuVmStatus::ready || walk.count == 0) {
            return false;
        }

        const GpuVmWalkEntry& entry = walk.entries[walk.count - 1];
        const bool leaf = entry.level == GpuVmLevel::ptb || (entry.value & pte_huge) != 0;

        if (!leaf || (entry.value & pte_valid) == 0 || (entry.value & pte_readable) == 0) {
            return false;
        }
    }

    return true;
}

UserQueueStatus UserQueues::emit(Queue& queue, const UserQueueIb& ib, uint32_t sequence,
                                 uint64_t& wptr) noexcept {
    const UserQueueReport& report = queue.report;
    const uint64_t control_va = report.queue_va + report.ring_bytes;
    Gfx11RingWriter writer{sdma::CommandBuffer(queue.cpu, report.ring_bytes), report.wptr};
    bool ok = false;

    if (report.type == Mes11QueueType::sdma) {
        ok = mes11_sdma_emit_ib_frame(writer, {control_va + user_queue_cond_exec_offset,
                 hdp_flush_req_, hdp_flush_done_, ib.va, ib.dwords,
                 control_va + user_queue_fence_offset, sequence, report.queue_id}) &&
             mes11_sdma_ring_pad(writer);
    } else {
        // amdgpu_ib_schedule: amdgpu_ib_get set AMDGPU_IB_FLAG_EMIT_MEM_SYNC, as
        // Gfx11Queues::submit; MES queue: inherit-vmid IB and RELEASE_MEM context
        // queue id | AMDGPU_FENCE_MES_QUEUE_FLAG; ring->pipe 0 of gfx/compute_ring[0].
        ok = gfx11_emit_ib_frame(writer, {report.type == Mes11QueueType::gfx, true, true,
                 report.queue_id, 0, control_va + user_queue_cond_exec_offset, hdp_flush_req_,
                 hdp_flush_done_, ib.va, ib.dwords, control_va + user_queue_fence_offset,
                 sequence}) && gfx11_ring_pad(writer);
    }

    const uint32_t alloc = report.type == Mes11QueueType::sdma ? sdma6_submit_alloc_dwords :
                                                                 gfx11_submit_alloc_dwords;

    if (!ok || writer.wptr - report.wptr > alloc) {
        return UserQueueStatus::invalid_memory;
    }

    wptr = writer.wptr;
    return UserQueueStatus::submitted;
}

UserQueueStatus UserQueues::submit(uint32_t queue_id, const UserQueueIb& ib,
                                   uint32_t& sequence) noexcept {
    sequence = 0;
    uint32_t index = 0;

    if (!initialized_) {
        return UserQueueStatus::unavailable;
    }

    if (recovery_blocked_) {
        return UserQueueStatus::suspended;
    }

    if (!find_queue(queue_id, index)) {
        return UserQueueStatus::not_found;
    }

    Queue& queue = queues_[index];
    UserQueueReport& report = queue.report;

    if (report.state != UserQueueState::active) {
        return report.state == UserQueueState::hung ? UserQueueStatus::hung :
                                                      UserQueueStatus::quarantined;
    }

    const Mes11Report mes = config_.mes->report();

    if (!mes.scheduler_ready) {
        return UserQueueStatus::unavailable;
    }

    if (mes.gangs_suspended) {
        return UserQueueStatus::suspended;
    }

    const bool sdma = report.type == Mes11QueueType::sdma;
    const uint64_t va_limit = gpuvm_max_pfn << 12;

    if (ib.va == 0 || ib.dwords == 0 || ib.dwords > gfx11_max_ib_dwords ||
        (ib.va & (sdma ? 31u : 3u)) != 0 || (sdma && (ib.dwords & 7u) != 0) ||
        ib.va >= va_limit || uint64_t{ib.dwords} * 4 > va_limit - ib.va) {
        return UserQueueStatus::invalid_argument;
    }

    if (!ib_mapped(queue.process, ib)) {
        return UserQueueStatus::invalid_memory;
    }

    const uint64_t time = now();
    process_fence(queue, time);

    if (report.state != UserQueueState::active) {
        return UserQueueStatus::hung;
    }

    // amdgpu_fence_emit's slot wait and amdgpu_ring_alloc's space check.
    const uint32_t alloc = sdma ? sdma6_submit_alloc_dwords : gfx11_submit_alloc_dwords;
    const uint64_t used = report.wptr - report.retired_position;

    if (report.sync_seq - report.last_seq >= user_queue_fence_slots ||
        used + alloc >= report.ring_bytes / 4) {
        return UserQueueStatus::ring_full;
    }

    const uint32_t seq = report.sync_seq + 1;
    uint64_t wptr = 0;

    if (emit(queue, ib, seq, wptr) != UserQueueStatus::submitted) {
        hang(queue, FenceHangReason::submit_failure);
        return finish(UserQueueStatus::hung);
    }

    // The fence becomes known before the engine can write it.
    queue.end_position[seq % user_queue_fence_slots] = wptr;
    report.sync_seq = seq;
    report.wptr = wptr;
    ++report.submissions;
    // drm_sched_job_begin -> drm_sched_start_timeout (mod_delayed_work re-arms).
    report.timeout_armed = true;
    report.timeout_deadline_us = saturating_add(time, uint64_t{report.timeout_ms} * 1000u);
    // gfx_v11_0_ring_set_wptr_gfx/compute and sdma_v6_0_ring_set_wptr for MES
    // queues: atomic64_set(wptr_cpu_addr) then WDOORBELL64 of the same value.
    // The shadow is 8-byte aligned inside the retained page-aligned buffer.
    // retain_gtt checked cpu_byte_count >= ring_bytes + user_queue_control_bytes,
    // so the slot lies inside the CPU mapping of the leased buffer.
    const uint64_t value = sdma ? wptr << 2 : wptr;
    uint8_t* const shadow = queue.cpu + report.ring_bytes + user_queue_wptr_offset;

    if (!config_.wptr_store.store(config_.wptr_store.context, shadow, value) ||
        !config_.doorbells.write64(config_.doorbells.context, report.doorbell_index, value)) {
        hang(queue, FenceHangReason::submit_failure);
        return finish(UserQueueStatus::hung);
    }

    sequence = seq;
    return UserQueueStatus::submitted;
}

UserQueueStatus UserQueues::query(uint32_t queue_id, uint32_t sequence) noexcept {
    uint32_t index = 0;

    if (!initialized_) {
        return UserQueueStatus::unavailable;
    }

    if (!find_queue(queue_id, index)) {
        return UserQueueStatus::not_found;
    }

    Queue& queue = queues_[index];
    const UserQueueReport& report = queue.report;
    process_fence(queue, now());

    if (report.sync_seq - sequence >= half_range || sequence == 0) {
        return UserQueueStatus::invalid_argument;
    }

    if (report.last_seq - sequence < half_range) {
        return UserQueueStatus::signalled;
    }

    return report.state == UserQueueState::active ? UserQueueStatus::pending :
           report.state == UserQueueState::hung ? UserQueueStatus::hung :
                                                  UserQueueStatus::quarantined;
}

void UserQueues::poll() noexcept {
    if (recovery_detached_ || recovery_saved_) {
        return;
    }

    if (!initialized_) {
        return;
    }

    const uint64_t time = now();
    const bool suspended = config_.mes->report().gangs_suspended;

    for (auto& queue : queues_) {
        UserQueueReport& report = queue.report;
        process_fence(queue, time);

        if (report.state != UserQueueState::active || !report.timeout_armed) {
            continue;
        }

        // Suspended gangs make no progress by design: no timeout runs meanwhile.
        if (suspended) {
            report.timeout_deadline_us = saturating_add(time, uint64_t{report.timeout_ms} * 1000u);
        } else if (time >= report.timeout_deadline_us) {
            // drm_sched_job_timedout -> amdgpu_job_timedout (read once more first).
            process_fence(queue, time);

            if (report.state == UserQueueState::active && report.timeout_armed &&
                time >= report.timeout_deadline_us) {
                hang(queue, FenceHangReason::job_timeout);
            }
        }
    }
}

uint64_t UserQueues::next_deadline_us() const noexcept {
    uint64_t next = UINT64_MAX;

    for (const auto& queue : queues_) {
        if (queue.report.state == UserQueueState::active && queue.report.timeout_armed &&
            queue.report.timeout_deadline_us < next) {
            next = queue.report.timeout_deadline_us;
        }
    }

    return next;
}

UserQueueStatus UserQueues::remove_index(uint32_t index) noexcept {
    Queue& queue = queues_[index];
    UserQueueReport& report = queue.report;
    Mes11& mes = *config_.mes;
    report.remove_status = mes.remove_hw_queue({report.doorbell_index,
                                                gangs_[queue.gang].context_gart});

    if (report.remove_status != Mes11Status::completed) {
        report.state = UserQueueState::quarantined;
        gangs_[queue.gang].quarantined = true;
        processes_[queue.process].quarantined = true;
        return finish(UserQueueStatus::quarantined);
    }

    // MES unmapped the HQD and saved the MQD: no engine reads the queue any more.
    mes.free_doorbell(report.doorbell_index);
    mes.free_queue_id(report.queue_id);
    queue.resources_registered = false;
    GpuVm& vm = vms_[queue.process];

    if (vm.unmap(report.queue_va) != GpuVmStatus::unmapped ||
        (vm.tlb_flush_needed() &&
         vm.flush_tlb(*config_.invalidator, tlb_flush_heavyweight) != GpuVmStatus::flushed)) {
        // The removed queue is gone, but its PTEs may still be cached for the
        // VMID of another queue of this process: keep the buffer.
        report.state = UserQueueState::quarantined;
        processes_[queue.process].quarantined = true;
        return finish(UserQueueStatus::quarantined);
    }

    config_.gtt_manager->unretain(queue.buffer);
    config_.gtt_manager->unretain(queue.mqd);
    queue = {};
    return finish(UserQueueStatus::removed);
}

UserQueueStatus UserQueues::remove_queue(uint32_t queue_id) noexcept {
    uint32_t index = 0;

    if (!initialized_) {
        return UserQueueStatus::unavailable;
    }

    if (recovery_blocked_) {
        return UserQueueStatus::suspended;
    }

    if (!find_queue(queue_id, index)) {
        return finish(UserQueueStatus::not_found);
    }

    Queue& queue = queues_[index];

    if (queue.report.state == UserQueueState::quarantined) {
        return finish(UserQueueStatus::quarantined);
    }

    process_fence(queue, now());

    if (queue.report.state == UserQueueState::active &&
        queue.report.sync_seq != queue.report.last_seq) {
        return finish(UserQueueStatus::busy);
    }

    return remove_index(index);
}

UserQueueStatus UserQueues::remove_gang(uint32_t gang_id) noexcept {
    uint32_t gang = 0;

    if (!initialized_) {
        return UserQueueStatus::unavailable;
    }

    if (recovery_blocked_) {
        return UserQueueStatus::suspended;
    }

    if (!find_gang(gang_id, gang)) {
        return finish(UserQueueStatus::not_found);
    }

    for (const auto& queue : queues_) {
        if (queue.report.state != UserQueueState::free && queue.gang == gang) {
            return finish(gangs_[gang].quarantined ? UserQueueStatus::quarantined :
                                                     UserQueueStatus::busy);
        }
    }

    config_.gtt_manager->unretain(gangs_[gang].context);
    gangs_[gang] = {};
    return finish(UserQueueStatus::removed);
}

UserQueueStatus UserQueues::destroy_process(uint32_t pasid) noexcept {
    uint32_t process = 0;

    if (!initialized_) {
        return UserQueueStatus::unavailable;
    }

    if (recovery_blocked_) {
        return UserQueueStatus::suspended;
    }

    if (!find_process(pasid, process)) {
        return finish(UserQueueStatus::not_found);
    }

    if (processes_[process].quarantined) {
        return finish(UserQueueStatus::quarantined);
    }

    const uint64_t time = now();

    for (auto& queue : queues_) {
        if (queue.report.state != UserQueueState::free && queue.process == process) {
            process_fence(queue, time);

            if (queue.report.state == UserQueueState::active &&
                queue.report.sync_seq != queue.report.last_seq) {
                return finish(UserQueueStatus::busy);
            }
        }
    }

    // amdgpu_mes_destroy_process: REMOVE_QUEUE for every queue of every gang.
    for (uint32_t index = 0; index < user_queue_max_queues; ++index) {
        if (queues_[index].report.state != UserQueueState::free &&
            queues_[index].process == process &&
            remove_index(index) != UserQueueStatus::removed) {
            return finish(UserQueueStatus::quarantined);
        }
    }

    for (auto& gang : gangs_) {
        if (gang.used && gang.process == process) {
            config_.gtt_manager->unretain(gang.context);
            gang = {};
        }
    }

    // Every queue of the PASID is removed: MES released its VMIDs, so no engine
    // walks the tables (amdgpu_vm_fini), then amdgpu_pasid_free.
    Process& owner = processes_[process];
    owner.idle = true;

    if (vms_[process].fini() != GpuVmStatus::released) {
        owner.idle = false;
        owner.quarantined = true;
        return finish(UserQueueStatus::quarantined);
    }

    config_.pasids->release(owner.pasid);
    config_.gtt_manager->unretain(owner.context);
    if (owner.backup_cpu != nullptr) {
        config_.gtt_manager->unretain(owner.backup);
    }
    owner = {};
    return finish(UserQueueStatus::destroyed);
}

UserQueueStatus UserQueues::evict_process_queues(uint32_t pasid) noexcept {
    uint32_t process = 0;

    if (!initialized_) {
        return UserQueueStatus::unavailable;
    }

    if (recovery_blocked_) {
        return UserQueueStatus::suspended;
    }

    if (!find_process(pasid, process)) {
        return finish(UserQueueStatus::not_found);
    }

    const uint64_t time = now();

    // A last fence that did not signal within the caller's bound is not
    // retirement proof; the queue is hung and only MES removal retires it.
    for (auto& queue : queues_) {
        if (queue.report.state == UserQueueState::free || queue.process != process) {
            continue;
        }

        if (queue.report.state == UserQueueState::quarantined) {
            return finish(UserQueueStatus::quarantined);
        }

        process_fence(queue, time);

        if (queue.report.state == UserQueueState::active &&
            queue.report.sync_seq != queue.report.last_seq) {
            hang(queue, FenceHangReason::job_timeout);
        }
    }

    for (uint32_t index = 0; index < user_queue_max_queues; ++index) {
        if (queues_[index].report.state != UserQueueState::free &&
            queues_[index].process == process &&
            remove_index(index) != UserQueueStatus::removed) {
            return finish(UserQueueStatus::quarantined);
        }
    }

    return finish(UserQueueStatus::removed);
}

UserQueueStatus UserQueues::continue_fences(uint32_t queue_id, uint32_t last_seq) noexcept {
    uint32_t index = 0;

    if (!initialized_) {
        return UserQueueStatus::unavailable;
    }

    if (!find_queue(queue_id, index)) {
        return finish(UserQueueStatus::not_found);
    }

    Queue& queue = queues_[index];
    UserQueueReport& report = queue.report;

    if (report.state != UserQueueState::active || report.submissions != 0 ||
        report.sync_seq != 0 || report.last_seq != 0) {
        return finish(UserQueueStatus::invalid_argument);
    }

    // No frame was published, so no engine writes the slot concurrently.
    if (!sdma::CommandBuffer(queue.cpu, report.ring_bytes + user_queue_control_bytes)
             .write_u32(report.ring_bytes + user_queue_fence_offset, last_seq)) {
        return finish(UserQueueStatus::invalid_memory);
    }

    report.sync_seq = last_seq;
    report.last_seq = last_seq;
    return finish(UserQueueStatus::ready);
}

UserQueueStatus UserQueues::release_after_reset(bool (*confirm)(void*) noexcept,
                                                void* context) noexcept {
    if (!initialized_) {
        return UserQueueStatus::unavailable;
    }

    if (confirm == nullptr || !confirm(context)) {
        return finish(UserQueueStatus::quarantined);
    }

    for (auto& queue : queues_) {
        if (queue.report.state == UserQueueState::free) {
            continue;
        }

        // Mes11::release_after_reset may already have forgotten these.
        if (queue.resources_registered) {
            config_.mes->free_doorbell(queue.report.doorbell_index);
            config_.mes->free_queue_id(queue.report.queue_id);
        }
        config_.gtt_manager->unretain(queue.buffer);
        config_.gtt_manager->unretain(queue.mqd);
        queue = {};
    }

    for (auto& gang : gangs_) {
        if (gang.used) {
            config_.gtt_manager->unretain(gang.context);
            gang = {};
        }
    }

    for (uint32_t index = 0; index < user_queue_max_processes; ++index) {
        Process& process = processes_[index];

        if (!process.used) {
            continue;
        }

        process.idle = true;
        vms_[index].fini();
        config_.pasids->release(process.pasid);
        config_.gtt_manager->unretain(process.context);
        if (process.backup_cpu != nullptr) {
            config_.gtt_manager->unretain(process.backup);
        }
        process = {};
    }

    recovery_blocked_ = false;
    recovery_saved_ = false;
    recovery_detached_ = false;
    return finish(UserQueueStatus::released);
}

UserQueueStatus UserQueues::quarantine_recovery() noexcept {
    recovery_blocked_ = true;
    for (auto& process : processes_) {
        if (process.used) {
            process.quarantined = true;
        }
    }
    for (auto& queue : queues_) {
        if (queue.report.state != UserQueueState::free) {
            queue.report.state = UserQueueState::quarantined;
        }
    }
    return finish(UserQueueStatus::quarantined);
}

UserQueueStatus UserQueues::cancel_recovery() noexcept {
    if (!initialized_) {
        return UserQueueStatus::unavailable;
    }
    if (recovery_detached_ || recovery_saved_ || report().reset_required) {
        return finish(UserQueueStatus::quarantined);
    }
    recovery_blocked_ = false;
    return finish(UserQueueStatus::ready);
}

UserQueueStatus UserQueues::suspend_for_recovery() noexcept {
    if (!initialized_) {
        return UserQueueStatus::unavailable;
    }
    recovery_blocked_ = true;
    if (recovery_saved_) {
        return finish(UserQueueStatus::suspended);
    }
    if (recovery_detached_ || report().reset_required) {
        return quarantine_recovery();
    }

    // Validate every backup and prove every fence before the first REMOVE.
    // No inference of idle from a timeout, lost interrupt or failed doorbell.
    for (const auto& process : processes_) {
        if (process.used && (process.backup_cpu == nullptr || process.quarantined)) {
            return finish(UserQueueStatus::invalid_memory);
        }
    }
    const uint64_t time = now();
    for (auto& queue : queues_) {
        if (queue.report.state == UserQueueState::free) {
            continue;
        }
        process_fence(queue, time);
        if (queue.report.state != UserQueueState::active) {
            return quarantine_recovery();
        }
        if (queue.report.sync_seq != queue.report.last_seq) {
            return finish(UserQueueStatus::busy);
        }
    }

    recovery_detached_ = true;
    for (auto& queue : queues_) {
        auto& report = queue.report;
        if (report.state == UserQueueState::free) {
            continue;
        }
        report.remove_status = config_.mes->remove_hw_queue(
            {report.doorbell_index, gangs_[queue.gang].context_gart});
        if (report.remove_status != Mes11Status::completed) {
            return quarantine_recovery();
        }
        queue.detached = true;
        if (config_.mes->free_doorbell(report.doorbell_index) != Mes11Status::completed ||
            config_.mes->free_queue_id(report.queue_id) != Mes11Status::completed) {
            return quarantine_recovery();
        }
        queue.resources_registered = false;
    }

    for (uint32_t index = 0; index < user_queue_max_processes; ++index) {
        auto& process = processes_[index];
        if (!process.used) {
            continue;
        }
        process.idle = true;
        if (vms_[index].save_tables(sdma::CommandBuffer(process.backup_cpu,
                static_cast<size_t>(process.backup_bytes))) != GpuVmStatus::ready) {
            return quarantine_recovery();
        }
    }
    recovery_saved_ = true;
    return finish(UserQueueStatus::suspended);
}

bool UserQueues::reattach_queue(Queue& queue) noexcept {
    auto& report = queue.report;
    Mes11& mes = *config_.mes;
    if (!queue.detached ||
        mes.reserve_queue_id(report.queue_id) != Mes11Status::completed) {
        return false;
    }
    if (mes.allocate_doorbell(report.type, report.doorbell_index) != Mes11Status::completed) {
        mes.free_queue_id(report.queue_id);
        return false;
    }
    queue.resources_registered = true;
    memory::AllocationInfo mqd{};
    const uint64_t control_va = report.queue_va + report.ring_bytes;
    if (config_.gtt_manager->describe(queue.mqd, mqd) != memory::Status::success ||
        !prepare_buffer(queue) || !zero(mqd.backing.cpu_bytes, mqd_bytes(report.type))) {
        mes.free_doorbell(report.doorbell_index);
        mes.free_queue_id(report.queue_id);
        queue.resources_registered = false;
        return false;
    }
    // Preserve the monotonically increasing public fence sequence across reset.
    // All older work was proven retired before detach; no pending IB is replayed.
    if (!sdma::CommandBuffer(queue.cpu, report.ring_bytes + user_queue_control_bytes)
             .write_u32(report.ring_bytes + user_queue_fence_offset, report.last_seq)) {
        return false;
    }
    const Gfx11MqdProperties properties{report.mqd_gart, report.queue_va,
        control_va + user_queue_rptr_offset, control_va + user_queue_wptr_offset,
        report.type == Mes11QueueType::compute ? control_va + user_queue_eop_offset : 0,
        report.ring_bytes, report.doorbell_index, pipe_priority_normal,
        queue_priority_minimum, false, false};
    if (mes.init_queue_mqd(report.type, properties,
            sdma::CommandBuffer(mqd.backing.cpu_bytes, mqd_bytes(report.type))) !=
        Mes11Status::completed) {
        return false;
    }
    const Gang& gang = gangs_[queue.gang];
    report.add_status = mes.add_hw_queue({report.type, report.pasid,
        vms_[queue.process].page_table_base_addr(), user_queue_process_va_end, 0,
        processes_[queue.process].context_gart, gang.quantum, gang.context_gart,
        gang.inprocess_gang_priority, gang.global_priority_level, report.doorbell_index,
        report.mqd_gart, control_va + user_queue_wptr_offset,
        report.buffer_gart + report.ring_bytes + user_queue_wptr_offset});
    if (report.add_status != Mes11Status::completed) {
        return false;
    }
    queue.detached = false;
    report.wptr = 0;
    report.retired_position = 0;
    report.timeout_armed = false;
    report.timeout_deadline_us = 0;
    for (auto& position : queue.end_position) {
        position = 0;
    }
    return true;
}

UserQueueStatus UserQueues::resume_after_recovery() noexcept {
    if (!initialized_ || !recovery_saved_ || !recovery_detached_ ||
        !config_.mes->report().scheduler_ready) {
        return UserQueueStatus::unavailable;
    }
    if (report().reset_required) {
        return quarantine_recovery();
    }
    for (uint32_t index = 0; index < user_queue_max_processes; ++index) {
        auto& process = processes_[index];
        if (!process.used) {
            continue;
        }
        memory::AllocationInfo context{};
        if (config_.gtt_manager->describe(process.context, context) != memory::Status::success ||
            !zero(context.backing.cpu_bytes, mes11_process_context_bytes) ||
            vms_[index].restore_tables(ByteView(process.backup_cpu,
                static_cast<size_t>(process.backup_bytes))) != GpuVmStatus::ready ||
            vms_[index].flush_tlb(*config_.invalidator, tlb_flush_heavyweight) !=
                GpuVmStatus::flushed) {
            return quarantine_recovery();
        }
    }
    for (const auto& gang : gangs_) {
        memory::AllocationInfo context{};
        if (gang.used &&
            (config_.gtt_manager->describe(gang.context, context) != memory::Status::success ||
             !zero(context.backing.cpu_bytes, mes11_gang_context_bytes))) {
            return quarantine_recovery();
        }
    }
    for (auto& queue : queues_) {
        if (queue.report.state != UserQueueState::free && !reattach_queue(queue)) {
            return quarantine_recovery();
        }
        if (queue.report.state != UserQueueState::free) {
            processes_[queue.process].idle = false;
        }
    }
    recovery_saved_ = false;
    recovery_detached_ = false;
    recovery_blocked_ = false;
    return finish(UserQueueStatus::ready);
}

UserQueueReport UserQueues::queue_report(uint32_t queue_id) const noexcept {
    uint32_t index = 0;
    return find_queue(queue_id, index) ? queues_[index].report : UserQueueReport{};
}

UserQueuesReport UserQueues::report() const noexcept {
    UserQueuesReport result{};
    result.status = status_;

    for (const auto& process : processes_) {
        result.processes += process.used ? 1 : 0;
        result.reset_required = result.reset_required || (process.used && process.quarantined);
    }

    for (const auto& gang : gangs_) {
        result.gangs += gang.used ? 1 : 0;
    }

    for (const auto& queue : queues_) {
        result.queues += queue.report.state != UserQueueState::free ? 1 : 0;
        result.hung_queues += queue.report.state == UserQueueState::hung ? 1 : 0;
        result.reset_required = result.reset_required ||
                                queue.report.state == UserQueueState::quarantined;
    }

    return result;
}

} // namespace rx7900::hardware

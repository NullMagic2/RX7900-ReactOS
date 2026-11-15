/*
Module: rx7900.hardware.user_queue
Purpose: Own MES-managed process queues: processes (PASID, GPUVM, process context), gangs, gfx/compute/SDMA queues, submission and completion.
Created: 2026-10-22
Architecture: Mes11 for MESAPI ADD_QUEUE/REMOVE_QUEUE, the MES doorbell page and
queue ids; GpuVm/PasidAllocator/GpuVmInvalidator for the process address space;
the shared gfx11_queues MQD and IB-frame builders and the Mes11 SDMA builders for
queue contents; the caller's GTT Manager for contexts, MQDs and queue buffers.
Completion is observed on each queue's own fence slot with FenceManager's job
timeout semantics (fence.hpp constants and FenceHangReason).
Copyright 2019 Advanced Micro Devices, Inc.; Copyright 2009 Jerome Glisse;
Copyright 2015 Advanced Micro Devices, Inc.
Adapted Linux v6.12 amdgpu_mes.c (amdgpu_mes_create_process, destroy_process,
add_gang, remove_gang, add_hw_queue, remove_hw_queue, queue_alloc_mqd,
queue_init_mqd, add_ring, remove_ring, ring_to_queue_props, kernel doorbell
get/free, ctx_map_meta_data), amdgpu_mes.h, amdgpu_mes_ctx.h, mes_v11_0.c
(add/remove_hw_queue), amdgpu_ring.c (ring_init, ring_alloc, commit),
amdgpu_ib.c (amdgpu_ib_schedule), amdgpu_fence.c (emit, process,
driver_init_ring), gfx_v11_0.c (ring_set_wptr_gfx/compute, eop_irq MES path),
sdma_v6_0.c (ring_set_wptr, mqd_init), amdgpu_vm.c (amdgpu_vm_init, bo_map,
fini), amdgpu_ids.c (amdgpu_pasid_alloc/free) and drm sched_main.c
(drm_sched_job_begin/start_timeout semantics for the queue fence timeout).
Notice and provenance: licenses/AMDGPU-FIRMWARE.txt.
*/
#pragma once

#include "fence.hpp"
#include "gfx11_queues.hpp"
#include "gpuvm.hpp"

namespace rx7900::hardware {

// Fixed device-wide capacities (Linux idr/lists are unbounded).
constexpr uint32_t user_queue_max_processes = 4;
constexpr uint32_t user_queue_max_gangs = 8;
constexpr uint32_t user_queue_max_queues = 16;

// One queue buffer (one GTT allocation, mapped into the process VM at queue_va,
// replacing the per-ring part of struct amdgpu_mes_ctx_meta_data): the ring of
// ring_bytes, then one control page holding the amdgpu_mes_ctx slots (32-byte
// amdgpu_wb_slot each, AMDGPU_MES_CTX_RPTR/WPTR/FENCE/COND_EXE/TRAIL_FENCE order)
// and, for compute, the GFX11_MEC_HPD_SIZE EOP buffer at byte 2048.
constexpr uint32_t user_queue_control_bytes = 4096;
constexpr uint32_t user_queue_rptr_offset = mes11_ctx::slot_rptr * mes11_ctx::slot_bytes;
constexpr uint32_t user_queue_wptr_offset = mes11_ctx::slot_wptr * mes11_ctx::slot_bytes;
constexpr uint32_t user_queue_fence_offset = mes11_ctx::slot_fence * mes11_ctx::slot_bytes;
constexpr uint32_t user_queue_cond_exec_offset = mes11_ctx::slot_cond_exe * mes11_ctx::slot_bytes;
constexpr uint32_t user_queue_eop_offset = 2048;
// amdgpu_ring_init(max_dw 1024) with amdgpu_sched_hw_submission 2 (the size every
// amdgpu_mes_add_ring ring gets) is the minimum; larger powers of two up to 1 MiB.
constexpr uint32_t user_queue_min_ring_bytes = mes11_ctx::ring_bytes;
constexpr uint32_t user_queue_max_ring_bytes = 1u << 20;
// amdgpu_fence_driver_init_ring with num_hw_submission 2: four fences may be
// outstanding per queue (num_fences_mask 3), as for kernel rings.
constexpr uint32_t user_queue_fence_slots = fence_slots;
// amdgpu_vm_adjust_size: ADD_QUEUE process_va_end = (max_pfn - 1) << 12.
constexpr uint64_t user_queue_process_va_end = (gpuvm_max_pfn - 1) << 12;

enum class UserQueueStatus {
    ready,
    created,
    added,
    submitted,
    signalled,
    pending,
    removed,
    destroyed,
    released,
    // Outstanding work, live children or a full fence window; nothing changed.
    busy,
    ring_full,
    // MES gangs are suspended (Mes11::suspend_all); no submission until resume.
    suspended,
    // The queue was quarantined by a job timeout, a fence outside the window or
    // a failed WPTR/doorbell store; only remove_queue or a reset follows.
    hung,
    not_found,
    exhausted,
    unavailable,
    invalid_argument,
    invalid_memory,
    vm_failure,
    mes_failure,
    // Uncertain hardware state: leases kept until release_after_reset.
    quarantined
};

enum class UserQueueState : uint32_t { free, active, hung, quarantined };

// Monotonic microseconds at PASSIVE_LEVEL (native: KeQueryInterruptTime / 10).
struct UserQueueClock final {
    void* context;
    uint64_t (*now_us)(void* context) noexcept;
};

// Device services, all outliving UserQueues. mes is the scheduler-ready Mes11
// (process queues need scheduler firmware >= 0x47 like the kernel queues);
// pasids the device PASID allocator; invalidator the MES-path TLB invalidator
// (AsicInit::vm_invalidator); vram_manager holds page-table pools (local VRAM),
// gtt_manager the CPU-visible GTT buffers inside gart. nbio_registers gives the
// HDP flush REQ/DONE registers the ring frames poll. wptr_store stores the 64-bit
// WPTR shadow atomically (Linux atomic64_set).
struct UserQueueConfig final {
    Mes11* mes;
    PasidAllocator* pasids;
    GpuVmInvalidator* invalidator;
    Nbio* nbio;
    memory::Manager* vram_manager;
    memory::Manager* gtt_manager;
    VramLocation vram;
    GartLayout gart;
    NbioRegisters nbio_registers;
    DoorbellIo doorbells;
    RingPointerStore wptr_store;
    UserQueueClock clock;
};

// create_process inputs: page_tables is a vram_manager allocation of table_count
// * gpuvm_table_bytes (GpuVmConfig), process_context a gtt_manager allocation of
// at least mes11_process_context_bytes (AMDGPU_MES_PROC_CTX_SIZE).
struct UserProcessMemory final {
    memory::Handle page_tables;
    uint32_t table_count;
    memory::Handle process_context;
    // Optional for legacy callers; live suspend requires a distinct pinned GTT
    // allocation of table_count * gpuvm_table_bytes, private to this process.
    memory::Handle table_backup{};
};

// amdgpu_mes_gang_properties as the add_gang caller passes them; priorities are
// AMDGPU_MES_PRIORITY_LEVEL_* (0..4; mes11_priority_normal is what the self-test
// uses). gang_quantum 0 selects default_gang_quantum, which v6.12 never sets (0).
// gang_context: a gtt_manager allocation of at least mes11_gang_context_bytes.
struct UserGangProperties final {
    uint32_t global_priority_level;
    uint32_t inprocess_gang_priority;
    uint64_t gang_quantum;
    memory::Handle gang_context;
};

// One MES queue. buffer: a gtt_manager allocation of ring_bytes +
// user_queue_control_bytes whose pinned pages are buffer_dma_pages (one LE
// 64-bit DMA bus address per 4 KiB page, caller-owned, read during the call);
// it is mapped read/write/executable, snooped, MTYPE NC at queue_va (page
// aligned, free in the process VM) as amdgpu_mes_ctx_map_meta_data maps the
// metadata. mqd: gtt_manager allocation of the engine's MQD size. timeout_ms:
// 0 selects the engine's Linux job timeout (fence_gfx/compute/sdma_timeout_ms).
struct UserQueueProperties final {
    Mes11QueueType type;
    uint32_t ring_bytes;
    uint64_t queue_va;
    memory::Handle buffer;
    ByteView buffer_dma_pages;
    memory::Handle mqd;
    uint32_t timeout_ms;
};

// One indirect buffer at a process VA of the queue's VM, mapped readable for the
// whole range and kept mapped by the caller until its fence signalled or the
// queue was removed. gfx/compute: 4-byte aligned, 1..gfx11_max_ib_dwords; SDMA:
// 32-byte aligned, a nonzero multiple of 8 dwords (amdgpu_ring_pad_ib).
struct UserQueueIb final {
    uint64_t va;
    uint32_t dwords;
};

struct UserQueueReport final {
    UserQueueState state = UserQueueState::free;
    Mes11QueueType type = Mes11QueueType::gfx;
    uint32_t pasid = 0;
    uint32_t gang_id = 0;
    uint32_t queue_id = 0;
    uint32_t doorbell_index = 0;
    uint32_t ring_bytes = 0;
    uint64_t queue_va = 0;
    uint64_t buffer_gart = 0;
    uint64_t mqd_gart = 0;
    // Driver write pointer in dwords (SDMA publishes wptr << 2).
    uint64_t wptr = 0;
    // fence_drv.sync_seq / last_seq and the ring position all signalled frames end at.
    uint32_t sync_seq = 0;
    uint32_t last_seq = 0;
    uint64_t retired_position = 0;
    uint32_t timeout_ms = 0;
    bool timeout_armed = false;
    uint64_t timeout_deadline_us = 0;
    FenceHangReason hang_reason = FenceHangReason::none;
    uint32_t hang_signalled_seq = 0;
    uint32_t hang_emitted_seq = 0;
    uint64_t submissions = 0;
    Mes11Status add_status = Mes11Status::unavailable;
    Mes11Status remove_status = Mes11Status::unavailable;
};

struct UserQueuesReport final {
    UserQueueStatus status = UserQueueStatus::unavailable;
    uint32_t processes = 0;
    uint32_t gangs = 0;
    uint32_t queues = 0;
    uint32_t hung_queues = 0;
    // A queue or process could not be torn down with certainty.
    bool reset_required = false;
};

// Fence progress hint for the IrqService drain consumer (DPC, DISPATCH_LEVEL):
// gfx_v11_0_eop_irq and sdma_v6_0_process_trap_irq route MES-queue fences by
// AMDGPU_FENCE_MES_QUEUE_FLAG to amdgpu_fence_process(queue ring). IrqService
// keeps only the last MES queue id, so any new mes_eop/sdma_mes_trap record (or
// lost vectors) means: run UserQueues::poll at PASSIVE_LEVEL. Only the DPC
// touches this object.
class UserQueueInterruptHint final {
public:
    UserQueueInterruptHint() noexcept = default;
    bool observe(const IrqDispatchView& view) noexcept;

private:
    uint32_t mes_eop_seen_ = 0;
    uint32_t sdma_mes_seen_ = 0;
};

// Device-owned, constructed once in nonpaged storage; trivially destructible
// after release. Serialized PASSIVE_LEVEL use by the exclusive device owner, on
// the same thread as Mes11, GpuVm and the GTT Manager (amdgpu_mes_lock and the
// srbm mutex in Linux). The MES must stay in its scheduler phase while queues
// exist; across a MES halt (suspend, kiq_hw_fini) every queue is removed first
// (Mes11::release refuses while queues are registered).
class UserQueues final {
public:
    UserQueues() noexcept = default;
    UserQueues(const UserQueues&) = delete;
    UserQueues& operator=(const UserQueues&) = delete;

    // Validates the services and the HDP flush registers. No hardware access.
    UserQueueStatus initialize(const UserQueueConfig& config) noexcept;

    // amdgpu_pasid_alloc(16), amdgpu_vm_init (root cleared and published),
    // amdgpu_vm_set_pasid, then amdgpu_mes_create_process: zeroed process
    // context, process_quantum = default_process_quantum (0). v6.12 MES11 has no
    // add-process frame: the process travels in every ADD_QUEUE. Full rollback
    // on failure.
    UserQueueStatus create_process(const UserProcessMemory& memory, uint32_t& pasid) noexcept;

    // The process VM for IBs and other caller buffers (amdgpu_vm_bo_map +
    // bo_update); see GpuVm. unmap needs flush_tlb (heavyweight, type 2 as KFD)
    // before the memory may be reused while queues of the process exist.
    GpuVmStatus map_system(uint32_t pasid, uint64_t va, ByteView dma_pages, bool cpu_cached,
                           GpuVmMapFlags flags) noexcept;
    GpuVmStatus map_vram(uint32_t pasid, uint64_t va, uint64_t vram_mc, uint64_t bytes,
                         GpuVmMapFlags flags) noexcept;
    GpuVmStatus unmap(uint32_t pasid, uint64_t va) noexcept;
    GpuVmStatus flush_tlb(uint32_t pasid, uint32_t flush_type) noexcept;
    GpuVmReport vm_report(uint32_t pasid) const noexcept;

    // amdgpu_mes_add_gang: zeroed gang context, lowest free gang id from 1. No frame.
    UserQueueStatus add_gang(uint32_t pasid, const UserGangProperties& properties,
                             uint32_t& gang_id) noexcept;

    // amdgpu_mes_add_ring + amdgpu_mes_add_hw_queue: map the queue buffer, ring
    // cleared with the engine NOP, cond_exec 1, fence 0 (amdgpu_ring_init), then
    // queue id (queue_id_idr), MES doorbell (amdgpu_mes_kernel_doorbell_get),
    // zeroed MQD, the engine mqd_init (Mes11::init_queue_mqd with
    // amdgpu_mes_queue_init_mqd's properties) and ADD_QUEUE. Before the frame
    // every failure rolls back; an ADD_QUEUE that did not complete leaves the
    // queue quarantined (MES may own it) with its leases until reset; queue_id is
    // then still set so queue_report can show it.
    UserQueueStatus add_queue(uint32_t gang_id, const UserQueueProperties& properties,
                              uint32_t& queue_id) noexcept;

    // amdgpu_ib_schedule(ring, 1, &ib, NULL, &f) on the queue: fence window and
    // ring space checks (amdgpu_ring_alloc 256 dwords gfx/compute,
    // sdma6_submit_alloc_dwords SDMA), the frame (gfx11_emit_ib_frame with
    // mem_sync and the MES variants; mes11_sdma_emit_ib_frame), commit padding,
    // then set_wptr: the 64-bit shadow (dwords, SDMA bytes) and WDOORBELL64 of the
    // same value, without an HDP flush (snooped GTT, Linux mb()). drm_sched_job_begin
    // re-arms the queue timeout. sequence is the fence value to query.
    UserQueueStatus submit(uint32_t queue_id, const UserQueueIb& ib, uint32_t& sequence) noexcept;

    // amdgpu_fence_process for one queue, then signalled, pending, hung (not
    // signalled on a hung queue), not_found or invalid_argument (never emitted).
    UserQueueStatus query(uint32_t queue_id, uint32_t sequence) noexcept;

    // Fence processing and job timeout for every active queue (Linux EOP/trap
    // interrupt plus fallback timer plus drm_sched TDR): a due timeout re-reads
    // the fence once, then quarantines the queue (amdgpu_job_timedout without
    // soft recovery, queue reset or GPU recovery). While MES gangs are suspended
    // the timeouts are re-armed instead.
    void poll() noexcept;
    // Earliest armed deadline or UINT64_MAX.
    uint64_t next_deadline_us() const noexcept;

    // amdgpu_mes_remove_ring -> amdgpu_mes_remove_hw_queue: REMOVE_QUEUE with
    // the doorbell and gang context, queue id and doorbell freed, the buffer
    // unmapped with a heavyweight TLB flush, MQD and buffer leases returned.
    // busy while fences are outstanding on a healthy queue (wait first); a hung
    // queue may be removed. A REMOVE_QUEUE that did not complete, or a failed
    // unmap/flush, quarantines the queue with its leases.
    UserQueueStatus remove_queue(uint32_t queue_id) noexcept;

    // amdgpu_mes_remove_gang: busy while it has queues (Linux -EBUSY).
    UserQueueStatus remove_gang(uint32_t gang_id) noexcept;

    // amdgpu_mes_destroy_process: removes every remaining queue (each as
    // remove_queue; busy before anything when a healthy queue has outstanding
    // fences) and gang, then amdgpu_vm_fini and amdgpu_pasid_free. quarantined
    // when any queue could not be removed: VM and contexts stay until reset.
    UserQueueStatus destroy_process(uint32_t pasid) noexcept;

    // Process-exit eviction after a bounded idle wait, as Linux amdgpu_userq
    // tears a queue down when its owner goes away (amdgpu_userq_cleanup:
    // amdgpu_userq_wait_for_last_fence, whose timeout marks the queue
    // AMDGPU_USERQ_STATE_HUNG, then amdgpu_userq_unmap_helper -> MES
    // remove_hw_queue). Every healthy queue of pasid that still has outstanding
    // fences is marked hung (job_timeout); then every queue of the PASID is
    // removed exactly as remove_queue removes a hung queue (REMOVE_QUEUE
    // preempts and unmaps the HQD, doorbell and queue id freed, queue VA
    // unmapped with a heavyweight TLB flush, leases returned). The process, its
    // gangs and its VM remain for the caller's unmaps and destroy_process.
    // removed: no queue of the PASID remains. quarantined: a REMOVE_QUEUE did not
    // complete (MES may still own the queue) or an unmap/flush failed; that
    // queue keeps its leases until a confirmed reset (Linux then schedules
    // amdgpu_device_gpu_recover from amdgpu_userq_unmap_helper). Also
    // unavailable, suspended or not_found, with nothing changed.
    UserQueueStatus evict_process_queues(uint32_t pasid) noexcept;

    // Reconstruction after a confirmed reset: a queue just created by add_queue
    // (nothing submitted, sync_seq == last_seq == 0) continues the fence values
    // a client observed before the reset instead of restarting at 1, as
    // reattach_queue preserves them across suspend: the writeback slot, the last
    // signalled and the last emitted sequence become last_seq, so the next
    // submission is last_seq + 1 and no earlier sequence aliases new work.
    // ready, or invalid_argument once anything was submitted, not_found.
    UserQueueStatus continue_fences(uint32_t queue_id, uint32_t last_seq) noexcept;

    // After a confirmed reset (the caller proves MES/CP/SDMA no longer access any
    // queue, context or page table): returns every lease, finishes every VM,
    // frees PASIDs and forgets all state. Call before Mes11::release_after_reset.
    UserQueueStatus release_after_reset(bool (*confirm_reset_quiescence)(void*) noexcept,
                                        void* context) noexcept;

    // Exclusive PASSIVE_LEVEL power owner. First blocks every mutating API,
    // proves all emitted fences retired, removes MES queues while preserving
    // process identities/mappings/leases and saves local-VRAM tables in GTT.
    // busy keeps admission blocked: poll and retry, or cancel before detaching.
    UserQueueStatus suspend_for_recovery() noexcept;
    UserQueueStatus cancel_recovery() noexcept;
    // Only after the ASIC owner completed resume/reset and restored GART and
    // client VRAM buffers. Restores tables, invalidates PASIDs, rebuilds contexts
    // and MQDs, reserves stable queue ids and reattaches queues. Any uncertainty
    // quarantines all ownership, never claiming an old fence retired.
    UserQueueStatus resume_after_recovery() noexcept;
    bool recovery_suspended() const noexcept { return recovery_saved_; }

    UserQueueReport queue_report(uint32_t queue_id) const noexcept;
    UserQueuesReport report() const noexcept;

private:
    struct Process final {
        bool used;
        bool idle;
        bool quarantined;
        uint32_t pasid;
        memory::Handle context;
        uint64_t context_gart;
        memory::Handle backup;
        uint8_t* backup_cpu;
        uint64_t backup_gart;
        uint64_t backup_bytes;
    };

    struct Gang final {
        bool used;
        bool quarantined;
        uint32_t gang_id;
        uint32_t process;
        memory::Handle context;
        uint64_t context_gart;
        uint64_t quantum;
        uint32_t global_priority_level;
        uint32_t inprocess_gang_priority;
    };

    struct Queue final {
        uint32_t gang;
        uint32_t process;
        memory::Handle buffer;
        memory::Handle mqd;
        uint8_t* cpu;
        uint64_t end_position[user_queue_fence_slots];
        bool mapped;
        bool detached;
        bool resources_registered;
        UserQueueReport report;
    };

    static bool confirm_process_idle(void* context) noexcept;
    bool find_process(uint32_t pasid, uint32_t& index) const noexcept;
    bool find_gang(uint32_t gang_id, uint32_t& index) const noexcept;
    bool find_queue(uint32_t queue_id, uint32_t& index) const noexcept;
    bool address_in_use(uint64_t gart) const noexcept;
    bool retain_gtt(memory::Handle handle, uint64_t bytes, memory::AllocationInfo& info) noexcept;
    bool prepare_buffer(Queue& queue) noexcept;
    UserQueueStatus rollback_queue(uint32_t index, bool free_ids, UserQueueStatus status) noexcept;
    void process_fence(Queue& queue, uint64_t now) noexcept;
    void hang(Queue& queue, FenceHangReason reason) noexcept;
    bool ib_mapped(uint32_t index, const UserQueueIb& ib) const noexcept;
    UserQueueStatus emit(Queue& queue, const UserQueueIb& ib, uint32_t sequence,
                         uint64_t& wptr) noexcept;
    UserQueueStatus remove_index(uint32_t index) noexcept;
    UserQueueStatus quarantine_recovery() noexcept;
    bool reattach_queue(Queue& queue) noexcept;
    uint64_t now() const noexcept { return config_.clock.now_us(config_.clock.context); }
    UserQueueStatus finish(UserQueueStatus status) noexcept;

    UserQueueConfig config_{};
    bool initialized_ = false;
    bool recovery_blocked_ = false;
    bool recovery_saved_ = false;
    bool recovery_detached_ = false;
    UserQueueStatus status_ = UserQueueStatus::unavailable;
    uint32_t hdp_flush_req_ = 0;
    uint32_t hdp_flush_done_ = 0;
    Process processes_[user_queue_max_processes]{};
    GpuVm vms_[user_queue_max_processes]{};
    Gang gangs_[user_queue_max_gangs]{};
    Queue queues_[user_queue_max_queues]{};
};

} // namespace rx7900::hardware

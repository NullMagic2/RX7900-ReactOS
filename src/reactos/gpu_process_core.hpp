/*
Module: rx7900.reactos.gpu_process_core
Purpose: Portable native-process session state machine: request execution, exit
reaping and destructive-reset retirement/reconstruction of GPU process sessions.
Created: 2026-10-24
Architecture: Split out of gpu_process.cpp so host fault-injection tests run the
production logic. Owns UserQueues, PasidAllocator and the per-session
ProcessMemory/ProcessSubmissions/ProcessGraphics; reaches the ASIC only through
ProcessPlatform. gpu_process.cpp owns every NT object (mutexes, the exit
notification, the reaper thread, process references) and calls these functions
with the service mutex held, at PASSIVE_LEVEL. No allocation, no OS calls.
Linux references: v6.12 amdgpu_mes.c (remove_hw_queue), mes_v11_0.c,
amdgpu_device.c (amdgpu_device_check_vram_lost, amdgpu_do_asic_reset),
amdgpu_vm.c (amdgpu_vm_fini, amdgpu_vm_generation), amdgpu_ids.c (pasid free),
all pinned in work/upstream-gpuinit; amdgpu_userq.c (amdgpu_userq_cleanup,
amdgpu_userq_wait_for_last_fence, amdgpu_userq_unmap_helper,
amdgpu_userq_pre_reset/post_reset) from later kernels, cited from memory and
not pinned. Notice and provenance: licenses/AMDGPU-FIRMWARE.txt.
*/
#pragma once

#include "gpu_process_abi.hpp"
#include "process_memory.hpp"
#include "process_submission.hpp"
#include "process_graphics.hpp"

namespace rx7900::reactos {

constexpr uint32_t process_service_tables = 64;
constexpr uint32_t process_service_queues = hardware::user_queue_max_queues;
// Bounded idle wait of an exited session's last fences before MES preemption
// (amdgpu_userq_wait_for_last_fence uses a bounded dma_fence wait, after which
// the queue is hung and unmapped). Recent kernels use 100 ms; not pinned here.
constexpr uint64_t process_reap_idle_timeout_us = 100000;
// Reaper polling period while an exited session still waits for fences.
constexpr uint64_t process_reap_retry_us = 10000;
// Reconstruction attempts per session after one confirmed reset; a failure
// destroys that attempt's VM before the next. Then the session is poisoned.
constexpr uint32_t process_rebuild_attempts = 2;
// Lost fence values remembered per queue (one transfer plus the graphics jobs
// of a reset, twice over); older lost values are covered by lost_floor.
constexpr uint32_t process_lost_sequences = 2 * (1 + graphics_jobs);
// Session::graphics_queue value of a graphics context on the kernel gfx ring
// (graphics_open with Request.handle 0); never a queues[] index.
constexpr uint32_t process_graphics_kernel_ring = UINT32_MAX - 1;

// The adopted pre-OS scanout that kernel presents may write (linear 32 bpp),
// recorded once before any graphics session opens; nullptr leaves present off.
struct GraphicsScanout final {
    uint64_t mc_address;
    uint32_t width;
    uint32_t height;
    uint32_t pitch_bytes;
};

// ASIC services of the current ASIC incarnation; every pointer outlives the core.
// dma_bus resolves only a gtt allocation to its actual contiguous DMA bus
// extent. queue_config fills a UserQueueConfig for the running scheduler (false
// when no ready ASIC exists). now_us is monotonic PASSIVE_LEVEL time.
struct ProcessPlatform final {
    void* context = nullptr;
    memory::Manager* gtt = nullptr;
    memory::Manager* vram = nullptr;
    bool (*dma_bus)(void*, memory::Handle, uint64_t&) noexcept = nullptr;
    bool (*queue_config)(void*, hardware::PasidAllocator&, hardware::UserQueueConfig&) noexcept = nullptr;
    uint64_t (*now_us)(void*) noexcept = nullptr;
    // Reference on the opaque process token while a session names it (native:
    // ObReferenceObject at open, ObDereferenceObject exactly once at retirement).
    void (*reference)(void*, void* process) noexcept = nullptr;
    void (*dereference)(void*, void* process) noexcept = nullptr;
    // The kernel gfx-ring submitter for graphics contexts without a MES queue
    // (native: KernelGraphicsBridge over AsicInit's Gfx11Queues and the
    // FenceManager); submit null leaves only the MES-queue route.
    opengl::GraphicsSubmitter kernel_ring{};
};

// A lock the native owner holds around core calls (KMUTEX in production).
struct ProcessLock final {
    void* context = nullptr;
    void (*acquire)(void*) noexcept = nullptr;
    void (*release)(void*) noexcept = nullptr;
};

// The device owner's destructive-recovery request (video.cpp: wake the recovery
// worker). Called only with no process lock held; must not block.
struct ProcessRecoveryHook final {
    void* context = nullptr;
    void (*request)(void*) noexcept = nullptr;
};

struct ProcessCoreReport final {
    // Confirmed destructive resets retired by this service (vram_lost_counter).
    uint64_t reset_generation = 0;
    uint32_t reaped = 0;
    // Exited sessions whose last fences missed the bound and were preempted.
    uint32_t reap_preempted = 0;
    uint32_t reap_quarantined = 0;
    uint32_t escalations = 0;
    uint32_t rebuilt = 0;
    uint32_t rebuild_retries = 0;
    uint32_t poisoned = 0;
    uint32_t lost_buffers = 0;
    uint32_t lost_work = 0;
};

// Device-owned, constructed once (trivially destructible after shutdown).
// Externally serialized: the native owner holds its service mutex around every
// function below. Process tokens are compared only, never dereferenced.
struct GpuProcessCore {
    struct Queue final {
        uint64_t handle = 0;
        uint32_t id = 0;
        memory::Handle ring{};
        memory::Handle mqd{};
        hardware::Mes11QueueType type = hardware::Mes11QueueType::sdma;
        bool quarantined = false;
        // Last fence value emitted before the latest confirmed reset; the
        // rebuilt queue continues after it (UserQueues::continue_fences).
        uint32_t fence_floor = 0;
        // Fence values in flight at a confirmed reset (work_lost); values at or
        // below lost_floor that fell out of this list are answered lost too.
        uint32_t lost[process_lost_sequences]{};
        uint32_t lost_floor = 0;
    };
    struct Session final {
        void* process = nullptr;
        uint64_t handle = 0;
        uint32_t pasid = 0;
        uint32_t gang = 0;
        memory::Handle tables{};
        memory::Handle table_backup{};
        memory::Handle context{};
        memory::Handle gang_context{};
        ProcessMemory buffers{};
        ProcessSubmissions submissions{};
        // Kernel-built graphics work (version 3); bound to queues[graphics_queue],
        // or to the kernel gfx ring (process_graphics_kernel_ring).
        ProcessGraphics graphics{};
        uint32_t graphics_queue = UINT32_MAX;
        Queue queues[process_service_queues]{};
        // Lost-work record of the kernel gfx-ring context (only lost and
        // lost_floor are used; its numbers are the session-local sequences).
        Queue kernel_graphics{};
        bool quarantined = false;
        bool exiting = false;
        // A live session a confirmed reset retired for reconstruction.
        bool reset_preserved = false;
        // Reconstruction failed: a tombstone (process reference and handle,
        // no GPU resources unless quarantined) answering session_lost.
        bool poisoned = false;
        // Destructive recovery was already requested for this session.
        bool escalated = false;
        // Process exit time plus process_reap_idle_timeout_us.
        uint64_t exit_deadline_us = 0;
    };

    ProcessPlatform platform{};
    ProcessRecoveryHook recovery{};
    hardware::PasidAllocator pasids{};
    hardware::UserQueues queues{};
    Session sessions[hardware::user_queue_max_processes]{};
    process_abi::Request request{};
    process_abi::Reply reply{};
    uint8_t payload[process_abi::copy_bytes]{};
    // Version-3 views of the captured METHOD_BUFFERED system buffer, valid only
    // during one execute call (bulk upload/submit input, readback output).
    ByteView bulk_input{nullptr, 0};
    uint8_t* bulk_output = nullptr;
    uint32_t bulk_output_bytes = 0;
    // Device-wide graphics: attribute ring, scanout, shader generator, scratch.
    GraphicsDevice graphics{};
    GraphicsWorkspace graphics_workspace{};
    graphics_abi::PresentRequest present_request{};
    memory::Handle attribute_ring{};
    bool attribute_ring_live = false;
    uint8_t queue_pages[(hardware::user_queue_min_ring_bytes +
                         hardware::user_queue_control_bytes) / memory::page_bytes * 8]{};
    uint64_t next_handle = 1;
    bool initialized = false;
    bool parked = false;
    bool failed = false;
    bool registered = false;
    bool reset_locked = false;
    // Set only after the ASIC owner's confirmed-reset callback retired every
    // process VM and released its native buffer/page-table ownership. This is
    // not proof that a new ASIC incarnation is ready; rearm checks that
    // independently before reopening the service.
    bool reset_retired = false;
    // A session asked for destructive recovery; the native owner calls the
    // hook after dropping its locks (process_core_take_escalation).
    bool escalation_pending = false;
    ProcessCoreReport counters{};

    // Externally serialized. Admission inspects actual retained owners, never
    // a caller-supplied reset flag. It does not mutate handles or resources.
    bool can_restart_after_reset() const noexcept;
};

// First binding to a ready ASIC: initializes UserQueues through the platform.
bool process_core_bind(GpuProcessCore& core, const ProcessPlatform& platform) noexcept;

// Executes core.request (decoded and field-validated, payload captured) for the
// executing process token and fills core.reply. Never blocks.
process_abi::Outcome process_core_execute(GpuProcessCore& core, void* process) noexcept;

// Process-exit notification body: marks every session of process exiting
// (no further request is admitted) with its reap deadline. Releases nothing:
// the reaper does that on its own thread. true when a session matched.
bool process_core_mark_exit(GpuProcessCore& core, void* process) noexcept;

// One reaping pass over exited sessions (service lock held). retry_us: 0 when
// no session needs a timed retry, else the delay before the next pass.
struct ProcessReapResult final {
    uint64_t retry_us = 0;
};
ProcessReapResult process_core_reap(GpuProcessCore& core) noexcept;

// Reaper thread body for one wake: under service_lock reap, then (unlocked)
// run any escalation. stop is read under the lock. Returns the next retry (0
// waits for a signal); stopped is set when stop was observed.
struct ProcessReaperPass final {
    uint64_t retry_us = 0;
    bool stopped = false;
};
ProcessReaperPass process_core_reaper_pass(GpuProcessCore& core, const ProcessLock& service_lock,
                                           const bool& stop) noexcept;

// Moves a pending escalation out (under the service lock). The caller invokes
// hook.request after releasing every process lock. false: none pending.
bool process_core_take_escalation(GpuProcessCore& core, ProcessRecoveryHook& hook) noexcept;

// Destructive reset, step 1 (before the reset; service lock held until end):
// admission stops; live sessions are marked for reconstruction.
void process_core_begin_reset(GpuProcessCore& core) noexcept;
// Step 2, inside the confirmed-reset window (engines provably quiescent, GTT
// destruction reset-safe): fence floors and lost work recorded, every queue,
// gang, VM and PASID released (UserQueues::release_after_reset), live sessions
// retired for reconstruction (lost VRAM tombstoned, GTT kept), every other
// owner released. false keeps what could not be released (quarantined).
bool process_core_retire_reset(GpuProcessCore& core, bool (*confirm)(void*) noexcept,
                               void* confirm_context) noexcept;
// Step 3, after a new ASIC incarnation completed bring-up: UserQueues bound
// again; exited sessions closed; each retired live session reconstructed
// (new PASID and page tables, buffers at the same GPUVAs, queues re-added with
// doorbells and continued fences), bounded attempts, else poisoned. true when
// the service accepts requests again.
bool process_core_rearm(GpuProcessCore& core) noexcept;

// Power path (D0 -> D3 and back, no destructive loss of saved state).
bool process_core_suspend(GpuProcessCore& core) noexcept;
bool process_core_resume(GpuProcessCore& core) noexcept;
// Shutdown: every session closed and the attribute ring released. false keeps
// uncertain ownership retained.
bool process_core_close_all(GpuProcessCore& core) noexcept;

bool process_core_configure_scanout(GpuProcessCore& core, const GraphicsScanout* scanout) noexcept;
// RAS page retirement: every live session owning VRAM in the MC range
// [mc_address, mc_address + bytes) is quarantined (no request is admitted, the
// reaper keeps its resources, a destructive reset closes instead of rebuilding
// it). Returns how many were newly quarantined; owners, when given, receives
// every live session owning the range (newly or already quarantined, or
// poisoned), so the caller can tell a session-owned page from a driver one.
uint32_t process_core_quarantine_vram(GpuProcessCore& core, uint64_t mc_address, uint64_t bytes,
                                      uint32_t* owners = nullptr) noexcept;
// Present of core.present_request for the executing process token.
process_abi::Outcome process_core_present(GpuProcessCore& core, void* process, uint32_t& copied) noexcept;

} // namespace rx7900::reactos

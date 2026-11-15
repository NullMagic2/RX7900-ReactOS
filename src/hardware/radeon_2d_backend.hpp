/*
Module: rx7900.hardware.radeon_2d_backend
Purpose: Bind the Radeon 2D IOCTL dispatcher's Blit2dBackend contract to one
Blit2dEngine on an SDMA 6.0 kernel ring, with GTT staging/IB memory and
GPU-only VRAM offscreen surfaces.
Created: 2026-10-22
Architecture: Portable, allocation-free and host-tested (tests/radeon_2d_backend_tests.cpp
drives it through the real AsicInit bring-up in the shared simulator). Consumes
src/reactos/radeon_2d_ioctl.hpp (contract), src/hardware/blit2d (encoding, ordering,
IB recycling), Sdma6::submit and FenceManager query/wait (kernel-ring submission),
the AsicInit GTT memory::Manager (IBs, bounce scratch, staging) and the whole-VRAM
VramAllocator (offscreen surfaces). rx7900video owns one Radeon2dBackend beside its
radeon2d::NativeDispatcher and hands table() to native_configure.
*/
#pragma once

#include "asic_init.hpp"
#include "blit2d.hpp"
#include "gfx11_queues.hpp"
#include "raster2d.hpp"
#include "vram_allocator.hpp"
#include "../reactos/radeon_2d_ioctl.hpp"

namespace rx7900::hardware {

// SDMA instance of the 2D engine. Instance 0 stays free for other kernel work:
// Linux uses sdma.instance[0].ring as the TTM buffer_funcs ring (moves, clears,
// GART/VM updates), and FenceManager allows one submitter per ring with only
// fence_slots outstanding fences, so sharing one ring would make 2D latency and
// kernel memory work block each other and need a cross-module ring lock.
constexpr uint32_t radeon2d_sdma_instance = 1;

// Offscreen pitch alignment (bytes). 256 bytes is the DCN/GFX11 linear pitch
// granule (amdgpu dm linear surfaces, Mesa linear pitch alignment for 32 bpp).
constexpr uint32_t radeon2d_pitch_alignment = 256;

// IB ring defaults: four slots match the four fences one SDMA ring may have
// outstanding (fence_slots); 16 KiB slots bound the latency of one submission.
constexpr uint32_t radeon2d_default_ib_count = fence_slots;
constexpr uint32_t radeon2d_default_ib_bytes = 16u << 10;
// Same-surface overlap bounce buffer in GTT (0 disables bouncing; strips are
// then always used, which is correct but slower for one-line scrolls).
constexpr uint32_t radeon2d_default_scratch_bytes = 2u << 20;

// Offscreen byte budget of the backend (VRAM- and system-placed surfaces
// together, page-rounded allocation sizes): the dispatcher's device budget.
constexpr uint64_t radeon2d_default_surface_budget = reactos::radeon2d::abi::default_device_surface_bytes;

// Compute raster engine (src/hardware/raster2d) on one GFX11 kernel compute
// ring. Module ring 1 + i is compute ring i and FenceManager ring
// fence_ring_compute0 + i (the same index); ring 1 keeps 2D compute work off the
// gfx ring (OpenGL) and off the SDMA ring, so neither engine's fence_slots limit
// blocks the other. Four IB slots of the engine minimum (one operation each);
// the args ring holds one region set per slot.
constexpr uint32_t radeon2d_compute_ring = 1;
constexpr uint32_t radeon2d_compute_ib_count = 4;
constexpr uint32_t radeon2d_compute_ib_bytes = raster2d_min_ib_bytes;
static_assert(fence_ring_compute0 == radeon2d_compute_ring, "module ring index = FenceManager ring index");
// Gradient-triangle row plan held by the backend (rows per triangle). Taller
// triangles are rejected (the client renders them otherwise).
constexpr uint32_t radeon2d_triangle_rows = reactos::radeon2d::abi::max_triangle_rows;

// Bounded backend handle tables: every dispatcher surface slot and session.
constexpr uint32_t radeon2d_max_surfaces = reactos::radeon2d::abi::max_surfaces;
constexpr uint32_t radeon2d_max_staging = reactos::radeon2d::abi::max_sessions;

enum class Radeon2dStatus {
    ready,
    uninitialized,
    invalid_argument,
    // The SDMA instance has no kernel submission (enable_kernel_submission).
    submission_unavailable,
    // GTT/VRAM allocation for the IB ring or scratch failed; nothing retained.
    memory_failure,
    // Blit2dEngine::initialize refused the configuration; nothing retained.
    engine_failure,
    // shutdown: work still in flight, or allocations remain live or quarantined.
    busy,
    // suspend completed: the engine and its IB ring are released; staging and
    // evicted (system-memory) surfaces stay allocated until initialize again.
    suspended,
    // Sticky: a submission, fence or completion became uncertain. Every later
    // operation reports uncertain; only release_after_reset recovers.
    faulted,
    released
};

struct Radeon2dBackendConfig final {
    // Started SDMA rings with enable_kernel_submission done (AsicInit::sdma_rings()).
    Sdma6* sdma;
    // The FenceManager given to AsicInit::enable_kernel_submission.
    FenceManager* fences;
    // AsicInit::gtt_manager(): snooped system pages bound through the GART. The
    // same Manager the Sdma6 rings use, so Sdma6::submit can lease the IBs.
    memory::Manager* gtt;
    // Whole-VRAM allocator (AsicBringup::vram_allocator) and the MC address of
    // VRAM offset 0 (VramLocation.mc_start, AsicInit::report().vram.mc_start).
    VramAllocator* vram;
    uint64_t vram_mc_base;
    // SDMA instance (radeon2d_sdma_instance), IB slots, scratch and per-IB
    // fence wait budget (0 = blit2d_default_wait_us).
    uint32_t instance;
    uint32_t ib_count;
    uint32_t ib_bytes;
    uint32_t scratch_bytes;
    uint64_t wait_timeout_us;
    // Offscreen budget in bytes (0 = radeon2d_default_surface_budget).
    uint64_t surface_budget_bytes;
    // Optional compute raster engine: AsicInit::kernel_queues() with submission
    // enabled (AsicInit::enable_kernel_submission) and its module ring index
    // (radeon2d_compute_ring), on the same FenceManager and GTT Manager. A
    // caller-supplied compute_submitter (submit != null) replaces the Gfx11Queues
    // adapter, for an owner that schedules compute IBs itself and for host tests.
    // Neither: no compute engine (raster requests are not advertised).
    Gfx11Queues* queues;
    uint32_t compute_ring;
    Raster2dSubmitter compute_submitter;
};

// Defaults above for the given owners (queues optional: no compute without it).
Radeon2dBackendConfig radeon2d_default_config(Sdma6& sdma, FenceManager& fences,
                                              memory::Manager& gtt, VramAllocator& vram,
                                              uint64_t vram_mc_base,
                                              Gfx11Queues* queues = nullptr) noexcept;

struct Radeon2dBackendReport final {
    Radeon2dStatus status = Radeon2dStatus::uninitialized;
    // Backend fence: the operation counter (every accepted fill, copy, upload,
    // readback and offscreen clear) and the highest value proven retired.
    uint64_t fence = 0;
    uint64_t completed = 0;
    uint64_t accepted = 0;
    uint64_t rejected = 0;
    uint64_t uncertain = 0;
    uint32_t live_surfaces = 0;
    uint32_t quarantined_surfaces = 0;
    // Live surfaces currently placed in system memory (evicted) and the bytes
    // all live surfaces account against surface_budget_bytes.
    uint32_t system_surfaces = 0;
    uint64_t surface_bytes = 0;
    uint64_t evictions = 0;
    uint64_t restores = 0;
    uint64_t eviction_failures = 0;
    // park_after_reset: completed calls and VRAM-resident surfaces dropped.
    uint32_t resets = 0;
    uint64_t reset_dropped_surfaces = 0;
    uint32_t live_staging = 0;
    uint32_t quarantined_staging = 0;
    // The Blit2dStatus that made this backend faulted (success when none).
    Blit2dStatus fault_reason = Blit2dStatus::success;
    Blit2dReport engine{};
    // Compute engine: present after initialize, the Raster2dStatus of its
    // initialization (success when present), the one that faulted the backend,
    // and the fence waits the cross-engine ordering rule inserted.
    bool compute = false;
    Raster2dStatus compute_status = Raster2dStatus::not_initialized;
    Raster2dStatus compute_fault = Raster2dStatus::success;
    uint64_t cross_engine_waits = 0;
    uint64_t compute_self_waits = 0;
    Raster2dReport raster{};
};

// Raster2dSubmitter over Gfx11Queues::submit and FenceManager query/wait for one
// kernel compute ring, the same contract as Sdma6Blit2dSubmitter: submit
// ready/running -> submitted, ring_full/busy -> busy, other -> failed; query
// signalled/pending, other -> failed; wait signalled -> signalled, timed_out ->
// pending, other -> failed. PASSIVE_LEVEL; the device owner serializes it with
// every other Gfx11Queues use (power transitions park this backend first).
// ib_bytes/ib_count describe the engine's IB ring (see radeon2d_compute_ib_offset).
struct Gfx11Raster2dSubmitter final {
    Gfx11Queues* queues = nullptr;
    FenceManager* fences = nullptr;
    uint32_t ring = radeon2d_compute_ring;
    uint32_t ib_bytes = radeon2d_compute_ib_bytes;
    uint32_t ib_count = radeon2d_compute_ib_count;

    Raster2dSubmitter submitter() noexcept { return {this, submit, query, wait}; }

    static Raster2dSubmitResult submit(void* context, memory::Handle ib, uint64_t ib_offset,
                                       uint32_t ib_dwords, uint32_t& sequence) noexcept;
    static Raster2dFenceState query(void* context, uint32_t sequence) noexcept;
    static Raster2dFenceState wait(void* context, uint32_t sequence, uint64_t timeout_us) noexcept;
};

// The byte offset of a submitted compute IB slot: Raster2dEngine passes the slot's
// BYTE offset inside the IB allocation (slot i at i * ib_bytes). false: the value
// names no slot start.
bool radeon2d_compute_ib_offset(uint64_t value, uint32_t ib_bytes, uint32_t ib_count,
                                uint64_t& offset) noexcept;

// Blit2dSubmitter over Sdma6::submit and FenceManager query/wait for one SDMA
// instance, exactly as blit2d.hpp documents: submit ready/started -> submitted,
// ring_full/busy -> busy, other -> failed; query signalled/pending, other ->
// failed; wait signalled -> signalled, timed_out -> pending, other -> failed.
// All PASSIVE_LEVEL, serialized by the engine's owner; the referenced objects
// outlive the submitter.
struct Sdma6Blit2dSubmitter final {
    Sdma6* sdma = nullptr;
    FenceManager* fences = nullptr;
    uint32_t instance = 0;

    Blit2dSubmitter submitter() noexcept { return {this, submit, query, wait}; }

    static Blit2dSubmitResult submit(void* context, memory::Handle ib, uint64_t ib_offset,
                                     uint32_t ib_dwords, uint32_t& sequence) noexcept;
    static Blit2dFenceState query(void* context, uint32_t sequence) noexcept;
    static Blit2dFenceState wait(void* context, uint32_t sequence, uint64_t timeout_us) noexcept;
};

// Pure status mappings (exposed for tests): Blit2dStatus success -> ok; the
// refused-before-encoding statuses -> rejected; faulted, submit_failed,
// fence_timeout and fence_failed -> uncertain.
reactos::radeon2d::BackendStatus radeon2d_backend_status(Blit2dStatus status) noexcept;
Blit2dSubmitResult radeon2d_submit_result(Sdma6Status status) noexcept;
Blit2dFenceState radeon2d_query_state(FenceStatus status) noexcept;
Blit2dFenceState radeon2d_wait_state(FenceStatus status) noexcept;
Raster2dSubmitResult radeon2d_compute_submit_result(Gfx11QueuesStatus status) noexcept;
// Raster2dStatus success -> ok; refused-before-submission statuses -> rejected;
// invalid_memory, faulted, submit_failed, fence_timeout, fence_failed -> uncertain.
reactos::radeon2d::BackendStatus radeon2d_raster_status(Raster2dStatus status) noexcept;

// Device-owned (trivially destructible, about 8.1 KiB on i386, value-initialized in the
// device extension), PASSIVE_LEVEL only and externally serialized: every call
// runs under the radeon2d::NativeDispatcher mutex, and the device owner never
// uses the GTT Manager, the VramAllocator or the Sdma6/FenceManager submission
// path for instance 1 concurrently (power transitions first shut this down).
//
// Mapping of the contract (radeon_2d_ioctl.hpp):
//  fill: one Blit2dEngine::fill with bounds = union of the rectangles and the
//    rectangles as its clip list (all validated before anything is encoded).
//  copy: when every rectangle has the same source offset (the dispatcher's
//    CopyCommand has one), one Blit2dEngine::copy with destination_rect = union,
//    source_point = union origin + offset and the rectangles as clip list; the
//    engine orders same-surface rectangles itself (snapshot semantics, stronger
//    than the per-rectangle memmove contract). Otherwise one copy per rectangle
//    in array order.
//  upload/readback: one engine call per TransferRect, Blit2dStaging{handle,
//    staging_offset, pitch, width, height} at staging point (0, 0).
//  Every accepted operation is flushed to the ring before returning ("submit
//    promptly") and returns fence = ++counter. A refusal of the first (or only)
//    engine call is rejected; any failure after an earlier engine call of the
//    same operation was accepted is uncertain, as are all fault statuses.
//  sync(fence): ok when fence <= completed; otherwise Blit2dEngine::synchronize
//    (bounded by the engine's per-IB fence wait), after which completed = the
//    counter value observed before it. A fence above the counter is rejected.
//  completed: Blit2dEngine::retire (non-blocking); when no IB is in flight or
//    open, completed = counter.
//  surface_create: a contiguous GPU-only (NO_CPU_ACCESS, TOPDOWN) VRAM kernel BO
//    from the VramAllocator, pitch aligned to radeon2d_pitch_alignment, cleared
//    to zero by the engine before it is returned (no stale VRAM contents reach
//    a session). staging_create: a CPU-visible GTT allocation of exactly the
//    requested size, zeroed by the CPU.
//  surface_destroy/staging_destroy: freed only when every operation the backend
//    accepted so far has provably retired (the dispatcher syncs first; the
//    backend re-checks and synchronizes itself); otherwise the allocation is
//    quarantined: never freed or reused until release_after_reset.
//  surface_evict: a GPU read/write GTT allocation of the page-rounded surface
//    size, one engine copy of the whole surface (same pitch) into it, flush and
//    synchronize, then the VRAM is released and the surface lives in GTT (the
//    SDMA addresses GART pages like VRAM, so every operation keeps working).
//    No GTT memory: rejected with nothing changed. surface_restore is the
//    reverse with a fresh contiguous VRAM allocation; failure keeps it in GTT.
//  Budget: surface_create refuses (rejected) when the page-rounded size would
//    exceed surface_budget_bytes over every live surface.
//
// Power transitions (Linux amdgpu_device_evict_resources, then the IP suspend):
// Dispatcher::suspend evicts every offscreen surface, then suspend() parks this
// backend (engine and IB ring released; staging and GTT surfaces kept, which
// AsicInit::suspend preserves unchanged). After AsicInit::resume, initialize
// again with the same owners and Dispatcher::resume restores the surfaces.
//
// Coherence (checked against Sdma6/FenceManager, so nothing is added here):
// every Sdma6::submit frame emits GCR_REQ and the POLL_REGMEM HDP flush before
// INDIRECT (amdgpu_ib_schedule: amdgpu_device_flush_hdp with a ring), and Sdma6
// flushes HDP from the CPU before publishing WPTR, so CPU BAR writes to VRAM
// (Eng* fallbacks on the primary) are visible to the next IB. Staging is GART
// system memory with snooped PTEs, so the CPU sees GPU writes once the fence
// signalled; soc21 HDP 6.0 has no invalidate_hdp (Linux amdgpu_device_invalidate_hdp
// is a no-op there), so a BAR read of VRAM after sync needs no extra step.
// Readbacks are ordered after earlier writes to the same surface by the engine's
// hazard tracker (NOP barrier, across IB boundaries) and the ring order.
//
// Compute raster engine (optional, raster_* callbacks): one Raster2dEngine
// operation per call (per clip rectangle for lines, per clip rectangle and
// 64-row chunk for triangles), flushed like SDMA work, with fence = ++counter
// on the same counter. Its submit frame brackets every IB with ACQUIRE_MEM
// (GL0/1/2 invalidate) and RELEASE_MEM CACHE_FLUSH_AND_INV_TS with GL2
// writeback, so data reaches memory before the fence signals.
// Cross-engine ordering rule: the SDMA ring and the compute ring do not order
// each other, and a compute queue does not wait for an earlier IB's dispatches
// to finish before starting the next. The backend therefore keeps the memory
// regions (Blit2dRegion: surface rectangles, or byte spans of staging and
// non-32-bpp operands) touched by not yet retired work of each engine; before
// an SDMA operation whose regions conflict (read/write, write/read or
// write/write) with pending compute work, and before a compute operation that
// conflicts with pending SDMA or compute work, it waits for the other engine's
// (or its own earlier) fences. A tracker resets when its engine is idle and
// forces that wait when it is full. sync and completed cover both engines.
class Radeon2dBackend final {
public:
    Radeon2dBackend() noexcept = default;
    Radeon2dBackend(const Radeon2dBackend&) = delete;
    Radeon2dBackend& operator=(const Radeon2dBackend&) = delete;

    // Validates the owners (submission enabled on the instance), allocates the
    // IB ring (ib_count * ib_bytes, CPU visible, GPU read) and optional scratch
    // (GPU read/write) from the GTT Manager and initializes the engine. Every
    // failure releases what it allocated. Refused while initialized or faulted.
    Radeon2dStatus initialize(const Radeon2dBackendConfig& config) noexcept;

    // The dispatcher's callback table; context is this object, which must stay
    // initialized while the dispatcher is configured with it.
    reactos::radeon2d::Blit2dBackend table() noexcept;

    // After radeon2d Dispatcher::shutdown: requires that every surface and
    // staging buffer was destroyed (none live or quarantined) and no work is in
    // flight; then shuts the engine down and releases the IB ring and scratch.
    // busy keeps everything; faulted requires release_after_reset.
    Radeon2dStatus shutdown() noexcept;

    // After radeon2d Dispatcher::suspend: requires that no surface is live in
    // VRAM or quarantined and that all work retired; shuts the engine down and
    // releases the IB ring and scratch, keeping staging and system-memory
    // surfaces. Returns suspended (parked) or busy/faulted with nothing released.
    // A parked backend accepts initialize (resume) or, once the dispatcher has
    // destroyed everything, shutdown.
    Radeon2dStatus suspend() noexcept;

    // Only after the device owner proved the SDMA engine quiescent (a completed
    // AsicInit::mode1_reset, before Sdma6/FenceManager release_after_reset):
    // drops the engine state and releases every allocation, quarantined ones
    // included, then returns released. The dispatcher must be shut down first.
    Radeon2dStatus release_after_reset() noexcept;

    // Destructive-reset recovery, in the same window as release_after_reset
    // (AsicBringup::surface_reset_retire): drops the engine state and frees the
    // IB ring, scratch and compute memory, every quarantined surface and staging
    // buffer, and the VRAM of every VRAM-resident surface (a mode-1 reset loses
    // VRAM contents, so those entries become free; the dispatcher reports them
    // lost from its own placement record). Live staging and system-memory
    // surfaces stay allocated with unchanged GPU addresses: system pages
    // survive the reset and AsicInit::restart_after_reset rebinds them. Fences
    // handed out so far count as retired. Returns suspended (parked: initialize
    // with the same owners restarts it) or uninitialized when there was no
    // backend. The dispatcher stays configured (begin_recovery) meanwhile.
    Radeon2dStatus park_after_reset() noexcept;

    // Sticky submission/fence uncertainty (report().status == faulted), cheap.
    bool faulted() const noexcept { return initialized_ && faulted_; }
    uint32_t resets() const noexcept { return resets_; }

    Radeon2dBackendReport report() const noexcept;

    // Contract implementation (also called by the table trampolines).
    reactos::radeon2d::BackendStatus fill(const reactos::radeon2d::EngineSurface& target,
                                          uint32_t color, const reactos::radeon2d::EngineRect* rects,
                                          uint32_t count, uint64_t& fence) noexcept;
    reactos::radeon2d::BackendStatus copy(const reactos::radeon2d::EngineSurface& target,
                                          const reactos::radeon2d::EngineSurface& source,
                                          const reactos::radeon2d::CopyRect* rects, uint32_t count,
                                          uint64_t& fence) noexcept;
    reactos::radeon2d::BackendStatus transfer(bool upload, const reactos::radeon2d::EngineSurface& surface,
                                              const reactos::radeon2d::StagingBuffer& staging,
                                              uint32_t staging_pitch,
                                              const reactos::radeon2d::TransferRect* rects,
                                              uint32_t count, uint64_t& fence) noexcept;
    reactos::radeon2d::BackendStatus surface_create(uint32_t width, uint32_t height, uint32_t format,
                                                    reactos::radeon2d::EngineSurface& output,
                                                    uint32_t& allocation) noexcept;
    void surface_destroy(uint32_t allocation) noexcept;
    reactos::radeon2d::BackendStatus staging_create(uint32_t bytes,
                                                    reactos::radeon2d::StagingBuffer& output) noexcept;
    void staging_destroy(const reactos::radeon2d::StagingBuffer& staging) noexcept;
    reactos::radeon2d::BackendStatus sync(uint64_t fence) noexcept;
    uint64_t completed() noexcept;
    reactos::radeon2d::BackendStatus surface_evict(uint32_t allocation,
                                                   reactos::radeon2d::EngineSurface& output) noexcept;
    reactos::radeon2d::BackendStatus surface_restore(uint32_t allocation,
                                                     reactos::radeon2d::EngineSurface& output) noexcept;
    reactos::radeon2d::BackendStatus raster_rop(const reactos::radeon2d::RasterRop& operation,
                                                const reactos::radeon2d::EngineRect* rects, uint32_t count,
                                                uint64_t& fence) noexcept;
    reactos::radeon2d::BackendStatus raster_alpha(const reactos::radeon2d::RasterAlpha& operation,
                                                  const reactos::radeon2d::EngineRect* rects, uint32_t count,
                                                  uint64_t& fence) noexcept;
    reactos::radeon2d::BackendStatus raster_line(const reactos::radeon2d::RasterLine& operation,
                                                 const reactos::radeon2d::EngineRect* rects, uint32_t count,
                                                 uint64_t& fence) noexcept;
    reactos::radeon2d::BackendStatus raster_gradient(const reactos::radeon2d::RasterGradient& operation,
                                                     const reactos::radeon2d::EngineRect* rects, uint32_t count,
                                                     uint64_t& fence) noexcept;

private:
    enum class Slot : uint8_t { free, live, quarantined };

    struct SurfaceEntry final {
        Slot state = Slot::free;
        // Placement: the surface address is in GTT (evicted) or VRAM.
        bool in_system = false;
        // Owned allocations. Both are held while a move is in progress or
        // became uncertain (then the entry is freed only by release_after_reset).
        bool has_vram = false;
        bool has_system = false;
        uint32_t generation = 0;
        uint64_t bytes = 0;
        VramAllocationHandle vram{};
        memory::Handle system{};
        reactos::radeon2d::EngineSurface surface{};
    };

    struct StagingEntry final {
        Slot state = Slot::free;
        uint32_t generation = 0;
        memory::Handle handle{};
        uint64_t gpu_address = 0;
        uint32_t bytes = 0;
    };

    // Memory one operation touches (up to three reads and one write).
    struct Touch final {
        Blit2dRegion reads[3];
        uint32_t read_count;
        Blit2dRegion write;
    };

    reactos::radeon2d::BackendStatus usable() const noexcept;
    reactos::radeon2d::BackendStatus compute_usable() const noexcept;
    // Cross-engine ordering (see the class comment). false: a wait failed and
    // the backend is uncertain.
    bool order_sdma(const Touch& touch) noexcept;
    bool order_compute(const Touch& touch) noexcept;
    void record(Blit2dHazardTracker& tracker, const Touch& touch) noexcept;
    bool wait_sdma() noexcept;
    bool wait_compute() noexcept;
    reactos::radeon2d::BackendStatus finish_compute(Raster2dStatus status, bool partial,
                                                    uint64_t& fence) noexcept;
    reactos::radeon2d::BackendStatus become_uncertain_compute(Raster2dStatus reason) noexcept;
    bool raster_surface(const reactos::radeon2d::EngineSurface& surface, Raster2dFormat format,
                        Raster2dSurface& output) const noexcept;
    bool raster_clip(const reactos::radeon2d::EngineRect* rects, uint32_t count) noexcept;
    Raster2dStatus initialize_compute() noexcept;
    void release_compute() noexcept;
    reactos::radeon2d::BackendStatus finish(Blit2dStatus status, bool partial,
                                            uint64_t& fence) noexcept;
    reactos::radeon2d::BackendStatus become_uncertain(Blit2dStatus reason) noexcept;
    bool engine_surface(const reactos::radeon2d::EngineSurface& surface,
                        Blit2dSurface& output) const noexcept;
    bool prepare_clip(const reactos::radeon2d::EngineRect* rects, uint32_t count,
                      Blit2dRect& bounds) noexcept;
    StagingEntry* find_staging(const reactos::radeon2d::StagingBuffer& staging) noexcept;
    SurfaceEntry* find_surface(uint32_t allocation) noexcept;
    // Contiguous VRAM of bytes at an SDMA-addressable MC address, or rejected.
    bool allocate_vram(uint64_t bytes, VramAllocationHandle& vram, uint64_t& gpu) noexcept;
    // One whole-surface engine copy, flushed and synchronized (ok = retired).
    reactos::radeon2d::BackendStatus move_contents(const reactos::radeon2d::EngineSurface& from,
                                                   uint64_t to_gpu) noexcept;
    // True once every accepted operation has provably retired (may synchronize).
    bool idle_for_release() noexcept;
    void release_surface(SurfaceEntry& entry) noexcept;
    void release_staging(StagingEntry& entry) noexcept;
    void release_engine_memory() noexcept;

    Radeon2dBackendConfig config_{};
    Sdma6Blit2dSubmitter submitter_{};
    Blit2dEngine engine_{};
    memory::Handle ib_{};
    memory::Handle scratch_{};
    bool has_scratch_ = false;
    bool initialized_ = false;
    // suspend() completed: no engine, nothing in flight, entries retained.
    bool parked_ = false;
    uint64_t surface_bytes_ = 0;
    uint64_t evictions_ = 0;
    uint64_t restores_ = 0;
    uint64_t eviction_failures_ = 0;
    // Completed park_after_reset calls, and VRAM surfaces they dropped.
    uint32_t resets_ = 0;
    uint64_t reset_dropped_ = 0;
    bool faulted_ = false;
    Blit2dStatus fault_reason_ = Blit2dStatus::success;
    uint64_t counter_ = 0;
    uint64_t completed_ = 0;
    uint64_t accepted_ = 0;
    uint64_t rejected_ = 0;
    uint64_t uncertain_ = 0;
    // Device-owned clip workspace (about 1 KiB): never on the kernel stack.
    Blit2dClipList clip_{};
    // Compute engine state (has_compute_ while raster_ is initialized).
    Gfx11Raster2dSubmitter compute_adapter_{};
    Raster2dEngine raster_{};
    memory::Handle compute_code_{};
    memory::Handle compute_args_{};
    memory::Handle compute_ib_{};
    bool has_compute_ = false;
    Raster2dStatus compute_status_ = Raster2dStatus::not_initialized;
    Gfx11QueuesReport queue_report_{};
    Raster2dStatus compute_fault_ = Raster2dStatus::success;
    uint64_t cross_waits_ = 0;
    uint64_t compute_self_waits_ = 0;
    // Regions of not yet retired work per engine (cross-engine ordering).
    Blit2dHazardTracker sdma_pending_{};
    Blit2dHazardTracker compute_pending_{};
    SurfaceEntry surfaces_[radeon2d_max_surfaces]{};
    StagingEntry staging_[radeon2d_max_staging]{};
};

// ---------------------------------------------------------------- recovery

// Completed destructive-reset recoveries allowed per device lifetime; the next
// fault after that many disables 2D (an ASIC that keeps hanging is not reset in
// a loop).
constexpr uint32_t radeon2d_max_recoveries = 3;

enum class Radeon2dRecoveryResult : uint32_t {
    recovered,
    // Recovery is sticky off already (an earlier failure).
    disabled,
    // radeon2d_max_recoveries recoveries completed: disabled without a reset.
    limit_reached,
    // The dispatcher refused to quiesce (not configured).
    dispatcher_refused,
    // The interrupt service could not be quiesced: nothing was reset.
    quiesce_failed,
    // No reset proof, or release_after_reset retained the ASIC.
    reset_failed,
    // The bring-up after the reset (or its kernel submission) failed.
    restart_failed,
    // The DCN state no longer matches the adopted pre-OS scanout.
    scanout_mismatch,
    // The backend did not restart on the recovered rings.
    backend_failed,
    // A restore after the restart was uncertain (a second fault).
    second_fault
};

// Device-owner services for one recovery, all PASSIVE_LEVEL under the owner's
// exclusive serialization (no IOCTL, power callback or other ASIC user runs).
//  display_quiesce/display_resume (optional): the cursor plane goes off before
//    the reset, and back on after the scanout was re-validated.
//  stop_interrupts: ISR disarmed under the interrupt lock, queued DPCs flushed,
//    acknowledged vectors drained, IrqService detached (suspended) and the fence
//    poll stopped. false: nothing was reset.
//  reset_and_release: the explicit mode-1 reset through the ASIC owner and
//    AsicInit::release_after_reset, whose reset window calls retire_after_reset
//    below; released expected. reset_proven: the reset proof exists (also when
//    the release retained the ASIC).
//  restart: AsicInit::restart_after_reset, interrupt sources, fence poll and
//    kernel submission on the same FenceManager; completed expected.
//  rearm_services (optional): other owners' confirmed-reset rearm (the process
//    service); its result is that owner's.
//  scanout_matches: the pre-OS same-scanout check against the restarted ASIC.
struct Radeon2dRecoveryPlatform final {
    void* context;
    void (*display_quiesce)(void* context) noexcept;
    void (*display_resume)(void* context) noexcept;
    bool (*stop_interrupts)(void* context) noexcept;
    AsicStatus (*reset_and_release)(void* context) noexcept;
    bool (*reset_proven)(void* context) noexcept;
    AsicStatus (*restart)(void* context) noexcept;
    void (*rearm_services)(void* context) noexcept;
    bool (*scanout_matches)(void* context) noexcept;
};

struct Radeon2dRecoveryReport final {
    uint32_t attempts = 0;
    uint32_t recovered = 0;
    uint32_t failed = 0;
    uint32_t scanout_mismatches = 0;
    Radeon2dRecoveryResult last = Radeon2dRecoveryResult::recovered;
    // reset_and_release and restart results of the last attempt.
    AsicStatus reset_status = AsicStatus::completed;
    AsicStatus restart_status = AsicStatus::completed;
    Radeon2dStatus backend_status = Radeon2dStatus::uninitialized;
    // Sticky off, and whether no earlier 2D GPU work can still execute.
    bool disabled = false;
    bool quiescent = false;
};

// Orderly destructive-reset recovery of physical 2D (Linux amdgpu_device_gpu_recover
// for a MODE1 reset, without its job resubmission): quiesce the dispatcher and
// mark work that is not proven retired uncertain, cursor off, interrupts and
// fence poll quiet, mode-1 reset and release through the ASIC owner (the
// backend parks inside the reset window), bring-up again, interrupt sources and
// kernel submission, other services' rearm, re-validation of the adopted
// scanout, backend restart on the same memory owners and dispatcher recovery
// (system-memory surfaces restored exactly, VRAM ones lost). Any failure
// disables 2D for the device lifetime (Dispatcher::disable); the report says why
// and whether the GPU is proven quiescent. Device-owned, trivially destructible.
// Never called from DPC or interrupt context.
class Radeon2dRecovery final {
public:
    // A recovery is due: the backend faulted (sticky submission/fence
    // uncertainty) or a kernel ring of fences is hung (job timeout, CP or
    // VMID0 fault). Takes the fence lock once per ring; any IRQL <= DISPATCH.
    static bool needed(const Radeon2dBackend& backend, FenceManager& fences) noexcept;

    // AsicBringup::surface_reset_retire: context is the Radeon2dBackend.
    // Runs Radeon2dBackend::park_after_reset; true unless it refused.
    static bool retire_after_reset(void* backend) noexcept;

    // One complete recovery. primary is the adopted scanout surface; config
    // names the same owners the backend was started with (their objects
    // survive the restart). The dispatcher must be configured; it may already
    // be quiesced (Dispatcher::begin_recovery at detection time).
    Radeon2dRecoveryResult recover(reactos::radeon2d::Dispatcher& dispatcher, Radeon2dBackend& backend,
                                   const Radeon2dBackendConfig& config,
                                   const reactos::radeon2d::EngineSurface& primary,
                                   const Radeon2dRecoveryPlatform& platform) noexcept;

    bool disabled() const noexcept { return report_.disabled; }
    Radeon2dRecoveryReport report() const noexcept { return report_; }

private:
    Radeon2dRecoveryResult fail(reactos::radeon2d::Dispatcher& dispatcher, Radeon2dRecoveryResult reason,
                                bool quiescent) noexcept;

    Radeon2dRecoveryReport report_{};
};

enum class Radeon2dRecoveryState : uint32_t { idle, requested, running };

struct Radeon2dRecoveryGateReport final {
    Radeon2dRecoveryState state = Radeon2dRecoveryState::idle;
    bool forced = false;
    bool stopping = false;
    bool disabled = false;
    uint32_t requests = 0;
    uint32_t coalesced = 0;
    uint32_t refused = 0;
    uint32_t deferred = 0;
    uint32_t dropped = 0;
    uint32_t runs = 0;
};

// Hand-off between the paths that detect a fault (a Radeon 2D IOCTL that saw an
// uncertain outcome, a failed power transition) and the owner's recovery worker
// thread, which runs Radeon2dRecovery on its own fresh kernel stack. Every call
// is made under the device owner's mutex (PASSIVE_LEVEL); the owner wakes the
// worker when request returns true. Device-owned, trivially destructible.
//  request: idle -> requested (true: wake the worker). A pending or running
//    recovery absorbs it (coalesced); after stop or a disabling recovery it is
//    refused. forced: run even when Radeon2dRecovery::needed is false (a failed
//    power transition rather than a hang).
//  begin (worker): requested -> running when not in low power and the recovery
//    is forced or still due; in low power it stays requested (deferred, the
//    owner wakes the worker again after D0); a request that is no longer due is
//    dropped.
//  finish (worker): running -> idle; disabled (the recovery left 2D off for
//    good) makes later requests refused.
//  stop (teardown): no request is accepted any more; true when no recovery is
//    running, so the worker may be told to exit and be joined.
class Radeon2dRecoveryGate final {
public:
    bool request(bool forced) noexcept;
    bool begin(bool low_power, bool due) noexcept;
    void finish(bool disabled) noexcept;
    bool stop() noexcept;
    bool pending() const noexcept { return state_ != Radeon2dRecoveryState::idle; }
    bool stopping() const noexcept { return stopping_; }
    Radeon2dRecoveryGateReport report() const noexcept;

private:
    Radeon2dRecoveryState state_ = Radeon2dRecoveryState::idle;
    bool forced_ = false;
    bool stopping_ = false;
    bool disabled_ = false;
    uint32_t requests_ = 0;
    uint32_t coalesced_ = 0;
    uint32_t refused_ = 0;
    uint32_t deferred_ = 0;
    uint32_t dropped_ = 0;
    uint32_t runs_ = 0;
};

} // namespace rx7900::hardware

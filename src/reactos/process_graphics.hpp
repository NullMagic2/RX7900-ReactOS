/*
Module: rx7900.reactos.process_graphics
Purpose: Kernel-built GFX11 graphics work for one native process session: decode
the typed version-3 draw description, resolve opaque buffer handles inside the
session's own GPUVM, generate the fixed-function shaders from validated keys,
build the PM4 IB with the gfx11_state allow-listed builder, submit it to the
session's MES gfx queue and lease every referenced buffer until the queue fence
retires it. Also performs window-clipped GPU presents into the adopted scanout.
Created: 2026-10-23
Architecture: One serialized owner per GpuProcessService session, analogous to
ProcessSubmissions (semantic SDMA). It reuses ProcessMemory (handles, mappings,
leases), UserQueues (VM mappings, submission, fences) and the GTT Manager for
driver-owned IB, vertex and shader memory.

Security model (the kernel is the only author of GPU commands):
  * User mode sends a bounded description: buffer handles, GL enumerants, float
    bit patterns, a gl_gfx11_abi ShaderKey and vertex records. The decoder in
    process_graphics_abi.hpp range-checks every field and every count against the
    exact captured payload size; anything unknown is refused.
  * Handles resolve only through this session's ProcessMemory, which belongs to
    the authenticated NT process (gpu_process authorize); a foreign or stale handle
    is stale_handle. Each resolved buffer must be mapped with the access the draw
    needs and large enough for the exact surface footprint; addresses are taken
    from the driver's own mapping record, never from the request.
  * Shader code is generated here from the validated key (gl_shader_gfx11) into a
    driver-owned GTT area mapped read-only/executable at a private VA. User
    buffers are never executable (ProcessMemory maps them non-executable).
  * The IB, vertex copy and shader areas are driver-owned GTT mapped read-only in
    the process VM at private VAs that ProcessMemory::map cannot overlap
    (UserQueues refuses overlapping mappings). Shaders read memory only through
    descriptors built here with exact NUM_RECORDS / extents.
  * Every register write passes Pm4Builder::set, which refuses registers outside
    the gfx11_reg allow-list; no request value selects a register.
  * Work runs in the session's own VM on its own MES queue; buffers stay leased
    (CPU access/unmap/release refused) until the actual fence signals or the
    queue/VM is proven removed. Uncertain publication quarantines and retains.
  * Kernel-ring route (open with queue 0, Linux amdgpu_cs on the gfx ring): the
    same IB, built and mapped exactly as above, runs as a process-VM job on the
    kernel gfx ring (Gfx11Queues grabs a VMID 1..7, flushes it to this
    session's page directory and maps its PASID). Each job slot has a
    PhysicalGraphicsSubmission that keeps the generated-shader area leased with
    the ring's IB lease on the job area until the FenceManager receipt or the
    confirmed reset window; the buffer leases are released only then. Sequence
    numbers returned to the client are session-local and monotonic (they keep
    counting across a reset, whose lost jobs the owner records).
  * Present is reachable only through the kernel-only present IOCTL that
    radeondisp issues from DrvDrawEscape with win32k's CLIPOBJ rectangles. The
    scanout is mapped write-only at a private VA of each graphics session; only
    kernel-built DMA_DATA rows bounded by those rectangles ever target it.
PASSIVE_LEVEL; externally serialized by the GpuProcessService mutex. Native
callers supply a floating-point save scope (KeSaveFloatingPointState) because
viewport and polygon-offset values are computed in float.
*/
#pragma once

#include "process_memory.hpp"
#include "process_graphics_abi.hpp"
#include "../opengl/gfx11_state.hpp"
#include "../opengl/physical_submission.hpp"

namespace rx7900::reactos {

// Private VA window of every graphics session (4 GiB above the SDMA IB page).
constexpr uint64_t graphics_private_va = (1ULL << 45) + (1ULL << 32);
constexpr uint64_t graphics_attribute_va = graphics_private_va;
constexpr uint64_t graphics_shader_va = graphics_private_va + (16ULL << 20);
constexpr uint64_t graphics_job_va = graphics_private_va + (32ULL << 20);
constexpr uint64_t graphics_scanout_va = graphics_private_va + (64ULL << 20);
constexpr uint64_t graphics_scanout_max_bytes = 64ULL << 20;

constexpr uint32_t graphics_jobs = 2;
constexpr uint32_t graphics_ib_bytes = 128u * 1024u;
// Job data: the vertex records, then (gfx11_abi v4) one kernel-built PS constant
// block per draw state, rounded so the job stays a whole number of pages.
constexpr uint32_t graphics_constants_offset = graphics_abi::max_vertex_bytes;
constexpr uint32_t graphics_constants_bytes =
    (graphics_abi::max_states * opengl::gfx11_abi::ps_constants_bytes + memory::page_bytes - 1u) /
    memory::page_bytes * memory::page_bytes;
constexpr uint32_t graphics_data_bytes = graphics_constants_offset + graphics_constants_bytes;
constexpr uint32_t graphics_job_bytes = graphics_ib_bytes + graphics_data_bytes;
constexpr uint32_t graphics_shader_slots = 8;
constexpr uint32_t graphics_shader_slot_bytes = 2u * opengl::gfx11_abi::shader_max_bytes;
constexpr uint32_t graphics_shader_bytes = graphics_shader_slots * graphics_shader_slot_bytes;
// Colour, depth, the GL_FRONT_AND_BACK second colour buffer, and per state a
// texture and a pixel-rectangle source.
constexpr uint32_t graphics_lease_capacity = 3 + 2 * graphics_abi::max_states;
constexpr uint32_t graphics_page_entries = graphics_jobs * graphics_job_bytes / memory::page_bytes;
constexpr uint32_t graphics_present_timeout_us = 200000;
// Bounded kernel-ring drain before a power transition (wait_kernel_idle).
constexpr uint32_t graphics_kernel_drain_timeout_us = 200000;
static_assert(graphics_abi::buffer_max_bytes == process_memory_max_bytes, "user-mode buffer cap");
static_assert(graphics_job_bytes % memory::page_bytes == 0, "job areas are whole pages");
static_assert(graphics_abi::buffer_page_bytes == memory::page_bytes, "user-mode page size");

// Shader generation (gl_shader_gfx11 in production). Each writes one program of
// at most gfx11_abi::shader_max_bytes into output and fills info. context is the
// device-owned generator workspace (shader::GeneratorWorkspace), used only under
// the service mutex so the about 2 KiB of assembler state never lives on a
// kernel stack.
struct GraphicsShaderGenerator final {
    void* context = nullptr;
    bool (*vertex)(void* context, const opengl::gfx11_abi::ShaderKey&, sdma::CommandBuffer output,
                   opengl::gfx11_abi::ProgramInfo& info) noexcept = nullptr;
    bool (*pixel)(void* context, const opengl::gfx11_abi::ShaderKey&, sdma::CommandBuffer output,
                  opengl::gfx11_abi::ProgramInfo& info) noexcept = nullptr;
};

// Runs body inside a saved floating-point state (native KeSaveFloatingPointState);
// false when the state could not be saved (body not run).
struct GraphicsFloatScope final {
    void* context = nullptr;
    bool (*run)(void* context, bool (*body)(void*) noexcept, void* argument) noexcept = nullptr;
};

// Device-wide graphics resources and services, owned by GpuProcessService.
struct GraphicsDevice final {
    GraphicsShaderGenerator generator{};
    GraphicsFloatScope fpu{};
    // Bounded wait used by present (native: KeDelayExecutionThread).
    void (*wait_us)(void* context, uint32_t microseconds) noexcept = nullptr;
    void* wait_context = nullptr;
    // The kernel-owned attribute ring (one per device, VRAM).
    bool attribute_ring = false;
    uint64_t attribute_ring_mc = 0;
    uint32_t attribute_ring_per_se_bytes = 0;
    uint32_t shader_engines = 0;
    // The adopted pre-OS scanout (XRGB8888, linear), or none.
    bool scanout = false;
    uint64_t scanout_mc = 0;
    uint32_t scanout_width = 0;
    uint32_t scanout_height = 0;
    uint32_t scanout_pitch_bytes = 0;
};

// Large decode/build scratch shared by every session; the service mutex
// serializes its use. Device-owned storage, never a kernel stack.
struct GraphicsWorkspace final {
    opengl::gfx11::Pm4Builder builder;
    graphics_abi::GraphicsState states[graphics_abi::max_states];
    graphics_abi::GraphicsCommand commands[graphics_abi::max_commands];
    graphics_abi::SubmitHeader header;
    opengl::gfx11::Framebuffer framebuffer;
    opengl::gfx11::ShaderPrograms programs;
    opengl::gfx11::DrawInputs draw;
    opengl::gfx11::RasterState raster;
    ProcessBufferInfo info;
    uint8_t pages[graphics_page_entries * sizeof(uint64_t)];
    // The kernel gfx-ring job description publish_kernel hands to its owner.
    opengl::TrustedGraphicsJob kernel_job;
};

struct ProcessGraphicsConfig final {
    hardware::UserQueues* queues = nullptr;
    ProcessMemory* buffers = nullptr;
    memory::Manager* gtt = nullptr;
    uint32_t pasid = 0;
    void* context = nullptr;
    bool (*dma_bus)(void*, memory::Handle, uint64_t&) noexcept = nullptr;
    const GraphicsDevice* device = nullptr;
    GraphicsWorkspace* workspace = nullptr;
    // The kernel gfx-ring submitter (KernelGraphicsBridge in production); its
    // submit callback null: only the MES-queue route is available.
    opengl::GraphicsSubmitter kernel_ring{};
};

class ProcessGraphics final {
public:
    ProcessGraphics() noexcept = default;
    ProcessGraphics(const ProcessGraphics&) = delete;
    ProcessGraphics& operator=(const ProcessGraphics&) = delete;

    // Binds the session's gfx queue, allocates and maps the private areas.
    // queue 0 selects the kernel gfx-ring route (config.kernel_ring required).
    memory::Status open(ProcessGraphicsConfig config, uint32_t queue) noexcept;
    bool opened() const noexcept { return opened_; }
    uint32_t queue() const noexcept { return queue_; }
    bool kernel_route() const noexcept { return kernel_route_; }
    // A kernel-ring job still waits for its fence (it cannot be preempted: the
    // owner must wait for it, or for the job timeout that quarantines it,
    // before the session's VM may be destroyed).
    bool kernel_pending() const noexcept;
    bool kernel_quarantined() const noexcept { return kernel_route_ && quarantined_; }
    // Bounded wait (device wait_us) for kernel-ring jobs; success when none is
    // left pending, busy on timeout, quarantined when one became uncertain.
    memory::Status wait_kernel_idle(uint32_t timeout_us) noexcept;

    // payload: the captured bytes after the process Request (SubmitHeader...).
    // success acknowledges submission; sequence is the queue fence to query.
    memory::Status submit(uint32_t queue, ByteView payload, uint32_t& sequence) noexcept;
    memory::Status query(uint32_t queue, uint32_t sequence) noexcept;
    memory::Status poll() noexcept;
    // Copies the clipped rectangles of an idle bgra8 buffer into the scanout and
    // waits for completion (bounded). busy when the buffer is still leased.
    memory::Status present(const graphics_abi::PresentRequest& request, uint32_t& copied) noexcept;
    bool idle() const noexcept;

    // queue: the recreated gfx queue id of the bound context (0: none bound).
    memory::Status rebind_after_reset(ProcessGraphicsConfig config, uint32_t queue) noexcept;
    bool ready_for_reset_rebind() const noexcept;
    // Inside a confirmed destructive reset window: every pending job is lost,
    // never replayed (Linux amdgpu_userq_pre_reset force-completes its fence);
    // its leases return. queue is the bound gfx queue, sequences the lost fence
    // values (0 for an unused job or an uncertain publication without one).
    // The private mappings died with the VM; the GTT areas are kept.
    memory::Status retire_after_reset(uint32_t& queue, uint32_t (&sequences)[graphics_jobs]) noexcept;
    // After a failed rebind_after_reset whose new VM the caller destroyed.
    void abandon_rebind() noexcept;
    memory::Status retire_removed_queue(uint32_t queue) noexcept;
    memory::Status close_after_idle() noexcept;
    bool empty() const noexcept { return !jobs_live_ && !shaders_live_ && !opened_; }

private:
    struct Job final {
        bool pending = false;
        uint32_t sequence = 0;
        uint32_t lease_count = 0;
        uint64_t leases[graphics_lease_capacity]{};
        uint32_t shader_mask = 0;
        // Kernel-ring route: the owner of this slot's ring receipt and its
        // shader-area lease (sequence is then the session-local number).
        opengl::PhysicalGraphicsSubmission kernel{};
    };
    struct ShaderSlot final {
        bool valid = false;
        opengl::gfx11_abi::ShaderKey key{};
        opengl::gfx11_abi::ProgramInfo vs{};
        opengl::gfx11_abi::ProgramInfo ps{};
        uint64_t last_use = 0;
    };
    struct SubmitContext;
    static bool submit_body(void* argument) noexcept;

    memory::Status map_private() noexcept;
    memory::Status bind_kernel_owners() noexcept;
    memory::Status release_job(Job& job) noexcept;
    memory::Status poll_job(Job& job) noexcept;
    Job* free_job() noexcept;
    memory::Status lease(Job& job, uint64_t handle, memory::Access access) noexcept;
    memory::Status programs(const opengl::gfx11_abi::ShaderKey& key, uint32_t job_bit,
                            opengl::gfx11::ShaderPrograms& output) noexcept;
    memory::Status build_submit(SubmitContext& context) noexcept;
    // Layout-4 surface/texture-level/pixel-source checks (before any lease).
    memory::Status validate_v4(const graphics_abi::SubmitHeader& header) noexcept;
    // The gfx11_abi v4 PS constant block of one validated state.
    void write_ps_constants(const graphics_abi::GraphicsState& state, uint32_t height,
                            uint8_t* output) noexcept;
    memory::Status emit_command(const graphics_abi::GraphicsCommand& command, uint64_t data_va,
                                const opengl::gfx11::DeviceRings& rings, uint32_t& shader_bits) noexcept;
    memory::Status publish(Job& job, uint32_t job_index, uint32_t dwords, uint32_t& sequence) noexcept;
    memory::Status publish_kernel(Job& job, uint32_t job_index, uint32_t dwords, uint32_t& sequence) noexcept;
    memory::Status poll_kernel(Job& job) noexcept;
    memory::Status retire_kernel_after_reset() noexcept;
    uint8_t* job_cpu(uint32_t index) const noexcept;

    ProcessGraphicsConfig config_{};
    memory::Handle jobs_{};
    memory::Handle shaders_{};
    uint8_t* jobs_cpu_ = nullptr;
    uint8_t* shaders_cpu_ = nullptr;
    bool jobs_live_ = false;
    bool jobs_retained_ = false;
    bool shaders_live_ = false;
    bool shaders_retained_ = false;
    bool opened_ = false;
    bool mapped_ = false;
    bool quarantined_ = false;
    bool kernel_route_ = false;
    uint32_t queue_ = 0;
    // Kernel-ring route: the last session-local sequence handed out.
    uint32_t kernel_issued_ = 0;
    uint64_t use_clock_ = 0;
    Job work_[graphics_jobs]{};
    ShaderSlot slots_[graphics_shader_slots]{};
};

} // namespace rx7900::reactos

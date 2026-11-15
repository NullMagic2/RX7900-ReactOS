/*
Module: rx7900.reactos.radeon_2d_ioctl
Purpose: Decode, authorize and validate private Radeon 2D IOCTLs, translate
opaque handles into engine surfaces and drive a bound 2D backend.
Created: 2026-10-22
Architecture: Portable, allocation-free miniport core (host-tested with a fake
backend). rx7900video owns one Dispatcher in its device extension and binds the
Blit2dBackend table to the SDMA 2D engine (src/hardware/blit2d). The optional
VideoPort glue at the end of this header is compiled only for the native
miniport (RX7900_RADEON2D_VIDEOPORT) and adds the PASSIVE_LEVEL mutex boundary.
No request names a GPU, physical or virtual address: surfaces are handles owned
by one session, staging transfers are offsets inside that session's allocation.
*/
#pragma once

#include "radeon_2d_abi.hpp"

#include <stddef.h>
#include <stdint.h>

namespace rx7900::reactos::radeon2d {

// ---------------------------------------------------------------------------
// Backend contract (bound by root to src/hardware/blit2d).
// ---------------------------------------------------------------------------

// ok: the operation was accepted (fence > 0 returned) or completed.
// rejected: nothing was submitted and no GPU or memory write occurred (queue
//   full, unsupported geometry, allocation failure). The caller may retry/fallback.
// uncertain: the GPU may or may not have executed some of the work (submission
//   fault, hang, timeout, lost device). The dispatcher poisons the session.
enum class BackendStatus : uint32_t { ok = 0, rejected = 1, uncertain = 2 };

// A GPU-addressable XRGB8888 surface: miniport primary or backend allocation.
// gpu_address is the backend's address of pixel (0, 0) in its own address
// space (MC/VMID0 VRAM address for the SDMA kernel ring); it never leaves the
// miniport. pitch_bytes >= width * 4, pitch_bytes * height fits the allocation.
struct EngineSurface final {
    uint64_t gpu_address = 0;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t pitch_bytes = 0;
    uint32_t format = abi::format_xrgb8888;
};

// Validated half-open pixel rectangle wholly inside the referenced surface.
struct EngineRect final {
    uint32_t x = 0;
    uint32_t y = 0;
    uint32_t width = 0;
    uint32_t height = 0;
};

// Destination rectangle and its equally sized source origin, both inside their
// surfaces. Lists are delivered in a dependency-safe order (see copy below).
struct CopyRect final {
    EngineRect destination{};
    uint32_t source_x = 0;
    uint32_t source_y = 0;
};

// One rectangle of an upload (staging -> surface) or readback (surface ->
// staging). staging_offset is the byte offset of the rectangle's first pixel in
// the session staging allocation; rows advance by the request pitch. The
// dispatcher proved staging_offset + (height - 1) * pitch + width * 4 <= bytes.
struct TransferRect final {
    EngineRect surface{};
    uint32_t staging_offset = 0;
};

// A GPU-visible, CPU-mapped buffer allocated by the backend for one session.
// cpu_address is the miniport's kernel mapping handed to the display DLL;
// gpu_address is what the backend's DMA engine uses. Both stay valid until
// staging_destroy. bytes is a multiple of four.
struct StagingBuffer final {
    uint64_t gpu_address = 0;
    uint64_t cpu_address = 0;
    uint32_t bytes = 0;
    uint32_t allocation = 0;
};

// A raster operand resolved by the dispatcher: an owned surface (format
// xrgb8888) or a region of the session's staging allocation (format per
// abi::format_*), both as engine memory. surface.format carries the abi format;
// pitch >= 4 * width and pitch * height bytes lie inside the allocation.
struct RasterOperand final {
    bool present = false;
    EngineSurface surface{};
    // Translated staging formats (abi::translated_format): GPU address of the
    // XLATEOBJ table that precedes surface (abi::staging_table_bytes), else 0.
    uint64_t table_address = 0;
};

// Compute raster operations (abi version 3). Rectangles are validated against
// target and bounds; operands against their allocations; the map against
// abi::SourceMap rules. pattern is 64 BGRX dwords or null for a solid brush.
struct RasterRop final {
    EngineSurface target{};
    abi::Rect bounds{};
    uint32_t rop4 = 0;
    uint32_t flags = 0;
    uint32_t brush_colour = 0;
    int32_t brush_x = 0;
    int32_t brush_y = 0;
    uint32_t mono_foreground = 0;
    uint32_t mono_background = 0;
    uint32_t colour_key = 0;
    RasterOperand source{};
    abi::SourceMap map{};
    RasterOperand mask{};
    int32_t mask_x = 0;
    int32_t mask_y = 0;
    const uint32_t* pattern = nullptr;
    // 0xa17 (one EngStretchBltROP DIB call): a mirrored axis samples
    // map.left + map.width - q instead of map.left + q, q = trunc((x -
    // bounds.left) * map.width / bounds width) with a possibly negative width; the source operand is
    // already cropped to the request's sample window (map in window
    // coordinates); stretch_mask reads the mask at (sx + mask_x, sy + mask_y)
    // of every sample and skips set or outside samples, and only the low ROP3
    // byte is evaluated. Other codes leave these false.
    bool mirror_x = false;
    bool mirror_y = false;
    bool stretch_mask = false;
    // 0xa17 stretch_flag_running_phase: masked tile PatternX.
    bool running_phase = false;
};

// AlphaBlend (source xrgb8888) or coverage8 glyph composition (source coverage8
// with colour).
struct RasterAlpha final {
    EngineSurface target{};
    abi::Rect bounds{};
    RasterOperand source{};
    abi::SourceMap map{};
    uint32_t constant_alpha = 255;
    bool source_alpha = false;
    bool coverage8 = false;
    uint32_t colour = 0;
};

struct RasterLine final {
    EngineSurface target{};
    int32_t x0 = 0;
    int32_t y0 = 0;
    int32_t x1 = 0;
    int32_t y1 = 0;
    uint32_t colour = 0;
    uint32_t rop3 = 0xf0;
    bool styled = false;
    uint32_t style_bit_count = 0;
    const uint8_t* style = nullptr;   // abi::style_bytes, MSB-first
    // 64 BGRX dwords (8x8, row-major) as P instead of colour, or null.
    const uint32_t* pattern = nullptr;
    int32_t brush_x = 0;
    int32_t brush_y = 0;
};

struct RasterGradient final {
    EngineSurface target{};
    uint32_t mode = abi::gradient_rect_h;
    abi::Rect extents{};
    abi::Vertex vertices[3]{};
};

// Every callback runs at PASSIVE_LEVEL inside the dispatcher's caller-held lock,
// receives only validated values and borrowed arrays valid for the call, and
// must not retain those arrays. All work executes on one ordered queue (or the
// backend enforces equivalent read/write hazards): an operation observes the
// results of every operation submitted before it. A successful submission
// returns a fence > 0 that is monotonically nondecreasing across all calls;
// completion of fence N implies completion of every fence below N. Clip lists
// above abi::max_rects arrive as consecutive requests; this ordering (with the
// backend's read/write barriers across IBs) is what keeps them exact. Accepted
// work must be submitted to the GPU before the callback returns (for
// Blit2dEngine: call flush()), because scanout shows the primary without any
// later sync. Memory coherence is the backend's job: CPU writes through the
// BAR (Eng* fallbacks on the primary) must be visible to the next submission
// (HDP flush before the IB) and GPU writes must be visible to CPU BAR and
// staging reads before sync reports ok (HDP invalidate; snooped GTT staging).
//
// Binding to src/hardware/blit2d (root adapter): keep an operation counter as
// the fence; map Blit2dStatus success -> ok; refused-before-encoding statuses
// (invalid_*, out_of_bounds, too_many_rects, overlapping_surfaces,
// unordered_overlap, engine_memory_overlap, invalid_staging, memory_failure,
// busy) -> rejected; faulted, submit_failed, fence_timeout, fence_failed ->
// uncertain. copy: one Blit2dCopy whose destination_rect is the union of the
// rectangles and whose source_point applies their common offset, with the
// rectangles as its clip list. upload/readback: one engine call per
// TransferRect with Blit2dStaging{handle, staging_offset, pitch, width,
// height} at staging_point (0, 0). sync: Blit2dEngine::synchronize, then
// completed = the counter value observed before it.
struct Blit2dBackend final {
    void* context = nullptr;
    // Solid 32-bit fill of each rectangle (the exact value, high byte included).
    BackendStatus (*fill)(void* context, const EngineSurface& target, uint32_t color,
                          const EngineRect* rects, uint32_t count,
                          uint64_t& fence) noexcept = nullptr;
    // Execute rectangles strictly in array order. Each single rectangle must have
    // memmove semantics when target and source alias and the two areas overlap.
    // The dispatcher has ordered the list so no earlier destination overwrites a
    // later source; target and source are identical or disjoint allocations.
    BackendStatus (*copy)(void* context, const EngineSurface& target, const EngineSurface& source,
                          const CopyRect* rects, uint32_t count, uint64_t& fence) noexcept = nullptr;
    BackendStatus (*upload)(void* context, const EngineSurface& target, const StagingBuffer& staging,
                            uint32_t staging_pitch, const TransferRect* rects, uint32_t count,
                            uint64_t& fence) noexcept = nullptr;
    // GPU writes staging; the dispatcher calls sync(fence) before replying, so
    // the CPU reads staging only after completion (and any needed cache flush,
    // which the backend's sync must include for its staging memory type).
    BackendStatus (*readback)(void* context, const EngineSurface& source, const StagingBuffer& staging,
                              uint32_t staging_pitch, const TransferRect* rects, uint32_t count,
                              uint64_t& fence) noexcept = nullptr;
    // Allocate a GPU-only XRGB8888 VRAM surface. allocation is the backend's id.
    BackendStatus (*surface_create)(void* context, uint32_t width, uint32_t height, uint32_t format,
                                    EngineSurface& output, uint32_t& allocation) noexcept = nullptr;
    // Called only after sync proved every fence that referenced it retired.
    void (*surface_destroy)(void* context, uint32_t allocation) noexcept = nullptr;
    BackendStatus (*staging_create)(void* context, uint32_t bytes, StagingBuffer& output) noexcept = nullptr;
    // Called only after sync proved every fence that referenced it retired.
    void (*staging_destroy)(void* context, const StagingBuffer& staging) noexcept = nullptr;
    // Wait (bounded by the backend's own timeout) until fence has retired.
    // ok = retired; any other status means retirement is unproven.
    BackendStatus (*sync)(void* context, uint64_t fence) noexcept = nullptr;
    // Highest fence the backend has observed retired (no waiting).
    uint64_t (*completed)(void* context) noexcept = nullptr;
    // Eviction (Linux amdgpu_device_evict_resources before suspend): move the
    // contents of a live offscreen surface from VRAM into backend-owned system
    // memory that stays GPU-addressable and unchanged across suspend; output is
    // the surface at its new placement (same size, pitch and format). Called only
    // after sync proved the surface idle. ok: moved (or already in system
    // memory) and the VRAM is free; rejected: nothing changed (no system memory),
    // the caller then destroys the surface and reports its contents lost;
    // uncertain: as for every other callback.
    BackendStatus (*surface_evict)(void* context, uint32_t allocation,
                                   EngineSurface& output) noexcept = nullptr;
    // After the backend restarted: move an evicted surface back into VRAM.
    // rejected leaves it usable in system memory (output unchanged).
    BackendStatus (*surface_restore)(void* context, uint32_t allocation,
                                     EngineSurface& output) noexcept = nullptr;
    // Optional compute raster engine. raster_operations is the subset of
    // abi::raster_operations it implements (0 without a compute engine; the
    // callbacks are then unused and may be null). Each executes the operation
    // over every rectangle in array order with the same fence contract as the
    // SDMA callbacks; the backend orders it against earlier SDMA and compute
    // work that touches the same memory.
    uint64_t raster_operations = 0;
    BackendStatus (*raster_rop)(void* context, const RasterRop& operation, const EngineRect* rects,
                                uint32_t count, uint64_t& fence) noexcept = nullptr;
    BackendStatus (*raster_alpha)(void* context, const RasterAlpha& operation, const EngineRect* rects,
                                  uint32_t count, uint64_t& fence) noexcept = nullptr;
    BackendStatus (*raster_line)(void* context, const RasterLine& operation, const EngineRect* rects,
                                 uint32_t count, uint64_t& fence) noexcept = nullptr;
    BackendStatus (*raster_gradient)(void* context, const RasterGradient& operation,
                                     const EngineRect* rects, uint32_t count, uint64_t& fence) noexcept = nullptr;
};

// Accounting limits of one configuration. Offscreen bytes are pitch * height of
// every live offscreen surface; creation beyond either budget is refused (busy)
// and the display DLL then keeps that bitmap in an engine DIB.
struct Limits final {
    uint32_t staging_bytes = 0;
    uint64_t session_surface_bytes = abi::default_session_surface_bytes;
    uint64_t device_surface_bytes = abi::default_device_surface_bytes;
};

// ---------------------------------------------------------------------------
// Dispatcher.
// ---------------------------------------------------------------------------
enum class DispatchStatus : uint32_t {
    success,            // reply() holds reply_bytes (possibly zero) valid bytes.
    invalid_parameter,  // Malformed, unauthorized or out-of-bounds; GPU untouched.
    insufficient_buffer,// Output capacity too small; checked before any execution.
    not_ready,          // Not configured (no adopted primary or backend).
    busy,               // Resource exhausted (sessions/surfaces); nothing changed.
    unknown_code        // Not a Radeon 2D IOCTL.
};

struct DispatchResult final {
    DispatchStatus status = DispatchStatus::invalid_parameter;
    uint32_t reply_bytes = 0;
};

struct Counters final {
    uint32_t accepted = 0;
    uint32_t rejected = 0;
    // Offscreen surfaces moved out of VRAM by suspend, moved back by resume,
    // and destroyed because no system memory could hold them (contents lost).
    uint32_t evicted = 0;
    uint32_t restored = 0;
    uint32_t lost = 0;
    // Destructive-reset recovery: completed recoveries, offscreen surfaces whose
    // contents survived exactly (system memory, no unproven write) and those
    // lost (VRAM-resident, or an unproven write in flight), open sessions whose
    // poisoning the reset proof resolved, and retained (quarantined) allocations
    // the reset proof released. lost_reported counts outcome_surface_lost
    // answers (one per lost handle).
    uint32_t recoveries = 0;
    uint32_t reset_restored = 0;
    uint32_t reset_lost = 0;
    uint32_t revived_sessions = 0;
    uint32_t released_quarantine = 0;
    uint32_t lost_reported = 0;
};

class Dispatcher final {
public:
    // Bind the backend and the adopted primary scanout. Requires a complete
    // backend table and a valid primary surface. seed initializes the session
    // key generator (native: KeQueryPerformanceCounter). Fails if already bound.
    bool configure(const Blit2dBackend& backend, const EngineSurface& primary,
                   uint32_t staging_bytes, uint32_t seed) noexcept;
    // Same with explicit offscreen budgets (limits.staging_bytes as above).
    bool configure(const Blit2dBackend& backend, const EngineSurface& primary,
                   const Limits& limits, uint32_t seed) noexcept;

    // Retire everything: syncs each session's work, destroys proven-idle
    // surfaces/staging and invalidates all handles. Allocations whose
    // retirement cannot be proven are retained (leaked, never reused) and the
    // function returns false. Afterwards the dispatcher is unconfigured.
    bool shutdown() noexcept;

    bool configured() const noexcept { return configured_; }

    // Before the backend stops for a power transition. Proves every session
    // idle, then evicts every live offscreen surface (surface_evict); a surface
    // the backend cannot save is destroyed and its handle answers
    // outcome_surface_lost until destroyed. Sessions, keys, staging and handles
    // stay valid; every IOCTL except query answers not_ready until resume.
    // false: idleness or an eviction could not be proven (the affected sessions
    // are poisoned); the caller must use shutdown() instead of resume().
    bool suspend() noexcept;
    bool suspended() const noexcept { return suspended_; }

    // After the backend restarted (same allocations, new table allowed): binds
    // the re-adopted primary (same geometry keeps its handle and owner, any
    // other geometry issues a new handle), restores evicted surfaces to VRAM
    // where the backend can and resumes dispatch. false leaves it suspended.
    bool resume(const Blit2dBackend& backend, const EngineSurface& primary) noexcept;

    // Destructive-reset recovery (Linux amdgpu_device_gpu_recover around a
    // mode-1 reset), driven by the device owner at PASSIVE_LEVEL.
    // begin_recovery quiesces dispatch (every IOCTL except query answers
    // not_ready) and marks work that is not proven retired uncertain: every
    // surface written by a fence above the backend's completed() at this point.
    // Nothing is synchronized or freed (the GPU may be hung). Allowed while
    // configured, also while suspended; false otherwise.
    bool begin_recovery() noexcept;
    bool recovering() const noexcept { return recovering_; }
    // After the reset proof, the backend's park_after_reset and its restart on
    // the same owners: binds the backend and the re-validated primary (same
    // geometry keeps its handle and owner). VRAM-resident offscreen surfaces
    // and system-memory ones with an uncertain write are lost (answered
    // exactly once with outcome_surface_lost); untouched system-memory surfaces
    // are restored to VRAM where the backend can (otherwise they stay usable in
    // system memory) with their exact contents. Open sessions survive with
    // their handles, keys and staging; their poisoning is resolved by the reset
    // proof. Quarantined surfaces and sessions are released. false: a restore
    // was uncertain (a second fault) or the call was out of order; the caller
    // then disables 2D.
    bool complete_recovery(const Blit2dBackend& backend, const EngineSurface& primary) noexcept;
    // query reports abi::recovery_pending between begin_recovery and the end of
    // the recovery (the owner's recovery thread may run it later).
    // Sticky for the device lifetime: every IOCTL except query answers
    // not_ready and the backend is never called again (sessions, staging and
    // surfaces stay allocated: the display DLL may still hold staging views).
    // quiescent: no GPU work issued through this dispatcher can still execute.
    void disable(bool quiescent) noexcept;
    bool disabled() const noexcept { return disabled_; }
    uint32_t recovery_generation() const noexcept { return recovery_generation_; }

    // Caller holds the device lock at PASSIVE_LEVEL. input is the complete
    // METHOD_BUFFERED input; output_capacity is the OutputBufferLength. All
    // input is decoded into device-owned storage before anything executes, so
    // the I/O manager's aliased system buffer may be overwritten afterwards
    // with exactly reply_bytes from reply().
    DispatchResult dispatch(uint32_t code, ByteView input, uint32_t output_capacity) noexcept;

    ByteView reply() const noexcept { return ByteView(reply_, reply_bytes_); }
    Counters counters() const noexcept { return counters_; }

private:
    struct Session final {
        bool open = false;
        bool poisoned = false;
        // Set by complete_recovery: the session survived a destructive reset.
        bool revived = false;
        // Retirement could not be proven at close: slot is never reused.
        bool quarantined = false;
        uint32_t generation = 0;
        uint32_t key = 0;
        uint32_t surfaces = 0;
        uint64_t surface_bytes = 0;
        uint64_t last_fence = 0;
        StagingBuffer staging{};
    };

    struct Surface final {
        bool used = false;
        bool primary = false;
        bool quarantined = false;
        // Moved to system memory by suspend (restore pending).
        bool evicted = false;
        // Contents lost in eviction or a destructive reset; no backend
        // allocation remains. Answered once with outcome_surface_lost, then freed.
        bool lost = false;
        // A write not proven retired when begin_recovery ran (or an uncertain
        // eviction move).
        bool uncertain = false;
        // Quarantined by surface_destroy: the handle and session share are gone.
        bool destroyed = false;
        uint32_t generation = 0;
        uint32_t owner = 0;
        uint32_t allocation = 0;
        uint64_t bytes = 0;
        uint64_t last_use = 0;
        // Last fence that wrote the surface (reads do not change contents).
        uint64_t last_write = 0;
        EngineSurface engine{};
    };

    DispatchResult query(ByteView input, uint32_t capacity) noexcept;
    DispatchResult open_session(ByteView input, uint32_t capacity) noexcept;
    DispatchResult close_session(ByteView input, uint32_t capacity) noexcept;
    DispatchResult assert_primary(ByteView input, uint32_t capacity) noexcept;
    DispatchResult fill(ByteView input, uint32_t capacity) noexcept;
    DispatchResult copy(ByteView input, uint32_t capacity) noexcept;
    DispatchResult transfer(ByteView input, uint32_t capacity, bool upload) noexcept;
    DispatchResult surface_create(ByteView input, uint32_t capacity) noexcept;
    DispatchResult surface_destroy(ByteView input, uint32_t capacity) noexcept;
    DispatchResult sync(ByteView input, uint32_t capacity) noexcept;
    DispatchResult raster(uint32_t code, ByteView input, uint32_t capacity) noexcept;
    DispatchResult raster_rop(uint32_t code, ByteView input, uint32_t capacity) noexcept;
    DispatchResult raster_text(ByteView input, uint32_t capacity) noexcept;
    DispatchResult raster_alpha(ByteView input, uint32_t capacity) noexcept;
    DispatchResult raster_line(ByteView input, uint32_t capacity) noexcept;
    DispatchResult raster_gradient(ByteView input, uint32_t capacity) noexcept;
    DispatchResult raster_stretch_rop(ByteView input, uint32_t capacity) noexcept;
    // Authorizes the session, resolves the target and converts the rectangles
    // (inside the target, pairwise disjoint). false: rejected (invalid_parameter).
    bool raster_target(const abi::Header& header, uint32_t handle, const abi::RectList& rects,
                       Session*& session, Surface*& target) noexcept;
    // An operand of session: false when invalid. surface is set for handles
    // (lost surfaces resolve; the caller answers surface_lost).
    bool resolve_operand(const abi::Operand& wire, const Session& session, RasterOperand& output,
                         Surface*& surface) noexcept;
    // Receipt of an executed raster request (fence bookkeeping as fill/copy).
    DispatchResult raster_receipt(uint32_t capacity, Session& session, Surface& target, Surface* other,
                                  BackendStatus status, uint64_t fence, Surface* mask = nullptr) noexcept;
    Session* authorize(const abi::Header& header) noexcept;
    uint32_t session_index(const Session& session) const noexcept;
    Surface* resolve(uint32_t handle, const Session& session) noexcept;
    bool convert_rects(const abi::RectList& list, const EngineSurface& surface) noexcept;
    bool order_copies(bool aliased) noexcept;
    bool accept_fence(Session& session, BackendStatus status, uint64_t fence) noexcept;
    bool retire_session(Session& session) noexcept;
    void release_surface(Surface& surface, Session& owner) noexcept;
    uint32_t surface_index(const Surface& surface) const noexcept;
    // The first lost surface among the operands (nullptr when none).
    static Surface* lost_operand(Surface* first, Surface* second, Surface* third) noexcept;
    // outcome_surface_lost naming surface; retires the handle (exactly once).
    DispatchResult lost_receipt(uint32_t capacity, Surface& surface) noexcept;
    // complete_recovery for one offscreen surface. false: an uncertain restore.
    bool recover_surface(Surface& surface) noexcept;
    DispatchResult receipt(uint32_t capacity, uint32_t outcome, uint64_t fence,
                           uint32_t lost_surface = 0) noexcept;
    DispatchResult reject(DispatchStatus status) noexcept;
    uint32_t next_key() noexcept;
    uint64_t completed() const noexcept;

    Blit2dBackend backend_{};
    bool configured_ = false;
    bool suspended_ = false;
    bool recovering_ = false;
    bool disabled_ = false;
    uint32_t recovery_state_ = abi::recovery_operational;
    uint32_t recovery_generation_ = 0;
    // completed() when begin_recovery ran: the last fence proven retired.
    uint64_t recovery_completed_ = 0;
    uint32_t staging_bytes_ = 0;
    uint64_t session_surface_bytes_ = 0;
    uint64_t device_surface_bytes_ = 0;
    uint64_t used_surface_bytes_ = 0;
    uint32_t key_state_ = 0;
    uint64_t last_fence_ = 0;
    // Index into sessions_ of the primary owner, or max_sessions when unowned.
    uint32_t primary_owner_ = abi::max_sessions;
    Counters counters_{};
    Session sessions_[abi::max_sessions]{};
    Surface surfaces_[abi::max_surfaces]{};
    // Device-owned decode/translation workspace: keeps kilobyte-sized request
    // state off the shallow NT kernel stack. Serialized by the caller's lock.
    abi::FillCommand fill_command_{};
    abi::CopyCommand copy_command_{};
    abi::TransferCommand transfer_command_{};
    abi::RopCommand rop_command_{};
    abi::TextCommand text_command_{};
    abi::AlphaCommand alpha_command_{};
    abi::LineCommand line_command_{};
    abi::GradientCommand gradient_command_{};
    abi::StretchRopCommand stretch_rop_command_{};
    EngineRect engine_rects_[abi::max_rects]{};
    CopyRect copy_input_[abi::max_rects]{};
    CopyRect copy_rects_[abi::max_rects]{};
    TransferRect transfer_rects_[abi::max_rects]{};
    uint8_t reply_[abi::max_reply_bytes]{};
    uint32_t reply_bytes_ = 0;
};

// Pure helpers exposed for tests and the display DLL's own admission checks.
// Rectangle inside [0,width) x [0,height), non-empty, using 64-bit arithmetic.
bool rect_inside(const abi::Rect& rect, uint32_t width, uint32_t height) noexcept;
bool rects_intersect(const abi::Rect& a, const abi::Rect& b) noexcept;

} // namespace rx7900::reactos::radeon2d

#if defined(RX7900_RADEON2D_VIDEOPORT)
// Native miniport boundary. Include video_sdk.hpp before this header.
namespace rx7900::reactos::radeon2d {

// Lives in the VideoPort device extension, constructed once with the rest of
// the extension. initialize() runs at PASSIVE_LEVEL before the first IOCTL.
struct NativeDispatcher final {
    KMUTEX lock;
    Dispatcher dispatcher;
    bool initialized;
};

void native_initialize(NativeDispatcher& native) noexcept;

// HwStartIO hook for codes where abi::is_radeon_2d_ioctl(code) is true. Requires
// PASSIVE_LEVEL (returns ERROR_INVALID_FUNCTION otherwise), takes the mutex,
// decodes METHOD_BUFFERED input before writing output, returns only initialized
// reply bytes and never retains the packet's buffers.
VP_STATUS native_start_io(NativeDispatcher& native, VIDEO_REQUEST_PACKET& packet) noexcept;

// Lock-holding wrappers for root's configure/teardown paths (PASSIVE_LEVEL).
bool native_configure(NativeDispatcher& native, const Blit2dBackend& backend,
                      const EngineSurface& primary, uint32_t staging_bytes) noexcept;
bool native_shutdown(NativeDispatcher& native) noexcept;
// Power-transition wrappers (PASSIVE_LEVEL, under the mutex): Dispatcher::suspend
// before the backend parks, Dispatcher::resume after it restarted.
bool native_suspend(NativeDispatcher& native) noexcept;
bool native_resume(NativeDispatcher& native, const Blit2dBackend& backend,
                   const EngineSurface& primary) noexcept;

// Holds the dispatcher mutex for a multi-step owner sequence (destructive-reset
// recovery), PASSIVE_LEVEL; the KMUTEX is recursive, so dispatcher wrappers may
// be called inside. native must be initialized.
class NativeGuard final {
public:
    explicit NativeGuard(NativeDispatcher& native) noexcept : lock_(native.lock) {
        KeWaitForSingleObject(&lock_, Executive, KernelMode, FALSE, nullptr);
    }
    ~NativeGuard() noexcept { KeReleaseMutex(&lock_, FALSE); }
    NativeGuard(const NativeGuard&) = delete;
    NativeGuard& operator=(const NativeGuard&) = delete;
private:
    KMUTEX& lock_;
};

} // namespace rx7900::reactos::radeon2d
#endif

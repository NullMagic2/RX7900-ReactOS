/*
Module: rx7900.hardware.blit2d
Purpose: Physical Navi31 2D engine on SDMA 6.0: 32-bpp linear rectangle fills,
rectangle copies (including same-surface overlap), staging uploads and readback.
Created: 2026-10-22
Architecture: Pure validation, ordering and packet encoding plus Blit2dEngine, which
writes indirect buffers (IBs) into caller-provided memory::Manager allocations and
hands them to a caller-supplied submitter (normally an adapter over Sdma6::submit and
FenceManager query/wait). No MMIO, OS service, heap or Sdma6 object is used here.
Copyright 2020, 2021 Advanced Micro Devices, Inc. Adapted Linux v6.12
sdma_v6_0_0_pkt_open.h (COPY_LINEAR_SUBWIN, CONSTANT_FILL, NOP layouts) and
sdma_v6_0.c (sdma_v6_0_emit_fill_buffer, sdma_v6_0_ring_pad_ib).
Copyright 2015-2021 Advanced Micro Devices, Inc.; Copyright 2023 Valve Corporation.
Adapted Mesa 24.2.0 radv_sdma.c (radv_sdma_emit_copy_linear_sub_window minus-one
field encoding, radv_sdma_check_pitches limits, radv_sdma_fill_buffer dword fill
size, radv_sdma_emit_nop wait semantics) and si_sdma_copy_image.c (MIT).
Notice: licenses/AMDGPU-SDMA.txt. Pinned copies: work/upstream-sdma and
work/upstream-gpuinit/mesa-24.2.0 (SHA256SUMS).
*/
#pragma once

#include "sdma6_init.hpp"

namespace rx7900::hardware {

constexpr uint32_t blit2d_bytes_per_pixel = 4;
// COPY_LINEAR_SUBWIN HEADER elementsize is log2(bytes per element).
constexpr uint32_t blit2d_element_size_log2 = 2;

// Clip lists: the GDI path enumerates CLIPOBJ rectangles in batches of 64. More
// rectangles in one operation are refused (too_many_rects); the caller splits.
constexpr uint32_t blit2d_max_rects = 64;

// COPY_LINEAR_SUBWIN DW_3/DW_8 src_x, src_y, dst_x, dst_y: 14-bit fields.
constexpr uint32_t blit2d_max_coordinate = 0x3FFFu;
// DW_11 rect_x/rect_y hold width - 1 and height - 1 in 14 bits.
constexpr uint32_t blit2d_max_extent = 0x4000u;
// DW_4/DW_9 src_pitch/dst_pitch hold pitch - 1 (elements) in 19 bits on SDMA 6,
// but Mesa radv_sdma_check_pitches and si_sdma_copy_image keep pitch <= 2^14
// elements (the smallest generation limit). This module keeps the Mesa limit.
constexpr uint32_t blit2d_pitch_field_limit = 0x80000u;
constexpr uint32_t blit2d_max_pitch_elements = 0x4000u;
// DW_5/DW_10 slice pitch - 1 (elements) in 28 bits.
constexpr uint32_t blit2d_max_slice_elements = 0x10000000u;
// Surfaces are bounded so that every in-surface coordinate and extent fits the
// fields above; 16384 x 16384 is also the GFX11 2D image limit.
constexpr uint32_t blit2d_max_surface_dimension = blit2d_max_extent;
static_assert(blit2d_max_surface_dimension - 1 == blit2d_max_coordinate,
              "every in-surface coordinate fits the 14-bit x/y fields");
static_assert(uint64_t{blit2d_max_pitch_elements} * blit2d_max_surface_dimension <=
              blit2d_max_slice_elements, "slice pitch of any valid surface fits 28 bits");
static_assert(blit2d_max_pitch_elements <= blit2d_pitch_field_limit, "pitch policy fits field");

// CONSTANT_FILL COUNT is bytes - 1 in 30 bits; fills split at the pinned Linux
// fill_max_bytes policy (sdma::max_transfer_bytes, 4 MiB), well inside the field.
constexpr uint32_t blit2d_fill_count_field_limit = 0x40000000u;
static_assert(sdma::max_transfer_bytes <= blit2d_fill_count_field_limit, "fill policy fits field");

constexpr uint32_t blit2d_sub_window_dwords = 13;
constexpr uint32_t blit2d_constant_fill_dwords = 5;
constexpr uint32_t blit2d_nop_dwords = 1;

// IB ring of the engine: 2..16 slots of one caller allocation, each slot a
// multiple of sdma6_ib_alignment and at least 256 bytes (the Linux IB pool granule).
constexpr uint32_t blit2d_min_ib_slots = 2;
constexpr uint32_t blit2d_max_ib_slots = 16;
constexpr uint32_t blit2d_min_ib_bytes = 256;
// Largest IB, a multiple of sdma6_ib_dword_multiple inside the 20-bit IB_SIZE.
constexpr uint32_t blit2d_max_ib_dwords =
    sdma6_max_ib_dwords - sdma6_max_ib_dwords % sdma6_ib_dword_multiple;
static_assert(blit2d_min_ib_bytes / sizeof(uint32_t) >=
              blit2d_sub_window_dwords + blit2d_nop_dwords + sdma6_ib_dword_multiple,
              "a minimum IB holds the largest packet, a barrier and padding");

// Regions tracked between two barriers; a full tracker forces a barrier.
constexpr uint32_t blit2d_hazard_capacity = 64;
// Default fence wait budget per IB: the bare-metal SDMA job timeout.
constexpr uint64_t blit2d_default_wait_us = uint64_t{fence_sdma_timeout_ms} * 1000;

enum class Blit2dStatus {
    success,
    not_initialized,
    invalid_argument,
    // Address/pitch alignment, dimensions or a field limit of a surface.
    invalid_surface,
    // A rectangle with right < left or bottom < top.
    invalid_rect,
    // More than blit2d_max_rects clip rectangles.
    too_many_rects,
    // A destination or mapped source rectangle leaves its surface.
    out_of_bounds,
    // Distinct surface descriptors share bytes (only identical ones may alias).
    overlapping_surfaces,
    // Same-surface clip rectangles have no order in which every source is read
    // before an earlier rectangle overwrites it; the caller must use another path.
    unordered_overlap,
    // An operation touches an IB buffer or the scratch allocation.
    engine_memory_overlap,
    // A staging handle failed describe, is not CPU visible or lacks access.
    invalid_staging,
    // Manager retain/describe failed for an engine allocation.
    memory_failure,
    // shutdown with unsubmitted or incomplete IBs.
    busy,
    // Sticky: a submission, wait or fence failed; only release_after_reset recovers.
    // report().last_failure then holds one of the three reasons below.
    faulted,
    // The submitter returned failed, or stayed busy with nothing of ours in flight.
    submit_failed,
    // A wait returned without the fence signalling within wait_timeout_us.
    fence_timeout,
    // query or wait reported a failed (hung or detached) ring.
    fence_failed
};

// Inclusive-exclusive pixel rectangle, the RECTL convention.
struct Blit2dRect final {
    int32_t left;
    int32_t top;
    int32_t right;
    int32_t bottom;
};

struct Blit2dPoint final {
    int32_t x;
    int32_t y;
};

// A 32-bpp linear surface in the SDMA's VMID0 address space (VRAM or GART-mapped
// system memory). pitch_bytes is a multiple of 4 with width <= pitch_bytes / 4 <=
// blit2d_max_pitch_elements; width and height are 1..16384; gpu_address is 4-byte
// aligned and the surface ends inside sdma::gpu_virtual_address_max. The caller
// keeps the memory mapped and resident until the IBs that reference it complete.
struct Blit2dSurface final {
    uint64_t gpu_address;
    uint32_t pitch_bytes;
    uint32_t width;
    uint32_t height;
};

// A CPU-visible staging rectangle inside a memory::Manager allocation (GTT).
// Upload requires GPU read access, readback GPU write access. The engine resolves
// the GPU address with Manager::describe; the caller writes (upload) or reads
// (readback, after synchronize) the bytes through the allocation's CPU backing.
struct Blit2dStaging final {
    memory::Handle allocation;
    uint64_t byte_offset;
    uint32_t pitch_bytes;
    uint32_t width;
    uint32_t height;
};

// Clip rectangles. count 0 means unclipped (the operation's bounds only);
// otherwise every rectangle is intersected with the bounds and empty results are
// skipped. count above blit2d_max_rects is refused.
struct Blit2dClipList final {
    Blit2dRect rects[blit2d_max_rects];
    uint32_t count;
};

struct Blit2dFill final {
    Blit2dSurface destination;
    Blit2dRect bounds;
    uint32_t color;
};

// Copies destination_rect from source at source_point (the pixel mapped to the
// rectangle's top-left corner). The result equals copying from a snapshot of the
// source taken before the operation, also when source and destination are the same
// surface and overlap (GDI DrvCopyBits/DrvBitBlt SRCCOPY semantics).
struct Blit2dCopy final {
    Blit2dSurface source;
    Blit2dSurface destination;
    Blit2dRect destination_rect;
    Blit2dPoint source_point;
};

// One COPY_LINEAR_SUBWIN packet in element (pixel) units, before the minus-one
// encoding: x/y <= blit2d_max_coordinate, pitch 1..blit2d_max_pitch_elements,
// slice 1..blit2d_max_slice_elements, width/height 1..blit2d_max_extent, depth 1.
struct Blit2dSubWindow final {
    uint64_t source_address;
    uint32_t source_x;
    uint32_t source_y;
    uint32_t source_pitch_elements;
    uint32_t source_slice_elements;
    uint64_t destination_address;
    uint32_t destination_x;
    uint32_t destination_y;
    uint32_t destination_pitch_elements;
    uint32_t destination_slice_elements;
    uint32_t width;
    uint32_t height;
};

// Packet encoders. Each validates every field against its header width before
// writing, writes little-endian dwords at byte_offset through the checked
// CommandBuffer and leaves the output unchanged on failure (invalid_argument).
//
// COPY_LINEAR_SUBWIN, 13 dwords:
//   DW0  op 1 [7:0] | sub_op 4 [15:8] | tmz 0 [18] | cpv 0 [19] | elementsize 2 [31:29]
//   DW1-2 src address; DW3 src_x [13:0] | src_y [29:16]
//   DW4  src_z 0 [12:0] | (src_pitch - 1) [31:13]; DW5 (src_slice_pitch - 1) [27:0]
//   DW6-7 dst address; DW8 dst_x | dst_y; DW9 dst_z 0 | (dst_pitch - 1) [31:13]
//   DW10 (dst_slice_pitch - 1); DW11 (width - 1) [13:0] | (height - 1) [29:16]
//   DW12 (depth - 1) = 0 [12:0], dst/src swap and cache policy 0.
Blit2dStatus encode_sub_window_copy(const Blit2dSubWindow& copy, sdma::CommandBuffer output,
                                    size_t byte_offset) noexcept;
// CONSTANT_FILL, 5 dwords: DW0 op 11 | fillsize 2 (dword pattern) [31:30];
// DW1-2 4-byte aligned address; DW3 pattern; DW4 byte_count - 1 [29:0], with
// byte_count a nonzero multiple of 4 up to sdma::max_transfer_bytes.
Blit2dStatus encode_constant_fill(uint64_t address, uint32_t value, uint32_t byte_count,
                                  sdma::CommandBuffer output, size_t byte_offset) noexcept;
// NOP, 1 dword, count 0. Mesa radv_sdma_emit_nop: the engine waits for pending
// copy operations, so a NOP is also the ordering barrier between dependent packets.
Blit2dStatus encode_nop(sdma::CommandBuffer output, size_t byte_offset) noexcept;

// A memory region one packet reads or writes: rect (pixels) of the 32-bpp layout
// at base with pitch_bytes; surface_height bounds the slice pitch.
struct Blit2dRegion final {
    uint64_t base;
    uint32_t pitch_bytes;
    uint32_t surface_height;
    Blit2dRect rect;
};

// Read-after-write, write-after-read and write-after-write detection between
// packets emitted since the last barrier. Regions of one layout (same base and
// pitch) compare exactly as rectangles; other regions compare conservatively as
// byte spans. Fixed capacity; no allocation.
class Blit2dHazardTracker final {
public:
    // read is null for packets that only write (fills).
    bool conflicts(const Blit2dRegion* read, const Blit2dRegion& write) const noexcept;
    bool has_room(uint32_t regions) const noexcept {
        return regions <= blit2d_hazard_capacity - count_;
    }
    void add(const Blit2dRegion& region, bool write) noexcept;
    void clear() noexcept { count_ = 0; }
    uint32_t size() const noexcept { return count_; }

private:
    struct Entry final {
        Blit2dRegion region;
        bool write;
    };

    Entry entries_[blit2d_hazard_capacity]{};
    uint32_t count_ = 0;
};

bool blit2d_regions_overlap(const Blit2dRegion& first, const Blit2dRegion& second) noexcept;

enum class Blit2dSubmitResult { submitted, busy, failed };
enum class Blit2dFenceState { signalled, pending, failed };

// Kernel-ring services, all PASSIVE_LEVEL, serialized with the engine by the
// device owner; context outlives the engine.
//  submit: one IB (ib handle, byte offset, dwords a nonzero multiple of 8,
//    32-byte aligned) on the engine's ring, returning its fence sequence.
//    Sdma6::submit maps ready/started to submitted, ring_full/busy to busy and
//    everything else to failed.
//  query: non-blocking; FenceManager::query(fence_ring_sdma0 + instance, seq)
//    signalled -> signalled, pending -> pending, other -> failed.
//  wait: blocks up to timeout_us; FenceManager::wait signalled -> signalled,
//    timed_out -> pending, other -> failed.
struct Blit2dSubmitter final {
    void* context;
    Blit2dSubmitResult (*submit)(void* context, memory::Handle ib, uint64_t ib_offset,
                                 uint32_t ib_dwords, uint32_t& sequence) noexcept;
    Blit2dFenceState (*query)(void* context, uint32_t sequence) noexcept;
    Blit2dFenceState (*wait)(void* context, uint32_t sequence, uint64_t timeout_us) noexcept;
};

// ib_buffer: one allocation of manager, CPU visible, GPU readable, with an
// sdma6_ib_alignment aligned GPU address, holding ib_count slots of ib_bytes each
// (ib_bytes a multiple of sdma6_ib_alignment, blit2d_min_ib_bytes up to
// blit2d_max_ib_dwords * 4; slot i at byte i * ib_bytes). Smaller slots bound
// the latency of one submission; more slots keep encoding while others run. scratch is
// optional (slot UINT32_MAX = none): a GPU read/write allocation, 4-byte aligned,
// used to bounce same-surface overlapping copies that would otherwise need more
// than two dependent strips. wait_timeout_us 0 selects blit2d_default_wait_us.
struct Blit2dEngineConfig final {
    memory::Manager* manager;
    memory::Handle ib_buffer;
    uint32_t ib_count;
    uint32_t ib_bytes;
    memory::Handle scratch;
    Blit2dSubmitter submitter;
    uint64_t wait_timeout_us;
};

struct Blit2dReport final {
    uint64_t operations = 0;
    uint64_t fill_packets = 0;
    uint64_t copy_packets = 0;
    uint64_t barrier_nops = 0;
    uint64_t padding_nops = 0;
    uint64_t ib_submissions = 0;
    uint64_t submitted_dwords = 0;
    uint64_t waits = 0;
    uint64_t busy_retries = 0;
    uint64_t bounced_rects = 0;
    uint64_t strip_rects = 0;
    uint32_t in_flight = 0;
    uint32_t open_dwords = 0;
    Blit2dStatus last_failure = Blit2dStatus::success;
    bool faulted = false;
};

// Device-owned, PASSIVE_LEVEL, externally serialized (one GDI/device lock). No
// method may run in an ISR/DPC: Manager calls and the submitter's wait block.
//
// Operations validate everything (surfaces, rectangles, bounds, aliasing, order)
// before encoding; a refused operation changes nothing. Accepted work is appended
// to the open IB and submitted when that IB fills, on flush or on synchronize.
// When every IB is in flight the engine waits for the oldest. Packets that touch
// memory an earlier, not yet barrier-separated packet wrote (or read, for a write)
// are preceded by a NOP barrier; the tracker spans IB boundaries and is cleared
// only after observed completion of all work.
//
// Fill: one CONSTANT_FILL per row, or one per 4 MiB of whole rows when a rect
// covers its full pitch. Copy: one COPY_LINEAR_SUBWIN per clip rectangle, ordered
// for same-surface overlap; a rectangle overlapping its own source is split into
// strips of |dy| rows (or |dx| columns when dy is 0), farthest first, each
// separated by a barrier, or bounced through scratch when that needs fewer packets.
//
// A failed submit, wait or fence makes the engine faulted: earlier IBs of the same
// operation may already be queued, so no partial result is retried. The device
// owner recovers through its reset path and release_after_reset.
class Blit2dEngine final {
public:
    Blit2dEngine() noexcept = default;
    Blit2dEngine(const Blit2dEngine&) = delete;
    Blit2dEngine& operator=(const Blit2dEngine&) = delete;

    // Validates and retains one lease on the IB allocation (Access::read) and the
    // scratch allocation (read_write). Refuses a second call while initialized.
    Blit2dStatus initialize(const Blit2dEngineConfig& config) noexcept;

    Blit2dStatus fill(const Blit2dFill& fill, const Blit2dClipList& clip) noexcept;
    Blit2dStatus copy(const Blit2dCopy& copy, const Blit2dClipList& clip) noexcept;
    // staging rectangle at staging_point -> destination_rect of destination.
    Blit2dStatus upload(const Blit2dStaging& staging, Blit2dPoint staging_point,
                        const Blit2dSurface& destination, const Blit2dRect& destination_rect,
                        const Blit2dClipList& clip) noexcept;
    // source_rect of source -> staging at staging_point; the bytes are valid for
    // the CPU after synchronize returns success.
    Blit2dStatus readback(const Blit2dSurface& source, const Blit2dRect& source_rect,
                          const Blit2dStaging& staging, Blit2dPoint staging_point) noexcept;

    // Submits the open IB, if it holds packets.
    Blit2dStatus flush() noexcept;
    // GdiFlush-style barrier: flush, then wait for every IB of this engine.
    Blit2dStatus synchronize() noexcept;
    // Non-blocking: recycles IBs whose fences signalled.
    Blit2dStatus retire() noexcept;

    // Requires an idle engine (nothing open or in flight): returns the leases.
    Blit2dStatus shutdown() noexcept;
    // Only after the device owner proved the SDMA engine quiescent (reset): drops
    // all IB state and returns every lease, from any state including faulted.
    Blit2dStatus release_after_reset() noexcept;

    Blit2dReport report() const noexcept;

private:
    struct IbSlot final {
        uint64_t offset;
        uint32_t capacity_dwords;
        uint32_t sequence;
        uint64_t order;
        bool in_flight;
    };

    struct Packet final {
        bool copy;
        Blit2dRegion read;
        Blit2dRegion write;
        uint32_t value;
    };

    struct Prepared final {
        Blit2dRect rects[blit2d_max_rects];
        uint8_t order[blit2d_max_rects];
        uint32_t count;
    };

    Blit2dStatus usable() const noexcept;
    sdma::CommandBuffer slot_output(const IbSlot& slot) const noexcept;
    Blit2dStatus fault(Blit2dStatus reason) noexcept;
    bool touches_engine_memory(const Blit2dSurface& surface) const noexcept;
    Blit2dStatus prepare_rects(const Blit2dSurface& destination, const Blit2dRect& bounds,
                               const Blit2dClipList& clip) noexcept;
    Blit2dStatus order_rects(int32_t dx, int32_t dy) noexcept;
    Blit2dStatus resolve_staging(const Blit2dStaging& staging, memory::Access access,
                                 Blit2dSurface& surface) const noexcept;
    Blit2dStatus run_copy(const Blit2dCopy& copy, const Blit2dClipList& clip) noexcept;
    void reset_state() noexcept;
    Blit2dStatus emit_rect_copy(const Blit2dCopy& copy, const Blit2dRect& rect, int32_t dx,
                                int32_t dy, bool same) noexcept;
    Blit2dStatus emit_strips(const Blit2dCopy& copy, const Blit2dRect& rect, int32_t dx,
                             int32_t dy) noexcept;
    Blit2dStatus emit(const Packet& packet) noexcept;
    Blit2dStatus emit_barrier() noexcept;
    Blit2dStatus reserve(uint32_t dwords) noexcept;
    Blit2dStatus acquire() noexcept;
    Blit2dStatus submit_open() noexcept;
    Blit2dStatus wait_slot(IbSlot& slot) noexcept;
    void refresh_completion() noexcept;
    void release_leases() noexcept;

    memory::Manager* manager_ = nullptr;
    Blit2dSubmitter submitter_{};
    uint64_t wait_timeout_us_ = 0;
    memory::Handle ib_handle_{};
    uint64_t ib_gpu_ = 0;
    uint64_t ib_bytes_total_ = 0;
    sdma::CommandBuffer ib_output_{nullptr, 0};
    IbSlot ibs_[blit2d_max_ib_slots]{};
    uint32_t ib_count_ = 0;
    uint32_t next_ = 0;
    uint64_t submit_order_ = 0;
    bool open_ = false;
    uint32_t open_dwords_ = 0;
    memory::Handle scratch_{};
    uint64_t scratch_gpu_ = 0;
    uint64_t scratch_bytes_ = 0;
    bool has_scratch_ = false;
    Blit2dHazardTracker hazards_{};
    Prepared prepared_{};
    Blit2dReport report_{};
    bool initialized_ = false;
};

} // namespace rx7900::hardware

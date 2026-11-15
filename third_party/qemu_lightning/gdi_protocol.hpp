/*
Module: lightning.gdi.protocol
Purpose: Share checked GDI rectangles, transfer bounds and canvas ownership.
Created: 2026-10-21
Architecture: Allocation-free C++17 policy; native/QEMU owners supply actual completion.
License: GPL-2.0-or-later.
*/
#pragma once

#include <stddef.h>
#include "hw/misc/lightning-gdi.h"

namespace lightning::gdi {

// Zero clips means the complete destination. Explicit clips must be positive,
// disjoint and fully contained in the bounded destination/canvas. This validates
// the entire list before callers may issue any mutating GPU operation.
bool validate_clip_batch(uint32_t width, uint32_t height, LightningDisplayRect destination,
                         const LightningDisplayRect* clips, uint32_t count) noexcept;

// Packed BGRA rows. Every failure zeroes both outputs; callers stripe larger
// transfers rather than truncating a full canvas into this fixed staging window.
bool packed_transfer_size(uint32_t width, uint32_t height, uint32_t& row_bytes,
                          uint32_t& total_bytes) noexcept;

// Canonical eight-bit ROP truth table, indexed by P/S/D bits. Invalid wider
// tables have no declared dependencies and are rejected by command validation.
bool rop3_uses_source(uint32_t table) noexcept;
bool rop3_uses_pattern(uint32_t table) noexcept;
bool rop3_uses_destination(uint32_t table) noexcept;
// Shared operand bounds for trusted backend calls and decoded wire commands.
// No mode/sequence token or pixel pointer is invented by the GPU adapter.
bool validate_rop3_inputs(uint32_t width, uint32_t height, LightningDisplayRect destination,
    uint32_t source_x, uint32_t source_y, uint32_t source_flags, uint32_t pattern_bgra,
    uint32_t table, uint32_t pixel_bytes) noexcept;
using RasterOperands = LightningGdiRasterOperands;
using GradientOperands = LightningGdiGradientOperands;
using ExternalImage = LightningGdiExternalImage;
bool validate_external_image(uint32_t width, uint32_t height,
    LightningDisplayRect destination, uint32_t source_x, uint32_t source_y,
    const ExternalImage& image) noexcept;
// No caller pointers or mesh indices cross this fixed per-element snapshot.
// Validation establishes signed geometry/color bounds before any GPU mutation.
bool validate_gradient_inputs(uint32_t width, uint32_t height, LightningDisplayRect destination,
    const GradientOperands& operands) noexcept;
bool metadata_size(uint32_t opcode, uint32_t clip_count, uint32_t& bytes) noexcept;
bool validate_raster_inputs(uint32_t width, uint32_t height, LightningDisplayRect destination,
    const RasterOperands& operands, uint32_t payload_bytes) noexcept;
// Arbitrary signed brush origins are reduced before GPU signed arithmetic.
bool normalize_pattern_origin(int32_t origin, uint32_t extent, uint32_t& normalized) noexcept;
struct RasterSourceView final {
    uint32_t physical_width = 0;
    uint32_t physical_height = 0;
    LightningDisplayRect logical_rect{};
    LightningDisplayRect valid_rect{};
    LightningDisplayRect screen_rect{};
};
// Validate source footprint geometry, not command sequence or operand meaning.
// UPLOAD dimensions describe the exact owned packed span; SCREEN returns the
// available canvas rectangle and its destination inside owned scratch storage.
// Missing guard pixels remain outside valid_rect. Every failure clears output.
// Call the complete raster validator before any read or GPU mutation.
bool raster_source_view(const RasterOperands& operands, uint32_t canvas_width,
    uint32_t canvas_height, RasterSourceView& output) noexcept;
struct StretchMapping final {
    LightningDisplayRect source{};
};
// Match EngStretchBltROP's per-clip endpoint truncation before DIB sampling.
// A zero derived width/height remains a valid constant texel, not an empty draw.
bool coloroncolor_clip_mapping(LightningGdiSignedRect original_destination,
    LightningDisplayRect source, LightningDisplayRect clip, StretchMapping& output) noexcept;
// Dispatch source-pinned endpoint rules using validated typed operation flags.
// Coordinates refer to owned snapshot storage. Guarded logical rectangles start
// after their leading texel(s); SCREEN canvas coordinates never escape scratch.
// Failure clears output; neither helper performs a source-memory access.
bool raster_clip_mapping(const RasterOperands& operands, LightningDisplayRect clip,
    StretchMapping& output) noexcept;

enum class Status : uint32_t {
    validated = LIGHTNING_GDI_OK,
    bad_packet = LIGHTNING_GDI_BAD_PACKET,
    stale_mode = LIGHTNING_GDI_STALE_MODE,
    bad_sequence = LIGHTNING_GDI_BAD_SEQUENCE,
    busy = LIGHTNING_GDI_BUSY,
    unsupported = LIGHTNING_GDI_UNSUPPORTED,
    not_ready = LIGHTNING_GDI_NOT_READY,
    no_memory = LIGHTNING_GDI_NO_MEMORY,
    gpu_failure = LIGHTNING_GDI_GPU_FAILURE,
    batch_incomplete = LIGHTNING_GDI_BATCH_INCOMPLETE
};

struct Mode final {
    uint32_t generation = 0;
    uint32_t epoch = 0;
    uint32_t width = 0;
    uint32_t height = 0;
};

struct Command final {
    LightningGdiCommand header{};
    RasterOperands raster{};
    GradientOperands gradient{};
    ExternalImage external{};
    LightningDisplayRect clips[LIGHTNING_GDI_MAX_RECTS]{};
};

// The private metadata snapshot is borrowed only during this call. Output is
// reset on failure. Pixel storage is validated separately and never retained by
// the parser. Validated means format/policy validity, never GPU completion.
Status decode_command(const uint8_t* metadata, size_t metadata_bytes, Command& output) noexcept;
Status validate_command(const Command& command, Mode mode) noexcept;

// Combine adjacent same-color fills without painting their bounding-box gaps.
// Appends are atomic: overlap, sequence gaps, other operations and clip capacity
// leave the existing batch unchanged. The caller retains every original receipt;
// one observed backend fence may retire those ordered receipts together.
class FillBatch final {
public:
    bool configure(Mode mode) noexcept;
    bool append(const Command& command) noexcept;
    LightningDisplayRect destination() const noexcept { return destination_; }
    const LightningDisplayRect* clips() const noexcept { return clips_; }
    uint32_t clip_count() const noexcept { return clip_count_; }
    uint32_t command_count() const noexcept { return command_count_; }
    uint32_t color() const noexcept { return color_; }
private:
    Mode mode_{};
    LightningDisplayRect destination_{};
    LightningDisplayRect clips_[LIGHTNING_GDI_MAX_RECTS]{};
    uint32_t clip_count_ = 0;
    uint32_t command_count_ = 0;
    uint32_t color_ = 0;
    uint32_t last_sequence_ = 0;
};

// The platform serializes all methods. This policy owns a fixed submission
// ledger; it owns no GPU handles, CPU mappings, allocations or physical address.
// accepted is called only after an actual host/backend acceptance receipt.
// completed is called only for the front ticket after observed GPU completion;
// READBACK additionally requires the bounded host write into the transfer window.
class CanvasState final {
public:
    Status configure(Mode mode) noexcept;
    Status validate(const Command& command) const noexcept;
    Status accepted(const Command& command, uint64_t owner_ticket) noexcept;
    Status completed(uint32_t sequence, uint64_t owner_ticket) noexcept;
    void poison() noexcept { poisoned_ = true; }
    // Only after actual backend release/context destruction retires all pending
    // device access. A native restart or LOST notification alone cannot do this.
    void retire_after_backend_release() noexcept;
    uint32_t next_sequence() const noexcept;
    uint32_t accepted_sequence() const noexcept { return accepted_; }
    uint32_t completed_sequence() const noexcept { return completed_; }
    uint32_t canvas_revision() const noexcept { return revision_; }
    uint32_t pending_count() const noexcept { return count_; }
    bool poisoned() const noexcept { return poisoned_; }
    bool batch_open() const noexcept { return upload_open_; }
    bool group_open() const noexcept { return group_open_; }
    // A drained coherent pixel prefix may be read back during an open logical
    // group, including power snapshots; it is not permission to publish it.
    bool readback_ready() const noexcept;
    bool publication_ready() const noexcept;
    Mode mode() const noexcept { return mode_; }
private:
    struct Pending final { uint32_t sequence = 0; uint64_t ticket = 0; };
    Mode mode_{};
    Pending pending_[LIGHTNING_GDI_MAX_PENDING_FENCES]{};
    uint32_t head_ = 0;
    uint32_t count_ = 0;
    uint32_t accepted_ = 0;
    uint32_t completed_ = 0;
    uint32_t revision_ = 0;
    uint32_t baseline_sequence_ = 0;
    uint64_t last_ticket_ = 0;
    LightningDisplayRect upload_batch_{};
    uint32_t upload_reference_ = 0;
    uint32_t upload_next_y_ = 0;
    uint32_t group_reference_ = 0;
    uint32_t group_end_sequence_ = 0;
    bool upload_open_ = false;
    bool baseline_accepted_ = false;
    bool group_open_ = false;
    bool poisoned_ = false;
};

enum class LeasePhase { free, awaiting_acceptance, awaiting_completion, readback_ready, quarantined };

// One fixed transfer window, held by its native device owner across all mode
// changes/restarts. Upload reads retire on exact acceptance. Readback writes
// retire only on actual completion plus the owner's copy into retained RAM.
class TransferLease final {
public:
    bool reserve(Mode mode, uint32_t sequence, bool readback) noexcept;
    bool accepted(Mode mode, uint32_t sequence) noexcept;
    bool rejected(Mode mode, uint32_t sequence) noexcept;
    bool completed(Mode mode, uint32_t sequence) noexcept;
    bool copied(Mode mode, uint32_t sequence) noexcept;
    void uncertain() noexcept { phase_ = LeasePhase::quarantined; }
    void retire_after_backend_release() noexcept;
    LeasePhase phase() const noexcept { return phase_; }
private:
    bool matches(Mode mode, uint32_t sequence) const noexcept;
    Mode mode_{};
    uint32_t sequence_ = 0;
    bool readback_ = false;
    LeasePhase phase_ = LeasePhase::free;
};

// Shared chunk planner for uploads and ROI readbacks. The caller only advances
// after the current stripe's actual acceptance/completion ownership transition.
class StripeCursor final {
public:
    bool begin(LightningDisplayRect rectangle, uint32_t width, uint32_t height) noexcept;
    bool current(LightningDisplayRect& rectangle) const noexcept;
    bool advance(LightningDisplayRect confirmed) noexcept;
    bool done() const noexcept { return valid_ && next_y_ == rectangle_.y + rectangle_.height; }
private:
    LightningDisplayRect rectangle_{};
    uint32_t next_y_ = 0;
    uint32_t rows_ = 0;
    bool valid_ = false;
};

} // namespace lightning::gdi

/*
Module: rx7900.reactos.radeon_display
Purpose: Admission policy and diagnostic ABI of the physical Radeon GDI display
driver: which DDI operations are exactly expressible as Radeon 2D IOCTLs.
Created: 2026-10-22
Architecture: Pure, allocation-free C++ shared by radeon_display.cpp (NT5 DDI
for rx7900video's adopted pre-OS scanout) and host tests. Contains no DDI types;
the DDI layer maps SURFOBJ/XLATEOBJ/BRUSHOBJ facts into these values.
Reuses framebuffer_shadow_policy.hpp rectangles read-only.
*/
#pragma once

#include "radeon_2d_abi.hpp"
#include "framebuffer_shadow_policy.hpp"

#include <stdint.h>

namespace rx7900::reactos::radeon_display {

namespace abi = rx7900::reactos::radeon2d::abi;
using Rectangle = rx7900::reactos::framebuffer::Rectangle;

// ---------------------------------------------------------------------------
// DrvEscape statistics (read-only, no input). Escape code "R2DS".
// Layout v3 (328 bytes, little-endian): +0 size, +4 version, +8 accelerated (0/1),
// +12 poisoned (0/1), +16 live_bitmaps, +20 reserved (0), then thirty-eight
// uint64 counters at +24 .. +320 in field order.
// ---------------------------------------------------------------------------
constexpr uint32_t statistics_escape = 0x53443252U;
constexpr uint32_t statistics_version = 3;
constexpr uint32_t statistics_counter_offset = 24;
constexpr uint32_t statistics_counters = 38;
constexpr uint32_t statistics_bytes = statistics_counter_offset + statistics_counters * 8U;
static_assert(statistics_bytes == 328, "Documented statistics layout");

struct Statistics final {
    uint32_t accelerated = 0;
    uint32_t poisoned = 0;
    uint32_t live_bitmaps = 0;       // Device bitmaps alive (GPU resident or demoted).
    uint64_t gpu_fills = 0;          // Accepted fill operations (one per DDI call).
    uint64_t gpu_copies = 0;         // Accepted device-to-device copies.
    uint64_t gpu_uploads = 0;        // Accepted bitmap-to-device uploads.
    uint64_t gpu_readbacks = 0;      // Completed device-to-bitmap readbacks.
    uint64_t fallback_bitblt = 0;    // DrvBitBlt calls rendered by EngBitBlt.
    uint64_t fallback_copybits = 0;  // DrvCopyBits calls rendered by EngCopyBits.
    uint64_t fallback_fillpath = 0;  // DrvFillPath calls passed to EngFillPath.
    uint64_t declined = 0;           // Miniport declines followed by a fallback.
    uint64_t syncs = 0;              // Successful GPU barriers before CPU access.
    uint64_t sync_failures = 0;      // Barriers that failed (operation refused).
    uint64_t lost = 0;               // Uncertain GPU outcomes (PDEV poisoned).
    // Clip lists above abi::max_rects rectangles.
    uint64_t clip_batches = 0;       // Requests submitted by multi-batch operations.
    uint64_t complex_clips = 0;      // Accelerated operations that needed batching.
    uint64_t clip_fallbacks = 0;     // Large clips sent to the engine (unbanded
                                     // same-surface order or enumeration error).
    uint64_t max_clip_rects = 0;     // Largest clip list accelerated (gauge).
    uint64_t cpu_finished_rects = 0; // Same-surface copy rectangles finished on the
                                     // CPU after a decline between batches.
    // Offscreen device bitmaps (DrvCreateDeviceBitmap).
    uint64_t bitmaps_created = 0;    // GPU-resident bitmaps handed to GDI.
    uint64_t bitmaps_refused = 0;    // Requests left to engine DIBs (policy/budget).
    uint64_t bitmaps_deleted = 0;
    uint64_t demotions = 0;          // Resident bitmaps moved to DIBs for CPU access.
    uint64_t demotion_failures = 0;  // CPU access refused: readback impossible.
    uint64_t promotions = 0;         // Demoted bitmaps moved back into VRAM.
    uint64_t surfaces_lost = 0;      // Bitmaps whose VRAM contents were lost.
    uint64_t offscreen_operations = 0; // GPU operations touching a device bitmap.
    uint64_t live_bitmap_bytes = 0;  // Offscreen bytes accounted (gauge).
    // Compute raster operations (one per DDI call) and their engine fallbacks.
    uint64_t gpu_raster_rops = 0;    // General ROP4/pattern/mono/mask DrvBitBlt.
    uint64_t gpu_transparent = 0;    // DrvTransparentBlt.
    uint64_t gpu_alpha = 0;          // DrvAlphaBlend.
    uint64_t gpu_gradients = 0;      // DrvGradientFill.
    uint64_t gpu_lines = 0;          // DrvLineTo (solid and pattern pens).
    uint64_t gpu_stretches = 0;      // DrvStretchBlt and guarded DrvStretchBltROP.
    uint64_t fallback_transparent = 0;
    uint64_t fallback_alphablend = 0;
    uint64_t fallback_gradient = 0;
    uint64_t fallback_lineto = 0;
    uint64_t fallback_stretch = 0;
    uint64_t brushes_realized = 0;   // DrvRealizeBrush 8x8 tiles.
    uint64_t brushes_refused = 0;    // Patterns left to the engine realization.
};

inline void count(uint64_t& value) noexcept {
    if (value != UINT64_MAX) {
        ++value;
    }
}

inline void add(uint64_t& value, uint64_t amount) noexcept {
    value = amount > UINT64_MAX - value ? UINT64_MAX : value + amount;
}

inline bool encode_statistics(const sdma::CommandBuffer& output, const Statistics& value) noexcept {
    const uint64_t counters[statistics_counters] = {value.gpu_fills, value.gpu_copies,
        value.gpu_uploads, value.gpu_readbacks, value.fallback_bitblt, value.fallback_copybits,
        value.fallback_fillpath, value.declined, value.syncs, value.sync_failures, value.lost,
        value.clip_batches, value.complex_clips, value.clip_fallbacks, value.max_clip_rects,
        value.cpu_finished_rects, value.bitmaps_created, value.bitmaps_refused,
        value.bitmaps_deleted, value.demotions, value.demotion_failures, value.promotions,
        value.surfaces_lost, value.offscreen_operations, value.live_bitmap_bytes, value.gpu_raster_rops,
        value.gpu_transparent, value.gpu_alpha, value.gpu_gradients, value.gpu_lines, value.gpu_stretches,
        value.fallback_transparent, value.fallback_alphablend, value.fallback_gradient, value.fallback_lineto,
        value.fallback_stretch, value.brushes_realized, value.brushes_refused};
    if (output.size() != statistics_bytes || !output.write_u32(0, statistics_bytes) ||
        !output.write_u32(4, statistics_version) || !output.write_u32(8, value.accelerated) ||
        !output.write_u32(12, value.poisoned) || !output.write_u32(16, value.live_bitmaps) ||
        !output.write_u32(20, 0)) {
        return false;
    }

    size_t offset = statistics_counter_offset;
    for (const uint64_t counter : counters) {
        if (!abi::detail::write_u64(output, offset, counter)) {
            return false;
        }

        offset += sizeof(uint64_t);
    }

    return offset == statistics_bytes;
}

// ---------------------------------------------------------------------------
// Destructive-reset recovery on the DLL side (abi version 6).
//
// Lost handles: the miniport answers outcome_surface_lost exactly once per lost
// handle (Receipt.lost_surface names it) and the handle is dead afterwards. The
// DLL remembers it so the engine-fallback path that follows (a readback that
// turns the bitmap into an engine-zeroed DIB) answers surface_lost locally
// instead of sending a dead handle. Bounded FIFO of every surface slot; a
// handle carries its slot generation, so it never repeats within a device and
// an overwritten entry can only belong to a surface that is gone already.
constexpr uint32_t lost_surface_slots = abi::max_surfaces;

struct LostSurfaces final {
    uint32_t handles[lost_surface_slots]{};
    uint32_t next = 0;
};

inline bool is_lost(const LostSurfaces& lost, uint32_t handle) noexcept {
    if (handle == 0) {
        return false;
    }

    for (const uint32_t entry : lost.handles) {
        if (entry == handle) {
            return true;
        }
    }

    return false;
}

inline void remember_lost(LostSurfaces& lost, uint32_t handle) noexcept {
    if (handle == 0 || is_lost(lost, handle)) {
        return;
    }

    lost.handles[lost.next] = handle;
    lost.next = (lost.next + 1U) % lost_surface_slots;
}

// Session revival. A poisoned PDEV (an uncertain outcome) asks the miniport
// (ioctl_query) whether a recovery resolved that uncertainty. Bounded: after
// max_revival_queries answers without a recovery in progress or completed the
// PDEV stays unaccelerated (a pending recovery is never counted).
constexpr uint32_t max_revival_queries = 16;

enum class Revival : uint32_t {
    // No recovery yet: stay poisoned (CPU access still refused) and ask later.
    wait,
    // recovery_generation advanced while operational: prove the session idle
    // (sync) and resume acceleration on it.
    revive,
    // 2D is off for good after a proven reset: no GPU work can touch memory, so
    // exact engine rendering on the CPU views is allowed again.
    quiescent,
    // 2D is off and GPU retirement is unknown, or the query budget is spent.
    give_up
};

inline Revival revival_decision(const abi::Capabilities& capabilities, uint32_t known_generation,
                                uint32_t queries) noexcept {
    // The miniport's recovery thread is still working: not counted as a query.
    if (capabilities.recovery_state == abi::recovery_pending) {
        return Revival::wait;
    }

    if (capabilities.recovery_state == abi::recovery_disabled_unproven) {
        return Revival::give_up;
    }

    if (capabilities.recovery_state == abi::recovery_disabled_quiescent) {
        return Revival::quiescent;
    }

    if (capabilities.recovery_generation != known_generation) {
        return Revival::revive;
    }

    return queries >= max_revival_queries ? Revival::give_up : Revival::wait;
}

// ---------------------------------------------------------------------------
// BitBlt/CopyBits admission. Exactness argument per plan:
//   fill:        BLACKNESS writes 0, WHITENESS writes 0xffffffff, PATCOPY with a
//                solid brush writes iSolidColor: the DIB32 engine stores exactly
//                these 32-bit values (high byte included), as does the SDMA fill.
//   device_copy: SRCCOPY between GPU surfaces of one PDEV (primary or resident
//                device bitmaps, trivial XLATE); pixels move verbatim with
//                temporary-copy overlap semantics.
//   upload:      SRCCOPY from a 32bpp engine bitmap (or demoted device bitmap)
//                with trivial XLATE; bytes are copied verbatim through staging.
//   readback:    SRCCOPY from a GPU surface into a 32bpp engine bitmap with
//                trivial XLATE; verbatim through staging after GPU completion.
// Everything else goes to the Eng* engine on CPU views (a GPU barrier for the
// primary, demotion to a DIB for device bitmaps), which is exact by
// construction. Raster requests (ROP3, patterns, text, alpha, stretch, lines,
// gradients) are added here as new plans once radeon_2d_abi.hpp defines them.
// ---------------------------------------------------------------------------
enum class SurfaceKind : uint32_t {
    none,        // No operand (or ROP does not use it).
    primary,     // This PDEV's primary device surface, trivially translated.
    offscreen,   // This PDEV's GPU-resident device bitmap, trivially translated.
    bitmap32,    // STYPE_BITMAP (or demoted device bitmap), BMF_32BPP, trivial XLATE.
    other        // Anything else: foreign device, other format, non-trivial XLATE.
};

enum class BlitPlan : uint32_t { fallback, fill, device_copy, upload, readback };

struct BlitDecision final {
    BlitPlan plan = BlitPlan::fallback;
    uint32_t color = 0;
};

constexpr uint32_t rop3_blackness = 0x00;
constexpr uint32_t rop3_patcopy = 0xf0;
constexpr uint32_t rop3_srccopy = 0xcc;
constexpr uint32_t rop3_whiteness = 0xff;
constexpr uint32_t no_solid_color = UINT32_MAX; // BRUSHOBJ iSolidColor of a pattern brush.

constexpr bool gpu_surface(SurfaceKind kind) noexcept {
    return kind == SurfaceKind::primary || kind == SurfaceKind::offscreen;
}

// rop4 is the DrvBitBlt ROP4 (CopyBits passes 0xcccc). brush_color is the
// realized iSolidColor or no_solid_color. masked means a mask surface is present.
constexpr BlitDecision classify_blit(uint32_t rop4, bool masked, uint32_t brush_color,
                                     SurfaceKind target, SurfaceKind source) noexcept {
    const uint32_t foreground = rop4 & 0xffU;
    const uint32_t background = (rop4 >> 8) & 0xffU;
    if (masked || rop4 > 0xffffU || foreground != background) {
        return {};
    }

    if (gpu_surface(target)) {
        if (foreground == rop3_blackness) {
            return {BlitPlan::fill, 0U};
        }

        if (foreground == rop3_whiteness) {
            return {BlitPlan::fill, UINT32_MAX};
        }

        if (foreground == rop3_patcopy) {
            return brush_color != no_solid_color ? BlitDecision{BlitPlan::fill, brush_color} :
                                                   BlitDecision{};
        }

        if (foreground == rop3_srccopy) {
            if (gpu_surface(source)) {
                return {BlitPlan::device_copy, 0U};
            }

            if (source == SurfaceKind::bitmap32) {
                return {BlitPlan::upload, 0U};
            }
        }

        return {};
    }

    if (target == SurfaceKind::bitmap32 && gpu_surface(source) && foreground == rop3_srccopy) {
        return {BlitPlan::readback, 0U};
    }

    return {};
}

// ---------------------------------------------------------------------------
// Clip lists larger than one request. An operation is split into batches of at
// most abi::max_rects rectangles, each intersected with the operation bounds,
// submitted in enumeration order; the miniport executes requests in order and
// the backend separates dependent packets with barriers, also across requests.
// Fills, distinct-surface copies, uploads and readbacks do not read what they
// write, so any batch order is exact and a later decline may be followed by an
// engine fallback of the whole operation. A same-surface copy is exact only
// if no earlier rectangle overwrites a later rectangle's source:
//
// CLIPOBJ regions are y-x banded: rectangles of one band share top and bottom
// and are disjoint in x; bands are disjoint in y. Enumerating top-down when the
// source lies below (dy > 0), bottom-up when above (dy < 0), and left-to-right
// when the source lies to the right (dx > 0), right-to-left when left (dx < 0),
// every source of a later rectangle lies in a later band (shifted further in
// the direction of travel) or later in the same band, so it is never written
// by an earlier rectangle. Each single rectangle is a memmove. BandOrder
// verifies this banded order on the actual enumeration (CLIPOBJ_cEnumStart
// with copy_direction) before anything is submitted; an unbanded enumeration
// is counted (clip_fallbacks) and rendered by the engine.
// ---------------------------------------------------------------------------
enum class CopyDirection : uint32_t { right_down, left_down, right_up, left_up };

// (dx, dy) = source - destination.
constexpr CopyDirection copy_direction(int32_t dx, int32_t dy) noexcept {
    return dy < 0 ? (dx < 0 ? CopyDirection::left_up : CopyDirection::right_up) :
                    (dx < 0 ? CopyDirection::left_down : CopyDirection::right_down);
}

constexpr bool moves_down(CopyDirection direction) noexcept {
    return direction == CopyDirection::right_down || direction == CopyDirection::left_down;
}

constexpr bool moves_right(CopyDirection direction) noexcept {
    return direction == CopyDirection::right_down || direction == CopyDirection::right_up;
}

class BandOrder final {
public:
    explicit constexpr BandOrder(CopyDirection direction) noexcept : direction_(direction) {}

    // Next nonempty rectangle of the enumeration. false: the sequence is not a
    // banded region in this direction (and stays refused).
    bool accept(const Rectangle& rect) noexcept {
        if (failed_ || rect.left >= rect.right || rect.top >= rect.bottom) {
            failed_ = true;
            return false;
        }

        if (started_ && rect.top == band_top_) {
            const bool ordered = moves_right(direction_) ? rect.left >= last_.right :
                                                           rect.right <= last_.left;
            failed_ = rect.bottom != band_bottom_ || !ordered;
        } else if (started_) {
            failed_ = moves_down(direction_) ? rect.top < band_bottom_ : rect.bottom > band_top_;
            band_top_ = rect.top;
            band_bottom_ = rect.bottom;
        } else {
            started_ = true;
            band_top_ = rect.top;
            band_bottom_ = rect.bottom;
        }

        last_ = rect;
        return !failed_;
    }

    bool failed() const noexcept { return failed_; }

private:
    CopyDirection direction_;
    bool started_ = false;
    bool failed_ = false;
    int32_t band_top_ = 0;
    int32_t band_bottom_ = 0;
    Rectangle last_{};
};

// Appends rect clipped to bounds when the result is nonempty and room remains.
inline bool append_clipped(const Rectangle& bounds, const Rectangle& rect, abi::RectList& list) noexcept {
    const Rectangle clipped = framebuffer::intersect(bounds, rect);
    if (framebuffer::empty(clipped) || list.count >= abi::max_rects) {
        return false;
    }

    list.rects[list.count++] = {clipped.left, clipped.top, clipped.right, clipped.bottom};
    return true;
}

// Union of a nonempty list (empty Rectangle for an empty one).
inline Rectangle list_bounds(const abi::RectList& list) noexcept {
    if (list.count == 0 || list.count > abi::max_rects) {
        return {};
    }

    Rectangle bounds{list.rects[0].left, list.rects[0].top, list.rects[0].right, list.rects[0].bottom};
    for (uint32_t index = 1; index < list.count; ++index) {
        const abi::Rect& rect = list.rects[index];
        bounds.left = rect.left < bounds.left ? rect.left : bounds.left;
        bounds.top = rect.top < bounds.top ? rect.top : bounds.top;
        bounds.right = rect.right > bounds.right ? rect.right : bounds.right;
        bounds.bottom = rect.bottom > bounds.bottom ? rect.bottom : bounds.bottom;
    }

    return bounds;
}

// Each rectangle translated by (dx, dy) stays inside a width x height surface.
inline bool translated_inside(const abi::RectList& list, int32_t dx, int32_t dy, uint32_t width,
                              uint32_t height) noexcept {
    for (uint32_t index = 0; index < list.count && index < abi::max_rects; ++index) {
        const abi::Rect& rect = list.rects[index];
        const int64_t left = static_cast<int64_t>(rect.left) + dx;
        const int64_t top = static_cast<int64_t>(rect.top) + dy;
        const int64_t right = static_cast<int64_t>(rect.right) + dx;
        const int64_t bottom = static_cast<int64_t>(rect.bottom) + dy;
        if (left < 0 || top < 0 || right > width || bottom > height) {
            return false;
        }
    }

    return true;
}

// ---------------------------------------------------------------------------
// Offscreen device bitmaps (DrvCreateDeviceBitmap). A bitmap is GPU resident
// (a miniport offscreen surface) or demoted (an engine DIB owned by the driver).
// CPU access to a resident bitmap demotes it: one readback of the whole bitmap,
// then the VRAM surface is destroyed. A demoted bitmap used in promote_after_uses
// consecutive GPU-expressible operations is uploaded into a new surface again;
// any CPU use resets the streak. Small bitmaps (icons, glyph caches, 1-line
// strips) stay engine DIBs because their CPU access dominates. Requests beyond
// the session's surface count or byte budget are refused and GDI creates an
// engine DIB instead (counted as bitmaps_refused), which is a complete and
// exact path under memory pressure.
// ---------------------------------------------------------------------------
constexpr uint32_t min_bitmap_dimension = 8;
constexpr uint64_t min_bitmap_pixels = 64 * 64;
constexpr uint32_t promote_after_uses = 4;
// Offscreen pitch granule of the backend (radeon2d_pitch_alignment); used only
// to predict the budget before the miniport reports the actual pitch.
constexpr uint32_t offscreen_pitch_alignment = 256;

enum class BitmapAdmission : uint32_t { admit, too_small, too_large, count_limit, budget };

struct BitmapLimits final {
    uint32_t max_count = 0;
    uint64_t budget_bytes = 0;
};

constexpr uint64_t offscreen_bytes(uint32_t width, uint32_t height) noexcept {
    return ((static_cast<uint64_t>(width) * abi::bytes_per_pixel + (offscreen_pitch_alignment - 1U)) &
            ~static_cast<uint64_t>(offscreen_pitch_alignment - 1U)) * height;
}

constexpr BitmapAdmission admit_bitmap(int32_t width, int32_t height, uint32_t live_count,
                                       uint64_t live_bytes, const BitmapLimits& limits) noexcept {
    if (width <= 0 || height <= 0 || static_cast<uint32_t>(width) > abi::max_surface_dimension ||
        static_cast<uint32_t>(height) > abi::max_surface_dimension) {
        return BitmapAdmission::too_large;
    }

    const uint32_t w = static_cast<uint32_t>(width);
    const uint32_t h = static_cast<uint32_t>(height);
    if (w < min_bitmap_dimension || h < min_bitmap_dimension ||
        static_cast<uint64_t>(w) * h < min_bitmap_pixels) {
        return BitmapAdmission::too_small;
    }

    if (live_count >= limits.max_count) {
        return BitmapAdmission::count_limit;
    }

    const uint64_t bytes = offscreen_bytes(w, h);
    return live_bytes > limits.budget_bytes || bytes > limits.budget_bytes - live_bytes ?
        BitmapAdmission::budget : BitmapAdmission::admit;
}

// ---------------------------------------------------------------------------
// DrvFillPath admission: one closed axis-aligned rectangle with integer FIX
// (28.4) vertices. GDI's fill convention lights pixels whose centers lie
// inside, including top/left and excluding bottom/right edges, so the filled
// set of an integer rectangle with corners (x0,y0)-(x1,y1) is [x0,x1) x [y0,y1).
// points holds point_count (x, y) pairs; a repeated closing point is allowed.
// ---------------------------------------------------------------------------
constexpr int32_t fix_one = 16;
constexpr uint32_t mix_copy_pen = 13; // R2_COPYPEN

inline bool rectangle_from_path(const int32_t* points, uint32_t point_count,
                                Rectangle& output) noexcept {
    if (points == nullptr || (point_count != 4 && point_count != 5)) {
        return false;
    }

    // Caller provides point_count pairs; indices below stay below 2 * point_count.
    if (point_count == 5 && (points[8] != points[0] || points[9] != points[1])) {
        return false;
    }

    int32_t min_x = points[0];
    int32_t max_x = points[0];
    int32_t min_y = points[1];
    int32_t max_y = points[1];
    for (uint32_t index = 0; index < 4; ++index) {
        const int32_t x = points[index * 2];
        const int32_t y = points[index * 2 + 1];
        if (x % fix_one != 0 || y % fix_one != 0) {
            return false;
        }

        min_x = x < min_x ? x : min_x;
        max_x = x > max_x ? x : max_x;
        min_y = y < min_y ? y : min_y;
        max_y = y > max_y ? y : max_y;
    }

    if (min_x == max_x || min_y == max_y) {
        return false;
    }

    // Every vertex is a corner, and consecutive vertices differ in exactly one axis.
    for (uint32_t index = 0; index < 4; ++index) {
        const uint32_t next = (index + 1U) % 4U;
        const int32_t x = points[index * 2];
        const int32_t y = points[index * 2 + 1];
        const int32_t next_x = points[next * 2];
        const int32_t next_y = points[next * 2 + 1];
        const bool corner = (x == min_x || x == max_x) && (y == min_y || y == max_y);
        const bool axis_edge = (x == next_x) != (y == next_y);
        if (!corner || !axis_edge) {
            return false;
        }
    }

    output = {min_x / fix_one, min_y / fix_one, max_x / fix_one, max_y / fix_one};
    return true;
}

// ---------------------------------------------------------------------------
// Staging plan. The session staging buffer is split into two halves used
// alternately so the CPU can fill one while the GPU still reads the other. A
// transfer of bounds (width x height pixels) fits a half when its packed size
// does; otherwise the caller bands each rectangle by band_rows().
// ---------------------------------------------------------------------------
constexpr uint32_t staging_halves = 2;

constexpr uint32_t staging_half_bytes(uint32_t staging_bytes) noexcept {
    return (staging_bytes / staging_halves) & ~3U;
}

constexpr bool fits_half(uint32_t width, uint32_t height, uint32_t half_bytes) noexcept {
    return width != 0 && height != 0 && width <= abi::max_surface_dimension &&
           static_cast<uint64_t>(width) * abi::bytes_per_pixel * height <= half_bytes;
}

// Rows of a width-pixel rectangle per half (0 if not even one row fits).
constexpr uint32_t band_rows(uint32_t width, uint32_t half_bytes) noexcept {
    if (width == 0 || width > abi::max_surface_dimension) {
        return 0;
    }

    return half_bytes / (width * abi::bytes_per_pixel);
}

// ---------------------------------------------------------------------------
// Compute raster admission (abi version 3). Each plan reproduces the pinned
// ReactOS engine (a97c8a2 win32ss/gdi) arithmetic so the GPU result equals the
// Eng* fallback; everything else keeps that fallback, counted per DDI.
// ---------------------------------------------------------------------------

// DIB_DoRop minterm index P << 2 | S << 1 | D.
constexpr bool rop3_uses_source(uint32_t rop3) noexcept { return ((rop3 ^ (rop3 >> 2)) & 0x33U) != 0; }
constexpr bool rop3_uses_pattern(uint32_t rop3) noexcept { return ((rop3 ^ (rop3 >> 4)) & 0x0fU) != 0; }
constexpr bool rop3_uses_destination(uint32_t rop3) noexcept { return ((rop3 ^ (rop3 >> 1)) & 0x55U) != 0; }
constexpr bool rop4_uses_destination(uint32_t rop4) noexcept {
    return rop3_uses_destination(rop4 & 0xffU) || rop3_uses_destination((rop4 >> 8) & 0xffU);
}
constexpr bool rop4_uses_source(uint32_t rop4) noexcept {
    return rop3_uses_source(rop4 & 0xffU) || rop3_uses_source((rop4 >> 8) & 0xffU);
}
constexpr bool rop4_uses_pattern(uint32_t rop4) noexcept {
    return rop3_uses_pattern(rop4 & 0xffU) || rop3_uses_pattern((rop4 >> 8) & 0xffU);
}

// Source of a compute ROP: a GPU surface of this PDEV, a 32bpp engine bitmap
// (trivial XLATE, staged), a 1bpp engine bitmap (expanded with its XLATE
// colours, staged), or none.
// translated: an 8/16/24bpp engine bitmap staged with its XLATEOBJ table (0xa17 only).
enum class RasterSource : uint32_t { none, gpu, bitmap32, mono, unsupported, translated };

// General DrvBitBlt through the compute ROP: target a GPU surface; any ROP4 up
// to 0xffff; distinct halves only with a 1bpp mask (BltMask: mask bit 1 selects
// the low byte); the source as above when the ROP reads it; a solid or realized
// 8x8 brush when the ROP reads the pattern.
constexpr bool compute_rop_admitted(uint32_t rop4, bool masked, bool mask_mono, SurfaceKind target,
                                    RasterSource source, bool solid_brush, bool tile) noexcept {
    if (rop4 > 0xffffU || !gpu_surface(target) || (masked && !mask_mono) ||
        (!masked && (rop4 & 0xffU) != ((rop4 >> 8) & 0xffU))) {
        return false;
    }

    if (rop4_uses_source(rop4) ? source == RasterSource::none || source == RasterSource::unsupported : false) {
        return false;
    }

    return !rop4_uses_pattern(rop4) || solid_brush || tile;
}

// Bit-exact copy of count bits starting at bit first (MSB first) of source into
// destination from bit 0, zero padding the last byte. Caller sized both spans.
inline void copy_mono_bits(const uint8_t* source, uint32_t first, uint32_t count, uint8_t* destination) noexcept {
    const uint32_t bytes = (count + 7U) / 8U;
    for (uint32_t index = 0; index < bytes; ++index) {
        destination[index] = 0;
    }

    for (uint32_t bit = 0; bit < count; ++bit) {
        const uint32_t from = first + bit;
        if (((source[from >> 3] >> (7U - (from & 7U))) & 1U) != 0) {
            destination[bit >> 3] = static_cast<uint8_t>(destination[bit >> 3] | (0x80U >> (bit & 7U)));
        }
    }
}

// Clip complexity as the engine sees it (DC_TRIVIAL, DC_RECT, DC_COMPLEX).
enum class ClipKind : uint32_t { trivial, rect, complex };

// EngTransparentBlt (eng/transblt.c): the engine clips the source rectangle by
// the clip bounds with a pixel shift (not scaled), then maps each clip
// rectangle back with integer division against the unclipped destination.
// stretch: output/input after that adjustment and the DIB-level sizes.
struct StretchSetup final {
    Rectangle output{};
    Rectangle input{};
    int64_t destination_width = 0;
    int64_t destination_height = 0;
    int64_t source_width = 0;
    int64_t source_height = 0;
};

// false: nothing is drawn (degenerate after clipping). destination and source
// are the rectangles DrvTransparentBlt received (destination well ordered).
inline bool transparent_setup(const Rectangle& destination, const Rectangle& source, bool clipped,
                              const Rectangle& clip_bounds, StretchSetup& setup) noexcept {
    Rectangle input = source;
    Rectangle output = destination;
    if (clipped) {
        if (output.left < clip_bounds.left) {
            input.left += clip_bounds.left - output.left;
            output.left = clip_bounds.left;
        }

        if (clip_bounds.right < output.right) {
            input.right -= output.right - clip_bounds.right;
            output.right = clip_bounds.right;
        }

        if (output.top < clip_bounds.top) {
            input.top += clip_bounds.top - output.top;
            output.top = clip_bounds.top;
        }

        if (clip_bounds.bottom < output.bottom) {
            input.bottom -= output.bottom - clip_bounds.bottom;
            output.bottom = clip_bounds.bottom;
        }
    }

    if (output.right <= output.left || output.bottom <= output.top) {
        return false;
    }

    // The DIB stage maps against the unclipped destination again.
    setup.output = destination;
    setup.input = input;
    setup.destination_width = static_cast<int64_t>(destination.right) - destination.left;
    setup.destination_height = static_cast<int64_t>(destination.bottom) - destination.top;
    setup.source_width = static_cast<int64_t>(input.right) - input.left;
    setup.source_height = static_cast<int64_t>(input.bottom) - input.top;
    return setup.destination_width > 0 && setup.destination_height > 0;
}

// One DC_RECT/DC_COMPLEX rectangle: CombinedRect and InputToCombinedRect.
inline bool transparent_rect(const StretchSetup& setup, const Rectangle& clip, Rectangle& combined,
                             Rectangle& input) noexcept {
    combined = framebuffer::intersect(setup.output, clip);
    if (framebuffer::empty(combined)) {
        return false;
    }

    const Rectangle& out = setup.output;
    input.top = static_cast<int32_t>(setup.input.top +
        (static_cast<int64_t>(combined.top) - out.top) * setup.source_height / setup.destination_height);
    input.bottom = static_cast<int32_t>(setup.input.top +
        (static_cast<int64_t>(combined.bottom) - out.top) * setup.source_height / setup.destination_height);
    input.left = static_cast<int32_t>(setup.input.left +
        (static_cast<int64_t>(combined.left) - out.left) * setup.source_width / setup.destination_width);
    input.right = static_cast<int32_t>(setup.input.left +
        (static_cast<int64_t>(combined.right) - out.left) * setup.source_width / setup.destination_width);
    return true;
}

// EngAlphaBlend (eng/alphablend.c) per clip rectangle: Rect from the clipped
// output with the engine's (asymmetric, truncating) formulas.
inline bool alpha_rect(const Rectangle& output, const Rectangle& input, const Rectangle& clip, Rectangle& combined,
                       Rectangle& source) noexcept {
    combined = framebuffer::intersect(output, clip);
    if (framebuffer::empty(combined)) {
        return false;
    }

    const int64_t in_width = static_cast<int64_t>(input.right) - input.left;
    const int64_t in_height = static_cast<int64_t>(input.bottom) - input.top;
    const int64_t out_width = static_cast<int64_t>(output.right) - output.left;
    const int64_t out_height = static_cast<int64_t>(output.bottom) - output.top;
    if (out_width <= 0 || out_height <= 0) {
        return false;
    }

    source.left = static_cast<int32_t>(input.left + (static_cast<int64_t>(combined.left) - output.left) * in_width / out_width);
    source.right = static_cast<int32_t>(input.right + (static_cast<int64_t>(combined.right) - output.right) * in_width / out_width);
    source.top = static_cast<int32_t>(input.top + (static_cast<int64_t>(combined.top) - output.top) * in_height / out_height);
    source.bottom = static_cast<int32_t>(input.bottom + (static_cast<int64_t>(combined.bottom) - output.bottom) * in_height / out_height);
    return true;
}

// The source span a nearest map reads for a destination rectangle (the map's
// dref is its own left/top): [left, left + span) per axis, intersected with a
// width x height bitmap. false when no source pixel exists.
inline bool mapped_span(int32_t left, int32_t top, int64_t map_width, int64_t map_height, uint32_t width,
                        uint32_t height, Rectangle& span) noexcept {
    const int64_t l = left < 0 ? 0 : left;
    const int64_t t = top < 0 ? 0 : top;
    int64_t r = static_cast<int64_t>(left) + map_width;
    int64_t b = static_cast<int64_t>(top) + map_height;
    r = r > width ? width : r;
    b = b > height ? height : b;
    if (r <= l || b <= t) {
        return false;
    }

    span = {static_cast<int32_t>(l), static_cast<int32_t>(t), static_cast<int32_t>(r), static_cast<int32_t>(b)};
    return true;
}

// EngGradientFill skips triangles with two equal vertices (IntEngIsNULLTriangle).
// A single-colour triangle uses the engine's solid span rules, which the
// backend's triangle planner reproduces (raster2d single-colour path).
constexpr bool null_triangle(const abi::Vertex& a, const abi::Vertex& b, const abi::Vertex& c) noexcept {
    return (a.x == b.x && a.y == b.y) || (a.x == c.x && a.y == c.y) || (b.x == c.x && b.y == c.y);
}

constexpr bool solid_triangle(const abi::Vertex& a, const abi::Vertex& b, const abi::Vertex& c) noexcept {
    return a.red == b.red && a.red == c.red && a.green == b.green && a.green == c.green && a.blue == b.blue &&
           a.blue == c.blue;
}

// Height of a triangle in rows (the backend plans at most a fixed number).
constexpr int64_t triangle_rows(const abi::Vertex& a, const abi::Vertex& b, const abi::Vertex& c) noexcept {
    const int32_t top = a.y < b.y ? (a.y < c.y ? a.y : c.y) : (b.y < c.y ? b.y : c.y);
    const int32_t bottom = a.y > b.y ? (a.y > c.y ? a.y : c.y) : (b.y > c.y ? b.y : c.y);
    return static_cast<int64_t>(bottom) - top;
}
constexpr int64_t max_triangle_rows = abi::max_triangle_rows;

// DrvLineTo: the engine's EngLineTo ignores MIX and styles come from private
// brush state the DDI cannot read, so a GPU line is drawn only for a call the
// paired kernel identified as unstyled (line_mix_tag) with R2_COPYPEN; the
// tag is stripped before any engine call.
constexpr uint32_t line_mix_tag = 0x4e4c0000U;   // RX7900_LINE_MIX_TAG
constexpr uint32_t mix_tag_mask = 0xffff0000U;
constexpr bool tagged_copy_pen(uint32_t mix) noexcept {
    return (mix & mix_tag_mask) == line_mix_tag && (mix & 0xffU) == mix_copy_pen &&
           (((mix >> 8) & 0xffU) == 0 || ((mix >> 8) & 0xffU) == mix_copy_pen);
}

// DrvLineTo pen colour. The pinned EngLineTo plots pbo->iSolidColor for every
// brush: a pattern pen (iSolidColor 0xffffffff, no realization consulted, no
// brush origin) draws 0xffffffff pixels, so that value is the exact GPU colour.
constexpr uint32_t engine_line_colour(uint32_t solid_colour) noexcept { return solid_colour; }

// The paired win32k bridge supplies the private brush's positive run lengths
// for one cosmetic segment. A declined call reaches EngLineTo on this driver's
// own DIB view (DrvLineTo -> cpu_view): IntEngEnter returns a zero Translate for
// an STYPE_BITMAP, so HandleStyles (pinned eng/lineto.c) computes offStyle =
// 0 % ulStyleSize = 0 and iStyle = 0, and seeds lStyleMax from the segment's
// first point: on the x axis when deltax > deltay (x0 + run0, or x0 - 1 when
// moving left), otherwise on the y axis (y0 + run0, or y0 - 1 moving up). The
// octant loops then step the major axis (y only when deltax < deltay, so a
// diagonal tie steps x against the y-seeded limit), and at each limit move to
// the next run (forward) or (iStyle - 1) % cStyles in ULONG (backward) with a
// new limit one run further. A pixel is plotted when the run index is even.
// The clip walk never touches this state, so the bit of step k is independent
// of the clip rectangles (accelerate_line relies on that for DC_COMPLEX).
constexpr uint32_t max_engine_style_runs = 32;
inline bool engine_style_bits(int32_t x0, int32_t y0, int32_t x1, int32_t y1,
                              const uint32_t* runs, uint32_t run_count,
                              uint8_t (&bits)[abi::style_bytes], uint32_t& bit_count) noexcept {
    bit_count = 0;
    if (runs == nullptr || run_count == 0 || run_count > max_engine_style_runs) {
        return false;
    }
    const int64_t dx = static_cast<int64_t>(x1) - x0;
    const int64_t dy = static_cast<int64_t>(y1) - y0;
    const uint64_t ax = static_cast<uint64_t>(dx < 0 ? -dx : dx);
    const uint64_t ay = static_cast<uint64_t>(dy < 0 ? -dy : dy);
    const uint64_t major = ax > ay ? ax : ay;
    if (major == 0 || major > abi::max_style_bits) {
        return false;
    }
    for (uint32_t index = 0; index < run_count; ++index) {
        if (runs[index] == 0 || runs[index] > abi::max_style_bits) {
            return false;
        }
    }
    for (uint8_t& value : bits) {
        value = 0;
    }
    const int64_t xchange = dx < 0 ? -1 : 1;
    const int64_t ychange = dy < 0 ? -1 : 1;
    // HandleStyles: the seeding axis (deltax > deltay) and the stepping axis
    // (deltax < deltay selects y) differ only for diagonal ties.
    const bool seed_x = ax > ay;
    const int64_t seed_direction = seed_x ? xchange : ychange;
    int64_t limit = seed_x ? x0 : y0;
    limit = seed_direction > 0 ? limit + runs[0] : limit - 1;
    const bool step_y = ax < ay;
    const int64_t direction = step_y ? ychange : xchange;
    int64_t position = step_y ? y0 : x0;
    uint32_t run = 0;
    for (uint32_t step = 0; step < major; ++step) {
        if ((run & 1U) == 0) {
            bits[step >> 3] |= static_cast<uint8_t>(0x80U >> (step & 7U));
        }
        position += direction;
        if (position == limit) {
            run = direction > 0 ? (run + 1U) % run_count : (run - 1U) % run_count;
            limit = position + direction * static_cast<int64_t>(runs[run]);
        }
    }
    bit_count = static_cast<uint32_t>(major);
    return true;
}

// ---------------------------------------------------------------------------
// DrvStretchBltROP behind the paired-kernel stretch guard (eng/stretchblt.c). The
// driver receives IntEngStretchBlt's well-ordered OutputRect/InputRect (after its
// Case0000..Case1110 +-1 adjustments and clip-bounds remap) and, through the
// guard, bLeftToRight/bTopToBottom. On FALSE the engine calls EngStretchBltROP
// with OutputRect swapped per flip; the plan below reproduces that call:
// well-ordering, the unscaled clip-bounds pre-adjustment, the degenerate test,
// the DC_TRIVIAL/DC_RECT/DC_COMPLEX InputToCombinedRect remap and one
// DIB_XXBPP_StretchBlt call per rectangle with the flipped CombinedRect. All
// arithmetic stays below 2^31 for admitted inputs (|coordinates| <= 2^24,
// extents <= 16384), so the engine's 32-bit LONG products are reproduced.
// ---------------------------------------------------------------------------
struct StretchRopSetup final {
    Rectangle output{};      // OutputRect after the pre-adjustment (well ordered)
    Rectangle input{};       // InputRect after it (source coordinates)
    Rectangle domain{};      // normalized prclSrc: a device source's IntEngEnter copy
    bool flip_x = false;     // bLeftToRight of EngStretchBltROP
    bool flip_y = false;     // bTopToBottom
    bool uses_source = false;
    int64_t destination_width = 0;
    int64_t destination_height = 0;
    int64_t source_width = 0;
    int64_t source_height = 0;
};

constexpr bool rop4_noop(uint32_t rop4) noexcept { return rop4 == 0xaaaaU; }

// The prclDest of the engine fallback: the driver's OutputRect with the flips.
inline Rectangle stretch_fallback_destination(const Rectangle& output, bool flip_x, bool flip_y) noexcept {
    Rectangle value = output;
    if (flip_x) {
        value.left = output.right;
        value.right = output.left;
    }

    if (flip_y) {
        value.top = output.bottom;
        value.bottom = output.top;
    }

    return value;
}

// EngStretchBltROP(prclDest = stretch_fallback_destination, prclSrc = input).
// clip_bounds: rclBounds of the CLIPOBJ, null without one. false: the engine
// draws nothing (ROP4_NOOP or a degenerate OutputRect).
inline bool stretch_rop_setup(const Rectangle& output, const Rectangle& input, bool flip_x, bool flip_y,
                              uint32_t rop4, const Rectangle* clip_bounds, StretchRopSetup& setup) noexcept {
    setup = {};
    const Rectangle destination = stretch_fallback_destination(output, flip_x, flip_y);
    const int64_t cx_source = static_cast<int64_t>(input.right) - input.left;
    const int64_t cy_source = static_cast<int64_t>(input.bottom) - input.top;
    const int64_t cx_destination = static_cast<int64_t>(destination.right) - destination.left;
    const int64_t cy_destination = static_cast<int64_t>(destination.bottom) - destination.top;
    setup.flip_x = !((cx_source < 0 && cx_destination < 0) || (cx_source >= 0 && cx_destination >= 0));
    setup.flip_y = !((cy_source < 0 && cy_destination < 0) || (cy_source >= 0 && cy_destination >= 0));
    setup.uses_source = rop4_uses_source(rop4);
    Rectangle out{destination.left < destination.right ? destination.left : destination.right,
                  destination.top < destination.bottom ? destination.top : destination.bottom,
                  destination.left < destination.right ? destination.right : destination.left,
                  destination.top < destination.bottom ? destination.bottom : destination.top};
    if (rop4_noop(rop4)) {
        return false;
    }

    Rectangle in{};
    if (setup.uses_source) {
        in = {input.left < input.right ? input.left : input.right, input.top < input.bottom ? input.top : input.bottom,
              input.left < input.right ? input.right : input.left, input.top < input.bottom ? input.bottom : input.top};
        setup.domain = in;
    } else {
        in = {0, 0, out.right - out.left, out.bottom - out.top};
    }

    if (clip_bounds != nullptr) {
        const Rectangle& b = *clip_bounds;
        if (out.left < b.left) {
            in.left += b.left - out.left;
            out.left = b.left;
        }

        if (b.right < out.right) {
            in.right -= out.right - b.right;
            out.right = b.right;
        }

        if (out.top < b.top) {
            in.top += b.top - out.top;
            out.top = b.top;
        }

        if (b.bottom < out.bottom) {
            in.bottom -= out.bottom - b.bottom;
            out.bottom = b.bottom;
        }
    }

    if (out.right <= out.left || out.bottom <= out.top) {
        return false;
    }

    setup.output = out;
    setup.input = in;
    setup.destination_width = static_cast<int64_t>(out.right) - out.left;
    setup.destination_height = static_cast<int64_t>(out.bottom) - out.top;
    setup.source_width = static_cast<int64_t>(in.right) - in.left;
    setup.source_height = static_cast<int64_t>(in.bottom) - in.top;
    return true;
}

// One DIB_XXBPP_StretchBlt call: destination is the well-ordered CombinedRect
// (the engine passes it swapped per flip; the DIB code re-orders it), source the
// InputToCombinedRect. clip: the DC_RECT bounds or a DC_COMPLEX rectangle, null
// for DC_TRIVIAL. false: the rectangle misses OutputRect (no call).
struct StretchCall final {
    Rectangle destination{};
    Rectangle source{};
};

inline bool stretch_rop_call(const StretchRopSetup& setup, const Rectangle* clip, StretchCall& call) noexcept {
    if (clip == nullptr) {
        call.destination = setup.output;
        call.source = setup.input;
        return true;
    }

    const Rectangle combined = framebuffer::intersect(setup.output, *clip);
    if (combined.left >= combined.right || combined.top >= combined.bottom) {
        return false;
    }

    const Rectangle& out = setup.output;
    const Rectangle& in = setup.input;
    call.destination = combined;
    call.source.top = static_cast<int32_t>(in.top + (static_cast<int64_t>(combined.top) - out.top) *
                                                        setup.source_height / setup.destination_height);
    call.source.bottom = static_cast<int32_t>(in.top + (static_cast<int64_t>(combined.bottom) - out.top) *
                                                           setup.source_height / setup.destination_height);
    call.source.left = static_cast<int32_t>(in.left + (static_cast<int64_t>(combined.left) - out.left) *
                                                          setup.source_width / setup.destination_width);
    call.source.right = static_cast<int32_t>(in.left + (static_cast<int64_t>(combined.right) - out.left) *
                                                           setup.source_width / setup.destination_width);
    return true;
}

// One axis of a DIB call: destination origin and extent, SourceRect low edge and
// extent, mirrored or not. The sample of destination coordinate d is
// low + q or low + extent - q with q = (d - origin) * extent / den, truncated
// toward zero as C's LONG division does (d - origin >= 0 and den > 0; extent is
// negative when the unscaled clip pre-adjustment crossed the source edges).
// DIB_XXBPP_StretchBlt recomputes its own orientation from the signs of that
// SourceRect and the flipped CombinedRect, so the DIB-level mirror is the call's
// flip XOR a negative extent. All products stay below 2^31 for admitted calls
// (|extent| and den <= 16384), so the 64-bit evaluation equals the engine's.
struct StretchAxis final {
    int64_t origin = 0;
    int64_t den = 1;
    int64_t low = 0;
    int64_t extent = 0;
    bool mirror = false;
};

inline StretchAxis stretch_axis_x(const StretchCall& call, bool flip) noexcept {
    const int64_t extent = static_cast<int64_t>(call.source.right) - call.source.left;
    return {call.destination.left, static_cast<int64_t>(call.destination.right) - call.destination.left,
            call.source.left, extent, flip != (extent < 0)};
}

inline StretchAxis stretch_axis_y(const StretchCall& call, bool flip) noexcept {
    const int64_t extent = static_cast<int64_t>(call.source.bottom) - call.source.top;
    return {call.destination.top, static_cast<int64_t>(call.destination.bottom) - call.destination.top,
            call.source.top, extent, flip != (extent < 0)};
}

inline int64_t stretch_sample(const StretchAxis& axis, int64_t d) noexcept {
    const int64_t q = (d - axis.origin) * axis.extent / axis.den;
    return axis.mirror ? axis.low + axis.extent - q : axis.low + q;
}

// Inclusive sample range [low, high] of destination coordinates [first, last)
// (first < last); the sample is monotonic in d.
inline void stretch_samples(const StretchAxis& axis, int64_t first, int64_t last, int64_t& low,
                            int64_t& high) noexcept {
    const int64_t a = stretch_sample(axis, first);
    const int64_t b = stretch_sample(axis, last - 1);
    low = a < b ? a : b;
    high = a < b ? b : a;
}

// [low, high] intersected with [limit_low, limit_high): first and count.
inline int64_t clamp_span(int64_t low, int64_t high, int64_t limit_low, int64_t limit_high, int64_t& first) noexcept {
    first = low > limit_low ? low : limit_low;
    const int64_t end = high + 1 < limit_high ? high + 1 : limit_high;
    return end > first ? end - first : 0;
}

// Guarded stretch admission (the GPU result equals the engine fallback): target
// a GPU surface; ROP4 up to 0xffff; a source when the ROP4 reads one; a 1bpp
// mask only with a source (the DIB code indexes it with the source sample); a
// realized 8x8 tile for a pattern brush. With a mask and a tile the DIB code's
// PatternX advances only for drawn pixels: the plan sends that running phase
// (stretch_flag_running_phase), which the rop kernel counts per row.
constexpr bool stretch_rop_admitted(uint32_t rop4, SurfaceKind target, RasterSource source, bool masked,
                                    bool mask_mono, bool tile_needed, bool tile) noexcept {
    return rop4 <= 0xffffU && gpu_surface(target) && (!masked || (mask_mono && rop4_uses_source(rop4))) &&
           (!rop4_uses_source(rop4) || (source != RasterSource::none && source != RasterSource::unsupported)) &&
           (!tile_needed || tile);
}

// ---------------------------------------------------------------------------
// Guarded stretch band planner, shared by radeondisp and its host tests. One
// operation fixes a StretchSourcePlan; every DIB call (stretch_rop_call) is cut
// into destination row bands whose staged source and mask spans fit one staging
// half (column strips when even one full-width row does not), and each band
// becomes one 0xa17 request. A staged span holds exactly the samples of its band
// that exist in the engine's sample domain; inside the domain but outside the
// bitmap (an IntEngEnter temporary of a device-managed source, which
// GDIOBJ_AllocateObject zero-fills) the staged pixel is 0.
// ---------------------------------------------------------------------------
enum class StretchOperand : uint32_t { none, surface, staged };

struct StretchSourcePlan final {
    StretchOperand operand = StretchOperand::none;
    uint32_t format = 0;          // staged: abi::format_xrgb8888, mono1 or a translated format
    uint32_t surface = 0;         // surface operand: the miniport handle (live source or snapshot)
    Rectangle domain{};           // sample domain in source coordinates
    Rectangle backed{};           // staged: the domain part with bitmap pixels (the rest stages 0)
    int32_t origin_x = 0;         // surface operand pixel = sample - origin
    int32_t origin_y = 0;
    int32_t mask_dx = 0;          // mask pixel = sample - (mask_dx, mask_dy)
    int32_t mask_dy = 0;
    int32_t mask_width = 0;       // 0: no mask
    int32_t mask_height = 0;
    bool running = false;         // tile and mask: stretch_flag_running_phase
};

// One staged operand span of a band (source or mask coordinates).
// Every span lies inside the int32 coordinate range (samples of admitted calls
// stay within |coordinate| + 16384 of 2^24).
struct StretchStage final {
    int32_t x = 0;
    int32_t y = 0;
    int32_t width = 0;
    int32_t height = 0;
    uint32_t offset = 0;          // first row, relative to the band's staging base
    uint32_t pitch = 0;
    bool zero = false;            // no sample in the domain: one zero pixel past every sample
};

struct StretchBand final {
    Rectangle rows{};             // destination band (inside the visible DIB call)
    bool draws = true;            // false: no mask sample in the band (nothing is drawn)
    StretchStage source{};        // staged source (StretchOperand::staged only)
    StretchStage mask{};          // staged mask (mask_width != 0 only)
    uint32_t table_offset = 0;    // translated source: its table, before the rows
    uint32_t bytes = 0;           // staging bytes of the band
    // A mask sample may land exactly on column mask_width or row mask_height,
    // where the two pinned engines differ (a97c8a2 skips the pixel; 822e864
    // tests "cx < sx", reads bit cx of the padding or the next row, and row cy
    // past the bitmap: undefined memory). Such a band is declined.
    bool mask_edge = false;
};

enum class StretchPlanResult : uint32_t { done, unsupported, stopped };

// A GPU source window is read live only when it lies wholly on its surface
// (the dispatcher's window rule; outside it the engine's temporary holds zeros).
inline bool stretch_window_on_surface(const StretchSourcePlan& plan) noexcept {
    return plan.backed.left == plan.domain.left && plan.backed.top == plan.domain.top &&
           plan.backed.right == plan.domain.right && plan.backed.bottom == plan.domain.bottom;
}

// A same-surface DIB call meeting the window needs the engine's snapshot: the
// GPU would otherwise read pixels earlier requests of the operation rewrote (and
// the dispatcher refuses a live window meeting the request bounds).
inline bool stretch_call_meets_window(const StretchSourcePlan& plan, const StretchCall& call) noexcept {
    return !framebuffer::empty(framebuffer::intersect(plan.domain, call.destination));
}

// The IntEngEnter temporary of a GPU source window as GPU work (radeondisp
// snapshot_window): the window minus backed (the part on the surface) is filled
// with 0 and backed is copied from the source, both in snapshot coordinates
// (snapshot pixel (0, 0) is domain.left/top; the copy reads snapshot pixel +
// that origin). Returns the number of zero rectangles (0..4); copy is empty
// when the window misses the surface.
inline uint32_t snapshot_rects(const Rectangle& domain, const Rectangle& backed, Rectangle (&zeros)[4],
                               Rectangle& copy) noexcept {
    const Rectangle window{0, 0, domain.right - domain.left, domain.bottom - domain.top};
    copy = framebuffer::empty(backed) ? Rectangle{} :
        Rectangle{backed.left - domain.left, backed.top - domain.top, backed.right - domain.left,
                  backed.bottom - domain.top};
    if (framebuffer::empty(copy)) {
        copy = {};
        zeros[0] = window;
        return 1;
    }

    const Rectangle parts[4] = {{0, 0, window.right, copy.top}, {0, copy.bottom, window.right, window.bottom},
                                {0, copy.top, copy.left, copy.bottom}, {copy.right, copy.top, window.right, copy.bottom}};
    uint32_t count = 0;
    for (const Rectangle& part : parts) {
        if (!framebuffer::empty(part)) {
            zeros[count++] = part;
        }
    }

    return count;
}

constexpr uint32_t staged_bytes_per_pixel(uint32_t format) noexcept {
    return format == abi::format_index8 ? 1U : format == abi::format_rgb16 ? 2U :
           format == abi::format_rgb24 ? 3U : abi::bytes_per_pixel;
}

// Staging row pitch: 4 * width for xrgb8888/mono1 (the ABI rule), the pixel
// bytes rounded up to four for translated formats.
constexpr uint64_t staged_pitch(uint32_t format, int64_t width) noexcept {
    return abi::translated_format(format) ?
        ((static_cast<uint64_t>(width) * staged_bytes_per_pixel(format) + 3U) & ~uint64_t{3}) :
        static_cast<uint64_t>(width) * abi::bytes_per_pixel;
}

// Spans, offsets and byte count of band (rows set by the caller).
inline void stretch_band_layout(const StretchAxis& x_axis, const StretchAxis& y_axis, const StretchCall& call,
                                const StretchSourcePlan& plan, StretchBand& band) noexcept {
    int64_t x_low = 0, x_high = 0, y_low = 0, y_high = 0;
    stretch_samples(x_axis, band.rows.left, band.rows.right, x_low, x_high);
    stretch_samples(y_axis, band.rows.top, band.rows.bottom, y_low, y_high);
    band.source = {};
    band.mask = {};
    band.draws = true;
    band.mask_edge = false;
    band.table_offset = 0;
    uint64_t bytes = 0;
    if (plan.operand == StretchOperand::staged) {
        StretchStage& stage = band.source;
        int64_t first_x = 0;
        int64_t first_y = 0;
        stage.width = static_cast<int32_t>(clamp_span(x_low, x_high, plan.domain.left, plan.domain.right, first_x));
        stage.height = static_cast<int32_t>(clamp_span(y_low, y_high, plan.domain.top, plan.domain.bottom, first_y));
        stage.x = static_cast<int32_t>(first_x);
        stage.y = static_cast<int32_t>(first_y);
        if (stage.width == 0 || stage.height == 0) {
            stage.x = static_cast<int32_t>(x_high + 1);
            stage.y = static_cast<int32_t>(y_high + 1);
            stage.width = stage.height = 1;
            stage.zero = true;
        }

        bytes = abi::staging_table_bytes(plan.format);
        stage.offset = static_cast<uint32_t>(bytes);
        stage.pitch = static_cast<uint32_t>(staged_pitch(plan.format, stage.width));
        bytes += static_cast<uint64_t>(stage.pitch) * static_cast<uint64_t>(stage.height);
    }

    if (plan.mask_width != 0) {
        int64_t first = 0;
        const int64_t own = clamp_span(x_low - plan.mask_dx, x_high - plan.mask_dx, 0, plan.mask_width, first);
        StretchStage& stage = band.mask;
        if (plan.running) {
            // The kernel counts drawn pixels from the DIB call's left edge.
            int64_t run_low = 0, run_high = 0;
            stretch_samples(x_axis, call.destination.left, band.rows.right, run_low, run_high);
            x_low = run_low;
            x_high = run_high;
        }

        // width is the x span even when no mask row exists (height 0): the
        // strip planner sizes rows with it.
        int64_t first_x = 0;
        int64_t first_y = 0;
        stage.width = static_cast<int32_t>(clamp_span(x_low - plan.mask_dx, x_high - plan.mask_dx, 0, plan.mask_width,
                                                      first_x));
        stage.height = static_cast<int32_t>(clamp_span(y_low - plan.mask_dy, y_high - plan.mask_dy, 0, plan.mask_height,
                                                       first_y));
        stage.x = static_cast<int32_t>(first_x);
        stage.y = static_cast<int32_t>(first_y);
        band.draws = own != 0 && stage.height != 0;
        band.mask_edge = (x_low - plan.mask_dx <= plan.mask_width && plan.mask_width <= x_high - plan.mask_dx) ||
                         (y_low - plan.mask_dy <= plan.mask_height && plan.mask_height <= y_high - plan.mask_dy);
        stage.offset = static_cast<uint32_t>(bytes > UINT32_MAX ? UINT32_MAX : bytes);
        stage.pitch = static_cast<uint32_t>(stage.width) * abi::bytes_per_pixel;
        bytes += static_cast<uint64_t>(stage.pitch) * static_cast<uint64_t>(stage.height);
    }

    band.bytes = bytes > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(bytes);
}

// The bands of one visible DIB call rectangle, top-down, in column strips.
// visit(const StretchBand&) returns false to stop (StretchPlanResult::stopped).
// unsupported: a band cannot fit one staging half (nothing was visited for it).
template <typename Visit>
StretchPlanResult stretch_call_bands(const StretchRopSetup& setup, const StretchCall& call,
                                     const StretchSourcePlan& plan, const Rectangle& visible, uint32_t half_bytes,
                                     Visit&& visit) noexcept {
    const StretchAxis x_axis = stretch_axis_x(call, setup.flip_x);
    const StretchAxis y_axis = stretch_axis_y(call, setup.flip_y);
    const bool grows = plan.operand == StretchOperand::staged || plan.mask_width != 0;
    // Two band records only (probe and growth share trial): the caller runs
    // this inside a kernel-mode stack frame.
    StretchBand band{};
    StretchBand trial{};
    for (int32_t left = visible.left; left < visible.right;) {
        // The widest strip whose single-row band fits (the x spans of a strip
        // are the same in every row; a mask row is assumed present).
        int32_t right = visible.right;
        for (;;) {
            trial.rows = {left, visible.top, right, visible.top + 1};
            stretch_band_layout(x_axis, y_axis, call, plan, trial);
            const uint64_t mask_row = plan.mask_width != 0 && trial.mask.height == 0 ? trial.mask.pitch : 0U;
            if (trial.mask_edge) {
                return StretchPlanResult::unsupported;
            }

            if (trial.bytes + mask_row <= half_bytes) {
                break;
            }

            if (plan.running || right - left == 1) {
                return StretchPlanResult::unsupported;
            }

            right = left + (right - left) / 2;
        }

        for (int32_t top = visible.top; top < visible.bottom;) {
            band.rows = {left, top, right, top + 1};
            stretch_band_layout(x_axis, y_axis, call, plan, band);
            if (band.bytes > half_bytes || band.mask_edge) {
                return StretchPlanResult::unsupported;
            }

            while (grows && band.rows.bottom < visible.bottom) {
                trial.rows = band.rows;
                ++trial.rows.bottom;
                stretch_band_layout(x_axis, y_axis, call, plan, trial);
                if (trial.bytes > half_bytes || trial.mask_edge) {
                    break;
                }

                band = trial;
            }

            if (!grows) {
                band.rows.bottom = visible.bottom;
            }

            top = band.rows.bottom;
            if (band.draws && !visit(band)) {
                return StretchPlanResult::stopped;
            }
        }

        left = right;
    }

    return StretchPlanResult::done;
}

// Fills the band-dependent fields of one 0xa17 request; the caller set session,
// target, rop4, brush, tile flag and pattern. Staged offsets are relative to the
// band's staging base (the caller adds the half's offset).
inline void stretch_band_command(const StretchRopSetup& setup, const StretchCall& call,
                                 const StretchSourcePlan& plan, const StretchBand& band,
                                 abi::StretchRopCommand& command) noexcept {
    const bool mirror_x = stretch_axis_x(call, setup.flip_x).mirror;
    const bool mirror_y = stretch_axis_y(call, setup.flip_y).mirror;
    command.flags = (command.flags & abi::stretch_flag_tile) | (mirror_x ? abi::stretch_flag_mirror_x : 0U) |
                    (mirror_y ? abi::stretch_flag_mirror_y : 0U) |
                    (plan.running ? abi::stretch_flag_running_phase : 0U);
    command.bounds = {call.destination.left, call.destination.top, call.destination.right, call.destination.bottom};
    command.rects = {};
    command.rects.count = 1;
    command.rects.rects[0] = {band.rows.left, band.rows.top, band.rows.right, band.rows.bottom};
    command.source = {};
    command.window = {};
    command.map = {};
    int32_t origin_x = 0;
    int32_t origin_y = 0;
    if (plan.operand == StretchOperand::surface) {
        origin_x = plan.origin_x;
        origin_y = plan.origin_y;
        command.source.kind = abi::operand_surface;
        command.source.handle = plan.surface;
        command.source.format = abi::format_xrgb8888;
        command.window = {plan.domain.left - origin_x, plan.domain.top - origin_y, plan.domain.right - origin_x,
                          plan.domain.bottom - origin_y};
    } else if (plan.operand == StretchOperand::staged) {
        const StretchStage& stage = band.source;
        origin_x = static_cast<int32_t>(stage.x);
        origin_y = static_cast<int32_t>(stage.y);
        command.source.kind = abi::operand_staging;
        command.source.handle = abi::translated_format(plan.format) ? band.table_offset : stage.offset;
        command.source.pitch = stage.pitch;
        command.source.width = static_cast<uint32_t>(stage.width);
        command.source.height = static_cast<uint32_t>(stage.height);
        command.source.format = plan.format;
        command.window = {0, 0, static_cast<int32_t>(stage.width), static_cast<int32_t>(stage.height)};
    }

    if (plan.operand != StretchOperand::none) {
        command.map = {call.source.left - origin_x, call.source.top - origin_y, call.source.right - call.source.left,
                       call.source.bottom - call.source.top};
    }

    command.mask = {};
    command.mask_x = 0;
    command.mask_y = 0;
    if (plan.mask_width != 0) {
        const StretchStage& stage = band.mask;
        command.mask.kind = abi::operand_staging;
        command.mask.handle = stage.offset;
        command.mask.pitch = stage.pitch;
        command.mask.width = static_cast<uint32_t>(stage.width);
        command.mask.height = static_cast<uint32_t>(stage.height);
        command.mask.format = abi::format_mono1;
        // Mask pixel of operand sample u: u + origin - mask_dx - stage origin.
        command.mask_x = origin_x - plan.mask_dx - static_cast<int32_t>(stage.x);
        command.mask_y = origin_y - plan.mask_dy - static_cast<int32_t>(stage.y);
    }
}

// Writes row y (0-based inside the stage) of a staged byte-format source into
// out (stage.pitch bytes): bitmap bytes for the backed part, 0 elsewhere.
// bitmap_row(sy) returns the bitmap row of source line sy (valid for backed
// rows); bytes_per_pixel is the source pixel size (4, 3, 2 or 1).
template <typename Row>
void stage_source_row(const StretchSourcePlan& plan, const StretchStage& stage, int64_t row, uint32_t bytes_per_pixel,
                      Row&& bitmap_row, uint8_t* out) noexcept {
    for (uint32_t index = 0; index < stage.pitch; ++index) {
        out[index] = 0;
    }

    const int64_t sy = stage.y + row;
    if (stage.zero || sy < plan.backed.top || sy >= plan.backed.bottom) {
        return;
    }

    const int64_t first = stage.x > plan.backed.left ? stage.x : plan.backed.left;
    const int64_t last = stage.x + stage.width < plan.backed.right ? stage.x + stage.width : plan.backed.right;
    if (last <= first) {
        return;
    }

    const uint8_t* source = bitmap_row(static_cast<int32_t>(sy));
    for (int64_t x = first; x < last; ++x) {
        for (uint32_t byte = 0; byte < bytes_per_pixel; ++byte) {
            out[static_cast<size_t>(x - stage.x) * bytes_per_pixel + byte] =
                source[static_cast<size_t>(x) * bytes_per_pixel + byte];
        }
    }
}

// The XLATEOBJ table of a translated staged source, written through output
// (staging_table_bytes(format) bytes): index8 and rgb16 sample every pixel
// value, rgb24 the three byte planes (exact only for an OR-linear translation,
// see or_linear_translation). xlate(value) is XLATEOBJ_iXlate.
template <typename Xlate>
bool write_translation_table(uint32_t format, Xlate&& xlate, const sdma::CommandBuffer& output) noexcept {
    if (output.size() != abi::staging_table_bytes(format) || output.size() == 0) {
        return false;
    }

    if (format == abi::format_rgb24) {
        for (uint32_t plane = 0; plane < 3; ++plane) {
            for (uint32_t value = 0; value < 256; ++value) {
                if (!output.write_u32((plane * 256U + value) * 4U, xlate(value << (8U * plane)))) {
                    return false;
                }
            }
        }

        return true;
    }

    const uint32_t entries = static_cast<uint32_t>(output.size() / 4U);
    for (uint32_t value = 0; value < entries; ++value) {
        if (!output.write_u32(value * 4U, xlate(value))) {
            return false;
        }
    }

    return true;
}

// pinned win32ss/gdi/eng/xlateobj.c EXLATEOBJ_vInitialize: for these palette
// types it selects EXLATEOBJ_iXlateTrivial, iXlateRGBtoBGR or
// iXlateShiftAndMask, which are linear over bitwise OR (masks and rotations
// distribute over OR), so a 24-bit pixel b0 | b1 << 8 | b2 << 16 translates to
// T0[b0] | T1[b1] | T2[b2]. Indexed and monochrome palettes (tables, nearest
// colour, thresholds) and the RGB16_555/565 optimizations (which win32k sets on
// BITFIELDS palettes with exactly those masks, ntgdi/palette.c, and which
// XLATEOBJ.iSrcType's 16 bits cannot show) are refused: callers pass
// *_rgb16 = true when the BITFIELDS masks are 555 or 565.
constexpr uint32_t palette_indexed = 0x1U;
constexpr uint32_t palette_bitfields = 0x2U;
constexpr uint32_t palette_rgb = 0x4U;
constexpr uint32_t palette_bgr = 0x8U;
constexpr uint32_t palette_monochrome = 0x2000U;
constexpr bool or_linear_translation(bool trivial, uint32_t source_type, uint32_t destination_type,
                                     bool source_rgb16, bool destination_rgb16) noexcept {
    if (trivial) {
        return true;
    }

    if (((source_type | destination_type) & (palette_monochrome | palette_indexed)) != 0 || destination_rgb16) {
        return false;
    }

    if ((source_type & (palette_rgb | palette_bgr)) != 0) {
        return true;
    }

    return !source_rgb16 && (source_type & palette_bitfields) != 0;
}

constexpr bool rgb16_masks(uint32_t red, uint32_t green, uint32_t blue) noexcept {
    return blue == 0x1fU && ((red == 0x7c00U && green == 0x3e0U) || (red == 0xf800U && green == 0x7e0U));
}

// Staging rows of a compute operation band: bytes per destination row of the
// staged source (4 * width) plus the staged mask (pitch 4 * width as the engine
// requires for every operand). 0 rows: not even one row fits.
constexpr uint32_t staged_rows(uint32_t source_width, uint32_t mask_width, uint32_t half_bytes) noexcept {
    const uint64_t row = static_cast<uint64_t>(source_width) * abi::bytes_per_pixel +
                         static_cast<uint64_t>(mask_width) * abi::bytes_per_pixel;
    if (row == 0) {
        return abi::max_surface_dimension;
    }

    const uint64_t rows = half_bytes / row;
    return rows > abi::max_surface_dimension ? abi::max_surface_dimension : static_cast<uint32_t>(rows);
}

// Columns of a compute operation strip so that one strip row (staged source and
// mask, 4 bytes per pixel each) fits a staging half: the whole width when it
// fits, otherwise the widest strip that does; 0 when not even one column fits.
constexpr uint32_t staged_strip_columns(bool staged_source, bool staged_mask, uint32_t width,
                                        uint32_t half_bytes) noexcept {
    const uint32_t per_column = (staged_source ? abi::bytes_per_pixel : 0U) + (staged_mask ? abi::bytes_per_pixel : 0U);
    if (per_column == 0) {
        return width;
    }

    const uint32_t columns = half_bytes / per_column;
    return columns >= width ? width : columns;
}

} // namespace rx7900::reactos::radeon_display

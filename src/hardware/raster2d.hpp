/*
Module: rx7900.hardware.raster2d
Purpose: Physical Navi31 compute 2D engine for the GDI operations SDMA cannot do:
general ROP3/ROP4, 8x8 pattern brushes, monochrome and coverage8 glyph expansion,
colour-key blits, AlphaBlend, nearest StretchBlt, styled cosmetic lines and integer
gradient fills, over 32-bpp linear BGRX/BGRA surfaces with clip lists (<=64).
Created: 2026-10-22
Architecture: Pure validation, GDI-exact argument building and PM4 IB encoding plus
Raster2dEngine, which writes compute IBs (SET_SH_REG of COMPUTE_PGM_* and
DISPATCH_DIRECT) into a caller-provided GTT allocation and submits them through a
caller-supplied kernel-ring submitter, the same shape as Blit2dSubmitter (normally
an adapter over Gfx11Queues::submit on a compute ring and FenceManager
query/wait). The submit frame (gfx11_emit_ib_frame, mem_sync) already brackets the
IB with an ACQUIRE_MEM cache invalidate and a RELEASE_MEM CACHE_FLUSH_AND_INV TS
fence with GL2 writeback, so the compute results are in memory and the fence is
raised only after the dispatches drain. Surface/rect/clip types are shared with
blit2d.hpp so one GDI backend can route an operation to either engine.
Kernels and their argument ABI: raster2d_shaders.hpp. No MMIO, OS service, heap or
Gfx11Queues object is used here. GDI semantics reproduce the pinned ReactOS 0.4.16
engine (GPL-2.0-or-later behaviour; see raster2d_shaders.cpp provenance).
*/
#pragma once

#include "blit2d.hpp"
#include "raster2d_shaders.hpp"

namespace rx7900::hardware {

// Rect/point/clip reuse: a single GDI backend builds these once and routes to SDMA
// (blit2d) or compute (raster2d).
using Raster2dRect = Blit2dRect;
using Raster2dPoint = Blit2dPoint;
using Raster2dClipList = Blit2dClipList;

constexpr uint32_t raster2d_max_rects = blit2d_max_rects;
constexpr uint32_t raster2d_bytes_per_pixel = 4;
// GFX11 2D image limit, also the largest coordinate the kernels address safely.
constexpr uint32_t raster2d_max_surface_dimension = 16384;

// Operand pixel formats. bgrx8888: 32-bpp BGRX/BGRA (destinations, colour
// sources, AlphaBlend sources). mono1: 1 bit per pixel, MSB first, rows of
// pitch_bytes >= ceil(width / 8) (GDI 1-bpp DIBs and glyph masks; any byte
// address). coverage8: one coverage byte per pixel, pitch_bytes >= width (any
// byte address).
// Translated source formats (rop sources only, raster2d_shaders.hpp rop_args):
// index8 one byte, rgb16 one little-endian halfword, rgb24 three bytes per pixel;
// pitch_bytes >= bytes per pixel * width, pitch and address multiples of four.
// Their pixels are translated through Raster2dRopRequest::source_table.
enum class Raster2dFormat : uint32_t { bgrx8888 = 0, mono1 = 1, coverage8 = 2, index8 = 3, rgb16 = 4, rgb24 = 5 };

// A linear surface in the compute queue's VMID0 address space, field-compatible
// with Blit2dSurface ({gpu_address, pitch_bytes, width, height} aggregate
// initialisation keeps working; format defaults to bgrx8888). Operand roles
// determine the required format: a monochrome source and every mask are mono1, a
// coverage8 AlphaBlend source is coverage8, everything else bgrx8888. A caller
// that leaves format at the bgrx8888 default for a mono1/coverage8 role is
// accepted as "format implied by the role" (backward compatible); an explicit
// different format is refused (invalid_surface). pitch_bytes * height must fit
// the 32-bit buffer descriptor NUM_RECORDS; every kernel access is bounded by it.
struct Raster2dSurface final {
    uint64_t gpu_address;
    uint32_t pitch_bytes;
    uint32_t width;
    uint32_t height;
    Raster2dFormat format = Raster2dFormat::bgrx8888;
};

enum class Raster2dStatus {
    success,
    not_initialized,
    invalid_argument,
    invalid_surface,       // address alignment, pitch or dimension of a surface
    invalid_rect,          // right < left or bottom < top
    too_many_rects,        // clip count above raster2d_max_rects
    out_of_bounds,         // a destination rectangle leaves the destination
    invalid_memory,        // Manager retain/describe/resolve failed for engine memory
    engine_memory_overlap, // an operation surface overlaps the code/args/IB memory
    busy,                  // shutdown with work in flight
    faulted,               // sticky submit/wait/fence failure; release_after_reset recovers
    submit_failed,
    fence_timeout,
    fence_failed,
    // A source or mask shares bytes with the destination area being written; the
    // kernels give no intra-dispatch ordering, so the caller must use another path
    // (EngAlphaBlend refuses this case too). Refused before anything is encoded.
    overlapping_surfaces
};

// GDI ROP4 (foreground ROP3 in the low byte, background ROP3 in the next byte).
// A general blit: destination always; source colour or monochrome optional;
// an 8x8 pattern brush or a solid brush; an optional 1-bpp mask (ROP4). Source
// mapping is identity for a 1:1 blit and a scale for StretchBlt COLORONCOLOR
// (nearest). Colour key suppresses writes where the mapped source equals key.
struct Raster2dRopRequest final {
    Raster2dSurface destination;
    Raster2dRect destination_rect;   // the operation bounds; clip intersects it

    bool has_source = false;
    bool source_monochrome = false;  // 1-bpp source expanded with mono0/mono1
    Raster2dSurface source;
    // Nearest source mapping: src_axis = base + sign*floor((dst_axis - dref)*num/den)
    // with dref = destination_rect.left/top, den = its width/height, base =
    // source_left/top and num = source_width/height (>= 0). A 1:1 blit has num =
    // den and sign +1. A mirrored StretchBlt axis (dib/stretchblt.c bLeftToRight /
    // bTopToBottom) has sign -1 and base = SourceRect.right/bottom, so the first
    // destination column reads the sample one past the source rectangle.
    int32_t source_left = 0;         // src x at destination_rect.left (the base)
    int32_t source_top = 0;
    int32_t source_width = 1;        // src span mapped across destination width
    int32_t source_height = 1;
    int32_t source_sign_x = 1;       // +1, or -1 for a mirrored axis
    int32_t source_sign_y = 1;
    uint32_t mono_foreground = 0;    // mono source: colour of a 1 bit
    uint32_t mono_background = 0;    // and of a 0 bit
    bool skip_outside_source = false; // StretchBlt SRCCOPY drops lanes with no source

    bool has_colour_key = false;
    uint32_t colour_key = 0;         // compared after masking off the high byte

    bool has_pattern = false;        // 8x8 colour tile, else solid brush
    const uint32_t* pattern = nullptr; // 64 BGRX dwords (row-major) when has_pattern
    uint32_t brush_colour = 0;        // solid brush / PATCOPY colour
    Raster2dPoint brush_origin{0, 0};

    bool has_mask = false;           // ROP4 1-bpp mask (BltMask)
    Raster2dSurface mask;            // 1-bpp packed, MSB first
    Raster2dPoint mask_origin{0, 0};
    // Stretch mask rule instead of BltMask (requires a source): the mask pixel
    // (sx + mask_origin.x, sy + mask_origin.y) of every mapped source sample
    // (sx, sy) is read; a set bit or a sample outside the mask skips the pixel,
    // and only the low ROP3 byte is evaluated (dib/stretchblt.c DIB_DoRop).
    bool stretch_mask = false;
    // DIB_XXBPP_StretchBlt PatternX with that mask (requires stretch_mask and
    // has_pattern): the tile column advances only for drawn pixels from
    // destination_rect.left (raster2d_shaders.hpp flag_pattern_running).
    bool running_phase = false;
    // Translated source (source.format index8, rgb16 or rgb24): GPU address of
    // the translation table (raster2d_shaders.hpp rop_args layout), 4-byte
    // aligned, ending at or before source.gpu_address; one raw descriptor spans
    // table and pixel rows. The table entry is S exactly as XLATEOBJ_iXlate of the
    // pixel the DIB code reads.
    uint64_t source_table = 0;

    uint32_t rop4 = 0x00cccc;        // SRCCOPY/SRCCOPY
};

// AlphaBlend (EngAlphaBlend AC_SRC_OVER) or coverage8 glyph composition.
struct Raster2dAlphaRequest final {
    Raster2dSurface destination;
    Raster2dRect destination_rect;
    Raster2dSurface source;          // 32-bpp (alpha), or 8-bpp coverage mask
    int32_t source_left = 0;
    int32_t source_top = 0;
    int32_t source_width = 1;
    int32_t source_height = 1;
    uint32_t constant_alpha = 255;   // BLENDFUNCTION.SourceConstantAlpha
    bool source_alpha = false;       // AC_SRC_ALPHA (premultiply by per-pixel alpha)
    bool coverage8 = false;          // 8-bpp coverage glyph blended with solid colour
    uint32_t solid_colour = 0;       // brush colour for coverage8
};

// GDI gradient rectangle (GRADIENT_FILL_RECT_H / _V), integer as EngGradientFill.
struct Raster2dGradientRectRequest final {
    Raster2dSurface destination;
    Raster2dRect destination_rect;
    bool horizontal = true;
    int32_t origin = 0;              // the v1 coordinate on the gradient axis
    uint32_t divisor = 1;            // dy = axis span (abs); 1 when single-colour
    uint16_t colour0[3]{};           // v1 channel >> 8 start values (R,G,B)
    uint16_t colour1[3]{};           // v2 channel >> 8 end values
};

// GDI gradient triangle: per destination row, three precomputed edge spans whose
// [low,high] x-range and per-channel colour ramp the kernel evaluates (the pinned
// eng/gradient.c FILLLINE order, later spans overwrite earlier ones).
struct Raster2dGradientSpan final {
    int32_t low;                     // inclusive span start x (device space)
    int32_t high;                    // inclusive span end x
    bool reverse;                    // colour ramps from high to low
    int32_t bias;                    // numerator bias (= -(gx>>1) folded)
    uint32_t divisor;                // gx (span length), 1 when degenerate
    uint16_t colour[3];              // channel start (>>8) at the ramp origin
    int32_t delta[3];                // |end-start| per channel
    bool negative[3];                // channel ramps downward
};
// Precomputed-rows form (kept for compatibility): rows[y - destination_rect.top]
// holds the spans of device row y. The clip list is honoured (each clip
// rectangle is intersected with destination_rect); row_count must equal the
// destination_rect height and is no longer limited to one IB.
struct Raster2dGradientTriangleRequest final {
    Raster2dSurface destination;
    Raster2dRect destination_rect;   // the rows/columns the rows array covers
    const Raster2dGradientSpan (*rows)[raster2d_triangle_spans];
    uint32_t row_count;
};

// GDI gradient-triangle vertex. x/y are device pixels (GDI dither origin already
// added); red/green/blue are the 16-bit TRIVERTEX channels (kernels use >> 8).
struct Raster2dTriVertex final {
    int32_t x, y;
    uint16_t red, green, blue;
};

// Vertex form (preferred): EngGradientFill GRADIENT_FILL_TRIANGLE for one
// triangle. The engine plans the rows itself in table-sized chunks, so any
// height is accepted; FillRect = clip rectangle & extents & surface, exactly as
// IntEngGradientFillTriangle. Single-colour triangles use the engine's solid span
// rules. Vertex coordinates are limited to +-raster2d_triangle_coordinate_limit
// (bounded CPU edge walking); a triangle with two equal vertices draws nothing
// (EngGradientFill IntEngIsNULLTriangle).
struct Raster2dTriangleRequest final {
    Raster2dSurface destination;
    Raster2dTriVertex vertices[3];
    Raster2dRect extents;            // prclExtents
};
constexpr int32_t raster2d_triangle_coordinate_limit = 1 << 16;

// One cosmetic line segment (EngLineTo), solid, styled or with an 8x8 tile, with a
// pattern-only ROP.
struct Raster2dLineRequest final {
    Raster2dSurface destination;
    Raster2dRect clip_bounds;        // device-space clip rectangle (<= one rect)
    int32_t x0, y0, x1, y1;          // device endpoints, start included, end excluded
    uint32_t colour = 0;             // pen colour (pattern operand of the ROP)
    uint32_t rop3 = 0xf0;            // pen ROP3 over P and D (PATCOPY default)
    bool load_destination = false;   // ROP reads the destination
    bool styled = false;
    const uint8_t* style_bits = nullptr; // styled: 1 bit per major-axis step, MSB first
    uint32_t style_bit_count = 0;
    // 8x8 tile as the ROP pattern operand instead of colour (not with styled):
    // P of pixel (x, y) is pattern[((y - brush_origin.y) & 7) * 8 +
    // ((x - brush_origin.x) & 7)], the DIB pattern brush convention.
    bool has_pattern = false;
    const uint32_t* pattern = nullptr; // 64 BGRX dwords (row-major) when has_pattern
    Raster2dPoint brush_origin{0, 0};
};

enum class Raster2dSubmitResult { submitted, busy, failed };
enum class Raster2dFenceState { signalled, pending, failed };

// Kernel-ring compute submitter, same contract as Blit2dSubmitter: one IB on the
// engine's compute ring returning a fence sequence; non-blocking query; blocking
// wait. An adapter maps Gfx11Queues::submit (ring, ib, offset, dwords) and
// FenceManager query/wait on the matching compute ring index.
struct Raster2dSubmitter final {
    void* context;
    Raster2dSubmitResult (*submit)(void* context, memory::Handle ib, uint64_t ib_offset,
                                   uint32_t ib_dwords, uint32_t& sequence) noexcept;
    Raster2dFenceState (*query)(void* context, uint32_t sequence) noexcept;
    Raster2dFenceState (*wait)(void* context, uint32_t sequence, uint64_t timeout_us) noexcept;
};

// code: one GTT allocation, CPU visible, GPU read (executed): at least
// raster2d_code_bytes, 256-byte aligned GPU address. The engine generates the
// kernels into it at initialize. args: one GTT allocation, CPU visible, GPU read,
// 64-byte aligned, holding one per-dispatch region of raster2d_args_region_bytes
// for each of up to raster2d_max_rects + 1 dispatches (so one operation's
// dispatches never alias). ib: one GTT allocation, CPU visible, GPU read, 32-byte
// aligned, holding ib_count slots of ib_bytes. wait_timeout_us 0 selects
// blit2d_default_wait_us.
struct Raster2dEngineConfig final {
    memory::Manager* manager;
    memory::Handle code;
    memory::Handle args;
    memory::Handle ib;
    uint32_t ib_count;
    uint32_t ib_bytes;
    Raster2dSubmitter submitter;
    uint64_t wait_timeout_us;
    // Opt-in GPU-side ordering of consecutive IBs on the compute ring: every IB
    // starts with EVENT_WRITE CS_PARTIAL_FLUSH + ACQUIRE_MEM (GLK/GLV/GL1
    // invalidate), so a dependent compute operation needs no CPU fence wait.
    // Off by default (unchanged IB stream for existing callers).
    bool order_ibs_on_gpu = false;
};

// One args region per dispatch: the 256-byte argument block, the 256-byte pattern
// tile (rop) or triangle row record, rounded to the args alignment. After the
// raster2d_max_dispatches regions every IB slot owns a gradient-triangle row
// table of raster2d_triangle_table_rows rows (the kernel indexes it by group Y).
constexpr uint32_t raster2d_args_region_bytes = raster2d_args_bytes + raster2d_pattern_bytes +
    raster2d_triangle_row_bytes;
constexpr uint32_t raster2d_max_dispatches = raster2d_max_rects + 1;
constexpr uint32_t raster2d_triangle_table_rows = 256;
constexpr uint32_t raster2d_triangle_table_offset = raster2d_args_region_bytes * raster2d_max_dispatches;
constexpr uint32_t raster2d_args_bytes_total =
    raster2d_triangle_table_offset + raster2d_triangle_table_rows * raster2d_triangle_row_bytes;
static_assert(raster2d_triangle_table_offset % 64 == 0 && raster2d_args_bytes_total % 64 == 0,
              "args regions and the row table stay 64-byte aligned");
// IB budget: the IB prologue (EVENT_WRITE CS_PARTIAL_FLUSH 2 dwords + ACQUIRE_MEM
// 8 dwords), the shared compute-state header (19 dwords), then one
// SET_SH_REG(USER_DATA, 2) and one DISPATCH_DIRECT per dispatch, with room for
// padding to the 8-dword multiple.
constexpr uint32_t raster2d_ib_header_dwords = 32;
constexpr uint32_t raster2d_ib_per_dispatch_dwords = 9;
constexpr uint32_t raster2d_min_ib_dwords =
    raster2d_ib_header_dwords + raster2d_ib_per_dispatch_dwords * raster2d_max_dispatches + 8;
constexpr uint32_t raster2d_min_ib_bytes = (raster2d_min_ib_dwords * 4 + 31u) & ~31u;
constexpr uint32_t raster2d_min_ib_slots = 2;
constexpr uint32_t raster2d_max_ib_slots = 16;

struct Raster2dReport final {
    uint64_t operations = 0;
    uint64_t rop_ops = 0;
    uint64_t alpha_ops = 0;
    uint64_t gradient_ops = 0;
    uint64_t line_ops = 0;
    uint64_t dispatches = 0;
    uint64_t ib_submissions = 0;
    uint64_t submitted_dwords = 0;
    uint64_t waits = 0;
    uint64_t busy_retries = 0;
    uint64_t skipped_empty = 0;
    uint32_t in_flight = 0;
    Raster2dStatus last_failure = Raster2dStatus::success;
    bool faulted = false;
    Raster2dCodeImage code{};
};

// ---------------------------------------------------------------------------
// Pure argument builders (host-testable; no memory or engine state). Each fills a
// raster2d_args_bytes/4 dword block for one dispatch rectangle from the GDI
// parameters and the four surface descriptors, so tests feed the same bytes to
// the kernel interpreter that the engine uploads. rect is one clip rectangle
// already intersected with the operation bounds (and, for stretch, the source).
// ---------------------------------------------------------------------------

// Fills destination/source/mask/pattern V# words (0 for absent operands).
struct Raster2dDescriptors final {
    uint32_t destination[4];
    uint32_t source[4];
    uint32_t mask[4];
    uint32_t pattern[4];
};

// Translated rop source formats and their table sizes (0 for other formats).
bool raster2d_translated_format(Raster2dFormat format) noexcept;
uint32_t raster2d_table_bytes(Raster2dFormat format) noexcept;
// The source V# of a rop request: the surface itself, or for a translated source
// one raw descriptor from source_table over the table and pixel rows.
void raster2d_rop_source_descriptor(const Raster2dRopRequest& request, uint32_t (&words)[4]) noexcept;
void raster2d_build_rop_args(const Raster2dRopRequest& request, const Raster2dRect& rect,
                             const Raster2dDescriptors& descriptors, uint32_t (&args)[64]) noexcept;
void raster2d_build_alpha_args(const Raster2dAlphaRequest& request, const Raster2dRect& rect,
                               const Raster2dDescriptors& descriptors, uint32_t (&args)[64]) noexcept;
void raster2d_build_gradient_rect_args(const Raster2dGradientRectRequest& request,
                                       const Raster2dRect& rect,
                                       const Raster2dDescriptors& descriptors,
                                       uint32_t (&args)[64]) noexcept;
void raster2d_build_line_args(const Raster2dLineRequest& request,
                              const Raster2dDescriptors& descriptors, uint32_t (&args)[64],
                              uint32_t& major_steps) noexcept;

// Decomposes one triangle (EngGradientFill GRADIENT_FILL_TRIANGLE semantics,
// eng/gradient.c IntEngGradientFillTriangle: bubble-sort by (y, x), three edges,
// per-row FILLLINE spans with the same integer colour DDA) into per-row spans for
// rows top..top+count-1 of out (count <= capacity). Returns the filled extents.
// out[r] holds raster2d_triangle_spans spans for device row top + r. Degenerate
// (zero-area) triangles yield count 0. This is pure and host-tested against a
// direct per-pixel port of the pinned engine.
bool raster2d_plan_triangle(const Raster2dTriVertex& v1, const Raster2dTriVertex& v2,
                            const Raster2dTriVertex& v3,
                            Raster2dGradientSpan (*out)[raster2d_triangle_spans], uint32_t capacity,
                            Raster2dRect& extents, uint32_t& count) noexcept;
// Same, starting at device row first_row (rows above it are walked but not
// stored): out[r] is row first_planned + r where first_planned =
// max(first_row, top vertex y). extents.top is first_planned. Removes the need
// for a caller-side full-height buffer.
bool raster2d_plan_triangle(const Raster2dTriVertex& v1, const Raster2dTriVertex& v2,
                            const Raster2dTriVertex& v3, int32_t first_row,
                            Raster2dGradientSpan (*out)[raster2d_triangle_spans], uint32_t capacity,
                            Raster2dRect& extents, uint32_t& count) noexcept;

// Incremental row planner behind raster2d_plan_triangle: sorts the vertices
// (SMALLER by y then x), selects the shaded rules (any channel differs) or the
// single-colour rules, and produces one row of spans per next() call, top down.
// Shaded (IntEngGradientFillTriangle gradient branch): INITLINE sx = x - 1,
// per row DOLINE(0); within [v1.y, v2.y] DOLINE(1), FILLLINE(0,1) -> span 0;
// within [v2.y, v3.y] FILLLINE(0,2) -> span 1, DOLINE(2), FILLLINE(0,2) -> span 2;
// spans inclusive of both ends with the FDOCOL colour DDA. Single colour
// (S_INITLINE sx = x, S_DOLINE, S_FILLLINE): span 0 = edge 0 to edge 1 after
// DOLINE(1), span 1 = edge 0 to edge 2 after DOLINE(2), each the half-open
// DIB_HLine range [min, max) drawn only when non-empty and inside FillRect (the
// pinned build's Rx7900GradientSpanVisible guard), colour v1 >> 8.
class Raster2dTrianglePlanner final {
public:
    void start(const Raster2dTriVertex& a, const Raster2dTriVertex& b,
               const Raster2dTriVertex& c) noexcept;
    int32_t top() const noexcept { return top_; }
    int32_t end() const noexcept { return end_; }      // exclusive: v3.y
    int32_t row() const noexcept { return row_; }      // next row next() produces
    bool solid() const noexcept { return solid_; }
    void next(Raster2dGradientSpan (&spans)[raster2d_triangle_spans]) noexcept;
    void skip_to(int32_t row) noexcept;

private:
    struct Edge final {
        int32_t x, sx, dx, dy, incx, ex, destx, y0, y1;
        int32_t c[3], dc[3], ec[3], ic[3];
    };

    void init_edge(Edge& e, const Raster2dTriVertex& a, const Raster2dTriVertex& b) noexcept;
    void step_edge(Edge& e) noexcept;
    void fill(const Edge& from, const Edge& to, Raster2dGradientSpan& span) const noexcept;

    Raster2dTriVertex v_[3]{};
    Edge e_[3]{};
    int32_t top_ = 0;
    int32_t end_ = 0;
    int32_t row_ = 0;
    bool solid_ = false;
};

// Device-owned, PASSIVE_LEVEL, externally serialized. Operations validate surfaces,
// rectangles, bounds and engine-memory aliasing, build GDI-exact arguments, upload
// them to the args allocation, encode the compute IB and submit it; a refused
// operation changes nothing. synchronize waits for every submitted IB.
class Raster2dEngine final {
public:
    Raster2dEngine() noexcept = default;
    Raster2dEngine(const Raster2dEngine&) = delete;
    Raster2dEngine& operator=(const Raster2dEngine&) = delete;

    // Validates and retains leases (code read, args read, ib read), resolves their
    // GPU addresses and generates the kernels into the code allocation.
    // Submissions pass ib_offset as the BYTE offset of the IB slot inside the ib
    // allocation (slot i at i * ib_bytes), the Blit2dSubmitter/Gfx11Submission
    // convention; IBs are padded to 8 dwords with PACKET3(NOP, 0x3FFF).
    Raster2dStatus initialize(const Raster2dEngineConfig& config) noexcept;

    Raster2dStatus rop(const Raster2dRopRequest& request, const Raster2dClipList& clip) noexcept;
    Raster2dStatus alpha(const Raster2dAlphaRequest& request,
                         const Raster2dClipList& clip) noexcept;
    Raster2dStatus gradient_rect(const Raster2dGradientRectRequest& request,
                                 const Raster2dClipList& clip) noexcept;
    Raster2dStatus gradient_triangle(const Raster2dGradientTriangleRequest& request,
                                     const Raster2dClipList& clip) noexcept;
    Raster2dStatus gradient_triangle(const Raster2dTriangleRequest& request,
                                     const Raster2dClipList& clip) noexcept;
    Raster2dStatus line(const Raster2dLineRequest& request) noexcept;

    Raster2dStatus flush() noexcept;        // submit the open IB if it holds dispatches
    Raster2dStatus synchronize() noexcept;  // flush, then wait for every IB
    Raster2dStatus retire() noexcept;       // recycle IBs whose fences signalled

    Raster2dStatus shutdown() noexcept;            // requires idle; returns leases
    Raster2dStatus release_after_reset() noexcept; // after confirmed reset quiescence

    Raster2dReport report() const noexcept { return report_; }

    // Grid helper (exposed for tests): group counts for a rectangle.
    static void grid(const Raster2dRect& rect, uint32_t& groups_x, uint32_t& groups_y) noexcept;

private:
    struct IbSlot final {
        uint64_t offset;
        uint32_t capacity_dwords;
        uint32_t sequence;
        uint64_t order;
        bool in_flight;
    };

    struct Dispatch final {
        Raster2dKernel kernel;
        uint64_t args_gpu;
        uint32_t groups_x;
        uint32_t groups_y;
    };

    Raster2dStatus usable() const noexcept;
    Raster2dStatus fault(Raster2dStatus reason) noexcept;
    bool validate_surface(const Raster2dSurface& surface, Raster2dFormat role) const noexcept;
    Raster2dStatus triangle_chunk(const Raster2dSurface& destination, const Raster2dRect* fills,
                                  uint32_t fill_count, int32_t first, int32_t last,
                                  Raster2dTrianglePlanner* planner,
                                  const Raster2dGradientSpan (*rows)[raster2d_triangle_spans],
                                  int32_t rows_top) noexcept;
    bool surface_touches_engine(const Raster2dSurface& surface) const noexcept;
    void descriptor(const Raster2dSurface& surface, uint32_t (&words)[4]) const noexcept;
    Raster2dStatus begin_operation(uint32_t dispatch_cap) noexcept;
    Raster2dStatus stage_args(const uint32_t (&args)[64], const uint32_t* tail, uint32_t tail_dwords,
                              uint64_t& args_gpu) noexcept;
    Raster2dStatus queue(Raster2dKernel kernel, const Raster2dRect& rect, uint64_t args_gpu) noexcept;
    Raster2dStatus submit_open() noexcept;
    Raster2dStatus acquire_slot() noexcept;
    Raster2dStatus wait_slot(IbSlot& slot) noexcept;
    void refresh_completion() noexcept;
    void release_leases() noexcept;

    memory::Manager* manager_ = nullptr;
    Raster2dSubmitter submitter_{};
    uint64_t wait_timeout_us_ = 0;
    bool order_ibs_on_gpu_ = false;
    memory::Handle code_handle_{};
    memory::Handle args_handle_{};
    memory::Handle ib_handle_{};
    uint64_t code_gpu_ = 0;
    uint64_t args_gpu_ = 0;
    uint64_t args_bytes_ = 0;
    uint8_t* args_cpu_ = nullptr;
    uint64_t ib_gpu_ = 0;
    uint8_t* ib_cpu_ = nullptr;
    sdma::CommandBuffer ib_output_{nullptr, 0};
    IbSlot ibs_[raster2d_max_ib_slots]{};
    uint32_t ib_count_ = 0;
    uint32_t next_ = 0;
    uint64_t submit_order_ = 0;
    bool open_ = false;
    uint32_t open_dwords_ = 0;
    uint32_t open_dispatches_ = 0;
    uint32_t args_cursor_ = 0;        // next free args region this operation
    Raster2dCodeImage code_image_{};
    Raster2dReport report_{};
    bool initialized_ = false;
};

} // namespace rx7900::hardware

/*
Module: rx7900.reactos.radeon_bridge
Purpose: Admit one styled cosmetic IntEngLineTo segment for the radeondisp DrvStrokePath.
Created: 2026-10-23
Architecture: Reuses the read-only radeon_line identity and the paired line
MIX tag of framebuffer_stretch_guard.hpp. No retained pointers, locks or
allocations; foreign drivers and plain engine bitmaps are never called.
*/
#include <stdarg.h>
#include <windef.h>
#include <wingdi.h>
#include <winddi.h>
#include <stdint.h>

#include "framebuffer_stretch_guard.hpp"
#include "radeon_bridge.hpp"
#include "radeon_line_identity.hpp"

namespace {

namespace line = rx7900::reactos::radeon_line;

constexpr ULONG copy_pen = R2_COPYPEN;
// POINTFIX is 28.4 fixed point: every admitted coordinate times 16 fits LONG.
constexpr LONG max_fix_coordinate = INT32_MAX / 16;
constexpr LONG min_fix_coordinate = INT32_MIN / 16;

bool fix_representable(LONG value) noexcept {
    return value >= min_fix_coordinate && value <= max_fix_coordinate;
}

// The pinned NWtoSE/SWtoNE/NEtoSW/SEtoNW loops read pulStyles[iStyle] as
// positive run lengths; zero runs stall HandleStyles and stay engine work.
bool positive_runs(ULONG count, const ULONG* runs) noexcept {
    if (runs == nullptr || count == 0 || count > line::max_stroke_style_runs) {
        return false;
    }

    for (ULONG index = 0; index < count; ++index) {
        if (runs[index] == 0 || runs[index] > static_cast<ULONG>(INT32_MAX)) {
            return false;
        }
    }

    return true;
}

// One read-only identity query naming the stroke capability.
bool identified_stroke(SURFOBJ* surface, PFN_DrvEscape escape) noexcept {
    uint32_t requested = line::flag_styled_stroke;
    line::Identity identity{};
    const ULONG bytes = escape(surface, line::identity_escape, sizeof(requested), &requested,
                              sizeof(identity), &identity);
    return bytes == sizeof(identity) &&
           line::admits_requested(identity, line::flag_styled_stroke, line::flag_styled_stroke);
}

} // namespace

extern "C" BOOL APIENTRY Rx7900RadeonStrokePrepare(RX7900_RADEON_STROKE* stroke, SURFOBJ* surface,
    PFN_DrvEscape escape, CLIPOBJ* clip, MIX mix, LONG x1, LONG y1, LONG x2, LONG y2,
    ULONG style_count, const ULONG* styles) noexcept {
    if (stroke == nullptr) {
        return FALSE;
    }

    *stroke = {};
    // Cheap engine-side prefilters before any driver probe. In this engine a
    // DC_TRIVIAL CLIPOBJ carries an empty rclBounds that EngLineTo enumerates
    // (nothing is drawn). DC_RECT and DC_COMPLEX are offered: the octant loops
    // advance the style run once per step and never from the clip walk, so the
    // styled pixel set is the clip-independent style stream restricted to the
    // clip rectangles (radeondisp replays it per clip batch).
    if (surface == nullptr || surface->iType == STYPE_BITMAP || escape == nullptr || clip == nullptr ||
        (clip->iDComplexity != DC_RECT && clip->iDComplexity != DC_COMPLEX) || (mix & 0xffff0000U) != 0 ||
        (mix & 0xffU) != copy_pen || !positive_runs(style_count, styles) ||
        !fix_representable(x1) || !fix_representable(y1) || !fix_representable(x2) ||
        !fix_representable(y2) || (x1 == x2 && y1 == y2)) {
        return FALSE;
    }

    if (!identified_stroke(surface, escape)) {
        return FALSE;
    }

    stroke->points[0] = {x1 * 16, y1 * 16};
    stroke->points[1] = {x2 * 16, y2 * 16};
    stroke->mix = mix | RX7900_LINE_MIX_TAG;
    stroke->style_count = style_count;
    stroke->styles = styles;
    return TRUE;
}

extern "C" BOOL APIENTRY Rx7900RadeonStrokeDraw(const RX7900_RADEON_STROKE* stroke, PFN_DrvStrokePath draw,
    SURFOBJ* surface, PATHOBJ* path, CLIPOBJ* clip, BRUSHOBJ* brush) noexcept {
    if (stroke == nullptr || draw == nullptr || surface == nullptr || path == nullptr ||
        brush == nullptr || !positive_runs(stroke->style_count, stroke->styles) ||
        (stroke->mix & 0xffff0000U) != RX7900_LINE_MIX_TAG) {
        return FALSE;
    }

    // Stack copies live until draw returns; the driver never retains them.
    FLOAT_LONG runs[line::max_stroke_style_runs]{};
    for (ULONG index = 0; index < stroke->style_count; ++index) {
        runs[index].l = static_cast<LONG>(stroke->styles[index]);
    }

    LINEATTRS attributes{};
    attributes.fl = LA_STYLED;
    attributes.iJoin = JOIN_ROUND;
    attributes.iEndCap = ENDCAP_ROUND;
    attributes.elWidth.l = 1;
    attributes.cstyle = stroke->style_count;
    attributes.pstyle = runs;
    attributes.elStyleState.l = 0;
    return draw(surface, path, clip, nullptr, brush, nullptr, &attributes, stroke->mix);
}

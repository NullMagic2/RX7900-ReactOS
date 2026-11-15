/*
Module: rx7900.tests.radeon_bridge
Purpose: Check the win32k radeondisp caller bridge: identification, refusal, unchanged
fallback ownership, exact DrvStrokePath arguments and the styled-segment pixels.
Created: 2026-10-23
Architecture: Actual SDK, production radeon_bridge.cpp and framebuffer_stretch_guard.cpp,
production radeon_line_identity.hpp and radeon_display.hpp style helpers. The engine
reference is a test-only port of the pinned eng/lineto.c styled octant loops (a97c8a2
and 822e864 are identical there); no hardware, VM or kernel is involved.
*/
#include <stdarg.h>
#include <windef.h>
#include <wingdi.h>
#include <winddi.h>
#include "framebuffer_stretch_guard.hpp"
#include "lightning_present_abi.hpp"
#include "radeon_bridge.hpp"
#include "radeon_display.hpp"
#include "radeon_line_identity.hpp"
#include <stdio.h>
#include <string.h>

namespace {

namespace line = rx7900::reactos::radeon_line;
namespace display = rx7900::reactos::radeon_display;
namespace abi = rx7900::reactos::radeon2d::abi;

unsigned checks = 0;
unsigned failures = 0;

void check(bool condition, const char* message) {
    ++checks;
    if (!condition) {
        ++failures;
        printf("FAIL %s\n", message);
    }
}

// ---------------------------------------------------------------------------
// Fake display drivers. radeondisp answers exactly as DrvEscape does: it decodes
// the optional request mask with the production helper and replies with the
// production make_identity.
// ---------------------------------------------------------------------------
enum class Radeon {
    current,         // the live driver contract
    refuses,         // Escape returns 0 (inactive session, or foreign)
    short_reply,     // wrong reply size
    bad_magic,
    bad_version,
    bad_size_field,
    unknown_bit,     // reports a bit nobody defined
    no_stroke,       // lines but no styled-stroke capability (no line IOCTL style)
    v1_only,         // older radeondisp: refuses any query with input
    ignores_request, // older radeondisp: answers v1 regardless of input
    stroke_in_v1,    // malformed: stroke bit in an input-free reply
};

struct EscapeLog {
    unsigned calls = 0;
    ULONG code = 0;
    ULONG input_bytes = 0;
    uint32_t request = 0;
    ULONG output_bytes = 0;
    SURFOBJ* surface = nullptr;
};

Radeon radeon_mode = Radeon::current;
EscapeLog radeon_log{};

ULONG APIENTRY radeon_escape(SURFOBJ* surface, ULONG code, ULONG input_bytes, PVOID input,
                             ULONG output_bytes, PVOID output) {
    ++radeon_log.calls;
    radeon_log.code = code;
    radeon_log.input_bytes = input_bytes;
    radeon_log.output_bytes = output_bytes;
    radeon_log.surface = surface;
    radeon_log.request = 0;
    if (input_bytes == sizeof(uint32_t) && input != nullptr) {
        memcpy(&radeon_log.request, input, sizeof(uint32_t));
    }
    // Lightning's query and every other code: not ours.
    if (code != line::identity_escape || output == nullptr || output_bytes != sizeof(line::Identity) ||
        radeon_mode == Radeon::refuses) {
        return 0;
    }
    uint32_t requested = 0;
    if (!line::decode_request(input_bytes, input, requested)) {
        return 0;
    }
    if (radeon_mode == Radeon::v1_only && input_bytes != 0) {
        return 0;
    }
    if (radeon_mode == Radeon::ignores_request) {
        requested = 0;
    }
    line::Identity reply = line::make_identity(true, true, radeon_mode != Radeon::no_stroke, requested);
    if (radeon_mode == Radeon::bad_magic) {
        ++reply.magic;
    }
    if (radeon_mode == Radeon::bad_version) {
        ++reply.version;
    }
    if (radeon_mode == Radeon::bad_size_field) {
        --reply.size;
    }
    if (radeon_mode == Radeon::unknown_bit) {
        reply.flags |= 8U;
    }
    if (radeon_mode == Radeon::stroke_in_v1) {
        reply.flags |= line::flag_styled_stroke;
    }
    memcpy(output, &reply, sizeof(reply));
    return radeon_mode == Radeon::short_reply ? sizeof(reply) - 1 : sizeof(reply);
}

unsigned foreign_calls = 0;
// A foreign driver that answers every escape with a plausible nonzero count
// and garbage bytes: positive identification must still fail.
ULONG APIENTRY foreign_escape(SURFOBJ*, ULONG, ULONG, PVOID, ULONG output_bytes, PVOID output) {
    ++foreign_calls;
    if (output != nullptr) {
        memset(output, 0xa5, output_bytes);
    }
    return output_bytes;
}

// DrvStrokePath capture.
struct StrokeLog {
    unsigned calls = 0;
    SURFOBJ* surface = nullptr;
    PATHOBJ* path = nullptr;
    CLIPOBJ* clip = nullptr;
    XFORMOBJ* transform = reinterpret_cast<XFORMOBJ*>(1);
    BRUSHOBJ* brush = nullptr;
    POINTL* origin = reinterpret_cast<POINTL*>(1);
    LINEATTRS attributes{};
    LONG runs[line::max_stroke_style_runs]{};
    MIX mix = 0;
};
StrokeLog stroke_log{};
BOOL stroke_result = TRUE;

BOOL APIENTRY stroke_capture(SURFOBJ* surface, PATHOBJ* path, CLIPOBJ* clip, XFORMOBJ* transform,
                             BRUSHOBJ* brush, POINTL* origin, LINEATTRS* attributes, MIX mix) {
    ++stroke_log.calls;
    stroke_log.surface = surface;
    stroke_log.path = path;
    stroke_log.clip = clip;
    stroke_log.transform = transform;
    stroke_log.brush = brush;
    stroke_log.origin = origin;
    stroke_log.mix = mix;
    if (attributes != nullptr) {
        stroke_log.attributes = *attributes;
        for (ULONG index = 0; index < attributes->cstyle && index < line::max_stroke_style_runs; ++index) {
            stroke_log.runs[index] = attributes->pstyle[index].l;
        }
    }
    return stroke_result;
}

constexpr MIX copy_mix = 0x0d0dU;   // ROP2_TO_MIX(R2_COPYPEN), as IntGdiLineTo passes

bool zeroed(const RX7900_RADEON_STROKE& stroke) {
    const RX7900_RADEON_STROKE empty{};
    return memcmp(&stroke, &empty, sizeof(stroke)) == 0;
}

void identity_contract() {
    const line::Identity v1 = line::make_identity(true, true, true, 0);
    check(v1.flags == (line::flag_gpu_lines | line::flag_gpu_stretch) &&
              line::make_identity(true, true).flags == v1.flags,
          "an input-free query never carries a request bit");
    const line::Identity bridge = line::make_identity(true, false, true, line::flag_styled_stroke);
    check(bridge.flags == (line::flag_gpu_lines | line::flag_styled_stroke) &&
              line::admits_requested(bridge, line::flag_styled_stroke, line::flag_styled_stroke),
          "a requested and available stroke capability is reported and admitted");
    check(line::make_identity(true, true, false, line::flag_styled_stroke).flags == v1.flags,
          "an unavailable stroke capability is never reported");
    check(line::make_identity(true, true, true, 0xfffffff8U).flags == v1.flags,
          "unknown request bits are never echoed");
    check(!line::admits(bridge, line::flag_gpu_lines),
          "the version-1 contract refuses an unrequested stroke bit (old kernels stay safe)");
    check(!line::admits_requested(bridge, line::flag_styled_stroke, line::flag_styled_stroke | 8U),
          "a caller cannot request an unknown bit");
    check(!line::admits_requested(v1, line::flag_styled_stroke, line::flag_styled_stroke),
          "a missing stroke bit does not admit a stroke");
    check(line::admits_lines(v1) && line::admits_stretch(v1), "version-1 admission is unchanged");
    uint32_t requested = 99;
    const uint32_t mask = line::flag_styled_stroke;
    check(line::decode_request(0, nullptr, requested) && requested == 0 &&
              line::decode_request(sizeof(mask), &mask, requested) && requested == mask,
          "empty and one-word request inputs decode");
    check(!line::decode_request(3, &mask, requested) && requested == 0 &&
              !line::decode_request(8, &mask, requested) &&
              !line::decode_request(sizeof(mask), nullptr, requested),
          "malformed request inputs are refused without reading beyond them");
}

void prepare_cases() {
    SURFOBJ surface{};
    surface.iType = STYPE_DEVICE;
    CLIPOBJ clip{};
    clip.iDComplexity = DC_RECT;
    clip.rclBounds = {0, 0, 640, 480};
    const ULONG dash[2]{6, 2};
    RX7900_RADEON_STROKE stroke{};

    radeon_mode = Radeon::current;
    radeon_log = {};
    check(Rx7900RadeonStrokePrepare(&stroke, &surface, radeon_escape, &clip, copy_mix, 10, 20, 300, 40, 2, dash),
          "identified radeondisp admits a styled cosmetic segment");
    check(radeon_log.calls == 1 && radeon_log.code == line::identity_escape &&
              radeon_log.input_bytes == sizeof(uint32_t) && radeon_log.request == line::flag_styled_stroke &&
              radeon_log.output_bytes == sizeof(line::Identity) && radeon_log.surface == &surface,
          "one read-only identity query names only the stroke capability");
    check(stroke.points[0].x == 160 && stroke.points[0].y == 320 && stroke.points[1].x == 4800 &&
              stroke.points[1].y == 640,
          "segment endpoints keep IntEngLineTo order as integer POINTFIX");
    check(stroke.mix == (copy_mix | RX7900_LINE_MIX_TAG) && stroke.style_count == 2 && stroke.styles == dash,
          "the paired line tag and the borrowed pen runs are forwarded unchanged");

    const struct {
        Radeon mode;
        const char* message;
    } refusals[] = {
        {Radeon::refuses, "an inactive or refusing radeondisp keeps the engine path"},
        {Radeon::short_reply, "a short identity reply keeps the engine path"},
        {Radeon::bad_magic, "a wrong identity magic keeps the engine path"},
        {Radeon::bad_version, "a wrong identity version keeps the engine path"},
        {Radeon::bad_size_field, "a wrong identity size field keeps the engine path"},
        {Radeon::unknown_bit, "an unknown capability bit is refused"},
        {Radeon::no_stroke, "a radeondisp without the stroke capability keeps the engine path"},
        {Radeon::v1_only, "an older radeondisp that refuses request input keeps the engine path"},
        {Radeon::ignores_request, "an older radeondisp that ignores the request keeps the engine path"},
    };
    for (const auto& refusal : refusals) {
        radeon_mode = refusal.mode;
        stroke.mix = 1;
        check(!Rx7900RadeonStrokePrepare(&stroke, &surface, radeon_escape, &clip, copy_mix, 10, 20, 300, 40, 2,
                                         dash) &&
                  zeroed(stroke),
              refusal.message);
    }
    radeon_mode = Radeon::current;

    foreign_calls = 0;
    check(!Rx7900RadeonStrokePrepare(&stroke, &surface, foreign_escape, &clip, copy_mix, 10, 20, 300, 40, 2, dash) &&
              foreign_calls == 1 && zeroed(stroke),
          "a foreign driver answering garbage is never identified");

    // Engine-side prefilters: none of these probe any driver.
    const unsigned before = radeon_log.calls;
    surface.iType = STYPE_BITMAP;
    check(!Rx7900RadeonStrokePrepare(&stroke, &surface, radeon_escape, &clip, copy_mix, 10, 20, 300, 40, 2, dash),
          "a plain engine bitmap is never probed");
    surface.iType = STYPE_DEVICE;
    check(!Rx7900RadeonStrokePrepare(&stroke, &surface, nullptr, &clip, copy_mix, 10, 20, 300, 40, 2, dash) &&
              !Rx7900RadeonStrokePrepare(&stroke, nullptr, radeon_escape, &clip, copy_mix, 10, 20, 300, 40, 2, dash) &&
              !Rx7900RadeonStrokePrepare(nullptr, &surface, radeon_escape, &clip, copy_mix, 10, 20, 300, 40, 2, dash),
          "missing escape, surface or context is refused");
    check(!Rx7900RadeonStrokePrepare(&stroke, &surface, radeon_escape, nullptr, copy_mix, 10, 20, 300, 40, 2, dash),
          "a missing clip (EngLineTo would clip to RectBounds) stays engine work");
    clip.iDComplexity = DC_TRIVIAL;
    check(!Rx7900RadeonStrokePrepare(&stroke, &surface, radeon_escape, &clip, copy_mix, 10, 20, 300, 40, 2, dash),
          "a DC_TRIVIAL clip (empty bounds in this engine) stays engine work");
    clip.iDComplexity = DC_COMPLEX;
    radeon_mode = Radeon::current;
    const unsigned complex_before = radeon_log.calls;
    check(Rx7900RadeonStrokePrepare(&stroke, &surface, radeon_escape, &clip, copy_mix, 10, 20, 300, 40, 2, dash) &&
              radeon_log.calls == complex_before + 1 && stroke.points[1].x == 4800 && stroke.styles == dash,
          "a complex clip is offered too: the style stream never depends on the clip walk");
    clip.iDComplexity = 7;
    check(!Rx7900RadeonStrokePrepare(&stroke, &surface, radeon_escape, &clip, copy_mix, 10, 20, 300, 40, 2, dash) &&
              radeon_log.calls == complex_before + 1 && zeroed(stroke),
          "an unknown clip complexity is refused before any probe");
    clip.iDComplexity = DC_RECT;
    check(!Rx7900RadeonStrokePrepare(&stroke, &surface, radeon_escape, &clip, 0x0707U, 10, 20, 300, 40, 2, dash) &&
              !Rx7900RadeonStrokePrepare(&stroke, &surface, radeon_escape, &clip, 0x12340d0dU, 10, 20, 300, 40, 2,
                                         dash) &&
              !Rx7900RadeonStrokePrepare(&stroke, &surface, radeon_escape, &clip, copy_mix | RX7900_LINE_MIX_TAG, 10,
                                         20, 300, 40, 2, dash),
          "non-copy, unknown and pre-tagged MIX values are not offered");
    const ULONG zero_run[2]{6, 0};
    const ULONG huge_run[2]{6, 0x80000000U};
    ULONG many[line::max_stroke_style_runs + 1];
    for (ULONG& run : many) {
        run = 1;
    }
    check(!Rx7900RadeonStrokePrepare(&stroke, &surface, radeon_escape, &clip, copy_mix, 10, 20, 300, 40, 0, dash) &&
              !Rx7900RadeonStrokePrepare(&stroke, &surface, radeon_escape, &clip, copy_mix, 10, 20, 300, 40, 2,
                                         nullptr) &&
              !Rx7900RadeonStrokePrepare(&stroke, &surface, radeon_escape, &clip, copy_mix, 10, 20, 300, 40, 2,
                                         zero_run) &&
              !Rx7900RadeonStrokePrepare(&stroke, &surface, radeon_escape, &clip, copy_mix, 10, 20, 300, 40, 2,
                                         huge_run) &&
              !Rx7900RadeonStrokePrepare(&stroke, &surface, radeon_escape, &clip, copy_mix, 10, 20, 300, 40,
                                         line::max_stroke_style_runs + 1, many),
          "unstyled, zero, sign-wrapping and oversized style runs are not offered");
    constexpr LONG fix_max = INT32_MAX / 16;
    constexpr LONG fix_min = INT32_MIN / 16;
    check(!Rx7900RadeonStrokePrepare(&stroke, &surface, radeon_escape, &clip, copy_mix, fix_max + 1, 0, 0, 0, 2,
                                     dash) &&
              !Rx7900RadeonStrokePrepare(&stroke, &surface, radeon_escape, &clip, copy_mix, 0, fix_min - 1, 0, 0, 2,
                                         dash) &&
              !Rx7900RadeonStrokePrepare(&stroke, &surface, radeon_escape, &clip, copy_mix, 0, 0, INT32_MAX, 0, 2,
                                         dash) &&
              !Rx7900RadeonStrokePrepare(&stroke, &surface, radeon_escape, &clip, copy_mix, 7, 7, 7, 7, 2, dash),
          "unrepresentable POINTFIX and degenerate segments are not offered");
    // The admitted DC_COMPLEX call above is the only probe in this block.
    check(radeon_log.calls == before + 1, "no prefiltered call probes the driver");
    check(Rx7900RadeonStrokePrepare(&stroke, &surface, radeon_escape, &clip, copy_mix, fix_max, fix_min, fix_min,
                                    fix_max, line::max_stroke_style_runs, many) &&
              stroke.points[0].x == fix_max * 16 && stroke.points[0].y == fix_min * 16 &&
              stroke.style_count == line::max_stroke_style_runs,
          "the full POINTFIX range and the pen style limit stay admissible to the driver");
}

void draw_cases() {
    SURFOBJ surface{};
    surface.iType = STYPE_DEVICE;
    CLIPOBJ clip{};
    clip.iDComplexity = DC_RECT;
    BRUSHOBJ brush{};
    brush.iSolidColor = 0x00123456U;
    PATHOBJ path{};
    const ULONG runs[6]{3, 1, 1, 1, 1, 1};
    RX7900_RADEON_STROKE stroke{};
    radeon_mode = Radeon::current;
    check(Rx7900RadeonStrokePrepare(&stroke, &surface, radeon_escape, &clip, copy_mix, 1, 2, 3, 40, 6, runs),
          "dash-dot-dot pen is prepared");
    stroke_log = {};
    stroke_result = TRUE;
    check(Rx7900RadeonStrokeDraw(&stroke, stroke_capture, &surface, &path, &clip, &brush) == TRUE &&
              stroke_log.calls == 1,
          "the driver's TRUE is the bridge result");
    check(stroke_log.surface == &surface && stroke_log.path == &path && stroke_log.clip == &clip &&
              stroke_log.brush == &brush && stroke_log.transform == nullptr && stroke_log.origin == nullptr &&
              stroke_log.mix == (copy_mix | RX7900_LINE_MIX_TAG),
          "DrvStrokePath receives the engine's surface, clip and pen brush, no transform or brush origin");
    const LINEATTRS& attributes = stroke_log.attributes;
    bool same_runs = attributes.cstyle == 6;
    for (ULONG index = 0; same_runs && index < 6; ++index) {
        same_runs = stroke_log.runs[index] == static_cast<LONG>(runs[index]);
    }
    check(attributes.fl == LA_STYLED && attributes.elWidth.l == 1 && attributes.elStyleState.l == 0 && same_runs,
          "cosmetic LA_STYLED width-1 attributes carry the exact pen runs from phase zero");
    stroke_result = FALSE;
    check(Rx7900RadeonStrokeDraw(&stroke, stroke_capture, &surface, &path, &clip, &brush) == FALSE &&
              stroke_log.calls == 2,
          "a driver decline is returned so IntEngLineTo continues unchanged");
    stroke_log.calls = 0;
    RX7900_RADEON_STROKE untagged = stroke;
    untagged.mix = copy_mix;
    RX7900_RADEON_STROKE corrupt = stroke;
    corrupt.style_count = line::max_stroke_style_runs + 1;
    check(!Rx7900RadeonStrokeDraw(&untagged, stroke_capture, &surface, &path, &clip, &brush) &&
              !Rx7900RadeonStrokeDraw(&corrupt, stroke_capture, &surface, &path, &clip, &brush) &&
              !Rx7900RadeonStrokeDraw(&stroke, nullptr, &surface, &path, &clip, &brush) &&
              !Rx7900RadeonStrokeDraw(&stroke, stroke_capture, nullptr, &path, &clip, &brush) &&
              !Rx7900RadeonStrokeDraw(&stroke, stroke_capture, &surface, nullptr, &clip, &brush) &&
              !Rx7900RadeonStrokeDraw(&stroke, stroke_capture, &surface, &path, &clip, nullptr) &&
              !Rx7900RadeonStrokeDraw(nullptr, stroke_capture, &surface, &path, &clip, &brush) &&
              stroke_log.calls == 0,
          "unprepared, corrupted or incomplete calls never reach the driver");
}

// The live stretch guard keeps its version-1 query against the new driver contract.
void stretch_guard_compatibility() {
    SURFOBJ surface{};
    surface.iType = STYPE_DEVICE;
    COLORADJUSTMENT adjustment{};
    RX7900_STRETCH_GUARD context{};
    radeon_mode = Radeon::current;
    radeon_log = {};
    check(Rx7900LinePrepare(&surface, radeon_escape, copy_mix, 0) == (copy_mix | RX7900_LINE_MIX_TAG) &&
              radeon_log.input_bytes == 0,
          "unstyled lines are still tagged through the input-free query");
    check(Rx7900StretchGuardPrepare(&context, &surface, radeon_escape, &adjustment, TRUE, FALSE) != &adjustment,
          "the guarded stretch is still tagged through the input-free query");
    radeon_mode = Radeon::stroke_in_v1;
    check(Rx7900LinePrepare(&surface, radeon_escape, copy_mix, 0) == copy_mix &&
              Rx7900StretchGuardPrepare(&context, &surface, radeon_escape, &adjustment, FALSE, FALSE) == &adjustment,
          "a stroke bit in an input-free reply is refused by the version-1 helpers");
    radeon_mode = Radeon::current;
    radeon_log = {};
    check(Rx7900LinePrepare(&surface, radeon_escape, copy_mix, 2) == copy_mix && radeon_log.calls == 0,
          "styled lines remain outside the unstyled tag");
}

// ---------------------------------------------------------------------------
// Test-only port of the pinned eng/lineto.c styled octant loops (HandleStyles,
// NWtoSE, SWtoNE, NEtoSW, SEtoNW) drawing into a DIB, i.e. with zero
// IntEngEnter translation, as radeondisp's DrvLineTo -> EngLineTo fallback
// does, through one DC_RECT clip rectangle (CLIPOBJ_bEnum yields it once).
// ---------------------------------------------------------------------------
constexpr int canvas = 96;
struct Canvas {
    bool pixel[canvas][canvas];
};

void plot(Canvas& target, LONG x, LONG y) {
    if (x >= 0 && x < canvas && y >= 0 && y < canvas) {
        target.pixel[y][x] = true;
    }
}

// translate is IntEngEnter's Translate: zero for a DIB (radeondisp's DrvLineTo
// fallback view), minus the line's DestRect origin for a temporary bitmap.
LONG handle_styles(const ULONG* styles, ULONG count, const POINTL& translate, LONG x, LONG y, LONG deltax,
                   LONG deltay, LONG dx, LONG dy, ULONG& style) {
    ULONG size = 0;
    for (ULONG index = 0; index < count; ++index) {
        size += styles[index];
    }
    LONG offset = 0;
    LONG di = 0;
    LONG max = 0;
    if (deltax > deltay) {
        offset = static_cast<LONG>(static_cast<ULONG>(-translate.x) % size);
        di = dx;
        max = x;
    } else {
        offset = static_cast<LONG>(static_cast<ULONG>(-translate.y) % size);
        di = dy;
        max = y;
    }
    for (style = 0; offset >= static_cast<LONG>(styles[style]); ++style) {
        offset -= static_cast<LONG>(styles[style]);
    }
    if (di > 0) {
        max += static_cast<LONG>(styles[style]) - offset;
    } else {
        max -= offset + 1;
    }
    return max;
}

void engine_styled_line(Canvas& target, const RECTL& clip, LONG x1, LONG y1, LONG x2, LONG y2,
                        const ULONG* styles, ULONG count, bool temporary_bitmap) {
    // EngLineTo's DestRect origin is the lesser coordinate on both axes.
    const POINTL translate = temporary_bitmap ? POINTL{-(x1 < x2 ? x1 : x2), -(y1 < y2 ? y1 : y2)} : POINTL{0, 0};
    LONG x = x1;
    LONG y = y1;
    LONG deltax = x2 - x1;
    LONG deltay = y2 - y1;
    if (deltax == 0 && deltay == 0) {
        return;
    }
    const LONG xchange = deltax < 0 ? -1 : 1;
    const LONG ychange = deltay < 0 ? -1 : 1;
    deltax = deltax < 0 ? -deltax : deltax;
    deltay = deltay < 0 ? -deltay : deltay;
    ULONG style = 0;
    LONG style_max = handle_styles(styles, count, translate, x, y, deltax, deltay, xchange, ychange, style);
    const LONG delta = deltax > deltay ? deltax : deltay;
    LONG error = delta >> 1;
    bool valid = true;   // ClipRect < RectEnum.arcl + RectEnum.c (one rectangle, EnumMore FALSE)
    for (LONG i = 0; i < delta && valid;) {
        // The octant's own "skip to the next clip rect" condition.
        bool skip = false;
        if (xchange > 0 && ychange > 0) {
            skip = clip.bottom <= y || (clip.top <= y && clip.right <= x);
        } else if (xchange > 0) {
            skip = y < clip.top || (y < clip.bottom && clip.right <= x);
        } else if (ychange > 0) {
            skip = clip.bottom <= y || (clip.top <= y && x < clip.left);
        } else {
            skip = y < clip.top || (y < clip.bottom && x < clip.left);
        }
        if (skip) {
            valid = false;
            break;
        }
        bool inside = false;
        if (xchange > 0 && ychange > 0) {
            inside = clip.left <= x && clip.top <= y;
        } else if (xchange > 0) {
            inside = clip.left <= x && y < clip.bottom;
        } else if (ychange > 0) {
            inside = x < clip.right && clip.top <= y;
        } else {
            inside = x < clip.right && y < clip.bottom;
        }
        if (inside && (style & 1U) == 0) {
            plot(target, x, y);
        }
        if (deltax < deltay) {
            y += ychange;
            if (y == style_max) {
                style = ychange > 0 ? (style + 1U) % count : (style - 1U) % count;
                style_max = y + ychange * static_cast<LONG>(styles[style]);
            }
            error += deltax;
            if (deltay <= error) {
                x += xchange;
                error -= deltay;
            }
        } else {
            x += xchange;
            if (x == style_max) {
                style = xchange > 0 ? (style + 1U) % count : (style - 1U) % count;
                style_max = x + xchange * static_cast<LONG>(styles[style]);
            }
            error += deltay;
            if (deltax <= error) {
                y += ychange;
                error -= deltax;
            }
        }
        ++i;
    }
}

// What the driver submits: engine_style_bits over the eng/lineto.c Bresenham
// pixel sequence, start included, end excluded, inside the clip rectangle.
bool driver_styled_line(Canvas& target, const RECTL& clip, LONG x1, LONG y1, LONG x2, LONG y2,
                        const uint32_t* runs, uint32_t count) {
    uint8_t bits[abi::style_bytes]{};
    uint32_t bit_count = 0;
    if (!display::engine_style_bits(x1, y1, x2, y2, runs, count, bits, bit_count)) {
        return false;
    }
    const LONG dx = x2 - x1;
    const LONG dy = y2 - y1;
    const LONG ax = dx < 0 ? -dx : dx;
    const LONG ay = dy < 0 ? -dy : dy;
    const bool x_major = ax >= ay;
    const LONG major = x_major ? ax : ay;
    const LONG minor = x_major ? ay : ax;
    for (LONG k = 0; k < major; ++k) {
        const LONG along = k;
        const LONG across = ((major >> 1) + k * minor) / major;
        const LONG x = x1 + (dx < 0 ? -1 : 1) * (x_major ? along : across);
        const LONG y = y1 + (dy < 0 ? -1 : 1) * (x_major ? across : along);
        const bool on = (bits[k >> 3] & (0x80U >> (k & 7))) != 0;
        if (on && x >= clip.left && x < clip.right && y >= clip.top && y < clip.bottom) {
            plot(target, x, y);
        }
    }
    return true;
}

void phase_cases() {
    static Canvas reference{};
    static Canvas drawn{};
    static Canvas temporary{};
    const ULONG stock[][6] = {{1, 1}, {6, 2}, {3, 2, 1, 2}, {3, 1, 1, 1, 1, 1}, {2, 1}, {5, 3, 2}, {1, 4}};
    const ULONG stock_count[] = {2, 2, 4, 6, 2, 3, 2};
    const RECTL clips[] = {{0, 0, canvas, canvas}, {7, 5, 61, 83}, {30, 30, 31, 90}};
    uint32_t seed = 0x2468ace1U;
    unsigned expressible = 0;
    unsigned mismatch = 0;
    unsigned ties = 0;
    unsigned temporary_mismatch = 0;
    for (unsigned sample = 0; sample < 6000; ++sample) {
        seed = seed * 1664525U + 1013904223U;
        LONG x1 = static_cast<LONG>((seed >> 8) % 90U);
        const LONG y1 = static_cast<LONG>((seed >> 16) % 90U);
        seed = seed * 1664525U + 1013904223U;
        const LONG x2 = static_cast<LONG>((seed >> 8) % 90U);
        const LONG y2 = static_cast<LONG>((seed >> 16) % 90U);
        if (sample % 11 == 0) {
            // Diagonal ties: HandleStyles seeds the y axis while the loop steps x.
            const LONG span = y2 - y1;
            x1 = x2 - (span < 0 ? -span : span);
        }
        const unsigned pen = sample % 7;
        const RECTL& clip = clips[(sample / 7) % 3];
        uint32_t runs[6]{};
        for (ULONG index = 0; index < stock_count[pen]; ++index) {
            runs[index] = stock[pen][index];
        }
        memset(&reference, 0, sizeof(reference));
        memset(&drawn, 0, sizeof(drawn));
        memset(&temporary, 0, sizeof(temporary));
        engine_styled_line(reference, clip, x1, y1, x2, y2, stock[pen], stock_count[pen], false);
        engine_styled_line(temporary, clip, x1, y1, x2, y2, stock[pen], stock_count[pen], true);
        if (!driver_styled_line(drawn, clip, x1, y1, x2, y2, runs, stock_count[pen])) {
            continue;
        }
        ++expressible;
        const LONG dx = x2 - x1;
        const LONG dy = y2 - y1;
        ties += (dx < 0 ? -dx : dx) == (dy < 0 ? -dy : dy) ? 1U : 0U;
        mismatch += memcmp(&reference, &drawn, sizeof(reference)) == 0 ? 0U : 1U;
        temporary_mismatch += memcmp(&temporary, &drawn, sizeof(temporary)) == 0 ? 0U : 1U;
    }
    printf("styled phase sweep: %u expressible segments (%u diagonal ties), %u differ from the pinned DIB "
           "EngLineTo; the temporary-bitmap translation would differ in %u\n", expressible, ties, mismatch,
           temporary_mismatch);
    check(expressible > 5000 && ties > 300 && mismatch == 0,
          "every expressible styled segment equals the pinned DIB EngLineTo pixels, all octants, ties and clips");

    // Independent vector: PS_USERSTYLE {2,1} from x=5 rightwards on the DIB view
    // starts run 0 at the first point (draw, draw, gap, ...).
    const uint32_t runs[2]{2, 1};
    uint8_t bits[abi::style_bytes]{};
    uint32_t count = 0;
    check(display::engine_style_bits(5, 1, 13, 1, runs, 2, bits, count) && count == 8 && bits[0] == 0xdb,
          "the style stream starts at the segment's first point, not the lesser coordinate");
    check(line::max_stroke_style_runs <= display::max_engine_style_runs,
          "every bridge style run fits the driver's style workspace");
}

} // namespace

int main() {
    identity_contract();
    prepare_cases();
    draw_cases();
    stretch_guard_compatibility();
    phase_cases();
    printf("Radeon caller bridge: %u checks, %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

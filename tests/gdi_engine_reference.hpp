/*
Module: rx7900.tests.gdi_engine_reference
Purpose: Direct C++ ports of the pinned ReactOS GDI engine paths that the
radeondisp stretch and line offload must reproduce pixel for pixel.
Created: 2026-10-22
Architecture: Host-only test support shared by raster2d_tests, radeon_2d_ioctl_tests
and radeon_2d_backend_tests. Ports (GPL-2.0-or-later behaviour, structure kept
statement by statement where it matters) of work/reactos a97c8a2:
win32ss/gdi/eng/stretchblt.c IntEngStretchBlt (up to the DrvStretchBltROP call,
including the Case0000..Case1110 +-1 adjustments) and EngStretchBltROP (clip
pre-adjustment, DC_TRIVIAL/DC_RECT/DC_COMPLEX InputToCombinedRect remap, flipped
CombinedRect), CallDibStretchBlt, dib/stretchblt.c DIB_XXBPP_StretchBlt (mask and
pattern phase rules, out-of-source samples), eng/engmisc.c IntEngEnter/IntEngLeave
temporary surfaces for device-managed operands (bytes outside the surface are
the zeros GDIOBJ_AllocateObject writes), eng/xlateobj.c XLATEOBJ_iXlate functions
for 8/16/24bpp sources, and eng/lineto.c EngLineTo (DIB_HLine/DIB_VLine spans and
the four Bresenham octant routines with their clip walking; HandleStyles and the
styled octant loops in eng_line_to_styled). EngLineTo writes the pen's
iSolidColor for every brush (0xffffffff for a pattern brush); the pattern line
reference applies the DIB pattern convention (texel ((x - org.x) mod 8,
(y - org.y) mod 8), as DIB_XXBPP_StretchBlt's PatternX/PatternY) to the same
pixel set.
*/
#pragma once

#include <algorithm>
#include <climits>
#include <cstdint>
#include <cstdlib>
#include <vector>

namespace gdi_reference {

struct Rect final {
    int32_t left = 0, top = 0, right = 0, bottom = 0;
};

struct Point final {
    int32_t x = 0, y = 0;
};

// IntEngEnter temporary bytes outside the device surface. EngCreateBitmap with
// BMF_TOPDOWN | BMF_NOZEROINIT and no pvBits takes SURFACE_AllocSurface's
// "buffer after the object" path: GDIOBJ_AllocObjWithHandle(sizeof(SURFACE) +
// cjBits) -> GDIOBJ_AllocateObject, which RtlZeroMemory()s the whole object
// (pinned ntgdi/gdiobj.c, identical at a97c8a2 and 822e864). BMF_NOZEROINIT
// only skips a second zeroing, so these bytes are defined zeros.
constexpr uint32_t temporary_fill = 0u;

struct Bitmap32 final {
    int32_t cx = 0, cy = 0;
    std::vector<uint32_t> px;

    Bitmap32() = default;
    Bitmap32(int32_t w, int32_t h, uint32_t fill = 0) : cx(w), cy(h), px(size_t(w) * size_t(h), fill) {}
    uint32_t get(int32_t x, int32_t y) const { return px[size_t(y) * size_t(cx) + size_t(x)]; }
    void put(int32_t x, int32_t y, uint32_t v) { px[size_t(y) * size_t(cx) + size_t(x)] = v; }
};

// 1bpp DIB, MSB first, rows of stride bytes.
struct Bitmap1 final {
    int32_t cx = 0, cy = 0;
    uint32_t stride = 0;
    std::vector<uint8_t> bits;

    Bitmap1() = default;
    Bitmap1(int32_t w, int32_t h) : cx(w), cy(h), stride(uint32_t((w + 31) / 32 * 4)), bits(size_t(stride) * size_t(h), 0) {}
    uint32_t get(int32_t x, int32_t y) const { return (bits[size_t(y) * stride + size_t(x) / 8] >> (7 - (x & 7))) & 1u; }
    void set(int32_t x, int32_t y, bool on) {
        uint8_t& b = bits[size_t(y) * stride + size_t(x) / 8];
        const uint8_t m = uint8_t(0x80u >> (x & 7));
        b = on ? uint8_t(b | m) : uint8_t(b & ~m);
    }
};

// An 8, 16 or 24bpp DIB (bytes_per_pixel 1..3), rows of stride bytes; get() is
// DIB_8BPP/16BPP/24BPP_GetPixel (little-endian, 24bpp b0 | b1 << 8 | b2 << 16).
struct BitmapBytes final {
    int32_t cx = 0, cy = 0;
    uint32_t bytes_per_pixel = 1;
    uint32_t stride = 0;
    std::vector<uint8_t> bits;

    BitmapBytes() = default;
    BitmapBytes(int32_t w, int32_t h, uint32_t bpp)
        : cx(w), cy(h), bytes_per_pixel(bpp), stride((uint32_t(w) * bpp + 3u) & ~3u), bits(size_t(stride) * size_t(h), 0) {}
    uint32_t get(int32_t x, int32_t y) const {
        const uint8_t* p = &bits[size_t(y) * stride + size_t(x) * bytes_per_pixel];
        uint32_t value = 0;
        for (uint32_t b = 0; b < bytes_per_pixel; ++b) value |= uint32_t(p[b]) << (8 * b);
        return value;
    }
};

// XLATEOBJ_iXlate of the pinned eng/xlateobj.c functions a 32bpp destination
// can receive: Trivial, Table (iColor >= cEntries -> 0), RGBtoBGR, ShiftAndMask
// (_rotl by the shift then mask, per channel), 555toBGR/565toBGR (5/6-bit to
// 8-bit expansion tables) and 555toRGB.
struct Xlate final {
    enum class Kind { trivial, table, rgb_to_bgr, shift_and_mask, x555_to_bgr, x565_to_bgr, x555_to_rgb } kind = Kind::trivial;
    std::vector<uint32_t> table;
    uint32_t masks[3] = {0, 0, 0};
    uint32_t shifts[3] = {0, 0, 0};

    static uint32_t rotl(uint32_t v, uint32_t s) { s &= 31u; return s == 0 ? v : (v << s) | (v >> (32u - s)); }
    static uint32_t x5to8(uint32_t v) {
        static const uint8_t t[32] = {0, 8, 16, 25, 33, 41, 49, 58, 66, 74, 82, 90, 99, 107, 115, 123,
                                      132, 140, 148, 156, 165, 173, 181, 189, 197, 206, 214, 222, 231, 239, 247, 255};
        return t[v & 31u];
    }
    static uint32_t x6to8(uint32_t v) {
        static const uint8_t t[64] = {0, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 45, 49, 52, 57, 61,
                                      65, 69, 73, 77, 81, 85, 89, 93, 97, 101, 105, 109, 113, 117, 121, 125,
                                      130, 134, 138, 142, 146, 150, 154, 158, 162, 166, 170, 174, 178, 182, 186, 190,
                                      194, 198, 202, 207, 210, 215, 219, 223, 227, 231, 235, 239, 243, 247, 251, 255};
        return t[v & 63u];
    }
    uint32_t operator()(uint32_t c) const {
        switch (kind) {
        case Kind::trivial: return c;
        case Kind::table: return c >= table.size() ? 0u : table[c];
        case Kind::rgb_to_bgr: return (c & 0xff00ff00u) | ((c & 0x00ff00ffu) >> 16) | ((c & 0x00ff00ffu) << 16);
        case Kind::shift_and_mask:
            return (rotl(c, shifts[0]) & masks[0]) | (rotl(c, shifts[1]) & masks[1]) | (rotl(c, shifts[2]) & masks[2]);
        case Kind::x555_to_bgr: return x5to8(c) | (x5to8(c >> 5) << 8) | (x5to8(c >> 10) << 16);
        case Kind::x565_to_bgr: return x5to8(c) | (x6to8(c >> 5) << 8) | (x5to8(c >> 11) << 16);
        case Kind::x555_to_rgb: return (x5to8(c) << 16) | (x5to8(c >> 5) << 8) | x5to8(c >> 10);
        }
        return 0;
    }
};

// A stretch source as EngStretchBltROP sees it: a 32bpp surface (trivial XLATE),
// a 1bpp DIB with its XLATE colours, or an 8/16/24bpp DIB with its XLATEOBJ.
// device: a device-managed 32bpp surface without bits, read through an
// IntEngEnter temporary copy of prclSrc.
struct Source final {
    const Bitmap32* colour = nullptr;
    const Bitmap1* mono = nullptr;
    const BitmapBytes* bytes = nullptr;
    const Xlate* xlate = nullptr;
    uint32_t mono0 = 0, mono1 = 0;
    bool device = false;
};

enum class Complexity { trivial, rect, complex };

// rclBounds and, for complex, the CLIPOBJ_bEnum rectangles in enumeration order.
struct Clip final {
    Complexity kind = Complexity::trivial;
    Rect bounds{INT32_MIN, INT32_MIN, INT32_MAX, INT32_MAX};   // gxcoTrivial
    std::vector<Rect> rects;
};

// BRUSHOBJ: iSolidColor (0xffffffff for a pattern brush) and the engine's 8x8
// pattern surface (BRUSHOBJ_psoPattern), 64 dwords row-major.
struct Brush final {
    uint32_t solid = 0xffffffffu;
    const uint32_t* tile = nullptr;
};

inline uint32_t do_rop(uint32_t rop, uint32_t d, uint32_t s, uint32_t p) {
    const uint32_t rop3 = rop & 0xffu;   // DIB_DoRop: Rop &= 0xFF
    uint32_t r = 0;
    for (uint32_t bit = 0; bit < 32; ++bit) {
        const uint32_t i = (((p >> bit) & 1u) << 2) | (((s >> bit) & 1u) << 1) | ((d >> bit) & 1u);
        r |= ((rop3 >> i) & 1u) << bit;
    }
    return r;
}

inline bool rop3_uses_source(uint32_t r) { return ((r ^ (r >> 2)) & 0x33u) != 0; }
inline bool rop4_uses_source(uint32_t r) { return (((r & 0xccccu) >> 2) != (r & 0x3333u)); }
inline bool rop4_uses_pattern(uint32_t r) { return (((r & 0xf0f0u) >> 4) != (r & 0x0f0fu)); }

inline void make_well_ordered(Rect& r) {
    if (r.left > r.right) { const int32_t t = r.left; r.left = r.right; r.right = t; }
    if (r.top > r.bottom) { const int32_t t = r.top; r.top = r.bottom; r.bottom = t; }
}

inline bool intersect(Rect& out, const Rect& a, const Rect& b) {
    out.left = a.left > b.left ? a.left : b.left;
    out.top = a.top > b.top ? a.top : b.top;
    out.right = a.right < b.right ? a.right : b.right;
    out.bottom = a.bottom < b.bottom ? a.bottom : b.bottom;
    if (out.left >= out.right || out.top >= out.bottom) {
        out = {};
        return false;
    }
    return true;
}

// The source as DIB_XXBPP_StretchBlt reads it (fnSource_GetPixel + XLATE).
struct SourceView final {
    const Bitmap32* colour = nullptr;
    const Bitmap1* mono = nullptr;
    const BitmapBytes* bytes = nullptr;
    const Xlate* xlate = nullptr;
    uint32_t mono0 = 0, mono1 = 0;
    int32_t cx = 0, cy = 0;
    uint32_t get(int32_t x, int32_t y) const {
        if (bytes != nullptr) return (*xlate)(bytes->get(x, y));
        return colour != nullptr ? colour->get(x, y) : (mono->get(x, y) != 0 ? mono1 : mono0);
    }
};

// dib/stretchblt.c DIB_XXBPP_StretchBlt for a 32bpp destination.
inline bool dib_stretch(Bitmap32& dest, const SourceView* src, const Bitmap1* mask, const uint32_t* pattern,
                        Rect DestRect, Rect SourceRect, Point BrushOrigin, const Brush* brush, uint32_t ROP) {
    int32_t sx = 0, sy = 0;
    const bool UsesSource = rop4_uses_source(ROP);
    const bool UsesPattern = rop4_uses_pattern(ROP);
    int32_t DstHeight = DestRect.bottom - DestRect.top;
    int32_t DstWidth = DestRect.right - DestRect.left;
    int32_t SrcHeight = SourceRect.bottom - SourceRect.top;
    int32_t SrcWidth = SourceRect.right - SourceRect.left;
    const bool bLeftToRight = !(((SrcWidth < 0) && (DstWidth < 0)) || ((SrcWidth >= 0) && (DstWidth >= 0)));
    const bool bTopToBottom = !(((SrcHeight < 0) && (DstHeight < 0)) || ((SrcHeight >= 0) && (DstHeight >= 0)));
    make_well_ordered(DestRect);
    DstHeight = DestRect.bottom - DestRect.top;
    DstWidth = DestRect.right - DestRect.left;
    SrcHeight = SourceRect.bottom - SourceRect.top;
    SrcWidth = SourceRect.right - SourceRect.left;
    uint32_t Source = 0, Pattern = 0;
    int32_t PatternX = 0, PatternY = 0;

    if (UsesPattern) {
        if (pattern != nullptr) {
            PatternY = (DestRect.top - BrushOrigin.y) % 8;
            if (PatternY < 0) PatternY += 8;
        } else if (brush != nullptr) {
            Pattern = brush->solid;
        }
    }

    for (int32_t DesY = DestRect.top; DesY < DestRect.bottom; DesY++) {
        if (pattern != nullptr) {
            PatternX = (DestRect.left - BrushOrigin.x) % 8;
            if (PatternX < 0) PatternX += 8;
        }
        if (UsesSource) {
            sy = bTopToBottom ? SourceRect.bottom - (DesY - DestRect.top) * SrcHeight / DstHeight
                              : SourceRect.top + (DesY - DestRect.top) * SrcHeight / DstHeight;
        }
        for (int32_t DesX = DestRect.left; DesX < DestRect.right; DesX++) {
            bool CanDraw = true;
            if (mask != nullptr) {
                sx = bLeftToRight ? SourceRect.right - (DesX - DestRect.left) * SrcWidth / DstWidth
                                  : SourceRect.left + (DesX - DestRect.left) * SrcWidth / DstWidth;
                if (sx < 0 || sy < 0 || mask->cx <= sx || mask->cy <= sy || mask->get(sx, sy) != 0) {
                    CanDraw = false;
                }
            }
            if (UsesSource && CanDraw) {
                sx = bLeftToRight ? SourceRect.right - (DesX - DestRect.left) * SrcWidth / DstWidth
                                  : SourceRect.left + (DesX - DestRect.left) * SrcWidth / DstWidth;
                if (sx >= 0 && sy >= 0 && src->cx > sx && src->cy > sy) {
                    Source = src->get(sx, sy);
                } else {
                    Source = 0;
                    CanDraw = ((ROP & 0xff) != 0xcc);
                }
            }
            if (CanDraw) {
                if (UsesPattern && pattern != nullptr) {
                    Pattern = pattern[PatternY * 8 + PatternX];
                    PatternX++;
                    PatternX %= 8;
                }
                const uint32_t Dest = dest.get(DesX, DesY);
                dest.put(DesX, DesY, do_rop(ROP, Dest, Source, Pattern));
            }
        }
        if (pattern != nullptr) {
            PatternY++;
            PatternY %= 8;
        }
    }
    return true;
}

// One CallDibStretchBlt, recorded in device coordinates (translation removed).
struct DibCall final {
    Rect dest;     // as passed (flipped form)
    Rect source;
};

// eng/stretchblt.c EngStretchBltROP with a device-managed 32bpp destination
// (target) and the source/mask/clip/brush given. Returns the engine result.
inline bool eng_stretch_blt_rop(Bitmap32& target, const Source& source, const Bitmap1* Mask, const Clip* ClipRegion,
                                const Point* BrushOrigin, Rect prclDest, Rect prclSrc, const Brush* pbo,
                                uint32_t Rop4, std::vector<DibCall>* calls = nullptr) {
    Rect InputRect, OutputRect;
    Point TranslateSrc{0, 0}, Translate{0, 0};
    const bool UsesSource = rop4_uses_source(Rop4);
    const int32_t cxSrc = prclSrc.right - prclSrc.left;
    const int32_t cySrc = prclSrc.bottom - prclSrc.top;
    const int32_t cxDest = prclDest.right - prclDest.left;
    const int32_t cyDest = prclDest.bottom - prclDest.top;
    const bool bLeftToRight = !(((cxSrc < 0) && (cxDest < 0)) || ((cxSrc >= 0) && (cxDest >= 0)));
    const bool bTopToBottom = !(((cySrc < 0) && (cyDest < 0)) || ((cySrc >= 0) && (cyDest >= 0)));
    OutputRect = prclDest;
    make_well_ordered(OutputRect);
    prclDest = OutputRect;
    if (Rop4 == 0xaaaa) {
        return true;
    }
    const Complexity clippingType = ClipRegion == nullptr ? Complexity::trivial : ClipRegion->kind;
    OutputRect = prclDest;

    // IntEngEnter of the source: a device source becomes a temporary copy of
    // the (normalized) prclSrc. engmisc.c clips ClippedDestRect to the surface
    // and calls DrvCopyBits only when the clipped part is nonempty (or empty in y
    // with a zero bottom, which copies nothing); the rest keeps temporary_fill.
    Bitmap32 source_temp;
    SourceView input{};
    if (UsesSource) {
        InputRect = prclSrc;
        make_well_ordered(InputRect);
        if (source.device) {
            source_temp = Bitmap32(InputRect.right - InputRect.left, InputRect.bottom - InputRect.top, temporary_fill);
            for (int32_t y = 0; y < source_temp.cy; ++y) {
                for (int32_t x = 0; x < source_temp.cx; ++x) {
                    const int32_t gx = x + InputRect.left, gy = y + InputRect.top;
                    if (gx >= 0 && gy >= 0 && gx < source.colour->cx && gy < source.colour->cy) {
                        source_temp.put(x, y, source.colour->get(gx, gy));
                    }
                }
            }
            TranslateSrc = {-InputRect.left, -InputRect.top};
            input.colour = &source_temp;
            input.cx = source_temp.cx;
            input.cy = source_temp.cy;
        } else {
            input.colour = source.colour;
            input.mono = source.mono;
            input.bytes = source.bytes;
            input.xlate = source.xlate;
            input.mono0 = source.mono0;
            input.mono1 = source.mono1;
            input.cx = source.colour != nullptr ? source.colour->cx : source.mono != nullptr ? source.mono->cx : source.bytes->cx;
            input.cy = source.colour != nullptr ? source.colour->cy : source.mono != nullptr ? source.mono->cy : source.bytes->cy;
        }
        InputRect.left += TranslateSrc.x;
        InputRect.right += TranslateSrc.x;
        InputRect.top += TranslateSrc.y;
        InputRect.bottom += TranslateSrc.y;
    } else {
        InputRect = {0, 0, OutputRect.right - OutputRect.left, OutputRect.bottom - OutputRect.top};
    }

    if (ClipRegion != nullptr) {
        const Rect& b = ClipRegion->bounds;
        if (OutputRect.left < b.left) { InputRect.left += b.left - OutputRect.left; OutputRect.left = b.left; }
        if (b.right < OutputRect.right) { InputRect.right -= OutputRect.right - b.right; OutputRect.right = b.right; }
        if (OutputRect.top < b.top) { InputRect.top += b.top - OutputRect.top; OutputRect.top = b.top; }
        if (b.bottom < OutputRect.bottom) { InputRect.bottom -= OutputRect.bottom - b.bottom; OutputRect.bottom = b.bottom; }
    }

    if (OutputRect.right <= OutputRect.left || OutputRect.bottom <= OutputRect.top) {
        return true;
    }

    // IntEngEnter of the device destination: temporary copy of OutputRect.
    const Rect dest_rect = OutputRect;
    Bitmap32 output(dest_rect.right - dest_rect.left, dest_rect.bottom - dest_rect.top, temporary_fill);
    for (int32_t y = 0; y < output.cy; ++y) {
        for (int32_t x = 0; x < output.cx; ++x) {
            const int32_t gx = x + dest_rect.left, gy = y + dest_rect.top;
            if (gx >= 0 && gy >= 0 && gx < target.cx && gy < target.cy) {
                output.put(x, y, target.get(gx, gy));
            }
        }
    }
    Translate = {-dest_rect.left, -dest_rect.top};
    OutputRect.left += Translate.x;
    OutputRect.right += Translate.x;
    OutputRect.top += Translate.y;
    OutputRect.bottom += Translate.y;
    const Point AdjustedBrushOrigin = BrushOrigin != nullptr ?
        Point{BrushOrigin->x + Translate.x, BrushOrigin->y + Translate.y} : Translate;

    const int32_t DstHeight = OutputRect.bottom - OutputRect.top;
    const int32_t DstWidth = OutputRect.right - OutputRect.left;
    const int32_t SrcHeight = InputRect.bottom - InputRect.top;
    const int32_t SrcWidth = InputRect.right - InputRect.left;

    // CallDibStretchBlt.
    bool Ret = true;
    auto call = [&](Rect d, const Rect& s) {
        const uint32_t* pattern = nullptr;
        if (rop4_uses_pattern(Rop4) && pbo != nullptr && pbo->solid == 0xffffffffu) {
            pattern = pbo->tile;
            if (pattern == nullptr) {
                return false;
            }
        }
        if (calls != nullptr) {
            calls->push_back({{d.left - Translate.x, d.top - Translate.y, d.right - Translate.x, d.bottom - Translate.y},
                              {s.left - TranslateSrc.x, s.top - TranslateSrc.y, s.right - TranslateSrc.x,
                               s.bottom - TranslateSrc.y}});
        }
        return dib_stretch(output, UsesSource ? &input : nullptr, Mask, pattern, d, s, AdjustedBrushOrigin, pbo, Rop4);
    };
    auto per_rect = [&](const Rect& clip_rect) {
        const Rect ClipRect{clip_rect.left + Translate.x, clip_rect.top + Translate.y, clip_rect.right + Translate.x,
                            clip_rect.bottom + Translate.y};
        Rect CombinedRect;
        if (!intersect(CombinedRect, OutputRect, ClipRect)) {
            return;
        }
        Rect ITC;
        ITC.top = InputRect.top + (CombinedRect.top - OutputRect.top) * SrcHeight / DstHeight;
        ITC.bottom = InputRect.top + (CombinedRect.bottom - OutputRect.top) * SrcHeight / DstHeight;
        ITC.left = InputRect.left + (CombinedRect.left - OutputRect.left) * SrcWidth / DstWidth;
        ITC.right = InputRect.left + (CombinedRect.right - OutputRect.left) * SrcWidth / DstWidth;
        if (bLeftToRight) { const int32_t t = CombinedRect.left; CombinedRect.left = CombinedRect.right; CombinedRect.right = t; }
        if (bTopToBottom) { const int32_t t = CombinedRect.top; CombinedRect.top = CombinedRect.bottom; CombinedRect.bottom = t; }
        Ret = call(CombinedRect, ITC);
    };

    switch (clippingType) {
    case Complexity::trivial: {
        if (bLeftToRight) { const int32_t t = OutputRect.left; OutputRect.left = OutputRect.right; OutputRect.right = t; }
        if (bTopToBottom) { const int32_t t = OutputRect.top; OutputRect.top = OutputRect.bottom; OutputRect.bottom = t; }
        Ret = call(OutputRect, InputRect);
        break;
    }
    case Complexity::rect:
        per_rect(ClipRegion->bounds);
        break;
    case Complexity::complex:
        for (const Rect& r : ClipRegion->rects) {
            per_rect(r);
        }
        break;
    }

    // IntEngLeave: copy the temporary back, clipped to the surface.
    for (int32_t y = 0; y < output.cy; ++y) {
        for (int32_t x = 0; x < output.cx; ++x) {
            const int32_t gx = x + dest_rect.left, gy = y + dest_rect.top;
            if (gx >= 0 && gy >= 0 && gx < target.cx && gy < target.cy) {
                target.put(gx, gy, output.get(x, y));
            }
        }
    }
    return Ret;
}

// IntEngStretchBlt up to the DrvStretchBltROP call: the OutputRect/InputRect the
// driver receives and the orientation the stretch guard carries (FLIP_X =
// bLeftToRight, FLIP_Y = bTopToBottom). bitblt: equal sizes go to IntEngBitBlt;
// nothing: everything was clipped away (TRUE without a driver call).
struct DriverStretch final {
    bool bitblt = false;
    bool nothing = false;
    Rect output{};
    Rect input{};
    bool flip_x = false;
    bool flip_y = false;
};

inline DriverStretch int_eng_stretch_blt(Rect DestRect, Rect SourceRect, const Clip& clip) {
    DriverStretch result{};
    const int32_t cxSrc = SourceRect.right - SourceRect.left;
    const int32_t cySrc = SourceRect.bottom - SourceRect.top;
    const int32_t cxDest = DestRect.right - DestRect.left;
    const int32_t cyDest = DestRect.bottom - DestRect.top;
    const bool Case1110 = cxDest > 0 && cyDest > 0 && cxSrc > 0 && cySrc < 0;
    const bool Case1101 = cxDest > 0 && cyDest > 0 && cxSrc < 0 && cySrc > 0;
    const bool Case1100 = cxDest > 0 && cyDest > 0 && cxSrc < 0 && cySrc < 0;
    const bool Case1011 = cxDest > 0 && cyDest < 0 && cxSrc > 0 && cySrc > 0;
    const bool Case1010 = cxDest > 0 && cyDest < 0 && cxSrc > 0 && cySrc < 0;
    const bool Case1001 = cxDest > 0 && cyDest < 0 && cxSrc < 0 && cySrc > 0;
    const bool Case1000 = cxDest > 0 && cyDest < 0 && cxSrc < 0 && cySrc < 0;
    const bool Case0111 = cxDest < 0 && cyDest > 0 && cxSrc > 0 && cySrc > 0;
    const bool Case0110 = cxDest < 0 && cyDest > 0 && cxSrc > 0 && cySrc < 0;
    const bool Case0101 = cxDest < 0 && cyDest > 0 && cxSrc < 0 && cySrc > 0;
    const bool Case0100 = cxDest < 0 && cyDest > 0 && cxSrc < 0 && cySrc < 0;
    const bool Case0011 = cxDest < 0 && cyDest < 0 && cxSrc > 0 && cySrc > 0;
    const bool Case0010 = cxDest < 0 && cyDest < 0 && cxSrc > 0 && cySrc < 0;
    const bool Case0001 = cxDest < 0 && cyDest < 0 && cxSrc < 0 && cySrc > 0;
    const bool Case0000 = cxDest < 0 && cyDest < 0 && cxSrc < 0 && cySrc < 0;
    make_well_ordered(DestRect);
    Rect OutputRect = DestRect;
    result.flip_x = !(((cxSrc < 0) && (cxDest < 0)) || ((cxSrc >= 0) && (cxDest >= 0)));
    result.flip_y = !(((cySrc < 0) && (cyDest < 0)) || ((cySrc >= 0) && (cyDest >= 0)));
    if (std::abs(DestRect.right - DestRect.left) == std::abs(SourceRect.right - SourceRect.left) &&
        std::abs(DestRect.bottom - DestRect.top) == std::abs(SourceRect.bottom - SourceRect.top)) {
        result.bitblt = true;
        return result;
    }
    auto swap_x = [](Rect& r) { const int32_t t = r.left; r.left = r.right; r.right = t; };
    auto swap_y = [](Rect& r) { const int32_t t = r.top; r.top = r.bottom; r.bottom = t; };
    if (cxSrc < 0) { swap_x(SourceRect); swap_x(DestRect); }
    if (cySrc < 0) { swap_y(DestRect); swap_y(SourceRect); }
    if (Case0010 || Case0111) { DestRect.left--; }
    if (Case0010 || Case0111 || Case1000 || Case1101) { SourceRect.left--; SourceRect.right--; }
    if (Case0001 || Case0100 || Case1011 || Case1110) { SourceRect.top--; SourceRect.bottom--; }
    if (Case0011 || Case0110 || Case1001 || Case1100) {
        SourceRect.left--; SourceRect.right--; SourceRect.top--; SourceRect.bottom--;
    }
    if (Case0000 || Case1010) { SourceRect.top++; SourceRect.bottom++; DestRect.top++; DestRect.bottom++; }
    if (Case0000 || Case0101) { SourceRect.left++; SourceRect.right++; DestRect.left++; DestRect.right++; }
    Rect InputClippedRect = DestRect;
    if (InputClippedRect.right < InputClippedRect.left) { InputClippedRect.left = DestRect.right; InputClippedRect.right = DestRect.left; }
    if (InputClippedRect.bottom < InputClippedRect.top) { InputClippedRect.top = DestRect.bottom; InputClippedRect.bottom = DestRect.top; }
    Rect InputRect = SourceRect;
    if (InputRect.right < InputRect.left || InputRect.bottom < InputRect.top) {
        result.nothing = true;
        return result;
    }
    if (clip.kind != Complexity::trivial) {
        if (!intersect(OutputRect, InputClippedRect, clip.bounds)) {
            result.nothing = true;
            return result;
        }
        const int32_t InputClWidth = InputClippedRect.right - InputClippedRect.left;
        const int32_t InputClHeight = InputClippedRect.bottom - InputClippedRect.top;
        const int32_t InputWidth = InputRect.right - InputRect.left;
        const int32_t InputHeight = InputRect.bottom - InputRect.top;
        InputRect.left += (InputWidth * (OutputRect.left - InputClippedRect.left)) / InputClWidth;
        InputRect.right -= (InputWidth * (InputClippedRect.right - OutputRect.right)) / InputClWidth;
        InputRect.top += (InputHeight * (OutputRect.top - InputClippedRect.top)) / InputClHeight;
        InputRect.bottom -= (InputHeight * (InputClippedRect.bottom - OutputRect.bottom)) / InputClHeight;
    } else {
        OutputRect = InputClippedRect;
    }
    result.output = OutputRect;
    result.input = InputRect;
    return result;
}

// The prclDest the engine fallback receives after a FALSE DrvStretchBltROP.
inline Rect fallback_destination(const DriverStretch& call) {
    Rect r = call.output;
    if (call.flip_x) { const int32_t t = r.left; r.left = r.right; r.right = t; }
    if (call.flip_y) { const int32_t t = r.top; r.top = r.bottom; r.bottom = t; }
    return r;
}

// eng/lineto.c EngLineTo pixel set: plot(x, y) for every pixel the engine writes
// (iSolidColor), with clip rectangles in CD_RIGHTDOWN enumeration order (bands
// top-down, left to right) and no style.
template <typename Plot>
void eng_line_to(const std::vector<Rect>& rects, int32_t x1, int32_t y1, int32_t x2, int32_t y2, Plot plot) {
    int32_t x = x1, y = y1, deltax = x2 - x1, deltay = y2 - y1, xchange, ychange, hx, vy;
    if (deltax == 0 && deltay == 0) {
        return;
    }
    if (deltax < 0) { xchange = -1; deltax = -deltax; hx = x2 + 1; } else { xchange = 1; hx = x1; }
    if (deltay < 0) { ychange = -1; deltay = -deltay; vy = y2 + 1; } else { ychange = 1; vy = y1; }
    const size_t c = rects.size();
    if (y1 == y2) {
        for (size_t i = 0; i < c && rects[i].top <= y1; i++) {
            const Rect& r = rects[i];
            const int32_t lo = hx > r.left ? hx : r.left;
            const int32_t hi = hx + deltax < r.right ? hx + deltax : r.right;
            if (y1 < r.bottom && r.left <= hx + deltax && hx < r.right && lo < hi) {
                for (int32_t px = lo; px < hi; ++px) plot(px, y1);
            }
        }
        return;
    }
    if (x1 == x2) {
        for (size_t i = 0; i < c; i++) {
            const Rect& r = rects[i];
            if (r.left <= x1 && x1 < r.right && r.top <= vy + deltay && vy < r.bottom) {
                const int32_t lo = vy > r.top ? vy : r.top;
                const int32_t hi = vy + deltay < r.bottom ? vy + deltay : r.bottom;
                for (int32_t py = lo; py < hi; ++py) plot(x1, py);
            }
        }
        return;
    }
    // NWtoSE / SWtoNE / NEtoSW / SEtoNW: one routine per octant pair, each
    // enumerating in its own CD_* direction (CD_RIGHTDOWN, CD_RIGHTUP,
    // CD_LEFTDOWN, CD_LEFTUP); the clip pointer walks forward past rectangles
    // the line has left. rects is a y-x banded region.
    std::vector<Rect> ordered = rects;
    std::sort(ordered.begin(), ordered.end(), [&](const Rect& a, const Rect& b) {
        if (a.top != b.top) return ychange > 0 ? a.top < b.top : a.top > b.top;
        return xchange > 0 ? a.left < b.left : a.left > b.left;
    });
    const std::vector<Rect>& rects_ = ordered;
    const int32_t delta = deltax > deltay ? deltax : deltay;
    int32_t error = delta >> 1;
    size_t k = 0;
    int32_t i = 0;
    while (i < delta && k < c) {
        auto behind = [&](const Rect& r) {
            if (xchange > 0 && ychange > 0) return r.bottom <= y || (r.top <= y && r.right <= x);
            if (xchange > 0) return y < r.top || (y < r.bottom && r.right <= x);
            if (ychange > 0) return r.bottom <= y || (r.top <= y && x < r.left);
            return y < r.top || (y < r.bottom && x < r.left);
        };
        while (k < c && behind(rects_[k])) {
            ++k;
        }
        if (k >= c) {
            break;
        }
        const Rect& r = rects_[k];
        const bool inside = xchange > 0 && ychange > 0 ? (r.left <= x && r.top <= y) :
                            xchange > 0 ? (r.left <= x && y < r.bottom) :
                            ychange > 0 ? (x < r.right && r.top <= y) : (x < r.right && y < r.bottom);
        if (inside) {
            plot(x, y);
        }
        if (deltax < deltay) {
            y += ychange;
            error = error + deltax;
            if (deltay <= error) { x += xchange; error = error - deltay; }
        } else {
            x += xchange;
            error = error + deltay;
            if (deltax <= error) { y += ychange; error = error - deltax; }
        }
        i++;
    }
}

// eng/lineto.c EngLineTo with a styled pen (dwStyleCount > 0: every direction,
// horizontal and vertical included, runs through NWtoSE/SWtoNE/NEtoSW/SEtoNW).
// translate is IntEngEnter's Translate: (0, 0) for an STYPE_BITMAP destination
// (radeondisp's own DIB view), (-DestRect.left, -DestRect.top) for a temporary
// of a device-managed one. Statement order of HandleStyles and of the four
// octant loops is kept, including the ULONG (iStyle - 1) % cStyles wrap and the
// diagonal tie that seeds lStyleMax on y but steps x. plot(x, y) in device
// coordinates; rects as for eng_line_to (banded, any order).
template <typename Plot>
void eng_line_to_styled(const std::vector<Rect>& rects, int32_t x1, int32_t y1, int32_t x2, int32_t y2,
                        const std::vector<uint32_t>& styles, Point translate, Plot plot) {
    if (x1 == x2 && y1 == y2) {
        return;
    }
    const uint32_t cStyles = uint32_t(styles.size());
    uint32_t ulStyleSize = 0;
    for (uint32_t s : styles) ulStyleSize += s;
    int32_t x = x1 + translate.x, y = y1 + translate.y;
    int32_t deltax = x2 - x1, deltay = y2 - y1;
    const int32_t xchange = deltax < 0 ? -1 : 1, ychange = deltay < 0 ? -1 : 1;
    deltax = deltax < 0 ? -deltax : deltax;
    deltay = deltay < 0 ? -deltay : deltay;
    // HandleStyles(pbo, Translate, x, y, deltax, deltay, dx = xchange, dy = ychange).
    uint32_t iStyle = 0;
    int32_t lStyleMax = INT32_MAX;
    if (cStyles > 0) {
        int32_t offStyle, diStyle;
        if (deltax > deltay) {
            offStyle = int32_t(uint32_t(-translate.x) % ulStyleSize);
            diStyle = xchange;
            lStyleMax = x;
        } else {
            offStyle = int32_t(uint32_t(-translate.y) % ulStyleSize);
            diStyle = ychange;
            lStyleMax = y;
        }
        for (iStyle = 0; uint32_t(offStyle) >= styles[iStyle]; iStyle++) {
            offStyle -= int32_t(styles[iStyle]);
        }
        lStyleMax = diStyle > 0 ? lStyleMax + int32_t(styles[iStyle]) - offStyle : lStyleMax - (offStyle + 1);
    }
    std::vector<Rect> ordered;
    for (const Rect& r : rects) {
        ordered.push_back({r.left + translate.x, r.top + translate.y, r.right + translate.x, r.bottom + translate.y});
    }
    std::sort(ordered.begin(), ordered.end(), [&](const Rect& a, const Rect& b) {
        if (a.top != b.top) return ychange > 0 ? a.top < b.top : a.top > b.top;
        return xchange > 0 ? a.left < b.left : a.left > b.left;
    });
    const size_t c = ordered.size();
    const int32_t delta = deltax > deltay ? deltax : deltay;
    int32_t error = delta >> 1;
    size_t k = 0;
    for (int32_t i = 0; i < delta && k < c; ) {
        auto behind = [&](const Rect& r) {
            if (xchange > 0 && ychange > 0) return r.bottom <= y || (r.top <= y && r.right <= x);
            if (xchange > 0) return y < r.top || (y < r.bottom && r.right <= x);
            if (ychange > 0) return r.bottom <= y || (r.top <= y && x < r.left);
            return y < r.top || (y < r.bottom && x < r.left);
        };
        while (k < c && behind(ordered[k])) {
            ++k;
        }
        if (k >= c) {
            break;
        }
        const Rect& r = ordered[k];
        const bool inside = xchange > 0 && ychange > 0 ? (r.left <= x && r.top <= y) :
                            xchange > 0 ? (r.left <= x && y < r.bottom) :
                            ychange > 0 ? (x < r.right && r.top <= y) : (x < r.right && y < r.bottom);
        if (inside && (iStyle & 1u) == 0) {
            plot(x - translate.x, y - translate.y);
        }
        if (deltax < deltay) {
            y += ychange;
            if (y == lStyleMax) {
                iStyle = ychange > 0 ? (iStyle + 1u) % cStyles : (iStyle - 1u) % cStyles;
                lStyleMax = y + ychange * int32_t(styles[iStyle]);
            }
            error = error + deltax;
            if (deltay <= error) { x += xchange; error = error - deltay; }
        } else {
            x += xchange;
            if (x == lStyleMax) {
                iStyle = xchange > 0 ? (iStyle + 1u) % cStyles : (iStyle - 1u) % cStyles;
                lStyleMax = x + xchange * int32_t(styles[iStyle]);
            }
            error = error + deltay;
            if (deltax <= error) { y += ychange; error = error - deltax; }
        }
        i++;
    }
}

} // namespace gdi_reference

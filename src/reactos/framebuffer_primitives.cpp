/*
Module: lightning.reactos.framebuffer_primitives
Purpose: Reuse engine bitmap ownership and GPU masked pattern composition.
Created: 2026-10-22
Architecture: GDI serializes source callbacks; shadow BitBlt owns canvas locking.
License: GPL-2.0-or-later.
*/
#include "framebuf.h"
#include "framebuffer_shadow.hpp"
#include "framebuffer_primitives.hpp"
#include "framebuffer_stretch_guard.hpp"
#include "engine_bitmap.hpp"
#include "line_coverage.hpp"
#include "hw/misc/lightning-gdi.h"

extern "C" BOOL APIENTRY FramebufferShadowLineTo(SURFOBJ* target,CLIPOBJ* clip,BRUSHOBJ* brush,
    LONG x1,LONG y1,LONG x2,LONG y2,RECTL*,MIX mix) {
    if (target==nullptr || brush==nullptr ||
        target->sizlBitmap.cx<=0 || target->sizlBitmap.cy<=0) return FALSE;
    if ((mix&0xffff0000U)!=RX7900_LINE_MIX_TAG) return FALSE;
    mix&=UINT16_MAX;
    // The paired engine admission retains styled lines in its original path.
    // This callback owns unstyled cosmetic geometry in all octants. A pattern
    // brush is the engine's solid 0xffffffff line (no tile or brush origin).
    // Without a clip object EngLineTo clips to a RectBounds this hook never
    // receives, so that unrealized case stays with the engine.
    using namespace lightning::reactos;
    const bool unrealized=brush->iSolidColor==UINT32_MAX;
    if (unrealized && clip==nullptr) return FALSE;
    LineRectangle bounds{};
    uint32_t rop=line_unrealized_pattern_rop4;
    if ((!unrealized && !line_rop(mix,rop)) || !line_bounds({x1,y1},{x2,y2},target->sizlBitmap.cx,target->sizlBitmap.cy,bounds)) return FALSE;
    if (clip!=nullptr && clip->iDComplexity!=DC_TRIVIAL)
        bounds=rx7900::reactos::framebuffer::intersect(bounds,
            {clip->rclBounds.left,clip->rclBounds.top,clip->rclBounds.right,clip->rclBounds.bottom});
    if (rx7900::reactos::framebuffer::empty(bounds)) return TRUE;
    EngineBitmap mask;
    if (!mask.create(bounds.right-bounds.left,bounds.bottom-bounds.top,BMF_1BPP,LIGHTNING_GDI_MAX_PIXEL_BYTES) ||
        !line_coverage({static_cast<uint8_t*>(mask.surface()->pvScan0),mask.bytes(),mask.stride(),bounds},
            {x1,y1},{x2,y2})) return FALSE;
    RECTL rectangle{bounds.left,bounds.top,bounds.right,bounds.bottom};
    POINTL mask_origin{};
    POINTL brush_origin{};
    return FramebufferShadowBitBlt(target,nullptr,mask.surface(),clip,nullptr,&rectangle,
        nullptr,&mask_origin,brush,&brush_origin,rop);
}


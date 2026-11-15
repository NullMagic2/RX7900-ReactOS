/*
Module: lightning.reactos.brush_origin
Purpose: Preserve signed brush phase without LONG overflow or stale DC caches.
Created: 2026-10-21
Architecture: Caller owns/locks the DC and its mapped attribute storage.
License: GPL-2.0-or-later.
*/
#include <windef.h>
#include <wingdi.h>
#include <stdint.h>
#include "framebuffer_brush_origin.hpp"

extern "C" BOOL APIENTRY Rx7900BrushOrigin(const POINTL* dc_brush,const POINTL* dc_origin,
    POINTL* output) noexcept {
    if (output==nullptr) return FALSE;
    if (dc_brush==nullptr || dc_origin==nullptr) {*output={}; return FALSE;}
    const POINTL brush=*dc_brush;
    const POINTL origin=*dc_origin;
    const int64_t x=static_cast<int64_t>(brush.x)+origin.x;
    const int64_t y=static_cast<int64_t>(brush.y)+origin.y;
    *output={};
    if (x<INT32_MIN || x>INT32_MAX || y<INT32_MIN || y>INT32_MAX) return FALSE;
    *output={static_cast<LONG>(x),static_cast<LONG>(y)};
    return TRUE;
}

extern "C" BOOL APIENTRY Rx7900PatternPhase(LONGLONG coordinate,LONG origin,LONG extent,
    LONG* output) noexcept {
    if (output==nullptr) return FALSE;
    *output=0;
    if (extent<=0 || coordinate<static_cast<int64_t>(INT32_MIN)-1 || coordinate>INT32_MAX) return FALSE;
    // The admitted coordinate domain makes subtraction bounded even for the
    // opposite extreme LONG origin. Match the engine mask path's corrected
    // remainder without ever computing a signed LONG difference first.
    int64_t phase=(coordinate-static_cast<int64_t>(origin))%extent;
    if (phase<0) phase+=extent;
    *output=static_cast<LONG>(phase);
    return TRUE;
}

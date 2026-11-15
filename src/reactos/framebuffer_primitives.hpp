/*
Module: lightning.reactos.framebuffer_primitives
Purpose: Route native primitive coverage through existing GPU mask/brush drawing.
Created: 2026-10-22
Architecture: Include after framebuf native SDK types; no independent canvas.
*/
#pragma once
#ifdef __cplusplus
extern "C" {
#endif
BOOL APIENTRY FramebufferShadowLineTo(SURFOBJ* target,CLIPOBJ* clip,BRUSHOBJ* brush,
    LONG x1,LONG y1,LONG x2,LONG y2,RECTL* bounds,MIX mix);
BOOL APIENTRY FramebufferShadowPaint(SURFOBJ* target,CLIPOBJ* clip,BRUSHOBJ* brush,
    POINTL* origin,MIX mix);
#ifdef __cplusplus
}
#endif

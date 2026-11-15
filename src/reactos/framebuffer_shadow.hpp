/*
Module: rx7900.reactos.framebuffer_shadow
Purpose: Optional cached GDI drawing surface for the ReactOS framebuf driver.
Created: 2026-10-21
Architecture: Native GDI owner; physical mapping remains owned by framebuf PDEV.
License: GPL-2.0-or-later, matching ReactOS framebuf integration.
*/
#pragma once

// Included after framebuf.h by C and C++. The opaque pointer is PDEV-owned.
#ifdef __cplusplus
extern "C" {
#endif
BOOL APIENTRY FramebufferShadowInitialize(VOID);
VOID APIENTRY FramebufferShadowShutdown(VOID);
HSURF APIENTRY FramebufferShadowEnable(PPDEV device, ULONG mapped_bytes);
VOID APIENTRY FramebufferShadowDisable(PPDEV device);
BOOL APIENTRY FramebufferShadowRefresh(PPDEV device);
BOOL APIENTRY FramebufferShadowSuspend(PPDEV device);
#ifdef RX7900_LIGHTNING_DISPLAY
BOOL APIENTRY FramebufferShadowAssertMode(PPDEV device,BOOL enable);
#endif
#ifdef RX7900_LIGHTNING_GDI_OFFLOAD
ULONG APIENTRY FramebufferShadowDrawEscape(SURFOBJ* surface,ULONG escape,
    CLIPOBJ* clip,RECTL* window,ULONG input_bytes,PVOID input);
VOID APIENTRY FramebufferShadowNotify(SURFOBJ* surface,ULONG notification,PVOID data);
BOOL APIENTRY FramebufferShadowRealizeBrush(BRUSHOBJ* brush,SURFOBJ* target,SURFOBJ* pattern,
    SURFOBJ* mask,XLATEOBJ* translation,ULONG hatch);
BOOL APIENTRY FramebufferShadowAlphaBlend(SURFOBJ* target,SURFOBJ* source,
    CLIPOBJ* clip,XLATEOBJ* translation,RECTL* destination_rectangle,
    RECTL* source_rectangle,BLENDOBJ* blend);
BOOL APIENTRY FramebufferShadowStretchBltROP(SURFOBJ* target,SURFOBJ* source,SURFOBJ* mask,
    CLIPOBJ* clip,XLATEOBJ* translation,COLORADJUSTMENT* adjustment,POINTL* brush_origin,
    RECTL* destination_rectangle,RECTL* source_rectangle,POINTL* mask_origin,
    ULONG mode,BRUSHOBJ* brush,ROP4 rop);
BOOL APIENTRY FramebufferShadowTransparentBlt(SURFOBJ* target,SURFOBJ* source,CLIPOBJ* clip,
    XLATEOBJ* translation,RECTL* destination_rectangle,RECTL* source_rectangle,
    ULONG transparent_color,ULONG reserved);
BOOL APIENTRY FramebufferShadowGradientFill(SURFOBJ* target,CLIPOBJ* clip,XLATEOBJ* translation,
    TRIVERTEX* vertices,ULONG vertex_count,PVOID mesh,ULONG mesh_count,
    RECTL* extents,POINTL* origin,ULONG mode);
VOID APIENTRY FramebufferShadowSynchronizeSurface(SURFOBJ* surface,RECTL* rectangle,ULONG flags);
#endif
#ifdef RX7900_LIGHTNING_DISPLAY
ULONG APIENTRY FramebufferShadowSetPointer(SURFOBJ* surface, SURFOBJ* mask,
    SURFOBJ* color, XLATEOBJ* translation, LONG hot_x, LONG hot_y,
    LONG x, LONG y, RECTL* exclusion, FLONG flags);
VOID APIENTRY FramebufferShadowMovePointer(SURFOBJ* surface, LONG x, LONG y,
    RECTL* exclusion);
#endif
ULONG APIENTRY FramebufferShadowEscape(SURFOBJ* surface, ULONG escape,
    ULONG input_bytes, PVOID input, ULONG output_bytes, PVOID output);
BOOL APIENTRY FramebufferShadowCopyBits(SURFOBJ* target, SURFOBJ* source,
    CLIPOBJ* clip, XLATEOBJ* translation, RECTL* rectangle, POINTL* source_point);
BOOL APIENTRY FramebufferShadowBitBlt(SURFOBJ* target, SURFOBJ* source,
    SURFOBJ* mask, CLIPOBJ* clip, XLATEOBJ* translation, RECTL* rectangle,
    POINTL* source_point, POINTL* mask_point, BRUSHOBJ* brush,
    POINTL* brush_origin, ROP4 rop);
#ifdef __cplusplus
}
#endif

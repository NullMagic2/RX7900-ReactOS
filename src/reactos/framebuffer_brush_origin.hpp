/*
Module: lightning.reactos.brush_origin
Purpose: Translate the current DC brush origin into checked surface coordinates.
Created: 2026-10-21
Architecture: Kernel-only borrowed POINTL inputs; no private driver or wire state.
License: GPL-2.0-or-later, matching the ReactOS integration.
*/
#pragma once
#ifdef __cplusplus
extern "C" {
#define RX7900_BRUSH_NOEXCEPT noexcept
#else
#define RX7900_BRUSH_NOEXCEPT
#endif
BOOL APIENTRY Rx7900BrushOrigin(const POINTL* dc_brush,const POINTL* dc_origin,
    POINTL* output) RX7900_BRUSH_NOEXCEPT;
// coordinate accepts LONG pixels and bottom-exclusive minus one. Positive
// extent is mandatory; output always lies within [0, extent) on success.
BOOL APIENTRY Rx7900PatternPhase(LONGLONG coordinate,LONG origin,LONG extent,
    LONG* output) RX7900_BRUSH_NOEXCEPT;
#ifdef __cplusplus
}
#endif
#undef RX7900_BRUSH_NOEXCEPT

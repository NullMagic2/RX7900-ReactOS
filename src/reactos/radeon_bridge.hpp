/*
Module: rx7900.reactos.radeon_bridge
Purpose: Win32k caller bridge from the pinned IntEngLineTo to the radeondisp DrvStrokePath.
Created: 2026-10-23
Architecture: Compiled into win32k from this tree (RX7900_RADEON_BRIDGE). One
read-only DrvEscape identity with a request mask (radeon_line_identity.hpp)
admits a positively identified radeondisp surface; every decline returns to
the original engine path unchanged. Include after the native GDI SDK types, in
either C glue or C++.
*/
#pragma once

// One styled cosmetic segment prepared for DrvStrokePath. The C glue builds
// a stack PATHOBJ over points; styles stays borrowed from the locked pen brush
// for the same synchronous IntEngLineTo call and is never retained.
typedef struct _RX7900_RADEON_STROKE {
    POINTFIX points[2];
    MIX mix;
    ULONG style_count;
    const ULONG* styles;
} RX7900_RADEON_STROKE;

#ifdef __cplusplus
extern "C" {
#define RX7900_RADEON_BRIDGE_NOEXCEPT noexcept
#else
#define RX7900_RADEON_BRIDGE_NOEXCEPT
#endif

// Admits IntEngLineTo's (x1,y1)-(x2,y2) with the pen's private style runs for
// the identified radeondisp surface. TRUE fills stroke (points in FIX, mix
// carrying the paired line tag); FALSE clears it and the caller continues with
// its original DrvLineTo/EngLineTo code. Runs at the caller's IRQL with the
// caller's DC and surface locks; it probes the driver with one Escape at most.
BOOL APIENTRY Rx7900RadeonStrokePrepare(RX7900_RADEON_STROKE* stroke, SURFOBJ* surface,
    PFN_DrvEscape escape, CLIPOBJ* clip, MIX mix, LONG x1, LONG y1, LONG x2, LONG y2,
    ULONG style_count, const ULONG* styles) RX7900_RADEON_BRIDGE_NOEXCEPT;

// Calls draw (the surface's DrvStrokePath) once with the prepared segment:
// no transform, no brush origin, cosmetic LA_STYLED attributes of width 1 and
// zero style state. Returns the driver's result; FALSE means the driver drew
// nothing it could not redraw identically, so the original path follows.
BOOL APIENTRY Rx7900RadeonStrokeDraw(const RX7900_RADEON_STROKE* stroke, PFN_DrvStrokePath draw,
    SURFOBJ* surface, PATHOBJ* path, CLIPOBJ* clip, BRUSHOBJ* brush) RX7900_RADEON_BRIDGE_NOEXCEPT;

#ifdef __cplusplus
}
#endif
#undef RX7900_RADEON_BRIDGE_NOEXCEPT

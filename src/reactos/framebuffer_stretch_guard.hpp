/*
Module: lightning.reactos.stretch_guard
Purpose: Preserve engine orientation in a positively identified kernel DDI call.
Created: 2026-10-21
Architecture: Synchronous stack-only contract; never a user Escape or wire ABI.
License: GPL-2.0-or-later.
*/
#pragma once

#define RX7900_STRETCH_GUARD_VERSION 1U
#define RX7900_STRETCH_GUARD_MAGIC 0x3153474cU
#define RX7900_STRETCH_FLIP_X 1U
#define RX7900_STRETCH_FLIP_Y 2U
#define RX7900_LINE_MIX_TAG 0x4e4c0000U

// Include after native GDI SDK types. The entire context fits ordinary
// COLORADJUSTMENT storage. Its remaining bytes carry a borrowed kernel pointer
// and orientation only for one synchronous callback, never user or wire data.
typedef struct _RX7900_STRETCH_GUARD {
    COLORADJUSTMENT tag;
} RX7900_STRETCH_GUARD;

#ifdef __cplusplus
extern "C" {
#define RX7900_STRETCH_NOEXCEPT noexcept
#else
#define RX7900_STRETCH_NOEXCEPT
#endif
COLORADJUSTMENT* APIENTRY Rx7900StretchGuardPrepare(RX7900_STRETCH_GUARD* context,
    SURFOBJ* surface,PFN_DrvEscape escape,COLORADJUSTMENT* original,
    BOOL horizontal_flip,BOOL vertical_flip) RX7900_STRETCH_NOEXCEPT;
BOOL APIENTRY Rx7900StretchGuardUnwrap(COLORADJUSTMENT* tagged,
    COLORADJUSTMENT** original,ULONG* orientation) RX7900_STRETCH_NOEXCEPT;
// Tag only positively identified unstyled cosmetic calls. Styled and unpaired
// engine calls retain their original MIX and exact original engine fallback.
MIX APIENTRY Rx7900LinePrepare(SURFOBJ* surface,PFN_DrvEscape escape,MIX mix,
    ULONG style_count) RX7900_STRETCH_NOEXCEPT;
BOOL APIENTRY Rx7900GradientSpanVisible(LONG left,LONG right,LONG row,
    const RECTL* clip) RX7900_STRETCH_NOEXCEPT;
BOOL APIENTRY Rx7900GradientCaptureBytes(ULONG mode,ULONG vertices,ULONG meshes,
    ULONG* vertex_bytes,ULONG* mesh_bytes) RX7900_STRETCH_NOEXCEPT;
VOID APIENTRY Rx7900SynchronizePrimary(FLONG flags) RX7900_STRETCH_NOEXCEPT;
#ifdef __cplusplus
}
#endif
#undef RX7900_STRETCH_NOEXCEPT

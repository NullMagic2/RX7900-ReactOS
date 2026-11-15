/*
Module: rx7900.reactos.radeon_text_engine
Purpose: Source-pinned Win32k bridge for physical Radeon coverage glyphs.
Created: 2026-10-23
Architecture: A read-only DDI identity query gates a synchronous, kernel-only
DrvNotify request. Borrowed GDI objects never cross a user or miniport ABI.
*/
#pragma once

#define RX7900_RADEON_GLYPH_UNSUPPORTED 0U
#define RX7900_RADEON_GLYPH_PERFORMED 1U
#define RX7900_RADEON_GLYPH_FAILED 2U

typedef struct _RX7900_RADEON_TEXT_CONTEXT {
    SURFOBJ* destination;
    PFN_DrvNotify notify;
} RX7900_RADEON_TEXT_CONTEXT;

#ifdef __cplusplus
extern "C" {
#define RX7900_RADEON_TEXT_NOEXCEPT noexcept
#else
#define RX7900_RADEON_TEXT_NOEXCEPT
#endif

VOID APIENTRY Rx7900RadeonTextBegin(RX7900_RADEON_TEXT_CONTEXT* context,
    SURFOBJ* destination, PFN_DrvEscape escape,
    PFN_DrvNotify notify) RX7900_RADEON_TEXT_NOEXCEPT;

ULONG APIENTRY Rx7900RadeonTextGlyph(RX7900_RADEON_TEXT_CONTEXT* context,
    SURFOBJ* mask, CLIPOBJ* clip, XLATEOBJ* rgb_to_device,
    XLATEOBJ* device_to_rgb, const RECTL* destination,
    const POINTL* mask_origin, BRUSHOBJ* brush) RX7900_RADEON_TEXT_NOEXCEPT;

#ifdef __cplusplus
}
#endif
#undef RX7900_RADEON_TEXT_NOEXCEPT

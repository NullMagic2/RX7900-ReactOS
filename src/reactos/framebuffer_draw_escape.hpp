/*
Module: lightning.reactos.draw_escape
Purpose: Expose a bounded window-clipped DrawEscape through the pinned GDI API.
Created: 2026-10-22
Architecture: Thin native syscall glue delegates to C++ DC/input ownership.
License: GPL-2.0-or-later, matching the ReactOS integration.
*/
#pragma once
#ifdef __cplusplus
extern "C" {
#define LIGHTNING_ESCAPE_NOEXCEPT noexcept
#else
#define LIGHTNING_ESCAPE_NOEXCEPT
#endif
INT APIENTRY LightningDrawEscape(HDC dc, INT escape, INT input_bytes,
    const void* input) LIGHTNING_ESCAPE_NOEXCEPT;
// USER owns this DCE under its exclusive lock. Foreign process region
// attributes must be refreshed later in their owning process, even during resize.
struct tagDCE;
BOOL APIENTRY LightningMayRefreshWindowDc(struct tagDCE* dce) LIGHTNING_ESCAPE_NOEXCEPT;
// Consume deferred work before UserGetDCEx replaces its transient DCE flags.
BOOL APIENTRY LightningWindowDcNeedsRefresh(const struct tagDCE* dce,
    BOOL existing_refresh) LIGHTNING_ESCAPE_NOEXCEPT;
#ifdef __cplusplus
}
#endif
#undef LIGHTNING_ESCAPE_NOEXCEPT

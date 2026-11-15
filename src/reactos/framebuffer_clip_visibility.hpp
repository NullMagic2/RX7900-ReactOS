/*
Module: lightning.reactos.clip_visibility
Purpose: Preserve an explicitly empty visible region at the native DDI boundary.
Created: 2026-10-22
Architecture: Called only after Win32k updates an owned, locked clip region.
License: GPL-2.0-or-later, matching the ReactOS integration.
*/
#pragma once
#ifdef __cplusplus
extern "C" {
#define RX7900_CLIP_NOEXCEPT noexcept
#else
#define RX7900_CLIP_NOEXCEPT
#endif
// The caller owns the live CLIPOBJ through this synchronous call. No allocation,
// pointer retention or IRQL change occurs. Never call for a newly initialized
// unbounded clip: only an explicit region update establishes empty visibility.
VOID APIENTRY Rx7900NormalizeClipRegion(CLIPOBJ* clip) RX7900_CLIP_NOEXCEPT;
#ifdef __cplusplus
}
#endif
#undef RX7900_CLIP_NOEXCEPT

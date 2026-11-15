/*
Module: lightning.reactos.cursor_recovery
Purpose: Query an optional display owner's confirmed cursor recovery request.
Created: 2026-10-21
Architecture: Internal Win32k C/C++ boundary; include after native Winddi types.
*/
#pragma once

#ifdef __cplusplus
extern "C" {
#define RX7900_CURSOR_NOEXCEPT noexcept
#else
#define RX7900_CURSOR_NOEXCEPT
#endif

// Caller holds the PDEV/DC locks after DrvMovePointer has returned. This function
// only issues a bounded read-only Escape; it never selects bitmaps or changes
// engine flags. Release those locks before acting on a TRUE recovery request.
BOOL APIENTRY Rx7900CursorMoveNeedsRecovery(SURFOBJ* surface,
    PFN_DrvEscape escape) RX7900_CURSOR_NOEXCEPT;

#ifdef __cplusplus
}
#endif
#undef RX7900_CURSOR_NOEXCEPT

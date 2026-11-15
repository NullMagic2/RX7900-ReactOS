/*
Module: lightning.reactos.cursor_recovery
Purpose: Recognize confirmed hidden-cursor failures without a second renderer.
Created: 2026-10-21
Architecture: Win32k queries a matching live display owner through read-only DDI.
*/
#include <stdarg.h>
#include <windef.h>
#include <wingdi.h>
#include <winddi.h>
#include "framebuffer_cursor_recovery.hpp"
#include "lightning_present_abi.hpp"

extern "C" BOOL APIENTRY Rx7900CursorMoveNeedsRecovery(SURFOBJ* surface,
    PFN_DrvEscape escape) noexcept {
    using namespace lightning::reactos;

    if (surface == nullptr || surface->iType != STYPE_DEVICE || escape == nullptr) {
        return FALSE;
    }

    CursorStatistics status{};
    // Both buffers are kernel-owned. The initialized output lives through this
    // synchronous callback; no returned address or driver-private object escapes.
    if (escape(surface, query_cursor_escape, 0, nullptr, sizeof(status), &status) != sizeof(status) ||
        status.size != sizeof(status) || status.version != cpu_scanout_abi_version ||
        status.magic != cursor_statistics_magic || status.mode_generation == 0 || status.reserved != 0) {
        return FALSE;
    }

    // A hidden plane alone can mean intentional hide, sleep or a retired PDEV.
    // Only the matching owner may request reselect after its failed live move
    // and actual hide acknowledgement; unavailable/uncertain hide cannot opt in.
    return status.recovery_required == 1 && status.hardware_active == 0 && status.visible == 0;
}

/*
Module: lightning.tests.cursor_recovery
Purpose: Verify the real Winddi callback boundary for optional move recovery.
Created: 2026-10-21
Architecture: SDK types and production helper; no display or device operation.
*/
#include <stdarg.h>
#include <windef.h>
#include <wingdi.h>
#include <winddi.h>
#include <stdio.h>
#include "framebuffer_cursor_recovery.hpp"
#include "lightning_present_abi.hpp"

namespace {
using namespace lightning::reactos;
unsigned checks = 0;
unsigned failures = 0;
unsigned probes = 0;
unsigned fault = 0;
SURFOBJ* expected_surface = nullptr;

void expect(bool condition, const char* description) {
    ++checks;
    if (!condition) {
        ++failures;
        printf("FAIL: %s\n", description);
    }
}

ULONG APIENTRY query(SURFOBJ* surface, ULONG code, ULONG input_bytes, PVOID input,
    ULONG output_bytes, PVOID output) {
    ++probes;
    expect(surface == expected_surface && code == query_cursor_escape && input_bytes == 0 && input == nullptr,
        "engine issues only the exact read-only capability query on its owned current surface");
    expect(output != nullptr && output_bytes == sizeof(CursorStatistics),
        "query output is a complete fixed104-byte owner record");
    if (output == nullptr || output_bytes != sizeof(CursorStatistics)) { return 0; }
    auto& status = *static_cast<CursorStatistics*>(output);
    expect(status.size == 0 && status.magic == 0 && status.mode_generation == 0 &&
        status.recovery_required == 0 && status.reserved == 0,
        "foreign callback receives fresh initialized output rather than prior owner state");
    if (fault == 1) { return 0; }
    if (fault == 2) { return sizeof(status); }
    status = {};
    status.size = sizeof(status);
    status.version = cpu_scanout_abi_version;
    status.magic = cursor_statistics_magic;
    status.mode_generation = 7;
    status.defined = 1;
    status.recovery_required = 1;
    switch (fault) {
    case 3: ++status.size; break;
    case 4: ++status.version; break;
    case 5: ++status.magic; break;
    case 6: status.mode_generation = 0; break;
    case 7: status.reserved = 1; break;
    case 8: status.hardware_active = 1; break;
    case 9: status.visible = 1; break;
    case 10: status.recovery_required = 0; break;
    case 11: status.recovery_required = 2; break;
    case 12: return sizeof(status) - 1;
    case 13: return sizeof(status) + 1;
    default: break;
    }
    return sizeof(status);
}
} // namespace

int main() {
    SURFOBJ surface{};
    surface.iType = STYPE_DEVICE;
    expected_surface = &surface;
    expect(!Rx7900CursorMoveNeedsRecovery(nullptr, query) && probes == 0,
        "null surface does not enter a foreign callback");
    expect(!Rx7900CursorMoveNeedsRecovery(&surface, nullptr) && probes == 0,
        "missing Escape means unchanged legacy driver behavior");
    surface.iType = STYPE_BITMAP;
    expect(!Rx7900CursorMoveNeedsRecovery(&surface, query) && probes == 0,
        "engine bitmap is never interpreted as a display owner");
    surface.iType = STYPE_DEVBITMAP;
    expect(!Rx7900CursorMoveNeedsRecovery(&surface, query) && probes == 0,
        "foreign device bitmap cannot opt into current display cursor recovery");
    surface.iType = STYPE_DEVICE;
    fault = 0;
    expect(Rx7900CursorMoveNeedsRecovery(&surface, query),
        "exact live owner explicitly requests original-shape reselect after confirmed hide");
    for (fault = 1; fault <= 13; ++fault) {
        expect(!Rx7900CursorMoveNeedsRecovery(&surface, query),
            "unsupported, truncated, stale, malformed, active or intentionally hidden owner cannot trigger recovery");
    }
    fault = 0;
    expect(Rx7900CursorMoveNeedsRecovery(&surface, query),
        "a prior declined probe cannot poison later valid owner recognition");
    printf("Framebuffer cursor recovery SDK boundary: %u checks, %u failures; live recovery untested\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

/*
Module: lightning.tests.draw_escape_owner
Purpose: Exercise the production deferred window-DC ownership gates.
Created: 2026-10-22
Architecture: Exact Win32k SDK state and production C++ functions; the owner
query is an injected kernel boundary, never a mirrored visibility algorithm.
*/
extern "C" {
#include <win32k.h>
typedef struct tagSERVERINFO* PSERVERINFO;
#include "user/ntuser/ntuser.h"
typedef struct _WND* PWND;
#include "user/ntuser/dce.h"
#include <stdio.h>
}
#include "framebuffer_draw_escape.hpp"

namespace {
unsigned checks = 0;
unsigned failures = 0;
unsigned owner_queries = 0;
ULONG current_owner = GDI_OBJ_HMGR_RESTRICTED;
HGDIOBJ queried_handle = nullptr;

void expect(bool condition, const char* description) {
    ++checks;
    if (!condition) {
        ++failures;
        printf("FAIL: %s\n", description);
    }
}
}

extern "C" ULONG NTAPI GreGetObjectOwner(HGDIOBJ handle) {
    ++owner_queries;
    queried_handle = handle;
    return current_owner;
}

int main() {
    expect(LightningMayRefreshWindowDc(nullptr) == FALSE,
        "missing DCE cannot authorize a visibility refresh");
    expect(owner_queries == 0, "missing DCE never queries a GDI object");

    const DWORD unavailable[]{DCX_DCEEMPTY, DCX_INDESTROY,
        DCX_DCEEMPTY | DCX_INDESTROY | DCX_DCEDIRTY};
    for (DWORD flags : unavailable) {
        DCE dce{};
        dce.DCXFlags = flags;
        const unsigned before = owner_queries;
        expect(LightningMayRefreshWindowDc(&dce) == FALSE,
            "empty or destroyed DCE cannot authorize a refresh");
        expect(dce.DCXFlags == flags && owner_queries == before,
            "unavailable DCE remains unchanged without a GDI lookup");
    }

    DCE invalid{};
    invalid.DCXFlags = DCX_DCEBUSY;
    current_owner = GDI_OBJ_HMGR_RESTRICTED;
    expect(LightningMayRefreshWindowDc(&invalid) == FALSE &&
        queried_handle == nullptr,
        "invalid retained handle cannot authorize a visibility refresh");
    expect(invalid.DCXFlags == (DCX_DCEBUSY | DCX_DCEDIRTY),
        "failed ownership lookup retains deferred work for safe recovery");

    const ULONG owners[]{GDI_OBJ_HMGR_POWNED, GDI_OBJ_HMGR_PUBLIC,
        GDI_OBJ_HMGR_RESTRICTED};
    const bool dirty_states[]{false, true};
    for (ULONG owner : owners) {
        for (bool already_dirty : dirty_states) {
            DCE dce{};
            dce.hDC = reinterpret_cast<HDC>(static_cast<ULONG_PTR>(0x1234));
            dce.hrgnClip = reinterpret_cast<HRGN>(static_cast<ULONG_PTR>(0x5678));
            const DWORD original = DCX_DCEBUSY | DCX_CLIPSIBLINGS |
                (already_dirty ? DCX_DCEDIRTY : 0);
            dce.DCXFlags = original;
            current_owner = owner;
            const unsigned before = owner_queries;
            const bool allowed = owner != GDI_OBJ_HMGR_RESTRICTED;
            expect((LightningMayRefreshWindowDc(&dce) != FALSE) == allowed,
                "only current-process or public GDI ownership is admitted");
            expect(owner_queries == before + 1 && queried_handle == dce.hDC,
                "ownership query uses the exact retained DC handle");
            expect(dce.DCXFlags == (allowed ? original : original | DCX_DCEDIRTY),
                "foreign ownership defers work without changing other DCE flags");
            expect(dce.hDC == reinterpret_cast<HDC>(static_cast<ULONG_PTR>(0x1234)) &&
                dce.hrgnClip == reinterpret_cast<HRGN>(static_cast<ULONG_PTR>(0x5678)),
                "ownership gate neither replaces nor dereferences retained region handles");
            expect(LightningWindowDcNeedsRefresh(&dce, FALSE) ==
                ((!allowed || already_dirty) ? TRUE : FALSE),
                "deferred work survives the GetDC flag-replacement boundary");
            expect(LightningWindowDcNeedsRefresh(&dce, TRUE) == TRUE,
                "an existing native refresh request is never suppressed");
        }
    }

    const unsigned before = owner_queries;
    expect(LightningWindowDcNeedsRefresh(nullptr, FALSE) == FALSE &&
        LightningWindowDcNeedsRefresh(nullptr, TRUE) == TRUE,
        "missing DCE preserves the caller's existing refresh decision");
    expect(owner_queries == before,
        "dirty-state query never accesses GDI ownership or foreign attributes");
    printf("draw-escape-owner: %u checks, %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

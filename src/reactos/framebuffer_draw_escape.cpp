/*
Module: lightning.reactos.draw_escape
Purpose: Capture untrusted escape bytes and preserve authoritative window clipping.
Created: 2026-10-22
Architecture: Win32k owns USER/DC/device locks; the driver owns image semantics.
License: GPL-2.0-or-later, matching the ReactOS integration.
Reference: Microsoft DrvDrawEscape contract, window rectangle stable during call.
*/
extern "C" {
#include <win32k.h>
// The pinned public USER header contains C-only anonymous structure members.
// Its server layout is irrelevant here; complete USER types stay C-owned.
typedef struct tagSERVERINFO* PSERVERINFO;
#include "user/ntuser/ntuser.h"
#if defined(RX7900_FRAMEBUFFER_WINDOW_VISIBILITY) || defined(RX7900_DRAW_ESCAPE_CLIP_TRACE)
typedef struct _WND* PWND;
#include "user/ntuser/dce.h"
// The complete USER window layout is C-only. Borrow the native handle lookup
// boundary from userfuncs.h without duplicating or dereferencing that layout.
PWND FASTCALL UserGetWindowObject(HWND handle);
#endif
#include "user/ntuser/mmcopy.h"
}
#include "framebuffer_draw_escape.hpp"
#include <stdint.h>

namespace {
constexpr ULONG maximum_escape_bytes = 65536;
constexpr ULONG escape_pool_tag = 'eDlQ';

class UserOwnership final {
public:
    UserOwnership() noexcept { UserEnterExclusive(); }
    ~UserOwnership() { UserLeave(); }
    UserOwnership(const UserOwnership&) = delete;
    UserOwnership& operator=(const UserOwnership&) = delete;
};

// USER is already exclusively locked. Refresh in the DC's real owning process
// before taking its GDI/PDEV locks; the native reset routine also updates moved
// origins and preserves saved/application clip handling. No foreign attributes
// are touched by the paired owner filter in DceResetActiveDCEs.
#ifdef RX7900_FRAMEBUFFER_WINDOW_VISIBILITY
bool refresh_window_visibility(HDC handle) noexcept {
    DCE* dce = DceGetDceFromDC(handle);
    if (dce == nullptr) return true;
    if ((dce->DCXFlags & (DCX_INDESTROY | DCX_DCEEMPTY)) != 0) return false;
    const ULONG owner = GreGetObjectOwner(handle);
    if (owner != GDI_OBJ_HMGR_POWNED && owner != GDI_OBJ_HMGR_PUBLIC) return false;
    if ((dce->DCXFlags & DCX_DCEDIRTY) == 0) return true;

    if (dce->hwndCurrent == nullptr) {
        DceUpdateVisRgn(dce, nullptr, dce->DCXFlags);
    } else {
        PWND window = UserGetWindowObject(dce->hwndCurrent);
        if (window == nullptr) return false;
        DceResetActiveDCEs(window);
    }
    return (dce->DCXFlags & DCX_DCEDIRTY) == 0;
}
#endif

// Caller memory is copied before locks. The synchronous driver callback never
// receives a user pointer, and malformed or partially copied input is discarded.
class CapturedInput final {
public:
    ~CapturedInput() {
        if (bytes_ != nullptr) ExFreePoolWithTag(bytes_, escape_pool_tag);
    }
    NTSTATUS capture(const void* input, ULONG size) noexcept {
        if (size == 0) return STATUS_SUCCESS;
        bytes_ = ExAllocatePoolWithTag(PagedPool, size, escape_pool_tag);

        if (bytes_ == nullptr) return STATUS_INSUFFICIENT_RESOURCES;
        return _MmCopyFromCaller(bytes_, const_cast<void*>(input), size);
    }
    void* get() const noexcept { return bytes_; }
    CapturedInput() noexcept = default;
    CapturedInput(const CapturedInput&) = delete;
    CapturedInput& operator=(const CapturedInput&) = delete;
private:
    void* bytes_ = nullptr;
};

// USER excludes window movement/visibility mutation. DC preparation updates
// effective clipping and pins the display surface/mode under its device lock.
// The driver finishes its owned image read before these locks are released.
class WindowDc final {
public:
    explicit WindowDc(HDC handle) noexcept {
#ifdef RX7900_FRAMEBUFFER_WINDOW_VISIBILITY
        if (!refresh_window_visibility(handle)) return;
#endif
        dc_ = DC_LockDc(handle);

        if (dc_ != nullptr && dc_->ppdev != nullptr) {
            DC_vPrepareDCsForBlit(dc_, nullptr, nullptr, nullptr);
            prepared_ = true;
#ifdef RX7900_DRAW_ESCAPE_CLIP_TRACE
            // Opt-in metadata only. USER and DC locks keep all borrowed region
            // objects valid. No state, caller pixels or addresses are retained.
            const DCE* dce = DceGetDceFromDC(handle);
            const REGION* visible = dc_->prgnVis;
            const REGION* effective = dc_->prgnRao;
            DbgPrint("QLCLIP dce=%lx dc=%lx co=%u box=%ld,%ld,%ld,%ld "
                "vis=%lu:%ld,%ld,%ld,%ld rao=%lu:%ld,%ld,%ld,%ld origin=%ld,%ld\n",
                dce != nullptr ? dce->DCXFlags : 0UL, static_cast<ULONG>(dc_->fs),
                static_cast<unsigned>(dc_->co.iDComplexity), dc_->co.rclBounds.left,
                dc_->co.rclBounds.top, dc_->co.rclBounds.right, dc_->co.rclBounds.bottom,
                visible != nullptr ? visible->rdh.nCount : 0UL,
                visible != nullptr ? visible->rdh.rcBound.left : 0L,
                visible != nullptr ? visible->rdh.rcBound.top : 0L,
                visible != nullptr ? visible->rdh.rcBound.right : 0L,
                visible != nullptr ? visible->rdh.rcBound.bottom : 0L,
                effective != nullptr ? effective->rdh.nCount : 0UL,
                effective != nullptr ? effective->rdh.rcBound.left : 0L,
                effective != nullptr ? effective->rdh.rcBound.top : 0L,
                effective != nullptr ? effective->rdh.rcBound.right : 0L,
                effective != nullptr ? effective->rdh.rcBound.bottom : 0L,
                dc_->ptlDCOrig.x, dc_->ptlDCOrig.y);
#endif
        }
    }
    ~WindowDc() {
        if (dc_ != nullptr) {
            if (prepared_) DC_vFinishBlit(dc_, nullptr);
            DC_UnlockDc(dc_);
        }
    }
    PDC get() const noexcept { return dc_; }
    WindowDc(const WindowDc&) = delete;
    WindowDc& operator=(const WindowDc&) = delete;
private:
    UserOwnership user_;
    PDC dc_ = nullptr;
    bool prepared_ = false;
};
}

#ifdef RX7900_FRAMEBUFFER_WINDOW_VISIBILITY
extern "C" BOOL APIENTRY LightningMayRefreshWindowDc(DCE* dce) noexcept {
    if (dce == nullptr || (dce->DCXFlags & (DCX_INDESTROY | DCX_DCEEMPTY)) != 0) return FALSE;
    const ULONG owner = GreGetObjectOwner(dce->hDC);
    if (owner == GDI_OBJ_HMGR_POWNED || owner == GDI_OBJ_HMGR_PUBLIC) return TRUE;

    // This is USER-owned metadata, not a foreign GDI/region attribute. Leave
    // the handle and process ownership intact and defer work to its owner.
    dce->DCXFlags |= DCX_DCEDIRTY;
    return FALSE;
}

extern "C" BOOL APIENTRY LightningWindowDcNeedsRefresh(const DCE* dce,
    BOOL existing_refresh) noexcept {
    return existing_refresh != FALSE ||
        (dce != nullptr && (dce->DCXFlags & DCX_DCEDIRTY) != 0) ? TRUE : FALSE;
}

// These unchanged C bodies own user-buffer capture/SEH and GDI resources. The
// public C++ syscall boundaries add USER ownership before any DC/PDEV lock and
// keep it through all early returns, so cross-process Z order cannot make the
// refreshed visibility stale between preparation and the actual DIB draw.
extern "C" INT APIENTRY LightningSetDIBitsImplementation(HDC, INT, INT, DWORD, DWORD,
    INT, INT, DWORD, DWORD, LPBYTE, LPBITMAPINFO, DWORD, UINT, UINT, BOOL, HANDLE);
extern "C" INT APIENTRY LightningStretchDIBitsImplementation(HDC, INT, INT, INT, INT,
    INT, INT, INT, INT, LPBYTE, LPBITMAPINFO, DWORD, DWORD, UINT, UINT, HANDLE);

extern "C" INT APIENTRY NtGdiSetDIBitsToDeviceInternal(HDC dc, INT destination_x,
    INT destination_y, DWORD width, DWORD height, INT source_x, INT source_y,
    DWORD first_scan, DWORD scan_lines, LPBYTE bits, LPBITMAPINFO information,
    DWORD color_use, UINT maximum_bits, UINT maximum_information, BOOL transform,
    HANDLE color_transform) {
    UserOwnership user;
    if (!refresh_window_visibility(dc)) {
        EngSetLastError(ERROR_INVALID_HANDLE);
        return 0;
    }
    return LightningSetDIBitsImplementation(dc, destination_x, destination_y, width,
        height, source_x, source_y, first_scan, scan_lines, bits, information,
        color_use, maximum_bits, maximum_information, transform, color_transform);
}

extern "C" INT APIENTRY NtGdiStretchDIBitsInternal(HDC dc, INT destination_x,
    INT destination_y, INT destination_width, INT destination_height, INT source_x,
    INT source_y, INT source_width, INT source_height, LPBYTE bits,
    LPBITMAPINFO information, DWORD usage, DWORD rop, UINT maximum_information,
    UINT maximum_bits, HANDLE color_transform) {
    UserOwnership user;
    if (!refresh_window_visibility(dc)) {
        EngSetLastError(ERROR_INVALID_HANDLE);
        return 0;
    }
    return LightningStretchDIBitsImplementation(dc, destination_x, destination_y,
        destination_width, destination_height, source_x, source_y, source_width,
        source_height, bits, information, usage, rop, maximum_information,
        maximum_bits, color_transform);
}
#endif

extern "C" INT APIENTRY LightningDrawEscape(HDC handle, INT escape,
    INT input_bytes, const void* input) noexcept {
    if (input_bytes < 0 || static_cast<ULONG>(input_bytes) > maximum_escape_bytes ||
        (input_bytes != 0 && input == nullptr)) {
        EngSetLastError(ERROR_INVALID_PARAMETER);
        return -1;
    }
    CapturedInput captured;
    const NTSTATUS status = captured.capture(input, static_cast<ULONG>(input_bytes));

    if (!NT_SUCCESS(status)) {
        SetLastNtError(status);
        return -1;
    }
    WindowDc owner(handle);
    PDC dc = owner.get();

    if (dc == nullptr) {
        EngSetLastError(ERROR_INVALID_HANDLE);
        return -1;
    }
    PSURFACE surface = dc->dclevel.pSurface;
    PPDEVOBJ device = dc->ppdev;

    if (surface == nullptr || device == nullptr ||
        device->DriverFunctions.DrawEscape == nullptr ||
        (device->flFlags & (PDEV_DISABLED | PDEV_DRIVER_PUNTED_CALL)) != 0) {
        EngSetLastError(ERROR_NOT_SUPPORTED);
        return -1;
    }
    // Retained DCs can move without erclWindow being refreshed by ReactOS.
    // Use the same current device origin as GDI readback and preserve extent.
    // pco includes current visibility and application clipping; drivers must
    // intersect their destination with it, including a fully occluded region.
    RECTL window = dc->erclWindow;
    const int64_t right = static_cast<int64_t>(dc->ptlDCOrig.x) +
        (static_cast<int64_t>(window.right) - window.left);
    const int64_t bottom = static_cast<int64_t>(dc->ptlDCOrig.y) +
        (static_cast<int64_t>(window.bottom) - window.top);

    if (right < INT32_MIN || right > INT32_MAX || bottom < INT32_MIN || bottom > INT32_MAX) {
        EngSetLastError(ERROR_INVALID_PARAMETER);
        return -1;
    }
    window.left = dc->ptlDCOrig.x;
    window.top = dc->ptlDCOrig.y;
    window.right = static_cast<int32_t>(right);
    window.bottom = static_cast<int32_t>(bottom);

    return static_cast<INT>(device->DriverFunctions.DrawEscape(&surface->SurfObj,
        static_cast<ULONG>(escape), static_cast<CLIPOBJ*>(&dc->co),
        &window, static_cast<ULONG>(input_bytes), captured.get()));
}


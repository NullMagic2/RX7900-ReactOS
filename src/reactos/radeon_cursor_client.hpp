/*
Module: rx7900.reactos.radeon_cursor_client
Purpose: radeondisp DrvSetPointerShape/DrvMovePointer over the rx7900video hardware cursor IOCTLs.
Created: 2026-10-22
Architecture: GDI-side half of the hardware cursor; the miniport half is
radeon_cursor_native, the shared wire contract radeon_cursor_policy.hpp. The
display DLL keeps one PointerClient per PDEV and forwards its pointer DDI calls
here under its own PDEV serialization. Include after the native Winddi types.
*/
#pragma once

#include <stdint.h>

namespace rx7900::reactos::radeon_cursor {

// One per PDEV, value-initialized, trivially destructible. GDI serializes
// pointer calls per PDEV (devlock) and the display DLL additionally holds its
// registry semaphore; no member is touched concurrently.
struct PointerClient final {
    // QUERY_POINTER_CAPABILITIES answered once, with the formats this driver needs.
    bool probed = false;
    bool capable = false;
    uint32_t max_width = 0;
    uint32_t max_height = 0;
    uint32_t formats = 0;
    // The engine believes the current pointer is the hardware cursor.
    bool hardware = false;
    bool visible = false;
    int32_t hot_x = 0;
    int32_t hot_y = 0;
    uint32_t width = 0;
    uint32_t height = 0;
};

// DrvSetPointerShape, PASSIVE_LEVEL in a win32k thread. Accepts monochrome,
// mask + color (any color format, translated to 32 bpp by EngCopyBits) and
// SPS_ALPHA 32 bpp (premultiplied, trivially translated) shapes up to the
// miniport maximum, normalizes them with EngineBitmap and sends one
// IOCTL_VIDEO_SET_POINTER_ATTR; returns SPS_ACCEPT_NOEXCLUDE. Anything else, and
// any miniport refusal, hides the hardware cursor and returns SPS_DECLINE: the
// engine then draws the pointer itself (ReactOS IntEngSetPointerShape calls
// EngSetPointerShape after a decline, as Windows GDI simulates declined
// pointers). bounds, when present, receives the pointer rectangle (or -1s when
// hidden), as EngMovePointer reports it.
ULONG set_pointer_shape(PointerClient& client, HANDLE driver, SURFOBJ* mask, SURFOBJ* color,
                        XLATEOBJ* translation, LONG hot_x, LONG hot_y, LONG x, LONG y, RECTL* bounds,
                        FLONG flags) noexcept;

// DrvMovePointer: x == -1 hides; otherwise SET_POINTER_POSITION with the
// top-left (hotspot subtracted) and ENABLE_POINTER when hidden. Only acts while
// the hardware cursor owns the pointer.
void move_pointer(PointerClient& client, HANDLE driver, LONG x, LONG y, RECTL* bounds) noexcept;

// DrvDisableSurface: disables the hardware cursor and forgets the shape.
void retire_pointer(PointerClient& client, HANDLE driver) noexcept;

} // namespace rx7900::reactos::radeon_cursor

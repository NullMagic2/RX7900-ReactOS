/*
Module: rx7900.reactos.radeon_line_identity
Purpose: Let the paired kernel line/stretch/stroke helpers identify the physical radeondisp display.
Created: 2026-10-22
Architecture: Header-only fixed-width reply of one read-only DrvEscape; no
addresses, handles or driver state beyond capability bits. Shared by
radeondisp, framebuffer_stretch_guard, radeon_bridge and radeon_devbitmap_bridge
(win32k) and their tests.
*/
#pragma once

#include <stdint.h>
#include <string.h>

namespace rx7900::reactos::radeon_line {

// DrvEscape code ('RDLQ'); output exactly sizeof(Identity). The input is empty
// (version-1 query) or exactly one uint32_t request mask (bridge query).
constexpr uint32_t identity_escape = 0x52444c51U;
constexpr uint32_t identity_version = 1;
constexpr uint32_t identity_magic = 0x4e494c52U;   // 'RLIN'
// The display's GPU line IOCTL is available on this surface.
constexpr uint32_t flag_gpu_lines = 1U;
// The display hooks DrvStretchBltROP and can draw the guarded (orientation
// preserving) stretch; set only by a driver that unwraps the stretch guard.
constexpr uint32_t flag_gpu_stretch = 2U;
// Bits a reply may carry without being requested (the version-1 set).
constexpr uint32_t base_flags = flag_gpu_lines | flag_gpu_stretch;
// DrvStrokePath draws the paired win32k bridge's single styled cosmetic
// segment. Reported only when the caller names it in its request mask, so a
// kernel built before this bit existed never receives a bit it refuses.
constexpr uint32_t flag_styled_stroke = 4U;
// DrvCreateDeviceBitmap/DrvDeleteDeviceBitmap/DrvCopyBits serve the paired
// win32k device-bitmap bridge (radeon_devbitmap_bridge) on this PDEV: compatible
// bitmaps may become driver device bitmaps. Request-only like the stroke bit;
// bit 8 stays unassigned.
constexpr uint32_t flag_device_bitmaps = 16U;
constexpr uint32_t request_flags = flag_styled_stroke | flag_device_bitmaps;
constexpr uint32_t known_flags = base_flags | request_flags;

// The paired bridge presents at most this many positive engine style runs:
// NtGdiExtCreatePen accepts at most 16 PS_USERSTYLE entries and the stock
// cosmetic styles of ntgdi/pen.c have at most 6.
constexpr uint32_t max_stroke_style_runs = 16;

struct Identity final {
    uint32_t size;
    uint32_t version;
    uint32_t magic;
    uint32_t flags;
};
static_assert(sizeof(Identity) == 16, "Fixed escape reply");

constexpr Identity make_identity(bool gpu_lines, bool gpu_stretch = false) noexcept {
    return {static_cast<uint32_t>(sizeof(Identity)), identity_version, identity_magic,
            (gpu_lines ? flag_gpu_lines : 0U) | (gpu_stretch ? flag_gpu_stretch : 0U)};
}

// Driver side: the request mask of one query's input, which is empty (0) or
// exactly one uint32_t; anything else is malformed. input stays owned by GDI
// for the synchronous Escape and is read once, by value.
inline bool decode_request(uint32_t input_bytes, const void* input, uint32_t& requested) noexcept {
    requested = 0;
    if (input_bytes == 0) {
        return true;
    }
    if (input_bytes != sizeof(requested) || input == nullptr) {
        return false;
    }
    memcpy(&requested, input, sizeof(requested));
    return true;
}

// Driver reply to a query carrying request mask requested (0 for the
// version-1 query): a request bit is reported only if requested and available.
constexpr Identity make_identity(bool gpu_lines, bool gpu_stretch, bool styled_stroke,
                                 uint32_t requested) noexcept {
    Identity value = make_identity(gpu_lines, gpu_stretch);
    value.flags |= styled_stroke ? (flag_styled_stroke & requested) : 0U;
    return value;
}

// The same reply for a driver that also serves the device-bitmap bridge.
constexpr Identity make_identity(bool gpu_lines, bool gpu_stretch, bool styled_stroke, bool device_bitmaps,
                                 uint32_t requested) noexcept {
    Identity value = make_identity(gpu_lines, gpu_stretch, styled_stroke, requested);
    value.flags |= device_bitmaps ? (flag_device_bitmaps & requested) : 0U;
    return value;
}

// A well-formed reply that grants capability and carries no bit outside the
// base set plus the caller's own (known) request mask.
constexpr bool admits_requested(const Identity& value, uint32_t capability, uint32_t requested) noexcept {
    return value.size == sizeof(Identity) && value.version == identity_version &&
           value.magic == identity_magic && (requested & ~request_flags) == 0 &&
           (value.flags & ~(base_flags | requested)) == 0 && (value.flags & capability) == capability;
}

// Version-1 query: any bit outside base_flags is unknown and refused.
constexpr bool admits(const Identity& value, uint32_t capability) noexcept {
    return admits_requested(value, capability, 0U);
}
constexpr bool admits_lines(const Identity& value) noexcept { return admits(value, flag_gpu_lines); }
constexpr bool admits_stretch(const Identity& value) noexcept { return admits(value, flag_gpu_stretch); }

} // namespace rx7900::reactos::radeon_line

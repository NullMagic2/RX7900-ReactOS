/*
Module: rx7900.reactos.radeon_text_bridge
Purpose: Versioned kernel-only glyph notification for the physical Radeon DDI.
Created: 2026-10-23
Architecture: The Escape is read-only identity; borrowed GDI pointers occur
only in the synchronous DrvNotify call from the source-pinned win32k helper.
Include after the native GDI SDK types. Never expose GlyphRequest to user mode
or serialize its pointers into a miniport IOCTL.
*/
#pragma once

namespace rx7900::reactos::radeon_text {

constexpr ULONG version = 1;
constexpr ULONG identity_escape = 0x52324751U; // R2GQ
constexpr ULONG identity_magic = 0x5232474cU;  // R2GL
constexpr ULONG glyph_notify = 0x52324747U;    // R2GG
constexpr ULONG unsupported = 0;
constexpr ULONG performed = 1;
constexpr ULONG failed = 2;

struct Capability final {
    ULONG size = sizeof(Capability);
    ULONG version = radeon_text::version;
    ULONG magic = identity_magic;
    ULONG notify_code = glyph_notify;
};

struct GlyphRequest final {
    ULONG size = sizeof(GlyphRequest);
    ULONG version = radeon_text::version;
    SURFOBJ* mask = nullptr;
    CLIPOBJ* clip = nullptr;
    XLATEOBJ* rgb_to_device = nullptr;
    XLATEOBJ* device_to_rgb = nullptr;
    RECTL destination{};
    POINTL mask_origin{};
    BRUSHOBJ* brush = nullptr;
    ULONG result = unsupported;
};

} // namespace rx7900::reactos::radeon_text

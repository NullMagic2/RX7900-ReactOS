/*
Module: rx7900.reactos.radeon_cursor_policy
Purpose: One checked wire contract for the VideoPort pointer IOCTLs shared by rx7900video and radeondisp.
Created: 2026-10-22
Architecture: Pure, allocation-free C++17 over ByteView; no SDK types, so the
miniport (radeon_cursor_native), the GDI DLL (radeon_cursor_client) and host
tests decode and encode the same bytes. The DCN image encoding is
src/display/dcn32_cursor; this header only describes the IOCTL payload.
Layouts follow the public ntddvdeo.h structures (VIDEO_POINTER_ATTRIBUTES,
VIDEO_POINTER_POSITION, VIDEO_POINTER_CAPABILITIES); the payload layout and the
alpha flag bits below are this driver's own definitions.
*/
#pragma once

#include <stdint.h>

#include "byte_view.hpp"

namespace rx7900::reactos::cursor_policy {

// VIDEO_POINTER_ATTRIBUTES: Flags +0, Width +4, Height +8, WidthInBytes +12,
// Enable +16, Column (SHORT) +20, Row (SHORT) +22, Pixels +24.
constexpr uint32_t attributes_header_bytes = 24;
constexpr uint32_t position_bytes = 4;      // VIDEO_POINTER_POSITION
constexpr uint32_t capabilities_bytes = 20; // VIDEO_POINTER_CAPABILITIES

// ntddvdeo.h VIDEO_POINTER_CAPABILITIES / VIDEO_POINTER_ATTRIBUTES flags.
constexpr uint32_t flag_async = 0x01;          // VIDEO_MODE_ASYNC_POINTER
constexpr uint32_t flag_mono = 0x02;           // VIDEO_MODE_MONO_POINTER
constexpr uint32_t flag_color = 0x04;          // VIDEO_MODE_COLOR_POINTER
constexpr uint32_t flag_animate_start = 0x08;  // VIDEO_MODE_ANIMATE_START
constexpr uint32_t flag_animate_update = 0x10; // VIDEO_MODE_ANIMATE_UPDATE
// Private (driver-defined, outside every ntddvdeo.h bit): a color pointer whose
// pixels carry per-pixel alpha and no AND mask, premultiplied or straight.
// Advertised in VIDEO_POINTER_CAPABILITIES.Flags by this miniport only.
constexpr uint32_t flag_premultiplied_alpha = 0x00010000u;
constexpr uint32_t flag_straight_alpha = 0x00020000u;
constexpr uint32_t accepted_flags = flag_mono | flag_color | flag_animate_start | flag_animate_update |
                                    flag_premultiplied_alpha | flag_straight_alpha;

// The DCN 3.2 limit (dcn32_cursor_max_dimension), restated for SDK-free callers.
constexpr uint32_t max_dimension = 64;

// Pixels layout:
//   mono:          AND rows then XOR rows, each Height rows of WidthInBytes
//                  bytes, 1 bpp MSB first; WidthInBytes >= ceil(Width / 8).
//   color:         AND mask rows (Height rows, mask_stride(Width) bytes, 1 bpp)
//                  then color rows (Height rows of WidthInBytes = Width * 4,
//                  B, G, R, X); GDI result (screen AND mask) XOR color.
//   color + alpha: color rows only (Height rows of Width * 4, B, G, R, A).
enum class PointerFormat : uint32_t { monochrome, masked_color, premultiplied_alpha, straight_alpha };

// 1 bpp mask rows of a color pointer are DWORD aligned, as EngCreateBitmap
// lays out a BMF_1BPP surface.
constexpr uint32_t mask_stride(uint32_t width) noexcept { return (width + 31) / 32 * 4; }

struct PointerShape final {
    PointerFormat format = PointerFormat::monochrome;
    uint32_t width = 0;
    uint32_t height = 0;
    bool enable = false;
    int16_t column = 0;
    int16_t row = 0;
    // Borrowed from the decoded buffer: valid only while it is.
    ByteView mask{nullptr, 0};
    uint32_t mask_stride = 0;
    ByteView color{nullptr, 0};
    uint32_t color_stride = 0;
};

// Payload bytes for a shape (0 for an invalid description).
constexpr uint32_t payload_bytes(PointerFormat format, uint32_t width, uint32_t height,
                                 uint32_t width_in_bytes) noexcept {
    if (width == 0 || height == 0 || width > max_dimension || height > max_dimension) {
        return 0;
    }

    switch (format) {
    case PointerFormat::monochrome:
        return width_in_bytes >= (width + 7) / 8 && width_in_bytes <= max_dimension ?
            width_in_bytes * height * 2 : 0;
    case PointerFormat::masked_color:
        return width_in_bytes == width * 4 ? mask_stride(width) * height + width_in_bytes * height : 0;
    case PointerFormat::premultiplied_alpha:
    case PointerFormat::straight_alpha:
        return width_in_bytes == width * 4 ? width_in_bytes * height : 0;
    default:
        return 0;
    }
}

// The largest legal request: a 64 x 64 masked color pointer.
constexpr uint32_t max_attributes_bytes =
    attributes_header_bytes + mask_stride(max_dimension) * max_dimension + max_dimension * 4 * max_dimension;

enum class DecodeStatus { decoded, truncated, invalid_flags, invalid_size, oversized };

// Strict decode of one SET_POINTER_ATTR input buffer. Exactly one of mono or
// color; alpha bits only with color and never both; Enable 0 or 1; the input
// holds header plus payload and at most max_attributes_bytes. output borrows
// input; it is written only on decoded.
inline DecodeStatus decode_attributes(ByteView input, PointerShape& output) noexcept {
    uint32_t flags = 0;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t width_in_bytes = 0;
    uint32_t enable = 0;
    uint16_t column = 0;
    uint16_t row = 0;

    if (input.size() > max_attributes_bytes) {
        return DecodeStatus::oversized;
    }

    if (!input.read_u32(0, flags) || !input.read_u32(4, width) || !input.read_u32(8, height) ||
        !input.read_u32(12, width_in_bytes) || !input.read_u32(16, enable) || !input.read_u16(20, column) ||
        !input.read_u16(22, row)) {
        return DecodeStatus::truncated;
    }

    const bool mono = (flags & flag_mono) != 0;
    const bool color = (flags & flag_color) != 0;
    const bool premultiplied = (flags & flag_premultiplied_alpha) != 0;
    const bool straight = (flags & flag_straight_alpha) != 0;

    if ((flags & ~accepted_flags) != 0 || mono == color || (mono && (premultiplied || straight)) ||
        (premultiplied && straight) || enable > 1) {
        return DecodeStatus::invalid_flags;
    }

    const PointerFormat format = mono ? PointerFormat::monochrome :
        (premultiplied ? PointerFormat::premultiplied_alpha :
                         (straight ? PointerFormat::straight_alpha : PointerFormat::masked_color));
    const uint32_t payload = payload_bytes(format, width, height, width_in_bytes);

    if (payload == 0) {
        return DecodeStatus::invalid_size;
    }

    ByteView pixels(nullptr, 0);

    if (!input.subview(attributes_header_bytes, payload, pixels)) {
        return DecodeStatus::truncated;
    }

    PointerShape shape{};
    shape.format = format;
    shape.width = width;
    shape.height = height;
    shape.enable = enable == 1;
    shape.column = static_cast<int16_t>(column);
    shape.row = static_cast<int16_t>(row);

    if (format == PointerFormat::monochrome) {
        shape.mask = pixels;
        shape.mask_stride = width_in_bytes;
    } else if (format == PointerFormat::masked_color) {
        const uint32_t mask_bytes = mask_stride(width) * height;
        pixels.subview(0, mask_bytes, shape.mask);
        pixels.subview(mask_bytes, payload - mask_bytes, shape.color);
        shape.mask_stride = mask_stride(width);
        shape.color_stride = width_in_bytes;
    } else {
        shape.color = pixels;
        shape.color_stride = width_in_bytes;
    }

    output = shape;
    return DecodeStatus::decoded;
}

// VIDEO_POINTER_POSITION: exactly position_bytes of input.
inline bool decode_position(ByteView input, int16_t& column, int16_t& row) noexcept {
    uint16_t x = 0;
    uint16_t y = 0;

    if (input.size() != position_bytes || !input.read_u16(0, x) || !input.read_u16(2, y)) {
        return false;
    }

    column = static_cast<int16_t>(x);
    row = static_cast<int16_t>(y);
    return true;
}

// GDI DrvSetPointerShape/DrvMovePointer report the hotspot position; x == -1
// hides. The VideoPort Column/Row are the image's top-left, signed 16 bit.
// False when the top-left does not fit (the caller then hides or declines).
constexpr bool top_left(int32_t x, int32_t y, int32_t hot_x, int32_t hot_y, int16_t& column,
                        int16_t& row) noexcept {
    const int64_t left = static_cast<int64_t>(x) - hot_x;
    const int64_t top = static_cast<int64_t>(y) - hot_y;

    if (hot_x < 0 || hot_y < 0 || left < INT16_MIN || left > INT16_MAX || top < INT16_MIN || top > INT16_MAX) {
        return false;
    }

    column = static_cast<int16_t>(left);
    row = static_cast<int16_t>(top);
    return true;
}

} // namespace rx7900::reactos::cursor_policy

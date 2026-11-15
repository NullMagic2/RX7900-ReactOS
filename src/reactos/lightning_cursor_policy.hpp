/*
Module: lightning.reactos.cursor_policy
Purpose: Validate shared cursor storage and signed native cursor coordinates.
Created: 2026-10-21
Architecture: Portable policy reused by cached GDI and the assigned VideoPort owner.
*/
#pragma once

#include <stdint.h>
#include "lightning-memory.h"

namespace lightning::reactos::cursor {

static_assert(LIGHTNING_SCANOUT_CURSOR_PITCH_ALIGN == sizeof(uint32_t),
    "Native cursor rows must support complete register word writes");

struct ShapeLayout final {
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t pitch = 0;
    uint32_t format = 0;
    uint32_t hot_x = 0;
    uint32_t hot_y = 0;
    uint32_t payload_bytes = 0;
};

// This validates geometry/storage only. Pixel-dependent background inversion
// still requires the host's exact conversion acknowledgement before acceptance.
// Word-aligned rows permit native bounded register writes without a tail read.
constexpr bool valid_shape(ShapeLayout shape) noexcept {
    if (shape.width == 0 || shape.height == 0 ||
        shape.width > LIGHTNING_SCANOUT_CURSOR_MAX_DIMENSION ||
        shape.height > LIGHTNING_SCANOUT_CURSOR_MAX_DIMENSION ||
        shape.hot_x >= shape.width || shape.hot_y >= shape.height ||
        shape.pitch == 0 || (shape.pitch % LIGHTNING_SCANOUT_CURSOR_PITCH_ALIGN) != 0 ||
        shape.payload_bytes == 0 || shape.payload_bytes > LIGHTNING_SCANOUT_CURSOR_SHAPE_BYTES) {
        return false;
    }

    uint32_t rows = shape.height;
    uint32_t minimum_pitch = 0;

    if (shape.format == LIGHTNING_SCANOUT_CURSOR_MONOCHROME) {
        rows *= LIGHTNING_SCANOUT_CURSOR_MONOCHROME_PLANES;
        minimum_pitch = (shape.width + LIGHTNING_SCANOUT_BITS_PER_BYTE - 1) / LIGHTNING_SCANOUT_BITS_PER_BYTE;
    } else if (shape.format == LIGHTNING_SCANOUT_CURSOR_COLOR ||
               shape.format == LIGHTNING_SCANOUT_CURSOR_MASKED_COLOR) {
        minimum_pitch = shape.width * LIGHTNING_SCANOUT_PIXEL_BYTES;
    } else {
        return false;
    }

    return shape.pitch >= minimum_pitch && uint64_t{shape.pitch} * rows == shape.payload_bytes;
}

struct Position final {
    int32_t left = 0;
    int32_t top = 0;
    uint32_t packed = 0;
    bool visible = false;
};

// Native positions are top-left coordinates; hiding ignores coordinates. On
// failure output is cleared, preventing reuse of a previous accepted position.
constexpr bool from_top_left(int32_t left, int32_t top, bool visible, Position& output) noexcept {
    output = {};

    if (!visible) {
        return true;
    }

    if (left < INT16_MIN || left > INT16_MAX || top < INT16_MIN || top > INT16_MAX) {
        return false;
    }

    output.left = left;
    output.top = top;
    output.packed = static_cast<uint16_t>(left) |
        (static_cast<uint32_t>(static_cast<uint16_t>(top)) << LIGHTNING_SCANOUT_CURSOR_POSITION_SHIFT);
    output.visible = true;
    return true;
}

// GDI reports the hotspot, with x == -1 reserved for hide. Offscreen negative
// top-left coordinates stay signed; subtraction uses 64 bits before narrowing.
// The shape validator separately checks hotspots against the actual dimensions.
constexpr bool from_hotspot(int32_t x, int32_t y, int32_t hot_x, int32_t hot_y,
                           Position& output) noexcept {
    output = {};

    if (hot_x < 0 || hot_y < 0 || hot_x >= static_cast<int32_t>(LIGHTNING_SCANOUT_CURSOR_MAX_DIMENSION) ||
        hot_y >= static_cast<int32_t>(LIGHTNING_SCANOUT_CURSOR_MAX_DIMENSION)) {
        return false;
    }

    if (x == -1) {
        return true;
    }

    const int64_t left = static_cast<int64_t>(x) - hot_x;
    const int64_t top = static_cast<int64_t>(y) - hot_y;

    if (left < INT16_MIN || left > INT16_MAX || top < INT16_MIN || top > INT16_MAX) {
        return false;
    }

    return from_top_left(static_cast<int32_t>(left), static_cast<int32_t>(top), true, output);
}

} // namespace lightning::reactos::cursor

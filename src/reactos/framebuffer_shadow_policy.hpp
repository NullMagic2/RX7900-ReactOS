/*
Module: rx7900.reactos.framebuffer_shadow_policy
Purpose: Bound framebuffer allocation and dirty rectangles before GDI operations.
Created: 2026-10-21
Architecture: Pure policy shared by the native framebuf adapter and host tests.
*/
#pragma once

#include <stdint.h>

namespace rx7900::reactos::framebuffer {

struct Rectangle final {
    int32_t left = 0;
    int32_t top = 0;
    int32_t right = 0;
    int32_t bottom = 0;
};

// Engine bitmap dimensions, row pitch and its signed byte-count intermediate
// must remain representable. The mapped framebuffer retains its separate owner.
constexpr bool valid_layout(uint32_t width, uint32_t height, uint32_t stride,
                            uint32_t mapped_bytes) noexcept {
    return width != 0 && height != 0 && width <= 0x7fffffffU &&
           height <= 0x7fffffffU && stride <= 0x1fffffffU && (stride & 3U) == 0 &&
           static_cast<uint64_t>(width) * 4U <= stride &&
           static_cast<uint64_t>(stride) * height <= 0x7fffffffU &&
           static_cast<uint64_t>(stride) * height <= mapped_bytes;
}

// Normalize without subtraction/negation, then intersect with a validated screen.
// Empty output is all zero. Coordinates never escape [0,width] x [0,height].
constexpr Rectangle clip(Rectangle input, int32_t width, int32_t height) noexcept {
    if (input.right < input.left) {
        const int32_t value = input.left;
        input.left = input.right;
        input.right = value;
    }
    if (input.bottom < input.top) {
        const int32_t value = input.top;
        input.top = input.bottom;
        input.bottom = value;
    }
    if (input.left < 0) input.left = 0;
    if (input.top < 0) input.top = 0;
    if (input.right > width) input.right = width;
    if (input.bottom > height) input.bottom = height;
    if (input.left >= input.right || input.top >= input.bottom) return {};
    return input;
}

constexpr bool empty(Rectangle rectangle) noexcept {
    return rectangle.left >= rectangle.right || rectangle.top >= rectangle.bottom;
}

constexpr Rectangle intersect(Rectangle a, Rectangle b) noexcept {
    if (a.left < b.left) a.left = b.left;
    if (a.top < b.top) a.top = b.top;
    if (a.right > b.right) a.right = b.right;
    if (a.bottom > b.bottom) a.bottom = b.bottom;
    return empty(a) ? Rectangle{} : a;
}

// Both inputs are previously clipped to the same screen. Retained dirty bounds
// survive a failed flush, so a later successful operation can repair the output.
constexpr Rectangle unite(Rectangle a, Rectangle b) noexcept {
    if (empty(a)) return b;
    if (empty(b)) return a;
    if (a.left > b.left) a.left = b.left;
    if (a.top > b.top) a.top = b.top;
    if (a.right < b.right) a.right = b.right;
    if (a.bottom < b.bottom) a.bottom = b.bottom;
    return a;
}

} // namespace rx7900::reactos::framebuffer

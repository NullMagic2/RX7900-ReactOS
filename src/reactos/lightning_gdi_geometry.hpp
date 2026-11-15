/*
Module: lightning.reactos.gdi_geometry
Purpose: Translate native GDI rectangles into bounded canvas/source regions.
Created: 2026-10-21
Architecture: Reuses cached framebuffer geometry; contains no rasterizer.
*/
#pragma once

#include "framebuffer_shadow_policy.hpp"
#include <stdint.h>

namespace lightning::reactos::gdi {
using rx7900::reactos::framebuffer::Rectangle;
namespace geometry = rx7900::reactos::framebuffer;

// These two ROP4s ignore source, brush and mask. Match the existing DIB engine's
// full32-bit constant, including the high byte; mixed foreground/background
// ROPs must still use their real mask/engine semantics.
constexpr bool constant_fill_rop(uint32_t rop, uint32_t& color) noexcept {
    constexpr uint32_t blackness = 0x0000U;
    constexpr uint32_t whiteness = 0xffffU;
    color = 0;

    if (rop == blackness) return true;
    if (rop == whiteness) {
        color = UINT32_MAX;
        return true;
    }

    return false;
}

// Mirrored or empty GDI rectangles deliberately use the engine fallback. Do
// every offset calculation in 64 bits before narrowing to native coordinates.
constexpr bool draw_bounds(Rectangle requested, int32_t width, int32_t height,
                           Rectangle& output) noexcept {
    if (width <= 0 || height <= 0 || geometry::empty(requested)) return false;
    output = geometry::intersect(requested, {0, 0, width, height});
    return true;
}

// Clip is a subset of the original destination. The returned source position
// describes the corresponding top-left source pixel, including screen clipping.
constexpr bool source_position(Rectangle original, Rectangle clipped,
                               int32_t source_x, int32_t source_y,
                               int32_t source_width, int32_t source_height,
                               int32_t& output_x, int32_t& output_y) noexcept {
    if (geometry::empty(original) || geometry::empty(clipped) ||
        clipped.left < original.left || clipped.top < original.top ||
        clipped.right > original.right || clipped.bottom > original.bottom ||
        source_width <= 0 || source_height <= 0) return false;
    const int64_t x = static_cast<int64_t>(source_x) + clipped.left - original.left;
    const int64_t y = static_cast<int64_t>(source_y) + clipped.top - original.top;
    const int64_t width = static_cast<int64_t>(clipped.right) - clipped.left;
    const int64_t height = static_cast<int64_t>(clipped.bottom) - clipped.top;
    if (x < 0 || y < 0 || x >= source_width || y >= source_height ||
        width > static_cast<int64_t>(source_width) - x ||
        height > static_cast<int64_t>(source_height) - y) return false;
    output_x = static_cast<int32_t>(x);
    output_y = static_cast<int32_t>(y);
    return true;
}

// GPU operations invalidate only their destination bounds. This owner tracks
// cache visibility, not GPU resource/fence lifetime (owned by the command core).
// A failed/uncertain readback never clears stale pixels or authorizes fallback.
class CacheVisibility final {
public:
    void gpu_write(Rectangle rectangle) noexcept {
        stale_ = geometry::unite(stale_, rectangle);
    }
    Rectangle stale() const noexcept { return stale_; }
    bool readable() const noexcept { return !poisoned_ && geometry::empty(stale_); }
    bool poisoned() const noexcept { return poisoned_; }
    void readback_complete() noexcept { if (!poisoned_) stale_ = {}; }
    void poison() noexcept { poisoned_ = true; }
    // Only a real mode/canvas reinitialization may establish a fresh cache.
    void reinitialize() noexcept { stale_ = {}; poisoned_ = false; }
private:
    Rectangle stale_{};
    bool poisoned_ = false;
};

} // namespace lightning::reactos::gdi

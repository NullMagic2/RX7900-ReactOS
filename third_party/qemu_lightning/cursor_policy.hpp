/*
 * Module: lightning.display.cursor_policy
 * Purpose: Share bounded exact-shape and viewport rules across cursor adapters.
 * Created: 2026-10-21
 * Architecture: Pure C++17, with no UI handles or retained input pointers.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef LIGHTNING_DISPLAY_CURSOR_POLICY_HPP
#define LIGHTNING_DISPLAY_CURSOR_POLICY_HPP
#include <stddef.h>
#include <stdint.h>
#include <limits.h>

namespace lightning::cursor {
enum class Format { monochrome, color, masked_color };

struct Layout {
    uint32_t width, height, pitch, hot_x, hot_y;
    Format format;
};

/* Limits are supplied from the single wire contract by each ABI adapter. */
inline bool exact_shape(const Layout &layout, uint32_t maximum_dimension,
    const uint8_t *source, size_t bytes) noexcept {
    if (source == nullptr || layout.width == 0 || layout.height == 0 ||
        layout.width > maximum_dimension || layout.height > maximum_dimension ||
        layout.hot_x >= layout.width || layout.hot_y >= layout.height ||
        layout.pitch == 0 || layout.pitch % 4 != 0 ||
        (layout.format != Format::monochrome && layout.format != Format::color &&
            layout.format != Format::masked_color)) {
        return false;
    }
    const bool mono = layout.format == Format::monochrome;
    const uint64_t row = mono ? (uint64_t{layout.width} + 7) / 8 :
        uint64_t{layout.width} * 4;
    const uint64_t rows = uint64_t{layout.height} * (mono ? 2 : 1);

    if (layout.pitch < row || layout.pitch > bytes / rows) {
        return false;
    }
    for (uint32_t y = 0; y < layout.height; ++y) {
        for (uint32_t x = 0; x < layout.width; ++x) {
            const size_t offset = size_t{y} * layout.pitch;

            if (mono) {
                const size_t byte = offset + x / 8;
                const auto bit = static_cast<uint8_t>(0x80u >> (x % 8));

                if ((source[byte] & source[size_t{layout.height} * layout.pitch + byte] & bit) != 0) {
                    return false;
                }
            } else if (layout.format == Format::masked_color) {
                const size_t pixel = offset + size_t{x} * 4;

                if (source[pixel + 3] >= 0x80 &&
                    (source[pixel] != 0 || source[pixel + 1] != 0 || source[pixel + 2] != 0)) {
                    return false;
                }
            }
        }
    }
    return true;
}

/* SPS_ALPHA is premultiplied; the existing COLOR wire/GdkPixbuf is straight.
 * Choose the nearest representable straight byte, clamping invalid premul RGB.
 * Alpha zero has no visible color. This does not alter opaque/masked shapes.
 */
constexpr uint8_t straight_channel(uint8_t channel, uint8_t alpha) noexcept {
    if (alpha == 0) {
        return 0;
    }
    const uint32_t result = (uint32_t{channel} * 255 + alpha / 2) / alpha;
    return static_cast<uint8_t>(result > 255 ? 255 : result);
}

/* Integer canvas extents match GTK input's truncated surface-size convention.
 * Negative/offscreen guest points remain offscreen; no silent edge clamp.
 */
inline bool viewport_axis(int32_t point, uint32_t canvas,
    uint32_t widget, double scale, int32_t &result) noexcept {
    if (!(scale > 0.0) || scale > 1024.0 || canvas == 0 || widget == 0) {
        return false;
    }
    const double extent = static_cast<double>(canvas) * scale;

    if (!(extent <= INT32_MAX)) {
        return false;
    }
    const auto extent_integer = static_cast<int32_t>(extent);
    const double offset = widget > static_cast<uint32_t>(extent_integer) ?
        (widget - static_cast<uint32_t>(extent_integer)) / 2 : 0;
    const double transformed = static_cast<double>(point) * scale + offset;

    if (!(transformed >= INT32_MIN &&
        transformed <= INT32_MAX)) {
        return false;
    }
    const auto truncated = static_cast<int32_t>(transformed);
    result = transformed < static_cast<double>(truncated) ? truncated - 1 : truncated;
    return true;
}

struct OverlayInput {
    int32_t hotspot_x = 0;
    int32_t hotspot_y = 0;
    uint32_t shape_width = 0;
    uint32_t shape_height = 0;
    uint32_t hot_x = 0;
    uint32_t hot_y = 0;
    uint32_t canvas_width = 0;
    uint32_t canvas_height = 0;
    uint32_t widget_width = 0;
    uint32_t widget_height = 0;
    uint32_t device_scale = 0;
    double scale_x = 0.0;
    double scale_y = 0.0;
    // True when the cursor's logical first row was uploaded at texture v = 0.
    bool texture_top_at_v0 = true;
};

struct OverlayPlacement {
    bool visible = false;
    uint32_t framebuffer_width = 0;
    uint32_t framebuffer_height = 0;
    // Physical GL bottom-left and upper-right edges, without integer rounding.
    double x0 = 0.0;
    double y0 = 0.0;
    double x1 = 0.0;
    double y1 = 0.0;
    // Texture coordinates corresponding to those same destination corners.
    double u0 = 0.0;
    double v0 = 0.0;
    double u1 = 0.0;
    double v1 = 0.0;
};

namespace detail {
inline double unit_fraction(double value) noexcept {
    // Only derived source fractions are clamped for floating point roundoff.
    // Cursor positions and destination geometry are never clamped to an edge.
    return value < 0.0 ? 0.0 : (value > 1.0 ? 1.0 : value);
}
}

/* Clip in guest space before applying the existing GTK viewport transform.
 * Widget sizes/scales are logical GTK units; device_scale converts once into
 * physical framebuffer pixels. GL framebuffer orientation is always bottom-up,
 * independent of the primary image's texture orientation. A valid cursor with
 * no visible pixels succeeds with visible = false, not a failed MOVE. Invalid
 * input clears output and performs no pointer, allocation or GL operation.
 */
inline bool overlay_placement(const OverlayInput &input, OverlayPlacement &output) noexcept {
    output = {};
    if (input.shape_width == 0 || input.shape_height == 0 ||
        input.shape_width > INT32_MAX || input.shape_height > INT32_MAX ||
        input.hot_x >= input.shape_width || input.hot_y >= input.shape_height ||
        input.canvas_width > INT32_MAX || input.canvas_height > INT32_MAX ||
        input.widget_width > INT32_MAX || input.widget_height > INT32_MAX ||
        input.device_scale == 0) {
        return false;
    }

    const uint64_t framebuffer_width = uint64_t{input.widget_width} * input.device_scale;
    const uint64_t framebuffer_height = uint64_t{input.widget_height} * input.device_scale;
    int32_t offset_x = 0, offset_y = 0;
    if (framebuffer_width > INT32_MAX || framebuffer_height > INT32_MAX ||
        !viewport_axis(0, input.canvas_width, input.widget_width, input.scale_x, offset_x) ||
        !viewport_axis(0, input.canvas_height, input.widget_height, input.scale_y, offset_y)) {
        return false;
    }

    OverlayPlacement candidate;
    candidate.framebuffer_width = static_cast<uint32_t>(framebuffer_width);
    candidate.framebuffer_height = static_cast<uint32_t>(framebuffer_height);
    const int64_t left = static_cast<int64_t>(input.hotspot_x) - input.hot_x;
    const int64_t top = static_cast<int64_t>(input.hotspot_y) - input.hot_y;
    const int64_t right = left + input.shape_width;
    const int64_t bottom = top + input.shape_height;
    const int64_t guest_left = left < 0 ? 0 : left;
    const int64_t guest_top = top < 0 ? 0 : top;
    const int64_t guest_right = right > input.canvas_width ? input.canvas_width : right;
    const int64_t guest_bottom = bottom > input.canvas_height ? input.canvas_height : bottom;
    if (guest_left >= guest_right || guest_top >= guest_bottom) {
        output = candidate;
        return true;
    }

    const double mapped_left = offset_x + static_cast<double>(guest_left) * input.scale_x;
    const double mapped_top = offset_y + static_cast<double>(guest_top) * input.scale_y;
    const double mapped_right = offset_x + static_cast<double>(guest_right) * input.scale_x;
    const double mapped_bottom = offset_y + static_cast<double>(guest_bottom) * input.scale_y;
    const double clipped_left = mapped_left < 0.0 ? 0.0 : mapped_left;
    const double clipped_top = mapped_top < 0.0 ? 0.0 : mapped_top;
    const double clipped_right = mapped_right > input.widget_width ? input.widget_width : mapped_right;
    const double clipped_bottom = mapped_bottom > input.widget_height ? input.widget_height : mapped_bottom;
    if (clipped_left >= clipped_right || clipped_top >= clipped_bottom) {
        output = candidate;
        return true;
    }

    const double source_left = static_cast<double>(guest_left - left) +
        (clipped_left - mapped_left) / input.scale_x;
    const double source_top = static_cast<double>(guest_top - top) +
        (clipped_top - mapped_top) / input.scale_y;
    const double source_right = static_cast<double>(guest_right - left) -
        (mapped_right - clipped_right) / input.scale_x;
    const double source_bottom = static_cast<double>(guest_bottom - top) -
        (mapped_bottom - clipped_bottom) / input.scale_y;
    const double u_left = detail::unit_fraction(source_left / input.shape_width);
    const double u_right = detail::unit_fraction(source_right / input.shape_width);
    const double v_top = detail::unit_fraction(source_top / input.shape_height);
    const double v_bottom = detail::unit_fraction(source_bottom / input.shape_height);
    if (!(u_left >= 0.0 && u_left < u_right && u_right <= 1.0 &&
        v_top >= 0.0 && v_top < v_bottom && v_bottom <= 1.0)) {
        return false;
    }

    candidate.x0 = clipped_left * input.device_scale;
    candidate.x1 = clipped_right * input.device_scale;
    candidate.y0 = (input.widget_height - clipped_bottom) * input.device_scale;
    candidate.y1 = (input.widget_height - clipped_top) * input.device_scale;
    candidate.u0 = u_left;
    candidate.u1 = u_right;
    candidate.v0 = input.texture_top_at_v0 ? v_bottom : 1.0 - v_bottom;
    candidate.v1 = input.texture_top_at_v0 ? v_top : 1.0 - v_top;
    candidate.visible = true;
    output = candidate;
    return true;
}
} // namespace lightning::cursor
#endif

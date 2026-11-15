/*
Module: rx7900.opengl.radeon_gdi_triangle
Purpose: Checked clip-space to GDI triangle conversion for an authorized HDC.
Created: 2026-10-23
Architecture: Reuses FixedFunction's vertices and the current GDI display path.
*/
#include "radeon_gdi_triangle.hpp"
#include "../reactos/radeon_2d_abi.hpp"

#include <cmath>

namespace rx7900::opengl {
namespace {

constexpr uint32_t maximum_drawable_dimension =
    reactos::radeon2d::abi::max_surface_dimension;

bool finite(const Vertex& vertex) noexcept {
    return std::isfinite(vertex.clip.x) && std::isfinite(vertex.clip.y) &&
           std::isfinite(vertex.clip.z) && std::isfinite(vertex.clip.w) &&
           std::isfinite(vertex.color.red) && std::isfinite(vertex.color.green) &&
           std::isfinite(vertex.color.blue) && std::isfinite(vertex.color.alpha);
}

bool valid_color(const Color& color) noexcept {
    return color.red >= 0 && color.red <= 1 && color.green >= 0 &&
           color.green <= 1 && color.blue >= 0 && color.blue <= 1 &&
           color.alpha == 1;
}

bool inside(const Vector4& clip) noexcept {
    return clip.w > 0 && clip.x >= -clip.w && clip.x <= clip.w &&
           clip.y >= -clip.w && clip.y <= clip.w && clip.z >= -clip.w &&
           clip.z <= clip.w;
}

uint16_t channel(double value) noexcept {
    return static_cast<uint16_t>(value * 65535.0 + 0.5);
}

bool project(const Vertex& vertex, const Viewport& viewport, uint32_t width,
             uint32_t height, GdiTriangleVertex& result) noexcept {
    const double inverse_w = 1.0 / vertex.clip.w;
    const double x = static_cast<double>(viewport.x) +
                     (vertex.clip.x * inverse_w + 1.0) * 0.5 * viewport.width;
    const double y = static_cast<double>(viewport.y) +
                     (1.0 - vertex.clip.y * inverse_w) * 0.5 * viewport.height;
    if (!std::isfinite(x) || !std::isfinite(y) || x < 0 || y < 0 ||
        x > width || y > height) {
        return false;
    }

    result.x = static_cast<int32_t>(x + 0.5);
    result.y = static_cast<int32_t>(y + 0.5);
    result.red = channel(vertex.color.red);
    result.green = channel(vertex.color.green);
    result.blue = channel(vertex.color.blue);
    return true;
}

} // namespace

GdiTriangleStatus RadeonGdiTriangle::triangle(const Vertex& a, const Vertex& b,
                                                const Vertex& c,
                                                const RasterState& state) noexcept {
    if (transport_.draw == nullptr || transport_.context == nullptr ||
        width_ == 0 || height_ == 0 || width_ > maximum_drawable_dimension ||
        height_ > maximum_drawable_dimension) {
        return GdiTriangleStatus::invalid_input;
    }
    if (!finite(a) || !finite(b) || !finite(c) ||
        !valid_color(a.color) || !valid_color(b.color) || !valid_color(c.color)) {
        return GdiTriangleStatus::invalid_input;
    }
    if (state.depth_enabled || state.smooth_colors ||
        a.color.red != b.color.red || a.color.red != c.color.red ||
        a.color.green != b.color.green || a.color.green != c.color.green ||
        a.color.blue != b.color.blue || a.color.blue != c.color.blue ||
        state.viewport.width == 0 || state.viewport.height == 0 ||
        state.viewport.width > maximum_drawable_dimension ||
        state.viewport.height > maximum_drawable_dimension ||
        !inside(a.clip) || !inside(b.clip) || !inside(c.clip)) {
        return GdiTriangleStatus::unsupported;
    }

    const double ax = a.clip.x / a.clip.w;
    const double ay = a.clip.y / a.clip.w;
    const double bx = b.clip.x / b.clip.w;
    const double by = b.clip.y / b.clip.w;
    const double cx = c.clip.x / c.clip.w;
    const double cy = c.clip.y / c.clip.w;
    const double area = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
    if (!std::isfinite(area)) {
        return GdiTriangleStatus::invalid_input;
    }
    if (area == 0 || (state.cull_back_faces && area < 0)) {
        return GdiTriangleStatus::empty;
    }

    GdiTriangleVertex vertices[3]{};
    if (!project(a, state.viewport, width_, height_, vertices[0]) ||
        !project(b, state.viewport, width_, height_, vertices[1]) ||
        !project(c, state.viewport, width_, height_, vertices[2])) {
        return GdiTriangleStatus::unsupported;
    }
    const int64_t dx1 = static_cast<int64_t>(vertices[1].x) - vertices[0].x;
    const int64_t dy1 = static_cast<int64_t>(vertices[1].y) - vertices[0].y;
    const int64_t dx2 = static_cast<int64_t>(vertices[2].x) - vertices[0].x;
    const int64_t dy2 = static_cast<int64_t>(vertices[2].y) - vertices[0].y;
    if (dx1 * dy2 == dy1 * dx2) {
        return GdiTriangleStatus::empty;
    }

    return transport_.draw(transport_.context, vertices) ?
        GdiTriangleStatus::accepted : GdiTriangleStatus::draw_failed;
}

GraphicsBackend RadeonGdiTriangle::graphics_backend() noexcept {
    return {this, submit_triangle, nullptr};
}

Status RadeonGdiTriangle::submit_triangle(void* context, const Vertex& a,
                                           const Vertex& b, const Vertex& c,
                                           const RasterState& state) noexcept {
    if (context == nullptr) {
        return Status::backend_not_ready;
    }
    switch (static_cast<RadeonGdiTriangle*>(context)->triangle(a, b, c, state)) {
    case GdiTriangleStatus::accepted:
    case GdiTriangleStatus::empty:
        return Status::success;
    case GdiTriangleStatus::invalid_input:
        return Status::invalid_value;
    case GdiTriangleStatus::unsupported:
        return Status::backend_not_ready;
    case GdiTriangleStatus::draw_failed:
        return Status::backend_failure;
    }
    return Status::backend_failure;
}

} // namespace rx7900::opengl

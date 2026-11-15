/*
Module: rx7900.opengl.radeon_gdi_triangle
Purpose: Send a restricted fixed-function triangle through the caller's GDI HDC.
Created: 2026-10-23
Architecture: The standard GradientFill route provides the current DC clip and
may reach radeondisp's Radeon2D compute gradient hook. No private GPU ABI is
exposed to user mode; this is not a conformant OpenGL or WGL implementation.
*/
#pragma once

#include "fixed_function.hpp"

namespace rx7900::opengl {

struct GdiTriangleVertex final {
    int32_t x = 0;
    int32_t y = 0;
    uint16_t red = 0;
    uint16_t green = 0;
    uint16_t blue = 0;
    uint16_t alpha = 0xffffU;
};

struct GdiTriangleTransport final {
    void* context = nullptr;
    // Caller owns a live, authorized HDC for the whole call. The implementation
    // must issue standard GDI GradientFill on that HDC; GDI owns DC clipping,
    // target selection and driver dispatch. Vertices are borrowed for the call.
    // TRUE means GDI drew or submitted, not proof that Radeon compute ran.
    bool (*draw)(void*, const GdiTriangleVertex (&vertices)[3]) noexcept = nullptr;
};

enum class GdiTriangleStatus {
    accepted, empty, unsupported, invalid_input, draw_failed
};

// User-mode only. The context owner serializes access and retains its HDC for
// this object's lifetime. Recreate after a drawable or mode transition.
class RadeonGdiTriangle final {
public:
    RadeonGdiTriangle(uint32_t width, uint32_t height,
                      GdiTriangleTransport transport) noexcept
        : width_(width), height_(height), transport_(transport) {}
    RadeonGdiTriangle(const RadeonGdiTriangle&) = delete;
    RadeonGdiTriangle& operator=(const RadeonGdiTriangle&) = delete;

    // Fully in-frustum, opaque flat-colour triangles only. Depth, smoothing,
    // clipping, exact GL coverage, clear and swap remain unsupported.
    GdiTriangleStatus triangle(const Vertex& a, const Vertex& b, const Vertex& c,
                               const RasterState& state) noexcept;
    GraphicsBackend graphics_backend() noexcept;

private:
    static Status submit_triangle(void* context, const Vertex& a, const Vertex& b,
                                  const Vertex& c, const RasterState& state) noexcept;
    uint32_t width_ = 0;
    uint32_t height_ = 0;
    GdiTriangleTransport transport_{};
};

} // namespace rx7900::opengl

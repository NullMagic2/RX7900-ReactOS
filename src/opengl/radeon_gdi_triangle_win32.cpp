/*
Module: rx7900.opengl.radeon_gdi_triangle_win32
Purpose: Call standard GDI GradientFill with the current HDC's clipping.
Created: 2026-10-23
Architecture: ReactOS gdi32 forwards through IntEngGradientFill/DrvGradientFill;
the OS, not this module, authenticates and clips the destination DC.
*/
#include "radeon_gdi_triangle_win32.hpp"

namespace rx7900::opengl {
namespace {

bool draw_gradient(void* context, const GdiTriangleVertex (&vertices)[3]) noexcept {
    HDC dc = static_cast<HDC>(context);
    if (dc == nullptr) {
        return false;
    }

    TRIVERTEX gdi_vertices[3]{};
    for (uint32_t index = 0; index < 3; ++index) {
        gdi_vertices[index].x = vertices[index].x;
        gdi_vertices[index].y = vertices[index].y;
        gdi_vertices[index].Red = vertices[index].red;
        gdi_vertices[index].Green = vertices[index].green;
        gdi_vertices[index].Blue = vertices[index].blue;
        gdi_vertices[index].Alpha = vertices[index].alpha;
    }
    GRADIENT_TRIANGLE mesh{0, 1, 2};
    return GradientFill(dc, gdi_vertices, 3, &mesh, 1,
                        GRADIENT_FILL_TRIANGLE) != FALSE;
}

} // namespace

GdiTriangleTransport win32_gdi_transport(HDC dc) noexcept {
    return {dc, draw_gradient};
}

} // namespace rx7900::opengl

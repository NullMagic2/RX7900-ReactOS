/*
Module: rx7900.tests.radeon_gdi_triangle
Purpose: Verify the restricted geometry path and user-mode HDC callback contract.
Created: 2026-10-23
Architecture: Host callback fixture; no claim of GPU execution or WGL hookup.
*/
#include "radeon_gdi_triangle.hpp"

#include <cstdio>
#include <limits>

namespace {
namespace gl = rx7900::opengl;

unsigned checks = 0;
unsigned failures = 0;

void check(bool result, const char* name) {
    ++checks;
    if (!result) {
        ++failures;
        std::fprintf(stderr, "FAIL: %s\n", name);
    }
}

struct Capture final {
    unsigned calls = 0;
    bool succeeds = true;
    gl::GdiTriangleVertex vertices[3]{};
};

bool draw(void* context, const gl::GdiTriangleVertex (&vertices)[3]) noexcept {
    Capture& capture = *static_cast<Capture*>(context);
    ++capture.calls;
    for (uint32_t index = 0; index < 3; ++index) {
        capture.vertices[index] = vertices[index];
    }
    return capture.succeeds;
}

gl::Vertex vertex(double x, double y, double red = 1.0) {
    return {{x, y, 0, 1}, {0, 0, 1}, {red, 0.25, 0.5, 1}};
}

void test_geometry() {
    Capture capture{};
    gl::RadeonGdiTriangle adapter(128, 128, {&capture, draw});
    gl::RasterState state{{0, 0, 128, 128}, false, false, false};
    const gl::Vertex a = vertex(-0.5, -0.5);
    const gl::Vertex b = vertex(0.5, -0.5);
    const gl::Vertex c = vertex(0, 0.5);
    check(adapter.triangle(a, b, c, state) == gl::GdiTriangleStatus::accepted &&
          capture.calls == 1, "flat triangle reaches authorized HDC callback");
    check(capture.vertices[0].x == 32 && capture.vertices[0].y == 96 &&
          capture.vertices[1].x == 96 && capture.vertices[1].y == 96 &&
          capture.vertices[2].x == 64 && capture.vertices[2].y == 32 &&
          capture.vertices[0].red == 0xffffU &&
          capture.vertices[0].green == 0x4000U &&
          capture.vertices[0].blue == 0x8000U &&
          capture.vertices[0].alpha == 0xffffU,
          "lower-left projection and 16-bit colour");
    const gl::GraphicsBackend backend = adapter.graphics_backend();
    check(backend.context == &adapter && backend.triangle != nullptr &&
          backend.clear == nullptr &&
          backend.triangle(backend.context, a, b, c, state) == gl::Status::success &&
          capture.calls == 2, "existing FixedFunction backend consumes adapter");

    state.depth_enabled = true;
    check(adapter.triangle(a, b, c, state) == gl::GdiTriangleStatus::unsupported &&
          capture.calls == 2, "depth rejected before GDI");
    state.depth_enabled = false;
    state.smooth_colors = true;
    check(adapter.triangle(a, b, c, state) == gl::GdiTriangleStatus::unsupported &&
          capture.calls == 2, "smooth shading rejected before GDI");
    state.smooth_colors = false;
    check(adapter.triangle(a, b, vertex(0, 0.5, 0.5), state) ==
          gl::GdiTriangleStatus::unsupported && capture.calls == 2,
          "nonflat colour rejected before GDI");
    check(adapter.triangle(a, b, vertex(2, 0), state) == gl::GdiTriangleStatus::unsupported &&
          capture.calls == 2, "unclipped geometry rejected before GDI");
    check(adapter.triangle(a, b, vertex(0, -0.5), state) == gl::GdiTriangleStatus::empty &&
          capture.calls == 2, "integer-degenerate geometry skipped");
    state.cull_back_faces = true;
    check(adapter.triangle(c, b, a, state) == gl::GdiTriangleStatus::empty &&
          capture.calls == 2, "back face culled");
    state.cull_back_faces = false;
    gl::Vertex nan = c;
    nan.clip.x = std::numeric_limits<double>::quiet_NaN();
    check(adapter.triangle(a, b, nan, state) == gl::GdiTriangleStatus::invalid_input &&
          capture.calls == 2, "nonfinite input refused");
    capture.succeeds = false;
    check(adapter.triangle(a, b, c, state) == gl::GdiTriangleStatus::draw_failed &&
          capture.calls == 3, "GDI failure propagated");
    gl::RadeonGdiTriangle invalid(0, 128, {&capture, draw});
    check(invalid.triangle(a, b, c, state) == gl::GdiTriangleStatus::invalid_input &&
          capture.calls == 3, "invalid drawable refused");
}

} // namespace

int main() {
    test_geometry();
    std::printf("Radeon GDI triangle: %u checks, %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

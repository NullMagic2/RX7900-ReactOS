/*
Module: rx7900.opengl.gl_raster
Purpose: Software reference rasterizer implementing the gl_pipeline Backend.
Created: 2026-10-23
Architecture: The portable fixed-function fallback renderer and the per-pixel
reference the GFX11 backend's pixel tests compare against. It owns BGRA8 color
buffers (front and optional back), a 32-bit depth buffer and an 8-bit stencil
buffer in caller-provided storage. User mode only; no allocation, no OS calls.
*/
#pragma once

#include "gl_clip.hpp"
#include "gl_pipeline.hpp"

namespace rx7900::opengl {

// Maximum drawable the software backend rasterizes into one caller allocation.
constexpr uint32_t raster_max_dimension = 4096;

// Byte size of the backing store for a width x height double-buffered drawable
// with depth and stencil: two BGRA8 color buffers, one 32-bit depth, one 8-bit
// stencil. Returns 0 on overflow or an out-of-range dimension.
uint64_t raster_storage_bytes(uint32_t width, uint32_t height, bool double_buffered) noexcept;

// Caller owns storage for the whole backend lifetime; it must be at least
// raster_storage_bytes(width, height, format.double_buffered) and 4-byte aligned.
// A zero dimension creates an empty, healthy backend (no drawing occurs).
class SoftwareBackend final : public Backend {
public:
    SoftwareBackend(uint8_t* storage, uint64_t storage_bytes, FramebufferFormat format) noexcept;
    SoftwareBackend(const SoftwareBackend&) = delete;
    SoftwareBackend& operator=(const SoftwareBackend&) = delete;

    bool resize(uint32_t width, uint32_t height) noexcept override;
    uint32_t width() const noexcept override { return width_; }
    uint32_t height() const noexcept override { return height_; }
    FramebufferFormat format() const noexcept override { return format_; }
    void clear(const ClearRequest& request) noexcept override;
    void draw(const DrawState& state, PrimitiveKind kind, const RasterVertex* vertices,
              uint32_t count) noexcept override;
    void draw_pixels(const DrawState& state, const PixelFragments& pixels) noexcept override;
    void read(ReadSource source, gl::Enum buffer, int32_t x, int32_t y, uint32_t width,
              uint32_t height, float* output) noexcept override;
    void finish() noexcept override {}
    void flush() noexcept override {}
    bool healthy() const noexcept override { return healthy_; }

    // BGRA8 scanout access for the presentation path (DrvSwapBuffers / readback).
    // Returns the front or back color buffer; null when empty. Row stride is
    // width pixels; rows are top to bottom for scanout (GL bottom-to-top rows are
    // flipped on access).
    const uint32_t* color_buffer(gl::Enum buffer) const noexcept;
    // Swap the front and back buffers (double-buffered only).
    void swap() noexcept;

private:
    struct Window final {
        float x, y, z;              // window coordinates and depth [0, 1]
        Vec4 color;
        Vec4 texcoord;
        float fog;
        float inv_w;                // 1 / clip.w for perspective-correct attributes
    };

    bool to_window(const RasterVertex& vertex, const DrawState& state, Window& output) const noexcept;
    void raster_triangle(const DrawState& state, const Window& a, const Window& b,
                         const Window& c, float offset) noexcept;
    void raster_line(const DrawState& state, const Window& a, const Window& b) noexcept;
    void raster_point(const DrawState& state, const Window& point) noexcept;
    // lambda: the fragment's texture level of detail (gl_pipeline quad_lod or
    // line_lod; -texture_lod_limit when the primitive has no derivatives).
    void shade_fragment(const DrawState& state, uint32_t x, uint32_t y, float z,
                        const Vec4& color, const Vec4& texcoord, float fog, float lambda) noexcept;
    void write_color(const DrawState& state, uint32_t* target, size_t index, const Vec4& color) noexcept;
    bool scissor_ok(const DrawState& state, int32_t x, int32_t y) const noexcept;
    Vec4 sample_texture(const Texture& texture, const Vec4& coord, float lambda) const noexcept;

    uint8_t* storage_;
    uint64_t storage_bytes_;
    FramebufferFormat format_{};
    uint32_t width_ = 0;
    uint32_t height_ = 0;
    uint32_t* front_ = nullptr;
    uint32_t* back_ = nullptr;
    uint32_t* depth_ = nullptr;     // 32-bit unsigned normalized depth
    uint8_t* stencil_ = nullptr;
    // Line stipple counter s (DrawState::line_stipple_reset).
    uint32_t stipple_counter_ = 0;
    bool healthy_ = true;
};

} // namespace rx7900::opengl

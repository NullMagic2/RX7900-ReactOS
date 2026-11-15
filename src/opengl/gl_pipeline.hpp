/*
Module: rx7900.opengl.gl_pipeline
Purpose: Contract between the portable OpenGL 1.1 frontend and its rasterization
backends: post-transform vertices, the per-draw fragment state snapshot, texture
objects and framebuffer operations.
Created: 2026-10-23
Architecture: The frontend (gl_context) performs vertex transformation, lighting,
texture-coordinate generation, user clip planes, flat-shading propagation and
primitive decomposition. A backend clips to the view volume, rasterizes and runs
every per-fragment operation. The software backend (gl_raster) is the fallback
renderer and the reference for GPU pixel tests; the GFX11 backend derives its
shader key and context registers from the same DrawState. User mode only.
*/
#pragma once

#include "gl_api.hpp"

namespace rx7900::opengl {

struct Vec3 final {
    float x = 0, y = 0, z = 0;
};

struct Vec4 final {
    float x = 0, y = 0, z = 0, w = 0;
};

// Column-major 4x4, element (row r, column c) at values[c * 4 + r], as GL stores it.
struct Matrix4 final {
    float values[16]{};
};

Matrix4 matrix_identity() noexcept;
Matrix4 matrix_multiply(const Matrix4& left, const Matrix4& right) noexcept;
Vec4 matrix_transform(const Matrix4& matrix, const Vec4& vector) noexcept;
// General inverse; false for a singular or non-finite matrix.
bool matrix_invert(const Matrix4& matrix, Matrix4& inverse) noexcept;

// One vertex after the frontend's vertex stage. clip is homogeneous clip space
// before the view-volume clip and perspective divide. color[1] is the back-face
// color (equal to color[0] unless two-sided lighting is active). fog is the
// already evaluated fog blend factor (1 keeps the fragment color). Flat shading
// is resolved by the frontend: every vertex of a flat primitive carries the
// provoking color, so backends never need provoking-vertex conventions.
struct RasterVertex final {
    Vec4 clip;
    Vec4 color[2];
    Vec4 texcoord;
    float fog = 1;
};

enum class PrimitiveKind : uint8_t { points, lines, triangles };

constexpr uint32_t texture_max_levels = 11;     // 1024 x 1024 base level
constexpr uint32_t texture_max_size = 1u << (texture_max_levels - 1);

// One mipmap image, stored as R, G, B, A bytes per texel, rows bottom to top.
// Base format (alpha, luminance, luminance_alpha, intensity, rgb, rgba) keeps the
// texture-environment semantics of the GL internal format.
struct TextureLevel final {
    uint32_t width = 0;
    uint32_t height = 0;
    gl::Enum base_format = gl::rgba;
    gl::Enum internal_format = gl::rgba;
    uint8_t* texels = nullptr;      // owned by the Texture's allocator
};

struct TextureSampler final {
    gl::Enum min_filter = gl::nearest_mipmap_linear;
    gl::Enum mag_filter = gl::linear;
    gl::Enum wrap_s = gl::repeat;
    gl::Enum wrap_t = gl::repeat;
    Vec4 border_color{0, 0, 0, 0};
};

// Shared texture object. generation increments on every image or sampler change
// so a GPU backend can detect stale uploads. Owned by the context share group.
struct Texture final {
    gl::Uint name = 0;
    gl::Enum target = 0;
    TextureLevel levels[texture_max_levels]{};
    TextureSampler sampler{};
    float priority = 1;
    uint64_t generation = 1;
};

// True when every level the minification filter can reach is present with
// halving dimensions and one base format (GL 1.1 texture completeness).
bool texture_complete(const Texture& texture) noexcept;

// ---- Texture level of detail (GL 1.1 sections 3.8.5 to 3.8.7) ----
// Every backend selects levels with exactly these rules and operation orders; the
// GFX11 pixel shader emits the same float operations (gl_shader_gfx11).
// lambda is clamped to [-texture_lod_limit, texture_lod_limit] so that "no
// derivative" (points, pixel rectangles) stays a finite magnification value.
constexpr float texture_lod_limit = 128.0f;

// True for the four *_MIPMAP_* minification filters.
inline bool mipmap_filter(gl::Enum min_filter) noexcept {
    return min_filter == gl::nearest_mipmap_nearest || min_filter == gl::linear_mipmap_nearest ||
           min_filter == gl::nearest_mipmap_linear || min_filter == gl::linear_mipmap_linear;
}
// True when the minification and magnification results can differ, so lambda
// must be computed (a mipmap filter, or NEAREST against LINEAR).
inline bool texture_needs_lod(const TextureSampler& sampler) noexcept {
    return mipmap_filter(sampler.min_filter) ||
           (sampler.min_filter == gl::nearest) != (sampler.mag_filter == gl::nearest);
}
// c of GL 1.1 3.8.7: 0.5 when the magnification filter is LINEAR and the
// minification filter NEAREST_MIPMAP_NEAREST or NEAREST_MIPMAP_LINEAR, else 0.
float texture_lod_threshold(const TextureSampler& sampler) noexcept;
// q: the last level the minification filter reaches (0 without mipmapping).
uint32_t texture_last_level(const Texture& texture) noexcept;

// lambda from coarse quad differences in level-0 texel units: (u0, v0) is the
// quad's first pixel, (u1, v1) its x neighbour and (u2, v2) its y neighbour.
// lambda = log2(max(dux*dux + dvx*dvx, duy*duy + dvy*dvy)) * 0.5, clamped.
float quad_lod(float u0, float v0, float u1, float v1, float u2, float v2) noexcept;
// lambda of a line segment: texel distance between the endpoint texture
// coordinates (s/q, t/q scaled by the level-0 size) over the window major-axis
// length; the per-segment constant both backends use for lines.
float line_lod(float ax, float ay, float bx, float by, const Vec4& texcoord_a, const Vec4& texcoord_b,
               uint32_t base_width, uint32_t base_height) noexcept;

// Level choice for one fragment. magnify: lambda <= c (level 0, magnification
// filter). Otherwise level_a/level_b and weight follow the minification filter:
// non-mipmap: level 0; *_MIPMAP_NEAREST: d = 0 for lambda <= 0.5, else
// min(ceil(lambda + 0.5) - 1, q); *_MIPMAP_LINEAR: lambda >= q gives q, q and 0,
// else floor(lambda), floor(lambda) + 1 and lambda - floor(lambda). The result
// is a * (1 - weight) + b * weight per channel.
struct TextureLevels final {
    bool magnify = true;
    uint32_t level_a = 0;
    uint32_t level_b = 0;
    float weight = 0;
};
TextureLevels texture_levels(const TextureSampler& sampler, float lambda, uint32_t last_level) noexcept;

struct StencilFace final {
    gl::Enum func = gl::always;
    int32_t ref = 0;
    uint32_t value_mask = 0xffffffffu;
    gl::Enum fail = gl::keep;
    gl::Enum depth_fail = gl::keep;
    gl::Enum depth_pass = gl::keep;
    uint32_t write_mask = 0xffffffffu;
};

// Complete snapshot of the rasterization and per-fragment state for one draw.
// texture is borrowed from the share group for the duration of the call.
struct DrawState final {
    int32_t viewport_x = 0, viewport_y = 0;
    int32_t viewport_width = 0, viewport_height = 0;
    double depth_near = 0, depth_far = 1;

    bool cull_enabled = false;
    gl::Enum cull_mode = gl::back;
    bool front_ccw = true;
    bool two_sided_color = false;

    float point_size = 1;
    float line_width = 1;
    bool line_stipple = false;
    uint32_t line_stipple_factor = 1;
    uint16_t line_stipple_pattern = 0xffff;
    // GL 1.1 3.4.2: the stipple counter s advances by one per rasterized line
    // sample. true restarts it at this draw's first segment (every independent
    // GL_LINES segment, the first segment after Begin of a strip or loop); false
    // continues from the previous lines draw of the same Begin.
    bool line_stipple_reset = true;
    // Row y & 31 of the window-anchored pattern; bit 31 is x & 31 == 0.
    bool polygon_stipple = false;
    uint32_t polygon_stipple_rows[32]{};

    bool offset_enabled = false;     // for the primitive kind being drawn
    float offset_factor = 0, offset_units = 0;

    bool scissor_enabled = false;
    int32_t scissor_x = 0, scissor_y = 0, scissor_width = 0, scissor_height = 0;

    const Texture* texture = nullptr;  // null when texturing is disabled/incomplete
    gl::Enum texture_env_mode = gl::modulate;
    Vec4 texture_env_color{0, 0, 0, 0};

    bool fog_enabled = false;
    Vec4 fog_color{0, 0, 0, 0};

    bool alpha_test = false;
    gl::Enum alpha_func = gl::always;
    float alpha_ref = 0;

    bool stencil_test = false;
    StencilFace stencil{};

    bool depth_test = false;
    gl::Enum depth_func = gl::less;
    bool depth_write = true;

    bool blend = false;
    gl::Enum blend_src = gl::one, blend_dst = gl::zero;
    bool logic_op = false;
    gl::Enum logic_op_mode = gl::logic_copy;
    bool color_write[4] = {true, true, true, true};
    bool dither = true;

    gl::Enum draw_buffer = gl::back;   // front, back, front_and_back or none
};

// Fragment rectangle for DrawPixels, Bitmap and CopyPixels (GL 1.1 3.6.4). (x, y)
// is the window position of the image origin (the raster position; for a
// bitmap floor(raster - origin) with zoom 1). Source pixel (column, row) yields
// a fragment at every framebuffer pixel whose centre (cx, cy) gives
// column = floor((cx - x) * (1 / zoom_x)) and row = floor((cy - y) * (1 / zoom_y))
// inside the image, with exactly these float operations in every backend (so a
// zoom of 0 produces nothing and a negative zoom mirrors). Exactly one of rgba,
// depth or stencil selects the data, except a bitmap: coverage plus color.
// rgba fragments take depth z; depth fragments take color; both are textured
// with texcoord (no derivatives: magnification) and fogged with fog, then run
// every per-fragment operation. Stencil fragments (GL 1.1 4.3.1) pass only the
// pixel ownership and scissor tests and write the index under the stencil
// write mask. Arrays are row-major from the bottom row, width * height entries,
// borrowed for the call; colours are already 8-bit quantized by the frontend.
// A large image arrives in bands: the arrays then hold image rows first_row ..
// first_row + height - 1, the rule above still uses the whole image's origin and
// only those rows produce fragments.
struct PixelFragments final {
    float x = 0, y = 0;
    uint32_t width = 0, height = 0;
    uint32_t first_row = 0;
    float zoom_x = 1, zoom_y = 1;
    float z = 0;                       // window depth in [0, 1]
    Vec4 color{};                      // constant color for coverage/depth/stencil writes
    Vec4 texcoord{};
    float fog = 1;
    const float* rgba = nullptr;       // 4 floats per pixel in [0, 1]
    const float* depth = nullptr;      // window depth per pixel
    const uint8_t* stencil = nullptr;  // stencil index per pixel
    const uint8_t* coverage = nullptr; // nonzero = fragment generated
};

// Window coordinates of a clip-space vertex under state's viewport, in the float
// operation order of gl_raster's viewport transform. false for w == 0, NaN or a
// non-finite result.
bool window_xy(const RasterVertex& vertex, const DrawState& state, float& x, float& y) noexcept;
// Number of DDA steps of a window-space segment: it rasterizes steps + 1 samples,
// sample k at t = k / steps (gl_raster raster_line and the GFX11 draw model).
int32_t line_steps(float ax, float ay, float bx, float by) noexcept;

struct ClearRequest final {
    gl::Bitfield mask = 0;             // color, depth and stencil buffer bits
    Vec4 color{};
    double depth = 1;
    int32_t stencil = 0;
    bool color_write[4] = {true, true, true, true};
    bool depth_write = true;
    uint32_t stencil_write = 0xffffffffu;
    bool scissor_enabled = false;
    int32_t scissor_x = 0, scissor_y = 0, scissor_width = 0, scissor_height = 0;
    gl::Enum draw_buffer = gl::back;
};

enum class ReadSource : uint8_t { color, depth, stencil };

// glCopyPixels with only the linear transfer operations (no pixel maps): the
// source rectangle of the read buffer, transformed per channel and written as
// the fragments of destination (its data pointers are unused). COLOR applies
// c * scale + bias clamped to [0, 1] and 8-bit quantized per RGBA channel; DEPTH
// scale[0] and bias[0], clamped; STENCIL shifts left by index_shift (right when
// negative), adds index_offset and keeps the low 8 bits. GL leaves source pixels
// outside the framebuffer undefined.
struct PixelCopy final {
    ReadSource source = ReadSource::color;
    gl::Enum buffer = gl::back;
    int32_t x = 0, y = 0;
    uint32_t width = 0, height = 0;
    PixelFragments destination{};
    float scale[4] = {1, 1, 1, 1};
    float bias[4] = {0, 0, 0, 0};
    int32_t index_shift = 0;
    int32_t index_offset = 0;
};

// Drawable framebuffer format exposed by a pixel format.
struct FramebufferFormat final {
    bool double_buffered = true;
    uint32_t depth_bits = 24;
    uint32_t stencil_bits = 8;
};

// Rasterization backend. Methods are called on the thread that owns the current
// context, never concurrently. A backend that fails sets its own sticky failure
// and reports it through healthy(); the frontend then raises GL_OUT_OF_MEMORY.
class Backend {
public:
    virtual ~Backend() = default;
    // (Re)size every buffer; contents become undefined. Zero-sized is allowed.
    virtual bool resize(uint32_t width, uint32_t height) noexcept = 0;
    virtual uint32_t width() const noexcept = 0;
    virtual uint32_t height() const noexcept = 0;
    virtual FramebufferFormat format() const noexcept = 0;
    virtual void clear(const ClearRequest& request) noexcept = 0;
    // count is a multiple of 1, 2 or 3 for points, lines or triangles.
    virtual void draw(const DrawState& state, PrimitiveKind kind,
                      const RasterVertex* vertices, uint32_t count) noexcept = 0;
    virtual void draw_pixels(const DrawState& state, const PixelFragments& pixels) noexcept = 0;
    // CopyPixels without a frontend round trip. false: not handled (the frontend
    // then reads, transforms and calls draw_pixels); nothing was drawn.
    virtual bool copy_pixels(const DrawState& state, const PixelCopy& copy) noexcept {
        (void)state;
        (void)copy;
        return false;
    }
    // Reads a clipped rectangle from the buffer named by gl::front/gl::back.
    // color: 4 floats per pixel; depth: 1 float; stencil: 1 float. Pixels
    // outside the framebuffer are left untouched. Rows bottom to top.
    virtual void read(ReadSource source, gl::Enum buffer, int32_t x, int32_t y,
                      uint32_t width, uint32_t height, float* output) noexcept = 0;
    // GL finish semantics: every earlier command has completed.
    virtual void finish() noexcept = 0;
    virtual void flush() noexcept = 0;
    virtual bool healthy() const noexcept = 0;
};

} // namespace rx7900::opengl

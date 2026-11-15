/*
Module: rx7900.opengl.gl_raster
Purpose: Scanline rasterization with the full OpenGL 1.1 per-fragment pipeline.
Created: 2026-10-23
Architecture: See gl_raster.hpp. Implements the OpenGL 1.1 specification chapters
3 (Rasterization) and 4 (Per-Fragment Operations); no third-party code. All
color buffers are BGRA8 (0xAARRGGBB little-endian), matching the Radeon scanout.
*/
#include "gl_raster.hpp"

#include <cmath>

namespace rx7900::opengl {
namespace {

float clamp01(float value) noexcept {
    return value < 0 ? 0 : (value > 1 ? 1 : value);
}

uint8_t to_byte(float value) noexcept {
    const float scaled = clamp01(value) * 255.0f + 0.5f;
    return static_cast<uint8_t>(scaled);
}

uint32_t pack_bgra(const Vec4& color) noexcept {
    return (static_cast<uint32_t>(to_byte(color.w)) << 24) |
           (static_cast<uint32_t>(to_byte(color.x)) << 16) |
           (static_cast<uint32_t>(to_byte(color.y)) << 8) |
           static_cast<uint32_t>(to_byte(color.z));
}

Vec4 unpack_bgra(uint32_t value) noexcept {
    return {static_cast<float>((value >> 16) & 0xff) / 255.0f,
            static_cast<float>((value >> 8) & 0xff) / 255.0f,
            static_cast<float>(value & 0xff) / 255.0f,
            static_cast<float>((value >> 24) & 0xff) / 255.0f};
}

bool compare(gl::Enum func, float reference, float value) noexcept {
    switch (func) {
    case gl::never: return false;
    case gl::less: return reference < value;
    case gl::equal: return reference == value;
    case gl::lequal: return reference <= value;
    case gl::greater: return reference > value;
    case gl::notequal: return reference != value;
    case gl::gequal: return reference >= value;
    case gl::always: return true;
    default: return true;
    }
}

bool compare_int(gl::Enum func, uint32_t reference, uint32_t value) noexcept {
    switch (func) {
    case gl::never: return false;
    case gl::less: return reference < value;
    case gl::equal: return reference == value;
    case gl::lequal: return reference <= value;
    case gl::greater: return reference > value;
    case gl::notequal: return reference != value;
    case gl::gequal: return reference >= value;
    case gl::always: return true;
    default: return true;
    }
}

uint8_t stencil_op(gl::Enum op, uint8_t current, int32_t ref, uint8_t mask) noexcept {
    switch (op) {
    case gl::keep: return current;
    case gl::zero: return 0;
    case gl::replace: return static_cast<uint8_t>(static_cast<uint32_t>(ref) & mask);
    case gl::incr: return current < 255 ? static_cast<uint8_t>(current + 1) : 255;
    case gl::decr: return current > 0 ? static_cast<uint8_t>(current - 1) : 0;
    case gl::invert: return static_cast<uint8_t>(~current);
    default: return current;
    }
}

// Per-channel blend factor; channel selects the component for *_color factors.
float factor_channel(gl::Enum factor, int channel, const Vec4& src, const Vec4& dst) noexcept {
    const float* s = &src.x;
    const float* d = &dst.x;

    switch (factor) {
    case gl::zero: return 0;
    case gl::one: return 1;
    case gl::src_color: return s[channel];
    case gl::one_minus_src_color: return 1 - s[channel];
    case gl::dst_color: return d[channel];
    case gl::one_minus_dst_color: return 1 - d[channel];
    case gl::src_alpha: return src.w;
    case gl::one_minus_src_alpha: return 1 - src.w;
    case gl::dst_alpha: return dst.w;
    case gl::one_minus_dst_alpha: return 1 - dst.w;
    case gl::src_alpha_saturate: {
        if (channel == 3) {
            return 1;
        }
        const float f = src.w < (1 - dst.w) ? src.w : (1 - dst.w);
        return f;
    }
    default: return 0;
    }
}

uint32_t logic_apply(gl::Enum mode, uint32_t s, uint32_t d) noexcept {
    switch (mode) {
    case gl::logic_clear: return 0;
    case gl::logic_and: return s & d;
    case gl::logic_and_reverse: return s & ~d;
    case gl::logic_copy: return s;
    case gl::logic_and_inverted: return ~s & d;
    case gl::logic_noop: return d;
    case gl::logic_xor: return s ^ d;
    case gl::logic_or: return s | d;
    case gl::logic_nor: return ~(s | d);
    case gl::logic_equiv: return ~(s ^ d);
    case gl::invert: return ~d;
    case gl::logic_or_reverse: return s | ~d;
    case gl::logic_copy_inverted: return ~s;
    case gl::logic_or_inverted: return ~s | d;
    case gl::logic_nand: return ~(s & d);
    case gl::logic_set: return 0xffffffffu;
    default: return s;
    }
}

float wrap_coord(float value, gl::Enum mode, uint32_t size) noexcept {
    // Returns a texel-space coordinate (0 .. size) with wrapping applied.
    const float texels = value * static_cast<float>(size);

    if (mode == gl::repeat) {
        float wrapped = std::fmod(texels, static_cast<float>(size));
        if (wrapped < 0) {
            wrapped += static_cast<float>(size);
        }
        return wrapped;
    }

    // clamp / clamp_to_edge
    if (texels < 0) {
        return 0;
    }
    if (texels > static_cast<float>(size)) {
        return static_cast<float>(size);
    }
    return texels;
}

Vec4 fetch_texel(const TextureLevel& level, int32_t x, int32_t y, gl::Enum wrap_s,
                 gl::Enum wrap_t) noexcept {
    const int32_t w = static_cast<int32_t>(level.width);
    const int32_t h = static_cast<int32_t>(level.height);

    auto wrap = [](int32_t coord, int32_t size, gl::Enum mode) -> int32_t {
        if (mode == gl::repeat) {
            int32_t m = coord % size;
            return m < 0 ? m + size : m;
        }
        return coord < 0 ? 0 : (coord >= size ? size - 1 : coord);
    };

    const int32_t tx = wrap(x, w, wrap_s);
    const int32_t ty = wrap(y, h, wrap_t);
    const uint8_t* texel = level.texels + (static_cast<size_t>(ty) * w + tx) * 4u;
    return {texel[0] / 255.0f, texel[1] / 255.0f, texel[2] / 255.0f, texel[3] / 255.0f};
}

// One level, GL nearest or bilinear (half-texel centres) filtering at (s, t).
Vec4 sample_level(const TextureLevel& level, bool nearest, const TextureSampler& sampler, float s,
                  float t) noexcept {
    if (level.texels == nullptr || level.width == 0 || level.height == 0) {
        return {1, 1, 1, 1};
    }

    if (nearest) {
        const float fx = wrap_coord(s, sampler.wrap_s, level.width);
        const float fy = wrap_coord(t, sampler.wrap_t, level.height);
        int32_t ix = static_cast<int32_t>(std::floor(fx));
        int32_t iy = static_cast<int32_t>(std::floor(fy));
        if (ix >= static_cast<int32_t>(level.width)) { ix = static_cast<int32_t>(level.width) - 1; }
        if (iy >= static_cast<int32_t>(level.height)) { iy = static_cast<int32_t>(level.height) - 1; }
        return fetch_texel(level, ix, iy, sampler.wrap_s, sampler.wrap_t);
    }

    const float fx = wrap_coord(s, sampler.wrap_s, level.width) - 0.5f;
    const float fy = wrap_coord(t, sampler.wrap_t, level.height) - 0.5f;
    const int32_t x0 = static_cast<int32_t>(std::floor(fx));
    const int32_t y0 = static_cast<int32_t>(std::floor(fy));
    const float ax = fx - static_cast<float>(x0);
    const float ay = fy - static_cast<float>(y0);
    const Vec4 c00 = fetch_texel(level, x0, y0, sampler.wrap_s, sampler.wrap_t);
    const Vec4 c10 = fetch_texel(level, x0 + 1, y0, sampler.wrap_s, sampler.wrap_t);
    const Vec4 c01 = fetch_texel(level, x0, y0 + 1, sampler.wrap_s, sampler.wrap_t);
    const Vec4 c11 = fetch_texel(level, x0 + 1, y0 + 1, sampler.wrap_s, sampler.wrap_t);
    auto mix = [](const Vec4& a, const Vec4& b, float f) -> Vec4 {
        return {a.x + (b.x - a.x) * f, a.y + (b.y - a.y) * f, a.z + (b.z - a.z) * f,
                a.w + (b.w - a.w) * f};
    };
    return mix(mix(c00, c10, ax), mix(c01, c11, ax), ay);
}

} // namespace

uint64_t raster_storage_bytes(uint32_t width, uint32_t height, bool double_buffered) noexcept {
    if (width > raster_max_dimension || height > raster_max_dimension) {
        return 0;
    }

    const uint64_t pixels = static_cast<uint64_t>(width) * height;
    const uint64_t color = pixels * 4u * (double_buffered ? 2u : 1u);
    const uint64_t depth = pixels * 4u;
    const uint64_t stencil = pixels;
    return color + depth + stencil;
}

SoftwareBackend::SoftwareBackend(uint8_t* storage, uint64_t storage_bytes,
                                 FramebufferFormat format) noexcept
    : storage_(storage), storage_bytes_(storage_bytes), format_(format) {}

bool SoftwareBackend::resize(uint32_t width, uint32_t height) noexcept {
    front_ = back_ = depth_ = nullptr;
    stencil_ = nullptr;
    width_ = height_ = 0;

    if (width == 0 || height == 0) {
        return true;
    }

    const uint64_t needed = raster_storage_bytes(width, height, format_.double_buffered);

    if (needed == 0 || needed > storage_bytes_ || storage_ == nullptr) {
        healthy_ = false;
        return false;
    }

    const uint64_t pixels = static_cast<uint64_t>(width) * height;
    uint8_t* cursor = storage_;
    front_ = reinterpret_cast<uint32_t*>(cursor);
    cursor += pixels * 4u;

    if (format_.double_buffered) {
        back_ = reinterpret_cast<uint32_t*>(cursor);
        cursor += pixels * 4u;
    } else {
        back_ = front_;
    }

    depth_ = reinterpret_cast<uint32_t*>(cursor);
    cursor += pixels * 4u;
    stencil_ = cursor;
    width_ = width;
    height_ = height;
    return true;
}

const uint32_t* SoftwareBackend::color_buffer(gl::Enum buffer) const noexcept {
    if (buffer == gl::front || buffer == gl::front_left) {
        return front_;
    }
    return back_;
}

void SoftwareBackend::swap() noexcept {
    if (format_.double_buffered && front_ != nullptr && back_ != nullptr) {
        uint32_t* temp = front_;
        front_ = back_;
        back_ = temp;
    }
}

bool SoftwareBackend::scissor_ok(const DrawState& state, int32_t x, int32_t y) const noexcept {
    if (!state.scissor_enabled) {
        return true;
    }
    return x >= state.scissor_x && x < state.scissor_x + state.scissor_width &&
           y >= state.scissor_y && y < state.scissor_y + state.scissor_height;
}

void SoftwareBackend::clear(const ClearRequest& request) noexcept {
    if (width_ == 0 || height_ == 0) {
        return;
    }

    const bool clear_color = (request.mask & gl::color_buffer_bit) != 0;
    const bool clear_depth = (request.mask & gl::depth_buffer_bit) != 0;
    const bool clear_stencil = (request.mask & gl::stencil_buffer_bit) != 0;
    const uint32_t packed = pack_bgra(request.color);
    const uint32_t depth_value = static_cast<uint32_t>(
        clamp01(static_cast<float>(request.depth)) * 4294967295.0);
    const uint8_t stencil_value = static_cast<uint8_t>(request.stencil & 0xff);

    const int32_t x0 = request.scissor_enabled ? (request.scissor_x < 0 ? 0 : request.scissor_x) : 0;
    const int32_t y0 = request.scissor_enabled ? (request.scissor_y < 0 ? 0 : request.scissor_y) : 0;
    const int32_t x1 = request.scissor_enabled
        ? (request.scissor_x + request.scissor_width > static_cast<int32_t>(width_)
               ? static_cast<int32_t>(width_)
               : request.scissor_x + request.scissor_width)
        : static_cast<int32_t>(width_);
    const int32_t y1 = request.scissor_enabled
        ? (request.scissor_y + request.scissor_height > static_cast<int32_t>(height_)
               ? static_cast<int32_t>(height_)
               : request.scissor_y + request.scissor_height)
        : static_cast<int32_t>(height_);

    uint32_t* color_front = (request.draw_buffer == gl::front || request.draw_buffer == gl::front_and_back)
                                ? front_
                                : nullptr;
    uint32_t* color_back = (request.draw_buffer == gl::back || request.draw_buffer == gl::front_and_back)
                               ? back_
                               : nullptr;
    const bool full_mask = request.color_write[0] && request.color_write[1] &&
                           request.color_write[2] && request.color_write[3];

    for (int32_t y = y0; y < y1; ++y) {
        for (int32_t x = x0; x < x1; ++x) {
            const size_t index = static_cast<size_t>(y) * width_ + x;

            if (clear_color) {
                if (full_mask) {
                    if (color_front != nullptr) {
                        color_front[index] = packed;
                    }
                    if (color_back != nullptr) {
                        color_back[index] = packed;
                    }
                } else {
                    uint32_t* const targets[2] = {color_front, color_back};
                    for (uint32_t* target : targets) {
                        if (target == nullptr) {
                            continue;
                        }
                        const Vec4 existing = unpack_bgra(target[index]);
                        Vec4 result = request.color;
                        if (!request.color_write[0]) { result.x = existing.x; }
                        if (!request.color_write[1]) { result.y = existing.y; }
                        if (!request.color_write[2]) { result.z = existing.z; }
                        if (!request.color_write[3]) { result.w = existing.w; }
                        target[index] = pack_bgra(result);
                    }
                }
            }

            if (clear_depth && request.depth_write) {
                depth_[index] = depth_value;
            }

            if (clear_stencil) {
                stencil_[index] = static_cast<uint8_t>(
                    (stencil_[index] & ~request.stencil_write) |
                    (stencil_value & request.stencil_write));
            }
        }
    }
}

bool SoftwareBackend::to_window(const RasterVertex& vertex, const DrawState& state,
                                Window& output) const noexcept {
    if (!window_xy(vertex, state, output.x, output.y)) {
        return false;      // w == 0, NaN or a non-finite position
    }

    const float inv_w = 1.0f / vertex.clip.w;
    const float ndc_z = vertex.clip.z * inv_w;
    const double near_d = state.depth_near;
    const double far_d = state.depth_far;
    output.z = static_cast<float>(near_d + (ndc_z + 1.0) * 0.5 * (far_d - near_d));
    output.color = vertex.color[0];
    output.texcoord = vertex.texcoord;
    output.fog = vertex.fog;
    output.inv_w = inv_w;

    return std::isfinite(output.z);
}

Vec4 SoftwareBackend::sample_texture(const Texture& texture, const Vec4& coord, float lambda) const noexcept {
    const float q = (coord.w != 0) ? coord.w : 1.0f;
    const float s = coord.x / q;
    const float t = coord.y / q;
    const TextureSampler& sampler = texture.sampler;

    if (!texture_needs_lod(sampler)) {
        // Minification and magnification use the same single-level filter.
        return sample_level(texture.levels[0], sampler.mag_filter == gl::nearest, sampler, s, t);
    }

    const TextureLevels levels = texture_levels(sampler, lambda, texture_last_level(texture));

    if (levels.magnify) {
        return sample_level(texture.levels[0], sampler.mag_filter == gl::nearest, sampler, s, t);
    }

    const bool nearest = sampler.min_filter == gl::nearest || sampler.min_filter == gl::nearest_mipmap_nearest ||
                         sampler.min_filter == gl::nearest_mipmap_linear;
    const Vec4 a = sample_level(texture.levels[levels.level_a], nearest, sampler, s, t);

    if (sampler.min_filter != gl::nearest_mipmap_linear && sampler.min_filter != gl::linear_mipmap_linear) {
        return a;
    }

    const Vec4 b = sample_level(texture.levels[levels.level_b], nearest, sampler, s, t);
    const float w = levels.weight;
    return {a.x * (1.0f - w) + b.x * w, a.y * (1.0f - w) + b.y * w, a.z * (1.0f - w) + b.z * w,
            a.w * (1.0f - w) + b.w * w};
}

void SoftwareBackend::shade_fragment(const DrawState& state, uint32_t x, uint32_t y, float z,
                                     const Vec4& color_in, const Vec4& texcoord, float fog,
                                     float lambda) noexcept {
    const size_t index = static_cast<size_t>(y) * width_ + x;
    Vec4 color = color_in;

    // Texturing.
    if (state.texture != nullptr) {
        const Vec4 sample = sample_texture(*state.texture, texcoord, lambda);
        const gl::Enum base = state.texture->levels[0].base_format;
        const bool has_alpha = base == gl::rgba || base == gl::alpha || base == gl::luminance_alpha;
        switch (state.texture_env_mode) {
        case gl::replace:
            if (base == gl::alpha) {
                color.w = sample.w;
            } else if (base == gl::luminance || base == gl::rgb) {
                color.x = sample.x; color.y = sample.y; color.z = sample.z;
            } else {
                color = sample;
            }
            break;
        case gl::decal:
            if (base == gl::rgba) {
                color.x = color.x * (1 - sample.w) + sample.x * sample.w;
                color.y = color.y * (1 - sample.w) + sample.y * sample.w;
                color.z = color.z * (1 - sample.w) + sample.z * sample.w;
            } else {
                color.x = sample.x; color.y = sample.y; color.z = sample.z;
            }
            break;
        case gl::blend:
            color.x = color.x * (1 - sample.x) + state.texture_env_color.x * sample.x;
            color.y = color.y * (1 - sample.y) + state.texture_env_color.y * sample.y;
            color.z = color.z * (1 - sample.z) + state.texture_env_color.z * sample.z;
            if (has_alpha) { color.w = color.w * sample.w; }
            break;
        case gl::modulate:
        default:
            color.x *= sample.x; color.y *= sample.y; color.z *= sample.z;
            if (has_alpha) { color.w *= sample.w; }
            break;
        }
    }

    // Fog.
    if (state.fog_enabled) {
        const float f = clamp01(fog);
        color.x = color.x * f + state.fog_color.x * (1 - f);
        color.y = color.y * f + state.fog_color.y * (1 - f);
        color.z = color.z * f + state.fog_color.z * (1 - f);
    }

    // Alpha test.
    if (state.alpha_test && !compare(state.alpha_func, color.w, state.alpha_ref)) {
        return;
    }

    // Scissor already applied by the span loops; stencil and depth next.
    uint8_t stencil_value = stencil_[index];
    bool stencil_pass = true;

    if (state.stencil_test) {
        const uint8_t masked_ref = static_cast<uint8_t>(
            static_cast<uint32_t>(state.stencil.ref) & state.stencil.value_mask);
        const uint8_t masked_value = static_cast<uint8_t>(stencil_value & state.stencil.value_mask);
        stencil_pass = compare_int(state.stencil.func, masked_ref, masked_value);
    }

    const float stored_depth = static_cast<float>(depth_[index]) / 4294967295.0f;
    bool depth_pass = true;
    if (state.depth_test) {
        depth_pass = compare(state.depth_func, z, stored_depth);
    }

    if (state.stencil_test) {
        gl::Enum op;
        if (!stencil_pass) {
            op = state.stencil.fail;
        } else if (!depth_pass) {
            op = state.stencil.depth_fail;
        } else {
            op = state.stencil.depth_pass;
        }
        const uint8_t updated = stencil_op(op, stencil_value, state.stencil.ref,
                                           static_cast<uint8_t>(state.stencil.value_mask));
        stencil_[index] = static_cast<uint8_t>((stencil_value & ~state.stencil.write_mask) |
                                               (updated & state.stencil.write_mask));
    }

    if (!stencil_pass || !depth_pass) {
        return;
    }

    if (state.depth_test && state.depth_write) {
        depth_[index] = static_cast<uint32_t>(clamp01(z) * 4294967295.0);
    }

    // GL_FRONT_AND_BACK writes both buffers, each blended, logic-op'ed and
    // masked against its own destination; the tests above ran once.
    uint32_t* front = state.draw_buffer == gl::front || state.draw_buffer == gl::front_and_back ? front_ : nullptr;
    uint32_t* back = state.draw_buffer == gl::back || state.draw_buffer == gl::front_and_back ? back_ : nullptr;

    if (front != nullptr) {
        write_color(state, front, index, color);
    }
    if (back != nullptr && back != front) {
        write_color(state, back, index, color);
    }
}

void SoftwareBackend::write_color(const DrawState& state, uint32_t* target, size_t index,
                                  const Vec4& color) noexcept {
    const Vec4 destination = unpack_bgra(target[index]);
    Vec4 source = color;

    if (state.blend) {
        Vec4 result{};
        for (int channel = 0; channel < 4; ++channel) {
            const float sf = factor_channel(state.blend_src, channel, source, destination);
            const float df = factor_channel(state.blend_dst, channel, source, destination);
            (&result.x)[channel] = (&source.x)[channel] * sf + (&destination.x)[channel] * df;
        }
        source = result;
    }

    uint32_t packed = pack_bgra(source);

    if (state.logic_op) {
        packed = logic_apply(state.logic_op_mode, packed, target[index]);
    }

    // Color mask (per BGRA channel in the packed value).
    uint32_t mask = 0;
    if (state.color_write[0]) { mask |= 0x00ff0000u; }   // red
    if (state.color_write[1]) { mask |= 0x0000ff00u; }   // green
    if (state.color_write[2]) { mask |= 0x000000ffu; }   // blue
    if (state.color_write[3]) { mask |= 0xff000000u; }   // alpha
    target[index] = (packed & mask) | (target[index] & ~mask);
}

void SoftwareBackend::raster_triangle(const DrawState& state, const Window& a, const Window& b,
                                      const Window& c, float offset) noexcept {
    const float area = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);

    if (area == 0 || !std::isfinite(area)) {
        return;
    }

    const float inv_area = 1.0f / area;

    float min_xf = a.x < b.x ? (a.x < c.x ? a.x : c.x) : (b.x < c.x ? b.x : c.x);
    float max_xf = a.x > b.x ? (a.x > c.x ? a.x : c.x) : (b.x > c.x ? b.x : c.x);
    float min_yf = a.y < b.y ? (a.y < c.y ? a.y : c.y) : (b.y < c.y ? b.y : c.y);
    float max_yf = a.y > b.y ? (a.y > c.y ? a.y : c.y) : (b.y > c.y ? b.y : c.y);

    int32_t min_x = static_cast<int32_t>(std::floor(min_xf));
    int32_t max_x = static_cast<int32_t>(std::ceil(max_xf));
    int32_t min_y = static_cast<int32_t>(std::floor(min_yf));
    int32_t max_y = static_cast<int32_t>(std::ceil(max_yf));

    if (min_x < 0) { min_x = 0; }
    if (min_y < 0) { min_y = 0; }
    if (max_x > static_cast<int32_t>(width_)) { max_x = static_cast<int32_t>(width_); }
    if (max_y > static_cast<int32_t>(height_)) { max_y = static_cast<int32_t>(height_); }

    // Perspective-correct barycentric weights of a pixel centre (unclamped, so a
    // quad neighbour outside the triangle extrapolates like a GPU helper lane).
    struct Weights final {
        float w0, w1, w2, inv_iw;
    };
    auto weights = [&](int32_t x, int32_t y) -> Weights {
        const float px = static_cast<float>(x) + 0.5f;
        const float py = static_cast<float>(y) + 0.5f;
        Weights w{};
        w.w0 = ((b.x - px) * (c.y - py) - (b.y - py) * (c.x - px)) * inv_area;
        w.w1 = ((c.x - px) * (a.y - py) - (c.y - py) * (a.x - px)) * inv_area;
        w.w2 = 1.0f - w.w0 - w.w1;
        const float iw = w.w0 * a.inv_w + w.w1 * b.inv_w + w.w2 * c.inv_w;
        w.inv_iw = iw != 0 ? 1.0f / iw : 0;
        return w;
    };
    auto attr = [&](const Weights& w, float va, float vb, float vc) -> float {
        return (w.w0 * a.inv_w * va + w.w1 * b.inv_w * vb + w.w2 * c.inv_w * vc) * w.inv_iw;
    };

    // Level of detail from the 2x2 quad's coarse differences in level-0 texels.
    const bool lod = state.texture != nullptr && texture_needs_lod(state.texture->sampler);
    const float texture_width = lod ? static_cast<float>(state.texture->levels[0].width) : 0.0f;
    const float texture_height = lod ? static_cast<float>(state.texture->levels[0].height) : 0.0f;
    auto texel_at = [&](int32_t x, int32_t y, float& u, float& v) {
        const Weights w = weights(x, y);
        const float qa = attr(w, a.texcoord.w, b.texcoord.w, c.texcoord.w);
        const float q = qa != 0 ? qa : 1.0f;
        u = attr(w, a.texcoord.x, b.texcoord.x, c.texcoord.x) / q * texture_width;
        v = attr(w, a.texcoord.y, b.texcoord.y, c.texcoord.y) / q * texture_height;
    };

    for (int32_t y = min_y; y < max_y; ++y) {
        for (int32_t x = min_x; x < max_x; ++x) {
            const Weights w = weights(x, y);

            if (w.w0 < 0 || w.w1 < 0 || w.w2 < 0) {
                continue;
            }

            if (!scissor_ok(state, x, y)) {
                continue;
            }

            if (state.polygon_stipple &&
                ((state.polygon_stipple_rows[y & 31] >> (31 - (x & 31))) & 1u) == 0) {
                continue;
            }

            const float z = w.w0 * a.z + w.w1 * b.z + w.w2 * c.z + offset;
            Vec4 color{attr(w, a.color.x, b.color.x, c.color.x), attr(w, a.color.y, b.color.y, c.color.y),
                       attr(w, a.color.z, b.color.z, c.color.z), attr(w, a.color.w, b.color.w, c.color.w)};
            Vec4 texcoord{attr(w, a.texcoord.x, b.texcoord.x, c.texcoord.x),
                          attr(w, a.texcoord.y, b.texcoord.y, c.texcoord.y),
                          attr(w, a.texcoord.z, b.texcoord.z, c.texcoord.z),
                          attr(w, a.texcoord.w, b.texcoord.w, c.texcoord.w)};
            const float fog = attr(w, a.fog, b.fog, c.fog);
            float lambda = -texture_lod_limit;

            if (lod) {
                const int32_t qx = x & ~1;
                const int32_t qy = y & ~1;
                float u0 = 0, v0 = 0, u1 = 0, v1 = 0, u2 = 0, v2 = 0;
                texel_at(qx, qy, u0, v0);
                texel_at(qx + 1, qy, u1, v1);
                texel_at(qx, qy + 1, u2, v2);
                lambda = quad_lod(u0, v0, u1, v1, u2, v2);
            }

            shade_fragment(state, static_cast<uint32_t>(x), static_cast<uint32_t>(y), z, color,
                           texcoord, fog, lambda);
        }
    }
}

void SoftwareBackend::raster_line(const DrawState& state, const Window& a, const Window& b) noexcept {
    const float dx = b.x - a.x;
    const float dy = b.y - a.y;
    const int32_t steps = line_steps(a.x, a.y, b.x, b.y);

    if (steps <= 0) {
        return;
    }

    const bool lod = state.texture != nullptr && texture_needs_lod(state.texture->sampler);
    const float lambda = lod ? line_lod(a.x, a.y, b.x, b.y, a.texcoord, b.texcoord,
                                        state.texture->levels[0].width, state.texture->levels[0].height)
                             : -texture_lod_limit;

    for (int32_t step = 0; step <= steps; ++step) {
        const float t = static_cast<float>(step) / static_cast<float>(steps);

        if (state.line_stipple) {
            const uint32_t bit = (stipple_counter_ / state.line_stipple_factor) & 15u;
            ++stipple_counter_;
            if (((state.line_stipple_pattern >> bit) & 1u) == 0) {
                continue;
            }
        }

        const int32_t x = static_cast<int32_t>(std::floor(a.x + dx * t));
        const int32_t y = static_cast<int32_t>(std::floor(a.y + dy * t));

        if (x < 0 || y < 0 || x >= static_cast<int32_t>(width_) || y >= static_cast<int32_t>(height_)) {
            continue;
        }

        if (!scissor_ok(state, x, y)) {
            continue;
        }

        const float z = a.z + (b.z - a.z) * t;
        const float iw = a.inv_w + (b.inv_w - a.inv_w) * t;
        const float inv_iw = iw != 0 ? 1.0f / iw : 0;
        auto attr = [&](float va, float vb) -> float {
            return (a.inv_w * va * (1 - t) + b.inv_w * vb * t) * inv_iw;
        };
        Vec4 color{attr(a.color.x, b.color.x), attr(a.color.y, b.color.y),
                   attr(a.color.z, b.color.z), attr(a.color.w, b.color.w)};
        Vec4 texcoord{attr(a.texcoord.x, b.texcoord.x), attr(a.texcoord.y, b.texcoord.y),
                      attr(a.texcoord.z, b.texcoord.z), attr(a.texcoord.w, b.texcoord.w)};
        const float fog = a.fog + (b.fog - a.fog) * t;
        shade_fragment(state, static_cast<uint32_t>(x), static_cast<uint32_t>(y), z, color,
                       texcoord, fog, lambda);
    }
}

void SoftwareBackend::raster_point(const DrawState& state, const Window& point) noexcept {
    const int32_t half = static_cast<int32_t>(state.point_size * 0.5f);
    const int32_t cx = static_cast<int32_t>(std::floor(point.x));
    const int32_t cy = static_cast<int32_t>(std::floor(point.y));

    for (int32_t y = cy - half; y <= cy + half; ++y) {
        for (int32_t x = cx - half; x <= cx + half; ++x) {
            if (x < 0 || y < 0 || x >= static_cast<int32_t>(width_) ||
                y >= static_cast<int32_t>(height_)) {
                continue;
            }
            if (!scissor_ok(state, x, y)) {
                continue;
            }
            // Every fragment of a point shares its attributes: no derivatives.
            shade_fragment(state, static_cast<uint32_t>(x), static_cast<uint32_t>(y), point.z,
                           point.color, point.texcoord, point.fog, -texture_lod_limit);
        }
    }
}

void SoftwareBackend::draw(const DrawState& state, PrimitiveKind kind,
                           const RasterVertex* vertices, uint32_t count) noexcept {
    if (width_ == 0 || height_ == 0 || vertices == nullptr) {
        return;
    }

    if (kind == PrimitiveKind::triangles) {
        const float offset = state.offset_enabled ? state.offset_units * (1.0f / 65536.0f) : 0;

        for (uint32_t base = 0; base + 3 <= count; base += 3) {
            Window w[3];
            bool ok = true;
            for (uint32_t i = 0; i < 3; ++i) {
                if (!to_window(vertices[base + i], state, w[i])) {
                    ok = false;
                }
            }
            if (!ok) {
                continue;
            }

            const float signed_area = (w[1].x - w[0].x) * (w[2].y - w[0].y) -
                                      (w[1].y - w[0].y) * (w[2].x - w[0].x);
            // to_window keeps GL's y-up window coordinates, so a counter-clockwise
            // polygon has positive signed area (glFrontFace: CCW is front by default).
            const bool front_face = state.front_ccw ? (signed_area > 0) : (signed_area < 0);

            if (state.cull_enabled) {
                const bool cull_front = state.cull_mode == gl::front ||
                                        state.cull_mode == gl::front_and_back;
                const bool cull_back = state.cull_mode == gl::back ||
                                       state.cull_mode == gl::front_and_back;
                if ((front_face && cull_front) || (!front_face && cull_back)) {
                    continue;
                }
            }

            // GL_LIGHT_MODEL_TWO_SIDE: back-facing polygons use the back colour.
            if (state.two_sided_color && !front_face) {
                for (uint32_t i = 0; i < 3; ++i) {
                    w[i].color = vertices[base + i].color[1];
                }
            }

            raster_triangle(state, w[0], w[1], w[2], offset);
        }
        return;
    }

    if (kind == PrimitiveKind::lines) {
        // Segments of one draw continue the counter; a reset draw restarts it.
        if (state.line_stipple_reset) {
            stipple_counter_ = 0;
        }
        for (uint32_t base = 0; base + 2 <= count; base += 2) {
            Window a, b;
            if (to_window(vertices[base], state, a) && to_window(vertices[base + 1], state, b)) {
                raster_line(state, a, b);
            }
        }
        return;
    }

    for (uint32_t index = 0; index < count; ++index) {
        Window point;
        if (to_window(vertices[index], state, point)) {
            raster_point(state, point);
        }
    }
}

void SoftwareBackend::draw_pixels(const DrawState& state, const PixelFragments& pixels) noexcept {
    if (width_ == 0 || height_ == 0 || pixels.width == 0 || pixels.height == 0 ||
        !(pixels.zoom_x != 0) || !(pixels.zoom_y != 0)) {
        return;
    }

    // Candidate window pixels: the zoomed image bounds, widened by one pixel on
    // each side; the exact rule below decides every centre.
    const float inv_zoom_x = 1.0f / pixels.zoom_x;
    const float inv_zoom_y = 1.0f / pixels.zoom_y;
    const float far_x = pixels.x + pixels.zoom_x * static_cast<float>(pixels.width);
    const float near_y = pixels.y + pixels.zoom_y * static_cast<float>(pixels.first_row);
    const float far_y = pixels.y + pixels.zoom_y * static_cast<float>(pixels.first_row + pixels.height);
    const float lo_xf = pixels.x < far_x ? pixels.x : far_x;
    const float hi_xf = pixels.x < far_x ? far_x : pixels.x;
    const float lo_yf = near_y < far_y ? near_y : far_y;
    const float hi_yf = near_y < far_y ? far_y : near_y;

    if (!std::isfinite(lo_xf) || !std::isfinite(hi_xf) || !std::isfinite(lo_yf) || !std::isfinite(hi_yf)) {
        return;
    }

    auto bound = [](float value, uint32_t limit) -> int32_t {
        return value <= 0 ? 0 : value >= static_cast<float>(limit) ? static_cast<int32_t>(limit)
                                                                   : static_cast<int32_t>(value);
    };
    const int32_t x0 = bound(std::floor(lo_xf) - 1.0f, width_);
    const int32_t y0 = bound(std::floor(lo_yf) - 1.0f, height_);
    const int32_t x1 = bound(std::ceil(hi_xf) + 1.0f, width_);
    const int32_t y1 = bound(std::ceil(hi_yf) + 1.0f, height_);
    const float columns = static_cast<float>(pixels.width);
    const float first_row = static_cast<float>(pixels.first_row);
    const float end_row = static_cast<float>(pixels.first_row + pixels.height);

    for (int32_t y = y0; y < y1; ++y) {
        const float row_f = (static_cast<float>(y) + 0.5f - pixels.y) * inv_zoom_y;
        if (!(row_f >= first_row) || !(row_f < end_row)) {
            continue;
        }
        const uint32_t row = static_cast<uint32_t>(std::floor(row_f)) - pixels.first_row;

        for (int32_t x = x0; x < x1; ++x) {
            const float column_f = (static_cast<float>(x) + 0.5f - pixels.x) * inv_zoom_x;
            if (!(column_f >= 0) || !(column_f < columns)) {
                continue;
            }
            const size_t source = static_cast<size_t>(row) * pixels.width + static_cast<uint32_t>(std::floor(column_f));

            if ((pixels.coverage != nullptr && pixels.coverage[source] == 0) || !scissor_ok(state, x, y)) {
                continue;
            }

            if (pixels.stencil != nullptr) {
                // GL 1.1 4.3.1: ownership and scissor only, then the write mask.
                uint8_t& stored = stencil_[static_cast<size_t>(y) * width_ + static_cast<size_t>(x)];
                const uint8_t mask = static_cast<uint8_t>(state.stencil.write_mask);
                stored = static_cast<uint8_t>((stored & ~mask) | (pixels.stencil[source] & mask));
                continue;
            }

            Vec4 color = pixels.color;
            if (pixels.rgba != nullptr) {
                color = {pixels.rgba[source * 4 + 0], pixels.rgba[source * 4 + 1],
                         pixels.rgba[source * 4 + 2], pixels.rgba[source * 4 + 3]};
            }
            const float z = pixels.depth != nullptr ? pixels.depth[source] : pixels.z;
            shade_fragment(state, static_cast<uint32_t>(x), static_cast<uint32_t>(y), z, color,
                           pixels.texcoord, pixels.fog, -texture_lod_limit);
        }
    }
}

void SoftwareBackend::read(ReadSource source, gl::Enum buffer, int32_t x, int32_t y,
                           uint32_t rect_width, uint32_t rect_height, float* output) noexcept {
    if (output == nullptr || width_ == 0 || height_ == 0) {
        return;
    }

    const uint32_t* color = color_buffer(buffer == gl::front ? gl::front : gl::back);

    for (uint32_t row = 0; row < rect_height; ++row) {
        for (uint32_t column = 0; column < rect_width; ++column) {
            const int32_t px = x + static_cast<int32_t>(column);
            const int32_t py = y + static_cast<int32_t>(row);
            const size_t destination = static_cast<size_t>(row) * rect_width + column;

            if (px < 0 || py < 0 || px >= static_cast<int32_t>(width_) ||
                py >= static_cast<int32_t>(height_)) {
                continue;
            }

            const size_t index = static_cast<size_t>(py) * width_ + px;

            if (source == ReadSource::color) {
                const Vec4 pixel = color != nullptr ? unpack_bgra(color[index]) : Vec4{};
                output[destination * 4 + 0] = pixel.x;
                output[destination * 4 + 1] = pixel.y;
                output[destination * 4 + 2] = pixel.z;
                output[destination * 4 + 3] = pixel.w;
            } else if (source == ReadSource::depth) {
                output[destination] = static_cast<float>(depth_[index]) / 4294967295.0f;
            } else {
                output[destination] = static_cast<float>(stencil_[index]);
            }
        }
    }
}

} // namespace rx7900::opengl

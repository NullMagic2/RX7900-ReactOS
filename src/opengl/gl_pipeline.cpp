/*
Module: rx7900.opengl.gl_pipeline
Purpose: Matrix arithmetic, texture completeness and level of detail, and the
window/line arithmetic shared by every GL module.
Created: 2026-10-23
Architecture: Pure functions over the gl_pipeline value types; no allocation.
*/
#include "gl_pipeline.hpp"

#include <cmath>

namespace rx7900::opengl {

Matrix4 matrix_identity() noexcept {
    Matrix4 result{};
    result.values[0] = result.values[5] = result.values[10] = result.values[15] = 1;
    return result;
}

Matrix4 matrix_multiply(const Matrix4& left, const Matrix4& right) noexcept {
    Matrix4 result{};

    for (uint32_t column = 0; column < 4; ++column) {
        for (uint32_t row = 0; row < 4; ++row) {
            float sum = 0;

            for (uint32_t inner = 0; inner < 4; ++inner) {
                sum += left.values[inner * 4 + row] * right.values[column * 4 + inner];
            }

            result.values[column * 4 + row] = sum;
        }
    }

    return result;
}

Vec4 matrix_transform(const Matrix4& m, const Vec4& v) noexcept {
    const float* a = m.values;
    return {a[0] * v.x + a[4] * v.y + a[8] * v.z + a[12] * v.w,
            a[1] * v.x + a[5] * v.y + a[9] * v.z + a[13] * v.w,
            a[2] * v.x + a[6] * v.y + a[10] * v.z + a[14] * v.w,
            a[3] * v.x + a[7] * v.y + a[11] * v.z + a[15] * v.w};
}

bool matrix_invert(const Matrix4& matrix, Matrix4& inverse) noexcept {
    // Gauss-Jordan elimination with partial pivoting in double precision.
    double work[4][8]{};

    for (uint32_t row = 0; row < 4; ++row) {
        for (uint32_t column = 0; column < 4; ++column) {
            work[row][column] = matrix.values[column * 4 + row];
        }

        work[row][4 + row] = 1;
    }

    for (uint32_t pivot = 0; pivot < 4; ++pivot) {
        uint32_t best = pivot;

        for (uint32_t row = pivot + 1; row < 4; ++row) {
            if (std::fabs(work[row][pivot]) > std::fabs(work[best][pivot])) {
                best = row;
            }
        }

        if (!(std::fabs(work[best][pivot]) > 1e-30)) {
            return false;
        }

        if (best != pivot) {
            for (uint32_t column = 0; column < 8; ++column) {
                const double swap = work[pivot][column];
                work[pivot][column] = work[best][column];
                work[best][column] = swap;
            }
        }

        const double scale = 1.0 / work[pivot][pivot];

        for (uint32_t column = 0; column < 8; ++column) {
            work[pivot][column] *= scale;
        }

        for (uint32_t row = 0; row < 4; ++row) {
            if (row == pivot) {
                continue;
            }

            const double factor = work[row][pivot];

            for (uint32_t column = 0; column < 8; ++column) {
                work[row][column] -= factor * work[pivot][column];
            }
        }
    }

    for (uint32_t row = 0; row < 4; ++row) {
        for (uint32_t column = 0; column < 4; ++column) {
            const double value = work[row][4 + column];

            if (!std::isfinite(value)) {
                return false;
            }

            inverse.values[column * 4 + row] = static_cast<float>(value);
        }
    }

    return true;
}

bool texture_complete(const Texture& texture) noexcept {
    const TextureLevel& base = texture.levels[0];

    if (base.texels == nullptr || base.width == 0 || base.height == 0) {
        return false;
    }

    const gl::Enum filter = texture.sampler.min_filter;

    if (filter == gl::nearest || filter == gl::linear) {
        return true;
    }

    uint32_t width = base.width;
    uint32_t height = base.height;

    for (uint32_t level = 1; level < texture_max_levels; ++level) {
        if (width == 1 && height == 1) {
            return true;
        }

        width = width > 1 ? width / 2 : 1;
        height = height > 1 ? height / 2 : 1;
        const TextureLevel& image = texture.levels[level];

        if (image.texels == nullptr || image.width != width || image.height != height ||
            image.base_format != base.base_format) {
            return false;
        }
    }

    return width == 1 && height == 1;
}

float texture_lod_threshold(const TextureSampler& sampler) noexcept {
    return sampler.mag_filter == gl::linear && (sampler.min_filter == gl::nearest_mipmap_nearest ||
                                                sampler.min_filter == gl::nearest_mipmap_linear)
               ? 0.5f
               : 0.0f;
}

uint32_t texture_last_level(const Texture& texture) noexcept {
    if (!mipmap_filter(texture.sampler.min_filter)) {
        return 0;
    }

    uint32_t width = texture.levels[0].width;
    uint32_t height = texture.levels[0].height;
    uint32_t level = 0;

    while ((width > 1 || height > 1) && level + 1 < texture_max_levels) {
        width = width > 1 ? width / 2 : 1;
        height = height > 1 ? height / 2 : 1;
        ++level;
    }

    return level;
}

namespace {

float clamp_lod(float lambda) noexcept {
    // NaN (no difference at all) and -inf both mean "no minification".
    if (!(lambda >= -texture_lod_limit)) {
        return -texture_lod_limit;
    }

    return lambda > texture_lod_limit ? texture_lod_limit : lambda;
}

} // namespace

float quad_lod(float u0, float v0, float u1, float v1, float u2, float v2) noexcept {
    const float dux = u1 - u0;
    const float dvx = v1 - v0;
    const float duy = u2 - u0;
    const float dvy = v2 - v0;
    const float rho_x = dux * dux + dvx * dvx;
    const float rho_y = duy * duy + dvy * dvy;
    const float rho = rho_x > rho_y ? rho_x : rho_y;
    return clamp_lod(std::log2(rho) * 0.5f);
}

float line_lod(float ax, float ay, float bx, float by, const Vec4& texcoord_a, const Vec4& texcoord_b,
               uint32_t base_width, uint32_t base_height) noexcept {
    const float qa = texcoord_a.w != 0 ? texcoord_a.w : 1.0f;
    const float qb = texcoord_b.w != 0 ? texcoord_b.w : 1.0f;
    const float du = (texcoord_b.x / qb - texcoord_a.x / qa) * static_cast<float>(base_width);
    const float dv = (texcoord_b.y / qb - texcoord_a.y / qa) * static_cast<float>(base_height);
    const float dx = std::fabs(bx - ax);
    const float dy = std::fabs(by - ay);
    const float major = dx > dy ? dx : dy;
    return clamp_lod(std::log2((du * du + dv * dv) / (major * major)) * 0.5f);
}

TextureLevels texture_levels(const TextureSampler& sampler, float lambda, uint32_t last_level) noexcept {
    TextureLevels levels{};

    if (lambda <= texture_lod_threshold(sampler)) {
        return levels;
    }

    levels.magnify = false;
    const float q = static_cast<float>(last_level);

    if (sampler.min_filter == gl::nearest_mipmap_nearest || sampler.min_filter == gl::linear_mipmap_nearest) {
        float d = lambda <= 0.5f ? 0.0f : std::ceil(lambda + 0.5f) - 1.0f;
        d = d < q ? d : q;
        levels.level_a = levels.level_b = static_cast<uint32_t>(d);
    } else if (sampler.min_filter == gl::nearest_mipmap_linear ||
               sampler.min_filter == gl::linear_mipmap_linear) {
        if (lambda >= q) {
            levels.level_a = levels.level_b = last_level;
        } else {
            const float base = std::floor(lambda);
            levels.level_a = static_cast<uint32_t>(base);
            levels.level_b = levels.level_a + 1u;
            levels.weight = lambda - base;
        }
    }

    return levels;
}

bool window_xy(const RasterVertex& vertex, const DrawState& state, float& x, float& y) noexcept {
    if (!(vertex.clip.w > 0) && !(vertex.clip.w < 0)) {
        return false;      // w == 0 or NaN
    }

    const float inv_w = 1.0f / vertex.clip.w;
    const float ndc_x = vertex.clip.x * inv_w;
    const float ndc_y = vertex.clip.y * inv_w;
    x = static_cast<float>(state.viewport_x) + (ndc_x + 1.0f) * 0.5f * static_cast<float>(state.viewport_width);
    y = static_cast<float>(state.viewport_y) + (ndc_y + 1.0f) * 0.5f * static_cast<float>(state.viewport_height);
    return std::isfinite(x) && std::isfinite(y);
}

int32_t line_steps(float ax, float ay, float bx, float by) noexcept {
    const float adx = std::fabs(bx - ax);
    const float ady = std::fabs(by - ay);
    return static_cast<int32_t>((adx > ady ? adx : ady) + 1.0f);
}

} // namespace rx7900::opengl

/*
Module: rx7900.opengl.gl_clip
Purpose: Clip-space Sutherland-Hodgman polygon and parametric line clipping.
Created: 2026-10-23
Architecture: See gl_clip.hpp. Algorithms follow the OpenGL 1.1 specification,
section 2.11 (Clipping); no third-party code.
*/
#include "gl_clip.hpp"

namespace rx7900::opengl {
namespace {

float distance(const Vec4& plane, const Vec4& p) noexcept {
    return plane.x * p.x + plane.y * p.y + plane.z * p.z + plane.w * p.w;
}

Vec4 lerp(const Vec4& a, const Vec4& b, float t) noexcept {
    return {a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t,
            a.w + (b.w - a.w) * t};
}

} // namespace

ClipPlanes view_volume_planes() noexcept {
    ClipPlanes result{};
    result.planes[0] = {1, 0, 0, 1};
    result.planes[1] = {-1, 0, 0, 1};
    result.planes[2] = {0, 1, 0, 1};
    result.planes[3] = {0, -1, 0, 1};
    result.planes[4] = {0, 0, 1, 1};
    result.planes[5] = {0, 0, -1, 1};
    result.count = clip_view_volume_planes;
    return result;
}

RasterVertex clip_lerp(const RasterVertex& a, const RasterVertex& b, float t) noexcept {
    RasterVertex result{};
    result.clip = lerp(a.clip, b.clip, t);
    result.color[0] = lerp(a.color[0], b.color[0], t);
    result.color[1] = lerp(a.color[1], b.color[1], t);
    result.texcoord = lerp(a.texcoord, b.texcoord, t);
    result.fog = a.fog + (b.fog - a.fog) * t;
    return result;
}

uint32_t clip_polygon(const RasterVertex* input, uint32_t count, const ClipPlanes& planes,
                      RasterVertex* output, RasterVertex* scratch, uint32_t capacity,
                      const bool* edges, bool* edges_out, bool* edge_scratch) noexcept {
    const bool track = edges != nullptr && edges_out != nullptr && edge_scratch != nullptr;

    if (count < 3 || capacity < count + planes.count) {
        return 0;
    }

    // Ping-pong between output and scratch so the final result lands in output.
    RasterVertex* source = (planes.count % 2 == 0) ? output : scratch;
    RasterVertex* target = (planes.count % 2 == 0) ? scratch : output;
    bool* source_edges = (planes.count % 2 == 0) ? edges_out : edge_scratch;
    bool* target_edges = (planes.count % 2 == 0) ? edge_scratch : edges_out;

    for (uint32_t index = 0; index < count; ++index) {
        source[index] = input[index];

        if (track) {
            source_edges[index] = edges[index];
        }
    }

    uint32_t current = count;

    for (uint32_t plane = 0; plane < planes.count; ++plane) {
        const Vec4& p = planes.planes[plane];
        uint32_t produced = 0;

        for (uint32_t index = 0; index < current; ++index) {
            const RasterVertex& a = source[index];
            const RasterVertex& b = source[(index + 1) % current];
            const float da = distance(p, a.clip);
            const float db = distance(p, b.clip);
            const bool edge = track ? source_edges[index] : true;

            if (da >= 0) {
                target[produced] = a;

                if (track) {
                    target_edges[produced] = edge;
                }

                ++produced;
            }

            if ((da >= 0) != (db >= 0)) {
                const float t = da / (da - db);
                target[produced] = clip_lerp(a, b, t);

                if (track) {
                    // Leaving the half-space starts a new boundary edge along the
                    // plane; entering continues the original edge.
                    target_edges[produced] = da >= 0 ? true : edge;
                }

                ++produced;
            }
        }

        if (produced < 3) {
            return 0;
        }

        current = produced;
        RasterVertex* swap = source;
        source = target;
        target = swap;
        bool* swap_edges = source_edges;
        source_edges = target_edges;
        target_edges = swap_edges;
    }

    return current;
}

bool clip_line(RasterVertex& a, RasterVertex& b, const ClipPlanes& planes) noexcept {
    float t0 = 0;
    float t1 = 1;

    for (uint32_t plane = 0; plane < planes.count; ++plane) {
        const float da = distance(planes.planes[plane], a.clip);
        const float db = distance(planes.planes[plane], b.clip);

        if (da < 0 && db < 0) {
            return false;
        }

        if (da < 0) {
            const float t = da / (da - db);
            t0 = t > t0 ? t : t0;
        } else if (db < 0) {
            const float t = da / (da - db);
            t1 = t < t1 ? t : t1;
        }
    }

    if (t0 > t1) {
        return false;
    }

    const RasterVertex original_a = a;

    if (t0 > 0) {
        a = clip_lerp(original_a, b, t0);
    }

    if (t1 < 1) {
        b = clip_lerp(original_a, b, t1);
    }

    return true;
}

bool clip_point_inside(const Vec4& clip, const ClipPlanes& planes) noexcept {
    for (uint32_t plane = 0; plane < planes.count; ++plane) {
        if (distance(planes.planes[plane], clip) < 0) {
            return false;
        }
    }

    return true;
}

} // namespace rx7900::opengl

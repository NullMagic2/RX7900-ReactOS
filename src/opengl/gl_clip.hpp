/*
Module: rx7900.opengl.gl_clip
Purpose: Homogeneous clip-space polygon and line clipping with attribute and
edge-flag interpolation.
Created: 2026-10-23
Architecture: Shared by the frontend (user clip planes, polygon modes, select and
feedback) and the software rasterizer (view volume). Pure, allocation-free.
*/
#pragma once

#include "gl_pipeline.hpp"

namespace rx7900::opengl {

constexpr uint32_t clip_view_volume_planes = 6;
constexpr uint32_t clip_user_planes = 6;
constexpr uint32_t clip_max_planes = clip_view_volume_planes + clip_user_planes;

// Plane p keeps clip-space points with p.x*x + p.y*y + p.z*z + p.w*w >= 0.
struct ClipPlanes final {
    Vec4 planes[clip_max_planes]{};
    uint32_t count = 0;
};

// The six GL view-volume planes -w <= x, y, z <= w.
ClipPlanes view_volume_planes() noexcept;

// Linear interpolation of every RasterVertex attribute: a + (b - a) * t.
RasterVertex clip_lerp(const RasterVertex& a, const RasterVertex& b, float t) noexcept;

// Sutherland-Hodgman clip of a convex polygon. input/output/scratch each hold
// capacity vertices; capacity must be at least count + planes.count. edges, when
// non-null, has one flag per input edge (vertex i to i+1) and receives the
// output edge flags in edges_out (capacity entries); edges created along a clip
// plane are boundary edges (true). Returns the output vertex count (0 if fully
// clipped, or if capacity is insufficient).
uint32_t clip_polygon(const RasterVertex* input, uint32_t count, const ClipPlanes& planes,
                      RasterVertex* output, RasterVertex* scratch, uint32_t capacity,
                      const bool* edges = nullptr, bool* edges_out = nullptr,
                      bool* edge_scratch = nullptr) noexcept;

// Clips segment a-b in place; false when nothing remains.
bool clip_line(RasterVertex& a, RasterVertex& b, const ClipPlanes& planes) noexcept;

// True when the point is inside every plane.
bool clip_point_inside(const Vec4& clip, const ClipPlanes& planes) noexcept;

} // namespace rx7900::opengl

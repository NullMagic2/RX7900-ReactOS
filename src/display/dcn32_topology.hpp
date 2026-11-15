/*
Module: rx7900.display.dcn32_topology
Purpose: DCN 3.2 pipe topology of one stream: ODM slices, MPC blending trees, Linux pipe acquisition and 1:1 scaler geometry.
Created: 2026-10-25
Architecture: Portable display core shared by dcn32_bandwidth (which runs the
Linux validation flow that acquires pipes between its two DML passes) and
dcn32_stream (which programs the pipes). Dcn32Topology mirrors the pipe_ctx
relations of one dc_state for one stream (stream/plane presence, top/bottom
MPC links, prev/next ODM links, OPP and DSC instances, scaler data). The
functions transcribe Linux v6.12 dc_resource.c (resource_is_pipe_type,
resource_get_odm_slice_count/index, resource_get_opp_heads_for_otg_master,
add_plane_to_opp_head_pipes, acquire_secondary_dpp_pipes_and_add_plane,
acquire_pipes_and_add_odm_slice, resource_get_odm_slice_dst_width/rect,
calculate_recout, calculate_init_and_vp at a 1:1 ratio with one tap,
resource_build_scaling_params' MIN_VIEWPORT_SIZE clamp), the dcn32 free-pipe
policies of dcn32_resource.c (dcn32_find_optimal_free_pipe_as_secondary_dpp_pipe,
find_optimal_free_pipe_as_secondary_opp_head with dcn20_acquire_dsc on a pool
with one DSC per OPP) and dcn32_is_pipe_topology_transition_seamless
(dcn32_hwseq.c). Planes are unscaled linear 32 bpp surfaces whose source is
the whole surface; the stream source and destination equal the active area.
Linux AMD notice and provenance: licenses/AMDGPU-DISPLAY.txt.
*/
#pragma once

#include <stdint.h>

namespace rx7900::display {

constexpr uint32_t dcn32_topology_pipes = 4;
// Planes of one stream this owner composes (plane 0 is the top layer).
constexpr uint32_t dcn32_max_planes = 2;
constexpr uint8_t dcn32_no_pipe = 0xff;
// resource.h MIN_VIEWPORT_SIZE.
constexpr int32_t dcn32_min_viewport_size = 12;

// dc rect.
struct Dcn32Rect final {
    int32_t x = 0;
    int32_t y = 0;
    int32_t width = 0;
    int32_t height = 0;
};

// The dc_plane_state geometry of one plane: src_rect = (0, 0, surface) and
// dst_rect = clip_rect = dst in stream source space (equal to timing active).
struct Dcn32PlaneGeometry final {
    uint32_t surface_width = 0;
    uint32_t surface_height = 0;
    Dcn32Rect dst{};
};

// One pipe_ctx as far as this owner uses it.
struct Dcn32TopologyPipe final {
    bool stream = false;      // pipe_ctx->stream != NULL
    bool has_plane = false;   // pipe_ctx->plane_state != NULL
    uint8_t plane = 0;        // index of plane_state in the stream status
    uint8_t top = dcn32_no_pipe;
    uint8_t bottom = dcn32_no_pipe;
    uint8_t prev_odm = dcn32_no_pipe;
    uint8_t next_odm = dcn32_no_pipe;
    uint8_t opp = 0;          // stream_res.opp->inst
    bool dsc = false;         // stream_res.dsc (instance = pipe index)
    // plane_res.scl_data: viewport in the surface, recout in the ODM slice.
    Dcn32Rect viewport{};
    Dcn32Rect recout{};
};

struct Dcn32Topology final {
    Dcn32TopologyPipe pipes[dcn32_topology_pipes]{};
    uint8_t otg_master = dcn32_no_pipe;
    uint8_t plane_count = 0;
};

// resource_is_pipe_type.
bool topology_is_otg_master(const Dcn32Topology& t, uint32_t pipe) noexcept;
bool topology_is_opp_head(const Dcn32Topology& t, uint32_t pipe) noexcept;
bool topology_is_dpp_pipe(const Dcn32Topology& t, uint32_t pipe) noexcept;
bool topology_is_free_pipe(const Dcn32Topology& t, uint32_t pipe) noexcept;

// resource_get_odm_slice_count of the OTG master (0 without a stream).
uint32_t topology_odm_slice_count(const Dcn32Topology& t) noexcept;
// resource_get_odm_slice_index (of the pipe's OPP head).
uint32_t topology_odm_slice_index(const Dcn32Topology& t, uint32_t pipe) noexcept;
// The OPP head of a pipe (walk top links).
uint32_t topology_opp_head(const Dcn32Topology& t, uint32_t pipe) noexcept;
// resource_get_opp_heads_for_otg_master: OTG master first, then next_odm.
uint32_t topology_opp_heads(const Dcn32Topology& t, uint8_t (&heads)[dcn32_topology_pipes]) noexcept;
// resource_get_dpp_pipes_for_opp_head: the head, then its bottom chain.
uint32_t topology_dpp_pipes(const Dcn32Topology& t, uint32_t opp_head,
                            uint8_t (&pipes)[dcn32_topology_pipes]) noexcept;
uint32_t topology_used_pipes(const Dcn32Topology& t) noexcept;
// resource_get_odm_slice_dst_width (RGB: no two-pixel alignment).
uint32_t topology_odm_slice_width(const Dcn32Topology& t, uint32_t h_active, bool last) noexcept;

// A dc_state that holds one new stream on otg (resource_map_pool_resources:
// OTG master pipe = OTG = OPP = pipe otg, DSC acquired when dsc) and no plane.
void topology_start_stream(Dcn32Topology& t, uint32_t otg, bool dsc) noexcept;
// resource_append_dpp_pipes_for_plane_composition (without the scaling step).
bool topology_append_plane(Dcn32Topology& new_t, const Dcn32Topology& cur, uint8_t plane) noexcept;
// resource_update_pipes_for_stream_with_slice_count, growing only (the new
// state of this owner always starts at one slice), without the scaling step.
bool topology_add_odm_slices(Dcn32Topology& new_t, const Dcn32Topology& cur, uint32_t slice_count) noexcept;
// resource_build_scaling_params for every pipe with a plane (1:1, one tap).
// false when a plane rectangle is empty or outside the active area.
bool topology_build_scaling(Dcn32Topology& t, uint32_t h_active, uint32_t v_active,
                            const Dcn32PlaneGeometry* planes, uint32_t plane_count) noexcept;
// dcn32_is_pipe_topology_transition_seamless for the same stream on both sides.
bool topology_transition_seamless(const Dcn32Topology& cur, const Dcn32Topology& next) noexcept;

} // namespace rx7900::display

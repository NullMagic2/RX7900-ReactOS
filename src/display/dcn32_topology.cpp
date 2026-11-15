/*
Module: rx7900.display.dcn32_topology
Purpose: Linux-ordered DCN 3.2 pipe acquisition, ODM slice and blending tree relations, and 1:1 scaler geometry.
Created: 2026-10-25
Architecture: Implementation of dcn32_topology.hpp. Each function names the
Linux v6.12 function it transcribes (dc_resource.c, dcn32_resource.c,
dcn32_hwseq.c); free-pipe searches visit pipes in index order exactly as the
pool loops do. Adapted from AMD DC (Copyright 2012-2023 Advanced Micro Devices,
Inc., MIT notice in licenses/AMDGPU-DISPLAY.txt).
*/
#include "dcn32_topology.hpp"

namespace rx7900::display {

namespace {

constexpr int free_pipe_not_found = -1; // FREE_PIPE_INDEX_NOT_FOUND

bool valid_pipe(uint32_t pipe) noexcept { return pipe < dcn32_topology_pipes; }

// get_tail_pipe.
uint32_t tail_pipe(const Dcn32Topology& t, uint32_t head) noexcept {
    uint32_t pipe = head;
    uint32_t guard = 0;
    while (t.pipes[pipe].bottom != dcn32_no_pipe && guard++ < dcn32_topology_pipes) {
        pipe = t.pipes[pipe].bottom;
    }
    return pipe;
}

// get_last_opp_head.
uint32_t last_opp_head(const Dcn32Topology& t, uint32_t otg_master) noexcept {
    uint32_t pipe = otg_master;
    uint32_t guard = 0;
    while (t.pipes[pipe].next_odm != dcn32_no_pipe && guard++ < dcn32_topology_pipes) {
        pipe = t.pipes[pipe].next_odm;
    }
    return pipe;
}

// resource_get_mpc_slice_index.
uint32_t mpc_slice_index(const Dcn32Topology& t, uint32_t pipe) noexcept {
    uint32_t index = 0;
    uint32_t split = t.pipes[pipe].top;
    while (split != dcn32_no_pipe && t.pipes[split].has_plane == t.pipes[pipe].has_plane &&
           t.pipes[split].plane == t.pipes[pipe].plane && index < dcn32_topology_pipes) {
        ++index;
        split = t.pipes[split].top;
    }
    return index;
}

uint32_t free_pipe_count(const Dcn32Topology& t) noexcept {
    uint32_t count = 0;
    for (uint32_t i = 0; i < dcn32_topology_pipes; ++i) {
        count += topology_is_free_pipe(t, i) ? 1u : 0u;
    }
    return count;
}

// recource_find_free_pipe_not_used_in_cur_res_ctx.
int free_pipe_not_used_in_cur(const Dcn32Topology& cur, const Dcn32Topology& next) noexcept {
    for (uint32_t i = 0; i < dcn32_topology_pipes; ++i) {
        if (topology_is_free_pipe(cur, i) && topology_is_free_pipe(next, i)) {
            return static_cast<int>(i);
        }
    }
    return free_pipe_not_found;
}

// resource_find_any_free_pipe.
int any_free_pipe(const Dcn32Topology& next) noexcept {
    for (uint32_t i = 0; i < dcn32_topology_pipes; ++i) {
        if (topology_is_free_pipe(next, i)) {
            return static_cast<int>(i);
        }
    }
    return free_pipe_not_found;
}

// dcn32_find_optimal_free_pipe_as_secondary_dpp_pipe.
int find_secondary_dpp_pipe(const Dcn32Topology& cur, const Dcn32Topology& next, uint32_t new_opp_head) noexcept {
    // resource_find_free_pipe_used_in_cur_mpc_blending_tree.
    uint32_t cur_sec_dpp = cur.pipes[new_opp_head].bottom;
    uint32_t guard = 0;
    while (cur_sec_dpp != dcn32_no_pipe && guard++ < dcn32_topology_pipes) {
        if (topology_is_free_pipe(next, cur_sec_dpp)) {
            return static_cast<int>(cur_sec_dpp);
        }
        cur_sec_dpp = cur.pipes[cur_sec_dpp].bottom;
    }
    int index = free_pipe_not_used_in_cur(cur, next);
    if (index == free_pipe_not_found) {
        // resource_find_free_pipe_used_as_cur_sec_dpp_in_mpcc_combine.
        for (uint32_t i = 0; i < dcn32_topology_pipes; ++i) {
            if (topology_is_dpp_pipe(cur, i) && !topology_is_opp_head(cur, i) && mpc_slice_index(cur, i) > 0 &&
                topology_is_free_pipe(next, i)) {
                index = static_cast<int>(i);
                break;
            }
        }
    }
    if (index == free_pipe_not_found) {
        index = any_free_pipe(next);
    }
    return index;
}

// find_optimal_free_pipe_as_secondary_opp_head.
int find_secondary_opp_head(const Dcn32Topology& cur, const Dcn32Topology& next, uint32_t new_otg_master) noexcept {
    // resource_find_free_pipe_used_as_sec_opp_head_by_cur_otg_master.
    uint32_t cur_sec_opp_head = cur.pipes[new_otg_master].next_odm;
    uint32_t guard = 0;
    while (cur_sec_opp_head != dcn32_no_pipe && guard++ < dcn32_topology_pipes) {
        if (topology_is_free_pipe(next, cur_sec_opp_head)) {
            return static_cast<int>(cur_sec_opp_head);
        }
        cur_sec_opp_head = cur.pipes[cur_sec_opp_head].next_odm;
    }
    int index = free_pipe_not_used_in_cur(cur, next);
    if (index == free_pipe_not_found) {
        index = any_free_pipe(next);
    }
    return index;
}

// dcn32_acquire_free_pipe_as_secondary_dpp_pipe.
int acquire_secondary_dpp_pipe(const Dcn32Topology& cur, Dcn32Topology& next, uint32_t opp_head) noexcept {
    const int index = find_secondary_dpp_pipe(cur, next, opp_head);
    if (index >= 0) {
        Dcn32TopologyPipe& pipe = next.pipes[index];
        pipe = Dcn32TopologyPipe{};
        pipe.stream = true;
        pipe.opp = next.pipes[opp_head].opp;
    }
    return index;
}

// dcn32_acquire_free_pipe_as_secondary_opp_head (dcn20_acquire_dsc: one DSC
// per OPP, so the DSC of pipe i is always free with the pipe).
int acquire_secondary_opp_head(const Dcn32Topology& cur, Dcn32Topology& next, uint32_t otg_master) noexcept {
    const int index = find_secondary_opp_head(cur, next, otg_master);
    if (index >= 0) {
        Dcn32TopologyPipe& pipe = next.pipes[index];
        pipe = Dcn32TopologyPipe{};
        pipe.stream = true;
        pipe.opp = static_cast<uint8_t>(index);
        pipe.dsc = next.pipes[otg_master].dsc;
    }
    return index;
}

// intersect_rec.
Dcn32Rect intersect(const Dcn32Rect& r0, const Dcn32Rect& r1) noexcept {
    Dcn32Rect rec{};
    const int32_t r0_x_end = r0.x + r0.width;
    const int32_t r1_x_end = r1.x + r1.width;
    const int32_t r0_y_end = r0.y + r0.height;
    const int32_t r1_y_end = r1.y + r1.height;
    rec.x = r0.x > r1.x ? r0.x : r1.x;
    rec.width = r0_x_end > r1_x_end ? r1_x_end - rec.x : r0_x_end - rec.x;
    rec.y = r0.y > r1.y ? r0.y : r1.y;
    rec.height = r0_y_end > r1_y_end ? r1_y_end - rec.y : r0_y_end - rec.y;
    if (rec.width < 0 || rec.height < 0) {
        rec = Dcn32Rect{};
    }
    return rec;
}

// shift_rec.
Dcn32Rect shift(const Dcn32Rect& rec, int32_t x, int32_t y) noexcept {
    Dcn32Rect out = rec;
    out.x += x;
    out.y += y;
    return out;
}

// calculate_init_and_vp with ratio 1.0 and one tap: init = (1 + 1 + 1) / 2 =
// 1.5, so the offset is the recout offset and the size the recout size, cut
// at the end of the source.
void init_and_vp(int32_t recout_offset, int32_t recout_size, int32_t src_size, int32_t& vp_offset,
                 int32_t& vp_size) noexcept {
    vp_offset = recout_offset;
    // dc_fixpt_floor(1.5 + (recout_size - 1)) for a recout size of 0 or more.
    vp_size = recout_size;
    if (vp_size + vp_offset > src_size) {
        vp_size = src_size - vp_offset;
    }
}

} // namespace

bool topology_is_otg_master(const Dcn32Topology& t, uint32_t pipe) noexcept {
    return valid_pipe(pipe) && t.pipes[pipe].prev_odm == dcn32_no_pipe && t.pipes[pipe].top == dcn32_no_pipe &&
           t.pipes[pipe].stream;
}

bool topology_is_opp_head(const Dcn32Topology& t, uint32_t pipe) noexcept {
    return valid_pipe(pipe) && t.pipes[pipe].top == dcn32_no_pipe && t.pipes[pipe].stream;
}

bool topology_is_dpp_pipe(const Dcn32Topology& t, uint32_t pipe) noexcept {
    return valid_pipe(pipe) && t.pipes[pipe].has_plane && t.pipes[pipe].stream;
}

bool topology_is_free_pipe(const Dcn32Topology& t, uint32_t pipe) noexcept {
    return valid_pipe(pipe) && !t.pipes[pipe].has_plane && !t.pipes[pipe].stream;
}

uint32_t topology_odm_slice_count(const Dcn32Topology& t) noexcept {
    if (!topology_is_otg_master(t, t.otg_master)) {
        return 0;
    }
    uint32_t count = 1;
    uint32_t pipe = t.otg_master;
    while (t.pipes[pipe].next_odm != dcn32_no_pipe && count < dcn32_topology_pipes) {
        ++count;
        pipe = t.pipes[pipe].next_odm;
    }
    return count;
}

uint32_t topology_opp_head(const Dcn32Topology& t, uint32_t pipe) noexcept {
    uint32_t guard = 0;
    while (valid_pipe(pipe) && t.pipes[pipe].top != dcn32_no_pipe && guard++ < dcn32_topology_pipes) {
        pipe = t.pipes[pipe].top;
    }
    return pipe;
}

uint32_t topology_odm_slice_index(const Dcn32Topology& t, uint32_t pipe) noexcept {
    pipe = topology_opp_head(t, pipe);
    if (!valid_pipe(pipe)) {
        return 0;
    }
    uint32_t index = 0;
    while (t.pipes[pipe].prev_odm != dcn32_no_pipe && index < dcn32_topology_pipes) {
        ++index;
        pipe = t.pipes[pipe].prev_odm;
    }
    return index;
}

uint32_t topology_opp_heads(const Dcn32Topology& t, uint8_t (&heads)[dcn32_topology_pipes]) noexcept {
    uint32_t count = 0;
    if (!topology_is_otg_master(t, t.otg_master)) {
        return 0;
    }
    uint32_t pipe = t.otg_master;
    while (pipe != dcn32_no_pipe && count < dcn32_topology_pipes) {
        heads[count++] = static_cast<uint8_t>(pipe);
        pipe = t.pipes[pipe].next_odm;
    }
    return count;
}

uint32_t topology_dpp_pipes(const Dcn32Topology& t, uint32_t opp_head,
                            uint8_t (&pipes)[dcn32_topology_pipes]) noexcept {
    uint32_t count = 0;
    if (!topology_is_opp_head(t, opp_head)) {
        return 0;
    }
    uint32_t pipe = opp_head;
    while (pipe != dcn32_no_pipe && topology_is_dpp_pipe(t, pipe) && count < dcn32_topology_pipes) {
        pipes[count++] = static_cast<uint8_t>(pipe);
        pipe = t.pipes[pipe].bottom;
    }
    return count;
}

uint32_t topology_used_pipes(const Dcn32Topology& t) noexcept {
    uint32_t count = 0;
    for (uint32_t i = 0; i < dcn32_topology_pipes; ++i) {
        count += t.pipes[i].stream ? 1u : 0u;
    }
    return count;
}

uint32_t topology_odm_slice_width(const Dcn32Topology& t, uint32_t h_active, bool last) noexcept {
    const uint32_t count = topology_odm_slice_count(t);
    if (count == 0) {
        return 0;
    }
    const uint32_t width = h_active / count;
    return last ? h_active - width * (count - 1) : width;
}

void topology_start_stream(Dcn32Topology& t, uint32_t otg, bool dsc) noexcept {
    t = Dcn32Topology{};
    if (!valid_pipe(otg)) {
        return;
    }
    t.otg_master = static_cast<uint8_t>(otg);
    t.pipes[otg].stream = true;
    t.pipes[otg].opp = static_cast<uint8_t>(otg);
    t.pipes[otg].dsc = dsc;
}

bool topology_append_plane(Dcn32Topology& new_t, const Dcn32Topology& cur, uint8_t plane) noexcept {
    const uint32_t master = new_t.otg_master;
    if (!topology_is_otg_master(new_t, master) || plane >= dcn32_max_planes) {
        return false;
    }
    if (!new_t.pipes[master].has_plane) {
        // add_plane_to_opp_head_pipes.
        uint32_t pipe = master;
        while (pipe != dcn32_no_pipe) {
            if (new_t.pipes[pipe].has_plane) {
                return false;
            }
            new_t.pipes[pipe].has_plane = true;
            new_t.pipes[pipe].plane = plane;
            pipe = new_t.pipes[pipe].next_odm;
        }
        ++new_t.plane_count;
        return true;
    }
    // acquire_secondary_dpp_pipes_and_add_plane.
    uint8_t heads[dcn32_topology_pipes]{};
    const uint32_t head_count = topology_opp_heads(new_t, heads);
    if (free_pipe_count(new_t) < head_count) {
        return false;
    }
    for (uint32_t i = 0; i < head_count; ++i) {
        const int sec = acquire_secondary_dpp_pipe(cur, new_t, heads[i]);
        if (sec < 0) {
            return false;
        }
        Dcn32TopologyPipe& sec_pipe = new_t.pipes[sec];
        sec_pipe.has_plane = true;
        sec_pipe.plane = plane;
        const uint32_t tail = tail_pipe(new_t, heads[i]);
        new_t.pipes[tail].bottom = static_cast<uint8_t>(sec);
        sec_pipe.top = static_cast<uint8_t>(tail);
        sec_pipe.bottom = dcn32_no_pipe;
        const uint8_t tail_prev = new_t.pipes[tail].prev_odm;
        if (tail_prev != dcn32_no_pipe) {
            const uint8_t neighbour = new_t.pipes[tail_prev].bottom;
            if (neighbour == dcn32_no_pipe) {
                return false;
            }
            sec_pipe.prev_odm = neighbour;
            new_t.pipes[neighbour].next_odm = static_cast<uint8_t>(sec);
        } else {
            sec_pipe.prev_odm = dcn32_no_pipe;
        }
    }
    ++new_t.plane_count;
    return true;
}

bool topology_add_odm_slices(Dcn32Topology& new_t, const Dcn32Topology& cur, uint32_t slice_count) noexcept {
    const uint32_t master = new_t.otg_master;
    if (!topology_is_otg_master(new_t, master) || slice_count == 0 || slice_count > dcn32_topology_pipes) {
        return false;
    }
    const uint32_t current = topology_odm_slice_count(new_t);
    if (slice_count < current) {
        return false;
    }
    for (uint32_t step = current; step < slice_count; ++step) {
        // acquire_pipes_and_add_odm_slice.
        const uint32_t last_head = last_opp_head(new_t, master);
        const int new_head = acquire_secondary_opp_head(cur, new_t, master);
        if (new_head < 0) {
            return false;
        }
        new_t.pipes[last_head].next_odm = static_cast<uint8_t>(new_head);
        new_t.pipes[new_head].prev_odm = static_cast<uint8_t>(last_head);
        new_t.pipes[new_head].next_odm = dcn32_no_pipe;
        new_t.pipes[new_head].has_plane = new_t.pipes[last_head].has_plane;
        new_t.pipes[new_head].plane = new_t.pipes[last_head].plane;
        uint32_t last_top = last_head;
        uint32_t new_top = static_cast<uint32_t>(new_head);
        uint32_t guard = 0;
        while (new_t.pipes[last_top].bottom != dcn32_no_pipe && guard++ < dcn32_topology_pipes) {
            const uint32_t last_bottom = new_t.pipes[last_top].bottom;
            const int new_bottom = acquire_secondary_dpp_pipe(cur, new_t, static_cast<uint32_t>(new_head));
            if (new_bottom < 0) {
                return false;
            }
            new_t.pipes[new_bottom].has_plane = new_t.pipes[last_bottom].has_plane;
            new_t.pipes[new_bottom].plane = new_t.pipes[last_bottom].plane;
            new_t.pipes[new_top].bottom = static_cast<uint8_t>(new_bottom);
            new_t.pipes[new_bottom].top = static_cast<uint8_t>(new_top);
            new_t.pipes[last_bottom].next_odm = static_cast<uint8_t>(new_bottom);
            new_t.pipes[new_bottom].prev_odm = static_cast<uint8_t>(last_bottom);
            new_t.pipes[new_bottom].next_odm = dcn32_no_pipe;
            last_top = last_bottom;
            new_top = static_cast<uint32_t>(new_bottom);
        }
    }
    return true;
}

bool topology_build_scaling(Dcn32Topology& t, uint32_t h_active, uint32_t v_active,
                            const Dcn32PlaneGeometry* planes, uint32_t plane_count) noexcept {
    if (planes == nullptr || h_active == 0 || v_active == 0) {
        return false;
    }
    const uint32_t odm_count = topology_odm_slice_count(t);
    if (odm_count == 0) {
        return false;
    }
    const Dcn32Rect stream_dst{0, 0, static_cast<int32_t>(h_active), static_cast<int32_t>(v_active)};
    for (uint32_t i = 0; i < dcn32_topology_pipes; ++i) {
        Dcn32TopologyPipe& pipe = t.pipes[i];
        if (!pipe.stream || !pipe.has_plane) {
            continue;
        }
        if (pipe.plane >= plane_count) {
            return false;
        }
        const Dcn32PlaneGeometry& plane = planes[pipe.plane];
        // resource_build_scaling_params: invalid input.
        if (plane.dst.width <= 0 || plane.dst.height <= 0 || plane.surface_width == 0 || plane.surface_height == 0) {
            return false;
        }
        // resource_get_odm_slice_src_rect (no left edge extra pixel for RGB).
        const uint32_t opp_head = topology_opp_head(t, i);
        const uint32_t slice = topology_odm_slice_index(t, i);
        const int32_t slice_width = static_cast<int32_t>(h_active / odm_count);
        Dcn32Rect odm_src{};
        odm_src.x = slice_width * static_cast<int32_t>(slice);
        odm_src.width = t.pipes[opp_head].next_odm == dcn32_no_pipe ?
                        static_cast<int32_t>(h_active) - slice_width * static_cast<int32_t>(odm_count - 1) :
                        slice_width;
        odm_src.y = 0;
        odm_src.height = static_cast<int32_t>(v_active);
        // calculate_recout: the plane clip in timing active (stream src == dst),
        // guarded by the stream dst, one MPC slice.
        const Dcn32Rect plane_clip = intersect(plane.dst, stream_dst);
        const Dcn32Rect overlap = intersect(plane_clip, odm_src);
        if (overlap.height > 0 && overlap.width > 0) {
            pipe.recout = shift(overlap, -odm_src.x, -odm_src.y);
        } else {
            pipe.recout = Dcn32Rect{};
        }
        // calculate_inits_and_viewports.
        const Dcn32Rect recout_clip = shift(pipe.recout, odm_src.x, odm_src.y);
        const Dcn32Rect overlap_dst = intersect(recout_clip, plane.dst);
        Dcn32Rect in_dst{};
        if (overlap_dst.width > 0 && overlap_dst.height > 0) {
            in_dst = shift(overlap_dst, -plane.dst.x, -plane.dst.y);
        }
        init_and_vp(in_dst.x, pipe.recout.width, static_cast<int32_t>(plane.surface_width), pipe.viewport.x,
                    pipe.viewport.width);
        init_and_vp(in_dst.y, pipe.recout.height, static_cast<int32_t>(plane.surface_height), pipe.viewport.y,
                    pipe.viewport.height);
        // Clamp minimum viewport size.
        if (pipe.viewport.height < dcn32_min_viewport_size) {
            pipe.viewport.height = dcn32_min_viewport_size;
        }
        if (pipe.viewport.width < dcn32_min_viewport_size) {
            pipe.viewport.width = dcn32_min_viewport_size;
        }
    }
    return true;
}

bool topology_transition_seamless(const Dcn32Topology& cur, const Dcn32Topology& next) noexcept {
    for (uint32_t i = 0; i < dcn32_topology_pipes; ++i) {
        if (topology_is_free_pipe(cur, i) || topology_is_free_pipe(next, i)) {
            continue;
        }
        if (topology_is_otg_master(cur, i)) {
            if (topology_is_otg_master(next, i)) {
                continue;
            }
        } else if (topology_is_opp_head(cur, i)) {
            if (topology_is_opp_head(next, i)) {
                continue;
            }
        } else if (topology_is_dpp_pipe(cur, i)) {
            if (topology_is_dpp_pipe(next, i) && cur.pipes[i].opp == next.pipes[i].opp) {
                continue;
            }
        }
        return false;
    }
    return true;
}

} // namespace rx7900::display

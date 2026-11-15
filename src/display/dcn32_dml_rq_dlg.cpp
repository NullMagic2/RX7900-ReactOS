/*
 * Copyright 2022 Advanced Micro Devices, Inc. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Authors: AMD
 */
/*
Module: rx7900.display.dcn32_dml_rq_dlg
Purpose: DML32 request, deadline and TTU register calculation.
Created: 2026-10-24 (adaptation; the upstream notice above is preserved)
Architecture: Part of the bounded C++17 port of the Linux v6.12 DML32 model behind
src/display/dcn32_bandwidth (one stream, up to four pipes). Identifiers, statement order and
arithmetic (including the float-typed dml_min/max/floor/ceil helpers and every
int/unsigned conversion) deliberately follow the pinned upstream text so each
function can be compared line by line with work/upstream-dcn-stream; this is a
documented exception to the snake_case rule, as in src/opengl/gfx11_registers.hpp.
Per-plane and per-pipe arrays hold the four DCN 3.2 pipes (DML32_NUM_PLANES; Linux
sizes them DC__NUM_DPP__MAX = 8) and per-state arrays DML32_NUM_STATES entries; debug printing is removed.
Upstream: dml/dcn32/display_rq_dlg_calc_32.c.
*/
#include "dcn32_dml_util.hpp"

namespace rx7900::display::dml32 {
static bool is_dual_plane(enum source_format_class source_format)
{
    bool ret_val = 0;

    if ((source_format == dm_420_12) || (source_format == dm_420_8) || (source_format == dm_420_10)
        || (source_format == dm_rgbe_alpha))
        ret_val = 1;

    return ret_val;
}

void dml32_rq_dlg_get_rq_reg(display_rq_regs_st *rq_regs,
        struct display_mode_lib *mode_lib,
        const display_e2e_pipe_params_st *e2e_pipe_param,
        const unsigned int num_pipes,
        const unsigned int pipe_idx)
{
    const display_pipe_source_params_st *src = &e2e_pipe_param[pipe_idx].pipe.src;
    bool dual_plane = is_dual_plane(static_cast<enum source_format_class>(src->source_format));
    double stored_swath_l_bytes;
    double stored_swath_c_bytes;
    bool is_phantom_pipe;
    uint32_t pixel_chunk_bytes = 0;
    uint32_t min_pixel_chunk_bytes = 0;
    uint32_t meta_chunk_bytes = 0;
    uint32_t min_meta_chunk_bytes = 0;
    uint32_t dpte_group_bytes = 0;
    uint32_t mpte_group_bytes = 0;

    uint32_t p1_pixel_chunk_bytes = 0;
    uint32_t p1_min_pixel_chunk_bytes = 0;
    uint32_t p1_meta_chunk_bytes = 0;
    uint32_t p1_min_meta_chunk_bytes = 0;
    uint32_t p1_dpte_group_bytes = 0;
    uint32_t p1_mpte_group_bytes = 0;

    unsigned int detile_buf_size_in_bytes;
    unsigned int detile_buf_plane1_addr;
    unsigned int pte_row_height_linear;

    *rq_regs = display_rq_regs_st{};

    pixel_chunk_bytes = get_pixel_chunk_size_in_kbyte(mode_lib, e2e_pipe_param, num_pipes) * 1024; // From VBA
    min_pixel_chunk_bytes = get_min_pixel_chunk_size_in_byte(mode_lib, e2e_pipe_param, num_pipes); // From VBA

    if (pixel_chunk_bytes == 64 * 1024)
        min_pixel_chunk_bytes = 0;

    meta_chunk_bytes = get_meta_chunk_size_in_kbyte(mode_lib, e2e_pipe_param, num_pipes) * 1024; // From VBA
    min_meta_chunk_bytes = get_min_meta_chunk_size_in_byte(mode_lib, e2e_pipe_param, num_pipes); // From VBA

    dpte_group_bytes = get_dpte_group_size_in_bytes(mode_lib, e2e_pipe_param, num_pipes, pipe_idx); // From VBA
    mpte_group_bytes = get_vm_group_size_in_bytes(mode_lib, e2e_pipe_param, num_pipes, pipe_idx); // From VBA

    p1_pixel_chunk_bytes = pixel_chunk_bytes;
    p1_min_pixel_chunk_bytes = min_pixel_chunk_bytes;
    p1_meta_chunk_bytes = meta_chunk_bytes;
    p1_min_meta_chunk_bytes = min_meta_chunk_bytes;
    p1_dpte_group_bytes = dpte_group_bytes;
    p1_mpte_group_bytes = mpte_group_bytes;

    if (static_cast<enum source_format_class>(src->source_format) == dm_rgbe_alpha)
        p1_pixel_chunk_bytes = get_alpha_pixel_chunk_size_in_kbyte(mode_lib, e2e_pipe_param, num_pipes) * 1024;

    rq_regs->rq_regs_l.chunk_size = dml_log2(pixel_chunk_bytes) - 10;
    rq_regs->rq_regs_c.chunk_size = dml_log2(p1_pixel_chunk_bytes) - 10;

    if (min_pixel_chunk_bytes == 0)
        rq_regs->rq_regs_l.min_chunk_size = 0;
    else
        rq_regs->rq_regs_l.min_chunk_size = dml_log2(min_pixel_chunk_bytes) - 8 + 1;

    if (p1_min_pixel_chunk_bytes == 0)
        rq_regs->rq_regs_c.min_chunk_size = 0;
    else
        rq_regs->rq_regs_c.min_chunk_size = dml_log2(p1_min_pixel_chunk_bytes) - 8 + 1;

    rq_regs->rq_regs_l.meta_chunk_size = dml_log2(meta_chunk_bytes) - 10;
    rq_regs->rq_regs_c.meta_chunk_size = dml_log2(p1_meta_chunk_bytes) - 10;

    if (min_meta_chunk_bytes == 0)
        rq_regs->rq_regs_l.min_meta_chunk_size = 0;
    else
        rq_regs->rq_regs_l.min_meta_chunk_size = dml_log2(min_meta_chunk_bytes) - 6 + 1;

    if (p1_min_meta_chunk_bytes == 0)
        rq_regs->rq_regs_c.min_meta_chunk_size = 0;
    else
        rq_regs->rq_regs_c.min_meta_chunk_size = dml_log2(p1_min_meta_chunk_bytes) - 6 + 1;

    rq_regs->rq_regs_l.dpte_group_size = dml_log2(dpte_group_bytes) - 6;
    rq_regs->rq_regs_l.mpte_group_size = dml_log2(mpte_group_bytes) - 6;
    rq_regs->rq_regs_c.dpte_group_size = dml_log2(p1_dpte_group_bytes) - 6;
    rq_regs->rq_regs_c.mpte_group_size = dml_log2(p1_mpte_group_bytes) - 6;

    detile_buf_size_in_bytes = get_det_buffer_size_kbytes(mode_lib, e2e_pipe_param, num_pipes, pipe_idx) * 1024;
    detile_buf_plane1_addr = 0;
    pte_row_height_linear = get_dpte_row_height_linear_l(mode_lib, e2e_pipe_param, num_pipes,
            pipe_idx);

    // (Linux: if (src->sw_mode == dm_sw_linear) ASSERT(pte_row_height_linear >= 8);)

    rq_regs->rq_regs_l.pte_row_height_linear = dml_floor(dml_log2(pte_row_height_linear), 1) - 3;

    if (dual_plane) {
        unsigned int p1_pte_row_height_linear = get_dpte_row_height_linear_c(mode_lib, e2e_pipe_param,
                num_pipes, pipe_idx);
        ;
        // (Linux: if (src->sw_mode == dm_sw_linear) ASSERT(p1_pte_row_height_linear >= 8);)

        rq_regs->rq_regs_c.pte_row_height_linear = dml_floor(dml_log2(p1_pte_row_height_linear), 1) - 3;
    }

    rq_regs->rq_regs_l.swath_height = dml_log2(get_swath_height_l(mode_lib, e2e_pipe_param, num_pipes, pipe_idx));
    rq_regs->rq_regs_c.swath_height = dml_log2(get_swath_height_c(mode_lib, e2e_pipe_param, num_pipes, pipe_idx));

    // FIXME: take the max between luma, chroma chunk size?
    // okay for now, as we are setting pixel_chunk_bytes to 8kb anyways
    if (pixel_chunk_bytes >= 32 * 1024 || (dual_plane && p1_pixel_chunk_bytes >= 32 * 1024)) { //32kb
        rq_regs->drq_expansion_mode = 0;
    } else {
        rq_regs->drq_expansion_mode = 2;
    }
    rq_regs->prq_expansion_mode = 1;
    rq_regs->mrq_expansion_mode = 1;
    rq_regs->crq_expansion_mode = 1;

    stored_swath_l_bytes = get_det_stored_buffer_size_l_bytes(mode_lib, e2e_pipe_param, num_pipes,
            pipe_idx);
    stored_swath_c_bytes = get_det_stored_buffer_size_c_bytes(mode_lib, e2e_pipe_param, num_pipes,
            pipe_idx);
    is_phantom_pipe = get_is_phantom_pipe(mode_lib, e2e_pipe_param, num_pipes, pipe_idx);

    // Note: detile_buf_plane1_addr is in unit of 1KB
    if (dual_plane) {
        if (is_phantom_pipe) {
            detile_buf_plane1_addr = ((1024.0 * 1024.0) / 2.0 / 1024.0); // half to chroma
        } else {
            if (stored_swath_l_bytes / stored_swath_c_bytes <= 1.5) {
                detile_buf_plane1_addr = (detile_buf_size_in_bytes / 2.0 / 1024.0); // half to chroma
            } else {
                detile_buf_plane1_addr =
                        dml_round_to_multiple(
                                static_cast<unsigned int>((2.0 * detile_buf_size_in_bytes) / 3.0),
                                1024, 0) / 1024.0; // 2/3 to luma
            }
        }
    }
    rq_regs->plane1_base_address = detile_buf_plane1_addr;
}

void dml32_rq_dlg_get_dlg_reg(struct display_mode_lib *mode_lib,
        display_dlg_regs_st *dlg_regs,
        display_ttu_regs_st *ttu_regs,
        display_e2e_pipe_params_st *e2e_pipe_param,
        const unsigned int num_pipes,
        const unsigned int pipe_idx)
{
    const display_pipe_source_params_st *src = &e2e_pipe_param[pipe_idx].pipe.src;
    const display_pipe_dest_params_st *dst = &e2e_pipe_param[pipe_idx].pipe.dest;
    const display_clocks_and_cfg_st *clks = &e2e_pipe_param[pipe_idx].clks_cfg;
    double refcyc_per_req_delivery_pre_cur0 = 0.;
    double refcyc_per_req_delivery_cur0 = 0.;
    double refcyc_per_req_delivery_pre_c = 0.;
    double refcyc_per_req_delivery_c = 0.;
    double refcyc_per_req_delivery_pre_l;
    double refcyc_per_req_delivery_l;
    double refcyc_per_line_delivery_pre_c = 0.;
    double refcyc_per_line_delivery_c = 0.;
    double refcyc_per_line_delivery_pre_l;
    double refcyc_per_line_delivery_l;
    double min_ttu_vblank;
    double vratio_pre_l;
    double vratio_pre_c;
    unsigned int min_dst_y_next_start;
    unsigned int htotal = dst->htotal;
    unsigned int hblank_end = dst->hblank_end;
    unsigned int vblank_end = dst->vblank_end;
    bool interlaced = dst->interlaced;
    double pclk_freq_in_mhz = dst->pixel_rate_mhz;
    unsigned int vready_after_vcount0;
    double refclk_freq_in_mhz = clks->refclk_mhz;
    double ref_freq_to_pix_freq = refclk_freq_in_mhz / pclk_freq_in_mhz;
    bool dual_plane = 0;
    unsigned int pipe_index_in_combine[DML32_NUM_PLANES];
    unsigned int dst_x_after_scaler;
    unsigned int dst_y_after_scaler;
    double dst_y_prefetch;
    double dst_y_per_vm_vblank;
    double dst_y_per_row_vblank;
    double dst_y_per_vm_flip;
    double dst_y_per_row_flip;
    double dst_y_per_pte_row_nom_l;
    double dst_y_per_pte_row_nom_c;
    double dst_y_per_meta_row_nom_l;
    double dst_y_per_meta_row_nom_c;
    double refcyc_per_pte_group_nom_l;
    double refcyc_per_pte_group_nom_c;
    double refcyc_per_pte_group_vblank_l;
    double refcyc_per_pte_group_vblank_c;
    double refcyc_per_pte_group_flip_l;
    double refcyc_per_pte_group_flip_c;
    double refcyc_per_meta_chunk_nom_l;
    double refcyc_per_meta_chunk_nom_c;
    double refcyc_per_meta_chunk_vblank_l;
    double refcyc_per_meta_chunk_vblank_c;
    double refcyc_per_meta_chunk_flip_l;
    double refcyc_per_meta_chunk_flip_c;

    *dlg_regs = display_dlg_regs_st{};
    *ttu_regs = display_ttu_regs_st{};

    dlg_regs->ref_freq_to_pix_freq = static_cast<unsigned int>(ref_freq_to_pix_freq * dml_pow(2, 19));
    dlg_regs->refcyc_per_htotal = static_cast<unsigned int>(ref_freq_to_pix_freq * static_cast<double>(htotal) * dml_pow(2, 8));
    dlg_regs->dlg_vblank_end = interlaced ? (vblank_end / 2) : vblank_end; // 15 bits

    min_ttu_vblank = get_min_ttu_vblank_in_us(mode_lib, e2e_pipe_param, num_pipes, pipe_idx); // From VBA
    min_dst_y_next_start = get_min_dst_y_next_start(mode_lib, e2e_pipe_param, num_pipes, pipe_idx);

    dual_plane = is_dual_plane(static_cast<enum source_format_class>(src->source_format));

    vready_after_vcount0 = get_vready_at_or_after_vsync(mode_lib, e2e_pipe_param, num_pipes,
            pipe_idx); // From VBA
    dlg_regs->vready_after_vcount0 = vready_after_vcount0;

    dst_x_after_scaler = dml_ceil(get_dst_x_after_scaler(mode_lib, e2e_pipe_param, num_pipes, pipe_idx), 1);
    dst_y_after_scaler = dml_ceil(get_dst_y_after_scaler(mode_lib, e2e_pipe_param, num_pipes, pipe_idx), 1);

    // do some adjustment on the dst_after scaler to account for odm combine mode

    // need to figure out which side of odm combine we're in
    if (dst->odm_combine == dm_odm_combine_mode_2to1 || dst->odm_combine == dm_odm_combine_mode_4to1) {
        // figure out which pipes go together
        bool visited[DML32_NUM_PLANES];
        unsigned int i, j, k;

        for (k = 0; k < num_pipes; ++k) {
            visited[k] = false;
            pipe_index_in_combine[k] = 0;
        }

        for (i = 0; i < num_pipes; i++) {
            if (e2e_pipe_param[i].pipe.src.is_hsplit && !visited[i]) {

                unsigned int grp = e2e_pipe_param[i].pipe.src.hsplit_grp;
                unsigned int grp_idx = 0;

                for (j = i; j < num_pipes; j++) {
                    if (e2e_pipe_param[j].pipe.src.hsplit_grp == grp
                        && e2e_pipe_param[j].pipe.src.is_hsplit && !visited[j]) {
                        pipe_index_in_combine[j] = grp_idx;
                        grp_idx++;
                        visited[j] = true;
                    }
                }
            }
        }
    }

    if (dst->odm_combine == dm_odm_combine_mode_disabled) {
        // FIXME how about ODM split??
        dlg_regs->refcyc_h_blank_end = static_cast<unsigned int>(static_cast<double>(hblank_end) * ref_freq_to_pix_freq);
    } else {
        if (dst->odm_combine == dm_odm_combine_mode_2to1 || dst->odm_combine == dm_odm_combine_mode_4to1) {
            // TODO: We should really check that 4to1 is supported before setting it to 4
            unsigned int odm_combine_factor = (dst->odm_combine == dm_odm_combine_mode_2to1 ? 2 : 4);
            unsigned int odm_pipe_index = pipe_index_in_combine[pipe_idx];

            dlg_regs->refcyc_h_blank_end = static_cast<unsigned int>((static_cast<double>(hblank_end)
                + odm_pipe_index * static_cast<double>(dst->hactive) / odm_combine_factor) * ref_freq_to_pix_freq);
        }
    }

    dst_y_prefetch = get_dst_y_prefetch(mode_lib, e2e_pipe_param, num_pipes, pipe_idx);        // From VBA
    // From VBA
    dst_y_per_vm_vblank = get_dst_y_per_vm_vblank(mode_lib, e2e_pipe_param, num_pipes, pipe_idx);
    // From VBA
    dst_y_per_row_vblank = get_dst_y_per_row_vblank(mode_lib, e2e_pipe_param, num_pipes, pipe_idx);
    dst_y_per_vm_flip = get_dst_y_per_vm_flip(mode_lib, e2e_pipe_param, num_pipes, pipe_idx);    // From VBA
    dst_y_per_row_flip = get_dst_y_per_row_flip(mode_lib, e2e_pipe_param, num_pipes, pipe_idx);  // From VBA

    // (Upstream only asserts the VM/row vblank lines against 32/16, or 100 for htotal <= 75.)

    vratio_pre_l = get_vratio_prefetch_l(mode_lib, e2e_pipe_param, num_pipes, pipe_idx);    // From VBA
    vratio_pre_c = get_vratio_prefetch_c(mode_lib, e2e_pipe_param, num_pipes, pipe_idx);    // From VBA

    // Active
    refcyc_per_line_delivery_pre_l = get_refcyc_per_line_delivery_pre_l_in_us(mode_lib, e2e_pipe_param, num_pipes, pipe_idx) * refclk_freq_in_mhz;   // From VBA
    refcyc_per_line_delivery_l = get_refcyc_per_line_delivery_l_in_us(mode_lib, e2e_pipe_param, num_pipes,
            pipe_idx) * refclk_freq_in_mhz;       // From VBA

    if (dual_plane) {
        refcyc_per_line_delivery_pre_c = get_refcyc_per_line_delivery_pre_c_in_us(mode_lib, e2e_pipe_param,
                num_pipes, pipe_idx) * refclk_freq_in_mhz;     // From VBA
        refcyc_per_line_delivery_c = get_refcyc_per_line_delivery_c_in_us(mode_lib, e2e_pipe_param, num_pipes,
                pipe_idx) * refclk_freq_in_mhz; // From VBA

    }

    if (src->dynamic_metadata_enable && src->gpuvm)
        dlg_regs->refcyc_per_vm_dmdata = get_refcyc_per_vm_dmdata_in_us(mode_lib, e2e_pipe_param, num_pipes,
                pipe_idx) * refclk_freq_in_mhz; // From VBA

    dlg_regs->dmdata_dl_delta = get_dmdata_dl_delta_in_us(mode_lib, e2e_pipe_param, num_pipes, pipe_idx)
        * refclk_freq_in_mhz; // From VBA

    refcyc_per_req_delivery_pre_l = get_refcyc_per_req_delivery_pre_l_in_us(mode_lib, e2e_pipe_param, num_pipes, pipe_idx) * refclk_freq_in_mhz; // From VBA
    refcyc_per_req_delivery_l = get_refcyc_per_req_delivery_l_in_us(mode_lib, e2e_pipe_param, num_pipes,
            pipe_idx) * refclk_freq_in_mhz;     // From VBA

    if (dual_plane) {
        refcyc_per_req_delivery_pre_c = get_refcyc_per_req_delivery_pre_c_in_us(mode_lib, e2e_pipe_param,
                num_pipes, pipe_idx) * refclk_freq_in_mhz;  // From VBA
        refcyc_per_req_delivery_c = get_refcyc_per_req_delivery_c_in_us(mode_lib, e2e_pipe_param, num_pipes,
                pipe_idx) * refclk_freq_in_mhz;      // From VBA

    }

    // TTU - Cursor
    if (src->num_cursors > 0) {
        refcyc_per_req_delivery_pre_cur0 = get_refcyc_per_cursor_req_delivery_pre_in_us(mode_lib,
                e2e_pipe_param, num_pipes, pipe_idx) * refclk_freq_in_mhz;  // From VBA
        refcyc_per_req_delivery_cur0 = get_refcyc_per_cursor_req_delivery_in_us(mode_lib, e2e_pipe_param,
                num_pipes, pipe_idx) * refclk_freq_in_mhz;      // From VBA

    }

    // Assign to register structures
    dlg_regs->min_dst_y_next_start = min_dst_y_next_start * dml_pow(2, 2);

    dlg_regs->dst_y_after_scaler = dst_y_after_scaler; // in terms of line
    dlg_regs->refcyc_x_after_scaler = dst_x_after_scaler * ref_freq_to_pix_freq; // in terms of refclk
    dlg_regs->dst_y_prefetch = static_cast<unsigned int>(dst_y_prefetch * dml_pow(2, 2));
    dlg_regs->dst_y_per_vm_vblank = static_cast<unsigned int>(dst_y_per_vm_vblank * dml_pow(2, 2));
    dlg_regs->dst_y_per_row_vblank = static_cast<unsigned int>(dst_y_per_row_vblank * dml_pow(2, 2));
    dlg_regs->dst_y_per_vm_flip = static_cast<unsigned int>(dst_y_per_vm_flip * dml_pow(2, 2));
    dlg_regs->dst_y_per_row_flip = static_cast<unsigned int>(dst_y_per_row_flip * dml_pow(2, 2));

    dlg_regs->vratio_prefetch = static_cast<unsigned int>(vratio_pre_l * dml_pow(2, 19));
    dlg_regs->vratio_prefetch_c = static_cast<unsigned int>(vratio_pre_c * dml_pow(2, 19));

    dlg_regs->refcyc_per_vm_group_vblank = get_refcyc_per_vm_group_vblank_in_us(mode_lib, e2e_pipe_param,
            num_pipes, pipe_idx) * refclk_freq_in_mhz;               // From VBA
    dlg_regs->refcyc_per_vm_group_flip = get_refcyc_per_vm_group_flip_in_us(mode_lib, e2e_pipe_param, num_pipes,
            pipe_idx) * refclk_freq_in_mhz;                 // From VBA
    dlg_regs->refcyc_per_vm_req_vblank = get_refcyc_per_vm_req_vblank_in_us(mode_lib, e2e_pipe_param, num_pipes,
            pipe_idx) * refclk_freq_in_mhz * dml_pow(2, 10);                 // From VBA
    dlg_regs->refcyc_per_vm_req_flip = get_refcyc_per_vm_req_flip_in_us(mode_lib, e2e_pipe_param, num_pipes,
            pipe_idx) * refclk_freq_in_mhz * dml_pow(2, 10);  // From VBA

    // From VBA
    dst_y_per_pte_row_nom_l = get_dst_y_per_pte_row_nom_l(mode_lib, e2e_pipe_param, num_pipes, pipe_idx);
    // From VBA
    dst_y_per_pte_row_nom_c = get_dst_y_per_pte_row_nom_c(mode_lib, e2e_pipe_param, num_pipes, pipe_idx);
    // From VBA
    dst_y_per_meta_row_nom_l = get_dst_y_per_meta_row_nom_l(mode_lib, e2e_pipe_param, num_pipes, pipe_idx);
    // From VBA
    dst_y_per_meta_row_nom_c = get_dst_y_per_meta_row_nom_c(mode_lib, e2e_pipe_param, num_pipes, pipe_idx);

    refcyc_per_pte_group_nom_l = get_refcyc_per_pte_group_nom_l_in_us(mode_lib, e2e_pipe_param, num_pipes,
            pipe_idx) * refclk_freq_in_mhz;         // From VBA
    refcyc_per_pte_group_nom_c = get_refcyc_per_pte_group_nom_c_in_us(mode_lib, e2e_pipe_param, num_pipes,
            pipe_idx) * refclk_freq_in_mhz;         // From VBA
    refcyc_per_pte_group_vblank_l = get_refcyc_per_pte_group_vblank_l_in_us(mode_lib, e2e_pipe_param,
            num_pipes, pipe_idx) * refclk_freq_in_mhz;      // From VBA
    refcyc_per_pte_group_vblank_c = get_refcyc_per_pte_group_vblank_c_in_us(mode_lib, e2e_pipe_param,
            num_pipes, pipe_idx) * refclk_freq_in_mhz;      // From VBA
    refcyc_per_pte_group_flip_l = get_refcyc_per_pte_group_flip_l_in_us(mode_lib, e2e_pipe_param, num_pipes,
            pipe_idx) * refclk_freq_in_mhz;        // From VBA
    refcyc_per_pte_group_flip_c = get_refcyc_per_pte_group_flip_c_in_us(mode_lib, e2e_pipe_param, num_pipes,
            pipe_idx) * refclk_freq_in_mhz;        // From VBA

    refcyc_per_meta_chunk_nom_l = get_refcyc_per_meta_chunk_nom_l_in_us(mode_lib, e2e_pipe_param, num_pipes,
            pipe_idx) * refclk_freq_in_mhz;        // From VBA
    refcyc_per_meta_chunk_nom_c = get_refcyc_per_meta_chunk_nom_c_in_us(mode_lib, e2e_pipe_param, num_pipes,
            pipe_idx) * refclk_freq_in_mhz;        // From VBA
    refcyc_per_meta_chunk_vblank_l = get_refcyc_per_meta_chunk_vblank_l_in_us(mode_lib, e2e_pipe_param,
            num_pipes, pipe_idx) * refclk_freq_in_mhz;     // From VBA
    refcyc_per_meta_chunk_vblank_c = get_refcyc_per_meta_chunk_vblank_c_in_us(mode_lib, e2e_pipe_param,
            num_pipes, pipe_idx) * refclk_freq_in_mhz;     // From VBA
    refcyc_per_meta_chunk_flip_l = get_refcyc_per_meta_chunk_flip_l_in_us(mode_lib, e2e_pipe_param,
            num_pipes, pipe_idx) * refclk_freq_in_mhz;       // From VBA
    refcyc_per_meta_chunk_flip_c = get_refcyc_per_meta_chunk_flip_c_in_us(mode_lib, e2e_pipe_param,
            num_pipes, pipe_idx) * refclk_freq_in_mhz;       // From VBA

    dlg_regs->dst_y_per_pte_row_nom_l = dst_y_per_pte_row_nom_l * dml_pow(2, 2);
    dlg_regs->dst_y_per_pte_row_nom_c = dst_y_per_pte_row_nom_c * dml_pow(2, 2);
    dlg_regs->dst_y_per_meta_row_nom_l = dst_y_per_meta_row_nom_l * dml_pow(2, 2);
    dlg_regs->dst_y_per_meta_row_nom_c = dst_y_per_meta_row_nom_c * dml_pow(2, 2);
    dlg_regs->refcyc_per_pte_group_nom_l = refcyc_per_pte_group_nom_l;
    dlg_regs->refcyc_per_pte_group_nom_c = refcyc_per_pte_group_nom_c;
    dlg_regs->refcyc_per_pte_group_vblank_l = refcyc_per_pte_group_vblank_l;
    dlg_regs->refcyc_per_pte_group_vblank_c = refcyc_per_pte_group_vblank_c;
    dlg_regs->refcyc_per_pte_group_flip_l = refcyc_per_pte_group_flip_l;
    dlg_regs->refcyc_per_pte_group_flip_c = refcyc_per_pte_group_flip_c;
    dlg_regs->refcyc_per_meta_chunk_nom_l = refcyc_per_meta_chunk_nom_l;
    dlg_regs->refcyc_per_meta_chunk_nom_c = refcyc_per_meta_chunk_nom_c;
    dlg_regs->refcyc_per_meta_chunk_vblank_l = refcyc_per_meta_chunk_vblank_l;
    dlg_regs->refcyc_per_meta_chunk_vblank_c = refcyc_per_meta_chunk_vblank_c;
    dlg_regs->refcyc_per_meta_chunk_flip_l = refcyc_per_meta_chunk_flip_l;
    dlg_regs->refcyc_per_meta_chunk_flip_c = refcyc_per_meta_chunk_flip_c;
    dlg_regs->refcyc_per_line_delivery_pre_l = static_cast<unsigned int>(dml_floor(refcyc_per_line_delivery_pre_l, 1));
    dlg_regs->refcyc_per_line_delivery_l = static_cast<unsigned int>(dml_floor(refcyc_per_line_delivery_l, 1));
    dlg_regs->refcyc_per_line_delivery_pre_c = static_cast<unsigned int>(dml_floor(refcyc_per_line_delivery_pre_c, 1));
    dlg_regs->refcyc_per_line_delivery_c = static_cast<unsigned int>(dml_floor(refcyc_per_line_delivery_c, 1));

    dlg_regs->chunk_hdl_adjust_cur0 = 3;
    dlg_regs->dst_y_offset_cur0 = 0;
    dlg_regs->chunk_hdl_adjust_cur1 = 3;
    dlg_regs->dst_y_offset_cur1 = 0;

    dlg_regs->dst_y_delta_drq_limit = 0x7fff; // off

    ttu_regs->refcyc_per_req_delivery_pre_l = static_cast<unsigned int>(refcyc_per_req_delivery_pre_l * dml_pow(2, 10));
    ttu_regs->refcyc_per_req_delivery_l = static_cast<unsigned int>(refcyc_per_req_delivery_l * dml_pow(2, 10));
    ttu_regs->refcyc_per_req_delivery_pre_c = static_cast<unsigned int>(refcyc_per_req_delivery_pre_c * dml_pow(2, 10));
    ttu_regs->refcyc_per_req_delivery_c = static_cast<unsigned int>(refcyc_per_req_delivery_c * dml_pow(2, 10));
    ttu_regs->refcyc_per_req_delivery_pre_cur0 =
            static_cast<unsigned int>(refcyc_per_req_delivery_pre_cur0 * dml_pow(2, 10));
    ttu_regs->refcyc_per_req_delivery_cur0 = static_cast<unsigned int>(refcyc_per_req_delivery_cur0 * dml_pow(2, 10));
    ttu_regs->refcyc_per_req_delivery_pre_cur1 = 0;
    ttu_regs->refcyc_per_req_delivery_cur1 = 0;
    ttu_regs->qos_level_low_wm = 0;

    ttu_regs->qos_level_high_wm = static_cast<unsigned int>(4.0 * static_cast<double>(htotal) * ref_freq_to_pix_freq);

    ttu_regs->qos_level_flip = 14;
    ttu_regs->qos_level_fixed_l = 8;
    ttu_regs->qos_level_fixed_c = 8;
    ttu_regs->qos_level_fixed_cur0 = 8;
    ttu_regs->qos_ramp_disable_l = 0;
    ttu_regs->qos_ramp_disable_c = 0;
    ttu_regs->qos_ramp_disable_cur0 = 0;
    ttu_regs->min_ttu_vblank = min_ttu_vblank * refclk_freq_in_mhz;

    // CHECK for HW registers' range, assert or clamp
    if (dlg_regs->refcyc_per_vm_group_vblank >= static_cast<unsigned int>(dml_pow(2, 23)))
        dlg_regs->refcyc_per_vm_group_vblank = dml_pow(2, 23) - 1;

    if (dlg_regs->refcyc_per_vm_group_flip >= static_cast<unsigned int>(dml_pow(2, 23)))
        dlg_regs->refcyc_per_vm_group_flip = dml_pow(2, 23) - 1;

    if (dlg_regs->refcyc_per_vm_req_vblank >= static_cast<unsigned int>(dml_pow(2, 23)))
        dlg_regs->refcyc_per_vm_req_vblank = dml_pow(2, 23) - 1;

    if (dlg_regs->refcyc_per_vm_req_flip >= static_cast<unsigned int>(dml_pow(2, 23)))
        dlg_regs->refcyc_per_vm_req_flip = dml_pow(2, 23) - 1;

    if (dual_plane) {
        if (dlg_regs->dst_y_per_pte_row_nom_c >= static_cast<unsigned int>(dml_pow(2, 17))) {
            // FIXME what so special about chroma, can we just assert?
        }
    }

    if (dlg_regs->refcyc_per_pte_group_nom_l >= static_cast<unsigned int>(dml_pow(2, 23)))
        dlg_regs->refcyc_per_pte_group_nom_l = dml_pow(2, 23) - 1;
    if (dual_plane) {
        if (dlg_regs->refcyc_per_pte_group_nom_c >= static_cast<unsigned int>(dml_pow(2, 23)))
            dlg_regs->refcyc_per_pte_group_nom_c = dml_pow(2, 23) - 1;
    }
    if (dual_plane) {
    }

    if (dlg_regs->refcyc_per_meta_chunk_nom_l >= static_cast<unsigned int>(dml_pow(2, 23)))
        dlg_regs->refcyc_per_meta_chunk_nom_l = dml_pow(2, 23) - 1;
    if (dual_plane) {
        if (dlg_regs->refcyc_per_meta_chunk_nom_c >= static_cast<unsigned int>(dml_pow(2, 23)))
            dlg_regs->refcyc_per_meta_chunk_nom_c = dml_pow(2, 23) - 1;
    }
}

} // namespace rx7900::display::dml32

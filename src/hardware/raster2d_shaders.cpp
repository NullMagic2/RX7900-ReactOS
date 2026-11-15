/*
Module: rx7900.hardware.raster2d_shaders
Purpose: Generate the raster2d GFX11 compute kernels and their argument helpers.
Created: 2026-10-22
Architecture: See raster2d_shaders.hpp. Pixel semantics reproduce the pinned
ReactOS 0.4.16 (822e864) win32ss/gdi engine (GPL-2.0-or-later behaviour, no code
copied): dib/dib.c DIB_DoRop, eng/bitblt.c BltMask and EngMaskBitBlt (8-bpp
coverage), dib/stretchblt.c DIB_XXBPP_StretchBlt, dib/dib32bpp.c
DIB_32BPP_TransparentBlt and DIB_32BPP_AlphaBlend, eng/lineto.c and
eng/gradient.c. The CPU planners in raster2d.cpp reduce each engine operation to
per-rectangle dispatches whose per-pixel arithmetic these kernels implement.
*/
#include "raster2d_shaders.hpp"

namespace rx7900::hardware {
namespace {

using namespace gfx11_isa;

constexpr uint32_t s_x0 = 4;          // left + 64 * group_x
constexpr uint32_t s_in_range = 5;    // VALU: x < right
constexpr uint32_t s_y = 6;
constexpr uint32_t s_row = 7;         // y * destination pitch
constexpr uint32_t s_write = 8;       // lanes to store (SALU only)
constexpr uint32_t s_temp = 9;        // SCC producer for uniform branches
constexpr uint32_t s_source_y = 12;
constexpr uint32_t s_source_row_valid = 13;
constexpr uint32_t s_source_row = 14;
constexpr uint32_t s_source_x_valid = 15;   // VALU
constexpr uint32_t s_source_valid = 80;     // SALU: both coordinates inside the source

Operand arg(uint32_t index) noexcept { return s(raster2d_first_arg_sgpr + index); }

uint32_t vgpr_field(uint32_t vgprs) noexcept { return vgprs / 8 - 1; }

void prologue(Assembler& a, uint32_t blocks) noexcept {
    for (uint32_t block = 0; block < blocks; ++block) {
        a.smem(Smem::s_load_b512, raster2d_first_arg_sgpr + 16 * block, 0, null_sgpr(),
               static_cast<int32_t>(64 * block));
    }

    a.vop2(Vop2::v_and_b32, v(0), literal(0x3ff), v(0));
    a.waitcnt(waitcnt_max_vm, waitcnt_max_exp, 0);
}

// x in v1, destination byte offset in v2, y in s_y, x < right lanes in s_write.
void pixel_position(Assembler& a, uint32_t left, uint32_t top, uint32_t right,
                    uint32_t pitch) noexcept {
    a.sop2(Sop2::s_lshl_b32, s(s_x0), s(2), imm(6));
    a.sop2(Sop2::s_add_i32, s(s_x0), s(s_x0), arg(left));
    a.vop2(Vop2::v_add_nc_u32, v(1), s(s_x0), v(0));
    a.vop3_cmp(Vopc::v_cmp_gt_i32, s(s_in_range), arg(right), v(1));
    a.sop2(Sop2::s_add_i32, s(s_y), arg(top), s(3));
    a.sop2(Sop2::s_mul_i32, s(s_row), s(s_y), arg(pitch));
    a.vop2(Vop2::v_lshlrev_b32, v(2), imm(2), v(1));
    a.vop2(Vop2::v_add_nc_u32, v(2), s(s_row), v(2));
    a.sop1(Sop1::s_mov_b32, s(s_write), s(s_in_range));
}

// q = floor(n / d) with Granlund-Montgomery parameters (vector).
void vector_divide(Assembler& a, uint32_t q, uint32_t n, uint32_t t, Operand magic,
                   Operand shift1, Operand shift2) noexcept {
    a.vop3(Vop3::v_mul_hi_u32, v(t), v(n), magic, Operand{});
    a.vop2(Vop2::v_sub_nc_u32, v(q), v(n), v(t));
    a.vop2(Vop2::v_lshrrev_b32, v(q), shift1, v(q));
    a.vop2(Vop2::v_add_nc_u32, v(q), v(t), v(q));
    a.vop2(Vop2::v_lshrrev_b32, v(q), shift2, v(q));
}

// Source coordinate mapping of rop/alpha: sy (scalar), source row offset, sx in v7
// and the lanes whose (sx, sy) lies inside the source in s_source_valid.
void source_mapping(Assembler& a, uint32_t pitch) noexcept {
    const uint32_t my = rop_args::map_y;
    const uint32_t mx = rop_args::map_x;

    a.sop2(Sop2::s_sub_i32, s(10), s(s_y), arg(my + rop_args::map_dref));
    a.sop2(Sop2::s_mul_i32, s(10), s(10), arg(my + rop_args::map_mag));
    a.sop2(Sop2::s_mul_hi_u32, s(11), s(10), arg(my + rop_args::map_magic));
    a.sop2(Sop2::s_sub_u32, s(s_source_y), s(10), s(11));
    a.sop2(Sop2::s_lshr_b32, s(s_source_y), s(s_source_y), arg(my + rop_args::map_shift1));
    a.sop2(Sop2::s_add_u32, s(s_source_y), s(11), s(s_source_y));
    a.sop2(Sop2::s_lshr_b32, s(s_source_y), s(s_source_y), arg(my + rop_args::map_shift2));
    a.sop2(Sop2::s_xor_b32, s(s_source_y), s(s_source_y), arg(my + rop_args::map_neg));
    a.sop2(Sop2::s_sub_u32, s(s_source_y), s(s_source_y), arg(my + rop_args::map_neg));
    a.sop2(Sop2::s_add_i32, s(s_source_y), s(s_source_y), arg(my + rop_args::map_base));
    a.sopc(Sopc::s_cmp_lt_u32, s(s_source_y), arg(rop_args::source_height));
    a.sop2(Sop2::s_cselect_b32, s(s_source_row_valid), imm(-1), imm(0));
    a.sop2(Sop2::s_mul_i32, s(s_source_row), s(s_source_y), arg(pitch));

    a.vop2(Vop2::v_subrev_nc_u32, v(5), arg(mx + rop_args::map_dref), v(1));
    a.vop3(Vop3::v_mul_lo_u32, v(5), v(5), arg(mx + rop_args::map_mag), Operand{});
    vector_divide(a, 7, 5, 6, arg(mx + rop_args::map_magic), arg(mx + rop_args::map_shift1),
                  arg(mx + rop_args::map_shift2));
    a.vop2(Vop2::v_xor_b32, v(7), arg(mx + rop_args::map_neg), v(7));
    a.vop2(Vop2::v_subrev_nc_u32, v(7), arg(mx + rop_args::map_neg), v(7));
    a.vop2(Vop2::v_add_nc_u32, v(7), arg(mx + rop_args::map_base), v(7));
    a.vop3_cmp(Vopc::v_cmp_gt_u32, s(s_source_x_valid), arg(rop_args::source_width), v(7));
    a.sop2(Sop2::s_and_b32, s(s_source_valid), s(s_source_x_valid), s(s_source_row_valid));
}

// dst = rop(P = v13, S = v4, D = v3) with minterm masks at arg(base .. base + 7).
void rop_evaluate(Assembler& a, uint32_t dst, uint32_t base) noexcept {
    a.vop3(Vop3::v_bfi_b32, v(14), v(3), arg(base + 1), arg(base + 0));
    a.vop3(Vop3::v_bfi_b32, v(15), v(3), arg(base + 3), arg(base + 2));
    a.vop3(Vop3::v_bfi_b32, v(16), v(3), arg(base + 5), arg(base + 4));
    a.vop3(Vop3::v_bfi_b32, v(17), v(3), arg(base + 7), arg(base + 6));
    a.vop3(Vop3::v_bfi_b32, v(14), v(4), v(15), v(14));
    a.vop3(Vop3::v_bfi_b32, v(15), v(4), v(17), v(16));
    a.vop3(Vop3::v_bfi_b32, v(dst), v(13), v(15), v(14));
}

// Uniform branch to label when (flags & bits) == 0.
void branch_unless(Assembler& a, uint32_t flags, uint32_t bits, Label label) noexcept {
    a.sop2(Sop2::s_and_b32, s(s_temp), arg(flags), literal(bits));
    a.branch(Sopp::s_cbranch_scc0, label);
}

// MSB-first 1-bpp bit -> 0 / 0xffffffff in v(dst); v(addr) holds the byte offset,
// v(bit) the pixel index in the row; descriptor at sgpr srsrc.
void mono_bit(Assembler& a, uint32_t dst, uint32_t address, uint32_t bit, uint32_t srsrc) noexcept {
    a.mubuf(Mubuf::buffer_load_u8, v(dst), v(address), srsrc, imm(0), 0);
    a.vop2(Vop2::v_and_b32, v(bit), imm(7), v(bit));
    a.vop2(Vop2::v_sub_nc_u32, v(bit), imm(7), v(bit));
    a.waitcnt(0, waitcnt_max_exp, waitcnt_max_lgkm);
    a.vop2(Vop2::v_lshrrev_b32, v(dst), v(bit), v(dst));
    a.vop3(Vop3::v_bfe_i32, v(dst), v(dst), imm(0), imm(1));
}

void store_and_end(Assembler& a, uint32_t value) noexcept {
    a.sop2(Sop2::s_and_b32, exec_lo(), exec_lo(), s(s_write));
    a.mubuf(Mubuf::buffer_store_b32, v(value), v(2), raster2d_first_arg_sgpr, imm(0), 0);
    a.sopk(Sopk::s_waitcnt_vscnt, null_sgpr(), 0);
    a.endpgm();
}

// Pack channels v(r), v(g), v(b) (already 8-bit) as r << 16 | g << 8 | b.
void pack_rgb(Assembler& a, uint32_t dst, uint32_t r, uint32_t g, uint32_t b) noexcept {
    a.vop3(Vop3::v_lshl_or_b32, v(dst), v(r), imm(8), v(g));
    a.vop3(Vop3::v_lshl_or_b32, v(dst), v(dst), imm(8), v(b));
}

// The row half of the dib/stretchblt.c mask test (sy in s_source_y): s87 = all
// lanes when mask row sy + mask_dy exists, s91 = that row's byte offset.
constexpr uint32_t s_mask_row_valid = 87;
constexpr uint32_t s_mask_row = 91;
constexpr uint32_t s_mask_drawn = 89;    // stretch_mask result for this wave's own pixels

void stretch_mask_row(Assembler& a) noexcept {
    using namespace rop_args;
    a.sop2(Sop2::s_add_i32, s(88), s(s_source_y), arg(mask_dy));
    a.sopc(Sopc::s_cmp_lt_u32, s(88), arg(stretch_mask_height));
    a.sop2(Sop2::s_cselect_b32, s(s_mask_row_valid), imm(-1), imm(0));
    a.sop2(Sop2::s_mul_i32, s(s_mask_row), s(88), arg(mask_pitch));
}

// The column half for the source sample sx in v(sample): out = lanes whose mask
// pixel (sx + mask_dx, sy + mask_dy) exists and is clear (CanDraw stays TRUE).
// Clobbers v(t0..t2) and s(ts).
void stretch_mask_lanes(Assembler& a, uint32_t sample, uint32_t out, uint32_t t0, uint32_t t1, uint32_t t2,
                        uint32_t ts) noexcept {
    using namespace rop_args;
    a.vop2(Vop2::v_add_nc_u32, v(t0), arg(mask_dx), v(sample));
    a.vop3_cmp(Vopc::v_cmp_gt_u32, s(ts), arg(stretch_mask_width), v(t0));
    a.sop2(Sop2::s_and_b32, s(out), s(s_mask_row_valid), s(ts));
    a.vop2(Vop2::v_lshrrev_b32, v(t1), imm(3), v(t0));
    a.vop2(Vop2::v_add_nc_u32, v(t1), s(s_mask_row), v(t1));
    mono_bit(a, t2, t1, t0, raster2d_first_arg_sgpr + mask);
    a.vop3_cmp(Vopc::v_cmp_eq_u32, s(ts), imm(0), v(t2));
    a.sop2(Sop2::s_and_b32, s(out), s(out), s(ts));
}

// dib/stretchblt.c DIB_XXBPP_StretchBlt mask test of the mapped source sample
// (sx in v7, sy in s_source_y): lanes whose mask sample lies outside the mask or
// has its bit set are removed from s_write; s_mask_drawn keeps the lane result.
// Clobbers v20..v22, s87..s91.
void stretch_mask(Assembler& a) noexcept {
    stretch_mask_row(a);
    stretch_mask_lanes(a, 7, s_mask_drawn, 20, 21, 22, 90);
    a.sop2(Sop2::s_and_b32, s(s_write), s(s_write), s(s_mask_drawn));
}

// The x half of source_mapping for the destination column in v(x): v(out) =
// base + sign * floor((x - dref) * mag / den). Clobbers v(t0), v(t1).
void map_x_lanes(Assembler& a, uint32_t out, uint32_t x, uint32_t t0, uint32_t t1) noexcept {
    const uint32_t mx = rop_args::map_x;
    a.vop2(Vop2::v_subrev_nc_u32, v(t0), arg(mx + rop_args::map_dref), v(x));
    a.vop3(Vop3::v_mul_lo_u32, v(t0), v(t0), arg(mx + rop_args::map_mag), Operand{});
    vector_divide(a, out, t0, t1, arg(mx + rop_args::map_magic), arg(mx + rop_args::map_shift1),
                  arg(mx + rop_args::map_shift2));
    a.vop2(Vop2::v_xor_b32, v(out), arg(mx + rop_args::map_neg), v(out));
    a.vop2(Vop2::v_subrev_nc_u32, v(out), arg(mx + rop_args::map_neg), v(out));
    a.vop2(Vop2::v_add_nc_u32, v(out), arg(mx + rop_args::map_base), v(out));
}

// Running PatternX of DIB_XXBPP_StretchBlt with a mask: v12 = dref + the number of
// pixels in [dref, x) of this row whose mask sample is drawable. Pixels before
// this wave are counted 32 at a time (s_bcnt1 over a lane mask of the same mask
// test), this wave's lower lanes with v_mbcnt_lo over s_mask_drawn. Every wave
// runs the loop under its full initial EXEC; the bound is wave-uniform.
void running_phase(Assembler& a) noexcept {
    using namespace rop_args;
    constexpr uint32_t s_wave_x0 = 93, s_drawn = 94, s_cursor = 95, s_chunk = 96, s_before = 97,
                       s_count = 98;
    const Label loop = a.make_label();
    const Label done = a.make_label();

    a.vop2(Vop2::v_and_b32, v(24), imm(31), v(0));          // lane index in the wave
    a.readfirstlane(s(s_wave_x0), v(1));                   // x of lane 0
    a.sop1(Sop1::s_mov_b32, s(s_drawn), imm(0));
    a.sop1(Sop1::s_mov_b32, s(s_cursor), arg(map_x + map_dref));
    a.bind(loop);
    a.sopc(Sopc::s_cmp_lt_i32, s(s_cursor), s(s_wave_x0));
    a.branch(Sopp::s_cbranch_scc0, done);
    a.vop2(Vop2::v_add_nc_u32, v(25), s(s_cursor), v(24));
    map_x_lanes(a, 27, 25, 26, 28);
    stretch_mask_lanes(a, 27, s_chunk, 28, 29, 30, s_count);
    a.vop3_cmp(Vopc::v_cmp_gt_i32, s(s_before), s(s_wave_x0), v(25));
    a.sop2(Sop2::s_and_b32, s(s_chunk), s(s_chunk), s(s_before));
    a.sop1(Sop1::s_bcnt1_i32_b32, s(s_count), s(s_chunk));
    a.sop2(Sop2::s_add_i32, s(s_drawn), s(s_drawn), s(s_count));
    a.sop2(Sop2::s_add_i32, s(s_cursor), s(s_cursor), imm(32));
    a.branch(Sopp::s_branch, loop);
    a.bind(done);
    a.vop3(Vop3::v_mbcnt_lo_u32_b32, v(12), s(s_mask_drawn), s(s_drawn), Operand{});
    a.vop2(Vop2::v_add_nc_u32, v(12), arg(map_x + map_dref), v(12));
}

// Translated source pixel (sx in v7, row offset in s_source_row) -> S in v4:
// the table entry of an index8, rgb16 or rgb24 pixel (raster2d_shaders.hpp
// rop_args translated-source layout). Clobbers v8..v13.
void translated_source(Assembler& a) noexcept {
    using namespace rop_args;
    const uint32_t srsrc = raster2d_first_arg_sgpr + source;
    const Label not_index8 = a.make_label();
    const Label not_rgb16 = a.make_label();
    const Label done = a.make_label();

    branch_unless(a, flags, flag_source_index8, not_index8);
    a.vop2(Vop2::v_add_nc_u32, v(8), s(s_source_row), v(7));
    a.vop2(Vop2::v_add_nc_u32, v(8), arg(source_offset), v(8));
    a.mubuf(Mubuf::buffer_load_u8, v(9), v(8), srsrc, imm(0), 0);
    a.waitcnt(0, waitcnt_max_exp, waitcnt_max_lgkm);
    a.vop2(Vop2::v_lshlrev_b32, v(9), imm(2), v(9));
    a.mubuf(Mubuf::buffer_load_b32, v(4), v(9), srsrc, imm(0), 0);
    a.waitcnt(0, waitcnt_max_exp, waitcnt_max_lgkm);
    a.branch(Sopp::s_branch, done);

    a.bind(not_index8);
    branch_unless(a, flags, flag_source_rgb16, not_rgb16);
    a.vop2(Vop2::v_lshlrev_b32, v(8), imm(1), v(7));
    a.vop2(Vop2::v_add_nc_u32, v(8), s(s_source_row), v(8));
    a.vop2(Vop2::v_add_nc_u32, v(8), arg(source_offset), v(8));
    a.vop2(Vop2::v_and_b32, v(10), imm(2), v(8));
    a.vop2(Vop2::v_lshlrev_b32, v(10), imm(3), v(10));     // 0 or 16: the halfword's bit offset
    a.vop2(Vop2::v_and_b32, v(8), literal(0xfffffffcu), v(8));
    a.mubuf(Mubuf::buffer_load_b32, v(9), v(8), srsrc, imm(0), 0);
    a.waitcnt(0, waitcnt_max_exp, waitcnt_max_lgkm);
    a.vop3(Vop3::v_bfe_u32, v(9), v(9), v(10), imm(16));
    a.vop2(Vop2::v_lshlrev_b32, v(9), imm(2), v(9));
    a.mubuf(Mubuf::buffer_load_b32, v(4), v(9), srsrc, imm(0), 0);
    a.waitcnt(0, waitcnt_max_exp, waitcnt_max_lgkm);
    a.branch(Sopp::s_branch, done);

    a.bind(not_rgb16);
    a.vop2(Vop2::v_mul_u32_u24, v(8), imm(3), v(7));
    a.vop2(Vop2::v_add_nc_u32, v(8), s(s_source_row), v(8));
    a.vop2(Vop2::v_add_nc_u32, v(8), arg(source_offset), v(8));
    a.mubuf(Mubuf::buffer_load_u8, v(9), v(8), srsrc, imm(0), 0);
    a.mubuf(Mubuf::buffer_load_u8, v(10), v(8), srsrc, imm(0), 1);
    a.mubuf(Mubuf::buffer_load_u8, v(11), v(8), srsrc, imm(0), 2);
    a.waitcnt(0, waitcnt_max_exp, waitcnt_max_lgkm);
    a.vop2(Vop2::v_lshlrev_b32, v(9), imm(2), v(9));
    a.vop2(Vop2::v_lshlrev_b32, v(10), imm(2), v(10));
    a.vop2(Vop2::v_lshlrev_b32, v(11), imm(2), v(11));
    a.mubuf(Mubuf::buffer_load_b32, v(4), v(9), srsrc, imm(0), 0);
    a.mubuf(Mubuf::buffer_load_b32, v(12), v(10), srsrc, imm(0), 1024);
    a.mubuf(Mubuf::buffer_load_b32, v(13), v(11), srsrc, imm(0), 2048);
    a.waitcnt(0, waitcnt_max_exp, waitcnt_max_lgkm);
    a.vop2(Vop2::v_or_b32, v(4), v(12), v(4));
    a.vop2(Vop2::v_or_b32, v(4), v(13), v(4));
    a.bind(done);
}

constexpr uint32_t rop_vgprs = 32;

void build_rop(Assembler& a) noexcept {
    using namespace rop_args;
    const Label no_destination = a.make_label();
    const Label no_source = a.make_label();
    const Label mono = a.make_label();
    const Label source_done = a.make_label();
    const Label no_key = a.make_label();
    const Label solid_pattern = a.make_label();
    const Label pattern_done = a.make_label();
    const Label store = a.make_label();
    const Label no_stretch_mask = a.make_label();
    const Label translated = a.make_label();
    const Label plain_phase = a.make_label();
    const Label phase_done = a.make_label();

    prologue(a, 4);
    pixel_position(a, left, top, right, destination_pitch);

    a.vop1(Vop1::v_mov_b32, v(3), imm(0));
    branch_unless(a, flags, flag_load_destination, no_destination);
    a.mubuf(Mubuf::buffer_load_b32, v(3), v(2), raster2d_first_arg_sgpr + destination, imm(0), 0);
    a.bind(no_destination);

    a.vop1(Vop1::v_mov_b32, v(4), imm(0));
    branch_unless(a, flags, flag_source_color | flag_source_mono | flag_source_index8 | flag_source_rgb16 |
                                flag_source_rgb24, no_source);
    source_mapping(a, source_pitch);
    branch_unless(a, flags, flag_stretch_mask, no_stretch_mask);
    stretch_mask(a);
    a.bind(no_stretch_mask);
    branch_unless(a, flags, flag_source_color, mono);
    a.vop2(Vop2::v_lshlrev_b32, v(8), imm(2), v(7));
    a.vop2(Vop2::v_add_nc_u32, v(8), s(s_source_row), v(8));
    a.mubuf(Mubuf::buffer_load_b32, v(4), v(8), raster2d_first_arg_sgpr + source, imm(0), 0);
    a.waitcnt(0, waitcnt_max_exp, waitcnt_max_lgkm);
    a.branch(Sopp::s_branch, source_done);
    a.bind(mono);
    branch_unless(a, flags, flag_source_mono, translated);
    a.vop2(Vop2::v_lshrrev_b32, v(8), imm(3), v(7));
    a.vop2(Vop2::v_add_nc_u32, v(8), s(s_source_row), v(8));
    a.vop1(Vop1::v_mov_b32, v(10), v(7));
    mono_bit(a, 9, 8, 10, raster2d_first_arg_sgpr + source);
    a.vop3(Vop3::v_bfi_b32, v(4), v(9), arg(mono1), arg(mono0));
    a.branch(Sopp::s_branch, source_done);
    a.bind(translated);
    translated_source(a);
    a.bind(source_done);
    // dib/stretchblt.c: a source pixel outside the source bitmap reads as 0.
    a.vop3(Vop3::v_cndmask_b32_e64, v(4), imm(0), v(4), s(s_source_valid));
    branch_unless(a, flags, flag_color_key, no_key);
    a.vop2(Vop2::v_and_b32, v(11), arg(key_mask), v(4));
    a.vop3_cmp(Vopc::v_cmp_ne_u32, s(81), arg(key), v(11));
    a.sop2(Sop2::s_and_b32, s(s_write), s(s_write), s(81));
    a.bind(no_key);
    branch_unless(a, flags, flag_skip_outside_source, no_source);
    a.sop2(Sop2::s_and_b32, s(s_write), s(s_write), s(s_source_valid));
    a.bind(no_source);

    branch_unless(a, flags, flag_pattern_tile, solid_pattern);
    branch_unless(a, flags, flag_pattern_running, plain_phase);
    running_phase(a);
    a.vop2(Vop2::v_subrev_nc_u32, v(12), arg(brush_x), v(12));
    a.branch(Sopp::s_branch, phase_done);
    a.bind(plain_phase);
    a.vop2(Vop2::v_subrev_nc_u32, v(12), arg(brush_x), v(1));
    a.bind(phase_done);
    a.vop2(Vop2::v_and_b32, v(12), imm(7), v(12));
    a.vop2(Vop2::v_lshlrev_b32, v(12), imm(2), v(12));
    a.sop2(Sop2::s_sub_i32, s(82), s(s_y), arg(brush_y));
    a.sop2(Sop2::s_and_b32, s(82), s(82), imm(7));
    a.sop2(Sop2::s_lshl_b32, s(82), s(82), imm(5));
    a.vop2(Vop2::v_add_nc_u32, v(12), s(82), v(12));
    a.mubuf(Mubuf::buffer_load_b32, v(13), v(12), raster2d_first_arg_sgpr + pattern, imm(0), 0);
    a.branch(Sopp::s_branch, pattern_done);
    a.bind(solid_pattern);
    a.vop1(Vop1::v_mov_b32, v(13), arg(solid));
    a.bind(pattern_done);
    a.waitcnt(0, waitcnt_max_exp, waitcnt_max_lgkm);

    rop_evaluate(a, 18, foreground);
    branch_unless(a, flags, flag_mask, store);
    rop_evaluate(a, 19, background);
    a.vop2(Vop2::v_add_nc_u32, v(20), arg(mask_dx), v(1));
    a.sop2(Sop2::s_add_i32, s(83), s(s_y), arg(mask_dy));
    a.sop2(Sop2::s_mul_i32, s(84), s(83), arg(mask_pitch));
    a.vop2(Vop2::v_lshrrev_b32, v(21), imm(3), v(20));
    a.vop2(Vop2::v_add_nc_u32, v(21), s(84), v(21));
    mono_bit(a, 22, 21, 20, raster2d_first_arg_sgpr + mask);
    // eng/bitblt.c BltMask: mask bit 1 selects the foreground (low) ROP byte.
    a.vop3(Vop3::v_bfi_b32, v(18), v(22), v(18), v(19));
    a.bind(store);
    store_and_end(a, 18);
}

// x / 255 for x <= 65535 as (x * 0x8081) >> 23 (exhaustively verified by tests).
void divide_255(Assembler& a, uint32_t value) noexcept {
    a.vop2(Vop2::v_mul_u32_u24, v(value), literal(0x8081), v(value));
    a.vop2(Vop2::v_lshrrev_b32, v(value), imm(23), v(value));
}

constexpr uint32_t alpha_vgprs = 24;

void build_alpha(Assembler& a) noexcept {
    using namespace alpha_args;
    const Label coverage = a.make_label();
    const Label constant = a.make_label();
    const Label have_alpha = a.make_label();
    const Label store = a.make_label();

    prologue(a, 4);
    pixel_position(a, left, top, right, destination_pitch);
    a.mubuf(Mubuf::buffer_load_b32, v(3), v(2), raster2d_first_arg_sgpr + destination, imm(0), 0);
    source_mapping(a, source_pitch);
    a.sop2(Sop2::s_and_b32, s(s_write), s(s_write), s(s_source_valid));
    a.sop2(Sop2::s_and_b32, s(s_temp), arg(flags), imm(static_cast<int32_t>(flag_coverage8)));
    a.branch(Sopp::s_cbranch_scc1, coverage);

    // dib/dib32bpp.c DIB_32BPP_AlphaBlend.
    a.vop2(Vop2::v_lshlrev_b32, v(8), imm(2), v(7));
    a.vop2(Vop2::v_add_nc_u32, v(8), s(s_source_row), v(8));
    a.mubuf(Mubuf::buffer_load_b32, v(4), v(8), raster2d_first_arg_sgpr + source, imm(0), 0);
    a.waitcnt(0, waitcnt_max_exp, waitcnt_max_lgkm);

    for (uint32_t channel = 0; channel < 4; ++channel) {
        const uint32_t value = 10 + channel;
        a.vop3(Vop3::v_bfe_u32, v(value), v(4), imm(static_cast<int32_t>(8 * channel)), imm(8));
        a.vop2(Vop2::v_mul_u32_u24, v(value), arg(constant_alpha), v(value));
        divide_255(a, value);
    }

    branch_unless(a, flags, flag_source_alpha, constant);
    a.vop1(Vop1::v_mov_b32, v(14), v(13));
    a.branch(Sopp::s_branch, have_alpha);
    a.bind(constant);
    a.vop1(Vop1::v_mov_b32, v(14), arg(constant_alpha));
    a.bind(have_alpha);
    a.vop2(Vop2::v_sub_nc_u32, v(14), literal(255), v(14));

    for (uint32_t channel = 0; channel < 4; ++channel) {
        a.vop3(Vop3::v_bfe_u32, v(15), v(3), imm(static_cast<int32_t>(8 * channel)), imm(8));
        a.vop2(Vop2::v_mul_u32_u24, v(15), v(14), v(15));
        divide_255(a, 15);
        a.vop2(Vop2::v_add_nc_u32, v(15), v(10 + channel), v(15));
        a.vop2(Vop2::v_min_u32, v(16 + channel), literal(255), v(15));
    }

    a.vop3(Vop3::v_lshl_or_b32, v(20), v(19), imm(8), v(18));
    a.vop3(Vop3::v_lshl_or_b32, v(20), v(20), imm(8), v(17));
    a.vop3(Vop3::v_lshl_or_b32, v(20), v(20), imm(8), v(16));
    a.branch(Sopp::s_branch, store);

    // eng/bitblt.c EngMaskBitBlt with an 8-bpp mask: 0 keeps the destination,
    // 0xff stores the brush colour, otherwise bg + ((m * (fg - bg)) >> 8) per
    // colour channel with an arithmetic shift, alpha byte 0.
    a.bind(coverage);
    a.vop2(Vop2::v_add_nc_u32, v(8), s(s_source_row), v(7));
    a.mubuf(Mubuf::buffer_load_u8, v(9), v(8), raster2d_first_arg_sgpr + source, imm(0), 0);
    a.waitcnt(0, waitcnt_max_exp, waitcnt_max_lgkm);
    a.vop3_cmp(Vopc::v_cmp_ne_u32, s(85), imm(0), v(9));
    a.sop2(Sop2::s_and_b32, s(s_write), s(s_write), s(85));

    for (uint32_t channel = 0; channel < 3; ++channel) {
        a.vop3(Vop3::v_bfe_u32, v(14), arg(solid), imm(static_cast<int32_t>(8 * channel)), imm(8));
        a.vop3(Vop3::v_bfe_u32, v(15), v(3), imm(static_cast<int32_t>(8 * channel)), imm(8));
        a.vop2(Vop2::v_sub_nc_u32, v(14), v(14), v(15));
        a.vop3(Vop3::v_mul_lo_u32, v(14), v(14), v(9), Operand{});
        a.vop2(Vop2::v_ashrrev_i32, v(14), imm(8), v(14));
        a.vop2(Vop2::v_add_nc_u32, v(14), v(15), v(14));
        a.vop3(Vop3::v_bfe_u32, v(16 + channel), v(14), imm(0), imm(8));
    }

    pack_rgb(a, 20, 18, 17, 16);
    a.vop3_cmp(Vopc::v_cmp_eq_u32, s(86), literal(255), v(9));
    a.vop1(Vop1::v_mov_b32, v(21), arg(solid));
    a.vop3(Vop3::v_cndmask_b32_e64, v(20), v(20), v(21), s(86));
    a.bind(store);
    store_and_end(a, 20);
}

constexpr uint32_t gradient_vgprs = 16;

// n = floor((k * delta + bias) / divisor); c = color +/- n; 8-bit result in v(out).
void gradient_channel(Assembler& a, uint32_t out, Operand delta, Operand bias, Operand magic,
                      Operand shift1, Operand shift2, Operand negative, Operand color) noexcept {
    a.vop3(Vop3::v_mul_lo_u32, v(4), v(3), delta, Operand{});
    a.vop2(Vop2::v_add_nc_u32, v(4), bias, v(4));
    vector_divide(a, 5, 4, 6, magic, shift1, shift2);
    a.vop2(Vop2::v_xor_b32, v(5), negative, v(5));
    a.vop2(Vop2::v_subrev_nc_u32, v(5), negative, v(5));
    a.vop2(Vop2::v_add_nc_u32, v(5), color, v(5));
    a.vop3(Vop3::v_bfe_u32, v(out), v(5), imm(0), imm(8));
}

void build_gradient_rect(Assembler& a) noexcept {
    using namespace gradient_args;
    const Label vertical = a.make_label();
    const Label have_step = a.make_label();

    prologue(a, 3);
    pixel_position(a, left, top, right, destination_pitch);
    branch_unless(a, flags, flag_horizontal, vertical);
    a.vop2(Vop2::v_subrev_nc_u32, v(3), arg(origin), v(1));
    a.branch(Sopp::s_branch, have_step);
    a.bind(vertical);
    a.sop2(Sop2::s_sub_i32, s(10), s(s_y), arg(origin));
    a.vop1(Vop1::v_mov_b32, v(3), s(10));
    a.bind(have_step);

    for (uint32_t channel = 0; channel < 3; ++channel) {
        gradient_channel(a, 10 + channel, arg(delta + channel), arg(bias), arg(magic), arg(shift1),
                         arg(shift2), arg(negative + channel), arg(color + channel));
    }

    pack_rgb(a, 13, 10, 11, 12);
    store_and_end(a, 13);
}

constexpr uint32_t triangle_vgprs = 24;

void build_gradient_triangle(Assembler& a) noexcept {
    using namespace triangle_args;
    constexpr uint32_t span_base = 48;

    prologue(a, 2);
    pixel_position(a, left, top, right, destination_pitch);
    a.sop2(Sop2::s_mul_i32, s(s_temp), s(3), arg(row_stride));

    for (uint32_t span = 0; span < raster2d_triangle_spans; ++span) {
        a.smem(Smem::s_load_b512, span_base + 16 * span, raster2d_first_arg_sgpr + table_low,
               s(s_temp), static_cast<int32_t>(64 * span));
    }

    a.waitcnt(waitcnt_max_vm, waitcnt_max_exp, 0);
    a.vop1(Vop1::v_mov_b32, v(20), imm(0));
    a.sop1(Sop1::s_mov_b32, s(10), imm(0));

    for (uint32_t span = 0; span < raster2d_triangle_spans; ++span) {
        const uint32_t base = span_base + 16 * span;
        const uint32_t covered = 102 + span;
        a.sopc(Sopc::s_cmp_eq_u32, s(base + span_reverse), imm(0));
        a.sop2(Sop2::s_cselect_b32, s(11), s(base + span_low), s(base + span_high));
        a.vop2(Vop2::v_subrev_nc_u32, v(3), s(11), v(1));
        a.vop2(Vop2::v_xor_b32, v(3), s(base + span_reverse), v(3));
        a.vop2(Vop2::v_subrev_nc_u32, v(3), s(base + span_reverse), v(3));
        a.vop3_cmp(Vopc::v_cmp_le_i32, s(96 + 2 * span), s(base + span_low), v(1));
        a.vop3_cmp(Vopc::v_cmp_ge_i32, s(97 + 2 * span), s(base + span_high), v(1));
        a.sop2(Sop2::s_and_b32, s(covered), s(96 + 2 * span), s(97 + 2 * span));
        a.sop2(Sop2::s_or_b32, s(10), s(10), s(covered));

        for (uint32_t channel = 0; channel < 3; ++channel) {
            gradient_channel(a, 10 + channel, s(base + span_delta + channel), s(base + span_bias),
                             s(base + span_magic), s(base + span_shift1), s(base + span_shift2),
                             s(base + span_negative + channel), s(base + span_color + channel));
        }

        pack_rgb(a, 13, 10, 11, 12);
        // eng/gradient.c: a later FILLLINE of the same row overwrites earlier ones.
        a.vop3(Vop3::v_cndmask_b32_e64, v(20), v(20), v(13), s(covered));
    }

    a.sop2(Sop2::s_and_b32, s(s_write), s(s_write), s(10));
    store_and_end(a, 20);
}

constexpr uint32_t line_vgprs = 16;

void build_line(Assembler& a) noexcept {
    using namespace line_args;
    const Label unstyled = a.make_label();
    const Label no_pattern = a.make_label();
    const Label no_destination = a.make_label();

    prologue(a, 3);
    a.sop2(Sop2::s_lshl_b32, s(s_x0), s(2), imm(6));
    a.sop2(Sop2::s_add_i32, s(s_x0), s(s_x0), arg(first_step));
    a.vop2(Vop2::v_add_nc_u32, v(1), s(s_x0), v(0));
    a.vop3_cmp(Vopc::v_cmp_gt_u32, s(s_in_range), arg(end_step), v(1));
    a.sop1(Sop1::s_mov_b32, s(s_write), s(s_in_range));

    // eng/lineto.c Bresenham: after k steps the minor axis advanced
    // floor((delta / 2 + k * minor_delta) / delta) times.
    a.vop3(Vop3::v_mul_lo_u32, v(2), v(1), arg(minor_delta), Operand{});
    a.vop2(Vop2::v_add_nc_u32, v(2), arg(error), v(2));
    vector_divide(a, 3, 2, 4, arg(magic), arg(shift1), arg(shift2));
    a.vop3(Vop3::v_mul_lo_u32, v(5), v(1), arg(x_major), Operand{});
    a.vop3(Vop3::v_mul_lo_u32, v(6), v(3), arg(x_minor), Operand{});
    a.vop2(Vop2::v_add_nc_u32, v(5), v(5), v(6));
    a.vop2(Vop2::v_add_nc_u32, v(5), arg(x_base), v(5));
    a.vop3(Vop3::v_mul_lo_u32, v(6), v(1), arg(y_major), Operand{});
    a.vop3(Vop3::v_mul_lo_u32, v(7), v(3), arg(y_minor), Operand{});
    a.vop2(Vop2::v_add_nc_u32, v(6), v(6), v(7));
    a.vop2(Vop2::v_add_nc_u32, v(6), arg(y_base), v(6));
    a.vop3_cmp(Vopc::v_cmp_le_i32, s(10), arg(left), v(5));
    a.vop3_cmp(Vopc::v_cmp_gt_i32, s(11), arg(right), v(5));
    a.vop3_cmp(Vopc::v_cmp_le_i32, s(12), arg(top), v(6));
    a.vop3_cmp(Vopc::v_cmp_gt_i32, s(13), arg(bottom), v(6));
    a.sop2(Sop2::s_and_b32, s(s_write), s(s_write), s(10));
    a.sop2(Sop2::s_and_b32, s(s_write), s(s_write), s(11));
    a.sop2(Sop2::s_and_b32, s(s_write), s(s_write), s(12));
    a.sop2(Sop2::s_and_b32, s(s_write), s(s_write), s(13));

    branch_unless(a, flags, flag_styled, unstyled);
    a.vop2(Vop2::v_lshrrev_b32, v(7), imm(3), v(1));
    a.mubuf(Mubuf::buffer_load_u8, v(8), v(7), raster2d_first_arg_sgpr + style, imm(0), 0);
    a.vop2(Vop2::v_and_b32, v(9), imm(7), v(1));
    a.waitcnt(0, waitcnt_max_exp, waitcnt_max_lgkm);
    a.vop3(Vop3::v_bfe_u32, v(8), v(8), v(9), imm(1));
    a.vop3_cmp(Vopc::v_cmp_ne_u32, s(14), imm(0), v(8));
    a.sop2(Sop2::s_and_b32, s(s_write), s(s_write), s(14));
    a.bind(unstyled);

    a.vop3(Vop3::v_mul_lo_u32, v(2), v(6), arg(destination_pitch), Operand{});
    a.vop2(Vop2::v_lshlrev_b32, v(10), imm(2), v(5));
    a.vop2(Vop2::v_add_nc_u32, v(2), v(2), v(10));
    // P: the pen colour, or the 8x8 tile texel at ((x - brush_x) & 7,
    // (y - brush_y) & 7) of the pixel this lane plots.
    a.vop1(Vop1::v_mov_b32, v(11), arg(color));
    branch_unless(a, flags, flag_pattern, no_pattern);
    a.vop2(Vop2::v_subrev_nc_u32, v(7), arg(brush_x), v(5));
    a.vop2(Vop2::v_and_b32, v(7), imm(7), v(7));
    a.vop2(Vop2::v_lshlrev_b32, v(7), imm(2), v(7));
    a.vop2(Vop2::v_subrev_nc_u32, v(8), arg(brush_y), v(6));
    a.vop2(Vop2::v_and_b32, v(8), imm(7), v(8));
    a.vop2(Vop2::v_lshlrev_b32, v(8), imm(5), v(8));
    a.vop2(Vop2::v_add_nc_u32, v(7), v(7), v(8));
    a.mubuf(Mubuf::buffer_load_b32, v(11), v(7), raster2d_first_arg_sgpr + pattern, imm(0), 0);
    a.bind(no_pattern);
    a.vop1(Vop1::v_mov_b32, v(12), imm(0));
    branch_unless(a, flags, flag_load_destination, no_destination);
    a.mubuf(Mubuf::buffer_load_b32, v(12), v(2), raster2d_first_arg_sgpr + destination, imm(0), 0);
    a.bind(no_destination);
    a.waitcnt(0, waitcnt_max_exp, waitcnt_max_lgkm);
    // ROP over P (v11) and D with S = 0.
    a.vop3(Vop3::v_bfi_b32, v(13), v(12), arg(rop + 1), arg(rop + 0));
    a.vop3(Vop3::v_bfi_b32, v(14), v(12), arg(rop + 5), arg(rop + 4));
    a.vop3(Vop3::v_bfi_b32, v(15), v(11), v(14), v(13));
    store_and_end(a, 15);
}

struct KernelSpec final {
    void (*build)(Assembler&) noexcept;
    uint32_t vgprs;
    uint32_t args_dwords;
};

constexpr KernelSpec specs[raster2d_kernel_count] = {
    {build_rop, rop_vgprs, 64},
    {build_alpha, alpha_vgprs, 64},
    {build_gradient_rect, gradient_vgprs, 48},
    {build_gradient_triangle, triangle_vgprs, 32},
    {build_line, line_vgprs, 48},
};

} // namespace

bool raster2d_build_code(sdma::CommandBuffer code, Raster2dCodeImage& image) noexcept {
    image = {};

    if (code.size() < raster2d_code_bytes) {
        return false;
    }

    uint32_t offset = 0;

    for (uint32_t index = 0; index < raster2d_kernel_count; ++index) {
        Assembler a(code.subbuffer(offset, raster2d_code_bytes - offset));
        specs[index].build(a);
        a.pad_code_end(raster2d_code_alignment);

        if (!a.finish()) {
            return false;
        }

        Raster2dKernelInfo& info = image.kernels[index];
        info.offset_bytes = offset;
        info.size_bytes = a.dwords() * 4;
        info.vgprs = specs[index].vgprs;
        info.args_dwords = specs[index].args_dwords;
        info.rsrc1 = vgpr_field(info.vgprs) | (rsrc1_float_mode << rsrc1_float_mode_shift) |
                     rsrc1_dx10_clamp | rsrc1_wgp_mode | rsrc1_mem_ordered;
        info.rsrc2 = (raster2d_user_sgprs << rsrc2_user_sgpr_shift) | rsrc2_tgid_x_en |
                     rsrc2_tgid_y_en;
        info.rsrc3 = 0;
        offset += info.size_bytes;
    }

    if (offset + raster2d_code_prefetch_pad > raster2d_code_bytes) {
        return false;
    }

    Assembler pad(code.subbuffer(offset, raster2d_code_prefetch_pad));

    for (uint32_t i = 0; i < raster2d_code_prefetch_pad / 4; ++i) {
        pad.sopp(Sopp::s_code_end, 0);
    }

    if (!pad.finish()) {
        return false;
    }

    image.total_bytes = offset + raster2d_code_prefetch_pad;
    return true;
}

void raster2d_buffer_descriptor(uint64_t address, uint32_t bytes, uint32_t (&words)[4]) noexcept {
    words[0] = static_cast<uint32_t>(address);
    words[1] = static_cast<uint32_t>(address >> 32) & 0xffffu;
    words[2] = bytes;
    words[3] = raster2d_raw_descriptor_word3;
}

void raster2d_rop_masks(uint32_t rop3, uint32_t (&masks)[8]) noexcept {
    for (uint32_t index = 0; index < 8; ++index) {
        masks[index] = ((rop3 >> index) & 1u) != 0 ? 0xffffffffu : 0u;
    }
}

Raster2dDivisor raster2d_divisor(uint32_t divisor) noexcept {
    const uint64_t d = divisor == 0 ? 1u : divisor;
    uint32_t l = 0;

    while ((uint64_t{1} << l) < d) {
        ++l;
    }

    const uint64_t magic = (((uint64_t{1} << l) - d) << 32) / d + 1;
    return {static_cast<uint32_t>(magic), l < 1 ? l : 1u, l > 0 ? l - 1 : 0u};
}

uint32_t raster2d_divide(uint32_t numerator, const Raster2dDivisor& divisor) noexcept {
    const uint32_t t = static_cast<uint32_t>((uint64_t{numerator} * divisor.magic) >> 32);
    return (t + ((numerator - t) >> divisor.shift1)) >> divisor.shift2;
}

} // namespace rx7900::hardware

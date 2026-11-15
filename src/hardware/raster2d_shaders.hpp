/*
Module: rx7900.hardware.raster2d_shaders
Purpose: GFX11 wave32 compute kernels for the GDI operations SDMA cannot perform:
general ROP3/ROP4 over pattern, source and destination, 8x8 pattern brushes,
monochrome expansion and glyph masks, colour-key blits, AlphaBlend, nearest
stretching (mirrored by a negative map sign, with the stretch mask rule), styled
or pattern cosmetic lines and integer gradient fills, on 32-bpp linear surfaces. Generated at initialization with the in-tree gfx11_isa encoder.
Created: 2026-10-22
Architecture: Pure, allocation-free. raster2d_build_code writes every kernel into a
caller-provided code image (one 256-byte aligned entry per kernel, s_code_end
padded, plus a trailing instruction-prefetch pad). raster2d.cpp fills the
per-dispatch argument blocks defined here and programs the COMPUTE_PGM_* values
reported in Raster2dKernelInfo. tests/raster2d_tests.cpp executes the generated
binary in an ISA interpreter against CPU references of the pinned GDI engine.

Kernel ABI (all kernels):
  COMPUTE_NUM_THREAD_X/Y/Z = 64/1/1 (two wave32 waves per workgroup, no LDS,
  no scratch, no barriers). User SGPRs s[0:1] = 64-bit GPU address of the
  dispatch's argument block (COMPUTE_USER_DATA_0/1, 64-byte aligned); s2 =
  workgroup id X, s3 = workgroup id Y (TGID_X_EN, TGID_Y_EN). v0 = work-item id;
  bits [9:0] are used (GFX11 packs Y/Z ids above them; the kernels mask them off).
  Argument dword i is loaded with s_load_b512 into s(16 + i). Pixel x = left +
  64 * group_x + id, y = top + group_y. Every lane computes under the initial
  EXEC; the lanes to write are accumulated in an SGPR and EXEC is narrowed once,
  immediately before the only buffer_store, so no VALU instruction follows an EXEC
  write (GFX11 VALU partial-forwarding hazard) and no SGPR read by a VALU as a lane
  mask is later written by SALU (GFX11 VALU mask-write hazard).
  Memory is accessed only through raw buffer descriptors (V#, OOB_SELECT_RAW):
  the hardware returns zero for loads and drops stores at offset + size >
  NUM_RECORDS, so a kernel cannot touch bytes outside the surfaces it was given.

ROP3 choice: one generic evaluator instead of 256 generated kernels. The rop code
becomes eight per-minterm masks (0 or 0xffffffff, rop bit i for index
i = P<<2 | S<<1 | D, the DIB_DoRop/ROP4 convention) and a 7-instruction
v_bfi_b32 Shannon multiplexer tree selects the result bit for every one of the 32
bits of a pixel in parallel. It costs 7 VALU per pixel for any code, keeps one
kernel (and one validated instruction sequence) for all 256 codes and both ROP4
halves, and uniform flags skip loads of unused operands.

Integer division by a dispatch-uniform divisor d (stretch mapping, line minor axis,
gradient steps) uses the Granlund-Montgomery round-down method: with
l = ceil(log2 d), m = floor(2^32 (2^l - d) / d) + 1, t = mulhi(n, m),
q = (t + ((n - t) >> min(l, 1))) >> max(l - 1, 0), exact for every 32-bit n and
1 <= d < 2^32. (Granlund and Montgomery, "Division by Invariant Integers using
Multiplication", PLDI 1994, figure 4.1.)
*/
#pragma once

#include "gfx11_isa.hpp"

namespace rx7900::hardware {

enum class Raster2dKernel : uint32_t { rop, alpha, gradient_rect, gradient_triangle, line };
constexpr uint32_t raster2d_kernel_count = 5;

constexpr uint32_t raster2d_workgroup_threads = 64;
constexpr uint32_t raster2d_wave_size = 32;
constexpr uint32_t raster2d_user_sgprs = 2;
constexpr uint32_t raster2d_first_arg_sgpr = 16;
constexpr uint32_t raster2d_code_alignment = 256;
// GFX11 instruction prefetch can read past the end of the last kernel; the image
// ends with this many bytes of s_code_end (LLVM's s_code_end padding policy).
constexpr uint32_t raster2d_code_prefetch_pad = 256;
constexpr uint32_t raster2d_code_bytes = 16384;
// Argument blocks are 256 bytes (64 dwords) and 64-byte aligned.
constexpr uint32_t raster2d_args_bytes = 256;
constexpr uint32_t raster2d_args_alignment = 64;

// GDI 8x8 pattern tile: 64 BGRX dwords, row-major, placed right after the
// argument block of a rop dispatch.
constexpr uint32_t raster2d_pattern_size = 8;
constexpr uint32_t raster2d_pattern_bytes = 64 * 4;

// Gradient triangle row table: three spans of 16 dwords per row.
constexpr uint32_t raster2d_triangle_spans = 3;
constexpr uint32_t raster2d_span_dwords = 16;
constexpr uint32_t raster2d_triangle_row_bytes = raster2d_triangle_spans * raster2d_span_dwords * 4;

// gc_11_0_0_sh_mask.h COMPUTE_PGM_RSRC1/2 fields used for every kernel (Mesa 24.2
// radv_postprocess_binary_config for GFX11 compute: FLOAT_MODE 0xc0 (fp16/fp64
// denormals), DX10_CLAMP, WGP_MODE, MEM_ORDERED; USER_SGPR, TGID_X/Y_EN).
constexpr uint32_t rsrc1_float_mode_shift = 12;
constexpr uint32_t rsrc1_float_mode = 0xc0;
constexpr uint32_t rsrc1_dx10_clamp = 1u << 21;
constexpr uint32_t rsrc1_wgp_mode = 1u << 29;
constexpr uint32_t rsrc1_mem_ordered = 1u << 30;
constexpr uint32_t rsrc2_user_sgpr_shift = 1;
constexpr uint32_t rsrc2_tgid_x_en = 1u << 7;
constexpr uint32_t rsrc2_tgid_y_en = 1u << 8;

struct Raster2dKernelInfo final {
    uint32_t offset_bytes;  // from the start of the code image, 256-byte aligned
    uint32_t size_bytes;    // including s_code_end padding to 256 bytes
    uint32_t vgprs;         // allocated VGPRs (multiple of 8)
    uint32_t args_dwords;   // argument dwords the kernel loads
    uint32_t rsrc1;
    uint32_t rsrc2;
    uint32_t rsrc3;
};

struct Raster2dCodeImage final {
    Raster2dKernelInfo kernels[raster2d_kernel_count];
    uint32_t total_bytes;   // all kernels plus raster2d_code_prefetch_pad
};

// Generates all kernels into code (at least raster2d_code_bytes). False when any
// encoding fails; code may then hold partial output that must not be used.
bool raster2d_build_code(sdma::CommandBuffer code, Raster2dCodeImage& image) noexcept;

// GFX11 raw buffer descriptor (Mesa ac_build_raw_buffer_descriptor): word0 base
// [31:0]; word1 base [47:32], stride 0, swizzle 0; word2 NUM_RECORDS = bytes;
// word3 DST_SEL XYZW, FORMAT 32_FLOAT (22), OOB_SELECT_RAW (3), type 0.
constexpr uint32_t raster2d_raw_descriptor_word3 =
    4u | (5u << 3) | (6u << 6) | (7u << 9) | (22u << 12) | (3u << 28);
void raster2d_buffer_descriptor(uint64_t address, uint32_t bytes, uint32_t (&words)[4]) noexcept;

// rop3 -> eight minterm masks for the v_bfi_b32 evaluator.
void raster2d_rop_masks(uint32_t rop3, uint32_t (&masks)[8]) noexcept;

// Granlund-Montgomery parameters for divisor d >= 1 (d = 0 is treated as 1).
struct Raster2dDivisor final {
    uint32_t magic;
    uint32_t shift1;
    uint32_t shift2;
};
Raster2dDivisor raster2d_divisor(uint32_t divisor) noexcept;
// The exact quotient the kernel computes with those parameters (CPU mirror).
uint32_t raster2d_divide(uint32_t numerator, const Raster2dDivisor& divisor) noexcept;

// Argument layouts: dword indices inside the 256-byte block.
namespace rop_args {
constexpr uint32_t destination = 0;   // V# (4 dwords)
constexpr uint32_t source = 4;        // V#
constexpr uint32_t mask = 8;          // V#
constexpr uint32_t pattern = 12;      // V# of the 8x8 tile
constexpr uint32_t left = 16, top = 17, right = 18, destination_pitch = 19;
constexpr uint32_t flags = 20, source_pitch = 21, mask_pitch = 22, solid = 23;
constexpr uint32_t foreground = 24;   // 8 minterm masks (mask bit 1 / no mask)
constexpr uint32_t background = 32;   // 8 minterm masks (mask bit 0)
constexpr uint32_t mono0 = 40, mono1 = 41, key = 42, key_mask = 43;
constexpr uint32_t brush_x = 44, brush_y = 45, mask_dx = 46, mask_dy = 47;
// Source mapping per axis: src = base + sign * floor((coord - dref) * mag / den),
// with magic/shift1/shift2 the divisor parameters of den and neg 0 or 0xffffffff.
constexpr uint32_t map_x = 48, map_y = 55;
constexpr uint32_t map_dref = 0, map_base = 1, map_mag = 2, map_magic = 3, map_shift1 = 4,
                   map_shift2 = 5, map_neg = 6;
constexpr uint32_t source_width = 62, source_height = 63;

constexpr uint32_t flag_load_destination = 1u << 0;
constexpr uint32_t flag_source_color = 1u << 1;
constexpr uint32_t flag_source_mono = 1u << 2;
constexpr uint32_t flag_pattern_tile = 1u << 3;
constexpr uint32_t flag_mask = 1u << 4;
constexpr uint32_t flag_color_key = 1u << 5;
constexpr uint32_t flag_skip_outside_source = 1u << 6;
// dib/stretchblt.c DIB_XXBPP_StretchBlt mask: the mask pixel (sx + mask_dx,
// sy + mask_dy) of the mapped source sample is read; a set bit or a sample
// outside the stretch_mask_width x stretch_mask_height mask skips the pixel.
// Exclusive with flag_mask: the background minterm slots, unused because the
// stretch evaluates only the low ROP3 byte, carry the mask size.
constexpr uint32_t flag_stretch_mask = 1u << 7;
constexpr uint32_t stretch_mask_width = background + 0;
constexpr uint32_t stretch_mask_height = background + 1;
// Translated sources (dib/stretchblt.c XLATEOBJ_iXlate(fnSource_GetPixel)): the
// source V# starts at a translation table and the pixel rows begin at byte
// source_offset (the mono0 slot, unused without flag_source_mono). index8: one
// byte per pixel indexes 256 dwords; rgb16: one little-endian halfword indexes
// 65536 dwords; rgb24: bytes b0, b1, b2 (GetPixel b0 | b1 << 8 | b2 << 16) select
// T0[b0] | T1[b1] | T2[b2] from three 256-dword tables at 0, 1024 and 2048 (the
// OR-separable form, exact only for OR-linear translations).
constexpr uint32_t flag_source_index8 = 1u << 8;
constexpr uint32_t flag_source_rgb16 = 1u << 9;
constexpr uint32_t flag_source_rgb24 = 1u << 10;
constexpr uint32_t source_offset = mono0;
constexpr uint32_t index8_table_bytes = 256u * 4u;
constexpr uint32_t rgb16_table_bytes = 65536u * 4u;
constexpr uint32_t rgb24_table_bytes = 3u * 256u * 4u;
// DIB_XXBPP_StretchBlt PatternX with a mask: PatternX restarts at the DIB
// call's left edge (map_x dref) and advances only for drawn pixels, so the tile
// column of pixel x is ((dref + drawn) - brush_x) & 7, drawn = number of pixels
// in [dref, x) of the row whose stretch-mask sample is drawable. Requires
// flag_stretch_mask and flag_pattern_tile.
constexpr uint32_t flag_pattern_running = 1u << 11;
} // namespace rop_args

namespace alpha_args {
constexpr uint32_t destination = 0, source = 4;
constexpr uint32_t left = 16, top = 17, right = 18, destination_pitch = 19;
constexpr uint32_t flags = 20, source_pitch = 21, constant_alpha = 22, solid = 23;
constexpr uint32_t map_x = rop_args::map_x, map_y = rop_args::map_y;
constexpr uint32_t source_width = 62, source_height = 63;
constexpr uint32_t flag_source_alpha = 1u << 0;  // AC_SRC_ALPHA
// Coverage8 glyphs: the source is an 8-bpp coverage mask (byte at sy * pitch + sx)
// blended with the solid brush colour as the pinned EngMaskBitBlt does.
constexpr uint32_t flag_coverage8 = 1u << 1;
} // namespace alpha_args

namespace gradient_args {
constexpr uint32_t destination = 0;
constexpr uint32_t left = 16, top = 17, right = 18, destination_pitch = 19;
constexpr uint32_t flags = 20, origin = 21, divisor = 22, bias = 23;
constexpr uint32_t magic = 24, shift1 = 25, shift2 = 26;
constexpr uint32_t color = 27;    // 3 channels (red, green, blue)
constexpr uint32_t delta = 30;    // 3 channels
constexpr uint32_t negative = 33; // 3 channels, 0 or 0xffffffff
constexpr uint32_t flag_horizontal = 1u << 0;
} // namespace gradient_args

namespace triangle_args {
constexpr uint32_t destination = 0;
constexpr uint32_t left = 16, top = 17, right = 18, destination_pitch = 19;
constexpr uint32_t table_low = 20, table_high = 21, row_stride = 22;
// Span dword indices within a row record.
constexpr uint32_t span_low = 0, span_high = 1, span_reverse = 2, span_bias = 3;
constexpr uint32_t span_magic = 4, span_shift1 = 5, span_shift2 = 6;
constexpr uint32_t span_color = 7, span_delta = 10, span_negative = 13;
} // namespace triangle_args

namespace line_args {
constexpr uint32_t destination = 0, style = 4;
constexpr uint32_t pattern = 12;  // V# of the 8x8 tile (flag_pattern)
constexpr uint32_t left = 16, top = 17, right = 18, destination_pitch = 19;
constexpr uint32_t bottom = 20, flags = 21, first_step = 22, end_step = 23;
constexpr uint32_t x_base = 24, x_major = 25, x_minor = 26;
constexpr uint32_t y_base = 27, y_major = 28, y_minor = 29;
constexpr uint32_t error = 30, minor_delta = 31, magic = 32, shift1 = 33, shift2 = 34;
constexpr uint32_t color = 35;
constexpr uint32_t rop = 36;      // 8 minterm masks; S is zero
// Pattern tile origin: P of pixel (x, y) is tile[((y - brush_y) & 7) * 8 +
// ((x - brush_x) & 7)] (the DIB pattern convention) instead of color.
constexpr uint32_t brush_x = 44, brush_y = 45;
constexpr uint32_t flag_styled = 1u << 0;
constexpr uint32_t flag_load_destination = 1u << 1;
constexpr uint32_t flag_pattern = 1u << 2;
} // namespace line_args

} // namespace rx7900::hardware

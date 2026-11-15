/*
Module: rx7900.opengl.gfx11_registers
Purpose: GC 11.0.0 (Navi31) register offsets and field shifts/masks used by the
GFX11 OpenGL pipeline, plus the PM4 register-space rules and the allow-list the
kernel graphics IB builder enforces.
Created: 2026-10-23
Architecture: Header-only, freestanding C++17 constants shared by the kernel-side
IB builder (gfx11_state) and the host tests. Values are transcribed from the
pinned AMD headers in work/upstream-gpuinit (SHA-256 in its SHA256SUMS):
  gc_11_0_0_offset.h  a2394677509a3415a9e6307d876cae9cd654a99275ab40b3027d983da26801f2
  gc_11_0_0_sh_mask.h 83dd0060886da2927e3f722b992af208e4d41999b0740123a770a6561f060c94
Every register keeps its header name (namespace) and every field its header
field name, so tests/opengl_gfx11_registers_tests.cpp re-parses both headers at
test time and checks each offset, BASE_IDX, shift and mask, and cross-checks the
absolute address against Mesa 24.2.0 gfx11.json. Upper-case names deliberately
mirror the AMD headers instead of the project snake_case rule. A register is
writable by the IB builder only when it appears in this catalog; the builder
refuses any other register (allowed_register).
Copyright 2021 Advanced Micro Devices, Inc. (gc_11_0_0_offset.h and
gc_11_0_0_sh_mask.h; MIT permission notice preserved in licenses/OPENGL.txt).
*/
#pragma once

#include <stdint.h>

namespace rx7900::opengl::gfx11_reg {

// A register as the AMD header names it: dword offset inside its GC segment and
// the segment (BASE_IDX). The absolute dword address adds the GC 11.0.0 segment
// base from IP discovery: segment 0 = 0x1260, segment 1 = 0xA000 (Navi31
// GC_BASE; the same absolute addresses Mesa sid.h and gfx11.json use).
struct Register final {
    uint32_t offset;
    uint32_t base_index;
};

// One bit field: value placement (shift) and the in-place mask.
struct Field final {
    uint32_t shift;
    uint32_t mask;
};

constexpr uint32_t gc_segment0 = 0x1260;
constexpr uint32_t gc_segment1 = 0xa000;

constexpr uint32_t absolute_dword(Register r) noexcept {
    return r.offset + (r.base_index == 0 ? gc_segment0 : gc_segment1);
}

// PM4 register spaces (Mesa sid.h SI_CONTEXT_REG_OFFSET/SI_SH_REG_OFFSET/
// CIK_UCONFIG_REG_OFFSET ranges, in dwords): SET_CONTEXT_REG 0xA000-0xBFFF,
// SET_SH_REG 0x2C00-0x2FFF, SET_UCONFIG_REG 0xC000-0xFFFF.
enum class Space : uint32_t { context, sh, uconfig, none };
constexpr uint32_t context_first = 0xa000, context_end = 0xc000;
constexpr uint32_t sh_first = 0x2c00, sh_end = 0x3000;
constexpr uint32_t uconfig_first = 0xc000, uconfig_end = 0x10000;

constexpr Space space(Register r) noexcept {
    const uint32_t a = absolute_dword(r);
    return a >= context_first && a < context_end ? Space::context :
           a >= sh_first && a < sh_end ? Space::sh :
           a >= uconfig_first && a < uconfig_end ? Space::uconfig : Space::none;
}

// True when value fits the field; place() then returns the in-place bits.
constexpr bool fits(Field f, uint32_t value) noexcept {
    return ((static_cast<uint64_t>(value) << f.shift) & ~static_cast<uint64_t>(f.mask)) == 0;
}
constexpr uint32_t place(Field f, uint32_t value) noexcept {
    return (value << f.shift) & f.mask;
}

namespace DB_RENDER_CONTROL {
constexpr Register reg{0x0000, 1};
constexpr Field DEPTH_CLEAR_ENABLE{0, 0x00000001};
constexpr Field STENCIL_CLEAR_ENABLE{1, 0x00000002};
constexpr Field DEPTH_COPY{2, 0x00000004};
constexpr Field STENCIL_COPY{3, 0x00000008};
constexpr Field RESUMMARIZE_ENABLE{4, 0x00000010};
constexpr Field STENCIL_COMPRESS_DISABLE{5, 0x00000020};
constexpr Field DEPTH_COMPRESS_DISABLE{6, 0x00000040};
constexpr Field COPY_CENTROID{7, 0x00000080};
constexpr Field COPY_SAMPLE{8, 0x00000f00};
constexpr Field DECOMPRESS_ENABLE{12, 0x00001000};
constexpr Field PS_INVOKE_DISABLE{14, 0x00004000};
constexpr Field OREO_MODE{16, 0x00030000};
constexpr Field FORCE_OREO_MODE{18, 0x00040000};
constexpr Field FORCE_EXPORT_ORDER{19, 0x00080000};
constexpr Field MAX_ALLOWED_TILES_IN_WAVE{20, 0x00f00000};
} // namespace DB_RENDER_CONTROL
namespace DB_COUNT_CONTROL {
constexpr Register reg{0x0001, 1};
constexpr Field PERFECT_ZPASS_COUNTS{1, 0x00000002};
constexpr Field DISABLE_CONSERVATIVE_ZPASS_COUNTS{2, 0x00000004};
constexpr Field ENHANCED_CONSERVATIVE_ZPASS_COUNTS{3, 0x00000008};
constexpr Field SAMPLE_RATE{4, 0x00000070};
constexpr Field ZPASS_ENABLE{8, 0x00000f00};
constexpr Field ZFAIL_ENABLE{12, 0x0000f000};
constexpr Field SFAIL_ENABLE{16, 0x000f0000};
constexpr Field DBFAIL_ENABLE{20, 0x00f00000};
constexpr Field SLICE_EVEN_ENABLE{24, 0x0f000000};
constexpr Field SLICE_ODD_ENABLE{28, 0xf0000000};
} // namespace DB_COUNT_CONTROL
namespace DB_RENDER_OVERRIDE {
constexpr Register reg{0x0003, 1};
constexpr Field FORCE_HIZ_ENABLE{0, 0x00000003};
constexpr Field FORCE_HIS_ENABLE0{2, 0x0000000c};
constexpr Field FORCE_HIS_ENABLE1{4, 0x00000030};
constexpr Field FORCE_SHADER_Z_ORDER{6, 0x00000040};
constexpr Field FAST_Z_DISABLE{7, 0x00000080};
constexpr Field FAST_STENCIL_DISABLE{8, 0x00000100};
constexpr Field NOOP_CULL_DISABLE{9, 0x00000200};
constexpr Field FORCE_COLOR_KILL{10, 0x00000400};
constexpr Field FORCE_Z_READ{11, 0x00000800};
constexpr Field FORCE_STENCIL_READ{12, 0x00001000};
constexpr Field FORCE_FULL_Z_RANGE{13, 0x00006000};
constexpr Field DISABLE_VIEWPORT_CLAMP{16, 0x00010000};
constexpr Field IGNORE_SC_ZRANGE{17, 0x00020000};
constexpr Field DISABLE_FULLY_COVERED{18, 0x00040000};
constexpr Field FORCE_Z_LIMIT_SUMM{19, 0x00180000};
constexpr Field MAX_TILES_IN_DTT{21, 0x03e00000};
constexpr Field DISABLE_TILE_RATE_TILES{26, 0x04000000};
constexpr Field FORCE_Z_DIRTY{27, 0x08000000};
constexpr Field FORCE_STENCIL_DIRTY{28, 0x10000000};
constexpr Field FORCE_Z_VALID{29, 0x20000000};
constexpr Field FORCE_STENCIL_VALID{30, 0x40000000};
constexpr Field PRESERVE_COMPRESSION{31, 0x80000000};
} // namespace DB_RENDER_OVERRIDE
namespace DB_RENDER_OVERRIDE2 {
constexpr Register reg{0x0004, 1};
constexpr Field PARTIAL_SQUAD_LAUNCH_CONTROL{0, 0x00000003};
constexpr Field PARTIAL_SQUAD_LAUNCH_COUNTDOWN{2, 0x0000001c};
constexpr Field DISABLE_ZMASK_EXPCLEAR_OPTIMIZATION{5, 0x00000020};
constexpr Field DISABLE_SMEM_EXPCLEAR_OPTIMIZATION{6, 0x00000040};
constexpr Field DISABLE_COLOR_ON_VALIDATION{7, 0x00000080};
constexpr Field DECOMPRESS_Z_ON_FLUSH{8, 0x00000100};
constexpr Field DISABLE_REG_SNOOP{9, 0x00000200};
constexpr Field DEPTH_BOUNDS_HIER_DEPTH_DISABLE{10, 0x00000400};
constexpr Field SEPARATE_HIZS_FUNC_ENABLE{11, 0x00000800};
constexpr Field HIZ_ZFUNC{12, 0x00007000};
constexpr Field HIS_SFUNC_FF{15, 0x00038000};
constexpr Field HIS_SFUNC_BF{18, 0x001c0000};
constexpr Field PRESERVE_ZRANGE{21, 0x00200000};
constexpr Field PRESERVE_SRESULTS{22, 0x00400000};
constexpr Field DISABLE_FAST_PASS{23, 0x00800000};
constexpr Field ALLOW_PARTIAL_RES_HIER_KILL{25, 0x02000000};
constexpr Field CENTROID_COMPUTATION_MODE{27, 0x18000000};
constexpr Field DISABLE_NOZ{29, 0x20000000};
} // namespace DB_RENDER_OVERRIDE2
namespace DB_DEPTH_VIEW {
constexpr Register reg{0x0002, 1};
constexpr Field SLICE_START{0, 0x000007ff};
constexpr Field SLICE_START_HI{11, 0x00001800};
constexpr Field SLICE_MAX{13, 0x00ffe000};
constexpr Field Z_READ_ONLY{24, 0x01000000};
constexpr Field STENCIL_READ_ONLY{25, 0x02000000};
constexpr Field MIPID{26, 0x3c000000};
constexpr Field SLICE_MAX_HI{30, 0xc0000000};
} // namespace DB_DEPTH_VIEW
namespace DB_HTILE_DATA_BASE {
constexpr Register reg{0x0005, 1};
constexpr Field BASE_256B{0, 0xffffffff};
} // namespace DB_HTILE_DATA_BASE
namespace DB_HTILE_DATA_BASE_HI {
constexpr Register reg{0x001e, 1};
constexpr Field BASE_HI{0, 0x000000ff};
} // namespace DB_HTILE_DATA_BASE_HI
namespace DB_HTILE_SURFACE {
constexpr Register reg{0x02af, 1};
constexpr Field RESERVED_FIELD_1{0, 0x00000001};
constexpr Field FULL_CACHE{1, 0x00000002};
constexpr Field RESERVED_FIELD_2{2, 0x00000004};
constexpr Field RESERVED_FIELD_3{3, 0x00000008};
constexpr Field RESERVED_FIELD_4{4, 0x000003f0};
constexpr Field RESERVED_FIELD_5{10, 0x0000fc00};
constexpr Field DST_OUTSIDE_ZERO_TO_ONE{16, 0x00010000};
constexpr Field RESERVED_FIELD_6{17, 0x00020000};
constexpr Field PIPE_ALIGNED{18, 0x00040000};
} // namespace DB_HTILE_SURFACE
namespace DB_DEPTH_SIZE_XY {
constexpr Register reg{0x0007, 1};
constexpr Field X_MAX{0, 0x00003fff};
constexpr Field Y_MAX{16, 0x3fff0000};
} // namespace DB_DEPTH_SIZE_XY
namespace DB_DEPTH_BOUNDS_MIN {
constexpr Register reg{0x0008, 1};
constexpr Field MIN{0, 0xffffffff};
} // namespace DB_DEPTH_BOUNDS_MIN
namespace DB_DEPTH_BOUNDS_MAX {
constexpr Register reg{0x0009, 1};
constexpr Field MAX{0, 0xffffffff};
} // namespace DB_DEPTH_BOUNDS_MAX
namespace DB_STENCIL_CLEAR {
constexpr Register reg{0x000a, 1};
constexpr Field CLEAR{0, 0x000000ff};
} // namespace DB_STENCIL_CLEAR
namespace DB_DEPTH_CLEAR {
constexpr Register reg{0x000b, 1};
constexpr Field DEPTH_CLEAR{0, 0xffffffff};
} // namespace DB_DEPTH_CLEAR
namespace DB_Z_INFO {
constexpr Register reg{0x0010, 1};
constexpr Field FORMAT{0, 0x00000003};
constexpr Field NUM_SAMPLES{2, 0x0000000c};
constexpr Field SW_MODE{4, 0x000001f0};
constexpr Field FAULT_BEHAVIOR{9, 0x00000600};
constexpr Field ITERATE_FLUSH{11, 0x00000800};
constexpr Field PARTIALLY_RESIDENT{12, 0x00001000};
constexpr Field RESERVED_FIELD_1{13, 0x0000e000};
constexpr Field MAXMIP{16, 0x000f0000};
constexpr Field ITERATE_256{20, 0x00100000};
constexpr Field DECOMPRESS_ON_N_ZPLANES{23, 0x07800000};
constexpr Field ALLOW_EXPCLEAR{27, 0x08000000};
constexpr Field READ_SIZE{28, 0x10000000};
constexpr Field TILE_SURFACE_ENABLE{29, 0x20000000};
constexpr Field ZRANGE_PRECISION{31, 0x80000000};
} // namespace DB_Z_INFO
namespace DB_STENCIL_INFO {
constexpr Register reg{0x0011, 1};
constexpr Field FORMAT{0, 0x00000001};
constexpr Field SW_MODE{4, 0x000001f0};
constexpr Field FAULT_BEHAVIOR{9, 0x00000600};
constexpr Field ITERATE_FLUSH{11, 0x00000800};
constexpr Field PARTIALLY_RESIDENT{12, 0x00001000};
constexpr Field RESERVED_FIELD_1{13, 0x0000e000};
constexpr Field ITERATE_256{20, 0x00100000};
constexpr Field ALLOW_EXPCLEAR{27, 0x08000000};
constexpr Field TILE_STENCIL_DISABLE{29, 0x20000000};
} // namespace DB_STENCIL_INFO
namespace DB_Z_READ_BASE {
constexpr Register reg{0x0012, 1};
constexpr Field BASE_256B{0, 0xffffffff};
} // namespace DB_Z_READ_BASE
namespace DB_STENCIL_READ_BASE {
constexpr Register reg{0x0013, 1};
constexpr Field BASE_256B{0, 0xffffffff};
} // namespace DB_STENCIL_READ_BASE
namespace DB_Z_WRITE_BASE {
constexpr Register reg{0x0014, 1};
constexpr Field BASE_256B{0, 0xffffffff};
} // namespace DB_Z_WRITE_BASE
namespace DB_STENCIL_WRITE_BASE {
constexpr Register reg{0x0015, 1};
constexpr Field BASE_256B{0, 0xffffffff};
} // namespace DB_STENCIL_WRITE_BASE
namespace DB_Z_READ_BASE_HI {
constexpr Register reg{0x001a, 1};
constexpr Field BASE_HI{0, 0x000000ff};
} // namespace DB_Z_READ_BASE_HI
namespace DB_STENCIL_READ_BASE_HI {
constexpr Register reg{0x001b, 1};
constexpr Field BASE_HI{0, 0x000000ff};
} // namespace DB_STENCIL_READ_BASE_HI
namespace DB_Z_WRITE_BASE_HI {
constexpr Register reg{0x001c, 1};
constexpr Field BASE_HI{0, 0x000000ff};
} // namespace DB_Z_WRITE_BASE_HI
namespace DB_STENCIL_WRITE_BASE_HI {
constexpr Register reg{0x001d, 1};
constexpr Field BASE_HI{0, 0x000000ff};
} // namespace DB_STENCIL_WRITE_BASE_HI
namespace DB_DEPTH_CONTROL {
constexpr Register reg{0x0200, 1};
constexpr Field STENCIL_ENABLE{0, 0x00000001};
constexpr Field Z_ENABLE{1, 0x00000002};
constexpr Field Z_WRITE_ENABLE{2, 0x00000004};
constexpr Field DEPTH_BOUNDS_ENABLE{3, 0x00000008};
constexpr Field ZFUNC{4, 0x00000070};
constexpr Field BACKFACE_ENABLE{7, 0x00000080};
constexpr Field STENCILFUNC{8, 0x00000700};
constexpr Field STENCILFUNC_BF{20, 0x00700000};
constexpr Field ENABLE_COLOR_WRITES_ON_DEPTH_FAIL{30, 0x40000000};
constexpr Field DISABLE_COLOR_WRITES_ON_DEPTH_PASS{31, 0x80000000};
} // namespace DB_DEPTH_CONTROL
namespace DB_STENCIL_CONTROL {
constexpr Register reg{0x010b, 1};
constexpr Field STENCILFAIL{0, 0x0000000f};
constexpr Field STENCILZPASS{4, 0x000000f0};
constexpr Field STENCILZFAIL{8, 0x00000f00};
constexpr Field STENCILFAIL_BF{12, 0x0000f000};
constexpr Field STENCILZPASS_BF{16, 0x000f0000};
constexpr Field STENCILZFAIL_BF{20, 0x00f00000};
} // namespace DB_STENCIL_CONTROL
namespace DB_STENCILREFMASK {
constexpr Register reg{0x010c, 1};
constexpr Field STENCILTESTVAL{0, 0x000000ff};
constexpr Field STENCILMASK{8, 0x0000ff00};
constexpr Field STENCILWRITEMASK{16, 0x00ff0000};
constexpr Field STENCILOPVAL{24, 0xff000000};
} // namespace DB_STENCILREFMASK
namespace DB_STENCILREFMASK_BF {
constexpr Register reg{0x010d, 1};
constexpr Field STENCILTESTVAL_BF{0, 0x000000ff};
constexpr Field STENCILMASK_BF{8, 0x0000ff00};
constexpr Field STENCILWRITEMASK_BF{16, 0x00ff0000};
constexpr Field STENCILOPVAL_BF{24, 0xff000000};
} // namespace DB_STENCILREFMASK_BF
namespace DB_SHADER_CONTROL {
constexpr Register reg{0x0203, 1};
constexpr Field Z_EXPORT_ENABLE{0, 0x00000001};
constexpr Field STENCIL_TEST_VAL_EXPORT_ENABLE{1, 0x00000002};
constexpr Field STENCIL_OP_VAL_EXPORT_ENABLE{2, 0x00000004};
constexpr Field Z_ORDER{4, 0x00000030};
constexpr Field KILL_ENABLE{6, 0x00000040};
constexpr Field COVERAGE_TO_MASK_ENABLE{7, 0x00000080};
constexpr Field MASK_EXPORT_ENABLE{8, 0x00000100};
constexpr Field EXEC_ON_HIER_FAIL{9, 0x00000200};
constexpr Field EXEC_ON_NOOP{10, 0x00000400};
constexpr Field ALPHA_TO_MASK_DISABLE{11, 0x00000800};
constexpr Field DEPTH_BEFORE_SHADER{12, 0x00001000};
constexpr Field CONSERVATIVE_Z_EXPORT{13, 0x00006000};
constexpr Field DUAL_QUAD_DISABLE{15, 0x00008000};
constexpr Field PRIMITIVE_ORDERED_PIXEL_SHADER{16, 0x00010000};
constexpr Field PRE_SHADER_DEPTH_COVERAGE_ENABLE{23, 0x00800000};
constexpr Field OREO_BLEND_ENABLE{24, 0x01000000};
constexpr Field OVERRIDE_INTRINSIC_RATE_ENABLE{25, 0x02000000};
constexpr Field OVERRIDE_INTRINSIC_RATE{26, 0x1c000000};
} // namespace DB_SHADER_CONTROL
namespace DB_RMI_L2_CACHE_CONTROL {
constexpr Register reg{0x001f, 1};
constexpr Field Z_WR_POLICY{0, 0x00000003};
constexpr Field S_WR_POLICY{2, 0x0000000c};
constexpr Field HTILE_WR_POLICY{4, 0x00000030};
constexpr Field ZPCPSD_WR_POLICY{6, 0x000000c0};
constexpr Field Z_RD_POLICY{16, 0x00030000};
constexpr Field S_RD_POLICY{18, 0x000c0000};
constexpr Field HTILE_RD_POLICY{20, 0x00300000};
constexpr Field Z_BIG_PAGE{24, 0x01000000};
constexpr Field S_BIG_PAGE{25, 0x02000000};
constexpr Field Z_NOALLOC{26, 0x04000000};
constexpr Field S_NOALLOC{27, 0x08000000};
constexpr Field HTILE_NOALLOC{28, 0x10000000};
constexpr Field ZPCPSD_NOALLOC{29, 0x20000000};
} // namespace DB_RMI_L2_CACHE_CONTROL
namespace DB_ALPHA_TO_MASK {
constexpr Register reg{0x02dc, 1};
constexpr Field ALPHA_TO_MASK_ENABLE{0, 0x00000001};
constexpr Field ALPHA_TO_MASK_OFFSET0{8, 0x00000300};
constexpr Field ALPHA_TO_MASK_OFFSET1{10, 0x00000c00};
constexpr Field ALPHA_TO_MASK_OFFSET2{12, 0x00003000};
constexpr Field ALPHA_TO_MASK_OFFSET3{14, 0x0000c000};
constexpr Field OFFSET_ROUND{16, 0x00010000};
} // namespace DB_ALPHA_TO_MASK
namespace DB_EQAA {
constexpr Register reg{0x0201, 1};
constexpr Field MAX_ANCHOR_SAMPLES{0, 0x00000007};
constexpr Field PS_ITER_SAMPLES{4, 0x00000070};
constexpr Field MASK_EXPORT_NUM_SAMPLES{8, 0x00000700};
constexpr Field ALPHA_TO_MASK_NUM_SAMPLES{12, 0x00007000};
constexpr Field HIGH_QUALITY_INTERSECTIONS{16, 0x00010000};
constexpr Field INCOHERENT_EQAA_READS{17, 0x00020000};
constexpr Field INTERPOLATE_COMP_Z{18, 0x00040000};
constexpr Field INTERPOLATE_SRC_Z{19, 0x00080000};
constexpr Field STATIC_ANCHOR_ASSOCIATIONS{20, 0x00100000};
constexpr Field ALPHA_TO_MASK_EQAA_DISABLE{21, 0x00200000};
constexpr Field OVERRASTERIZATION_AMOUNT{24, 0x07000000};
constexpr Field ENABLE_POSTZ_OVERRASTERIZATION{27, 0x08000000};
} // namespace DB_EQAA
namespace TA_BC_BASE_ADDR {
constexpr Register reg{0x0020, 1};
constexpr Field ADDRESS{0, 0xffffffff};
} // namespace TA_BC_BASE_ADDR
namespace TA_BC_BASE_ADDR_HI {
constexpr Register reg{0x0021, 1};
constexpr Field ADDRESS{0, 0x000000ff};
} // namespace TA_BC_BASE_ADDR_HI
namespace PA_SC_WINDOW_OFFSET {
constexpr Register reg{0x0080, 1};
constexpr Field WINDOW_X_OFFSET{0, 0x0000ffff};
constexpr Field WINDOW_Y_OFFSET{16, 0xffff0000};
} // namespace PA_SC_WINDOW_OFFSET
namespace PA_SC_WINDOW_SCISSOR_TL {
constexpr Register reg{0x0081, 1};
constexpr Field TL_X{0, 0x00007fff};
constexpr Field TL_Y{16, 0x7fff0000};
constexpr Field WINDOW_OFFSET_DISABLE{31, 0x80000000};
} // namespace PA_SC_WINDOW_SCISSOR_TL
namespace PA_SC_WINDOW_SCISSOR_BR {
constexpr Register reg{0x0082, 1};
constexpr Field BR_X{0, 0x00007fff};
constexpr Field BR_Y{16, 0x7fff0000};
} // namespace PA_SC_WINDOW_SCISSOR_BR
namespace PA_SC_SCREEN_SCISSOR_TL {
constexpr Register reg{0x000c, 1};
constexpr Field TL_X{0, 0x0000ffff};
constexpr Field TL_Y{16, 0xffff0000};
} // namespace PA_SC_SCREEN_SCISSOR_TL
namespace PA_SC_SCREEN_SCISSOR_BR {
constexpr Register reg{0x000d, 1};
constexpr Field BR_X{0, 0x0000ffff};
constexpr Field BR_Y{16, 0xffff0000};
} // namespace PA_SC_SCREEN_SCISSOR_BR
namespace PA_SC_GENERIC_SCISSOR_TL {
constexpr Register reg{0x0090, 1};
constexpr Field TL_X{0, 0x00007fff};
constexpr Field TL_Y{16, 0x7fff0000};
constexpr Field WINDOW_OFFSET_DISABLE{31, 0x80000000};
} // namespace PA_SC_GENERIC_SCISSOR_TL
namespace PA_SC_GENERIC_SCISSOR_BR {
constexpr Register reg{0x0091, 1};
constexpr Field BR_X{0, 0x00007fff};
constexpr Field BR_Y{16, 0x7fff0000};
} // namespace PA_SC_GENERIC_SCISSOR_BR
namespace PA_SC_VPORT_SCISSOR_0_TL {
constexpr Register reg{0x0094, 1};
constexpr Field TL_X{0, 0x00007fff};
constexpr Field TL_Y{16, 0x7fff0000};
constexpr Field WINDOW_OFFSET_DISABLE{31, 0x80000000};
} // namespace PA_SC_VPORT_SCISSOR_0_TL
namespace PA_SC_VPORT_SCISSOR_0_BR {
constexpr Register reg{0x0095, 1};
constexpr Field BR_X{0, 0x00007fff};
constexpr Field BR_Y{16, 0x7fff0000};
} // namespace PA_SC_VPORT_SCISSOR_0_BR
namespace PA_SC_VPORT_ZMIN_0 {
constexpr Register reg{0x00b4, 1};
constexpr Field VPORT_ZMIN{0, 0xffffffff};
} // namespace PA_SC_VPORT_ZMIN_0
namespace PA_SC_VPORT_ZMAX_0 {
constexpr Register reg{0x00b5, 1};
constexpr Field VPORT_ZMAX{0, 0xffffffff};
} // namespace PA_SC_VPORT_ZMAX_0
namespace PA_SC_CLIPRECT_RULE {
constexpr Register reg{0x0083, 1};
constexpr Field CLIP_RULE{0, 0x0000ffff};
} // namespace PA_SC_CLIPRECT_RULE
namespace PA_SC_EDGERULE {
constexpr Register reg{0x008c, 1};
constexpr Field ER_TRI{0, 0x0000000f};
constexpr Field ER_POINT{4, 0x000000f0};
constexpr Field ER_RECT{8, 0x00000f00};
constexpr Field ER_LINE_LR{12, 0x0003f000};
constexpr Field ER_LINE_RL{18, 0x00fc0000};
constexpr Field ER_LINE_TB{24, 0x0f000000};
constexpr Field ER_LINE_BT{28, 0xf0000000};
} // namespace PA_SC_EDGERULE
namespace PA_SU_HARDWARE_SCREEN_OFFSET {
constexpr Register reg{0x008d, 1};
constexpr Field HW_SCREEN_OFFSET_X{0, 0x000001ff};
constexpr Field HW_SCREEN_OFFSET_Y{16, 0x01ff0000};
} // namespace PA_SU_HARDWARE_SCREEN_OFFSET
namespace PA_SC_VRS_OVERRIDE_CNTL {
constexpr Register reg{0x00f4, 1};
constexpr Field VRS_OVERRIDE_RATE_COMBINER_MODE{0, 0x00000007};
constexpr Field VRS_RATE{4, 0x000000f0};
constexpr Field VRS_SURFACE_ENABLE{12, 0x00001000};
constexpr Field RATE_HINT_WRITE_BACK_ENABLE{13, 0x00002000};
constexpr Field VRS_FEEDBACK_RATE_OVERRIDE{14, 0x00004000};
} // namespace PA_SC_VRS_OVERRIDE_CNTL
namespace PA_SC_MODE_CNTL_0 {
constexpr Register reg{0x0292, 1};
constexpr Field MSAA_ENABLE{0, 0x00000001};
constexpr Field VPORT_SCISSOR_ENABLE{1, 0x00000002};
constexpr Field LINE_STIPPLE_ENABLE{2, 0x00000004};
constexpr Field SEND_UNLIT_STILES_TO_PKR{3, 0x00000008};
constexpr Field ALTERNATE_RBS_PER_TILE{5, 0x00000020};
constexpr Field COARSE_TILE_STARTS_ON_EVEN_RB{6, 0x00000040};
} // namespace PA_SC_MODE_CNTL_0
namespace PA_SC_MODE_CNTL_1 {
constexpr Register reg{0x0293, 1};
constexpr Field WALK_SIZE{0, 0x00000001};
constexpr Field WALK_ALIGNMENT{1, 0x00000002};
constexpr Field WALK_ALIGN8_PRIM_FITS_ST{2, 0x00000004};
constexpr Field WALK_FENCE_ENABLE{3, 0x00000008};
constexpr Field WALK_FENCE_SIZE{4, 0x00000070};
constexpr Field SUPERTILE_WALK_ORDER_ENABLE{7, 0x00000080};
constexpr Field TILE_WALK_ORDER_ENABLE{8, 0x00000100};
constexpr Field TILE_COVER_DISABLE{9, 0x00000200};
constexpr Field TILE_COVER_NO_SCISSOR{10, 0x00000400};
constexpr Field ZMM_LINE_EXTENT{11, 0x00000800};
constexpr Field ZMM_LINE_OFFSET{12, 0x00001000};
constexpr Field ZMM_RECT_EXTENT{13, 0x00002000};
constexpr Field KILL_PIX_POST_HI_Z{14, 0x00004000};
constexpr Field KILL_PIX_POST_DETAIL_MASK{15, 0x00008000};
constexpr Field PS_ITER_SAMPLE{16, 0x00010000};
constexpr Field MULTI_SHADER_ENGINE_PRIM_DISCARD_ENABLE{17, 0x00020000};
constexpr Field MULTI_GPU_SUPERTILE_ENABLE{18, 0x00040000};
constexpr Field GPU_ID_OVERRIDE_ENABLE{19, 0x00080000};
constexpr Field GPU_ID_OVERRIDE{20, 0x00f00000};
constexpr Field MULTI_GPU_PRIM_DISCARD_ENABLE{24, 0x01000000};
constexpr Field FORCE_EOV_CNTDWN_ENABLE{25, 0x02000000};
constexpr Field FORCE_EOV_REZ_ENABLE{26, 0x04000000};
constexpr Field OUT_OF_ORDER_PRIMITIVE_ENABLE{27, 0x08000000};
constexpr Field OUT_OF_ORDER_WATER_MARK{28, 0x70000000};
} // namespace PA_SC_MODE_CNTL_1
namespace PA_SC_AA_CONFIG {
constexpr Register reg{0x02f8, 1};
constexpr Field MSAA_NUM_SAMPLES{0, 0x00000007};
constexpr Field AA_MASK_CENTROID_DTMN{4, 0x00000010};
constexpr Field MAX_SAMPLE_DIST{13, 0x0001e000};
constexpr Field MSAA_EXPOSED_SAMPLES{20, 0x00700000};
constexpr Field DETAIL_TO_EXPOSED_MODE{24, 0x03000000};
constexpr Field COVERAGE_TO_SHADER_SELECT{26, 0x0c000000};
constexpr Field SAMPLE_COVERAGE_ENCODING{28, 0x10000000};
constexpr Field COVERED_CENTROID_IS_CENTER{29, 0x20000000};
} // namespace PA_SC_AA_CONFIG
namespace PA_SC_AA_MASK_X0Y0_X1Y0 {
constexpr Register reg{0x030e, 1};
constexpr Field AA_MASK_X0Y0{0, 0x0000ffff};
constexpr Field AA_MASK_X1Y0{16, 0xffff0000};
} // namespace PA_SC_AA_MASK_X0Y0_X1Y0
namespace PA_SC_AA_MASK_X0Y1_X1Y1 {
constexpr Register reg{0x030f, 1};
constexpr Field AA_MASK_X0Y1{0, 0x0000ffff};
constexpr Field AA_MASK_X1Y1{16, 0xffff0000};
} // namespace PA_SC_AA_MASK_X0Y1_X1Y1
namespace PA_SC_LINE_CNTL {
constexpr Register reg{0x02f7, 1};
constexpr Field EXPAND_LINE_WIDTH{9, 0x00000200};
constexpr Field LAST_PIXEL{10, 0x00000400};
constexpr Field PERPENDICULAR_ENDCAP_ENA{11, 0x00000800};
constexpr Field DX10_DIAMOND_TEST_ENA{12, 0x00001000};
constexpr Field EXTRA_DX_DY_PRECISION{13, 0x00002000};
} // namespace PA_SC_LINE_CNTL
namespace PA_SC_LINE_STIPPLE {
constexpr Register reg{0x0283, 1};
constexpr Field LINE_PATTERN{0, 0x0000ffff};
constexpr Field REPEAT_COUNT{16, 0x00ff0000};
constexpr Field PATTERN_BIT_ORDER{28, 0x10000000};
constexpr Field AUTO_RESET_CNTL{29, 0x60000000};
} // namespace PA_SC_LINE_STIPPLE
namespace PA_SC_BINNER_CNTL_0 {
constexpr Register reg{0x0311, 1};
constexpr Field BINNING_MODE{0, 0x00000003};
constexpr Field BIN_SIZE_X{2, 0x00000004};
constexpr Field BIN_SIZE_Y{3, 0x00000008};
constexpr Field BIN_SIZE_X_EXTEND{4, 0x00000070};
constexpr Field BIN_SIZE_Y_EXTEND{7, 0x00000380};
constexpr Field CONTEXT_STATES_PER_BIN{10, 0x00001c00};
constexpr Field PERSISTENT_STATES_PER_BIN{13, 0x0003e000};
constexpr Field DISABLE_START_OF_PRIM{18, 0x00040000};
constexpr Field FPOVS_PER_BATCH{19, 0x07f80000};
constexpr Field OPTIMAL_BIN_SELECTION{27, 0x08000000};
constexpr Field FLUSH_ON_BINNING_TRANSITION{28, 0x10000000};
constexpr Field BIN_MAPPING_MODE{29, 0x60000000};
} // namespace PA_SC_BINNER_CNTL_0
namespace PA_SC_BINNER_CNTL_1 {
constexpr Register reg{0x0312, 1};
constexpr Field MAX_ALLOC_COUNT{0, 0x0000ffff};
constexpr Field MAX_PRIM_PER_BATCH{16, 0xffff0000};
} // namespace PA_SC_BINNER_CNTL_1
namespace PA_SC_SHADER_CONTROL {
constexpr Register reg{0x0310, 1};
constexpr Field REALIGN_DQUADS_AFTER_N_WAVES{0, 0x00000003};
constexpr Field LOAD_COLLISION_WAVEID{2, 0x00000004};
constexpr Field LOAD_INTRAWAVE_COLLISION{3, 0x00000008};
constexpr Field WAVE_BREAK_REGION_SIZE{5, 0x00000060};
constexpr Field DISABLE_OREO_CONFLICT_QUAD{7, 0x00000080};
} // namespace PA_SC_SHADER_CONTROL
namespace PA_SC_NGG_MODE_CNTL {
constexpr Register reg{0x0314, 1};
constexpr Field MAX_DEALLOCS_IN_WAVE{0, 0x000007ff};
constexpr Field DISABLE_FPOG_AND_DEALLOC_CONFLICT{12, 0x00001000};
constexpr Field DISABLE_MAX_DEALLOC{13, 0x00002000};
constexpr Field DISABLE_MAX_ATTRIBUTES{14, 0x00004000};
constexpr Field MAX_FPOVS_IN_WAVE{16, 0x00ff0000};
constexpr Field MAX_ATTRIBUTES_IN_WAVE{24, 0xff000000};
} // namespace PA_SC_NGG_MODE_CNTL
namespace PA_CL_VPORT_XSCALE {
constexpr Register reg{0x010f, 1};
constexpr Field VPORT_XSCALE{0, 0xffffffff};
} // namespace PA_CL_VPORT_XSCALE
namespace PA_CL_VPORT_XOFFSET {
constexpr Register reg{0x0110, 1};
constexpr Field VPORT_XOFFSET{0, 0xffffffff};
} // namespace PA_CL_VPORT_XOFFSET
namespace PA_CL_VPORT_YSCALE {
constexpr Register reg{0x0111, 1};
constexpr Field VPORT_YSCALE{0, 0xffffffff};
} // namespace PA_CL_VPORT_YSCALE
namespace PA_CL_VPORT_YOFFSET {
constexpr Register reg{0x0112, 1};
constexpr Field VPORT_YOFFSET{0, 0xffffffff};
} // namespace PA_CL_VPORT_YOFFSET
namespace PA_CL_VPORT_ZSCALE {
constexpr Register reg{0x0113, 1};
constexpr Field VPORT_ZSCALE{0, 0xffffffff};
} // namespace PA_CL_VPORT_ZSCALE
namespace PA_CL_VPORT_ZOFFSET {
constexpr Register reg{0x0114, 1};
constexpr Field VPORT_ZOFFSET{0, 0xffffffff};
} // namespace PA_CL_VPORT_ZOFFSET
namespace PA_CL_VTE_CNTL {
constexpr Register reg{0x0206, 1};
constexpr Field VPORT_X_SCALE_ENA{0, 0x00000001};
constexpr Field VPORT_X_OFFSET_ENA{1, 0x00000002};
constexpr Field VPORT_Y_SCALE_ENA{2, 0x00000004};
constexpr Field VPORT_Y_OFFSET_ENA{3, 0x00000008};
constexpr Field VPORT_Z_SCALE_ENA{4, 0x00000010};
constexpr Field VPORT_Z_OFFSET_ENA{5, 0x00000020};
constexpr Field VTX_XY_FMT{8, 0x00000100};
constexpr Field VTX_Z_FMT{9, 0x00000200};
constexpr Field VTX_W0_FMT{10, 0x00000400};
constexpr Field PERFCOUNTER_REF{11, 0x00000800};
} // namespace PA_CL_VTE_CNTL
namespace PA_CL_CLIP_CNTL {
constexpr Register reg{0x0204, 1};
constexpr Field UCP_ENA_0{0, 0x00000001};
constexpr Field UCP_ENA_1{1, 0x00000002};
constexpr Field UCP_ENA_2{2, 0x00000004};
constexpr Field UCP_ENA_3{3, 0x00000008};
constexpr Field UCP_ENA_4{4, 0x00000010};
constexpr Field UCP_ENA_5{5, 0x00000020};
constexpr Field PS_UCP_Y_SCALE_NEG{13, 0x00002000};
constexpr Field PS_UCP_MODE{14, 0x0000c000};
constexpr Field CLIP_DISABLE{16, 0x00010000};
constexpr Field UCP_CULL_ONLY_ENA{17, 0x00020000};
constexpr Field BOUNDARY_EDGE_FLAG_ENA{18, 0x00040000};
constexpr Field DX_CLIP_SPACE_DEF{19, 0x00080000};
constexpr Field DIS_CLIP_ERR_DETECT{20, 0x00100000};
constexpr Field VTX_KILL_OR{21, 0x00200000};
constexpr Field DX_RASTERIZATION_KILL{22, 0x00400000};
constexpr Field DX_LINEAR_ATTR_CLIP_ENA{24, 0x01000000};
constexpr Field VTE_VPORT_PROVOKE_DISABLE{25, 0x02000000};
constexpr Field ZCLIP_NEAR_DISABLE{26, 0x04000000};
constexpr Field ZCLIP_FAR_DISABLE{27, 0x08000000};
constexpr Field ZCLIP_PROG_NEAR_ENA{28, 0x10000000};
} // namespace PA_CL_CLIP_CNTL
namespace PA_CL_GB_VERT_CLIP_ADJ {
constexpr Register reg{0x02fa, 1};
constexpr Field DATA_REGISTER{0, 0xffffffff};
} // namespace PA_CL_GB_VERT_CLIP_ADJ
namespace PA_CL_GB_VERT_DISC_ADJ {
constexpr Register reg{0x02fb, 1};
constexpr Field DATA_REGISTER{0, 0xffffffff};
} // namespace PA_CL_GB_VERT_DISC_ADJ
namespace PA_CL_GB_HORZ_CLIP_ADJ {
constexpr Register reg{0x02fc, 1};
constexpr Field DATA_REGISTER{0, 0xffffffff};
} // namespace PA_CL_GB_HORZ_CLIP_ADJ
namespace PA_CL_GB_HORZ_DISC_ADJ {
constexpr Register reg{0x02fd, 1};
constexpr Field DATA_REGISTER{0, 0xffffffff};
} // namespace PA_CL_GB_HORZ_DISC_ADJ
namespace PA_CL_VS_OUT_CNTL {
constexpr Register reg{0x0207, 1};
constexpr Field CLIP_DIST_ENA_0{0, 0x00000001};
constexpr Field CLIP_DIST_ENA_1{1, 0x00000002};
constexpr Field CLIP_DIST_ENA_2{2, 0x00000004};
constexpr Field CLIP_DIST_ENA_3{3, 0x00000008};
constexpr Field CLIP_DIST_ENA_4{4, 0x00000010};
constexpr Field CLIP_DIST_ENA_5{5, 0x00000020};
constexpr Field CLIP_DIST_ENA_6{6, 0x00000040};
constexpr Field CLIP_DIST_ENA_7{7, 0x00000080};
constexpr Field CULL_DIST_ENA_0{8, 0x00000100};
constexpr Field CULL_DIST_ENA_1{9, 0x00000200};
constexpr Field CULL_DIST_ENA_2{10, 0x00000400};
constexpr Field CULL_DIST_ENA_3{11, 0x00000800};
constexpr Field CULL_DIST_ENA_4{12, 0x00001000};
constexpr Field CULL_DIST_ENA_5{13, 0x00002000};
constexpr Field CULL_DIST_ENA_6{14, 0x00004000};
constexpr Field CULL_DIST_ENA_7{15, 0x00008000};
constexpr Field USE_VTX_POINT_SIZE{16, 0x00010000};
constexpr Field USE_VTX_EDGE_FLAG{17, 0x00020000};
constexpr Field USE_VTX_RENDER_TARGET_INDX{18, 0x00040000};
constexpr Field USE_VTX_VIEWPORT_INDX{19, 0x00080000};
constexpr Field USE_VTX_KILL_FLAG{20, 0x00100000};
constexpr Field VS_OUT_MISC_VEC_ENA{21, 0x00200000};
constexpr Field VS_OUT_CCDIST0_VEC_ENA{22, 0x00400000};
constexpr Field VS_OUT_CCDIST1_VEC_ENA{23, 0x00800000};
constexpr Field VS_OUT_MISC_SIDE_BUS_ENA{24, 0x01000000};
constexpr Field USE_VTX_LINE_WIDTH{27, 0x08000000};
constexpr Field USE_VTX_VRS_RATE{28, 0x10000000};
constexpr Field BYPASS_VTX_RATE_COMBINER{29, 0x20000000};
constexpr Field BYPASS_PRIM_RATE_COMBINER{30, 0x40000000};
} // namespace PA_CL_VS_OUT_CNTL
namespace PA_CL_NGG_CNTL {
constexpr Register reg{0x020e, 1};
constexpr Field VERTEX_REUSE_OFF{0, 0x00000001};
constexpr Field INDEX_BUF_EDGE_FLAG_ENA{1, 0x00000002};
constexpr Field VERTEX_REUSE_DEPTH{2, 0x000003fc};
} // namespace PA_CL_NGG_CNTL
namespace PA_CL_VRS_CNTL {
constexpr Register reg{0x0212, 1};
constexpr Field VERTEX_RATE_COMBINER_MODE{0, 0x00000007};
constexpr Field PRIMITIVE_RATE_COMBINER_MODE{3, 0x00000038};
constexpr Field HTILE_RATE_COMBINER_MODE{6, 0x000001c0};
constexpr Field SAMPLE_ITER_COMBINER_MODE{9, 0x00000e00};
constexpr Field EXPOSE_VRS_PIXELS_MASK{13, 0x00002000};
constexpr Field CMASK_RATE_HINT_FORCE_ZERO{14, 0x00004000};
} // namespace PA_CL_VRS_CNTL
namespace PA_SU_VTX_CNTL {
constexpr Register reg{0x02f9, 1};
constexpr Field PIX_CENTER{0, 0x00000001};
constexpr Field ROUND_MODE{1, 0x00000006};
constexpr Field QUANT_MODE{3, 0x00000038};
} // namespace PA_SU_VTX_CNTL
namespace PA_SU_POINT_SIZE {
constexpr Register reg{0x0280, 1};
constexpr Field HEIGHT{0, 0x0000ffff};
constexpr Field WIDTH{16, 0xffff0000};
} // namespace PA_SU_POINT_SIZE
namespace PA_SU_POINT_MINMAX {
constexpr Register reg{0x0281, 1};
constexpr Field MIN_SIZE{0, 0x0000ffff};
constexpr Field MAX_SIZE{16, 0xffff0000};
} // namespace PA_SU_POINT_MINMAX
namespace PA_SU_LINE_CNTL {
constexpr Register reg{0x0282, 1};
constexpr Field WIDTH{0, 0x0000ffff};
} // namespace PA_SU_LINE_CNTL
namespace PA_SU_SC_MODE_CNTL {
constexpr Register reg{0x0205, 1};
constexpr Field CULL_FRONT{0, 0x00000001};
constexpr Field CULL_BACK{1, 0x00000002};
constexpr Field FACE{2, 0x00000004};
constexpr Field POLY_MODE{3, 0x00000018};
constexpr Field POLYMODE_FRONT_PTYPE{5, 0x000000e0};
constexpr Field POLYMODE_BACK_PTYPE{8, 0x00000700};
constexpr Field POLY_OFFSET_FRONT_ENABLE{11, 0x00000800};
constexpr Field POLY_OFFSET_BACK_ENABLE{12, 0x00001000};
constexpr Field POLY_OFFSET_PARA_ENABLE{13, 0x00002000};
constexpr Field VTX_WINDOW_OFFSET_ENABLE{16, 0x00010000};
constexpr Field PROVOKING_VTX_LAST{19, 0x00080000};
constexpr Field PERSP_CORR_DIS{20, 0x00100000};
constexpr Field MULTI_PRIM_IB_ENA{21, 0x00200000};
constexpr Field RIGHT_TRIANGLE_ALTERNATE_GRADIENT_REF{22, 0x00400000};
constexpr Field NEW_QUAD_DECOMPOSITION{23, 0x00800000};
constexpr Field KEEP_TOGETHER_ENABLE{24, 0x01000000};
} // namespace PA_SU_SC_MODE_CNTL
namespace PA_SU_POLY_OFFSET_DB_FMT_CNTL {
constexpr Register reg{0x02de, 1};
constexpr Field POLY_OFFSET_NEG_NUM_DB_BITS{0, 0x000000ff};
constexpr Field POLY_OFFSET_DB_IS_FLOAT_FMT{8, 0x00000100};
} // namespace PA_SU_POLY_OFFSET_DB_FMT_CNTL
namespace PA_SU_POLY_OFFSET_CLAMP {
constexpr Register reg{0x02df, 1};
constexpr Field CLAMP{0, 0xffffffff};
} // namespace PA_SU_POLY_OFFSET_CLAMP
namespace PA_SU_POLY_OFFSET_FRONT_SCALE {
constexpr Register reg{0x02e0, 1};
constexpr Field SCALE{0, 0xffffffff};
} // namespace PA_SU_POLY_OFFSET_FRONT_SCALE
namespace PA_SU_POLY_OFFSET_FRONT_OFFSET {
constexpr Register reg{0x02e1, 1};
constexpr Field OFFSET{0, 0xffffffff};
} // namespace PA_SU_POLY_OFFSET_FRONT_OFFSET
namespace PA_SU_POLY_OFFSET_BACK_SCALE {
constexpr Register reg{0x02e2, 1};
constexpr Field SCALE{0, 0xffffffff};
} // namespace PA_SU_POLY_OFFSET_BACK_SCALE
namespace PA_SU_POLY_OFFSET_BACK_OFFSET {
constexpr Register reg{0x02e3, 1};
constexpr Field OFFSET{0, 0xffffffff};
} // namespace PA_SU_POLY_OFFSET_BACK_OFFSET
namespace PA_SU_PRIM_FILTER_CNTL {
constexpr Register reg{0x020b, 1};
constexpr Field TRIANGLE_FILTER_DISABLE{0, 0x00000001};
constexpr Field LINE_FILTER_DISABLE{1, 0x00000002};
constexpr Field POINT_FILTER_DISABLE{2, 0x00000004};
constexpr Field RECTANGLE_FILTER_DISABLE{3, 0x00000008};
constexpr Field TRIANGLE_EXPAND_ENA{4, 0x00000010};
constexpr Field LINE_EXPAND_ENA{5, 0x00000020};
constexpr Field POINT_EXPAND_ENA{6, 0x00000040};
constexpr Field RECTANGLE_EXPAND_ENA{7, 0x00000080};
constexpr Field PRIM_EXPAND_CONSTANT{8, 0x0000ff00};
constexpr Field XMAX_RIGHT_EXCLUSION{30, 0x40000000};
constexpr Field YMAX_BOTTOM_EXCLUSION{31, 0x80000000};
} // namespace PA_SU_PRIM_FILTER_CNTL
namespace PA_SU_SMALL_PRIM_FILTER_CNTL {
constexpr Register reg{0x020c, 1};
constexpr Field SMALL_PRIM_FILTER_ENABLE{0, 0x00000001};
constexpr Field TRIANGLE_FILTER_DISABLE{1, 0x00000002};
constexpr Field LINE_FILTER_DISABLE{2, 0x00000004};
constexpr Field POINT_FILTER_DISABLE{3, 0x00000008};
constexpr Field RECTANGLE_FILTER_DISABLE{4, 0x00000010};
} // namespace PA_SU_SMALL_PRIM_FILTER_CNTL
namespace PA_SU_LINE_STIPPLE_CNTL {
constexpr Register reg{0x0209, 1};
constexpr Field LINE_STIPPLE_RESET{0, 0x00000003};
constexpr Field EXPAND_FULL_LENGTH{2, 0x00000004};
constexpr Field FRACTIONAL_ACCUM{3, 0x00000008};
} // namespace PA_SU_LINE_STIPPLE_CNTL
namespace CB_COLOR0_BASE {
constexpr Register reg{0x0318, 1};
constexpr Field BASE_256B{0, 0xffffffff};
} // namespace CB_COLOR0_BASE
namespace CB_COLOR0_BASE_EXT {
constexpr Register reg{0x0390, 1};
constexpr Field BASE_256B{0, 0x000000ff};
} // namespace CB_COLOR0_BASE_EXT
namespace CB_COLOR0_VIEW {
constexpr Register reg{0x031b, 1};
constexpr Field SLICE_START{0, 0x00001fff};
constexpr Field SLICE_MAX{13, 0x03ffe000};
constexpr Field MIP_LEVEL{26, 0x3c000000};
} // namespace CB_COLOR0_VIEW
namespace CB_COLOR0_INFO {
constexpr Register reg{0x031c, 1};
constexpr Field FORMAT{0, 0x0000001f};
constexpr Field LINEAR_GENERAL{7, 0x00000080};
constexpr Field NUMBER_TYPE{8, 0x00000700};
constexpr Field COMP_SWAP{11, 0x00001800};
constexpr Field BLEND_CLAMP{15, 0x00008000};
constexpr Field BLEND_BYPASS{16, 0x00010000};
constexpr Field SIMPLE_FLOAT{17, 0x00020000};
constexpr Field ROUND_MODE{18, 0x00040000};
constexpr Field BLEND_OPT_DONT_RD_DST{20, 0x00700000};
constexpr Field BLEND_OPT_DISCARD_PIXEL{23, 0x03800000};
} // namespace CB_COLOR0_INFO
namespace CB_COLOR0_ATTRIB {
constexpr Register reg{0x031d, 1};
constexpr Field NUM_FRAGMENTS{0, 0x00000003};
constexpr Field FORCE_DST_ALPHA_1{2, 0x00000004};
constexpr Field DISABLE_FMASK_NOALLOC_OPT{3, 0x00000008};
constexpr Field LIMIT_COLOR_FETCH_TO_256B_MAX{4, 0x00000010};
constexpr Field FORCE_LIMIT_COLOR_SECTOR_TO_256B_MAX{5, 0x00000020};
} // namespace CB_COLOR0_ATTRIB
namespace CB_COLOR0_FDCC_CONTROL {
constexpr Register reg{0x031e, 1};
constexpr Field SAMPLE_MASK_TRACKER_DISABLE{0, 0x00000001};
constexpr Field SAMPLE_MASK_TRACKER_FEA_FORCE{1, 0x00000002};
constexpr Field MAX_UNCOMPRESSED_BLOCK_SIZE{2, 0x0000000c};
constexpr Field MIN_COMPRESSED_BLOCK_SIZE{4, 0x00000010};
constexpr Field MAX_COMPRESSED_BLOCK_SIZE{5, 0x00000060};
constexpr Field COLOR_TRANSFORM{7, 0x00000180};
constexpr Field INDEPENDENT_64B_BLOCKS{9, 0x00000200};
constexpr Field INDEPENDENT_128B_BLOCKS{10, 0x00000400};
constexpr Field DISABLE_CONSTANT_ENCODE_REG{18, 0x00040000};
constexpr Field ENABLE_CONSTANT_ENCODE_REG_WRITE{19, 0x00080000};
constexpr Field SKIP_LOW_COMP_RATIO{21, 0x00200000};
constexpr Field FDCC_ENABLE{22, 0x00400000};
constexpr Field DCC_COMPRESS_DISABLE{23, 0x00800000};
constexpr Field FRAGMENT_COMPRESS_DISABLE{24, 0x01000000};
} // namespace CB_COLOR0_FDCC_CONTROL
namespace CB_COLOR0_DCC_BASE {
constexpr Register reg{0x0325, 1};
constexpr Field BASE_256B{0, 0xffffffff};
} // namespace CB_COLOR0_DCC_BASE
namespace CB_COLOR0_DCC_BASE_EXT {
constexpr Register reg{0x03a8, 1};
constexpr Field BASE_256B{0, 0x000000ff};
} // namespace CB_COLOR0_DCC_BASE_EXT
namespace CB_COLOR0_ATTRIB2 {
constexpr Register reg{0x03b0, 1};
constexpr Field MIP0_HEIGHT{0, 0x00003fff};
constexpr Field MIP0_WIDTH{14, 0x0fffc000};
constexpr Field MAX_MIP{28, 0xf0000000};
} // namespace CB_COLOR0_ATTRIB2
namespace CB_COLOR0_ATTRIB3 {
constexpr Register reg{0x03b8, 1};
constexpr Field MIP0_DEPTH{0, 0x00001fff};
constexpr Field META_LINEAR{13, 0x00002000};
constexpr Field COLOR_SW_MODE{14, 0x0007c000};
constexpr Field RESOURCE_TYPE{24, 0x03000000};
constexpr Field DCC_PIPE_ALIGNED{30, 0x40000000};
} // namespace CB_COLOR0_ATTRIB3
namespace CB_COLOR1_INFO {
constexpr Register reg{0x032b, 1};
constexpr Field FORMAT{0, 0x0000001f};
constexpr Field LINEAR_GENERAL{7, 0x00000080};
constexpr Field NUMBER_TYPE{8, 0x00000700};
constexpr Field COMP_SWAP{11, 0x00001800};
constexpr Field BLEND_CLAMP{15, 0x00008000};
constexpr Field BLEND_BYPASS{16, 0x00010000};
constexpr Field SIMPLE_FLOAT{17, 0x00020000};
constexpr Field ROUND_MODE{18, 0x00040000};
constexpr Field BLEND_OPT_DONT_RD_DST{20, 0x00700000};
constexpr Field BLEND_OPT_DISCARD_PIXEL{23, 0x03800000};
} // namespace CB_COLOR1_INFO
namespace CB_COLOR2_INFO {
constexpr Register reg{0x033a, 1};
constexpr Field FORMAT{0, 0x0000001f};
constexpr Field LINEAR_GENERAL{7, 0x00000080};
constexpr Field NUMBER_TYPE{8, 0x00000700};
constexpr Field COMP_SWAP{11, 0x00001800};
constexpr Field BLEND_CLAMP{15, 0x00008000};
constexpr Field BLEND_BYPASS{16, 0x00010000};
constexpr Field SIMPLE_FLOAT{17, 0x00020000};
constexpr Field ROUND_MODE{18, 0x00040000};
constexpr Field BLEND_OPT_DONT_RD_DST{20, 0x00700000};
constexpr Field BLEND_OPT_DISCARD_PIXEL{23, 0x03800000};
} // namespace CB_COLOR2_INFO
namespace CB_COLOR3_INFO {
constexpr Register reg{0x0349, 1};
constexpr Field FORMAT{0, 0x0000001f};
constexpr Field LINEAR_GENERAL{7, 0x00000080};
constexpr Field NUMBER_TYPE{8, 0x00000700};
constexpr Field COMP_SWAP{11, 0x00001800};
constexpr Field BLEND_CLAMP{15, 0x00008000};
constexpr Field BLEND_BYPASS{16, 0x00010000};
constexpr Field SIMPLE_FLOAT{17, 0x00020000};
constexpr Field ROUND_MODE{18, 0x00040000};
constexpr Field BLEND_OPT_DONT_RD_DST{20, 0x00700000};
constexpr Field BLEND_OPT_DISCARD_PIXEL{23, 0x03800000};
} // namespace CB_COLOR3_INFO
namespace CB_COLOR4_INFO {
constexpr Register reg{0x0358, 1};
constexpr Field FORMAT{0, 0x0000001f};
constexpr Field LINEAR_GENERAL{7, 0x00000080};
constexpr Field NUMBER_TYPE{8, 0x00000700};
constexpr Field COMP_SWAP{11, 0x00001800};
constexpr Field BLEND_CLAMP{15, 0x00008000};
constexpr Field BLEND_BYPASS{16, 0x00010000};
constexpr Field SIMPLE_FLOAT{17, 0x00020000};
constexpr Field ROUND_MODE{18, 0x00040000};
constexpr Field BLEND_OPT_DONT_RD_DST{20, 0x00700000};
constexpr Field BLEND_OPT_DISCARD_PIXEL{23, 0x03800000};
} // namespace CB_COLOR4_INFO
namespace CB_COLOR5_INFO {
constexpr Register reg{0x0367, 1};
constexpr Field FORMAT{0, 0x0000001f};
constexpr Field LINEAR_GENERAL{7, 0x00000080};
constexpr Field NUMBER_TYPE{8, 0x00000700};
constexpr Field COMP_SWAP{11, 0x00001800};
constexpr Field BLEND_CLAMP{15, 0x00008000};
constexpr Field BLEND_BYPASS{16, 0x00010000};
constexpr Field SIMPLE_FLOAT{17, 0x00020000};
constexpr Field ROUND_MODE{18, 0x00040000};
constexpr Field BLEND_OPT_DONT_RD_DST{20, 0x00700000};
constexpr Field BLEND_OPT_DISCARD_PIXEL{23, 0x03800000};
} // namespace CB_COLOR5_INFO
namespace CB_COLOR6_INFO {
constexpr Register reg{0x0376, 1};
constexpr Field FORMAT{0, 0x0000001f};
constexpr Field LINEAR_GENERAL{7, 0x00000080};
constexpr Field NUMBER_TYPE{8, 0x00000700};
constexpr Field COMP_SWAP{11, 0x00001800};
constexpr Field BLEND_CLAMP{15, 0x00008000};
constexpr Field BLEND_BYPASS{16, 0x00010000};
constexpr Field SIMPLE_FLOAT{17, 0x00020000};
constexpr Field ROUND_MODE{18, 0x00040000};
constexpr Field BLEND_OPT_DONT_RD_DST{20, 0x00700000};
constexpr Field BLEND_OPT_DISCARD_PIXEL{23, 0x03800000};
} // namespace CB_COLOR6_INFO
namespace CB_COLOR7_INFO {
constexpr Register reg{0x0385, 1};
constexpr Field FORMAT{0, 0x0000001f};
constexpr Field LINEAR_GENERAL{7, 0x00000080};
constexpr Field NUMBER_TYPE{8, 0x00000700};
constexpr Field COMP_SWAP{11, 0x00001800};
constexpr Field BLEND_CLAMP{15, 0x00008000};
constexpr Field BLEND_BYPASS{16, 0x00010000};
constexpr Field SIMPLE_FLOAT{17, 0x00020000};
constexpr Field ROUND_MODE{18, 0x00040000};
constexpr Field BLEND_OPT_DONT_RD_DST{20, 0x00700000};
constexpr Field BLEND_OPT_DISCARD_PIXEL{23, 0x03800000};
} // namespace CB_COLOR7_INFO
namespace CB_TARGET_MASK {
constexpr Register reg{0x008e, 1};
constexpr Field TARGET0_ENABLE{0, 0x0000000f};
constexpr Field TARGET1_ENABLE{4, 0x000000f0};
constexpr Field TARGET2_ENABLE{8, 0x00000f00};
constexpr Field TARGET3_ENABLE{12, 0x0000f000};
constexpr Field TARGET4_ENABLE{16, 0x000f0000};
constexpr Field TARGET5_ENABLE{20, 0x00f00000};
constexpr Field TARGET6_ENABLE{24, 0x0f000000};
constexpr Field TARGET7_ENABLE{28, 0xf0000000};
} // namespace CB_TARGET_MASK
namespace CB_SHADER_MASK {
constexpr Register reg{0x008f, 1};
constexpr Field OUTPUT0_ENABLE{0, 0x0000000f};
constexpr Field OUTPUT1_ENABLE{4, 0x000000f0};
constexpr Field OUTPUT2_ENABLE{8, 0x00000f00};
constexpr Field OUTPUT3_ENABLE{12, 0x0000f000};
constexpr Field OUTPUT4_ENABLE{16, 0x000f0000};
constexpr Field OUTPUT5_ENABLE{20, 0x00f00000};
constexpr Field OUTPUT6_ENABLE{24, 0x0f000000};
constexpr Field OUTPUT7_ENABLE{28, 0xf0000000};
} // namespace CB_SHADER_MASK
namespace CB_COLOR_CONTROL {
constexpr Register reg{0x0202, 1};
constexpr Field DISABLE_DUAL_QUAD{0, 0x00000001};
constexpr Field ENABLE_1FRAG_PS_INVOKE{1, 0x00000002};
constexpr Field DEGAMMA_ENABLE{3, 0x00000008};
constexpr Field MODE{4, 0x00000070};
constexpr Field ROP3{16, 0x00ff0000};
} // namespace CB_COLOR_CONTROL
namespace CB_BLEND0_CONTROL {
constexpr Register reg{0x01e0, 1};
constexpr Field COLOR_SRCBLEND{0, 0x0000001f};
constexpr Field COLOR_COMB_FCN{5, 0x000000e0};
constexpr Field COLOR_DESTBLEND{8, 0x00001f00};
constexpr Field ALPHA_SRCBLEND{16, 0x001f0000};
constexpr Field ALPHA_COMB_FCN{21, 0x00e00000};
constexpr Field ALPHA_DESTBLEND{24, 0x1f000000};
constexpr Field SEPARATE_ALPHA_BLEND{29, 0x20000000};
constexpr Field ENABLE{30, 0x40000000};
constexpr Field DISABLE_ROP3{31, 0x80000000};
} // namespace CB_BLEND0_CONTROL
namespace CB_BLEND_RED {
constexpr Register reg{0x0105, 1};
constexpr Field BLEND_RED{0, 0xffffffff};
} // namespace CB_BLEND_RED
namespace CB_BLEND_GREEN {
constexpr Register reg{0x0106, 1};
constexpr Field BLEND_GREEN{0, 0xffffffff};
} // namespace CB_BLEND_GREEN
namespace CB_BLEND_BLUE {
constexpr Register reg{0x0107, 1};
constexpr Field BLEND_BLUE{0, 0xffffffff};
} // namespace CB_BLEND_BLUE
namespace CB_BLEND_ALPHA {
constexpr Register reg{0x0108, 1};
constexpr Field BLEND_ALPHA{0, 0xffffffff};
} // namespace CB_BLEND_ALPHA
namespace CB_RMI_GL2_CACHE_CONTROL {
constexpr Register reg{0x0104, 1};
constexpr Field DCC_WR_POLICY{0, 0x00000003};
constexpr Field COLOR_WR_POLICY{2, 0x0000000c};
constexpr Field DCC_RD_POLICY{20, 0x00300000};
constexpr Field COLOR_RD_POLICY{22, 0x00c00000};
constexpr Field DCC_L3_BYPASS{26, 0x04000000};
constexpr Field COLOR_L3_BYPASS{27, 0x08000000};
constexpr Field COLOR_BIG_PAGE{31, 0x80000000};
} // namespace CB_RMI_GL2_CACHE_CONTROL
namespace CB_FDCC_CONTROL {
constexpr Register reg{0x0109, 1};
constexpr Field SAMPLE_MASK_TRACKER_DISABLE{0, 0x00000001};
constexpr Field SAMPLE_MASK_TRACKER_WATERMARK{2, 0x0000007c};
constexpr Field DISABLE_CONSTANT_ENCODE_AC01{8, 0x00000100};
constexpr Field DISABLE_CONSTANT_ENCODE_SINGLE{9, 0x00000200};
constexpr Field DISABLE_CONSTANT_ENCODE_REG{10, 0x00000400};
constexpr Field DISABLE_ELIMFC_SKIP_OF_AC01{12, 0x00001000};
constexpr Field DISABLE_ELIMFC_SKIP_OF_SINGLE{13, 0x00002000};
constexpr Field ENABLE_ELIMFC_SKIP_OF_REG{14, 0x00004000};
} // namespace CB_FDCC_CONTROL
namespace SX_PS_DOWNCONVERT {
constexpr Register reg{0x01d5, 1};
constexpr Field MRT0{0, 0x0000000f};
constexpr Field MRT1{4, 0x000000f0};
constexpr Field MRT2{8, 0x00000f00};
constexpr Field MRT3{12, 0x0000f000};
constexpr Field MRT4{16, 0x000f0000};
constexpr Field MRT5{20, 0x00f00000};
constexpr Field MRT6{24, 0x0f000000};
constexpr Field MRT7{28, 0xf0000000};
} // namespace SX_PS_DOWNCONVERT
namespace SX_BLEND_OPT_EPSILON {
constexpr Register reg{0x01d6, 1};
constexpr Field MRT0_EPSILON{0, 0x0000000f};
constexpr Field MRT1_EPSILON{4, 0x000000f0};
constexpr Field MRT2_EPSILON{8, 0x00000f00};
constexpr Field MRT3_EPSILON{12, 0x0000f000};
constexpr Field MRT4_EPSILON{16, 0x000f0000};
constexpr Field MRT5_EPSILON{20, 0x00f00000};
constexpr Field MRT6_EPSILON{24, 0x0f000000};
constexpr Field MRT7_EPSILON{28, 0xf0000000};
} // namespace SX_BLEND_OPT_EPSILON
namespace SX_BLEND_OPT_CONTROL {
constexpr Register reg{0x01d7, 1};
constexpr Field MRT0_COLOR_OPT_DISABLE{0, 0x00000001};
constexpr Field MRT0_ALPHA_OPT_DISABLE{1, 0x00000002};
constexpr Field MRT1_COLOR_OPT_DISABLE{4, 0x00000010};
constexpr Field MRT1_ALPHA_OPT_DISABLE{5, 0x00000020};
constexpr Field MRT2_COLOR_OPT_DISABLE{8, 0x00000100};
constexpr Field MRT2_ALPHA_OPT_DISABLE{9, 0x00000200};
constexpr Field MRT3_COLOR_OPT_DISABLE{12, 0x00001000};
constexpr Field MRT3_ALPHA_OPT_DISABLE{13, 0x00002000};
constexpr Field MRT4_COLOR_OPT_DISABLE{16, 0x00010000};
constexpr Field MRT4_ALPHA_OPT_DISABLE{17, 0x00020000};
constexpr Field MRT5_COLOR_OPT_DISABLE{20, 0x00100000};
constexpr Field MRT5_ALPHA_OPT_DISABLE{21, 0x00200000};
constexpr Field MRT6_COLOR_OPT_DISABLE{24, 0x01000000};
constexpr Field MRT6_ALPHA_OPT_DISABLE{25, 0x02000000};
constexpr Field MRT7_COLOR_OPT_DISABLE{28, 0x10000000};
constexpr Field MRT7_ALPHA_OPT_DISABLE{29, 0x20000000};
constexpr Field PIXEN_ZERO_OPT_DISABLE{31, 0x80000000};
} // namespace SX_BLEND_OPT_CONTROL
namespace SX_MRT0_BLEND_OPT {
constexpr Register reg{0x01d8, 1};
constexpr Field COLOR_SRC_OPT{0, 0x00000007};
constexpr Field COLOR_DST_OPT{4, 0x00000070};
constexpr Field COLOR_COMB_FCN{8, 0x00000700};
constexpr Field ALPHA_SRC_OPT{16, 0x00070000};
constexpr Field ALPHA_DST_OPT{20, 0x00700000};
constexpr Field ALPHA_COMB_FCN{24, 0x07000000};
} // namespace SX_MRT0_BLEND_OPT
namespace SX_PS_DOWNCONVERT_CONTROL {
constexpr Register reg{0x01d4, 1};
constexpr Field MRT0_FMT_MAPPING_DISABLE{0, 0x00000001};
constexpr Field MRT1_FMT_MAPPING_DISABLE{1, 0x00000002};
constexpr Field MRT2_FMT_MAPPING_DISABLE{2, 0x00000004};
constexpr Field MRT3_FMT_MAPPING_DISABLE{3, 0x00000008};
constexpr Field MRT4_FMT_MAPPING_DISABLE{4, 0x00000010};
constexpr Field MRT5_FMT_MAPPING_DISABLE{5, 0x00000020};
constexpr Field MRT6_FMT_MAPPING_DISABLE{6, 0x00000040};
constexpr Field MRT7_FMT_MAPPING_DISABLE{7, 0x00000080};
} // namespace SX_PS_DOWNCONVERT_CONTROL
namespace SPI_SHADER_POS_FORMAT {
constexpr Register reg{0x01c3, 1};
constexpr Field POS0_EXPORT_FORMAT{0, 0x0000000f};
constexpr Field POS1_EXPORT_FORMAT{4, 0x000000f0};
constexpr Field POS2_EXPORT_FORMAT{8, 0x00000f00};
constexpr Field POS3_EXPORT_FORMAT{12, 0x0000f000};
constexpr Field POS4_EXPORT_FORMAT{16, 0x000f0000};
} // namespace SPI_SHADER_POS_FORMAT
namespace SPI_SHADER_IDX_FORMAT {
constexpr Register reg{0x01c2, 1};
constexpr Field IDX0_EXPORT_FORMAT{0, 0x0000000f};
} // namespace SPI_SHADER_IDX_FORMAT
namespace SPI_SHADER_Z_FORMAT {
constexpr Register reg{0x01c4, 1};
constexpr Field Z_EXPORT_FORMAT{0, 0x0000000f};
} // namespace SPI_SHADER_Z_FORMAT
namespace SPI_SHADER_COL_FORMAT {
constexpr Register reg{0x01c5, 1};
constexpr Field COL0_EXPORT_FORMAT{0, 0x0000000f};
constexpr Field COL1_EXPORT_FORMAT{4, 0x000000f0};
constexpr Field COL2_EXPORT_FORMAT{8, 0x00000f00};
constexpr Field COL3_EXPORT_FORMAT{12, 0x0000f000};
constexpr Field COL4_EXPORT_FORMAT{16, 0x000f0000};
constexpr Field COL5_EXPORT_FORMAT{20, 0x00f00000};
constexpr Field COL6_EXPORT_FORMAT{24, 0x0f000000};
constexpr Field COL7_EXPORT_FORMAT{28, 0xf0000000};
} // namespace SPI_SHADER_COL_FORMAT
namespace SPI_PS_INPUT_ENA {
constexpr Register reg{0x01b3, 1};
constexpr Field PERSP_SAMPLE_ENA{0, 0x00000001};
constexpr Field PERSP_CENTER_ENA{1, 0x00000002};
constexpr Field PERSP_CENTROID_ENA{2, 0x00000004};
constexpr Field PERSP_PULL_MODEL_ENA{3, 0x00000008};
constexpr Field LINEAR_SAMPLE_ENA{4, 0x00000010};
constexpr Field LINEAR_CENTER_ENA{5, 0x00000020};
constexpr Field LINEAR_CENTROID_ENA{6, 0x00000040};
constexpr Field LINE_STIPPLE_TEX_ENA{7, 0x00000080};
constexpr Field POS_X_FLOAT_ENA{8, 0x00000100};
constexpr Field POS_Y_FLOAT_ENA{9, 0x00000200};
constexpr Field POS_Z_FLOAT_ENA{10, 0x00000400};
constexpr Field POS_W_FLOAT_ENA{11, 0x00000800};
constexpr Field FRONT_FACE_ENA{12, 0x00001000};
constexpr Field ANCILLARY_ENA{13, 0x00002000};
constexpr Field SAMPLE_COVERAGE_ENA{14, 0x00004000};
constexpr Field POS_FIXED_PT_ENA{15, 0x00008000};
} // namespace SPI_PS_INPUT_ENA
namespace SPI_PS_INPUT_ADDR {
constexpr Register reg{0x01b4, 1};
constexpr Field PERSP_SAMPLE_ENA{0, 0x00000001};
constexpr Field PERSP_CENTER_ENA{1, 0x00000002};
constexpr Field PERSP_CENTROID_ENA{2, 0x00000004};
constexpr Field PERSP_PULL_MODEL_ENA{3, 0x00000008};
constexpr Field LINEAR_SAMPLE_ENA{4, 0x00000010};
constexpr Field LINEAR_CENTER_ENA{5, 0x00000020};
constexpr Field LINEAR_CENTROID_ENA{6, 0x00000040};
constexpr Field LINE_STIPPLE_TEX_ENA{7, 0x00000080};
constexpr Field POS_X_FLOAT_ENA{8, 0x00000100};
constexpr Field POS_Y_FLOAT_ENA{9, 0x00000200};
constexpr Field POS_Z_FLOAT_ENA{10, 0x00000400};
constexpr Field POS_W_FLOAT_ENA{11, 0x00000800};
constexpr Field FRONT_FACE_ENA{12, 0x00001000};
constexpr Field ANCILLARY_ENA{13, 0x00002000};
constexpr Field SAMPLE_COVERAGE_ENA{14, 0x00004000};
constexpr Field POS_FIXED_PT_ENA{15, 0x00008000};
} // namespace SPI_PS_INPUT_ADDR
namespace SPI_PS_IN_CONTROL {
constexpr Register reg{0x01b6, 1};
constexpr Field NUM_INTERP{0, 0x0000003f};
constexpr Field PARAM_GEN{6, 0x00000040};
constexpr Field OFFCHIP_PARAM_EN{7, 0x00000080};
constexpr Field LATE_PC_DEALLOC{8, 0x00000100};
constexpr Field NUM_PRIM_INTERP{9, 0x00003e00};
constexpr Field BC_OPTIMIZE_DISABLE{14, 0x00004000};
constexpr Field PS_W32_EN{15, 0x00008000};
} // namespace SPI_PS_IN_CONTROL
namespace SPI_BARYC_CNTL {
constexpr Register reg{0x01b8, 1};
constexpr Field PERSP_CENTER_CNTL{0, 0x00000001};
constexpr Field PERSP_CENTROID_CNTL{4, 0x00000010};
constexpr Field LINEAR_CENTER_CNTL{8, 0x00000100};
constexpr Field LINEAR_CENTROID_CNTL{12, 0x00001000};
constexpr Field POS_FLOAT_LOCATION{16, 0x00030000};
constexpr Field POS_FLOAT_ULC{20, 0x00100000};
constexpr Field FRONT_FACE_ALL_BITS{24, 0x01000000};
} // namespace SPI_BARYC_CNTL
namespace SPI_PS_INPUT_CNTL_0 {
constexpr Register reg{0x0191, 1};
constexpr Field OFFSET{0, 0x0000003f};
constexpr Field DEFAULT_VAL{8, 0x00000300};
constexpr Field FLAT_SHADE{10, 0x00000400};
constexpr Field ROTATE_PC_PTR{11, 0x00000800};
constexpr Field PRIM_ATTR{12, 0x00001000};
constexpr Field PT_SPRITE_TEX{17, 0x00020000};
constexpr Field DUP{18, 0x00040000};
constexpr Field FP16_INTERP_MODE{19, 0x00080000};
constexpr Field USE_DEFAULT_ATTR1{20, 0x00100000};
constexpr Field DEFAULT_VAL_ATTR1{21, 0x00600000};
constexpr Field PT_SPRITE_TEX_ATTR1{23, 0x00800000};
constexpr Field ATTR0_VALID{24, 0x01000000};
constexpr Field ATTR1_VALID{25, 0x02000000};
} // namespace SPI_PS_INPUT_CNTL_0
namespace SPI_PS_INPUT_CNTL_1 {
constexpr Register reg{0x0192, 1};
constexpr Field OFFSET{0, 0x0000003f};
constexpr Field DEFAULT_VAL{8, 0x00000300};
constexpr Field FLAT_SHADE{10, 0x00000400};
constexpr Field ROTATE_PC_PTR{11, 0x00000800};
constexpr Field PRIM_ATTR{12, 0x00001000};
constexpr Field PT_SPRITE_TEX{17, 0x00020000};
constexpr Field DUP{18, 0x00040000};
constexpr Field FP16_INTERP_MODE{19, 0x00080000};
constexpr Field USE_DEFAULT_ATTR1{20, 0x00100000};
constexpr Field DEFAULT_VAL_ATTR1{21, 0x00600000};
constexpr Field PT_SPRITE_TEX_ATTR1{23, 0x00800000};
constexpr Field ATTR0_VALID{24, 0x01000000};
constexpr Field ATTR1_VALID{25, 0x02000000};
} // namespace SPI_PS_INPUT_CNTL_1
namespace SPI_PS_INPUT_CNTL_2 {
constexpr Register reg{0x0193, 1};
constexpr Field OFFSET{0, 0x0000003f};
constexpr Field DEFAULT_VAL{8, 0x00000300};
constexpr Field FLAT_SHADE{10, 0x00000400};
constexpr Field ROTATE_PC_PTR{11, 0x00000800};
constexpr Field PRIM_ATTR{12, 0x00001000};
constexpr Field PT_SPRITE_TEX{17, 0x00020000};
constexpr Field DUP{18, 0x00040000};
constexpr Field FP16_INTERP_MODE{19, 0x00080000};
constexpr Field USE_DEFAULT_ATTR1{20, 0x00100000};
constexpr Field DEFAULT_VAL_ATTR1{21, 0x00600000};
constexpr Field PT_SPRITE_TEX_ATTR1{23, 0x00800000};
constexpr Field ATTR0_VALID{24, 0x01000000};
constexpr Field ATTR1_VALID{25, 0x02000000};
} // namespace SPI_PS_INPUT_CNTL_2
namespace SPI_PS_INPUT_CNTL_3 {
constexpr Register reg{0x0194, 1};
constexpr Field OFFSET{0, 0x0000003f};
constexpr Field DEFAULT_VAL{8, 0x00000300};
constexpr Field FLAT_SHADE{10, 0x00000400};
constexpr Field ROTATE_PC_PTR{11, 0x00000800};
constexpr Field PRIM_ATTR{12, 0x00001000};
constexpr Field PT_SPRITE_TEX{17, 0x00020000};
constexpr Field DUP{18, 0x00040000};
constexpr Field FP16_INTERP_MODE{19, 0x00080000};
constexpr Field USE_DEFAULT_ATTR1{20, 0x00100000};
constexpr Field DEFAULT_VAL_ATTR1{21, 0x00600000};
constexpr Field PT_SPRITE_TEX_ATTR1{23, 0x00800000};
constexpr Field ATTR0_VALID{24, 0x01000000};
constexpr Field ATTR1_VALID{25, 0x02000000};
} // namespace SPI_PS_INPUT_CNTL_3
namespace SPI_VS_OUT_CONFIG {
constexpr Register reg{0x01b1, 1};
constexpr Field VS_EXPORT_COUNT{1, 0x0000003e};
constexpr Field NO_PC_EXPORT{7, 0x00000080};
constexpr Field PRIM_EXPORT_COUNT{8, 0x00001f00};
} // namespace SPI_VS_OUT_CONFIG
namespace SPI_INTERP_CONTROL_0 {
constexpr Register reg{0x01b5, 1};
constexpr Field FLAT_SHADE_ENA{0, 0x00000001};
constexpr Field PNT_SPRITE_ENA{1, 0x00000002};
constexpr Field PNT_SPRITE_OVRD_X{2, 0x0000001c};
constexpr Field PNT_SPRITE_OVRD_Y{5, 0x000000e0};
constexpr Field PNT_SPRITE_OVRD_Z{8, 0x00000700};
constexpr Field PNT_SPRITE_OVRD_W{11, 0x00003800};
constexpr Field PNT_SPRITE_TOP_1{14, 0x00004000};
} // namespace SPI_INTERP_CONTROL_0
namespace SPI_TMPRING_SIZE {
constexpr Register reg{0x01ba, 1};
constexpr Field WAVES{0, 0x00000fff};
constexpr Field WAVESIZE{12, 0x07fff000};
} // namespace SPI_TMPRING_SIZE
namespace VGT_SHADER_STAGES_EN {
constexpr Register reg{0x02d5, 1};
constexpr Field LS_EN{0, 0x00000003};
constexpr Field HS_EN{2, 0x00000004};
constexpr Field ES_EN{3, 0x00000018};
constexpr Field GS_EN{5, 0x00000020};
constexpr Field VS_EN{6, 0x000000c0};
constexpr Field DYNAMIC_HS{8, 0x00000100};
constexpr Field VS_WAVE_ID_EN{12, 0x00001000};
constexpr Field PRIMGEN_EN{13, 0x00002000};
constexpr Field ORDERED_ID_MODE{14, 0x00004000};
constexpr Field MAX_PRIMGRP_IN_WAVE{15, 0x00078000};
constexpr Field GS_FAST_LAUNCH{19, 0x00180000};
constexpr Field HS_W32_EN{21, 0x00200000};
constexpr Field GS_W32_EN{22, 0x00400000};
constexpr Field VS_W32_EN{23, 0x00800000};
constexpr Field NGG_WAVE_ID_EN{24, 0x01000000};
constexpr Field PRIMGEN_PASSTHRU_EN{25, 0x02000000};
constexpr Field PRIMGEN_PASSTHRU_NO_MSG{26, 0x04000000};
} // namespace VGT_SHADER_STAGES_EN
namespace VGT_GS_MAX_VERT_OUT {
constexpr Register reg{0x02ce, 1};
constexpr Field MAX_VERT_OUT{0, 0x000007ff};
} // namespace VGT_GS_MAX_VERT_OUT
namespace VGT_ESGS_RING_ITEMSIZE {
constexpr Register reg{0x02ab, 1};
constexpr Field ITEMSIZE{0, 0x00007fff};
} // namespace VGT_ESGS_RING_ITEMSIZE
namespace VGT_REUSE_OFF {
constexpr Register reg{0x02ad, 1};
constexpr Field REUSE_OFF{0, 0x00000001};
} // namespace VGT_REUSE_OFF
namespace VGT_PRIMITIVEID_EN {
constexpr Register reg{0x02a1, 1};
constexpr Field PRIMITIVEID_EN{0, 0x00000001};
constexpr Field DISABLE_RESET_ON_EOI{1, 0x00000002};
constexpr Field NGG_DISABLE_PROVOK_REUSE{2, 0x00000004};
} // namespace VGT_PRIMITIVEID_EN
namespace VGT_DRAW_PAYLOAD_CNTL {
constexpr Register reg{0x02a6, 1};
constexpr Field EN_REG_RT_INDEX{1, 0x00000002};
constexpr Field EN_PRIM_PAYLOAD{3, 0x00000008};
constexpr Field EN_DRAW_VP{4, 0x00000010};
} // namespace VGT_DRAW_PAYLOAD_CNTL
namespace VGT_GS_INSTANCE_CNT {
constexpr Register reg{0x02e4, 1};
constexpr Field ENABLE{0, 0x00000001};
constexpr Field CNT{2, 0x000001fc};
constexpr Field EN_MAX_VERT_OUT_PER_GS_INSTANCE{31, 0x80000000};
} // namespace VGT_GS_INSTANCE_CNT
namespace VGT_MULTI_PRIM_IB_RESET_INDX {
constexpr Register reg{0x0103, 1};
constexpr Field RESET_INDX{0, 0xffffffff};
} // namespace VGT_MULTI_PRIM_IB_RESET_INDX
namespace VGT_HOS_MAX_TESS_LEVEL {
constexpr Register reg{0x0286, 1};
constexpr Field MAX_TESS{0, 0xffffffff};
} // namespace VGT_HOS_MAX_TESS_LEVEL
namespace VGT_TESS_DISTRIBUTION {
constexpr Register reg{0x02d4, 1};
constexpr Field ACCUM_ISOLINE{0, 0x000000ff};
constexpr Field ACCUM_TRI{8, 0x0000ff00};
constexpr Field ACCUM_QUAD{16, 0x00ff0000};
constexpr Field DONUT_SPLIT{24, 0x1f000000};
constexpr Field TRAP_SPLIT{29, 0xe0000000};
} // namespace VGT_TESS_DISTRIBUTION
namespace GE_NGG_SUBGRP_CNTL {
constexpr Register reg{0x02d3, 1};
constexpr Field PRIM_AMP_FACTOR{0, 0x000001ff};
constexpr Field THDS_PER_SUBGRP{9, 0x0003fe00};
} // namespace GE_NGG_SUBGRP_CNTL
namespace GE_MAX_OUTPUT_PER_SUBGROUP {
constexpr Register reg{0x01ff, 1};
constexpr Field MAX_VERTS_PER_SUBGROUP{0, 0x000003ff};
} // namespace GE_MAX_OUTPUT_PER_SUBGROUP
namespace SPI_SHADER_PGM_LO_ES {
constexpr Register reg{0x1a68, 0};
constexpr Field MEM_BASE{0, 0xffffffff};
} // namespace SPI_SHADER_PGM_LO_ES
namespace SPI_SHADER_PGM_HI_ES {
constexpr Register reg{0x1a69, 0};
constexpr Field MEM_BASE{0, 0x000000ff};
} // namespace SPI_SHADER_PGM_HI_ES
namespace SPI_SHADER_PGM_RSRC1_GS {
constexpr Register reg{0x1a2a, 0};
constexpr Field VGPRS{0, 0x0000003f};
constexpr Field SGPRS{6, 0x000003c0};
constexpr Field PRIORITY{10, 0x00000c00};
constexpr Field FLOAT_MODE{12, 0x000ff000};
constexpr Field PRIV{20, 0x00100000};
constexpr Field DX10_CLAMP{21, 0x00200000};
constexpr Field IEEE_MODE{23, 0x00800000};
constexpr Field CU_GROUP_ENABLE{24, 0x01000000};
constexpr Field MEM_ORDERED{25, 0x02000000};
constexpr Field FWD_PROGRESS{26, 0x04000000};
constexpr Field WGP_MODE{27, 0x08000000};
constexpr Field GS_VGPR_COMP_CNT{29, 0x60000000};
constexpr Field FP16_OVFL{31, 0x80000000};
} // namespace SPI_SHADER_PGM_RSRC1_GS
namespace SPI_SHADER_PGM_RSRC2_GS {
constexpr Register reg{0x1a2b, 0};
constexpr Field SCRATCH_EN{0, 0x00000001};
constexpr Field USER_SGPR{1, 0x0000003e};
constexpr Field TRAP_PRESENT{6, 0x00000040};
constexpr Field EXCP_EN{7, 0x0000ff80};
constexpr Field ES_VGPR_COMP_CNT{16, 0x00030000};
constexpr Field OC_LDS_EN{18, 0x00040000};
constexpr Field LDS_SIZE{19, 0x07f80000};
constexpr Field USER_SGPR_MSB{27, 0x08000000};
constexpr Field SHARED_VGPR_CNT{28, 0xf0000000};
} // namespace SPI_SHADER_PGM_RSRC2_GS
namespace SPI_SHADER_PGM_RSRC3_GS {
constexpr Register reg{0x1a27, 0};
constexpr Field CU_EN{0, 0x0000ffff};
constexpr Field WAVE_LIMIT{16, 0x003f0000};
constexpr Field LOCK_LOW_THRESHOLD{22, 0x03c00000};
constexpr Field GROUP_FIFO_DEPTH{26, 0xfc000000};
} // namespace SPI_SHADER_PGM_RSRC3_GS
namespace SPI_SHADER_PGM_RSRC4_GS {
constexpr Register reg{0x1a21, 0};
constexpr Field CU_EN{0, 0x00000001};
constexpr Field RESERVED{1, 0x00003ffe};
constexpr Field PH_THROTTLE_EN{14, 0x00004000};
constexpr Field SPI_THROTTLE_EN{15, 0x00008000};
constexpr Field SPI_SHADER_LATE_ALLOC_GS{16, 0x007f0000};
constexpr Field INST_PREF_SIZE{23, 0x1f800000};
constexpr Field TRAP_ON_START{29, 0x20000000};
constexpr Field TRAP_ON_END{30, 0x40000000};
constexpr Field IMAGE_OP{31, 0x80000000};
} // namespace SPI_SHADER_PGM_RSRC4_GS
namespace SPI_SHADER_USER_DATA_GS_0 {
constexpr Register reg{0x1a2c, 0};
constexpr Field DATA{0, 0xffffffff};
} // namespace SPI_SHADER_USER_DATA_GS_0
namespace SPI_SHADER_USER_DATA_GS_1 {
constexpr Register reg{0x1a2d, 0};
constexpr Field DATA{0, 0xffffffff};
} // namespace SPI_SHADER_USER_DATA_GS_1
namespace SPI_SHADER_USER_DATA_GS_2 {
constexpr Register reg{0x1a2e, 0};
constexpr Field DATA{0, 0xffffffff};
} // namespace SPI_SHADER_USER_DATA_GS_2
namespace SPI_SHADER_USER_DATA_GS_3 {
constexpr Register reg{0x1a2f, 0};
constexpr Field DATA{0, 0xffffffff};
} // namespace SPI_SHADER_USER_DATA_GS_3
namespace SPI_SHADER_USER_DATA_GS_4 {
constexpr Register reg{0x1a30, 0};
constexpr Field DATA{0, 0xffffffff};
} // namespace SPI_SHADER_USER_DATA_GS_4
namespace SPI_SHADER_USER_DATA_GS_5 {
constexpr Register reg{0x1a31, 0};
constexpr Field DATA{0, 0xffffffff};
} // namespace SPI_SHADER_USER_DATA_GS_5
namespace SPI_SHADER_USER_DATA_GS_6 {
constexpr Register reg{0x1a32, 0};
constexpr Field DATA{0, 0xffffffff};
} // namespace SPI_SHADER_USER_DATA_GS_6
namespace SPI_SHADER_USER_DATA_GS_7 {
constexpr Register reg{0x1a33, 0};
constexpr Field DATA{0, 0xffffffff};
} // namespace SPI_SHADER_USER_DATA_GS_7
namespace SPI_SHADER_USER_DATA_GS_8 {
constexpr Register reg{0x1a34, 0};
constexpr Field DATA{0, 0xffffffff};
} // namespace SPI_SHADER_USER_DATA_GS_8
namespace SPI_SHADER_USER_DATA_GS_9 {
constexpr Register reg{0x1a35, 0};
constexpr Field DATA{0, 0xffffffff};
} // namespace SPI_SHADER_USER_DATA_GS_9
namespace SPI_SHADER_USER_DATA_GS_10 {
constexpr Register reg{0x1a36, 0};
constexpr Field DATA{0, 0xffffffff};
} // namespace SPI_SHADER_USER_DATA_GS_10
namespace SPI_SHADER_USER_DATA_GS_11 {
constexpr Register reg{0x1a37, 0};
constexpr Field DATA{0, 0xffffffff};
} // namespace SPI_SHADER_USER_DATA_GS_11
namespace SPI_SHADER_USER_DATA_GS_12 {
constexpr Register reg{0x1a38, 0};
constexpr Field DATA{0, 0xffffffff};
} // namespace SPI_SHADER_USER_DATA_GS_12
namespace SPI_SHADER_PGM_LO_PS {
constexpr Register reg{0x19a8, 0};
constexpr Field MEM_BASE{0, 0xffffffff};
} // namespace SPI_SHADER_PGM_LO_PS
namespace SPI_SHADER_PGM_HI_PS {
constexpr Register reg{0x19a9, 0};
constexpr Field MEM_BASE{0, 0x000000ff};
} // namespace SPI_SHADER_PGM_HI_PS
namespace SPI_SHADER_PGM_RSRC1_PS {
constexpr Register reg{0x19aa, 0};
constexpr Field VGPRS{0, 0x0000003f};
constexpr Field SGPRS{6, 0x000003c0};
constexpr Field PRIORITY{10, 0x00000c00};
constexpr Field FLOAT_MODE{12, 0x000ff000};
constexpr Field PRIV{20, 0x00100000};
constexpr Field DX10_CLAMP{21, 0x00200000};
constexpr Field IEEE_MODE{23, 0x00800000};
constexpr Field CU_GROUP_DISABLE{24, 0x01000000};
constexpr Field MEM_ORDERED{25, 0x02000000};
constexpr Field FWD_PROGRESS{26, 0x04000000};
constexpr Field LOAD_PROVOKING_VTX{27, 0x08000000};
constexpr Field FP16_OVFL{29, 0x20000000};
} // namespace SPI_SHADER_PGM_RSRC1_PS
namespace SPI_SHADER_PGM_RSRC2_PS {
constexpr Register reg{0x19ab, 0};
constexpr Field SCRATCH_EN{0, 0x00000001};
constexpr Field USER_SGPR{1, 0x0000003e};
constexpr Field TRAP_PRESENT{6, 0x00000040};
constexpr Field WAVE_CNT_EN{7, 0x00000080};
constexpr Field EXTRA_LDS_SIZE{8, 0x0000ff00};
constexpr Field EXCP_EN{16, 0x01ff0000};
constexpr Field LOAD_COLLISION_WAVEID{25, 0x02000000};
constexpr Field LOAD_INTRAWAVE_COLLISION{26, 0x04000000};
constexpr Field USER_SGPR_MSB{27, 0x08000000};
constexpr Field SHARED_VGPR_CNT{28, 0xf0000000};
} // namespace SPI_SHADER_PGM_RSRC2_PS
namespace SPI_SHADER_PGM_RSRC3_PS {
constexpr Register reg{0x19a7, 0};
constexpr Field CU_EN{0, 0x0000ffff};
constexpr Field WAVE_LIMIT{16, 0x003f0000};
constexpr Field LDS_GROUP_SIZE{22, 0x00c00000};
} // namespace SPI_SHADER_PGM_RSRC3_PS
namespace SPI_SHADER_PGM_RSRC4_PS {
constexpr Register reg{0x19a1, 0};
constexpr Field CU_EN{0, 0x0000ffff};
constexpr Field INST_PREF_SIZE{16, 0x003f0000};
constexpr Field TRAP_ON_START{29, 0x20000000};
constexpr Field TRAP_ON_END{30, 0x40000000};
constexpr Field IMAGE_OP{31, 0x80000000};
} // namespace SPI_SHADER_PGM_RSRC4_PS
namespace SPI_SHADER_REQ_CTRL_PS {
constexpr Register reg{0x19d0, 0};
constexpr Field SOFT_GROUPING_EN{0, 0x00000001};
constexpr Field NUMBER_OF_REQUESTS_PER_CU{1, 0x0000001e};
constexpr Field SOFT_GROUPING_ALLOCATION_TIMEOUT{5, 0x000001e0};
constexpr Field HARD_LOCK_HYSTERESIS{9, 0x00000200};
constexpr Field HARD_LOCK_LOW_THRESHOLD{10, 0x00007c00};
constexpr Field PRODUCER_REQUEST_LOCKOUT{15, 0x00008000};
constexpr Field GLOBAL_SCANNING_EN{16, 0x00010000};
constexpr Field ALLOCATION_RATE_THROTTLING_THRESHOLD{17, 0x000e0000};
} // namespace SPI_SHADER_REQ_CTRL_PS
namespace SPI_SHADER_USER_DATA_PS_0 {
constexpr Register reg{0x19ac, 0};
constexpr Field DATA{0, 0xffffffff};
} // namespace SPI_SHADER_USER_DATA_PS_0
namespace SPI_SHADER_USER_DATA_PS_1 {
constexpr Register reg{0x19ad, 0};
constexpr Field DATA{0, 0xffffffff};
} // namespace SPI_SHADER_USER_DATA_PS_1
namespace SPI_SHADER_USER_DATA_PS_2 {
constexpr Register reg{0x19ae, 0};
constexpr Field DATA{0, 0xffffffff};
} // namespace SPI_SHADER_USER_DATA_PS_2
namespace SPI_SHADER_USER_DATA_PS_3 {
constexpr Register reg{0x19af, 0};
constexpr Field DATA{0, 0xffffffff};
} // namespace SPI_SHADER_USER_DATA_PS_3
namespace SPI_SHADER_USER_DATA_PS_4 {
constexpr Register reg{0x19b0, 0};
constexpr Field DATA{0, 0xffffffff};
} // namespace SPI_SHADER_USER_DATA_PS_4
namespace SPI_SHADER_USER_DATA_PS_5 {
constexpr Register reg{0x19b1, 0};
constexpr Field DATA{0, 0xffffffff};
} // namespace SPI_SHADER_USER_DATA_PS_5
namespace SPI_SHADER_USER_DATA_PS_6 {
constexpr Register reg{0x19b2, 0};
constexpr Field DATA{0, 0xffffffff};
} // namespace SPI_SHADER_USER_DATA_PS_6
namespace SPI_SHADER_USER_DATA_PS_7 {
constexpr Register reg{0x19b3, 0};
constexpr Field DATA{0, 0xffffffff};
} // namespace SPI_SHADER_USER_DATA_PS_7
namespace SPI_SHADER_USER_DATA_PS_8 {
constexpr Register reg{0x19b4, 0};
constexpr Field DATA{0, 0xffffffff};
} // namespace SPI_SHADER_USER_DATA_PS_8
namespace SPI_SHADER_USER_DATA_PS_9 {
constexpr Register reg{0x19b5, 0};
constexpr Field DATA{0, 0xffffffff};
} // namespace SPI_SHADER_USER_DATA_PS_9
namespace SPI_SHADER_USER_DATA_PS_10 {
constexpr Register reg{0x19b6, 0};
constexpr Field DATA{0, 0xffffffff};
} // namespace SPI_SHADER_USER_DATA_PS_10
namespace SPI_SHADER_USER_DATA_PS_11 {
constexpr Register reg{0x19b7, 0};
constexpr Field DATA{0, 0xffffffff};
} // namespace SPI_SHADER_USER_DATA_PS_11
namespace SPI_SHADER_USER_DATA_PS_12 {
constexpr Register reg{0x19b8, 0};
constexpr Field DATA{0, 0xffffffff};
} // namespace SPI_SHADER_USER_DATA_PS_12
namespace SPI_SHADER_USER_DATA_PS_13 {
constexpr Register reg{0x19b9, 0};
constexpr Field DATA{0, 0xffffffff};
} // namespace SPI_SHADER_USER_DATA_PS_13
namespace SPI_SHADER_USER_DATA_PS_14 {
constexpr Register reg{0x19ba, 0};
constexpr Field DATA{0, 0xffffffff};
} // namespace SPI_SHADER_USER_DATA_PS_14
namespace SPI_SHADER_USER_DATA_PS_15 {
constexpr Register reg{0x19bb, 0};
constexpr Field DATA{0, 0xffffffff};
} // namespace SPI_SHADER_USER_DATA_PS_15
namespace SPI_SHADER_USER_DATA_PS_16 {
constexpr Register reg{0x19bc, 0};
constexpr Field DATA{0, 0xffffffff};
} // namespace SPI_SHADER_USER_DATA_PS_16
namespace SPI_SHADER_USER_DATA_PS_17 {
constexpr Register reg{0x19bd, 0};
constexpr Field DATA{0, 0xffffffff};
} // namespace SPI_SHADER_USER_DATA_PS_17
namespace SPI_SHADER_USER_DATA_PS_18 {
constexpr Register reg{0x19be, 0};
constexpr Field DATA{0, 0xffffffff};
} // namespace SPI_SHADER_USER_DATA_PS_18
namespace SPI_SHADER_USER_DATA_PS_19 {
constexpr Register reg{0x19bf, 0};
constexpr Field DATA{0, 0xffffffff};
} // namespace SPI_SHADER_USER_DATA_PS_19
namespace SPI_SHADER_USER_ACCUM_PS_0 {
constexpr Register reg{0x19d2, 0};
constexpr Field CONTRIBUTION{0, 0x0000007f};
} // namespace SPI_SHADER_USER_ACCUM_PS_0
namespace SPI_SHADER_USER_ACCUM_PS_1 {
constexpr Register reg{0x19d3, 0};
constexpr Field CONTRIBUTION{0, 0x0000007f};
} // namespace SPI_SHADER_USER_ACCUM_PS_1
namespace SPI_SHADER_USER_ACCUM_PS_2 {
constexpr Register reg{0x19d4, 0};
constexpr Field CONTRIBUTION{0, 0x0000007f};
} // namespace SPI_SHADER_USER_ACCUM_PS_2
namespace SPI_SHADER_USER_ACCUM_PS_3 {
constexpr Register reg{0x19d5, 0};
constexpr Field CONTRIBUTION{0, 0x0000007f};
} // namespace SPI_SHADER_USER_ACCUM_PS_3
namespace SPI_SHADER_USER_ACCUM_ESGS_0 {
constexpr Register reg{0x1a52, 0};
constexpr Field CONTRIBUTION{0, 0x0000007f};
} // namespace SPI_SHADER_USER_ACCUM_ESGS_0
namespace SPI_SHADER_USER_ACCUM_ESGS_1 {
constexpr Register reg{0x1a53, 0};
constexpr Field CONTRIBUTION{0, 0x0000007f};
} // namespace SPI_SHADER_USER_ACCUM_ESGS_1
namespace SPI_SHADER_USER_ACCUM_ESGS_2 {
constexpr Register reg{0x1a54, 0};
constexpr Field CONTRIBUTION{0, 0x0000007f};
} // namespace SPI_SHADER_USER_ACCUM_ESGS_2
namespace SPI_SHADER_USER_ACCUM_ESGS_3 {
constexpr Register reg{0x1a55, 0};
constexpr Field CONTRIBUTION{0, 0x0000007f};
} // namespace SPI_SHADER_USER_ACCUM_ESGS_3
namespace SPI_SHADER_USER_ACCUM_LSHS_0 {
constexpr Register reg{0x1ad2, 0};
constexpr Field CONTRIBUTION{0, 0x0000007f};
} // namespace SPI_SHADER_USER_ACCUM_LSHS_0
namespace SPI_SHADER_USER_ACCUM_LSHS_1 {
constexpr Register reg{0x1ad3, 0};
constexpr Field CONTRIBUTION{0, 0x0000007f};
} // namespace SPI_SHADER_USER_ACCUM_LSHS_1
namespace SPI_SHADER_USER_ACCUM_LSHS_2 {
constexpr Register reg{0x1ad4, 0};
constexpr Field CONTRIBUTION{0, 0x0000007f};
} // namespace SPI_SHADER_USER_ACCUM_LSHS_2
namespace SPI_SHADER_USER_ACCUM_LSHS_3 {
constexpr Register reg{0x1ad5, 0};
constexpr Field CONTRIBUTION{0, 0x0000007f};
} // namespace SPI_SHADER_USER_ACCUM_LSHS_3
namespace SPI_SHADER_PGM_RSRC3_HS {
constexpr Register reg{0x1aa7, 0};
constexpr Field WAVE_LIMIT{0, 0x0000003f};
constexpr Field LOCK_LOW_THRESHOLD{6, 0x000003c0};
constexpr Field GROUP_FIFO_DEPTH{10, 0x0000fc00};
constexpr Field CU_EN{16, 0xffff0000};
} // namespace SPI_SHADER_PGM_RSRC3_HS
namespace SPI_SHADER_PGM_HI_LS {
constexpr Register reg{0x1ae9, 0};
constexpr Field MEM_BASE{0, 0x000000ff};
} // namespace SPI_SHADER_PGM_HI_LS
namespace VGT_PRIMITIVE_TYPE {
constexpr Register reg{0x2242, 1};
constexpr Field PRIM_TYPE{0, 0x0000003f};
} // namespace VGT_PRIMITIVE_TYPE
namespace GE_CNTL {
constexpr Register reg{0x225b, 1};
constexpr Field PRIMS_PER_SUBGRP{0, 0x000001ff};
constexpr Field VERTS_PER_SUBGRP{9, 0x0003fe00};
constexpr Field BREAK_SUBGRP_AT_EOI{18, 0x00040000};
constexpr Field PACKET_TO_ONE_PA{19, 0x00080000};
constexpr Field BREAK_PRIMGRP_AT_EOI{20, 0x00100000};
constexpr Field PRIM_GRP_SIZE{21, 0x3fe00000};
constexpr Field GCR_DISABLE{30, 0x40000000};
constexpr Field DIS_PG_SIZE_ADJUST_FOR_STRIP{31, 0x80000000};
} // namespace GE_CNTL
namespace GE_MULTI_PRIM_IB_RESET_EN {
constexpr Register reg{0x224b, 1};
constexpr Field RESET_EN{0, 0x00000001};
constexpr Field MATCH_ALL_BITS{1, 0x00000002};
constexpr Field DISABLE_FOR_AUTO_INDEX{2, 0x00000004};
} // namespace GE_MULTI_PRIM_IB_RESET_EN
namespace GE_MIN_VTX_INDX {
constexpr Register reg{0x2249, 1};
constexpr Field MIN_INDX{0, 0xffffffff};
} // namespace GE_MIN_VTX_INDX
namespace GE_MAX_VTX_INDX {
constexpr Register reg{0x2259, 1};
constexpr Field MAX_INDX{0, 0xffffffff};
} // namespace GE_MAX_VTX_INDX
namespace GE_INDX_OFFSET {
constexpr Register reg{0x224a, 1};
constexpr Field INDX_OFFSET{0, 0xffffffff};
} // namespace GE_INDX_OFFSET
namespace VGT_INSTANCE_BASE_ID {
constexpr Register reg{0x225a, 1};
constexpr Field INSTANCE_BASE_ID{0, 0xffffffff};
} // namespace VGT_INSTANCE_BASE_ID
namespace GE_STEREO_CNTL {
constexpr Register reg{0x225f, 1};
constexpr Field RT_SLICE{0, 0x00000007};
constexpr Field VIEWPORT{3, 0x00000078};
constexpr Field EN_STEREO{8, 0x00000100};
} // namespace GE_STEREO_CNTL
namespace GE_USER_VGPR_EN {
constexpr Register reg{0x2262, 1};
constexpr Field EN_USER_VGPR1{0, 0x00000001};
constexpr Field EN_USER_VGPR2{1, 0x00000002};
constexpr Field EN_USER_VGPR3{2, 0x00000004};
} // namespace GE_USER_VGPR_EN
namespace PA_SU_LINE_STIPPLE_VALUE {
constexpr Register reg{0x2280, 1};
constexpr Field LINE_STIPPLE_VALUE{0, 0x00ffffff};
} // namespace PA_SU_LINE_STIPPLE_VALUE
namespace PA_SC_LINE_STIPPLE_STATE {
constexpr Register reg{0x2281, 1};
constexpr Field CURRENT_PTR{0, 0x0000000f};
constexpr Field CURRENT_COUNT{8, 0x0000ff00};
} // namespace PA_SC_LINE_STIPPLE_STATE
namespace GE_PC_ALLOC {
constexpr Register reg{0x2260, 1};
constexpr Field OVERSUB_EN{0, 0x00000001};
constexpr Field NUM_PC_LINES{1, 0x000007fe};
} // namespace GE_PC_ALLOC
namespace SPI_GS_THROTTLE_CNTL1 {
constexpr Register reg{0x2444, 1};
constexpr Field PH_POLL_INTERVAL{0, 0x0000000f};
constexpr Field PH_THROTTLE_BASE{4, 0x000000f0};
constexpr Field PH_THROTTLE_STEP_SIZE{8, 0x00000f00};
constexpr Field SPI_VGPR_THRESHOLD{12, 0x0000f000};
constexpr Field SPI_LDS_THRESHOLD{16, 0x000f0000};
constexpr Field SPI_POLL_INTERVAL{20, 0x00f00000};
constexpr Field SPI_THROTTLE_BASE{24, 0x0f000000};
constexpr Field SPI_THROTTLE_STEP_SIZE{28, 0xf0000000};
} // namespace SPI_GS_THROTTLE_CNTL1
namespace SPI_GS_THROTTLE_CNTL2 {
constexpr Register reg{0x2445, 1};
constexpr Field SPI_THROTTLE_MODE{0, 0x00000003};
constexpr Field GRP_LIFETIME_THRESHOLD{2, 0x0000003c};
constexpr Field GRP_LIFETIME_THRESHOLD_FACTOR{6, 0x000000c0};
constexpr Field GRP_LIFETIME_PENALTY1{8, 0x00000700};
constexpr Field GRP_LIFETIME_PENALTY2{11, 0x00003800};
constexpr Field PS_STALL_THRESHOLD{14, 0x0000c000};
constexpr Field PH_MODE{16, 0x00010000};
constexpr Field RESERVED{17, 0xfffe0000};
} // namespace SPI_GS_THROTTLE_CNTL2
namespace SPI_ATTRIBUTE_RING_BASE {
constexpr Register reg{0x2446, 1};
constexpr Field BASE{0, 0xffffffff};
} // namespace SPI_ATTRIBUTE_RING_BASE
namespace SPI_ATTRIBUTE_RING_SIZE {
constexpr Register reg{0x2447, 1};
constexpr Field MEM_SIZE{0, 0x000000ff};
constexpr Field BIG_PAGE{16, 0x00010000};
constexpr Field L1_POLICY{17, 0x00060000};
constexpr Field L2_POLICY{19, 0x00180000};
constexpr Field LLC_NOALLOC{21, 0x00200000};
constexpr Field GL1_PERF_COUNTER_DISABLE{22, 0x00400000};
} // namespace SPI_ATTRIBUTE_RING_SIZE
namespace VGT_GS_OUT_PRIM_TYPE {
constexpr Register reg{0x2266, 1};
constexpr Field OUTPRIM_TYPE{0, 0x0000003f};
} // namespace VGT_GS_OUT_PRIM_TYPE

// Every register the pipeline may write, in catalog order. The IB builder refuses
// any register outside this list (no user-mode value ever selects a register).
// Added 2026-10-24 (radeongl-complete): MRT1 for GL_FRONT_AND_BACK and the v4
// PS constant-block user data (transcribed from the same pinned headers).
namespace CB_COLOR1_BASE {
constexpr Register reg{0x0327, 1};
constexpr Field BASE_256B{0, 0xffffffff};
} // namespace CB_COLOR1_BASE
namespace CB_COLOR1_BASE_EXT {
constexpr Register reg{0x0391, 1};
constexpr Field BASE_256B{0, 0x000000ff};
} // namespace CB_COLOR1_BASE_EXT
namespace CB_COLOR1_VIEW {
constexpr Register reg{0x032a, 1};
constexpr Field SLICE_START{0, 0x00001fff};
constexpr Field SLICE_MAX{13, 0x03ffe000};
constexpr Field MIP_LEVEL{26, 0x3c000000};
} // namespace CB_COLOR1_VIEW
namespace CB_COLOR1_ATTRIB {
constexpr Register reg{0x032c, 1};
constexpr Field NUM_FRAGMENTS{0, 0x00000003};
constexpr Field FORCE_DST_ALPHA_1{2, 0x00000004};
constexpr Field DISABLE_FMASK_NOALLOC_OPT{3, 0x00000008};
constexpr Field LIMIT_COLOR_FETCH_TO_256B_MAX{4, 0x00000010};
constexpr Field FORCE_LIMIT_COLOR_SECTOR_TO_256B_MAX{5, 0x00000020};
} // namespace CB_COLOR1_ATTRIB
namespace CB_COLOR1_FDCC_CONTROL {
constexpr Register reg{0x032d, 1};
constexpr Field SAMPLE_MASK_TRACKER_DISABLE{0, 0x00000001};
constexpr Field SAMPLE_MASK_TRACKER_FEA_FORCE{1, 0x00000002};
constexpr Field MAX_UNCOMPRESSED_BLOCK_SIZE{2, 0x0000000c};
constexpr Field MIN_COMPRESSED_BLOCK_SIZE{4, 0x00000010};
constexpr Field MAX_COMPRESSED_BLOCK_SIZE{5, 0x00000060};
constexpr Field COLOR_TRANSFORM{7, 0x00000180};
constexpr Field INDEPENDENT_64B_BLOCKS{9, 0x00000200};
constexpr Field INDEPENDENT_128B_BLOCKS{10, 0x00000400};
constexpr Field DISABLE_CONSTANT_ENCODE_REG{18, 0x00040000};
constexpr Field ENABLE_CONSTANT_ENCODE_REG_WRITE{19, 0x00080000};
constexpr Field SKIP_LOW_COMP_RATIO{21, 0x00200000};
constexpr Field FDCC_ENABLE{22, 0x00400000};
constexpr Field DCC_COMPRESS_DISABLE{23, 0x00800000};
constexpr Field FRAGMENT_COMPRESS_DISABLE{24, 0x01000000};
} // namespace CB_COLOR1_FDCC_CONTROL
namespace CB_COLOR1_DCC_BASE {
constexpr Register reg{0x0334, 1};
constexpr Field BASE_256B{0, 0xffffffff};
} // namespace CB_COLOR1_DCC_BASE
namespace CB_COLOR1_DCC_BASE_EXT {
constexpr Register reg{0x03a9, 1};
constexpr Field BASE_256B{0, 0x000000ff};
} // namespace CB_COLOR1_DCC_BASE_EXT
namespace CB_COLOR1_ATTRIB2 {
constexpr Register reg{0x03b1, 1};
constexpr Field MIP0_HEIGHT{0, 0x00003fff};
constexpr Field MIP0_WIDTH{14, 0x0fffc000};
constexpr Field MAX_MIP{28, 0xf0000000};
} // namespace CB_COLOR1_ATTRIB2
namespace CB_COLOR1_ATTRIB3 {
constexpr Register reg{0x03b9, 1};
constexpr Field MIP0_DEPTH{0, 0x00001fff};
constexpr Field META_LINEAR{13, 0x00002000};
constexpr Field COLOR_SW_MODE{14, 0x0007c000};
constexpr Field RESOURCE_TYPE{24, 0x03000000};
constexpr Field DCC_PIPE_ALIGNED{30, 0x40000000};
} // namespace CB_COLOR1_ATTRIB3
namespace CB_BLEND1_CONTROL {
constexpr Register reg{0x01e1, 1};
constexpr Field COLOR_SRCBLEND{0, 0x0000001f};
constexpr Field COLOR_COMB_FCN{5, 0x000000e0};
constexpr Field COLOR_DESTBLEND{8, 0x00001f00};
constexpr Field ALPHA_SRCBLEND{16, 0x001f0000};
constexpr Field ALPHA_COMB_FCN{21, 0x00e00000};
constexpr Field ALPHA_DESTBLEND{24, 0x1f000000};
constexpr Field SEPARATE_ALPHA_BLEND{29, 0x20000000};
constexpr Field ENABLE{30, 0x40000000};
constexpr Field DISABLE_ROP3{31, 0x80000000};
} // namespace CB_BLEND1_CONTROL
namespace SX_MRT1_BLEND_OPT {
constexpr Register reg{0x01d9, 1};
constexpr Field COLOR_SRC_OPT{0, 0x00000007};
constexpr Field COLOR_DST_OPT{4, 0x00000070};
constexpr Field COLOR_COMB_FCN{8, 0x00000700};
constexpr Field ALPHA_SRC_OPT{16, 0x00070000};
constexpr Field ALPHA_DST_OPT{20, 0x00700000};
constexpr Field ALPHA_COMB_FCN{24, 0x07000000};
} // namespace SX_MRT1_BLEND_OPT
namespace SPI_SHADER_USER_DATA_PS_20 {
constexpr Register reg{0x19c0, 0};
constexpr Field DATA{0, 0xffffffff};
} // namespace SPI_SHADER_USER_DATA_PS_20
namespace SPI_SHADER_USER_DATA_PS_21 {
constexpr Register reg{0x19c1, 0};
constexpr Field DATA{0, 0xffffffff};
} // namespace SPI_SHADER_USER_DATA_PS_21
namespace SPI_SHADER_USER_DATA_PS_22 {
constexpr Register reg{0x19c2, 0};
constexpr Field DATA{0, 0xffffffff};
} // namespace SPI_SHADER_USER_DATA_PS_22
namespace SPI_SHADER_USER_DATA_PS_23 {
constexpr Register reg{0x19c3, 0};
constexpr Field DATA{0, 0xffffffff};
} // namespace SPI_SHADER_USER_DATA_PS_23
constexpr Register allowed_registers[] = {
    DB_RENDER_CONTROL::reg,
    DB_COUNT_CONTROL::reg,
    DB_RENDER_OVERRIDE::reg,
    DB_RENDER_OVERRIDE2::reg,
    DB_DEPTH_VIEW::reg,
    DB_HTILE_DATA_BASE::reg,
    DB_HTILE_DATA_BASE_HI::reg,
    DB_HTILE_SURFACE::reg,
    DB_DEPTH_SIZE_XY::reg,
    DB_DEPTH_BOUNDS_MIN::reg,
    DB_DEPTH_BOUNDS_MAX::reg,
    DB_STENCIL_CLEAR::reg,
    DB_DEPTH_CLEAR::reg,
    DB_Z_INFO::reg,
    DB_STENCIL_INFO::reg,
    DB_Z_READ_BASE::reg,
    DB_STENCIL_READ_BASE::reg,
    DB_Z_WRITE_BASE::reg,
    DB_STENCIL_WRITE_BASE::reg,
    DB_Z_READ_BASE_HI::reg,
    DB_STENCIL_READ_BASE_HI::reg,
    DB_Z_WRITE_BASE_HI::reg,
    DB_STENCIL_WRITE_BASE_HI::reg,
    DB_DEPTH_CONTROL::reg,
    DB_STENCIL_CONTROL::reg,
    DB_STENCILREFMASK::reg,
    DB_STENCILREFMASK_BF::reg,
    DB_SHADER_CONTROL::reg,
    DB_RMI_L2_CACHE_CONTROL::reg,
    DB_ALPHA_TO_MASK::reg,
    DB_EQAA::reg,
    TA_BC_BASE_ADDR::reg,
    TA_BC_BASE_ADDR_HI::reg,
    PA_SC_WINDOW_OFFSET::reg,
    PA_SC_WINDOW_SCISSOR_TL::reg,
    PA_SC_WINDOW_SCISSOR_BR::reg,
    PA_SC_SCREEN_SCISSOR_TL::reg,
    PA_SC_SCREEN_SCISSOR_BR::reg,
    PA_SC_GENERIC_SCISSOR_TL::reg,
    PA_SC_GENERIC_SCISSOR_BR::reg,
    PA_SC_VPORT_SCISSOR_0_TL::reg,
    PA_SC_VPORT_SCISSOR_0_BR::reg,
    PA_SC_VPORT_ZMIN_0::reg,
    PA_SC_VPORT_ZMAX_0::reg,
    PA_SC_CLIPRECT_RULE::reg,
    PA_SC_EDGERULE::reg,
    PA_SU_HARDWARE_SCREEN_OFFSET::reg,
    PA_SC_VRS_OVERRIDE_CNTL::reg,
    PA_SC_MODE_CNTL_0::reg,
    PA_SC_MODE_CNTL_1::reg,
    PA_SC_AA_CONFIG::reg,
    PA_SC_AA_MASK_X0Y0_X1Y0::reg,
    PA_SC_AA_MASK_X0Y1_X1Y1::reg,
    PA_SC_LINE_CNTL::reg,
    PA_SC_LINE_STIPPLE::reg,
    PA_SC_BINNER_CNTL_0::reg,
    PA_SC_BINNER_CNTL_1::reg,
    PA_SC_SHADER_CONTROL::reg,
    PA_SC_NGG_MODE_CNTL::reg,
    PA_CL_VPORT_XSCALE::reg,
    PA_CL_VPORT_XOFFSET::reg,
    PA_CL_VPORT_YSCALE::reg,
    PA_CL_VPORT_YOFFSET::reg,
    PA_CL_VPORT_ZSCALE::reg,
    PA_CL_VPORT_ZOFFSET::reg,
    PA_CL_VTE_CNTL::reg,
    PA_CL_CLIP_CNTL::reg,
    PA_CL_GB_VERT_CLIP_ADJ::reg,
    PA_CL_GB_VERT_DISC_ADJ::reg,
    PA_CL_GB_HORZ_CLIP_ADJ::reg,
    PA_CL_GB_HORZ_DISC_ADJ::reg,
    PA_CL_VS_OUT_CNTL::reg,
    PA_CL_NGG_CNTL::reg,
    PA_CL_VRS_CNTL::reg,
    PA_SU_VTX_CNTL::reg,
    PA_SU_POINT_SIZE::reg,
    PA_SU_POINT_MINMAX::reg,
    PA_SU_LINE_CNTL::reg,
    PA_SU_SC_MODE_CNTL::reg,
    PA_SU_POLY_OFFSET_DB_FMT_CNTL::reg,
    PA_SU_POLY_OFFSET_CLAMP::reg,
    PA_SU_POLY_OFFSET_FRONT_SCALE::reg,
    PA_SU_POLY_OFFSET_FRONT_OFFSET::reg,
    PA_SU_POLY_OFFSET_BACK_SCALE::reg,
    PA_SU_POLY_OFFSET_BACK_OFFSET::reg,
    PA_SU_PRIM_FILTER_CNTL::reg,
    PA_SU_SMALL_PRIM_FILTER_CNTL::reg,
    PA_SU_LINE_STIPPLE_CNTL::reg,
    CB_COLOR0_BASE::reg,
    CB_COLOR0_BASE_EXT::reg,
    CB_COLOR0_VIEW::reg,
    CB_COLOR0_INFO::reg,
    CB_COLOR0_ATTRIB::reg,
    CB_COLOR0_FDCC_CONTROL::reg,
    CB_COLOR0_DCC_BASE::reg,
    CB_COLOR0_DCC_BASE_EXT::reg,
    CB_COLOR0_ATTRIB2::reg,
    CB_COLOR0_ATTRIB3::reg,
    CB_COLOR1_INFO::reg,
    CB_COLOR2_INFO::reg,
    CB_COLOR3_INFO::reg,
    CB_COLOR4_INFO::reg,
    CB_COLOR5_INFO::reg,
    CB_COLOR6_INFO::reg,
    CB_COLOR7_INFO::reg,
    CB_TARGET_MASK::reg,
    CB_SHADER_MASK::reg,
    CB_COLOR_CONTROL::reg,
    CB_BLEND0_CONTROL::reg,
    CB_BLEND_RED::reg,
    CB_BLEND_GREEN::reg,
    CB_BLEND_BLUE::reg,
    CB_BLEND_ALPHA::reg,
    CB_RMI_GL2_CACHE_CONTROL::reg,
    CB_FDCC_CONTROL::reg,
    SX_PS_DOWNCONVERT::reg,
    SX_BLEND_OPT_EPSILON::reg,
    SX_BLEND_OPT_CONTROL::reg,
    SX_MRT0_BLEND_OPT::reg,
    SX_PS_DOWNCONVERT_CONTROL::reg,
    SPI_SHADER_POS_FORMAT::reg,
    SPI_SHADER_IDX_FORMAT::reg,
    SPI_SHADER_Z_FORMAT::reg,
    SPI_SHADER_COL_FORMAT::reg,
    SPI_PS_INPUT_ENA::reg,
    SPI_PS_INPUT_ADDR::reg,
    SPI_PS_IN_CONTROL::reg,
    SPI_BARYC_CNTL::reg,
    SPI_PS_INPUT_CNTL_0::reg,
    SPI_PS_INPUT_CNTL_1::reg,
    SPI_PS_INPUT_CNTL_2::reg,
    SPI_PS_INPUT_CNTL_3::reg,
    SPI_VS_OUT_CONFIG::reg,
    SPI_INTERP_CONTROL_0::reg,
    SPI_TMPRING_SIZE::reg,
    VGT_SHADER_STAGES_EN::reg,
    VGT_GS_MAX_VERT_OUT::reg,
    VGT_ESGS_RING_ITEMSIZE::reg,
    VGT_REUSE_OFF::reg,
    VGT_PRIMITIVEID_EN::reg,
    VGT_DRAW_PAYLOAD_CNTL::reg,
    VGT_GS_INSTANCE_CNT::reg,
    VGT_MULTI_PRIM_IB_RESET_INDX::reg,
    VGT_HOS_MAX_TESS_LEVEL::reg,
    VGT_TESS_DISTRIBUTION::reg,
    GE_NGG_SUBGRP_CNTL::reg,
    GE_MAX_OUTPUT_PER_SUBGROUP::reg,
    SPI_SHADER_PGM_LO_ES::reg,
    SPI_SHADER_PGM_HI_ES::reg,
    SPI_SHADER_PGM_RSRC1_GS::reg,
    SPI_SHADER_PGM_RSRC2_GS::reg,
    SPI_SHADER_PGM_RSRC3_GS::reg,
    SPI_SHADER_PGM_RSRC4_GS::reg,
    SPI_SHADER_USER_DATA_GS_0::reg,
    SPI_SHADER_USER_DATA_GS_1::reg,
    SPI_SHADER_USER_DATA_GS_2::reg,
    SPI_SHADER_USER_DATA_GS_3::reg,
    SPI_SHADER_USER_DATA_GS_4::reg,
    SPI_SHADER_USER_DATA_GS_5::reg,
    SPI_SHADER_USER_DATA_GS_6::reg,
    SPI_SHADER_USER_DATA_GS_7::reg,
    SPI_SHADER_USER_DATA_GS_8::reg,
    SPI_SHADER_USER_DATA_GS_9::reg,
    SPI_SHADER_USER_DATA_GS_10::reg,
    SPI_SHADER_USER_DATA_GS_11::reg,
    SPI_SHADER_USER_DATA_GS_12::reg,
    SPI_SHADER_PGM_LO_PS::reg,
    SPI_SHADER_PGM_HI_PS::reg,
    SPI_SHADER_PGM_RSRC1_PS::reg,
    SPI_SHADER_PGM_RSRC2_PS::reg,
    SPI_SHADER_PGM_RSRC3_PS::reg,
    SPI_SHADER_PGM_RSRC4_PS::reg,
    SPI_SHADER_REQ_CTRL_PS::reg,
    SPI_SHADER_USER_DATA_PS_0::reg,
    SPI_SHADER_USER_DATA_PS_1::reg,
    SPI_SHADER_USER_DATA_PS_2::reg,
    SPI_SHADER_USER_DATA_PS_3::reg,
    SPI_SHADER_USER_DATA_PS_4::reg,
    SPI_SHADER_USER_DATA_PS_5::reg,
    SPI_SHADER_USER_DATA_PS_6::reg,
    SPI_SHADER_USER_DATA_PS_7::reg,
    SPI_SHADER_USER_DATA_PS_8::reg,
    SPI_SHADER_USER_DATA_PS_9::reg,
    SPI_SHADER_USER_DATA_PS_10::reg,
    SPI_SHADER_USER_DATA_PS_11::reg,
    SPI_SHADER_USER_DATA_PS_12::reg,
    SPI_SHADER_USER_DATA_PS_13::reg,
    SPI_SHADER_USER_DATA_PS_14::reg,
    SPI_SHADER_USER_DATA_PS_15::reg,
    SPI_SHADER_USER_DATA_PS_16::reg,
    SPI_SHADER_USER_DATA_PS_17::reg,
    SPI_SHADER_USER_DATA_PS_18::reg,
    SPI_SHADER_USER_DATA_PS_19::reg,
    SPI_SHADER_USER_ACCUM_PS_0::reg,
    SPI_SHADER_USER_ACCUM_PS_1::reg,
    SPI_SHADER_USER_ACCUM_PS_2::reg,
    SPI_SHADER_USER_ACCUM_PS_3::reg,
    SPI_SHADER_USER_ACCUM_ESGS_0::reg,
    SPI_SHADER_USER_ACCUM_ESGS_1::reg,
    SPI_SHADER_USER_ACCUM_ESGS_2::reg,
    SPI_SHADER_USER_ACCUM_ESGS_3::reg,
    SPI_SHADER_USER_ACCUM_LSHS_0::reg,
    SPI_SHADER_USER_ACCUM_LSHS_1::reg,
    SPI_SHADER_USER_ACCUM_LSHS_2::reg,
    SPI_SHADER_USER_ACCUM_LSHS_3::reg,
    SPI_SHADER_PGM_RSRC3_HS::reg,
    SPI_SHADER_PGM_HI_LS::reg,
    VGT_PRIMITIVE_TYPE::reg,
    GE_CNTL::reg,
    GE_MULTI_PRIM_IB_RESET_EN::reg,
    GE_MIN_VTX_INDX::reg,
    GE_MAX_VTX_INDX::reg,
    GE_INDX_OFFSET::reg,
    VGT_INSTANCE_BASE_ID::reg,
    GE_STEREO_CNTL::reg,
    GE_USER_VGPR_EN::reg,
    PA_SU_LINE_STIPPLE_VALUE::reg,
    PA_SC_LINE_STIPPLE_STATE::reg,
    GE_PC_ALLOC::reg,
    SPI_GS_THROTTLE_CNTL1::reg,
    SPI_GS_THROTTLE_CNTL2::reg,
    SPI_ATTRIBUTE_RING_BASE::reg,
    SPI_ATTRIBUTE_RING_SIZE::reg,
    VGT_GS_OUT_PRIM_TYPE::reg,
    CB_COLOR1_BASE::reg,
    CB_COLOR1_BASE_EXT::reg,
    CB_COLOR1_VIEW::reg,
    CB_COLOR1_ATTRIB::reg,
    CB_COLOR1_FDCC_CONTROL::reg,
    CB_COLOR1_DCC_BASE::reg,
    CB_COLOR1_DCC_BASE_EXT::reg,
    CB_COLOR1_ATTRIB2::reg,
    CB_COLOR1_ATTRIB3::reg,
    CB_BLEND1_CONTROL::reg,
    SX_MRT1_BLEND_OPT::reg,
    SPI_SHADER_USER_DATA_PS_20::reg,
    SPI_SHADER_USER_DATA_PS_21::reg,
    SPI_SHADER_USER_DATA_PS_22::reg,
    SPI_SHADER_USER_DATA_PS_23::reg,
};
constexpr uint32_t allowed_register_count = sizeof(allowed_registers) / sizeof(allowed_registers[0]);

constexpr bool allowed_register(Register r) noexcept {
    for (uint32_t index = 0; index < allowed_register_count; ++index) {
        if (allowed_registers[index].offset == r.offset &&
            allowed_registers[index].base_index == r.base_index) {
            return true;
        }
    }
    return false;
}

// Register arrays addressed by index (consecutive dwords in the header).
constexpr Register user_data_gs(uint32_t index) noexcept {
    return {SPI_SHADER_USER_DATA_GS_0::reg.offset + index, SPI_SHADER_USER_DATA_GS_0::reg.base_index};
}
constexpr Register user_data_ps(uint32_t index) noexcept {
    return {SPI_SHADER_USER_DATA_PS_0::reg.offset + index, SPI_SHADER_USER_DATA_PS_0::reg.base_index};
}
constexpr Register ps_input_cntl(uint32_t index) noexcept {
    return {SPI_PS_INPUT_CNTL_0::reg.offset + index, SPI_PS_INPUT_CNTL_0::reg.base_index};
}
static_assert(user_data_gs(12).offset == SPI_SHADER_USER_DATA_GS_12::reg.offset, "GS user data array");
static_assert(user_data_ps(19).offset == SPI_SHADER_USER_DATA_PS_19::reg.offset &&
                  user_data_ps(23).offset == SPI_SHADER_USER_DATA_PS_23::reg.offset, "PS user data array");
static_assert(ps_input_cntl(3).offset == SPI_PS_INPUT_CNTL_3::reg.offset, "PS input array");
static_assert(space(DB_RENDER_CONTROL::reg) == Space::context, "context space");
static_assert(space(SPI_SHADER_PGM_LO_ES::reg) == Space::sh, "sh space");
static_assert(space(SPI_ATTRIBUTE_RING_SIZE::reg) == Space::uconfig, "uconfig space");

} // namespace rx7900::opengl::gfx11_reg

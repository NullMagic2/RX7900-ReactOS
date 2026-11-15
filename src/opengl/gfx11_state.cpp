/*
Module: rx7900.opengl.gfx11_state
Purpose: Bounded GFX11 PM4 emission of the OpenGL 1.1 pipeline state.
Created: 2026-10-23
Architecture: See gfx11_state.hpp. Each emitter maps one Mesa 24.2.0 radeonsi
GFX11 state atom onto the gfx11_registers catalog. Floating-point arithmetic is
used for viewport, guard band and polygon-offset values; native kernel callers
hold a saved floating-point state (KeSaveFloatingPointState) around the build.
Copyright 2012 Advanced Micro Devices, Inc. and Mesa contributors (radeonsi,
MIT); the preserved permission notice is in licenses/OPENGL.txt.
*/
#include "gfx11_state.hpp"

#include <string.h>

namespace rx7900::opengl::gfx11 {
namespace {

namespace r = gfx11_reg;
namespace abi = gfx11_abi;

// Kernel stack: each state atom keeps its own bounded frame instead of one large
// inlined emit_draw frame on the IOCTL path.
#if defined(_MSC_VER)
#define RX7900_GFX11_NOINLINE __declspec(noinline)
#else
#define RX7900_GFX11_NOINLINE __attribute__((noinline))
#endif

uint32_t space_base(r::Space space) noexcept {
    return space == r::Space::context ? r::context_first :
           space == r::Space::sh ? r::sh_first : r::uconfig_first;
}

uint32_t set_opcode(r::Space space) noexcept {
    return space == r::Space::context ? op::set_context_reg :
           space == r::Space::sh ? op::set_sh_reg : op::set_uconfig_reg;
}

bool emit(Pm4Builder& builder, r::Register reg, const RegisterValue& value) noexcept {
    if (!value.valid()) {
        builder.fail(StateStatus::field_overflow);
        return false;
    }
    return builder.set(reg, value.word());
}

RegisterValue v() noexcept { return RegisterValue(); }

uint32_t low(uint64_t value) noexcept { return static_cast<uint32_t>(value); }
uint32_t high(uint64_t value) noexcept { return static_cast<uint32_t>(value >> 32); }

float minimum(float a, float b) noexcept { return a < b ? a : b; }
int32_t clamp_int(int64_t value, int32_t low_bound, int32_t high_bound) noexcept {
    return value < low_bound ? low_bound : value > high_bound ? high_bound :
           static_cast<int32_t>(value);
}

uint32_t field_max(r::Field field) noexcept { return field.mask >> field.shift; }

uint32_t compare_code(Compare compare) noexcept {
    return static_cast<uint32_t>(compare);    // GL order == CompareFrag
}

uint32_t stencil_code(StencilOp op_value) noexcept {
    switch (op_value) {
    case StencilOp::keep: return hw::stencil_keep;
    case StencilOp::zero: return hw::stencil_zero;
    case StencilOp::replace: return hw::stencil_replace_test;
    case StencilOp::increment: return hw::stencil_add_clamp;
    case StencilOp::decrement: return hw::stencil_sub_clamp;
    case StencilOp::invert: return hw::stencil_invert;
    }
    return hw::stencil_keep;
}

uint32_t blend_code(BlendFactor factor) noexcept {
    switch (factor) {
    case BlendFactor::zero: return hw::blend_zero;
    case BlendFactor::one: return hw::blend_one;
    case BlendFactor::src_color: return hw::blend_src_color;
    case BlendFactor::one_minus_src_color: return hw::blend_one_minus_src_color;
    case BlendFactor::dst_color: return hw::blend_dst_color;
    case BlendFactor::one_minus_dst_color: return hw::blend_one_minus_dst_color;
    case BlendFactor::src_alpha: return hw::blend_src_alpha;
    case BlendFactor::one_minus_src_alpha: return hw::blend_one_minus_src_alpha;
    case BlendFactor::dst_alpha: return hw::blend_dst_alpha;
    case BlendFactor::one_minus_dst_alpha: return hw::blend_one_minus_dst_alpha;
    case BlendFactor::src_alpha_saturate: return hw::blend_src_alpha_saturate;
    }
    return hw::blend_one;
}

bool aligned(uint64_t value, uint64_t alignment) noexcept {
    return (value & (alignment - 1u)) == 0;
}

bool valid_framebuffer(const Framebuffer& fb) noexcept {
    if (fb.width == 0 || fb.height == 0 || fb.width > max_dimension ||
        fb.height > max_dimension || fb.color.va == 0 ||
        !aligned(fb.color.va, surface_base_alignment) ||
        fb.color.pitch_pixels != linear_pitch_pixels(fb.width, fb.color.kind)) {
        return false;
    }
    if (fb.depth.present) {
        DepthLayout layout{};
        if (!depth_layout(fb.width, fb.height, layout) || fb.depth.z_va == 0 ||
            !aligned(fb.depth.z_va, depth_block_bytes) ||
            fb.depth.stencil_va != fb.depth.z_va + layout.stencil_offset) {
            return false;
        }
    }
    if (fb.dual && (fb.color.kind != TargetKind::bgra8 || fb.color1.kind != TargetKind::bgra8 ||
                    fb.color1.va == 0 || fb.color1.va == fb.color.va ||
                    !aligned(fb.color1.va, surface_base_alignment) ||
                    fb.color1.pitch_pixels != fb.color.pitch_pixels)) {
        return false;
    }
    return true;
}

// ---- Viewport, guard band and scissors (si_state_viewport.c) ----

struct HardwareRect final {
    int32_t minx, miny, maxx, maxy;   // top-down rows, max exclusive
};

HardwareRect viewport_rect(const Framebuffer& fb, const RasterState& s) noexcept {
    const int64_t height = fb.height;
    return {clamp_int(s.viewport_x, -32768, 32767),
            clamp_int(height - s.viewport_y - static_cast<int64_t>(s.viewport_height), -32768, 32767),
            clamp_int(static_cast<int64_t>(s.viewport_x) + s.viewport_width, -32768, 32767),
            clamp_int(height - s.viewport_y, -32768, 32767)};
}

uint32_t quant_mode_index(const HardwareRect& rect) noexcept {
    int32_t corner = 0;
    const int32_t values[4] = {rect.minx, rect.miny, rect.maxx, rect.maxy};
    for (int32_t value : values) {
        const int32_t magnitude = value < 0 ? -value : value;
        corner = magnitude > corner ? magnitude : corner;
    }
    // SI_QUANT_MODE_* index: 0 = 16.8, 1 = 14.10, 2 = 12.12.
    return corner <= 1024 ? 2u : corner <= 4096 ? 1u : 0u;
}

RX7900_GFX11_NOINLINE bool emit_viewport(Pm4Builder& b, const Framebuffer& fb, const RasterState& s,
                   Primitive primitive) noexcept {
    const float half_width = static_cast<float>(s.viewport_width) * 0.5f;
    const float half_height = static_cast<float>(s.viewport_height) * 0.5f;
    const float x_offset = static_cast<float>(s.viewport_x) + half_width;
    // GL window y grows up from the bottom; hardware rows grow down.
    const float y_offset = static_cast<float>(fb.height) - static_cast<float>(s.viewport_y) -
                           half_height;
    const float z_scale = (s.depth_far - s.depth_near) * 0.5f;
    const float z_offset = (s.depth_far + s.depth_near) * 0.5f;
    bool ok = b.set(r::PA_CL_VPORT_XSCALE::reg, float_bits(half_width)) &&
              b.set(r::PA_CL_VPORT_XOFFSET::reg, float_bits(x_offset)) &&
              b.set(r::PA_CL_VPORT_YSCALE::reg, float_bits(-half_height)) &&
              b.set(r::PA_CL_VPORT_YOFFSET::reg, float_bits(y_offset)) &&
              b.set(r::PA_CL_VPORT_ZSCALE::reg, float_bits(z_scale)) &&
              b.set(r::PA_CL_VPORT_ZOFFSET::reg, float_bits(z_offset));
    const float z_min = minimum(s.depth_near, s.depth_far);
    const float z_max = s.depth_near > s.depth_far ? s.depth_near : s.depth_far;
    ok = ok && b.set(r::PA_SC_VPORT_ZMIN_0::reg, float_bits(z_min)) &&
         b.set(r::PA_SC_VPORT_ZMAX_0::reg, float_bits(z_max));

    // si_emit_guardband: centre the viewport with the hardware screen offset.
    HardwareRect rect = viewport_rect(fb, s);
    const uint32_t quant = quant_mode_index(rect);
    const int32_t max_viewport_size[3] = {65536, 16384, 4096};
    const int32_t offset_alignment = 32;            // GFX11
    const int32_t max_offset = 8176;
    int32_t screen_x = clamp_int((static_cast<int64_t>(rect.maxx) + rect.minx) / 2, 0, max_offset);
    int32_t screen_y = clamp_int((static_cast<int64_t>(rect.maxy) + rect.miny) / 2, 0, max_offset);
    screen_x &= ~(offset_alignment - 1);
    screen_y &= ~(offset_alignment - 1);
    rect.minx -= screen_x;
    rect.maxx -= screen_x;
    rect.miny -= screen_y;
    rect.maxy -= screen_y;
    const float translate_x = (static_cast<float>(rect.minx) + static_cast<float>(rect.maxx)) * 0.5f;
    const float translate_y = (static_cast<float>(rect.miny) + static_cast<float>(rect.maxy)) * 0.5f;
    float scale_x = static_cast<float>(rect.maxx) - translate_x;
    float scale_y = static_cast<float>(rect.maxy) - translate_y;
    if (rect.minx == rect.maxx) {
        scale_x = 0.5f;
    }
    if (rect.miny == rect.maxy) {
        scale_y = 0.5f;
    }
    const float max_range = static_cast<float>(max_viewport_size[quant] / 2);
    const float left = (-max_range - 1 - translate_x) / scale_x;
    const float right = (max_range - translate_x) / scale_x;
    const float top = (-max_range - 1 - translate_y) / scale_y;
    const float bottom = (max_range - translate_y) / scale_y;
    const float guard_x = minimum(-left, right);
    const float guard_y = minimum(-top, bottom);
    // si_update_..._clip_discard_distance: half the point size / line width.
    const float distance = primitive == Primitive::points ? s.point_size :
                           primitive == Primitive::lines ? s.line_width : 0.0f;
    const float discard_x = minimum(1.0f + distance / (2.0f * scale_x), guard_x);
    const float discard_y = minimum(1.0f + distance / (2.0f * scale_y), guard_y);
    ok = ok && emit(b, r::PA_SU_VTX_CNTL::reg,
                    v().with(r::PA_SU_VTX_CNTL::PIX_CENTER, 1)
                       .with(r::PA_SU_VTX_CNTL::ROUND_MODE, hw::x_round_to_even)
                       .with(r::PA_SU_VTX_CNTL::QUANT_MODE, hw::quant_16_8_1_256th + quant)) &&
         b.set(r::PA_CL_GB_VERT_CLIP_ADJ::reg, float_bits(guard_y)) &&
         b.set(r::PA_CL_GB_VERT_DISC_ADJ::reg, float_bits(discard_y)) &&
         b.set(r::PA_CL_GB_HORZ_CLIP_ADJ::reg, float_bits(guard_x)) &&
         b.set(r::PA_CL_GB_HORZ_DISC_ADJ::reg, float_bits(discard_x)) &&
         emit(b, r::PA_SU_HARDWARE_SCREEN_OFFSET::reg,
              v().with(r::PA_SU_HARDWARE_SCREEN_OFFSET::HW_SCREEN_OFFSET_X,
                       static_cast<uint32_t>(screen_x) >> 4)
                 .with(r::PA_SU_HARDWARE_SCREEN_OFFSET::HW_SCREEN_OFFSET_Y,
                       static_cast<uint32_t>(screen_y) >> 4));

    // si_emit_one_scissor: the viewport clamped to the framebuffer, then the
    // GL scissor (converted to rows). Bottom-right is exclusive before GFX12.
    HardwareRect clip = viewport_rect(fb, s);
    const int32_t fb_width = static_cast<int32_t>(fb.width);
    const int32_t fb_height = static_cast<int32_t>(fb.height);
    clip.minx = clip.minx < 0 ? 0 : clip.minx;
    clip.miny = clip.miny < 0 ? 0 : clip.miny;
    clip.maxx = clip.maxx > fb_width ? fb_width : clip.maxx;
    clip.maxy = clip.maxy > fb_height ? fb_height : clip.maxy;
    if (s.scissor) {
        const int64_t sx0 = s.scissor_x;
        const int64_t sx1 = sx0 + s.scissor_width;
        const int64_t sy0 = static_cast<int64_t>(fb.height) - s.scissor_y -
                            static_cast<int64_t>(s.scissor_height);
        const int64_t sy1 = static_cast<int64_t>(fb.height) - s.scissor_y;
        clip.minx = clamp_int(sx0 > clip.minx ? sx0 : clip.minx, 0, fb_width);
        clip.miny = clamp_int(sy0 > clip.miny ? sy0 : clip.miny, 0, fb_height);
        clip.maxx = clamp_int(sx1 < clip.maxx ? sx1 : clip.maxx, 0, fb_width);
        clip.maxy = clamp_int(sy1 < clip.maxy ? sy1 : clip.maxy, 0, fb_height);
    }
    if (clip.maxx < clip.minx) {
        clip.maxx = clip.minx;
    }
    if (clip.maxy < clip.miny) {
        clip.maxy = clip.miny;
    }
    ok = ok && emit(b, r::PA_SC_VPORT_SCISSOR_0_TL::reg,
                    v().with(r::PA_SC_VPORT_SCISSOR_0_TL::TL_X, static_cast<uint32_t>(clip.minx))
                       .with(r::PA_SC_VPORT_SCISSOR_0_TL::TL_Y, static_cast<uint32_t>(clip.miny))
                       .with(r::PA_SC_VPORT_SCISSOR_0_TL::WINDOW_OFFSET_DISABLE, 1)) &&
         emit(b, r::PA_SC_VPORT_SCISSOR_0_BR::reg,
              v().with(r::PA_SC_VPORT_SCISSOR_0_BR::BR_X, static_cast<uint32_t>(clip.maxx))
                 .with(r::PA_SC_VPORT_SCISSOR_0_BR::BR_Y, static_cast<uint32_t>(clip.maxy)));
    return ok;
}

// ---- Rasterizer (si_create_rs_state, si_emit_msaa_config, dpbb disable) ----

RX7900_GFX11_NOINLINE bool emit_rasterizer(Pm4Builder& b, const RasterState& s, Primitive primitive) noexcept {
    const bool triangles = primitive == Primitive::triangles;
    const bool cull_front = triangles && (s.cull == CullMode::front || s.cull == CullMode::front_and_back);
    const bool cull_back = triangles && (s.cull == CullMode::back || s.cull == CullMode::front_and_back);
    namespace m = r::PA_SU_SC_MODE_CNTL;
    // The colour buffer rows run top to bottom, so the viewport inverts y and
    // GL's counter-clockwise front face is clockwise in hardware screen space
    // (st/mesa Y_0_TOP front_ccw flip, then radeonsi FACE = !front_ccw).
    const uint32_t polymode_triangles = 2;   // X_DRAW_TRIANGLES
    bool ok = emit(b, m::reg, v().with(m::CULL_FRONT, cull_front ? 1 : 0)
                                  .with(m::CULL_BACK, cull_back ? 1 : 0)
                                  .with(m::FACE, s.front_ccw ? 1 : 0)
                                  .with(m::POLY_MODE, 0)
                                  .with(m::POLYMODE_FRONT_PTYPE, polymode_triangles)
                                  .with(m::POLYMODE_BACK_PTYPE, polymode_triangles)
                                  .with(m::POLY_OFFSET_FRONT_ENABLE, triangles && s.offset ? 1 : 0)
                                  .with(m::POLY_OFFSET_BACK_ENABLE, triangles && s.offset ? 1 : 0)
                                  .with(m::POLY_OFFSET_PARA_ENABLE, !triangles && s.offset ? 1 : 0)
                                  .with(m::PROVOKING_VTX_LAST, 1));
    if (s.offset) {
        // Z32_FLOAT depth: NEG_NUM_DB_BITS = -23 (8-bit two's complement), float
        // format, units unscaled, slope factor * 16.
        ok = ok && emit(b, r::PA_SU_POLY_OFFSET_DB_FMT_CNTL::reg,
                        v().with(r::PA_SU_POLY_OFFSET_DB_FMT_CNTL::POLY_OFFSET_NEG_NUM_DB_BITS, 0xe9u)
                           .with(r::PA_SU_POLY_OFFSET_DB_FMT_CNTL::POLY_OFFSET_DB_IS_FLOAT_FMT, 1)) &&
             b.set(r::PA_SU_POLY_OFFSET_CLAMP::reg, 0) &&
             b.set(r::PA_SU_POLY_OFFSET_FRONT_SCALE::reg, float_bits(s.offset_factor * 16.0f)) &&
             b.set(r::PA_SU_POLY_OFFSET_FRONT_OFFSET::reg, float_bits(s.offset_units)) &&
             b.set(r::PA_SU_POLY_OFFSET_BACK_SCALE::reg, float_bits(s.offset_factor * 16.0f)) &&
             b.set(r::PA_SU_POLY_OFFSET_BACK_OFFSET::reg, float_bits(s.offset_units));
    }
    const uint32_t point = static_cast<uint32_t>(s.point_size * 8.0f);
    ok = ok && emit(b, r::PA_SU_POINT_SIZE::reg,
                    v().with(r::PA_SU_POINT_SIZE::HEIGHT, point & 0xffffu)
                       .with(r::PA_SU_POINT_SIZE::WIDTH, point & 0xffffu)) &&
         emit(b, r::PA_SU_POINT_MINMAX::reg,
              v().with(r::PA_SU_POINT_MINMAX::MIN_SIZE, pack_12p4(s.point_size * 0.5f))
                 .with(r::PA_SU_POINT_MINMAX::MAX_SIZE, pack_12p4(s.point_size * 0.5f))) &&
         emit(b, r::PA_SU_LINE_CNTL::reg,
              v().with(r::PA_SU_LINE_CNTL::WIDTH, pack_12p4(s.line_width * 0.5f))) &&
         b.set(r::PA_SC_LINE_STIPPLE::reg, 0) && b.set(r::PA_SU_LINE_STIPPLE_CNTL::reg, 0) &&
         b.set(r::PA_SC_LINE_CNTL::reg, 0);
    // OpenGL window-system framebuffer edge rule (bottom_edge_rule).
    namespace e = r::PA_SC_EDGERULE;
    ok = ok && emit(b, e::reg, v().with(e::ER_TRI, 0xa).with(e::ER_POINT, 0x5)
                                  .with(e::ER_RECT, 0x9).with(e::ER_LINE_LR, 0x2a)
                                  .with(e::ER_LINE_RL, 0x2a).with(e::ER_LINE_TB, 0xa)
                                  .with(e::ER_LINE_BT, 0xa));
    namespace c = r::PA_CL_CLIP_CNTL;
    ok = ok && emit(b, c::reg, v().with(c::DX_CLIP_SPACE_DEF, 0)
                                  .with(c::ZCLIP_NEAR_DISABLE, 0)
                                  .with(c::ZCLIP_FAR_DISABLE, 0)
                                  .with(c::DX_LINEAR_ATTR_CLIP_ENA, 1));
    namespace m0 = r::PA_SC_MODE_CNTL_0;
    ok = ok && emit(b, m0::reg, v().with(m0::VPORT_SCISSOR_ENABLE, 1)
                                   .with(m0::ALTERNATE_RBS_PER_TILE, 1));
    // Linear colour target: WALK_SIZE 1 and no walk fence; Navi31 has more
    // than two tile pipes (WALK_FENCE_SIZE 3); no HiZ/HiS.
    namespace m1 = r::PA_SC_MODE_CNTL_1;
    ok = ok && emit(b, m1::reg, v().with(m1::WALK_SIZE, 1).with(m1::WALK_FENCE_ENABLE, 0)
                                   .with(m1::WALK_FENCE_SIZE, 3)
                                   .with(m1::OUT_OF_ORDER_WATER_MARK, 7)
                                   .with(m1::WALK_ALIGN8_PRIM_FITS_ST, 1)
                                   .with(m1::SUPERTILE_WALK_ORDER_ENABLE, 1)
                                   .with(m1::TILE_WALK_ORDER_ENABLE, 1)
                                   .with(m1::MULTI_SHADER_ENGINE_PRIM_DISCARD_ENABLE, 1)
                                   .with(m1::FORCE_EOV_CNTDWN_ENABLE, 1)
                                   .with(m1::FORCE_EOV_REZ_ENABLE, 1));
    namespace q = r::DB_EQAA;
    ok = ok && b.set(r::PA_SC_AA_CONFIG::reg, 0) &&
         b.set(r::PA_SC_AA_MASK_X0Y0_X1Y0::reg, 0xffffffffu) &&
         b.set(r::PA_SC_AA_MASK_X0Y1_X1Y1::reg, 0xffffffffu) &&
         emit(b, q::reg, v().with(q::HIGH_QUALITY_INTERSECTIONS, 1)
                            .with(q::INCOHERENT_EQAA_READS, 1)
                            .with(q::STATIC_ANCHOR_ASSOCIATIONS, 1));
    namespace bn = r::PA_SC_BINNER_CNTL_0;
    ok = ok && emit(b, bn::reg, v().with(bn::BINNING_MODE, hw::binning_disable_new_sc)
                                   .with(bn::BIN_SIZE_X_EXTEND, hw::bin_size_128_pixels)
                                   .with(bn::BIN_SIZE_Y_EXTEND, hw::bin_size_128_pixels)
                                   .with(bn::DISABLE_START_OF_PRIM, 1)
                                   .with(bn::FPOVS_PER_BATCH, 63)
                                   .with(bn::OPTIMAL_BIN_SELECTION, 0)
                                   .with(bn::FLUSH_ON_BINNING_TRANSITION, 1));
    namespace ng = r::PA_CL_NGG_CNTL;
    ok = ok && emit(b, ng::reg, v().with(ng::VERTEX_REUSE_DEPTH, 30)) &&
         emit(b, r::SPI_INTERP_CONTROL_0::reg,
              v().with(r::SPI_INTERP_CONTROL_0::FLAT_SHADE_ENA, 1));
    return ok;
}

// ---- Depth, stencil and DB control (si_create_dsa_state, db render state) ----

RX7900_GFX11_NOINLINE bool emit_depth_stencil(Pm4Builder& b, const Framebuffer& fb, const RasterState& s,
                        const ShaderPrograms& p, const DrawInputs& d) noexcept {
    const bool has_depth = fb.depth.present;
    const bool depth = has_depth && !d.depth_copy && s.depth_test;
    const bool stencil = has_depth && !d.depth_copy && s.stencil_test;
    namespace dc = r::DB_DEPTH_CONTROL;
    bool ok = emit(b, dc::reg, v().with(dc::Z_ENABLE, depth ? 1 : 0)
                                  .with(dc::Z_WRITE_ENABLE, depth && s.depth_write ? 1 : 0)
                                  .with(dc::ZFUNC, compare_code(s.depth_func))
                                  .with(dc::STENCIL_ENABLE, stencil ? 1 : 0)
                                  .with(dc::STENCILFUNC, compare_code(s.stencil_func))
                                  .with(dc::BACKFACE_ENABLE, 0));
    namespace sc = r::DB_STENCIL_CONTROL;
    ok = ok && emit(b, sc::reg, v().with(sc::STENCILFAIL, stencil_code(s.stencil_fail))
                                   .with(sc::STENCILZFAIL, stencil_code(s.stencil_depth_fail))
                                   .with(sc::STENCILZPASS, stencil_code(s.stencil_depth_pass))
                                   .with(sc::STENCILFAIL_BF, stencil_code(s.stencil_fail))
                                   .with(sc::STENCILZFAIL_BF, stencil_code(s.stencil_depth_fail))
                                   .with(sc::STENCILZPASS_BF, stencil_code(s.stencil_depth_pass)));
    namespace rf = r::DB_STENCILREFMASK;
    namespace rb = r::DB_STENCILREFMASK_BF;
    ok = ok && emit(b, rf::reg, v().with(rf::STENCILTESTVAL, s.stencil_ref & 0xffu)
                                   .with(rf::STENCILMASK, s.stencil_value_mask & 0xffu)
                                   .with(rf::STENCILWRITEMASK, s.stencil_write_mask & 0xffu)
                                   .with(rf::STENCILOPVAL, 1)) &&
         emit(b, rb::reg, v().with(rb::STENCILTESTVAL_BF, s.stencil_ref & 0xffu)
                             .with(rb::STENCILMASK_BF, s.stencil_value_mask & 0xffu)
                             .with(rb::STENCILWRITEMASK_BF, s.stencil_write_mask & 0xffu)
                             .with(rb::STENCILOPVAL_BF, 1));
    namespace rc = r::DB_RENDER_CONTROL;
    ok = ok && emit(b, rc::reg, v().with(rc::DEPTH_COPY, d.depth_copy ? 1 : 0)
                                   .with(rc::STENCIL_COPY, d.depth_copy ? 1 : 0)
                                   .with(rc::COPY_CENTROID, d.depth_copy ? 1 : 0)
                                   .with(rc::COPY_SAMPLE, 0)
                                   .with(rc::MAX_ALLOWED_TILES_IN_WAVE, 0));
    ok = ok && emit(b, r::DB_COUNT_CONTROL::reg,
                    v().with(r::DB_COUNT_CONTROL::DISABLE_CONSERVATIVE_ZPASS_COUNTS, 1));
    namespace ro = r::DB_RENDER_OVERRIDE;
    ok = ok && emit(b, ro::reg, v().with(ro::FORCE_HIZ_ENABLE, hw::force_disable)
                                   .with(ro::FORCE_HIS_ENABLE0, hw::force_disable)
                                   .with(ro::FORCE_HIS_ENABLE1, hw::force_disable)) &&
         emit(b, r::DB_RENDER_OVERRIDE2::reg,
              v().with(r::DB_RENDER_OVERRIDE2::CENTROID_COMPUTATION_MODE, 1)) &&
         emit(b, r::PA_SC_VRS_OVERRIDE_CNTL::reg,
              v().with(r::PA_SC_VRS_OVERRIDE_CNTL::VRS_OVERRIDE_RATE_COMBINER_MODE,
                       hw::vrs_comb_passthru)
                 .with(r::PA_SC_VRS_OVERRIDE_CNTL::VRS_RATE, 0));
    namespace am = r::DB_ALPHA_TO_MASK;
    ok = ok && emit(b, am::reg, v().with(am::ALPHA_TO_MASK_ENABLE, 0)
                                   .with(am::ALPHA_TO_MASK_OFFSET0, 2)
                                   .with(am::ALPHA_TO_MASK_OFFSET1, 2)
                                   .with(am::ALPHA_TO_MASK_OFFSET2, 2)
                                   .with(am::ALPHA_TO_MASK_OFFSET3, 2));
    // si_shader_ps Z_ORDER case 1; a killing PS (alpha test, stipple, pixel
    // rectangle coverage) or one exporting Z or the stencil test value uses late
    // Z (si_shader_ps). GFX11 export-conflict workaround with blending (1 sample).
    const bool exports_z = gfx11_abi::pixel_source_depth(p.key.pixel_source);
    const bool exports_stencil = gfx11_abi::pixel_source_stencil(p.key.pixel_source);
    const bool late = p.ps.kills != 0 || exports_z || exports_stencil;
    const bool blending = s.blend && !s.logic_op && !d.depth_copy;
    namespace sh = r::DB_SHADER_CONTROL;
    ok = ok && emit(b, sh::reg, v().with(sh::Z_EXPORT_ENABLE, exports_z ? 1 : 0)
                                   .with(sh::STENCIL_TEST_VAL_EXPORT_ENABLE, exports_stencil ? 1 : 0)
                                   .with(sh::Z_ORDER, late ? hw::late_z : hw::early_z_then_late_z)
                                   .with(sh::KILL_ENABLE, p.ps.kills != 0 ? 1 : 0)
                                   .with(sh::OVERRIDE_INTRINSIC_RATE_ENABLE, blending ? 1 : 0)
                                   .with(sh::OVERRIDE_INTRINSIC_RATE, blending ? 2 : 0));
    return ok;
}

// ---- Blend and colour output (si_create_blend_state, cb render state) ----

RX7900_GFX11_NOINLINE bool emit_blend(Pm4Builder& b, const Framebuffer& fb, const RasterState& s,
                const ShaderPrograms& p, const DrawInputs& d) noexcept {
    const uint32_t mask = d.depth_copy ? 0xfu : (s.color_mask & 0xfu);
    const bool logic = s.logic_op && !d.depth_copy;
    const bool blend = s.blend && !logic && !d.depth_copy && mask != 0;
    namespace cc = r::CB_COLOR_CONTROL;
    bool ok = emit(b, cc::reg, v().with(cc::MODE, mask != 0 ? hw::cb_normal : hw::cb_disable)
                                  .with(cc::ROP3, logic ? logic_op_rop3(s.logic_op_code) : 0xccu)
                                  .with(cc::DISABLE_DUAL_QUAD, logic ? 1 : 0));
    namespace bc = r::CB_BLEND0_CONTROL;
    RegisterValue blend_value = v();
    if (blend) {
        blend_value.with(bc::ENABLE, 1)
            .with(bc::COLOR_COMB_FCN, hw::comb_dst_plus_src)
            .with(bc::COLOR_SRCBLEND, blend_code(s.blend_src))
            .with(bc::COLOR_DESTBLEND, blend_code(s.blend_dst));
    }
    // GL_FRONT_AND_BACK: MRT1 (the second buffer) gets the same blend and mask.
    const uint32_t mask1 = fb.dual ? mask : 0u;
    RegisterValue blend1_value = v();
    if (fb.dual && blend) {
        namespace b1 = r::CB_BLEND1_CONTROL;
        blend1_value.with(b1::ENABLE, 1)
            .with(b1::COLOR_COMB_FCN, hw::comb_dst_plus_src)
            .with(b1::COLOR_SRCBLEND, blend_code(s.blend_src))
            .with(b1::COLOR_DESTBLEND, blend_code(s.blend_dst));
    }
    const uint32_t shader_mask = abi::cb_shader_mask_of(p.key);
    ok = ok && emit(b, bc::reg, blend_value) && emit(b, r::CB_BLEND1_CONTROL::reg, blend1_value) &&
         b.set(r::CB_BLEND_RED::reg, 0) && b.set(r::CB_BLEND_GREEN::reg, 0) &&
         b.set(r::CB_BLEND_BLUE::reg, 0) && b.set(r::CB_BLEND_ALPHA::reg, 0);
    ok = ok && emit(b, r::CB_TARGET_MASK::reg, v().with(r::CB_TARGET_MASK::TARGET0_ENABLE, mask)
                                                 .with(r::CB_TARGET_MASK::TARGET1_ENABLE, mask1)) &&
         emit(b, r::CB_SHADER_MASK::reg, v().with(r::CB_SHADER_MASK::OUTPUT0_ENABLE, shader_mask & 0xfu)
                                           .with(r::CB_SHADER_MASK::OUTPUT1_ENABLE, (shader_mask >> 4) & 0xfu));
    // RB+ (si_emit_cb_render_state): a 32_ABGR export needs no down-conversion
    // (NO_CONVERSION; Mesa never pairs 32_ABGR with an 8-bit target, so its
    // 32_R fallback for "no colour outputs" does not apply). Disabled channels
    // turn off value checking; blend optimisation is off (OPT_COMB_NONE) when
    // blending and BLEND_DISABLED otherwise.
    uint32_t downconvert = hw::sx_rt_export_no_conversion;
    if (fb.color.kind == TargetKind::bgra8 && p.ps.col_format == hw::spi_shader_fp16_abgr) {
        downconvert = hw::sx_rt_export_8_8_8_8;
    }
    namespace oc = r::SX_BLEND_OPT_CONTROL;
    namespace bo = r::SX_MRT0_BLEND_OPT;
    namespace bo1 = r::SX_MRT1_BLEND_OPT;
    const uint32_t comb = blend ? hw::opt_comb_none : hw::opt_comb_blend_disabled;
    ok = ok && emit(b, r::SX_PS_DOWNCONVERT::reg, v().with(r::SX_PS_DOWNCONVERT::MRT0, downconvert)
                                                    .with(r::SX_PS_DOWNCONVERT::MRT1, fb.dual ? downconvert : 0u)) &&
         b.set(r::SX_BLEND_OPT_EPSILON::reg, 0) &&
         emit(b, oc::reg, v().with(oc::MRT0_COLOR_OPT_DISABLE, (mask & 7u) == 0 ? 1 : 0)
                             .with(oc::MRT0_ALPHA_OPT_DISABLE, (mask & 8u) == 0 ? 1 : 0)
                             .with(oc::MRT1_COLOR_OPT_DISABLE, (mask1 & 7u) == 0 ? 1 : 0)
                             .with(oc::MRT1_ALPHA_OPT_DISABLE, (mask1 & 8u) == 0 ? 1 : 0)) &&
         emit(b, bo::reg, v().with(bo::COLOR_COMB_FCN, comb).with(bo::ALPHA_COMB_FCN, comb)) &&
         emit(b, bo1::reg, v().with(bo1::COLOR_COMB_FCN, fb.dual ? comb : hw::opt_comb_none)
                              .with(bo1::ALPHA_COMB_FCN, fb.dual ? comb : hw::opt_comb_none)) &&
         b.set(r::CB_FDCC_CONTROL::reg, 0);
    return ok;
}

// ---- NGG and PS shader registers (gfx10_shader_ngg, si_shader_ps) ----

bool valid_programs(const ShaderPrograms& p, Primitive primitive) noexcept {
    const uint32_t vpp = static_cast<uint32_t>(primitive);
    return abi::key_valid(p.key) && p.key.vertices_per_primitive == vpp &&
           p.key.vertex_input == abi::vertex_input_clip_space &&
           p.vs_va != 0 && p.ps_va != 0 && aligned(p.vs_va, abi::shader_code_alignment) &&
           aligned(p.ps_va, abi::shader_code_alignment) &&
           p.vs.code_dwords != 0 && p.ps.code_dwords != 0 &&
           p.vs.code_dwords * 4u <= abi::shader_max_bytes &&
           p.ps.code_dwords * 4u <= abi::shader_max_bytes &&
           p.vs.scratch_bytes == 0 && p.ps.scratch_bytes == 0 &&
           p.vs.user_sgpr_count == abi::vs_user_sgpr_count &&
           p.ps.user_sgpr_count == abi::ps_user_sgprs(p.key) &&
           p.vs.pos_export_count == 1 && p.vs.param_count >= 1 && p.vs.param_count <= 4 &&
           p.ps.interpolant_count <= 4 && p.vs.vgpr_count <= 256 && p.ps.vgpr_count <= 256 &&
           p.vs.max_es_verts <= abi::ngg_max_verts_per_subgroup &&
           p.vs.max_gs_prims <= abi::ngg_max_prims_per_subgroup;
}

RX7900_GFX11_NOINLINE bool emit_shaders(Pm4Builder& b, const ShaderPrograms& p, Primitive primitive) noexcept {
    const uint32_t vpp = static_cast<uint32_t>(primitive);
    namespace g1 = r::SPI_SHADER_PGM_RSRC1_GS;
    namespace g2 = r::SPI_SHADER_PGM_RSRC2_GS;
    namespace g3 = r::SPI_SHADER_PGM_RSRC3_GS;
    namespace g4 = r::SPI_SHADER_PGM_RSRC4_GS;
    const uint32_t vs_lds_blocks = (p.vs.lds_bytes + 511u) / 512u;
    bool ok = emit(b, r::SPI_SHADER_PGM_LO_ES::reg,
                   v().with(r::SPI_SHADER_PGM_LO_ES::MEM_BASE, low(p.vs_va >> 8))) &&
              emit(b, r::SPI_SHADER_PGM_HI_ES::reg,
                   v().with(r::SPI_SHADER_PGM_HI_ES::MEM_BASE, low(p.vs_va >> 40))) &&
              emit(b, g1::reg, v().with(g1::VGPRS, abi::rsrc1_vgpr_blocks(p.vs.vgpr_count))
                                  .with(g1::FLOAT_MODE, p.vs.float_mode)
                                  .with(g1::DX10_CLAMP, 1)
                                  .with(g1::MEM_ORDERED, 0)
                                  .with(g1::GS_VGPR_COMP_CNT, p.vs.gs_vgpr_comp_cnt)) &&
              emit(b, g2::reg, v().with(g2::SCRATCH_EN, 0)
                                  .with(g2::USER_SGPR, p.vs.user_sgpr_count & 0x1fu)
                                  .with(g2::USER_SGPR_MSB, p.vs.user_sgpr_count >> 5)
                                  .with(g2::ES_VGPR_COMP_CNT, p.vs.es_vgpr_comp_cnt)
                                  .with(g2::OC_LDS_EN, 0)
                                  .with(g2::LDS_SIZE, vs_lds_blocks)) &&
              emit(b, g3::reg, v().with(g3::CU_EN, 0xffff).with(g3::WAVE_LIMIT, 0x3f)) &&
              emit(b, g4::reg, v().with(g4::CU_EN, 1).with(g4::SPI_SHADER_LATE_ALLOC_GS, 0)
                                  .with(g4::INST_PREF_SIZE, abi::inst_pref_size(p.vs.code_dwords * 4u)));
    namespace pc = r::GE_PC_ALLOC;
    // No late allocation: OVERSUB_EN 0 and NUM_PC_LINES(0 - 1) as Mesa writes it.
    ok = ok && emit(b, pc::reg, v().with(pc::OVERSUB_EN, 0)
                                   .with(pc::NUM_PC_LINES, field_max(pc::NUM_PC_LINES)));
    ok = ok && b.set(r::GE_MAX_OUTPUT_PER_SUBGROUP::reg, abi::ge_max_output_per_subgroup) &&
         b.set(r::GE_NGG_SUBGRP_CNTL::reg, abi::ge_ngg_subgrp_cntl) &&
         b.set(r::VGT_PRIMITIVEID_EN::reg, abi::vgt_primitiveid_en) &&
         b.set(r::VGT_GS_MAX_VERT_OUT::reg, abi::vgt_gs_max_vert_out) &&
         b.set(r::VGT_GS_INSTANCE_CNT::reg, 0) &&
         b.set(r::SPI_VS_OUT_CONFIG::reg, abi::spi_vs_out_config(p.vs.param_count)) &&
         b.set(r::SPI_SHADER_POS_FORMAT::reg, abi::spi_shader_pos_format) &&
         b.set(r::SPI_SHADER_IDX_FORMAT::reg, abi::spi_shader_idx_format) &&
         b.set(r::PA_CL_VTE_CNTL::reg, abi::pa_cl_vte_cntl) &&
         emit(b, r::PA_CL_VS_OUT_CNTL::reg,
              v().with(r::PA_CL_VS_OUT_CNTL::BYPASS_VTX_RATE_COMBINER, 1)
                 .with(r::PA_CL_VS_OUT_CNTL::BYPASS_PRIM_RATE_COMBINER, 1)) &&
         b.set(r::VGT_SHADER_STAGES_EN::reg, abi::vgt_shader_stages_en) &&
         b.set(r::VGT_REUSE_OFF::reg, 0) && b.set(r::VGT_DRAW_PAYLOAD_CNTL::reg, 0) &&
         b.set(r::VGT_MULTI_PRIM_IB_RESET_INDX::reg, 0) &&
         b.set(r::GE_CNTL::reg, abi::ge_cntl) &&
         b.set(r::VGT_GS_OUT_PRIM_TYPE::reg, abi::vgt_gs_out_prim_type(vpp));
    static_assert(abi::pa_cl_vs_out_cntl == 0, "ABI exports no clip/misc vector");

    namespace p1 = r::SPI_SHADER_PGM_RSRC1_PS;
    namespace p2 = r::SPI_SHADER_PGM_RSRC2_PS;
    namespace p4 = r::SPI_SHADER_PGM_RSRC4_PS;
    ok = ok && emit(b, r::SPI_SHADER_PGM_LO_PS::reg,
                    v().with(r::SPI_SHADER_PGM_LO_PS::MEM_BASE, low(p.ps_va >> 8))) &&
         emit(b, r::SPI_SHADER_PGM_HI_PS::reg,
              v().with(r::SPI_SHADER_PGM_HI_PS::MEM_BASE, low(p.ps_va >> 40))) &&
         emit(b, p1::reg, v().with(p1::VGPRS, abi::rsrc1_vgpr_blocks(p.ps.vgpr_count))
                             .with(p1::FLOAT_MODE, p.ps.float_mode)
                             .with(p1::DX10_CLAMP, 1)
                             .with(p1::MEM_ORDERED, 0)) &&
         emit(b, p2::reg, v().with(p2::SCRATCH_EN, 0)
                             .with(p2::USER_SGPR, p.ps.user_sgpr_count & 0x1fu)
                             .with(p2::USER_SGPR_MSB, p.ps.user_sgpr_count >> 5)
                             .with(p2::EXTRA_LDS_SIZE, 0)) &&
         emit(b, p4::reg, v().with(p4::CU_EN, 0)
                             .with(p4::INST_PREF_SIZE, abi::inst_pref_size(p.ps.code_dwords * 4u)));
    ok = ok && b.set(r::SPI_PS_INPUT_ENA::reg, p.ps.spi_ps_input_ena) &&
         b.set(r::SPI_PS_INPUT_ADDR::reg, p.ps.spi_ps_input_ena) &&
         b.set(r::SPI_BARYC_CNTL::reg, abi::spi_baryc_cntl) &&
         b.set(r::SPI_PS_IN_CONTROL::reg, abi::spi_ps_in_control(p.ps.interpolant_count)) &&
         b.set(r::SPI_SHADER_Z_FORMAT::reg, p.ps.z_format) &&
         b.set(r::SPI_SHADER_COL_FORMAT::reg, p.ps.col_format);
    for (uint32_t slot = 0; ok && slot < p.ps.interpolant_count; ++slot) {
        ok = b.set(r::ps_input_cntl(slot), abi::spi_ps_input_cntl(slot, ((p.ps.flat_mask >> slot) & 1u) != 0));
    }
    return ok;
}

RX7900_GFX11_NOINLINE bool emit_user_data(Pm4Builder& b, const ShaderPrograms& p, const DrawInputs& d,
                    const DeviceRings& rings) noexcept {
    const abi::BufferDescriptor vertices = abi::raw_buffer_descriptor(d.vertex_va, d.vertex_bytes);
    const abi::BufferDescriptor ring = abi::attribute_ring_descriptor(
        rings.attribute_ring_va, attribute_ring_bytes(rings), p.vs.param_count);
    bool ok = true;
    for (uint32_t i = 0; ok && i < 4; ++i) {
        ok = b.set(r::user_data_gs(abi::vs_user_data_index(abi::vs_sgpr_vertex_buffer) + i), vertices.words[i]) &&
             b.set(r::user_data_gs(abi::vs_user_data_index(abi::vs_sgpr_attribute_ring) + i), ring.words[i]) &&
             // Clip-space keys read no VertexConstants: a zero (NUM_RECORDS 0) V#.
             b.set(r::user_data_gs(abi::vs_user_data_index(abi::vs_sgpr_constants) + i), 0);
    }
    ok = ok && b.set(r::user_data_gs(abi::vs_user_data_index(abi::vs_sgpr_base_vertex)), 0);
    for (uint32_t i = 0; ok && i < 8; ++i) {
        ok = b.set(r::user_data_ps(abi::ps_sgpr_texture + i), p.key.textured ? d.texture.words[i] : 0);
    }
    for (uint32_t i = 0; ok && i < 4; ++i) {
        ok = b.set(r::user_data_ps(abi::ps_sgpr_sampler + i), p.key.textured ? d.sampler.words[i] : 0);
    }
    if (abi::uses_ps_constants(p.key)) {
        // v4: the PS constant block, read-only and bounded to exactly its bytes.
        const abi::BufferDescriptor constants = abi::raw_buffer_descriptor(d.ps_constants_va, d.ps_constants_bytes);
        for (uint32_t i = 0; ok && i < 4; ++i) {
            ok = b.set(r::user_data_ps(abi::ps_sgpr_constants + i), constants.words[i]);
        }
    }
    return ok;
}

RX7900_GFX11_NOINLINE bool emit_ps_constants(Pm4Builder& b, const RasterState& s) noexcept {
    bool ok = true;
    for (uint32_t i = 0; ok && i < 4; ++i) {
        ok = b.set(r::user_data_ps(abi::ps_sgpr_env_color + i), float_bits(s.env_color[i]));
    }
    for (uint32_t i = 0; ok && i < 3; ++i) {
        ok = b.set(r::user_data_ps(abi::ps_sgpr_fog_color + i), float_bits(s.fog_color[i]));
    }
    return ok && b.set(r::user_data_ps(abi::ps_sgpr_alpha_ref), float_bits(s.alpha_ref));
}

RX7900_GFX11_NOINLINE bool emit_draw_packets(Pm4Builder& b, Primitive primitive, uint32_t vertex_count) noexcept {
    const uint32_t prim_type = primitive == Primitive::points ? hw::di_pt_pointlist :
                               primitive == Primitive::lines ? hw::di_pt_linelist : hw::di_pt_trilist;
    const uint32_t instances[1] = {1};
    // VGT_DRAW_INITIATOR SOURCE_SELECT = DI_SRC_SEL_AUTO_INDEX (2).
    const uint32_t draw[2] = {vertex_count, 2};
    return emit(b, r::VGT_PRIMITIVE_TYPE::reg, v().with(r::VGT_PRIMITIVE_TYPE::PRIM_TYPE, prim_type)) &&
           b.packet(op::num_instances, instances, 1) &&
           b.packet(op::draw_index_auto, draw, 2);
}

} // namespace

// ---------------------------------------------------------------------------

uint32_t float_bits(float value) noexcept {
    uint32_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value), "IEEE-754 binary32");
    memcpy(&bits, &value, sizeof(bits));
    return bits;
}

// si_pack_float_12p4: unsigned 12.4 fixed point, saturating.
uint32_t pack_12p4(float value) noexcept {
    if (!(value > 0)) {
        return 0;
    }
    if (value >= 4096.0f) {
        return 0xffffu;
    }
    return static_cast<uint32_t>(value * 16.0f);
}

// GL logic op (GL_CLEAR + code) to the 4-bit truth table over (src 0xC, dst
// 0xA), replicated into ROP3 as radeonsi does: ROP3 = rop | rop << 4.
uint32_t logic_op_rop3(uint32_t code) noexcept {
    static constexpr uint8_t table[16] = {
        0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe,   // clear and and_reverse copy and_inverted noop xor or
        0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf};  // nor equiv invert or_reverse copy_inverted or_inverted nand set
    const uint32_t rop = table[code & 15u];
    return rop | (rop << 4);
}

bool depth_layout(uint32_t width, uint32_t height, DepthLayout& layout) noexcept {
    layout = {};
    if (width == 0 || height == 0 || width > max_dimension || height > max_dimension) {
        return false;
    }
    const uint64_t z_columns = (width + 127u) / 128u;
    const uint64_t z_rows = (height + 127u) / 128u;
    const uint64_t s_columns = (width + 255u) / 256u;
    const uint64_t s_rows = (height + 255u) / 256u;
    layout.z_bytes = z_columns * z_rows * depth_block_bytes;
    layout.stencil_offset = layout.z_bytes;
    layout.total_bytes = layout.stencil_offset + s_columns * s_rows * depth_block_bytes;
    return true;
}

void Pm4Builder::begin(sdma::CommandBuffer output) noexcept {
    output_ = output;
    dwords_ = 0;
    status_ = output.size() == 0 ? StateStatus::output_too_small : StateStatus::success;
    memset(&context_, 0, sizeof(context_));
    memset(&sh_, 0, sizeof(sh_));
    memset(&uconfig_, 0, sizeof(uconfig_));
}

void Pm4Builder::fail(StateStatus status) noexcept {
    if (status_ == StateStatus::success) {
        status_ = status;
    }
}

bool Pm4Builder::put(uint32_t word) noexcept {
    if (status_ != StateStatus::success) {
        return false;
    }
    const size_t offset = static_cast<size_t>(dwords_) * 4u;
    if (offset + 4u > output_.size() || !output_.write_u32(offset, word)) {
        fail(StateStatus::output_too_small);
        return false;
    }
    ++dwords_;
    return true;
}

Pm4Builder::Shadow* Pm4Builder::shadow_for(r::Space space) noexcept {
    return space == r::Space::context ? &context_ : space == r::Space::sh ? &sh_ :
           space == r::Space::uconfig ? &uconfig_ : nullptr;
}

bool Pm4Builder::set(r::Register reg, uint32_t value) noexcept {
    if (status_ != StateStatus::success) {
        return false;
    }
    const r::Space space = r::space(reg);
    Shadow* shadow = shadow_for(space);
    const uint32_t index = r::absolute_dword(reg) - (shadow != nullptr ? space_base(space) : 0);
    if (shadow == nullptr || index >= window || !r::allowed_register(reg)) {
        fail(StateStatus::disallowed_register);
        return false;
    }
    if (shadow->known[index] != 0 && shadow->value[index] == value) {
        return true;
    }
    const uint32_t words[3] = {packet3(set_opcode(space), 2), index, value};
    const size_t needed = static_cast<size_t>(dwords_ + 3u) * 4u;
    if (needed > output_.size()) {
        fail(StateStatus::output_too_small);
        return false;
    }
    for (uint32_t word : words) {
        put(word);
    }
    shadow->known[index] = 1;
    shadow->value[index] = value;
    return status_ == StateStatus::success;
}

bool Pm4Builder::packet(uint32_t opcode, const uint32_t* body, uint32_t body_dwords) noexcept {
    if (status_ != StateStatus::success) {
        return false;
    }
    if (body == nullptr || body_dwords == 0 || body_dwords > 0x4000u ||
        opcode == op::set_context_reg || opcode == op::set_sh_reg ||
        opcode == op::set_uconfig_reg) {
        fail(StateStatus::invalid_argument);   // register writes only through set()
        return false;
    }
    const size_t needed = static_cast<size_t>(dwords_ + 1u + body_dwords) * 4u;
    if (needed > output_.size()) {
        fail(StateStatus::output_too_small);
        return false;
    }
    put(packet3(opcode, body_dwords));
    for (uint32_t index = 0; index < body_dwords; ++index) {
        put(body[index]);
    }
    return status_ == StateStatus::success;
}

bool Pm4Builder::pad(uint32_t alignment_dwords) noexcept {
    if (alignment_dwords == 0 || (alignment_dwords & (alignment_dwords - 1u)) != 0) {
        fail(StateStatus::invalid_argument);
        return false;
    }
    while (status_ == StateStatus::success && (dwords_ & (alignment_dwords - 1u)) != 0) {
        put(0xffff1000u);   // PACKET3(NOP, 0x3FFF): header-only NOP (gfx11_ring_nop)
    }
    return status_ == StateStatus::success;
}

StateStatus emit_preamble(Pm4Builder& b, const DeviceRings& rings) noexcept {
    if (!b.ok()) {
        return b.status();
    }
    if (rings.attribute_ring_va == 0 || !aligned(rings.attribute_ring_va, 65536) ||
        rings.attribute_ring_per_se_bytes == 0 || !aligned(rings.attribute_ring_per_se_bytes, 65536) ||
        rings.shader_engines == 0 || rings.shader_engines > 8 ||
        static_cast<uint64_t>(rings.attribute_ring_per_se_bytes) * rings.shader_engines > (16u << 20)) {
        b.fail(StateStatus::invalid_argument);
        return b.status();
    }
    // CONTEXT_CONTROL CC0_UPDATE_LOAD_ENABLES(1), CC1_UPDATE_SHADOW_ENABLES(1).
    const uint32_t control[2] = {1u << 31, 1u << 31};
    const uint32_t clear[1] = {0};
    b.packet(op::context_control, control, 2);
    b.packet(op::clear_state, clear, 1);

    // Shader registers (PS, ES/GS, HS) of gfx10_init_gfx_preamble_state.
    namespace p3 = r::SPI_SHADER_PGM_RSRC3_PS;
    namespace rq = r::SPI_SHADER_REQ_CTRL_PS;
    namespace h3 = r::SPI_SHADER_PGM_RSRC3_HS;
    emit(b, p3::reg, v().with(p3::CU_EN, 0xffff).with(p3::WAVE_LIMIT, 0x3f).with(p3::LDS_GROUP_SIZE, 1));
    emit(b, rq::reg, v().with(rq::SOFT_GROUPING_EN, 1).with(rq::NUMBER_OF_REQUESTS_PER_CU, 3));
    // Constant tables live in read-only data, not in the (AMD64) kernel frame.
    static constexpr r::Register zeroed[] = {
        r::SPI_SHADER_USER_ACCUM_PS_0::reg, r::SPI_SHADER_USER_ACCUM_PS_1::reg,
        r::SPI_SHADER_USER_ACCUM_PS_2::reg, r::SPI_SHADER_USER_ACCUM_PS_3::reg,
        r::SPI_SHADER_USER_ACCUM_ESGS_0::reg, r::SPI_SHADER_USER_ACCUM_ESGS_1::reg,
        r::SPI_SHADER_USER_ACCUM_ESGS_2::reg, r::SPI_SHADER_USER_ACCUM_ESGS_3::reg,
        r::SPI_SHADER_USER_ACCUM_LSHS_0::reg, r::SPI_SHADER_USER_ACCUM_LSHS_1::reg,
        r::SPI_SHADER_USER_ACCUM_LSHS_2::reg, r::SPI_SHADER_USER_ACCUM_LSHS_3::reg,
        r::TA_BC_BASE_ADDR::reg, r::TA_BC_BASE_ADDR_HI::reg, r::SPI_TMPRING_SIZE::reg,
        r::DB_HTILE_DATA_BASE::reg, r::DB_HTILE_DATA_BASE_HI::reg, r::DB_HTILE_SURFACE::reg,
        r::DB_STENCIL_CLEAR::reg, r::DB_DEPTH_CLEAR::reg, r::DB_DEPTH_BOUNDS_MIN::reg,
        r::PA_SC_WINDOW_OFFSET::reg};
    for (const auto& reg : zeroed) {
        b.set(reg, 0);
    }
    b.set(r::DB_DEPTH_BOUNDS_MAX::reg, float_bits(1.0f));
    emit(b, h3::reg, v().with(h3::CU_EN, 0xffff).with(h3::WAVE_LIMIT, 0x3f));

    namespace dl = r::DB_RMI_L2_CACHE_CONTROL;
    emit(b, dl::reg, v().with(dl::Z_WR_POLICY, hw::cache_stream).with(dl::S_WR_POLICY, hw::cache_stream)
                        .with(dl::HTILE_WR_POLICY, hw::cache_stream)
                        .with(dl::ZPCPSD_WR_POLICY, hw::cache_stream)
                        .with(dl::Z_RD_POLICY, hw::cache_noa_rd).with(dl::S_RD_POLICY, hw::cache_noa_rd)
                        .with(dl::HTILE_RD_POLICY, hw::cache_noa_rd));
    namespace cl = r::CB_RMI_GL2_CACHE_CONTROL;
    emit(b, cl::reg, v().with(cl::COLOR_WR_POLICY, hw::cache_stream)
                        .with(cl::COLOR_RD_POLICY, hw::cache_noa_rd)
                        .with(cl::DCC_WR_POLICY, hw::cache_stream)
                        .with(cl::DCC_RD_POLICY, hw::cache_noa_rd));
    b.set(r::SX_PS_DOWNCONVERT_CONTROL::reg, 0xff);
    emit(b, r::PA_SU_PRIM_FILTER_CNTL::reg,
         v().with(r::PA_SU_PRIM_FILTER_CNTL::XMAX_RIGHT_EXCLUSION, 1)
            .with(r::PA_SU_PRIM_FILTER_CNTL::YMAX_BOTTOM_EXCLUSION, 1));
    emit(b, r::PA_SU_SMALL_PRIM_FILTER_CNTL::reg,
         v().with(r::PA_SU_SMALL_PRIM_FILTER_CNTL::SMALL_PRIM_FILTER_ENABLE, 1));
    emit(b, r::PA_CL_VRS_CNTL::reg,
         v().with(r::PA_CL_VRS_CNTL::VERTEX_RATE_COMBINER_MODE, hw::vrs_comb_override)
            .with(r::PA_CL_VRS_CNTL::SAMPLE_ITER_COMBINER_MODE, hw::vrs_comb_override));
    b.set(r::VGT_HOS_MAX_TESS_LEVEL::reg, float_bits(64.0f));
    b.set(r::VGT_ESGS_RING_ITEMSIZE::reg, 1);
    namespace td = r::VGT_TESS_DISTRIBUTION;
    emit(b, td::reg, v().with(td::ACCUM_ISOLINE, 128).with(td::ACCUM_TRI, 128)
                        .with(td::ACCUM_QUAD, 128).with(td::DONUT_SPLIT, 24).with(td::TRAP_SPLIT, 6));
    emit(b, r::PA_SC_BINNER_CNTL_1::reg,
         v().with(r::PA_SC_BINNER_CNTL_1::MAX_ALLOC_COUNT, 16)
            .with(r::PA_SC_BINNER_CNTL_1::MAX_PRIM_PER_BATCH, 1023));
    emit(b, r::PA_SC_NGG_MODE_CNTL::reg, v().with(r::PA_SC_NGG_MODE_CNTL::MAX_DEALLOCS_IN_WAVE, 16));
    emit(b, r::PA_SC_CLIPRECT_RULE::reg, v().with(r::PA_SC_CLIPRECT_RULE::CLIP_RULE, 0xffff));
    for (uint32_t index = 1; index < 8; ++index) {
        // CB_COLORn_INFO (n = 1..7) FORMAT = COLOR_INVALID: one colour target.
        static constexpr r::Register infos[] = {r::CB_COLOR1_INFO::reg, r::CB_COLOR2_INFO::reg, r::CB_COLOR3_INFO::reg,
                                     r::CB_COLOR4_INFO::reg, r::CB_COLOR5_INFO::reg, r::CB_COLOR6_INFO::reg,
                                     r::CB_COLOR7_INFO::reg};
        b.set(infos[index - 1], hw::color_invalid);
    }

    // UCONFIG registers.
    b.set(r::GE_MIN_VTX_INDX::reg, 0);
    b.set(r::GE_INDX_OFFSET::reg, 0);
    emit(b, r::GE_MULTI_PRIM_IB_RESET_EN::reg,
         v().with(r::GE_MULTI_PRIM_IB_RESET_EN::DISABLE_FOR_AUTO_INDEX, 1));
    b.set(r::GE_MAX_VTX_INDX::reg, 0xffffffffu);
    b.set(r::VGT_INSTANCE_BASE_ID::reg, 0);
    b.set(r::GE_STEREO_CNTL::reg, 0);
    b.set(r::GE_USER_VGPR_EN::reg, 0);
    b.set(r::PA_SU_LINE_STIPPLE_VALUE::reg, 0);
    b.set(r::PA_SC_LINE_STIPPLE_STATE::reg, 0);

    // Attribute ring (si_state_shaders.cpp): wait for idle with a bottom-of-pipe
    // RELEASE_MEM that increments the PWS counter, wait for it with ACQUIRE_MEM,
    // then program the throttles and the ring.
    static constexpr uint32_t release[7] = {hw::event_bottom_of_pipe_ts | (5u << 8) | (1u << 31), 0, 0, 0, 0, 0, 0};
    static constexpr uint32_t acquire[7] = {(hw::pws_stage_cp_me << 11) | (hw::pws_counter_ts << 14) | (1u << 17),
                                 0xffffffffu, 0x01ffffffu, 0, 0, 1u << 31, 0};
    b.packet(op::release_mem, release, 7);
    b.packet(op::acquire_mem, acquire, 7);
    b.set(r::SPI_GS_THROTTLE_CNTL1::reg, 0x12355123u);
    b.set(r::SPI_GS_THROTTLE_CNTL2::reg, 0x1544du);
    emit(b, r::SPI_ATTRIBUTE_RING_BASE::reg,
         v().with(r::SPI_ATTRIBUTE_RING_BASE::BASE, low(rings.attribute_ring_va >> 16)));
    emit(b, r::SPI_ATTRIBUTE_RING_SIZE::reg,
         v().with(r::SPI_ATTRIBUTE_RING_SIZE::MEM_SIZE, (rings.attribute_ring_per_se_bytes >> 16) - 1u)
            .with(r::SPI_ATTRIBUTE_RING_SIZE::BIG_PAGE, 0)
            .with(r::SPI_ATTRIBUTE_RING_SIZE::L1_POLICY, 1));
    return b.status();
}

StateStatus emit_framebuffer(Pm4Builder& b, const Framebuffer& fb) noexcept {
    if (!b.ok()) {
        return b.status();
    }
    if (!valid_framebuffer(fb)) {
        b.fail(StateStatus::invalid_argument);
        return b.status();
    }
    const bool color = fb.color.kind == TargetKind::bgra8;
    namespace ci = r::CB_COLOR0_INFO;
    // ac_init_cb_surface: B8G8R8A8_UNORM = COLOR_8_8_8_8 SWAP_ALT UNORM with
    // blend clamp; Z32_FLOAT_S8X24 copy target = COLOR_X24_8_32_FLOAT FLOAT
    // with blend bypass and round mode.
    emit(b, r::CB_COLOR0_BASE::reg, v().with(r::CB_COLOR0_BASE::BASE_256B, low(fb.color.va >> 8)));
    emit(b, r::CB_COLOR0_BASE_EXT::reg, v().with(r::CB_COLOR0_BASE_EXT::BASE_256B, low(fb.color.va >> 40)));
    b.set(r::CB_COLOR0_VIEW::reg, 0);
    emit(b, ci::reg, v().with(ci::FORMAT, color ? hw::color_8_8_8_8 : hw::color_x24_8_32_float)
                        .with(ci::NUMBER_TYPE, color ? hw::number_unorm : hw::number_float)
                        .with(ci::COMP_SWAP, color ? hw::swap_alt : hw::swap_std)
                        .with(ci::BLEND_CLAMP, color ? 1 : 0)
                        .with(ci::BLEND_BYPASS, color ? 0 : 1)
                        .with(ci::SIMPLE_FLOAT, 1)
                        .with(ci::ROUND_MODE, color ? 0 : 1));
    b.set(r::CB_COLOR0_ATTRIB::reg, 0);
    emit(b, r::CB_COLOR0_FDCC_CONTROL::reg,
         v().with(r::CB_COLOR0_FDCC_CONTROL::MAX_UNCOMPRESSED_BLOCK_SIZE, 2));  // 256B, DCC off
    b.set(r::CB_COLOR0_DCC_BASE::reg, 0);
    b.set(r::CB_COLOR0_DCC_BASE_EXT::reg, 0);
    // GFX10.3+ custom linear pitch: MIP0_WIDTH carries the pitch.
    emit(b, r::CB_COLOR0_ATTRIB2::reg,
         v().with(r::CB_COLOR0_ATTRIB2::MIP0_WIDTH, fb.color.pitch_pixels - 1u)
            .with(r::CB_COLOR0_ATTRIB2::MIP0_HEIGHT, fb.height - 1u)
            .with(r::CB_COLOR0_ATTRIB2::MAX_MIP, 0));
    emit(b, r::CB_COLOR0_ATTRIB3::reg,
         v().with(r::CB_COLOR0_ATTRIB3::MIP0_DEPTH, 0)
            .with(r::CB_COLOR0_ATTRIB3::COLOR_SW_MODE, hw::sw_linear)
            .with(r::CB_COLOR0_ATTRIB3::RESOURCE_TYPE, 1));   // RADEON_RESOURCE_2D

    // MRT1: the second BGRA8 buffer of GL_FRONT_AND_BACK, else no target.
    namespace c1 = r::CB_COLOR1_INFO;
    if (fb.dual) {
        emit(b, r::CB_COLOR1_BASE::reg, v().with(r::CB_COLOR1_BASE::BASE_256B, low(fb.color1.va >> 8)));
        emit(b, r::CB_COLOR1_BASE_EXT::reg, v().with(r::CB_COLOR1_BASE_EXT::BASE_256B, low(fb.color1.va >> 40)));
        b.set(r::CB_COLOR1_VIEW::reg, 0);
        emit(b, c1::reg, v().with(c1::FORMAT, hw::color_8_8_8_8).with(c1::NUMBER_TYPE, hw::number_unorm)
                            .with(c1::COMP_SWAP, hw::swap_alt).with(c1::BLEND_CLAMP, 1)
                            .with(c1::BLEND_BYPASS, 0).with(c1::SIMPLE_FLOAT, 1).with(c1::ROUND_MODE, 0));
        b.set(r::CB_COLOR1_ATTRIB::reg, 0);
        emit(b, r::CB_COLOR1_FDCC_CONTROL::reg,
             v().with(r::CB_COLOR1_FDCC_CONTROL::MAX_UNCOMPRESSED_BLOCK_SIZE, 2));
        b.set(r::CB_COLOR1_DCC_BASE::reg, 0);
        b.set(r::CB_COLOR1_DCC_BASE_EXT::reg, 0);
        emit(b, r::CB_COLOR1_ATTRIB2::reg,
             v().with(r::CB_COLOR1_ATTRIB2::MIP0_WIDTH, fb.color1.pitch_pixels - 1u)
                .with(r::CB_COLOR1_ATTRIB2::MIP0_HEIGHT, fb.height - 1u)
                .with(r::CB_COLOR1_ATTRIB2::MAX_MIP, 0));
        emit(b, r::CB_COLOR1_ATTRIB3::reg,
             v().with(r::CB_COLOR1_ATTRIB3::MIP0_DEPTH, 0)
                .with(r::CB_COLOR1_ATTRIB3::COLOR_SW_MODE, hw::sw_linear)
                .with(r::CB_COLOR1_ATTRIB3::RESOURCE_TYPE, 1));
    } else {
        emit(b, c1::reg, v().with(c1::FORMAT, hw::color_invalid));
    }

    namespace zi = r::DB_Z_INFO;
    namespace si = r::DB_STENCIL_INFO;
    if (fb.depth.present) {
        emit(b, zi::reg, v().with(zi::FORMAT, hw::z_32_float).with(zi::NUM_SAMPLES, 0)
                            .with(zi::SW_MODE, hw::sw_64kb_z_x).with(zi::MAXMIP, 0)
                            .with(zi::ITERATE_256, 1).with(zi::TILE_SURFACE_ENABLE, 0));
        emit(b, si::reg, v().with(si::FORMAT, hw::stencil_8).with(si::SW_MODE, hw::sw_64kb_z_x)
                            .with(si::ITERATE_256, 1));
        const uint32_t z = low(fb.depth.z_va >> 8);
        const uint32_t s = low(fb.depth.stencil_va >> 8);
        const uint32_t z_hi = low(fb.depth.z_va >> 40);
        const uint32_t s_hi = low(fb.depth.stencil_va >> 40);
        emit(b, r::DB_Z_READ_BASE::reg, v().with(r::DB_Z_READ_BASE::BASE_256B, z));
        emit(b, r::DB_Z_WRITE_BASE::reg, v().with(r::DB_Z_WRITE_BASE::BASE_256B, z));
        emit(b, r::DB_STENCIL_READ_BASE::reg, v().with(r::DB_STENCIL_READ_BASE::BASE_256B, s));
        emit(b, r::DB_STENCIL_WRITE_BASE::reg, v().with(r::DB_STENCIL_WRITE_BASE::BASE_256B, s));
        emit(b, r::DB_Z_READ_BASE_HI::reg, v().with(r::DB_Z_READ_BASE_HI::BASE_HI, z_hi));
        emit(b, r::DB_Z_WRITE_BASE_HI::reg, v().with(r::DB_Z_WRITE_BASE_HI::BASE_HI, z_hi));
        emit(b, r::DB_STENCIL_READ_BASE_HI::reg, v().with(r::DB_STENCIL_READ_BASE_HI::BASE_HI, s_hi));
        emit(b, r::DB_STENCIL_WRITE_BASE_HI::reg, v().with(r::DB_STENCIL_WRITE_BASE_HI::BASE_HI, s_hi));
        emit(b, r::DB_DEPTH_SIZE_XY::reg, v().with(r::DB_DEPTH_SIZE_XY::X_MAX, fb.width - 1u)
                                             .with(r::DB_DEPTH_SIZE_XY::Y_MAX, fb.height - 1u));
        b.set(r::DB_DEPTH_VIEW::reg, 0);
    } else {
        emit(b, zi::reg, v().with(zi::FORMAT, hw::z_invalid).with(zi::NUM_SAMPLES, 0));
        emit(b, si::reg, v().with(si::FORMAT, hw::stencil_invalid));
    }

    namespace wt = r::PA_SC_WINDOW_SCISSOR_TL;
    namespace wb = r::PA_SC_WINDOW_SCISSOR_BR;
    namespace st = r::PA_SC_SCREEN_SCISSOR_TL;
    namespace sb = r::PA_SC_SCREEN_SCISSOR_BR;
    namespace gt = r::PA_SC_GENERIC_SCISSOR_TL;
    namespace gb = r::PA_SC_GENERIC_SCISSOR_BR;
    emit(b, wt::reg, v().with(wt::TL_X, 0).with(wt::TL_Y, 0).with(wt::WINDOW_OFFSET_DISABLE, 1));
    emit(b, wb::reg, v().with(wb::BR_X, fb.width).with(wb::BR_Y, fb.height));
    emit(b, st::reg, v().with(st::TL_X, 0).with(st::TL_Y, 0));
    emit(b, sb::reg, v().with(sb::BR_X, fb.width).with(sb::BR_Y, fb.height));
    emit(b, gt::reg, v().with(gt::TL_X, 0).with(gt::TL_Y, 0).with(gt::WINDOW_OFFSET_DISABLE, 1));
    emit(b, gb::reg, v().with(gb::BR_X, fb.width).with(gb::BR_Y, fb.height));
    return b.status();
}

StateStatus emit_draw(Pm4Builder& b, const Framebuffer& fb, const RasterState& s,
                      const ShaderPrograms& p, const DrawInputs& d,
                      const DeviceRings& rings) noexcept {
    if (!b.ok()) {
        return b.status();
    }
    const uint32_t vpp = static_cast<uint32_t>(d.primitive);
    const uint32_t stride = abi::record_stride(p.key);
    if (vpp < 1 || vpp > 3 || d.vertex_count == 0 || d.vertex_count % vpp != 0 ||
        d.vertex_va == 0 || !aligned(d.vertex_va, 16) ||
        static_cast<uint64_t>(d.vertex_count) * stride > d.vertex_bytes ||
        !valid_framebuffer(fb) || !valid_programs(p, d.primitive) ||
        (d.depth_copy && (!fb.depth.present || fb.color.kind != TargetKind::depth_copy ||
                          d.primitive != Primitive::triangles)) ||
        (!d.depth_copy && fb.color.kind != TargetKind::bgra8) ||
        (p.key.front_and_back != 0) != fb.dual || (d.depth_copy && fb.dual) ||
        (abi::uses_ps_constants(p.key) &&
         (d.ps_constants_va == 0 || !aligned(d.ps_constants_va, 16) ||
          d.ps_constants_bytes != abi::ps_constants_bytes)) ||
        ((abi::pixel_source_depth(p.key.pixel_source) || abi::pixel_source_stencil(p.key.pixel_source)) &&
         !fb.depth.present) ||
        s.viewport_width > max_dimension * 2u || s.viewport_height > max_dimension * 2u ||
        s.logic_op_code > 15) {
        b.fail(StateStatus::invalid_argument);
        return b.status();
    }
    emit_viewport(b, fb, s, d.primitive) && emit_rasterizer(b, s, d.primitive) &&
        emit_depth_stencil(b, fb, s, p, d) && emit_blend(b, fb, s, p, d) &&
        emit_shaders(b, p, d.primitive) && emit_user_data(b, p, d, rings) &&
        emit_ps_constants(b, s) && emit_draw_packets(b, d.primitive, d.vertex_count);
    return b.status();
}

StateStatus emit_end_of_ib(Pm4Builder& b) noexcept {
    const uint32_t cb_meta[1] = {hw::event_flush_and_inv_cb_meta};
    const uint32_t db_meta[1] = {hw::event_flush_and_inv_db_meta};
    const uint32_t ps_flush[1] = {hw::event_ps_partial_flush | (4u << 8)};
    b.packet(op::event_write, cb_meta, 1) && b.packet(op::event_write, db_meta, 1) &&
        b.packet(op::event_write, ps_flush, 1);
    return b.status();
}

StateStatus emit_row_copy(Pm4Builder& b, const RowCopy& c) noexcept {
    if (!b.ok()) {
        return b.status();
    }
    const uint64_t row_bytes = static_cast<uint64_t>(c.width) * 4u;
    if (c.width == 0 || c.height == 0 || c.source_va == 0 || c.destination_va == 0 ||
        row_bytes > dma_max_bytes || c.source_pitch_bytes < (c.source_x + static_cast<uint64_t>(c.width)) * 4u ||
        c.destination_pitch_bytes < (c.destination_x + static_cast<uint64_t>(c.width)) * 4u ||
        (c.source_va & 3u) != 0 || (c.destination_va & 3u) != 0 ||
        (c.source_pitch_bytes & 3u) != 0 || (c.destination_pitch_bytes & 3u) != 0) {
        b.fail(StateStatus::invalid_argument);
        return b.status();
    }
    for (uint32_t row = 0; row < c.height && b.ok(); ++row) {
        const uint64_t source = c.source_va + (static_cast<uint64_t>(c.source_y) + row) * c.source_pitch_bytes +
                                static_cast<uint64_t>(c.source_x) * 4u;
        const uint64_t destination = c.destination_va +
                                     (static_cast<uint64_t>(c.destination_y) + row) * c.destination_pitch_bytes +
                                     static_cast<uint64_t>(c.destination_x) * 4u;
        // DMA_DATA_WORD0_gfx9: ENGINE ME, SRC_SEL SRC_ADDR_TC_L2 [30:29],
        // DST_SEL DST_ADDR_TC_L2 [21:20], CP_SYNC [31] on the last row so the CP
        // waits for the copy; COMMAND_gfx9 BYTE_COUNT [25:0].
        const bool last = row + 1u == c.height;
        const uint32_t body[6] = {(hw::dma_src_addr_tc_l2 << 29) | (hw::dma_dst_addr_tc_l2 << 20) |
                                      (last ? 1u << 31 : 0u),
                                  low(source), high(source), low(destination), high(destination),
                                  static_cast<uint32_t>(row_bytes)};
        b.packet(op::dma_data, body, 6);
    }
    return b.status();
}

} // namespace rx7900::opengl::gfx11

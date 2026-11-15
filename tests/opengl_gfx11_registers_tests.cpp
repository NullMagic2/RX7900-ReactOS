/*
Module: rx7900.tests.opengl_gfx11_registers
Purpose: Validate every GFX11 register offset, BASE_IDX, field shift and mask the
OpenGL pipeline uses against the pinned AMD gc_11_0_0 headers, every enumerant
against Mesa 24.2.0 gfx11.json/pkt3.json, and decode emitted IBs packet by packet
checking each register write against the header masks and the allow-list.
Created: 2026-10-23
Architecture: Host test over gfx11_state/gfx11_registers and the test-only
gfx11_header_db parser (which re-reads the upstream text and verifies SHA-256).
The catalog is checked by parsing src/opengl/gfx11_registers.hpp itself, so
every transcribed constant is compared, not a hand-picked subset.
*/
#include <cmath>
#include <cstdio>
#include <sstream>
#include <set>

#include "asic_test_device.hpp"
#include "gfx11_header_db.hpp"
#include "../src/opengl/gfx11_state.hpp"

namespace {

using namespace rx7900::opengl;
namespace r = gfx11_reg;
namespace hw = gfx11::hw;
using gfx11_header_db::Database;

rx7900_test::Checks checks;
void expect(bool value, const char* what) { checks.expect(value, what); }

std::string source_dir() {
#ifdef RX7900_SOURCE_DIR
    return RX7900_SOURCE_DIR;
#else
    return ".";
#endif
}

// ---- 1. Catalog transcription ----
void catalog(const Database& db) {
    std::string text;
    expect(gfx11_header_db::read_file(source_dir() + "/src/opengl/gfx11_registers.hpp", text),
           "catalog source is readable");
    unsigned registers = 0, fields = 0, register_errors = 0, field_errors = 0, json_errors = 0;
    std::set<std::string> names;
    // Line parser for the catalog layout: "namespace NAME {", one Register line,
    // Field lines, "} // namespace NAME".
    std::istringstream lines(text);
    std::string line;
    std::string name;
    const gfx11_header_db::Register* header = nullptr;
    std::set<std::string> seen;
    bool in_register = false;
    while (std::getline(lines, line)) {
        char token[128];
        unsigned a = 0, b = 0;
        if (std::sscanf(line.c_str(), "namespace %127[A-Z0-9_] {", token) == 1 && line.back() == '{') {
            name = token;
            in_register = false;
            header = nullptr;
            seen.clear();
        } else if (!name.empty() && std::sscanf(line.c_str(), "constexpr Register reg{0x%x, %u};", &a, &b) == 2) {
            in_register = true;
            names.insert(name);
            ++registers;
            header = db.find(name);
            if (header == nullptr || header->offset != a || header->base_index != b) {
                ++register_errors;
                std::fprintf(stderr, "register mismatch: %s\n", name.c_str());
                header = nullptr;
                continue;
            }
            const int64_t json = db.json_address(name);
            if (json < 0 || static_cast<uint32_t>(json / 4) != header->absolute()) {
                ++json_errors;
                std::fprintf(stderr, "gfx11.json address mismatch: %s\n", name.c_str());
            }
        } else if (in_register && std::sscanf(line.c_str(), "constexpr Field %127[A-Z0-9_]{%u, 0x%x};", token, &a, &b) == 3) {
            ++fields;
            seen.insert(token);
            const auto* field = header != nullptr ? db.field(name, token) : nullptr;
            if (field == nullptr || field->shift != a || field->mask != b) {
                ++field_errors;
                std::fprintf(stderr, "field mismatch: %s.%s\n", name.c_str(), token);
            }
        } else if (in_register && line.rfind("} // namespace ", 0) == 0) {
            if (header != nullptr && seen.size() != header->fields.size()) {
                ++field_errors;
                std::fprintf(stderr, "field set differs: %s\n", name.c_str());
            }
            in_register = false;
            name.clear();
        }
    }
    std::printf("catalog: %u registers, %u fields transcribed\n", registers, fields);
    expect(registers == r::allowed_register_count, "every catalog register is on the allow-list");
    expect(registers >= 220 && fields >= 900, "catalog covers the full GL 1.1 register set");
    expect(register_errors == 0, "every register offset and BASE_IDX equals gc_11_0_0_offset.h");
    expect(field_errors == 0, "every field shift/mask equals gc_11_0_0_sh_mask.h");
    expect(json_errors == 0, "every absolute register address equals Mesa gfx11.json");
    // GFX11 removed VGT_GS_ONCHIP_CNTL; its subgroup sizes are GE_CNTL fields.
    expect(db.find("VGT_GS_ONCHIP_CNTL") == nullptr && db.field("GE_CNTL", "PRIMS_PER_SUBGRP") != nullptr &&
           db.field("GE_CNTL", "VERTS_PER_SUBGRP") != nullptr,
           "GC 11.0.0 has no VGT_GS_ONCHIP_CNTL; NGG subgroup sizes are in GE_CNTL");
    expect(db.find("IA_MULTI_VGT_PARAM") == nullptr, "GC 11.0.0 has no IA_MULTI_VGT_PARAM (GE_CNTL replaces it)");
    for (uint32_t i = 0; i < r::allowed_register_count; ++i) {
        const r::Space space = r::space(r::allowed_registers[i]);
        if (space == r::Space::none) expect(false, "allow-listed register outside every PM4 space");
    }
}

// ---- 2. Enumerants ----
void enumerants(const Database& db) {
    struct Entry { const char* type; const char* name; uint32_t value; bool packet; };
    const Entry entries[] = {
        {"ColorFormat", "COLOR_INVALID", hw::color_invalid, false},
        {"ColorFormat", "COLOR_8_8_8_8", hw::color_8_8_8_8, false},
        {"ColorFormat", "COLOR_X24_8_32_FLOAT", hw::color_x24_8_32_float, false},
        {"SurfaceNumber", "NUMBER_UNORM", hw::number_unorm, false},
        {"SurfaceNumber", "NUMBER_FLOAT", hw::number_float, false},
        {"SurfaceSwap", "SWAP_STD", hw::swap_std, false},
        {"SurfaceSwap", "SWAP_ALT", hw::swap_alt, false},
        {"ZFormat", "Z_INVALID", hw::z_invalid, false},
        {"ZFormat", "Z_32_FLOAT", hw::z_32_float, false},
        {"StencilFormat", "STENCIL_INVALID", hw::stencil_invalid, false},
        {"StencilFormat", "STENCIL_8", hw::stencil_8, false},
        {"StencilOp", "STENCIL_KEEP", hw::stencil_keep, false},
        {"StencilOp", "STENCIL_ZERO", hw::stencil_zero, false},
        {"StencilOp", "STENCIL_REPLACE_TEST", hw::stencil_replace_test, false},
        {"StencilOp", "STENCIL_ADD_CLAMP", hw::stencil_add_clamp, false},
        {"StencilOp", "STENCIL_SUB_CLAMP", hw::stencil_sub_clamp, false},
        {"StencilOp", "STENCIL_INVERT", hw::stencil_invert, false},
        {"BlendOp", "BLEND_ZERO", hw::blend_zero, false},
        {"BlendOp", "BLEND_ONE", hw::blend_one, false},
        {"BlendOp", "BLEND_SRC_COLOR", hw::blend_src_color, false},
        {"BlendOp", "BLEND_ONE_MINUS_SRC_COLOR", hw::blend_one_minus_src_color, false},
        {"BlendOp", "BLEND_SRC_ALPHA", hw::blend_src_alpha, false},
        {"BlendOp", "BLEND_ONE_MINUS_SRC_ALPHA", hw::blend_one_minus_src_alpha, false},
        {"BlendOp", "BLEND_DST_ALPHA", hw::blend_dst_alpha, false},
        {"BlendOp", "BLEND_ONE_MINUS_DST_ALPHA", hw::blend_one_minus_dst_alpha, false},
        {"BlendOp", "BLEND_DST_COLOR", hw::blend_dst_color, false},
        {"BlendOp", "BLEND_ONE_MINUS_DST_COLOR", hw::blend_one_minus_dst_color, false},
        {"BlendOp", "BLEND_SRC_ALPHA_SATURATE", hw::blend_src_alpha_saturate, false},
        {"CombFunc", "COMB_DST_PLUS_SRC", hw::comb_dst_plus_src, false},
        {"CBMode", "CB_DISABLE", hw::cb_disable, false},
        {"CBMode", "CB_NORMAL", hw::cb_normal, false},
        {"ForceControl", "FORCE_DISABLE", hw::force_disable, false},
        {"ZOrder", "LATE_Z", hw::late_z, false},
        {"ZOrder", "EARLY_Z_THEN_LATE_Z", hw::early_z_then_late_z, false},
        {"SX_DOWNCONVERT_FORMAT", "SX_RT_EXPORT_NO_CONVERSION", hw::sx_rt_export_no_conversion, false},
        {"SX_DOWNCONVERT_FORMAT", "SX_RT_EXPORT_32_R", hw::sx_rt_export_32_r, false},
        {"SX_DOWNCONVERT_FORMAT", "SX_RT_EXPORT_8_8_8_8", hw::sx_rt_export_8_8_8_8, false},
        {"SX_OPT_COMB_FCN", "OPT_COMB_NONE", hw::opt_comb_none, false},
        {"SX_OPT_COMB_FCN", "OPT_COMB_BLEND_DISABLED", hw::opt_comb_blend_disabled, false},
        {"SPI_SHADER_EX_FORMAT", "SPI_SHADER_FP16_ABGR", hw::spi_shader_fp16_abgr, false},
        {"SPI_SHADER_EX_FORMAT", "SPI_SHADER_32_ABGR", hw::spi_shader_32_abgr, false},
        {"VGT_DI_PRIM_TYPE", "DI_PT_POINTLIST", hw::di_pt_pointlist, false},
        {"VGT_DI_PRIM_TYPE", "DI_PT_LINELIST", hw::di_pt_linelist, false},
        {"VGT_DI_PRIM_TYPE", "DI_PT_TRILIST", hw::di_pt_trilist, false},
        {"PA_SU_VTX_CNTL__ROUND_MODE", "X_ROUND_TO_EVEN", hw::x_round_to_even, false},
        {"QUANT_MODE", "X_16_8_FIXED_POINT_1_256TH", hw::quant_16_8_1_256th, false},
        {"QUANT_MODE", "X_14_10_FIXED_POINT_1_1024TH", hw::quant_14_10_1_1024th, false},
        {"QUANT_MODE", "X_12_12_FIXED_POINT_1_4096TH", hw::quant_12_12_1_4096th, false},
        {"BinSizeExtend", "BIN_SIZE_128_PIXELS", hw::bin_size_128_pixels, false},
        {"WritePolicy", "CACHE_LRU_WR", hw::cache_lru_wr, false},
        {"WritePolicy", "CACHE_STREAM", hw::cache_stream, false},
        {"ReadPolicy", "CACHE_NOA", hw::cache_noa_rd, false},
        {"VRSCombinerModeSC", "SC_VRS_COMB_MODE_PASSTHRU", hw::vrs_comb_passthru, false},
        {"VRSCombinerModeSC", "SC_VRS_COMB_MODE_OVERRIDE", hw::vrs_comb_override, false},
        {"VGT_EVENT_TYPE", "PS_PARTIAL_FLUSH", hw::event_ps_partial_flush, false},
        {"VGT_EVENT_TYPE", "BOTTOM_OF_PIPE_TS", hw::event_bottom_of_pipe_ts, false},
        {"VGT_EVENT_TYPE", "FLUSH_AND_INV_DB_META", hw::event_flush_and_inv_db_meta, false},
        {"VGT_EVENT_TYPE", "FLUSH_AND_INV_CB_META", hw::event_flush_and_inv_cb_meta, false},
        {"PWS_STAGE_SEL", "CP_ME", hw::pws_stage_cp_me, true},
        {"PWS_COUNTER_SEL", "TS_SELECT", hw::pws_counter_ts, true},
        {"CP_DMA_WORD1__SRC_SEL_cik", "SRC_ADDR_TC_L2", hw::dma_src_addr_tc_l2, true},
        {"CP_DMA_WORD1__DST_SEL_gfx9", "DST_ADDR_TC_L2", hw::dma_dst_addr_tc_l2, true},
    };
    unsigned bad = 0;
    for (const auto& e : entries) {
        if (db.enumerant(e.type, e.name, e.packet) != static_cast<int64_t>(e.value)) {
            ++bad;
            std::fprintf(stderr, "enumerant mismatch: %s.%s\n", e.type, e.name);
        }
    }
    expect(bad == 0, "every hardware enumerant equals Mesa gfx11.json / pkt3.json");
    // BinningMode 2 is Mesa's V_028C44_DISABLE_BINNING_USE_NEW_SC (the GFX10-11 path).
    expect(db.enumerant("BinningMode", "BINNING_ONE_PRIM_PER_BATCH") == hw::binning_disable_new_sc,
           "binning-disabled mode value matches GFX11 BinningMode");
    // Packet word layouts used by emit_preamble / emit_row_copy.
    uint32_t lo = 0, hi = 0;
    expect(db.json_field("RELEASE_MEM_OP_gfx11", "PWS_ENABLE", lo, hi, true) && lo == 31 && hi == 31 &&
           db.json_field("RELEASE_MEM_OP_gfx11", "EVENT_INDEX", lo, hi, true) && lo == 8 && hi == 11,
           "RELEASE_MEM PWS_ENABLE [31] and EVENT_INDEX [11:8]");
    expect(db.json_field("ACQUIRE_MEM_PWS_2", "PWS_STAGE_SEL", lo, hi, true) && lo == 11 && hi == 13 &&
           db.json_field("ACQUIRE_MEM_PWS_2", "PWS_COUNTER_SEL", lo, hi, true) && lo == 14 && hi == 15 &&
           db.json_field("ACQUIRE_MEM_PWS_2", "PWS_ENA2", lo, hi, true) && lo == 17 &&
           db.json_field("ACQUIRE_MEM_PWS_7", "PWS_ENA", lo, hi, true) && lo == 31,
           "ACQUIRE_MEM PWS word layout");
    expect(db.json_field("DMA_DATA_WORD0_gfx9", "SRC_SEL", lo, hi, true) && lo == 29 && hi == 30 &&
           db.json_field("DMA_DATA_WORD0_gfx9", "DST_SEL", lo, hi, true) && lo == 20 && hi == 21 &&
           db.json_field("DMA_DATA_WORD0_gfx9", "CP_SYNC", lo, hi, true) && lo == 31 &&
           db.json_field("COMMAND_gfx9", "BYTE_COUNT", lo, hi, true) && lo == 0 && hi == 25,
           "DMA_DATA word 0 and COMMAND layouts");
}

// ---- 3. PM4 decoding of emitted IBs ----
struct Decoded final {
    unsigned registers = 0;
    unsigned packets = 0;
    unsigned bad = 0;
    unsigned draws = 0;
    std::map<uint32_t, uint32_t> state;   // absolute dword -> last value
};

Decoded decode(const Database& db, const std::vector<uint8_t>& ib, uint32_t dwords) {
    Decoded d{};
    const auto word = [&](uint32_t i) {
        return static_cast<uint32_t>(ib[i * 4]) | (static_cast<uint32_t>(ib[i * 4 + 1]) << 8) |
               (static_cast<uint32_t>(ib[i * 4 + 2]) << 16) | (static_cast<uint32_t>(ib[i * 4 + 3]) << 24);
    };
    for (uint32_t i = 0; i < dwords;) {
        const uint32_t header = word(i);
        if ((header >> 30) != 3) { ++d.bad; break; }
        const uint32_t count = ((header >> 16) & 0x3fff) + 1;
        const uint32_t opcode = (header >> 8) & 0xff;
        ++d.packets;
        if (opcode == gfx11::op::set_context_reg || opcode == gfx11::op::set_sh_reg ||
            opcode == gfx11::op::set_uconfig_reg) {
            const uint32_t base = opcode == gfx11::op::set_context_reg ? 0xa000 :
                                  opcode == gfx11::op::set_sh_reg ? 0x2c00 : 0xc000;
            for (uint32_t k = 1; k < count; ++k) {
                const uint32_t absolute = base + word(i + 1) + (k - 1);
                const uint32_t value = word(i + 1 + k);
                const auto* reg = db.at(absolute);
                ++d.registers;
                const bool in_space = (opcode == gfx11::op::set_context_reg && absolute >= 0xa000 && absolute < 0xc000) ||
                                      (opcode == gfx11::op::set_sh_reg && absolute >= 0x2c00 && absolute < 0x3000) ||
                                      (opcode == gfx11::op::set_uconfig_reg && absolute >= 0xc000);
                bool allowed = false;
                for (uint32_t a = 0; a < r::allowed_register_count; ++a) {
                    allowed = allowed || r::absolute_dword(r::allowed_registers[a]) == absolute;
                }
                // Whole-word registers (floats, addresses) have one full-width field.
                if (reg == nullptr || !in_space || !allowed || (value & ~reg->mask_union()) != 0) {
                    ++d.bad;
                    std::fprintf(stderr, "bad register write 0x%x = 0x%08x (%s)\n", absolute, value,
                                 reg != nullptr ? reg->name.c_str() : "unknown");
                }
                d.state[absolute] = value;
            }
        } else if (opcode == gfx11::op::draw_index_auto) {
            ++d.draws;
        } else if (opcode != gfx11::op::context_control && opcode != gfx11::op::clear_state &&
                   opcode != gfx11::op::num_instances && opcode != gfx11::op::event_write &&
                   opcode != gfx11::op::release_mem && opcode != gfx11::op::acquire_mem &&
                   opcode != gfx11::op::dma_data && opcode != gfx11::op::nop) {
            ++d.bad;
        }
        i += count + 1;
    }
    return d;
}

uint32_t reg_value(const Database& db, const Decoded& d, const char* name) {
    const auto* reg = db.find(name);
    const auto found = reg != nullptr ? d.state.find(reg->absolute()) : d.state.end();
    return found == d.state.end() ? 0xdeadbeefu : found->second;
}

using gfx11::Primitive;

gfx11::ShaderPrograms programs(Primitive primitive_kind, bool textured, bool alpha_test) {
    gfx11::ShaderPrograms p{};
    p.key.vertices_per_primitive = static_cast<uint8_t>(primitive_kind);
    p.key.textured = textured ? 1 : 0;
    p.key.alpha_test = alpha_test ? 1 : 0;
    p.key.alpha_func = alpha_test ? gfx11_abi::compare_greater : gfx11_abi::compare_always;
    p.vs_va = 0x200100000000ULL;
    p.ps_va = 0x200100004000ULL;
    for (auto* info : {&p.vs, &p.ps}) {
        info->code_dwords = 64;
        info->vgpr_count = 16;
        info->float_mode = gfx11_abi::float_mode;
        info->pos_export_count = 1;
        info->param_count = textured ? 2 : 1;
        info->max_es_verts = 32;
        info->max_gs_prims = 32;
    }
    p.vs.user_sgpr_count = gfx11_abi::vs_user_sgpr_count;
    p.ps.user_sgpr_count = gfx11_abi::ps_user_sgpr_count;
    p.vs.gs_vgpr_comp_cnt = gfx11_abi::vs_gs_vgpr_comp_cnt(static_cast<uint32_t>(primitive_kind));
    p.ps.spi_ps_input_ena = gfx11_abi::spi_ps_input_persp_center;
    p.ps.interpolant_count = textured ? 2 : 1;
    p.ps.col_format = gfx11_abi::spi_shader_col_format;
    p.ps.kills = alpha_test ? 1 : 0;
    return p;
}

void decoder(const Database& db) {
    std::vector<uint8_t> ib(128 * 1024);
    static gfx11::Pm4Builder builder;
    builder.begin(rx7900::sdma::CommandBuffer(ib.data(), ib.size()));
    const gfx11::DeviceRings rings{0x200100000000ULL, gfx11::attribute_ring_per_se_bytes_l3,
                                   gfx11::navi31_shader_engines};
    gfx11::Framebuffer fb{};
    fb.width = 640;
    fb.height = 480;
    fb.color = {0x10000000ULL, gfx11::linear_pitch_pixels(640, gfx11::TargetKind::bgra8), gfx11::TargetKind::bgra8};
    gfx11::DepthLayout layout{};
    gfx11::depth_layout(640, 480, layout);
    fb.depth = {true, 0x20000000ULL, 0x20000000ULL + layout.stencil_offset};
    expect(gfx11::emit_preamble(builder, rings) == gfx11::StateStatus::success, "preamble emits");
    expect(gfx11::emit_framebuffer(builder, fb) == gfx11::StateStatus::success, "framebuffer emits");
    unsigned draws = 0;
    for (uint32_t variant = 0; variant < 48; ++variant) {
        gfx11::RasterState s{};
        s.viewport_x = static_cast<int32_t>(variant * 3) - 20;
        s.viewport_y = 7;
        s.viewport_width = 600 - variant;
        s.viewport_height = 400;
        s.depth_near = variant & 1 ? 1.0f : 0.0f;
        s.depth_far = variant & 1 ? 0.25f : 1.0f;
        s.scissor = (variant & 2) != 0;
        s.scissor_x = 10;
        s.scissor_y = 20;
        s.scissor_width = 300;
        s.scissor_height = 200;
        s.cull = static_cast<gfx11::CullMode>(variant & 3);
        s.front_ccw = (variant & 4) != 0;
        s.point_size = 1.0f + static_cast<float>(variant);
        s.line_width = 0.5f + static_cast<float>(variant) * 0.25f;
        s.offset = (variant & 8) != 0;
        s.offset_factor = 1.5f;
        s.offset_units = 2.0f;
        s.depth_test = (variant & 1) != 0;
        s.depth_func = static_cast<gfx11::Compare>(variant & 7);
        s.stencil_test = (variant & 16) != 0;
        s.stencil_func = static_cast<gfx11::Compare>((variant >> 1) & 7);
        s.stencil_ref = variant * 5;
        s.stencil_fail = static_cast<gfx11::StencilOp>(variant % 6);
        s.stencil_depth_fail = static_cast<gfx11::StencilOp>((variant + 1) % 6);
        s.stencil_depth_pass = static_cast<gfx11::StencilOp>((variant + 2) % 6);
        s.blend = (variant & 32) != 0;
        s.blend_src = static_cast<gfx11::BlendFactor>(variant % 11);
        s.blend_dst = static_cast<gfx11::BlendFactor>((variant * 7) % 11);
        s.logic_op = variant % 5 == 0;
        s.logic_op_code = variant % 16;
        s.color_mask = variant % 16;
        s.alpha_ref = 0.5f;
        const Primitive kind = static_cast<Primitive>(1 + variant % 3);
        const auto p = programs(kind, (variant & 4) != 0, (variant & 8) != 0);
        gfx11::DrawInputs d{};
        d.primitive = kind;
        d.vertex_count = static_cast<uint32_t>(kind) * 4;
        d.vertex_va = 0x200102000000ULL;
        d.vertex_bytes = d.vertex_count * gfx11_abi::clip_record_stride;
        d.texture = gfx11_abi::texture_descriptor(0x30000000ULL, 64, 32, 64);
        d.sampler = gfx11_abi::sampler_descriptor(0, 2, 1);
        if (gfx11::emit_draw(builder, fb, s, p, d, rings) == gfx11::StateStatus::success) ++draws;
    }
    expect(draws == 48, "48 state variants (every primitive, cull, compare, stencil op, blend factor, logic op) emit");
    // A depth/stencil copy pass into an X24_8_32 target.
    gfx11::Framebuffer copy = fb;
    copy.color = {0x40000000ULL, gfx11::linear_pitch_pixels(640, gfx11::TargetKind::depth_copy),
                  gfx11::TargetKind::depth_copy};
    gfx11::emit_framebuffer(builder, copy);
    gfx11::DrawInputs dc{};
    dc.primitive = Primitive::triangles;
    dc.vertex_count = 6;
    dc.vertex_va = 0x200102000000ULL;
    dc.vertex_bytes = 6 * gfx11_abi::clip_record_stride;
    dc.depth_copy = true;
    expect(gfx11::emit_draw(builder, copy, gfx11::RasterState{}, programs(Primitive::triangles, false, false), dc,
                            rings) == gfx11::StateStatus::success, "DB->CB depth copy pass emits");
    gfx11::RowCopy rows{0x10000000ULL, 2560, 3, 4, 0x50000000ULL, 7680, 100, 200, 64, 32};
    expect(gfx11::emit_row_copy(builder, rows) == gfx11::StateStatus::success, "present rows emit");
    expect(gfx11::emit_end_of_ib(builder) == gfx11::StateStatus::success && builder.ok(), "IB completes");
    const Decoded decoded = decode(db, ib, builder.dwords());
    std::printf("decoder: %u packets, %u register writes, %u draws in %u dwords\n", decoded.packets,
                decoded.registers, decoded.draws, builder.dwords());
    expect(decoded.bad == 0, "every emitted register exists in GC 11.0.0, is allow-listed, sits in its "
                             "PM4 space and sets no bit outside the header field masks");
    expect(decoded.draws == 49 && decoded.registers > 600, "decoder walked every draw and register write");
    expect(reg_value(db, decoded, "SPI_ATTRIBUTE_RING_BASE") == static_cast<uint32_t>(rings.attribute_ring_va >> 16) &&
           db.get("SPI_ATTRIBUTE_RING_SIZE", "MEM_SIZE", reg_value(db, decoded, "SPI_ATTRIBUTE_RING_SIZE")) ==
               static_cast<int64_t>(rings.attribute_ring_per_se_bytes / 65536 - 1) &&
           db.get("SPI_ATTRIBUTE_RING_SIZE", "L1_POLICY", reg_value(db, decoded, "SPI_ATTRIBUTE_RING_SIZE")) == 1,
           "attribute ring BASE (64 KiB units) and per-SE MEM_SIZE as Mesa programs them");
    std::set<uint32_t> distinct;
    for (const auto& entry : decoded.state) distinct.insert(entry.first);
    std::printf("decoder: %zu distinct registers programmed\n", distinct.size());
    expect(distinct.size() >= 180, "the IB programs the full context/SH/UCONFIG register set");

    // Semantic spot checks by header field names (independent of the catalog).
    std::vector<uint8_t> one(16384);
    builder.begin(rx7900::sdma::CommandBuffer(one.data(), one.size()));
    gfx11::emit_framebuffer(builder, fb);
    gfx11::RasterState s{};
    s.viewport_width = 640;
    s.viewport_height = 480;
    s.cull = gfx11::CullMode::back;
    s.front_ccw = true;
    s.depth_test = true;
    s.depth_func = gfx11::Compare::lequal;
    s.stencil_test = true;
    s.stencil_func = gfx11::Compare::equal;
    s.stencil_ref = 0x5a;
    s.stencil_depth_pass = gfx11::StencilOp::increment;
    s.blend = true;
    s.blend_src = gfx11::BlendFactor::src_alpha;
    s.blend_dst = gfx11::BlendFactor::one_minus_src_alpha;
    s.color_mask = 0x7;
    gfx11::DrawInputs d{};
    d.primitive = Primitive::triangles;
    d.vertex_count = 3;
    d.vertex_va = 0x200102000000ULL;
    d.vertex_bytes = 3 * gfx11_abi::clip_record_stride;
    gfx11::emit_draw(builder, fb, s, programs(Primitive::triangles, false, false), d, rings);
    const Decoded single = decode(db, one, builder.dwords());
    const uint32_t info = reg_value(db, single, "CB_COLOR0_INFO");
    expect(db.get("CB_COLOR0_INFO", "FORMAT", info) == db.enumerant("ColorFormat", "COLOR_8_8_8_8") &&
           db.get("CB_COLOR0_INFO", "COMP_SWAP", info) == db.enumerant("SurfaceSwap", "SWAP_ALT") &&
           db.get("CB_COLOR0_INFO", "NUMBER_TYPE", info) == db.enumerant("SurfaceNumber", "NUMBER_UNORM"),
           "colour target is B8G8R8A8_UNORM (COLOR_8_8_8_8, SWAP_ALT, UNORM)");
    expect(db.get("CB_COLOR0_ATTRIB2", "MIP0_WIDTH", reg_value(db, single, "CB_COLOR0_ATTRIB2")) == 639 &&
           db.get("CB_COLOR0_ATTRIB3", "COLOR_SW_MODE", reg_value(db, single, "CB_COLOR0_ATTRIB3")) == 0,
           "linear colour target with the 256-byte custom pitch in MIP0_WIDTH");
    const uint32_t zinfo = reg_value(db, single, "DB_Z_INFO");
    expect(db.get("DB_Z_INFO", "FORMAT", zinfo) == db.enumerant("ZFormat", "Z_32_FLOAT") &&
           db.get("DB_Z_INFO", "SW_MODE", zinfo) == 24 && db.get("DB_Z_INFO", "TILE_SURFACE_ENABLE", zinfo) == 0,
           "Z32_FLOAT depth, SW_64KB_Z_X, HTILE off");
    const uint32_t dctl = reg_value(db, single, "DB_DEPTH_CONTROL");
    expect(db.get("DB_DEPTH_CONTROL", "Z_ENABLE", dctl) == 1 && db.get("DB_DEPTH_CONTROL", "ZFUNC", dctl) ==
           db.enumerant("CompareFrag", "FRAG_LEQUAL") && db.get("DB_DEPTH_CONTROL", "STENCILFUNC", dctl) ==
           db.enumerant("CompareFrag", "FRAG_EQUAL"), "DB_DEPTH_CONTROL depth/stencil functions");
    expect(db.get("DB_STENCIL_CONTROL", "STENCILZPASS", reg_value(db, single, "DB_STENCIL_CONTROL")) ==
           db.enumerant("StencilOp", "STENCIL_ADD_CLAMP") &&
           db.get("DB_STENCILREFMASK", "STENCILTESTVAL", reg_value(db, single, "DB_STENCILREFMASK")) == 0x5a,
           "stencil GL_INCR maps to ADD_CLAMP with the reference value");
    const uint32_t bc = reg_value(db, single, "CB_BLEND0_CONTROL");
    expect(db.get("CB_BLEND0_CONTROL", "ENABLE", bc) == 1 &&
           db.get("CB_BLEND0_CONTROL", "COLOR_SRCBLEND", bc) == db.enumerant("BlendOp", "BLEND_SRC_ALPHA") &&
           db.get("CB_BLEND0_CONTROL", "COLOR_DESTBLEND", bc) == db.enumerant("BlendOp", "BLEND_ONE_MINUS_SRC_ALPHA"),
           "blend factors map to BlendOp");
    expect(db.get("CB_TARGET_MASK", "TARGET0_ENABLE", reg_value(db, single, "CB_TARGET_MASK")) == 7,
           "colour mask to CB_TARGET_MASK");
    const uint32_t su = reg_value(db, single, "PA_SU_SC_MODE_CNTL");
    expect(db.get("PA_SU_SC_MODE_CNTL", "CULL_BACK", su) == 1 && db.get("PA_SU_SC_MODE_CNTL", "CULL_FRONT", su) == 0 &&
           db.get("PA_SU_SC_MODE_CNTL", "FACE", su) == 1 &&
           db.get("PA_SU_SC_MODE_CNTL", "PROVOKING_VTX_LAST", su) == 1,
           "back-face cull, GL CCW front under the y-inverting viewport (FACE=1), provoking last");
    const uint32_t ys = reg_value(db, single, "PA_CL_VPORT_YSCALE");
    float yscale = 0;
    std::memcpy(&yscale, &ys, 4);
    expect(yscale == -240.0f, "viewport inverts y for top-down rows");
    expect(reg_value(db, single, "GE_CNTL") == gfx11_abi::ge_cntl &&
           reg_value(db, single, "VGT_SHADER_STAGES_EN") == gfx11_abi::vgt_shader_stages_en,
           "NGG GE_CNTL / VGT_SHADER_STAGES_EN follow the gl_gfx11_abi contract");
    expect(db.get("VGT_PRIMITIVE_TYPE", "PRIM_TYPE", reg_value(db, single, "VGT_PRIMITIVE_TYPE")) ==
           db.enumerant("VGT_DI_PRIM_TYPE", "DI_PT_TRILIST"), "VGT_PRIMITIVE_TYPE via UCONFIG");
}

// ---- 4. Builder refusals ----
const Database* database_for_refusals = nullptr;

void refusals() {
    std::vector<uint8_t> ib(4096);
    static gfx11::Pm4Builder b;
    b.begin(rx7900::sdma::CommandBuffer(ib.data(), ib.size()));
    expect(!b.set({0x0f00, 0}, 1) && b.status() == gfx11::StateStatus::disallowed_register,
           "a register outside the allow-list is refused");
    b.begin(rx7900::sdma::CommandBuffer(ib.data(), ib.size()));
    expect(!b.set({0x1000, 1}, 1) && b.status() == gfx11::StateStatus::disallowed_register,
           "a context-space register not in the catalog is refused");
    // A real context-space register inside the builder window that the catalog does
    // not list (taken from the header) is refused too.
    {
        const auto* outside = static_cast<const gfx11_header_db::Register*>(nullptr);
        for (const auto& entry : database_for_refusals->registers()) {
            const auto& reg = entry.second;
            if (reg.base_index != 1 || reg.offset >= 0x400) continue;
            bool listed = false;
            for (uint32_t a = 0; a < r::allowed_register_count; ++a)
                listed = listed || (r::allowed_registers[a].offset == reg.offset && r::allowed_registers[a].base_index == 1);
            if (!listed) { outside = &reg; break; }
        }
        b.begin(rx7900::sdma::CommandBuffer(ib.data(), ib.size()));
        expect(outside != nullptr && !b.set({outside->offset, 1}, 1) &&
               b.status() == gfx11::StateStatus::disallowed_register,
               "a GC 11.0.0 context register outside the catalog is refused by the allow-list");
    }
    b.begin(rx7900::sdma::CommandBuffer(ib.data(), ib.size()));
    const uint32_t body[1] = {0};
    expect(!b.packet(gfx11::op::set_context_reg, body, 1) && b.status() == gfx11::StateStatus::invalid_argument,
           "raw SET_CONTEXT_REG packets cannot bypass set()");
    b.begin(rx7900::sdma::CommandBuffer(ib.data(), ib.size()));
    gfx11::RegisterValue overflow;
    overflow.with(r::DB_STENCILREFMASK::STENCILTESTVAL, 256);
    expect(!overflow.valid(), "field overflow is detected before emission");
    b.begin(rx7900::sdma::CommandBuffer(ib.data(), 8));
    expect(!b.set(r::DB_RENDER_CONTROL::reg, 0) && b.status() == gfx11::StateStatus::output_too_small &&
           b.dwords() == 0, "a full IB refuses without partial packets");
    b.begin(rx7900::sdma::CommandBuffer(ib.data(), ib.size()));
    expect(b.set(r::DB_RENDER_CONTROL::reg, 5) && b.set(r::DB_RENDER_CONTROL::reg, 5) && b.dwords() == 3,
           "an unchanged register value is not re-emitted");
    gfx11::Framebuffer fb{};
    fb.width = 64;
    fb.height = 64;
    fb.color = {0x1001, 64, gfx11::TargetKind::bgra8};
    b.begin(rx7900::sdma::CommandBuffer(ib.data(), ib.size()));
    expect(gfx11::emit_framebuffer(b, fb) == gfx11::StateStatus::invalid_argument, "unaligned colour base refused");
    fb.color = {0x1000, 32, gfx11::TargetKind::bgra8};
    b.begin(rx7900::sdma::CommandBuffer(ib.data(), ib.size()));
    expect(gfx11::emit_framebuffer(b, fb) == gfx11::StateStatus::invalid_argument, "wrong linear pitch refused");
    fb.color = {0x1000, 64, gfx11::TargetKind::bgra8};
    fb.depth = {true, 0x10000, 0x10000};
    b.begin(rx7900::sdma::CommandBuffer(ib.data(), ib.size()));
    expect(gfx11::emit_framebuffer(b, fb) == gfx11::StateStatus::invalid_argument,
           "stencil plane must follow the Z plane at the 64 KiB layout offset");
    expect(gfx11::logic_op_rop3(3) == 0xcc && gfx11::logic_op_rop3(6) == 0x66 && gfx11::logic_op_rop3(10) == 0x55,
           "GL logic ops map to ROP3 truth tables (copy 0xCC, xor 0x66, invert 0x55)");
    expect(gfx11::pack_12p4(0.5f) == 8 && gfx11::pack_12p4(5000.0f) == 0xffff && gfx11::pack_12p4(-1.0f) == 0,
           "12.4 fixed-point packing saturates");
    gfx11::DepthLayout layout{};
    expect(gfx11::depth_layout(129, 257, layout) && layout.z_bytes == 2u * 3u * 65536u &&
           layout.total_bytes == layout.z_bytes + 1u * 2u * 65536u,
           "SW_64KB_Z_X plane sizes are whole 128x128 (32bpp) and 256x256 (8bpp) blocks");
}

} // namespace

int main() {
    Database db;
    expect(db.load(), "pinned GC 11.0.0 headers load");
    expect(db.pinned_ok(), "gc_11_0_0_offset.h, gc_11_0_0_sh_mask.h, gfx11.json and pkt3.json match SHA256SUMS");
    catalog(db);
    enumerants(db);
    decoder(db);
    database_for_refusals = &db;
    refusals();
    return checks.finish("opengl_gfx11_registers");
}

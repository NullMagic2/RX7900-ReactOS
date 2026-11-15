/*
Module: rx7900.tests.opengl_shader
Purpose: Contract checks for the GFX11 fixed-function shader generator and the
graphics parts of the independent interpreter: key validation, ProgramInfo, binary
layout, determinism and worst-case size, the clip-space key mapping, and focused
interpreter semantics (GS_ALLOC_REQ capture, attribute-ring swizzled addressing,
s_buffer_load bounds, MIMG descriptor decoding, lds_param_load/v_interp quad
lanes, s_wqm_b32, float compares).
Created: 2026-10-23
Architecture: Generates programs with gl_shader_gfx11 and runs hand-assembled
snippets on tests/gfx11_interp.hpp, which shares no tables with the encoder or with
gl_gfx11_abi.hpp. Whole-draw pixel comparisons live in opengl_shader_pixel_tests.cpp.
Host only; no GPU.
*/
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include "gfx11_interp.hpp"
#include "../src/opengl/gl_shader_gfx11.hpp"

namespace {

using namespace rx7900::opengl;
using namespace rx7900::hardware::gfx11_isa;
namespace abi = rx7900::opengl::gfx11_abi;
namespace sdma = rx7900::sdma;

unsigned checks = 0;
unsigned failures = 0;

void expect(bool condition, const char* contract) {
    ++checks;
    if (!condition) {
        ++failures;
        std::fprintf(stderr, "FAIL: %s\n", contract);
    }
}

uint32_t bits(float value) {
    uint32_t out = 0;
    std::memcpy(&out, &value, 4);
    return out;
}

float as_float(uint32_t value) {
    float out = 0;
    std::memcpy(&out, &value, 4);
    return out;
}

abi::ShaderKey worst_case_key() {
    abi::ShaderKey key{};
    key.vertex_input = abi::vertex_input_object_space;
    key.lighting = 1;
    key.light_mask = 0xff;
    key.positional_mask = 0xff;
    key.spot_mask = 0xff;
    key.normalize = 1;
    key.two_sided = 1;
    key.color_material = abi::color_material_ambient_and_diffuse;
    key.textured = 1;
    key.tex_env = abi::tex_env_blend;
    key.fog = 1;
    key.fog_mode = abi::fog_exp2;
    key.alpha_test = 1;
    key.alpha_func = abi::compare_gequal;
    key.smooth = 0;
    return key;
}

void test_key_validation() {
    abi::ShaderKey key{};
    expect(abi::key_valid(key), "default key is valid");
    key.vertices_per_primitive = 0;
    expect(!abi::key_valid(key), "zero vertices per primitive rejected");
    key = {};
    key.reserved[1] = 1;
    expect(!abi::key_valid(key), "nonzero reserved byte rejected");
    key = {};
    key.lighting = 1;
    expect(!abi::key_valid(key), "lighting on a clip-space key rejected");
    key = {};
    key.vertex_input = abi::vertex_input_object_space;
    key.lighting = 1;
    key.light_mask = 1;
    key.spot_mask = 2;
    expect(!abi::key_valid(key), "spot bit for a disabled light rejected");
    key.spot_mask = 1;
    expect(abi::key_valid(key), "consistent object-space lit key accepted");
    key.alpha_func = 8;
    expect(!abi::key_valid(key), "out-of-range alpha function rejected");
    expect(abi::key_valid(worst_case_key()), "worst-case key is valid");

    uint8_t code[64] = {};
    abi::ProgramInfo info{};
    expect(shader::generate_vertex_shader(key, sdma::CommandBuffer(code, sizeof(code)), info) ==
               shader::Status::invalid_argument && info.code_dwords == 0,
           "generator rejects an invalid key and clears info");
}

void test_program_info() {
    std::vector<uint8_t> code(abi::shader_max_bytes);
    abi::ProgramInfo vs{}, ps{};
    abi::ShaderKey key{};
    key.textured = 1;
    key.fog = 1;
    key.two_sided = 1;
    key.alpha_test = 1;
    key.alpha_func = abi::compare_less;
    key.vertices_per_primitive = 2;

    expect(shader::generate_vertex_shader(key, sdma::CommandBuffer(code.data(), code.size()), vs) ==
               shader::Status::success, "clip-space vertex program generates");
    expect(shader::generate_pixel_shader(key, sdma::CommandBuffer(code.data(), code.size()), ps) ==
               shader::Status::success, "pixel program generates");
    expect(vs.code_dwords * 4u % abi::shader_code_alignment == 0 && ps.code_dwords * 4u % abi::shader_code_alignment == 0,
           "programs are 256-byte padded");
    expect(vs.user_sgpr_count == abi::vs_user_sgpr_count && ps.user_sgpr_count == abi::ps_user_sgpr_count,
           "user SGPR counts match the ABI slot tables");
    expect(vs.param_count == 4 && ps.interpolant_count == 4, "front, back, texcoord and fog slots");
    expect(vs.gs_vgpr_comp_cnt == 0 && vs.es_vgpr_comp_cnt == 0, "line key needs VGPR0 only");
    expect(vs.pos_export_count == 1 && vs.writes_attribute_ring == 1 && vs.scratch_bytes == 0 && vs.lds_bytes == 0,
           "one position export, attribute ring, no scratch or LDS");
    expect(vs.max_es_verts == 32 && vs.max_gs_prims == 32, "single-wave NGG subgroups");
    expect(ps.spi_ps_input_ena == (abi::spi_ps_input_persp_center | abi::spi_ps_input_front_face) &&
               ps.reads_front_face == 1, "two-sided pixel program enables PERSP_CENTER and FRONT_FACE");
    expect(ps.kills == 1 && ps.col_format == 9 && ps.z_format == 0, "alpha test kills; MRT0 32_ABGR");
    expect(vs.vgpr_count <= 256 && ps.vgpr_count <= 256 && vs.sgpr_count <= 106 && ps.sgpr_count <= 106 &&
               vs.float_mode == abi::float_mode, "register counts within wave32 limits");

    key.smooth = 0;
    expect(shader::generate_pixel_shader(key, sdma::CommandBuffer(code.data(), code.size()), ps) ==
               shader::Status::success && ps.flat_mask == 0x3u, "flat key marks both colour slots FLAT_SHADE");

    key = {};
    expect(shader::generate_pixel_shader(key, sdma::CommandBuffer(code.data(), code.size()), ps) ==
               shader::Status::success && ps.kills == 0 && ps.spi_ps_input_ena == abi::spi_ps_input_persp_center &&
               ps.interpolant_count == 1, "plain pixel program: one slot, no kill");
}

void test_determinism_and_size() {
    std::vector<uint8_t> a(abi::shader_max_bytes), b(abi::shader_max_bytes);
    abi::ProgramInfo ia{}, ib{};
    const abi::ShaderKey key = worst_case_key();
    const shader::Status sa = shader::generate_vertex_shader(key, sdma::CommandBuffer(a.data(), a.size()), ia);
    const shader::Status sb = shader::generate_vertex_shader(key, sdma::CommandBuffer(b.data(), b.size()), ib);
    expect(sa == shader::Status::success && sb == shader::Status::success,
           "worst-case vertex program fits shader_max_bytes");
    expect(ia.code_dwords == ib.code_dwords && std::memcmp(a.data(), b.data(), ia.code_dwords * 4u) == 0,
           "equal keys give byte-identical programs");
    std::printf("worst-case vertex program: %u bytes (limit %u)\n", ia.code_dwords * 4u, abi::shader_max_bytes);

    // At least shader_tail_padding_min bytes of s_code_end after s_endpgm.
    uint32_t last_endpgm = 0;
    for (uint32_t i = 0; i < ia.code_dwords; ++i) {
        uint32_t w = 0;
        std::memcpy(&w, a.data() + i * 4u, 4);
        if (w == 0xbfb00000u) {
            last_endpgm = i;
        }
    }
    expect((ia.code_dwords - last_endpgm - 1) * 4u >= abi::shader_tail_padding_min, "s_code_end tail padding");

    abi::ProgramInfo ps{};
    expect(shader::generate_pixel_shader(key, sdma::CommandBuffer(a.data(), a.size()), ps) == shader::Status::success,
           "worst-case pixel program generates");

    uint8_t small[256] = {};
    expect(shader::generate_vertex_shader(key, sdma::CommandBuffer(small, sizeof(small)), ia) ==
               shader::Status::output_too_small && ia.code_dwords == 0, "short output reports output_too_small");

    // An explicit workspace reused after a failed and a different generation gives
    // the same bytes as a fresh one.
    shader::GeneratorWorkspace workspace;
    std::vector<uint8_t> c(abi::shader_max_bytes);
    abi::ProgramInfo ic{};
    expect(shader::generate_vertex_shader(key, sdma::CommandBuffer(small, sizeof(small)), ic, workspace) ==
               shader::Status::output_too_small, "workspace generation reports output_too_small");
    expect(shader::generate_pixel_shader(key, sdma::CommandBuffer(c.data(), c.size()), ic, workspace) ==
               shader::Status::success, "workspace reused for a pixel program");
    expect(shader::generate_vertex_shader(key, sdma::CommandBuffer(c.data(), c.size()), ic, workspace) ==
               shader::Status::success && ic.code_dwords == ib.code_dwords &&
               std::memcmp(c.data(), b.data(), ic.code_dwords * 4u) == 0,
           "reused workspace output is byte-identical");
    std::printf("generator workspace: %zu bytes\n", shader::generator_workspace_bytes);
}

void test_clip_space_key() {
    uint8_t texels[4] = {1, 2, 3, 4};
    Texture texture{};
    texture.levels[0].width = 1;
    texture.levels[0].height = 1;
    texture.levels[0].base_format = gl::luminance_alpha;
    texture.levels[0].texels = texels;
    DrawState state{};
    state.texture = &texture;
    state.texture_env_mode = gl::decal;
    state.fog_enabled = true;
    state.alpha_test = true;
    state.alpha_func = gl::notequal;
    const abi::ShaderKey key = shader::clip_space_key(state, PrimitiveKind::points);
    expect(abi::key_valid(key) && key.vertices_per_primitive == 1 && key.textured == 1 &&
               key.tex_env == abi::tex_env_decal && key.texture_base == abi::texture_base_luminance_alpha &&
               key.fog == 1 && key.alpha_test == 1 && key.alpha_func == abi::compare_notequal &&
               key.vertex_input == abi::vertex_input_clip_space && key.two_sided == 0,
           "clip_space_key maps DrawState to the ABI key");
    DrawState two_sided{};
    two_sided.two_sided_color = true;
    expect(shader::clip_space_key(two_sided, PrimitiveKind::triangles).two_sided == 1 &&
               abi::key_valid(shader::clip_space_key(two_sided, PrimitiveKind::triangles)),
           "two_sided_color maps to a two-sided clip-space key");
    expect(shader::clip_space_key(DrawState{}, PrimitiveKind::triangles).vertices_per_primitive == 3 &&
               shader::clip_space_key(DrawState{}, PrimitiveKind::lines).vertices_per_primitive == 2,
           "primitive kind selects vertices per primitive");
}

// Assembles a snippet into code; returns its dword count (0 on failure).
template <typename Emit>
uint32_t assemble(std::vector<uint8_t>& code, Emit emit) {
    code.assign(1024, 0);
    Assembler a(sdma::CommandBuffer(code.data(), code.size()));
    emit(a);
    a.endpgm();
    return a.finish() ? a.dwords() : 0u;
}

void test_interpreter_graphics() {
    std::vector<uint8_t> code;

    // GS_ALLOC_REQ capture with m0, and s_wqm_b32.
    uint32_t n = assemble(code, [](Assembler& a) {
        a.sop1(Sop1::s_mov_b32, m0(), literal(0x00003005u));
        a.sendmsg(msg_gs_alloc_req);
        a.sop1(Sop1::s_wqm_b32, s(1), s(0));
    });
    gfx11_interp::Wave w0;
    w0.sgpr[0] = 0x00100201u;
    gfx11_interp::run(code.data(), n, w0);
    expect(w0.message_count == 1 && w0.messages[0].id == 9 && w0.messages[0].m0 == 0x3005u,
           "s_sendmsg GS_ALLOC_REQ captured with m0");
    expect(w0.sgpr[1] == 0x00f00f0fu, "s_wqm_b32 sets whole quads");

    // Attribute-ring store: swizzled 16-byte elements, index stride 32, per-wave
    // soffset; must land at slot * 512 + lane * 16 (the ABI per-wave layout).
    gfx11_interp::Buffer ring{0x40000, std::vector<uint8_t>(4096, 0)};
    gfx11_interp::Memory memory{};
    memory.buffers = {&ring};
    n = assemble(code, [](Assembler& a) {
        MubufFlags idxen{};
        idxen.idxen = true;
        a.mubuf_ex(Mubuf::buffer_store_b128, v(4), v(0), 8, s(12), 16, idxen);
    });
    gfx11_interp::Wave w1;
    const abi::BufferDescriptor rd = abi::attribute_ring_descriptor(0x40000, 4096, 2);
    for (uint32_t k = 0; k < 4; ++k) { w1.sgpr[8 + k] = rd.words[k]; }
    w1.sgpr[12] = 1024;                             // wave offset (2 * 512)
    w1.exec = 0x0000000fu;
    for (uint32_t lane = 0; lane < 4; ++lane) {
        w1.vgpr[0][lane] = lane;
        for (uint32_t c = 0; c < 4; ++c) { w1.vgpr[4 + c][lane] = 0x1000u * lane + c + 1; }
    }
    gfx11_interp::run(code.data(), n, w1, &memory);
    bool ring_ok = true;
    for (uint32_t lane = 0; lane < 4; ++lane) {
        for (uint32_t c = 0; c < 4; ++c) {
            uint32_t value = 0;
            std::memcpy(&value, ring.bytes.data() + 1024 + 512 + lane * 16 + c * 4, 4);
            ring_ok = ring_ok && value == 0x1000u * lane + c + 1;
        }
    }
    expect(ring_ok && w1.unsupported == 0, "attribute-ring store follows the swizzled per-wave layout");

    // s_buffer_load honours NUM_RECORDS.
    gfx11_interp::Buffer constants{0x50000, std::vector<uint8_t>(64, 0)};
    for (uint32_t i = 0; i < 16; ++i) { std::memcpy(constants.bytes.data() + i * 4, &i, 4); }
    memory.buffers = {&constants};
    n = assemble(code, [](Assembler& a) { a.smem(Smem::s_buffer_load_b512, 20, 16, null_sgpr(), 16); });
    gfx11_interp::Wave w2;
    const abi::BufferDescriptor cd = abi::raw_buffer_descriptor(0x50000, 40);
    for (uint32_t k = 0; k < 4; ++k) { w2.sgpr[16 + k] = cd.words[k]; }
    for (uint32_t k = 0; k < 16; ++k) { w2.sgpr[20 + k] = 0xdeadu; }
    gfx11_interp::run(code.data(), n, w2, &memory);
    expect(w2.sgpr[20] == 4 && w2.sgpr[25] == 9 && w2.sgpr[26] == 0 && w2.sgpr[35] == 0,
           "s_buffer_load returns in-range dwords and zero past NUM_RECORDS");

    // MIMG: 2x2 linear RGBA8 texture, point and bilinear samples via T#/S#.
    gfx11_interp::Buffer texture{0x60000, std::vector<uint8_t>(64 * 2 * 4, 0)};
    const uint8_t texel[4][4] = {{255, 0, 0, 255}, {0, 255, 0, 255}, {0, 0, 255, 255}, {255, 255, 255, 0}};
    std::memcpy(texture.bytes.data(), texel[0], 8);                // row 0: texels 0, 1
    std::memcpy(texture.bytes.data() + 64 * 4, texel[2], 8);       // row 1: texels 2, 3
    memory.buffers = {&texture};
    n = assemble(code, [](Assembler& a) {
        a.mimg_sample(Mimg::image_sample_lz, v(4), v(0), 0, 8, 0xf, mimg_dim_2d);
    });
    for (uint32_t filter = 0; filter < 2; ++filter) {
        gfx11_interp::Wave w3;
        const abi::ImageDescriptor t = abi::texture_descriptor(0x60000, 2, 2, 64);
        const abi::SamplerDescriptor sd = abi::sampler_descriptor(abi::tex_clamp_last_texel,
                                                                  abi::tex_clamp_last_texel, filter);
        for (uint32_t k = 0; k < 8; ++k) { w3.sgpr[k] = t.words[k]; }
        for (uint32_t k = 0; k < 4; ++k) { w3.sgpr[8 + k] = sd.words[k]; }
        w3.exec = 1;
        w3.vgpr[0][0] = bits(0.75f);
        w3.vgpr[1][0] = bits(0.25f);
        gfx11_interp::run(code.data(), n, w3, &memory);
        const bool ok = filter == 0
                            ? (as_float(w3.vgpr[4][0]) == 0.0f && as_float(w3.vgpr[5][0]) == 1.0f &&
                               as_float(w3.vgpr[6][0]) == 0.0f && w3.unsupported == 0)
                            : (std::fabs(as_float(w3.vgpr[4][0]) - 0.0f) < 1e-6f &&
                               std::fabs(as_float(w3.vgpr[5][0]) - 1.0f) < 1e-6f && w3.unsupported == 0);
        expect(ok, filter == 0 ? "image_sample_lz point sample decodes T#/S#"
                               : "image_sample_lz bilinear at a texel centre returns that texel");
    }

    // lds_param_load + v_interp_p10/p2: lanes of a quad get P0/P10/P20; m0 gates.
    gfx11_interp::ParamLds lds{};
    lds.prim_mask = 0x8123u;
    lds.attributes = 1;
    lds.data.assign(2 * 4 * 3, 0);
    const float p[3] = {0.25f, 0.5f, -0.125f};
    for (uint32_t q = 0; q < 2; ++q) {
        for (uint32_t t = 0; t < 3; ++t) { lds.data[(q * 4 + 1) * 3 + t] = bits(p[t] * static_cast<float>(q + 1)); }
    }
    n = assemble(code, [](Assembler& a) {
        a.sop1(Sop1::s_mov_b32, m0(), s(0));
        a.lds_param_load(v(5), 0, 1, 0);
        a.vinterp(Vinterp::v_interp_p10_f32, v(6), v(5), v(0), v(5), 0);
        a.vinterp(Vinterp::v_interp_p2_f32, v(7), v(5), v(1), v(6), 0);
    });
    for (uint32_t correct = 0; correct < 2; ++correct) {
        gfx11_interp::Wave w4;
        w4.sgpr[0] = correct != 0 ? 0x8123u : 0x8124u;
        w4.lds = &lds;
        w4.exec = 0xffu;
        for (uint32_t lane = 0; lane < 8; ++lane) {
            w4.vgpr[0][lane] = bits(0.5f);
            w4.vgpr[1][lane] = bits(0.25f);
        }
        gfx11_interp::run(code.data(), n, w4, &memory);
        bool ok = true;
        for (uint32_t lane = 0; lane < 8; ++lane) {
            const float k = static_cast<float>(lane / 4 + 1);
            const float expected = std::fma(p[2] * k, 0.25f, std::fma(p[1] * k, 0.5f, p[0] * k));
            ok = ok && as_float(w4.vgpr[7][lane]) == expected;
        }
        expect(correct != 0 ? ok : !ok, correct != 0 ? "lds_param_load + v_interp_p10/p2 interpolate per quad"
                                                     : "lds_param_load with a wrong m0 yields no parameter data");
    }

    // Float compares (VOP3): lt, eq, neq with NaN.
    n = assemble(code, [](Assembler& a) {
        a.vop3_cmp(Vopc::v_cmp_lt_f32, s(2), v(0), s(10));
        a.vop3_cmp(Vopc::v_cmp_eq_f32, s(3), v(0), s(10));
        a.vop3_cmp(Vopc::v_cmp_neq_f32, s(4), v(0), s(10));
    });
    gfx11_interp::Wave w5;
    w5.exec = 0x7u;
    w5.sgpr[10] = bits(0.5f);
    w5.vgpr[0][0] = bits(0.25f);
    w5.vgpr[0][1] = bits(0.5f);
    w5.vgpr[0][2] = 0x7fc00000u;
    gfx11_interp::run(code.data(), n, w5);
    expect(w5.sgpr[2] == 0x1u && w5.sgpr[3] == 0x2u && w5.sgpr[4] == 0x5u,
           "v_cmp_lt/eq/neq_f32 lane masks (neq true for NaN)");
}

} // namespace

int main() {
    test_key_validation();
    test_program_info();
    test_determinism_and_size();
    test_clip_space_key();
    test_interpreter_graphics();
    std::printf("opengl_shader: %u checks, %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

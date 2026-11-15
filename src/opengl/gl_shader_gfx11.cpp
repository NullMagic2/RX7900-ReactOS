/*
Module: rx7900.opengl.gl_shader_gfx11
Purpose: Emit the GFX11 NGG vertex and pixel shader instruction streams for the
OpenGL 1.1 fixed-function pipeline.
Created: 2026-10-23
Architecture: See gl_shader_gfx11.hpp and gl_gfx11_abi.hpp. Uses the gfx11_isa
Assembler (kept in the caller's GeneratorWorkspace) exclusively. Register assignment is static per stage (documented below);
arithmetic mirrors gl_context transform/apply_lighting and gl_raster
shade_fragment operation by operation so the interpreter pixel tests compare like
with like. Freestanding: no heap, no floating-point library.
*/
#include "gl_shader_gfx11.hpp"

namespace rx7900::opengl::shader {
namespace {

// Out-of-line pieces keep every generator frame within 512 bytes on AMD64 too.
#if defined(_MSC_VER)
#define RX7900_SHADER_NOINLINE __declspec(noinline)
#else
#define RX7900_SHADER_NOINLINE __attribute__((noinline))
#endif

using namespace rx7900::hardware::gfx11_isa;
namespace abi = rx7900::opengl::gfx11_abi;

// ---- Vertex program register plan ----
// SGPRs: s0..s7 system, s8..s20 user data (gl_gfx11_abi), then:
constexpr uint32_t vs_s_saved_exec = 21;
constexpr uint32_t vs_s_vertex_count = 22;
constexpr uint32_t vs_s_prim_count = 23;
constexpr uint32_t vs_s_temp = 24;
constexpr uint32_t vs_s_es_mask = 25;
constexpr uint32_t vs_s_gs_mask = 26;
constexpr uint32_t vs_s_ring_offset = 27;
constexpr uint32_t vs_s_mask_a = 28;
constexpr uint32_t vs_s_mask_b = 29;
constexpr uint32_t vs_s_window = 32;             // constant window s32..s79
constexpr uint32_t vs_sgpr_count = 80;
// VGPRs (v0..v8 are SPI inputs):
constexpr uint32_t vs_v_lane = 9;
constexpr uint32_t vs_v_index = 10;
constexpr uint32_t vs_v_offset = 11;
constexpr uint32_t vs_v_one = 12;
constexpr uint32_t vs_v_prim_a = 13;
constexpr uint32_t vs_v_prim_b = 14;
constexpr uint32_t vs_v_in0 = 16;                // record vec4 inputs 16..35
constexpr uint32_t vs_v_eye = 36;
constexpr uint32_t vs_v_clip = 40;
constexpr uint32_t vs_v_normal = 44;
constexpr uint32_t vs_v_front = 48;
constexpr uint32_t vs_v_back = 52;
constexpr uint32_t vs_v_tex = 56;
constexpr uint32_t vs_v_fog = 60;
constexpr uint32_t vs_v_work = 64;               // lighting/fog temporaries 64..95
constexpr uint32_t vs_vgpr_count = 96;

// Lighting window layout inside s32..s79.
constexpr uint32_t win_material = vs_s_window;          // emission, ambient, diffuse, specular, shininess
constexpr uint32_t win_light_model = vs_s_window + 20;  // light model ambient
constexpr uint32_t win_light = vs_s_window + 24;        // one LightConstants (24 dwords)

// ---- Pixel program register plan ----
// v3 keys keep the original static plan (their programs stay byte-identical);
// keys with v4 features move the work registers above the extra SPI inputs.
struct PsPlan final {
    // SPI inputs.
    uint32_t persp_i, persp_j, linear_i, linear_j, pos_x, pos_y, front_face;
    // Work VGPRs.
    uint32_t zero, one, param, temp, color, back, tex, fog, coord, sample, work;
    uint32_t sample_b, lod, level_a, level_b, weight, texel, scratch, vgpr_count;
    // SGPRs.
    uint32_t prim_mask, live, pass, front, keep, temp_s, level, level_offset, magnify, minify, quads;
    uint32_t sgpr_count;
};

// SGPR window of the PS constant header (s_buffer_load) and one level's T#.
constexpr uint32_t ps_s_constants = 40;
constexpr uint32_t ps_s_level_texture = 80;

uint32_t constant_sgpr(uint32_t dword) noexcept { return ps_s_constants + dword; }

PsPlan ps_plan(const abi::ShaderKey& key) noexcept {
    PsPlan p{};
    const uint32_t ena = abi::ps_input_ena(key);
    p.persp_i = abi::ps_input_vgpr(ena, abi::spi_ps_input_persp_center);
    p.persp_j = p.persp_i + 1;
    p.linear_i = abi::ps_input_vgpr(ena, abi::spi_ps_input_linear_center);
    p.linear_j = p.linear_i + 1;
    p.pos_x = abi::ps_input_vgpr(ena, abi::spi_ps_input_pos_x);
    p.pos_y = abi::ps_input_vgpr(ena, abi::spi_ps_input_pos_y);
    p.front_face = abi::ps_input_vgpr(ena, abi::spi_ps_input_front_face);

    if (!abi::uses_ps_constants(key)) {
        p.zero = 3; p.one = 4; p.param = 5; p.temp = 6; p.color = 8; p.back = 12; p.tex = 16;
        p.fog = 20; p.coord = 22; p.sample = 24; p.work = 28; p.vgpr_count = 32;
        p.prim_mask = abi::ps_sgpr_prim_mask;
        p.live = 21; p.pass = 22; p.front = 23; p.sgpr_count = 24;
        return p;
    }

    p.zero = 8; p.one = 9; p.param = 10; p.temp = 11; p.color = 12; p.back = 16; p.tex = 20;
    p.fog = 24; p.coord = 28; p.sample = 32; p.work = 36; p.sample_b = 40; p.lod = 44; p.level_a = 45;
    p.level_b = 46; p.weight = 47; p.texel = 48; p.scratch = 52; p.vgpr_count = 64;
    p.prim_mask = abi::ps_user_sgpr_count_v4;
    p.live = 25; p.pass = 26; p.front = 27; p.keep = 28; p.temp_s = 29; p.level = 30; p.level_offset = 31;
    p.magnify = 32; p.minify = 33; p.quads = 34;
    p.sgpr_count = ps_s_level_texture + 8;
    return p;
}

constexpr uint32_t literal_abs_mask = 0x7fffffffu;
constexpr uint32_t literal_sign_mask = 0x80000000u;

bool is_vgpr(const Operand& op) noexcept { return op.kind == OperandKind::vgpr; }

// Float arithmetic helpers over an Assembler. Each helper keeps the operation
// order its caller requests (one rounding per call), choosing the VOP2 operand
// placement the encoding allows; a pair of non-VGPR sources is first moved.
class Emitter final {
public:
    explicit Emitter(Assembler& assembler) noexcept : a(assembler) {}

    void mov(Operand d, Operand x) noexcept { a.vop1(Vop1::v_mov_b32, d, x); }

    void mul(Operand d, Operand x, Operand y) noexcept { commutative(Vop2::v_mul_f32, d, x, y); }
    void add(Operand d, Operand x, Operand y) noexcept { commutative(Vop2::v_add_f32, d, x, y); }

    // d = x - y.
    RX7900_SHADER_NOINLINE void sub(Operand d, Operand x, Operand y) noexcept {
        if (is_vgpr(y)) {
            a.vop2(Vop2::v_sub_f32, d, x, y);
        } else if (is_vgpr(x)) {
            a.vop2(Vop2::v_subrev_f32, d, y, x);
        } else {
            mov(d, x);
            a.vop2(Vop2::v_subrev_f32, d, y, d);
        }
    }

    // d = clamp(x, 0, 1); one holds 1.0f.
    void clamp01(Operand d, Operand x, Operand one) noexcept {
        a.vop2(Vop2::v_max_f32, d, imm(0), x);
        a.vop2(Vop2::v_min_f32, d, one, d);
    }

    void max0(Operand d, Operand x) noexcept { a.vop2(Vop2::v_max_f32, d, imm(0), x); }

    // d = mask ? when_true : when_false (wave32 lane mask in an SGPR).
    void select(Operand d, Operand mask, Operand when_true, Operand when_false) noexcept {
        a.vop3(Vop3::v_cndmask_b32_e64, d, when_false, when_true, mask);
    }

    void compare(Vopc op, Operand mask, Operand x, Operand y) noexcept { a.vop3_cmp(op, mask, x, y); }

    void unary(Vop1 op, Operand d, Operand x) noexcept { a.vop1(op, d, x); }

    // d = (x0*y0 + x1*y1) + x2*y2, rounding after every operation; t is scratch.
    RX7900_SHADER_NOINLINE void dot3(Operand d, Operand t, const Operand x[3], const Operand y[3]) noexcept {
        mul(d, x[0], y[0]);
        mul(t, x[1], y[1]);
        add(d, d, t);
        mul(t, x[2], y[2]);
        add(d, d, t);
    }

    Assembler& a;

private:
    RX7900_SHADER_NOINLINE void commutative(Vop2 op, Operand d, Operand x, Operand y) noexcept {
        if (is_vgpr(y)) {
            a.vop2(op, d, x, y);
        } else if (is_vgpr(x)) {
            a.vop2(op, d, y, x);
        } else {
            mov(d, x);
            a.vop2(op, d, y, d);
        }
    }
};

Status finish(Assembler& assembler, abi::ProgramInfo& info) noexcept {
    // At least the ABI tail of s_code_end, then pad to the code alignment.
    for (uint32_t i = 0; i < abi::shader_tail_padding_min / 4u; ++i) {
        assembler.sopp(Sopp::s_code_end, 0);
    }

    assembler.pad_code_end(abi::shader_code_alignment);

    if (!assembler.finish()) {
        info = {};
        return Status::output_too_small;
    }

    info.code_dwords = assembler.dwords();
    return Status::success;
}

// ---------------------------------------------------------------------------
// Vertex program pieces.
// ---------------------------------------------------------------------------

// Loads dwords (4, 8 or 16) of VertexConstants at dword_offset into s[sdata..].
void load_constants(Assembler& a, uint32_t sdata, uint32_t dword_offset, uint32_t dwords) noexcept {
    const Smem op = dwords == 16 ? Smem::s_buffer_load_b512
                    : dwords == 8 ? Smem::s_buffer_load_b256
                                  : Smem::s_buffer_load_b128;
    a.smem(op, sdata, abi::vs_sgpr_constants, null_sgpr(), static_cast<int32_t>(dword_offset * 4u));
}

void wait_constants(Assembler& a) noexcept { a.waitcnt(waitcnt_max_vm, waitcnt_max_exp, 0); }

// out[r] = ((m[r]*x + m[4+r]*y) + m[8+r]*z) + m[12+r]*w, matrix column-major in
// s[matrix..matrix+15], as gl_pipeline matrix_transform.
void transform(Emitter& e, uint32_t out, uint32_t matrix, uint32_t in, uint32_t temp) noexcept {
    for (uint32_t r = 0; r < 4; ++r) {
        e.mul(v(out + r), s(matrix + r), v(in));
        for (uint32_t c = 1; c < 4; ++c) {
            e.mul(v(temp), s(matrix + c * 4 + r), v(in + c));
            e.add(v(out + r), v(out + r), v(temp));
        }
    }
}

// Normalizes v[base..base+2] in place when its length is positive (gl_context
// divides by the length; here x * (1 / length)).
RX7900_SHADER_NOINLINE void normalize3(Emitter& e, uint32_t base, uint32_t length, uint32_t temp,
                                       uint32_t mask) noexcept {
    const Operand x[3] = {v(base), v(base + 1), v(base + 2)};
    e.dot3(v(length), v(temp), x, x);
    e.unary(Vop1::v_sqrt_f32, v(length), v(length));
    e.compare(Vopc::v_cmp_gt_f32, s(mask), v(length), imm(0));
    e.unary(Vop1::v_rcp_f32, v(length), v(length));

    for (uint32_t c = 0; c < 3; ++c) {
        e.mul(v(temp), v(base + c), v(length));
        e.select(v(base + c), s(mask), v(temp), v(base + c));
    }
}

// d = pow(base, exponent) for base >= 0, as std::pow: exp2(exponent * log2(base))
// when base > 0, else (exponent == 0 ? 1 : 0). exponent is an SGPR.
RX7900_SHADER_NOINLINE void power(Emitter& e, uint32_t d, uint32_t base, Operand exponent, uint32_t temp,
                                  uint32_t mask_a, uint32_t mask_b) noexcept {
    e.unary(Vop1::v_log_f32, v(temp), v(base));
    e.mul(v(temp), exponent, v(temp));
    e.unary(Vop1::v_exp_f32, v(temp), v(temp));
    e.compare(Vopc::v_cmp_gt_f32, s(mask_a), v(base), imm(0));
    e.compare(Vopc::v_cmp_eq_f32, s(mask_b), exponent, imm(0));
    e.select(v(d), s(mask_b), v(vs_v_one), imm(0));
    e.select(v(d), s(mask_a), v(temp), v(d));
}

// Work registers of light_face shared with light_one.
struct LightRegisters final {
    uint32_t n, ve, dir, att, t0, t1, ndotl, spot, half, spec, len, mask_a, mask_b;
};

// One enabled light's ambient, diffuse and specular terms added to v[out..].
RX7900_SHADER_NOINLINE void light_one(Emitter& e, const abi::ShaderKey& key, uint32_t light, uint32_t out,
                                      const LightRegisters& r, const Operand* ambient_material,
                                      const Operand* diffuse_material) noexcept {
    Assembler& a = e.a;
    const uint32_t n = r.n, ve = r.ve, dir = r.dir, att = r.att, t0 = r.t0, t1 = r.t1, ndotl = r.ndotl;
    const uint32_t spot = r.spot, half = r.half, spec = r.spec, len = r.len, mask_a = r.mask_a, mask_b = r.mask_b;
    const uint32_t base = abi::vc_lights + light * abi::vc_light_dwords;
    const uint32_t l_ambient = win_light;
    const uint32_t l_diffuse = win_light + 4;
    const uint32_t l_specular = win_light + 8;
    const uint32_t l_position = win_light + 12;
    const uint32_t l_spot = win_light + 16;
    const uint32_t l_atten = win_light + 20;
    load_constants(a, win_light, base, 16);
    load_constants(a, win_light + 16, base + 16, 8);
    wait_constants(a);

    if (((key.positional_mask >> light) & 1u) == 0) {
        for (uint32_t c = 0; c < 3; ++c) {
            e.mov(v(dir + c), s(l_position + c));
        }

        e.mov(v(att), v(vs_v_one));
    } else {
        for (uint32_t c = 0; c < 3; ++c) {
            e.sub(v(dir + c), s(l_position + c), v(ve + c));
        }

        // attenuation = 1 / ((c + l*d) + (q*d)*d), d = |direction|.
        const Operand d3[3] = {v(dir), v(dir + 1), v(dir + 2)};
        e.dot3(v(len), v(t0), d3, d3);
        e.unary(Vop1::v_sqrt_f32, v(len), v(len));
        e.mul(v(t0), s(l_atten + 1), v(len));
        e.add(v(att), s(l_atten), v(t0));
        e.mul(v(t1), s(l_atten + 2), v(len));
        e.mul(v(t1), v(t1), v(len));
        e.add(v(att), v(att), v(t1));
        e.unary(Vop1::v_rcp_f32, v(att), v(att));
    }

    normalize3(e, dir, len, t0, mask_a);

    if (((key.spot_mask >> light) & 1u) != 0) {
        // cosine = -(direction . spot_direction); spot = cosine < cos(cutoff) ? 0
        // : pow(max(cosine, 0), exponent).
        const Operand d3[3] = {v(dir), v(dir + 1), v(dir + 2)};
        const Operand sd[3] = {s(l_spot), s(l_spot + 1), s(l_spot + 2)};
        e.dot3(v(t1), v(t0), d3, sd);
        a.vop2(Vop2::v_xor_b32, v(t1), literal(literal_sign_mask), v(t1));
        e.max0(v(spot), v(t1));
        power(e, spot, spot, s(l_spot + 3), t0, mask_a, mask_b);
        e.compare(Vopc::v_cmp_lt_f32, s(mask_a), v(t1), s(l_atten + 3));
        e.select(v(spot), s(mask_a), imm(0), v(spot));
        e.mul(v(att), v(att), v(spot));
    }

    // Ambient: result += (factor * light.ambient) * ambient_material.
    for (uint32_t c = 0; c < 3; ++c) {
        e.mul(v(t0), v(att), s(l_ambient + c));
        e.mul(v(t0), v(t0), ambient_material[c]);
        e.add(v(out + c), v(out + c), v(t0));
    }

    // Diffuse: ndotl = max(n . direction, 0).
    const Operand n3[3] = {v(n), v(n + 1), v(n + 2)};
    const Operand d3[3] = {v(dir), v(dir + 1), v(dir + 2)};
    e.dot3(v(ndotl), v(t0), n3, d3);
    e.max0(v(ndotl), v(ndotl));

    for (uint32_t c = 0; c < 3; ++c) {
        e.mul(v(t0), v(att), v(ndotl));
        e.mul(v(t0), v(t0), s(l_diffuse + c));
        e.mul(v(t0), v(t0), diffuse_material[c]);
        e.add(v(out + c), v(out + c), v(t0));
    }

    // Specular (only when ndotl > 0): half = normalize(direction + (0, 0, 1)).
    e.mov(v(half), v(dir));
    e.mov(v(half + 1), v(dir + 1));
    e.add(v(half + 2), v(dir + 2), v(vs_v_one));
    normalize3(e, half, len, t0, mask_a);
    const Operand h3[3] = {v(half), v(half + 1), v(half + 2)};
    e.dot3(v(spec), v(t0), n3, h3);
    e.max0(v(spec), v(spec));
    power(e, spec, spec, s(win_material + 16), t0, mask_a, mask_b);
    e.compare(Vopc::v_cmp_gt_f32, s(mask_a), v(ndotl), imm(0));
    e.select(v(spec), s(mask_a), v(spec), imm(0));

    for (uint32_t c = 0; c < 3; ++c) {
        e.mul(v(t0), v(att), v(spec));
        e.mul(v(t0), v(t0), s(l_specular + c));
        e.mul(v(t0), v(t0), s(win_material + 12 + c));
        e.add(v(out + c), v(out + c), v(t0));
    }
}

// gl_context apply_lighting for one face into v[out..out+3]. The material for the
// face is loaded into the window first; color holds the vertex colour.
void light_face(Emitter& e, const abi::ShaderKey& key, bool back, uint32_t out, uint32_t color) noexcept {
    Assembler& a = e.a;
    const uint32_t material = back ? abi::vc_back_material : abi::vc_front_material;
    // Work registers.
    const uint32_t n = vs_v_work;            // normal 64..66
    const uint32_t ve = vs_v_work + 4;       // vertex eye position 68..70
    const uint32_t dir = vs_v_work + 8;      // light direction 72..74
    const uint32_t att = vs_v_work + 11;     // attenuation / factor 75
    const uint32_t t0 = vs_v_work + 12;      // 76
    const uint32_t t1 = vs_v_work + 13;      // 77
    const uint32_t ndotl = vs_v_work + 14;   // 78
    const uint32_t spot = vs_v_work + 15;    // 79
    const uint32_t half = vs_v_work + 16;    // 80..82
    const uint32_t spec = vs_v_work + 19;    // 83
    const uint32_t len = vs_v_work + 20;     // 84
    const uint32_t ew = vs_v_work + 21;      // 85
    const uint32_t mask_a = vs_s_mask_a;
    const uint32_t mask_b = vs_s_mask_b;

    load_constants(a, win_material, material, 16);
    load_constants(a, win_material + 16, material + 16, 4);
    load_constants(a, win_light_model, abi::vc_light_model_ambient, 4);
    wait_constants(a);

    const bool ambient_from_color = key.color_material == abi::color_material_ambient ||
                                    key.color_material == abi::color_material_ambient_and_diffuse;
    const bool diffuse_from_color = key.color_material == abi::color_material_diffuse ||
                                    key.color_material == abi::color_material_ambient_and_diffuse;
    Operand ambient_material[4];
    Operand diffuse_material[4];

    for (uint32_t c = 0; c < 4; ++c) {
        ambient_material[c] = ambient_from_color ? v(color + c) : s(win_material + 4 + c);
        diffuse_material[c] = diffuse_from_color ? v(color + c) : s(win_material + 8 + c);
    }

    // n = normalize(eye normal), negated for the back face.
    for (uint32_t c = 0; c < 3; ++c) {
        if (back) {
            a.vop2(Vop2::v_xor_b32, v(n + c), literal(literal_sign_mask), v(vs_v_normal + c));
        } else {
            e.mov(v(n + c), v(vs_v_normal + c));
        }
    }

    normalize3(e, n, len, t0, mask_a);

    // result = emission + light_model_ambient * ambient_material.
    for (uint32_t c = 0; c < 3; ++c) {
        e.mul(v(t0), s(win_light_model + c), ambient_material[c]);
        e.add(v(out + c), s(win_material + c), v(t0));
    }

    // Vertex eye position, eye.xyz / (eye.w != 0 ? eye.w : 1).
    e.compare(Vopc::v_cmp_neq_f32, s(mask_a), v(vs_v_eye + 3), imm(0));
    e.select(v(ew), s(mask_a), v(vs_v_eye + 3), v(vs_v_one));
    e.unary(Vop1::v_rcp_f32, v(ew), v(ew));

    for (uint32_t c = 0; c < 3; ++c) {
        e.mul(v(ve + c), v(vs_v_eye + c), v(ew));
    }

    const LightRegisters r{n, ve, dir, att, t0, t1, ndotl, spot, half, spec, len, mask_a, mask_b};
    for (uint32_t light = 0; light < abi::max_lights; ++light) {
        if (((key.light_mask >> light) & 1u) != 0) {
            light_one(e, key, light, out, r, ambient_material, diffuse_material);
        }
    }

    // alpha = diffuse_material.a; clamp every channel to [0, 1].
    e.mov(v(out + 3), diffuse_material[3]);

    for (uint32_t c = 0; c < 4; ++c) {
        e.clamp01(v(out + c), v(out + c), v(vs_v_one));
    }
}

// gl_context transform fog factor into v[vs_v_fog] (y, z, w zero).
void object_fog(Emitter& e, const abi::ShaderKey& key) noexcept {
    Assembler& a = e.a;
    const uint32_t z = vs_v_work;
    const uint32_t t = vs_v_work + 1;
    const uint32_t f = vs_v_fog;
    const uint32_t scale = vs_s_window;
    const uint32_t bias = vs_s_window + 1;
    const uint32_t density = vs_s_window + 2;
    const uint32_t exp_scale = vs_s_window + 3;

    load_constants(a, vs_s_window, abi::vc_fog, 4);
    wait_constants(a);
    // z = |eye.w != 0 ? eye.z / eye.w : eye.z|.
    e.compare(Vopc::v_cmp_neq_f32, s(vs_s_mask_a), v(vs_v_eye + 3), imm(0));
    e.select(v(t), s(vs_s_mask_a), v(vs_v_eye + 3), v(vs_v_one));
    e.unary(Vop1::v_rcp_f32, v(t), v(t));
    e.mul(v(z), v(vs_v_eye + 2), v(t));
    a.vop2(Vop2::v_and_b32, v(z), literal(literal_abs_mask), v(z));

    if (key.fog_mode == abi::fog_linear) {
        e.mul(v(t), s(scale), v(z));
        e.add(v(f), s(bias), v(t));
    } else if (key.fog_mode == abi::fog_exp) {
        e.mul(v(t), s(exp_scale), v(z));
        e.unary(Vop1::v_exp_f32, v(f), v(t));
    } else {
        e.mul(v(t), s(density), v(z));
        e.mul(v(t), v(t), v(t));
        e.mul(v(t), s(exp_scale), v(t));
        e.unary(Vop1::v_exp_f32, v(f), v(t));
    }

    e.clamp01(v(f), v(f), v(vs_v_one));

    for (uint32_t c = 1; c < 4; ++c) {
        e.mov(v(f + c), imm(0));
    }
}

// Vertex-lane body: fetch, compute and store the attribute slots; returns the
// register base of the clip-space position.
uint32_t vertex_body(Emitter& e, const abi::ShaderKey& key, const abi::AttributeLayout& layout) noexcept {
    Assembler& a = e.a;
    const bool object = key.vertex_input == abi::vertex_input_object_space;
    MubufFlags offen{};
    offen.offen = true;

    // Record byte offset: (vertex_id + base_vertex) * stride.
    a.vop2(Vop2::v_add_nc_u32, v(vs_v_index), s(abi::vs_sgpr_base_vertex), v(abi::vs_vgpr_vertex_id));
    a.vop3(Vop3::v_mul_lo_u32, v(vs_v_offset), v(vs_v_index), literal(abi::record_stride(key)), Operand{});
    a.vop1(Vop1::v_cvt_f32_u32, v(vs_v_one), imm(1));

    // Fetch the record's vec4 fields into v16.. (offset / 4 apart).
    const uint32_t fields = abi::record_stride(key) / 16u;

    for (uint32_t field = 0; field < fields; ++field) {
        const bool needed = object ? (field != 1 || key.lighting != 0) && (field != 3 || key.textured != 0)
                                   : (field != 2 || key.two_sided != 0) && (field != 3 || key.textured != 0) &&
                                         (field != 4 || abi::needs_misc_slot(key));

        if (needed) {
            a.mubuf_ex(Mubuf::buffer_load_b128, v(vs_v_in0 + field * 4), v(vs_v_offset), abi::vs_sgpr_vertex_buffer,
                       imm(0), field * 16u, offen);
        }
    }

    a.waitcnt(0, waitcnt_max_exp, waitcnt_max_lgkm);

    uint32_t position = 0;
    uint32_t front = 0;
    uint32_t back = 0;
    uint32_t texcoord = 0;
    uint32_t fog = 0;

    if (!object) {
        position = vs_v_in0 + abi::clip_record_position / 4u;
        front = vs_v_in0 + abi::clip_record_front_color / 4u;
        back = vs_v_in0 + abi::clip_record_back_color / 4u;
        texcoord = vs_v_in0 + abi::clip_record_texcoord / 4u;
        fog = vs_v_in0 + abi::clip_record_fog / 4u;
    } else {
        const uint32_t in_position = vs_v_in0 + abi::object_record_position / 4u;
        const uint32_t in_normal = vs_v_in0 + abi::object_record_normal / 4u;
        const uint32_t in_color = vs_v_in0 + abi::object_record_color / 4u;
        const uint32_t in_texcoord = vs_v_in0 + abi::object_record_texcoord / 4u;

        // eye = modelview * object; clip = projection * eye.
        load_constants(a, vs_s_window, abi::vc_modelview, 16);
        load_constants(a, vs_s_window + 16, abi::vc_projection, 16);
        wait_constants(a);
        transform(e, vs_v_eye, vs_s_window, in_position, vs_v_work);
        transform(e, vs_v_clip, vs_s_window + 16, vs_v_eye, vs_v_work);
        position = vs_v_clip;

        if (key.lighting != 0) {
            // Eye normal = rows of the inverse modelview (transpose(inverse) * n).
            load_constants(a, vs_s_window, abi::vc_normal_matrix, 8);
            load_constants(a, vs_s_window + 8, abi::vc_normal_matrix + 8, 4);
            wait_constants(a);

            for (uint32_t r = 0; r < 3; ++r) {
                const Operand row[3] = {s(vs_s_window + r * 4), s(vs_s_window + r * 4 + 1),
                                        s(vs_s_window + r * 4 + 2)};
                const Operand normal[3] = {v(in_normal), v(in_normal + 1), v(in_normal + 2)};
                e.dot3(v(vs_v_normal + r), v(vs_v_work), row, normal);
            }

            if (key.normalize != 0) {
                normalize3(e, vs_v_normal, vs_v_work, vs_v_work + 1, vs_s_mask_a);
            }

            light_face(e, key, false, vs_v_front, in_color);
            front = vs_v_front;

            if (key.two_sided != 0) {
                light_face(e, key, true, vs_v_back, in_color);
                back = vs_v_back;
            }
        } else {
            front = in_color;
        }

        if (key.textured != 0) {
            load_constants(a, vs_s_window, abi::vc_texture_matrix, 16);
            wait_constants(a);
            transform(e, vs_v_tex, vs_s_window, in_texcoord, vs_v_work);
            texcoord = vs_v_tex;
        }

        if (key.fog != 0) {
            object_fog(e, key);
            fog = vs_v_fog;
        }
    }

    // Attribute-ring stores: index = lane, soffset = wave ring offset, slot * 16.
    MubufFlags idxen{};
    idxen.idxen = true;
    const uint32_t sources[4] = {front, back, texcoord, fog};
    const uint32_t slots[4] = {layout.front_color, layout.back_color, layout.texcoord, layout.fog};

    for (uint32_t i = 0; i < 4; ++i) {
        if (slots[i] != abi::slot_none) {
            a.mubuf_ex(Mubuf::buffer_store_b128, v(sources[i]), v(vs_v_lane), abi::vs_sgpr_attribute_ring,
                       s(vs_s_ring_offset), slots[i] * 16u, idxen);
        }
    }

    return position;
}

// ---------------------------------------------------------------------------
// Pixel program pieces.
// ---------------------------------------------------------------------------

// Interpolates channels of a slot into v[dst..]: lds_param_load then
// v_interp_p10/p2 at (i, j), or at (0, 0) for a flat slot (P0, the provoking value).
RX7900_SHADER_NOINLINE void interpolate(Assembler& a, const PsPlan& p, uint32_t slot, uint32_t dst, uint32_t channels,
                                        bool flat) noexcept {
    const Operand i = flat ? v(p.zero) : v(p.persp_i);
    const Operand j = flat ? v(p.zero) : v(p.persp_j);

    for (uint32_t c = 0; c < channels; ++c) {
        a.lds_param_load(v(p.param), slot, c, 0);
        a.vinterp(Vinterp::v_interp_p10_f32, v(p.temp), v(p.param), i, v(p.param), 0);
        a.vinterp(Vinterp::v_interp_p2_f32, v(dst + c), v(p.param), j, v(p.temp), 0);
    }
}

// One channel of a slot at the given barycentric VGPRs.
void interpolate_channel(Assembler& a, const PsPlan& p, uint32_t slot, uint32_t channel, uint32_t dst,
                         uint32_t i, uint32_t j) noexcept {
    a.lds_param_load(v(p.param), slot, channel, 0);
    a.vinterp(Vinterp::v_interp_p10_f32, v(p.temp), v(p.param), v(i), v(p.param), 0);
    a.vinterp(Vinterp::v_interp_p2_f32, v(dst), v(p.param), v(j), v(p.temp), 0);
}

bool base_has_alpha(uint8_t base) noexcept {
    return base == abi::texture_base_rgba || base == abi::texture_base_alpha ||
           base == abi::texture_base_luminance_alpha;
}

// gl_raster shade_fragment texture environment on v[p.color..] with the
// sample in v[p.sample..].
void texture_environment(Emitter& e, const abi::ShaderKey& key, const PsPlan& p) noexcept {
    const uint32_t c = p.color;
    const uint32_t t = p.sample;
    const uint32_t w0 = p.work;
    const uint32_t w1 = p.work + 1;
    const Operand one = v(p.one);

    switch (key.tex_env) {
    case abi::tex_env_replace:
        if (key.texture_base == abi::texture_base_alpha) {
            e.mov(v(c + 3), v(t + 3));
        } else if (key.texture_base == abi::texture_base_luminance || key.texture_base == abi::texture_base_rgb) {
            for (uint32_t k = 0; k < 3; ++k) {
                e.mov(v(c + k), v(t + k));
            }
        } else {
            for (uint32_t k = 0; k < 4; ++k) {
                e.mov(v(c + k), v(t + k));
            }
        }
        break;
    case abi::tex_env_decal:
        if (key.texture_base == abi::texture_base_rgba) {
            // c = c * (1 - a) + t * a.
            e.sub(v(w0), one, v(t + 3));
            for (uint32_t k = 0; k < 3; ++k) {
                e.mul(v(w1), v(c + k), v(w0));
                e.mul(v(c + k), v(t + k), v(t + 3));
                e.add(v(c + k), v(w1), v(c + k));
            }
        } else {
            for (uint32_t k = 0; k < 3; ++k) {
                e.mov(v(c + k), v(t + k));
            }
        }
        break;
    case abi::tex_env_blend:
        // c = c * (1 - t) + env * t; alpha modulates when the base has alpha.
        for (uint32_t k = 0; k < 3; ++k) {
            e.sub(v(w0), one, v(t + k));
            e.mul(v(w0), v(c + k), v(w0));
            e.mul(v(w1), s(abi::ps_sgpr_env_color + k), v(t + k));
            e.add(v(c + k), v(w0), v(w1));
        }
        if (base_has_alpha(key.texture_base)) {
            e.mul(v(c + 3), v(c + 3), v(t + 3));
        }
        break;
    case abi::tex_env_modulate:
    default:
        for (uint32_t k = 0; k < 3; ++k) {
            e.mul(v(c + k), v(c + k), v(t + k));
        }
        if (base_has_alpha(key.texture_base)) {
            e.mul(v(c + 3), v(c + 3), v(t + 3));
        }
        break;
    }
}

Vopc alpha_compare(uint8_t func) noexcept {
    switch (func) {
    case abi::compare_less: return Vopc::v_cmp_lt_f32;
    case abi::compare_equal: return Vopc::v_cmp_eq_f32;
    case abi::compare_lequal: return Vopc::v_cmp_le_f32;
    case abi::compare_greater: return Vopc::v_cmp_gt_f32;
    case abi::compare_notequal: return Vopc::v_cmp_neq_f32;
    case abi::compare_gequal:
    default: return Vopc::v_cmp_ge_f32;
    }
}

constexpr uint32_t literal_half = 0x3f000000u;          // 0.5f
constexpr uint32_t literal_255 = 0x437f0000u;           // 255.0f
constexpr uint32_t literal_inv_255 = 0x3b808081u;       // 1.0f / 255.0f
constexpr uint32_t literal_lod_min = 0xc3000000u;       // -texture_lod_limit (-128.0f)
constexpr uint32_t literal_lod_max = 0x43000000u;       // texture_lod_limit (128.0f)

// keep &= the lanes of mask (an SGPR the caller just wrote).
void narrow_keep(Assembler& a, const PsPlan& p) noexcept {
    a.sop2(Sop2::s_and_b32, s(p.keep), s(p.keep), s(p.temp_s));
}

// GL 1.1 3.5.2: window-anchored 32x32 pattern, row (GL y) & 31, bit 31 - (x & 31).
// GL y = height - pos_y truncated (pos_y is the top-down row centre).
RX7900_SHADER_NOINLINE void polygon_stipple(Emitter& e, const PsPlan& p) noexcept {
    Assembler& a = e.a;
    const uint32_t x = p.scratch;
    const uint32_t y = p.scratch + 1;
    const uint32_t row = p.scratch + 2;
    a.vop1(Vop1::v_cvt_u32_f32, v(x), v(p.pos_x));
    a.vop2(Vop2::v_and_b32, v(x), imm(31), v(x));
    e.sub(v(y), s(constant_sgpr(abi::pc_height)), v(p.pos_y));
    a.vop1(Vop1::v_cvt_u32_f32, v(y), v(y));
    a.vop2(Vop2::v_and_b32, v(y), imm(31), v(y));
    a.vop2(Vop2::v_lshlrev_b32, v(y), imm(2), v(y));
    a.mubuf(Mubuf::buffer_load_b32, v(row), v(y), abi::ps_sgpr_constants, imm(0), abi::pc_stipple_rows * 4u);
    a.waitcnt(0, waitcnt_max_exp, waitcnt_max_lgkm);
    a.vop2(Vop2::v_sub_nc_u32, v(x), imm(31), v(x));
    a.vop2(Vop2::v_lshrrev_b32, v(row), v(x), v(row));
    a.vop2(Vop2::v_and_b32, v(row), imm(1), v(row));
    e.compare(Vopc::v_cmp_ne_u32, s(p.temp_s), imm(0), v(row));
    narrow_keep(a, p);
}

// GL 1.1 3.4.2: sample k of the segment carries s = c0 + k in the misc slot's .y
// (linear interpolation); the fragment passes when pattern bit
// floor(s / factor) & 15 is set. k = floor(s + 0.5); floor((k + 0.5) / factor)
// is exact for the counter range the backend keeps (below 16 * factor + 8192).
RX7900_SHADER_NOINLINE void line_stipple(Emitter& e, const PsPlan& p, uint32_t misc_slot) noexcept {
    Assembler& a = e.a;
    const uint32_t k = p.scratch;
    const uint32_t bits = p.scratch + 1;
    interpolate_channel(a, p, misc_slot, 1, k, p.linear_i, p.linear_j);
    e.add(v(k), literal(literal_half), v(k));
    e.unary(Vop1::v_floor_f32, v(k), v(k));
    e.add(v(k), literal(literal_half), v(k));
    e.mul(v(k), s(constant_sgpr(abi::pc_stipple_inv_factor)), v(k));
    e.unary(Vop1::v_floor_f32, v(k), v(k));
    a.vop1(Vop1::v_cvt_u32_f32, v(k), v(k));
    a.vop2(Vop2::v_and_b32, v(k), imm(15), v(k));
    e.mov(v(bits), s(constant_sgpr(abi::pc_stipple_pattern)));
    a.vop2(Vop2::v_lshrrev_b32, v(bits), v(k), v(bits));
    a.vop2(Vop2::v_and_b32, v(bits), imm(1), v(bits));
    e.compare(Vopc::v_cmp_ne_u32, s(p.temp_s), imm(0), v(bits));
    narrow_keep(a, p);
}

// Pixel rectangle (gl_pipeline PixelFragments): image column/row of this pixel
// centre, kill outside the present rows, fetch through the image V#, then the
// optional transfer. Colour sources set v[p.color..]; depth and stencil
// sources leave the MRTZ value in p.scratch + 6.
RX7900_SHADER_NOINLINE void pixel_source(Emitter& e, const abi::ShaderKey& key, const PsPlan& p) noexcept {
    Assembler& a = e.a;
    const uint32_t column = p.scratch;
    const uint32_t row = p.scratch + 1;
    const uint32_t offset = p.scratch + 2;
    const uint32_t data = p.scratch + 3;
    const uint32_t value = p.scratch + 6;
    const uint8_t source = key.pixel_source;

    // column = (x + 0.5 - raster_x) * (1 / zoom_x); row with the GL y.
    e.sub(v(column), v(p.pos_x), s(constant_sgpr(abi::pc_raster_x)));
    e.mul(v(column), v(column), s(constant_sgpr(abi::pc_inv_zoom_x)));
    e.sub(v(row), s(constant_sgpr(abi::pc_height)), v(p.pos_y));
    e.sub(v(row), v(row), s(constant_sgpr(abi::pc_raster_y)));
    e.mul(v(row), v(row), s(constant_sgpr(abi::pc_inv_zoom_y)));
    e.compare(Vopc::v_cmp_ge_f32, s(p.temp_s), v(column), imm(0));
    narrow_keep(a, p);
    e.compare(Vopc::v_cmp_lt_f32, s(p.temp_s), v(column), s(constant_sgpr(abi::pc_columns)));
    narrow_keep(a, p);
    e.compare(Vopc::v_cmp_ge_f32, s(p.temp_s), v(row), s(constant_sgpr(abi::pc_first_row)));
    narrow_keep(a, p);
    e.compare(Vopc::v_cmp_lt_f32, s(p.temp_s), v(row), s(constant_sgpr(abi::pc_end_row)));
    narrow_keep(a, p);

    // Integer image position (clamped first so every lane converts in range).
    a.vop2(Vop2::v_max_f32, v(column), imm(0), v(column));
    a.vop2(Vop2::v_min_f32, v(column), s(constant_sgpr(abi::pc_columns)), v(column));
    e.unary(Vop1::v_floor_f32, v(column), v(column));
    a.vop1(Vop1::v_cvt_u32_f32, v(column), v(column));
    a.vop2(Vop2::v_max_f32, v(row), s(constant_sgpr(abi::pc_first_row)), v(row));
    a.vop2(Vop2::v_min_f32, v(row), s(constant_sgpr(abi::pc_end_row)), v(row));
    e.unary(Vop1::v_floor_f32, v(row), v(row));
    a.vop1(Vop1::v_cvt_u32_f32, v(row), v(row));

    // offset = image_offset + row * row_stride + column * bytes per pixel.
    a.vop3(Vop3::v_mul_lo_u32, v(offset), v(row), s(constant_sgpr(abi::pc_row_stride)), Operand{});
    a.vop2(Vop2::v_add_nc_u32, v(offset), s(constant_sgpr(abi::pc_image_offset)), v(offset));
    const uint32_t bytes = abi::pixel_source_bytes(source);
    if (bytes > 1) {
        a.vop2(Vop2::v_lshlrev_b32, v(column), imm(bytes == 8 ? 3 : 2), v(column));
    }
    a.vop2(Vop2::v_add_nc_u32, v(offset), v(column), v(offset));
    const uint32_t image = constant_sgpr(abi::pc_pixel_buffer);
    const bool byte = source == abi::pixel_source_coverage8 || abi::pixel_source_stencil(source);
    a.mubuf(byte ? Mubuf::buffer_load_u8 : Mubuf::buffer_load_b32, v(data), v(offset), image, imm(0),
            source == abi::pixel_source_stencil_copy ? 4u : 0u);
    a.waitcnt(0, waitcnt_max_exp, waitcnt_max_lgkm);

    if (source == abi::pixel_source_rgba8 || source == abi::pixel_source_bgra8) {
        const bool bgra = source == abi::pixel_source_bgra8;
        const Vop1 bytes_of[4] = {Vop1::v_cvt_f32_ubyte0, Vop1::v_cvt_f32_ubyte1, Vop1::v_cvt_f32_ubyte2,
                                  Vop1::v_cvt_f32_ubyte3};
        for (uint32_t c = 0; c < 4; ++c) {
            const uint32_t from = bgra && c != 1 && c != 3 ? 2u - c : c;
            e.unary(bytes_of[from], v(p.color + c), v(data));
            e.mul(v(p.color + c), literal(literal_inv_255), v(p.color + c));
            if (key.pixel_transfer != 0) {
                // c * scale + bias, clamped and quantized to the framebuffer's 8 bits.
                e.mul(v(p.color + c), v(p.color + c), s(constant_sgpr(abi::pc_scale + c)));
                e.add(v(p.color + c), v(p.color + c), s(constant_sgpr(abi::pc_bias + c)));
                e.clamp01(v(p.color + c), v(p.color + c), v(p.one));
                e.mul(v(p.color + c), literal(literal_255), v(p.color + c));
                e.add(v(p.color + c), literal(literal_half), v(p.color + c));
                e.unary(Vop1::v_floor_f32, v(p.color + c), v(p.color + c));
                e.mul(v(p.color + c), literal(literal_inv_255), v(p.color + c));
            }
        }
    } else if (source == abi::pixel_source_coverage8) {
        e.compare(Vopc::v_cmp_ne_u32, s(p.temp_s), imm(0), v(data));
        narrow_keep(a, p);
    } else if (abi::pixel_source_depth(source)) {
        e.mov(v(value), v(data));
        if (key.pixel_transfer != 0) {
            e.mul(v(value), v(value), s(constant_sgpr(abi::pc_scale)));
            e.add(v(value), v(value), s(constant_sgpr(abi::pc_bias)));
            e.clamp01(v(value), v(value), v(p.one));
        }
    } else {
        if (key.pixel_transfer != 0) {
            // ((index << left) >> right) + offset, low 8 bits.
            a.vop2(Vop2::v_lshlrev_b32, v(data), s(constant_sgpr(abi::pc_index_shift)), v(data));
            a.vop2(Vop2::v_lshrrev_b32, v(data), s(constant_sgpr(abi::pc_index_right_shift)), v(data));
            a.vop2(Vop2::v_add_nc_u32, v(data), s(constant_sgpr(abi::pc_index_offset)), v(data));
            a.vop2(Vop2::v_and_b32, v(data), literal(0xffu), v(data));
        }
        // UINT16_ABGR MRTZ: the stencil test value in x[23:16].
        a.vop2(Vop2::v_lshlrev_b32, v(value), imm(16), v(data));
    }
}

// lambda of this fragment into v[p.lod]: per segment for lines (misc .z),
// coarse quad differences otherwise (gl_pipeline quad_lod; points and pixel
// rectangles have none and clamp to -texture_lod_limit). (s/q, t/q) is in
// v[p.coord..].
void texture_lod(Emitter& e, const abi::ShaderKey& key, const PsPlan& p, uint32_t misc_slot) noexcept {
    Assembler& a = e.a;

    if (key.vertices_per_primitive == 2) {
        interpolate_channel(a, p, misc_slot, 2, p.lod, p.persp_i, p.persp_j);
        return;
    }

    const uint32_t u = p.texel;
    const uint32_t w = p.texel + 1;
    const uint32_t q = p.scratch;     // u0, u1, u2, v0, v1, v2
    e.mul(v(u), v(p.coord), s(constant_sgpr(abi::pc_texture_width)));
    e.mul(v(w), v(p.coord + 1), s(constant_sgpr(abi::pc_texture_height)));
    for (uint32_t axis = 0; axis < 2; ++axis) {
        const uint32_t source = axis == 0 ? u : w;
        const uint32_t base = q + axis * 3u;
        // VINTERP reads quad lanes 0, 1 and 2 of its first source: lane 0 as
        // fma(x[1], 0, x[0]), lane 1 as fma(x[1], 1, 0), lane 2 as fma(x[2], 1, 0).
        a.vinterp(Vinterp::v_interp_p10_f32, v(base), v(source), v(p.zero), v(source), 0);
        a.vinterp(Vinterp::v_interp_p10_f32, v(base + 1), v(source), v(p.one), v(p.zero), 0);
        a.vinterp(Vinterp::v_interp_p2_f32, v(base + 2), v(source), v(p.one), v(p.zero), 0);
    }
    e.sub(v(q + 1), v(q + 1), v(q));            // dux
    e.sub(v(q + 2), v(q + 2), v(q));            // duy
    e.sub(v(q + 4), v(q + 4), v(q + 3));        // dvx
    e.sub(v(q + 5), v(q + 5), v(q + 3));        // dvy
    e.mul(v(q), v(q + 1), v(q + 1));
    e.mul(v(q + 3), v(q + 4), v(q + 4));
    e.add(v(q), v(q), v(q + 3));                // rho_x
    e.mul(v(q + 1), v(q + 2), v(q + 2));
    e.mul(v(q + 3), v(q + 5), v(q + 5));
    e.add(v(q + 1), v(q + 1), v(q + 3));        // rho_y
    a.vop2(Vop2::v_max_f32, v(q), v(q + 1), v(q));
    e.unary(Vop1::v_log_f32, v(q), v(q));
    e.mul(v(p.lod), v(q), literal(literal_half));
    a.vop2(Vop2::v_max_f32, v(p.lod), literal(literal_lod_min), v(p.lod));
    a.vop2(Vop2::v_min_f32, v(p.lod), literal(literal_lod_max), v(p.lod));
}

// gl_pipeline texture_levels and the level loop: magnifying lanes sample level
// 0 with the magnification S#; minifying lanes sample level_a (and level_b for
// *_MIPMAP_LINEAR) with the single-level T#s of the PS constants, then blend
// a * (1 - weight) + b * weight. EXEC is the whole-quad mask on entry and exit.
void texture_levels(Emitter& e, const abi::ShaderKey& key, const PsPlan& p) noexcept {
    Assembler& a = e.a;
    const uint8_t filter = key.min_filter;
    const bool mip_nearest = filter == abi::texture_min_nearest_mipmap_nearest ||
                             filter == abi::texture_min_linear_mipmap_nearest;
    const bool mip_linear = filter == abi::texture_min_nearest_mipmap_linear ||
                            filter == abi::texture_min_linear_mipmap_linear;
    const uint32_t t0 = p.scratch;
    const uint32_t t1 = p.scratch + 1;
    const Operand last = s(constant_sgpr(abi::pc_last_level));

    a.sop1(Sop1::s_mov_b32, s(p.quads), exec_lo());
    e.compare(Vopc::v_cmp_le_f32, s(p.magnify), v(p.lod), s(constant_sgpr(abi::pc_lod_threshold)));
    a.sop2(Sop2::s_xor_b32, s(p.minify), exec_lo(), s(p.magnify));
    e.mov(v(p.level_a), imm(0));
    e.mov(v(p.level_b), imm(0));
    e.mov(v(p.weight), imm(0));

    if (mip_nearest) {
        // d = lambda <= 0.5 ? 0 : min(ceil(lambda + 0.5) - 1, q).
        e.add(v(t0), literal(literal_half), v(p.lod));
        e.unary(Vop1::v_ceil_f32, v(t0), v(t0));
        e.sub(v(t0), v(t0), v(p.one));
        e.compare(Vopc::v_cmp_le_f32, s(p.temp_s), v(p.lod), literal(literal_half));
        e.select(v(t0), s(p.temp_s), v(p.zero), v(t0));
        a.vop2(Vop2::v_min_f32, v(t0), last, v(t0));
        a.vop1(Vop1::v_cvt_u32_f32, v(p.level_a), v(t0));
    } else if (mip_linear) {
        // lambda >= q: q, q, 0; else floor(lambda), floor(lambda) + 1, lambda - floor(lambda).
        e.unary(Vop1::v_floor_f32, v(t0), v(p.lod));
        e.sub(v(p.weight), v(p.lod), v(t0));
        e.add(v(t1), v(t0), v(p.one));
        e.compare(Vopc::v_cmp_ge_f32, s(p.temp_s), v(p.lod), last);
        e.select(v(t0), s(p.temp_s), last, v(t0));
        e.select(v(t1), s(p.temp_s), last, v(t1));
        e.select(v(p.weight), s(p.temp_s), v(p.zero), v(p.weight));
        e.select(v(p.weight), s(p.magnify), v(p.zero), v(p.weight));
        a.vop2(Vop2::v_max_f32, v(t0), imm(0), v(t0));
        a.vop2(Vop2::v_max_f32, v(t1), imm(0), v(t1));
        a.vop1(Vop1::v_cvt_u32_f32, v(p.level_a), v(t0));
        a.vop1(Vop1::v_cvt_u32_f32, v(p.level_b), v(t1));
    }
    for (uint32_t c = 0; c < 4; ++c) {
        e.mov(v(p.sample_b + c), imm(0));
    }

    // Magnification: level 0 (the user T#) with the magnification S#.
    Label after_magnify = a.make_label();
    a.sop1(Sop1::s_mov_b32, exec_lo(), s(p.magnify));
    a.branch(Sopp::s_cbranch_execz, after_magnify);
    a.mimg_sample(Mimg::image_sample_lz, v(p.sample), v(p.coord), abi::ps_sgpr_texture,
                  constant_sgpr(abi::pc_mag_sampler), 0xf, mimg_dim_2d);
    a.waitcnt(0, waitcnt_max_exp, waitcnt_max_lgkm);
    a.bind(after_magnify);
    a.sop1(Sop1::s_mov_b32, exec_lo(), s(p.quads));

    // Minification: one pass per level present.
    Label loop = a.make_label();
    a.sop1(Sop1::s_mov_b32, s(p.level), imm(0));
    a.bind(loop);
    a.sop2(Sop2::s_lshl_b32, s(p.level_offset), s(p.level), imm(5));
    a.sop2(Sop2::s_add_u32, s(p.level_offset), s(p.level_offset), literal(abi::pc_levels * 4u));
    a.smem(Smem::s_buffer_load_b256, ps_s_level_texture, abi::ps_sgpr_constants, s(p.level_offset), 0);
    a.waitcnt(waitcnt_max_vm, waitcnt_max_exp, 0);
    for (uint32_t pass = 0; pass < (mip_linear ? 2u : 1u); ++pass) {
        Label skip = a.make_label();
        const uint32_t level = pass == 0 ? p.level_a : p.level_b;
        const uint32_t target = pass == 0 ? p.sample : p.sample_b;
        a.sop1(Sop1::s_mov_b32, exec_lo(), s(p.quads));
        e.compare(Vopc::v_cmp_eq_u32, s(p.temp_s), s(p.level), v(level));
        a.sop2(Sop2::s_and_b32, exec_lo(), s(p.minify), s(p.temp_s));
        a.branch(Sopp::s_cbranch_execz, skip);
        a.mimg_sample(Mimg::image_sample_lz, v(p.texel), v(p.coord), ps_s_level_texture,
                      constant_sgpr(abi::pc_min_sampler), 0xf, mimg_dim_2d);
        a.waitcnt(0, waitcnt_max_exp, waitcnt_max_lgkm);
        for (uint32_t c = 0; c < 4; ++c) {
            e.mov(v(target + c), v(p.texel + c));
        }
        a.bind(skip);
    }
    a.sop1(Sop1::s_mov_b32, exec_lo(), s(p.quads));
    a.sop2(Sop2::s_add_u32, s(p.level), s(p.level), imm(1));
    a.sopc(Sopc::s_cmp_lt_u32, s(p.level), s(constant_sgpr(abi::pc_level_count)));
    a.branch(Sopp::s_cbranch_scc1, loop);

    if (mip_linear) {
        e.sub(v(t0), v(p.one), v(p.weight));
        for (uint32_t c = 0; c < 4; ++c) {
            e.mul(v(p.sample + c), v(p.sample + c), v(t0));
            e.mul(v(t1), v(p.sample_b + c), v(p.weight));
            e.add(v(p.sample + c), v(p.sample + c), v(t1));
        }
    }
}

// Texture coordinates, sampling (level 0, or the LOD level loop) and the
// texture environment.
RX7900_SHADER_NOINLINE void pixel_texture(Emitter& e, const abi::ShaderKey& key, const PsPlan& p,
                                          const abi::AttributeLayout& layout) noexcept {
    Assembler& a = e.a;
    interpolate(a, p, layout.texcoord, p.tex, 4, false);
    // (s, t) / (q != 0 ? q : 1), then sample.
    e.compare(Vopc::v_cmp_neq_f32, s(p.pass), v(p.tex + 3), imm(0));
    e.select(v(p.work), s(p.pass), v(p.tex + 3), v(p.one));
    e.unary(Vop1::v_rcp_f32, v(p.work), v(p.work));
    e.mul(v(p.coord), v(p.tex), v(p.work));
    e.mul(v(p.coord + 1), v(p.tex + 1), v(p.work));
    if (key.min_filter == abi::texture_lod_none) {
        // Level 0 with the one S# filter (v3).
        a.mimg_sample(Mimg::image_sample_lz, v(p.sample), v(p.coord), abi::ps_sgpr_texture,
                      abi::ps_sgpr_sampler, 0xf, mimg_dim_2d);
        a.waitcnt(0, waitcnt_max_exp, waitcnt_max_lgkm);
    } else {
        texture_lod(e, key, p, layout.fog);
        texture_levels(e, key, p);
    }
    texture_environment(e, key, p);
}

// MRTZ first (aco export order), then MRT0 and, for GL_FRONT_AND_BACK, MRT1;
// the last export is done. A stencil source exports MRTZ only.
RX7900_SHADER_NOINLINE void pixel_exports(Assembler& a, const abi::ShaderKey& key, const PsPlan& p) noexcept {
    const uint32_t value = p.scratch + 6;
    if (abi::pixel_source_stencil(key.pixel_source)) {
        a.exp(ExpTarget::mrtz, v(value), Operand{}, Operand{}, Operand{}, true);
        return;
    }
    if (abi::pixel_source_depth(key.pixel_source)) {
        a.exp(ExpTarget::mrtz, v(value), Operand{}, Operand{}, Operand{}, false);
    }
    a.exp(ExpTarget::mrt0, v(p.color), v(p.color + 1), v(p.color + 2), v(p.color + 3), key.front_and_back == 0);
    if (key.front_and_back != 0) {
        a.exp(static_cast<ExpTarget>(static_cast<uint8_t>(ExpTarget::mrt0) + 1u), v(p.color), v(p.color + 1),
              v(p.color + 2), v(p.color + 3), true);
    }
}

} // namespace

Status generate_vertex_shader(const abi::ShaderKey& key, rx7900::sdma::CommandBuffer output,
                              abi::ProgramInfo& info, GeneratorWorkspace& workspace) noexcept {
    info = {};

    if (!abi::key_valid(key)) {
        return Status::invalid_argument;
    }

    const abi::AttributeLayout layout = abi::attribute_layout(key);
    Assembler& a = workspace.assembler;
    a.reset(output);
    Emitter e(a);

    // Subgroup counts from merged_wave_info (one wave per subgroup, see the ABI).
    a.sop1(Sop1::s_mov_b32, s(vs_s_saved_exec), exec_lo());
    a.sop2(Sop2::s_and_b32, s(vs_s_vertex_count), s(abi::vs_sgpr_merged_wave_info), imm(255));
    a.sop2(Sop2::s_lshr_b32, s(vs_s_prim_count), s(abi::vs_sgpr_merged_wave_info), imm(8));
    a.sop2(Sop2::s_and_b32, s(vs_s_prim_count), s(vs_s_prim_count), imm(255));
    // Attribute-ring wave offset: gs_attr_offset[14:0] in 512-byte units.
    a.sop2(Sop2::s_and_b32, s(vs_s_ring_offset), s(abi::vs_sgpr_attr_offset), literal(0x7fffu));
    a.sop2(Sop2::s_lshl_b32, s(vs_s_ring_offset), s(vs_s_ring_offset), imm(9));

    // GS_ALLOC_REQ from wave 0 of the subgroup: m0 = vertices | primitives << 12.
    Label after_alloc = a.make_label();
    a.sop2(Sop2::s_lshr_b32, s(vs_s_temp), s(abi::vs_sgpr_merged_wave_info), imm(24));
    a.sop2(Sop2::s_and_b32, s(vs_s_temp), s(vs_s_temp), imm(15));
    a.sopc(Sopc::s_cmp_lg_u32, s(vs_s_temp), imm(0));
    a.branch(Sopp::s_cbranch_scc1, after_alloc);
    a.sop2(Sop2::s_lshl_b32, s(vs_s_temp), s(vs_s_prim_count), imm(12));
    a.sop2(Sop2::s_or_b32, s(vs_s_temp), s(vs_s_temp), s(vs_s_vertex_count));
    a.sop1(Sop1::s_mov_b32, m0(), s(vs_s_temp));
    a.sendmsg(msg_gs_alloc_req);
    a.bind(after_alloc);

    // Lane id within the wave.
    a.vop3(Vop3::v_mbcnt_lo_u32_b32, v(vs_v_lane), imm(-1), imm(0), Operand{});

    // Primitive lanes: export the packed vertex indices.
    Label after_prim = a.make_label();
    e.compare(Vopc::v_cmp_gt_u32, s(vs_s_gs_mask), s(vs_s_prim_count), v(vs_v_lane));
    a.sop2(Sop2::s_and_b32, exec_lo(), s(vs_s_saved_exec), s(vs_s_gs_mask));
    a.branch(Sopp::s_cbranch_execz, after_prim);
    a.vop2(Vop2::v_and_b32, v(vs_v_prim_a), literal(0xffffu), v(abi::vs_vgpr_vertex_offsets01));

    if (key.vertices_per_primitive >= 2) {
        a.vop2(Vop2::v_lshrrev_b32, v(vs_v_prim_b), imm(16), v(abi::vs_vgpr_vertex_offsets01));
        a.vop3(Vop3::v_lshl_or_b32, v(vs_v_prim_a), v(vs_v_prim_b), imm(10), v(vs_v_prim_a));
    }

    if (key.vertices_per_primitive == 3) {
        a.vop2(Vop2::v_and_b32, v(vs_v_prim_b), literal(0xffffu), v(abi::vs_vgpr_vertex_offsets23));
        a.vop3(Vop3::v_lshl_or_b32, v(vs_v_prim_a), v(vs_v_prim_b), imm(20), v(vs_v_prim_a));
    }

    a.exp(ExpTarget::prim, v(vs_v_prim_a), Operand{}, Operand{}, Operand{}, true);
    a.bind(after_prim);

    // Vertex lanes (the compare needs the full wave EXEC again).
    Label end = a.make_label();
    a.sop1(Sop1::s_mov_b32, exec_lo(), s(vs_s_saved_exec));
    e.compare(Vopc::v_cmp_gt_u32, s(vs_s_es_mask), s(vs_s_vertex_count), v(vs_v_lane));
    a.sop2(Sop2::s_and_b32, exec_lo(), s(vs_s_saved_exec), s(vs_s_es_mask));
    a.branch(Sopp::s_cbranch_execz, end);
    const uint32_t position = vertex_body(e, key, layout);
    // GFX11: the attribute-ring stores must complete before the position export.
    a.sopk(Sopk::s_waitcnt_vscnt, null_sgpr(), 0);
    a.exp(ExpTarget::pos0, v(position), v(position + 1), v(position + 2), v(position + 3), true);
    a.bind(end);
    a.endpgm();

    const Status status = finish(a, info);

    if (status != Status::success) {
        return status;
    }

    info.vgpr_count = vs_vgpr_count;
    info.sgpr_count = vs_sgpr_count;
    info.user_sgpr_count = abi::vs_user_sgpr_count;
    info.float_mode = abi::float_mode;
    info.es_vgpr_comp_cnt = abi::vs_es_vgpr_comp_cnt;
    info.gs_vgpr_comp_cnt = abi::vs_gs_vgpr_comp_cnt(key.vertices_per_primitive);
    info.pos_export_count = 1;
    info.param_count = layout.count;
    info.max_es_verts = abi::ngg_max_verts_per_subgroup;
    info.max_gs_prims = abi::ngg_max_prims_per_subgroup;
    info.writes_attribute_ring = 1;
    return Status::success;
}

Status generate_pixel_shader(const abi::ShaderKey& key, rx7900::sdma::CommandBuffer output,
                             abi::ProgramInfo& info, GeneratorWorkspace& workspace) noexcept {
    info = {};

    if (!abi::key_valid(key)) {
        return Status::invalid_argument;
    }

    const abi::AttributeLayout layout = abi::attribute_layout(key);
    const PsPlan p = ps_plan(key);
    const bool constants = abi::uses_ps_constants(key);
    const bool alpha_kills = key.alpha_test != 0 && key.alpha_func != abi::compare_always;
    // Stipple and pixel-rectangle coverage narrow p.keep (v4 plan only).
    const bool keep_kills = key.line_stipple != 0 || key.polygon_stipple != 0 ||
                            key.pixel_source != abi::pixel_source_none;
    const bool kills = alpha_kills || keep_kills;
    const bool stencil_out = abi::pixel_source_stencil(key.pixel_source);
    Assembler& a = workspace.assembler;
    a.reset(output);
    Emitter e(a);

    // Live mask, whole-quad mode for lds_param_load, primitive mask in m0.
    a.sop1(Sop1::s_mov_b32, s(p.live), exec_lo());
    a.sop1(Sop1::s_wqm_b32, exec_lo(), exec_lo());
    a.sop1(Sop1::s_mov_b32, m0(), s(p.prim_mask));
    e.mov(v(p.zero), imm(0));
    a.vop1(Vop1::v_cvt_f32_u32, v(p.one), imm(1));

    if (constants) {
        // The PS constant header into s40..s79; keep starts as the live mask.
        a.sop1(Sop1::s_mov_b32, s(p.keep), s(p.live));
        a.smem(Smem::s_buffer_load_b512, ps_s_constants, abi::ps_sgpr_constants, null_sgpr(), 0);
        a.smem(Smem::s_buffer_load_b512, ps_s_constants + 16, abi::ps_sgpr_constants, null_sgpr(), 64);
        a.smem(Smem::s_buffer_load_b256, ps_s_constants + 32, abi::ps_sgpr_constants, null_sgpr(), 128);
        a.waitcnt(waitcnt_max_vm, waitcnt_max_exp, 0);
    }

    const bool flat_front = (layout.flat_mask & 1u) != 0;
    interpolate(a, p, layout.front_color, p.color, 4, flat_front);

    if (layout.back_color != abi::slot_none) {
        interpolate(a, p, layout.back_color, p.back, 4, ((layout.flat_mask >> layout.back_color) & 1u) != 0);
        // front_face nonzero selects the front colour.
        e.compare(Vopc::v_cmp_ne_u32, s(p.front), imm(0), v(p.front_face));

        for (uint32_t c = 0; c < 4; ++c) {
            e.select(v(p.color + c), s(p.front), v(p.color + c), v(p.back + c));
        }
    }

    if (key.polygon_stipple != 0) {
        polygon_stipple(e, p);
    }

    if (key.line_stipple != 0) {
        line_stipple(e, p, layout.fog);
    }

    if (key.pixel_source != abi::pixel_source_none) {
        pixel_source(e, key, p);
    }

    if (layout.texcoord != abi::slot_none) {
        pixel_texture(e, key, p, layout);
    }

    if (key.fog != 0) {
        interpolate(a, p, layout.fog, p.fog, 1, false);
        // c = c * f + fog_color * (1 - f), f = clamp(fog).
        const uint32_t f = p.fog;
        e.clamp01(v(f), v(f), v(p.one));
        e.sub(v(p.work + 1), v(p.one), v(f));

        for (uint32_t c = 0; c < 3; ++c) {
            e.mul(v(p.work), v(p.color + c), v(f));
            e.mul(v(p.work + 2), s(abi::ps_sgpr_fog_color + c), v(p.work + 1));
            e.add(v(p.color + c), v(p.work), v(p.work + 2));
        }
    }

    // Back to the live pixels, narrowed by stipple/coverage and the alpha test.
    const uint32_t surviving = keep_kills ? p.keep : p.live;
    if (!alpha_kills) {
        a.sop1(Sop1::s_mov_b32, exec_lo(), s(surviving));
    } else if (key.alpha_func == abi::compare_never) {
        a.sop1(Sop1::s_mov_b32, exec_lo(), imm(0));
    } else {
        e.compare(alpha_compare(key.alpha_func), s(p.pass), v(p.color + 3), s(abi::ps_sgpr_alpha_ref));
        a.sop2(Sop2::s_and_b32, exec_lo(), s(surviving), s(p.pass));
    }

    Label killed = a.make_label();

    if (kills) {
        a.branch(Sopp::s_cbranch_execz, killed);
    }

    pixel_exports(a, key, p);
    a.endpgm();

    if (kills) {
        a.bind(killed);
        a.exp(stencil_out ? ExpTarget::mrtz : ExpTarget::mrt0, Operand{}, Operand{}, Operand{}, Operand{}, true);
        a.endpgm();
    }

    const Status status = finish(a, info);

    if (status != Status::success) {
        return status;
    }

    info.vgpr_count = p.vgpr_count;
    info.sgpr_count = p.sgpr_count;
    info.user_sgpr_count = abi::ps_user_sgprs(key);
    info.float_mode = abi::float_mode;
    info.spi_ps_input_ena = abi::ps_input_ena(key);
    info.interpolant_count = layout.count;
    info.param_count = layout.count;
    info.flat_mask = layout.flat_mask;
    info.col_format = abi::spi_shader_col_format_of(key);
    info.z_format = abi::spi_shader_z_format_of(key);
    info.kills = kills ? 1u : 0u;
    info.reads_front_face = layout.back_color != abi::slot_none ? 1u : 0u;
    return Status::success;
}

gfx11_abi::ShaderKey clip_space_key(const DrawState& state, PrimitiveKind kind) noexcept {
    abi::ShaderKey key{};
    key.vertex_input = abi::vertex_input_clip_space;
    key.smooth = 1;
    key.two_sided = state.two_sided_color ? 1 : 0;
    key.vertices_per_primitive = kind == PrimitiveKind::points ? 1 : (kind == PrimitiveKind::lines ? 2 : 3);
    key.fog = state.fog_enabled ? 1 : 0;

    if (state.texture != nullptr) {
        key.textured = 1;

        switch (state.texture_env_mode) {
        case gl::decal: key.tex_env = abi::tex_env_decal; break;
        case gl::blend: key.tex_env = abi::tex_env_blend; break;
        case gl::replace: key.tex_env = abi::tex_env_replace; break;
        default: key.tex_env = abi::tex_env_modulate; break;
        }

        // v4: per-fragment level selection when minification and
        // magnification can differ (gl_pipeline texture_levels).
        if (texture_needs_lod(state.texture->sampler)) {
            switch (state.texture->sampler.min_filter) {
            case gl::nearest: key.min_filter = abi::texture_min_nearest; break;
            case gl::linear: key.min_filter = abi::texture_min_linear; break;
            case gl::nearest_mipmap_nearest: key.min_filter = abi::texture_min_nearest_mipmap_nearest; break;
            case gl::linear_mipmap_nearest: key.min_filter = abi::texture_min_linear_mipmap_nearest; break;
            case gl::nearest_mipmap_linear: key.min_filter = abi::texture_min_nearest_mipmap_linear; break;
            default: key.min_filter = abi::texture_min_linear_mipmap_linear; break;
            }
            key.mag_filter = state.texture->sampler.mag_filter == gl::nearest ? abi::texture_mag_nearest
                                                                               : abi::texture_mag_linear;
        }

        switch (state.texture->levels[0].base_format) {
        case gl::alpha: key.texture_base = abi::texture_base_alpha; break;
        case gl::luminance: key.texture_base = abi::texture_base_luminance; break;
        case gl::luminance_alpha: key.texture_base = abi::texture_base_luminance_alpha; break;
        case gl::intensity: key.texture_base = abi::texture_base_intensity; break;
        case gl::rgb: key.texture_base = abi::texture_base_rgb; break;
        default: key.texture_base = abi::texture_base_rgba; break;
        }
    }

    if (state.alpha_test) {
        key.alpha_test = 1;

        switch (state.alpha_func) {
        case gl::never: key.alpha_func = abi::compare_never; break;
        case gl::less: key.alpha_func = abi::compare_less; break;
        case gl::equal: key.alpha_func = abi::compare_equal; break;
        case gl::lequal: key.alpha_func = abi::compare_lequal; break;
        case gl::greater: key.alpha_func = abi::compare_greater; break;
        case gl::notequal: key.alpha_func = abi::compare_notequal; break;
        case gl::gequal: key.alpha_func = abi::compare_gequal; break;
        default: key.alpha_func = abi::compare_always; break;
        }
    }

    key.line_stipple = kind == PrimitiveKind::lines && state.line_stipple ? 1 : 0;
    key.polygon_stipple = kind == PrimitiveKind::triangles && state.polygon_stipple ? 1 : 0;
    key.front_and_back = state.draw_buffer == gl::front_and_back ? 1 : 0;
    return key;
}

} // namespace rx7900::opengl::shader

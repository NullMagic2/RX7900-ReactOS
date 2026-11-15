/*
Module: rx7900.tests.opengl_shader_pixel
Purpose: Run complete generated GFX11 NGG vertex + pixel programs on a modelled
draw and compare every framebuffer pixel with the gl_raster software reference.
Created: 2026-10-23
Architecture: The programs come from gl_shader_gfx11 for a gl_gfx11_abi key; the
descriptors are built with the ABI helpers; tests/gfx11_interp.hpp (which decodes
raw dwords and descriptor words on its own) executes them through its
GE/SPI/NGG/PA/SC/PS/CB model (graphics::run_draw). The oracle is either
SoftwareBackend::draw with the same RasterVertex list (clip-space keys, the path
the GFX11 backend uses) or a full gl_context over SoftwareBackend (object-space
keys: transform, lighting, texture matrix and fog in the vertex shader). Cases:
flat and smooth (including perspective) triangles, lit geometry, fog modes,
textured quads for every texture environment with nearest and linear filtering,
alpha test functions, points, lines, multi-subgroup draws, two-sided back faces
(clip and object space) and face culling (front, back, front_and_back with both
front-face windings). Mutation cases patch
one instruction of a generated program and must make the comparison fail.
Tolerance: one 8-bit step per channel (approximate rcp/sqrt/exp/log, separate
roundings); coverage is exact because the model reproduces gl_raster's rules.
Host only; no GPU.
*/
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <vector>

#include "gfx11_interp.hpp"
#include "../src/opengl/gl_context.hpp"
#include "../src/opengl/gl_raster.hpp"
#include "../src/opengl/gl_shader_gfx11.hpp"

namespace {

using namespace rx7900::opengl;
namespace abi = rx7900::opengl::gfx11_abi;
namespace sdma = rx7900::sdma;

unsigned checks = 0;
unsigned failures = 0;
bool quiet = false;   // mutation runs suppress comparison diagnostics

void expect(bool condition, const char* contract) {
    ++checks;
    if (!condition) {
        ++failures;
        std::fprintf(stderr, "FAIL: %s\n", contract);
    }
}

constexpr uint32_t fb_width = 64;
constexpr uint32_t fb_height = 64;
constexpr uint64_t vertex_base = 0x10000;
constexpr uint64_t ring_base = 0x100000;
constexpr uint64_t constants_base = 0x200000;
constexpr uint64_t texture_base = 0x300000;
constexpr uint32_t ring_bytes = 8 * 512 * 8;   // 8 wave slots of up to 8 params

uint32_t bits(float value) {
    uint32_t out = 0;
    std::memcpy(&out, &value, 4);
    return out;
}

// ---------------------------------------------------------------------------
// Instruction walker and mutations.
// ---------------------------------------------------------------------------

// Dword length of the instruction at words[i] for the subset the generators emit.
uint32_t instruction_length(const std::vector<uint32_t>& words, size_t i) {
    const uint32_t w0 = words[i];
    auto lit = [](uint32_t code) { return code == 255; };

    if ((w0 >> 31) == 0) {                                  // VOP1 / VOP2 / VOPC
        return lit(w0 & 0x1ff) ? 2u : 1u;
    }

    if ((w0 >> 30) == 2) {                                  // scalar
        const uint32_t group = (w0 >> 23) & 0x1ff;
        if (group == 0x17f || ((w0 >> 28) & 0xf) == 0xb) {  // SOPP, SOPK
            return 1;
        }
        if (group == 0x17d) {                               // SOP1
            return lit(w0 & 0xff) ? 2u : 1u;
        }
        return (lit(w0 & 0xff) || lit((w0 >> 8) & 0xff)) ? 2u : 1u;   // SOPC / SOP2
    }

    if ((w0 >> 24) == 0xce) {                               // LDSDIR
        return 1;
    }

    if ((w0 >> 26) == 0x35) {                               // VOP3
        const uint32_t w1 = words[i + 1];
        return (lit(w1 & 0x1ff) || lit((w1 >> 9) & 0x1ff) || lit((w1 >> 18) & 0x1ff)) ? 3u : 2u;
    }

    return 2;                                               // SMEM, MUBUF, MIMG, EXP, VINTERP
}

enum class Stage { vertex, pixel };

// One-instruction mutation: the first (or n-th) instruction whose first dword
// satisfies (w0 & mask) == value gets its dword at word_offset XOR-ed with flip.
struct Mutation final {
    const char* name;
    Stage stage;
    uint32_t mask;
    uint32_t value;
    uint32_t occurrence;
    uint32_t word_offset;
    uint32_t flip;
};

const Mutation* active_mutation = nullptr;
bool mutation_applied = false;

void apply_mutation(std::vector<uint32_t>& words, Stage stage) {
    if (active_mutation == nullptr || active_mutation->stage != stage) {
        return;
    }

    uint32_t seen = 0;

    for (size_t i = 0; i < words.size(); i += instruction_length(words, i)) {
        if ((words[i] >> 23) == 0x17f && ((words[i] >> 16) & 0x7f) == 0x1f) {
            break;                                          // s_code_end padding
        }
        if ((words[i] & active_mutation->mask) == active_mutation->value) {
            if (seen++ == active_mutation->occurrence) {
                words[i + active_mutation->word_offset] ^= active_mutation->flip;
                mutation_applied = true;
                return;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// GPU model draw.
// ---------------------------------------------------------------------------

struct Programs final {
    std::vector<uint32_t> vs, ps;
    abi::ProgramInfo vs_info{}, ps_info{};
};

// One workspace reused for every program, as the kernel reuses device storage.
shader::GeneratorWorkspace workspace;

bool generate(const abi::ShaderKey& key, Programs& programs) {
    std::vector<uint8_t> bytes(abi::shader_max_bytes);

    if (shader::generate_vertex_shader(key, sdma::CommandBuffer(bytes.data(), bytes.size()), programs.vs_info,
                                       workspace) !=
        shader::Status::success) {
        return false;
    }

    programs.vs.assign(programs.vs_info.code_dwords, 0);
    std::memcpy(programs.vs.data(), bytes.data(), programs.vs.size() * 4);

    if (shader::generate_pixel_shader(key, sdma::CommandBuffer(bytes.data(), bytes.size()), programs.ps_info,
                                      workspace) !=
        shader::Status::success) {
        return false;
    }

    programs.ps.assign(programs.ps_info.code_dwords, 0);
    std::memcpy(programs.ps.data(), bytes.data(), programs.ps.size() * 4);
    // RX7900_SHADER_DUMP=path appends key, VS and PS bytes of every generated
    // program (byte-identity evidence across generator changes).
    if (active_mutation == nullptr) {
        if (const char* path = std::getenv("RX7900_SHADER_DUMP")) {
            if (std::FILE* file = std::fopen(path, "ab")) {
                std::fwrite(&key, sizeof(key), 1, file);
                std::fwrite(programs.vs.data(), 4, programs.vs.size(), file);
                std::fwrite(programs.ps.data(), 4, programs.ps.size(), file);
                std::fclose(file);
            }
        }
    }

    apply_mutation(programs.vs, Stage::vertex);
    apply_mutation(programs.ps, Stage::pixel);
    return true;
}

struct TextureImage final {
    uint32_t width = 0, height = 0;
    const uint8_t* rgba = nullptr;     // rows bottom to top, R G B A
    uint32_t clamp_s = abi::tex_clamp_wrap, clamp_t = abi::tex_clamp_wrap;
    uint32_t filter = abi::tex_filter_point;
};

struct FragmentConstants final {
    float env_color[4] = {0, 0, 0, 0};
    float fog_color[4] = {0, 0, 0, 0};
    float alpha_ref = 0;
};

struct GpuDraw final {
    abi::ShaderKey key{};
    std::vector<uint8_t> records;
    uint32_t vertex_count = 0;
    std::vector<uint32_t> indices;     // empty: 0..vertex_count-1
    const abi::VertexConstants* constants = nullptr;
    const TextureImage* texture = nullptr;
    FragmentConstants fragment{};
    bool front_ccw = true;
    bool cull_front = false;
    bool cull_back = false;
    float point_size = 1;
};

gfx11_interp::graphics::DrawStats run_gpu(const GpuDraw& draw, std::vector<uint32_t>& framebuffer,
                                          uint32_t clear) {
    gfx11_interp::graphics::DrawStats failed{};
    failed.errors = 1;
    failed.first_error = "program generation failed";
    Programs programs;

    framebuffer.assign(fb_width * fb_height, clear);

    if (!generate(draw.key, programs)) {
        return failed;
    }

    const abi::AttributeLayout layout = abi::attribute_layout(draw.key);
    gfx11_interp::Buffer vertices{vertex_base, draw.records};
    gfx11_interp::Buffer ring{ring_base, std::vector<uint8_t>(ring_bytes, 0xcd)};
    gfx11_interp::Buffer constants{constants_base, {}};
    gfx11_interp::Buffer texture{texture_base, {}};
    gfx11_interp::Memory memory{};
    memory.buffers = {&vertices, &ring, &constants, &texture};

    if (draw.constants != nullptr) {
        constants.bytes.resize(sizeof(abi::VertexConstants));
        std::memcpy(constants.bytes.data(), draw.constants, sizeof(abi::VertexConstants));
    }

    gfx11_interp::graphics::DrawSetup setup{};
    setup.vs_code = reinterpret_cast<const uint8_t*>(programs.vs.data());
    setup.vs_dwords = static_cast<uint32_t>(programs.vs.size());
    setup.ps_code = reinterpret_cast<const uint8_t*>(programs.ps.data());
    setup.ps_dwords = static_cast<uint32_t>(programs.ps.size());
    setup.vs_user_count = programs.vs_info.user_sgpr_count;
    setup.ps_user_count = programs.ps_info.user_sgpr_count;

    // VS user SGPRs (gl_gfx11_abi slot table).
    const abi::BufferDescriptor vb = abi::raw_buffer_descriptor(vertex_base, static_cast<uint32_t>(draw.records.size()));
    const abi::BufferDescriptor rd = abi::attribute_ring_descriptor(ring_base, ring_bytes, layout.count);
    const abi::BufferDescriptor cd = abi::raw_buffer_descriptor(constants_base, static_cast<uint32_t>(constants.bytes.size()));

    for (uint32_t k = 0; k < 4; ++k) {
        setup.vs_user[abi::vs_user_data_index(abi::vs_sgpr_vertex_buffer) + k] = vb.words[k];
        setup.vs_user[abi::vs_user_data_index(abi::vs_sgpr_attribute_ring) + k] = rd.words[k];
        setup.vs_user[abi::vs_user_data_index(abi::vs_sgpr_constants) + k] = cd.words[k];
    }

    setup.vs_user[abi::vs_user_data_index(abi::vs_sgpr_base_vertex)] = 0;

    // PS user SGPRs.
    if (draw.texture != nullptr) {
        const uint32_t pitch = (draw.texture->width + 63u) & ~63u;
        texture.bytes.assign(static_cast<size_t>(pitch) * draw.texture->height * 4u, 0);
        for (uint32_t y = 0; y < draw.texture->height; ++y) {
            std::memcpy(texture.bytes.data() + static_cast<size_t>(y) * pitch * 4u,
                        draw.texture->rgba + static_cast<size_t>(y) * draw.texture->width * 4u,
                        draw.texture->width * 4u);
        }
        const abi::ImageDescriptor t = abi::texture_descriptor(texture_base, draw.texture->width,
                                                               draw.texture->height, pitch);
        const abi::SamplerDescriptor sd = abi::sampler_descriptor(draw.texture->clamp_s, draw.texture->clamp_t,
                                                                  draw.texture->filter);
        for (uint32_t k = 0; k < 8; ++k) {
            setup.ps_user[abi::ps_sgpr_texture + k] = t.words[k];
        }
        for (uint32_t k = 0; k < 4; ++k) {
            setup.ps_user[abi::ps_sgpr_sampler + k] = sd.words[k];
        }
    }

    for (uint32_t k = 0; k < 4; ++k) {
        setup.ps_user[abi::ps_sgpr_env_color + k] = bits(draw.fragment.env_color[k]);
    }

    for (uint32_t k = 0; k < 3; ++k) {
        setup.ps_user[abi::ps_sgpr_fog_color + k] = bits(draw.fragment.fog_color[k]);
    }

    setup.ps_user[abi::ps_sgpr_alpha_ref] = bits(draw.fragment.alpha_ref);
    setup.vertices_per_primitive = draw.key.vertices_per_primitive;

    if (draw.indices.empty()) {
        for (uint32_t i = 0; i < draw.vertex_count; ++i) {
            setup.indices.push_back(i);
        }
    } else {
        setup.indices = draw.indices;
    }

    // Context state as the backend programs it from the ProgramInfo.
    setup.param_count = programs.vs_info.param_count;
    setup.flat_mask = programs.ps_info.flat_mask;
    setup.front_face_input = (programs.ps_info.spi_ps_input_ena & abi::spi_ps_input_front_face) != 0;
    setup.provoking_last = (abi::pa_su_sc_mode_cntl_required & (1u << 19)) != 0;
    setup.front_ccw = draw.front_ccw;
    setup.cull_front = draw.cull_front;
    setup.cull_back = draw.cull_back;
    setup.point_size = draw.point_size;
    setup.viewport = {0, 0, static_cast<int32_t>(fb_width), static_cast<int32_t>(fb_height)};
    setup.ring_base = ring_base;
    setup.ring_bytes = ring_bytes;
    setup.memory = &memory;
    setup.width = fb_width;
    setup.height = fb_height;
    setup.framebuffer = framebuffer.data();

    if (programs.vs_info.param_count != programs.ps_info.interpolant_count) {
        return failed;
    }

    return gfx11_interp::graphics::run_draw(setup);
}

// ---------------------------------------------------------------------------
// Comparison.
// ---------------------------------------------------------------------------

struct Comparison final {
    uint32_t max_diff = 0;
    uint32_t mismatches = 0;    // pixels beyond tolerance
    uint32_t exact = 0;
    uint32_t drawn = 0;         // oracle pixels differing from the clear colour
};

Comparison compare(const uint32_t* oracle, const std::vector<uint32_t>& model, uint32_t clear, uint32_t tolerance) {
    Comparison result{};

    for (size_t i = 0; i < model.size(); ++i) {
        uint32_t diff = 0;
        for (uint32_t c = 0; c < 4; ++c) {
            const int32_t a = static_cast<int32_t>((oracle[i] >> (8 * c)) & 0xff);
            const int32_t b = static_cast<int32_t>((model[i] >> (8 * c)) & 0xff);
            const uint32_t d = static_cast<uint32_t>(a > b ? a - b : b - a);
            diff = d > diff ? d : diff;
        }
        result.max_diff = diff > result.max_diff ? diff : result.max_diff;
        result.mismatches += diff > tolerance ? 1u : 0u;
        result.exact += diff == 0 ? 1u : 0u;
        result.drawn += oracle[i] != clear ? 1u : 0u;
    }

    return result;
}

constexpr uint32_t clear_value = 0x40102030u;
constexpr uint32_t tolerance = 1;

// Checks the model framebuffer against the oracle; returns true on a match.
bool matches(const char* name, const uint32_t* oracle, const std::vector<uint32_t>& model,
             const gfx11_interp::graphics::DrawStats& stats, uint32_t min_drawn) {
    const Comparison c = compare(oracle, model, clear_value, tolerance);
    const bool ok = stats.errors == 0 && c.mismatches == 0 && c.drawn >= min_drawn;

    if (!quiet) {
        std::printf("  %-34s drawn %4u exact %4u max_diff %u subgroups %u ps_waves %u%s%s\n", name, c.drawn,
                    c.exact - (fb_width * fb_height - c.drawn), c.max_diff, stats.subgroups, stats.ps_waves,
                    stats.errors != 0 ? " MODEL ERROR: " : "", stats.errors != 0 ? stats.first_error : "");
    }

    return ok;
}

// ---------------------------------------------------------------------------
// Clip-space cases (RasterVertex lists through SoftwareBackend::draw).
// ---------------------------------------------------------------------------

struct Oracle final {
    std::vector<uint8_t> storage;
    SoftwareBackend backend;

    Oracle() : storage(raster_storage_bytes(fb_width, fb_height, true)),
               backend(storage.data(), storage.size(), FramebufferFormat{true, 24, 8}) {
        backend.resize(fb_width, fb_height);
        ClearRequest clear{};
        clear.mask = gl::color_buffer_bit;
        clear.color = {static_cast<float>((clear_value >> 16) & 0xff) / 255.0f,
                       static_cast<float>((clear_value >> 8) & 0xff) / 255.0f,
                       static_cast<float>(clear_value & 0xff) / 255.0f,
                       static_cast<float>(clear_value >> 24) / 255.0f};
        backend.clear(clear);
    }

    const uint32_t* pixels() const { return backend.color_buffer(gl::back); }
};

DrawState base_state() {
    DrawState state{};
    state.viewport_width = fb_width;
    state.viewport_height = fb_height;
    return state;
}

RasterVertex rv(float x, float y, float w, Vec4 color, Vec4 tex = {0, 0, 0, 1}, float fog = 1) {
    RasterVertex v{};
    v.clip = {x * w, y * w, 0, w};
    v.color[0] = v.color[1] = color;
    v.texcoord = tex;
    v.fog = fog;
    return v;
}

std::vector<uint8_t> clip_records(const RasterVertex* vertices, uint32_t count) {
    std::vector<uint8_t> out(static_cast<size_t>(count) * abi::clip_record_stride, 0);

    for (uint32_t i = 0; i < count; ++i) {
        float record[20] = {};
        const RasterVertex& v = vertices[i];
        const float values[20] = {v.clip.x, v.clip.y, v.clip.z, v.clip.w,
                                  v.color[0].x, v.color[0].y, v.color[0].z, v.color[0].w,
                                  v.color[1].x, v.color[1].y, v.color[1].z, v.color[1].w,
                                  v.texcoord.x, v.texcoord.y, v.texcoord.z, v.texcoord.w,
                                  v.fog, 0, 0, 0};
        std::memcpy(record, values, sizeof(record));
        std::memcpy(out.data() + static_cast<size_t>(i) * abi::clip_record_stride, record, sizeof(record));
    }

    return out;
}

// Draws vertices through both paths; returns whether the model matched.
bool clip_case(const char* name, DrawState state, PrimitiveKind kind, const std::vector<RasterVertex>& oracle_vertices,
               const std::vector<RasterVertex>& gpu_vertices, const abi::ShaderKey& key,
               const TextureImage* texture, uint32_t min_drawn) {
    Oracle oracle;
    oracle.backend.draw(state, kind, oracle_vertices.data(), static_cast<uint32_t>(oracle_vertices.size()));
    GpuDraw draw{};
    draw.key = key;
    draw.records = clip_records(gpu_vertices.data(), static_cast<uint32_t>(gpu_vertices.size()));
    draw.vertex_count = static_cast<uint32_t>(gpu_vertices.size());
    draw.texture = texture;
    draw.front_ccw = state.front_ccw;
    draw.cull_front = state.cull_enabled && (state.cull_mode == gl::front || state.cull_mode == gl::front_and_back);
    draw.cull_back = state.cull_enabled && (state.cull_mode == gl::back || state.cull_mode == gl::front_and_back);
    draw.point_size = state.point_size;
    draw.fragment.env_color[0] = state.texture_env_color.x;
    draw.fragment.env_color[1] = state.texture_env_color.y;
    draw.fragment.env_color[2] = state.texture_env_color.z;
    draw.fragment.env_color[3] = state.texture_env_color.w;
    draw.fragment.fog_color[0] = state.fog_color.x;
    draw.fragment.fog_color[1] = state.fog_color.y;
    draw.fragment.fog_color[2] = state.fog_color.z;
    draw.fragment.alpha_ref = state.alpha_ref;
    std::vector<uint32_t> model;
    const auto stats = run_gpu(draw, model, clear_value);
    return matches(name, oracle.pixels(), model, stats, min_drawn);
}

bool case_smooth_triangle() {
    const DrawState state = base_state();
    const std::vector<RasterVertex> v = {rv(-0.8f, -0.7f, 1, {1, 0, 0, 1}), rv(0.9f, -0.5f, 1, {0, 1, 0, 0.5f}),
                                         rv(-0.1f, 0.85f, 1, {0, 0, 1, 0.25f})};
    return clip_case("smooth triangle", state, PrimitiveKind::triangles, v, v,
                     shader::clip_space_key(state, PrimitiveKind::triangles), nullptr, 1000);
}

bool case_perspective_triangle() {
    const DrawState state = base_state();
    const std::vector<RasterVertex> v = {rv(-0.9f, -0.9f, 1, {1, 0.5f, 0, 1}), rv(0.9f, -0.6f, 3, {0, 1, 0.25f, 1}),
                                         rv(0.1f, 0.9f, 1.75f, {0.5f, 0, 1, 1})};
    return clip_case("smooth perspective triangle", state, PrimitiveKind::triangles, v, v,
                     shader::clip_space_key(state, PrimitiveKind::triangles), nullptr, 1000);
}

bool case_flat_triangle() {
    // The GPU gets distinct vertex colours and a flat key (FLAT_SHADE, provoking
    // last); the oracle gets the frontend's flat-resolved colours.
    const DrawState state = base_state();
    const std::vector<RasterVertex> gpu = {rv(-0.8f, -0.8f, 1, {1, 0, 0, 1}), rv(0.8f, -0.6f, 1, {0, 1, 0, 1}),
                                           rv(0.0f, 0.8f, 1, {0.25f, 0.5f, 0.75f, 0.5f}),
                                           rv(-0.9f, 0.9f, 1, {1, 1, 0, 1}), rv(-0.9f, 0.1f, 1, {0, 1, 1, 1}),
                                           rv(-0.2f, 0.9f, 1, {0.75f, 0.125f, 0.5f, 1})};
    std::vector<RasterVertex> oracle = gpu;
    for (uint32_t i = 0; i < oracle.size(); ++i) {
        oracle[i].color[0] = gpu[(i / 3) * 3 + 2].color[0];
    }
    abi::ShaderKey key = shader::clip_space_key(state, PrimitiveKind::triangles);
    key.smooth = 0;
    return clip_case("flat triangles (provoking last)", state, PrimitiveKind::triangles, oracle, gpu, key, nullptr,
                     1000);
}

// 8x8 RGBA test texture with varied colours and alphas.
std::vector<uint8_t> make_texture() {
    std::vector<uint8_t> t(8 * 8 * 4);
    for (uint32_t y = 0; y < 8; ++y) {
        for (uint32_t x = 0; x < 8; ++x) {
            uint8_t* p = &t[(y * 8 + x) * 4];
            p[0] = static_cast<uint8_t>(x * 36);
            p[1] = static_cast<uint8_t>(y * 36);
            p[2] = static_cast<uint8_t>(((x ^ y) & 1) ? 230 : 40);
            p[3] = static_cast<uint8_t>(64 + (x + y) * 12);
        }
    }
    return t;
}

bool case_textured_quad(const char* name, gl::Enum env, gl::Enum filter, gl::Enum wrap, gl::Enum base) {
    static std::vector<uint8_t> texels = make_texture();
    Texture texture{};
    texture.target = gl::texture_2d;
    texture.levels[0].width = 8;
    texture.levels[0].height = 8;
    texture.levels[0].base_format = base;
    texture.levels[0].texels = texels.data();
    texture.sampler.min_filter = filter;
    texture.sampler.mag_filter = filter;
    texture.sampler.wrap_s = wrap;
    texture.sampler.wrap_t = wrap;
    DrawState state = base_state();
    state.texture = &texture;
    state.texture_env_mode = env;
    state.texture_env_color = {0.2f, 0.6f, 0.9f, 0.5f};
    const float lo = wrap == gl::repeat ? -0.3f : -0.2f;
    const float hi = wrap == gl::repeat ? 1.4f : 1.15f;
    const RasterVertex a = rv(-0.75f, -0.75f, 1, {1, 0.5f, 0.25f, 0.75f}, {lo, lo, 0, 1});
    const RasterVertex b = rv(0.75f, -0.75f, 1, {0.5f, 1, 0.5f, 1}, {hi, lo, 0, 1});
    const RasterVertex c = rv(0.75f, 0.75f, 1, {0.25f, 0.5f, 1, 0.5f}, {hi, hi, 0, 1});
    const RasterVertex d = rv(-0.75f, 0.75f, 1, {1, 1, 1, 1}, {lo, hi, 0, 1});
    const std::vector<RasterVertex> v = {a, b, c, a, c, d};
    TextureImage image{};
    image.width = 8;
    image.height = 8;
    image.rgba = texels.data();
    image.clamp_s = image.clamp_t = wrap == gl::repeat ? abi::tex_clamp_wrap : abi::tex_clamp_last_texel;
    image.filter = filter == gl::nearest ? abi::tex_filter_point : abi::tex_filter_bilinear;
    return clip_case(name, state, PrimitiveKind::triangles, v, v,
                     shader::clip_space_key(state, PrimitiveKind::triangles), &image, 2000);
}

bool case_projective_texture() {
    static std::vector<uint8_t> texels = make_texture();
    Texture texture{};
    texture.levels[0].width = 8;
    texture.levels[0].height = 8;
    texture.levels[0].base_format = gl::rgba;
    texture.levels[0].texels = texels.data();
    texture.sampler.min_filter = gl::linear;
    texture.sampler.mag_filter = gl::linear;
    DrawState state = base_state();
    state.texture = &texture;
    state.texture_env_mode = gl::modulate;
    const std::vector<RasterVertex> v = {rv(-0.8f, -0.8f, 1, {1, 1, 1, 1}, {0, 0, 0, 1}),
                                         rv(0.8f, -0.8f, 2, {1, 1, 1, 1}, {1.6f, 0, 0, 2}),
                                         rv(0.0f, 0.8f, 1.5f, {1, 1, 1, 1}, {0.4f, 1.2f, 0, 1.5f})};
    TextureImage image{};
    image.width = 8;
    image.height = 8;
    image.rgba = texels.data();
    image.filter = abi::tex_filter_bilinear;
    return clip_case("projective texcoord (s/q, t/q)", state, PrimitiveKind::triangles, v, v,
                     shader::clip_space_key(state, PrimitiveKind::triangles), &image, 800);
}

bool case_fog_clip() {
    DrawState state = base_state();
    state.fog_enabled = true;
    state.fog_color = {0.9f, 0.8f, 0.1f, 1};
    const std::vector<RasterVertex> v = {rv(-0.8f, -0.8f, 1, {0, 0, 1, 1}, {0, 0, 0, 1}, 0.1f),
                                         rv(0.8f, -0.8f, 1, {0, 1, 0, 1}, {0, 0, 0, 1}, 0.6f),
                                         rv(0.0f, 0.8f, 1, {1, 0, 0, 1}, {0, 0, 0, 1}, 1.0f)};
    return clip_case("fog blend (clip-space factor)", state, PrimitiveKind::triangles, v, v,
                     shader::clip_space_key(state, PrimitiveKind::triangles), nullptr, 1000);
}

bool case_alpha(const char* name, gl::Enum func, float ref) {
    DrawState state = base_state();
    state.alpha_test = true;
    state.alpha_func = func;
    state.alpha_ref = ref;
    // Left triangle: alpha gradient left to right; right triangle: constant alpha 0.
    const std::vector<RasterVertex> v = {rv(-1.0f, -0.9f, 1, {1, 0, 0, 0.0f}), rv(0.0f, -0.9f, 1, {0, 1, 0, 1.0f}),
                                         rv(-1.0f, 0.9f, 1, {0, 0, 1, 0.0f}), rv(0.1f, -0.9f, 1, {1, 1, 0, 0}),
                                         rv(0.9f, -0.9f, 1, {1, 1, 0, 0}), rv(0.5f, 0.9f, 1, {1, 1, 0, 0})};
    return clip_case(name, state, PrimitiveKind::triangles, v, v,
                     shader::clip_space_key(state, PrimitiveKind::triangles), nullptr, 0);
}

bool case_points() {
    DrawState state = base_state();
    state.point_size = 3;
    std::vector<RasterVertex> v;
    for (uint32_t i = 0; i < 40; ++i) {
        const float x = -0.9f + 0.045f * static_cast<float>(i);
        const float y = -0.8f + 0.04f * static_cast<float>((i * 7) % 40);
        v.push_back(rv(x, y, 1, {static_cast<float>(i) / 40.0f, 1 - static_cast<float>(i) / 40.0f, 0.5f, 1}));
    }
    return clip_case("points (size 3, 2 subgroups)", state, PrimitiveKind::points, v, v,
                     shader::clip_space_key(state, PrimitiveKind::points), nullptr, 150);
}

bool case_lines() {
    const DrawState state = base_state();
    std::vector<RasterVertex> v;
    for (uint32_t i = 0; i < 12; ++i) {
        const float t = static_cast<float>(i) / 12.0f;
        v.push_back(rv(-0.9f, -0.9f + 1.5f * t, 1, {1, t, 0, 1}));
        v.push_back(rv(0.8f - 0.3f * t, 0.9f - 1.6f * t, 1, {0, 1 - t, 1, 1}));
    }
    return clip_case("lines (smooth, 12 segments)", state, PrimitiveKind::lines, v, v,
                     shader::clip_space_key(state, PrimitiveKind::lines), nullptr, 300);
}

// Triangles of both windings with distinct front (color[0]) and back (color[1])
// colours; returns the vertex list.
std::vector<RasterVertex> mixed_winding() {
    std::vector<RasterVertex> v;
    for (uint32_t i = 0; i < 8; ++i) {
        const float x = -0.95f + 0.48f * static_cast<float>(i % 4);
        const float y = -0.9f + 0.95f * static_cast<float>(i / 4);
        const float k = static_cast<float>(i) / 8.0f;
        RasterVertex a = rv(x, y, 1, {1, k, 0, 1});
        RasterVertex b = rv(x + 0.42f, y + 0.1f, 1, {0.5f, 1, k, 1});
        RasterVertex c = rv(x + 0.1f, y + 0.8f, 1, {k, 0.25f, 1, 1});
        a.color[1] = {0, 0.5f, 1 - k, 0.5f};
        b.color[1] = {0.2f, k, 0.9f, 0.75f};
        c.color[1] = {1, 1 - k, 0.3f, 1};
        if (i & 1u) {
            v.push_back(a); v.push_back(c); v.push_back(b);   // clockwise
        } else {
            v.push_back(a); v.push_back(b); v.push_back(c);   // counter-clockwise
        }
    }
    return v;
}

bool case_two_sided_clip(bool front_ccw) {
    DrawState state = base_state();
    state.two_sided_color = true;
    state.front_ccw = front_ccw;
    const std::vector<RasterVertex> v = mixed_winding();
    const abi::ShaderKey key = shader::clip_space_key(state, PrimitiveKind::triangles);
    return key.two_sided == 1 &&
           clip_case(front_ccw ? "two-sided clip, front CCW" : "two-sided clip, front CW", state,
                     PrimitiveKind::triangles, v, v, key, nullptr, 1000);
}

bool case_cull(gl::Enum mode, bool front_ccw) {
    DrawState state = base_state();
    state.cull_enabled = true;
    state.cull_mode = mode;
    state.front_ccw = front_ccw;
    state.two_sided_color = true;
    const std::vector<RasterVertex> v = mixed_winding();
    static char names[6][48];
    const uint32_t slot = (mode == gl::front ? 0u : mode == gl::back ? 1u : 2u) * 2u + (front_ccw ? 0u : 1u);
    std::snprintf(names[slot], sizeof(names[slot]), "cull %s, front %s",
                  mode == gl::front ? "front" : mode == gl::back ? "back" : "front_and_back", front_ccw ? "CCW" : "CW");
    // front_and_back draws nothing; the others draw only one winding.
    return clip_case(names[slot], state, PrimitiveKind::triangles, v, v,
                     shader::clip_space_key(state, PrimitiveKind::triangles), nullptr,
                     mode == gl::front_and_back ? 0u : 400u);
}

bool case_many_triangles() {
    // 40 separate triangles: 120 unique vertices split over four NGG subgroups.
    const DrawState state = base_state();
    std::vector<RasterVertex> v;
    for (uint32_t i = 0; i < 40; ++i) {
        const float x = -0.95f + 0.19f * static_cast<float>(i % 10);
        const float y = -0.95f + 0.45f * static_cast<float>(i / 10);
        const float k = static_cast<float>(i) / 40.0f;
        v.push_back(rv(x, y, 1, {k, 0.2f, 1 - k, 1}));
        v.push_back(rv(x + 0.17f, y + 0.05f, 1, {0.3f, k, 0.6f, 1}));
        v.push_back(rv(x + 0.05f, y + 0.4f, 1, {1, 1 - k, k, 1}));
    }
    return clip_case("40 triangles (4 subgroups)", state, PrimitiveKind::triangles, v, v,
                     shader::clip_space_key(state, PrimitiveKind::triangles), nullptr, 1200);
}

// ---------------------------------------------------------------------------
// Object-space cases (gl_context over SoftwareBackend as the oracle).
// ---------------------------------------------------------------------------

struct ObjectVertex final {
    float position[4];
    float normal[3];
    float color[4];
    float texcoord[4];
};

struct ContextOracle final {
    std::vector<uint8_t> storage;
    SoftwareBackend backend;
    std::unique_ptr<Context> context;

    ContextOracle() : storage(raster_storage_bytes(fb_width, fb_height, true)),
                      backend(storage.data(), storage.size(), FramebufferFormat{true, 24, 8}),
                      context(new Context(&backend, default_allocator())) {
        backend.resize(fb_width, fb_height);
        context->viewport(0, 0, fb_width, fb_height);
        context->clear_color(static_cast<float>((clear_value >> 16) & 0xff) / 255.0f,
                             static_cast<float>((clear_value >> 8) & 0xff) / 255.0f,
                             static_cast<float>(clear_value & 0xff) / 255.0f,
                             static_cast<float>(clear_value >> 24) / 255.0f);
        context->clear(gl::color_buffer_bit);
    }

    void draw(const std::vector<ObjectVertex>& vertices) {
        context->begin(gl::triangles);
        for (const ObjectVertex& v : vertices) {
            context->color(v.color[0], v.color[1], v.color[2], v.color[3]);
            context->normal(v.normal[0], v.normal[1], v.normal[2]);
            context->tex_coord(v.texcoord[0], v.texcoord[1], v.texcoord[2], v.texcoord[3]);
            context->vertex(v.position[0], v.position[1], v.position[2], v.position[3]);
        }
        context->end();
    }
};

std::vector<uint8_t> object_records(const std::vector<ObjectVertex>& vertices) {
    std::vector<uint8_t> out(vertices.size() * abi::object_record_stride, 0);

    for (size_t i = 0; i < vertices.size(); ++i) {
        const ObjectVertex& v = vertices[i];
        const float values[16] = {v.position[0], v.position[1], v.position[2], v.position[3],
                                  v.normal[0], v.normal[1], v.normal[2], 0,
                                  v.color[0], v.color[1], v.color[2], v.color[3],
                                  v.texcoord[0], v.texcoord[1], v.texcoord[2], v.texcoord[3]};
        std::memcpy(out.data() + i * abi::object_record_stride, values, sizeof(values));
    }

    return out;
}

// Column-major matrices given to both glLoadMatrix and the VertexConstants.
void set_matrix(Context& context, gl::Enum mode, const float m[16], float out[16]) {
    double d[16];
    for (uint32_t i = 0; i < 16; ++i) {
        d[i] = m[i];
        out[i] = m[i];
    }
    context.matrix_mode(mode);
    context.load_matrix(d);
}

void fill_normal_matrix(const float modelview[16], float out[12]) {
    Matrix4 m{};
    Matrix4 inverse{};
    std::memcpy(m.values, modelview, sizeof(m.values));
    if (!matrix_invert(m, inverse)) {
        inverse = matrix_identity();
    }
    for (uint32_t r = 0; r < 3; ++r) {
        out[r * 4 + 0] = inverse.values[r * 4 + 0];
        out[r * 4 + 1] = inverse.values[r * 4 + 1];
        out[r * 4 + 2] = inverse.values[r * 4 + 2];
        out[r * 4 + 3] = 0;
    }
}

const float identity[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

struct LightSetup final {
    float ambient[4], diffuse[4], specular[4], position[4];
    float spot_direction[3];
    float spot_exponent, spot_cutoff;
    float attenuation[3];
};

// Sets light i on the context (with modelview = identity, so eye = given) and the
// matching VertexConstants entry; returns the key bits.
void set_light(Context& context, abi::VertexConstants& vc, abi::ShaderKey& key, uint32_t i, const LightSetup& l) {
    const gl::Enum light = gl::light0 + i;
    context.enable(light, true);
    context.light(light, gl::ambient, l.ambient);
    context.light(light, gl::diffuse, l.diffuse);
    context.light(light, gl::specular, l.specular);
    context.light(light, gl::position, l.position);
    context.light(light, gl::spot_direction, l.spot_direction);
    context.light(light, gl::spot_exponent, &l.spot_exponent);
    context.light(light, gl::spot_cutoff, &l.spot_cutoff);
    context.light(light, gl::constant_attenuation, &l.attenuation[0]);
    context.light(light, gl::linear_attenuation, &l.attenuation[1]);
    context.light(light, gl::quadratic_attenuation, &l.attenuation[2]);
    abi::LightConstants& c = vc.lights[i];
    for (uint32_t k = 0; k < 4; ++k) {
        c.ambient[k] = l.ambient[k];
        c.diffuse[k] = l.diffuse[k];
        c.specular[k] = l.specular[k];
        c.position[k] = l.position[k];
    }
    c.spot[0] = l.spot_direction[0];
    c.spot[1] = l.spot_direction[1];
    c.spot[2] = l.spot_direction[2];
    c.spot[3] = l.spot_exponent;
    c.attenuation[0] = l.attenuation[0];
    c.attenuation[1] = l.attenuation[1];
    c.attenuation[2] = l.attenuation[2];
    // gl_context: std::cos(spot_cutoff * 0.017453292519943295f) in float.
    c.attenuation[3] = std::cos(l.spot_cutoff * 0.017453292519943295f);
    key.light_mask |= static_cast<uint8_t>(1u << i);
    if (l.position[3] != 0) {
        key.positional_mask |= static_cast<uint8_t>(1u << i);
    }
    if (l.spot_cutoff != 180) {
        key.spot_mask |= static_cast<uint8_t>(1u << i);
    }
}

void set_material(Context& context, abi::MaterialConstants& m, gl::Enum face, const float emission[4],
                  const float ambient[4], const float diffuse[4], const float specular[4], float shininess) {
    context.material(face, gl::emission, emission);
    context.material(face, gl::ambient, ambient);
    context.material(face, gl::diffuse, diffuse);
    context.material(face, gl::specular, specular);
    const float s4[4] = {shininess, 0, 0, 0};
    context.material(face, gl::shininess, s4);
    for (uint32_t k = 0; k < 4; ++k) {
        m.emission[k] = emission[k];
        m.ambient[k] = ambient[k];
        m.diffuse[k] = diffuse[k];
        m.specular[k] = specular[k];
        m.shininess[k] = k == 0 ? shininess : 0;
    }
}

// A tessellated, gently curved patch with per-vertex normals (front facing).
std::vector<ObjectVertex> patch(float z_base, bool textured) {
    std::vector<ObjectVertex> out;
    const uint32_t n = 4;
    auto vertex = [&](uint32_t i, uint32_t j) {
        const float u = static_cast<float>(i) / n;
        const float w = static_cast<float>(j) / n;
        ObjectVertex v{};
        v.position[0] = -1.5f + 3.0f * u;
        v.position[1] = -1.5f + 3.0f * w;
        v.position[2] = z_base - 0.5f * (u - 0.5f) * (u - 0.5f);
        v.position[3] = 1;
        v.normal[0] = (u - 0.5f) * 0.8f;
        v.normal[1] = (w - 0.5f) * 0.6f;
        v.normal[2] = 1;
        v.color[0] = 0.25f + 0.5f * u;
        v.color[1] = 0.75f - 0.5f * w;
        v.color[2] = 0.5f;
        v.color[3] = 1;
        v.texcoord[0] = textured ? u * 1.25f : 0;
        v.texcoord[1] = textured ? w : 0;
        v.texcoord[2] = 0;
        v.texcoord[3] = 1;
        return v;
    };
    for (uint32_t j = 0; j < n; ++j) {
        for (uint32_t i = 0; i < n; ++i) {
            out.push_back(vertex(i, j));
            out.push_back(vertex(i + 1, j));
            out.push_back(vertex(i + 1, j + 1));
            out.push_back(vertex(i, j));
            out.push_back(vertex(i + 1, j + 1));
            out.push_back(vertex(i, j + 1));
        }
    }
    return out;
}

// Common object-space scene: perspective projection, translated modelview.
void scene_matrices(Context& context, abi::VertexConstants& vc) {
    const float n = 1, f = 10, r = 0.5f, t = 0.5f;
    const float projection[16] = {n / r, 0, 0, 0, 0, n / t, 0, 0, 0, 0, -(f + n) / (f - n), -1,
                                  0, 0, -2 * f * n / (f - n), 0};
    const float modelview[16] = {0.96f, 0.0f, -0.28f, 0, 0.0f, 1, 0, 0, 0.28f, 0.0f, 0.96f, 0, 0.1f, -0.05f, -4.5f, 1};
    set_matrix(context, gl::projection, projection, vc.projection);
    set_matrix(context, gl::texture, identity, vc.texture_matrix);
    set_matrix(context, gl::modelview, modelview, vc.modelview);
    fill_normal_matrix(vc.modelview, vc.normal_matrix);
}

bool object_case(const char* name, ContextOracle& oracle, const abi::VertexConstants& vc, abi::ShaderKey key,
                 const std::vector<ObjectVertex>& vertices, const TextureImage* texture, const FragmentConstants& fc,
                 uint32_t min_drawn) {
    oracle.draw(vertices);
    key.vertex_input = abi::vertex_input_object_space;
    GpuDraw draw{};
    draw.key = key;
    draw.records = object_records(vertices);
    draw.vertex_count = static_cast<uint32_t>(vertices.size());
    draw.constants = &vc;
    draw.texture = texture;
    draw.fragment = fc;
    std::vector<uint32_t> model;
    const auto stats = run_gpu(draw, model, clear_value);
    return matches(name, oracle.backend.color_buffer(gl::back), model, stats, min_drawn);
}

bool case_lit(bool two_sided, uint8_t color_material) {
    ContextOracle oracle;
    Context& context = *oracle.context;
    abi::VertexConstants vc{};
    abi::ShaderKey key{};
    scene_matrices(context, vc);
    context.matrix_mode(gl::modelview);
    context.push_matrix();
    context.load_identity();                         // light positions in eye space
    context.enable(gl::lighting, true);
    key.lighting = 1;
    const float lma[4] = {0.15f, 0.1f, 0.05f, 1};
    context.light_model(gl::light_model_ambient, lma);
    std::memcpy(vc.light_model_ambient, lma, sizeof(lma));
    if (two_sided) {
        const float one = 1;
        context.light_model(gl::light_model_two_side, &one);
        key.two_sided = 1;
    }
    const LightSetup directional{{0.1f, 0.1f, 0.1f, 1}, {0.7f, 0.6f, 0.5f, 1}, {0.8f, 0.8f, 0.8f, 1},
                                 {0.3f, 0.4f, 0.866f, 0}, {0, 0, -1}, 0, 180, {1, 0, 0}};
    const LightSetup positional{{0.05f, 0.0f, 0.1f, 1}, {0.2f, 0.5f, 0.9f, 1}, {0.5f, 0.6f, 0.3f, 1},
                                {-1.5f, 1.0f, -2.0f, 1}, {0, 0, -1}, 0, 180, {0.6f, 0.15f, 0.03f}};
    const LightSetup spot{{0.0f, 0.0f, 0.0f, 1}, {0.9f, 0.3f, 0.2f, 1}, {0.4f, 0.4f, 0.4f, 1},
                          {1.0f, -0.5f, -1.0f, 1}, {-0.25f, 0.1f, -1.0f}, 6.0f, 35.0f, {1, 0.05f, 0}};
    set_light(context, vc, key, 0, directional);
    set_light(context, vc, key, 2, positional);
    set_light(context, vc, key, 5, spot);
    context.pop_matrix();
    const float em[4] = {0.02f, 0.02f, 0.05f, 1};
    const float am[4] = {0.3f, 0.25f, 0.2f, 1};
    const float dm[4] = {0.8f, 0.7f, 0.6f, 0.8f};
    const float sm[4] = {0.6f, 0.6f, 0.6f, 1};
    set_material(context, vc.front, gl::front, em, am, dm, sm, 24.0f);
    const float bdm[4] = {0.2f, 0.9f, 0.3f, 1};
    set_material(context, vc.back, gl::back, em, am, bdm, sm, 4.0f);
    if (color_material != abi::color_material_none) {
        context.color_material(gl::front_and_back, color_material == abi::color_material_ambient_and_diffuse
                                                        ? gl::ambient_and_diffuse
                                                        : (color_material == abi::color_material_ambient ? gl::ambient
                                                                                                         : gl::diffuse));
        context.enable(gl::color_material, true);
        key.color_material = color_material;
    }
    context.enable(gl::normalize, true);
    key.normalize = 1;
    const char* name = two_sided ? "lit, two-sided (both windings)"
                       : color_material != abi::color_material_none ? "lit, color material" : "lit, 3 lights";
    std::vector<ObjectVertex> vertices = patch(-0.2f, false);
    if (two_sided) {
        // Reverse every other triangle so half the patch faces away (back material,
        // negated normal through gl_raster color[1]).
        for (size_t t = 3; t + 2 < vertices.size(); t += 6) {
            const ObjectVertex swap = vertices[t + 1];
            vertices[t + 1] = vertices[t + 2];
            vertices[t + 2] = swap;
        }
    }
    return object_case(name, oracle, vc, key, vertices, nullptr, {}, 1200);
}

bool case_fog_object(gl::Enum mode) {
    ContextOracle oracle;
    Context& context = *oracle.context;
    abi::VertexConstants vc{};
    abi::ShaderKey key{};
    scene_matrices(context, vc);
    const float color[4] = {0.7f, 0.75f, 0.8f, 1};
    const float m = static_cast<float>(mode);
    const float start = 2.0f, end = 6.5f, density = 0.35f;
    context.enable(gl::fog, true);
    context.fog(gl::fog_mode, &m);
    context.fog(gl::fog_start, &start);
    context.fog(gl::fog_end, &end);
    context.fog(gl::fog_density, &density);
    context.fog(gl::fog_color, color);
    key.fog = 1;
    const float log2e = 1.44269504088896340736f;
    if (mode == gl::linear) {
        key.fog_mode = abi::fog_linear;
        vc.fog.scale = -1.0f / (end - start);
        vc.fog.bias = end / (end - start);
    } else if (mode == gl::exp) {
        key.fog_mode = abi::fog_exp;
        vc.fog.exp_scale = -density * log2e;
    } else {
        key.fog_mode = abi::fog_exp2;
        vc.fog.density = density;
        vc.fog.exp_scale = -log2e;
    }
    FragmentConstants fc{};
    std::memcpy(fc.fog_color, color, sizeof(color));
    const char* name = mode == gl::linear ? "fog linear (object space)"
                       : mode == gl::exp  ? "fog exp (object space)"
                                          : "fog exp2 (object space)";
    // Two patches at different depths for a visible fog range.
    std::vector<ObjectVertex> vertices = patch(0.5f, false);
    std::vector<ObjectVertex> far = patch(-2.5f, false);
    for (ObjectVertex& v : far) {
        v.position[0] = v.position[0] * 0.6f + 0.9f;
    }
    vertices.insert(vertices.end(), far.begin(), far.end());
    return object_case(name, oracle, vc, key, vertices, nullptr, fc, 1200);
}

bool case_flat_object() {
    ContextOracle oracle;
    Context& context = *oracle.context;
    abi::VertexConstants vc{};
    abi::ShaderKey key{};
    scene_matrices(context, vc);
    context.shade_model(gl::flat);
    key.smooth = 0;
    return object_case("flat shade model (object space)", oracle, vc, key, patch(0.0f, false), nullptr, {}, 1200);
}

bool case_textured_object() {
    static std::vector<uint8_t> texels = make_texture();
    ContextOracle oracle;
    Context& context = *oracle.context;
    abi::VertexConstants vc{};
    abi::ShaderKey key{};
    scene_matrices(context, vc);
    // Texture matrix: scale and translate.
    const float tm[16] = {1.5f, 0, 0, 0, 0, 0.75f, 0, 0, 0, 0, 1, 0, 0.125f, 0.25f, 0, 1};
    set_matrix(context, gl::texture, tm, vc.texture_matrix);
    context.matrix_mode(gl::modelview);
    uint32_t name = 0;
    context.gen_textures(1, &name);
    context.bind_texture(gl::texture_2d, name);
    const float linear = static_cast<float>(gl::linear);
    context.tex_parameter(gl::texture_2d, gl::texture_min_filter, &linear);
    context.tex_parameter(gl::texture_2d, gl::texture_mag_filter, &linear);
    context.tex_image_2d(gl::texture_2d, 0, gl::rgba, 8, 8, 0, gl::rgba, gl::type_unsigned_byte, texels.data());
    context.enable(gl::texture_2d, true);
    const float env = static_cast<float>(gl::modulate);
    context.tex_env(gl::texture_env, gl::texture_env_mode, &env);
    key.textured = 1;
    key.tex_env = abi::tex_env_modulate;
    key.texture_base = abi::texture_base_rgba;
    TextureImage image{};
    image.width = 8;
    image.height = 8;
    image.rgba = texels.data();
    image.filter = abi::tex_filter_bilinear;
    return object_case("textured + texture matrix (object)", oracle, vc, key, patch(0.0f, true), &image, {}, 1200);
}

// ---------------------------------------------------------------------------
// Case table and mutations.
// ---------------------------------------------------------------------------

bool run_all(bool stop_on_failure) {
    bool all = true;
    auto step = [&](bool ok, const char* contract) {
        if (!quiet) {
            expect(ok, contract);
        }
        all = all && ok;
        return !stop_on_failure || all;
    };

    if (!step(case_flat_triangle(), "flat triangle matches gl_raster")) return all;
    if (!step(case_smooth_triangle(), "smooth triangle matches gl_raster")) return all;
    if (!step(case_perspective_triangle(), "perspective-correct smooth triangle matches gl_raster")) return all;
    if (!step(case_many_triangles(), "multi-subgroup triangle draw matches gl_raster")) return all;
    if (!step(case_fog_clip(), "clip-space fog blend matches gl_raster")) return all;

    const struct {
        const char* name;
        gl::Enum env, filter, wrap, base;
    } textured[] = {
        {"modulate nearest repeat rgba", gl::modulate, gl::nearest, gl::repeat, gl::rgba},
        {"modulate linear repeat rgba", gl::modulate, gl::linear, gl::repeat, gl::rgba},
        {"decal nearest clamp rgba", gl::decal, gl::nearest, gl::clamp, gl::rgba},
        {"decal linear repeat rgba", gl::decal, gl::linear, gl::repeat, gl::rgba},
        {"blend nearest repeat rgba", gl::blend, gl::nearest, gl::repeat, gl::rgba},
        {"blend linear clamp rgba", gl::blend, gl::linear, gl::clamp, gl::rgba},
        {"replace nearest repeat rgba", gl::replace, gl::nearest, gl::repeat, gl::rgba},
        {"replace linear repeat rgba", gl::replace, gl::linear, gl::repeat, gl::rgba},
        {"replace nearest rgb", gl::replace, gl::nearest, gl::repeat, gl::rgb},
        {"replace linear alpha", gl::replace, gl::linear, gl::repeat, gl::alpha},
        {"decal nearest rgb", gl::decal, gl::nearest, gl::repeat, gl::rgb},
        {"modulate nearest luminance", gl::modulate, gl::nearest, gl::repeat, gl::luminance},
        {"blend linear luminance_alpha", gl::blend, gl::linear, gl::repeat, gl::luminance_alpha},
    };

    for (const auto& t : textured) {
        if (!step(case_textured_quad(t.name, t.env, t.filter, t.wrap, t.base), "textured quad matches gl_raster")) {
            return all;
        }
    }

    if (!step(case_projective_texture(), "projective texture coordinates match gl_raster")) return all;

    const struct {
        const char* name;
        gl::Enum func;
        float ref;
    } alpha[] = {
        {"alpha test less 0.5", gl::less, 0.5f},        {"alpha test greater 0.5", gl::greater, 0.5f},
        {"alpha test lequal 0.25", gl::lequal, 0.25f},  {"alpha test gequal 0.75", gl::gequal, 0.75f},
        {"alpha test equal 0", gl::equal, 0.0f},        {"alpha test notequal 0", gl::notequal, 0.0f},
        {"alpha test never", gl::never, 0.5f},          {"alpha test always", gl::always, 0.5f},
    };

    for (const auto& t : alpha) {
        if (!step(case_alpha(t.name, t.func, t.ref), "alpha test matches gl_raster")) {
            return all;
        }
    }

    if (!step(case_points(), "points match gl_raster")) return all;
    if (!step(case_lines(), "lines match gl_raster")) return all;
    if (!step(case_lit(false, abi::color_material_none), "lit geometry matches gl_context")) return all;
    if (!step(case_lit(false, abi::color_material_ambient_and_diffuse), "color-material lighting matches gl_context")) return all;
    if (!step(case_lit(true, abi::color_material_none), "two-sided lighting (both windings) matches gl_context")) return all;
    if (!step(case_two_sided_clip(true), "two-sided clip-space back faces match gl_raster (front CCW)")) return all;
    if (!step(case_two_sided_clip(false), "two-sided clip-space back faces match gl_raster (front CW)")) return all;
    {
        const gl::Enum modes[3] = {gl::front, gl::back, gl::front_and_back};
        for (gl::Enum mode : modes) {
            for (uint32_t ccw = 0; ccw < 2; ++ccw) {
                if (!step(case_cull(mode, ccw == 0), "culling matches gl_raster")) return all;
            }
        }
    }
    if (!step(case_fog_object(gl::linear), "linear fog matches gl_context")) return all;
    if (!step(case_fog_object(gl::exp), "exp fog matches gl_context")) return all;
    if (!step(case_fog_object(gl::exp2), "exp2 fog matches gl_context")) return all;
    if (!step(case_flat_object(), "flat shade model matches gl_context")) return all;
    if (!step(case_textured_object(), "texture matrix + texturing matches gl_context")) return all;
    return all;
}

// Mutations: each must make at least one case fail (model error or pixels). The
// match patterns are assembled with the same encoder so they track the generator.
namespace isa = rx7900::hardware::gfx11_isa;

template <typename Emit>
uint32_t encoded(Emit emit, uint32_t index) {
    uint8_t buffer[16] = {};
    isa::Assembler a(sdma::CommandBuffer(buffer, sizeof(buffer)));
    emit(a);
    uint32_t value = 0;
    std::memcpy(&value, buffer + index * 4, 4);
    return value;
}

std::vector<Mutation> build_mutations() {
    using namespace isa;
    std::vector<Mutation> m;
    const uint32_t full = 0xffffffffu;
    // VS: GS_ALLOC_REQ m0 = prims << 12 becomes << 13.
    m.push_back({"GS_ALLOC_REQ primitive shift", Stage::vertex, full,
                 encoded([](Assembler& a) { a.sop2(Sop2::s_lshl_b32, s(24), s(23), imm(12)); }, 0), 0, 0,
                 (140u ^ 141u) << 8});
    // VS: primitive export vertex-1 field at bit 10 becomes bit 11.
    m.push_back({"prim export index packing", Stage::vertex, full,
                 encoded([](Assembler& a) { a.vop3(Vop3::v_lshl_or_b32, v(13), v(14), imm(10), v(13)); }, 0), 0,
                 1, (138u ^ 139u) << 9});
    // VS: attribute-ring wave offset << 9 becomes << 8.
    m.push_back({"attribute ring wave offset", Stage::vertex, full,
                 encoded([](Assembler& a) { a.sop2(Sop2::s_lshl_b32, s(27), s(27), imm(9)); }, 0), 0, 0,
                 (137u ^ 136u) << 8});
    // VS: first attribute-ring store moves to the next slot (offset + 16).
    m.push_back({"attribute ring slot offset", Stage::vertex, 0xfffc0000u, 0xe0740000u, 0, 0, 0x10u});
    // VS: first vertex-record fetch reads the next field.
    m.push_back({"vertex fetch offset", Stage::vertex, 0xfffc0000u, 0xe05c0000u, 0, 0, 0x10u});
    // VS: pos0 export retargeted to pos1.
    m.push_back({"pos0 export target", Stage::vertex, 0xffffff0fu,
                 encoded([](Assembler& a) { a.exp(ExpTarget::pos0, v(1), v(2), v(3), v(4), true); }, 0) & 0xffffff0fu,
                 0, 0, 0x10u});
    // VS: the specular half-vector normalisation (fourth v_sqrt_f32 of the lit
    // case) becomes v_rsq_f32. The earlier sqrts are equivalent mutants there:
    // GL_NORMALIZE is repeated by apply_lighting, and the already-unit normal and
    // directional-light vectors have length 1.
    m.push_back({"half-vector sqrt -> rsq", Stage::vertex, 0xfe01fe00u, 0x7e000000u | (0x33u << 9), 3, 0,
                 (0x33u ^ 0x2eu) << 9});
    // VS: pow/fog v_exp_f32 becomes v_log_f32.
    m.push_back({"exp2 -> log2", Stage::vertex, 0xfe01fe00u, 0x7e000000u | (0x25u << 9), 0, 0,
                 (0x25u ^ 0x27u) << 9});
    // VS: primitive export without done.
    m.push_back({"prim export done bit", Stage::vertex, 0xffffff0fu,
                 encoded([](Assembler& a) { a.exp(ExpTarget::prim, v(1), Operand{}, Operand{}, Operand{}, true); }, 0) &
                     0xffffff0fu, 0, 0, 0x800u});
    // PS: v_interp_p2_f32 becomes v_interp_p10_f32.
    m.push_back({"v_interp_p2 -> p10", Stage::pixel, 0xffff0000u, 0xcd010000u, 0, 0, 0x10000u});
    // PS: first lds_param_load reads channel y instead of x.
    m.push_back({"lds_param_load channel", Stage::pixel, 0xff000300u, 0xce000000u, 0, 0, 0x100u});
    // PS: s_wqm_b32 exec becomes s_mov_b32 exec (helper lanes stay off).
    m.push_back({"whole-quad mode removed", Stage::pixel, full,
                 encoded([](Assembler& a) { a.sop1(Sop1::s_wqm_b32, exec_lo(), exec_lo()); }, 0), 0, 0, 0x1c00u});
    // PS: m0 loaded from s21 instead of the primitive mask s20.
    m.push_back({"prim mask in m0", Stage::pixel, full,
                 encoded([](Assembler& a) { a.sop1(Sop1::s_mov_b32, m0(), s(20)); }, 0), 0, 0, 0x1u});
    // PS: image_sample_lz dmask 0xf becomes 0x7.
    m.push_back({"image_sample dmask", Stage::pixel, 0xfffc0000u, 0xf07c0000u, 0, 0, 0x800u});
    // PS: S# operand s8 becomes s12 (SSAMP field 2 becomes 3).
    m.push_back({"sampler descriptor operand", Stage::pixel, 0xfffc0000u, 0xf07c0000u, 0, 1, 0x1u << 26});
    // PS: two-sided front_face test inverted (v_cmp_ne_u32 becomes v_cmp_eq_u32).
    m.push_back({"front_face selection swapped", Stage::pixel, 0xffff0000u, 0xd44d0000u, 0, 0,
                 (0x4du ^ 0x4au) << 16});
    // PS: alpha compare lt becomes gt.
    m.push_back({"alpha compare op", Stage::pixel, 0xffff0000u, 0xd4110000u, 0, 0, (0x11u ^ 0x14u) << 16});
    // PS: MRT0 red and green VGPR fields swapped.
    m.push_back({"mrt0 channel order", Stage::pixel, full,
                 encoded([](Assembler& a) { a.exp(ExpTarget::mrt0, v(8), v(9), v(10), v(11), true); }, 0), 0, 1,
                 0x00000101u});
    // PS: first v_mul_f32 (texcoord divide / texenv / fog) becomes v_add_f32.
    m.push_back({"texenv/fog multiply", Stage::pixel, 0xfe000000u, 0x10000000u, 0, 0, (0x08u ^ 0x03u) << 25});
    // PS: alpha-test EXEC narrowing dropped (s_and_b32 exec -> s_or_b32 exec).
    m.push_back({"alpha kill mask", Stage::pixel, full,
                 encoded([](Assembler& a) { a.sop2(Sop2::s_and_b32, exec_lo(), s(21), s(22)); }, 0), 0, 0,
                 (0x16u ^ 0x18u) << 23});
    return m;
}

} // namespace

int main() {
    std::printf("opengl_shader_pixel cases (tolerance %u per 8-bit channel):\n", tolerance);
    run_all(false);

    // Mutation evidence: every mutant must be killed by at least one case.
    quiet = true;
    unsigned killed = 0;

    const std::vector<Mutation> mutations = build_mutations();

    for (const Mutation& mutation : mutations) {
        active_mutation = &mutation;
        mutation_applied = false;
        const bool survived = run_all(true);
        const bool applied = mutation_applied;
        std::printf("  mutant %-32s %s%s\n", mutation.name, survived ? "SURVIVED" : "killed",
                    applied ? "" : " (pattern not found)");
        ++checks;
        if (survived || !applied) {
            ++failures;
            std::fprintf(stderr, "FAIL: mutant %s not killed\n", mutation.name);
        } else {
            ++killed;
        }
    }

    active_mutation = nullptr;
    std::printf("opengl_shader_pixel: %u checks, %u failures, %u/%zu mutants killed\n", checks, failures, killed,
                mutations.size());
    return failures == 0 ? 0 : 1;
}

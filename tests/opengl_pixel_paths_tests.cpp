/*
Module: rx7900.tests.opengl_pixel_paths
Purpose: The GFX11 paths that used to migrate to software: line and polygon
stipple, GL_FRONT_AND_BACK, DrawPixels/Bitmap/CopyPixels (raster position,
pixel transfer, zoom, depth and stencil images, staging of overlapping copies)
and mipmapped sampling. Each scene runs the same Context commands on gl_raster
and on Gfx11Backend over the production kernel modules (submission layout 4,
kernel-built PS constants, kernel shader generation) and the PM4 pipeline
model, and compares both colour buffers, depth and stencil. Generator
mutations (patched program words) must make the matching scene fail.
Created: 2026-10-24
Architecture: Host ASan/UBSan test over tests/gfx11_graphics_device.hpp. Not
hardware evidence: CP, SPI, PS and CB/DB are models.
*/
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

#include "asic_test_device.hpp"
#include "gfx11_graphics_device.hpp"
#include "../src/opengl/gl_context.hpp"

namespace {

using namespace rx7900::opengl;
namespace abi = rx7900::opengl::gfx11_abi;
using gfx11_graphics_device::Device;

rx7900_test::Checks checks;
bool quiet = false;
void expect(bool value, const std::string& what) {
    if (quiet) {
        ++checks.total;
        checks.failed += value ? 0u : 1u;
        return;
    }
    checks.expect(value, what.c_str());
}

constexpr uint32_t size = 48;

gfx11_header_db::Database& database() {
    static gfx11_header_db::Database db;
    static bool loaded = db.load();
    (void)loaded;
    return db;
}

// ---------------------------------------------------------------------------
// Generator mutations: one program word of the first matching instruction.
// ---------------------------------------------------------------------------

struct Mutation final {
    const char* name;
    bool pixel;                 // pixel program (else vertex)
    uint32_t mask, value;       // first dword match
    uint32_t occurrence;
    uint32_t flip;              // XOR into the first dword
};
const Mutation* active = nullptr;
bool applied = false;

shader::GeneratorWorkspace& mutation_workspace() {
    static shader::GeneratorWorkspace workspace;
    return workspace;
}

// Generates the program and, for v4 pixel programs, applies the active mutation.
bool mutate(const abi::ShaderKey& key, rx7900::sdma::CommandBuffer out, abi::ProgramInfo& info, bool pixel) {
    auto& workspace = mutation_workspace();
    std::vector<uint8_t> bytes(abi::shader_max_bytes, 0);
    const rx7900::sdma::CommandBuffer local(bytes.data(), bytes.size());
    const auto status = pixel ? shader::generate_pixel_shader(key, local, info, workspace)
                              : shader::generate_vertex_shader(key, local, info, workspace);
    if (status != shader::Status::success) return false;
    const bool v4 = abi::uses_ps_constants(key) || key.front_and_back != 0;
    if (active != nullptr && active->pixel == pixel && v4) {
        uint32_t seen = 0;
        for (uint32_t i = 0; i < info.code_dwords; ++i) {
            uint32_t word = 0;
            std::memcpy(&word, bytes.data() + i * 4u, 4);
            if ((word & active->mask) == active->value && seen++ == active->occurrence) {
                word ^= active->flip;
                std::memcpy(bytes.data() + i * 4u, &word, 4);
                applied = true;
                break;
            }
        }
    }
    for (uint32_t i = 0; i < info.code_dwords; ++i) {
        uint32_t word = 0;
        std::memcpy(&word, bytes.data() + i * 4u, 4);
        if (!out.write_u32(i * 4u, word)) return false;
    }
    return true;
}

bool mutated_vertex(void*, const abi::ShaderKey& key, rx7900::sdma::CommandBuffer out,
                    abi::ProgramInfo& info) noexcept {
    return mutate(key, out, info, false);
}

bool mutated_pixel(void*, const abi::ShaderKey& key, rx7900::sdma::CommandBuffer out,
                   abi::ProgramInfo& info) noexcept {
    return mutate(key, out, info, true);
}

// ---------------------------------------------------------------------------
// Reference and GPU drawables.
// ---------------------------------------------------------------------------

struct Software final {
    explicit Software(uint32_t w = size, uint32_t h = size)
        : storage(raster_storage_bytes(w, h, true)),
          backend(storage.data(), storage.size(), FramebufferFormat{true, 24, 8}) {
        backend.resize(w, h);
        context.viewport(0, 0, static_cast<int32_t>(w), static_cast<int32_t>(h));
    }
    std::vector<uint8_t> storage;
    SoftwareBackend backend;
    Context context{&backend, default_allocator()};
};

struct Gpu final {
    explicit Gpu(uint32_t w = size, uint32_t h = size, uint64_t gtt = 0)
        : device(database(), 96, 80, gtt), storage(raster_storage_bytes(w, h, true)),
          fallback(storage.data(), storage.size(), FramebufferFormat{true, 24, 8}),
          scratch(gfx11_backend_scratch_bytes) {
        device.device.generator = {nullptr, mutated_vertex, mutated_pixel};
        backend = std::make_unique<Gfx11Backend>(device, scratch.data(), scratch.size(), fallback);
        backend->limit_texture_heap(1024 * 1024);
        backend->initialize();
        backend->resize(w, h);
        context = std::make_unique<Context>(backend.get(), default_allocator());
        context->viewport(0, 0, static_cast<int32_t>(w), static_cast<int32_t>(h));
    }
    Device device;
    std::vector<uint8_t> storage;
    SoftwareBackend fallback;
    std::vector<uint8_t> scratch;
    std::unique_ptr<Gfx11Backend> backend;
    std::unique_ptr<Context> context;
};

std::vector<float> read(Backend& backend, ReadSource source, gl::Enum buffer, uint32_t w = size, uint32_t h = size) {
    std::vector<float> out(static_cast<size_t>(w) * h * (source == ReadSource::color ? 4u : 1u), -1.0f);
    backend.read(source, buffer, 0, 0, w, h, out.data());
    return out;
}

unsigned differences(const std::vector<float>& a, const std::vector<float>& b, uint32_t channels, float tolerance) {
    unsigned bad = 0;
    for (size_t p = 0; p < a.size() / channels; ++p) {
        bool differs = false;
        for (uint32_t c = 0; c < channels; ++c) {
            differs = differs || !(std::fabs(a[p * channels + c] - b[p * channels + c]) <= tolerance);
        }
        if (differs && std::getenv("RX7900_DIFF") != nullptr) {
            std::fprintf(stderr, "  pixel (%zu, %zu): reference %g %g %g gpu %g %g %g\n", p % size, p / size,
                         a[p * channels], channels > 1 ? a[p * channels + 1] : 0.0f, channels > 2 ? a[p * channels + 2] : 0.0f,
                         b[p * channels], channels > 1 ? b[p * channels + 1] : 0.0f, channels > 2 ? b[p * channels + 2] : 0.0f);
        }
        bad += differs ? 1u : 0u;
    }
    return bad;
}

using Scene = std::function<void(Context&)>;

struct Outcome final {
    unsigned color = 0, depth = 0, stencil = 0, model_errors = 0;
    bool gpu = false;
    uint32_t layout4 = 0;
};

// Runs a scene on both and compares the front and back colour, depth and
// stencil buffers. budget: differing colour pixels allowed (LOD boundaries).
Outcome run(const Scene& scene, unsigned budget = 0, uint32_t w = size, uint32_t h = size, uint64_t gtt = 0,
            bool compare_depth = true) {
    Software reference(w, h);
    Gpu gpu(w, h, gtt);
    scene(reference.context);
    scene(*gpu.context);
    gpu.context->finish();
    Outcome o{};
    o.gpu = gpu.backend->gpu_active();
    o.layout4 = gpu.backend->layout4_submissions();
    o.model_errors = gpu.device.model->stats().errors + gpu.device.ib_faults;
    for (gl::Enum buffer : {gl::front, gl::back}) {
        o.color += differences(read(reference.backend, ReadSource::color, buffer, w, h),
                               read(*gpu.backend, ReadSource::color, buffer, w, h), 4, 2.5f / 255.0f);
    }
    if (compare_depth) {
        o.depth = differences(read(reference.backend, ReadSource::depth, gl::back, w, h),
                              read(*gpu.backend, ReadSource::depth, gl::back, w, h), 1, 1e-5f);
        o.stencil = differences(read(reference.backend, ReadSource::stencil, gl::back, w, h),
                                read(*gpu.backend, ReadSource::stencil, gl::back, w, h), 1, 0.0f);
    }
    if (!quiet && (o.color > budget || o.depth != 0 || o.stencil != 0 || o.model_errors != 0 || !o.gpu)) {
        std::fprintf(stderr, "  colour %u (budget %u) depth %u stencil %u model errors %u (%s) gpu %d reason %s\n",
                     o.color, budget, o.depth, o.stencil, o.model_errors,
                     gpu.device.model->stats().first_error.c_str(), o.gpu ? 1 : 0,
                     fallback_reason_text(gpu.backend->reason()));
    }
    (void)budget;
    return o;
}

// RX7900_SCENE=text runs only the scenes whose name contains text.
bool selected(const std::string& name) {
    const char* only = std::getenv("RX7900_SCENE");
    return only == nullptr || name.find(only) != std::string::npos;
}

bool scene(const std::string& name, const Scene& draw, unsigned budget = 0, bool layout4 = true) {
    if (!selected(name)) return true;
    const Outcome o = run(draw, budget);
    const bool ok = o.gpu && o.model_errors == 0 && o.color <= budget && o.depth == 0 && o.stencil == 0 &&
                    (!layout4 || o.layout4 > 0);
    if (!quiet) {
        std::printf("%-40s gpu=%d layout4=%u colour=%u depth=%u stencil=%u\n", name.c_str(), o.gpu ? 1 : 0,
                    o.layout4, o.color, o.depth, o.stencil);
    }
    expect(o.gpu && o.model_errors == 0, name + ": stays on the GPU path, IBs execute without model errors");
    expect(!layout4 || o.layout4 > 0, name + ": travels as submission layout 4");
    expect(o.color <= budget && o.depth == 0 && o.stencil == 0, name + ": colour, depth and stencil match gl_raster");
    return ok;
}

// ---------------------------------------------------------------------------
// Drawing helpers.
// ---------------------------------------------------------------------------

void quad(Context& c, float x0, float y0, float x1, float y1, float z = 0) {
    c.begin(gl::quads);
    c.tex_coord(0, 0, 0, 1); c.vertex(x0, y0, z, 1);
    c.tex_coord(1, 0, 0, 1); c.vertex(x1, y0, z, 1);
    c.tex_coord(1, 1, 0, 1); c.vertex(x1, y1, z, 1);
    c.tex_coord(0, 1, 0, 1); c.vertex(x0, y1, z, 1);
    c.end();
}

void clear(Context& c, float r, float g, float b) {
    c.clear_color(r, g, b, 1);
    c.clear_depth(1.0);
    c.clear_stencil(0);
    c.clear(gl::color_buffer_bit | gl::depth_buffer_bit | gl::stencil_buffer_bit);
}

// A window-coordinate projection: one unit is one pixel.
void pixel_projection(Context& c) {
    c.matrix_mode(gl::projection);
    c.load_identity();
    c.ortho(0, size, 0, size, -1, 1);
    c.matrix_mode(gl::modelview);
    c.load_identity();
}

void nearest(Context& c, gl::Enum min = gl::nearest, gl::Enum mag = gl::nearest) {
    const float a[1] = {static_cast<float>(min)}, b[1] = {static_cast<float>(mag)};
    c.tex_parameter(gl::texture_2d, gl::texture_min_filter, a);
    c.tex_parameter(gl::texture_2d, gl::texture_mag_filter, b);
}

// A 32x32 RGBA mip chain: level 0 a checker, then one solid colour per level.
void mip_texture(Context& c, gl::Enum min, gl::Enum mag) {
    uint32_t name = 0;
    c.gen_textures(1, &name);
    c.bind_texture(gl::texture_2d, name);
    nearest(c, min, mag);
    const uint8_t colors[6][4] = {{255, 0, 0, 255}, {0, 255, 0, 255}, {0, 0, 255, 255},
                                  {255, 255, 0, 255}, {0, 255, 255, 255}, {255, 0, 255, 255}};
    for (int32_t level = 0, dim = 32; dim >= 1; ++level, dim /= 2) {
        std::vector<uint8_t> texels(static_cast<size_t>(dim) * dim * 4);
        for (int32_t i = 0; i < dim * dim; ++i) {
            const bool odd = ((i % dim) / 4 + (i / dim) / 4) % 2 != 0;
            for (int ch = 0; ch < 4; ++ch) {
                texels[static_cast<size_t>(i) * 4 + ch] =
                    level == 0 ? static_cast<uint8_t>(odd ? 230 : 40 + ch * 30) : colors[level % 6][ch];
            }
        }
        c.tex_image_2d(gl::texture_2d, level, static_cast<int32_t>(gl::rgba), dim, dim, 0, gl::rgba,
                       gl::type_unsigned_byte, texels.data());
    }
    c.enable(gl::texture_2d, true);
}

// ---------------------------------------------------------------------------
// Scenes.
// ---------------------------------------------------------------------------

void stipple_scenes() {
    scene("line stipple: strip, lines, loop", [](Context& c) {
        clear(c, 0, 0, 0);
        c.enable(gl::line_stipple, true);
        c.line_stipple(2, 0x0f0f);
        c.color(1, 1, 0, 1);
        c.begin(gl::line_strip);
        c.vertex(-0.9, -0.9, 0, 1); c.vertex(0.8, -0.7, 0, 1); c.vertex(0.6, 0.1, 0, 1); c.vertex(-0.7, 0.3, 0, 1);
        c.end();
        c.line_stipple(1, 0xa5c3);
        c.color(0, 1, 1, 1);
        c.begin(gl::lines);
        c.vertex(-0.9, 0.5, 0, 1); c.vertex(0.9, 0.55, 0, 1);
        c.vertex(-0.8, 0.9, 0, 1); c.vertex(0.2, 0.62, 0, 1);
        c.end();
        c.line_stipple(3, 0x3cf1);
        c.color(1, 0, 1, 1);
        c.begin(gl::line_loop);
        c.vertex(-0.3, -0.5, 0, 1); c.vertex(0.4, -0.45, 0, 1); c.vertex(0.1, 0.0, 0, 1);
        c.end();
    });
    scene("line stipple: perspective, smooth, clipped", [](Context& c) {
        clear(c, 0.1f, 0.1f, 0.1f);
        c.enable(gl::line_stipple, true);
        c.line_stipple(4, 0x00ff);
        c.begin(gl::line_strip);
        c.color(1, 0, 0, 1); c.vertex(-1.8, -0.9, 0, 2);
        c.color(0, 1, 0, 1); c.vertex(0.9, 0.2, 0, 1);
        c.color(0, 0, 1, 1); c.vertex(1.6, 1.8, 0, 1.5);   // leaves the view volume
        c.color(1, 1, 1, 1); c.vertex(-0.4, 0.9, 0, 1);
        c.end();
    });
    scene("polygon stipple: pattern, scissor, texture", [](Context& c) {
        clear(c, 0, 0, 0.2f);
        uint8_t mask[128];
        for (int row = 0; row < 32; ++row) {
            for (int byte = 0; byte < 4; ++byte) {
                mask[row * 4 + byte] = static_cast<uint8_t>(((row * 37 + byte * 11) * 0x9d) ^ (row & 1 ? 0xf0 : 0x0f));
            }
        }
        c.polygon_stipple(mask);
        c.enable(gl::polygon_stipple, true);
        c.color(1, 0.5f, 0, 1);
        quad(c, -0.95f, -0.95f, 0.95f, 0.1f);
        c.enable(gl::scissor_test, true);
        c.scissor(5, 20, 30, 20);
        c.begin(gl::triangles);
        c.color(0, 1, 0, 1); c.vertex(-0.9, 0.0, 0, 1);
        c.color(0, 0, 1, 1); c.vertex(0.9, 0.2, 0, 1);
        c.color(1, 1, 1, 1); c.vertex(0.1, 0.95, 0, 1);
        c.end();
        c.enable(gl::scissor_test, false);
        // Polygon stipple stays window-anchored and does not touch lines.
        c.color(1, 1, 1, 1);
        c.begin(gl::lines);
        c.vertex(-0.9, -0.2, 0, 1); c.vertex(0.9, -0.25, 0, 1);
        c.end();
    });
}

void front_and_back_scenes() {
    scene("GL_FRONT_AND_BACK: blend, mask, clear", [](Context& c) {
        c.draw_buffer(gl::front);
        clear(c, 0.8f, 0.1f, 0.1f);
        c.draw_buffer(gl::back);
        c.clear_color(0.1f, 0.2f, 0.9f, 1);
        c.clear(gl::color_buffer_bit);
        c.draw_buffer(gl::front_and_back);
        c.enable(gl::blend, true);
        c.blend_func(gl::src_alpha, gl::one_minus_src_alpha);
        c.color(0, 1, 0, 0.5f);
        quad(c, -0.6f, -0.6f, 0.5f, 0.4f);
        c.enable(gl::blend, false);
        c.color_mask(true, false, true, true);
        c.color(1, 1, 1, 1);
        quad(c, -0.2f, 0.2f, 0.9f, 0.9f);
        c.color_mask(true, true, true, true);
        c.enable(gl::scissor_test, true);
        c.scissor(30, 2, 12, 10);
        c.clear_color(1, 1, 0, 1);
        c.clear(gl::color_buffer_bit);
    });
    scene("GL_FRONT_AND_BACK: depth tested once", [](Context& c) {
        clear(c, 0, 0, 0);
        c.enable(gl::depth_test, true);
        c.depth_func(gl::less);
        c.draw_buffer(gl::front_and_back);
        c.color(1, 0, 0, 1);
        quad(c, -0.8f, -0.8f, 0.4f, 0.4f, 0.2f);
        c.color(0, 1, 0, 1);
        quad(c, -0.4f, -0.4f, 0.8f, 0.8f, 0.3f);   // behind: fails LESS in both buffers
        c.color(0, 0, 1, 1);
        quad(c, -0.2f, -0.9f, 0.1f, 0.9f, -0.5f);
    });
}

void draw_pixels_scenes() {
    // 7x5 RGBA image with distinct bytes.
    static std::vector<uint8_t> rgba;
    rgba.resize(7 * 5 * 4);
    for (size_t i = 0; i < rgba.size(); ++i) rgba[i] = static_cast<uint8_t>((i * 53 + 17) & 0xff);
    scene("DrawPixels: RGBA, zoom 1 / 2x3 / -2x1 / 0.5x0.75", [](Context& c) {
        clear(c, 0.05f, 0.05f, 0.05f);
        pixel_projection(c);
        c.raster_pos(2, 3, 0, 1);
        c.draw_pixels(7, 5, gl::rgba, gl::type_unsigned_byte, rgba.data());
        c.pixel_zoom(2, 3);
        c.raster_pos(12.25, 2.5, 0, 1);
        c.draw_pixels(7, 5, gl::rgba, gl::type_unsigned_byte, rgba.data());
        c.pixel_zoom(-2, 1);
        c.raster_pos(44.5, 30, 0, 1);
        c.draw_pixels(7, 5, gl::rgba, gl::type_unsigned_byte, rgba.data());
        c.pixel_zoom(0.5f, 0.75f);
        c.raster_pos(3.75, 40.1, 0, 1);
        c.draw_pixels(7, 5, gl::rgba, gl::type_unsigned_byte, rgba.data());
    });
    scene("DrawPixels: formats, unpack rows, transfer and maps", [](Context& c) {
        clear(c, 0, 0.1f, 0);
        pixel_projection(c);
        // RGB floats with row length / skip and scale/bias.
        std::vector<float> rgb(10 * 6 * 3);
        for (size_t i = 0; i < rgb.size(); ++i) rgb[i] = static_cast<float>((i * 7) % 11) / 10.0f;
        c.pixel_store(gl::unpack_row_length, 10);
        c.pixel_store(gl::unpack_skip_pixels, 2);
        c.pixel_store(gl::unpack_skip_rows, 1);
        c.pixel_transfer(gl::red_scale, 0.5f);
        c.pixel_transfer(gl::green_bias, 0.25f);
        c.raster_pos(1, 1, 0, 1);
        c.draw_pixels(6, 4, gl::rgb, gl::type_float, rgb.data());
        c.pixel_store(gl::unpack_row_length, 0);
        c.pixel_store(gl::unpack_skip_pixels, 0);
        c.pixel_store(gl::unpack_skip_rows, 0);
        c.pixel_transfer(gl::red_scale, 1);
        c.pixel_transfer(gl::green_bias, 0);
        // LUMINANCE_ALPHA unsigned shorts with swapped bytes.
        std::vector<uint16_t> la(5 * 4 * 2);
        for (size_t i = 0; i < la.size(); ++i) la[i] = static_cast<uint16_t>(0x1234 * (i + 1));
        c.pixel_store(gl::unpack_swap_bytes, 1);
        c.enable(gl::blend, true);
        c.blend_func(gl::src_alpha, gl::one_minus_src_alpha);
        c.raster_pos(20, 4, 0, 1);
        c.draw_pixels(5, 4, gl::luminance_alpha, gl::type_unsigned_short, la.data());
        c.enable(gl::blend, false);
        c.pixel_store(gl::unpack_swap_bytes, 0);
        // BGRA bytes through MAP_COLOR R_TO_R / B_TO_B.
        std::vector<uint8_t> bgra(4 * 4 * 4);
        for (size_t i = 0; i < bgra.size(); ++i) bgra[i] = static_cast<uint8_t>(i * 16);
        const float ramp[4] = {1.0f, 0.6f, 0.3f, 0.0f};
        c.pixel_map(gl::pixel_map_r_to_r, 4, ramp);
        c.pixel_map(gl::pixel_map_r_to_r + 2, 4, ramp);
        c.pixel_transfer(gl::map_color, 1);
        c.raster_pos(30, 20, 0, 1);
        c.draw_pixels(4, 4, gl::bgra_ext, gl::type_unsigned_byte, bgra.data());
        c.pixel_transfer(gl::map_color, 0);
        // COLOR_INDEX through the I_TO_RGBA maps.
        const float red[4] = {0, 1, 0.5f, 0.25f}, green[4] = {1, 0, 0.75f, 0.5f};
        c.pixel_map(gl::pixel_map_i_to_r, 4, red);
        c.pixel_map(gl::pixel_map_i_to_r + 1, 4, green);
        const uint8_t indices[12] = {0, 1, 2, 3, 3, 2, 1, 0, 2, 2, 1, 1};
        c.pixel_store(gl::unpack_alignment, 1);
        c.raster_pos(8, 30, 0, 1);
        c.draw_pixels(4, 3, gl::color_index, gl::type_unsigned_byte, indices);
    });
    scene("DrawPixels: texture, fog, alpha test, depth test", [](Context& c) {
        clear(c, 0, 0, 0);
        pixel_projection(c);
        c.enable(gl::depth_test, true);
        c.color(0.2f, 0.4f, 0.6f, 1);
        quad(c, 10, 10, 30, 30, 0.2f);
        uint8_t texel[4] = {200, 100, 50, 255};
        uint32_t name = 0;
        c.gen_textures(1, &name);
        c.bind_texture(gl::texture_2d, name);
        nearest(c);
        c.tex_image_2d(gl::texture_2d, 0, static_cast<int32_t>(gl::rgba), 1, 1, 0, gl::rgba, gl::type_unsigned_byte,
                       texel);
        c.enable(gl::texture_2d, true);
        c.enable(gl::fog, true);
        const float mode[1] = {static_cast<float>(gl::linear)}, start[1] = {0}, end[1] = {2};
        const float fog_color[4] = {0.1f, 0.9f, 0.3f, 1};
        c.fog(gl::fog_mode, mode); c.fog(gl::fog_start, start); c.fog(gl::fog_end, end);
        c.fog(gl::fog_color, fog_color);
        c.enable(gl::alpha_test, true);
        c.alpha_func(gl::greater, 0.4f);
        std::vector<uint8_t> image(9 * 9 * 4);
        for (size_t i = 0; i < image.size(); ++i) image[i] = static_cast<uint8_t>((i * 29) & 0xff);
        c.tex_coord(0.3, 0.7, 0, 1);
        c.raster_pos(6, 6, 0.6, 1);           // behind the quad (depth 0.2 maps to 0.4)
        c.pixel_zoom(3, 3);
        c.draw_pixels(9, 9, gl::rgba, gl::type_unsigned_byte, image.data());
    });
    scene("Bitmap: glyphs in a display list, raster colour", [](Context& c) {
        clear(c, 0, 0, 0);
        pixel_projection(c);
        uint8_t glyph[2 * 10];
        for (int i = 0; i < 20; ++i) glyph[i] = static_cast<uint8_t>(0xa5 ^ (i * 0x37));
        c.pixel_store(gl::unpack_alignment, 2);
        const uint32_t list = c.gen_lists(1);
        c.new_list(list, gl::compile);
        c.bitmap(11, 10, 1.5f, 2.0f, 12.5f, 0.75f, glyph);
        c.end_list();
        c.color(1, 0.8f, 0.2f, 1);
        c.raster_pos(4.25, 6.5, 0, 1);
        for (int i = 0; i < 3; ++i) c.call_list(list);
        c.color(0.3f, 1, 0.6f, 1);
        c.raster_pos(3, 30, 0, 1);
        c.pixel_store(gl::unpack_lsb_first, 1);
        c.bitmap(11, 10, 0, 0, 14, 0, glyph);
        c.bitmap(11, 10, 0, 0, 14, 0, glyph);
    });
    scene("DrawPixels: DEPTH_COMPONENT against a depth test", [](Context& c) {
        clear(c, 0, 0, 0);
        pixel_projection(c);
        c.enable(gl::depth_test, true);
        c.color(0, 0, 1, 1);
        quad(c, 5, 5, 40, 40, 0.013f);              // no image depth ties with the quad
        std::vector<float> depths(12 * 10);
        for (size_t i = 0; i < depths.size(); ++i) depths[i] = static_cast<float>((i * 13) % 17) / 16.0f;
        c.color(1, 0.5f, 0, 1);
        c.raster_pos(8, 9, 0, 1);
        c.pixel_zoom(2, 2);
        c.pixel_transfer(gl::depth_scale, 0.9f);
        c.pixel_transfer(gl::depth_bias, 0.05f);
        c.draw_pixels(12, 10, gl::depth_component, gl::type_float, depths.data());
    });
    scene("DrawPixels: STENCIL_INDEX with write mask and shift", [](Context& c) {
        clear(c, 0, 0, 0);
        pixel_projection(c);
        std::vector<uint8_t> stencil(9 * 7);
        for (size_t i = 0; i < stencil.size(); ++i) stencil[i] = static_cast<uint8_t>(i * 7);
        c.stencil_mask(0x3f);
        c.pixel_transfer(gl::index_shift, 1);
        c.pixel_transfer(gl::index_offset, 3);
        c.pixel_store(gl::unpack_alignment, 1);
        c.raster_pos(4, 5, 0, 1);
        c.pixel_zoom(3, 2);
        c.draw_pixels(9, 7, gl::stencil_index, gl::type_unsigned_byte, stencil.data());
        c.stencil_mask(0xff);
        c.enable(gl::stencil_test, true);
        c.stencil_func(gl::less, 20, 0xff);
        c.color(0, 1, 0, 1);
        quad(c, 0, 0, size, size, 0);
    });
}

void copy_pixels_scenes() {
    const auto setup = [](Context& c) {
        clear(c, 0, 0, 0);
        pixel_projection(c);
        c.begin(gl::triangles);
        c.color(1, 0, 0, 1); c.vertex(2, 2, 0, 1);
        c.color(0, 1, 0, 1); c.vertex(30, 6, 0, 1);
        c.color(0, 0, 1, 1); c.vertex(10, 28, 0, 1);
        c.end();
    };
    scene("CopyPixels: overlapping copy in the back buffer", [setup](Context& c) {
        setup(c);
        c.raster_pos(9, 7, 0, 1);
        c.copy_pixels(2, 2, 26, 24, gl::color);
        c.pixel_zoom(1, -1);
        c.raster_pos(30, 46, 0, 1);
        c.copy_pixels(0, 0, 18, 20, gl::color);
    });
    scene("CopyPixels: scale/bias, front to back, zoom", [setup](Context& c) {
        setup(c);
        c.draw_buffer(gl::front);
        c.color(1, 1, 0, 1);
        quad(c, 30, 30, 46, 46, 0);
        c.draw_buffer(gl::back);
        c.read_buffer(gl::front);
        c.pixel_transfer(gl::red_scale, 0.5f);
        c.pixel_transfer(gl::blue_bias, 0.3f);
        c.pixel_zoom(1.5f, 0.5f);
        c.raster_pos(4, 30, 0, 1);
        c.copy_pixels(28, 28, 18, 18, gl::color);
    });
    scene("CopyPixels: MAP_COLOR uses the frontend path", [setup](Context& c) {
        setup(c);
        const float ramp[2] = {1, 0};
        c.pixel_map(gl::pixel_map_r_to_r + 1, 2, ramp);
        c.pixel_transfer(gl::map_color, 1);
        c.raster_pos(20, 20, 0, 1);
        c.copy_pixels(0, 0, 20, 20, gl::color);
    });
    scene("CopyPixels: DEPTH and STENCIL", [](Context& c) {
        clear(c, 0, 0, 0);
        pixel_projection(c);
        c.enable(gl::depth_test, true);
        c.enable(gl::stencil_test, true);
        c.stencil_func(gl::always, 5, 0xff);
        c.stencil_op(gl::keep, gl::keep, gl::replace);
        c.color(1, 0, 0, 1);
        c.begin(gl::triangles);
        c.vertex(2, 2, -0.5, 1); c.vertex(22, 4, 0.3, 1); c.vertex(6, 20, 0.8, 1);
        c.end();
        c.enable(gl::stencil_test, false);
        c.depth_func(gl::always);
        c.color(0, 1, 0, 1);
        c.raster_pos(24, 2, 0, 1);
        c.pixel_transfer(gl::depth_bias, -0.1f);
        c.copy_pixels(0, 0, 24, 24, gl::depth);
        c.pixel_transfer(gl::index_shift, 2);
        c.pixel_transfer(gl::index_offset, 1);
        c.raster_pos(2, 24, 0, 1);
        c.copy_pixels(0, 0, 24, 24, gl::stencil);
    });
}

void mipmap_scenes() {
    const gl::Enum filters[6] = {gl::nearest, gl::linear, gl::nearest_mipmap_nearest, gl::linear_mipmap_nearest,
                                 gl::nearest_mipmap_linear, gl::linear_mipmap_linear};
    const char* names[6] = {"NEAREST", "LINEAR", "NEAREST_MIPMAP_NEAREST", "LINEAR_MIPMAP_NEAREST",
                            "NEAREST_MIPMAP_LINEAR", "LINEAR_MIPMAP_LINEAR"};
    for (uint32_t f = 0; f < 6; ++f) {
        for (gl::Enum mag : {gl::nearest, gl::linear}) {
            const gl::Enum min = filters[f];
            const std::string name = std::string("mipmap ") + names[f] + (mag == gl::nearest ? " / NEAREST" : " / LINEAR");
            // Minified quads at lambda about 1.3, 2.6 and 0.3, and a magnified one.
            // Equal non-mipmap filters need no LOD and stay submission layout 3.
            const bool lod = min != (mag == gl::nearest ? gl::nearest : gl::linear);
            scene(name, [min, mag](Context& c) {
                clear(c, 0, 0, 0);
                pixel_projection(c);
                mip_texture(c, min, mag);
                c.color(1, 1, 1, 1);
                quad(c, 1, 1, 13.0f, 13.0f);         // 32 texels over 12 pixels: lambda 1.415
                quad(c, 16, 2, 21.0f, 7.0f);         // 32 over 5: lambda 2.678
                quad(c, 24, 1.3f, 47.0f, 24.3f);     // 32 over 23: lambda 0.476 (no texel-edge centres)
                quad(c, 2, 16, 22.0f, 47.0f);        // anisotropic: 32 over 20 and 31
                quad(c, 26, 28, 40.0f, 42.0f);       // 32 over 14: lambda 1.19
            }, 0, lod);
        }
    }
    scene("mipmap: perspective quad, wrap, LINEAR_MIPMAP_LINEAR", [](Context& c) {
        clear(c, 0, 0, 0);
        mip_texture(c, gl::linear_mipmap_linear, gl::linear);
        const float clamp[1] = {static_cast<float>(gl::clamp_to_edge)};
        c.tex_parameter(gl::texture_2d, gl::texture_wrap_t, clamp);
        c.matrix_mode(gl::projection);
        c.frustum(-1, 1, -1, 1, 1, 10);
        c.matrix_mode(gl::modelview);
        c.color(1, 1, 1, 1);
        c.begin(gl::quads);
        c.tex_coord(0, 0, 0, 1); c.vertex(-1.5, -1, -1.2, 1);
        c.tex_coord(4, 0, 0, 1); c.vertex(1.5, -1, -1.2, 1);
        c.tex_coord(4, 3, 0, 1); c.vertex(1.5, 1, -6, 1);
        c.tex_coord(0, 3, 0, 1); c.vertex(-1.5, 1, -6, 1);
        c.end();
    }, 60);
    scene("mipmap: lines and points", [](Context& c) {
        clear(c, 0, 0, 0);
        pixel_projection(c);
        mip_texture(c, gl::nearest_mipmap_linear, gl::linear);
        c.color(1, 1, 1, 1);
        c.begin(gl::lines);
        c.tex_coord(0, 0.5, 0, 1); c.vertex(2, 5, 0, 1);
        c.tex_coord(3, 0.5, 0, 1); c.vertex(44, 9, 0, 1);
        c.tex_coord(0.1, 0.1, 0, 1); c.vertex(3, 20, 0, 1);
        c.tex_coord(0.4, 0.9, 0, 1); c.vertex(40, 41, 0, 1);
        c.end();
        c.point_size(3);
        c.begin(gl::points);
        c.tex_coord(0.3, 0.6, 0, 1); c.vertex(20, 30, 0, 1);
        c.tex_coord(0.8, 0.2, 0, 1); c.vertex(35, 15, 0, 1);
        c.end();
    }, 4);
}

// The drawable whose colour buffers exceed 16 MiB (pitch 9216 B x 2048 rows):
// the GL bottom rows live past 16 MiB in the top-down buffer.
void large_surface() {
    if (!selected("surface")) return;
    constexpr uint32_t w = 2304, h = 2048;
    const Outcome o = run([](Context& c) {
        c.viewport(0, 0, static_cast<int32_t>(w), static_cast<int32_t>(h));
        c.matrix_mode(gl::projection);
        c.ortho(0, w, 0, h, -1, 1);
        c.matrix_mode(gl::modelview);
        c.enable(gl::scissor_test, true);
        c.scissor(2200, 0, 104, 40);
        c.clear_color(0.2f, 0.3f, 0.4f, 1);
        c.clear(gl::color_buffer_bit);
        c.enable(gl::scissor_test, false);
        c.begin(gl::triangles);
        c.color(1, 0, 0, 1); c.vertex(2210, 2, 0, 1);
        c.color(0, 1, 0, 1); c.vertex(2300, 10, 0, 1);
        c.color(0, 0, 1, 1); c.vertex(2250, 36, 0, 1);
        c.end();
        c.raster_pos(2205, 20, 0, 1);
        const uint8_t image[16] = {255, 0, 0, 255, 0, 255, 0, 255, 0, 0, 255, 255, 255, 255, 255, 255};
        c.pixel_zoom(4, 4);
        c.draw_pixels(2, 2, gl::rgba, gl::type_unsigned_byte, image);
    }, 0, w, h, 160ULL << 20, false);
    std::printf("%-40s gpu=%d colour=%u errors=%u\n", "surface over 16 MiB (2304x2048)", o.gpu ? 1 : 0, o.color,
                o.model_errors);
    expect(o.gpu && o.model_errors == 0, "a 2304x2048 drawable (18.9 MB colour buffers) stays on the GPU");
    expect(o.color == 0, "pixels past the 16 MiB offset match gl_raster");
}

// ---------------------------------------------------------------------------
// Mutations of the generated programs.
// ---------------------------------------------------------------------------

void mutations() {
    if (!selected("mutation")) return;
    // Each patches the first matching dword of a v4 pixel program.
    const Mutation list[] = {
        // v_cmp_ne_u32 (VOP3 0x4d) of the stipple bit test -> v_cmp_eq_u32 (0x4a).
        {"stipple bit compare", true, 0xffff0000u, 0xd44d0000u, 0, 0x00070000u},
        // v_floor_f32 -> v_ceil_f32 (VOP1 0x24 -> 0x22) in the first floor.
        {"floor to ceil", true, 0xfe01fe00u, 0x7e004800u, 0, 0x00000c00u},
        // MRT1 export target 1 -> target 0 (EXP target field [9:4]).
        {"MRT1 target", true, 0xfc0003f0u, 0xf8000010u, 0, 0x00000010u},
        // v_cvt_f32_ubyte2 -> ubyte1 (BGRA/RGBA channel swizzle).
        {"byte swizzle", true, 0xfe01fe00u, 0x7e002600u, 0, 0x00000600u},
        // v_log_f32 -> v_exp_f32 (LOD).
        {"lod log", true, 0xfe01fe00u, 0x7e004e00u, 0, 0x00000a00u},
    };
    const struct {
        const char* scene;
        Scene draw;
    } targets[] = {
        {"stipple", [](Context& c) {
             clear(c, 0, 0, 0);
             c.enable(gl::line_stipple, true);
             c.line_stipple(1, 0x0f0f);
             c.begin(gl::lines); c.vertex(-0.9, 0.1, 0, 1); c.vertex(0.9, 0.3, 0, 1); c.end();
             c.line_stipple(1, 0xffff);
             uint8_t mask[128];
             for (int i = 0; i < 128; ++i) mask[i] = static_cast<uint8_t>(i & 1 ? 0x33 : 0xcc);
             c.polygon_stipple(mask);
             c.enable(gl::polygon_stipple, true);
             quad(c, -0.9f, -0.9f, 0.9f, -0.1f);
         }},
        {"pixels", [](Context& c) {
             clear(c, 0, 0, 0);
             pixel_projection(c);
             const uint8_t image[24] = {255, 0, 0, 255, 0, 255, 0, 255, 0, 0, 255, 255,
                                        10, 20, 30, 255, 200, 100, 0, 255, 0, 100, 200, 255};
             c.raster_pos(3.5, 4.25, 0, 1);
             c.pixel_zoom(3, 2.5f);
             c.draw_pixels(3, 2, gl::rgba, gl::type_unsigned_byte, image);
             c.raster_pos(20, 20, 0, 1);
             c.copy_pixels(0, 0, 16, 16, gl::color);
         }},
        {"front and back", [](Context& c) {
             c.draw_buffer(gl::front);
             clear(c, 1, 0, 0);
             c.draw_buffer(gl::front_and_back);
             c.color(0, 1, 0, 1);
             quad(c, -0.5f, -0.5f, 0.5f, 0.5f);
         }},
        {"mipmap", [](Context& c) {
             clear(c, 0, 0, 0);
             pixel_projection(c);
             mip_texture(c, gl::linear_mipmap_linear, gl::linear);
             quad(c, 1, 1, 13.0f, 13.0f);
             quad(c, 16, 2, 21.0f, 7.0f);
         }},
    };
    unsigned killed = 0, total = 0;
    for (const Mutation& m : list) {
        bool detected = false;
        bool hit = false;
        for (const auto& t : targets) {
            active = &m;
            applied = false;
            quiet = true;
            const Outcome o = run(t.draw);
            quiet = false;
            active = nullptr;
            hit = hit || applied;
            if (applied && (o.color != 0 || o.model_errors != 0 || !o.gpu || o.depth != 0 || o.stencil != 0)) {
                detected = true;
            }
        }
        ++total;
        killed += detected ? 1u : 0u;
        std::printf("  mutation %-22s applied=%d killed=%d\n", m.name, hit ? 1 : 0, detected ? 1 : 0);
        expect(hit, std::string("mutation applies: ") + m.name);
        expect(detected, std::string("mutation is detected: ") + m.name);
    }
    std::printf("generator mutations: %u of %u killed\n", killed, total);
}

} // namespace

int main() {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    expect(database().pinned_ok(), "pinned GC 11.0.0 headers and Mesa JSON load");
    stipple_scenes();
    front_and_back_scenes();
    draw_pixels_scenes();
    copy_pixels_scenes();
    mipmap_scenes();
    large_surface();
    mutations();
    return checks.finish("opengl_pixel_paths");
}

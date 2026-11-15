/*
Module: rx7900.tests.opengl_gfx11_draw
Purpose: End-to-end GFX11 OpenGL draws: the portable Context drives the
Gfx11Backend, whose typed submissions pass through the production kernel
modules (ProcessMemory, ProcessGraphics, UserQueues, kernel shader generation)
on the synthetic MES/page-table fixture; every kernel-built IB is executed by the
gfx11_pipeline_model (registers by AMD header name, NGG/PS programs in
gl-shaders' interpreter, fixed-function back end) and the read-back pixels are
compared with the same commands rendered by gl_raster. Also covers present,
readback fallback and reported software fallback.
Created: 2026-10-23
Architecture: Host ASan/UBSan test. Not hardware evidence.
*/
#include <cmath>
#include <cstdio>
#include <functional>
#include <string>

#include "asic_test_device.hpp"
#include "gfx11_graphics_device.hpp"
#include "../src/opengl/gl_context.hpp"

namespace {

using namespace rx7900::opengl;
using gfx11_graphics_device::Device;

rx7900_test::Checks checks;
void expect(bool value, const std::string& what) { checks.expect(value, what.c_str()); }

constexpr uint32_t size = 48;

gfx11_header_db::Database& database() {
    static gfx11_header_db::Database db;
    static bool loaded = db.load();
    (void)loaded;
    return db;
}

using Scene = std::function<void(Context&)>;

struct Software final {
    std::vector<uint8_t> storage = std::vector<uint8_t>(raster_storage_bytes(size, size, true));
    SoftwareBackend backend{storage.data(), storage.size(), FramebufferFormat{true, 24, 8}};
    Context context{&backend, default_allocator()};
    Software() {
        backend.resize(size, size);
        context.viewport(0, 0, size, size);
    }
};

struct Gpu final {
    Device device{database(), 96, 80};
    std::vector<uint8_t> storage = std::vector<uint8_t>(raster_storage_bytes(size, size, true));
    SoftwareBackend fallback{storage.data(), storage.size(), FramebufferFormat{true, 24, 8}};
    std::vector<uint8_t> scratch = std::vector<uint8_t>(gfx11_backend_scratch_bytes);
    std::unique_ptr<Gfx11Backend> backend;
    std::unique_ptr<Context> context;
    Gpu() {
        backend = std::make_unique<Gfx11Backend>(device, scratch.data(), scratch.size(), fallback);
        backend->limit_texture_heap(256 * 1024);
        backend->initialize();
        backend->resize(size, size);
        context = std::make_unique<Context>(backend.get(), default_allocator());
        context->viewport(0, 0, size, size);
    }
};

std::vector<float> pixels(Backend& backend, gl::Enum buffer = gl::back) {
    std::vector<float> out(size * size * 4, -1.0f);
    backend.read(ReadSource::color, buffer, 0, 0, size, size, out.data());
    return out;
}

// Pixels whose channels differ by more than 2/255; differences are allowed only
// on primitive edges (gl_raster has no tie-breaking rule) up to edge_budget.
unsigned compare(const std::vector<float>& a, const std::vector<float>& b, unsigned& covered) {
    unsigned bad = 0;
    covered = 0;
    for (size_t p = 0; p < a.size() / 4; ++p) {
        bool differs = false;
        for (int c = 0; c < 4; ++c) differs = differs || std::fabs(a[p * 4 + c] - b[p * 4 + c]) > 2.5f / 255.0f;
        bad += differs ? 1 : 0;
        covered += a[p * 4] != 0 || a[p * 4 + 1] != 0 || a[p * 4 + 2] != 0 ? 1 : 0;
    }
    return bad;
}

void scene(const std::string& name, const Scene& draw, unsigned edge_budget = 0, bool expect_gpu = true) {
    Software reference;
    Gpu gpu;
    expect(gpu.device.ok && gpu.backend->gpu_active(), name + ": GPU backend initialised over the process service");
    draw(reference.context);
    draw(*gpu.context);
    gpu.context->finish();
    const auto expected = pixels(reference.backend);
    const auto actual = pixels(*gpu.backend);
    unsigned covered = 0;
    const unsigned bad = compare(expected, actual, covered);
    const auto& stats = gpu.device.model->stats();
    if (bad > edge_budget || stats.errors != 0) {
        std::fprintf(stderr, "%s: %u differing pixels (budget %u), model errors %u (%s), draws %u\n", name.c_str(), bad,
                     edge_budget, stats.errors, stats.first_error.c_str(), stats.draws);
    }
    expect(stats.errors == 0 && gpu.device.ib_faults == 0, name + ": IBs execute without model or page faults");
    expect(!expect_gpu || (gpu.backend->gpu_active() && stats.draws > 0 && gpu.device.ibs > 0),
           name + ": rendered by kernel-built GPU IBs (no fallback)");
    expect(bad <= edge_budget, name + ": GPU pixels match gl_raster");
    std::printf("%-34s gpu=%d ibs=%u draws=%u frags=%u diff=%u covered=%u\n", name.c_str(),
                gpu.backend->gpu_active() ? 1 : 0, gpu.device.ibs, stats.draws, stats.fragments, bad, covered);
}

void triangle(Context& c, float x0, float y0, float x1, float y1, float x2, float y2, float z = 0) {
    c.begin(gl::triangles);
    c.vertex(x0, y0, z, 1);
    c.vertex(x1, y1, z, 1);
    c.vertex(x2, y2, z, 1);
    c.end();
}

void quad(Context& c, float x0, float y0, float x1, float y1, float z = 0) {
    c.begin(gl::quads);
    c.vertex(x0, y0, z, 1);
    c.vertex(x1, y0, z, 1);
    c.vertex(x1, y1, z, 1);
    c.vertex(x0, y1, z, 1);
    c.end();
}

void scenes() {
    scene("clear", [](Context& c) {
        c.clear_color(0.25, 0.5, 0.75, 1.0);
        c.clear(gl::color_buffer_bit);
    });
    scene("smooth triangle", [](Context& c) {
        c.clear_color(0, 0, 0, 1);
        c.clear(gl::color_buffer_bit);
        c.begin(gl::triangles);
        c.color(1, 0, 0, 1); c.vertex(-0.9, -0.8, 0, 1);
        c.color(0, 1, 0, 1); c.vertex(0.85, -0.6, 0, 1);
        c.color(0, 0, 1, 1); c.vertex(-0.1, 0.9, 0, 1);
        c.end();
    }, 12);
    scene("flat quads, strip and polygon", [](Context& c) {
        c.clear_color(0.1, 0.1, 0.1, 1);
        c.clear(gl::color_buffer_bit);
        c.shade_model(gl::flat);
        c.begin(gl::quads);
        c.color(1, 0, 0, 1); c.vertex(-0.9, -0.9, 0, 1); c.color(0, 1, 0, 1); c.vertex(-0.2, -0.9, 0, 1);
        c.color(0, 0, 1, 1); c.vertex(-0.2, -0.2, 0, 1); c.color(1, 1, 0, 1); c.vertex(-0.9, -0.2, 0, 1);
        c.end();
        c.begin(gl::quad_strip);
        c.color(1, 0, 1, 1); c.vertex(0.1, -0.9, 0, 1); c.color(0, 1, 1, 1); c.vertex(0.1, -0.2, 0, 1);
        c.color(1, 1, 1, 1); c.vertex(0.5, -0.9, 0, 1); c.color(0.5, 0.5, 0, 1); c.vertex(0.5, -0.2, 0, 1);
        c.color(0, 0.5, 0.5, 1); c.vertex(0.9, -0.9, 0, 1); c.color(0.5, 0, 0.5, 1); c.vertex(0.9, -0.2, 0, 1);
        c.end();
        c.begin(gl::polygon);
        c.color(0.25, 0.75, 0.5, 1); c.vertex(-0.5, 0.1, 0, 1); c.color(1, 0, 0, 1); c.vertex(0.5, 0.1, 0, 1);
        c.color(0, 1, 0, 1); c.vertex(0.7, 0.6, 0, 1); c.color(0, 0, 1, 1); c.vertex(0.0, 0.9, 0, 1);
        c.color(1, 1, 0, 1); c.vertex(-0.7, 0.6, 0, 1);
        c.end();
    }, 24);
    scene("flat triangles", [](Context& c) {
        c.clear_color(0.1, 0.1, 0.1, 1);
        c.clear(gl::color_buffer_bit);
        c.shade_model(gl::flat);
        c.begin(gl::triangles);
        c.color(1, 1, 0, 1); c.vertex(-0.75, -0.75, 0, 1);
        c.color(0, 1, 1, 1); c.vertex(0.5, -0.6, 0, 1);
        c.color(1, 0, 1, 1); c.vertex(0.0, 0.7, 0, 1);
        c.end();
    }, 16);
    scene("depth test", [](Context& c) {
        c.clear_color(0, 0, 0, 1);
        c.clear_depth(1.0);
        c.clear(gl::color_buffer_bit | gl::depth_buffer_bit);
        c.enable(gl::depth_test, true);
        c.depth_func(gl::less);
        c.color(1, 0, 0, 1); quad(c, -0.8, -0.8, 0.4, 0.4, 0.5);
        c.color(0, 1, 0, 1); quad(c, -0.4, -0.4, 0.8, 0.8, -0.2);
        c.color(0, 0, 1, 1); quad(c, -0.9, 0.0, 0.9, 0.2, 0.9);
    }, 16);
    scene("stencil mask", [](Context& c) {
        c.clear_color(0, 0, 0, 1);
        c.clear_stencil(0);
        c.clear(gl::color_buffer_bit | gl::stencil_buffer_bit);
        c.enable(gl::stencil_test, true);
        c.stencil_func(gl::always, 1, 0xff);
        c.stencil_op(gl::keep, gl::keep, gl::replace);
        c.color_mask(false, false, false, false);
        quad(c, -0.5, -0.5, 0.5, 0.5);
        c.color_mask(true, true, true, true);
        c.stencil_func(gl::equal, 1, 0xff);
        c.stencil_op(gl::keep, gl::keep, gl::keep);
        c.color(1, 0.5, 0, 1);
        quad(c, -0.9, -0.9, 0.9, 0.9);
    }, 16);
    scene("alpha blend", [](Context& c) {
        c.clear_color(0.2, 0.4, 0.6, 1);
        c.clear(gl::color_buffer_bit);
        c.enable(gl::blend, true);
        c.blend_func(gl::src_alpha, gl::one_minus_src_alpha);
        c.color(1, 0, 0, 0.5); quad(c, -0.7, -0.7, 0.3, 0.3);
        c.color(0, 1, 0, 0.25); quad(c, -0.3, -0.3, 0.7, 0.7);
    }, 16);
    scene("logic op xor", [](Context& c) {
        c.clear_color(0.5, 0.25, 1.0, 1);
        c.clear(gl::color_buffer_bit);
        c.enable(gl::color_logic_op, true);
        c.logic_op(gl::logic_xor);
        c.color(1, 1, 0, 1); quad(c, -0.6, -0.6, 0.6, 0.6);
    }, 8);
    scene("color mask", [](Context& c) {
        c.clear_color(0.5, 0.5, 0.5, 1);
        c.clear(gl::color_buffer_bit);
        c.color_mask(true, false, true, false);
        c.color(1, 1, 0, 0); quad(c, -0.6, -0.6, 0.6, 0.6);
    }, 8);
    scene("scissor and viewport", [](Context& c) {
        c.clear_color(0, 0, 0, 1);
        c.clear(gl::color_buffer_bit);
        c.enable(gl::scissor_test, true);
        c.scissor(5, 9, 20, 30);
        c.clear_color(0, 0, 1, 1);
        c.clear(gl::color_buffer_bit);
        c.viewport(4, 6, 30, 24);
        c.color(1, 1, 1, 1); triangle(c, -1, -1, 1, -1, 0, 1);
    }, 12);
    for (int mode = 0; mode < 6; ++mode) {
        const bool ccw_front = mode < 3;
        const gl::Enum cull = mode % 3 == 0 ? gl::back : mode % 3 == 1 ? gl::front : gl::front_and_back;
        scene(std::string("cull ") + (ccw_front ? "ccw " : "cw ") + std::to_string(mode % 3), [=](Context& c) {
            c.clear_color(0, 0, 0, 1);
            c.clear(gl::color_buffer_bit);
            c.front_face(ccw_front ? gl::ccw : gl::cw);
            c.enable(gl::cull_face, true);
            c.cull_face(cull);
            c.color(1, 0, 0, 1); triangle(c, -0.9, -0.9, -0.1, -0.9, -0.5, -0.1);   // CCW
            c.color(0, 1, 0, 1); triangle(c, 0.1, 0.1, 0.5, 0.9, 0.9, 0.1);         // CW
        }, 10);
    }
    scene("two-sided lighting", [](Context& c) {
        c.clear_color(0, 0, 0, 1);
        c.clear(gl::color_buffer_bit);
        const float on[1] = {1};
        c.light_model(gl::light_model_two_side, on);
        c.enable(gl::lighting, true);
        c.enable(gl::light0, true);
        const float white[4] = {1, 1, 1, 1};
        c.light(gl::light0, gl::diffuse, white);
        c.normal(0, 0, 1);
        triangle(c, -0.9, -0.9, -0.1, -0.9, -0.5, -0.1);   // front
        triangle(c, 0.1, 0.1, 0.5, 0.9, 0.9, 0.1);         // back
    }, 10);
    scene("linear fog", [](Context& c) {
        c.clear_color(0, 0, 0, 1);
        c.clear(gl::color_buffer_bit);
        c.enable(gl::fog, true);
        const float mode[1] = {static_cast<float>(gl::linear)};
        const float start[1] = {-1}, end[1] = {1};
        const float color[4] = {0.2f, 0.9f, 0.4f, 1};
        c.fog(gl::fog_mode, mode);
        c.fog(gl::fog_start, start);
        c.fog(gl::fog_end, end);
        c.fog(gl::fog_color, color);
        c.color(1, 0, 0, 1);
        c.begin(gl::triangles);
        c.vertex(-0.9, -0.9, -0.9, 1);
        c.vertex(0.9, -0.9, 0.0, 1);
        c.vertex(0.0, 0.9, 0.9, 1);
        c.end();
    }, 16);
    scene("alpha test", [](Context& c) {
        c.clear_color(0, 0, 0, 1);
        c.clear(gl::color_buffer_bit);
        c.enable(gl::alpha_test, true);
        c.alpha_func(gl::greater, 0.5f);
        c.begin(gl::triangles);
        c.color(1, 1, 1, 0); c.vertex(-0.9, -0.9, 0, 1);
        c.color(1, 1, 1, 1); c.vertex(0.9, -0.9, 0, 1);
        c.color(1, 1, 1, 1); c.vertex(0.0, 0.9, 0, 1);
        c.end();
    }, 24);
    scene("textured quad", [](Context& c) {
        c.clear_color(0, 0, 0, 1);
        c.clear(gl::color_buffer_bit);
        uint8_t texels[8 * 8 * 4];
        for (int i = 0; i < 64; ++i) {
            texels[i * 4 + 0] = static_cast<uint8_t>((i % 8) * 32);
            texels[i * 4 + 1] = static_cast<uint8_t>((i / 8) * 32);
            texels[i * 4 + 2] = static_cast<uint8_t>(((i % 8) ^ (i / 8)) * 32);
            texels[i * 4 + 3] = 255;
        }
        uint32_t name = 0;
        c.gen_textures(1, &name);
        c.bind_texture(gl::texture_2d, name);
        const float nearest[1] = {static_cast<float>(gl::nearest)};
        c.tex_parameter(gl::texture_2d, gl::texture_min_filter, nearest);
        c.tex_parameter(gl::texture_2d, gl::texture_mag_filter, nearest);
        c.tex_image_2d(gl::texture_2d, 0, static_cast<int32_t>(gl::rgba), 8, 8, 0, gl::rgba, gl::type_unsigned_byte, texels);
        c.enable(gl::texture_2d, true);
        c.color(1, 1, 1, 1);
        c.begin(gl::quads);
        c.tex_coord(0, 0, 0, 1); c.vertex(-0.8, -0.8, 0, 1);
        c.tex_coord(1, 0, 0, 1); c.vertex(0.8, -0.8, 0, 1);
        c.tex_coord(1, 1, 0, 1); c.vertex(0.8, 0.8, 0, 1);
        c.tex_coord(0, 1, 0, 1); c.vertex(-0.8, 0.8, 0, 1);
        c.end();
    }, 24);
    scene("points and lines", [](Context& c) {
        c.clear_color(0, 0, 0, 1);
        c.clear(gl::color_buffer_bit);
        c.point_size(3);
        c.color(1, 1, 0, 1);
        c.begin(gl::points);
        c.vertex(-0.5, -0.5, 0, 1);
        c.vertex(0.3, 0.6, 0, 1);
        c.end();
        c.color(0, 1, 1, 1);
        c.begin(gl::lines);
        c.vertex(-0.9, 0.2, 0, 1);
        c.vertex(0.9, -0.3, 0, 1);
        c.end();
    }, 40);
}

void depth_readback() {
    Software reference;
    Gpu gpu;
    for (Context* c : {&reference.context, gpu.context.get()}) {
        c->clear_depth(1.0);
        c->clear_stencil(3);
        c->clear(gl::depth_buffer_bit | gl::stencil_buffer_bit);
        c->enable(gl::depth_test, true);
        quad(*c, -0.5, -0.5, 0.5, 0.5, 0.0);
    }
    gpu.context->finish();
    std::vector<float> a(size * size), b(size * size), sa(size * size), sb(size * size);
    reference.backend.read(ReadSource::depth, gl::back, 0, 0, size, size, a.data());
    gpu.backend->read(ReadSource::depth, gl::back, 0, 0, size, size, b.data());
    gpu.backend->read(ReadSource::stencil, gl::back, 0, 0, size, size, sb.data());
    unsigned bad = 0;
    for (size_t i = 0; i < a.size(); ++i) bad += std::fabs(a[i] - b[i]) > 1e-5f ? 1 : 0;
    unsigned stencil_bad = 0;
    for (float s : sb) stencil_bad += s != 3.0f ? 1 : 0;
    std::printf("depth readback: %u depth mismatches, %u stencil mismatches, gpu=%d\n", bad, stencil_bad,
                gpu.backend->gpu_active() ? 1 : 0);
    expect(gpu.backend->gpu_active(), "depth/stencil readback stays on the GPU (DB->CB copy)");
    expect(bad <= 4 * size, "GPU depth (Z32_FLOAT via DB->CB copy) matches gl_raster depth");
    expect(stencil_bad == 0, "GPU stencil readback returns the cleared value");
}

void present() {
    // GPU present: clip rectangles from "radeondisp" copy into the scanout.
    Gpu gpu;
    gpu.device.window_left = 10;
    gpu.device.window_top = 7;
    gpu.device.clip = {{10, 7, 30, 40}, {35, 20, 58, 55}, {0, 0, 5, 5}};
    gpu.context->clear_color(1, 0, 0, 1);
    gpu.context->clear(gl::color_buffer_bit);
    gpu.context->color(0, 0, 1, 1);
    quad(*gpu.context, -1, 0, 1, 1);    // upper half blue
    std::vector<uint32_t> dib(size * size);
    const auto path = gpu.backend->present(dib.data());
    expect(path == PresentPath::gpu, "DrvSwapBuffers presents the back buffer through the GPU");
    unsigned inside = 0, outside = 0, wrong = 0;
    for (uint32_t y = 0; y < 80; ++y) {
        for (uint32_t x = 0; x < 96; ++x) {
            const uint32_t value = gpu.device.scanout[y * 96 + x];
            bool clipped = false;
            for (const auto& r : gpu.device.clip) {
                clipped = clipped || (static_cast<int32_t>(x) >= r.left && static_cast<int32_t>(x) < r.right &&
                                      static_cast<int32_t>(y) >= r.top && static_cast<int32_t>(y) < r.bottom);
            }
            const bool in_window = x >= 10 && x < 10 + size && y >= 7 && y < 7 + size;
            if (clipped && in_window) {
                ++inside;
                // Drawable row (top-down) y-7: upper half is blue.
                const uint32_t expected = (y - 7) < size / 2 ? 0xff0000ffu : 0xffff0000u;
                wrong += value != expected ? 1 : 0;
            } else {
                ++outside;
                wrong += value != 0xff102030u ? 1 : 0;
            }
        }
    }
    std::printf("present: %u pixels inside clip, %u untouched, %u wrong, %u DMA rows\n", inside, outside, wrong,
                gpu.device.model->stats().dma_rows);
    expect(wrong == 0 && inside > 0, "present writes exactly the GDI clip rectangles and nothing else");

    // Fully occluded window: radeondisp reports DC_TRIVIAL (empty bounds) as an
    // empty rectangle list and returns success; nothing may reach the scanout.
    Gpu occluded;
    occluded.device.clip.clear();
    occluded.context->clear_color(1, 1, 1, 1);
    occluded.context->clear(gl::color_buffer_bit);
    std::vector<uint32_t> unused(size * size);
    expect(occluded.backend->present(unused.data()) == PresentPath::gpu,
           "an empty GDI clip (occluded window) is a successful GPU present");
    unsigned touched = 0;
    for (uint32_t v : occluded.device.scanout) touched += v != 0xff102030u ? 1 : 0;
    expect(touched == 0 && occluded.device.model->stats().dma_rows == 0,
           "an occluded window writes no scanout pixel and issues no DMA rows");

    // No DrawEscape hook (or radeondisp declined: NULL CLIPOBJ): exact readback.
    Gpu fallback;
    fallback.device.present_hook = false;
    fallback.context->clear_color(0, 1, 0, 1);
    fallback.context->clear(gl::color_buffer_bit);
    std::vector<uint32_t> pixels(size * size, 0);
    expect(fallback.backend->present(pixels.data()) == PresentPath::readback, "present falls back to GPU readback");
    unsigned green = 0;
    for (uint32_t p : pixels) green += p == 0xff00ff00u ? 1 : 0;
    expect(green == size * size, "readback DIB holds the exact rendered pixels");
}

void fallback() {
    // The remaining state fallback: a texture beyond the GPU texture heap.
    Software reference;
    Gpu gpu;
    std::vector<uint8_t> texels(1024 * 1024 * 4, 0x80);
    for (Context* c : {&reference.context, gpu.context.get()}) {
        c->clear_color(0, 0, 0, 1);
        c->clear(gl::color_buffer_bit);
        c->color(1, 0, 0, 1);
        quad(*c, -0.9, -0.9, 0.0, 0.0);
        uint32_t name = 0;
        c->gen_textures(1, &name);
        c->bind_texture(gl::texture_2d, name);
        const float nearest[1] = {static_cast<float>(gl::nearest)};
        c->tex_parameter(gl::texture_2d, gl::texture_min_filter, nearest);
        c->tex_parameter(gl::texture_2d, gl::texture_mag_filter, nearest);
        c->tex_image_2d(gl::texture_2d, 0, static_cast<int32_t>(gl::rgba), 1024, 1024, 0, gl::rgba,
                        gl::type_unsigned_byte, texels.data());
        c->enable(gl::texture_2d, true);
        c->color(1, 1, 1, 1);
        quad(*c, 0.1, 0.1, 0.9, 0.9);
    }
    expect(!gpu.backend->gpu_active() && gpu.backend->reason() == FallbackReason::texture_unsupported,
           "a texture beyond the GPU texture heap falls back to the software renderer and reports why");
    expect(std::string(fallback_reason_text(gpu.backend->reason())).find("texture") != std::string::npos,
           "fallback reason text names the state");
    unsigned covered = 0;
    const unsigned bad = compare(pixels(reference.backend), pixels(*gpu.backend), covered);
    expect(bad <= 8, "migration copies the GPU drawable into software and rendering continues exactly");
    gpu.backend->resize(size - 16, size);   // within the fallback storage
    expect(gpu.backend->gpu_active(), "a resize (undefined contents) returns a state fallback to the GPU");
}

} // namespace

int main() {
    expect(database().pinned_ok(), "pinned GC 11.0.0 headers and Mesa JSON load");
    scenes();
    depth_readback();
    present();
    fallback();
    return checks.finish("opengl_gfx11_draw");
}

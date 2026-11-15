/*
Module: rx7900.tests.opengl_context
Purpose: Drive the OpenGL 1.1 context through the software backend and verify
clears, flat and smooth triangles, depth test, display lists, vertex arrays and
texturing produce the expected BGRA8 pixels; raster position, DrawPixels zoom
and pixel transfer, depth/stencil images, Bitmap, CopyPixels, stipple counter
restarts, polygon stipple, GL_FRONT_AND_BACK and mipmap level selection
(2026-10-24) against hand-computed GL 1.1 results.
Created: 2026-10-23
Architecture: Uses gl_context over gl_raster (SoftwareBackend), the renderer the
ICD uses for its software fallback. Host only; no GPU, no OS.
*/
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>
#include <string>

#include "../src/opengl/gl_context.hpp"
#include "../src/opengl/gl_raster.hpp"

namespace {

using namespace rx7900::opengl;

unsigned checks = 0;
unsigned failures = 0;

void expect(bool condition, const char* contract) {
    ++checks;
    if (!condition) {
        ++failures;
        std::fprintf(stderr, "FAIL: %s\n", contract);
    }
}

struct Fixture final {
    std::vector<uint8_t> storage;
    SoftwareBackend backend;
    Context context;

    Fixture(uint32_t width, uint32_t height)
        : storage(raster_storage_bytes(width, height, true)),
          backend(storage.data(), storage.size(), FramebufferFormat{true, 24, 8}),
          context(&backend, default_allocator()) {
        backend.resize(width, height);
        context.viewport(0, 0, static_cast<int32_t>(width), static_cast<int32_t>(height));
    }

    uint32_t pixel(gl::Enum buffer, uint32_t x, uint32_t y) {
        const uint32_t* color = backend.color_buffer(buffer);
        return color != nullptr ? color[static_cast<size_t>(y) * backend.width() + x] : 0;
    }
};

uint32_t bgra(uint8_t r, uint8_t g, uint8_t b, uint8_t a) {
    return (static_cast<uint32_t>(a) << 24) | (static_cast<uint32_t>(r) << 16) |
           (static_cast<uint32_t>(g) << 8) | b;
}

void test_clear() {
    Fixture fixture(64, 64);
    fixture.context.clear_color(0.25, 0.5, 0.75, 1.0);
    fixture.context.clear(gl::color_buffer_bit);
    const uint32_t expected = bgra(64, 128, 191, 255);
    expect(fixture.pixel(gl::back, 0, 0) == expected, "clear fills corner");
    expect(fixture.pixel(gl::back, 32, 32) == expected, "clear fills center");
    expect(fixture.context.get_error() == gl::no_error, "clear leaves no error");
}

void test_flat_triangle() {
    Fixture fixture(64, 64);
    fixture.context.clear_color(0, 0, 0, 1);
    fixture.context.clear(gl::color_buffer_bit);
    fixture.context.shade_model(gl::flat);
    // A big triangle covering the center, identity transform (clip == NDC).
    fixture.context.color(1.0, 0.0, 0.0, 1.0);
    fixture.context.begin(gl::triangles);
    fixture.context.vertex(-0.8, -0.8, 0, 1);
    fixture.context.vertex(0.8, -0.8, 0, 1);
    fixture.context.vertex(0.0, 0.8, 0, 1);
    fixture.context.end();
    expect(fixture.pixel(gl::back, 32, 32) == bgra(255, 0, 0, 255), "flat triangle fills center red");
    expect(fixture.pixel(gl::back, 1, 62) == bgra(0, 0, 0, 255), "triangle leaves top corner clear");
    expect(fixture.context.get_error() == gl::no_error, "triangle leaves no error");
}

void test_smooth_triangle() {
    Fixture fixture(64, 64);
    fixture.context.clear_color(0, 0, 0, 1);
    fixture.context.clear(gl::color_buffer_bit);
    fixture.context.shade_model(gl::smooth);
    fixture.context.begin(gl::triangles);
    fixture.context.color(1, 0, 0, 1);
    fixture.context.vertex(-0.9, -0.9, 0, 1);
    fixture.context.color(0, 1, 0, 1);
    fixture.context.vertex(0.9, -0.9, 0, 1);
    fixture.context.color(0, 0, 1, 1);
    fixture.context.vertex(0.0, 0.9, 0, 1);
    fixture.context.end();
    const uint32_t center = fixture.pixel(gl::back, 32, 32);
    const uint8_t r = (center >> 16) & 0xff;
    const uint8_t g = (center >> 8) & 0xff;
    const uint8_t b = center & 0xff;
    expect(r > 20 && g > 20 && b > 20, "smooth triangle blends colors at the centroid");
}

void test_depth() {
    Fixture fixture(32, 32);
    fixture.context.clear_color(0, 0, 0, 1);
    fixture.context.clear_depth(1.0);
    fixture.context.clear(gl::color_buffer_bit | gl::depth_buffer_bit);
    fixture.context.enable(gl::depth_test, true);
    fixture.context.depth_func(gl::less);
    // Near red triangle.
    fixture.context.color(1, 0, 0, 1);
    fixture.context.begin(gl::triangles);
    fixture.context.vertex(-1, -1, -0.5, 1);
    fixture.context.vertex(1, -1, -0.5, 1);
    fixture.context.vertex(0, 1, -0.5, 1);
    fixture.context.end();
    // Farther green triangle must be occluded.
    fixture.context.color(0, 1, 0, 1);
    fixture.context.begin(gl::triangles);
    fixture.context.vertex(-1, -1, 0.5, 1);
    fixture.context.vertex(1, -1, 0.5, 1);
    fixture.context.vertex(0, 1, 0.5, 1);
    fixture.context.end();
    expect(fixture.pixel(gl::back, 16, 10) == bgra(255, 0, 0, 255), "depth test keeps the near triangle");
}

void test_display_list() {
    Fixture fixture(64, 64);
    fixture.context.clear_color(0, 0, 0, 1);
    fixture.context.clear(gl::color_buffer_bit);
    const uint32_t list = fixture.context.gen_lists(1);
    expect(list != 0, "gen_lists returns a name");
    fixture.context.new_list(list, gl::compile);
    fixture.context.shade_model(gl::flat);
    fixture.context.color(0, 1, 0, 1);
    fixture.context.begin(gl::triangles);
    fixture.context.vertex(-0.8, -0.8, 0, 1);
    fixture.context.vertex(0.8, -0.8, 0, 1);
    fixture.context.vertex(0.0, 0.8, 0, 1);
    fixture.context.end();
    fixture.context.end_list();
    expect(fixture.pixel(gl::back, 32, 32) == bgra(0, 0, 0, 255), "compile does not draw");
    fixture.context.call_list(list);
    expect(fixture.pixel(gl::back, 32, 32) == bgra(0, 255, 0, 255), "call_list replays the triangle");
    expect(fixture.context.is_list(list), "is_list reports the compiled list");
}

void test_vertex_arrays() {
    Fixture fixture(64, 64);
    fixture.context.clear_color(0, 0, 0, 1);
    fixture.context.clear(gl::color_buffer_bit);
    fixture.context.shade_model(gl::flat);
    fixture.context.color(0, 0, 1, 1);
    const float positions[9] = {-0.8f, -0.8f, 0, 0.8f, -0.8f, 0, 0.0f, 0.8f, 0};
    fixture.context.vertex_pointer(3, gl::type_float, 0, positions);
    fixture.context.enable_client_state(gl::vertex_array, true);
    fixture.context.draw_arrays(gl::triangles, 0, 3);
    expect(fixture.pixel(gl::back, 32, 32) == bgra(0, 0, 255, 255), "draw_arrays renders the triangle");
}

void test_texture_modulate() {
    Fixture fixture(32, 32);
    fixture.context.clear_color(0, 0, 0, 1);
    fixture.context.clear(gl::color_buffer_bit);
    uint32_t name = 0;
    fixture.context.gen_textures(1, &name);
    fixture.context.bind_texture(gl::texture_2d, name);
    const uint8_t texel[4] = {128, 255, 64, 255};   // RGBA
    fixture.context.tex_image_2d(gl::texture_2d, 0, gl::rgba, 1, 1, 0, gl::rgba,
                                 gl::type_unsigned_byte, texel);
    const float nearest = static_cast<float>(gl::nearest);
    fixture.context.tex_parameter(gl::texture_2d, gl::texture_min_filter, &nearest);
    fixture.context.tex_parameter(gl::texture_2d, gl::texture_mag_filter, &nearest);
    fixture.context.enable(gl::texture_2d, true);
    fixture.context.shade_model(gl::flat);
    fixture.context.color(1, 1, 1, 1);     // white * texel = texel (modulate)
    fixture.context.begin(gl::triangles);
    fixture.context.tex_coord(0, 0, 0, 1); fixture.context.vertex(-1, -1, 0, 1);
    fixture.context.tex_coord(1, 0, 0, 1); fixture.context.vertex(1, -1, 0, 1);
    fixture.context.tex_coord(0, 1, 0, 1); fixture.context.vertex(0, 1, 0, 1);
    fixture.context.end();
    const uint32_t center = fixture.pixel(gl::back, 16, 8);
    expect(((center >> 16) & 0xff) == 128 && ((center >> 8) & 0xff) == 255 && (center & 0xff) == 64,
           "modulated texture sample reaches the framebuffer");
}

void test_strings() {
    Fixture fixture(4, 4);
    expect(std::string(fixture.context.get_string(gl::version)) == "1.1.0", "GL version is 1.1.0");
    expect(fixture.context.get_string(gl::renderer)[0] != 0, "renderer string present");
}

} // namespace


// GL winding: CCW in y-up window coordinates is front; GL_CULL_FACE drops the
// selected face; two-sided colour selects color[1] for back faces.
void draw_ccw(Fixture& fixture, bool reverse) {
    fixture.context.begin(gl::triangles);
    fixture.context.vertex(-0.8, -0.8, 0, 1);
    if (reverse) {
        fixture.context.vertex(0.0, 0.8, 0, 1);
        fixture.context.vertex(0.8, -0.8, 0, 1);
    } else {
        fixture.context.vertex(0.8, -0.8, 0, 1);
        fixture.context.vertex(0.0, 0.8, 0, 1);
    }
    fixture.context.end();
}

void test_face_culling() {
    struct Case {
        gl::Enum cull;
        gl::Enum front;
        bool reverse;
        bool drawn;
        const char* contract;
    };
    const Case cases[] = {
        {gl::back, gl::ccw, false, true, "CCW triangle survives GL_BACK culling"},
        {gl::front, gl::ccw, false, false, "CCW triangle is culled by GL_FRONT"},
        {gl::back, gl::ccw, true, false, "CW triangle is culled by GL_BACK"},
        {gl::back, gl::cw, false, false, "glFrontFace(GL_CW) makes CCW a back face"},
        {gl::front_and_back, gl::ccw, false, false, "GL_FRONT_AND_BACK culls everything"},
    };
    for (const Case& test : cases) {
        Fixture fixture(64, 64);
        fixture.context.clear_color(0, 0, 0, 1);
        fixture.context.clear(gl::color_buffer_bit);
        fixture.context.color(1.0, 0.0, 0.0, 1.0);
        fixture.context.enable(gl::cull_face, true);
        fixture.context.cull_face(test.cull);
        fixture.context.front_face(test.front);
        draw_ccw(fixture, test.reverse);
        const bool drawn = fixture.pixel(gl::back, 32, 32) == bgra(255, 0, 0, 255);
        expect(drawn == test.drawn, test.contract);
    }
}

void test_two_sided_color() {
    for (const bool reverse : {false, true}) {
        const uint32_t size = 64;
        std::vector<uint8_t> storage(raster_storage_bytes(size, size, true));
        SoftwareBackend backend(storage.data(), storage.size(), FramebufferFormat{true, 24, 8});
        backend.resize(size, size);
        DrawState state{};
        state.viewport_width = size;
        state.viewport_height = size;
        state.two_sided_color = true;
        RasterVertex vertices[3]{};
        const float x[3] = {-0.8f, reverse ? 0.0f : 0.8f, reverse ? 0.8f : 0.0f};
        const float y[3] = {-0.8f, reverse ? 0.8f : -0.8f, reverse ? -0.8f : 0.8f};
        for (uint32_t i = 0; i < 3; ++i) {
            vertices[i].clip = {x[i], y[i], 0, 1};
            vertices[i].color[0] = {1, 0, 0, 1};
            vertices[i].color[1] = {0, 0, 1, 1};
        }
        backend.draw(state, PrimitiveKind::triangles, vertices, 3);
        const uint32_t* color = backend.color_buffer(gl::back);
        const uint32_t center = color != nullptr ? color[32u * size + 32u] : 0;
        expect(center == (reverse ? bgra(0, 0, 255, 255) : bgra(255, 0, 0, 255)),
               reverse ? "two-sided back face uses the back colour" : "two-sided front face uses the front colour");
    }
}

// GL_FLAT takes each primitive's colour from its provoking vertex (GL 1.1
// table 2.6): last for triangles, 4th for quads, (2i+4)th for quad strips,
// first for polygons. Every vertex gets a distinct colour.
void test_flat_provoking() {
    struct Case {
        gl::Enum mode;
        uint32_t provoking;
        const char* contract;
    };
    const Case cases[] = {
        {gl::quads, 3, "flat GL_QUADS uses the 4th vertex"},
        {gl::quad_strip, 3, "flat GL_QUAD_STRIP uses vertex 2i+4"},
        {gl::polygon, 0, "flat GL_POLYGON uses the first vertex"},
        {gl::triangle_fan, 3, "flat GL_TRIANGLE_FAN uses each triangle's last vertex"},
    };
    const double colors[4][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 1, 0}};
    // CCW square; quad_strip order is (0,1,3,2) of the same corners.
    const double square[4][2] = {{-0.8, -0.8}, {0.8, -0.8}, {0.8, 0.8}, {-0.8, 0.8}};
    const uint32_t strip_order[4] = {0, 1, 3, 2};
    for (const Case& test : cases) {
        Fixture fixture(64, 64);
        fixture.context.clear_color(0, 0, 0, 1);
        fixture.context.clear(gl::color_buffer_bit);
        fixture.context.shade_model(gl::flat);
        fixture.context.begin(test.mode);
        for (uint32_t i = 0; i < 4; ++i) {
            const uint32_t corner = test.mode == gl::quad_strip ? strip_order[i] : i;
            fixture.context.color(colors[i][0], colors[i][1], colors[i][2], 1.0);
            fixture.context.vertex(square[corner][0], square[corner][1], 0, 1);
        }
        fixture.context.end();
        const uint8_t r = static_cast<uint8_t>(colors[test.provoking][0] * 255);
        const uint8_t g = static_cast<uint8_t>(colors[test.provoking][1] * 255);
        const uint8_t b = static_cast<uint8_t>(colors[test.provoking][2] * 255);
        const uint32_t want = bgra(r, g, b, 255);
        if (test.mode == gl::triangle_fan) {
            // Fan triangles (0,1,2) and (0,2,3): last vertices 2 and 3.
            const uint32_t want_first = bgra(0, 0, 255, 255);
            expect(fixture.pixel(gl::back, 48, 20) == want_first && fixture.pixel(gl::back, 16, 44) == want,
                   test.contract);
        } else {
            expect(fixture.pixel(gl::back, 20, 20) == want && fixture.pixel(gl::back, 44, 44) == want,
                   test.contract);
        }
    }
}

// ---- Raster position, pixel rectangles, stipple, draw buffers, mipmaps ----

void pixel_projection(Fixture& f, uint32_t size) {
    f.context.matrix_mode(gl::projection);
    f.context.load_identity();
    f.context.ortho(0, size, 0, size, -1, 1);
    f.context.matrix_mode(gl::modelview);
    f.context.load_identity();
}

void test_raster_position() {
    Fixture f(16, 16);
    pixel_projection(f, 16);
    f.context.color(0.5, 0.25, 1, 1);
    f.context.tex_coord(0.5, 0.75, 0, 1);
    f.context.raster_pos(3.5, 4.25, 0, 1);
    float position[4] = {}, color[4] = {}, texcoord[4] = {};
    uint8_t valid = 0;
    f.context.get_floatv(gl::current_raster_position, position);
    f.context.get_floatv(gl::current_raster_color, color);
    f.context.get_floatv(gl::current_raster_texture_coords, texcoord);
    f.context.get_booleanv(gl::current_raster_position_valid, &valid);
    expect(position[0] == 3.5f && position[1] == 4.25f && position[2] == 0.5f && position[3] == 1.0f,
           "RasterPos transforms to window x, y, z and keeps clip w");
    expect(valid == 1 && color[0] == 0.5f && color[2] == 1.0f && texcoord[1] == 0.75f,
           "the raster position is valid and takes the current colour and texture coordinates");
    f.context.raster_pos(40, 4, 0, 1);
    f.context.get_booleanv(gl::current_raster_position_valid, &valid);
    expect(valid == 0, "a raster position outside the view volume is invalid");
    const uint8_t white[4] = {255, 255, 255, 255};
    f.context.clear_color(0, 0, 0, 0);
    f.context.clear(gl::color_buffer_bit);
    f.context.draw_pixels(1, 1, gl::rgba, gl::type_unsigned_byte, white);
    bool untouched = true;
    for (uint32_t y = 0; y < 16; ++y) {
        for (uint32_t x = 0; x < 16; ++x) untouched = untouched && f.pixel(gl::back, x, y) == 0;
    }
    expect(untouched, "DrawPixels at an invalid raster position draws nothing");
}

void test_draw_pixels_zoom() {
    Fixture f(16, 16);
    pixel_projection(f, 16);
    f.context.clear_color(0, 0, 0, 0);
    f.context.clear(gl::color_buffer_bit);
    // 2x2 image, rows bottom to top: (r) (g) / (b) (w).
    const uint8_t image[16] = {255, 0, 0, 255, 0, 255, 0, 255, 0, 0, 255, 255, 255, 255, 255, 255};
    f.context.raster_pos(4, 4, 0, 1);
    f.context.draw_pixels(2, 2, gl::rgba, gl::type_unsigned_byte, image);
    expect(f.pixel(gl::back, 4, 4) == bgra(255, 0, 0, 255) && f.pixel(gl::back, 5, 4) == bgra(0, 255, 0, 255) &&
               f.pixel(gl::back, 4, 5) == bgra(0, 0, 255, 255) && f.pixel(gl::back, 5, 5) == bgra(255, 255, 255, 255) &&
               f.pixel(gl::back, 6, 4) == 0 && f.pixel(gl::back, 3, 4) == 0,
           "DrawPixels writes the image at the raster position, row 0 at the bottom");
    f.context.pixel_zoom(2, 3);
    f.context.raster_pos(8, 2, 0, 1);
    f.context.draw_pixels(2, 2, gl::rgba, gl::type_unsigned_byte, image);
    expect(f.pixel(gl::back, 9, 4) == bgra(255, 0, 0, 255) && f.pixel(gl::back, 10, 2) == bgra(0, 255, 0, 255) &&
               f.pixel(gl::back, 8, 5) == bgra(0, 0, 255, 255) && f.pixel(gl::back, 11, 7) == bgra(255, 255, 255, 255) &&
               f.pixel(gl::back, 12, 2) == 0 && f.pixel(gl::back, 8, 8) == 0,
           "zoom 2x3: each source pixel covers 2 x 3 window pixels");
    f.context.pixel_zoom(-2, 1);
    f.context.raster_pos(15, 12, 0, 1);
    f.context.draw_pixels(2, 2, gl::rgba, gl::type_unsigned_byte, image);
    expect(f.pixel(gl::back, 14, 12) == bgra(255, 0, 0, 255) && f.pixel(gl::back, 13, 12) == bgra(255, 0, 0, 255) &&
               f.pixel(gl::back, 12, 12) == bgra(0, 255, 0, 255) && f.pixel(gl::back, 15, 12) == 0,
           "a negative zoom mirrors the image leftwards from the raster position");
    f.context.pixel_zoom(0, 1);
    f.context.raster_pos(0, 0, 0, 1);
    f.context.draw_pixels(2, 2, gl::rgba, gl::type_unsigned_byte, image);
    expect(f.pixel(gl::back, 0, 0) == 0, "a zero zoom generates no fragment");
    expect(f.context.get_error() == gl::no_error, "pixel rectangles leave no error");
}

void test_pixel_transfer() {
    Fixture f(8, 8);
    pixel_projection(f, 8);
    const uint8_t gray[4] = {200, 100, 50, 255};
    f.context.pixel_transfer(gl::red_scale, 0.5f);
    f.context.pixel_transfer(gl::green_bias, 0.25f);
    f.context.raster_pos(1, 1, 0, 1);
    f.context.draw_pixels(1, 1, gl::rgba, gl::type_unsigned_byte, gray);
    // r = 200/255 * 0.5 = 0.392 -> 100; g = 100/255 + 0.25 = 0.642 -> 164.
    expect(f.pixel(gl::back, 1, 1) == bgra(100, 164, 50, 255), "RED_SCALE and GREEN_BIAS apply before the write");
    f.context.pixel_transfer(gl::red_scale, 1);
    f.context.pixel_transfer(gl::green_bias, 0);
    const float invert[2] = {1, 0};
    f.context.pixel_map(gl::pixel_map_r_to_r, 2, invert);
    f.context.pixel_transfer(gl::map_color, 1);
    f.context.raster_pos(2, 1, 0, 1);
    f.context.draw_pixels(1, 1, gl::rgba, gl::type_unsigned_byte, gray);
    // R_TO_R[round(0.78)] = 0; G, B, A use the default one-entry maps (value 0).
    expect(f.pixel(gl::back, 2, 1) == bgra(0, 0, 0, 0), "MAP_COLOR looks every channel up (R_TO_R and defaults)");
    f.context.pixel_transfer(gl::map_color, 0);
    const float red[4] = {0, 1, 0.5f, 0}, alpha[4] = {1, 1, 1, 1};
    f.context.pixel_map(gl::pixel_map_i_to_r, 4, red);
    f.context.pixel_map(gl::pixel_map_i_to_a, 4, alpha);
    const uint8_t index = 6;   // 6 & 3 = 2 -> red 0.5
    f.context.pixel_store(gl::unpack_alignment, 1);
    f.context.raster_pos(3, 1, 0, 1);
    f.context.draw_pixels(1, 1, gl::color_index, gl::type_unsigned_byte, &index);
    expect(f.pixel(gl::back, 3, 1) == bgra(128, 0, 0, 255), "COLOR_INDEX converts through the I_TO_RGBA maps");
    f.context.pixel_map(gl::pixel_map_i_to_r, 3, red);
    expect(f.context.get_error() == gl::invalid_value, "an index map size must be a power of two");
    float back[4] = {};
    f.context.get_pixel_map(gl::pixel_map_i_to_r, back);
    expect(f.context.pixel_map_size(gl::pixel_map_i_to_r) == 4 && back[2] == 0.5f, "GetPixelMap returns the map");
}

void test_depth_stencil_pixels() {
    Fixture f(8, 8);
    pixel_projection(f, 8);
    f.context.clear_depth(1.0);
    f.context.clear_stencil(0);
    f.context.clear(gl::color_buffer_bit | gl::depth_buffer_bit | gl::stencil_buffer_bit);
    f.context.enable(gl::depth_test, true);
    f.context.color(1, 0, 0, 1);
    const float depths[2] = {0.25f, 1.0f};
    f.context.raster_pos(2, 2, 0, 1);
    f.context.draw_pixels(2, 1, gl::depth_component, gl::type_float, depths);
    float stored[2] = {};
    f.backend.read(ReadSource::depth, gl::back, 2, 2, 2, 1, stored);
    expect(f.pixel(gl::back, 2, 2) == bgra(255, 0, 0, 255) && f.pixel(gl::back, 3, 2) == 0 &&
               std::fabs(stored[0] - 0.25f) < 1e-6f,
           "DEPTH_COMPONENT fragments take the raster colour and pass the depth test per pixel");
    f.context.enable(gl::depth_test, false);
    f.context.stencil_mask(0x0f);
    f.context.enable(gl::scissor_test, true);
    f.context.scissor(0, 0, 5, 8);
    const uint8_t indices[3] = {0xa7, 0x13, 0x55};
    f.context.pixel_store(gl::unpack_alignment, 1);
    f.context.pixel_transfer(gl::index_offset, 1);
    f.context.raster_pos(3, 4, 0, 1);
    f.context.draw_pixels(3, 1, gl::stencil_index, gl::type_unsigned_byte, indices);
    float s[3] = {};
    f.backend.read(ReadSource::stencil, gl::back, 3, 4, 3, 1, s);
    expect(s[0] == 8.0f && s[1] == 4.0f && s[2] == 0.0f && f.pixel(gl::back, 3, 4) == 0,
           "STENCIL_INDEX writes (index + offset) under the stencil mask and scissor, no colour");
}

void test_bitmap() {
    Fixture f(16, 16);
    pixel_projection(f, 16);
    f.context.clear_color(0, 0, 0, 0);
    f.context.clear(gl::color_buffer_bit);
    f.context.color(0, 1, 0, 1);
    f.context.raster_pos(4.75, 3.25, 0, 1);
    const uint8_t rows[2] = {0xc0, 0x80};   // row 0: x 0, 1; row 1: x 0
    f.context.pixel_store(gl::unpack_alignment, 1);
    f.context.bitmap(2, 2, 1.5f, 0, 6, 1, rows);
    // Lower left corner floor(4.75 - 1.5, 3.25 - 0) = (3, 3).
    expect(f.pixel(gl::back, 3, 3) == bgra(0, 255, 0, 255) && f.pixel(gl::back, 4, 3) == bgra(0, 255, 0, 255) &&
               f.pixel(gl::back, 3, 4) == bgra(0, 255, 0, 255) && f.pixel(gl::back, 4, 4) == 0,
           "Bitmap sets the raster colour where bits are set, at floor(raster - origin)");
    float position[4] = {};
    f.context.get_floatv(gl::current_raster_position, position);
    expect(position[0] == 10.75f && position[1] == 4.25f, "Bitmap advances the raster position by its move");
    f.context.pixel_store(gl::unpack_lsb_first, 1);
    f.context.bitmap(2, 1, 0, 0, 0, 0, rows);   // LSB first: 0xc0 has bits 6, 7 -> x 6, 7 (outside width 2)
    expect(f.pixel(gl::back, 10, 4) == 0, "GL_UNPACK_LSB_FIRST reads bit 0 as the first pixel");
    const uint32_t list = f.context.gen_lists(1);
    f.context.pixel_store(gl::unpack_lsb_first, 0);
    f.context.new_list(list, gl::compile);
    f.context.bitmap(1, 1, 0, 0, 1, 0, rows);
    f.context.end_list();
    f.context.raster_pos(12, 12, 0, 1);
    f.context.call_list(list);
    f.context.call_list(list);
    expect(f.pixel(gl::back, 12, 12) == bgra(0, 255, 0, 255) && f.pixel(gl::back, 13, 12) == bgra(0, 255, 0, 255),
           "a Bitmap recorded in a display list replays and advances");
}

void test_copy_pixels() {
    Fixture f(8, 8);
    pixel_projection(f, 8);
    f.context.clear_color(0, 0, 0, 0);
    f.context.clear(gl::color_buffer_bit);
    const uint8_t image[16] = {10, 0, 0, 255, 20, 0, 0, 255, 30, 0, 0, 255, 40, 0, 0, 255};
    f.context.raster_pos(0, 0, 0, 1);
    f.context.draw_pixels(4, 1, gl::rgba, gl::type_unsigned_byte, image);
    // Overlapping copy one pixel right: GL reads the whole source first.
    f.context.raster_pos(1, 0, 0, 1);
    f.context.copy_pixels(0, 0, 4, 1, gl::color);
    expect(f.pixel(gl::back, 1, 0) == bgra(10, 0, 0, 255) && f.pixel(gl::back, 2, 0) == bgra(20, 0, 0, 255) &&
               f.pixel(gl::back, 4, 0) == bgra(40, 0, 0, 255),
           "an overlapping CopyPixels copies the source as it was");
}

void test_line_stipple_restart() {
    for (int strip = 0; strip < 2; ++strip) {
        Fixture f(16, 4);
        f.context.matrix_mode(gl::projection);
        f.context.ortho(0, 16, 0, 4, -1, 1);
        f.context.matrix_mode(gl::modelview);
        f.context.clear_color(0, 0, 0, 0);
        f.context.clear(gl::color_buffer_bit);
        f.context.enable(gl::line_stipple, true);
        f.context.line_stipple(1, 0x003f);   // samples 0..5 drawn
        f.context.color(1, 1, 1, 1);
        f.context.begin(strip != 0 ? gl::line_strip : gl::lines);
        f.context.vertex(0.5, 1.5, 0, 1);
        f.context.vertex(4.5, 1.5, 0, 1);
        if (strip == 0) { f.context.vertex(4.5, 1.5, 0, 1); }
        f.context.vertex(8.5, 1.5, 0, 1);
        f.context.end();
        // Each segment rasterizes 6 samples: the strip's second segment
        // continues at s = 6 (pattern bits 6.. clear), GL_LINES restarts.
        const bool second = f.pixel(gl::back, 6, 1) != 0;
        expect(f.pixel(gl::back, 2, 1) != 0 && second == (strip == 0),
               strip != 0 ? "a line strip continues the stipple counter" : "GL_LINES restarts the counter per segment");
    }
}

void test_polygon_stipple() {
    Fixture f(40, 4);
    f.context.clear_color(0, 0, 0, 0);
    f.context.clear(gl::color_buffer_bit);
    uint8_t mask[128] = {};
    mask[0] = 0x80;          // row 0: x 0 only
    mask[4 * 1 + 3] = 0x01;  // row 1: x 31 only
    f.context.polygon_stipple(mask);
    f.context.enable(gl::polygon_stipple, true);
    f.context.color(1, 0, 0, 1);
    f.context.begin(gl::quads);
    f.context.vertex(-1, -1, 0, 1); f.context.vertex(1, -1, 0, 1); f.context.vertex(1, 1, 0, 1); f.context.vertex(-1, 1, 0, 1);
    f.context.end();
    expect(f.pixel(gl::back, 0, 0) != 0 && f.pixel(gl::back, 1, 0) == 0 && f.pixel(gl::back, 32, 0) != 0 &&
               f.pixel(gl::back, 31, 1) != 0 && f.pixel(gl::back, 30, 1) == 0,
           "polygon stipple: window-anchored 32x32 pattern, MSB first, repeating");
    uint8_t back[128] = {};
    f.context.get_polygon_stipple(back);
    expect(back[0] == 0x80 && back[7] == 0x01 && back[1] == 0, "GetPolygonStipple returns the pattern");
}

void test_front_and_back() {
    Fixture f(8, 8);
    f.context.draw_buffer(gl::front);
    f.context.clear_color(1, 0, 0, 1);
    f.context.clear(gl::color_buffer_bit);
    f.context.draw_buffer(gl::back);
    f.context.clear_color(0, 0, 1, 1);
    f.context.clear(gl::color_buffer_bit);
    f.context.draw_buffer(gl::left);
    int32_t buffer = 0;
    f.context.get_integerv(gl::draw_buffer, &buffer);
    expect(buffer == static_cast<int32_t>(gl::front_and_back), "GL_LEFT names both colour buffers");
    f.context.enable(gl::blend, true);
    f.context.blend_func(gl::one, gl::one);
    f.context.color(0, 0.5, 0, 1);
    f.context.begin(gl::triangles);
    f.context.vertex(-1, -1, 0, 1); f.context.vertex(3, -1, 0, 1); f.context.vertex(-1, 3, 0, 1);
    f.context.end();
    // (5, 2) is off the clipped fan's internal diagonal (gl_raster has no tie rule).
    expect(f.pixel(gl::front, 5, 2) == bgra(255, 128, 0, 255) && f.pixel(gl::back, 5, 2) == bgra(0, 128, 255, 255),
           "GL_FRONT_AND_BACK writes both buffers, each blended with its own contents");
    f.context.draw_buffer(gl::right);
    expect(f.context.get_error() == gl::invalid_operation, "a missing right buffer is GL_INVALID_OPERATION");
    f.context.draw_buffer(gl::none);
    f.context.color(1, 1, 1, 1);
    f.context.enable(gl::blend, false);
    f.context.begin(gl::triangles);
    f.context.vertex(-1, -1, 0, 1); f.context.vertex(3, -1, 0, 1); f.context.vertex(-1, 3, 0, 1);
    f.context.end();
    expect(f.pixel(gl::back, 5, 2) == bgra(0, 128, 255, 255), "GL_NONE writes no colour buffer");
}

void test_mipmap_levels() {
    Fixture f(16, 16);
    pixel_projection(f, 16);
    uint32_t name = 0;
    f.context.gen_textures(1, &name);
    f.context.bind_texture(gl::texture_2d, name);
    const uint8_t level_color[3][4] = {{255, 0, 0, 255}, {0, 255, 0, 255}, {0, 0, 255, 255}};
    for (int32_t level = 0, dim = 4; dim >= 1; ++level, dim /= 2) {
        std::vector<uint8_t> texels(static_cast<size_t>(dim) * dim * 4);
        for (size_t i = 0; i < texels.size(); ++i) texels[i] = level_color[level][i % 4];
        f.context.tex_image_2d(gl::texture_2d, level, static_cast<int32_t>(gl::rgba), dim, dim, 0, gl::rgba,
                               gl::type_unsigned_byte, texels.data());
    }
    const float min[1] = {static_cast<float>(gl::nearest_mipmap_nearest)}, mag[1] = {static_cast<float>(gl::nearest)};
    f.context.tex_parameter(gl::texture_2d, gl::texture_min_filter, min);
    f.context.tex_parameter(gl::texture_2d, gl::texture_mag_filter, mag);
    f.context.enable(gl::texture_2d, true);
    f.context.color(1, 1, 1, 1);
    auto quad = [&](float x, float y, float extent) {
        f.context.begin(gl::quads);
        f.context.tex_coord(0, 0, 0, 1); f.context.vertex(x, y, 0, 1);
        f.context.tex_coord(1, 0, 0, 1); f.context.vertex(x + extent, y, 0, 1);
        f.context.tex_coord(1, 1, 0, 1); f.context.vertex(x + extent, y + extent, 0, 1);
        f.context.tex_coord(0, 1, 0, 1); f.context.vertex(x, y + extent, 0, 1);
        f.context.end();
    };
    quad(0, 0, 8);    // 4 texels over 8 pixels: lambda -1, magnification (level 0)
    quad(8, 0, 2);    // lambda 1: ceil(1.5) - 1 = level 1
    quad(12, 0, 1);   // lambda 2: level 2
    quad(8, 8, 0.5f); // lambda 3: clamped to the last level
    expect(f.pixel(gl::back, 3, 3) == bgra(255, 0, 0, 255), "a magnified mipmapped texture samples level 0");
    expect(f.pixel(gl::back, 8, 0) == bgra(0, 255, 0, 255), "lambda 1 with NEAREST_MIPMAP_NEAREST samples level 1");
    expect(f.pixel(gl::back, 12, 0) == bgra(0, 0, 255, 255), "lambda 2 samples level 2");
    expect(f.pixel(gl::back, 8, 8) == bgra(0, 0, 255, 255), "lambda beyond q clamps to the last level");
    const float linear[1] = {static_cast<float>(gl::linear_mipmap_linear)};
    f.context.tex_parameter(gl::texture_2d, gl::texture_min_filter, linear);
    quad(10, 10, 2);  // lambda 1 exactly: level 1, weight 0
    expect(f.pixel(gl::back, 10, 10) == bgra(0, 255, 0, 255), "LINEAR_MIPMAP_LINEAR at integer lambda is one level");
    TextureSampler sampler{};
    sampler.min_filter = gl::nearest_mipmap_linear;
    sampler.mag_filter = gl::linear;
    expect(texture_lod_threshold(sampler) == 0.5f && texture_levels(sampler, 0.4f, 2).magnify &&
               !texture_levels(sampler, 0.6f, 2).magnify,
           "c = 0.5 for LINEAR magnification with NEAREST_MIPMAP_* minification");
    const TextureLevels mid = texture_levels(sampler, 1.25f, 2);
    expect(mid.level_a == 1 && mid.level_b == 2 && mid.weight == 0.25f, "MIPMAP_LINEAR blends floor and floor + 1");
}

int main() {
    test_raster_position();
    test_draw_pixels_zoom();
    test_pixel_transfer();
    test_depth_stencil_pixels();
    test_bitmap();
    test_copy_pixels();
    test_line_stipple_restart();
    test_polygon_stipple();
    test_front_and_back();
    test_mipmap_levels();
    test_flat_provoking();
    test_face_culling();
    test_two_sided_color();
    test_clear();
    test_flat_triangle();
    test_smooth_triangle();
    test_depth();
    test_display_list();
    test_vertex_arrays();
    test_texture_modulate();
    test_strings();
    std::printf("opengl_context: %u checks, %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

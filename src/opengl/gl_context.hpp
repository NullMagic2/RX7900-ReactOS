/*
Module: rx7900.opengl.gl_context
Purpose: Portable OpenGL 1.1 fixed-function state machine driving a rasterization
backend.
Created: 2026-10-23
Architecture: The ICD (src/opengl/icd) wraps this with the ReactOS WGL entry
points and the 336-entry dispatch table; the host tests drive it directly. The
context performs transform, lighting, texgen, primitive assembly and view-volume
plus user-plane clipping (gl_clip), then issues clip-space primitives, clears and
pixel operations to a Backend (gl_raster for software, a GFX11 backend later).
It allocates texture image storage through a caller-supplied allocator and uses
only fixed-capacity internal storage otherwise. One context is current on one
thread at a time; the caller serializes access. No OS or Windows dependency.
*/
#pragma once

#include "gl_api.hpp"
#include "gl_clip.hpp"
#include "gl_pipeline.hpp"

namespace rx7900::opengl {

constexpr uint32_t context_matrix_stack_depth = 32;
constexpr uint32_t context_max_lights = 8;
constexpr uint32_t context_max_textures = 256;
constexpr uint32_t context_max_lists = 4096;
constexpr uint32_t context_list_arena = 1u << 18;   // recorded commands across all lists
constexpr uint32_t context_list_bytes = 1u << 20;   // recorded bitmap images across all lists
constexpr uint32_t context_vertex_buffer = 4096;    // immediate-mode vertices per primitive batch
constexpr uint32_t context_max_pixel_map = 256;     // GL_MAX_PIXEL_MAP_TABLE
constexpr uint32_t context_pixel_maps = 10;         // GL_PIXEL_MAP_I_TO_I .. GL_PIXEL_MAP_A_TO_A
// Pixel rectangles are converted and drawn in bands of at most this many pixels.
constexpr uint32_t context_pixel_band = 1u << 18;

// Heap for texture images. malloc/free by default; the ICD may pass its own.
struct Allocator final {
    void* context = nullptr;
    void* (*allocate)(void*, size_t) noexcept = nullptr;
    void (*release)(void*, void*) noexcept = nullptr;
};

Allocator default_allocator() noexcept;

class Context final {
public:
    // backend and allocator outlive the context; backend may be null until the
    // first make-current resize. The share group (textures, lists) is this
    // context's own unless shared through adopt_share_group.
    Context(Backend* backend, Allocator allocator) noexcept;
    ~Context() noexcept;
    Context(const Context&) = delete;
    Context& operator=(const Context&) = delete;

    void set_backend(Backend* backend) noexcept { backend_ = backend; }
    Backend* backend() const noexcept { return backend_; }
    // Point this context's textures and display lists at another context's share
    // group (wglShareLists). Only valid before either created objects.
    bool adopt_share_group(Context& other) noexcept;

    // ---- Error and strings ----
    gl::Enum get_error() noexcept;
    const char* get_string(gl::Enum name) noexcept;

    // ---- Immediate mode ----
    void begin(gl::Enum mode) noexcept;
    void end() noexcept;
    void vertex(double x, double y, double z, double w) noexcept;
    void color(double r, double g, double b, double a) noexcept;
    void normal(double x, double y, double z) noexcept;
    void tex_coord(double s, double t, double r, double q) noexcept;
    void edge_flag(bool flag) noexcept;

    // ---- Matrices ----
    void matrix_mode(gl::Enum mode) noexcept;
    void load_identity() noexcept;
    void load_matrix(const double* values) noexcept;
    void mult_matrix(const double* values) noexcept;
    void push_matrix() noexcept;
    void pop_matrix() noexcept;
    void translate(double x, double y, double z) noexcept;
    void rotate(double angle, double x, double y, double z) noexcept;
    void scale(double x, double y, double z) noexcept;
    void frustum(double l, double r, double b, double t, double n, double f) noexcept;
    void ortho(double l, double r, double b, double t, double n, double f) noexcept;

    // ---- Viewport / depth range / scissor ----
    void viewport(int32_t x, int32_t y, int32_t width, int32_t height) noexcept;
    void depth_range(double near_val, double far_val) noexcept;
    void scissor(int32_t x, int32_t y, int32_t width, int32_t height) noexcept;

    // ---- Enable / disable ----
    void enable(gl::Enum cap, bool value) noexcept;
    bool is_enabled(gl::Enum cap) noexcept;

    // ---- Clears ----
    void clear(gl::Bitfield mask) noexcept;
    void clear_color(double r, double g, double b, double a) noexcept;
    void clear_depth(double depth) noexcept;
    void clear_stencil(int32_t s) noexcept;

    // ---- Per-fragment state ----
    void shade_model(gl::Enum mode) noexcept;
    void cull_face(gl::Enum mode) noexcept;
    void front_face(gl::Enum mode) noexcept;
    void polygon_mode(gl::Enum face, gl::Enum mode) noexcept;
    void polygon_offset(float factor, float units) noexcept;
    void line_width(float width) noexcept;
    void point_size(float size) noexcept;
    void line_stipple(int32_t factor, uint16_t pattern) noexcept;
    void depth_func(gl::Enum func) noexcept;
    void depth_mask(bool flag) noexcept;
    void alpha_func(gl::Enum func, float ref) noexcept;
    void blend_func(gl::Enum src, gl::Enum dst) noexcept;
    void logic_op(gl::Enum op) noexcept;
    void stencil_func(gl::Enum func, int32_t ref, uint32_t mask) noexcept;
    void stencil_op(gl::Enum fail, gl::Enum depth_fail, gl::Enum depth_pass) noexcept;
    void stencil_mask(uint32_t mask) noexcept;
    void color_mask(bool r, bool g, bool b, bool a) noexcept;
    void hint(gl::Enum target, gl::Enum mode) noexcept;
    void draw_buffer(gl::Enum buffer) noexcept;
    void read_buffer(gl::Enum buffer) noexcept;

    // ---- Lighting ----
    void light(gl::Enum light, gl::Enum pname, const float* values) noexcept;
    void light_model(gl::Enum pname, const float* values) noexcept;
    void material(gl::Enum face, gl::Enum pname, const float* values) noexcept;
    void color_material(gl::Enum face, gl::Enum mode) noexcept;

    // ---- Fog ----
    void fog(gl::Enum pname, const float* values) noexcept;

    // ---- Textures ----
    void gen_textures(int32_t count, uint32_t* names) noexcept;
    void delete_textures(int32_t count, const uint32_t* names) noexcept;
    void bind_texture(gl::Enum target, uint32_t name) noexcept;
    bool is_texture(uint32_t name) noexcept;
    void tex_image_2d(gl::Enum target, int32_t level, int32_t internal_format, int32_t width,
                      int32_t height, int32_t border, gl::Enum format, gl::Enum type,
                      const void* pixels) noexcept;
    void tex_sub_image_2d(gl::Enum target, int32_t level, int32_t xoffset, int32_t yoffset,
                          int32_t width, int32_t height, gl::Enum format, gl::Enum type,
                          const void* pixels) noexcept;
    void tex_parameter(gl::Enum target, gl::Enum pname, const float* values) noexcept;
    void tex_env(gl::Enum target, gl::Enum pname, const float* values) noexcept;
    void pixel_store(gl::Enum pname, int32_t value) noexcept;

    // ---- Vertex arrays ----
    void vertex_pointer(int32_t size, gl::Enum type, int32_t stride, const void* pointer) noexcept;
    void normal_pointer(gl::Enum type, int32_t stride, const void* pointer) noexcept;
    void color_pointer(int32_t size, gl::Enum type, int32_t stride, const void* pointer) noexcept;
    void tex_coord_pointer(int32_t size, gl::Enum type, int32_t stride, const void* pointer) noexcept;
    void enable_client_state(gl::Enum array, bool value) noexcept;
    void array_element(int32_t index) noexcept;
    void draw_arrays(gl::Enum mode, int32_t first, int32_t count) noexcept;
    void draw_elements(gl::Enum mode, int32_t count, gl::Enum type, const void* indices) noexcept;

    // ---- Display lists ----
    uint32_t gen_lists(int32_t range) noexcept;
    void new_list(uint32_t list, gl::Enum mode) noexcept;
    void end_list() noexcept;
    void call_list(uint32_t list) noexcept;
    void call_lists(int32_t count, gl::Enum type, const void* lists) noexcept;
    void list_base(uint32_t base) noexcept;
    void delete_lists(uint32_t list, int32_t range) noexcept;
    bool is_list(uint32_t list) noexcept;

    // ---- Raster position and pixel rectangles (GL 1.1 2.12, 3.6, 4.3) ----
    void raster_pos(double x, double y, double z, double w) noexcept;
    // bitmap: unpacked with the GL_UNPACK_* state; null with a zero size only moves.
    void bitmap(int32_t width, int32_t height, float xorig, float yorig, float xmove, float ymove,
                const uint8_t* bitmap) noexcept;
    void draw_pixels(int32_t width, int32_t height, gl::Enum format, gl::Enum type,
                     const void* pixels) noexcept;
    void copy_pixels(int32_t x, int32_t y, int32_t width, int32_t height, gl::Enum type) noexcept;
    void pixel_zoom(float x, float y) noexcept;
    void pixel_transfer(gl::Enum pname, float value) noexcept;
    // values: size floats; RGBA maps hold [0, 1] values, index maps integers.
    void pixel_map(gl::Enum map, int32_t size, const float* values) noexcept;
    // values receives pixel_map_size(map) floats.
    void get_pixel_map(gl::Enum map, float* values) noexcept;
    uint32_t pixel_map_size(gl::Enum map) noexcept;
    // 32x32 bitmap unpacked/packed with the GL_UNPACK_* / GL_PACK_ALIGNMENT state.
    void polygon_stipple(const uint8_t* mask) noexcept;
    void get_polygon_stipple(uint8_t* mask) noexcept;

    // ---- Framebuffer read/write and sync ----
    void read_pixels(int32_t x, int32_t y, int32_t width, int32_t height, gl::Enum format,
                     gl::Enum type, void* pixels) noexcept;
    void finish() noexcept;
    void flush() noexcept;

    // ---- Queries ----
    void get_integerv(gl::Enum name, int32_t* values) noexcept;
    void get_floatv(gl::Enum name, float* values) noexcept;
    void get_booleanv(gl::Enum name, uint8_t* values) noexcept;

    // Present the back buffer (used by the ICD's DrvSwapBuffers via the backend).
    void swap_buffers() noexcept;

private:
    struct Light final {
        Vec4 ambient{0, 0, 0, 1};
        Vec4 diffuse{0, 0, 0, 1};
        Vec4 specular{0, 0, 0, 1};
        Vec4 position{0, 0, 1, 0};
        Vec3 spot_direction{0, 0, -1};
        float spot_exponent = 0;
        float spot_cutoff = 180;
        float constant_attenuation = 1;
        float linear_attenuation = 0;
        float quadratic_attenuation = 0;
        bool enabled = false;
    };

    struct Material final {
        Vec4 ambient{0.2f, 0.2f, 0.2f, 1};
        Vec4 diffuse{0.8f, 0.8f, 0.8f, 1};
        Vec4 specular{0, 0, 0, 1};
        Vec4 emission{0, 0, 0, 1};
        float shininess = 0;
    };

    struct ShareGroup final {
        Texture textures[context_max_textures]{};
        uint32_t texture_count = 0;
        struct List final {
            bool used = false;
            uint32_t first = 0;
            uint32_t count = 0;
        };
        List lists[context_max_lists]{};
        uint32_t ref_count = 1;
    };

    struct ListCommand;

    Matrix4& current_stack_top() noexcept;
    void set_error(gl::Enum error) noexcept;
    void emit_vertex(const Vec4& object_position) noexcept;
    void flush_primitive() noexcept;
    void assemble_and_draw() noexcept;
    DrawState build_draw_state() noexcept;
    RasterVertex transform(const Vec4& object_position, const Vec4& color_in, const Vec3& normal_in,
                           const Vec4& texcoord_in) noexcept;
    Vec4 apply_lighting(const Vec4& eye, const Vec3& eye_normal, const Vec4& base_color,
                        bool front) noexcept;
    Texture* bound_texture() noexcept;
    Texture* find_texture(uint32_t name) noexcept;
    bool record(const ListCommand& command) noexcept;
    void execute(const ListCommand& command) noexcept;
    // Bitmap rows already located: stride bytes per row, the first pixel at
    // bit_offset of each row, MSB first unless lsb_first.
    void bitmap_rows(int32_t width, int32_t height, float xorig, float yorig, float xmove, float ymove,
                     const uint8_t* rows, size_t stride, uint32_t bit_offset, bool lsb_first) noexcept;
    // Pixel transfer of one RGBA group, one depth value and one stencil index.
    void transfer_color(float rgba[4]) const noexcept;
    float transfer_depth(float depth) const noexcept;
    uint8_t transfer_stencil(int64_t index) const noexcept;
    // Draws converted image rows [first_row, first_row + rows) at the raster position.
    void draw_image_band(const DrawState& state, PixelFragments& fragments, uint32_t first_row,
                         uint32_t rows) noexcept;
    bool linear_transfer() const noexcept;
    ShareGroup* share() noexcept { return share_; }

    Backend* backend_;
    Allocator allocator_;
    ShareGroup own_share_{};
    ShareGroup* share_ = &own_share_;

    gl::Enum error_ = gl::no_error;

    // Matrix stacks.
    gl::Enum matrix_mode_ = gl::modelview;
    Matrix4 modelview_[context_matrix_stack_depth];
    Matrix4 projection_[context_matrix_stack_depth];
    Matrix4 texture_matrix_[context_matrix_stack_depth];
    uint32_t modelview_depth_ = 0, projection_depth_ = 0, texture_depth_ = 0;

    // Current vertex attributes.
    Vec4 current_color_{1, 1, 1, 1};
    Vec3 current_normal_{0, 0, 1};
    Vec4 current_texcoord_{0, 0, 0, 1};
    bool current_edge_flag_ = true;

    // Primitive assembly.
    bool in_begin_ = false;
    gl::Enum primitive_mode_ = 0;
    RasterVertex vertices_[context_vertex_buffer];
    bool vertex_edge_[context_vertex_buffer];
    uint32_t vertex_count_ = 0;

    // Viewport and depth range.
    int32_t viewport_x_ = 0, viewport_y_ = 0, viewport_w_ = 0, viewport_h_ = 0;
    double depth_near_ = 0, depth_far_ = 1;

    // Scissor.
    bool scissor_enabled_ = false;
    int32_t scissor_x_ = 0, scissor_y_ = 0, scissor_w_ = 0, scissor_h_ = 0;

    // Capabilities and per-fragment state.
    bool depth_test_ = false;
    gl::Enum depth_func_ = gl::less;
    bool depth_write_ = true;
    bool blend_ = false;
    gl::Enum blend_src_ = gl::one, blend_dst_ = gl::zero;
    bool alpha_test_ = false;
    gl::Enum alpha_func_ = gl::always;
    float alpha_ref_ = 0;
    bool stencil_test_ = false;
    StencilFace stencil_{};
    bool logic_op_ = false;
    gl::Enum logic_op_mode_ = gl::logic_copy;
    bool color_mask_[4] = {true, true, true, true};
    bool cull_ = false;
    gl::Enum cull_mode_ = gl::back;
    bool front_ccw_ = true;
    bool dither_ = true;
    gl::Enum shade_model_ = gl::smooth;
    gl::Enum polygon_mode_ = gl::fill;
    bool polygon_offset_fill_ = false;
    float polygon_offset_factor_ = 0, polygon_offset_units_ = 0;
    float point_size_ = 1, line_width_ = 1;
    bool line_stipple_enabled_ = false;
    int32_t line_stipple_factor_ = 1;
    uint16_t line_stipple_pattern_ = 0xffff;
    bool scissor_test_enabled_ = false;
    gl::Enum draw_buffer_ = gl::back;
    gl::Enum read_buffer_ = gl::back;

    // Clear values.
    Vec4 clear_color_{0, 0, 0, 0};
    double clear_depth_ = 1;
    int32_t clear_stencil_ = 0;

    // Lighting.
    bool lighting_ = false;
    bool normalize_ = false;
    bool color_material_ = false;
    gl::Enum color_material_mode_ = gl::ambient_and_diffuse;
    Light lights_[context_max_lights];
    Material material_front_{};
    Material material_back_{};
    Vec4 light_model_ambient_{0.2f, 0.2f, 0.2f, 1};
    bool light_model_two_side_ = false;

    // Fog.
    bool fog_enabled_ = false;
    gl::Enum fog_mode_ = gl::exp;
    float fog_density_ = 1;
    float fog_start_ = 0, fog_end_ = 1;
    Vec4 fog_color_{0, 0, 0, 0};

    // Raster position (GL 1.1 2.12): window x, y, z and clip w.
    bool raster_valid_ = true;
    Vec4 raster_window_{0, 0, 0, 1};
    Vec4 raster_color_{1, 1, 1, 1};
    Vec4 raster_texcoord_{0, 0, 0, 1};
    float raster_fog_ = 1;
    float raster_distance_ = 0;

    // Pixel transfer, maps and zoom (GL 1.1 3.6.3, 3.6.4).
    struct PixelMap final {
        uint32_t size = 1;
        float values[context_max_pixel_map]{};
    };
    PixelMap pixel_maps_[context_pixel_maps]{};
    bool map_color_ = false;
    bool map_stencil_ = false;
    int32_t index_shift_ = 0;
    int32_t index_offset_ = 0;
    float color_scale_[4] = {1, 1, 1, 1};
    float color_bias_[4] = {0, 0, 0, 0};
    float depth_scale_ = 1;
    float depth_bias_ = 0;
    float zoom_x_ = 1, zoom_y_ = 1;

    // Polygon stipple (GL 1.1 3.5.2): row y & 31, bit 31 is x & 31 == 0.
    bool polygon_stipple_enabled_ = false;
    uint32_t polygon_stipple_rows_[32] = {
        0xffffffffu, 0xffffffffu, 0xffffffffu, 0xffffffffu, 0xffffffffu, 0xffffffffu, 0xffffffffu, 0xffffffffu,
        0xffffffffu, 0xffffffffu, 0xffffffffu, 0xffffffffu, 0xffffffffu, 0xffffffffu, 0xffffffffu, 0xffffffffu,
        0xffffffffu, 0xffffffffu, 0xffffffffu, 0xffffffffu, 0xffffffffu, 0xffffffffu, 0xffffffffu, 0xffffffffu,
        0xffffffffu, 0xffffffffu, 0xffffffffu, 0xffffffffu, 0xffffffffu, 0xffffffffu, 0xffffffffu, 0xffffffffu};

    // Texturing.
    bool texture_2d_enabled_ = false;
    uint32_t bound_texture_2d_ = 0;
    gl::Enum texture_env_mode_ = gl::modulate;
    Vec4 texture_env_color_{0, 0, 0, 0};
    int32_t unpack_alignment_ = 4;
    int32_t pack_alignment_ = 4;
    int32_t unpack_row_length_ = 0;
    int32_t unpack_skip_rows_ = 0;
    int32_t unpack_skip_pixels_ = 0;
    bool unpack_swap_bytes_ = false;
    bool unpack_lsb_first_ = false;

    // Vertex arrays.
    struct Array final {
        bool enabled = false;
        int32_t size = 4;
        gl::Enum type = gl::type_float;
        int32_t stride = 0;
        const void* pointer = nullptr;
    };
    Array vertex_array_, normal_array_, color_array_, texcoord_array_;

    // Display list recording.
    bool compiling_ = false;
    bool compile_execute_ = false;
    uint32_t compiling_list_ = 0;
    uint32_t compile_start_ = 0;
    uint32_t list_base_ = 0;
    uint32_t replay_depth_ = 0;
    ListCommand* arena_ = nullptr;      // context_list_arena commands, heap-owned
    uint32_t arena_used_ = 0;
    uint8_t* list_bytes_ = nullptr;     // context_list_bytes, heap-owned (bitmap images)
    uint32_t list_bytes_used_ = 0;
};

} // namespace rx7900::opengl

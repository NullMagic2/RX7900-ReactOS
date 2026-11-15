/*
Module: rx7900.opengl.gl_context
Purpose: OpenGL 1.1 fixed-function state machine, transform, lighting and
primitive assembly over a rasterization backend.
Created: 2026-10-23
Architecture: See gl_context.hpp. Implements the OpenGL 1.1 specification
chapters 2 (transform, lighting), 2.11 (clipping, via gl_clip) and the state for
chapters 3 and 4 (handed to the Backend). No third-party code. User mode.
*/
#include "gl_context.hpp"

#include <cmath>
#include <cstdlib>
#include <cstring>

namespace rx7900::opengl {

namespace {

void* malloc_allocate(void*, size_t bytes) noexcept { return std::malloc(bytes); }
void malloc_release(void*, void* pointer) noexcept { std::free(pointer); }

Vec4 from4(const float* v) noexcept { return {v[0], v[1], v[2], v[3]}; }

float clamp01(float value) noexcept { return value < 0 ? 0 : (value > 1 ? 1 : value); }

// Reset a texture object to its default state. Avoids a large aggregate
// temporary (which triggers a GCC MinGW gimplify ICE at -O1); the caller frees
// the level texels beforehand.
void reset_texture(Texture& texture) noexcept {
    texture.name = 0;
    texture.target = 0;
    for (TextureLevel& level : texture.levels) { level = TextureLevel{}; }
    texture.sampler = TextureSampler{};
    texture.priority = 1;
    texture.generation = 1;
}

uint32_t base_format_of(int32_t internal_format) noexcept {
    switch (internal_format) {
    case 1: case gl::luminance: case gl::luminance4: case gl::luminance16: return gl::luminance;
    case 2: case gl::luminance_alpha: case gl::luminance4_alpha4: case gl::luminance16_alpha16:
        return gl::luminance_alpha;
    case 3: case gl::rgb: case gl::r3_g3_b2: case gl::rgb4: case gl::rgb16: return gl::rgb;
    case gl::alpha: case gl::alpha4: case gl::alpha8: case gl::alpha16: return gl::alpha;
    case gl::intensity: case gl::intensity4: case gl::intensity16: return gl::intensity;
    default: return gl::rgba;
    }
}

} // namespace

Allocator default_allocator() noexcept { return {nullptr, malloc_allocate, malloc_release}; }

// Recordable display-list command. Only the fixed-function subset classic apps
// put inside lists is recorded; other calls during compilation execute directly.
struct Context::ListCommand final {
    enum class Op : uint8_t {
        begin, end, vertex, color, normal, tex_coord, shade_model, material, enable, disable,
        color_material, translate, rotate, scale, call_list, raster_pos, bitmap
    };
    Op op;
    uint32_t e0 = 0;
    uint32_t e1 = 0;
    double v[6] = {};
};

Context::Context(Backend* backend, Allocator allocator) noexcept
    : backend_(backend), allocator_(allocator) {
    if (allocator_.allocate == nullptr || allocator_.release == nullptr) {
        allocator_ = default_allocator();
    }
    modelview_[0] = projection_[0] = texture_matrix_[0] = matrix_identity();
    arena_ = static_cast<ListCommand*>(
        allocator_.allocate(allocator_.context, sizeof(ListCommand) * context_list_arena));
    list_bytes_ = static_cast<uint8_t*>(allocator_.allocate(allocator_.context, context_list_bytes));
    for (uint32_t map = 0; map < context_pixel_maps; ++map) {
        pixel_maps_[map].size = 1;
    }
}

Context::~Context() noexcept {
    for (uint32_t index = 0; index < context_max_textures; ++index) {
        Texture& texture = share_->textures[index];
        for (TextureLevel& level : texture.levels) {
            if (level.texels != nullptr) {
                allocator_.release(allocator_.context, level.texels);
                level.texels = nullptr;
            }
        }
    }
    if (arena_ != nullptr) {
        allocator_.release(allocator_.context, arena_);
        arena_ = nullptr;
    }
    if (list_bytes_ != nullptr) {
        allocator_.release(allocator_.context, list_bytes_);
        list_bytes_ = nullptr;
    }
}

bool Context::adopt_share_group(Context& other) noexcept {
    if (share_->texture_count != 0 || other.share_ == share_) {
        return false;
    }
    share_ = other.share_;
    ++share_->ref_count;
    return true;
}

void Context::set_error(gl::Enum error) noexcept {
    if (error_ == gl::no_error) {
        error_ = error;
    }
}

gl::Enum Context::get_error() noexcept {
    const gl::Enum result = error_;
    error_ = gl::no_error;
    return result;
}

const char* Context::get_string(gl::Enum name) noexcept {
    switch (name) {
    case gl::vendor: return "RX7900-ReactOS";
    case gl::renderer: return "Radeon RX 7900 XTX (Navi31) fixed-function";
    case gl::version: return "1.1.0";
    case gl::extensions: return "GL_EXT_bgra";
    default: set_error(gl::invalid_enum); return "";
    }
}

Matrix4& Context::current_stack_top() noexcept {
    if (matrix_mode_ == gl::projection) {
        return projection_[projection_depth_];
    }
    if (matrix_mode_ == gl::texture) {
        return texture_matrix_[texture_depth_];
    }
    return modelview_[modelview_depth_];
}

// ---- Matrices ----

void Context::matrix_mode(gl::Enum mode) noexcept {
    if (mode != gl::modelview && mode != gl::projection && mode != gl::texture) {
        set_error(gl::invalid_enum);
        return;
    }
    matrix_mode_ = mode;
}

void Context::load_identity() noexcept { current_stack_top() = matrix_identity(); }

void Context::load_matrix(const double* values) noexcept {
    Matrix4 matrix{};
    for (uint32_t index = 0; index < 16; ++index) {
        matrix.values[index] = static_cast<float>(values[index]);
    }
    current_stack_top() = matrix;
}

void Context::mult_matrix(const double* values) noexcept {
    Matrix4 right{};
    for (uint32_t index = 0; index < 16; ++index) {
        right.values[index] = static_cast<float>(values[index]);
    }
    current_stack_top() = matrix_multiply(current_stack_top(), right);
}

void Context::push_matrix() noexcept {
    if (matrix_mode_ == gl::projection) {
        if (projection_depth_ + 1 >= context_matrix_stack_depth) { set_error(gl::stack_overflow); return; }
        projection_[projection_depth_ + 1] = projection_[projection_depth_];
        ++projection_depth_;
    } else if (matrix_mode_ == gl::texture) {
        if (texture_depth_ + 1 >= context_matrix_stack_depth) { set_error(gl::stack_overflow); return; }
        texture_matrix_[texture_depth_ + 1] = texture_matrix_[texture_depth_];
        ++texture_depth_;
    } else {
        if (modelview_depth_ + 1 >= context_matrix_stack_depth) { set_error(gl::stack_overflow); return; }
        modelview_[modelview_depth_ + 1] = modelview_[modelview_depth_];
        ++modelview_depth_;
    }
}

void Context::pop_matrix() noexcept {
    uint32_t& depth = matrix_mode_ == gl::projection ? projection_depth_
                      : matrix_mode_ == gl::texture  ? texture_depth_
                                                     : modelview_depth_;
    if (depth == 0) {
        set_error(gl::stack_underflow);
        return;
    }
    --depth;
}

void Context::translate(double x, double y, double z) noexcept {
    Matrix4 m = matrix_identity();
    m.values[12] = static_cast<float>(x);
    m.values[13] = static_cast<float>(y);
    m.values[14] = static_cast<float>(z);
    current_stack_top() = matrix_multiply(current_stack_top(), m);
}

void Context::rotate(double angle, double x, double y, double z) noexcept {
    const double length = std::sqrt(x * x + y * y + z * z);
    if (length == 0) {
        return;
    }
    x /= length;
    y /= length;
    z /= length;
    const double radians = angle * 0.017453292519943295;
    const double c = std::cos(radians);
    const double s = std::sin(radians);
    const double t = 1 - c;
    Matrix4 m = matrix_identity();
    m.values[0] = static_cast<float>(t * x * x + c);
    m.values[1] = static_cast<float>(t * x * y + s * z);
    m.values[2] = static_cast<float>(t * x * z - s * y);
    m.values[4] = static_cast<float>(t * x * y - s * z);
    m.values[5] = static_cast<float>(t * y * y + c);
    m.values[6] = static_cast<float>(t * y * z + s * x);
    m.values[8] = static_cast<float>(t * x * z + s * y);
    m.values[9] = static_cast<float>(t * y * z - s * x);
    m.values[10] = static_cast<float>(t * z * z + c);
    current_stack_top() = matrix_multiply(current_stack_top(), m);
}

void Context::scale(double x, double y, double z) noexcept {
    Matrix4 m = matrix_identity();
    m.values[0] = static_cast<float>(x);
    m.values[5] = static_cast<float>(y);
    m.values[10] = static_cast<float>(z);
    current_stack_top() = matrix_multiply(current_stack_top(), m);
}

void Context::frustum(double l, double r, double b, double t, double n, double f) noexcept {
    if (l == r || b == t || n == f || n <= 0 || f <= 0) {
        set_error(gl::invalid_value);
        return;
    }
    Matrix4 m{};
    m.values[0] = static_cast<float>(2 * n / (r - l));
    m.values[5] = static_cast<float>(2 * n / (t - b));
    m.values[8] = static_cast<float>((r + l) / (r - l));
    m.values[9] = static_cast<float>((t + b) / (t - b));
    m.values[10] = static_cast<float>(-(f + n) / (f - n));
    m.values[11] = -1;
    m.values[14] = static_cast<float>(-2 * f * n / (f - n));
    current_stack_top() = matrix_multiply(current_stack_top(), m);
}

void Context::ortho(double l, double r, double b, double t, double n, double f) noexcept {
    if (l == r || b == t || n == f) {
        set_error(gl::invalid_value);
        return;
    }
    Matrix4 m = matrix_identity();
    m.values[0] = static_cast<float>(2 / (r - l));
    m.values[5] = static_cast<float>(2 / (t - b));
    m.values[10] = static_cast<float>(-2 / (f - n));
    m.values[12] = static_cast<float>(-(r + l) / (r - l));
    m.values[13] = static_cast<float>(-(t + b) / (t - b));
    m.values[14] = static_cast<float>(-(f + n) / (f - n));
    current_stack_top() = matrix_multiply(current_stack_top(), m);
}

// ---- Viewport / depth / scissor ----

void Context::viewport(int32_t x, int32_t y, int32_t width, int32_t height) noexcept {
    if (width < 0 || height < 0) {
        set_error(gl::invalid_value);
        return;
    }
    viewport_x_ = x;
    viewport_y_ = y;
    viewport_w_ = width;
    viewport_h_ = height;
}

void Context::depth_range(double near_val, double far_val) noexcept {
    depth_near_ = clamp01(static_cast<float>(near_val));
    depth_far_ = clamp01(static_cast<float>(far_val));
}

void Context::scissor(int32_t x, int32_t y, int32_t width, int32_t height) noexcept {
    if (width < 0 || height < 0) {
        set_error(gl::invalid_value);
        return;
    }
    scissor_x_ = x;
    scissor_y_ = y;
    scissor_w_ = width;
    scissor_h_ = height;
}

// ---- Enable / disable ----

void Context::enable(gl::Enum cap, bool value) noexcept {
    switch (cap) {
    case gl::depth_test: depth_test_ = value; break;
    case gl::blend: blend_ = value; break;
    case gl::alpha_test: alpha_test_ = value; break;
    case gl::stencil_test: stencil_test_ = value; break;
    case gl::color_logic_op: logic_op_ = value; break;
    case gl::cull_face: cull_ = value; break;
    case gl::dither: dither_ = value; break;
    case gl::scissor_test: scissor_test_enabled_ = value; break;
    case gl::lighting: lighting_ = value; break;
    case gl::normalize: normalize_ = value; break;
    case gl::color_material: color_material_ = value; break;
    case gl::fog: fog_enabled_ = value; break;
    case gl::texture_2d: texture_2d_enabled_ = value; break;
    case gl::polygon_offset_fill: polygon_offset_fill_ = value; break;
    case gl::line_stipple: line_stipple_enabled_ = value; break;
    case gl::polygon_stipple: polygon_stipple_enabled_ = value; break;
    case gl::light0: case gl::light0 + 1: case gl::light0 + 2: case gl::light0 + 3:
    case gl::light0 + 4: case gl::light0 + 5: case gl::light0 + 6: case gl::light0 + 7:
        lights_[cap - gl::light0].enabled = value;
        break;
    case gl::texture_1d: case gl::point_smooth: case gl::line_smooth: case gl::polygon_smooth:
    case gl::auto_normal: case gl::index_logic_op:
        break;   // accepted, no effect in this fixed-function subset
    default:
        set_error(gl::invalid_enum);
        break;
    }
}

bool Context::is_enabled(gl::Enum cap) noexcept {
    switch (cap) {
    case gl::depth_test: return depth_test_;
    case gl::blend: return blend_;
    case gl::alpha_test: return alpha_test_;
    case gl::stencil_test: return stencil_test_;
    case gl::color_logic_op: return logic_op_;
    case gl::cull_face: return cull_;
    case gl::dither: return dither_;
    case gl::scissor_test: return scissor_test_enabled_;
    case gl::lighting: return lighting_;
    case gl::normalize: return normalize_;
    case gl::color_material: return color_material_;
    case gl::fog: return fog_enabled_;
    case gl::texture_2d: return texture_2d_enabled_;
    case gl::polygon_offset_fill: return polygon_offset_fill_;
    case gl::line_stipple: return line_stipple_enabled_;
    case gl::polygon_stipple: return polygon_stipple_enabled_;
    case gl::light0: case gl::light0 + 1: case gl::light0 + 2: case gl::light0 + 3:
    case gl::light0 + 4: case gl::light0 + 5: case gl::light0 + 6: case gl::light0 + 7:
        return lights_[cap - gl::light0].enabled;
    default:
        set_error(gl::invalid_enum);
        return false;
    }
}

// ---- Clears ----

void Context::clear(gl::Bitfield mask) noexcept {
    if ((mask & ~(gl::color_buffer_bit | gl::depth_buffer_bit | gl::stencil_buffer_bit |
                  gl::accum_buffer_bit)) != 0) {
        set_error(gl::invalid_value);
        return;
    }
    if (backend_ == nullptr) {
        return;
    }
    ClearRequest request{};
    request.mask = mask & (gl::color_buffer_bit | gl::depth_buffer_bit | gl::stencil_buffer_bit);
    request.color = clear_color_;
    request.depth = clear_depth_;
    request.stencil = clear_stencil_;
    for (uint32_t index = 0; index < 4; ++index) {
        request.color_write[index] = color_mask_[index];
    }
    request.depth_write = depth_write_;
    request.stencil_write = stencil_.write_mask;
    request.scissor_enabled = scissor_test_enabled_;
    request.scissor_x = scissor_x_;
    request.scissor_y = scissor_y_;
    request.scissor_width = scissor_w_;
    request.scissor_height = scissor_h_;
    request.draw_buffer = draw_buffer_;
    backend_->clear(request);
}

void Context::clear_color(double r, double g, double b, double a) noexcept {
    clear_color_ = {clamp01(static_cast<float>(r)), clamp01(static_cast<float>(g)),
                    clamp01(static_cast<float>(b)), clamp01(static_cast<float>(a))};
}

void Context::clear_depth(double depth) noexcept { clear_depth_ = clamp01(static_cast<float>(depth)); }
void Context::clear_stencil(int32_t s) noexcept { clear_stencil_ = s; }

// ---- Per-fragment state ----

void Context::shade_model(gl::Enum mode) noexcept {
    if (compiling_) { ListCommand c{}; c.op = ListCommand::Op::shade_model; c.e0 = mode; if (record(c) && !compile_execute_) return; }
    if (mode != gl::flat && mode != gl::smooth) { set_error(gl::invalid_enum); return; }
    shade_model_ = mode;
}

void Context::cull_face(gl::Enum mode) noexcept {
    if (mode != gl::front && mode != gl::back && mode != gl::front_and_back) {
        set_error(gl::invalid_enum);
        return;
    }
    cull_mode_ = mode;
}

void Context::front_face(gl::Enum mode) noexcept {
    if (mode != gl::cw && mode != gl::ccw) { set_error(gl::invalid_enum); return; }
    front_ccw_ = mode == gl::ccw;
}

void Context::polygon_mode(gl::Enum face, gl::Enum mode) noexcept {
    if (face != gl::front && face != gl::back && face != gl::front_and_back) {
        set_error(gl::invalid_enum);
        return;
    }
    if (mode != gl::point && mode != gl::line && mode != gl::fill) {
        set_error(gl::invalid_enum);
        return;
    }
    polygon_mode_ = mode;
}

void Context::polygon_offset(float factor, float units) noexcept {
    polygon_offset_factor_ = factor;
    polygon_offset_units_ = units;
}

void Context::line_width(float width) noexcept {
    if (width <= 0) { set_error(gl::invalid_value); return; }
    line_width_ = width;
}

void Context::point_size(float size) noexcept {
    if (size <= 0) { set_error(gl::invalid_value); return; }
    point_size_ = size;
}

void Context::line_stipple(int32_t factor, uint16_t pattern) noexcept {
    line_stipple_factor_ = factor < 1 ? 1 : (factor > 256 ? 256 : factor);
    line_stipple_pattern_ = pattern;
}

void Context::depth_func(gl::Enum func) noexcept {
    if (func < gl::never || func > gl::always) { set_error(gl::invalid_enum); return; }
    depth_func_ = func;
}

void Context::depth_mask(bool flag) noexcept { depth_write_ = flag; }

void Context::alpha_func(gl::Enum func, float ref) noexcept {
    if (func < gl::never || func > gl::always) { set_error(gl::invalid_enum); return; }
    alpha_func_ = func;
    alpha_ref_ = clamp01(ref);
}

void Context::blend_func(gl::Enum src, gl::Enum dst) noexcept {
    blend_src_ = src;
    blend_dst_ = dst;
}

void Context::logic_op(gl::Enum op) noexcept {
    if (op < gl::logic_clear || op > gl::logic_set) { set_error(gl::invalid_enum); return; }
    logic_op_mode_ = op;
}

void Context::stencil_func(gl::Enum func, int32_t ref, uint32_t mask) noexcept {
    if (func < gl::never || func > gl::always) { set_error(gl::invalid_enum); return; }
    stencil_.func = func;
    stencil_.ref = ref < 0 ? 0 : (ref > 255 ? 255 : ref);
    stencil_.value_mask = mask;
}

void Context::stencil_op(gl::Enum fail, gl::Enum depth_fail, gl::Enum depth_pass) noexcept {
    stencil_.fail = fail;
    stencil_.depth_fail = depth_fail;
    stencil_.depth_pass = depth_pass;
}

void Context::stencil_mask(uint32_t mask) noexcept { stencil_.write_mask = mask; }

void Context::color_mask(bool r, bool g, bool b, bool a) noexcept {
    color_mask_[0] = r;
    color_mask_[1] = g;
    color_mask_[2] = b;
    color_mask_[3] = a;
}

void Context::hint(gl::Enum target, gl::Enum mode) noexcept {
    (void)target;
    if (mode != gl::fastest && mode != gl::nicest && mode != gl::dont_care) {
        set_error(gl::invalid_enum);
    }
}

// The drawable is mono with no auxiliary buffers: LEFT names both colour
// buffers, FRONT_LEFT/BACK_LEFT the single front/back buffer; right, aux and
// (single-buffered) back buffers do not exist (GL 1.1 4.2.1).
void Context::draw_buffer(gl::Enum buffer) noexcept {
    const bool doubled = backend_ == nullptr || backend_->format().double_buffered;
    switch (buffer) {
    case gl::none: draw_buffer_ = gl::none; break;
    case gl::front: case gl::front_left: draw_buffer_ = gl::front; break;
    case gl::back: case gl::back_left:
        if (!doubled) { set_error(gl::invalid_operation); return; }
        draw_buffer_ = gl::back;
        break;
    case gl::front_and_back: case gl::left:
        draw_buffer_ = doubled ? gl::front_and_back : gl::front;
        break;
    case gl::front_right: case gl::back_right: case gl::right: case gl::aux0:
        set_error(gl::invalid_operation);
        return;
    default: set_error(gl::invalid_enum); return;
    }
}

void Context::read_buffer(gl::Enum buffer) noexcept {
    const bool doubled = backend_ == nullptr || backend_->format().double_buffered;
    switch (buffer) {
    case gl::front: case gl::front_left: case gl::left: read_buffer_ = gl::front; break;
    case gl::back: case gl::back_left:
        if (!doubled) { set_error(gl::invalid_operation); return; }
        read_buffer_ = gl::back;
        break;
    case gl::front_right: case gl::back_right: case gl::right: case gl::aux0:
        set_error(gl::invalid_operation);
        return;
    default: set_error(gl::invalid_enum); return;
    }
}

// ---- Lighting ----

void Context::light(gl::Enum light, gl::Enum pname, const float* values) noexcept {
    if (light < gl::light0 || light >= gl::light0 + context_max_lights) {
        set_error(gl::invalid_enum);
        return;
    }
    Light& target = lights_[light - gl::light0];
    switch (pname) {
    case gl::ambient: target.ambient = from4(values); break;
    case gl::diffuse: target.diffuse = from4(values); break;
    case gl::specular: target.specular = from4(values); break;
    case gl::position: {
        const Vec4 eye = matrix_transform(modelview_[modelview_depth_], from4(values));
        target.position = eye;
        break;
    }
    case gl::spot_direction: {
        const Vec4 d = matrix_transform(modelview_[modelview_depth_], {values[0], values[1], values[2], 0});
        target.spot_direction = {d.x, d.y, d.z};
        break;
    }
    case gl::spot_exponent: target.spot_exponent = values[0]; break;
    case gl::spot_cutoff: target.spot_cutoff = values[0]; break;
    case gl::constant_attenuation: target.constant_attenuation = values[0]; break;
    case gl::linear_attenuation: target.linear_attenuation = values[0]; break;
    case gl::quadratic_attenuation: target.quadratic_attenuation = values[0]; break;
    default: set_error(gl::invalid_enum); break;
    }
}

void Context::light_model(gl::Enum pname, const float* values) noexcept {
    switch (pname) {
    case gl::light_model_ambient: light_model_ambient_ = from4(values); break;
    case gl::light_model_two_side: light_model_two_side_ = values[0] != 0; break;
    case gl::light_model_local_viewer: break;
    default: set_error(gl::invalid_enum); break;
    }
}

void Context::material(gl::Enum face, gl::Enum pname, const float* values) noexcept {
    if (compiling_) {
        ListCommand c{};
        c.op = ListCommand::Op::material;
        c.e0 = face;
        c.e1 = pname;
        for (uint32_t i = 0; i < 4; ++i) { c.v[i] = values[i]; }
        if (record(c) && !compile_execute_) { return; }
    }
    if (face != gl::front && face != gl::back && face != gl::front_and_back) {
        set_error(gl::invalid_enum);
        return;
    }
    Material* targets[2] = {nullptr, nullptr};
    if (face != gl::back) { targets[0] = &material_front_; }
    if (face != gl::front) { targets[1] = &material_back_; }
    for (Material* material : targets) {
        if (material == nullptr) { continue; }
        switch (pname) {
        case gl::ambient: material->ambient = from4(values); break;
        case gl::diffuse: material->diffuse = from4(values); break;
        case gl::ambient_and_diffuse: material->ambient = material->diffuse = from4(values); break;
        case gl::specular: material->specular = from4(values); break;
        case gl::emission: material->emission = from4(values); break;
        case gl::shininess: material->shininess = values[0]; break;
        default: set_error(gl::invalid_enum); return;
        }
    }
}

void Context::color_material(gl::Enum face, gl::Enum mode) noexcept {
    (void)face;
    color_material_mode_ = mode;
}

// ---- Fog ----

void Context::fog(gl::Enum pname, const float* values) noexcept {
    switch (pname) {
    case gl::fog_mode: {
        const gl::Enum mode = static_cast<gl::Enum>(values[0]);
        if (mode != gl::linear && mode != gl::exp && mode != gl::exp2) {
            set_error(gl::invalid_enum);
            return;
        }
        fog_mode_ = mode;
        break;
    }
    case gl::fog_density: fog_density_ = values[0]; break;
    case gl::fog_start: fog_start_ = values[0]; break;
    case gl::fog_end: fog_end_ = values[0]; break;
    case gl::fog_color: fog_color_ = from4(values); break;
    case gl::fog_index: break;
    default: set_error(gl::invalid_enum); break;
    }
}

// ---- Framebuffer sync and read ----

void Context::finish() noexcept {
    if (backend_ != nullptr) { backend_->finish(); }
}

void Context::flush() noexcept {
    if (backend_ != nullptr) { backend_->flush(); }
}

void Context::swap_buffers() noexcept {
    if (backend_ != nullptr) { backend_->flush(); }
}

void Context::read_pixels(int32_t x, int32_t y, int32_t width, int32_t height, gl::Enum format,
                          gl::Enum type, void* pixels) noexcept {
    if (width < 0 || height < 0) { set_error(gl::invalid_value); return; }
    if (backend_ == nullptr || pixels == nullptr || width == 0 || height == 0) { return; }
    if (type != gl::type_unsigned_byte && type != gl::type_float) {
        set_error(gl::invalid_enum);
        return;
    }

    const uint32_t pixel_count = static_cast<uint32_t>(width) * static_cast<uint32_t>(height);
    float* scratch = static_cast<float*>(
        allocator_.allocate(allocator_.context, sizeof(float) * 4 * pixel_count));
    if (scratch == nullptr) { set_error(gl::out_of_memory); return; }

    ReadSource source = ReadSource::color;
    if (format == gl::depth_component) { source = ReadSource::depth; }
    else if (format == gl::stencil_index) { source = ReadSource::stencil; }
    backend_->read(source, read_buffer_, x, y, static_cast<uint32_t>(width),
                   static_cast<uint32_t>(height), scratch);

    const uint32_t channels = source != ReadSource::color ? 1u
                              : format == gl::rgb || format == gl::bgr_ext ? 3u
                              : format == gl::rgba || format == gl::bgra_ext ? 4u
                              : format == gl::luminance ? 1u
                              : format == gl::alpha ? 1u : 4u;
    const bool bgr = format == gl::bgr_ext || format == gl::bgra_ext;

    for (uint32_t pixel = 0; pixel < pixel_count; ++pixel) {
        float rgba[4] = {scratch[pixel * 4 + 0], scratch[pixel * 4 + 1], scratch[pixel * 4 + 2],
                         scratch[pixel * 4 + 3]};
        if (source != ReadSource::color) {
            if (type == gl::type_float) {
                static_cast<float*>(pixels)[pixel] = scratch[pixel * 4];
            } else {
                static_cast<uint8_t*>(pixels)[pixel] =
                    static_cast<uint8_t>(clamp01(scratch[pixel * 4]) * 255.0f + 0.5f);
            }
            continue;
        }
        float ordered[4] = {rgba[0], rgba[1], rgba[2], rgba[3]};
        if (bgr) { ordered[0] = rgba[2]; ordered[2] = rgba[0]; }
        if (format == gl::alpha) { ordered[0] = rgba[3]; }
        for (uint32_t channel = 0; channel < channels; ++channel) {
            if (type == gl::type_float) {
                static_cast<float*>(pixels)[pixel * channels + channel] = ordered[channel];
            } else {
                static_cast<uint8_t*>(pixels)[pixel * channels + channel] =
                    static_cast<uint8_t>(clamp01(ordered[channel]) * 255.0f + 0.5f);
            }
        }
    }

    allocator_.release(allocator_.context, scratch);
}

// ---- Raster position and pixel rectangles ----

namespace {

// GL 1.1 3.6.3 unpacking of one image: the first group of the first row, the
// row stride and, for GL_BITMAP data, the bit position of the first pixel.
struct Unpack final {
    const uint8_t* first = nullptr;
    size_t stride = 0;
    uint32_t components = 0;
    uint32_t element = 0;          // bytes per component; 0 for GL_BITMAP
    uint32_t bit_offset = 0;
};

// 0: invalid type for pixel rectangles.
uint32_t element_bytes(gl::Enum type) noexcept {
    switch (type) {
    case gl::type_unsigned_byte: case gl::type_byte: return 1;
    case gl::type_unsigned_short: case gl::type_short: return 2;
    case gl::type_unsigned_int: case gl::type_int: case gl::type_float: return 4;
    default: return 0;
    }
}

// 0: invalid format.
uint32_t format_components(gl::Enum format) noexcept {
    switch (format) {
    case gl::color_index: case gl::stencil_index: case gl::depth_component: case gl::red:
    case gl::green: case gl::blue: case gl::alpha: case gl::luminance: return 1;
    case gl::luminance_alpha: return 2;
    case gl::rgb: case gl::bgr_ext: return 3;
    case gl::rgba: case gl::bgra_ext: return 4;
    default: return 0;
    }
}

Unpack unpack_layout(const void* pixels, uint32_t width, uint32_t components, gl::Enum type,
                     int32_t alignment, int32_t row_length, int32_t skip_rows, int32_t skip_pixels) noexcept {
    Unpack u{};
    const size_t a = static_cast<size_t>(alignment < 1 ? 1 : alignment);
    const size_t l = row_length > 0 ? static_cast<size_t>(row_length) : width;
    u.components = components;
    if (type == gl::type_bitmap) {
        u.stride = a * ((l + 8u * a - 1u) / (8u * a));
        u.first = static_cast<const uint8_t*>(pixels) + static_cast<size_t>(skip_rows) * u.stride +
                  static_cast<size_t>(skip_pixels) / 8u;
        u.bit_offset = static_cast<uint32_t>(skip_pixels) % 8u;
        return u;
    }
    const size_t s = element_bytes(type);
    const size_t elements = s >= a ? components * l : (a / s) * ((s * components * l + a - 1u) / a);
    u.element = static_cast<uint32_t>(s);
    u.stride = elements * s;
    u.first = static_cast<const uint8_t*>(pixels) + static_cast<size_t>(skip_rows) * u.stride +
              static_cast<size_t>(skip_pixels) * components * s;
    return u;
}

// Component index of a group, in client memory order with optional byte swap.
double raw_component(const uint8_t* p, gl::Enum type, bool swap) noexcept {
    uint8_t bytes[4] = {p[0], 0, 0, 0};
    const uint32_t size = element_bytes(type);
    for (uint32_t i = 0; i < size; ++i) { bytes[i] = p[swap ? size - 1u - i : i]; }
    switch (type) {
    case gl::type_unsigned_byte: return bytes[0];
    case gl::type_byte: return static_cast<int8_t>(bytes[0]);
    case gl::type_unsigned_short: { uint16_t v; std::memcpy(&v, bytes, 2); return v; }
    case gl::type_short: { int16_t v; std::memcpy(&v, bytes, 2); return v; }
    case gl::type_unsigned_int: { uint32_t v; std::memcpy(&v, bytes, 4); return v; }
    case gl::type_int: { int32_t v; std::memcpy(&v, bytes, 4); return v; }
    case gl::type_float: { float v; std::memcpy(&v, bytes, 4); return v; }
    default: return 0;
    }
}

// GL 1.1 table 2.6 conversion of a colour or depth component to [0, 1] / [-1, 1].
float normalized_component(double raw, gl::Enum type) noexcept {
    switch (type) {
    case gl::type_unsigned_byte: return static_cast<float>(raw / 255.0);
    case gl::type_byte: return static_cast<float>((2.0 * raw + 1.0) / 255.0);
    case gl::type_unsigned_short: return static_cast<float>(raw / 65535.0);
    case gl::type_short: return static_cast<float>((2.0 * raw + 1.0) / 65535.0);
    case gl::type_unsigned_int: return static_cast<float>(raw / 4294967295.0);
    case gl::type_int: return static_cast<float>((2.0 * raw + 1.0) / 4294967295.0);
    default: return static_cast<float>(raw);
    }
}

// The 8-bit framebuffer value of a [0, 1] colour (gl_raster's packing rule).
float quantize8(float value) noexcept {
    return static_cast<float>(static_cast<uint8_t>(clamp01(value) * 255.0f + 0.5f)) / 255.0f;
}

bool bitmap_bit(const uint8_t* row, uint32_t bit, bool lsb_first) noexcept {
    const uint8_t byte = row[bit / 8u];
    return ((lsb_first ? byte >> (bit % 8u) : byte >> (7u - bit % 8u)) & 1u) != 0;
}

uint32_t pixel_map_index(gl::Enum map) noexcept {
    return map >= gl::pixel_map_i_to_i && map <= gl::pixel_map_a_to_a ? map - gl::pixel_map_i_to_i
                                                                       : context_pixel_maps;
}

} // namespace

void Context::raster_pos(double x, double y, double z, double w) noexcept {
    if (compiling_) {
        ListCommand c{}; c.op = ListCommand::Op::raster_pos; c.v[0] = x; c.v[1] = y; c.v[2] = z; c.v[3] = w;
        if (record(c) && !compile_execute_) { return; }
    }
    if (in_begin_) { set_error(gl::invalid_operation); return; }
    // Transformed and lit as a vertex; invalid when outside the view volume.
    const Vec4 object{static_cast<float>(x), static_cast<float>(y), static_cast<float>(z), static_cast<float>(w)};
    const RasterVertex vertex = transform(object, current_color_, current_normal_, current_texcoord_);
    const DrawState state = build_draw_state();
    float window_x = 0, window_y = 0;
    if (!clip_point_inside(vertex.clip, view_volume_planes()) || !window_xy(vertex, state, window_x, window_y)) {
        raster_valid_ = false;
        return;
    }
    const float ndc_z = vertex.clip.z * (1.0f / vertex.clip.w);
    raster_window_ = {window_x, window_y,
                      static_cast<float>(depth_near_ + (ndc_z + 1.0) * 0.5 * (depth_far_ - depth_near_)),
                      vertex.clip.w};
    raster_color_ = vertex.color[0];
    raster_texcoord_ = vertex.texcoord;
    raster_fog_ = vertex.fog;
    const Vec4 eye = matrix_transform(modelview_[modelview_depth_], object);
    raster_distance_ = std::fabs(eye.z);
    raster_valid_ = true;
}

void Context::bitmap(int32_t width, int32_t height, float xorig, float yorig, float xmove, float ymove,
                     const uint8_t* bitmap) noexcept {
    if (width < 0 || height < 0) { set_error(gl::invalid_value); return; }
    const bool image = width > 0 && height > 0 && bitmap != nullptr;
    const Unpack u = image ? unpack_layout(bitmap, static_cast<uint32_t>(width), 1, gl::type_bitmap,
                                           unpack_alignment_, unpack_row_length_, unpack_skip_rows_,
                                           unpack_skip_pixels_)
                           : Unpack{};
    if (compiling_) {
        // Record a normalized copy: byte-aligned MSB-first rows.
        ListCommand c{};
        c.op = ListCommand::Op::bitmap;
        c.v[0] = width; c.v[1] = height; c.v[2] = xorig; c.v[3] = yorig; c.v[4] = xmove; c.v[5] = ymove;
        const size_t row_bytes = (static_cast<size_t>(width) + 7u) / 8u;
        const size_t bytes = image ? row_bytes * static_cast<size_t>(height) : 0;
        if (bytes > context_list_bytes - list_bytes_used_ || list_bytes_ == nullptr) {
            set_error(gl::out_of_memory);
            return;
        }
        if (bytes != 0) {
            uint8_t* out = list_bytes_ + list_bytes_used_;
            std::memset(out, 0, bytes);
            for (int32_t row = 0; row < height; ++row) {
                const uint8_t* source = u.first + static_cast<size_t>(row) * u.stride;
                for (int32_t column = 0; column < width; ++column) {
                    if (bitmap_bit(source, u.bit_offset + static_cast<uint32_t>(column), unpack_lsb_first_)) {
                        out[static_cast<size_t>(row) * row_bytes + static_cast<size_t>(column) / 8u] |=
                            static_cast<uint8_t>(0x80u >> (column % 8));
                    }
                }
            }
            c.e0 = list_bytes_used_;
            c.e1 = static_cast<uint32_t>(bytes);
        }
        if (!record(c)) { return; }
        list_bytes_used_ += static_cast<uint32_t>(bytes);
        if (!compile_execute_) { return; }
    }
    bitmap_rows(width, height, xorig, yorig, xmove, ymove, image ? u.first : nullptr, u.stride, u.bit_offset,
                unpack_lsb_first_);
}

void Context::bitmap_rows(int32_t width, int32_t height, float xorig, float yorig, float xmove, float ymove,
                          const uint8_t* rows, size_t stride, uint32_t bit_offset, bool lsb_first) noexcept {
    if (in_begin_) { set_error(gl::invalid_operation); return; }
    if (!raster_valid_) { return; }
    if (backend_ != nullptr && rows != nullptr && width > 0 && height > 0) {
        // GL 1.1 3.6.5: lower left corner floor(raster - origin), zoom 1, the
        // raster colour, depth and texture coordinates.
        const uint32_t band = context_pixel_band / static_cast<uint32_t>(width) == 0
                                  ? 1u : context_pixel_band / static_cast<uint32_t>(width);
        const uint32_t band_rows = band < static_cast<uint32_t>(height) ? band : static_cast<uint32_t>(height);
        uint8_t* coverage = static_cast<uint8_t*>(
            allocator_.allocate(allocator_.context, static_cast<size_t>(band_rows) * static_cast<uint32_t>(width)));
        if (coverage == nullptr) { set_error(gl::out_of_memory); return; }
        PixelFragments fragments{};
        fragments.x = std::floor(raster_window_.x - xorig);
        fragments.y = std::floor(raster_window_.y - yorig);
        fragments.width = static_cast<uint32_t>(width);
        fragments.color = raster_color_;
        fragments.coverage = coverage;
        const DrawState state = build_draw_state();
        for (uint32_t first = 0; first < static_cast<uint32_t>(height); first += band_rows) {
            const uint32_t count = static_cast<uint32_t>(height) - first < band_rows ?
                                   static_cast<uint32_t>(height) - first : band_rows;
            for (uint32_t row = 0; row < count; ++row) {
                const uint8_t* source = rows + static_cast<size_t>(first + row) * stride;
                for (int32_t column = 0; column < width; ++column) {
                    coverage[static_cast<size_t>(row) * static_cast<uint32_t>(width) + static_cast<uint32_t>(column)] =
                        bitmap_bit(source, bit_offset + static_cast<uint32_t>(column), lsb_first) ? 1 : 0;
                }
            }
            draw_image_band(state, fragments, first, count);
        }
        allocator_.release(allocator_.context, coverage);
    }
    raster_window_.x += xmove;
    raster_window_.y += ymove;
}

void Context::draw_image_band(const DrawState& state, PixelFragments& fragments, uint32_t first_row,
                              uint32_t rows) noexcept {
    fragments.first_row = first_row;
    fragments.height = rows;
    fragments.z = raster_window_.z;
    fragments.texcoord = raster_texcoord_;
    fragments.fog = raster_fog_;
    if (fragments.coverage == nullptr) {
        fragments.color = raster_color_;
    }
    backend_->draw_pixels(state, fragments);
}

bool Context::linear_transfer() const noexcept { return !map_color_ && !map_stencil_; }

void Context::transfer_color(float rgba[4]) const noexcept {
    for (uint32_t channel = 0; channel < 4; ++channel) {
        float value = rgba[channel] * color_scale_[channel] + color_bias_[channel];
        if (map_color_) {
            const PixelMap& map = pixel_maps_[pixel_map_index(gl::pixel_map_r_to_r) + channel];
            const uint32_t index = static_cast<uint32_t>(clamp01(value) * static_cast<float>(map.size - 1u) + 0.5f);
            value = map.values[index < map.size ? index : map.size - 1u];
        }
        rgba[channel] = quantize8(value);
    }
}

float Context::transfer_depth(float depth) const noexcept {
    return clamp01(depth * depth_scale_ + depth_bias_);
}

uint8_t Context::transfer_stencil(int64_t index) const noexcept {
    int64_t value = index_shift_ >= 0 ? index * (int64_t{1} << (index_shift_ > 31 ? 31 : index_shift_))
                                      : index >> (-index_shift_ > 31 ? 31 : -index_shift_);
    value += index_offset_;
    if (map_stencil_) {
        const PixelMap& map = pixel_maps_[pixel_map_index(gl::pixel_map_s_to_s)];
        value = static_cast<int64_t>(map.values[static_cast<uint64_t>(value) & (map.size - 1u)]);
    }
    return static_cast<uint8_t>(static_cast<uint64_t>(value) & 0xffu);
}

void Context::draw_pixels(int32_t width, int32_t height, gl::Enum format, gl::Enum type,
                          const void* pixels) noexcept {
    if (in_begin_) { set_error(gl::invalid_operation); return; }
    if (width < 0 || height < 0) { set_error(gl::invalid_value); return; }
    const uint32_t components = format_components(format);
    const bool indices = format == gl::color_index || format == gl::stencil_index;
    if (components == 0 || (type == gl::type_bitmap ? !indices : element_bytes(type) == 0)) {
        set_error(gl::invalid_enum);
        return;
    }
    if (backend_ == nullptr) { return; }
    if ((format == gl::stencil_index && backend_->format().stencil_bits == 0) ||
        (format == gl::depth_component && backend_->format().depth_bits == 0)) {
        set_error(gl::invalid_operation);
        return;
    }
    if (!raster_valid_ || width == 0 || height == 0 || pixels == nullptr) { return; }

    const Unpack u = unpack_layout(pixels, static_cast<uint32_t>(width), components, type, unpack_alignment_,
                                   unpack_row_length_, unpack_skip_rows_, unpack_skip_pixels_);
    const uint32_t w = static_cast<uint32_t>(width);
    const uint32_t band = context_pixel_band / w == 0 ? 1u : context_pixel_band / w;
    const uint32_t band_rows = band < static_cast<uint32_t>(height) ? band : static_cast<uint32_t>(height);
    const size_t band_pixels = static_cast<size_t>(band_rows) * w;
    const bool color = format != gl::stencil_index && format != gl::depth_component;
    const size_t bytes = band_pixels * (color ? 4u * sizeof(float) : format == gl::depth_component ? sizeof(float) : 1u);
    void* band_storage = allocator_.allocate(allocator_.context, bytes);
    if (band_storage == nullptr) { set_error(gl::out_of_memory); return; }

    PixelFragments fragments{};
    fragments.x = raster_window_.x;
    fragments.y = raster_window_.y;
    fragments.width = w;
    fragments.zoom_x = zoom_x_;
    fragments.zoom_y = zoom_y_;
    float* rgba = color ? static_cast<float*>(band_storage) : nullptr;
    float* depth = format == gl::depth_component ? static_cast<float*>(band_storage) : nullptr;
    uint8_t* stencil = format == gl::stencil_index ? static_cast<uint8_t*>(band_storage) : nullptr;
    fragments.rgba = rgba;
    fragments.depth = depth;
    fragments.stencil = stencil;
    const DrawState state = build_draw_state();

    for (uint32_t first = 0; first < static_cast<uint32_t>(height); first += band_rows) {
        const uint32_t count = static_cast<uint32_t>(height) - first < band_rows ?
                               static_cast<uint32_t>(height) - first : band_rows;
        for (uint32_t row = 0; row < count; ++row) {
            const uint8_t* source = u.first + static_cast<size_t>(first + row) * u.stride;
            for (uint32_t column = 0; column < w; ++column) {
                const size_t out = static_cast<size_t>(row) * w + column;
                double raw[4] = {0, 0, 0, 0};
                for (uint32_t c = 0; c < components; ++c) {
                    raw[c] = type == gl::type_bitmap
                                 ? (bitmap_bit(source, u.bit_offset + column, unpack_lsb_first_) ? 1.0 : 0.0)
                                 : raw_component(source + (static_cast<size_t>(column) * components + c) * u.element,
                                                 type, unpack_swap_bytes_);
                }
                if (format == gl::stencil_index) {
                    stencil[out] = transfer_stencil(static_cast<int64_t>(raw[0]));
                    continue;
                }
                if (format == gl::depth_component) {
                    depth[out] = transfer_depth(normalized_component(raw[0], type));
                    continue;
                }
                float* group = rgba + out * 4u;
                if (format == gl::color_index) {
                    // Index arithmetic, then the I_TO_RGBA maps (RGBA mode).
                    const int64_t index = static_cast<int64_t>(raw[0]);
                    int64_t value = index_shift_ >= 0 ? index * (int64_t{1} << (index_shift_ > 31 ? 31 : index_shift_))
                                                      : index >> (-index_shift_ > 31 ? 31 : -index_shift_);
                    value += index_offset_;
                    for (uint32_t c = 0; c < 4; ++c) {
                        const PixelMap& map = pixel_maps_[pixel_map_index(gl::pixel_map_i_to_r) + c];
                        group[c] = quantize8(map.values[static_cast<uint64_t>(value) & (map.size - 1u)]);
                    }
                    continue;
                }
                float value[4] = {0, 0, 0, 1};
                for (uint32_t c = 0; c < components; ++c) {
                    value[c] = normalized_component(raw[c], type);
                }
                switch (format) {
                case gl::red: group[0] = value[0]; group[1] = 0; group[2] = 0; group[3] = 1; break;
                case gl::green: group[0] = 0; group[1] = value[0]; group[2] = 0; group[3] = 1; break;
                case gl::blue: group[0] = 0; group[1] = 0; group[2] = value[0]; group[3] = 1; break;
                case gl::alpha: group[0] = 0; group[1] = 0; group[2] = 0; group[3] = value[0]; break;
                case gl::luminance: group[0] = group[1] = group[2] = value[0]; group[3] = 1; break;
                case gl::luminance_alpha: group[0] = group[1] = group[2] = value[0]; group[3] = value[1]; break;
                case gl::rgb: group[0] = value[0]; group[1] = value[1]; group[2] = value[2]; group[3] = 1; break;
                case gl::bgr_ext: group[0] = value[2]; group[1] = value[1]; group[2] = value[0]; group[3] = 1; break;
                case gl::bgra_ext:
                    group[0] = value[2]; group[1] = value[1]; group[2] = value[0]; group[3] = value[3]; break;
                default: group[0] = value[0]; group[1] = value[1]; group[2] = value[2]; group[3] = value[3]; break;
                }
                transfer_color(group);
            }
        }
        draw_image_band(state, fragments, first, count);
    }
    allocator_.release(allocator_.context, band_storage);
}

void Context::copy_pixels(int32_t x, int32_t y, int32_t width, int32_t height, gl::Enum type) noexcept {
    if (in_begin_) { set_error(gl::invalid_operation); return; }
    if (width < 0 || height < 0) { set_error(gl::invalid_value); return; }
    if (type != gl::color && type != gl::depth && type != gl::stencil) { set_error(gl::invalid_enum); return; }
    if (backend_ == nullptr) { return; }
    if ((type == gl::stencil && backend_->format().stencil_bits == 0) ||
        (type == gl::depth && backend_->format().depth_bits == 0)) {
        set_error(gl::invalid_operation);
        return;
    }
    if (!raster_valid_ || width == 0 || height == 0) { return; }
    const uint64_t pixel_count = static_cast<uint64_t>(width) * static_cast<uint64_t>(height);
    if (pixel_count > (uint64_t{1} << 26)) { set_error(gl::out_of_memory); return; }

    const ReadSource source = type == gl::color ? ReadSource::color :
                              type == gl::depth ? ReadSource::depth : ReadSource::stencil;
    const DrawState state = build_draw_state();
    const uint32_t w = static_cast<uint32_t>(width);
    const uint32_t h = static_cast<uint32_t>(height);
    PixelFragments fragments{};
    fragments.x = raster_window_.x;
    fragments.y = raster_window_.y;
    fragments.width = w;
    fragments.height = h;
    fragments.zoom_x = zoom_x_;
    fragments.zoom_y = zoom_y_;
    fragments.z = raster_window_.z;
    fragments.color = raster_color_;
    fragments.texcoord = raster_texcoord_;
    fragments.fog = raster_fog_;

    if (linear_transfer()) {
        PixelCopy copy{};
        copy.source = source;
        copy.buffer = read_buffer_;
        copy.x = x;
        copy.y = y;
        copy.width = w;
        copy.height = h;
        copy.destination = fragments;
        for (uint32_t c = 0; c < 4; ++c) {
            copy.scale[c] = source == ReadSource::depth ? (c == 0 ? depth_scale_ : 1.0f) : color_scale_[c];
            copy.bias[c] = source == ReadSource::depth ? (c == 0 ? depth_bias_ : 0.0f) : color_bias_[c];
        }
        copy.index_shift = index_shift_;
        copy.index_offset = index_offset_;
        if (backend_->copy_pixels(state, copy)) { return; }
    }

    // Read the whole source first (GL: as if read completely before any write),
    // compactly: framebuffer colours are 8-bit, so bytes keep them exactly.
    const size_t element = source == ReadSource::depth ? sizeof(float) : source == ReadSource::color ? 4u : 1u;
    uint8_t* image = static_cast<uint8_t*>(allocator_.allocate(allocator_.context, static_cast<size_t>(pixel_count) * element));
    const uint32_t band = context_pixel_band / w == 0 ? 1u : context_pixel_band / w;
    const uint32_t band_rows = band < h ? band : h;
    float* scratch = static_cast<float*>(
        allocator_.allocate(allocator_.context, static_cast<size_t>(band_rows) * w * 4u * sizeof(float)));
    if (image == nullptr || scratch == nullptr) {
        if (image != nullptr) { allocator_.release(allocator_.context, image); }
        if (scratch != nullptr) { allocator_.release(allocator_.context, scratch); }
        set_error(gl::out_of_memory);
        return;
    }
    for (uint32_t first = 0; first < h; first += band_rows) {
        const uint32_t count = h - first < band_rows ? h - first : band_rows;
        const size_t values = static_cast<size_t>(count) * w * (source == ReadSource::color ? 4u : 1u);
        for (size_t i = 0; i < values; ++i) { scratch[i] = 0; }   // undefined outside: zero
        backend_->read(source, read_buffer_, x, y + static_cast<int32_t>(first), w, count, scratch);
        for (size_t i = 0; i < values; ++i) {
            const size_t at = static_cast<size_t>(first) * w * (source == ReadSource::color ? 4u : 1u) + i;
            if (source == ReadSource::depth) {
                std::memcpy(image + at * sizeof(float), &scratch[i], sizeof(float));
            } else if (source == ReadSource::stencil) {
                image[at] = static_cast<uint8_t>(scratch[i] < 0 ? 0 : scratch[i] > 255 ? 255 : scratch[i]);
            } else {
                image[at] = static_cast<uint8_t>(clamp01(scratch[i]) * 255.0f + 0.5f);
            }
        }
    }
    for (uint32_t first = 0; first < h; first += band_rows) {
        const uint32_t count = h - first < band_rows ? h - first : band_rows;
        const size_t pixels_in_band = static_cast<size_t>(count) * w;
        uint8_t* stencil = reinterpret_cast<uint8_t*>(scratch);
        for (size_t i = 0; i < pixels_in_band; ++i) {
            const size_t at = static_cast<size_t>(first) * w + i;
            if (source == ReadSource::color) {
                float* group = scratch + i * 4u;
                for (uint32_t c = 0; c < 4; ++c) { group[c] = static_cast<float>(image[at * 4u + c]) / 255.0f; }
                transfer_color(group);
            } else if (source == ReadSource::depth) {
                float value = 0;
                std::memcpy(&value, image + at * sizeof(float), sizeof(float));
                scratch[i] = transfer_depth(value);
            } else {
                stencil[i] = transfer_stencil(image[at]);
            }
        }
        fragments.rgba = source == ReadSource::color ? scratch : nullptr;
        fragments.depth = source == ReadSource::depth ? scratch : nullptr;
        fragments.stencil = source == ReadSource::stencil ? stencil : nullptr;
        draw_image_band(state, fragments, first, count);
    }
    allocator_.release(allocator_.context, scratch);
    allocator_.release(allocator_.context, image);
}

void Context::pixel_zoom(float x, float y) noexcept {
    if (in_begin_) { set_error(gl::invalid_operation); return; }
    zoom_x_ = x;
    zoom_y_ = y;
}

void Context::pixel_transfer(gl::Enum pname, float value) noexcept {
    if (in_begin_) { set_error(gl::invalid_operation); return; }
    switch (pname) {
    case gl::map_color: map_color_ = value != 0; break;
    case gl::map_stencil: map_stencil_ = value != 0; break;
    case gl::index_shift: index_shift_ = static_cast<int32_t>(value); break;
    case gl::index_offset: index_offset_ = static_cast<int32_t>(value); break;
    case gl::red_scale: color_scale_[0] = value; break;
    case gl::green_scale: color_scale_[1] = value; break;
    case gl::blue_scale: color_scale_[2] = value; break;
    case gl::alpha_scale: color_scale_[3] = value; break;
    case gl::red_bias: color_bias_[0] = value; break;
    case gl::green_bias: color_bias_[1] = value; break;
    case gl::blue_bias: color_bias_[2] = value; break;
    case gl::alpha_bias: color_bias_[3] = value; break;
    case gl::depth_scale: depth_scale_ = value; break;
    case gl::depth_bias: depth_bias_ = value; break;
    default: set_error(gl::invalid_enum); break;
    }
}

void Context::pixel_map(gl::Enum map, int32_t size, const float* values) noexcept {
    if (in_begin_) { set_error(gl::invalid_operation); return; }
    const uint32_t index = pixel_map_index(map);
    if (index == context_pixel_maps) { set_error(gl::invalid_enum); return; }
    // Index maps (I_TO_*, S_TO_S) are addressed by masking: power-of-two sizes.
    const bool indexed = map <= gl::pixel_map_i_to_a;
    if (size < 1 || size > static_cast<int32_t>(context_max_pixel_map) ||
        (indexed && (size & (size - 1)) != 0) || values == nullptr) {
        set_error(gl::invalid_value);
        return;
    }
    PixelMap& target = pixel_maps_[index];
    target.size = static_cast<uint32_t>(size);
    for (int32_t i = 0; i < size; ++i) {
        target.values[i] = values[i];
    }
}

uint32_t Context::pixel_map_size(gl::Enum map) noexcept {
    const uint32_t index = pixel_map_index(map);
    if (index == context_pixel_maps) { set_error(gl::invalid_enum); return 0; }
    return pixel_maps_[index].size;
}

void Context::get_pixel_map(gl::Enum map, float* values) noexcept {
    const uint32_t index = pixel_map_index(map);
    if (index == context_pixel_maps) { set_error(gl::invalid_enum); return; }
    if (values == nullptr) { return; }
    for (uint32_t i = 0; i < pixel_maps_[index].size; ++i) { values[i] = pixel_maps_[index].values[i]; }
}

void Context::polygon_stipple(const uint8_t* mask) noexcept {
    if (in_begin_) { set_error(gl::invalid_operation); return; }
    if (mask == nullptr) { return; }
    const Unpack u = unpack_layout(mask, 32, 1, gl::type_bitmap, unpack_alignment_, unpack_row_length_,
                                   unpack_skip_rows_, unpack_skip_pixels_);
    for (uint32_t row = 0; row < 32; ++row) {
        const uint8_t* source = u.first + static_cast<size_t>(row) * u.stride;
        uint32_t bits = 0;
        for (uint32_t column = 0; column < 32; ++column) {
            if (bitmap_bit(source, u.bit_offset + column, unpack_lsb_first_)) { bits |= 0x80000000u >> column; }
        }
        polygon_stipple_rows_[row] = bits;
    }
}

void Context::get_polygon_stipple(uint8_t* mask) noexcept {
    if (in_begin_) { set_error(gl::invalid_operation); return; }
    if (mask == nullptr) { return; }
    // Packed MSB first, four bytes per row padded to GL_PACK_ALIGNMENT.
    const size_t stride = pack_alignment_ == 8 ? 8u : 4u;
    for (uint32_t row = 0; row < 32; ++row) {
        for (uint32_t byte = 0; byte < stride; ++byte) {
            mask[row * stride + byte] = byte < 4 ? static_cast<uint8_t>(polygon_stipple_rows_[row] >> (24 - 8 * byte)) : 0;
        }
    }
}

// ---- Queries ----

void Context::get_integerv(gl::Enum name, int32_t* values) noexcept {
    switch (name) {
    case gl::max_texture_size: values[0] = static_cast<int32_t>(texture_max_size); break;
    case gl::max_lights: values[0] = context_max_lights; break;
    case gl::max_modelview_stack_depth:
    case gl::max_projection_stack_depth:
    case gl::max_texture_stack_depth: values[0] = context_matrix_stack_depth; break;
    case gl::max_clip_planes: values[0] = 0; break;
    case gl::viewport:
        values[0] = viewport_x_; values[1] = viewport_y_;
        values[2] = viewport_w_; values[3] = viewport_h_; break;
    case gl::depth_bits: values[0] = backend_ != nullptr ? static_cast<int32_t>(backend_->format().depth_bits) : 0; break;
    case gl::stencil_bits: values[0] = backend_ != nullptr ? static_cast<int32_t>(backend_->format().stencil_bits) : 0; break;
    case gl::red_bits: case gl::green_bits: case gl::blue_bits: case gl::alpha_bits: values[0] = 8; break;
    case gl::texture_binding_2d: values[0] = static_cast<int32_t>(bound_texture_2d_); break;
    case gl::index_shift: values[0] = index_shift_; break;
    case gl::index_offset: values[0] = index_offset_; break;
    case gl::max_pixel_map_table: values[0] = static_cast<int32_t>(context_max_pixel_map); break;
    case gl::unpack_alignment: values[0] = unpack_alignment_; break;
    case gl::pack_alignment: values[0] = pack_alignment_; break;
    case gl::unpack_row_length: values[0] = unpack_row_length_; break;
    case gl::unpack_skip_rows: values[0] = unpack_skip_rows_; break;
    case gl::unpack_skip_pixels: values[0] = unpack_skip_pixels_; break;
    case gl::draw_buffer: values[0] = static_cast<int32_t>(draw_buffer_); break;
    case gl::read_buffer: values[0] = static_cast<int32_t>(read_buffer_); break;
    case gl::line_stipple_pattern: values[0] = line_stipple_pattern_; break;
    case gl::line_stipple_repeat: values[0] = line_stipple_factor_; break;
    default:
        if (name >= gl::pixel_map_i_to_i_size && name <= gl::pixel_map_a_to_a_size) {
            values[0] = static_cast<int32_t>(pixel_maps_[name - gl::pixel_map_i_to_i_size].size);
            break;
        }
        set_error(gl::invalid_enum);
        break;
    }
}

void Context::get_floatv(gl::Enum name, float* values) noexcept {
    switch (name) {
    case gl::modelview_matrix:
        for (uint32_t i = 0; i < 16; ++i) { values[i] = modelview_[modelview_depth_].values[i]; }
        break;
    case gl::projection_matrix:
        for (uint32_t i = 0; i < 16; ++i) { values[i] = projection_[projection_depth_].values[i]; }
        break;
    case gl::current_color:
        values[0] = current_color_.x; values[1] = current_color_.y;
        values[2] = current_color_.z; values[3] = current_color_.w; break;
    case gl::depth_clear_value: values[0] = static_cast<float>(clear_depth_); break;
    case gl::line_width: values[0] = line_width_; break;
    case gl::point_size: values[0] = point_size_; break;
    case gl::current_raster_position:
        values[0] = raster_window_.x; values[1] = raster_window_.y;
        values[2] = raster_window_.z; values[3] = raster_window_.w; break;
    case gl::current_raster_color:
        values[0] = raster_color_.x; values[1] = raster_color_.y;
        values[2] = raster_color_.z; values[3] = raster_color_.w; break;
    case gl::current_raster_texture_coords:
        values[0] = raster_texcoord_.x; values[1] = raster_texcoord_.y;
        values[2] = raster_texcoord_.z; values[3] = raster_texcoord_.w; break;
    case gl::current_raster_distance: values[0] = raster_distance_; break;
    case gl::zoom_x: values[0] = zoom_x_; break;
    case gl::zoom_y: values[0] = zoom_y_; break;
    case gl::red_scale: values[0] = color_scale_[0]; break;
    case gl::green_scale: values[0] = color_scale_[1]; break;
    case gl::blue_scale: values[0] = color_scale_[2]; break;
    case gl::alpha_scale: values[0] = color_scale_[3]; break;
    case gl::red_bias: values[0] = color_bias_[0]; break;
    case gl::green_bias: values[0] = color_bias_[1]; break;
    case gl::blue_bias: values[0] = color_bias_[2]; break;
    case gl::alpha_bias: values[0] = color_bias_[3]; break;
    case gl::depth_scale: values[0] = depth_scale_; break;
    case gl::depth_bias: values[0] = depth_bias_; break;
    default: {
        int32_t integer[4] = {};
        get_integerv(name, integer);
        if (error_ != gl::invalid_enum) {
            for (uint32_t i = 0; i < 4; ++i) { values[i] = static_cast<float>(integer[i]); }
        }
        break;
    }
    }
}

void Context::get_booleanv(gl::Enum name, uint8_t* values) noexcept {
    switch (name) {
    case gl::current_raster_position_valid: values[0] = raster_valid_ ? 1 : 0; break;
    case gl::map_color: values[0] = map_color_ ? 1 : 0; break;
    case gl::map_stencil: values[0] = map_stencil_ ? 1 : 0; break;
    case gl::unpack_swap_bytes: values[0] = unpack_swap_bytes_ ? 1 : 0; break;
    case gl::unpack_lsb_first: values[0] = unpack_lsb_first_ ? 1 : 0; break;
    default: values[0] = is_enabled(name) ? 1 : 0; break;
    }
}

// ---- Immediate mode and transform ----

void Context::color(double r, double g, double b, double a) noexcept {
    if (compiling_) {
        ListCommand c{}; c.op = ListCommand::Op::color; c.v[0] = r; c.v[1] = g; c.v[2] = b; c.v[3] = a;
        if (record(c) && !compile_execute_) { return; }
    }
    current_color_ = {static_cast<float>(r), static_cast<float>(g), static_cast<float>(b),
                      static_cast<float>(a)};
}

void Context::normal(double x, double y, double z) noexcept {
    if (compiling_) {
        ListCommand c{}; c.op = ListCommand::Op::normal; c.v[0] = x; c.v[1] = y; c.v[2] = z;
        if (record(c) && !compile_execute_) { return; }
    }
    current_normal_ = {static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)};
}

void Context::tex_coord(double s, double t, double r, double q) noexcept {
    if (compiling_) {
        ListCommand c{}; c.op = ListCommand::Op::tex_coord; c.v[0] = s; c.v[1] = t; c.v[2] = r; c.v[3] = q;
        if (record(c) && !compile_execute_) { return; }
    }
    current_texcoord_ = {static_cast<float>(s), static_cast<float>(t), static_cast<float>(r),
                         static_cast<float>(q)};
}

void Context::edge_flag(bool flag) noexcept { current_edge_flag_ = flag; }

void Context::begin(gl::Enum mode) noexcept {
    if (compiling_) {
        ListCommand c{}; c.op = ListCommand::Op::begin; c.e0 = mode;
        if (record(c) && !compile_execute_) { return; }
    }
    if (mode > gl::polygon) { set_error(gl::invalid_enum); return; }
    if (in_begin_) { set_error(gl::invalid_operation); return; }
    in_begin_ = true;
    primitive_mode_ = mode;
    vertex_count_ = 0;
}

void Context::end() noexcept {
    if (compiling_) {
        ListCommand c{}; c.op = ListCommand::Op::end;
        if (record(c) && !compile_execute_) { return; }
    }
    if (!in_begin_) { set_error(gl::invalid_operation); return; }
    assemble_and_draw();
    in_begin_ = false;
    vertex_count_ = 0;
}

Vec4 Context::apply_lighting(const Vec4& eye, const Vec3& eye_normal, const Vec4& base_color,
                             bool front) noexcept {
    const Material& material = front ? material_front_ : material_back_;
    Vec4 ambient_material = material.ambient;
    Vec4 diffuse_material = material.diffuse;
    if (color_material_) {
        if (color_material_mode_ == gl::ambient || color_material_mode_ == gl::ambient_and_diffuse) {
            ambient_material = base_color;
        }
        if (color_material_mode_ == gl::diffuse || color_material_mode_ == gl::ambient_and_diffuse) {
            diffuse_material = base_color;
        }
    }

    Vec3 n = eye_normal;
    const float len = std::sqrt(n.x * n.x + n.y * n.y + n.z * n.z);
    if (len > 0) { n.x /= len; n.y /= len; n.z /= len; }
    if (!front) { n.x = -n.x; n.y = -n.y; n.z = -n.z; }

    Vec4 result = material.emission;
    result.x += light_model_ambient_.x * ambient_material.x;
    result.y += light_model_ambient_.y * ambient_material.y;
    result.z += light_model_ambient_.z * ambient_material.z;

    const float ew = eye.w != 0 ? eye.w : 1.0f;
    const Vec3 vertex_eye{eye.x / ew, eye.y / ew, eye.z / ew};

    for (const Light& light : lights_) {
        if (!light.enabled) { continue; }
        Vec3 direction;
        float attenuation = 1;
        if (light.position.w == 0) {
            direction = {light.position.x, light.position.y, light.position.z};
        } else {
            direction = {light.position.x - vertex_eye.x, light.position.y - vertex_eye.y,
                         light.position.z - vertex_eye.z};
            const float d = std::sqrt(direction.x * direction.x + direction.y * direction.y +
                                      direction.z * direction.z);
            attenuation = 1.0f / (light.constant_attenuation + light.linear_attenuation * d +
                                  light.quadratic_attenuation * d * d);
        }
        const float dl = std::sqrt(direction.x * direction.x + direction.y * direction.y +
                                   direction.z * direction.z);
        if (dl > 0) { direction.x /= dl; direction.y /= dl; direction.z /= dl; }

        float spot = 1;
        if (light.spot_cutoff != 180) {
            const float cosine = -(direction.x * light.spot_direction.x +
                                   direction.y * light.spot_direction.y +
                                   direction.z * light.spot_direction.z);
            const float cutoff = std::cos(light.spot_cutoff * 0.017453292519943295f);
            if (cosine < cutoff) { spot = 0; }
            else { spot = std::pow(cosine > 0 ? cosine : 0, light.spot_exponent); }
        }
        const float factor = attenuation * spot;

        result.x += factor * light.ambient.x * ambient_material.x;
        result.y += factor * light.ambient.y * ambient_material.y;
        result.z += factor * light.ambient.z * ambient_material.z;

        float ndotl = n.x * direction.x + n.y * direction.y + n.z * direction.z;
        if (ndotl < 0) { ndotl = 0; }
        result.x += factor * ndotl * light.diffuse.x * diffuse_material.x;
        result.y += factor * ndotl * light.diffuse.y * diffuse_material.y;
        result.z += factor * ndotl * light.diffuse.z * diffuse_material.z;

        if (ndotl > 0 && material.shininess >= 0) {
            Vec3 half{direction.x, direction.y, direction.z + 1};
            const float hl = std::sqrt(half.x * half.x + half.y * half.y + half.z * half.z);
            if (hl > 0) { half.x /= hl; half.y /= hl; half.z /= hl; }
            float ndoth = n.x * half.x + n.y * half.y + n.z * half.z;
            if (ndoth < 0) { ndoth = 0; }
            const float spec = std::pow(ndoth, material.shininess);
            result.x += factor * spec * light.specular.x * material.specular.x;
            result.y += factor * spec * light.specular.y * material.specular.y;
            result.z += factor * spec * light.specular.z * material.specular.z;
        }
    }

    result.w = diffuse_material.w;
    result.x = clamp01(result.x);
    result.y = clamp01(result.y);
    result.z = clamp01(result.z);
    result.w = clamp01(result.w);
    return result;
}

RasterVertex Context::transform(const Vec4& object_position, const Vec4& color_in,
                                const Vec3& normal_in, const Vec4& texcoord_in) noexcept {
    const Matrix4& modelview = modelview_[modelview_depth_];
    const Vec4 eye = matrix_transform(modelview, object_position);
    RasterVertex result{};
    result.clip = matrix_transform(projection_[projection_depth_], eye);

    Vec3 eye_normal = normal_in;
    Matrix4 inverse;
    if (matrix_invert(modelview, inverse)) {
        // Inverse-transpose for the normal: multiply by transpose(inverse).
        eye_normal = {inverse.values[0] * normal_in.x + inverse.values[1] * normal_in.y +
                          inverse.values[2] * normal_in.z,
                      inverse.values[4] * normal_in.x + inverse.values[5] * normal_in.y +
                          inverse.values[6] * normal_in.z,
                      inverse.values[8] * normal_in.x + inverse.values[9] * normal_in.y +
                          inverse.values[10] * normal_in.z};
    }
    if (normalize_) {
        const float len = std::sqrt(eye_normal.x * eye_normal.x + eye_normal.y * eye_normal.y +
                                    eye_normal.z * eye_normal.z);
        if (len > 0) { eye_normal.x /= len; eye_normal.y /= len; eye_normal.z /= len; }
    }

    if (lighting_) {
        result.color[0] = apply_lighting(eye, eye_normal, color_in, true);
        result.color[1] = light_model_two_side_ ? apply_lighting(eye, eye_normal, color_in, false)
                                                 : result.color[0];
    } else {
        result.color[0] = result.color[1] = color_in;
    }

    result.texcoord = matrix_transform(texture_matrix_[texture_depth_], texcoord_in);

    if (fog_enabled_) {
        const float z = std::fabs(eye.w != 0 ? eye.z / eye.w : eye.z);
        float factor = 1;
        if (fog_mode_ == gl::linear) {
            factor = fog_end_ != fog_start_ ? (fog_end_ - z) / (fog_end_ - fog_start_) : 1;
        } else if (fog_mode_ == gl::exp) {
            factor = std::exp(-fog_density_ * z);
        } else {
            factor = std::exp(-(fog_density_ * z) * (fog_density_ * z));
        }
        result.fog = clamp01(factor);
    } else {
        result.fog = 1;
    }

    return result;
}

void Context::emit_vertex(const Vec4& object_position) noexcept {
    if (vertex_count_ >= context_vertex_buffer) {
        // Flush a full strip/fan batch is unsafe mid-primitive; drop extra vertices.
        return;
    }
    vertices_[vertex_count_] = transform(object_position, current_color_, current_normal_,
                                         current_texcoord_);
    vertex_edge_[vertex_count_] = current_edge_flag_;
    ++vertex_count_;
}

void Context::vertex(double x, double y, double z, double w) noexcept {
    if (compiling_) {
        ListCommand c{}; c.op = ListCommand::Op::vertex; c.v[0] = x; c.v[1] = y; c.v[2] = z; c.v[3] = w;
        if (record(c) && !compile_execute_) { return; }
    }
    if (!in_begin_) { set_error(gl::invalid_operation); return; }
    emit_vertex({static_cast<float>(x), static_cast<float>(y), static_cast<float>(z),
                 static_cast<float>(w)});
}

DrawState Context::build_draw_state() noexcept {
    DrawState state{};
    state.viewport_x = viewport_x_;
    state.viewport_y = viewport_y_;
    state.viewport_width = viewport_w_;
    state.viewport_height = viewport_h_;
    state.depth_near = depth_near_;
    state.depth_far = depth_far_;
    state.cull_enabled = cull_;
    state.cull_mode = cull_mode_;
    state.front_ccw = front_ccw_;
    state.two_sided_color = light_model_two_side_;
    state.point_size = point_size_;
    state.line_width = line_width_;
    state.line_stipple = line_stipple_enabled_;
    state.line_stipple_factor = static_cast<uint32_t>(line_stipple_factor_);
    state.line_stipple_pattern = line_stipple_pattern_;
    state.polygon_stipple = polygon_stipple_enabled_;
    for (uint32_t row = 0; row < 32; ++row) { state.polygon_stipple_rows[row] = polygon_stipple_rows_[row]; }
    state.offset_enabled = polygon_offset_fill_;
    state.offset_factor = polygon_offset_factor_;
    state.offset_units = polygon_offset_units_;
    state.scissor_enabled = scissor_test_enabled_;
    state.scissor_x = scissor_x_;
    state.scissor_y = scissor_y_;
    state.scissor_width = scissor_w_;
    state.scissor_height = scissor_h_;
    Texture* texture = bound_texture();
    state.texture = (texture_2d_enabled_ && texture != nullptr && texture_complete(*texture))
                        ? texture
                        : nullptr;
    state.texture_env_mode = texture_env_mode_;
    state.texture_env_color = texture_env_color_;
    state.fog_enabled = fog_enabled_;
    state.fog_color = fog_color_;
    state.alpha_test = alpha_test_;
    state.alpha_func = alpha_func_;
    state.alpha_ref = alpha_ref_;
    state.stencil_test = stencil_test_;
    state.stencil = stencil_;
    state.depth_test = depth_test_;
    state.depth_func = depth_func_;
    state.depth_write = depth_write_;
    state.blend = blend_;
    state.blend_src = blend_src_;
    state.blend_dst = blend_dst_;
    state.logic_op = logic_op_;
    state.logic_op_mode = logic_op_mode_;
    for (uint32_t i = 0; i < 4; ++i) { state.color_write[i] = color_mask_[i]; }
    state.dither = dither_;
    state.draw_buffer = draw_buffer_;
    return state;
}

void Context::assemble_and_draw() noexcept {
    if (backend_ == nullptr || vertex_count_ == 0) {
        return;
    }
    DrawState state = build_draw_state();
    const ClipPlanes planes = view_volume_planes();

    // Flat shading colours each primitive from its provoking vertex (GL 1.1
    // table 2.6): the last vertex of a triangle, the 4th of a quad, the
    // (2i+4)th of a quad strip and the first of a polygon. The provoking
    // vertex need not be one of the three triangle vertices (quads), so it is
    // passed by value.
    auto flat = [&](RasterVertex* verts, const RasterVertex& provoking) {
        if (shade_model_ != gl::flat) { return; }
        for (uint32_t i = 0; i < 3; ++i) {
            verts[i].color[0] = provoking.color[0];
            verts[i].color[1] = provoking.color[1];
        }
    };

    auto draw_triangle = [&](RasterVertex a, RasterVertex b, RasterVertex c, RasterVertex provoking) {
        RasterVertex tri[3] = {a, b, c};
        flat(tri, provoking);
        RasterVertex output[16];
        RasterVertex scratch[16];
        const uint32_t count = clip_polygon(tri, 3, planes, output, scratch, 16);
        for (uint32_t i = 2; i < count; ++i) {
            RasterVertex fan[3] = {output[0], output[i - 1], output[i]};
            backend_->draw(state, PrimitiveKind::triangles, fan, 3);
        }
    };

    // GL 1.1 3.4.2: the stipple counter restarts for every independent segment
    // and at Begin; strip and loop segments continue it. A segment clipped away
    // produces no sample, so a pending restart moves to the next drawn segment.
    bool restart = true;
    auto draw_line = [&](RasterVertex a, RasterVertex b, bool reset) {
        if (shade_model_ == gl::flat) { a.color[0] = b.color[0]; }
        restart = restart || reset;
        if (clip_line(a, b, planes)) {
            state.line_stipple_reset = restart;
            restart = false;
            RasterVertex line[2] = {a, b};
            backend_->draw(state, PrimitiveKind::lines, line, 2);
        }
    };

    switch (primitive_mode_) {
    case gl::points:
        for (uint32_t i = 0; i < vertex_count_; ++i) {
            if (clip_point_inside(vertices_[i].clip, planes)) {
                backend_->draw(state, PrimitiveKind::points, &vertices_[i], 1);
            }
        }
        break;
    case gl::lines:
        for (uint32_t i = 0; i + 1 < vertex_count_; i += 2) { draw_line(vertices_[i], vertices_[i + 1], true); }
        break;
    case gl::line_strip:
        for (uint32_t i = 0; i + 1 < vertex_count_; ++i) { draw_line(vertices_[i], vertices_[i + 1], i == 0); }
        break;
    case gl::line_loop:
        for (uint32_t i = 0; i + 1 < vertex_count_; ++i) { draw_line(vertices_[i], vertices_[i + 1], i == 0); }
        if (vertex_count_ > 1) { draw_line(vertices_[vertex_count_ - 1], vertices_[0], vertex_count_ == 2); }
        break;
    case gl::triangles:
        for (uint32_t i = 0; i + 2 < vertex_count_; i += 3) {
            draw_triangle(vertices_[i], vertices_[i + 1], vertices_[i + 2], vertices_[i + 2]);
        }
        break;
    case gl::triangle_strip:
        for (uint32_t i = 0; i + 2 < vertex_count_; ++i) {
            if (i & 1u) { draw_triangle(vertices_[i + 1], vertices_[i], vertices_[i + 2], vertices_[i + 2]); }
            else { draw_triangle(vertices_[i], vertices_[i + 1], vertices_[i + 2], vertices_[i + 2]); }
        }
        break;
    case gl::triangle_fan:
        for (uint32_t i = 1; i + 1 < vertex_count_; ++i) {
            draw_triangle(vertices_[0], vertices_[i], vertices_[i + 1], vertices_[i + 1]);
        }
        break;
    case gl::polygon:
        for (uint32_t i = 1; i + 1 < vertex_count_; ++i) {
            draw_triangle(vertices_[0], vertices_[i], vertices_[i + 1], vertices_[0]);
        }
        break;
    case gl::quads:
        for (uint32_t i = 0; i + 3 < vertex_count_; i += 4) {
            draw_triangle(vertices_[i], vertices_[i + 1], vertices_[i + 2], vertices_[i + 3]);
            draw_triangle(vertices_[i], vertices_[i + 2], vertices_[i + 3], vertices_[i + 3]);
        }
        break;
    case gl::quad_strip:
        for (uint32_t i = 0; i + 3 < vertex_count_; i += 2) {
            draw_triangle(vertices_[i], vertices_[i + 1], vertices_[i + 2], vertices_[i + 3]);
            draw_triangle(vertices_[i + 2], vertices_[i + 1], vertices_[i + 3], vertices_[i + 3]);
        }
        break;
    default:
        break;
    }
}

// ---- Textures ----

Texture* Context::find_texture(uint32_t name) noexcept {
    if (name == 0) { return nullptr; }
    for (uint32_t index = 0; index < context_max_textures; ++index) {
        if (share_->textures[index].name == name) { return &share_->textures[index]; }
    }
    return nullptr;
}

Texture* Context::bound_texture() noexcept { return find_texture(bound_texture_2d_); }

void Context::gen_textures(int32_t count, uint32_t* names) noexcept {
    if (count < 0) { set_error(gl::invalid_value); return; }
    int32_t produced = 0;
    uint32_t next_name = 1;
    for (uint32_t index = 0; index < context_max_textures && produced < count; ++index) {
        Texture& slot = share_->textures[index];
        if (slot.name != 0) { continue; }
        // Find an unused name.
        bool unique = false;
        while (!unique) {
            unique = true;
            for (uint32_t scan = 0; scan < context_max_textures; ++scan) {
                if (share_->textures[scan].name == next_name) { ++next_name; unique = false; break; }
            }
        }
        reset_texture(slot);
        slot.name = next_name;
        slot.target = gl::texture_2d;
        names[produced++] = next_name;
        ++next_name;
    }
    for (; produced < count; ++produced) { names[produced] = 0; }   // capacity exhausted
}

void Context::delete_textures(int32_t count, const uint32_t* names) noexcept {
    if (count < 0) { set_error(gl::invalid_value); return; }
    for (int32_t index = 0; index < count; ++index) {
        Texture* texture = find_texture(names[index]);
        if (texture == nullptr) { continue; }
        for (TextureLevel& level : texture->levels) {
            if (level.texels != nullptr) {
                allocator_.release(allocator_.context, level.texels);
                level.texels = nullptr;
            }
        }
        if (bound_texture_2d_ == texture->name) { bound_texture_2d_ = 0; }
        reset_texture(*texture);
    }
}

void Context::bind_texture(gl::Enum target, uint32_t name) noexcept {
    if (target != gl::texture_2d) { set_error(gl::invalid_enum); return; }
    if (name != 0 && find_texture(name) == nullptr) {
        for (uint32_t index = 0; index < context_max_textures; ++index) {
            if (share_->textures[index].name == 0) {
                reset_texture(share_->textures[index]);
                share_->textures[index].name = name;
                share_->textures[index].target = gl::texture_2d;
                break;
            }
        }
    }
    bound_texture_2d_ = name;
}

bool Context::is_texture(uint32_t name) noexcept { return find_texture(name) != nullptr; }

void Context::tex_image_2d(gl::Enum target, int32_t level, int32_t internal_format, int32_t width,
                           int32_t height, int32_t border, gl::Enum format, gl::Enum type,
                           const void* pixels) noexcept {
    if (target != gl::texture_2d) { set_error(gl::invalid_enum); return; }
    if (level < 0 || level >= static_cast<int32_t>(texture_max_levels) || border != 0 ||
        width < 0 || height < 0 || width > static_cast<int32_t>(texture_max_size) ||
        height > static_cast<int32_t>(texture_max_size)) {
        set_error(gl::invalid_value);
        return;
    }
    if (type != gl::type_unsigned_byte && type != gl::type_float) {
        set_error(gl::invalid_enum);
        return;
    }
    Texture* texture = bound_texture();
    if (texture == nullptr) { set_error(gl::invalid_operation); return; }

    TextureLevel& image = texture->levels[level];
    if (image.texels != nullptr) {
        allocator_.release(allocator_.context, image.texels);
        image.texels = nullptr;
    }
    image.width = static_cast<uint32_t>(width);
    image.height = static_cast<uint32_t>(height);
    image.base_format = base_format_of(internal_format);
    image.internal_format = static_cast<gl::Enum>(internal_format);
    if (width == 0 || height == 0) { return; }

    const size_t texels = static_cast<size_t>(width) * height * 4u;
    image.texels = static_cast<uint8_t*>(allocator_.allocate(allocator_.context, texels));
    if (image.texels == nullptr) { set_error(gl::out_of_memory); image.width = image.height = 0; return; }
    std::memset(image.texels, 0, texels);
    ++texture->generation;
    if (pixels != nullptr) {
        tex_sub_image_2d(target, level, 0, 0, width, height, format, type, pixels);
    }
}

void Context::tex_sub_image_2d(gl::Enum target, int32_t level, int32_t xoffset, int32_t yoffset,
                               int32_t width, int32_t height, gl::Enum format, gl::Enum type,
                               const void* pixels) noexcept {
    if (target != gl::texture_2d) { set_error(gl::invalid_enum); return; }
    Texture* texture = bound_texture();
    if (texture == nullptr || level < 0 || level >= static_cast<int32_t>(texture_max_levels)) {
        set_error(gl::invalid_operation);
        return;
    }
    TextureLevel& image = texture->levels[level];
    if (image.texels == nullptr || pixels == nullptr || xoffset < 0 || yoffset < 0 ||
        xoffset + width > static_cast<int32_t>(image.width) ||
        yoffset + height > static_cast<int32_t>(image.height)) {
        set_error(gl::invalid_value);
        return;
    }

    // Source channel layout.
    uint32_t src_channels = 4;
    bool bgr = false;
    bool alpha_only = false;
    bool luminance = false;
    switch (format) {
    case gl::rgba: src_channels = 4; break;
    case gl::bgra_ext: src_channels = 4; bgr = true; break;
    case gl::rgb: src_channels = 3; break;
    case gl::bgr_ext: src_channels = 3; bgr = true; break;
    case gl::luminance: src_channels = 1; luminance = true; break;
    case gl::alpha: src_channels = 1; alpha_only = true; break;
    case gl::luminance_alpha: src_channels = 2; luminance = true; break;
    default: set_error(gl::invalid_enum); return;
    }

    const uint32_t element = type == gl::type_float ? 4u : 1u;
    const size_t row_bytes = static_cast<size_t>(width) * src_channels * element;
    const size_t alignment = static_cast<size_t>(unpack_alignment_ < 1 ? 1 : unpack_alignment_);
    const size_t stride = (row_bytes + alignment - 1) / alignment * alignment;
    const uint8_t* source = static_cast<const uint8_t*>(pixels);

    for (int32_t row = 0; row < height; ++row) {
        const uint8_t* src_row = source + static_cast<size_t>(row) * stride;
        uint8_t* dst_row =
            image.texels + (static_cast<size_t>(yoffset + row) * image.width + xoffset) * 4u;
        for (int32_t column = 0; column < width; ++column) {
            float rgba[4] = {0, 0, 0, 1};
            for (uint32_t channel = 0; channel < src_channels; ++channel) {
                float value;
                if (type == gl::type_float) {
                    std::memcpy(&value, src_row + (static_cast<size_t>(column) * src_channels + channel) * 4u, 4);
                } else {
                    value = src_row[static_cast<size_t>(column) * src_channels + channel] / 255.0f;
                }
                if (luminance && channel == 0) { rgba[0] = rgba[1] = rgba[2] = value; }
                else if (luminance && channel == 1) { rgba[3] = value; }
                else if (alpha_only) { rgba[3] = value; }
                else { rgba[channel] = value; }
            }
            if (bgr) { const float swap = rgba[0]; rgba[0] = rgba[2]; rgba[2] = swap; }
            for (uint32_t channel = 0; channel < 4; ++channel) {
                dst_row[static_cast<size_t>(column) * 4u + channel] =
                    static_cast<uint8_t>(clamp01(rgba[channel]) * 255.0f + 0.5f);
            }
        }
    }
    ++texture->generation;
}

void Context::tex_parameter(gl::Enum target, gl::Enum pname, const float* values) noexcept {
    if (target != gl::texture_2d) { set_error(gl::invalid_enum); return; }
    Texture* texture = bound_texture();
    if (texture == nullptr) { set_error(gl::invalid_operation); return; }
    const gl::Enum value = static_cast<gl::Enum>(values[0]);
    switch (pname) {
    case gl::texture_min_filter: texture->sampler.min_filter = value; break;
    case gl::texture_mag_filter: texture->sampler.mag_filter = value; break;
    case gl::texture_wrap_s: texture->sampler.wrap_s = value; break;
    case gl::texture_wrap_t: texture->sampler.wrap_t = value; break;
    case gl::texture_border_color: texture->sampler.border_color = from4(values); break;
    case gl::texture_priority: texture->priority = clamp01(values[0]); break;
    default: set_error(gl::invalid_enum); break;
    }
    ++texture->generation;
}

void Context::tex_env(gl::Enum target, gl::Enum pname, const float* values) noexcept {
    if (target != gl::texture_env) { set_error(gl::invalid_enum); return; }
    if (pname == gl::texture_env_mode) {
        const gl::Enum mode = static_cast<gl::Enum>(values[0]);
        if (mode != gl::modulate && mode != gl::decal && mode != gl::blend && mode != gl::replace) {
            set_error(gl::invalid_enum);
            return;
        }
        texture_env_mode_ = mode;
    } else if (pname == gl::texture_env_color) {
        texture_env_color_ = from4(values);
    } else {
        set_error(gl::invalid_enum);
    }
}

void Context::pixel_store(gl::Enum pname, int32_t value) noexcept {
    if (pname == gl::unpack_alignment || pname == gl::pack_alignment) {
        if (value != 1 && value != 2 && value != 4 && value != 8) { set_error(gl::invalid_value); return; }
        if (pname == gl::unpack_alignment) { unpack_alignment_ = value; } else { pack_alignment_ = value; }
        return;
    }
    switch (pname) {
    case gl::unpack_row_length: case gl::unpack_skip_rows: case gl::unpack_skip_pixels:
        if (value < 0) { set_error(gl::invalid_value); return; }
        if (pname == gl::unpack_row_length) { unpack_row_length_ = value; }
        else if (pname == gl::unpack_skip_rows) { unpack_skip_rows_ = value; }
        else { unpack_skip_pixels_ = value; }
        break;
    case gl::unpack_swap_bytes: unpack_swap_bytes_ = value != 0; break;
    case gl::unpack_lsb_first: unpack_lsb_first_ = value != 0; break;
    default:
        // Pack modes other than the alignment are accepted silently (defaults kept).
        break;
    }
}

// ---- Vertex arrays ----

namespace {
double array_component(const void* base, gl::Enum type, int32_t index) noexcept {
    switch (type) {
    case gl::type_float: return static_cast<const float*>(base)[index];
    case gl::type_double: return static_cast<const double*>(base)[index];
    case gl::type_byte: return static_cast<const int8_t*>(base)[index];
    case gl::type_unsigned_byte: return static_cast<const uint8_t*>(base)[index];
    case gl::type_short: return static_cast<const int16_t*>(base)[index];
    case gl::type_unsigned_short: return static_cast<const uint16_t*>(base)[index];
    case gl::type_int: return static_cast<const int32_t*>(base)[index];
    case gl::type_unsigned_int: return static_cast<const uint32_t*>(base)[index];
    default: return 0;
    }
}
uint32_t type_size(gl::Enum type) noexcept {
    switch (type) {
    case gl::type_byte: case gl::type_unsigned_byte: return 1;
    case gl::type_short: case gl::type_unsigned_short: return 2;
    case gl::type_int: case gl::type_unsigned_int: case gl::type_float: return 4;
    case gl::type_double: return 8;
    default: return 0;
    }
}
}

void Context::vertex_pointer(int32_t size, gl::Enum type, int32_t stride, const void* pointer) noexcept {
    vertex_array_ = {vertex_array_.enabled, size, type, stride, pointer};
}
void Context::normal_pointer(gl::Enum type, int32_t stride, const void* pointer) noexcept {
    normal_array_ = {normal_array_.enabled, 3, type, stride, pointer};
}
void Context::color_pointer(int32_t size, gl::Enum type, int32_t stride, const void* pointer) noexcept {
    color_array_ = {color_array_.enabled, size, type, stride, pointer};
}
void Context::tex_coord_pointer(int32_t size, gl::Enum type, int32_t stride, const void* pointer) noexcept {
    texcoord_array_ = {texcoord_array_.enabled, size, type, stride, pointer};
}

void Context::enable_client_state(gl::Enum array, bool value) noexcept {
    switch (array) {
    case gl::vertex_array: vertex_array_.enabled = value; break;
    case gl::normal_array: normal_array_.enabled = value; break;
    case gl::color_array: color_array_.enabled = value; break;
    case gl::texture_coord_array: texcoord_array_.enabled = value; break;
    case gl::edge_flag_array: case gl::index_array: break;
    default: set_error(gl::invalid_enum); break;
    }
}

void Context::array_element(int32_t index) noexcept {
    if (index < 0) { set_error(gl::invalid_value); return; }
    auto element = [&](const Array& array) -> const uint8_t* {
        const uint32_t stride = array.stride != 0
            ? static_cast<uint32_t>(array.stride)
            : static_cast<uint32_t>(array.size) * type_size(array.type);
        return static_cast<const uint8_t*>(array.pointer) + static_cast<size_t>(index) * stride;
    };
    if (normal_array_.enabled && normal_array_.pointer != nullptr) {
        const uint8_t* p = element(normal_array_);
        normal(array_component(p, normal_array_.type, 0), array_component(p, normal_array_.type, 1),
               array_component(p, normal_array_.type, 2));
    }
    if (color_array_.enabled && color_array_.pointer != nullptr) {
        const uint8_t* p = element(color_array_);
        const bool byte = color_array_.type == gl::type_unsigned_byte;
        const double scale = byte ? 1.0 / 255.0 : 1.0;
        color(array_component(p, color_array_.type, 0) * scale,
              array_component(p, color_array_.type, 1) * scale,
              array_component(p, color_array_.type, 2) * scale,
              color_array_.size >= 4 ? array_component(p, color_array_.type, 3) * scale : 1.0);
    }
    if (texcoord_array_.enabled && texcoord_array_.pointer != nullptr) {
        const uint8_t* p = element(texcoord_array_);
        tex_coord(array_component(p, texcoord_array_.type, 0),
                  texcoord_array_.size >= 2 ? array_component(p, texcoord_array_.type, 1) : 0.0,
                  texcoord_array_.size >= 3 ? array_component(p, texcoord_array_.type, 2) : 0.0,
                  texcoord_array_.size >= 4 ? array_component(p, texcoord_array_.type, 3) : 1.0);
    }
    if (vertex_array_.enabled && vertex_array_.pointer != nullptr) {
        const uint8_t* p = element(vertex_array_);
        vertex(array_component(p, vertex_array_.type, 0),
               vertex_array_.size >= 2 ? array_component(p, vertex_array_.type, 1) : 0.0,
               vertex_array_.size >= 3 ? array_component(p, vertex_array_.type, 2) : 0.0,
               vertex_array_.size >= 4 ? array_component(p, vertex_array_.type, 3) : 1.0);
    }
}

void Context::draw_arrays(gl::Enum mode, int32_t first, int32_t count) noexcept {
    if (count < 0 || first < 0) { set_error(gl::invalid_value); return; }
    if (!vertex_array_.enabled || vertex_array_.pointer == nullptr) { return; }
    begin(mode);
    for (int32_t index = 0; index < count; ++index) { array_element(first + index); }
    end();
}

void Context::draw_elements(gl::Enum mode, int32_t count, gl::Enum type, const void* indices) noexcept {
    if (count < 0) { set_error(gl::invalid_value); return; }
    if (indices == nullptr || !vertex_array_.enabled || vertex_array_.pointer == nullptr) { return; }
    begin(mode);
    for (int32_t index = 0; index < count; ++index) {
        int32_t element_index = 0;
        switch (type) {
        case gl::type_unsigned_byte: element_index = static_cast<const uint8_t*>(indices)[index]; break;
        case gl::type_unsigned_short: element_index = static_cast<const uint16_t*>(indices)[index]; break;
        case gl::type_unsigned_int:
            element_index = static_cast<int32_t>(static_cast<const uint32_t*>(indices)[index]); break;
        default: set_error(gl::invalid_enum); end(); return;
        }
        array_element(element_index);
    }
    end();
}

// ---- Display lists ----

bool Context::record(const ListCommand& command) noexcept {
    if (arena_ == nullptr || arena_used_ >= context_list_arena) {
        set_error(gl::out_of_memory);
        return false;
    }
    arena_[arena_used_++] = command;
    return true;
}

void Context::execute(const ListCommand& command) noexcept {
    using Op = ListCommand::Op;
    const double* v = command.v;
    switch (command.op) {
    case Op::begin: begin(command.e0); break;
    case Op::end: end(); break;
    case Op::vertex: vertex(v[0], v[1], v[2], v[3]); break;
    case Op::color: color(v[0], v[1], v[2], v[3]); break;
    case Op::normal: normal(v[0], v[1], v[2]); break;
    case Op::tex_coord: tex_coord(v[0], v[1], v[2], v[3]); break;
    case Op::shade_model: shade_model(command.e0); break;
    case Op::material: {
        const float f[4] = {static_cast<float>(v[0]), static_cast<float>(v[1]),
                            static_cast<float>(v[2]), static_cast<float>(v[3])};
        material(command.e0, command.e1, f);
        break;
    }
    case Op::enable: enable(command.e0, true); break;
    case Op::disable: enable(command.e0, false); break;
    case Op::color_material: color_material(command.e0, command.e1); break;
    case Op::translate: translate(v[0], v[1], v[2]); break;
    case Op::rotate: rotate(v[0], v[1], v[2], v[3]); break;
    case Op::scale: scale(v[0], v[1], v[2]); break;
    case Op::call_list: call_list(command.e0); break;
    case Op::raster_pos: raster_pos(v[0], v[1], v[2], v[3]); break;
    case Op::bitmap: {
        // Recorded rows are byte aligned, MSB first, ceil(width / 8) bytes each.
        const int32_t width = static_cast<int32_t>(v[0]);
        const uint8_t* rows = command.e1 != 0 && list_bytes_ != nullptr ? list_bytes_ + command.e0 : nullptr;
        bitmap_rows(width, static_cast<int32_t>(v[1]), static_cast<float>(v[2]), static_cast<float>(v[3]),
                    static_cast<float>(v[4]), static_cast<float>(v[5]), rows,
                    (static_cast<size_t>(width) + 7u) / 8u, 0, false);
        break;
    }
    }
}

uint32_t Context::gen_lists(int32_t range) noexcept {
    if (range < 0) { set_error(gl::invalid_value); return 0; }
    if (range == 0) { return 0; }
    for (uint32_t base = 1; base + range <= context_max_lists; ++base) {
        bool free_run = true;
        for (int32_t offset = 0; offset < range; ++offset) {
            if (share_->lists[base + offset - 1].used) { free_run = false; break; }
        }
        if (free_run) {
            for (int32_t offset = 0; offset < range; ++offset) {
                share_->lists[base + offset - 1] = {true, 0, 0};
            }
            return base;
        }
    }
    set_error(gl::out_of_memory);
    return 0;
}

void Context::new_list(uint32_t list, gl::Enum mode) noexcept {
    if (list == 0 || list > context_max_lists) { set_error(gl::invalid_value); return; }
    if (mode != gl::compile && mode != gl::compile_and_execute) { set_error(gl::invalid_enum); return; }
    if (compiling_ || in_begin_) { set_error(gl::invalid_operation); return; }
    compiling_ = true;
    compile_execute_ = mode == gl::compile_and_execute;
    compiling_list_ = list;
    compile_start_ = arena_used_;
}

void Context::end_list() noexcept {
    if (!compiling_) { set_error(gl::invalid_operation); return; }
    ShareGroup::List& entry = share_->lists[compiling_list_ - 1];
    entry.used = true;
    entry.first = compile_start_;
    entry.count = arena_used_ - compile_start_;
    compiling_ = false;
    compile_execute_ = false;
    compiling_list_ = 0;
}

void Context::call_list(uint32_t list) noexcept {
    if (compiling_ && replay_depth_ == 0) {
        ListCommand c{}; c.op = ListCommand::Op::call_list; c.e0 = list;
        if (record(c) && !compile_execute_) { return; }
    }
    if (list == 0 || list > context_max_lists) { return; }
    const ShareGroup::List entry = share_->lists[list - 1];
    if (!entry.used || entry.count == 0) { return; }
    if (replay_depth_ >= 16) { return; }
    ++replay_depth_;
    for (uint32_t index = 0; index < entry.count; ++index) {
        execute(arena_[entry.first + index]);
    }
    --replay_depth_;
}

void Context::call_lists(int32_t count, gl::Enum type, const void* lists) noexcept {
    if (count < 0 || lists == nullptr) { set_error(gl::invalid_value); return; }
    for (int32_t index = 0; index < count; ++index) {
        uint32_t offset = 0;
        switch (type) {
        case gl::type_unsigned_byte: offset = static_cast<const uint8_t*>(lists)[index]; break;
        case gl::type_unsigned_short: offset = static_cast<const uint16_t*>(lists)[index]; break;
        case gl::type_unsigned_int: offset = static_cast<const uint32_t*>(lists)[index]; break;
        default: set_error(gl::invalid_enum); return;
        }
        call_list(list_base_ + offset);
    }
}

void Context::list_base(uint32_t base) noexcept { list_base_ = base; }

void Context::delete_lists(uint32_t list, int32_t range) noexcept {
    if (range < 0) { set_error(gl::invalid_value); return; }
    for (int32_t offset = 0; offset < range; ++offset) {
        const uint32_t target = list + offset;
        if (target >= 1 && target <= context_max_lists) {
            share_->lists[target - 1] = {false, 0, 0};
        }
    }
}

bool Context::is_list(uint32_t list) noexcept {
    return list >= 1 && list <= context_max_lists && share_->lists[list - 1].used;
}

} // namespace rx7900::opengl

/*
Module: rx7900.opengl.fixed_function
Purpose: Bounded compatibility geometry and display lists for the wglgears workload.
Created: 2026-10-21
Architecture: User-mode C++ frontend; a real graphics backend must rasterize output.
No ICD, GL version, native context or hardware acceleration is advertised here.
*/
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace rx7900::opengl {

enum class Status {
    success, invalid_enum, invalid_value, invalid_operation, stack_overflow,
    stack_underflow, capacity_exceeded, list_not_found, recursion_limit,
    backend_not_ready, backend_failure
};

// These are the public OpenGL enumerants accepted by the future ICD translation.
constexpr uint32_t triangles = 0x0004;
constexpr uint32_t quads = 0x0007;
constexpr uint32_t quad_strip = 0x0008;
constexpr uint32_t modelview = 0x1700;
constexpr uint32_t projection = 0x1701;
constexpr uint32_t flat = 0x1d00;
constexpr uint32_t smooth = 0x1d01;
constexpr uint32_t lighting = 0x0b50;
constexpr uint32_t light0 = 0x4000;
constexpr uint32_t normalize = 0x0ba1;
constexpr uint32_t depth_test = 0x0b71;
constexpr uint32_t cull_face = 0x0b44;
constexpr uint32_t position = 0x1203;
constexpr uint32_t ambient = 0x1200;
constexpr uint32_t diffuse = 0x1201;
constexpr uint32_t ambient_and_diffuse = 0x1602;
constexpr uint32_t color_buffer_bit = 0x00004000;
constexpr uint32_t depth_buffer_bit = 0x00000100;

enum class Operation : uint32_t {
    begin, end, vertex, normal, matrix_mode, load_identity, push_matrix,
    pop_matrix, translate, rotate, frustum, viewport, enable, disable,
    shade_model, material, light, call_list, clear, clear_color, clear_depth
};

// Typed, process-owned commands, never a kernel/user wire ABI or GPU packets.
// Record preserves object-space operations so each call uses its caller's matrix.
struct Command final {
    Operation operation{};
    uint32_t parameter = 0;
    double values[6]{};
};

struct Vector4 final { double x, y, z, w; };
struct Vector3 final { double x, y, z; };
struct Color final { double red, green, blue, alpha; };
struct Matrix final { double values[16]{}; };
struct Viewport final { int32_t x, y; uint32_t width, height; };

// Coordinates are homogeneous clip space, before clipping or perspective divide.
// A backend must clip all six planes, interpolate correctly and honor viewport,
// winding/culling and depth. The frontend performs no software rasterization.
struct Vertex final {
    Vector4 clip;
    Vector3 eye_normal;
    Color color;
};

struct RasterState final {
    Viewport viewport;
    bool depth_enabled;
    bool cull_back_faces;
    bool smooth_colors;
};

struct GraphicsBackend final {
    void* context = nullptr;
    // Success means accepted by an initialized graphics submission backend.
    // All three vertices and state are borrowed only for this synchronous call.
    // This is not presentation or completion; those need separately tracked fences.
    // Callbacks must not reenter or modify the originating FixedFunction object.
    Status (*triangle)(void*, const Vertex&, const Vertex&, const Vertex&,
                       const RasterState&) noexcept = nullptr;
    // Clear covers the full owned drawable, independently of viewport. Backends
    // provide both color and depth attachments; default masks enable all writes.
    Status (*clear)(void*, uint32_t mask, const Color&, double depth) noexcept = nullptr;
};

constexpr uint32_t matrix_stack_capacity = 32;
constexpr uint32_t display_list_capacity = 32;
constexpr uint32_t maximum_list_nesting = 8;
constexpr uint32_t maximum_replay_commands = 262144;

class FixedFunction final {
public:
    // Caller owns capacity writable Command objects for the entire context life.
    // Storage must not alias this object, submitted commands or backend state.
    // No allocations or OS handles; caller serializes all access in user mode.
    FixedFunction(Command* storage, size_t capacity,
                  GraphicsBackend backend = {}) noexcept;
    FixedFunction(const FixedFunction&) = delete;
    FixedFunction& operator=(const FixedFunction&) = delete;

    // GL_COMPILE-style storage. IDs 1..display_list_capacity are accepted.
    // Recompiling a list replaces its previous body only on successful end_list.
    // Command arena is append-only; reset_lists reclaims it while idle.
    Status begin_list(uint32_t id) noexcept;
    Status end_list() noexcept;
    Status abort_list() noexcept;
    Status reset_lists() noexcept;
    Status call_list(uint32_t id) noexcept;
    Status command(const Command& command) noexcept;
    Status get_error() noexcept;

    // Read-only state inspection; also used by the eventual ICD query functions.
    Matrix current_matrix() const noexcept;
    RasterState raster_state() const noexcept { return raster_; }
    size_t stored_commands() const noexcept { return used_; }

private:
    struct List final { size_t first = 0, count = 0; bool valid = false; };
    Status execute(const Command& command) noexcept;
    Status replay(uint32_t id) noexcept;
    Status fail(Status status) noexcept;
    Status transform_vertex(const Command& command, Vertex& output) noexcept;
    Status vertex(const Command& command) noexcept;
    Status emit(const Vertex& a, const Vertex& b, const Vertex& c,
                const Vertex& provoking) noexcept;
    Matrix& selected_matrix() noexcept;

    Command* storage_;
    size_t capacity_, used_ = 0, compile_start_ = 0;
    List lists_[display_list_capacity]{};
    uint32_t compiling_ = 0, nesting_ = 0, matrix_mode_ = modelview;
    uint32_t replay_remaining_ = 0;
    bool compile_primitive_ = false, compile_failed_ = false;
    Matrix modelview_stack_[matrix_stack_capacity]{};
    Matrix projection_stack_[matrix_stack_capacity]{};
    uint32_t modelview_depth_ = 0, projection_depth_ = 0;
    GraphicsBackend backend_;
    Status error_ = Status::success;
    bool primitive_active_ = false;
    uint32_t primitive_ = 0, vertex_count_ = 0;
    Vertex vertices_[4]{};
    Vector3 normal_{0, 0, 1};
    Color material_ambient_{0.2, 0.2, 0.2, 1};
    Color material_diffuse_{0.8, 0.8, 0.8, 1};
    Color light_ambient_{0, 0, 0, 1};
    Color light_diffuse_{1, 1, 1, 1};
    Vector4 light_position_{0, 0, 1, 0};
    bool lighting_ = false, light0_ = false, normalize_ = false;
    Color clear_color_{0, 0, 0, 0};
    double clear_depth_ = 1;
    RasterState raster_{{0, 0, 0, 0}, false, false, true};
};

} // namespace rx7900::opengl

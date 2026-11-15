/*
Module: rx7900.tests.opengl
Purpose: Verify bounded display lists, wglgears primitives, transforms and lighting.
Created: 2026-10-21
Architecture: Triangle capture is a unit-test observer, never a graphics backend.
*/
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>

#include "fixed_function.hpp"
#include "gfx11.hpp"
#include "byte_view.hpp"
#include "memory_test_backend.hpp"

namespace {
using namespace rx7900::opengl;
unsigned checks = 0, failures = 0;

void expect(bool value, const char* contract) {
    ++checks;
    if (!value) { ++failures; std::fprintf(stderr, "FAIL: %s\n", contract); }
}

bool close(double left, double right) { return std::abs(left-right) < 0.000001; }

Command command(Operation operation, uint32_t parameter = 0, double x = 0,
                double y = 0, double z = 0, double w = 0, double fifth = 0,
                double sixth = 0) {
    return {operation, parameter, {x, y, z, w, fifth, sixth}};
}

struct Capture final {
    std::array<Vertex, 512> vertices{};
    size_t count = 0;
    RasterState last_state{};
    Status result = Status::success;
    uint32_t clear_mask = 0;
    Color clear_color{};
    double clear_depth = 0;

    static Status triangle(void* opaque, const Vertex& a, const Vertex& b,
                           const Vertex& c, const RasterState& state) noexcept {
        auto& capture = *static_cast<Capture*>(opaque);
        if (capture.result != Status::success) { return capture.result; }
        if (capture.count > capture.vertices.size()-3) { return Status::capacity_exceeded; }
        capture.vertices[capture.count++] = a;
        capture.vertices[capture.count++] = b;
        capture.vertices[capture.count++] = c;
        capture.last_state = state;
        return Status::success;
    }

    static Status clear(void* opaque, uint32_t mask, const Color& color, double depth) noexcept {
        auto& capture = *static_cast<Capture*>(opaque);
        if (capture.result != Status::success) { return capture.result; }
        capture.clear_mask = mask;
        capture.clear_color = color;
        capture.clear_depth = depth;
        return Status::success;
    }

    GraphicsBackend backend() { return {this, triangle, clear}; }
};

Status triangle(FixedFunction& context) {
    Status result = context.command(command(Operation::begin, triangles));
    if (result != Status::success) { return result; }
    for (const auto& point : std::array<std::array<double, 3>, 3>{{{1, 0, 0}, {0, 1, 0}, {0, 0, 0}}}) {
        result = context.command(command(Operation::vertex, 0, point[0], point[1], point[2]));
        if (result != Status::success) { return result; }
    }
    return context.command(command(Operation::end));
}

void check_transforms() {
    std::array<Command, 64> arena{};
    Capture capture{};
    FixedFunction context(arena.data(), arena.size(), capture.backend());
    expect(context.command(command(Operation::translate, 0, 10, 20, -30)) == Status::success,
           "translation accepted");
    expect(context.command(command(Operation::rotate, 0, 90, 0, 0, 1)) == Status::success,
           "rotation accepted");
    expect(triangle(context) == Status::success && capture.count == 3, "triangle reaches observer");
    expect(close(capture.vertices[0].clip.x, 10) && close(capture.vertices[0].clip.y, 21) &&
           close(capture.vertices[0].clip.z, -30), "OpenGL postmultiplication applies rotation before translation");
    expect(context.command(command(Operation::matrix_mode, projection)) == Status::success,
           "projection stack selected");
    expect(context.command(command(Operation::frustum, 0, -1, 1, -1, 1, 1, 100)) == Status::success,
           "frustum accepted");
    capture.count = 0;
    expect(triangle(context) == Status::success, "projected triangle accepted");
    expect(close(capture.vertices[0].clip.w, 30) && close(capture.vertices[0].clip.x, 10),
           "homogeneous coordinates retained for backend clipping");
    const Matrix previous = context.current_matrix();
    expect(context.command(command(Operation::frustum, 0, -1, 1, -1, 1, 0, 10)) == Status::invalid_value,
           "nonpositive near plane rejected");
    expect(close(context.current_matrix().values[0], previous.values[0]), "invalid frustum keeps matrix");
}

void check_stacks_and_errors() {
    FixedFunction context(nullptr, SIZE_MAX);
    expect(context.command(command(Operation::pop_matrix)) == Status::stack_underflow,
           "empty matrix pop rejected");
    expect(context.command(command(Operation::matrix_mode, 123)) == Status::invalid_enum,
           "unknown matrix mode rejected");
    expect(context.get_error() == Status::stack_underflow && context.get_error() == Status::success,
           "first error retained until queried");
    for (uint32_t index = 1; index < matrix_stack_capacity; ++index) {
        expect(context.command(command(Operation::push_matrix)) == Status::success, "bounded matrix push");
    }
    expect(context.command(command(Operation::push_matrix)) == Status::stack_overflow,
           "full matrix stack rejected");
    expect(context.command(command(Operation::matrix_mode, projection)) == Status::success &&
           context.command(command(Operation::push_matrix)) == Status::success, "projection stack independent");
    expect(context.command(command(Operation::rotate, 0, 90, 0, 0, 0)) == Status::invalid_value,
           "zero rotation axis rejected");
    expect(context.command(command(Operation::translate, 0, std::numeric_limits<double>::infinity())) ==
           Status::invalid_value, "nonfinite input rejected before state mutation");
    expect(context.command(command(Operation::viewport, 0, 0, 0, -1, 100)) == Status::invalid_value,
           "negative viewport width rejected before unsigned conversion");
}

void check_primitives() {
    std::array<Command, 16> arena{};
    Capture capture{};
    FixedFunction context(arena.data(), arena.size(), capture.backend());
    expect(context.command(command(Operation::begin, quad_strip)) == Status::success, "quad strip begins");
    expect(context.command(command(Operation::push_matrix)) == Status::invalid_operation,
           "matrix mutation inside begin rejected");
    for (uint32_t index = 0; index < 6; ++index) {
        expect(context.command(command(Operation::vertex, 0, index/2, index%2, 0)) == Status::success,
               "quad strip vertex accepted");
    }
    expect(context.command(command(Operation::end)) == Status::success && capture.count == 12,
           "six strip vertices produce four triangles");
    expect(close(capture.vertices[2].clip.x, 1) && close(capture.vertices[2].clip.y, 1) &&
           close(capture.vertices[5].clip.x, 1) && close(capture.vertices[5].clip.y, 0),
           "quad strip winding follows 0 1 3 2");
    expect(context.command(command(Operation::vertex, 0, 0, 0, 0)) == Status::invalid_operation,
           "vertex outside begin rejected");
    expect(context.command(command(Operation::begin, 12345)) == Status::invalid_enum,
           "unimplemented primitive rejected");
}

void check_lighting_and_flat_color() {
    Capture capture{};
    FixedFunction context(nullptr, 0, capture.backend());
    expect(context.command(command(Operation::enable, lighting)) == Status::success &&
           context.command(command(Operation::enable, light0)) == Status::success &&
           context.command(command(Operation::enable, normalize)) == Status::success,
           "gears light and normal state enabled");
    expect(context.command(command(Operation::material, ambient_and_diffuse, 0.8, 0.1, 0, 1)) == Status::success,
           "gear material assigned");
    expect(context.command(command(Operation::shade_model, flat)) == Status::success &&
           context.command(command(Operation::enable, depth_test)) == Status::success &&
           context.command(command(Operation::enable, cull_face)) == Status::success,
           "gears raster state preserved for backend");
    expect(triangle(context) == Status::success, "lit triangle accepted");
    expect(close(capture.vertices[0].color.red, 0.96) && close(capture.vertices[0].color.green, 0.12) &&
           close(capture.vertices[0].color.blue, 0), "ambient plus diffuse lighting follows material");
    expect(capture.last_state.depth_enabled && capture.last_state.cull_back_faces &&
           !capture.last_state.smooth_colors, "depth/culling/flat state reaches backend");
    capture.count = 0;
    expect(context.command(command(Operation::begin, quads)) == Status::success, "flat quad begins");
    for (uint32_t index = 0; index < 4; ++index) {
        expect(context.command(command(Operation::normal, 0, 0, 0, index == 3 ? -1 : 1)) == Status::success,
               "per-vertex normal accepted");
        expect(context.command(command(Operation::vertex, 0, index%2, index/2, 0)) == Status::success,
               "flat quad vertex accepted");
    }
    expect(context.command(command(Operation::end)) == Status::success && capture.count == 6,
           "quad split into two triangles");
    for (size_t index = 0; index < capture.count; ++index) {
        expect(close(capture.vertices[index].color.red, 0.16),
               "both flat triangles use original quad final provoking vertex");
    }
}

void check_lists() {
    std::array<Command, 32> arena{};
    Capture capture{};
    FixedFunction context(arena.data(), arena.size(), capture.backend());
    expect(context.begin_list(1) == Status::success && triangle(context) == Status::success &&
           context.end_list() == Status::success, "object geometry compiled without execution");
    expect(capture.count == 0 && context.stored_commands() == 5, "compile has no draw side effects");
    expect(context.command(command(Operation::translate, 0, 3, 0, 0)) == Status::success &&
           context.call_list(1) == Status::success, "list replay uses caller modelview");
    expect(close(capture.vertices[0].clip.x, 4), "stored vertices remain object space");
    expect(context.begin_list(1) == Status::success &&
           context.command(command(Operation::begin, triangles)) == Status::success &&
           context.end_list() == Status::invalid_operation, "incomplete replacement rolls back arena");
    capture.count = 0;
    expect(context.call_list(1) == Status::success && capture.count == 3,
           "failed replacement preserves prior list body");
    expect(context.begin_list(2) == Status::success && context.call_list(2) == Status::success &&
           context.end_list() == Status::success, "recursive list can be recorded");
    expect(context.call_list(2) == Status::recursion_limit, "recursive execution bounded");
    expect(context.call_list(display_list_capacity+1) == Status::list_not_found, "out of range list rejected");
    expect(context.reset_lists() == Status::success && context.stored_commands() == 0 &&
           context.call_list(1) == Status::list_not_found, "explicit reset reclaims all list storage");
    FixedFunction empty(nullptr, SIZE_MAX);
    expect(empty.begin_list(1) == Status::success &&
           empty.command(command(Operation::normal, 0, 0, 0, 1)) == Status::capacity_exceeded,
           "null arena cannot be written despite nonzero supplied capacity");
    expect(empty.abort_list() == Status::success, "failed compilation aborts cleanly");
}

void check_gears_workload() {
    std::array<Command, 1024> arena{};
    Capture capture{};
    FixedFunction context(arena.data(), arena.size(), capture.backend());
    expect(context.begin_list(1) == Status::success, "gear display list begins");
    expect(context.command(command(Operation::normal, 0, 0, 0, 1)) == Status::success &&
           context.command(command(Operation::begin, quad_strip)) == Status::success,
           "gear face quad strip begins");
    // A 20-segment annular face exercises the same retained primitive topology
    // and per-frame matrix use as gears. This observer does not render pixels.
    for (uint32_t segment = 0; segment <= 20; ++segment) {
        const double angle = static_cast<double>(segment)*6.283185307179586/20;
        for (double radius : {1.0, 3.0}) {
            expect(context.command(command(Operation::vertex, 0, radius*std::cos(angle),
                   radius*std::sin(angle), 0.5)) == Status::success, "gear face vertex compiled");
        }
    }
    expect(context.command(command(Operation::end)) == Status::success && context.end_list() == Status::success,
           "gear list complete");
    expect(context.command(command(Operation::matrix_mode, projection)) == Status::success &&
           context.command(command(Operation::frustum, 0, -1, 1, -1, 1, 5, 60)) == Status::success &&
           context.command(command(Operation::matrix_mode, modelview)) == Status::success &&
           context.command(command(Operation::translate, 0, 0, 0, -40)) == Status::success,
           "gear scene projection initialized");
    for (double angle : {0.0, 30.0, 60.0}) {
        expect(context.command(command(Operation::push_matrix)) == Status::success &&
               context.command(command(Operation::rotate, 0, angle, 0, 0, 1)) == Status::success &&
               context.call_list(1) == Status::success &&
               context.command(command(Operation::pop_matrix)) == Status::success,
               "gear list replayed under changing frame transform");
    }
    expect(capture.count == 360, "three gear frames produce expected triangles");
}

void check_unavailable_backend() {
    FixedFunction context(nullptr, 0);
    expect(triangle(context) == Status::backend_not_ready, "missing graphics backend cannot report draw success");
    expect(context.command(command(Operation::end)) == Status::success, "caller can close failed immediate primitive");
    Capture capture{};
    capture.result = Status::backend_failure;
    FixedFunction failed(nullptr, 0, capture.backend());
    expect(triangle(failed) == Status::backend_failure && capture.count == 0,
           "hardware submission error propagates without invented output");
}

void check_numeric_boundaries() {
    Capture capture{};
    FixedFunction context(nullptr, 0, capture.backend());
    const double largest = std::numeric_limits<double>::max();
    expect(context.command(command(Operation::translate, 0, largest, 0, 0)) == Status::success,
           "finite representable matrix accepted");
    expect(context.command(command(Operation::translate, 0, largest, 0, 0)) == Status::invalid_value,
           "overflowing matrix composition rejected before commit");
    expect(std::isfinite(context.current_matrix().values[12]), "matrix remains finite after overflow");
    expect(context.command(command(Operation::load_identity)) == Status::success &&
           context.command(command(Operation::viewport, 0, 0, 0, largest, 1)) == Status::invalid_value,
           "huge viewport rejected before narrowing");
    expect(context.command(command(Operation::normal, 0, 0, 0, 0)) == Status::success &&
           context.command(command(Operation::enable, normalize)) == Status::success &&
           context.command(command(Operation::enable, lighting)) == Status::success &&
           context.command(command(Operation::enable, light0)) == Status::success &&
           triangle(context) == Status::success, "zero normal has finite ambient-only result");
    expect(close(capture.vertices[0].color.red, 0.04), "zero normal does not create diffuse light");
    capture.count = 0;
    expect(context.command(command(Operation::material, ambient_and_diffuse, largest, largest, largest, 1)) ==
           Status::success && context.command(command(Operation::light, diffuse, largest, largest, largest, 1)) ==
           Status::success, "finite lighting values retained without unsafe narrowing");
    expect(context.command(command(Operation::normal, 0, 0, 0, 1)) == Status::success &&
           triangle(context) == Status::success && close(capture.vertices[0].color.red, 1),
           "large positive lighting clamps to finite output");
}

void check_clear_and_normal_transform() {
    Capture capture{};
    FixedFunction context(nullptr, 0, capture.backend());
    expect(context.command(command(Operation::clear, color_buffer_bit | depth_buffer_bit)) == Status::success,
           "gears default color and depth clear reaches backend");
    expect(capture.clear_mask == (color_buffer_bit | depth_buffer_bit) &&
           close(capture.clear_color.red, 0) && close(capture.clear_depth, 1),
           "default clear is black and far depth independent of viewport");
    expect(context.command(command(Operation::clear_color, 0, -1, 0.5, 3, 1)) == Status::success &&
           context.command(command(Operation::clear_depth, 0, -1)) == Status::success &&
           context.command(command(Operation::clear, color_buffer_bit)) == Status::success,
           "explicit clear state accepted");
    expect(close(capture.clear_color.red, 0) && close(capture.clear_color.green, 0.5) &&
           close(capture.clear_color.blue, 1) && close(capture.clear_depth, 0),
           "clear components clamp before backend boundary");
    expect(context.command(command(Operation::clear, 0xffffffffu)) == Status::invalid_value,
           "unknown attachment mask rejected");
    expect(context.command(command(Operation::rotate, 0, 90, 0, 1, 0)) == Status::success &&
           triangle(context) == Status::success, "normal transformed by rotated modelview");
    expect(close(capture.vertices[0].eye_normal.x, 1) && close(capture.vertices[0].eye_normal.z, 0),
           "normal inverse transpose matches right handed Y rotation");
    FixedFunction unavailable(nullptr, 0);
    expect(unavailable.command(command(Operation::clear, color_buffer_bit)) == Status::backend_not_ready &&
           unavailable.command(command(Operation::clear, 0)) == Status::success,
           "nonempty clear needs hardware; empty clear performs no work");
}

void check_expansion_budget_and_storage_boundary() {
    std::array<Command, 400> arena{};
    FixedFunction context(arena.data(), arena.size());
    expect(context.begin_list(1) == Status::success &&
           context.command(command(Operation::normal, 0, 0, 0, 1)) == Status::success &&
           context.end_list() == Status::success, "leaf state list compiled");
    for (uint32_t id = 2; id <= 4; ++id) {
        expect(context.begin_list(id) == Status::success, "expansion list begins");
        for (uint32_t index = 0; index < 100; ++index) {
            expect(context.call_list(id-1) == Status::success, "nested reference retained");
        }
        expect(context.end_list() == Status::success, "expansion list complete");
    }
    expect(context.call_list(4) == Status::capacity_exceeded,
           "acyclic exponential list expansion bounded independently of recursion depth");
    expect(context.call_list(1) == Status::success, "new root replay receives fresh execution budget");

    std::array<Command, 2> limited{};
    limited[1].parameter = 0xfeed1234u;
    FixedFunction bounded(limited.data(), 1);
    expect(bounded.begin_list(1) == Status::success &&
           bounded.command(command(Operation::normal, 0, 1, 0, 0)) == Status::success &&
           bounded.command(command(Operation::normal, 0, 0, 1, 0)) == Status::capacity_exceeded,
           "exact arena capacity checked before second write");
    expect(limited[1].parameter == 0xfeed1234u, "object beyond declared arena capacity unchanged");
}

void check_gfx11_packets() {
    namespace gfx = rx7900::opengl::gfx11;
    namespace memory = rx7900::memory;
    rx7900_test::TestBacking backing{};
    memory::Manager manager;
    expect(manager.initialize({0x120000000000ULL, 0x100000}, backing.backend()) == memory::Status::success,
           "GFX packet fixture uses existing managed-memory test backend");
    memory::Handle code{}, data{}, write_only{};
    expect(manager.allocate(memory::page_bytes, memory::page_bytes, memory::Access::read, false, code) ==
           memory::Status::success &&
           manager.allocate(memory::page_bytes, memory::page_bytes, memory::Access::read, false, data) ==
           memory::Status::success &&
           manager.allocate(memory::page_bytes, memory::page_bytes, memory::Access::write, false, write_only) ==
           memory::Status::success, "managed code, vertex and access-denied fixtures allocated");
    std::array<uint8_t, gfx::triangle_commands_bytes> bytes{};
    rx7900::sdma::EncodeResult result{};
    const rx7900::sdma::CommandBuffer output(bytes.data(), bytes.size());
    const rx7900::ByteView view(bytes.data(), bytes.size());
    const auto word = [&](size_t offset) {
        uint32_t value = 0;
        expect(view.read_u32(offset, value), "packet word inside checked ByteView");
        return value;
    };
    gfx::ShaderCode primitive{gfx::Stage::primitive_ngg, {code, 0}, 256};
    gfx::ShaderCode pixel{gfx::Stage::pixel, {code, 256}, 256};
    gfx::ShaderPointer pointer{gfx::Stage::primitive_ngg, {data, 0}, 96, 0, 2};
    const gfx::TriangleDraw draw{3, 1};
    expect(gfx::encode_shader_address(manager, primitive, output, result) == gfx::Status::success &&
           result.bytes_written == gfx::shader_address_bytes, "NGG program address encoded");
    expect(word(0) == 0xc0027600u && word(4) == 0xc8u && word(8) == 0 && word(12) == 0x12u,
           "GFX11 ES address uses SET_SH_REG and retains bits40..47");
    expect(gfx::encode_shader_address(manager, pixel, output, result) == gfx::Status::success &&
           word(4) == 8 && word(8) == 1 && word(12) == 0x12u,
           "PS program address uses separate register bank and aligned offset");
    expect(gfx::encode_shader_pointer(manager, pointer, output, result) == gfx::Status::success &&
           word(4) == 0x8cu && word(8) == 0x1000u && word(12) == 0x1200u,
           "NGG raw-pointer pair uses GS user-data bank and all48 address bits");
    expect(gfx::encode_triangle_draw(draw, output, result) == gfx::Status::success &&
           result.bytes_written == gfx::triangle_draw_bytes,
           "triangle draw tail encoded");
    expect(word(0) == 0xc0017900u && word(4) == 0x242u && word(8) == 4 &&
           word(16) == 0x24bu && word(20) == 0 && word(24) == 0xc0002f00u &&
           word(28) == 1 && word(32) == 0xc0012d00u && word(36) == 3 && word(40) == 2,
           "triangle/restart/instances/draw packet words match pinned Mesa definitions");
    gfx::TriangleCommands commands{primitive, pixel, pointer, draw, 32};
    expect(gfx::encode_triangle_commands(manager, commands, output, result) == gfx::Status::success &&
           result.bytes_written == gfx::triangle_commands_bytes,
           "transactional combined partial draw sequence encoded");
    expect(word(0) == 0xc0027600u && word(16) == 0xc0027600u && word(32) == 0xc0027600u &&
           word(48) == 0xc0017900u, "combined packet boundaries are exact");
    const auto previous = bytes;
    commands.pixel_shader.code.byte_offset = 1;
    expect(gfx::encode_triangle_commands(manager, commands, output, result) == gfx::Status::invalid_alignment &&
           result.bytes_written == 0 && bytes == previous,
           "bad second shader leaves entire output unchanged");
    commands.pixel_shader = pixel;
    commands.vertex_pointer.byte_count = 95;
    expect(gfx::encode_triangle_commands(manager, commands, output, result) == gfx::Status::invalid_draw &&
           bytes == previous, "vertex footprint validated before any write");
    commands.vertex_pointer = pointer;
    for (size_t capacity : {size_t{0}, size_t{15}, gfx::triangle_commands_bytes-1}) {
        expect(gfx::encode_triangle_commands(manager, commands, {bytes.data(), capacity}, result) ==
               gfx::Status::output_too_small && result.bytes_written == 0 &&
               result.bytes_required == gfx::triangle_commands_bytes && bytes == previous,
               "combined output capacity failure is transactional");
    }
    pointer.compiled_user_sgpr_count = 1;
    expect(gfx::encode_shader_pointer(manager, pointer, output, result) == gfx::Status::invalid_shader_abi,
           "pointer cannot overflow a one-register compiler ABI");
    pointer.compiled_user_sgpr_count = 32;
    pointer.first_user_sgpr = UINT32_MAX;
    expect(gfx::encode_shader_pointer(manager, pointer, output, result) == gfx::Status::invalid_shader_abi,
           "SGPR index arithmetic cannot wrap");
    pointer.first_user_sgpr = 30;
    expect(gfx::encode_shader_pointer(manager, pointer, output, result) == gfx::Status::success,
           "last complete pointer pair fits32 user SGPRs");
    pointer.data = {write_only, 0};
    expect(gfx::encode_shader_pointer(manager, pointer, output, result) == gfx::Status::invalid_memory,
           "write-only allocation cannot become a shader read pointer");
    primitive.padded_byte_count = 255;
    expect(gfx::encode_shader_address(manager, primitive, output, result) == gfx::Status::invalid_shader_size,
           "shader instruction prefetch padding must be explicit and aligned");
    primitive = {gfx::Stage::primitive_ngg, {code, UINT64_MAX}, 256};
    expect(gfx::encode_shader_address(manager, primitive, output, result) == gfx::Status::invalid_memory,
           "shader handle offset cannot overflow");
    expect(gfx::encode_triangle_draw({2, 1}, output, result) == gfx::Status::invalid_draw &&
           gfx::encode_triangle_draw({3, 0}, output, result) == gfx::Status::invalid_draw,
           "incomplete triangles and empty instancing rejected");
    expect(manager.release(code) == memory::Status::success, "code allocation released");
    primitive = {gfx::Stage::primitive_ngg, {code, 0}, 256};
    expect(gfx::encode_shader_address(manager, primitive, output, result) == gfx::Status::invalid_memory,
           "stale released shader cannot be encoded");
    expect(manager.shutdown() == memory::Status::success && backing.creates == backing.destroys,
           "packet fixtures release all memory through shared owner");
}
} // namespace

int main() {
    check_transforms();
    check_stacks_and_errors();
    check_primitives();
    check_lighting_and_flat_color();
    check_lists();
    check_gears_workload();
    check_unavailable_backend();
    check_numeric_boundaries();
    check_clear_and_normal_transform();
    check_expansion_budget_and_storage_boundary();
    check_gfx11_packets();
    std::printf("OpenGL frontend: %u checks, %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

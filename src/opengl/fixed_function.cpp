/*
Module: rx7900.opengl.fixed_function
Purpose: Checked fixed-function transform, lighting, primitives and display lists.
Created: 2026-10-21
Architecture: Portable user-mode frontend; needs a GFX11 raster/depth backend.
Contracts studied in Khronos specifications and ReactOS ICD sources; OPENGL.txt.
*/
#include "fixed_function.hpp"

#include <cmath>

namespace rx7900::opengl {
namespace {

Matrix identity() noexcept {
    Matrix value{};
    value.values[0] = value.values[5] = value.values[10] = value.values[15] = 1;
    return value;
}

bool finite(const Matrix& matrix) noexcept {
    for (double value : matrix.values) {
        if (!std::isfinite(value)) { return false; }
    }
    return true;
}

Matrix multiply(const Matrix& left, const Matrix& right) noexcept {
    Matrix output{};
    for (size_t column = 0; column < 4; ++column) {
        for (size_t row = 0; row < 4; ++row) {
            for (size_t inner = 0; inner < 4; ++inner) {
                output.values[column * 4 + row] += left.values[inner * 4 + row] *
                                                   right.values[column * 4 + inner];
            }
        }
    }
    return output;
}

Vector4 transform(const Matrix& m, const Vector4& v) noexcept {
    return {m.values[0]*v.x + m.values[4]*v.y + m.values[8]*v.z + m.values[12]*v.w,
            m.values[1]*v.x + m.values[5]*v.y + m.values[9]*v.z + m.values[13]*v.w,
            m.values[2]*v.x + m.values[6]*v.y + m.values[10]*v.z + m.values[14]*v.w,
            m.values[3]*v.x + m.values[7]*v.y + m.values[11]*v.z + m.values[15]*v.w};
}

bool finite(const Vector4& v) noexcept {
    return std::isfinite(v.x) && std::isfinite(v.y) &&
           std::isfinite(v.z) && std::isfinite(v.w);
}

bool unit_vector(Vector3& vector) noexcept {
    const double magnitude = std::sqrt(vector.x*vector.x + vector.y*vector.y +
                                       vector.z*vector.z);
    if (!std::isfinite(magnitude) || magnitude == 0) { return false; }
    vector.x /= magnitude;
    vector.y /= magnitude;
    vector.z /= magnitude;
    return true;
}

double clamp(double value) noexcept {
    return value < 0 ? 0 : (value > 1 ? 1 : value);
}

// Inverse transpose of the modelview upper-left 3x3, including singular rejection.
bool transform_normal(const Matrix& m, const Vector3& n, Vector3& result) noexcept {
    const double a=m.values[0], b=m.values[4], c=m.values[8];
    const double d=m.values[1], e=m.values[5], f=m.values[9];
    const double g=m.values[2], h=m.values[6], i=m.values[10];
    const double determinant = a*(e*i-f*h) - b*(d*i-f*g) + c*(d*h-e*g);
    if (determinant == 0 || !std::isfinite(determinant)) { return false; }
    result = {((e*i-f*h)*n.x + (f*g-d*i)*n.y + (d*h-e*g)*n.z)/determinant,
              ((c*h-b*i)*n.x + (a*i-c*g)*n.y + (b*g-a*h)*n.z)/determinant,
              ((b*f-c*e)*n.x + (c*d-a*f)*n.y + (a*e-b*d)*n.z)/determinant};
    return std::isfinite(result.x) && std::isfinite(result.y) && std::isfinite(result.z);
}

bool immediate(Operation operation) noexcept {
    return operation == Operation::vertex || operation == Operation::normal ||
           operation == Operation::material || operation == Operation::end ||
           operation == Operation::call_list;
}

} // namespace

FixedFunction::FixedFunction(Command* storage, size_t capacity,
                             GraphicsBackend backend) noexcept
    : storage_(storage), capacity_(storage != nullptr ? capacity : 0), backend_(backend) {
    modelview_stack_[0] = projection_stack_[0] = identity();
}

Status FixedFunction::fail(Status status) noexcept {
    if (status != Status::success && error_ == Status::success) { error_ = status; }
    return status;
}

Status FixedFunction::get_error() noexcept {
    const Status result = error_;
    error_ = Status::success;
    return result;
}

Matrix& FixedFunction::selected_matrix() noexcept {
    return matrix_mode_ == modelview ? modelview_stack_[modelview_depth_] :
                                      projection_stack_[projection_depth_];
}

Matrix FixedFunction::current_matrix() const noexcept {
    return matrix_mode_ == modelview ? modelview_stack_[modelview_depth_] :
                                      projection_stack_[projection_depth_];
}

Status FixedFunction::begin_list(uint32_t id) noexcept {
    if (compiling_ != 0 || primitive_active_) { return fail(Status::invalid_operation); }
    if (id == 0 || id > display_list_capacity) { return fail(Status::invalid_value); }
    compiling_ = id;
    compile_start_ = used_;
    compile_primitive_ = compile_failed_ = false;
    return Status::success;
}

Status FixedFunction::end_list() noexcept {
    if (compiling_ == 0) { return fail(Status::invalid_operation); }
    if (compile_failed_ || compile_primitive_) {
        abort_list();
        return fail(Status::invalid_operation);
    }
    lists_[compiling_ - 1] = {compile_start_, used_ - compile_start_, true};
    compiling_ = 0;
    return Status::success;
}

Status FixedFunction::abort_list() noexcept {
    if (compiling_ == 0) { return fail(Status::invalid_operation); }
    used_ = compile_start_;
    compiling_ = 0;
    compile_primitive_ = compile_failed_ = false;
    return Status::success;
}

Status FixedFunction::reset_lists() noexcept {
    if (compiling_ != 0 || primitive_active_ || nesting_ != 0) {
        return fail(Status::invalid_operation);
    }
    for (List& list : lists_) { list = {}; }
    used_ = 0;
    return Status::success;
}

Status FixedFunction::command(const Command& value) noexcept {
    for (double number : value.values) {
        if (!std::isfinite(number)) {
            if (compiling_ != 0) { compile_failed_ = true; }
            return fail(Status::invalid_value);
        }
    }
    if (compiling_ == 0) { return fail(execute(value)); }
    if (compile_failed_) { return fail(Status::invalid_operation); }
    if (used_ == capacity_) {
        compile_failed_ = true;
        return fail(Status::capacity_exceeded);
    }
    if ((value.operation == Operation::begin && compile_primitive_) ||
        (value.operation == Operation::end && !compile_primitive_) ||
        (value.operation == Operation::vertex && !compile_primitive_) ||
        (compile_primitive_ && !immediate(value.operation))) {
        compile_failed_ = true;
        return fail(Status::invalid_operation);
    }
    if (value.operation == Operation::begin) { compile_primitive_ = true; }
    if (value.operation == Operation::end) { compile_primitive_ = false; }
    // Constructor's lifetime contract and capacity check guard the only writes.
    storage_[used_] = value;
    ++used_;
    return Status::success;
}

Status FixedFunction::call_list(uint32_t id) noexcept {
    Command value{};
    value.operation = Operation::call_list;
    value.parameter = id;
    return command(value);
}

Status FixedFunction::replay(uint32_t id) noexcept {
    if (id == 0 || id > display_list_capacity || !lists_[id - 1].valid) {
        return Status::list_not_found;
    }
    if (nesting_ == maximum_list_nesting) { return Status::recursion_limit; }
    if (nesting_ == 0) { replay_remaining_ = maximum_replay_commands; }
    const List list = lists_[id - 1];
    ++nesting_;
    Status result = Status::success;
    for (size_t index = 0; index < list.count; ++index) {
        if (replay_remaining_ == 0) { result = Status::capacity_exceeded; break; }
        --replay_remaining_;
        result = execute(storage_[list.first + index]);
        if (result != Status::success) { break; }
    }
    --nesting_;
    if (result != Status::success) {
        // An interrupted begin/end body cannot poison subsequent commands.
        primitive_active_ = false;
        vertex_count_ = 0;
    }
    return result;
}

Status FixedFunction::emit(const Vertex& a, const Vertex& b, const Vertex& c,
                           const Vertex& provoking) noexcept {
    if (backend_.triangle == nullptr) { return Status::backend_not_ready; }
    Vertex first = a, second = b, third = c;
    if (!raster_.smooth_colors) {
        first.color = second.color = third.color = provoking.color;
    }
    return backend_.triangle(backend_.context, first, second, third, raster_);
}

Status FixedFunction::transform_vertex(const Command& value, Vertex& output) noexcept {
    const Matrix& model = modelview_stack_[modelview_depth_];
    const Vector4 eye = transform(model, {value.values[0], value.values[1], value.values[2], 1});
    Vertex result{};
    result.clip = transform(projection_stack_[projection_depth_], eye);
    if (!finite(result.clip) || !finite(eye) ||
        !transform_normal(model, normal_, result.eye_normal)) { return Status::invalid_value; }
    if (normalize_ && !unit_vector(result.eye_normal)) {
        // Zero normals yield no diffuse component, rather than a divide by zero.
        result.eye_normal = {};
    }
    result.color = {1, 1, 1, 1};
    if (lighting_) {
        double factor = 0;
        Vector3 direction{light_position_.x, light_position_.y, light_position_.z};
        if (light_position_.w != 0) {
            if (eye.w == 0) { return Status::invalid_value; }
            direction = {light_position_.x/light_position_.w - eye.x/eye.w,
                         light_position_.y/light_position_.w - eye.y/eye.w,
                         light_position_.z/light_position_.w - eye.z/eye.w};
        }
        if (light0_ && unit_vector(direction)) {
            factor = direction.x*result.eye_normal.x + direction.y*result.eye_normal.y +
                     direction.z*result.eye_normal.z;
            if (factor < 0) { factor = 0; }
        }
        const double ambient_red = 0.2 + (light0_ ? light_ambient_.red : 0);
        const double ambient_green = 0.2 + (light0_ ? light_ambient_.green : 0);
        const double ambient_blue = 0.2 + (light0_ ? light_ambient_.blue : 0);
        result.color = {clamp(material_ambient_.red*ambient_red + material_diffuse_.red*light_diffuse_.red*factor),
                        clamp(material_ambient_.green*ambient_green + material_diffuse_.green*light_diffuse_.green*factor),
                        clamp(material_ambient_.blue*ambient_blue + material_diffuse_.blue*light_diffuse_.blue*factor),
                        clamp(material_diffuse_.alpha)};
    }
    if (!std::isfinite(result.color.red) || !std::isfinite(result.color.green) ||
        !std::isfinite(result.color.blue) || !std::isfinite(result.color.alpha)) {
        return Status::invalid_value;
    }
    output = result;
    return Status::success;
}

Status FixedFunction::vertex(const Command& value) noexcept {
    if (!primitive_active_) { return Status::invalid_operation; }
    Vertex next{};
    Status result = transform_vertex(value, next);
    if (result != Status::success) { return result; }
    vertices_[vertex_count_] = next;
    ++vertex_count_;
    if (primitive_ == triangles && vertex_count_ == 3) {
        vertex_count_ = 0;
        return emit(vertices_[0], vertices_[1], vertices_[2], vertices_[2]);
    }
    if (primitive_ != triangles && vertex_count_ == 4) {
        result = primitive_ == quads ? emit(vertices_[0], vertices_[1], vertices_[2], vertices_[3]) :
                                      emit(vertices_[0], vertices_[1], vertices_[3], vertices_[3]);
        if (result == Status::success) {
            result = primitive_ == quads ? emit(vertices_[0], vertices_[2], vertices_[3], vertices_[3]) :
                                          emit(vertices_[0], vertices_[3], vertices_[2], vertices_[3]);
        }
        if (primitive_ == quad_strip) {
            vertices_[0] = vertices_[2];
            vertices_[1] = vertices_[3];
            vertex_count_ = 2;
        } else { vertex_count_ = 0; }
    }
    return result;
}

Status FixedFunction::execute(const Command& value) noexcept {
    if (primitive_active_ && !immediate(value.operation)) { return Status::invalid_operation; }
    switch (value.operation) {
    case Operation::clear:
        if ((value.parameter & ~(color_buffer_bit | depth_buffer_bit)) != 0) {
            return Status::invalid_value;
        }
        if (value.parameter == 0) { return Status::success; }
        if (backend_.clear == nullptr) { return Status::backend_not_ready; }
        return backend_.clear(backend_.context, value.parameter, clear_color_, clear_depth_);
    case Operation::clear_color:
        clear_color_ = {clamp(value.values[0]), clamp(value.values[1]),
                        clamp(value.values[2]), clamp(value.values[3])};
        return Status::success;
    case Operation::clear_depth:
        clear_depth_ = clamp(value.values[0]);
        return Status::success;
    case Operation::call_list: return replay(value.parameter);
    case Operation::begin:
        if (value.parameter != triangles && value.parameter != quads && value.parameter != quad_strip) {
            return Status::invalid_enum;
        }
        primitive_ = value.parameter;
        primitive_active_ = true;
        vertex_count_ = 0;
        return Status::success;
    case Operation::end:
        if (!primitive_active_) { return Status::invalid_operation; }
        primitive_active_ = false;
        vertex_count_ = 0;
        return Status::success;
    case Operation::normal:
        normal_ = {value.values[0], value.values[1], value.values[2]};
        return Status::success;
    case Operation::vertex: return vertex(value);
    case Operation::matrix_mode:
        if (value.parameter != modelview && value.parameter != projection) { return Status::invalid_enum; }
        matrix_mode_ = value.parameter;
        return Status::success;
    case Operation::load_identity: selected_matrix() = identity(); return Status::success;
    case Operation::push_matrix: {
        uint32_t& depth = matrix_mode_ == modelview ? modelview_depth_ : projection_depth_;
        Matrix* stack = matrix_mode_ == modelview ? modelview_stack_ : projection_stack_;
        if (depth + 1 == matrix_stack_capacity) { return Status::stack_overflow; }
        stack[depth + 1] = stack[depth];
        ++depth;
        return Status::success;
    }
    case Operation::pop_matrix: {
        uint32_t& depth = matrix_mode_ == modelview ? modelview_depth_ : projection_depth_;
        if (depth == 0) { return Status::stack_underflow; }
        --depth;
        return Status::success;
    }
    case Operation::translate:
    case Operation::rotate:
    case Operation::frustum: {
        Matrix right = identity();
        if (value.operation == Operation::translate) {
            right.values[12] = value.values[0];
            right.values[13] = value.values[1];
            right.values[14] = value.values[2];
        } else if (value.operation == Operation::rotate) {
            Vector3 axis{value.values[1], value.values[2], value.values[3]};
            if (!unit_vector(axis)) { return Status::invalid_value; }
            const double radians = std::fmod(value.values[0], 360.0)*0.017453292519943295769;
            const double c=std::cos(radians), s=std::sin(radians), t=1-c;
            right.values[0]=axis.x*axis.x*t+c;
            right.values[1]=axis.y*axis.x*t+axis.z*s;
            right.values[2]=axis.z*axis.x*t-axis.y*s;
            right.values[4]=axis.x*axis.y*t-axis.z*s;
            right.values[5]=axis.y*axis.y*t+c;
            right.values[6]=axis.z*axis.y*t+axis.x*s;
            right.values[8]=axis.x*axis.z*t+axis.y*s;
            right.values[9]=axis.y*axis.z*t-axis.x*s;
            right.values[10]=axis.z*axis.z*t+c;
        } else {
            const double l=value.values[0], r=value.values[1], b=value.values[2];
            const double t=value.values[3], n=value.values[4], f=value.values[5];
            if (l == r || b == t || n == f || n <= 0 || f <= 0) { return Status::invalid_value; }
            right = {};
            right.values[0]=2*n/(r-l);
            right.values[5]=2*n/(t-b);
            right.values[8]=(r+l)/(r-l);
            right.values[9]=(t+b)/(t-b);
            right.values[10]=-(f+n)/(f-n);
            right.values[11]=-1;
            right.values[14]=-2*f*n/(f-n);
        }
        const Matrix result = multiply(selected_matrix(), right);
        if (!finite(result)) { return Status::invalid_value; }
        selected_matrix() = result;
        return Status::success;
    }
    case Operation::viewport:
        if (value.values[0] < INT32_MIN || value.values[0] > INT32_MAX ||
            value.values[1] < INT32_MIN || value.values[1] > INT32_MAX ||
            value.values[2] < 0 || value.values[2] > INT32_MAX ||
            value.values[3] < 0 || value.values[3] > INT32_MAX) { return Status::invalid_value; }
        raster_.viewport = {static_cast<int32_t>(value.values[0]), static_cast<int32_t>(value.values[1]),
                            static_cast<uint32_t>(value.values[2]), static_cast<uint32_t>(value.values[3])};
        return Status::success;
    case Operation::enable:
    case Operation::disable: {
        const bool enabled = value.operation == Operation::enable;
        switch (value.parameter) {
        case lighting: lighting_ = enabled; break;
        case light0: light0_ = enabled; break;
        case normalize: normalize_ = enabled; break;
        case depth_test: raster_.depth_enabled = enabled; break;
        case cull_face: raster_.cull_back_faces = enabled; break;
        default: return Status::invalid_enum;
        }
        return Status::success;
    }
    case Operation::shade_model:
        if (value.parameter != flat && value.parameter != smooth) { return Status::invalid_enum; }
        raster_.smooth_colors = value.parameter == smooth;
        return Status::success;
    case Operation::material:
    case Operation::light: {
        const Color color{value.values[0], value.values[1], value.values[2], value.values[3]};
        if (value.operation == Operation::material) {
            if (value.parameter != ambient && value.parameter != diffuse &&
                value.parameter != ambient_and_diffuse) { return Status::invalid_enum; }
            if (value.parameter != diffuse) { material_ambient_ = color; }
            if (value.parameter != ambient) { material_diffuse_ = color; }
        } else if (value.parameter == position) {
            const Vector4 transformed = transform(modelview_stack_[modelview_depth_],
                {value.values[0], value.values[1], value.values[2], value.values[3]});
            if (!finite(transformed)) { return Status::invalid_value; }
            light_position_ = transformed;
        } else if (value.parameter == ambient) { light_ambient_ = color; }
        else if (value.parameter == diffuse) { light_diffuse_ = color; }
        else { return Status::invalid_enum; }
        return Status::success;
    }
    }
    return Status::invalid_enum;
}

} // namespace rx7900::opengl

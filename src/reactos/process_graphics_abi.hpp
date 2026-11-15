/*
Module: rx7900.reactos.process_graphics_abi
Purpose: Define the bounded, typed graphics-submission and present wire formats of
the version-3 native process API: opaque buffer handles, enumerated GL state,
shader keys, draw counts and vertex records. No register, register value, GPU
address, shader byte or command stream crosses this ABI.
Created: 2026-10-23
Architecture: Header-only and SDK-free; shared by the kernel decoder
(process_graphics), the radeonglicd encoder (gl_gfx11_backend) and host tests.
Every structure is fixed-width little-endian with explicit offsets; decoders read
captured bytes through ByteView (no overlays) and reject unknown enumerants,
nonzero reserved fields, out-of-range counts, non-finite floats and any size
mismatch. The ShaderKey layout is gl_gfx11_abi.hpp's (owned by gl-shaders).
Submission layout 4 (2026-10-24, additive): a 64-byte header (second colour
buffer for GL_FRONT_AND_BACK), 628-byte states carrying the 28-byte v4 key,
stipple patterns, texture levels 1..10 and the pixel-rectangle source; the
header size selects the layout and layout-3 submissions decode unchanged.
*/
#pragma once

#include <stdint.h>
#include "../common/byte_view.hpp"
#include "../opengl/gl_gfx11_abi.hpp"

namespace rx7900::reactos::graphics_abi {

constexpr uint32_t version = 3;
constexpr uint32_t max_states = 16;
constexpr uint32_t max_commands = 64;
constexpr uint32_t max_vertex_bytes = 192u * 1024u;
constexpr uint32_t bulk_bytes = 256u * 1024u;          // upload/readback per request
constexpr uint32_t max_dimension = 4096;
constexpr uint32_t max_present_rects = 64;
// Process-buffer limits user mode plans with (ProcessMemory enforces them;
// process_graphics.hpp asserts equality).
constexpr uint64_t buffer_max_bytes = 128ULL << 20;
constexpr uint64_t buffer_page_bytes = 4096;

enum class TargetKind : uint32_t { bgra8 = 0, depth_copy = 1 };
enum class Cull : uint32_t { none = 0, front = 1, back = 2, front_and_back = 3 };
enum class Wrap : uint32_t { repeat = 0, clamp = 1 };
enum class Filter : uint32_t { nearest = 0, linear = 1 };
constexpr uint32_t command_depth_copy = 1u;             // GraphicsCommand.flags

// Header of a graphics_submit payload, followed by state_count GraphicsState,
// command_count GraphicsCommand and vertex_bytes of vertex records.
struct SubmitHeader final {
    uint32_t header_bytes;      // submit_header_bytes (layout 3) or submit_header_v4_bytes
    uint32_t state_count;       // 1..max_states
    uint32_t command_count;     // 1..max_commands
    uint32_t vertex_bytes;      // 16-byte multiple, <= max_vertex_bytes
    uint64_t color;             // opaque process-buffer handles
    uint64_t depth;             // 0 = no depth/stencil buffer
    uint32_t width;
    uint32_t height;
    uint32_t target_kind;       // TargetKind
    uint32_t reserved;
    // Layout 4 only (zero in layout 3).
    uint64_t color2 = 0;        // GL_FRONT_AND_BACK second colour buffer (MRT1), 0 = none
    uint32_t layout = 0;        // == submit_layout_v4
    uint32_t reserved2 = 0;
};
constexpr uint32_t submit_header_bytes = 48;
constexpr uint32_t submit_header_v4_bytes = 64;
constexpr uint32_t submit_layout_v4 = 4;

struct GraphicsState final {
    int32_t viewport_x, viewport_y;
    uint32_t viewport_width, viewport_height;   // 0..2*max_dimension
    uint32_t depth_near, depth_far;             // float bits in [0, 1]
    uint32_t scissor;                           // 0/1
    int32_t scissor_x, scissor_y;
    uint32_t scissor_width, scissor_height;     // <= max_dimension*2
    uint32_t cull;                              // Cull
    uint32_t front_ccw;                         // 0/1
    uint32_t point_size, line_width;            // float bits in (0, 256]
    uint32_t offset;                            // 0/1
    uint32_t offset_factor, offset_units;       // finite float bits
    uint32_t depth_test, depth_func, depth_write;
    uint32_t stencil_test, stencil_func, stencil_ref, stencil_value_mask, stencil_write_mask;
    uint32_t stencil_fail, stencil_depth_fail, stencil_depth_pass;   // 0..5
    uint32_t blend, blend_src, blend_dst;       // factors 0..10
    uint32_t logic_op, logic_op_code;           // code 0..15 (GL_CLEAR..GL_SET)
    uint32_t color_mask;                        // 0..15
    uint32_t alpha_ref;                         // float bits in [0, 1]
    uint32_t env_color[4];                      // float bits in [0, 1]
    uint32_t fog_color[3];                      // float bits in [0, 1]
    uint64_t texture;                           // opaque handle, 0 when untextured
    uint32_t texture_offset;                    // 256-byte aligned
    uint32_t texture_width, texture_height;     // 1..max_dimension
    uint32_t texture_pitch;                     // texels, multiple of 64, >= width
    uint32_t wrap_s, wrap_t;                    // Wrap
    uint32_t filter;                            // Filter
    rx7900::opengl::gfx11_abi::ShaderKey key;   // gl_gfx11_abi wire structure
    // Layout 4 (zero for layout-3 states and for unused features).
    uint32_t line_stipple_factor;               // 1..256 with key.line_stipple
    uint32_t line_stipple_pattern;              // 0..0xffff
    uint32_t polygon_stipple[32];               // with key.polygon_stipple
    uint32_t level_count;                       // levels in use: 0 untextured, 1, or the mip chain
    struct Level final {
        uint32_t offset, width, height, pitch;  // as texture_offset/width/height/pitch
    } levels[10];                               // levels 1..10
    uint64_t pixel_buffer;                      // pixel source image handle
    uint32_t pixel_offset;                      // byte offset of image row 0 column 0 (wraps)
    int32_t pixel_row_stride;                   // bytes between image rows
    uint32_t pixel_columns;                     // 1..max_pixel_columns
    uint32_t pixel_first_row, pixel_end_row;    // rows present, first < end <= max_pixel_rows
    uint32_t raster_x, raster_y;                // finite float bits: image origin
    uint32_t inv_zoom_x, inv_zoom_y;            // finite float bits: 1 / zoom
    uint32_t transfer_scale[4], transfer_bias[4];   // finite float bits (key.pixel_transfer)
    uint32_t index_shift_left, index_shift_right;   // 0..31 (key.pixel_transfer)
    int32_t index_offset;
    uint32_t reserved_v4;
};
// Layout 3 carries the 20-byte v3 key (17 fields and three zero bytes).
constexpr uint32_t shader_key_bytes = rx7900::opengl::gfx11_abi::shader_key_v3_bytes;
constexpr uint32_t shader_key_v4_bytes = sizeof(rx7900::opengl::gfx11_abi::ShaderKey);
static_assert(shader_key_bytes == 20 && shader_key_v4_bytes == 28, "gl_gfx11_abi v4 ShaderKey sizes");
// 43 dwords, the texture handle and 7 dwords (208 bytes), then the key.
constexpr uint32_t state_key_offset = 208;
constexpr uint32_t state_bytes = state_key_offset + shader_key_bytes;
// Layout 4: the 28-byte key, then the extension (offsets in the encoder).
constexpr uint32_t state_v4_extension_offset = state_key_offset + shader_key_v4_bytes;
constexpr uint32_t state_v4_bytes = 628;
constexpr uint32_t max_texture_levels = 11;
constexpr uint32_t max_pixel_columns = 65536;
constexpr uint32_t max_pixel_rows = 1u << 24;

struct GraphicsCommand final {
    uint32_t state_index;
    uint32_t primitive;        // vertices per primitive: 1, 2 or 3
    uint32_t first_vertex;
    uint32_t vertex_count;     // multiple of primitive
    uint32_t flags;            // command_depth_copy
    uint32_t reserved;
};
constexpr uint32_t command_bytes = 24;

// buffer_upload/readback payload descriptor: Request.handle names the buffer,
// Request.offset the byte offset and Request.bytes the length (<= bulk_bytes).

// Present: the ICD's DrawEscape input (PresentInput) names its session and the
// rendered back buffer; radeondisp DrvDrawEscape, the only caller of the private
// present IOCTL, appends the window origin and the CLIPOBJ rectangles win32k
// supplied (screen coordinates, already intersected with the window and the
// primary surface) and forwards PresentRequest. User mode cannot reach the
// IOCTL: the VideoPort device is exclusive to win32k and radeondisp never
// forwards caller bytes as this IOCTL code.
constexpr uint32_t present_escape = 0x52474c50u;        // 'RGLP', DrawEscape code
// ExtEscape code radeondisp DrvEscape forwards verbatim as process_abi::ioctl
// (input: Request + payload, output: Reply + payload); see gl_gfx11_backend.
constexpr uint32_t process_escape = 0x52474c51u;        // 'RGLQ'
struct PresentInput final {
    uint32_t size;              // == present_input_bytes
    uint32_t version;           // == version
    uint64_t session;
    uint64_t buffer;            // bgra8 colour buffer handle
    uint32_t width, height;     // drawable size the buffer was rendered at
};
constexpr uint32_t present_input_bytes = 32;

struct PresentRect final {
    int32_t left, top, right, bottom;   // screen pixels, right/bottom exclusive
};
struct PresentRequest final {
    PresentInput input;
    int32_t window_left, window_top;    // screen position of drawable pixel (0, 0)
    uint32_t rect_count;                // 0..max_present_rects
    uint32_t reserved;
    PresentRect rects[max_present_rects];
};
constexpr uint32_t present_request_bytes = present_input_bytes + 16 + 16 * max_present_rects;
// FILE_DEVICE_VIDEO, private function 0xb01, METHOD_BUFFERED, FILE_ANY_ACCESS.
constexpr uint32_t present_ioctl = (0x23u << 16) | (0xb01u << 2);
struct PresentReply final {
    uint32_t outcome;           // process_abi::Outcome
    uint32_t rects_copied;
};
constexpr uint32_t present_reply_bytes = 8;

// ---- Validation helpers over float bit patterns (no floating point) ----

constexpr bool finite_bits(uint32_t bits) noexcept { return (bits & 0x7f800000u) != 0x7f800000u; }
// Non-negative finite floats order like their bit patterns.
constexpr bool unit_bits(uint32_t bits) noexcept { return bits <= 0x3f800000u; }   // [+0, 1]
constexpr bool size_bits(uint32_t bits) noexcept {
    return bits >= 0x3c000000u && bits <= 0x43800000u;     // [2^-7, 256]
}

class Reader final {
public:
    Reader(ByteView view, size_t offset) noexcept : view_(view), offset_(offset) {}
    bool u32(uint32_t& value) noexcept { return advance(view_.read_u32(offset_, value), 4); }
    bool i32(int32_t& value) noexcept {
        uint32_t raw = 0;
        if (!u32(raw)) return false;
        value = static_cast<int32_t>(raw);
        return true;
    }
    bool u64(uint64_t& value) noexcept { return advance(view_.read_u64(offset_, value), 8); }
    bool u8(uint8_t& value) noexcept { return advance(view_.read_u8(offset_, value), 1); }
    size_t offset() const noexcept { return offset_; }
    bool ok() const noexcept { return ok_; }

private:
    bool advance(bool read, size_t bytes) noexcept {
        if (!read || !ok_) {
            ok_ = false;
            return false;
        }
        offset_ += bytes;
        return true;
    }
    ByteView view_;
    size_t offset_;
    bool ok_ = true;
};

// Little-endian encoder for user-mode producers and tests. capacity bounds every
// store; ok() is false after any overflow and nothing past capacity is written.
class Writer final {
public:
    Writer(uint8_t* output, size_t capacity, size_t offset) noexcept
        : output_(output), capacity_(output != nullptr ? capacity : 0), offset_(offset) {}
    void u32(uint32_t value) noexcept { bytes(value, 4); }
    void i32(int32_t value) noexcept { bytes(static_cast<uint32_t>(value), 4); }
    void u64(uint64_t value) noexcept { bytes(value, 8); }
    void u8(uint8_t value) noexcept { bytes(value, 1); }
    size_t offset() const noexcept { return offset_; }
    bool ok() const noexcept { return ok_; }

private:
    void bytes(uint64_t value, uint32_t count) noexcept {
        if (!ok_ || offset_ > capacity_ || count > capacity_ - offset_) {
            ok_ = false;
            return;
        }
        for (uint32_t i = 0; i < count; ++i) {
            output_[offset_ + i] = static_cast<uint8_t>(value >> (8 * i));
        }
        offset_ += count;
    }
    uint8_t* output_;
    size_t capacity_;
    size_t offset_;
    bool ok_ = true;
};

inline bool encode_header(const SubmitHeader& h, uint8_t* output, size_t capacity) noexcept {
    Writer w(output, capacity, 0);
    w.u32(h.header_bytes); w.u32(h.state_count); w.u32(h.command_count); w.u32(h.vertex_bytes);
    w.u64(h.color); w.u64(h.depth); w.u32(h.width); w.u32(h.height); w.u32(h.target_kind);
    w.u32(h.reserved);
    if (h.header_bytes == submit_header_v4_bytes) {
        w.u64(h.color2); w.u32(h.layout); w.u32(h.reserved2);
    }
    return w.ok() && w.offset() == h.header_bytes &&
           (h.header_bytes == submit_header_bytes || h.header_bytes == submit_header_v4_bytes);
}

inline bool encode_state(const GraphicsState& s, uint8_t* output, size_t capacity) noexcept {
    Writer w(output, capacity, 0);
    w.i32(s.viewport_x); w.i32(s.viewport_y); w.u32(s.viewport_width); w.u32(s.viewport_height);
    w.u32(s.depth_near); w.u32(s.depth_far); w.u32(s.scissor); w.i32(s.scissor_x); w.i32(s.scissor_y);
    w.u32(s.scissor_width); w.u32(s.scissor_height); w.u32(s.cull); w.u32(s.front_ccw);
    w.u32(s.point_size); w.u32(s.line_width); w.u32(s.offset); w.u32(s.offset_factor);
    w.u32(s.offset_units); w.u32(s.depth_test); w.u32(s.depth_func); w.u32(s.depth_write);
    w.u32(s.stencil_test); w.u32(s.stencil_func); w.u32(s.stencil_ref); w.u32(s.stencil_value_mask);
    w.u32(s.stencil_write_mask); w.u32(s.stencil_fail); w.u32(s.stencil_depth_fail);
    w.u32(s.stencil_depth_pass); w.u32(s.blend); w.u32(s.blend_src); w.u32(s.blend_dst);
    w.u32(s.logic_op); w.u32(s.logic_op_code); w.u32(s.color_mask); w.u32(s.alpha_ref);
    for (uint32_t c : s.env_color) w.u32(c);
    for (uint32_t c : s.fog_color) w.u32(c);
    w.u64(s.texture); w.u32(s.texture_offset); w.u32(s.texture_width); w.u32(s.texture_height);
    w.u32(s.texture_pitch); w.u32(s.wrap_s); w.u32(s.wrap_t); w.u32(s.filter);
    const auto& k = s.key;
    w.u8(k.vertex_input); w.u8(k.smooth); w.u8(k.two_sided); w.u8(k.lighting); w.u8(k.light_mask);
    w.u8(k.positional_mask); w.u8(k.spot_mask); w.u8(k.normalize); w.u8(k.color_material);
    w.u8(k.fog); w.u8(k.fog_mode); w.u8(k.textured); w.u8(k.tex_env); w.u8(k.texture_base);
    w.u8(k.alpha_test); w.u8(k.alpha_func); w.u8(k.vertices_per_primitive);
    w.u8(k.reserved[0]); w.u8(k.reserved[1]); w.u8(k.reserved[2]);
    return w.ok() && w.offset() == state_bytes;
}

// Layout-4 state: the layout-3 fields, the 28-byte key and the extension.
inline bool encode_state_v4(const GraphicsState& s, uint8_t* output, size_t capacity) noexcept {
    uint8_t head[state_bytes]{};
    if (!encode_state(s, head, sizeof(head)) || output == nullptr || capacity < state_v4_bytes) return false;
    Writer w(output, capacity, 0);
    for (uint32_t i = 0; i < state_key_offset; ++i) w.u8(head[i]);
    const auto& k = s.key;
    w.u8(k.vertex_input); w.u8(k.smooth); w.u8(k.two_sided); w.u8(k.lighting); w.u8(k.light_mask);
    w.u8(k.positional_mask); w.u8(k.spot_mask); w.u8(k.normalize); w.u8(k.color_material);
    w.u8(k.fog); w.u8(k.fog_mode); w.u8(k.textured); w.u8(k.tex_env); w.u8(k.texture_base);
    w.u8(k.alpha_test); w.u8(k.alpha_func); w.u8(k.vertices_per_primitive);
    w.u8(k.min_filter); w.u8(k.mag_filter); w.u8(k.line_stipple); w.u8(k.polygon_stipple);
    w.u8(k.front_and_back); w.u8(k.pixel_source); w.u8(k.pixel_transfer);
    for (uint8_t b : k.reserved) w.u8(b);
    w.u32(s.line_stipple_factor); w.u32(s.line_stipple_pattern);
    for (uint32_t row : s.polygon_stipple) w.u32(row);
    w.u32(s.level_count);
    for (const auto& level : s.levels) { w.u32(level.offset); w.u32(level.width); w.u32(level.height); w.u32(level.pitch); }
    w.u64(s.pixel_buffer); w.u32(s.pixel_offset); w.i32(s.pixel_row_stride); w.u32(s.pixel_columns);
    w.u32(s.pixel_first_row); w.u32(s.pixel_end_row); w.u32(s.raster_x); w.u32(s.raster_y);
    w.u32(s.inv_zoom_x); w.u32(s.inv_zoom_y);
    for (uint32_t c : s.transfer_scale) w.u32(c);
    for (uint32_t c : s.transfer_bias) w.u32(c);
    w.u32(s.index_shift_left); w.u32(s.index_shift_right); w.i32(s.index_offset); w.u32(s.reserved_v4);
    return w.ok() && w.offset() == state_v4_bytes;
}

inline bool encode_command(const GraphicsCommand& c, uint8_t* output, size_t capacity) noexcept {
    Writer w(output, capacity, 0);
    w.u32(c.state_index); w.u32(c.primitive); w.u32(c.first_vertex); w.u32(c.vertex_count);
    w.u32(c.flags); w.u32(c.reserved);
    return w.ok() && w.offset() == command_bytes;
}

inline bool encode_present_input(const PresentInput& p, uint8_t* output, size_t capacity) noexcept {
    Writer w(output, capacity, 0);
    w.u32(p.size); w.u32(p.version); w.u64(p.session); w.u64(p.buffer); w.u32(p.width); w.u32(p.height);
    return w.ok() && w.offset() == present_input_bytes;
}

// radeondisp side: input bytes from DrawEscape plus the GDI clip.
inline bool encode_present_request(const PresentRequest& p, uint8_t* output, size_t capacity) noexcept {
    if (!encode_present_input(p.input, output, capacity)) return false;
    Writer w(output, capacity, present_input_bytes);
    w.i32(p.window_left); w.i32(p.window_top); w.u32(p.rect_count); w.u32(p.reserved);
    for (const auto& rect : p.rects) {
        w.i32(rect.left); w.i32(rect.top); w.i32(rect.right); w.i32(rect.bottom);
    }
    return w.ok() && w.offset() == present_request_bytes;
}

inline bool decode_header(ByteView input, SubmitHeader& h) noexcept {
    h = {};
    Reader r(input, 0);
    r.u32(h.header_bytes); r.u32(h.state_count); r.u32(h.command_count); r.u32(h.vertex_bytes);
    r.u64(h.color); r.u64(h.depth); r.u32(h.width); r.u32(h.height); r.u32(h.target_kind);
    r.u32(h.reserved);
    if (r.ok() && h.header_bytes == submit_header_v4_bytes) {
        r.u64(h.color2); r.u32(h.layout); r.u32(h.reserved2);
        if (!r.ok() || h.layout != submit_layout_v4 || h.reserved2 != 0 ||
            (h.color2 != 0 && (h.color2 == h.color || h.color2 == h.depth ||
                               h.target_kind != static_cast<uint32_t>(TargetKind::bgra8)))) {
            return false;
        }
    } else if (h.header_bytes != submit_header_bytes) {
        return false;
    }
    return r.ok() && h.state_count >= 1 &&
           h.state_count <= max_states && h.command_count >= 1 && h.command_count <= max_commands &&
           (h.vertex_bytes & 15u) == 0 && h.vertex_bytes <= max_vertex_bytes && h.color != 0 &&
           h.width >= 1 && h.width <= max_dimension && h.height >= 1 && h.height <= max_dimension &&
           h.target_kind <= static_cast<uint32_t>(TargetKind::depth_copy) && h.reserved == 0 &&
           (h.target_kind == static_cast<uint32_t>(TargetKind::bgra8) || h.depth != 0);
}

constexpr uint32_t submit_state_bytes(const SubmitHeader& h) noexcept {
    return h.header_bytes == submit_header_v4_bytes ? state_v4_bytes : state_bytes;
}

constexpr uint32_t submit_payload_bytes(const SubmitHeader& h) noexcept {
    return h.header_bytes + h.state_count * submit_state_bytes(h) + h.command_count * command_bytes +
           h.vertex_bytes;
}

// The state decoder runs in kernel frames: its rule checks stay out of line.
#if defined(_MSC_VER)
#define RX7900_WIRE_NOINLINE __declspec(noinline)
#else
#define RX7900_WIRE_NOINLINE __attribute__((noinline))
#endif

namespace detail {

// Layout-4 extension rules: unused features carry zero, used ones are bounded.
inline RX7900_WIRE_NOINLINE bool extension_valid(const GraphicsState& s) noexcept {
    namespace abi = rx7900::opengl::gfx11_abi;
    const auto& k = s.key;
    bool ok = k.line_stipple != 0 ? s.line_stipple_factor >= 1 && s.line_stipple_factor <= 256 &&
                                        s.line_stipple_pattern <= 0xffffu
                                  : s.line_stipple_factor == 0 && s.line_stipple_pattern == 0;
    for (uint32_t row : s.polygon_stipple) {
        ok = ok && (k.polygon_stipple != 0 || row == 0);
    }
    const bool textured = s.texture != 0;
    const bool mipmap = k.min_filter >= abi::texture_min_nearest_mipmap_nearest;
    ok = ok && (textured ? (mipmap ? s.level_count >= 1 && s.level_count <= max_texture_levels : s.level_count == 1)
                         : s.level_count == 0);
    for (uint32_t i = 0; i < 10; ++i) {
        const auto& level = s.levels[i];
        if (i + 1 < s.level_count) {
            ok = ok && (level.offset & 255u) == 0 && level.width >= 1 && level.width <= max_dimension &&
                 level.height >= 1 && level.height <= max_dimension && (level.pitch & 63u) == 0 &&
                 level.pitch >= level.width && level.pitch <= max_dimension;
        } else {
            ok = ok && level.offset == 0 && level.width == 0 && level.height == 0 && level.pitch == 0;
        }
    }
    if (k.pixel_source != abi::pixel_source_none) {
        ok = ok && s.pixel_buffer != 0 && s.pixel_columns >= 1 && s.pixel_columns <= max_pixel_columns &&
             s.pixel_first_row < s.pixel_end_row && s.pixel_end_row <= max_pixel_rows &&
             finite_bits(s.raster_x) && finite_bits(s.raster_y) && finite_bits(s.inv_zoom_x) &&
             finite_bits(s.inv_zoom_y);
    } else {
        ok = ok && s.pixel_buffer == 0 && s.pixel_offset == 0 && s.pixel_row_stride == 0 && s.pixel_columns == 0 &&
             s.pixel_first_row == 0 && s.pixel_end_row == 0 && s.raster_x == 0 && s.raster_y == 0 &&
             s.inv_zoom_x == 0 && s.inv_zoom_y == 0;
    }
    for (uint32_t c = 0; c < 4; ++c) {
        ok = ok && (k.pixel_transfer != 0 ? finite_bits(s.transfer_scale[c]) && finite_bits(s.transfer_bias[c])
                                          : s.transfer_scale[c] == 0 && s.transfer_bias[c] == 0);
    }
    ok = ok && (k.pixel_transfer != 0 ? s.index_shift_left <= 31 && s.index_shift_right <= 31
                                      : s.index_shift_left == 0 && s.index_shift_right == 0 && s.index_offset == 0);
    return ok && s.reserved_v4 == 0;
}

// Layout-3 field rules (both layouts).
inline RX7900_WIRE_NOINLINE bool state_fields_valid(const GraphicsState& s) noexcept {
    const auto& k = s.key;
    const auto flag = [](uint32_t value) { return value <= 1; };
    const auto colors = [](const uint32_t* values, uint32_t count) {
        for (uint32_t i = 0; i < count; ++i) {
            if (!unit_bits(values[i])) return false;
        }
        return true;
    };
    const bool textured = s.texture != 0;
    return s.viewport_width <= 2 * max_dimension && s.viewport_height <= 2 * max_dimension &&
           s.viewport_x >= -32768 && s.viewport_x <= 32767 && s.viewport_y >= -32768 &&
           s.viewport_y <= 32767 && unit_bits(s.depth_near) && unit_bits(s.depth_far) &&
           flag(s.scissor) && s.scissor_x >= -32768 && s.scissor_x <= 32767 &&
           s.scissor_y >= -32768 && s.scissor_y <= 32767 &&
           s.scissor_width <= 2 * max_dimension && s.scissor_height <= 2 * max_dimension &&
           s.cull <= static_cast<uint32_t>(Cull::front_and_back) && flag(s.front_ccw) &&
           size_bits(s.point_size) && size_bits(s.line_width) && flag(s.offset) &&
           finite_bits(s.offset_factor) && finite_bits(s.offset_units) &&
           flag(s.depth_test) && s.depth_func <= 7 && flag(s.depth_write) &&
           flag(s.stencil_test) && s.stencil_func <= 7 && s.stencil_ref <= 255 &&
           s.stencil_value_mask <= 255 && s.stencil_write_mask <= 255 &&
           s.stencil_fail <= 5 && s.stencil_depth_fail <= 5 && s.stencil_depth_pass <= 5 &&
           flag(s.blend) && s.blend_src <= 10 && s.blend_dst <= 10 && flag(s.logic_op) &&
           s.logic_op_code <= 15 && s.color_mask <= 15 && unit_bits(s.alpha_ref) &&
           colors(s.env_color, 4) && colors(s.fog_color, 3) &&
           rx7900::opengl::gfx11_abi::key_valid(k) &&
           k.vertex_input == rx7900::opengl::gfx11_abi::vertex_input_clip_space &&
           (k.textured != 0) == textured &&
           (textured ? (s.texture_offset & 255u) == 0 && s.texture_width >= 1 &&
                       s.texture_width <= max_dimension && s.texture_height >= 1 &&
                       s.texture_height <= max_dimension && (s.texture_pitch & 63u) == 0 &&
                       s.texture_pitch >= s.texture_width && s.texture_pitch <= max_dimension &&
                       s.wrap_s <= 1 && s.wrap_t <= 1 && s.filter <= 1
                     : s.texture_offset == 0 && s.texture_width == 0 && s.texture_height == 0 &&
                       s.texture_pitch == 0 && s.wrap_s == 0 && s.wrap_t == 0 && s.filter == 0) &&
           true;
}

// Both state layouts: layout 3 (v4 == false) reads the 20-byte v3 key and
// leaves every layout-4 field zero; layout 4 reads the 28-byte key and the
// extension.
inline RX7900_WIRE_NOINLINE bool decode_state_layout(ByteView input, size_t offset, GraphicsState& s, bool v4) noexcept {
    // Every field is assigned below (no state-sized temporary in a kernel frame).
    Reader r(input, offset);
    r.i32(s.viewport_x); r.i32(s.viewport_y); r.u32(s.viewport_width); r.u32(s.viewport_height);
    r.u32(s.depth_near); r.u32(s.depth_far); r.u32(s.scissor); r.i32(s.scissor_x); r.i32(s.scissor_y);
    r.u32(s.scissor_width); r.u32(s.scissor_height); r.u32(s.cull); r.u32(s.front_ccw);
    r.u32(s.point_size); r.u32(s.line_width); r.u32(s.offset); r.u32(s.offset_factor);
    r.u32(s.offset_units); r.u32(s.depth_test); r.u32(s.depth_func); r.u32(s.depth_write);
    r.u32(s.stencil_test); r.u32(s.stencil_func); r.u32(s.stencil_ref); r.u32(s.stencil_value_mask);
    r.u32(s.stencil_write_mask); r.u32(s.stencil_fail); r.u32(s.stencil_depth_fail);
    r.u32(s.stencil_depth_pass); r.u32(s.blend); r.u32(s.blend_src); r.u32(s.blend_dst);
    r.u32(s.logic_op); r.u32(s.logic_op_code); r.u32(s.color_mask); r.u32(s.alpha_ref);
    for (auto& c : s.env_color) r.u32(c);
    for (auto& c : s.fog_color) r.u32(c);
    r.u64(s.texture); r.u32(s.texture_offset); r.u32(s.texture_width); r.u32(s.texture_height);
    r.u32(s.texture_pitch); r.u32(s.wrap_s); r.u32(s.wrap_t); r.u32(s.filter);
    if (!r.ok() || r.offset() != offset + state_key_offset) return false;
    auto& k = s.key;
    r.u8(k.vertex_input); r.u8(k.smooth); r.u8(k.two_sided); r.u8(k.lighting); r.u8(k.light_mask);
    r.u8(k.positional_mask); r.u8(k.spot_mask); r.u8(k.normalize); r.u8(k.color_material);
    r.u8(k.fog); r.u8(k.fog_mode); r.u8(k.textured); r.u8(k.tex_env); r.u8(k.texture_base);
    r.u8(k.alpha_test); r.u8(k.alpha_func); r.u8(k.vertices_per_primitive);
    if (v4) {
        r.u8(k.min_filter); r.u8(k.mag_filter); r.u8(k.line_stipple); r.u8(k.polygon_stipple);
        r.u8(k.front_and_back); r.u8(k.pixel_source); r.u8(k.pixel_transfer);
        for (auto& b : k.reserved) r.u8(b);
    } else {
        r.u8(k.reserved[0]); r.u8(k.reserved[1]); r.u8(k.reserved[2]);
        k.reserved[3] = 0;
        k.min_filter = k.mag_filter = k.line_stipple = k.polygon_stipple = 0;
        k.front_and_back = k.pixel_source = k.pixel_transfer = 0;
    }
    if (!r.ok() || r.offset() != offset + (v4 ? state_v4_extension_offset : state_bytes)) return false;
    if (v4) {
        r.u32(s.line_stipple_factor); r.u32(s.line_stipple_pattern);
        for (auto& row : s.polygon_stipple) r.u32(row);
        r.u32(s.level_count);
        for (auto& level : s.levels) { r.u32(level.offset); r.u32(level.width); r.u32(level.height); r.u32(level.pitch); }
        r.u64(s.pixel_buffer); r.u32(s.pixel_offset); r.i32(s.pixel_row_stride); r.u32(s.pixel_columns);
        r.u32(s.pixel_first_row); r.u32(s.pixel_end_row); r.u32(s.raster_x); r.u32(s.raster_y);
        r.u32(s.inv_zoom_x); r.u32(s.inv_zoom_y);
        for (auto& c : s.transfer_scale) r.u32(c);
        for (auto& c : s.transfer_bias) r.u32(c);
        r.u32(s.index_shift_left); r.u32(s.index_shift_right); r.i32(s.index_offset); r.u32(s.reserved_v4);
        if (!r.ok() || r.offset() != offset + state_v4_bytes) return false;
    } else {
        // Layout 3 has no extension: every layout-4 field is zero.
        s.line_stipple_factor = s.line_stipple_pattern = 0;
        for (auto& row : s.polygon_stipple) row = 0;
        for (auto& level : s.levels) level = {0, 0, 0, 0};
        s.pixel_buffer = 0;
        s.pixel_offset = s.pixel_columns = s.pixel_first_row = s.pixel_end_row = 0;
        s.pixel_row_stride = 0;
        s.raster_x = s.raster_y = s.inv_zoom_x = s.inv_zoom_y = 0;
        for (uint32_t c = 0; c < 4; ++c) s.transfer_scale[c] = s.transfer_bias[c] = 0;
        s.index_shift_left = s.index_shift_right = 0;
        s.index_offset = 0;
        s.reserved_v4 = 0;
        s.level_count = s.texture != 0 ? 1u : 0u;
    }

    return state_fields_valid(s) && (!v4 || extension_valid(s));
}

} // namespace detail

// Layout-3 state (the v3 wire key).
inline bool decode_state(ByteView input, size_t offset, GraphicsState& s) noexcept {
    return detail::decode_state_layout(input, offset, s, false);
}

// Layout-4 state.
inline bool decode_state_v4(ByteView input, size_t offset, GraphicsState& s) noexcept {
    return detail::decode_state_layout(input, offset, s, true);
}

inline bool decode_command(ByteView input, size_t offset, uint32_t state_count,
                           uint32_t vertex_bytes, GraphicsCommand& c) noexcept {
    c = {};
    Reader r(input, offset);
    r.u32(c.state_index); r.u32(c.primitive); r.u32(c.first_vertex); r.u32(c.vertex_count);
    r.u32(c.flags); r.u32(c.reserved);
    const uint64_t stride = rx7900::opengl::gfx11_abi::clip_record_stride;
    return r.ok() && c.state_index < state_count && c.primitive >= 1 && c.primitive <= 3 &&
           c.vertex_count >= c.primitive && c.vertex_count % c.primitive == 0 &&
           c.flags <= command_depth_copy && c.reserved == 0 &&
           (static_cast<uint64_t>(c.first_vertex) + c.vertex_count) * stride <= vertex_bytes &&
           ((c.flags & command_depth_copy) == 0 || c.primitive == 3);
}

inline bool decode_present_input(ByteView input, PresentInput& p) noexcept {
    p = {};
    Reader r(input, 0);
    r.u32(p.size); r.u32(p.version); r.u64(p.session); r.u64(p.buffer); r.u32(p.width);
    r.u32(p.height);
    return r.ok() && p.size == present_input_bytes && p.version == version && p.session != 0 &&
           p.buffer != 0 && p.width >= 1 && p.width <= max_dimension && p.height >= 1 &&
           p.height <= max_dimension;
}

inline bool decode_present_request(ByteView input, PresentRequest& p) noexcept {
    p = {};
    ByteView head(nullptr, 0);
    if (input.size() != present_request_bytes || !input.subview(0, present_input_bytes, head) ||
        !decode_present_input(head, p.input)) {
        return false;
    }
    Reader r(input, present_input_bytes);
    r.i32(p.window_left); r.i32(p.window_top); r.u32(p.rect_count); r.u32(p.reserved);
    for (auto& rect : p.rects) {
        r.i32(rect.left); r.i32(rect.top); r.i32(rect.right); r.i32(rect.bottom);
    }
    if (!r.ok() || p.rect_count > max_present_rects || p.reserved != 0) return false;
    for (uint32_t i = 0; i < p.rect_count; ++i) {
        const auto& rect = p.rects[i];
        if (rect.left >= rect.right || rect.top >= rect.bottom) return false;
    }
    return true;
}

} // namespace rx7900::reactos::graphics_abi

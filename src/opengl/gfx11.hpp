/*
Module: rx7900.opengl.gfx11
Purpose: Build bounded real GFX11 PM4 shader-address and triangle draw packets.
Created: 2026-10-21
Architecture: Trusted graphics command construction, never a hardware queue.
*/
#pragma once

#include "manager.hpp"

namespace rx7900::opengl::gfx11 {

enum class Status {
    success, invalid_stage, invalid_alignment, invalid_shader_size,
    invalid_shader_abi, invalid_draw, invalid_memory, output_too_small
};

// GFX11 has no legacy VS pipeline. A vertex-only NGG program runs through the
// ES program address and GS user-data bank; pixel uses the PS bank.
enum class Stage { primitive_ngg, pixel };

struct ShaderCode final {
    Stage stage;
    memory::Slice code;
    uint64_t padded_byte_count;
};

struct ShaderPointer final {
    Stage stage;
    memory::Slice data;
    uint64_t byte_count;
    uint32_t first_user_sgpr;
    uint32_t compiled_user_sgpr_count;
};

struct TriangleDraw final {
    uint32_t vertex_count;
    uint32_t instance_count;
};

constexpr size_t shader_address_bytes = 4u * sizeof(uint32_t);
constexpr size_t shader_pointer_bytes = 4u * sizeof(uint32_t);
constexpr size_t triangle_draw_bytes = 11u * sizeof(uint32_t);

// These are internal trusted-driver interfaces, never raw-packet IOCTLs.
// Manager resolutions prove live owned ranges/read permissions and bounds.
// The queue owner serializes manager access and retains every referenced handle
// before encoding, through confirmed GFX completion or DMA quiescence. The same
// owner keeps command output exclusive and nonoverlapping with input/state.
// Encoding does not retain buffers, submit work, wait for completion or validate
// shader ISA. Inputs are copied before output; all failures leave bytes unchanged
// and bytes_written zero. output_too_small reports the exact required byte count.
// ShaderCode must contain externally compiled NGG/PS instructions, 256-byte
// aligned/padded with executable GPU mappings and enough safe prefetch storage.
Status encode_shader_address(const memory::Manager& memory, const ShaderCode& shader,
                             sdma::CommandBuffer output, sdma::EncodeResult& result) noexcept;

// Writes one raw 64-bit GPU pointer to an explicitly compiled pair of user SGPRs.
// The compiler ABI must assign this exact role to that pair; this does not build
// AMD vertex/resource descriptors, set resource counts or infer a shader ABI.
Status encode_shader_pointer(const memory::Manager& memory, const ShaderPointer& pointer,
                             sdma::CommandBuffer output, sdma::EncodeResult& result) noexcept;

// Selects triangle list, disables primitive restart, emits NUM_INSTANCES and
// DRAW_INDEX_AUTO with automatic indices and one end-of-packet draw. Requires
// start vertex/instance zero and a matching preinstalled compiler/user-SGPR ABI.
// This is only the draw tail: validated NGG/PS resource registers, GE/NGG sizing,
// stage enables, vertex data/descriptor uploads, color/depth targets, interpolation,
// viewport/scissor/raster state, cache visibility and required chip workarounds
// must already be programmed by the graphics pipeline owner. No completeness or
// hardware readiness assertion is made from successful packet construction.
Status encode_triangle_draw(const TriangleDraw& draw, sdma::CommandBuffer output,
                            sdma::EncodeResult& result) noexcept;

struct TriangleCommands final {
    ShaderCode primitive_shader;
    ShaderCode pixel_shader;
    ShaderPointer vertex_pointer;
    TriangleDraw draw;
    uint32_t vertex_stride_bytes;
};

constexpr size_t triangle_commands_bytes = 2u * shader_address_bytes +
                                          shader_pointer_bytes + triangle_draw_bytes;

// Transactional address/pointer/draw combination for an external raw-pointer
// vertex shader ABI. Validates the entire vertex_count times stride footprint.
// It is still a partial graphics command sequence with all prerequisites above.
Status encode_triangle_commands(const memory::Manager& memory,
                                const TriangleCommands& commands,
                                sdma::CommandBuffer output,
                                sdma::EncodeResult& result) noexcept;

} // namespace rx7900::opengl::gfx11

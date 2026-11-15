/*
Module: rx7900.opengl.gl_shader_gfx11
Purpose: Generate the GFX11 NGG vertex (primitive) shader and the pixel shader that
implement the OpenGL 1.1 fixed-function pipeline for one gfx11_abi::ShaderKey.
Created: 2026-10-23
Architecture: Trusted shader generation, freestanding C++17 (no heap, no
exceptions or RTTI, no floating-point library) so the kernel can generate from a
validated key. Every instruction goes through hardware::gfx11_isa::Assembler, whose
encodings are byte-exact against the pinned LLVM 18.1.8 GFX11 MC vectors
(tests/gfx11_isa_tests.cpp). The register, descriptor and memory interface is
gl_gfx11_abi.hpp.

Vertex program (merged ES+GS, NGG, wave32, one subgroup per wave), after Mesa
24.2.0 ac_nir_lower_ngg.c non-culling NGG: GS_ALLOC_REQ from m0 =
vertices | primitives << 12 taken from merged_wave_info; the primitive lanes export
the packed 10-bit vertex indices (exp prim done); the vertex lanes fetch their
record (buffer_load_b128), either pass the frontend's clip-space attributes through
or perform the gl_context transform, lighting, texture matrix and fog in the same
operation order as gl_context (separate multiply and add roundings), store every
attribute slot to the GFX11 attribute ring (buffer_store_b128 idxen, swizzled,
index = lane, soffset = gs_attr_offset << 9), wait for those stores (s_waitcnt_vscnt)
and only then export pos0 with done.

Pixel program: the live mask is saved, the wave enters whole-quad mode, m0 takes
the primitive mask, each attribute channel is lds_param_load + v_interp_p10_f32 +
v_interp_p2_f32 at the perspective centre (flat slots interpolate with zero
coordinates, giving P0, the provoking value), two-sided colour selects on
front_face, the texture is sampled with image_sample_lz at (s/q, t/q), the
texture environment and fog follow gl_raster shade_fragment, and the alpha test
(v_cmp_*_f32 against the alpha reference) narrows EXEC to the passing live pixels
before exp mrt0 done; a wave with no surviving pixel exports MRT0 with no channels
(GFX11 has no null export target). Blending and depth/stencil are CB/DB state.

Verified only in the independent interpreter (tests/gfx11_interp.hpp) against the
gl_raster pixel oracle; not run on hardware. User or kernel mode; no allocation.
*/
#pragma once

#include "../hardware/gfx11_isa.hpp"
#include "gl_gfx11_abi.hpp"
#include "gl_pipeline.hpp"

namespace rx7900::opengl::shader {

enum class Status : uint8_t { success, invalid_argument, output_too_small, encode_failed };

// Caller-owned generator state (the assembler with its label and fixup tables), so
// that no generator frame exceeds 512 bytes on native i686. The kernel keeps one in
// device-owned storage; it is reused across calls, one call at a time (calls on the
// same workspace must be serialized). Construct it once; it needs no destruction.
struct GeneratorWorkspace final {
    hardware::gfx11_isa::Assembler assembler{rx7900::sdma::CommandBuffer(nullptr, 0)};
};

constexpr size_t generator_workspace_bytes = sizeof(GeneratorWorkspace);

// Generate the NGG vertex program for key into output (little-endian dwords,
// padded per the gfx11_abi binary layout) using workspace. info is overwritten in
// every case and describes the program on success. invalid_argument: key_valid(key)
// is false. output_too_small: output cannot hold the program (shader_max_bytes
// always does). The output depends only on key.
Status generate_vertex_shader(const gfx11_abi::ShaderKey& key, rx7900::sdma::CommandBuffer output,
                              gfx11_abi::ProgramInfo& info, GeneratorWorkspace& workspace) noexcept;

// Generate the pixel program for key; same contract as generate_vertex_shader.
Status generate_pixel_shader(const gfx11_abi::ShaderKey& key, rx7900::sdma::CommandBuffer output,
                             gfx11_abi::ProgramInfo& info, GeneratorWorkspace& workspace) noexcept;

// Host and user-mode conveniences with a workspace of generator_workspace_bytes on
// the stack. Kernel callers use the explicit-workspace forms above.
inline Status generate_vertex_shader(const gfx11_abi::ShaderKey& key, rx7900::sdma::CommandBuffer output,
                                     gfx11_abi::ProgramInfo& info) noexcept {
    GeneratorWorkspace workspace;
    return generate_vertex_shader(key, output, info, workspace);
}

inline Status generate_pixel_shader(const gfx11_abi::ShaderKey& key, rx7900::sdma::CommandBuffer output,
                                    gfx11_abi::ProgramInfo& info) noexcept {
    GeneratorWorkspace workspace;
    return generate_pixel_shader(key, output, info, workspace);
}

// The clip-space key for one backend draw: the frontend has already transformed,
// lit, fogged and flat-resolved every RasterVertex, so the key only carries the
// primitive kind and the per-fragment stages (texture environment, fog blend,
// alpha test). two_sided follows DrawState::two_sided_color: back-facing
// primitives then take RasterVertex color[1] (record offset 32), as gl_raster does.
// v4 fields: the LOD filters when the texture needs them, line stipple for
// lines, polygon stipple for triangles and GL_FRONT_AND_BACK; the backend sets
// the pixel-rectangle fields itself.
gfx11_abi::ShaderKey clip_space_key(const DrawState& state, PrimitiveKind kind) noexcept;

} // namespace rx7900::opengl::shader

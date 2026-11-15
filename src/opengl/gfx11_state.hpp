/*
Module: rx7900.opengl.gfx11_state
Purpose: Translate validated OpenGL 1.1 pipeline state into GFX11 context, SH and
UCONFIG register writes and draw packets, through a bounded PM4 builder that
refuses every register outside the gfx11_registers allow-list.
Created: 2026-10-23
Architecture: Trusted kernel-side IB construction (process_graphics) and host
tests. Inputs are already-resolved GPU virtual addresses of driver-validated
buffers and range-checked GL enumerants; user mode never supplies a register,
a register value, an address or a packet. The shader contract (user SGPR slots,
descriptors, export formats, NGG limits) comes from gl_gfx11_abi.hpp, owned by
gl-shaders. Register programming follows Mesa 24.2.0 radeonsi for GFX11 (MIT;
pinned with SHA-256 in work/upstream-gpuinit/SHA256SUMS):
  si_state.c            e4c3f939... gfx10_init_gfx_preamble_state (GFX11 branch),
                        blend/DSA/rasterizer creation, db render state, MSAA config
  si_state_shaders.cpp  358e8e12... gfx10_shader_ngg and si_shader_ps (GFX11)
  si_state_viewport.c   fcc19c63... guard band, quant mode, scissor rules
  si_state_binning.c    d16f67ba... si_emit_dpbb_disable (GFX10-11 branch)
  ac_descriptors.c      f3137976... ac_init_gfx9_ds_surface, ac_init_cb_surface
  si_blit.c / u_blitter.c 61f50b6e.../155cb06c... DB->CB depth/stencil copy
  pkt3.json             RELEASE_MEM/ACQUIRE_MEM PWS and DMA_DATA word layouts
Field placement uses gfx11_registers (AMD gc_11_0_0 headers); enumerant values
are the gfx11.json values and are re-checked by tests. No allocation, no OS
calls, no floating-point library calls; freestanding C++17 for the kernel.
Hardware execution of any emitted stream is unverified.
*/
#pragma once

#include "gfx11_registers.hpp"
#include "gl_gfx11_abi.hpp"
#include "sdma.hpp"

namespace rx7900::opengl::gfx11 {

enum class StateStatus : uint32_t {
    success,
    invalid_argument,
    output_too_small,
    disallowed_register,
    field_overflow
};

// GFX11 enumerant values (Mesa 24.2.0 gfx11.json / pkt3.json; test-verified).
namespace hw {
constexpr uint32_t color_invalid = 0;            // ColorFormat
constexpr uint32_t color_8_8_8_8 = 10;
constexpr uint32_t color_x24_8_32_float = 22;
constexpr uint32_t number_unorm = 0;             // SurfaceNumber
constexpr uint32_t number_float = 7;
constexpr uint32_t swap_std = 0;                 // SurfaceSwap
constexpr uint32_t swap_alt = 1;
constexpr uint32_t z_invalid = 0;                // ZFormat
constexpr uint32_t z_32_float = 3;
constexpr uint32_t stencil_invalid = 0;          // StencilFormat
constexpr uint32_t stencil_8 = 1;
constexpr uint32_t sw_linear = 0;                // swizzle mode
constexpr uint32_t sw_64kb_z_x = 24;
constexpr uint32_t stencil_keep = 0;             // StencilOp
constexpr uint32_t stencil_zero = 1;
constexpr uint32_t stencil_replace_test = 3;
constexpr uint32_t stencil_add_clamp = 5;
constexpr uint32_t stencil_sub_clamp = 6;
constexpr uint32_t stencil_invert = 7;
constexpr uint32_t blend_zero = 0;               // BlendOp
constexpr uint32_t blend_one = 1;
constexpr uint32_t blend_src_color = 2;
constexpr uint32_t blend_one_minus_src_color = 3;
constexpr uint32_t blend_src_alpha = 4;
constexpr uint32_t blend_one_minus_src_alpha = 5;
constexpr uint32_t blend_dst_alpha = 6;
constexpr uint32_t blend_one_minus_dst_alpha = 7;
constexpr uint32_t blend_dst_color = 8;
constexpr uint32_t blend_one_minus_dst_color = 9;
constexpr uint32_t blend_src_alpha_saturate = 10;
constexpr uint32_t comb_dst_plus_src = 0;        // CombFunc
constexpr uint32_t cb_disable = 0;               // CBMode
constexpr uint32_t cb_normal = 1;
constexpr uint32_t force_disable = 2;            // ForceControl
constexpr uint32_t late_z = 0;                   // ZOrder
constexpr uint32_t early_z_then_late_z = 1;
constexpr uint32_t sx_rt_export_no_conversion = 0;  // SX_DOWNCONVERT_FORMAT
constexpr uint32_t sx_rt_export_32_r = 1;
constexpr uint32_t sx_rt_export_8_8_8_8 = 5;
constexpr uint32_t opt_comb_none = 0;            // SX_OPT_COMB_FCN
constexpr uint32_t opt_comb_blend_disabled = 6;
constexpr uint32_t spi_shader_fp16_abgr = 4;     // SPI_SHADER_EX_FORMAT
constexpr uint32_t spi_shader_32_abgr = 9;
constexpr uint32_t di_pt_pointlist = 1;          // VGT_DI_PRIM_TYPE
constexpr uint32_t di_pt_linelist = 2;
constexpr uint32_t di_pt_trilist = 4;
constexpr uint32_t x_round_to_even = 2;          // PA_SU_VTX_CNTL__ROUND_MODE
constexpr uint32_t quant_16_8_1_256th = 5;       // QUANT_MODE
constexpr uint32_t quant_14_10_1_1024th = 6;
constexpr uint32_t quant_12_12_1_4096th = 7;
constexpr uint32_t binning_disable_new_sc = 2;   // BinningMode 2 (Mesa DISABLE_BINNING_USE_NEW_SC)
constexpr uint32_t bin_size_128_pixels = 2;      // BinSizeExtend
constexpr uint32_t cache_lru_wr = 0;             // WritePolicy
constexpr uint32_t cache_stream = 1;
constexpr uint32_t cache_noa_rd = 2;             // ReadPolicy CACHE_NOA
constexpr uint32_t vrs_comb_passthru = 0;        // VRSCombinerModeSC
constexpr uint32_t vrs_comb_override = 1;
constexpr uint32_t event_ps_partial_flush = 16;  // VGT_EVENT_TYPE
constexpr uint32_t event_bottom_of_pipe_ts = 40;
constexpr uint32_t event_flush_and_inv_db_meta = 44;
constexpr uint32_t event_flush_and_inv_cb_meta = 46;
constexpr uint32_t pws_stage_cp_me = 5;          // pkt3.json PWS_STAGE_SEL
constexpr uint32_t pws_counter_ts = 0;           // PWS_COUNTER_SEL
constexpr uint32_t dma_src_addr_tc_l2 = 3;       // CP_DMA_WORD1__SRC_SEL_cik
constexpr uint32_t dma_dst_addr_tc_l2 = 3;       // CP_DMA_WORD1__DST_SEL_gfx9
} // namespace hw

// PM4 type-3 opcodes (Mesa sid.h PKT3_*, Linux nvd.h).
namespace op {
constexpr uint32_t nop = 0x10;
constexpr uint32_t clear_state = 0x12;
constexpr uint32_t context_control = 0x28;
constexpr uint32_t draw_index_auto = 0x2d;
constexpr uint32_t num_instances = 0x2f;
constexpr uint32_t event_write = 0x46;
constexpr uint32_t release_mem = 0x49;
constexpr uint32_t dma_data = 0x50;
constexpr uint32_t acquire_mem = 0x58;
constexpr uint32_t set_context_reg = 0x69;
constexpr uint32_t set_sh_reg = 0x76;
constexpr uint32_t set_uconfig_reg = 0x79;
} // namespace op

constexpr uint32_t packet3(uint32_t opcode, uint32_t body_dwords) noexcept {
    return (3u << 30) | (((body_dwords - 1u) & 0x3fffu) << 16) | ((opcode & 0xffu) << 8);
}

// Bounded PM4 writer. Every register write goes through set(), which refuses a
// register outside gfx11_reg::allowed_registers or outside its PM4 space, and
// suppresses a write equal to the value already emitted in this IB (the IB
// starts from CLEAR_STATE, so the shadow starts unknown). Failures are sticky:
// after the first error nothing more is written and status() reports why.
// The object is large (register shadow); keep it in device-owned storage, not on
// a kernel stack.
class Pm4Builder final {
public:
    Pm4Builder() noexcept = default;
    Pm4Builder(const Pm4Builder&) = delete;
    Pm4Builder& operator=(const Pm4Builder&) = delete;

    // Starts a new IB over output (4-byte aligned writable storage owned by the
    // caller for the whole build). Clears the shadow.
    void begin(sdma::CommandBuffer output) noexcept;
    bool set(gfx11_reg::Register reg, uint32_t value) noexcept;
    // Raw packet header plus body. Only gfx11_state's own emitters call this.
    bool packet(uint32_t opcode, const uint32_t* body, uint32_t body_dwords) noexcept;
    // Pads with type-3 NOPs to a dword multiple of alignment (power of two).
    bool pad(uint32_t alignment_dwords) noexcept;
    // Marks a sticky failure (used by emitters for field overflow).
    void fail(StateStatus status) noexcept;
    uint32_t dwords() const noexcept { return dwords_; }
    StateStatus status() const noexcept { return status_; }
    bool ok() const noexcept { return status_ == StateStatus::success; }

private:
    static constexpr uint32_t window = 0x500;   // largest space span used (uconfig)
    struct Shadow final {
        uint32_t value[window];
        uint8_t known[window];
    };
    bool put(uint32_t word) noexcept;
    Shadow* shadow_for(gfx11_reg::Space space) noexcept;

    sdma::CommandBuffer output_{nullptr, 0};
    uint32_t dwords_ = 0;
    StateStatus status_ = StateStatus::invalid_argument;
    Shadow context_{};
    Shadow sh_{};
    Shadow uconfig_{};
};

// Builds one register value field by field; any value that does not fit its
// field marks the value invalid (the emitter then fails with field_overflow).
class RegisterValue final {
public:
    RegisterValue& with(gfx11_reg::Field field, uint32_t value) noexcept {
        if (!gfx11_reg::fits(field, value)) {
            valid_ = false;
        }
        word_ |= gfx11_reg::place(field, value);
        return *this;
    }
    uint32_t word() const noexcept { return word_; }
    bool valid() const noexcept { return valid_; }

private:
    uint32_t word_ = 0;
    bool valid_ = true;
};

// ---------------------------------------------------------------------------
// Semantic inputs (already validated by the kernel wire decoder).
// ---------------------------------------------------------------------------
enum class Compare : uint8_t { never, less, equal, lequal, greater, notequal, gequal, always };
enum class StencilOp : uint8_t { keep, zero, replace, increment, decrement, invert };
enum class BlendFactor : uint8_t {
    zero, one, src_color, one_minus_src_color, dst_color, one_minus_dst_color,
    src_alpha, one_minus_src_alpha, dst_alpha, one_minus_dst_alpha, src_alpha_saturate
};
enum class CullMode : uint8_t { none, front, back, front_and_back };
// Value = vertices per primitive (gl_gfx11_abi ShaderKey.vertices_per_primitive).
enum class Primitive : uint8_t { points = 1, lines = 2, triangles = 3 };
// bgra8: the window colour buffer (B, G, R, A bytes, linear, rows top to bottom).
// depth_copy: X24_8_32_FLOAT linear target of a DB->CB copy (float depth then a
// dword whose low byte is stencil), used for glReadPixels of depth/stencil.
enum class TargetKind : uint8_t { bgra8, depth_copy };

constexpr uint32_t max_dimension = 4096;
constexpr uint32_t linear_pitch_alignment_bytes = 256;   // GFX10.3+ custom linear pitch
constexpr uint32_t surface_base_alignment = 256;
constexpr uint32_t depth_block_bytes = 65536;            // SW_64KB_Z_X

constexpr uint32_t target_bytes_per_pixel(TargetKind kind) noexcept {
    return kind == TargetKind::bgra8 ? 4u : 8u;
}

// Linear pitch in pixels: width rounded up to 256 bytes.
constexpr uint32_t linear_pitch_pixels(uint32_t width, TargetKind kind) noexcept {
    const uint32_t per_row = linear_pitch_alignment_bytes / target_bytes_per_pixel(kind);
    return (width + per_row - 1u) / per_row * per_row;
}

// SW_64KB_Z_X layout of one Z32_FLOAT plane followed by one 8-bit stencil plane:
// a 64 KiB block is 128x128 32-bit or 256x256 8-bit elements; each plane is a
// whole number of blocks and the stencil plane starts on a block boundary.
struct DepthLayout final {
    uint64_t z_bytes = 0;
    uint64_t stencil_offset = 0;
    uint64_t total_bytes = 0;
};
bool depth_layout(uint32_t width, uint32_t height, DepthLayout& layout) noexcept;

struct ColorTarget final {
    uint64_t va = 0;               // 256-byte aligned
    uint32_t pitch_pixels = 0;     // linear_pitch_pixels(width, kind)
    TargetKind kind = TargetKind::bgra8;
};

struct DepthTarget final {
    bool present = false;
    uint64_t z_va = 0;             // 64 KiB aligned
    uint64_t stencil_va = 0;       // z_va + DepthLayout.stencil_offset
};

struct Framebuffer final {
    uint32_t width = 0;
    uint32_t height = 0;
    ColorTarget color{};
    // GL_FRONT_AND_BACK (ShaderKey front_and_back): the second colour buffer as
    // MRT1, same size and kind as color, blended against its own contents.
    bool dual = false;
    ColorTarget color1{};
    DepthTarget depth{};
};

// GL window-coordinate state (y up from the bottom of the drawable). The
// emitters convert to the hardware's top-down rows.
struct RasterState final {
    int32_t viewport_x = 0, viewport_y = 0;
    uint32_t viewport_width = 0, viewport_height = 0;
    float depth_near = 0, depth_far = 1;
    bool scissor = false;
    int32_t scissor_x = 0, scissor_y = 0;
    uint32_t scissor_width = 0, scissor_height = 0;
    CullMode cull = CullMode::none;
    bool front_ccw = true;
    float point_size = 1, line_width = 1;
    bool offset = false;
    float offset_factor = 0, offset_units = 0;
    bool depth_test = false;
    Compare depth_func = Compare::less;
    bool depth_write = true;
    bool stencil_test = false;
    Compare stencil_func = Compare::always;
    uint32_t stencil_ref = 0, stencil_value_mask = 0xff, stencil_write_mask = 0xff;
    StencilOp stencil_fail = StencilOp::keep;
    StencilOp stencil_depth_fail = StencilOp::keep;
    StencilOp stencil_depth_pass = StencilOp::keep;
    bool blend = false;
    BlendFactor blend_src = BlendFactor::one, blend_dst = BlendFactor::zero;
    bool logic_op = false;
    uint32_t logic_op_code = 3;    // GL_CLEAR..GL_SET minus 0x1500; 3 = GL_COPY
    uint32_t color_mask = 0xf;     // bit 0 R, 1 G, 2 B, 3 A; 0 disables colour writes
    float alpha_ref = 0;           // clamped to [0, 1] by the decoder
    float env_color[4] = {0, 0, 0, 0};
    float fog_color[3] = {0, 0, 0};
};

// Generated programs for one ShaderKey, resident at kernel-owned VAs.
struct ShaderPrograms final {
    gfx11_abi::ShaderKey key{};
    uint64_t vs_va = 0;            // 256-byte aligned
    uint64_t ps_va = 0;
    gfx11_abi::ProgramInfo vs{};
    gfx11_abi::ProgramInfo ps{};
};

// The kernel-owned attribute ring (one per device), mapped at the same VA in
// every process VM. per_se_bytes follows ac_gpu_info.c for chips with L3.
struct DeviceRings final {
    uint64_t attribute_ring_va = 0;      // 64 KiB aligned
    uint32_t attribute_ring_per_se_bytes = 0;
    uint32_t shader_engines = 0;
};
constexpr uint32_t navi31_shader_engines = 6;
constexpr uint32_t attribute_ring_per_se_bytes_l3 = (1400u * 1024u + 65535u) / 65536u * 65536u;
constexpr uint32_t attribute_ring_bytes(const DeviceRings& rings) noexcept {
    return rings.attribute_ring_per_se_bytes * rings.shader_engines;
}

struct DrawInputs final {
    Primitive primitive = Primitive::triangles;
    uint32_t vertex_count = 0;           // multiple of the vertices per primitive
    uint64_t vertex_va = 0;              // record 0 of this draw, 16-byte aligned
    uint32_t vertex_bytes = 0;           // >= vertex_count * record stride
    gfx11_abi::ImageDescriptor texture{};
    gfx11_abi::SamplerDescriptor sampler{};
    bool depth_copy = false;             // DB->CB copy pass (no colour/depth state)
    // v4 keys (gfx11_abi uses_ps_constants): the kernel-built PS constant block.
    uint64_t ps_constants_va = 0;        // 16-byte aligned
    uint32_t ps_constants_bytes = 0;     // gfx11_abi::ps_constants_bytes
};

// One rectangle copy between two linear 32-bit surfaces (present). Coordinates
// are top-down pixels; each row becomes one CP DMA_DATA through L2.
struct RowCopy final {
    uint64_t source_va = 0;
    uint32_t source_pitch_bytes = 0;
    uint32_t source_x = 0, source_y = 0;
    uint64_t destination_va = 0;
    uint32_t destination_pitch_bytes = 0;
    uint32_t destination_x = 0, destination_y = 0;
    uint32_t width = 0, height = 0;
};
constexpr uint32_t dma_data_dwords = 7;
constexpr uint32_t dma_max_bytes = (1u << 26) - 1u;

// CONTEXT_CONTROL + CLEAR_STATE, the GFX11 graphics preamble registers and the
// attribute ring (with Mesa's bottom-of-pipe PWS wait before it changes).
StateStatus emit_preamble(Pm4Builder& builder, const DeviceRings& rings) noexcept;
// Colour and depth/stencil surfaces, window/screen scissors.
StateStatus emit_framebuffer(Pm4Builder& builder, const Framebuffer& framebuffer) noexcept;
// All per-draw state, shader registers, user data and the DRAW_INDEX_AUTO.
StateStatus emit_draw(Pm4Builder& builder, const Framebuffer& framebuffer,
                      const RasterState& state, const ShaderPrograms& programs,
                      const DrawInputs& draw, const DeviceRings& rings) noexcept;
// CB/DB metadata flush and PS partial flush at the end of an IB; the queue
// frame's RELEASE_MEM CACHE_FLUSH_AND_INV_TS flushes CB/DB data and L2.
StateStatus emit_end_of_ib(Pm4Builder& builder) noexcept;
// CP DMA_DATA rows for a present rectangle (no register writes).
StateStatus emit_row_copy(Pm4Builder& builder, const RowCopy& copy) noexcept;

// Field helpers exposed for tests.
uint32_t pack_12p4(float value) noexcept;
uint32_t float_bits(float value) noexcept;
uint32_t logic_op_rop3(uint32_t gl_logic_op_code) noexcept;

} // namespace rx7900::opengl::gfx11

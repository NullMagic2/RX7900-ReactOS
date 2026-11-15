/*
Module: rx7900.opengl.gl_gfx11_backend
Purpose: GFX11 hardware Backend for the portable OpenGL 1.1 frontend: batches
the frontend's clip-space primitives into typed graphics submissions for the
native process service, owns the drawable's colour/depth/texture/image
buffers as opaque process-buffer handles, and falls back to the software
reference backend (reporting why) only when the GPU path is unavailable, an
allocation or submission fails, or a texture exceeds the GPU limits.
Created: 2026-10-23
Architecture: User mode (radeonglicd). Plugs into Context::set_backend. It never
sees a GPU address, register or packet: requests carry handles, GL enumerants,
float bits, the gl_gfx11_abi ShaderKey (shader::clip_space_key) and 80-byte
clip-space vertex records; the kernel (process_graphics) validates, generates
shaders, builds the IB and leases buffers. Submission layout 4 carries the v4
features (2026-10-24): line stipple (per-segment counter positions in the
vertex records, PS discard), polygon stipple (PS discard), GL_FRONT_AND_BACK
(MRT0 front + MRT1 back), DrawPixels/Bitmap/CopyPixels (a screen rectangle
whose PS fetches the uploaded image or a GPU colour/depth-copy surface) and
mipmapped sampling (every level uploaded, LOD in the PS); plain draws keep
layout 3. A GpuTransport delivers the bytes:
the ICD uses ExtEscape/DrawEscape through radeondisp, tests use the production
kernel modules over a synthetic device. Colour buffers are linear BGRA8 with rows
top to bottom (scanout order) and a 256-byte pitch; depth/stencil is one
SW_64KB_Z_X Z32_FLOAT + S8 buffer. Buffers live in GTT (domain system): the
shared VRAM pool is small and GTT keeps CPU readback exact. Surfaces up to the
4096 x 4096 drawable limit (graphics_abi buffer_max_bytes 128 MiB per buffer);
transfers are chunked to the bulk size. Single-threaded per context, like
every Backend.
*/
#pragma once

#include "gl_pipeline.hpp"
#include "gl_raster.hpp"
#include "gfx11_state.hpp"
#include "../reactos/gpu_process_abi.hpp"
#include "../reactos/process_graphics_abi.hpp"

namespace rx7900::opengl {

// Byte transport to the native process service.
class GpuTransport {
public:
    virtual ~GpuTransport() = default;
    // request: a 64-byte process_abi::Request plus its payload; reply receives the
    // 48-byte Reply plus any payload. false: the transport itself failed.
    virtual bool call(const uint8_t* request, uint32_t request_bytes, uint8_t* reply,
                      uint32_t reply_capacity, uint32_t& reply_bytes) noexcept = 0;
    // Window-clipped present of a graphics_abi::PresentInput (DrawEscape on the
    // caller's HDC). false: unavailable (no radeondisp hook, old win32k).
    virtual bool present(const uint8_t* input, uint32_t input_bytes, uint32_t& outcome,
                         uint32_t& rects_copied) noexcept = 0;
    // Called between polls of a busy fence or job slot (the ICD yields the thread).
    virtual void yield() noexcept {}
};

enum class FallbackReason : uint32_t {
    none,
    gpu_unavailable,          // no process service / session / gfx queue
    allocation_failed,        // buffers could not be allocated or mapped
    drawable_too_large,       // a dimension or viewport beyond the 4096 GPU limit
    depth_readback_too_large, // depth/stencil copy target beyond the buffer cap
    texture_unsupported,      // texture larger than the ABI's 4096 or the heap
    submission_failed,        // a submission or fence failed or timed out
    present_failed            // neither GPU present nor readback worked
};
const char* fallback_reason_text(FallbackReason reason) noexcept;

// Scratch the ICD provides: request staging and readback rows.
// Two halves: request (Request + payload) and reply (Reply + payload).
constexpr uint32_t gfx11_backend_half_bytes = 64u + rx7900::reactos::process_abi::bulk_bytes;
constexpr uint32_t gfx11_backend_scratch_bytes = 2u * gfx11_backend_half_bytes;

enum class PresentPath : uint8_t { gpu, readback, software, failed };

class Gfx11Backend final : public Backend {
public:
    // scratch: gfx11_backend_scratch_bytes, 8-byte aligned, owned by the caller
    // for the backend lifetime. software: the fallback renderer over the same
    // drawable; the caller keeps its storage sized for every resize().
    Gfx11Backend(GpuTransport& transport, uint8_t* scratch, uint64_t scratch_bytes,
                 SoftwareBackend& software) noexcept;
    ~Gfx11Backend() noexcept override;
    Gfx11Backend(const Gfx11Backend&) = delete;
    Gfx11Backend& operator=(const Gfx11Backend&) = delete;

    // Opens the session, a gfx queue and the graphics context. false keeps the
    // backend in software mode with reason() == gpu_unavailable.
    bool initialize() noexcept;

    bool resize(uint32_t width, uint32_t height) noexcept override;
    uint32_t width() const noexcept override { return width_; }
    uint32_t height() const noexcept override { return height_; }
    FramebufferFormat format() const noexcept override { return software_.format(); }
    void clear(const ClearRequest& request) noexcept override;
    void draw(const DrawState& state, PrimitiveKind kind, const RasterVertex* vertices,
              uint32_t count) noexcept override;
    void draw_pixels(const DrawState& state, const PixelFragments& pixels) noexcept override;
    bool copy_pixels(const DrawState& state, const PixelCopy& copy) noexcept override;
    void read(ReadSource source, gl::Enum buffer, int32_t x, int32_t y, uint32_t width,
              uint32_t height, float* output) noexcept override;
    void finish() noexcept override;
    void flush() noexcept override;
    bool healthy() const noexcept override { return healthy_; }

    // DrvSwapBuffers: GPU present of the back buffer through the transport, else
    // an exact readback into pixels (width*height BGRA, rows bottom to top for a
    // bottom-up DIB), else the software renderer's own buffer. Swaps afterwards.
    PresentPath present(uint32_t* pixels) noexcept;

    bool gpu_active() const noexcept { return gpu_active_; }
    FallbackReason reason() const noexcept { return reason_; }
    uint64_t session() const noexcept { return session_; }
    uint32_t submissions() const noexcept { return submissions_; }
    // Smaller texture heap (tests on a small synthetic GTT); before any texture.
    void limit_texture_heap(uint64_t bytes) noexcept { texture_heap_bytes_ = bytes; }
    // Submissions that needed layout 4 (tests).
    uint32_t layout4_submissions() const noexcept { return layout4_submissions_; }

private:
    struct Buffer final {
        uint64_t handle = 0;
        uint64_t va = 0;
        uint64_t bytes = 0;
    };
    // Every level a texture's minification filter reaches, in the heap.
    struct TextureEntry final {
        const Texture* texture = nullptr;
        uint32_t name = 0;
        uint64_t generation = 0;
        uint32_t levels = 0;
        uint32_t offset[texture_max_levels]{};
        uint32_t pitch[texture_max_levels]{};
    };
    // The surfaces one submission renders into.
    struct BatchTarget final {
        const Buffer* color = nullptr;
        const Buffer* color2 = nullptr;      // GL_FRONT_AND_BACK: the back buffer as MRT1
        const Buffer* depth = nullptr;
        uint32_t width = 0, height = 0;
        uint32_t kind = 0;                   // graphics_abi::TargetKind
    };
    // The image a pixel-rectangle draw reads (gl_gfx11_abi pixel_source).
    struct PixelSource final {
        uint8_t kind = 0;
        const Buffer* buffer = nullptr;
        uint32_t offset = 0;                 // byte offset of image row 0, column 0 (wraps)
        int32_t stride = 0;                  // bytes between image rows
        bool transfer = false;
        float scale[4] = {1, 1, 1, 1};
        float bias[4] = {0, 0, 0, 0};
        uint32_t shift_left = 0, shift_right = 0;
        int32_t index_offset = 0;
    };
    static constexpr uint32_t texture_entries = 64;
    uint64_t texture_heap_bytes_ = 16ULL << 20;

    // Process API helpers (all synchronous through the transport).
    bool call(uint32_t operation, uint64_t handle, uint64_t bytes, uint64_t offset,
              uint64_t address, uint32_t domain, uint32_t access, const uint8_t* payload,
              uint32_t payload_bytes, uint8_t* reply_payload, uint32_t reply_capacity,
              uint32_t& outcome, uint64_t& reply_handle, uint64_t& reply_bytes) noexcept;
    bool allocate(uint64_t bytes, uint64_t alignment, Buffer& buffer) noexcept;
    void release(Buffer& buffer) noexcept;
    bool upload(const Buffer& buffer, uint64_t offset, const uint8_t* bytes, uint64_t count) noexcept;
    bool readback(const Buffer& buffer, uint64_t offset, uint8_t* bytes, uint32_t count) noexcept;
    void release_surfaces() noexcept;

    // Batching.
    bool unsupported(const DrawState& state, FallbackReason& why) noexcept;
    BatchTarget draw_target(gl::Enum draw_buffer) const noexcept;
    bool encode_state(const DrawState& state, PrimitiveKind kind, const PixelSource* pixel,
                      const PixelFragments* rectangle, uint32_t& index) noexcept;
    bool texture_for(const Texture& texture, const TextureEntry*& entry) noexcept;
    bool append(uint32_t state_index, uint32_t primitive, uint32_t flags, const RasterVertex* vertices,
                uint32_t count, const BatchTarget& target) noexcept;
    bool submit_batch() noexcept;
    bool wait_idle() noexcept;
    bool depth_copy() noexcept;
    void fallback(FallbackReason why) noexcept;
    void migrate() noexcept;
    void put_vertex(uint32_t index, const RasterVertex& vertex, float stipple, float lod) noexcept;
    // Line stipple positions and line LODs of this draw's segments.
    void line_attributes(const DrawState& state, const RasterVertex* vertices, uint32_t count) noexcept;
    // A pixel rectangle: the zoomed image bounds as two triangles whose PS
    // applies the exact PixelFragments rule and fetches source.
    bool pixel_rectangle(const DrawState& state, const PixelFragments& pixels, const PixelSource& source,
                         const BatchTarget& target) noexcept;
    // Uploads bytes into the image heap; offset receives the region.
    bool upload_image(const uint8_t* bytes, uint64_t count, uint32_t& offset) noexcept;
    uint8_t* image_staging(uint64_t& capacity) noexcept;

    GpuTransport& transport_;
    uint8_t* scratch_;
    uint64_t scratch_bytes_;
    SoftwareBackend& software_;
    bool healthy_ = true;
    bool gpu_active_ = false;
    FallbackReason reason_ = FallbackReason::gpu_unavailable;
    uint64_t session_ = 0;
    uint64_t queue_ = 0;
    uint32_t width_ = 0;
    uint32_t height_ = 0;
    uint64_t next_va_ = 1ULL << 40;
    Buffer color_[2]{};        // [0] front, [1] back
    Buffer depth_{};
    Buffer copy_{};            // depth/stencil DB->CB copy target
    Buffer textures_{};
    uint64_t texture_used_ = 0;
    TextureEntry texture_cache_[texture_entries]{};
    Buffer images_{};          // pixel-rectangle images (bump allocated per idle period)
    uint64_t image_used_ = 0;
    bool images_in_flight_ = false;
    Buffer staging_{};         // CopyPixels colour staging (source rectangle)
    uint32_t last_sequence_ = 0;
    uint32_t submissions_ = 0;
    uint32_t layout4_submissions_ = 0;
    bool front_dirty_ = false;
    uint32_t stipple_counter_ = 0;

    // Pending batch (the wire payload is assembled at submit time).
    BatchTarget batch_target_{};
    bool batch_open_ = false;
    bool batch_images_ = false;
    // States are kept as values and as encoded layout-4 bytes so identical
    // states deduplicate.
    rx7900::reactos::graphics_abi::GraphicsState state_values_[rx7900::reactos::graphics_abi::max_states]{};
    uint8_t states_[rx7900::reactos::graphics_abi::max_states][rx7900::reactos::graphics_abi::state_v4_bytes]{};
    uint32_t state_count_ = 0;
    rx7900::reactos::graphics_abi::GraphicsCommand commands_[rx7900::reactos::graphics_abi::max_commands]{};
    uint32_t command_count_ = 0;
    uint8_t vertices_[rx7900::reactos::graphics_abi::max_vertex_bytes]{};
    uint32_t vertex_count_ = 0;
    // Per-vertex misc values of the line draw being appended (stipple position,
    // segment LOD); draws are appended in chunks of line_chunk vertices.
    static constexpr uint32_t line_chunk = 4096;
    float line_stipple_[line_chunk]{};
    float line_lod_[line_chunk]{};

    // Row staging for readback and software migration.
    float row_floats_[4 * rx7900::reactos::graphics_abi::max_dimension]{};
    float row_depth_[rx7900::reactos::graphics_abi::max_dimension]{};
    uint8_t row_stencil_[rx7900::reactos::graphics_abi::max_dimension]{};
};

} // namespace rx7900::opengl

/*
Module: rx7900.opengl.gl_gfx11_backend
Purpose: Batch frontend primitives and pixel rectangles into typed graphics
submissions and keep the software reference backend as an exact, reported
fallback.
Created: 2026-10-23
Architecture: See gl_gfx11_backend.hpp. All requests are encoded with the
process_abi / graphics_abi wire definitions the kernel decodes.
*/
#include "gl_gfx11_backend.hpp"

#include <string.h>

#include "gl_shader_gfx11.hpp"

namespace rx7900::opengl {
namespace {

namespace pabi = rx7900::reactos::process_abi;
namespace wire = rx7900::reactos::graphics_abi;
namespace abi = gfx11_abi;

constexpr uint32_t request_bytes = 64;
constexpr uint32_t reply_bytes = 48;
constexpr uint32_t spin_limit = 50000000;
constexpr uint64_t large_buffer_alignment = 2ULL << 20;   // lets GPUVM use 2 MiB fragments

uint32_t bits(float value) noexcept {
    uint32_t out = 0;
    memcpy(&out, &value, sizeof(out));
    return out;
}

float clamp_unit(double value) noexcept {
    return value < 0 ? 0.0f : value > 1 ? 1.0f : static_cast<float>(value);
}

float clamp_size(float value) noexcept {
    return !(value >= 1.0f / 128.0f) ? 1.0f / 128.0f : value > 256.0f ? 256.0f : value;
}

uint32_t finite_or_zero(float value) noexcept {
    const uint32_t raw = bits(value);
    return wire::finite_bits(raw) ? raw : 0;
}

uint32_t compare_code(gl::Enum func) noexcept {
    return func >= gl::never && func <= gl::never + 7 ? func - gl::never : 7;
}

uint32_t stencil_code(gl::Enum op) noexcept {
    switch (op) {
    case gl::zero: return 1;
    case gl::replace: return 2;
    case gl::incr: return 3;
    case gl::decr: return 4;
    case gl::invert: return 5;
    default: return 0;   // keep
    }
}

uint32_t blend_code(gl::Enum factor) noexcept {
    switch (factor) {
    case gl::zero: return 0;
    case gl::one: return 1;
    case gl::src_color: return 2;
    case gl::one_minus_src_color: return 3;
    case gl::dst_color: return 4;
    case gl::one_minus_dst_color: return 5;
    case gl::src_alpha: return 6;
    case gl::one_minus_src_alpha: return 7;
    case gl::dst_alpha: return 8;
    case gl::one_minus_dst_alpha: return 9;
    case gl::src_alpha_saturate: return 10;
    default: return 1;
    }
}

uint64_t align_up(uint64_t value, uint64_t alignment) noexcept {
    return (value + alignment - 1u) & ~(alignment - 1u);
}

uint32_t color_pitch_bytes(uint32_t width) noexcept {
    return gfx11::linear_pitch_pixels(width, gfx11::TargetKind::bgra8) * 4u;
}

uint32_t copy_pitch_bytes(uint32_t width) noexcept {
    return gfx11::linear_pitch_pixels(width, gfx11::TargetKind::depth_copy) * 8u;
}

float unorm8(uint8_t value) noexcept { return static_cast<float>(value) / 255.0f; }

// Levels the texture's minification filter reaches (all of them for the PS
// level loop, else level 0 only).
uint32_t uploaded_levels(const Texture& texture) noexcept {
    return texture_needs_lod(texture.sampler) ? texture_last_level(texture) + 1u : 1u;
}

uint64_t texture_bytes(const Texture& texture) noexcept {
    uint64_t total = 0;
    for (uint32_t level = 0; level < uploaded_levels(texture); ++level) {
        const TextureLevel& image = texture.levels[level];
        total += align_up(static_cast<uint64_t>(align_up(image.width, 64)) * image.height * 4u, 256);
    }
    return total;
}

// v4 features any state of a batch uses: such batches travel as layout 4.
bool needs_layout4(const wire::GraphicsState& s) noexcept {
    const auto& k = s.key;
    return k.min_filter != 0 || k.line_stipple != 0 || k.polygon_stipple != 0 || k.front_and_back != 0 ||
           k.pixel_source != 0 || k.pixel_transfer != 0 || s.level_count > 1;
}

} // namespace

const char* fallback_reason_text(FallbackReason reason) noexcept {
    switch (reason) {
    case FallbackReason::none: return "GPU path active";
    case FallbackReason::gpu_unavailable: return "native GPU process service unavailable";
    case FallbackReason::allocation_failed: return "GPU buffer allocation or mapping failed";
    case FallbackReason::drawable_too_large: return "drawable or viewport exceeds the 4096-pixel GPU limit";
    case FallbackReason::depth_readback_too_large: return "depth/stencil copy target exceeds the buffer limit";
    case FallbackReason::texture_unsupported: return "texture exceeds the GPU texture limits";
    case FallbackReason::submission_failed: return "GPU submission or fence failed";
    case FallbackReason::present_failed: return "GPU present and readback both failed";
    }
    return "unknown";
}

Gfx11Backend::Gfx11Backend(GpuTransport& transport, uint8_t* scratch, uint64_t scratch_bytes,
                           SoftwareBackend& software) noexcept
    : transport_(transport), scratch_(scratch), scratch_bytes_(scratch_bytes), software_(software) {}

Gfx11Backend::~Gfx11Backend() noexcept {
    if (session_ != 0) {
        wait_idle();
        release_surfaces();
        release(textures_);
        release(images_);
        release(staging_);
        uint32_t outcome = 0;
        uint64_t handle = 0, bytes = 0;
        call(static_cast<uint32_t>(pabi::Operation::close), 0, 0, 0, 0, 0, 0, nullptr, 0, nullptr, 0,
             outcome, handle, bytes);
    }
}

bool Gfx11Backend::call(uint32_t operation, uint64_t handle, uint64_t bytes, uint64_t offset,
                        uint64_t address, uint32_t domain, uint32_t access, const uint8_t* payload,
                        uint32_t payload_bytes, uint8_t* reply_payload, uint32_t reply_capacity,
                        uint32_t& outcome, uint64_t& reply_handle, uint64_t& reply_count) noexcept {
    outcome = static_cast<uint32_t>(pabi::Outcome::unavailable);
    reply_handle = 0;
    reply_count = 0;
    if (scratch_ == nullptr || scratch_bytes_ < gfx11_backend_scratch_bytes ||
        payload_bytes > pabi::bulk_bytes || reply_capacity > pabi::bulk_bytes) {
        return false;
    }
    uint8_t* request = scratch_;
    uint8_t* reply = scratch_ + gfx11_backend_half_bytes;
    wire::Writer w(request, request_bytes, 0);
    const bool session_free = operation == static_cast<uint32_t>(pabi::Operation::query) ||
                              operation == static_cast<uint32_t>(pabi::Operation::open);
    w.u32(request_bytes); w.u32(pabi::graphics_version); w.u32(operation); w.u32(0);
    w.u64(session_free ? 0 : session_); w.u64(handle); w.u64(bytes); w.u64(offset); w.u64(address);
    w.u32(domain); w.u32(access);
    if (!w.ok()) return false;
    if (payload_bytes != 0 && payload != request + request_bytes) {
        memmove(request + request_bytes, payload, payload_bytes);
    }
    uint32_t received = 0;
    if (!transport_.call(request, request_bytes + payload_bytes, reply, reply_bytes + reply_capacity, received) ||
        received < reply_bytes) {
        return false;
    }
    const ByteView view(reply, received);
    uint32_t returned_payload = 0;
    uint64_t returned_session = 0;
    view.read_u32(8, outcome);
    view.read_u32(12, returned_payload);
    view.read_u64(16, returned_session);
    view.read_u64(24, reply_handle);
    view.read_u64(32, reply_count);
    if (operation == static_cast<uint32_t>(pabi::Operation::open) &&
        outcome == static_cast<uint32_t>(pabi::Outcome::success)) {
        reply_handle = returned_session;
    }
    if (returned_payload != 0) {
        if (reply_payload == nullptr || returned_payload > reply_capacity ||
            received != reply_bytes + returned_payload) {
            return false;
        }
        memcpy(reply_payload, reply + reply_bytes, returned_payload);
    }
    return true;
}

bool Gfx11Backend::initialize() noexcept {
    uint32_t outcome = 0;
    uint64_t handle = 0, bytes = 0;
    const auto ok = [&]() { return outcome == static_cast<uint32_t>(pabi::Outcome::success); };
    if (!call(static_cast<uint32_t>(pabi::Operation::query), 0, 0, 0, 0, 0, 0, nullptr, 0, nullptr, 0,
              outcome, handle, bytes) || !ok() ||
        !call(static_cast<uint32_t>(pabi::Operation::open), 0, 0, 0, 0, 0, 0, nullptr, 0, nullptr, 0,
              outcome, handle, bytes) || !ok() || handle == 0) {
        reason_ = FallbackReason::gpu_unavailable;
        return false;
    }
    session_ = handle;
    // The session's MES gfx queue first; without one (no queue, or a context
    // the queue refused) the kernel gfx ring runs the same typed draws
    // (graphics_open with queue handle 0).
    queue_ = 0;
    if (call(static_cast<uint32_t>(pabi::Operation::queue_create), 0, 0, 0, 0, 0, 0, nullptr, 0, nullptr, 0,
             outcome, handle, bytes) && ok() && handle != 0) {
        queue_ = handle;
        if (!call(static_cast<uint32_t>(pabi::Operation::graphics_open), queue_, 0, 0, 0, 0, 0, nullptr, 0,
                  nullptr, 0, outcome, handle, bytes) || !ok()) {
            call(static_cast<uint32_t>(pabi::Operation::queue_destroy), queue_, 0, 0, 0, 0, 0, nullptr, 0,
                 nullptr, 0, outcome, handle, bytes);
            queue_ = 0;
        }
    }
    if (queue_ == 0 &&
        (!call(static_cast<uint32_t>(pabi::Operation::graphics_open), 0, 0, 0, 0, 0, 0, nullptr, 0,
               nullptr, 0, outcome, handle, bytes) || !ok())) {
        reason_ = FallbackReason::gpu_unavailable;
        return false;
    }
    gpu_active_ = true;
    reason_ = FallbackReason::none;
    return true;
}

bool Gfx11Backend::allocate(uint64_t bytes, uint64_t alignment, Buffer& buffer) noexcept {
    buffer = {};
    const uint64_t size = align_up(bytes, wire::buffer_page_bytes);
    if (size == 0 || size > wire::buffer_max_bytes) return false;
    uint32_t outcome = 0;
    uint64_t handle = 0, count = 0;
    if (!call(static_cast<uint32_t>(pabi::Operation::allocate), 0, size, 0, 0, 0, 3, nullptr, 0, nullptr, 0,
              outcome, handle, count) || outcome != 0 || handle == 0) return false;
    // Buffers of 2 MiB or more start on a 2 MiB boundary so the page tables can
    // use large fragments where the backing allows it.
    const uint64_t va = align_up(next_va_, size >= large_buffer_alignment && alignment < large_buffer_alignment
                                               ? large_buffer_alignment : alignment);
    uint64_t ignored = 0;
    if (!call(static_cast<uint32_t>(pabi::Operation::map), handle, 0, 0, va, 0, 3, nullptr, 0, nullptr, 0,
              outcome, ignored, count) || outcome != 0) {
        call(static_cast<uint32_t>(pabi::Operation::release), handle, 0, 0, 0, 0, 0, nullptr, 0, nullptr, 0,
             outcome, ignored, count);
        return false;
    }
    next_va_ = va + size + wire::buffer_page_bytes;   // unmapped guard page between buffers
    buffer = {handle, va, size};
    return true;
}

void Gfx11Backend::release(Buffer& buffer) noexcept {
    if (buffer.handle == 0) return;
    uint32_t outcome = 0;
    uint64_t handle = 0, count = 0;
    call(static_cast<uint32_t>(pabi::Operation::unmap), buffer.handle, 0, 0, 0, 0, 0, nullptr, 0, nullptr, 0,
         outcome, handle, count);
    call(static_cast<uint32_t>(pabi::Operation::release), buffer.handle, 0, 0, 0, 0, 0, nullptr, 0, nullptr, 0,
         outcome, handle, count);
    buffer = {};
}

void Gfx11Backend::release_surfaces() noexcept {
    release(color_[0]);
    release(color_[1]);
    release(depth_);
    release(copy_);
}

bool Gfx11Backend::upload(const Buffer& buffer, uint64_t offset, const uint8_t* bytes, uint64_t count) noexcept {
    for (uint64_t done = 0; done < count;) {
        const uint32_t chunk = static_cast<uint32_t>(count - done < pabi::bulk_bytes ? count - done : pabi::bulk_bytes);
        uint32_t outcome = 0;
        uint64_t handle = 0, returned = 0;
        if (!call(static_cast<uint32_t>(pabi::Operation::buffer_upload), buffer.handle, chunk, offset + done, 0, 0, 0,
                  bytes + done, chunk, nullptr, 0, outcome, handle, returned) || outcome != 0) return false;
        done += chunk;
    }
    return true;
}

bool Gfx11Backend::readback(const Buffer& buffer, uint64_t offset, uint8_t* bytes, uint32_t count) noexcept {
    uint32_t outcome = 0;
    uint64_t handle = 0, returned = 0;
    return count <= pabi::bulk_bytes &&
           call(static_cast<uint32_t>(pabi::Operation::buffer_readback), buffer.handle, count, offset, 0, 0, 0,
                nullptr, 0, bytes, count, outcome, handle, returned) && outcome == 0;
}

bool Gfx11Backend::resize(uint32_t width, uint32_t height) noexcept {
    if (width == width_ && height == height_ && (!gpu_active_ || color_[1].handle != 0)) {
        return true;
    }
    if (gpu_active_) {
        wait_idle();
        release_surfaces();
    }
    if (!software_.resize(width, height)) {
        healthy_ = false;
        return false;
    }
    width_ = width;
    height_ = height;
    // Contents are undefined after a resize, so a state-driven fallback may
    // return to the GPU; service-level failures stay in software.
    const bool resumable = reason_ != FallbackReason::none && reason_ != FallbackReason::gpu_unavailable &&
                           reason_ != FallbackReason::submission_failed &&
                           reason_ != FallbackReason::allocation_failed &&
                           reason_ != FallbackReason::present_failed && session_ != 0;
    if (!gpu_active_ && !resumable) return true;
    gpu_active_ = true;
    reason_ = FallbackReason::none;
    if (width == 0 || height == 0) return true;
    gfx11::DepthLayout layout{};
    const uint64_t color_bytes = static_cast<uint64_t>(color_pitch_bytes(width)) * height;
    if (width > wire::max_dimension || height > wire::max_dimension ||
        !gfx11::depth_layout(width, height, layout) ||
        color_bytes > wire::buffer_max_bytes ||
        layout.total_bytes > wire::buffer_max_bytes) {
        fallback(FallbackReason::drawable_too_large);
        return true;
    }
    if (!allocate(color_bytes, 256, color_[0]) || !allocate(color_bytes, 256, color_[1]) ||
        !allocate(layout.total_bytes, gfx11::depth_block_bytes, depth_)) {
        release_surfaces();
        fallback(FallbackReason::allocation_failed);
        return true;
    }
    return true;
}

void Gfx11Backend::fallback(FallbackReason why) noexcept {
    if (!gpu_active_) return;
    gpu_active_ = false;
    reason_ = why;
    command_count_ = 0;
    state_count_ = 0;
    vertex_count_ = 0;
    batch_open_ = false;
    batch_images_ = false;
}

bool Gfx11Backend::unsupported(const DrawState& state, FallbackReason& why) noexcept {
    if (state.viewport_width > static_cast<int32_t>(2 * wire::max_dimension) ||
        state.viewport_height > static_cast<int32_t>(2 * wire::max_dimension) ||
        state.viewport_width < 0 || state.viewport_height < 0) {
        why = FallbackReason::drawable_too_large;
    } else if (state.texture != nullptr &&
               (state.texture->levels[0].width > wire::max_dimension ||
                state.texture->levels[0].height > wire::max_dimension ||
                texture_bytes(*state.texture) > texture_heap_bytes_)) {
        why = FallbackReason::texture_unsupported;
    } else {
        return false;
    }
    return true;
}

Gfx11Backend::BatchTarget Gfx11Backend::draw_target(gl::Enum draw_buffer) const noexcept {
    BatchTarget target{};
    target.color = draw_buffer == gl::front || draw_buffer == gl::front_and_back ? &color_[0] : &color_[1];
    target.color2 = draw_buffer == gl::front_and_back ? &color_[1] : nullptr;
    target.depth = &depth_;
    target.width = width_;
    target.height = height_;
    target.kind = static_cast<uint32_t>(wire::TargetKind::bgra8);
    return target;
}

bool Gfx11Backend::texture_for(const Texture& texture, const TextureEntry*& entry) noexcept {
    TextureEntry* slot = nullptr;
    for (auto& cached : texture_cache_) {
        if (cached.texture == &texture && cached.name == texture.name) {
            if (cached.generation == texture.generation) {
                entry = &cached;
                return true;
            }
            slot = &cached;
            break;
        }
    }
    // The heap is one leased process buffer: drain GPU work before writing it.
    if (!submit_batch() || !wait_idle()) return false;
    if (textures_.handle == 0 && !allocate(texture_heap_bytes_, 256, textures_)) return false;
    const uint64_t bytes = texture_bytes(texture);
    if (texture_used_ + bytes > texture_heap_bytes_) {
        texture_used_ = 0;
        for (auto& cached : texture_cache_) cached = {};
        slot = nullptr;
    }
    if (slot == nullptr) {
        for (auto& cached : texture_cache_) {
            if (cached.texture == nullptr) { slot = &cached; break; }
        }
    }
    if (slot == nullptr) {
        texture_used_ = 0;
        for (auto& cached : texture_cache_) cached = {};
        slot = &texture_cache_[0];
    }
    *slot = {};
    // Every reached level, rows in bulk-sized groups through the request staging area.
    uint8_t* staging = scratch_ + request_bytes;
    const uint32_t levels = uploaded_levels(texture);
    for (uint32_t index = 0; index < levels; ++index) {
        const TextureLevel& level = texture.levels[index];
        const uint32_t region = static_cast<uint32_t>(texture_used_);
        const uint32_t row_pitch = static_cast<uint32_t>(align_up(level.width, 64));
        const uint32_t row_bytes = row_pitch * 4u;
        const uint32_t rows_per_chunk = pabi::bulk_bytes / row_bytes;
        for (uint32_t row = 0; row < level.height; row += rows_per_chunk) {
            const uint32_t rows = level.height - row < rows_per_chunk ? level.height - row : rows_per_chunk;
            memset(staging, 0, static_cast<size_t>(rows) * row_bytes);
            for (uint32_t r = 0; r < rows; ++r) {
                memcpy(staging + static_cast<size_t>(r) * row_bytes,
                       level.texels + (static_cast<size_t>(row + r) * level.width) * 4u, level.width * 4u);
            }
            if (!upload(textures_, region + static_cast<uint64_t>(row) * row_bytes, staging,
                        static_cast<uint64_t>(rows) * row_bytes)) return false;
        }
        texture_used_ = align_up(texture_used_ + static_cast<uint64_t>(row_bytes) * level.height, 256);
        slot->offset[index] = region;
        slot->pitch[index] = row_pitch;
    }
    slot->texture = &texture;
    slot->name = texture.name;
    slot->generation = texture.generation;
    slot->levels = levels;
    entry = slot;
    return true;
}

bool Gfx11Backend::encode_state(const DrawState& state, PrimitiveKind kind, const PixelSource* pixel,
                                const PixelFragments* rectangle, uint32_t& index) noexcept {
    wire::GraphicsState s{};
    s.viewport_x = state.viewport_x;
    s.viewport_y = state.viewport_y;
    s.viewport_width = static_cast<uint32_t>(state.viewport_width);
    s.viewport_height = static_cast<uint32_t>(state.viewport_height);
    s.depth_near = bits(clamp_unit(state.depth_near));
    s.depth_far = bits(clamp_unit(state.depth_far));
    s.scissor = state.scissor_enabled ? 1 : 0;
    if (state.scissor_enabled) {
        s.scissor_x = state.scissor_x;
        s.scissor_y = state.scissor_y;
        s.scissor_width = static_cast<uint32_t>(state.scissor_width < 0 ? 0 :
            state.scissor_width > 2 * static_cast<int32_t>(wire::max_dimension) ?
            2 * static_cast<int32_t>(wire::max_dimension) : state.scissor_width);
        s.scissor_height = static_cast<uint32_t>(state.scissor_height < 0 ? 0 :
            state.scissor_height > 2 * static_cast<int32_t>(wire::max_dimension) ?
            2 * static_cast<int32_t>(wire::max_dimension) : state.scissor_height);
        s.scissor_x = s.scissor_x < -32768 ? -32768 : s.scissor_x > 32767 ? 32767 : s.scissor_x;
        s.scissor_y = s.scissor_y < -32768 ? -32768 : s.scissor_y > 32767 ? 32767 : s.scissor_y;
    }
    s.viewport_x = s.viewport_x < -32768 ? -32768 : s.viewport_x > 32767 ? 32767 : s.viewport_x;
    s.viewport_y = s.viewport_y < -32768 ? -32768 : s.viewport_y > 32767 ? 32767 : s.viewport_y;
    if (state.cull_enabled) {
        s.cull = state.cull_mode == gl::front ? 1 : state.cull_mode == gl::back ? 2 : 3;
    }
    s.front_ccw = state.front_ccw ? 1 : 0;
    s.point_size = bits(clamp_size(state.point_size));
    s.line_width = bits(clamp_size(state.line_width));
    s.offset = state.offset_enabled ? 1 : 0;
    if (state.offset_enabled) {
        s.offset_factor = finite_or_zero(state.offset_factor);
        s.offset_units = finite_or_zero(state.offset_units);
    }
    s.depth_test = state.depth_test ? 1 : 0;
    s.depth_func = compare_code(state.depth_func);
    s.depth_write = state.depth_write ? 1 : 0;
    s.stencil_test = state.stencil_test ? 1 : 0;
    s.stencil_func = compare_code(state.stencil.func);
    s.stencil_ref = static_cast<uint32_t>(state.stencil.ref < 0 ? 0 : state.stencil.ref > 255 ? 255 :
                                          state.stencil.ref);
    s.stencil_value_mask = state.stencil.value_mask & 0xffu;
    s.stencil_write_mask = state.stencil.write_mask & 0xffu;
    s.stencil_fail = stencil_code(state.stencil.fail);
    s.stencil_depth_fail = stencil_code(state.stencil.depth_fail);
    s.stencil_depth_pass = stencil_code(state.stencil.depth_pass);
    s.blend = state.blend ? 1 : 0;
    s.blend_src = blend_code(state.blend_src);
    s.blend_dst = blend_code(state.blend_dst);
    s.logic_op = state.logic_op ? 1 : 0;
    s.logic_op_code = state.logic_op_mode >= gl::logic_clear && state.logic_op_mode <= gl::logic_set ?
                      state.logic_op_mode - gl::logic_clear : 3;
    s.color_mask = state.draw_buffer == gl::none ? 0u :
        (state.color_write[0] ? 1u : 0u) | (state.color_write[1] ? 2u : 0u) |
        (state.color_write[2] ? 4u : 0u) | (state.color_write[3] ? 8u : 0u);
    s.alpha_ref = bits(clamp_unit(state.alpha_ref));
    const float env[4] = {state.texture_env_color.x, state.texture_env_color.y,
                          state.texture_env_color.z, state.texture_env_color.w};
    const float fog[3] = {state.fog_color.x, state.fog_color.y, state.fog_color.z};
    for (uint32_t i = 0; i < 4; ++i) s.env_color[i] = bits(clamp_unit(env[i]));
    for (uint32_t i = 0; i < 3; ++i) s.fog_color[i] = bits(clamp_unit(fog[i]));
    s.key = shader::clip_space_key(state, kind);
    if (state.texture != nullptr) {
        const TextureEntry* entry = nullptr;
        if (!texture_for(*state.texture, entry)) return false;
        const TextureSampler& sampler = state.texture->sampler;
        s.texture = textures_.handle;
        s.texture_offset = entry->offset[0];
        s.texture_width = state.texture->levels[0].width;
        s.texture_height = state.texture->levels[0].height;
        s.texture_pitch = entry->pitch[0];
        s.wrap_s = sampler.wrap_s == gl::repeat ? 0 : 1;
        s.wrap_t = sampler.wrap_t == gl::repeat ? 0 : 1;
        // Without LOD selection minification and magnification are one filter.
        s.filter = sampler.mag_filter == gl::nearest ? 0 : 1;
        s.level_count = entry->levels;
        for (uint32_t level = 1; level < entry->levels; ++level) {
            s.levels[level - 1u] = {entry->offset[level], state.texture->levels[level].width,
                                    state.texture->levels[level].height, entry->pitch[level]};
        }
    }
    if (s.key.line_stipple != 0) {
        s.line_stipple_factor = state.line_stipple_factor < 1 ? 1u : state.line_stipple_factor > 256 ? 256u :
                                state.line_stipple_factor;
        s.line_stipple_pattern = state.line_stipple_pattern;
    }
    if (s.key.polygon_stipple != 0) {
        for (uint32_t row = 0; row < 32; ++row) s.polygon_stipple[row] = state.polygon_stipple_rows[row];
    }
    if (pixel != nullptr && rectangle != nullptr) {
        s.key.pixel_source = pixel->kind;
        s.key.pixel_transfer = pixel->transfer ? 1 : 0;
        s.pixel_buffer = pixel->buffer->handle;
        s.pixel_offset = pixel->offset;
        s.pixel_row_stride = pixel->stride;
        s.pixel_columns = rectangle->width;
        s.pixel_first_row = rectangle->first_row;
        s.pixel_end_row = rectangle->first_row + rectangle->height;
        s.raster_x = bits(rectangle->x);
        s.raster_y = bits(rectangle->y);
        s.inv_zoom_x = bits(1.0f / rectangle->zoom_x);
        s.inv_zoom_y = bits(1.0f / rectangle->zoom_y);
        if (pixel->transfer) {
            for (uint32_t c = 0; c < 4; ++c) {
                s.transfer_scale[c] = finite_or_zero(pixel->scale[c]);
                s.transfer_bias[c] = finite_or_zero(pixel->bias[c]);
            }
            s.index_shift_left = pixel->shift_left;
            s.index_shift_right = pixel->shift_right;
            s.index_offset = pixel->index_offset;
        }
    }
    uint8_t encoded[wire::state_v4_bytes]{};
    if (!wire::encode_state_v4(s, encoded, sizeof(encoded))) return false;
    for (uint32_t i = 0; i < state_count_; ++i) {
        if (memcmp(states_[i], encoded, sizeof(encoded)) == 0) {
            index = i;
            return true;
        }
    }
    if (state_count_ == wire::max_states && !submit_batch()) return false;
    memcpy(states_[state_count_], encoded, sizeof(encoded));
    state_values_[state_count_] = s;
    index = state_count_++;
    return true;
}

void Gfx11Backend::put_vertex(uint32_t index, const RasterVertex& v, float stipple, float lod) noexcept {
    const float values[20] = {v.clip.x, v.clip.y, v.clip.z, v.clip.w,
                              v.color[0].x, v.color[0].y, v.color[0].z, v.color[0].w,
                              v.color[1].x, v.color[1].y, v.color[1].z, v.color[1].w,
                              v.texcoord.x, v.texcoord.y, v.texcoord.z, v.texcoord.w,
                              v.fog, stipple, lod, 0};
    uint8_t* record = vertices_ + static_cast<size_t>(index) * abi::clip_record_stride;
    for (uint32_t i = 0; i < 20; ++i) {
        const uint32_t word = bits(values[i]);
        for (uint32_t b = 0; b < 4; ++b) record[i * 4 + b] = static_cast<uint8_t>(word >> (8 * b));
    }
}

void Gfx11Backend::line_attributes(const DrawState& state, const RasterVertex* vertices, uint32_t count) noexcept {
    // The gl_raster DDA: steps + 1 samples per segment, sample k at counter
    // c0 + k. Records carry c0 and c0 + steps; the PS recovers k from the
    // linear interpolation. The counter wraps at 16 * factor (the pattern
    // period) so the PS arithmetic stays exact.
    if (state.line_stipple_reset) stipple_counter_ = 0;
    const uint32_t factor = state.line_stipple_factor < 1 ? 1u : state.line_stipple_factor > 256 ? 256u :
                            state.line_stipple_factor;
    const bool lod = state.texture != nullptr && texture_needs_lod(state.texture->sampler);
    for (uint32_t base = 0; base + 2 <= count; base += 2) {
        float ax = 0, ay = 0, bx = 0, by = 0;
        line_stipple_[base] = line_stipple_[base + 1] = 0;
        line_lod_[base] = line_lod_[base + 1] = -texture_lod_limit;
        if (!window_xy(vertices[base], state, ax, ay) || !window_xy(vertices[base + 1], state, bx, by)) {
            continue;
        }
        if (lod) {
            line_lod_[base] = line_lod_[base + 1] =
                line_lod(ax, ay, bx, by, vertices[base].texcoord, vertices[base + 1].texcoord,
                         state.texture->levels[0].width, state.texture->levels[0].height);
        }
        const int32_t steps = line_steps(ax, ay, bx, by);
        if (!state.line_stipple || steps <= 0) {
            continue;
        }
        line_stipple_[base] = static_cast<float>(stipple_counter_);
        line_stipple_[base + 1] = static_cast<float>(stipple_counter_ + static_cast<uint32_t>(steps));
        stipple_counter_ = (stipple_counter_ + static_cast<uint32_t>(steps) + 1u) % (16u * factor);
    }
}

bool Gfx11Backend::append(uint32_t state_index, uint32_t primitive, uint32_t flags,
                          const RasterVertex* vertices, uint32_t count, const BatchTarget& target) noexcept {
    const uint32_t capacity = wire::max_vertex_bytes / abi::clip_record_stride;
    if ((batch_open_ && (batch_target_.color != target.color || batch_target_.color2 != target.color2 ||
                         batch_target_.depth != target.depth || batch_target_.width != target.width ||
                         batch_target_.height != target.height || batch_target_.kind != target.kind)) ||
        vertex_count_ + count > capacity) {
        // State indices refer to this batch; re-encode them in the next one.
        return false;
    }
    batch_target_ = target;
    batch_open_ = true;
    wire::GraphicsCommand* last = command_count_ != 0 ? &commands_[command_count_ - 1] : nullptr;
    if (last == nullptr || last->state_index != state_index || last->primitive != primitive ||
        last->flags != flags) {
        if (command_count_ == wire::max_commands) return false;
        commands_[command_count_++] = {state_index, primitive, vertex_count_, 0, flags, 0};
        last = &commands_[command_count_ - 1];
    }
    const bool lines = primitive == 2;
    for (uint32_t i = 0; i < count; ++i) {
        put_vertex(vertex_count_ + i, vertices[i], lines ? line_stipple_[i] : 0.0f, lines ? line_lod_[i] : 0.0f);
    }
    vertex_count_ += count;
    last->vertex_count += count;
    batch_images_ = batch_images_ ||
                    (images_.handle != 0 && state_values_[state_index].pixel_buffer == images_.handle);
    return true;
}

bool Gfx11Backend::submit_batch() noexcept {
    if (command_count_ == 0) {
        state_count_ = 0;
        vertex_count_ = 0;
        batch_open_ = false;
        batch_images_ = false;
        return true;
    }
    bool layout4 = batch_target_.color2 != nullptr;
    for (uint32_t i = 0; i < state_count_; ++i) {
        layout4 = layout4 || needs_layout4(state_values_[i]);
    }
    wire::SubmitHeader header{};
    header.header_bytes = layout4 ? wire::submit_header_v4_bytes : wire::submit_header_bytes;
    header.state_count = state_count_;
    header.command_count = command_count_;
    header.vertex_bytes = vertex_count_ * abi::clip_record_stride;
    header.color = batch_target_.color->handle;
    header.depth = batch_target_.depth != nullptr ? batch_target_.depth->handle : 0;
    header.width = batch_target_.width;
    header.height = batch_target_.height;
    header.target_kind = batch_target_.kind;
    if (layout4) {
        header.color2 = batch_target_.color2 != nullptr ? batch_target_.color2->handle : 0;
        header.layout = wire::submit_layout_v4;
    }
    const uint32_t payload = wire::submit_payload_bytes(header);
    uint8_t* out = scratch_ + request_bytes;
    bool ok = payload <= pabi::bulk_bytes && wire::encode_header(header, out, pabi::bulk_bytes);
    uint32_t offset = header.header_bytes;
    for (uint32_t i = 0; ok && i < state_count_; ++i, offset += wire::submit_state_bytes(header)) {
        if (layout4) {
            memcpy(out + offset, states_[i], wire::state_v4_bytes);
        } else {
            ok = wire::encode_state(state_values_[i], out + offset, pabi::bulk_bytes - offset);
        }
    }
    for (uint32_t i = 0; ok && i < command_count_; ++i, offset += wire::command_bytes) {
        ok = wire::encode_command(commands_[i], out + offset, pabi::bulk_bytes - offset);
    }
    if (ok) memcpy(out + offset, vertices_, header.vertex_bytes);
    const bool images = batch_images_;
    command_count_ = 0;
    state_count_ = 0;
    vertex_count_ = 0;
    batch_open_ = false;
    batch_images_ = false;
    uint32_t outcome = 0;
    uint64_t handle = 0, sequence = 0;
    for (uint32_t attempt = 0; ok; ++attempt) {
        if (!call(static_cast<uint32_t>(pabi::Operation::graphics_submit), queue_, payload, 0, 0, 0, 0,
                  out, payload, nullptr, 0, outcome, handle, sequence)) {
            ok = false;
            break;
        }
        if (outcome == static_cast<uint32_t>(pabi::Outcome::busy) && attempt < spin_limit) {
            transport_.yield();       // both job slots in flight: retry
            continue;
        }
        ok = outcome == static_cast<uint32_t>(pabi::Outcome::success) && sequence != 0;
        break;
    }
    if (!ok) {
        fallback(FallbackReason::submission_failed);
        return false;
    }
    last_sequence_ = static_cast<uint32_t>(sequence);
    images_in_flight_ = images_in_flight_ || images;
    ++submissions_;
    layout4_submissions_ += layout4 ? 1u : 0u;
    return true;
}

bool Gfx11Backend::wait_idle() noexcept {
    if (!submit_batch()) return false;
    if (last_sequence_ == 0) {
        images_in_flight_ = false;
        return true;
    }
    for (uint32_t attempt = 0; attempt < spin_limit; ++attempt) {
        uint32_t outcome = 0;
        uint64_t handle = 0, sequence = 0;
        if (!call(static_cast<uint32_t>(pabi::Operation::graphics_query), queue_, 0, last_sequence_, 0, 0, 0,
                  nullptr, 0, nullptr, 0, outcome, handle, sequence)) break;
        if (outcome == static_cast<uint32_t>(pabi::Outcome::success)) {
            last_sequence_ = 0;
            images_in_flight_ = false;
            return true;
        }
        if (outcome != static_cast<uint32_t>(pabi::Outcome::busy)) break;
        transport_.yield();
    }
    fallback(FallbackReason::submission_failed);
    return false;
}

void Gfx11Backend::draw(const DrawState& state, PrimitiveKind kind, const RasterVertex* vertices,
                        uint32_t count) noexcept {
    FallbackReason why = FallbackReason::none;
    if (gpu_active_ && unsupported(state, why)) {
        migrate();
        fallback(why);
    }
    if (!gpu_active_) {
        software_.draw(state, kind, vertices, count);
        return;
    }
    if (vertices == nullptr || count == 0 || width_ == 0 || height_ == 0) return;
    const uint32_t primitive = kind == PrimitiveKind::points ? 1u : kind == PrimitiveKind::lines ? 2u : 3u;
    const BatchTarget target = draw_target(state.draw_buffer);
    if (state.draw_buffer == gl::front || state.draw_buffer == gl::front_and_back) front_dirty_ = true;
    const uint32_t usable = count - count % primitive;
    // Lines carry per-segment stipple positions and LODs: appended in chunks.
    const uint32_t chunk = kind == PrimitiveKind::lines ? line_chunk : usable;
    DrawState continued = state;
    for (uint32_t first = 0; first < usable && gpu_active_; first += chunk) {
        const uint32_t part = usable - first < chunk ? usable - first : chunk;
        if (kind == PrimitiveKind::lines) {
            line_attributes(continued, vertices + first, part);
            continued.line_stipple_reset = false;
        }
        bool appended = false;
        for (uint32_t attempt = 0; attempt < 2 && gpu_active_ && !appended; ++attempt) {
            uint32_t index = 0;
            if (!encode_state(state, kind, nullptr, nullptr, index)) break;
            appended = append(index, primitive, 0, vertices + first, part, target);
            if (!appended && !submit_batch()) break;
        }
        if (!appended) {
            if (gpu_active_) fallback(FallbackReason::submission_failed);
            software_.draw(state, kind, vertices, count);
            return;
        }
    }
}

void Gfx11Backend::clear(const ClearRequest& request) noexcept {
    if (!gpu_active_) {
        software_.clear(request);
        return;
    }
    // A clear is a full-viewport rectangle with depth ALWAYS/stencil REPLACE;
    // GL_FRONT_AND_BACK writes both colour buffers in one MRT0+MRT1 pass.
    const bool color = (request.mask & gl::color_buffer_bit) != 0;
    const bool depth = (request.mask & gl::depth_buffer_bit) != 0;
    const bool stencil = (request.mask & gl::stencil_buffer_bit) != 0;
    DrawState state{};
    state.viewport_width = static_cast<int32_t>(width_);
    state.viewport_height = static_cast<int32_t>(height_);
    state.scissor_enabled = request.scissor_enabled;
    state.scissor_x = request.scissor_x;
    state.scissor_y = request.scissor_y;
    state.scissor_width = request.scissor_width;
    state.scissor_height = request.scissor_height;
    state.depth_test = depth;
    state.depth_func = gl::always;
    state.depth_write = depth && request.depth_write;
    state.stencil_test = stencil;
    state.stencil.func = gl::always;
    state.stencil.ref = request.stencil;
    state.stencil.write_mask = request.stencil_write;
    state.stencil.depth_pass = gl::replace;
    for (uint32_t i = 0; i < 4; ++i) state.color_write[i] = color && request.color_write[i];
    state.draw_buffer = request.draw_buffer;
    state.dither = false;
    const float z = static_cast<float>((request.depth < 0 ? 0 : request.depth > 1 ? 1 : request.depth) * 2.0 - 1.0);
    const Vec4 c{clamp_unit(request.color.x), clamp_unit(request.color.y), clamp_unit(request.color.z),
                 clamp_unit(request.color.w)};
    const float corners[6][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, -1}, {1, 1}, {-1, 1}};
    RasterVertex quad[6];
    for (uint32_t i = 0; i < 6; ++i) {
        quad[i].clip = {corners[i][0], corners[i][1], z, 1};
        quad[i].color[0] = c;
        quad[i].color[1] = c;
    }
    if (!color && !depth && !stencil) return;
    draw(state, PrimitiveKind::triangles, quad, 6);
}

uint8_t* Gfx11Backend::image_staging(uint64_t& capacity) noexcept {
    capacity = pabi::bulk_bytes;
    return scratch_ + request_bytes;
}

bool Gfx11Backend::upload_image(const uint8_t* bytes, uint64_t count, uint32_t& offset) noexcept {
    // The heap is leased while submitted work reads it: bump-allocate inside
    // one idle period, drain when it is full or still in flight.
    const uint64_t needed = align_up(count, 256);
    if (needed > wire::buffer_max_bytes) return false;
    if (images_in_flight_ || image_used_ + needed > images_.bytes) {
        if (!wait_idle()) return false;
        image_used_ = 0;
    }
    if (needed > images_.bytes) {
        release(images_);
        const uint64_t size = needed < (4ULL << 20) ? (4ULL << 20) : needed;
        if (!allocate(size, 256, images_)) return false;
    }
    offset = static_cast<uint32_t>(image_used_);
    if (bytes != nullptr && !upload(images_, image_used_, bytes, count)) return false;
    image_used_ += needed;
    return true;
}

bool Gfx11Backend::pixel_rectangle(const DrawState& state, const PixelFragments& pixels, const PixelSource& source,
                                   const BatchTarget& target) noexcept {
    // Candidate pixels: the zoomed image bounds of the present rows, widened
    // by one pixel and clamped to the target (as gl_raster draw_pixels); the
    // PS decides every centre with the exact PixelFragments rule.
    const float far_x = pixels.x + pixels.zoom_x * static_cast<float>(pixels.width);
    const float near_y = pixels.y + pixels.zoom_y * static_cast<float>(pixels.first_row);
    const float far_y = pixels.y + pixels.zoom_y * static_cast<float>(pixels.first_row + pixels.height);
    const float lo_x = (pixels.x < far_x ? pixels.x : far_x) - 1.0f;
    const float hi_x = (pixels.x < far_x ? far_x : pixels.x) + 1.0f;
    const float lo_y = (near_y < far_y ? near_y : far_y) - 1.0f;
    const float hi_y = (near_y < far_y ? far_y : near_y) + 1.0f;
    const float width = static_cast<float>(target.width);
    const float height = static_cast<float>(target.height);
    const float x0 = lo_x < 0 ? 0 : lo_x > width ? width : lo_x;
    const float x1 = hi_x < 0 ? 0 : hi_x > width ? width : hi_x;
    const float y0 = lo_y < 0 ? 0 : lo_y > height ? height : lo_y;
    const float y1 = hi_y < 0 ? 0 : hi_y > height ? height : hi_y;
    if (!(x1 > x0) || !(y1 > y0)) return true;

    DrawState s = state;
    if (target.color2 == nullptr && s.draw_buffer == gl::front_and_back) {
        s.draw_buffer = gl::back;     // the single target the caller chose
    }
    s.viewport_x = 0;
    s.viewport_y = 0;
    s.viewport_width = static_cast<int32_t>(target.width);
    s.viewport_height = static_cast<int32_t>(target.height);
    s.cull_enabled = false;
    s.offset_enabled = false;
    s.polygon_stipple = false;
    s.line_stipple = false;
    s.two_sided_color = false;
    // Colour and coverage fragments take the raster depth exactly (a zero
    // depth range at z); depth sources export their own value.
    const bool depth_source = abi::pixel_source_depth(source.kind);
    s.depth_near = depth_source ? 0.0 : (pixels.z < 0 ? 0.0 : pixels.z > 1 ? 1.0 : pixels.z);
    s.depth_far = depth_source ? 1.0 : s.depth_near;
    if (abi::pixel_source_stencil(source.kind)) {
        s.texture = nullptr;
        s.fog_enabled = false;
        s.alpha_test = false;
    }
    PixelFragments rectangle = pixels;
    uint32_t index = 0;
    if (!encode_state(s, PrimitiveKind::triangles, &source, &rectangle, index)) return false;

    // One triangle with the rectangle in its interior (no shared edge whose
    // pixel centres two triangles could both or neither claim).
    const float nx0 = x0 / width * 2.0f - 1.0f, nx1 = (2.0f * x1 - x0) / width * 2.0f - 1.0f;
    const float ny0 = y0 / height * 2.0f - 1.0f, ny1 = (2.0f * y1 - y0) / height * 2.0f - 1.0f;
    const float corners[3][2] = {{nx0, ny0}, {nx1, ny0}, {nx0, ny1}};
    RasterVertex triangle[3];
    for (uint32_t i = 0; i < 3; ++i) {
        triangle[i].clip = {corners[i][0], corners[i][1], 0, 1};
        triangle[i].color[0] = triangle[i].color[1] = pixels.color;
        triangle[i].texcoord = pixels.texcoord;
        triangle[i].fog = pixels.fog;
    }
    if (append(index, 3, 0, triangle, 3, target)) return true;
    if (!submit_batch() || !encode_state(s, PrimitiveKind::triangles, &source, &rectangle, index)) return false;
    return append(index, 3, 0, triangle, 3, target);
}

void Gfx11Backend::draw_pixels(const DrawState& state, const PixelFragments& pixels) noexcept {
    FallbackReason why = FallbackReason::none;
    if (gpu_active_ && unsupported(state, why)) {
        migrate();
        fallback(why);
    }
    if (!gpu_active_ || pixels.width == 0 || pixels.height == 0 || !(pixels.zoom_x != 0) ||
        !(pixels.zoom_y != 0) || width_ == 0 || height_ == 0) {
        if (!gpu_active_) software_.draw_pixels(state, pixels);
        return;
    }
    // Upload the band as the PS reads it: R, G, B, A bytes, float depth, or
    // one byte per pixel (coverage, stencil), rows bottom to top.
    PixelSource source{};
    uint32_t bytes_per_pixel = 4;
    if (pixels.stencil != nullptr) {
        source.kind = abi::pixel_source_stencil8;
        bytes_per_pixel = 1;
    } else if (pixels.depth != nullptr) {
        source.kind = abi::pixel_source_depth32;
    } else if (pixels.rgba != nullptr) {
        source.kind = abi::pixel_source_rgba8;
    } else if (pixels.coverage != nullptr) {
        source.kind = abi::pixel_source_coverage8;
        bytes_per_pixel = 1;
    } else {
        return;
    }
    const uint64_t row_bytes = static_cast<uint64_t>(pixels.width) * bytes_per_pixel;
    const uint64_t total = row_bytes * pixels.height;
    uint32_t region = 0;
    if (row_bytes > pabi::bulk_bytes || total > wire::buffer_max_bytes || !upload_image(nullptr, total, region)) {
        if (gpu_active_) {
            migrate();
            fallback(FallbackReason::allocation_failed);
        }
        software_.draw_pixels(state, pixels);
        return;
    }
    uint64_t capacity = 0;
    uint8_t* staging = image_staging(capacity);
    const uint32_t rows_per_chunk = static_cast<uint32_t>(capacity / row_bytes);
    for (uint32_t row = 0; row < pixels.height; row += rows_per_chunk) {
        const uint32_t rows = pixels.height - row < rows_per_chunk ? pixels.height - row : rows_per_chunk;
        for (uint32_t r = 0; r < rows; ++r) {
            for (uint32_t column = 0; column < pixels.width; ++column) {
                const size_t from = static_cast<size_t>(row + r) * pixels.width + column;
                uint8_t* to = staging + static_cast<size_t>(r) * row_bytes + static_cast<size_t>(column) * bytes_per_pixel;
                if (source.kind == abi::pixel_source_rgba8) {
                    for (uint32_t c = 0; c < 4; ++c) {
                        const float v = pixels.rgba[from * 4 + c];
                        to[c] = static_cast<uint8_t>((v < 0 ? 0 : v > 1 ? 1 : v) * 255.0f + 0.5f);
                    }
                } else if (source.kind == abi::pixel_source_depth32) {
                    const uint32_t word = bits(pixels.depth[from]);
                    for (uint32_t b = 0; b < 4; ++b) to[b] = static_cast<uint8_t>(word >> (8 * b));
                } else if (source.kind == abi::pixel_source_stencil8) {
                    to[0] = pixels.stencil[from];
                } else {
                    to[0] = pixels.coverage[from] != 0 ? 1 : 0;
                }
            }
        }
        if (!upload(images_, region + static_cast<uint64_t>(row) * row_bytes, staging,
                    static_cast<uint64_t>(rows) * row_bytes)) {
            fallback(FallbackReason::submission_failed);
            software_.draw_pixels(state, pixels);
            return;
        }
    }
    source.buffer = &images_;
    // Image row first_row is the band's first row: offset of row 0 may wrap.
    source.stride = static_cast<int32_t>(row_bytes);
    source.offset = region - pixels.first_row * static_cast<uint32_t>(row_bytes);
    // A stencil source writes one buffer; the colour target only names it.
    const BatchTarget target = draw_target(source.kind == abi::pixel_source_stencil8 &&
                                           state.draw_buffer == gl::front_and_back ? gl::back : state.draw_buffer);
    if (state.draw_buffer == gl::front || state.draw_buffer == gl::front_and_back) front_dirty_ = true;
    if (!pixel_rectangle(state, pixels, source, target) && gpu_active_) {
        fallback(FallbackReason::submission_failed);
        software_.draw_pixels(state, pixels);
    }
}

bool Gfx11Backend::copy_pixels(const DrawState& state, const PixelCopy& copy) noexcept {
    const PixelFragments& pixels = copy.destination;
    FallbackReason why = FallbackReason::none;
    if (!gpu_active_ || unsupported(state, why) || width_ == 0 || height_ == 0 || copy.width == 0 ||
        copy.height == 0 || copy.width > wire::max_dimension || copy.height > wire::max_dimension ||
        !(pixels.zoom_x != 0) || !(pixels.zoom_y != 0)) {
        return false;
    }
    PixelSource source{};
    for (uint32_t c = 0; c < 4; ++c) {
        source.scale[c] = copy.scale[c];
        source.bias[c] = copy.bias[c];
    }
    source.shift_left = copy.index_shift > 0 ? static_cast<uint32_t>(copy.index_shift > 31 ? 31 : copy.index_shift) : 0u;
    source.shift_right = copy.index_shift < 0 ? static_cast<uint32_t>(copy.index_shift < -31 ? 31 : -copy.index_shift) : 0u;
    source.index_offset = copy.index_offset;
    const int64_t bottom_row = static_cast<int64_t>(height_) - 1 - copy.y;   // buffer row of GL row y
    if (copy.source == ReadSource::color) {
        source.kind = abi::pixel_source_bgra8;
        source.transfer = copy.scale[0] != 1 || copy.scale[1] != 1 || copy.scale[2] != 1 || copy.scale[3] != 1 ||
                          copy.bias[0] != 0 || copy.bias[1] != 0 || copy.bias[2] != 0 || copy.bias[3] != 0;
        const Buffer& read = copy.buffer == gl::front ? color_[0] : color_[1];
        const uint32_t pitch = color_pitch_bytes(width_);
        source.buffer = &read;
        source.offset = static_cast<uint32_t>(bottom_row * pitch + static_cast<int64_t>(copy.x) * 4);
        source.stride = -static_cast<int32_t>(pitch);
        const bool aliased = &read == &color_[1] ? state.draw_buffer != gl::front : state.draw_buffer != gl::back;
        if (aliased && state.draw_buffer != gl::none) {
            // GL reads the whole source before writing: copy it to a staging
            // surface of the rectangle's size first (a plain pixel draw).
            const uint64_t bytes = static_cast<uint64_t>(color_pitch_bytes(copy.width)) * copy.height;
            if (staging_.bytes < bytes) {
                if (!wait_idle()) return false;
                release(staging_);
                if (!allocate(bytes, 256, staging_)) return false;
            }
            DrawState plain{};
            plain.dither = false;
            plain.draw_buffer = gl::back;
            PixelFragments whole{};
            whole.width = copy.width;
            whole.height = copy.height;
            BatchTarget staging{};
            staging.color = &staging_;
            staging.width = copy.width;
            staging.height = copy.height;
            staging.kind = static_cast<uint32_t>(wire::TargetKind::bgra8);
            PixelSource raw = source;
            raw.transfer = false;
            if (!pixel_rectangle(plain, whole, raw, staging)) {
                if (gpu_active_) fallback(FallbackReason::submission_failed);
                return false;
            }
            const uint32_t staging_pitch = color_pitch_bytes(copy.width);
            source.buffer = &staging_;
            source.offset = (copy.height - 1u) * staging_pitch;
            source.stride = -static_cast<int32_t>(staging_pitch);
        }
    } else {
        // Depth and stencil come from the DB->CB copy (X24_8_32, top-down).
        if (!depth_copy()) return false;
        const uint32_t pitch = copy_pitch_bytes(width_);
        source.kind = copy.source == ReadSource::depth ? abi::pixel_source_depth_copy : abi::pixel_source_stencil_copy;
        source.transfer = copy.source == ReadSource::depth ? copy.scale[0] != 1 || copy.bias[0] != 0
                                                           : copy.index_shift != 0 || copy.index_offset != 0;
        source.buffer = &copy_;
        source.offset = static_cast<uint32_t>(bottom_row * pitch + static_cast<int64_t>(copy.x) * 8);
        source.stride = -static_cast<int32_t>(pitch);
    }
    if (!source.transfer) {
        for (uint32_t c = 0; c < 4; ++c) {
            source.scale[c] = 1;
            source.bias[c] = 0;
        }
        source.shift_left = source.shift_right = 0;
        source.index_offset = 0;
    }
    PixelFragments destination = pixels;
    destination.width = copy.width;
    destination.height = copy.height;
    destination.first_row = 0;
    const BatchTarget target = draw_target(abi::pixel_source_stencil(source.kind) &&
                                           state.draw_buffer == gl::front_and_back ? gl::back : state.draw_buffer);
    if (state.draw_buffer == gl::front || state.draw_buffer == gl::front_and_back) front_dirty_ = true;
    if (!pixel_rectangle(state, destination, source, target)) {
        if (gpu_active_) fallback(FallbackReason::submission_failed);
        return false;
    }
    return true;
}

bool Gfx11Backend::depth_copy() noexcept {
    // DB->CB copy of the whole depth/stencil buffer into a linear X24_8_32
    // target (radeonsi si_blit_dbcb_copy): one full-screen rectangle.
    const uint64_t bytes = static_cast<uint64_t>(copy_pitch_bytes(width_)) * height_;
    if (bytes > wire::buffer_max_bytes) {
        return false;
    }
    if (copy_.handle == 0 && !allocate(bytes, 256, copy_)) return false;
    if (!submit_batch()) return false;
    DrawState state{};
    state.viewport_width = static_cast<int32_t>(width_);
    state.viewport_height = static_cast<int32_t>(height_);
    state.draw_buffer = gl::back;
    const float corners[6][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, -1}, {1, 1}, {-1, 1}};
    RasterVertex quad[6];
    for (uint32_t i = 0; i < 6; ++i) quad[i].clip = {corners[i][0], corners[i][1], 0, 1};
    BatchTarget target{};
    target.color = &copy_;
    target.depth = &depth_;
    target.width = width_;
    target.height = height_;
    target.kind = static_cast<uint32_t>(wire::TargetKind::depth_copy);
    uint32_t index = 0;
    return encode_state(state, PrimitiveKind::triangles, nullptr, nullptr, index) &&
           append(index, 3, wire::command_depth_copy, quad, 6, target) && wait_idle();
}

void Gfx11Backend::read(ReadSource source, gl::Enum buffer, int32_t x, int32_t y, uint32_t w,
                        uint32_t h, float* output) noexcept {
    if (!gpu_active_) {
        software_.read(source, buffer, x, y, w, h, output);
        return;
    }
    if (output == nullptr || w == 0 || h == 0) return;
    if (!wait_idle()) {
        software_.read(source, buffer, x, y, w, h, output);
        return;
    }
    const bool color = source == ReadSource::color;
    if (!color && !depth_copy()) {
        migrate();
        fallback(FallbackReason::depth_readback_too_large);
        software_.read(source, buffer, x, y, w, h, output);
        return;
    }
    const Buffer& surface = color ? (buffer == gl::front ? color_[0] : color_[1]) : copy_;
    const uint32_t pitch = color ? color_pitch_bytes(width_) : copy_pitch_bytes(width_);
    const uint32_t texel = color ? 4u : 8u;
    for (uint32_t row = 0; row < h; ++row) {
        const int32_t gy = y + static_cast<int32_t>(row);
        if (gy < 0 || gy >= static_cast<int32_t>(height_)) continue;
        int32_t x0 = x < 0 ? 0 : x;
        int32_t x1 = x + static_cast<int32_t>(w);
        x1 = x1 > static_cast<int32_t>(width_) ? static_cast<int32_t>(width_) : x1;
        if (x0 >= x1) continue;
        // GL rows grow upward; the buffer stores rows top to bottom.
        const uint64_t offset = static_cast<uint64_t>(height_ - 1u - static_cast<uint32_t>(gy)) * pitch +
                                static_cast<uint64_t>(x0) * texel;
        const uint32_t count = static_cast<uint32_t>(x1 - x0) * texel;
        // The reply lands in the reply half; copy out before the next call.
        uint8_t* bytes = scratch_ + request_bytes;
        if (!readback(surface, offset, bytes, count)) {
            fallback(FallbackReason::submission_failed);
            return;
        }
        for (int32_t px = x0; px < x1; ++px) {
            const size_t out = static_cast<size_t>(row) * w + static_cast<size_t>(px - x);
            const uint8_t* p = bytes + static_cast<size_t>(px - x0) * texel;
            if (color) {
                output[out * 4 + 0] = unorm8(p[2]);
                output[out * 4 + 1] = unorm8(p[1]);
                output[out * 4 + 2] = unorm8(p[0]);
                output[out * 4 + 3] = unorm8(p[3]);
            } else if (source == ReadSource::depth) {
                uint32_t raw = static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
                               (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
                float depth = 0;
                memcpy(&depth, &raw, sizeof(depth));
                output[out] = depth;
            } else {
                output[out] = static_cast<float>(p[4]);
            }
        }
    }
}

void Gfx11Backend::migrate() noexcept {
    if (!gpu_active_ || width_ == 0 || height_ == 0 || !wait_idle()) return;
    // Copy the GPU drawable into the software renderer row by row: colour for
    // both buffers, then depth (depth test ALWAYS, colour masked) and stencil
    // (stencil-index rows: written under the full write mask). The software
    // path then continues from identical contents. Without a depth copy only
    // colour transfers.
    const uint32_t pitch = color_pitch_bytes(width_);
    uint8_t* bytes = scratch_ + request_bytes;
    DrawState base{};
    base.viewport_width = static_cast<int32_t>(width_);
    base.viewport_height = static_cast<int32_t>(height_);
    base.dither = false;
    for (uint32_t which = 0; which < 2; ++which) {
        for (uint32_t gy = 0; gy < height_; ++gy) {
            if (!readback(color_[which], static_cast<uint64_t>(height_ - 1u - gy) * pitch, bytes, width_ * 4u)) {
                return;
            }
            for (uint32_t px = 0; px < width_; ++px) {
                row_floats_[px * 4 + 0] = unorm8(bytes[px * 4 + 2]);
                row_floats_[px * 4 + 1] = unorm8(bytes[px * 4 + 1]);
                row_floats_[px * 4 + 2] = unorm8(bytes[px * 4 + 0]);
                row_floats_[px * 4 + 3] = unorm8(bytes[px * 4 + 3]);
            }
            DrawState state = base;
            state.draw_buffer = which == 0 ? gl::front : gl::back;
            PixelFragments pixels{};
            pixels.y = static_cast<float>(gy);
            pixels.width = width_;
            pixels.height = 1;
            pixels.rgba = row_floats_;
            software_.draw_pixels(state, pixels);
        }
    }
    if (!depth_copy()) return;
    const uint32_t copy_pitch = copy_pitch_bytes(width_);
    for (uint32_t gy = 0; gy < height_; ++gy) {
        if (!readback(copy_, static_cast<uint64_t>(height_ - 1u - gy) * copy_pitch, bytes, width_ * 8u)) {
            return;
        }
        for (uint32_t px = 0; px < width_; ++px) {
            const uint8_t* p = bytes + px * 8u;
            const uint32_t raw = static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
                                 (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
            memcpy(&row_depth_[px], &raw, sizeof(float));
            row_stencil_[px] = p[4];
        }
        DrawState depth = base;
        depth.depth_test = true;
        depth.depth_func = gl::always;
        depth.depth_write = true;
        for (auto& write : depth.color_write) write = false;
        PixelFragments pixels{};
        pixels.y = static_cast<float>(gy);
        pixels.width = width_;
        pixels.height = 1;
        pixels.depth = row_depth_;
        software_.draw_pixels(depth, pixels);
        PixelFragments stencil = pixels;
        stencil.depth = nullptr;
        stencil.stencil = row_stencil_;
        software_.draw_pixels(base, stencil);
    }
}

void Gfx11Backend::flush() noexcept {
    if (!gpu_active_) return;
    submit_batch();
}

void Gfx11Backend::finish() noexcept {
    if (!gpu_active_) return;
    wait_idle();
}

PresentPath Gfx11Backend::present(uint32_t* pixels) noexcept {
    if (!gpu_active_) {
        return PresentPath::software;
    }
    if (!wait_idle() || width_ == 0 || height_ == 0) {
        return gpu_active_ ? PresentPath::failed : PresentPath::software;
    }
    wire::PresentInput input{wire::present_input_bytes, wire::version, session_, color_[1].handle, width_,
                             height_};
    uint8_t encoded[wire::present_input_bytes]{};
    uint32_t outcome = 0, rects = 0;
    if (wire::encode_present_input(input, encoded, sizeof(encoded)) &&
        transport_.present(encoded, sizeof(encoded), outcome, rects) &&
        outcome == static_cast<uint32_t>(pabi::Outcome::success)) {
        const Buffer front = color_[0];
        color_[0] = color_[1];
        color_[1] = front;
        front_dirty_ = false;
        return PresentPath::gpu;
    }
    // Exact fallback: read the back buffer into a bottom-up DIB for GDI.
    if (pixels == nullptr) return PresentPath::failed;
    const uint32_t pitch = color_pitch_bytes(width_);
    uint8_t* bytes = scratch_ + request_bytes;
    const uint32_t rows_per_call = pabi::bulk_bytes / pitch;
    for (uint32_t row = 0; row < height_; row += rows_per_call) {
        const uint32_t rows = height_ - row < rows_per_call ? height_ - row : rows_per_call;
        if (!readback(color_[1], static_cast<uint64_t>(row) * pitch, bytes, rows * pitch)) {
            fallback(FallbackReason::present_failed);
            return PresentPath::failed;
        }
        for (uint32_t r = 0; r < rows; ++r) {
            // Buffer row (top-down) row + r is DIB row height-1-(row+r).
            const uint32_t dib_row = height_ - 1u - (row + r);
            const uint8_t* source = bytes + static_cast<size_t>(r) * pitch;
            for (uint32_t px = 0; px < width_; ++px) {
                const uint8_t* p = source + px * 4u;
                pixels[static_cast<size_t>(dib_row) * width_ + px] =
                    static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
                    (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
            }
        }
    }
    const Buffer front = color_[0];
    color_[0] = color_[1];
    color_[1] = front;
    return PresentPath::readback;
}

} // namespace rx7900::opengl

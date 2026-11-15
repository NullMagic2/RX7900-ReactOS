/*
Module: rx7900.reactos.process_graphics
Purpose: Decode, validate, build and lease kernel-authored GFX11 graphics work.
Created: 2026-10-23
Architecture: See process_graphics.hpp for the security model. Mirrors the
ProcessSubmissions lease rules: nothing is released on a timeout or a missing
receipt; uncertain publication quarantines and keeps every lease. The kernel
gfx-ring route (2026-10-24) follows Linux v6.12 amdgpu_cs/amdgpu_job: the IB
stays in the process VM and the ring switches to that VM (Gfx11Queues).
*/
#include "process_graphics.hpp"

#include <string.h>

namespace rx7900::reactos {
namespace {

using memory::Status;
using hardware::UserQueueStatus;
namespace gfx = opengl::gfx11;
namespace abi = opengl::gfx11_abi;
namespace wire = graphics_abi;

#if defined(_MSC_VER)
#define RX7900_GRAPHICS_NOINLINE __declspec(noinline)
#else
#define RX7900_GRAPHICS_NOINLINE __attribute__((noinline))
#endif

bool uncertain(hardware::GpuVmStatus status) noexcept {
    return status == hardware::GpuVmStatus::quarantined ||
           status == hardware::GpuVmStatus::mmio_failure;
}

float bits_float(uint32_t bits) noexcept {
    float value = 0;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

uint64_t round_up_page(uint64_t value) noexcept {
    return (value + memory::page_bytes - 1u) & ~static_cast<uint64_t>(memory::page_bytes - 1u);
}

bool usable(const ProcessBufferInfo& info, memory::Access access) noexcept {
    const uint32_t granted = static_cast<uint32_t>(info.mapped_access);
    const uint32_t needed = static_cast<uint32_t>(access);
    return info.mapped && !info.quarantined && (granted & needed) == needed;
}

gfx::RasterState raster_state(const wire::GraphicsState& s) noexcept {
    gfx::RasterState out{};
    out.viewport_x = s.viewport_x;
    out.viewport_y = s.viewport_y;
    out.viewport_width = s.viewport_width;
    out.viewport_height = s.viewport_height;
    out.depth_near = bits_float(s.depth_near);
    out.depth_far = bits_float(s.depth_far);
    out.scissor = s.scissor != 0;
    out.scissor_x = s.scissor_x;
    out.scissor_y = s.scissor_y;
    out.scissor_width = s.scissor_width;
    out.scissor_height = s.scissor_height;
    out.cull = static_cast<gfx::CullMode>(s.cull);
    out.front_ccw = s.front_ccw != 0;
    out.point_size = bits_float(s.point_size);
    out.line_width = bits_float(s.line_width);
    out.offset = s.offset != 0;
    out.offset_factor = bits_float(s.offset_factor);
    out.offset_units = bits_float(s.offset_units);
    out.depth_test = s.depth_test != 0;
    out.depth_func = static_cast<gfx::Compare>(s.depth_func);
    out.depth_write = s.depth_write != 0;
    out.stencil_test = s.stencil_test != 0;
    out.stencil_func = static_cast<gfx::Compare>(s.stencil_func);
    out.stencil_ref = s.stencil_ref;
    out.stencil_value_mask = s.stencil_value_mask;
    out.stencil_write_mask = s.stencil_write_mask;
    out.stencil_fail = static_cast<gfx::StencilOp>(s.stencil_fail);
    out.stencil_depth_fail = static_cast<gfx::StencilOp>(s.stencil_depth_fail);
    out.stencil_depth_pass = static_cast<gfx::StencilOp>(s.stencil_depth_pass);
    out.blend = s.blend != 0;
    out.blend_src = static_cast<gfx::BlendFactor>(s.blend_src);
    out.blend_dst = static_cast<gfx::BlendFactor>(s.blend_dst);
    out.logic_op = s.logic_op != 0;
    out.logic_op_code = s.logic_op_code;
    out.color_mask = s.color_mask;
    out.alpha_ref = bits_float(s.alpha_ref);
    for (uint32_t i = 0; i < 4; ++i) out.env_color[i] = bits_float(s.env_color[i]);
    for (uint32_t i = 0; i < 3; ++i) out.fog_color[i] = bits_float(s.fog_color[i]);
    if (abi::pixel_source_stencil(s.key.pixel_source)) {
        // GL 1.1 4.3.1: stencil indices pass only ownership and scissor and
        // replace the stored index under the write mask with the PS value.
        out.depth_test = false;
        out.depth_write = false;
        out.stencil_test = true;
        out.stencil_func = gfx::Compare::always;
        out.stencil_fail = out.stencil_depth_fail = out.stencil_depth_pass = gfx::StencilOp::replace;
        out.color_mask = 0;
        out.blend = false;
        out.logic_op = false;
    }
    return out;
}

void put_dword(uint8_t* output, uint32_t index, uint32_t value) noexcept {
    for (uint32_t byte = 0; byte < 4; ++byte) {
        output[index * 4u + byte] = static_cast<uint8_t>(value >> (byte * 8u));
    }
}

uint32_t float_word(float value) noexcept {
    uint32_t word = 0;
    memcpy(&word, &value, sizeof(word));
    return word;
}

} // namespace

struct ProcessGraphics::SubmitContext final {
    ProcessGraphics* self;
    uint32_t queue;
    ByteView payload;
    uint32_t sequence;
    Status status;
};

uint8_t* ProcessGraphics::job_cpu(uint32_t index) const noexcept {
    return jobs_cpu_ + static_cast<size_t>(index) * graphics_job_bytes;
}

memory::Status ProcessGraphics::map_private() noexcept {
    auto& queues = *config_.queues;
    auto& pages = config_.workspace->pages;
    const auto map_gtt = [&](memory::Handle handle, uint64_t va, uint32_t bytes, bool executable,
                             uint8_t*& cpu) -> Status {
        memory::AllocationInfo info{};
        uint64_t bus = 0;
        const uint32_t count = bytes / memory::page_bytes;
        if (config_.gtt->describe(handle, info) != Status::success || info.retiring ||
            info.backing.cpu_bytes == nullptr || info.backing.cpu_byte_count < bytes ||
            info.gpu.byte_length < bytes || count * sizeof(uint64_t) > sizeof(pages) ||
            !config_.dma_bus(config_.context, handle, bus) || (bus & (memory::page_bytes - 1u)) != 0 ||
            bus > UINT64_MAX - bytes) {
            return Status::backend_failure;
        }
        for (uint32_t page = 0; page < count; ++page) {
            const uint64_t address = bus + static_cast<uint64_t>(page) * memory::page_bytes;
            for (uint32_t byte = 0; byte < 8; ++byte) {
                pages[page * 8 + byte] = static_cast<uint8_t>(address >> (byte * 8));
            }
        }
        const hardware::GpuVmMapFlags flags{memory::Access::read, executable, hardware::GpuVmMtype::nc, false};
        const auto mapped = queues.map_system(config_.pasid, va, ByteView(pages, count * sizeof(uint64_t)),
                                              false, flags);
        if (mapped != hardware::GpuVmStatus::mapped) {
            quarantined_ = uncertain(mapped);
            return quarantined_ ? Status::quarantined : Status::backend_failure;
        }
        cpu = info.backing.cpu_bytes;
        return Status::success;
    };
    auto status = map_gtt(jobs_, graphics_job_va, graphics_jobs * graphics_job_bytes, false, jobs_cpu_);
    if (status != Status::success) return status;
    status = map_gtt(shaders_, graphics_shader_va, graphics_shader_bytes, true, shaders_cpu_);
    if (status != Status::success) return status;

    const GraphicsDevice& device = *config_.device;
    if (device.attribute_ring) {
        const uint64_t bytes = static_cast<uint64_t>(device.attribute_ring_per_se_bytes) * device.shader_engines;
        const hardware::GpuVmMapFlags flags{memory::Access::read_write, false, hardware::GpuVmMtype::nc, false};
        const auto mapped = queues.map_vram(config_.pasid, graphics_attribute_va, device.attribute_ring_mc,
                                            bytes, flags);
        if (mapped != hardware::GpuVmStatus::mapped) {
            quarantined_ = uncertain(mapped);
            return quarantined_ ? Status::quarantined : Status::backend_failure;
        }
    }
    if (device.scanout) {
        // Write-only, uncached: only kernel-built DMA_DATA rows target it.
        const uint64_t base = device.scanout_mc & ~static_cast<uint64_t>(memory::page_bytes - 1u);
        const uint64_t bytes = round_up_page((device.scanout_mc - base) +
            static_cast<uint64_t>(device.scanout_pitch_bytes) * device.scanout_height);
        const hardware::GpuVmMapFlags flags{memory::Access::write, false, hardware::GpuVmMtype::uc, true};
        if (bytes > graphics_scanout_max_bytes) return Status::invalid_argument;
        const auto mapped = queues.map_vram(config_.pasid, graphics_scanout_va, base, bytes, flags);
        if (mapped != hardware::GpuVmStatus::mapped) {
            quarantined_ = uncertain(mapped);
            return quarantined_ ? Status::quarantined : Status::backend_failure;
        }
    }
    if (queues.flush_tlb(config_.pasid, 0) != hardware::GpuVmStatus::flushed) {
        quarantined_ = true;
        return Status::quarantined;
    }
    return Status::success;
}

// Kernel-ring route: every job slot's receipt owner over the GTT Manager that
// holds the shader area (already-initialized owners are kept).
memory::Status ProcessGraphics::bind_kernel_owners() noexcept {
    for (auto& job : work_) {
        if (job.kernel.initialized()) continue;
        if (job.kernel.initialize(*config_.gtt, config_.kernel_ring) != opengl::GraphicsJobStatus::ready) {
            return Status::invalid_argument;
        }
    }
    return Status::success;
}

memory::Status ProcessGraphics::open(ProcessGraphicsConfig config, uint32_t queue) noexcept {
    if (opened_) return Status::busy;
    if (config.queues == nullptr || config.buffers == nullptr || config.gtt == nullptr ||
        config.pasid == 0 || config.dma_bus == nullptr || config.device == nullptr ||
        config.workspace == nullptr || config.device->generator.vertex == nullptr ||
        config.device->generator.pixel == nullptr || config.device->fpu.run == nullptr ||
        config.device->wait_us == nullptr || !config.device->attribute_ring ||
        config.queues->vm_report(config.pasid).pasid != config.pasid) {
        return Status::invalid_argument;
    }
    const bool kernel = queue == 0;
    if (kernel) {
        if (config.kernel_ring.submit == nullptr || config.kernel_ring.query == nullptr ||
            config.kernel_ring.retire == nullptr || config.kernel_ring.retired_after_reset == nullptr) {
            return Status::unavailable;
        }
    } else {
        const auto report = config.queues->queue_report(queue);
        if (report.queue_id != queue || report.pasid != config.pasid ||
            report.type != hardware::Mes11QueueType::gfx ||
            report.state != hardware::UserQueueState::active) {
            return Status::invalid_argument;
        }
    }
    if (quarantined_) return Status::quarantined;
    if (jobs_live_ || shaders_live_) {
        // A previous context lost its queue (or a mapping step failed); the
        // private allocations are reused and mapped once.
        if (config.queues != config_.queues || config.pasid != config_.pasid ||
            !jobs_retained_ || !shaders_retained_) return Status::busy;
        config_.kernel_ring = config.kernel_ring;
        if (kernel && bind_kernel_owners() != Status::success) return Status::invalid_argument;
        if (!mapped_) {
            const auto mapped = map_private();
            if (mapped != Status::success) return mapped;
            mapped_ = true;
        }
        queue_ = queue;
        kernel_route_ = kernel;
        opened_ = true;
        return Status::success;
    }
    config_ = config;
    auto status = config_.gtt->allocate(graphics_jobs * graphics_job_bytes, memory::page_bytes,
                                        memory::Access::read_write, true, jobs_);
    jobs_live_ = status == Status::success;
    if (!jobs_live_) return status;
    status = config_.gtt->retain(jobs_, memory::Access::read_write);
    jobs_retained_ = status == Status::success;
    if (!jobs_retained_) return status;
    status = config_.gtt->allocate(graphics_shader_bytes, memory::page_bytes,
                                   memory::Access::read_write, true, shaders_);
    shaders_live_ = status == Status::success;
    if (!shaders_live_) return status;
    status = config_.gtt->retain(shaders_, memory::Access::read_write);
    shaders_retained_ = status == Status::success;
    if (!shaders_retained_) return status;
    if (kernel && bind_kernel_owners() != Status::success) return Status::invalid_argument;
    status = map_private();
    if (status != Status::success) return status;
    mapped_ = true;
    queue_ = queue;
    kernel_route_ = kernel;
    opened_ = true;
    return Status::success;
}

ProcessGraphics::Job* ProcessGraphics::free_job() noexcept {
    for (auto& job : work_) {
        if (!job.pending) return &job;
    }
    return nullptr;
}

memory::Status ProcessGraphics::release_job(Job& job) noexcept {
    while (job.lease_count != 0) {
        if (config_.buffers->unretain(job.leases[job.lease_count - 1]) != Status::success) {
            quarantined_ = true;
            return Status::quarantined;
        }
        job.leases[--job.lease_count] = 0;
    }
    job.pending = false;
    job.sequence = 0;
    job.shader_mask = 0;
    return Status::success;
}

// The ring receipt (or a confirmed reset) is the only release; a hung ring or
// an uncertain publication keeps every lease (quarantined).
memory::Status ProcessGraphics::poll_kernel(Job& job) noexcept {
    switch (job.kernel.poll()) {
    case opengl::GraphicsJobStatus::signalled:
    case opengl::GraphicsJobStatus::ready:
        return release_job(job);
    case opengl::GraphicsJobStatus::busy:
        return Status::busy;
    default:
        quarantined_ = true;
        return Status::quarantined;
    }
}

bool ProcessGraphics::kernel_pending() const noexcept {
    if (!kernel_route_) return false;
    for (const auto& job : work_) {
        if (job.pending || job.kernel.pending()) return true;
    }
    return false;
}

memory::Status ProcessGraphics::wait_kernel_idle(uint32_t timeout_us) noexcept {
    for (uint32_t waited = 0;; waited += 50) {
        const auto status = poll();
        if (status == Status::quarantined) return status;
        if (!kernel_pending()) return Status::success;
        if (waited >= timeout_us || config_.device == nullptr) return Status::busy;
        config_.device->wait_us(config_.device->wait_context, 50);
    }
}

memory::Status ProcessGraphics::poll_job(Job& job) noexcept {
    if (!job.pending) return Status::success;
    if (kernel_route_) return poll_kernel(job);
    const auto state = config_.queues->query(queue_, job.sequence);
    if (state == UserQueueStatus::signalled) return release_job(job);
    if (state == UserQueueStatus::pending) return Status::busy;
    // A missing or hung queue is not a receipt; only removal retires leases.
    quarantined_ = true;
    return Status::quarantined;
}

memory::Status ProcessGraphics::poll() noexcept {
    if (quarantined_) return Status::quarantined;
    Status result = Status::success;
    for (auto& job : work_) {
        const auto status = poll_job(job);
        if (status == Status::quarantined) return status;
        if (status == Status::busy) result = Status::busy;
    }
    return result;
}

bool ProcessGraphics::idle() const noexcept {
    if (quarantined_) return false;
    for (const auto& job : work_) {
        if (job.pending) return false;
    }
    return true;
}

memory::Status ProcessGraphics::query(uint32_t queue, uint32_t sequence) noexcept {
    if (!opened_) return Status::unavailable;
    if (queue != queue_ || sequence == 0) return Status::invalid_argument;
    for (auto& job : work_) {
        if (job.pending && job.sequence == sequence) return poll_job(job);
    }
    // Not pending here: either already retired or never issued by this owner.
    if (kernel_route_) {
        return kernel_issued_ - sequence < 0x80000000u ? Status::success : Status::invalid_argument;
    }
    const auto report = config_.queues->queue_report(queue_);
    return report.queue_id == queue_ && sequence <= report.sync_seq ? Status::success :
                                                                       Status::invalid_argument;
}

memory::Status ProcessGraphics::lease(Job& job, uint64_t handle, memory::Access access) noexcept {
    for (uint32_t i = 0; i < job.lease_count; ++i) {
        if (job.leases[i] == handle) return Status::success;
    }
    if (job.lease_count == graphics_lease_capacity) return Status::exhausted;
    const auto status = config_.buffers->retain(handle, access);
    if (status == Status::success) job.leases[job.lease_count++] = handle;
    return status;
}

memory::Status ProcessGraphics::programs(const abi::ShaderKey& key, uint32_t job_bit,
                                         gfx::ShaderPrograms& output) noexcept {
    uint32_t pinned = 0;
    for (const auto& job : work_) {
        if (job.pending) pinned |= job.shader_mask;
    }
    ShaderSlot* chosen = nullptr;
    uint32_t chosen_index = 0;
    for (uint32_t index = 0; index < graphics_shader_slots; ++index) {
        if (slots_[index].valid && abi::keys_equal(slots_[index].key, key)) {
            chosen = &slots_[index];
            chosen_index = index;
            break;
        }
    }
    if (chosen == nullptr) {
        // Replace the least recently used slot no pending job executes.
        for (uint32_t index = 0; index < graphics_shader_slots; ++index) {
            ShaderSlot& slot = slots_[index];
            if ((pinned & (1u << index)) != 0 || (job_bit & (1u << index)) != 0) continue;
            if (chosen == nullptr || !slot.valid || (chosen->valid && slot.last_use < chosen->last_use)) {
                chosen = &slot;
                chosen_index = index;
            }
        }
        if (chosen == nullptr) return Status::busy;
        chosen->valid = false;
        uint8_t* code = shaders_cpu_ + static_cast<size_t>(chosen_index) * graphics_shader_slot_bytes;
        const auto& generator = config_.device->generator;
        abi::ProgramInfo vs{};
        abi::ProgramInfo ps{};
        if (!generator.vertex(generator.context, key, sdma::CommandBuffer(code, abi::shader_max_bytes), vs) ||
            !generator.pixel(generator.context, key, sdma::CommandBuffer(code + abi::shader_max_bytes, abi::shader_max_bytes), ps) ||
            vs.code_dwords == 0 || ps.code_dwords == 0 ||
            vs.code_dwords * 4u > abi::shader_max_bytes || ps.code_dwords * 4u > abi::shader_max_bytes) {
            return Status::invalid_argument;
        }
        chosen->key = key;
        chosen->vs = vs;
        chosen->ps = ps;
        chosen->valid = true;
    }
    chosen->last_use = ++use_clock_;
    output.key = key;
    output.vs = chosen->vs;
    output.ps = chosen->ps;
    output.vs_va = graphics_shader_va + static_cast<uint64_t>(chosen_index) * graphics_shader_slot_bytes;
    output.ps_va = output.vs_va + abi::shader_max_bytes;
    return Status::success;
}

bool ProcessGraphics::submit_body(void* argument) noexcept {
    auto& context = *static_cast<SubmitContext*>(argument);
    context.status = context.self->build_submit(context);
    return true;
}

memory::Status ProcessGraphics::submit(uint32_t queue, ByteView payload, uint32_t& sequence) noexcept {
    sequence = 0;
    if (!opened_) return Status::unavailable;
    if (queue != queue_) return Status::invalid_argument;
    const auto previous = poll();
    if (previous == Status::quarantined) return previous;
    SubmitContext context{this, queue, payload, 0, Status::invalid_argument};
    const auto& fpu = config_.device->fpu;
    if (!fpu.run(fpu.context, submit_body, &context)) return Status::unavailable;
    sequence = context.sequence;
    return context.status;
}

RX7900_GRAPHICS_NOINLINE memory::Status ProcessGraphics::emit_command(
    const wire::GraphicsCommand& command, uint64_t data_va, const gfx::DeviceRings& rings,
    uint32_t& shader_bits) noexcept {
    auto& workspace = *config_.workspace;
    const auto& s = workspace.states[command.state_index];
    gfx::ShaderPrograms& programs_in_use = workspace.programs;
    programs_in_use = {};
    const auto status = programs(s.key, shader_bits, programs_in_use);
    if (status != Status::success) return status;
    shader_bits |= 1u << ((programs_in_use.vs_va - graphics_shader_va) / graphics_shader_slot_bytes);
    gfx::DrawInputs& draw = workspace.draw;
    draw = {};
    draw.primitive = static_cast<gfx::Primitive>(command.primitive);
    draw.vertex_count = command.vertex_count;
    draw.vertex_va = data_va + static_cast<uint64_t>(command.first_vertex) * abi::clip_record_stride;
    draw.vertex_bytes = command.vertex_count * abi::clip_record_stride;
    draw.depth_copy = (command.flags & wire::command_depth_copy) != 0;
    if (abi::uses_ps_constants(s.key)) {
        draw.ps_constants_va = data_va + graphics_constants_offset +
                               static_cast<uint64_t>(command.state_index) * abi::ps_constants_bytes;
        draw.ps_constants_bytes = abi::ps_constants_bytes;
    }
    if (s.texture != 0) {
        ProcessBufferInfo& texture = workspace.info;
        config_.buffers->describe(s.texture, texture);
        draw.texture = abi::texture_descriptor(texture.va + s.texture_offset, s.texture_width,
                                               s.texture_height, s.texture_pitch);
        const uint32_t clamp_s = s.wrap_s != 0 ? abi::tex_clamp_last_texel : abi::tex_clamp_wrap;
        const uint32_t clamp_t = s.wrap_t != 0 ? abi::tex_clamp_last_texel : abi::tex_clamp_wrap;
        draw.sampler = abi::sampler_descriptor(clamp_s, clamp_t, s.filter != 0 ?
                                               abi::tex_filter_bilinear : abi::tex_filter_point);
    }
    workspace.raster = raster_state(s);
    gfx::emit_draw(workspace.builder, workspace.framebuffer, workspace.raster, programs_in_use, draw, rings);
    return Status::success;
}

// Layout-4 resources: the second colour buffer (same footprint as the first),
// texture levels 1.. inside their texture buffer and readable pixel sources.
// Every address comes from this session's own mapping records.
RX7900_GRAPHICS_NOINLINE memory::Status ProcessGraphics::validate_v4(const wire::SubmitHeader& header) noexcept {
    auto& workspace = *config_.workspace;
    gfx::Framebuffer& fb = workspace.framebuffer;
    ProcessBufferInfo& info = workspace.info;
    if (header.color2 != 0) {
        const auto status = config_.buffers->describe(header.color2, info);
        if (status != Status::success) return status;
        const uint64_t bytes = static_cast<uint64_t>(fb.color.pitch_pixels) * header.height * 4u;
        if (!usable(info, memory::Access::read_write)) return Status::access_denied;
        if (info.bytes < bytes || (info.va & 255u) != 0) return Status::invalid_argument;
        fb.dual = true;
        fb.color1 = {info.va, fb.color.pitch_pixels, gfx::TargetKind::bgra8};
    }
    for (uint32_t index = 0; index < header.state_count; ++index) {
        const auto& s = workspace.states[index];
        if (s.level_count > 1) {
            const auto status = config_.buffers->describe(s.texture, info);
            if (status != Status::success) return status;
            for (uint32_t level = 0; level + 1 < s.level_count; ++level) {
                const auto& l = s.levels[level];
                const uint64_t bytes = static_cast<uint64_t>(l.pitch) * l.height * 4u;
                if (l.offset > info.bytes || bytes > info.bytes - l.offset || ((info.va + l.offset) & 255u) != 0) {
                    return Status::invalid_argument;
                }
            }
        }
        if (s.pixel_buffer != 0) {
            const auto status = config_.buffers->describe(s.pixel_buffer, info);
            if (status != Status::success) return status;
            if (!usable(info, memory::Access::read)) return Status::access_denied;
            if (info.bytes > UINT32_MAX) return Status::invalid_argument;
        }
    }
    return Status::success;
}

// gfx11_abi v4 PS constant block (dword layout pc_*): kernel-built descriptors
// whose bases come from the session's mappings and whose extents are exact.
RX7900_GRAPHICS_NOINLINE void ProcessGraphics::write_ps_constants(const wire::GraphicsState& s, uint32_t height,
                                                                  uint8_t* out) noexcept {
    for (uint32_t index = 0; index < abi::ps_constants_dwords; ++index) {
        put_dword(out, index, 0);
    }
    const auto& k = s.key;
    put_dword(out, abi::pc_height, float_word(static_cast<float>(height)));
    if (k.line_stipple != 0) {
        put_dword(out, abi::pc_stipple_inv_factor, float_word(1.0f / static_cast<float>(s.line_stipple_factor)));
        put_dword(out, abi::pc_stipple_pattern, s.line_stipple_pattern);
    }
    for (uint32_t row = 0; row < 32; ++row) {
        put_dword(out, abi::pc_stipple_rows + row, s.polygon_stipple[row]);
    }
    ProcessBufferInfo& info = config_.workspace->info;
    if (k.min_filter != abi::texture_lod_none && config_.buffers->describe(s.texture, info) == Status::success) {
        const bool mag_linear = k.mag_filter == abi::texture_mag_linear;
        const bool threshold = mag_linear && (k.min_filter == abi::texture_min_nearest_mipmap_nearest ||
                                              k.min_filter == abi::texture_min_nearest_mipmap_linear);
        const bool min_linear = k.min_filter == abi::texture_min_linear ||
                                k.min_filter == abi::texture_min_linear_mipmap_nearest ||
                                k.min_filter == abi::texture_min_linear_mipmap_linear;
        put_dword(out, abi::pc_lod_threshold, float_word(threshold ? 0.5f : 0.0f));
        put_dword(out, abi::pc_level_count, s.level_count);
        put_dword(out, abi::pc_last_level, float_word(static_cast<float>(s.level_count - 1u)));
        put_dword(out, abi::pc_texture_width, float_word(static_cast<float>(s.texture_width)));
        put_dword(out, abi::pc_texture_height, float_word(static_cast<float>(s.texture_height)));
        const uint32_t clamp_s = s.wrap_s != 0 ? abi::tex_clamp_last_texel : abi::tex_clamp_wrap;
        const uint32_t clamp_t = s.wrap_t != 0 ? abi::tex_clamp_last_texel : abi::tex_clamp_wrap;
        const abi::SamplerDescriptor mag = abi::sampler_descriptor(
            clamp_s, clamp_t, mag_linear ? abi::tex_filter_bilinear : abi::tex_filter_point);
        const abi::SamplerDescriptor min = abi::sampler_descriptor(
            clamp_s, clamp_t, min_linear ? abi::tex_filter_bilinear : abi::tex_filter_point);
        for (uint32_t w = 0; w < 4; ++w) {
            put_dword(out, abi::pc_mag_sampler + w, mag.words[w]);
            put_dword(out, abi::pc_min_sampler + w, min.words[w]);
        }
        for (uint32_t level = 0; level < s.level_count; ++level) {
            const uint32_t offset = level == 0 ? s.texture_offset : s.levels[level - 1u].offset;
            const uint32_t width = level == 0 ? s.texture_width : s.levels[level - 1u].width;
            const uint32_t level_height = level == 0 ? s.texture_height : s.levels[level - 1u].height;
            const uint32_t pitch = level == 0 ? s.texture_pitch : s.levels[level - 1u].pitch;
            const abi::ImageDescriptor t = abi::texture_descriptor(info.va + offset, width, level_height, pitch);
            for (uint32_t w = 0; w < 8; ++w) {
                put_dword(out, abi::pc_levels + level * 8u + w, t.words[w]);
            }
        }
    }
    if (k.pixel_source != abi::pixel_source_none && config_.buffers->describe(s.pixel_buffer, info) == Status::success) {
        put_dword(out, abi::pc_raster_x, s.raster_x);
        put_dword(out, abi::pc_raster_y, s.raster_y);
        put_dword(out, abi::pc_inv_zoom_x, s.inv_zoom_x);
        put_dword(out, abi::pc_inv_zoom_y, s.inv_zoom_y);
        put_dword(out, abi::pc_columns, float_word(static_cast<float>(s.pixel_columns)));
        put_dword(out, abi::pc_first_row, float_word(static_cast<float>(s.pixel_first_row)));
        put_dword(out, abi::pc_end_row, float_word(static_cast<float>(s.pixel_end_row)));
        put_dword(out, abi::pc_image_offset, s.pixel_offset);
        put_dword(out, abi::pc_row_stride, static_cast<uint32_t>(s.pixel_row_stride));
        // Every image read is bounded to the pixel buffer by this V#.
        const abi::BufferDescriptor image = abi::raw_buffer_descriptor(info.va, static_cast<uint32_t>(info.bytes));
        for (uint32_t w = 0; w < 4; ++w) {
            put_dword(out, abi::pc_pixel_buffer + w, image.words[w]);
        }
        for (uint32_t c = 0; c < 4; ++c) {
            put_dword(out, abi::pc_scale + c, s.transfer_scale[c]);
            put_dword(out, abi::pc_bias + c, s.transfer_bias[c]);
        }
        put_dword(out, abi::pc_index_shift, s.index_shift_left);
        put_dword(out, abi::pc_index_right_shift, s.index_shift_right);
        put_dword(out, abi::pc_index_offset, static_cast<uint32_t>(s.index_offset));
    }
}

memory::Status ProcessGraphics::build_submit(SubmitContext& c) noexcept {
    Job* job = free_job();
    if (job == nullptr) return Status::busy;
    const uint32_t job_index = static_cast<uint32_t>(job - work_);
    auto& workspace = *config_.workspace;

    wire::SubmitHeader& header = workspace.header;
    if (!wire::decode_header(c.payload, header) ||
        c.payload.size() != wire::submit_payload_bytes(header)) {
        return Status::invalid_argument;
    }
    // Layout 3 (48-byte header) or layout 4 (64-byte header, v4 states).
    const bool layout4 = header.header_bytes == wire::submit_header_v4_bytes;
    size_t offset = header.header_bytes;
    for (uint32_t index = 0; index < header.state_count; ++index, offset += wire::submit_state_bytes(header)) {
        if (!(layout4 ? wire::decode_state_v4(c.payload, offset, workspace.states[index])
                      : wire::decode_state(c.payload, offset, workspace.states[index]))) {
            return Status::invalid_argument;
        }
    }
    for (uint32_t index = 0; index < header.command_count; ++index, offset += wire::command_bytes) {
        auto& command = workspace.commands[index];
        if (!wire::decode_command(c.payload, offset, header.state_count, header.vertex_bytes, command)) {
            return Status::invalid_argument;
        }
        // One submission targets one framebuffer: GL_FRONT_AND_BACK states and
        // the second colour buffer go together; depth/stencil pixel sources
        // write the depth buffer.
        const auto& key = workspace.states[command.state_index].key;
        if (key.vertices_per_primitive != command.primitive ||
            (((command.flags & wire::command_depth_copy) != 0) !=
             (header.target_kind == static_cast<uint32_t>(wire::TargetKind::depth_copy))) ||
            ((key.front_and_back != 0) != (header.color2 != 0)) ||
            ((abi::pixel_source_depth(key.pixel_source) || abi::pixel_source_stencil(key.pixel_source)) &&
             header.depth == 0)) {
            return Status::invalid_argument;
        }
    }
    const size_t vertex_offset = offset;

    // Surfaces: exact footprints inside this session's own mapped buffers.
    gfx::Framebuffer& fb = workspace.framebuffer;
    fb = {};
    fb.width = header.width;
    fb.height = header.height;
    fb.color.kind = header.target_kind == 0 ? gfx::TargetKind::bgra8 : gfx::TargetKind::depth_copy;
    fb.color.pitch_pixels = gfx::linear_pitch_pixels(header.width, fb.color.kind);
    ProcessBufferInfo& color = workspace.info;
    auto status = config_.buffers->describe(header.color, color);
    if (status != Status::success) return status;
    const uint64_t color_bytes = static_cast<uint64_t>(fb.color.pitch_pixels) * header.height *
                                 gfx::target_bytes_per_pixel(fb.color.kind);
    if (!usable(color, memory::Access::read_write)) return Status::access_denied;
    if (color.bytes < color_bytes || (color.va & 255u) != 0) return Status::invalid_argument;
    fb.color.va = color.va;
    if (header.depth != 0) {
        ProcessBufferInfo& depth = workspace.info;
        gfx::DepthLayout layout{};
        status = config_.buffers->describe(header.depth, depth);
        if (status != Status::success) return status;
        if (!usable(depth, memory::Access::read_write)) return Status::access_denied;
        if (!gfx::depth_layout(header.width, header.height, layout) || depth.bytes < layout.total_bytes ||
            (depth.va & (gfx::depth_block_bytes - 1u)) != 0 || header.depth == header.color) {
            return Status::invalid_argument;
        }
        fb.depth.present = true;
        fb.depth.z_va = depth.va;
        fb.depth.stencil_va = depth.va + layout.stencil_offset;
    }

    // Textures are checked before any lease or shader change.
    for (uint32_t index = 0; index < header.state_count; ++index) {
        const auto& s = workspace.states[index];
        if (s.texture == 0) continue;
        ProcessBufferInfo& texture = workspace.info;
        status = config_.buffers->describe(s.texture, texture);
        if (status != Status::success) return status;
        const uint64_t bytes = static_cast<uint64_t>(s.texture_pitch) * s.texture_height * 4u;
        if (!usable(texture, memory::Access::read)) return Status::access_denied;
        if (s.texture_offset > texture.bytes || bytes > texture.bytes - s.texture_offset ||
            ((texture.va + s.texture_offset) & 255u) != 0) {
            return Status::invalid_argument;
        }
    }

    if (layout4) {
        status = validate_v4(header);
        if (status != Status::success) return status;
    }

    // Copy vertex records into the driver-owned, read-only-mapped data area.
    uint8_t* data = job_cpu(job_index) + graphics_ib_bytes;
    for (uint32_t byte = 0; byte < header.vertex_bytes; byte += 4) {
        uint32_t word = 0;
        if (!c.payload.read_u32(vertex_offset + byte, word)) return Status::invalid_argument;
        for (uint32_t shift = 0; shift < 4; ++shift) {
            data[byte + shift] = static_cast<uint8_t>(word >> (shift * 8));
        }
    }
    // gfx11_abi v4 PS constant blocks, built here from the validated states.
    for (uint32_t index = 0; index < header.state_count; ++index) {
        if (abi::uses_ps_constants(workspace.states[index].key)) {
            write_ps_constants(workspace.states[index], header.height,
                               data + graphics_constants_offset + static_cast<size_t>(index) * abi::ps_constants_bytes);
        }
    }

    // Build the IB.
    const uint64_t job_va = graphics_job_va + static_cast<uint64_t>(job_index) * graphics_job_bytes;
    const uint64_t data_va = job_va + graphics_ib_bytes;
    gfx::Pm4Builder& b = workspace.builder;
    b.begin(sdma::CommandBuffer(job_cpu(job_index), graphics_ib_bytes));
    const GraphicsDevice& device = *config_.device;
    const gfx::DeviceRings rings{graphics_attribute_va, device.attribute_ring_per_se_bytes,
                                 device.shader_engines};
    gfx::emit_preamble(b, rings);
    gfx::emit_framebuffer(b, fb);
    uint32_t shader_bits = 0;
    for (uint32_t index = 0; index < header.command_count && b.ok(); ++index) {
        status = emit_command(workspace.commands[index], data_va, rings, shader_bits);
        if (status != Status::success) return status;
    }
    gfx::emit_end_of_ib(b);
    if (!b.ok()) {
        return b.status() == gfx::StateStatus::output_too_small ? Status::exhausted :
                                                                  Status::invalid_argument;
    }

    // Lease every referenced buffer, then publish.
    status = lease(*job, header.color, memory::Access::read_write);
    if (status == Status::success && header.depth != 0) {
        status = lease(*job, header.depth, memory::Access::read_write);
    }
    if (status == Status::success && header.color2 != 0) {
        status = lease(*job, header.color2, memory::Access::read_write);
    }
    for (uint32_t index = 0; status == Status::success && index < header.state_count; ++index) {
        if (workspace.states[index].texture != 0) {
            status = lease(*job, workspace.states[index].texture, memory::Access::read);
        }
        if (status == Status::success && workspace.states[index].pixel_buffer != 0) {
            status = lease(*job, workspace.states[index].pixel_buffer, memory::Access::read);
        }
    }
    if (status != Status::success) {
        const auto released = release_job(*job);
        return released == Status::success ? status : released;
    }
    job->shader_mask = shader_bits;
    return publish(*job, job_index, b.dwords(), c.sequence);
}

// A process-VM job on the kernel gfx ring: the IB at its private VA, the shader
// area leased by the slot's receipt owner, the job area by the ring itself.
memory::Status ProcessGraphics::publish_kernel(Job& job, uint32_t job_index, uint32_t dwords,
                                               uint32_t& sequence) noexcept {
    const auto vm = config_.queues->vm_report(config_.pasid);
    if (vm.pasid != config_.pasid || vm.pd_address == 0) {
        const auto released = release_job(job);
        return released == Status::success ? Status::unavailable : released;
    }
    // Device-owned scratch (service mutex); every field the owner reads is set.
    opengl::TrustedGraphicsJob& work = config_.workspace->kernel_job;
    work.ib = jobs_;
    work.ib_offset = static_cast<uint64_t>(job_index) * graphics_job_bytes;
    work.ib_dwords = dwords;
    work.resources[0] = {{shaders_, 0}, graphics_shader_bytes, memory::Access::read};
    work.resource_count = 1;
    work.in_vm = true;
    work.vm = {config_.pasid, vm.pd_address, graphics_job_va + work.ib_offset};
    uint32_t ring_sequence = 0;
    const auto result = job.kernel.submit(work, ring_sequence);
    if (result == opengl::GraphicsJobStatus::submitted || result == opengl::GraphicsJobStatus::quarantined) {
        // Session-local, monotonic and never 0 (the ring numbering restarts
        // after a reset; this one does not).
        if (++kernel_issued_ == 0) ++kernel_issued_;
        job.pending = true;
        job.sequence = kernel_issued_;
        if (result == opengl::GraphicsJobStatus::quarantined) {
            // The CP may still run it: every lease stays until the reset proof.
            quarantined_ = true;
            return Status::quarantined;
        }
        sequence = kernel_issued_;
        return Status::success;
    }
    const auto released = release_job(job);
    if (released != Status::success) return released;
    return result == opengl::GraphicsJobStatus::busy ? Status::busy : Status::unavailable;
}

memory::Status ProcessGraphics::publish(Job& job, uint32_t job_index, uint32_t dwords,
                                        uint32_t& sequence) noexcept {
    if (kernel_route_) return publish_kernel(job, job_index, dwords, sequence);
    const uint64_t ib_va = graphics_job_va + static_cast<uint64_t>(job_index) * graphics_job_bytes;
    uint32_t submitted = 0;
    const auto result = config_.queues->submit(queue_, {ib_va, dwords}, submitted);
    if (result == UserQueueStatus::submitted && submitted != 0) {
        job.pending = true;
        job.sequence = submitted;
        sequence = submitted;
        return Status::success;
    }
    if (result == UserQueueStatus::hung || result == UserQueueStatus::quarantined) {
        // WPTR/doorbell failure may follow publication: keep every lease.
        job.pending = true;
        job.sequence = submitted;
        quarantined_ = true;
        return Status::quarantined;
    }
    const auto released = release_job(job);
    if (released != Status::success) return released;
    return result == UserQueueStatus::ring_full || result == UserQueueStatus::busy ||
           result == UserQueueStatus::suspended ? Status::busy : Status::unavailable;
}

memory::Status ProcessGraphics::present(const wire::PresentRequest& request, uint32_t& copied) noexcept {
    copied = 0;
    const GraphicsDevice& device = *config_.device;
    if (!opened_ || !device.scanout) return Status::unavailable;
    const auto previous = poll();
    if (previous == Status::quarantined) return previous;
    ProcessBufferInfo buffer{};
    auto status = config_.buffers->describe(request.input.buffer, buffer);
    if (status != Status::success) return status;
    const uint32_t pitch_bytes = gfx::linear_pitch_pixels(request.input.width, gfx::TargetKind::bgra8) * 4u;
    if (!usable(buffer, memory::Access::read)) return Status::access_denied;
    if (buffer.bytes < static_cast<uint64_t>(pitch_bytes) * request.input.height) {
        return Status::invalid_argument;
    }
    if (buffer.leases != 0) return Status::busy;     // rendering not yet retired
    Job* job = free_job();
    if (job == nullptr) return Status::busy;
    const uint32_t job_index = static_cast<uint32_t>(job - work_);
    gfx::Pm4Builder& b = config_.workspace->builder;
    b.begin(sdma::CommandBuffer(job_cpu(job_index), graphics_ib_bytes));
    const uint64_t page_offset = device.scanout_mc & (memory::page_bytes - 1u);
    const int64_t window_left = request.window_left;
    const int64_t window_top = request.window_top;
    for (uint32_t index = 0; index < request.rect_count; ++index) {
        const auto& rect = request.rects[index];
        // Intersect with the rendered drawable and with the scanout.
        int64_t left = rect.left > window_left ? rect.left : window_left;
        int64_t top = rect.top > window_top ? rect.top : window_top;
        int64_t right = rect.right < window_left + request.input.width ? rect.right :
                        window_left + request.input.width;
        int64_t bottom = rect.bottom < window_top + request.input.height ? rect.bottom :
                         window_top + request.input.height;
        left = left < 0 ? 0 : left;
        top = top < 0 ? 0 : top;
        right = right > device.scanout_width ? device.scanout_width : right;
        bottom = bottom > device.scanout_height ? device.scanout_height : bottom;
        if (left >= right || top >= bottom) continue;
        gfx::RowCopy copy{};
        copy.source_va = buffer.va;
        copy.source_pitch_bytes = pitch_bytes;
        copy.source_x = static_cast<uint32_t>(left - window_left);
        copy.source_y = static_cast<uint32_t>(top - window_top);
        copy.destination_va = graphics_scanout_va + page_offset;
        copy.destination_pitch_bytes = device.scanout_pitch_bytes;
        copy.destination_x = static_cast<uint32_t>(left);
        copy.destination_y = static_cast<uint32_t>(top);
        copy.width = static_cast<uint32_t>(right - left);
        copy.height = static_cast<uint32_t>(bottom - top);
        if (static_cast<uint64_t>(b.dwords()) * 4u +
                static_cast<uint64_t>(copy.height) * gfx::dma_data_dwords * 4u > graphics_ib_bytes) {
            return Status::exhausted;   // more rows than one IB: the ICD falls back
        }
        gfx::emit_row_copy(b, copy);
        ++copied;
    }
    if (!b.ok()) return Status::invalid_argument;
    if (copied == 0) return Status::success;
    status = lease(*job, request.input.buffer, memory::Access::read);
    if (status != Status::success) {
        copied = 0;
        const auto released = release_job(*job);
        return released == Status::success ? status : released;
    }
    uint32_t sequence = 0;
    status = publish(*job, job_index, b.dwords(), sequence);
    if (status != Status::success) {
        copied = 0;
        return status;
    }
    // win32k holds USER/DC locks: finish the read while the clip is current.
    for (uint32_t waited = 0; waited < graphics_present_timeout_us; waited += 50) {
        config_.queues->poll();
        status = poll_job(*job);
        if (status != Status::busy) return status;
        device.wait_us(device.wait_context, 50);
    }
    return Status::busy;   // still pending: lease retained until the fence
}

bool ProcessGraphics::ready_for_reset_rebind() const noexcept {
    return idle() && (!jobs_live_ || jobs_retained_) && (!shaders_live_ || shaders_retained_);
}

// The confirmed-reset window: each slot's receipt owner returns its shader
// lease on the device owner's reset proof (KernelGraphicsBridge).
memory::Status ProcessGraphics::retire_kernel_after_reset() noexcept {
    for (auto& job : work_) {
        if (!job.kernel.initialized() || !job.kernel.pending()) continue;
        if (job.kernel.retire_after_reset() != opengl::GraphicsJobStatus::signalled) {
            return Status::quarantined;
        }
    }
    return Status::success;
}

memory::Status ProcessGraphics::retire_after_reset(uint32_t& queue,
                                                   uint32_t (&sequences)[graphics_jobs]) noexcept {
    queue = queue_;
    if (kernel_route_) {
        const auto retired = retire_kernel_after_reset();
        if (retired != Status::success) return retired;
    }
    for (uint32_t index = 0; index < graphics_jobs; ++index) {
        sequences[index] = work_[index].pending ? work_[index].sequence : 0;
        if (!work_[index].pending) continue;
        const auto released = release_job(work_[index]);
        if (released != Status::success) return released;
    }
    quarantined_ = false;
    mapped_ = false;
    config_.pasid = 0;
    return Status::success;
}

void ProcessGraphics::abandon_rebind() noexcept {
    // The caller destroyed the VM the failed rebind mapped into; a failed
    // rebind never published work, so no job can be pending here.
    for (const auto& job : work_) {
        if (job.pending) return;
    }
    quarantined_ = false;
    mapped_ = false;
    config_.pasid = 0;
}

memory::Status ProcessGraphics::rebind_after_reset(ProcessGraphicsConfig config, uint32_t queue) noexcept {
    if (!ready_for_reset_rebind()) return Status::unavailable;
    if (!jobs_live_ && !shaders_live_) return Status::success;
    if (config.buffers != config_.buffers || config.gtt != config_.gtt ||
        config.queues == nullptr || config.pasid == 0) {
        return Status::unavailable;
    }
    config_ = config;
    mapped_ = false;
    // GTT contents survive the reset; regenerating on next use avoids trusting them.
    for (auto& slot : slots_) slot.valid = false;
    const auto status = map_private();
    if (status != Status::success) {
        quarantined_ = true;
        opened_ = false;
        return status;
    }
    mapped_ = true;
    queue_ = queue;
    // The kernel-ring route needs no queue: it follows its rebuilt VM.
    opened_ = queue != 0 || kernel_route_;
    return Status::success;
}

memory::Status ProcessGraphics::retire_removed_queue(uint32_t queue) noexcept {
    if (queue != queue_ || queue == 0) return Status::success;
    if (config_.queues->queue_report(queue).queue_id != 0) return Status::busy;
    for (auto& job : work_) {
        const auto released = release_job(job);
        if (released != Status::success) return released;
    }
    quarantined_ = false;
    opened_ = false;
    queue_ = 0;
    return Status::success;
}

memory::Status ProcessGraphics::close_after_idle() noexcept {
    if (empty()) {
        config_ = {};
        return Status::success;
    }
    if (config_.queues == nullptr || config_.queues->vm_report(config_.pasid).pasid != 0) {
        return Status::busy;
    }
    // A destroyed VM proves no MES queue runs; it proves nothing about the
    // kernel gfx ring, whose receipt (or reset proof) must come first.
    if (kernel_route_) {
        poll();
        if (kernel_pending()) return quarantined_ ? Status::quarantined : Status::busy;
    }
    for (auto& job : work_) {
        const auto released = release_job(job);
        if (released != Status::success) return released;
        if (job.kernel.initialized() && job.kernel.close_idle() != opengl::GraphicsJobStatus::ready) {
            return Status::quarantined;
        }
    }
    const auto drop = [&](memory::Handle& handle, bool& retained, bool& live) -> bool {
        if (retained) {
            if (config_.gtt->unretain(handle) != Status::success) return false;
            retained = false;
        }
        if (live) {
            if (config_.gtt->release(handle) != Status::success) return false;
            live = false;
            handle = {};
        }
        return true;
    };
    if (!drop(jobs_, jobs_retained_, jobs_live_) || !drop(shaders_, shaders_retained_, shaders_live_)) {
        return Status::quarantined;
    }
    jobs_cpu_ = nullptr;
    shaders_cpu_ = nullptr;
    opened_ = false;
    mapped_ = false;
    quarantined_ = false;
    kernel_route_ = false;
    // Only session teardown closes a context: the next owner of this slot
    // numbers its kernel-ring work from 1 again.
    kernel_issued_ = 0;
    queue_ = 0;
    for (auto& slot : slots_) slot = {};
    config_ = {};
    return Status::success;
}

} // namespace rx7900::reactos

/*
Module: rx7900.hardware.radeon_2d_backend
Purpose: Implement the Radeon 2D Blit2dBackend callbacks on Blit2dEngine over the
SDMA 6.0 kernel ring, GTT staging/IB memory and GPU-only VRAM surfaces.
Created: 2026-10-22
Architecture: Allocation-free; see radeon_2d_backend.hpp for the contract mapping,
fence model, coherence argument and fail-closed rules.
*/
#if defined(RX7900_RADEON2D_VIDEOPORT)
// The dispatcher header's VideoPort glue needs the native SDK types first.
#include "video_sdk.hpp"
#endif
#include "radeon_2d_backend.hpp"

namespace rx7900::hardware {
namespace {

using reactos::radeon2d::BackendStatus;
using reactos::radeon2d::CopyRect;
using reactos::radeon2d::EngineRect;
using reactos::radeon2d::EngineSurface;
using reactos::radeon2d::StagingBuffer;
using reactos::radeon2d::TransferRect;
using reactos::radeon2d::RasterAlpha;
using reactos::radeon2d::RasterGradient;
using reactos::radeon2d::RasterLine;
using reactos::radeon2d::RasterOperand;
using reactos::radeon2d::RasterRop;
namespace abi = reactos::radeon2d::abi;

// Constant-initialized (no static constructor): count 0 means unclipped.
constexpr Blit2dClipList unclipped{};

// Backend allocation ids: slot index + 1 in the low byte (0 is never valid),
// a 24-bit generation above it, so a stale id never names a reused slot.
constexpr uint32_t id_index_mask = 0xffu;
constexpr uint32_t id_generation_shift = 8;
constexpr uint32_t id_generation_limit = 0x00ffffffu;
static_assert(radeon2d_max_surfaces <= id_index_mask && radeon2d_max_staging <= id_index_mask,
              "slot indices fit the id low byte");

constexpr uint32_t make_id(uint32_t index, uint32_t generation) noexcept {
    return (index + 1u) | (generation << id_generation_shift);
}

constexpr uint32_t next_generation(uint32_t generation) noexcept {
    return generation >= id_generation_limit ? 1u : generation + 1u;
}

bool id_slot(uint32_t id, uint32_t limit, uint32_t& index, uint32_t& generation) noexcept {
    const uint32_t encoded = id & id_index_mask;

    if (encoded == 0 || encoded > limit) {
        return false;
    }

    index = encoded - 1u;
    generation = id >> id_generation_shift;
    return true;
}

uint64_t round_page(uint64_t bytes) noexcept {
    return (bytes + (memory::page_bytes - 1)) & ~(memory::page_bytes - 1);
}

bool fault_status(Blit2dStatus status) noexcept {
    return status == Blit2dStatus::faulted || status == Blit2dStatus::submit_failed ||
           status == Blit2dStatus::fence_timeout || status == Blit2dStatus::fence_failed;
}

// In-surface coordinates and extents are at most blit2d_max_surface_dimension,
// so the int32 rectangle cannot overflow; anything larger is refused.
bool to_rect(uint32_t x, uint32_t y, uint32_t width, uint32_t height, Blit2dRect& output) noexcept {
    constexpr uint32_t limit = blit2d_max_surface_dimension;

    if (width == 0 || height == 0 || x > limit || y > limit || width > limit || height > limit ||
        x + width > limit || y + height > limit) {
        return false;
    }

    output = {static_cast<int32_t>(x), static_cast<int32_t>(y), static_cast<int32_t>(x + width),
              static_cast<int32_t>(y + height)};
    return true;
}

Blit2dRect united(const Blit2dRect& first, const Blit2dRect& second) noexcept {
    return {first.left < second.left ? first.left : second.left,
            first.top < second.top ? first.top : second.top,
            first.right > second.right ? first.right : second.right,
            first.bottom > second.bottom ? first.bottom : second.bottom};
}

Radeon2dBackend& self(void* context) noexcept {
    return *static_cast<Radeon2dBackend*>(context);
}

BackendStatus fill_trampoline(void* context, const EngineSurface& target, uint32_t color,
                              const EngineRect* rects, uint32_t count, uint64_t& fence) noexcept {
    return self(context).fill(target, color, rects, count, fence);
}

BackendStatus copy_trampoline(void* context, const EngineSurface& target, const EngineSurface& source,
                              const CopyRect* rects, uint32_t count, uint64_t& fence) noexcept {
    return self(context).copy(target, source, rects, count, fence);
}

BackendStatus upload_trampoline(void* context, const EngineSurface& target, const StagingBuffer& staging,
                                uint32_t pitch, const TransferRect* rects, uint32_t count,
                                uint64_t& fence) noexcept {
    return self(context).transfer(true, target, staging, pitch, rects, count, fence);
}

BackendStatus readback_trampoline(void* context, const EngineSurface& source, const StagingBuffer& staging,
                                  uint32_t pitch, const TransferRect* rects, uint32_t count,
                                  uint64_t& fence) noexcept {
    return self(context).transfer(false, source, staging, pitch, rects, count, fence);
}

BackendStatus surface_create_trampoline(void* context, uint32_t width, uint32_t height, uint32_t format,
                                        EngineSurface& output, uint32_t& allocation) noexcept {
    return self(context).surface_create(width, height, format, output, allocation);
}

void surface_destroy_trampoline(void* context, uint32_t allocation) noexcept {
    self(context).surface_destroy(allocation);
}

BackendStatus staging_create_trampoline(void* context, uint32_t bytes, StagingBuffer& output) noexcept {
    return self(context).staging_create(bytes, output);
}

void staging_destroy_trampoline(void* context, const StagingBuffer& staging) noexcept {
    self(context).staging_destroy(staging);
}

BackendStatus sync_trampoline(void* context, uint64_t fence) noexcept {
    return self(context).sync(fence);
}

uint64_t completed_trampoline(void* context) noexcept {
    return self(context).completed();
}

BackendStatus surface_evict_trampoline(void* context, uint32_t allocation, EngineSurface& output) noexcept {
    return self(context).surface_evict(allocation, output);
}

BackendStatus surface_restore_trampoline(void* context, uint32_t allocation, EngineSurface& output) noexcept {
    return self(context).surface_restore(allocation, output);
}

BackendStatus raster_rop_trampoline(void* context, const RasterRop& operation, const EngineRect* rects,
                                    uint32_t count, uint64_t& fence) noexcept {
    return self(context).raster_rop(operation, rects, count, fence);
}

BackendStatus raster_alpha_trampoline(void* context, const RasterAlpha& operation, const EngineRect* rects,
                                      uint32_t count, uint64_t& fence) noexcept {
    return self(context).raster_alpha(operation, rects, count, fence);
}

BackendStatus raster_line_trampoline(void* context, const RasterLine& operation, const EngineRect* rects,
                                     uint32_t count, uint64_t& fence) noexcept {
    return self(context).raster_line(operation, rects, count, fence);
}

BackendStatus raster_gradient_trampoline(void* context, const RasterGradient& operation,
                                         const EngineRect* rects, uint32_t count, uint64_t& fence) noexcept {
    return self(context).raster_gradient(operation, rects, count, fence);
}

bool raster_fault_status(Raster2dStatus status) noexcept {
    return status == Raster2dStatus::invalid_memory || status == Raster2dStatus::faulted ||
           status == Raster2dStatus::submit_failed || status == Raster2dStatus::fence_timeout ||
           status == Raster2dStatus::fence_failed;
}

// A rectangle of a 32-bpp surface as a hazard region.
Blit2dRegion surface_region(const EngineSurface& surface, const Blit2dRect& rect) noexcept {
    return {surface.gpu_address, surface.pitch_bytes, surface.height, rect};
}

// Every byte of pitch * height (operands of any format, staging buffers).
Blit2dRegion span_region(uint64_t base, uint32_t pitch, uint32_t height) noexcept {
    // Byte-addressed operands may have a pitch that is not a multiple of 4.
    return {base, pitch, height, {0, 0, static_cast<int32_t>((pitch + 3u) / 4u), static_cast<int32_t>(height)}};
}

// The surface rectangle a SourceMap reads, clamped to the surface (pixels
// outside it read as zero and touch no memory).
Blit2dRect mapped_rect(const reactos::radeon2d::abi::SourceMap& map, const EngineSurface& surface) noexcept {
    const int64_t left = map.left < 0 ? 0 : map.left;
    const int64_t top = map.top < 0 ? 0 : map.top;
    int64_t right = static_cast<int64_t>(map.left) + map.width;
    int64_t bottom = static_cast<int64_t>(map.top) + map.height;
    right = right > surface.width ? surface.width : right;
    bottom = bottom > surface.height ? surface.height : bottom;
    if (right <= left || bottom <= top) {
        return {0, 0, 0, 0};
    }

    return {static_cast<int32_t>(left), static_cast<int32_t>(top), static_cast<int32_t>(right),
            static_cast<int32_t>(bottom)};
}

constexpr bool rop3_uses_destination(uint32_t rop3) noexcept {
    return ((rop3 ^ (rop3 >> 1)) & 0x55u) != 0;
}

} // namespace

BackendStatus radeon2d_backend_status(Blit2dStatus status) noexcept {
    if (status == Blit2dStatus::success) {
        return BackendStatus::ok;
    }

    return fault_status(status) ? BackendStatus::uncertain : BackendStatus::rejected;
}

Blit2dSubmitResult radeon2d_submit_result(Sdma6Status status) noexcept {
    switch (status) {
    case Sdma6Status::ready:
    case Sdma6Status::started:
        return Blit2dSubmitResult::submitted;
    case Sdma6Status::ring_full:
    case Sdma6Status::busy:
        return Blit2dSubmitResult::busy;
    default:
        return Blit2dSubmitResult::failed;
    }
}

Blit2dFenceState radeon2d_query_state(FenceStatus status) noexcept {
    switch (status) {
    case FenceStatus::signalled: return Blit2dFenceState::signalled;
    case FenceStatus::pending: return Blit2dFenceState::pending;
    default: return Blit2dFenceState::failed;
    }
}

Blit2dFenceState radeon2d_wait_state(FenceStatus status) noexcept {
    switch (status) {
    case FenceStatus::signalled: return Blit2dFenceState::signalled;
    case FenceStatus::timed_out: return Blit2dFenceState::pending;
    default: return Blit2dFenceState::failed;
    }
}

Raster2dSubmitResult radeon2d_compute_submit_result(Gfx11QueuesStatus status) noexcept {
    switch (status) {
    case Gfx11QueuesStatus::ready:
    case Gfx11QueuesStatus::running:
        return Raster2dSubmitResult::submitted;
    case Gfx11QueuesStatus::ring_full:
    case Gfx11QueuesStatus::busy:
        return Raster2dSubmitResult::busy;
    default:
        return Raster2dSubmitResult::failed;
    }
}

BackendStatus radeon2d_raster_status(Raster2dStatus status) noexcept {
    if (status == Raster2dStatus::success) {
        return BackendStatus::ok;
    }

    return raster_fault_status(status) ? BackendStatus::uncertain : BackendStatus::rejected;
}

bool radeon2d_compute_ib_offset(uint64_t value, uint32_t ib_bytes, uint32_t ib_count,
                                uint64_t& offset) noexcept {
    offset = 0;
    const uint64_t total = uint64_t{ib_bytes} * ib_count;

    if (ib_bytes == 0 || ib_count == 0 || ib_count > 4) {
        return false;
    }

    if (value < total && value % ib_bytes == 0) {
        offset = value;
        return true;
    }

    return false;
}

Raster2dSubmitResult Gfx11Raster2dSubmitter::submit(void* context, memory::Handle ib, uint64_t ib_offset,
                                                    uint32_t ib_dwords, uint32_t& sequence) noexcept {
    const auto& adapter = *static_cast<Gfx11Raster2dSubmitter*>(context);
    uint64_t offset = 0;
    sequence = 0;

    if (!radeon2d_compute_ib_offset(ib_offset, adapter.ib_bytes, adapter.ib_count, offset)) {
        return Raster2dSubmitResult::failed;
    }

    return radeon2d_compute_submit_result(
        adapter.queues->submit({adapter.ring, ib, offset, ib_dwords}, sequence));
}

Raster2dFenceState Gfx11Raster2dSubmitter::query(void* context, uint32_t sequence) noexcept {
    const auto& adapter = *static_cast<Gfx11Raster2dSubmitter*>(context);
    switch (adapter.fences->query(adapter.ring, sequence)) {
    case FenceStatus::signalled: return Raster2dFenceState::signalled;
    case FenceStatus::pending: return Raster2dFenceState::pending;
    default: return Raster2dFenceState::failed;
    }
}

Raster2dFenceState Gfx11Raster2dSubmitter::wait(void* context, uint32_t sequence, uint64_t timeout_us) noexcept {
    const auto& adapter = *static_cast<Gfx11Raster2dSubmitter*>(context);
    switch (adapter.fences->wait(adapter.ring, sequence, timeout_us)) {
    case FenceStatus::signalled: return Raster2dFenceState::signalled;
    case FenceStatus::timed_out: return Raster2dFenceState::pending;
    default: return Raster2dFenceState::failed;
    }
}

Blit2dSubmitResult Sdma6Blit2dSubmitter::submit(void* context, memory::Handle ib, uint64_t ib_offset,
                                                uint32_t ib_dwords, uint32_t& sequence) noexcept {
    const auto& adapter = *static_cast<Sdma6Blit2dSubmitter*>(context);
    sequence = 0;
    return radeon2d_submit_result(
        adapter.sdma->submit({adapter.instance, ib, ib_offset, ib_dwords}, sequence));
}

Blit2dFenceState Sdma6Blit2dSubmitter::query(void* context, uint32_t sequence) noexcept {
    const auto& adapter = *static_cast<Sdma6Blit2dSubmitter*>(context);
    return radeon2d_query_state(adapter.fences->query(fence_ring_sdma0 + adapter.instance, sequence));
}

Blit2dFenceState Sdma6Blit2dSubmitter::wait(void* context, uint32_t sequence,
                                            uint64_t timeout_us) noexcept {
    const auto& adapter = *static_cast<Sdma6Blit2dSubmitter*>(context);
    return radeon2d_wait_state(
        adapter.fences->wait(fence_ring_sdma0 + adapter.instance, sequence, timeout_us));
}

Radeon2dBackendConfig radeon2d_default_config(Sdma6& sdma, FenceManager& fences, memory::Manager& gtt,
                                              VramAllocator& vram, uint64_t vram_mc_base,
                                              Gfx11Queues* queues) noexcept {
    return {&sdma, &fences, &gtt, &vram, vram_mc_base, radeon2d_sdma_instance,
            radeon2d_default_ib_count, radeon2d_default_ib_bytes, radeon2d_default_scratch_bytes, 0,
            radeon2d_default_surface_budget, queues, radeon2d_compute_ring, {nullptr, nullptr, nullptr, nullptr}};
}

Radeon2dStatus Radeon2dBackend::initialize(const Radeon2dBackendConfig& config) noexcept {
    if (initialized_ || faulted_) {
        return Radeon2dStatus::invalid_argument;
    }

    // Resume: the retained staging and system surfaces belong to these owners.
    if (parked_ && (config.gtt != config_.gtt || config.vram != config_.vram ||
                    config.vram_mc_base != config_.vram_mc_base)) {
        return Radeon2dStatus::invalid_argument;
    }

    if (config.sdma == nullptr || config.fences == nullptr || config.gtt == nullptr ||
        config.vram == nullptr || !config.vram->initialized() ||
        config.instance >= sdma6_instance_count || config.vram_mc_base > sdma::gpu_virtual_address_max ||
        config.ib_count < blit2d_min_ib_slots || config.ib_count > blit2d_max_ib_slots ||
        config.ib_bytes < blit2d_min_ib_bytes || config.ib_bytes % sdma6_ib_alignment != 0 ||
        config.ib_bytes / sizeof(uint32_t) > blit2d_max_ib_dwords) {
        return Radeon2dStatus::invalid_argument;
    }

    const Sdma6Report rings = config.sdma->report();

    if (!rings.submission_enabled || !rings.submit_enabled[config.instance]) {
        return Radeon2dStatus::submission_unavailable;
    }

    const uint64_t ring_bytes = round_page(uint64_t{config.ib_count} * config.ib_bytes);
    memory::Handle ib{};

    if (config.gtt->allocate(ring_bytes, memory::page_bytes, memory::Access::read, true, ib) !=
        memory::Status::success) {
        return Radeon2dStatus::memory_failure;
    }

    memory::Handle scratch{};
    const bool has_scratch = config.scratch_bytes != 0;

    if (has_scratch &&
        config.gtt->allocate(round_page(config.scratch_bytes), memory::page_bytes,
                             memory::Access::read_write, false, scratch) != memory::Status::success) {
        config.gtt->release(ib);
        return Radeon2dStatus::memory_failure;
    }

    submitter_ = {config.sdma, config.fences, config.instance};
    const Blit2dEngineConfig engine{config.gtt, ib, config.ib_count, config.ib_bytes,
                                    has_scratch ? scratch : memory::Handle{}, submitter_.submitter(),
                                    config.wait_timeout_us};

    if (engine_.initialize(engine) != Blit2dStatus::success) {
        if (has_scratch) {
            config.gtt->release(scratch);
        }

        config.gtt->release(ib);
        submitter_ = {};
        return Radeon2dStatus::engine_failure;
    }

    config_ = config;
    if (config_.surface_budget_bytes == 0) {
        config_.surface_budget_bytes = radeon2d_default_surface_budget;
    }

    ib_ = ib;
    scratch_ = scratch;
    has_scratch_ = has_scratch;
    fault_reason_ = Blit2dStatus::success;
    // Fences restart above every fence an earlier configuration handed out.
    completed_ = counter_;
    initialized_ = true;
    parked_ = false;
    sdma_pending_.clear();
    compute_pending_.clear();
    // The compute engine is optional: without it the backend is a complete
    // SDMA backend that does not advertise raster requests.
    compute_status_ = initialize_compute();
    return Radeon2dStatus::ready;
}

Raster2dStatus Radeon2dBackend::initialize_compute() noexcept {
    has_compute_ = false;
    compute_fault_ = Raster2dStatus::success;
    const bool custom = config_.compute_submitter.submit != nullptr;

    if (!custom && config_.queues == nullptr) {
        return Raster2dStatus::not_initialized;
    }

    if (!custom) {
        // Device-owned copy: the queue report is too large for a kernel stack.
        queue_report_ = config_.queues->report();

        if (config_.compute_ring == 0 || config_.compute_ring > queue_report_.compute_rings ||
            config_.compute_ring >= gfx11_max_rings || !queue_report_.submission_enabled ||
            !queue_report_.rings[config_.compute_ring].submit_enabled) {
            return Raster2dStatus::invalid_argument;
        }
    }

    const uint64_t code_bytes = round_page(raster2d_code_bytes);
    const uint64_t args_bytes = round_page(uint64_t{raster2d_args_bytes_total} * radeon2d_compute_ib_count);
    const uint64_t ib_bytes = round_page(uint64_t{radeon2d_compute_ib_bytes} * radeon2d_compute_ib_count);
    memory::Handle code{};
    memory::Handle args{};
    memory::Handle ib{};

    if (config_.gtt->allocate(code_bytes, memory::page_bytes, memory::Access::read, true, code) !=
        memory::Status::success) {
        return Raster2dStatus::invalid_memory;
    }

    if (config_.gtt->allocate(args_bytes, memory::page_bytes, memory::Access::read, true, args) !=
        memory::Status::success) {
        config_.gtt->release(code);
        return Raster2dStatus::invalid_memory;
    }

    if (config_.gtt->allocate(ib_bytes, memory::page_bytes, memory::Access::read, true, ib) !=
        memory::Status::success) {
        config_.gtt->release(args);
        config_.gtt->release(code);
        return Raster2dStatus::invalid_memory;
    }

    compute_adapter_ = {config_.queues, config_.fences, config_.compute_ring, radeon2d_compute_ib_bytes,
                        radeon2d_compute_ib_count};
    const Raster2dSubmitter submitter = custom ? config_.compute_submitter : compute_adapter_.submitter();
    // order_ibs_on_gpu: every compute IB starts with CS_PARTIAL_FLUSH + ACQUIRE_MEM,
    // so dependent compute operations are ordered by the CP (order_compute's own
    // compute wait is then redundant but kept as defence in depth).
    const Raster2dEngineConfig engine{config_.gtt, code, args, ib, radeon2d_compute_ib_count,
                                      radeon2d_compute_ib_bytes, submitter, config_.wait_timeout_us, true};
    const Raster2dStatus status = raster_.initialize(engine);

    if (status != Raster2dStatus::success) {
        // A failed generation left the engine faulted but holding no leases.
        config_.gtt->release(ib);
        config_.gtt->release(args);
        config_.gtt->release(code);
        compute_adapter_ = {};
        return status;
    }

    compute_code_ = code;
    compute_args_ = args;
    compute_ib_ = ib;
    has_compute_ = true;
    return Raster2dStatus::success;
}

// Only after the engine is shut down (or released after a reset).
void Radeon2dBackend::release_compute() noexcept {
    if (!has_compute_) {
        return;
    }

    config_.gtt->release(compute_ib_);
    config_.gtt->release(compute_args_);
    config_.gtt->release(compute_code_);
    compute_ib_ = {};
    compute_args_ = {};
    compute_code_ = {};
    compute_adapter_ = {};
    compute_pending_.clear();
    has_compute_ = false;
}

reactos::radeon2d::Blit2dBackend Radeon2dBackend::table() noexcept {
    reactos::radeon2d::Blit2dBackend value{this, fill_trampoline, copy_trampoline, upload_trampoline,
        readback_trampoline, surface_create_trampoline, surface_destroy_trampoline,
        staging_create_trampoline, staging_destroy_trampoline, sync_trampoline, completed_trampoline,
        surface_evict_trampoline, surface_restore_trampoline};

    if (has_compute_) {
        value.raster_operations = abi::raster_operations;
        value.raster_rop = raster_rop_trampoline;
        value.raster_alpha = raster_alpha_trampoline;
        value.raster_line = raster_line_trampoline;
        value.raster_gradient = raster_gradient_trampoline;
    }

    return value;
}

BackendStatus Radeon2dBackend::usable() const noexcept {
    if (!initialized_) {
        return BackendStatus::rejected;
    }

    return faulted_ ? BackendStatus::uncertain : BackendStatus::ok;
}

BackendStatus Radeon2dBackend::become_uncertain(Blit2dStatus reason) noexcept {
    if (!faulted_) {
        faulted_ = true;
        // The engine records the actual submit/fence failure behind faulted.
        fault_reason_ = reason == Blit2dStatus::faulted ? engine_.report().last_failure : reason;
    }

    ++uncertain_;
    return BackendStatus::uncertain;
}

// Maps one operation's engine result. Accepted work is flushed to the ring now
// (the primary is scanned out without any later sync). partial: an earlier
// engine call of the same operation was already accepted, so a refusal can no
// longer promise that nothing will be written.
BackendStatus Radeon2dBackend::finish(Blit2dStatus status, bool partial, uint64_t& fence) noexcept {
    fence = 0;

    if (status == Blit2dStatus::success) {
        const Blit2dStatus flushed = engine_.flush();

        if (flushed != Blit2dStatus::success) {
            return become_uncertain(flushed);
        }

        ++counter_;
        ++accepted_;
        fence = counter_;
        return BackendStatus::ok;
    }

    if (!partial && radeon2d_backend_status(status) == BackendStatus::rejected) {
        ++rejected_;
        return BackendStatus::rejected;
    }

    return become_uncertain(status);
}

bool Radeon2dBackend::engine_surface(const EngineSurface& surface, Blit2dSurface& output) const noexcept {
    // The engine validates alignment, pitch and address range itself.
    if (surface.format != abi::format_xrgb8888 || surface.width == 0 || surface.height == 0 ||
        surface.width > blit2d_max_surface_dimension || surface.height > blit2d_max_surface_dimension) {
        return false;
    }

    output = {surface.gpu_address, surface.pitch_bytes, surface.width, surface.height};
    return true;
}

bool Radeon2dBackend::prepare_clip(const EngineRect* rects, uint32_t count, Blit2dRect& bounds) noexcept {
    clip_.count = 0;

    if (rects == nullptr || count == 0 || count > blit2d_max_rects) {
        return false;
    }

    for (uint32_t index = 0; index < count; ++index) {
        Blit2dRect rect{};

        if (!to_rect(rects[index].x, rects[index].y, rects[index].width, rects[index].height, rect)) {
            clip_.count = 0;
            return false;
        }

        clip_.rects[index] = rect;
        bounds = index == 0 ? rect : united(bounds, rect);
    }

    clip_.count = count;
    return true;
}

BackendStatus Radeon2dBackend::fill(const EngineSurface& target, uint32_t color, const EngineRect* rects,
                                    uint32_t count, uint64_t& fence) noexcept {
    fence = 0;
    const BackendStatus state = usable();

    if (state != BackendStatus::ok) {
        return state;
    }

    Blit2dFill operation{};
    operation.color = color;

    if (!engine_surface(target, operation.destination) || !prepare_clip(rects, count, operation.bounds)) {
        ++rejected_;
        return BackendStatus::rejected;
    }

    Touch touch{};
    touch.write = surface_region(target, operation.bounds);

    if (!order_sdma(touch)) {
        return BackendStatus::uncertain;
    }

    const BackendStatus result = finish(engine_.fill(operation, clip_), false, fence);

    if (result == BackendStatus::ok) {
        record(sdma_pending_, touch);
    }

    return result;
}

BackendStatus Radeon2dBackend::copy(const EngineSurface& target, const EngineSurface& source,
                                    const CopyRect* rects, uint32_t count, uint64_t& fence) noexcept {
    fence = 0;
    const BackendStatus state = usable();

    if (state != BackendStatus::ok) {
        return state;
    }

    Blit2dCopy operation{};

    if (rects == nullptr || count == 0 || count > blit2d_max_rects ||
        !engine_surface(target, operation.destination) || !engine_surface(source, operation.source)) {
        ++rejected_;
        return BackendStatus::rejected;
    }

    // Every source and destination rectangle is proven inside the 16384-pixel
    // coordinate range first, so the conversions below cannot fail and the
    // source offsets stay far inside int32.
    const int64_t dx = int64_t{rects[0].source_x} - rects[0].destination.x;
    const int64_t dy = int64_t{rects[0].source_y} - rects[0].destination.y;
    bool common = true;

    for (uint32_t index = 0; index < count; ++index) {
        const CopyRect& rect = rects[index];
        Blit2dRect checked{};

        if (!to_rect(rect.source_x, rect.source_y, rect.destination.width, rect.destination.height,
                     checked) ||
            !to_rect(rect.destination.x, rect.destination.y, rect.destination.width,
                     rect.destination.height, checked)) {
            ++rejected_;
            return BackendStatus::rejected;
        }

        common = common && int64_t{rect.source_x} - rect.destination.x == dx &&
                 int64_t{rect.source_y} - rect.destination.y == dy;
    }

    if (common) {
        // One engine copy: the union as destination_rect, the rectangles as clip.
        Blit2dRect bounds{};

        for (uint32_t index = 0; index < count; ++index) {
            const EngineRect& rect = rects[index].destination;
            Blit2dRect converted{};

            to_rect(rect.x, rect.y, rect.width, rect.height, converted);
            clip_.rects[index] = converted;
            bounds = index == 0 ? converted : united(bounds, converted);
        }

        clip_.count = count;
        const int64_t source_x = int64_t{bounds.left} + dx;
        const int64_t source_y = int64_t{bounds.top} + dy;

        if (source_x < 0 || source_y < 0 || source_x > blit2d_max_surface_dimension ||
            source_y > blit2d_max_surface_dimension) {
            ++rejected_;
            return BackendStatus::rejected;
        }

        operation.destination_rect = bounds;
        operation.source_point = {static_cast<int32_t>(source_x), static_cast<int32_t>(source_y)};
        Touch touch{};
        touch.read_count = 1;
        touch.reads[0] = surface_region(source, {static_cast<int32_t>(source_x), static_cast<int32_t>(source_y),
            static_cast<int32_t>(source_x + (bounds.right - bounds.left)),
            static_cast<int32_t>(source_y + (bounds.bottom - bounds.top))});
        touch.write = surface_region(target, bounds);

        if (!order_sdma(touch)) {
            return BackendStatus::uncertain;
        }

        const BackendStatus result = finish(engine_.copy(operation, clip_), false, fence);

        if (result == BackendStatus::ok) {
            record(sdma_pending_, touch);
        }

        return result;
    }

    // Differing offsets: one engine copy per rectangle in the dispatcher's order.
    Touch whole{};
    whole.read_count = 1;
    whole.reads[0] = span_region(source.gpu_address, source.pitch_bytes, source.height);
    whole.write = span_region(target.gpu_address, target.pitch_bytes, target.height);

    if (!order_sdma(whole)) {
        return BackendStatus::uncertain;
    }

    record(sdma_pending_, whole);

    for (uint32_t index = 0; index < count; ++index) {
        const CopyRect& rect = rects[index];
        const EngineRect& area = rect.destination;

        to_rect(area.x, area.y, area.width, area.height, operation.destination_rect);
        operation.source_point = {static_cast<int32_t>(rect.source_x), static_cast<int32_t>(rect.source_y)};
        const Blit2dStatus status = engine_.copy(operation, unclipped);

        if (status != Blit2dStatus::success) {
            return finish(status, index != 0, fence);
        }
    }

    return finish(Blit2dStatus::success, false, fence);
}

Radeon2dBackend::StagingEntry* Radeon2dBackend::find_staging(const StagingBuffer& staging) noexcept {
    uint32_t index = 0;
    uint32_t generation = 0;

    if (!id_slot(staging.allocation, radeon2d_max_staging, index, generation)) {
        return nullptr;
    }

    StagingEntry& entry = staging_[index];

    if (entry.state != Slot::live || entry.generation != generation ||
        entry.gpu_address != staging.gpu_address || entry.bytes != staging.bytes) {
        return nullptr;
    }

    return &entry;
}

BackendStatus Radeon2dBackend::transfer(bool upload, const EngineSurface& surface,
                                        const StagingBuffer& staging, uint32_t staging_pitch,
                                        const TransferRect* rects, uint32_t count,
                                        uint64_t& fence) noexcept {
    fence = 0;
    const BackendStatus state = usable();

    if (state != BackendStatus::ok) {
        return state;
    }

    Blit2dSurface engine_target{};
    const StagingEntry* entry = find_staging(staging);

    if (entry == nullptr || !engine_surface(surface, engine_target) || rects == nullptr || count == 0 ||
        count > blit2d_max_rects || staging_pitch == 0 || (staging_pitch & 3u) != 0 ||
        staging_pitch / blit2d_bytes_per_pixel > blit2d_max_pitch_elements) {
        ++rejected_;
        return BackendStatus::rejected;
    }

    // Prove every rectangle before the first engine call, so a refusal after an
    // accepted rectangle (uncertain) can only come from a fault.
    for (uint32_t index = 0; index < count; ++index) {
        const TransferRect& rect = rects[index];
        Blit2dRect checked{};
        const uint64_t row_bytes = uint64_t{rect.surface.width} * blit2d_bytes_per_pixel;

        if (!to_rect(rect.surface.x, rect.surface.y, rect.surface.width, rect.surface.height, checked) ||
            (rect.staging_offset & 3u) != 0 || row_bytes > staging_pitch ||
            uint64_t{rect.staging_offset} + uint64_t{rect.surface.height - 1u} * staging_pitch +
                    row_bytes > entry->bytes) {
            ++rejected_;
            return BackendStatus::rejected;
        }
    }

    Touch touch{};
    touch.read_count = 1;

    for (uint32_t index = 0; index < count; ++index) {
        Blit2dRect area{};
        to_rect(rects[index].surface.x, rects[index].surface.y, rects[index].surface.width,
                rects[index].surface.height, area);
        touch.write.rect = index == 0 ? area : united(touch.write.rect, area);
    }

    const Blit2dRegion surface_part = surface_region(surface, touch.write.rect);
    const Blit2dRegion staging_part = span_region(entry->gpu_address, entry->bytes, 1);
    touch.reads[0] = upload ? staging_part : surface_part;
    touch.write = upload ? surface_part : staging_part;

    if (!order_sdma(touch)) {
        return BackendStatus::uncertain;
    }

    record(sdma_pending_, touch);

    for (uint32_t index = 0; index < count; ++index) {
        const TransferRect& rect = rects[index];
        Blit2dRect area{};
        to_rect(rect.surface.x, rect.surface.y, rect.surface.width, rect.surface.height, area);
        const Blit2dStaging transfer_staging{entry->handle, rect.staging_offset, staging_pitch,
                                             rect.surface.width, rect.surface.height};
        const Blit2dStatus status = upload ?
            engine_.upload(transfer_staging, {0, 0}, engine_target, area, unclipped) :
            engine_.readback(engine_target, area, transfer_staging, {0, 0});

        if (status != Blit2dStatus::success) {
            return finish(status, index != 0, fence);
        }
    }

    return finish(Blit2dStatus::success, false, fence);
}

BackendStatus Radeon2dBackend::surface_create(uint32_t width, uint32_t height, uint32_t format,
                                              EngineSurface& output, uint32_t& allocation) noexcept {
    output = {};
    allocation = 0;
    const BackendStatus state = usable();

    if (state != BackendStatus::ok) {
        return state;
    }

    SurfaceEntry* entry = nullptr;
    uint32_t slot_index = 0;

    for (uint32_t index = 0; index < radeon2d_max_surfaces; ++index) {
        if (surfaces_[index].state == Slot::free) {
            entry = &surfaces_[index];
            slot_index = index;
            break;
        }
    }

    if (entry == nullptr || format != abi::format_xrgb8888 || width == 0 || height == 0 ||
        width > blit2d_max_surface_dimension || height > blit2d_max_surface_dimension) {
        ++rejected_;
        return BackendStatus::rejected;
    }

    // width * 4 <= 65536, already a 256-byte multiple at the limit: pitch / 4
    // never exceeds blit2d_max_pitch_elements.
    const uint32_t pitch = (width * blit2d_bytes_per_pixel + (radeon2d_pitch_alignment - 1)) &
                           ~(radeon2d_pitch_alignment - 1);
    const uint64_t bytes = round_page(uint64_t{pitch} * height);
    VramAllocationHandle vram{};
    uint64_t gpu = 0;

    // surface_bytes_ never exceeds the budget, so the subtraction cannot wrap.
    if (bytes > config_.surface_budget_bytes - surface_bytes_ || !allocate_vram(bytes, vram, gpu)) {
        ++rejected_;
        return BackendStatus::rejected;
    }

    // Clear the whole pitch (row_bytes == pitch: 4 MiB packets) before any
    // session can read the surface; stale VRAM never reaches a caller.
    Blit2dFill clear{};
    clear.destination = {gpu, pitch, pitch / blit2d_bytes_per_pixel, height};
    clear.bounds = {0, 0, static_cast<int32_t>(pitch / blit2d_bytes_per_pixel), static_cast<int32_t>(height)};
    clear.color = 0;
    Blit2dStatus status = engine_.fill(clear, unclipped);

    if (status == Blit2dStatus::success) {
        status = engine_.flush();
    }

    if (status != Blit2dStatus::success) {
        if (!fault_status(status)) {
            config_.vram->release(vram);
            ++rejected_;
            return BackendStatus::rejected;
        }

        // The clear may still be writing: keep the VRAM out of circulation.
        entry->state = Slot::quarantined;
        entry->in_system = false;
        entry->has_vram = true;
        entry->has_system = false;
        entry->vram = vram;
        entry->bytes = bytes;
        surface_bytes_ += bytes;
        return become_uncertain(status);
    }

    ++counter_;
    entry->state = Slot::live;
    entry->in_system = false;
    entry->has_vram = true;
    entry->has_system = false;
    entry->generation = next_generation(entry->generation);
    entry->bytes = bytes;
    entry->vram = vram;
    entry->surface = {gpu, width, height, pitch, abi::format_xrgb8888};
    surface_bytes_ += bytes;
    output = entry->surface;
    allocation = make_id(slot_index, entry->generation);
    return BackendStatus::ok;
}

bool Radeon2dBackend::allocate_vram(uint64_t bytes, VramAllocationHandle& vram, uint64_t& gpu) noexcept {
    const VramRequest request{bytes, memory::page_bytes, vram_kernel_bo_flags(false)};

    if (config_.vram->allocate(request, vram) != VramStatus::success) {
        return false;
    }

    VramAllocationInfo info{};

    // The SDMA kernel ring addresses VRAM by MC address without GPUVM, so the
    // surface must be one physically contiguous span.
    if (config_.vram->describe(vram, info) != VramStatus::success || !info.contiguous ||
        info.allocated_bytes < bytes || info.first_byte > sdma::gpu_virtual_address_max - config_.vram_mc_base ||
        bytes > sdma::gpu_virtual_address_max - config_.vram_mc_base - info.first_byte) {
        config_.vram->release(vram);
        vram = {};
        return false;
    }

    gpu = config_.vram_mc_base + info.first_byte;
    return true;
}

bool Radeon2dBackend::idle_for_release() noexcept {
    // A parked backend synchronized everything and has no engine left.
    if (parked_) {
        return true;
    }

    if (!initialized_ || faulted_) {
        return false;
    }

    return completed() >= counter_ || sync(counter_) == BackendStatus::ok;
}

void Radeon2dBackend::release_surface(SurfaceEntry& entry) noexcept {
    if (entry.has_system) {
        config_.gtt->release(entry.system);
    }

    if (entry.has_vram) {
        config_.vram->release(entry.vram);
    }

    surface_bytes_ -= entry.bytes;
    entry.state = Slot::free;
    entry.in_system = false;
    entry.has_vram = false;
    entry.has_system = false;
    entry.bytes = 0;
    entry.vram = {};
    entry.system = {};
    entry.surface = {};
}

Radeon2dBackend::SurfaceEntry* Radeon2dBackend::find_surface(uint32_t allocation) noexcept {
    uint32_t index = 0;
    uint32_t generation = 0;

    if (!id_slot(allocation, radeon2d_max_surfaces, index, generation)) {
        return nullptr;
    }

    SurfaceEntry& entry = surfaces_[index];
    return entry.state == Slot::live && entry.generation == generation ? &entry : nullptr;
}

void Radeon2dBackend::surface_destroy(uint32_t allocation) noexcept {
    SurfaceEntry* entry = find_surface(allocation);

    if (entry == nullptr) {
        return;
    }

    if (idle_for_release()) {
        release_surface(*entry);
    } else {
        entry->state = Slot::quarantined;
    }
}

BackendStatus Radeon2dBackend::move_contents(const EngineSurface& from, uint64_t to_gpu) noexcept {
    Blit2dCopy operation{};
    operation.source = {from.gpu_address, from.pitch_bytes, from.width, from.height};
    operation.destination = {to_gpu, from.pitch_bytes, from.width, from.height};
    operation.destination_rect = {0, 0, static_cast<int32_t>(from.width), static_cast<int32_t>(from.height)};
    operation.source_point = {0, 0};
    uint64_t fence = 0;
    const BackendStatus accepted = finish(engine_.copy(operation, unclipped), false, fence);

    if (accepted != BackendStatus::ok) {
        return accepted;
    }

    // The old placement is released right after: prove the copy retired first.
    return sync(fence);
}

BackendStatus Radeon2dBackend::surface_evict(uint32_t allocation, EngineSurface& output) noexcept {
    output = {};
    const BackendStatus state = usable();

    if (state != BackendStatus::ok) {
        return state;
    }

    SurfaceEntry* entry = find_surface(allocation);

    if (entry == nullptr) {
        ++rejected_;
        return BackendStatus::rejected;
    }

    if (entry->in_system) {
        output = entry->surface;
        return BackendStatus::ok;
    }

    memory::Handle system{};
    memory::AllocationInfo info{};

    if (config_.gtt->allocate(entry->bytes, memory::page_bytes, memory::Access::read_write, false, system) !=
        memory::Status::success) {
        ++eviction_failures_;
        ++rejected_;
        return BackendStatus::rejected;
    }

    if (config_.gtt->describe(system, info) != memory::Status::success || info.gpu.byte_length < entry->bytes ||
        (info.gpu.virtual_address & 3u) != 0 || info.gpu.virtual_address > sdma::gpu_virtual_address_max ||
        entry->bytes > sdma::gpu_virtual_address_max - info.gpu.virtual_address) {
        config_.gtt->release(system);
        ++eviction_failures_;
        ++rejected_;
        return BackendStatus::rejected;
    }

    const BackendStatus moved = move_contents(entry->surface, info.gpu.virtual_address);

    if (moved == BackendStatus::rejected) {
        config_.gtt->release(system);
        ++eviction_failures_;
        return moved;
    }

    entry->system = system;
    entry->has_system = true;

    if (moved != BackendStatus::ok) {
        // The copy may still write the GTT pages: both stay held until reset.
        entry->state = Slot::quarantined;
        return moved;
    }

    config_.vram->release(entry->vram);
    entry->vram = {};
    entry->has_vram = false;
    entry->in_system = true;
    entry->surface.gpu_address = info.gpu.virtual_address;
    ++evictions_;
    output = entry->surface;
    return BackendStatus::ok;
}

BackendStatus Radeon2dBackend::surface_restore(uint32_t allocation, EngineSurface& output) noexcept {
    output = {};
    const BackendStatus state = usable();

    if (state != BackendStatus::ok) {
        return state;
    }

    SurfaceEntry* entry = find_surface(allocation);

    if (entry == nullptr) {
        ++rejected_;
        return BackendStatus::rejected;
    }

    if (!entry->in_system) {
        output = entry->surface;
        return BackendStatus::ok;
    }

    VramAllocationHandle vram{};
    uint64_t gpu = 0;

    if (!allocate_vram(entry->bytes, vram, gpu)) {
        ++rejected_;
        return BackendStatus::rejected;
    }

    const BackendStatus moved = move_contents(entry->surface, gpu);

    if (moved == BackendStatus::rejected) {
        config_.vram->release(vram);
        return moved;
    }

    entry->vram = vram;
    entry->has_vram = true;

    if (moved != BackendStatus::ok) {
        // The copy may still write the VRAM: both stay held until reset.
        entry->state = Slot::quarantined;
        return moved;
    }

    config_.gtt->release(entry->system);
    entry->system = {};
    entry->has_system = false;
    entry->in_system = false;
    entry->surface.gpu_address = gpu;
    ++restores_;
    output = entry->surface;
    return BackendStatus::ok;
}

BackendStatus Radeon2dBackend::staging_create(uint32_t bytes, StagingBuffer& output) noexcept {
    output = {};
    const BackendStatus state = usable();

    if (state != BackendStatus::ok) {
        return state;
    }

    StagingEntry* entry = nullptr;
    uint32_t slot_index = 0;

    for (uint32_t index = 0; index < radeon2d_max_staging; ++index) {
        if (staging_[index].state == Slot::free) {
            entry = &staging_[index];
            slot_index = index;
            break;
        }
    }

    memory::Handle handle{};

    if (entry == nullptr || bytes == 0 || (bytes & 3u) != 0 ||
        config_.gtt->allocate(round_page(bytes), memory::page_bytes, memory::Access::read_write, true,
                              handle) != memory::Status::success) {
        ++rejected_;
        return BackendStatus::rejected;
    }

    memory::AllocationInfo info{};

    if (config_.gtt->describe(handle, info) != memory::Status::success ||
        info.backing.cpu_bytes == nullptr || info.backing.cpu_byte_count < bytes ||
        info.gpu.byte_length < bytes || (info.gpu.virtual_address & 3u) != 0) {
        config_.gtt->release(handle);
        ++rejected_;
        return BackendStatus::rejected;
    }

    // Zero the CPU view: pages from the native pool may hold earlier contents.
    const sdma::CommandBuffer view(info.backing.cpu_bytes, bytes);

    for (uint32_t offset = 0; offset < bytes; offset += 4) {
        view.write_u32(offset, 0);
    }

    entry->state = Slot::live;
    entry->generation = next_generation(entry->generation);
    entry->handle = handle;
    entry->gpu_address = info.gpu.virtual_address;
    entry->bytes = bytes;
    // The kernel virtual address is handed to the display driver as an opaque
    // 64-bit value (StagingBuffer.cpu_address); it stays valid until
    // staging_destroy, which is the only path that releases the allocation.
    output.gpu_address = info.gpu.virtual_address;
    output.cpu_address = static_cast<uint64_t>(reinterpret_cast<uintptr_t>(info.backing.cpu_bytes));
    output.bytes = bytes;
    output.allocation = make_id(slot_index, entry->generation);
    return BackendStatus::ok;
}

void Radeon2dBackend::release_staging(StagingEntry& entry) noexcept {
    config_.gtt->release(entry.handle);
    entry.state = Slot::free;
    entry.handle = {};
    entry.gpu_address = 0;
    entry.bytes = 0;
}

void Radeon2dBackend::staging_destroy(const StagingBuffer& staging) noexcept {
    StagingEntry* entry = find_staging(staging);

    if (entry == nullptr) {
        return;
    }

    if (idle_for_release()) {
        release_staging(*entry);
    } else {
        entry->state = Slot::quarantined;
    }
}

BackendStatus Radeon2dBackend::sync(uint64_t fence) noexcept {
    const BackendStatus state = usable();

    if (state != BackendStatus::ok) {
        return state;
    }

    if (fence <= completed_) {
        return BackendStatus::ok;
    }

    if (fence > counter_) {
        return BackendStatus::rejected;
    }

    // Every accepted operation was flushed, so synchronizing both engines
    // retires them all.
    const uint64_t observed = counter_;
    const Blit2dStatus status = engine_.synchronize();

    if (status != Blit2dStatus::success) {
        return fault_status(status) ? become_uncertain(status) : BackendStatus::rejected;
    }

    sdma_pending_.clear();

    if (has_compute_) {
        const Raster2dStatus compute = raster_.synchronize();

        if (compute != Raster2dStatus::success) {
            return raster_fault_status(compute) ? become_uncertain_compute(compute) : BackendStatus::rejected;
        }

        compute_pending_.clear();
    }

    completed_ = observed;
    return BackendStatus::ok;
}

uint64_t Radeon2dBackend::completed() noexcept {
    if (!initialized_ || faulted_ || completed_ == counter_) {
        return completed_;
    }

    const Blit2dStatus status = engine_.retire();

    if (status != Blit2dStatus::success) {
        if (fault_status(status)) {
            become_uncertain(status);
        }

        return completed_;
    }

    const Blit2dReport engine = engine_.report();
    bool compute_idle = true;

    if (has_compute_) {
        const Raster2dStatus compute = raster_.retire();

        if (compute != Raster2dStatus::success) {
            if (raster_fault_status(compute)) {
                become_uncertain_compute(compute);
            }

            return completed_;
        }

        compute_idle = raster_.report().in_flight == 0;
    }

    if (engine.in_flight == 0 && engine.open_dwords == 0 && compute_idle) {
        completed_ = counter_;
    }

    return completed_;
}

void Radeon2dBackend::release_engine_memory() noexcept {
    if (has_scratch_) {
        config_.gtt->release(scratch_);
    }

    config_.gtt->release(ib_);
    ib_ = {};
    scratch_ = {};
    has_scratch_ = false;
}

Radeon2dStatus Radeon2dBackend::suspend() noexcept {
    if (!initialized_) {
        return Radeon2dStatus::uninitialized;
    }

    if (faulted_) {
        return Radeon2dStatus::faulted;
    }

    for (const SurfaceEntry& entry : surfaces_) {
        if (entry.state == Slot::quarantined || (entry.state == Slot::live && !entry.in_system)) {
            return Radeon2dStatus::busy;
        }
    }

    for (const StagingEntry& entry : staging_) {
        if (entry.state == Slot::quarantined) {
            return Radeon2dStatus::busy;
        }
    }

    if (sync(counter_) != BackendStatus::ok) {
        return faulted_ ? Radeon2dStatus::faulted : Radeon2dStatus::busy;
    }

    if (engine_.shutdown() != Blit2dStatus::success ||
        (has_compute_ && raster_.shutdown() != Raster2dStatus::success)) {
        return Radeon2dStatus::busy;
    }

    config_.sdma->retire();
    release_engine_memory();
    release_compute();
    submitter_ = {};
    completed_ = counter_;
    initialized_ = false;
    parked_ = true;
    return Radeon2dStatus::suspended;
}

Radeon2dStatus Radeon2dBackend::shutdown() noexcept {
    if (parked_) {
        // Parked: only the dispatcher's destroy calls may remain to be made.
        for (const SurfaceEntry& entry : surfaces_) {
            if (entry.state != Slot::free) {
                return Radeon2dStatus::busy;
            }
        }

        for (const StagingEntry& entry : staging_) {
            if (entry.state != Slot::free) {
                return Radeon2dStatus::busy;
            }
        }

        parked_ = false;
        return Radeon2dStatus::released;
    }

    if (!initialized_) {
        return Radeon2dStatus::uninitialized;
    }

    if (faulted_) {
        return Radeon2dStatus::faulted;
    }

    for (const SurfaceEntry& entry : surfaces_) {
        if (entry.state != Slot::free) {
            return Radeon2dStatus::busy;
        }
    }

    for (const StagingEntry& entry : staging_) {
        if (entry.state != Slot::free) {
            return Radeon2dStatus::busy;
        }
    }

    if (sync(counter_) != BackendStatus::ok) {
        return faulted_ ? Radeon2dStatus::faulted : Radeon2dStatus::busy;
    }

    if (engine_.shutdown() != Blit2dStatus::success ||
        (has_compute_ && raster_.shutdown() != Raster2dStatus::success)) {
        return Radeon2dStatus::busy;
    }

    // Return Sdma6's IB leases for the signalled fences before releasing the IBs
    // (a lease still held only defers the Manager's destruction).
    config_.sdma->retire();

    if (has_compute_ && config_.queues != nullptr) {
        config_.queues->retire();
    }

    release_engine_memory();
    release_compute();
    submitter_ = {};
    initialized_ = false;
    return Radeon2dStatus::released;
}

Radeon2dStatus Radeon2dBackend::release_after_reset() noexcept {
    if (!initialized_ && !parked_) {
        return Radeon2dStatus::uninitialized;
    }

    if (initialized_) {
        engine_.release_after_reset();

        if (has_compute_) {
            raster_.release_after_reset();
        }
    }

    for (SurfaceEntry& entry : surfaces_) {
        if (entry.state != Slot::free) {
            release_surface(entry);
        }
    }

    for (StagingEntry& entry : staging_) {
        if (entry.state != Slot::free) {
            release_staging(entry);
        }
    }

    if (initialized_) {
        release_engine_memory();
        release_compute();
    }

    submitter_ = {};
    sdma_pending_.clear();
    compute_fault_ = Raster2dStatus::success;
    // The reset proved that nothing handed out so far can still execute.
    completed_ = counter_;
    faulted_ = false;
    fault_reason_ = Blit2dStatus::success;
    initialized_ = false;
    parked_ = false;
    return Radeon2dStatus::released;
}

Radeon2dStatus Radeon2dBackend::park_after_reset() noexcept {
    if (!initialized_ && !parked_) {
        return Radeon2dStatus::uninitialized;
    }

    // The reset proof: no engine can still fetch an IB, read a source or write
    // a destination of any operation handed out so far.
    if (initialized_) {
        engine_.release_after_reset();

        if (has_compute_) {
            raster_.release_after_reset();
        }

        release_engine_memory();
        release_compute();
    }

    for (SurfaceEntry& entry : surfaces_) {
        if (entry.state == Slot::quarantined) {
            release_surface(entry);
        } else if (entry.state == Slot::live && !entry.in_system) {
            // VRAM contents do not survive a mode-1 reset.
            release_surface(entry);
            ++reset_dropped_;
        }
    }

    for (StagingEntry& entry : staging_) {
        if (entry.state == Slot::quarantined) {
            release_staging(entry);
        }
    }

    submitter_ = {};
    sdma_pending_.clear();
    compute_pending_.clear();
    compute_fault_ = Raster2dStatus::success;
    completed_ = counter_;
    faulted_ = false;
    fault_reason_ = Blit2dStatus::success;
    initialized_ = false;
    parked_ = true;
    ++resets_;
    return Radeon2dStatus::suspended;
}

Radeon2dBackendReport Radeon2dBackend::report() const noexcept {
    Radeon2dBackendReport output{};
    output.status = parked_ ? Radeon2dStatus::suspended :
                    !initialized_ ? Radeon2dStatus::uninitialized :
                    faulted_ ? Radeon2dStatus::faulted : Radeon2dStatus::ready;
    output.fence = counter_;
    output.completed = completed_;
    output.accepted = accepted_;
    output.rejected = rejected_;
    output.uncertain = uncertain_;

    for (const SurfaceEntry& entry : surfaces_) {
        output.live_surfaces += entry.state == Slot::live ? 1u : 0u;
        output.quarantined_surfaces += entry.state == Slot::quarantined ? 1u : 0u;
        output.system_surfaces += entry.state == Slot::live && entry.in_system ? 1u : 0u;
    }

    output.surface_bytes = surface_bytes_;
    output.evictions = evictions_;
    output.restores = restores_;
    output.eviction_failures = eviction_failures_;
    output.resets = resets_;
    output.reset_dropped_surfaces = reset_dropped_;

    for (const StagingEntry& entry : staging_) {
        output.live_staging += entry.state == Slot::live ? 1u : 0u;
        output.quarantined_staging += entry.state == Slot::quarantined ? 1u : 0u;
    }

    output.fault_reason = fault_reason_;
    output.engine = engine_.report();
    output.compute = has_compute_;
    output.compute_status = compute_status_;
    output.compute_fault = compute_fault_;
    output.cross_engine_waits = cross_waits_;
    output.compute_self_waits = compute_self_waits_;
    output.raster = raster_.report();
    return output;
}

// ------------------------------------------------------------ compute raster

BackendStatus Radeon2dBackend::compute_usable() const noexcept {
    const BackendStatus state = usable();

    if (state != BackendStatus::ok) {
        return state;
    }

    return has_compute_ ? BackendStatus::ok : BackendStatus::rejected;
}

BackendStatus Radeon2dBackend::become_uncertain_compute(Raster2dStatus reason) noexcept {
    if (!faulted_) {
        faulted_ = true;
        fault_reason_ = Blit2dStatus::faulted;
        compute_fault_ = reason == Raster2dStatus::faulted ? raster_.report().last_failure : reason;
    }

    ++uncertain_;
    return BackendStatus::uncertain;
}

bool Radeon2dBackend::wait_sdma() noexcept {
    const Blit2dStatus status = engine_.synchronize();

    if (status != Blit2dStatus::success) {
        become_uncertain(status);
        return false;
    }

    sdma_pending_.clear();
    return true;
}

bool Radeon2dBackend::wait_compute() noexcept {
    const Raster2dStatus status = raster_.synchronize();

    if (status != Raster2dStatus::success) {
        become_uncertain_compute(status);
        return false;
    }

    compute_pending_.clear();
    return true;
}

namespace {

bool touches(const Blit2dHazardTracker& tracker, const Blit2dRegion* reads, uint32_t read_count,
             const Blit2dRegion& write) noexcept {
    if (tracker.conflicts(nullptr, write)) {
        return true;
    }

    for (uint32_t index = 0; index < read_count; ++index) {
        if (tracker.conflicts(&reads[index], write)) {
            return true;
        }
    }

    return false;
}

} // namespace

bool Radeon2dBackend::order_sdma(const Touch& touch) noexcept {
    if (!has_compute_) {
        return true;
    }

    if (raster_.retire() == Raster2dStatus::success && raster_.report().in_flight == 0) {
        compute_pending_.clear();
    }

    if (touches(compute_pending_, touch.reads, touch.read_count, touch.write)) {
        ++cross_waits_;
        return wait_compute();
    }

    return true;
}

bool Radeon2dBackend::order_compute(const Touch& touch) noexcept {
    if (engine_.retire() == Blit2dStatus::success) {
        const Blit2dReport engine = engine_.report();

        if (engine.in_flight == 0 && engine.open_dwords == 0) {
            sdma_pending_.clear();
        }
    }

    if (raster_.retire() == Raster2dStatus::success && raster_.report().in_flight == 0) {
        compute_pending_.clear();
    }

    if (touches(sdma_pending_, touch.reads, touch.read_count, touch.write)) {
        ++cross_waits_;

        if (!wait_sdma()) {
            return false;
        }
    }

    // A compute queue starts the next IB's dispatches without waiting for the
    // previous ones: dependent compute work waits for its own fences too.
    if (touches(compute_pending_, touch.reads, touch.read_count, touch.write)) {
        ++compute_self_waits_;
        return wait_compute();
    }

    return true;
}

void Radeon2dBackend::record(Blit2dHazardTracker& tracker, const Touch& touch) noexcept {
    if (!has_compute_) {
        return;
    }

    if (!tracker.has_room(touch.read_count + 1)) {
        // A full tracker cannot describe more pending work: retire it.
        if (&tracker == &sdma_pending_ ? !wait_sdma() : !wait_compute()) {
            return;
        }
    }

    for (uint32_t index = 0; index < touch.read_count; ++index) {
        tracker.add(touch.reads[index], false);
    }

    tracker.add(touch.write, true);
}

BackendStatus Radeon2dBackend::finish_compute(Raster2dStatus status, bool partial, uint64_t& fence) noexcept {
    fence = 0;

    if (status == Raster2dStatus::success) {
        // Every engine operation submitted its own IB; flush is a no-op check.
        const Raster2dStatus flushed = raster_.flush();

        if (flushed != Raster2dStatus::success) {
            return become_uncertain_compute(flushed);
        }

        ++counter_;
        ++accepted_;
        fence = counter_;
        return BackendStatus::ok;
    }

    if (!partial && radeon2d_raster_status(status) == BackendStatus::rejected) {
        ++rejected_;
        return BackendStatus::rejected;
    }

    return become_uncertain_compute(status);
}

// The engine validates the format-specific pitch/address rules (raster2d.hpp
// Raster2dFormat); only the dimensions are pre-checked here.
bool Radeon2dBackend::raster_surface(const EngineSurface& surface, Raster2dFormat format,
                                     Raster2dSurface& output) const noexcept {
    if (surface.width == 0 || surface.height == 0 || surface.width > raster2d_max_surface_dimension ||
        surface.height > raster2d_max_surface_dimension) {
        return false;
    }

    output = {surface.gpu_address, surface.pitch_bytes, surface.width, surface.height, format};
    return true;
}

bool Radeon2dBackend::raster_clip(const EngineRect* rects, uint32_t count) noexcept {
    Blit2dRect bounds{};
    return prepare_clip(rects, count, bounds);
}

BackendStatus Radeon2dBackend::raster_rop(const RasterRop& op, const EngineRect* rects, uint32_t count,
                                          uint64_t& fence) noexcept {
    fence = 0;
    const BackendStatus state = compute_usable();

    if (state != BackendStatus::ok) {
        if (state == BackendStatus::rejected) {
            ++rejected_;
        }

        return state;
    }

    Raster2dRopRequest request{};
    Touch touch{};
    const uint32_t source_format = op.source.present ? op.source.surface.format : abi::format_xrgb8888;
    const Raster2dFormat source_role = source_format == abi::format_mono1 ? Raster2dFormat::mono1 :
                                       source_format == abi::format_index8 ? Raster2dFormat::index8 :
                                       source_format == abi::format_rgb16 ? Raster2dFormat::rgb16 :
                                       source_format == abi::format_rgb24 ? Raster2dFormat::rgb24 :
                                                                            Raster2dFormat::bgrx8888;
    const bool translated = op.source.present && abi::translated_format(source_format);

    if (op.target.format != abi::format_xrgb8888 || !raster_surface(op.target, Raster2dFormat::bgrx8888, request.destination) ||
        !raster_clip(rects, count) || (translated && op.source.table_address == 0) ||
        (op.source.present && !raster_surface(op.source.surface, source_role, request.source)) ||
        (op.mask.present && (op.mask.surface.format != abi::format_mono1 ||
                             !raster_surface(op.mask.surface, Raster2dFormat::mono1, request.mask)))) {
        ++rejected_;
        return BackendStatus::rejected;
    }

    request.destination_rect = {op.bounds.left, op.bounds.top, op.bounds.right, op.bounds.bottom};
    request.has_source = op.source.present;
    request.source_monochrome = op.source.present && op.source.surface.format == abi::format_mono1;
    request.source_table = translated ? op.source.table_address : 0;
    // A mirrored axis (0xa17) maps from the far edge of the DIB SourceRect. The
    // DIB code truncates q = k * width / den toward zero; for k >= 0 and den > 0
    // that is sign(width) * floor(k * |width| / den), so a negative extent becomes
    // the kernel's unsigned magnitude with the step sign flipped.
    request.source_left = op.mirror_x ? op.map.left + op.map.width : op.map.left;
    request.source_top = op.mirror_y ? op.map.top + op.map.height : op.map.top;
    request.source_width = op.map.width < 0 ? -op.map.width : op.map.width;
    request.source_height = op.map.height < 0 ? -op.map.height : op.map.height;
    request.source_sign_x = (op.mirror_x ? -1 : 1) * (op.map.width < 0 ? -1 : 1);
    request.source_sign_y = (op.mirror_y ? -1 : 1) * (op.map.height < 0 ? -1 : 1);
    request.stretch_mask = op.stretch_mask;
    request.running_phase = op.running_phase;
    request.mono_foreground = op.mono_foreground;
    request.mono_background = op.mono_background;
    request.skip_outside_source = (op.flags & abi::rop_flag_skip_outside_source) != 0;
    request.has_colour_key = (op.flags & abi::rop_flag_colour_key) != 0;
    request.colour_key = op.colour_key;
    request.has_pattern = op.pattern != nullptr;
    request.pattern = op.pattern;
    request.brush_colour = op.brush_colour;
    request.brush_origin = {op.brush_x, op.brush_y};
    request.has_mask = op.mask.present;
    request.mask_origin = {op.mask_x, op.mask_y};
    request.rop4 = op.rop4;

    touch.write = surface_region(op.target, clip_.count != 0 ? clip_.rects[0] : Blit2dRect{});
    for (uint32_t index = 1; index < clip_.count; ++index) {
        touch.write.rect = united(touch.write.rect, clip_.rects[index]);
    }

    const bool whole_source = op.mirror_x || op.mirror_y || op.stretch_mask;

    if (translated) {
        // The table and the pixel rows: one staging byte span.
        const uint64_t bytes = op.source.surface.gpu_address - op.source.table_address +
                               uint64_t{op.source.surface.pitch_bytes} * op.source.surface.height;
        touch.reads[touch.read_count++] = span_region(op.source.table_address, static_cast<uint32_t>(bytes), 1);
    } else if (op.source.present) {
        const bool signed_map = op.map.width < 0 || op.map.height < 0;
        touch.reads[touch.read_count++] = op.source.surface.format == abi::format_xrgb8888 ?
            surface_region(op.source.surface,
                           whole_source || signed_map ?
                               Blit2dRect{0, 0, static_cast<int32_t>(op.source.surface.width),
                                          static_cast<int32_t>(op.source.surface.height)} :
                               mapped_rect(op.map, op.source.surface)) :
            span_region(op.source.surface.gpu_address, op.source.surface.pitch_bytes, op.source.surface.height);
    }

    if (op.mask.present) {
        touch.reads[touch.read_count++] =
            span_region(op.mask.surface.gpu_address, op.mask.surface.pitch_bytes, op.mask.surface.height);
    }

    if (!order_compute(touch)) {
        return BackendStatus::uncertain;
    }

    const BackendStatus result = finish_compute(raster_.rop(request, clip_), false, fence);

    if (result == BackendStatus::ok) {
        record(compute_pending_, touch);
    }

    return result;
}

BackendStatus Radeon2dBackend::raster_alpha(const RasterAlpha& op, const EngineRect* rects, uint32_t count,
                                            uint64_t& fence) noexcept {
    fence = 0;
    const BackendStatus state = compute_usable();

    if (state != BackendStatus::ok) {
        if (state == BackendStatus::rejected) {
            ++rejected_;
        }

        return state;
    }

    Raster2dAlphaRequest request{};
    const uint32_t source_format = op.coverage8 ? abi::format_coverage8 : abi::format_xrgb8888;

    if (op.target.format != abi::format_xrgb8888 || !raster_surface(op.target, Raster2dFormat::bgrx8888, request.destination) ||
        !op.source.present || op.source.surface.format != source_format ||
        !raster_surface(op.source.surface, op.coverage8 ? Raster2dFormat::coverage8 : Raster2dFormat::bgrx8888,
                        request.source) || !raster_clip(rects, count) ||
        op.constant_alpha > 255) {
        ++rejected_;
        return BackendStatus::rejected;
    }

    request.destination_rect = {op.bounds.left, op.bounds.top, op.bounds.right, op.bounds.bottom};
    request.source_left = op.map.left;
    request.source_top = op.map.top;
    request.source_width = op.map.width;
    request.source_height = op.map.height;
    request.constant_alpha = op.constant_alpha;
    request.source_alpha = op.source_alpha;
    request.coverage8 = op.coverage8;
    request.solid_colour = op.colour;

    Touch touch{};
    touch.write = surface_region(op.target, clip_.rects[0]);
    for (uint32_t index = 1; index < clip_.count; ++index) {
        touch.write.rect = united(touch.write.rect, clip_.rects[index]);
    }

    touch.read_count = 1;
    touch.reads[0] = op.coverage8 ?
        span_region(op.source.surface.gpu_address, op.source.surface.pitch_bytes, op.source.surface.height) :
        surface_region(op.source.surface, mapped_rect(op.map, op.source.surface));

    if (!order_compute(touch)) {
        return BackendStatus::uncertain;
    }

    const BackendStatus result = finish_compute(raster_.alpha(request, clip_), false, fence);

    if (result == BackendStatus::ok) {
        record(compute_pending_, touch);
    }

    return result;
}

BackendStatus Radeon2dBackend::raster_line(const RasterLine& op, const EngineRect* rects, uint32_t count,
                                           uint64_t& fence) noexcept {
    fence = 0;
    const BackendStatus state = compute_usable();

    if (state != BackendStatus::ok) {
        if (state == BackendStatus::rejected) {
            ++rejected_;
        }

        return state;
    }

    Raster2dLineRequest request{};
    const int64_t adx = op.x1 >= op.x0 ? int64_t{op.x1} - op.x0 : int64_t{op.x0} - op.x1;
    const int64_t ady = op.y1 >= op.y0 ? int64_t{op.y1} - op.y0 : int64_t{op.y0} - op.y1;
    const int64_t major = adx > ady ? adx : ady;

    // The styled kernel stages one style bit per major step (raster2d: at most
    // raster2d_pattern_bytes * 8 steps).
    if (op.target.format != abi::format_xrgb8888 || !raster_surface(op.target, Raster2dFormat::bgrx8888, request.destination) ||
        !raster_clip(rects, count) || op.rop3 > 0xff ||
        (op.styled && (op.style == nullptr || op.style_bit_count == 0 ||
                       major > int64_t{raster2d_pattern_bytes} * 8))) {
        ++rejected_;
        return BackendStatus::rejected;
    }

    request.x0 = op.x0;
    request.y0 = op.y0;
    request.x1 = op.x1;
    request.y1 = op.y1;
    request.colour = op.colour;
    request.rop3 = op.rop3;
    request.load_destination = rop3_uses_destination(op.rop3);
    request.styled = op.styled;
    request.style_bits = op.style;
    request.style_bit_count = op.style_bit_count;
    request.has_pattern = op.pattern != nullptr;
    request.pattern = op.pattern;
    request.brush_origin = {op.brush_x, op.brush_y};

    Touch touch{};
    touch.write = surface_region(op.target, clip_.rects[0]);
    for (uint32_t index = 1; index < clip_.count; ++index) {
        touch.write.rect = united(touch.write.rect, clip_.rects[index]);
    }

    if (!order_compute(touch)) {
        return BackendStatus::uncertain;
    }

    // One engine line per clip rectangle (the kernel clips to one rectangle);
    // each pixel lies in at most one of the disjoint rectangles.
    const uint32_t rect_count = clip_.count;

    for (uint32_t index = 0; index < rect_count; ++index) {
        request.clip_bounds = clip_.rects[index];
        const Raster2dStatus status = raster_.line(request);

        if (status != Raster2dStatus::success) {
            return finish_compute(status, index != 0, fence);
        }
    }

    const BackendStatus result = finish_compute(Raster2dStatus::success, false, fence);

    if (result == BackendStatus::ok) {
        record(compute_pending_, touch);
    }

    return result;
}

BackendStatus Radeon2dBackend::raster_gradient(const RasterGradient& op, const EngineRect* rects,
                                               uint32_t count, uint64_t& fence) noexcept {
    fence = 0;
    const BackendStatus state = compute_usable();

    if (state != BackendStatus::ok) {
        if (state == BackendStatus::rejected) {
            ++rejected_;
        }

        return state;
    }

    Raster2dSurface destination{};

    if (op.target.format != abi::format_xrgb8888 || !raster_surface(op.target, Raster2dFormat::bgrx8888, destination) ||
        !raster_clip(rects, count) || op.mode > abi::gradient_triangle) {
        ++rejected_;
        return BackendStatus::rejected;
    }

    Touch touch{};
    touch.write = surface_region(op.target, clip_.rects[0]);
    for (uint32_t index = 1; index < clip_.count; ++index) {
        touch.write.rect = united(touch.write.rect, clip_.rects[index]);
    }

    Raster2dClipList& clip = clip_;

    if (op.mode != abi::gradient_triangle) {
        // eng/gradient.c IntEngGradientFillRect: rcSG from the two vertices, the
        // colour stepped from v1 (UpperLeft) towards v2 over the axis span, a
        // single v1 colour when the colours match or the span is at most 1.
        const abi::Vertex& v1 = op.vertices[0];
        const abi::Vertex& v2 = op.vertices[1];
        Raster2dGradientRectRequest request{};
        request.destination = destination;
        request.destination_rect = {v1.x < v2.x ? v1.x : v2.x, v1.y < v2.y ? v1.y : v2.y,
                                    v1.x > v2.x ? v1.x : v2.x, v1.y > v2.y ? v1.y : v2.y};
        request.horizontal = op.mode == abi::gradient_rect_h;
        const int64_t span = request.horizontal ?
            int64_t{request.destination_rect.right} - request.destination_rect.left :
            int64_t{request.destination_rect.bottom} - request.destination_rect.top;
        const bool solid = (v1.red == v2.red && v1.green == v2.green && v1.blue == v2.blue) || span <= 1;
        request.origin = request.horizontal ? request.destination_rect.left : request.destination_rect.top;
        request.divisor = solid ? 1u : static_cast<uint32_t>(span);
        request.colour0[0] = v1.red;
        request.colour0[1] = v1.green;
        request.colour0[2] = v1.blue;
        request.colour1[0] = solid ? v1.red : v2.red;
        request.colour1[1] = solid ? v1.green : v2.green;
        request.colour1[2] = solid ? v1.blue : v2.blue;

        if (!order_compute(touch)) {
            return BackendStatus::uncertain;
        }

        const BackendStatus result = finish_compute(raster_.gradient_rect(request, clip), false, fence);

        if (result == BackendStatus::ok) {
            record(compute_pending_, touch);
        }

        return result;
    }

    // eng/gradient.c IntEngGradientFillTriangle, shaded and single-colour: the
    // engine plans the rows itself (any height) and intersects every clip
    // rectangle with the extents, exactly as the engine's FillRect loop.
    Raster2dTriangleRequest request{};
    request.destination = destination;
    request.extents = {op.extents.left, op.extents.top, op.extents.right, op.extents.bottom};

    for (uint32_t index = 0; index < 3; ++index) {
        const abi::Vertex& vertex = op.vertices[index];
        request.vertices[index] = {vertex.x, vertex.y, vertex.red, vertex.green, vertex.blue};
    }

    if (!order_compute(touch)) {
        return BackendStatus::uncertain;
    }

    const BackendStatus result = finish_compute(raster_.gradient_triangle(request, clip), false, fence);

    if (result == BackendStatus::ok) {
        record(compute_pending_, touch);
    }

    return result;
}

// ---------------------------------------------------------------- recovery

bool Radeon2dRecovery::needed(const Radeon2dBackend& backend, FenceManager& fences) noexcept {
    if (backend.faulted()) {
        return true;
    }

    for (uint32_t ring = 0; ring < fence_max_rings; ++ring) {
        if (fences.ring_report(ring).state == FenceRingState::hung) {
            return true;
        }
    }

    return false;
}

bool Radeon2dRecovery::retire_after_reset(void* backend) noexcept {
    if (backend == nullptr) {
        return true;
    }

    const Radeon2dStatus status = static_cast<Radeon2dBackend*>(backend)->park_after_reset();
    return status == Radeon2dStatus::suspended || status == Radeon2dStatus::uninitialized;
}

Radeon2dRecoveryResult Radeon2dRecovery::fail(reactos::radeon2d::Dispatcher& dispatcher,
                                              Radeon2dRecoveryResult reason, bool quiescent) noexcept {
    dispatcher.disable(quiescent);
    report_.disabled = true;
    report_.quiescent = quiescent;
    report_.last = reason;
    ++report_.failed;
    return reason;
}

Radeon2dRecoveryResult Radeon2dRecovery::recover(reactos::radeon2d::Dispatcher& dispatcher, Radeon2dBackend& backend,
                                                 const Radeon2dBackendConfig& config,
                                                 const reactos::radeon2d::EngineSurface& primary,
                                                 const Radeon2dRecoveryPlatform& platform) noexcept {
    if (report_.disabled) {
        return Radeon2dRecoveryResult::disabled;
    }

    ++report_.attempts;
    report_.reset_status = AsicStatus::completed;
    report_.restart_status = AsicStatus::completed;
    report_.backend_status = Radeon2dStatus::uninitialized;

    if (platform.stop_interrupts == nullptr || platform.reset_and_release == nullptr ||
        platform.reset_proven == nullptr || platform.restart == nullptr || platform.scanout_matches == nullptr) {
        return fail(dispatcher, Radeon2dRecoveryResult::dispatcher_refused, false);
    }

    // A device that keeps hanging is not reset in a loop.
    if (report_.recovered >= radeon2d_max_recoveries) {
        return fail(dispatcher, Radeon2dRecoveryResult::limit_reached, false);
    }

    // No new 2D work from here on; unproven work becomes per-surface uncertainty
    // (already done when the detecting path quiesced the dispatcher).
    if (!dispatcher.recovering() && !dispatcher.begin_recovery()) {
        return fail(dispatcher, Radeon2dRecoveryResult::dispatcher_refused, false);
    }

    if (platform.display_quiesce != nullptr) {
        platform.display_quiesce(platform.context);
    }

    if (!platform.stop_interrupts(platform.context)) {
        return fail(dispatcher, Radeon2dRecoveryResult::quiesce_failed, false);
    }

    // Mode-1 reset and release; the backend parks inside the reset window.
    report_.reset_status = platform.reset_and_release(platform.context);

    if (report_.reset_status != AsicStatus::released) {
        return fail(dispatcher, Radeon2dRecoveryResult::reset_failed, platform.reset_proven(platform.context));
    }

    // From here on the reset proof holds: a failure leaves no GPU work running.
    report_.restart_status = platform.restart(platform.context);

    if (report_.restart_status != AsicStatus::completed) {
        return fail(dispatcher, Radeon2dRecoveryResult::restart_failed, true);
    }

    if (platform.rearm_services != nullptr) {
        platform.rearm_services(platform.context);
    }

    if (!platform.scanout_matches(platform.context)) {
        ++report_.scanout_mismatches;
        return fail(dispatcher, Radeon2dRecoveryResult::scanout_mismatch, true);
    }

    report_.backend_status = backend.initialize(config);

    if (report_.backend_status != Radeon2dStatus::ready) {
        return fail(dispatcher, Radeon2dRecoveryResult::backend_failed, true);
    }

    if (!dispatcher.complete_recovery(backend.table(), primary)) {
        // A restore on the restarted engine was uncertain: it may still run.
        return fail(dispatcher, Radeon2dRecoveryResult::second_fault, !backend.faulted());
    }

    if (platform.display_resume != nullptr) {
        platform.display_resume(platform.context);
    }

    ++report_.recovered;
    report_.last = Radeon2dRecoveryResult::recovered;
    return Radeon2dRecoveryResult::recovered;
}

bool Radeon2dRecoveryGate::request(bool forced) noexcept {
    if (stopping_ || disabled_) {
        ++refused_;
        return false;
    }

    forced_ = forced_ || forced;

    if (state_ != Radeon2dRecoveryState::idle) {
        ++coalesced_;
        return false;
    }

    state_ = Radeon2dRecoveryState::requested;
    ++requests_;
    return true;
}

bool Radeon2dRecoveryGate::begin(bool low_power, bool due) noexcept {
    if (stopping_ || state_ != Radeon2dRecoveryState::requested) {
        return false;
    }

    // Never during a power transition's D3 interval: resumed later.
    if (low_power) {
        ++deferred_;
        return false;
    }

    if (!forced_ && !due) {
        state_ = Radeon2dRecoveryState::idle;
        ++dropped_;
        return false;
    }

    state_ = Radeon2dRecoveryState::running;
    forced_ = false;
    ++runs_;
    return true;
}

void Radeon2dRecoveryGate::finish(bool disabled) noexcept {
    if (state_ == Radeon2dRecoveryState::running) {
        state_ = Radeon2dRecoveryState::idle;
    }

    disabled_ = disabled_ || disabled;
}

bool Radeon2dRecoveryGate::stop() noexcept {
    stopping_ = true;
    return state_ != Radeon2dRecoveryState::running;
}

Radeon2dRecoveryGateReport Radeon2dRecoveryGate::report() const noexcept {
    return {state_, forced_, stopping_, disabled_, requests_, coalesced_, refused_, deferred_, dropped_, runs_};
}

} // namespace rx7900::hardware

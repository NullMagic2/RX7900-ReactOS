/*
Module: rx7900.display.modes
Purpose: Checked Linux-derived DCN3.2 timing preparation and mode transactions.
Created: 2026-10-21
Architecture: Portable C++ display core; no MMIO, firmware, DMA or NT entry points.
Timing validation/field calculations adapted from Linux v6.12 AMD display code.
Copyright 2012-15, 2022 Advanced Micro Devices, Inc.
Permission notice and pinned sources: licenses/AMDGPU-DISPLAY.txt.
*/
#include "modes.hpp"

namespace rx7900::display {
namespace {

// DCN3.2 masks and minimum constraints are from the pinned AMD sources in the
// license file. These bounds do not imply that every fitting mode is supported.
constexpr uint32_t timing_field_mask = 0x7fffu;
constexpr uint32_t maximum_total = timing_field_mask + 1u;
constexpr uint32_t minimum_h_blank = 32u;
constexpr uint32_t minimum_v_blank = 3u;
constexpr uint32_t minimum_h_sync = 4u;
constexpr uint32_t minimum_v_sync = 1u;
constexpr uint32_t video_mode_map_mem_linear = 0x40000000u;
constexpr uint32_t video_mode_no_zero_memory = 0x80000000u;

bool same_timing(const Timing& left, const Timing& right) noexcept {
    return left.pixel_clock_khz == right.pixel_clock_khz &&
           left.width == right.width && left.h_total == right.h_total &&
           left.h_front_porch == right.h_front_porch &&
           left.h_sync_width == right.h_sync_width &&
           left.height == right.height && left.v_total == right.v_total &&
           left.v_front_porch == right.v_front_porch &&
           left.v_sync_width == right.v_sync_width && left.flags == right.flags;
}

class BusyScope final {
public:
    explicit BusyScope(bool& busy) noexcept : busy_(busy) { busy_ = true; }
    ~BusyScope() { busy_ = false; }
    BusyScope(const BusyScope&) = delete;
    BusyScope& operator=(const BusyScope&) = delete;

private:
    bool& busy_;
};

} // namespace

Status prepare_optc_timing(const Timing& timing, OptcTiming& output) noexcept {
    if ((timing.flags & ~(positive_hsync | positive_vsync | interlaced)) != 0 ||
        (timing.flags & interlaced) != 0) {
        return Status::unsupported_timing;
    }

    if (timing.pixel_clock_khz == 0 || timing.width == 0 || timing.height == 0 ||
        timing.h_total <= timing.width || timing.v_total <= timing.height ||
        timing.h_total > maximum_total || timing.v_total > maximum_total) {
        return Status::invalid_timing;
    }

    const uint32_t h_blank = timing.h_total - timing.width;
    const uint32_t v_blank = timing.v_total - timing.height;

    // Reject invalid porch/sync geometry instead of inheriting unsigned wrap.
    // The upstream progressive porch workaround requires at least one line;
    // rejecting zero keeps the advertised and programmed timing identical.
    if (h_blank < minimum_h_blank || v_blank < minimum_v_blank ||
        timing.h_front_porch == 0 || timing.v_front_porch == 0 ||
        timing.h_front_porch > h_blank || timing.v_front_porch > v_blank ||
        timing.h_sync_width < minimum_h_sync || timing.v_sync_width < minimum_v_sync ||
        timing.h_sync_width > h_blank - timing.h_front_porch ||
        timing.v_sync_width > v_blank - timing.v_front_porch) {
        return Status::invalid_timing;
    }

    const OptcTiming prepared{
        timing.h_total - 1u,
        timing.h_total - timing.h_front_porch,
        h_blank - timing.h_front_porch,
        timing.h_sync_width,
        (timing.flags & positive_hsync) != 0 ? 0u : 1u,
        timing.v_total - 1u,
        timing.v_total - timing.v_front_porch,
        v_blank - timing.v_front_porch,
        timing.v_sync_width,
        (timing.flags & positive_vsync) != 0 ? 0u : 1u
    };
    output = prepared;
    return Status::success;
}

Status refresh_millihz(const Timing& timing, uint32_t& output) noexcept {
    OptcTiming prepared{};
    const Status status = prepare_optc_timing(timing, prepared);

    if (status != Status::success) {
        return status;
    }

    // A uint32 clock times 1,000,000 and two bounded totals fit uint64.
    const uint64_t numerator = static_cast<uint64_t>(timing.pixel_clock_khz) * 1000000u;
    const uint64_t denominator = static_cast<uint64_t>(timing.h_total) * timing.v_total;
    const uint64_t frequency = numerator / denominator;

    if (frequency == 0 || frequency > UINT32_MAX) {
        return Status::arithmetic_overflow;
    }

    output = static_cast<uint32_t>(frequency);
    return Status::success;
}

Status validate_framebuffer(const Timing& timing, const Framebuffer& framebuffer,
                            ScanoutLayout& output) noexcept {
    OptcTiming prepared{};
    const Status status = prepare_optc_timing(timing, prepared);

    if (status != Status::success) {
        return status;
    }

    const uint64_t minimum_stride = static_cast<uint64_t>(timing.width) *
                                    xrgb8888_bytes_per_pixel;

    if (framebuffer.stride_bytes < minimum_stride ||
        framebuffer.stride_bytes > INT32_MAX ||
        framebuffer.stride_bytes % xrgb8888_bytes_per_pixel != 0 ||
        framebuffer.surface_offset % xrgb8888_bytes_per_pixel != 0 ||
        framebuffer.allocation_base.value % xrgb8888_bytes_per_pixel != 0 ||
        framebuffer.surface_offset > framebuffer.allocation_bytes) {
        return Status::invalid_framebuffer;
    }

    // uint32 stride times uint32 height fits uint64 without a helper or narrowing.
    const uint64_t surface_bytes = static_cast<uint64_t>(framebuffer.stride_bytes) *
                                   timing.height;

    if (surface_bytes > framebuffer.allocation_bytes - framebuffer.surface_offset) {
        return Status::invalid_framebuffer;
    }

    if (framebuffer.surface_offset > UINT64_MAX - framebuffer.allocation_base.value) {
        return Status::arithmetic_overflow;
    }

    const uint64_t surface_address = framebuffer.allocation_base.value +
                                     framebuffer.surface_offset;

    if (framebuffer.allocation_bytes == 0 ||
        framebuffer.allocation_bytes - 1u > UINT64_MAX - framebuffer.allocation_base.value) {
        return Status::arithmetic_overflow;
    }

    if (surface_bytes - 1u > UINT64_MAX - surface_address) {
        return Status::arithmetic_overflow;
    }

    output = ScanoutLayout{{surface_address}, surface_bytes, framebuffer.stride_bytes};
    return Status::success;
}

Status ModeCatalog::append(const Timing& timing, uint32_t& mode_index) noexcept {
    OptcTiming prepared{};
    uint32_t frequency = 0;
    const Status status = prepare_optc_timing(timing, prepared);

    if (status != Status::success) {
        return status;
    }

    const Status frequency_status = refresh_millihz(timing, frequency);

    if (frequency_status != Status::success) {
        return frequency_status;
    }

    for (uint32_t index = 0; index < count_; ++index) {
        if (same_timing(modes_[index], timing)) {
            return Status::duplicate_mode;
        }
    }

    if (count_ == mode_capacity) {
        return Status::capacity_exceeded;
    }

    modes_[count_] = timing;
    mode_index = count_;
    ++count_;
    return Status::success;
}

Status ModeCatalog::get(uint32_t mode_index, Timing& output) const noexcept {
    if (mode_index >= count_) {
        return Status::mode_not_found;
    }

    output = modes_[mode_index];
    return Status::success;
}

Status decode_video_mode_request(ByteView input, ModeRequest& output) noexcept {
    uint32_t raw = 0;

    if (!input.read_u32(0, raw)) {
        return Status::invalid_request;
    }

    const uint32_t flags = video_mode_map_mem_linear | video_mode_no_zero_memory;
    output = ModeRequest{raw & ~flags, (raw & video_mode_no_zero_memory) != 0};
    return Status::success;
}

Status ModeController::append_mode(const Timing& timing, uint32_t& mode_index) noexcept {
    if (busy_) {
        return Status::busy;
    }

    return catalog_.append(timing, mode_index);
}

Status ModeController::query_mode(uint32_t mode_index, Timing& output) const noexcept {
    return catalog_.get(mode_index, output);
}

Status ModeController::set_mode(const ModeRequest& request,
                                const Framebuffer& framebuffer) noexcept {
    if (busy_) {
        return Status::busy;
    }

    BusyScope transaction(busy_);
    ModePlan next{};
    next.mode_index = request.mode_index;
    next.preserve_contents = request.preserve_contents;
    Status status = catalog_.get(request.mode_index, next.timing);

    if (status != Status::success) {
        return status;
    }

    status = prepare_optc_timing(next.timing, next.optc);

    if (status != Status::success) {
        return status;
    }

    status = validate_framebuffer(next.timing, framebuffer, next.scanout);

    if (status != Status::success) {
        return status;
    }

    if (backend_.prepare == nullptr || backend_.apply == nullptr) {
        return Status::hardware_not_ready;
    }

    status = backend_.prepare(backend_.context, next);

    if (status != Status::success) {
        return status;
    }

    const ApplyResult applied = backend_.apply(backend_.context, next);

    if (applied == ApplyResult::committed) {
        current_ = next;
        current_known_ = true;
        return Status::success;
    }

    if (applied == ApplyResult::unchanged) {
        return Status::hardware_failure;
    }

    if (current_known_) {
        ModePlan previous = current_;
        previous.preserve_contents = true;
        current_known_ = false;

        // The previous allocation is retained by the owner until this returns.
        if (backend_.prepare(backend_.context, previous) == Status::success &&
            backend_.apply(backend_.context, previous) == ApplyResult::committed) {
            current_ = previous;
            current_known_ = true;
            return Status::hardware_failure;
        }
    }

    current_known_ = false;
    return Status::state_unknown;
}

Status ModeController::current_mode(ModePlan& output) const noexcept {
    if (busy_) {
        return Status::busy;
    }

    if (!current_known_) {
        return Status::state_unknown;
    }

    output = current_;
    return Status::success;
}

Status ModeController::invalidate_current() noexcept {
    if (busy_) {
        return Status::busy;
    }

    current_ = {};
    current_known_ = false;
    return Status::success;
}

} // namespace rx7900::display

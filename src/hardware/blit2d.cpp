/*
Module: rx7900.hardware.blit2d
Purpose: Validate, order and encode SDMA 6.0 rectangle fills/copies and recycle IBs.
Created: 2026-10-22
Architecture: Allocation-free; IB bytes go through sdma::CommandBuffer, GPU addresses
come from memory::Manager::describe and submission from the caller's Blit2dSubmitter.
Copyright 2020, 2021 Advanced Micro Devices, Inc. Adapted Linux v6.12
sdma_v6_0_0_pkt_open.h and sdma_v6_0.c. Copyright 2015-2021 Advanced Micro Devices,
Inc.; Copyright 2023 Valve Corporation. Adapted Mesa 24.2.0 radv_sdma.c and
si_sdma_copy_image.c (MIT). Notice: licenses/AMDGPU-SDMA.txt.
*/
#include "blit2d.hpp"

namespace rx7900::hardware {
namespace {

// SDMA_OP_COPY | SDMA_SUBOP_COPY_LINEAR_SUB_WIND << 8 | elementsize << 29.
constexpr uint32_t sub_window_header = 1u | (4u << 8) | (blit2d_element_size_log2 << 29);
// SDMA_OP_CONST_FILL | fillsize 2 << 30: radv_sdma_fill_buffer, "count is in
// dwords" of the pattern (still programmed in bytes). Linux leaves fillsize 0
// because it only fills byte-uniform values; GDI colors are arbitrary dwords.
constexpr uint32_t constant_fill_header = 11u | (2u << 30);
constexpr uint32_t nop_header = 0;
constexpr uint32_t coordinate_shift = 16;
constexpr uint32_t pitch_shift = 13;
// Constant-initialized (no static constructor): count 0 means unclipped.
constexpr Blit2dClipList unclipped_list{};

bool aligned4(uint64_t value) noexcept {
    return (value & 3) == 0;
}

bool permits(memory::Access granted, memory::Access needed) noexcept {
    const auto have = static_cast<uint32_t>(granted);
    const auto want = static_cast<uint32_t>(needed);
    return (have & want) == want;
}

bool same_handle(memory::Handle first, memory::Handle second) noexcept {
    return first.slot == second.slot && first.generation == second.generation;
}

bool rect_inverted(const Blit2dRect& rect) noexcept {
    return rect.right < rect.left || rect.bottom < rect.top;
}

bool rect_empty(const Blit2dRect& rect) noexcept {
    return rect.right <= rect.left || rect.bottom <= rect.top;
}

int64_t rect_width(const Blit2dRect& rect) noexcept {
    return int64_t{rect.right} - rect.left;
}

int64_t rect_height(const Blit2dRect& rect) noexcept {
    return int64_t{rect.bottom} - rect.top;
}

int32_t max32(int32_t first, int32_t second) noexcept {
    return first > second ? first : second;
}

int32_t min32(int32_t first, int32_t second) noexcept {
    return first < second ? first : second;
}

Blit2dRect intersect(const Blit2dRect& first, const Blit2dRect& second) noexcept {
    return {max32(first.left, second.left), max32(first.top, second.top),
            min32(first.right, second.right), min32(first.bottom, second.bottom)};
}

bool rects_intersect(const Blit2dRect& first, const Blit2dRect& second) noexcept {
    return !rect_empty(first) && !rect_empty(second) && !rect_empty(intersect(first, second));
}

// Only called for rectangles proven inside a 16384-pixel surface, so the shifted
// coordinates stay far inside int32.
Blit2dRect shifted(const Blit2dRect& rect, int32_t dx, int32_t dy) noexcept {
    return {rect.left - dx, rect.top - dy, rect.right - dx, rect.bottom - dy};
}

// True when first comes after second in GDI enumeration order for a move by
// (dx, dy): rows against the vertical move, then columns against the horizontal.
bool gdi_after(const Blit2dRect& first, const Blit2dRect& second, int32_t dx,
               int32_t dy) noexcept {
    const int32_t first_row = dy > 0 ? -first.top : first.top;
    const int32_t second_row = dy > 0 ? -second.top : second.top;

    if (first_row != second_row) {
        return first_row > second_row;
    }

    return (dx > 0 ? -first.left : first.left) > (dx > 0 ? -second.left : second.left);
}

bool inside(const Blit2dRect& rect, const Blit2dSurface& surface) noexcept {
    return rect.left >= 0 && rect.top >= 0 &&
           int64_t{rect.right} <= int64_t{surface.width} &&
           int64_t{rect.bottom} <= int64_t{surface.height};
}

uint64_t surface_span(const Blit2dSurface& surface) noexcept {
    return uint64_t{surface.pitch_bytes} * (surface.height - 1) +
           uint64_t{surface.width} * blit2d_bytes_per_pixel;
}

bool valid_surface(const Blit2dSurface& surface) noexcept {
    if (surface.width == 0 || surface.height == 0 ||
        surface.width > blit2d_max_surface_dimension ||
        surface.height > blit2d_max_surface_dimension ||
        !aligned4(surface.gpu_address) || !aligned4(surface.pitch_bytes)) {
        return false;
    }

    const uint32_t pitch_elements = surface.pitch_bytes / blit2d_bytes_per_pixel;

    if (pitch_elements < surface.width || pitch_elements > blit2d_max_pitch_elements ||
        surface.gpu_address > sdma::gpu_virtual_address_max) {
        return false;
    }

    return surface_span(surface) - 1 <= sdma::gpu_virtual_address_max - surface.gpu_address;
}

bool spans_overlap(uint64_t first, uint64_t first_bytes,
                   uint64_t second, uint64_t second_bytes) noexcept {
    if (first_bytes == 0 || second_bytes == 0) {
        return false;
    }

    return first <= second ? second - first < first_bytes : first - second < second_bytes;
}

bool identical(const Blit2dSurface& first, const Blit2dSurface& second) noexcept {
    return first.gpu_address == second.gpu_address && first.pitch_bytes == second.pitch_bytes &&
           first.width == second.width && first.height == second.height;
}

// Byte span covered by a non-empty region (first byte of its first row through the
// last byte of its last row).
void region_span(const Blit2dRegion& region, uint64_t& start, uint64_t& bytes) noexcept {
    const uint64_t row_offset = uint64_t{region.pitch_bytes};
    start = region.base + row_offset * static_cast<uint32_t>(region.rect.top) +
            uint64_t{static_cast<uint32_t>(region.rect.left)} * blit2d_bytes_per_pixel;
    const uint64_t end = region.base +
        row_offset * static_cast<uint32_t>(region.rect.bottom - 1) +
        uint64_t{static_cast<uint32_t>(region.rect.right)} * blit2d_bytes_per_pixel;
    bytes = end - start;
}

Blit2dRegion surface_region(const Blit2dSurface& surface, const Blit2dRect& rect) noexcept {
    return {surface.gpu_address, surface.pitch_bytes, surface.height, rect};
}

void write_words(sdma::CommandBuffer output, size_t offset, const uint32_t* words,
                 size_t count) noexcept {
    for (size_t index = 0; index < count; ++index) {
        output.write_u32(offset + index * sizeof(uint32_t), words[index]);
    }
}

bool output_fits(sdma::CommandBuffer output, size_t offset, size_t dwords) noexcept {
    return offset <= output.size() && dwords * sizeof(uint32_t) <= output.size() - offset;
}

bool valid_window_side(uint64_t address, uint32_t x, uint32_t y, uint32_t pitch,
                       uint32_t slice) noexcept {
    return address <= sdma::gpu_virtual_address_max && aligned4(address) &&
           x <= blit2d_max_coordinate && y <= blit2d_max_coordinate &&
           pitch != 0 && pitch <= blit2d_max_pitch_elements &&
           slice != 0 && slice <= blit2d_max_slice_elements;
}

} // namespace

Blit2dStatus encode_sub_window_copy(const Blit2dSubWindow& copy, sdma::CommandBuffer output,
                                    size_t byte_offset) noexcept {
    if (!valid_window_side(copy.source_address, copy.source_x, copy.source_y,
                           copy.source_pitch_elements, copy.source_slice_elements) ||
        !valid_window_side(copy.destination_address, copy.destination_x, copy.destination_y,
                           copy.destination_pitch_elements, copy.destination_slice_elements) ||
        copy.width == 0 || copy.width > blit2d_max_extent ||
        copy.height == 0 || copy.height > blit2d_max_extent ||
        !output_fits(output, byte_offset, blit2d_sub_window_dwords)) {
        return Blit2dStatus::invalid_argument;
    }

    const uint32_t words[blit2d_sub_window_dwords] = {
        sub_window_header,
        static_cast<uint32_t>(copy.source_address),
        static_cast<uint32_t>(copy.source_address >> 32),
        copy.source_x | (copy.source_y << coordinate_shift),
        (copy.source_pitch_elements - 1) << pitch_shift,
        copy.source_slice_elements - 1,
        static_cast<uint32_t>(copy.destination_address),
        static_cast<uint32_t>(copy.destination_address >> 32),
        copy.destination_x | (copy.destination_y << coordinate_shift),
        (copy.destination_pitch_elements - 1) << pitch_shift,
        copy.destination_slice_elements - 1,
        (copy.width - 1) | ((copy.height - 1) << coordinate_shift),
        0
    };
    write_words(output, byte_offset, words, blit2d_sub_window_dwords);
    return Blit2dStatus::success;
}

Blit2dStatus encode_constant_fill(uint64_t address, uint32_t value, uint32_t byte_count,
                                  sdma::CommandBuffer output, size_t byte_offset) noexcept {
    if (address > sdma::gpu_virtual_address_max || !aligned4(address) ||
        byte_count == 0 || !aligned4(byte_count) || byte_count > sdma::max_transfer_bytes ||
        byte_count - 1 > sdma::gpu_virtual_address_max - address ||
        !output_fits(output, byte_offset, blit2d_constant_fill_dwords)) {
        return Blit2dStatus::invalid_argument;
    }

    const uint32_t words[blit2d_constant_fill_dwords] = {
        constant_fill_header,
        static_cast<uint32_t>(address),
        static_cast<uint32_t>(address >> 32),
        value,
        byte_count - 1
    };
    write_words(output, byte_offset, words, blit2d_constant_fill_dwords);
    return Blit2dStatus::success;
}

Blit2dStatus encode_nop(sdma::CommandBuffer output, size_t byte_offset) noexcept {
    if (!output_fits(output, byte_offset, blit2d_nop_dwords)) {
        return Blit2dStatus::invalid_argument;
    }

    output.write_u32(byte_offset, nop_header);
    return Blit2dStatus::success;
}

bool blit2d_regions_overlap(const Blit2dRegion& first, const Blit2dRegion& second) noexcept {
    if (rect_empty(first.rect) || rect_empty(second.rect)) {
        return false;
    }

    if (first.base == second.base && first.pitch_bytes == second.pitch_bytes) {
        return rects_intersect(first.rect, second.rect);
    }

    // One layout seen through two bases (a sample window cropped out of its
    // surface: same pitch, base offset dy * pitch + 4 * dx): translate the
    // higher region into the lower one's pixel grid and compare exactly, as
    // long as no translated row reaches past the pitch (no wrap into the next
    // row); anything else stays a conservative byte-span comparison.
    if (first.pitch_bytes == second.pitch_bytes && first.pitch_bytes % 4u == 0 && first.pitch_bytes != 0 &&
        (first.base > second.base ? first.base - second.base : second.base - first.base) % 4u == 0) {
        const Blit2dRegion& low = first.base < second.base ? first : second;
        const Blit2dRegion& high = first.base < second.base ? second : first;
        const uint64_t delta = high.base - low.base;
        const uint64_t dy = delta / low.pitch_bytes;
        const uint64_t dx = (delta % low.pitch_bytes) / 4u;
        const int64_t columns = low.pitch_bytes / 4u;
        const int64_t shifted_left = int64_t{high.rect.left} + static_cast<int64_t>(dx);
        const int64_t shifted_right = int64_t{high.rect.right} + static_cast<int64_t>(dx);
        if (dy <= static_cast<uint64_t>(INT32_MAX) && high.rect.left >= 0 && low.rect.left >= 0 &&
            shifted_right <= columns && low.rect.right <= columns &&
            int64_t{high.rect.bottom} + static_cast<int64_t>(dy) <= INT32_MAX) {
            const Blit2dRect shifted{static_cast<int32_t>(shifted_left),
                                     static_cast<int32_t>(int64_t{high.rect.top} + static_cast<int64_t>(dy)),
                                     static_cast<int32_t>(shifted_right),
                                     static_cast<int32_t>(int64_t{high.rect.bottom} + static_cast<int64_t>(dy))};
            return rects_intersect(low.rect, shifted);
        }
    }

    uint64_t first_start = 0;
    uint64_t first_bytes = 0;
    uint64_t second_start = 0;
    uint64_t second_bytes = 0;
    region_span(first, first_start, first_bytes);
    region_span(second, second_start, second_bytes);
    return spans_overlap(first_start, first_bytes, second_start, second_bytes);
}

bool Blit2dHazardTracker::conflicts(const Blit2dRegion* read,
                                    const Blit2dRegion& write) const noexcept {
    for (uint32_t index = 0; index < count_; ++index) {
        const auto& entry = entries_[index];

        if (blit2d_regions_overlap(entry.region, write)) {
            return true;
        }

        if (entry.write && read != nullptr && blit2d_regions_overlap(entry.region, *read)) {
            return true;
        }
    }

    return false;
}

void Blit2dHazardTracker::add(const Blit2dRegion& region, bool write) noexcept {
    // Consecutive rows or strips of one rectangle extend the newest entry, so a
    // per-row fill or strip copy occupies one entry instead of forcing barriers.
    if (count_ != 0) {
        auto& last = entries_[count_ - 1];
        const auto previous = last.region.rect;
        const auto& next = region.rect;

        if (last.write == write && last.region.base == region.base &&
            last.region.pitch_bytes == region.pitch_bytes &&
            previous.left == next.left && previous.right == next.right &&
            (previous.bottom == next.top || next.bottom == previous.top)) {
            last.region.rect.top = min32(previous.top, next.top);
            last.region.rect.bottom = max32(previous.bottom, next.bottom);
            return;
        }
    }

    if (count_ < blit2d_hazard_capacity) {
        entries_[count_] = {region, write};
        ++count_;
    }
}

Blit2dStatus Blit2dEngine::initialize(const Blit2dEngineConfig& config) noexcept {
    if (initialized_) {
        return Blit2dStatus::invalid_argument;
    }

    if (config.manager == nullptr || config.submitter.submit == nullptr ||
        config.submitter.query == nullptr || config.submitter.wait == nullptr ||
        config.ib_count < blit2d_min_ib_slots || config.ib_count > blit2d_max_ib_slots ||
        config.ib_bytes < blit2d_min_ib_bytes || config.ib_bytes % sdma6_ib_alignment != 0 ||
        config.ib_bytes / sizeof(uint32_t) > blit2d_max_ib_dwords) {
        return Blit2dStatus::invalid_argument;
    }

    memory::AllocationInfo ib{};
    const uint64_t ring_bytes = uint64_t{config.ib_bytes} * config.ib_count;

    if (config.manager->describe(config.ib_buffer, ib) != memory::Status::success ||
        ib.retiring || ib.backing.cpu_bytes == nullptr ||
        !permits(ib.access, memory::Access::read) ||
        ib.gpu.virtual_address % sdma6_ib_alignment != 0 ||
        ring_bytes > ib.gpu.byte_length || ring_bytes > ib.backing.cpu_byte_count) {
        return Blit2dStatus::invalid_argument;
    }

    const bool has_scratch = config.scratch.slot != UINT32_MAX;
    memory::AllocationInfo scratch{};

    if (has_scratch &&
        (same_handle(config.scratch, config.ib_buffer) ||
         config.manager->describe(config.scratch, scratch) != memory::Status::success ||
         scratch.retiring || !permits(scratch.access, memory::Access::read_write) ||
         !aligned4(scratch.gpu.virtual_address) ||
         scratch.gpu.byte_length < blit2d_bytes_per_pixel)) {
        return Blit2dStatus::invalid_argument;
    }

    if (config.manager->retain(config.ib_buffer, memory::Access::read) !=
        memory::Status::success) {
        return Blit2dStatus::memory_failure;
    }

    if (has_scratch &&
        config.manager->retain(config.scratch, memory::Access::read_write) !=
            memory::Status::success) {
        config.manager->unretain(config.ib_buffer);
        return Blit2dStatus::memory_failure;
    }

    reset_state();
    manager_ = config.manager;
    submitter_ = config.submitter;
    wait_timeout_us_ = config.wait_timeout_us != 0 ? config.wait_timeout_us
                                                   : blit2d_default_wait_us;
    ib_handle_ = config.ib_buffer;
    ib_gpu_ = ib.gpu.virtual_address;
    ib_bytes_total_ = ib.gpu.byte_length;
    // The Manager's CPU descriptor stays valid while the lease is held.
    ib_output_ = sdma::CommandBuffer(ib.backing.cpu_bytes, static_cast<size_t>(ring_bytes));
    ib_count_ = config.ib_count;

    for (uint32_t index = 0; index < ib_count_; ++index) {
        ibs_[index] = {uint64_t{index} * config.ib_bytes,
                       config.ib_bytes / static_cast<uint32_t>(sizeof(uint32_t)), 0, 0, false};
    }

    has_scratch_ = has_scratch;
    scratch_ = config.scratch;
    scratch_gpu_ = has_scratch ? scratch.gpu.virtual_address : 0;
    scratch_bytes_ = has_scratch ? scratch.gpu.byte_length : 0;
    initialized_ = true;
    return Blit2dStatus::success;
}

void Blit2dEngine::reset_state() noexcept {
    manager_ = nullptr;
    submitter_ = {};
    wait_timeout_us_ = 0;

    for (auto& slot : ibs_) {
        slot = {};
    }

    ib_handle_ = {};
    ib_gpu_ = 0;
    ib_bytes_total_ = 0;
    ib_output_ = sdma::CommandBuffer(nullptr, 0);
    ib_count_ = 0;
    next_ = 0;
    submit_order_ = 0;
    open_ = false;
    open_dwords_ = 0;
    scratch_ = {};
    scratch_gpu_ = 0;
    scratch_bytes_ = 0;
    has_scratch_ = false;
    hazards_.clear();
    prepared_.count = 0;
    report_ = {};
    initialized_ = false;
}

Blit2dReport Blit2dEngine::report() const noexcept {
    Blit2dReport report = report_;
    report.in_flight = 0;

    for (uint32_t index = 0; index < ib_count_; ++index) {
        report.in_flight += ibs_[index].in_flight ? 1u : 0u;
    }

    report.open_dwords = open_ ? open_dwords_ : 0;
    return report;
}

sdma::CommandBuffer Blit2dEngine::slot_output(const IbSlot& slot) const noexcept {
    return ib_output_.subbuffer(static_cast<size_t>(slot.offset),
                                size_t{slot.capacity_dwords} * sizeof(uint32_t));
}

Blit2dStatus Blit2dEngine::usable() const noexcept {
    if (!initialized_) {
        return Blit2dStatus::not_initialized;
    }

    return report_.faulted ? Blit2dStatus::faulted : Blit2dStatus::success;
}

Blit2dStatus Blit2dEngine::fault(Blit2dStatus reason) noexcept {
    report_.faulted = true;
    report_.last_failure = reason;
    return Blit2dStatus::faulted;
}

bool Blit2dEngine::touches_engine_memory(const Blit2dSurface& surface) const noexcept {
    const uint64_t bytes = surface_span(surface);

    return spans_overlap(surface.gpu_address, bytes, ib_gpu_, ib_bytes_total_) ||
           (has_scratch_ &&
            spans_overlap(surface.gpu_address, bytes, scratch_gpu_, scratch_bytes_));
}

Blit2dStatus Blit2dEngine::prepare_rects(const Blit2dSurface& destination,
                                         const Blit2dRect& bounds,
                                         const Blit2dClipList& clip) noexcept {
    prepared_.count = 0;

    if (rect_inverted(bounds)) {
        return Blit2dStatus::invalid_rect;
    }

    if (clip.count > blit2d_max_rects) {
        return Blit2dStatus::too_many_rects;
    }

    const uint32_t inputs = clip.count == 0 ? 1 : clip.count;
    uint32_t count = 0;

    for (uint32_t index = 0; index < inputs; ++index) {
        Blit2dRect rect = bounds;

        if (clip.count != 0) {
            if (rect_inverted(clip.rects[index])) {
                return Blit2dStatus::invalid_rect;
            }

            rect = intersect(bounds, clip.rects[index]);
        }

        if (rect_empty(rect)) {
            continue;
        }

        if (!inside(rect, destination)) {
            return Blit2dStatus::out_of_bounds;
        }

        prepared_.rects[count] = rect;
        prepared_.order[count] = static_cast<uint8_t>(count);
        ++count;
    }

    prepared_.count = count;
    return Blit2dStatus::success;
}

// GDI enumerates same-surface clip rectangles against the move direction (bottom
// to top when moving down, right to left when moving right). That order is the
// tie-break of a dependency sort: rectangle a must precede b when b's destination
// overlaps a's source, so every source is read before anything overwrites it.
// Banded GDI regions always satisfy the GDI order; a cyclic dependency (possible
// only for unbanded lists) is refused rather than approximated.
Blit2dStatus Blit2dEngine::order_rects(int32_t dx, int32_t dy) noexcept {
    const uint32_t count = prepared_.count;
    uint8_t sorted[blit2d_max_rects]{};
    uint8_t pending[blit2d_max_rects]{};
    bool picked[blit2d_max_rects]{};

    for (uint32_t index = 0; index < count; ++index) {
        uint32_t position = index;

        while (position > 0 &&
               gdi_after(prepared_.rects[sorted[position - 1]], prepared_.rects[index], dx, dy)) {
            sorted[position] = sorted[position - 1];
            --position;
        }

        sorted[position] = static_cast<uint8_t>(index);
    }

    for (uint32_t later = 0; later < count; ++later) {
        for (uint32_t earlier = 0; earlier < count; ++earlier) {
            if (earlier != later &&
                rects_intersect(prepared_.rects[later],
                                shifted(prepared_.rects[earlier], dx, dy))) {
                ++pending[later];
            }
        }
    }

    for (uint32_t output = 0; output < count; ++output) {
        uint32_t chosen = blit2d_max_rects;

        for (uint32_t position = 0; position < count; ++position) {
            const uint32_t candidate = sorted[position];

            if (!picked[candidate] && pending[candidate] == 0) {
                chosen = candidate;
                break;
            }
        }

        if (chosen == blit2d_max_rects) {
            return Blit2dStatus::unordered_overlap;
        }

        picked[chosen] = true;
        prepared_.order[output] = static_cast<uint8_t>(chosen);
        const Blit2dRect source = shifted(prepared_.rects[chosen], dx, dy);

        for (uint32_t later = 0; later < count; ++later) {
            if (!picked[later] && rects_intersect(prepared_.rects[later], source)) {
                --pending[later];
            }
        }
    }

    return Blit2dStatus::success;
}

Blit2dStatus Blit2dEngine::fill(const Blit2dFill& fill, const Blit2dClipList& clip) noexcept {
    auto status = usable();

    if (status != Blit2dStatus::success) {
        return status;
    }

    if (!valid_surface(fill.destination)) {
        return Blit2dStatus::invalid_surface;
    }

    if (touches_engine_memory(fill.destination)) {
        return Blit2dStatus::engine_memory_overlap;
    }

    status = prepare_rects(fill.destination, fill.bounds, clip);

    if (status != Blit2dStatus::success) {
        return status;
    }

    const uint32_t pitch = fill.destination.pitch_bytes;

    for (uint32_t index = 0; index < prepared_.count; ++index) {
        const auto& rect = prepared_.rects[index];
        const uint64_t row_bytes = static_cast<uint64_t>(rect_width(rect)) * blit2d_bytes_per_pixel;
        // A rectangle as wide as the pitch is one contiguous run of rows.
        const int32_t rows_per_packet = row_bytes == pitch
            ? static_cast<int32_t>(sdma::max_transfer_bytes / pitch) : 1;

        for (int32_t top = rect.top; top < rect.bottom; top += rows_per_packet) {
            const int32_t bottom = rect.bottom - top > rows_per_packet
                ? top + rows_per_packet : rect.bottom;
            Packet packet{};
            packet.copy = false;
            packet.write = surface_region(fill.destination, {rect.left, top, rect.right, bottom});
            packet.value = fill.color;
            status = emit(packet);

            if (status != Blit2dStatus::success) {
                return status;
            }
        }
    }

    ++report_.operations;
    return Blit2dStatus::success;
}

Blit2dStatus Blit2dEngine::copy(const Blit2dCopy& copy, const Blit2dClipList& clip) noexcept {
    const auto status = usable();
    return status == Blit2dStatus::success ? run_copy(copy, clip) : status;
}

Blit2dStatus Blit2dEngine::run_copy(const Blit2dCopy& copy, const Blit2dClipList& clip) noexcept {
    if (!valid_surface(copy.source) || !valid_surface(copy.destination)) {
        return Blit2dStatus::invalid_surface;
    }

    const bool same = identical(copy.source, copy.destination);

    if (!same && spans_overlap(copy.source.gpu_address, surface_span(copy.source),
                               copy.destination.gpu_address, surface_span(copy.destination))) {
        return Blit2dStatus::overlapping_surfaces;
    }

    if (touches_engine_memory(copy.source) || touches_engine_memory(copy.destination)) {
        return Blit2dStatus::engine_memory_overlap;
    }

    auto status = prepare_rects(copy.destination, copy.destination_rect, clip);

    if (status != Blit2dStatus::success) {
        return status;
    }

    const int64_t dx64 = int64_t{copy.destination_rect.left} - copy.source_point.x;
    const int64_t dy64 = int64_t{copy.destination_rect.top} - copy.source_point.y;

    for (uint32_t index = 0; index < prepared_.count; ++index) {
        const auto& rect = prepared_.rects[index];

        if (rect.left - dx64 < 0 || rect.top - dy64 < 0 ||
            rect.right - dx64 > int64_t{copy.source.width} ||
            rect.bottom - dy64 > int64_t{copy.source.height}) {
            return Blit2dStatus::out_of_bounds;
        }
    }

    if (prepared_.count == 0) {
        ++report_.operations;
        return Blit2dStatus::success;
    }

    // Both rectangles lie inside 16384-pixel surfaces, so |dx|, |dy| < 2^15.
    const auto dx = static_cast<int32_t>(dx64);
    const auto dy = static_cast<int32_t>(dy64);

    if (same) {
        status = order_rects(dx, dy);

        if (status != Blit2dStatus::success) {
            return status;
        }
    }

    for (uint32_t index = 0; index < prepared_.count; ++index) {
        status = emit_rect_copy(copy, prepared_.rects[prepared_.order[index]], dx, dy, same);

        if (status != Blit2dStatus::success) {
            return status;
        }
    }

    ++report_.operations;
    return Blit2dStatus::success;
}

Blit2dStatus Blit2dEngine::emit_rect_copy(const Blit2dCopy& copy, const Blit2dRect& rect,
                                          int32_t dx, int32_t dy, bool same) noexcept {
    if (same && dx == 0 && dy == 0) {
        return Blit2dStatus::success;
    }

    const int64_t width = rect_width(rect);
    const int64_t height = rect_height(rect);
    const int64_t abs_dx = dx < 0 ? -int64_t{dx} : dx;
    const int64_t abs_dy = dy < 0 ? -int64_t{dy} : dy;
    Packet packet{};
    packet.copy = true;

    if (!same || abs_dx >= width || abs_dy >= height) {
        packet.read = surface_region(copy.source, shifted(rect, dx, dy));
        packet.write = surface_region(copy.destination, rect);
        return emit(packet);
    }

    // The rectangle overlaps its own source: one packet's internal order is not
    // specified, so split into dependent packets.
    // In-surface extents and moves are below 2^15: 32-bit division (no i386
    // 64-bit division helper in kernel code).
    const auto extent = static_cast<uint32_t>(abs_dy != 0 ? height : width);
    const auto step = static_cast<uint32_t>(abs_dy != 0 ? abs_dy : abs_dx);
    const uint32_t strips = (extent + step - 1) / step;
    const uint64_t bounce_bytes = static_cast<uint64_t>(width) * height * blit2d_bytes_per_pixel;

    if (strips <= 2 || !has_scratch_ || bounce_bytes > scratch_bytes_) {
        ++report_.strip_rects;
        return emit_strips(copy, rect, dx, dy);
    }

    ++report_.bounced_rects;
    const Blit2dRegion scratch{scratch_gpu_,
                               static_cast<uint32_t>(width) * blit2d_bytes_per_pixel,
                               static_cast<uint32_t>(height),
                               {0, 0, static_cast<int32_t>(width), static_cast<int32_t>(height)}};
    packet.read = surface_region(copy.source, shifted(rect, dx, dy));
    packet.write = scratch;
    const auto status = emit(packet);

    if (status != Blit2dStatus::success) {
        return status;
    }

    packet.read = scratch;
    packet.write = surface_region(copy.destination, rect);
    return emit(packet);
}

// Strips of |dy| rows (|dx| columns when dy is 0) never overlap their own source;
// the strip farthest along the move direction goes first, and the hazard tracker
// separates each strip that overwrites an earlier strip's source by a barrier.
Blit2dStatus Blit2dEngine::emit_strips(const Blit2dCopy& copy, const Blit2dRect& rect,
                                       int32_t dx, int32_t dy) noexcept {
    const bool rows = dy != 0;
    const int32_t step = rows ? (dy < 0 ? -dy : dy) : (dx < 0 ? -dx : dx);
    const bool from_end = rows ? dy > 0 : dx > 0;
    const int32_t first = rows ? rect.top : rect.left;
    const int32_t last = rows ? rect.bottom : rect.right;
    Packet packet{};
    packet.copy = true;

    for (int32_t done = 0; done < last - first; done += step) {
        const int32_t remaining = last - first - done;
        const int32_t length = remaining < step ? remaining : step;
        const int32_t start = from_end ? last - done - length : first + done;
        Blit2dRect strip = rect;

        if (rows) {
            strip.top = start;
            strip.bottom = start + length;
        } else {
            strip.left = start;
            strip.right = start + length;
        }

        packet.read = surface_region(copy.source, shifted(strip, dx, dy));
        packet.write = surface_region(copy.destination, strip);
        const auto status = emit(packet);

        if (status != Blit2dStatus::success) {
            return status;
        }
    }

    return Blit2dStatus::success;
}

Blit2dStatus Blit2dEngine::resolve_staging(const Blit2dStaging& staging, memory::Access access,
                                           Blit2dSurface& surface) const noexcept {
    memory::AllocationInfo info{};

    if (manager_->describe(staging.allocation, info) != memory::Status::success ||
        info.retiring || info.backing.cpu_bytes == nullptr || !permits(info.access, access) ||
        staging.byte_offset > info.gpu.byte_length) {
        return Blit2dStatus::invalid_staging;
    }

    surface = {info.gpu.virtual_address + staging.byte_offset, staging.pitch_bytes,
               staging.width, staging.height};

    if (!valid_surface(surface)) {
        return Blit2dStatus::invalid_surface;
    }

    return surface_span(surface) <= info.gpu.byte_length - staging.byte_offset
        ? Blit2dStatus::success : Blit2dStatus::invalid_staging;
}

Blit2dStatus Blit2dEngine::upload(const Blit2dStaging& staging, Blit2dPoint staging_point,
                                  const Blit2dSurface& destination,
                                  const Blit2dRect& destination_rect,
                                  const Blit2dClipList& clip) noexcept {
    auto status = usable();

    if (status != Blit2dStatus::success) {
        return status;
    }

    Blit2dCopy copy{{}, destination, destination_rect, staging_point};
    status = resolve_staging(staging, memory::Access::read, copy.source);
    return status == Blit2dStatus::success ? run_copy(copy, clip) : status;
}

Blit2dStatus Blit2dEngine::readback(const Blit2dSurface& source, const Blit2dRect& source_rect,
                                    const Blit2dStaging& staging,
                                    Blit2dPoint staging_point) noexcept {
    auto status = usable();

    if (status != Blit2dStatus::success) {
        return status;
    }

    if (rect_inverted(source_rect)) {
        return Blit2dStatus::invalid_rect;
    }

    const int64_t right = int64_t{staging_point.x} + rect_width(source_rect);
    const int64_t bottom = int64_t{staging_point.y} + rect_height(source_rect);

    if (right > INT32_MAX || bottom > INT32_MAX) {
        return Blit2dStatus::out_of_bounds;
    }

    Blit2dCopy copy{source, {},
                    {staging_point.x, staging_point.y, static_cast<int32_t>(right),
                     static_cast<int32_t>(bottom)},
                    {source_rect.left, source_rect.top}};
    status = resolve_staging(staging, memory::Access::write, copy.destination);

    if (status != Blit2dStatus::success) {
        return status;
    }

    return run_copy(copy, unclipped_list);
}

Blit2dStatus Blit2dEngine::emit(const Packet& packet) noexcept {
    const bool conflict = hazards_.conflicts(packet.copy ? &packet.read : nullptr, packet.write);

    if (conflict || !hazards_.has_room(2)) {
        const auto status = emit_barrier();

        if (status != Blit2dStatus::success) {
            return status;
        }
    }

    const uint32_t dwords = packet.copy ? blit2d_sub_window_dwords : blit2d_constant_fill_dwords;
    auto status = reserve(dwords);

    if (status != Blit2dStatus::success) {
        return status;
    }

    auto& slot = ibs_[next_];
    const sdma::CommandBuffer output = slot_output(slot);
    const size_t offset = size_t{open_dwords_} * sizeof(uint32_t);
    const auto& write = packet.write;

    if (packet.copy) {
        const auto& read = packet.read;
        const uint32_t read_pitch = read.pitch_bytes / blit2d_bytes_per_pixel;
        const uint32_t write_pitch = write.pitch_bytes / blit2d_bytes_per_pixel;
        const Blit2dSubWindow window{
            read.base, static_cast<uint32_t>(read.rect.left), static_cast<uint32_t>(read.rect.top),
            read_pitch, read_pitch * read.surface_height,
            write.base, static_cast<uint32_t>(write.rect.left), static_cast<uint32_t>(write.rect.top),
            write_pitch, write_pitch * write.surface_height,
            static_cast<uint32_t>(rect_width(write.rect)),
            static_cast<uint32_t>(rect_height(write.rect))};
        status = encode_sub_window_copy(window, output, offset);
        ++report_.copy_packets;
    } else {
        uint64_t start = 0;
        uint64_t bytes = 0;
        region_span(write, start, bytes);
        status = encode_constant_fill(start, packet.value, static_cast<uint32_t>(bytes),
                                      output, offset);
        ++report_.fill_packets;
    }

    if (status != Blit2dStatus::success) {
        // Validation guarantees encodable packets; anything else is a defect.
        return fault(status);
    }

    open_dwords_ += dwords;

    if (packet.copy) {
        hazards_.add(packet.read, false);
    }

    hazards_.add(packet.write, true);
    return Blit2dStatus::success;
}

Blit2dStatus Blit2dEngine::emit_barrier() noexcept {
    const auto status = reserve(blit2d_nop_dwords);

    if (status != Blit2dStatus::success) {
        return status;
    }

    auto& slot = ibs_[next_];
    const sdma::CommandBuffer output = slot_output(slot);
    encode_nop(output, size_t{open_dwords_} * sizeof(uint32_t));
    open_dwords_ += blit2d_nop_dwords;
    ++report_.barrier_nops;
    hazards_.clear();
    return Blit2dStatus::success;
}

Blit2dStatus Blit2dEngine::reserve(uint32_t dwords) noexcept {
    if (!open_) {
        const auto status = acquire();

        if (status != Blit2dStatus::success) {
            return status;
        }
    }

    if (ibs_[next_].capacity_dwords - open_dwords_ >= dwords) {
        return Blit2dStatus::success;
    }

    auto status = submit_open();
    return status == Blit2dStatus::success ? acquire() : status;
}

Blit2dStatus Blit2dEngine::acquire() noexcept {
    auto& slot = ibs_[next_];

    if (slot.in_flight) {
        const auto status = wait_slot(slot);

        if (status != Blit2dStatus::success) {
            return status;
        }
    }

    open_ = true;
    open_dwords_ = 0;
    return Blit2dStatus::success;
}

Blit2dStatus Blit2dEngine::wait_slot(IbSlot& slot) noexcept {
    auto state = submitter_.query(submitter_.context, slot.sequence);

    if (state == Blit2dFenceState::pending) {
        ++report_.waits;
        state = submitter_.wait(submitter_.context, slot.sequence, wait_timeout_us_);
    }

    if (state != Blit2dFenceState::signalled) {
        return fault(state == Blit2dFenceState::pending ? Blit2dStatus::fence_timeout
                                                        : Blit2dStatus::fence_failed);
    }

    slot.in_flight = false;
    return Blit2dStatus::success;
}

// sdma_v6_0_ring_pad_ib: pad to a multiple of eight dwords. Single NOPs are valid
// with or without burst-NOP firmware support.
Blit2dStatus Blit2dEngine::submit_open() noexcept {
    auto& slot = ibs_[next_];
    const sdma::CommandBuffer output = slot_output(slot);

    while (open_dwords_ % sdma6_ib_dword_multiple != 0) {
        encode_nop(output, size_t{open_dwords_} * sizeof(uint32_t));
        ++open_dwords_;
        ++report_.padding_nops;
    }

    for (uint32_t attempt = 0; attempt <= ib_count_; ++attempt) {
        uint32_t sequence = 0;
        const auto result = submitter_.submit(submitter_.context, ib_handle_, slot.offset,
                                              open_dwords_, sequence);

        if (result == Blit2dSubmitResult::submitted) {
            slot.in_flight = true;
            slot.sequence = sequence;
            slot.order = ++submit_order_;
            ++report_.ib_submissions;
            report_.submitted_dwords += open_dwords_;
            open_ = false;
            open_dwords_ = 0;
            next_ = (next_ + 1) % ib_count_;
            return Blit2dStatus::success;
        }

        if (result != Blit2dSubmitResult::busy) {
            return fault(Blit2dStatus::submit_failed);
        }

        // Ring full: wait for this engine's oldest IB, then retry. With nothing of
        // ours in flight the ring is held by other submitters; fail closed.
        ++report_.busy_retries;
        IbSlot* oldest = nullptr;

        for (uint32_t index = 0; index < ib_count_; ++index) {
            auto& candidate = ibs_[index];

            if (candidate.in_flight && (oldest == nullptr || candidate.order < oldest->order)) {
                oldest = &candidate;
            }
        }

        if (oldest == nullptr) {
            return fault(Blit2dStatus::submit_failed);
        }

        const auto status = wait_slot(*oldest);

        if (status != Blit2dStatus::success) {
            return status;
        }
    }

    return fault(Blit2dStatus::submit_failed);
}

void Blit2dEngine::refresh_completion() noexcept {
    if (open_ && open_dwords_ != 0) {
        return;
    }

    for (uint32_t index = 0; index < ib_count_; ++index) {
        if (ibs_[index].in_flight) {
            return;
        }
    }

    hazards_.clear();
}

Blit2dStatus Blit2dEngine::flush() noexcept {
    const auto status = usable();

    if (status != Blit2dStatus::success) {
        return status;
    }

    return open_ && open_dwords_ != 0 ? submit_open() : Blit2dStatus::success;
}

Blit2dStatus Blit2dEngine::synchronize() noexcept {
    auto status = flush();

    if (status != Blit2dStatus::success) {
        return status;
    }

    for (;;) {
        IbSlot* oldest = nullptr;

        for (uint32_t index = 0; index < ib_count_; ++index) {
            auto& candidate = ibs_[index];

            if (candidate.in_flight && (oldest == nullptr || candidate.order < oldest->order)) {
                oldest = &candidate;
            }
        }

        if (oldest == nullptr) {
            break;
        }

        status = wait_slot(*oldest);

        if (status != Blit2dStatus::success) {
            return status;
        }
    }

    refresh_completion();
    return Blit2dStatus::success;
}

Blit2dStatus Blit2dEngine::retire() noexcept {
    const auto status = usable();

    if (status != Blit2dStatus::success) {
        return status;
    }

    for (uint32_t index = 0; index < ib_count_; ++index) {
        auto& slot = ibs_[index];

        if (!slot.in_flight) {
            continue;
        }

        const auto state = submitter_.query(submitter_.context, slot.sequence);

        if (state == Blit2dFenceState::failed) {
            return fault(Blit2dStatus::fence_failed);
        }

        slot.in_flight = state != Blit2dFenceState::signalled;
    }

    refresh_completion();
    return Blit2dStatus::success;
}

void Blit2dEngine::release_leases() noexcept {
    manager_->unretain(ib_handle_);

    if (has_scratch_) {
        manager_->unretain(scratch_);
    }
}

Blit2dStatus Blit2dEngine::shutdown() noexcept {
    const auto status = retire();

    if (status != Blit2dStatus::success) {
        return status;
    }

    if (report().in_flight != 0 || (open_ && open_dwords_ != 0)) {
        return Blit2dStatus::busy;
    }

    release_leases();
    reset_state();
    return Blit2dStatus::success;
}

Blit2dStatus Blit2dEngine::release_after_reset() noexcept {
    if (!initialized_) {
        return Blit2dStatus::not_initialized;
    }

    release_leases();
    reset_state();
    return Blit2dStatus::success;
}

} // namespace rx7900::hardware

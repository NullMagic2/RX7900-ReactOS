/*
Module: rx7900.acceleration.sdma
Purpose: Validate GPU ranges and encode little-endian SDMA copy/fill/fence packets.
Created: 2026-10-21
Architecture: Allocation-free command construction, not a hardware backend.
Adapted from Linux v6.12 sdma_v6_0.c and sdma_v6_0_0_pkt_open.h.
Copyright 2020, 2021 Advanced Micro Devices, Inc.
Permission notice: licenses/AMDGPU-SDMA.txt. Provenance: THIRD_PARTY.txt.
*/
#include "sdma.hpp"

namespace rx7900 {
namespace sdma {
namespace {

constexpr uint32_t copy_header = 1;
constexpr uint32_t fill_header = 11;
constexpr uint32_t fence_header = 5 | (3 << 16);
constexpr uint32_t trap_header = 6;

struct TransferPlan final {
    uint64_t source;
    uint64_t destination;
    uint64_t byte_count;
    uint32_t value;
    bool copy;
    size_t bytes;
};

struct FencePlan final {
    uint64_t destination;
    uint64_t sequence;
    uint32_t byte_count;
    bool interrupt;
    size_t bytes;
};

EncodeStatus resolve_range(const GpuRange& range, uint64_t count,
                           uint64_t& address) noexcept {
    const auto& allocation = range.allocation;

    if (allocation.byte_length == 0 ||
        allocation.virtual_address > gpu_virtual_address_max ||
        allocation.byte_length - 1 > gpu_virtual_address_max - allocation.virtual_address ||
        range.byte_offset > allocation.byte_length ||
        count == 0 || count > allocation.byte_length - range.byte_offset) {
        return EncodeStatus::invalid_range;
    }

    address = allocation.virtual_address + range.byte_offset;
    return EncodeStatus::success;
}

bool overlaps(uint64_t first, uint64_t first_count,
              uint64_t second, uint64_t second_count) noexcept {
    return first <= second ? second - first < first_count
                           : first - second < second_count;
}

EncodeStatus transfer_size(uint64_t count, size_t packet_bytes,
                           size_t& bytes) noexcept {
    const uint64_t packets = count / max_transfer_bytes +
                             (count % max_transfer_bytes != 0 ? 1 : 0);

    if (packets > SIZE_MAX / packet_bytes) {
        return EncodeStatus::size_overflow;
    }

    bytes = static_cast<size_t>(packets) * packet_bytes;
    return EncodeStatus::success;
}

EncodeStatus plan_copy(const CopyRequest& request, TransferPlan& plan) noexcept {
    if (request.byte_count == 0) {
        return EncodeStatus::invalid_count;
    }

    auto status = resolve_range(request.source, request.byte_count, plan.source);

    if (status != EncodeStatus::success) {
        return status;
    }

    status = resolve_range(request.destination, request.byte_count, plan.destination);

    if (status != EncodeStatus::success) {
        return status;
    }

    if (overlaps(plan.source, request.byte_count, plan.destination, request.byte_count)) {
        return EncodeStatus::overlapping_ranges;
    }

    plan.copy = true;
    plan.byte_count = request.byte_count;
    return transfer_size(plan.byte_count, copy_packet_bytes, plan.bytes);
}

EncodeStatus plan_fill(const FillRequest& request, TransferPlan& plan) noexcept {
    if (request.byte_count == 0) {
        return EncodeStatus::invalid_count;
    }

    auto status = resolve_range(request.destination, request.byte_count, plan.destination);

    if (status != EncodeStatus::success) {
        return status;
    }

    if ((plan.destination & 3) != 0 || (request.byte_count & 3) != 0) {
        return EncodeStatus::invalid_alignment;
    }

    plan.copy = false;
    plan.byte_count = request.byte_count;
    plan.value = request.value;
    return transfer_size(plan.byte_count, fill_packet_bytes, plan.bytes);
}

EncodeStatus plan_fence(const FenceRequest& request, FencePlan& plan) noexcept {
    switch (request.width) {
    case FenceWidth::bits32:
        if (request.sequence > UINT32_MAX) {
            return EncodeStatus::invalid_count;
        }
        plan.byte_count = 4;
        break;
    case FenceWidth::bits64:
        plan.byte_count = 8;
        break;
    default:
        return EncodeStatus::invalid_fence_width;
    }

    auto status = resolve_range(request.destination, plan.byte_count, plan.destination);

    if (status != EncodeStatus::success) {
        return status;
    }

    if ((plan.destination & 3) != 0) {
        return EncodeStatus::invalid_alignment;
    }

    plan.sequence = request.sequence;
    plan.interrupt = request.interrupt;
    plan.bytes = fence32_packet_bytes * (plan.byte_count / 4) +
                 (plan.interrupt ? trap_packet_bytes : 0);
    return EncodeStatus::success;
}

// Only instantiated after the complete size has been checked. Each write still
// checks its bounds at the only raw access boundary, CommandBuffer::write_u32.
class Writer final {
public:
    explicit Writer(CommandBuffer output) noexcept : output_(output), offset_(0) {}

    void word(uint32_t value) noexcept {
        output_.write_u32(offset_, value);
        offset_ += sizeof(value);
    }

    void address(uint64_t value) noexcept {
        word(static_cast<uint32_t>(value));
        word(static_cast<uint32_t>(value >> 32));
    }

private:
    CommandBuffer output_;
    size_t offset_;
};

void write_transfer(TransferPlan plan, Writer& writer) noexcept {
    while (plan.byte_count != 0) {
        const uint32_t count = plan.byte_count > max_transfer_bytes
            ? max_transfer_bytes : static_cast<uint32_t>(plan.byte_count);

        if (plan.copy) {
            writer.word(copy_header);
            writer.word(count - 1);
            writer.word(0);
            writer.address(plan.source);
            writer.address(plan.destination);
        } else {
            writer.word(fill_header);
            writer.address(plan.destination);
            writer.word(plan.value);
            writer.word(count - 1);
        }

        plan.source += count;
        plan.destination += count;
        plan.byte_count -= count;
    }
}

void write_fence(const FencePlan& plan, Writer& writer) noexcept {
    writer.word(fence_header);
    writer.address(plan.destination);
    writer.word(static_cast<uint32_t>(plan.sequence));

    if (plan.byte_count == 8) {
        writer.word(fence_header);
        writer.address(plan.destination + 4);
        writer.word(static_cast<uint32_t>(plan.sequence >> 32));
    }

    if (plan.interrupt) {
        writer.word(trap_header);
        writer.word(0);
    }
}

EncodeStatus encode_plans(const TransferPlan* transfer, const FencePlan* fence,
                          CommandBuffer output, EncodeResult& result) noexcept {
    const size_t transfer_bytes = transfer != nullptr ? transfer->bytes : 0;
    const size_t fence_bytes = fence != nullptr ? fence->bytes : 0;

    if (transfer_bytes > SIZE_MAX - fence_bytes) {
        return EncodeStatus::size_overflow;
    }

    result.bytes_required = transfer_bytes + fence_bytes;

    if (output.size() < result.bytes_required) {
        return EncodeStatus::output_too_small;
    }

    Writer writer(output);

    if (transfer != nullptr) {
        write_transfer(*transfer, writer);
    }

    if (fence != nullptr) {
        write_fence(*fence, writer);
    }

    result.bytes_written = result.bytes_required;
    return EncodeStatus::success;
}

EncodeStatus encode_submission(EncodeStatus transfer_status, const TransferPlan& transfer,
                                const FenceRequest& request, CommandBuffer output,
                                EncodeResult& result) noexcept {
    if (transfer_status != EncodeStatus::success) {
        return transfer_status;
    }

    FencePlan fence{};
    const auto status = plan_fence(request, fence);

    if (status != EncodeStatus::success) {
        return status;
    }

    if (overlaps(transfer.destination, transfer.byte_count, fence.destination, fence.byte_count) ||
        (transfer.copy && overlaps(transfer.source, transfer.byte_count,
                                   fence.destination, fence.byte_count))) {
        return EncodeStatus::overlapping_ranges;
    }

    return encode_plans(&transfer, &fence, output, result);
}

} // namespace

EncodeStatus encode_copy(const CopyRequest& request, CommandBuffer output,
                         EncodeResult& result) noexcept {
    result = {};
    TransferPlan plan{};
    const auto status = plan_copy(request, plan);
    return status == EncodeStatus::success
        ? encode_plans(&plan, nullptr, output, result) : status;
}

EncodeStatus encode_fill(const FillRequest& request, CommandBuffer output,
                         EncodeResult& result) noexcept {
    result = {};
    TransferPlan plan{};
    const auto status = plan_fill(request, plan);
    return status == EncodeStatus::success
        ? encode_plans(&plan, nullptr, output, result) : status;
}

EncodeStatus encode_fence(const FenceRequest& request, CommandBuffer output,
                          EncodeResult& result) noexcept {
    result = {};
    FencePlan plan{};
    const auto status = plan_fence(request, plan);
    return status == EncodeStatus::success
        ? encode_plans(nullptr, &plan, output, result) : status;
}

EncodeStatus encode_cache_sync(CommandBuffer output, EncodeResult& result) noexcept {
    result = {0, cache_sync_packet_bytes};

    if (output.size() < cache_sync_packet_bytes) {
        return EncodeStatus::output_too_small;
    }

    // sdma_v6_0_ring_emit_mem_sync: GL2_INV/WB, GLM/GL1/GLV/GLK_INV,
    // and GLI_INV(1). Whole-cache operation uses zero base/limit and VMID0.
    constexpr uint32_t gcr_control = (1u << 14) | (1u << 15) | (1u << 5) |
                                     (1u << 9) | (1u << 8) | (1u << 7) | 1u;
    Writer writer(output);
    writer.word(17);
    writer.word(0);
    writer.word(gcr_control << 16);
    writer.word(0);
    writer.word(0);
    result.bytes_written = cache_sync_packet_bytes;
    return EncodeStatus::success;
}

EncodeStatus encode_copy_submission(const CopyRequest& request,
                                    const FenceRequest& fence, CommandBuffer output,
                                    EncodeResult& result) noexcept {
    result = {};
    TransferPlan plan{};
    const auto status = plan_copy(request, plan);
    return encode_submission(status, plan, fence, output, result);
}

EncodeStatus encode_fill_submission(const FillRequest& request,
                                    const FenceRequest& fence, CommandBuffer output,
                                    EncodeResult& result) noexcept {
    result = {};
    TransferPlan plan{};
    const auto status = plan_fill(request, plan);
    return encode_submission(status, plan, fence, output, result);
}

} // namespace sdma
} // namespace rx7900

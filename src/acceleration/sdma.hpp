/*
Module: rx7900.acceleration.sdma
Purpose: Construct bounded Linux-derived SDMA 6.0 command streams.
Created: 2026-10-21
Architecture: Pure C++ encoder; owns no GPU resources and submits no hardware work.
*/
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace rx7900 {
namespace sdma {

constexpr uint64_t gpu_virtual_address_max = 0x0000ffffffffffffULL;
constexpr uint32_t max_transfer_bytes = 0x400000;
constexpr size_t copy_packet_bytes = 7 * sizeof(uint32_t);
constexpr size_t fill_packet_bytes = 5 * sizeof(uint32_t);
constexpr size_t fence32_packet_bytes = 4 * sizeof(uint32_t);
constexpr size_t trap_packet_bytes = 2 * sizeof(uint32_t);
constexpr size_t cache_sync_packet_bytes = 5 * sizeof(uint32_t);

enum class EncodeStatus {
    success,
    invalid_range,
    invalid_count,
    invalid_alignment,
    overlapping_ranges,
    invalid_fence_width,
    size_overflow,
    output_too_small
};

// Describes caller-owned, already pinned GPUVM memory in the target address space.
// This is a GPU virtual address, never a PCI BAR, CPU address or user pointer.
// An encoder can check numerical bounds, not prove mappings or access permissions.
struct GpuAllocation final {
    uint64_t virtual_address;
    uint64_t byte_length;
};

struct GpuRange final {
    GpuAllocation allocation;
    uint64_t byte_offset;
};

struct CopyRequest final {
    GpuRange source;
    GpuRange destination;
    uint64_t byte_count;
};

// Repeats a 32-bit pattern; address and byte_count must be dword aligned.
struct FillRequest final {
    GpuRange destination;
    uint64_t byte_count;
    uint32_t value;
};

enum class FenceWidth {
    bits32,
    bits64
};

// A 64-bit fence is two ordered 32-bit writes, not an atomic 64-bit write.
// bits32 rejects values above UINT32_MAX instead of silently truncating them.
// interrupt appends a legacy (non-MES) TRAP with context zero; it requires a
// configured interrupt route and handler before the commands may be submitted.
struct FenceRequest final {
    GpuRange destination;
    uint64_t sequence;
    FenceWidth width;
    bool interrupt;
};

class CommandBuffer final {
public:
    // Caller keeps size writable bytes alive, exclusively owned for this call.
    // Storage must not overlap request/result objects. Null always means empty.
    constexpr CommandBuffer(uint8_t* data, size_t size) noexcept
        : data_(data), size_(data != nullptr ? size : 0) {}

    constexpr size_t size() const noexcept { return size_; }

    CommandBuffer subbuffer(size_t offset, size_t count) const noexcept {
        if (offset > size_ || count > size_ - offset || data_ == nullptr) {
            return {nullptr, 0};
        }

        // The parent view bounds and lifetime contract cover this child view.
        return {data_ + offset, count};
    }

    bool write_u32(size_t offset, uint32_t value) const noexcept {
        if (offset > size_ || sizeof(value) > size_ - offset) {
            return false;
        }

        // Constructor lifetime/exclusive-access contract and bounds cover every
        // raw store. Unaligned CPU storage is valid; output is little-endian.
        for (size_t index = 0; index < sizeof(value); ++index) {
            data_[offset + index] = static_cast<uint8_t>(value >> (index * 8));
        }

        return true;
    }

private:
    uint8_t* data_;
    size_t size_;
};

struct EncodeResult final {
    size_t bytes_written;
    size_t bytes_required;
};

// All functions validate before writing. On failure bytes_written is zero and
// output bytes are unchanged; output_too_small reports bytes_required. Successful
// encoding is CPU work only: no hardware operation has occurred. They allocate
// nothing and use no OS services; caller provides resident buffers at its IRQL
// and serializes access. GPU allocation lifetime extends through GPU completion,
// not merely encoding. Source/destination copy overlap is rejected (not memmove).
// Copy accepts byte alignment; fill/fence require 4-byte GPU address alignment.
// Transfers split at the pinned Linux 4 MiB policy limit; zero counts fail.
EncodeStatus encode_copy(const CopyRequest& request, CommandBuffer output,
                         EncodeResult& result) noexcept;
EncodeStatus encode_fill(const FillRequest& request, CommandBuffer output,
                         EncodeResult& result) noexcept;
EncodeStatus encode_fence(const FenceRequest& request, CommandBuffer output,
                          EncodeResult& result) noexcept;

// Linux SDMA 6.0 full graphics-cache rinse for VMID0, ordered before a completion
// fence. This constructs GCR_REQ only; CPU/GPU publication and HDP visibility
// still belong to the native memory/queue backend.
EncodeStatus encode_cache_sync(CommandBuffer output, EncodeResult& result) noexcept;

// Reuses the same transfer and fence encoders, validating the entire submission
// before any bytes change. Fence memory must be separate from transfer ranges.
// Produces unpadded packet bytes only; the eventual ring owner must initialize
// firmware/GPUVM/rings, arrange cache visibility, ring space, IB/ring alignment,
// interrupts and timeout recovery before committing them. No submit API exists.
EncodeStatus encode_copy_submission(const CopyRequest& request,
                                    const FenceRequest& fence,
                                    CommandBuffer output,
                                    EncodeResult& result) noexcept;
EncodeStatus encode_fill_submission(const FillRequest& request,
                                    const FenceRequest& fence,
                                    CommandBuffer output,
                                    EncodeResult& result) noexcept;

} // namespace sdma
} // namespace rx7900

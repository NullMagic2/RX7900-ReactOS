/*
Module: lightning.reactos.image
Purpose: Share checked exported-image layout between NT5 registration and native ownership.
Created: 2026-10-22
Architecture: Pure fixed-width C++ policy; native transport owns resources and mappings.
*/
#pragma once
#include "lightning_abi.hpp"

namespace lightning::reactos {
struct ImageLayout final {
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t stride = 0;
    uint32_t offset = 0;
    uint32_t format = 0;
};

// Decode the existing packed scanout ABI and prove every pixel lies inside the
// owned blob. A row may be padded. Failure leaves output unchanged; this proves
// byte layout only, never GPU completion or exportability of the allocation.
constexpr bool image_layout(const QLV_HEADER& request, uint64_t blob_bytes,
                            ImageLayout& output) noexcept {
    const ImageLayout candidate{
        static_cast<uint32_t>(request.bytes),
        static_cast<uint32_t>(request.bytes >> QLV_PACKED_WORD_SHIFT),
        static_cast<uint32_t>(request.offset),
        static_cast<uint32_t>(request.offset >> QLV_PACKED_WORD_SHIFT),
        request.flags};

    if (candidate.width == 0 || candidate.height == 0 ||
        candidate.width > QLV_SCANOUT_MAX_DIMENSION || candidate.height > QLV_SCANOUT_MAX_DIMENSION ||
        (candidate.format != QLV_SCANOUT_BGRA8 && candidate.format != QLV_SCANOUT_RGBA8) ||
        candidate.stride % QLV_SCANOUT_PIXEL_BYTES != 0 || candidate.offset % QLV_SCANOUT_PIXEL_BYTES != 0) {
        return false;
    }

    const uint64_t row_bytes = static_cast<uint64_t>(candidate.width) * QLV_SCANOUT_PIXEL_BYTES;
    const uint64_t preceding_rows = static_cast<uint64_t>(candidate.height - 1) * candidate.stride;

    if (candidate.stride < row_bytes || candidate.offset > blob_bytes ||
        preceding_rows > blob_bytes - candidate.offset ||
        row_bytes > blob_bytes - candidate.offset - preceding_rows) {
        return false;
    }

    output = candidate;
    return true;
}
} // namespace lightning::reactos

/*
Module: rx7900.common.byte_view
Purpose: Read untrusted little-endian bytes without unaligned struct overlays.
Created: 2026-10-21
Architecture: Allocation-free shared binary boundary.
*/
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace rx7900 {

class ByteView final {
public:
    // The caller owns data and must keep size readable bytes alive for this view.
    // A null pointer is always treated as an empty input, irrespective of size.
    constexpr ByteView(const uint8_t* data, size_t size) noexcept
        : data_(data), size_(data != nullptr ? size : 0) {}

    constexpr size_t size() const noexcept { return size_; }

    constexpr bool contains(size_t offset, size_t count) const noexcept {
        return offset <= size_ && count <= size_ - offset;
    }

    // Borrow a checked portion; the original owner must outlive both views.
    // Output is unchanged on failure. Null empty views require no pointer math.
    bool subview(size_t offset, size_t count, ByteView& output) const noexcept {
        if (!contains(offset, count)) {
            return false;
        }

        output = ByteView(data_ != nullptr ? data_ + offset : nullptr, count);
        return true;
    }

    bool read_u8(size_t offset, uint8_t& value) const noexcept {
        if (!contains(offset, 1)) {
            return false;
        }

        // The lifetime contract and contains check make this the only raw read.
        value = data_[offset];
        return true;
    }

    bool read_u16(size_t offset, uint16_t& value) const noexcept {
        if (!contains(offset, sizeof(value))) {
            return false;
        }

        uint8_t low = 0;
        uint8_t high = 0;
        read_u8(offset, low);
        read_u8(offset + 1, high);
        value = static_cast<uint16_t>(low | (static_cast<uint16_t>(high) << 8));
        return true;
    }

    bool read_u32(size_t offset, uint32_t& value) const noexcept {
        if (!contains(offset, sizeof(value))) {
            return false;
        }

        uint16_t low = 0;
        uint16_t high = 0;
        read_u16(offset, low);
        read_u16(offset + 2, high);
        value = static_cast<uint32_t>(low) | (static_cast<uint32_t>(high) << 16);
        return true;
    }

    bool read_u64(size_t offset, uint64_t& value) const noexcept {
        if (!contains(offset, sizeof(value))) {
            return false;
        }

        uint32_t low = 0;
        uint32_t high = 0;
        read_u32(offset, low);
        read_u32(offset + sizeof(low), high);
        value = static_cast<uint64_t>(low) | (static_cast<uint64_t>(high) << 32);
        return true;
    }

private:
    const uint8_t* data_;
    size_t size_;
};

} // namespace rx7900

/*
Module: rx7900.tests.discovery_fixture
Purpose: Reuse explicit synthetic discovery bytes across GC, PSP and parser tests.
Created: 2026-10-21
Architecture: Test-only checked byte construction, never device discovery evidence.
*/
#pragma once

#include <array>
#include "sdma.hpp"
#include "discovery.hpp"

namespace rx7900_test {

template<size_t Size>
void discovery_u16(std::array<uint8_t, Size>& bytes, size_t offset, uint16_t value) {
    bytes.at(offset) = static_cast<uint8_t>(value);
    bytes.at(offset + 1) = static_cast<uint8_t>(value >> 8);
}

template<size_t Size>
void discovery_checksums(std::array<uint8_t, Size>& bytes, bool update_table = true) {
    const rx7900::ByteView view(bytes.data(), bytes.size());
    uint16_t size = 0;
    uint16_t table = 0;
    uint16_t table_bytes = 0;
    view.read_u16(10, size);
    view.read_u16(12, table);
    view.read_u16(table + 6, table_bytes);
    uint16_t sum = 0;

    if (update_table) {
        for (size_t index = table; index < static_cast<size_t>(table) + table_bytes; ++index) {
            sum = static_cast<uint16_t>(sum + bytes.at(index));
        }
        discovery_u16(bytes, 14, sum);
    }
    sum = 0;

    for (size_t index = 10; index < size; ++index) {
        sum = static_cast<uint16_t>(sum + bytes.at(index));
    }

    discovery_u16(bytes, 8, sum);
}

inline std::array<uint8_t, 168> discovery_binary(bool wide = false,
    uint16_t hardware_id = rx7900::hardware::discovery_gc_hardware_id,
    uint8_t major = 11, uint32_t segment0 = 0x1000) {
    std::array<uint8_t, 168> bytes{};
    rx7900::sdma::CommandBuffer output(bytes.data(), bytes.size());
    const uint16_t size = wide ? 168 : 160;
    output.write_u32(0, 0x28211407);
    discovery_u16(bytes, 4, 1);
    discovery_u16(bytes, 10, size);
    discovery_u16(bytes, 12, 60);
    discovery_u16(bytes, 16, static_cast<uint16_t>(size - 60));
    output.write_u32(60, 0x53445049);
    discovery_u16(bytes, 64, wide ? 4 : 3);
    discovery_u16(bytes, 66, static_cast<uint16_t>(size - 60));
    discovery_u16(bytes, 72, 1);
    discovery_u16(bytes, 76, 140);
    bytes.at(138) = wide ? 1 : 0;
    discovery_u16(bytes, 142, 1);
    discovery_u16(bytes, 144, hardware_id);
    bytes.at(147) = 2;
    bytes.at(148) = major;
    output.write_u32(152, segment0);
    output.write_u32(wide ? 160 : 156, 0x2000);
    discovery_checksums(bytes);
    return bytes;
}

} // namespace rx7900_test

/*
Module: rx7900.tests.ucode_files
Purpose: Build synthetic GC/SMU/SDMA/DMCUB/TA firmware headers for catalog tests.
Created: 2026-10-21
Architecture: Test-only header layouts from Linux v6.12 amdgpu_ucode.h; payloads
are deterministic filler, never AMD firmware.
*/
#pragma once

#include <vector>

#include "sdma.hpp"
#include "byte_view.hpp"

namespace rx7900_test::ucode_files {

struct File final {
    std::vector<uint8_t> bytes;

    File(size_t size, uint16_t major, uint16_t minor, uint16_t ip_major, uint16_t ip_minor,
         uint32_t header_bytes) : bytes(size, 0) {
        u32(0, static_cast<uint32_t>(size));
        u32(4, header_bytes);
        u16(8, major);
        u16(10, minor);
        u16(12, ip_major);
        u16(14, ip_minor);
        u32(16, 0x11223344);
    }

    void u16(size_t offset, uint16_t value) {
        bytes.at(offset) = static_cast<uint8_t>(value);
        bytes.at(offset + 1) = static_cast<uint8_t>(value >> 8);
    }

    void u32(size_t offset, uint32_t value) {
        rx7900::sdma::CommandBuffer(bytes.data(), bytes.size()).write_u32(offset, value);
    }

    rx7900::ByteView view() const { return {bytes.data(), bytes.size()}; }
};

inline File gfx(uint32_t code, uint32_t data) {
    File file(0x400, 2, 0, 11, 0, 60);
    file.u32(24, 0x100);
    file.u32(20, code + data);
    file.u32(36, code);
    file.u32(40, 0x100);
    file.u32(44, data);
    file.u32(48, 0x100 + code);
    return file;
}

inline File rlc(uint16_t minor) {
    File file(0x800, 2, minor, 11, 0, 0xcc);
    file.u32(20, 0x80);
    file.u32(24, 0x100);
    file.u32(116, 0x40);  // restore list cntl
    file.u32(120, 0x200);
    file.u32(132, 0);     // gpm omitted
    file.u32(148, 0x20);  // srm
    file.u32(152, 0x280);
    file.u32(156, 0x40);  // iram
    file.u32(160, 0x300);
    file.u32(164, 0x40);  // dram
    file.u32(168, 0x340);

    if (minor == 3) {
        file.u32(180, 0x10);
        file.u32(184, 0x400);
        file.u32(196, 0x10);
        file.u32(200, 0x410);
    } else if (minor == 4) {
        file.u32(204, 0x10);
        file.u32(208, 0x500);
        file.u32(212, 0x10);
        file.u32(216, 0x510);
    }

    return file;
}

inline File mes() {
    File file(0x400, 1, 0, 11, 0, 72);
    file.u32(24, 0x100);
    file.u32(36, 0x80);
    file.u32(40, 0x100);
    file.u32(48, 0x40);
    file.u32(52, 0x180);
    return file;
}

inline File imu() {
    File file(0x400, 1, 0, 11, 0, 48);
    file.u32(24, 0x100);
    file.u32(32, 0x80);
    file.u32(36, 0x999);  // ignored by Linux; array offset plus IRAM size is used
    file.u32(40, 0x60);
    return file;
}

inline File smu() {
    File file(0x400, 2, 1, 13, 0, 44);
    file.u32(20, 0x200);
    file.u32(24, 0x100);
    return file;
}

inline File sdma() {
    File file(0x200, 2, 0, 6, 0, 64);
    file.u32(20, 0x40);
    file.u32(24, 0x100);
    file.u32(36, 0x20);
    file.u32(48, 0x120);
    file.u32(52, 0x20);
    return file;
}

inline File dmcub() {
    File file(0x200, 1, 0, 3, 2, 40);
    file.u32(20, 0x40);
    file.u32(24, 0x100);
    file.u32(32, 0x30);
    file.u32(36, 0x10);
    return file;
}

inline File ta() {
    File file(0x400, 2, 0, 13, 0, 36);
    file.u32(20, 0x200);
    file.u32(24, 0x100);
    file.u32(32, 2);
    file.u32(36, 4);       // DTM descriptor first
    file.u32(44, 0x80);
    file.u32(48, 0x40);
    file.u32(52, 1);       // ASD
    file.u32(60, 0x10);
    file.u32(64, 0x60);
    return file;
}

// psp_13_0_0_ta.bin layout: ASD, HDCP, DTM, RAS descriptors in that file order
// (types 1, 4, 5, 3) with the real file's versions, sizes scaled down and
// deterministic payload bytes (seed = type) inside the ucode array at 0x100.
struct TaLayout final {
    uint32_t type;
    uint32_t version;
    uint32_t relative;
    uint32_t bytes;
};

constexpr TaLayout navi31_ta_layout[]{
    {1, 0x21000109, 0x000, 0x100},
    {4, 0x1700004d, 0x100, 0x84},
    {5, 0x1200001d, 0x200, 0x40},
    {3, 0x1b000205, 0x240, 0x63},
};

inline File ta_navi31(size_t count = 4) {
    constexpr uint32_t array_offset = 0x100;
    constexpr uint32_t array_bytes = 0x300;
    File file(array_offset + array_bytes, 2, 0, 13, 0, 36);
    file.u32(20, array_bytes);
    file.u32(24, array_offset);
    file.u32(32, static_cast<uint32_t>(count));

    for (size_t index = 0; index < count && index < 4; ++index) {
        const TaLayout& layout = navi31_ta_layout[index];
        const size_t descriptor = 36 + index * 16;
        file.u32(descriptor, layout.type);
        file.u32(descriptor + 4, layout.version);
        file.u32(descriptor + 8, layout.relative);
        file.u32(descriptor + 12, layout.bytes);

        for (uint32_t byte = 0; byte < layout.bytes; ++byte) {
            file.bytes.at(array_offset + layout.relative + byte) =
                static_cast<uint8_t>(layout.type * 0x31 + byte);
        }
    }

    return file;
}

} // namespace rx7900_test::ucode_files

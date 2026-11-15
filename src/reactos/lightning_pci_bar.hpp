/*
Module: lightning.reactos.pci_bar
Purpose: Decode the two supported Lightning graphics BAR layouts without truncation.
Created: 2026-10-23
Architecture: Pure C++17 policy shared by VideoPort discovery and native tests.
*/
#pragma once
#include <stdint.h>

namespace lightning::reactos {

constexpr uint32_t low_mmio_graphics_bytes = 512U * 1024U * 1024U;

constexpr bool framebuffer_bar_address(uint32_t low, uint32_t high,
                                       uint64_t& address) noexcept {
    address = 0;
    if ((low & 0x9U) != 0x8U) { return false; }
    const uint32_t type = low & 0x6U;
    if (type == 0x4U) {
        address = (static_cast<uint64_t>(high) << 32) | (low & ~0xfU);
    } else if (type == 0 && high == 0) {
        address = low & ~0xfU;
    } else {
        return false;
    }
    return address != 0;
}

constexpr bool framebuffer_bar_length_valid(uint32_t low, uint32_t bytes) noexcept {
    if ((low & 0x9U) != 0x8U) { return false; }
    const uint32_t type = low & 0x6U;
    return (type == 0 && bytes == low_mmio_graphics_bytes) ||
           (type == 0x4U && bytes == 1024U * 1024U * 1024U);
}

} // namespace lightning::reactos

/*
Module: rx7900.hardware.pm4
Purpose: Share the GFX11 PM4 type-3 header and the opcodes used by ASIC bring-up.
Created: 2026-10-21
Architecture: Header-only constants for RLC clear state, KIQ/MES and CP packets.
Copyright 2019 Advanced Micro Devices, Inc. Adapted Linux v6.12 nvd.h (PACKET3,
opcodes). Notice and provenance: licenses/AMDGPU-FIRMWARE.txt.
*/
#pragma once

#include <stdint.h>

namespace rx7900::hardware::pm4 {

// nvd.h PACKET3(op, n): type 3, 8-bit opcode, 14-bit count of dwords minus one.
constexpr uint32_t packet3(uint32_t opcode, uint32_t count) noexcept {
    return (3u << 30) | ((opcode & 0xffu) << 8) | ((count & 0x3fffu) << 16);
}

constexpr uint32_t clear_state = 0x12;
constexpr uint32_t context_control = 0x28;
constexpr uint32_t write_data = 0x37;
constexpr uint32_t preamble_cntl = 0x4a;
constexpr uint32_t preamble_begin_clear_state = 2u << 28;
constexpr uint32_t preamble_end_clear_state = 3u << 28;
constexpr uint32_t set_context_reg = 0x69;
constexpr uint32_t set_context_reg_start = 0x0000a000;
constexpr uint32_t set_resources = 0xa0;
constexpr uint32_t map_queues = 0xa2;

} // namespace rx7900::hardware::pm4

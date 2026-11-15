/*
Module: rx7900.tests.atom_asic_init_rom
Purpose: Synthetic VBIOS images whose ATOM ASIC_Init command table re-POSTs the
simulated Navi31 (or fails to), for the destructive-reset restart tests.
Created: 2026-10-24
Architecture: Test-only. Lays out exactly what AtomInterpreter::bind and
asic_init read (0xAA55, ATI magic at 0x30, ATOM header pointer at 0x48, "ATOM",
master command/data tables, ASIC_Init format 2 content 1) with one MOVE_REG
immediate to BIOS_SCRATCH_7 and EOT. It is not a real VBIOS.
*/
#pragma once

#include <cstdint>
#include <cstring>
#include <vector>

namespace rx7900_test {

enum class AsicInitRom {
    // BIOS_SCRATCH_7 = ASIC_INIT_COMPLETE: the repost confirms the card posted.
    posting,
    // The table runs but leaves BIOS_SCRATCH_7 clear: repost fails (retryable).
    silent,
    // The MOVE instruction does not fit its table: the interpreter quarantines.
    malformed
};

inline std::vector<uint8_t> asic_init_rom(uint32_t bios_scratch_reg, AsicInitRom kind) {
    std::vector<uint8_t> rom(0x200, 0);
    const auto put16 = [&rom](size_t offset, uint16_t value) {
        rom.at(offset) = static_cast<uint8_t>(value);
        rom.at(offset + 1) = static_cast<uint8_t>(value >> 8);
    };
    const auto put32 = [&](size_t offset, uint32_t value) {
        put16(offset, static_cast<uint16_t>(value));
        put16(offset + 2, static_cast<uint16_t>(value >> 16));
    };
    constexpr size_t header = 0x60;
    constexpr size_t commands = 0x100;
    constexpr size_t asic_init = 0x140;
    constexpr size_t data = 0x180;
    const char magic[] = " 761295520";

    put16(0, 0xaa55);
    std::memcpy(rom.data() + 0x30, magic, sizeof(magic) - 1);
    put16(0x48, header);
    put16(header, 0x30);
    rom.at(header + 2) = 1;
    rom.at(header + 3) = 1;
    put32(header + 4, 0x4d4f5441u); // "ATOM"
    put16(header + 0x1e, commands);
    put16(header + 0x20, data);
    // Master command table: ASIC_Init is entry 0.
    put16(commands, 8);
    rom.at(commands + 2) = 1;
    rom.at(commands + 3) = 1;
    put16(commands + 4, asic_init);
    // ASIC_Init: header, WS 0, PS 8 bytes, MOVE_REG imm32 to BIOS_SCRATCH_7, EOT.
    const uint32_t value = kind == AsicInitRom::posting ? 0x00000200u : 0;
    const uint16_t bytes = kind == AsicInitRom::malformed ? 10 : 15;
    put16(asic_init, bytes);
    rom.at(asic_init + 2) = 2;
    rom.at(asic_init + 3) = 1;
    rom.at(asic_init + 4) = 0;
    rom.at(asic_init + 5) = 8;
    rom.at(asic_init + 6) = 1;    // MOVE, destination REG
    rom.at(asic_init + 7) = 0x05; // source IMM, dword alignment, full destination
    put16(asic_init + 8, static_cast<uint16_t>(bios_scratch_reg + 7));
    put32(asic_init + 10, value);
    rom.at(asic_init + 14) = 91;  // EOT
    // Master data table without entries.
    put16(data, 8);
    rom.at(data + 2) = 1;
    rom.at(data + 3) = 1;
    return rom;
}

} // namespace rx7900_test

/*
Module: rx7900.common.atom_bios
Purpose: Validate the ROM and ATOM header signatures of caller-owned BIOS bytes.
Created: 2026-10-21
Architecture: Linux-derived validation without ROM acquisition or execution.
*/
#pragma once

#include "byte_view.hpp"
#include "status.hpp"

namespace rx7900 {

struct AtomHeader final {
    uint16_t offset;
    bool reversed_signature;
};

constexpr uint32_t maximum_atom_file_bytes = 1024u * 1024u;

struct AtomImage final {
    AtomHeader header{};
    uint32_t image_bytes = 0;
};

// Success recognizes ATOM/MOTA signatures only, not firmware trust or usability.
// Output is cleared on failure; input ownership/lifetime remains with the caller.
ParseStatus validate_atom_bios(ByteView bios, AtomHeader& header) noexcept;

// Check the first PCI option-ROM image against the owning function's identity,
// then validate ATOM strictly within its declared PCIR image extent. Trailing
// option-ROM images are not interpreted. Output is unchanged on failure.
// This validates structure/identity, not authenticity or a ROM checksum.
ParseStatus validate_atom_image(ByteView rom, uint16_t vendor, uint16_t device,
                                AtomImage& image) noexcept;

// One ATOM common table header (u16 size, u8 format, u8 content revision, low six
// revision bits as in Linux amdgpu_atom_parse_data_header) wholly inside the ROM.
struct AtomTable final {
    size_t base = 0;
    size_t bytes = 0;
    uint8_t format = 0;
    uint8_t revision = 0;
};

enum class AtomTableStatus { success, absent, truncated_input, invalid_table, unsupported_revision };

AtomTableStatus atom_table_at(ByteView rom, size_t offset, AtomTable& output) noexcept;

// Validated ATOM signature, ROM header 2.2 and master data table 2.1, then the
// u16 entry at index of atom_master_list_of_data_tables_v2_1. A zero entry is
// absent. Output unchanged on failure; no ATOM command table is ever executed.
AtomTableStatus atom_data_table(ByteView rom, size_t index, AtomTable& output) noexcept;

} // namespace rx7900

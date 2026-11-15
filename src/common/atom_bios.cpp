/*
 * Copyright 2008 Advanced Micro Devices, Inc.
 * Copyright 2008 Red Hat Inc.
 * Copyright 2009 Jerome Glisse.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Authors: Dave Airlie, Alex Deucher, Jerome Glisse.
 */
/*
Module: rx7900.common.atom_bios
Purpose: Adapt Linux v6.12 amdgpu_bios.c check_atom_bios to checked C++ reads.
Created: 2026-10-21 (adaptation; upstream dates above are preserved)
Architecture: Reuses the signature algorithm; fixes length/offset boundary checks.
*/
#include "atom_bios.hpp"

namespace rx7900 {
namespace {

constexpr size_t atom_header_pointer_offset = 0x48;
constexpr size_t atom_signature_relative_offset = 4;
constexpr uint16_t pci_rom_signature = 0xaa55;
constexpr uint32_t atom_signature = 0x4d4f5441;
constexpr uint32_t mota_signature = 0x41544f4d;
constexpr size_t pci_data_pointer_offset = 0x18;
constexpr uint32_t pci_data_signature = 0x52494350;
constexpr size_t pci_data_vendor = 4;
constexpr size_t pci_data_device = 6;
constexpr size_t pci_data_length = 0x0a;
constexpr size_t pci_data_class = 0x0f;
constexpr size_t pci_data_image_units = 0x10;
constexpr size_t pci_data_code_type = 0x14;
constexpr size_t minimum_pci_data_bytes = 0x18;
constexpr uint32_t option_rom_unit_bytes = 512;
constexpr uint8_t display_class = 3;
constexpr uint8_t legacy_code_type = 0;
constexpr uint8_t efi_code_type = 3;

} // namespace

ParseStatus validate_atom_bios(ByteView bios, AtomHeader& header) noexcept {
    header = {};
    uint16_t signature = 0;
    uint16_t header_offset = 0;

    if (!bios.read_u16(0, signature) ||
        !bios.read_u16(atom_header_pointer_offset, header_offset)) {
        return ParseStatus::truncated_input;
    }
    if (signature != pci_rom_signature) {
        return ParseStatus::invalid_rom_signature;
    }
    if (header_offset == 0) {
        return ParseStatus::invalid_atom_header;
    }

    // Widen before adding; the 16-bit firmware offset must not wrap at 0xffff.
    const size_t signature_offset = static_cast<size_t>(header_offset) +
                                    atom_signature_relative_offset;
    uint32_t atom = 0;

    if (!bios.read_u32(signature_offset, atom)) {
        return ParseStatus::truncated_input;
    }
    if (atom != atom_signature && atom != mota_signature) {
        return ParseStatus::invalid_atom_header;
    }

    header.offset = header_offset;
    header.reversed_signature = atom == mota_signature;
    return ParseStatus::success;
}

ParseStatus validate_atom_image(ByteView rom, uint16_t vendor, uint16_t device,
                                AtomImage& image) noexcept {
    if (rom.size() > maximum_atom_file_bytes) {
        return ParseStatus::malformed_header;
    }

    uint16_t signature = 0;
    uint16_t pci_offset = 0;

    if (!rom.read_u16(0, signature) || !rom.read_u16(pci_data_pointer_offset, pci_offset)) {
        return ParseStatus::truncated_input;
    }
    if (signature != pci_rom_signature) {
        return ParseStatus::invalid_rom_signature;
    }
    if (pci_offset < pci_data_pointer_offset + sizeof(pci_offset)) {
        return ParseStatus::malformed_header;
    }

    uint32_t pci_signature = 0;
    uint16_t found_vendor = 0, found_device = 0, header_bytes = 0, image_units = 0;
    uint8_t base_class = 0, code_type = 0;
    const size_t base = pci_offset;

    if (!rom.read_u32(base, pci_signature) ||
        !rom.read_u16(base + pci_data_vendor, found_vendor) ||
        !rom.read_u16(base + pci_data_device, found_device) ||
        !rom.read_u16(base + pci_data_length, header_bytes) ||
        !rom.read_u8(base + pci_data_class, base_class) ||
        !rom.read_u16(base + pci_data_image_units, image_units) ||
        !rom.read_u8(base + pci_data_code_type, code_type)) {
        return ParseStatus::truncated_input;
    }
    if (pci_signature != pci_data_signature || header_bytes < minimum_pci_data_bytes ||
        image_units == 0 || (code_type != legacy_code_type && code_type != efi_code_type)) {
        return ParseStatus::malformed_header;
    }
    if (found_vendor != vendor || found_device != device || base_class != display_class) {
        return ParseStatus::unsupported_device;
    }

    const uint32_t length = static_cast<uint32_t>(image_units) * option_rom_unit_bytes;
    ByteView first(nullptr, 0);

    if (!rom.subview(0, length, first)) {
        return ParseStatus::truncated_input;
    }
    if (!first.contains(base, header_bytes)) {
        return ParseStatus::malformed_header;
    }

    AtomImage candidate{};
    const ParseStatus status = validate_atom_bios(first, candidate.header);

    if (status != ParseStatus::success) {
        return status;
    }

    candidate.image_bytes = length;
    image = candidate;
    return ParseStatus::success;
}

AtomTableStatus atom_table_at(ByteView rom, size_t offset, AtomTable& output) noexcept {
    constexpr size_t common_header_bytes = 4;
    uint16_t bytes = 0;
    uint8_t format = 0;
    uint8_t revision = 0;

    if (!rom.contains(offset, common_header_bytes) || !rom.read_u16(offset, bytes) ||
        !rom.read_u8(offset + 2, format) || !rom.read_u8(offset + 3, revision)) {
        return AtomTableStatus::truncated_input;
    }

    if (bytes < common_header_bytes) {
        return AtomTableStatus::invalid_table;
    }

    if (!rom.contains(offset, bytes)) {
        return AtomTableStatus::truncated_input;
    }

    output = AtomTable{offset, bytes, static_cast<uint8_t>(format & 0x3fu),
                       static_cast<uint8_t>(revision & 0x3fu)};
    return AtomTableStatus::success;
}

AtomTableStatus atom_data_table(ByteView rom, size_t index, AtomTable& output) noexcept {
    constexpr size_t rom_header_minimum_bytes = 40;
    constexpr size_t master_data_pointer = 32;
    constexpr size_t common_header_bytes = 4;
    AtomHeader atom{};
    const ParseStatus validated = validate_atom_bios(rom, atom);

    if (validated != ParseStatus::success) {
        return validated == ParseStatus::truncated_input ? AtomTableStatus::truncated_input :
                                                           AtomTableStatus::invalid_table;
    }

    AtomTable header{};
    AtomTableStatus status = atom_table_at(rom, atom.offset, header);

    if (status != AtomTableStatus::success) {
        return status;
    }

    if (header.format != 2 || header.revision != 2) {
        return AtomTableStatus::unsupported_revision;
    }

    if (header.bytes < rom_header_minimum_bytes) {
        return AtomTableStatus::invalid_table;
    }

    uint16_t master_offset = 0;
    rom.read_u16(header.base + master_data_pointer, master_offset);
    AtomTable master{};
    status = atom_table_at(rom, master_offset, master);

    if (master_offset == 0 || status != AtomTableStatus::success) {
        return master_offset == 0 ? AtomTableStatus::invalid_table : status;
    }

    if (master.format != 2 || master.revision != 1) {
        return AtomTableStatus::unsupported_revision;
    }

    const size_t entry = common_header_bytes + index * 2;
    uint16_t table_offset = 0;

    if (index > master.bytes || master.bytes < entry + 2 ||
        !rom.read_u16(master.base + entry, table_offset)) {
        return AtomTableStatus::invalid_table;
    }

    if (table_offset == 0) {
        return AtomTableStatus::absent;
    }

    AtomTable table{};
    status = atom_table_at(rom, table_offset, table);

    if (status == AtomTableStatus::success) {
        output = table;
    }

    return status;
}

} // namespace rx7900

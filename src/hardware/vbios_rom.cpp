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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Authors: Dave Airlie, Alex Deucher, Jerome Glisse.
 */
/*
Module: rx7900.hardware.vbios_rom
Purpose: Adapt Linux v6.12 amdgpu_bios.c VFCT, SMUIO ROM and validity rules to
checked C++ reads.
Created: 2026-10-22 (adaptation; upstream dates above are preserved)
Architecture: The expansion-ROM image walk is written from the PCI Firmware
Specification layout to match the observable result of Linux drivers/pci/rom.c
pci_get_rom_size; no GPL code is copied. Register offsets come from the pinned
smuio_13_0_6_offset.h and nbio_4_3_0_offset.h/sh_mask.h.
*/
#include "vbios_rom.hpp"

namespace rx7900::hardware {
namespace {

constexpr uint16_t rom_signature = 0xaa55;
constexpr uint32_t pci_data_signature = 0x52494350;
constexpr uint32_t pci_data_pointer_offset = 24;
constexpr uint32_t pci_data_length_offset = 16;
constexpr uint32_t pci_data_indicator_offset = 21;
constexpr uint8_t pci_data_last_image = 0x80;
constexpr char amd_vbios_signature[vbios_signature_bytes + 1] = " 761295520";

// atom_rom_header_v2_2 fields after the common header and "ATOM" string.
constexpr size_t rom_header_subsystem_vendor = 24;
constexpr size_t rom_header_subsystem_device = 26;
constexpr size_t rom_header_subsystem_bytes = 28;

// UEFI_ACPI_VFCT (atombios.h, ULONG is 32 bits) and VFCT_IMAGE_HEADER.
constexpr size_t acpi_length_offset = 4;
constexpr size_t vfct_header_bytes = 0x4c;
constexpr size_t vfct_image_offset = 0x34;
constexpr size_t vfct_image_header_bytes = 0x1c;
constexpr size_t vfct_bus = 0x00;
constexpr size_t vfct_device = 0x04;
constexpr size_t vfct_function = 0x08;
constexpr size_t vfct_vendor = 0x0c;
constexpr size_t vfct_device_id = 0x0e;
constexpr size_t vfct_image_length = 0x18;

// smuio_13_0_6_offset.h (BASE_IDX 0) and nbio_4_3_0 REGS_ROM_OFFSET_CTRL (BASE_IDX 5).
constexpr uint32_t smuio_rom_index = 0x00e4;
constexpr uint32_t smuio_rom_data = 0x00e5;
constexpr size_t nbio_rom_offset_segment = 5;
constexpr uint32_t nbio_regs_rom_offset_ctrl = 0xcc23;
constexpr uint32_t nbio_rom_offset_mask = 0x7f;
constexpr uint32_t rom_offset_shift = 17;

bool byte_view_read32(const void* context, uint32_t offset, uint32_t& value) noexcept {
    return context != nullptr && static_cast<const ByteView*>(context)->read_u32(offset, value);
}

// Unaligned u8/u16 reads composed from the aligned dwords the window provides.
bool window_u8(const RomWindow& window, uint32_t offset, uint8_t& value) noexcept {
    uint32_t dword = 0;

    if (offset >= window.bytes || !window.read32(window.context, offset & ~3u, dword)) {
        return false;
    }

    value = static_cast<uint8_t>(dword >> ((offset & 3u) * 8));
    return true;
}

bool window_u16(const RomWindow& window, uint32_t offset, uint16_t& value) noexcept {
    uint8_t low = 0;
    uint8_t high = 0;

    if (offset > UINT32_MAX - 1 || !window_u8(window, offset, low) ||
        !window_u8(window, offset + 1, high)) {
        return false;
    }

    value = static_cast<uint16_t>(low | (static_cast<uint16_t>(high) << 8));
    return true;
}

bool window_u32(const RomWindow& window, uint32_t offset, uint32_t& value) noexcept {
    uint16_t low = 0;
    uint16_t high = 0;

    if (offset > UINT32_MAX - 3 || !window_u16(window, offset, low) ||
        !window_u16(window, offset + 2, high)) {
        return false;
    }

    value = static_cast<uint32_t>(low) | (static_cast<uint32_t>(high) << 16);
    return true;
}

bool in_window(const RomWindow& window, uint64_t offset, uint64_t count) noexcept {
    return offset <= window.bytes && count <= window.bytes - offset;
}

// amdgpu_soc15_read_bios_from_rom: one ROM_INDEX write, then auto-incrementing
// ROM_DATA reads. length is a dword multiple that fits output.
VbiosStatus soc15_read(const RegisterIo& io, const SmuioRomRegisters& smuio, Nbio& nbio,
                       sdma::CommandBuffer output, uint32_t length, uint32_t& rom_offset) noexcept {
    uint32_t control = 0;

    // nbio_v4_3_get_rom_offset, read on every call exactly as Linux does.
    if (!nbio.read_register(smuio.rom_offset_base(), nbio_regs_rom_offset_ctrl, control)) {
        return VbiosStatus::mmio_failure;
    }

    rom_offset = (control & nbio_rom_offset_mask) << rom_offset_shift;

    if (!write_ip_register(io, smuio.base(), smuio_rom_index, rom_offset)) {
        return VbiosStatus::mmio_failure;
    }

    for (uint32_t offset = 0; offset < length; offset += sizeof(uint32_t)) {
        uint32_t value = 0;

        if (!read_ip_register(io, smuio.base(), smuio_rom_data, value) ||
            !output.write_u32(offset, value)) {
            return VbiosStatus::mmio_failure;
        }
    }

    return VbiosStatus::success;
}

} // namespace

VbiosStatus check_atom_bios(ByteView candidate) noexcept {
    if (candidate.size() < minimum_vbios_bytes) {
        return VbiosStatus::invalid_length;
    }

    AtomHeader header{};
    const ParseStatus status = validate_atom_bios(candidate, header);

    if (status == ParseStatus::invalid_rom_signature) {
        return VbiosStatus::no_signature;
    }

    return status == ParseStatus::success ? VbiosStatus::success : VbiosStatus::invalid_atom;
}

VbiosStatus validate_vbios_image(ByteView candidate, uint16_t vendor, uint16_t device,
                                 AtomImage& image) noexcept {
    if (candidate.size() > maximum_atom_file_bytes) {
        return VbiosStatus::invalid_length;
    }

    const VbiosStatus linux_check = check_atom_bios(candidate);

    if (linux_check != VbiosStatus::success) {
        return linux_check;
    }

    AtomImage found{};
    const ParseStatus status = validate_atom_image(candidate, vendor, device, found);

    if (status == ParseStatus::unsupported_device) {
        return VbiosStatus::identity_mismatch;
    }

    if (status == ParseStatus::truncated_input) {
        return VbiosStatus::invalid_length;
    }

    if (status != ParseStatus::success) {
        return VbiosStatus::invalid_atom;
    }

    image = found;
    return VbiosStatus::success;
}

VbiosStatus check_atom_subsystem(ByteView image, uint16_t subsystem_vendor,
                                 uint16_t subsystem_device) noexcept {
    AtomHeader header{};

    if (validate_atom_bios(image, header) != ParseStatus::success) {
        return VbiosStatus::invalid_atom;
    }

    AtomTable rom_header{};
    uint16_t vendor = 0;
    uint16_t device = 0;

    if (atom_table_at(image, header.offset, rom_header) != AtomTableStatus::success ||
        rom_header.bytes < rom_header_subsystem_bytes ||
        !image.read_u16(rom_header.base + rom_header_subsystem_vendor, vendor) ||
        !image.read_u16(rom_header.base + rom_header_subsystem_device, device)) {
        return VbiosStatus::invalid_atom;
    }

    return vendor == subsystem_vendor && device == subsystem_device ?
        VbiosStatus::success : VbiosStatus::identity_mismatch;
}

RomWindow byte_view_rom_window(const ByteView& view) noexcept {
    const size_t bytes = view.size() > UINT32_MAX ? UINT32_MAX : view.size();
    return RomWindow{&view, static_cast<uint32_t>(bytes), byte_view_read32};
}

bool pci_rom_image_bytes(const RomWindow& window, uint32_t& bytes) noexcept {
    bytes = 0;

    if (window.read32 == nullptr) {
        return false;
    }

    uint64_t image = 0;
    bool last = false;
    uint16_t units = 0;

    do {
        uint16_t signature = 0;
        uint16_t pci_pointer = 0;
        uint32_t pci_signature = 0;
        uint8_t indicator = 0;

        if (!in_window(window, image, pci_data_pointer_offset + sizeof(pci_pointer))) {
            break;
        }

        const uint32_t base = static_cast<uint32_t>(image);

        if (!window_u16(window, base, signature)) {
            return false;
        }

        if (signature != rom_signature) {
            break;
        }

        if (!window_u16(window, base + pci_data_pointer_offset, pci_pointer)) {
            return false;
        }

        const uint64_t pds = image + pci_pointer;

        if (!in_window(window, pds, pci_data_indicator_offset + 1)) {
            break;
        }

        const uint32_t pci_base = static_cast<uint32_t>(pds);

        if (!window_u32(window, pci_base, pci_signature)) {
            return false;
        }

        if (pci_signature != pci_data_signature) {
            break;
        }

        if (!window_u8(window, pci_base + pci_data_indicator_offset, indicator) ||
            !window_u16(window, pci_base + pci_data_length_offset, units)) {
            return false;
        }

        last = (indicator & pci_data_last_image) != 0;
        image += static_cast<uint64_t>(units) * vbios_length_unit_bytes;

        // Linux stops before iterating outside the resource window.
        if (image >= window.bytes) {
            break;
        }

        if (!last) {
            uint16_t next = 0;

            if (!window_u16(window, static_cast<uint32_t>(image), next)) {
                return false;
            }

            if (next != rom_signature) {
                break;
            }
        }
    } while (units != 0 && !last);

    // Never larger than the window (ROMs with wrong sizes exist, as Linux notes).
    bytes = static_cast<uint32_t>(image < window.bytes ? image : window.bytes);
    return true;
}

VbiosStatus find_vfct_image(ByteView table, const VfctTarget& target, ByteView& image) noexcept {
    uint32_t length = 0;
    uint32_t first = 0;

    if (!table.read_u32(acpi_length_offset, length)) {
        return VbiosStatus::invalid_length;
    }

    // Linux "too short #1"; the declared length must also be really present.
    if (length < vfct_header_bytes || length > table.size() ||
        !table.read_u32(vfct_image_offset, first)) {
        return VbiosStatus::invalid_length;
    }

    uint64_t offset = first;

    while (offset < length) {
        const uint64_t header = offset;
        uint32_t bus = 0, device = 0, function = 0, image_bytes = 0;
        uint16_t vendor = 0, device_id = 0;

        offset += vfct_image_header_bytes;

        // Linux "image header truncated".
        if (offset > length) {
            return VbiosStatus::invalid_length;
        }

        const size_t at = static_cast<size_t>(header);
        table.read_u32(at + vfct_bus, bus);
        table.read_u32(at + vfct_device, device);
        table.read_u32(at + vfct_function, function);
        table.read_u16(at + vfct_vendor, vendor);
        table.read_u16(at + vfct_device_id, device_id);
        table.read_u32(at + vfct_image_length, image_bytes);
        offset += image_bytes;

        // Linux "image truncated".
        if (offset > length) {
            return VbiosStatus::invalid_length;
        }

        if (image_bytes != 0 && bus == target.bus && device == target.device &&
            function == target.function && vendor == target.vendor_id &&
            device_id == target.device_id) {
            ByteView content(nullptr, 0);

            if (!table.subview(at + vfct_image_header_bytes, image_bytes, content)) {
                return VbiosStatus::invalid_length;
            }

            const VbiosStatus status = check_atom_bios(content);

            if (status == VbiosStatus::success) {
                image = content;
            }

            return status;
        }
    }

    // Linux "too short #2": no matching image.
    return VbiosStatus::absent;
}

DiscoveryStatus validate_smuio_rom_discovery(ByteView binary, const NbioRegisters& nbio,
                                             SmuioRomRegisters& output) noexcept {
    output = {};

    if (!nbio.valid()) {
        return DiscoveryStatus::invalid_argument;
    }

    const uint32_t aperture = nbio.aperture_bytes();

    DiscoveredIp smuio;
    const DiscoveryStatus status =
        discover_ip(binary, {discovery_smuio_hardware_id, 0, 13, 0, 6, 1}, smuio);

    if (status != DiscoveryStatus::validated) {
        return status;
    }

    uint32_t index = 0;
    uint32_t data = 0;

    if (!discovery_register_offset(smuio.segment(0), smuio_rom_index, aperture, index) ||
        !discovery_register_offset(smuio.segment(0), smuio_rom_data, aperture, data)) {
        return DiscoveryStatus::invalid_argument;
    }

    output.smuio_ = smuio;
    output.rom_offset_base_ = nbio.nbif_base(nbio_rom_offset_segment);
    output.aperture_bytes_ = aperture;
    return DiscoveryStatus::validated;
}

SmuioRomReport read_smuio_rom(const RegisterIo& io, const SmuioRomRegisters& smuio, Nbio& nbio,
                              sdma::CommandBuffer output, uint32_t& bytes) noexcept {
    bytes = 0;
    SmuioRomReport report{};

    if (!smuio.valid() || !register_io_valid(io) || io.aperture_bytes != smuio.aperture_bytes()) {
        return report;
    }

    uint8_t probe[vbios_probe_bytes]{};
    report.status = soc15_read(io, smuio, nbio, sdma::CommandBuffer(probe, sizeof(probe)),
                               sizeof(probe), report.rom_offset);

    if (report.status != VbiosStatus::success) {
        return report;
    }

    const ByteView header(probe, sizeof(probe));
    uint16_t signature = 0;
    uint8_t units = 0;
    bool matched = header.read_u16(0, signature) && signature == rom_signature &&
                   header.read_u8(2, units);

    for (uint32_t index = 0; matched && index < vbios_signature_bytes; ++index) {
        uint8_t value = 0;
        matched = header.read_u8(vbios_signature_offset + index, value) &&
                  value == static_cast<uint8_t>(amd_vbios_signature[index]);
    }

    if (!matched) {
        report.status = VbiosStatus::no_signature;
        return report;
    }

    // AMD_VBIOS_LENGTH is a multiple of 512, hence already dword aligned.
    report.declared_bytes = static_cast<uint32_t>(units) * vbios_length_unit_bytes;

    if (report.declared_bytes == 0) {
        report.status = VbiosStatus::invalid_length;
        return report;
    }

    if (report.declared_bytes > output.size()) {
        report.status = VbiosStatus::insufficient_storage;
        return report;
    }

    report.status = soc15_read(io, smuio, nbio, output, report.declared_bytes, report.rom_offset);

    if (report.status == VbiosStatus::success) {
        bytes = report.declared_bytes;
    }

    return report;
}

} // namespace rx7900::hardware

/*
Module: rx7900.tests.common
Purpose: Verify parser boundaries and actual 64-bit Navi 31 resource decoding.
Created: 2026-10-21
Architecture: Exercise the same common implementation linked by the driver.
*/
#include <array>
#include <cstdio>
#include <limits>
#include <vector>

#include "atom_bios.hpp"
#include "pci.hpp"

namespace {

unsigned checks = 0;
unsigned failures = 0;

void expect(bool condition, const char* contract) {
    ++checks;

    if (!condition) {
        ++failures;
        std::fprintf(stderr, "FAIL: %s\n", contract);
    }
}

template<size_t capacity>
void write_u16(std::array<uint8_t, capacity>& bytes, size_t offset, uint16_t value) {
    bytes.at(offset) = static_cast<uint8_t>(value & 0xff);
    bytes.at(offset + 1) = static_cast<uint8_t>(value >> 8);
}

template<size_t capacity>
void write_u32(std::array<uint8_t, capacity>& bytes, size_t offset, uint32_t value) {
    write_u16(bytes, offset, static_cast<uint16_t>(value & 0xffff));
    write_u16(bytes, offset + 2, static_cast<uint16_t>(value >> 16));
}

auto navi31_header() {
    std::array<uint8_t, rx7900::pci_header_bytes> bytes{};
    write_u16(bytes, rx7900::pci_vendor_offset, rx7900::amd_vendor_id);
    write_u16(bytes, rx7900::pci_device_offset, rx7900::navi31_device_id);
    bytes.at(rx7900::pci_class_offset) = 3;
    bytes.at(rx7900::pci_revision_offset) = 0xc8;
    write_u16(bytes, rx7900::pci_subsystem_vendor_offset, 0x1da2);
    write_u16(bytes, rx7900::pci_subsystem_device_offset, 0x471e);
    // Real host BAR address observed read-only on 2026-10-21: 0x2800000000.
    write_u32(bytes, rx7900::pci_bars_offset, 0x0000000c);
    write_u32(bytes, rx7900::pci_bars_offset + 4, 0x00000028);
    write_u32(bytes, rx7900::pci_bars_offset + 8, 0x0000000c);
    write_u32(bytes, rx7900::pci_bars_offset + 12, 0x00000030);
    write_u32(bytes, rx7900::pci_bars_offset + 16, 0x0000f001);
    write_u32(bytes, rx7900::pci_bars_offset + 20, 0xdfa00000);
    return bytes;
}

void check_byte_view() {
    const std::array<uint8_t, 4> bytes{0x12, 0x34, 0x56, 0x78};
    const rx7900::ByteView view(bytes.data(), bytes.size());
    uint32_t value = 0;

    expect(view.read_u32(0, value) && value == 0x78563412, "little-endian decode");
    expect(!view.read_u32(1, value), "reject truncated u32");
    expect(!view.read_u32(std::numeric_limits<size_t>::max(), value),
           "reject overflowing offset without pointer arithmetic");
    expect(!rx7900::ByteView(nullptr, bytes.size()).read_u32(0, value),
           "null view cannot be read");
    rx7900::ByteView child(nullptr, 0);
    uint16_t short_value = 0;
    expect(view.subview(1, 2, child) && child.read_u16(0, short_value) &&
           short_value == 0x5634, "checked subview retains little-endian reads");
    expect(!view.subview(3, 2, child) && child.size() == 2,
           "invalid subview preserves output");
    expect(!view.subview(std::numeric_limits<size_t>::max(), 1, child),
           "subview rejects overflowing offset");
    expect(view.subview(bytes.size(), 0, child) && child.size() == 0,
           "empty trailing subview is valid");
    expect(rx7900::ByteView(nullptr, 0).subview(0, 0, child),
           "empty null subview requires no pointer arithmetic");
}

void check_pci() {
    auto bytes = navi31_header();
    rx7900::Adapter adapter{};
    const rx7900::PciLocation location{0, 3, 0, 0};
    const auto decode = [&] {
        return rx7900::decode_adapter(rx7900::ByteView(bytes.data(), bytes.size()),
                                      location, adapter);
    };

    expect(decode() == rx7900::ParseStatus::success, "recognize host Navi 31 identity");
    expect(adapter.bars[0].bus_address == UINT64_C(0x2800000000),
           "preserve above-4GiB BAR address");
    expect(adapter.bars[1].kind == static_cast<uint32_t>(rx7900::BarKind::continuation),
           "do not decode upper BAR dword as another resource");
    expect(adapter.bars[2].bus_address == UINT64_C(0x3000000000),
           "decode second 64-bit aperture");
    expect(adapter.bars[4].bus_address == 0xf000 &&
           adapter.bars[4].kind == static_cast<uint32_t>(rx7900::BarKind::io),
           "mask IO BAR address correctly");
    expect(adapter.bars[5].bus_address == 0xdfa00000,
           "decode MMIO address without mapping it");
    expect(adapter.subsystem_vendor_id == 0x1da2 && adapter.subsystem_device_id == 0x471e,
           "retain board identity separately from shared device ID");

    for (size_t length = 0; length < bytes.size(); ++length) {
        expect(rx7900::decode_adapter(rx7900::ByteView(bytes.data(), length),
            location, adapter) == rx7900::ParseStatus::truncated_input,
            "every short PCI header fails");
        expect(adapter.vendor_id == 0 && adapter.bars[0].bus_address == 0,
               "failed decode clears output");
    }

    bytes.at(rx7900::pci_class_offset) = 4;
    expect(decode() == rx7900::ParseStatus::unsupported_device,
           "do not identify an audio-class function as a display adapter");
    bytes = navi31_header();
    write_u16(bytes, rx7900::pci_device_offset, 0x15bf);
    expect(decode() == rx7900::ParseStatus::unsupported_device,
           "leave other AMD GPUs unsupported");

    for (const uint16_t device : {uint16_t{0x7448}, uint16_t{0x744a}, uint16_t{0x744b}, uint16_t{0x745e}}) {
        bytes = navi31_header();
        write_u16(bytes, rx7900::pci_device_offset, device);
        expect(decode() == rx7900::ParseStatus::success && adapter.device_id == device,
               "every Navi31 GPU function is identified");
    }

    for (const uint16_t device : {uint16_t{0x7446}, uint16_t{0xab30}, uint16_t{0x744d}}) {
        bytes = navi31_header();
        write_u16(bytes, rx7900::pci_device_offset, device);
        expect(decode() == rx7900::ParseStatus::unsupported_device,
               "Navi31 USB/audio functions and unknown IDs are not GPUs");
    }
    bytes = navi31_header();
    write_u32(bytes, rx7900::pci_bars_offset + 20, 0x00000004);
    expect(decode() == rx7900::ParseStatus::malformed_header,
           "reject a 64-bit BAR without a high dword");
    bytes = navi31_header();
    write_u32(bytes, rx7900::pci_bars_offset, UINT32_MAX);
    expect(decode() == rx7900::ParseStatus::malformed_header,
           "reject invalid all-ones BAR read");
    bytes = navi31_header();
    bytes.at(rx7900::pci_header_type_offset) = 1;
    expect(decode() == rx7900::ParseStatus::malformed_header,
           "reject bridge header layout");
}

void check_atom() {
    constexpr size_t header_pointer = 0x48;
    constexpr size_t header_offset = 0x80;
    constexpr size_t signature_offset = header_offset + 4;
    constexpr size_t signature_bytes = 4;
    std::array<uint8_t, signature_offset + signature_bytes> bios{};
    write_u16(bios, 0, 0xaa55);
    write_u16(bios, header_pointer, header_offset);
    write_u32(bios, signature_offset, 0x4d4f5441);
    rx7900::AtomHeader header{};

    expect(rx7900::validate_atom_bios(rx7900::ByteView(bios.data(), bios.size()),
        header) == rx7900::ParseStatus::success && header.offset == header_offset,
        "accept exact-sized ATOM signature input");

    for (size_t length = 0; length < bios.size(); ++length) {
        expect(rx7900::validate_atom_bios(rx7900::ByteView(bios.data(), length),
            header) != rx7900::ParseStatus::success, "reject every truncated BIOS");
    }

    write_u32(bios, signature_offset, 0x41544f4d);
    expect(rx7900::validate_atom_bios(rx7900::ByteView(bios.data(), bios.size()),
        header) == rx7900::ParseStatus::success && header.reversed_signature,
        "retain upstream MOTA recognition");
    write_u16(bios, header_pointer, UINT16_MAX);
    expect(rx7900::validate_atom_bios(rx7900::ByteView(bios.data(), bios.size()),
        header) == rx7900::ParseStatus::truncated_input,
        "do not wrap a 16-bit firmware header offset");
    expect(header.offset == 0, "failed BIOS validation clears output");
    write_u16(bios, header_pointer, 0);
    expect(rx7900::validate_atom_bios(rx7900::ByteView(bios.data(), bios.size()),
        header) == rx7900::ParseStatus::invalid_atom_header, "reject null ATOM offset");
    bios.at(0) = 0;
    expect(rx7900::validate_atom_bios(rx7900::ByteView(bios.data(), bios.size()),
        header) == rx7900::ParseStatus::invalid_rom_signature, "reject invalid ROM signature");
}

void check_atom_image() {
    constexpr size_t rom_bytes = 1024;
    constexpr size_t pcir = 0x80;
    constexpr size_t atom = 0xc0;
    std::array<uint8_t, rom_bytes> bytes{};
    write_u16(bytes, 0, 0xaa55);
    write_u16(bytes, 0x18, pcir);
    write_u16(bytes, 0x48, atom);
    write_u32(bytes, pcir, 0x52494350);
    write_u16(bytes, pcir + 4, rx7900::amd_vendor_id);
    write_u16(bytes, pcir + 6, rx7900::navi31_device_id);
    write_u16(bytes, pcir + 0x0a, 0x18);
    bytes.at(pcir + 0x0f) = 3;
    write_u16(bytes, pcir + 0x10, 1);
    write_u32(bytes, atom + 4, 0x4d4f5441);
    rx7900::AtomImage image{};
    const auto parse = [&](size_t length) {
        return rx7900::validate_atom_image(rx7900::ByteView(bytes.data(), length),
            rx7900::amd_vendor_id, rx7900::navi31_device_id, image);
    };

    expect(parse(bytes.size()) == rx7900::ParseStatus::success && image.image_bytes == 512 &&
        image.header.offset == atom, "bind ATOM to matching PCI ROM image extent");

    for (size_t length = 0; length < image.image_bytes; ++length) {
        expect(parse(length) != rx7900::ParseStatus::success,
               "every truncated declared PCI ROM extent fails");
    }

    expect(image.image_bytes == 512 && image.header.offset == atom,
           "failed ROM parse preserves previously validated metadata");
    write_u16(bytes, pcir + 6, 0x1234);
    expect(parse(bytes.size()) == rx7900::ParseStatus::unsupported_device,
           "reject ROM for a different PCI device");
    write_u16(bytes, pcir + 6, rx7900::navi31_device_id);
    write_u16(bytes, 0x48, 700);
    write_u32(bytes, 704, 0x4d4f5441);
    expect(parse(bytes.size()) == rx7900::ParseStatus::truncated_input,
           "ATOM signature in a trailing image cannot escape first-image bounds");
    write_u16(bytes, 0x48, atom);
    write_u16(bytes, pcir + 0x0a, 512);
    expect(parse(bytes.size()) == rx7900::ParseStatus::malformed_header,
           "PCIR header cannot extend beyond its image");
    write_u16(bytes, pcir + 0x0a, 0x18);
    write_u16(bytes, pcir + 0x10, 0);
    expect(parse(bytes.size()) == rx7900::ParseStatus::malformed_header,
           "zero image extent is not a valid ROM");
    write_u16(bytes, pcir + 0x10, UINT16_MAX);
    expect(parse(bytes.size()) == rx7900::ParseStatus::truncated_input,
           "maximum image units cannot wrap the byte extent");
    write_u16(bytes, pcir + 0x10, 1);
    bytes.at(pcir + 0x14) = 0x7f;
    expect(parse(bytes.size()) == rx7900::ParseStatus::malformed_header,
           "unknown option-ROM code type is rejected");
}

} // namespace

int main() {
    check_byte_view();
    check_pci();
    check_atom();
    check_atom_image();
    std::printf("%u checks; %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

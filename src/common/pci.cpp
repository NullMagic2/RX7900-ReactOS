/*
Module: rx7900.common.pci
Purpose: Validate device identity and retain all bits of 64-bit PCI BARs.
Created: 2026-10-21
Architecture: Single allocation-free parser shared by NT and host inspection.
*/
#include "pci.hpp"

namespace rx7900 {
namespace {

constexpr uint32_t vga_display_class = 0x030000;
constexpr uint32_t other_display_class = 0x038000;
constexpr uint32_t pci_bar_io_bit = 1;
constexpr uint32_t pci_bar_type_mask = 6;
constexpr uint32_t pci_bar_type64 = 4;
constexpr uint32_t pci_bar_prefetch_bit = 8;
constexpr uint32_t pci_bar_io_address_mask = 0xfffffffcu;
constexpr uint32_t pci_bar_memory_address_mask = 0xfffffff0u;

ParseStatus decode_bars(ByteView config, Adapter& adapter) noexcept {
    for (uint32_t index = 0; index < pci_bar_count; ++index) {
        uint32_t low = 0;
        const size_t offset = pci_bars_offset + index * sizeof(uint32_t);

        if (!config.read_u32(offset, low)) {
            return ParseStatus::truncated_input;
        }
        if (low == 0) {
            continue;
        }
        if (low == UINT32_MAX) {
            return ParseStatus::malformed_header;
        }

        PciBar& bar = adapter.bars[index];

        if ((low & pci_bar_io_bit) != 0) {
            bar.kind = static_cast<uint32_t>(BarKind::io);
            bar.bus_address = low & pci_bar_io_address_mask;
            continue;
        }

        const uint32_t type = low & pci_bar_type_mask;

        if (type != 0 && type != pci_bar_type64) {
            return ParseStatus::malformed_header;
        }

        bar.bus_address = low & pci_bar_memory_address_mask;
        bar.flags = (low & pci_bar_prefetch_bit) != 0 ? bar_prefetchable : 0;
        bar.kind = static_cast<uint32_t>(BarKind::memory32);

        if (type == pci_bar_type64) {
            uint32_t high = 0;

            if (index + 1 >= pci_bar_count ||
                !config.read_u32(offset + sizeof(uint32_t), high)) {
                return ParseStatus::malformed_header;
            }

            bar.bus_address |= static_cast<uint64_t>(high) << 32;
            bar.kind = static_cast<uint32_t>(BarKind::memory64);
            ++index;
            adapter.bars[index].kind = static_cast<uint32_t>(BarKind::continuation);
        }
    }

    return ParseStatus::success;
}

} // namespace

ParseStatus decode_adapter(ByteView config, PciLocation location,
                           Adapter& adapter) noexcept {
    adapter = {};

    if (config.size() < pci_header_bytes) {
        return ParseStatus::truncated_input;
    }
    if (location.slot >= 32 || location.function >= 8) {
        return ParseStatus::malformed_header;
    }

    Adapter decoded{};
    uint8_t header_type = 0;
    uint8_t base_class = 0;
    uint8_t subclass = 0;
    uint8_t programming_interface = 0;

    if (!config.read_u16(pci_vendor_offset, decoded.vendor_id) ||
        !config.read_u16(pci_device_offset, decoded.device_id) ||
        !config.read_u8(pci_header_type_offset, header_type) ||
        !config.read_u8(pci_class_offset, base_class) ||
        !config.read_u8(pci_subclass_offset, subclass) ||
        !config.read_u8(pci_interface_offset, programming_interface) ||
        !config.read_u8(pci_revision_offset, decoded.revision) ||
        !config.read_u16(pci_subsystem_vendor_offset, decoded.subsystem_vendor_id) ||
        !config.read_u16(pci_subsystem_device_offset, decoded.subsystem_device_id)) {
        return ParseStatus::truncated_input;
    }

    decoded.class_code = (static_cast<uint32_t>(base_class) << 16) |
                         (static_cast<uint32_t>(subclass) << 8) |
                         programming_interface;

    if (decoded.vendor_id != amd_vendor_id || !is_navi31_device(decoded.device_id) ||
        (decoded.class_code != vga_display_class &&
         decoded.class_code != other_display_class)) {
        return ParseStatus::unsupported_device;
    }
    if ((header_type & pci_header_type_mask) != 0) {
        return ParseStatus::malformed_header;
    }

    decoded.pci_domain = location.domain;
    decoded.bus = location.bus;
    decoded.slot = location.slot;
    decoded.function = location.function;
    const ParseStatus result = decode_bars(config, decoded);

    if (result == ParseStatus::success) {
        adapter = decoded;
    }

    return result;
}

} // namespace rx7900

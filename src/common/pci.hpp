/*
Module: rx7900.common.pci
Purpose: Decode Navi 31 PCI identity and BAR addresses once for every consumer.
Created: 2026-10-21
Architecture: Pure parser with no configuration-space writes or hardware claims.
*/
#pragma once

#include "byte_view.hpp"
#include "protocol.hpp"
#include "status.hpp"

namespace rx7900 {

constexpr size_t pci_header_bytes = 64;
constexpr size_t pci_vendor_offset = 0;
constexpr size_t pci_device_offset = 2;
constexpr size_t pci_revision_offset = 8;
constexpr size_t pci_interface_offset = 9;
constexpr size_t pci_subclass_offset = 10;
constexpr size_t pci_class_offset = 11;
constexpr size_t pci_header_type_offset = 14;
constexpr size_t pci_bars_offset = 16;
constexpr size_t pci_subsystem_vendor_offset = 44;
constexpr size_t pci_subsystem_device_offset = 46;
constexpr uint16_t amd_vendor_id = 0x1002;
constexpr uint16_t navi31_device_id = 0x744c;
// Navi31 GPU functions (pci.ids): W7900 0x7448, W7900 Dual Slot 0x744a, W7900D
// 0x744b, RX 7900 XT/XTX/GRE/7900M 0x744c, W7800 0x745e. The card's USB (0x7446)
// and HDMI/DP audio (0xab30) functions are not GPUs. Linux matches any AMD
// display-class device and identifies the ASIC from IP discovery; ASIC
// initialization still refuses any part whose discovered IP versions differ.
constexpr uint16_t navi31_device_ids[]{0x7448, 0x744a, 0x744b, 0x744c, 0x745e};

constexpr bool is_navi31_device(uint16_t device_id) noexcept {
    for (const uint16_t known : navi31_device_ids) {
        if (known == device_id) {
            return true;
        }
    }

    return false;
}
constexpr uint8_t pci_multifunction = 0x80;
constexpr uint8_t pci_header_type_mask = 0x7f;

struct PciLocation final {
    uint32_t domain;
    uint8_t bus;
    uint8_t slot;
    uint8_t function;
};

// Input is the complete type-0 header, not a native PCI_COMMON_CONFIG overlay.
// Output is cleared on failure. BARs are PCI bus addresses, not mapped resources.
ParseStatus decode_adapter(ByteView config, PciLocation location,
                           Adapter& adapter) noexcept;

} // namespace rx7900

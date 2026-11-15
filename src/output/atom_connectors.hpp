/*
Module: rx7900.output.atom_connectors
Purpose: Discover real connector routing and DDC/HPD assignments from ATOM ROM.
Created: 2026-10-21
Architecture: Checked firmware input reused by the native VideoPort miniport.
Linux AMD source notices and exact provenance: licenses/AMDGPU-OUTPUT.txt.
*/
#pragma once

#include "atom_bios.hpp"

namespace rx7900::output {

enum class FirmwareStatus {
    success,
    truncated_input,
    invalid_table,
    unsupported_revision,
    capacity_exceeded,
    duplicate_connector
};

enum class ConnectorKind {
    unknown,
    hdmi_type_a,
    display_port,
    embedded_display_port,
    usb_type_c
};

constexpr uint32_t connector_capacity = 32;

// An ATOM register index, not a byte offset, CPU pointer or physical address.
// Its hardware block/address space must be resolved before any MMIO access.
struct AtomGpioAssignment final {
    uint32_t register_index = 0;
    uint8_t bit_shift = 0;
    uint8_t mask_bit_shift = 0;
    bool found = false;
};

struct Connector final {
    ConnectorKind kind = ConnectorKind::unknown;
    uint16_t connector_object_id = 0;
    uint16_t encoder_object_id = 0;
    uint16_t external_encoder_object_id = 0;
    uint16_t device_tag = 0;
    uint8_t firmware_path_index = 0;
    uint8_t i2c_id = 0;
    uint8_t i2c_slave_address = 0;
    uint8_t hpd_pin_id = 0;
    uint8_t hpd_plugged_level = 0;
    bool has_i2c_record = false;
    bool has_hpd_record = false;
    AtomGpioAssignment ddc_gpio{};
    AtomGpioAssignment hpd_gpio{};
};

// Firmware topology only. A connector record does not prove a monitor is plugged
// in or establish link capabilities. No ROM pointer escapes parse_connectors.
struct ConnectorCatalog final {
    Connector connectors[connector_capacity]{};
    uint32_t count = 0;
};

// Any IRQL with resident inputs. Caller owns complete ROM bytes for this call,
// serializes replacement and supplies approximately 2 KiB of parser stack space.
// Supports ATOM ROM2.2, master-data2.1 and object-info1.4/1.5. Skips generic paths
// without an encoder; malformed tables/ambiguous duplicate records fail without
// changing output. GPIO lookup may be absent and remains explicitly unresolved.
// Never executes ATOM command tables or touches DDC/AUX/GPIO/encoder hardware.
FirmwareStatus parse_connectors(ByteView rom, ConnectorCatalog& output) noexcept;

} // namespace rx7900::output

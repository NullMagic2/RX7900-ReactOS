/*
Module: rx7900.output.atom_connectors
Purpose: Adapt AMD BIOS display-path and record traversal into bounded C++.
Created: 2026-10-21
Architecture: Reuses common ATOM signature validation and ByteView binary reads.
Copyright 2012-15 Advanced Micro Devices, Inc.
Copyright 2014 Advanced Micro Devices, Inc.
Copyright 2006-2007 Advanced Micro Devices, Inc.
Permission notices and pinned upstream files: licenses/AMDGPU-OUTPUT.txt.
*/
#include "atom_connectors.hpp"

namespace rx7900::output {
namespace {

constexpr size_t common_header_bytes = 4;
constexpr size_t gpio_table_entry = 12;
constexpr size_t object_table_entry = 22;
constexpr size_t object_header_bytes = 8;
constexpr size_t path_bytes = 16;
constexpr size_t gpio_assignment_bytes = 8;
constexpr uint8_t i2c_record_type = 1;
constexpr uint8_t hpd_record_type = 2;
constexpr uint8_t end_record_type = 0xff;

using Table = AtomTable;

// Shared common/atom_bios table lookup; an absent entry is mapped by each caller.
FirmwareStatus firmware_status(AtomTableStatus status) noexcept {
    switch (status) {
    case AtomTableStatus::success: return FirmwareStatus::success;
    case AtomTableStatus::truncated_input: return FirmwareStatus::truncated_input;
    case AtomTableStatus::unsupported_revision: return FirmwareStatus::unsupported_revision;
    case AtomTableStatus::absent:
    case AtomTableStatus::invalid_table:
    default: return FirmwareStatus::invalid_table;
    }
}

uint8_t byte_at(ByteView rom, size_t offset) noexcept {
    uint8_t value = 0;
    rom.read_u8(offset, value);
    return value;
}

uint16_t word_at(ByteView rom, size_t offset) noexcept {
    uint16_t value = 0;
    rom.read_u16(offset, value);
    return value;
}

bool is_object(uint16_t value, uint16_t type) noexcept {
    return (value & 0x7000u) == type && (value & 0x0700u) != 0 &&
           (value & 0x00ffu) != 0 && (value & 0x8800u) == 0;
}

ConnectorKind connector_kind(uint16_t object) noexcept {
    switch (object & 0x00ffu) {
    case 0x0c: return ConnectorKind::hdmi_type_a;
    case 0x13: return ConnectorKind::display_port;
    case 0x14: return ConnectorKind::embedded_display_port;
    case 0x17: return ConnectorKind::usb_type_c;
    default: return ConnectorKind::unknown;
    }
}

FirmwareStatus parse_records(ByteView rom, const Table& table, size_t path_end,
                             uint16_t relative, Connector& connector) noexcept {
    if (relative == 0) {
        return FirmwareStatus::success;
    }

    size_t offset = relative;

    if (offset < path_end || offset >= table.bytes) {
        return FirmwareStatus::invalid_table;
    }

    while (offset < table.bytes) {
        if (table.bytes - offset < 2) {
            return FirmwareStatus::invalid_table;
        }

        const uint8_t type = byte_at(rom, table.base + offset);
        const uint8_t bytes = byte_at(rom, table.base + offset + 1);

        if (type == end_record_type) {
            return FirmwareStatus::success;
        }

        if (bytes < 2 || bytes > table.bytes - offset) {
            return FirmwareStatus::invalid_table;
        }

        if (type == i2c_record_type) {
            if (bytes < 4 || connector.has_i2c_record) {
                return FirmwareStatus::invalid_table;
            }

            connector.i2c_id = byte_at(rom, table.base + offset + 2);
            connector.i2c_slave_address = byte_at(rom, table.base + offset + 3);
            connector.has_i2c_record = true;
        } else if (type == hpd_record_type) {
            if (bytes < 4 || connector.has_hpd_record) {
                return FirmwareStatus::invalid_table;
            }

            connector.hpd_pin_id = byte_at(rom, table.base + offset + 2);
            connector.hpd_plugged_level = byte_at(rom, table.base + offset + 3);

            if (connector.hpd_plugged_level > 1) {
                return FirmwareStatus::invalid_table;
            }

            connector.has_hpd_record = true;
        }

        offset += bytes;
    }

    return FirmwareStatus::invalid_table; // Require an in-table terminator.
}

FirmwareStatus resolve_gpio(ByteView rom, ConnectorCatalog& connectors) noexcept {
    Table gpio{};
    const AtomTableStatus lookup = atom_data_table(rom, gpio_table_entry, gpio);

    if (lookup == AtomTableStatus::absent) {
        return FirmwareStatus::success;
    }

    if (lookup != AtomTableStatus::success) {
        return firmware_status(lookup);
    }

    if (gpio.format != 2 || gpio.revision != 1) {
        return FirmwareStatus::unsupported_revision;
    }

    if ((gpio.bytes - common_header_bytes) % gpio_assignment_bytes != 0) {
        return FirmwareStatus::invalid_table;
    }

    // Eight-byte records and the 16-bit table size give a finite work bound.
    for (size_t offset = common_header_bytes; offset < gpio.bytes;
         offset += gpio_assignment_bytes) {
        AtomGpioAssignment assignment{};
        rom.read_u32(gpio.base + offset, assignment.register_index);
        assignment.bit_shift = byte_at(rom, gpio.base + offset + 4);
        assignment.mask_bit_shift = byte_at(rom, gpio.base + offset + 5);
        const uint8_t id = byte_at(rom, gpio.base + offset + 6);

        if (assignment.bit_shift >= 32 || assignment.mask_bit_shift >= 32) {
            return FirmwareStatus::invalid_table;
        }

        assignment.found = true;

        for (uint32_t index = 0; index < connectors.count; ++index) {
            Connector& connector = connectors.connectors[index];

            // Matching all three I2C id subfields is equivalent to byte equality;
            // retain the raw id so the native service resolves its engine/lane.
            if (connector.has_i2c_record && connector.i2c_id == id) {
                if (connector.ddc_gpio.found) {
                    return FirmwareStatus::invalid_table;
                }

                connector.ddc_gpio = assignment;
            }

            if (connector.has_hpd_record && connector.hpd_pin_id == id) {
                if (connector.hpd_gpio.found) {
                    return FirmwareStatus::invalid_table;
                }

                connector.hpd_gpio = assignment;
            }
        }
    }

    return FirmwareStatus::success;
}

} // namespace

FirmwareStatus parse_connectors(ByteView rom, ConnectorCatalog& output) noexcept {
    Table objects{};
    FirmwareStatus status = firmware_status(atom_data_table(rom, object_table_entry, objects));

    if (status != FirmwareStatus::success) {
        return status;
    }

    if (objects.format != 1 || (objects.revision != 4 && objects.revision != 5)) {
        return FirmwareStatus::unsupported_revision;
    }

    if (objects.bytes < object_header_bytes) {
        return FirmwareStatus::invalid_table;
    }

    const uint8_t path_count = byte_at(rom, objects.base + 6);

    if (path_count > connector_capacity) {
        return FirmwareStatus::capacity_exceeded;
    }

    const size_t paths_end = object_header_bytes + path_bytes * path_count;

    if (paths_end > objects.bytes) {
        return FirmwareStatus::invalid_table;
    }

    ConnectorCatalog candidate{};

    for (uint32_t index = 0; index < path_count; ++index) {
        const size_t path = objects.base + object_header_bytes + path_bytes * index;
        const uint16_t encoder = word_at(rom, path + 4);

        if (encoder == 0) {
            continue; // AMD firmware includes generic non-connector paths.
        }

        Connector connector{};
        connector.connector_object_id = word_at(rom, path);
        connector.encoder_object_id = encoder;
        connector.external_encoder_object_id = objects.revision == 4 ?
                                                 word_at(rom, path + 6) : 0;
        connector.device_tag = word_at(rom, path + 12);
        connector.firmware_path_index = static_cast<uint8_t>(index);

        if (!is_object(connector.connector_object_id, 0x3000) ||
            !is_object(encoder, 0x2000) ||
            (connector.external_encoder_object_id != 0 &&
             !is_object(connector.external_encoder_object_id, 0x2000))) {
            return FirmwareStatus::invalid_table;
        }

        for (uint32_t prior = 0; prior < candidate.count; ++prior) {
            if (candidate.connectors[prior].connector_object_id ==
                connector.connector_object_id) {
                return FirmwareStatus::duplicate_connector;
            }
        }

        connector.kind = connector_kind(connector.connector_object_id);
        status = parse_records(rom, objects, paths_end, word_at(rom, path + 2), connector);

        if (status != FirmwareStatus::success) {
            return status;
        }

        candidate.connectors[candidate.count] = connector;
        ++candidate.count;
    }

    status = resolve_gpio(rom, candidate);

    if (status != FirmwareStatus::success) {
        return status;
    }

    output = candidate;
    return FirmwareStatus::success;
}

} // namespace rx7900::output

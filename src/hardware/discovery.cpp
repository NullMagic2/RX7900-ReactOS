/*
Module: rx7900.hardware.discovery
Purpose: Validate checksummed IP-discovery records for shared register descriptors.
Created: 2026-10-21
Architecture: Extracted from the GFXHUB validator; allocation-free and no MMIO.
Copyright 2018 Advanced Micro Devices, Inc.
Adapted Linux v6.12 amdgpu_discovery.c, discovery.h and soc15_hw_ip.h.
Permission/provenance: licenses/AMDGPU-SDMA.txt, licenses/AMDGPU-FIRMWARE.txt.
*/
#include "discovery.hpp"

namespace rx7900::hardware {
namespace {

uint32_t u32(ByteView view, size_t offset) noexcept {
    uint32_t value = 0;
    view.read_u32(offset, value);
    return value;
}

uint16_t u16(ByteView view, size_t offset) noexcept {
    uint16_t value = 0;
    view.read_u16(offset, value);
    return value;
}

uint8_t u8(ByteView view, size_t offset) noexcept {
    uint8_t value = 0;
    view.read_u8(offset, value);
    return value;
}

uint16_t checksum(ByteView view, size_t offset, size_t count) noexcept {
    uint16_t sum = 0;

    for (size_t index = 0; index < count; ++index) {
        sum = static_cast<uint16_t>(sum + u8(view, offset + index));
    }

    return sum;
}

} // namespace

bool discovery_register_offset(uint32_t base, uint32_t reg, uint32_t aperture,
                                uint32_t& output) noexcept {
    const uint64_t offset = (static_cast<uint64_t>(base) + reg) * sizeof(uint32_t);

    if ((aperture & 3) != 0 || offset > aperture || sizeof(uint32_t) > aperture - offset) {
        return false;
    }

    output = static_cast<uint32_t>(offset);
    return true;
}

uint32_t discovery_mall_half_use(ByteView discovery) noexcept {
    // binary_header table_list[MALL_INFO] (slot 4, 8-byte table_info entries
    // from byte 12): u16 offset; a v1 table carries m_half_use at byte 20.
    constexpr size_t mall_table_offset = 12 + 4 * 8;
    constexpr uint32_t mall_info_table_id = 0x4c4c414du;
    uint16_t offset = 0;
    uint32_t table_id = 0;
    uint16_t major = 0;
    uint32_t half_use = 0;

    if (!discovery.read_u16(mall_table_offset, offset) || offset == 0 ||
        !discovery.read_u32(offset, table_id) || table_id != mall_info_table_id ||
        !discovery.read_u16(offset + 4u, major) || major != 1 ||
        !discovery.read_u32(offset + 20u, half_use)) {
        return 0;
    }

    return half_use;
}

bool discovery_harvest(ByteView discovery, uint16_t hardware_id, uint32_t& mask, uint32_t& count) noexcept {
    // table_list[HARVEST_INFO] (slot 2); harvest_table: an 8-byte header, then
    // 32 harvest_info entries {u16 hw_id, u8 number_instance, u8 reserved}
    // ending at the first zero hw_id.
    constexpr size_t harvest_table_offset = 12 + 2 * 8;
    constexpr size_t harvest_header_bytes = 8;
    constexpr size_t harvest_entries = 32;
    constexpr size_t harvest_entry_bytes = 4;
    uint16_t offset = 0;
    uint32_t found_mask = 0;
    uint32_t found_count = 0;
    mask = 0;
    count = 0;

    if (!discovery.read_u16(harvest_table_offset, offset)) {
        return false;
    }

    if (offset == 0) {
        return true;
    }

    for (size_t index = 0; index < harvest_entries; ++index) {
        const size_t entry = offset + harvest_header_bytes + index * harvest_entry_bytes;
        uint16_t id = 0;
        uint8_t instance = 0;

        if (!discovery.read_u16(entry, id)) {
            return false;
        }

        if (id == 0) {
            break;
        }

        if (id != hardware_id) {
            continue;
        }

        if (!discovery.read_u8(entry + 2, instance) || instance >= 32) {
            return false;
        }

        found_mask |= 1u << instance;
        ++found_count;
    }

    mask = found_mask;
    count = found_count;
    return true;
}

DiscoveryStatus discover_ip(ByteView binary, DiscoveryQuery query, DiscoveredIp& output) noexcept {
    output = {};
    constexpr size_t binary_header_bytes = 60;
    constexpr size_t ip_header_bytes = 80;

    if (query.required_segments == 0 || query.required_segments > discovery_max_segments) {
        return DiscoveryStatus::invalid_argument;
    }

    if (!binary.contains(0, binary_header_bytes) || u32(binary, 0) != 0x28211407) {
        return DiscoveryStatus::invalid_binary;
    }

    const size_t binary_bytes = u16(binary, 10);
    const size_t table = u16(binary, 12);

    if (binary_bytes < binary_header_bytes || !binary.contains(0, binary_bytes) ||
        checksum(binary, 10, binary_bytes - 10) != u16(binary, 8) ||
        table < binary_header_bytes || table > binary_bytes ||
        ip_header_bytes > binary_bytes - table || u32(binary, table) != 0x53445049) {
        return DiscoveryStatus::invalid_binary;
    }

    const size_t table_bytes = u16(binary, table + 6);
    const uint16_t version = u16(binary, table + 4);
    const size_t die_count = u16(binary, table + 12);

    if (table_bytes < ip_header_bytes || table_bytes > binary_bytes - table ||
        (u16(binary, 16) != 0 && u16(binary, 16) != table_bytes) ||
        checksum(binary, table, table_bytes) != u16(binary, 14) ||
        die_count == 0 || die_count > 16) {
        return DiscoveryStatus::invalid_binary;
    }

    if (version != 3 && version != 4) {
        return DiscoveryStatus::unsupported;
    }

    const uint8_t address_flags = u8(binary, table + 78);

    if ((version == 3 && address_flags != 0) || (address_flags & ~1u) != 0 ||
        u8(binary, table + 79) != 0) {
        return DiscoveryStatus::unsupported;
    }

    const size_t address_bytes = version == 4 && (address_flags & 1) != 0 ? 8 : 4;
    const size_t table_end = table + table_bytes;
    size_t die_starts[16]{};
    size_t die_ends[16]{};
    DiscoveredIp found;

    for (size_t die = 0; die < die_count; ++die) {
        const size_t die_info = table + 14 + die * 4;
        const size_t die_offset = u16(binary, die_info + 2);

        if (die_offset < table + ip_header_bytes || die_offset > table_end ||
            4 > table_end - die_offset || u16(binary, die_offset) != u16(binary, die_info)) {
            return DiscoveryStatus::invalid_binary;
        }

        const size_t ip_count = u16(binary, die_offset + 2);
        size_t cursor = die_offset + 4;

        if (ip_count > (table_end - cursor) / 8) {
            return DiscoveryStatus::invalid_binary;
        }

        for (size_t ip = 0; ip < ip_count; ++ip) {
            if (cursor > table_end || 8 > table_end - cursor) {
                return DiscoveryStatus::invalid_binary;
            }

            const size_t bases = u8(binary, cursor + 3);
            const size_t entry_bytes = 8 + bases * address_bytes;

            if (entry_bytes > table_end - cursor) {
                return DiscoveryStatus::invalid_binary;
            }

            if (u16(binary, cursor) == query.hardware_id && u8(binary, cursor + 2) == query.instance) {
                if (found.valid() || bases < query.required_segments ||
                    u8(binary, cursor + 4) != query.major || u8(binary, cursor + 5) != query.minor ||
                    u8(binary, cursor + 6) != query.revision || u8(binary, cursor + 7) != 0) {
                    return DiscoveryStatus::unsupported;
                }

                for (size_t segment = 0; segment < query.required_segments; ++segment) {
                    const size_t address = cursor + 8 + segment * address_bytes;
                    const uint32_t low = u32(binary, address);

                    if ((low & 0xc0000000u) != 0 ||
                        (address_bytes == 8 && u32(binary, address + 4) != 0)) {
                        return DiscoveryStatus::unsupported;
                    }

                    found.bases_[segment] = low;
                }

                found.segment_count_ = query.required_segments;
            }

            cursor += entry_bytes;
        }

        for (size_t prior = 0; prior < die; ++prior) {
            if (die_offset < die_ends[prior] && die_starts[prior] < cursor) {
                return DiscoveryStatus::invalid_binary;
            }
        }

        die_starts[die] = die_offset;
        die_ends[die] = cursor;
    }

    if (!found.valid()) {
        return DiscoveryStatus::invalid_binary;
    }

    output = found;
    return DiscoveryStatus::validated;
}

} // namespace rx7900::hardware

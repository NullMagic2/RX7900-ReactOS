/*
Module: rx7900.hardware.discovery
Purpose: Share checked IP-discovery traversal and local register-base provenance.
Created: 2026-10-21
Architecture: Immutable borrowed binary input, bounded tokens, no hardware access.
*/
#pragma once

#include "byte_view.hpp"

namespace rx7900::hardware {

constexpr uint16_t discovery_gc_hardware_id = 11;
constexpr uint16_t discovery_mp0_hardware_id = 255;
constexpr uint16_t discovery_mp1_hardware_id = 1;
constexpr uint16_t discovery_thm_hardware_id = 3;
constexpr uint16_t discovery_mmhub_hardware_id = 34;
constexpr uint16_t discovery_nbif_hardware_id = 108;
constexpr uint16_t discovery_umc_hardware_id = 150;
// Local segment bases copied per IP; Linux soc15 register tables use up to six.
constexpr uint8_t discovery_max_segments = 6;

enum class DiscoveryStatus { validated, invalid_argument, invalid_binary, unsupported };

struct DiscoveryQuery final {
    uint16_t hardware_id;
    uint8_t instance;
    uint8_t major;
    uint8_t minor;
    uint8_t revision;
    uint8_t required_segments; // 1..discovery_max_segments local bases; no remote/XCC extensions.
};

class DiscoveredIp final {
public:
    DiscoveredIp() noexcept = default;
    bool valid() const noexcept { return segment_count_ != 0; }
    uint32_t segment(size_t index) const noexcept { return index < segment_count_ ? bases_[index] : 0; }

private:
    friend DiscoveryStatus discover_ip(ByteView, DiscoveryQuery, DiscoveredIp&) noexcept;
    uint32_t bases_[discovery_max_segments]{};
    uint8_t segment_count_ = 0;
};

// Serialized native-owner operation on readable immutable discovery bytes. Copies
// only validated bases; the binary may be released after return. Checks both
// byte-sum checksums, complete die/IP extents and nonoverlapping die records.
// Exactly one matching hardware-ID/instance with exact version/subrevision0 is
// required. Output is cleared on every failure; no hardware presence is inferred.
DiscoveryStatus discover_ip(ByteView binary, DiscoveryQuery query, DiscoveredIp& output) noexcept;

// Translate a discovered dword base plus register index into an owned BAR byte
// offset. Output is unchanged on failure; a whole aligned dword must fit.
bool discovery_register_offset(uint32_t base, uint32_t reg, uint32_t aperture_bytes,
                                uint32_t& output) noexcept;

// amdgpu_discovery_get_mall_info: binary_header table_list[MALL_INFO]; a v1
// table carries m_half_use (adev->gmc.m_half_use). No table, another version or
// a table outside the binary gives 0 (Linux: v2 has none).
uint32_t discovery_mall_half_use(ByteView binary) noexcept;

// amdgpu_discovery_read_from_harvest_table for one hardware ID: the harvested
// instance mask and the number of harvest entries naming that ID. Linux treats
// a zero table_list[HARVEST_INFO] offset as "nothing harvested" (mask 0, count
// 0, true). false when an entry before the terminator lies outside binary or
// names an instance >= 32 (Linux would shift past its 32-bit mask).
bool discovery_harvest(ByteView binary, uint16_t hardware_id, uint32_t& mask, uint32_t& count) noexcept;

} // namespace rx7900::hardware

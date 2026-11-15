/*
Module: rx7900.hardware.umc_v8_10
Purpose: UMC 8.10 topology validation, MCA error-record queries and the
normalized-to-physical VRAM address conversion.
Created: 2026-10-24
Architecture: See umc_v8_10.hpp. Register values are transcribed from the
pinned umc_8_10_0_offset.h/_sh_mask.h (UMC BASE_IDX 2); the per-channel
register distances, channel tables and swizzle arithmetic follow umc_v8_10.c/.h.
Copyright 2019, 2022 Advanced Micro Devices, Inc. Notice and provenance:
licenses/AMDGPU-FIRMWARE.txt.
*/
#include "umc_v8_10.hpp"

namespace rx7900::hardware {
namespace {

// umc_8_10_0_offset.h, all BASE_IDX 2.
constexpr uint32_t umc_gecc_err_cnt_sel = 0x0328;
constexpr uint32_t umc_gecc_err_cnt = 0x0329;
constexpr uint32_t umc_mcumc_status = 0x03c2;
constexpr uint32_t umc_mcumc_addr = 0x03c4;
// The highest dword any access touches (ADDRT0 high half).
constexpr uint32_t umc_last_register = umc_mcumc_addr + 1;
constexpr uint32_t umc_discovery_segment = 2;
// umc_8_10_0_sh_mask.h UMCCH0_0_GeccErrCntSel GeccErrInt (bits 13:12).
constexpr uint32_t umc_gecc_err_int_shift = 12;
constexpr uint32_t umc_gecc_err_int_mask = 0x00003000u;
// UMC_V8_10_CE_CNT_INIT = UMC_V8_10_CE_CNT_MAX - UUMC_V8_10_CE_INT_THRESHOLD.
constexpr uint32_t umc_ce_count_init = 0xffffu - 0xffffu;
// umc_v8_10.c / umc_v8_10.h register distances in dwords.
constexpr uint32_t umc_node_distance = 0x800000;
constexpr uint32_t umc_instance_distance = 0x4000;
constexpr uint32_t umc_channel_distance = 0x400; // UMC_V8_10_PER_CHANNEL_OFFSET
// UMC_V8_10_NA_C5_BIT: [C6 C5] are normalized-address bits 15:14.
constexpr uint32_t umc_na_c5_bit = 14;
// Discovery instance numbers are 8-bit; Linux counts every UMC record.
constexpr uint32_t umc_discovery_instances = 32;

struct ChannelColumnBit final {
    uint32_t channels;
    uint32_t column_bit;
};

// umc_v8_10_channelnum_map_colbit_table.
constexpr ChannelColumnBit channel_column_bits[]{
    {24, 13}, {20, 13}, {16, 12}, {14, 12}, {12, 12}, {10, 12}, {6, 11},
};

// umc_v8_10_channel_idx_tbl[node][umc][channel] and _ext0 (four nodes).
constexpr uint8_t channel_table[umc_v8_10_max_nodes][umc_v8_10_umc_instances][umc_v8_10_channel_instances]{
    {{16, 18}, {17, 19}}, {{15, 11}, {3, 7}}, {{1, 5}, {13, 9}},
    {{23, 21}, {22, 20}}, {{0, 4}, {12, 8}}, {{14, 10}, {2, 6}},
};
constexpr uint32_t ext0_nodes = 4;
constexpr uint8_t channel_table_ext0[ext0_nodes][umc_v8_10_umc_instances][umc_v8_10_channel_instances]{
    {{1, 5}, {7, 3}}, {{14, 15}, {13, 12}}, {{10, 11}, {9, 8}}, {{6, 2}, {0, 4}},
};

uint32_t population(uint32_t value) noexcept {
    uint32_t count = 0;

    for (; value != 0; value &= value - 1) {
        ++count;
    }

    return count;
}

uint32_t column_bit_for(uint32_t channels) noexcept {
    for (const auto& entry : channel_column_bits) {
        if (entry.channels == channels) {
            return entry.column_bit;
        }
    }

    return 0;
}

// get_umc_v8_10_reg_offset (dwords).
uint32_t channel_offset(uint32_t node, uint32_t umc, uint32_t channel) noexcept {
    return umc_channel_distance * channel + umc_instance_distance * umc + umc_node_distance * node;
}

// umc_v8_10_swizzle_mode_na_to_pa: SWIZZLE_MODE_TMP_ADDR, then the high bits
// in place, NA bits 9:8 at the column bit, the interleaved bits 10.. moved down
// to 8.. and the low byte unchanged. na is at most 56 bits, so tmp < 2^61.
uint64_t swizzle_na_to_pa(uint64_t na, uint32_t channels, uint32_t channel_index, uint32_t column_bit) noexcept {
    const uint64_t tmp = ((na >> 10) * channels + channel_index) << 10;
    const uint64_t high = (tmp >> (column_bit + 2)) << (column_bit + 2);
    const uint64_t middle = ((na >> 8) & 0x3u) << column_bit;
    const uint64_t low = ((tmp >> 10) & ((1ULL << (column_bit - 8)) - 1)) << 8;
    return high | middle | low | (na & 0xffu);
}

} // namespace

bool UmcTopology::channel_index(uint32_t node, uint32_t umc, uint32_t channel, uint32_t& index) const noexcept {
    if (!valid() || node >= node_count_ || umc >= umc_v8_10_umc_instances ||
        channel >= umc_v8_10_channel_instances) {
        return false;
    }

    index = node_count_ == ext0_nodes ? channel_table_ext0[node][umc][channel] : channel_table[node][umc][channel];
    return true;
}

DiscoveryStatus validate_umc_discovery(ByteView binary, UmcTopology& output) noexcept {
    output = {};
    DiscoveredIp first;
    auto status = discover_ip(binary, {discovery_umc_hardware_id, 0, 8, 10, 0, umc_discovery_segment + 1}, first);

    if (status != DiscoveryStatus::validated) {
        return status;
    }

    // amdgpu_discovery_reg_base_init: node_inst_num counts every UMC record;
    // umc_v8_10 addresses node n at reg_offset[UMC][0] + n * UMC_8_NODE_DIST,
    // so the instances must be 0..n-1 with exactly those bases.
    uint32_t nodes = 0;
    bool gap = false;

    for (uint32_t instance = 0; instance < umc_discovery_instances; ++instance) {
        DiscoveredIp ip;
        status = discover_ip(binary, {discovery_umc_hardware_id, static_cast<uint8_t>(instance), 8, 10, 0,
                                      umc_discovery_segment + 1}, ip);

        if (status == DiscoveryStatus::invalid_binary) {
            gap = true;
            continue;
        }

        const uint64_t expected = first.segment(umc_discovery_segment) +
                                  static_cast<uint64_t>(umc_node_distance) * instance;

        if (status != DiscoveryStatus::validated || gap || ip.segment(umc_discovery_segment) != expected) {
            return DiscoveryStatus::unsupported;
        }

        ++nodes;
    }

    if (nodes > umc_v8_10_max_nodes) {
        return DiscoveryStatus::unsupported;
    }

    // amdgpu_discovery_read_from_harvest_table and amdgpu_discovery_harvest_ip.
    uint32_t harvest_mask = 0;
    uint32_t harvest_count = 0;

    if (!discovery_harvest(binary, discovery_umc_hardware_id, harvest_mask, harvest_count)) {
        return DiscoveryStatus::invalid_binary;
    }

    const uint32_t active = ((1u << nodes) - 1) & ~harvest_mask;
    const uint32_t umcs = harvest_count < nodes ? nodes - harvest_count : nodes;
    // UMC_V8_10_TOTAL_CHANNEL_NUM: 2 * 2 * num_umc - hweight32(m_half_use) * 2.
    const uint32_t lanes = umc_v8_10_umc_instances * umc_v8_10_channel_instances * umcs;
    const uint32_t halved = population(discovery_mall_half_use(binary)) * 2;

    if (active == 0 || halved >= lanes) {
        return DiscoveryStatus::unsupported;
    }

    uint32_t highest = 0;

    for (uint32_t node = 0; node < nodes; ++node) {
        highest = (active & (1u << node)) != 0 ? node : highest;
    }

    const uint64_t last = (static_cast<uint64_t>(first.segment(umc_discovery_segment)) +
                           channel_offset(highest, umc_v8_10_umc_instances - 1, umc_v8_10_channel_instances - 1) +
                           umc_last_register) * 4 + 3;

    if (last > UINT32_MAX) {
        return DiscoveryStatus::unsupported;
    }

    output.base_ = first.segment(umc_discovery_segment);
    output.node_count_ = nodes;
    output.active_mask_ = active;
    output.channel_count_ = lanes - halved;
    output.column_bit_ = column_bit_for(lanes - halved);
    return DiscoveryStatus::validated;
}

bool umc_v8_10_convert_error_address(const UmcTopology& topology, uint32_t node, uint32_t umc,
                                     uint32_t channel, uint64_t status, uint64_t error_address,
                                     UmcErrorRecord (&records)[umc_v8_10_retire_unit]) noexcept {
    uint32_t index = 0;

    if (topology.column_bit() == 0 || !topology.channel_index(node, umc, channel, index)) {
        return false;
    }

    // The lowest AddrLsb bits are ignored, then [C6 C5] take every value.
    const uint32_t lsb = static_cast<uint32_t>((status & umc_status_addr_lsb_mask) >> umc_status_addr_lsb_shift);
    const uint64_t address = (error_address & umc_addr_error_mask) & ~((1ULL << lsb) - 1);
    const uint64_t base = address & ~(0x3ULL << umc_na_c5_bit);

    for (uint32_t column = 0; column < umc_v8_10_retire_unit; ++column) {
        const uint64_t na = base | (static_cast<uint64_t>(column) << umc_na_c5_bit);
        const uint64_t pa = swizzle_na_to_pa(na, topology.channel_count(), index, topology.column_bit());
        // amdgpu_umc_fill_error_record keeps the page frame (>> AMDGPU_GPU_PAGE_SHIFT).
        records[column] = {pa & ~0xfffULL, na, index, static_cast<uint8_t>(node), static_cast<uint8_t>(umc),
                           static_cast<uint8_t>(channel)};
    }

    return true;
}

UmcStatus Umc810::initialize(const UmcTopology& topology, NbioRegisters registers, RegisterIo io) noexcept {
    ready_ = false;

    if (!topology.valid()) {
        return UmcStatus::invalid_argument;
    }

    if (nbio_.initialize(registers, io) != NbioStatus::ready) {
        return UmcStatus::unavailable;
    }

    topology_ = topology;
    ready_ = true;
    return UmcStatus::ready;
}

bool Umc810::smn_address(uint32_t reg, uint32_t offset, uint32_t& address) const noexcept {
    // (SOC15_REG_OFFSET(UMC, 0, reg) + umc_reg_offset) * 4 with room for the
    // high dword of a 64-bit register.
    const uint64_t byte = (static_cast<uint64_t>(topology_.base()) + reg + offset) * 4;

    if (!ready_ || byte > UINT32_MAX - 7) {
        return false;
    }

    address = static_cast<uint32_t>(byte);
    return true;
}

// amdgpu_device_indirect_rreg64: low dword, then high dword at +4.
bool Umc810::read64(uint32_t reg, uint32_t offset, uint64_t& value) noexcept {
    uint32_t address = 0;
    uint32_t low = 0;
    uint32_t high = 0;

    if (!smn_address(reg, offset, address) || !nbio_.read_smn(address, low) || !nbio_.read_smn(address + 4, high)) {
        return false;
    }

    value = (static_cast<uint64_t>(high) << 32) | low;
    return true;
}

// amdgpu_device_indirect_wreg64: low dword, then high dword at +4.
bool Umc810::write64(uint32_t reg, uint32_t offset, uint64_t value) noexcept {
    uint32_t address = 0;

    return smn_address(reg, offset, address) && nbio_.write_smn(address, static_cast<uint32_t>(value)) &&
           nbio_.write_smn(address + 4, static_cast<uint32_t>(value >> 32));
}

bool Umc810::read32(uint32_t reg, uint32_t offset, uint32_t& value) noexcept {
    uint32_t address = 0;
    return smn_address(reg, offset, address) && nbio_.read_smn(address, value);
}

bool Umc810::write32(uint32_t reg, uint32_t offset, uint32_t value) noexcept {
    uint32_t address = 0;
    return smn_address(reg, offset, address) && nbio_.write_smn(address, value);
}

bool Umc810::initialize_counters() noexcept {
    if (!ready_) {
        return false;
    }

    for (uint32_t node = 0; node < topology_.node_count(); ++node) {
        if ((topology_.active_mask() & (1u << node)) == 0) {
            continue;
        }

        for (uint32_t umc = 0; umc < umc_v8_10_umc_instances; ++umc) {
            for (uint32_t channel = 0; channel < umc_v8_10_channel_instances; ++channel) {
                const uint32_t offset = channel_offset(node, umc, channel);
                uint32_t select = 0;

                if (!read32(umc_gecc_err_cnt_sel, offset, select) ||
                    !write32(umc_gecc_err_cnt_sel, offset,
                             (select & ~umc_gecc_err_int_mask) | (1u << umc_gecc_err_int_shift)) ||
                    !write32(umc_gecc_err_cnt, offset, umc_ce_count_init)) {
                    return false;
                }
            }
        }
    }

    return true;
}

UmcQuery Umc810::query(UmcErrorRecord* records, uint32_t capacity) noexcept {
    UmcQuery result{};

    if (!ready_ || (records == nullptr && capacity != 0)) {
        result.access_failed = true;
        return result;
    }

    // LOOP_UMC_EACH_NODE_INST_AND_CH over adev->umc.active_mask.
    for (uint32_t node = 0; node < topology_.node_count(); ++node) {
        if ((topology_.active_mask() & (1u << node)) == 0) {
            continue;
        }

        for (uint32_t umc = 0; umc < umc_v8_10_umc_instances; ++umc) {
            for (uint32_t channel = 0; channel < umc_v8_10_channel_instances; ++channel) {
                const uint32_t offset = channel_offset(node, umc, channel);
                uint64_t status = 0;

                if (!read64(umc_mcumc_status, offset, status)) {
                    result.access_failed = true;
                    return result;
                }

                ++result.channels;
                const bool uncorrectable = umc_status_uncorrectable(status);
                result.correctable += umc_status_correctable(status) ? 1 : 0;
                result.uncorrectable += uncorrectable ? 1 : 0;
                result.deferred += uncorrectable && (status & umc_status_deferred) != 0 ? 1 : 0;
                result.overflowed += uncorrectable && (status & umc_status_overflow) != 0 ? 1 : 0;

                if (status != 0) {
                    UmcErrorRecord converted[umc_v8_10_retire_unit]{};
                    bool decoded = false;

                    if (umc_status_has_address(status)) {
                        uint64_t address = 0;

                        if (!read64(umc_mcumc_addr, offset, address)) {
                            result.access_failed = true;
                            return result;
                        }

                        decoded = umc_v8_10_convert_error_address(topology_, node, umc, channel, status,
                                                                  address & umc_addr_error_mask, converted);
                    }

                    if (decoded) {
                        ++result.decoded;

                        for (const auto& record : converted) {
                            if (result.records < capacity) {
                                records[result.records++] = record;
                            } else {
                                ++result.dropped;
                            }
                        }
                    } else if (uncorrectable) {
                        ++result.undecoded;
                    }

                    // umc_v8_10_query_error_address: clear umc status.
                    if (!write64(umc_mcumc_status, offset, 0)) {
                        result.access_failed = true;
                        return result;
                    }
                }

                // umc_v8_10_clear_error_count_per_channel.
                if (!write32(umc_gecc_err_cnt, offset, umc_ce_count_init)) {
                    result.access_failed = true;
                    return result;
                }
            }
        }
    }

    return result;
}

} // namespace rx7900::hardware

/*
Module: rx7900.hardware.umc_v8_10
Purpose: Read, classify, convert and clear the UMC 8.10 MCA error records that
report Navi31 VRAM ECC errors, so that uncorrectable errors retire exact pages.
Created: 2026-10-24
Architecture: UmcTopology is derived once from IP discovery (UMC node count,
harvest table, MALL m_half_use), as amdgpu_discovery and gmc_v11_0_set_umc_funcs
do. Umc810 reaches the per-channel registers only through Nbio's SMN pair
(Linux RREG64_PCIE/WREG64_PCIE/WREG32_PCIE) and is driven by GpuHealth at
PASSIVE_LEVEL under the device serialization. The address conversion is a pure
function, shared with the tests. No policy (retirement, quarantine, recovery)
lives here.
Copyright 2019, 2022 Advanced Micro Devices, Inc.
Adapted Linux v6.12 umc_v8_10.c and umc_v8_10.h (channel tables, swizzle-mode
NA to PA, query_ras_error_count/address, err_cnt_init), umc_8_10_0_offset.h and
umc_8_10_0_sh_mask.h, amdgpu_umc.c and amdgpu_umc.h (loop_channels,
fill_error_record, handle_bad_pages), gmc_v11_0.c (gmc_v11_0_set_umc_funcs),
amdgpu_discovery.c (UMC count, harvest table) and amdgpu_device.c
(indirect_rreg64/wreg64), all MIT, pinned in work/upstream-gpuinit
(SHA256SUMS). Notice and provenance: licenses/AMDGPU-FIRMWARE.txt.
*/
#pragma once

#include "nbio.hpp"

namespace rx7900::hardware {

// UMC_V8_10_CHANNEL_INSTANCE_NUM and UMC_V8_10_UMC_INSTANCE_NUM.
constexpr uint32_t umc_v8_10_channel_instances = 2;
constexpr uint32_t umc_v8_10_umc_instances = 2;
// Rows of umc_v8_10_channel_idx_tbl (umc_v8_10_channel_idx_tbl_ext0 has 4).
constexpr uint32_t umc_v8_10_max_nodes = 6;
constexpr uint32_t umc_v8_10_max_channels =
    umc_v8_10_max_nodes * umc_v8_10_umc_instances * umc_v8_10_channel_instances;
// adev->umc.retire_unit (UMC_V8_10_NA_COL_2BITS_POWER_OF_2_NUM): one MCA error
// retires the pages of all four [C6 C5] column-bit values of its address.
constexpr uint32_t umc_v8_10_retire_unit = 4;
// adev->umc.max_ras_err_cnt_per_query = UMC_V8_10_TOTAL_CHANNEL_NUM: records
// (pages) one query can return; at most 24, so 6 uncorrectable errors.
constexpr uint32_t umc_v8_10_record_capacity = umc_v8_10_max_channels;

// umc_8_10_0_sh_mask.h MCA_UMC_UMC0_MCUMC_STATUST0 fields.
constexpr uint64_t umc_status_val = 1ULL << 63;
constexpr uint64_t umc_status_overflow = 1ULL << 62;
constexpr uint64_t umc_status_uc = 1ULL << 61;
constexpr uint64_t umc_status_addrv = 1ULL << 58;
constexpr uint64_t umc_status_pcc = 1ULL << 57;
constexpr uint64_t umc_status_tcc = 1ULL << 55;
constexpr uint64_t umc_status_cecc = 1ULL << 46;
constexpr uint64_t umc_status_uecc = 1ULL << 45;
constexpr uint64_t umc_status_deferred = 1ULL << 44;
constexpr uint64_t umc_status_poison = 1ULL << 43;
constexpr uint32_t umc_status_addr_lsb_shift = 24;
constexpr uint64_t umc_status_addr_lsb_mask = 0x3fULL << umc_status_addr_lsb_shift;
// MCA_UMC_UMC0_MCUMC_ADDRT0 ErrorAddr.
constexpr uint64_t umc_addr_error_mask = 0x00ffffffffffffffULL;

// The UMC instance layout of one board. Only validate_umc_discovery creates a
// valid token: UMC 8.10.0 instances 0..n-1 (n <= umc_v8_10_max_nodes) whose
// segment 2 bases are UMC_8_NODE_DIST apart, a readable harvest table and
// every per-channel register address inside the 32-bit SMN space.
class UmcTopology final {
public:
    UmcTopology() noexcept = default;
    bool valid() const noexcept { return node_count_ != 0; }
    // reg_offset[UMC_HWIP][0][2]: dword base of every UMC 8.10 register.
    uint32_t base() const noexcept { return base_; }
    // adev->umc.node_inst_num and adev->umc.active_mask.
    uint32_t node_count() const noexcept { return node_count_; }
    uint32_t active_mask() const noexcept { return active_mask_; }
    // UMC_V8_10_TOTAL_CHANNEL_NUM(adev): channels of the swizzle interleave.
    uint32_t channel_count() const noexcept { return channel_count_; }
    // umc_v8_10_get_col_bit(channel_count); 0 when the table has no entry (Linux
    // U32_MAX: errors are counted but no address is converted).
    uint32_t column_bit() const noexcept { return column_bit_; }
    // adev->umc.channel_idx_tbl[node][umc][channel]: the channel_idx_tbl_ext0
    // table when node_count is 4, else channel_idx_tbl. false out of range.
    bool channel_index(uint32_t node, uint32_t umc, uint32_t channel, uint32_t& index) const noexcept;

private:
    friend DiscoveryStatus validate_umc_discovery(ByteView, UmcTopology&) noexcept;
    uint32_t base_ = 0;
    uint32_t node_count_ = 0;
    uint32_t active_mask_ = 0;
    uint32_t channel_count_ = 0;
    uint32_t column_bit_ = 0;
};

// Checks the discovery checksums through discover_ip; output is cleared on
// every failure. Pure, any IRQL.
DiscoveryStatus validate_umc_discovery(ByteView binary, UmcTopology& output) noexcept;

// One amdgpu_umc_fill_error_record entry. vram_offset is the retired page's VRAM
// byte offset (retired_page << AMDGPU_GPU_PAGE_SHIFT: the SoC physical address
// within VRAM, page aligned); normalized is the normalized address (err_rec
// address) with its [C6 C5] variant; channel_index is mem_channel.
struct UmcErrorRecord final {
    uint64_t vram_offset = 0;
    uint64_t normalized = 0;
    uint32_t channel_index = 0;
    uint8_t node = 0;
    uint8_t umc = 0;
    uint8_t channel = 0;
};

// umc_v8_10_convert_error_address for the MCA error of channel (node, umc,
// channel): clears the AddrLsb bits of error_address, then maps every [C6 C5]
// variant through umc_v8_10_swizzle_mode_na_to_pa into records. false (records
// untouched) without a column-bit mapping or for an invalid topology/index.
bool umc_v8_10_convert_error_address(const UmcTopology& topology, uint32_t node, uint32_t umc,
                                     uint32_t channel, uint64_t status, uint64_t error_address,
                                     UmcErrorRecord (&records)[umc_v8_10_retire_unit]) noexcept;

// umc_v8_10_query_correctable_error_count / _uncorrectable_error_count.
inline bool umc_status_correctable(uint64_t status) noexcept {
    return (status & umc_status_val) != 0 && (status & umc_status_cecc) != 0;
}

inline bool umc_status_uncorrectable(uint64_t status) noexcept {
    return (status & umc_status_val) != 0 &&
           (status & (umc_status_deferred | umc_status_uecc | umc_status_pcc | umc_status_uc |
                      umc_status_tcc)) != 0;
}

// umc_v8_10_query_error_address: only a valid UECC record with AddrV has an
// address Linux converts.
inline bool umc_status_has_address(uint64_t status) noexcept {
    return (status & umc_status_val) != 0 && (status & umc_status_addrv) != 0 &&
           (status & umc_status_uecc) != 0;
}

// One query over every active channel. Counts follow Linux's ce_count/ue_count
// (one per channel status); the rest says how complete the records are.
struct UmcQuery final {
    uint32_t channels = 0;       // channel statuses read
    uint32_t correctable = 0;    // ce_count: Val and CECC
    uint32_t uncorrectable = 0;  // ue_count: Val and Deferred/UECC/PCC/UC/TCC
    uint32_t deferred = 0;       // of those, Deferred (poison created)
    uint32_t decoded = 0;        // of those, converted (Val, AddrV, UECC)
    uint32_t undecoded = 0;      // of those, no address or no column-bit mapping
    uint32_t overflowed = 0;     // of those, Overflow: an earlier record was lost
    uint32_t records = 0;        // filled UmcErrorRecord entries
    uint32_t dropped = 0;        // converted pages beyond the caller's capacity
    bool access_failed = false;  // an SMN access failed; later channels unread
};

enum class UmcStatus { ready, unavailable, invalid_argument };

// Device-owned, trivially destructible. initialize: any time before use.
// initialize_counters and query: PASSIVE_LEVEL, one caller at a time, under the
// device serialization that owns the SMN index/data pair (no ISR/DPC uses it).
class Umc810 final {
public:
    Umc810() noexcept = default;

    UmcStatus initialize(const UmcTopology& topology, NbioRegisters registers, RegisterIo io) noexcept;
    bool ready() const noexcept { return ready_; }
    const UmcTopology& topology() const noexcept { return topology_; }

    // umc_v8_10_err_cnt_init (amdgpu_umc_ras_late_init): per channel set
    // GeccErrCntSel.GeccErrInt to 1 (APIC interrupt) and GeccErrCnt to
    // UMC_V8_10_CE_CNT_INIT. false after the first failed access.
    bool initialize_counters() noexcept;

    // amdgpu_umc_handle_bad_pages, direct error query (SMU 13.0.0 has no ECC
    // table): umc_v8_10_query_ras_error_count, umc_v8_10_clear_error_count and
    // umc_v8_10_query_ras_error_address in one pass per channel (the STATUS
    // register stays latched until cleared, so one read serves both). Every
    // nonzero status is cleared after use; records receives at most capacity
    // entries (records may be null when capacity is 0).
    UmcQuery query(UmcErrorRecord* records, uint32_t capacity) noexcept;

private:
    bool smn_address(uint32_t reg, uint32_t channel_offset, uint32_t& address) const noexcept;
    bool read64(uint32_t reg, uint32_t channel_offset, uint64_t& value) noexcept;
    bool write64(uint32_t reg, uint32_t channel_offset, uint64_t value) noexcept;
    bool read32(uint32_t reg, uint32_t channel_offset, uint32_t& value) noexcept;
    bool write32(uint32_t reg, uint32_t channel_offset, uint32_t value) noexcept;

    Nbio nbio_{};
    UmcTopology topology_{};
    bool ready_ = false;
};

} // namespace rx7900::hardware

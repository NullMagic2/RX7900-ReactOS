/*
Module: rx7900.hardware.smu13
Purpose: Implement the SMU13 C2PMSG_66/82/90 message protocol, table setup,
late-init default policy, hardware teardown and the explicit mode-1 reset.
Created: 2026-10-21
Architecture: Bounded mailbox state machine; native owner supplies MMIO/VRAM.
Copyright 2020 Advanced Micro Devices, Inc. Adapted Linux v6.12 amdgpu_smu.c,
smu_internal.h, smu_cmn.c, smu_v13_0.c/.h, smu_v13_0_0_ppt.c, smu_v13_0_0_ppsmc.h,
smu13_driver_if_v13_0_0.h, smu_v13_0_0_pptable.h, kgd_pp_interface.h,
amdgpu_irq.c, mp_13_0_0 and thm_13_0_2 headers. Notice: licenses/AMDGPU-FIRMWARE.txt.
*/
#include "smu13.hpp"

namespace rx7900::hardware {
namespace {

constexpr uint32_t c2pmsg_66 = 0x282;
constexpr uint32_t c2pmsg_82 = 0x292;
constexpr uint32_t c2pmsg_90 = 0x29a;
constexpr uint32_t mp1_public = 0x03b00000u;
constexpr uint32_t smn_mp1_firmware_flags = 0x03010024u;
constexpr uint32_t firmware_interrupts_enabled = 0x1u;
constexpr uint32_t firmware_poll_microseconds = 1000;
constexpr uint32_t maximum_response_polls = 10000000;
constexpr uint32_t maximum_firmware_polls = 10000;

constexpr uint32_t response_none = 0x00;
constexpr uint32_t response_ok = 0x01;
constexpr uint32_t response_failed = 0xff;
constexpr uint32_t response_unknown = 0xfe;
constexpr uint32_t response_prerequisite = 0xfd;
constexpr uint32_t response_busy_other = 0xfc;

SmuStatus decode(uint32_t response) noexcept {
    switch (response) {
    case response_none: return SmuStatus::timeout;
    case response_ok: return SmuStatus::completed;
    case response_failed: return SmuStatus::command_failed;
    case response_unknown: return SmuStatus::unknown_command;
    case response_prerequisite: return SmuStatus::rejected_prerequisite;
    case response_busy_other: return SmuStatus::busy_other;
    default: return SmuStatus::hung;
    }
}

constexpr uint32_t table_pptable = 0;
constexpr uint32_t table_combo_pptable = 1;
constexpr uint32_t table_smu_metrics = 5;
constexpr uint32_t table_overdrive = 8;
constexpr uint32_t powerplay_platform_caps_offset = 18;
constexpr uint32_t pcie_gen_offset = 1232;
constexpr uint32_t pcie_lane_offset = 1235;
constexpr uint32_t lclk_offset = 1238;
constexpr uint32_t gfxoff_feature_bit = 19;
constexpr uint32_t gfxoff_minimum_pmfw = 0x004e3a00;
constexpr uint32_t powerplay_shutdown_temp_offset = 29;
constexpr uint32_t platform_cap_hardware_dc = 0x4;
constexpr uint32_t acdc_feature_bit = 27;
// smu13_driver_if_v13_0_0.h FEATURE_*_BIT and PPCLK_* for the DPM tables.
constexpr uint32_t dpm_feature_bits[smu13_dpm_clock_count]{5, 1, 3, 4, 21, 21, 8};
constexpr uint32_t dpm_clock_ids[smu13_dpm_clock_count]{1, 0, 2, 3, 5, 4, 11};
constexpr uint32_t dpm_level_count_query = 0xff;
constexpr uint32_t dpm_fine_grained = 0x80000000u;
// PPTable_t SkuTable.DriverReportedClocks.GameClockAc (measured, uint16_t).
constexpr uint32_t game_clock_ac_offset = 2782;
// Measured PPTable_t SkuTable offsets (uint16_t unless noted):
// DriverReportedClocks.BaseClockAc, SocketPowerLimitAc/Dc[PPT_THROTTLER_PPT0],
// MsgLimits.Power[PPT0][POWER_SOURCE_AC], OverDriveLimitsBasicMax.FeatureCtrlMask (u32).
constexpr uint32_t base_clock_ac_offset = 2780;
constexpr uint32_t socket_power_limit_ac_offset = 16;
constexpr uint32_t socket_power_limit_dc_offset = 24;
constexpr uint32_t msg_limit_ppt0_ac_offset = 2808;
constexpr uint32_t od_basic_max_feature_ctrl_offset = 3016;
constexpr uint32_t od_feature_ppt_bit = 2;
// smu_13_0_0_powerplay_table (packed) overdrive_table.min[4] (u32), measured.
constexpr uint32_t powerplay_od_power_percent_min_offset = 527;
// SmuMetricsExternal_t SmuMetrics.PublicSerialNumberLower/Upper, measured.
constexpr uint32_t metrics_serial_lower_offset = 236;
constexpr uint32_t metrics_serial_upper_offset = 240;
// SmuMetrics_t AvgTemperature[TEMP_COUNT] (uint16_t, TEMP_EDGE 0, TEMP_HOTSPOT
// 1, TEMP_MEM 4) and ThrottlingPercentage[THROTTLER_COUNT 22] (uint8_t); PPTable_t
// SkuTable.TemperatureLimit[TEMP_COUNT] (uint16_t) and FanAbnormalTempLimitOffset
// (int16_t). Measured by compiling the pinned smu13_driver_if_v13_0_0.h.
constexpr uint32_t metrics_avg_temperature_offset = 120;
constexpr uint32_t metrics_throttling_offset = 154;
constexpr uint32_t throttler_count = 22;
constexpr uint32_t temp_edge = 0;
constexpr uint32_t temp_hotspot = 1;
constexpr uint32_t temp_mem = 4;
constexpr uint32_t sku_temperature_limit_offset = 50;
constexpr uint32_t sku_fan_abnormal_offset = 1412;
// smu_v13_0_0_clk_map: PPCLK_GFXCLK is 0 (dpm_clock_ids[SmuClock::gfxclk]).
constexpr uint32_t ppclk_gfxclk = 0;
// FEATURE_THROTTLERS_BIT backs SMU_FEATURE_PPT_BIT in smu_v13_0_0_feature_mask_map.
constexpr uint32_t throttlers_feature_bit = 24;
// smu_v13_0_0_pwr_src_map: POWER_SOURCE_AC 0, POWER_SOURCE_DC 1.
constexpr uint32_t power_source_ac = 0;
constexpr uint32_t power_source_dc = 1;
// kgd_pp_interface.h PP_SMC_POWER_PROFILE and smu_v13_0_0_workload_map bits.
constexpr uint32_t profile_fullscreen3d = 1;
constexpr uint32_t workload_fullscreen3d_bit = 1;
constexpr uint32_t workload_power_saving_bit = 2;
// smu_v13_0_0_set_power_profile_mode adds the power-saving bit for these images.
constexpr uint32_t workload_power_saving_image = 0x004e6601;
constexpr uint32_t workload_power_saving_minimum_image = 0x004e7300;
static_assert(smu13_overdrive_table_bytes <= smu13_driver_table_bytes &&
              metrics_serial_upper_offset + 4 <= smu13_metrics_table_bytes &&
              smu13_metrics_table_bytes <= smu13_driver_table_bytes,
              "late-init tables fit the driver table");
static_assert(metrics_throttling_offset + throttler_count <= smu13_metrics_table_bytes &&
              sku_fan_abnormal_offset + 2 <= smu13_pptable_bytes, "thermal fields fit their tables");
static_assert(od_basic_max_feature_ctrl_offset + 4 <= smu13_board_table_offset &&
              powerplay_od_power_percent_min_offset + 4 <= smu13_smc_pptable_offset,
              "late-init PPTable fields precede the appended BoardTable");
constexpr uint32_t firmware_header_bytes = 32;
constexpr uint32_t firmware_ucode_version_offset = 16;

// thm_13_0_2 / mp_13_0_0 offsets and masks (BASE_IDX 0).
constexpr uint32_t thm_thermal_int_ena = 0x000a;
constexpr uint32_t thm_thermal_int_ctrl = 0x000b;
constexpr uint32_t int_ctrl_dig_therm_inth = 0x000000ffu;
constexpr uint32_t int_ctrl_dig_therm_intl = 0x0000ff00u;
constexpr uint32_t int_ctrl_inth_mask = 0x01000000u;
constexpr uint32_t int_ctrl_intl_mask = 0x02000000u;
constexpr uint32_t int_ctrl_trigger_mask = 0x04000000u;
constexpr uint32_t int_ctrl_ih_hw_enable = 0x10000000u;
constexpr uint32_t int_ctrl_max_ih_credit = 0xe0000000u;
constexpr uint32_t int_ena_inth_clear = 0x8u;
constexpr uint32_t int_ena_intl_clear = 0x10u;
constexpr uint32_t int_ena_trigger_clear = 0x20u;
constexpr uint32_t mp1_smn_ih_sw_int = 0x02c2;
constexpr uint32_t mp1_smn_ih_sw_int_ctrl = 0x02c3;
constexpr uint32_t sw_int_id = 0xffu;
constexpr uint32_t sw_int_valid = 0x100u;
constexpr uint32_t sw_int_ctrl_mask = 0x1u;
// smu13_thermal_policy[0].min is -273150 mC; SMU_THERMAL_MINIMUM/MAXIMUM_ALERT_TEMP.
constexpr uint32_t thermal_alert_minimum = 0;
constexpr uint32_t thermal_alert_maximum = 255;

// Linux treats every non-OK mailbox response the same; this driver additionally
// refuses to continue after responses that leave the mailbox state unknown.
bool terminal(SmuStatus status) noexcept {
    return status == SmuStatus::timeout || status == SmuStatus::hung ||
           status == SmuStatus::mmio_failure || status == SmuStatus::unavailable ||
           status == SmuStatus::quarantined;
}

uint32_t low(uint64_t value) noexcept { return static_cast<uint32_t>(value); }
uint32_t high(uint64_t value) noexcept { return static_cast<uint32_t>(value >> 32); }

} // namespace

DiscoveryStatus validate_smu_discovery(ByteView binary, uint32_t aperture,
                                       SmuRegisters& output) noexcept {
    output = {};

    if (aperture == 0 || (aperture & 3u) != 0) {
        return DiscoveryStatus::invalid_argument;
    }

    DiscoveredIp ip;
    const auto status = discover_ip(binary, {discovery_mp1_hardware_id, 0, 13, 0, 0, 1}, ip);

    if (status != DiscoveryStatus::validated) {
        return status;
    }

    uint32_t offset = 0;

    if (!discovery_register_offset(ip.segment(0), c2pmsg_90, aperture, offset)) {
        return DiscoveryStatus::invalid_argument;
    }

    output.ip_ = ip;
    output.aperture_bytes_ = aperture;
    return DiscoveryStatus::validated;
}

DiscoveryStatus validate_thm_discovery(ByteView binary, uint32_t aperture,
                                       ThmRegisters& output) noexcept {
    output = {};

    if (aperture == 0 || (aperture & 3u) != 0) {
        return DiscoveryStatus::invalid_argument;
    }

    DiscoveredIp ip;
    const auto status = discover_ip(binary, {discovery_thm_hardware_id, 0, 13, 0, 3, 1}, ip);

    if (status != DiscoveryStatus::validated) {
        return status;
    }

    uint32_t offset = 0;

    if (!discovery_register_offset(ip.segment(0), thm_thermal_int_ctrl, aperture, offset)) {
        return DiscoveryStatus::invalid_argument;
    }

    output.ip_ = ip;
    output.aperture_bytes_ = aperture;
    return DiscoveryStatus::validated;
}

bool Smu13::read(uint32_t reg, uint32_t& value) noexcept {
    return read_ip_register(io_, registers_.segment0_dword_base(), reg, value);
}

bool Smu13::write(uint32_t reg, uint32_t value) noexcept {
    return write_ip_register(io_, registers_.segment0_dword_base(), reg, value);
}

SmuStatus Smu13::finish(SmuStatus status) noexcept {
    report_.status = status;
    return status;
}

bool Smu13::poll(uint32_t& response) noexcept {
    response = response_none;

    for (uint32_t iteration = 0; iteration < policy_.response_poll_iterations; ++iteration) {
        if (!read(c2pmsg_90, response)) {
            return false;
        }

        if (response != response_none) {
            return true;
        }

        io_.delay_microseconds(io_.context, 1);
    }

    return true;
}

SmuStatus Smu13::initialize(SmuRegisters registers, RegisterIo io, Nbio& nbio,
                            SmuPolicy policy, SmuRecovery recovery) noexcept {
    if (report_.tables_retained || reset_issued_) {
        return SmuStatus::busy;
    }

    initialized_ = false;
    report_ = {};

    if (!registers.valid()) {
        return finish(SmuStatus::unsupported_ip);
    }

    if (!register_io_valid(io) || io.aperture_bytes != registers.aperture_bytes() ||
        policy.response_poll_iterations == 0 ||
        policy.response_poll_iterations > maximum_response_polls ||
        policy.firmware_poll_iterations == 0 ||
        policy.firmware_poll_iterations > maximum_firmware_polls) {
        return finish(SmuStatus::unavailable);
    }

    registers_ = registers;
    thm_ = {};
    io_ = io;
    nbio_ = &nbio;
    policy_ = policy;
    recovery_ = recovery;
    first_message_ = true;
    hung_ = false;
    initialized_ = true;
    return finish(SmuStatus::ready);
}

SmuStatus Smu13::send(uint16_t message, uint32_t parameter, uint32_t* read_argument) noexcept {
    bool written = false;
    return transact(message, parameter, read_argument, written);
}

SmuStatus Smu13::transact(uint16_t message, uint32_t parameter, uint32_t* read_argument,
                          bool& written) noexcept {
    written = false;

    if (!initialized_) {
        return finish(SmuStatus::unavailable);
    }

    if (reset_issued_) {
        return finish(SmuStatus::quarantined);
    }

    if (hung_) {
        return finish(SmuStatus::hung);
    }

    report_.last_message = message;
    report_.last_parameter = parameter;
    uint32_t response = response_none;

    // Linux SMU_FW_INIT: the response register is not meaningful before the
    // driver's first message, so only later messages wait for the prior result.
    if (!first_message_) {
        if (!poll(response)) {
            return finish(SmuStatus::mmio_failure);
        }

        report_.last_response = response;
        const auto previous = decode(response);

        if (previous == SmuStatus::timeout || previous == SmuStatus::hung) {
            return finish(previous);
        }
    }

    first_message_ = false;
    written = true;

    if (!write(c2pmsg_90, 0) || !write(c2pmsg_82, parameter) || !write(c2pmsg_66, message)) {
        hung_ = true;
        return finish(SmuStatus::mmio_failure);
    }

    if (!poll(response)) {
        hung_ = true;
        return finish(SmuStatus::mmio_failure);
    }

    report_.last_response = response;
    const auto status = decode(response);

    if (status == SmuStatus::hung) {
        hung_ = true;
    }

    if (status == SmuStatus::completed && read_argument != nullptr &&
        !read(c2pmsg_82, *read_argument)) {
        return finish(SmuStatus::mmio_failure);
    }

    return finish(status);
}

SmuStatus Smu13::start() noexcept {
    if (!initialized_) {
        return finish(SmuStatus::unavailable);
    }

    bool running = false;

    for (uint32_t iteration = 0; iteration < policy_.firmware_poll_iterations; ++iteration) {
        if (!nbio_->read_smn(mp1_public | smn_mp1_firmware_flags, report_.firmware_flags)) {
            return finish(SmuStatus::mmio_failure);
        }

        if ((report_.firmware_flags & firmware_interrupts_enabled) != 0) {
            running = true;
            break;
        }

        io_.delay_microseconds(io_.context, firmware_poll_microseconds);
    }

    if (!running) {
        return finish(SmuStatus::firmware_not_running);
    }

    uint32_t interface = 0;
    uint32_t version = 0;
    auto status = send(smu13_message::get_driver_if_version, 0, &interface);

    if (status != SmuStatus::completed) {
        return status;
    }

    status = send(smu13_message::get_smu_version, 0, &version);

    if (status != SmuStatus::completed) {
        return status;
    }

    report_.interface_version = interface;
    report_.smu_version = version;
    report_.interface_matches = interface == smu13_driver_interface_version;
    return finish(SmuStatus::ready);
}

SmuStatus Smu13::set_tables(memory::Manager& manager, memory::Handle driver_table,
                            memory::Handle tool_table, const VramLocation& vram) noexcept {
    if (!initialized_) {
        return finish(SmuStatus::unavailable);
    }

    if (report_.tables_retained) {
        return finish(SmuStatus::busy);
    }

    const bool tool = tool_table.slot != UINT32_MAX;
    memory::AllocationInfo driver_info{};
    memory::AllocationInfo tool_info{};

    if (!retain_local_vram(manager, driver_table, vram, smu13_driver_table_bytes,
                           memory::page_bytes, true, driver_info)) {
        return finish(SmuStatus::invalid_memory);
    }

    if (tool && !retain_local_vram(manager, tool_table, vram, smu13_tool_table_bytes,
                                   memory::page_bytes, false, tool_info)) {
        manager.unretain(driver_table);
        return finish(SmuStatus::invalid_memory);
    }

    manager_ = &manager;
    driver_table_ = driver_table;
    tool_table_ = tool ? tool_table : memory::Handle{};
    report_.tables_retained = true;
    driver_cpu_ = driver_info.backing.cpu_bytes;
    report_.driver_table_mc = driver_info.gpu.virtual_address;
    report_.tool_table_mc = tool ? tool_info.gpu.virtual_address : 0;

    auto status = send(smu13_message::set_driver_dram_address_high,
                       high(report_.driver_table_mc), nullptr);

    if (status == SmuStatus::completed) {
        status = send(smu13_message::set_driver_dram_address_low,
                      low(report_.driver_table_mc), nullptr);
    }

    if (status == SmuStatus::completed && tool) {
        status = send(smu13_message::set_tools_dram_address_high,
                      high(report_.tool_table_mc), nullptr);

        if (status == SmuStatus::completed) {
            status = send(smu13_message::set_tools_dram_address_low,
                          low(report_.tool_table_mc), nullptr);
        }
    }

    return status;
}

SmuStatus Smu13::setup_power_play(bool scpm, ByteView board) noexcept {
    if (!initialized_ || driver_cpu_ == nullptr) {
        return finish(SmuStatus::unavailable);
    }

    if (!scpm && board.size() != smu13_board_table_bytes) {
        return finish(SmuStatus::invalid_memory);
    }

    // smu_cmn_get_combo_pptable; HDP6 has no read invalidate.
    auto status = send(smu13_message::transfer_table_smu_to_dram, table_combo_pptable, nullptr);

    if (status != SmuStatus::completed) {
        return status;
    }

    const ByteView combo(driver_cpu_, smu13_driver_table_bytes);

    for (uint32_t index = 0; index < smu13_pptable_bytes; ++index) {
        combo.read_u8(smu13_smc_pptable_offset + index, pptable_[index]);
    }

    combo.read_u32(powerplay_platform_caps_offset, report_.platform_caps);
    combo.read_u16(powerplay_shutdown_temp_offset, report_.software_shutdown_temp);
    combo.read_u32(powerplay_od_power_percent_min_offset, od_power_percent_min_);
    report_.pptable_loaded = true;

    if (scpm) {
        return finish(SmuStatus::completed);
    }

    for (uint32_t index = 0; index < smu13_board_table_bytes; ++index) {
        board.read_u8(index, pptable_[smu13_board_table_offset + index]);
    }

    report_.board_table_appended = true;

    // smu_cmn_write_pptable: driver table copy, HDP flush, Dram2Smu PPTABLE.
    static_assert(smu13_pptable_bytes % 4 == 0, "PPTable_t is dword sized");
    const ByteView source(pptable_, sizeof(pptable_));
    const sdma::CommandBuffer destination(driver_cpu_, smu13_driver_table_bytes);

    for (uint32_t offset = 0; offset < smu13_pptable_bytes; offset += sizeof(uint32_t)) {
        uint32_t value = 0;
        source.read_u32(offset, value);
        destination.write_u32(offset, value);
    }

    if (!nbio_->flush_hdp()) {
        return finish(SmuStatus::mmio_failure);
    }

    return send(smu13_message::transfer_table_dram_to_smu, table_pptable, nullptr);
}

SmuStatus Smu13::run_btc() noexcept {
    return send(smu13_message::run_dc_btc, 0, nullptr);
}

SmuStatus Smu13::set_allowed_features(bool scpm) noexcept {
    // With SCPM every feature is treated as allowed and the message is refused.
    uint64_t allowed = UINT64_MAX;

    if (report_.smu_version < gfxoff_minimum_pmfw) {
        allowed &= ~(1ULL << gfxoff_feature_bit);
    }

    report_.allowed_features = allowed;

    if (scpm) {
        report_.allowed_features = UINT64_MAX;
        return finish(SmuStatus::completed);
    }

    auto status = send(smu13_message::set_allowed_features_high, high(allowed), nullptr);

    if (status == SmuStatus::completed) {
        status = send(smu13_message::set_allowed_features_low, low(allowed), nullptr);
    }

    return status;
}

SmuStatus Smu13::update_pcie(SmuPcieLink link) noexcept {
    if (!report_.pptable_loaded || link.gen > 3 || link.lane == 0 || link.lane > 6) {
        return finish(SmuStatus::unavailable);
    }

    const ByteView table(pptable_, sizeof(pptable_));
    report_.pcie_levels = 0;

    // smu_v13_0_0_set_default_dpm_table: skip levels with no gen, lane or LCLK.
    for (uint32_t level = 0; level < smu13_link_levels; ++level) {
        uint8_t gen = 0;
        uint8_t lane = 0;
        uint16_t lclk = 0;
        table.read_u8(pcie_gen_offset + level, gen);
        table.read_u8(pcie_lane_offset + level, lane);
        table.read_u16(lclk_offset + level * 2, lclk);

        if (gen == 0 && lane == 0 && lclk == 0) {
            continue;
        }

        report_.pcie_gen[report_.pcie_levels] = gen > link.gen ? link.gen : gen;
        report_.pcie_lane[report_.pcie_levels] = lane > link.lane ? link.lane : lane;
        ++report_.pcie_levels;
    }

    for (uint32_t level = 0; level < report_.pcie_levels; ++level) {
        const uint32_t argument = (level << 16) |
            (static_cast<uint32_t>(report_.pcie_gen[level]) << 8) | report_.pcie_lane[level];
        const auto status = send(smu13_message::override_pcie_parameters, argument, nullptr);

        if (status != SmuStatus::completed) {
            return status;
        }
    }

    return finish(SmuStatus::completed);
}

SmuStatus Smu13::dpm_query(size_t clock, uint32_t parameter, uint32_t boot_mhz,
                           uint32_t& value) noexcept {
    // smu_v13_0_get_dpm_freq_by_index / get_fine_grained_status first ask
    // smu_cmn_clk_dpm_is_enabled, which re-reads the feature mask for every
    // clock with a DPM feature (DCEFCLK has none and is always queried).
    if (static_cast<SmuClock>(clock) != SmuClock::dcefclk) {
        bool enabled = false;
        const auto status = feature_enabled(dpm_feature_bits[clock], enabled);

        if (status != SmuStatus::completed) {
            return status;
        }

        if (!enabled) {
            // smu_v13_0_get_boot_freq_by_index.
            value = boot_mhz;
            return SmuStatus::completed;
        }
    }

    return send(smu13_message::get_dpm_freq_by_index, (dpm_clock_ids[clock] << 16) | parameter,
                &value);
}

SmuStatus Smu13::single_dpm_table(size_t clock, uint32_t boot_mhz, SmuDpmTable& table) noexcept {
    // smu_v13_0_get_dpm_level_count then get_fine_grained_status: the same
    // query, masked to 31 bits and to bit 31.
    uint32_t value = 0;
    auto status = dpm_query(clock, dpm_level_count_query, boot_mhz, value);

    if (status != SmuStatus::completed) {
        return status;
    }

    table.count = value & ~dpm_fine_grained;

    if (table.count > smu13_max_dpm_levels) {
        return finish(SmuStatus::command_failed);
    }

    status = dpm_query(clock, dpm_level_count_query, boot_mhz, value);

    if (status != SmuStatus::completed) {
        return status;
    }

    table.fine_grained = (value & dpm_fine_grained) != 0;
    table.from_firmware = true;

    for (uint32_t level = 0; level < table.count; ++level) {
        status = dpm_query(clock, level, boot_mhz, value);

        if (status != SmuStatus::completed) {
            return status;
        }

        table.levels[level] = value & ~dpm_fine_grained;

        // Linux sets max only from a last level that is not also the first.
        if (level == 0) {
            table.min = table.levels[level];
        } else if (level == table.count - 1) {
            table.max = table.levels[level];
        }
    }

    return SmuStatus::completed;
}

SmuStatus Smu13::set_default_dpm_tables(SmuBootClocks boot) noexcept {
    if (!report_.pptable_loaded) {
        return finish(SmuStatus::unavailable);
    }

    const uint32_t boot_10khz[smu13_dpm_clock_count]{boot.socclk, boot.gfxclk, boot.uclk,
        boot.fclk, boot.vclk, boot.dclk, boot.dcefclk};
    report_.dpm_tables_ready = false;

    for (size_t clock = 0; clock < smu13_dpm_clock_count; ++clock) {
        SmuDpmTable table{};
        bool enabled = false;
        // smu_cmn_feature_is_enabled re-reads the mask (High, then Low) per clock.
        auto status = feature_enabled(dpm_feature_bits[clock], enabled);

        if (status != SmuStatus::completed) {
            return status;
        }

        if (enabled) {
            status = single_dpm_table(clock, boot_10khz[clock] / 100, table);

            if (status != SmuStatus::completed) {
                return status;
            }
        } else {
            table.count = 1;
            table.levels[0] = boot_10khz[clock] / 100;
            table.min = table.levels[0];
            table.max = table.levels[0];
        }

        if (static_cast<SmuClock>(clock) == SmuClock::gfxclk && table.from_firmware &&
            table.count != 0) {
            uint16_t game_clock = 0;
            ByteView(pptable_, sizeof(pptable_)).read_u16(game_clock_ac_offset, game_clock);

            if (game_clock != 0 && table.levels[table.count - 1] > game_clock) {
                table.levels[table.count - 1] = game_clock;
                table.max = game_clock;
            }
        }

        report_.dpm[clock] = table;
    }

    report_.dpm_tables_ready = true;
    return finish(SmuStatus::completed);
}

SmuStatus Smu13::enable_thermal_alert(ThmRegisters thm) noexcept {
    if (hung_) {
        return finish(SmuStatus::hung);
    }

    if (!initialized_ || !report_.pptable_loaded || !thm.valid()) {
        return finish(SmuStatus::unavailable);
    }

    const uint32_t thm_base = thm.segment0_dword_base();
    const uint32_t alert_low = thermal_alert_minimum;
    const uint32_t alert_high = report_.software_shutdown_temp < thermal_alert_maximum ?
        report_.software_shutdown_temp : thermal_alert_maximum;
    uint32_t control = 0;

    if (!read_ip_register(io_, thm_base, thm_thermal_int_ctrl, control) || control == UINT32_MAX) {
        return finish(SmuStatus::mmio_failure);
    }

    control &= ~(int_ctrl_max_ih_credit | int_ctrl_inth_mask | int_ctrl_intl_mask |
                 int_ctrl_dig_therm_inth | int_ctrl_dig_therm_intl | int_ctrl_trigger_mask);
    control |= (5u << 29) | int_ctrl_ih_hw_enable | (alert_high & 0xffu) | ((alert_low & 0xffu) << 8);

    if (!write_ip_register(io_, thm_base, thm_thermal_int_ctrl, control) ||
        !write_ip_register(io_, thm_base, thm_thermal_int_ena,
                           int_ena_inth_clear | int_ena_intl_clear | int_ena_trigger_clear)) {
        return finish(SmuStatus::mmio_failure);
    }

    uint32_t value = 0;

    if (!read(mp1_smn_ih_sw_int, value) || value == UINT32_MAX ||
        !write(mp1_smn_ih_sw_int, (value & ~(sw_int_id | sw_int_valid)) | 0xfeu) ||
        !read(mp1_smn_ih_sw_int_ctrl, value) || value == UINT32_MAX ||
        !write(mp1_smn_ih_sw_int_ctrl, value & ~sw_int_ctrl_mask)) {
        return finish(SmuStatus::mmio_failure);
    }

    thm_ = thm;
    report_.thermal_alert_low = static_cast<uint8_t>(alert_low);
    report_.thermal_alert_high = static_cast<uint8_t>(alert_high);
    report_.thermal_alert_enabled = true;

    // smu_v13_0_process_pending_interrupt.
    if ((report_.platform_caps & platform_cap_hardware_dc) != 0 &&
        (report_.enabled_features & (1ULL << acdc_feature_bit)) != 0) {
        const auto status = send(smu13_message::allow_ih_host_interrupt, 0, nullptr);

        if (status != SmuStatus::completed) {
            return status;
        }

        report_.ih_host_interrupt_allowed = true;
    }

    return finish(SmuStatus::completed);
}

SmuStatus Smu13::enable_all_features() noexcept {
    uint32_t enabled_low = 0;
    uint32_t enabled_high = 0;
    auto status = send(smu13_message::enable_all_features, 0, nullptr);

    // smu_feature_get_enabled_mask: GetRunningSmuFeaturesHigh, then Low.
    if (status == SmuStatus::completed) {
        status = send(smu13_message::get_running_features_high, 0, &enabled_high);
    }

    if (status == SmuStatus::completed) {
        status = send(smu13_message::get_running_features_low, 0, &enabled_low);
    }

    if (status == SmuStatus::completed) {
        report_.enabled_features = (static_cast<uint64_t>(enabled_high) << 32) | enabled_low;
    }

    return status;
}

bool smu13_firmware_image_version(ByteView smc_file, uint32_t& version) noexcept {
    uint32_t declared = 0;

    if (!smc_file.contains(0, firmware_header_bytes) || !smc_file.read_u32(0, declared) ||
        declared != smc_file.size()) {
        return false;
    }

    return smc_file.read_u32(firmware_ucode_version_offset, version);
}

SmuStatus Smu13::feature_enabled(uint32_t bit, bool& enabled) noexcept {
    // smu_cmn_feature_is_enabled -> smu_cmn_get_enabled_mask: 13.0.0 maps no
    // GetEnabledSmuFeatures message, so High is read before Low on every query.
    // Linux reports a failed query as "disabled".
    enabled = false;
    uint32_t enabled_high = 0;
    uint32_t enabled_low = 0;
    auto status = send(smu13_message::get_running_features_high, 0, &enabled_high);

    if (status == SmuStatus::completed) {
        status = send(smu13_message::get_running_features_low, 0, &enabled_low);
    }

    if (status != SmuStatus::completed) {
        return status;
    }

    report_.enabled_features = (static_cast<uint64_t>(enabled_high) << 32) | enabled_low;
    enabled = report_.enabled_features == UINT64_MAX ||
              (report_.enabled_features & (1ULL << bit)) != 0;
    return status;
}

SmuStatus Smu13::notify_power_source(bool ac_power) noexcept {
    // smu_set_ac_dc: firmware switches AC/DC itself when the GPIO-driven
    // HARDWAREDC cap set dc_controlled_by_gpio (smu_v13_0_0_check_powerplay_table).
    if ((report_.platform_caps & platform_cap_hardware_dc) != 0) {
        return SmuStatus::completed;
    }

    const auto status = send(smu13_message::notify_power_source,
                             ac_power ? power_source_ac : power_source_dc, nullptr);
    report_.power_source_notified = status == SmuStatus::completed;
    return status;
}

SmuStatus Smu13::load_overdrive_table() noexcept {
    // smu_v13_0_0_set_default_od_settings -> get_overdrive_table. Linux copies
    // the boot table into od_table/user_od_table and derives the sysfs OD mask;
    // only OD sysfs and resume restore read those copies, and this driver has
    // neither, so the bytes stay in the driver table and are not retained.
    const auto status = send(smu13_message::transfer_table_smu_to_dram, table_overdrive, nullptr);
    report_.overdrive_table_loaded = status == SmuStatus::completed;
    return status;
}

void Smu13::populate_umd_state_clk() noexcept {
    const ByteView table(pptable_, sizeof(pptable_));
    uint16_t game_clock = 0;
    uint16_t base_clock = 0;
    table.read_u16(game_clock_ac_offset, game_clock);
    table.read_u16(base_clock_ac_offset, base_clock);
    const auto& gfx = report_.dpm[static_cast<size_t>(SmuClock::gfxclk)];
    const auto& mem = report_.dpm[static_cast<size_t>(SmuClock::uclk)];
    const auto& soc = report_.dpm[static_cast<size_t>(SmuClock::socclk)];
    const auto& vclk = report_.dpm[static_cast<size_t>(SmuClock::vclk)];
    const auto& dclk = report_.dpm[static_cast<size_t>(SmuClock::dclk)];
    const auto& fclk = report_.dpm[static_cast<size_t>(SmuClock::fclk)];
    auto& pstate = report_.pstate;

    pstate.gfxclk.min = gfx.min;
    pstate.gfxclk.peak = game_clock != 0 && game_clock < gfx.max ? game_clock : gfx.max;
    pstate.gfxclk.standard = base_clock != 0 && base_clock < gfx.max ? base_clock : gfx.max;
    pstate.uclk = {mem.min, mem.max, mem.max};
    pstate.socclk = {soc.min, soc.max, soc.min};
    pstate.vclk = {vclk.min, vclk.max, vclk.min};
    pstate.dclk = {dclk.min, dclk.max, dclk.min};
    pstate.fclk = {fclk.min, fclk.max, fclk.min};
}

SmuStatus Smu13::read_power_limits(bool ac_power) noexcept {
    // smu_v13_0_0_get_power_limit with od_enabled false (default ppfeaturemask
    // lacks PP_OVERDRIVE_MASK): no OD headroom above, OD minimum percentage below.
    const ByteView table(pptable_, sizeof(pptable_));
    uint16_t socket_ac = 0;
    uint16_t socket_dc = 0;
    uint16_t msg_limit = 0;
    uint32_t od_features = 0;
    table.read_u16(socket_power_limit_ac_offset, socket_ac);
    table.read_u16(socket_power_limit_dc_offset, socket_dc);
    table.read_u16(msg_limit_ppt0_ac_offset, msg_limit);
    table.read_u32(od_basic_max_feature_ctrl_offset, od_features);

    // smu_v13_0_get_current_power_limit: needs SMU_FEATURE_PPT (THROTTLERS).
    bool throttlers = false;
    auto status = feature_enabled(throttlers_feature_bit, throttlers);

    if (terminal(status)) {
        return status;
    }

    uint32_t power_limit = 0;
    bool from_firmware = false;

    if (status == SmuStatus::completed && throttlers) {
        status = send(smu13_message::get_ppt_limit,
                      (ac_power ? power_source_ac : power_source_dc) << 16, &power_limit);

        if (terminal(status)) {
            return status;
        }

        from_firmware = status == SmuStatus::completed;
    }

    if (!from_firmware) {
        power_limit = ac_power ? socket_ac : socket_dc;
    }

    report_.od_ppt_supported = (od_features & (1u << od_feature_ppt_bit)) != 0;
    const uint32_t od_percent_upper = 0;
    const uint32_t od_percent_lower = report_.od_ppt_supported ? od_power_percent_min_ : 0;
    auto& limits = report_.power_limits;
    limits.current_limit = power_limit;
    limits.default_limit = power_limit;
    limits.max_limit = static_cast<uint32_t>(msg_limit) * (100u + od_percent_upper) / 100u;
    // Linux evaluates this in uint32_t; the modular arithmetic is kept exactly.
    limits.min_limit = power_limit * (100u - od_percent_lower) / 100u;
    limits.from_firmware = from_firmware;
    return SmuStatus::completed;
}

SmuStatus Smu13::read_unique_id() noexcept {
    // smu_v13_0_0_get_unique_id -> smu_cmn_get_metrics_table: metrics_time is
    // still zero, so the table is transferred; failures leave unique_id 0.
    const auto status = send(smu13_message::transfer_table_smu_to_dram, table_smu_metrics, nullptr);

    if (status != SmuStatus::completed) {
        return status;
    }

    const ByteView table(driver_cpu_, smu13_driver_table_bytes);
    uint32_t lower = 0;
    uint32_t upper = 0;
    table.read_u32(metrics_serial_lower_offset, lower);
    table.read_u32(metrics_serial_upper_offset, upper);
    report_.unique_id = (static_cast<uint64_t>(upper) << 32) | lower;
    return status;
}

SmuStatus Smu13::apply_default_profile(uint32_t firmware_image_version) noexcept {
    // smu_handle_task(COMPLETE_INIT) -> smu_adjust_power_state_dynamic(level,
    // skip_display, init): level equals the AUTO dpm_level from smu_sw_init, so
    // set_performance_level (and every soft/hard frequency limit) is skipped.
    // workload_mask starts at 1 << priority(FULLSCREEN3D) = 2, fls gives index 1,
    // workload_setting[1] = FULLSCREEN3D, and init forces the profile update.
    // FULLSCREEN3D is not CUSTOM, so no ACTIVITY_MONITOR_COEFF table moves.
    uint32_t mask = 1u << workload_fullscreen3d_bit;

    if (firmware_image_version == workload_power_saving_image ||
        firmware_image_version >= workload_power_saving_minimum_image) {
        mask |= 1u << workload_power_saving_bit;
    }

    report_.power_profile_mode = profile_fullscreen3d;
    report_.workload_mask = mask;
    const auto status = send(smu13_message::set_workload_mask, mask, nullptr);
    report_.workload_applied = status == SmuStatus::completed;
    return status;
}

SmuStatus Smu13::late_init(SmuLateInit input) noexcept {
    if (!initialized_ || reset_issued_ || driver_cpu_ == nullptr || !report_.pptable_loaded ||
        !report_.dpm_tables_ready || report_.late_initialized || report_.hardware_finished) {
        return finish(SmuStatus::unavailable);
    }

    if (hung_) {
        return finish(SmuStatus::hung);
    }

    // set_fine_grain_gfx_freq_parameters and post_init have no 13.0.0 hook.
    // smu_set_ac_dc logs and ignores a rejected NotifyPowerSource.
    auto status = notify_power_source(input.ac_power);

    if (terminal(status)) {
        return status;
    }

    // MP1 13.0.0 is neither 13.0.1 nor 13.0.3, and bare metal is not SR-IOV.
    status = load_overdrive_table();

    if (status != SmuStatus::completed) {
        return status;
    }

    populate_umd_state_clk();
    status = read_power_limits(input.ac_power);

    if (status != SmuStatus::completed) {
        return status;
    }

    status = read_unique_id();

    if (terminal(status)) {
        return status;
    }

    // get_fan_parameters has no 13.0.0 hook. smu_late_init ignores the
    // smu_handle_task result.
    status = apply_default_profile(input.firmware_image_version);

    if (terminal(status)) {
        return status;
    }

    // smu_apply_default_config_table_settings: no 13.0.0 hook (-EOPNOTSUPP is
    // accepted); smu_restore_dpm_user_profile returns when not resuming.
    report_.late_initialized = true;
    return finish(SmuStatus::completed);
}

SmuStatus Smu13::read_thermal_metrics(SmuThermalMetrics& output) noexcept {
    output = {};

    if (driver_cpu_ == nullptr) {
        return finish(SmuStatus::unavailable);
    }

    // smu_cmn_get_metrics_table(bypass_cache false) refreshes the cached table
    // at most every 1 ms; every action call here is far apart, so it transfers.
    const auto status = send(smu13_message::transfer_table_smu_to_dram, table_smu_metrics, nullptr);

    if (status != SmuStatus::completed) {
        return status;
    }

    const ByteView table(driver_cpu_, smu13_driver_table_bytes);
    uint16_t edge = 0;
    uint16_t hotspot = 0;
    uint16_t memory = 0;

    if (!table.read_u16(metrics_avg_temperature_offset + temp_edge * 2, edge) ||
        !table.read_u16(metrics_avg_temperature_offset + temp_hotspot * 2, hotspot) ||
        !table.read_u16(metrics_avg_temperature_offset + temp_mem * 2, memory)) {
        return finish(SmuStatus::invalid_memory);
    }

    output.edge_c = edge;
    output.hotspot_c = hotspot;
    output.memory_c = memory;

    for (uint32_t index = 0; index < throttler_count; ++index) {
        uint8_t percentage = 0;

        if (!table.read_u8(metrics_throttling_offset + index, percentage)) {
            return finish(SmuStatus::invalid_memory);
        }

        output.throttler_status |= percentage != 0 ? 1u << index : 0;
    }

    return status;
}

SmuThermalLimits Smu13::thermal_limits() const noexcept {
    SmuThermalLimits limits{};

    if (!report_.pptable_loaded) {
        return limits;
    }

    const ByteView pptable(pptable_, sizeof(pptable_));
    uint16_t hotspot = 0;
    uint16_t offset = 0;
    pptable.read_u16(sku_temperature_limit_offset + temp_hotspot * 2, hotspot);
    pptable.read_u16(sku_fan_abnormal_offset, offset);
    limits.software_shutdown_c = report_.software_shutdown_temp;
    limits.software_shutdown_offset_c = static_cast<int16_t>(offset);
    limits.hotspot_limit_c = hotspot;
    return limits;
}

SmuStatus Smu13::limit_gfxclk(bool limit) noexcept {
    const SmuDpmTable& gfxclk = report_.dpm[static_cast<size_t>(SmuClock::gfxclk)];

    if (!report_.dpm_tables_ready || gfxclk.count == 0 || gfxclk.max == 0 || gfxclk.max > 0xffffu) {
        return finish(SmuStatus::unavailable);
    }

    const uint32_t mhz = limit ? gfxclk.min : gfxclk.max;
    const auto status = send(smu13_message::set_soft_max_by_freq, (ppclk_gfxclk << 16) | (mhz & 0xffffu),
                             nullptr);

    if (status == SmuStatus::completed) {
        report_.gfxclk_soft_max_mhz = mhz;
        report_.gfxclk_limited = limit;
    }

    return status;
}

SmuStatus Smu13::set_thermal_alert_high(uint32_t celsius) noexcept {
    if (hung_) {
        return finish(SmuStatus::hung);
    }

    if (!initialized_ || reset_issued_ || !report_.thermal_alert_enabled || !thm_.valid()) {
        return finish(SmuStatus::unavailable);
    }

    const uint32_t high = celsius < thermal_alert_maximum ? celsius : thermal_alert_maximum;
    const uint32_t thm_base = thm_.segment0_dword_base();
    uint32_t control = 0;

    if (!read_ip_register(io_, thm_base, thm_thermal_int_ctrl, control) || control == UINT32_MAX ||
        !write_ip_register(io_, thm_base, thm_thermal_int_ctrl,
                           (control & ~(int_ctrl_dig_therm_inth | int_ctrl_trigger_mask)) | (high & 0xffu))) {
        return finish(SmuStatus::mmio_failure);
    }

    report_.thermal_alert_high = static_cast<uint8_t>(high);
    return finish(SmuStatus::completed);
}

SmuStatus Smu13::acknowledge_ac_dc() noexcept {
    const auto status = send(smu13_message::reenable_ac_dc_interrupt, 0, nullptr);

    if (status == SmuStatus::completed) {
        ++report_.ac_dc_acknowledged;
    }

    return status;
}

SmuStatus Smu13::hw_fini() noexcept {
    if (!initialized_ || report_.hardware_finished) {
        return finish(SmuStatus::unavailable);
    }

    // Linux cleans up before any reset; after one, THM/MP1 state is unknown.
    if (reset_issued_) {
        return finish(SmuStatus::quarantined);
    }

    // amdgpu_irq_put WARNs and returns -EINVAL for a never-enabled source, so
    // smu_smc_hw_cleanup fails before smu_disable_dpms.
    if (!report_.thermal_alert_enabled || !thm_.valid()) {
        return finish(SmuStatus::unavailable);
    }

    // smu_v13_0_set_irq_state(AMDGPU_IRQ_STATE_DISABLE).
    const uint32_t thm_base = thm_.segment0_dword_base();
    uint32_t control = 0;

    if (!read_ip_register(io_, thm_base, thm_thermal_int_ctrl, control) || control == UINT32_MAX ||
        !write_ip_register(io_, thm_base, thm_thermal_int_ctrl,
                           control | int_ctrl_inth_mask | int_ctrl_intl_mask) ||
        !write_ip_register(io_, thm_base, thm_thermal_int_ena, 0)) {
        return finish(SmuStatus::mmio_failure);
    }

    uint32_t value = 0;

    if (!read(mp1_smn_ih_sw_int_ctrl, value) || value == UINT32_MAX ||
        !write(mp1_smn_ih_sw_int_ctrl, value | sw_int_ctrl_mask)) {
        return finish(SmuStatus::mmio_failure);
    }

    report_.thermal_alert_enabled = false;
    // smu_disable_dpms: "For SMU 13.0.0 and 13.0.7, PMFW will handle the DPM
    // features ... properly on suspend/reset/unload" - no DisableAllSmuFeatures.
    report_.hardware_finished = true;
    return finish(SmuStatus::completed);
}

SmuStatus Smu13::mode1_reset() noexcept {
    if (!initialized_ || reset_issued_ || report_.smu_version == 0) {
        return finish(SmuStatus::unavailable);
    }

    // smu_v13_0_0_is_mode1_reset_supported uses the cached GetSmuVersion value.
    if (report_.smu_version < smu13_mode1_reset_minimum_pmfw) {
        return finish(SmuStatus::unsupported_firmware);
    }

    // smu_v13_0_0_set_mode1_reset_param: the 1 << 16 RAS fatal-error flag needs
    // amdgpu_ras_get_fed_status, which is never set without RAS.
    bool written = false;
    const auto status = transact(smu13_message::mode1_reset, 0, nullptr, written);

    if (written) {
        reset_issued_ = true;
        report_.mode1_reset_issued = true;
    }

    if (status != SmuStatus::completed) {
        return status;
    }

    io_.delay_microseconds(io_.context, smu13_mode1_reset_wait_microseconds);
    return finish(SmuStatus::completed);
}

SmuStatus Smu13::release_after_reset() noexcept {
    if (recovery_.confirm_reset_quiescence == nullptr ||
        !recovery_.confirm_reset_quiescence(recovery_.context)) {
        return finish(SmuStatus::quarantined);
    }

    if (report_.tables_retained && manager_ != nullptr) {
        manager_->unretain(driver_table_);

        if (tool_table_.slot != UINT32_MAX) {
            manager_->unretain(tool_table_);
        }
    }

    manager_ = nullptr;
    driver_cpu_ = nullptr;
    reset_issued_ = false;
    driver_table_ = {};
    tool_table_ = {};
    report_.tables_retained = false;
    report_.driver_table_mc = 0;
    report_.tool_table_mc = 0;
    initialized_ = false;
    hung_ = false;
    return finish(SmuStatus::released);
}

} // namespace rx7900::hardware

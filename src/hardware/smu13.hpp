/*
Module: rx7900.hardware.smu13
Purpose: Exchange SMU13.0.0 (MP1) mailbox messages, run the Linux hw_init,
late_init and hw_fini SMU steps and expose the explicit mode-1 reset message.
Created: 2026-10-21
Architecture: Discovery-validated MP1/THM tokens, Nbio SMN reads and Manager
leases; only the Linux default policy (auto DPM, default profiles) is applied.
*/
#pragma once

#include "nbio.hpp"
#include "atom_firmware.hpp"

namespace rx7900::hardware {

constexpr uint32_t smu13_driver_interface_version = 0x3d;
// Largest smu_v13_0_0 transfer table (COMBO_PPTABLE 0x4000), excluding the
// separate PMSTATUSLOG tool table (SMU13_TOOL_SIZE 0x19000).
constexpr uint32_t smu13_driver_table_bytes = 0x4000;
constexpr uint32_t smu13_tool_table_bytes = 0x19000;
// Layout measured by compiling the pinned smu13_driver_if_v13_0_0.h and
// smu_v13_0_0_pptable.h: smu_13_0_0_powerplay_table is 4804 bytes with PPTable_t
// (3972 bytes) at 832; BoardTable at 3484; SkuTable PcieGenSpeed/PcieLaneCount/
// LclkFreq at 1232/1235/1238.
constexpr uint32_t smu13_pptable_bytes = 3972;
constexpr uint32_t smu13_powerplay_bytes = 4804;
constexpr uint32_t smu13_smc_pptable_offset = 832;
constexpr uint32_t smu13_board_table_offset = 3484;
constexpr uint32_t smu13_link_levels = 3;
// smu_v13_0.h MAX_DPM_LEVELS.
constexpr uint32_t smu13_max_dpm_levels = 16;

namespace smu13_message {
constexpr uint16_t test = 0x1;
constexpr uint16_t get_smu_version = 0x2;
constexpr uint16_t get_driver_if_version = 0x3;
constexpr uint16_t enable_all_features = 0x6;
constexpr uint16_t get_running_features_low = 0xc;
constexpr uint16_t get_running_features_high = 0xd;
constexpr uint16_t set_driver_dram_address_high = 0xe;
constexpr uint16_t set_driver_dram_address_low = 0xf;
constexpr uint16_t set_tools_dram_address_high = 0x10;
constexpr uint16_t set_tools_dram_address_low = 0x11;
constexpr uint16_t set_allowed_features_low = 0x4;
constexpr uint16_t set_allowed_features_high = 0x5;
constexpr uint16_t transfer_table_smu_to_dram = 0x12;
constexpr uint16_t transfer_table_dram_to_smu = 0x13;
constexpr uint16_t override_pcie_parameters = 0x20;
constexpr uint16_t run_dc_btc = 0x36;
constexpr uint16_t get_dpm_freq_by_index = 0x1f;
constexpr uint16_t allow_ih_host_interrupt = 0x4c;
// smu_v13_0_0_ppsmc.h ids of messages other modules may send through Smu13::send
// (clock/power gating: smu_v13_0_gfx_off_control, set_vcn/jpeg_enable,
// set_df_cstate, gpo_control) and of the late-init/reset messages used here.
constexpr uint16_t disable_all_features = 0x7;
constexpr uint16_t set_soft_min_by_freq = 0x19;
constexpr uint16_t set_soft_max_by_freq = 0x1a;
constexpr uint16_t set_hard_min_by_freq = 0x1b;
constexpr uint16_t set_hard_max_by_freq = 0x1c;
constexpr uint16_t get_min_dpm_freq = 0x1d;
constexpr uint16_t get_max_dpm_freq = 0x1e;
constexpr uint16_t set_workload_mask = 0x24;
constexpr uint16_t get_dc_mode_max_dpm_freq = 0x27;
constexpr uint16_t allow_gfx_off = 0x28;
constexpr uint16_t disallow_gfx_off = 0x29;
// Parameter: instance << 16 (PowerUp/DownVcn); 0 for JPEG.
constexpr uint16_t power_up_vcn = 0x2a;
constexpr uint16_t power_down_vcn = 0x2b;
constexpr uint16_t power_up_jpeg = 0x2c;
constexpr uint16_t power_down_jpeg = 0x2d;
constexpr uint16_t prepare_mp1_for_unload = 0x2e;
constexpr uint16_t mode1_reset = 0x2f;
constexpr uint16_t set_ppt_limit = 0x32;
constexpr uint16_t get_ppt_limit = 0x33;
constexpr uint16_t reenable_ac_dc_interrupt = 0x34;
constexpr uint16_t notify_power_source = 0x35;
// SMU_MSG_DFCstateControl (PPSMC_MSG_SetExternalClientDfCstateAllow).
constexpr uint16_t df_cstate_control = 0x3b;
constexpr uint16_t set_mgpu_fan_boost_limit_rpm = 0x3c;
// SMU_MSG_AllowGpo (PPSMC_MSG_SetGpoAllow).
constexpr uint16_t allow_gpo = 0x41;
constexpr uint16_t dal_not_present = 0x4e;
constexpr uint16_t enable_uclk_shadow = 0x51;
} // namespace smu13_message

// Measured from the pinned headers: OverDriveTableExternal_t 140 bytes,
// SmuMetricsExternal_t 392 bytes (both inside the 0x4000 driver table).
constexpr uint32_t smu13_overdrive_table_bytes = 140;
constexpr uint32_t smu13_metrics_table_bytes = 392;
// smu_v13_0_0_is_mode1_reset_supported: PMFW 78.41 and later.
constexpr uint32_t smu13_mode1_reset_minimum_pmfw = 0x004e2900;
// SMU13_MODE1_RESET_WAIT_TIME_IN_MS after a completed Mode1Reset message.
constexpr uint32_t smu13_mode1_reset_wait_microseconds = 500000;

// Negotiated PCIe capability in SMU encoding: gen 0..3 is Gen1..Gen4 and lane
// 1..6 is x1, x2, x4, x8, x12, x16 (smu_smc_hw_setup).
struct SmuPcieLink final {
    uint8_t gen;
    uint8_t lane;
};

// smu_v13_0_0_set_default_dpm_table order (the PCIe table is update_pcie's).
enum class SmuClock : uint8_t { socclk, gfxclk, uclk, fclk, vclk, dclk, dcefclk, count };
constexpr size_t smu13_dpm_clock_count = static_cast<size_t>(SmuClock::count);

// smu_table.boot_values in 10 kHz units: firmwareinfo sclk/mclk and smu_info
// socclk/dcefclk/vclk/dclk/fclk (AtomFirmwareInfo).
struct SmuBootClocks final {
    uint32_t socclk;
    uint32_t gfxclk;
    uint32_t uclk;
    uint32_t fclk;
    uint32_t vclk;
    uint32_t dclk;
    uint32_t dcefclk;
};

// smu_13_0_dpm_table in MHz.
struct SmuDpmTable final {
    uint32_t count = 0;
    bool fine_grained = false;
    // True when PMFW reported the levels; false for the one boot-value level.
    bool from_firmware = false;
    uint32_t levels[smu13_max_dpm_levels]{};
    uint32_t min = 0;
    uint32_t max = 0;
};

// adev->pm.fw_version for SMU13.0.0: common_firmware_header.ucode_version of
// the PSP-loaded smu_13_0_0.bin (smu_v13_0_init_microcode), not the PMFW-reported
// version. Fails unless the file holds a 32-byte header whose size field matches.
bool smu13_firmware_image_version(ByteView smc_file, uint32_t& version) noexcept;

// smu_late_init inputs that Linux takes from the host and module defaults.
struct SmuLateInit final {
    // adev->pm.ac_power = power_supply_is_system_supplied() > 0; a desktop with
    // no system power-supply class device reports AC.
    bool ac_power;
    // smu13_firmware_image_version of the loaded SMC image (adev->pm.fw_version).
    uint32_t firmware_image_version;
};

// smu_umd_pstate_table entry in MHz.
struct SmuPstateClock final {
    uint32_t min = 0;
    uint32_t peak = 0;
    uint32_t standard = 0;
};

// smu_v13_0_0_populate_umd_state_clk.
struct SmuUmdPstate final {
    SmuPstateClock gfxclk{};
    SmuPstateClock uclk{};
    SmuPstateClock socclk{};
    SmuPstateClock vclk{};
    SmuPstateClock dclk{};
    SmuPstateClock fclk{};
};

// smu_v13_0_0_get_power_limit, in Watts.
struct SmuPowerLimits final {
    uint32_t current_limit = 0;
    uint32_t default_limit = 0;
    uint32_t max_limit = 0;
    uint32_t min_limit = 0;
    // True when GetPptLimit answered; false when the SocketPowerLimit fallback applied.
    bool from_firmware = false;
};

// smu_v13_0_0_get_smu_metrics_data temperatures in degrees C (SmuMetrics_t
// AvgTemperature[TEMP_EDGE, TEMP_HOTSPOT, TEMP_MEM]) and
// smu_v13_0_get_throttler_status: bit n set when ThrottlingPercentage[n] (n in
// THROTTLER_*_BIT order, 22 entries) is non-zero.
struct SmuThermalMetrics final {
    uint32_t edge_c = 0;
    uint32_t hotspot_c = 0;
    uint32_t memory_c = 0;
    uint32_t throttler_status = 0;
};

// The thermal-action subset of SmuReport (small enough for any kernel frame).
struct SmuThermalState final {
    bool alert_enabled = false;
    uint8_t alert_high_c = 0;
    bool gfxclk_limited = false;
    uint32_t gfxclk_soft_max_mhz = 0;
};

// smu_v13_0_0_get_thermal_temperature_range subset in degrees C:
// powerplay software_shutdown_temp, SkuTable.FanAbnormalTempLimitOffset (the
// signed offset FAN_ABNORMAL applies to it) and TemperatureLimit[TEMP_HOTSPOT].
struct SmuThermalLimits final {
    uint32_t software_shutdown_c = 0;
    int32_t software_shutdown_offset_c = 0;
    uint32_t hotspot_limit_c = 0;
};

enum class SmuStatus {
    ready,
    completed,
    released,
    unavailable,
    unsupported_ip,
    firmware_not_running,
    mmio_failure,
    timeout,
    command_failed,
    unknown_command,
    rejected_prerequisite,
    busy_other,
    hung,
    invalid_memory,
    busy,
    quarantined,
    // PMFW too old for the requested operation (mode-1 reset before 78.41).
    unsupported_firmware
};

// Only validate_smu_discovery creates a valid token: unique MP1 instance0 13.0.0
// whose C2PMSG_66/82/90 mailbox fits the assigned BAR5 aperture.
class SmuRegisters final {
public:
    SmuRegisters() noexcept = default;
    bool valid() const noexcept { return ip_.valid() && aperture_bytes_ != 0; }
    uint32_t segment0_dword_base() const noexcept { return ip_.segment(0); }
    uint32_t aperture_bytes() const noexcept { return aperture_bytes_; }

private:
    friend DiscoveryStatus validate_smu_discovery(ByteView, uint32_t, SmuRegisters&) noexcept;
    DiscoveredIp ip_{};
    uint32_t aperture_bytes_ = 0;
};

DiscoveryStatus validate_smu_discovery(ByteView binary, uint32_t register_aperture_bytes,
                                       SmuRegisters& output) noexcept;

// Only validate_thm_discovery creates a valid token: unique THM instance0 13.0.3
// (the Navi31 board record; Linux uses thm_13_0_2 offsets for every SMU13) whose
// THM_THERMAL_INT_CTRL fits the assigned BAR5 aperture.
class ThmRegisters final {
public:
    ThmRegisters() noexcept = default;
    bool valid() const noexcept { return ip_.valid() && aperture_bytes_ != 0; }
    uint32_t segment0_dword_base() const noexcept { return ip_.segment(0); }

private:
    friend DiscoveryStatus validate_thm_discovery(ByteView, uint32_t, ThmRegisters&) noexcept;
    DiscoveredIp ip_{};
    uint32_t aperture_bytes_ = 0;
};

DiscoveryStatus validate_thm_discovery(ByteView binary, uint32_t register_aperture_bytes,
                                       ThmRegisters& output) noexcept;

struct SmuPolicy final {
    // Linux __smu_cmn_poll_stat: usec_timeout * 20 polls with 1 us delays.
    uint32_t response_poll_iterations;
    // Linux checks MP1 FIRMWARE_FLAGS once after PSP loads PMFW; extra 1 ms
    // polls are an explicit native policy, not a Linux retry.
    uint32_t firmware_poll_iterations;
};

struct SmuRecovery final {
    void* context;
    // True only after reset proves MP1 no longer accesses the table memory.
    bool (*confirm_reset_quiescence)(void*) noexcept;
};

struct SmuReport final {
    SmuStatus status = SmuStatus::unavailable;
    uint16_t last_message = 0;
    uint32_t last_parameter = 0;
    uint32_t last_response = 0;
    uint32_t firmware_flags = 0;
    uint32_t smu_version = 0;
    uint32_t interface_version = 0;
    bool interface_matches = false;
    uint64_t enabled_features = 0;
    uint64_t driver_table_mc = 0;
    uint64_t tool_table_mc = 0;
    bool tables_retained = false;
    bool pptable_loaded = false;
    bool board_table_appended = false;
    uint32_t platform_caps = 0;
    uint64_t allowed_features = 0;
    uint32_t pcie_levels = 0;
    uint8_t pcie_gen[smu13_link_levels]{};
    uint8_t pcie_lane[smu13_link_levels]{};
    SmuDpmTable dpm[smu13_dpm_clock_count]{};
    bool dpm_tables_ready = false;
    // smu_13_0_0_powerplay_table.software_shutdown_temp (degrees C).
    uint16_t software_shutdown_temp = 0;
    uint8_t thermal_alert_low = 0;
    uint8_t thermal_alert_high = 0;
    bool thermal_alert_enabled = false;
    // HARDWAREDC platform cap with FEATURE_ACDC running: AllowIHHostInterrupt sent.
    bool ih_host_interrupt_allowed = false;

    // late_init results.
    bool late_initialized = false;
    // NotifyPowerSource sent (false when HARDWAREDC lets PMFW switch AC/DC by GPIO).
    bool power_source_notified = false;
    // Boot OverDriveTableExternal_t read with TransferTableSmu2Dram OVERDRIVE.
    bool overdrive_table_loaded = false;
    // SkuTable.OverDriveLimitsBasicMax.FeatureCtrlMask has PP_OD_FEATURE_PPT_BIT.
    bool od_ppt_supported = false;
    SmuUmdPstate pstate{};
    SmuPowerLimits power_limits{};
    // SmuMetrics PublicSerialNumberUpper:Lower (adev->unique_id); 0 if unread.
    uint64_t unique_id = 0;
    // PP_SMC_POWER_PROFILE selected by smu_adjust_power_state_dynamic and the
    // SetWorkloadMask argument sent; workload_applied when PMFW accepted it.
    uint32_t power_profile_mode = 0;
    uint32_t workload_mask = 0;
    bool workload_applied = false;
    // hw_fini (smu_smc_hw_cleanup) ran; adev->pm.dpm_enabled is false again.
    bool hardware_finished = false;
    // Mode1Reset was written to the mailbox; messages stay refused until
    // release_after_reset proves reset quiescence.
    bool mode1_reset_issued = false;
    // Interrupt actions (gpu_health): the GFXCLK soft maximum last sent
    // (0: never limited, PMFW default), whether it is the thermal limit, and
    // the AC/DC acknowledgements sent.
    uint32_t gfxclk_soft_max_mhz = 0;
    bool gfxclk_limited = false;
    uint32_t ac_dc_acknowledged = 0;
};

// Serialized PASSIVE_LEVEL use by the exclusive device owner after PSP loaded
// PMFW. Nbio, Manager and mapped registers outlive this object. A response that
// is not a documented mailbox code marks the firmware hung (Linux SMU_FW_HANG)
// and refuses every later message until re-initialization after reset.
class Smu13 final {
public:
    Smu13() noexcept = default;
    Smu13(const Smu13&) = delete;
    Smu13& operator=(const Smu13&) = delete;

    SmuStatus initialize(SmuRegisters registers, RegisterIo io, Nbio& nbio,
                         SmuPolicy policy, SmuRecovery recovery) noexcept;

    // smu_v13_0_check_fw_status then smu_v13_0_check_fw_version. An interface
    // version mismatch is reported, not fatal, exactly as Linux.
    SmuStatus start() noexcept;

    // smu_cmn_send_smc_msg_with_param. read_argument may be null.
    SmuStatus send(uint16_t message, uint32_t parameter, uint32_t* read_argument) noexcept;

    // smu_v13_0_set_driver_table_location and set_tool_table_location. Both are
    // local-VRAM page-aligned allocations retained until confirmed reset; the
    // driver table must be CPU-mapped (uncached) for table transfers. A null
    // tool handle (slot UINT32_MAX) skips the tool table as Linux does when absent.
    SmuStatus set_tables(memory::Manager& manager, memory::Handle driver_table,
                         memory::Handle tool_table, const VramLocation& vram) noexcept;

    // smu_v13_0_0_setup_pptable: TransferTableSmu2Dram COMBO_PPTABLE, keep the
    // embedded PPTable_t; without SCPM append the ATOM BoardTable (board must be
    // exactly smu13_board_table_bytes) and write it with TransferTableDram2Smu
    // PPTABLE after an HDP flush. With SCPM, PSP owns the table and board is unused.
    SmuStatus setup_power_play(bool scpm, ByteView board) noexcept;
    // smu_v13_0_run_btc: RunDcBtc.
    SmuStatus run_btc() noexcept;
    // smu_v13_0_0_get_allowed_feature_mask with the default ppfeaturemask
    // 0xfff7bfff and ATHUB/MMHUB power gating: every feature, minus GFXOFF on PMFW
    // older than 78.58. Sent high word first. Not permitted (skipped) under SCPM.
    SmuStatus set_allowed_features(bool scpm) noexcept;
    // EnableAllSmuFeatures and the running-feature readback.
    SmuStatus enable_all_features() noexcept;
    // smu_v13_0_0_set_default_dpm_table: for every clock whose DPM feature runs,
    // smu_v13_0_set_single_dpm_table (GetDpmFreqByIndex level count, fine-grain
    // bit, each level); otherwise one boot-value level. GFXCLK's top level is
    // capped at SkuTable DriverReportedClocks.GameClockAc. A level count above
    // smu13_max_dpm_levels (Linux would overrun dpm_levels) fails closed.
    // Requires setup_power_play and enable_all_features.
    SmuStatus set_default_dpm_tables(SmuBootClocks boot) noexcept;
    // smu_v13_0_update_pcie_parameters with PP_PCIE_DPM enabled: clamp every
    // PPTable link level to link and send OverridePcieParameters per level.
    SmuStatus update_pcie(SmuPcieLink link) noexcept;
    // smu_get_thermal_temperature_range then smu_v13_0_enable_thermal_alert:
    // set_irq_state(ENABLE) programs THM DIG_THERM_INTL/INTH = [0, min(255,
    // software_shutdown_temp)] and unmasks the THM and MP1 SW interrupts, then
    // process_pending_interrupt. Requires setup_power_play and enable_all_features.
    // The THM token is retained for hw_fini's thermal alert disable.
    SmuStatus enable_thermal_alert(ThmRegisters thm) noexcept;

    // smu_late_init for bare-metal 13.0.0 with default module parameters, in
    // Linux order: set_ac_dc (NotifyPowerSource unless HARDWAREDC), set_default_od
    // settings (TransferTableSmu2Dram OVERDRIVE), populate_umd_state_clk,
    // get_power_limit (running-feature query, GetPptLimit when THROTTLERS runs),
    // get_unique_id (TransferTableSmu2Dram SMU_METRICS), then COMPLETE_INIT:
    // auto DPM level is unchanged, so only the FULLSCREEN3D SetWorkloadMask is
    // sent. Requires set_default_dpm_tables (Linux pm.dpm_enabled) and runs once.
    // Mirrors Linux error handling: an OD table failure fails, other rejected
    // messages are recorded and skipped; timeouts, hangs and MMIO failures always
    // fail closed. Integrators call it after the soc21/GMC/IH/PSP late steps and
    // before GFX/SDMA/MES late steps and any clock/power gating.
    SmuStatus late_init(SmuLateInit input) noexcept;

    // smu_hw_fini / smu_smc_hw_cleanup for bare-metal 13.0.0: VCN/JPEG/VPE/UMSCH
    // are not ungated by this driver, so no power-down message is due;
    // smu_v13_0_disable_thermal_alert (amdgpu_irq_put -> set_irq_state DISABLE)
    // masks THM INTH/INTL, clears THM_THERMAL_INT_ENA and masks the MP1 SW
    // interrupt; smu_disable_dpms returns early for 13.0.0 (PMFW owns feature
    // shutdown) and smu_reset_mp1_state applies only to 13.0.10. No message is
    // sent. Fails without register writes when the alert was never enabled
    // (Linux amdgpu_irq_put -EINVAL). Tables stay retained until reset proof.
    SmuStatus hw_fini() noexcept;

    // smu_v13_0_0_mode1_reset, never called implicitly: refuses PMFW older than
    // 78.41, sends Mode1Reset with parameter 0 (no RAS fatal-error flag, as this
    // driver has no RAS) and waits 500 ms after completion. Afterwards every
    // message is refused until release_after_reset. The caller owns the rest of
    // amdgpu_device_mode1_reset (bus-master disable, PCI state, PSP/NBIO waits).
    SmuStatus mode1_reset() noexcept;

    SmuStatus release_after_reset() noexcept;
    SmuReport report() const noexcept { return report_; }

    // PASSIVE_LEVEL interrupt actions, serialized by the device owner like every
    // message here. read_thermal_metrics: TransferTableSmu2Dram SMU_METRICS
    // (smu_cmn_get_metrics_table) into the driver table, then the decoded
    // fields; needs set_tables. thermal_limits: from the loaded PPTable (zero
    // before setup_power_play).
    SmuStatus read_thermal_metrics(SmuThermalMetrics& output) noexcept;
    SmuThermalLimits thermal_limits() const noexcept;
    SmuThermalState thermal_state() const noexcept {
        return {report_.thermal_alert_enabled, report_.thermal_alert_high, report_.gfxclk_limited,
                report_.gfxclk_soft_max_mhz};
    }
    // smu_v13_0_set_soft_freq_limited_range(GFXCLK, 0, max): SetSoftMaxByFreq
    // with (PPCLK_GFXCLK << 16) | max. limit selects the lowest GFXCLK DPM
    // level, otherwise the highest (the boot policy). Requires the DPM tables;
    // only table values are ever sent.
    SmuStatus limit_gfxclk(bool limit) noexcept;
    // smu_v13_0_irq_process FAN_ABNORMAL/FAN_RECOVERY: THM_THERMAL_INT_CTRL
    // DIG_THERM_INTH = min(255, celsius) with THERM_TRIGGER_MASK cleared.
    // Requires enable_thermal_alert.
    SmuStatus set_thermal_alert_high(uint32_t celsius) noexcept;
    // smu_v13_0_ack_ac_dc_interrupt: ReenableAcDcInterrupt.
    SmuStatus acknowledge_ac_dc() noexcept;

private:
    bool read(uint32_t reg, uint32_t& value) noexcept;
    bool write(uint32_t reg, uint32_t value) noexcept;
    bool poll(uint32_t& response) noexcept;
    SmuStatus transact(uint16_t message, uint32_t parameter, uint32_t* read_argument,
                       bool& written) noexcept;
    SmuStatus dpm_query(size_t clock, uint32_t parameter, uint32_t boot_mhz, uint32_t& value) noexcept;
    SmuStatus single_dpm_table(size_t clock, uint32_t boot_mhz, SmuDpmTable& table) noexcept;
    SmuStatus feature_enabled(uint32_t bit, bool& enabled) noexcept;
    SmuStatus notify_power_source(bool ac_power) noexcept;
    SmuStatus load_overdrive_table() noexcept;
    void populate_umd_state_clk() noexcept;
    SmuStatus read_power_limits(bool ac_power) noexcept;
    SmuStatus read_unique_id() noexcept;
    SmuStatus apply_default_profile(uint32_t firmware_image_version) noexcept;
    SmuStatus finish(SmuStatus status) noexcept;

    SmuRegisters registers_{};
    RegisterIo io_{};
    Nbio* nbio_ = nullptr;
    SmuPolicy policy_{};
    SmuRecovery recovery_{};
    SmuReport report_{};
    memory::Manager* manager_ = nullptr;
    memory::Handle driver_table_{};
    memory::Handle tool_table_{};
    uint8_t* driver_cpu_ = nullptr;
    uint8_t pptable_[smu13_pptable_bytes]{};
    // smu_13_0_0_powerplay_table overdrive_table.min[ODSETTING_POWERPERCENTAGE].
    uint32_t od_power_percent_min_ = 0;
    ThmRegisters thm_{};
    bool initialized_ = false;
    bool reset_issued_ = false;
    bool first_message_ = true;
    bool hung_ = false;
};

} // namespace rx7900::hardware

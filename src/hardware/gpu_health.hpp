/*
Module: rx7900.hardware.gpu_health
Purpose: Act on the thermal and RAS interrupt sources Navi31 exposes: logged
state, a bounded SMU13 clock response to thermal alerts, quiesce and refusal on
a critical temperature trip, and page retirement, session quarantine and
escalation for RAS errors.
Created: 2026-10-24
Architecture: Two halves around one locked mailbox. consume is an IrqService
drain consumer (DISPATCH_LEVEL): it turns IrqReport sequence counters into
pending event counts and, while DF RAS is hardware-enabled, reads and clears the
NBIO ERREVENT_ATHUB status once per drain. run is the PASSIVE_LEVEL action pass
on the native owner's worker under the device serialization; it reaches the
ASIC only through Smu13, the VramAllocator, the UMC 8.10 error-record reader
(umc_v8_10, SMN) and GpuHealthPlatform callbacks (refuse, escalate,
quarantine, log).
Copyright 2019, 2021, 2022 Advanced Micro Devices, Inc.
Adapted Linux v6.12 smu_v13_0.c (smu_v13_0_irq_process, ack_ac_dc_interrupt),
amdgpu_smu.c (smu_swctf_delayed_work_handler, throttling_logging_work),
smu_v13_0_0_ppt.c (get_thermal_temperature_range, metrics temperatures,
smu_v13_0_get_throttler_status), amdgpu_smu.h, smu_cmn.h (interrupt context
ids), amdgpu.h (AMDGPU_SWCTF_EXTRA_DELAY), amdgpu_irq.c (amdgpu_irq_handler),
amdgpu_ras.c (amdgpu_ras_interrupt_fatal_error_handler, amdgpu_ras_global_ras_isr,
nbio ras selection), amdgpu_ras.h (block ids), nbio_v4_3.c
(handle_ras_err_event_athub_intr_no_bifring), nbio_4_3_0 offset/sh_mask,
gmc_v11_0.c (ecc_irq), gfx_v11_0.c (rlc_gc_fed_irq), umc_v8_10.c, amdgpu_umc.c
(amdgpu_umc_process_ecc_irq, do_page_retirement, handle_bad_pages) and
amdgpu_ras_eeprom.c (bad page threshold warning), all pinned in
work/upstream-gpuinit. Notice and provenance: licenses/AMDGPU-FIRMWARE.txt.
*/
#pragma once

#include "irq_service.hpp"
#include "umc_v8_10.hpp"
#include "vram_allocator.hpp"

namespace rx7900::hardware {

// AMDGPU_SWCTF_EXTRA_DELAY: the THM L2H check runs 50 ms after the alert.
constexpr uint32_t health_swctf_delay_us = 50000;
// adev->throttling_logging_rs: one throttling report per (60 - 1) seconds.
constexpr uint64_t health_throttle_log_interval_us = 59ULL * 1000000ULL;
// The native clock response (not in Linux, whose PMFW alone throttles): after a
// passed SW CTF check the GFXCLK cap is held, re-checked every hold period and
// lifted once the hotspot is hysteresis below the shutdown temperature; after
// the hold limit it stays applied and no more checks are scheduled.
constexpr uint32_t health_thermal_hold_us = 2000000;
constexpr uint32_t health_thermal_hysteresis_c = 5;
constexpr uint32_t health_thermal_max_holds = 30;
// SMU_THERMAL_MAXIMUM_ALERT_TEMP.
constexpr uint32_t health_thermal_alert_maximum_c = 255;
// smu_cmn.h SMU_IH_INTERRUPT_CONTEXT_ID_* consumed by smu_v13_0_irq_process.
constexpr uint32_t health_smu_context_ac = 0x3;
constexpr uint32_t health_smu_context_dc = 0x4;
constexpr uint32_t health_smu_context_throttling = 0x7;
constexpr uint32_t health_smu_context_fan_abnormal = 0x8;
constexpr uint32_t health_smu_context_fan_recovery = 0x9;
// adev->ras_hw_enabled bit positions (amdgpu_ras.h AMDGPU_RAS_BLOCK__UMC/__DF).
constexpr uint32_t health_ras_block_umc = 0;
constexpr uint32_t health_ras_block_df = 8;
// nbio_4_3_0: regBIF_BX0_BIF_DOORBELL_INT_CNTL (BASE_IDX 2) and its
// RAS_ATHUB_ERR_EVENT_INTERRUPT_STATUS/CLEAR fields.
constexpr uint32_t health_bif_doorbell_int_cntl = 0x00fe;
constexpr uint32_t health_athub_status = 0x00000004u;
constexpr uint32_t health_athub_clear = 0x00040000u;
// Bounded records: UMC error records (pages) per action pass, as Linux's
// max_ras_err_cnt_per_query; retired VRAM pages remembered (more than the
// VramAllocator's reservation capacity, so every retired page is listed).
constexpr uint32_t health_umc_record_capacity = umc_v8_10_record_capacity;
constexpr uint32_t health_retired_page_capacity = 64;
constexpr uint64_t health_vram_page_bytes = 4096;
// amdgpu_ras_validate_threshold with amdgpu_bad_page_threshold = -1 (default):
// one bad page per RAS_BAD_PAGE_COVER of VRAM; reaching it is only reported
// (amdgpu_ras_eeprom_append sets is_rma only for an explicit threshold).
constexpr uint64_t health_bad_page_cover_bytes = 100ULL * 1024 * 1024;

enum class HealthStatus { unavailable, ready, invalid_argument, busy };

enum class HealthState : uint8_t {
    running,
    // A thermal alert capped GFXCLK; the hold checks may lift it.
    thermal_limited,
    // Critical temperature (hardware or software CTF) or an unrecoverable RAS
    // error without a recovery path: no new GPU work is admitted.
    refused
};

enum class HealthReason : uint8_t {
    none,
    hardware_ctf,     // ROM_SMUIO GPIO19: Linux orderly_poweroff
    software_ctf,     // THM L2H confirmed after 50 ms: Linux orderly_poweroff
    ras_fatal,        // ERREVENT_ATHUB: Linux amdgpu_ras_reset_gpu (mode-1)
    ras_uncorrectable // UMC uncorrectable ECC / poison that page retirement and
                      // session quarantine cannot contain: mode-1 recovery
};

// One logged event; values per kind are listed with the kind.
enum class HealthEventKind : uint8_t {
    hardware_ctf,            // -
    thermal_alert,           // hotspot? 0, limit applied (1/0), INTH
    swctf_confirmed,         // hotspot C, shutdown C, read status
    swctf_cleared,           // hotspot C, shutdown C
    thermal_restored,        // hotspot C, gfxclk soft max MHz
    thermal_held,            // hotspot C, holds
    under_temperature,       // count
    throttling,              // throttler status, hotspot C, suppressed since last
    fan_abnormal,            // new INTH C, offset C
    fan_recovery,            // new INTH C
    ac_dc,                   // 1 AC / 0 DC, acknowledgement status
    smu_failure,             // SmuStatus, step
    ras_athub_fatal,         // IV count, status reads
    ras_umc_ignored,         // count (no UMC RAS support: Linux has no ras_if)
    ras_umc_correctable,     // channel records
    ras_umc_uncorrectable,   // channel records, pages retired, sessions quarantined
    ras_umc_unknown,         // count (records unavailable)
    ras_umc_page,            // VRAM offset low, high, channel index | umc << 8 | node << 16
    ras_umc_undecoded,       // records without address, overflowed records, dropped pages
    ras_umc_uncontained,     // VRAM offset low, high, HealthContainment
    ras_umc_contained,       // channel records, pages: no recovery needed
    ras_umc_counters,        // 1 armed, 0 failed (umc_v8_10_err_cnt_init)
    ras_bad_page_threshold,  // retired pages, threshold
    ras_gc_fed,              // count (GC 11.0.0: no handler in Linux)
    page_retire_failed,      // VramStatus
    refused,                 // HealthReason
    escalated                // HealthReason
};

struct HealthEvent final {
    HealthEventKind kind = HealthEventKind::refused;
    uint32_t value0 = 0;
    uint32_t value1 = 0;
    uint32_t value2 = 0;
};

// Why a retired page's corrupted contents are not contained by quarantine.
enum class HealthContainment : uint8_t {
    contained,
    driver_reserved,  // inside a driver VRAM reservation (firmware, TMR, stolen)
    driver_owned,     // in use, but by no process session (kernel allocation)
    retire_failed     // outside VRAM or reservation metadata exhausted
};

// Native services of the action pass and the mailbox lock. now_us is
// monotonic; lock/unlock exclude consume (DPC) from run (a spin lock natively,
// usable at DISPATCH_LEVEL and below). Every action callback runs at
// PASSIVE_LEVEL from run only, under the owner's device serialization:
// refuse stops admission of new GPU work for good (and is logged); escalate
// requests the destructive mode-1 recovery, false when no recovery path exists;
// quarantine_vram marks every process session owning VRAM in [vram_offset,
// vram_offset + bytes), returns how many were newly quarantined and sets owners
// to the live sessions owning that range (newly or already quarantined); log
// prints.
struct GpuHealthPlatform final {
    void* context = nullptr;
    uint64_t (*now_us)(void*) noexcept = nullptr;
    void (*lock)(void*) noexcept = nullptr;
    void (*unlock)(void*) noexcept = nullptr;
    void (*refuse)(void*, HealthReason) noexcept = nullptr;
    bool (*escalate)(void*, HealthReason) noexcept = nullptr;
    uint32_t (*quarantine_vram)(void*, uint64_t vram_offset, uint64_t bytes, uint32_t& owners) noexcept = nullptr;
    void (*log)(void*, const HealthEvent&) noexcept = nullptr;
};

// smu and vram may be null (thermal or RAS actions then only log and refuse).
// io and nbio address BAR5 for the DPC ERREVENT_ATHUB status check, done only
// while ras_hw_enabled has the DF bit (amdgpu_ras_init selects nbio_v4_3_ras
// then), and the SMN pair the PASSIVE_LEVEL UMC reader uses. umc_ras is
// amdgpu_ras_is_supported(UMC) (PspRasSupport umc_supported); umc is the
// validate_umc_discovery token (invalid: records cannot be read).
struct GpuHealthConfig final {
    RegisterIo io{};
    NbioRegisters nbio{};
    uint32_t ras_hw_enabled = 0;
    bool umc_ras = false;
    Smu13* smu = nullptr;
    VramAllocator* vram = nullptr;
    UmcTopology umc{};
};

// Event counts consume has published and run has not taken yet.
struct HealthPending final {
    uint32_t thermal_high = 0;
    uint32_t thermal_low = 0;
    uint32_t hardware_ctf = 0;
    uint32_t smu_ac = 0;
    uint32_t smu_dc = 0;
    uint32_t smu_throttling = 0;
    uint32_t smu_fan_abnormal = 0;
    uint32_t smu_fan_recovery = 0;
    uint32_t ras_umc_ecc = 0;
    uint32_t ras_athub = 0;
    uint32_t ras_athub_status = 0;
    uint32_t ras_gc_fed = 0;
    bool events_lost = false;
};

struct HealthReport final {
    HealthStatus status = HealthStatus::unavailable;
    HealthState state = HealthState::running;
    HealthReason reason = HealthReason::none;
    // Totals of every event taken by run.
    HealthPending totals{};
    uint32_t consumes = 0;
    uint32_t action_passes = 0;
    uint32_t athub_status_reads = 0;
    uint32_t athub_status_failures = 0;
    // Thermal response.
    SmuThermalLimits limits{};
    SmuThermalMetrics last_metrics{};
    bool metrics_valid = false;
    uint32_t swctf_checks = 0;
    uint32_t thermal_holds = 0;
    uint32_t gfxclk_limits = 0;
    uint32_t gfxclk_restores = 0;
    uint32_t smu_failures = 0;
    uint32_t throttle_logged = 0;
    uint32_t throttle_suppressed = 0;
    bool ac_power = true;
    // RAS response. umc_correctable/uncorrectable count channel records
    // (Linux ce_count/ue_count); umc_unknown counts ECC interrupts whose records
    // could not be read; umc_undecoded uncorrectable records without a usable
    // address (no AddrV/UECC, no column-bit mapping, lost to Overflow or
    // capacity); umc_uncontained retired pages whose owner is not a process
    // session; umc_contained ECC passes handled without recovery.
    uint32_t umc_correctable = 0;
    uint32_t umc_uncorrectable = 0;
    uint32_t umc_deferred = 0;
    uint32_t umc_unknown = 0;
    uint32_t umc_undecoded = 0;
    uint32_t umc_uncontained = 0;
    uint32_t umc_contained = 0;
    uint32_t umc_counter_inits = 0;
    uint32_t bad_page_threshold = 0;
    uint32_t pages_retired = 0;
    uint32_t pages_pending = 0;
    uint32_t pages_failed = 0;
    uint32_t sessions_quarantined = 0;
    uint32_t retired_listed = 0;
    uint32_t escalations = 0;
    uint32_t refusals = 0;
};

// What run asks of its caller: wait at most delay_us before the next run
// (0: until consume signals again).
struct HealthStep final {
    uint64_t delay_us = 0;
};

// Device-owned, constructed once; trivially destructible.
// initialize: PASSIVE_LEVEL before the consumer is installed. consume: from the
// IrqService drain consumer only (never reentrant), DISPATCH_LEVEL or below.
// run: PASSIVE_LEVEL, one caller at a time, under the device serialization the
// SMU13 mailbox and the VramAllocator require. report: under the same
// serialization (the mailbox part is copied under the lock).
class GpuHealth final {
public:
    GpuHealth() noexcept = default;
    GpuHealth(const GpuHealth&) = delete;
    GpuHealth& operator=(const GpuHealth&) = delete;

    HealthStatus initialize(const GpuHealthConfig& config, const GpuHealthPlatform& platform) noexcept;
    // A confirmed reset or resume rebuilt the SMU/IH owners: rebind the SMU
    // and RAS configuration; counters, retired pages and a refusal persist.
    HealthStatus rebind(const GpuHealthConfig& config) noexcept;
    // true when run has work (an event arrived or vectors were lost).
    bool consume(const IrqDispatchView& view) noexcept;
    HealthStep run() noexcept;
    HealthReport report() const noexcept;
    bool refused() const noexcept { return report_.state == HealthState::refused; }
    // The index-th retired VRAM page offset (index < report().retired_listed).
    bool retired_page(uint32_t index, uint64_t& vram_offset) const noexcept;

private:
    struct Seen final {
        uint32_t thermal_high;
        uint32_t thermal_low;
        uint32_t hardware_ctf;
        uint32_t smu_context[irq_smu_context_count];
        uint32_t ras_umc_ecc;
        uint32_t ras_athub;
        uint32_t ras_gc_fed;
    };

    HealthPending take() noexcept;
    void log(HealthEventKind kind, uint32_t a = 0, uint32_t b = 0, uint32_t c = 0) noexcept;
    void smu_failed(SmuStatus status, uint32_t step) noexcept;
    bool read_metrics() noexcept;
    void refuse(HealthReason reason) noexcept;
    void escalate(HealthReason reason) noexcept;
    void thermal_alert(uint64_t now) noexcept;
    void swctf_check(uint64_t now) noexcept;
    void hold_check(uint64_t now) noexcept;
    void smu_notifications(const HealthPending& pending, uint64_t now) noexcept;
    void umc_ecc(uint32_t count) noexcept;
    HealthContainment retire_page(const UmcErrorRecord& record, uint32_t& quarantined) noexcept;
    void bind_umc() noexcept;
    bool athub_status() noexcept;

    GpuHealthConfig config_{};
    GpuHealthPlatform platform_{};
    HealthReport report_{};
    // consume only (never concurrent with itself).
    Seen seen_{};
    bool seen_valid_ = false;
    // Guarded by platform_.lock.
    HealthPending pending_{};
    // run only.
    uint64_t swctf_due_us_ = 0;
    uint64_t hold_due_us_ = 0;
    uint64_t last_throttle_log_us_ = 0;
    bool throttle_logged_once_ = false;
    Umc810 umc_{};
    // umc_v8_10_err_cnt_init is due after initialize/rebind (reset or resume
    // cleared the UMC registers); run performs it under the serialization.
    bool umc_counters_due_ = false;
    UmcErrorRecord records_[health_umc_record_capacity]{};
    uint64_t retired_pages_[health_retired_page_capacity]{};
    bool initialized_ = false;
};

} // namespace rx7900::hardware

/*
Module: rx7900.hardware.gpu_health
Purpose: Turn thermal and RAS interrupt counters into bounded PASSIVE_LEVEL actions.
Created: 2026-10-24
Architecture: See gpu_health.hpp. Linux reacts to a hardware or confirmed
software CTF with orderly_poweroff; a display miniport must not power the host
down, so the device stops admitting GPU work for good instead (refuse), which is
never silent. Fatal RAS errors follow amdgpu_ras_reset_gpu: the owner's
destructive mode-1 recovery, or the same refusal when it has none. An
uncorrectable UMC ECC error whose MCA record converts to VRAM pages retires
those pages and quarantines their owning process sessions instead (Linux resets
in non-poison mode and only logs poison creation for UMC 8.10); it escalates as
before when the records cannot be read or converted, or a page belongs to the
driver.
*/
#include "gpu_health.hpp"

namespace rx7900::hardware {
namespace {

uint32_t delta(uint32_t now, uint32_t seen) noexcept {
    return now - seen;
}

uint32_t add_saturated(uint32_t a, uint32_t b) noexcept {
    return a > UINT32_MAX - b ? UINT32_MAX : a + b;
}

void add(HealthPending& total, const HealthPending& more) noexcept {
    total.thermal_high = add_saturated(total.thermal_high, more.thermal_high);
    total.thermal_low = add_saturated(total.thermal_low, more.thermal_low);
    total.hardware_ctf = add_saturated(total.hardware_ctf, more.hardware_ctf);
    total.smu_ac = add_saturated(total.smu_ac, more.smu_ac);
    total.smu_dc = add_saturated(total.smu_dc, more.smu_dc);
    total.smu_throttling = add_saturated(total.smu_throttling, more.smu_throttling);
    total.smu_fan_abnormal = add_saturated(total.smu_fan_abnormal, more.smu_fan_abnormal);
    total.smu_fan_recovery = add_saturated(total.smu_fan_recovery, more.smu_fan_recovery);
    total.ras_umc_ecc = add_saturated(total.ras_umc_ecc, more.ras_umc_ecc);
    total.ras_athub = add_saturated(total.ras_athub, more.ras_athub);
    total.ras_athub_status = add_saturated(total.ras_athub_status, more.ras_athub_status);
    total.ras_gc_fed = add_saturated(total.ras_gc_fed, more.ras_gc_fed);
    total.events_lost = total.events_lost || more.events_lost;
}

bool any(const HealthPending& p) noexcept {
    return p.thermal_high != 0 || p.thermal_low != 0 || p.hardware_ctf != 0 || p.smu_ac != 0 ||
           p.smu_dc != 0 || p.smu_throttling != 0 || p.smu_fan_abnormal != 0 || p.smu_fan_recovery != 0 ||
           p.ras_umc_ecc != 0 || p.ras_athub != 0 || p.ras_athub_status != 0 || p.ras_gc_fed != 0 ||
           p.events_lost;
}

uint32_t clamp_alert(int64_t celsius) noexcept {
    return celsius < 0 ? 0 : celsius > health_thermal_alert_maximum_c ? health_thermal_alert_maximum_c :
                                                                          static_cast<uint32_t>(celsius);
}

} // namespace

HealthStatus GpuHealth::initialize(const GpuHealthConfig& config, const GpuHealthPlatform& platform) noexcept {
    if (initialized_) {
        return HealthStatus::busy;
    }

    if (platform.now_us == nullptr || platform.lock == nullptr || platform.unlock == nullptr ||
        platform.refuse == nullptr || platform.escalate == nullptr || platform.quarantine_vram == nullptr ||
        platform.log == nullptr) {
        return HealthStatus::invalid_argument;
    }

    config_ = config;
    platform_ = platform;
    report_.status = HealthStatus::ready;
    report_.limits = config.smu != nullptr ? config.smu->thermal_limits() : SmuThermalLimits{};
    bind_umc();
    initialized_ = true;
    return HealthStatus::ready;
}

// The UMC reader exists only with UMC RAS support and a validated topology;
// its counter setup is redone after every (re)bind.
void GpuHealth::bind_umc() noexcept {
    umc_ = Umc810{};
    umc_counters_due_ = config_.umc_ras && config_.umc.valid() &&
                        umc_.initialize(config_.umc, config_.nbio, config_.io) == UmcStatus::ready;
}

HealthStatus GpuHealth::rebind(const GpuHealthConfig& config) noexcept {
    if (!initialized_) {
        return HealthStatus::unavailable;
    }

    // consume (DPC) reads the RAS/NBIO part under the mailbox lock.
    platform_.lock(platform_.context);
    config_ = config;
    platform_.unlock(platform_.context);
    report_.limits = config.smu != nullptr ? config.smu->thermal_limits() : SmuThermalLimits{};
    bind_umc();
    // The restarted SMU runs its boot policy again: no cap is applied.
    swctf_due_us_ = 0;
    hold_due_us_ = 0;

    if (report_.state == HealthState::thermal_limited) {
        report_.state = HealthState::running;
    }

    return HealthStatus::ready;
}

// nbio_v4_3_handle_ras_err_event_athub_intr_no_bifring: the IH cookie would go
// to the (disabled) BIF ring, so the status bit is polled and cleared.
bool GpuHealth::athub_status() noexcept {
    const uint32_t base = config_.nbio.valid() ? config_.nbio.nbif_base(2) : 0;
    uint32_t control = 0;

    if (base == 0 || !read_ip_register(config_.io, base, health_bif_doorbell_int_cntl, control) ||
        control == UINT32_MAX) {
        ++report_.athub_status_failures;
        return false;
    }

    ++report_.athub_status_reads;

    if ((control & health_athub_status) == 0) {
        return false;
    }

    write_ip_register(config_.io, base, health_bif_doorbell_int_cntl, control | health_athub_clear);
    return true;
}

bool GpuHealth::consume(const IrqDispatchView& view) noexcept {
    if (!initialized_ || view.report == nullptr) {
        return false;
    }

    // Only fields dispatch writes are read (IrqDispatchView contract).
    const IrqReport& r = *view.report;
    Seen now{};
    now.thermal_high = r.sequence[static_cast<uint32_t>(IrqSource::thermal_high)];
    now.thermal_low = r.sequence[static_cast<uint32_t>(IrqSource::thermal_low)];
    now.hardware_ctf = r.sequence[static_cast<uint32_t>(IrqSource::thermal_ctf)];

    for (uint32_t index = 0; index < irq_smu_context_count; ++index) {
        now.smu_context[index] = r.smu_context[index];
    }

    now.ras_umc_ecc = r.ras_umc_ecc;
    now.ras_athub = r.ras_athub;
    now.ras_gc_fed = r.ras_gc_fed;

    HealthPending delta_events{};
    delta_events.thermal_high = delta(now.thermal_high, seen_.thermal_high);
    delta_events.thermal_low = delta(now.thermal_low, seen_.thermal_low);
    delta_events.hardware_ctf = delta(now.hardware_ctf, seen_.hardware_ctf);
    delta_events.smu_ac = delta(now.smu_context[health_smu_context_ac], seen_.smu_context[health_smu_context_ac]);
    delta_events.smu_dc = delta(now.smu_context[health_smu_context_dc], seen_.smu_context[health_smu_context_dc]);
    delta_events.smu_throttling = delta(now.smu_context[health_smu_context_throttling],
                                        seen_.smu_context[health_smu_context_throttling]);
    delta_events.smu_fan_abnormal = delta(now.smu_context[health_smu_context_fan_abnormal],
                                          seen_.smu_context[health_smu_context_fan_abnormal]);
    delta_events.smu_fan_recovery = delta(now.smu_context[health_smu_context_fan_recovery],
                                          seen_.smu_context[health_smu_context_fan_recovery]);
    delta_events.ras_umc_ecc = delta(now.ras_umc_ecc, seen_.ras_umc_ecc);
    delta_events.ras_athub = delta(now.ras_athub, seen_.ras_athub);
    delta_events.ras_gc_fed = delta(now.ras_gc_fed, seen_.ras_gc_fed);
    // Lost vectors may have been a thermal alert: the action pass re-checks.
    delta_events.events_lost = view.events_lost;
    seen_ = now;

    // amdgpu_irq_handler -> amdgpu_ras_interrupt_fatal_error_handler after each
    // IH pass, only with nbio_v4_3_ras selected (DF RAS hardware-enabled).
    platform_.lock(platform_.context);
    const bool check_athub = (config_.ras_hw_enabled & (1u << health_ras_block_df)) != 0;

    if (check_athub && athub_status()) {
        delta_events.ras_athub_status = 1;
    }

    const bool work = any(delta_events);
    add(pending_, delta_events);
    platform_.unlock(platform_.context);
    return work;
}

HealthPending GpuHealth::take() noexcept {
    platform_.lock(platform_.context);
    const HealthPending taken = pending_;
    pending_ = {};
    platform_.unlock(platform_.context);
    add(report_.totals, taken);
    return taken;
}

bool GpuHealth::retired_page(uint32_t index, uint64_t& vram_offset) const noexcept {
    vram_offset = 0;

    if (index >= report_.retired_listed) {
        return false;
    }

    vram_offset = retired_pages_[index];
    return true;
}

HealthReport GpuHealth::report() const noexcept {
    HealthReport copy{};

    if (!initialized_) {
        return report_;
    }

    platform_.lock(platform_.context);
    copy = report_;
    platform_.unlock(platform_.context);
    return copy;
}

void GpuHealth::log(HealthEventKind kind, uint32_t a, uint32_t b, uint32_t c) noexcept {
    platform_.log(platform_.context, {kind, a, b, c});
}

void GpuHealth::smu_failed(SmuStatus status, uint32_t step) noexcept {
    ++report_.smu_failures;
    log(HealthEventKind::smu_failure, static_cast<uint32_t>(status), step);
}

bool GpuHealth::read_metrics() noexcept {
    if (config_.smu == nullptr) {
        report_.metrics_valid = false;
        return false;
    }

    SmuThermalMetrics metrics{};
    const auto status = config_.smu->read_thermal_metrics(metrics);
    report_.metrics_valid = status == SmuStatus::completed;

    if (!report_.metrics_valid) {
        smu_failed(status, 0);
        return false;
    }

    report_.last_metrics = metrics;
    return true;
}

void GpuHealth::refuse(HealthReason reason) noexcept {
    if (report_.state == HealthState::refused) {
        return;
    }

    report_.state = HealthState::refused;
    report_.reason = reason;
    ++report_.refusals;
    swctf_due_us_ = 0;
    hold_due_us_ = 0;
    log(HealthEventKind::refused, static_cast<uint32_t>(reason));
    platform_.refuse(platform_.context, reason);
}

// amdgpu_ras_reset_gpu (mode-1 for these sources); without a recovery path the
// device refuses to continue instead of running on unverified state.
void GpuHealth::escalate(HealthReason reason) noexcept {
    if (report_.state == HealthState::refused) {
        return;
    }

    if (platform_.escalate(platform_.context, reason)) {
        ++report_.escalations;
        log(HealthEventKind::escalated, static_cast<uint32_t>(reason));
        return;
    }

    refuse(reason);
}

// THM DIG_THERM_L2H: the temperature crossed DIG_THERM_INTH (the software
// shutdown temperature). Linux schedules swctf_delayed_work; the native
// response additionally caps GFXCLK at its lowest DPM level meanwhile.
void GpuHealth::thermal_alert(uint64_t now) noexcept {
    const SmuThermalState smu = config_.smu != nullptr ? config_.smu->thermal_state() : SmuThermalState{};
    bool limited = smu.gfxclk_limited;

    if (config_.smu != nullptr && !limited) {
        const auto status = config_.smu->limit_gfxclk(true);
        limited = status == SmuStatus::completed;

        if (limited) {
            ++report_.gfxclk_limits;
        } else {
            smu_failed(status, 1);
        }
    }

    if (limited && report_.state == HealthState::running) {
        report_.state = HealthState::thermal_limited;
    }

    log(HealthEventKind::thermal_alert, limited ? 1 : 0, smu.alert_high_c);

    if (swctf_due_us_ == 0) {
        swctf_due_us_ = now + health_swctf_delay_us;
    }

    hold_due_us_ = 0;
}

// smu_swctf_delayed_work_handler: below the software shutdown temperature after
// the delay, nothing more happens; otherwise (including an unreadable sensor or
// no shutdown temperature) the GPU must stop.
void GpuHealth::swctf_check(uint64_t now) noexcept {
    swctf_due_us_ = 0;
    ++report_.swctf_checks;
    report_.limits = config_.smu != nullptr ? config_.smu->thermal_limits() : SmuThermalLimits{};
    const uint32_t shutdown = report_.limits.software_shutdown_c;
    const bool read = read_metrics();

    if (shutdown != 0 && read && report_.last_metrics.hotspot_c < shutdown) {
        log(HealthEventKind::swctf_cleared, report_.last_metrics.hotspot_c, shutdown);

        if (report_.state == HealthState::thermal_limited) {
            report_.thermal_holds = 0;
            hold_due_us_ = now + health_thermal_hold_us;
        }

        return;
    }

    log(HealthEventKind::swctf_confirmed, read ? report_.last_metrics.hotspot_c : 0, shutdown, read ? 1 : 0);

    // Keep the clock low while work stops; the refusal does not depend on it.
    if (config_.smu != nullptr && !config_.smu->thermal_state().gfxclk_limited &&
        config_.smu->limit_gfxclk(true) == SmuStatus::completed) {
        ++report_.gfxclk_limits;
    }

    refuse(HealthReason::software_ctf);
}

// The bounded hold: lift the cap once the hotspot is hysteresis below the
// shutdown temperature, re-check at most health_thermal_max_holds times.
void GpuHealth::hold_check(uint64_t now) noexcept {
    hold_due_us_ = 0;

    if (report_.state != HealthState::thermal_limited || config_.smu == nullptr) {
        return;
    }

    const uint32_t shutdown = report_.limits.software_shutdown_c;
    const bool read = read_metrics();

    if (read && (shutdown == 0 || report_.last_metrics.hotspot_c >= shutdown)) {
        swctf_check(now);
        return;
    }

    if (read && report_.last_metrics.hotspot_c + health_thermal_hysteresis_c < shutdown) {
        const auto status = config_.smu->limit_gfxclk(false);

        if (status == SmuStatus::completed) {
            report_.state = HealthState::running;
            ++report_.gfxclk_restores;
            log(HealthEventKind::thermal_restored, report_.last_metrics.hotspot_c,
                config_.smu->thermal_state().gfxclk_soft_max_mhz);
            return;
        }

        smu_failed(status, 3);
    }

    ++report_.thermal_holds;

    if (report_.thermal_holds < health_thermal_max_holds) {
        hold_due_us_ = now + health_thermal_hold_us;
    } else {
        log(HealthEventKind::thermal_held, report_.last_metrics.hotspot_c, report_.thermal_holds);
    }
}

// smu_v13_0_irq_process MP1 SMU_TO_DRIVER contexts (the IrqService already
// acknowledged MP1). AC/DC and fan acknowledgements are sent once per kind
// and pass: PMFW raises the next one only after the acknowledgement.
void GpuHealth::smu_notifications(const HealthPending& p, uint64_t now) noexcept {
    Smu13* smu = config_.smu;

    for (uint32_t kind = 0; kind < 2; ++kind) {
        const bool ac = kind == 0;

        if ((ac ? p.smu_ac : p.smu_dc) == 0) {
            continue;
        }

        SmuStatus status = SmuStatus::unavailable;

        if (smu != nullptr) {
            status = smu->acknowledge_ac_dc();

            if (status != SmuStatus::completed) {
                smu_failed(status, 4);
            }
        }

        report_.ac_power = ac;
        log(HealthEventKind::ac_dc, ac ? 1 : 0, static_cast<uint32_t>(status));
    }

    if (p.smu_throttling != 0) {
        // throttle_int_counter, then the rate-limited report with the cause.
        if (throttle_logged_once_ && now - last_throttle_log_us_ < health_throttle_log_interval_us) {
            report_.throttle_suppressed = add_saturated(report_.throttle_suppressed, p.smu_throttling);
        } else {
            const bool read = read_metrics();
            log(HealthEventKind::throttling, read ? report_.last_metrics.throttler_status : 0,
                read ? report_.last_metrics.hotspot_c : 0, report_.throttle_suppressed);
            throttle_logged_once_ = true;
            last_throttle_log_us_ = now;
            report_.throttle_suppressed = 0;
            ++report_.throttle_logged;
        }
    }

    if (p.smu_fan_abnormal == 0 && p.smu_fan_recovery == 0) {
        return;
    }

    report_.limits = smu != nullptr ? smu->thermal_limits() : SmuThermalLimits{};
    const int64_t shutdown = report_.limits.software_shutdown_c;

    if (p.smu_fan_abnormal != 0) {
        // Reduce the soft CTF limit by the SKU's fan-abnormal offset.
        const uint32_t high = clamp_alert(shutdown + report_.limits.software_shutdown_offset_c);
        const auto status = smu != nullptr ? smu->set_thermal_alert_high(high) : SmuStatus::unavailable;

        if (status != SmuStatus::completed) {
            smu_failed(status, 5);
        }

        log(HealthEventKind::fan_abnormal, high, static_cast<uint32_t>(report_.limits.software_shutdown_offset_c));
    }

    if (p.smu_fan_recovery != 0) {
        const uint32_t high = clamp_alert(shutdown);
        const auto status = smu != nullptr ? smu->set_thermal_alert_high(high) : SmuStatus::unavailable;

        if (status != SmuStatus::completed) {
            smu_failed(status, 6);
        }

        log(HealthEventKind::fan_recovery, high);
    }
}

// amdgpu_ras_add_bad_pages -> amdgpu_vram_mgr_reserve_range for one GPU page:
// claimed now, or as soon as its current owner releases it; every process
// session owning the page is quarantined. The page is contained when it was
// retired before, was free, or only process sessions own it.
HealthContainment GpuHealth::retire_page(const UmcErrorRecord& record, uint32_t& quarantined) noexcept {
    const uint64_t page = record.vram_offset & ~(health_vram_page_bytes - 1);
    quarantined = 0;

    for (uint32_t index = 0; index < report_.retired_listed; ++index) {
        if (retired_pages_[index] == page) {
            return HealthContainment::contained;
        }
    }

    log(HealthEventKind::ras_umc_page, static_cast<uint32_t>(page), static_cast<uint32_t>(page >> 32),
        record.channel_index | (static_cast<uint32_t>(record.umc) << 8) | (static_cast<uint32_t>(record.node) << 16));

    if (config_.vram == nullptr) {
        ++report_.pages_failed;
        return HealthContainment::retire_failed;
    }

    // A reservation this pass did not make (firmware, TMR, stolen memory, the
    // pre-OS scanout tail): the driver's own data there is corrupted.
    const VramStatus before = config_.vram->query_page_status(page);

    if (before == VramStatus::success || before == VramStatus::busy) {
        return HealthContainment::driver_reserved;
    }

    const VramStatus status = config_.vram->reserve_range(page, health_vram_page_bytes);
    uint32_t owners = 0;
    quarantined = platform_.quarantine_vram(platform_.context, page, health_vram_page_bytes, owners);

    if (status != VramStatus::success && status != VramStatus::pending) {
        // Outside VRAM or reservation metadata exhausted (Linux: bad page
        // threshold); not listed, so a repeated error escalates again.
        ++report_.pages_failed;
        log(HealthEventKind::page_retire_failed, static_cast<uint32_t>(status));
        return HealthContainment::retire_failed;
    }

    if (report_.retired_listed < health_retired_page_capacity) {
        retired_pages_[report_.retired_listed++] = page;
    }

    if (status == VramStatus::success) {
        ++report_.pages_retired;
        return HealthContainment::contained;
    }

    ++report_.pages_pending;
    return owners != 0 ? HealthContainment::contained : HealthContainment::driver_owned;
}

// The GMC ecc_irq (DF source 0): amdgpu_umc_process_ecc_irq dispatches to the
// UMC RAS block only when it exists; amdgpu_umc_do_page_retirement then reads
// the MCA error records (amdgpu_umc_handle_bad_pages, direct query) and retires
// the pages of uncorrectable ones. Recovery is requested only for what page
// retirement and quarantine cannot contain.
void GpuHealth::umc_ecc(uint32_t count) noexcept {
    if (!config_.umc_ras) {
        log(HealthEventKind::ras_umc_ignored, count);
        return;
    }

    UmcQuery query{};

    if (umc_.ready()) {
        query = umc_.query(records_, health_umc_record_capacity);
    } else {
        query.access_failed = true;
    }

    report_.umc_correctable = add_saturated(report_.umc_correctable, query.correctable);
    report_.umc_uncorrectable = add_saturated(report_.umc_uncorrectable, query.uncorrectable);
    report_.umc_deferred = add_saturated(report_.umc_deferred, query.deferred);

    if (query.correctable != 0) {
        log(HealthEventKind::ras_umc_correctable, query.correctable);
    }

    // Records already read had their status cleared: retire them in any case.
    uint32_t quarantined = 0;
    uint32_t uncontained = 0;
    const uint32_t retired_before = report_.pages_retired + report_.pages_pending;
    const uint32_t records = query.records <= health_umc_record_capacity ? query.records : 0;

    for (uint32_t index = 0; index < records; ++index) {
        uint32_t newly = 0;
        const HealthContainment containment = retire_page(records_[index], newly);
        quarantined = add_saturated(quarantined, newly);

        if (containment != HealthContainment::contained) {
            ++uncontained;
            log(HealthEventKind::ras_umc_uncontained, static_cast<uint32_t>(records_[index].vram_offset),
                static_cast<uint32_t>(records_[index].vram_offset >> 32), static_cast<uint32_t>(containment));
        }
    }

    const uint32_t retired = report_.pages_retired + report_.pages_pending - retired_before;
    report_.sessions_quarantined = add_saturated(report_.sessions_quarantined, quarantined);
    report_.umc_uncontained = add_saturated(report_.umc_uncontained, uncontained);

    if (query.uncorrectable != 0) {
        log(HealthEventKind::ras_umc_uncorrectable, query.uncorrectable, retired, quarantined);
    }

    if (retired != 0 && config_.vram != nullptr) {
        const uint64_t threshold = config_.vram->vram_bytes() / health_bad_page_cover_bytes;
        report_.bad_page_threshold = threshold > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(threshold);
        const uint32_t pages = report_.pages_retired + report_.pages_pending;

        // amdgpu_ras_eeprom_append: "Saved bad pages %d reaches threshold value %d".
        if (pages >= report_.bad_page_threshold) {
            log(HealthEventKind::ras_bad_page_threshold, pages, report_.bad_page_threshold);
        }
    }

    if (query.access_failed) {
        // Severity and address of the unread channels unknown: VRAM contents
        // can no longer be trusted.
        ++report_.umc_unknown;
        log(HealthEventKind::ras_umc_unknown, count);
        escalate(HealthReason::ras_uncorrectable);
        return;
    }

    const uint32_t undecoded = query.undecoded + query.overflowed + (query.dropped != 0 ? 1 : 0);

    if (undecoded != 0) {
        report_.umc_undecoded = add_saturated(report_.umc_undecoded, undecoded);
        log(HealthEventKind::ras_umc_undecoded, query.undecoded, query.overflowed, query.dropped);
    }

    if (query.uncorrectable == 0) {
        return;
    }

    if (undecoded != 0 || uncontained != 0) {
        escalate(HealthReason::ras_uncorrectable);
        return;
    }

    ++report_.umc_contained;
    log(HealthEventKind::ras_umc_contained, query.uncorrectable, retired);
}

HealthStep GpuHealth::run() noexcept {
    HealthStep step{};

    if (!initialized_) {
        return step;
    }

    const uint64_t now = platform_.now_us(platform_.context);
    const HealthPending p = take();
    ++report_.action_passes;

    // ROM_SMUIO GPIO19: the hardware CTF already tripped (Linux powers off).
    if (p.hardware_ctf != 0) {
        log(HealthEventKind::hardware_ctf, p.hardware_ctf);
        refuse(HealthReason::hardware_ctf);
    }

    const bool active = report_.state != HealthState::refused;

    if (p.thermal_high != 0 && active) {
        thermal_alert(now);
    }

    // Dropped IH vectors could have been an L2H: check the sensor now.
    if (p.events_lost && active && swctf_due_us_ == 0 && hold_due_us_ == 0) {
        swctf_due_us_ = now;
    }

    if (p.thermal_low != 0) {
        log(HealthEventKind::under_temperature, p.thermal_low);
    }

    if (active) {
        smu_notifications(p, now);
    }

    if (p.ras_gc_fed != 0) {
        log(HealthEventKind::ras_gc_fed, p.ras_gc_fed);
    }

    // umc_v8_10_err_cnt_init (amdgpu_umc_ras_late_init) after every bind.
    if (umc_counters_due_) {
        umc_counters_due_ = false;
        const bool armed = umc_.initialize_counters();
        report_.umc_counter_inits += armed ? 1 : 0;
        log(HealthEventKind::ras_umc_counters, armed ? 1 : 0);
    }

    if (p.ras_umc_ecc != 0) {
        umc_ecc(p.ras_umc_ecc);
    }

    // amdgpu_ras_global_ras_isr: uncorrectable fatal error, mode-1 reset.
    if (p.ras_athub != 0 || p.ras_athub_status != 0) {
        log(HealthEventKind::ras_athub_fatal, p.ras_athub, p.ras_athub_status);
        escalate(HealthReason::ras_fatal);
    }

    if (swctf_due_us_ != 0 && now >= swctf_due_us_) {
        swctf_check(now);
    }

    if (hold_due_us_ != 0 && now >= hold_due_us_) {
        hold_check(now);
    }

    const uint64_t due[2]{swctf_due_us_, hold_due_us_};

    for (const uint64_t at : due) {
        if (at == 0) {
            continue;
        }

        const uint64_t delay = at > now ? at - now : 1;
        step.delay_us = step.delay_us == 0 || delay < step.delay_us ? delay : step.delay_us;
    }

    return step;
}

} // namespace rx7900::hardware

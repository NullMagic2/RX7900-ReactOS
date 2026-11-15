/*
Module: rx7900.tests.gpu_health
Purpose: Verify the thermal and RAS interrupt actions: counter consumption, the
SW CTF check and bounded GFXCLK response, CTF refusal, MP1 notifications, fan
limits, UMC 8.10 error records (production reader over a synthetic SMN UMC
register file), page retirement, session quarantine and containment,
ERREVENT_ATHUB escalation.
Created: 2026-10-24
Architecture: Production GpuHealth over the production Smu13 (synthetic MP1/THM
register machine plus test PMFW answers for the combo PPTable and SMU metrics),
the production VramAllocator and recording platform callbacks. Expected values
are hand-derived from Linux v6.12 smu_v13_0.c, amdgpu_smu.c, smu_v13_0_0_ppt.c,
amdgpu_ras.c and nbio_v4_3.c. Not hardware evidence.
*/
#include <map>
#include <memory>
#include <set>
#include <vector>

#include "asic_test_device.hpp"
#include "gpu_health.hpp"

namespace {

using namespace rx7900::hardware;
using rx7900_test::SimulatedNavi31;

rx7900_test::Checks checks;

void expect(bool condition, const char* contract) { checks.expect(condition, contract); }

constexpr uint32_t thm_int_ctrl = rx7900_test::thm_base + 0xb;
constexpr uint32_t bif_doorbell_int_cntl = rx7900_test::nbif_bases[2] + 0xfe;

// The model's mailbox plus the PMFW answers GpuHealth needs: combo-PPTable
// thermal fields (shutdown 110 C from the shared model, fan-abnormal offset,
// hotspot limit) and SMU metrics (temperatures, throttling percentages).
struct Pmfw final {
    SimulatedNavi31* device = nullptr;
    int16_t fan_offset = -10;
    uint16_t hotspot_limit = 105;
    uint16_t edge = 60;
    uint16_t hotspot = 70;
    uint16_t memory = 65;
    uint8_t throttling[22]{};
    unsigned metrics_transfers = 0;
    unsigned bif_reads = 0;
    // UMC 8.10 registers in SMN space (board UMC base 0x0C400000 * 4 and up),
    // reached through the RSMU data dword; umc_fail fails those accesses.
    std::map<uint32_t, uint32_t> umc;
    bool umc_fail = false;

    static constexpr uint32_t rsmu_data = (rx7900_test::nbif_bases[1] + 1) * 4;

    bool umc_access(uint32_t offset) const {
        return offset == rsmu_data && device->smn_index >= 0x31000000u && device->smn_index < 0x3c000000u;
    }

    static bool read(void* context, uint32_t offset, uint32_t& value) noexcept {
        auto& self = *static_cast<Pmfw*>(context);
        self.bif_reads += offset == bif_doorbell_int_cntl * 4 ? 1 : 0;

        if (self.umc_access(offset)) {
            const auto found = self.umc.find(self.device->smn_index);
            value = found == self.umc.end() ? 0 : found->second;
            return !self.umc_fail;
        }

        return SimulatedNavi31::read(self.device, offset, value);
    }

    static bool write(void* context, uint32_t offset, uint32_t value) noexcept {
        auto& self = *static_cast<Pmfw*>(context);

        if (self.umc_access(offset)) {
            self.umc[self.device->smn_index] = value;
            return !self.umc_fail;
        }

        if (!SimulatedNavi31::write(self.device, offset, value)) {
            return false;
        }

        if (offset == (rx7900_test::mp_base + 0x282) * 4) {
            self.answer(value);
        }

        return true;
    }

    static void delay(void* context, uint32_t microseconds) noexcept {
        SimulatedNavi31::delay(static_cast<Pmfw*>(context)->device, microseconds);
    }

    RegisterIo io() noexcept { return {this, rx7900_test::aperture_bytes, read, write, delay}; }

    // An MCA record of channel (node, umc, channel): STATUST0 0x3c2, ADDRT0
    // 0x3c4 at (base + reg + 0x400 * channel + 0x4000 * umc + 0x800000 * node) * 4.
    static uint32_t umc_register(uint32_t node, uint32_t umc_instance, uint32_t channel, uint32_t reg) {
        return (0x0c400000u + reg + 0x400u * channel + 0x4000u * umc_instance + 0x800000u * node) * 4;
    }

    void set64(uint32_t address, uint64_t value) {
        umc[address] = static_cast<uint32_t>(value);
        umc[address + 4] = static_cast<uint32_t>(value >> 32);
    }

    void umc_error(uint32_t node, uint32_t umc_instance, uint32_t channel, uint64_t status, uint64_t address) {
        set64(umc_register(node, umc_instance, channel, 0x3c2), status);
        set64(umc_register(node, umc_instance, channel, 0x3c4), address);
    }

    uint64_t umc_status(uint32_t node, uint32_t umc_instance, uint32_t channel) {
        return umc[umc_register(node, umc_instance, channel, 0x3c2)] |
               static_cast<uint64_t>(umc[umc_register(node, umc_instance, channel, 0x3c2) + 4]) << 32;
    }

    void put_u16(uint64_t address, uint16_t value) {
        const uint64_t word = address & ~3ULL;
        const uint32_t shift = static_cast<uint32_t>(address & 2) * 8;
        const uint32_t old = device->vram_u32(word);
        device->put_vram_u32(word, (old & ~(0xffffu << shift)) | (static_cast<uint32_t>(value) << shift));
    }

    void answer(uint32_t message) {
        const uint32_t parameter = device->smu_parameters.back();
        const uint64_t table = device->smu_driver_table;

        if (message == device->smu_fail_message) {
            return;
        }

        if (message == 0x12 && parameter == 1) {
            // PPTable_t (combo offset 832): SkuTable.TemperatureLimit[TEMP_HOTSPOT]
            // at 50 + 2, FanAbnormalTempLimitOffset at 1412.
            put_u16(table + 832 + 52, hotspot_limit);
            put_u16(table + 832 + 1412, static_cast<uint16_t>(fan_offset));
        } else if (message == 0x12 && parameter == 5) {
            // SmuMetrics_t AvgTemperature[TEMP_EDGE/HOTSPOT/MEM] at 120/122/128,
            // ThrottlingPercentage[22] at 154.
            ++metrics_transfers;
            put_u16(table + 120, edge);
            put_u16(table + 122, hotspot);
            put_u16(table + 128, memory);

            for (uint32_t index = 0; index < 22; index += 2) {
                put_u16(table + 154 + index,
                        static_cast<uint16_t>(throttling[index] | (throttling[index + 1] << 8)));
            }
        }
    }
};

struct Platform final {
    uint64_t now = 1000000;
    bool escalate_ok = true;
    // VRAM pages (offset) that a process session owns: quarantine reports them.
    std::map<uint64_t, uint32_t> session_owned;
    std::vector<HealthReason> refusals;
    std::vector<HealthReason> escalations;
    std::vector<std::pair<uint64_t, uint64_t>> quarantined;
    std::set<uint64_t> quarantined_pages;
    std::vector<HealthEvent> events;
    int locks = 0;
    int lock_errors = 0;

    static uint64_t clock(void* c) noexcept { return static_cast<Platform*>(c)->now; }
    static void lock(void* c) noexcept {
        auto& self = *static_cast<Platform*>(c);
        if (++self.locks != 1) ++self.lock_errors;
    }
    static void unlock(void* c) noexcept {
        auto& self = *static_cast<Platform*>(c);
        if (--self.locks != 0) ++self.lock_errors;
    }
    static void refuse(void* c, HealthReason reason) noexcept {
        static_cast<Platform*>(c)->refusals.push_back(reason);
    }
    static bool escalate(void* c, HealthReason reason) noexcept {
        auto& self = *static_cast<Platform*>(c);
        self.escalations.push_back(reason);
        return self.escalate_ok;
    }
    static uint32_t quarantine(void* c, uint64_t offset, uint64_t bytes, uint32_t& owners) noexcept {
        auto& self = *static_cast<Platform*>(c);
        self.quarantined.emplace_back(offset, bytes);
        const auto found = self.session_owned.find(offset);
        owners = found == self.session_owned.end() ? 0 : found->second;
        // The owners are newly quarantined only the first time.
        return self.quarantined_pages.insert(offset).second ? owners : 0;
    }
    static void log(void* c, const HealthEvent& event) noexcept {
        static_cast<Platform*>(c)->events.push_back(event);
    }

    GpuHealthPlatform callbacks() noexcept {
        return {this, clock, lock, unlock, refuse, escalate, quarantine, log};
    }

    unsigned count(HealthEventKind kind) const {
        unsigned n = 0;
        for (const auto& event : events) n += event.kind == kind ? 1 : 0;
        return n;
    }
};

struct Fixture final {
    std::unique_ptr<SimulatedNavi31> device = std::make_unique<SimulatedNavi31>();
    rx7900_test::VramBackend backend{};
    std::unique_ptr<rx7900::memory::Manager> manager = std::make_unique<rx7900::memory::Manager>();
    Nbio nbio{};
    NbioRegisters nbio_registers{};
    std::unique_ptr<Smu13> smu = std::make_unique<Smu13>();
    SmuRegisters registers{};
    ThmRegisters thm{};
    Pmfw pmfw{};
    std::unique_ptr<VramAllocator> vram = std::make_unique<VramAllocator>();
    rx7900::memory::Handle driver{};
    Platform platform{};
    std::unique_ptr<GpuHealth> health = std::make_unique<GpuHealth>();
    IrqReport irq{};
    IrqVmFaultLog faults{};
    bool ready = false;

    UmcTopology umc{};

    explicit Fixture(uint32_t ras_hw_enabled = 0, bool umc_ras = false, bool umc_topology = true) {
        backend.device = device.get();
        pmfw.device = device.get();
        manager->initialize({rx7900_test::fb_base + rx7900_test::arena_offset, rx7900_test::arena_bytes},
                            backend.backend());
        const auto binary = rx7900_test::navi31_discovery();
        validate_nbio_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes, nbio_registers);
        validate_smu_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes, registers);
        validate_thm_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes, thm);
        nbio.initialize(nbio_registers, device->io());
        nbio.program_common();
        smu->initialize(registers, pmfw.io(), nbio, {100, 3}, {device.get(), SimulatedNavi31::confirm});
        manager->allocate(smu13_driver_table_bytes, 0x1000, rx7900::memory::Access::read_write, true, driver);
        device->running_features_low = (1u << 1) | (1u << 3) | (1u << 24);
        device->reg(thm_int_ctrl) = 0x4fffffffu;
        device->reg(rx7900_test::mp_base + 0x2c2) = 0x1ffu;
        device->reg(rx7900_test::mp_base + 0x2c3) = 0x3u;
        std::vector<uint8_t> board(smu13_board_table_bytes, 0x5a);
        ready = smu->start() == SmuStatus::ready &&
                smu->set_tables(*manager, driver, rx7900::memory::Handle{}, rx7900_test::navi31_vram()) ==
                    SmuStatus::completed &&
                smu->setup_power_play(false, {board.data(), board.size()}) == SmuStatus::completed &&
                smu->set_allowed_features(false) == SmuStatus::completed &&
                smu->enable_all_features() == SmuStatus::completed &&
                smu->set_default_dpm_tables({120000, 250000, 125000, 140000, 90000, 80000, 60000}) ==
                    SmuStatus::completed &&
                smu->enable_thermal_alert(thm) == SmuStatus::completed;
        ready = ready && vram->initialize({64ULL << 20, 16ULL << 20}, nullptr, 0) == VramStatus::success;
        // The board's six UMC 8.10 nodes (24 channels).
        const auto umc_binary = rx7900_test::discovery_with(rx7900_test::navi31_umc_records());

        if (umc_topology) {
            ready = ready && validate_umc_discovery({umc_binary.data(), umc_binary.size()}, umc) ==
                                 DiscoveryStatus::validated;
        }

        const GpuHealthConfig config{pmfw.io(), nbio_registers, ras_hw_enabled, umc_ras, smu.get(), vram.get(), umc};
        ready = ready && health->initialize(config, platform.callbacks()) == HealthStatus::ready;
    }

    bool consume(bool lost = false) { return health->consume({&irq, &faults, lost}); }
    void bump(IrqSource source) { ++irq.sequence[static_cast<uint32_t>(source)]; }
    size_t sent() const { return device->smu_messages.size(); }
    bool last_sent(uint32_t message, uint32_t parameter) const {
        return !device->smu_messages.empty() && device->smu_messages.back() == message &&
               device->smu_parameters.back() == parameter;
    }
};

void initialization() {
    Fixture f;
    expect(f.ready, "SMU13 through the thermal alert, VRAM allocator and health owner ready");
    const auto limits = f.smu->thermal_limits();
    expect(limits.software_shutdown_c == 110 && limits.software_shutdown_offset_c == -10 &&
           limits.hotspot_limit_c == 105, "thermal range from the PPTable: shutdown 110, fan offset -10, hotspot 105");
    GpuHealth other;
    GpuHealthPlatform partial = f.platform.callbacks();
    partial.quarantine_vram = nullptr;
    expect(other.initialize({}, partial) == HealthStatus::invalid_argument &&
           f.health->initialize({}, f.platform.callbacks()) == HealthStatus::busy,
           "every platform callback is required; initialize runs once");
    expect(!f.consume() && f.health->run().delay_us == 0 && f.platform.events.empty(),
           "no counter change: no work, nothing logged");
}

void thermal_alert_and_hold() {
    Fixture f;
    const auto start = f.sent();
    f.bump(IrqSource::thermal_high);
    expect(f.consume(), "THM L2H wakes the action pass");
    expect(!f.consume(), "the same counters are consumed once");
    HealthStep step = f.health->run();
    // GFXCLK fine-grained levels 1000..1200 MHz (model); PPCLK_GFXCLK 0.
    expect(f.sent() == start + 1 && f.last_sent(0x1a, 1000) && f.smu->thermal_state().gfxclk_limited,
           "SetSoftMaxByFreq(GFXCLK, lowest DPM level) applied at once");
    expect(f.health->report().state == HealthState::thermal_limited && step.delay_us == 50000,
           "thermal_limited; the SW CTF check is due after AMDGPU_SWCTF_EXTRA_DELAY (50 ms)");
    f.platform.now += 10000;
    step = f.health->run();
    expect(f.pmfw.metrics_transfers == 0 && step.delay_us == 40000, "not yet due: no sensor read");
    f.platform.now += 40000;
    f.pmfw.hotspot = 100;
    step = f.health->run();
    expect(f.pmfw.metrics_transfers == 1 && f.health->report().swctf_checks == 1 &&
           f.platform.count(HealthEventKind::swctf_cleared) == 1 && f.platform.refusals.empty() &&
           step.delay_us == health_thermal_hold_us,
           "hotspot 100 < shutdown 110 after the delay: nothing stops; the cap is held");
    f.platform.now += health_thermal_hold_us;
    f.pmfw.hotspot = 108;
    step = f.health->run();
    expect(f.health->report().state == HealthState::thermal_limited && f.health->report().thermal_holds == 1 &&
           step.delay_us == health_thermal_hold_us, "108 C is within the hysteresis: keep the cap, re-check");
    f.platform.now += health_thermal_hold_us;
    f.pmfw.hotspot = 90;
    const auto before = f.sent();
    step = f.health->run();
    expect(f.last_sent(0x1a, 1200) && f.sent() == before + 2 && !f.smu->thermal_state().gfxclk_limited &&
           f.health->report().state == HealthState::running && step.delay_us == 0 &&
           f.health->report().gfxclk_restores == 1,
           "90 C: the cap is lifted to the highest GFXCLK level and nothing is scheduled");

    // Bounded: a hotspot that stays just under the limit stops re-checking.
    Fixture held;
    held.bump(IrqSource::thermal_high);
    held.consume();
    held.health->run();
    held.platform.now += 50000;
    held.pmfw.hotspot = 107;
    held.health->run();
    unsigned rounds = 0;
    for (; rounds < 40; ++rounds) {
        held.platform.now += health_thermal_hold_us;
        if (held.health->run().delay_us == 0) break;
    }
    expect(rounds == health_thermal_max_holds - 1 && held.platform.count(HealthEventKind::thermal_held) == 1 &&
           held.smu->thermal_state().gfxclk_limited, "after the hold limit the cap stays and checks stop");
}

void software_ctf() {
    Fixture f;
    f.bump(IrqSource::thermal_high);
    f.consume();
    f.health->run();
    f.platform.now += 50000;
    f.pmfw.hotspot = 112;
    f.health->run();
    expect(f.platform.refusals == std::vector<HealthReason>{HealthReason::software_ctf} &&
           f.health->refused() && f.platform.count(HealthEventKind::swctf_confirmed) == 1,
           "hotspot at or above the shutdown temperature after 50 ms: SW CTF refuses further work");
    const auto sent = f.sent();
    f.bump(IrqSource::thermal_high);
    f.consume();
    expect(f.health->run().delay_us == 0 && f.sent() == sent && f.platform.refusals.size() == 1,
           "after the refusal later alerts send nothing and refuse once");

    Fixture unreadable;
    unreadable.bump(IrqSource::thermal_high);
    unreadable.consume();
    unreadable.health->run();
    unreadable.platform.now += 50000;
    unreadable.device->smu_fail_message = 0x12;
    unreadable.health->run();
    expect(unreadable.health->refused() && unreadable.health->report().smu_failures >= 1,
           "an unreadable hotspot sensor is treated as SW CTF (Linux shuts down unless proven below)");

    Fixture lost;
    expect(lost.consume(true), "dropped IH vectors wake the action pass");
    lost.pmfw.hotspot = 111;
    lost.health->run();
    expect(lost.health->refused() && lost.health->report().swctf_checks == 1,
           "lost vectors (a possible L2H): the sensor is checked at once");

    Fixture hardware;
    hardware.bump(IrqSource::thermal_ctf);
    hardware.consume();
    const auto before = hardware.sent();
    hardware.health->run();
    expect(hardware.platform.refusals == std::vector<HealthReason>{HealthReason::hardware_ctf} &&
           hardware.sent() == before && hardware.platform.count(HealthEventKind::hardware_ctf) == 1,
           "ROM_SMUIO GPIO19 hardware CTF: logged and refused immediately, no SMU traffic");
}

void smu_notifications() {
    Fixture f;
    f.bump(IrqSource::thermal_low);
    ++f.irq.smu_context[health_smu_context_dc];
    f.consume();
    f.health->run();
    expect(f.last_sent(0x34, 0) && !f.health->report().ac_power && f.platform.count(HealthEventKind::ac_dc) == 1 &&
           f.platform.count(HealthEventKind::under_temperature) == 1,
           "DC notification: ReenableAcDcInterrupt sent, DC recorded; H2L logged");
    ++f.irq.smu_context[health_smu_context_ac];
    f.consume();
    f.health->run();
    expect(f.health->report().ac_power && f.smu->report().ac_dc_acknowledged == 2, "AC notification acknowledged");

    f.pmfw.throttling[1] = 20;     // THROTTLER_TEMP_HOTSPOT_BIT
    f.pmfw.throttling[16] = 5;     // THROTTLER_PPT0_BIT
    ++f.irq.smu_context[health_smu_context_throttling];
    f.consume();
    f.health->run();
    expect(f.platform.count(HealthEventKind::throttling) == 1 && f.platform.events.back().value0 == 0x10002u &&
           f.platform.events.back().value1 == 70, "throttling logged with the throttler status bits and hotspot");
    const unsigned transfers = f.pmfw.metrics_transfers;
    f.irq.smu_context[health_smu_context_throttling] += 3;
    f.platform.now += 1000000;
    f.consume();
    f.health->run();
    expect(f.platform.count(HealthEventKind::throttling) == 1 && f.health->report().throttle_suppressed == 3 &&
           f.pmfw.metrics_transfers == transfers, "within 59 s throttling reports are counted, not logged");
    ++f.irq.smu_context[health_smu_context_throttling];
    f.platform.now += health_throttle_log_interval_us;
    f.consume();
    f.health->run();
    expect(f.platform.count(HealthEventKind::throttling) == 2 && f.platform.events.back().value2 == 3,
           "the next report after the interval carries the suppressed count");

    const uint32_t before = f.device->reg(thm_int_ctrl);
    ++f.irq.smu_context[health_smu_context_fan_abnormal];
    f.consume();
    f.health->run();
    expect((f.device->reg(thm_int_ctrl) & 0xffu) == 100 && (f.device->reg(thm_int_ctrl) & 0x04000000u) == 0 &&
           (f.device->reg(thm_int_ctrl) & ~0x040000ffu) == (before & ~0x040000ffu),
           "FAN_ABNORMAL: DIG_THERM_INTH = 110 + (-10), THERM_TRIGGER_MASK cleared, other fields kept");
    ++f.irq.smu_context[health_smu_context_fan_recovery];
    f.consume();
    f.health->run();
    expect((f.device->reg(thm_int_ctrl) & 0xffu) == 110 && f.smu->thermal_state().alert_high_c == 110,
           "FAN_RECOVERY: INTH back to the software shutdown temperature");
}

constexpr uint64_t umc_ue = umc_status_val | umc_status_addrv | umc_status_uecc;

// The four pages one converted record retires (the conversion itself is
// checked against the pinned Linux code in umc_v8_10_tests).
std::vector<uint64_t> pages_of(const Fixture& f, uint32_t node, uint32_t umc, uint32_t channel, uint64_t status,
                               uint64_t address) {
    UmcErrorRecord records[umc_v8_10_retire_unit]{};
    std::vector<uint64_t> pages;

    if (umc_v8_10_convert_error_address(f.umc, node, umc, channel, status, address, records)) {
        for (const auto& record : records) pages.push_back(record.vram_offset);
    }

    return pages;
}

void ecc(Fixture& f) {
    ++f.irq.ras_umc_ecc;
    f.consume();
    f.health->run();
}

bool last_uncontained(const Fixture& f, HealthContainment containment) {
    for (auto event = f.platform.events.rbegin(); event != f.platform.events.rend(); ++event) {
        if (event->kind == HealthEventKind::ras_umc_uncontained) {
            return event->value2 == static_cast<uint32_t>(containment);
        }
    }

    return false;
}

void ras_umc() {
    Fixture none;
    ++none.irq.ras_umc_ecc;
    none.consume();
    none.health->run();
    expect(none.platform.count(HealthEventKind::ras_umc_ignored) == 1 && none.platform.escalations.empty() &&
           none.platform.refusals.empty() && none.pmfw.umc.empty(),
           "no UMC RAS support: the ECC vector is logged and ignored as Linux, no UMC register is touched");

    Fixture f(1u << health_ras_block_umc, true);
    f.health->run();
    expect(f.platform.count(HealthEventKind::ras_umc_counters) == 1 && f.health->report().umc_counter_inits == 1 &&
           f.pmfw.umc[Pmfw::umc_register(5, 1, 1, 0x328)] == 0x1000u &&
           f.pmfw.umc.count(Pmfw::umc_register(0, 0, 0, 0x329)) == 1,
           "first pass: umc_v8_10_err_cnt_init on all 24 channels (GeccErrInt = 1, GeccErrCnt = 0)");

    // One UECC record with four free pages plus one CE: retired, nothing escalates.
    const auto pages = pages_of(f, 1, 0, 1, umc_ue, 0x20000);
    f.pmfw.umc_error(1, 0, 1, umc_ue, 0x20000);
    f.pmfw.umc_error(3, 1, 0, umc_status_val | umc_status_cecc, 0);
    ecc(f);
    auto report = f.health->report();
    bool listed = pages.size() == 4 && report.retired_listed == 4;

    for (uint32_t index = 0; listed && index < 4; ++index) {
        uint64_t page = 0;
        listed = f.health->retired_page(index, page) && page == pages[index] &&
                 f.vram->query_page_status(page) == VramStatus::success;
    }

    expect(report.umc_uncorrectable == 1 && report.umc_correctable == 1 && report.pages_retired == 4 && listed,
           "an uncorrectable record retires the exact four [C6 C5] pages Linux computes; the CE is only counted");
    expect(f.platform.escalations.empty() && f.platform.refusals.empty() && report.umc_contained == 1 &&
           f.platform.count(HealthEventKind::ras_umc_page) == 4 &&
           f.platform.count(HealthEventKind::ras_umc_contained) == 1 && f.pmfw.umc_status(1, 0, 1) == 0 &&
           f.pmfw.umc_status(3, 1, 0) == 0,
           "decoded and contained: no recovery, statuses cleared");

    f.pmfw.umc_error(1, 0, 1, umc_ue, 0x20000);
    ecc(f);
    expect(f.health->report().pages_retired == 4 && f.health->report().retired_listed == 4 &&
           f.platform.escalations.empty() && f.health->report().umc_contained == 2,
           "the same error again: already retired pages collapse (amdgpu_ras_check_bad_page)");
    expect(f.platform.count(HealthEventKind::ras_bad_page_threshold) == 1 &&
           f.health->report().bad_page_threshold == 0,
           "64 MiB of VRAM gives Linux's threshold 0 (one page per 100 MiB): reaching it is only reported");

    // A page owned by a process session: claimed on release, the owner quarantined.
    Fixture s(1u << health_ras_block_umc, true);
    const auto owned = pages_of(s, 2, 1, 1, umc_ue, 0x40000);
    VramAllocationHandle buffer{};
    expect(owned.size() == 4 &&
           s.vram->allocate_at(owned[2], 4096, {false, false, true, false, false}, buffer) == VramStatus::success,
           "a buffer covers the third page");
    s.platform.session_owned[owned[2]] = 1;
    s.pmfw.umc_error(2, 1, 1, umc_ue, 0x40000);
    ecc(s);
    report = s.health->report();
    expect(report.pages_pending == 1 && report.pages_retired == 3 && report.sessions_quarantined == 1 &&
           s.platform.escalations.empty() && s.vram->query_page_status(owned[2]) == VramStatus::busy &&
           s.platform.quarantined.size() == 4 && s.platform.quarantined[2].first == owned[2] &&
           s.platform.quarantined[2].second == 4096,
           "a session-owned page: pending reservation, only that session quarantined, no recovery");
    s.vram->release(buffer);
    expect(s.vram->query_page_status(owned[2]) == VramStatus::success,
           "the pending page is retired as soon as its owner releases it");

    // The same page owned by the driver (no session): recovery requested.
    Fixture d(1u << health_ras_block_umc, true);
    expect(d.vram->allocate_at(owned[1], 4096, {false, false, true, false, false}, buffer) == VramStatus::success,
           "a driver allocation covers the second page");
    d.pmfw.umc_error(2, 1, 1, umc_ue, 0x40000);
    ecc(d);
    expect(d.platform.escalations == std::vector<HealthReason>{HealthReason::ras_uncorrectable} &&
           d.health->report().umc_uncontained == 1 && last_uncontained(d, HealthContainment::driver_owned) &&
           d.health->report().pages_retired == 3 && d.health->report().pages_pending == 1,
           "a driver-owned bad page escalates; the other pages are still retired");

    // A page inside a driver VRAM reservation (firmware, TMR, stolen memory).
    Fixture r(1u << health_ras_block_umc, true);
    const VramRange firmware{owned[0], owned[0] + 4096};
    expect(r.vram->shutdown() == VramStatus::success &&
           r.vram->initialize({64ULL << 20, 16ULL << 20}, &firmware, 1) == VramStatus::success,
           "VRAM with a reserved firmware page");
    r.pmfw.umc_error(2, 1, 1, umc_ue, 0x40000);
    ecc(r);
    expect(r.platform.escalations.size() == 1 && last_uncontained(r, HealthContainment::driver_reserved) &&
           r.health->report().pages_retired == 3,
           "a bad page inside a driver reservation escalates");

    // Deferred (poison) without UECC, and a UE without AddrV: no address.
    Fixture u(1u << health_ras_block_umc, true);
    u.pmfw.umc_error(0, 0, 0, umc_status_val | umc_status_addrv | umc_status_deferred, 0x1000);
    u.pmfw.umc_error(4, 1, 0, umc_status_val | umc_status_uecc, 0x1000);
    ecc(u);
    report = u.health->report();
    expect(report.umc_uncorrectable == 2 && report.umc_deferred == 1 && report.umc_undecoded == 2 &&
           report.pages_retired == 0 && u.platform.escalations.size() == 1 &&
           u.platform.count(HealthEventKind::ras_umc_undecoded) == 1 && u.pmfw.umc_status(0, 0, 0) == 0 &&
           u.pmfw.umc_status(4, 1, 0) == 0,
           "uncorrectable records without a usable address keep the escalation (and are cleared)");

    // Overflow: an earlier record of that channel was lost.
    Fixture o(1u << health_ras_block_umc, true);
    o.pmfw.umc_error(1, 1, 0, umc_ue | umc_status_overflow, 0x20000);
    ecc(o);
    expect(o.health->report().pages_retired == 4 && o.platform.escalations.size() == 1 &&
           o.health->report().umc_undecoded == 1,
           "an overflowed record retires its pages and still escalates for the lost one");

    // More pages than one query holds (7 UECC records = 28 pages > 24).
    Fixture c(1u << health_ras_block_umc, true);
    const uint32_t nodes[7]{0, 0, 1, 2, 3, 4, 5};

    for (uint32_t index = 0; index < 7; ++index) {
        c.pmfw.umc_error(nodes[index], index & 1, 0, umc_ue, 0x10000ULL * (index + 1));
    }

    ecc(c);
    expect(c.health->report().pages_retired == 24 && c.platform.escalations.size() == 1 &&
           c.health->report().umc_uncorrectable == 7,
           "records beyond max_ras_err_cnt_per_query: 24 pages retired, the rest escalates");

    // A converted page outside VRAM cannot be reserved.
    Fixture v(1u << health_ras_block_umc, true);
    v.pmfw.umc_error(0, 1, 1, umc_ue, 0x00ffff00000000ULL);
    ecc(v);
    expect(v.platform.escalations.size() == 1 && last_uncontained(v, HealthContainment::retire_failed) &&
           v.health->report().pages_failed == 4 && v.health->report().retired_listed == 0,
           "a page beyond VRAM fails retirement and escalates");
    v.pmfw.umc_error(0, 1, 1, umc_ue, 0x00ffff00000000ULL);
    ecc(v);
    expect(v.platform.escalations.size() == 2 && v.health->report().pages_failed == 8,
           "an unretired page is not remembered: the same error escalates again");

    // SMN access failure while reading.
    Fixture a(1u << health_ras_block_umc, true);
    a.pmfw.umc_error(0, 0, 0, umc_ue, 0x20000);
    a.pmfw.umc_fail = true;
    ecc(a);
    expect(a.health->report().umc_unknown == 1 && a.platform.escalations.size() == 1 &&
           a.platform.count(HealthEventKind::ras_umc_unknown) == 1 &&
           a.platform.count(HealthEventKind::ras_umc_counters) == 1 && a.health->report().umc_counter_inits == 0,
           "failed UMC register access: counters not armed, records unknown, escalation");

    Fixture unknown(1u << health_ras_block_umc, true, false);
    unknown.platform.escalate_ok = false;
    ecc(unknown);
    expect(unknown.health->report().umc_unknown == 1 && unknown.pmfw.umc.empty() &&
           unknown.platform.refusals == std::vector<HealthReason>{HealthReason::ras_uncorrectable},
           "no validated UMC layout and no recovery path: the device refuses to continue");
}

void ras_athub_and_gc() {
    Fixture plain;
    plain.consume();
    expect(plain.pmfw.bif_reads == 0,
           "without DF RAS (nbio_v4_3_ras not selected) no BIF status is read");

    Fixture f(1u << health_ras_block_df, false);
    f.device->reg(bif_doorbell_int_cntl) = 0x00000004u;
    expect(f.consume() && (f.device->reg(bif_doorbell_int_cntl) & 0x00040000u) != 0,
           "each drain reads BIF_DOORBELL_INT_CNTL; a set ERREVENT status is cleared (CLEAR bit written)");
    f.device->reg(bif_doorbell_int_cntl) = 0;
    expect(!f.consume() && f.health->report().athub_status_reads == 2, "a clear status is no event");
    f.health->run();
    expect(f.platform.escalations == std::vector<HealthReason>{HealthReason::ras_fatal} &&
           f.platform.count(HealthEventKind::ras_athub_fatal) == 1,
           "ERREVENT_ATHUB: uncorrectable fatal error, mode-1 recovery requested");

    Fixture iv;
    ++iv.irq.ras_athub;
    ++iv.irq.ras_gc_fed;
    iv.platform.escalate_ok = false;
    iv.consume();
    iv.health->run();
    expect(iv.platform.count(HealthEventKind::ras_gc_fed) == 1 &&
           iv.platform.refusals == std::vector<HealthReason>{HealthReason::ras_fatal},
           "an ATHUB vector without a recovery path refuses; RLC GC FED is only logged (GC 11.0.0)");
    expect(iv.platform.lock_errors == 0 && f.platform.lock_errors == 0, "mailbox lock balanced");
}

void rebind_after_reset() {
    Fixture f;
    f.bump(IrqSource::thermal_high);
    f.consume();
    f.health->run();
    expect(f.health->report().state == HealthState::thermal_limited, "limited before the reset");
    const GpuHealthConfig config{f.device->io(), f.nbio_registers, 0, false, f.smu.get(), f.vram.get()};
    expect(f.health->rebind(config) == HealthStatus::ready && f.health->report().state == HealthState::running &&
           f.health->run().delay_us == 0, "a rebuilt SMU runs its boot policy: the cap and its checks are gone");

    Fixture u(1u << health_ras_block_umc, true);
    u.health->run();
    u.pmfw.umc.clear();
    const GpuHealthConfig umc_config{u.pmfw.io(), u.nbio_registers, 1u << health_ras_block_umc, true, u.smu.get(),
                                     u.vram.get(), u.umc};
    expect(u.health->rebind(umc_config) == HealthStatus::ready && u.health->run().delay_us == 0 &&
           u.health->report().umc_counter_inits == 2 && u.pmfw.umc[Pmfw::umc_register(2, 0, 1, 0x328)] == 0x1000u,
           "after a reset or resume the UMC error counters are armed again");
}

} // namespace

int main() {
    initialization();
    thermal_alert_and_hold();
    software_ctf();
    smu_notifications();
    ras_umc();
    ras_athub_and_gc();
    rebind_after_reset();
    return checks.finish("gpu_health");
}

/*
Module: rx7900.tests.smu13
Purpose: Verify SMU13 mailbox ordering, response decoding, tables, hang handling,
late-init default policy, hardware teardown and the explicit mode-1 reset.
Created: 2026-10-21
Architecture: Synthetic register machine plus a local PMFW response wrapper; no
PMFW is contacted. Expected values are hand-derived from Linux v6.12.
*/
#include <initializer_list>
#include <memory>
#include <vector>
#include <utility>

#include "asic_test_device.hpp"

namespace {

using namespace rx7900::hardware;
using rx7900_test::SimulatedNavi31;

rx7900_test::Checks checks;

void expect(bool condition, const char* contract) { checks.expect(condition, contract); }

struct Fixture final {
    std::unique_ptr<SimulatedNavi31> device = std::make_unique<SimulatedNavi31>();
    rx7900_test::VramBackend backend{};
    std::unique_ptr<rx7900::memory::Manager> manager = std::make_unique<rx7900::memory::Manager>();
    Nbio nbio{};
    std::unique_ptr<Smu13> smu = std::make_unique<Smu13>();
    SmuRegisters registers{};

    Fixture() {
        backend.device = device.get();
        manager->initialize({rx7900_test::fb_base + rx7900_test::arena_offset,
                             rx7900_test::arena_bytes}, backend.backend());
        const auto binary = rx7900_test::navi31_discovery();
        NbioRegisters nbio_registers;
        validate_nbio_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes,
                                nbio_registers);
        validate_smu_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes, registers);
        nbio.initialize(nbio_registers, device->io());
        nbio.program_common();
        smu->initialize(registers, device->io(), nbio, {100, 3},
                        {device.get(), SimulatedNavi31::confirm});
    }

    uint32_t mailbox(uint32_t reg) { return device->reg(rx7900_test::mp_base + reg); }
};

void discovery() {
    const auto binary = rx7900_test::navi31_discovery();
    SmuRegisters registers;
    expect(validate_smu_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes,
                                  registers) == DiscoveryStatus::validated &&
           registers.segment0_dword_base() == rx7900_test::mp_base, "MP1 13.0.0 validated");
    expect(validate_smu_discovery({binary.data(), binary.size()}, 0x58000, registers) ==
           DiscoveryStatus::invalid_argument && !registers.valid(),
           "mailbox outside the aperture refused");
    const auto other = rx7900_test::discovery_with({{discovery_mp1_hardware_id, 13, 0, {0x16000}},
                                                    {discovery_mp1_hardware_id, 13, 0, {0x17000}}});
    expect(validate_smu_discovery({other.data(), other.size()}, rx7900_test::aperture_bytes,
                                  registers) == DiscoveryStatus::unsupported,
           "duplicate MP1 instance0 records refused");
}

void startup_and_messages() {
    Fixture fixture;
    expect(fixture.smu->start() == SmuStatus::ready, "PMFW running and versions read");
    const auto report = fixture.smu->report();
    expect(report.interface_version == 0x3d && report.interface_matches &&
           report.smu_version == 0x004e8300 && report.firmware_flags == 1,
           "driver interface 0x3D and SMU version reported");
    expect(fixture.device->smu_messages.size() == 2 && fixture.device->smu_messages.at(0) == 0x3 &&
           fixture.device->smu_messages.at(1) == 0x2,
           "GetDriverIfVersion precedes GetSmuVersion as in smu_cmn_get_smc_version");

    uint32_t value = 0;
    fixture.device->smu_response = 0xfe;
    expect(fixture.smu->send(0x7f, 9, &value) == SmuStatus::unknown_command && value == 0 &&
           fixture.mailbox(0x292) == 9, "unknown command decoded; argument not read");
    fixture.device->smu_response = 0xfd;
    expect(fixture.smu->send(0x7f, 0, nullptr) == SmuStatus::rejected_prerequisite,
           "prerequisite rejection decoded");
    fixture.device->smu_response = 0xfc;
    expect(fixture.smu->send(0x7f, 0, nullptr) == SmuStatus::busy_other, "busy-other decoded");
    fixture.device->smu_response = 1;
    expect(fixture.smu->send(smu13_message::test, 0, nullptr) == SmuStatus::completed,
           "non-terminal failures leave the mailbox usable");

    fixture.device->reg(rx7900_test::mp_base + 0x29a) = 0;
    const auto messages = fixture.device->smu_messages.size();
    expect(fixture.smu->send(smu13_message::test, 0, nullptr) == SmuStatus::timeout &&
           fixture.device->smu_messages.size() == messages,
           "busy previous message blocks a new send");

    fixture.device->reg(rx7900_test::mp_base + 0x29a) = 1;
    fixture.device->smu_response = 0xfb;
    expect(fixture.smu->send(smu13_message::test, 0, nullptr) == SmuStatus::hung,
           "debug/unknown response marks firmware hung");
    fixture.device->smu_response = 1;
    expect(fixture.smu->send(smu13_message::test, 0, nullptr) == SmuStatus::hung &&
           fixture.device->smu_messages.size() == messages + 1, "hung firmware refuses messages");
}

void first_message_and_firmware() {
    Fixture fixture;
    fixture.device->reg(rx7900_test::mp_base + 0x29a) = 0;
    expect(fixture.smu->send(smu13_message::test, 0, nullptr) == SmuStatus::completed,
           "first message ignores the initial response register");

    Fixture stopped;
    stopped.device->mp1_flags = 0;
    expect(stopped.smu->start() == SmuStatus::firmware_not_running &&
           stopped.device->smu_messages.empty() && stopped.device->delays.size() == 3,
           "PMFW without interrupts enabled is not messaged");

    Smu13 fresh;
    expect(fresh.send(1, 0, nullptr) == SmuStatus::unavailable, "uninitialized mailbox refuses");
    expect(fresh.initialize(SmuRegisters{}, stopped.device->io(), stopped.nbio, {1, 1}, {}) ==
           SmuStatus::unsupported_ip, "default MP1 token refused");
}

void tables_and_features() {
    Fixture fixture;
    rx7900::memory::Handle driver{};
    rx7900::memory::Handle tool{};
    fixture.manager->allocate(smu13_driver_table_bytes, 0x1000, rx7900::memory::Access::read_write,
                              false, driver);
    fixture.manager->allocate(smu13_tool_table_bytes, 0x1000, rx7900::memory::Access::read_write,
                              false, tool);
    rx7900::memory::AllocationInfo driver_info{};
    rx7900::memory::AllocationInfo tool_info{};
    fixture.manager->describe(driver, driver_info);
    fixture.manager->describe(tool, tool_info);
    fixture.smu->start();
    fixture.device->events.clear();

    expect(fixture.smu->set_tables(*fixture.manager, driver, tool, rx7900_test::navi31_vram()) ==
           SmuStatus::completed, "driver and tool tables located");
    const auto& events = fixture.device->events;
    expect(events.size() == 4 && events.at(0).value == 0xe &&
           events.at(0).detail == static_cast<uint32_t>(driver_info.gpu.virtual_address >> 32) &&
           events.at(1).value == 0xf &&
           events.at(1).detail == static_cast<uint32_t>(driver_info.gpu.virtual_address) &&
           events.at(2).value == 0x10 && events.at(3).value == 0x11 &&
           events.at(3).detail == static_cast<uint32_t>(tool_info.gpu.virtual_address),
           "SetDriverDramAddr and SetToolsDramAddr high then low");
    expect(fixture.smu->set_tables(*fixture.manager, driver, tool, rx7900_test::navi31_vram()) ==
           SmuStatus::busy, "tables are located once");

    expect(fixture.smu->enable_all_features() == SmuStatus::completed &&
           fixture.smu->report().enabled_features == 0x0000000500001234ULL,
           "EnableAllSmuFeatures followed by running-feature readback");

    fixture.manager->release(driver);
    expect(!fixture.manager->empty() && fixture.smu->release_after_reset() == SmuStatus::quarantined,
           "tables stay leased until reset proof");
    fixture.device->reset_confirmed = true;
    expect(fixture.smu->release_after_reset() == SmuStatus::released, "confirmed reset releases");
    fixture.manager->release(tool);
    expect(fixture.manager->empty(), "both tables destroyed after release");

    Fixture small;
    rx7900::memory::Handle tiny{};
    small.manager->allocate(0x1000, 0x1000, rx7900::memory::Access::read_write, false, tiny);
    small.smu->start();
    const auto before = small.device->smu_messages.size();
    expect(small.smu->set_tables(*small.manager, tiny, rx7900::memory::Handle{},
                                 rx7900_test::navi31_vram()) == SmuStatus::invalid_memory &&
           small.device->smu_messages.size() == before, "undersized driver table refused");

    Fixture optional;
    rx7900::memory::Handle only{};
    optional.manager->allocate(smu13_driver_table_bytes, 0x1000, rx7900::memory::Access::read_write,
                               false, only);
    optional.smu->start();
    optional.device->events.clear();
    expect(optional.smu->set_tables(*optional.manager, only, rx7900::memory::Handle{},
                                    rx7900_test::navi31_vram()) == SmuStatus::completed &&
           optional.device->events.size() == 2, "absent tool table skipped as in Linux");
}

void power_play() {
    Fixture fixture;
    rx7900::memory::Handle driver{};
    fixture.manager->allocate(smu13_driver_table_bytes, 0x1000, rx7900::memory::Access::read_write,
                              true, driver);
    fixture.smu->start();
    fixture.smu->set_tables(*fixture.manager, driver, rx7900::memory::Handle{},
                            rx7900_test::navi31_vram());
    std::vector<uint8_t> board(smu13_board_table_bytes);

    for (size_t index = 0; index < board.size(); ++index) {
        board.at(index) = static_cast<uint8_t>(0x80 + index);
    }

    expect(fixture.smu->setup_power_play(false, {board.data(), board.size() - 1}) ==
           SmuStatus::invalid_memory, "board table must be exactly BoardTable_t sized");
    const auto messages = fixture.device->smu_messages.size();
    expect(fixture.smu->setup_power_play(false, {board.data(), board.size()}) == SmuStatus::completed,
           "non-SCPM PPTable read, board append and write");
    const auto& sent = fixture.device->smu_messages;
    const auto& parameters = fixture.device->smu_parameters;
    expect(sent.size() == messages + 2 && sent.at(messages) == 0x12 && parameters.at(messages) == 1 &&
           sent.at(messages + 1) == 0x13 && parameters.at(messages + 1) == 0,
           "TransferTableSmu2Dram COMBO then TransferTableDram2Smu PPTABLE");
    const auto& uploaded = fixture.device->uploaded_pptable;
    expect(uploaded.size() == smu13_pptable_bytes && uploaded.at(0) == 0x0d &&
           uploaded.at(smu13_board_table_offset) == 0x80 &&
           uploaded.at(smu13_board_table_offset + smu13_board_table_bytes - 1) ==
               static_cast<uint8_t>(0x80 + smu13_board_table_bytes - 1) &&
           fixture.smu->report().platform_caps == 0x1234 && fixture.smu->report().board_table_appended,
           "uploaded PPTable_t keeps the PMFW SKU data and carries the ATOM BoardTable");

    expect(fixture.smu->run_btc() == SmuStatus::completed && sent.back() == 0x36, "RunDcBtc sent");
    expect(fixture.smu->set_allowed_features(false) == SmuStatus::completed &&
           sent.at(sent.size() - 2) == 0x5 && sent.back() == 0x4 &&
           fixture.smu->report().allowed_features == UINT64_MAX,
           "allowed mask high then low; GFXOFF allowed on PMFW 78.131");

    expect(fixture.smu->update_pcie({3, 6}) == SmuStatus::completed &&
           fixture.smu->report().pcie_levels == 3 && parameters.back() == ((2u << 16) | (3u << 8) | 6u) &&
           parameters.at(parameters.size() - 3) == 6u,
           "OverridePcieParameters per PPTable link level");
    expect(fixture.smu->update_pcie({2, 4}) == SmuStatus::completed &&
           parameters.back() == ((2u << 16) | (2u << 8) | 4u) && fixture.smu->report().pcie_gen[1] == 2,
           "link levels clamped to the negotiated link");
    expect(fixture.smu->update_pcie({4, 6}) == SmuStatus::unavailable, "invalid PCIe gen refused");

    // set_default_dpm_table: GFXCLK (feature 1) and UCLK (3) running, others boot values.
    fixture.device->running_features_low = (1u << 1) | (1u << 3);
    expect(fixture.smu->enable_all_features() == SmuStatus::completed, "feature mask re-read");
    const auto dpm_start = sent.size();
    expect(fixture.smu->set_default_dpm_tables({120000, 250000, 125000, 140000, 90000, 80000, 60000}) ==
           SmuStatus::completed && fixture.smu->report().dpm_tables_ready,
           "default DPM tables built");
    const auto first = fixture.smu->report();
    const auto& dpm = first.dpm;
    const auto& gfx = dpm[static_cast<size_t>(SmuClock::gfxclk)];
    const auto& uclk = dpm[static_cast<size_t>(SmuClock::uclk)];
    const auto& soc = dpm[static_cast<size_t>(SmuClock::socclk)];
    const auto& dcef = dpm[static_cast<size_t>(SmuClock::dcefclk)];
    std::vector<uint32_t> frequency_queries;
    bool mask_before_each = true;

    for (size_t index = dpm_start; index < sent.size(); ++index) {
        if (sent.at(index) == 0x1f) {
            frequency_queries.push_back(parameters.at(index));
            mask_before_each = mask_before_each && index >= 2 && sent.at(index - 2) == 0xd &&
                               sent.at(index - 1) == 0xc;
        }
    }

    const std::vector<uint32_t> expected_queries{0xff, 0xff, 0, 1, 2,
                                                 0x200ff, 0x200ff, 0x20000, 0x20001, 0x20002};
    expect(sent.size() == dpm_start + 44 && sent.at(dpm_start) == 0xd && sent.at(dpm_start + 1) == 0xc &&
           frequency_queries == expected_queries && mask_before_each,
           "per clock a High/Low mask read; GFXCLK (PPCLK 0) and UCLK (PPCLK 2) queried with a mask read before each");
    expect(gfx.from_firmware && gfx.fine_grained && gfx.count == 3 && gfx.levels[0] == 1000 &&
           gfx.min == 1000 && gfx.max == gfx.levels[2] && !uclk.fine_grained && uclk.max == 3200,
           "PMFW levels masked to 31 bits with the fine-grained bit separated");
    expect(soc.count == 1 && !soc.from_firmware && soc.levels[0] == 1200 && soc.max == 1200 &&
           dcef.levels[0] == 600, "disabled clocks get one boot level in MHz");

    // GameClockAc caps the reported top GFXCLK level.
    fixture.device->put_vram_u32(fixture.smu->report().driver_table_mc + 832 + 2780, 0x047e0000u);
    expect(fixture.smu->setup_power_play(false, {board.data(), board.size()}) == SmuStatus::completed &&
           fixture.smu->set_default_dpm_tables({}) == SmuStatus::completed &&
           fixture.smu->report().dpm[static_cast<size_t>(SmuClock::gfxclk)].levels[2] == 1150 &&
           fixture.smu->report().dpm[static_cast<size_t>(SmuClock::gfxclk)].max == 1150 &&
           fixture.smu->report().dpm[static_cast<size_t>(SmuClock::gfxclk)].levels[1] == 1100,
           "GFXCLK top level capped at DriverReportedClocks.GameClockAc");

    fixture.device->dpm_levels = 1;
    expect(fixture.smu->set_default_dpm_tables({}) == SmuStatus::completed &&
           fixture.smu->report().dpm[static_cast<size_t>(SmuClock::uclk)].count == 1 &&
           fixture.smu->report().dpm[static_cast<size_t>(SmuClock::uclk)].min == 3000 &&
           fixture.smu->report().dpm[static_cast<size_t>(SmuClock::uclk)].max == 0,
           "single PMFW level leaves max unset, as smu_v13_0_set_single_dpm_table does");
    fixture.device->dpm_levels = 17;
    expect(fixture.smu->set_default_dpm_tables({}) == SmuStatus::command_failed &&
           !fixture.smu->report().dpm_tables_ready, "more than MAX_DPM_LEVELS refused");
    fixture.device->dpm_levels = 3;
    fixture.device->running_features_low = 0x1234;
    fixture.smu->enable_all_features();

    const auto binary = rx7900_test::navi31_discovery();
    ThmRegisters thm;
    expect(validate_thm_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes, thm) ==
           DiscoveryStatus::validated && thm.segment0_dword_base() == rx7900_test::thm_base,
           "THM 13.0.3 instance0 validated");
    expect(fixture.smu->enable_thermal_alert(ThmRegisters{}) == SmuStatus::unavailable,
           "thermal alert needs a THM token");
    auto& device = *fixture.device;
    device.reg(rx7900_test::thm_base + 0xb) = 0x4fffffffu;
    device.reg(rx7900_test::mp_base + 0x2c2) = 0x1ffu;
    device.reg(rx7900_test::mp_base + 0x2c3) = 0x3u;
    const auto quiet = sent.size();
    expect(fixture.smu->enable_thermal_alert(thm) == SmuStatus::completed &&
           fixture.smu->report().software_shutdown_temp == 110 &&
           fixture.smu->report().thermal_alert_high == 110 && fixture.smu->report().thermal_alert_low == 0,
           "alert range [0, software_shutdown_temp]");
    expect(device.reg(rx7900_test::thm_base + 0xb) == 0xb8ff006eu &&
           device.reg(rx7900_test::thm_base + 0xa) == 0x38u,
           "THM: 5 IH credits, HW IH enable, INTH/INTL unmasked, trigger mask cleared, clears latched");
    expect(device.reg(rx7900_test::mp_base + 0x2c2) == 0xfeu && device.reg(rx7900_test::mp_base + 0x2c3) == 2u,
           "MP1 SW interrupt ID 0xFE, not valid, unmasked");
    expect(sent.size() == quiet && !fixture.smu->report().ih_host_interrupt_allowed,
           "no AllowIHHostInterrupt without the ACDC feature");
    device.running_features_low = 1u << 27;
    expect(fixture.smu->enable_all_features() == SmuStatus::completed &&
           fixture.smu->enable_thermal_alert(thm) == SmuStatus::completed && sent.back() == 0x4c &&
           fixture.smu->report().ih_host_interrupt_allowed,
           "HARDWAREDC with ACDC running sends AllowIHHostInterrupt");
    const auto missing = rx7900_test::discovery_with({{discovery_thm_hardware_id, 13, 0, {0x16600}}});
    expect(validate_thm_discovery({missing.data(), missing.size()}, rx7900_test::aperture_bytes, thm) ==
           DiscoveryStatus::unsupported && !thm.valid(), "THM revision other than 13.0.3 refused");

    Fixture scpm;
    rx7900::memory::Handle scpm_driver{};
    scpm.manager->allocate(smu13_driver_table_bytes, 0x1000, rx7900::memory::Access::read_write,
                           true, scpm_driver);
    scpm.smu->start();
    scpm.smu->set_tables(*scpm.manager, scpm_driver, rx7900::memory::Handle{},
                         rx7900_test::navi31_vram());
    const auto before = scpm.device->smu_messages.size();
    expect(scpm.smu->setup_power_play(true, {nullptr, 0}) == SmuStatus::completed &&
           scpm.smu->set_allowed_features(true) == SmuStatus::completed &&
           scpm.device->smu_messages.size() == before + 1 && scpm.device->uploaded_pptable.empty(),
           "SCPM: PPTable read only; no write or allowed mask");

    Smu13 legacy;
    expect(legacy.setup_power_play(true, {nullptr, 0}) == SmuStatus::unavailable,
           "PPTable setup needs located tables");
}


// Wraps the shared model's RegisterIo to add the late-init PMFW responses it does
// not model: combo-PPTable power fields, GetPptLimit, metrics serial numbers and
// an overridable GetSmuVersion answer.
struct PmfwExtras final {
    SimulatedNavi31* device = nullptr;
    uint32_t platform_caps = 0;
    uint32_t ppt_limit = 330;
    uint32_t smu_version = 0x004e8300;
    unsigned overdrive_transfers = 0;

    static bool read(void* context, uint32_t offset, uint32_t& value) noexcept {
        return SimulatedNavi31::read(static_cast<PmfwExtras*>(context)->device, offset, value);
    }

    static bool write(void* context, uint32_t offset, uint32_t value) noexcept {
        auto& self = *static_cast<PmfwExtras*>(context);

        if (!SimulatedNavi31::write(self.device, offset, value)) {
            return false;
        }

        if (offset == (rx7900_test::mp_base + 0x282) * 4) {
            self.answer(value);
        }

        return true;
    }

    static void delay(void* context, uint32_t microseconds) noexcept {
        SimulatedNavi31::delay(static_cast<PmfwExtras*>(context)->device, microseconds);
    }

    rx7900::hardware::RegisterIo io() noexcept {
        return {this, rx7900_test::aperture_bytes, read, write, delay};
    }

    void answer(uint32_t message) {
        auto& model = *device;
        const uint32_t parameter = model.smu_parameters.back();
        const uint64_t table = model.smu_driver_table;

        if (message == 0x2) {
            model.reg(rx7900_test::mp_base + 0x292) = smu_version;
        } else if (message == 0x33 && message != model.smu_fail_message) {
            model.reg(rx7900_test::mp_base + 0x292) = ppt_limit;
        } else if (message == 0x12 && parameter == 1) {
            // platform_caps (u32 at 18) and overdrive_table.min[POWERPERCENTAGE] = 10 (527).
            model.put_vram_u32(table + 16, (platform_caps & 0xffffu) << 16);
            model.put_vram_u32(table + 20, platform_caps >> 16);
            model.put_vram_u32(table + 527, 10);
            // SkuTable: SocketPowerLimitAc[0] 355 (16), SocketPowerLimitDc[0] 300 (24),
            // BaseClockAc 1100 / GameClockAc 1150 (2780), MsgLimits.Power[0][AC] 402
            // (2808), OverDriveLimitsBasicMax.FeatureCtrlMask with PPT bit 2 (3016).
            model.put_vram_u32(table + 832 + 16, 355);
            model.put_vram_u32(table + 832 + 24, 300);
            model.put_vram_u32(table + 832 + 2780, (1150u << 16) | 1100u);
            model.put_vram_u32(table + 832 + 2808, 402);
            model.put_vram_u32(table + 832 + 3016, 1u << 2);
        } else if (message == 0x12 && parameter == 8) {
            ++overdrive_transfers;
        } else if (message == 0x12 && parameter == 5) {
            model.put_vram_u32(table + 236, 0x89abcdefu);
            model.put_vram_u32(table + 240, 0x01234567u);
        }
    }
};

// Fixture through smu_hw_init: tables, PPTable, BTC, features, DPM, PCIe, thermal.
struct HardwareInit final {
    Fixture fixture;
    PmfwExtras extras;
    ThmRegisters thm{};
    std::vector<uint8_t> board = std::vector<uint8_t>(smu13_board_table_bytes, 0x5a);
    rx7900::memory::Handle driver{};
    bool ready = false;

    explicit HardwareInit(uint32_t caps, uint32_t features_low) {
        extras.device = fixture.device.get();
        extras.platform_caps = caps;
        fixture.smu->initialize(fixture.registers, extras.io(), fixture.nbio, {100, 3},
                                {fixture.device.get(), SimulatedNavi31::confirm});
        fixture.manager->allocate(smu13_driver_table_bytes, 0x1000,
                                  rx7900::memory::Access::read_write, true, driver);
        const auto binary = rx7900_test::navi31_discovery();
        validate_thm_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes, thm);
        fixture.device->running_features_low = features_low;
        fixture.device->reg(rx7900_test::thm_base + 0xb) = 0x4fffffffu;
        fixture.device->reg(rx7900_test::mp_base + 0x2c2) = 0x1ffu;
        fixture.device->reg(rx7900_test::mp_base + 0x2c3) = 0x3u;
        auto& smu = *fixture.smu;
        ready = smu.start() == SmuStatus::ready &&
                smu.set_tables(*fixture.manager, driver, rx7900::memory::Handle{},
                               rx7900_test::navi31_vram()) == SmuStatus::completed &&
                smu.setup_power_play(false, {board.data(), board.size()}) == SmuStatus::completed &&
                smu.run_btc() == SmuStatus::completed &&
                smu.set_allowed_features(false) == SmuStatus::completed &&
                smu.enable_all_features() == SmuStatus::completed &&
                smu.set_default_dpm_tables({120000, 250000, 125000, 140000, 90000, 80000, 60000}) ==
                    SmuStatus::completed &&
                smu.update_pcie({3, 6}) == SmuStatus::completed &&
                smu.enable_thermal_alert(thm) == SmuStatus::completed;
    }

    size_t sent() const { return fixture.device->smu_messages.size(); }

    bool sequence(size_t start, std::initializer_list<std::pair<uint32_t, uint32_t>> expected) const {
        const auto& messages = fixture.device->smu_messages;
        const auto& parameters = fixture.device->smu_parameters;

        if (messages.size() != start + expected.size()) {
            return false;
        }

        size_t index = start;

        for (const auto& item : expected) {
            if (messages.at(index) != item.first || parameters.at(index) != item.second) {
                return false;
            }

            ++index;
        }

        return true;
    }
};

constexpr uint32_t gfx_uclk_throttlers = (1u << 1) | (1u << 3) | (1u << 24);

void firmware_image_version() {
    std::vector<uint8_t> file(64, 0);
    rx7900::sdma::CommandBuffer(file.data(), file.size()).write_u32(0, 64);
    rx7900::sdma::CommandBuffer(file.data(), file.size()).write_u32(16, 0x004e8300u);
    uint32_t version = 0;
    expect(smu13_firmware_image_version({file.data(), file.size()}, version) && version == 0x004e8300u,
           "adev->pm.fw_version from common_firmware_header.ucode_version");
    version = 7;
    expect(!smu13_firmware_image_version({file.data(), 48}, version) && version == 7 &&
           !smu13_firmware_image_version({file.data(), 16}, version),
           "SMC image with mismatched size or truncated header refused");
}

void late_init_default_policy() {
    // No HARDWAREDC cap, AC, THROTTLERS running, PMFW image 78.131.
    HardwareInit init(0, gfx_uclk_throttlers);
    expect(init.ready, "hw_init sequence completes before late_init");
    auto& smu = *init.fixture.smu;
    expect(smu.report().dpm[static_cast<size_t>(SmuClock::gfxclk)].max == 1150,
           "GameClockAc 1150 caps GFXCLK max in the default DPM table");
    const auto start = init.sent();
    expect(smu.late_init({true, 0x004e8300u}) == SmuStatus::completed, "late_init completed");
    expect(init.sequence(start, {{0x35, 0}, {0x12, 8}, {0xd, 0}, {0xc, 0}, {0x33, 0}, {0x12, 5},
                                 {0x24, 0x6}}),
           "NotifyPowerSource(AC), Smu2Dram OVERDRIVE, running features high/low, "
           "GetPptLimit(AC<<16), Smu2Dram SMU_METRICS, SetWorkloadMask(3D|POWER_SAVING)");
    const auto report = smu.report();
    expect(report.late_initialized && report.power_source_notified && report.overdrive_table_loaded &&
           init.extras.overdrive_transfers == 1, "late-init state recorded");
    expect(report.power_limits.from_firmware && report.power_limits.current_limit == 330 &&
           report.power_limits.default_limit == 330 && report.power_limits.max_limit == 402 &&
           report.power_limits.min_limit == 297 && report.od_ppt_supported,
           "power limits: GetPptLimit 330 W, max MsgLimits 402 W, min 330*(100-10)/100");
    expect(report.pstate.gfxclk.min == 1000 && report.pstate.gfxclk.peak == 1150 &&
           report.pstate.gfxclk.standard == 1100 && report.pstate.uclk.min == 3000 &&
           report.pstate.uclk.peak == 3200 && report.pstate.uclk.standard == 3200 &&
           report.pstate.socclk.min == 1200 && report.pstate.socclk.standard == 1200 &&
           report.pstate.vclk.peak == 900 && report.pstate.dclk.standard == 800 &&
           report.pstate.fclk.min == 1400,
           "UMD pstate: BaseClockAc standard, GFX max peak, boot-level clocks");
    expect(report.unique_id == 0x0123456789abcdefULL, "unique id from metrics serial numbers");
    expect(report.power_profile_mode == 1 && report.workload_mask == 0x6 && report.workload_applied,
           "FULLSCREEN3D profile applied once");
    const auto after = init.sent();
    expect(smu.late_init({true, 0x004e8300u}) == SmuStatus::unavailable && init.sent() == after,
           "late_init runs once");
}

void late_init_hardware_dc_and_fallbacks() {
    // HARDWAREDC: no NotifyPowerSource. THROTTLERS off: SocketPowerLimitDc fallback.
    HardwareInit gpio(0x4, (1u << 1) | (1u << 3));
    auto start = gpio.sent();
    expect(gpio.ready && gpio.fixture.smu->late_init({false, 0x004e7200u}) == SmuStatus::completed &&
           gpio.sequence(start, {{0x12, 8}, {0xd, 0}, {0xc, 0}, {0x12, 5}, {0x24, 0x2}}),
           "HARDWAREDC skips NotifyPowerSource; no GetPptLimit; image 78.114 sends only 3D");
    auto report = gpio.fixture.smu->report();
    expect(!report.power_source_notified && !report.power_limits.from_firmware &&
           report.power_limits.current_limit == 300 && report.power_limits.min_limit == 270 &&
           report.power_limits.max_limit == 402, "DC SocketPowerLimitDc fallback");

    HardwareInit dc(0, gfx_uclk_throttlers);
    start = dc.sent();
    dc.fixture.device->smu_fail_message = 0x33;
    expect(dc.ready && dc.fixture.smu->late_init({false, 0x004e6601u}) == SmuStatus::completed &&
           dc.sequence(start, {{0x35, 1}, {0x12, 8}, {0xd, 0}, {0xc, 0}, {0x33, 1u << 16},
                               {0x12, 5}, {0x24, 0x6}}),
           "DC: NotifyPowerSource(1), GetPptLimit(DC<<16); image 0x004e6601 adds power saving");
    report = dc.fixture.smu->report();
    expect(!report.power_limits.from_firmware && report.power_limits.current_limit == 300 &&
           report.late_initialized, "rejected GetPptLimit falls back as Linux does");

    HardwareInit workload(0, gfx_uclk_throttlers);
    workload.fixture.device->smu_fail_message = 0x24;
    expect(workload.ready && workload.fixture.smu->late_init({true, 0x004e8300u}) == SmuStatus::completed &&
           !workload.fixture.smu->report().workload_applied,
           "rejected SetWorkloadMask ignored as smu_late_init ignores smu_handle_task");

    HardwareInit od(0, gfx_uclk_throttlers);
    start = od.sent();
    od.fixture.device->smu_fail_message = 0x12;
    expect(od.ready && od.fixture.smu->late_init({true, 0x004e8300u}) == SmuStatus::command_failed &&
           od.sequence(start, {{0x35, 0}, {0x12, 8}}) && !od.fixture.smu->report().late_initialized,
           "OD table transfer failure fails late_init");

    HardwareInit hang(0, gfx_uclk_throttlers);
    start = hang.sent();
    hang.fixture.device->smu_response = 0xfb;
    expect(hang.ready && hang.fixture.smu->late_init({true, 0x004e8300u}) == SmuStatus::hung &&
           hang.sent() == start + 1, "hung PMFW stops late_init at the first message");

    HardwareInit busy(0, gfx_uclk_throttlers);
    busy.fixture.device->smu_fail_message = 0xd;
    expect(busy.ready && busy.fixture.smu->late_init({true, 0x004e8300u}) == SmuStatus::completed &&
           !busy.fixture.smu->report().power_limits.from_firmware,
           "failed running-feature query treated as disabled (smu_cmn_feature_is_enabled)");

    Fixture early;
    early.smu->start();
    expect(early.smu->late_init({true, 0}) == SmuStatus::unavailable,
           "late_init requires the hw_init DPM tables");
}

void hardware_fini() {
    HardwareInit init(0, gfx_uclk_throttlers);
    auto& device = *init.fixture.device;
    auto& smu = *init.fixture.smu;
    expect(init.ready && device.reg(rx7900_test::thm_base + 0xb) == 0xb8ff006eu &&
           device.reg(rx7900_test::mp_base + 0x2c3) == 2u, "thermal alert enabled before fini");
    smu.late_init({true, 0x004e8300u});
    const auto start = init.sent();
    expect(smu.hw_fini() == SmuStatus::completed && init.sent() == start,
           "13.0.0 hw_fini sends no message (smu_disable_dpms returns early)");
    expect(device.reg(rx7900_test::thm_base + 0xb) == 0xbbff006eu &&
           device.reg(rx7900_test::thm_base + 0xa) == 0 &&
           device.reg(rx7900_test::mp_base + 0x2c3) == 3u,
           "set_irq_state DISABLE: INTH/INTL masked, INT_ENA cleared, MP1 SW interrupt masked");
    const auto report = smu.report();
    expect(report.hardware_finished && !report.thermal_alert_enabled && report.tables_retained,
           "teardown recorded; tables stay retained");
    expect(smu.hw_fini() == SmuStatus::unavailable && smu.late_init({true, 0}) == SmuStatus::unavailable,
           "fini runs once and late_init is refused afterwards");

    Fixture never;
    never.smu->start();
    never.device->reg(rx7900_test::thm_base + 0xb) = 0x12345678u;
    expect(never.smu->hw_fini() == SmuStatus::unavailable &&
           never.device->reg(rx7900_test::thm_base + 0xb) == 0x12345678u,
           "never-enabled alert: amdgpu_irq_put -EINVAL, no register writes");

    HardwareInit broken(0, gfx_uclk_throttlers);
    broken.fixture.device->reads_ok = false;
    expect(broken.ready && broken.fixture.smu->hw_fini() == SmuStatus::mmio_failure &&
           !broken.fixture.smu->report().hardware_finished, "THM read failure fails closed");
}

void explicit_mode1_reset() {
    HardwareInit init(0, gfx_uclk_throttlers);
    auto& device = *init.fixture.device;
    auto& smu = *init.fixture.smu;
    expect(init.ready && smu.hw_fini() == SmuStatus::completed, "cleanup precedes reset as in Linux");
    const auto start = init.sent();
    const auto delays = device.delays.size();
    expect(smu.mode1_reset() == SmuStatus::completed && init.sequence(start, {{0x2f, 0}}) &&
           device.delays.size() == delays + 1 && device.delays.back() == 500000,
           "Mode1Reset(0) then SMU13_MODE1_RESET_WAIT_TIME_IN_MS");
    expect(smu.report().mode1_reset_issued && smu.send(smu13_message::test, 0, nullptr) ==
           SmuStatus::quarantined && init.sent() == start + 1,
           "messages refused after the reset");
    expect(smu.mode1_reset() == SmuStatus::unavailable &&
           smu.initialize(init.fixture.registers, init.extras.io(), init.fixture.nbio, {100, 3},
                          {init.fixture.device.get(), SimulatedNavi31::confirm}) == SmuStatus::busy,
           "no second reset or re-initialization before release");
    expect(smu.release_after_reset() == SmuStatus::quarantined, "release still needs reset proof");
    device.reset_confirmed = true;
    expect(smu.release_after_reset() == SmuStatus::released &&
           !smu.report().tables_retained, "confirmed reset releases the tables");

    HardwareInit old(0, gfx_uclk_throttlers);
    old.extras.smu_version = 0x004e2800;
    old.fixture.smu->start();
    const auto before = old.sent();
    expect(old.fixture.smu->mode1_reset() == SmuStatus::unsupported_firmware && old.sent() == before,
           "PMFW older than 78.41 refuses mode-1 reset without a message");

    HardwareInit rejected(0, gfx_uclk_throttlers);
    rejected.fixture.device->smu_fail_message = 0x2f;
    const auto waits = rejected.fixture.device->delays.size();
    expect(rejected.ready && rejected.fixture.smu->mode1_reset() == SmuStatus::command_failed &&
           rejected.fixture.device->delays.size() == waits &&
           rejected.fixture.smu->send(smu13_message::test, 0, nullptr) == SmuStatus::quarantined,
           "rejected Mode1Reset skips the wait but the written reset still quarantines");

    Fixture unstarted;
    expect(unstarted.smu->mode1_reset() == SmuStatus::unavailable && unstarted.device->smu_messages.empty(),
           "mode-1 reset needs the cached PMFW version");
}

// The interrupt-action messages (gpu_health): thermal metrics decode, the
// GFXCLK soft maximum, THM DIG_THERM_INTH updates and the AC/DC acknowledgement.
void thermal_actions() {
    Fixture bare;
    SmuThermalMetrics metrics{};
    expect(bare.smu->start() == SmuStatus::ready && bare.smu->read_thermal_metrics(metrics) ==
               SmuStatus::unavailable && bare.smu->limit_gfxclk(true) == SmuStatus::unavailable &&
               bare.smu->set_thermal_alert_high(90) == SmuStatus::unavailable &&
               bare.smu->thermal_limits().software_shutdown_c == 0,
           "no tables, DPM tables or thermal alert yet: every action refuses without a message");

    HardwareInit init(0, gfx_uclk_throttlers);
    auto& device = *init.fixture.device;
    auto& smu = *init.fixture.smu;
    const uint64_t table = device.smu_driver_table;
    const auto start = init.sent();
    expect(init.ready && smu.read_thermal_metrics(metrics) == SmuStatus::completed &&
           init.sequence(start, {{0x12, 5}}), "TransferTableSmu2Dram(SMU_METRICS)");
    // AvgTemperature[EDGE/HOTSPOT/MEM] at 120/122/128, ThrottlingPercentage[] at 154.
    device.put_vram_u32(table + 120, (88u << 16) | 61u);
    device.put_vram_u32(table + 128, 72u);
    for (uint32_t offset = 152; offset < 180; offset += 4) device.put_vram_u32(table + offset, 0);
    device.put_vram_u32(table + 152, 0x00030000u);      // ThrottlingPercentage[0] = 3
    device.put_vram_u32(table + 172, 0x00000900u);      // ThrottlingPercentage[19] = 9
    // The model's SMU_METRICS answer leaves these bytes in the table.
    expect(smu.read_thermal_metrics(metrics) == SmuStatus::completed && metrics.edge_c == 61 &&
           metrics.hotspot_c == 88 && metrics.memory_c == 72 &&
           metrics.throttler_status == ((1u << 0) | (1u << 19)),
           "temperatures and throttler status bits decoded from the measured offsets");
    const auto before = init.sent();
    const SmuDpmTable gfx = smu.report().dpm[static_cast<size_t>(SmuClock::gfxclk)];
    expect(smu.limit_gfxclk(true) == SmuStatus::completed && init.sequence(before, {{0x1a, gfx.min}}) &&
           smu.thermal_state().gfxclk_limited && smu.thermal_state().gfxclk_soft_max_mhz == gfx.min,
           "SetSoftMaxByFreq((PPCLK_GFXCLK 0 << 16) | lowest level)");
    expect(smu.limit_gfxclk(false) == SmuStatus::completed && init.sequence(before, {{0x1a, gfx.min}, {0x1a, gfx.max}}) &&
           !smu.thermal_state().gfxclk_limited, "and back to the highest level");
    device.reg(rx7900_test::thm_base + 0xb) |= 0x04000000u;     // THERM_TRIGGER_MASK set
    const uint32_t control = device.reg(rx7900_test::thm_base + 0xb);
    expect(smu.set_thermal_alert_high(300) == SmuStatus::completed &&
           device.reg(rx7900_test::thm_base + 0xb) == ((control & ~0x040000ffu) | 0xffu) &&
           smu.thermal_state().alert_high_c == 255, "INTH clamps to SMU_THERMAL_MAXIMUM_ALERT_TEMP, trigger mask cleared");
    expect(smu.acknowledge_ac_dc() == SmuStatus::completed && init.sequence(before,
           {{0x1a, gfx.min}, {0x1a, gfx.max}, {0x34, 0}}) && smu.report().ac_dc_acknowledged == 1,
           "ReenableAcDcInterrupt");
    device.smu_fail_message = 0x1a;
    expect(smu.limit_gfxclk(true) == SmuStatus::command_failed && !smu.thermal_state().gfxclk_limited,
           "a refused message records no limit");
}

} // namespace

int main() {
    thermal_actions();
    power_play();
    discovery();
    startup_and_messages();
    first_message_and_firmware();
    tables_and_features();
    firmware_image_version();
    late_init_default_policy();
    late_init_hardware_dc_and_fallbacks();
    hardware_fini();
    explicit_mode1_reset();
    return checks.finish("smu13");
}

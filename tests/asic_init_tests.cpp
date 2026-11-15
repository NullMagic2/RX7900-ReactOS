/*
Module: rx7900.tests.asic_init
Purpose: Verify the complete Linux-ordered Navi31 firmware-loading sequence.
Created: 2026-10-21
Architecture: Synthetic register/VRAM machine and synthetic firmware headers;
it proves ordering and ownership logic only, never physical GPU behavior.
*/
#include <array>
#include <vector>
#include <memory>

#include "asic_test_device.hpp"
#include "mes_test_model.hpp"
#include "ucode_test_files.hpp"
#include "atom_asic_init_rom.hpp"

namespace {

using namespace rx7900::hardware;
using namespace rx7900_test::ucode_files;
using rx7900_test::Event;
using rx7900_test::SimulatedNavi31;

rx7900_test::Checks checks;

void expect(bool condition, const char* contract) { checks.expect(condition, contract); }

std::vector<uint8_t> sos_container() {
    const std::array<uint32_t, 6> identifiers{2, 1, 4, 3, 5, 6};
    constexpr uint32_t part_bytes = 64;
    std::vector<uint8_t> bytes(256 + identifiers.size() * part_bytes);
    const rx7900::sdma::CommandBuffer output(bytes.data(), bytes.size());
    output.write_u32(0, static_cast<uint32_t>(bytes.size()));
    output.write_u32(4, 36);
    bytes.at(8) = 2;
    bytes.at(12) = 13;
    output.write_u32(16, 0x310035);
    output.write_u32(20, static_cast<uint32_t>(identifiers.size() * part_bytes));
    output.write_u32(24, 256);
    output.write_u32(32, static_cast<uint32_t>(identifiers.size()));

    for (size_t index = 0; index < identifiers.size(); ++index) {
        output.write_u32(36 + index * 16, identifiers.at(index));
        output.write_u32(40 + index * 16, 0x310035);
        output.write_u32(44 + index * 16, static_cast<uint32_t>(index * part_bytes));
        output.write_u32(48 + index * 16, part_bytes);

        for (uint32_t word = 0; word < part_bytes / 4; ++word) {
            output.write_u32(256 + index * part_bytes + word * 4,
                             0x50000000u + identifiers.at(index) * 0x100 + word);
        }
    }

    return bytes;
}

struct ResetLog final {
    std::vector<int> calls;
    bool restore_ok = true;
    static bool cache(void* context) noexcept { static_cast<ResetLog*>(context)->calls.push_back(1); return true; }
    static bool master(void* context) noexcept { static_cast<ResetLog*>(context)->calls.push_back(2); return true; }
    static bool restore(void* context) noexcept {
        auto& self = *static_cast<ResetLog*>(context);
        self.calls.push_back(3);
        return self.restore_ok;
    }
    ResetPlatform platform() noexcept { return {this, cache, master, restore}; }
};

// FencePlatform on the host: time advances by every block timeout.
struct HostFences final {
    uint64_t now = 0;
    static void lock(void*) noexcept {}
    static void unlock(void*) noexcept {}
    static uint64_t now_us(void* context) noexcept { return static_cast<HostFences*>(context)->now += 1000; }
    static void signal(void*, uint32_t) noexcept {}
    static void reset(void*, uint32_t) noexcept {}
    static bool block(void* context, uint32_t, uint64_t timeout) noexcept {
        static_cast<HostFences*>(context)->now += timeout;
        return false;
    }
    static void vm_fault(void*, const IrqVmFault&, uint32_t) noexcept {}
    static void hang(void*, uint32_t, FenceHangReason, uint32_t, uint32_t) noexcept {}
    static bool store(void*, uint8_t* address, uint64_t value) noexcept {
        const rx7900::sdma::CommandBuffer out(address, 8);
        return out.write_u32(0, static_cast<uint32_t>(value)) && out.write_u32(4, static_cast<uint32_t>(value >> 32));
    }
};

struct Fixture final {
    std::unique_ptr<SimulatedNavi31> device = std::make_unique<SimulatedNavi31>();
    // MES scheduler/KIQ firmware over the same machine; SDMA doorbells pass through.
    rx7900_test::mes_model::MesModel mes_firmware{device.get()};
    rx7900_test::VramBackend backend{};
    std::unique_ptr<rx7900::memory::Manager> manager = std::make_unique<rx7900::memory::Manager>();
    std::unique_ptr<UcodeCatalog> catalog = std::make_unique<UcodeCatalog>();
    std::unique_ptr<AsicInit> asic = std::make_unique<AsicInit>();
    std::vector<uint8_t> discovery = rx7900_test::navi31_discovery();
    std::vector<uint8_t> sos = sos_container();
    File pfp = gfx(0x40, 0x20);
    File me = gfx(0x40, 0x20);
    File mec = gfx(0x80, 0x40);
    File rlc_file = rlc(3);
    File mes_file = mes();
    File kiq = mes();
    File imu_file = imu();
    File smu_file = smu();
    File sdma_file = sdma();
    File dmcub_file = dmcub();
    File ta_file = ta_navi31();
    AsicInitPolicy policy{{1000, 1000}, {1000, 50, 10}, {100, 3}, {1000, 2100000}, true};
    std::vector<uint8_t> board = std::vector<uint8_t>(smu13_board_table_bytes, 0x5a);
    SmuPcieLink pcie{3, 6};
    AtomFirmwareInfo atom_info{};
    const AtomFirmwareInfo* atom = nullptr;
    bool training_callbacks = true;
    // InterruptReady: called once, right after IH6 is enabled and before PSP firmware.
    unsigned ih_ready_calls = 0;
    size_t psp_commands_at_ready = SIZE_MAX;
    bool ih_enabled_at_ready = false;
    bool ih_ready_result = true;
    static bool on_ih_ready(void* context, Ih6& ih) noexcept {
        auto& self = *static_cast<Fixture*>(context);
        ++self.ih_ready_calls;
        self.ih_enabled_at_ready = ih.report().status == Ih6Status::enabled;
        self.psp_commands_at_ready = self.device->commands.size();
        return self.ih_ready_result;
    }
    InterruptReady interrupt_hook{this, on_ih_ready};

    explicit Fixture(bool include_imu = true) {
        backend.device = device.get();
        // mes_firmware_header_v1_0 mes_uc_start_addr as in gc_11_0_0_mes_2.bin/mes1.bin.
        mes_file.u32(56, 0xf0005000u);
        mes_file.u32(60, 0xffffffffu);
        kiq.u32(56, 0xf0005000u);
        kiq.u32(60, 0xffffffffu);
        manager->initialize({rx7900_test::fb_base + rx7900_test::arena_offset,
                             rx7900_test::arena_bytes}, backend.backend());
        catalog->add(UcodeSource::pfp, pfp.view());
        catalog->add(UcodeSource::me, me.view());
        catalog->add(UcodeSource::mec, mec.view());
        catalog->add(UcodeSource::rlc, rlc_file.view());
        catalog->add(UcodeSource::mes, mes_file.view());
        catalog->add(UcodeSource::mes_kiq, kiq.view());
        catalog->add(UcodeSource::smu, smu_file.view());
        catalog->add(UcodeSource::sdma, sdma_file.view());
        catalog->add(UcodeSource::dmcub, dmcub_file.view());

        if (include_imu) {
            catalog->add(UcodeSource::imu, imu_file.view());
        }
    }

    AsicInputs inputs() {
        return {mes_firmware.io(), {discovery.data(), discovery.size()}, {sos.data(), sos.size()},
                catalog.get(), ta_file.view(), manager.get(), atom,
                training_callbacks ? device->training_io() : PspTrainingIo{},
                {board.data(), board.size()}, pcie, 0x0000000012345000ULL,
                {device.get(), SimulatedNavi31::bus_master, nullptr, SimulatedNavi31::confirm}, false,
                0x000000e000000000ULL, mes_firmware.doorbells(), device->gtt(), true, interrupt_hook};
    }

    AsicStatus run() {
        return asic->run(inputs(), policy, {device.get(), SimulatedNavi31::confirm});
    }

    std::vector<Event> of(Event::Kind kind) const {
        std::vector<Event> output;

        for (const auto& event : device->events) {
            if (event.kind == kind) {
                output.push_back(event);
            }
        }

        return output;
    }

    size_t first(Event::Kind kind) const {
        for (size_t index = 0; index < device->events.size(); ++index) {
            if (device->events.at(index).kind == kind) {
                return index;
            }
        }

        return SIZE_MAX;
    }
};

void complete_sequence() {
    Fixture fixture;
    expect(fixture.run() == AsicStatus::completed, "complete synthetic Navi31 sequence");
    const auto report = fixture.asic->report();
    expect(report.step == AsicStep::complete && !report.sos_was_running &&
           report.rlc_autoload_started && report.asd_session == 0x77,
           "report covers SOS boot, RLC autoload and ASD session");
    expect(fixture.first(Event::Kind::hdp_remap) < fixture.first(Event::Kind::psp_boot) &&
           fixture.first(Event::Kind::strap) < fixture.first(Event::Kind::psp_boot),
           "NBIO strap and HDP remap precede PSP access");

    const auto boot = fixture.of(Event::Kind::psp_boot);
    expect(boot.size() == 4 && boot.at(0).value == 0x80000 && boot.at(1).value == 0x10000000 &&
           boot.at(2).value == 0x10000 && boot.at(3).value == 0x20000,
           "bootloader order KDB, SPL, SYS, SOS");
    expect(fixture.first(Event::Kind::ring_create) > fixture.first(Event::Kind::psp_boot),
           "KM ring created after SOS");

    const auto commands = fixture.of(Event::Kind::psp_command);
    // ...ASD, RL, then LOAD_TA for RAS, HDCP and DTM (psp_load_fw order).
    const std::array<std::pair<uint32_t, uint32_t>, 36> expected{{
        {0x20, 0}, {6, 18}, {5, 0},
        {6, 71}, {6, 72}, {6, 87}, {6, 88}, {6, 89}, {6, 90}, {6, 91}, {6, 92}, {6, 93},
        {6, 94}, {6, 95}, {6, 96}, {6, 97}, {6, 33}, {6, 34}, {6, 81}, {6, 82}, {6, 68},
        {6, 69}, {6, 22}, {6, 21}, {6, 26}, {6, 48}, {6, 25}, {6, 7}, {6, 8}, {0x21, 0},
        {6, 51}, {4, 0}, {6, 67}, {1, 0}, {1, 0}, {1, 0}}};
    bool ordered = commands.size() == expected.size();

    for (size_t index = 0; ordered && index < expected.size(); ++index) {
        ordered = commands.at(index).value == expected.at(index).first &&
                  commands.at(index).detail == expected.at(index).second;
    }

    expect(ordered, "TOC, PMFW, SETUP_TMR, IP firmware, RLC autoload after RLC_G, DMCUB, ASD, RL");
    expect(report.tmr_bytes == 0x4f2000 && fixture.device->commands.at(2).words[2] == 0x4f2000,
           "TOC TMR size rounded to whole pages");
    expect(report.loaded_ucode == 27, "every non-SMC catalog payload loaded once");

    const auto& messages = fixture.device->smu_messages;
    // Enabled-mask reads are High (0xd) then Low (0xc). set_default_dpm_table
    // re-reads the mask per clock and before every GetDpmFreqByIndex of a clock
    // with a DPM feature; SOCCLK and FCLK run here (count, fine-grain, 3 levels).
    std::vector<uint32_t> smu_order{0x3, 0x2, 0xe, 0xf, 0x10, 0x11, 0x12, 0x36, 0x6, 0xd, 0xc};
    const bool running[7]{true, false, false, true, false, false, false};

    for (const bool on : running) {
        smu_order.insert(smu_order.end(), {0xd, 0xc});

        for (int query = 0; on && query < 5; ++query) {
            smu_order.insert(smu_order.end(), {0xd, 0xc, 0x1f});
        }
    }

    smu_order.insert(smu_order.end(), {0x20, 0x20, 0x20});
    // smu_late_init: HARDWAREDC skips NotifyPowerSource; OVERDRIVE table read,
    // feature mask (THROTTLERS off, so no GetPptLimit), SMU_METRICS, SetWorkloadMask.
    smu_order.insert(smu_order.end(), {0x12, 0xd, 0xc, 0x12, 0x24});
    // set_pg_state(GATE): amdgpu_gfx_off_ctrl -> AllowGfxOff after the 100 ms delay.
    smu_order.push_back(0x28);
    bool smu_ordered = messages.size() == smu_order.size();

    for (size_t index = 0; smu_ordered && index < smu_order.size(); ++index) {
        smu_ordered = messages.at(index) == smu_order.at(index);
    }

    expect(smu_ordered && fixture.first(Event::Kind::smu_message) >
           fixture.device->events.size() - smu_order.size() - 1,
           "SCPM SMU setup: versions, tables, PPTable read, BTC, features, DPM tables, PCIe after PSP");
    expect(fixture.asic->smu_report().enabled_features == 0x0000000500001234ULL &&
           fixture.asic->smu_report().pcie_levels == 3, "enabled features and PCIe levels reported");
    expect(fixture.asic->smu_report().thermal_alert_enabled &&
           fixture.asic->smu_report().thermal_alert_high == 110 &&
           (fixture.device->reg(rx7900_test::thm_base + 0xb) & 0x10000000u) != 0,
           "thermal alert enabled after the PCIe override");
    const auto& dpm = fixture.asic->smu_report().dpm;
    expect(fixture.asic->smu_report().dpm_tables_ready &&
           dpm[static_cast<size_t>(SmuClock::socclk)].count == 3 &&
           dpm[static_cast<size_t>(SmuClock::socclk)].max == 2200 &&
           dpm[static_cast<size_t>(SmuClock::fclk)].min == 4000 &&
           !dpm[static_cast<size_t>(SmuClock::gfxclk)].from_firmware &&
           dpm[static_cast<size_t>(SmuClock::gfxclk)].count == 1,
           "PMFW DPM levels for running clocks; boot-value level for the others");
    const auto sdma = fixture.asic->sdma_report();
    expect(fixture.asic->report().sdma == Sdma6Status::started && sdma.ring_ready[0] && sdma.ring_ready[1] &&
           sdma.doorbell_mode && fixture.device->sdma_executions[0] == 2 &&
           fixture.device->sdma_executions[1] == 2 && sdma.wptr_bytes[0] == 192 &&
           fixture.asic->report().sdma_ib == Sdma6Status::started &&
           fixture.asic->report().gfx_ib == Gfx11QueuesStatus::running,
           "both SDMA rings pass their ring tests, then gfx, compute and SDMA IB tests pass");
    const auto mes = fixture.asic->mes_report();
    expect(fixture.asic->report().mes == Mes11Status::scheduler_ready && mes.mes_enabled &&
           mes.legacy_queue_map && fixture.mes_firmware.mapped &&
           !fixture.mes_firmware.mes_frames.empty(),
           "MES KIQ maps the scheduler, which completes SET_HW_RESOURCES and status queries");
    // amdgpu_mes_self_test: the shared model cannot run queues through a process
    // VM, so the gfx queue's ring test fails here (tests/mes_selftest_tests.cpp
    // covers the passing path). Linux ignores the result; bring-up still completes
    // and the self-test memory stays quarantined until reset proof.
    const auto self_test = fixture.asic->mes_self_test_report();
    expect(fixture.asic->report().mes_self_test.map_status == MesSelfTestVmStatus::mapped &&
           self_test.ran && self_test.queues_added == 3 && self_test.memory_quarantined &&
           fixture.asic->report().step == AsicStep::complete,
           "MES self-test adds gfx/compute/SDMA queues, reports its result and never fails bring-up");
    const auto apps = fixture.asic->trusted_apps_report();
    expect(apps.ras.state == PspTaState::loaded && apps.hdcp.state == PspTaState::loaded &&
           apps.dtm.state == PspTaState::loaded && !apps.boot_config.dynamic_supported &&
           !apps.boot_config.set_sent,
           "RAS, HDCP and DTM TAs loaded after RL; no BOOT_CFG without the dynamic boot-config cap");
    const auto queues = fixture.asic->queues_report();
    bool tested = fixture.asic->report().queues == Gfx11QueuesStatus::running && queues.queues_mapped == 9 &&
                  fixture.mes_firmware.queues.size() == 9;

    for (uint32_t ring = 0; ring < gfx11_max_rings; ++ring) {
        tested = tested && queues.rings[ring].mapped && queues.rings[ring].ring_test == Gfx11RingTest::passed;
    }

    expect(tested, "gfx ring and eight compute rings mapped through MES and ring-tested before SDMA");
    // VRAM: GART table, scratch, boot staging, PSP control/firmware, TMR, SMU
    // driver/tool tables, clear state, 4 self-test page tables, 3 TA shared buffers. GTT: IH 3, SDMA 3,
    // MES 7, kernel queues 20, IB 2, self-test 8 (metadata, process, 3 gangs, 3 MQDs).
    expect(fixture.backend.destroys == 1 && fixture.backend.creates == 9 + 4 + 3 &&
           fixture.device->gtt_allocations == 35 + 8,
           "bootloader staging released after SOS; VRAM firmware/table memory and GTT ring memory retained");
    // 46 GART binds (the 1 MiB KIQ ring takes four 64-page chunks). Before the MES
    // scheduler runs they flush on the engine-17 register path: IH 3 + MES 10 binds
    // on MMHUB, the 10 MES binds on GFXHUB (enabled after IH). After it, Linux
    // gmc_v11_0_flush_gpu_tlb sends each hub's flush as a MES MISC frame: 33 x 2.
    size_t misc_frames = 0;

    for (const auto& frame : fixture.mes_firmware.mes_frames) {
        misc_frames += ((frame.at(0) >> 4) & 0xffu) == 14 ? 1 : 0;
    }

    expect(fixture.device->mmhub_invalidations == 1 + 13 && fixture.asic->gmc_report().mmhub_enabled &&
           fixture.asic->gmc_report().gfxhub_enabled && fixture.device->gfxhub_invalidations == 1 + 10 &&
           misc_frames == 33 * 2,
           "GART flushes on the register path until MES runs, then through MES for both hubs");
    expect(fixture.asic->gfx_report().cp_started && fixture.asic->gfx_report().rlc_resumed &&
           fixture.device->reg(0x00d20 + 0xf5) == 0x3u &&
           fixture.device->reg(0x00d20 + 0xf3) == 0xe0u,
           "GFX core started and doorbell self-ring aperture enabled at the BAR base");
    expect(fixture.device->bus_master_enables == 1 && fixture.asic->report().ih == Ih6Status::enabled &&
           fixture.asic->ih_report().ring_mc[0] >= fixture.asic->gmc_report().gart.start &&
           fixture.asic->ih_report().ring_mc[1] <= fixture.asic->gmc_report().gart.end &&
           fixture.asic->sdma_report().ring_mc[0] >= fixture.asic->gmc_report().gart.start,
           "IH rings enabled with bus mastering before PSP firmware; IH and SDMA rings in GTT");
    expect(fixture.ih_ready_calls == 1 && fixture.ih_enabled_at_ready && fixture.psp_commands_at_ready == 0,
           "interrupt owner armed right after IH enable, before any PSP command (Linux IRQ handler order)");
    expect(fixture.run() == AsicStatus::busy, "sequence cannot be repeated without reset");
    expect(fixture.asic->release_after_reset() == AsicStatus::quarantined && !fixture.manager->empty(),
           "nothing is released without reset proof");
    fixture.device->reset_confirmed = true;
    expect(fixture.asic->release_after_reset() == AsicStatus::released && fixture.manager->empty(),
           "confirmed reset releases every lease and allocation");
}

void refusals() {
    Fixture missing(false);
    expect(missing.run() == AsicStatus::invalid_input &&
           missing.asic->report().step == AsicStep::inputs && missing.device->events.empty() &&
           missing.device->hdp_flushes == 0, "missing IMU firmware refused before MMIO");

    Fixture no_mp1;
    no_mp1.discovery = rx7900_test::discovery_with({
        {discovery_mp0_hardware_id, 13, 0, {rx7900_test::mp_base}},
        {discovery_nbif_hardware_id, 4, 3, {0, 0x14, 0xd20, 0x10400, 0x241b000, 0x4040000}},
        {discovery_mmhub_hardware_id, 3, 0, {rx7900_test::mmhub_base}}});
    expect(no_mp1.run() == AsicStatus::invalid_input &&
           no_mp1.asic->report().step == AsicStep::discovery && no_mp1.device->events.empty(),
           "discovery without MP1 refused before MMIO");

    Fixture running;
    running.device->reg(rx7900_test::mp_base + 0x91) = 1;
    running.device->reg(rx7900_test::mp_base + 0x80) = 0x80000000u;
    expect(running.run() == AsicStatus::completed && running.asic->report().sos_was_running &&
           running.of(Event::Kind::psp_boot).empty() && running.backend.destroys == 0,
           "running SOS skips the bootloader exactly as Linux");

    Fixture without_display;
    without_display.policy.load_dmcub = false;
    without_display.run();
    bool dmcub = false;

    for (const auto& event : without_display.of(Event::Kind::psp_command)) {
        dmcub = dmcub || event.detail == 51;
    }

    expect(without_display.asic->report().status == AsicStatus::completed && !dmcub &&
           without_display.asic->report().loaded_ucode == 26, "DMCUB load is a policy choice");

    Fixture no_scpm;
    no_scpm.device->runtime_db.at(0x218) = 0;
    expect(no_scpm.run() == AsicStatus::completed && !no_scpm.asic->report().scpm_enabled &&
           no_scpm.device->uploaded_pptable.size() == smu13_pptable_bytes &&
           no_scpm.device->uploaded_pptable.at(smu13_board_table_offset) == 0x5a &&
           no_scpm.device->smu_messages.size() == 17 + 44 + 5 + 1,
           "without SCPM the board PPTable is written and the allowed mask sent");

    Fixture no_board;
    no_board.device->runtime_db.at(0x218) = 0;
    no_board.board.clear();
    expect(no_board.run() == AsicStatus::failed && no_board.asic->report().step == AsicStep::smu_pptable,
           "non-SCPM boards need the ATOM BoardTable");

    Fixture dead_mes;
    dead_mes.mes_firmware.mes_dead = true;
    expect(dead_mes.run() == AsicStatus::quarantined && dead_mes.asic->report().step == AsicStep::mes_kiq &&
           dead_mes.asic->report().sdma == Sdma6Status::unavailable,
           "an unanswered MES API quarantines before SDMA starts");
    expect(dead_mes.asic->release_after_reset() == AsicStatus::quarantined,
           "MES leases stay retained without reset proof");
    dead_mes.device->reset_confirmed = true;
    expect(dead_mes.asic->release_after_reset() == AsicStatus::released && dead_mes.manager->empty(),
           "confirmed reset releases MES and every sequencer allocation");

    Fixture stop;
    expect(stop.run() == AsicStatus::completed && stop.asic->shutdown() == AsicStatus::completed &&
           stop.asic->report().shut_down && stop.asic->report().fini_failure == AsicStep::stopped,
           "shutdown confirms every reverse-order hw_fini stage");
    const auto& psp = stop.device->commands;
    expect(psp.size() >= 2 && psp.at(psp.size() - 2).id == 0x2 && psp.back().id == 0x7 &&
           psp.at(psp.size() - 2).words[0] == stop.asic->report().asd_session &&
           psp.at(psp.size() - 3).id == 0x2 && psp.at(psp.size() - 4).id == 0x2 && psp.at(psp.size() - 5).id == 0x2,
           "PSP fini: UNLOAD_TA for RAS, DTM, HDCP, then the ASD session, then DESTROY_TMR");
    expect(stop.asic->trusted_apps_report().ras.lease_retained == false &&
           stop.asic->trusted_apps_report().hdcp.lease_retained == false,
           "TA shared-buffer leases returned after confirmed unloads");
    expect((stop.device->reg(0xa000 + 0x589a) & 1u) == 1u && (stop.device->reg(0xa000 + 0x589a + 0x20) & 1u) == 1u &&
           (stop.device->reg(0xa000 + 0x803) & 0x14000000u) == 0x14000000u &&
           (stop.device->reg(0xa000 + 0x2904) & 0x40000000u) != 0 &&
           stop.mes_firmware.device->reg(0xa000 + 0x2807) == 0x40030010u,
           "both SDMA engines, ME/PFP, the MEC and MES halted");
    expect(stop.mes_firmware.queues.empty() && stop.mes_firmware.removed_queues.size() == 3 + 9,
           "self-test queues removed in late init; all nine kernel queues removed before KIQ fini");
    expect(stop.device->reg(rx7900_test::mmhub_base + mmhub_3_0_0_layout.context0_control) == 0 &&
           stop.device->reg(rx7900_test::mmhub_base + mmhub_3_0_0_layout.context1_control + 14) == 0 &&
           stop.device->reg(rx7900_test::gc_base + gfxhub_3_0_layout.context0_control) == 0 &&
           (stop.device->reg(rx7900_test::mmhub_base + mmhub_3_0_0_layout.l2_control) & 1u) == 0 &&
           !stop.asic->gmc_report().mmhub_enabled && !stop.asic->gmc_report().gfxhub_enabled &&
           (stop.device->reg(0xd20 + 0xc0) & 1u) == 0 && stop.device->reg(0xd20 + 0xf5) == 0,
           "both VM hubs disabled and the doorbell and self-ring apertures closed");
    expect(stop.asic->shutdown() == AsicStatus::busy && stop.asic->release_after_reset() == AsicStatus::quarantined,
           "shutdown runs once and never releases memory without reset proof");
    stop.device->reset_confirmed = true;
    expect(stop.asic->release_after_reset() == AsicStatus::released && stop.manager->empty() &&
           stop.device->gtt_releases == 43,
           "confirmed reset after shutdown releases every lease and all 43 GTT buffers");

    Fixture reset;
    reset.atom_info.bios_scratch_reg = 0x5c9;
    reset.atom = &reset.atom_info;
    ResetLog log;
    struct ExternalResetOwner final {
        Fixture* fixture;
        unsigned calls = 0;
        bool allow = false;
        bool ordered = true;
        static bool retire(void* context) noexcept {
            auto& owner = *static_cast<ExternalResetOwner*>(context);
            ++owner.calls;
            owner.ordered = owner.ordered && !owner.fixture->manager->empty() &&
                            owner.fixture->device->gtt_releases == 0;
            return owner.allow;
        }
    } external{&reset};
    expect(reset.asic->release_after_reset(&external, ExternalResetOwner::retire) == AsicStatus::busy &&
           external.calls == 0, "external process cleanup never runs without a device reset proof");
    expect(reset.asic->mode1_reset(log.platform()) == AsicStatus::busy && log.calls.empty(),
           "mode1_reset refused before any run");
    expect(reset.run() == AsicStatus::completed && reset.asic->shutdown() == AsicStatus::completed,
           "run and shutdown before the explicit reset");
    reset.device->reg(0x5c9 + 3) = 0x00000004u;
    const auto before_reset = reset.device->smu_messages.size();
    expect(reset.asic->mode1_reset(log.platform()) == AsicStatus::completed &&
           reset.asic->report().step == AsicStep::reset_done &&
           log.calls == std::vector<int>{1, 2, 3} &&
           reset.device->smu_messages.size() == before_reset + 1 && reset.device->smu_messages.back() == 0x2f &&
           reset.device->reg(0x5c9 + 3) == 0x00000004u,
           "PCI cache, bus master off, SMU Mode1Reset, PCI restore; engine-hung flag set then cleared");
    expect(reset.asic->release_after_reset(&external, ExternalResetOwner::retire) ==
               AsicStatus::quarantined && external.calls == 1 && external.ordered &&
               !reset.manager->empty() && reset.device->gtt_releases == 0,
           "failed process retirement retains MES/GART and every firmware allocation");
    external.allow = true;
    expect(reset.asic->release_after_reset(&external, ExternalResetOwner::retire) ==
               AsicStatus::released && external.calls == 2 && external.ordered && reset.manager->empty() &&
           !reset.device->reset_confirmed && reset.device->gtt_releases == 43,
           "a completed mode-1 reset is the reset proof: every lease and GTT page released");

    // Reload after reset: the same AsicInit runs again on the reset ASIC and loads
    // every firmware from the bootloader state (psp_hw_init / psp_load_fw again).
    reset.mes_firmware.reset();
    const auto boots_before = reset.device->boot_commands;
    const auto psp_before = reset.device->commands.size();
    expect(reset.device->mode1_resets == 1 && reset.run() == AsicStatus::completed &&
           reset.asic->report().step == AsicStep::complete && !reset.asic->report().sos_was_running &&
           reset.device->boot_commands > boots_before && reset.device->commands.size() > psp_before &&
           reset.asic->report().loaded_ucode == 27 && reset.asic->report().sdma_ib == Sdma6Status::started,
           "after a proven mode-1 reset the full sequence reloads SOS and every firmware again");
    expect(reset.asic->shutdown() == AsicStatus::completed, "second bring-up shuts down cleanly");

    Fixture stuck;
    ResetLog stuck_log;
    expect(stuck.run() == AsicStatus::completed && stuck.asic->shutdown() == AsicStatus::completed,
           "run and shutdown before a failing reset");
    stuck.device->bootloader_after_reset = false;
    expect(stuck.asic->mode1_reset(stuck_log.platform()) == AsicStatus::quarantined &&
           stuck.asic->report().step == AsicStep::reset_bootloader &&
           stuck.asic->release_after_reset() == AsicStatus::quarantined,
           "a bootloader that never signals ready leaves the reset unproven and leases retained");

    Fixture no_irq;
    no_irq.ih_ready_result = false;
    expect(no_irq.run() == AsicStatus::failed && no_irq.asic->report().step == AsicStep::interrupts,
           "a refused interrupt owner fails bring-up at the IH step");

    // MALL_INFO v1 (amdgpu_discovery_get_mall_info): m_half_use 0b101 gives the
    // RAS TA channel_dis_num hweight32(m_half_use) * 2 = 4.
    Fixture mall;
    {
        auto& bin = mall.discovery;
        const size_t table = bin.size();
        bin.resize(table + 36, 0);
        const auto put16 = [&](size_t at, uint16_t v) { bin.at(at) = uint8_t(v); bin.at(at + 1) = uint8_t(v >> 8); };
        const auto put32 = [&](size_t at, uint32_t v) { put16(at, uint16_t(v)); put16(at + 2, uint16_t(v >> 16)); };
        put32(table, 0x4c4c414du);
        put16(table + 4, 1);
        put32(table + 8, 36);
        put32(table + 20, 0x5u);
        put16(12 + 4 * 8, static_cast<uint16_t>(table));
        put16(10, static_cast<uint16_t>(bin.size()));
        uint16_t sum = 0;

        for (size_t index = 10; index < bin.size(); ++index) {
            sum = static_cast<uint16_t>(sum + bin.at(index));
        }

        put16(8, sum);
    }
    mall.device->discovery = mall.discovery;
    expect(mall.run() == AsicStatus::completed &&
           (mall.asic->trusted_apps_report().ras_init_flags_high & 0xffu) == 4,
           "MALL_INFO v1 m_half_use feeds the RAS TA channel_dis_num");

    // Kernel submission after bring-up: an IB on gfx ring 0 (WRITE_DATA) and on
    // SDMA0 (WRITE_LINEAR), both into GTT, each completing through its fence.
    Fixture submit;
    HostFences host;
    FenceManager fences;
    expect(submit.run() == AsicStatus::completed &&
           fences.initialize({&host, HostFences::lock, HostFences::unlock, HostFences::now_us, HostFences::signal,
                              HostFences::reset, HostFences::block, HostFences::vm_fault, HostFences::hang}) ==
               FenceStatus::ready &&
           submit.asic->enable_kernel_submission(fences, {&host, HostFences::store}) == AsicStatus::completed,
           "kernel submission enabled on the gfx, compute and SDMA rings after bring-up");
    rx7900::memory::Handle ib{};
    rx7900::memory::AllocationInfo ib_info{};
    expect(submit.asic->gtt_manager().allocate(0x1000, 0x1000, rx7900::memory::Access::read_write, true, ib) ==
               rx7900::memory::Status::success &&
           submit.asic->gtt_manager().describe(ib, ib_info) == rx7900::memory::Status::success,
           "IB page allocated in GTT");
    const uint64_t target = ib_info.gpu.virtual_address + 0x800;
    const rx7900::sdma::CommandBuffer ib_cpu(ib_info.backing.cpu_bytes, 0x1000);
    const uint32_t gfx_ib[]{0xc0033700u, 0x00100500u, static_cast<uint32_t>(target), static_cast<uint32_t>(target >> 32), 0x12345678u};
    const uint32_t sdma_ib[]{2u, static_cast<uint32_t>(target + 4), static_cast<uint32_t>((target + 4) >> 32), 0u, 0x9abcdef0u, 0u, 0u, 0u};

    for (uint32_t index = 0; index < 5; ++index) {
        ib_cpu.write_u32(index * 4, gfx_ib[index]);
    }

    for (uint32_t index = 0; index < 8; ++index) {
        ib_cpu.write_u32(0x100 + index * 4, sdma_ib[index]);
    }

    uint32_t gfx_seq = 0;
    uint32_t sdma_seq = 0;
    expect(submit.asic->kernel_queues().submit({0, ib, 0, 5}, gfx_seq) == Gfx11QueuesStatus::running &&
           submit.asic->sdma_rings().submit({0, ib, 0x100, 8}, sdma_seq) == Sdma6Status::started,
           "IBs scheduled on gfx ring 0 and SDMA0");
    expect(fences.query(0, gfx_seq) == FenceStatus::signalled &&
           fences.query(fence_ring_sdma0, sdma_seq) == FenceStatus::signalled &&
           submit.device->vram_u32(target) == 0x12345678u && submit.device->vram_u32(target + 4) == 0x9abcdef0u,
           "both fences signalled and both IBs executed");
    expect(submit.asic->shutdown() == AsicStatus::completed && submit.asic->report().fini_failure == AsicStep::stopped,
           "shutdown waits for fences, disables submission and stops every block");

    Fixture hung_compute;
    hung_compute.mes_firmware.hung_queue_doorbell = (0x3 + 5) << 1;
    expect(hung_compute.run() == AsicStatus::quarantined &&
           hung_compute.asic->report().step == AsicStep::kernel_queues &&
           hung_compute.asic->queues_report().failed_ring == 6 &&
           hung_compute.asic->report().sdma == Sdma6Status::unavailable,
           "a compute ring that never fetches quarantines the kernel-queue step");

    Fixture sdma_hang;
    sdma_hang.device->sdma_hang_instance = 1;
    expect(sdma_hang.run() == AsicStatus::quarantined && sdma_hang.asic->report().step == AsicStep::sdma &&
           sdma_hang.asic->sdma_report().ring_ready[0] && !sdma_hang.asic->sdma_report().ring_ready[1],
           "SDMA1 ring-test timeout quarantines after SDMA0 passed");

    Fixture no_link;
    no_link.pcie = {0, 0};
    expect(no_link.run() == AsicStatus::failed && no_link.asic->report().step == AsicStep::smu_pcie,
           "unknown PCIe link refused rather than guessed");
}

void memory_training() {
    Fixture absent;
    absent.device->runtime_db.at(0) = 0;
    expect(absent.run() == AsicStatus::unsupported &&
           absent.asic->report().step == AsicStep::memory_training &&
           absent.asic->report().training_requested && absent.of(Event::Kind::psp_boot).empty() &&
           absent.backend.creates == 2,
           "no runtime database forces training; unknown firmwareinfo stops before PSP");

    Fixture unsupported;
    unsupported.device->runtime_db.at(0) = 0;
    unsupported.atom = &unsupported.atom_info;
    expect(unsupported.run() == AsicStatus::completed && unsupported.asic->report().training_requested &&
           unsupported.of(Event::Kind::training_save).empty(),
           "firmwareinfo without 2-stage BIST training skips training as Linux does");

    Fixture trained;
    trained.device->runtime_db.at(0x210) = 2;
    trained.atom_info.capability = atom_capability_two_stage_training;
    trained.atom_info.reserved_fb_bytes = 4u << 20;
    trained.atom = &trained.atom_info;
    const auto status = trained.run();
    const auto boot = trained.of(Event::Kind::psp_boot);
    expect(status == AsicStatus::completed && trained.asic->report().training.status ==
           PspTrainingStatus::completed && boot.size() == 5 && boot.at(0).value == 0x100000 &&
           boot.at(1).value == 0x80000,
           "requested and supported long training runs before the first bootloader step");
    expect(trained.first(Event::Kind::training_restore) < trained.first(Event::Kind::ring_create),
           "encroached VRAM restored before any SOS or ring use");

    Fixture no_callbacks;
    no_callbacks.device->runtime_db.at(0x210) = 2;
    no_callbacks.atom_info.capability = atom_capability_two_stage_training;
    no_callbacks.atom = &no_callbacks.atom_info;
    no_callbacks.training_callbacks = false;
    expect(no_callbacks.run() == AsicStatus::unsupported &&
           no_callbacks.of(Event::Kind::psp_boot).empty(), "training without save/restore refused");

    Fixture hang;
    hang.device->runtime_db.at(0x210) = 2;
    hang.device->train_hang = true;
    hang.atom_info.capability = atom_capability_two_stage_training;
    hang.atom = &hang.atom_info;
    expect(hang.run() == AsicStatus::failed && hang.asic->report().step == AsicStep::memory_training &&
           hang.asic->report().training.status == PspTrainingStatus::timeout &&
           hang.of(Event::Kind::psp_boot).size() == 1, "training timeout stops before SOS boot");

    Fixture alive;
    alive.device->runtime_db.at(0x210) = 2;
    alive.atom = nullptr;
    alive.device->reg(rx7900_test::mp_base + 0x91) = 1;
    alive.device->reg(rx7900_test::mp_base + 0x80) = 0x80000000u;
    expect(alive.run() == AsicStatus::completed, "running SOS skips training exactly as Linux");

    Fixture scpm;
    expect(scpm.run() == AsicStatus::completed && scpm.asic->report().scpm_enabled &&
           scpm.asic->report().runtime.present && !scpm.asic->report().training_requested,
           "SCPM status and cleared training bit reported");
}

void failures() {
    Fixture rejected;
    rejected.device->fail_fw_type = 89;
    expect(rejected.run() == AsicStatus::failed &&
           rejected.asic->report().step == AsicStep::ip_firmware &&
           rejected.asic->report().failed_ucode_id == 17 &&
           rejected.asic->report().ring == PspRingStatus::command_failed,
           "PSP rejection of MEC stops the sequence and names the ucode");

    Fixture lost;
    lost.device->drop_fences = true;
    expect(lost.run() == AsicStatus::quarantined && lost.asic->report().step == AsicStep::psp_toc &&
           lost.asic->ring_report().memory_retained, "fence loss quarantines PSP memory");
    lost.device->reset_confirmed = true;
    expect(lost.asic->release_after_reset() == AsicStatus::released && lost.manager->empty(),
           "quarantined memory released only after confirmed reset");

    Fixture smu;
    smu.device->smu_fail_message = 0xe;
    expect(smu.run() == AsicStatus::failed && smu.asic->report().step == AsicStep::smu_tables &&
           smu.asic->report().smu == SmuStatus::command_failed, "SMU table rejection reported");

    Fixture pmfw;
    pmfw.device->mp1_flags = 0;
    expect(pmfw.run() == AsicStatus::failed && pmfw.asic->report().step == AsicStep::smu_start &&
           pmfw.asic->report().smu == SmuStatus::firmware_not_running,
           "PMFW not running after PSP load is reported");

    Fixture remap;
    remap.device->remap_readback_corrupt = true;
    expect(remap.run() == AsicStatus::failed && remap.asic->report().step == AsicStep::nbio &&
           remap.of(Event::Kind::psp_boot).empty(), "unconfirmed HDP remap stops before PSP");
}

// IBs on gfx ring 0 (WRITE_DATA of value at target) and SDMA0 (WRITE_LINEAR of
// value + 1 at target + 4), both in the GTT page ib, through the fences.
bool submit_pair(AsicInit& asic, rx7900::memory::Handle ib, const rx7900::memory::AllocationInfo& info,
                 uint32_t value, uint32_t& gfx_seq, uint32_t& sdma_seq) {
    const uint64_t target = info.gpu.virtual_address + 0x800;
    const rx7900::sdma::CommandBuffer ib_cpu(info.backing.cpu_bytes, 0x1000);
    const uint32_t gfx_ib[]{0xc0033700u, 0x00100500u, static_cast<uint32_t>(target),
                            static_cast<uint32_t>(target >> 32), value};
    const uint32_t sdma_ib[]{2u, static_cast<uint32_t>(target + 4), static_cast<uint32_t>((target + 4) >> 32),
                             0u, value + 1, 0u, 0u, 0u};

    for (uint32_t index = 0; index < 5; ++index) {
        ib_cpu.write_u32(index * 4, gfx_ib[index]);
    }

    for (uint32_t index = 0; index < 8; ++index) {
        ib_cpu.write_u32(0x100 + index * 4, sdma_ib[index]);
    }

    return asic.kernel_queues().submit({0, ib, 0, 5}, gfx_seq) == Gfx11QueuesStatus::running &&
           asic.sdma_rings().submit({0, ib, 0x100, 8}, sdma_seq) == Sdma6Status::started;
}

// SuspendPlatform::ih_stopping: the interrupt owner stops reading the IH rings.
struct InterruptOwner final {
    AsicInit* asic = nullptr;
    unsigned stops = 0;
    bool ih_enabled_at_stop = false;
    bool result = true;
    static bool stopping(void* context) noexcept {
        auto& self = *static_cast<InterruptOwner*>(context);
        ++self.stops;
        self.ih_enabled_at_stop = self.asic->ih_report().enabled;
        return self.result;
    }
};

// A cold-booted (two-stage trained, VBIOS-posted) device with kernel submission.
struct PoweredFixture final {
    Fixture fixture;
    HostFences host;
    FenceManager fences;
    ResetLog pci;
    InterruptOwner interrupts;

    explicit PoweredFixture(bool train = true) {
        fixture.atom_info.bios_scratch_reg = 0x5c9;
        fixture.atom = &fixture.atom_info;

        if (train) {
            fixture.device->runtime_db.at(0x210) = 2;
            fixture.atom_info.capability = atom_capability_two_stage_training;
            fixture.atom_info.reserved_fb_bytes = 4u << 20;
        }

        // ATOM_S7_ASIC_INIT_COMPLETE: the VBIOS posted the card at boot.
        fixture.device->reg(0x5c9 + 7) = 0x200;
        interrupts.asic = fixture.asic.get();
    }

    bool start() {
        return fixture.run() == AsicStatus::completed &&
               fences.initialize({&host, HostFences::lock, HostFences::unlock, HostFences::now_us,
                                  HostFences::signal, HostFences::reset, HostFences::block,
                                  HostFences::vm_fault, HostFences::hang}) == FenceStatus::ready &&
               fixture.asic->enable_kernel_submission(fences, {&host, HostFences::store}) ==
                   AsicStatus::completed;
    }

    SuspendPlatform platform() noexcept { return {pci.platform(), &interrupts, InterruptOwner::stopping}; }

    // The power loss of S3/S4 plus the MES/CP firmware forgetting its queues.
    void lose_power() {
        fixture.device->power_cycle();
        fixture.mes_firmware.reset();
    }
};

void suspend_resume() {
    PoweredFixture powered;
    Fixture& f = powered.fixture;
    expect(powered.start(), "trained cold boot with kernel submission enabled");
    rx7900::memory::Handle ib{};
    rx7900::memory::AllocationInfo ib_info{};
    expect(f.asic->gtt_manager().allocate(0x1000, 0x1000, rx7900::memory::Access::read_write, true, ib) ==
               rx7900::memory::Status::success &&
           f.asic->gtt_manager().describe(ib, ib_info) == rx7900::memory::Status::success,
           "caller-owned IB page allocated in GTT");
    uint32_t gfx_before = 0;
    uint32_t sdma_before = 0;
    expect(submit_pair(*f.asic, ib, ib_info, 0x11110000u, gfx_before, sdma_before) &&
           powered.fences.query(0, gfx_before) == FenceStatus::signalled &&
           powered.fences.query(fence_ring_sdma0, sdma_before) == FenceStatus::signalled,
           "gfx and SDMA work signalled before suspend");

    const uint64_t tmr_before = f.asic->ring_report().tmr_mc_address;
    const uint64_t gfx_ring_before = f.asic->queues_report().rings[0].ring_mc;
    const auto gtt_before = f.device->gtt_allocations;
    const auto vram_before = f.backend.creates;
    const auto commands_before = f.device->commands.size();
    const auto smu_before = f.device->smu_messages.size();
    const auto suspend_status = f.asic->suspend(powered.platform());
    const auto report = f.asic->report();
    expect(suspend_status == AsicStatus::completed && report.suspended && report.step == AsicStep::suspended &&
           report.suspend_failure == AsicStep::suspended && report.suspends == 1,
           "suspend confirms every stage and the suspend_noirq reset");

    std::vector<uint32_t> smu_suspend(f.device->smu_messages.begin() + static_cast<long>(smu_before),
                                      f.device->smu_messages.end());
    const auto smu_parameter = f.device->smu_parameters.at(smu_before + 1);
    expect(smu_suspend == std::vector<uint32_t>{0x29, 0x3b, 0x2f} && smu_parameter == 0,
           "SMU: DisallowGfxOff (PG ungate), DFCstateControl(DISALLOW), then Mode1Reset last");
    std::vector<uint32_t> psp_suspend;

    for (size_t index = commands_before; index < f.device->commands.size(); ++index) {
        psp_suspend.push_back(f.device->commands.at(index).id);
    }

    const auto& apps = f.asic->trusted_apps_report();
    expect(psp_suspend == std::vector<uint32_t>{2, 2, 2, 2, 7} &&
           f.device->commands.at(commands_before + 3).words[0] == report.asd_session &&
           apps.ras.state == PspTaState::unloaded && apps.hdcp.state == PspTaState::unloaded &&
           apps.dtm.state == PspTaState::unloaded && !f.asic->ring_report().ring_live,
           "PSP: UNLOAD_TA RAS, HDCP, DTM, the ASD, DESTROY_TMR, then the KM ring stopped");
    expect(powered.interrupts.stops == 1 && powered.interrupts.ih_enabled_at_stop &&
           !f.asic->ih_report().enabled, "interrupt owner stopped right before the IH rings were disabled");
    expect(!f.asic->gmc_report().mmhub_enabled && !f.asic->gmc_report().gfxhub_enabled &&
           (f.device->reg(0xd20 + 0xc0) & 1u) == 0 && f.device->mode1_resets == 1 &&
           powered.pci.calls == std::vector<int>{1, 2, 3},
           "hubs and doorbell aperture off; PCI cache, bus master off, Mode1Reset, PCI restore");
    uint32_t refused_seq = 0;
    expect(f.asic->kernel_queues().submit({0, ib, 0, 5}, refused_seq) != Gfx11QueuesStatus::running &&
           f.asic->suspend(powered.platform()) == AsicStatus::busy && f.asic->shutdown() == AsicStatus::busy,
           "no submission, second suspend or shutdown while suspended");
    rx7900::memory::Handle refused{};
    expect(f.asic->gtt_manager().allocate(0x1000, 0x1000, rx7900::memory::Access::read_write, true, refused) !=
               rx7900::memory::Status::success,
           "no GTT binding while the GART is powered down");

    // S3: the whole card loses power, the VBIOS POST flag included.
    powered.lose_power();
    const auto boots_before = f.device->boot_commands;
    expect(f.asic->resume(f.inputs()) == AsicStatus::unsupported &&
           f.asic->report().step == AsicStep::resume_post && !f.asic->report().posted &&
           f.device->boot_commands == boots_before && f.asic->report().suspended,
           "an unposted card needs ATOM asic_init: resume refuses before any write and stays retryable");

    // The platform firmware re-posted the card (ASIC_INIT_COMPLETE set again).
    f.device->reg(0x5c9 + 7) = 0x200;
    const auto events_before = f.device->events.size();
    const auto ih_ready_before = f.ih_ready_calls;
    const auto resumed = f.asic->resume(f.inputs());
    const auto after = f.asic->report();
    expect(resumed == AsicStatus::completed && after.step == AsicStep::complete && after.resumes == 1 &&
           !after.suspended && after.posted, "resume completes on the power-cycled device");

    std::vector<uint32_t> boot;

    for (size_t index = events_before; index < f.device->events.size(); ++index) {
        if (f.device->events.at(index).kind == Event::Kind::psp_boot) {
            boot.push_back(f.device->events.at(index).value);
        }
    }

    expect(boot == std::vector<uint32_t>{0x200000, 0x80000, 0x10000000, 0x10000, 0x20000} &&
           after.training.operations == (psp_training_op_short | psp_training_op_restore) &&
           after.training.status == PspTrainingStatus::completed,
           "PSP_MEM_TRAIN_RESUME: restore and short training, then KDB, SPL, SYS, SOS");
    size_t c2p_writes = 0;
    bool c2p_signature = false;

    for (const auto& write : f.device->vram_writes) {
        if (write.first >= after.training.c2p_offset && write.first < after.training.c2p_offset + 0x1000) {
            ++c2p_writes;
            c2p_signature = c2p_signature || (write.first == after.training.c2p_offset &&
                                              write.second == psp_training_signature);
        }
    }

    expect(c2p_writes == 0x400 && c2p_signature, "the saved P2C training data is written back to C2P");
    expect(after.loaded_ucode == 27 && after.rlc_autoload_started && after.asd_session == 0x77 &&
           f.asic->ring_report().tmr_mc_address == tmr_before &&
           f.asic->trusted_apps_report().ras.state == PspTaState::loaded,
           "every firmware, the ASD and the TAs reloaded into the same TMR");
    expect(f.device->gtt_allocations == gtt_before && f.backend.creates == vram_before &&
           f.asic->queues_report().rings[0].ring_mc == gfx_ring_before,
           "resume reuses every GTT and VRAM allocation");
    expect(after.gart_rebound_allocations == gtt_before - f.device->gtt_releases &&
           f.device->cpu(ib_info.gpu.virtual_address) == ib_info.backing.cpu_bytes &&
           f.asic->gmc_report().mmhub_enabled && f.asic->gmc_report().gfxhub_enabled,
           "amdgpu_gtt_mgr_recover rebound every live GTT allocation, the caller's IB included");
    const uint64_t last_page = f.asic->gmc_report().gart.end & ~0xfffULL;
    expect(f.device->vram_u32(f.asic->gmc_report().table_mc +
                              (last_page - f.asic->gmc_report().gart.start) / 4096 * 8) == 0x12345000u,
           "unbound GART entries point at the dummy page again");
    expect(f.ih_ready_calls == ih_ready_before + 1 && f.asic->report().sdma_ib == Sdma6Status::started &&
           f.asic->report().gfx_ib == Gfx11QueuesStatus::running && f.mes_firmware.queues.size() == 9,
           "interrupt owner rebound after IH resume; IB tests pass; nine kernel queues mapped again");

    uint32_t gfx_after = 0;
    uint32_t sdma_after = 0;
    expect(submit_pair(*f.asic, ib, ib_info, 0x22220000u, gfx_after, sdma_after) &&
           powered.fences.query(0, gfx_after) == FenceStatus::signalled &&
           powered.fences.query(fence_ring_sdma0, sdma_after) == FenceStatus::signalled &&
           f.device->vram_u32(ib_info.gpu.virtual_address + 0x800) == 0x22220000u &&
           f.device->vram_u32(ib_info.gpu.virtual_address + 0x804) == 0x22220001u,
           "submission works again on the resumed rings");
    expect(gfx_after - gfx_before < 0x80000000u && gfx_after != gfx_before &&
           sdma_after - sdma_before < 0x80000000u && sdma_after != sdma_before &&
           powered.fences.query(0, gfx_before) == FenceStatus::signalled,
           "fence sequences continue across suspend; earlier fences stay signalled");
    expect(f.asic->shutdown() == AsicStatus::completed, "the resumed device shuts down cleanly");
}

void suspend_failures() {
    Fixture idle;
    ResetLog idle_pci;
    expect(idle.asic->suspend({idle_pci.platform(), nullptr, nullptr}) == AsicStatus::busy &&
           idle.asic->resume(idle.inputs()) == AsicStatus::busy && idle_pci.calls.empty(),
           "suspend and resume refused before any run");

    PoweredFixture no_reset;
    expect(no_reset.start(), "bring-up before a failing suspend reset");
    expect(no_reset.fixture.asic->resume(no_reset.fixture.inputs()) == AsicStatus::busy,
           "resume refused without a suspend");
    no_reset.fixture.device->bootloader_after_reset = false;
    expect(no_reset.fixture.asic->suspend(no_reset.platform()) == AsicStatus::quarantined &&
           no_reset.fixture.asic->report().suspend_failure == AsicStep::suspend_reset &&
           !no_reset.fixture.asic->report().suspended &&
           no_reset.fixture.asic->resume(no_reset.fixture.inputs()) == AsicStatus::busy &&
           no_reset.fixture.asic->release_after_reset() == AsicStatus::quarantined,
           "an unproven suspend reset can neither be resumed nor released");

    PoweredFixture hung;
    expect(hung.start(), "bring-up before a hung SDMA fence");
    rx7900::memory::Handle ib{};
    rx7900::memory::AllocationInfo info{};
    hung.fixture.asic->gtt_manager().allocate(0x1000, 0x1000, rx7900::memory::Access::read_write, true, ib);
    hung.fixture.asic->gtt_manager().describe(ib, info);
    hung.fixture.device->sdma_hang_instance = 0;
    uint32_t gfx_seq = 0;
    uint32_t sdma_seq = 0;
    submit_pair(*hung.fixture.asic, ib, info, 0x33330000u, gfx_seq, sdma_seq);
    const auto commands = hung.fixture.device->commands.size();
    const auto smu_messages = hung.fixture.device->smu_messages.size();
    const auto status = hung.fixture.asic->suspend(hung.platform());
    bool df_allowed = false;

    for (size_t index = smu_messages; index < hung.fixture.device->smu_messages.size(); ++index) {
        df_allowed = df_allowed || (hung.fixture.device->smu_messages.at(index) == 0x3b &&
                                    hung.fixture.device->smu_parameters.at(index) == 1);
    }

    expect(status != AsicStatus::completed && hung.fixture.asic->report().step == AsicStep::complete &&
           hung.fixture.asic->report().suspend_failure == AsicStep::suspend_fences &&
           hung.fixture.device->commands.size() == commands && hung.fixture.device->mode1_resets == 0 &&
           hung.fixture.asic->gating_report().status == ClockGatingStatus::gated && df_allowed &&
           hung.fixture.device->smu_messages.back() == 0x28,
           "an unsignalled fence aborts before any block suspends; DF C-states and gating restored");

    PoweredFixture stuck_ih;
    expect(stuck_ih.start(), "bring-up before a refused interrupt stop");
    stuck_ih.interrupts.result = false;
    expect(stuck_ih.fixture.asic->suspend(stuck_ih.platform()) == AsicStatus::quarantined &&
           stuck_ih.fixture.asic->report().suspend_failure == AsicStep::suspend_ih &&
           stuck_ih.fixture.asic->ih_report().enabled && stuck_ih.fixture.device->mode1_resets == 1,
           "an interrupt owner that cannot stop keeps the IH rings enabled and fails the suspend");

    PoweredFixture rejected;
    expect(rejected.start() && rejected.fixture.asic->suspend(rejected.platform()) == AsicStatus::completed,
           "suspend before a PSP rejection on resume");
    rejected.lose_power();
    rejected.fixture.device->reg(0x5c9 + 7) = 0x200;
    rejected.fixture.device->fail_fw_type = 89;
    const auto gtt = rejected.fixture.device->gtt_allocations;
    const auto vram = rejected.fixture.backend.creates;
    expect(rejected.fixture.asic->resume(rejected.fixture.inputs()) == AsicStatus::failed &&
           rejected.fixture.asic->report().step == AsicStep::ip_firmware &&
           rejected.fixture.asic->report().failed_ucode_id == 17 &&
           rejected.fixture.device->gtt_allocations == gtt && rejected.fixture.backend.creates == vram &&
           rejected.fixture.asic->resume(rejected.fixture.inputs()) == AsicStatus::busy,
           "a PSP rejection fails the resume once, allocating nothing; it is not retryable");

    PoweredFixture released;
    expect(released.start() && released.fixture.asic->suspend(released.platform()) == AsicStatus::completed,
           "suspend before giving the device up");
    expect(released.fixture.asic->release_after_reset() == AsicStatus::released &&
           released.fixture.manager->empty() && !released.fixture.asic->report().suspended,
           "the suspend reset is reset proof: the owner may release everything instead of resuming");

    // Cold boot found the SOS alive (no training, empty training cache); after the
    // power loss the P2C block is gone too, so resume escalates to LONG training.
    PoweredFixture untrained;
    untrained.fixture.device->reg(rx7900_test::mp_base + 0x91) = 1;
    untrained.fixture.device->reg(rx7900_test::mp_base + 0x80) = 0x80000000u;
    expect(untrained.start() && untrained.fixture.asic->report().sos_was_running &&
           untrained.fixture.asic->suspend(untrained.platform()) == AsicStatus::completed,
           "suspend after a cold boot that skipped training");
    untrained.lose_power();
    untrained.fixture.device->reg(0x5c9 + 7) = 0x200;
    untrained.fixture.training_callbacks = false;
    expect(untrained.fixture.asic->resume(untrained.fixture.inputs()) == AsicStatus::unsupported &&
           untrained.fixture.asic->report().step == AsicStep::resume_training &&
           untrained.fixture.asic->report().training.operations ==
               (psp_training_op_long | psp_training_op_save | psp_training_op_restore),
           "LONG training without the encroached save/restore callbacks is refused");

    PoweredFixture retrained;
    retrained.fixture.device->reg(rx7900_test::mp_base + 0x91) = 1;
    retrained.fixture.device->reg(rx7900_test::mp_base + 0x80) = 0x80000000u;
    expect(retrained.start() && retrained.fixture.asic->suspend(retrained.platform()) == AsicStatus::completed,
           "suspend before a long resume training");
    retrained.lose_power();
    retrained.fixture.device->reg(0x5c9 + 7) = 0x200;
    const auto saves = retrained.fixture.of(Event::Kind::training_save).size();
    expect(retrained.fixture.asic->resume(retrained.fixture.inputs()) == AsicStatus::completed &&
           retrained.fixture.of(Event::Kind::training_save).size() == saves + 1 &&
           retrained.fixture.of(Event::Kind::training_restore).size() == saves + 1 &&
           retrained.fixture.asic->report().training.message == 0x100000 &&
           retrained.fixture.asic->report().training.operations ==
               (psp_training_op_long | psp_training_op_save | psp_training_op_restore),
           "LONG resume training saves and restores the encroached VRAM, then resume completes");

    PoweredFixture gangs;
    gangs.fixture.mes_firmware.banked_reg(rx7900_test::mes_model::sched_select,
                                          rx7900_test::mes_model::gp3_lo) = 0x00001063u;
    expect(gangs.start() && gangs.fixture.asic->suspend(gangs.platform()) == AsicStatus::completed &&
           gangs.fixture.asic->mes_report().gangs_suspended,
           "MES firmware 0x63 suspends all gangs (amdgpu_mes_suspend)");
    gangs.lose_power();
    gangs.fixture.device->reg(0x5c9 + 7) = 0x200;
    bool suspend_frame = false;
    bool resume_frame = false;

    expect(gangs.fixture.asic->resume(gangs.fixture.inputs()) == AsicStatus::completed &&
           !gangs.fixture.asic->mes_report().gangs_suspended, "amdgpu_mes_resume after MES hw_init");

    for (const auto& frame : gangs.fixture.mes_firmware.mes_frames) {
        const uint32_t opcode = (frame.at(0) >> 4) & 0xffu;
        suspend_frame = suspend_frame || (opcode == 6 && (frame.at(1) & 1u) == 1u);
        resume_frame = resume_frame || (opcode == 7 && (frame.at(1) & 1u) == 1u);
    }

    expect(suspend_frame && resume_frame, "SUSPEND and RESUME frames carry the all-gangs bit");
}

} // namespace

// Destructive-reset recovery: after an explicit mode-1 reset and release with a
// retained GTT allocation (2D staging, an evicted surface, a process buffer),
// restart_after_reset runs the whole bring-up again, binds the retained pages
// into the new GART at their old addresses and uses resume-style PSP training.
void restart_after_reset() {
    Fixture fixture;
    ResetLog log;
    fixture.device->runtime_db.at(0x210) = 2;
    fixture.atom_info.capability = atom_capability_two_stage_training;
    fixture.atom_info.reserved_fb_bytes = 4u << 20;
    fixture.atom_info.bios_scratch_reg = 0x5c9;
    fixture.atom = &fixture.atom_info;
    expect(fixture.run() == AsicStatus::completed &&
               fixture.asic->report().training.status == PspTrainingStatus::completed,
           "trained cold bring-up before the reset");
    const std::vector<uint8_t> posting = rx7900_test::asic_init_rom(0x5c9, rx7900_test::AsicInitRom::posting);
    const std::vector<uint8_t> silent = rx7900_test::asic_init_rom(0x5c9, rx7900_test::AsicInitRom::silent);
    AsicInputs with_rom = fixture.inputs();
    with_rom.atom_rom = {posting.data(), posting.size()};
    AsicInputs with_silent_rom = fixture.inputs();
    with_silent_rom.atom_rom = {silent.data(), silent.size()};

    rx7900::memory::Manager& gtt = fixture.asic->gtt_manager();
    rx7900::memory::Handle kept{};
    rx7900::memory::AllocationInfo before{};
    uint64_t bus_before = 0;
    expect(gtt.allocate(3 * rx7900::memory::page_bytes, rx7900::memory::page_bytes,
                        rx7900::memory::Access::read_write, true, kept) == rx7900::memory::Status::success &&
               gtt.describe(kept, before) == rx7900::memory::Status::success &&
               fixture.asic->describe_gtt_bus(kept, bus_before),
           "an owner allocates GTT it will retain across the reset");

    for (size_t offset = 0; offset < before.backing.cpu_byte_count; ++offset) {
        before.backing.cpu_bytes[offset] = static_cast<uint8_t>(offset * 7u + 3u);
    }

    expect(fixture.device->gart(before.gpu.virtual_address) == before.backing.cpu_bytes,
           "the GART maps the retained allocation before the reset");
    expect(fixture.asic->restart_after_reset(with_rom, fixture.policy,
                                             {fixture.device.get(), SimulatedNavi31::confirm}) == AsicStatus::busy,
           "restart refused while the incarnation runs");
    expect(fixture.asic->mode1_reset(log.platform()) == AsicStatus::completed && fixture.asic->reset_proven(),
           "explicit mode-1 reset is the proof");
    struct Owner final {
        unsigned calls = 0;
        static bool keep(void* context) noexcept {
            ++static_cast<Owner*>(context)->calls;
            return true;
        }
    } owner{};
    expect(fixture.asic->release_after_reset(&owner, Owner::keep) == AsicStatus::released && owner.calls == 1 &&
               !fixture.asic->reset_proven() && !gtt.empty(),
           "release keeps the owner's GTT allocation");
    AsicInputs foreign = with_rom;
    foreign.gtt.context = &log;
    // The input check happens before any hardware access.
    expect(fixture.asic->restart_after_reset(foreign, fixture.policy,
                                             {fixture.device.get(), SimulatedNavi31::confirm}) ==
               AsicStatus::invalid_input && fixture.asic->report().status == AsicStatus::released,
           "restart with another GTT platform is refused before any access");

    // The reset also restarts the shared MES firmware model (as the reload test).
    fixture.mes_firmware.reset();
    // amdgpu_do_asic_reset re-POSTs first: an ASIC_Init that leaves the card
    // unposted fails the restart before any bring-up step, retryably.
    const size_t psp_before_repost = fixture.device->commands.size();
    expect(fixture.asic->restart_after_reset(with_silent_rom, fixture.policy,
                                             {fixture.device.get(), SimulatedNavi31::confirm}) == AsicStatus::failed &&
               fixture.asic->report().status == AsicStatus::released &&
               fixture.asic->repost_report().step == AsicStep::repost_confirm &&
               !fixture.asic->repost_report().posted && fixture.device->commands.size() == psp_before_repost,
           "a repost that does not post fails the restart and leaves it retryable");
    const uint32_t trainings_before = static_cast<uint32_t>(fixture.of(Event::Kind::training_save).size());
    const auto status = fixture.asic->restart_after_reset(with_rom, fixture.policy,
                                                          {fixture.device.get(), SimulatedNavi31::confirm});
    expect(fixture.asic->repost_report().status == AsicStatus::completed && fixture.asic->repost_report().posted &&
               fixture.asic->repost_report().attempts == 2 &&
               (fixture.device->reg(0x5c9 + 7) & 0x200u) != 0,
           "the retried restart re-POSTs through ATOM ASIC_Init before the bring-up");
    const auto report = fixture.asic->report();
    expect(status == AsicStatus::completed && report.step == AsicStep::complete && fixture.asic->restarts() == 1 &&
               report.gart_rebound_allocations == 1 && report.gart_rebound_pages == 3,
           "the restart completes and rebinds exactly the retained allocation");
    expect((report.training.operations & psp_training_op_long) == 0 &&
               (report.training.operations & psp_training_op_short) != 0 &&
               fixture.of(Event::Kind::training_save).size() == trainings_before,
           "restart training is PSP_MEM_TRAIN_RESUME (short, no encroached save)");

    rx7900::memory::AllocationInfo after{};
    uint64_t bus_after = 0;
    bool intact = gtt.describe(kept, after) == rx7900::memory::Status::success &&
                  fixture.asic->describe_gtt_bus(kept, bus_after) && after.gpu.virtual_address ==
                  before.gpu.virtual_address && after.backing.cpu_bytes == before.backing.cpu_bytes &&
                  bus_after == bus_before;

    for (size_t offset = 0; intact && offset < after.backing.cpu_byte_count; ++offset) {
        intact = after.backing.cpu_bytes[offset] == static_cast<uint8_t>(offset * 7u + 3u);
    }

    expect(intact, "same GART address, bus pages and bytes after the restart");
    expect(fixture.device->gart(before.gpu.virtual_address) == before.backing.cpu_bytes &&
               fixture.device->gart(before.gpu.virtual_address + 2 * rx7900::memory::page_bytes) ==
                   before.backing.cpu_bytes + 2 * rx7900::memory::page_bytes,
           "the new GART table maps the retained pages again");

    rx7900::memory::Handle fresh{};
    rx7900::memory::AllocationInfo fresh_info{};
    expect(gtt.allocate(rx7900::memory::page_bytes, rx7900::memory::page_bytes, rx7900::memory::Access::read_write,
                        true, fresh) == rx7900::memory::Status::success &&
               gtt.describe(fresh, fresh_info) == rx7900::memory::Status::success &&
               (fresh_info.gpu.virtual_address + fresh_info.gpu.byte_length <= before.gpu.virtual_address ||
                fresh_info.gpu.virtual_address >= before.gpu.virtual_address + before.gpu.byte_length),
           "new GTT allocations never alias the retained one");
    expect(gtt.release(fresh) == rx7900::memory::Status::success &&
               gtt.release(kept) == rx7900::memory::Status::success &&
               fixture.asic->shutdown() == AsicStatus::completed,
           "the restarted incarnation releases and shuts down normally");
    expect(fixture.asic->restart_after_reset(with_rom, fixture.policy,
                                             {fixture.device.get(), SimulatedNavi31::confirm}) == AsicStatus::busy,
           "no restart without a new reset and release");

    // A structurally unsafe ASIC_Init quarantines the interpreter: the restart
    // stays refused (released, nothing programmed) for that ROM.
    Fixture unsafe;
    ResetLog unsafe_log;
    unsafe.atom_info.bios_scratch_reg = 0x5c9;
    unsafe.atom = &unsafe.atom_info;
    const std::vector<uint8_t> malformed = rx7900_test::asic_init_rom(0x5c9, rx7900_test::AsicInitRom::malformed);
    AsicInputs with_malformed = unsafe.inputs();
    with_malformed.atom_rom = {malformed.data(), malformed.size()};
    expect(unsafe.run() == AsicStatus::completed && unsafe.asic->mode1_reset(unsafe_log.platform()) ==
               AsicStatus::completed && unsafe.asic->release_after_reset() == AsicStatus::released,
           "second device reset and released");
    const size_t unsafe_psp = unsafe.device->commands.size();
    expect(unsafe.asic->restart_after_reset(with_malformed, unsafe.policy,
                                            {unsafe.device.get(), SimulatedNavi31::confirm}) == AsicStatus::quarantined &&
               unsafe.asic->restart_after_reset(with_malformed, unsafe.policy,
                                                {unsafe.device.get(), SimulatedNavi31::confirm}) ==
                   AsicStatus::quarantined &&
               unsafe.asic->report().status == AsicStatus::released && unsafe.device->commands.size() == unsafe_psp,
           "a malformed ASIC_Init quarantines the repost; the restart never programs the ASIC");
}

int main() {
    complete_sequence();
    refusals();
    memory_training();
    failures();
    suspend_resume();
    suspend_failures();
    restart_after_reset();
    return checks.finish("asic_init");
}

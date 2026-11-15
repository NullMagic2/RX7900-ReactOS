/*
Module: rx7900.tests.psp_ta
Purpose: Verify RAS/HDCP/DTM TA load/unload frames, RAS decisions and failure handling.
Created: 2026-10-22
Architecture: Synthetic register/VRAM machine plus a test PSP responder that sets
sessions, response status, boot configuration and TA shared-memory status.
*/
#include <memory>
#include <vector>

#include "asic_test_device.hpp"
#include "psp_ta.hpp"
#include "ucode_test_files.hpp"

namespace {

using namespace rx7900::hardware;
using rx7900::ByteView;
using rx7900_test::SimulatedNavi31;
using rx7900_test::ucode_files::File;
using rx7900_test::ucode_files::ta_navi31;

rx7900_test::Checks checks;

void expect(bool condition, const char* contract) { checks.expect(condition, contract); }

constexpr uint32_t ras_bytes = 0x63;
constexpr uint32_t hdcp_bytes = 0x84;
constexpr uint32_t dtm_bytes = 0x40;

// Answers each GPCOM command after the simulated device consumed the doorbell
// and before PspRing observes the fence, as a real PSP writes its response.
struct Responder final {
    SimulatedNavi31* device = nullptr;
    RegisterIo inner{};
    uint64_t control_mc = 0;
    uint32_t next_session = 0x100;
    uint32_t fail_load_bytes = 0;
    uint32_t timeout_load_bytes = 0;
    uint32_t ras_status = 0;
    uint32_t boot_config = 0;
    uint32_t boot_get_status = 0;
    uint32_t unload_status = 0;
    uint32_t seen_flags_low = 0;
    uint32_t seen_flags_high = 0;
    uint32_t seen_shared_tail = 0xffffffffu;
    std::vector<std::pair<uint32_t, uint32_t>> sessions; // (app_len, session)

    RegisterIo io() noexcept { return {this, inner.aperture_bytes, read, write, delay}; }

    static bool read(void* context, uint32_t offset, uint32_t& value) noexcept {
        auto& self = *static_cast<Responder*>(context);
        return self.inner.read32(self.inner.context, offset, value);
    }

    static void delay(void* context, uint32_t microseconds) noexcept {
        auto& self = *static_cast<Responder*>(context);
        self.inner.delay_microseconds(self.inner.context, microseconds);
    }

    static bool write(void* context, uint32_t offset, uint32_t value) noexcept {
        auto& self = *static_cast<Responder*>(context);
        const size_t before = self.device->commands.size();
        const bool written = self.inner.write32(self.inner.context, offset, value);

        if (self.device->commands.size() > before) {
            self.respond();
        }

        return written;
    }

    void respond() {
        const auto& record = device->commands.back();
        const uint64_t response = control_mc + 0x1000 + 864;

        if (record.id == 1) {
            const uint64_t shared = (static_cast<uint64_t>(record.words[4]) << 32) | record.words[3];
            const uint32_t session = next_session++;
            sessions.push_back({record.words[2], session});
            device->put_vram_u32(response + 4, session);
            device->put_vram_u32(response, record.words[2] == fail_load_bytes ? 0xffff0007u : 0);

            if (record.words[2] == ras_bytes) {
                seen_flags_low = device->vram_u32(shared + 16);
                seen_flags_high = device->vram_u32(shared + 20);
                device->put_vram_u32(shared + 8, ras_status);
            }

            seen_shared_tail = device->vram_u32(shared + 0x3ffc);

            if (record.words[2] == timeout_load_bytes) {
                device->put_vram_u32(control_mc + 0x2000, record.fence - 1);
            }
        } else if (record.id == 0x22 && record.words[1] == 2) {
            device->put_vram_u32(response, boot_get_status);
            device->put_vram_u32(response + 64, boot_config);
        } else if (record.id == 2) {
            device->put_vram_u32(response, unload_status);
        }
    }
};

struct Fixture final {
    std::unique_ptr<SimulatedNavi31> device = std::make_unique<SimulatedNavi31>();
    rx7900_test::VramBackend backend{};
    std::unique_ptr<rx7900::memory::Manager> manager = std::make_unique<rx7900::memory::Manager>();
    Nbio nbio{};
    Responder responder{};
    std::unique_ptr<PspRing> ring = std::make_unique<PspRing>();
    std::unique_ptr<PspTrustedApps> apps = std::make_unique<PspTrustedApps>();
    rx7900::memory::Handle control{};
    rx7900::memory::Handle firmware{};
    PspTaBuffers buffers{};
    File file = ta_navi31();
    PspTaInputs inputs{};
    uint32_t asd_session = 0;
    size_t first_ta_command = 0;

    Fixture() {
        backend.device = device.get();
        device->reg(rx7900_test::mp_base + 0x80) = 0x80000000u;
        manager->initialize({rx7900_test::fb_base + rx7900_test::arena_offset,
                             rx7900_test::arena_bytes}, backend.backend());
        const auto binary = rx7900_test::navi31_discovery();
        NbioRegisters registers;
        PspRegisters psp{};
        validate_nbio_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes, registers);
        validate_psp_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes, psp);
        nbio.initialize(registers, device->io());
        nbio.program_common();
        manager->allocate(psp_control_bytes, psp_ring_bytes, rx7900::memory::Access::read_write,
                          true, control);
        manager->allocate(psp_firmware_buffer_bytes, psp_firmware_buffer_bytes,
                          rx7900::memory::Access::read_write, true, firmware);

        for (auto* handle : {&buffers.ras, &buffers.hdcp, &buffers.dtm}) {
            manager->allocate(psp_ta_shared_bytes, psp_ta_shared_alignment,
                              rx7900::memory::Access::read_write, true, *handle);
        }

        responder.device = device.get();
        responder.inner = device->io();
        responder.control_mc = address(control);
        ring->initialize(psp, responder.io(), nbio, rx7900_test::navi31_vram(), {1000, 50, 10},
                         {device.get(), SimulatedNavi31::confirm});
        inputs.ta_file = file.view();
        inputs.has_display_hardware = true;
        inputs.xcc_mask = 1;
    }

    // psp_load_fw prefix: ring, ASD, RL; the TA initializers follow.
    void start() {
        ring->create(*manager, control, firmware);
        uint32_t offset = 0;
        uint32_t bytes = 0;
        ByteView asd(nullptr, 0);
        find_ta_asd(file.view(), offset, bytes);
        file.view().subview(offset, bytes, asd);
        ring->load_asd(asd, asd_session);
        const std::vector<uint8_t> rl(0x20, 0x11);
        ring->load_register_list({rl.data(), rl.size()});
        first_ta_command = device->commands.size();
    }

    PspTaStatus run(PspTaPolicy policy = {}) {
        start();
        return apps->initialize(*ring, *manager, rx7900_test::navi31_vram(), buffers, inputs, policy,
                                {device.get(), SimulatedNavi31::confirm});
    }

    uint64_t address(rx7900::memory::Handle handle) const {
        rx7900::memory::AllocationInfo info{};
        manager->describe(handle, info);
        return info.gpu.virtual_address;
    }

    std::vector<rx7900_test::PspCommandRecord> ta_commands() const {
        return {device->commands.begin() + static_cast<long>(first_ta_command), device->commands.end()};
    }

    // hdcp_released: that allocation was already retired while still leased.
    bool released_all(bool hdcp_released) {
        for (auto handle : {buffers.ras, buffers.hdcp, buffers.dtm, control, firmware}) {
            if (hdcp_released && handle.slot == buffers.hdcp.slot) {
                continue;
            }

            if (manager->release(handle) != rx7900::memory::Status::success) {
                return false;
            }
        }

        return manager->empty();
    }
};

uint32_t staged_word(const File& file, uint32_t type) {
    TaDescriptors descriptors{};
    parse_ta_descriptors(file.view(), descriptors);
    uint32_t word = 0;
    file.view().read_u32(descriptors.get(type).offset, word);
    return word;
}

void consumer_navi31_sequence() {
    Fixture fixture;
    expect(fixture.run() == PspTaStatus::completed, "TA initialization completes");
    const auto commands = fixture.ta_commands();
    const uint64_t firmware = fixture.address(fixture.firmware);
    const uint64_t ras_mc = fixture.address(fixture.buffers.ras);
    const uint64_t hdcp_mc = fixture.address(fixture.buffers.hdcp);
    const uint64_t dtm_mc = fixture.address(fixture.buffers.dtm);
    expect(commands.size() == 3, "no VBIOS dynamic boot config: no BOOT_CFG, three LOAD_TA");

    const auto frame = [&](size_t index, uint32_t bytes, uint64_t shared, uint32_t type) {
        const auto& command = commands.at(index);
        return command.id == 1 && command.words[0] == static_cast<uint32_t>(firmware) &&
               command.words[1] == static_cast<uint32_t>(firmware >> 32) &&
               command.words[2] == bytes && command.words[3] == static_cast<uint32_t>(shared) &&
               command.words[4] == static_cast<uint32_t>(shared >> 32) &&
               command.words[5] == psp_ta_shared_bytes &&
               command.first_payload_word == staged_word(fixture.file, type) && command.flushed;
    };

    expect(commands.size() == 3 && frame(0, ras_bytes, ras_mc, ta_fw_type::ras) &&
           frame(1, hdcp_bytes, hdcp_mc, ta_fw_type::hdcp) &&
           frame(2, dtm_bytes, dtm_mc, ta_fw_type::dtm),
           "psp_load_fw order RAS, HDCP, DTM: staged binary, app_len, 16 KiB shared buffer");
    expect(fixture.device->vram_u32(firmware + 0x40) == 0 &&
           fixture.device->vram_u32(firmware + 0x60) == 0,
           "DTM staging cleared the larger HDCP binary behind it");

    const auto report = fixture.apps->report();
    expect(!report.ras_support.context && !report.ras_support.umc_supported &&
           !report.ras_support.poison_mode && report.ras_support.hw_enabled == 0,
           "MEM/SRAM ECC not presented: no RAS context, UMC RAS unsupported");
    expect(report.ras_init_flags_low == 0x00010100u && report.ras_init_flags_high == 0 &&
           fixture.responder.seen_flags_low == 0x00010100u && fixture.responder.seen_flags_high == 0,
           "init_flags poison 0, dgpu_mode 1, xcc_mask 1, channel_dis_num 0 visible to PSP");
    expect(fixture.responder.seen_shared_tail == 0, "whole shared buffer cleared before LOAD_TA");
    expect(report.boot_config.action == PspBootConfigAction::not_supported &&
           !report.boot_config.queried, "boot config untouched without DYNAMIC_BOOT_CFG_ENABLE");
    expect(report.ras.state == PspTaState::loaded && report.ras.session == 0x100 &&
           report.hdcp.state == PspTaState::loaded && report.hdcp.session == 0x101 &&
           report.dtm.state == PspTaState::loaded && report.dtm.session == 0x102 &&
           report.ras.ras_status == 0 && report.ras.fw_version == 0x1b000205 &&
           report.hdcp.fw_version == 0x1700004d && report.dtm.fw_version == 0x1200001d,
           "sessions from resp.session_id with descriptor versions");
    expect(report.ras.lease_retained && report.hdcp.lease_retained && report.dtm.lease_retained &&
           fixture.manager->release(fixture.buffers.hdcp) == rx7900::memory::Status::deferred,
           "shared buffers leased while sessions live");
    expect(!report.xgmi_present && !report.rap_present && !report.securedisplay_present,
           "Navi31 TA file has no XGMI, RAP or secure display");

    expect(fixture.apps->initialize(*fixture.ring, *fixture.manager, rx7900_test::navi31_vram(),
                                    fixture.buffers, fixture.inputs, {}, {}) == PspTaStatus::busy,
           "second initialize refused while sessions are live");

    const size_t before = fixture.device->commands.size();
    expect(fixture.apps->terminate() == PspTaStatus::completed, "terminate completes");
    const auto& all = fixture.device->commands;
    expect(all.size() == before + 3 && all.at(before).id == 2 && all.at(before).words[0] == 0x100 &&
           all.at(before + 1).id == 2 && all.at(before + 1).words[0] == 0x102 &&
           all.at(before + 2).id == 2 && all.at(before + 2).words[0] == 0x101,
           "psp_hw_fini order: RAS, DTM, HDCP unload");
    const auto after = fixture.apps->report();
    expect(after.ras.state == PspTaState::unloaded && after.dtm.state == PspTaState::unloaded &&
           after.hdcp.state == PspTaState::unloaded && !after.ras.lease_retained &&
           !after.hdcp.lease_retained && !after.dtm.lease_retained,
           "confirmed unloads release the shared-buffer leases");
    expect(fixture.ring->unload_ta(fixture.asd_session) == PspRingStatus::completed &&
           fixture.device->commands.back().words[0] == 0x77, "ASD unload follows the TAs");
    fixture.ring->destroy_tmr();
    fixture.ring->stop();
    expect(fixture.released_all(true), "every allocation destroyed after shutdown");
}

void decision_tables() {
    PspRasCapabilityInputs inputs{};
    auto support = linux_ras_support(inputs);
    expect(!support.context && !support.umc_supported, "defaults without ECC: no RAS");

    inputs.mem_ecc_supported = true;
    support = linux_ras_support(inputs);
    expect(support.context && support.umc_supported && support.hw_enabled == 0x101 &&
           support.enabled == 0x101, "MEM ECC: UMC and DF");

    inputs.df_umc_poison_mode = true;
    expect(linux_ras_support(inputs).poison_mode, "poison mode with a RAS context");
    inputs.ras_enable = 0;
    support = linux_ras_support(inputs);
    expect(!support.context && !support.umc_supported && !support.poison_mode && support.hw_enabled == 0x101,
           "amdgpu_ras_enable=0 drops the context and poison mode");
    inputs.ras_enable = -1;
    inputs.ras_mask = ~1u;
    support = linux_ras_support(inputs);
    expect(support.context && !support.umc_supported, "ras_mask without UMC");

    PspRasCapabilityInputs sram{};
    sram.atom_capability = atom_capability_sram_ecc;
    sram.df_umc_poison_mode = true;
    support = linux_ras_support(sram);
    expect(support.context && !support.umc_supported && support.poison_mode &&
           support.hw_enabled == (0x7ffffu & ~(0x101u | 0x80u)),
           "SRAM ECC only: every block but UMC/DF/XGMI, poison from DF/UMC query");
    PspRasCapabilityInputs quirk{};
    quirk.vbios_gfx_quirk = true;
    quirk.df_umc_poison_mode = true;
    support = linux_ras_support(quirk);
    expect(support.hw_enabled == 4 && support.context && !support.umc_supported && support.poison_mode,
           "vbios_pn quirk enables GFX only");
    PspRasCapabilityInputs no_context{};
    no_context.df_umc_poison_mode = true;
    expect(!linux_ras_support(no_context).poison_mode, "no context: poison mode false");

    using A = PspBootConfigAction;
    expect(linux_boot_config_action(false, 1, true) == A::not_supported &&
           linux_boot_config_action(true, 0, false) == A::keep_disabled &&
           linux_boot_config_action(true, 1, false) == A::set_disable &&
           linux_boot_config_action(true, 0xff, false) == A::set_disable &&
           linux_boot_config_action(true, 1, true) == A::keep_enabled &&
           linux_boot_config_action(true, 0, true) == A::set_enable &&
           linux_boot_config_action(true, 0xff, true) == A::set_enable,
           "v6.12 psp_ras_initialize boot-config table");
}

void boot_config_paths() {
    Fixture withheld;
    withheld.inputs.ras.atom_capability = atom_capability_dynamic_boot_config;
    withheld.responder.boot_config = 0x3;
    expect(withheld.run() == PspTaStatus::completed, "dynamic boot config path completes");
    auto commands = withheld.ta_commands();
    auto boot = withheld.apps->report().boot_config;
    expect(commands.size() == 4 && commands.at(0).id == 0x22 && commands.at(0).words[1] == 2 &&
           commands.at(1).id == 1 && commands.at(1).words[2] == ras_bytes,
           "BOOT_CFG get precedes the RAS load; no set sent by default");
    expect(boot.queried && boot.raw_boot_config == 3 && boot.linux_boot_cfg == 1 &&
           boot.action == PspBootConfigAction::set_disable && boot.set_value == 0 &&
           boot.set_withheld && !boot.set_sent,
           "GECC on without UMC RAS: Linux would set 0; withheld by policy");

    Fixture sent;
    sent.inputs.ras.atom_capability = atom_capability_dynamic_boot_config;
    sent.responder.boot_config = 1;
    expect(sent.run({true}) == PspTaStatus::completed, "opt-in boot config set completes");
    commands = sent.ta_commands();
    boot = sent.apps->report().boot_config;
    expect(commands.size() == 5 && commands.at(1).id == 0x22 && commands.at(1).words[1] == 1 &&
           commands.at(1).words[2] == 0 && commands.at(1).words[3] == 0 && commands.at(2).id == 1 &&
           boot.set_sent && boot.set_status == PspRingStatus::completed,
           "opt-in sends BOOT_CFG set 0 before the RAS load");

    Fixture disabled;
    disabled.inputs.ras.atom_capability = atom_capability_dynamic_boot_config;
    disabled.responder.boot_config = 2;
    disabled.run({true});
    boot = disabled.apps->report().boot_config;
    expect(boot.action == PspBootConfigAction::keep_disabled && !boot.set_sent &&
           disabled.ta_commands().size() == 4, "GECC bit clear: GECC disabled, nothing set");

    Fixture ecc;
    ecc.inputs.ras.atom_capability = atom_capability_dynamic_boot_config;
    ecc.inputs.ras.mem_ecc_supported = true;
    ecc.inputs.ras.df_umc_poison_mode = true;
    ecc.inputs.m_half_use = 0x5;
    ecc.inputs.xcc_mask = 1;
    ecc.run({true});
    commands = ecc.ta_commands();
    boot = ecc.apps->report().boot_config;
    expect(boot.action == PspBootConfigAction::set_enable && commands.at(1).words[2] == 1 &&
           commands.at(1).words[3] == 1, "UMC RAS with GECC off: BOOT_CFG set GECC");
    expect(ecc.responder.seen_flags_low == 0x00010101u && ecc.responder.seen_flags_high == 4,
           "poison_mode_en 1 and channel_dis_num hweight32(m_half_use) * 2");

    Fixture status;
    status.inputs.ras.atom_capability = atom_capability_dynamic_boot_config;
    status.responder.boot_config = 1;
    status.responder.boot_get_status = 0xa;
    status.run();
    boot = status.apps->report().boot_config;
    expect(boot.get_status == PspRingStatus::command_failed && boot.get_response_status == 0xa &&
           boot.linux_boot_cfg == 1 && boot.action == PspBootConfigAction::set_disable &&
           status.apps->report().ras.state == PspTaState::loaded,
           "nonzero get status: Linux still uses the response bit; RAS still loads");
}

void failures() {
    Fixture ras_status;
    ras_status.responder.ras_status = 0xa003;
    expect(ras_status.run() == PspTaStatus::completed, "RAS status failure is non-fatal");
    auto report = ras_status.apps->report();
    expect(report.ras.state == PspTaState::ras_status_failed && report.ras.ras_status == 0xa003 &&
           report.ras.lease_retained && report.hdcp.state == PspTaState::loaded &&
           report.dtm.state == PspTaState::loaded, "nonzero ras_status: RAS not initialized");
    size_t before = ras_status.device->commands.size();
    ras_status.apps->terminate();
    expect(ras_status.device->commands.size() == before + 2 &&
           ras_status.device->commands.at(before).words[0] == 0x102 &&
           ras_status.apps->report().ras.lease_retained,
           "uninitialized RAS is not unloaded; its lease waits for reset");
    expect(ras_status.apps->release_after_reset() == PspTaStatus::quarantined,
           "lease release needs reset proof");
    ras_status.device->reset_confirmed = true;
    expect(ras_status.apps->release_after_reset() == PspTaStatus::released &&
           !ras_status.apps->report().ras.lease_retained, "confirmed reset releases RAS lease");

    Fixture hdcp_status;
    hdcp_status.responder.fail_load_bytes = hdcp_bytes;
    expect(hdcp_status.run() == PspTaStatus::completed, "HDCP load failure is non-fatal");
    report = hdcp_status.apps->report();
    expect(report.hdcp.state == PspTaState::load_failed && report.hdcp.response_status == 0xffff0007u &&
           report.hdcp.lease_retained && report.dtm.state == PspTaState::loaded,
           "HDCP status reported, DTM still loaded");
    before = hdcp_status.device->commands.size();
    hdcp_status.apps->terminate();
    expect(hdcp_status.device->commands.size() == before + 2, "failed HDCP is not unloaded");

    Fixture unload;
    unload.run();
    unload.responder.unload_status = 0x5;
    expect(unload.apps->terminate() == PspTaStatus::completed &&
           unload.apps->report().dtm.state == PspTaState::unload_failed &&
           unload.apps->report().dtm.lease_retained && unload.apps->report().hdcp.response_status == 5,
           "failed unloads continue and keep leases");

    Fixture headless;
    headless.inputs.has_display_hardware = false;
    headless.run();
    report = headless.apps->report();
    expect(headless.ta_commands().size() == 1 && report.ras.state == PspTaState::loaded &&
           report.hdcp.state == PspTaState::skipped_no_display &&
           report.dtm.state == PspTaState::skipped_no_display, "no display hardware: RAS only");

    Fixture no_ras;
    no_ras.file = ta_navi31(3);
    no_ras.inputs.ta_file = no_ras.file.view();
    no_ras.inputs.ras.atom_capability = atom_capability_dynamic_boot_config;
    no_ras.run();
    report = no_ras.apps->report();
    expect(report.ras.state == PspTaState::not_available && !report.boot_config.queried &&
           no_ras.ta_commands().size() == 2, "absent RAS binary: no BOOT_CFG, HDCP and DTM load");

    Fixture timeout;
    timeout.responder.timeout_load_bytes = hdcp_bytes;
    expect(timeout.run() == PspTaStatus::quarantined, "fence timeout quarantines");
    report = timeout.apps->report();
    expect(report.hdcp.state == PspTaState::quarantined && report.dtm.state == PspTaState::not_attempted &&
           report.ras.lease_retained && report.hdcp.lease_retained && report.ring_quarantined &&
           timeout.ta_commands().size() == 2, "DTM not attempted after the uncertain HDCP load");
    expect(timeout.apps->terminate() == PspTaStatus::quarantined &&
           timeout.ta_commands().size() == 2, "no unload on a quarantined ring");
    timeout.device->reset_confirmed = true;
    expect(timeout.apps->release_after_reset() == PspTaStatus::released &&
           !timeout.apps->report().ras.lease_retained && !timeout.apps->report().hdcp.lease_retained,
           "reset proof releases quarantined leases");

    Fixture small;
    small.manager->release(small.buffers.dtm);
    small.manager->allocate(0x2000, psp_ta_shared_alignment, rx7900::memory::Access::read_write,
                            true, small.buffers.dtm);
    small.run();
    report = small.apps->report();
    expect(report.dtm.state == PspTaState::invalid_memory && !report.dtm.lease_retained &&
           small.ta_commands().size() == 2 && report.hdcp.state == PspTaState::loaded,
           "undersized shared buffer: that TA skipped before PSP");

    Fixture malformed;
    malformed.file.u32(36 + 3 * 16 + 12, 0x1000);
    malformed.inputs.ta_file = malformed.file.view();
    malformed.start();
    expect(malformed.apps->initialize(*malformed.ring, *malformed.manager, rx7900_test::navi31_vram(),
                                      malformed.buffers, malformed.inputs, {}, {}) ==
           PspTaStatus::invalid_input && malformed.ta_commands().empty(),
           "malformed TA file refused before any command");

    Fixture absent;
    absent.inputs.ta_file = {nullptr, 0};
    expect(absent.run() == PspTaStatus::completed && absent.ta_commands().empty() &&
           absent.apps->terminate() == PspTaStatus::completed, "no TA firmware: nothing to do");

    Fixture not_live;
    expect(not_live.apps->initialize(*not_live.ring, *not_live.manager, rx7900_test::navi31_vram(),
                                     not_live.buffers, not_live.inputs, {}, {}) == PspTaStatus::busy,
           "TA loading requires a live ring");
}

struct Rom final {
    std::vector<uint8_t> bytes = std::vector<uint8_t>(0x400, 0);

    void u16(size_t offset, uint16_t value) {
        bytes.at(offset) = static_cast<uint8_t>(value);
        bytes.at(offset + 1) = static_cast<uint8_t>(value >> 8);
    }

    void u32(size_t offset, uint32_t value) {
        rx7900::sdma::CommandBuffer(bytes.data(), bytes.size()).write_u32(offset, value);
    }

    // ROM header 2.2 at 0x100, master data 2.1 at 0x200, umc_info at 0x300.
    Rom(uint8_t format, uint8_t revision, uint16_t size, bool present = true) {
        u16(0, 0xaa55);
        u16(0x48, 0x100);
        u16(0x100, 0x30);
        bytes.at(0x102) = 2;
        bytes.at(0x103) = 2;
        u32(0x104, 0x4d4f5441);
        u16(0x100 + 32, 0x200);
        u16(0x200, 4 + 35 * 2);
        bytes.at(0x202) = 2;
        bytes.at(0x203) = 1;
        u16(0x200 + 4 + 24 * 2, present ? 0x300 : 0);
        u16(0x300, size);
        bytes.at(0x302) = format;
        bytes.at(0x303) = revision;
    }

    ByteView view() const { return {bytes.data(), bytes.size()}; }
};

void mem_ecc() {
    bool supported = true;
    expect(atom_mem_ecc_supported(Rom(3, 1, 32, false).view(), supported) == AtomFirmwareStatus::success &&
           !supported, "no umc_info: MEM ECC not presented");

    Rom v31(3, 1, 32);
    v31.bytes.at(0x300 + 27) = 0x02;
    expect(atom_mem_ecc_supported(v31.view(), supported) == AtomFirmwareStatus::success && supported,
           "umc_info 3.1 DEFAULT_MEM_ECC_ENABLE");

    Rom v33(3, 3, 68);
    v33.u32(0x300 + 52, 0x00010000);
    expect(atom_mem_ecc_supported(v33.view(), supported) == AtomFirmwareStatus::success && supported,
           "umc_info 3.3 ENABLE_ECC_CAPABLE");

    Rom v40(4, 0, 84);
    v40.bytes.at(0x300 + 27) = 0x02;
    expect(atom_mem_ecc_supported(v40.view(), supported) == AtomFirmwareStatus::success && !supported,
           "umc_info 4.0 reads only umc_config1");
    v40.u32(0x300 + 52, 0x00010000);
    expect(atom_mem_ecc_supported(v40.view(), supported) == AtomFirmwareStatus::success && supported,
           "umc_info 4.0 ENABLE_ECC_CAPABLE");

    Rom other(3, 4, 68);
    other.bytes.at(0x300 + 27) = 0x02;
    expect(atom_mem_ecc_supported(other.view(), supported) == AtomFirmwareStatus::success && !supported,
           "unsupported content revision is not ECC");

    Rom truncated(3, 3, 54);
    supported = true;
    expect(atom_mem_ecc_supported(truncated.view(), supported) == AtomFirmwareStatus::invalid_rom &&
           supported, "umc_config1 outside the table fails closed, output unchanged");
}

} // namespace

// psp_suspend: RAS, HDCP, DTM (unlike psp_hw_fini's RAS, DTM, HDCP); the first
// refused unload ends it; psp_resume loads every TA again.
void suspend_order() {
    Fixture fixture;
    expect(fixture.run() == PspTaStatus::completed, "TAs loaded before suspend");
    size_t before = fixture.device->commands.size();
    expect(fixture.apps->suspend() == PspTaStatus::completed, "suspend unloads every TA");
    const auto& all = fixture.device->commands;
    expect(all.size() == before + 3 && all.at(before).words[0] == 0x100 &&
           all.at(before + 1).words[0] == 0x101 && all.at(before + 2).words[0] == 0x102 &&
           !fixture.apps->report().hdcp.lease_retained,
           "psp_suspend order: RAS, HDCP, DTM unload; leases returned");
    expect(fixture.apps->suspend() == PspTaStatus::busy, "suspend once");
    before = all.size();
    expect(fixture.apps->initialize(*fixture.ring, *fixture.manager, rx7900_test::navi31_vram(),
                                    fixture.buffers, fixture.inputs, {}, {}) == PspTaStatus::completed &&
           all.size() == before + 3 && all.at(before).id == 1 && all.at(before + 2).id == 1 &&
           fixture.apps->report().dtm.state == PspTaState::loaded,
           "psp_resume loads RAS, HDCP and DTM again into the same shared buffers");

    Fixture refused;
    refused.run();
    refused.responder.unload_status = 0x5;
    before = refused.device->commands.size();
    expect(refused.apps->suspend() == PspTaStatus::refused &&
           refused.device->commands.size() == before + 1 &&
           refused.apps->report().ras.state == PspTaState::unload_failed &&
           refused.apps->report().hdcp.state == PspTaState::loaded &&
           refused.apps->report().ras.lease_retained,
           "a refused RAS unload ends psp_suspend; HDCP and DTM stay loaded");
}

int main() {
    consumer_navi31_sequence();
    decision_tables();
    boot_config_paths();
    failures();
    mem_ecc();
    suspend_order();
    return checks.finish("psp_ta");
}

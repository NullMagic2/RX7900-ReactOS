/*
Module: rx7900.tests.psp_ring
Purpose: Verify KM ring creation, GPCOM frames, command layouts, fences and quarantine.
Created: 2026-10-21
Architecture: Synthetic register/VRAM machine; no firmware is sent to hardware.
*/
#include <memory>

#include "asic_test_device.hpp"

namespace {

using namespace rx7900::hardware;
using rx7900::ByteView;
using rx7900_test::SimulatedNavi31;

rx7900_test::Checks checks;

void expect(bool condition, const char* contract) { checks.expect(condition, contract); }

struct Fixture final {
    std::unique_ptr<SimulatedNavi31> device = std::make_unique<SimulatedNavi31>();
    rx7900_test::VramBackend backend{};
    std::unique_ptr<rx7900::memory::Manager> manager = std::make_unique<rx7900::memory::Manager>();
    Nbio nbio{};
    std::unique_ptr<PspRing> ring = std::make_unique<PspRing>();
    rx7900::memory::Handle control{};
    rx7900::memory::Handle firmware{};
    PspRegisters psp{};

    explicit Fixture(VramLocation vram = rx7900_test::navi31_vram()) {
        backend.device = device.get();
        device->reg(rx7900_test::mp_base + 0x80) = 0x80000000u;
        manager->initialize({rx7900_test::fb_base + rx7900_test::arena_offset,
                             rx7900_test::arena_bytes}, backend.backend());
        const auto binary = rx7900_test::navi31_discovery();
        NbioRegisters registers;
        validate_nbio_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes, registers);
        validate_psp_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes, psp);
        nbio.initialize(registers, device->io());
        nbio.program_common();
        manager->allocate(psp_control_bytes, psp_ring_bytes, rx7900::memory::Access::read_write,
                          true, control);
        manager->allocate(psp_firmware_buffer_bytes, psp_firmware_buffer_bytes,
                          rx7900::memory::Access::read_write, true, firmware);
        ring->initialize(psp, device->io(), nbio, vram, {1000, 50, 10},
                         {device.get(), SimulatedNavi31::confirm});
    }

    uint64_t address(rx7900::memory::Handle handle) const {
        rx7900::memory::AllocationInfo info{};
        manager->describe(handle, info);
        return info.gpu.virtual_address;
    }
};

std::vector<uint8_t> pattern(size_t size, uint8_t seed) {
    std::vector<uint8_t> bytes(size);

    for (size_t index = 0; index < size; ++index) {
        bytes.at(index) = static_cast<uint8_t>(seed + index * 7);
    }

    return bytes;
}

void creation_and_commands() {
    Fixture fixture;
    auto& device = *fixture.device;
    const uint64_t control = fixture.address(fixture.control);
    const uint64_t firmware = fixture.address(fixture.firmware);

    expect(fixture.ring->create(*fixture.manager, fixture.control, fixture.firmware) ==
           PspRingStatus::ring_created, "KM ring created after SOS ready");
    expect(device.ring_mc == control && device.ring_bytes == psp_ring_bytes &&
           device.reg(rx7900_test::mp_base + 0x85) == static_cast<uint32_t>(control) &&
           device.reg(rx7900_test::mp_base + 0x86) == static_cast<uint32_t>(control >> 32),
           "C2PMSG_69/70/71 carry ring MC address and size");
    expect(!device.delays.empty() && device.delays.back() == 20000,
           "20 ms ring handshake delay observed");
    expect(device.vram_u32(control) == 0 && device.vram_u32(control + 0x2000) == 0,
           "ring and fence memory zeroed before publication");
    expect(fixture.ring->report().memory_retained && fixture.ring->report().ring_live,
           "ring retains its PSP-visible memory");

    const auto toc = pattern(0x900, 3);
    uint32_t tmr = 0;
    expect(fixture.ring->load_toc({toc.data(), toc.size()}, tmr) == PspRingStatus::completed &&
           tmr == device.toc_tmr_bytes, "LOAD_TOC returns the PSP TMR size");
    const auto& first = device.commands.at(0);
    uint32_t staged = 0;
    ByteView(toc.data(), toc.size()).read_u32(0, staged);
    expect(first.id == 0x20 && first.words[0] == static_cast<uint32_t>(firmware) &&
           first.words[1] == static_cast<uint32_t>(firmware >> 32) && first.words[2] == 0x900 &&
           first.first_payload_word == staged && first.fence == 1 && first.flushed &&
           first.frame_offset == 0, "TOC frame, command buffer and fence match Linux layout");
    expect(device.vram_u32(firmware + 0x900) == 0 &&
           device.vram_u32(firmware + psp_firmware_buffer_bytes - 4) == 0,
           "psp_copy_fw clears the rest of the 1 MiB buffer");
    expect(device.vram_u32(control + 0x1000 + 4) == 0 && device.vram_u32(control + 0x1000) == 0,
           "buf_size and buf_version stay zero as in Linux");

    rx7900::memory::Handle tmr_handle{};
    fixture.manager->allocate(0x500000, psp_tmr_alignment, rx7900::memory::Access::read_write,
                              false, tmr_handle);
    const uint64_t tmr_mc = fixture.address(tmr_handle);
    const uint64_t physical = tmr_mc - rx7900_test::fb_base + (0x20ULL << 24);
    expect(fixture.ring->setup_tmr(tmr_handle, 0x500000) == PspRingStatus::completed,
           "SETUP_TMR completes");
    const auto& setup = device.commands.at(1);
    expect(setup.id == 5 && setup.words[0] == static_cast<uint32_t>(tmr_mc) && setup.words[2] == 0x500000 &&
           setup.words[3] == 2 && setup.words[4] == static_cast<uint32_t>(physical) &&
           setup.words[5] == static_cast<uint32_t>(physical >> 32) && setup.frame_offset == 64,
           "SETUP_TMR passes MC address, size, virt_phy flag and VRAM physical address");
    expect(fixture.ring->setup_tmr(tmr_handle, 0x500000) == PspRingStatus::busy,
           "only one TMR is accepted");

    const auto odd = pattern(0x13, 9);
    expect(fixture.ring->load_ip_firmware({odd.data(), odd.size()}, 87) == PspRingStatus::completed,
           "LOAD_IP_FW completes");
    const auto& ip = device.commands.at(2);
    expect(ip.id == 6 && ip.words[2] == 0x13 && ip.words[3] == 87 &&
           device.vram_u32(firmware + 0x10) == (0x00000000u | odd.at(16) | (odd.at(17) << 8) |
                                                 (odd.at(18) << 16)),
           "partial final dword staged without reading past the payload");
    expect(fixture.ring->load_ip_firmware({odd.data(), odd.size()}, 0) == PspRingStatus::invalid_payload,
           "zero firmware type refused");

    device.fail_fw_type = 88;
    expect(fixture.ring->load_ip_firmware({odd.data(), odd.size()}, 88) == PspRingStatus::command_failed &&
           fixture.ring->report().response_status == 0xffff000au, "nonzero PSP status fails closed");
    device.fail_fw_type = UINT32_MAX;
    expect(fixture.ring->autoload_rlc() == PspRingStatus::completed && device.commands.back().id == 0x21,
           "ring remains usable after a reported command failure");

    uint32_t session = 0;
    expect(fixture.ring->load_asd({odd.data(), odd.size()}, session) == PspRingStatus::completed &&
           session == 0x77 && device.commands.back().id == 4 && device.commands.back().words[5] == 0,
           "LOAD_ASD returns the session without a shared buffer");
    expect(fixture.ring->load_register_list({odd.data(), odd.size()}) == PspRingStatus::completed &&
           device.commands.back().words[3] == 67, "RL uses GFX_FW_TYPE_REG_LIST");

    for (unsigned index = 0; index < 60; ++index) {
        fixture.ring->autoload_rlc();
    }

    expect(device.commands.size() == 67 && device.commands.at(64).frame_offset == 0 &&
           device.commands.at(66).fence == 67 &&
           device.reg(rx7900_test::mp_base + 0x83) == (67u * 16u) % 1024u,
           "write pointer advances 16 dwords per frame and wraps the 4 KiB ring");

    bool flushed = true;

    for (const auto& command : device.commands) {
        flushed = flushed && command.flushed;
    }

    expect(flushed, "HDP flush precedes every write-pointer doorbell");
    expect(fixture.ring->destroy_tmr() == PspRingStatus::completed && device.commands.back().id == 7,
           "DESTROY_TMR encoded");
    expect(fixture.ring->stop() == PspRingStatus::stopped &&
           !fixture.ring->report().memory_retained && fixture.ring->report().tmr_retained &&
           device.events.back().kind == rx7900_test::Event::Kind::ring_destroy,
           "ring stop releases ring memory but keeps the TMR lease");
    expect(fixture.ring->autoload_rlc() == PspRingStatus::busy, "stopped ring refuses commands");
    expect(fixture.ring->release_after_reset() == PspRingStatus::quarantined &&
           fixture.ring->report().tmr_retained, "TMR release needs reset proof");
    device.reset_confirmed = true;
    expect(fixture.ring->release_after_reset() == PspRingStatus::released &&
           !fixture.ring->report().tmr_retained, "confirmed reset relinquishes the TMR");
    fixture.manager->release(tmr_handle);
    fixture.manager->release(fixture.control);
    fixture.manager->release(fixture.firmware);
    expect(fixture.manager->empty(), "all allocations destroyed after lease release");
}

void failures() {
    Fixture timeout;
    timeout.ring->create(*timeout.manager, timeout.control, timeout.firmware);
    timeout.device->drop_fences = true;
    expect(timeout.ring->autoload_rlc() == PspRingStatus::timeout, "missing fence times out");
    expect(timeout.ring->autoload_rlc() == PspRingStatus::quarantined &&
           timeout.manager->release(timeout.control) == rx7900::memory::Status::deferred &&
           !timeout.manager->empty(), "timeout quarantines the ring and keeps leases");

    Fixture rejected;
    rejected.device->ring_status = 0x80000005u;
    expect(rejected.ring->create(*rejected.manager, rejected.control, rejected.firmware) ==
           PspRingStatus::unexpected_response && rejected.ring->report().memory_retained,
           "ring-create error status quarantines published memory");
    expect(rejected.ring->create(*rejected.manager, rejected.control, rejected.firmware) ==
           PspRingStatus::quarantined, "quarantined ring cannot be recreated");

    Fixture not_ready;
    not_ready.device->reg(rx7900_test::mp_base + 0x80) = 0;
    expect(not_ready.ring->create(*not_ready.manager, not_ready.control, not_ready.firmware) ==
           PspRingStatus::timeout && !not_ready.ring->report().memory_retained &&
           not_ready.device->reg(rx7900_test::mp_base + 0x85) == 0,
           "SOS not ready: nothing published, leases returned");

    Fixture misaligned;
    rx7900::memory::Handle small{};
    rx7900::memory::Handle filler{};
    // First fit would otherwise land on the next 1 MiB boundary after the firmware buffer.
    misaligned.manager->allocate(0x1000, 0x200000, rx7900::memory::Access::read_write, true, filler);
    const size_t events_before = misaligned.device->events.size();
    misaligned.manager->allocate(psp_firmware_buffer_bytes, 0x1000,
                                 rx7900::memory::Access::read_write, true, small);
    expect(misaligned.ring->create(*misaligned.manager, misaligned.control, small) ==
           PspRingStatus::invalid_memory && misaligned.device->events.size() == events_before &&
           misaligned.device->hdp_flushes == 0,
           "firmware buffer must be 1 MiB aligned; no PSP access on refusal");

    VramLocation elsewhere = rx7900_test::navi31_vram();
    elsewhere.mc_start += 1ULL << 30;
    Fixture outside(elsewhere);
    expect(outside.ring->create(*outside.manager, outside.control, outside.firmware) ==
           PspRingStatus::invalid_memory, "memory outside local VRAM MC range refused");

    Fixture torn;
    torn.ring->create(*torn.manager, torn.control, torn.firmware);
    torn.device->reg(rx7900_test::mp_base + 0x83) = 5;
    expect(torn.ring->autoload_rlc() == PspRingStatus::unexpected_response &&
           torn.device->commands.empty(), "write pointer inside a frame is refused");

    Fixture uninitialized;
    PspRing fresh;
    expect(fresh.create(*uninitialized.manager, uninitialized.control, uninitialized.firmware) ==
           PspRingStatus::busy, "uninitialized ring refuses creation");
    expect(fresh.initialize(PspRegisters{}, uninitialized.device->io(), uninitialized.nbio,
                            rx7900_test::navi31_vram(), {1, 1, 1}, {}) == PspRingStatus::unsupported_ip,
           "invalid MP0 token refused");
    expect(fresh.initialize(uninitialized.psp, uninitialized.device->io(), uninitialized.nbio,
                            rx7900_test::navi31_vram(), {0, 1, 1}, {}) == PspRingStatus::unavailable,
           "zero poll policy refused");
}

void trusted_app_commands() {
    Fixture fixture;
    auto& device = *fixture.device;
    const uint64_t firmware = fixture.address(fixture.firmware);
    fixture.ring->create(*fixture.manager, fixture.control, fixture.firmware);
    rx7900::memory::Handle shared{};
    fixture.manager->allocate(0x4000, psp_ta_shared_alignment, rx7900::memory::Access::read_write,
                              true, shared);
    const uint64_t shared_mc = fixture.address(shared);

    // Fill the staging buffer first so the clear of psp_copy_fw is observable.
    const auto large = pattern(0x2000, 1);
    fixture.ring->load_ip_firmware({large.data(), large.size()}, 87);
    const auto ta = pattern(0x63, 5);
    uint32_t session = 0xdead;
    expect(fixture.ring->load_ta({ta.data(), ta.size()}, shared_mc, 0x4000, session) ==
           PspRingStatus::completed && session == 0, "LOAD_TA returns resp.session_id");
    const auto& load = device.commands.back();
    uint32_t staged = 0;
    ByteView(ta.data(), ta.size()).read_u32(0, staged);
    expect(load.id == 1 && load.words[0] == static_cast<uint32_t>(firmware) &&
           load.words[1] == static_cast<uint32_t>(firmware >> 32) && load.words[2] == 0x63 &&
           load.words[3] == static_cast<uint32_t>(shared_mc) &&
           load.words[4] == static_cast<uint32_t>(shared_mc >> 32) && load.words[5] == 0x4000 &&
           load.first_payload_word == staged && load.flushed,
           "psp_prep_ta_load_cmd_buf: fw buffer address, app_len, shared address and length");
    expect(device.vram_u32(firmware + 0x60) == (0u | ta.at(0x60) | (ta.at(0x61) << 8) |
                                                (ta.at(0x62) << 16)) &&
           device.vram_u32(firmware + 0x64) == 0 && device.vram_u32(firmware + 0x1ffc) == 0,
           "TA staged as psp_copy_fw: payload in front, previous payload cleared");

    uint32_t asd_session = 0;
    expect(fixture.ring->load_asd({ta.data(), ta.size()}, asd_session) == PspRingStatus::completed &&
           asd_session == 0x77 && device.commands.back().id == 4 &&
           device.commands.back().words[2] == 0x63 && device.commands.back().words[3] == 0 &&
           device.commands.back().words[4] == 0 && device.commands.back().words[5] == 0,
           "LOAD_ASD keeps its frame on the shared path: no shared buffer");

    const size_t sent = device.commands.size();
    expect(fixture.ring->load_ta({ta.data(), ta.size()}, shared_mc + 0x10, 0x4000, session) ==
               PspRingStatus::invalid_memory &&
           fixture.ring->load_ta({ta.data(), ta.size()}, shared_mc, 0, session) ==
               PspRingStatus::invalid_memory &&
           fixture.ring->load_ta({ta.data(), ta.size()}, rx7900_test::navi31_vram().mc_end + 1,
                                 0x4000, session) == PspRingStatus::invalid_memory &&
           fixture.ring->load_ta({ta.data(), ta.size()}, rx7900_test::navi31_vram().mc_end & ~0xfffull,
                                 0x2000, session) == PspRingStatus::invalid_memory &&
           fixture.ring->load_ta({nullptr, 0}, shared_mc, 0x4000, session) ==
               PspRingStatus::invalid_payload && device.commands.size() == sent,
           "misaligned, empty, out-of-VRAM shared buffers and empty TA refused before PSP");

    device.psp_response_status = 0xa5;
    session = 0xdead;
    expect(fixture.ring->load_ta({ta.data(), ta.size()}, shared_mc, 0x4000, session) ==
           PspRingStatus::command_failed && session == 0xdead &&
           fixture.ring->report().response_status == 0xa5,
           "nonzero LOAD_TA status fails closed without a session");
    device.psp_response_status = 0;

    uint32_t raw = 0xffff;
    expect(fixture.ring->boot_config_get(raw) == PspRingStatus::completed && raw == 0 &&
           device.commands.back().id == 0x22 && device.commands.back().words[0] == 0 &&
           device.commands.back().words[1] == 2 && device.commands.back().words[2] == 0 &&
           device.commands.back().words[3] == 0, "BOOT_CFG get: sub_cmd BOOTCFG_CMD_GET only");
    expect(fixture.ring->boot_config_set(psp_boot_config_gecc) == PspRingStatus::completed &&
           device.commands.back().id == 0x22 && device.commands.back().words[1] == 1 &&
           device.commands.back().words[2] == 1 && device.commands.back().words[3] == 1,
           "BOOT_CFG set GECC: boot_config and boot_config_valid both 1");
    expect(fixture.ring->boot_config_set(0) == PspRingStatus::completed &&
           device.commands.back().words[1] == 1 && device.commands.back().words[2] == 0 &&
           device.commands.back().words[3] == 0, "BOOT_CFG set 0 disables GECC");
    expect(fixture.ring->unload_ta(0x1234) == PspRingStatus::completed &&
           device.commands.back().id == 2 && device.commands.back().words[0] == 0x1234,
           "UNLOAD_TA carries the session id");

    fixture.ring->stop();
    expect(fixture.ring->load_ta({ta.data(), ta.size()}, shared_mc, 0x4000, session) ==
           PspRingStatus::busy && fixture.ring->boot_config_get(raw) == PspRingStatus::busy,
           "stopped ring refuses TA and boot-config commands");
}

} // namespace

int main() {
    creation_and_commands();
    failures();
    trusted_app_commands();
    return checks.finish("psp_ring");
}

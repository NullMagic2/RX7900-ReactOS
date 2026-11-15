/*
Module: rx7900.tests.gfx11_init
Purpose: Verify the GC11 autoload wait, RS64 setup, constants, clear state and CP start.
Created: 2026-10-21
Architecture: Synthetic register/VRAM machine with a write trace; no GPU is programmed.
*/
#include <memory>
#include <vector>

#include "asic_test_device.hpp"
#include "gfx11_init.hpp"
#include "gfx11_clear_state.hpp"

namespace {

using namespace rx7900::hardware;
using rx7900_test::SimulatedNavi31;

rx7900_test::Checks checks;

void expect(bool condition, const char* contract) { checks.expect(condition, contract); }

constexpr uint32_t seg0 = rx7900_test::gc_base;
constexpr uint32_t seg1 = 0xa000;

struct Trace final {
    SimulatedNavi31* device = nullptr;
    std::vector<std::pair<uint32_t, uint32_t>> writes;

    static bool read(void* context, uint32_t offset, uint32_t& value) noexcept {
        return SimulatedNavi31::read(static_cast<Trace*>(context)->device, offset, value);
    }

    static bool write(void* context, uint32_t offset, uint32_t value) noexcept {
        auto& self = *static_cast<Trace*>(context);
        self.writes.emplace_back(offset / 4, value);
        return SimulatedNavi31::write(self.device, offset, value);
    }

    static void delay(void* context, uint32_t microseconds) noexcept {
        SimulatedNavi31::delay(static_cast<Trace*>(context)->device, microseconds);
    }

    // Values written to one dword register, in order.
    std::vector<uint32_t> values(uint32_t index) const {
        std::vector<uint32_t> out;

        for (const auto& write : writes) {
            if (write.first == index) {
                out.push_back(write.second);
            }
        }

        return out;
    }
};

struct Fixture final {
    std::unique_ptr<SimulatedNavi31> device = std::make_unique<SimulatedNavi31>();
    rx7900_test::VramBackend backend{};
    std::unique_ptr<rx7900::memory::Manager> manager = std::make_unique<rx7900::memory::Manager>();
    Trace trace{};
    Nbio nbio{};
    std::unique_ptr<Gfx11Init> gfx = std::make_unique<Gfx11Init>();
    rx7900::memory::Handle clear_state{};

    Fixture() {
        backend.device = device.get();
        trace.device = device.get();
        manager->initialize({rx7900_test::fb_base + rx7900_test::arena_offset,
                             rx7900_test::arena_bytes}, backend.backend());
        const auto binary = rx7900_test::navi31_discovery();
        NbioRegisters nbio_registers;
        validate_nbio_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes,
                                nbio_registers);
        nbio.initialize(nbio_registers, device->io());
        nbio.program_common();
        rx7900::memory::GcRegisterBase gc;
        rx7900::memory::validate_gc_discovery({binary.data(), binary.size()},
                                              rx7900_test::aperture_bytes, gc);
        device->reg(seg1 + 0x4e82) = 0x80000000u;
        device->reg(seg0 + 0x13de) = 0x00000444u;
        device->reg(seg1 + 0x19a2) = 0x00000005u;
        device->reg(seg1 + 0x803) = 0x15000000u;
        device->reg(seg1 + 0x2904) = 0x40000000u;
        device->reg(seg1 + 0x4068) = 0x0b000012u;
        gfx->initialize(gc, {&trace, rx7900_test::aperture_bytes, Trace::read, Trace::write, Trace::delay},
                        nbio, rx7900_test::navi31_vram(), {100}, {device.get(), SimulatedNavi31::confirm});
        manager->allocate((Gfx11Init::clear_state_bytes() + 4095) & ~4095u, 0x1000,
                          rx7900::memory::Access::read_write, true, clear_state);
    }
};

void sequence() {
    Fixture f;
    expect(f.gfx->configure_rs64({}) == Gfx11Status::busy, "RS64 setup waits for autoload");
    expect(f.gfx->wait_for_autoload() == Gfx11Status::completed && f.gfx->report().autoload_complete,
           "CP idle and BOOTLOAD_COMPLETE observed");
    expect(f.gfx->read_gb_addr_config() == Gfx11Status::completed &&
           f.gfx->report().gb_addr_config == 0x444 && !f.gfx->report().gb_addr_config_invalid,
           "GB_ADDR_CONFIG captured");

    const Gfx11StartAddresses addresses{0x1000, 0x1, 0x2000, 0x2, 0x3000, 0x3};
    expect(f.gfx->configure_rs64(addresses) == Gfx11Status::completed, "RS64 program counters set");
    const auto pfp = f.trace.values(seg0 + 0x1e44);
    const auto mec = f.trace.values(seg1 + 0x2900);
    const auto mec_hi = f.trace.values(seg1 + 0x2938);
    const auto me_control = f.trace.values(seg1 + 0x803);
    const auto mec_control = f.trace.values(seg1 + 0x2904);
    const auto selects = f.trace.values(seg1 + 0x900);
    expect(pfp.size() == 2 && pfp.at(0) == ((1u << 30) | (0x1000u >> 2)) &&
           f.trace.values(seg0 + 0x1e59).at(1) == 0, "PFP start for both pipes: hi<<30 | lo>>2");
    expect(mec.size() == 4 && mec.at(3) == ((0x3000u >> 2) | (3u << 30)) && mec_hi.at(0) == 0,
           "MEC start for four pipes");
    expect(me_control.size() == 4 && me_control.at(0) == (0x15000000u | 0xc0000u) &&
           me_control.at(1) == 0x15000000u && me_control.at(2) == (0x15000000u | 0x300000u),
           "PFP then ME pipe reset pulses preserve other CP_ME_CNTL bits");
    expect(mec_control.size() == 2 && mec_control.at(0) == (0x40000000u | 0xf0000u) &&
           mec_control.at(1) == 0x40000000u, "MEC pipe reset pulse");
    expect(selects.size() == 11 && selects.at(0) == 0 && selects.at(1) == 1 && selects.at(6) == (1u << 2) &&
           selects.at(9) == ((1u << 2) | 3u) && selects.back() == 0,
           "GRBM_GFX_CNTL selects ME0/ME1 pipes and restores 0");

    expect(f.gfx->golden_registers() == Gfx11Status::completed &&
           f.device->reg(seg1 + 0x19a2) == 0x20000005u, "golden TCP_CNTL bit 29 set, others kept");
    expect(f.gfx->constants_init() == Gfx11Status::completed, "constants initialized");
    expect((f.device->reg(seg0 + 0xda0) & 0xff) == 0xff && f.device->reg(seg1 + 0x9e4) == 0xc00c,
           "GRBM read timeout and DEFAULT_SH_MEM_CONFIG");
    const auto bases = f.trace.values(seg1 + 0x9e3);
    expect(bases.size() == 15 + 8 && bases.at(0) == 0x20001000u && bases.back() == 0x00010002u,
           "SH_MEM_BASES for VMIDs 1..15, then KFD VMIDs 8..15 apertures");
    expect((f.device->reg(seg0 + 0x1f72) & 0x8) != 0 && f.trace.values(seg0 + 0x20a0 + 2 * 15).size() == 2,
           "KFD trap enable and GDS cleared by both VMID passes");

    expect(f.gfx->rlc_resume(*f.manager, f.clear_state) == Gfx11Status::completed, "RLC resumed");
    rx7900::memory::AllocationInfo info{};
    f.manager->describe(f.clear_state, info);
    const uint64_t csb = info.gpu.virtual_address;
    const uint32_t dwords = f.gfx->report().clear_state_dwords;
    expect(dwords == 12 + 7 * 2 + 934 && dwords * 4 == Gfx11Init::clear_state_bytes(),
           "clear-state size matches gfx_v11_0_get_csb_size");
    expect(f.device->vram_u32(csb) == 0xc0004a00u && f.device->vram_u32(csb + 4) == 0x20000000u &&
           f.device->vram_u32(csb + 8) == 0xc0012800u &&
           f.device->vram_u32(csb + 20) == ((3u << 30) | (0x69u << 8) | (215u << 16)) &&
           f.device->vram_u32(csb + 24) == 0, "PREAMBLE, CONTEXT_CONTROL and first SET_CONTEXT_REG");
    expect(f.device->vram_u32(csb + (dwords - 5) * 4) == 0 &&
           f.device->vram_u32(csb + (dwords - 2) * 4) == 0xc0001200u &&
           f.device->vram_u32(csb + (dwords - 6) * 4) == (seg1 + 0xd7 - 0xa000),
           "tile steering override, end preamble and CLEAR_STATE close the buffer");
    expect(f.device->reg(seg1 + 0x987) == static_cast<uint32_t>(csb) &&
           f.device->reg(seg1 + 0x988) == static_cast<uint32_t>(csb >> 32) &&
           f.device->reg(seg1 + 0x989) == dwords && (f.device->reg(seg1 + 0x4c80) & 3u) == 3u,
           "RLC_CSIB address/length and SRM enabled");

    f.device->reg(seg0 + 0x1e0a) = 0x003c0001u;
    expect(f.gfx->cp_start() == Gfx11Status::completed && f.gfx->report().cp_started &&
           !f.gfx->report().cp_gfx_enable_timeout && f.gfx->report().imu_version == 0x0b000012u,
           "CP started and IMU version read");
    expect(f.device->reg(seg0 + 0x1e0a) == 1u && f.device->reg(seg0 + 0x1dfa) == 0x458u &&
           f.device->reg(seg0 + 0x1dfb) == 0x7f8u && f.device->reg(seg0 + 0x1dfc) == 0 &&
           f.device->reg(seg0 + 0x1dfd) == 0x450u, "GUI idle interrupts off and CP doorbell ranges");
    expect(f.device->reg(seg1 + 0x2904) == 0x3c000000u && (f.device->reg(seg1 + 0x803) & 0x14000000u) == 0,
           "MEC pipes active and unhalted; ME and PFP unhalted");

    expect(f.gfx->release_after_reset() == Gfx11Status::quarantined, "release needs reset proof");
    f.device->reset_confirmed = true;
    expect(f.gfx->release_after_reset() == Gfx11Status::released, "confirmed reset releases");
}

void failures() {
    Fixture stuck;
    stuck.device->reg(seg1 + 0x4e82) = 0;
    expect(stuck.gfx->wait_for_autoload() == Gfx11Status::timeout &&
           stuck.gfx->golden_registers() == Gfx11Status::quarantined,
           "autoload timeout quarantines GFX bring-up");

    Fixture zero;
    zero.device->reg(seg0 + 0x13de) = 0;
    zero.gfx->wait_for_autoload();
    expect(zero.gfx->read_gb_addr_config() == Gfx11Status::completed &&
           zero.gfx->report().gb_addr_config_invalid, "zero GB_ADDR_CONFIG warned as Linux does");

    Fixture small;
    rx7900::memory::Handle tiny{};
    small.manager->allocate(0x1000, 0x1000, rx7900::memory::Access::read_write, true, tiny);
    small.manager->release(tiny);
    small.gfx->wait_for_autoload();
    small.gfx->configure_rs64({});
    expect(Gfx11Init::clear_state_bytes() == 3840 &&
           small.gfx->rlc_resume(*small.manager, tiny) == Gfx11Status::invalid_memory,
           "stale clear-state handle refused");

    Fixture writes;
    writes.gfx->wait_for_autoload();
    writes.device->writes_ok = false;
    expect(writes.gfx->configure_rs64({}) == Gfx11Status::mmio_failure &&
           writes.gfx->constants_init() == Gfx11Status::quarantined, "MMIO failure quarantines");

    Gfx11Init fresh;
    expect(fresh.wait_for_autoload() == Gfx11Status::unavailable, "uninitialized refused");
}

} // namespace

int main() {
    sequence();
    failures();
    return checks.finish("gfx11_init");
}

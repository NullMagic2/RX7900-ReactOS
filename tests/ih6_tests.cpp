/*
Module: rx7900.tests.ih6
Purpose: Verify IH6 ring programming order, enable/disable, wptr/rptr and quarantine.
Created: 2026-10-21
Architecture: Test-local OSSSYS/NBIO register model with the shared VRAM fixture;
expected values are derived by hand from Linux v6.12 ih_v6_0.c and nbio_v4_3.c.
*/
#include <map>
#include <memory>

#include "asic_test_device.hpp"
#include "ih6.hpp"

namespace {

using namespace rx7900::hardware;
using rx7900::ByteView;
using rx7900::memory::Access;
using rx7900::memory::Handle;

rx7900_test::Checks checks;

void expect(bool condition, const char* contract) { checks.expect(condition, contract); }

constexpr uint32_t osssys_base = 0x4260;
constexpr uint32_t nbif2 = rx7900_test::nbif_bases[2];
constexpr uint32_t nbif3 = rx7900_test::nbif_bases[3];
constexpr uint32_t rsmu_index = rx7900_test::nbif_bases[1];
constexpr uint32_t rsmu_data = rx7900_test::nbif_bases[1] + 1;
constexpr uint32_t hdp_flush_index = hdp_remap_byte_offset / 4;
constexpr uint32_t s2a_smn = (nbif3 + 0x4f7641u) * 4;
constexpr uint64_t dummy_page = 0x0000001234567000ULL;

constexpr uint32_t reg(uint32_t offset) { return osssys_base + offset; }

constexpr uint32_t rb_cntl = reg(0x80);
constexpr uint32_t rb_base = reg(0x81);
constexpr uint32_t rb_base_hi = reg(0x82);
constexpr uint32_t rb_rptr = reg(0x83);
constexpr uint32_t rb_wptr = reg(0x84);
constexpr uint32_t wptr_addr_hi = reg(0x85);
constexpr uint32_t wptr_addr_lo = reg(0x86);
constexpr uint32_t doorbell_rptr = reg(0x87);
constexpr uint32_t rb_cntl1 = reg(0x8c);
constexpr uint32_t rb_base1 = reg(0x8d);
constexpr uint32_t rb_base_hi1 = reg(0x8e);
constexpr uint32_t rb_rptr1 = reg(0x8f);
constexpr uint32_t rb_wptr1 = reg(0x90);
constexpr uint32_t doorbell_rptr1 = reg(0x93);
constexpr uint32_t cntl2 = reg(0xc1);
constexpr uint32_t flood = reg(0xd5);
constexpr uint32_t storm_list = reg(0xda);
constexpr uint32_t clk_ctrl = reg(0xdb);
constexpr uint32_t msi_storm = reg(0xf1);
constexpr uint32_t cfg_index = reg(0x185);
constexpr uint32_t cfg_data = reg(0x186);
constexpr uint32_t bif_cntl = nbif2 + 0xf1;
constexpr uint32_t bif_cntl2 = nbif2 + 0xf2;

struct Access32 final {
    enum class Kind { mmio, smn, bus_master, doorbell };
    Kind kind;
    uint32_t target;
    uint32_t value;

    bool operator==(const Access32& other) const {
        return kind == other.kind && target == other.target && value == other.value;
    }
};

Access32 mmio(uint32_t target, uint32_t value) { return {Access32::Kind::mmio, target, value}; }
Access32 smn(uint32_t target, uint32_t value) { return {Access32::Kind::smn, target, value}; }

// Plain register file; RSMU index/data model the SMN path for registers beyond BAR5.
class IhDevice final {
public:
    std::vector<uint32_t> registers = std::vector<uint32_t>(rx7900_test::aperture_bytes / 4, 0);
    std::map<uint32_t, uint32_t> smn_space;
    std::vector<Access32> log;
    std::vector<uint32_t> delays;
    uint32_t smn_address = 0;
    uint32_t fail_write = UINT32_MAX;
    uint32_t all_ones_read = UINT32_MAX;
    bool bus_master_ok = true;
    bool doorbell_ok = true;
    bool reset_confirmed = false;

    IhDevice() {
        // Hardware state left by a previous owner; unrelated bits must survive.
        at(rb_cntl) = 0x80020801u;
        at(rb_cntl1) = 0x00000801u;
        at(bif_cntl) = 0x00000005u;
        at(storm_list) = 0x00000001u;
        at(msi_storm) = 0x00010fffu;
        at(cfg_index) = 0x00000105u;
        at(cfg_data) = 0x00fe1234u;
        at(cntl2) = 0x00000213u;
        at(clk_ctrl) = 0x00800000u;
        smn_space[s2a_smn] = 0x0202007eu;
    }

    uint32_t& at(uint32_t index) { return registers.at(index); }

    RegisterIo io() noexcept { return {this, rx7900_test::aperture_bytes, read, write, delay}; }

    Ih6Platform platform(bool doorbells) noexcept {
        return {this, bus_master, doorbells ? doorbell : nullptr, confirm};
    }

    static bool read(void* context, uint32_t offset, uint32_t& value) noexcept {
        auto& self = *static_cast<IhDevice*>(context);
        const uint32_t index = offset / 4;

        if ((offset & 3) != 0 || index >= self.registers.size()) {
            return false;
        }

        if (index == self.all_ones_read) {
            value = UINT32_MAX;
            return true;
        }

        if (index == rsmu_data) {
            value = self.smn_space[self.smn_address];
            return true;
        }

        value = self.registers.at(index);
        return true;
    }

    static bool write(void* context, uint32_t offset, uint32_t value) noexcept {
        auto& self = *static_cast<IhDevice*>(context);
        const uint32_t index = offset / 4;

        if ((offset & 3) != 0 || index >= self.registers.size() || index == self.fail_write) {
            return false;
        }

        if (index == rsmu_index) {
            self.smn_address = value;
            self.registers.at(index) = value;
            return true;
        }

        if (index == rsmu_data) {
            self.smn_space[self.smn_address] = value;
            self.log.push_back(smn(self.smn_address, value));
            return true;
        }

        self.registers.at(index) = value;
        self.log.push_back(mmio(index, value));
        return true;
    }

    static void delay(void* context, uint32_t microseconds) noexcept {
        static_cast<IhDevice*>(context)->delays.push_back(microseconds);
    }

    static bool bus_master(void* context) noexcept {
        auto& self = *static_cast<IhDevice*>(context);
        self.log.push_back({Access32::Kind::bus_master, 0, 0});
        return self.bus_master_ok;
    }

    static bool doorbell(void* context, uint32_t index, uint32_t value) noexcept {
        auto& self = *static_cast<IhDevice*>(context);
        self.log.push_back({Access32::Kind::doorbell, index, value});
        return self.doorbell_ok;
    }

    static bool confirm(void* context) noexcept {
        return static_cast<IhDevice*>(context)->reset_confirmed;
    }
};

std::vector<uint8_t> ih_discovery(uint8_t osssys_major = 6, uint32_t base = osssys_base) {
    const auto& n = rx7900_test::nbif_bases;
    return rx7900_test::discovery_with({
        {discovery_nbif_hardware_id, 4, 3, {n[0], n[1], n[2], n[3], n[4], n[5]}},
        {discovery_mmhub_hardware_id, 3, 0, {rx7900_test::mmhub_base}},
        {discovery_osssys_hardware_id, osssys_major, 0, {base}},
    });
}

struct Fixture final {
    std::unique_ptr<rx7900_test::SimulatedNavi31> vram = std::make_unique<rx7900_test::SimulatedNavi31>();
    std::unique_ptr<IhDevice> device = std::make_unique<IhDevice>();
    rx7900_test::VramBackend backend{};
    std::unique_ptr<rx7900::memory::Manager> manager = std::make_unique<rx7900::memory::Manager>();
    Nbio nbio{};
    NbioRegisters nbio_registers{};
    Ih6Registers registers{};
    std::unique_ptr<Ih6> ih = std::make_unique<Ih6>();
    Handle ring0{};
    Handle ring1{};
    Handle writeback{};

    explicit Fixture(bool doorbells = false, bool msi = true) {
        backend.device = vram.get();
        manager->initialize({rx7900_test::fb_base + rx7900_test::arena_offset,
                             rx7900_test::arena_bytes}, backend.backend());
        const auto binary = ih_discovery();
        validate_nbio_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes,
                                nbio_registers);
        validate_ih6_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes,
                               registers);
        nbio.initialize(nbio_registers, device->io());
        nbio.program_common();
        manager->allocate(ih6_default_ring_bytes, ih6_ring_alignment, Access::read_write, true, ring0);
        manager->allocate(ih6_default_ring_bytes, ih6_ring_alignment, Access::read_write, true, ring1);
        manager->allocate(rx7900::memory::page_bytes, rx7900::memory::page_bytes,
                          Access::read_write, true, writeback);
        ih->initialize(registers, nbio_registers, device->io(), nbio, rx7900_test::navi31_vram(),
                       {ih6_default_ring_bytes, msi, dummy_page}, device->platform(doorbells));
        device->log.clear();
    }

    uint64_t address(Handle handle) const {
        rx7900::memory::AllocationInfo info{};
        manager->describe(handle, info);
        return info.gpu.virtual_address;
    }

    uint32_t leases(Handle handle) const {
        rx7900::memory::AllocationInfo info{};
        manager->describe(handle, info);
        return info.leases;
    }

    Ih6Status init() { return ih->hw_init(*manager, ring0, ring1, writeback); }
};

// ih_v6_0_irq_init expected trace for the fixture's initial register state.
std::vector<Access32> expected_init(const Fixture& fixture, bool doorbells) {
    const uint64_t g0 = fixture.address(fixture.ring0);
    const uint64_t g1 = fixture.address(fixture.ring1);
    const uint64_t wb = fixture.address(fixture.writeback);
    return {
        // toggle_interrupts(false): ring 0 then ring 1.
        mmio(rb_cntl, 0x80000800u), mmio(rb_rptr, 0), mmio(rb_wptr, 0),
        mmio(rb_cntl1, 0x00000800u), mmio(rb_rptr1, 0), mmio(rb_wptr1, 0),
        // nbio_v4_3_ih_control.
        mmio(bif_cntl2, 0x12345670u), mmio(bif_cntl, 0x0000000cu),
        // enable_ring(ih).
        mmio(rb_base, static_cast<uint32_t>(g0 >> 8)), mmio(rb_base_hi, static_cast<uint32_t>(g0 >> 40) & 0xff),
        mmio(rb_cntl, 0xc0310920u),
        mmio(wptr_addr_lo, static_cast<uint32_t>(wb)), mmio(wptr_addr_hi, static_cast<uint32_t>(wb >> 32) & 0xffff),
        mmio(rb_wptr, 0), mmio(rb_rptr, 0), mmio(doorbell_rptr, doorbells ? 0x100002f0u : 0),
        // enable_ring(ih1).
        mmio(rb_base1, static_cast<uint32_t>(g1 >> 8)), mmio(rb_base_hi1, static_cast<uint32_t>(g1 >> 40) & 0xff),
        mmio(rb_cntl1, 0xc0100b20u),
        mmio(rb_wptr1, 0), mmio(rb_rptr1, 0), mmio(doorbell_rptr1, doorbells ? 0x100002f2u : 0),
        // nbio_v4_3_ih_doorbell_range through SMN.
        smn(s2a_smn, doorbells ? 0x02057841u : 0x0200007eu),
        mmio(storm_list, 0x00040001u), mmio(flood, 0x00000008u), mmio(msi_storm, 0x00010003u),
        mmio(cfg_index, 0x00000100u), mmio(cfg_data, 0x00ff000au),
        {Access32::Kind::bus_master, 0, 0},
        // toggle_interrupts(true).
        mmio(rb_cntl, 0x40310921u), mmio(rb_cntl, 0xc0310921u), mmio(rb_cntl, 0x40330921u),
        mmio(rb_cntl1, 0x40100b21u), mmio(rb_cntl1, 0xc0100b21u), mmio(rb_cntl1, 0x40100b21u),
        // force_update_wptr_for_self_int(0, 8, true).
        mmio(rb_cntl1, 0x40100b21u), mmio(cntl2, 0x00000308u),
    };
}

void discovery_validation() {
    Ih6Registers registers;
    auto binary = ih_discovery();
    expect(validate_ih6_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes,
                                  registers) == DiscoveryStatus::validated && registers.valid() &&
           registers.segment0_dword_base() == osssys_base, "OSSSYS 6.0.0 token validated");
    binary = ih_discovery(5);
    expect(validate_ih6_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes,
                                  registers) != DiscoveryStatus::validated && !registers.valid(),
           "OSSSYS 5.x rejected");
    binary = rx7900_test::discovery_with({
        {discovery_nbif_hardware_id, 4, 3, {0, 0x14, 0xd20, 0x10400, 0x241b000, 0x4040000}}});
    expect(validate_ih6_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes,
                                  registers) != DiscoveryStatus::validated, "missing OSSSYS rejected");
    binary = ih_discovery(6, rx7900_test::aperture_bytes / 4 - 0x100);
    expect(validate_ih6_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes,
                                  registers) == DiscoveryStatus::invalid_argument,
           "IH_CHICKEN beyond BAR5 rejected");
    binary = ih_discovery();
    expect(validate_ih6_discovery({binary.data(), binary.size()}, 6, registers) ==
           DiscoveryStatus::invalid_argument, "unaligned aperture rejected");
}

void initialize_arguments() {
    Fixture fixture;
    auto& device = *fixture.device;
    const auto vram = rx7900_test::navi31_vram();
    const auto attempt = [&](Ih6Config config, Ih6Platform platform) {
        Ih6 ih;
        return ih.initialize(fixture.registers, fixture.nbio_registers, device.io(), fixture.nbio,
                             vram, config, platform);
    };

    expect(attempt({ih6_default_ring_bytes, true, dummy_page}, device.platform(false)) ==
           Ih6Status::ready, "valid configuration ready");
    expect(attempt({ih6_default_ring_bytes - 4096, true, dummy_page}, device.platform(false)) ==
           Ih6Status::invalid_argument, "non power-of-two ring rejected");
    expect(attempt({2 * ih6_maximum_ring_bytes, true, dummy_page}, device.platform(false)) ==
           Ih6Status::invalid_argument, "ring beyond WPTR OFFSET range rejected");
    expect(attempt({2048, true, dummy_page}, device.platform(false)) ==
           Ih6Status::invalid_argument, "sub-page ring rejected");
    expect(attempt({ih6_default_ring_bytes, true, 0}, device.platform(false)) ==
           Ih6Status::invalid_argument, "zero dummy page rejected");
    expect(attempt({ih6_default_ring_bytes, true, dummy_page + 0x100}, device.platform(false)) ==
           Ih6Status::invalid_argument, "unaligned dummy page rejected");
    expect(attempt({ih6_default_ring_bytes, true, 1ULL << 40}, device.platform(false)) ==
           Ih6Status::invalid_argument, "dummy page beyond CNTL2 range rejected");
    Ih6Platform no_master = device.platform(false);
    no_master.enable_bus_master = nullptr;
    expect(attempt({ih6_default_ring_bytes, true, dummy_page}, no_master) == Ih6Status::unavailable,
           "bus-master callback required");
    Ih6 ih;
    expect(ih.initialize({}, fixture.nbio_registers, device.io(), fixture.nbio, vram,
                         {ih6_default_ring_bytes, true, dummy_page}, device.platform(false)) ==
           Ih6Status::unsupported_ip, "invalid OSSSYS token rejected");
    expect(ih.hw_init(*fixture.manager, fixture.ring0, fixture.ring1, fixture.writeback) ==
           Ih6Status::unavailable, "hw_init requires initialize");
    expect(device.log.empty(), "argument checks touch no registers");
}

void register_mode_sequence() {
    Fixture fixture;
    auto& device = *fixture.device;
    fixture.vram->vram.assign(fixture.vram->vram.size(), 0xa5);

    expect(fixture.init() == Ih6Status::enabled, "hw_init enables both rings");
    auto expected = expected_init(fixture, false);
    expected.insert(expected.begin(), mmio(hdp_flush_index, 0));
    expect(device.log == expected, "register-mode trace matches ih_v6_0_irq_init order and values");

    if (!(device.log == expected)) {
        for (size_t index = 0; index < device.log.size(); ++index) {
            std::fprintf(stderr, "  %zu: kind %d target %#x value %#x\n", index,
                         static_cast<int>(device.log[index].kind), device.log[index].target,
                         device.log[index].value);
        }
    }

    const Ih6Report report = fixture.ih->report();
    expect(report.enabled && report.memory_retained && !report.doorbell_mode &&
           report.ring_mc[0] == fixture.address(fixture.ring0) &&
           report.writeback_mc == fixture.address(fixture.writeback) &&
           report.rb_cntl[0] == 0x40330921u && report.rb_cntl[1] == 0x40100b21u &&
           report.doorbell_range == 0x0200007eu && report.interrupt_cntl == 0x0000000cu,
           "report mirrors programmed state");
    expect(fixture.leases(fixture.ring0) == 1 && fixture.leases(fixture.ring1) == 1 &&
           fixture.leases(fixture.writeback) == 1, "one lease on each allocation");
    expect(fixture.vram->vram_u32(fixture.address(fixture.ring0)) == 0 &&
           fixture.vram->vram_u32(fixture.address(fixture.ring1) + ih6_default_ring_bytes - 4) == 0 &&
           fixture.vram->vram_u32(fixture.address(fixture.writeback)) == 0,
           "rings and writeback zeroed before enable");

    ByteView view(nullptr, 0);
    expect(fixture.ih->ring_view(Ih6Ring::primary, view) && view.size() == ih6_default_ring_bytes,
           "ring 0 CPU view exposed");
    expect(fixture.ih->hw_init(*fixture.manager, fixture.ring0, fixture.ring1, fixture.writeback) ==
           Ih6Status::busy, "second hw_init refused while enabled");

    device.log.clear();
    expect(fixture.ih->set_rptr(Ih6Ring::primary, 0x40) == Ih6Status::enabled &&
           fixture.ih->set_rptr(Ih6Ring::page_fault, 0x20) == Ih6Status::enabled,
           "rptr acknowledged");
    expect(device.log == std::vector<Access32>{mmio(rb_rptr, 0x40), mmio(rb_rptr1, 0x20)},
           "register mode acknowledges through IH_RB_RPTR registers");
    expect(fixture.ih->set_rptr(Ih6Ring::primary, ih6_default_ring_bytes) == Ih6Status::invalid_argument &&
           fixture.ih->set_rptr(Ih6Ring::primary, 0x44) == Ih6Status::invalid_argument &&
           device.log.size() == 2, "out-of-range or unaligned rptr rejected without access");
    expect(fixture.ih->rptr(Ih6Ring::primary) == 0x40, "software rptr tracked");
}

void doorbell_mode_sequence() {
    Fixture fixture(true);
    auto& device = *fixture.device;

    expect(fixture.init() == Ih6Status::enabled, "doorbell-mode hw_init enabled");
    auto expected = expected_init(fixture, true);
    expected.insert(expected.begin(), mmio(hdp_flush_index, 0));
    expect(device.log == expected, "doorbell trace: IH_DOORBELL_RPTR and S2A range per Linux");
    device.log.clear();
    expect(fixture.ih->set_rptr(Ih6Ring::primary, 0x100) == Ih6Status::enabled &&
           fixture.ih->set_rptr(Ih6Ring::page_fault, 0x80) == Ih6Status::enabled,
           "doorbell rptr accepted");
    expect(device.log == std::vector<Access32>{{Access32::Kind::doorbell, 0x2f0, 0x100},
                                               {Access32::Kind::doorbell, 0x2f2, 0x80}},
           "WDOORBELL32 at (0x178 << 1) and ((0x178 + 1) << 1)");
    device.doorbell_ok = false;
    expect(fixture.ih->set_rptr(Ih6Ring::primary, 0x120) == Ih6Status::platform_failure &&
           fixture.ih->report().status == Ih6Status::platform_failure,
           "failed doorbell quarantines");
    expect(fixture.ih->set_rptr(Ih6Ring::primary, 0x140) == Ih6Status::quarantined,
           "quarantined protocol refuses rptr");
}

void msi_disabled_rearm() {
    Fixture fixture(false, false);

    expect(fixture.init() == Ih6Status::enabled, "hw_init without MSI");
    expect((fixture.ih->report().rb_cntl[0] & 0x00200000u) == 0 &&
           (fixture.device->at(rb_cntl) & 0x00200000u) == 0, "RPTR_REARM follows msi_enabled");
}

void write_pointer() {
    Fixture fixture;
    auto& device = *fixture.device;
    fixture.init();
    const uint64_t wb = fixture.address(fixture.writeback);
    uint32_t wptr = 0;

    fixture.vram->put_vram_u32(wb, 0x00000440u);
    expect(fixture.ih->read_wptr(Ih6Ring::primary, wptr) == Ih6Status::enabled && wptr == 0x440,
           "ring 0 wptr from writeback");
    expect(!fixture.ih->record_self_interrupt(0, 0x60), "self interrupt for ring 0 ignored");
    expect(fixture.ih->record_self_interrupt(1, 0x60) &&
           fixture.ih->read_wptr(Ih6Ring::page_fault, wptr) == Ih6Status::enabled && wptr == 0x60,
           "ring 1 wptr from self interrupt");

    fixture.vram->put_vram_u32(wb, 0x00000101u);
    device.at(rb_wptr) = 0x00000200u;
    device.log.clear();
    expect(fixture.ih->read_wptr(Ih6Ring::primary, wptr) == Ih6Status::enabled && wptr == 0x200 &&
           device.log.empty(), "stale writeback overflow resolved by IH_RB_WPTR");

    device.at(rb_wptr) = 0x00000301u;
    const uint32_t cntl = device.at(rb_cntl);
    expect(fixture.ih->read_wptr(Ih6Ring::primary, wptr) == Ih6Status::overflow && wptr == 0x300,
           "confirmed overflow reported");
    expect(fixture.ih->rptr(Ih6Ring::primary) == 0x320 && fixture.ih->report().overflows == 1,
           "rptr resumes at wptr + 32");
    expect(device.log == std::vector<Access32>{mmio(rb_cntl, cntl | 0x80000000u),
                                               mmio(rb_cntl, cntl & ~0x80000000u)},
           "WPTR_OVERFLOW_CLEAR pulsed");

    fixture.ih->record_self_interrupt(1, 0x0003ffe1u);
    device.at(rb_wptr1) = 0x0003ffe1u;
    expect(fixture.ih->read_wptr(Ih6Ring::page_fault, wptr) == Ih6Status::overflow &&
           wptr == 0x3ffe0 && fixture.ih->rptr(Ih6Ring::page_fault) == 0,
           "ring 1 overflow wraps rptr inside the ring");
}

void disable_resume_release() {
    Fixture fixture;
    auto& device = *fixture.device;
    fixture.init();
    device.log.clear();

    expect(fixture.ih->disable() == Ih6Status::disabled, "irq_disable completes");
    expect(device.log == std::vector<Access32>{
               mmio(rb_cntl1, 0x40100b21u), mmio(cntl2, 0x00000208u),
               mmio(rb_cntl, 0x40310920u), mmio(rb_rptr, 0), mmio(rb_wptr, 0),
               mmio(rb_cntl1, 0x40100b20u), mmio(rb_rptr1, 0), mmio(rb_wptr1, 0)},
           "disable order: self-int off, ring 0 off, ring 1 off");
    expect(!device.delays.empty() && device.delays.back() == 1000, "1 ms settle delay");
    expect(!fixture.ih->report().enabled && fixture.ih->report().memory_retained,
           "disabled rings keep leases");
    ByteView view(nullptr, 0);
    uint32_t wptr = 0;
    expect(!fixture.ih->ring_view(Ih6Ring::primary, view) &&
           fixture.ih->read_wptr(Ih6Ring::primary, wptr) == Ih6Status::unavailable,
           "disabled ring exposes no view or wptr");
    expect(fixture.ih->disable() == Ih6Status::busy, "double disable refused");

    device.log.clear();
    expect(fixture.ih->resume() == Ih6Status::enabled, "resume repeats irq_init");
    expect(device.log.size() == expected_init(fixture, false).size() + 1, "resume trace length");
    fixture.ih->disable();
    expect(fixture.ih->release() == Ih6Status::released && fixture.leases(fixture.ring0) == 0 &&
           fixture.leases(fixture.ring1) == 0 && fixture.leases(fixture.writeback) == 0,
           "release after disable drops leases");
    expect(fixture.init() == Ih6Status::enabled, "hw_init again after release");
    expect(fixture.ih->release() == Ih6Status::busy, "release refused while enabled");
}

void clockgating() {
    Fixture fixture;
    auto& device = *fixture.device;
    fixture.init();
    device.log.clear();

    expect(fixture.ih->set_clockgating(true) == Ih6Status::enabled && device.log.empty(),
           "gate with no overrides set writes nothing");
    expect(fixture.ih->set_clockgating(false) == Ih6Status::enabled &&
           device.log == std::vector<Access32>{mmio(clk_ctrl, 0xf8800000u)},
           "ungate sets the five IH_CLK_CTRL soft overrides");
    device.log.clear();
    expect(fixture.ih->set_clockgating(true) == Ih6Status::enabled &&
           device.log == std::vector<Access32>{mmio(clk_ctrl, 0x00800000u)} &&
           fixture.ih->report().clock_gated, "gate clears only those overrides");
}

void memory_validation() {
    Fixture fixture;
    auto& device = *fixture.device;
    Handle small{};
    fixture.manager->allocate(rx7900::memory::page_bytes, rx7900::memory::page_bytes,
                              Access::read_write, true, small);

    expect(fixture.ih->hw_init(*fixture.manager, small, fixture.ring1, fixture.writeback) ==
           Ih6Status::invalid_memory, "ring smaller than ring_bytes rejected");
    expect(fixture.ih->hw_init(*fixture.manager, fixture.ring0, fixture.ring0, fixture.writeback) ==
           Ih6Status::invalid_memory, "overlapping rings rejected");
    expect(fixture.ih->hw_init(*fixture.manager, fixture.ring0, fixture.ring1, Handle{}) ==
           Ih6Status::invalid_memory, "stale writeback handle rejected");
    expect(device.log.empty() && fixture.leases(fixture.ring0) == 0 &&
           fixture.leases(fixture.ring1) == 0 && fixture.leases(small) == 0,
           "memory failures roll back leases and touch no registers");
    expect(fixture.init() == Ih6Status::enabled, "valid memory still accepted afterwards");
}

void failures_quarantine() {
    {
        Fixture fixture;
        auto& device = *fixture.device;
        device.fail_write = rb_base1;

        expect(fixture.init() == Ih6Status::mmio_failure, "failed ring 1 base write reported");
        expect(device.log.back() == mmio(doorbell_rptr, 0), "no write after the failed store");
        uint32_t wptr = 0;
        expect(fixture.ih->read_wptr(Ih6Ring::primary, wptr) == Ih6Status::quarantined &&
               fixture.ih->disable() == Ih6Status::busy &&
               fixture.ih->release() == Ih6Status::quarantined &&
               fixture.ih->set_clockgating(true) == Ih6Status::quarantined,
               "quarantined protocol refuses service");
        expect(fixture.ih->initialize(fixture.registers, fixture.nbio_registers, device.io(),
                                      fixture.nbio, rx7900_test::navi31_vram(),
                                      {ih6_default_ring_bytes, true, dummy_page},
                                      device.platform(false)) == Ih6Status::busy,
               "re-initialize refused while quarantined");
        expect(fixture.leases(fixture.ring0) == 1, "leases kept while quarantined");
        expect(fixture.ih->release_after_reset() == Ih6Status::quarantined &&
               fixture.leases(fixture.ring0) == 1, "unconfirmed reset keeps leases");
        device.reset_confirmed = true;
        expect(fixture.ih->release_after_reset() == Ih6Status::released &&
               fixture.leases(fixture.ring0) == 0 && fixture.leases(fixture.ring1) == 0 &&
               fixture.leases(fixture.writeback) == 0, "confirmed reset releases leases");
    }
    {
        Fixture fixture;
        auto& device = *fixture.device;
        device.bus_master_ok = false;

        expect(fixture.init() == Ih6Status::platform_failure, "bus-master failure reported");
        expect(device.log.back().kind == Access32::Kind::bus_master &&
               (device.at(rb_cntl) & 1u) == 0, "rings never enabled without bus mastering");
    }
    {
        Fixture fixture;
        fixture.device->all_ones_read = rb_cntl;

        expect(fixture.init() == Ih6Status::mmio_failure &&
               fixture.device->log.size() == 1, "all-ones RB_CNTL read stops before any IH write");
    }
    {
        Fixture fixture;
        fixture.device->fail_write = rsmu_data;

        expect(fixture.init() == Ih6Status::mmio_failure &&
               fixture.ih->report().memory_retained, "failed SMN doorbell-range write quarantines");
    }
}

} // namespace

int main() {
    discovery_validation();
    initialize_arguments();
    register_mode_sequence();
    doorbell_mode_sequence();
    msi_disabled_rearm();
    write_pointer();
    disable_resume_release();
    clockgating();
    memory_validation();
    failures_quarantine();
    return checks.finish("ih6_tests");
}

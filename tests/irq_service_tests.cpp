/*
Module: rx7900.tests.irq_service
Purpose: Verify IH6 ring 0/1 servicing, vector routing and interrupt-source enables.
Created: 2026-10-21
Architecture: Test-local BAR5 register file and a real Ih6 over the shared VRAM
fixture with a synthetic 4 KiB ring; expected values derived by hand from Linux
v6.12 amdgpu_ih.c, amdgpu_irq.c, ih_v6_0.c, gfx_v11_0.c, sdma_v6_0.c, gmc_v11_0.c,
smu_v13_0.c and the gc_11_0_0/mmhub_3_0_0/mp_13_0_0 headers. The drain consumer
tests route decoded vectors into a real FenceManager.
*/
#include <algorithm>
#include <map>
#include <memory>

#include "asic_test_device.hpp"
#include "fence.hpp"
#include "irq_service.hpp"

namespace {

using namespace rx7900::hardware;
using rx7900::memory::Access;
using rx7900::memory::Handle;

rx7900_test::Checks checks;

void expect(bool condition, const char* contract) { checks.expect(condition, contract); }

constexpr uint32_t osssys_base = 0x4260;
constexpr uint32_t gc = rx7900_test::gc_base;
constexpr uint32_t mmhub = rx7900_test::mmhub_base;
constexpr uint32_t mp1 = rx7900_test::mp_base;
constexpr uint32_t rsmu_index = rx7900_test::nbif_bases[1];
constexpr uint32_t rsmu_data = rx7900_test::nbif_bases[1] + 1;
constexpr uint32_t ring_bytes = 4096;
constexpr uint32_t entry = 32;
constexpr uint64_t dummy_page = 0x0000001234567000ULL;

constexpr uint32_t rb_cntl = osssys_base + 0x80;
constexpr uint32_t rb_rptr = osssys_base + 0x83;
constexpr uint32_t rb_wptr = osssys_base + 0x84;
constexpr uint32_t ring0_doorbell = 0x178u << 1;
constexpr uint32_t ring1_doorbell = (0x178u + 1) << 1;

constexpr uint32_t cp_int_cntl_ring0 = gc + 0x1e0a;
constexpr uint32_t cp_int_cntl_ring1 = gc + 0x1e0b;
constexpr uint32_t cp_me1_pipe0 = gc + 0x1e25;
constexpr uint32_t sdma0_cntl = gc + 0x1c;
constexpr uint32_t sdma1_cntl = gc + 0x61c;
constexpr uint32_t mmvm_context0_cntl = mmhub + 0x740;
constexpr uint32_t gcvm_context0_cntl = gc + 0x1688;
constexpr uint32_t mmvm_fault_cntl = mmhub + 0x708;
constexpr uint32_t mmvm_fault_status = mmhub + 0x70c;
constexpr uint32_t gcvm_fault_cntl = gc + 0x15c4;
constexpr uint32_t gcvm_fault_status = gc + 0x15c8;
constexpr uint32_t mp1_sw_int_ctrl = mp1 + 0x2c3;

struct Access32 final {
    enum class Kind { write, doorbell };
    Kind kind;
    uint32_t target;
    uint32_t value;

    bool operator==(const Access32& other) const {
        return kind == other.kind && target == other.target && value == other.value;
    }
};

Access32 mmio(uint32_t target, uint32_t value) { return {Access32::Kind::write, target, value}; }
Access32 doorbell(uint32_t target, uint32_t value) { return {Access32::Kind::doorbell, target, value}; }

// Plain BAR5 register file with an RSMU index/data SMN window (NBIO doorbell range).
class Device final {
public:
    std::vector<uint32_t> registers = std::vector<uint32_t>(rx7900_test::aperture_bytes / 4, 0);
    std::map<uint32_t, uint32_t> smn_space;
    std::map<uint32_t, unsigned> reads;
    std::vector<Access32> log;
    uint32_t smn_address = 0;
    uint32_t fail_write = UINT32_MAX;
    bool doorbell_ok = true;

    uint32_t& at(uint32_t index) { return registers.at(index); }

    RegisterIo io() noexcept { return {this, rx7900_test::aperture_bytes, read, write, delay}; }

    Ih6Platform platform(bool doorbells) noexcept {
        return {this, bus_master, doorbells ? ring_doorbell : nullptr, confirm};
    }

    static bool read(void* context, uint32_t offset, uint32_t& value) noexcept {
        auto& self = *static_cast<Device*>(context);
        const uint32_t index = offset / 4;

        if ((offset & 3) != 0 || index >= self.registers.size()) {
            return false;
        }

        ++self.reads[index];
        value = index == rsmu_data ? self.smn_space[self.smn_address] : self.registers.at(index);
        return true;
    }

    static bool write(void* context, uint32_t offset, uint32_t value) noexcept {
        auto& self = *static_cast<Device*>(context);
        const uint32_t index = offset / 4;

        if ((offset & 3) != 0 || index >= self.registers.size() || index == self.fail_write) {
            return false;
        }

        if (index == rsmu_index) {
            self.smn_address = value;
        } else if (index == rb_cntl && (value & 0x80000000u) != 0) {
            // WPTR_OVERFLOW_CLEAR clears the latched IH_RB_WPTR.RB_OVERFLOW.
            self.registers.at(rb_wptr) &= ~1u;
        } else if (index == rsmu_data) {
            self.smn_space[self.smn_address] = value;
            return true;
        }

        self.registers.at(index) = value;
        self.log.push_back(mmio(index, value));
        return true;
    }

    static void delay(void*, uint32_t) noexcept {}
    static bool bus_master(void*) noexcept { return true; }
    static bool confirm(void*) noexcept { return false; }

    static bool ring_doorbell(void* context, uint32_t index, uint32_t value) noexcept {
        auto& self = *static_cast<Device*>(context);
        self.log.push_back(doorbell(index, value));
        return self.doorbell_ok;
    }
};

std::vector<uint8_t> discovery(bool with_mp1 = true, uint32_t gc_segment = gc) {
    const auto& n = rx7900_test::nbif_bases;
    std::vector<rx7900_test::IpRecord> records{
        {discovery_nbif_hardware_id, 4, 3, {n[0], n[1], n[2], n[3], n[4], n[5]}},
        {discovery_mmhub_hardware_id, 3, 0, {mmhub}},
        {discovery_osssys_hardware_id, 6, 0, {osssys_base}},
        {discovery_gc_hardware_id, 11, 0, {gc_segment, 0xa000}},
    };

    if (with_mp1) {
        records.push_back({discovery_mp1_hardware_id, 13, 0, {mp1}});
    }

    return rx7900_test::discovery_with(records);
}

// One IV: amdgpu_ih_decode_iv_helper dword layout.
struct Vector final {
    uint32_t client;
    uint32_t source;
    uint32_t ring = 0;
    uint32_t vmid = 0;
    bool vmid_source = false;
    uint32_t pasid = 0;
    uint32_t data[4]{};
};

// Synchronization double: runs routines directly; an optional hook runs once
// before a chosen call (0 collect, 1 commit, 2 finish) to model a concurrent ISR.
struct Lock final {
    unsigned calls = 0;
    unsigned hook_at = UINT32_MAX;
    void (*hook)(void*) = nullptr;
    void* hook_context = nullptr;

    static bool run(void* context, bool (*routine)(void*) noexcept, void* argument) noexcept {
        auto& self = *static_cast<Lock*>(context);

        if (self.calls++ == self.hook_at && self.hook != nullptr) {
            self.hook(self.hook_context);
        }

        return routine(argument);
    }

    IrqSynchronize sync() noexcept { return {this, run}; }
};

struct Fixture final {
    std::unique_ptr<rx7900_test::SimulatedNavi31> vram = std::make_unique<rx7900_test::SimulatedNavi31>();
    std::unique_ptr<Device> device = std::make_unique<Device>();
    rx7900_test::VramBackend backend{};
    std::unique_ptr<rx7900::memory::Manager> manager = std::make_unique<rx7900::memory::Manager>();
    Nbio nbio{};
    NbioRegisters nbio_registers{};
    Ih6Registers ih_registers{};
    IrqRegisters irq_registers{};
    std::unique_ptr<Ih6> ih = std::make_unique<Ih6>();
    std::unique_ptr<IrqService> irq = std::make_unique<IrqService>();
    Handle ring0{};
    Handle ring1{};
    Handle writeback{};
    uint32_t written = 0;

    explicit Fixture(bool doorbells = true, bool attach = true) {
        backend.device = vram.get();
        manager->initialize({rx7900_test::fb_base + rx7900_test::arena_offset,
                             rx7900_test::arena_bytes}, backend.backend());
        const auto binary = discovery();
        validate_nbio_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes,
                                nbio_registers);
        validate_ih6_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes,
                               ih_registers);
        validate_irq_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes,
                               irq_registers);
        nbio.initialize(nbio_registers, device->io());
        nbio.program_common();
        manager->allocate(ring_bytes, ih6_ring_alignment, Access::read_write, true, ring0);
        manager->allocate(ring_bytes, ih6_ring_alignment, Access::read_write, true, ring1);
        manager->allocate(rx7900::memory::page_bytes, rx7900::memory::page_bytes,
                          Access::read_write, true, writeback);
        ih->initialize(ih_registers, nbio_registers, device->io(), nbio, rx7900_test::navi31_vram(),
                       {ring_bytes, false, dummy_page}, device->platform(doorbells));
        ih->hw_init(*manager, ring0, ring1, writeback);

        if (attach) {
            irq->attach(*ih, device->io(), irq_registers);
            irq->arm_locked();
        }

        device->log.clear();
        device->reads.clear();
    }

    uint64_t address(Handle handle) const {
        rx7900::memory::AllocationInfo info{};
        manager->describe(handle, info);
        return info.gpu.virtual_address;
    }

    void put(Handle ring, uint32_t offset, const Vector& v) {
        const uint64_t at = address(ring) + (offset & (ring_bytes - 1));
        const uint32_t words[8]{
            v.client | (v.source << 8) | (v.ring << 16) | (v.vmid << 24) |
                (v.vmid_source ? 0x80000000u : 0u),
            0x89abcdefu, 0x00004567u, v.pasid | (0x5u << 16),
            v.data[0], v.data[1], v.data[2], v.data[3]};

        for (uint32_t index = 0; index < 8; ++index) {
            vram->put_vram_u32(at + index * 4, words[index]);
        }
    }

    // Writes vectors at ring 0 from the current hardware position and publishes WPTR.
    uint32_t publish(uint32_t from, const std::vector<Vector>& vectors) {
        uint32_t offset = from;

        for (const auto& v : vectors) {
            put(ring0, offset, v);
            offset = (offset + entry) & (ring_bytes - 1);
        }

        vram->put_vram_u32(address(writeback), offset);
        return offset;
    }

    IrqDrain drain(Lock& lock) { return irq->drain(lock.sync()); }

    // Drains until nothing remains, bounded by the ring capacity.
    unsigned drain_all(Lock& lock) {
        unsigned rounds = 0;

        for (; rounds < 2 * ring_bytes / entry; ++rounds) {
            if (!drain(lock).more_pending) {
                return rounds + 1;
            }
        }

        return rounds;
    }
};

uint32_t seq(const IrqReport& report, IrqSource source) {
    return report.sequence[static_cast<uint32_t>(source)];
}

void discovery_validation() {
    IrqRegisters registers;
    auto binary = discovery();
    expect(validate_irq_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes,
                                  registers) == DiscoveryStatus::validated && registers.valid() &&
           registers.gc_base() == gc && registers.mmhub_base() == mmhub &&
           registers.mp1_base() == mp1, "GMC and MP1 tokens compose the IRQ token");
    binary = discovery(false);
    expect(validate_irq_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes,
                                  registers) != DiscoveryStatus::validated && !registers.valid(),
           "missing MP1 rejected");
    binary = discovery(true, rx7900_test::aperture_bytes / 4 - 0x1d00);
    expect(validate_irq_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes,
                                  registers) == DiscoveryStatus::invalid_argument,
           "CP_ME1_PIPE3_INT_CNTL beyond BAR5 rejected");
}

void attach_contract() {
    Fixture fixture(true, false);
    IrqService irq;
    expect(irq.attach(*fixture.ih, fixture.device->io(), {}) == IrqStatus::unsupported_ip,
           "invalid token rejected");
    RegisterIo io = fixture.device->io();
    io.aperture_bytes = 0x1000;
    expect(irq.attach(*fixture.ih, io, fixture.irq_registers) == IrqStatus::invalid_argument,
           "aperture mismatch rejected");
    Ih6 disabled;
    expect(irq.attach(disabled, fixture.device->io(), fixture.irq_registers) ==
           IrqStatus::unavailable, "IH that is not enabled rejected");
    expect(!irq.isr() && irq.drain({nullptr, nullptr}).status == IrqStatus::invalid_argument,
           "detached ISR never claims; drain needs a lock");
    Lock lock;
    expect(irq.drain(lock.sync()).status == IrqStatus::unavailable, "detached drain unavailable");
    expect(irq.attach(*fixture.ih, fixture.device->io(), fixture.irq_registers) ==
           IrqStatus::attached, "enabled IH attaches");
    expect(irq.attach(*fixture.ih, fixture.device->io(), fixture.irq_registers) == IrqStatus::busy,
           "attach once");
    fixture.publish(0, {{0x14, 181}});
    expect(!irq.isr() && fixture.device->log.empty() && fixture.device->reads.empty(),
           "unarmed ISR touches no register or doorbell");
    irq.arm_locked();
    expect(irq.isr() && irq.report().armed, "armed ISR claims pending work");
    irq.disarm_locked();
    expect(!irq.isr(), "disarmed ISR declines");
}

void claim_and_acknowledge() {
    Fixture fixture;
    auto& device = *fixture.device;
    auto& irq = *fixture.irq;

    expect(!irq.isr() && device.log.empty(), "no new vectors: shared interrupt not ours");
    const uint32_t wptr = fixture.publish(0, {{0x14, 181}, {0x0a, 49, 0x10}});
    expect(irq.isr(), "pending vectors claimed");
    expect(device.log == std::vector<Access32>{doorbell(ring0_doorbell, wptr)},
           "ISR acknowledges RPTR = WPTR through the IH doorbell only");
    expect(irq.report().pending_bytes == 2 * entry && fixture.ih->rptr(Ih6Ring::primary) == wptr,
           "acknowledged span recorded for the DPC");
    expect(irq.report().isr_calls == 2 && irq.report().isr_claimed == 1, "claim statistics");
    expect(!irq.isr(), "re-entry after acknowledgement declines");

    Lock lock;
    const IrqDrain result = fixture.drain(lock);
    const IrqReport report = irq.report();
    expect(result.status == IrqStatus::attached && result.dispatched[0] == 2 &&
           result.dispatched[1] == 0 && !result.more_pending, "drain dispatches both vectors");
    expect(lock.calls == 3, "collect, commit and finish run under the lock");
    expect(report.gfx_eop[0] == 1 && report.sdma_trap[1] == 1 && report.pending_bytes == 0 &&
           report.dispatched[0] == 2, "fence counters advanced");
}

void register_mode_acknowledge() {
    Fixture fixture(false);
    const uint32_t wptr = fixture.publish(0, {{0x14, 181}});
    expect(fixture.irq->isr() &&
           fixture.device->log == std::vector<Access32>{mmio(rb_rptr, wptr)},
           "register-mode acknowledgement writes IH_RB_RPTR");
}

void routing() {
    Fixture fixture;
    auto& device = *fixture.device;
    auto& irq = *fixture.irq;
    device.at(mmvm_fault_status) = 0x00341234u;
    device.at(gcvm_fault_status) = 0x00560077u;
    device.at(mmvm_fault_cntl) = 0x00001ffcu;
    device.at(gcvm_fault_cntl) = 0x40001ffcu;
    device.at(mp1_sw_int_ctrl) = 0x00000002u;

    Vector compute{0x14, 181, (5u << 4) | (1u << 2) | 2u};
    Vector mes{0x14, 181};
    mes.data[0] = 0x1000000u | 0x42u;
    Vector gfx_pipe1{0x14, 181, 1};
    Vector priv_reg{0x14, 184, (3u << 4) | (1u << 2) | 3u, 2, false, 0x77};
    Vector priv_instr{0x14, 185};
    Vector bad_op{0x14, 183, (2u << 2)};
    Vector sdma0{0x0a, 49, 0x00};
    Vector sdma_queue1{0x0a, 49, 0x01};
    Vector sdma_bad_instance{0x0a, 49, 0x20};
    Vector mmhub_fault{0x12, 0, 0x03, 5, true, 0x1234};
    mmhub_fault.data[0] = 0x00abcdefu;
    mmhub_fault.data[1] = 0x000000f9u;
    Vector thm_high{0x0f, 0};
    Vector thm_low{0x0f, 1};
    Vector ctf{0x16, 83};
    Vector smu{0x1f, 0xfe};
    smu.data[0] = 7; // SMU_IH_INTERRUPT_CONTEXT_ID_THERMAL_THROTTLING
    Vector df{0x17, 0};
    Vector athub{0x03, 0x61};
    Vector fed{0x0a, 128};
    Vector unknown{0x10, 5};
    Vector invalid{0x25, 0};
    const std::vector<Vector> vectors{compute, mes, gfx_pipe1, priv_reg, priv_instr, bad_op, sdma0,
        sdma_queue1, sdma_bad_instance, mmhub_fault, thm_high, thm_low, ctf, smu, df, athub, fed,
        unknown, invalid};
    fixture.publish(0, vectors);
    expect(irq.isr(), "routing burst claimed");
    device.log.clear();
    Lock lock;
    const IrqDrain result = fixture.drain(lock);
    const IrqReport r = irq.report();

    expect(result.dispatched[0] == vectors.size(), "every vector dispatched once");
    expect(r.compute_eop[0][2][5] == 1 && seq(r, IrqSource::compute_eop) == 1,
           "compute EOP decoded to MEC1 pipe 2 queue 5 (three-bit queue field)");
    expect(seq(r, IrqSource::mes_eop) == 1 && r.last_mes_queue == 0x42,
           "MES-flagged EOP records the MES queue id");
    expect(r.gfx_eop[1] == 1 && r.gfx_eop[0] == 0 && seq(r, IrqSource::gfx_eop) == 1,
           "ME0 pipe 1 EOP routed to gfx ring 1");
    expect(seq(r, IrqSource::cp_priv_reg) == 1 && seq(r, IrqSource::cp_priv_instr) == 1 &&
           seq(r, IrqSource::cp_bad_opcode) == 1 && r.queue_fault,
           "CP faults counted and queue quarantine latched");
    expect(r.last_cp_fault.source == IrqSource::cp_bad_opcode && r.last_cp_fault.me == 2 &&
           r.last_cp_fault.pipe == 0 && r.last_cp_fault.queue == 0 && r.last_cp_fault.sequence == 1,
           "last CP fault identifies ME2");
    expect(r.sdma_trap[0] == 1 && seq(r, IrqSource::sdma_trap) == 1 &&
           seq(r, IrqSource::unhandled) == 2, "SDMA queue 0 trap counted, queue 1/instance 2 not");
    expect(r.vm_fault && r.last_vm_fault.address == ((0x00abcdefULL << 12) | (0x9ULL << 44)) &&
           r.last_vm_fault.vmid == 5 && r.last_vm_fault.pasid == 0x1234 &&
           r.last_vm_fault.ring_id == 3 && r.last_vm_fault.client == 0x12 &&
           r.last_vm_fault.register_hub == VmHub::mmhub && r.last_vm_fault.cache_hub == VmHub::mmhub &&
           r.last_vm_fault.status == 0x00341234u && r.last_vm_fault.status_read &&
           r.last_vm_fault.status_cleared && !r.last_vm_fault.page_fault_ring,
           "MMHUB fault address, ids and status decoded exactly");
    expect(device.reads[mmvm_fault_status] == 1, "vmid_src MMHUB: no dummy status read");
    expect(seq(r, IrqSource::thermal_high) == 1 && seq(r, IrqSource::thermal_low) == 1 &&
           r.thermal_work && seq(r, IrqSource::thermal_ctf) == 1 && r.hardware_ctf,
           "THM L2H/H2L and SMUIO CTF recorded for the PASSIVE_LEVEL owner");
    expect(seq(r, IrqSource::smu_to_driver) == 1 && r.smu_last_context == 7 &&
           r.smu_context[7] == 1 && r.smu_ack_failures == 0, "MP1 context recorded");
    expect(seq(r, IrqSource::ras) == 3, "DF, ATHUB and RLC FED counted as RAS");
    expect(r.ras_umc_ecc == 1 && r.ras_athub == 1 && r.ras_gc_fed == 1,
           "RAS vectors split by origin (GMC ecc_irq, ERREVENT_ATHUB, RLC GC FED) for the action owner");
    expect(seq(r, IrqSource::unknown) == 1 && r.unknown_by_client[0x10] == 1 &&
           seq(r, IrqSource::invalid) == 1, "unregistered and out-of-range clients counted");
    expect(device.log == std::vector<Access32>{mmio(mmvm_fault_cntl, 0x00001ffdu),
                                               mmio(mp1_sw_int_ctrl, 0x00000102u)},
           "dispatch writes only CLEAR_PROTECTION_FAULT_STATUS_ADDR and MP1 INT_ACK");
}

void gfxhub_fault_ring1() {
    Fixture fixture;
    auto& device = *fixture.device;
    auto& irq = *fixture.irq;
    device.at(gcvm_fault_status) = 0x00560077u;
    Vector self{0x00, 0, 1};
    self.data[0] = 3 * entry;
    Vector self_ring0{0x00, 0, 0};
    fixture.publish(0, {self_ring0, self});

    for (uint32_t index = 0; index < 3; ++index) {
        Vector fault{0x0a, 0, 0x10 + index, 3, false, 0x99};
        fault.data[0] = 0x100u + index;
        fixture.put(fixture.ring1, index * entry, fault);
    }

    expect(irq.isr(), "self interrupt claimed");
    device.log.clear();
    Lock lock;
    IrqDrain result = fixture.drain(lock);
    expect(result.dispatched[0] == 2 && result.dispatched[1] == 0 && result.more_pending &&
           seq(irq.report(), IrqSource::self_interrupt) == 2 && irq.report().ring1_wptr == 3 * entry,
           "ring 1 WPTR from the self interrupt schedules a ring 1 drain");
    result = fixture.drain(lock);
    const IrqReport r = irq.report();
    expect(result.dispatched[1] == 3 && !result.more_pending &&
           seq(r, IrqSource::gfxhub_fault) == 3, "ring 1 UTCL2 faults dispatched");
    expect(r.last_vm_fault.page_fault_ring && r.last_vm_fault.register_hub == VmHub::gfxhub &&
           r.last_vm_fault.cache_hub == VmHub::gfxhub && r.last_vm_fault.address == (0x102ULL << 12) &&
           r.last_vm_fault.ring_id == 0x12 && r.last_vm_fault.status == 0x00560077u,
           "GFXHUB fault decoded from the page-fault ring");
    expect(device.reads[gcvm_fault_status] == 6, "vmid_src GFXHUB(0): dummy read before each status read");
    expect(fixture.ih->rptr(Ih6Ring::page_fault) == 3 * entry &&
           std::find(device.log.begin(), device.log.end(), doorbell(ring1_doorbell, 3 * entry)) !=
               device.log.end(),
           "ring 1 RPTR acknowledged after decoding through its doorbell");
}

void wrap_and_bounded_drain() {
    Fixture fixture;
    auto& irq = *fixture.irq;
    Lock lock;
    // Advance the hardware position to 3 entries before the end of the ring in
    // two acknowledged spans that each stay within the backlog bound.
    uint32_t wptr = fixture.publish(0, std::vector<Vector>(64, Vector{0x14, 181}));
    irq.isr();
    fixture.drain_all(lock);
    wptr = fixture.publish(wptr, std::vector<Vector>(61, Vector{0x14, 181}));
    irq.isr();
    fixture.drain_all(lock);
    expect(fixture.ih->rptr(Ih6Ring::primary) == ring_bytes - 3 * entry &&
           irq.report().gfx_eop[0] == 125 && !irq.report().events_lost,
           "filler fully drained without loss");

    std::vector<Vector> burst;

    for (uint32_t index = 0; index < 40; ++index) {
        burst.push_back({0x14, 181, (index % 4) | (1u << 2) | ((index / 4 % 2) << 4)});
    }

    wptr = fixture.publish(wptr, burst);
    expect(wptr == 37 * entry && irq.isr() && irq.report().pending_bytes == 40 * entry,
           "wrapped span acknowledged");
    IrqDrain result = fixture.drain(lock);
    expect(result.dispatched[0] == irq_drain_batch_entries && result.more_pending,
           "drain bounded to AMDGPU_IH_MAX_NUM_IVS");
    result = fixture.drain(lock);
    const IrqReport r = irq.report();
    expect(result.dispatched[0] == 8 && !result.more_pending && r.pending_bytes == 0,
           "remainder drained next call");
    bool exact = true;

    for (uint32_t pipe = 0; pipe < 4; ++pipe) {
        exact = exact && r.compute_eop[0][pipe][0] == 5 && r.compute_eop[0][pipe][1] == 5;
    }

    expect(exact && seq(r, IrqSource::compute_eop) == 40, "wrapped vectors decoded in order");
}

void overflow_and_backlog() {
    {
        Fixture fixture;
        auto& device = *fixture.device;
        auto& irq = *fixture.irq;
        const uint64_t wb = fixture.address(fixture.writeback);
        fixture.publish(0, std::vector<Vector>(4, Vector{0x14, 181}));
        irq.isr();
        // IH wrapped over the unread span: writeback and IH_RB_WPTR carry RB_OVERFLOW.
        fixture.vram->put_vram_u32(wb, 0x00000101u);
        device.at(rb_wptr) = 0x00000101u;
        const uint32_t cntl = device.at(rb_cntl);
        device.log.clear();
        expect(irq.isr(), "overflow claimed");
        const IrqReport r = irq.report();
        expect(r.overflows == 1 && r.events_lost &&
               r.entries_lost == 4 + (ring_bytes - entry - ring_bytes / 2) / entry &&
               r.pending_bytes == ring_bytes / 2, "overflow discards unread and over-bound vectors");
        expect(device.log == std::vector<Access32>{mmio(rb_cntl, cntl | 0x80000000u),
                                                   mmio(rb_cntl, cntl & ~0x80000000u),
                                                   doorbell(ring0_doorbell, 0x100)},
               "WPTR_OVERFLOW_CLEAR pulse then RPTR = WPTR");
        Lock lock;
        fixture.drain(lock);
        expect(irq.report().pending_bytes == ring_bytes / 2 - irq_drain_batch_entries * entry,
               "post-overflow backlog drains in bounded batches");
    }
    {
        Fixture fixture;
        auto& irq = *fixture.irq;
        fixture.publish(0, std::vector<Vector>(ring_bytes / entry / 2 + 6, Vector{0x14, 181}));
        expect(irq.isr() && irq.report().entries_lost == 6 && irq.report().events_lost &&
               irq.report().pending_bytes == ring_bytes / 2, "backlog bounded to half the ring");
        Lock lock;
        fixture.drain_all(lock);
        expect(irq.report().gfx_eop[0] == ring_bytes / entry / 2, "only retained vectors dispatched");
    }
}

struct RaceContext final {
    Fixture* fixture;
    uint32_t from;
};

void concurrent_discard() {
    Fixture fixture;
    auto& irq = *fixture.irq;
    const uint32_t wptr = fixture.publish(0, std::vector<Vector>(10, Vector{0x14, 181}));
    irq.isr();
    RaceContext race{&fixture, wptr};
    Lock lock;
    lock.hook_at = 1; // before commit: an ISR on another CPU overruns the backlog bound
    lock.hook_context = &race;
    lock.hook = [](void* context) {
        auto& self = *static_cast<RaceContext*>(context);
        self.fixture->publish(self.from, std::vector<Vector>(ring_bytes / entry / 2, Vector{0x0a, 49}));
        self.fixture->irq->isr();
    };
    const IrqDrain result = fixture.drain(lock);
    const IrqReport r = irq.report();
    expect(result.dispatched[0] == 0 && r.gfx_eop[0] == 0 && r.entries_lost == 10 &&
           result.more_pending, "batch decoded during an ISR discard is dropped, loss counted once");
    fixture.drain_all(lock);
    expect(irq.report().sdma_trap[0] == ring_bytes / entry / 2 &&
           irq.report().pending_bytes == 0, "retained newer vectors dispatched afterwards");
}

struct NestedContext final {
    IrqService* irq;
    Lock* lock;
    IrqStatus status;
};

void reentrancy_and_quarantine() {
    {
        Fixture fixture;
        fixture.publish(0, {{0x14, 181}});
        fixture.irq->isr();
        Lock inner;
        NestedContext nested{fixture.irq.get(), &inner, IrqStatus::detached};
        Lock outer;
        outer.hook_at = 1;
        outer.hook_context = &nested;
        outer.hook = [](void* context) {
            auto& self = *static_cast<NestedContext*>(context);
            self.status = self.irq->drain(self.lock->sync()).status;
        };
        fixture.drain(outer);
        expect(nested.status == IrqStatus::busy && fixture.irq->report().gfx_eop[0] == 1,
               "concurrent drain refused while one is active");
    }
    {
        Fixture fixture;
        fixture.device->doorbell_ok = false;
        fixture.publish(0, {{0x14, 181}});
        expect(fixture.irq->isr() && fixture.irq->report().status == IrqStatus::quarantined &&
               !fixture.irq->report().armed, "failed RPTR doorbell quarantines the service");
        fixture.publish(entry, {{0x14, 181}});
        Lock lock;
        expect(!fixture.irq->isr() && fixture.drain(lock).status == IrqStatus::quarantined,
               "quarantined service neither claims nor drains");
    }
}

void enable_sources() {
    Fixture fixture;
    auto& device = *fixture.device;
    auto& irq = *fixture.irq;
    // Prior-owner bits must survive every read-modify-write.
    device.at(cp_int_cntl_ring0) = 0x00000100u;
    device.at(cp_int_cntl_ring1) = 0x00000200u;
    device.at(sdma0_cntl) = 0x10000000u;
    device.at(sdma1_cntl) = 0x10000002u;

    for (uint32_t pipe = 0; pipe < 4; ++pipe) {
        device.at(cp_me1_pipe0 + pipe) = 0x1000u * (pipe + 1);
    }

    for (uint32_t context = 0; context < 16; ++context) {
        device.at(mmvm_context0_cntl + context) = 0x00555401u;
        device.at(gcvm_context0_cntl + context) = 0x00000001u;
    }

    expect(irq.enable_sources({0x4, 0, 0}) == IrqStatus::invalid_argument &&
           irq.enable_sources({0, 0x10, 0}) == IrqStatus::invalid_argument &&
           irq.enable_sources({0, 0, 0x4}) == IrqStatus::invalid_argument && device.log.empty(),
           "plan outside ME0/MEC1/SDMA ranges rejected before any write");
    expect(irq.enable_sources({0x1, 0xf, 0x3}) == IrqStatus::enabled &&
           irq.report().sources_enabled, "sources enabled");

    std::vector<Access32> expected{mmio(cp_int_cntl_ring0, 0x84000100u)};

    for (uint32_t pipe = 0; pipe < 4; ++pipe) {
        expected.push_back(mmio(cp_me1_pipe0 + pipe, 0x84000000u | (0x1000u * (pipe + 1))));
    }

    expected.push_back(mmio(sdma0_cntl, 0x10000001u));
    expected.push_back(mmio(sdma1_cntl, 0x10000003u));

    for (uint32_t context = 0; context < 16; ++context) {
        expected.push_back(mmio(mmvm_context0_cntl + context, 0x007ffe01u));
    }

    for (uint32_t context = 0; context < 16; ++context) {
        expected.push_back(mmio(gcvm_context0_cntl + context, 0x002aaa01u));
    }

    expected.push_back(mmio(cp_int_cntl_ring0, 0x84800100u));

    for (uint32_t pipe = 0; pipe < 4; ++pipe) {
        expected.push_back(mmio(cp_me1_pipe0 + pipe, 0x84800000u | (0x1000u * (pipe + 1))));
    }

    expected.push_back(mmio(cp_int_cntl_ring0, 0x84c00100u));
    expected.push_back(mmio(cp_int_cntl_ring0, 0x85c00100u));

    for (uint32_t pipe = 0; pipe < 4; ++pipe) {
        expected.push_back(mmio(cp_me1_pipe0 + pipe, 0x85800000u | (0x1000u * (pipe + 1))));
    }

    expect(device.log == expected, "fence hw_init then gmc/gfx late_init RMW order and values");

    if (!(device.log == expected)) {
        for (size_t index = 0; index < device.log.size(); ++index) {
            std::fprintf(stderr, "  %zu: target %#x value %#x\n", index, device.log[index].target,
                         device.log[index].value);
        }
    }

    const IrqReport r = irq.report();
    expect(r.cp_int_cntl_ring[0] == 0x85c00100u && r.cp_int_cntl_ring[1] == 0 &&
           r.cp_me1_pipe_int_cntl[3] == 0x85804000u && r.sdma_cntl[1] == 0x10000003u &&
           r.vm_context_cntl[0][15] == 0x007ffe01u && r.vm_context_cntl[1][0] == 0x002aaa01u &&
           device.at(cp_int_cntl_ring1) == 0x00000200u, "report records the programmed values");
    expect(irq.enable_sources({0x1, 0xf, 0x3}) == IrqStatus::busy, "enable_sources runs once");

    Fixture partial;
    partial.device->fail_write = gcvm_context0_cntl + 3;
    expect(partial.irq->enable_sources({0x1, 0x1, 0x1}) == IrqStatus::mmio_failure &&
           !partial.irq->report().sources_enabled, "failed store reported, enable incomplete");
    partial.publish(0, {{0x14, 181}});
    expect(partial.irq->isr(), "servicing continues after a failed enable");

    Fixture pipe1;
    pipe1.irq->enable_sources({0x2, 0x5, 0x2});
    expect(pipe1.device->log.front() == mmio(cp_int_cntl_ring1, 0x84000000u) &&
           pipe1.device->at(cp_me1_pipe0 + 1) == 0x01800000u &&
           pipe1.device->at(cp_me1_pipe0 + 2) == 0x85800000u && pipe1.device->at(sdma0_cntl) == 0 &&
           pipe1.device->at(sdma1_cntl) == 1, "only planned EOP/trap sources enabled");
}

// Drain consumer that records what each call saw.
struct Recorder final {
    unsigned calls = 0;
    std::vector<bool> lost;
    uint32_t vm_total = 0;
    uint32_t cp_fault = 0;

    static void consume(void* context, const IrqDispatchView& view) noexcept {
        auto& self = *static_cast<Recorder*>(context);
        ++self.calls;
        self.lost.push_back(view.events_lost);
        self.vm_total = view.vm_faults->total;
        self.cp_fault = view.report->cp_fault_count[1][2][1];
    }

    IrqConsumer consumer() noexcept { return {this, consume}; }
};

void fault_records_and_consumer() {
    Fixture fixture;
    auto& device = *fixture.device;
    auto& irq = *fixture.irq;
    device.at(mmvm_fault_status) = 0x00341234u;
    device.at(gcvm_fault_status) = 0x00560077u;
    Vector priv_reg{0x14, 184, (1u << 4) | (1u << 2) | 2u};
    Vector bad_me{0x14, 183, 0x0cu};
    Vector mmhub_fault{0x12, 0, 0x03, 5, true, 0x1234};
    mmhub_fault.data[0] = 0x10u;
    Vector gfx_fault{0x0a, 0, 0x07, 0, false, 0};
    gfx_fault.data[0] = 0x20u;
    fixture.publish(0, {priv_reg, bad_me, mmhub_fault, gfx_fault});
    irq.isr();
    Lock lock;
    Recorder recorder;
    const IrqDrain result = irq.drain(lock.sync(), recorder.consumer());
    const IrqReport r = irq.report();
    expect(result.dispatched[0] == 4 && recorder.calls == 1 && recorder.lost.at(0) == false &&
           recorder.cp_fault == 1 && recorder.vm_total == 2,
           "consumer sees the dispatched batch once, after dispatch");
    expect(r.cp_fault_count[1][2][1] == 1 && r.cp_fault_source[1][2][1] == IrqSource::cp_priv_reg &&
           r.cp_fault_invalid_me == 1 && r.last_cp_fault.me == 3,
           "per-queue CP fault counters; ME 3 counted apart (Linux BUG)");
    const IrqVmFaultLog& log = irq.vm_fault_log();
    expect(log.total == 2 && log.entries[0].register_hub == VmHub::mmhub &&
           log.entries[0].address == (0x10ULL << 12) && log.entries[0].sequence == 1 &&
           log.entries[1].register_hub == VmHub::gfxhub && log.entries[1].vmid == 0 &&
           log.entries[1].status == 0x00560077u, "VM fault log in dispatch order");
    // Lost vectors are reported to the next consumer exactly once.
    const uint64_t wb = fixture.address(fixture.writeback);
    const uint32_t at = fixture.publish(4 * entry, std::vector<Vector>(4, Vector{0x14, 181}));
    (void)at;
    irq.isr();
    fixture.vram->put_vram_u32(wb, 0x00000101u + 4 * entry);
    device.at(rb_wptr) = 0x00000101u + 4 * entry;
    irq.isr();
    irq.drain(lock.sync(), recorder.consumer());
    expect(recorder.calls == 2 && recorder.lost.at(1), "overflow loss reaches the consumer");
    irq.drain(lock.sync(), recorder.consumer());
    expect(recorder.calls == 3 && !recorder.lost.at(2), "loss reported once");
    // A drain that did not run calls no consumer.
    IrqService detached;
    detached.drain(lock.sync(), recorder.consumer());
    expect(recorder.calls == 3, "no consumer call without a drain");
}

// Minimal FenceManager platform: plain lock flag, fixed clock, events unused.
struct FencePlatformModel final {
    bool held = false;
    unsigned errors = 0;
    uint64_t now = 100;

    FencePlatform platform() {
        return {this,
                [](void* c) noexcept { auto& p = *static_cast<FencePlatformModel*>(c); p.errors += p.held; p.held = true; },
                [](void* c) noexcept { auto& p = *static_cast<FencePlatformModel*>(c); p.errors += !p.held; p.held = false; },
                [](void* c) noexcept { return static_cast<FencePlatformModel*>(c)->now; },
                [](void*, uint32_t) noexcept {}, [](void*, uint32_t) noexcept {},
                [](void*, uint32_t, uint64_t) noexcept { return false; }, nullptr, nullptr};
    }
};

void fence_routing() {
    Fixture fixture;
    auto& device = *fixture.device;
    auto& irq = *fixture.irq;
    FencePlatformModel model;
    FenceManager fences;
    fences.initialize(model.platform());
    uint8_t compute_wb[4]{};
    uint8_t sdma_wb[4]{};
    uint8_t gfx_wb[4]{};
    // Compute ring 6 of Gfx11Queues: MEC1 pipe 2 queue 1 (fence ring 7).
    expect(fences.attach_ring(7, {FenceEngine::compute, 1, 2, 1, 0, {compute_wb, 4}, 0, 0, 0}) ==
               FenceStatus::attached &&
           fences.attach_ring(fence_ring_sdma0, {FenceEngine::sdma, 0, 0, 0, 0, {sdma_wb, 4}, 0, 0, 0}) ==
               FenceStatus::attached &&
           fences.attach_ring(0, {FenceEngine::gfx, 0, 0, 0, 0, {gfx_wb, 4}, 0, 0, 0}) ==
               FenceStatus::attached, "rings attached");
    FenceReservation reservation{};
    fences.begin_submission(7, reservation);
    fences.publish_submission(7, reservation.sequence, 256);
    fences.begin_submission(0, reservation);
    fences.publish_submission(0, reservation.sequence, 256);
    rx7900::sdma::CommandBuffer(compute_wb, 4).write_u32(0, 1);
    Lock lock;
    fixture.publish(0, {{0x14, 181, (1u << 4) | (1u << 2) | 2u}});
    irq.isr();
    irq.drain(lock.sync(), fences.consumer());
    FenceRingReport ring = fences.ring_report(7);
    expect(ring.last_seq == 1 && ring.interrupt_processes == 1 && !ring.timeout_armed,
           "decoded CP EOP signals the compute fence in the DPC consumer");
    // gfx_v11_0_priv_inst_irq on the same queue: drm_sched_fault -> quarantine.
    fixture.publish(entry, {{0x14, 185, (1u << 4) | (1u << 2) | 2u}});
    irq.isr();
    irq.drain(lock.sync(), fences.consumer());
    ring = fences.ring_report(7);
    expect(ring.state == FenceRingState::hung && ring.hang_reason == FenceHangReason::cp_priv_instr &&
           fences.ring_report(0).state == FenceRingState::active,
           "privileged-instruction fault quarantines only its ring");
    // UTCL2 VMID 0 fault whose GCVM status names SDMA0 (CID 13).
    device.at(gcvm_fault_status) = (13u << 9) | 1u;
    fixture.publish(2 * entry, {{0x0a, 0, 0, 0, false, 0}});
    irq.isr();
    irq.drain(lock.sync(), fences.consumer());
    const FenceReport report = fences.report();
    expect(report.rings[fence_ring_sdma0].state == FenceRingState::hung &&
           report.rings[fence_ring_sdma0].hang_reason == FenceHangReason::vm_fault &&
           report.rings[0].state == FenceRingState::active && report.vm_faults_vmid0 == 1 &&
           report.vm_faults_printed == 1 && report.reset_required,
           "VMID 0 SDMA0 fault quarantines the SDMA0 kernel ring");
    expect(device.log.back() == mmio(gcvm_fault_cntl, 0x00000001u),
           "status still cleared by the IRQ service");
    expect(model.errors == 0 && !model.held, "fence lock balanced in the consumer");
}

} // namespace

// System suspend: the owner disarms and drains, IrqService stops using the
// rings, IH is disabled and later resumed on zeroed rings; counters continue.
void suspend_and_resume() {
    Fixture fixture;
    auto& irq = *fixture.irq;
    Lock lock;
    const uint32_t wptr = fixture.publish(0, {{0x14, 181}, {0x0a, 49, 0x10}});
    expect(irq.isr() && irq.enable_sources({0x1, 0xf, 0x3}) == IrqStatus::enabled,
           "vectors pending and sources enabled before suspend");
    expect(irq.suspend() == IrqStatus::busy, "suspend refused while armed");
    irq.disarm_locked();
    expect(irq.suspend() == IrqStatus::busy, "suspend refused while acknowledged vectors are undispatched");
    fixture.drain_all(lock);
    const IrqReport before = irq.report();
    expect(before.gfx_eop[0] == 1 && before.pending_bytes == 0 && irq.suspend() == IrqStatus::suspended,
           "final drain dispatches the backlog, then the service suspends");
    expect(fixture.ih->disable() == Ih6Status::disabled && !irq.isr() &&
           irq.drain(lock.sync()).status == IrqStatus::unavailable && irq.attach(*fixture.ih,
               fixture.device->io(), fixture.irq_registers) == IrqStatus::busy,
           "a suspended service never touches the disabled rings and cannot be re-attached");
    expect(irq.resume(*fixture.ih) == IrqStatus::unavailable, "resume needs the IH enabled again");
    expect(fixture.ih->resume() == Ih6Status::enabled && irq.resume(*fixture.ih) == IrqStatus::attached &&
           !irq.report().sources_enabled && irq.report().gfx_eop[0] == before.gfx_eop[0] &&
           irq.report().sdma_trap[1] == before.sdma_trap[1] && irq.report().dispatched[0] == before.dispatched[0],
           "resume rebinds the zeroed rings and keeps every counter");
    expect(fixture.ih->rptr(Ih6Ring::primary) == 0 && wptr != 0, "IH resumed at RPTR 0");
    expect(irq.enable_sources({0x1, 0xf, 0x3}) == IrqStatus::enabled, "sources enabled again after power loss");
    irq.arm_locked();
    fixture.publish(0, {{0x14, 181}});
    expect(irq.isr() && fixture.drain_all(lock) >= 1 && irq.report().gfx_eop[0] == before.gfx_eop[0] + 1,
           "vectors after resume continue the same EOP counter");
    expect(irq.resume(*fixture.ih) == IrqStatus::busy, "resume only from suspended");
}

int main() {
    discovery_validation();
    attach_contract();
    claim_and_acknowledge();
    register_mode_acknowledge();
    routing();
    gfxhub_fault_ring1();
    wrap_and_bounded_drain();
    overflow_and_backlog();
    concurrent_discard();
    reentrancy_and_quarantine();
    enable_sources();
    fault_records_and_consumer();
    fence_routing();
    suspend_and_resume();
    return checks.finish("irq_service_tests");
}

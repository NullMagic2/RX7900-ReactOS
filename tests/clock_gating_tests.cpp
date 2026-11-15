/*
Module: rx7900.tests.clock_gating
Purpose: Verify the late-init clock/power gating order, register values, RLC
safe-mode handshake, GFXOFF messages, SMN fallback and quarantine.
Created: 2026-10-21
Architecture: Shared synthetic Navi31 model behind a local tracing RegisterIo;
expected values are derived by hand from Linux v6.12 amdgpu_device.c, soc21.c,
nbio_v4_3.c, hdp_v6_0.c, mmhub_v3_0.c, athub_v3_0.c, ih_v6_0.c, lsdma_v6_0.c,
gfx_v11_0.c, amdgpu_rlc.c, amdgpu_gfx.c and smu_v13_0.c. Not hardware evidence.
*/
#include <map>
#include <memory>

#include "asic_test_device.hpp"
#include "clock_gating.hpp"

namespace {

using namespace rx7900::hardware;
using rx7900_test::SimulatedNavi31;

rx7900_test::Checks checks;

void expect(bool condition, const char* contract) { checks.expect(condition, contract); }

constexpr uint32_t gc0 = rx7900_test::gc_base;
constexpr uint32_t gc1 = 0xa000;
constexpr uint32_t mmhub = rx7900_test::mmhub_base;
constexpr uint32_t mp = rx7900_test::mp_base;
constexpr uint32_t osssys = 0x4260;
constexpr uint32_t athub = 0x0c00;
constexpr uint32_t hdp = 0x0f20;
// Beyond the 1 MiB aperture: every HDP access must take the RSMU path.
constexpr uint32_t hdp_far = 0x02400000;
constexpr uint32_t lsdma = 0x6180;
constexpr uint32_t rsmu_index = rx7900_test::nbif_bases[1];
constexpr uint32_t rsmu_data = rx7900_test::nbif_bases[1] + 1;
constexpr uint32_t nbif5 = rx7900_test::nbif_bases[5];
constexpr uint32_t cpm_smn = (nbif5 + 0x28a0118u) * 4;
constexpr uint32_t pcie_cntl2_smn = (nbif5 + 0x28a001cu) * 4;

constexpr uint32_t rlc_cntl = gc1 + 0x4c00;
constexpr uint32_t safe_mode = gc1 + 0x0980;
constexpr uint32_t mgcg_override = gc1 + 0x4c48;
constexpr uint32_t cgcg_cgls = gc1 + 0x4c49;
constexpr uint32_t cgcg_cgls_3d = gc1 + 0x4cc5;
constexpr uint32_t wptr_poll = gc0 + 0x0f62;
constexpr uint32_t cp_int = gc0 + 0x1de9;
constexpr uint32_t cp_int_ring0 = gc0 + 0x1e0a;
constexpr uint32_t sdma0_cgcg = gc0 + 0x005c;
constexpr uint32_t sdma1_cgcg = gc0 + 0x065c;
constexpr uint32_t mm_atc_cg = mmhub + 0x06cd;
constexpr uint32_t athub_misc = athub + 0x0075;
constexpr uint32_t hdp_mem = hdp + 0x00d4;
constexpr uint32_t hdp_clk = hdp + 0x00d8;
constexpr uint32_t lsdma_mem = lsdma + 0x0071;
constexpr uint32_t ih_clk = osssys + 0x00db;
constexpr uint32_t smu_response = mp + 0x29a;
constexpr uint32_t smu_argument = mp + 0x292;
constexpr uint32_t smu_message = mp + 0x282;

struct Access final {
    enum class Kind { mmio, smn, delay };
    Kind kind;
    uint32_t target;
    uint32_t value;

    bool operator==(const Access& other) const {
        return kind == other.kind && target == other.target && value == other.value;
    }
};

Access mmio(uint32_t target, uint32_t value) { return {Access::Kind::mmio, target, value}; }
Access smn(uint32_t target, uint32_t value) { return {Access::Kind::smn, target, value}; }
Access delay(uint32_t microseconds) { return {Access::Kind::delay, 0, microseconds}; }

// Wraps the shared model's RegisterIo: records every store and delay, keeps a
// writable SMN space for registers beyond BAR5 and models the RLC acknowledging
// a safe-mode request by clearing RLC_SAFE_MODE.CMD after ack_after polls.
struct Trace final {
    SimulatedNavi31* device = nullptr;
    std::vector<Access> log;
    std::map<uint32_t, uint32_t> smn_space;
    uint32_t safe_mode_ack_after = 0;
    bool safe_mode_never_ack = false;
    uint32_t safe_mode_reads = 0;
    uint32_t fail_write = UINT32_MAX;

    static bool read(void* context, uint32_t offset, uint32_t& value) noexcept {
        auto& self = *static_cast<Trace*>(context);
        const uint32_t index = offset / 4;

        if (index == rsmu_data) {
            const auto found = self.smn_space.find(self.device->smn_index);

            if (found != self.smn_space.end()) {
                value = found->second;
                return self.device->reads_ok;
            }
        }

        if (index == safe_mode) {
            ++self.safe_mode_reads;

            if (!self.safe_mode_never_ack && self.safe_mode_reads > self.safe_mode_ack_after) {
                self.device->reg(safe_mode) &= ~1u;
            }
        }

        return SimulatedNavi31::read(self.device, offset, value);
    }

    static bool write(void* context, uint32_t offset, uint32_t value) noexcept {
        auto& self = *static_cast<Trace*>(context);
        const uint32_t index = offset / 4;

        if (index == self.fail_write) {
            return false;
        }

        if (index == rsmu_data) {
            self.log.push_back(smn(self.device->smn_index, value));
            self.smn_space[self.device->smn_index] = value;
        } else if (index != rsmu_index) {
            self.log.push_back(mmio(index, value));
        }

        if (index == safe_mode) {
            self.safe_mode_reads = 0;
        }

        return SimulatedNavi31::write(self.device, offset, value);
    }

    static void wait(void* context, uint32_t microseconds) noexcept {
        auto& self = *static_cast<Trace*>(context);
        self.log.push_back(delay(microseconds));
        SimulatedNavi31::delay(self.device, microseconds);
    }

    RegisterIo io() noexcept { return {this, rx7900_test::aperture_bytes, read, write, wait}; }
};

struct Layout final {
    bool athub_present = true;
    uint8_t athub_minor = 0;
    uint32_t hdp_base = hdp;
    bool lsdma_present = true;
    uint8_t lsdma_revision = 0;
};

// rx7900_test::navi31_discovery plus the ATHUB, HDP and LSDMA records it omits.
std::vector<uint8_t> discovery(const Layout& layout) {
    std::vector<rx7900_test::IpRecord> records{
        {discovery_mp0_hardware_id, 13, 0, {mp}},
        {discovery_nbif_hardware_id, 4, 3,
         {rx7900_test::nbif_bases[0], rx7900_test::nbif_bases[1], rx7900_test::nbif_bases[2],
          rx7900_test::nbif_bases[3], rx7900_test::nbif_bases[4], rx7900_test::nbif_bases[5]}},
        {discovery_mmhub_hardware_id, 3, 0, {mmhub}},
        {discovery_mp1_hardware_id, 13, 0, {mp}},
        {discovery_gc_hardware_id, 11, 0, {gc0, gc1}},
        {discovery_osssys_hardware_id, 6, 0, {osssys}},
        {discovery_thm_hardware_id, 13, 0, {rx7900_test::thm_base, 0x02400c00}, 3},
        {discovery_hdp_hardware_id, 6, 0, {layout.hdp_base}},
    };

    if (layout.athub_present) {
        records.push_back({discovery_athub_hardware_id, 3, layout.athub_minor, {athub}});
    }

    if (layout.lsdma_present) {
        records.push_back({discovery_lsdma_hardware_id, 6, 0, {lsdma}, layout.lsdma_revision});
    }

    return rx7900_test::discovery_with(records);
}

struct Fixture final {
    std::unique_ptr<SimulatedNavi31> device = std::make_unique<SimulatedNavi31>();
    Trace trace{};
    std::vector<uint8_t> binary;
    NbioRegisters nbio_registers{};
    rx7900::memory::GcRegisterBase gc{};
    Nbio nbio{};
    std::unique_ptr<Ih6> ih = std::make_unique<Ih6>();
    std::unique_ptr<Smu13> smu = std::make_unique<Smu13>();
    std::unique_ptr<ClockGating> gating = std::make_unique<ClockGating>();
    ClockGatingStatus initialized = ClockGatingStatus::unavailable;
    size_t initialize_accesses = 0;

    explicit Fixture(Layout layout = {}, ClockGatingPolicy policy = clock_gating_linux_policy,
                     bool smu_features = true) : binary(discovery(layout)) {
        trace.device = device.get();
        const rx7900::ByteView view(binary.data(), binary.size());
        const RegisterIo io = trace.io();
        validate_nbio_discovery(view, rx7900_test::aperture_bytes, nbio_registers);
        rx7900::memory::validate_gc_discovery(view, rx7900_test::aperture_bytes, gc);
        nbio.initialize(nbio_registers, io);
        nbio.program_common();

        Ih6Registers ih_registers;
        validate_ih6_discovery(view, rx7900_test::aperture_bytes, ih_registers);
        ih->initialize(ih_registers, nbio_registers, io, nbio, rx7900_test::navi31_vram(),
                       {ih6_default_ring_bytes, true, 0x0000001234567000ULL},
                       {device.get(), SimulatedNavi31::bus_master, nullptr, SimulatedNavi31::confirm});

        SmuRegisters smu_registers;
        validate_smu_discovery(view, rx7900_test::aperture_bytes, smu_registers);
        smu->initialize(smu_registers, io, nbio, {100, 3}, {device.get(), SimulatedNavi31::confirm});

        if (smu_features) {
            smu->enable_all_features();
        }

        const size_t before = trace.log.size();
        initialized = gating->initialize(view, gc, nbio_registers, io, nbio, policy,
                                         {device.get(), SimulatedNavi31::confirm});
        initialize_accesses = trace.log.size() - before;
        // State left by the VBIOS and hw_init; unrelated bits must survive.
        reg(rlc_cntl) = 0x00000001u;
        reg(mgcg_override) = 0xffffffffu;
        reg(cgcg_cgls) = 0;
        reg(cgcg_cgls_3d) = 0;
        reg(wptr_poll) = 0;
        reg(cp_int) = 0x00000001u;
        reg(cp_int_ring0) = 0;
        reg(sdma0_cgcg) = 0;
        reg(sdma1_cgcg) = 0;
        reg(mm_atc_cg) = 0x00000001u;
        reg(athub_misc) = 0;
        reg(hdp_clk) = 0x80000001u;
        reg(hdp_mem) = 0xffffffffu;
        reg(lsdma_mem) = 0x00000010u;
        reg(ih_clk) = 0xf8800000u;
        trace.smn_space[cpm_smn] = 0x0000ffffu;
        trace.smn_space[pcie_cntl2_smn] = 0x00010001u;
        trace.log.clear();
    }

    uint32_t& reg(uint32_t index) { return device->reg(index); }
    ClockGatingStatus gate() { return gating->gate(*ih, *smu); }
    ClockGatingStatus ungate() { return gating->ungate(*ih, *smu); }

    size_t count(uint32_t target) const {
        size_t total = 0;

        for (const auto& access : trace.log) {
            total += access.kind == Access::Kind::mmio && access.target == target ? 1 : 0;
        }

        return total;
    }
};

std::vector<Access> smu_send(uint32_t message) {
    return {mmio(smu_response, 0), mmio(smu_argument, 0), mmio(smu_message, message)};
}

std::vector<Access> join(std::initializer_list<std::vector<Access>> parts) {
    std::vector<Access> all;

    for (const auto& part : parts) {
        all.insert(all.end(), part.begin(), part.end());
    }

    return all;
}

// hdp_v6_0_update_clock_gating(true) from CLK 0x80000001 / MEM 0xffffffff.
const std::vector<Access> hdp_gate{mmio(hdp_clk, 0x88000001u), mmio(hdp_mem, 0xfff0fff0u),
                                   mmio(hdp_mem, 0xfff9fff9u), mmio(hdp_clk, 0x80000001u)};
const std::vector<Access> gmc_gate{mmio(mm_atc_cg, 0x00040001u), mmio(mm_atc_cg, 0x000c0001u),
                                   mmio(athub_misc, 0x40u), mmio(athub_misc, 0xc0u)};
const std::vector<Access> ih_gate{mmio(ih_clk, 0x00800000u)};
const std::vector<Access> gfx_gate{
    mmio(safe_mode, 0x3u),
    mmio(mgcg_override, 0xffffffe7u), mmio(cgcg_cgls, 0x0000363fu),
    mmio(wptr_poll, 0x00900100u), mmio(cp_int, 0x003c0001u),
    mmio(sdma0_cgcg, 0x2u), mmio(sdma1_cgcg, 0x2u),
    mmio(mgcg_override, 0xffffffc1u), mmio(mgcg_override, 0xfffffdc1u),
    mmio(mgcg_override, 0xfffffcc1u), mmio(mgcg_override, 0xfffff8c1u),
    mmio(cp_int_ring0, 0x003c0000u), mmio(safe_mode, 0x1u)};
const std::vector<Access> lsdma_gate{mmio(lsdma_mem, 0x10u), mmio(lsdma_mem, 0x11u)};

void initialize_contract() {
    Fixture fixture;
    expect(fixture.initialized == ClockGatingStatus::ready, "Navi31 tokens accepted");
    const auto report = fixture.gating->report();
    expect(report.athub_base == athub && report.hdp_base == hdp && report.lsdma_base == lsdma &&
           report.lsdma_present && report.gfx_off_requests == 1 && !report.gfx_off_allowed,
           "discovered ATHUB/HDP/LSDMA bases and Linux gfx_off_req_count 1");
    expect(report.cg_flags == 0x3fd00030dULL && report.pg_flags == 0x3e000u,
           "soc21 GC 11.0.0 cg_flags/pg_flags with default masks");
    expect(fixture.initialize_accesses == 0 && fixture.device->smu_messages.size() == 3,
           "initialize touches no register");

    Layout no_athub;
    no_athub.athub_present = false;
    expect(Fixture(no_athub).initialized == ClockGatingStatus::unsupported_ip, "ATHUB required");
    Layout athub_301;
    athub_301.athub_minor = 1;
    expect(Fixture(athub_301).initialized == ClockGatingStatus::unsupported_ip,
           "ATHUB 3.0.1 (other CG register) refused");
    Layout no_lsdma;
    no_lsdma.lsdma_present = false;
    Fixture absent(no_lsdma);
    expect(absent.initialized == ClockGatingStatus::ready && !absent.gating->report().lsdma_present,
           "absent LSDMA is the Linux no-op");
    Layout lsdma_602;
    lsdma_602.lsdma_revision = 2;
    Fixture navi33_lsdma(lsdma_602);
    expect(navi33_lsdma.initialized == ClockGatingStatus::ready &&
           navi33_lsdma.gating->report().lsdma_present, "LSDMA 6.0.2 gated as Linux does");
    Layout lsdma_601;
    lsdma_601.lsdma_revision = 1;
    expect(Fixture(lsdma_601).initialized == ClockGatingStatus::unsupported_ip,
           "LSDMA of another version refused");

    ClockGatingPolicy zero = clock_gating_linux_policy;
    zero.rlc_safe_mode_poll_iterations = 0;
    expect(Fixture({}, zero).initialized == ClockGatingStatus::invalid_argument,
           "zero safe-mode poll budget refused");

    ClockGating fresh;
    const rx7900::ByteView view(fixture.binary.data(), fixture.binary.size());
    expect(fresh.initialize(view, {}, fixture.nbio_registers, fixture.trace.io(), fixture.nbio,
                            clock_gating_linux_policy, {}) == ClockGatingStatus::unsupported_ip,
           "default GC token refused");
    RegisterIo small = fixture.trace.io();
    small.aperture_bytes = 0x80000;
    expect(fresh.initialize(view, fixture.gc, fixture.nbio_registers, small, fixture.nbio,
                            clock_gating_linux_policy, {}) == ClockGatingStatus::unavailable,
           "aperture mismatch refused");
    expect(fresh.gate(*fixture.ih, *fixture.smu) == ClockGatingStatus::unavailable &&
           fresh.ungate(*fixture.ih, *fixture.smu) == ClockGatingStatus::unavailable &&
           fresh.gfx_off_control(*fixture.smu, true) == ClockGatingStatus::unavailable,
           "uninitialized object refuses every operation");
}

void gate_sequence() {
    Fixture fixture;
    expect(fixture.gate() == ClockGatingStatus::gated, "late-init gating completes");
    const auto expected = join({hdp_gate, gmc_gate, ih_gate, gfx_gate, lsdma_gate,
                                {delay(100000)}, smu_send(smu13_allow_gfx_off_message)});
    expect(fixture.trace.log == expected,
           "soc21 HDP, MMHUB/ATHUB, IH, GFX CG in safe mode, LSDMA PG, then delayed AllowGfxOff");
    expect(fixture.device->smu_messages.back() == 0x28, "PPSMC_MSG_AllowGfxOff 0x28 reached PMFW");
    expect(fixture.reg(cgcg_cgls_3d) == 0, "3D CGCG/CGLS stay off (compiled out for 11.0.0)");
    const auto report = fixture.gating->report();
    expect(report.common_gated && report.gmc_gated && report.ih_gated && report.gfx_gated &&
           report.lsdma_gated && report.gfx_off_allowed && report.gfx_off_requests == 0 &&
           report.rlc_enabled && !report.rlc_in_safe_mode && report.rlc_safe_mode_polls == 1 &&
           report.completed_step == ClockGatingStep::complete &&
           report.smu_status == SmuStatus::completed && fixture.ih->report().clock_gated,
           "report records every gated block and GFXOFF allowed");
    expect(fixture.gate() == ClockGatingStatus::busy, "second gate refused");
}

void ungate_sequence() {
    Fixture fixture;
    fixture.gate();
    fixture.trace.log.clear();
    expect(fixture.ungate() == ClockGatingStatus::ungated, "fini ungating completes");
    const auto expected = join({
        smu_send(smu13_disallow_gfx_off_message),
        {mmio(lsdma_mem, 0x10u), mmio(lsdma_mem, 0x10u)},
        {mmio(safe_mode, 0x3u), mmio(cgcg_cgls, 0x0000363cu), mmio(sdma0_cgcg, 0),
         mmio(sdma1_cgcg, 0), mmio(mgcg_override, 0xfffff8e7u), mmio(mgcg_override, 0xfffffae7u),
         mmio(mgcg_override, 0xfffffbe7u), mmio(mgcg_override, 0xffffffe7u),
         mmio(cp_int_ring0, 0), mmio(safe_mode, 0x1u)},
        {mmio(ih_clk, 0xf8800000u)},
        {mmio(mm_atc_cg, 0x00080001u), mmio(mm_atc_cg, 0x00000001u), mmio(athub_misc, 0x80u),
         mmio(athub_misc, 0)},
        {smn(cpm_smn, 0x0000fe1cu), smn(pcie_cntl2_smn, 0x00000001u)},
        {mmio(hdp_clk, 0x88000001u), mmio(hdp_mem, 0xfff0fff0u), mmio(hdp_clk, 0x80000001u)}});
    expect(fixture.trace.log == expected,
           "DisallowGfxOff, LSDMA, then GFX, IH, MMHUB/ATHUB, NBIO (via SMN) and HDP ungated in reverse");
    expect(fixture.reg(wptr_poll) == 0x00900100u && fixture.reg(cp_int) == 0x003c0001u,
           "IDLE_POLL_COUNT and CP_INT_CNTL keep their gated values as in Linux");
    const auto report = fixture.gating->report();
    expect(!report.gfx_off_allowed && report.gfx_off_requests == 1 && !report.gfx_gated &&
           !report.common_gated && report.completed_step == ClockGatingStep::complete,
           "report returns to the ungated state");

    fixture.trace.log.clear();
    expect(fixture.gate() == ClockGatingStatus::gated &&
           fixture.trace.log.back() == mmio(smu_message, 0x28),
           "resume late init gates again");
}

void safe_mode_handshake() {
    Fixture slow;
    slow.trace.safe_mode_ack_after = 3;
    expect(slow.gate() == ClockGatingStatus::gated && slow.gating->report().rlc_safe_mode_polls == 4,
           "fourth RLC_SAFE_MODE read sees CMD cleared");
    size_t one_us = 0;

    for (const auto& access : slow.trace.log) {
        one_us += access == delay(1) ? 1 : 0;
    }

    expect(one_us == 3, "udelay(1) after each unacknowledged poll");

    ClockGatingPolicy short_budget = clock_gating_linux_policy;
    short_budget.rlc_safe_mode_poll_iterations = 5;
    Fixture hung({}, short_budget);
    hung.trace.safe_mode_never_ack = true;
    expect(hung.gate() == ClockGatingStatus::rlc_timeout, "unacknowledged safe mode times out");
    const auto expected = join({hdp_gate, gmc_gate, ih_gate,
                                {mmio(safe_mode, 0x3u), delay(1), delay(1), delay(1), delay(1),
                                 delay(1)}});
    expect(hung.trace.log == expected, "no GC CG store or safe-mode exit after the timeout");
    const auto report = hung.gating->report();
    expect(report.rlc_safe_mode_timeout && report.rlc_in_safe_mode &&
           report.rlc_safe_mode_polls == 5 && report.failed_step == ClockGatingStep::gfx_cg &&
           report.completed_step == ClockGatingStep::ih_cg && hung.device->smu_messages.size() == 3,
           "timeout reported at the GFX step with the request still outstanding");
    hung.trace.log.clear();
    expect(hung.gate() == ClockGatingStatus::quarantined &&
           hung.ungate() == ClockGatingStatus::quarantined &&
           hung.gating->gfx_off_control(*hung.smu, false) == ClockGatingStatus::quarantined &&
           hung.trace.log.empty(), "quarantine refuses all later register access");
    const rx7900::ByteView view(hung.binary.data(), hung.binary.size());
    expect(hung.gating->initialize(view, hung.gc, hung.nbio_registers, hung.trace.io(), hung.nbio,
                                   clock_gating_linux_policy, {}) == ClockGatingStatus::quarantined,
           "quarantined object cannot be re-initialized");
    expect(hung.gating->release_after_reset() == ClockGatingStatus::quarantined,
           "release requires confirmed reset");
    hung.device->reset_confirmed = true;
    expect(hung.gating->release_after_reset() == ClockGatingStatus::ungated &&
           hung.gating->initialize(view, hung.gc, hung.nbio_registers, hung.trace.io(), hung.nbio,
                                   clock_gating_linux_policy, {}) == ClockGatingStatus::ready,
           "confirmed reset allows re-initialization");

    Fixture disabled;
    disabled.reg(rlc_cntl) = 0;
    expect(disabled.gate() == ClockGatingStatus::gated && disabled.count(safe_mode) == 0 &&
           disabled.count(cgcg_cgls) == 1 && !disabled.gating->report().rlc_enabled,
           "RLC F32 disabled: no safe mode, CG still programmed (amdgpu_gfx_rlc_enter_safe_mode)");
}

void smn_fallback() {
    Layout far;
    far.hdp_base = hdp_far;
    Fixture fixture(far);
    expect(fixture.initialized == ClockGatingStatus::ready, "HDP beyond BAR5 accepted");
    constexpr uint32_t far_clk = (hdp_far + 0xd8u) * 4;
    constexpr uint32_t far_mem = (hdp_far + 0xd4u) * 4;
    fixture.trace.smn_space[far_clk] = 0x80000001u;
    fixture.trace.smn_space[far_mem] = 0xffffffffu;
    expect(fixture.gate() == ClockGatingStatus::gated, "gating through SMN completes");
    const std::vector<Access> head(fixture.trace.log.begin(), fixture.trace.log.begin() + 4);
    expect(head == std::vector<Access>{smn(far_clk, 0x88000001u), smn(far_mem, 0xfff0fff0u),
                                       smn(far_mem, 0xfff9fff9u), smn(far_clk, 0x80000001u)},
           "HDP stores use RSMU index/data at (base + reg) * 4");
    expect(fixture.reg(hdp_clk) == 0x80000001u, "near HDP alias untouched");

    Fixture gate_only;
    gate_only.gate();
    bool nbio_touched = false;

    for (const auto& access : gate_only.trace.log) {
        nbio_touched = nbio_touched || access.target == cpm_smn || access.target == pcie_cntl2_smn;
    }

    expect(!nbio_touched && gate_only.trace.smn_space[cpm_smn] == 0x0000ffffu,
           "NBIO MGCG/LS gating returns early without BIF_MGCG/BIF_LS");
}

void gfx_off_requests() {
    Fixture fixture;
    fixture.gate();
    fixture.trace.log.clear();
    auto& gating = *fixture.gating;
    expect(gating.gfx_off_control(*fixture.smu, false) == ClockGatingStatus::gated &&
           fixture.trace.log == smu_send(smu13_disallow_gfx_off_message),
           "first disable request sends DisallowGfxOff");
    fixture.trace.log.clear();
    gating.gfx_off_control(*fixture.smu, false);
    gating.gfx_off_control(*fixture.smu, true);
    expect(fixture.trace.log.empty() && gating.report().gfx_off_requests == 1,
           "nested requests only count");
    gating.gfx_off_control(*fixture.smu, true);
    expect(fixture.trace.log == join({{delay(100000)}, smu_send(smu13_allow_gfx_off_message)}) &&
           gating.report().gfx_off_allowed, "last release re-allows GFXOFF after the delay");
    fixture.trace.log.clear();
    expect(gating.gfx_off_control(*fixture.smu, true) == ClockGatingStatus::invalid_argument &&
           fixture.trace.log.empty() && gating.report().gfx_off_requests == 0,
           "unbalanced enable ignored (Linux WARN_ON_ONCE)");

    ClockGatingPolicy no_feature = clock_gating_linux_policy;
    no_feature.gfx_off_feature = false;
    Fixture disabled({}, no_feature);
    expect(disabled.gate() == ClockGatingStatus::gated &&
           disabled.device->smu_messages.size() == 3 && disabled.count(smu_message) == 0,
           "without PP_GFXOFF_MASK amdgpu_gfx_off_ctrl does nothing");

    Fixture no_dpm({}, clock_gating_linux_policy, false);
    expect(no_dpm.gate() == ClockGatingStatus::gated && no_dpm.device->smu_messages.empty() &&
           no_dpm.gating->report().gfx_off_without_dpm && !no_dpm.gating->report().gfx_off_allowed,
           "smu_dpm_set_power_gate refuses before DPM: no AllowGfxOff");
}

void smu_failures() {
    Fixture refused;
    refused.device->smu_fail_message = 0x28;
    expect(refused.gate() == ClockGatingStatus::smu_refused, "PMFW refusing AllowGfxOff reported");
    auto report = refused.gating->report();
    expect(report.gfx_gated && report.lsdma_gated && !report.gfx_off_allowed &&
           report.smu_status == SmuStatus::command_failed, "clock gating kept, GFXOFF not allowed");
    refused.trace.log.clear();
    refused.device->smu_fail_message = UINT32_MAX;
    expect(refused.ungate() == ClockGatingStatus::ungated && refused.count(smu_message) == 0,
           "ungate after refusal sends no DisallowGfxOff");

    Fixture failed;
    failed.gate();
    failed.trace.log.clear();
    failed.device->smu_fail_message = 0x29;
    expect(failed.ungate() == ClockGatingStatus::smu_refused &&
           failed.trace.log == smu_send(smu13_disallow_gfx_off_message),
           "failed DisallowGfxOff stops before any GC register");
    report = failed.gating->report();
    expect(report.failed_step == ClockGatingStep::gfx_pg && report.gfx_off_allowed &&
           failed.ungate() == ClockGatingStatus::quarantined, "GFX may be off: quarantined");
}

void mmio_failures() {
    Fixture fixture;
    fixture.trace.fail_write = athub_misc;
    expect(fixture.gate() == ClockGatingStatus::mmio_failure, "uncertain ATHUB store fails");
    const auto expected = join({hdp_gate, {mmio(mm_atc_cg, 0x00040001u), mmio(mm_atc_cg, 0x000c0001u)}});
    expect(fixture.trace.log == expected, "no store after the uncertain one");
    const auto report = fixture.gating->report();
    expect(report.failed_step == ClockGatingStep::gmc_cg &&
           report.completed_step == ClockGatingStep::common_cg && report.common_gated &&
           !report.gmc_gated, "failure step recorded");
    expect(fixture.ungate() == ClockGatingStatus::quarantined, "teardown refused while quarantined");

    Fixture gfx_store;
    gfx_store.trace.fail_write = cp_int;
    expect(gfx_store.gate() == ClockGatingStatus::mmio_failure &&
           gfx_store.count(safe_mode) == 1 && gfx_store.gating->report().rlc_in_safe_mode,
           "a GC store failure inside safe mode leaves no exit write");

    Fixture smn_store;
    smn_store.gate();
    smn_store.trace.fail_write = rsmu_data;
    expect(smn_store.ungate() == ClockGatingStatus::mmio_failure &&
           smn_store.gating->report().failed_step == ClockGatingStep::common_cg,
           "uncertain SMN store fails at the NBIO step");

    Fixture ih_down;
    ih_down.ih = std::make_unique<Ih6>();
    expect(ih_down.gate() == ClockGatingStatus::ih_failure &&
           ih_down.gating->report().ih_status == Ih6Status::unavailable &&
           ih_down.count(safe_mode) == 0, "uninitialized IH stops before GFX");
}

} // namespace

int main() {
    initialize_contract();
    gate_sequence();
    ungate_sequence();
    safe_mode_handshake();
    smn_fallback();
    gfx_off_requests();
    smu_failures();
    mmio_failures();
    return checks.finish("clock_gating_tests");
}

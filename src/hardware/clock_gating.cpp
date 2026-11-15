/*
Module: rx7900.hardware.clock_gating
Purpose: Program the Linux late-init clock/power gating and GFXOFF enable for Navi31.
Created: 2026-10-21
Architecture: Bounded register protocol through Nbio (direct BAR5 or SMN), Ih6 for
OSSSYS clock gating and Smu13 for the GFXOFF messages; the native owner supplies
MMIO and timing through RegisterIo.
Copyright 2021 Advanced Micro Devices, Inc.; Copyright 2022 Advanced Micro Devices, Inc.;
Copyright 2020 Advanced Micro Devices, Inc.; Copyright 2014 Advanced Micro Devices, Inc.;
Copyright 2011 Advanced Micro Devices, Inc.; Copyright 2008 Advanced Micro Devices, Inc.;
Copyright 2008 Red Hat Inc.; Copyright 2009 Jerome Glisse.
Adapted Linux v6.12 amdgpu_device.c (ip_late_init, set_cg_state, set_pg_state,
ip_fini_early), soc21.c, nbio_v4_3.c, hdp_v6_0.c, gmc_v11_0.c, mmhub_v3_0.c,
athub_v3_0.c, lsdma_v6_0.c, gfx_v11_0.c, amdgpu_rlc.c, amdgpu_gfx.c, amdgpu_dpm.c,
amdgpu_smu.c, smu_v13_0.c, sdma_v6_0.c, amdgpu_psp.c, mes_v11_0.c and the gc_11_0_0,
nbio_4_3_0, mmhub_3_0_0, athub_3_0_0, hdp_6_0_0 and lsdma_6_0_0 register headers.
Notice and provenance: licenses/AMDGPU-FIRMWARE.txt.

Hooks that are no-ops on this ASIC, so nothing is programmed for them:
psp_set_clockgating_state/psp_set_powergating_state and smu_set_clockgating_state/
smu_set_powergating_state return 0; sdma_v6_0_set_clockgating_state and
set_powergating_state return 0; gmc_v11_0_set_powergating_state returns 0;
ih_v6_0_set_powergating_state needs AMD_PG_SUPPORT_IH_SRAM_PG (absent); mes_v11_0
has no gating hooks; VCN and JPEG are skipped by amdgpu_device_set_cg/pg_state;
nbio_v4_3 MGCG/LS gating returns early without BIF_MGCG/BIF_LS (ungating still
clears the bits); gfx_v11_0 power gating for GC 11.0.0 is only amdgpu_gfx_off_ctrl.
*/
#include "clock_gating.hpp"

namespace rx7900::hardware {
namespace {

// gc_11_0_0_offset.h. RLC registers use GC segment 1, the others segment 0.
constexpr uint32_t rlc_cntl = 0x4c00;
constexpr uint32_t rlc_safe_mode = 0x0980;
constexpr uint32_t rlc_cgtt_mgcg_override = 0x4c48;
constexpr uint32_t rlc_cgcg_cgls_ctrl = 0x4c49;
constexpr uint32_t rlc_cgcg_cgls_ctrl_3d = 0x4cc5;
constexpr uint32_t cp_rb_wptr_poll_cntl = 0x0f62;
constexpr uint32_t cp_int_cntl = 0x1de9;
constexpr uint32_t cp_int_cntl_ring0 = 0x1e0a;
constexpr uint32_t sdma0_rlc_cgcg_ctrl = 0x005c;
constexpr uint32_t sdma1_rlc_cgcg_ctrl = 0x065c;

// gc_11_0_0_sh_mask.h.
constexpr uint32_t rlc_cntl_rlc_enable_f32 = 0x00000001u;
constexpr uint32_t rlc_safe_mode_cmd = 0x00000001u;
constexpr uint32_t rlc_safe_mode_message_shift = 1;
constexpr uint32_t override_rlc_cgtt_sclk = 0x00000002u;
constexpr uint32_t override_gfxip_mgcg = 0x00000004u;
constexpr uint32_t override_gfxip_cgcg = 0x00000008u;
constexpr uint32_t override_gfxip_cgls = 0x00000010u;
constexpr uint32_t override_grbm_cgtt_sclk = 0x00000020u;
constexpr uint32_t override_gfxip_fgcg = 0x00000100u;
constexpr uint32_t override_gfxip_repeater_fgcg = 0x00000200u;
constexpr uint32_t override_perfmon_clock_state = 0x00000400u;
constexpr uint32_t cgcg_cgls_cgcg_en = 0x00000001u;
constexpr uint32_t cgcg_cgls_cgls_en = 0x00000002u;
constexpr uint32_t cgcg_cgls_rep_compansat_delay_mask = 0x000000fcu;
constexpr uint32_t cgcg_cgls_rep_compansat_delay_shift = 2;
constexpr uint32_t cgcg_cgls_gfx_idle_threshold_mask = 0x07ffff00u;
constexpr uint32_t cgcg_cgls_gfx_idle_threshold_shift = 8;
constexpr uint32_t poll_cntl_poll_frequency_mask = 0x0000ffffu;
constexpr uint32_t poll_cntl_poll_frequency_shift = 0;
constexpr uint32_t poll_cntl_idle_poll_count_mask = 0xffff0000u;
constexpr uint32_t poll_cntl_idle_poll_count_shift = 16;
// CP_INT_CNTL and CP_INT_CNTL_RING0 share these bit positions.
constexpr uint32_t cp_int_cmp_busy = 0x00040000u;
constexpr uint32_t cp_int_cntx_busy = 0x00080000u;
constexpr uint32_t cp_int_cntx_empty = 0x00100000u;
constexpr uint32_t cp_int_gfx_idle = 0x00200000u;
constexpr uint32_t sdma_rlc_cgcg_int_enable = 0x00000002u;

// gfx_v11_0_update_coarse_grain_clock_gating field values.
constexpr uint32_t cgcg_gfx_idle_threshold = 0x36;
constexpr uint32_t cgls_rep_compansat_delay = 0x000f;
constexpr uint32_t poll_frequency = 0x0100;
constexpr uint32_t idle_poll_count = 0x0090;

constexpr uint32_t gui_idle_interrupts =
    cp_int_cntx_busy | cp_int_cntx_empty | cp_int_cmp_busy | cp_int_gfx_idle;

// nbio_4_3_0_offset.h, BASE_IDX 5 (beyond BAR5 on Navi31: Linux reaches them
// through amdgpu_device_indirect_rreg/wreg, as Nbio::read_register does).
constexpr uint32_t pcie_cntl2 = 0x28a001c;
constexpr uint32_t cpm_control = 0x28a0118;
constexpr size_t nbio_cpm_segment = 5;
// nbio_4_3_0_sh_mask.h.
constexpr uint32_t pcie_cntl2_slv_mem_ls_en = 0x00010000u;
constexpr uint32_t cpm_lclk_dyn_gate_enable = 0x00000001u;
constexpr uint32_t cpm_txclk_dyn_gate_enable = 0x00000002u;
constexpr uint32_t cpm_txclk_lcnt_gate_enable = 0x00000020u;
constexpr uint32_t cpm_txclk_regs_gate_enable = 0x00000040u;
constexpr uint32_t cpm_txclk_prbs_gate_enable = 0x00000080u;
constexpr uint32_t cpm_refclk_regs_gate_enable = 0x00000100u;
constexpr uint32_t cpm_gate_bits = cpm_lclk_dyn_gate_enable | cpm_txclk_dyn_gate_enable |
                                   cpm_txclk_lcnt_gate_enable | cpm_txclk_regs_gate_enable |
                                   cpm_txclk_prbs_gate_enable | cpm_refclk_regs_gate_enable;

// mmhub_3_0_0_offset.h / _sh_mask.h, BASE_IDX 0.
constexpr uint32_t mm_atc_l2_misc_cg = 0x06cd;
constexpr uint32_t mm_atc_l2_misc_cg_enable = 0x00040000u;
constexpr uint32_t mm_atc_l2_misc_cg_mem_ls_enable = 0x00080000u;

// athub_3_0_0_offset.h / _sh_mask.h, BASE_IDX 0 (ATHUB 3.0.0 uses
// regATHUB_MISC_CNTL; only 3.0.1 and 3.3.0 use the alternative offsets).
constexpr uint32_t athub_misc_cntl = 0x0075;
constexpr uint32_t athub_misc_cntl_cg_enable = 0x00000040u;
constexpr uint32_t athub_misc_cntl_cg_mem_ls_enable = 0x00000080u;

// hdp_6_0_0_offset.h / _sh_mask.h, BASE_IDX 0 (HDP 6.0.0, not the 6.1.0 offset).
constexpr uint32_t hdp_mem_power_ctrl = 0x00d4;
constexpr uint32_t hdp_clk_cntl = 0x00d8;
constexpr uint32_t hdp_clk_rc_mem_clk_soft_override = 0x08000000u;
constexpr uint32_t hdp_atomic_mem_power_ctrl_en = 0x00000001u;
constexpr uint32_t hdp_atomic_mem_power_ls_en = 0x00000002u;
constexpr uint32_t hdp_atomic_mem_power_ds_en = 0x00000004u;
constexpr uint32_t hdp_atomic_mem_power_sd_en = 0x00000008u;
constexpr uint32_t hdp_rc_mem_power_ctrl_en = 0x00010000u;
constexpr uint32_t hdp_rc_mem_power_ls_en = 0x00020000u;
constexpr uint32_t hdp_rc_mem_power_ds_en = 0x00040000u;
constexpr uint32_t hdp_rc_mem_power_sd_en = 0x00080000u;
constexpr uint32_t hdp_mem_power_all =
    hdp_atomic_mem_power_ctrl_en | hdp_atomic_mem_power_ls_en | hdp_atomic_mem_power_ds_en |
    hdp_atomic_mem_power_sd_en | hdp_rc_mem_power_ctrl_en | hdp_rc_mem_power_ls_en |
    hdp_rc_mem_power_ds_en | hdp_rc_mem_power_sd_en;

// lsdma_6_0_0_offset.h / _sh_mask.h, BASE_IDX 0.
constexpr uint32_t lsdma_mem_power_ctrl = 0x0071;
constexpr uint32_t lsdma_mem_power_ctrl_en = 0x00000001u;

static_assert((soc21_gc_11_0_0_cg_flags & (amd_cg::gfx_cgcg | amd_cg::gfx_mgcg)) != 0,
              "amdgpu_gfx_rlc_enter_safe_mode requires CGCG or MGCG");
static_assert((soc21_gc_11_0_0_cg_flags & amd_cg::hdp_sd) != 0, "HDP SD is the only HDP mode");

bool register_reachable(uint32_t base, uint32_t reg) noexcept {
    return (static_cast<uint64_t>(base) + reg) * 4 <= UINT32_MAX - 3;
}

// Smu13 responses that prove PMFW did not act on the message (or that no
// message was sent); anything else leaves the GFXOFF state uncertain.
bool definitive_refusal(SmuStatus status) noexcept {
    return status == SmuStatus::unavailable || status == SmuStatus::command_failed ||
           status == SmuStatus::unknown_command || status == SmuStatus::rejected_prerequisite ||
           status == SmuStatus::busy_other;
}

} // namespace

ClockGatingStatus ClockGating::initialize(ByteView discovery, memory::GcRegisterBase gc,
                                          NbioRegisters nbio_registers, RegisterIo io,
                                          Nbio& nbio, ClockGatingPolicy policy,
                                          ClockGatingRecovery recovery) noexcept {
    if (phase_ == Phase::faulted) {
        return ClockGatingStatus::quarantined;
    }

    if (phase_ == Phase::gated) {
        return ClockGatingStatus::busy;
    }

    phase_ = Phase::uninitialized;
    report_ = {};

    if (!gc.valid() || !nbio_registers.valid()) {
        report_.status = ClockGatingStatus::unsupported_ip;
        return report_.status;
    }

    if (!register_io_valid(io) || io.aperture_bytes != gc.aperture_bytes() ||
        io.aperture_bytes != nbio_registers.aperture_bytes()) {
        report_.status = ClockGatingStatus::unavailable;
        return report_.status;
    }

    if (policy.rlc_safe_mode_poll_iterations == 0) {
        report_.status = ClockGatingStatus::invalid_argument;
        return report_.status;
    }

    DiscoveredIp athub;
    DiscoveredIp hdp;
    DiscoveredIp lsdma;

    if (discover_ip(discovery, {discovery_athub_hardware_id, 0, 3, 0, 0, 1}, athub) !=
            DiscoveryStatus::validated ||
        discover_ip(discovery, {discovery_hdp_hardware_id, 0, 6, 0, 0, 1}, hdp) !=
            DiscoveryStatus::validated) {
        report_.status = ClockGatingStatus::unsupported_ip;
        return report_.status;
    }

    // soc21_common_set_powergating_state gates LSDMA 6.0.0 and 6.0.2. The binary
    // already validated above, so a missing record (invalid_binary) means no
    // LSDMA and Linux does nothing; a record of another version is refused.
    auto lsdma_status = discover_ip(discovery, {discovery_lsdma_hardware_id, 0, 6, 0, 0, 1}, lsdma);

    if (lsdma_status == DiscoveryStatus::unsupported) {
        lsdma_status = discover_ip(discovery, {discovery_lsdma_hardware_id, 0, 6, 0, 2, 1}, lsdma);
    }

    if (lsdma_status != DiscoveryStatus::validated && lsdma_status != DiscoveryStatus::invalid_binary) {
        report_.status = ClockGatingStatus::unsupported_ip;
        return report_.status;
    }

    const bool lsdma_present = lsdma_status == DiscoveryStatus::validated;

    if (!register_reachable(gc.segment(0), cp_int_cntl_ring0) ||
        !register_reachable(gc.segment(1), rlc_cgcg_cgls_ctrl_3d) ||
        !register_reachable(nbio_registers.nbif_base(nbio_cpm_segment), cpm_control) ||
        !register_reachable(nbio_registers.mmhub_base(), mm_atc_l2_misc_cg) ||
        !register_reachable(athub.segment(0), athub_misc_cntl) ||
        !register_reachable(hdp.segment(0), hdp_clk_cntl) ||
        (lsdma_present && !register_reachable(lsdma.segment(0), lsdma_mem_power_ctrl))) {
        report_.status = ClockGatingStatus::unsupported_ip;
        return report_.status;
    }

    gc_ = gc;
    nbio_registers_ = nbio_registers;
    io_ = io;
    nbio_ = &nbio;
    policy_ = policy;
    recovery_ = recovery;
    report_.athub_base = athub.segment(0);
    report_.hdp_base = hdp.segment(0);
    report_.lsdma_base = lsdma_present ? lsdma.segment(0) : 0;
    report_.lsdma_present = lsdma_present;
    report_.status = ClockGatingStatus::ready;
    phase_ = Phase::ready;
    return report_.status;
}

// amdgpu_device_ip_late_init: set_cg_state(GATE), set_pg_state(GATE), each over
// soc21_common, gmc_v11_0, ih_v6_0, psp_v13_0, smu_v13_0, gfx_v11_0, sdma_v6_0,
// vcn/jpeg (skipped by type) and mes_v11_0 (no hooks), in that order.
ClockGatingStatus ClockGating::gate(Ih6& ih, Smu13& smu) noexcept {
    if (phase_ != Phase::ready) {
        return phase_ == Phase::faulted ? ClockGatingStatus::quarantined :
               phase_ == Phase::gated ? ClockGatingStatus::busy : ClockGatingStatus::unavailable;
    }

    report_.completed_step = ClockGatingStep::none;
    report_.failed_step = ClockGatingStep::none;

    if (common_clock_gating(true) != ClockGatingStatus::ready) {
        return report_.status;
    }

    report_.completed_step = ClockGatingStep::common_cg;

    if (gmc_clock_gating(true) != ClockGatingStatus::ready) {
        return report_.status;
    }

    report_.completed_step = ClockGatingStep::gmc_cg;

    if (ih_clock_gating(ih, true) != ClockGatingStatus::ready) {
        return report_.status;
    }

    report_.completed_step = ClockGatingStep::ih_cg;

    // psp and smu set_clockgating_state: no-ops.
    if (gfx_clock_gating(true) != ClockGatingStatus::ready) {
        return report_.status;
    }

    report_.completed_step = ClockGatingStep::gfx_cg;

    // sdma_v6_0_set_clockgating_state: no-op. Power gating follows.
    if (lsdma_power_gating(true) != ClockGatingStatus::ready) {
        return report_.status;
    }

    report_.completed_step = ClockGatingStep::common_pg;
    // gmc, ih (no IH_SRAM_PG), psp and smu set_powergating_state: no-ops.
    phase_ = Phase::gated;
    report_.status = ClockGatingStatus::gated;

    // gfx_v11_0_set_powergating_state for GC 11.0.0: amdgpu_gfx_off_ctrl(true).
    // The request count drops from 1 to 0, so Linux schedules gfx_off_delay_work
    // and AllowGfxOff follows 100 ms later; sdma_v6_0 power gating is a no-op.
    const ClockGatingStatus status = gfx_off_control(smu, true);

    if (phase_ == Phase::faulted) {
        return status;
    }

    report_.completed_step = ClockGatingStep::complete;
    return report_.status;
}

// amdgpu_device_ip_fini_early / suspend_phase1: set_pg_state(UNGATE) then
// set_cg_state(UNGATE), both walking the IP blocks in reverse order.
ClockGatingStatus ClockGating::ungate(Ih6& ih, Smu13& smu) noexcept {
    if (phase_ == Phase::faulted || phase_ == Phase::uninitialized) {
        return phase_ == Phase::faulted ? ClockGatingStatus::quarantined :
                                          ClockGatingStatus::unavailable;
    }

    report_.completed_step = ClockGatingStep::none;
    report_.failed_step = ClockGatingStep::none;

    // sdma_v6_0 PG no-op, then gfx_v11_0: amdgpu_gfx_off_ctrl(false).
    const ClockGatingStatus gfx_off = gfx_off_control(smu, false);

    if (phase_ == Phase::faulted) {
        return gfx_off;
    }

    report_.completed_step = ClockGatingStep::gfx_pg;

    // smu, psp, ih, gmc PG no-ops, then soc21_common LSDMA.
    if (lsdma_power_gating(false) != ClockGatingStatus::ready) {
        return report_.status;
    }

    report_.completed_step = ClockGatingStep::common_pg;

    // Clock gating in reverse: sdma no-op, gfx, smu/psp no-ops, ih, gmc, common.
    if (gfx_clock_gating(false) != ClockGatingStatus::ready) {
        return report_.status;
    }

    report_.completed_step = ClockGatingStep::gfx_cg;

    if (ih_clock_gating(ih, false) != ClockGatingStatus::ready) {
        return report_.status;
    }

    report_.completed_step = ClockGatingStep::ih_cg;

    if (gmc_clock_gating(false) != ClockGatingStatus::ready) {
        return report_.status;
    }

    report_.completed_step = ClockGatingStep::gmc_cg;

    if (common_clock_gating(false) != ClockGatingStatus::ready) {
        return report_.status;
    }

    report_.completed_step = ClockGatingStep::complete;
    phase_ = Phase::ready;
    report_.status = ClockGatingStatus::ungated;
    return report_.status;
}

// amdgpu_gfx_off_ctrl with adev->in_s0ix false. Linux defers the enable to
// gfx_off_delay_work; calls here are serialized, so the delay runs inline and
// cannot be cancelled by a concurrent disable request.
ClockGatingStatus ClockGating::gfx_off_control(Smu13& smu, bool enable) noexcept {
    if (phase_ == Phase::faulted || phase_ == Phase::uninitialized) {
        return phase_ == Phase::faulted ? ClockGatingStatus::quarantined :
                                          ClockGatingStatus::unavailable;
    }

    if (!policy_.gfx_off_feature) {
        return report_.status;
    }

    if (enable) {
        // Linux WARN_ON_ONCE: an unbalanced enable is ignored.
        if (report_.gfx_off_requests == 0) {
            return ClockGatingStatus::invalid_argument;
        }

        --report_.gfx_off_requests;

        if (report_.gfx_off_requests != 0 || report_.gfx_off_allowed) {
            return report_.status;
        }

        io_.delay_microseconds(io_.context, policy_.gfx_off_delay_microseconds);
        return smu_gfx_off(smu, true);
    }

    if (report_.gfx_off_requests == UINT32_MAX) {
        return ClockGatingStatus::invalid_argument;
    }

    if (report_.gfx_off_requests == 0 && report_.gfx_off_allowed) {
        const ClockGatingStatus status = smu_gfx_off(smu, false);

        if (phase_ == Phase::faulted) {
            return status;
        }
    }

    ++report_.gfx_off_requests;
    return report_.status;
}

ClockGatingStatus ClockGating::release_after_reset() noexcept {
    if (phase_ == Phase::uninitialized) {
        return ClockGatingStatus::unavailable;
    }

    if (recovery_.confirm_reset_quiescence == nullptr ||
        !recovery_.confirm_reset_quiescence(recovery_.context)) {
        return phase_ == Phase::faulted ? ClockGatingStatus::quarantined : ClockGatingStatus::busy;
    }

    report_ = {};
    report_.status = ClockGatingStatus::ungated;
    phase_ = Phase::uninitialized;
    nbio_ = nullptr;
    return report_.status;
}

// Linux RREG32/WREG32: direct BAR5 MMIO, or the RSMU index/data pair beyond it.
bool ClockGating::read(uint32_t base, uint32_t reg, uint32_t& value) noexcept {
    return nbio_ != nullptr && nbio_->read_register(base, reg, value);
}

bool ClockGating::write(uint32_t base, uint32_t reg, uint32_t value) noexcept {
    return nbio_ != nullptr && nbio_->write_register(base, reg, value);
}

// def = data = RREG32(); data = (data & ~clear) | set; if (def != data) WREG32().
// always_write models the Linux sequences that store without comparing.
bool ClockGating::update(uint32_t base, uint32_t reg, uint32_t clear, uint32_t set,
                         bool always_write) noexcept {
    uint32_t value = 0;

    if (!read(base, reg, value)) {
        return false;
    }

    const uint32_t desired = (value & ~clear) | set;
    return (!always_write && desired == value) || write(base, reg, desired);
}

// soc21_common_set_clockgating_state for NBIO 4.3.0.
ClockGatingStatus ClockGating::common_clock_gating(bool gate) noexcept {
    const uint32_t nbio5 = nbio_registers_.nbif_base(nbio_cpm_segment);

    // nbio_v4_3_update_medium_grain_clock_gating and _light_sleep return before
    // any access when gating without BIF_MGCG/BIF_LS; ungating always clears.
    if (!gate && (!update(nbio5, cpm_control, cpm_gate_bits, 0, false) ||
                  !update(nbio5, pcie_cntl2, pcie_cntl2_slv_mem_ls_en, 0, false))) {
        return fail(ClockGatingStep::common_cg, ClockGatingStatus::mmio_failure);
    }

    // hdp_v6_0_update_clock_gating with only AMD_CG_SUPPORT_HDP_SD.
    const uint32_t hdp = report_.hdp_base;
    uint32_t clk = 0;
    uint32_t mem = 0;

    if (!read(hdp, hdp_clk_cntl, clk) || !read(hdp, hdp_mem_power_ctrl, mem)) {
        return fail(ClockGatingStep::common_cg, ClockGatingStatus::mmio_failure);
    }

    clk |= hdp_clk_rc_mem_clk_soft_override;

    if (!write(hdp, hdp_clk_cntl, clk)) {
        return fail(ClockGatingStep::common_cg, ClockGatingStatus::mmio_failure);
    }

    mem &= ~hdp_mem_power_all;

    if (!write(hdp, hdp_mem_power_ctrl, mem)) {
        return fail(ClockGatingStep::common_cg, ClockGatingStatus::mmio_failure);
    }

    if (gate) {
        mem |= hdp_atomic_mem_power_sd_en | hdp_rc_mem_power_sd_en |
               hdp_atomic_mem_power_ctrl_en | hdp_rc_mem_power_ctrl_en;

        if (!write(hdp, hdp_mem_power_ctrl, mem)) {
            return fail(ClockGatingStep::common_cg, ClockGatingStatus::mmio_failure);
        }
    }

    clk &= ~hdp_clk_rc_mem_clk_soft_override;

    if (!write(hdp, hdp_clk_cntl, clk)) {
        return fail(ClockGatingStep::common_cg, ClockGatingStatus::mmio_failure);
    }

    report_.common_gated = gate;
    return ClockGatingStatus::ready;
}

// gmc_v11_0_set_clockgating_state: mmhub_v3_0 (MC_MGCG then MC_LS), then
// athub_v3_0 3.0.0 (ATHUB_MGCG then ATHUB_LS), each its own read-modify-write.
ClockGatingStatus ClockGating::gmc_clock_gating(bool gate) noexcept {
    const uint32_t mmhub = nbio_registers_.mmhub_base();
    const uint32_t athub = report_.athub_base;
    const uint32_t masks[2]{mm_atc_l2_misc_cg_enable, mm_atc_l2_misc_cg_mem_ls_enable};
    const uint32_t athub_masks[2]{athub_misc_cntl_cg_enable, athub_misc_cntl_cg_mem_ls_enable};

    for (const uint32_t mask : masks) {
        if (!update(mmhub, mm_atc_l2_misc_cg, gate ? 0 : mask, gate ? mask : 0, false)) {
            return fail(ClockGatingStep::gmc_cg, ClockGatingStatus::mmio_failure);
        }
    }

    for (const uint32_t mask : athub_masks) {
        if (!update(athub, athub_misc_cntl, gate ? 0 : mask, gate ? mask : 0, false)) {
            return fail(ClockGatingStep::gmc_cg, ClockGatingStatus::mmio_failure);
        }
    }

    report_.gmc_gated = gate;
    return ClockGatingStatus::ready;
}

// ih_v6_0_set_clockgating_state (AMD_CG_SUPPORT_IH_CG), owned by Ih6.
ClockGatingStatus ClockGating::ih_clock_gating(Ih6& ih, bool gate) noexcept {
    const Ih6Status status = ih.set_clockgating(gate);
    report_.ih_status = status;

    if (status == Ih6Status::quarantined || status == Ih6Status::unavailable ||
        status == Ih6Status::mmio_failure || ih.report().clock_gated != gate) {
        return fail(ClockGatingStep::ih_cg, ClockGatingStatus::ih_failure);
    }

    report_.ih_gated = gate;
    return ClockGatingStatus::ready;
}

// gfx_v11_0_update_gfx_clock_gating: safe mode, coarse grain, medium grain,
// repeater FGCG, SRAM FGCG, perfmon clock, GUI-idle interrupt, safe-mode exit.
ClockGatingStatus ClockGating::gfx_clock_gating(bool gate) noexcept {
    const ClockGatingStatus entered = enter_safe_mode();

    if (entered != ClockGatingStatus::ready) {
        return entered;
    }

    // gfx_v11_0_enable_gui_idle_interrupt: GC 11.0.0 has one ME with one gfx
    // pipe, so only CP_INT_CNTL_RING0 (me 0, pipe 0) is written.
    if (!gfx_coarse_grain(gate) || !gfx_override_bits(gate) ||
        !update(gc_.segment(0), cp_int_cntl_ring0, gate ? 0 : gui_idle_interrupts,
                gate ? gui_idle_interrupts : 0, true) ||
        !exit_safe_mode()) {
        return fail(ClockGatingStep::gfx_cg, ClockGatingStatus::mmio_failure);
    }

    report_.gfx_gated = gate;
    return ClockGatingStatus::ready;
}

// gfx_v11_0_update_coarse_grain_clock_gating with CGCG and CGLS (no 3D flags).
bool ClockGating::gfx_coarse_grain(bool gate) noexcept {
    const uint32_t gc0 = gc_.segment(0);
    const uint32_t gc1 = gc_.segment(1);
    uint32_t unused_3d = 0;

    if (!gate) {
        // RLC_CGCG_CGLS_CTRL_3D is read but unchanged without the 3D flags; the
        // SDMA CGCG interrupt enables are cleared with unconditional stores.
        return update(gc1, rlc_cgcg_cgls_ctrl, cgcg_cgls_cgcg_en | cgcg_cgls_cgls_en, 0, false) &&
               read(gc1, rlc_cgcg_cgls_ctrl_3d, unused_3d) &&
               update(gc0, sdma0_rlc_cgcg_ctrl, sdma_rlc_cgcg_int_enable, 0, true) &&
               update(gc0, sdma1_rlc_cgcg_ctrl, sdma_rlc_cgcg_int_enable, 0, true);
    }

    // Linux clears the threshold field then the delay field in two steps; the
    // masks and set values do not overlap, so one combined update is identical
    // (0x0000363f from a zero register).
    constexpr uint32_t ctrl_clear = cgcg_cgls_gfx_idle_threshold_mask |
                                    cgcg_cgls_rep_compansat_delay_mask;
    constexpr uint32_t ctrl_set =
        (cgcg_gfx_idle_threshold << cgcg_cgls_gfx_idle_threshold_shift) | cgcg_cgls_cgcg_en |
        (cgls_rep_compansat_delay << cgcg_cgls_rep_compansat_delay_shift) | cgcg_cgls_cgls_en;
    constexpr uint32_t poll_clear = poll_cntl_poll_frequency_mask | poll_cntl_idle_poll_count_mask;
    constexpr uint32_t poll_set = (poll_frequency << poll_cntl_poll_frequency_shift) |
                                  (idle_poll_count << poll_cntl_idle_poll_count_shift);
    static_assert(ctrl_set == 0x0000363fu, "RLC_CGCG_CGLS_CTRL FSM value");
    static_assert(poll_set == 0x00900100u, "IDLE_POLL_COUNT value");

    return update(gc1, rlc_cgtt_mgcg_override, override_gfxip_cgcg | override_gfxip_cgls, 0,
                  false) &&
           update(gc1, rlc_cgcg_cgls_ctrl, ctrl_clear, ctrl_set, false) &&
           read(gc1, rlc_cgcg_cgls_ctrl_3d, unused_3d) &&
           update(gc0, cp_rb_wptr_poll_cntl, poll_clear, poll_set, false) &&
           update(gc0, cp_int_cntl, 0, gui_idle_interrupts, true) &&
           update(gc0, sdma0_rlc_cgcg_ctrl, 0, sdma_rlc_cgcg_int_enable, true) &&
           update(gc0, sdma1_rlc_cgcg_ctrl, 0, sdma_rlc_cgcg_int_enable, true);
}

// gfx_v11_0_update_medium_grain_clock_gating (MGCG), update_repeater_fgcg,
// update_sram_fgcg and update_perf_clk: four separate RLC_CGTT_MGCG_OVERRIDE
// read-modify-writes, each storing only when the value changes.
bool ClockGating::gfx_override_bits(bool gate) noexcept {
    const uint32_t masks[4]{
        override_grbm_cgtt_sclk | override_rlc_cgtt_sclk | override_gfxip_mgcg,
        override_gfxip_repeater_fgcg, override_gfxip_fgcg, override_perfmon_clock_state};

    for (const uint32_t mask : masks) {
        if (!update(gc_.segment(1), rlc_cgtt_mgcg_override, gate ? mask : 0, gate ? 0 : mask,
                    false)) {
            return false;
        }
    }

    return true;
}

// amdgpu_gfx_rlc_enter_safe_mode / gfx_v11_0_set_safe_mode. With the RLC F32
// disabled Linux skips safe mode yet still programs the CG registers.
ClockGatingStatus ClockGating::enter_safe_mode() noexcept {
    if (report_.rlc_in_safe_mode) {
        return ClockGatingStatus::ready;
    }

    uint32_t value = 0;

    if (!read(gc_.segment(1), rlc_cntl, value)) {
        return fail(ClockGatingStep::gfx_cg, ClockGatingStatus::mmio_failure);
    }

    report_.rlc_enabled = (value & rlc_cntl_rlc_enable_f32) != 0;

    if (!report_.rlc_enabled) {
        return ClockGatingStatus::ready;
    }

    if (!write(gc_.segment(1), rlc_safe_mode,
               rlc_safe_mode_cmd | (1u << rlc_safe_mode_message_shift))) {
        return fail(ClockGatingStep::gfx_cg, ClockGatingStatus::mmio_failure);
    }

    report_.rlc_in_safe_mode = true;
    report_.rlc_safe_mode_polls = 0;

    for (uint32_t poll = 0; poll < policy_.rlc_safe_mode_poll_iterations; ++poll) {
        if (!read(gc_.segment(1), rlc_safe_mode, value)) {
            return fail(ClockGatingStep::gfx_cg, ClockGatingStatus::mmio_failure);
        }

        ++report_.rlc_safe_mode_polls;

        if ((value & rlc_safe_mode_cmd) == 0) {
            return ClockGatingStatus::ready;
        }

        io_.delay_microseconds(io_.context, 1);
    }

    // Linux continues silently after the timeout. The RLC has not acknowledged
    // safe mode, so changing its gating FSM now could race it: stop instead.
    report_.rlc_safe_mode_timeout = true;
    return fail(ClockGatingStep::gfx_cg, ClockGatingStatus::rlc_timeout);
}

// amdgpu_gfx_rlc_exit_safe_mode / gfx_v11_0_unset_safe_mode (MESSAGE 0).
bool ClockGating::exit_safe_mode() noexcept {
    if (!report_.rlc_in_safe_mode) {
        return true;
    }

    uint32_t value = 0;

    if (!read(gc_.segment(1), rlc_cntl, value)) {
        return false;
    }

    if ((value & rlc_cntl_rlc_enable_f32) == 0) {
        // Linux leaves in_safe_mode set when the RLC has been disabled meanwhile.
        report_.rlc_enabled = false;
        return true;
    }

    if (!write(gc_.segment(1), rlc_safe_mode, rlc_safe_mode_cmd)) {
        return false;
    }

    report_.rlc_in_safe_mode = false;
    return true;
}

// soc21_common_set_powergating_state: lsdma_v6_0_update_memory_power_gating
// clears MEM_POWER_CTRL_EN, then stores it equal to the requested state.
ClockGatingStatus ClockGating::lsdma_power_gating(bool gate) noexcept {
    if (!report_.lsdma_present) {
        return ClockGatingStatus::ready;
    }

    uint32_t value = 0;

    if (!read(report_.lsdma_base, lsdma_mem_power_ctrl, value)) {
        return fail(ClockGatingStep::common_pg, ClockGatingStatus::mmio_failure);
    }

    value &= ~lsdma_mem_power_ctrl_en;

    if (!write(report_.lsdma_base, lsdma_mem_power_ctrl, value)) {
        return fail(ClockGatingStep::common_pg, ClockGatingStatus::mmio_failure);
    }

    if (gate) {
        value |= lsdma_mem_power_ctrl_en;
    }

    if (!write(report_.lsdma_base, lsdma_mem_power_ctrl, value)) {
        return fail(ClockGatingStep::common_pg, ClockGatingStatus::mmio_failure);
    }

    report_.lsdma_gated = gate;
    return ClockGatingStatus::ready;
}

// amdgpu_dpm_set_powergating_by_smu(GFX) -> smu_dpm_set_power_gate ->
// smu_v13_0_gfx_off_control for MP1 13.0.0: AllowGfxOff or DisallowGfxOff.
ClockGatingStatus ClockGating::smu_gfx_off(Smu13& smu, bool allow) noexcept {
    // smu_dpm_set_power_gate refuses (-EOPNOTSUPP) until smu_hw_init enabled
    // DPM; EnableAllSmuFeatures and its readback are this module's evidence.
    if (smu.report().enabled_features == 0) {
        report_.gfx_off_without_dpm = true;

        if (allow) {
            return report_.status;
        }

        return fail(ClockGatingStep::gfx_pg, ClockGatingStatus::smu_failure);
    }

    const SmuStatus status = smu.send(allow ? smu13_allow_gfx_off_message :
                                              smu13_disallow_gfx_off_message, 0, nullptr);
    report_.smu_status = status;

    if (status == SmuStatus::completed) {
        report_.gfx_off_allowed = allow;
        return report_.status;
    }

    // A refused AllowGfxOff leaves GFX powered (Linux logs and keeps
    // gfx_off_state false). A failed DisallowGfxOff leaves GFX possibly off,
    // so no GC register may be touched afterwards.
    if (allow && definitive_refusal(status)) {
        report_.status = ClockGatingStatus::smu_refused;
        return report_.status;
    }

    return fail(ClockGatingStep::gfx_pg, definitive_refusal(status) ?
                ClockGatingStatus::smu_refused : ClockGatingStatus::smu_failure);
}

ClockGatingStatus ClockGating::fail(ClockGatingStep step, ClockGatingStatus reason) noexcept {
    phase_ = Phase::faulted;
    report_.failed_step = step;
    report_.status = reason;
    return reason;
}

} // namespace rx7900::hardware

/*
Module: rx7900.hardware.clock_gating
Purpose: Enable and disable Navi31 clock and power gating as Linux late init/fini do.
Created: 2026-10-21
Architecture: Discovery-validated GC/NBIO tokens plus ATHUB/HDP/LSDMA records,
Nbio register access (SMN beyond BAR5), Ih6::set_clockgating for OSSSYS and
Smu13::send for the GFXOFF mailbox messages; no clock or DPM policy here.
Copyright 2021 Advanced Micro Devices, Inc.; Copyright 2022 Advanced Micro Devices, Inc.;
Copyright 2020 Advanced Micro Devices, Inc.; Copyright 2014 Advanced Micro Devices, Inc.;
Copyright 2008 Advanced Micro Devices, Inc.; Copyright 2008 Red Hat Inc.;
Copyright 2009 Jerome Glisse.
Adapted Linux v6.12 amdgpu_device.c, soc21.c, amd_shared.h, amdgpu_drv.c,
amdgpu_gfx.c, amdgpu_rlc.c, amdgpu_dpm.c, amdgpu_smu.c, smu_v13_0.c,
smu_v13_0_0_ppsmc.h and soc15_hw_ip.h. Notice and provenance: licenses/AMDGPU-FIRMWARE.txt.
*/
#pragma once

#include "gfxhub.hpp"
#include "ih6.hpp"
#include "smu13.hpp"

namespace rx7900::hardware {

// soc15_hw_ip.h identifiers that discovery.hpp does not own.
constexpr uint16_t discovery_athub_hardware_id = 35;
constexpr uint16_t discovery_hdp_hardware_id = 41;
constexpr uint16_t discovery_lsdma_hardware_id = 91;

// amd_shared.h AMD_CG_SUPPORT_* and AMD_PG_SUPPORT_* bits used by this module.
namespace amd_cg {
constexpr uint64_t gfx_mgcg = 1ULL << 0;
constexpr uint64_t gfx_cgcg = 1ULL << 2;
constexpr uint64_t gfx_cgls = 1ULL << 3;
constexpr uint64_t mc_ls = 1ULL << 8;
constexpr uint64_t mc_mgcg = 1ULL << 9;
constexpr uint64_t vcn_mgcg = 1ULL << 24;
constexpr uint64_t hdp_sd = 1ULL << 26;
constexpr uint64_t ih_cg = 1ULL << 27;
constexpr uint64_t athub_ls = 1ULL << 28;
constexpr uint64_t athub_mgcg = 1ULL << 29;
constexpr uint64_t jpeg_mgcg = 1ULL << 30;
constexpr uint64_t gfx_fgcg = 1ULL << 31;
constexpr uint64_t repeater_fgcg = 1ULL << 32;
constexpr uint64_t gfx_perf_clk = 1ULL << 33;
} // namespace amd_cg

namespace amd_pg {
constexpr uint32_t mmhub = 1u << 13;
constexpr uint32_t vcn = 1u << 14;
constexpr uint32_t vcn_dpg = 1u << 15;
constexpr uint32_t athub = 1u << 16;
constexpr uint32_t jpeg = 1u << 17;
} // namespace amd_pg

// soc21_common_early_init for GC 11.0.0 (3D_CGCG/3D_CGLS are compiled out by
// "#if 0"), then amdgpu_device_ip_early_init masks with the default module
// parameters amdgpu_cg_mask = ~0ULL and amdgpu_pg_mask = ~0u, which keep them.
// BIF_MGCG/BIF_LS, HDP_LS/HDP_DS, GFX_PG and IH_SRAM_PG are absent.
constexpr uint64_t soc21_gc_11_0_0_cg_flags =
    amd_cg::gfx_cgcg | amd_cg::gfx_cgls | amd_cg::gfx_mgcg | amd_cg::repeater_fgcg |
    amd_cg::gfx_fgcg | amd_cg::gfx_perf_clk | amd_cg::vcn_mgcg | amd_cg::jpeg_mgcg |
    amd_cg::athub_mgcg | amd_cg::athub_ls | amd_cg::mc_mgcg | amd_cg::mc_ls |
    amd_cg::ih_cg | amd_cg::hdp_sd;
constexpr uint32_t soc21_gc_11_0_0_pg_flags =
    amd_pg::vcn | amd_pg::vcn_dpg | amd_pg::jpeg | amd_pg::athub | amd_pg::mmhub;

// smu_v13_0_0_ppsmc.h PPSMC_MSG_AllowGfxOff / PPSMC_MSG_DisallowGfxOff
// (smu_v13_0_0_ppt.c maps SMU_MSG_AllowGfxOff/DisallowGfxOff to them).
constexpr uint16_t smu13_allow_gfx_off_message = 0x28;
constexpr uint16_t smu13_disallow_gfx_off_message = 0x29;

enum class ClockGatingStatus {
    ready,
    gated,
    ungated,
    unavailable,
    unsupported_ip,
    invalid_argument,
    mmio_failure,
    rlc_timeout,
    ih_failure,
    smu_refused,
    smu_failure,
    busy,
    quarantined
};

// Linux amdgpu_device_set_cg_state/set_pg_state visit IP blocks in this order
// (gate) and in reverse (ungate). Steps without a hook that changes state on
// this ASIC are recorded so a report shows how far a sequence progressed.
enum class ClockGatingStep : uint8_t {
    none,
    common_cg,   // soc21_common: nbio_v4_3 MGCG and LS, hdp_v6_0
    gmc_cg,      // gmc_v11_0: mmhub_v3_0 MGCG and LS, athub_v3_0 MGCG and LS
    ih_cg,       // ih_v6_0 IH_CLK_CTRL soft overrides
    gfx_cg,      // gfx_v11_0_update_gfx_clock_gating inside RLC safe mode
    common_pg,   // soc21_common: lsdma_v6_0 memory power gating
    gfx_pg,      // gfx_v11_0: amdgpu_gfx_off_ctrl and SMU AllowGfxOff/DisallowGfxOff
    complete
};

struct ClockGatingPolicy final {
    // Linux adev->usec_timeout (100000) polls of RLC_SAFE_MODE.CMD, 1 us apart.
    uint32_t rlc_safe_mode_poll_iterations;
    // Linux GFX_OFF_DELAY_ENABLE (100 ms) before AllowGfxOff.
    uint32_t gfx_off_delay_microseconds;
    // adev->pm.pp_feature PP_GFXOFF_MASK: set by the default amdgpu_pp_feature_mask
    // 0xfff7bfff and kept on bare metal with the default HWS scheduler policy.
    bool gfx_off_feature;
};

constexpr ClockGatingPolicy clock_gating_linux_policy{100000, 100000, true};

struct ClockGatingRecovery final {
    void* context;
    // True only after a reset proves the gating registers returned to their
    // reset defaults and no RLC safe-mode request is outstanding.
    bool (*confirm_reset_quiescence)(void*) noexcept;
};

struct ClockGatingReport final {
    ClockGatingStatus status = ClockGatingStatus::unavailable;
    uint64_t cg_flags = soc21_gc_11_0_0_cg_flags;
    uint32_t pg_flags = soc21_gc_11_0_0_pg_flags;
    uint32_t athub_base = 0;
    uint32_t hdp_base = 0;
    uint32_t lsdma_base = 0;
    // LSDMA instance0 6.0.0 or 6.0.2 present; otherwise soc21 PG is a Linux no-op.
    bool lsdma_present = false;
    // Last step that finished, and the step that failed (none on success).
    ClockGatingStep completed_step = ClockGatingStep::none;
    ClockGatingStep failed_step = ClockGatingStep::none;
    bool common_gated = false;
    bool gmc_gated = false;
    bool ih_gated = false;
    bool gfx_gated = false;
    bool lsdma_gated = false;
    // RLC_CNTL.RLC_ENABLE_F32 seen by the last safe-mode entry.
    bool rlc_enabled = false;
    bool rlc_in_safe_mode = false;
    bool rlc_safe_mode_timeout = false;
    uint32_t rlc_safe_mode_polls = 0;
    // amdgpu_gfx_off_ctrl state: request count (1 after init) and gfx_off_state.
    uint32_t gfx_off_requests = 1;
    bool gfx_off_allowed = false;
    // smu_dpm_set_power_gate refused because SMU features were never enabled.
    bool gfx_off_without_dpm = false;
    SmuStatus smu_status = SmuStatus::unavailable;
    Ih6Status ih_status = Ih6Status::unavailable;
};

// Serialized PASSIVE_LEVEL use by the exclusive device owner, sharing Nbio's SMN
// pair with the other physical-init protocols; no ISR/DPC may touch GC, HDP,
// MMHUB, ATHUB, LSDMA or NBIO registers while a call runs. gate blocks for the
// GFXOFF delay (Linux runs it as delayed work 100 ms after late init).
//
// Any uncertain register store, an RLC safe-mode timeout, an IH failure or an
// uncertain SMU response quarantines the object: no later register access occurs
// until release_after_reset. After gate allows GFXOFF, other owners must bracket
// direct GC register access with gfx_off_control(false)/(true) as Linux does.
class ClockGating final {
public:
    ClockGating() noexcept = default;
    ClockGating(const ClockGating&) = delete;
    ClockGating& operator=(const ClockGating&) = delete;

    // discovery is the validated immutable IP-discovery binary (borrowed only
    // for this call): ATHUB instance0 3.0.0 and HDP instance0 6.0.0 are required,
    // LSDMA instance0 is optional (6.0.0 or 6.0.2; another version is refused).
    // gc and nbio_registers are the tokens Gfx11/Nbio were initialized with
    // (nbio_registers also supplies the MMHUB 3.0.0 base); nbio must already be
    // initialized on the same BAR5 io. No register is accessed here.
    ClockGatingStatus initialize(ByteView discovery, memory::GcRegisterBase gc,
                                 NbioRegisters nbio_registers, RegisterIo io, Nbio& nbio,
                                 ClockGatingPolicy policy, ClockGatingRecovery recovery) noexcept;

    // amdgpu_device_ip_late_init: amdgpu_device_set_cg_state(GATE) then
    // amdgpu_device_set_pg_state(GATE) for soc21_common, gmc_v11_0, ih_v6_0,
    // psp, smu, gfx_v11_0 and sdma_v6_0 in IP-block order, then the delayed
    // GFXOFF enable. Requires every hw_init and the soc21 late-init doorbell step
    // to have completed; ih must be initialized and smu running. Returns gated, or
    // smu_refused when PMFW rejected AllowGfxOff after all gating was programmed.
    ClockGatingStatus gate(Ih6& ih, Smu13& smu) noexcept;

    // amdgpu_device_ip_fini_early / amdgpu_device_ip_suspend_phase1:
    // set_pg_state(UNGATE) then set_cg_state(UNGATE), each in reverse IP order.
    // Must run before any hw_fini/suspend of those blocks. Returns ungated.
    ClockGatingStatus ungate(Ih6& ih, Smu13& smu) noexcept;

    // amdgpu_gfx_off_ctrl. false increments the request count, sending
    // DisallowGfxOff when GFXOFF was allowed; true decrements it and, on reaching
    // zero, waits gfx_off_delay_microseconds and sends AllowGfxOff. Without the
    // GFXOFF feature it is a no-op returning the current status.
    ClockGatingStatus gfx_off_control(Smu13& smu, bool enable) noexcept;

    // After a confirmed reset: forget all state so initialize may run again.
    ClockGatingStatus release_after_reset() noexcept;

    ClockGatingReport report() const noexcept { return report_; }

private:
    enum class Phase { uninitialized, ready, gated, faulted };

    bool read(uint32_t base, uint32_t reg, uint32_t& value) noexcept;
    bool write(uint32_t base, uint32_t reg, uint32_t value) noexcept;
    bool update(uint32_t base, uint32_t reg, uint32_t clear, uint32_t set,
                bool always_write) noexcept;
    ClockGatingStatus common_clock_gating(bool gate) noexcept;
    ClockGatingStatus gmc_clock_gating(bool gate) noexcept;
    ClockGatingStatus ih_clock_gating(Ih6& ih, bool gate) noexcept;
    ClockGatingStatus gfx_clock_gating(bool gate) noexcept;
    bool gfx_coarse_grain(bool gate) noexcept;
    bool gfx_override_bits(bool gate) noexcept;
    ClockGatingStatus enter_safe_mode() noexcept;
    bool exit_safe_mode() noexcept;
    ClockGatingStatus lsdma_power_gating(bool gate) noexcept;
    ClockGatingStatus smu_gfx_off(Smu13& smu, bool allow) noexcept;
    ClockGatingStatus fail(ClockGatingStep step, ClockGatingStatus reason) noexcept;

    memory::GcRegisterBase gc_{};
    NbioRegisters nbio_registers_{};
    RegisterIo io_{};
    Nbio* nbio_ = nullptr;
    ClockGatingPolicy policy_{};
    ClockGatingRecovery recovery_{};
    ClockGatingReport report_{};
    Phase phase_ = Phase::uninitialized;
};

} // namespace rx7900::hardware

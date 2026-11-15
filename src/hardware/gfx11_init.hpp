/*
Module: rx7900.hardware.gfx11_init
Purpose: Bring the GC11.0.0 RLC/CP core up after PSP autoload, as gfx_v11_0_hw_init.
Created: 2026-10-21
Architecture: Register protocol over the shared GC discovery token; queue mapping
through MES/KIQ is a separate module. Nbio, Manager and Gmc11 are reused.
*/
#pragma once

#include "nbio.hpp"
#include "gfxhub.hpp"

namespace rx7900::hardware {

// soc21_init_doorbell_index / amdgpu_doorbell.h (AMDGPU_NAVI10_DOORBELL_*).
namespace soc21_doorbell {
constexpr uint32_t kiq = 0x000;
constexpr uint32_t mec_ring0 = 0x003;
constexpr uint32_t mes_ring0 = 0x00b;
constexpr uint32_t mes_ring1 = 0x00c;
constexpr uint32_t userqueue_end = 0x08a;
constexpr uint32_t gfx_ring0 = 0x08b;
constexpr uint32_t gfx_userqueue_end = 0x0ff;
constexpr uint32_t sdma_engine0 = 0x100;
} // namespace soc21_doorbell

enum class Gfx11Status {
    ready,
    completed,
    released,
    stopped,
    unavailable,
    unsupported_ip,
    invalid_argument,
    invalid_memory,
    busy,
    timeout,
    mmio_failure,
    quarantined
};

// gfx_firmware_header_v2_0 ucode_start_addr_lo/hi of the RS64 images.
struct Gfx11StartAddresses final {
    uint32_t pfp_low;
    uint32_t pfp_high;
    uint32_t me_low;
    uint32_t me_high;
    uint32_t mec_low;
    uint32_t mec_high;
};

struct Gfx11Policy final {
    // Linux adev->usec_timeout polls with 1 us delays.
    uint32_t poll_iterations;
};

struct Gfx11Recovery final {
    void* context;
    bool (*confirm_reset_quiescence)(void*) noexcept;
};

struct Gfx11Report final {
    Gfx11Status status = Gfx11Status::unavailable;
    uint32_t cp_status = 0;
    uint32_t bootload_status = 0;
    uint32_t gb_addr_config = 0;
    // get_gb_addr_config returns -EINVAL for zero; Linux only warns and continues.
    bool gb_addr_config_invalid = false;
    uint32_t clear_state_dwords = 0;
    uint64_t clear_state_mc = 0;
    // gfx_v11_0_cp_gfx_enable logs a halt/unhalt timeout and continues.
    bool cp_gfx_enable_timeout = false;
    uint32_t imu_version = 0;
    bool autoload_complete = false;
    bool rs64_configured = false;
    bool rlc_resumed = false;
    bool cp_started = false;
    bool cp_stopped = false;
    bool cp_halt_timeout = false;
    bool memory_retained = false;
};

// Serialized PASSIVE_LEVEL use by the exclusive device owner, with GRBM_GFX_CNTL
// selection owned by this object (Linux srbm_mutex) for the whole sequence. Run
// order matches gfx_v11_0_hw_init: wait_for_autoload, read_gb_addr_config,
// configure_rs64, (Gmc11::enable_gfxhub), golden_registers, constants_init,
// (Nbio::gc_doorbell_init), rlc_resume, cp_start. Host-side RB/CU/TCC topology
// queries in constants_init only fill Linux bookkeeping and are not performed.
class Gfx11Init final {
public:
    Gfx11Init() noexcept = default;
    Gfx11Init(const Gfx11Init&) = delete;
    Gfx11Init& operator=(const Gfx11Init&) = delete;

    Gfx11Status initialize(memory::GcRegisterBase registers, RegisterIo io, Nbio& nbio,
                           VramLocation vram, Gfx11Policy policy, Gfx11Recovery recovery) noexcept;

    // gfx_v11_0_wait_for_rlc_autoload_complete: CP_STAT idle and
    // RLC_RLCS_BOOTLOAD_STATUS.BOOTLOAD_COMPLETE (GC11.0.0 register).
    Gfx11Status wait_for_autoload() noexcept;
    // get_gb_addr_config.
    Gfx11Status read_gb_addr_config() noexcept;
    // gfx_v11_0_config_gfx_rs64 for PSP-loaded RS64 PFP/ME (2 pipes) and MEC (4).
    Gfx11Status configure_rs64(Gfx11StartAddresses addresses) noexcept;
    // gfx_v11_0_init_golden_registers: golden_settings_gc_11_0 (TCP_CNTL).
    Gfx11Status golden_registers() noexcept;
    // gfx_v11_0_constants_init register writes, init_compute_vmid, init_gds_vmid.
    Gfx11Status constants_init() noexcept;
    // gfx_v11_0_rlc_resume (PSP path): build the clear-state buffer from
    // clearstate_gfx11 into a CPU-mapped local-VRAM allocation (retained until
    // reset), program RLC_CSIB_* and enable the RLC save/restore machine.
    Gfx11Status rlc_resume(memory::Manager& manager, memory::Handle clear_state) noexcept;
    // gfx_v11_0_cp_resume up to engine start: GUI idle interrupts off, CP doorbell
    // ranges, RS64 compute enable and GFX unhalt. KIQ/KCQ/KGQ come from MES.
    Gfx11Status cp_start() noexcept;
    // gfx_v11_0_hw_fini tail after the queues are unmapped: cp_enable(false)
    // (ME/PFP halt with the CP_STAT idle poll, then RS64 MEC halt, pipe resets
    // and icache invalidate) and enable_gui_idle_interrupt(false). The clear-state
    // lease stays retained until confirmed reset. Allowed when quarantined.
    Gfx11Status cp_stop() noexcept;

    Gfx11Status release_after_reset() noexcept;
    Gfx11Report report() const noexcept { return report_; }
    // Required clear-state buffer size in bytes (gfx_v11_0_get_csb_size * 4).
    static uint32_t clear_state_bytes() noexcept;

private:
    bool read(size_t segment, uint32_t reg, uint32_t& value) noexcept;
    bool write(size_t segment, uint32_t reg, uint32_t value) noexcept;
    bool modify(size_t segment, uint32_t reg, uint32_t clear, uint32_t set) noexcept;
    bool grbm_select(uint32_t me, uint32_t pipe, uint32_t queue, uint32_t vmid) noexcept;
    Gfx11Status fail(Gfx11Status status) noexcept;

    memory::GcRegisterBase registers_{};
    RegisterIo io_{};
    Nbio* nbio_ = nullptr;
    VramLocation vram_{};
    Gfx11Policy policy_{};
    Gfx11Recovery recovery_{};
    Gfx11Report report_{};
    memory::Manager* manager_ = nullptr;
    memory::Handle clear_state_{};
    bool initialized_ = false;
    bool faulted_ = false;
};

} // namespace rx7900::hardware

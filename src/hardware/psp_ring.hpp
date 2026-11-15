/*
Module: rx7900.hardware.psp_ring
Purpose: Execute PSP13.0.0 kernel-mode ring commands after the SOS has started.
Created: 2026-10-21
Architecture: Reuses PspRegisters, Nbio HDP flush and memory::Manager leases;
uncertain submissions retain every PSP-visible allocation until reset proof.
*/
#pragma once

#include "psp_boot.hpp"
#include "nbio.hpp"

namespace rx7900::hardware {

constexpr uint32_t psp_ring_bytes = 0x1000;
constexpr uint32_t psp_command_bytes = 0x1000;
constexpr uint32_t psp_fence_bytes = 0x1000;
// Ring, command buffer and fence buffer share one 4 KiB-aligned control allocation.
constexpr uint32_t psp_control_bytes = psp_ring_bytes + psp_command_bytes + psp_fence_bytes;
// Linux fw_pri_buf: 1 MiB, 1 MiB aligned. Every payload is copied here first.
constexpr uint32_t psp_firmware_buffer_bytes = 0x100000;
constexpr uint32_t psp_tmr_default_bytes = 0x400000;
constexpr uint32_t psp_tmr_alignment = 0x100000;
// psp_ta_init_shared_buf: TA shared buffers are PAGE_SIZE aligned.
constexpr uint32_t psp_ta_shared_alignment = 0x1000;
// psp_gfx_boot_config BOOT_CONFIG_GECC: the only dynamic boot configuration bit.
constexpr uint32_t psp_boot_config_gecc = 0x1;

enum class PspRingStatus {
    ready,
    ring_created,
    completed,
    stopped,
    released,
    unavailable,
    unsupported_ip,
    invalid_memory,
    invalid_payload,
    busy,
    timeout,
    mmio_failure,
    unexpected_response,
    command_failed,
    quarantined
};

struct PspRingPolicy final {
    // Linux psp_wait_for: adev->usec_timeout (100000) polls with 1 us delays.
    uint32_t register_poll_iterations;
    // Linux psp_cmd_submit_buf: psp_timeout (20000) polls sleeping 10-100 us.
    uint32_t fence_poll_iterations;
    uint32_t fence_poll_microseconds;
};

struct PspRingRecovery final {
    void* context;
    // True only after the native reset/power owner proves PSP can no longer access
    // ring, command, fence, firmware or TMR memory. Never inferred from a timeout.
    bool (*confirm_reset_quiescence)(void*) noexcept;
};

struct PspRingReport final {
    PspRingStatus status = PspRingStatus::unavailable;
    uint32_t last_command = 0;
    uint32_t last_fw_type = 0;
    uint32_t last_fence = 0;
    uint32_t response_status = 0;
    uint32_t response_session = 0;
    uint32_t response_tmr_bytes = 0;
    uint64_t response_fw_address = 0;
    // resp.uresp.boot_cfg.boot_cfg (response offset 64) of the last command.
    uint32_t response_boot_config = 0;
    uint32_t raw_mailbox = 0;
    uint32_t write_pointer = 0;
    uint32_t completed_commands = 0;
    uint64_t tmr_mc_address = 0;
    uint32_t tmr_bytes = 0;
    bool ring_live = false;
    bool memory_retained = false;
    bool tmr_retained = false;
};

// Serialized PASSIVE_LEVEL use by the exclusive device owner after PspBoot has
// observed a running SOS. Nbio, Manager, register mapping and all allocations
// outlive this object. Control and firmware allocations are CPU-visible local
// VRAM inside vram (MC addresses), mapped uncached so CPU stores are ordered
// before the HDP flush register write; their Manager GPU address is the MC
// address. No GTT/GART memory and no DMA bus address substitutes for them.
//
// Linux only warns for a nonzero response status on bare metal. This protocol
// fails closed with command_failed and reports the status; the ring stays usable.
// A fence timeout or uncertain MMIO quarantines the ring and every lease.
class PspRing final {
public:
    PspRing() noexcept = default;
    PspRing(const PspRing&) = delete;
    PspRing& operator=(const PspRing&) = delete;

    PspRingStatus initialize(PspRegisters registers, RegisterIo io, Nbio& nbio,
                             VramLocation vram, PspRingPolicy policy,
                             PspRingRecovery recovery) noexcept;

    // psp_ring_init + psp_v13_0_ring_create (non-SR-IOV KM ring). Retains one
    // lease on each allocation; zeroes the ring and fence before publication.
    PspRingStatus create(memory::Manager& manager, memory::Handle control,
                         memory::Handle firmware) noexcept;

    // GFX_CMD_ID_LOAD_TOC; tmr_bytes receives the PSP-computed TMR size.
    PspRingStatus load_toc(ByteView toc, uint32_t& tmr_bytes) noexcept;
    // GFX_CMD_ID_SETUP_TMR with virtual and physical addresses. tmr must be a
    // local-VRAM allocation >= bytes, 1 MiB aligned; it is retained until reset.
    PspRingStatus setup_tmr(memory::Handle tmr, uint32_t bytes) noexcept;
    // GFX_CMD_ID_LOAD_IP_FW for one catalog payload of the given GFX_FW_TYPE.
    PspRingStatus load_ip_firmware(ByteView payload, uint32_t psp_type) noexcept;
    // GFX_CMD_ID_AUTOLOAD_RLC after all GFX firmware including RLC_G.
    PspRingStatus autoload_rlc() noexcept;
    // GFX_CMD_ID_LOAD_ASD without a shared buffer (PSP_ASD_SHARED_MEM_SIZE 0).
    PspRingStatus load_asd(ByteView asd, uint32_t& session) noexcept;
    // psp_ta_load with ta_load_type GFX_CMD_ID_LOAD_TA: ta is staged in the 1 MiB
    // firmware buffer exactly as psp_copy_fw; the command carries that buffer's
    // MC address, ta.size(), and shared_mc/shared_bytes as cmd_buf_phy_addr/len.
    // The caller owns and retains the shared allocation for the session lifetime;
    // it must be 4 KiB aligned, nonempty and inside local VRAM. session receives
    // resp.session_id only on completed. command_failed reports resp.status.
    PspRingStatus load_ta(ByteView ta, uint64_t shared_mc, uint32_t shared_bytes,
                          uint32_t& session) noexcept;
    // psp_boot_config_get: GFX_CMD_ID_BOOT_CFG sub_cmd BOOTCFG_CMD_GET. On
    // completed or command_failed raw_boot_config receives resp.uresp.boot_cfg;
    // Linux uses that value whenever no timeout occurred (see psp_ta.hpp).
    PspRingStatus boot_config_get(uint32_t& raw_boot_config) noexcept;
    // psp_boot_config_set: sub_cmd BOOTCFG_CMD_SET with boot_config and
    // boot_config_valid both boot_config. This changes the card's persistent
    // dynamic boot configuration (GECC) for the next boot.
    PspRingStatus boot_config_set(uint32_t boot_config) noexcept;
    // SOS RL part, as LOAD_IP_FW GFX_FW_TYPE_REG_LIST (psp_rl_load).
    PspRingStatus load_register_list(ByteView register_list) noexcept;
    // GFX_CMD_ID_DESTROY_TMR. The TMR lease is still retained until reset.
    PspRingStatus destroy_tmr() noexcept;
    // psp_ta_unload: GFX_CMD_ID_UNLOAD_TA for a session load_asd or load_ta
    // returned (psp_asd/ras/hdcp/dtm_terminate).
    PspRingStatus unload_ta(uint32_t session) noexcept;
    // psp_v13_0_ring_stop: GFX_CTRL_CMD_ID_DESTROY_RINGS. On confirmed completion
    // the control and firmware leases are released; the TMR lease is not.
    PspRingStatus stop() noexcept;
    // After confirmed reset quiescence: relinquish all retained leases.
    PspRingStatus release_after_reset() noexcept;

    PspRingReport report() const noexcept { return report_; }

private:
    struct Command final {
        uint32_t id;
        uint32_t words[6];
        uint32_t word_count;
        uint32_t fw_type;
    };

    enum class Phase { uninitialized, ready, live, stopped, faulted };

    bool read(uint32_t reg, uint32_t& value) noexcept;
    bool write(uint32_t reg, uint32_t value) noexcept;
    PspRingStatus wait_mailbox(uint32_t value, uint32_t mask) noexcept;
    PspRingStatus quarantine(PspRingStatus reason) noexcept;
    PspRingStatus submit(const Command& command) noexcept;
    bool stage(ByteView payload) noexcept;
    PspRingStatus load_trusted_app(uint32_t load_type, ByteView binary, uint64_t shared_mc,
                                   uint32_t shared_bytes, uint32_t& session) noexcept;
    bool retain(memory::Handle handle, uint32_t minimum_bytes, uint64_t alignment,
                bool cpu_visible, memory::AllocationInfo& output) noexcept;
    void release_leases(bool include_tmr) noexcept;
    uint64_t firmware_address() const noexcept { return firmware_.gpu.virtual_address; }

    PspRegisters registers_{};
    RegisterIo io_{};
    Nbio* nbio_ = nullptr;
    VramLocation vram_{};
    PspRingPolicy policy_{};
    PspRingRecovery recovery_{};
    PspRingReport report_{};
    Phase phase_ = Phase::uninitialized;
    memory::Manager* manager_ = nullptr;
    memory::Handle control_handle_{};
    memory::Handle firmware_handle_{};
    memory::Handle tmr_handle_{};
    memory::AllocationInfo control_{};
    memory::AllocationInfo firmware_{};
    uint32_t fence_value_ = 0;
};

} // namespace rx7900::hardware

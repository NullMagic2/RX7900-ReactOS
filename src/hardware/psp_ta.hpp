/*
Module: rx7900.hardware.psp_ta
Purpose: Load and unload the PSP13.0.0 RAS, HDCP and DTM trusted applications.
Created: 2026-10-22
Architecture: Runs after PspRing::load_register_list (psp_load_fw order) on the
live KM ring; decodes TA binaries through ucode parse_ta_descriptors, leases
caller-owned local-VRAM shared buffers like the other PSP buffers and leaves
the persistent BOOT_CFG set to an explicit integrator policy.
*/
#pragma once

#include "atom_firmware.hpp"
#include "psp_ring.hpp"
#include "ucode.hpp"

namespace rx7900::hardware {

// PSP_RAS/HDCP/DTM_SHARED_MEM_SIZE (amdgpu_psp.h): 16 KiB, PAGE_SIZE aligned.
constexpr uint32_t psp_ta_shared_bytes = 0x4000;
// ATOM firmwareinfo capability bits read by the RAS path (atomfirmware.h).
constexpr uint32_t atom_capability_sram_ecc = 0x00000200;
constexpr uint32_t atom_capability_dynamic_boot_config = 0x00020000;

// amdgpu_atomfirmware_mem_ecc_supported: umc_info (master data table index 24)
// format 3 content 1/2 umc_config bit DEFAULT_MEM_ECC_ENABLE (0x2), content 3 that
// bit or umc_config1 ENABLE_ECC_CAPABLE (0x10000), format 4 content 0 that
// umc_config1 bit. An absent table or another revision is "not supported"
// (success, false), as in Linux; a field outside the declared table fails closed
// with invalid_rom. Output unchanged on failure.
AtomFirmwareStatus atom_mem_ecc_supported(ByteView rom, bool& supported) noexcept;

// Inputs of amdgpu_ras_check_supported for bare-metal discrete MP0 13.0.0
// (amdgpu_ras_asic_supported true, no PSP RAS capability register, not
// connected to a CPU, not an APP APU, no XGMI hive, VCN 4.0.0).
struct PspRasCapabilityInputs final {
    // AtomFirmwareInfo::capability (adev->mode_info.firmware_flags).
    uint32_t atom_capability = 0;
    // atom_mem_ecc_supported.
    bool mem_ecc_supported = false;
    // amdgpu_ras_get_quirks: ATOM vbios_pn contains D16406 or D36002.
    bool vbios_gfx_quirk = false;
    // df_v4_3 and umc_v8_10 query_ras_poison_mode both true (register reads the
    // native owner performs). Linux consults it only while a RAS context exists.
    bool df_umc_poison_mode = false;
    // Module parameters amdgpu_ras_enable / amdgpu_ras_mask; defaults -1 / ~0.
    int32_t ras_enable = -1;
    uint32_t ras_mask = 0xffffffffu;
};

struct PspRasSupport final {
    uint32_t hw_enabled = 0;   // adev->ras_hw_enabled
    uint32_t enabled = 0;      // adev->ras_enabled
    bool context = false;      // amdgpu_ras_init keeps its context (enabled != 0)
    bool umc_supported = false; // amdgpu_ras_is_supported(adev, AMDGPU_RAS_BLOCK__UMC)
    bool poison_mode = false;  // amdgpu_ras_is_poison_mode_supported
};

PspRasSupport linux_ras_support(const PspRasCapabilityInputs& inputs) noexcept;

// psp_ras_initialize boot-configuration branch. boot_cfg is Linux's value: 0xFF
// before or after a failed BOOT_CFG get, else the GECC bit (0 or 1).
enum class PspBootConfigAction : uint8_t {
    not_supported,  // firmwareinfo lacks DYNAMIC_BOOT_CFG_ENABLE: no BOOT_CFG at all
    keep_disabled,  // "GECC is disabled"
    keep_enabled,   // "GECC is enabled"
    set_disable,    // psp_boot_config_set(adev, 0)
    set_enable      // psp_boot_config_set(adev, BOOT_CONFIG_GECC)
};

PspBootConfigAction linux_boot_config_action(bool dynamic_boot_config, uint32_t boot_cfg,
                                             bool umc_supported) noexcept;

struct PspBootConfigReport final {
    bool dynamic_supported = false;
    bool queried = false;
    PspRingStatus get_status = PspRingStatus::unavailable;
    uint32_t get_response_status = 0;
    uint32_t raw_boot_config = 0;
    uint32_t linux_boot_cfg = 0xff;
    PspBootConfigAction action = PspBootConfigAction::not_supported;
    uint32_t set_value = 0;
    // A set Linux would send but PspTaPolicy did not allow.
    bool set_withheld = false;
    bool set_sent = false;
    PspRingStatus set_status = PspRingStatus::unavailable;
    uint32_t set_response_status = 0;
};

// Caller-owned shared buffers: each a Manager allocation of at least
// psp_ta_shared_bytes, 4 KiB aligned, CPU-mapped local VRAM (Linux places them
// VRAM|GTT and normally gets VRAM). A lease is retained while PSP may use one.
struct PspTaBuffers final {
    memory::Handle ras{};
    memory::Handle hdcp{};
    memory::Handle dtm{};
};

struct PspTaInputs final {
    // psp_13_0_0_ta.bin; empty means psp->ta_fw is NULL and nothing is loaded.
    ByteView ta_file{nullptr, 0};
    // amdgpu_device_has_display_hardware: DCE_HWIP discovered and DMU not harvested.
    bool has_display_hardware = false;
    PspRasCapabilityInputs ras{};
    // adev->gfx.xcc_mask: GC_HWID discovery instances minus harvested (1 on Navi31).
    uint16_t xcc_mask = 0;
    // adev->gmc.m_half_use: discovery MALL_INFO v1 m_half_use, 0 without that table.
    uint32_t m_half_use = 0;
};

struct PspTaPolicy final {
    // Linux sends BOOT_CFG set when its decision requires it; that persistently
    // changes the next boot's GECC state. Default: report the decision only.
    bool send_boot_config_set = false;
};

enum class PspTaState : uint8_t {
    not_attempted,
    skipped_no_display,  // HDCP/DTM: amdgpu_device_has_display_hardware false
    not_available,       // no descriptor or zero size ("optional ... ucode is not available")
    invalid_memory,      // shared buffer could not be leased; PSP not contacted
    load_failed,         // LOAD_TA refused or reported a nonzero response status
    ras_status_failed,   // LOAD_TA completed, ta_ras_shared_memory.ras_status nonzero
    loaded,              // Linux context->initialized
    unloaded,
    unload_failed,
    quarantined          // ring state uncertain; lease kept until reset proof
};

struct PspTaResult final {
    PspTaState state = PspTaState::not_attempted;
    PspRingStatus ring = PspRingStatus::unavailable;
    uint32_t fw_version = 0;
    uint32_t bytes = 0;
    uint32_t session = 0;
    // context->resp_status of the last load or unload.
    uint32_t response_status = 0;
    // RAS only: ta_ras_shared_memory.ras_status after LOAD_TA.
    uint32_t ras_status = 0;
    uint64_t shared_mc = 0;
    bool lease_retained = false;
};

struct PspTaReport final {
    PspRasSupport ras_support{};
    PspBootConfigReport boot_config{};
    // ta_ras_shared_memory dwords at 16 (poison_mode_en, dgpu_mode, xcc_mask)
    // and 20 (channel_dis_num) as written before the RAS LOAD_TA.
    uint32_t ras_init_flags_low = 0;
    uint32_t ras_init_flags_high = 0;
    PspTaResult ras{};
    PspTaResult hdcp{};
    PspTaResult dtm{};
    // Descriptors this bring-up does not load (Linux bare metal never loads
    // XGMI here; RAP and secure display are absent from psp_13_0_0_ta.bin).
    bool xgmi_present = false;
    bool rap_present = false;
    bool securedisplay_present = false;
    bool ring_quarantined = false;
};

// refused: PSP answered an UNLOAD_TA with an error status (suspend stops there).
enum class PspTaStatus { completed, invalid_input, busy, quarantined, released, refused };

// Serialized PASSIVE_LEVEL use by the exclusive device owner; the ring, Manager
// and every buffer outlive this object. TA failures are non-fatal as in Linux
// psp_load_fw (dev_err and continue): initialize returns completed and reports
// each TA. Only an uncertain ring (timeout, MMIO failure, quarantine) stops the
// sequence with quarantined; every published lease then stays retained.
class PspTrustedApps final {
public:
    PspTrustedApps() noexcept = default;
    PspTrustedApps(const PspTrustedApps&) = delete;
    PspTrustedApps& operator=(const PspTrustedApps&) = delete;

    // psp_load_fw after psp_rl_load: psp_ras_initialize, psp_hdcp_initialize,
    // psp_dtm_initialize (RAP/secure display are absent). A malformed TA file
    // is invalid_input before any PSP command.
    PspTaStatus initialize(PspRing& ring, memory::Manager& manager, VramLocation vram,
                           const PspTaBuffers& buffers, const PspTaInputs& inputs,
                           PspTaPolicy policy, PspRingRecovery recovery) noexcept;
    // psp_hw_fini before psp_asd_terminate: psp_ras_terminate, psp_dtm_terminate,
    // psp_hdcp_terminate (UNLOAD_TA for each loaded session). A confirmed unload
    // releases that shared-buffer lease; a failed one keeps it until reset.
    PspTaStatus terminate() noexcept;
    // psp_suspend: psp_ras_terminate, psp_hdcp_terminate, psp_dtm_terminate (RAP
    // and secure display absent) in that order. Linux stops psp_suspend at the
    // first failed terminate; so does this, returning refused for a PSP error
    // status (that buffer stays leased) or quarantined for an uncertain ring.
    // After a completed suspend, initialize loads every TA again (psp_resume).
    PspTaStatus suspend() noexcept;
    // After confirmed reset quiescence: relinquish every retained lease.
    PspTaStatus release_after_reset() noexcept;

    PspTaReport report() const noexcept { return report_; }
    // The amdgpu_ras_check_supported result of the last load (no report copy).
    const PspRasSupport& ras_support() const noexcept { return report_.ras_support; }

private:
    enum class Phase { idle, initialized, terminated, faulted };

    void load(PspTaResult& result, memory::Handle handle, ByteView file, TaBinary binary,
              bool ras, const PspTaInputs& inputs) noexcept;
    void unload(PspTaResult& result, memory::Handle handle) noexcept;
    void boot_config(const PspTaInputs& inputs, PspTaPolicy policy) noexcept;
    void release(PspTaResult& result, memory::Handle handle) noexcept;
    bool any_retained() const noexcept;

    PspRing* ring_ = nullptr;
    memory::Manager* manager_ = nullptr;
    VramLocation vram_{};
    PspTaBuffers buffers_{};
    PspRingRecovery recovery_{};
    PspTaReport report_{};
    Phase phase_ = Phase::idle;
};

} // namespace rx7900::hardware

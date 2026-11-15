/*
Module: rx7900.hardware.psp_ta
Purpose: Decide and execute Linux psp_ras/hdcp/dtm_initialize and _terminate.
Created: 2026-10-22
Architecture: Commands go through PspRing (LOAD_TA, BOOT_CFG, UNLOAD_TA); shared
buffers are leased with retain_local_vram and written through CommandBuffer.
Copyright 2016, 2019 Advanced Micro Devices, Inc. Adapted Linux v6.12
amdgpu_psp.c/.h (psp_load_fw, psp_hw_fini, psp_ras/hdcp/dtm_initialize and
_terminate, psp_boot_config_get/set, psp_ta_init_shared_buf), ta_ras_if.h,
amdgpu_ras.c/.h (amdgpu_ras_init, amdgpu_ras_check_supported,
amdgpu_ras_asic_supported, amdgpu_ras_query_ras_capablity_from_vbios,
amdgpu_ras_query_poison_mode, amdgpu_ras_is_supported), psp_v13_0.c
(get_ras_capability), amdgpu_atomfirmware.c (mem_ecc_supported,
sram_ecc_supported, dynamic_boot_config_supported) and atomfirmware.h.
Notice and exact provenance: licenses/AMDGPU-FIRMWARE.txt.
*/
#include "psp_ta.hpp"

namespace rx7900::hardware {
namespace {

// atom_master_list_of_data_tables_v2_1 umc_info and atom_umc_info_v3_x/v4_0
// fields, measured by compiling the pinned atomfirmware.h.
constexpr size_t umc_info_index = 24;
constexpr size_t umc_config_offset = 27;
constexpr size_t umc_config1_offset = 52;
constexpr uint8_t umc_config_default_mem_ecc = 0x02;
constexpr uint32_t umc_config1_ecc_capable = 0x00010000;

// enum amdgpu_ras_block bits; AMDGPU_RAS_BLOCK_COUNT is 19.
constexpr uint32_t ras_block_umc = 1u << 0;
constexpr uint32_t ras_block_gfx = 1u << 2;
constexpr uint32_t ras_block_xgmi_wafl = 1u << 7;
constexpr uint32_t ras_block_df = 1u << 8;
constexpr uint32_t ras_block_vcn = 1u << 15;
constexpr uint32_t ras_block_jpeg = 1u << 16;
constexpr uint32_t ras_block_mask = (1u << 19) - 1;

// ta_ras_shared_memory (ta_ras_if.h, 2064 bytes): ras_status at 8 and
// ras_in_message.init_flags at 16 (poison_mode_en u8, dgpu_mode u8, xcc_mask
// u16, channel_dis_num u8 at 20), measured from the pinned header.
constexpr size_t ras_status_offset = 8;
constexpr size_t ras_init_flags_offset = 16;
constexpr size_t ras_channel_disable_offset = 20;

bool uncertain(PspRingStatus status) noexcept {
    return status == PspRingStatus::timeout || status == PspRingStatus::mmio_failure ||
           status == PspRingStatus::unexpected_response || status == PspRingStatus::quarantined;
}

bool submitted(PspRingStatus status) noexcept {
    return status == PspRingStatus::completed || status == PspRingStatus::command_failed;
}

uint32_t population(uint32_t value) noexcept {
    uint32_t count = 0;

    for (; value != 0; value &= value - 1) {
        ++count;
    }

    return count;
}

} // namespace

AtomFirmwareStatus atom_mem_ecc_supported(ByteView rom, bool& supported) noexcept {
    AtomTable table{};
    const AtomTableStatus lookup = atom_data_table(rom, umc_info_index, table);

    if (lookup == AtomTableStatus::absent) {
        supported = false;
        return AtomFirmwareStatus::success;
    }

    if (lookup != AtomTableStatus::success) {
        return lookup == AtomTableStatus::unsupported_revision ?
                   AtomFirmwareStatus::unsupported_revision : AtomFirmwareStatus::invalid_rom;
    }

    ByteView info(nullptr, 0);

    if (!rom.subview(table.base, table.bytes, info)) {
        return AtomFirmwareStatus::invalid_rom;
    }

    const bool config = table.format == 3 && table.revision >= 1 && table.revision <= 3;
    const bool config1 = (table.format == 3 && table.revision == 3) ||
                         (table.format == 4 && table.revision == 0);
    uint8_t umc_config = 0;
    uint32_t umc_config1 = 0;

    if ((config && !info.read_u8(umc_config_offset, umc_config)) ||
        (config1 && !info.read_u32(umc_config1_offset, umc_config1))) {
        return AtomFirmwareStatus::invalid_rom;
    }

    // Other format/content revisions: Linux returns false ("unsupported crev").
    supported = (config && (umc_config & umc_config_default_mem_ecc) != 0) ||
                (config1 && (umc_config1 & umc_config1_ecc_capable) != 0);
    return AtomFirmwareStatus::success;
}

PspRasSupport linux_ras_support(const PspRasCapabilityInputs& inputs) noexcept {
    PspRasSupport support{};
    uint32_t hw = 0;

    // amdgpu_ras_asic_supported is true for MP0 13.0.0 and psp_v13_0
    // get_ras_capability is false outside 13.0.6/13.0.14, so the VBIOS decides.
    if (inputs.mem_ecc_supported) {
        hw |= ras_block_umc | ras_block_df;
    }

    if ((inputs.atom_capability & atom_capability_sram_ecc) != 0) {
        hw |= ~(ras_block_umc | ras_block_df);
        // VCN 4.0.0 keeps VCN/JPEG; no XGMI physical nodes clears XGMI_WAFL.
        hw |= ras_block_vcn | ras_block_jpeg;
        hw &= ~ras_block_xgmi_wafl;
    }

    if (inputs.vbios_gfx_quirk) {
        hw |= ras_block_gfx;
    }

    support.hw_enabled = hw & ras_block_mask;
    support.enabled = inputs.ras_enable == 0 ? 0 : support.hw_enabled & inputs.ras_mask;
    // amdgpu_ras_init frees its context when ras_enabled is zero; poison and
    // is_supported then read a NULL context. The GFX/SDMA/VCN/JPEG poison
    // exception of amdgpu_ras_is_supported does not apply to UMC.
    support.context = support.enabled != 0;
    support.poison_mode = support.context && inputs.df_umc_poison_mode;
    support.umc_supported = support.context && (support.enabled & ras_block_umc) != 0;
    return support;
}

PspBootConfigAction linux_boot_config_action(bool dynamic_boot_config, uint32_t boot_cfg,
                                             bool umc_supported) noexcept {
    if (!dynamic_boot_config) {
        return PspBootConfigAction::not_supported;
    }

    if (!umc_supported) {
        return boot_cfg == 0 ? PspBootConfigAction::keep_disabled : PspBootConfigAction::set_disable;
    }

    return boot_cfg == 1 ? PspBootConfigAction::keep_enabled : PspBootConfigAction::set_enable;
}

bool PspTrustedApps::any_retained() const noexcept {
    return report_.ras.lease_retained || report_.hdcp.lease_retained ||
           report_.dtm.lease_retained;
}

void PspTrustedApps::release(PspTaResult& result, memory::Handle handle) noexcept {
    if (result.lease_retained && manager_ != nullptr) {
        manager_->unretain(handle);
        result.lease_retained = false;
    }
}

void PspTrustedApps::boot_config(const PspTaInputs& inputs, PspTaPolicy policy) noexcept {
    PspBootConfigReport& boot = report_.boot_config;
    boot.dynamic_supported =
        (inputs.ras.atom_capability & atom_capability_dynamic_boot_config) != 0;

    if (!boot.dynamic_supported) {
        boot.action = PspBootConfigAction::not_supported;
        return;
    }

    boot.queried = true;
    boot.get_status = ring_->boot_config_get(boot.raw_boot_config);

    if (uncertain(boot.get_status)) {
        report_.ring_quarantined = true;
        return;
    }

    // psp_cmd_submit_buf returns 0 for a nonzero response status on bare metal,
    // so Linux uses the response bit then too; boot_cfg stays 0xFF otherwise.
    if (submitted(boot.get_status)) {
        boot.get_response_status = ring_->report().response_status;
        boot.linux_boot_cfg = (boot.raw_boot_config & psp_boot_config_gecc) != 0 ? 1 : 0;
    }

    boot.action = linux_boot_config_action(true, boot.linux_boot_cfg,
                                           report_.ras_support.umc_supported);

    if (boot.action != PspBootConfigAction::set_disable &&
        boot.action != PspBootConfigAction::set_enable) {
        return;
    }

    boot.set_value = boot.action == PspBootConfigAction::set_enable ? psp_boot_config_gecc : 0;

    if (!policy.send_boot_config_set) {
        boot.set_withheld = true;
        return;
    }

    boot.set_sent = true;
    boot.set_status = ring_->boot_config_set(boot.set_value);

    if (submitted(boot.set_status)) {
        boot.set_response_status = ring_->report().response_status;
    }

    if (uncertain(boot.set_status)) {
        report_.ring_quarantined = true;
    }
}

void PspTrustedApps::load(PspTaResult& result, memory::Handle handle, ByteView file,
                          TaBinary binary, bool ras, const PspTaInputs& inputs) noexcept {
    result.fw_version = binary.fw_version;
    result.bytes = binary.bytes;
    ByteView payload(nullptr, 0);

    if (!binary.present || binary.bytes == 0 ||
        !file.subview(binary.offset, binary.bytes, payload)) {
        result.state = PspTaState::not_available;
        return;
    }

    memory::AllocationInfo shared{};

    if (!retain_local_vram(*manager_, handle, vram_, psp_ta_shared_bytes,
                           psp_ta_shared_alignment, true, shared)) {
        result.state = PspTaState::invalid_memory;
        return;
    }

    result.lease_retained = true;
    result.shared_mc = shared.gpu.virtual_address;
    const sdma::CommandBuffer bytes(shared.backing.cpu_bytes, psp_ta_shared_bytes);

    // Linux clears only the RAS structure and leaves HDCP/DTM buffers as
    // allocated; every byte PSP may read is defined here instead.
    for (uint32_t offset = 0; offset < psp_ta_shared_bytes; offset += sizeof(uint32_t)) {
        bytes.write_u32(offset, 0);
    }

    if (ras) {
        // psp_ras_initialize init_flags; dgpu_mode because a discrete Navi31 is
        // neither XGMI-connected to a CPU nor an APP APU.
        const uint32_t poison = report_.ras_support.poison_mode ? 1 : 0;
        const uint32_t dgpu = 1;
        report_.ras_init_flags_low = poison | (dgpu << 8) |
                                     (static_cast<uint32_t>(inputs.xcc_mask) << 16);
        report_.ras_init_flags_high = static_cast<uint8_t>(population(inputs.m_half_use) * 2);
        bytes.write_u32(ras_init_flags_offset, report_.ras_init_flags_low);
        bytes.write_u32(ras_channel_disable_offset, report_.ras_init_flags_high);
    }

    // PspRing::submit flushes HDP after these stores and before the doorbell.
    uint32_t session = 0;
    result.ring = ring_->load_ta(payload, result.shared_mc, psp_ta_shared_bytes, session);

    if (submitted(result.ring)) {
        result.response_status = ring_->report().response_status;
    }

    if (uncertain(result.ring)) {
        result.state = PspTaState::quarantined;
        report_.ring_quarantined = true;
        return;
    }

    if (result.ring == PspRingStatus::command_failed) {
        // Linux would mark the TA initialized with the returned session; fail
        // closed: never invoked or unloaded, the lease kept until reset.
        result.state = PspTaState::load_failed;
        return;
    }

    if (result.ring != PspRingStatus::completed) {
        // Refused before submission: PSP never received the shared address.
        release(result, handle);
        result.state = PspTaState::load_failed;
        return;
    }

    result.session = session;
    result.state = PspTaState::loaded;

    if (ras) {
        const ByteView view(shared.backing.cpu_bytes, psp_ta_shared_bytes);
        view.read_u32(ras_status_offset, result.ras_status);

        // Linux: initialized stays false and the session is never unloaded.
        if (result.ras_status != 0) {
            result.state = PspTaState::ras_status_failed;
        }
    }
}

void PspTrustedApps::unload(PspTaResult& result, memory::Handle handle) noexcept {
    if (result.state != PspTaState::loaded) {
        return;
    }

    result.ring = ring_->unload_ta(result.session);

    if (submitted(result.ring)) {
        result.response_status = ring_->report().response_status;
    }

    if (uncertain(result.ring)) {
        result.state = PspTaState::quarantined;
        report_.ring_quarantined = true;
        return;
    }

    if (result.ring != PspRingStatus::completed) {
        // Linux clears initialized regardless; PSP may still hold the buffer.
        result.state = PspTaState::unload_failed;
        return;
    }

    result.state = PspTaState::unloaded;
    release(result, handle);
}

PspTaStatus PspTrustedApps::initialize(PspRing& ring, memory::Manager& manager, VramLocation vram,
                                       const PspTaBuffers& buffers, const PspTaInputs& inputs,
                                       PspTaPolicy policy, PspRingRecovery recovery) noexcept {
    if ((phase_ != Phase::idle && phase_ != Phase::terminated) || any_retained()) {
        return phase_ == Phase::faulted ? PspTaStatus::quarantined : PspTaStatus::busy;
    }

    TaDescriptors descriptors{};

    if (inputs.ta_file.size() != 0 &&
        parse_ta_descriptors(inputs.ta_file, descriptors) != UcodeStatus::success) {
        return PspTaStatus::invalid_input;
    }

    if (inputs.ta_file.size() != 0 && !ring.report().ring_live) {
        return PspTaStatus::busy;
    }

    report_ = {};
    ring_ = &ring;
    manager_ = &manager;
    vram_ = vram;
    buffers_ = buffers;
    recovery_ = recovery;
    report_.ras_support = linux_ras_support(inputs.ras);
    phase_ = Phase::initialized;

    // psp_load_fw runs the TA initializers only when psp->ta_fw exists.
    if (inputs.ta_file.size() == 0) {
        return PspTaStatus::completed;
    }

    report_.xgmi_present = descriptors.get(ta_fw_type::xgmi).present;
    report_.rap_present = descriptors.get(ta_fw_type::rap).present;
    report_.securedisplay_present = descriptors.get(ta_fw_type::securedisplay).present;
    const TaBinary ras = descriptors.get(ta_fw_type::ras);

    if (!ras.present || ras.bytes == 0) {
        report_.ras.state = PspTaState::not_available;
    } else {
        boot_config(inputs, policy);

        if (!report_.ring_quarantined) {
            load(report_.ras, buffers.ras, inputs.ta_file, ras, true, inputs);
        }
    }

    PspTaResult* const results[]{&report_.hdcp, &report_.dtm};
    const memory::Handle handles[]{buffers.hdcp, buffers.dtm};
    const uint32_t types[]{ta_fw_type::hdcp, ta_fw_type::dtm};

    for (size_t index = 0; index < 2 && !report_.ring_quarantined; ++index) {
        if (!inputs.has_display_hardware) {
            results[index]->state = PspTaState::skipped_no_display;
            continue;
        }

        load(*results[index], handles[index], inputs.ta_file, descriptors.get(types[index]),
             false, inputs);
    }

    if (report_.ring_quarantined) {
        phase_ = Phase::faulted;
        return PspTaStatus::quarantined;
    }

    return PspTaStatus::completed;
}

PspTaStatus PspTrustedApps::terminate() noexcept {
    if (phase_ != Phase::initialized) {
        return phase_ == Phase::faulted ? PspTaStatus::quarantined : PspTaStatus::busy;
    }

    // psp_hw_fini: ras, (securedisplay, rap absent), dtm, hdcp; ASD follows.
    unload(report_.ras, buffers_.ras);

    if (!report_.ring_quarantined) {
        unload(report_.dtm, buffers_.dtm);
    }

    if (!report_.ring_quarantined) {
        unload(report_.hdcp, buffers_.hdcp);
    }

    if (report_.ring_quarantined) {
        phase_ = Phase::faulted;
        return PspTaStatus::quarantined;
    }

    phase_ = Phase::terminated;
    return PspTaStatus::completed;
}

PspTaStatus PspTrustedApps::suspend() noexcept {
    if (phase_ != Phase::initialized) {
        return phase_ == Phase::faulted ? PspTaStatus::quarantined : PspTaStatus::busy;
    }

    // psp_suspend: ras, hdcp, dtm, each "goto out" on failure.
    PspTaResult* const order[]{&report_.ras, &report_.hdcp, &report_.dtm};
    const memory::Handle handles[]{buffers_.ras, buffers_.hdcp, buffers_.dtm};
    bool refused = false;

    for (size_t index = 0; index < 3 && !refused && !report_.ring_quarantined; ++index) {
        unload(*order[index], handles[index]);
        refused = order[index]->state == PspTaState::unload_failed;
    }

    if (report_.ring_quarantined) {
        phase_ = Phase::faulted;
        return PspTaStatus::quarantined;
    }

    phase_ = Phase::terminated;
    return refused ? PspTaStatus::refused : PspTaStatus::completed;
}

PspTaStatus PspTrustedApps::release_after_reset() noexcept {
    if (!any_retained() && phase_ != Phase::faulted) {
        return PspTaStatus::busy;
    }

    if (recovery_.confirm_reset_quiescence == nullptr ||
        !recovery_.confirm_reset_quiescence(recovery_.context)) {
        return PspTaStatus::quarantined;
    }

    release(report_.ras, buffers_.ras);
    release(report_.hdcp, buffers_.hdcp);
    release(report_.dtm, buffers_.dtm);
    phase_ = Phase::idle;
    return PspTaStatus::released;
}

} // namespace rx7900::hardware

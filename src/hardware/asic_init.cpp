/*
Module: rx7900.hardware.asic_init
Purpose: Sequence the Linux v6.12 Navi31 firmware-loading hardware init order.
Created: 2026-10-21
Architecture: Composition only; every register/command encoding lives in its module.
Copyright 2016, 2020 Advanced Micro Devices, Inc. Adapted Linux v6.12 amdgpu_psp.c
(psp_load_fw, psp_hw_start, psp_load_non_psp_fw, psp_tmr_init, psp_suspend,
psp_resume), amdgpu_smu.c (smu_hw_init, smu_smc_hw_setup, smu_suspend, smu_resume),
amdgpu_device.c (amdgpu_device_suspend/resume, ip_suspend/resume phases,
amdgpu_device_need_post, amdgpu_device_asic_init), amdgpu_atomfirmware.c
(amdgpu_atomfirmware_asic_init), psp_v13_0.c (psp_v13_0_wait_for_bootloader),
amdgpu_drv.c (amdgpu_pmops_*), amdgpu_gtt_mgr.c (amdgpu_gtt_mgr_recover) and
soc21.c. Notice: licenses/AMDGPU-FIRMWARE.txt.
*/
#include "asic_init.hpp"

namespace rx7900::hardware {
namespace {

constexpr uint32_t sos_part_toc = 4;
constexpr uint32_t sos_part_register_list = 6;
constexpr uint8_t dmcub_ucode_id = 61;
constexpr uint32_t maximum_boot_steps = 8;
// amdgpu_fence_wait_empty has no bound; shutdown and suspend stop waiting after
// the 60 s kernel-ring job timeout instead (amdgpu_job_timedout's budget).
constexpr uint64_t fence_drain_timeout_us = 60000000;
// atombios.h ATOM_S7_ASIC_INIT_COMPLETE_MASK in BIOS_SCRATCH_7.
constexpr uint32_t atom_s7_asic_init_complete = 0x00000200u;
// enum pp_df_cstate (kgd_pp_interface.h).
constexpr uint32_t df_cstate_disallow = 0;
constexpr uint32_t df_cstate_allow = 1;
// psp_v13_0_wait_for_bootloader: psp_wait_for(MP0 C2PMSG_35 == 0x80000000, mask
// 0xffffffff) retried 10 times, each adev->usec_timeout 1 us polls.
constexpr uint32_t psp_c2pmsg_35 = 0x63;
constexpr uint32_t psp_bootloader_ready = 0x80000000u;
constexpr uint32_t psp_bootloader_retries = 10;
constexpr uint32_t adev_usec_timeout = 100000;

bool wait_psp_bootloader(const RegisterIo& io, const PspRegisters& psp) noexcept {
    for (uint32_t attempt = 0; attempt < psp_bootloader_retries * adev_usec_timeout; ++attempt) {
        uint32_t value = 0;

        if (read_ip_register(io, psp.segment0_dword_base(), psp_c2pmsg_35, value) &&
            value == psp_bootloader_ready) {
            return true;
        }

        io.delay_microseconds(io.context, 1);
    }

    return false;
}

// amdgpu_atombios_scratch_need_asic_init: BIOS_SCRATCH_7 ASIC_INIT_COMPLETE.
// A failed read or an all-ones bus value proves nothing either way.
bool read_posted(const RegisterIo& io, uint32_t bios_scratch_reg, bool& posted) noexcept {
    uint32_t scratch = 0;

    if (!read_ip_register(io, 0, bios_scratch_reg + 7, scratch) || scratch == UINT32_MAX) {
        return false;
    }

    posted = (scratch & atom_s7_asic_init_complete) != 0;
    return true;
}

const FirmwarePart* sos_part(const FirmwareImage& image, uint32_t identifier) noexcept {
    for (uint32_t index = 0; index < image.part_count; ++index) {
        if (image.parts[index].identifier == identifier) {
            return &image.parts[index];
        }
    }

    return nullptr;
}

// Statuses after which PSP may still access ring, command, fence or staging memory.
bool ring_uncertain(PspRingStatus status) noexcept {
    return status == PspRingStatus::timeout || status == PspRingStatus::mmio_failure ||
           status == PspRingStatus::unexpected_response || status == PspRingStatus::quarantined;
}

} // namespace

bool AsicInit::read32(void* context, uint32_t offset, uint32_t& value) noexcept {
    const auto& self = *static_cast<AsicInit*>(context);
    return self.io_.read32(self.io_.context, offset, value);
}

bool AsicInit::write32(void* context, uint32_t offset, uint32_t value) noexcept {
    const auto& self = *static_cast<AsicInit*>(context);
    return self.io_.write32(self.io_.context, offset, value);
}

bool AsicInit::publish(void* context, PspMcAddress, size_t) noexcept {
    // Boot staging is local VRAM written through the uncached BAR0 window, so
    // publication to PSP is exactly the HDP flush.
    return static_cast<AsicInit*>(context)->nbio_.flush_hdp();
}

void AsicInit::delay(void* context, uint32_t microseconds) noexcept {
    const auto& self = *static_cast<AsicInit*>(context);
    self.io_.delay_microseconds(self.io_.context, microseconds);
}

bool AsicInit::confirm(void* context) noexcept {
    const auto& self = *static_cast<AsicInit*>(context);
    return self.reset_done_ || (self.recovery_.confirm_reset_quiescence != nullptr &&
                                self.recovery_.confirm_reset_quiescence(self.recovery_.context));
}

AsicStatus AsicInit::fail(AsicStep step, AsicStatus status) noexcept {
    report_.step = step;
    report_.status = status;
    return status;
}

bool AsicInit::mes_misc(void* context, ByteView frame, uint32_t api_status_offset) noexcept {
    return static_cast<AsicInit*>(context)->mes_.misc_op(frame, api_status_offset) ==
           Mes11Status::completed;
}

bool AsicInit::ih_bus_master(void* context) noexcept {
    const auto& platform = static_cast<AsicInit*>(context)->ih_platform_;
    return platform.enable_bus_master != nullptr && platform.enable_bus_master(platform.context);
}

bool AsicInit::ih_doorbell(void* context, uint32_t index, uint32_t value) noexcept {
    const auto& platform = static_cast<AsicInit*>(context)->ih_platform_;
    return platform.write_doorbell32 != nullptr && platform.write_doorbell32(platform.context, index, value);
}

bool AsicInit::in_gtt(Owned slot) noexcept {
    const auto index = static_cast<uint32_t>(slot);

    switch (slot) {
    case Owned::ib_gfx: case Owned::ib_sdma:
    case Owned::selftest_metadata: case Owned::selftest_process: case Owned::selftest_gang0:
    case Owned::selftest_gang1: case Owned::selftest_gang2: case Owned::selftest_mqd0:
    case Owned::selftest_mqd1: case Owned::selftest_mqd2:
    case Owned::ih_ring0: case Owned::ih_ring1: case Owned::ih_writeback:
    case Owned::sdma_ring0: case Owned::sdma_ring1: case Owned::sdma_writeback:
    case Owned::kiq_ring: case Owned::mes_ring: case Owned::kiq_mqd: case Owned::mes_mqd:
    case Owned::kiq_eop: case Owned::mes_eop: case Owned::mes_writeback:
    case Owned::gfx_ring: case Owned::gfx_mqd: case Owned::compute_eop: case Owned::queue_writeback:
        return true;
    default:
        return index >= static_cast<uint32_t>(Owned::compute_ring0) &&
               index < static_cast<uint32_t>(Owned::compute_eop);
    }
}

memory::CreateResult AsicInit::gtt_create(void* context, sdma::GpuAllocation gpu,
                                          memory::Access access, bool,
                                          memory::Backing& backing) noexcept {
    auto& self = *static_cast<AsicInit*>(context);
    const GartLayout gart = self.gmc_.gart();
    backing = {};

    // Between suspend and resume the GART table and the hubs are powered down.
    if (self.suspended_ || gpu.byte_length == 0 || gpu.byte_length > UINT32_MAX ||
        gpu.virtual_address < gart.start || (gpu.virtual_address & (memory::page_bytes - 1)) != 0) {
        return memory::CreateResult::no_resources;
    }

    uint8_t* cpu = nullptr;
    uint64_t bus = 0;
    void* token = nullptr;

    if (!self.gtt_platform_.allocate(self.gtt_platform_.context,
                                     static_cast<uint32_t>(gpu.byte_length), cpu, bus, token)) {
        return memory::CreateResult::no_resources;
    }

    if (cpu == nullptr || (bus & (memory::page_bytes - 1)) != 0) {
        return self.gtt_platform_.release(self.gtt_platform_.context, token) ?
                   memory::CreateResult::no_resources : memory::CreateResult::quarantined;
    }

    backing = {token, cpu, static_cast<size_t>(gpu.byte_length)};

    if (!self.bind_gtt(gpu.virtual_address, bus, gpu.byte_length, access)) {
        return memory::CreateResult::quarantined;
    }

    for (auto& entry : self.gtt_pages_) {
        if (entry.gart == 0) {
            entry = {gpu.virtual_address, bus, gpu.byte_length, access};
            break;
        }
    }

    return memory::CreateResult::created;
}

// amdgpu_gart_bind (and, on resume, amdgpu_ttm_recover_gart) of one physically
// contiguous GTT buffer at its GART address, in bounded chunks; a partial bind
// leaves PTEs uncertain, so the caller quarantines on false.
bool AsicInit::bind_gtt(uint64_t gart_address, uint64_t bus, uint64_t bytes,
                        memory::Access access) noexcept {
    const GartLayout gart = gmc_.gart();

    if (bytes == 0 || gart_address < gart.start) {
        return false;
    }

    const uint64_t offset = gart_address - gart.start;
    const uint64_t pages = (bytes + memory::page_bytes - 1) / memory::page_bytes;
    constexpr uint64_t chunk_pages = 64;
    uint8_t list[chunk_pages * 8]{};

    for (uint64_t done = 0; done < pages; done += chunk_pages) {
        const uint64_t count = pages - done < chunk_pages ? pages - done : chunk_pages;
        const sdma::CommandBuffer out(list, sizeof(list));

        for (uint64_t page = 0; page < count; ++page) {
            const uint64_t address = bus + (done + page) * memory::page_bytes;
            out.write_u32(static_cast<size_t>(page * 8), static_cast<uint32_t>(address));
            out.write_u32(static_cast<size_t>(page * 8 + 4), static_cast<uint32_t>(address >> 32));
        }

        if (gmc_.map_system_pages(offset + done * memory::page_bytes,
                                  {list, static_cast<size_t>(count * 8)}, access) != GmcStatus::mapped) {
            return false;
        }
    }

    return true;
}

bool AsicInit::gtt_bus(memory::Handle handle, uint64_t& bus) const noexcept {
    memory::AllocationInfo info{};

    if (gtt_.describe(handle, info) != memory::Status::success) {
        return false;
    }

    for (const auto& entry : gtt_pages_) {
        if (entry.gart != 0 && entry.gart == info.gpu.virtual_address) {
            bus = entry.bus;
            return true;
        }
    }

    return false;
}

bool AsicInit::gtt_destroy(void* context, const memory::Backing& backing,
                           sdma::GpuAllocation gpu) noexcept {
    auto& self = *static_cast<AsicInit*>(context);
    const uint64_t pages = (gpu.byte_length + memory::page_bytes - 1) / memory::page_bytes;

    // A suspended device cannot unbind (the table and HDP are powered down): the
    // entry stays recorded and is rebound on resume, and the Manager keeps the
    // allocation quarantined until a later confirmed destroy.
    if (self.suspended_ && !self.reset_confirmed_) {
        return false;
    }

    for (auto& entry : self.gtt_pages_) {
        if (entry.gart == gpu.virtual_address) {
            entry = {};
        }
    }

    // After a confirmed reset the hubs are back at their defaults and no engine
    // can translate through the old table; only the pages remain to free.
    if (self.reset_confirmed_) {
        return self.gtt_platform_.release(self.gtt_platform_.context, backing.token);
    }


    return self.gmc_.unmap_system_pages(gpu.virtual_address - self.gmc_.gart().start, pages) ==
               GmcStatus::flushed &&
           self.gtt_platform_.release(self.gtt_platform_.context, backing.token);
}

bool AsicInit::allocate(Owned slot, uint64_t bytes, uint64_t alignment, bool cpu_visible) noexcept {
    const size_t index = static_cast<size_t>(slot);
    memory::Handle created{};

    // Resume re-runs hw_init on the kernel memory sw_init created once (the
    // amdgpu_bo_create_kernel BOs stay pinned across suspend; psp_tmr_init finds
    // its tmr_bo): the slot's own allocation, which must still satisfy the request.
    if (resuming_ && owned_[index]) {
        memory::AllocationInfo info{};
        const bool fits = manager_for(slot).describe(handles_[index], info) == memory::Status::success &&
            info.gpu.byte_length >= bytes && alignment != 0 &&
            info.gpu.virtual_address % alignment == 0 &&
            (!cpu_visible || info.backing.cpu_bytes != nullptr);
        report_.memory = fits ? memory::Status::success : memory::Status::unavailable;
        return fits;
    }

    if (in_gtt(slot) && !gtt_ready_) {
        report_.memory = memory::Status::unavailable;
        return false;
    }

    report_.memory = manager_for(slot).allocate(bytes, alignment, memory::Access::read_write,
                                                cpu_visible, created);

    if (report_.memory != memory::Status::success) {
        return false;
    }

    handles_[index] = created;
    owned_[index] = true;
    return true;
}

AsicStatus AsicInit::train_if_needed(const AsicInputs& inputs, PspRegisters registers) noexcept {
    // psp_sw_init: training is requested by the runtime database and supported
    // by firmwareinfo; psp_v13_0_memory_training skips it once the SOS is alive.
    if (report_.training_requested) {
        if (inputs.atom == nullptr) {
            return fail(AsicStep::memory_training, AsicStatus::unsupported);
        }

        // Recovery after a mode-1 reset: amdgpu_device_fw_loading calls
        // psp_resume while in reset, whose psp_mem_training(PSP_MEM_TRAIN_RESUME)
        // escalates to a long training only when the P2C cache demands it (and
        // then needs the callbacks, refused before any write without them).
        if (restarting_ && (inputs.atom->capability & atom_capability_two_stage_training) != 0) {
            const auto trained = run_psp_resume_training(registers, io_, nbio_, report_.vram.bytes,
                inputs.atom->reserved_fb_bytes, inputs.training, training_data_, report_.training);

            if (trained != PspTrainingStatus::completed) {
                return fail(AsicStep::memory_training, trained == PspTrainingStatus::unavailable ?
                                                       AsicStatus::unsupported : AsicStatus::failed);
            }

            return AsicStatus::completed;
        }

        if ((inputs.atom->capability & atom_capability_two_stage_training) != 0) {
            if (inputs.training.save_encroached == nullptr ||
                inputs.training.restore_encroached == nullptr) {
                return fail(AsicStep::memory_training, AsicStatus::unsupported);
            }

            const auto trained = run_psp_long_training(registers, io_, nbio_, report_.vram.bytes,
                inputs.atom->reserved_fb_bytes, inputs.training, training_data_, report_.training);

            if (trained != PspTrainingStatus::completed) {
                return fail(AsicStep::memory_training, AsicStatus::failed);
            }
        }
    }

    return AsicStatus::completed;
}

AsicStatus AsicInit::boot_psp(const AsicInputs& inputs, PspRegisters registers,
                              Ih6Registers ih_registers, NbioRegisters nbio) noexcept {
    const PspBootIo boot_io{this, read32, write32, publish, delay, confirm};
    report_.boot = boot_.initialize(registers, boot_io, policy_.boot);

    if (report_.boot == PspBootStatus::already_running) {
        // psp_v13_0_bootloader_load_sos returns early when the SOS is alive and
        // psp_v13_0_memory_training skips; IH phase1 init still precedes firmware.
        report_.sos_was_running = true;
        return start_ih(inputs, ih_registers, nbio);
    }

    if (report_.boot != PspBootStatus::bootloader_ready) {
        return fail(AsicStep::psp_bootloader, AsicStatus::failed);
    }

    // Training belongs to PSP sw_init; IH (phase1) follows before firmware load.
    const auto trained = train_if_needed(inputs, registers);

    if (trained != AsicStatus::completed) {
        return trained;
    }

    const auto interrupts = start_ih(inputs, ih_registers, nbio);

    if (interrupts != AsicStatus::completed) {
        return interrupts;
    }

    if (!allocate(Owned::boot_staging, psp_staging_bytes, psp_staging_bytes, true)) {
        return fail(AsicStep::psp_bootloader, AsicStatus::failed);
    }

    const AsicStatus booted = load_sos(inputs, handle(Owned::boot_staging));

    if (booted == AsicStatus::completed) {
        report_.memory = manager_->release(handle(Owned::boot_staging));
        owned_[static_cast<size_t>(Owned::boot_staging)] = report_.memory != memory::Status::success;
    }

    return booted;
}

// psp_hw_start bootloader part: KDB, SPL, SYS, (SOC/INTF/DBG/RAS), SOS staged in
// one 1 MiB, 1 MiB-aligned CPU-visible VRAM buffer (Linux psp->fw_pri_buf).
AsicStatus AsicInit::load_sos(const AsicInputs& inputs, memory::Handle staging) noexcept {
    memory::AllocationInfo info{};
    manager_->describe(staging, info);
    report_.boot = boot_.prepare(inputs.sos, *manager_, staging, {info.gpu.virtual_address});

    if (report_.boot != PspBootStatus::prepared) {
        return fail(AsicStep::psp_bootloader, AsicStatus::failed);
    }

    for (uint32_t step = 0; step < maximum_boot_steps; ++step) {
        report_.boot = boot_.submit_next();

        if (report_.boot == PspBootStatus::submitted) {
            report_.boot = boot_.wait();
        }

        if (report_.boot == PspBootStatus::secure_os_running) {
            return AsicStatus::completed;
        }

        if (report_.boot != PspBootStatus::component_completed) {
            break;
        }
    }

    return fail(AsicStep::psp_bootloader, boot_.report().staging_retained ?
                AsicStatus::quarantined : AsicStatus::failed);
}

AsicStatus AsicInit::load_firmware(const AsicInputs& inputs, const FirmwareImage& sos) noexcept {
    const auto ring_failure = [&](AsicStep step) noexcept {
        report_.ring = ring_.report().status;
        return fail(step, ring_uncertain(report_.ring) ? AsicStatus::quarantined :
                                                          AsicStatus::failed);
    };

    if (!allocate(Owned::psp_control, psp_control_bytes, psp_ring_bytes, true) ||
        !allocate(Owned::psp_firmware, psp_firmware_buffer_bytes, psp_firmware_buffer_bytes, true)) {
        return fail(AsicStep::psp_ring, AsicStatus::failed);
    }

    if (ring_.create(*manager_, handle(Owned::psp_control), handle(Owned::psp_firmware)) !=
        PspRingStatus::ring_created) {
        return ring_failure(AsicStep::psp_ring);
    }

    // psp_tmr_init: PSP sizes the TMR from the TOC; otherwise PSP_TMR_SIZE.
    uint32_t tmr_bytes = psp_tmr_default_bytes;
    const FirmwarePart* toc = sos_part(sos, sos_part_toc);

    if (toc != nullptr) {
        ByteView toc_bytes(nullptr, 0);

        if (!inputs.sos.subview(toc->offset, toc->bytes, toc_bytes) ||
            ring_.load_toc(toc_bytes, tmr_bytes) != PspRingStatus::completed) {
            return ring_failure(AsicStep::psp_toc);
        }
    }

    const uint64_t page_mask = memory::page_bytes - 1;
    const uint64_t rounded = (static_cast<uint64_t>(tmr_bytes) + page_mask) & ~page_mask;

    if (rounded > UINT32_MAX) {
        return fail(AsicStep::tmr_allocation, AsicStatus::failed);
    }

    report_.tmr_bytes = static_cast<uint32_t>(rounded);

    if (!allocate(Owned::tmr, report_.tmr_bytes, psp_tmr_alignment, false)) {
        return fail(AsicStep::tmr_allocation, AsicStatus::failed);
    }

    // pmfw_centralized_cstate_management: PMFW loads after TOC, before SETUP_TMR.
    UcodeEntry smc{};
    ByteView payload(nullptr, 0);

    if (!inputs.catalog->find(ucode_id::smc, smc) || !inputs.catalog->payload(smc, payload) ||
        ring_.load_ip_firmware(payload, smc.psp_type) != PspRingStatus::completed) {
        report_.failed_ucode_id = ucode_id::smc;
        return ring_failure(AsicStep::smu_firmware);
    }

    if (ring_.setup_tmr(handle(Owned::tmr), report_.tmr_bytes) != PspRingStatus::completed) {
        return ring_failure(AsicStep::psp_tmr);
    }

    for (size_t index = 0; index < inputs.catalog->count(); ++index) {
        const UcodeEntry entry = inputs.catalog->entry(index);

        if (entry.linux_id == ucode_id::smc ||
            (entry.linux_id == dmcub_ucode_id && !policy_.load_dmcub)) {
            continue;
        }

        if (!inputs.catalog->payload(entry, payload) ||
            ring_.load_ip_firmware(payload, entry.psp_type) != PspRingStatus::completed) {
            report_.failed_ucode_id = entry.linux_id;
            return ring_failure(AsicStep::ip_firmware);
        }

        ++report_.loaded_ucode;

        if (entry.linux_id == ucode_id::rlc_g) {
            if (ring_.autoload_rlc() != PspRingStatus::completed) {
                return ring_failure(AsicStep::rlc_autoload);
            }

            report_.rlc_autoload_started = true;
        }
    }

    if (inputs.ta.size() != 0) {
        uint32_t offset = 0;
        uint32_t bytes = 0;

        if (find_ta_asd(inputs.ta, offset, bytes) != UcodeStatus::success ||
            !inputs.ta.subview(offset, bytes, payload) ||
            ring_.load_asd(payload, report_.asd_session) != PspRingStatus::completed) {
            return ring_failure(AsicStep::asd);
        }
    }

    const FirmwarePart* register_list = sos_part(sos, sos_part_register_list);

    if (register_list != nullptr &&
        (!inputs.sos.subview(register_list->offset, register_list->bytes, payload) ||
         ring_.load_register_list(payload) != PspRingStatus::completed)) {
        return ring_failure(AsicStep::register_list);
    }

    // psp_load_fw after psp_rl_load: RAS, HDCP and DTM TAs from the TA image.
    // Linux treats each failure as non-fatal; only an uncertain ring stops here.
    // amdgpu_device_has_display_hardware: DMU (DCE_HWIP) discovered, not harvested.
    DiscoveredIp dmu;
    const bool display = discover_ip(inputs.discovery, {271, 0, 3, 2, 0, 1}, dmu) ==
                         DiscoveryStatus::validated;
    PspTaInputs ta{};
    ta.ta_file = inputs.ta;
    ta.has_display_hardware = display;
    ta.ras.atom_capability = inputs.atom != nullptr ? inputs.atom->capability : 0;
    bool mem_ecc = false;

    if (inputs.atom_rom.size() != 0 && atom_mem_ecc_supported(inputs.atom_rom, mem_ecc) !=
                                           AtomFirmwareStatus::success) {
        mem_ecc = false;
    }

    ta.ras.mem_ecc_supported = mem_ecc;
    // adev->gfx.xcc_mask: one GC instance on Navi31.
    ta.xcc_mask = 1;
    ta.m_half_use = discovery_mall_half_use(inputs.discovery);

    if (!allocate(Owned::ta_ras, psp_ta_shared_bytes, memory::page_bytes, true) ||
        !allocate(Owned::ta_hdcp, psp_ta_shared_bytes, memory::page_bytes, true) ||
        !allocate(Owned::ta_dtm, psp_ta_shared_bytes, memory::page_bytes, true)) {
        return fail(AsicStep::register_list, AsicStatus::failed);
    }

    if (apps_.initialize(ring_, *manager_, report_.vram,
                         {handle(Owned::ta_ras), handle(Owned::ta_hdcp), handle(Owned::ta_dtm)},
                         ta, {false}, {this, confirm}) == PspTaStatus::quarantined) {
        return ring_failure(AsicStep::register_list);
    }

    report_.ring = ring_.report().status;
    return AsicStatus::completed;
}

AsicStatus AsicInit::start_gmc(const AsicInputs& inputs, GmcRegisters registers) noexcept {
    // amdgpu_device_ip_init: GMC hw_init runs during sw_init, right after the
    // VRAM scratch page, so later firmware/ring memory has a working MMHUB.
    report_.gmc = gmc_.initialize(registers, io_, nbio_, report_.vram, {100000}, {this, confirm});

    if (report_.gmc != GmcStatus::ready ||
        !allocate(Owned::gart_table, gart_table_bytes, memory::page_bytes, true) ||
        !allocate(Owned::scratch, memory::page_bytes, memory::page_bytes, false)) {
        return fail(AsicStep::gmc, AsicStatus::failed);
    }

    report_.gmc = gmc_.prepare_gart(*manager_, handle(Owned::gart_table), handle(Owned::scratch),
                                    inputs.dummy_page_bus);

    if (report_.gmc == GmcStatus::prepared) {
        report_.gmc = gmc_.enable_mmhub();
    }

    if (report_.gmc != GmcStatus::enabled) {
        return fail(AsicStep::gmc, gmc_.report().mmhub_enabled || report_.gmc == GmcStatus::timeout ||
                                   report_.gmc == GmcStatus::mmio_failure ?
                                   AsicStatus::quarantined : AsicStatus::failed);
    }

    // The GTT domain: the whole GART aperture, bound page by page on allocation.
    const GartLayout gart = gmc_.gart();
    const sdma::GpuAllocation arena{gart.start, gart.end - gart.start + 1};

    if (restarting_ && !gtt_.empty()) {
        // amdgpu_do_asic_reset -> amdgpu_ttm_recover_gart: allocations retained
        // across the confirmed reset keep their GART addresses (same aperture)
        // and their system pages are bound into the new table, exactly as
        // resume_gmc rebinds them; a partial bind leaves PTEs uncertain.
        if (arena.virtual_address != gtt_arena_.virtual_address || arena.byte_length != gtt_arena_.byte_length) {
            return fail(AsicStep::gmc, AsicStatus::failed);
        }

        const AsicStatus rebound = rebind_gtt_pages(AsicStep::gmc);

        if (rebound != AsicStatus::completed) {
            return rebound;
        }

        gtt_ready_ = true;
        return AsicStatus::completed;
    }

    if (gtt_.initialize(arena, {this, gtt_create, gtt_destroy}) != memory::Status::success) {
        return fail(AsicStep::gmc, AsicStatus::failed);
    }

    gtt_arena_ = arena;
    gtt_ready_ = true;

    return AsicStatus::completed;
}

AsicStatus AsicInit::start_ih(const AsicInputs& inputs, Ih6Registers registers,
                              NbioRegisters nbio) noexcept {
    // amdgpu_device_ip_hw_init_phase1: IH hw_init after every sw_init (including
    // PSP training) and before firmware loading. Linux sizes both rings 256 KiB.
    report_.ih = ih_.initialize(registers, nbio, io_, nbio_, gtt_window(),
        {ih6_default_ring_bytes, inputs.msi_enabled, inputs.dummy_page_bus},
        {this, ih_bus_master, inputs.ih_platform.write_doorbell32 != nullptr ? ih_doorbell : nullptr,
         confirm});

    if (report_.ih != Ih6Status::ready ||
        !allocate(Owned::ih_ring0, ih6_default_ring_bytes, ih6_ring_alignment, true) ||
        !allocate(Owned::ih_ring1, ih6_default_ring_bytes, ih6_ring_alignment, true) ||
        !allocate(Owned::ih_writeback, memory::page_bytes, memory::page_bytes, true)) {
        return fail(AsicStep::interrupts, AsicStatus::failed);
    }

    report_.ih = ih_.hw_init(gtt_, handle(Owned::ih_ring0), handle(Owned::ih_ring1),
                             handle(Owned::ih_writeback));

    if (report_.ih != Ih6Status::enabled) {
        return fail(AsicStep::interrupts, report_.ih == Ih6Status::quarantined ||
                                          report_.ih == Ih6Status::mmio_failure ||
                                          report_.ih == Ih6Status::platform_failure ?
                                          AsicStatus::quarantined : AsicStatus::failed);
    }

    if (inputs.interrupt_ready.ih_ready != nullptr &&
        !inputs.interrupt_ready.ih_ready(inputs.interrupt_ready.context, ih_)) {
        return fail(AsicStep::interrupts, AsicStatus::failed);
    }

    return AsicStatus::completed;
}

AsicStatus AsicInit::start_gfx(const AsicInputs& inputs, memory::GcRegisterBase registers) noexcept {
    // gfx_v11_0_hw_init on the PSP-loaded RS64 path.
    Gfx11StartAddresses start{};

    if (rs64_start_address(inputs.catalog->file(UcodeSource::pfp), start.pfp_low, start.pfp_high) !=
            UcodeStatus::success ||
        rs64_start_address(inputs.catalog->file(UcodeSource::me), start.me_low, start.me_high) !=
            UcodeStatus::success ||
        rs64_start_address(inputs.catalog->file(UcodeSource::mec), start.mec_low, start.mec_high) !=
            UcodeStatus::success) {
        return fail(AsicStep::gfx, AsicStatus::invalid_input);
    }

    report_.gfx = gfx_.initialize(registers, io_, nbio_, report_.vram, {100000}, {this, confirm});

    if (report_.gfx == Gfx11Status::ready) {
        report_.gfx = gfx_.wait_for_autoload();
    }

    if (report_.gfx == Gfx11Status::completed) {
        report_.gfx = gfx_.read_gb_addr_config();
    }

    if (report_.gfx == Gfx11Status::completed) {
        report_.gfx = gfx_.configure_rs64(start);
    }

    if (report_.gfx != Gfx11Status::completed) {
        return fail(AsicStep::gfx, AsicStatus::failed);
    }

    report_.gmc = gmc_.enable_gfxhub();

    if (report_.gmc != GmcStatus::enabled) {
        return fail(AsicStep::gfx, AsicStatus::quarantined);
    }

    report_.gfx = gfx_.golden_registers();

    if (report_.gfx == Gfx11Status::completed) {
        report_.gfx = gfx_.constants_init();
    }

    if (report_.gfx == Gfx11Status::completed && nbio_.gc_doorbell_init() != NbioStatus::ready) {
        report_.gfx = Gfx11Status::mmio_failure;
    }

    if (report_.gfx == Gfx11Status::completed &&
        !allocate(Owned::clear_state, (Gfx11Init::clear_state_bytes() + memory::page_bytes - 1) &
                  ~(memory::page_bytes - 1), memory::page_bytes, true)) {
        return fail(AsicStep::gfx, AsicStatus::failed);
    }

    if (report_.gfx == Gfx11Status::completed) {
        report_.gfx = gfx_.rlc_resume(*manager_, handle(Owned::clear_state));
    }

    if (report_.gfx == Gfx11Status::completed) {
        report_.gfx = gfx_.cp_start();
    }

    if (report_.gfx != Gfx11Status::completed) {
        return fail(AsicStep::gfx, report_.gfx == Gfx11Status::invalid_memory ?
                                   AsicStatus::failed : AsicStatus::quarantined);
    }

    return AsicStatus::completed;
}

void AsicInit::run_self_test(NbioRegisters nbio) noexcept {
    // amdgpu_mes_self_test: Linux ignores the result, so allocation or test
    // failures are recorded and bring-up continues; quarantined memory stays.
    const Owned tables[]{Owned::selftest_table0, Owned::selftest_table1, Owned::selftest_table2,
                         Owned::selftest_table3};
    const Owned gtt[]{Owned::selftest_process, Owned::selftest_gang0, Owned::selftest_gang1,
                      Owned::selftest_gang2, Owned::selftest_mqd0, Owned::selftest_mqd1,
                      Owned::selftest_mqd2};
    bool allocated = allocate(Owned::selftest_metadata, mes11_ctx::bytes, memory::page_bytes, true);

    for (const auto slot : tables) {
        allocated = allocated && allocate(slot, memory::page_bytes, memory::page_bytes, true);
    }

    for (const auto slot : gtt) {
        allocated = allocated && allocate(slot, memory::page_bytes, memory::page_bytes, true);
    }

    uint64_t bus = 0;

    if (!allocated || !gtt_bus(handle(Owned::selftest_metadata), bus)) {
        return;
    }

    // The metadata is one contiguous GttPlatform buffer: its pages in order.
    uint8_t pages[mes_selftest_metadata_pages * 8]{};
    const sdma::CommandBuffer list(pages, sizeof(pages));

    for (uint32_t page = 0; page < mes_selftest_metadata_pages; ++page) {
        const uint64_t address = bus + static_cast<uint64_t>(page) * memory::page_bytes;
        list.write_u32(page * 8, static_cast<uint32_t>(address));
        list.write_u32(page * 8 + 4, static_cast<uint32_t>(address >> 32));
    }

    MesSelfTestVmMemory vm_tables{};
    Mes11SelfTestMemory queues{};

    for (uint32_t level = 0; level < mes_selftest_vm_levels; ++level) {
        vm_tables.tables[level] = handle(tables[level]);
    }

    queues.process_context = handle(Owned::selftest_process);

    for (uint32_t queue = 0; queue < mes11_self_test_queues; ++queue) {
        queues.gang_context[queue] = handle(gtt[1 + queue]);
        queues.mqd[queue] = handle(gtt[4 + queue]);
    }

    const MesSelfTestInputs inputs{manager_, vm_tables, &gtt_,
        {report_.vram, gmc_.gart(), handle(Owned::selftest_metadata), {pages, sizeof(pages)},
         {this, confirm}},
        queues, {nbio, gmc_.gart(), mes11_self_test_ib_timeout_us}, &nbio_};
    report_.mes_self_test = run_mes_self_test(mes_, selftest_vm_, inputs);
}

AsicStatus AsicInit::late_gating_and_ib_tests(const AsicInputs& inputs, memory::GcRegisterBase gc,
                                              NbioRegisters nbio) noexcept {
    // amdgpu_device_ip_late_init tail: set_cg_state(GATE), then set_pg_state(GATE)
    // (GFXOFF allowed through the SMU after the 100 ms GFX_OFF_DELAY_ENABLE).
    report_.gating = gating_.initialize(inputs.discovery, gc, nbio, io_, nbio_,
                                        clock_gating_linux_policy, {this, confirm});

    if (report_.gating != ClockGatingStatus::ready) {
        return fail(AsicStep::clock_gating, AsicStatus::failed);
    }

    report_.gating = gating_.gate(ih_, smu_);

    // Linux ignores gating results; a PMFW refusal of AllowGfxOff only leaves GFX
    // powered. An uncertain register store or SMU response is quarantined.
    if (report_.gating != ClockGatingStatus::gated && report_.gating != ClockGatingStatus::smu_refused) {
        return fail(AsicStep::clock_gating, AsicStatus::quarantined);
    }

    // amdgpu_device_delayed_init_work_handler -> amdgpu_ib_ring_tests, which
    // Linux runs AMDGPU_RESUME_MS (2 s) later from a work item; here it runs
    // directly. Gfx ring 0 then compute; SDMA only if gfx ring 0 passed.
    uint32_t sdma_feature_version = 0;

    // sdma_firmware_header_v1/v2/v3 ucode_feature_version follows the common header.
    if (!inputs.catalog->file(UcodeSource::sdma).read_u32(32, sdma_feature_version)) {
        return fail(AsicStep::ib_tests, AsicStatus::invalid_input);
    }

    if (!allocate(Owned::ib_gfx, memory::page_bytes, memory::page_bytes, true) ||
        !allocate(Owned::ib_sdma, memory::page_bytes, memory::page_bytes, true)) {
        return fail(AsicStep::ib_tests, AsicStatus::failed);
    }

    report_.gfx_ib = queues_.test_ib(gtt_, handle(Owned::ib_gfx), {nbio, gfx11_ib_test_timeout_us});

    if (report_.gfx_ib == Gfx11QueuesStatus::running) {
        report_.sdma_ib = sdma_.test_ib(gtt_, handle(Owned::ib_sdma),
                                        {sdma_feature_version, sdma6_ib_test_timeout_us});
    }

    // Linux only logs an IB-test failure (accel_working = false); a ring that
    // missed its fence may still fetch, so its memory stays quarantined here.
    if (report_.gfx_ib != Gfx11QueuesStatus::running || report_.sdma_ib != Sdma6Status::started) {
        return fail(AsicStep::ib_tests, AsicStatus::quarantined);
    }

    return AsicStatus::completed;
}

AsicStatus AsicInit::start_mes_kiq(const AsicInputs& inputs, Mes11Registers registers) noexcept {
    Mes11Config config{0, 0, sdma6_instance_count};

    if (!mes11_uc_start_address(inputs.catalog->file(UcodeSource::mes), config.scheduler_uc_start) ||
        !mes11_uc_start_address(inputs.catalog->file(UcodeSource::mes_kiq), config.kiq_uc_start)) {
        return fail(AsicStep::mes_kiq, AsicStatus::invalid_input);
    }

    report_.mes = mes_.initialize(registers, io_, inputs.doorbells, nbio_, gtt_window(), config,
                                  policy_.mes, {this, confirm});

    if (report_.mes != Mes11Status::ready) {
        return fail(AsicStep::mes_kiq, AsicStatus::failed);
    }

    // amdgpu_mes_init / mes_v11_0_sw_init buffers; Manager allocations are whole pages.
    const auto pages = [](uint32_t bytes) {
        return (static_cast<uint64_t>(bytes) + memory::page_bytes - 1) & ~(memory::page_bytes - 1);
    };

    if (!allocate(Owned::kiq_ring, mes11_kiq_ring_bytes, mes11_page_alignment, true) ||
        !allocate(Owned::mes_ring, mes11_scheduler_ring_bytes, mes11_page_alignment, true) ||
        !allocate(Owned::kiq_mqd, pages(mes11_mqd_bytes), mes11_page_alignment, true) ||
        !allocate(Owned::mes_mqd, pages(mes11_mqd_bytes), mes11_page_alignment, true) ||
        !allocate(Owned::kiq_eop, pages(mes11_eop_bytes), mes11_page_alignment, true) ||
        !allocate(Owned::mes_eop, pages(mes11_eop_bytes), mes11_page_alignment, true) ||
        !allocate(Owned::mes_writeback, mes11_writeback_bytes, mes11_page_alignment, true)) {
        return fail(AsicStep::mes_kiq, AsicStatus::failed);
    }

    // gfx_v11_0_cp_resume -> amdgpu_mes_kiq_hw_init; with scheduler firmware
    // >= 0x47 this also runs mes_v11_0_hw_init (scheduler map and resources).
    report_.mes = mes_.kiq_hw_init(gtt_, {handle(Owned::kiq_ring), handle(Owned::mes_ring),
        handle(Owned::kiq_mqd), handle(Owned::mes_mqd), handle(Owned::kiq_eop),
        handle(Owned::mes_eop), handle(Owned::mes_writeback)});

    if (report_.mes != Mes11Status::scheduler_ready && report_.mes != Mes11Status::kiq_ready) {
        return fail(AsicStep::mes_kiq, report_.mes == Mes11Status::invalid_memory ?
                                       AsicStatus::failed : AsicStatus::quarantined);
    }

    // gmc_v11_0_flush_gpu_tlb: with the MES ring ready every engine-17 flush goes
    // through amdgpu_mes_reg_write_reg_wait.
    if (report_.mes == Mes11Status::scheduler_ready) {
        gmc_.set_firmware_invalidation({this, mes_misc});
    }

    return AsicStatus::completed;
}

AsicStatus AsicInit::start_queues(const AsicInputs& inputs,
                                  memory::GcRegisterBase registers) noexcept {
    report_.queues = queues_.initialize(registers, io_, inputs.doorbells, nbio_, gtt_window(),
                                        {gfx11_default_compute_rings}, {gfx11_usec_timeout},
                                        {this, confirm});

    if (report_.queues != Gfx11QueuesStatus::ready) {
        return fail(AsicStep::kernel_queues, AsicStatus::failed);
    }

    Gfx11QueueMemory memory{};
    const auto slot = [](Owned base, uint32_t index) {
        return static_cast<Owned>(static_cast<uint32_t>(base) + index);
    };
    bool allocated = allocate(Owned::gfx_ring, gfx11_ring_bytes, gfx11_queue_alignment, true) &&
        allocate(Owned::gfx_mqd, memory::page_bytes, gfx11_queue_alignment, true) &&
        allocate(Owned::compute_eop, gfx11_default_compute_rings * gfx11_mec_hpd_bytes,
                 gfx11_queue_alignment, true) &&
        allocate(Owned::queue_writeback, gfx11_writeback_bytes, gfx11_queue_alignment, true);

    for (uint32_t ring = 0; allocated && ring < gfx11_default_compute_rings; ++ring) {
        allocated = allocate(slot(Owned::compute_ring0, ring), gfx11_ring_bytes,
                             gfx11_queue_alignment, true) &&
            allocate(slot(Owned::compute_mqd0, ring), memory::page_bytes, gfx11_queue_alignment, true);
    }

    if (!allocated) {
        return fail(AsicStep::kernel_queues, AsicStatus::failed);
    }

    memory.gfx_ring = handle(Owned::gfx_ring);
    memory.gfx_mqd = handle(Owned::gfx_mqd);
    memory.compute_eop = handle(Owned::compute_eop);
    memory.writeback = handle(Owned::queue_writeback);

    for (uint32_t ring = 0; ring < gfx11_default_compute_rings; ++ring) {
        memory.compute_ring[ring] = handle(slot(Owned::compute_ring0, ring));
        memory.compute_mqd[ring] = handle(slot(Owned::compute_mqd0, ring));
    }

    // gfx_v11_0_cp_resume after amdgpu_mes_kiq_hw_init: kcq_resume, async gfx
    // ring resume, then the gfx and compute ring tests.
    report_.queues = queues_.resume(gtt_, memory, mes_);

    if (report_.queues != Gfx11QueuesStatus::running) {
        return fail(AsicStep::kernel_queues, report_.queues == Gfx11QueuesStatus::invalid_memory ?
                                             AsicStatus::failed : AsicStatus::quarantined);
    }

    return AsicStatus::completed;
}

AsicStatus AsicInit::start_sdma(const AsicInputs& inputs, Sdma6Registers registers,
                                NbioRegisters nbio) noexcept {
    report_.sdma = sdma_.initialize(registers, nbio, io_, nbio_, gtt_window(),
                                    {sdma6_default_ring_bytes}, inputs.doorbells, {this, confirm});

    if (report_.sdma != Sdma6Status::ready) {
        return fail(AsicStep::sdma, AsicStatus::failed);
    }

    if (!allocate(Owned::sdma_ring0, sdma6_default_ring_bytes, sdma6_ring_alignment, true) ||
        !allocate(Owned::sdma_ring1, sdma6_default_ring_bytes, sdma6_ring_alignment, true) ||
        !allocate(Owned::sdma_writeback, memory::page_bytes, memory::page_bytes, true)) {
        return fail(AsicStep::sdma, AsicStatus::failed);
    }

    // sdma_v6_0_hw_init: start, then both kernel ring tests.
    report_.sdma = sdma_.hw_init(gtt_, handle(Owned::sdma_ring0), handle(Owned::sdma_ring1),
                                 handle(Owned::sdma_writeback));

    if (report_.sdma != Sdma6Status::started) {
        return fail(AsicStep::sdma, report_.sdma == Sdma6Status::invalid_memory ?
                                    AsicStatus::failed : AsicStatus::quarantined);
    }

    return AsicStatus::completed;
}

AsicStatus AsicInit::start_smu(const AsicInputs& inputs, SmuRegisters registers,
                               ThmRegisters thm) noexcept {
    report_.smu = smu_.initialize(registers, io_, nbio_, policy_.smu, {this, confirm});

    if (report_.smu != SmuStatus::ready) {
        return fail(AsicStep::smu_start, AsicStatus::failed);
    }

    report_.smu = smu_.start();

    if (report_.smu != SmuStatus::ready) {
        return fail(AsicStep::smu_start, AsicStatus::failed);
    }

    if (!allocate(Owned::smu_driver, smu13_driver_table_bytes, memory::page_bytes, true) ||
        !allocate(Owned::smu_tool, smu13_tool_table_bytes, memory::page_bytes, false)) {
        return fail(AsicStep::smu_tables, AsicStatus::failed);
    }

    report_.smu = smu_.set_tables(*manager_, handle(Owned::smu_driver),
                                  handle(Owned::smu_tool), report_.vram);

    if (report_.smu != SmuStatus::completed) {
        return fail(AsicStep::smu_tables, AsicStatus::failed);
    }

    // smu_smc_hw_setup order. The memory pool is zero-sized by default (skipped);
    // SCPM makes PSP responsible for the PPTable and the allowed-feature mask.
    report_.smu = smu_.setup_power_play(report_.scpm_enabled, inputs.smu_board_table);

    if (report_.smu != SmuStatus::completed) {
        return fail(AsicStep::smu_pptable, AsicStatus::failed);
    }

    report_.smu = smu_.run_btc();

    if (report_.smu != SmuStatus::completed) {
        return fail(AsicStep::smu_btc, AsicStatus::failed);
    }

    report_.smu = smu_.set_allowed_features(report_.scpm_enabled);

    if (report_.smu != SmuStatus::completed) {
        return fail(AsicStep::smu_allowed_features, AsicStatus::failed);
    }

    report_.smu = smu_.enable_all_features();

    if (report_.smu != SmuStatus::completed) {
        return fail(AsicStep::smu_features, AsicStatus::failed);
    }

    // smu_set_default_dpm_table with smu_v13_0_get_vbios_bootup_values.
    SmuBootClocks boot{};

    if (inputs.atom != nullptr) {
        boot = {inputs.atom->bootup_socclk_10khz, inputs.atom->bootup_gfxclk_10khz,
                inputs.atom->bootup_uclk_10khz, inputs.atom->bootup_fclk_10khz,
                inputs.atom->bootup_vclk_10khz, inputs.atom->bootup_dclk_10khz,
                inputs.atom->bootup_dcefclk_10khz};
    }

    report_.smu = smu_.set_default_dpm_tables(boot);

    if (report_.smu != SmuStatus::completed) {
        return fail(AsicStep::smu_dpm_tables, AsicStatus::failed);
    }

    report_.smu = smu_.update_pcie(inputs.pcie);

    if (report_.smu != SmuStatus::completed) {
        return fail(AsicStep::smu_pcie, AsicStatus::failed);
    }

    // smu_get_thermal_temperature_range is folded into enable_thermal_alert.
    report_.smu = smu_.enable_thermal_alert(thm);

    if (report_.smu != SmuStatus::completed) {
        return fail(AsicStep::smu_thermal, AsicStatus::failed);
    }

    return AsicStatus::completed;
}

// The run/resume input contract: complete catalog, register and GTT callbacks,
// a valid PSP 13.0 SOS container and every discovered register token, all
// checked before any hardware access.
// amdgpu_device_ip_late_init up to the MES block: soc21_common_late_init's
// self-ring doorbell aperture at the doorbell BAR, then smu_late_init (after the
// common/GMC/IH/PSP late hooks, before any clock or power gating) with
// adev->pm.fw_version from the SMC image header.
AsicStatus AsicInit::early_late_init(const AsicInputs& inputs) noexcept {
    if (inputs.doorbell_bus_base != 0 &&
        nbio_.enable_doorbell_selfring(inputs.doorbell_bus_base) != NbioStatus::ready) {
        return fail(AsicStep::late_init, AsicStatus::quarantined);
    }

    uint32_t smc_image_version = 0;

    if (!smu13_firmware_image_version(inputs.catalog->file(UcodeSource::smu), smc_image_version)) {
        return fail(AsicStep::smu_late_init, AsicStatus::invalid_input);
    }

    report_.smu = smu_.late_init({inputs.ac_power, smc_image_version});

    if (report_.smu != SmuStatus::completed) {
        return fail(AsicStep::smu_late_init, AsicStatus::failed);
    }

    return AsicStatus::completed;
}

AsicStatus AsicInit::validate_inputs(const AsicInputs& inputs, Discovered& tokens,
                                     FirmwareImage& sos) noexcept {
    const UcodeSource required[]{UcodeSource::pfp, UcodeSource::me, UcodeSource::mec,
        UcodeSource::rlc, UcodeSource::mes, UcodeSource::mes_kiq, UcodeSource::imu,
        UcodeSource::sdma, UcodeSource::smu};
    bool complete_catalog = inputs.catalog != nullptr &&
        (!policy_.load_dmcub || inputs.catalog->has(UcodeSource::dmcub));

    for (const auto source : required) {
        complete_catalog = complete_catalog && inputs.catalog->has(source);
    }

    if (!register_io_valid(inputs.io) || inputs.vram_manager == nullptr || !complete_catalog ||
        !gtt_platform_valid(inputs.gtt) ||
        validate_firmware(inputs.sos, FirmwareKind::psp_13_0, sos) != FirmwareStatus::success) {
        return fail(AsicStep::inputs, AsicStatus::invalid_input);
    }

    const uint32_t aperture = inputs.io.aperture_bytes;
    report_.discovery = validate_psp_discovery(inputs.discovery, aperture, tokens.psp);

    if (report_.discovery == DiscoveryStatus::validated) {
        report_.discovery = validate_nbio_discovery(inputs.discovery, aperture, tokens.nbio);
    }

    if (report_.discovery == DiscoveryStatus::validated) {
        report_.discovery = validate_smu_discovery(inputs.discovery, aperture, tokens.smu);
    }

    if (report_.discovery == DiscoveryStatus::validated) {
        report_.discovery = validate_thm_discovery(inputs.discovery, aperture, tokens.thm);
    }

    if (report_.discovery == DiscoveryStatus::validated) {
        report_.discovery = validate_gmc_discovery(inputs.discovery, aperture, tokens.gmc);
    }

    if (report_.discovery == DiscoveryStatus::validated) {
        report_.discovery = validate_ih6_discovery(inputs.discovery, aperture, tokens.ih);
    }

    if (report_.discovery == DiscoveryStatus::validated) {
        report_.discovery = validate_sdma6_discovery(inputs.discovery, aperture, tokens.sdma);
    }

    if (report_.discovery == DiscoveryStatus::validated) {
        report_.discovery = validate_mes11_discovery(inputs.discovery, aperture, tokens.mes);
    }

    if (report_.discovery == DiscoveryStatus::validated &&
        memory::validate_gc_discovery(inputs.discovery, aperture, tokens.gc) !=
            memory::GfxhubStatus::discovery_validated) {
        report_.discovery = DiscoveryStatus::unsupported;
    }

    if (report_.discovery != DiscoveryStatus::validated) {
        return fail(AsicStep::discovery, AsicStatus::invalid_input);
    }

    return AsicStatus::completed;
}

AsicStatus AsicInit::run(const AsicInputs& inputs, AsicInitPolicy policy,
                         AsicRecovery recovery) noexcept {
    if (ran_) {
        return AsicStatus::busy;
    }

    report_ = {};
    policy_ = policy;
    FirmwareImage sos{};
    Discovered tokens{};
    const AsicStatus validated = validate_inputs(inputs, tokens, sos);

    if (validated != AsicStatus::completed) {
        return validated;
    }

    const PspRegisters& psp = tokens.psp;
    const NbioRegisters& nbio = tokens.nbio;
    const Ih6Registers& ih = tokens.ih;
    const memory::GcRegisterBase& gc = tokens.gc;
    // Hardware access starts here; a second run needs confirmed reset recovery.
    ran_ = true;
    gtt_platform_ = inputs.gtt;
    ih_platform_ = inputs.ih_platform;
    process_doorbells_ = inputs.doorbells;
    psp_registers_ = psp;
    nbio_registers_ = nbio;
    ih_registers_ = ih;
    bios_scratch_reg_ = inputs.atom != nullptr ? inputs.atom->bios_scratch_reg : 0;
    io_ = inputs.io;
    recovery_ = recovery;
    manager_ = inputs.vram_manager;
    report_.nbio = nbio_.initialize(nbio, io_);

    if (report_.nbio == NbioStatus::ready) {
        report_.nbio = nbio_.program_common();
    }

    if (report_.nbio != NbioStatus::ready) {
        return fail(AsicStep::nbio, AsicStatus::failed);
    }

    report_.nbio = nbio_.vram_location(report_.vram);

    if (report_.nbio != NbioStatus::ready) {
        return fail(AsicStep::vram_location, AsicStatus::failed);
    }

    if (read_psp_runtime_database(io_, report_.vram.bytes, report_.runtime) !=
        VramDiscoveryStatus::acquired) {
        return fail(AsicStep::runtime_database, AsicStatus::failed);
    }

    report_.training_requested = !report_.runtime.boot_config_valid ||
        (report_.runtime.boot_config & psp_boot_config_two_stage_training) != 0;
    report_.scpm_enabled = report_.runtime.scpm_valid && report_.runtime.scpm_status != 0;

    auto memory_status = start_gmc(inputs, tokens.gmc);

    if (memory_status != AsicStatus::completed) {
        return memory_status;
    }

    auto status = boot_psp(inputs, psp, ih, nbio);

    if (status != AsicStatus::completed) {
        return status;
    }

    report_.ring = ring_.initialize(psp, io_, nbio_, report_.vram, policy_.ring, {this, confirm});

    if (report_.ring != PspRingStatus::ready) {
        return fail(AsicStep::psp_ring, AsicStatus::failed);
    }

    status = load_firmware(inputs, sos);

    if (status != AsicStatus::completed) {
        return status;
    }

    status = start_smu(inputs, tokens.smu, tokens.thm);

    if (status != AsicStatus::completed) {
        return status;
    }

    status = start_gfx(inputs, gc);

    if (status != AsicStatus::completed) {
        return status;
    }

    status = start_mes_kiq(inputs, tokens.mes);

    if (status != AsicStatus::completed) {
        return status;
    }

    // KGQ/KCQ mapping needs the scheduler; firmware below 0x47 only starts it in
    // the MES IP block, where Linux would map through the KIQ instead (refused).
    status = start_queues(inputs, gc);

    if (status != AsicStatus::completed) {
        return status;
    }

    status = start_sdma(inputs, tokens.sdma, nbio);

    if (status != AsicStatus::completed) {
        return status;
    }

    // MES IP block hw_init: immediate when kiq_hw_init already started the scheduler.
    report_.mes = mes_.hw_init();

    if (report_.mes != Mes11Status::scheduler_ready) {
        return fail(AsicStep::mes, AsicStatus::quarantined);
    }

    gmc_.set_firmware_invalidation({this, mes_misc});

    status = early_late_init(inputs);

    if (status != AsicStatus::completed) {
        return status;
    }

    // mes_v11_0_late_init (MES is the last late_init block, before gating).
    run_self_test(nbio);
    status = late_gating_and_ib_tests(inputs, gc, nbio);

    if (status != AsicStatus::completed) {
        return status;
    }

    // Process-VM TLB invalidation (gmc_v11_0_flush_gpu_tlb_pasid) through MES;
    // with GFXOFF allowed the GFXHUB register path is refused, as Linux avoids it.
    if (vm_invalidator_.initialize(gmc_, ih, io_, {this, mes_misc},
                                   gating_.report().gfx_off_allowed) != GpuVmStatus::ready) {
        return fail(AsicStep::complete, AsicStatus::failed);
    }

    report_.step = AsicStep::complete;
    report_.status = AsicStatus::completed;
    return report_.status;
}

bool AsicInit::user_queue_config(PasidAllocator& pasids, UserQueueConfig& output) noexcept {
    output = {};
    if (!ran_ || suspended_ || report_.shut_down || report_.status != AsicStatus::completed ||
        manager_ == nullptr || !mes_.report().scheduler_ready || store_.store == nullptr ||
        !doorbell_io_valid(process_doorbells_)) {
        return false;
    }
    output = {&mes_, &pasids, &vm_invalidator_, &nbio_, manager_, &gtt_, report_.vram,
              gmc_.gart(), nbio_registers_, process_doorbells_, store_, {}};
    return true;
}

AsicStatus AsicInit::enable_kernel_submission(FenceManager& fences, RingPointerStore store) noexcept {
    if (!ran_ || report_.step != AsicStep::complete || fences_ != nullptr) {
        return AsicStatus::busy;
    }

    // The GMC and OSSSYS tokens also enable process-VM jobs on the gfx ring.
    if (queues_.enable_submission(fences, {nbio_registers_, store, gmc_.registers(), ih_registers_}) !=
        Gfx11QueuesStatus::running) {
        return AsicStatus::failed;
    }

    fences_ = &fences;
    store_ = store;

    if (sdma_.enable_submission(fences, {store}) != Sdma6Status::started) {
        return AsicStatus::failed;
    }

    return AsicStatus::completed;
}

AsicStatus AsicInit::shutdown() noexcept {
    if (!ran_ || report_.step != AsicStep::complete || report_.shut_down) {
        return AsicStatus::busy;
    }
    // The native process owner must remove every MES process queue first.
    // Refuse before disabling engines rather than leaving live VMs half stopped.
    if (mes_.report().process_queues != 0) {
        return AsicStatus::busy;
    }

    report_.fini_failure = AsicStep::stopped;
    const auto stage = [this](bool stopped, AsicStep step) {
        if (!stopped && report_.fini_failure == AsicStep::stopped) {
            report_.fini_failure = step;
        }
    };

    // amdgpu_fence_driver_hw_fini: wait for every emitted fence (bounded by the
    // ring's job timeout), then stop kernel submission on both engines.
    if (fences_ != nullptr) {
        for (uint32_t ring = 0; ring < fence_max_rings; ++ring) {
            const auto state = fences_->ring_report(ring);

            if (state.state != FenceRingState::detached && state.sync_seq != state.last_seq) {
                stage(fences_->wait(ring, state.sync_seq, fence_drain_timeout_us) == FenceStatus::signalled,
                      AsicStep::fini_fences);
            }
        }

        stage(queues_.disable_submission() == Gfx11QueuesStatus::running, AsicStep::fini_fences);
        stage(sdma_.disable_submission() == Sdma6Status::started, AsicStep::fini_fences);
    }

    // amdgpu_device_ip_fini_early: set_pg_state(UNGATE) (DisallowGfxOff) and
    // set_cg_state(UNGATE) before any block's hw_fini.
    report_.gating = gating_.ungate(ih_, smu_);
    stage(report_.gating == ClockGatingStatus::ungated, AsicStep::fini_ungate);

    // amdgpu_device_smu_fini_early: SMU hw_fini (thermal alert off; 13.0.0 keeps
    // its DPM features) before every other block.
    report_.smu = smu_.hw_fini();
    stage(report_.smu == SmuStatus::completed, AsicStep::fini_smu);

    // mes_v11_0_hw_fini frees only the SR-IOV resource_1 on bare metal.
    stage(true, AsicStep::fini_mes);

    report_.sdma = sdma_.hw_fini();
    stage(report_.sdma == Sdma6Status::stopped, AsicStep::fini_sdma);

    // gfx_v11_0_hw_fini: disable_kgq/kcq through MES, amdgpu_mes_kiq_hw_fini,
    // cp_enable(false), gui idle interrupts off, gfxhub gart_disable.
    report_.queues = queues_.fini(mes_);
    stage(report_.queues == Gfx11QueuesStatus::stopped, AsicStep::fini_gfx);
    // The MES ring stops here: later GART flushes use the register path again.
    gmc_.set_firmware_invalidation({nullptr, nullptr});
    report_.mes = mes_.kiq_hw_fini();
    stage(report_.mes == Mes11Status::disabled, AsicStep::fini_gfx);
    report_.gfx = gfx_.cp_stop();
    stage(report_.gfx == Gfx11Status::stopped, AsicStep::fini_gfx);
    report_.gmc = gmc_.disable_hub(VmHub::gfxhub);
    stage(report_.gmc == GmcStatus::disabled, AsicStep::fini_gfx);

    // psp_hw_fini: RAS, DTM, HDCP TAs (RAP/secure display absent), then ASD.
    stage(apps_.terminate() == PspTaStatus::completed, AsicStep::fini_psp);

    if (report_.asd_session != 0) {
        report_.ring = ring_.unload_ta(report_.asd_session);
        stage(report_.ring == PspRingStatus::completed, AsicStep::fini_psp);
    }

    report_.ring = ring_.destroy_tmr();
    stage(report_.ring == PspRingStatus::completed, AsicStep::fini_psp);
    report_.ring = ring_.stop();
    stage(report_.ring == PspRingStatus::stopped, AsicStep::fini_psp);

    report_.ih = ih_.disable();
    stage(report_.ih == Ih6Status::disabled, AsicStep::fini_ih);

    report_.gmc = gmc_.disable_hub(VmHub::mmhub);
    stage(report_.gmc == GmcStatus::disabled, AsicStep::fini_gmc);

    // soc21_common_hw_fini.
    stage(nbio_.enable_doorbell_aperture(false) == NbioStatus::ready &&
          nbio_.disable_doorbell_selfring() == NbioStatus::ready, AsicStep::fini_common);

    report_.shut_down = true;
    report_.step = AsicStep::stopped;
    report_.status = report_.fini_failure == AsicStep::stopped ? AsicStatus::completed :
                                                                 AsicStatus::quarantined;
    return report_.status;
}

AsicStatus AsicInit::mode1_reset(ResetPlatform platform) noexcept {
    if (!ran_ || reset_done_ || suspended_ || platform.cache_pci_state == nullptr ||
        platform.clear_bus_master == nullptr || platform.restore_pci_state == nullptr) {
        return AsicStatus::busy;
    }

    return reset_asic(platform);
}

// amdgpu_device_mode1_reset (soc21_asic_reset, MODE1 for MP1 13.0.0).
AsicStatus AsicInit::reset_asic(ResetPlatform platform) noexcept {
    constexpr uint32_t engine_hung = 0x20000000u;  // ATOM_S3_ASIC_GUI_ENGINE_HUNG
    constexpr uint32_t usec_timeout = adev_usec_timeout;
    const auto scratch = [this](bool hung) {
        uint32_t value = 0;

        // amdgpu_atombios_scratch_regs_engine_hung on BIOS_SCRATCH_3.
        return bios_scratch_reg_ == 0 ||
               (read_ip_register(io_, 0, bios_scratch_reg_ + 3, value) &&
                write_ip_register(io_, 0, bios_scratch_reg_ + 3,
                                  hung ? (value | engine_hung) : (value & ~engine_hung)));
    };

    if (!scratch(true) || !platform.cache_pci_state(platform.context) ||
        !platform.clear_bus_master(platform.context)) {
        return fail(AsicStep::reset_prepare, AsicStatus::quarantined);
    }

    report_.smu = smu_.mode1_reset();

    if (report_.smu != SmuStatus::completed) {
        return fail(AsicStep::reset_smu, AsicStatus::quarantined);
    }

    if (!platform.restore_pci_state(platform.context)) {
        return fail(AsicStep::reset_restore, AsicStatus::quarantined);
    }

    // psp_v13_0_wait_for_bootloader: psp_wait_for(C2PMSG_35 == 0x80000000) x 10.
    if (!wait_psp_bootloader(io_, psp_registers_)) {
        return fail(AsicStep::reset_bootloader, AsicStatus::quarantined);
    }

    // Wait for the ASIC to come out of reset: RCC_CONFIG_MEMSIZE readable.
    bool out_of_reset = false;

    for (uint32_t poll = 0; poll < usec_timeout; ++poll) {
        uint32_t memsize = 0;

        if (nbio_.read_memsize(memsize) && memsize != UINT32_MAX) {
            out_of_reset = true;
            break;
        }

        io_.delay_microseconds(io_.context, 1);
    }

    if (!out_of_reset || !scratch(false)) {
        return fail(AsicStep::reset_memsize, AsicStatus::quarantined);
    }

    reset_done_ = true;
    report_.step = AsicStep::reset_done;
    report_.status = AsicStatus::completed;
    return report_.status;
}

AsicStatus AsicInit::release_after_reset(void* external_context,
    bool (*retire_external)(void*) noexcept) noexcept {
    if (!ran_) {
        return AsicStatus::busy;
    }

    if (!confirm(this)) {
        return AsicStatus::quarantined;
    }

    reset_confirmed_ = true;
    if (retire_external != nullptr && !retire_external(external_context)) {
        reset_confirmed_ = false;
        return AsicStatus::quarantined;
    }
    gmc_.set_firmware_invalidation({nullptr, nullptr});
    boot_.release_after_reset();
    ih_.release_after_reset();
    gfx_.release_after_reset();
    sdma_.release_after_reset();
    queues_.release_after_reset();
    gating_.release_after_reset();
    selftest_vm_.release_after_reset();
    mes_.release_after_reset();
    apps_.release_after_reset();
    ring_.release_after_reset();
    smu_.release_after_reset();
    gmc_.release_after_reset();
    bool released = true;

    for (size_t index = 0; index < static_cast<size_t>(Owned::count); ++index) {
        if (!owned_[index]) {
            continue;
        }

        if (manager_for(static_cast<Owned>(index)).release(handles_[index]) != memory::Status::success) {
            released = false;
            continue;
        }

        owned_[index] = false;
        handles_[index] = {};
    }

    reset_confirmed_ = false;

    if (!released) {
        return AsicStatus::quarantined;
    }

    suspended_ = false;
    resuming_ = false;
    report_.suspended = false;
    gtt_ready_ = false;
    // Modules detached their fence rings above; then the manager itself.
    if (fences_ != nullptr && fences_->release_after_reset({this, confirm}) == FenceStatus::released) {
        fences_ = nullptr;
    }

    reset_done_ = false;
    ran_ = false;
    report_.status = AsicStatus::released;
    return report_.status;
}

AsicStatus AsicInit::restart_after_reset(const AsicInputs& inputs, AsicInitPolicy policy,
                                        AsicRecovery recovery) noexcept {
    if (ran_ || report_.status != AsicStatus::released) {
        return AsicStatus::busy;
    }

    // Retained GTT allocations are only meaningful with the same owners.
    if (inputs.vram_manager != manager_ || inputs.gtt.context != gtt_platform_.context ||
        inputs.gtt.allocate != gtt_platform_.allocate || inputs.gtt.release != gtt_platform_.release) {
        return AsicStatus::invalid_input;
    }

    // amdgpu_do_asic_reset: a full reset is followed by amdgpu_device_asic_init
    // (no need_post check) before amdgpu_device_ip_resume_phase1. A failure
    // leaves the status released, so the restart may be retried.
    const AsicStatus posted = repost(inputs);

    if (posted != AsicStatus::completed) {
        return posted;
    }

    restarting_ = true;
    const AsicStatus status = run(inputs, policy, recovery);
    restarting_ = false;

    if (status == AsicStatus::completed) {
        ++restarts_;
    }

    return status;
}

AsicStatus AsicInit::suspend(SuspendPlatform platform) noexcept {
    if (!ran_ || report_.step != AsicStep::complete || report_.shut_down || suspended_ ||
        reset_done_ || platform.reset.cache_pci_state == nullptr ||
        platform.reset.clear_bus_master == nullptr || platform.reset.restore_pci_state == nullptr) {
        return AsicStatus::busy;
    }

    // amdgpu_amdkfd_suspend evicts process queues before MES suspends; here their
    // owner removes them first, so MES holds kernel queues only.
    if (mes_.report().process_queues != 0) {
        report_.suspend_failure = AsicStep::suspend_mes;
        return AsicStatus::busy;
    }

    report_.suspend_failure = AsicStep::suspended;
    const auto stage = [this](bool done, AsicStep step) {
        if (!done && report_.suspend_failure == AsicStep::suspended) {
            report_.suspend_failure = step;
        }

        return done;
    };

    // amdgpu_device_prepare / amdgpu_device_evict_resources: TTM evicts unpinned
    // VRAM BOs only. Every sequencer VRAM buffer is pinned kernel memory whose
    // contents resume re-creates (GART table, PSP ring/firmware/TMR, SMU tables,
    // clear state, TA buffers, self-test tables), so nothing is copied out.

    // amdgpu_device_ip_suspend_phase1: set_pg_state/set_cg_state(UNGATE), then
    // amdgpu_dpm_set_df_cstate(DF_CSTATE_DISALLOW), whose failure Linux only warns
    // about (a PMFW refusal is accepted, a lost mailbox is not). No DCE block exists.
    report_.step = AsicStep::suspend_ungate;
    report_.gating = gating_.ungate(ih_, smu_);
    stage(report_.gating == ClockGatingStatus::ungated, AsicStep::suspend_ungate);
    report_.df_cstate = smu_.send(smu13_message::df_cstate_control, df_cstate_disallow, nullptr);
    stage(report_.df_cstate == SmuStatus::completed || report_.df_cstate == SmuStatus::command_failed,
          AsicStep::suspend_df_cstate);

    // amdgpu_fence_driver_hw_fini: amdgpu_fence_wait_empty on every ring, then the
    // fence interrupts and fallback timers stop (the rings detach). Linux waits
    // without bound; a fence missing the job timeout aborts here, before any block
    // suspends, with DF C-states and gating restored and the device still running.
    if (fences_ != nullptr) {
        FenceStatus waited = FenceStatus::signalled;

        for (uint32_t ring = 0; waited == FenceStatus::signalled && ring < fence_max_rings; ++ring) {
            const auto state = fences_->ring_report(ring);

            if (state.state != FenceRingState::detached && state.sync_seq != state.last_seq) {
                waited = fences_->wait(ring, state.sync_seq, fence_drain_timeout_us);
            }
        }

        if (waited != FenceStatus::signalled) {
            report_.suspend_failure = AsicStep::suspend_fences;
            smu_.send(smu13_message::df_cstate_control, df_cstate_allow, nullptr);
            report_.gating = gating_.gate(ih_, smu_);
            report_.step = AsicStep::complete;
            report_.status = waited == FenceStatus::timed_out ? AsicStatus::busy :
                                                                AsicStatus::quarantined;
            return report_.status;
        }

        report_.step = AsicStep::suspend_fences;
        stage(queues_.disable_submission() == Gfx11QueuesStatus::running, AsicStep::suspend_fences);
        stage(sdma_.disable_submission() == Sdma6Status::started, AsicStep::suspend_fences);
    }

    // amdgpu_device_ip_suspend_phase2 in reverse block order; Linux continues
    // after a failed block. mes_v11_0_suspend: amdgpu_mes_suspend, then hw_fini
    // (no bare-metal effect).
    report_.step = AsicStep::suspend_mes;
    report_.mes = mes_.suspend_all();
    stage(report_.mes == Mes11Status::completed, AsicStep::suspend_mes);

    // sdma_v6_0_suspend = hw_fini.
    report_.step = AsicStep::suspend_sdma;
    report_.sdma = sdma_.hw_fini();
    stage(report_.sdma == Sdma6Status::stopped, AsicStep::suspend_sdma);

    // gfx_v11_0_suspend = hw_fini: KGQ/KCQ unmap through MES, amdgpu_mes_kiq_hw_fini,
    // cp_enable(false), GUI idle interrupt off, GFXHUB gart_disable. The MES ring
    // stops here, so later GART flushes would use the register path.
    report_.step = AsicStep::suspend_gfx;
    report_.queues = queues_.fini(mes_);
    stage(report_.queues == Gfx11QueuesStatus::stopped, AsicStep::suspend_gfx);
    gmc_.set_firmware_invalidation({nullptr, nullptr});
    report_.mes = mes_.kiq_hw_fini();
    stage(report_.mes == Mes11Status::disabled, AsicStep::suspend_gfx);
    report_.gfx = gfx_.cp_stop();
    stage(report_.gfx == Gfx11Status::stopped, AsicStep::suspend_gfx);
    report_.gmc = gmc_.disable_hub(VmHub::gfxhub);
    stage(report_.gmc == GmcStatus::disabled, AsicStep::suspend_gfx);

    // smu_suspend: smu_smc_hw_cleanup (thermal alert off; smu_disable_dpms returns
    // early for 13.0.0), no set_gfx_cgpg or gfxoff entry-count hook on 13.0.0,
    // then amdgpu_dpm_set_mp1_state(PP_MP1_STATE_NONE), which 13.0.0 ignores.
    report_.step = AsicStep::suspend_smu;
    report_.smu = smu_.hw_fini();
    stage(report_.smu == SmuStatus::completed, AsicStep::suspend_smu);

    // psp_suspend: RAS, HDCP, DTM terminate, ASD terminate, TMR unload (the TMR
    // allocation stays: psp_tmr_init reuses tmr_bo on resume), ring stop; the
    // first failure ends psp_suspend ("goto out").
    report_.step = AsicStep::suspend_psp;
    bool psp = stage(apps_.suspend() == PspTaStatus::completed, AsicStep::suspend_psp);

    if (psp && report_.asd_session != 0) {
        report_.ring = ring_.unload_ta(report_.asd_session);
        psp = stage(report_.ring == PspRingStatus::completed, AsicStep::suspend_psp);
    }

    if (psp) {
        report_.ring = ring_.destroy_tmr();
        psp = stage(report_.ring == PspRingStatus::completed, AsicStep::suspend_psp);
    }

    if (psp) {
        report_.ring = ring_.stop();
        stage(report_.ring == PspRingStatus::stopped, AsicStep::suspend_psp);
    }

    // ih_v6_0_suspend = hw_fini, once the interrupt owner stopped reading the rings.
    report_.step = AsicStep::suspend_ih;

    if (stage(platform.ih_stopping == nullptr || platform.ih_stopping(platform.context),
              AsicStep::suspend_ih)) {
        report_.ih = ih_.disable();
        stage(report_.ih == Ih6Status::disabled, AsicStep::suspend_ih);
    }

    // gmc_v11_0_suspend = hw_fini: MMHUB gart_disable.
    report_.step = AsicStep::suspend_gmc;
    report_.gmc = gmc_.disable_hub(VmHub::mmhub);
    stage(report_.gmc == GmcStatus::disabled, AsicStep::suspend_gmc);

    // soc21_common_suspend = hw_fini: doorbell and self-ring apertures off.
    report_.step = AsicStep::suspend_common;
    stage(nbio_.enable_doorbell_aperture(false) == NbioStatus::ready &&
          nbio_.disable_doorbell_selfring() == NbioStatus::ready, AsicStep::suspend_common);

    // amdgpu_dpm_notify_rlc_state(false): SMU 13.0.0 has no notify_rlc_state hook.
    // amdgpu_pmops_suspend_noirq: amdgpu_acpi_should_gpu_reset is true for a dGPU
    // entering S3 or S4, so amdgpu_asic_reset runs soc21's MODE1 reset.
    stage(reset_asic(platform.reset) == AsicStatus::completed, AsicStep::suspend_reset);
    ++report_.suspends;
    suspended_ = report_.suspend_failure == AsicStep::suspended;
    report_.suspended = suspended_;
    report_.step = AsicStep::suspended;
    report_.status = suspended_ ? AsicStatus::completed : AsicStatus::quarantined;
    return report_.status;
}

// Under the suspend reset proof every protocol owner that resume re-runs from its
// hw_init forgets the pre-suspend hardware and its leases (never the sequencer's
// allocations). GMC, IH, SDMA and the kernel queues keep their suspended state:
// resume continues them as Linux resumes those blocks.
void AsicInit::forget_reset_hardware() noexcept {
    gmc_.set_firmware_invalidation({nullptr, nullptr});
    boot_.release_after_reset();
    apps_.release_after_reset();
    ring_.release_after_reset();
    smu_.release_after_reset();
    gfx_.release_after_reset();
    mes_.release_after_reset();
    selftest_vm_.release_after_reset();
    gating_.release_after_reset();
}

// amdgpu_gtt_mgr_recover / amdgpu_ttm_recover_gart: every live GTT allocation
// bound again at its GART address into the current table (resume and the
// restart after a destructive reset). A partial bind leaves PTEs uncertain.
AsicStatus AsicInit::rebind_gtt_pages(AsicStep step) noexcept {
    report_.gart_rebound_allocations = 0;
    report_.gart_rebound_pages = 0;

    for (const auto& entry : gtt_pages_) {
        if (entry.gart == 0) {
            continue;
        }

        if (!bind_gtt(entry.gart, entry.bus, entry.bytes, entry.access)) {
            return fail(step, AsicStatus::quarantined);
        }

        ++report_.gart_rebound_allocations;
        report_.gart_rebound_pages +=
            static_cast<uint32_t>((entry.bytes + memory::page_bytes - 1) / memory::page_bytes);
    }

    return AsicStatus::completed;
}

// gmc_v11_0_resume -> gmc_v11_0_hw_init -> gart_enable: amdgpu_gtt_mgr_recover
// (every live GTT allocation rebound at its GART address) on a refilled table,
// then MMHUB gart_enable, HDP flush, fault defaults and the VMID0 flush.
AsicStatus AsicInit::resume_gmc() noexcept {
    report_.gmc = gmc_.reset_gart_table();

    if (report_.gmc != GmcStatus::prepared) {
        return fail(AsicStep::resume_gmc, report_.gmc == GmcStatus::mmio_failure ?
                                          AsicStatus::quarantined : AsicStatus::failed);
    }

    const AsicStatus rebound = rebind_gtt_pages(AsicStep::resume_gmc);

    if (rebound != AsicStatus::completed) {
        return rebound;
    }

    report_.gmc = gmc_.enable_mmhub();

    if (report_.gmc != GmcStatus::enabled) {
        return fail(AsicStep::resume_gmc, AsicStatus::quarantined);
    }

    return AsicStatus::completed;
}

// psp_resume: psp_mem_training(PSP_MEM_TRAIN_RESUME) when training is enabled,
// psp_hw_start (bootloader into psp->fw_pri_buf, KM ring, TOC and the kept TMR,
// PMFW, SETUP_TMR), psp_load_non_psp_fw, psp_asd_initialize, psp_rl_load and the
// RAS/HDCP/DTM TAs: load_firmware with every allocation reused.
AsicStatus AsicInit::resume_psp(const AsicInputs& inputs, const Discovered& tokens,
                                const FirmwareImage& sos) noexcept {
    if (report_.training_requested) {
        if (inputs.atom == nullptr) {
            return fail(AsicStep::resume_training, AsicStatus::unsupported);
        }

        if ((inputs.atom->capability & atom_capability_two_stage_training) != 0) {
            const auto trained = run_psp_resume_training(tokens.psp, io_, nbio_, report_.vram.bytes,
                inputs.atom->reserved_fb_bytes, inputs.training, training_data_, report_.training);

            // An alive SOS contradicts the suspend reset that preceded this resume.
            if (trained == PspTrainingStatus::skipped_sos_alive) {
                return fail(AsicStep::resume_training, AsicStatus::quarantined);
            }

            if (trained != PspTrainingStatus::completed) {
                return fail(AsicStep::resume_training, trained == PspTrainingStatus::unavailable ?
                                                       AsicStatus::unsupported : AsicStatus::failed);
            }
        }
    }

    const PspBootIo boot_io{this, read32, write32, publish, delay, confirm};
    report_.boot = boot_.initialize(tokens.psp, boot_io, policy_.boot);
    report_.sos_was_running = report_.boot == PspBootStatus::already_running;

    if (report_.boot != PspBootStatus::bootloader_ready) {
        return fail(AsicStep::psp_bootloader, report_.sos_was_running ? AsicStatus::quarantined :
                                                                        AsicStatus::failed);
    }

    const AsicStatus booted = load_sos(inputs, handle(Owned::psp_firmware));

    if (booted != AsicStatus::completed) {
        return booted;
    }

    report_.ring = ring_.initialize(tokens.psp, io_, nbio_, report_.vram, policy_.ring, {this, confirm});

    if (report_.ring != PspRingStatus::ready) {
        return fail(AsicStep::psp_ring, AsicStatus::failed);
    }

    report_.loaded_ucode = 0;
    report_.failed_ucode_id = 0;
    report_.asd_session = 0;
    report_.rlc_autoload_started = false;
    return load_firmware(inputs, sos);
}

AsicStatus AsicInit::resume(const AsicInputs& inputs) noexcept {
    if (!ran_ || !suspended_ || !reset_done_ || report_.shut_down) {
        return AsicStatus::busy;
    }

    resuming_ = true;
    const AsicStatus status = resume_sequence(inputs);
    resuming_ = false;
    return status;
}

AsicStatus AsicInit::resume_sequence(const AsicInputs& inputs) noexcept {
    FirmwareImage sos{};
    Discovered tokens{};

    // The same resident inputs as run: anything else would rebind live memory.
    if (inputs.vram_manager != manager_ || inputs.gtt.context != gtt_platform_.context ||
        inputs.gtt.allocate != gtt_platform_.allocate || inputs.gtt.release != gtt_platform_.release) {
        return fail(AsicStep::inputs, AsicStatus::invalid_input);
    }

    const AsicStatus validated = validate_inputs(inputs, tokens, sos);

    if (validated != AsicStatus::completed) {
        return validated;
    }

    // amdgpu_device_resume: "post card" when amdgpu_device_need_post, which on
    // Navi31 is amdgpu_atombios_scratch_need_asic_init (has_hw_reset is never set
    // in v6.12), then amdgpu_device_asic_init. Linux only logs a failed asic_init
    // and resumes anyway; here a failed repost stops with the ASIC still
    // suspended under its reset proof, so resume may be retried.
    bool posted = false;

    if (bios_scratch_reg_ == 0) {
        return fail(AsicStep::resume_post, AsicStatus::unsupported);
    }

    if (!read_posted(inputs.io, bios_scratch_reg_, posted)) {
        return fail(AsicStep::resume_post, AsicStatus::failed);
    }

    report_.posted = posted;

    if (!posted) {
        // Without an ATOM image and its firmwareinfo nothing can re-POST the
        // card: refuse before any write.
        if (inputs.atom == nullptr || inputs.atom_rom.size() == 0) {
            return fail(AsicStep::resume_post, AsicStatus::unsupported);
        }

        const AsicStatus reposted = repost(inputs);
        report_.posted = repost_report_.posted;

        if (reposted != AsicStatus::completed) {
            return fail(repost_report_.step, reposted);
        }
    }

    // Hardware programming starts: a failure from here on is not retryable.
    io_ = inputs.io;
    ih_platform_ = inputs.ih_platform;
    forget_reset_hardware();
    reset_done_ = false;
    suspended_ = false;
    report_.suspended = false;

    // amdgpu_device_ip_resume_phase1: soc21_common_resume (hw_init: strap, HDP
    // remap, doorbell aperture), with the MC placement unchanged.
    VramLocation vram{};
    report_.nbio = nbio_.initialize(tokens.nbio, io_);

    if (report_.nbio == NbioStatus::ready) {
        report_.nbio = nbio_.program_common();
    }

    if (report_.nbio == NbioStatus::ready) {
        report_.nbio = nbio_.vram_location(vram);
    }

    if (report_.nbio != NbioStatus::ready || vram.mc_start != report_.vram.mc_start ||
        vram.mc_end != report_.vram.mc_end || vram.bytes != report_.vram.bytes ||
        vram.physical_offset != report_.vram.physical_offset) {
        return fail(AsicStep::resume_common, AsicStatus::failed);
    }

    AsicStatus status = resume_gmc();

    if (status != AsicStatus::completed) {
        return status;
    }

    // ih_v6_0_resume = hw_init on the kept rings; the interrupt owner rebinds.
    report_.ih = ih_.resume();

    if (report_.ih != Ih6Status::enabled) {
        return fail(AsicStep::resume_ih, AsicStatus::quarantined);
    }

    if (inputs.interrupt_ready.ih_ready != nullptr &&
        !inputs.interrupt_ready.ih_ready(inputs.interrupt_ready.context, ih_)) {
        return fail(AsicStep::resume_ih, AsicStatus::failed);
    }

    // amdgpu_device_fw_loading: psp_resume.
    status = resume_psp(inputs, tokens, sos);

    if (status != AsicStatus::completed) {
        return status;
    }

    // amdgpu_device_ip_resume_phase2: smu_resume (start_smc_engine, smc_hw_setup),
    // gfx_v11_0_resume (hw_init with the MES KIQ and the in_suspend KCQ/KGQ path),
    // sdma_v6_0_resume, mes_v11_0_resume (hw_init, amdgpu_mes_resume).
    status = start_smu(inputs, tokens.smu, tokens.thm);

    if (status == AsicStatus::completed) {
        status = start_gfx(inputs, tokens.gc);
    }

    if (status == AsicStatus::completed) {
        status = start_mes_kiq(inputs, tokens.mes);
    }

    if (status != AsicStatus::completed) {
        return status;
    }

    report_.queues = queues_.resume_from_suspend(mes_);

    if (report_.queues != Gfx11QueuesStatus::running) {
        return fail(AsicStep::kernel_queues, AsicStatus::quarantined);
    }

    report_.sdma = sdma_.resume();

    if (report_.sdma != Sdma6Status::started) {
        return fail(AsicStep::sdma, AsicStatus::quarantined);
    }

    report_.mes = mes_.hw_init();

    if (report_.mes != Mes11Status::scheduler_ready) {
        return fail(AsicStep::mes, AsicStatus::quarantined);
    }

    gmc_.set_firmware_invalidation({this, mes_misc});
    report_.mes = mes_.resume_all();

    if (report_.mes != Mes11Status::completed) {
        return fail(AsicStep::resume_mes, AsicStatus::quarantined);
    }

    // amdgpu_device_ip_late_init (mes_v11_0_late_init skips its self-test while
    // in_suspend), CG/PG gate, then the delayed-work IB tests the resume flushes.
    status = early_late_init(inputs);

    if (status == AsicStatus::completed) {
        status = late_gating_and_ib_tests(inputs, tokens.gc, tokens.nbio);
    }

    if (status != AsicStatus::completed) {
        return status;
    }

    // amdgpu_device_resume tail: amdgpu_mes_self_test (result ignored).
    run_self_test(tokens.nbio);

    // amdgpu_fence_driver_hw_init: the kernel rings rejoin the same FenceManager;
    // their fence sequences continue where the suspend left them.
    if (fences_ != nullptr &&
        (queues_.enable_submission(*fences_, {nbio_registers_, store_}) != Gfx11QueuesStatus::running ||
         sdma_.enable_submission(*fences_, {store_}) != Sdma6Status::started)) {
        return fail(AsicStep::resume_submission, AsicStatus::failed);
    }

    ++report_.resumes;
    report_.step = AsicStep::complete;
    report_.status = AsicStatus::completed;
    return report_.status;
}

AsicStatus AsicInit::repost(const AsicInputs& inputs) noexcept {
    // A live ASIC owned by run is never re-POSTed underneath its protocols.
    if (ran_ && !reset_done_) {
        return AsicStatus::busy;
    }

    ++repost_report_.attempts;
    repost_report_.status = repost_sequence(inputs);

    if (repost_report_.status == AsicStatus::completed) {
        ++repost_report_.completed;
    }

    return repost_report_.status;
}

// amdgpu_device_asic_init: soc21_pre_asic_init is empty; GC 11.0.0 takes the
// amdgpu_psp_wait_for_bootloader + amdgpu_atomfirmware_asic_init(adev, true) path.
AsicStatus AsicInit::repost_sequence(const AsicInputs& inputs) noexcept {
    PspRegisters psp{};
    repost_report_.step = AsicStep::inputs;
    repost_report_.bootloader_ready = false;
    repost_report_.posted = false;
    repost_report_.atom = {};

    if (!register_io_valid(inputs.io) ||
        validate_psp_discovery(inputs.discovery, inputs.io.aperture_bytes, psp) != DiscoveryStatus::validated) {
        return AsicStatus::invalid_input;
    }

    // amdgpu_atomfirmware_asic_init needs firmware_info (else -EINVAL); the
    // confirmation needs bios_scratch_reg_startaddr.
    if (inputs.atom == nullptr || inputs.atom->bios_scratch_reg == 0 || inputs.atom_rom.size() == 0) {
        return AsicStatus::unsupported;
    }

    // amdgpu_atombios_init: amdgpu_atom_parse with the FB scratch that
    // amdgpu_atomfirmware_allocate_fb_scratch sizes from vram_usagebyfirmware.
    // Rebinding each call keeps the borrowed view current; the same ROM content
    // keeps its scratch and any quarantine.
    AtomVramUsage usage{};
    parse_atom_vram_usage(inputs.atom_rom, usage);

    if (atom_.bind(inputs.atom_rom, atom_allocate_fb_scratch(usage).scratch_bytes) != AtomStatus::completed) {
        return AsicStatus::unsupported;
    }

    repost_report_.scratch_bytes = atom_.scratch_bytes();
    repost_report_.step = AsicStep::repost_bootloader;
    repost_report_.bootloader_ready = wait_psp_bootloader(inputs.io, psp);
    repost_report_.step = AsicStep::repost_atom;
    const AtomStatus executed = atom_.asic_init(inputs.io, inputs.atom->bootup_gfxclk_10khz,
                                                inputs.atom->bootup_uclk_10khz, true, atom_default_policy);
    repost_report_.atom = atom_.report();

    switch (executed) {
    case AtomStatus::completed:
        break;
    case AtomStatus::invalid_argument:
    case AtomStatus::invalid_rom:
    case AtomStatus::absent_table:
    case AtomStatus::unsupported_revision:
        return AsicStatus::unsupported;
    case AtomStatus::malformed_table:
    case AtomStatus::register_range:
    case AtomStatus::call_depth:
    case AtomStatus::quarantined:
        return AsicStatus::quarantined;
    default:
        return AsicStatus::failed;
    }

    // The amdgpu_device_need_post check, now expected to report posted.
    bool posted = false;
    repost_report_.step = AsicStep::repost_confirm;

    if (!read_posted(inputs.io, inputs.atom->bios_scratch_reg, posted)) {
        return AsicStatus::failed;
    }

    repost_report_.posted = posted;
    return posted ? AsicStatus::completed : AsicStatus::failed;
}

} // namespace rx7900::hardware

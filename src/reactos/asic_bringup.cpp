/*
Module: rx7900.reactos.asic_bringup
Purpose: Supply native MMIO, timing, VRAM backing and firmware to AsicInit.
Created: 2026-10-21
Architecture: Thin NT/VideoPort adapter; protocol decisions stay in src/hardware.
*/
#include "asic_bringup.hpp"

namespace rx7900::reactos {
namespace {

constexpr ULONG stall_limit_microseconds = 50;

bool mmio_read(void* context, uint32_t offset, uint32_t& value) noexcept {
    return read_register(*static_cast<HardwareState*>(context), offset, value);
}

bool mmio_write(void* context, uint32_t offset, uint32_t value) noexcept {
    return write_register(*static_cast<HardwareState*>(context), offset, value);
}

// PASSIVE_LEVEL only. Short register polls busy-wait; longer documented waits
// (IFWI 1 ms, PSP 20 ms handshakes) yield. Both wait at least the request.
void wait(void*, uint32_t microseconds) noexcept {
    if (microseconds <= stall_limit_microseconds) {
        KeStallExecutionProcessor(microseconds);
        return;
    }

    LARGE_INTEGER interval{};
    interval.QuadPart = -10LL * static_cast<LONGLONG>(microseconds);
    KeDelayExecutionThread(KernelMode, FALSE, &interval);
}

bool bus_master(void* context) noexcept {
    const auto& bringup = *static_cast<const AsicBringup*>(context);
    return bringup.hardware != nullptr &&
           enable_bus_master(*bringup.hardware, bringup.extension);
}

// No native reset/power owner exists, so quiescence can never be confirmed and
// uncertain PSP/SMU memory stays reserved until the next system reset.
// Ring write pointers (Linux WDOORBELL32/64) through the mapped BAR2 aperture.
bool doorbell32(void* context, uint32_t index, uint32_t value) noexcept {
    auto& bringup = *static_cast<AsicBringup*>(context);
    return bringup.hardware != nullptr && write_doorbell32(*bringup.hardware, index, value);
}

bool doorbell64(void* context, uint32_t index, uint64_t value) noexcept {
    auto& bringup = *static_cast<AsicBringup*>(context);
    return bringup.hardware != nullptr && write_doorbell64(*bringup.hardware, index, value);
}

// GttPlatform over the device DMA pool: one contiguous common buffer each.
bool gtt_allocate(void* context, uint32_t bytes, uint8_t*& cpu, uint64_t& bus, void*& token) noexcept {
    auto& bringup = *static_cast<AsicBringup*>(context);

    for (auto& slot : bringup.gtt) {
        if (slot.slot != UINT32_MAX) {
            continue;
        }

        DmaHandle handle{};
        DmaSpan span{};

        if (!NT_SUCCESS(bringup.dma.allocate(bytes, memory::page_bytes, handle))) {
            return false;
        }

        if (!NT_SUCCESS(bringup.dma.describe(handle, span)) || span.byte_count < bytes) {
            bringup.dma.release(handle);
            return false;
        }

        slot = handle;
        cpu = span.cpu_bytes;
        bus = span.bus.value;
        token = &slot;
        return true;
    }

    return false;
}

bool gtt_release(void* context, void* token) noexcept {
    auto& bringup = *static_cast<AsicBringup*>(context);

    for (auto& slot : bringup.gtt) {
        if (&slot == token && slot.slot != UINT32_MAX) {
            if (!NT_SUCCESS(bringup.dma.release(slot))) {
                return false;
            }

            slot = {};
            return true;
        }
    }

    return false;
}

// ResetPlatform: PCI configuration space through VideoPort bus data.
// amdgpu_device_cache_pci_state: Linux pci_save_state.
bool cache_pci_state(void* context) noexcept {
    auto& bringup = *static_cast<AsicBringup*>(context);

    if (bringup.hardware == nullptr ||
        !NT_SUCCESS(save_state(*bringup.hardware, bringup.pci_workspace, bringup.pci_saved))) {
        return false;
    }

    bringup.pci_state_cached = true;
    return true;
}

bool clear_bus_master(void* context) noexcept {
    auto& bringup = *static_cast<AsicBringup*>(context);
    constexpr ULONG command_offset = 0x04;
    uint8_t bytes[2]{};

    if (bringup.hardware == nullptr ||
        VideoPortGetBusData(bringup.extension, PCIConfiguration, bringup.hardware->pci_slot, bytes,
                            command_offset, sizeof(bytes)) != sizeof(bytes)) {
        return false;
    }

    bytes[0] = static_cast<uint8_t>(bytes[0] & ~0x04u);
    return VideoPortSetBusData(bringup.extension, PCIConfiguration, bringup.hardware->pci_slot, bytes,
                               command_offset, sizeof(bytes)) == sizeof(bytes);
}

bool restore_pci_state(void* context) noexcept {
    auto& bringup = *static_cast<AsicBringup*>(context);

    // amdgpu_device_load_pci_state: Linux pci_restore_state.
    return bringup.hardware != nullptr && bringup.pci_state_cached &&
           NT_SUCCESS(restore_state(*bringup.hardware, bringup.pci_saved));
}

bool no_reset_owner(void*) noexcept {
    return false;
}

bool copy_vram(AsicBringup& bringup, uint64_t from, uint64_t to, uint64_t bytes) noexcept {
    if (bringup.hardware == nullptr) {
        return false;
    }

    for (uint64_t offset = 0; offset < bytes; offset += asic_vram_copy_chunk_bytes) {
        PUCHAR source = nullptr;
        PUCHAR destination = nullptr;

        if (map_vram_temporary(*bringup.hardware, bringup.extension, from + offset,
                               asic_vram_copy_chunk_bytes, source) != NO_ERROR) {
            return false;
        }

        if (map_vram_temporary(*bringup.hardware, bringup.extension, to + offset,
                               asic_vram_copy_chunk_bytes, destination) != NO_ERROR) {
            unmap_vram_temporary(bringup.extension, source);
            return false;
        }

        // Both mappings hold asic_vram_copy_chunk_bytes; indices stay inside them.
        auto* read = static_cast<PULONG>(static_cast<PVOID>(source));
        auto* written = static_cast<PULONG>(static_cast<PVOID>(destination));

        for (ULONG index = 0; index < asic_vram_copy_chunk_bytes / sizeof(ULONG); ++index) {
            VideoPortWriteRegisterUlong(written + index, VideoPortReadRegisterUlong(read + index));
        }

        unmap_vram_temporary(bringup.extension, destination);
        unmap_vram_temporary(bringup.extension, source);
    }

    return true;
}

bool save_encroached(void* context) noexcept {
    auto& bringup = *static_cast<AsicBringup*>(context);
    return bringup.training_backup_offset != 0 &&
           copy_vram(bringup, 0, bringup.training_backup_offset, hardware::psp_training_encroached_bytes);
}

bool restore_encroached(void* context) noexcept {
    auto& bringup = *static_cast<AsicBringup*>(context);
    return bringup.training_backup_offset != 0 &&
           copy_vram(bringup, bringup.training_backup_offset, 0, hardware::psp_training_encroached_bytes);
}

// The Linux reservation map (pre-OS scanout from DCN, ATOM vram_usagebyfirmware,
// top-of-VRAM firmware region, training C2P, BIST range) and the arena/backup
// placement inside the BAR0 window. Only reads: three DCN registers.
BringupResult place_arena(const HardwareState& hardware, const hardware::RegisterIo& io, ByteView rom,
                          const hardware::VramLocation& vram, AsicBringup& bringup) noexcept {
    hardware::AtomVramUsage usage{};
    const auto usage_status = hardware::parse_atom_vram_usage(rom, usage);
    hardware::DcnFbRegisters dcn;

    if (usage_status != hardware::AtomFirmwareStatus::success &&
        usage_status != hardware::AtomFirmwareStatus::absent) {
        return BringupResult::rom_unavailable;
    }

    // Navi31 always carries an unharvested DMU 3.2.0 (amdgpu_device_has_display_hardware).
    if (hardware::validate_dcn_fb_discovery(bringup.discovery.bytes(), hardware.register_bytes, dcn) !=
            hardware::DiscoveryStatus::validated ||
        !hardware::read_vbios_fb_bytes(io, dcn, bringup.vbios_fb_bytes)) {
        return BringupResult::discovery_unavailable;
    }

    const uint64_t window = hardware.vram_range_bytes;
    const uint64_t visible = window < vram.bytes ? window : vram.bytes;
    const hardware::VramReservationInputs inputs{vram.bytes, visible, bringup.atom.capability,
                                                 bringup.atom.reserved_fb_bytes, usage, true,
                                                 bringup.vbios_fb_bytes};

    if (visible == 0 ||
        hardware::build_vram_reservation_map(inputs, bringup.reservations) !=
            hardware::VramReservationStatus::built) {
        return BringupResult::vram_reserved;
    }

    static_assert(asic_vram_arena_offset >= hardware::psp_training_encroached_bytes,
                  "the preferred arena must lie above the BIST-encroached range");
    const hardware::VramPlacementRequest request{
        asic_vram_arena_offset, asic_vram_arena_bytes + hardware::psp_training_encroached_bytes,
        asic_vram_placement_alignment, visible};
    uint64_t offset = 0;
    bringup.placement = hardware::place_vram_region(bringup.reservations, request, offset);

    if ((bringup.placement != hardware::VramPlacementStatus::preferred &&
         bringup.placement != hardware::VramPlacementStatus::relocated) ||
        hardware::vram_range_reserved(bringup.reservations, offset, request.bytes)) {
        return BringupResult::vram_reserved;
    }

    bringup.arena_offset = offset;
    bringup.training_backup_offset = offset + asic_vram_arena_bytes;
    return BringupResult::completed;
}

// smu_smc_hw_setup encoding from the negotiated link of the owned function.
// amdgpu_device_get_pcie_info over the GPU and its real upstream bridges (the
// card's own AMD switch is skipped, as amdgpu_device_partner_bandwidth does),
// then the smu_smc_hw_setup encoding. Zero fields make AsicInit refuse the step.
hardware::SmuPcieLink pcie_link(const HardwareState& hardware, AsicBringup& bringup) noexcept {
    PcieInfoReport report{};
    PcieSmuLink link{};

    if (!NT_SUCCESS(query_pcie_smu_link(hardware, bringup.pci_workspace, report, link))) {
        return {0, 0};
    }

    return {link.gen, link.lane};
}

bool load_source(AsicBringup& bringup, hardware::UcodeSource source, uint8_t revision) noexcept {
    const size_t index = static_cast<size_t>(source);
    DriverBlob blob;
    NTSTATUS status = blob.load_ucode(source, revision, false);

    if (!NT_SUCCESS(status) && source == hardware::UcodeSource::mes) {
        status = blob.load_ucode(source, revision, true);
    }

    bringup.last_file_status = status;

    return NT_SUCCESS(status) && blob.detach(bringup.files[index]) &&
           bringup.catalog.add(source, bringup.files[index].bytes()) ==
               hardware::UcodeStatus::success;
}

// The AsicInputs of run and resume: the resident firmware, catalog, discovery,
// VRAM Manager, GTT and doorbell callbacks of this device. training selects the
// BIST save/restore callbacks; resume passes them only while both holds exist.
hardware::AsicInputs asic_inputs(HardwareState& hardware, ByteView rom,
                                 hardware::InterruptReady interrupt_ready, bool training,
                                 AsicBringup& bringup) noexcept {
    const hardware::RegisterIo io{&hardware, hardware.register_bytes, mmio_read, mmio_write, wait};
    // Absent or malformed smc_dpm_info stays empty; only non-SCPM boards need it.
    ByteView board(nullptr, 0);
    hardware::find_smu13_board_table(rom, board);
    const hardware::PspTrainingIo training_io =
        training ? hardware::PspTrainingIo{&bringup, save_encroached, restore_encroached} :
                   hardware::PspTrainingIo{};
    return {io, bringup.discovery.bytes(), bringup.sos.bytes(), &bringup.catalog, bringup.ta.bytes(),
            &bringup.vram, &bringup.atom, training_io, board, bringup.pcie, bringup.dummy_page_bus,
            // ih_v6_0 uses the IH doorbell on soc21.
            {&bringup, bus_master, doorbell32, no_reset_owner},
            // VideoPort on ReactOS delivers line-based interrupts.
            false,
            // adev->doorbell.base: the assigned BAR2 bus address.
            hardware.doorbell_range_bytes != 0 ?
                static_cast<uint64_t>(hardware.doorbell_range.QuadPart) : 0,
            {&bringup, hardware.doorbell_bytes / static_cast<uint32_t>(sizeof(uint32_t)),
                doorbell32, doorbell64},
            {&bringup, gtt_allocate, gtt_release},
            // ReactOS exposes no power-supply class; Linux then reports AC.
            true, interrupt_ready, rom};
}

// The AsicInitPolicy of run and restart: Linux timeouts, DMCUB when present.
hardware::AsicInitPolicy asic_policy(const AsicBringup& bringup) noexcept {
    return {{1000000, 1000000},
            {100000, 20000, 10},
            {2000000, 1},
            {100000, 2100000},
            bringup.catalog.has(hardware::UcodeSource::dmcub)};
}

// AsicInit::release_after_reset external retire: the 2D owner first (it keeps
// only system memory that the reset cannot touch), then the process service.
bool retire_reset_owners(void* context) noexcept {
    auto& bringup = *static_cast<AsicBringup*>(context);

    if (bringup.surface_reset_retire != nullptr && !bringup.surface_reset_retire(bringup.surface_owner)) {
        return false;
    }

    return bringup.process_reset_retire == nullptr || bringup.process_reset_retire(bringup.process_owner);
}

} // namespace

BringupResult run_asic_bringup(HardwareState& hardware, PVOID extension, ByteView rom,
                               hardware::InterruptReady interrupt_ready,
                               AsicBringup& bringup) noexcept {
    if (bringup.attempted || !hardware.acquired || KeGetCurrentIrql() != PASSIVE_LEVEL) {
        return BringupResult::not_attempted;
    }

    bringup.attempted = true;

    if (hardware::parse_atom_firmware_info(rom, bringup.atom) != hardware::AtomFirmwareStatus::success) {
        return BringupResult::rom_unavailable;
    }

    const hardware::RegisterIo io{&hardware, hardware.register_bytes, mmio_read, mmio_write, wait};

    if (hardware::read_vram_discovery(io, bringup.discovery, bringup.discovery_report) !=
        hardware::VramDiscoveryStatus::acquired) {
        return BringupResult::discovery_unavailable;
    }

    // Read-only placement probe: AsicInit repeats it after programming NBIO.
    hardware::NbioRegisters nbio_registers;
    hardware::Nbio probe;
    hardware::VramLocation vram{};

    if (hardware::validate_nbio_discovery(bringup.discovery.bytes(), hardware.register_bytes,
                                          nbio_registers) != hardware::DiscoveryStatus::validated ||
        probe.initialize(nbio_registers, io) != hardware::NbioStatus::ready ||
        probe.vram_location(vram) != hardware::NbioStatus::ready) {
        return BringupResult::discovery_unavailable;
    }

    // Arena and training backup avoid every Linux reservation (stolen pre-OS
    // memory, firmware usage, top-of-VRAM firmware region, training C2P, BIST range).
    const BringupResult placed = place_arena(hardware, io, rom, vram, bringup);

    if (placed != BringupResult::completed) {
        return placed;
    }

    // amdgpu_vram_mgr_init with the lifetime reservations, then the training
    // regions held for this run only, then the sequencer's CPU-visible pool.
    if (bringup.aperture.initialize(hardware, extension, vram.bytes, asic_vram_pool_bytes) != NO_ERROR) {
        return BringupResult::vram_window_unavailable;
    }

    hardware::VramRange lifetime[hardware::VramReservationMap::capacity]{};
    size_t lifetime_count = 0;

    for (uint32_t index = 0; index < bringup.reservations.count; ++index) {
        const auto& entry = bringup.reservations.entries[index];

        if (entry.lifetime) {
            lifetime[lifetime_count++] = {entry.offset, entry.offset + entry.bytes};
        }
    }

    if (bringup.vram_allocator.initialize({vram.bytes, bringup.aperture.visible_bytes()}, lifetime,
                                          lifetime_count) != hardware::VramStatus::success) {
        return BringupResult::vram_reserved;
    }

    const auto hold = hardware::vram_kernel_bo_flags(false);

    if (bringup.vram_allocator.allocate_at(0, hardware::psp_training_encroached_bytes, hold,
                                           bringup.training_hold) != hardware::VramStatus::success ||
        bringup.vram_allocator.allocate_at(bringup.training_backup_offset,
                                           hardware::psp_training_encroached_bytes, hold,
                                           bringup.backup_hold) != hardware::VramStatus::success ||
        bringup.visible_pool.initialize(bringup.vram_allocator, vram.mc_start,
                                        hardware::VramPool::cpu_visible, asic_vram_pool_bytes, 0,
                                        bringup.aperture.mapper()) != hardware::VramStatus::success ||
        bringup.vram.initialize(bringup.visible_pool.arena(), bringup.visible_pool.backend()) !=
            memory::Status::success) {
        return BringupResult::vram_reserved;
    }

    const hardware::UcodeSource required[]{hardware::UcodeSource::pfp, hardware::UcodeSource::me,
        hardware::UcodeSource::mec, hardware::UcodeSource::rlc, hardware::UcodeSource::mes,
        hardware::UcodeSource::mes_kiq, hardware::UcodeSource::imu, hardware::UcodeSource::sdma,
        hardware::UcodeSource::smu};
    const uint8_t revision = hardware.adapter.revision;

    for (const auto source : required) {
        if (!load_source(bringup, source, revision)) {
            return BringupResult::firmware_unavailable;
        }
    }

    // DMCUB and the PSP TA are optional exactly where Linux allows their absence.
    load_source(bringup, hardware::UcodeSource::dmcub, revision);
    DriverBlob sos;
    DriverBlob ta;
    bringup.last_file_status = sos.load_firmware(hardware::FirmwareKind::psp_13_0);

    if (!NT_SUCCESS(bringup.last_file_status) || !sos.detach(bringup.sos)) {
        return BringupResult::firmware_unavailable;
    }

    if (NT_SUCCESS(ta.load_psp_ta())) {
        ta.detach(bringup.ta);
    }

    const hardware::AsicInitPolicy policy = asic_policy(bringup);
    DmaSpan dummy{};

    if (!NT_SUCCESS(bringup.dma.initialize(extension, hardware)) ||
        !NT_SUCCESS(bringup.dma.allocate(memory::page_bytes, memory::page_bytes, bringup.dummy_page)) ||
        !NT_SUCCESS(bringup.dma.describe(bringup.dummy_page, dummy)) ||
        dummy.byte_count < memory::page_bytes) {
        return BringupResult::vram_window_unavailable;
    }

    const sdma::CommandBuffer zero(dummy.cpu_bytes, dummy.byte_count);

    for (ULONG offset = 0; offset < memory::page_bytes; offset += sizeof(uint32_t)) {
        zero.write_u32(offset, 0);
    }

    // Doorbell writes are required by the SDMA/MES ring bring-up that follows.
    if (map_doorbells(hardware, extension) != NO_ERROR) {
        return BringupResult::vram_window_unavailable;
    }

    bringup.hardware = &hardware;
    bringup.extension = extension;
    // amdgpu_device_get_pcie_info runs once at init; resume reuses adev->pm's masks.
    bringup.pcie = pcie_link(hardware, bringup);
    bringup.dummy_page_bus = dummy.bus.value;
    const hardware::AsicInputs inputs = asic_inputs(hardware, rom, interrupt_ready, true, bringup);

    // hardware/extension stay set: the device extension owns both for its lifetime.
    const auto status = bringup.asic.run(inputs, policy, {nullptr, no_reset_owner});

    // Training is over (or was skipped): release the bring-up-only holds.
    bringup.vram_allocator.release(bringup.backup_hold);
    bringup.vram_allocator.release(bringup.training_hold);
    bringup.backup_hold = {};
    bringup.training_hold = {};

    switch (status) {
    case hardware::AsicStatus::completed: return BringupResult::completed;
    case hardware::AsicStatus::quarantined: return BringupResult::sequence_quarantined;
    default: return BringupResult::sequence_failed;
    }
}

void release_asic_files(AsicBringup& bringup) noexcept {
    bringup.catalog.reset();

    for (auto& file : bringup.files) {
        DriverBlob::release(file);
    }

    DriverBlob::release(bringup.sos);
    DriverBlob::release(bringup.ta);
}

hardware::AsicStatus shutdown_asic(AsicBringup& bringup) noexcept {
    if (bringup.process_shutdown != nullptr && !bringup.process_shutdown(bringup.process_owner)) {
        return hardware::AsicStatus::busy;
    }
    return bringup.asic.shutdown();
}

hardware::AsicStatus suspend_asic(AsicBringup& bringup, void* interrupt_owner,
                                  bool (*ih_stopping)(void*) noexcept) noexcept {
    if (bringup.hardware == nullptr || KeGetCurrentIrql() != PASSIVE_LEVEL) {
        return hardware::AsicStatus::busy;
    }

    const auto status = bringup.asic.suspend({{&bringup, cache_pci_state, clear_bus_master,
                                               restore_pci_state}, interrupt_owner, ih_stopping});

    // pci_pm_suspend_noirq: pci_save_state after the driver's suspend_noirq (the
    // mode-1 reset), the record pci_pm_resume_noirq restores on D0 entry.
    bringup.pci_state_cached = false;
    bringup.power_saved = false;

    if (status == hardware::AsicStatus::completed && cache_pci_state(&bringup)) {
        bringup.power_saved = true;
    }

    return status;
}

hardware::AsicStatus resume_asic(HardwareState& hardware, ByteView rom,
                                 hardware::InterruptReady interrupt_ready,
                                 AsicBringup& bringup) noexcept {
    if (bringup.hardware != &hardware || KeGetCurrentIrql() != PASSIVE_LEVEL ||
        !bringup.asic.report().suspended) {
        return hardware::AsicStatus::busy;
    }

    // pci_pm_resume_noirq -> pci_restore_state: after S3 the function comes back
    // with reset configuration space (BARs, command, PCIe, MSI state).
    if (!bringup.power_saved || !restore_pci_state(&bringup)) {
        return hardware::AsicStatus::failed;
    }

    // The BIST-encroached range and its backup are held again while resume may
    // train (psp_mem_training with PSP_MEM_TRAIN_RESUME can escalate to a long
    // training); without both holds resume refuses a long training.
    const auto hold = hardware::vram_kernel_bo_flags(false);
    bool held = bringup.vram_allocator.allocate_at(0, hardware::psp_training_encroached_bytes, hold,
                                                   bringup.training_hold) == hardware::VramStatus::success;

    if (held && bringup.vram_allocator.allocate_at(bringup.training_backup_offset,
                                                   hardware::psp_training_encroached_bytes, hold,
                                                   bringup.backup_hold) != hardware::VramStatus::success) {
        bringup.vram_allocator.release(bringup.training_hold);
        bringup.training_hold = {};
        held = false;
    }

    const auto status = bringup.asic.resume(asic_inputs(hardware, rom, interrupt_ready, held, bringup));

    if (held) {
        bringup.vram_allocator.release(bringup.backup_hold);
        bringup.vram_allocator.release(bringup.training_hold);
        bringup.backup_hold = {};
        bringup.training_hold = {};
    }

    if (status == hardware::AsicStatus::completed) {
        bringup.power_saved = false;
    }

    return status;
}

hardware::AsicStatus mode1_reset_asic(AsicBringup& bringup) noexcept {
    // Serialize the entire reset against process-exit callbacks and IOCTLs,
    // including the interval before the hardware reset proof exists.
    if (bringup.process_reset_begin != nullptr &&
        !bringup.process_reset_begin(bringup.process_owner)) {
        return hardware::AsicStatus::busy;
    }
    // A suspend whose closing mode-1 reset completed but whose other stages
    // failed already holds the reset proof: AsicInit refuses a second reset.
    const auto status = bringup.asic.reset_proven() ? hardware::AsicStatus::completed :
        bringup.asic.mode1_reset({&bringup, cache_pci_state, clear_bus_master, restore_pci_state});
    const auto released = status == hardware::AsicStatus::completed ?
        bringup.asic.release_after_reset(&bringup, retire_reset_owners) : status;
    if (bringup.process_reset_end != nullptr) bringup.process_reset_end(bringup.process_owner);
    return released;
}

hardware::AsicStatus restart_asic_after_reset(HardwareState& hardware, ByteView rom,
                                              hardware::InterruptReady interrupt_ready,
                                              AsicBringup& bringup) noexcept {
    if (bringup.hardware != &hardware || KeGetCurrentIrql() != PASSIVE_LEVEL ||
        bringup.asic.report().status != hardware::AsicStatus::released) {
        return hardware::AsicStatus::busy;
    }

    // As resume_asic: the BIST-encroached range and its backup are held while
    // the training may run; allocations made since bring-up may occupy them.
    const auto hold = hardware::vram_kernel_bo_flags(false);
    bool held = bringup.vram_allocator.allocate_at(0, hardware::psp_training_encroached_bytes, hold,
                                                   bringup.training_hold) == hardware::VramStatus::success;

    if (held && bringup.vram_allocator.allocate_at(bringup.training_backup_offset,
                                                   hardware::psp_training_encroached_bytes, hold,
                                                   bringup.backup_hold) != hardware::VramStatus::success) {
        bringup.vram_allocator.release(bringup.training_hold);
        bringup.training_hold = {};
        held = false;
    }

    const auto status = bringup.asic.restart_after_reset(asic_inputs(hardware, rom, interrupt_ready, held, bringup),
                                                         asic_policy(bringup), {nullptr, no_reset_owner});

    if (held) {
        bringup.vram_allocator.release(bringup.backup_hold);
        bringup.vram_allocator.release(bringup.training_hold);
        bringup.backup_hold = {};
        bringup.training_hold = {};
    }

    return status;
}

} // namespace rx7900::reactos

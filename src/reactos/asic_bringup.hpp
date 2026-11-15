/*
Module: rx7900.reactos.asic_bringup
Purpose: Bind the portable Navi31 ASIC sequencer to real ReactOS MMIO, VRAM and files.
Created: 2026-10-21
Architecture: Device-extension owner used only by the opt-in rx7900video path;
src/hardware owns every protocol, this module only acquires resources.
*/
#pragma once

#include "hardware.hpp"
#include "dma.hpp"
#include "pci_config.hpp"
#include "vram_window.hpp"
#include "../hardware/vram_allocator.hpp"
#include "../hardware/vram_backend.hpp"
#include "asic_init.hpp"
#include "vram_discovery.hpp"
#include "vram_reservations.hpp"

namespace rx7900::reactos {

// Driver-owned VRAM seen through BAR0: the 32 MiB local-VRAM arena followed by the
// 32 MiB that receives the BIST-encroached lowest 32 MiB during PSP long training.
// Placement policy (hardware::place_vram_region over the Linux v6.12 reservation
// map built at run time from ATOM firmwareinfo/vram_usagebyfirmware, the DCN
// pre-OS scanout size and the VRAM size): the pair starts at the preferred
// 64 MiB, above the BIST range and a 4K 32bpp pre-OS buffer, when that is clear
// of every reservation; otherwise at the lowest 2 MiB boundary above it that is
// clear and still ends inside the BAR0 window; otherwise bring-up refuses
// (vram_reserved) before any VRAM is mapped or written.
constexpr uint64_t asic_vram_arena_offset = 64ULL << 20;
constexpr ULONG asic_vram_arena_bytes = 32u << 20;
// CPU-visible VRAM pool carved by the whole-VRAM allocator for every sequencer
// VRAM buffer (the PSP TMR alone is 0x1300000 bytes on this card).
constexpr uint64_t asic_vram_pool_bytes = 64ULL << 20;
constexpr uint64_t asic_vram_placement_alignment = 2ULL << 20;
constexpr ULONG asic_vram_copy_chunk_bytes = 1u << 20;

// Values are reported numerically by the miniport; new values are appended.
enum class BringupResult {
    completed,
    not_attempted,
    discovery_unavailable,
    rom_unavailable,
    vram_window_unavailable,
    firmware_unavailable,
    sequence_failed,
    sequence_quarantined,
    // The reservation map could not be built or leaves no compliant placement.
    vram_reserved
};


// Construct once inside the zeroed device extension; trivially destructible.
// release_files must run before the extension is freed; GPU-held resources stay
// reserved (there is no reset/teardown path yet) until the next system reset.
// After a completed bring-up the firmware files stay resident for resume.
struct AsicBringup final {
    hardware::DiscoveryBinary discovery{};
    hardware::VramDiscoveryReport discovery_report{};
    // amdgpu_vram_mgr over all usable VRAM (Linux reservations carved out), the
    // BAR0 aperture for CPU-visible allocations, and the sequencer's pool.
    hardware::VramAllocator vram_allocator{};
    VramAperture aperture{};
    hardware::VramBackend visible_pool{};
    memory::Manager vram{};
    // Held only while bring-up (PSP training) runs: the BIST-encroached range and
    // the training backup, as Linux releases its training reservations.
    hardware::VramAllocationHandle training_hold{};
    hardware::VramAllocationHandle backup_hold{};
    hardware::UcodeCatalog catalog{};
    BlobStorage files[hardware::ucode_source_count]{};
    BlobStorage sos{};
    BlobStorage ta{};
    hardware::AtomFirmwareInfo atom{};
    // Linux reservations and the chosen VRAM offsets of the arena and the training
    // backup (arena_offset + asic_vram_arena_bytes); fixed for the device lifetime.
    hardware::VramReservationMap reservations{};
    hardware::VramPlacementStatus placement = hardware::VramPlacementStatus::invalid_argument;
    uint32_t vbios_fb_bytes = 0;
    uint64_t arena_offset = 0;
    uint64_t training_backup_offset = 0;
    // GART dummy page (amdgpu_gart_dummy_page_init). The hubs may read or write it
    // at any time once enabled, so it is never freed: without a reset owner the
    // common buffer is deliberately retained until the system restarts.
    DmaPool dma{};
    DmaHandle dummy_page{};
    // GTT system pages (rings, EOP, writeback, MQDs) bound through the GART;
    // the GttPlatform token is the address of the slot holding the DMA handle.
    DmaHandle gtt[memory::allocation_capacity]{};
    // PCI config workspace (amdgpu_device_get_pcie_info) and the Linux
    // pci_save_state record for the explicit mode-1 reset.
    PciConfigWorkspace pci_workspace{};
    PciSavedState pci_saved{};
    bool pci_state_cached = false;
    // Set by a completed suspend_asic: pci_saved holds the D3-entry record that
    // resume_asic restores (Linux pci_save_state in pci_pm_suspend_noirq).
    bool power_saved = false;
    // amdgpu_device_get_pcie_info result and the GART dummy page bus address,
    // fixed at bring-up and reused by resume.
    hardware::SmuPcieLink pcie{};
    uint64_t dummy_page_bus = 0;
    hardware::AsicInit asic{};
    // Set by run_asic_bringup and kept for the device lifetime: the IH doorbell
    // (interrupt acknowledgement), GTT and doorbell callbacks use it after run.
    HardwareState* hardware = nullptr;
    PVOID extension = nullptr;
    NTSTATUS last_file_status = STATUS_SUCCESS;
    bool attempted = false;
    // Registered native process service; explicit reset retires it only after
    // ASIC reset proof, before MES/GART and native DMA backing are released.
    void* process_owner = nullptr;
    bool (*process_reset_begin)(void*) noexcept = nullptr;
    bool (*process_reset_retire)(void*) noexcept = nullptr;
    void (*process_reset_end)(void*) noexcept = nullptr;
    bool (*process_shutdown)(void*) noexcept = nullptr;
    // Registered physical 2D owner (Radeon2dRecovery::retire_after_reset on the
    // Radeon2dBackend): runs in the same confirmed-reset window, before the
    // process service, and drops its engine state and VRAM surfaces while
    // keeping staging and system-memory surfaces. false keeps every lease.
    void* surface_owner = nullptr;
    bool (*surface_reset_retire)(void*) noexcept = nullptr;
};

// PASSIVE_LEVEL in HwInitialize, after acquire and before any interrupt source is
// claimed. One attempt per device start. rom is the validated ATOM image owned by
// hardware (acquire_atom_rom). Reads discovery from VRAM, maps the BAR0 arena,
// loads fixed-name firmware under SystemRoot\System32\drivers\rx7900 and runs
// hardware::AsicInit. Never resets the GPU or changes PCI bindings.
BringupResult run_asic_bringup(HardwareState& hardware, PVOID extension, ByteView rom,
                               hardware::InterruptReady interrupt_ready,
                               AsicBringup& bringup) noexcept;

// Explicit owner requests only; nothing in the driver calls these implicitly.
// shutdown_asic runs AsicInit::shutdown (Linux amdgpu_device_ip_fini_early order).
// mode1_reset_asic runs AsicInit::mode1_reset with PCI config cache/restore and
// bus-master clear, then releases every lease once the reset is proven. The
// caller must first disarm its IrqService (disarm_locked through
// VideoPortSynchronizeExecution): the release frees the GTT IH ring the ISR reads.
// The PCI save/restore follows Linux pci_save_state/pci_restore_state for the
// header, PCIe, LTR and MSI state; extended (ReBAR) state only where the HAL
// reaches extended config space, which current ReactOS does not.
// mode1_reset_asic also accepts an ASIC whose reset already completed (the
// mode-1 reset at the end of a suspend that then failed): it skips the reset
// and releases with that proof. The confirmed-reset window retires the 2D owner
// (surface_reset_retire) and then the process service.
hardware::AsicStatus shutdown_asic(AsicBringup& bringup) noexcept;
hardware::AsicStatus mode1_reset_asic(AsicBringup& bringup) noexcept;

// Destructive-reset recovery, PASSIVE_LEVEL, exclusive owner, after
// mode1_reset_asic returned released: AsicInit::restart_after_reset with the
// same resident firmware, catalog, discovery, VRAM Manager, GTT/DMA platform
// and policy as the first run (interrupt_ready rebinds the IrqService after IH
// starts). The training holds are taken again only where nothing occupies
// them; without them a resume training that escalates to a long one refuses.
hardware::AsicStatus restart_asic_after_reset(HardwareState& hardware, ByteView rom,
                                              hardware::InterruptReady interrupt_ready,
                                              AsicBringup& bringup) noexcept;

// VideoPort D0 -> D3 of a completed bring-up (HwSetPowerState), PASSIVE_LEVEL,
// exclusive owner: AsicInit::suspend with this device's PCI reset callbacks;
// ih_stopping(interrupt_owner) runs right before the IH rings are disabled (the
// caller disarms, drains and suspends its IrqService there). On completion the
// post-reset PCI configuration is saved for D0 (pci_pm_suspend_noirq). The
// firmware files, VRAM pool and DMA pages stay resident: resume needs them.
hardware::AsicStatus suspend_asic(AsicBringup& bringup, void* interrupt_owner,
                                  bool (*ih_stopping)(void*) noexcept) noexcept;
// D3 -> D0 after a completed suspend_asic: PCI state restored (pci_restore_state),
// the training holds taken again for the resume training, then AsicInit::resume
// with the same resident inputs (interrupt_ready rebinds the IrqService after IH
// resume). A card that lost its POST is re-POSTed first by AsicInit::repost
// (VBIOS ASIC_Init from rom through the bounded ATOM interpreter, MMIO and the
// delay callback above, all at PASSIVE_LEVEL). A failed repost (status with step
// repost_*) or unsupported (no usable ATOM image) leaves it suspended with its
// PCI save record, and the call may be repeated.
hardware::AsicStatus resume_asic(HardwareState& hardware, ByteView rom,
                                 hardware::InterruptReady interrupt_ready,
                                 AsicBringup& bringup) noexcept;

// Frees firmware pool copies only. PSP copied accepted payloads to TMR, but
// psp_resume loads every payload again, so a completed bring-up keeps them.
void release_asic_files(AsicBringup& bringup) noexcept;

} // namespace rx7900::reactos

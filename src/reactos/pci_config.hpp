/*
Module: rx7900.reactos.pci_config
Purpose: Acquire PCI configuration images of the owned GPU and its upstream bridges
through the ReactOS HAL, and execute the Linux-ordered save/restore plan.
Created: 2026-10-22
Architecture: Platform adapter only. src/common/pci_caps owns every decode and
policy; this module reads/writes bytes with HalGetBusDataByOffset and
HalSetBusDataByOffset on PCI segment 0 and translates errors.

ReactOS facts this module depends on (tree a97c8a2c, read 2026-10-22):
- VideoPortGetBusData forwards to HalGetBusDataByOffset with the device's own
  SystemIoBusNumber and, for PnP devices, ignores the caller's SlotNumber
  (win32ss/drivers/videoprt/resource.c), so another function's configuration
  space is reachable only through the HAL export with explicit bus/slot.
- Both HALs serve PCIConfiguration from HalpGetPCIData/HalpSetPCIData
  (hal/halx86/legacy/bus/pcibus.c, acpi/busemul.c): CF8/CFC type-1 access with
  a 6-bit RegisterNumber and Length clamped to 256 bytes. Offsets 0x100-0xFFF
  alias offsets 0x00-0xFF; no MCFG/ECAM path exists in the HAL. This module
  never forwards an extended offset to the HAL except for the Linux
  pci_cfg_space_size_ext aliasing probe, which only reads.
*/
#pragma once

#include "hardware.hpp"
#include "pci_caps.hpp"

namespace rx7900::reactos {

// One function's configuration image. valid_bytes is 256, or 4096 only when the
// aliasing probe proved extended space reachable at capture time.
struct ConfigImage final {
    PciLocation location;
    uint32_t valid_bytes;
    bool extended_reachable;
    uint8_t bytes[pci_extended_config_bytes];
};

// Borrowed view of the valid prefix; the image must outlive it.
ByteView config_view(const ConfigImage& image) noexcept;

// About 6.3 KiB: keep it in the zeroed device extension, never on the kernel stack.
// Single owner; calls using one workspace must not overlap.
struct PciConfigWorkspace final {
    ConfigImage device;
    UpstreamChain upstream;
};

// Segment-0 location of the owned function recorded by acquire().
PciLocation owned_location(const HardwareState& state) noexcept;

// amdgpu defaults plus amdgpu_device_pcie_dynamic_switching_supported: false
// only on a GenuineIntel CPU (CPUID leaf 0). ReactOS exposes no removable-device
// attribute for PCI functions, so dev_is_removable is treated as false.
PcieInfoPolicy native_pcie_policy() noexcept;

// PASSIVE_LEVEL. Reads 256 bytes, then the 15 aliasing-probe dwords at
// 0x100..0xF00; if (and only if) the probe proves reachability, reads the rest
// of the 4 KiB space in 256-byte chunks. STATUS_NO_SUCH_DEVICE for an absent
// function, STATUS_NOT_SUPPORTED for a nonzero segment.
NTSTATUS capture_config(PciLocation location, ConfigImage& image) noexcept;

// PASSIVE_LEVEL. resolve_upstream_path over HAL dword reads, then the 256-byte
// image of every bridge, nearest first. A device on bus 0 has an empty chain.
// STATUS_NOT_FOUND when no bus-0 bridge forwards the device's bus.
NTSTATUS capture_upstream_chain(PciLocation device, UpstreamChain& chain) noexcept;

// PASSIVE_LEVEL, read-only. Linux amdgpu_device_get_pcie_info for the owned
// function and its real upstream bridges, followed by the smu_smc_hw_setup
// encoding; replaces asic_bringup's LNKSTA-based conversion. On failure link
// is {0, 0}, which the SMU step refuses.
NTSTATUS query_pcie_smu_link(const HardwareState& state, PciConfigWorkspace& workspace,
                             PcieInfoReport& report, PcieSmuLink& link) noexcept;

// Decoded view of the owned function's capabilities. rebar_status is
// extended_unavailable whenever the HAL cannot reach 0x100-0xFFF (current
// ReactOS): the Resizable BAR capability is then reported unknown, never guessed.
struct PciCapabilityReport final {
    PcieCapability express;
    PowerManagementCapability power;
    MsiCapability msi;
    MsixCapability msix;
    bool extended_reachable;
    CapStatus rebar_status;
    ResizableBarCapability rebar;
};

// PASSIVE_LEVEL, read-only; decode only (interrupt mode is never changed).
NTSTATUS query_capabilities(const HardwareState& state, PciConfigWorkspace& workspace,
                            PciCapabilityReport& report) noexcept;

// PASSIVE_LEVEL. Linux pci_save_state of the owned function into a
// caller-owned record (amdgpu_device_cache_pci_state).
NTSTATUS save_state(const HardwareState& state, PciConfigWorkspace& workspace,
                    PciSavedState& saved) noexcept;

// PASSIVE_LEVEL, exclusive owner, explicit reset/resume paths only
// (amdgpu_device_load_pci_state). Executes plan_pci_restore with Linux
// compare-before-write and BAR retry semantics (1 ms stalls). Extended writes
// run only if the probe still proves reachability; otherwise nothing is
// written. Direct HAL writes are not serialized with the PCI driver's own
// sizing gate; no PnP resource query may be in flight.
NTSTATUS restore_state(const HardwareState& state, const PciSavedState& saved) noexcept;

} // namespace rx7900::reactos

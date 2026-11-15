/*
Module: rx7900.common.pci_caps
Purpose: Decode PCI/PCIe capabilities from configuration images and reproduce the
Linux v6.12 amdgpu PCIe capability, Resizable BAR and save/restore policies.
Created: 2026-10-22
Architecture: Pure, allocation-free parsers over ByteView images; platform code
(src/reactos/pci_config) only acquires the bytes and executes write plans.
Adapted from Linux v6.12 drivers/pci (pci.c, probe.c, access.c, setup-res.c,
pcie/aspm.c, msi/msi.c), include/uapi/linux/pci_regs.h and
drivers/gpu/drm/amd (amdgpu_device.c, amdgpu_smu.c, include/amd_pcie.h).
*/
#pragma once

#include "pci.hpp"

namespace rx7900 {

// PCI_CFG_SPACE_SIZE and PCI_CFG_SPACE_EXP_SIZE.
constexpr size_t pci_config_bytes = 256;
constexpr size_t pci_extended_config_bytes = 4096;

enum class CapStatus {
    success,
    absent,               // capability not implemented (Linux reads zero)
    truncated_input,      // image shorter than the region the decoder needs
    malformed,            // looping list, reserved encodings or out-of-image pointers
    capacity_exceeded,    // more entries than the fixed-capacity output holds
    extended_unavailable  // image has no reachable extended space (0x100-0xFFF)
};

// Standard capability IDs (pci_regs.h PCI_CAP_ID_*).
constexpr uint8_t pci_cap_id_power_management = 0x01;
constexpr uint8_t pci_cap_id_msi = 0x05;
constexpr uint8_t pci_cap_id_vendor = 0x09;
constexpr uint8_t pci_cap_id_express = 0x10;
constexpr uint8_t pci_cap_id_msix = 0x11;
// Extended capability IDs (PCI_EXT_CAP_ID_*).
constexpr uint16_t pci_ext_cap_id_aer = 0x01;
constexpr uint16_t pci_ext_cap_id_vendor = 0x0b;
constexpr uint16_t pci_ext_cap_id_rebar = 0x15;
constexpr uint16_t pci_ext_cap_id_ltr = 0x18;

constexpr size_t pci_command_offset = 0x04;
constexpr size_t pci_status_offset = 0x06;
constexpr uint16_t pci_command_memory = 0x0002;
constexpr uint16_t pci_command_master = 0x0004;
constexpr uint16_t pci_command_intx_disable = 0x0400;
constexpr uint8_t pci_header_type_normal = 0;
constexpr uint8_t pci_header_type_bridge = 1;

// ---------------------------------------------------------------------------
// Capability lists
// ---------------------------------------------------------------------------

struct CapabilityEntry final {
    uint8_t id;
    uint8_t offset;
};

// PCI_FIND_CAP_TTL: at most 48 standard capabilities are followed.
constexpr size_t pci_capability_capacity = 48;

struct CapabilityList final {
    CapabilityEntry entries[pci_capability_capacity];
    uint32_t count;
};

// Input: at least the 256-byte legacy image. Follows Linux __pci_find_next_cap_ttl
// (Status.CapList gate, header-type pointer, pointers below 0x40 or ID 0xFF end
// the list, low two pointer bits ignored). Divergence: a list that does not end
// within 48 entries is malformed instead of silently "not found". Output is
// cleared first and is complete only on success.
CapStatus walk_capabilities(ByteView config, CapabilityList& output) noexcept;

// Linux pci_find_capability over an image. absent when the ID is not listed.
CapStatus find_capability(ByteView config, uint8_t id, uint8_t& offset) noexcept;

struct ExtCapabilityEntry final {
    uint16_t id;
    uint8_t version;
    uint16_t offset;
};

constexpr size_t pci_ext_capability_capacity = 64;

struct ExtCapabilityList final {
    ExtCapabilityEntry entries[pci_ext_capability_capacity];
    uint32_t count;
};

// Linux pci_cfg_space_size_ext reachability rule for live reads: identity is
// the dword at offset 0 and block_heads[i] the dword at 0x100 * (i + 1). The
// extended space is reachable when the dword at 0x100 is not all ones and not
// every 0x100-aligned dword aliases offset 0 (a mechanism that silently drops
// register bits above 0xFF, like the ReactOS CF8/CFC HAL, aliases them all).
// Divergence: callers must treat any failed probe read as unreachable.
struct ExtendedConfigProbe final {
    uint32_t identity;
    uint32_t block_heads[15];
};
bool extended_config_reachable(const ExtendedConfigProbe& probe) noexcept;

// Extended walks need a 4096-byte image (extended_unavailable otherwise). An
// all-zero header at 0x100 means no extended capabilities (absent). Bounded by
// Linux's 480-entry TTL; non-termination is malformed.
CapStatus walk_ext_capabilities(ByteView config, ExtCapabilityList& output) noexcept;
// Linux pci_find_ext_capability(dev, id) over an image.
CapStatus find_ext_capability(ByteView config, uint16_t id, uint16_t& offset) noexcept;

// ---------------------------------------------------------------------------
// PCI Express capability
// ---------------------------------------------------------------------------

// Linux PCI_EXP_TYPE_* port types.
constexpr uint8_t pcie_type_endpoint = 0x0;
constexpr uint8_t pcie_type_legacy_endpoint = 0x1;
constexpr uint8_t pcie_type_root_port = 0x4;
constexpr uint8_t pcie_type_upstream = 0x5;
constexpr uint8_t pcie_type_downstream = 0x6;
constexpr uint8_t pcie_type_pci_bridge = 0x7;
constexpr uint8_t pcie_type_pcie_bridge = 0x8;
constexpr uint8_t pcie_type_rc_endpoint = 0x9;
constexpr uint8_t pcie_type_rc_event_collector = 0xa;

// Register offsets inside the PCIe capability (PCI_EXP_*).
constexpr size_t pcie_flags_register = 0x02;
constexpr size_t pcie_devctl_register = 0x08;
constexpr size_t pcie_lnkctl_register = 0x10;
constexpr size_t pcie_sltctl_register = 0x18;
constexpr size_t pcie_rtctl_register = 0x1c;
constexpr size_t pcie_devctl2_register = 0x28;
constexpr size_t pcie_lnkctl2_register = 0x30;
constexpr size_t pcie_sltctl2_register = 0x38;

// Every register Linux pcie_capability_reg_implemented accepts, read from the
// image; registers that are not implemented for the port type/version stay
// zero exactly as pcie_capability_read_* returns them. SLTCAP2/SLTCTL2 are
// never implemented in Linux v6.12's accessor, so they are not decoded.
struct PcieCapability final {
    bool present;
    uint8_t offset;
    uint16_t flags;
    uint8_t version;
    uint8_t port_type;
    bool link_implemented;   // pcie_cap_has_lnkctl
    bool slot_implemented;   // pcie_cap_has_sltctl
    bool root_implemented;   // pcie_cap_has_rtctl
    bool version2;           // pcie_cap_version > 1 (DEVCAP2/DEVCTL2)
    bool link2_implemented;  // pcie_cap_has_lnkctl2
    uint32_t device_capabilities;
    uint16_t device_control;
    uint16_t device_status;
    uint32_t link_capabilities;
    uint16_t link_control;
    uint16_t link_status;
    uint32_t slot_capabilities;
    uint16_t slot_control;
    uint16_t slot_status;
    uint16_t root_control;
    uint16_t root_capabilities;
    uint32_t root_status;
    uint32_t device_capabilities2;
    uint16_t device_control2;
    uint32_t link_capabilities2;
    uint16_t link_control2;
    uint16_t link_status2;
};

// absent (with output zeroed, present false) when the function has no PCIe
// capability; malformed when the implemented registers extend past the image.
CapStatus decode_pcie(ByteView config, PcieCapability& output) noexcept;

// Linux enum pci_bus_speed restricted to PCIe values.
enum class PcieSpeed : uint8_t { unknown, gt2_5, gt5_0, gt8_0, gt16_0, gt32_0, gt64_0 };
// PCIE_LNK_WIDTH_UNKNOWN; widths are otherwise raw MLW/NLW fields (0 = reserved).
constexpr uint8_t pcie_width_unknown = 0xff;

// pcie_link_speed[] indexed by LNKCAP.SLS or LNKSTA.CLS (low four bits).
PcieSpeed pcie_speed_from_code(uint32_t code) noexcept;
// pcie_get_speed_cap: LNKCAP2 vector when nonzero, else LNKCAP SLS 2.5/5 only.
PcieSpeed pcie_speed_cap(const PcieCapability& express) noexcept;
// pcie_get_width_cap: LNKCAP MLW when LNKCAP is nonzero, else unknown.
uint8_t pcie_width_cap(const PcieCapability& express) noexcept;
// Current link from LNKSTA (to_pcie_link_speed and PCI_EXP_LNKSTA_NLW).
PcieSpeed pcie_link_speed(const PcieCapability& express) noexcept;
uint8_t pcie_link_width(const PcieCapability& express) noexcept;
// width * PCIE_SPEED2MBS_ENC(speed), in Mb/s.
uint32_t pcie_bandwidth_mbps(PcieSpeed speed, uint8_t width) noexcept;

// ---------------------------------------------------------------------------
// Power management, MSI, MSI-X, LTR
// ---------------------------------------------------------------------------

struct PowerManagementCapability final {
    bool present;
    uint8_t offset;
    uint16_t capabilities;    // PMC
    uint16_t control_status;  // PMCSR
    uint8_t power_state;      // PMCSR bits 1:0, D0..D3hot
};
CapStatus decode_power_management(ByteView config, PowerManagementCapability& output) noexcept;

struct MsiCapability final {
    bool present;
    uint8_t offset;
    uint16_t control;
    bool enabled;
    bool address64;
    bool per_vector_mask;
    uint8_t vectors_capable;  // 1 << Multiple Message Capable
    uint8_t vectors_enabled;  // 1 << Multiple Message Enable
    uint32_t address_low;
    uint32_t address_high;
    uint16_t data;
    uint32_t mask;
    uint32_t pending;
    uint8_t data_offset;   // absolute config offsets, for save/restore
    uint8_t mask_offset;   // zero without per-vector masking
};
// Reserved MMC/MME encodings (6, 7) are malformed.
CapStatus decode_msi(ByteView config, MsiCapability& output) noexcept;

struct MsixCapability final {
    bool present;
    uint8_t offset;
    uint16_t control;
    bool enabled;
    bool function_masked;
    uint16_t table_entries;
    uint8_t table_bar;
    uint32_t table_offset;
    uint8_t pba_bar;
    uint32_t pba_offset;
};
// BAR indicators above 5 are malformed.
CapStatus decode_msix(ByteView config, MsixCapability& output) noexcept;

struct LtrCapability final {
    bool present;
    uint16_t offset;
    uint32_t max_latencies;  // dword at +4: max snoop (low) and no-snoop (high)
};
CapStatus decode_ltr(ByteView config, LtrCapability& output) noexcept;

// ---------------------------------------------------------------------------
// Resizable BAR
// ---------------------------------------------------------------------------

constexpr size_t pci_rebar_capacity = 6;

struct ResizableBar final {
    uint8_t bar_index;
    // pci_rebar_get_possible_sizes: bit n means 1 MiB << n is supported.
    uint32_t supported_sizes;
    // pci_rebar_get_current_size: CTRL BAR Size field.
    uint8_t current_size;
    uint16_t control_offset;  // absolute offset of this entry's CTRL dword
    uint32_t control;
};

struct ResizableBarCapability final {
    bool present;
    uint16_t offset;
    uint8_t count;
    ResizableBar bars[pci_rebar_capacity];
};

// Requires the 4096-byte image. Validates NBAR 1..6 and unique BAR indices <= 5
// (malformed otherwise); applies Linux's Sapphire RX 5600 XT (1002:731F) quirk.
CapStatus decode_resizable_bar(ByteView config, ResizableBarCapability& output) noexcept;
// Linux pci_rebar_find_pos + supported/current size for one BAR index.
CapStatus find_resizable_bar(const ResizableBarCapability& rebar, uint8_t bar_index,
                             ResizableBar& output) noexcept;
// pci_rebar_size_to_bytes (sizes above 43 are not representable: returns 0).
uint64_t rebar_size_to_bytes(uint8_t size) noexcept;
// pci_rebar_bytes_to_size: roundup_pow_of_two, then max(ilog2, 20) - 20.
// Zero and values above 2^63 have no Linux definition and return 0xff.
uint8_t rebar_bytes_to_size(uint64_t bytes) noexcept;

// Inputs to amdgpu_device_resize_fb_bar. The platform states facts Linux reads
// from the kernel configuration, SR-IOV state and PCI core.
struct FbBarInputs final {
    uint64_t real_vram_bytes;
    uint64_t bar0_bytes;               // currently assigned BAR0 length
    uint32_t bar0_supported_sizes;     // ResizableBar.supported_sizes, 0 if unknown
    bool physical_address_64bit;       // CONFIG_PHYS_ADDR_T_64BIT
    bool sriov_vf;
    bool root_window_above_4g;         // root bus memory window starting above 4 GiB
    bool firmware_config_preserved;    // pci_host_bridge.preserve_config
};

enum class FbBarAction {
    keep_32bit_physical,     // !CONFIG_PHYS_ADDR_T_64BIT
    keep_sriov_vf,
    keep_large_enough,       // BAR0 already covers real VRAM
    keep_no_root_window,     // no 64-bit root window above 4 GiB
    keep_config_preserved,   // pci_resize_resource -ENOTSUPP (preserve_config)
    keep_not_resizable,      // no ReBAR sizes: pci_resize_resource -ENOTSUPP
    keep_size_unsupported,   // target size bit not offered: -EINVAL
    resize                   // pci_resize_resource(pdev, 0, target_size)
};

struct FbBarPlan final {
    FbBarAction action;
    uint8_t target_size;  // ReBAR size encoding; valid only for resize
};

// malformed when real VRAM is zero (Linux has no defined size for it).
CapStatus plan_fb_bar_resize(const FbBarInputs& inputs, FbBarPlan& output) noexcept;

// ---------------------------------------------------------------------------
// amdgpu_device_get_pcie_info and smu_smc_hw_setup encoding
// ---------------------------------------------------------------------------

// drivers/gpu/drm/amd/include/amd_pcie.h
constexpr uint32_t cail_pcie_link_speed_gen1 = 0x00010000;
constexpr uint32_t cail_pcie_link_speed_gen2 = 0x00020000;
constexpr uint32_t cail_pcie_link_speed_gen3 = 0x00040000;
constexpr uint32_t cail_pcie_link_speed_gen4 = 0x00080000;
constexpr uint32_t cail_pcie_link_speed_gen5 = 0x00100000;
constexpr uint32_t cail_asic_pcie_link_speed_gen1 = 0x00000001;
constexpr uint32_t cail_asic_pcie_link_speed_gen2 = 0x00000002;
constexpr uint32_t cail_asic_pcie_link_speed_gen3 = 0x00000004;
constexpr uint32_t cail_asic_pcie_link_speed_gen4 = 0x00000008;
constexpr uint32_t cail_asic_pcie_link_speed_gen5 = 0x00000010;
constexpr uint32_t cail_pcie_link_width_x1 = 0x00010000;
constexpr uint32_t cail_pcie_link_width_x2 = 0x00020000;
constexpr uint32_t cail_pcie_link_width_x4 = 0x00040000;
constexpr uint32_t cail_pcie_link_width_x8 = 0x00080000;
constexpr uint32_t cail_pcie_link_width_x12 = 0x00100000;
constexpr uint32_t cail_pcie_link_width_x16 = 0x00200000;
constexpr uint32_t cail_pcie_link_width_x32 = 0x00400000;
constexpr uint32_t amdgpu_default_pcie_gen_mask =
    cail_pcie_link_speed_gen1 | cail_pcie_link_speed_gen2 | cail_asic_pcie_link_speed_gen1 |
    cail_asic_pcie_link_speed_gen2 | cail_asic_pcie_link_speed_gen3;
constexpr uint32_t amdgpu_default_pcie_mlw_mask =
    cail_pcie_link_width_x1 | cail_pcie_link_width_x2 | cail_pcie_link_width_x4 |
    cail_pcie_link_width_x8 | cail_pcie_link_width_x16;

struct PcieInfoPolicy final {
    uint32_t gen_cap_override;   // amdgpu.pcie_gen_cap module parameter (default 0)
    uint32_t lane_cap_override;  // amdgpu.pcie_lane_cap module parameter (default 0)
    // amdgpu_device_pcie_dynamic_switching_supported: false only on Intel x86
    // CPUs with a non-removable device.
    bool dynamic_switching;
    bool passthrough;            // amdgpu_passthrough (hypervisor pass-through)
};
constexpr PcieInfoPolicy linux_default_pcie_policy{0, 0, true, false};

// Upstream bridges nearest first: bridges[0] is pci_upstream_bridge(device),
// the last entry sits on a root bus. Each image is a full 256-byte type-1
// header; count 0 means the device itself sits on a root bus.
constexpr size_t pci_upstream_capacity = 8;

struct PciFunctionImage final {
    PciLocation location;
    uint8_t bytes[pci_config_bytes];
};

struct UpstreamChain final {
    PciFunctionImage bridges[pci_upstream_capacity];
    uint32_t count;
};

struct PcieMasks final {
    uint32_t gen_mask;  // adev->pm.pcie_gen_mask
    uint32_t mlw_mask;  // adev->pm.pcie_mlw_mask
};

struct PcieInfoReport final {
    PcieMasks masks;
    bool root_bus;               // returned early with the default masks
    PcieSpeed device_speed_cap;  // pcie_get_speed_cap(adev->pdev)
    PcieSpeed platform_speed;    // amdgpu_device_partner_bandwidth speed
    uint8_t platform_width;      // ... and width (raw, or pcie_width_unknown)
    int32_t partner_index;       // UpstreamChain index used, -1 for none/device
};

// smu_smc_hw_setup: gen 0..3 is Gen1..Gen4 (from chipset bits only) and lane
// 1..6 is x1, x2, x4, x8, x12, x16; zero fields mean no bit was set.
struct PcieSmuLink final {
    uint8_t gen;
    uint8_t lane;
};

// Exact amdgpu_device_get_pcie_info. The device image must be a type-0 header
// and each chain entry a type-1 header (malformed otherwise); capability lists
// must be well formed. Absent PCIe capabilities read as zero like Linux.
CapStatus derive_pcie_info(ByteView device, const UpstreamChain& upstream,
                           PcieInfoPolicy policy, PcieInfoReport& output) noexcept;
PcieSmuLink encode_smu_pcie(PcieMasks masks) noexcept;
// derive_pcie_info followed by encode_smu_pcie.
CapStatus pcie_smu_link(ByteView device, const UpstreamChain& upstream, PcieInfoPolicy policy,
                        PcieSmuLink& output) noexcept;

// ---------------------------------------------------------------------------
// Upstream path discovery
// ---------------------------------------------------------------------------

// Dword configuration read on PCI segment 0. Returns false when the HAL
// returns fewer bytes; a function whose identity read fails is absent.
struct ConfigDwordReader final {
    void* context;
    bool (*read)(void* context, PciLocation location, uint16_t offset, uint32_t& value) noexcept;
};

struct UpstreamPath final {
    PciLocation bridges[pci_upstream_capacity];  // nearest first
    uint32_t count;
};

// Descends from root bus 0 through the unique type-1 function on each bus
// whose secondary..subordinate range contains device.bus until a bridge's
// secondary bus equals device.bus. absent when bus 0 has no such bridge
// (another root bus); malformed for ambiguous or non-advancing ranges;
// capacity_exceeded beyond pci_upstream_capacity levels.
CapStatus resolve_upstream_path(ConfigDwordReader reader, PciLocation device,
                                UpstreamPath& output) noexcept;

// ---------------------------------------------------------------------------
// pci_save_state / pci_restore_state
// ---------------------------------------------------------------------------

struct PciSavedState final {
    bool valid;
    uint8_t header_type;
    uint32_t header[16];          // dev->saved_config_space
    PcieCapability express;       // control words saved by pci_save_pcie_state
    PowerManagementCapability power;  // recorded only; Linux does not restore PMCSR
    MsiCapability msi;
    MsixCapability msix;
    bool extended;                // extended space was reachable at save time
    LtrCapability ltr;            // pci_save_ltr_state
    ResizableBarCapability rebar; // pci_restore_rebar_state input
};

// Linux pci_save_state on an image (dwords 0..15, PCIe controls, LTR) plus the
// MSI/MSI-X registers Linux keeps in msi_desc and the ReBAR control dwords.
// The 4096-byte image form is required for LTR/ReBAR; a 256-byte image saves
// none of them (extended false). Header types other than 0/1 are malformed.
CapStatus save_pci_state(ByteView config, PciSavedState& output) noexcept;

struct ConfigWrite final {
    uint16_t offset;
    uint8_t width;    // 2 or 4 bytes
    uint8_t retries;  // re-read/compare/1 ms delay rounds (BARs: 10)
    bool force;       // false: skip when the current value already matches
    uint32_t value;
};

constexpr size_t pci_restore_capacity = 48;

struct RestorePlan final {
    ConfigWrite writes[pci_restore_capacity];
    uint32_t count;
};

// Linux pci_restore_state order: LTR, PCIe controls (unconditional writes),
// ReBAR sizes, header dwords high to low (type 0: 15..10, BARs 9..4 with 10
// retries, 3..0; type 1: 15..12, 9..11 forced, 8..0; compare-before-write),
// then MSI (disabled, message, mask, saved control) and MSI-X (masked enable,
// saved control) when they were enabled. Extended writes appear only when the
// state was saved with extended space. Not restored: PASID/PRI/ATS/VC/DPC/PTM,
// AER, ASPM L1SS, ACS, SR-IOV and MSI-X table contents (BAR memory).
CapStatus plan_pci_restore(const PciSavedState& state, RestorePlan& output) noexcept;

} // namespace rx7900

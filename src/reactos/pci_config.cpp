/*
Module: rx7900.reactos.pci_config
Purpose: Bounded HAL configuration reads/writes for capability decoding, the
amdgpu PCIe link query and Linux-ordered save/restore.
Created: 2026-10-22
Architecture: Raw buffers exist only at the HAL call; every result is decoded by
src/common/pci_caps through ByteView. No PnP, VideoPort or interrupt state changes.
*/
#include "pci_config.hpp"

#include <intrin.h>

namespace rx7900::reactos {
namespace {

constexpr ULONG hal_chunk_bytes = 256;
constexpr ULONG probe_blocks = 15;
constexpr ULONG one_millisecond = 1000;

PCI_SLOT_NUMBER slot_number(PciLocation location) noexcept {
    PCI_SLOT_NUMBER slot{};
    slot.u.bits.DeviceNumber = location.slot;
    slot.u.bits.FunctionNumber = location.function;
    return slot;
}

bool valid_location(PciLocation location) noexcept {
    return location.domain == 0 && location.slot < 32 && location.function < 8;
}

// The only raw read. buffer holds bytes writable bytes owned by the caller;
// offset + bytes stays inside the 4 KiB space and bytes inside one HAL chunk.
// Offsets at or above 0x100 reach real registers only where the aliasing
// probe proved it; on the CF8/CFC HAL they return aliased low registers.
bool hal_read(PciLocation location, ULONG offset, uint8_t* buffer, ULONG bytes) noexcept {
    if (buffer == nullptr || bytes == 0 || bytes > hal_chunk_bytes ||
        offset >= pci_extended_config_bytes || bytes > pci_extended_config_bytes - offset) {
        return false;
    }

    return HalGetBusDataByOffset(PCIConfiguration, location.bus, slot_number(location).u.AsULONG,
                                 buffer, offset, bytes) == bytes;
}

bool hal_read_dword(PciLocation location, ULONG offset, uint32_t& value) noexcept {
    uint8_t bytes[4]{};

    if (!hal_read(location, offset, bytes, sizeof(bytes))) {
        return false;
    }

    return ByteView(bytes, sizeof(bytes)).read_u32(0, value);
}

bool hal_read_value(PciLocation location, ULONG offset, uint8_t width, uint32_t& value) noexcept {
    uint8_t bytes[4]{};
    const ByteView view(bytes, width);
    uint16_t half = 0;

    if ((width != 2 && width != 4) || !hal_read(location, offset, bytes, width)) {
        return false;
    }

    if (width == 4) {
        return view.read_u32(0, value);
    }

    view.read_u16(0, half);
    value = half;
    return true;
}

// The only raw write: an explicit little-endian encoding of width bytes.
bool hal_write_value(PciLocation location, ULONG offset, uint8_t width, uint32_t value) noexcept {
    uint8_t bytes[4]{};

    if (width != 2 && width != 4) {
        return false;
    }

    for (uint8_t index = 0; index < width; ++index) {
        bytes[index] = static_cast<uint8_t>(value >> (8u * index));
    }

    return HalSetBusDataByOffset(PCIConfiguration, location.bus, slot_number(location).u.AsULONG,
                                 bytes, offset, width) == width;
}

bool reader_callback(void*, PciLocation location, uint16_t offset, uint32_t& value) noexcept {
    return hal_read_dword(location, offset, value);
}

// pci_cfg_space_size_ext against the live mechanism; any failed read is unreachable.
bool probe_extended(PciLocation location, uint32_t identity) noexcept {
    ExtendedConfigProbe probe{};
    probe.identity = identity;

    for (ULONG block = 0; block < probe_blocks; ++block) {
        if (!hal_read_dword(location, (block + 1) * pci_config_bytes, probe.block_heads[block])) {
            return false;
        }
    }

    return extended_config_reachable(probe);
}

NTSTATUS to_ntstatus(CapStatus status) noexcept {
    switch (status) {
    case CapStatus::success: return STATUS_SUCCESS;
    case CapStatus::absent: return STATUS_NOT_FOUND;
    case CapStatus::truncated_input: return STATUS_DEVICE_DATA_ERROR;
    case CapStatus::capacity_exceeded: return STATUS_BUFFER_OVERFLOW;
    case CapStatus::extended_unavailable: return STATUS_NOT_SUPPORTED;
    default: return STATUS_DEVICE_CONFIGURATION_ERROR;
    }
}

bool owned_ready(const HardwareState& state) noexcept {
    return state.acquired && KeGetCurrentIrql() == PASSIVE_LEVEL;
}

CapStatus optional(CapStatus status) noexcept {
    return status == CapStatus::absent ? CapStatus::success : status;
}

} // namespace

ByteView config_view(const ConfigImage& image) noexcept {
    const uint32_t bytes = image.valid_bytes <= sizeof(image.bytes) ? image.valid_bytes : 0;
    return ByteView(image.bytes, bytes);
}

PciLocation owned_location(const HardwareState& state) noexcept {
    return {state.adapter.pci_domain, static_cast<uint8_t>(state.adapter.bus),
            static_cast<uint8_t>(state.adapter.slot), static_cast<uint8_t>(state.adapter.function)};
}

PcieInfoPolicy native_pcie_policy() noexcept {
    // "GenuineIntel" in EBX, EDX, ECX order.
    constexpr int genu = 0x756e6547;
    constexpr int inei = 0x49656e69;
    constexpr int ntel = 0x6c65746e;
    int registers[4]{};
    __cpuid(registers, 0);
    PcieInfoPolicy policy = linux_default_pcie_policy;
    policy.dynamic_switching = !(registers[1] == genu && registers[3] == inei && registers[2] == ntel);
    return policy;
}

NTSTATUS capture_config(PciLocation location, ConfigImage& image) noexcept {
    image.valid_bytes = 0;
    image.extended_reachable = false;
    image.location = location;

    if (KeGetCurrentIrql() != PASSIVE_LEVEL) {
        return STATUS_INVALID_DEVICE_STATE;
    }

    if (!valid_location(location)) {
        return STATUS_NOT_SUPPORTED;
    }

    if (!hal_read(location, 0, image.bytes, hal_chunk_bytes)) {
        return STATUS_NO_SUCH_DEVICE;
    }

    const ByteView legacy(image.bytes, pci_config_bytes);
    uint32_t identity = 0;
    legacy.read_u32(0, identity);

    if ((identity & 0xffffu) == 0xffffu || (identity & 0xffffu) == 0) {
        return STATUS_NO_SUCH_DEVICE;
    }

    image.valid_bytes = pci_config_bytes;

    if (!probe_extended(location, identity)) {
        return STATUS_SUCCESS;
    }

    for (ULONG offset = pci_config_bytes; offset < pci_extended_config_bytes; offset += hal_chunk_bytes) {
        // offset < 4096 - 255 by the loop bound, so the indexed element is in range.
        // The image keeps only the legacy prefix if any extended chunk fails.
        if (!hal_read(location, offset, &image.bytes[offset], hal_chunk_bytes)) {
            return STATUS_SUCCESS;
        }
    }

    image.valid_bytes = pci_extended_config_bytes;
    image.extended_reachable = true;
    return STATUS_SUCCESS;
}

NTSTATUS capture_upstream_chain(PciLocation device, UpstreamChain& chain) noexcept {
    chain.count = 0;

    if (KeGetCurrentIrql() != PASSIVE_LEVEL) {
        return STATUS_INVALID_DEVICE_STATE;
    }

    if (!valid_location(device)) {
        return STATUS_NOT_SUPPORTED;
    }

    UpstreamPath path{};
    const CapStatus status = resolve_upstream_path({nullptr, reader_callback}, device, path);

    if (status != CapStatus::success) {
        return to_ntstatus(status);
    }

    for (uint32_t index = 0; index < path.count && index < pci_upstream_capacity; ++index) {
        PciFunctionImage& bridge = chain.bridges[index];
        bridge.location = path.bridges[index];

        if (!hal_read(bridge.location, 0, bridge.bytes, hal_chunk_bytes)) {
            chain.count = 0;
            return STATUS_DEVICE_DATA_ERROR;
        }
    }

    chain.count = path.count;
    return STATUS_SUCCESS;
}

NTSTATUS query_pcie_smu_link(const HardwareState& state, PciConfigWorkspace& workspace,
                             PcieInfoReport& report, PcieSmuLink& link) noexcept {
    report = {};
    link = {0, 0};

    if (!owned_ready(state)) {
        return STATUS_INVALID_DEVICE_STATE;
    }

    const PciLocation location = owned_location(state);
    NTSTATUS status = capture_config(location, workspace.device);

    if (NT_SUCCESS(status)) {
        status = capture_upstream_chain(location, workspace.upstream);
    }

    if (!NT_SUCCESS(status)) {
        return status;
    }

    const CapStatus derived = derive_pcie_info(config_view(workspace.device), workspace.upstream,
                                               native_pcie_policy(), report);

    if (derived != CapStatus::success) {
        report = {};
        return to_ntstatus(derived);
    }

    link = encode_smu_pcie(report.masks);
    return STATUS_SUCCESS;
}

NTSTATUS query_capabilities(const HardwareState& state, PciConfigWorkspace& workspace,
                            PciCapabilityReport& report) noexcept {
    report = {};

    if (!owned_ready(state)) {
        return STATUS_INVALID_DEVICE_STATE;
    }

    const NTSTATUS captured = capture_config(owned_location(state), workspace.device);

    if (!NT_SUCCESS(captured)) {
        return captured;
    }

    const ByteView config = config_view(workspace.device);
    PciCapabilityReport decoded{};
    CapStatus status = optional(decode_pcie(config, decoded.express));

    if (status == CapStatus::success) {
        status = optional(decode_power_management(config, decoded.power));
    }

    if (status == CapStatus::success) {
        status = optional(decode_msi(config, decoded.msi));
    }

    if (status == CapStatus::success) {
        status = optional(decode_msix(config, decoded.msix));
    }

    if (status != CapStatus::success) {
        return to_ntstatus(status);
    }

    decoded.extended_reachable = workspace.device.extended_reachable;
    decoded.rebar_status = decode_resizable_bar(config, decoded.rebar);
    report = decoded;
    return STATUS_SUCCESS;
}

NTSTATUS save_state(const HardwareState& state, PciConfigWorkspace& workspace,
                    PciSavedState& saved) noexcept {
    saved = {};

    if (!owned_ready(state)) {
        return STATUS_INVALID_DEVICE_STATE;
    }

    const NTSTATUS captured = capture_config(owned_location(state), workspace.device);

    if (!NT_SUCCESS(captured)) {
        return captured;
    }

    return to_ntstatus(save_pci_state(config_view(workspace.device), saved));
}

NTSTATUS restore_state(const HardwareState& state, const PciSavedState& saved) noexcept {
    if (!owned_ready(state)) {
        return STATUS_INVALID_DEVICE_STATE;
    }

    const PciLocation location = owned_location(state);
    RestorePlan plan{};
    const CapStatus planned = plan_pci_restore(saved, plan);

    if (planned != CapStatus::success) {
        return to_ntstatus(planned);
    }

    if (!valid_location(location)) {
        return STATUS_NOT_SUPPORTED;
    }

    uint32_t identity = 0;

    if (!hal_read_dword(location, 0, identity) || identity != saved.header[0]) {
        return STATUS_NO_SUCH_DEVICE;
    }

    // Prove reachability again before a single write; never write aliased offsets.
    if (saved.extended && !probe_extended(location, identity)) {
        return STATUS_NOT_SUPPORTED;
    }

    bool verified = true;

    for (uint32_t index = 0; index < plan.count && index < pci_restore_capacity; ++index) {
        const ConfigWrite& write = plan.writes[index];
        uint32_t current = 0;

        if (write.offset + write.width > (saved.extended ? pci_extended_config_bytes : pci_config_bytes)) {
            return STATUS_DEVICE_CONFIGURATION_ERROR;
        }

        if (!write.force) {
            if (!hal_read_value(location, write.offset, write.width, current)) {
                return STATUS_DEVICE_DATA_ERROR;
            }

            if (current == write.value) {
                continue;
            }
        }

        // pci_restore_config_dword: write, then re-read and rewrite after 1 ms.
        for (uint8_t retries = write.retries;; --retries) {
            if (!hal_write_value(location, write.offset, write.width, write.value)) {
                return STATUS_DEVICE_DATA_ERROR;
            }

            if (retries == 0) {
                verified = verified && write.retries == 0;
                break;
            }

            if (!hal_read_value(location, write.offset, write.width, current)) {
                return STATUS_DEVICE_DATA_ERROR;
            }

            if (current == write.value) {
                break;
            }

            KeStallExecutionProcessor(one_millisecond);
        }
    }

    // Linux continues after an unverified BAR; the caller learns about it.
    return verified ? STATUS_SUCCESS : STATUS_DEVICE_CONFIGURATION_ERROR;
}

} // namespace rx7900::reactos

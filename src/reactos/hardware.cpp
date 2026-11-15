/*
Module: rx7900.reactos.hardware
Purpose: Validate and map device-owned resources using the actual VideoPort API.
Created: 2026-10-21
Architecture: Minimal native ownership layer; GPU readiness is never inferred.
*/
#include "hardware.hpp"

namespace rx7900::reactos {

VP_STATUS acquire(HardwareState& state, PVOID extension,
                  PVIDEO_PORT_CONFIG_INFO config) noexcept {
    if (extension == nullptr || config == nullptr || state.acquired ||
        state.registers != nullptr || VideoPortGetCurrentIrql() != PASSIVE_LEVEL ||
        config->Length < SIZE_OF_NT4_VIDEO_PORT_CONFIG_INFO ||
        config->AdapterInterfaceType != PCIBus || config->SystemIoBusNumber > UINT8_MAX) {
        return ERROR_INVALID_PARAMETER;
    }

    VIDEO_ACCESS_RANGE ranges[6]{};
    ULONG slot = 0;
    VP_STATUS status = ERROR_DEV_NOT_EXIST;

    // Every Navi31 GPU function; VideoPort matches one vendor/device pair per call.
    for (const uint16_t id : navi31_device_ids) {
        USHORT vendor = amd_vendor_id;
        USHORT device = id;
        slot = 0;
        status = VideoPortGetAccessRanges(extension, 0, nullptr, RTL_NUMBER_OF(ranges), ranges,
                                          &vendor, &device, &slot);

        if (status == NO_ERROR) {
            break;
        }
    }

    if (status != NO_ERROR) {
        return status;
    }

    uint8_t bytes[pci_header_bytes]{};
    const ULONG received = VideoPortGetBusData(extension, PCIConfiguration, slot,
        bytes, 0, sizeof(bytes));
    PCI_SLOT_NUMBER location{};
    location.u.AsULONG = slot;
    Adapter adapter{};

    if (received != sizeof(bytes) || decode_adapter(ByteView(bytes, received),
        {0, static_cast<uint8_t>(config->SystemIoBusNumber),
         static_cast<uint8_t>(location.u.bits.DeviceNumber),
         static_cast<uint8_t>(location.u.bits.FunctionNumber)}, adapter) != ParseStatus::success) {
        return ERROR_DEV_NOT_EXIST;
    }

    const auto& bar = adapter.bars[5];
    const auto& vram_bar = adapter.bars[0];
    const auto& doorbell_bar = adapter.bars[2];
    PHYSICAL_ADDRESS vram_range{};
    ULONG vram_range_bytes = 0;
    PHYSICAL_ADDRESS doorbell_range{};
    ULONG doorbell_range_bytes = 0;

    for (const auto& range : ranges) {
        if (range.RangeInIoSpace != 0 || range.RangeStart.QuadPart <= 0 || range.RangeLength == 0) {
            continue;
        }

        const uint64_t start = static_cast<uint64_t>(range.RangeStart.QuadPart);

        if (vram_bar.kind == static_cast<uint32_t>(BarKind::memory64) && vram_bar.bus_address != 0 &&
            start == vram_bar.bus_address) {
            vram_range = range.RangeStart;
            vram_range_bytes = range.RangeLength;
        }

        if (doorbell_bar.kind == static_cast<uint32_t>(BarKind::memory64) &&
            doorbell_bar.bus_address != 0 && start == doorbell_bar.bus_address) {
            doorbell_range = range.RangeStart;
            doorbell_range_bytes = range.RangeLength;
        }
    }

    if (bar.kind != static_cast<uint32_t>(BarKind::memory32) || bar.bus_address == 0) {
        return ERROR_INVALID_PARAMETER;
    }

    for (const auto& range : ranges) {
        if (range.RangeInIoSpace != 0 || range.RangeStart.QuadPart < 0 ||
            static_cast<uint64_t>(range.RangeStart.QuadPart) != bar.bus_address ||
            range.RangeLength < sizeof(ULONG) || range.RangeShareable != 0 ||
            range.RangeLength > UINT64_MAX - bar.bus_address) {
            continue;
        }

        // VideoPort translates this bus-relative range; PCI BAR bus addresses
        // are never cast to CPU pointers or used as CPU physical addresses.
        PVOID mapped = VideoPortGetDeviceBase(extension, range.RangeStart,
            range.RangeLength, VIDEO_MEMORY_SPACE_MEMORY);

        if (mapped == nullptr) {
            return ERROR_NOT_ENOUGH_MEMORY;
        }

        state.adapter = adapter;
        state.registers = static_cast<PULONG>(mapped);
        state.register_bytes = range.RangeLength;
        state.pci_slot = slot;
        state.vram_range = vram_range;
        state.vram_range_bytes = vram_range_bytes;
        state.doorbell_range = doorbell_range;
        state.doorbell_range_bytes = doorbell_range_bytes;
        state.interrupt_level = config->BusInterruptLevel;
        state.interrupt_vector = config->BusInterruptVector;
        state.interrupt_mode = config->InterruptMode;
        state.interrupt_assigned = config->BusInterruptLevel != 0 || config->BusInterruptVector != 0;
        state.acquired = true;
        return NO_ERROR;
    }

    return ERROR_DEV_NOT_EXIST;
}

void release(HardwareState& state, PVOID extension) noexcept {
    // Caller has retired ISR/DPC/queue users before releasing the mapping.
    if (state.doorbells != nullptr) {
        VideoPortFreeDeviceBase(extension, state.doorbells);
    }

    if (state.registers != nullptr) {
        VideoPortFreeDeviceBase(extension, state.registers);
    }

    DriverBlob::release(state.rom);
    state.rom_image_bytes = 0;
    state.adapter = {};
    state.registers = nullptr;
    state.register_bytes = 0;
    state.pci_slot = 0;
    state.vram_range = {};
    state.vram_range_bytes = 0;
    state.doorbell_range = {};
    state.doorbell_range_bytes = 0;
    state.doorbells = nullptr;
    state.doorbell_bytes = 0;
    state.interrupt_level = 0;
    state.interrupt_vector = 0;
    state.interrupt_mode = LevelSensitive;
    state.interrupt_assigned = false;
    state.acquired = false;
}

VP_STATUS map_doorbells(HardwareState& state, PVOID extension) noexcept {
    constexpr ULONG maximum_doorbell_bytes = 8u << 20;

    if (!state.acquired || extension == nullptr || state.doorbells != nullptr ||
        VideoPortGetCurrentIrql() != PASSIVE_LEVEL || state.doorbell_range_bytes == 0) {
        return ERROR_INVALID_PARAMETER;
    }

    const ULONG bytes = state.doorbell_range_bytes < maximum_doorbell_bytes ?
                        state.doorbell_range_bytes : maximum_doorbell_bytes;
    PVOID mapped = VideoPortGetDeviceBase(extension, state.doorbell_range, bytes,
                                          VIDEO_MEMORY_SPACE_MEMORY);

    if (mapped == nullptr) {
        return ERROR_NOT_ENOUGH_MEMORY;
    }

    state.doorbells = static_cast<PULONG>(mapped);
    state.doorbell_bytes = bytes;
    return NO_ERROR;
}

bool write_doorbell32(HardwareState& state, uint32_t dword_index, uint32_t value) noexcept {
    if (state.doorbells == nullptr || dword_index >= state.doorbell_bytes / sizeof(ULONG)) {
        return false;
    }

    VideoPortWriteRegisterUlong(state.doorbells + dword_index, value);
    return true;
}

bool write_doorbell64(HardwareState& state, uint32_t dword_index, uint64_t value) noexcept {
    if (state.doorbells == nullptr || dword_index >= state.doorbell_bytes / sizeof(ULONG) - 1) {
        return false;
    }

    VideoPortWriteRegisterUlong(state.doorbells + dword_index, static_cast<ULONG>(value));
    VideoPortWriteRegisterUlong(state.doorbells + dword_index + 1, static_cast<ULONG>(value >> 32));
    return true;
}

VP_STATUS map_vram_temporary(const HardwareState& state, PVOID extension, uint64_t offset,
                             ULONG bytes, PUCHAR& output) noexcept {
    output = nullptr;

    if (!state.acquired || extension == nullptr || VideoPortGetCurrentIrql() != PASSIVE_LEVEL ||
        bytes == 0 || state.vram_range_bytes == 0 || offset > state.vram_range_bytes ||
        bytes > state.vram_range_bytes - offset ||
        static_cast<uint64_t>(state.vram_range.QuadPart) > INT64_MAX - offset) {
        return ERROR_INVALID_PARAMETER;
    }

    PHYSICAL_ADDRESS start = state.vram_range;
    start.QuadPart += static_cast<LONGLONG>(offset);
    // VIDEO_MEMORY_SPACE_MEMORY without P6CACHE requests an uncached mapping.
    PVOID mapped = VideoPortGetDeviceBase(extension, start, bytes, VIDEO_MEMORY_SPACE_MEMORY);

    if (mapped == nullptr) {
        return ERROR_NOT_ENOUGH_MEMORY;
    }

    output = static_cast<PUCHAR>(mapped);
    return NO_ERROR;
}

void unmap_vram_temporary(PVOID extension, PUCHAR mapping) noexcept {
    if (mapping != nullptr) {
        VideoPortFreeDeviceBase(extension, mapping);
    }
}

bool enable_bus_master(const HardwareState& state, PVOID extension) noexcept {
    constexpr ULONG command_offset = 0x04;
    constexpr uint16_t bus_master = 0x0004;

    if (!state.acquired || extension == nullptr || VideoPortGetCurrentIrql() != PASSIVE_LEVEL) {
        return false;
    }

    uint8_t bytes[2]{};

    if (VideoPortGetBusData(extension, PCIConfiguration, state.pci_slot, bytes,
                            command_offset, sizeof(bytes)) != sizeof(bytes)) {
        return false;
    }

    uint16_t command = 0;
    ByteView(bytes, sizeof(bytes)).read_u16(0, command);
    command = static_cast<uint16_t>(command | bus_master);
    bytes[0] = static_cast<uint8_t>(command);
    bytes[1] = static_cast<uint8_t>(command >> 8);

    if (VideoPortSetBusData(extension, PCIConfiguration, state.pci_slot, bytes,
                            command_offset, sizeof(bytes)) != sizeof(bytes) ||
        VideoPortGetBusData(extension, PCIConfiguration, state.pci_slot, bytes,
                            command_offset, sizeof(bytes)) != sizeof(bytes)) {
        return false;
    }

    return (bytes[0] & bus_master) != 0;
}

bool read_register(HardwareState& state, uint32_t byte_offset, uint32_t& value) noexcept {
    if (!state.acquired || state.registers == nullptr || (byte_offset & 3u) != 0 ||
        byte_offset > state.register_bytes ||
        sizeof(ULONG) > state.register_bytes - byte_offset) {
        return false;
    }

    // Only this checked MMIO boundary forms a register pointer. VideoPort owns
    // memory ordering and volatile device access; ordinary RAM reads are invalid.
    value = VideoPortReadRegisterUlong(state.registers + byte_offset / sizeof(ULONG));
    return true;
}

bool write_register(HardwareState& state, uint32_t byte_offset, uint32_t value) noexcept {
    if (!state.acquired || state.registers == nullptr || (byte_offset & 3u) != 0 ||
        byte_offset > state.register_bytes ||
        sizeof(ULONG) > state.register_bytes - byte_offset) {
        return false;
    }

    VideoPortWriteRegisterUlong(state.registers + byte_offset / sizeof(ULONG), value);
    return true;
}

NTSTATUS acquire_atom_rom(HardwareState& state) noexcept {
    if (!state.acquired) { return STATUS_INVALID_DEVICE_STATE; }
    if (state.rom.bytes().size() != 0) { return STATUS_DEVICE_BUSY; }

    DriverBlob candidate;
    const NTSTATUS status = candidate.load_atom_rom();

    if (!NT_SUCCESS(status)) { return status; }

    AtomImage image{};

    if (validate_atom_image(candidate.bytes(), state.adapter.vendor_id,
        state.adapter.device_id, image) != ParseStatus::success) {
        return STATUS_INVALID_IMAGE_FORMAT;
    }
    if (!candidate.detach(state.rom)) { return STATUS_INVALID_DEVICE_STATE; }

    state.rom_image_bytes = image.image_bytes;
    return STATUS_SUCCESS;
}

ByteView atom_rom(const HardwareState& state) noexcept {
    ByteView image(nullptr, 0);
    state.rom.bytes().subview(0, state.rom_image_bytes, image);
    return image;
}

} // namespace rx7900::reactos

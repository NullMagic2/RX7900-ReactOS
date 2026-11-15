/*
Module: rx7900.reactos.driver
Purpose: Provide read-only PCI discovery through native ReactOS NT/HAL APIs.
Created: 2026-10-21
Architecture: Demand-start diagnostic service, separate from display ownership.
*/
#include <ntddk.h>

#include "pci.hpp"
#include "lifetime.hpp"

namespace {

using rx7900::Adapter;
using rx7900::ByteView;
using rx7900::ParseStatus;
using rx7900::QueryRequest;
using rx7900::Snapshot;

struct DeviceState final {
    Snapshot snapshot;
};
static_assert(alignof(DeviceState) <= MEMORY_ALLOCATION_ALIGNMENT,
              "I/O manager extension must satisfy the state alignment");

constexpr ULONG pci_bus_count = 256;
constexpr ULONG pci_slot_count = 32;
constexpr ULONG pci_function_count = 8;

// PASSIVE_LEVEL, DriverEntry only. HAL fills a caller-owned 64-byte array.
// Segment zero only; this legacy read API does not enumerate other PCI domains.
bool read_config(ULONG bus, ULONG slot, ULONG function,
                 uint8_t (&bytes)[rx7900::pci_header_bytes]) noexcept {
    PCI_SLOT_NUMBER slot_number{};
    slot_number.u.bits.DeviceNumber = slot;
    slot_number.u.bits.FunctionNumber = function;
    const ULONG received = HalGetBusDataByOffset(PCIConfiguration, bus,
        slot_number.u.AsULONG, bytes, 0, static_cast<ULONG>(sizeof(bytes)));
    uint16_t vendor = 0;

    return received == sizeof(bytes) &&
           ByteView(bytes, sizeof(bytes)).read_u16(rx7900::pci_vendor_offset, vendor) &&
           vendor != 0 && vendor != UINT16_MAX;
}

void record_function(ByteView config, rx7900::PciLocation location,
                     Snapshot& snapshot) noexcept {
    ++snapshot.pci_functions_seen;
    Adapter adapter{};

    if (rx7900::decode_adapter(config, location, adapter) != ParseStatus::success) {
        return;
    }
    if (snapshot.adapter_count == rx7900::adapter_capacity) {
        ++snapshot.omitted_adapters;
        return;
    }

    snapshot.adapters[snapshot.adapter_count] = adapter;
    ++snapshot.adapter_count;
}

void collect_snapshot(Snapshot& snapshot) noexcept {
    snapshot = {};
    snapshot.size = sizeof(snapshot);
    snapshot.version = rx7900::protocol_version;
    snapshot.feature_flags = rx7900::feature_read_only_pci;

    for (ULONG bus = 0; bus < pci_bus_count; ++bus) {
        for (ULONG slot = 0; slot < pci_slot_count; ++slot) {
            uint8_t config[rx7900::pci_header_bytes]{};

            if (!read_config(bus, slot, 0, config)) {
                continue;
            }

            const rx7900::PciLocation location{0, static_cast<uint8_t>(bus),
                                              static_cast<uint8_t>(slot), 0};
            const ByteView view(config, sizeof(config));
            uint8_t header_type = 0;
            view.read_u8(rx7900::pci_header_type_offset, header_type);
            record_function(view, location, snapshot);

            if ((header_type & rx7900::pci_multifunction) == 0) {
                continue;
            }

            for (ULONG function = 1; function < pci_function_count; ++function) {
                if (read_config(bus, slot, function, config)) {
                    auto function_location = location;
                    function_location.function = static_cast<uint8_t>(function);
                    record_function(ByteView(config, sizeof(config)),
                                    function_location, snapshot);
                }
            }
        }
    }
}

NTSTATUS complete_irp(PIRP irp, NTSTATUS status, ULONG_PTR bytes = 0) noexcept {
    irp->IoStatus.Status = status;
    irp->IoStatus.Information = bytes;
    IoCompleteRequest(irp, IO_NO_INCREMENT);
    return status;
}

extern "C" NTSTATUS NTAPI dispatch_irp(PDEVICE_OBJECT device, PIRP irp) {
    // The I/O manager owns this IRP until complete_irp; no reference escapes.
    const PIO_STACK_LOCATION stack = IoGetCurrentIrpStackLocation(irp);

    switch (stack->MajorFunction) {
    case IRP_MJ_CREATE:
    case IRP_MJ_CLOSE:
    case IRP_MJ_CLEANUP:
        return complete_irp(irp, STATUS_SUCCESS);
    case IRP_MJ_DEVICE_CONTROL:
        break;
    default:
        return complete_irp(irp, STATUS_INVALID_DEVICE_REQUEST);
    }

    const auto& control = stack->Parameters.DeviceIoControl;

    if (control.IoControlCode != rx7900::ioctl_query_snapshot) {
        return complete_irp(irp, STATUS_INVALID_DEVICE_REQUEST);
    }
    if (control.InputBufferLength < sizeof(QueryRequest) ||
        control.OutputBufferLength < sizeof(Snapshot)) {
        return complete_irp(irp, STATUS_BUFFER_TOO_SMALL);
    }
    if (irp->AssociatedIrp.SystemBuffer == nullptr) {
        return complete_irp(irp, STATUS_INVALID_PARAMETER);
    }

    // METHOD_BUFFERED supplies an aligned kernel buffer at least as large as
    // both validated lengths. Copy the request before overwriting that buffer.
    QueryRequest request{};
    RtlCopyMemory(&request, irp->AssociatedIrp.SystemBuffer, sizeof(request));

    if (request.size != sizeof(QueryRequest) ||
        request.version != rx7900::protocol_version) {
        return complete_irp(irp, STATUS_REVISION_MISMATCH);
    }

    // IoCreateDevice allocated this extension for the device lifetime. The
    // immutable initialized snapshot is copied, never exposed as a pointer.
    const auto* state = static_cast<const DeviceState*>(device->DeviceExtension);
    RtlCopyMemory(irp->AssociatedIrp.SystemBuffer, &state->snapshot,
                  sizeof(state->snapshot));
    return complete_irp(irp, STATUS_SUCCESS, sizeof(state->snapshot));
}

extern "C" void NTAPI unload_driver(PDRIVER_OBJECT driver) {
    // No timers, work items, interrupts or pending IRPs reference this device.
    UNICODE_STRING link{};
    RtlInitUnicodeString(&link, rx7900::dos_device_name);
    IoDeleteSymbolicLink(&link);

    if (driver->DeviceObject != nullptr) {
        IoDeleteDevice(driver->DeviceObject);
    }
}

} // namespace

extern "C" NTSTATUS NTAPI DriverEntry(PDRIVER_OBJECT driver,
                                     PUNICODE_STRING registry_path) {
    UNREFERENCED_PARAMETER(registry_path);
    UNICODE_STRING name{};
    RtlInitUnicodeString(&name, rx7900::nt_device_name);
    PDEVICE_OBJECT device = nullptr;
    NTSTATUS status = IoCreateDevice(driver, sizeof(DeviceState), &name,
        FILE_DEVICE_UNKNOWN, FILE_DEVICE_SECURE_OPEN, FALSE, &device);

    if (!NT_SUCCESS(status)) {
        return status;
    }

    auto* state = rx7900::reactos::construct_device_state<DeviceState>(device->DeviceExtension);
    collect_snapshot(state->snapshot);

    for (ULONG index = 0; index <= IRP_MJ_MAXIMUM_FUNCTION; ++index) {
        driver->MajorFunction[index] = dispatch_irp;
    }

    driver->DriverUnload = unload_driver;
    device->Flags |= DO_BUFFERED_IO;
    UNICODE_STRING link{};
    RtlInitUnicodeString(&link, rx7900::dos_device_name);
    status = IoCreateSymbolicLink(&link, &name);

    if (!NT_SUCCESS(status)) {
        IoDeleteDevice(device);
        return status;
    }

    device->Flags &= ~DO_DEVICE_INITIALIZING;
    DbgPrint("RX7900Probe: read-only snapshot, %lu adapters, %lu PCI functions\n",
        static_cast<ULONG>(state->snapshot.adapter_count),
        static_cast<ULONG>(state->snapshot.pci_functions_seen));
    return STATUS_SUCCESS;
}

/*
Module: rx7900.common.protocol
Purpose: Define the one pointer-free diagnostic ABI for driver and client.
Created: 2026-10-21
Architecture: Fixed capacity and explicit alignment across i386 and AMD64.
*/
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace rx7900 {

constexpr uint32_t protocol_version = 1;
constexpr uint32_t adapter_capacity = 16;
constexpr uint32_t pci_bar_count = 6;
constexpr uint32_t feature_read_only_pci = 1;

// CTL_CODE(FILE_DEVICE_UNKNOWN, 0x800, METHOD_BUFFERED, FILE_READ_ACCESS).
constexpr uint32_t ioctl_query_snapshot = (0x22u << 16) | (1u << 14) | (0x800u << 2);

constexpr wchar_t service_name[] = L"RX7900Probe";
constexpr wchar_t driver_filename[] = L"rx7900probe.sys";
constexpr wchar_t nt_device_name[] = L"\\Device\\RX7900Probe";
constexpr wchar_t dos_device_name[] = L"\\DosDevices\\RX7900Probe";
constexpr wchar_t win32_device_name[] = L"\\\\.\\RX7900Probe";

enum class BarKind : uint32_t {
    unused = 0,
    io = 1,
    memory32 = 2,
    memory64 = 3,
    continuation = 4
};

constexpr uint32_t bar_prefetchable = 1;

struct alignas(8) PciBar final {
    uint64_t bus_address;
    uint32_t kind;
    uint32_t flags;
};

struct alignas(8) Adapter final {
    uint32_t pci_domain;
    uint8_t bus;
    uint8_t slot;
    uint8_t function;
    uint8_t revision;
    uint16_t vendor_id;
    uint16_t device_id;
    uint16_t subsystem_vendor_id;
    uint16_t subsystem_device_id;
    uint32_t class_code;
    uint32_t reserved;
    PciBar bars[pci_bar_count];
};

struct QueryRequest final {
    uint32_t size;
    uint32_t version;
};

struct alignas(8) Snapshot final {
    uint32_t size;
    uint32_t version;
    uint32_t adapter_count;
    uint32_t omitted_adapters;
    uint32_t pci_functions_seen;
    uint32_t feature_flags;
    Adapter adapters[adapter_capacity];
};

static_assert(sizeof(PciBar) == 16, "Stable PCI BAR ABI");
static_assert(offsetof(Adapter, bars) == 24, "Stable adapter fields");
static_assert(sizeof(Adapter) == 120, "Stable adapter ABI");
static_assert(sizeof(QueryRequest) == 8, "Stable query input ABI");
static_assert(offsetof(Snapshot, adapters) == 24, "Stable snapshot fields");
static_assert(sizeof(Snapshot) == 1944, "Stable snapshot ABI");

} // namespace rx7900

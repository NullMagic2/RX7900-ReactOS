/*
Module: lightning.reactos.abi
Purpose: Share NT5 capabilities, bounded QLV exchange ABI and service identity.
Created: 2026-10-21
Architecture: Fixed wire ABI; renderer sessions own every returned user mapping.
*/
#pragma once

#include "transport.h"

namespace lightning::reactos {

constexpr uint16_t pci_vendor_id = 0x1234;
constexpr uint16_t pci_device_id = 0x11f1;
constexpr uint32_t ioctl_query_capabilities = 0x00226400;
constexpr uint32_t ioctl_exchange = 0x0022e404;
constexpr uint32_t capabilities_version = 1;
constexpr uint32_t capabilities_observed = 1;
constexpr uint32_t capability_renderer_exchange = 2;
// ioctl_exchange_batch (METHOD_BUFFERED, advertised by capability_exchange_batch):
// input is 1..exchange_batch_packets complete QLV request packets back to back
// (header plus payload_bytes each, at most QLV_HEADER_BYTES + QLV_MAX_PAYLOAD_BYTES
// in total). Every packet but the last is a SUBMIT or WRITE_SYNC of at most
// exchange_batch_packet_bytes; the last is any operation with a header-only reply
// (not OPEN, CLOSE or GET_CAPS). Packets run in order as separate exchanges; the
// output receives one QLV_HEADER_BYTES reply per executed packet. Execution stops
// after the first reply whose status is not success or the first failed exchange
// (which fails the IOCTL only when it is the first packet).
constexpr uint32_t ioctl_exchange_batch = 0x0022e408;
constexpr uint32_t capability_exchange_batch = 4;
constexpr uint32_t exchange_batch_packets = 8;
constexpr uint32_t exchange_batch_packet_bytes = static_cast<uint32_t>(QLV_RING_SLOT_BYTES);
constexpr wchar_t service_name[] = L"LightningNT5";
constexpr wchar_t driver_filename[] = L"lightningnt5.sys";
constexpr wchar_t nt_device_name[] = L"\\Device\\LightningNT5";
constexpr wchar_t dos_device_name[] = L"\\DosDevices\\LightningNT5";
constexpr wchar_t win32_device_name[] = L"\\\\.\\LightningNT5";
constexpr wchar_t hardware_id[] = L"PCI\\VEN_1234&DEV_11F1";

struct LightningCapabilities final {
    uint32_t size;
    uint32_t version;
    uint32_t flags;
    uint32_t transport_capabilities;
    QLV_CAPS caps;
};
static_assert(sizeof(QLV_HEADER) == QLV_HEADER_BYTES, "Generated QLV header ABI must remain64bytes");
static_assert(sizeof(QLV_CAPS) == QLV_CAPS_BYTES, "Generated capset ABI must remain160bytes");
static_assert(sizeof(LightningCapabilities) == 176, "Capabilities IOCTL ABI must be fixed width");
static_assert(offsetof(LightningCapabilities, caps) == 16, "Capset offset is part of IOCTL ABI");

// Every flag bit the NT5 filter can set in an ioctl_query_capabilities reply:
// the observed host exchange itself, and renderer exchange plus exchange
// batching, which lightning_driver.cpp advertises together once its renderer
// is ready. A reply carrying any other bit is from an unknown ABI revision.
constexpr uint32_t known_capability_flags =
    capabilities_observed | capability_renderer_exchange | capability_exchange_batch;

constexpr bool valid_capability_reply(const LightningCapabilities& caps, uint64_t returned) noexcept {
    return returned == sizeof(caps) && caps.size == sizeof(caps) && caps.version == capabilities_version &&
        (caps.flags & capabilities_observed) != 0 && (caps.flags & ~known_capability_flags) == 0;
}

} // namespace lightning::reactos

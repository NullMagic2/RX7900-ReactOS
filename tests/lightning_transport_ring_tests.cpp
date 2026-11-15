/*
Module: lightning.tests.transport_ring
Purpose: Exercise production NativeTransport over a modeled Lightning device: mailbox and shared ring.
Created: 2026-10-23
Architecture: Exact NT SDK types with bounded OS fixtures (PCI bus interface, DMA adapter,
MMIO through READ/WRITE_REGISTER_ULONG) and a synchronous device model that follows
hw/misc/lightning-venus.c and the Venus host's reply rules. Counts every MMIO access.
No physical device, interrupt, scheduler or VM is simulated.
*/
#include <stdint.h>
#include <ntifs.h>
#include "lightning_transport.hpp"
#include "lifetime.hpp"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace {
using namespace lightning::reactos;
unsigned checks = 0, failures = 0;
void check(bool value, const char* text) {
    ++checks;
    if (!value) { ++failures; printf("FAIL %s\n", text); }
}

// Session tokens the fixture's renderer would know (reset with the world).
uint64_t known[32]{};

// ---- Guest physical memory: common buffers at fake 32-bit bus addresses.
struct Allocation final { uint32_t address; uint8_t* memory; uint32_t bytes; bool live; };
Allocation allocations[16]{};
uint32_t next_address = 0x100000;
unsigned common_frees = 0;

uint8_t* guest(uint64_t address, uint64_t bytes) {
    for (const auto& allocation : allocations) {
        if (allocation.live && address >= allocation.address &&
            address + bytes <= uint64_t{allocation.address} + allocation.bytes) {
            return allocation.memory + (address - allocation.address);
        }
    }
    return nullptr;
}
uint32_t g32(uint64_t address) { uint32_t v = 0; memcpy(&v, guest(address, 4), 4); return v; }
void p32(uint64_t address, uint32_t v) { memcpy(guest(address, 4), &v, 4); }
void p64(uint64_t address, uint64_t v) { memcpy(guest(address, 8), &v, 8); }
uint16_t h16(const uint8_t* b, size_t o) { uint16_t v; memcpy(&v, b + o, 2); return v; }
uint32_t h32(const uint8_t* b, size_t o) { uint32_t v; memcpy(&v, b + o, 4); return v; }
uint64_t h64(const uint8_t* b, size_t o) { uint64_t v; memcpy(&v, b + o, 8); return v; }
void s32(uint8_t* b, size_t o, uint32_t v) { memcpy(b + o, &v, 4); }
void s64(uint8_t* b, size_t o, uint64_t v) { memcpy(b + o, &v, 8); }

// ---- The Venus host behind QEMU (wire_service.rs reply rules, bounded).
struct HostSync final { uint64_t token = 0, context = 0, value = 0; int slot = -1; };
struct Host final {
    bool publish = true;
    bool fail_submits = false;
    bool fail_writes = false;
    uint64_t next_context = UINT64_C(0x100000001), next_sync = UINT64_C(0x100000100);
    HostSync syncs[32]{};
    unsigned operations[32]{};
    uint16_t order[64]{};
    unsigned order_count = 0;
    uint32_t last_create_flags = 0;
    int next_slot = 0;
} host;

// ---- The device (lightning-venus.c), synchronous: a doorbell consumes the
// ring immediately unless deferred, in which case it runs at the next sleep.
struct Device final {
    bool ring_capable = true, deferred = false, error_next = false, posted_writes = false;
    uint32_t status = QLV_MMIO_IDLE, response_bytes = 0, reset_sleeps = 0;
    uint64_t request = 0, response = 0, staged_control = 0, staged_data = 0;
    uint32_t request_bytes = 0, response_capacity = 0;
    bool registered = false, idle = true, work_pending = false;
    uint32_t ring_state = QLV_RING_OFF, head = 0, retired = 0, posted_errors = 0;
    uint64_t control = 0, data = 0, ring_response = 0;
    uint32_t ring_capacity = 0;
    uint32_t sequence[QLV_RING_TIMELINE_SLOTS]{};
    unsigned reads = 0, writes = 0, doorbells = 0, mailbox_submits = 0, registrations = 0, ring_requests = 0, register_commands = 0;
} device;

void publish_value(const HostSync& sync) {
    if (sync.slot < 0 || !device.registered || device.ring_state != QLV_RING_ACTIVE) { return; }
    const uint64_t base = device.control + QLV_RING_TIMELINE_OFFSET + uint64_t(sync.slot) * QLV_RING_TIMELINE_SLOT_BYTES;
    uint32_t& seq = device.sequence[sync.slot];
    p32(base, ++seq);
    p32(base + QLV_RING_TIMELINE_NATIVE_OFFSET, static_cast<uint32_t>(sync.token));
    p64(base + QLV_RING_TIMELINE_VALUE_OFFSET, sync.value);
    p32(base, ++seq);
}
HostSync* find_sync(uint64_t token) {
    for (auto& sync : host.syncs) { if (sync.token == token && token != 0) { return &sync; } }
    return nullptr;
}

// One request in, one reply out (header plus optional payload).
uint32_t serve(const uint8_t* request, uint32_t bytes, uint8_t* reply) {
    memcpy(reply, request, QLV_HEADER_BYTES);
    const uint16_t op = h16(request, QLV_OPERATION_OFFSET);
    s32(reply, QLV_STATUS_OFFSET, QLV_STATUS_SUCCESS);
    s32(reply, QLV_PAYLOAD_BYTES_OFFSET, 0);
    s64(reply, QLV_VALUE_OFFSET, 0);
    if (op < 32) { ++host.operations[op]; }
    if (host.order_count < 64) { host.order[host.order_count++] = op; }
    uint32_t payload = 0;
    switch (op) {
    case QLV_OPEN:
        s64(reply, QLV_CONTEXT_OFFSET, host.next_context++);
        s32(reply, QLV_FLAGS_OFFSET, static_cast<uint32_t>(QLV_HOST_WATCHES | QLV_HOST_APERTURE_SIZE |
            (host.publish ? QLV_HOST_SYNC_PUBLISH : 0)));
        s64(reply, QLV_BYTES_OFFSET, QLV_APERTURE_BYTES);
        break;
    case QLV_GET_CAPS:
        payload = QLV_CAPS_BYTES;
        memset(reply + QLV_HEADER_BYTES, 0, payload);
        s64(reply, QLV_BYTES_OFFSET, payload);
        break;
    case QLV_CREATE_SYNC: {
        host.last_create_flags = h32(request, QLV_FLAGS_OFFSET);
        HostSync* sync = &host.syncs[0];
        for (auto& candidate : host.syncs) { if (candidate.token == 0) { sync = &candidate; break; } }
        sync->token = host.next_sync++;
        sync->context = h64(request, QLV_CONTEXT_OFFSET);
        sync->value = h64(request, QLV_VALUE_OFFSET);
        sync->slot = (host.last_create_flags & QLV_SYNC_PUBLISH) != 0 ? host.next_slot++ : -1;
        s64(reply, QLV_RESOURCE_OFFSET, sync->token);
        s64(reply, QLV_OFFSET_OFFSET, sync->slot >= 0 ? uint64_t(sync->slot) + 1 : 0);
        s32(reply, QLV_FLAGS_OFFSET, sync->slot >= 0 ? static_cast<uint32_t>(QLV_SYNC_PUBLISH) : 0);
        publish_value(*sync); // SYNC_VALUE precedes the reply
        break;
    }
    case QLV_READ_SYNC: {
        HostSync* sync = find_sync(h64(request, QLV_RESOURCE_OFFSET));
        s64(reply, QLV_VALUE_OFFSET, sync != nullptr ? sync->value : 0);
        if (sync == nullptr) { s32(reply, QLV_STATUS_OFFSET, QLV_STATUS_INVALID); }
        break;
    }
    case QLV_WRITE_SYNC: {
        HostSync* sync = find_sync(h64(request, QLV_RESOURCE_OFFSET));
        if (host.fail_writes) { s32(reply, QLV_STATUS_OFFSET, QLV_STATUS_INVALID); break; }
        if (sync != nullptr) { sync->value = h64(request, QLV_VALUE_OFFSET); publish_value(*sync); }
        break;
    }
    case QLV_WAIT_SYNCS: {
        const uint64_t count = h64(request, QLV_BYTES_OFFSET);
        uint64_t reached = 0;
        for (uint64_t index = 0; index < count; ++index) {
            HostSync* sync = find_sync(h64(request, QLV_HEADER_BYTES + index * 16));
            reached += sync != nullptr && sync->value >= h64(request, QLV_HEADER_BYTES + index * 16 + 8);
        }
        const bool any = (h32(request, QLV_FLAGS_OFFSET) & QLV_WAIT_ANY) != 0;
        s32(reply, QLV_STATUS_OFFSET, reached == count || (any && reached != 0) ? QLV_STATUS_SUCCESS : QLV_STATUS_TIMEOUT);
        break;
    }
    case QLV_DESTROY_SYNC: {
        HostSync* sync = find_sync(h64(request, QLV_RESOURCE_OFFSET));
        if (sync != nullptr) { *sync = HostSync{}; }
        break;
    }
    case QLV_SUBMIT:
        if (host.fail_submits) { s32(reply, QLV_STATUS_OFFSET, QLV_STATUS_INVALID); }
        break;
    default:
        break;
    }
    (void)bytes;
    s32(reply, QLV_PAYLOAD_BYTES_OFFSET, payload);
    return QLV_HEADER_BYTES + payload;
}

void ring_fault() { device.ring_state = QLV_RING_FAULT; p32(device.control + QLV_RING_STATE_OFFSET, QLV_RING_FAULT); }

void consume() {
    device.work_pending = false;
    while (device.registered && device.ring_state == QLV_RING_ACTIVE) {
        const uint32_t tail = g32(device.control + QLV_RING_TAIL_OFFSET);
        if (tail == device.head) { break; }
        if (static_cast<uint32_t>(tail - device.head) > QLV_RING_ENTRIES) { ring_fault(); return; }
        const uint32_t seq = device.head + 1, slot = (seq - 1) % QLV_RING_ENTRIES;
        const uint64_t d = device.control + QLV_RING_DESCRIPTORS_OFFSET + slot * QLV_RING_DESCRIPTOR_BYTES;
        const uint32_t flags = g32(d + 4), bytes = g32(d + 8), capacity = g32(d + 12);
        const bool posted = (flags & QLV_RING_POSTED) != 0;
        if (g32(d) != seq || (flags & ~uint32_t(QLV_RING_POSTED)) != 0 || bytes < QLV_HEADER_BYTES ||
            bytes > QLV_RING_SLOT_BYTES || (posted ? capacity != 0 : capacity < QLV_HEADER_BYTES || capacity > device.ring_capacity)) {
            ring_fault();
            return;
        }
        static uint8_t request[QLV_RING_SLOT_BYTES], reply[QLV_HEADER_BYTES + QLV_CAPS_BYTES];
        memcpy(request, guest(device.data + uint64_t(slot) * QLV_RING_SLOT_BYTES, bytes), bytes);
        device.head = seq;
        p32(device.control + QLV_RING_HEAD_OFFSET, seq);
        ++device.ring_requests;
        if (device.error_next) {
            // lightning-venus.c retires invalid request bytes with ERROR.
            device.error_next = false;
            p32(device.control + QLV_RING_RESULT_STATUS_OFFSET, QLV_MMIO_ERROR);
            p32(device.control + QLV_RING_RESULT_BYTES_OFFSET, 0);
            device.retired = seq;
            p32(device.control + QLV_RING_RETIRED_OFFSET, seq);
            continue;
        }
        const uint32_t replied = serve(request, bytes, reply);
        const bool failed = h32(reply, QLV_STATUS_OFFSET) != QLV_STATUS_SUCCESS;
        if (!posted) { memcpy(guest(device.ring_response, replied), reply, replied); }
        if (posted && failed) {
            p64(device.control + QLV_RING_POSTED_ERROR_CONTEXT_OFFSET, h64(request, QLV_CONTEXT_OFFSET));
            p32(device.control + QLV_RING_POSTED_ERRORS_OFFSET, ++device.posted_errors);
        }
        p32(device.control + QLV_RING_RESULT_STATUS_OFFSET, QLV_MMIO_COMPLETE);
        p32(device.control + QLV_RING_RESULT_BYTES_OFFSET, posted ? 0 : replied);
        device.retired = seq;
        p32(device.control + QLV_RING_RETIRED_OFFSET, seq);
    }
    if (device.registered) { device.idle = true; p32(device.control + QLV_RING_IDLE_OFFSET, 1); }
}

uint32_t device_read(uint32_t offset) {
    switch (offset) {
    case QLV_MMIO_REG_MAGIC: return QLV_MAGIC;
    case QLV_MMIO_REG_CAPABILITIES:
        return static_cast<uint32_t>(QLV_MMIO_COMPLETION_IRQ | QLV_MMIO_SYNC_EVENTS) |
            (device.ring_capable ? static_cast<uint32_t>(QLV_MMIO_SHARED_RING) : 0) |
            (device.ring_capable && device.posted_writes ? static_cast<uint32_t>(QLV_MMIO_POSTED_SYNC_WRITES) : 0);
    case QLV_MMIO_REG_STATUS: return device.status;
    case QLV_MMIO_REG_RESPONSE_BYTES: return device.response_bytes;
    case QLV_MMIO_REG_RING_STATUS: return device.ring_capable ? device.ring_state : 0;
    default: return 0;
    }
}

void device_register() {
    device.registered = false;
    const uint64_t control = device.staged_control, data = device.staged_data;
    if (control % 4096 || data % 4096 || guest(control, QLV_RING_CONTROL_BYTES) == nullptr ||
        guest(data, QLV_RING_ENTRIES * QLV_RING_SLOT_BYTES) == nullptr || guest(device.response, device.response_capacity) == nullptr ||
        g32(control) != QLV_RING_MAGIC || g32(control + QLV_RING_VERSION_OFFSET) != QLV_RING_VERSION ||
        g32(control + QLV_RING_TAIL_OFFSET) != 0) {
        device.ring_state = QLV_RING_FAULT;
        return;
    }
    ++device.registrations;
    device.control = control;
    device.data = data;
    device.ring_response = device.response;
    device.ring_capacity = device.response_capacity;
    device.head = device.retired = device.posted_errors = 0;
    memset(device.sequence, 0, sizeof(device.sequence));
    memset(guest(control + QLV_RING_TIMELINE_OFFSET, 4096), 0, 4096);
    device.registered = true;
    device.idle = true;
    p32(control + QLV_RING_IDLE_OFFSET, 1);
    device.ring_state = QLV_RING_ACTIVE;
    p32(control + QLV_RING_STATE_OFFSET, QLV_RING_ACTIVE);
}

void device_write(uint32_t offset, uint32_t value) {
    switch (offset) {
    case QLV_MMIO_REG_DOORBELL:
        ++device.doorbells;
        if (device.deferred) {
            device.work_pending = true;
            device.idle = false;
            if (device.registered) { p32(device.control + QLV_RING_IDLE_OFFSET, 0); }
        } else {
            consume();
        }
        return;
    case QLV_MMIO_REG_REQUEST_LO: device.request = value; return;
    case QLV_MMIO_REG_REQUEST_BYTES: device.request_bytes = value; return;
    case QLV_MMIO_REG_RESPONSE_LO: device.response = value; return;
    case QLV_MMIO_REG_RESPONSE_CAPACITY: device.response_capacity = value; return;
    case QLV_MMIO_REG_RING_LO: device.staged_control = value; return;
    case QLV_MMIO_REG_RING_DATA_LO: device.staged_data = value; return;
    case QLV_MMIO_REG_COMMAND:
        if (value == QLV_MMIO_RESET) {
            device.registered = false;
            device.ring_state = QLV_RING_OFF;
            device.status = QLV_MMIO_ERROR;
            device.response_bytes = 0;
            device.reset_sleeps = 1;
            return;
        }
        if (value == QLV_MMIO_REGISTER) { ++device.register_commands; }
        if (value == QLV_MMIO_REGISTER && device.ring_capable) { device_register(); return; }
        if (value == QLV_MMIO_SUBMIT) {
            static uint8_t reply[QLV_HEADER_BYTES + QLV_CAPS_BYTES];
            ++device.mailbox_submits;
            const uint32_t replied = serve(guest(device.request, device.request_bytes), device.request_bytes, reply);
            memcpy(guest(device.response, replied), reply, replied);
            device.response_bytes = replied;
            device.status = QLV_MMIO_COMPLETE;
        }
        return;
    default:
        return;
    }
}

uint8_t bar[QLV_CONTROL_BAR_BYTES]{};
ULONGLONG clock_value = 0;
unsigned stalls = 0, sleeps = 0, dbg_lines = 0;
char last_dbg[512]{};
uint8_t config[256]{};

unsigned glitched_vendor_reads = 0;
bool hal_empty_slot = false;
ULONG NTAPI get_bus_data(PVOID, ULONG, PVOID buffer, ULONG offset, ULONG length) {
    if (offset > sizeof(config) || length > sizeof(config) - offset) { return 0; }
    if (offset == 0 && glitched_vendor_reads != 0) {
        --glitched_vendor_reads;
        memset(buffer, 0xff, length);
        return length;
    }
    // ReactOS HalGetBusData answers only the 0xFFFF vendor word for an empty slot.
    if (hal_empty_slot) {
        if (offset >= 2) { return 0; }
        memset(buffer, 0xff, 2 - offset < length ? 2 - offset : length);
        return 2 - offset < length ? 2 - offset : length;
    }
    memcpy(buffer, config + offset, length);
    return length;
}
ULONG NTAPI set_bus_data(PVOID, ULONG, PVOID buffer, ULONG offset, ULONG length) {
    if (offset > sizeof(config) || length > sizeof(config) - offset) { return 0; }
    memcpy(config + offset, buffer, length);
    return length;
}
VOID NTAPI dereference(PVOID) {}

PVOID NTAPI allocate_common(PDMA_ADAPTER, ULONG bytes, PPHYSICAL_ADDRESS address, BOOLEAN) {
    for (auto& allocation : allocations) {
        if (allocation.live) { continue; }
        allocation.memory = static_cast<uint8_t*>(calloc(1, bytes));
        allocation.address = next_address;
        allocation.bytes = bytes;
        allocation.live = true;
        next_address += (bytes + 0xfff) & ~0xfffu;
        address->QuadPart = allocation.address;
        return allocation.memory;
    }
    return nullptr;
}
VOID NTAPI free_common(PDMA_ADAPTER, ULONG, PHYSICAL_ADDRESS, PVOID memory, BOOLEAN) {
    for (auto& allocation : allocations) {
        if (allocation.live && allocation.memory == memory) { free(memory); allocation = Allocation{}; ++common_frees; }
    }
}
VOID NTAPI put_adapter(PDMA_ADAPTER) {}
DMA_OPERATIONS operations{};
DMA_ADAPTER adapter{};
alignas(16) uint8_t irp_storage[sizeof(IRP) + 4 * sizeof(IO_STACK_LOCATION)]{};

struct Resources final {
    CM_RESOURCE_LIST list{};
} raw_resources, translated_resources;

void reset_world() {
    for (auto& allocation : allocations) { if (allocation.live) { free(allocation.memory); } allocation = Allocation{}; }
    common_frees = 0;
    device = Device{};
    host = Host{};
    memset(known, 0, sizeof(known));
    memset(config, 0, sizeof(config));
    glitched_vendor_reads = 0;
    hal_empty_slot = false;
    const uint16_t vendor = 0x1234, id = 0x11f1, command = 2;
    const uint32_t bar0 = 0xfe000000u;
    memcpy(config, &vendor, 2);
    memcpy(config + 2, &id, 2);
    memcpy(config + 4, &command, 2);
    memcpy(config + 0x10, &bar0, 4);
    Resources* const lists[] = {&raw_resources, &translated_resources};
    for (auto* resources : lists) {
        resources->list = {};
        resources->list.Count = 1;
        resources->list.List[0].InterfaceType = PCIBus;
        resources->list.List[0].PartialResourceList.Count = 1;
        auto& descriptor = resources->list.List[0].PartialResourceList.PartialDescriptors[0];
        descriptor.Type = CmResourceTypeMemory;
        descriptor.u.Memory.Start.QuadPart = bar0;
        descriptor.u.Memory.Length = QLV_CONTROL_BAR_BYTES;
    }
    operations.Size = sizeof(operations);
    operations.AllocateCommonBuffer = allocate_common;
    operations.FreeCommonBuffer = free_common;
    operations.PutDmaAdapter = put_adapter;
    adapter.DmaOperations = &operations;
}

// ---- Packets for renderer_exchange.
struct Packet final { uint8_t bytes[QLV_HEADER_BYTES + 8192]{}; size_t size = QLV_HEADER_BYTES; };
Packet packet(uint16_t operation, uint64_t context) {
    Packet p;
    s32(p.bytes, QLV_MAGIC_OFFSET, static_cast<uint32_t>(QLV_MAGIC));
    const uint32_t framing = static_cast<uint32_t>(QLV_VERSION) | (uint32_t{operation} << 16);
    s32(p.bytes, QLV_VERSION_OFFSET, framing);
    s64(p.bytes, QLV_CONTEXT_OFFSET, context);
    s32(p.bytes, QLV_STATUS_OFFSET, static_cast<uint32_t>(QLV_STATUS_UNSUPPORTED));
    return p;
}
void payload(Packet& p, const void* data, uint32_t bytes) {
    memcpy(p.bytes + QLV_HEADER_BYTES, data, bytes);
    s32(p.bytes, QLV_PAYLOAD_BYTES_OFFSET, bytes);
    p.size = QLV_HEADER_BYTES + bytes;
}
bool lookup(void*, uint64_t token, ResourceInfo& info) noexcept {
    for (auto value : known) {
        if (value == token && token != 0) { info = {}; info.kind = ResourceKind::sync; return true; }
    }
    return false;
}
void remember(uint64_t token) { for (auto& value : known) { if (value == 0) { value = token; return; } } }

NativeTransport* transport_storage = nullptr;
NTSTATUS exchange(NativeTransport& transport, const Packet& p, uint64_t context, ExchangeReply& reply) {
    return transport.renderer_exchange({p.bytes, p.size}, response_capacity,
        {context, QLV_APERTURE_BYTES, nullptr, lookup}, reply);
}
struct Costs final { unsigned reads, writes, mailbox, ring, doorbells; };
Costs costs() { return {device.reads, device.writes, device.mailbox_submits, device.ring_requests, device.doorbells}; }

uint64_t open_context(NativeTransport& transport) {
    ExchangeReply reply;
    exchange(transport, packet(QLV_OPEN, 0), 0, reply);
    return reply.header.context;
}
uint64_t create_sync(NativeTransport& transport, uint64_t context, uint64_t value, ExchangeReply& reply) {
    Packet p = packet(QLV_CREATE_SYNC, context);
    s64(p.bytes, QLV_VALUE_OFFSET, value);
    const NTSTATUS status = exchange(transport, p, context, reply);
    check(NT_SUCCESS(status), "CREATE_SYNC exchange succeeds");
    if (!NT_SUCCESS(status)) { printf("CREATE_SYNC status %08lx dbg '%s'\n", static_cast<unsigned long>(status), last_dbg); }
    remember(reply.header.resource);
    return reply.header.resource;
}
NTSTATUS read_sync(NativeTransport& transport, uint64_t context, uint64_t token, ExchangeReply& reply) {
    Packet p = packet(QLV_READ_SYNC, context);
    s64(p.bytes, QLV_RESOURCE_OFFSET, token);
    return exchange(transport, p, context, reply);
}
NTSTATUS wait_sync(NativeTransport& transport, uint64_t context, uint64_t token, uint64_t value, ExchangeReply& reply) {
    Packet p = packet(QLV_WAIT_SYNCS, context);
    s64(p.bytes, QLV_BYTES_OFFSET, 1);
    const uint64_t point[2] = {token, value};
    payload(p, point, sizeof(point));
    return exchange(transport, p, context, reply);
}
NTSTATUS submit(NativeTransport& transport, uint64_t context, uint32_t command_bytes, ExchangeReply& reply) {
    Packet p = packet(QLV_SUBMIT, context);
    static uint8_t commands[2048]{};
    s64(p.bytes, QLV_BYTES_OFFSET, command_bytes);
    payload(p, commands, command_bytes);
    return exchange(transport, p, context, reply);
}
HostSync scratch_sync{};
// A missing host sync is a test failure, never a null dereference.
HostSync* host_sync(uint64_t token) {
    HostSync* sync = find_sync(token);
    if (sync == nullptr) { check(false, "the host knows the created sync"); scratch_sync = {}; return &scratch_sync; }
    return sync;
}

alignas(16) uint8_t transport_memory[sizeof(NativeTransport)];
NativeTransport& fresh_transport() {
    reset_world();
    memset(transport_memory, 0, sizeof(transport_memory));
    transport_storage = rx7900::reactos::construct_device_state<NativeTransport>(transport_memory);
    return *transport_storage;
}
DEVICE_OBJECT pdo{};
NTSTATUS start(NativeTransport& transport) {
    return transport.start(&pdo, &raw_resources.list, &translated_resources.list);
}

void old_qemu_keeps_the_mailbox() {
    NativeTransport& transport = fresh_transport();
    device.ring_capable = false;
    check(NT_SUCCESS(start(transport)) && transport.ready(), "start succeeds against a device without the ring");
    check(device.register_commands == 0, "no REGISTER is written without the capability bit");
    const uint64_t context = open_context(transport);
    ExchangeReply reply;
    const Costs before = costs();
    create_sync(transport, context, 3, reply);
    check(host.last_create_flags == 0, "an old peer never sees SYNC_PUBLISH");
    const Costs after = costs();
    check(after.mailbox == before.mailbox + 1 && after.ring == before.ring && after.writes - before.writes == 9 &&
        after.reads - before.reads >= 4, "the mailbox exchange keeps its 9 register writes and polled reads");
    printf("mailbox exchange: %u MMIO writes, %u MMIO reads\n", after.writes - before.writes, after.reads - before.reads);
    check(NT_SUCCESS(transport.stop()), "stop releases the mailbox transport");
}

void ring_exchanges_and_local_answers() {
    NativeTransport& transport = fresh_transport();
    check(NT_SUCCESS(start(transport)) && device.registrations == 1 && device.ring_state == QLV_RING_ACTIVE,
        "start registers the ring after OPEN/GET_CAPS");
    check(stalls == 1, "the spin budget is calibrated once, outside every completion wait");
    const uint64_t context = open_context(transport);
    ExchangeReply reply;
    Costs before = costs();
    const uint64_t token = create_sync(transport, context, 3, reply);
    Costs after = costs();
    check(after.writes - before.writes == 1 && after.reads == before.reads && after.ring == before.ring + 1,
        "a ring exchange costs one MMIO write (the doorbell) and no MMIO reads");
    printf("ring exchange: %u MMIO writes, %u MMIO reads\n", after.writes - before.writes, after.reads - before.reads);
    check(host.last_create_flags == QLV_SYNC_PUBLISH, "CREATE_SYNC asks a publishing host for a slot");
    check(reply.header.status == QLV_STATUS_SUCCESS && reply.header.offset == 0 && reply.header.flags == 0,
        "the caller sees the reply its request implies (slot and flag stay private)");

    before = costs();
    read_sync(transport, context, token, reply);
    after = costs();
    check(reply.header.status == QLV_STATUS_SUCCESS && reply.header.value == 3 && after.reads == before.reads &&
        after.writes == before.writes && host.operations[QLV_READ_SYNC] == 0,
        "READ_SYNC is answered from the timeline page: no MMIO, no host request");
    host_sync(token)->value = 9;
    publish_value(*host_sync(token));
    read_sync(transport, context, token, reply);
    check(reply.header.value == 9 && host.operations[QLV_READ_SYNC] == 0, "a newly published value is read locally");
    for (uint32_t index = 0; index < ring::refresh_reads; ++index) { read_sync(transport, context, token, reply); }
    check(host.operations[QLV_READ_SYNC] == 1 && reply.header.value == 9,
        "unchanged answers periodically refresh through the host");

    wait_sync(transport, context, token, 9, reply);
    check(reply.header.status == QLV_STATUS_SUCCESS && host.operations[QLV_WAIT_SYNCS] == 0,
        "a satisfied zero-timeout WAIT_SYNCS is answered locally");
    wait_sync(transport, context, token, 10, reply);
    check(reply.header.status == QLV_STATUS_TIMEOUT && host.operations[QLV_WAIT_SYNCS] == 0,
        "an unsatisfied check is a local TIMEOUT");
    for (uint32_t index = 0; index < ring::refresh_reads; ++index) { wait_sync(transport, context, token, 10, reply); }
    check(host.operations[QLV_WAIT_SYNCS] == 1, "repeated unsatisfied checks periodically ask the host");

    const uint64_t other = open_context(transport);
    remember(token);
    read_sync(transport, other, token, reply);
    check(host.operations[QLV_READ_SYNC] == 2, "another context's read of this sync is never answered locally");

    Packet write = packet(QLV_WRITE_SYNC, context);
    s64(write.bytes, QLV_RESOURCE_OFFSET, token);
    s64(write.bytes, QLV_VALUE_OFFSET, 0);
    exchange(transport, write, context, reply);
    read_sync(transport, context, token, reply);
    check(reply.header.value == 0, "a WRITE_SYNC reset is visible to the next local read (published before its reply)");

    check(transport.await_published(context, token, 0) == false, "await returns without progress after its budget");
    host_sync(token)->value = 12;
    publish_value(*host_sync(token));
    check(transport.await_published(context, token, 0), "await sees published progress in guest RAM");

    Packet destroy = packet(QLV_DESTROY_SYNC, context);
    s64(destroy.bytes, QLV_RESOURCE_OFFSET, token);
    exchange(transport, destroy, context, reply);
    const unsigned host_reads = host.operations[QLV_READ_SYNC];
    read_sync(transport, context, token, reply);
    check(host.operations[QLV_READ_SYNC] == host_reads + 1, "a destroyed sync is no longer answered locally");

    before = costs();
    submit(transport, context, 1600, reply);
    check(reply.header.status == QLV_STATUS_SUCCESS && costs().ring == before.ring + 1, "a slot-sized SUBMIT uses the ring");
    check(NT_SUCCESS(transport.stop()) && common_frees == 4, "stop frees request, response and both ring buffers");
}

void posted_submits_and_failures() {
    NativeTransport& transport = fresh_transport();
    start(transport);
    transport.post_submits(true);
    const uint64_t context = open_context(transport);
    const uint64_t neighbor = open_context(transport);
    ExchangeReply reply;
    device.deferred = true;
    const unsigned submits = host.operations[QLV_SUBMIT];
    const Costs before = costs();
    for (int index = 0; index < 3; ++index) {
        check(NT_SUCCESS(submit(transport, context, 64, reply)) && reply.header.status == QLV_STATUS_SUCCESS,
            "a posted SUBMIT returns success once queued");
    }
    const Costs after = costs();
    check(host.operations[QLV_SUBMIT] == submits && after.doorbells == before.doorbells + 1 && after.reads == before.reads,
        "three posted SUBMITs behind a busy consumer: one doorbell, no reads, nothing processed yet");
    const uint64_t token = create_sync(transport, context, 1, reply);
    check(host.operations[QLV_SUBMIT] == submits + 3 && host.order[host.order_count - 1] == QLV_CREATE_SYNC,
        "a synchronous request waits behind the posted ones in order");
    device.deferred = false;

    host.fail_submits = true;
    submit(transport, context, 64, reply);
    host.fail_submits = false;
    read_sync(transport, context, token, reply);
    check(reply.header.status == QLV_STATUS_DEVICE_LOST, "a failed posted SUBMIT reports device loss to its context");
    read_sync(transport, neighbor, token, reply);
    check(reply.header.status != QLV_STATUS_DEVICE_LOST, "other contexts are unaffected");
    Packet close = packet(QLV_CLOSE, context);
    exchange(transport, close, context, reply);
    check(reply.header.status == QLV_STATUS_SUCCESS, "the poisoned context can still CLOSE");

    device.deferred = true;
    for (uint32_t index = 0; index < QLV_RING_ENTRIES + 2; ++index) { submit(transport, neighbor, 64, reply); }
    check(sleeps > 0 && reply.header.status == QLV_STATUS_SUCCESS, "a full ring waits (sleeping) for the consumer, then queues");
    device.deferred = false;
    check(NT_SUCCESS(transport.stop()), "stop drains and resets a posting transport");
}

void oversized_requests_and_recovery() {
    NativeTransport& transport = fresh_transport();
    start(transport);
    transport.post_submits(true);
    const uint64_t context = open_context(transport);
    ExchangeReply reply;
    const uint64_t token = create_sync(transport, context, 5, reply);
    device.deferred = true;
    submit(transport, context, 64, reply);
    const unsigned mailbox = device.mailbox_submits;
    const unsigned order = host.order_count;
    // 300 points (4864 bytes) exceed one ring slot: the mailbox carries it,
    // but only after every queued ring entry has retired. The slot is cleared
    // first so the wait cannot be answered locally.
    Packet big = packet(QLV_WAIT_SYNCS, context);
    static uint64_t points[2 * 300]{};
    for (size_t index = 0; index < 300; ++index) { points[2 * index] = token; points[2 * index + 1] = 5; }
    s64(big.bytes, QLV_BYTES_OFFSET, 300);
    payload(big, points, sizeof(points));
    const unsigned host_waits = host.operations[QLV_WAIT_SYNCS];
    host_sync(token)->slot = -1;
    memset(guest(device.control + QLV_RING_TIMELINE_OFFSET, 4096), 0, 4096);
    const NTSTATUS status = exchange(transport, big, context, reply);
    check(NT_SUCCESS(status) && reply.header.status == QLV_STATUS_SUCCESS && device.mailbox_submits == mailbox + 1 &&
        host.operations[QLV_WAIT_SYNCS] == host_waits + 1, "an oversized request uses the mailbox");
    check(host.order_count == order + 2 && host.order[order] == QLV_SUBMIT && host.order[order + 1] == QLV_WAIT_SYNCS,
        "the mailbox request follows the drained posted entry");
    device.deferred = false;

    device.error_next = true;
    const unsigned ring_before = device.ring_requests;
    check(!NT_SUCCESS(read_sync(transport, context, token, reply)) && device.ring_requests == ring_before + 1 &&
        !device.error_next, "a ring entry retired with ERROR fails its exchange");
    const NTSTATUS recovered = transport.recover_renderer();
    check(NT_SUCCESS(recovered), "recovery after an ERROR retirement");
    if (!NT_SUCCESS(recovered)) { printf("recover %08lx dbg '%s' status %u\n", static_cast<unsigned long>(recovered), last_dbg, device.status); }
    const uint64_t context2 = open_context(transport);
    device.ring_state = QLV_RING_LOST;
    p32(device.control + QLV_RING_STATE_OFFSET, QLV_RING_LOST);
    const uint32_t tail_before = g32(device.control + QLV_RING_TAIL_OFFSET);
    const NTSTATUS lost = read_sync(transport, context2, token, reply);
    check(!NT_SUCCESS(lost) && g32(device.control + QLV_RING_TAIL_OFFSET) == tail_before,
        "a lost ring fails the exchange before queuing anything into it");
    check(NT_SUCCESS(transport.recover_renderer()) && device.registrations == 3 && device.ring_state == QLV_RING_ACTIVE,
        "recovery RESETs, reconnects and registers a fresh ring");
    const uint64_t again = open_context(transport);
    const uint64_t fresh_token = create_sync(transport, again, 4, reply);
    read_sync(transport, again, fresh_token, reply);
    check(reply.header.value == 4 && host.operations[QLV_READ_SYNC] == 0, "the fresh ring serves local answers again");
    check(NT_SUCCESS(transport.stop()), "stop after recovery");
}

NTSTATUS write_sync(NativeTransport& transport, uint64_t context, uint64_t token, uint64_t value, ExchangeReply& reply) {
    Packet p = packet(QLV_WRITE_SYNC, context);
    s64(p.bytes, QLV_RESOURCE_OFFSET, token);
    s64(p.bytes, QLV_VALUE_OFFSET, value);
    return exchange(transport, p, context, reply);
}

// Ring_batching: posted entries of a batch share one IDLE probe and doorbell;
// every wait announces first; a posted WRITE_SYNC hides its sync's stale page
// value until it retires. The consumer here runs only when rung.
void batched_doorbells_and_posted_writes() {
    NativeTransport& transport = fresh_transport();
    device.posted_writes = true;
    start(transport);
    transport.post_submits(true);
    const uint64_t context = open_context(transport);
    ExchangeReply reply;
    const uint64_t token = create_sync(transport, context, 1, reply);
    const unsigned submits = host.operations[QLV_SUBMIT];
    Costs before = costs();
    transport.begin_batch();
    for (int index = 0; index < 3; ++index) {
        check(NT_SUCCESS(submit(transport, context, 64, reply)) && reply.header.status == QLV_STATUS_SUCCESS,
            "a batched posted SUBMIT returns success once queued");
    }
    check(costs().doorbells == before.doorbells && host.operations[QLV_SUBMIT] == submits && costs().reads == before.reads,
        "inside a batch nothing is rung or read: QEMU has not seen the entries yet");
    transport.end_batch();
    Costs after = costs();
    check(after.doorbells == before.doorbells + 1 && host.operations[QLV_SUBMIT] == submits + 3 &&
        after.writes == before.writes + 1 && after.reads == before.reads,
        "end_batch rings once for the whole batch; all entries run in order");
    transport.end_batch();
    check(costs().doorbells == after.doorbells, "a second end_batch owes nothing");

    // A posted reset hides the page's stale 1 until QEMU retires it; the
    // synchronous fallback probes (one doorbell) and runs behind the write.
    before = costs();
    transport.begin_batch();
    const unsigned writes = host.operations[QLV_WRITE_SYNC];
    const unsigned reads = host.operations[QLV_READ_SYNC];
    check(NT_SUCCESS(write_sync(transport, context, token, 0, reply)) && reply.header.status == QLV_STATUS_SUCCESS &&
        host.operations[QLV_WRITE_SYNC] == writes && costs().doorbells == before.doorbells,
        "a WRITE_SYNC is posted when QEMU accepts it, inside the batch unrung");
    check(NT_SUCCESS(read_sync(transport, context, token, reply)) && reply.header.value == 0 &&
        host.operations[QLV_READ_SYNC] == reads + 1 && host.order[host.order_count - 2] == QLV_WRITE_SYNC &&
        costs().doorbells == before.doorbells + 1, "a read before the write retires asks the host, behind the write");
    transport.end_batch();
    check(costs().doorbells == before.doorbells + 1, "the synchronous entry's probe already covered the batch");
    read_sync(transport, context, token, reply);
    check(reply.header.value == 0 && host.operations[QLV_READ_SYNC] == reads + 1,
        "after retirement the published value answers locally again");

    // Outside a batch a posted write still probes and rings by itself.
    before = costs();
    write_sync(transport, context, token, 1, reply);
    check(costs().doorbells == before.doorbells + 1 && host.operations[QLV_WRITE_SYNC] == writes + 2,
        "an unbatched posted WRITE_SYNC rings when the consumer is idle");
    wait_sync(transport, context, token, 1, reply);
    check(reply.header.status == QLV_STATUS_SUCCESS && host.operations[QLV_WAIT_SYNCS] == 0,
        "a retired write lets WAIT_SYNCS answer locally");

    // A guest-RAM spin inside a batch announces first, too.
    before = costs();
    transport.begin_batch();
    submit(transport, context, 64, reply);
    check(costs().doorbells == before.doorbells, "the batched entry is not yet announced");
    transport.await_published(context, token, 5);
    check(costs().doorbells == before.doorbells + 1 && host.operations[QLV_SUBMIT] == submits + 4,
        "await_published announces owed entries before it spins");
    transport.end_batch();

    // A full ring inside a batch announces before it waits for space.
    const unsigned sleeps_before = sleeps;
    transport.begin_batch();
    for (uint32_t index = 0; index < QLV_RING_ENTRIES + 2; ++index) { submit(transport, context, 64, reply); }
    check(reply.header.status == QLV_STATUS_SUCCESS && sleeps == sleeps_before,
        "a full ring inside a batch rings before waiting (no deadline wait)");
    transport.end_batch();
    check(device.retired == g32(device.control + QLV_RING_TAIL_OFFSET), "every batched entry retired");

    // A failed posted write poisons its context like a failed posted SUBMIT.
    host.fail_writes = true;
    write_sync(transport, context, token, 0, reply);
    host.fail_writes = false;
    check(reply.header.status == QLV_STATUS_SUCCESS, "the posted write itself reports queued success");
    read_sync(transport, context, token, reply);
    check(reply.header.status == QLV_STATUS_DEVICE_LOST, "its failure reaches the next exchange as device loss");

    // A ring that faulted inside a batch is never rung; RESET recovers it.
    const uint64_t faulting = open_context(transport);
    const unsigned queued = device.ring_requests;
    transport.begin_batch();
    submit(transport, faulting, 64, reply);
    check(device.ring_requests == queued && g32(device.control + QLV_RING_TAIL_OFFSET) != device.head,
        "the batched entry is queued, unannounced");
    ring_fault();
    before = costs();
    transport.end_batch();
    check(costs().doorbells == before.doorbells, "end_batch does not ring a faulted ring");
    check(NT_SUCCESS(transport.recover_renderer()), "recovery after a fault inside a batch");

    // A reset inside a batch leaves nothing owed to the dead ring.
    const uint64_t other = open_context(transport);
    transport.begin_batch();
    submit(transport, other, 64, reply);
    check(NT_SUCCESS(transport.recover_renderer()), "recovery inside a batch");
    before = costs();
    transport.end_batch();
    check(costs().doorbells == before.doorbells, "end_batch after a reset rings nothing");
    check(NT_SUCCESS(transport.stop()), "stop after batching");
}

void old_qemu_keeps_synchronous_writes() {
    NativeTransport& transport = fresh_transport();
    start(transport);
    transport.post_submits(true);
    const uint64_t context = open_context(transport);
    ExchangeReply reply;
    const uint64_t token = create_sync(transport, context, 1, reply);
    host.fail_writes = true;
    write_sync(transport, context, token, 0, reply);
    host.fail_writes = false;
    check(reply.header.status == QLV_STATUS_INVALID,
        "without MMIO_POSTED_SYNC_WRITES a WRITE_SYNC stays synchronous and reports its own error");
    transport.begin_batch();
    const Costs before = costs();
    write_sync(transport, context, token, 0, reply);
    check(reply.header.status == QLV_STATUS_SUCCESS && costs().doorbells == before.doorbells + 1 &&
        host_sync(token)->value == 0, "inside a batch a synchronous write still completes before it returns");
    transport.end_batch();
    check(NT_SUCCESS(transport.stop()), "stop with an old peer");
}

void no_publishing_host() {
    NativeTransport& transport = fresh_transport();
    host.publish = false;
    start(transport);
    const uint64_t context = open_context(transport);
    ExchangeReply reply;
    const uint64_t token = create_sync(transport, context, 2, reply);
    check(host.last_create_flags == 0, "SYNC_PUBLISH is only sent to a host that advertised it");
    read_sync(transport, context, token, reply);
    check(reply.header.value == 2 && host.operations[QLV_READ_SYNC] == 1, "unpublished syncs keep exchanging, through the ring");
    transport.stop();
}

} // namespace

// ---- NT fixtures (exact SDK declarations; this image is not a driver).
extern "C" {
ULONG NTAPI READ_REGISTER_ULONG(PULONG reg) { ++device.reads; return device_read(static_cast<uint32_t>(reinterpret_cast<uint8_t*>(reg) - bar)); }
VOID NTAPI WRITE_REGISTER_ULONG(PULONG reg, ULONG value) { ++device.writes; device_write(static_cast<uint32_t>(reinterpret_cast<uint8_t*>(reg) - bar), value); }
PVOID NTAPI MmMapIoSpace(PHYSICAL_ADDRESS, SIZE_T, MEMORY_CACHING_TYPE) { return bar; }
VOID NTAPI MmUnmapIoSpace(PVOID, SIZE_T) {}
PDMA_ADAPTER NTAPI IoGetDmaAdapter(PDEVICE_OBJECT, PDEVICE_DESCRIPTION, PULONG registers) { *registers = 16; return &adapter; }
KIRQL NTAPI KeGetCurrentIrql(VOID) { return PASSIVE_LEVEL; }
ULONGLONG NTAPI KeQueryInterruptTime(VOID) { return clock_value; }
VOID NTAPI KeStallExecutionProcessor(ULONG) { ++stalls; }
NTSTATUS NTAPI KeDelayExecutionThread(KPROCESSOR_MODE, BOOLEAN, PLARGE_INTEGER interval) {
    ++sleeps;
    clock_value += static_cast<ULONGLONG>(-interval->QuadPart);
    if (device.reset_sleeps != 0 && --device.reset_sleeps == 0) { device.status = QLV_MMIO_IDLE; }
    if (device.work_pending || (device.deferred && device.registered)) { consume(); }
    return STATUS_SUCCESS;
}
VOID NTAPI KeInitializeEvent(PRKEVENT, EVENT_TYPE, BOOLEAN) {}
NTSTATUS NTAPI KeWaitForSingleObject(PVOID, KWAIT_REASON, KPROCESSOR_MODE, BOOLEAN, PLARGE_INTEGER) { return STATUS_SUCCESS; }
PIRP NTAPI IoBuildSynchronousFsdRequest(ULONG, PDEVICE_OBJECT, PVOID, ULONG, PLARGE_INTEGER, PKEVENT, PIO_STATUS_BLOCK status) {
    memset(irp_storage, 0, sizeof(irp_storage));
    auto* irp = reinterpret_cast<PIRP>(irp_storage);
    auto* stacks = reinterpret_cast<PIO_STACK_LOCATION>(irp_storage + sizeof(IRP));
    irp->StackCount = 2;
    irp->CurrentLocation = 2;
    irp->Tail.Overlay.CurrentStackLocation = stacks + 2;
    irp->UserIosb = status;
    return irp;
}
NTSTATUS FASTCALL IofCallDriver(PDEVICE_OBJECT, PIRP irp) {
    auto* stack = IoGetNextIrpStackLocation(irp);
    auto* bus = reinterpret_cast<PBUS_INTERFACE_STANDARD>(stack->Parameters.QueryInterface.Interface);
    bus->Size = sizeof(*bus);
    bus->Version = 1;
    bus->Context = nullptr;
    bus->GetBusData = get_bus_data;
    bus->SetBusData = set_bus_data;
    bus->InterfaceReference = dereference;
    bus->InterfaceDereference = dereference;
    irp->IoStatus.Status = STATUS_SUCCESS;
    if (irp->UserIosb != nullptr) { irp->UserIosb->Status = STATUS_SUCCESS; }
    return STATUS_SUCCESS;
}
VOID NTAPI RtlAssert(PVOID assertion, PVOID file, ULONG line, PSTR) {
    printf("ASSERT %s %s:%lu\n", static_cast<const char*>(assertion), static_cast<const char*>(file), line);
    ++failures;
}
ULONG __cdecl DbgPrint(PCSTR format, ...) {
    va_list arguments;
    va_start(arguments, format);
    vsnprintf(last_dbg, sizeof(last_dbg), format, arguments);
    va_end(arguments);
    ++dbg_lines;
    if (strncmp(last_dbg, "QLVRING", 7) == 0) { printf("%s", last_dbg); }
    return 0;
}
}

void surprise_removal_needs_an_absent_function() {
    ExchangeReply reply;
    NativeTransport& present = fresh_transport();
    start(present);
    present.post_submits(true);
    const uint64_t context = open_context(present);
    submit(present, context, 64, reply);
    check(!NT_SUCCESS(present.stop(false)) && present.quarantined(),
        "a surprise removal of a function still on the bus keeps the transport quarantined");
    NativeTransport& gone = fresh_transport();
    start(gone);
    gone.post_submits(true);
    submit(gone, open_context(gone), 64, reply);
    hal_empty_slot = true;
    check(NT_SUCCESS(gone.stop(false)) && !gone.quarantined(),
        "a hot-unplugged function (HAL answers only the 0xFFFF vendor word) proves stop without MMIO");
    NativeTransport& flaky = fresh_transport();
    start(flaky);
    flaky.post_submits(true);
    submit(flaky, open_context(flaky), 64, reply);
    glitched_vendor_reads = 1;
    check(!NT_SUCCESS(flaky.stop(false)), "one glitched all-ones vendor read is not absence");
}

int main() {
    old_qemu_keeps_the_mailbox();
    ring_exchanges_and_local_answers();
    posted_submits_and_failures();
    oversized_requests_and_recovery();
    no_publishing_host();
    batched_doorbells_and_posted_writes();
    old_qemu_keeps_synchronous_writes();
    surprise_removal_needs_an_absent_function();
    printf("Lightning transport ring: %u checks, %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

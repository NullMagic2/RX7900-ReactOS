/*
Module: lightning.reactos.transport
Purpose: Execute QLV session/capability exchanges with actual NT DMA and MMIO.
Created: 2026-10-21
Architecture: PCI PnP resources only; uncertain DMA is retained, never freed.
The shared ring (Lightning_doorbell) reuses the same buffers, validation and
recovery: a ring request is one slot copy, one optional doorbell write and a
RAM poll; RESET always unregisters it, and old peers keep the mailbox. A
renderer batch (Ring_batching) publishes its posted entries and probes IDLE
once, so one doorbell covers the batch.
*/
#include "lightning_transport.hpp"
#include "pci.hpp"
#include <initguid.h>
#include <wdmguid.h>

namespace lightning::reactos {
namespace {

constexpr ULONG dma_bytes = 4096;
constexpr ULONG pci_command_offset = 4;
constexpr USHORT pci_memory_enable = 2;
constexpr USHORT pci_master_enable = 4;
constexpr ULONG poll_milliseconds = 1;
constexpr ULONG maximum_poll_iterations = transport_wait::maximum_sleeps;

#if !defined(RX7900_LIGHTNING_TRANSPORT_TIMING)
// Compile-time disabled instrumentation leaves no QPC calls or aggregate store.
class UnmeasuredScope final {
public:
    NTSTATUS finish(NTSTATUS status) noexcept { return status; }
    void after_submit() noexcept {}
    void pending() noexcept {}
    void complete() noexcept {}
    void before_sleep() noexcept {}
    void stall() noexcept {}
};
#endif

bool dma_span(PHYSICAL_ADDRESS address, ULONG bytes = dma_bytes) noexcept {
    return bytes != 0 && address.HighPart == 0 && address.LowPart <= MAXULONG - (bytes - 1);
}

NTSTATUS host_error(uint32_t status) noexcept {
    switch (status) {
    case QLV_STATUS_UNSUPPORTED: return STATUS_NOT_SUPPORTED;
    case QLV_STATUS_INVALID: return STATUS_INVALID_PARAMETER;
    case QLV_STATUS_DEVICE_LOST: return STATUS_DEVICE_NOT_CONNECTED;
    case QLV_STATUS_NO_MEMORY: return STATUS_INSUFFICIENT_RESOURCES;
    case QLV_STATUS_TIMEOUT: return STATUS_IO_TIMEOUT;
    default: return STATUS_DATA_ERROR;
    }
}

} // namespace

// The NT5 target is x86: TSO orders load-load and store-store, so acquire and
// release need only stop compiler reordering around the volatile ring words;
// store-load needs a locked op. KeMemoryBarrier is not used: some pinned SDK
// revisions define it without a compiler memory clobber.
void NativeTransport::KernelBarriers::acquire() noexcept { _ReadWriteBarrier(); }
void NativeTransport::KernelBarriers::release() noexcept { _ReadWriteBarrier(); }
void NativeTransport::KernelBarriers::full() noexcept {
#if defined(__GNUC__)
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
#else
    volatile LONG fence = 0;
    InterlockedOr(&fence, 0);
    _ReadWriteBarrier();
#endif
}

NTSTATUS NativeTransport::acquire_bus(PDEVICE_OBJECT pdo) noexcept {
    KEVENT event;
    KeInitializeEvent(&event, NotificationEvent, FALSE);
    IO_STATUS_BLOCK result{};
    PIRP request = IoBuildSynchronousFsdRequest(IRP_MJ_PNP, pdo, nullptr, 0,
                                               nullptr, &event, &result);

    if (request == nullptr) {
        return STATUS_INSUFFICIENT_RESOURCES;
    }

    auto* stack = IoGetNextIrpStackLocation(request);
    request->IoStatus.Status = STATUS_NOT_SUPPORTED;
    stack->MinorFunction = IRP_MN_QUERY_INTERFACE;
    stack->Parameters.QueryInterface.InterfaceType = &GUID_BUS_INTERFACE_STANDARD;
    stack->Parameters.QueryInterface.Size = sizeof(bus_);
    stack->Parameters.QueryInterface.Version = 1;
    stack->Parameters.QueryInterface.Interface = reinterpret_cast<PINTERFACE>(&bus_);
    stack->Parameters.QueryInterface.InterfaceSpecificData = nullptr;
    NTSTATUS status = IoCallDriver(pdo, request);

    if (status == STATUS_PENDING) {
        KeWaitForSingleObject(&event, Executive, KernelMode, FALSE, nullptr);
        status = result.Status;
    }

    if (!NT_SUCCESS(status)) {
        return status;
    }

    bus_acquired_ = true;

    if (bus_.Size < sizeof(bus_) || bus_.Version != 1 || bus_.GetBusData == nullptr ||
        bus_.SetBusData == nullptr || bus_.InterfaceDereference == nullptr) {
        quarantined_ = bus_.InterfaceDereference == nullptr;
        return STATUS_NOT_SUPPORTED;
    }

    return STATUS_SUCCESS;
}

NTSTATUS NativeTransport::find_bar(PCM_RESOURCE_LIST raw, PCM_RESOURCE_LIST translated,
                                   PHYSICAL_ADDRESS& base) noexcept {
    uint8_t bytes[rx7900::pci_header_bytes]{};
    const ULONG received = bus_.GetBusData(bus_.Context, PCI_WHICHSPACE_CONFIG,
        bytes, 0, sizeof(bytes));
    rx7900::ByteView config(bytes, received <= sizeof(bytes) ? received : 0);
    uint16_t vendor = 0;
    uint16_t device = 0;
    uint16_t command = 0;
    uint32_t bar = 0;
    uint8_t header = 0;

    if (received != sizeof(bytes) || !config.read_u16(rx7900::pci_vendor_offset, vendor) ||
        !config.read_u16(rx7900::pci_device_offset, device) ||
        vendor != pci_vendor_id || device != pci_device_id ||
        !config.read_u8(rx7900::pci_header_type_offset, header) || (header & 0x7f) != 0 ||
        !config.read_u16(pci_command_offset, command) || (command & pci_memory_enable) == 0 ||
        !config.read_u32(rx7900::pci_bars_offset, bar) || (bar & 0xf) != 0 || bar == 0) {
        return STATUS_DEVICE_CONFIGURATION_ERROR;
    }

    identity_valid_ = true;

    // NT5 PCI supplies one full descriptor. Reject other layouts instead of
    // traversing an unbounded variable-size CM_FULL_RESOURCE_DESCRIPTOR array.
    if (raw == nullptr || translated == nullptr || raw->Count != 1 || translated->Count != 1 ||
        raw->List[0].InterfaceType != PCIBus || translated->List[0].InterfaceType != PCIBus ||
        raw->List[0].PartialResourceList.Count == 0 ||
        raw->List[0].PartialResourceList.Count > 64 ||
        raw->List[0].PartialResourceList.Count != translated->List[0].PartialResourceList.Count) {
        return STATUS_DEVICE_CONFIGURATION_ERROR;
    }

    const auto& raw_list = raw->List[0].PartialResourceList;
    const auto& translated_list = translated->List[0].PartialResourceList;
    bool found = false;

    for (ULONG index = 0; index < raw_list.Count; ++index) {
        const auto& source = raw_list.PartialDescriptors[index];
        const auto& target = translated_list.PartialDescriptors[index];

        if (source.Type != CmResourceTypeMemory || source.u.Memory.Start.QuadPart != (bar & ~0xfu)) {
            continue;
        }

        if (found || target.Type != CmResourceTypeMemory ||
            (target.Flags & CM_RESOURCE_MEMORY_READ_ONLY) != 0 ||
            source.u.Memory.Length != QLV_CONTROL_BAR_BYTES ||
            target.u.Memory.Length != QLV_CONTROL_BAR_BYTES ||
            target.u.Memory.Start.QuadPart < 0 ||
            static_cast<ULONGLONG>(target.u.Memory.Start.QuadPart) >
                MAXLONGLONG - (QLV_CONTROL_BAR_BYTES - 1)) {
            return STATUS_DEVICE_CONFIGURATION_ERROR;
        }

        base = target.u.Memory.Start;
        found = true;
    }

    return found ? STATUS_SUCCESS : STATUS_DEVICE_CONFIGURATION_ERROR;
}

bool NativeTransport::bus_master(bool enable) noexcept {
    if (!bus_acquired_ || !identity_valid_ || bus_.GetBusData == nullptr || bus_.SetBusData == nullptr) {
        return false;
    }

    USHORT command = 0;

    if (bus_.GetBusData(bus_.Context, PCI_WHICHSPACE_CONFIG, &command,
        pci_command_offset, sizeof(command)) != sizeof(command) || command == MAXUSHORT) {
        return false;
    }

    USHORT wanted = enable ? static_cast<USHORT>(command | pci_master_enable) :
        static_cast<USHORT>(command & ~pci_master_enable);

    if (bus_.SetBusData(bus_.Context, PCI_WHICHSPACE_CONFIG,
        &wanted, pci_command_offset, sizeof(wanted)) != sizeof(wanted)) {
        return false;
    }

    USHORT observed = 0;
    return bus_.GetBusData(bus_.Context, PCI_WHICHSPACE_CONFIG, &observed,
        pci_command_offset, sizeof(observed)) == sizeof(observed) && observed == wanted;
}

// A surprise-removed function whose vendor ID reads 0xFFFF on two reads is no
// longer on the bus (QEMU device_del, a physical unplug): it can neither DMA
// into nor interrupt the guest, which proves stop without MMIO. The HAL answers
// only the vendor word for an empty slot, so exactly that word is read.
bool NativeTransport::function_absent() const noexcept {
    if (!bus_acquired_ || bus_.GetBusData == nullptr) {
        return false;
    }

    for (int read = 0; read < 2; ++read) {
        USHORT vendor = 0;

        if (bus_.GetBusData(bus_.Context, PCI_WHICHSPACE_CONFIG, &vendor, 0, sizeof(vendor)) !=
            sizeof(vendor) || vendor != MAXUSHORT) {
            return false;
        }
    }

    return true;
}

void NativeTransport::find_aperture(PCM_RESOURCE_LIST raw, PCM_RESOURCE_LIST translated) noexcept {
    aperture_bytes_ = 0;
    ULONG bar_index = 0;
    ULONG mib = 0;

    if (!read(QLV_MMIO_REG_APERTURE_BAR, bar_index) || !read(QLV_MMIO_REG_APERTURE_MIB, mib) ||
        (bar_index != QLV_APERTURE_BAR && bar_index != QLV_LARGE_APERTURE_BAR)) {
        return;
    }

    const uint64_t bytes = static_cast<uint64_t>(mib) << QLV_MIB_SHIFT;

    // NT5 resource lengths are32-bit. A later large-aperture backend needs
    // MemoryLarge support; do not truncate it into a false usable mapping.
    if (bytes < QLV_APERTURE_BYTES || bytes > MAXULONG || (bytes & (bytes - 1)) != 0) {
        return;
    }

    uint8_t config_bytes[rx7900::pci_header_bytes]{};

    if (bus_.GetBusData(bus_.Context, PCI_WHICHSPACE_CONFIG, config_bytes, 0,
        sizeof(config_bytes)) != sizeof(config_bytes)) {
        return;
    }

    rx7900::ByteView config(config_bytes, sizeof(config_bytes));
    uint32_t low = 0;
    uint32_t high = 0;
    config.read_u32(rx7900::pci_bars_offset + bar_index * 4, low);

    if ((low & 1) != 0 || (low & 6) == 2 || (low & 6) == 6) {
        return;
    }

    if ((low & 6) == 4) {
        config.read_u32(rx7900::pci_bars_offset + (bar_index + 1) * 4, high);
    }

    const uint64_t bus_base = (static_cast<uint64_t>(high) << 32) | (low & ~0xfu);

    if (bus_base == 0) {
        return;
    }

    for (ULONG index = 0; index < raw->List[0].PartialResourceList.Count; ++index) {
        const auto& source = raw->List[0].PartialResourceList.PartialDescriptors[index];
        const auto& target = translated->List[0].PartialResourceList.PartialDescriptors[index];

        if (source.Type == CmResourceTypeMemory && target.Type == CmResourceTypeMemory &&
            static_cast<uint64_t>(source.u.Memory.Start.QuadPart) == bus_base &&
            source.u.Memory.Length == bytes && target.u.Memory.Length == bytes &&
            target.u.Memory.Start.QuadPart >= 0 &&
            static_cast<uint64_t>(target.u.Memory.Start.QuadPart) <= MAXLONGLONG - (bytes - 1) &&
            (target.Flags & CM_RESOURCE_MEMORY_READ_ONLY) == 0) {
            if (aperture_bytes_ != 0) {
                aperture_bytes_ = 0;
                return;
            }

            aperture_base_ = target.u.Memory.Start;
            aperture_bytes_ = bytes;
        }
    }
}

bool NativeTransport::aperture(PHYSICAL_ADDRESS& base, uint64_t& bytes) const noexcept {
    if (aperture_bytes_ == 0 || !ready_ || quarantined_) {
        return false;
    }

    base = aperture_base_;
    bytes = aperture_bytes_;
    return true;
}

bool NativeTransport::read(ULONG offset, ULONG& value) const noexcept {
    if (registers_ == nullptr || (offset & 3) != 0 || offset > QLV_CONTROL_BAR_BYTES - sizeof(ULONG)) {
        return false;
    }

    // The assigned mapping owns the whole checked aligned dword. Native register
    // intrinsics provide device access semantics; no CPU physical pointer cast.
    value = READ_REGISTER_ULONG(reinterpret_cast<PULONG>(registers_ + offset));
    return value != MAXULONG;
}

bool NativeTransport::write(ULONG offset, ULONG value) const noexcept {
    if (registers_ == nullptr || (offset & 3) != 0 || offset > QLV_CONTROL_BAR_BYTES - sizeof(ULONG)) {
        return false;
    }

    WRITE_REGISTER_ULONG(reinterpret_cast<PULONG>(registers_ + offset), value);
    return true;
}

bool NativeTransport::ring_usable() const noexcept {
    return ring_active_ && ring_.health() == ring::Health::active;
}

void NativeTransport::announce() noexcept {
    if (!announce_owed_) {
        return;
    }

    announce_owed_ = false;

    // A faulted or lost ring is not rung; the next wait reports it.
    if (ring_usable() && ring_.doorbell_needed()) {
        write(QLV_MMIO_REG_DOORBELL, 1);
#if defined(RX7900_LIGHTNING_TRANSPORT_TIMING)
        timing_.add(timing_.ring.doorbells, 1);
#endif
    }
}

void NativeTransport::begin_batch() noexcept {
    batching_ = true;
#if defined(RX7900_LIGHTNING_TRANSPORT_TIMING)
    timing_.add(timing_.ring.batches, 1);
#endif
}

void NativeTransport::end_batch() noexcept {
    batching_ = false;
    announce();
}

void NativeTransport::note_submission_wait(bool ledger_full) noexcept {
#if defined(RX7900_LIGHTNING_TRANSPORT_TIMING)
    timing_.add(ledger_full ? timing_.ring.ledger_waits : timing_.ring.busy_waits, 1);
    if (batching_) {
        timing_.add(timing_.ring.batched_waits, 1);
    }
#else
    (void)ledger_full;
#endif
}

// Wait for entry sequence to retire (or, with drain, for every entry). The
// spin reads guest RAM only; it is bounded by the calibrated TSC budget and
// then sleeps 1 ms at a time under the mailbox's real elapsed-time deadline.
NTSTATUS NativeTransport::ring_wait(uint32_t sequence, bool drain, bool& waited, bool& slept) noexcept {
    const auto done = [this, sequence, drain]() noexcept {
        return drain ? ring_.drained() : ring_.completed(sequence);
    };
    waited = slept = false;
    // Nothing is waited for before QEMU has been told about it.
    announce();

    if (done()) {
        return STATUS_SUCCESS;
    }

    waited = true;
    const ULONGLONG began = KeQueryInterruptTime();
    const uint64_t spin_began = __rdtsc();

    for (ULONG iteration = 0;; ++iteration) {
        if (done()) {
#if defined(RX7900_LIGHTNING_TRANSPORT_TIMING)
            timing_.add(timing_.ring.spin_complete, 1);
#endif
            return STATUS_SUCCESS;
        }

        if (ring_.health() != ring::Health::active) {
            return STATUS_DEVICE_PROTOCOL_ERROR;
        }

        if (spin_ticks_ != 0 ? __rdtsc() - spin_began >= spin_ticks_ :
            iteration >= transport_wait::ring_spin_iterations) {
            break;
        }

        YieldProcessor();
    }

    LARGE_INTEGER delay;
    delay.QuadPart = -static_cast<LONGLONG>(poll_milliseconds * 10000);

    for (ULONG sleeps = 0; sleeps < transport_wait::maximum_sleeps &&
         KeQueryInterruptTime() - began < QLV_KERNEL_EXCHANGE_TIMEOUT_100NS; ++sleeps) {
        slept = true;
#if defined(RX7900_LIGHTNING_TRANSPORT_TIMING)
        timing_.add(timing_.ring.sleeps, 1);
#endif
        KeDelayExecutionThread(KernelMode, FALSE, &delay);

        if (done()) {
            return STATUS_SUCCESS;
        }

        if (ring_.health() != ring::Health::active) {
            return STATUS_DEVICE_PROTOCOL_ERROR;
        }
    }

    return STATUS_IO_TIMEOUT;
}

NTSTATUS NativeTransport::transact(PVOID request, PHYSICAL_ADDRESS address, ULONG bytes,
                                   ULONG capacity, ULONG& returned, bool* posted) noexcept {
#if defined(RX7900_LIGHTNING_TRANSPORT_TIMING)
    uint16_t operation = 0;
    uint64_t context = 0;
    // Internal common-buffer snapshot, not caller memory or an MMIO overlay.
    if (request != nullptr && bytes >= QLV_HEADER_BYTES && bytes <= QLV_HEADER_BYTES + QLV_MAX_PAYLOAD_BYTES) {
        const rx7900::ByteView snapshot(static_cast<const uint8_t*>(request), bytes);
        snapshot.read_u16(QLV_OPERATION_OFFSET, operation);
        snapshot.read_u64(QLV_CONTEXT_OFFSET, context);
    }
    transport_timing::Scope timing(timing_, operation, context);
#else
    UnmeasuredScope timing;
#endif
    returned = 0;
    ULONG status = 0;
    ULONG magic = 0;
    const bool may_post = posted != nullptr && *posted;

    if (posted != nullptr) {
        *posted = false;
    }

    if (quarantined_ || in_flight_ || request == nullptr || response_ == nullptr ||
        bytes < QLV_HEADER_BYTES || bytes > QLV_HEADER_BYTES + QLV_MAX_PAYLOAD_BYTES ||
        capacity < QLV_HEADER_BYTES || capacity > response_capacity) {
        return timing.finish(STATUS_DEVICE_NOT_READY);
    }

    if (ring_active_) {
        // A lost or faulted ring is recovered by RESET, never bypassed.
        if (!ring_usable()) {
            return timing.finish(STATUS_DEVICE_NOT_READY);
        }

        bool waited = false;
        bool slept = false;

        if (bytes > ring::slot_bytes) {
            // Oversized requests keep the mailbox, after every queued entry.
#if defined(RX7900_LIGHTNING_TRANSPORT_TIMING)
            timing_.add(timing_.ring.mailbox, 1);
#endif
            const NTSTATUS drained = ring_wait(0, true, waited, slept);

            if (!NT_SUCCESS(drained)) {
                in_flight_ = true;
                return timing.finish(drained);
            }
        } else {
            uint16_t operation = 0;
            rx7900::ByteView(static_cast<const uint8_t*>(request), bytes).read_u16(QLV_OPERATION_OFFSET, operation);
            const bool post = may_post && (operation == QLV_SUBMIT ||
                (operation == QLV_WRITE_SYNC && posted_writes_));

            if (ring_.full()) {
                const NTSTATUS space = ring_wait(ring_.tail() - ring::entries + 1, false, waited, slept);

                if (!NT_SUCCESS(space)) {
                    in_flight_ = true;
                    return timing.finish(space);
                }
            }

            // The slot is ours until QEMU copies it (HEAD), checked above.
            RtlCopyMemory(static_cast<uint8_t*>(ring_slots_) + size_t{ring_.next_slot()} * ring::slot_bytes,
                request, bytes);

            if (!post) {
                RtlZeroMemory(response_, capacity);
            }

            ever_submitted_ = true;
            // Inside a batch a posted entry leaves the IDLE probe to the next
            // probing publish or announce; a probe covers every earlier TAIL.
            const bool probe = !(post && batching_);
            const auto published = ring_.publish(post ? static_cast<uint32_t>(QLV_RING_POSTED) : 0,
                bytes, post ? 0 : capacity, probe);
            announce_owed_ = !probe;
#if defined(RX7900_LIGHTNING_TRANSPORT_TIMING)
            if (!probe) {
                timing_.add(timing_.ring.deferred, 1);
            }
#endif

            if (published.doorbell) {
                write(QLV_MMIO_REG_DOORBELL, 1);
#if defined(RX7900_LIGHTNING_TRANSPORT_TIMING)
                timing_.add(timing_.ring.doorbells, 1);
#endif
            }

            timing.after_submit();

            if (post) {
#if defined(RX7900_LIGHTNING_TRANSPORT_TIMING)
                timing_.add(timing_.ring.posted, 1);
                if (operation == QLV_WRITE_SYNC) {
                    timing_.add(timing_.ring.posted_writes, 1);
                }
#endif
                posted_sequence_ = published.sequence;
                *posted = true;
                timing.complete();
                return timing.finish(STATUS_SUCCESS);
            }

#if defined(RX7900_LIGHTNING_TRANSPORT_TIMING)
            timing_.add(timing_.ring.synchronous, 1);
#endif
            in_flight_ = true;
            const NTSTATUS completed = ring_wait(published.sequence, false, waited, slept);

            if (waited) {
                timing.pending();
            }

            if (slept) {
                timing.before_sleep();
            }

            if (!NT_SUCCESS(completed)) {
                return timing.finish(completed);
            }

            if (ring_.result_status() != QLV_MMIO_COMPLETE) {
                return timing.finish(STATUS_DEVICE_PROTOCOL_ERROR);
            }

            returned = ring_.result_bytes();

            if (returned < QLV_HEADER_BYTES || returned > capacity) {
                return timing.finish(STATUS_DATA_ERROR);
            }

            timing.complete();
            return timing.finish(STATUS_SUCCESS);
        }
    }

    if (!read(QLV_MMIO_REG_MAGIC, magic) || magic != QLV_MAGIC ||
        !read(QLV_MMIO_REG_STATUS, status) ||
        (status != QLV_MMIO_IDLE && status != QLV_MMIO_COMPLETE)) {
        return timing.finish(STATUS_DEVICE_NOT_READY);
    }

    RtlZeroMemory(response_, capacity);
    write(QLV_MMIO_REG_IRQ_ENABLE, 0);
    write(QLV_MMIO_REG_IRQ, static_cast<ULONG>(QLV_MMIO_COMPLETION_IRQ));
    write(QLV_MMIO_REG_REQUEST_LO, address.LowPart);
    write(QLV_MMIO_REG_REQUEST_HI, 0);
    write(QLV_MMIO_REG_REQUEST_BYTES, bytes);
    write(QLV_MMIO_REG_RESPONSE_LO, response_address_.LowPart);
    write(QLV_MMIO_REG_RESPONSE_HI, 0);
    write(QLV_MMIO_REG_RESPONSE_CAPACITY, capacity);
    KeMemoryBarrier();
    ever_submitted_ = true;
    in_flight_ = true;
    write(QLV_MMIO_REG_COMMAND, static_cast<ULONG>(QLV_MMIO_SUBMIT));
    timing.after_submit();
    const ULONGLONG began = KeQueryInterruptTime();
    LARGE_INTEGER delay;
    delay.QuadPart = -static_cast<LONGLONG>(poll_milliseconds * 10000);

    transport_wait::Budget wait_budget;
    for (;;) {
        if (!read(QLV_MMIO_REG_STATUS, status)) {
            return timing.finish(STATUS_DEVICE_NOT_CONNECTED);
        }

        if (status == QLV_MMIO_COMPLETE) {
            KeMemoryBarrier();

            if (!read(QLV_MMIO_REG_RESPONSE_BYTES, returned) || returned < QLV_HEADER_BYTES || returned > capacity) {
                return timing.finish(STATUS_DATA_ERROR);
            }

            timing.complete();
            return timing.finish(STATUS_SUCCESS);
        }

        if (status != QLV_MMIO_PENDING) {
            return timing.finish(STATUS_DEVICE_PROTOCOL_ERROR);
        }

        timing.pending();
        const auto pause = wait_budget.next(KeQueryInterruptTime() - began);
        if (pause == transport_wait::Pause::expired) {
            break;
        }
        if (pause == transport_wait::Pause::stall) {
            timing.stall();
            KeStallExecutionProcessor(transport_wait::short_poll_microseconds);
            continue;
        }

        timing.before_sleep();
        KeDelayExecutionThread(KernelMode, FALSE, &delay);
    }

    return timing.finish(STATUS_IO_TIMEOUT);
}

NTSTATUS NativeTransport::exchange(uint16_t operation, uint64_t context, Reply& output) noexcept {
    output = {};

    if (encode_request(operation, context, {static_cast<uint8_t*>(request_), dma_bytes}) != ProtocolStatus::valid) {
        return STATUS_INVALID_PARAMETER;
    }

    ULONG count = 0;
    const NTSTATUS status = transact(request_, request_address_, static_cast<ULONG>(QLV_HEADER_BYTES),
                                     static_cast<ULONG>(response_capacity), count);

    if (!NT_SUCCESS(status)) {
        return status;
    }

    const auto decoded = decode_reply(operation, context, {static_cast<const uint8_t*>(response_), count}, output);

    if (decoded != ProtocolStatus::valid && decoded != ProtocolStatus::host_error) {
        return STATUS_DATA_ERROR;
    }

    in_flight_ = false;
    return decoded == ProtocolStatus::valid ? STATUS_SUCCESS : host_error(output.host_status);
}

void NativeTransport::release_large_request() noexcept {
    if (large_request_ != nullptr) {
        adapter_->DmaOperations->FreeCommonBuffer(adapter_, large_request_bytes_,
            large_request_address_, large_request_, FALSE);
        large_request_ = nullptr;
        large_request_bytes_ = 0;
    }
}

NTSTATUS NativeTransport::renderer_exchange(rx7900::ByteView packet, size_t capacity,
    const ExchangeSession& session, ExchangeReply& output,
    const PrivateSubmitFence* fence, bool* fused) noexcept {
    output = {};

    if (fused != nullptr) {
        *fused = false;
    }

    if (!ready_ || quarantined_ || in_flight_) {
        return STATUS_DEVICE_NOT_READY;
    }

    const size_t actual_capacity = capacity < response_capacity ? capacity : response_capacity;
    ValidatedExchange validated;
    const auto policy = validate_exchange_request(packet, actual_capacity, session, validated);

    if (policy != ExchangeStatus::validated) {
        return policy == ExchangeStatus::resource_busy ? STATUS_DEVICE_BUSY : STATUS_INVALID_PARAMETER;
    }

    const uint64_t caller_sync_count = validated.header().offset;
    const uint16_t operation = validated.header().operation;
    const uint64_t context = validated.header().context;

    if (ring_usable()) {
        uint32_t failures = 0;
        uint64_t failed_context = 0;
        ring_.posted_errors(failures, failed_context);
        ring_failures_.observe(failures, failed_context);

        // A failed posted SUBMIT poisons its context, as a lost device would.
        if (operation != QLV_OPEN && operation != QLV_CLOSE && ring_failures_.lost(context)) {
#if defined(RX7900_LIGHTNING_TRANSPORT_TIMING)
            timing_.add(timing_.ring.local_lost, 1);
#endif
            return synthesized_reply(validated, packet, 0, QLV_STATUS_DEVICE_LOST, output) ?
                STATUS_SUCCESS : STATUS_DATA_ERROR;
        }

        if (local_answer(validated, packet, output)) {
            return STATUS_SUCCESS;
        }
    }

    PVOID target = request_;
    PHYSICAL_ADDRESS address = request_address_;
    ULONG target_bytes = dma_bytes;

    if (packet.size() > dma_bytes) {
        const ULONG rounded = static_cast<ULONG>((packet.size() + dma_bytes - 1) & ~(dma_bytes - 1));

        if (large_request_bytes_ < rounded) {
            release_large_request();
            large_request_ = adapter_->DmaOperations->AllocateCommonBuffer(adapter_, rounded,
                &large_request_address_, FALSE);

            if (large_request_ == nullptr) {
                return STATUS_INSUFFICIENT_RESOURCES;
            }

            large_request_bytes_ = rounded;

            if (!dma_span(large_request_address_, rounded)) {
                release_large_request();
                return STATUS_INSUFFICIENT_RESOURCES;
            }
        }

        target = large_request_;
        address = large_request_address_;
        target_bytes = large_request_bytes_;
    }

    const bool can_fuse = fence != nullptr && fused != nullptr &&
        fence->sync != 0 && fence->point != 0 &&
        validated.header().operation == QLV_SUBMIT &&
        validated.header().offset < QLV_MAX_SUBMIT_SYNCS &&
        validated.header().payload_bytes <= QLV_MAX_PAYLOAD_BYTES - QLV_SYNC_POINT_BYTES &&
        packet.size() + QLV_SYNC_POINT_BYTES <= target_bytes;
    size_t request_bytes = packet.size();
    const auto snapshot = can_fuse ? snapshot_submit_with_private_fence(packet,
        actual_capacity, session, fence->sync, fence->point,
        static_cast<uint8_t*>(target), target_bytes, validated, request_bytes) :
        snapshot_exchange_request(packet, actual_capacity, session,
            {static_cast<uint8_t*>(target), target_bytes}, validated);

    if (snapshot != ExchangeStatus::validated) {
        return STATUS_INVALID_PARAMETER;
    }

    // Ask a publishing host for a timeline slot. The private snapshot changes,
    // never the caller's validated packet; QEMU strips the flag without a ring.
    const rx7900::sdma::CommandBuffer staged(static_cast<uint8_t*>(target), target_bytes);

    if (operation == QLV_CREATE_SYNC && ring_usable() && host_publish_ &&
        !staged.write_u32(QLV_FLAGS_OFFSET, static_cast<uint32_t>(QLV_SYNC_PUBLISH))) {
        return STATUS_INVALID_PARAMETER;
    }

    if (operation == QLV_DESTROY_SYNC) {
        ring_syncs_.erase(validated.header().resource);
    }

    ULONG count = 0;
    bool posted = post_submits_ && (operation == QLV_SUBMIT ||
        (operation == QLV_WRITE_SYNC && posted_writes_));
    const NTSTATUS status = transact(target, address, static_cast<ULONG>(request_bytes),
                                     static_cast<ULONG>(actual_capacity), count, &posted);

    if (!NT_SUCCESS(status)) {
        DbgPrint("LightningNT5: renderer exchange operation %u failed %08lx in-flight %u\n",
            static_cast<unsigned>(validated.header().operation), static_cast<ULONG>(status),
            static_cast<unsigned>(in_flight_));
        return status;
    }

    if (posted) {
        // Queued in order ahead of any later exchange; the host's reply is
        // header-only success, so publish exactly that.
        if (!synthesized_reply(validated, packet, 0, QLV_STATUS_SUCCESS, output)) {
            return STATUS_DATA_ERROR;
        }

        // The page may show this sync's old value until the write retires.
        if (operation == QLV_WRITE_SYNC) {
            ring_syncs_.posted_write(validated.header().resource, context, posted_sequence_);
        }

        if (can_fuse) {
            output.header.offset = caller_sync_count;
            *fused = true;
        }

        return STATUS_SUCCESS;
    }

    // A publishing host returns slot + 1 in offset and echoes the flag; keep
    // both private and restore the reply the caller's request implies.
    uint64_t published_slot = 0;

    if (operation == QLV_CREATE_SYNC && count >= QLV_HEADER_BYTES) {
        const rx7900::sdma::CommandBuffer reply(static_cast<uint8_t*>(response_), count);
        rx7900::ByteView(static_cast<const uint8_t*>(response_), count).read_u64(QLV_OFFSET_OFFSET, published_slot);
        reply.write_u32(QLV_OFFSET_OFFSET, 0);
        reply.write_u32(QLV_OFFSET_OFFSET + 4, 0);
        reply.write_u32(QLV_FLAGS_OFFSET, validated.header().flags);
    }

    const auto result = validate_exchange_reply(validated, {static_cast<const uint8_t*>(response_), count}, output);

    if (result != ExchangeStatus::validated && result != ExchangeStatus::host_error) {
        const rx7900::ByteView response(static_cast<const uint8_t*>(response_), count);
        uint32_t response_value = 0;
        uint32_t response_bytes = 0;
        uint32_t response_offset = 0;
        response.read_u32(QLV_VALUE_OFFSET, response_value);
        response.read_u32(QLV_BYTES_OFFSET, response_bytes);
        response.read_u32(QLV_OFFSET_OFFSET, response_offset);
        const auto& expected = validated.header();
        DbgPrint("LightningNT5: reply operation %u policy %u value %lu/%lu bytes %lu/%lu offset %lu/%lu\n",
            static_cast<unsigned>(expected.operation), static_cast<unsigned>(result),
            static_cast<ULONG>(response_value), static_cast<ULONG>(expected.value),
            static_cast<ULONG>(response_bytes), static_cast<ULONG>(expected.bytes),
            static_cast<ULONG>(response_offset), static_cast<ULONG>(expected.offset));
        return STATUS_DATA_ERROR;
    }

    in_flight_ = false;

    if (result == ExchangeStatus::validated && operation == QLV_CREATE_SYNC && ring_usable() &&
        published_slot >= 1 && published_slot <= ring::timeline_slots &&
        ring_syncs_.insert(static_cast<uint32_t>(published_slot - 1), output.header.resource, context)) {
#if defined(RX7900_LIGHTNING_TRANSPORT_TIMING)
        timing_.add(timing_.ring.published_syncs, 1);
#endif
    }

    if (operation == QLV_CLOSE && output.header.status == QLV_STATUS_SUCCESS) {
        ring_syncs_.erase_context(context);
        ring_failures_.forget(context);
    }

    if (can_fuse) {
        // The added point is a private lifetime record, not a caller sync.
        output.header.offset = caller_sync_count;

        if (fused != nullptr) {
            *fused = true;
        }
    }
    if (validated.header().operation == QLV_CLOSE && output.header.status == QLV_STATUS_SUCCESS) {
        dump_timing();
    }
    return STATUS_SUCCESS;
}

bool NativeTransport::synthesized_reply(const ValidatedExchange& validated, rx7900::ByteView,
    uint64_t value, int32_t status, ExchangeReply& output) noexcept {
    // The response buffer is idle here: posted entries never write it and no
    // synchronous entry is outstanding while the serialized caller runs.
    const rx7900::sdma::CommandBuffer reply(static_cast<uint8_t*>(response_), QLV_HEADER_BYTES);

    if (!ring::local_reply(validated.header(), reply, value, status)) {
        return false;
    }

    const auto result = validate_exchange_reply(validated,
        {static_cast<const uint8_t*>(response_), QLV_HEADER_BYTES}, output);
    return result == ExchangeStatus::validated || result == ExchangeStatus::host_error;
}

// READ_SYNC and zero-timeout WAIT_SYNCS from host-published values. False
// means exchange instead: unpublished, foreign, unstable, or a periodic refresh.
bool NativeTransport::local_answer(const ValidatedExchange& validated, rx7900::ByteView packet,
                                   ExchangeReply& output) noexcept {
    const auto& header = validated.header();
    ring_syncs_.retire_writes(ring_.retired());

    if (header.operation == QLV_READ_SYNC) {
        uint32_t slot = 0;
        uint64_t value = 0;

        if (!ring_syncs_.find(header.resource, header.context, slot) || !ring_syncs_.settled(slot) ||
            ring::read_timeline<KernelBarriers>(ring_.control(), slot,
                static_cast<uint32_t>(header.resource), value) != ring::TimelineRead::value) {
            return false;
        }

        if (ring_syncs_.refresh(slot, value)) {
#if defined(RX7900_LIGHTNING_TRANSPORT_TIMING)
            timing_.add(timing_.ring.refreshes, 1);
#endif
            return false;
        }

        if (!synthesized_reply(validated, packet, value, QLV_STATUS_SUCCESS, output)) {
            return false;
        }

#if defined(RX7900_LIGHTNING_TRANSPORT_TIMING)
        timing_.add(timing_.ring.local_reads, 1);
#endif
        return true;
    }

    if (header.operation != QLV_WAIT_SYNCS || header.value != 0 || header.bytes > QLV_MAX_WAIT_SYNCS) {
        return false;
    }

    rx7900::ByteView points(nullptr, 0);
    bool satisfied = false;

    if (!packet.subview(QLV_HEADER_BYTES, header.payload_bytes, points) ||
        !ring::local_wait<KernelBarriers>(ring_.control(), ring_syncs_, header.context, points,
            static_cast<uint32_t>(header.bytes), (header.flags & QLV_WAIT_ANY) != 0, satisfied)) {
        return false;
    }

    if (satisfied) {
        ring_waits_.satisfied();
    } else if (ring_waits_.refresh_after_unsatisfied()) {
#if defined(RX7900_LIGHTNING_TRANSPORT_TIMING)
        timing_.add(timing_.ring.refreshes, 1);
#endif
        return false;
    }

    if (!synthesized_reply(validated, packet, 0,
        satisfied ? QLV_STATUS_SUCCESS : QLV_STATUS_TIMEOUT, output)) {
        return false;
    }

#if defined(RX7900_LIGHTNING_TRANSPORT_TIMING)
    timing_.add(timing_.ring.local_waits, 1);
#endif
    return true;
}

bool NativeTransport::published(uint64_t context, uint64_t token) const noexcept {
    uint32_t slot = 0;
    return ring_usable() && ring_syncs_.find(token, context, slot);
}

bool NativeTransport::await_published(uint64_t context, uint64_t token, uint64_t value) noexcept {
    uint32_t slot = 0;
    announce();

    if (!ring_usable() || !ring_syncs_.find(token, context, slot)) {
        return false;
    }

    const uint64_t began = __rdtsc();

    for (ULONG iteration = 0;; ++iteration) {
        uint64_t current = 0;
        const auto read = ring::read_timeline<KernelBarriers>(ring_.control(), slot,
            static_cast<uint32_t>(token), current);

        if (read == ring::TimelineRead::absent) {
            return false;
        }

        if (read == ring::TimelineRead::value && current > value) {
            return true;
        }

        if (spin_ticks_ != 0 ? __rdtsc() - began >= spin_ticks_ / transport_wait::published_spin_divisor :
            iteration >= transport_wait::ring_spin_iterations / transport_wait::published_spin_divisor) {
            return false;
        }

        YieldProcessor();
    }
}

void NativeTransport::dump_timing() noexcept {
#if defined(RX7900_LIGHTNING_TRANSPORT_TIMING)
    if (KeGetCurrentIrql() != PASSIVE_LEVEL) { return; }
    timing_.add(timing_.dumps, 1);
    // Each line fits the kernel debug-print boundary even with saturated
    // 64-bit fields. Dump only at context close or a native lifecycle boundary.
    DbgPrint("QLVTIME dump %I64u counts_only 1 clock_reads 0 saturated %u cumulative 1\n",
        timing_.dumps, static_cast<unsigned>(timing_.saturated));
    for (size_t operation = 0; operation < transport_timing::Aggregates::row_count; ++operation) {
        const auto& row = timing_.rows[operation];
        if (row.calls == 0) { continue; }
        DbgPrint("QLVTIME op %u context %I64u mixed %u calls %I64u ok %I64u fail %I64u "
            "submitted %I64u immediate %I64u pending %I64u sleeps %I64u stalls %I64u short_complete %I64u status %08lx\n",
            static_cast<unsigned>(operation), row.context, static_cast<unsigned>(row.mixed_contexts),
            row.calls, row.succeeded, row.failed, row.submitted, row.immediate,
            row.pending, row.sleeps, row.stalls, row.short_complete, row.last_status);
    }
    const auto& ring_row = timing_.ring;
    DbgPrint("QLVRING active %u publish %u doorbells %I64u sync %I64u posted %I64u mailbox %I64u "
        "local_reads %I64u local_waits %I64u local_lost %I64u refreshes %I64u\n",
        static_cast<unsigned>(ring_active_), static_cast<unsigned>(host_publish_), ring_row.doorbells,
        ring_row.synchronous, ring_row.posted, ring_row.mailbox, ring_row.local_reads,
        ring_row.local_waits, ring_row.local_lost, ring_row.refreshes);
    DbgPrint("QLVRING spin_complete %I64u sleeps %I64u published_syncs %I64u spin_ticks %I64u\n",
        ring_row.spin_complete, ring_row.sleeps, ring_row.published_syncs, spin_ticks_);
    DbgPrint("QLVRING posted_writes_supported %u batches %I64u deferred %I64u posted_writes %I64u\n",
        static_cast<unsigned>(posted_writes_), ring_row.batches, ring_row.deferred, ring_row.posted_writes);
    DbgPrint("QLVRING busy_waits %I64u ledger_waits %I64u batched_waits %I64u\n",
        ring_row.busy_waits, ring_row.ledger_waits, ring_row.batched_waits);
#endif
}

NTSTATUS NativeTransport::claim_renderer() noexcept {
    if (!ready_ || quarantined_) {
        return STATUS_DEVICE_NOT_READY;
    }

    if (context_ != 0) {
        Reply ignored;
        const NTSTATUS status = exchange(static_cast<uint16_t>(QLV_CLOSE), context_, ignored);

        if (!NT_SUCCESS(status)) {
            return status;
        }

        context_ = 0;
    }

    return STATUS_SUCCESS;
}

NTSTATUS NativeTransport::recover_renderer() noexcept {
    ready_ = false;

    if (quarantined_ || !reset()) {
        bus_master(false);
        quarantined_ = true;
        return STATUS_DEVICE_BUSY;
    }

    const NTSTATUS status = resume();
    return NT_SUCCESS(status) ? claim_renderer() : status;
}

bool NativeTransport::reset() noexcept {
    // RESET unregisters the ring in QEMU; the guest view goes first.
    forget_ring();

    for (unsigned attempt = 0; attempt < 3; ++attempt) {
        ULONG status = 0;
        ULONG bytes = 1;
        write(QLV_MMIO_REG_IRQ_ENABLE, 0);
        write(QLV_MMIO_REG_COMMAND, static_cast<ULONG>(QLV_MMIO_RESET));

        // QEMU disconnects handlers/timers before ERROR, then reconnects later.
        // IDLE alone cannot prove cancellation of this generation's request.
        if (read(QLV_MMIO_REG_STATUS, status) && status == QLV_MMIO_ERROR &&
            read(QLV_MMIO_REG_RESPONSE_BYTES, bytes) && bytes == 0) {
            in_flight_ = false;
            context_ = 0;
            return true;
        }
    }

    return false;
}

void NativeTransport::forget_ring() noexcept {
    ring_active_ = false;
    host_publish_ = false;
    posted_writes_ = false;
    announce_owed_ = false;
    ring_.detach();
    ring_syncs_.clear();
    ring_failures_.clear();
    ring_waits_ = {};
}

// One-time TSC calibration against the HAL's calibrated stall at PnP start;
// a budget, not a completion wait. Zero keeps the iteration bound.
void NativeTransport::calibrate_spin() noexcept {
    if (spin_ticks_ != 0) {
        return;
    }

    const uint64_t began = __rdtsc();
    KeStallExecutionProcessor(transport_wait::ring_calibration_microseconds);
    const uint64_t elapsed = __rdtsc() - began;
    spin_ticks_ = elapsed >= transport_wait::ring_calibration_microseconds ?
        elapsed / transport_wait::ring_calibration_microseconds * transport_wait::ring_spin_microseconds : 0;
}

// Register both ring buffers and the response buffer. Any refusal leaves the
// mailbox in use; nothing here is required for a working transport.
bool NativeTransport::register_ring() noexcept {
    ring_active_ = false;

    if ((capabilities_.transport_capabilities & QLV_MMIO_SHARED_RING) == 0 ||
        ring_control_ == nullptr || ring_slots_ == nullptr ||
        !ring_.initialize({static_cast<volatile uint32_t*>(ring_control_), ring::control_bytes})) {
        return false;
    }

    ULONG state = 0;
    write(QLV_MMIO_REG_RING_LO, ring_control_address_.LowPart);
    write(QLV_MMIO_REG_RING_HI, 0);
    write(QLV_MMIO_REG_RING_DATA_LO, ring_slots_address_.LowPart);
    write(QLV_MMIO_REG_RING_DATA_HI, 0);
    write(QLV_MMIO_REG_RESPONSE_LO, response_address_.LowPart);
    write(QLV_MMIO_REG_RESPONSE_HI, 0);
    write(QLV_MMIO_REG_RESPONSE_CAPACITY, dma_bytes);
    write(QLV_MMIO_REG_COMMAND, static_cast<ULONG>(QLV_MMIO_REGISTER));

    if (!read(QLV_MMIO_REG_RING_STATUS, state) || state != QLV_RING_ACTIVE ||
        ring_.health() != ring::Health::active) {
        ring_.detach();
        return false;
    }

    calibrate_spin();
    ring_active_ = true;
    posted_writes_ = (capabilities_.transport_capabilities & QLV_MMIO_POSTED_SYNC_WRITES) != 0;
    return true;
}

void NativeTransport::release_resources() noexcept {
    forget_ring();

    if (adapter_ != nullptr && adapter_->DmaOperations != nullptr) {
        release_large_request();

        if (ring_slots_ != nullptr) {
            adapter_->DmaOperations->FreeCommonBuffer(adapter_, static_cast<ULONG>(ring::data_bytes),
                ring_slots_address_, ring_slots_, FALSE);
            ring_slots_ = nullptr;
        }

        if (ring_control_ != nullptr) {
            adapter_->DmaOperations->FreeCommonBuffer(adapter_, static_cast<ULONG>(ring::control_bytes),
                ring_control_address_, ring_control_, FALSE);
            ring_control_ = nullptr;
        }

        if (response_ != nullptr) {
            adapter_->DmaOperations->FreeCommonBuffer(adapter_, dma_bytes,
                response_address_, response_, FALSE);
            response_ = nullptr;
        }

        if (request_ != nullptr) {
            adapter_->DmaOperations->FreeCommonBuffer(adapter_, dma_bytes,
                request_address_, request_, FALSE);
            request_ = nullptr;
        }

        adapter_->DmaOperations->PutDmaAdapter(adapter_);
        adapter_ = nullptr;
    }

    if (registers_ != nullptr) {
        MmUnmapIoSpace(registers_, static_cast<ULONG>(QLV_CONTROL_BAR_BYTES));
        registers_ = nullptr;
    }

    if (bus_acquired_) {
        if (bus_.InterfaceDereference != nullptr) {
            bus_.InterfaceDereference(bus_.Context);
        }

        bus_ = {};
        bus_acquired_ = false;
    }

    identity_valid_ = false;
    aperture_bytes_ = 0;
    ever_submitted_ = false;
    context_ = 0;
}

NTSTATUS NativeTransport::start(PDEVICE_OBJECT pdo,
                               PCM_RESOURCE_LIST raw, PCM_RESOURCE_LIST translated) noexcept {
    if (KeGetCurrentIrql() != PASSIVE_LEVEL || pdo == nullptr ||
        bus_acquired_ || adapter_ != nullptr || registers_ != nullptr || quarantined_) {
        return STATUS_INVALID_DEVICE_STATE;
    }

    capabilities_ = {};
    // BUS_INTERFACE_STANDARD belongs to this referenced PCI PDO. ReactOS
    // VideoPort does not forward QUERY_INTERFACE through its function device.
    NTSTATUS status = acquire_bus(pdo);
    PHYSICAL_ADDRESS base{};

    if (NT_SUCCESS(status)) {
        status = find_bar(raw, translated, base);
    }

    if (NT_SUCCESS(status)) {
        registers_ = static_cast<PUCHAR>(MmMapIoSpace(base,
            static_cast<ULONG>(QLV_CONTROL_BAR_BYTES), MmNonCached));
        status = registers_ != nullptr ? STATUS_SUCCESS : STATUS_INSUFFICIENT_RESOURCES;
    }

    if (NT_SUCCESS(status)) {
        ULONG magic = 0;
        ULONG transport_caps = 0;

        if (!read(QLV_MMIO_REG_MAGIC, magic) || magic != QLV_MAGIC ||
            !read(QLV_MMIO_REG_CAPABILITIES, transport_caps)) {
            status = STATUS_DEVICE_NOT_READY;
        }

        capabilities_.transport_capabilities = transport_caps;
    }

    if (NT_SUCCESS(status)) {
        find_aperture(raw, translated);
        DEVICE_DESCRIPTION description{};
        description.Version = DEVICE_DESCRIPTION_VERSION;
        description.Master = TRUE;
        description.ScatterGather = TRUE;
        description.Dma32BitAddresses = TRUE;
        description.InterfaceType = PCIBus;
        description.BusNumber = raw->List[0].BusNumber;
        description.DmaWidth = Width32Bits;
        description.DmaSpeed = Compatible;
        description.MaximumLength = static_cast<ULONG>(QLV_MAX_PAYLOAD_BYTES + QLV_HEADER_BYTES + dma_bytes);
        ULONG map_registers = 0;
        adapter_ = IoGetDmaAdapter(pdo, &description, &map_registers);

        if (adapter_ == nullptr || adapter_->DmaOperations == nullptr ||
            adapter_->DmaOperations->Size < offsetof(DMA_OPERATIONS, AllocateAdapterChannel) ||
            adapter_->DmaOperations->AllocateCommonBuffer == nullptr ||
            adapter_->DmaOperations->FreeCommonBuffer == nullptr ||
            adapter_->DmaOperations->PutDmaAdapter == nullptr) {
            status = STATUS_NOT_SUPPORTED;
            // An unusable native adapter cannot be released through unknown
            // operations. Retain it and the containing FDO until reboot.
            quarantined_ = adapter_ != nullptr;
        }
    }

    if (NT_SUCCESS(status)) {
        request_ = adapter_->DmaOperations->AllocateCommonBuffer(adapter_, dma_bytes,
            &request_address_, FALSE);
        response_ = adapter_->DmaOperations->AllocateCommonBuffer(adapter_, dma_bytes,
            &response_address_, FALSE);

        if (request_ == nullptr || response_ == nullptr ||
            !dma_span(request_address_) || !dma_span(response_address_)) {
            status = STATUS_INSUFFICIENT_RESOURCES;
        }
    }

    if (NT_SUCCESS(status)) {
        // Optional: without both ring buffers the mailbox remains the path.
        ring_control_ = adapter_->DmaOperations->AllocateCommonBuffer(adapter_,
            static_cast<ULONG>(ring::control_bytes), &ring_control_address_, FALSE);
        ring_slots_ = adapter_->DmaOperations->AllocateCommonBuffer(adapter_,
            static_cast<ULONG>(ring::data_bytes), &ring_slots_address_, FALSE);

        if (ring_control_ != nullptr && !dma_span(ring_control_address_, static_cast<ULONG>(ring::control_bytes))) {
            adapter_->DmaOperations->FreeCommonBuffer(adapter_, static_cast<ULONG>(ring::control_bytes),
                ring_control_address_, ring_control_, FALSE);
            ring_control_ = nullptr;
        }

        if (ring_slots_ != nullptr && !dma_span(ring_slots_address_, static_cast<ULONG>(ring::data_bytes))) {
            adapter_->DmaOperations->FreeCommonBuffer(adapter_, static_cast<ULONG>(ring::data_bytes),
                ring_slots_address_, ring_slots_, FALSE);
            ring_slots_ = nullptr;
        }
    }

    if (NT_SUCCESS(status)) {
        status = resume();
    }

    if (!NT_SUCCESS(status)) {
        return NT_SUCCESS(stop()) ? status : STATUS_DEVICE_BUSY;
    }

    return STATUS_SUCCESS;
}

NTSTATUS NativeTransport::resume() noexcept {
    if (KeGetCurrentIrql() != PASSIVE_LEVEL || quarantined_ || ready_ || in_flight_ ||
        context_ != 0 || request_ == nullptr || response_ == nullptr || registers_ == nullptr) {
        return STATUS_INVALID_DEVICE_STATE;
    }

    // RESET reconnects asynchronously. Wait for a genuinely live connection;
    // IDLE here means usable transport only, not proof of prior cancellation.
    LARGE_INTEGER delay;
    delay.QuadPart = -10000;
    bool available = false;

    for (ULONG iteration = 0; iteration < maximum_poll_iterations; ++iteration) {
        ULONG magic = 0;
        ULONG state = 0;

        if (!read(QLV_MMIO_REG_MAGIC, magic) || !read(QLV_MMIO_REG_STATUS, state)) {
            return STATUS_DEVICE_NOT_CONNECTED;
        }

        if (magic == QLV_MAGIC && (state == QLV_MMIO_IDLE || state == QLV_MMIO_COMPLETE)) {
            available = true;
            break;
        }

        KeDelayExecutionThread(KernelMode, FALSE, &delay);
    }

    if (!available) {
        return STATUS_DEVICE_NOT_READY;
    }

    bus_master_owned_ = true;

    if (!bus_master(true)) {
        return STATUS_DEVICE_NOT_READY;
    }

    Reply reply;
    NTSTATUS status = exchange(static_cast<uint16_t>(QLV_OPEN), 0, reply);

    if (NT_SUCCESS(status)) {
        uint32_t host_flags = 0;
        rx7900::ByteView(static_cast<const uint8_t*>(response_), QLV_HEADER_BYTES).read_u32(QLV_FLAGS_OFFSET, host_flags);
        host_publish_ = (host_flags & QLV_HOST_SYNC_PUBLISH) != 0;
        context_ = reply.context;
        status = exchange(static_cast<uint16_t>(QLV_GET_CAPS), context_, reply);
    }

    if (!NT_SUCCESS(status)) {
        return status;
    }

    capabilities_.size = sizeof(capabilities_);
    capabilities_.version = capabilities_version;
    capabilities_.flags = capabilities_observed;
    ULONG transport_caps = 0;

    if (!read(QLV_MMIO_REG_CAPABILITIES, transport_caps)) {
        return STATUS_DEVICE_NOT_CONNECTED;
    }

    capabilities_.transport_capabilities = transport_caps;
    capabilities_.caps = reply.caps;
    ready_ = true;
    register_ring();
    return STATUS_SUCCESS;
}

NTSTATUS NativeTransport::quiesce(bool allow_mmio) noexcept {
    if (KeGetCurrentIrql() != PASSIVE_LEVEL || quarantined_) {
        return STATUS_DEVICE_BUSY;
    }

    ready_ = false;
    capabilities_ = {};

    if (allow_mmio && context_ != 0 && !in_flight_) {
        Reply reply;
        exchange(static_cast<uint16_t>(QLV_CLOSE), context_, reply);
    }

    // Without MMIO only an absent function proves the host stopped using our pages.
    const bool absent = !allow_mmio && (ever_submitted_ || bus_master_owned_) && function_absent();
    const bool cancelled = !ever_submitted_ || (allow_mmio && reset()) || absent;
    // Isolate the assigned virtual function as additional protection. BME-off
    // alone is insufficient to free old addresses that a later owner can re-enable.
    const bool isolated = !bus_master_owned_ || absent || bus_master(false);

    if (isolated) {
        bus_master_owned_ = false;
    }

    if (!cancelled || !isolated) {
        quarantined_ = true;
        return STATUS_DEVICE_BUSY;
    }

    ever_submitted_ = false;
    return STATUS_SUCCESS;
}

NTSTATUS NativeTransport::suspend() noexcept {
    const NTSTATUS status = quiesce(true);
    dump_timing();
    return status;
}

NTSTATUS NativeTransport::stop(bool allow_mmio) noexcept {
    const NTSTATUS status = quiesce(allow_mmio);
    dump_timing();

    if (NT_SUCCESS(status)) {
        release_resources();
    }

    return status;
}

} // namespace lightning::reactos

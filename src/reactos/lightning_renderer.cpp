/*
Module: lightning.reactos.renderer
Purpose: Execute actual per-process QLV rendering, aperture mappings and private fences.
Created: 2026-10-21
Architecture: Native NT5 APIs and observed host responses only; all calls under FDO guard.
*/
#include <ntifs.h>
#include "lightning_renderer.hpp"
#include "lightning_completion.hpp"
#include "lifetime.hpp"

namespace lightning::reactos {
namespace {

// Zink can keep map-transition allocations live while the previous scene is
// retiring. The host bounds total resources separately from the 1024 mapped
// aperture views. Leave room for all 960 user syncs under its 4096-row bound.
constexpr size_t maximum_blobs = 3072;
constexpr size_t maximum_user_syncs = QLV_MAX_SYNCS - QLV_MAX_TIMELINES;
constexpr size_t maximum_resources = maximum_blobs + maximum_user_syncs;
constexpr ULONG session_tag = 0x73564c51;
// A frame names a few WSI/Venus fences and per-queue timeline syncs; older
// records age out to the conservative stamp. Swapchains keep a few images.
constexpr size_t signal_records = 64;
constexpr size_t pinned_images = 16;

struct NativeMapping final {
    HANDLE section = nullptr;
    PVOID user = nullptr;
    uint64_t offset = 0;
    uint64_t bytes = 0;
    uint32_t cache = 0;
};

// PASSIVE_LEVEL, in the owning process. The caller proves the complete span
// belongs to its live renderer allocation inside the assigned aperture. The
// kernel-only section handle and exact user view remain owned until revocation.
// ReactOS physical-section views have independent VAD cleanup; its user MDL
// unmap path cannot safely retire these multi-page device mappings on NT5.
NTSTATUS map_user(PHYSICAL_ADDRESS physical, MEMORY_CACHING_TYPE cache,
                  NativeMapping& mapping) noexcept {
    UNICODE_STRING name;
    RtlInitUnicodeString(&name, L"\\Device\\PhysicalMemory");
    OBJECT_ATTRIBUTES attributes;
    InitializeObjectAttributes(&attributes, &name, OBJ_KERNEL_HANDLE | OBJ_CASE_INSENSITIVE,
        nullptr, nullptr);
    NTSTATUS status = ZwOpenSection(&mapping.section, SECTION_MAP_READ | SECTION_MAP_WRITE, &attributes);

    if (!NT_SUCCESS(status)) { return status; }
    LARGE_INTEGER offset = physical;
    SIZE_T bytes = static_cast<SIZE_T>(mapping.bytes);
    const ULONG protection = PAGE_READWRITE | (cache == MmCached ? 0 : PAGE_WRITECOMBINE);
    status = ZwMapViewOfSection(mapping.section, NtCurrentProcess(), &mapping.user,
        0, 0, &offset, &bytes, ViewUnmap, 0, protection);

    if (!NT_SUCCESS(status)) { return status; }
    return offset.QuadPart == physical.QuadPart && bytes == mapping.bytes ?
        STATUS_SUCCESS : STATUS_INVALID_VIEW_SIZE;
}

NTSTATUS revoke_mapping(NativeMapping& mapping, PEPROCESS process) noexcept {
    if (mapping.user != nullptr) {
        const bool attached = PsGetCurrentProcess() != process;
        KAPC_STATE attach{};

        if (attached) {
            // NT's documented opaque-process conversion; no field offset or
            // copied EPROCESS layout is used by this native API boundary.
            KeStackAttachProcess(reinterpret_cast<PKPROCESS>(process), &attach);
        }

        const NTSTATUS status = ZwUnmapViewOfSection(NtCurrentProcess(), mapping.user);

        if (attached) {
            KeUnstackDetachProcess(&attach);
        }

        if (!NT_SUCCESS(status)) {
            DbgPrint("LightningNT5: borrowed view retirement failed %08lx address %p\n",
                static_cast<ULONG>(status), mapping.user);
            return status;
        }

        mapping.user = nullptr;
    }

    if (mapping.section != nullptr) {
        const NTSTATUS status = ZwClose(mapping.section);

        if (!NT_SUCCESS(status)) { return status; }
        mapping.section = nullptr;
    }

    mapping = {};
    return STATUS_SUCCESS;
}

bool write_packet(QLV_HEADER h, uint8_t (&packet)[80], uint64_t sync, uint64_t point) noexcept {
    ExchangeReply encoded;
    encoded.header = h;
    encoded.header.payload_bytes = 0;
    size_t written = 0;
    rx7900::sdma::CommandBuffer output(packet, sizeof(packet));

    if (encode_exchange_reply(encoded, output, written) != ExchangeStatus::validated) {
        return false;
    }

    output.write_u32(QLV_PAYLOAD_BYTES_OFFSET, h.payload_bytes);

    if (h.payload_bytes == QLV_SYNC_POINT_BYTES) {
        output.write_u32(64, static_cast<uint32_t>(sync));
        output.write_u32(68, static_cast<uint32_t>(sync >> 32));
        output.write_u32(72, static_cast<uint32_t>(point));
        output.write_u32(76, static_cast<uint32_t>(point >> 32));
    }

    return h.payload_bytes == 0 || h.payload_bytes == QLV_SYNC_POINT_BYTES;
}

QLV_HEADER request_header(uint16_t operation, uint64_t context) noexcept {
    QLV_HEADER h{};
    h.magic = static_cast<uint32_t>(QLV_MAGIC);
    h.version = static_cast<uint16_t>(QLV_VERSION);
    h.operation = operation;
    h.context = context;
    h.status = static_cast<int32_t>(QLV_STATUS_UNSUPPORTED);
    return h;
}

void wire_error(const QLV_HEADER& request, int32_t status, ExchangeReply& reply) noexcept {
    reply = {};
    reply.header = request;
    reply.header.status = status;
    reply.header.payload_bytes = 0;
    reply.header.value = 0;
}

} // namespace

struct NativeRenderer::FileSession final {
    struct Resource final {
        uint64_t token = 0;
        ResourceInfo info{};
        uint64_t last_serial = 0;
        NativeMapping mapping{};
        uint64_t image_lease = 0;
        ImageLayout image_layout{};
    };
    struct Timeline final { uint64_t token = 0; };
    PFILE_OBJECT file = nullptr;
    PEPROCESS process = nullptr;
    HANDLE process_id = nullptr;
    uint64_t context = 0;
    uint64_t retired_context = 0;
    bool exiting = false;
    bool lost = false;
    size_t blob_high_water = 0;
    uint64_t blob_creates = 0;
    uint64_t blob_destroys = 0;
    uint64_t memory_rejects = 0;
    Resource resources[maximum_resources]{};
    Timeline timelines[QLV_MAX_TIMELINES]{};
    CompletionLedger completion{};
    // async-present: latest recorded signal of recently signaled syncs, and
    // images whose newest asynchronous copy may still read them. Both bounded;
    // a missing record keeps the conservative stamp, a full pin table keeps
    // the synchronous copy.
    SignalTable<signal_records> signals{};
    PinTable<pinned_images> pinned{};
};

static_assert(maximum_resources <= 4096,
    "renderer blob and sync capacity must fit the host resource limit");

NativeRenderer::FileSession* NativeRenderer::session(PFILE_OBJECT file) const noexcept {
    if (file == nullptr) {
        return nullptr;
    }

    for (const auto allocation : sessions_) {
        auto* candidate = static_cast<FileSession*>(allocation);

        if (candidate != nullptr && candidate == file->FsContext && candidate->file == file) {
            return candidate;
        }
    }

    return nullptr;
}

bool NativeRenderer::lookup_public(void* owner, uint64_t token, ResourceInfo& output) noexcept {
    auto& file = *static_cast<FileSession*>(owner);

    for (const auto& resource : file.resources) {
        if (resource.token == token && token != 0) {
            output = resource.info;
            output.pending_uses = file.completion.pending(resource.last_serial) ? 1 : 0;

            // A renderer sync is named only as a SUBMIT signal point; Venus
            // command streams cannot reference it. Its recorded latest signal
            // is therefore its only pending use (reset/destroy must not race it).
            const SignalPoint* signal = resource.info.kind == ResourceKind::sync ?
                file.signals.find(token) : nullptr;

            if (signal != nullptr) {
                output.pending_uses = file.completion.completed(signal->ring) < signal->point ? 1 : 0;
            }

            return true;
        }
    }

    return false;
}

bool NativeRenderer::lookup_all(void* owner, uint64_t token, ResourceInfo& output) noexcept {
    if (lookup_public(owner, token, output)) {
        return true;
    }

    const auto& file = *static_cast<FileSession*>(owner);

    for (size_t ring = 0; ring < QLV_MAX_TIMELINES; ++ring) {
        const auto& timeline = file.timelines[ring];

        if (timeline.token == token && token != 0) {
            output = {ResourceKind::sync, 0, 0, 0, file.completion.completed(ring)};
            return true;
        }
    }

    return false;
}

bool NativeRenderer::token_exists(uint64_t token) const noexcept {
    for (const auto allocation : sessions_) {
        auto* file = static_cast<FileSession*>(allocation);
        ResourceInfo ignored;

        if (file != nullptr && (file->context == token || lookup_all(file, token, ignored))) {
            return true;
        }
    }

    return false;
}

NTSTATUS NativeRenderer::resolve_image(uint64_t lease, PEPROCESS process,
                                      NativeImageLease& output, const ProducerPoint* producer) noexcept {
    output = {};

    if (KeGetCurrentIrql() != PASSIVE_LEVEL || !ready() || lease == 0 ||
        process == nullptr || PsGetCurrentProcess() != process) {
        return STATUS_ACCESS_DENIED;
    }

    for (const auto allocation : sessions_) {
        auto* file = static_cast<FileSession*>(allocation);

        if (file == nullptr || file->file == nullptr || file->process != process ||
            file->context == 0 || file->lost || file->exiting) {
            continue;
        }

        for (const auto& resource : file->resources) {
            if (resource.token != 0 && resource.image_lease == lease) {
                // Wait for the named producer submission alone when it is a
                // recorded signal on a host-published private timeline. Later
                // submissions (the next frame) then never delay this present.
                const SignalPoint* signal = producer != nullptr ? file->signals.find(producer->sync) : nullptr;
                const bool targeted = signal != nullptr &&
                    producer_usable(*producer, *signal, resource.last_serial) &&
                    producer_published(*file, *signal);
                const NTSTATUS status = targeted ? wait_signal(*file, *signal) :
                    drain(*file, resource.last_serial);

                if (!NT_SUCCESS(status)) {
                    recover_all();
                    return status;
                }

                output = {file->context, resource.token, resource.image_layout};
                return STATUS_SUCCESS;
            }
        }
    }

    return STATUS_ACCESS_DENIED;
}

bool NativeRenderer::can_track_receipt(uint64_t lease, PEPROCESS process) const noexcept {
    if (!async_receipts() || lease == 0 || process == nullptr) {
        return false;
    }

    for (const auto allocation : sessions_) {
        const auto* file = static_cast<const FileSession*>(allocation);

        if (file == nullptr || file->process != process || file->context == 0 || file->lost) {
            continue;
        }

        for (const auto& resource : file->resources) {
            if (resource.token != 0 && resource.image_lease == lease) {
                return file->pinned.can_pin(resource.token);
            }
        }
    }

    return false;
}

bool NativeRenderer::track_receipt(uint64_t lease, PEPROCESS process, const ImageReceipt& receipt) noexcept {
    if (!async_receipts() || lease == 0 || process == nullptr) {
        return false;
    }

    for (const auto allocation : sessions_) {
        auto* file = static_cast<FileSession*>(allocation);

        if (file == nullptr || file->process != process || file->context == 0 || file->lost) {
            continue;
        }

        for (const auto& resource : file->resources) {
            if (resource.token != 0 && resource.image_lease == lease) {
                return file->pinned.pin(resource.token, receipt);
            }
        }
    }

    return false;
}

ReceiptState NativeRenderer::query_receipt(uint64_t lease, PEPROCESS process) noexcept {
    if (lease == 0 || process == nullptr) {
        return ReceiptState::retired;
    }

    for (const auto allocation : sessions_) {
        auto* file = static_cast<FileSession*>(allocation);

        if (file == nullptr || file->process != process) {
            continue;
        }

        for (const auto& resource : file->resources) {
            if (resource.token == 0 || resource.image_lease != lease) {
                continue;
            }

            const NTSTATUS status = settle_receipts(*file, resource.token, false);

            if (!NT_SUCCESS(status)) {
                return ReceiptState::uncertain;
            }

            return file->pinned.find(resource.token) != nullptr ? ReceiptState::pending : ReceiptState::retired;
        }
    }

    return ReceiptState::retired;
}

void NativeRenderer::retire_receipts_after_display_reset() noexcept {
    for (const auto allocation : sessions_) {
        if (allocation != nullptr) {
            static_cast<FileSession*>(allocation)->pinned.clear();
        }
    }
}

// wait=false observes progress and keeps pending pins; wait=true settles each
// pin through the bounded lower wait. A read that can no longer be proven
// retired contains the renderer: its host image must outlive the copy.
NTSTATUS NativeRenderer::settle_receipts(FileSession& file, uint64_t token, bool wait) noexcept {
    if (file.pinned.empty()) {
        return STATUS_SUCCESS;
    }

    for (size_t index = 0; index < file.pinned.capacity(); ++index) {
        const uint64_t pinned = file.pinned.token(index);
        const ImageReceipt* receipt = file.pinned.find(pinned);

        if (pinned == 0 || receipt == nullptr || (token != 0 && pinned != token)) {
            continue;
        }

        const ReceiptState state = receipt_owner_.resolve != nullptr ?
            receipt_owner_.resolve(receipt_owner_.context, *receipt, wait) : ReceiptState::uncertain;

        if (state == ReceiptState::retired) {
            file.pinned.erase(pinned);
        } else if (state == ReceiptState::uncertain || wait) {
            DbgPrint("LightningNT5: image read receipt unprovable epoch %lu sequence %lu\n",
                static_cast<ULONG>(receipt->epoch), static_cast<ULONG>(receipt->sequence));
            quarantined_ = true;
            active_ = false;
            return STATUS_DEVICE_BUSY;
        }
    }

    return STATUS_SUCCESS;
}

NTSTATUS NativeRenderer::settle_all_receipts() noexcept {
    for (const auto allocation : sessions_) {
        if (allocation == nullptr) {
            continue;
        }

        const NTSTATUS status = settle_receipts(*static_cast<FileSession*>(allocation), 0, true);

        if (!NT_SUCCESS(status)) {
            return status;
        }
    }

    return STATUS_SUCCESS;
}

NTSTATUS NativeRenderer::initialize(NativeTransport& transport) noexcept {
    if (quarantined_ || !transport.aperture(aperture_base_, aperture_bytes_)) {
        return STATUS_DEVICE_NOT_READY;
    }

    const NTSTATUS status = transport.claim_renderer();

    if (!NT_SUCCESS(status)) {
        return status;
    }

#ifdef RX7900_LIGHTNING_POSTED_SUBMIT
    // Opt-in: SUBMITs return once queued on the shared ring (the transport posts
    // only when QEMU negotiated it). A later host failure poisons the context and
    // surfaces as DEVICE_LOST at the next drain, which recovers every context.
    transport.post_submits(true);
#endif
    transport_ = &transport;
    active_ = true;
    return STATUS_SUCCESS;
}

NTSTATUS NativeRenderer::open_file(PFILE_OBJECT file) noexcept {
    static_assert(sizeof(FileSession) <= 512 * 1024,
        "a renderer session must fit a bounded nonpaged-pool allocation");
    if (KeGetCurrentIrql() != PASSIVE_LEVEL || !ready() || file == nullptr || file->FsContext != nullptr) {
        return STATUS_DEVICE_NOT_READY;
    }

    for (auto& allocation : sessions_) {
        if (allocation != nullptr) {
            continue;
        }

        allocation = ExAllocatePoolWithTag(NonPagedPool, sizeof(FileSession), session_tag);

        if (allocation == nullptr) {
            return STATUS_INSUFFICIENT_RESOURCES;
        }

        auto* created = rx7900::reactos::construct_device_state<FileSession>(allocation);
        created->file = file;
        created->process = PsGetCurrentProcess();
        created->process_id = PsGetCurrentProcessId();
        ObReferenceObject(created->process);
        file->FsContext = created;
        return STATUS_SUCCESS;
    }

    return STATUS_INSUFFICIENT_RESOURCES;
}

NTSTATUS NativeRenderer::call(FileSession& file, QLV_HEADER header, ExchangeReply& output,
                              uint64_t sync, uint64_t point) noexcept {
    uint8_t bytes[80]{};

    if (!write_packet(header, bytes, sync, point)) {
        return STATUS_INVALID_PARAMETER;
    }

    return transport_->renderer_exchange({bytes, static_cast<size_t>(QLV_HEADER_BYTES + header.payload_bytes)},
        header.operation == QLV_GET_CAPS ? response_capacity : QLV_HEADER_BYTES,
        {file.context, aperture_bytes_, &file, lookup_all}, output);
}

NTSTATUS NativeRenderer::poll_ring(FileSession& file, size_t ring) noexcept {
    const auto& timeline = file.timelines[ring];

    if (timeline.token == 0 || file.completion.completed(ring) == file.completion.issued(ring)) {
        return STATUS_SUCCESS;
    }

    auto header = request_header(static_cast<uint16_t>(QLV_READ_SYNC), file.context);
    header.resource = timeline.token;
    ExchangeReply reply;
    const NTSTATUS status = call(file, header, reply);

    if (!NT_SUCCESS(status) || reply.header.status != QLV_STATUS_SUCCESS ||
        !file.completion.observe(ring, reply.header.value)) {
        DbgPrint("LightningNT5: renderer fence poll ring %lu failed %08lx host %ld value %I64u issued %I64u\n",
            static_cast<ULONG>(ring), static_cast<ULONG>(status), static_cast<LONG>(reply.header.status),
            reply.header.value, file.completion.issued(ring));
        return NT_SUCCESS(status) ? STATUS_DATA_ERROR : status;
    }

    return STATUS_SUCCESS;
}

NTSTATUS NativeRenderer::poll(FileSession& file) noexcept {
    for (size_t ring = 0; ring < QLV_MAX_TIMELINES; ++ring) {
        const NTSTATUS status = poll_ring(file, ring);

        if (!NT_SUCCESS(status)) {
            return status;
        }
    }

    return STATUS_SUCCESS;
}

bool NativeRenderer::producer_published(const FileSession& file, const SignalPoint& signal) const noexcept {
    return signal.ring < QLV_MAX_TIMELINES && signal.point <= file.completion.issued(signal.ring) &&
        file.timelines[signal.ring].token != 0 &&
        transport_->published(file.context, file.timelines[signal.ring].token);
}

// Same bounds as drain, but only the signal's own private ring point must be
// observed. Other rings and later points on this ring may still be pending;
// READ_SYNC is answered from the published timeline page below the exchange.
NTSTATUS NativeRenderer::wait_signal(FileSession& file, const SignalPoint& recorded) noexcept {
    const SignalPoint signal = recorded;
    const size_t ring = signal.ring;
    const ULONGLONG began = KeQueryInterruptTime();
    LARGE_INTEGER pause;
    pause.QuadPart = -10000;

    if (ring >= QLV_MAX_TIMELINES || signal.point > file.completion.issued(ring)) {
        return STATUS_DATA_ERROR;
    }

    while (file.completion.completed(ring) < signal.point) {
        const NTSTATUS status = poll_ring(file, ring);

        if (!NT_SUCCESS(status)) {
            return status;
        }

        if (file.completion.completed(ring) >= signal.point) {
            break;
        }

        if (KeQueryInterruptTime() - began >= QLV_KERNEL_EXCHANGE_TIMEOUT_100NS) {
            return STATUS_IO_TIMEOUT;
        }

        if (transport_->await_published(file.context, file.timelines[ring].token,
            file.completion.completed(ring))) {
            continue;
        }

        KeDelayExecutionThread(KernelMode, FALSE, &pause);
    }

    return STATUS_SUCCESS;
}

// Called after a committed SUBMIT. The validated payload starts with exactly
// header.offset caller QLV_SYNC_POINT records; the private fence is not among
// them. Only syncs of this session are recorded.
void NativeRenderer::record_signals(FileSession& file, rx7900::ByteView input, uint64_t serial,
                                    size_t ring, uint64_t point) noexcept {
    uint64_t count = 0;

    if (!input.read_u64(QLV_OFFSET_OFFSET, count) || count > QLV_MAX_SUBMIT_SYNCS) {
        return;
    }

    for (uint64_t index = 0; index < count; ++index) {
        const size_t offset = static_cast<size_t>(QLV_HEADER_BYTES + index * QLV_SYNC_POINT_BYTES);
        uint64_t token = 0;
        uint64_t value = 0;

        if (!input.read_u64(offset, token) || !input.read_u64(offset + QLV_SYNC_VALUE_OFFSET, value)) {
            return;
        }

        for (const auto& resource : file.resources) {
            if (resource.token == token && token != 0 && resource.info.kind == ResourceKind::sync) {
                file.signals.record(token, {serial, point, value, static_cast<uint32_t>(ring)});
                break;
            }
        }
    }
}

NTSTATUS NativeRenderer::drain(FileSession& file, uint64_t serial) noexcept {
    const ULONGLONG began = KeQueryInterruptTime();
    LARGE_INTEGER pause;
    pause.QuadPart = -10000;

    while (file.completion.pending(serial)) {
        const NTSTATUS status = poll(file);

        if (!NT_SUCCESS(status)) {
            return status;
        }

        if (!file.completion.pending(serial)) {
            break;
        }

        if (file.completion.count() == 0 || KeQueryInterruptTime() - began >= QLV_KERNEL_EXCHANGE_TIMEOUT_100NS) {
            return STATUS_IO_TIMEOUT;
        }

        // A published private timeline advances in guest RAM without an
        // exchange; spin on it briefly before sleeping. poll still observes
        // the value through the transport before anything is retired.
        bool progressed = false;

        for (size_t ring = 0; ring < QLV_MAX_TIMELINES; ++ring) {
            const auto& timeline = file.timelines[ring];

            if (timeline.token != 0 && file.completion.completed(ring) != file.completion.issued(ring)) {
                progressed = transport_->await_published(file.context, timeline.token,
                    file.completion.completed(ring));
                break;
            }
        }

        if (progressed) {
            continue;
        }

        KeDelayExecutionThread(KernelMode, FALSE, &pause);
    }

    return STATUS_SUCCESS;
}

NTSTATUS NativeRenderer::revoke_blob(FileSession& file, uint64_t token, bool in_exit) noexcept {
    for (auto& resource : file.resources) {
        if (resource.token != token || token == 0) {
            continue;
        }

        if (file.exiting && !in_exit && resource.mapping.user != nullptr) {
            return STATUS_DEVICE_BUSY;
        }

        return revoke_mapping(resource.mapping, file.process);
    }

    return STATUS_INVALID_PARAMETER;
}

NTSTATUS NativeRenderer::revoke_views(FileSession& file, bool in_exit) noexcept {
    NTSTATUS result = STATUS_SUCCESS;

    for (auto& resource : file.resources) {
        if (resource.mapping.section != nullptr || resource.mapping.user != nullptr) {
            const NTSTATUS status = revoke_blob(file, resource.token, in_exit);

            if (!NT_SUCCESS(status) && NT_SUCCESS(result)) {
                result = status;
            }
        }
    }

    return result;
}

void NativeRenderer::invalidate_session(FileSession& file, bool retain_views) noexcept {
    if (file.context != 0) {
        file.retired_context = file.context;
    }

    file.context = 0;
    file.lost = true;

    for (auto& resource : file.resources) {
        if (retain_views && (resource.mapping.user != nullptr || resource.mapping.section != nullptr)) {
            resource.info = {};
            resource.last_serial = 0;
            resource.image_lease = 0;
            resource.image_layout = {};
        } else {
            resource = {};
        }
    }

    for (auto& timeline : file.timelines) {
        timeline = {};
    }

    file.completion = {};
    // Callers settled every pinned read before host invalidation.
    file.signals.clear();
    file.pinned.clear();
}

bool NativeRenderer::has_views(const FileSession& file) noexcept {
    for (const auto& resource : file.resources) {
        if (resource.mapping.user != nullptr || resource.mapping.section != nullptr) { return true; }
    }
    return false;
}

bool NativeRenderer::unrevocable_borrow(const FileSession& file) noexcept {
    return !file.exiting && has_views(file);
}

BorrowCensus NativeRenderer::borrow_census() const noexcept {
    BorrowCensus census{};

    for (const auto allocation : sessions_) {
        if (allocation == nullptr) { continue; }
        const auto& file = *static_cast<const FileSession*>(allocation);

        if (!unrevocable_borrow(file)) { continue; }
        ++census.sessions;

        for (const auto& resource : file.resources) {
            census.views += resource.mapping.user != nullptr || resource.mapping.section != nullptr ? 1U : 0U;
        }
    }

    return census;
}

void NativeRenderer::finish_recovery() noexcept {
    if (!recovery_waiting_views_ || quarantined_) { return; }

    for (const auto allocation : sessions_) {
        if (allocation == nullptr) { continue; }
        const auto& file = *static_cast<FileSession*>(allocation);

        if (has_views(file)) { return; }
    }
    recovery_waiting_views_ = false;
    active_ = transport_ != nullptr && transport_->ready();
}

NTSTATUS NativeRenderer::recover_all() noexcept {
    active_ = false;

    // RESET revokes every host image import. A pinned asynchronous read must
    // retire first, or its canvas copy would find no image; unprovable is
    // containment, exactly as an uncertain synchronous copy always was.
    if (!NT_SUCCESS(settle_all_receipts())) {
        return STATUS_DEVICE_BUSY;
    }

    const NTSTATUS status = transport_->recover_renderer();

    if (transport_->quarantined()) {
        quarantined_ = true;
        return STATUS_DEVICE_BUSY;
    }

    // RESET cancels GPU ownership, not the caller's borrowed CPU pointers.
    // Keep their VADs until explicit close/process exit. Block a replacement
    // session from publishing aperture aliases until all old views retire.
    for (const auto allocation : sessions_) {
        if (allocation != nullptr) {
            invalidate_session(*static_cast<FileSession*>(allocation), true);
        }
    }
    recovery_waiting_views_ = NT_SUCCESS(status);
    finish_recovery();
    return status;
}

NTSTATUS NativeRenderer::close_session(FileSession& file) noexcept {
    NTSTATUS status = settle_receipts(file, 0, true);

    if (!NT_SUCCESS(status)) {
        return status;
    }

    status = revoke_views(file, file.exiting);

    if (!NT_SUCCESS(status)) {
        quarantined_ = true;
        active_ = false;
        return status;
    }

    if (file.context != 0) {
        ExchangeReply reply;
        status = call(file, request_header(static_cast<uint16_t>(QLV_CLOSE), file.context), reply);

        if (!NT_SUCCESS(status) || reply.header.status != QLV_STATUS_SUCCESS) {
            return recover_all();
        }

        invalidate_session(file);
    }
    finish_recovery();
    return STATUS_SUCCESS;
}

void NativeRenderer::release_session(size_t slot) noexcept {
    auto* file = static_cast<FileSession*>(sessions_[slot]);
    ObDereferenceObject(file->process);
    ExFreePoolWithTag(sessions_[slot], session_tag);
    sessions_[slot] = nullptr;
}

NTSTATUS NativeRenderer::close_file(PFILE_OBJECT object) noexcept {
    auto* file = session(object);

    if (file == nullptr) {
        return STATUS_SUCCESS;
    }

    if (quarantined_) {
        file->file = nullptr;
        object->FsContext = nullptr;
        return STATUS_DEVICE_BUSY;
    }

    const NTSTATUS status = close_session(*file);

    if (quarantined_ || (!NT_SUCCESS(status) && file->context != 0)) {
        return status;
    }

    for (size_t slot = 0; slot < QLV_KERNEL_MAX_CLIENTS; ++slot) {
        if (sessions_[slot] == file) {
            release_session(slot);
            object->FsContext = nullptr;
            return STATUS_SUCCESS;
        }
    }

    return STATUS_INVALID_PARAMETER;
}

void NativeRenderer::process_exit(HANDLE process_id) noexcept {
    for (const auto allocation : sessions_) {
        auto* file = static_cast<FileSession*>(allocation);

        if (file == nullptr || file->process_id != process_id || file->exiting) {
            continue;
        }

        file->exiting = true;
        // The notification runs before MmCleanProcessAddressSpace. Revocation
        // must finish here even if another process holds a duplicate file handle.
        if (!NT_SUCCESS(revoke_views(*file, true))) {
            quarantined_ = true;
            active_ = false;
            continue;
        }

        if (!quarantined_) {
            close_session(*file);
        }
    }
}

NTSTATUS NativeRenderer::shutdown(bool allow_host) noexcept {
    active_ = false;
    NTSTATUS result = quarantined_ ? STATUS_DEVICE_BUSY : STATUS_SUCCESS;

    for (const auto allocation : sessions_) {
        if (allocation != nullptr) {
            auto& file = *static_cast<FileSession*>(allocation);
            // STOP, power loss and quarantine do not end the caller's CPU
            // borrow. Compiler/submit threads can still read these ring views.
            // Retain the VAD, section, host allocation and device ownership;
            // only explicit file/resource retirement or process exit may unmap.
            if (unrevocable_borrow(file)) {
                quarantined_ = true;
                result = STATUS_DEVICE_BUSY;
                continue;
            }
            const bool can_call_host = allow_host && !quarantined_;
            const NTSTATUS status = can_call_host ? close_session(file) : revoke_views(file, file.exiting);

            if (!NT_SUCCESS(status)) {
                quarantined_ = true;

                if (NT_SUCCESS(result)) {
                    result = status;
                }
            }

            if (!can_call_host && file.context != 0) {
                quarantined_ = true;

                if (NT_SUCCESS(result)) {
                    result = STATUS_DEVICE_BUSY;
                }
            }
        }
    }

    active_ = false;
    return result;
}

NTSTATUS NativeRenderer::map_blob(FileSession& file, uint64_t token, ExchangeReply& reply) noexcept {
    FileSession::Resource* owner = nullptr;
    size_t total_maps = 0;
    const ApertureMapping candidate{file.context, token, reply.header.offset, reply.header.bytes, reply.header.flags};

    for (const auto allocation : sessions_) {
        auto* current = static_cast<FileSession*>(allocation);

        if (current == nullptr) {
            continue;
        }

        for (auto& resource : current->resources) {
            if (current == &file && resource.token == token) {
                owner = &resource;
            }

            if (resource.mapping.user == nullptr) {
                continue;
            }

            ++total_maps;
            const auto& mapping = resource.mapping;
            const ApertureMapping existing{current->context, resource.token, mapping.offset, mapping.bytes, mapping.cache};
            const auto relation = mapping_relation(candidate, existing, aperture_bytes_);

            if (relation == MappingRelation::conflict || relation == MappingRelation::invalid) {
                return STATUS_CONFLICTING_ADDRESSES;
            }
        }
    }

    if (owner == nullptr) {
        return STATUS_DATA_ERROR;
    }

    if (owner->mapping.user != nullptr) {
        reply.header.value = reinterpret_cast<ULONG_PTR>(owner->mapping.user);
        return STATUS_SUCCESS;
    }

    const uint64_t bytes = reply.header.bytes;

    if (total_maps >= QLV_KERNEL_MAX_MAPPINGS ||
        bytes > MAXULONG || reply.header.offset > aperture_bytes_ || bytes > aperture_bytes_ - reply.header.offset ||
        static_cast<uint64_t>(aperture_base_.QuadPart) > MAXLONGLONG - reply.header.offset) {
        DbgPrint("LightningNT5: renderer map rejected by bounds maps %lu bytes %I64u offset %I64u aperture %I64u\n",
            static_cast<ULONG>(total_maps), bytes, reply.header.offset, aperture_bytes_);
        return STATUS_INSUFFICIENT_RESOURCES;
    }

    NativeMapping created;
    created.bytes = bytes;
    created.offset = reply.header.offset;
    created.cache = reply.header.flags;
    PHYSICAL_ADDRESS physical = aperture_base_;
    physical.QuadPart += static_cast<LONGLONG>(created.offset);
    const auto cache = created.cache == QLV_MAP_CACHE_CACHED ? MmCached : MmWriteCombined;
    NTSTATUS status = map_user(physical, cache, created);

    if (NT_SUCCESS(status) && !valid_user_mapping(reinterpret_cast<ULONG_PTR>(created.user), bytes,
        reinterpret_cast<ULONG_PTR>(MmHighestUserAddress), sizeof(ULONG_PTR) * 8)) {
        status = STATUS_INVALID_ADDRESS;
    }

    if (!NT_SUCCESS(status)) {
        DbgPrint("LightningNT5: renderer user map failed %08lx maps %lu bytes %I64u offset %I64u\n",
            static_cast<ULONG>(status), static_cast<ULONG>(total_maps), bytes, reply.header.offset);
        const NTSTATUS revoked = revoke_mapping(created, file.process);

        if (!NT_SUCCESS(revoked)) {
            owner->mapping = created;
            quarantined_ = true;
            active_ = false;
            return revoked;
        }

        return status;
    }

    owner->mapping = created;
    reply.header.value = reinterpret_cast<ULONG_PTR>(created.user);
    return STATUS_SUCCESS;
}

NTSTATUS NativeRenderer::exchange_file(PFILE_OBJECT object, PEPROCESS requestor,
    rx7900::ByteView input, rx7900::sdma::CommandBuffer output, size_t& written) noexcept {
    written = 0;
    auto* file = session(object);

    if (file == nullptr || quarantined_ || file->exiting || requestor != file->process ||
        PsGetCurrentProcess() != file->process || input.size() > QLV_HEADER_BYTES + QLV_MAX_PAYLOAD_BYTES) {
        return STATUS_ACCESS_DENIED;
    }

    uint16_t operation = 0;
    input.read_u16(QLV_OPERATION_OFFSET, operation);
    ExchangeReply reply;

    if (operation == QLV_IMPORT_BLOB || operation == QLV_WATCH_SYNCS) {
        if (unsupported_exchange_reply(input, file->context, reply) != ExchangeStatus::validated) {
            return STATUS_INVALID_PARAMETER;
        }

        return encode_exchange_reply(reply, output, written) == ExchangeStatus::validated ? STATUS_SUCCESS : STATUS_BUFFER_TOO_SMALL;
    }

    const uint64_t policy_context = file->lost && operation == QLV_CLOSE ? file->retired_context : file->context;
    const size_t capacity = output.size() < response_capacity ? output.size() : response_capacity;
    ValidatedExchange validated;
    auto policy = validate_exchange_request(input, capacity,
        {policy_context, aperture_bytes_, file, lookup_public}, validated);

    // Destructive operations wait for private real fences, then revalidate the
    // newly observed lease state. A queue-acceptance response never clears it.
    if (policy == ExchangeStatus::resource_busy) {
        // A busy sync (fence reset or destroy) waits for its own recorded
        // signal when that ring is host-published, never for later work.
        uint64_t busy_token = 0;
        input.read_u64(QLV_RESOURCE_OFFSET, busy_token);
        const SignalPoint* signal = (operation == QLV_WRITE_SYNC || operation == QLV_DESTROY_SYNC) ?
            file->signals.find(busy_token) : nullptr;
        const bool targeted = signal != nullptr && producer_published(*file, *signal);
        transport_->note_submission_wait(false);
        const NTSTATUS drained = targeted ? wait_signal(*file, *signal) :
            drain(*file, file->completion.latest());

        if (!NT_SUCCESS(drained)) {
            recover_all();
            return drained;
        }

        policy = validate_exchange_request(input, capacity, {policy_context, aperture_bytes_, file, lookup_public}, validated);
    }

    if (policy != ExchangeStatus::validated) {
        uint32_t ring = 0;
        uint32_t sync_count = 0;
        uint32_t command_bytes = 0;
        input.read_u32(QLV_VALUE_OFFSET, ring);
        input.read_u32(QLV_OFFSET_OFFSET, sync_count);
        input.read_u32(QLV_BYTES_OFFSET, command_bytes);
        DbgPrint("LightningNT5: reject operation %u policy %u ring %lu syncs %lu commands %lu packet %lu\n",
            static_cast<unsigned>(operation), static_cast<unsigned>(policy), static_cast<ULONG>(ring),
            static_cast<ULONG>(sync_count), static_cast<ULONG>(command_bytes), static_cast<ULONG>(input.size()));
        return STATUS_INVALID_PARAMETER;
    }

    const auto header = validated.header();

    if (file->lost) {
        if (operation != QLV_CLOSE) {
            wire_error(header, QLV_STATUS_DEVICE_LOST, reply);
        } else {
            // Only a confirmed CLOSE/RESET invalidation creates this state.
            wire_error(header, QLV_STATUS_SUCCESS, reply);
        }

        return encode_exchange_reply(reply, output, written) == ExchangeStatus::validated ? STATUS_SUCCESS : STATUS_BUFFER_TOO_SMALL;
    }

    if (!ready()) {
        return STATUS_DEVICE_NOT_READY;
    }

    FileSession::Resource* resource = nullptr;
    FileSession::Resource* available = nullptr;
    size_t blobs = 0;
    size_t syncs = 0;

    for (auto& row : file->resources) {
        if (row.token == header.resource && row.token != 0) {
            resource = &row;
        }

        if (row.token == 0 && available == nullptr) {
            available = &row;
        } else if (row.token != 0) {
            row.info.kind == ResourceKind::blob ? ++blobs : ++syncs;
        }
    }

    if ((operation == QLV_CREATE_BLOB || operation == QLV_CREATE_SYNC) &&
        (available == nullptr || (operation == QLV_CREATE_BLOB ? blobs >= maximum_blobs : syncs >= maximum_user_syncs))) {
        DbgPrint("LightningNT5: renderer resource cap operation %u blobs %lu/%lu syncs %lu/%lu created %I64u destroyed %I64u\n",
            static_cast<unsigned>(operation), static_cast<ULONG>(blobs), static_cast<ULONG>(maximum_blobs),
            static_cast<ULONG>(syncs), static_cast<ULONG>(maximum_user_syncs),
            file->blob_creates, file->blob_destroys);
        wire_error(header, QLV_STATUS_NO_MEMORY, reply);
        return encode_exchange_reply(reply, output, written) == ExchangeStatus::validated ? STATUS_SUCCESS : STATUS_BUFFER_TOO_SMALL;
    }

    if (operation == QLV_REGISTER_SCANOUT && next_image_lease_ == UINT64_MAX) {
        wire_error(header, QLV_STATUS_NO_MEMORY, reply);
        return encode_exchange_reply(reply, output, written) == ExchangeStatus::validated ? STATUS_SUCCESS : STATUS_BUFFER_TOO_SMALL;
    }

    // Only DESTROY waits for the resource's private fences. MAP and UNMAP never
    // change GPU-visible memory, so they must not stall on all queued work:
    // - MAP publishes a new CPU view of an allocation that stays live; Vulkan
    //   leaves CPU/GPU ordering on mapped memory to the caller's own fences.
    // - UNMAP first revokes this process view below (ZwUnmapViewOfSection),
    //   so no CPU access can follow, and pending write-combined stores are
    //   retired by that syscall and the serializing MMIO doorbell. The host
    //   then only removes its aperture subregion (venus_revoke) and drops its
    //   publication (aperture revoked); the blob and its device memory stay
    //   owned by the host renderer until DESTROY_BLOB, so queued GPU reads of
    //   unmapped-but-live memory are unaffected.
    // - DESTROY frees that allocation, so it still drains here and the policy
    //   reports it busy while any submitted use may remain.
    if (operation == QLV_DESTROY_BLOB && resource != nullptr) {
        const NTSTATUS drained = drain(*file, resource->last_serial);

        if (!NT_SUCCESS(drained)) {
            recover_all();
            return drained;
        }
    }

    // async-present: an image whose asynchronous copy may still read it is
    // pinned until that read retires. Destroying, re-registering or closing
    // first waits for the lower canvas; an unprovable read is containment.
    if (operation == QLV_DESTROY_BLOB || operation == QLV_REGISTER_SCANOUT || operation == QLV_CLOSE) {
        const NTSTATUS settled = settle_receipts(*file, operation == QLV_CLOSE ? 0 : header.resource, true);

        if (!NT_SUCCESS(settled)) {
            return settled;
        }
    }

    if (operation == QLV_CLOSE || operation == QLV_UNMAP_BLOB || operation == QLV_DESTROY_BLOB) {
        const NTSTATUS revoked = operation == QLV_CLOSE ? revoke_views(*file) : revoke_blob(*file, header.resource);

        if (!NT_SUCCESS(revoked)) {
            quarantined_ = true;
            active_ = false;
            return revoked;
        }
    }

    if (operation == QLV_REGISTER_SCANOUT && resource != nullptr) {
        // Registration can replace a host import before a later host error.
        // The previous lease must never remain an alternate view of that
        // mutable registration. Publish a fresh identity only after success.
        resource->image_lease = 0;
        resource->image_layout = {};
    }

    auto* timeline = operation == QLV_SUBMIT ? &file->timelines[static_cast<size_t>(header.value)] : nullptr;

    uint64_t submission_serial = 0;
    uint64_t submission_point = 0;

    if (timeline != nullptr) {
        if (file->completion.count() == CompletionLedger::capacity) {
            // A full bounded ledger is queue backpressure, not an allocation
            // failure. Wait only for its oldest retained real fence; waiting
            // for every later queue would unnecessarily serialize independent
            // work. drain rechecks authoritative host counters before reuse.
            transport_->note_submission_wait(true);
            const NTSTATUS status = drain(*file, file->completion.retired() + 1);

            if (!NT_SUCCESS(status)) {
                recover_all();
                return status;
            }
        }

        // Acceptance does not require an eager READ_SYNC roundtrip. Keep every
        // use conservatively pending until capacity pressure or an existing
        // image/destructive/teardown drain observes its actual private fence.

        if (!file->completion.prepare(static_cast<size_t>(header.value), submission_serial, submission_point)) {
            DbgPrint("LightningNT5: submit ledger prepare failed pending %lu payload %lu\n",
                static_cast<ULONG>(file->completion.count()), static_cast<ULONG>(header.payload_bytes));
            wire_error(header, QLV_STATUS_NO_MEMORY, reply);
            return encode_exchange_reply(reply, output, written) == ExchangeStatus::validated ? STATUS_SUCCESS : STATUS_BUFFER_TOO_SMALL;
        }

        if (timeline->token == 0) {
            ExchangeReply created;
            const NTSTATUS status = call(*file, request_header(static_cast<uint16_t>(QLV_CREATE_SYNC), file->context), created);

            if (!NT_SUCCESS(status) || created.header.status != QLV_STATUS_SUCCESS) {
                DbgPrint("LightningNT5: submit timeline creation failed nt %08lx wire %ld pending %lu\n",
                    static_cast<ULONG>(status), static_cast<LONG>(created.header.status),
                    static_cast<ULONG>(file->completion.count()));
                if (!NT_SUCCESS(status)) {
                    recover_all();
                    return status;
                }

                wire_error(header, created.header.status, reply);
                return encode_exchange_reply(reply, output, written) == ExchangeStatus::validated ? STATUS_SUCCESS : STATUS_BUFFER_TOO_SMALL;
            }

            if (token_exists(created.header.resource)) {
                recover_all();
                return STATUS_DATA_ERROR;
            }

            timeline->token = created.header.resource;
        }
    }

    const NativeTransport::PrivateSubmitFence private_fence = timeline != nullptr ?
        NativeTransport::PrivateSubmitFence{timeline->token, submission_point} :
        NativeTransport::PrivateSubmitFence{};
    bool fused_submission = false;
    NTSTATUS status = transport_->renderer_exchange(input, capacity,
        {file->context, aperture_bytes_, file, lookup_all}, reply,
        timeline != nullptr ? &private_fence : nullptr, &fused_submission);

    if (!NT_SUCCESS(status)) {
        if (status == STATUS_INSUFFICIENT_RESOURCES) {
            if (operation == QLV_SUBMIT) {
                DbgPrint("LightningNT5: submit transport memory failed payload %lu pending %lu\n",
                    static_cast<ULONG>(header.payload_bytes),
                    static_cast<ULONG>(file->completion.count()));
            }
            wire_error(header, QLV_STATUS_NO_MEMORY, reply);
        } else {
            recover_all();
            return status;
        }
    } else if (reply.header.status == QLV_STATUS_SUCCESS) {
        if (operation == QLV_OPEN) {
            if (token_exists(reply.header.context)) {
                recover_all();
                return STATUS_DATA_ERROR;
            }

            file->context = reply.header.context;
            // Publish the validated, assigned aperture capacity rather than
            // the host's raw claim. Older clients may ignore these fields.
            reply.header.flags = (reply.header.flags & QLV_HOST_WATCHES) |
                QLV_HOST_APERTURE_SIZE;
            reply.header.bytes = aperture_bytes_;
        } else if (operation == QLV_CLOSE) {
            invalidate_session(*file);
        } else if (operation == QLV_CREATE_BLOB || operation == QLV_CREATE_SYNC) {
            if (available == nullptr || token_exists(reply.header.resource)) {
                recover_all();
                return STATUS_DATA_ERROR;
            }

            available->token = reply.header.resource;
            available->info.kind = operation == QLV_CREATE_BLOB ? ResourceKind::blob : ResourceKind::sync;
            available->info.byte_count = operation == QLV_CREATE_BLOB ? header.bytes : 0;
            available->info.blob_flags = operation == QLV_CREATE_BLOB ? header.flags : 0;
            available->info.observed_sync_value = operation == QLV_CREATE_SYNC ? header.value : 0;
            if (operation == QLV_CREATE_BLOB && file->blob_creates != UINT64_MAX) {
                ++file->blob_creates;
            }
            if (operation == QLV_CREATE_BLOB && blobs + 1 > file->blob_high_water) {
                file->blob_high_water = blobs + 1;
                if (file->blob_high_water % 128 == 0) {
                    DbgPrint("LightningNT5: renderer live blob high-water %lu/%lu\n",
                        static_cast<ULONG>(file->blob_high_water), static_cast<ULONG>(maximum_blobs));
                }
            }
        } else if (operation == QLV_REGISTER_SCANOUT) {
            ImageLayout layout;

            if (resource == nullptr || !image_layout(header, resource->info.byte_count, layout)) {
                recover_all();
                return STATUS_DATA_ERROR;
            }

            resource->image_lease = next_image_lease_++;
            resource->image_layout = layout;
            reply.header.value = resource->image_lease;
        } else if (operation == QLV_MAP_BLOB) {
            status = map_blob(*file, header.resource, reply);

            if (!NT_SUCCESS(status)) {
                if (quarantined_) {
                    return status;
                }

                auto undo = request_header(static_cast<uint16_t>(QLV_UNMAP_BLOB), file->context);
                undo.resource = header.resource;
                ExchangeReply rollback;
                const NTSTATUS undone = call(*file, undo, rollback);

                if (!NT_SUCCESS(undone) || rollback.header.status != QLV_STATUS_SUCCESS) {
                    recover_all();
                    return NT_SUCCESS(undone) ? STATUS_DATA_ERROR : undone;
                }

                wire_error(header, QLV_STATUS_NO_MEMORY, reply);
            }
        } else if ((operation == QLV_DESTROY_BLOB || operation == QLV_DESTROY_SYNC) && resource != nullptr) {
            if (operation == QLV_DESTROY_BLOB && file->blob_destroys != UINT64_MAX) {
                ++file->blob_destroys;
            }
            file->signals.erase(resource->token);
            file->pinned.erase(resource->token);
            *resource = {};
        } else if ((operation == QLV_READ_SYNC || operation == QLV_WRITE_SYNC) && resource != nullptr) {
            resource->info.observed_sync_value = operation == QLV_READ_SYNC ? reply.header.value : header.value;
        } else if (timeline != nullptr) {
            // Queue acknowledgement is followed by a separate sync-only packet
            // so all64 caller sync points remain available. One timeline per
            // ring prevents out-of-order queues from retiring one another.
            // Venus command streams are opaque here, so every live row is
            // stamped: any of them may be referenced. Only destructive
            // operations, REGISTER_SCANOUT and image resolution consult it;
            // MAP and UNMAP no longer wait for it.
            for (auto& row : file->resources) {
                if (row.token != 0) {
                    row.last_serial = submission_serial;
                }
            }

            if (!fused_submission) {
                auto fence = request_header(static_cast<uint16_t>(QLV_SUBMIT), file->context);
                fence.value = header.value;
                fence.offset = 1;
                fence.payload_bytes = QLV_SYNC_POINT_BYTES;
                ExchangeReply completion;
                status = call(*file, fence, completion, timeline->token, submission_point);

                if (!NT_SUCCESS(status) || completion.header.status != QLV_STATUS_SUCCESS) {
                    recover_all();
                    return NT_SUCCESS(status) ? STATUS_DATA_ERROR : status;
                }
            }

            if (!file->completion.commit(static_cast<size_t>(header.value), submission_serial, submission_point)) {
                recover_all();
                return STATUS_DATA_ERROR;
            }

            // The caller syncs this batch signals complete with its private
            // point; a present or fence reset can then wait for it alone.
            record_signals(*file, input, submission_serial, static_cast<size_t>(header.value), submission_point);
        }
    } else if (reply.header.status == QLV_STATUS_NO_MEMORY &&
               (operation == QLV_CREATE_BLOB || operation == QLV_MAP_BLOB)) {
        uint64_t mapped_bytes = 0;
        size_t mapped_views = 0;
        for (const auto allocation : sessions_) {
            const auto* current = static_cast<const FileSession*>(allocation);
            if (current == nullptr) { continue; }
            for (const auto& row : current->resources) {
                if (row.mapping.user == nullptr) { continue; }
                ++mapped_views;
                mapped_bytes = row.mapping.bytes > UINT64_MAX - mapped_bytes ?
                    UINT64_MAX : mapped_bytes + row.mapping.bytes;
            }
        }
        const uint64_t requested_bytes = operation == QLV_MAP_BLOB && resource != nullptr ?
            resource->info.byte_count : header.bytes;
        if (file->memory_rejects != UINT64_MAX) { ++file->memory_rejects; }
        if (file->memory_rejects <= 3 || file->memory_rejects % 128 == 0) {
            DbgPrint("LightningNT5: host renderer memory rejected operation %u count %I64u blobs %lu requested %I64u mapped %I64u views %lu aperture %I64u\n",
                static_cast<unsigned>(operation), file->memory_rejects,
                static_cast<ULONG>(blobs), requested_bytes, mapped_bytes,
                static_cast<ULONG>(mapped_views), aperture_bytes_);
        }
    } else if (operation == QLV_SUBMIT || operation == QLV_CLOSE) {
        if (operation == QLV_SUBMIT) {
            DbgPrint("LightningNT5: submit rejected wire %ld payload %lu pending %lu\n",
                static_cast<LONG>(reply.header.status), static_cast<ULONG>(header.payload_bytes),
                static_cast<ULONG>(file->completion.count()));
        }
        // Host error may follow partial command processing. Invalidate generation
        // only after real reset cancellation; never continue with guessed leases.
        recover_all();
    }

    return encode_exchange_reply(reply, output, written) == ExchangeStatus::validated ? STATUS_SUCCESS : STATUS_BUFFER_TOO_SMALL;
}

// Ring_batching: the guest runtime holds requests whose reply it does not need
// at once (a SUBMIT that signals syncs, a WRITE_SYNC) and sends them with its
// next exchange. Each packet is still one complete exchange_file call, in
// order and under this FDO guard; only the doorbell is shared. A packet is
// framed and checked before anything runs, so a malformed batch has no effect.
NTSTATUS NativeRenderer::exchange_batch(PFILE_OBJECT object, PEPROCESS requestor,
    rx7900::ByteView input, rx7900::sdma::CommandBuffer output, size_t& written) noexcept {
    written = 0;
    size_t offsets[exchange_batch_packets]{};
    size_t sizes[exchange_batch_packets]{};
    size_t count = 0;

    if (transport_ == nullptr || input.size() > QLV_HEADER_BYTES + QLV_MAX_PAYLOAD_BYTES) {
        return STATUS_INVALID_PARAMETER;
    }

    for (size_t offset = 0; offset < input.size(); ++count) {
        uint32_t payload = 0;

        if (count == exchange_batch_packets || !input.read_u32(offset + QLV_PAYLOAD_BYTES_OFFSET, payload) ||
            payload > QLV_MAX_PAYLOAD_BYTES || input.size() - offset < QLV_HEADER_BYTES + payload) {
            return STATUS_INVALID_PARAMETER;
        }

        offsets[count] = offset;
        sizes[count] = QLV_HEADER_BYTES + payload;
        offset += sizes[count];
    }

    if (count == 0) {
        return STATUS_INVALID_PARAMETER;
    }

    if (output.size() < count * QLV_HEADER_BYTES) {
        return STATUS_BUFFER_TOO_SMALL;
    }

    for (size_t index = 0; index < count; ++index) {
        uint16_t operation = 0;
        input.read_u16(offsets[index] + QLV_OPERATION_OFFSET, operation);
        const bool last = index + 1 == count;

        if (last ? operation == QLV_OPEN || operation == QLV_CLOSE || operation == QLV_GET_CAPS :
            (operation != QLV_SUBMIT && operation != QLV_WRITE_SYNC) || sizes[index] > exchange_batch_packet_bytes) {
            return STATUS_INVALID_PARAMETER;
        }
    }

    NTSTATUS result = STATUS_SUCCESS;
    transport_->begin_batch();

    for (size_t index = 0; index < count; ++index) {
        // A private reply keeps every header-only reply off the caller's
        // buffer, which METHOD_BUFFERED shares with the later packets.
        uint8_t reply[QLV_HEADER_BYTES]{};
        rx7900::ByteView packet(nullptr, 0);
        size_t one = 0;
        input.subview(offsets[index], sizes[index], packet);
        const NTSTATUS status = exchange_file(object, requestor, packet, {reply, sizeof(reply)}, one);

        if (!NT_SUCCESS(status)) {
            result = index == 0 ? status : STATUS_SUCCESS;
            break;
        }

        const rx7900::ByteView encoded(reply, one);
        const auto slot = output.subbuffer(index * QLV_HEADER_BYTES, QLV_HEADER_BYTES);
        uint32_t wire = 0;

        for (size_t word = 0; word < one; word += 4) {
            uint32_t value = 0;
            encoded.read_u32(word, value);
            slot.write_u32(word, value);
        }

        written += one;

        if (one != QLV_HEADER_BYTES || !encoded.read_u32(QLV_STATUS_OFFSET, wire) || wire != QLV_STATUS_SUCCESS) {
            break;
        }
    }

    transport_->end_batch();
    return result;
}

} // namespace lightning::reactos

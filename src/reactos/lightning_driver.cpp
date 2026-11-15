/*
Module: lightning.reactos.driver
Purpose: Bind Lightning through native NT5 PnP, capabilities and owned renderer I/O.
Created: 2026-10-21
Architecture: Unnamed PnP filter plus named control device; per-file owned sessions.
*/
#include <ntifs.h>
#include <ntddvdeo.h>
#include "lightning_transport.hpp"
#include "lightning_renderer.hpp"
#include "lightning_present_abi.hpp"
#include "lightning_power_queue.hpp"
#include "lightning_display_gate.hpp"
#include "lightning_lifecycle.hpp"
#include "lightning_retirement.hpp"
#include "lightning_rundown.hpp"
#include "lightning_staging_resolve.hpp"
#include "lightning_gdi_abi.hpp"
#include "lightning-display.h"
#include "lifetime.hpp"

namespace {

using namespace lightning::reactos;
static_assert(power_state::system_working == PowerSystemWorking &&
    power_state::system_hibernate == PowerSystemHibernate &&
    power_state::system_shutdown == PowerSystemShutdown &&
    power_state::device_d0 == PowerDeviceD0 && power_state::device_d3 == PowerDeviceD3,
    "Lifecycle power numbering must match the NT power enumerations");

enum class DeviceRole { filter, control };
struct DeviceHeader final { DeviceRole role = DeviceRole::filter; };

// Rundown cell (lightning_rundown.hpp) over the native interlocked word.
struct InterlockedWord final {
    volatile LONG value = 0;
    int32_t load() const noexcept {
        return InterlockedCompareExchange(const_cast<volatile LONG*>(&value), 0, 0);
    }
    int32_t compare_exchange(int32_t desired, int32_t expected) noexcept {
        return InterlockedCompareExchange(&value, desired, expected);
    }
};
using EntryRundown = Rundown<InterlockedWord>;

struct DeviceState final {
    DeviceHeader header{};
    PDEVICE_OBJECT self = nullptr;
    PDEVICE_OBJECT lower = nullptr;
    PDEVICE_OBJECT control = nullptr;
    PDEVICE_OBJECT pdo = nullptr;
    PIRP abandon_irp = nullptr;
    KEVENT abandon_event{};
    IO_REMOVE_LOCK remove_lock{};
    IO_REMOVE_LOCK cleanup_lock{};
    KMUTEX guard{};
    KSPIN_LOCK power_lock{};
    KEVENT power_event{};
    HANDLE power_thread = nullptr;
    PowerQueue<PIRP> power_queue{};
    bool power_active = false;
    volatile LONG power_overflow = 0;
    NativeTransport transport{};
    NativeRenderer renderer{};
    // Every PnP/power transition goes through this state machine and its
    // ordered run_* sequences (lightning_lifecycle.hpp); containment is its
    // sticky latch. Guarded by the lifecycle mutex.
    DeviceLifecycle lifecycle{};
    bool link_live = false;
    bool lower_referenced = false;
    bool attached = false;
    bool display_child = false;
    bool display_quiesced = false;
    bool display_lost = false;
    // Published only after an exact successful private lower ABI query. Once
    // recognized, the same lower device's staging leases stay kernel-only even
    // across STOP, failure containment and subsequent START attempts.
    volatile LONG display_identity = 0;
    // Opened once at REMOVE of a recognized display filter, before the cleanup
    // gate closes. The object, image, lower reference and attachment are then
    // retained for staging retirement only. A retirement-channel dispatch holds
    // no device reference, so it is admitted here before it reads anything
    // else and releases as its last access; retirement_teardown closes it and
    // deletes the object only after every admitted dispatch released. Lock-free
    // so the dispatch entry can branch without the guard; the gate below
    // carries the guarded teardown bookkeeping.
    EntryRundown retirement_entry{};
    // Decides when the retirement channel may finally delete itself: win32k's
    // device reference released and every staging lease retired. This and the
    // two fields below are only touched under the lifecycle guard.
    RetirementGate retirement{};
    // The retirement channel is still attached to its removed lower device.
    bool detach_pending = false;
    // Reserved at REMOVE for the final teardown (retirement_teardown).
    PIO_WORKITEM retirement_work = nullptr;
    // A read-only resolve query answered the miniport's held-lease count.
    bool leases_observed = false;
};

// NT5 process notification has no context argument. This single unnamed FDO is
// published under a spin lock; callbacks acquire its remove lock before releasing
// the registry lock. REMOVE unpublishes it before waiting for callback retirement.
KSPIN_LOCK notification_lock;
PDEVICE_OBJECT notification_device = nullptr;
bool notification_registered = false;
// FDOs of this image that ever latched containment. Each one stays retained
// (never deleted) until reboot, so DriverUnload may be restored only while
// this is zero; a retirement channel's teardown of another FDO must not re-arm
// unload under a retained one. Incremented once per FDO, never decremented.
volatile LONG contained_devices = 0;
static_assert(offsetof(DeviceState, header) == 0, "Dispatch role prefix must be first");
static_assert(alignof(DeviceState) <= MEMORY_ALLOCATION_ALIGNMENT,
              "Native device extension must satisfy C++ state alignment");

class Guard final {
public:
    explicit Guard(KMUTEX& mutex) noexcept : mutex_(mutex) {
        KeWaitForSingleObject(&mutex_, Executive, KernelMode, FALSE, nullptr);
    }
    ~Guard() noexcept { KeReleaseMutex(&mutex_, FALSE); }
    Guard(const Guard&) = delete;
    Guard& operator=(const Guard&) = delete;
private:
    KMUTEX& mutex_;
};

// The lifecycle KMUTEX is recursive for its owner. No supported path calls
// back into this filter while holding it (every private request goes down to
// the lower stack), so a request that arrives on the owning thread is a
// reentrant miniport/VideoPort callback that would observe half-updated
// lifecycle state. Request entry points refuse it with a stable status
// instead of recursing. Only the owner can read its own thread here.
bool guard_held_by_caller(const DeviceState& state) noexcept {
    return state.guard.OwnerThread == KeGetCurrentThread();
}

NTSTATUS finish(PIRP irp, NTSTATUS status, ULONG_PTR information = 0) noexcept {
    irp->IoStatus.Status = status;
    irp->IoStatus.Information = information;
    IoCompleteRequest(irp, IO_NO_INCREMENT);
    return status;
}

void remove_link(DeviceState& state) noexcept {
    if (state.link_live) {
        UNICODE_STRING link;
        RtlInitUnicodeString(&link, dos_device_name);
        IoDeleteSymbolicLink(&link);
        state.link_live = false;
    }
}

NTSTATUS abandon_display(DeviceState& state) noexcept;
extern "C" NTSTATUS NTAPI signal_completion(PDEVICE_OBJECT, PIRP, PVOID);

// Guarded. The single containment entry. The lifecycle latch is sticky, so a
// later STOP, SURPRISE, REMOVE, START or power rollback can never reuse what
// this retains; repeated calls only retry the software abandon and re-close
// the renderer gate. step names the lifecycle sequence that contained, or 0.
void quarantine(DeviceState& state, ULONG cause_line, LifecycleStep step = LifecycleStep{}) noexcept {
    DbgPrint("LightningNT5: containment cause line %lu step %lu phase %u previous %u\n",
        cause_line, static_cast<ULONG>(step), static_cast<unsigned>(state.lifecycle.phase()),
        static_cast<unsigned>(state.lifecycle.previous()));

    if (state.lifecycle.contain()) {
        InterlockedIncrement(&contained_devices);
    }

    remove_link(state);
    // Reboot-only containment. We deliberately retain FDO/DMA/adapter/interface
    // ownership and disable the actual unload entry before detaching on REMOVE.
    // An FDO reference alone is not an unload guarantee. The owned power thread
    // is joined before REMOVE detaches; no later START may reuse these pages.
    state.self->DriverObject->DriverUnload = nullptr;
    const NTSTATUS abandoned = abandon_display(state);
    // Containment must also close the local renderer execution gate even when
    // a preceding display failure prevented ordinary teardown. These paths
    // retain borrowed CPU views and never send QLV MMIO to a disappeared device.
    const NTSTATUS renderer = state.renderer.shutdown(false);
    const NTSTATUS transport = NT_SUCCESS(renderer) ? state.transport.stop(false) : STATUS_DEVICE_BUSY;
    DbgPrint("LightningNT5: retained quarantine, display %08lx renderer %08lx transport %08lx, reboot required\n",
        abandoned, renderer, transport);
}

extern "C" VOID NTAPI process_notification(HANDLE, HANDLE process, BOOLEAN create) {
    if (create) {
        return;
    }

    KIRQL level;
    PDEVICE_OBJECT device = nullptr;
    ULONG token = 0;
    KeAcquireSpinLock(&notification_lock, &level);

    if (notification_device != nullptr) {
        auto& candidate = *static_cast<DeviceState*>(notification_device->DeviceExtension);

        if (NT_SUCCESS(IoAcquireRemoveLock(&candidate.remove_lock, &token))) {
            device = notification_device;
        }
    }

    KeReleaseSpinLock(&notification_lock, level);

    if (device != nullptr) {
        auto& state = *static_cast<DeviceState*>(device->DeviceExtension);
        {
            Guard guard(state.guard);
            state.renderer.process_exit(process);

            if (state.renderer.quarantined() || state.transport.quarantined()) {
                quarantine(state, __LINE__);
            }
        }

        IoReleaseRemoveLock(&state.remove_lock, &token);
    }
}

// Guarded. Closes the renderer gate; a busy or quarantined renderer (live
// borrowed views, uncertain host session) fails and the calling lifecycle
// sequence contains the device.
NTSTATUS stop_renderer(DeviceState& state, bool mmio) noexcept {
    const NTSTATUS status = state.renderer.shutdown(mmio);
    return !NT_SUCCESS(status) || state.renderer.quarantined() ? STATUS_DEVICE_BUSY : STATUS_SUCCESS;
}

// These private requests address the already-started lower stack directly,
// never the control DO. The VideoPort dispatch completes its miniport StartIO
// synchronously; a pending lower request is nevertheless joined before stack
// event/output storage retires. Every call is guarded at PASSIVE_LEVEL.
NTSTATUS display_request(DeviceState& state, ULONG code, PVOID output = nullptr,
                         ULONG output_bytes = 0, ULONG_PTR* returned = nullptr,
                         PVOID input = nullptr, ULONG input_bytes = 0) noexcept {
    if (code == ioctl_abandon_cpu_scanout) {
        if (state.abandon_irp == nullptr || !state.attached) {
            return STATUS_DEVICE_NOT_CONNECTED;
        }

        // This zero-payload containment IRP was reserved before publishing the
        // device. Completion is halted so reuse/free cannot race lower ownership.
        IoReuseIrp(state.abandon_irp, STATUS_NOT_SUPPORTED);
        KeClearEvent(&state.abandon_event);
        state.abandon_irp->RequestorMode = KernelMode;
        auto* next = IoGetNextIrpStackLocation(state.abandon_irp);
        *next = IO_STACK_LOCATION{};
        next->MajorFunction = IRP_MJ_DEVICE_CONTROL;
        next->Parameters.DeviceIoControl.IoControlCode = code;
        IoSetCompletionRoutine(state.abandon_irp, signal_completion, &state.abandon_event, TRUE, TRUE, TRUE);
        const NTSTATUS status = IoCallDriver(state.lower, state.abandon_irp);

        if (status == STATUS_PENDING) {
            KeWaitForSingleObject(&state.abandon_event, Executive, KernelMode, FALSE, nullptr);
        }

        if (returned != nullptr) {
            *returned = state.abandon_irp->IoStatus.Information;
        }

        return state.abandon_irp->IoStatus.Status;
    }

    KEVENT event;
    IO_STATUS_BLOCK result{};
    KeInitializeEvent(&event, NotificationEvent, FALSE);
    // Optional METHOD_BUFFERED input (async-present receipt queries) is copied
    // by the I/O manager before the lower call; nothing here retains it.
    PIRP request = IoBuildDeviceIoControlRequest(code, state.lower, input, input_bytes,
        output, output_bytes, FALSE, &event, &result);

    if (request == nullptr) {
        return STATUS_INSUFFICIENT_RESOURCES;
    }

    const NTSTATUS status = IoCallDriver(state.lower, request);

    if (status == STATUS_PENDING) {
        KeWaitForSingleObject(&event, Executive, KernelMode, FALSE, nullptr);
    }

    if (returned != nullptr) {
        *returned = result.Information;
    }

    return result.Status;
}

NTSTATUS discover_display(DeviceState& state) noexcept {
    CpuScanoutInformation information{};
    ULONG_PTR returned = 0;
    const NTSTATUS status = display_request(state, ioctl_query_cpu_scanout,
        &information, sizeof(information), &returned);
    state.display_child = false;
    state.display_quiesced = false;
    state.display_lost = false;

    if (status == STATUS_NOT_SUPPORTED || status == STATUS_INVALID_DEVICE_REQUEST ||
        status == STATUS_NOT_IMPLEMENTED) {
        // PCI standalone and legacy miniports have no private presentation ABI.
        return STATUS_SUCCESS;
    }

    if (!NT_SUCCESS(status)) {
        return status;
    }

    if (returned != sizeof(information) || information.size != sizeof(information) ||
        information.version != cpu_scanout_abi_version || information.magic != LIGHTNING_DISPLAY_MAGIC ||
        information.width == 0 || information.height == 0 || information.enabled > 1 ||
        static_cast<uint64_t>(information.width) * 4 > information.stride ||
        static_cast<uint64_t>(information.stride) * information.height > information.mapped_bytes) {
        return STATUS_DEVICE_PROTOCOL_ERROR;
    }

    state.display_child = true;
    InterlockedExchange(&state.display_identity, 1);
    state.display_quiesced = information.enabled == 0;
    return STATUS_SUCCESS;
}

// Software-only containment of the display client's staging leases. A lower
// device whose private ABI was ever recognized keeps its lease ledger even when
// a later restart's discovery failed (display_child cleared), so the latched
// identity alone is enough to require the abandon.
bool display_ever_recognized(const DeviceState& state) noexcept {
    return state.display_child || InterlockedCompareExchange(
        const_cast<volatile LONG*>(&state.display_identity), 0, 0) != 0;
}

NTSTATUS abandon_display(DeviceState& state) noexcept {
    if (!display_ever_recognized(state) || state.display_lost || !state.attached) {
        return STATUS_SUCCESS;
    }

    ULONG_PTR returned = 0;
    const NTSTATUS status = display_request(state, ioctl_abandon_cpu_scanout, nullptr, 0, &returned);

    if (NT_SUCCESS(status) && returned == 0) {
        state.display_lost = true;
        state.display_quiesced = true;
        return STATUS_SUCCESS;
    }

    return NT_SUCCESS(status) ? STATUS_DEVICE_PROTOCOL_ERROR : status;
}

NTSTATUS quiesce_display(DeviceState& state) noexcept {
    if (!state.display_child || state.display_quiesced) {
        return STATUS_SUCCESS;
    }

    ULONG_PTR returned = 0;
    const NTSTATUS status = display_request(state, ioctl_quiesce_cpu_scanout, nullptr, 0, &returned);

    if (NT_SUCCESS(status) && returned == 0) {
        state.display_quiesced = true;
        // The miniport drained and reset its GPU canvas (confirmed backend
        // release) before answering, so no asynchronous image read remains.
        state.renderer.retire_receipts_after_display_reset();
        return STATUS_SUCCESS;
    }

    return NT_SUCCESS(status) ? STATUS_DEVICE_PROTOCOL_ERROR : status;
}

NTSTATUS resume_display(DeviceState& state) noexcept {
    if (state.display_lost) {
        return STATUS_DEVICE_NOT_CONNECTED;
    }

    if (!state.display_child) {
        return STATUS_SUCCESS;
    }

    ULONG_PTR returned = 0;
    const NTSTATUS status = display_request(state, ioctl_resume_cpu_scanout, nullptr, 0, &returned);

    if (NT_SUCCESS(status) && returned == 0) {
        state.display_quiesced = false;
        return STATUS_SUCCESS;
    }

    return NT_SUCCESS(status) ? STATUS_DEVICE_PROTOCOL_ERROR : status;
}

void observe_retained_leases(DeviceState& state) noexcept;

// The lightning_lifecycle.hpp Owners concept over one FDO: what each ordered
// lifecycle step means natively. Constructed and used only under the lifecycle
// mutex at PASSIVE_LEVEL, for one IRP. status keeps that IRP's first failing
// native status. The renderer is optional (no aperture, no renderer), so its
// start result is not a transition failure; a busy stop is.
struct NativeOwners final {
    DeviceState& state;
    PCM_RESOURCE_LIST raw = nullptr;
    PCM_RESOURCE_LIST translated = nullptr;
    // START keeps its established evidence lines for the VM log checker.
    bool trace_start = false;
    NTSTATUS status = STATUS_SUCCESS;

    bool record(NTSTATUS result) noexcept {
        if (!NT_SUCCESS(result) && NT_SUCCESS(status)) {
            status = result;
        }

        return NT_SUCCESS(result);
    }

    bool transport_start() noexcept {
        const NTSTATUS result = state.transport.start(state.pdo, raw, translated);
        DbgPrint("LightningNT5: transport start status %08lx\n", static_cast<ULONG>(result));
        return record(result);
    }

    bool transport_resume() noexcept { return record(state.transport.resume()); }
    bool transport_suspend() noexcept { return record(state.transport.suspend()); }
    bool transport_stop(bool hardware) noexcept { return record(state.transport.stop(hardware)); }
    bool transport_ready() const noexcept { return state.transport.ready(); }
    bool transport_contained() const noexcept { return state.transport.quarantined(); }

    bool display_discover() noexcept {
        const NTSTATUS result = discover_display(state);
        DbgPrint("LightningNT5: display discovery status %08lx\n", static_cast<ULONG>(result));
        return record(result);
    }

    bool display_resume() noexcept {
        const NTSTATUS result = resume_display(state);

        if (trace_start) {
            DbgPrint("LightningNT5: display resume status %08lx\n", static_cast<ULONG>(result));
        }

        return record(result);
    }

    bool display_quiesce() noexcept { return record(quiesce_display(state)); }
    bool display_abandon() noexcept { return record(abandon_display(state)); }
    bool display_recognized() const noexcept { return display_ever_recognized(state); }
    bool display_lost() const noexcept { return state.display_lost; }
    void observe_leases() noexcept { observe_retained_leases(state); }
    void renderer_start() noexcept { state.renderer.initialize(state.transport); }
    bool renderer_stop(bool hardware) noexcept { return record(stop_renderer(state, hardware)); }

    bool publish_control() noexcept {
        UNICODE_STRING link;
        UNICODE_STRING name;
        RtlInitUnicodeString(&link, dos_device_name);
        RtlInitUnicodeString(&name, nt_device_name);
        const NTSTATUS result = IoCreateSymbolicLink(&link, &name);
        state.link_live = NT_SUCCESS(result);
        return record(result);
    }

    void withdraw_control() noexcept { remove_link(state); }
    void contain(LifecycleStep step) noexcept { quarantine(state, __LINE__, step); }
};

extern "C" NTSTATUS NTAPI signal_completion(PDEVICE_OBJECT, PIRP, PVOID context) {
    KeSetEvent(static_cast<PKEVENT>(context), IO_NO_INCREMENT, FALSE);
    return STATUS_MORE_PROCESSING_REQUIRED;
}

// PASSIVE_LEVEL only. Completion halts unwinding so the caller still owns the
// IRP while acquiring/releasing resources. The native lower driver owns progress.
NTSTATUS forward_wait(DeviceState& state, PIRP irp, bool power = false) noexcept {
    KEVENT event;
    KeInitializeEvent(&event, NotificationEvent, FALSE);
    IoCopyCurrentIrpStackLocationToNext(irp);
    IoSetCompletionRoutine(irp, signal_completion, &event, TRUE, TRUE, TRUE);
    const NTSTATUS status = power ? PoCallDriver(state.lower, irp) : IoCallDriver(state.lower, irp);

    if (status == STATUS_PENDING) {
        KeWaitForSingleObject(&event, Executive, KernelMode, FALSE, nullptr);
    }

    return irp->IoStatus.Status;
}

extern "C" NTSTATUS NTAPI release_completion(PDEVICE_OBJECT, PIRP irp, PVOID context) {
    auto& state = *static_cast<DeviceState*>(context);

    if (irp->PendingReturned) {
        IoMarkIrpPending(irp);
    }

    IoReleaseRemoveLock(&state.remove_lock, irp);
    return STATUS_CONTINUE_COMPLETION;
}

NTSTATUS forward_locked(DeviceState& state, PIRP irp, bool power = false) noexcept {
    IoCopyCurrentIrpStackLocationToNext(irp);
    IoSetCompletionRoutine(irp, release_completion, &state, TRUE, TRUE, TRUE);
    return power ? PoCallDriver(state.lower, irp) : IoCallDriver(state.lower, irp);
}

bool is_control(PDEVICE_OBJECT device) noexcept {
    // Both extensions belong to this DriverObject and are constructed before
    // publication. Only the shared first member is inspected before role dispatch.
    return static_cast<DeviceHeader*>(device->DeviceExtension)->role == DeviceRole::control;
}

extern "C" NTSTATUS NTAPI release_cleanup_completion(PDEVICE_OBJECT, PIRP irp, PVOID context) {
    auto& state = *static_cast<DeviceState*>(context);

    if (irp->PendingReturned) {
        IoMarkIrpPending(irp);
    }

    IoReleaseRemoveLock(&state.cleanup_lock, irp);
    return STATUS_CONTINUE_COMPLETION;
}

// async-present: what authorize_external_image decided for one display IRP.
// image: an external-image submit whose receipt must prove retirement or be
// pinned; async: forwarded as version 2, so an accepted outstanding read is
// pinned per image; answered: an ioctl_external_receipt answered here alone.
struct ExternalAuthorization final {
    bool image = false;
    bool async = false;
    bool answered = false;
    uint64_t lease = 0;
    ULONG_PTR information = 0;
};

// Guarded. Coherence-only native GDI query of the lower canvas progress.
bool query_display_progress(DeviceState& state, gdi::Information& information) noexcept {
    gdi::QueryRequest request{sizeof(request), gdi::native_abi_version, gdi::query_coherence_only, 0};
    ULONG_PTR returned = 0;
    const NTSTATUS status = display_request(state, gdi::ioctl_query, &information, sizeof(information),
        &returned, &request, sizeof(request));

    return NT_SUCCESS(status) && returned == sizeof(information) && information.size == sizeof(information) &&
        information.version == gdi::native_abi_version && information.magic == LIGHTNING_GDI_MAGIC;
}

// ReceiptOwner callback, guarded by the renderer caller. wait=false never
// blocks: an unanswered query stays pending. wait=true uses the miniport's
// bounded completion wait; anything it cannot prove retired is uncertain.
ReceiptState resolve_display_receipt(void* context, const ImageReceipt& receipt, bool wait) noexcept {
    auto& state = *static_cast<DeviceState*>(context);
    gdi::Information information{};

    if (!query_display_progress(state, information)) {
        if (wait) {
            DbgPrint("LightningNT5: receipt %lu/%lu progress query failed\n",
                static_cast<ULONG>(receipt.epoch), static_cast<ULONG>(receipt.sequence));
        }
        return wait ? ReceiptState::uncertain : ReceiptState::pending;
    }

    const ReceiptState observed = decide_receipt(receipt, {information.epoch, information.accepted_sequence,
        information.completed_sequence, information.quarantined != 0});

    if (observed == ReceiptState::uncertain || (wait && observed == ReceiptState::pending)) {
        DbgPrint("LightningNT5: receipt %lu/%lu canvas generation %lu epoch %lu accepted %lu completed %lu "
            "quarantined %lu state %u\n", static_cast<ULONG>(receipt.epoch), static_cast<ULONG>(receipt.sequence),
            information.mode_generation, information.epoch, information.accepted_sequence,
            information.completed_sequence, information.quarantined, static_cast<unsigned>(observed));
    }

    if (observed != ReceiptState::pending || !wait) {
        return observed;
    }

    gdi::WaitRequest request{sizeof(request), gdi::native_abi_version, information.mode_generation, receipt.sequence};
    gdi::Receipt completion{};
    ULONG_PTR returned = 0;
    const NTSTATUS status = display_request(state, gdi::ioctl_wait, &completion, sizeof(completion),
        &returned, &request, sizeof(request));

    if (!NT_SUCCESS(status) || returned != sizeof(completion) || completion.size != sizeof(completion) ||
        completion.version != gdi::native_abi_version) {
        DbgPrint("LightningNT5: receipt %lu/%lu wait request failed %08lx returned %lu\n",
            static_cast<ULONG>(receipt.epoch), static_cast<ULONG>(receipt.sequence), status,
            static_cast<ULONG>(returned));
        return ReceiptState::uncertain;
    }

    const ReceiptState waited = decide_receipt(receipt, {completion.epoch, completion.accepted_sequence,
        completion.completed_sequence, completion.outcome != gdi::accepted});

    if (waited != ReceiptState::retired) {
        DbgPrint("LightningNT5: receipt %lu/%lu wait status %08lx outcome %lu result %lu generation %lu epoch %lu "
            "accepted %lu completed %lu\n", static_cast<ULONG>(receipt.epoch), static_cast<ULONG>(receipt.sequence),
            status, completion.outcome, completion.result, completion.mode_generation, completion.epoch,
            completion.accepted_sequence, completion.completed_sequence);
    }
    return waited == ReceiptState::retired ? waited : ReceiptState::uncertain;
}

// Guarded, once per device: only a miniport that accepts the async query flag
// honors version-2 external images. An explicit rejection is final; any other
// failure (quiesced, not yet initialized) is retried at the next probe.
void install_receipt_owner(DeviceState& state) noexcept {
    if (state.renderer.receipt_owner_installed() || !state.display_child) {
        return;
    }

    gdi::QueryRequest request{sizeof(request), gdi::native_abi_version,
        gdi::query_coherence_only | gdi::query_async_external, 0};
    gdi::Information information{};
    ULONG_PTR returned = 0;
    const NTSTATUS status = display_request(state, gdi::ioctl_query, &information, sizeof(information),
        &returned, &request, sizeof(request));
    const bool accepted = NT_SUCCESS(status) && returned == sizeof(information);

    if (accepted || status == STATUS_INVALID_PARAMETER) {
        state.renderer.set_receipt_owner({&state, resolve_display_receipt, accepted});
    }
}

// Kernel display client only. Answered under the guard without forwarding; the
// lower canvas is observed, never waited for. lease zero is the async probe.
NTSTATUS answer_external_receipt(DeviceState& state, PIRP irp, ExternalAuthorization& external) noexcept {
    external.answered = true;
    const auto* stack = IoGetCurrentIrpStackLocation(irp);
    void* buffer = irp->AssociatedIrp.SystemBuffer;

    if (irp->RequestorMode != KernelMode || buffer == nullptr ||
        stack->Parameters.DeviceIoControl.InputBufferLength != sizeof(gdi::ExternalReceiptRequest) ||
        stack->Parameters.DeviceIoControl.OutputBufferLength < sizeof(gdi::ExternalReceipt)) {
        return STATUS_ACCESS_DENIED;
    }

    gdi::ExternalReceiptRequest request;
    RtlCopyMemory(&request, buffer, sizeof(request));

    if (request.size != sizeof(request) || request.version != gdi::native_abi_version) {
        return STATUS_INVALID_PARAMETER;
    }

    install_receipt_owner(state);
    gdi::ExternalReceipt reply{sizeof(reply), gdi::native_abi_version, gdi::external_receipt_retired,
        state.renderer.async_receipts() ? gdi::external_feature_async : 0U};

    if (request.lease != 0) {
        const ReceiptState current = state.renderer.query_receipt(request.lease, PsGetCurrentProcess());
        reply.status = current == ReceiptState::retired ? gdi::external_receipt_retired :
            (current == ReceiptState::pending ? gdi::external_receipt_pending : gdi::external_receipt_failed);
    }

    RtlCopyMemory(buffer, &reply, sizeof(reply));
    external.information = sizeof(reply);
    return STATUS_SUCCESS;
}

// Called under the FDO lifecycle guard before forwarding a kernel GDI submit.
// METHOD_BUFFERED owns the complete input/output extent. No user address or
// caller-chosen context/resource reaches the lower miniport. The guard must
// stay held through the lower synchronous external-image execution barrier;
// a version-2 copy instead stays pinned per image until its read retires.
NTSTATUS authorize_external_image(DeviceState& state, PIRP irp, ExternalAuthorization& external) noexcept {
    external = {};
    const auto* stack = IoGetCurrentIrpStackLocation(irp);

    if (stack->MajorFunction == IRP_MJ_DEVICE_CONTROL &&
        stack->Parameters.DeviceIoControl.IoControlCode == gdi::ioctl_external_receipt) {
        return answer_external_receipt(state, irp, external);
    }

    if (stack->MajorFunction != IRP_MJ_DEVICE_CONTROL ||
        stack->Parameters.DeviceIoControl.IoControlCode != gdi::ioctl_submit) {
        return STATUS_SUCCESS;
    }

    constexpr size_t opcode_offset = offsetof(gdi::SubmitRequest, command) + offsetof(LightningGdiCommand, opcode);
    const size_t input_bytes = stack->Parameters.DeviceIoControl.InputBufferLength;
    auto* buffer = static_cast<uint8_t*>(irp->AssociatedIrp.SystemBuffer);
    uint32_t opcode = 0;

    if (!rx7900::ByteView(buffer, input_bytes).read_u32(opcode_offset, opcode) ||
        opcode != LIGHTNING_GDI_EXTERNAL_IMAGE) {
        return STATUS_SUCCESS;
    }

    external.image = true;
    constexpr size_t descriptor_offset = sizeof(gdi::SubmitRequest);
    constexpr size_t minimum_input_bytes = descriptor_offset + sizeof(LightningGdiExternalImage);
    static_assert(sizeof(LightningGdiExternalImage) == LIGHTNING_GDI_EXTERNAL_IMAGE_BYTES &&
        offsetof(LightningGdiExternalImage, context) == 8 &&
        offsetof(LightningGdiExternalImage, lease) == 24 &&
        offsetof(LightningGdiExternalImage, width) == 32, "Canonical external image ABI");

    if (irp->RequestorMode != KernelMode || buffer == nullptr || input_bytes < minimum_input_bytes ||
        stack->Parameters.DeviceIoControl.OutputBufferLength < sizeof(gdi::Receipt)) {
        return STATUS_ACCESS_DENIED;
    }

    gdi::SubmitRequest submit;
    LightningGdiExternalImage descriptor;
    RtlCopyMemory(&submit, buffer, sizeof(submit));
    RtlCopyMemory(&descriptor, buffer + descriptor_offset, sizeof(descriptor));
    const uint64_t expected_bytes = minimum_input_bytes +
        static_cast<uint64_t>(submit.command.clip_count) * sizeof(LightningDisplayRect);

    // Version 2 carries the producer sync point in context (value) and
    // resource (token); version 1 requires both zero, as before.
    const bool asynchronous = submit.version == gdi::native_async_abi_version;

    if ((submit.version != gdi::native_abi_version && !asynchronous) || submit.size != input_bytes ||
        submit.command.clip_count > LIGHTNING_GDI_MAX_RECTS || expected_bytes != input_bytes ||
        submit.command.metadata_bytes != input_bytes - offsetof(gdi::SubmitRequest, command) ||
        descriptor.size != sizeof(descriptor) || descriptor.version != LIGHTNING_GDI_EXTERNAL_IMAGE_VERSION ||
        (!asynchronous && (descriptor.context != 0 || descriptor.resource != 0))) {
        return STATUS_INVALID_PARAMETER;
    }

    const ProducerPoint producer{descriptor.resource, descriptor.context};
    NativeImageLease owned;
    const NTSTATUS status = state.renderer.resolve_image(descriptor.lease, PsGetCurrentProcess(), owned,
        asynchronous ? &producer : nullptr);

    if (!NT_SUCCESS(status)) {
        return status;
    }

    if (owned.layout.format != QLV_SCANOUT_BGRA8 || descriptor.width != owned.layout.width ||
        descriptor.height != owned.layout.height || descriptor.stride != owned.layout.stride ||
        descriptor.offset != owned.layout.offset) {
        return STATUS_INVALID_PARAMETER;
    }

    descriptor.context = owned.context;
    descriptor.resource = owned.resource;
    RtlCopyMemory(buffer + descriptor_offset, &descriptor, sizeof(descriptor));

    if (asynchronous) {
        // Keep version 2 only while this exact lease can be pinned; otherwise
        // the lower copy stays synchronous under this guard, as for version 1.
        install_receipt_owner(state);
        external.lease = descriptor.lease;
        external.async = state.renderer.can_track_receipt(descriptor.lease, PsGetCurrentProcess());

        if (!external.async) {
            submit.version = gdi::native_abi_version;
            RtlCopyMemory(buffer, &submit, sizeof(submit));
        }
    }

    return STATUS_SUCCESS;
}

bool external_image_retired(PIRP irp, NTSTATUS status) noexcept {
    if (!NT_SUCCESS(status) || irp->IoStatus.Information != sizeof(gdi::Receipt) ||
        irp->AssociatedIrp.SystemBuffer == nullptr) {
        return false;
    }

    gdi::Receipt receipt;
    RtlCopyMemory(&receipt, irp->AssociatedIrp.SystemBuffer, sizeof(receipt));

    if (receipt.size != sizeof(receipt) || receipt.version != gdi::native_abi_version) {
        return false;
    }

    // A declined receipt guarantees no external read was accepted. Otherwise
    // only the authoritative execution counter can release the source lease.
    return receipt.outcome == gdi::declined ||
        (receipt.outcome == gdi::accepted && receipt.result == LIGHTNING_GDI_OK &&
         receipt.accepted_sequence != 0 && receipt.completed_sequence >= receipt.accepted_sequence);
}

// Guarded, after the lower copy returned. A retired or declined receipt needs
// nothing; an accepted outstanding version-2 read is pinned on its image.
// False: the source read can no longer be proven retired (containment).
bool finish_external_image(DeviceState& state, PIRP irp, NTSTATUS status,
                           const ExternalAuthorization& external) noexcept {
    if (external_image_retired(irp, status)) {
        return true;
    }

    if (!external.async || !NT_SUCCESS(status) || irp->IoStatus.Information != sizeof(gdi::Receipt) ||
        irp->AssociatedIrp.SystemBuffer == nullptr) {
        return false;
    }

    gdi::Receipt receipt;
    RtlCopyMemory(&receipt, irp->AssociatedIrp.SystemBuffer, sizeof(receipt));

    if (receipt.size != sizeof(receipt) || receipt.version != gdi::native_abi_version ||
        receipt.outcome != gdi::accepted || receipt.result != LIGHTNING_GDI_OK ||
        receipt.accepted_sequence == 0 || receipt.completed_sequence >= receipt.accepted_sequence) {
        return false;
    }

    return state.renderer.track_receipt(external.lease, PsGetCurrentProcess(),
        {receipt.mode_generation, receipt.epoch, receipt.accepted_sequence});
}

extern "C" VOID NTAPI unload(PDRIVER_OBJECT);

// At most this many 10 ms polls for win32k's final CLOSE to unwind.
constexpr ULONG retirement_drain_polls = 100;
// At most this many 10 ms polls for admitted retirement dispatches once
// admission closed: each is one synchronous lower StartIO, bounded by the
// miniport. Beyond this the channel is retained, never deleted under it.
constexpr ULONG retirement_entry_drain_polls = 600;

// Final teardown of the retirement channel, on the work item reserved at
// REMOVE. The gate has latched (begin_teardown returned true), so this runs
// exactly once, after the IRP that dropped the last reference has completed.
// The work item references this device object: deleting it here may unload the
// driver (IopUnloadDevice runs DriverUnload synchronously), but the image stays
// mapped until this routine has returned. Detaching last lets the removed lower
// device and its miniport unload in turn.
extern "C" VOID NTAPI retirement_teardown(PDEVICE_OBJECT self, PVOID) {
    auto& state = *static_cast<DeviceState*>(self->DeviceExtension);
    PIO_WORKITEM work = nullptr;
    PDEVICE_OBJECT lower = nullptr;
    bool lower_ref = false;
    bool detach = false;

    // The dispatch that queued this has to leave the image first: nothing holds
    // a reference on this device for a CLOSE or DEVICE_CONTROL, and ReactOS
    // unloads synchronously from IoDeleteDevice. For win32k's final CLOSE the
    // I/O manager drops its reference on the lower device only after that
    // dispatch returned. Bounded: a stuck foreign open only delays this.
    LARGE_INTEGER pause{};
    pause.QuadPart = -10LL * 1000 * 10; // 10 ms, relative

    for (ULONG waited = 0; waited < retirement_drain_polls; ++waited) {
        if (state.retirement_entry.active() == 0 &&
            (state.lower == nullptr || state.lower->ReferenceCount == 0)) {
            break;
        }

        KeDelayExecutionThread(KernelMode, FALSE, &pause);
    }

    // lightning-overlap: refuse new retirement dispatches, then wait for every
    // admitted one. Each admitted dispatch counted itself before it read the
    // channel state, so none can still reach the guard, the lower device or
    // the gate after the count drains. A dispatch stuck in the lower device
    // keeps the channel (object, image, lower reference, attachment, work
    // item) until reboot: deleting the extension under it would be a
    // use-after-free, and DriverUnload stays cleared.
    state.retirement_entry.close();
    bool drained = false;

    for (ULONG waited = 0; waited < retirement_entry_drain_polls; ++waited) {
        if (state.retirement_entry.drained()) {
            drained = true;
            break;
        }

        KeDelayExecutionThread(KernelMode, FALSE, &pause);
    }

    if (!drained) {
        DbgPrint("LightningNT5: retirement teardown contained, %lu dispatches still admitted; reboot required\n",
            static_cast<ULONG>(state.retirement_entry.active()));
        return;
    }

    // One more pause covers the few instructions a dispatch runs after its
    // release, the last access it makes to this extension.
    KeDelayExecutionThread(KernelMode, FALSE, &pause);
    {
        Guard guard(state.guard);
        work = state.retirement_work;
        lower = state.lower;
        lower_ref = state.lower_referenced;
        detach = state.detach_pending;
        state.retirement_work = nullptr;
        state.lower = nullptr;
        state.pdo = nullptr;
        state.lower_referenced = false;
        state.detach_pending = false;
    }

    // Another FDO of this image may be contained (retained until reboot): its
    // quarantine cleared DriverUnload and that must survive this teardown.
    const LONG contained = InterlockedCompareExchange(&contained_devices, 0, 0);

    if (contained == 0) {
        self->DriverObject->DriverUnload = unload;
        DbgPrint("LightningNT5: retirement complete; deleting detached filter and re-arming unload\n");
    } else {
        DbgPrint("LightningNT5: retirement complete; deleting detached filter, unload stays disarmed (%ld contained)\n",
            contained);
    }

    IoFreeWorkItem(work);

    if (detach) {
        IoDetachDevice(lower);
    }

    if (lower_ref) {
        ObDereferenceObject(lower);
    }

    IoDeleteDevice(self);
}

void queue_retirement_teardown(DeviceState& state) noexcept {
    IoQueueWorkItem(state.retirement_work, retirement_teardown, DelayedWorkQueue, nullptr);
}

// After abandon the miniport has already freed staging no client holds. Ask
// how many leases remain, so a removal with none outstanding retires on the
// last close alone. Any read-only query answers the ledger's held count; a
// failed one leaves the gate waiting for a client's resolve receipt.
void observe_retained_leases(DeviceState& state) noexcept {
    const StagingResolveRequest request{sizeof(request), cpu_scanout_abi_version, 1, staging_resolve_query, 0};
    StagingResolveReceipt receipt{};
    ULONG_PTR returned = 0;
    StagingResolveRequest input = request;
    const NTSTATUS status = display_request(state, ioctl_resolve_owned_staging, &receipt, sizeof(receipt),
        &returned, &input, sizeof(input));

    if (NT_SUCCESS(status) && returned == sizeof(receipt) && valid_staging_resolve_receipt(receipt, request)) {
        state.retirement.observe_held_leases(receipt.held_leases);
        state.leases_observed = true;
    }
}

// REMOVE of a display whose staging the client may still lease: keep this
// object, its image, the lower reference and the attachment as the
// retirement-only channel. The display client frees each lease after moving
// its views, and win32k's final CLOSE arrives here. The teardown work item is
// reserved now so that the last release cannot fail; without it, or in
// containment, the channel stays until reboot. Completes the REMOVE IRP.
NTSTATUS retain_for_retirement(DeviceState& state, PDEVICE_OBJECT device, PIRP irp, NTSTATUS status) noexcept {
    bool teardown = false;
    {
        Guard guard(state.guard);
        device->DriverObject->DriverUnload = nullptr;
        state.retirement_work = state.lifecycle.retirement_teardown_allowed() ? IoAllocateWorkItem(device) : nullptr;

        if (state.retirement_work != nullptr) {
            // The REMOVE-time held-lease query is the only lease report a
            // client holding no lease ever causes; retry it once if it failed.
            if (!state.leases_observed && display_ever_recognized(state)) {
                observe_retained_leases(state);
            }

            state.retirement.enter_retirement();
            // Nothing opened and nothing leased: no later event would come.
            teardown = state.retirement.begin_teardown();
        }

        // REMOVE already opened admission before closing the cleanup gate.
        state.retirement_entry.open();
        DbgPrint("LightningNT5: removed display retained as staging retirement channel (opens %ld, leases %s, teardown %s)\n",
            state.retirement.foreign_opens(), state.retirement.leases_retired() ? "retired" : "held",
            state.retirement_work != nullptr ? "armed" : "never");
    }

    finish(irp, status);

    if (teardown) {
        queue_retirement_teardown(state);
    }

    return status;
}

// Guarded, after a lower resolve returned. Its receipt reports the miniport's
// post-operation held-lease count. Only once the display is abandoned is that
// count final (no MAP can issue a new lease), so only then does it reach the
// retirement gate: a client that releases its last lease while REMOVE is still
// between the abandon and the remove-lock drain is observed here, not lost.
bool observe_resolve_receipt(DeviceState& state, PIRP irp, NTSTATUS status, ULONG_PTR information) noexcept {
    const auto* stack = IoGetCurrentIrpStackLocation(irp);

    if (!state.display_lost || !NT_SUCCESS(status) || stack->MajorFunction != IRP_MJ_DEVICE_CONTROL ||
        stack->Parameters.DeviceIoControl.IoControlCode != ioctl_resolve_owned_staging ||
        information != sizeof(StagingResolveReceipt) || irp->AssociatedIrp.SystemBuffer == nullptr) {
        return false;
    }

    StagingResolveReceipt receipt;
    RtlCopyMemory(&receipt, irp->AssociatedIrp.SystemBuffer, sizeof(receipt));
    state.retirement.observe_held_leases(receipt.held_leases);
    return true;
}

// After REMOVE, win32k can still call this filter through the top-of-stack
// pointer it captured at open. Kernel staging retirement alone is forwarded,
// synchronously and serialized, to the explicitly referenced lower device (a
// removed videoprt FDO still serves its miniport until its last close), so the
// display client can release leased RAM once it has moved its views off it.
NTSTATUS forward_retirement(DeviceState& state, PIRP irp) noexcept {
    const auto* stack = IoGetCurrentIrpStackLocation(irp);

    if (KeGetCurrentIrql() != PASSIVE_LEVEL || guard_held_by_caller(state) ||
        stack->MajorFunction != IRP_MJ_DEVICE_CONTROL ||
        !retained_retirement_admitted(stack->Parameters.DeviceIoControl.IoControlCode,
            irp->RequestorMode == KernelMode, IOCTL_VIDEO_UNMAP_VIDEO_MEMORY)) {
        return finish(irp, STATUS_DELETE_PENDING);
    }

    ULONG_PTR information = 0;
    NTSTATUS status = STATUS_SUCCESS;
    bool teardown = false;
    {
        Guard guard(state.guard);
        // Teardown may already have taken the lower device.
        status = state.lower != nullptr ? forward_wait(state, irp) : STATUS_DELETE_PENDING;
        information = state.lower != nullptr ? irp->IoStatus.Information : 0;

        // When the held-lease count reaches zero the last RAM lease is gone;
        // if win32k has also released the device this completes the retirement.
        if (state.lower != nullptr && observe_resolve_receipt(state, irp, status, information)) {
            teardown = state.retirement.begin_teardown();
        }
    }

    finish(irp, status, information);

    if (teardown) {
        queue_retirement_teardown(state);
    }

    return status;
}

// win32k releasing its display file object arrives here as the final CLOSE:
// the I/O manager sends it to the top of the removed lower device's stack,
// which is still this filter (REMOVE keeps a retained display attached).
// Forward it to the lower device, drop the open count, and - if that was the
// last reference and the staging leases are already retired - queue the
// teardown so the driver can unload without a reboot. PASSIVE only, admitted
// through retirement_entry; the teardown waits for that admission to drain.
NTSTATUS close_retirement(DeviceState& state, PIRP irp) noexcept {
    NTSTATUS status = STATUS_SUCCESS;
    ULONG_PTR information = 0;
    bool teardown = false;
    {
        Guard guard(state.guard);

        // lightning-lifecycle: before the last close leaves the removed lower
        // device, re-ask an unanswered held-lease count. A failed REMOVE-time
        // query is otherwise answered only by a client lease release, which a
        // client holding no lease never sends: the channel would stay forever.
        if (state.lower != nullptr && state.display_lost && state.retirement.foreign_opens() == 1 &&
            !state.retirement.leases_retired()) {
            observe_retained_leases(state);
        }

        if (state.lower != nullptr) {
            status = forward_wait(state, irp);
            information = irp->IoStatus.Information;
        }

        if (state.retirement.note_close()) {
            teardown = state.retirement.begin_teardown();
        }
    }

    finish(irp, status, information);

    if (teardown) {
        queue_retirement_teardown(state);
    }

    return status;
}

NTSTATUS forward_foreign(PDEVICE_OBJECT device, PIRP irp) noexcept {
    auto& state = *static_cast<DeviceState*>(device->DeviceExtension);
    const auto major = IoGetCurrentIrpStackLocation(irp)->MajorFunction;
    NTSTATUS status = IoAcquireRemoveLock(&state.remove_lock, irp);

    if (NT_SUCCESS(status)) {
        const auto* stack = IoGetCurrentIrpStackLocation(irp);

        if (major == IRP_MJ_CREATE || major == IRP_MJ_DEVICE_CONTROL) {
            // VideoPort OPEN calls HwInitialize, even before the first display
            // initialization. Its StartIO path also requires PASSIVE_LEVEL.
            // Hold the lifecycle mutex until actual lower completion: a phase
            // check without this join races D3/STOP resource retirement.
            if (KeGetCurrentIrql() != PASSIVE_LEVEL || guard_held_by_caller(state)) {
                IoReleaseRemoveLock(&state.remove_lock, irp);
                return finish(irp, STATUS_INVALID_DEVICE_STATE);
            }

            ULONG_PTR information = 0;
            {
                Guard guard(state.guard);
                const bool recognized = InterlockedCompareExchange(&state.display_identity, 0, 0) != 0;
                const bool active = state.lifecycle.accepts_work() && !state.display_quiesced &&
                    !state.display_lost && InterlockedCompareExchange(&state.power_overflow, 0, 0) == 0;
                DisplayRequest request = DisplayRequest::open;

                if (major == IRP_MJ_DEVICE_CONTROL) {
                    const ULONG code = stack->Parameters.DeviceIoControl.IoControlCode;
                    request = DisplayRequest::hardware;

                    if (code == IOCTL_VIDEO_QUERY_NUM_AVAIL_MODES || code == IOCTL_VIDEO_QUERY_AVAIL_MODES ||
                        code == IOCTL_VIDEO_QUERY_CURRENT_MODE || code == ioctl_query_cpu_scanout ||
                        code == ioctl_query_cursor) {
                        request = DisplayRequest::inventory;
                    } else if (code == IOCTL_VIDEO_UNMAP_VIDEO_MEMORY || code == ioctl_abandon_cpu_scanout ||
                               code == ioctl_release_owned_staging || code == ioctl_resolve_owned_staging) {
                        request = DisplayRequest::retirement;
                    }
                }

                const auto admission = display_admission(request, recognized, irp->RequestorMode == KernelMode, active);
                status = admission == DisplayAdmission::access_denied ? STATUS_ACCESS_DENIED : STATUS_DEVICE_NOT_READY;

                if (admission == DisplayAdmission::allow) {
                    // This completion halts lower unwinding; finish below owns
                    // the sole completion after releasing guard/remove lock.
                    ExternalAuthorization external{};
                    status = authorize_external_image(state, irp, external);

                    if (NT_SUCCESS(status) && external.answered) {
                        // async-present receipt query: answered here, never forwarded.
                        information = external.information;
                    } else if (NT_SUCCESS(status)) {
                        status = forward_wait(state, irp);
                        information = irp->IoStatus.Information;

                        // A lease release racing REMOVE (abandoned display,
                        // remove lock still open) still reports its count.
                        observe_resolve_receipt(state, irp, status, information);

                        if (major == IRP_MJ_CREATE && NT_SUCCESS(status)) {
                            // Count the win32k (or any foreign) open of the
                            // top-of-stack device so a later REMOVE knows a
                            // device reference is still outstanding and must be
                            // released before the retirement channel is deleted.
                            state.retirement.note_open();
                        }

                        if (external.image && !finish_external_image(state, irp, status, external)) {
                            // Source ownership is uncertain after possible GPU
                            // acceptance. Never permit CLOSE to destroy this
                            // host Vulkan allocation behind the imported image.
                            quarantine(state, __LINE__);
                        }
                    }

                    if (external.answered && state.renderer.quarantined()) {
                        // A receipt query found an unprovable pinned read.
                        quarantine(state, __LINE__);
                    }
                }
            }

            IoReleaseRemoveLock(&state.remove_lock, irp);
            return finish(irp, status, information);
        }

        if (major == IRP_MJ_CLOSE && KeGetCurrentIrql() == PASSIVE_LEVEL) {
            // A foreign file closed before any removal ends its counted open.
            Guard guard(state.guard);
            state.retirement.note_close();
        }

        return forward_locked(state, irp);
    }

    // After REMOVE kept this filter as the staging retirement channel, kernel
    // retirement requests and win32k's final CLOSE are admitted through the
    // retirement rundown before anything else here is read, and release it as
    // their last access: no device reference keeps this extension alive, and
    // retirement_teardown deletes it once the admitted count drains. Before
    // admission opens (and after teardown closes it) every request falls
    // through to the stable refusals below.
    if ((major == IRP_MJ_DEVICE_CONTROL || (major == IRP_MJ_CLOSE && KeGetCurrentIrql() == PASSIVE_LEVEL)) &&
        state.retirement_entry.acquire()) {
        status = major == IRP_MJ_DEVICE_CONTROL ? forward_retirement(state, irp) : close_retirement(state, irp);
        state.retirement_entry.release();
        return status;
    }

    if (major != IRP_MJ_CLEANUP && major != IRP_MJ_CLOSE) {
        return finish(irp, status);
    }

    // REMOVE may have retired ordinary work while still forwarding lower REMOVE.
    // Keep cleanup forwarding open until the unnamed filter is detached. The
    // explicit lower reference and this second lock survive every completion.
    status = IoAcquireRemoveLock(&state.cleanup_lock, irp);

    if (!NT_SUCCESS(status)) {
        return finish(irp, status);
    }

    if (major == IRP_MJ_CLOSE && KeGetCurrentIrql() == PASSIVE_LEVEL) {
        Guard guard(state.guard);
        state.retirement.note_close();
    }

    IoCopyCurrentIrpStackLocationToNext(irp);
    IoSetCompletionRoutine(irp, release_cleanup_completion, &state, TRUE, TRUE, TRUE);
    return IoCallDriver(state.lower, irp);
}

PDEVICE_OBJECT acquire_published_owner(PVOID tag) noexcept {
    KIRQL level;
    PDEVICE_OBJECT owner = nullptr;
    KeAcquireSpinLock(&notification_lock, &level);

    if (notification_device != nullptr) {
        auto& state = *static_cast<DeviceState*>(notification_device->DeviceExtension);

        if (NT_SUCCESS(IoAcquireRemoveLock(&state.remove_lock, tag))) {
            owner = notification_device;
            ObReferenceObject(owner);
        }
    }

    KeReleaseSpinLock(&notification_lock, level);
    return owner;
}

void unpublish_owner(PDEVICE_OBJECT owner) noexcept {
    KIRQL level;
    KeAcquireSpinLock(&notification_lock, &level);

    if (notification_device == owner) {
        notification_device = nullptr;
    }

    KeReleaseSpinLock(&notification_lock, level);
}

extern "C" NTSTATUS NTAPI dispatch_io(PDEVICE_OBJECT device, PIRP irp) {
    if (!is_control(device)) {
        // Foreign VideoPort FILE_OBJECT fields remain completely lower-owned.
        return forward_foreign(device, irp);
    }

    const auto* stack = IoGetCurrentIrpStackLocation(irp);
    PFILE_OBJECT file = stack->FileObject;

    if (KeGetCurrentIrql() != PASSIVE_LEVEL || file == nullptr) {
        return finish(irp, STATUS_INVALID_DEVICE_STATE);
    }

    const bool opening = stack->MajorFunction == IRP_MJ_CREATE;
    const bool closing = stack->MajorFunction == IRP_MJ_CLOSE;
    const bool cleanup = stack->MajorFunction == IRP_MJ_CLEANUP;

    if (opening && file->FsContext2 != nullptr) {
        return finish(irp, STATUS_INVALID_PARAMETER);
    }

    PDEVICE_OBJECT owner = opening ? acquire_published_owner(irp) : static_cast<PDEVICE_OBJECT>(file->FsContext2);

    if (owner == nullptr) {
        return finish(irp, closing || cleanup ? STATUS_SUCCESS : STATUS_DEVICE_NOT_READY);
    }

    auto& state = *static_cast<DeviceState*>(owner->DeviceExtension);

    if (closing || cleanup) {
        // The real control FILE_OBJECT keeps its control DO on the driver list,
        // pinning code through synchronous CLOSE. FsContext2 separately owns an
        // object reference to this exact old FDO extension after delete-pending.
        // CLEANUP keeps that reference; CLOSE releases it after the final access.
        {
            Guard guard(state.guard);
            state.renderer.close_file(file);

            if (state.renderer.quarantined() || state.transport.quarantined()) {
                quarantine(state, __LINE__);
            }
        }

        if (closing) {
            file->FsContext2 = nullptr;
            ObDereferenceObject(owner);
        }

        return finish(irp, STATUS_SUCCESS);
    }

    NTSTATUS status = opening ? STATUS_SUCCESS : IoAcquireRemoveLock(&state.remove_lock, irp);

    if (!NT_SUCCESS(status)) {
        return finish(irp, status);
    }

    if (guard_held_by_caller(state)) {
        // Reentrant control request on the guard owner's thread: refused, not nested.
        IoReleaseRemoveLock(&state.remove_lock, irp);

        if (opening) {
            ObDereferenceObject(owner);
        }

        return finish(irp, STATUS_INVALID_DEVICE_STATE);
    }

    ULONG_PTR information = 0;
    bool bound = false;
    {
        Guard guard(state.guard);
        status = STATUS_INVALID_DEVICE_REQUEST;

        if (!state.lifecycle.accepts_work() || !state.transport.ready() ||
            InterlockedCompareExchange(&state.power_overflow, 0, 0) != 0) {
            status = STATUS_DEVICE_NOT_READY;
        } else if (opening) {
            status = state.renderer.ready() ? state.renderer.open_file(file) : STATUS_SUCCESS;

            if (NT_SUCCESS(status)) {
                file->FsContext2 = owner;
                bound = true;
            }
        } else if (stack->MajorFunction == IRP_MJ_DEVICE_CONTROL &&
                   stack->Parameters.DeviceIoControl.IoControlCode == ioctl_query_capabilities) {
            if (stack->Parameters.DeviceIoControl.InputBufferLength != 0) {
                status = STATUS_INVALID_PARAMETER;
            } else if (irp->AssociatedIrp.SystemBuffer == nullptr ||
                       stack->Parameters.DeviceIoControl.OutputBufferLength < sizeof(LightningCapabilities)) {
                status = STATUS_BUFFER_TOO_SMALL;
            } else {
                auto* output = static_cast<LightningCapabilities*>(irp->AssociatedIrp.SystemBuffer);
                *output = state.transport.capabilities();

                if (state.renderer.ready()) {
                    output->flags |= capability_renderer_exchange | capability_exchange_batch;
                }

                information = sizeof(*output);
                status = STATUS_SUCCESS;
            }
        } else if (stack->MajorFunction == IRP_MJ_DEVICE_CONTROL &&
                   (stack->Parameters.DeviceIoControl.IoControlCode == ioctl_exchange ||
                    stack->Parameters.DeviceIoControl.IoControlCode == ioctl_exchange_batch) &&
                   irp->AssociatedIrp.SystemBuffer != nullptr) {
            size_t returned = 0;
            const rx7900::ByteView input(static_cast<const uint8_t*>(irp->AssociatedIrp.SystemBuffer),
                stack->Parameters.DeviceIoControl.InputBufferLength);
            const rx7900::sdma::CommandBuffer reply(static_cast<uint8_t*>(irp->AssociatedIrp.SystemBuffer),
                stack->Parameters.DeviceIoControl.OutputBufferLength);
            status = stack->Parameters.DeviceIoControl.IoControlCode == ioctl_exchange ?
                state.renderer.exchange_file(file, IoGetRequestorProcess(irp), input, reply, returned) :
                state.renderer.exchange_batch(file, IoGetRequestorProcess(irp), input, reply, returned);
            information = NT_SUCCESS(status) ? returned : 0;

            if (state.renderer.quarantined() || state.transport.quarantined()) {
                quarantine(state, __LINE__);
            }
        }
    }

    IoReleaseRemoveLock(&state.remove_lock, irp);

    if (opening && !bound) {
        ObDereferenceObject(owner);
    }

    return finish(irp, status, information);
}

extern "C" NTSTATUS NTAPI dispatch_passthrough(PDEVICE_OBJECT device, PIRP irp) {
    return is_control(device) ? finish(irp, STATUS_INVALID_DEVICE_REQUEST) : forward_foreign(device, irp);
}

// Owned power thread only. The original IRP/remove lock stay live through lower
// completion; no DispatchPower call waits for hardware or lower power progress.
// Every owned SET (power_route: device D0-D3 and the final system shutdown)
// arrives here in FIFO order. Power-down runs before the lower stack sees the
// request (a filter must not fail a legitimate SET, so an uncertain step
// contains instead); D0 resumes after the lower D0; a lower refusal of a
// prepared power-down rolls back, because the OS still owns the previous D0.
NTSTATUS process_device_power(DeviceState& state, PIRP irp) noexcept {
    const auto* stack = IoGetCurrentIrpStackLocation(irp);
    const bool system = stack->Parameters.Power.Type == SystemPowerState;
    const auto target = stack->Parameters.Power.State.DeviceState;
    const bool power_up = !system && target == PowerDeviceD0;
    bool prepared = false;

    if (!power_up) {
        Guard guard(state.guard);
        NativeOwners owners{state};
        prepared = run_power_down(state.lifecycle, owners);
        DbgPrint("LightningNT5: %s power-down %lu prepared %u status %08lx contained %u\n",
            system ? "system" : "device", system ? static_cast<ULONG>(stack->Parameters.Power.State.SystemState) :
            static_cast<ULONG>(target), prepared ? 1U : 0U, static_cast<ULONG>(owners.status),
            state.lifecycle.contained() ? 1U : 0U);
    }

    const NTSTATUS status = forward_wait(state, irp, true);

    if (NT_SUCCESS(status) ? power_up : prepared) {
        Guard guard(state.guard);
        NativeOwners owners{state};
        run_power_up(state.lifecycle, owners);
    }

    if (NT_SUCCESS(status) && !system) {
        POWER_STATE observed{};
        observed.DeviceState = target;
        PoSetPowerState(state.self, DevicePowerState, observed);
    }

    return status;
}

void start_next_power(PIRP irp) noexcept {
    const auto minor = IoGetCurrentIrpStackLocation(irp)->MinorFunction;

    if (minor == IRP_MN_QUERY_POWER || minor == IRP_MN_SET_POWER) {
        PoStartNextPowerIrp(irp);
    }
}

extern "C" VOID NTAPI power_thread_entry(PVOID context) {
    auto& state = *static_cast<DeviceState*>(context);

    for (;;) {
        KeWaitForSingleObject(&state.power_event, Executive, KernelMode, FALSE, nullptr);

        for (;;) {
            KIRQL level;
            PIRP irp = nullptr;
            KeAcquireSpinLock(&state.power_lock, &level);
            const bool available = state.power_queue.pop(irp);
            const bool stopping = state.power_queue.stopping();
            state.power_active = available;

            if (!available) {
                // The producer also signals under power_lock, preventing a
                // clear-after-signal race when the last accepted IRP retires.
                KeClearEvent(&state.power_event);
            }

            KeReleaseSpinLock(&state.power_lock, level);

            if (!available) {
                if (stopping) {
                    PsTerminateSystemThread(STATUS_SUCCESS);
                    return;
                }

                break;
            }

            if (InterlockedCompareExchange(&state.power_overflow, 0, 0) != 0) {
                Guard guard(state.guard);
                quarantine(state, __LINE__);
            }

            const NTSTATUS status = process_device_power(state, irp);

            if (InterlockedCompareExchange(&state.power_overflow, 0, 0) != 0) {
                Guard guard(state.guard);
                quarantine(state, __LINE__);
            }

            KeAcquireSpinLock(&state.power_lock, &level);
            state.power_active = false;
            KeReleaseSpinLock(&state.power_lock, level);
            // Device SET notification belongs after lower completion and local
            // resume. The thread and remove lock own the IRP until this point.
            start_next_power(irp);
            IoReleaseRemoveLock(&state.remove_lock, irp);
            finish(irp, status);
        }
    }
}

NTSTATUS start_power_thread(DeviceState& state) noexcept {
    OBJECT_ATTRIBUTES attributes;
    InitializeObjectAttributes(&attributes, nullptr, OBJ_KERNEL_HANDLE, nullptr, nullptr);
    return PsCreateSystemThread(&state.power_thread, THREAD_ALL_ACCESS, &attributes,
        nullptr, nullptr, power_thread_entry, &state);
}

// REMOVE calls this only after the primary remove-lock drain. All accepted IRPs
// have retired, and no subsequent producer can acquire that lock. Keep the FDO
// attached/on its driver list until the actual system thread has terminated.
NTSTATUS stop_power_thread(DeviceState& state) noexcept {
    if (state.power_thread == nullptr) {
        return STATUS_SUCCESS;
    }

    KIRQL level;
    KeAcquireSpinLock(&state.power_lock, &level);
    NT_ASSERT(state.power_queue.count() == 0 && !state.power_active);
    state.power_queue.stop();
    KeSetEvent(&state.power_event, IO_NO_INCREMENT, FALSE);
    KeReleaseSpinLock(&state.power_lock, level);
    const NTSTATUS status = ZwWaitForSingleObject(state.power_thread, FALSE, nullptr);

    if (status != STATUS_SUCCESS) {
        return NT_SUCCESS(status) ? STATUS_UNSUCCESSFUL : status;
    }

    const NTSTATUS closed = ZwClose(state.power_thread);

    if (!NT_SUCCESS(closed)) {
        return closed;
    }

    state.power_thread = nullptr;
    return STATUS_SUCCESS;
}

extern "C" NTSTATUS NTAPI release_power_completion(PDEVICE_OBJECT, PIRP irp, PVOID context) {
    auto& state = *static_cast<DeviceState*>(context);
    const auto* stack = IoGetCurrentIrpStackLocation(irp);

    if (stack->Parameters.Power.Type == DevicePowerState) {
        start_next_power(irp);
    }

    if (irp->PendingReturned) {
        IoMarkIrpPending(irp);
    }

    IoReleaseRemoveLock(&state.remove_lock, irp);
    return STATUS_CONTINUE_COMPLETION;
}

extern "C" NTSTATUS NTAPI dispatch_power(PDEVICE_OBJECT device, PIRP irp) {
    if (is_control(device)) {
        start_next_power(irp);
        return finish(irp, STATUS_INVALID_DEVICE_REQUEST);
    }

    auto& state = *static_cast<DeviceState*>(device->DeviceExtension);
    const auto* stack = IoGetCurrentIrpStackLocation(irp);
    NTSTATUS status = IoAcquireRemoveLock(&state.remove_lock, irp);

    if (!NT_SUCCESS(status)) {
        start_next_power(irp);
        return finish(irp, status);
    }

    const bool set = stack->MinorFunction == IRP_MN_SET_POWER;
    const bool query = stack->MinorFunction == IRP_MN_QUERY_POWER;
    const bool system = stack->Parameters.Power.Type == SystemPowerState;
    const ULONG requested = system ? static_cast<ULONG>(stack->Parameters.Power.State.SystemState) :
        static_cast<ULONG>(stack->Parameters.Power.State.DeviceState);
    const PowerRoute route = set || query ? power_route(set, system, requested) : PowerRoute::forward;

    if (route == PowerRoute::invalid) {
        IoReleaseRemoveLock(&state.remove_lock, irp);
        start_next_power(irp);
        return finish(irp, STATUS_INVALID_PARAMETER);
    }

    if (route == PowerRoute::owned) {
        KIRQL level;
        KeAcquireSpinLock(&state.power_lock, &level);
        const bool accepted = state.power_queue.push(irp);

        if (accepted) {
            IoMarkIrpPending(irp);
        } else {
            // No allocation can fail during legitimate SET handling: all slots
            // and the sole PASSIVE consumer exist before this FDO is published.
            // More than one active plus eight queued SETs is malformed kernel
            // overlap, not an advertised unlimited queue. Retain resources and
            // close local execution; never power down before quiesce by blindly
            // passing this exceptional overflow around the owned FIFO.
            InterlockedExchange(&state.power_overflow, 1);
        }

        KeSetEvent(&state.power_event, IO_NO_INCREMENT, FALSE);
        KeReleaseSpinLock(&state.power_lock, level);

        if (accepted) {
            return STATUS_PENDING;
        }

        IoReleaseRemoveLock(&state.remove_lock, irp);
        start_next_power(irp);
        return finish(irp, STATUS_DEVICE_BUSY);
    }

    if (query) {
        KIRQL level;
        KeAcquireSpinLock(&state.power_lock, &level);
        const bool busy = state.power_active || state.power_queue.count() != 0 || state.power_queue.stopping();
        KeReleaseSpinLock(&state.power_lock, level);

        if (busy) {
            status = STATUS_DEVICE_BUSY;
        } else if (route == PowerRoute::veto) {
            // Sleep and hibernation need a device power policy owner that
            // ReactOS videoprt does not provide (power_route). Veto QUERY, not
            // SET; shutdown is admitted and handled as the final power-down.
            status = STATUS_NOT_SUPPORTED;
        } else if (!system && requested > PowerDeviceD0 && KeGetCurrentIrql() == PASSIVE_LEVEL &&
                   !guard_held_by_caller(state)) {
            // lightning-pnp-veto: a device D1-D3 SET would contain while a
            // renderer session still maps borrowed views; refuse its QUERY.
            // A query at DISPATCH_LEVEL cannot take the guard and is
            // forwarded unchanged (ReactOS videoprt sends none).
            Guard guard(state.guard);
            const BorrowCensus census = state.renderer.borrow_census();

            if (borrow_vetoes_power_query(false, requested, census.sessions)) {
                status = STATUS_DEVICE_BUSY;
                DbgPrint("LightningNT5: device power query D%lu vetoed, %lu renderer session(s) hold %lu "
                    "borrowed view(s)\n", requested - 1, static_cast<ULONG>(census.sessions),
                    static_cast<ULONG>(census.views));
            }
        }

        if (!NT_SUCCESS(status)) {
            IoReleaseRemoveLock(&state.remove_lock, irp);
            start_next_power(irp);
            return finish(irp, status);
        }
    }

    if (stack->Parameters.Power.Type == SystemPowerState) {
        // NT5 filter ordering: system requests notify before forwarding;
        // device requests notify in completion, after lower processing.
        start_next_power(irp);
    }

    IoCopyCurrentIrpStackLocationToNext(irp);
    IoSetCompletionRoutine(irp, release_power_completion, &state, TRUE, TRUE, TRUE);
    return PoCallDriver(state.lower, irp);
}

// IRP_MN_REMOVE_DEVICE, entered with the primary remove lock held. Order:
// unpublish the notification owner; withdraw the link and control device;
// run_remove_owners (renderer, transport, display abandon); drain the
// primary remove lock; join the power thread; forward the lower REMOVE;
// detach (unless the display stays as retirement channel); drain the
// cleanup gate; free the reserved abandon IRP; then the lifecycle
// disposition. Every exit completes the IRP exactly once.
NTSTATUS remove_device(DeviceState& state, PDEVICE_OBJECT device, PIRP irp) noexcept {
    NTSTATUS status = STATUS_SUCCESS;
    // A queued control CREATE can no longer resolve an owner once unpublished.
    unpublish_owner(device);
    {
        // Link before its target device; then the run_remove_owners order.
        // The display abandon inside it is software only and precedes the
        // lower REMOVE, so the client transfers its views and leases.
        Guard guard(state.guard);
        remove_link(state);

        if (state.control != nullptr) {
            IoDeleteDevice(state.control);
            state.control = nullptr;
        }

        NativeOwners owners{state};
        run_remove_owners(state.lifecycle, owners);
    }

    // Keep the second gate open during the attached-but-ordinary-work-closed
    // interval. New lower file closes can still reach us until detach.
    status = IoAcquireRemoveLock(&state.cleanup_lock, irp);

    if (!NT_SUCCESS(status)) {
        Guard guard(state.guard);
        quarantine(state, __LINE__);
        IoReleaseRemoveLock(&state.remove_lock, irp);
        return finish(irp, status);
    }

    IoReleaseRemoveLockAndWait(&state.remove_lock, irp);
    status = stop_power_thread(state);

    if (!NT_SUCCESS(status)) {
        // A failed join cannot justify detaching or releasing storage that
        // the thread may still access. Retain the attached lower reference,
        // cleanup gate and image in explicit reboot-only containment.
        Guard guard(state.guard);
        quarantine(state, __LINE__);
        IoReleaseRemoveLock(&state.cleanup_lock, irp);
        return finish(irp, status);
    }

    irp->IoStatus.Status = STATUS_SUCCESS;
    status = forward_wait(state, irp);
    {
        // A display kept as retirement channel stays attached to its
        // removed lower device: win32k's final CLOSE of the display file
        // object goes to the top of that device's stack and must reach us.
        // An abandoned display may still hold staging leases, and any
        // outstanding foreign open (win32k keeps the top-of-stack pointer it
        // captured) would otherwise call a deleted filter: both keep the
        // channel. Without a recognized private ABI no lease can exist.
        Guard guard(state.guard);
        state.detach_pending = state.lower_referenced &&
            (state.display_lost || state.retirement.foreign_opens() != 0);

        if (state.detach_pending && !display_ever_recognized(state)) {
            state.retirement.observe_held_leases(0);
        }

        if (state.detach_pending) {
            // lightning-overlap: admit the retirement channel before the
            // cleanup gate below closes, so no CLOSE or lease resolve in
            // between is refused unforwarded and uncounted. The lower
            // REMOVE has completed; the gate stays non-retiring until
            // retain_for_retirement, which repeats begin_teardown.
            state.retirement_entry.open();
        }

        if (!state.detach_pending) {
            IoDetachDevice(state.lower);
        }

        state.attached = false;
    }

    IoReleaseRemoveLockAndWait(&state.cleanup_lock, irp);

    // The power thread is joined and no dispatch can reach the reserved
    // abandon IRP any more (the retirement channel forwards with its own).
    if (state.abandon_irp != nullptr) {
        IoFreeIrp(state.abandon_irp);
        state.abandon_irp = nullptr;
    }

    const RemoveDisposition disposition = state.lifecycle.remove_disposition(state.detach_pending);

    if (disposition == RemoveDisposition::retirement_channel) {
        return retain_for_retirement(state, device, irp, status);
    }

    if (state.lower_referenced) {
        ObDereferenceObject(state.lower);
        state.lower_referenced = false;
    }

    state.lower = nullptr;
    state.pdo = nullptr;

    if (disposition == RemoveDisposition::delete_device) {
        IoDeleteDevice(device);
    } else {
        // Detached, but the FDO and what containment retained (DMA,
        // adapter, borrowed views) stay until reboot; DriverUnload stays
        // cleared and contained_devices blocks any later re-arm.
        DbgPrint("LightningNT5: removed contained filter retained until reboot\n");
    }

    return finish(irp, status);
}

extern "C" NTSTATUS NTAPI dispatch_pnp(PDEVICE_OBJECT device, PIRP irp) {
    if (is_control(device)) {
        return finish(irp, STATUS_INVALID_DEVICE_REQUEST);
    }

    auto& state = *static_cast<DeviceState*>(device->DeviceExtension);
    const auto* stack = IoGetCurrentIrpStackLocation(irp);
    NTSTATUS status = IoAcquireRemoveLock(&state.remove_lock, irp);

    if (!NT_SUCCESS(status)) {
        return finish(irp, status);
    }

    if (KeGetCurrentIrql() != PASSIVE_LEVEL) {
        IoReleaseRemoveLock(&state.remove_lock, irp);
        return finish(irp, STATUS_INVALID_DEVICE_STATE);
    }

    switch (stack->MinorFunction) {
    case IRP_MN_START_DEVICE:
        status = forward_wait(state, irp);
        DbgPrint("LightningNT5: lower START_DEVICE status %08lx\n", static_cast<ULONG>(status));

        if (NT_SUCCESS(status)) {
            // The lower stack starts first; ours follows in the run_start order.
            Guard guard(state.guard);
            NativeOwners owners{state, stack->Parameters.StartDevice.AllocatedResources,
                stack->Parameters.StartDevice.AllocatedResourcesTranslated, true};
            const StartResult started = run_start(state.lifecycle, owners);

            if (started == StartResult::refused) {
                // Contained pages are never reused by a restart, and a
                // duplicate START keeps (and reports) the running device
                // instead of tearing it down.
                status = state.lifecycle.contained() ? STATUS_DEVICE_BUSY :
                    state.lifecycle.phase() == DevicePhase::started ? STATUS_SUCCESS : STATUS_INVALID_DEVICE_STATE;
                DbgPrint("LightningNT5: START refused, phase %u contained %u\n",
                    static_cast<unsigned>(state.lifecycle.phase()), state.lifecycle.contained() ? 1U : 0U);
            } else if (started == StartResult::failed) {
                status = NT_SUCCESS(owners.status) ? STATUS_UNSUCCESSFUL : owners.status;
            }
        }
        break;

    case IRP_MN_QUERY_STOP_DEVICE:
    case IRP_MN_QUERY_REMOVE_DEVICE:
        {
            // lightning-pnp-veto: a renderer session still mapping borrowed
            // views would make the STOP/REMOVE contain (reboot only). Refuse
            // the query before the lower stack sees it; the census is read
            // under the same guard that closes new work on admission.
            Guard guard(state.guard);
            const bool stop = stack->MinorFunction == IRP_MN_QUERY_STOP_DEVICE;
            const BorrowCensus census = state.renderer.borrow_census();
            const QueryAdmission admission = state.lifecycle.admit_query(stop, census.sessions);

            if (admission == QueryAdmission::borrowed) {
                status = STATUS_DEVICE_BUSY;
                DbgPrint("LightningNT5: query-%s vetoed, %lu renderer session(s) hold %lu borrowed view(s); "
                    "device stays started\n", stop ? "stop" : "remove", static_cast<ULONG>(census.sessions),
                    static_cast<ULONG>(census.views));
            } else if (admission == QueryAdmission::refused) {
                status = STATUS_DEVICE_BUSY;
                DbgPrint("LightningNT5: query-%s refused, phase %u contained %u\n", stop ? "stop" : "remove",
                    static_cast<unsigned>(state.lifecycle.phase()), state.lifecycle.contained() ? 1U : 0U);
            } else {
                DbgPrint("LightningNT5: query-%s admitted, no borrowed renderer views\n", stop ? "stop" : "remove");
            }
        }

        if (NT_SUCCESS(status)) {
            irp->IoStatus.Status = STATUS_SUCCESS;
            status = forward_wait(state, irp);

            if (!NT_SUCCESS(status)) {
                Guard guard(state.guard);
                state.lifecycle.end_query(stack->MinorFunction == IRP_MN_QUERY_STOP_DEVICE);
            }
        }
        break;

    case IRP_MN_CANCEL_STOP_DEVICE:
    case IRP_MN_CANCEL_REMOVE_DEVICE:
        irp->IoStatus.Status = STATUS_SUCCESS;
        status = forward_wait(state, irp);
        {
            // The PnP manager withdrew the query whatever the lower stack
            // answered; a stale pending phase would refuse all later work.
            Guard guard(state.guard);
            const bool stop = stack->MinorFunction == IRP_MN_CANCEL_STOP_DEVICE;
            state.lifecycle.end_query(stop);
            // After this filter's own veto the phase never left started.
            DbgPrint("LightningNT5: cancel-%s, phase %u contained %u\n", stop ? "stop" : "remove",
                static_cast<unsigned>(state.lifecycle.phase()), state.lifecycle.contained() ? 1U : 0U);
        }
        break;

    case IRP_MN_STOP_DEVICE:
        {
            Guard guard(state.guard);
            NativeOwners owners{state};
            status = run_stop(state.lifecycle, owners) ? STATUS_SUCCESS :
                (NT_SUCCESS(owners.status) ? STATUS_DEVICE_BUSY : owners.status);
        }

        if (NT_SUCCESS(status)) {
            irp->IoStatus.Status = STATUS_SUCCESS;
            status = forward_wait(state, irp);
        }
        break;

    case IRP_MN_SURPRISE_REMOVAL:
        {
            Guard guard(state.guard);
            NativeOwners owners{state};
            run_surprise(state.lifecycle, owners);
        }
        irp->IoStatus.Status = STATUS_SUCCESS;
        status = forward_wait(state, irp);
        break;

    case IRP_MN_REMOVE_DEVICE:
        return remove_device(state, device, irp);

    default:
        return forward_locked(state, irp);
    }

    IoReleaseRemoveLock(&state.remove_lock, irp);
    return finish(irp, status);
}

extern "C" NTSTATUS NTAPI add_device(PDRIVER_OBJECT driver, PDEVICE_OBJECT pdo) {
    DbgPrint("LightningNT5: AddDevice begin\n");
    if (KeGetCurrentIrql() != PASSIVE_LEVEL) {
        return STATUS_INVALID_DEVICE_STATE;
    }

    PDEVICE_OBJECT device = nullptr;
    NTSTATUS status = IoCreateDevice(driver, sizeof(DeviceState), nullptr,
        FILE_DEVICE_UNKNOWN, FILE_DEVICE_SECURE_OPEN, FALSE, &device);

    if (!NT_SUCCESS(status)) {
        return status;
    }

    auto* state = rx7900::reactos::construct_device_state<DeviceState>(device->DeviceExtension);
    state->self = device;
    state->pdo = pdo;
    IoInitializeRemoveLock(&state->remove_lock, static_cast<ULONG>(QLV_KERNEL_POOL_TAG), 0, 0);
    IoInitializeRemoveLock(&state->cleanup_lock, static_cast<ULONG>(QLV_KERNEL_POOL_TAG), 0, 0);
    KeInitializeMutex(&state->guard, 0);
    KeInitializeSpinLock(&state->power_lock);
    KeInitializeEvent(&state->power_event, NotificationEvent, FALSE);
    state->lower = IoAttachDeviceToDeviceStack(device, pdo);

    if (state->lower == nullptr) {
        IoDeleteDevice(device);
        return STATUS_NO_SUCH_DEVICE;
    }

    ObReferenceObject(state->lower);
    state->lower_referenced = true;
    state->attached = true;
    KeInitializeEvent(&state->abandon_event, NotificationEvent, FALSE);
    state->abandon_irp = IoAllocateIrp(state->lower->StackSize, FALSE);

    if (state->abandon_irp == nullptr) {
        IoDetachDevice(state->lower);
        ObDereferenceObject(state->lower);
        IoDeleteDevice(device);
        return STATUS_INSUFFICIENT_RESOURCES;
    }

    UNICODE_STRING name;
    RtlInitUnicodeString(&name, nt_device_name);
    status = IoCreateDevice(driver, sizeof(DeviceHeader), &name,
        FILE_DEVICE_UNKNOWN, FILE_DEVICE_SECURE_OPEN, FALSE, &state->control);

    if (!NT_SUCCESS(status)) {
        IoFreeIrp(state->abandon_irp);
        IoDetachDevice(state->lower);
        ObDereferenceObject(state->lower);
        IoDeleteDevice(device);
        return status;
    }

    auto* control = rx7900::reactos::construct_device_state<DeviceHeader>(state->control->DeviceExtension);
    control->role = DeviceRole::control;
    status = start_power_thread(*state);
    DbgPrint("LightningNT5: AddDevice power owner status %08lx\n", static_cast<ULONG>(status));

    if (!NT_SUCCESS(status)) {
        IoDeleteDevice(state->control);
        IoFreeIrp(state->abandon_irp);
        IoDetachDevice(state->lower);
        ObDereferenceObject(state->lower);
        IoDeleteDevice(device);
        return status;
    }

    state->control->Flags |= DO_BUFFERED_IO | DO_POWER_PAGABLE;
    state->control->Flags &= ~DO_DEVICE_INITIALIZING;
    device->Flags |= state->lower->Flags & (DO_BUFFERED_IO | DO_DIRECT_IO | DO_POWER_PAGABLE);
    device->DeviceType = state->lower->DeviceType;
    device->Flags &= ~DO_DEVICE_INITIALIZING;
    KIRQL level;
    KeAcquireSpinLock(&notification_lock, &level);
    notification_device = device;
    KeReleaseSpinLock(&notification_lock, level);
    return STATUS_SUCCESS;
}

extern "C" VOID NTAPI unload(PDRIVER_OBJECT driver) {
    // PnP REMOVE releases each clean FDO. Quarantine clears DriverUnload before
    // detaching and retains its device/resources; it cannot enter this routine,
    // and no retirement teardown re-arms it while any FDO is contained.
    NT_ASSERT(driver->DeviceObject == nullptr);
    NT_ASSERT(InterlockedCompareExchange(&contained_devices, 0, 0) == 0);

    if (notification_registered) {
        const NTSTATUS status = PsSetCreateProcessNotifyRoutine(process_notification, TRUE);
        NT_ASSERT(NT_SUCCESS(status));
        notification_registered = false;
    }
}

} // namespace

extern "C" NTSTATUS NTAPI DriverEntry(PDRIVER_OBJECT driver, PUNICODE_STRING) {
    if (KeGetCurrentIrql() != PASSIVE_LEVEL) {
        return STATUS_INVALID_DEVICE_STATE;
    }

    KeInitializeSpinLock(&notification_lock);
    const NTSTATUS tracking = PsSetCreateProcessNotifyRoutine(process_notification, FALSE);

    if (!NT_SUCCESS(tracking)) {
        return tracking;
    }

    notification_registered = true;
    DbgPrint("LightningNT5: DriverEntry ready\n");

    for (ULONG index = 0; index <= IRP_MJ_MAXIMUM_FUNCTION; ++index) {
        driver->MajorFunction[index] = dispatch_io;
    }

    driver->MajorFunction[IRP_MJ_PNP] = dispatch_pnp;
    driver->MajorFunction[IRP_MJ_POWER] = dispatch_power;
    driver->MajorFunction[IRP_MJ_SYSTEM_CONTROL] = dispatch_passthrough;
    driver->DriverExtension->AddDevice = add_device;
    driver->DriverUnload = unload;
    return STATUS_SUCCESS;
}

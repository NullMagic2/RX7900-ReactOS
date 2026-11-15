/*
Module: lightning.tests.lifecycle_native
Purpose: Run the production LightningNT5 PnP/power dispatch through full IRP sequences.
Created: 2026-10-25
Architecture: Exact NT SDK types. The production lightning_driver.cpp object is linked
unchanged and reached only through DriverEntry, its AddDevice and its dispatch table.
This file supplies a bounded I/O manager (IRP stack locations and completion routines,
remove locks, device stacks, object references, work items, buffered device-control
requests), dispatcher objects and system threads over real Win32 threads
(tests/lightning_lifecycle_native_platform.cpp, run under Wine), a modeled lower videoprt
stack answering the private display ABI, and NativeTransport/NativeRenderer owners whose
member functions replace lightning_transport.cpp/lightning_renderer.cpp. Each scenario
runs in its own child process because the driver keeps per-image globals; every wait is
bounded; and every fallible step of every scenario is failed in turn. The portable policy
is covered exhaustively by tests/lightning_lifecycle_model_tests.cpp.
*/
#include <stdint.h>
#include <ntifs.h>
#include <ntddvdeo.h>
#include "lightning_transport.hpp"
#include "lightning_renderer.hpp"
#include "lightning_present_abi.hpp"
#include "lightning_gdi_abi.hpp"
#include "lightning-display.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern "C" {
void* lifecycle_platform_thread(void (*routine)(void*), void* context);
int lifecycle_platform_join(void* thread, unsigned milliseconds);
void lifecycle_platform_close(void* thread);
void lifecycle_platform_exit_thread();
unsigned long lifecycle_platform_thread_id();
void lifecycle_platform_lock(int which);
void lifecycle_platform_unlock(int which);
int lifecycle_platform_wait(unsigned milliseconds);
void lifecycle_platform_broadcast();
void lifecycle_platform_sleep(unsigned milliseconds);
unsigned long long lifecycle_platform_ticks();
int lifecycle_platform_child(const char* arguments, unsigned milliseconds);
void lifecycle_platform_exit(int code);
NTSTATUS NTAPI DriverEntry(PDRIVER_OBJECT, PUNICODE_STRING);
}

// Opaque in the DDK; owned entirely by this fixture's I/O manager.
struct _IO_WORKITEM {
    PDEVICE_OBJECT device;
    PIO_WORKITEM_ROUTINE routine;
    PVOID context;
};

namespace {
using namespace lightning::reactos;

constexpr unsigned wait_milliseconds = 10000;
constexpr int exit_pass = 0;
constexpr int exit_fail = 1;
constexpr int exit_hang = 3;
constexpr int exit_sweep_done = 7;
constexpr unsigned no_fault = 0xffffffffu;
constexpr int dispatcher_lock = 0;
constexpr int spin_lock = 1;

unsigned checks = 0;
unsigned failures = 0;
bool verbose = false;

void check(bool condition, const char* description) {
    ++checks;

    if (!condition) {
        ++failures;
        printf("FAIL %s\n", description);
    }
}

[[noreturn]] void die(const char* reason) {
    printf("FATAL %s\n", reason);
    lifecycle_platform_exit(exit_hang);
    for (;;) {
    }
}

struct Lock final {
    explicit Lock(int which) : which_(which) { lifecycle_platform_lock(which_); }
    ~Lock() { lifecycle_platform_unlock(which_); }
    Lock(const Lock&) = delete;
    Lock& operator=(const Lock&) = delete;
    int which_;
};

// ---- fault injection: the n-th fallible step fails (one per run).
enum class Fault : uint8_t {
    create_device, allocate_irp, create_thread, lower_start, transport_start, ioctl_query, ioctl_resume,
    link_create, ioctl_quiesce, renderer_shutdown, transport_suspend, transport_resume, transport_stop,
    lower_query, lower_d3, ioctl_abandon, ioctl_resolve, work_item, thread_join
};

unsigned fault_index = 0;
unsigned fault_at = no_fault;
bool fault_hit = false;
// A scenario may also fail the next step of one kind, outside the sweep.
unsigned forced_active = 0;
Fault forced = Fault::create_device;

bool inject(Fault fault) {
    Lock lock(dispatcher_lock);

    if (forced_active != 0 && fault == forced) {
        --forced_active;
        return true;
    }

    const bool hit = fault_index++ == fault_at;
    fault_hit = fault_hit || hit;
    return hit;
}

void force_next(Fault fault, unsigned count = 1) {
    Lock lock(dispatcher_lock);
    forced = fault;
    forced_active = count;
}

// ---- event log of owner and lower-stack operations.
enum class Op : uint8_t {
    transport_start, transport_resume, transport_suspend, transport_stop_hardware, transport_stop_quiet,
    renderer_start, renderer_stop_host, renderer_stop_quiet, lower_pnp, lower_power, lower_ioctl,
    containment, unload, receipts_retired
};

struct Entry final { Op op; uint32_t a; uint32_t b; };
Entry entries[2048]{};
unsigned entry_count = 0;
bool contained_seen = false;
unsigned hardware_after_containment = 0;

bool hardware_op(Op op, uint32_t a) {
    switch (op) {
    case Op::transport_start: case Op::transport_resume: case Op::transport_suspend:
    case Op::transport_stop_hardware: case Op::renderer_stop_host:
        return true;
    case Op::lower_ioctl:
        return a == ioctl_query_cpu_scanout || a == ioctl_quiesce_cpu_scanout || a == ioctl_resume_cpu_scanout ||
            a == ioctl_present_cpu_scanout;
    default:
        return false;
    }
}

void record(Op op, uint32_t a = 0, uint32_t b = 0) {
    Lock lock(dispatcher_lock);

    if (contained_seen && hardware_op(op, a)) {
        ++hardware_after_containment;
        printf("note: hardware op %u (%08x) after containment\n", static_cast<unsigned>(op), a);
    }

    if (entry_count < sizeof(entries) / sizeof(entries[0])) {
        entries[entry_count] = {op, a, b};
    }

    ++entry_count;
}

unsigned find(Op op, uint32_t a, unsigned from = 0) {
    for (unsigned index = from; index < entry_count && index < sizeof(entries) / sizeof(entries[0]); ++index) {
        if (entries[index].op == op && (a == 0xffffffffu || entries[index].a == a)) {
            return index;
        }
    }

    return 0xffffffffu;
}

unsigned count_ops(Op op, uint32_t a = 0xffffffffu, unsigned from = 0) {
    unsigned total = 0;

    for (unsigned index = from; index < entry_count && index < sizeof(entries) / sizeof(entries[0]); ++index) {
        total += entries[index].op == op && (a == 0xffffffffu || entries[index].a == a) ? 1 : 0;
    }

    return total;
}

char log_text[1 << 17]{};
size_t log_used = 0;

bool logged(const char* text) {
    Lock lock(dispatcher_lock);
    return strstr(log_text, text) != nullptr;
}

// ---- tracked objects (device objects, the fake process) and their references.
struct Tracked final {
    void* object;
    long references;
    bool deleted;
    bool listed;
    size_t extension_bytes;
};

Tracked tracked[32]{};
unsigned tracked_count = 0;
unsigned asserts = 0;

Tracked* lookup(void* object) {
    for (unsigned index = 0; index < tracked_count; ++index) {
        if (tracked[index].object == object) {
            return &tracked[index];
        }
    }

    if (tracked_count == sizeof(tracked) / sizeof(tracked[0])) {
        die("object table full");
    }

    tracked[tracked_count] = {object, 0, false, false, 0};
    return &tracked[tracked_count++];
}

PDRIVER_OBJECT filter_driver = nullptr;
PDRIVER_OBJECT pci_driver = nullptr;
PDRIVER_OBJECT video_driver = nullptr;
PDEVICE_OBJECT pdo = nullptr;
PDEVICE_OBJECT lower_fdo = nullptr;
PDEVICE_OBJECT named_device = nullptr;
unsigned unload_invoked = 0;
unsigned detaches = 0;

void poison_if_released(Tracked& entry) {
    auto* device = static_cast<PDEVICE_OBJECT>(entry.object);

    if (entry.deleted && entry.references == 0 && device->ReferenceCount == 0 && !entry.listed &&
        entry.extension_bytes != 0) {
        // A later access through a released extension now reads garbage.
        memset(device->DeviceExtension, 0xdd, entry.extension_bytes);
        entry.extension_bytes = 0;
    }
}

// IopUnloadDevice: a delete-pending device leaves its driver list once it has
// no attached device and no open count; an emptied list unloads the driver.
void unload_check(PDEVICE_OBJECT device) {
    Tracked* entry = nullptr;
    PDRIVER_OBJECT driver = device->DriverObject;
    bool call_unload = false;
    {
        Lock lock(dispatcher_lock);
        entry = lookup(device);

        if (!entry->deleted || device->AttachedDevice != nullptr || device->ReferenceCount != 0) {
            return;
        }

        if (entry->listed) {
            PDEVICE_OBJECT* link = &driver->DeviceObject;

            while (*link != nullptr && *link != device) {
                link = &(*link)->NextDevice;
            }

            if (*link == device) {
                *link = device->NextDevice;
            }

            entry->listed = false;
        }

        call_unload = driver == filter_driver && driver->DeviceObject == nullptr &&
            driver->DriverUnload != nullptr && unload_invoked == 0;

        if (call_unload) {
            ++unload_invoked;
        }
    }

    if (call_unload) {
        record(Op::unload);
        driver->DriverUnload(driver);
    }

    Lock lock(dispatcher_lock);
    poison_if_released(*entry);
}

// ---- IRP accounting.
struct Sent final {
    PIRP irp;
    unsigned completions;
    unsigned start_next;
    bool power;
};

Sent sent[128]{};
unsigned sent_count = 0;
unsigned built_allocated = 0;
unsigned built_freed = 0;
unsigned driver_irps_allocated = 0;
unsigned driver_irps_freed = 0;
PIRP built[32]{};

Sent* sent_record(PIRP irp) {
    for (unsigned index = 0; index < sent_count; ++index) {
        if (sent[index].irp == irp) {
            return &sent[index];
        }
    }

    return nullptr;
}

PIRP raw_irp(CCHAR stack_size) {
    const size_t bytes = sizeof(IRP) + static_cast<size_t>(stack_size) * sizeof(IO_STACK_LOCATION);
    auto* irp = static_cast<PIRP>(calloc(1, bytes));

    if (irp == nullptr) {
        die("out of memory");
    }

    irp->Type = IO_TYPE_IRP;
    irp->Size = static_cast<USHORT>(bytes);
    irp->StackCount = stack_size;
    irp->CurrentLocation = static_cast<CHAR>(stack_size + 1);
    irp->Tail.Overlay.CurrentStackLocation = reinterpret_cast<PIO_STACK_LOCATION>(irp + 1) + stack_size;
    return irp;
}

bool built_irp(PIRP irp, bool remove) {
    for (auto& entry : built) {
        if (entry == irp) {
            if (remove) {
                entry = nullptr;
            }

            return true;
        }
    }

    return false;
}

// The I/O manager's final completion of a packet that left its top location.
void final_completion(PIRP irp) {
    if (built_irp(irp, true)) {
        if (NT_SUCCESS(irp->IoStatus.Status) && irp->UserBuffer != nullptr &&
            irp->AssociatedIrp.SystemBuffer != nullptr) {
            memcpy(irp->UserBuffer, irp->AssociatedIrp.SystemBuffer, irp->IoStatus.Information);
        }

        *irp->UserIosb = irp->IoStatus;
        PKEVENT event = irp->UserEvent;
        free(irp->AssociatedIrp.SystemBuffer);
        free(irp);
        {
            Lock lock(dispatcher_lock);
            ++built_freed;
        }
        KeSetEvent(event, IO_NO_INCREMENT, FALSE);
        return;
    }

    Lock lock(dispatcher_lock);
    Sent* record = sent_record(irp);

    if (record == nullptr) {
        die("an IRP the fixture does not own reached final completion");
    }

    ++record->completions;
    lifecycle_platform_broadcast();
}

// ---- lower videoprt stack and the private display ABI.
struct LowerConfig final {
    bool display = true;
    bool pend_power = false;
    bool abandoned = false;
    uint32_t held_leases = 0;
    // Resolve answers after the abandon: zero means the filter never learned
    // the held-lease count (every query and release failed).
    unsigned answered_after_abandon = 0;
} lower;

void* helper_threads[64]{};
unsigned helper_count = 0;

void keep_helper(void* thread) {
    Lock lock(dispatcher_lock);

    if (thread == nullptr || helper_count == sizeof(helper_threads) / sizeof(helper_threads[0])) {
        die("helper thread table");
    }

    helper_threads[helper_count++] = thread;
}

void late_completion(void* context) {
    lifecycle_platform_sleep(2);
    IofCompleteRequest(static_cast<PIRP>(context), IO_NO_INCREMENT);
}

NTSTATUS lower_control(PIRP irp, const IO_STACK_LOCATION& stack, ULONG_PTR& information) {
    const ULONG code = stack.Parameters.DeviceIoControl.IoControlCode;
    void* buffer = irp->AssociatedIrp.SystemBuffer;
    record(Op::lower_ioctl, code);

    if (code == ioctl_query_cpu_scanout) {
        if (!lower.display) {
            return STATUS_NOT_SUPPORTED;
        }

        if (inject(Fault::ioctl_query)) {
            return STATUS_DEVICE_NOT_READY;
        }

        CpuScanoutInformation output{};
        output.size = sizeof(output);
        output.version = cpu_scanout_abi_version;
        output.magic = LIGHTNING_DISPLAY_MAGIC;
        output.width = 1024;
        output.height = 768;
        output.stride = 4096;
        output.mapped_bytes = 4096 * 768;
        output.enabled = 1;
        check(buffer != nullptr && stack.Parameters.DeviceIoControl.OutputBufferLength == sizeof(output),
            "display discovery is a buffered private query");
        memcpy(buffer, &output, sizeof(output));
        information = sizeof(output);
        return STATUS_SUCCESS;
    }

    if (code == ioctl_quiesce_cpu_scanout) {
        return inject(Fault::ioctl_quiesce) ? STATUS_DEVICE_PROTOCOL_ERROR : STATUS_SUCCESS;
    }

    if (code == ioctl_resume_cpu_scanout) {
        return inject(Fault::ioctl_resume) ? STATUS_DEVICE_PROTOCOL_ERROR : STATUS_SUCCESS;
    }

    if (code == ioctl_abandon_cpu_scanout) {
        if (inject(Fault::ioctl_abandon)) {
            return STATUS_DEVICE_PROTOCOL_ERROR;
        }

        lower.abandoned = true;
        return STATUS_SUCCESS;
    }

    if (code == ioctl_resolve_owned_staging) {
        if (buffer == nullptr || stack.Parameters.DeviceIoControl.InputBufferLength != sizeof(StagingResolveRequest) ||
            stack.Parameters.DeviceIoControl.OutputBufferLength != sizeof(StagingResolveReceipt)) {
            return STATUS_INVALID_PARAMETER;
        }

        if (inject(Fault::ioctl_resolve)) {
            return STATUS_DEVICE_NOT_READY;
        }

        lower.answered_after_abandon += lower.abandoned ? 1 : 0;

        StagingResolveRequest request{};
        memcpy(&request, buffer, sizeof(request));
        StagingResolveReceipt receipt{};
        receipt.size = sizeof(receipt);
        receipt.version = cpu_scanout_abi_version;
        receipt.lease_generation = request.lease_generation;
        receipt.issued_generation = request.lease_generation;
        receipt.flags = lower.abandoned ? staging_flag_device_lost : 0;

        if (request.operation == staging_resolve_release) {
            receipt.outcome = lower.held_leases != 0 ? staging_lease_released : staging_lease_not_held;
            lower.held_leases -= lower.held_leases != 0 ? 1 : 0;
        } else {
            receipt.outcome = lower.held_leases != 0 ? staging_lease_held : staging_lease_not_held;
        }

        receipt.held_leases = lower.held_leases;
        memcpy(buffer, &receipt, sizeof(receipt));
        information = sizeof(receipt);
        return STATUS_SUCCESS;
    }

    return STATUS_SUCCESS;
}

NTSTATUS NTAPI lower_dispatch(PDEVICE_OBJECT, PIRP irp) {
    const auto* stack = IoGetCurrentIrpStackLocation(irp);
    NTSTATUS status = STATUS_SUCCESS;
    ULONG_PTR information = 0;

    switch (stack->MajorFunction) {
    case IRP_MJ_PNP:
        record(Op::lower_pnp, stack->MinorFunction);

        if (stack->MinorFunction == IRP_MN_START_DEVICE && inject(Fault::lower_start)) {
            status = STATUS_UNSUCCESSFUL;
        } else if ((stack->MinorFunction == IRP_MN_QUERY_STOP_DEVICE ||
                    stack->MinorFunction == IRP_MN_QUERY_REMOVE_DEVICE) && inject(Fault::lower_query)) {
            status = STATUS_UNSUCCESSFUL;
        } else if (stack->MinorFunction > IRP_MN_SURPRISE_REMOVAL) {
            status = irp->IoStatus.Status;
        }
        break;

    case IRP_MJ_POWER: {
        const bool system = stack->Parameters.Power.Type == SystemPowerState;
        const ULONG state = system ? static_cast<ULONG>(stack->Parameters.Power.State.SystemState) :
            static_cast<ULONG>(stack->Parameters.Power.State.DeviceState);
        record(Op::lower_power, stack->MinorFunction, (system ? 0x100U : 0U) | state);

        if (stack->MinorFunction == IRP_MN_SET_POWER && !system && state != PowerDeviceD0 &&
            inject(Fault::lower_d3)) {
            status = STATUS_UNSUCCESSFUL;
        }

        if (lower.pend_power) {
            irp->IoStatus.Status = status;
            IoMarkIrpPending(irp);
            keep_helper(lifecycle_platform_thread(late_completion, irp));
            return STATUS_PENDING;
        }
        break;
    }

    case IRP_MJ_DEVICE_CONTROL:
        status = lower_control(irp, *stack, information);
        break;

    default:
        break;
    }

    irp->IoStatus.Status = status;
    irp->IoStatus.Information = information;
    IofCompleteRequest(irp, IO_NO_INCREMENT);
    return status;
}

// ---- remove locks.
struct RemoveLockRecord final { PIO_REMOVE_LOCK lock; bool drained; LONG drained_count; };
RemoveLockRecord remove_locks[16]{};
unsigned remove_lock_count = 0;

// ---- threads, work items and the process notification.
struct SystemThread final { PKSTART_ROUTINE routine; PVOID context; void* thread; bool joined; bool closed; };
SystemThread system_threads[8]{};
unsigned system_thread_count = 0;
unsigned work_allocated = 0;
unsigned work_freed = 0;
PCREATE_PROCESS_NOTIFY_ROUTINE notify_routine = nullptr;
unsigned notify_registered = 0;
unsigned notify_unregistered = 0;
bool link_live = false;
unsigned links_created = 0;
unsigned links_deleted = 0;
int process_storage[2]{};
PEPROCESS fake_process = reinterpret_cast<PEPROCESS>(&process_storage[0]);

void system_thread_trampoline(void* context) {
    auto& thread = *static_cast<SystemThread*>(context);
    thread.routine(thread.context);
}

void work_trampoline(void* context) {
    auto* item = static_cast<PIO_WORKITEM>(context);
    PDEVICE_OBJECT device = item->device;
    item->routine(device, item->context);
    ObfDereferenceObject(device);
    unload_check(device);
}

// ---- fake owners: what lightning_driver.cpp sees of transport and renderer.
unsigned transport_acquired = 0;
unsigned transport_released = 0;
bool start_fault_quarantines = false;
bool renderer_views = false;
bool exit_quarantines = false;
} // namespace

namespace lightning::reactos {

NTSTATUS NativeTransport::start(PDEVICE_OBJECT device, PCM_RESOURCE_LIST raw, PCM_RESOURCE_LIST translated) noexcept {
    record(Op::transport_start);
    check(device == pdo && raw != nullptr && translated != nullptr, "transport start receives the PDO and resources");

    if (bus_acquired_ || quarantined_) {
        return STATUS_INVALID_DEVICE_STATE;
    }

    if (inject(Fault::transport_start)) {
        quarantined_ = start_fault_quarantines;
        return STATUS_DEVICE_NOT_READY;
    }

    bus_acquired_ = true;
    ready_ = true;
    capabilities_.size = sizeof(capabilities_);
    ++transport_acquired;
    return STATUS_SUCCESS;
}

NTSTATUS NativeTransport::stop(bool allow_mmio) noexcept {
    record(allow_mmio ? Op::transport_stop_hardware : Op::transport_stop_quiet);

    if (quarantined_) {
        return STATUS_DEVICE_BUSY;
    }

    if (bus_acquired_ && inject(Fault::transport_stop)) {
        quarantined_ = true;
        ready_ = false;
        return STATUS_DEVICE_BUSY;
    }

    ready_ = false;

    if (bus_acquired_) {
        bus_acquired_ = false;
        ++transport_released;
    }

    return STATUS_SUCCESS;
}

NTSTATUS NativeTransport::suspend() noexcept {
    record(Op::transport_suspend);

    if (quarantined_) {
        return STATUS_DEVICE_BUSY;
    }

    if (inject(Fault::transport_suspend)) {
        quarantined_ = true;
        ready_ = false;
        return STATUS_DEVICE_BUSY;
    }

    ready_ = false;
    return STATUS_SUCCESS;
}

NTSTATUS NativeTransport::resume() noexcept {
    record(Op::transport_resume);

    if (quarantined_ || ready_ || !bus_acquired_) {
        return STATUS_INVALID_DEVICE_STATE;
    }

    if (inject(Fault::transport_resume)) {
        return STATUS_DEVICE_NOT_READY;
    }

    ready_ = true;
    return STATUS_SUCCESS;
}

NTSTATUS NativeRenderer::initialize(NativeTransport& transport) noexcept {
    record(Op::renderer_start);

    if (quarantined_ || !transport.ready()) {
        return STATUS_DEVICE_NOT_READY;
    }

    transport_ = &transport;
    active_ = true;
    return STATUS_SUCCESS;
}

NTSTATUS NativeRenderer::shutdown(bool allow_host) noexcept {
    record(allow_host ? Op::renderer_stop_host : Op::renderer_stop_quiet);
    active_ = false;

    if (renderer_views || inject(Fault::renderer_shutdown)) {
        quarantined_ = true;
    }

    return quarantined_ ? STATUS_DEVICE_BUSY : STATUS_SUCCESS;
}

// One GL process mapping its ring and one blob, as shutdown() would find it.
BorrowCensus NativeRenderer::borrow_census() const noexcept {
    return renderer_views ? BorrowCensus{1, 2} : BorrowCensus{};
}

NTSTATUS NativeRenderer::open_file(PFILE_OBJECT) noexcept { return ready() ? STATUS_SUCCESS : STATUS_DEVICE_NOT_READY; }
NTSTATUS NativeRenderer::close_file(PFILE_OBJECT) noexcept { return STATUS_SUCCESS; }

NTSTATUS NativeRenderer::exchange_file(PFILE_OBJECT, PEPROCESS, rx7900::ByteView, rx7900::sdma::CommandBuffer,
                                       size_t& written) noexcept {
    written = 0;
    return STATUS_NOT_SUPPORTED;
}

NTSTATUS NativeRenderer::exchange_batch(PFILE_OBJECT, PEPROCESS, rx7900::ByteView, rx7900::sdma::CommandBuffer,
                                        size_t& written) noexcept {
    written = 0;
    return STATUS_NOT_SUPPORTED;
}

NTSTATUS NativeRenderer::resolve_image(uint64_t, PEPROCESS, NativeImageLease&, const ProducerPoint*) noexcept {
    return STATUS_NOT_SUPPORTED;
}

bool NativeRenderer::track_receipt(uint64_t, PEPROCESS, const ImageReceipt&) noexcept { return false; }
bool NativeRenderer::can_track_receipt(uint64_t, PEPROCESS) const noexcept { return false; }
ReceiptState NativeRenderer::query_receipt(uint64_t, PEPROCESS) noexcept { return ReceiptState::retired; }
void NativeRenderer::retire_receipts_after_display_reset() noexcept { record(Op::receipts_retired); }

void NativeRenderer::process_exit(HANDLE) noexcept {
    if (exit_quarantines) {
        quarantined_ = true;
    } else {
        // The exit notification revokes the exiting process's views.
        renderer_views = false;
    }
}

} // namespace lightning::reactos

// ---- kernel exports used by the production object.
extern "C" {

ULONG __cdecl DbgPrint(PCSTR format, ...) {
    char line[512];
    va_list arguments;
    va_start(arguments, format);
    vsnprintf(line, sizeof(line), format, arguments);
    va_end(arguments);
    {
        Lock lock(dispatcher_lock);
        const size_t length = strlen(line);

        if (log_used + length + 1 < sizeof(log_text)) {
            memcpy(log_text + log_used, line, length + 1);
            log_used += length;
        }

        if (strncmp(line, "LightningNT5: containment cause line", 36) == 0) {
            contained_seen = true;
        }
    }

    if (verbose) {
        printf("  dbg: %s", line);
    }

    return 0;
}

VOID NTAPI RtlAssert(PVOID assertion, PVOID file, ULONG line, PSTR) {
    ++asserts;
    printf("ASSERT %s at %s:%lu\n", static_cast<const char*>(assertion), static_cast<const char*>(file), line);
}

VOID NTAPI RtlInitUnicodeString(PUNICODE_STRING target, PCWSTR source) {
    size_t length = 0;

    while (source != nullptr && source[length] != 0) {
        ++length;
    }

    target->Buffer = const_cast<PWSTR>(source);
    target->Length = static_cast<USHORT>(length * sizeof(WCHAR));
    target->MaximumLength = static_cast<USHORT>(target->Length + sizeof(WCHAR));
}

KIRQL NTAPI KeGetCurrentIrql(VOID) { return PASSIVE_LEVEL; }

PKTHREAD NTAPI KeGetCurrentThread(VOID) {
    return reinterpret_cast<PKTHREAD>(static_cast<uintptr_t>(lifecycle_platform_thread_id()) * 16 + 0x10000);
}

// Dispatcher objects: one fixture lock and condition; bounded waits.
VOID NTAPI KeInitializeEvent(PRKEVENT event, EVENT_TYPE type, BOOLEAN state) {
    memset(event, 0, sizeof(*event));
    event->Header.Type = type == NotificationEvent ? 0 : 1;
    event->Header.SignalState = state ? 1 : 0;
}

LONG NTAPI KeSetEvent(PRKEVENT event, KPRIORITY, BOOLEAN) {
    Lock lock(dispatcher_lock);
    const LONG previous = event->Header.SignalState;
    event->Header.SignalState = 1;
    lifecycle_platform_broadcast();
    return previous;
}

VOID NTAPI KeClearEvent(PRKEVENT event) {
    Lock lock(dispatcher_lock);
    event->Header.SignalState = 0;
}

VOID NTAPI KeInitializeMutex(PRKMUTEX mutex, ULONG) {
    memset(mutex, 0, sizeof(*mutex));
    mutex->Header.Type = 2;
    mutex->Header.SignalState = 1;
}

void wait_or_die(unsigned long long started) {
    const unsigned long long now = lifecycle_platform_ticks();

    if (now - started >= wait_milliseconds) {
        die("dispatcher wait exceeded its bound (deadlock)");
    }

    lifecycle_platform_wait(static_cast<unsigned>(wait_milliseconds - (now - started)));
}

NTSTATUS NTAPI KeWaitForSingleObject(PVOID object, KWAIT_REASON, KPROCESSOR_MODE, BOOLEAN, PLARGE_INTEGER timeout) {
    check(timeout == nullptr, "lifecycle waits are untimed kernel waits");
    auto* header = static_cast<DISPATCHER_HEADER*>(object);
    const unsigned long long started = lifecycle_platform_ticks();
    Lock lock(dispatcher_lock);

    if (header->Type == 2) {
        auto* mutex = static_cast<PRKMUTEX>(object);
        const PKTHREAD self = KeGetCurrentThread();

        if (mutex->OwnerThread == self) {
            --mutex->Header.SignalState;
            return STATUS_SUCCESS;
        }

        while (mutex->Header.SignalState <= 0) {
            wait_or_die(started);
        }

        mutex->Header.SignalState = 0;
        mutex->OwnerThread = self;
        return STATUS_SUCCESS;
    }

    while (header->SignalState == 0) {
        wait_or_die(started);
    }

    if (header->Type == 1) {
        header->SignalState = 0;
    }

    return STATUS_SUCCESS;
}

LONG NTAPI KeReleaseMutex(PRKMUTEX mutex, BOOLEAN) {
    Lock lock(dispatcher_lock);
    check(mutex->OwnerThread == KeGetCurrentThread(), "only the owner releases the lifecycle mutex");
    const LONG previous = mutex->Header.SignalState;

    if (++mutex->Header.SignalState == 1) {
        mutex->OwnerThread = nullptr;
    }

    lifecycle_platform_broadcast();
    return previous;
}

NTSTATUS NTAPI KeDelayExecutionThread(KPROCESSOR_MODE, BOOLEAN, PLARGE_INTEGER) {
    lifecycle_platform_sleep(1);
    return STATUS_SUCCESS;
}

KIRQL FASTCALL KfAcquireSpinLock(PKSPIN_LOCK) {
    lifecycle_platform_lock(spin_lock);
    return PASSIVE_LEVEL;
}

VOID FASTCALL KfReleaseSpinLock(PKSPIN_LOCK, KIRQL) {
    lifecycle_platform_unlock(spin_lock);
}

// Remove locks (the documented IO_REMOVE_LOCK common-block protocol).
VOID NTAPI IoInitializeRemoveLockEx(PIO_REMOVE_LOCK lock, ULONG, ULONG, ULONG, ULONG size) {
    check(size == sizeof(IO_REMOVE_LOCK), "remove lock size matches the SDK");
    memset(lock, 0, sizeof(*lock));
    lock->Common.IoCount = 1;
    KeInitializeEvent(&lock->Common.RemoveEvent, NotificationEvent, FALSE);
    Lock guard(dispatcher_lock);

    if (remove_lock_count < sizeof(remove_locks) / sizeof(remove_locks[0])) {
        remove_locks[remove_lock_count++] = {lock, false, 0};
    }
}

NTSTATUS NTAPI IoAcquireRemoveLockEx(PIO_REMOVE_LOCK lock, PVOID, PCSTR, ULONG, ULONG) {
    Lock guard(dispatcher_lock);
    ++lock->Common.IoCount;

    if (lock->Common.Removed) {
        if (--lock->Common.IoCount == 0) {
            KeSetEvent(&lock->Common.RemoveEvent, IO_NO_INCREMENT, FALSE);
        }

        return STATUS_DELETE_PENDING;
    }

    return STATUS_SUCCESS;
}

VOID NTAPI IoReleaseRemoveLockEx(PIO_REMOVE_LOCK lock, PVOID, ULONG) {
    Lock guard(dispatcher_lock);

    if (lock->Common.IoCount <= 0) {
        check(false, "remove lock released more often than acquired");
        return;
    }

    if (--lock->Common.IoCount == 0) {
        KeSetEvent(&lock->Common.RemoveEvent, IO_NO_INCREMENT, FALSE);
    }
}

VOID NTAPI IoReleaseRemoveLockAndWaitEx(PIO_REMOVE_LOCK lock, PVOID, ULONG) {
    {
        Lock guard(dispatcher_lock);
        check(!lock->Common.Removed, "a remove lock is drained once");
        lock->Common.Removed = TRUE;
        --lock->Common.IoCount;
        check(lock->Common.IoCount > 0, "the drain caller held the lock");

        if (--lock->Common.IoCount == 0) {
            KeSetEvent(&lock->Common.RemoveEvent, IO_NO_INCREMENT, FALSE);
        }
    }

    KeWaitForSingleObject(&lock->Common.RemoveEvent, Executive, KernelMode, FALSE, nullptr);
    Lock guard(dispatcher_lock);

    for (unsigned index = 0; index < remove_lock_count; ++index) {
        if (remove_locks[index].lock == lock) {
            remove_locks[index].drained = true;
            remove_locks[index].drained_count = lock->Common.IoCount;
        }
    }
}

// Device objects and stacks.
NTSTATUS NTAPI IoCreateDevice(PDRIVER_OBJECT driver, ULONG extension, PUNICODE_STRING name, DEVICE_TYPE type,
                              ULONG, BOOLEAN, PDEVICE_OBJECT* output) {
    *output = nullptr;

    if (driver == filter_driver && inject(Fault::create_device)) {
        return STATUS_INSUFFICIENT_RESOURCES;
    }

    if (name != nullptr && named_device != nullptr) {
        return STATUS_OBJECT_NAME_COLLISION;
    }

    const size_t header = (sizeof(DEVICE_OBJECT) + 15) & ~static_cast<size_t>(15);
    auto* storage = static_cast<uint8_t*>(calloc(1, header + extension + 16));

    if (storage == nullptr) {
        die("out of memory");
    }

    auto* device = reinterpret_cast<PDEVICE_OBJECT>(storage);
    device->Type = IO_TYPE_DEVICE;
    device->Size = static_cast<USHORT>(sizeof(DEVICE_OBJECT) + extension);
    device->DriverObject = driver;
    device->DeviceExtension = extension != 0 ? storage + header : nullptr;
    device->StackSize = 1;
    device->Flags = DO_DEVICE_INITIALIZING;
    device->DeviceType = type;
    Lock lock(dispatcher_lock);
    device->NextDevice = driver->DeviceObject;
    driver->DeviceObject = device;
    Tracked* entry = lookup(device);
    entry->listed = true;
    entry->extension_bytes = extension;

    if (name != nullptr) {
        named_device = device;
    }

    *output = device;
    return STATUS_SUCCESS;
}

VOID NTAPI IoDeleteDevice(PDEVICE_OBJECT device) {
    {
        Lock lock(dispatcher_lock);
        Tracked* entry = lookup(device);
        check(!entry->deleted, "a device object is deleted once");
        entry->deleted = true;

        if (named_device == device) {
            named_device = nullptr;
        }
    }

    unload_check(device);
}

PDEVICE_OBJECT NTAPI IoAttachDeviceToDeviceStack(PDEVICE_OBJECT source, PDEVICE_OBJECT target) {
    Lock lock(dispatcher_lock);
    PDEVICE_OBJECT top = target;

    while (top->AttachedDevice != nullptr) {
        top = top->AttachedDevice;
    }

    top->AttachedDevice = source;
    source->StackSize = static_cast<CCHAR>(top->StackSize + 1);
    return top;
}

VOID NTAPI IoDetachDevice(PDEVICE_OBJECT target) {
    {
        Lock lock(dispatcher_lock);
        check(target->AttachedDevice != nullptr, "detach only an attached device");
        target->AttachedDevice = nullptr;
        ++detaches;
    }

    unload_check(target);
}

LONG_PTR FASTCALL ObfReferenceObject(PVOID object) {
    Lock lock(dispatcher_lock);
    return ++lookup(object)->references;
}

LONG_PTR FASTCALL ObfDereferenceObject(PVOID object) {
    Lock lock(dispatcher_lock);
    Tracked* entry = lookup(object);
    check(entry->references > 0, "object references never go negative");
    const long references = --entry->references;

    if (object != fake_process) {
        poison_if_released(*entry);
    }

    return references;
}

PEPROCESS NTAPI IoGetCurrentProcess(VOID) { return fake_process; }
PEPROCESS NTAPI IoGetRequestorProcess(PIRP) { return fake_process; }

NTSTATUS NTAPI IoCreateSymbolicLink(PUNICODE_STRING, PUNICODE_STRING) {
    if (inject(Fault::link_create)) {
        return STATUS_OBJECT_NAME_COLLISION;
    }

    Lock lock(dispatcher_lock);

    if (link_live) {
        return STATUS_OBJECT_NAME_COLLISION;
    }

    link_live = true;
    ++links_created;
    return STATUS_SUCCESS;
}

NTSTATUS NTAPI IoDeleteSymbolicLink(PUNICODE_STRING) {
    Lock lock(dispatcher_lock);
    check(link_live, "only a live control link is deleted");
    link_live = false;
    ++links_deleted;
    return STATUS_SUCCESS;
}

// IRPs.
PIRP NTAPI IoAllocateIrp(CCHAR stack_size, BOOLEAN) {
    if (inject(Fault::allocate_irp)) {
        return nullptr;
    }

    Lock lock(dispatcher_lock);
    ++driver_irps_allocated;
    return raw_irp(stack_size);
}

VOID NTAPI IoFreeIrp(PIRP irp) {
    Lock lock(dispatcher_lock);
    ++driver_irps_freed;
    free(irp);
}

VOID NTAPI IoReuseIrp(PIRP irp, NTSTATUS status) {
    const CCHAR stack_size = irp->StackCount;
    const USHORT size = irp->Size;
    memset(irp, 0, size);
    irp->Type = IO_TYPE_IRP;
    irp->Size = size;
    irp->StackCount = stack_size;
    irp->CurrentLocation = static_cast<CHAR>(stack_size + 1);
    irp->Tail.Overlay.CurrentStackLocation = reinterpret_cast<PIO_STACK_LOCATION>(irp + 1) + stack_size;
    irp->IoStatus.Status = status;
}

PIRP NTAPI IoBuildDeviceIoControlRequest(ULONG code, PDEVICE_OBJECT device, PVOID input, ULONG input_bytes,
                                         PVOID output, ULONG output_bytes, BOOLEAN internal, PKEVENT event,
                                         PIO_STATUS_BLOCK status) {
    check((code & 3) == METHOD_BUFFERED && !internal && event != nullptr && status != nullptr,
        "private display requests are buffered external IOCTLs with a caller event");
    PIRP irp = raw_irp(device->StackSize);
    const ULONG bytes = input_bytes > output_bytes ? input_bytes : output_bytes;
    void* buffer = calloc(1, bytes != 0 ? bytes : 1);

    if (buffer == nullptr) {
        die("out of memory");
    }

    if (input != nullptr && input_bytes != 0) {
        memcpy(buffer, input, input_bytes);
    }

    irp->AssociatedIrp.SystemBuffer = buffer;
    irp->UserBuffer = output;
    irp->UserEvent = event;
    irp->UserIosb = status;
    irp->RequestorMode = KernelMode;
    auto* next = IoGetNextIrpStackLocation(irp);
    next->MajorFunction = IRP_MJ_DEVICE_CONTROL;
    next->Parameters.DeviceIoControl.IoControlCode = code;
    next->Parameters.DeviceIoControl.InputBufferLength = input_bytes;
    next->Parameters.DeviceIoControl.OutputBufferLength = output_bytes;
    Lock lock(dispatcher_lock);
    ++built_allocated;

    for (auto& entry : built) {
        if (entry == nullptr) {
            entry = irp;
            return irp;
        }
    }

    die("built IRP table full");
}

NTSTATUS FASTCALL IofCallDriver(PDEVICE_OBJECT device, PIRP irp) {
    if (--irp->CurrentLocation <= 0) {
        die("IRP stack overflow");
    }

    --irp->Tail.Overlay.CurrentStackLocation;
    auto* stack = IoGetCurrentIrpStackLocation(irp);
    stack->DeviceObject = device;
    return device->DriverObject->MajorFunction[stack->MajorFunction](device, irp);
}

VOID FASTCALL IofCompleteRequest(PIRP irp, CCHAR) {
    if (irp->CurrentLocation > irp->StackCount + 1) {
        die("IRP completed beyond its stack");
    }

    while (irp->CurrentLocation <= irp->StackCount) {
        auto* stack = IoGetCurrentIrpStackLocation(irp);
        irp->PendingReturned = (stack->Control & SL_PENDING_RETURNED) != 0;
        const PIO_COMPLETION_ROUTINE routine = stack->CompletionRoutine;
        const PVOID context = stack->Context;
        const UCHAR control = stack->Control;
        stack->CompletionRoutine = nullptr;
        stack->Context = nullptr;
        stack->Control = 0;
        ++irp->CurrentLocation;
        ++irp->Tail.Overlay.CurrentStackLocation;
        const NTSTATUS status = irp->IoStatus.Status;
        const bool invoke = routine != nullptr &&
            ((NT_SUCCESS(status) && (control & SL_INVOKE_ON_SUCCESS) != 0) ||
             (!NT_SUCCESS(status) && (control & SL_INVOKE_ON_ERROR) != 0) ||
             (irp->Cancel && (control & SL_INVOKE_ON_CANCEL) != 0));

        if (invoke) {
            PDEVICE_OBJECT device = irp->CurrentLocation <= irp->StackCount ?
                IoGetCurrentIrpStackLocation(irp)->DeviceObject : nullptr;

            if (routine(device, irp, context) == STATUS_MORE_PROCESSING_REQUIRED) {
                return;
            }
        } else if (irp->CurrentLocation <= irp->StackCount && irp->PendingReturned) {
            IoMarkIrpPending(irp);
        }
    }

    final_completion(irp);
}

NTSTATUS NTAPI PoCallDriver(PDEVICE_OBJECT device, PIRP irp) { return IofCallDriver(device, irp); }

VOID NTAPI PoStartNextPowerIrp(PIRP irp) {
    Lock lock(dispatcher_lock);
    Sent* record = sent_record(irp);
    check(record != nullptr && record->power, "PoStartNextPowerIrp names a power IRP");

    if (record != nullptr) {
        ++record->start_next;
    }
}

POWER_STATE NTAPI PoSetPowerState(PDEVICE_OBJECT, POWER_STATE_TYPE type, POWER_STATE state) {
    check(type == DevicePowerState && state.DeviceState >= PowerDeviceD0 && state.DeviceState <= PowerDeviceD3,
        "the filter reports valid device power states only");
    return state;
}

// Threads and work items.
NTSTATUS NTAPI PsCreateSystemThread(PHANDLE handle, ULONG, POBJECT_ATTRIBUTES, HANDLE, PCLIENT_ID,
                                    PKSTART_ROUTINE routine, PVOID context) {
    *handle = nullptr;

    if (inject(Fault::create_thread)) {
        return STATUS_INSUFFICIENT_RESOURCES;
    }

    Lock lock(dispatcher_lock);

    if (system_thread_count == sizeof(system_threads) / sizeof(system_threads[0])) {
        die("system thread table");
    }

    SystemThread& thread = system_threads[system_thread_count++];
    thread.routine = routine;
    thread.context = context;
    thread.thread = lifecycle_platform_thread(system_thread_trampoline, &thread);
    *handle = &thread;
    return STATUS_SUCCESS;
}

NTSTATUS NTAPI PsTerminateSystemThread(NTSTATUS) {
    lifecycle_platform_exit_thread();
    return STATUS_SUCCESS;
}

NTSTATUS NTAPI ZwWaitForSingleObject(HANDLE handle, BOOLEAN, PLARGE_INTEGER timeout) {
    check(timeout == nullptr, "the power thread join is untimed");
    auto* thread = static_cast<SystemThread*>(handle);

    if (inject(Fault::thread_join)) {
        return STATUS_INVALID_HANDLE;
    }

    if (!lifecycle_platform_join(thread->thread, wait_milliseconds)) {
        die("power thread did not terminate");
    }

    thread->joined = true;
    return STATUS_SUCCESS;
}

NTSTATUS NTAPI ZwClose(HANDLE handle) {
    auto* thread = static_cast<SystemThread*>(handle);
    check(thread->joined && !thread->closed, "a joined thread handle is closed once");
    lifecycle_platform_close(thread->thread);
    thread->closed = true;
    return STATUS_SUCCESS;
}

PIO_WORKITEM NTAPI IoAllocateWorkItem(PDEVICE_OBJECT device) {
    if (inject(Fault::work_item)) {
        return nullptr;
    }

    auto* item = static_cast<PIO_WORKITEM>(calloc(1, sizeof(_IO_WORKITEM)));
    item->device = device;
    Lock lock(dispatcher_lock);
    ++work_allocated;
    return item;
}

VOID NTAPI IoQueueWorkItem(PIO_WORKITEM item, PIO_WORKITEM_ROUTINE routine, WORK_QUEUE_TYPE, PVOID context) {
    check(item != nullptr, "only a reserved work item is queued");
    ObfReferenceObject(item->device);
    item->routine = routine;
    item->context = context;
    keep_helper(lifecycle_platform_thread(work_trampoline, item));
}

VOID NTAPI IoFreeWorkItem(PIO_WORKITEM item) {
    Lock lock(dispatcher_lock);
    ++work_freed;
    // The routine may still be running on its thread; ReactOS frees the
    // structure here too and the trampoline only reads its saved device.
    item->routine = nullptr;
}

NTSTATUS NTAPI PsSetCreateProcessNotifyRoutine(PCREATE_PROCESS_NOTIFY_ROUTINE routine, BOOLEAN remove) {
    Lock lock(dispatcher_lock);

    if (remove) {
        check(notify_routine == routine, "the registered notification is removed");
        notify_routine = nullptr;
        ++notify_unregistered;
    } else {
        notify_routine = routine;
        ++notify_registered;
    }

    return STATUS_SUCCESS;
}

} // extern "C"

namespace {

// ---- the PnP manager / I/O manager side of the scenarios.
struct Fixture final {
    PDEVICE_OBJECT fdo = nullptr;
    PDEVICE_OBJECT foreign_top = nullptr;
    PDEVICE_OBJECT control = nullptr;
    FILE_OBJECT foreign_file{};
    FILE_OBJECT control_file{};
    bool foreign_open = false;
    bool control_open = false;
    bool removed = false;
    bool remove_failed = false;
    NTSTATUS last = STATUS_SUCCESS;
};

Fixture fixture;
CM_RESOURCE_LIST resources{};

PIRP new_irp(PDEVICE_OBJECT top, UCHAR major, UCHAR minor, bool power = false) {
    PIRP irp = raw_irp(top->StackSize);
    auto* next = IoGetNextIrpStackLocation(irp);
    next->MajorFunction = major;
    next->MinorFunction = minor;
    irp->IoStatus.Status = STATUS_NOT_SUPPORTED;
    irp->RequestorMode = KernelMode;
    Lock lock(dispatcher_lock);

    if (sent_count == sizeof(sent) / sizeof(sent[0])) {
        die("sent IRP table full");
    }

    sent[sent_count++] = {irp, 0, 0, power};
    return irp;
}

NTSTATUS send(PDEVICE_OBJECT top, PIRP irp) {
    ObfReferenceObject(top);
    IofCallDriver(top, irp);
    const unsigned long long started = lifecycle_platform_ticks();
    {
        Lock lock(dispatcher_lock);
        Sent* record = sent_record(irp);

        while (record->completions == 0) {
            wait_or_die(started);
        }
    }

    const NTSTATUS status = irp->IoStatus.Status;
    ObfDereferenceObject(top);
    unload_check(top);
    return status;
}

NTSTATUS pnp(UCHAR minor) {
    PIRP irp = new_irp(fixture.fdo, IRP_MJ_PNP, minor);

    if (minor == IRP_MN_START_DEVICE) {
        auto* next = IoGetNextIrpStackLocation(irp);
        next->Parameters.StartDevice.AllocatedResources = &resources;
        next->Parameters.StartDevice.AllocatedResourcesTranslated = &resources;
    }

    return fixture.last = send(fixture.fdo, irp);
}

NTSTATUS power(UCHAR minor, bool system, ULONG state) {
    PIRP irp = new_irp(fixture.fdo, IRP_MJ_POWER, minor, true);
    auto* next = IoGetNextIrpStackLocation(irp);
    next->Parameters.Power.Type = system ? SystemPowerState : DevicePowerState;

    if (system) {
        next->Parameters.Power.State.SystemState = static_cast<SYSTEM_POWER_STATE>(state);
    } else {
        next->Parameters.Power.State.DeviceState = static_cast<DEVICE_POWER_STATE>(state);
    }

    return fixture.last = send(fixture.fdo, irp);
}

NTSTATUS remove_device() {
    const NTSTATUS status = pnp(IRP_MN_REMOVE_DEVICE);
    fixture.removed = NT_SUCCESS(status);
    fixture.remove_failed = !NT_SUCCESS(status);
    return status;
}

// win32k opens \Device\VideoN; the I/O manager sends CREATE to the top of
// its stack and counts the open on the named lower device.
NTSTATUS open_foreign() {
    PDEVICE_OBJECT top = lower_fdo;

    while (top->AttachedDevice != nullptr) {
        top = top->AttachedDevice;
    }

    ++lower_fdo->ReferenceCount;
    PIRP irp = new_irp(top, IRP_MJ_CREATE, 0);
    IoGetNextIrpStackLocation(irp)->FileObject = &fixture.foreign_file;
    const NTSTATUS status = send(top, irp);

    if (NT_SUCCESS(status)) {
        fixture.foreign_top = top;
        fixture.foreign_open = true;
    } else {
        --lower_fdo->ReferenceCount;
    }

    return status;
}

void close_foreign() {
    if (!fixture.foreign_open) {
        return;
    }

    PIRP cleanup = new_irp(fixture.foreign_top, IRP_MJ_CLEANUP, 0);
    IoGetNextIrpStackLocation(cleanup)->FileObject = &fixture.foreign_file;
    send(fixture.foreign_top, cleanup);
    PIRP close = new_irp(fixture.foreign_top, IRP_MJ_CLOSE, 0);
    IoGetNextIrpStackLocation(close)->FileObject = &fixture.foreign_file;
    send(fixture.foreign_top, close);
    fixture.foreign_open = false;
    --lower_fdo->ReferenceCount;
}

bool fdo_deleted(PDEVICE_OBJECT device);

// The display client's kernel-mode lease release through the top of stack.
NTSTATUS release_lease(uint32_t generation) {
    PDEVICE_OBJECT top = fixture.foreign_top != nullptr ? fixture.foreign_top : fixture.fdo;

    // No client can reach a deleted object (it holds no lease without an open).
    if (fdo_deleted(top)) {
        return STATUS_NO_SUCH_DEVICE;
    }

    PIRP irp = new_irp(top, IRP_MJ_DEVICE_CONTROL, 0);
    auto* next = IoGetNextIrpStackLocation(irp);
    next->Parameters.DeviceIoControl.IoControlCode = ioctl_resolve_owned_staging;
    next->Parameters.DeviceIoControl.InputBufferLength = sizeof(StagingResolveRequest);
    next->Parameters.DeviceIoControl.OutputBufferLength = sizeof(StagingResolveReceipt);
    uint8_t buffer[sizeof(StagingResolveReceipt)]{};
    const StagingResolveRequest request{sizeof(request), cpu_scanout_abi_version, generation,
        staging_resolve_release, 0};
    memcpy(buffer, &request, sizeof(request));
    irp->AssociatedIrp.SystemBuffer = buffer;
    const NTSTATUS status = send(top, irp);
    irp->AssociatedIrp.SystemBuffer = nullptr;
    return status;
}

NTSTATUS open_control() {
    PDEVICE_OBJECT control = named_device;

    if (control == nullptr) {
        return STATUS_OBJECT_NAME_NOT_FOUND;
    }

    ++control->ReferenceCount;
    PIRP irp = new_irp(control, IRP_MJ_CREATE, 0);
    IoGetNextIrpStackLocation(irp)->FileObject = &fixture.control_file;
    const NTSTATUS status = send(control, irp);

    if (NT_SUCCESS(status)) {
        fixture.control = control;
        fixture.control_open = true;
    } else {
        --control->ReferenceCount;
        unload_check(control);
    }

    return status;
}

void close_control() {
    if (!fixture.control_open) {
        return;
    }

    PIRP cleanup = new_irp(fixture.control, IRP_MJ_CLEANUP, 0);
    IoGetNextIrpStackLocation(cleanup)->FileObject = &fixture.control_file;
    send(fixture.control, cleanup);
    PIRP close = new_irp(fixture.control, IRP_MJ_CLOSE, 0);
    IoGetNextIrpStackLocation(close)->FileObject = &fixture.control_file;
    send(fixture.control, close);
    fixture.control_open = false;
    --fixture.control->ReferenceCount;
    unload_check(fixture.control);
}

void process_exit() {
    if (notify_routine != nullptr) {
        notify_routine(nullptr, reinterpret_cast<HANDLE>(0x1234), FALSE);
    }
}

PDRIVER_OBJECT make_driver() {
    auto* driver = static_cast<PDRIVER_OBJECT>(calloc(1, sizeof(DRIVER_OBJECT)));
    auto* extension = static_cast<PDRIVER_EXTENSION>(calloc(1, sizeof(DRIVER_EXTENSION)));

    if (driver == nullptr || extension == nullptr) {
        die("out of memory");
    }

    driver->Type = IO_TYPE_DRIVER;
    driver->DriverExtension = extension;
    extension->DriverObject = driver;
    return driver;
}

bool fdo_deleted(PDEVICE_OBJECT device) {
    Lock lock(dispatcher_lock);
    return device != nullptr && lookup(device)->deleted;
}

void wait_until_deleted(PDEVICE_OBJECT device) {
    const unsigned long long started = lifecycle_platform_ticks();

    while (!fdo_deleted(device) && lifecycle_platform_ticks() - started < 3000) {
        lifecycle_platform_sleep(2);
    }
}

// Boot: DriverEntry, the lower PCI/videoprt stack and AddDevice.
NTSTATUS boot() {
    filter_driver = make_driver();
    pci_driver = make_driver();
    video_driver = make_driver();

    for (auto& entry : video_driver->MajorFunction) {
        entry = lower_dispatch;
    }

    for (auto& entry : pci_driver->MajorFunction) {
        entry = lower_dispatch;
    }

    IoCreateDevice(pci_driver, 0, nullptr, FILE_DEVICE_BUS_EXTENDER, 0, FALSE, &pdo);
    IoCreateDevice(video_driver, 0, nullptr, FILE_DEVICE_VIDEO, 0, FALSE, &lower_fdo);
    IoAttachDeviceToDeviceStack(lower_fdo, pdo);
    UNICODE_STRING path{};
    const NTSTATUS entered = DriverEntry(filter_driver, &path);
    check(NT_SUCCESS(entered) && notify_registered == 1, "DriverEntry registers process tracking");
    const NTSTATUS added = filter_driver->DriverExtension->AddDevice(filter_driver, pdo);

    if (NT_SUCCESS(added)) {
        fixture.fdo = lower_fdo->AttachedDevice;
        check(fixture.fdo != nullptr && fixture.fdo->DriverObject == filter_driver, "AddDevice attaches on top");
    }

    return added;
}

PDEVICE_OBJECT add_second_device() {
    IoCreateDevice(video_driver, 0, nullptr, FILE_DEVICE_VIDEO, 0, FALSE, &lower_fdo);
    PDEVICE_OBJECT second_pdo = nullptr;
    IoCreateDevice(pci_driver, 0, nullptr, FILE_DEVICE_BUS_EXTENDER, 0, FALSE, &second_pdo);
    IoAttachDeviceToDeviceStack(lower_fdo, second_pdo);
    pdo = second_pdo;
    const NTSTATUS added = filter_driver->DriverExtension->AddDevice(filter_driver, second_pdo);
    check(NT_SUCCESS(added) || fault_at != no_fault, "a second FDO is added after the first was removed");
    fixture = Fixture{};
    fixture.fdo = NT_SUCCESS(added) ? lower_fdo->AttachedDevice : nullptr;
    return fixture.fdo;
}

// Complete the retirement channel: the client releases its leases and
// win32k drops its display file object.
void finish_retirement(PDEVICE_OBJECT fdo, uint32_t leases) {
    // lightningdisp retries a failed release a bounded three times.
    for (uint32_t generation = 1; generation <= leases; ++generation) {
        unsigned attempts = 0;

        while (attempts < 3 && !NT_SUCCESS(release_lease(generation))) {
            ++attempts;
        }
    }

    close_foreign();
    wait_until_deleted(fdo);
}

bool clean() { return fault_at == no_fault; }

// ---- universal end-of-run invariants (every fault included).
void final_checks(PDEVICE_OBJECT fdo) {
    // Let any queued teardown and helper finish before judging.
    for (unsigned index = 0; index < helper_count; ++index) {
        if (!lifecycle_platform_join(helper_threads[index], wait_milliseconds)) {
            die("helper thread did not finish");
        }
    }

    for (unsigned index = 0; index < sent_count; ++index) {
        check(sent[index].completions == 1, "every IRP the PnP/I/O manager sent completes exactly once");
        check(!sent[index].power || sent[index].start_next == 1, "every power IRP starts the next one exactly once");
    }

    check(built_allocated == built_freed, "every private display request IRP is freed");
    check(hardware_after_containment == 0, "no register access after containment");
    check(asserts == 0, "no NT_ASSERT fired");
    check(transport_released <= transport_acquired, "transport ownership is released at most once per start");

    for (unsigned index = 0; index < remove_lock_count; ++index) {
        // Read at drain time: the extension may be released (poisoned) now.
        if (remove_locks[index].drained) {
            check(remove_locks[index].drained_count == 0, "a drained remove lock has no holder left");
        }
    }

    const bool never_armed = logged("teardown never");
    // Documented limit: an armed channel whose held-lease count no query ever
    // answered (and whose client released nothing) stays until reboot.
    const bool unanswered = lower.abandoned && lower.answered_after_abandon == 0;
    const bool retained = contained_seen || never_armed || fixture.remove_failed || unanswered;

    if (fixture.removed && !retained) {
        check(fdo_deleted(fdo), "a clean REMOVE deletes the FDO (after its retirement channel)");
        check(transport_released == transport_acquired, "a clean REMOVE released every transport start");
        check(!link_live && links_created == links_deleted, "the control link is withdrawn exactly once");
        check(work_allocated == work_freed, "the retirement work item is freed");
        check(driver_irps_allocated == driver_irps_freed, "the reserved abandon IRP is freed");
        check(system_threads[0].joined && system_threads[0].closed, "the power thread is joined and closed");
        check(lookup(lower_fdo)->references == 0, "the lower device reference is dropped");
        check(lower_fdo->AttachedDevice == nullptr, "the filter detached from the lower device");

        if (!fixture.control_open) {
            check(unload_invoked == 1 && notify_unregistered == 1, "the driver unloads once and unregisters");
        }
    }

    if (retained && (fixture.removed || fixture.remove_failed)) {
        check(!fdo_deleted(fdo), "a retained FDO is never deleted");
        check(unload_invoked == 0 && filter_driver->DriverUnload == nullptr, "a retained FDO keeps unload disarmed");
        check(!link_live, "a retained FDO has no control link");
    }
}

// ---- scenarios.
void scenario_display_lifecycle() {
    lower.pend_power = true;

    if (!NT_SUCCESS(boot())) {
        return;
    }

    PDEVICE_OBJECT fdo = fixture.fdo;
    const NTSTATUS started = pnp(IRP_MN_START_DEVICE);

    if (clean()) {
        check(NT_SUCCESS(started) && link_live, "START publishes the control link");
        const unsigned transport = find(Op::transport_start, 0xffffffffu);
        const unsigned discover = find(Op::lower_ioctl, ioctl_query_cpu_scanout);
        const unsigned resume = find(Op::lower_ioctl, ioctl_resume_cpu_scanout);
        const unsigned renderer = find(Op::renderer_start, 0xffffffffu);
        check(find(Op::lower_pnp, IRP_MN_START_DEVICE) < transport && transport < discover && discover < resume &&
            resume < renderer, "START: lower, transport, display discovery, display resume, renderer");
    }

    open_foreign();
    open_control();
    close_control();
    const unsigned before_d3 = entry_count;
    power(IRP_MN_SET_POWER, false, PowerDeviceD3);

    if (clean()) {
        const unsigned quiesce = find(Op::lower_ioctl, ioctl_quiesce_cpu_scanout, before_d3);
        const unsigned renderer = find(Op::renderer_stop_host, 0xffffffffu, before_d3);
        const unsigned suspend = find(Op::transport_suspend, 0xffffffffu, before_d3);
        const unsigned lower_d3 = find(Op::lower_power, IRP_MN_SET_POWER, before_d3);
        check(quiesce < renderer && renderer < suspend && suspend < lower_d3,
            "D3: display quiesce, renderer, transport suspend, then the lower D3");
        check(!NT_SUCCESS(open_control()), "no control open while suspended");
    }

    const unsigned before_d0 = entry_count;
    power(IRP_MN_SET_POWER, false, PowerDeviceD0);

    if (clean()) {
        check(find(Op::lower_power, IRP_MN_SET_POWER, before_d0) < find(Op::transport_resume, 0xffffffffu, before_d0) &&
            find(Op::transport_resume, 0xffffffffu, before_d0) <
                find(Op::lower_ioctl, ioctl_resume_cpu_scanout, before_d0),
            "D0: lower D0, transport resume, display resume");
        check(NT_SUCCESS(open_control()), "control opens again after D0");
        close_control();
    }

    if (NT_SUCCESS(pnp(IRP_MN_QUERY_STOP_DEVICE))) {
        pnp(IRP_MN_STOP_DEVICE);

        if (clean()) {
            check(transport_released == 1 && !link_live, "STOP releases the transport and the link");
        }

        pnp(IRP_MN_START_DEVICE);

        if (clean()) {
            check(transport_acquired == 2 && link_live, "START after STOP restarts the owners");
        }
    }

    lower.held_leases = 1;
    pnp(IRP_MN_QUERY_REMOVE_DEVICE);
    remove_device();

    if (clean()) {
        check(fixture.removed && logged("removed display retained as staging retirement channel") &&
            logged("teardown armed"), "REMOVE keeps the recognized display as an armed retirement channel");
        check(!fdo_deleted(fdo), "the retirement channel outlives REMOVE");
    }

    finish_retirement(fdo, 1);

    if (clean()) {
        check(logged("retirement complete; deleting detached filter and re-arming unload"),
            "the last lease and close tear the channel down and re-arm unload");
    }

    final_checks(fdo);
}

void scenario_standalone() {
    lower.display = false;

    if (!NT_SUCCESS(boot())) {
        return;
    }

    PDEVICE_OBJECT fdo = fixture.fdo;
    pnp(IRP_MN_START_DEVICE);
    power(IRP_MN_SET_POWER, false, PowerDeviceD3);
    power(IRP_MN_SET_POWER, false, PowerDeviceD0);
    // A legacy display stack below the filter: win32k still opens it.
    open_foreign();
    pnp(IRP_MN_QUERY_REMOVE_DEVICE);
    remove_device();

    if (clean()) {
        check(!fdo_deleted(fdo) && logged("leases retired, teardown armed"),
            "win32k's open keeps a channel for a standalone transport (no leases possible)");
    }

    // win32k's close through its captured pointer must reach a live filter.
    close_foreign();
    wait_until_deleted(fdo);

    if (clean()) {
        check(fdo_deleted(fdo) && unload_invoked == 1, "the last close deletes the standalone channel and unloads");
    }

    final_checks(fdo);
}

void scenario_surprise() {
    if (!NT_SUCCESS(boot())) {
        return;
    }

    PDEVICE_OBJECT fdo = fixture.fdo;
    pnp(IRP_MN_START_DEVICE);
    open_foreign();
    open_control();
    const unsigned before = entry_count;
    pnp(IRP_MN_SURPRISE_REMOVAL);

    if (clean()) {
        check(find(Op::transport_stop_hardware, 0xffffffffu, before) == 0xffffffffu &&
            find(Op::renderer_stop_host, 0xffffffffu, before) == 0xffffffffu &&
            find(Op::lower_ioctl, ioctl_quiesce_cpu_scanout, before) == 0xffffffffu,
            "SURPRISE touches no registers");
        check(find(Op::lower_ioctl, ioctl_abandon_cpu_scanout, before) <
            find(Op::lower_pnp, IRP_MN_SURPRISE_REMOVAL, before), "SURPRISE abandons the display before the lower stack");
    }

    remove_device();
    // A GL process still holds its control file across REMOVE: its CLOSE
    // still reaches the referenced FDO extension, then unload can follow.
    close_control();
    finish_retirement(fdo, 0);

    if (clean()) {
        check(unload_invoked == 1, "unload follows the last control close and the retirement teardown");
    }

    final_checks(fdo);
}

void scenario_contained_restart() {
    if (!NT_SUCCESS(boot())) {
        return;
    }

    PDEVICE_OBJECT fdo = fixture.fdo;
    pnp(IRP_MN_START_DEVICE);
    pnp(IRP_MN_QUERY_STOP_DEVICE);
    exit_quarantines = true;
    process_exit();
    exit_quarantines = false;

    if (clean()) {
        check(contained_seen, "an unprovable process exit contains the device");
    }

    pnp(IRP_MN_STOP_DEVICE);
    const unsigned starts = count_ops(Op::transport_start);
    const NTSTATUS restarted = pnp(IRP_MN_START_DEVICE);

    if (clean()) {
        check(restarted == STATUS_DEVICE_BUSY && count_ops(Op::transport_start) == starts,
            "a contained device never restarts its transport");
        check(pnp(IRP_MN_QUERY_REMOVE_DEVICE) == STATUS_DEVICE_BUSY, "a contained device vetoes QUERY_REMOVE");
        check(!NT_SUCCESS(open_control()), "a contained device refuses control opens");
    }

    power(IRP_MN_SET_POWER, false, PowerDeviceD0);
    pnp(IRP_MN_SURPRISE_REMOVAL);
    remove_device();

    if (clean()) {
        check(logged("removed contained filter retained until reboot") || logged("teardown never"),
            "REMOVE retains the contained FDO");
    }

    final_checks(fdo);
}

void scenario_contained_sibling() {
    if (!NT_SUCCESS(boot())) {
        return;
    }

    PDEVICE_OBJECT first = fixture.fdo;
    pnp(IRP_MN_START_DEVICE);
    renderer_views = true;
    power(IRP_MN_SET_POWER, false, PowerDeviceD3);
    renderer_views = false;
    pnp(IRP_MN_SURPRISE_REMOVAL);
    remove_device();
    const bool first_contained = contained_seen;

    if (clean()) {
        check(first_contained && !fdo_deleted(first), "D3 with live views contains and REMOVE retains the FDO");
    }

    // The second FDO is clean; its retirement teardown must not re-arm the
    // unload entry that the first FDO's containment cleared.
    contained_seen = false;
    lower.abandoned = false;
    lower.answered_after_abandon = 0;
    PDEVICE_OBJECT second = add_second_device();

    if (second != nullptr) {
        pnp(IRP_MN_START_DEVICE);
        open_foreign();
        pnp(IRP_MN_QUERY_REMOVE_DEVICE);
        remove_device();
        finish_retirement(second, 0);
    }

    if (clean()) {
        check(fdo_deleted(second) && logged("unload stays disarmed"),
            "a sibling's retirement teardown deletes itself but keeps unload disarmed");
        check(unload_invoked == 0 && filter_driver->DriverUnload == nullptr && !fdo_deleted(first),
            "the contained FDO keeps the image loaded");
    }

    // Universal checks for the image as a whole.
    for (unsigned index = 0; index < helper_count; ++index) {
        lifecycle_platform_join(helper_threads[index], wait_milliseconds);
    }

    for (unsigned index = 0; index < sent_count; ++index) {
        check(sent[index].completions == 1, "every IRP completes exactly once");
    }

    check(built_allocated == built_freed && asserts == 0, "no leaked private request or assertion");
    check(unload_invoked == 0 || !first_contained, "an image with a contained FDO never unloads");
}

void scenario_shutdown() {
    if (!NT_SUCCESS(boot())) {
        return;
    }

    PDEVICE_OBJECT fdo = fixture.fdo;
    pnp(IRP_MN_START_DEVICE);
    const NTSTATUS query = power(IRP_MN_QUERY_POWER, true, PowerSystemShutdown);
    const unsigned before = entry_count;
    power(IRP_MN_SET_POWER, true, PowerSystemShutdown);

    if (clean()) {
        check(NT_SUCCESS(query), "the shutdown QUERY is admitted");
        const unsigned quiesce = find(Op::lower_ioctl, ioctl_quiesce_cpu_scanout, before);
        const unsigned suspend = find(Op::transport_suspend, 0xffffffffu, before);
        const unsigned lower_set = find(Op::lower_power, IRP_MN_SET_POWER, before);
        check(quiesce < suspend && suspend < lower_set && transport_released == 0,
            "shutdown quiesces display and transport before the lower stack and retains DMA");
        check(!NT_SUCCESS(open_control()), "no control work after shutdown");
    }

    // A late REMOVE (not sent on ReactOS shutdown) still tears down cleanly.
    remove_device();
    final_checks(fdo);
}

void scenario_sleep_and_refusal() {
    if (!NT_SUCCESS(boot())) {
        return;
    }

    PDEVICE_OBJECT fdo = fixture.fdo;
    pnp(IRP_MN_START_DEVICE);
    const unsigned before = entry_count;
    const NTSTATUS sleep = power(IRP_MN_QUERY_POWER, true, PowerSystemSleeping3);
    const NTSTATUS hibernate = power(IRP_MN_QUERY_POWER, true, PowerSystemHibernate);
    power(IRP_MN_SET_POWER, true, PowerSystemSleeping3);

    if (clean()) {
        check(sleep == STATUS_NOT_SUPPORTED && hibernate == STATUS_NOT_SUPPORTED, "sleep and hibernate are vetoed");
        check(count_ops(Op::lower_power, IRP_MN_QUERY_POWER, before) == 0, "a vetoed QUERY never reaches the lower stack");
        check(count_ops(Op::transport_suspend, 0xffffffffu, before) == 0, "an unqueried sleep SET is only forwarded");
    }

    const unsigned refused = entry_count;
    power(IRP_MN_SET_POWER, false, PowerDeviceD3);

    if (clean()) {
        check(count_ops(Op::transport_resume, 0xffffffffu, refused) == 0, "an accepted D3 does not resume");
    }

    power(IRP_MN_SET_POWER, false, PowerDeviceD0);
    pnp(IRP_MN_QUERY_REMOVE_DEVICE);
    remove_device();
    finish_retirement(fdo, 0);
    final_checks(fdo);
}

void scenario_lower_refuses_d3() {
    if (!NT_SUCCESS(boot())) {
        return;
    }

    PDEVICE_OBJECT fdo = fixture.fdo;
    pnp(IRP_MN_START_DEVICE);
    const unsigned before = entry_count;
    force_next(Fault::lower_d3);
    const NTSTATUS refused = power(IRP_MN_SET_POWER, false, PowerDeviceD3);

    if (clean()) {
        check(!NT_SUCCESS(refused), "the lower refusal is returned");
        check(find(Op::transport_suspend, 0xffffffffu, before) < find(Op::lower_power, IRP_MN_SET_POWER, before) &&
            find(Op::lower_power, IRP_MN_SET_POWER, before) < find(Op::transport_resume, 0xffffffffu, before) &&
            !contained_seen, "a refused D3 rolls back to an owned D0 after the lower refusal");
        check(NT_SUCCESS(open_control()), "control work resumes after the rollback");
        close_control();
    }

    pnp(IRP_MN_QUERY_REMOVE_DEVICE);
    remove_device();
    finish_retirement(fdo, 0);
    final_checks(fdo);
}

void scenario_join_failure() {
    if (!NT_SUCCESS(boot())) {
        return;
    }

    PDEVICE_OBJECT fdo = fixture.fdo;
    pnp(IRP_MN_START_DEVICE);
    pnp(IRP_MN_QUERY_REMOVE_DEVICE);
    force_next(Fault::thread_join);
    const NTSTATUS removed = remove_device();

    if (clean()) {
        check(!NT_SUCCESS(removed) && contained_seen && count_ops(Op::lower_pnp, IRP_MN_REMOVE_DEVICE) == 0,
            "a failed power-thread join contains and never forwards or detaches");
    }

    final_checks(fdo);
}

void scenario_add_device() {
    const NTSTATUS added = boot();

    if (clean()) {
        check(NT_SUCCESS(added), "AddDevice succeeds without faults");
    } else if (!NT_SUCCESS(added)) {
        // Every partial AddDevice acquisition is released.
        check(filter_driver->DeviceObject == nullptr, "a failed AddDevice deletes its devices");
        check(lower_fdo->AttachedDevice == nullptr && lookup(lower_fdo)->references == 0,
            "a failed AddDevice detaches and drops its lower reference");
        check(driver_irps_allocated == driver_irps_freed, "a failed AddDevice frees its abandon IRP");
        check(system_thread_count == 0 || (fault_at != no_fault), "no power thread outlives a failed AddDevice");
        return;
    }

    PDEVICE_OBJECT fdo = fixture.fdo;
    pnp(IRP_MN_START_DEVICE);
    const NTSTATUS duplicate = pnp(IRP_MN_START_DEVICE);

    if (clean()) {
        check(NT_SUCCESS(duplicate) && count_ops(Op::transport_start) == 1 && transport_released == 0,
            "a duplicate START keeps the running device");
    }

    pnp(IRP_MN_QUERY_REMOVE_DEVICE);
    pnp(IRP_MN_CANCEL_REMOVE_DEVICE);

    if (clean()) {
        check(NT_SUCCESS(open_control()), "CANCEL_REMOVE restores control work");
        close_control();
    }

    pnp(IRP_MN_QUERY_REMOVE_DEVICE);
    remove_device();
    finish_retirement(fdo, 0);
    final_checks(fdo);
}

// A REMOVE that no admitted QUERY_REMOVE preceded (the PnP manager sends
// none after a surprise removal; nothing else bypasses the veto below) while
// a GL process still borrows ring views: the renderer cannot end the borrow,
// so REMOVE contains, still abandons the display (lease transfer) and retains
// the FDO with unload disarmed.
void scenario_busy_remove() {
    if (!NT_SUCCESS(boot())) {
        return;
    }

    PDEVICE_OBJECT fdo = fixture.fdo;
    pnp(IRP_MN_START_DEVICE);
    open_foreign();
    open_control();
    renderer_views = true;
    const unsigned before = entry_count;
    remove_device();

    if (clean()) {
        check(contained_seen && logged(" step 7 ") && logged("teardown never"),
            "REMOVE with live views contains at remove_owners and never arms its teardown");
        check(find(Op::lower_ioctl, ioctl_abandon_cpu_scanout, before) <
            find(Op::lower_pnp, IRP_MN_REMOVE_DEVICE, before), "the display is still abandoned before the lower REMOVE");
    }

    // The process exits and win32k releases the display: nothing is deleted.
    close_control();
    finish_retirement(fdo, 0);

    if (clean()) {
        check(!fdo_deleted(fdo) && unload_invoked == 0, "the contained FDO and image stay until reboot");
    }

    final_checks(fdo);
}

// lightning-pnp-veto: Device Manager disable (QUERY_REMOVE), a rebalance
// (QUERY_STOP) and a device D3 QUERY while a GL process maps renderer views.
// Each query is refused before the lower stack sees it and the device keeps
// serving; after the borrow ends the same disable completes and unloads.
// Before the veto the QUERY_REMOVE was admitted and REMOVE contained.
void scenario_busy_disable_veto() {
    if (!NT_SUCCESS(boot())) {
        return;
    }

    PDEVICE_OBJECT fdo = fixture.fdo;
    // Only a started renderer can have handed out views.
    const NTSTATUS started = pnp(IRP_MN_START_DEVICE);
    open_foreign();
    open_control();
    renderer_views = NT_SUCCESS(started);
    const unsigned before = entry_count;
    const NTSTATUS query = pnp(IRP_MN_QUERY_REMOVE_DEVICE);

    if (clean()) {
        check(query == STATUS_DEVICE_BUSY &&
            logged("LightningNT5: query-remove vetoed, 1 renderer session(s) hold 2 borrowed view(s)"),
            "QUERY_REMOVE with live renderer views is vetoed with a log line naming the holders");
        check(count_ops(Op::lower_pnp, IRP_MN_QUERY_REMOVE_DEVICE, before) == 0,
            "the vetoed QUERY_REMOVE never reaches videoprt");
        check(!contained_seen && count_ops(Op::renderer_stop_host, 0xffffffffu, before) == 0 &&
            count_ops(Op::renderer_stop_quiet, 0xffffffffu, before) == 0 &&
            count_ops(Op::lower_ioctl, ioctl_quiesce_cpu_scanout, before) == 0 &&
            count_ops(Op::lower_ioctl, ioctl_abandon_cpu_scanout, before) == 0,
            "the veto touches no owner: no containment, renderer stop, quiesce or abandon");
    }

    const unsigned cancel = entry_count;
    pnp(IRP_MN_CANCEL_REMOVE_DEVICE);

    if (clean()) {
        check(count_ops(Op::lower_pnp, IRP_MN_CANCEL_REMOVE_DEVICE, cancel) == 1 &&
            logged("LightningNT5: cancel-remove, phase 1 contained 0"),
            "CANCEL_REMOVE after the veto is forwarded and finds the device started");
        close_control();
        check(NT_SUCCESS(open_control()), "control work continues after the vetoed disable");
    }

    const unsigned stop = entry_count;
    const NTSTATUS query_stop = pnp(IRP_MN_QUERY_STOP_DEVICE);
    pnp(IRP_MN_CANCEL_STOP_DEVICE);
    const NTSTATUS query_d3 = power(IRP_MN_QUERY_POWER, false, PowerDeviceD3);

    if (clean()) {
        check(query_stop == STATUS_DEVICE_BUSY && logged("LightningNT5: query-stop vetoed") &&
            count_ops(Op::lower_pnp, IRP_MN_QUERY_STOP_DEVICE, stop) == 0, "QUERY_STOP with live views is vetoed");
        check(query_d3 == STATUS_DEVICE_BUSY && logged("device power query D3 vetoed") &&
            count_ops(Op::lower_power, IRP_MN_QUERY_POWER, stop) == 0, "a device D3 QUERY with live views is vetoed");
        check(!contained_seen && count_ops(Op::transport_stop_hardware, 0xffffffffu, before) == 0 &&
            count_ops(Op::transport_suspend, 0xffffffffu, before) == 0, "the vetoes keep the transport running");
    }

    // The GL process exits: its views are revoked by the exit notification.
    process_exit();
    close_control();
    const unsigned after = entry_count;
    const NTSTATUS query_d3_free = power(IRP_MN_QUERY_POWER, false, PowerDeviceD3);
    lower.held_leases = 1;
    const NTSTATUS admitted = pnp(IRP_MN_QUERY_REMOVE_DEVICE);

    if (clean()) {
        check(NT_SUCCESS(query_d3_free) && count_ops(Op::lower_power, IRP_MN_QUERY_POWER, after) == 1,
            "a device D3 QUERY without views is forwarded");
        check(NT_SUCCESS(admitted) && logged("LightningNT5: query-remove admitted") &&
            count_ops(Op::lower_pnp, IRP_MN_QUERY_REMOVE_DEVICE, after) == 1,
            "QUERY_REMOVE after the borrow ended is admitted and forwarded");
        check(!NT_SUCCESS(open_control()), "an admitted query closes new control work");
    }

    remove_device();
    finish_retirement(fdo, 1);

    if (clean()) {
        check(!contained_seen && fdo_deleted(fdo) && unload_invoked == 1 &&
            logged("retirement complete; deleting detached filter and re-arming unload"),
            "the disable after the borrow ended completes and the driver unloads");
    }

    final_checks(fdo);
}

// Surprise removal (QMP device_del) cannot be vetoed: with live views it
// still contains, register-free, after abandoning the display.
void scenario_busy_surprise() {
    if (!NT_SUCCESS(boot())) {
        return;
    }

    PDEVICE_OBJECT fdo = fixture.fdo;
    pnp(IRP_MN_START_DEVICE);
    open_foreign();
    open_control();
    renderer_views = true;
    const unsigned before = entry_count;
    pnp(IRP_MN_SURPRISE_REMOVAL);

    if (clean()) {
        check(contained_seen && logged(" step 6 ") &&
            find(Op::lower_ioctl, ioctl_abandon_cpu_scanout, before) <
            find(Op::lower_pnp, IRP_MN_SURPRISE_REMOVAL, before),
            "SURPRISE with live views abandons the display, then contains at the renderer");
        check(find(Op::transport_stop_hardware, 0xffffffffu, before) == 0xffffffffu &&
            find(Op::renderer_stop_host, 0xffffffffu, before) == 0xffffffffu, "SURPRISE stays register-free");
    }

    remove_device();
    close_control();
    finish_retirement(fdo, 0);

    if (clean()) {
        check(!fdo_deleted(fdo) && unload_invoked == 0 && filter_driver->DriverUnload == nullptr,
            "a surprise-removed FDO with a live borrow stays until reboot");
    }

    final_checks(fdo);
}

// Both REMOVE-time held-lease queries fail and the client holds no lease:
// only the re-query before win32k's last close can retire the channel.
void scenario_unanswered_leases() {
    if (!NT_SUCCESS(boot())) {
        return;
    }

    PDEVICE_OBJECT fdo = fixture.fdo;
    pnp(IRP_MN_START_DEVICE);
    open_foreign();
    pnp(IRP_MN_QUERY_REMOVE_DEVICE);
    force_next(Fault::ioctl_resolve, 2);
    remove_device();

    if (clean()) {
        check(logged("leases held, teardown armed") && !fdo_deleted(fdo),
            "unanswered lease queries keep the armed channel");
    }

    finish_retirement(fdo, 0);

    if (clean()) {
        check(fdo_deleted(fdo) && unload_invoked == 1, "the last close re-asks and tears the channel down");
    }

    final_checks(fdo);
}

struct Scenario final {
    const char* name;
    void (*run)();
};

const Scenario scenarios[] = {
    {"display lifecycle", scenario_display_lifecycle},
    {"standalone transport", scenario_standalone},
    {"surprise removal", scenario_surprise},
    {"contained restart", scenario_contained_restart},
    {"contained sibling", scenario_contained_sibling},
    {"system shutdown", scenario_shutdown},
    {"sleep veto", scenario_sleep_and_refusal},
    {"lower refuses D3", scenario_lower_refuses_d3},
    {"power thread join failure", scenario_join_failure},
    {"add device", scenario_add_device},
    {"busy remove", scenario_busy_remove},
    {"busy disable veto", scenario_busy_disable_veto},
    {"busy surprise", scenario_busy_surprise},
    {"unanswered leases", scenario_unanswered_leases},
};

constexpr unsigned scenario_count = sizeof(scenarios) / sizeof(scenarios[0]);

int run_child(unsigned scenario, unsigned fault) {
    fault_at = fault;
    scenarios[scenario].run();

    if (fault != no_fault && !fault_hit) {
        return exit_sweep_done;
    }

    if (failures != 0) {
        printf("scenario '%s' fault %u: %u checks, %u failures\n", scenarios[scenario].name, fault, checks, failures);
    }

    return failures == 0 ? exit_pass : exit_fail;
}

} // namespace

int main(int argc, char** argv) {
    verbose = getenv("LIFECYCLE_VERBOSE") != nullptr;
    setvbuf(stdout, nullptr, _IONBF, 0);

    if (argc == 3) {
        const unsigned scenario = static_cast<unsigned>(strtoul(argv[1], nullptr, 10));
        const unsigned fault = strcmp(argv[2], "clean") == 0 ? no_fault :
            static_cast<unsigned>(strtoul(argv[2], nullptr, 10));
        lifecycle_platform_exit(scenario < scenario_count ? run_child(scenario, fault) : exit_fail);
    }

    unsigned runs = 0;
    unsigned failed = 0;
    unsigned swept = 0;
    const unsigned only = argc == 2 ? static_cast<unsigned>(strtoul(argv[1], nullptr, 10)) : scenario_count;

    for (unsigned scenario = 0; scenario < scenario_count; ++scenario) {
        if (only != scenario_count && only != scenario) {
            continue;
        }

        char arguments[32];
        snprintf(arguments, sizeof(arguments), "%u clean", scenario);
        ++runs;
        const int clean_result = lifecycle_platform_child(arguments, 60000);

        if (clean_result != exit_pass) {
            ++failed;
            printf("FAIL scenario '%s' clean run: exit %d\n", scenarios[scenario].name, clean_result);
        }

        for (unsigned fault = 0; fault < 400; ++fault) {
            snprintf(arguments, sizeof(arguments), "%u %u", scenario, fault);
            const int result = lifecycle_platform_child(arguments, 60000);

            if (result == exit_sweep_done) {
                break;
            }

            ++runs;
            ++swept;

            if (result != exit_pass) {
                ++failed;
                printf("FAIL scenario '%s' fault %u: exit %d\n", scenarios[scenario].name, fault, result);
            }
        }
    }

    printf("Lightning lifecycle native: %u runs (%u fault injections), %u failed\n", runs, swept, failed);
    return failed == 0 ? 0 : 1;
}

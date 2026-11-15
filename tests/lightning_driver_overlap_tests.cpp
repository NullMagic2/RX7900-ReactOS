/*
Module: lightning.tests.driver_overlap
Purpose: Explore production LightningNT5 entry-point interleavings under IOCTL, PnP, power and process-exit overlap.
Created: 2026-10-25
Architecture: Compiles the unmodified lightning_driver.cpp against a cooperative fake NT kernel.
    Every actor (PnP and power managers, a GL process, the kernel display client,
    win32k, the filter's own power thread and work items) is a Win32 fiber. Each
    fake kernel wait, delay, lower call and completion is a scheduling point, and a
    seeded scheduler chooses the next runnable actor there, so every run is an
    exact, replayable interleaving. Device objects, extensions, the driver image
    and IRPs follow the ReactOS lifetime rules the filter relies on (iomgr
    device.c IoDeleteDevice/IopUnloadDevice, irp.c completion order): a deleted
    extension is poisoned when its last reference drops, the image is unmapped
    once DriverUnload ran and no device object remains. A run fails on any use of
    a freed extension (including later writes into the poison), driver code
    resuming in an unmapped image, a lost or double IRP completion, overlapping
    lower IOCTLs, hardware I/O after the lower REMOVE or in D3, an exchange on a
    stopped transport, a host image freed under an outstanding read, deadlock or
    an unbounded run. NativeTransport/NativeRenderer are link-time models of
    their documented contracts; the lower device models videoprt + the miniport.
*/
#include <stdint.h>
#include <ntifs.h>
#include <ntddvdeo.h>
#include "lightning_renderer.hpp"
#include "lightning_present_abi.hpp"
#include "lightning_staging_resolve.hpp"
#include "lightning_gdi_abi.hpp"
#include "lightning_abi.hpp"
#include "lightning-display.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <process.h>

extern "C" NTSTATUS NTAPI DriverEntry(PDRIVER_OBJECT driver, PUNICODE_STRING path);

extern "C" {
__declspec(dllimport) PVOID NTAPI ConvertThreadToFiber(PVOID parameter);
__declspec(dllimport) PVOID NTAPI CreateFiber(SIZE_T stack, VOID (NTAPI* start)(PVOID), PVOID parameter);
__declspec(dllimport) VOID NTAPI SwitchToFiber(PVOID fiber);
__declspec(dllimport) VOID NTAPI DeleteFiber(PVOID fiber);
}

using namespace lightning::reactos;

namespace {
unsigned checks = 0;
unsigned failures = 0;

void check(bool condition, const char* description) {
    ++checks;

    if (!condition) {
        ++failures;
        printf("FAIL %s\n", description);
    }
}

// ndk KOBJECTS::MutantObject; dispatcher headers of events use EVENT_TYPE.
constexpr UCHAR mutant_object_type = 2;

// ---- Scheduler -----------------------------------------------------------
enum class Block { none, mutex, event, actor, flag };
struct Actor;
using Body = void (*)(Actor&);

constexpr int max_actors = 16;
constexpr int process_count = 4;
uint8_t process_storage[process_count][64]{};

struct Actor final {
    char name[24]{};
    Body body = nullptr;
    void* argument = nullptr;
    PVOID fiber = nullptr;
    bool finished = false;
    Block block = Block::none;
    const void* object = nullptr;
    const volatile bool* flag = nullptr;
    int process = 0;
    uint8_t thread_token[16]{};
    int index = 0;
};

struct Violation final {
    char text[160]{};
};

struct World;
World* world = nullptr;

// Lower (videoprt FDO + Lightning miniport) model.
struct Lower final {
    bool started = false;
    bool removed = false;
    bool surprise = false;
    bool d3 = false;
    bool lost = false;
    bool quiesced = true;
    bool in_ioctl = false;
    unsigned opens = 0;
    unsigned closes = 0;
    unsigned hardware = 0;
    unsigned retirement = 0;
    unsigned abandons = 0;
    uint32_t held_leases = 1;
    uint32_t lease_generation = 5;
    uint32_t mode_generation = 3;
    uint32_t epoch = 1;
    uint32_t accepted = 0;
    uint32_t completed = 0;
    uint64_t read_lease = 0;
    uint32_t read_sequence = 0;
    unsigned slow_resolve = 0;
    unsigned reenter_code = 0;
    NTSTATUS reentered = STATUS_SUCCESS;
    bool reentry_done = false;
};

struct FakeDevice final {
    DEVICE_OBJECT object{};
    uint8_t* extension = nullptr;
    size_t extension_bytes = 0;
    bool ours = false;
    bool deleted = false;
    bool keepalive = true;
    bool freed = false;
    LONG object_refs = 0;
};

struct FakeIrp final {
    bool harness = false;
    bool built = false;
    unsigned completions = 0;
    bool done = false;
    NTSTATUS status = STATUS_PENDING;
    ULONG_PTR information = 0;
    Actor* requestor = nullptr;
    PVOID user_buffer = nullptr;
    ULONG output_bytes = 0;
    PKEVENT event = nullptr;
    PIO_STATUS_BLOCK iosb = nullptr;
    PVOID system_buffer = nullptr;
    size_t allocation = 0;
};

struct Session final {
    PFILE_OBJECT file = nullptr;
    int process = 0;
    uint64_t lease = 0;
    ImageReceipt pin{};
    bool pinned = false;
    bool exiting = false;
    bool views = false;
    bool lost = false;
};

struct WorkItem final {
    PDEVICE_OBJECT device = nullptr;
    PIO_WORKITEM_ROUTINE routine = nullptr;
    PVOID context = nullptr;
    bool queued = false;
};

struct World final {
    Actor actors[max_actors]{};
    int count = 0;
    Actor* current = nullptr;
    Actor* join = nullptr;
    PVOID scheduler = nullptr;
    uint32_t rng = 1;
    uint64_t steps = 0;
    uint64_t step_limit = 400000;
    uint64_t clock = 0;
    bool aborted = false;
    Violation violation{};
    // NT objects
    DRIVER_OBJECT driver{};
    DRIVER_EXTENSION driver_extension{};
    DRIVER_OBJECT video_driver{};
    FakeDevice* devices[16]{};
    int device_count = 0;
    FakeDevice* pdo = nullptr;
    FakeDevice* lower_device = nullptr;
    FakeDevice* filter = nullptr;
    FakeDevice* control = nullptr;
    bool unload_invoked = false;
    bool unmapped = false;
    bool link = false;
    PCREATE_PROCESS_NOTIFY_ROUTINE notify = nullptr;
    WorkItem work_items[4]{};
    int work_count = 0;
    FakeIrp* irps[256]{};
    int irp_count = 0;
    Lower lower{};
    // renderer/transport models
    bool transport_ready = false;
    Session sessions[8]{};
    unsigned exchanges = 0;
    unsigned exchange_refusals = 0;
    unsigned pins_by_reset = 0;
    unsigned pins_by_proof = 0;
    bool renderer_contained = false;
    unsigned containments = 0;
    unsigned retained_reboot = 0;
    // harness events
    volatile bool started_flag = false;
    volatile bool abandon_seen = false;
    volatile bool remove_done = false;
    volatile bool cleanup_gate_closed = false;
    volatile bool d3_done = false;
    volatile bool exit_done = false;
    volatile bool stale_inside = false;
    volatile bool stale_second = false;
    volatile bool display_ready = false;
    // Clients still making their first open; PnP/power actors start after.
    int clients_pending = 0;
    volatile bool apps_ready = false;
    bool display_opened = false;
    bool lease_released = false;
    volatile bool* in_flight = nullptr;
    unsigned and_wait_calls = 0;
    char log[8192]{};
    size_t log_used = 0;
};

void trace(const char* format, ...) {
    char line[160];
    va_list arguments;
    va_start(arguments, format);
    vsnprintf(line, sizeof(line), format, arguments);
    va_end(arguments);
    const size_t length = strlen(line);

    if (world->log_used + length + 1 < sizeof(world->log)) {
        memcpy(world->log + world->log_used, line, length);
        world->log_used += length;
        world->log[world->log_used] = 0;
    }
}

uint32_t next_random() {
    world->rng = world->rng * 1103515245u + 12345u;
    return world->rng >> 8;
}

void violate(const char* text) {
    if (world->aborted) {
        return;
    }

    world->aborted = true;
    snprintf(world->violation.text, sizeof(world->violation.text), "%s [actor %s step %lu]", text,
        world->current != nullptr ? world->current->name : "-", static_cast<unsigned long>(world->steps));
}

bool is_device(const void* pointer, FakeDevice** found = nullptr) {
    for (int index = 0; index < world->device_count; ++index) {
        if (&world->devices[index]->object == pointer) {
            if (found != nullptr) {
                *found = world->devices[index];
            }
            return true;
        }
    }
    return false;
}

void check_pointer(const void* pointer, const char* what) {
    const auto* byte = static_cast<const uint8_t*>(pointer);

    for (int index = 0; index < world->device_count; ++index) {
        const FakeDevice& device = *world->devices[index];

        if (!device.freed) {
            continue;
        }

        if ((device.extension != nullptr && byte >= device.extension &&
             byte < device.extension + device.extension_bytes) || pointer == &device.object) {
            char text[120];
            snprintf(text, sizeof(text), "use of a deleted device extension: %s", what);
            violate(text);
        }
    }
}

// Every fake export is driver code calling the kernel; it must run in a mapped image.
void driver_code() {
    if (world->unmapped) {
        violate("driver code ran after its image was unmapped");
    }
}

bool runnable(const Actor& actor) {
    if (actor.finished || actor.fiber == nullptr) {
        return false;
    }

    switch (actor.block) {
    case Block::none:
        return true;
    case Block::mutex: {
        const auto* mutex = static_cast<const KMUTEX*>(actor.object);
        return mutex->OwnerThread == nullptr ||
            mutex->OwnerThread == reinterpret_cast<const struct _KTHREAD*>(actor.thread_token);
    }
    case Block::event:
        return static_cast<const DISPATCHER_HEADER*>(actor.object)->SignalState != 0;
    case Block::actor:
        return static_cast<const Actor*>(actor.object)->finished;
    case Block::flag:
        return *actor.flag;
    }
    return false;
}

// Return to the scheduler; resumes when chosen again (and unblocked).
void yield() {
    Actor* self = world->current;
    SwitchToFiber(world->scheduler);
    world->current = self;
}

// The scheduler resumes a blocked actor only once its condition holds, and
// nothing runs between that check and the resumption.
void block_until(Block kind, const void* object, const volatile bool* flag = nullptr) {
    Actor& self = *world->current;
    self.block = kind;
    self.object = object;
    self.flag = flag;
    yield();
    self.block = Block::none;
}

void wait_flag(const volatile bool* flag) {
    while (!*flag && !world->aborted) {
        block_until(Block::flag, nullptr, flag);
    }
}

// Yield inside a fake export; driver code resumes afterwards.
void preempt() {
    yield();
    driver_code();
}

VOID NTAPI fiber_main(PVOID parameter) {
    auto& actor = *static_cast<Actor*>(parameter);
    world->current = &actor;
    actor.body(actor);
    actor.finished = true;

    for (;;) {
        SwitchToFiber(world->scheduler);
    }
}

Actor& spawn(const char* name, Body body, int process = 0, void* argument = nullptr) {
    Actor& actor = world->actors[world->count];
    actor.index = world->count++;
    snprintf(actor.name, sizeof(actor.name), "%s", name);
    actor.body = body;
    actor.argument = argument;
    actor.process = process;
    actor.fiber = CreateFiber(256 * 1024, fiber_main, &actor);
    return actor;
}

// Run until every actor finished, a violation, a deadlock or the step bound.
void run_scheduler() {
    while (!world->aborted) {
        Actor* candidates[max_actors];
        int count = 0;
        bool live = false;

        for (int index = 0; index < world->count; ++index) {
            Actor& actor = world->actors[index];

            if (!actor.finished && actor.fiber != nullptr) {
                live = true;
            }

            if (runnable(actor)) {
                candidates[count++] = &actor;
            }
        }

        if (!live || (world->join != nullptr && world->join->finished)) {
            return;
        }

        if (count == 0) {
            char text[160];
            size_t used = static_cast<size_t>(snprintf(text, sizeof(text), "deadlock: blocked"));

            for (int index = 0; index < world->count && used < sizeof(text); ++index) {
                if (!world->actors[index].finished) {
                    used += static_cast<size_t>(snprintf(text + used, sizeof(text) - used, " %s",
                        world->actors[index].name));
                }
            }

            violate(text);
            return;
        }

        if (++world->steps > world->step_limit) {
            violate("unbounded run (step limit)");
            return;
        }

        Actor* chosen = candidates[next_random() % static_cast<uint32_t>(count)];

        // Prefer continuing the current actor so runs get deep as well as wide.
        if (world->current != nullptr && runnable(*world->current) && next_random() % 3 != 0) {
            chosen = world->current;
        }

        world->current = chosen;
        SwitchToFiber(chosen->fiber);
    }
}

// ---- Device and IRP model ----------------------------------------------
void maybe_unmap() {
    if (!world->unload_invoked || world->unmapped) {
        return;
    }

    for (int index = 0; index < world->device_count; ++index) {
        if (world->devices[index]->ours && !world->devices[index]->freed) {
            return;
        }
    }

    world->unmapped = true;
}

void maybe_free(FakeDevice& device) {
    if (device.freed || !device.deleted || device.keepalive || device.object_refs != 0) {
        return;
    }

    if (device.extension != nullptr) {
        memset(device.extension, 0xdd, device.extension_bytes);
    }

    device.freed = true;
    maybe_unmap();
}

void unlink_device(FakeDevice& device) {
    PDEVICE_OBJECT* link = &world->driver.DeviceObject;

    while (*link != nullptr && *link != &device.object) {
        link = &(*link)->NextDevice;
    }

    if (*link != nullptr) {
        *link = device.object.NextDevice;
    }
}

// IopUnloadDevice: drop the keep-alive and maybe run DriverUnload synchronously.
void unload_device(FakeDevice& device) {
    if (device.deleted && device.keepalive) {
        if (device.object.AttachedDevice != nullptr || device.object.ReferenceCount != 0) {
            return;
        }

        unlink_device(device);
        device.keepalive = false;
    }

    if (!world->unload_invoked && world->driver.DeviceObject == nullptr && world->driver.DriverUnload != nullptr) {
        world->unload_invoked = true;
        world->driver.DriverUnload(&world->driver);
    }

    maybe_free(device);
}

void reference(PDEVICE_OBJECT object) {
    FakeDevice* device = nullptr;

    if (is_device(object, &device)) {
        ++device->object_refs;
    }
}

void dereference(PDEVICE_OBJECT object) {
    FakeDevice* device = nullptr;

    if (is_device(object, &device)) {
        --device->object_refs;
        maybe_free(*device);
    }
}

FakeDevice* new_device(size_t extension_bytes, bool ours) {
    auto* device = static_cast<FakeDevice*>(calloc(1, sizeof(FakeDevice)));
    device->extension_bytes = extension_bytes;
    device->extension = extension_bytes != 0 ? static_cast<uint8_t*>(calloc(1, extension_bytes)) : nullptr;
    device->ours = ours;
    device->keepalive = true;
    device->object.Type = IO_TYPE_DEVICE;
    device->object.Size = sizeof(DEVICE_OBJECT);
    device->object.StackSize = 1;
    device->object.DeviceExtension = device->extension;
    world->devices[world->device_count++] = device;
    return device;
}

FakeIrp* info(PIRP irp) { return reinterpret_cast<FakeIrp*>(irp) - 1; }

PIRP allocate_irp(CCHAR stack_size) {
    const size_t bytes = sizeof(FakeIrp) + IoSizeOfIrp(stack_size);
    auto* storage = static_cast<uint8_t*>(calloc(1, bytes));
    auto* record = reinterpret_cast<FakeIrp*>(storage);
    auto* irp = reinterpret_cast<PIRP>(storage + sizeof(FakeIrp));
    record->allocation = bytes;
    IoInitializeIrp(irp, IoSizeOfIrp(stack_size), stack_size);
    record->requestor = world->current;
    world->irps[world->irp_count++ % 256] = record;
    return irp;
}

void free_irp(PIRP irp) {
    FakeIrp* record = info(irp);

    for (auto& slot : world->irps) {
        if (slot == record) {
            slot = nullptr;
        }
    }

    free(record->system_buffer);
    free(record);
}

NTSTATUS call_driver(PDEVICE_OBJECT device, PIRP irp) {
    FakeDevice* fake = nullptr;

    if (!is_device(device, &fake)) {
        violate("IRP sent to an unknown device object");
        return STATUS_INVALID_DEVICE_REQUEST;
    }

    if (fake->freed) {
        violate("IRP sent to a freed device object");
        return STATUS_DELETE_PENDING;
    }

    if (--irp->CurrentLocation <= 0) {
        violate("IRP stack exhausted");
        return STATUS_INVALID_DEVICE_REQUEST;
    }

    --irp->Tail.Overlay.CurrentStackLocation;
    auto* stack = IoGetCurrentIrpStackLocation(irp);
    stack->DeviceObject = device;
    return device->DriverObject->MajorFunction[stack->MajorFunction](device, irp);
}

void complete_irp(PIRP irp) {
    FakeIrp* record = info(irp);

    for (auto* stack = IoGetCurrentIrpStackLocation(irp); irp->CurrentLocation <= irp->StackCount; ++stack) {
        irp->PendingReturned = (stack->Control & SL_PENDING_RETURNED) != 0;
        ++irp->CurrentLocation;
        ++irp->Tail.Overlay.CurrentStackLocation;
        const bool success = NT_SUCCESS(irp->IoStatus.Status);
        const auto routine = stack->CompletionRoutine;
        const auto context = stack->Context;
        const UCHAR control = stack->Control;

        if (routine != nullptr && ((success && (control & SL_INVOKE_ON_SUCCESS) != 0) ||
            (!success && (control & SL_INVOKE_ON_ERROR) != 0))) {
            const bool last = irp->CurrentLocation == irp->StackCount + 1;
            PDEVICE_OBJECT device = last ? nullptr : IoGetCurrentIrpStackLocation(irp)->DeviceObject;
            stack->CompletionRoutine = nullptr;
            stack->Control = 0;

            if (routine(device, irp, context) == STATUS_MORE_PROCESSING_REQUIRED) {
                return;
            }
        } else if (irp->CurrentLocation <= irp->StackCount && irp->PendingReturned) {
            IoMarkIrpPending(irp);
        }
    }

    if (++record->completions != 1) {
        violate("IRP completed twice");
        return;
    }

    record->status = irp->IoStatus.Status;
    record->information = irp->IoStatus.Information;

    if (record->built) {
        if (record->user_buffer != nullptr && record->system_buffer != nullptr && NT_SUCCESS(record->status)) {
            const size_t copied = record->information < record->output_bytes ? record->information : record->output_bytes;
            memcpy(record->user_buffer, record->system_buffer, copied);
        }

        if (record->iosb != nullptr) {
            *record->iosb = irp->IoStatus;
        }

        record->done = true;
        KEVENT* event = record->event;
        free_irp(irp);
        event->Header.SignalState = 1;
        return;
    }

    record->done = true;
}

// ---- Harness requests -----------------------------------------------------
PIRP harness_irp(PDEVICE_OBJECT target, UCHAR major, UCHAR minor, PFILE_OBJECT file = nullptr,
                 KPROCESSOR_MODE mode = KernelMode) {
    PIRP irp = allocate_irp(target->StackSize);
    info(irp)->harness = true;
    irp->RequestorMode = mode;
    irp->IoStatus.Status = STATUS_NOT_SUPPORTED;
    auto* next = IoGetNextIrpStackLocation(irp);
    next->MajorFunction = major;
    next->MinorFunction = minor;
    next->FileObject = file;
    return irp;
}

// Send and wait for the single completion. pnp: the sender holds an object
// reference on the target (IoGetAttachedDeviceReference), as the PnP and
// power managers do; win32k's captured pointer holds none.
NTSTATUS send(PDEVICE_OBJECT target, PIRP irp, bool pnp, ULONG_PTR* information = nullptr) {
    FakeIrp* record = info(irp);
    const auto* first = IoGetNextIrpStackLocation(irp);
    const unsigned major = first->MajorFunction;
    const unsigned minor = major == IRP_MJ_DEVICE_CONTROL ?
        static_cast<unsigned>(first->Parameters.DeviceIoControl.IoControlCode) : first->MinorFunction;

    if (pnp) {
        reference(target);
    }

    // in_flight: the caller is already inside the dispatch when this returns
    // control, so a concurrent CLOSE can only follow it (no preemption first).
    if (world->in_flight != nullptr) {
        *world->in_flight = true;
        world->in_flight = nullptr;
    } else {
        yield();
    }

    call_driver(target, irp);
    wait_flag(&record->done);

    if (pnp) {
        dereference(target);
    }

    if (information != nullptr) {
        *information = record->information;
    }

    const NTSTATUS status = record->status;
    trace("%s: IRP %u/%u -> %08lx\n", world->current->name, major, minor, static_cast<unsigned long>(status));
    free_irp(irp);
    return status;
}

NTSTATUS send_ioctl(PDEVICE_OBJECT target, ULONG code, const void* input, ULONG input_bytes, void* output,
                    ULONG output_bytes, PFILE_OBJECT file = nullptr, KPROCESSOR_MODE mode = KernelMode,
                    ULONG_PTR* information = nullptr) {
    PIRP irp = harness_irp(target, IRP_MJ_DEVICE_CONTROL, 0, file, mode);
    const size_t bytes = input_bytes > output_bytes ? input_bytes : output_bytes;
    FakeIrp* record = info(irp);
    record->system_buffer = calloc(1, bytes + 1);
    irp->AssociatedIrp.SystemBuffer = bytes != 0 ? record->system_buffer : nullptr;

    if (input != nullptr && input_bytes != 0) {
        memcpy(record->system_buffer, input, input_bytes);
    }

    auto* next = IoGetNextIrpStackLocation(irp);
    next->Parameters.DeviceIoControl.IoControlCode = code;
    next->Parameters.DeviceIoControl.InputBufferLength = input_bytes;
    next->Parameters.DeviceIoControl.OutputBufferLength = output_bytes;
    ULONG_PTR returned = 0;
    // The buffer outlives free_irp only through this copy.
    void* buffer = record->system_buffer;
    record->system_buffer = nullptr;
    const NTSTATUS status = send(target, irp, false, &returned);

    if (output != nullptr && NT_SUCCESS(status)) {
        memcpy(output, buffer, returned < output_bytes ? returned : output_bytes);
    }

    free(buffer);

    if (information != nullptr) {
        *information = returned;
    }

    return status;
}

NTSTATUS send_pnp(UCHAR minor) {
    PIRP irp = harness_irp(&world->filter->object, IRP_MJ_PNP, minor);
    return send(&world->filter->object, irp, true);
}

NTSTATUS send_power(DEVICE_POWER_STATE state) {
    PIRP irp = harness_irp(&world->filter->object, IRP_MJ_POWER, IRP_MN_SET_POWER);
    auto* next = IoGetNextIrpStackLocation(irp);
    next->Parameters.Power.Type = DevicePowerState;
    next->Parameters.Power.State.DeviceState = state;
    return send(&world->filter->object, irp, true);
}

PDEVICE_OBJECT top_of(PDEVICE_OBJECT device) {
    while (device->AttachedDevice != nullptr) {
        device = device->AttachedDevice;
    }
    return device;
}

// ---- Fake lower driver ---------------------------------------------------
NTSTATUS lower_finish(PIRP irp, NTSTATUS status, ULONG_PTR information = 0) {
    irp->IoStatus.Status = status;
    irp->IoStatus.Information = information;
    complete_irp(irp);
    return status;
}

bool retirement_code(ULONG code) {
    return code == ioctl_resolve_owned_staging || code == ioctl_release_owned_staging ||
        code == IOCTL_VIDEO_UNMAP_VIDEO_MEMORY || code == ioctl_abandon_cpu_scanout;
}

bool inventory_code(ULONG code) {
    return code == ioctl_query_cpu_scanout || code == ioctl_query_cursor || code == IOCTL_VIDEO_QUERY_CURRENT_MODE ||
        code == IOCTL_VIDEO_QUERY_AVAIL_MODES || code == IOCTL_VIDEO_QUERY_NUM_AVAIL_MODES;
}

void reset_canvas(Lower& lower) {
    // A confirmed canvas reset retires every outstanding read.
    lower.completed = lower.accepted;
    lower.read_lease = 0;
    lower.read_sequence = 0;
    ++lower.epoch;
    lower.accepted = 0;
    lower.completed = 0;
}

NTSTATUS lower_ioctl(PIRP irp) {
    Lower& lower = world->lower;
    auto* stack = IoGetCurrentIrpStackLocation(irp);
    const ULONG code = stack->Parameters.DeviceIoControl.IoControlCode;
    void* buffer = irp->AssociatedIrp.SystemBuffer;
    const ULONG input_bytes = stack->Parameters.DeviceIoControl.InputBufferLength;

    if (lower.in_ioctl && !lower.reentry_done) {
        violate("lower IOCTLs overlapped (the filter guard must serialize them)");
    }

    if (lower.removed && !retirement_code(code)) {
        violate("non-retirement IOCTL reached the removed lower device");
    }

    if (lower.surprise && !retirement_code(code) && !inventory_code(code)) {
        violate("hardware or canvas IOCTL reached a surprise-removed lower device");
    }

    lower.in_ioctl = true;
    yield();

    if (lower.reenter_code == code && !lower.reentry_done) {
        // A reentrant VideoPort/miniport callback into the top of the stack.
        lower.reentry_done = true;
        lower.in_ioctl = false;
        lower.reentered = send_ioctl(&world->filter->object, IOCTL_VIDEO_QUERY_CURRENT_MODE, nullptr, 0, nullptr, 0);
        lower.in_ioctl = true;
    }

    NTSTATUS status = STATUS_SUCCESS;
    ULONG_PTR information = 0;

    if (code == ioctl_query_cpu_scanout) {
        if (lower.lost) {
            status = STATUS_DEVICE_DOES_NOT_EXIST;
        } else {
            CpuScanoutInformation answer{};
            answer.size = sizeof(answer);
            answer.version = cpu_scanout_abi_version;
            answer.magic = LIGHTNING_DISPLAY_MAGIC;
            answer.width = 64;
            answer.height = 32;
            answer.stride = 256;
            answer.mapped_bytes = 256 * 32;
            answer.mode_generation = lower.mode_generation;
            answer.enabled = lower.quiesced ? 0 : 1;
            memcpy(buffer, &answer, sizeof(answer));
            information = sizeof(answer);
        }
    } else if (code == ioctl_quiesce_cpu_scanout) {
        if (lower.lost) {
            status = STATUS_DEVICE_DOES_NOT_EXIST;
        } else {
            if (!lower.quiesced) {
                reset_canvas(lower);
            }
            lower.quiesced = true;
        }
    } else if (code == ioctl_resume_cpu_scanout) {
        if (lower.lost || lower.d3) {
            status = lower.lost ? STATUS_DEVICE_DOES_NOT_EXIST : STATUS_DEVICE_NOT_READY;
        } else {
            lower.quiesced = false;
        }
    } else if (code == ioctl_abandon_cpu_scanout) {
        ++lower.abandons;
        lower.lost = true;
        lower.quiesced = true;
        world->abandon_seen = true;
    } else if (code == ioctl_resolve_owned_staging) {
        ++lower.retirement;
        StagingResolveRequest request{};
        memcpy(&request, buffer, sizeof(request));

        for (unsigned pause = 0; pause < lower.slow_resolve; ++pause) {
            world->clock += 10000;
            yield();
        }

        StagingResolveReceipt receipt{};
        receipt.size = sizeof(receipt);
        receipt.version = cpu_scanout_abi_version;
        receipt.lease_generation = request.lease_generation;
        receipt.issued_generation = lower.lease_generation;

        if (request.lease_generation != lower.lease_generation) {
            receipt.outcome = staging_lease_unknown;
        } else if (request.operation == staging_resolve_release && lower.held_leases != 0) {
            --lower.held_leases;
            receipt.outcome = staging_lease_released;
        } else {
            receipt.outcome = lower.held_leases != 0 ? staging_lease_held : staging_lease_not_held;
        }

        receipt.flags = lower.lost ? staging_flag_device_lost : 0;
        receipt.held_leases = lower.held_leases;
        memcpy(buffer, &receipt, sizeof(receipt));
        information = sizeof(receipt);
    } else if (code == gdi::ioctl_query) {
        if (lower.lost || (lower.quiesced && lower.accepted != lower.completed)) {
            status = STATUS_DEVICE_DOES_NOT_EXIST;
        } else {
            gdi::Information answer{};
            answer.size = sizeof(answer);
            answer.version = gdi::native_abi_version;
            answer.magic = LIGHTNING_GDI_MAGIC;
            answer.mode_generation = lower.mode_generation;
            answer.epoch = lower.epoch;
            answer.accepted_sequence = lower.accepted;
            answer.completed_sequence = lower.completed;
            memcpy(buffer, &answer, sizeof(answer));
            information = sizeof(answer);
        }
    } else if (code == gdi::ioctl_submit) {
        ++lower.hardware;

        if (lower.quiesced || lower.lost || lower.d3) {
            violate("GDI submit reached a quiesced, lost or powered-down canvas");
            status = STATUS_DEVICE_DOES_NOT_EXIST;
        } else {
            gdi::SubmitRequest submit{};
            LightningGdiExternalImage image{};
            memcpy(&submit, buffer, sizeof(submit));
            memcpy(&image, static_cast<uint8_t*>(buffer) + sizeof(submit), sizeof(image));
            ++lower.accepted;

            if (submit.version == gdi::native_async_abi_version) {
                lower.read_lease = image.lease;
                lower.read_sequence = lower.accepted;
            } else {
                lower.completed = lower.accepted;
            }

            gdi::Receipt receipt{sizeof(receipt), gdi::native_abi_version, gdi::accepted, LIGHTNING_GDI_OK,
                lower.mode_generation, lower.epoch, lower.accepted, lower.completed, lower.accepted};
            memcpy(buffer, &receipt, sizeof(receipt));
            information = sizeof(receipt);
        }
    } else if (code == gdi::ioctl_wait) {
        if (lower.quiesced || lower.lost) {
            status = STATUS_DEVICE_DOES_NOT_EXIST;
        } else {
            gdi::WaitRequest request{};
            memcpy(&request, buffer, sizeof(request));
            world->clock += 10000;
            yield();

            if (lower.completed < request.sequence && request.sequence <= lower.accepted) {
                lower.completed = request.sequence;
                if (lower.read_sequence <= lower.completed) {
                    lower.read_lease = 0;
                }
            }

            gdi::Receipt receipt{sizeof(receipt), gdi::native_abi_version, gdi::accepted, LIGHTNING_GDI_OK,
                lower.mode_generation, lower.epoch, lower.accepted, lower.completed, lower.accepted};
            memcpy(buffer, &receipt, sizeof(receipt));
            information = sizeof(receipt);
        }
    } else if (inventory_code(code)) {
        status = STATUS_SUCCESS;
    } else {
        ++lower.hardware;

        if (lower.d3 || lower.lost) {
            violate("hardware IOCTL reached a lost or powered-down lower device");
        }
    }

    (void)input_bytes;
    trace("lower: IOCTL %08lx -> %08lx\n", static_cast<unsigned long>(code), static_cast<unsigned long>(status));
    lower.in_ioctl = false;
    return lower_finish(irp, status, information);
}

NTSTATUS NTAPI lower_dispatch(PDEVICE_OBJECT, PIRP irp) {
    Lower& lower = world->lower;
    auto* stack = IoGetCurrentIrpStackLocation(irp);

    switch (stack->MajorFunction) {
    case IRP_MJ_PNP:
        yield();
        switch (stack->MinorFunction) {
        case IRP_MN_START_DEVICE: lower.started = true; lower.removed = false; break;
        case IRP_MN_STOP_DEVICE: lower.started = false; break;
        case IRP_MN_SURPRISE_REMOVAL: lower.surprise = true; break;
        case IRP_MN_REMOVE_DEVICE: lower.removed = true; break;
        default: break;
        }
        return lower_finish(irp, STATUS_SUCCESS);
    case IRP_MJ_POWER:
        yield();
        if (stack->MinorFunction == IRP_MN_SET_POWER && stack->Parameters.Power.Type == DevicePowerState) {
            lower.d3 = stack->Parameters.Power.State.DeviceState != PowerDeviceD0;
        }
        return lower_finish(irp, STATUS_SUCCESS);
    case IRP_MJ_CREATE:
        ++lower.opens;
        return lower_finish(irp, STATUS_SUCCESS);
    case IRP_MJ_CLOSE:
        ++lower.closes;
        return lower_finish(irp, STATUS_SUCCESS);
    case IRP_MJ_CLEANUP:
        return lower_finish(irp, STATUS_SUCCESS);
    case IRP_MJ_DEVICE_CONTROL:
        return lower_ioctl(irp);
    default:
        return lower_finish(irp, STATUS_NOT_SUPPORTED);
    }
}

// ---- World setup -----------------------------------------------------------
void reset_world(uint32_t seed) {
    static World storage;

    for (int index = 0; index < storage.count; ++index) {
        if (storage.actors[index].fiber != nullptr) {
            DeleteFiber(storage.actors[index].fiber);
        }
    }

    for (int index = 0; index < storage.device_count; ++index) {
        free(storage.devices[index]->extension);
        free(storage.devices[index]);
    }

    for (auto* record : storage.irps) {
        if (record != nullptr) {
            free(record->system_buffer);
            free(record);
        }
    }

    PVOID scheduler = storage.scheduler;
    static const World pristine{};
    storage = pristine;
    storage.scheduler = scheduler;
    storage.rng = seed * 2654435761u + 7;
    world = &storage;
    world->driver.DriverExtension = &world->driver_extension;
    world->driver_extension.DriverObject = &world->driver;

    for (auto& function : world->video_driver.MajorFunction) {
        function = lower_dispatch;
    }

    world->pdo = new_device(0, false);
    world->lower_device = new_device(0, false);
    world->lower_device->object.DriverObject = &world->video_driver;
    world->pdo->object.DriverObject = &world->video_driver;
    world->pdo->object.AttachedDevice = &world->lower_device->object;
    world->lower_device->object.StackSize = 2;
}

// ---- Shared actor steps ------------------------------------------------------
struct Files final {
    FILE_OBJECT control{};
    FILE_OBJECT second{};
    FILE_OBJECT display{};
    // Handles to files.control: a duplicate keeps the file open (no CLEANUP).
    unsigned control_handles = 1;
};
Files* files = nullptr;

NTSTATUS open_control(PFILE_OBJECT file) {
    file->DeviceObject = &world->control->object;
    PIRP irp = harness_irp(&world->control->object, IRP_MJ_CREATE, 0, file, UserMode);
    ++world->control->object.ReferenceCount;
    const NTSTATUS status = send(&world->control->object, irp, false);

    if (!NT_SUCCESS(status)) {
        --world->control->object.ReferenceCount;
    }

    return status;
}

// CLEANUP then CLOSE once the last handle goes; the I/O manager drops the
// device reference afterwards.
void close_control(PFILE_OBJECT file) {
    if (file == &files->control && --files->control_handles != 0) {
        return;
    }

    send(&world->control->object, harness_irp(&world->control->object, IRP_MJ_CLEANUP, 0, file, UserMode), false);
    send(&world->control->object, harness_irp(&world->control->object, IRP_MJ_CLOSE, 0, file, UserMode), false);
    --world->control->object.ReferenceCount;
    unload_device(*world->control);
}

NTSTATUS exchange(PFILE_OBJECT file) {
    uint8_t packet[QLV_HEADER_BYTES]{};
    uint8_t reply[QLV_HEADER_BYTES]{};
    return send_ioctl(&world->control->object, ioctl_exchange, packet, sizeof(packet), reply, sizeof(reply),
        file, UserMode);
}

NTSTATUS resolve_release();

// win32k opens the display through the top of the lower stack. Only an open
// that succeeded is ever cleaned up and closed (the I/O manager's rule).
bool display_open() {
    files->display.DeviceObject = &world->lower_device->object;
    ++world->lower_device->object.ReferenceCount;
    PDEVICE_OBJECT top = top_of(&world->lower_device->object);
    const bool opened = NT_SUCCESS(send(top, harness_irp(top, IRP_MJ_CREATE, 0, &files->display), false));

    if (!opened) {
        --world->lower_device->object.ReferenceCount;
    }

    world->display_opened = opened;
    world->display_ready = true;
    return opened;
}

// close_inside (optional) is raised once the final CLOSE is inside the filter.
void display_close(volatile bool* close_inside = nullptr) {
    if (!world->display_opened) {
        return;
    }

    if (world->abandon_seen && !world->lease_released) {
        resolve_release();
    }

    PDEVICE_OBJECT top = top_of(&world->lower_device->object);
    send(top, harness_irp(top, IRP_MJ_CLEANUP, 0, &files->display), false);
    top = top_of(&world->lower_device->object);
    world->in_flight = close_inside;
    send(top, harness_irp(top, IRP_MJ_CLOSE, 0, &files->display), false);
    --world->lower_device->object.ReferenceCount;
}

// The display client's generation-exact lease release (lightning_display.cpp
// resolve_lease). A refused request keeps its token for the next bounded
// retry, which PDEV teardown makes before win32k's final close.
NTSTATUS resolve_release() {
    StagingResolveRequest request{sizeof(request), cpu_scanout_abi_version, world->lower.lease_generation,
        staging_resolve_release, 0};
    StagingResolveReceipt receipt{};
    const NTSTATUS status = send_ioctl(&world->filter->object, ioctl_resolve_owned_staging, &request,
        sizeof(request), &receipt, sizeof(receipt));

    if (NT_SUCCESS(status) && valid_staging_resolve_receipt(receipt, request)) {
        world->lease_released = true;
    }

    return status;
}

// DrawEscape asynchronous present through the kernel display client.
NTSTATUS present_async(uint64_t lease) {
    struct Packet final {
        gdi::SubmitRequest submit;
        LightningGdiExternalImage image;
    } packet{};
    packet.submit.size = sizeof(packet);
    packet.submit.version = gdi::native_async_abi_version;
    packet.submit.command.opcode = LIGHTNING_GDI_EXTERNAL_IMAGE;
    packet.submit.command.metadata_bytes = sizeof(packet) - offsetof(gdi::SubmitRequest, command);
    packet.image.size = sizeof(packet.image);
    packet.image.version = LIGHTNING_GDI_EXTERNAL_IMAGE_VERSION;
    packet.image.lease = lease;
    packet.image.width = 64;
    packet.image.height = 32;
    packet.image.stride = 256;
    gdi::Receipt receipt{};
    return send_ioctl(&world->filter->object, gdi::ioctl_submit, &packet, sizeof(packet), &receipt, sizeof(receipt));
}

NTSTATUS receipt_query(uint64_t lease, uint32_t& status) {
    gdi::ExternalReceiptRequest request{sizeof(request), gdi::native_abi_version, lease};
    gdi::ExternalReceipt reply{};
    const NTSTATUS result = send_ioctl(&world->filter->object, gdi::ioctl_external_receipt, &request,
        sizeof(request), &reply, sizeof(reply));
    status = reply.status;
    return result;
}

void start_device(Actor&) {
    world->filter = nullptr;
    NTSTATUS status = world->driver_extension.AddDevice(&world->driver, &world->pdo->object);
    check(NT_SUCCESS(status), "AddDevice publishes the filter");
    PIRP irp = harness_irp(&world->filter->object, IRP_MJ_PNP, IRP_MN_START_DEVICE);
    status = send(&world->filter->object, irp, true);
    check(NT_SUCCESS(status) && world->transport_ready && world->link, "START brings the transport and link up");
    world->started_flag = true;
}

// The host GPU retires the outstanding canvas read at its own pace.
void host_body(Actor&) {
    for (unsigned tick = 0; tick < 60 && !world->remove_done; ++tick) {
        world->clock += 10000;
        yield();

        if (world->lower.read_lease != 0 && next_random() % 5 == 0) {
            world->lower.completed = world->lower.accepted;
            world->lower.read_lease = 0;
        }
    }
}

void client_opened() { --world->clients_pending; }

void wait_for_clients() {
    wait_flag(&world->started_flag);

    while (world->clients_pending > 0 && !world->aborted) {
        yield();
    }
}

void process_exit(int process) {
    world->notify(nullptr, reinterpret_cast<HANDLE>(static_cast<ULONG_PTR>(100 + process * 4)), FALSE);
}
} // namespace

// ---- Fake kernel exports ---------------------------------------------------
extern "C" {
ULONG __cdecl DbgPrint(PCSTR format, ...) {
    va_list arguments;
    va_start(arguments, format);
    char line[256];
    vsnprintf(line, sizeof(line), format, arguments);
    va_end(arguments);

    if (strstr(line, "containment cause") != nullptr) {
        ++world->containments;
    }

    if (strstr(line, "retirement teardown contained") != nullptr) {
        ++world->retained_reboot;
    }

    const size_t length = strlen(line);

    if (world->log_used + length + 1 < sizeof(world->log)) {
        memcpy(world->log + world->log_used, line, length);
        world->log_used += length;
        world->log[world->log_used] = 0;
    }

    return 0;
}

VOID NTAPI RtlAssert(PVOID assertion, PVOID, ULONG line, PCHAR) {
    char text[120];
    snprintf(text, sizeof(text), "NT_ASSERT failed: %s line %lu", static_cast<const char*>(assertion), line);
    violate(text);
}

VOID NTAPI RtlInitUnicodeString(PUNICODE_STRING target, PCWSTR source) {
    size_t length = 0;
    while (source != nullptr && source[length] != 0) { ++length; }
    target->Buffer = const_cast<PWSTR>(source);
    target->Length = static_cast<USHORT>(length * sizeof(WCHAR));
    target->MaximumLength = static_cast<USHORT>(target->Length + sizeof(WCHAR));
}

KIRQL NTAPI KeGetCurrentIrql() { return PASSIVE_LEVEL; }
PKTHREAD NTAPI KeGetCurrentThread() { return reinterpret_cast<PKTHREAD>(world->current->thread_token); }
PEPROCESS NTAPI IoGetCurrentProcess() {
    return reinterpret_cast<PEPROCESS>(process_storage[world->current->process]);
}
PEPROCESS NTAPI IoGetRequestorProcess(PIRP irp) {
    Actor* requestor = info(irp)->requestor;
    return reinterpret_cast<PEPROCESS>(process_storage[requestor != nullptr ? requestor->process : 0]);
}

KIRQL FASTCALL KfAcquireSpinLock(PKSPIN_LOCK lock) {
    driver_code();
    check_pointer(lock, "spin lock");
    if (*lock != 0) { violate("spin lock held across a scheduling point"); }
    *lock = 1;
    return PASSIVE_LEVEL;
}
VOID FASTCALL KfReleaseSpinLock(PKSPIN_LOCK lock, KIRQL) {
    check_pointer(lock, "spin lock release");
    *lock = 0;
}

VOID NTAPI KeInitializeEvent(PRKEVENT event, EVENT_TYPE type, BOOLEAN state) {
    event->Header.Type = static_cast<UCHAR>(type);
    event->Header.SignalState = state ? 1 : 0;
}
LONG NTAPI KeSetEvent(PRKEVENT event, KPRIORITY, BOOLEAN) {
    driver_code();
    check_pointer(event, "event set");
    const LONG previous = event->Header.SignalState;
    event->Header.SignalState = 1;
    return previous;
}
VOID NTAPI KeClearEvent(PRKEVENT event) {
    check_pointer(event, "event clear");
    event->Header.SignalState = 0;
}
VOID NTAPI KeInitializeMutex(PRKMUTEX mutex, ULONG) {
    memset(mutex, 0, sizeof(*mutex));
    mutex->Header.Type = mutant_object_type;
    mutex->Header.SignalState = 1;
}
LONG NTAPI KeReleaseMutex(PRKMUTEX mutex, BOOLEAN) {
    driver_code();
    check_pointer(mutex, "guard release");
    if (mutex->OwnerThread != KeGetCurrentThread()) { violate("guard released by a non-owner"); }
    if (++mutex->Header.SignalState == 1) { mutex->OwnerThread = nullptr; }
    preempt();
    return 0;
}
NTSTATUS NTAPI KeWaitForSingleObject(PVOID object, KWAIT_REASON, KPROCESSOR_MODE, BOOLEAN, PLARGE_INTEGER) {
    driver_code();
    check_pointer(object, "wait object");
    auto* header = static_cast<DISPATCHER_HEADER*>(object);

    if (header->Type == mutant_object_type) {
        auto* mutex = static_cast<PRKMUTEX>(object);
        preempt();
        block_until(Block::mutex, mutex);
        driver_code();
        check_pointer(object, "guard acquired after wait");
        mutex->OwnerThread = KeGetCurrentThread();
        --mutex->Header.SignalState;
        return STATUS_SUCCESS;
    }

    block_until(Block::event, object);
    driver_code();

    if (header->Type == SynchronizationEvent) {
        header->SignalState = 0;
    }

    return STATUS_SUCCESS;
}
NTSTATUS NTAPI KeDelayExecutionThread(KPROCESSOR_MODE, BOOLEAN, PLARGE_INTEGER interval) {
    driver_code();
    world->clock += static_cast<uint64_t>(-interval->QuadPart);
    preempt();
    return STATUS_SUCCESS;
}

LONG_PTR FASTCALL ObfReferenceObject(PVOID object) {
    check_pointer(object, "object reference");
    reference(static_cast<PDEVICE_OBJECT>(object));
    return 1;
}
LONG_PTR FASTCALL ObfDereferenceObject(PVOID object) {
    check_pointer(object, "object dereference");
    dereference(static_cast<PDEVICE_OBJECT>(object));
    return 0;
}

NTSTATUS NTAPI IoCreateDevice(PDRIVER_OBJECT driver, ULONG extension_bytes, PUNICODE_STRING name, DEVICE_TYPE type,
                              ULONG, BOOLEAN, PDEVICE_OBJECT* output) {
    driver_code();
    FakeDevice* device = new_device(extension_bytes, true);
    device->object.DriverObject = driver;
    device->object.DeviceType = type;
    device->object.Flags = DO_DEVICE_INITIALIZING;
    device->object.NextDevice = driver->DeviceObject;
    driver->DeviceObject = &device->object;

    if (name != nullptr) {
        world->control = device;
    } else {
        world->filter = device;
    }

    *output = &device->object;
    return STATUS_SUCCESS;
}
VOID NTAPI IoDeleteDevice(PDEVICE_OBJECT object) {
    driver_code();
    FakeDevice* device = nullptr;

    if (!is_device(object, &device) || device->deleted) {
        violate("IoDeleteDevice of an unknown or already deleted device");
        return;
    }

    device->deleted = true;

    if (object->ReferenceCount == 0) {
        unload_device(*device);
    }
}
PDEVICE_OBJECT NTAPI IoAttachDeviceToDeviceStack(PDEVICE_OBJECT source, PDEVICE_OBJECT target) {
    PDEVICE_OBJECT top = top_of(target);
    top->AttachedDevice = source;
    source->StackSize = static_cast<CCHAR>(top->StackSize + 1);
    return top;
}
VOID NTAPI IoDetachDevice(PDEVICE_OBJECT target) {
    driver_code();
    target->AttachedDevice = nullptr;
}
NTSTATUS NTAPI IoCreateSymbolicLink(PUNICODE_STRING, PUNICODE_STRING) { world->link = true; return STATUS_SUCCESS; }
NTSTATUS NTAPI IoDeleteSymbolicLink(PUNICODE_STRING) { world->link = false; return STATUS_SUCCESS; }

VOID NTAPI IoInitializeRemoveLockEx(PIO_REMOVE_LOCK lock, ULONG, ULONG, ULONG, ULONG) {
    lock->Common.Removed = FALSE;
    lock->Common.IoCount = 1;
    KeInitializeEvent(&lock->Common.RemoveEvent, NotificationEvent, FALSE);
}
NTSTATUS NTAPI IoAcquireRemoveLockEx(PIO_REMOVE_LOCK lock, PVOID, PCSTR, ULONG, ULONG) {
    driver_code();
    check_pointer(lock, "remove lock acquire");
    ++lock->Common.IoCount;

    if (lock->Common.Removed) {
        if (--lock->Common.IoCount == 0) {
            lock->Common.RemoveEvent.Header.SignalState = 1;
        }
        return STATUS_DELETE_PENDING;
    }

    return STATUS_SUCCESS;
}
VOID NTAPI IoReleaseRemoveLockEx(PIO_REMOVE_LOCK lock, PVOID, ULONG) {
    driver_code();
    check_pointer(lock, "remove lock release");
    if (lock->Common.IoCount <= 0) { violate("remove lock released below zero"); }
    if (--lock->Common.IoCount == 0) {
        lock->Common.RemoveEvent.Header.SignalState = 1;
    }
}
VOID NTAPI IoReleaseRemoveLockAndWaitEx(PIO_REMOVE_LOCK lock, PVOID tag, ULONG size) {
    driver_code();
    check_pointer(lock, "remove lock drain");
    lock->Common.Removed = TRUE;
    --lock->Common.IoCount;
    IoReleaseRemoveLockEx(lock, tag, size);

    if (++world->and_wait_calls == 2) {
        world->cleanup_gate_closed = true;
    }

    block_until(Block::event, &lock->Common.RemoveEvent);
    driver_code();
}

PIRP NTAPI IoAllocateIrp(CCHAR stack_size, BOOLEAN) { driver_code(); return allocate_irp(stack_size); }
VOID NTAPI IoFreeIrp(PIRP irp) { driver_code(); free_irp(irp); }
VOID NTAPI IoReuseIrp(PIRP irp, NTSTATUS status) {
    const CCHAR stack = irp->StackCount;
    IoInitializeIrp(irp, IoSizeOfIrp(stack), stack);
    irp->IoStatus.Status = status;
    info(irp)->completions = 0;
    info(irp)->done = false;
}
VOID NTAPI IoInitializeIrp(PIRP irp, USHORT size, CCHAR stack) {
    memset(irp, 0, size);
    irp->Type = IO_TYPE_IRP;
    irp->Size = size;
    irp->StackCount = stack;
    irp->CurrentLocation = static_cast<CCHAR>(stack + 1);
    irp->Tail.Overlay.CurrentStackLocation = reinterpret_cast<PIO_STACK_LOCATION>(irp + 1) + stack;
}
PIRP NTAPI IoBuildDeviceIoControlRequest(ULONG code, PDEVICE_OBJECT device, PVOID input, ULONG input_bytes,
                                        PVOID output, ULONG output_bytes, BOOLEAN internal, PKEVENT event,
                                        PIO_STATUS_BLOCK iosb) {
    driver_code();
    PIRP irp = allocate_irp(device->StackSize);
    FakeIrp* record = info(irp);
    record->built = true;
    record->event = event;
    record->iosb = iosb;
    record->user_buffer = output;
    record->output_bytes = output_bytes;
    const ULONG bytes = input_bytes > output_bytes ? input_bytes : output_bytes;
    record->system_buffer = calloc(1, bytes + 1);
    irp->AssociatedIrp.SystemBuffer = bytes != 0 ? record->system_buffer : nullptr;

    if (input != nullptr && input_bytes != 0) {
        memcpy(record->system_buffer, input, input_bytes);
    }

    irp->RequestorMode = KernelMode;
    auto* next = IoGetNextIrpStackLocation(irp);
    next->MajorFunction = internal ? IRP_MJ_INTERNAL_DEVICE_CONTROL : IRP_MJ_DEVICE_CONTROL;
    next->Parameters.DeviceIoControl.IoControlCode = code;
    next->Parameters.DeviceIoControl.InputBufferLength = input_bytes;
    next->Parameters.DeviceIoControl.OutputBufferLength = output_bytes;
    return irp;
}
NTSTATUS FASTCALL IofCallDriver(PDEVICE_OBJECT device, PIRP irp) {
    driver_code();
    check_pointer(device, "IoCallDriver target");
    const NTSTATUS status = call_driver(device, irp);
    driver_code();
    return status;
}
VOID FASTCALL IofCompleteRequest(PIRP irp, CCHAR) {
    driver_code();
    complete_irp(irp);
    preempt();
}
NTSTATUS NTAPI PoCallDriver(PDEVICE_OBJECT device, PIRP irp) { return IofCallDriver(device, irp); }
VOID NTAPI PoStartNextPowerIrp(PIRP) { driver_code(); }
POWER_STATE NTAPI PoSetPowerState(PDEVICE_OBJECT, POWER_STATE_TYPE, POWER_STATE state) { return state; }

PIO_WORKITEM NTAPI IoAllocateWorkItem(PDEVICE_OBJECT device) {
    driver_code();
    WorkItem& item = world->work_items[world->work_count++];
    item.device = device;
    return reinterpret_cast<PIO_WORKITEM>(&item);
}
VOID NTAPI IoFreeWorkItem(PIO_WORKITEM item) {
    driver_code();
    reinterpret_cast<WorkItem*>(item)->device = nullptr;
}
VOID NTAPI IoQueueWorkItem(PIO_WORKITEM handle, PIO_WORKITEM_ROUTINE routine, WORK_QUEUE_TYPE, PVOID context) {
    driver_code();
    auto& item = *reinterpret_cast<WorkItem*>(handle);

    if (item.queued) {
        violate("work item queued twice");
        return;
    }

    item.queued = true;
    item.routine = routine;
    item.context = context;
    reference(item.device);
    spawn("work-item", [](Actor& actor) {
        auto& work = *static_cast<WorkItem*>(actor.argument);
        PDEVICE_OBJECT device = work.device;
        yield();
        work.routine(device, work.context);
        // IopWorkItemCallback (ntoskrnl code): drop the reference afterwards.
        dereference(device);
    }, 0, &item);
}

NTSTATUS NTAPI PsCreateSystemThread(PHANDLE handle, ULONG, POBJECT_ATTRIBUTES, HANDLE, PCLIENT_ID,
                                    PKSTART_ROUTINE start, PVOID context) {
    driver_code();
    struct Start final { PKSTART_ROUTINE routine; PVOID context; };
    static Start starts[4];
    static int next = 0;
    Start& entry = starts[next++ % 4];
    entry = {start, context};
    Actor& actor = spawn("power-thread", [](Actor& self) {
        auto& routine = *static_cast<Start*>(self.argument);
        routine.routine(routine.context);
    }, 0, &entry);
    *handle = reinterpret_cast<HANDLE>(static_cast<ULONG_PTR>(0x1000 + actor.index * 4));
    return STATUS_SUCCESS;
}
NTSTATUS NTAPI PsTerminateSystemThread(NTSTATUS) {
    driver_code();
    world->current->finished = true;

    for (;;) {
        SwitchToFiber(world->scheduler);
    }
}
NTSTATUS NTAPI ZwWaitForSingleObject(HANDLE handle, BOOLEAN, PLARGE_INTEGER) {
    driver_code();
    const int index = static_cast<int>((reinterpret_cast<ULONG_PTR>(handle) - 0x1000) / 4);
    block_until(Block::actor, &world->actors[index]);
    driver_code();
    return STATUS_SUCCESS;
}
NTSTATUS NTAPI ZwClose(HANDLE) { driver_code(); return STATUS_SUCCESS; }
NTSTATUS NTAPI PsSetCreateProcessNotifyRoutine(PCREATE_PROCESS_NOTIFY_ROUTINE routine, BOOLEAN remove) {
    world->notify = remove ? nullptr : routine;
    return STATUS_SUCCESS;
}
} // extern "C"

// ---- Transport and renderer contract models ---------------------------------
namespace lightning::reactos {
NTSTATUS NativeTransport::start(PDEVICE_OBJECT, PCM_RESOURCE_LIST, PCM_RESOURCE_LIST) noexcept {
    driver_code();
    ready_ = true;
    world->transport_ready = true;
    return STATUS_SUCCESS;
}
NTSTATUS NativeTransport::stop(bool) noexcept {
    driver_code();
    ready_ = false;
    world->transport_ready = false;
    return STATUS_SUCCESS;
}
NTSTATUS NativeTransport::suspend() noexcept { return stop(true); }
NTSTATUS NativeTransport::resume() noexcept {
    driver_code();
    ready_ = true;
    world->transport_ready = true;
    return STATUS_SUCCESS;
}

namespace {
Session* session_of(PFILE_OBJECT file) {
    for (auto& session : world->sessions) {
        if (session.file != nullptr && session.file == file) {
            return &session;
        }
    }
    return nullptr;
}

// Settle one pin through the installed owner (resolve_display_receipt).
bool settle(const ReceiptOwner& owner, Session& session, bool wait) {
    if (!session.pinned) {
        return true;
    }

    const ReceiptState state = owner.resolve != nullptr ? owner.resolve(owner.context, session.pin, wait) :
        ReceiptState::uncertain;

    if (state == ReceiptState::retired) {
        session.pinned = false;
        ++world->pins_by_proof;
        return true;
    }

    return state == ReceiptState::pending && !wait;
}

// The host image behind a lease is freed: no lower read may still use it.
void release_host_image(Session& session) {
    if (session.lease != 0 && world->lower.read_lease == session.lease &&
        world->lower.completed < world->lower.read_sequence) {
        violate("host image freed while the canvas read of it is outstanding");
    }
}
} // namespace

NTSTATUS NativeRenderer::initialize(NativeTransport& transport) noexcept {
    driver_code();
    if (quarantined_) { return STATUS_DEVICE_NOT_READY; }
    if (!transport.ready()) { violate("renderer initialized on a stopped transport"); }
    transport_ = &transport;
    active_ = true;
    return STATUS_SUCCESS;
}
NTSTATUS NativeRenderer::open_file(PFILE_OBJECT file) noexcept {
    driver_code();
    if (!ready()) { return STATUS_DEVICE_NOT_READY; }

    for (auto& session : world->sessions) {
        if (session.file == nullptr) {
            session = {};
            session.file = file;
            session.process = world->current->process;
            session.lease = 0x100 + static_cast<uint64_t>(&session - world->sessions);
            file->FsContext = &session;
            return STATUS_SUCCESS;
        }
    }

    return STATUS_INSUFFICIENT_RESOURCES;
}
NTSTATUS NativeRenderer::close_file(PFILE_OBJECT file) noexcept {
    driver_code();
    Session* session = session_of(file);
    if (session == nullptr) { return STATUS_SUCCESS; }

    if (quarantined_) {
        file->FsContext = nullptr;
        session->file = nullptr;
        return STATUS_DEVICE_BUSY;
    }

    if (!settle(receipt_owner_, *session, true)) {
        quarantined_ = true;
        active_ = false;
        world->renderer_contained = true;
        return STATUS_DEVICE_BUSY;
    }

    release_host_image(*session);
    *session = {};
    file->FsContext = nullptr;
    return STATUS_SUCCESS;
}
NTSTATUS NativeRenderer::exchange_file(PFILE_OBJECT file, PEPROCESS requestor, rx7900::ByteView,
                                       rx7900::sdma::CommandBuffer, size_t& written) noexcept {
    driver_code();
    written = 0;
    Session* session = session_of(file);

    if (session == nullptr || quarantined_ || session->exiting || session->lost ||
        requestor != reinterpret_cast<PEPROCESS>(process_storage[session->process])) {
        ++world->exchange_refusals;
        return STATUS_ACCESS_DENIED;
    }

    if (!world->transport_ready || transport_ == nullptr || !transport_->ready()) {
        violate("renderer exchange ran on a stopped or suspended transport");
    }

    // A bounded host exchange and GPU wait.
    for (unsigned pause = 0; pause < 2; ++pause) {
        world->clock += 10000;
        preempt();
    }

    if (!world->transport_ready) {
        violate("transport stopped while an admitted exchange was in flight");
    }

    ++world->exchanges;
    return STATUS_SUCCESS;
}
NTSTATUS NativeRenderer::exchange_batch(PFILE_OBJECT file, PEPROCESS requestor, rx7900::ByteView input,
                                        rx7900::sdma::CommandBuffer output, size_t& written) noexcept {
    return exchange_file(file, requestor, input, output, written);
}
NTSTATUS NativeRenderer::resolve_image(uint64_t lease, PEPROCESS process, NativeImageLease& output,
                                       const ProducerPoint*) noexcept {
    driver_code();
    output = {};
    if (!ready()) { return STATUS_ACCESS_DENIED; }

    for (auto& session : world->sessions) {
        if (session.file != nullptr && session.lease == lease && !session.exiting && !session.lost &&
            process == reinterpret_cast<PEPROCESS>(process_storage[session.process])) {
            world->clock += 10000;
            preempt();
            output.context = 0x7000 + lease;
            output.resource = 0x8000 + lease;
            output.layout = {64, 32, 256, 0, static_cast<uint32_t>(QLV_SCANOUT_BGRA8)};
            return STATUS_SUCCESS;
        }
    }

    return STATUS_ACCESS_DENIED;
}
bool NativeRenderer::can_track_receipt(uint64_t lease, PEPROCESS process) const noexcept {
    driver_code();
    if (!async_receipts()) { return false; }

    for (const auto& session : world->sessions) {
        if (session.file != nullptr && session.lease == lease &&
            process == reinterpret_cast<PEPROCESS>(process_storage[session.process])) {
            return true;
        }
    }
    return false;
}
bool NativeRenderer::track_receipt(uint64_t lease, PEPROCESS process, const ImageReceipt& receipt) noexcept {
    driver_code();
    for (auto& session : world->sessions) {
        if (session.file != nullptr && session.lease == lease &&
            process == reinterpret_cast<PEPROCESS>(process_storage[session.process])) {
            session.pin = receipt;
            session.pinned = true;
            return true;
        }
    }
    return false;
}
ReceiptState NativeRenderer::query_receipt(uint64_t lease, PEPROCESS process) noexcept {
    driver_code();
    for (auto& session : world->sessions) {
        if (session.file != nullptr && session.lease == lease &&
            process == reinterpret_cast<PEPROCESS>(process_storage[session.process])) {
            if (!settle(receipt_owner_, session, false)) {
                quarantined_ = true;
                active_ = false;
                world->renderer_contained = true;
                return ReceiptState::uncertain;
            }
            return session.pinned ? ReceiptState::pending : ReceiptState::retired;
        }
    }
    return ReceiptState::retired;
}
void NativeRenderer::retire_receipts_after_display_reset() noexcept {
    driver_code();
    for (auto& session : world->sessions) {
        if (session.pinned) {
            session.pinned = false;
            ++world->pins_by_reset;
        }
    }
}
void NativeRenderer::process_exit(HANDLE process_id) noexcept {
    driver_code();
    const int process = static_cast<int>((reinterpret_cast<ULONG_PTR>(process_id) - 100) / 4);

    for (auto& session : world->sessions) {
        if (session.file == nullptr || session.process != process || session.exiting) {
            continue;
        }

        session.exiting = true;
        session.views = false;

        if (!quarantined_ && !settle(receipt_owner_, session, true)) {
            quarantined_ = true;
            active_ = false;
            world->renderer_contained = true;
            continue;
        }

        if (!quarantined_) {
            release_host_image(session);
            session.lost = true;
        }
    }
}
NTSTATUS NativeRenderer::shutdown(bool allow_host) noexcept {
    driver_code();
    active_ = false;
    NTSTATUS result = quarantined_ ? STATUS_DEVICE_BUSY : STATUS_SUCCESS;

    for (auto& session : world->sessions) {
        if (session.file == nullptr || session.lost) {
            continue;
        }

        if (!session.exiting && session.views) {
            quarantined_ = true;
            world->renderer_contained = true;
            result = STATUS_DEVICE_BUSY;
            continue;
        }

        if (!allow_host || quarantined_ || !settle(receipt_owner_, session, true)) {
            // Without host access a pinned read stays contained, never dropped.
            quarantined_ = true;
            world->renderer_contained = true;
            result = STATUS_DEVICE_BUSY;
            continue;
        }

        release_host_image(session);
        session.lost = true;
    }

    return result;
}
} // namespace lightning::reactos

// ---- Scenarios ---------------------------------------------------------------
namespace {
struct Outcome final {
    unsigned runs = 0;
    unsigned violations = 0;
    unsigned deleted = 0;
    unsigned contained = 0;
    unsigned retained = 0;
    char first[200]{};
    uint32_t first_seed = 0;
};

FakeDevice* filter_device() {
    for (int index = 0; index < world->device_count; ++index) {
        FakeDevice* device = world->devices[index];
        if (device->ours && device->extension_bytes > sizeof(int) * 4) {
            return device;
        }
    }
    return nullptr;
}

// Writes after free land in the poison: detect them at the end of a run.
void check_poison() {
    for (int index = 0; index < world->device_count; ++index) {
        const FakeDevice& device = *world->devices[index];

        if (!device.freed || device.extension == nullptr) {
            continue;
        }

        for (size_t byte = 0; byte < device.extension_bytes; ++byte) {
            if (device.extension[byte] != 0xdd) {
                violate("write into a deleted device extension");
                return;
            }
        }
    }
}

void record(Outcome& outcome, uint32_t seed, const char* expectation) {
    ++outcome.runs;
    check_poison();

    if (!world->aborted && expectation != nullptr) {
        violate(expectation);
    }

    if (world->aborted) {
        ++outcome.violations;

        if (outcome.first[0] == 0) {
            snprintf(outcome.first, sizeof(outcome.first), "%s", world->violation.text);
            outcome.first_seed = seed;

            if (getenv("OVERLAP_LOG") != nullptr) {
                printf("%s", world->log);
            }
        }
    }

    FakeDevice* filter = filter_device();

    if (filter != nullptr && filter->freed) {
        ++outcome.deleted;
    }

    if (world->containments != 0 || world->renderer_contained) {
        ++outcome.contained;
    }

    if (world->retained_reboot != 0) {
        ++outcome.retained;
    }
}

using Scenario = const char* (*)();

Outcome explore(Scenario scenario, unsigned runs) {
    Outcome outcome;

    for (unsigned seed = 1; seed <= runs; ++seed) {
        reset_world(seed);
        static Files storage;
        storage = Files{};
        files = &storage;
        DriverEntry(&world->driver, nullptr);
        scenario();
        run_scheduler();
        record(outcome, seed, nullptr);
    }

    return outcome;
}

void report(const char* name, const Outcome& outcome) {
    printf("%-34s runs %4u violations %4u deleted %4u contained %4u retained %3u%s%s\n", name, outcome.runs,
        outcome.violations, outcome.deleted, outcome.contained, outcome.retained,
        outcome.first[0] != 0 ? "  first: " : "", outcome.first);
    if (outcome.first[0] != 0) {
        printf("  (seed %lu)\n", static_cast<unsigned long>(outcome.first_seed));
    }
}

// Scenario end-state checks run in a final actor that waits for the others.
struct Join final {
    Actor* actors[8]{};
    int count = 0;
    const char* (*verdict)() = nullptr;
    const char* result = nullptr;
};
Join join_state;

void join_body(Actor&) {
    for (int index = 0; index < join_state.count; ++index) {
        while (!join_state.actors[index]->finished && !world->aborted) {
            block_until(Block::actor, join_state.actors[index]);
        }
    }

    // Let queued work items (teardown) run until nothing else can progress;
    // an actor blocked forever (a stuck lower request) is left blocked.
    for (unsigned spin = 0; spin < 100000; ++spin) {
        bool pending = false;

        for (int index = 0; index < world->count; ++index) {
            Actor& actor = world->actors[index];
            if (&actor != world->current && runnable(actor) && strcmp(actor.name, "power-thread") != 0) {
                pending = true;
            }
        }

        if (!pending) {
            break;
        }

        world->clock += 10000;
        yield();
    }

    if (join_state.verdict != nullptr) {
        const char* verdict = join_state.verdict();
        if (verdict != nullptr) { violate(verdict); }
    }
}

void begin_join(const char* (*verdict)()) {
    join_state = Join{};
    join_state.verdict = verdict;
}
void joined(Actor& actor) { join_state.actors[join_state.count++] = &actor; }
void end_join() { world->join = &spawn("join", join_body); }

// Common prefix: START (synchronously in its own actor), then the scenario.
Actor& starter() { return spawn("pnp-start", start_device); }

// S1: GL exchanges and a kernel async present race Device Manager disable
// (QUERY_REMOVE, REMOVE) and the process exit that follows; win32k then
// transfers its lease and closes. The filter must retire through the channel.
const char* verdict_remove() {
    FakeDevice* filter = filter_device();
    if (world->display_opened && world->lower.closes != 1) { return "win32k CLOSE lost or duplicated"; }
    if (world->containments == 0 && (filter == nullptr || !filter->freed)) {
        return "retirement channel was never torn down after its last reference";
    }
    // The image-wide containment count (an earlier contained run in this
    // process) legitimately keeps DriverUnload disarmed.
    if (world->containments == 0 && !world->unload_invoked && strstr(world->log, "unload stays disarmed") == nullptr) {
        return "driver never unloaded after removal";
    }
    return nullptr;
}

const char* scenario_remove_overlap() {
    starter();
    world->clients_pending = 2;
    spawn("host", host_body);
    begin_join(verdict_remove);
    joined(spawn("gl-app", [](Actor&) {
        wait_flag(&world->started_flag);
        const NTSTATUS opened = open_control(&files->control);
        client_opened();
        if (!NT_SUCCESS(opened)) { return; }
        for (unsigned frame = 0; frame < 4 && !world->remove_done; ++frame) {
            exchange(&files->control);
            uint32_t status = 0;
            present_async(0x100);
            receipt_query(0x100, status);
        }
        wait_flag(&world->remove_done);
        process_exit(1);
        world->exit_done = true;
        close_control(&files->control);
    }, 1));
    joined(spawn("win32k", [](Actor&) {
        wait_flag(&world->started_flag);
        display_open();
        client_opened();
        send_ioctl(&world->filter->object, IOCTL_VIDEO_SET_CURRENT_MODE, nullptr, 0, nullptr, 0);
        wait_flag(&world->abandon_seen);
        // Its next drawing call notices the loss and transfers the lease; a
        // refusal inside the REMOVE window is retried by PDEV teardown, which
        // follows the removal notification and precedes the final close.
        resolve_release();
        wait_flag(&world->remove_done);
        display_close();
    }));
    joined(spawn("pnp", [](Actor&) {
        wait_for_clients();
        for (unsigned pause = 0; pause < next_random() % 6; ++pause) { yield(); }
        if (NT_SUCCESS(send_pnp(IRP_MN_QUERY_REMOVE_DEVICE))) {
            send_pnp(IRP_MN_REMOVE_DEVICE);
        }
        world->remove_done = true;
    }));
    end_join();
    return nullptr;
}

// S2: SET_POWER D3 then D0 through the precreated power thread while the GL
// process exchanges and presents asynchronously; the pinned read must be
// retired by the confirmed display quiesce, never dropped.
const char* verdict_power() {
    if (world->lower.quiesced && !world->lower.lost) { return "D0 did not resume the display"; }
    if (!world->transport_ready) { return "D0 did not resume the transport"; }
    return nullptr;
}

const char* scenario_power_overlap() {
    starter();
    world->clients_pending = 1;
    spawn("host", host_body);
    begin_join(verdict_power);
    joined(spawn("gl-app", [](Actor&) {
        wait_flag(&world->started_flag);
        const NTSTATUS opened = open_control(&files->control);
        client_opened();
        if (!NT_SUCCESS(opened)) { return; }
        for (unsigned frame = 0; frame < 5; ++frame) {
            exchange(&files->control);
            uint32_t status = 0;
            present_async(0x100);
            receipt_query(0x100, status);
        }
        close_control(&files->control);
    }, 1));
    joined(spawn("power", [](Actor&) {
        wait_for_clients();
        for (unsigned pause = 0; pause < next_random() % 8; ++pause) { yield(); }
        send_power(PowerDeviceD3);
        world->d3_done = true;
        for (unsigned pause = 0; pause < next_random() % 4; ++pause) { yield(); }
        send_power(PowerDeviceD0);
    }));
    end_join();
    return nullptr;
}

// S3: surprise removal (QMP device_del) under exchanges, a pinned present and
// a display hardware request, then REMOVE, process exit and win32k retirement.
const char* verdict_surprise() {
    if (world->display_opened && world->lower.closes != 1) { return "win32k CLOSE lost or duplicated"; }
    return nullptr;
}

const char* scenario_surprise_overlap() {
    starter();
    world->clients_pending = 2;
    begin_join(verdict_surprise);
    joined(spawn("gl-app", [](Actor&) {
        wait_flag(&world->started_flag);
        const NTSTATUS opened = open_control(&files->control);
        client_opened();
        if (!NT_SUCCESS(opened)) { return; }
        for (unsigned frame = 0; frame < 4 && !world->remove_done; ++frame) {
            exchange(&files->control);
            present_async(0x100);
        }
        wait_flag(&world->remove_done);
        process_exit(1);
        close_control(&files->control);
    }, 1));
    joined(spawn("win32k", [](Actor&) {
        wait_flag(&world->started_flag);
        display_open();
        client_opened();
        send_ioctl(&world->filter->object, IOCTL_VIDEO_SET_CURRENT_MODE, nullptr, 0, nullptr, 0);
        wait_flag(&world->abandon_seen);
        // Its next drawing call notices the loss and transfers the lease; a
        // refusal inside the REMOVE window is retried by PDEV teardown, which
        // follows the removal notification and precedes the final close.
        resolve_release();
        wait_flag(&world->remove_done);
        display_close();
    }));
    joined(spawn("pnp", [](Actor&) {
        wait_for_clients();
        for (unsigned pause = 0; pause < next_random() % 6; ++pause) { yield(); }
        send_pnp(IRP_MN_SURPRISE_REMOVAL);
        send_pnp(IRP_MN_REMOVE_DEVICE);
        world->remove_done = true;
    }));
    end_join();
    return nullptr;
}

// S4: two GL processes (one holding a duplicate of the other's handle) exit
// while their exchanges and presents are in flight; STOP follows.
const char* verdict_exits() {
    for (const auto& session : world->sessions) {
        if (session.file != nullptr && session.pinned && !world->renderer_contained) {
            return "a pinned read survived its process exit uncontained";
        }
    }
    return nullptr;
}

const char* scenario_process_exits() {
    starter();
    world->clients_pending = 2;
    spawn("host", host_body);
    files->control_handles = 2;
    begin_join(verdict_exits);
    joined(spawn("gl-a", [](Actor&) {
        wait_flag(&world->started_flag);
        const NTSTATUS opened = open_control(&files->control);
        client_opened();
        if (!NT_SUCCESS(opened)) { return; }
        for (unsigned frame = 0; frame < 3; ++frame) {
            exchange(&files->control);
            present_async(0x100);
        }
        process_exit(1);
        close_control(&files->control);
    }, 1));
    joined(spawn("gl-b", [](Actor&) {
        wait_flag(&world->started_flag);
        const NTSTATUS opened = open_control(&files->second);
        client_opened();
        if (!NT_SUCCESS(opened)) { return; }
        for (unsigned frame = 0; frame < 3; ++frame) {
            exchange(&files->second);
            // The duplicate handle of process 1's file: refused, never crossed.
            exchange(&files->control);
        }
        process_exit(2);
        close_control(&files->second);
        // Process 2's duplicate handle of process 1's file goes with it.
        close_control(&files->control);
    }, 2));
    joined(spawn("pnp", [](Actor&) {
        wait_for_clients();
        for (unsigned pause = 0; pause < next_random() % 10; ++pause) { yield(); }
        if (NT_SUCCESS(send_pnp(IRP_MN_QUERY_STOP_DEVICE))) {
            send_pnp(IRP_MN_STOP_DEVICE);
        }
    }));
    end_join();
    return nullptr;
}

// S5: win32k's final CLOSE races the REMOVE window between the cleanup gate
// closing and the retirement channel opening; a stale display IOCTL and a
// duplicate (slow) lease resolve are in flight while the teardown runs.
const char* verdict_window() {
    if (world->display_opened && world->lower.closes != 1) { return "win32k CLOSE refused unforwarded during REMOVE"; }
    FakeDevice* filter = filter_device();
    if (world->containments == 0 && world->retained_reboot == 0 && (filter == nullptr || !filter->freed)) {
        return "retirement channel was never torn down after its last reference";
    }
    return nullptr;
}

const char* scenario_remove_window() {
    starter();
    world->clients_pending = 1;
    world->lower.slow_resolve = 40 + next_random() % 140;
    begin_join(verdict_window);
    joined(spawn("win32k-close", [](Actor&) {
        wait_flag(&world->started_flag);
        display_open();
        client_opened();
        wait_flag(&world->abandon_seen);
        resolve_release();
        // Close at a random point around the cleanup gate.
        if (next_random() % 2 == 0) { wait_flag(&world->cleanup_gate_closed); }
        for (unsigned pause = 0; pause < next_random() % 4; ++pause) { yield(); }
        wait_flag(&world->stale_inside);
        display_close();
    }));
    joined(spawn("win32k-stale", [](Actor&) {
        wait_flag(&world->abandon_seen);
        wait_flag(&world->cleanup_gate_closed);
        // A duplicate release retry, then a stale inventory query, both through
        // the captured top-of-stack pointer (no file reference) and both
        // already inside the filter when win32k's final CLOSE may follow.
        world->in_flight = &world->stale_inside;
        resolve_release();
        if (world->display_opened && world->lower.closes == 0) {
            world->in_flight = &world->stale_second;
            send_ioctl(&world->filter->object, ioctl_query_cursor, nullptr, 0, nullptr, 0);
        }
    }));
    joined(spawn("pnp", [](Actor&) {
        wait_for_clients();
        if (NT_SUCCESS(send_pnp(IRP_MN_QUERY_REMOVE_DEVICE))) {
            send_pnp(IRP_MN_REMOVE_DEVICE);
        }
        world->remove_done = true;
    }));
    end_join();
    return nullptr;
}

// S7: the display client releases its last lease while REMOVE is between the
// display abandon and the remove-lock drain (it noticed the loss on its next
// drawing call), then win32k closes. The filter must still learn that no lease
// remains, or the retirement channel and the driver image are kept forever.
const char* scenario_release_during_remove() {
    starter();
    world->clients_pending = 1;
    begin_join(verdict_remove);
    joined(spawn("display-client", [](Actor&) {
        wait_flag(&world->started_flag);
        display_open();
        client_opened();
        wait_flag(&world->abandon_seen);
        resolve_release();
        wait_flag(&world->remove_done);
        display_close();
    }));
    joined(spawn("pnp", [](Actor&) {
        wait_for_clients();
        if (NT_SUCCESS(send_pnp(IRP_MN_QUERY_REMOVE_DEVICE))) {
            send_pnp(IRP_MN_REMOVE_DEVICE);
        }
        world->remove_done = true;
    }));
    end_join();
    return nullptr;
}

// S6: a reentrant call into the top of the stack from inside the lower
// device while the filter holds its guard must be refused, not nested.
const char* verdict_reentry() {
    if (!world->lower.reentry_done) { return "reentry was never attempted"; }
    if (world->lower.reentered != STATUS_INVALID_DEVICE_STATE) { return "reentrant request was not refused"; }
    return nullptr;
}

const char* scenario_reentry() {
    starter();
    world->clients_pending = 1;
    world->lower.reenter_code = IOCTL_VIDEO_SET_CURRENT_MODE;
    begin_join(verdict_reentry);
    joined(spawn("win32k", [](Actor&) {
        wait_flag(&world->started_flag);
        display_open();
        client_opened();
        send_ioctl(&world->filter->object, IOCTL_VIDEO_SET_CURRENT_MODE, nullptr, 0, nullptr, 0);
    }));
    end_join();
    return nullptr;
}
} // namespace

int main(int argc, char** argv) {
    const unsigned runs = argc > 1 ? static_cast<unsigned>(atoi(argv[1])) : 300;
    // This thread is the scheduler fiber; every actor is a fiber of it.
    PVOID scheduler = ConvertThreadToFiber(nullptr);
    reset_world(0);
    world->scheduler = scheduler;
    struct Entry final { const char* name; Scenario scenario; };
    const Entry entries[] = {
        {"remove vs exchange/present/exit", scenario_remove_overlap},
        {"D3/D0 vs exchange/async present", scenario_power_overlap},
        {"surprise vs exchange/present", scenario_surprise_overlap},
        {"process exits vs STOP", scenario_process_exits},
        {"REMOVE window vs CLOSE/stale", scenario_remove_window},
        {"lease release during REMOVE", scenario_release_during_remove},
        {"reentrant lower callback", scenario_reentry},
    };

    // Replay: <runs> <scenario> <seed> prints that run's filter log and state.
    if (argc > 3) {
        const auto& entry = entries[atoi(argv[2])];
        const auto seed = static_cast<uint32_t>(atoi(argv[3]));
        reset_world(seed);
        static Files storage;
        storage = Files{};
        files = &storage;
        DriverEntry(&world->driver, nullptr);
        entry.scenario();
        run_scheduler();
        check_poison();
        printf("%s seed %lu: %s\n%s", entry.name, static_cast<unsigned long>(seed),
            world->aborted ? world->violation.text : "no violation", world->log);
        printf("lower closes %u opens %u held %u removed %d lost %d; unload %d unmapped %d containments %u\n",
            world->lower.closes, world->lower.opens, world->lower.held_leases, world->lower.removed,
            world->lower.lost, world->unload_invoked, world->unmapped, world->containments);
        return 0;
    }

    // Each scenario runs in its own process: the driver image's globals (the
    // process-notification owner, the containment count) live as long as the
    // image, exactly as in the kernel, and must not leak between scenarios.
    const int count = static_cast<int>(sizeof(entries) / sizeof(entries[0]));

    if (argc != 3) {
        int failed = 0;
        char runs_text[16];
        snprintf(runs_text, sizeof(runs_text), "%u", runs);

        for (int index = 0; index < count; ++index) {
            char index_text[8];
            snprintf(index_text, sizeof(index_text), "%d", index);
            const char* arguments[] = {argv[0], runs_text, index_text, nullptr};
            fflush(stdout);
            const intptr_t result = _spawnv(_P_WAIT, argv[0], arguments);
            check(result == 0, entries[index].name);
            failed += result != 0 ? 1 : 0;
        }

        printf("%d scenarios, %d failed\n", count, failed);
        return failed == 0 ? 0 : 1;
    }

    const auto& entry = entries[atoi(argv[2]) % count];
    const Outcome outcome = explore(entry.scenario, entry.scenario == scenario_reentry ? 20 : runs);
    report(entry.name, outcome);
    check(outcome.violations == 0, entry.name);

    printf("%u driver overlap checks; %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

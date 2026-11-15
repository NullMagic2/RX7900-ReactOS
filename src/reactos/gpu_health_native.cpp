/*
Module: rx7900.reactos.gpu_health_native
Purpose: NT services of the thermal and RAS interrupt actions.
Created: 2026-10-24
Architecture: KSPIN_LOCK, KEVENT, a system thread and KeQueryInterruptTime
behind GpuHealthPlatform; actions reach the device only through
GpuHealthHooks. No thermal or RAS policy here.
*/
#include "gpu_health_native.hpp"

namespace rx7900::reactos {
namespace {

using hardware::HealthEvent;
using hardware::HealthEventKind;
using hardware::HealthReason;

// KeQueryInterruptTime counts 100 ns units.
constexpr ULONGLONG interrupt_time_per_microsecond = 10;
// Retry period while the device cannot act (D3, recovery in progress).
constexpr uint64_t blocked_retry_us = 1000000;

GpuHealthNative& native_of(void* context) noexcept {
    return *static_cast<GpuHealthNative*>(context);
}

uint64_t now_us(void*) noexcept {
    return KeQueryInterruptTime() / interrupt_time_per_microsecond;
}

// <= DISPATCH_LEVEL, never recursive: one saved IRQL suffices.
void lock(void* context) noexcept {
    auto& native = native_of(context);
    KIRQL irql = PASSIVE_LEVEL;
    KeAcquireSpinLock(&native.lock, &irql);
    native.held_irql = irql;
}

void unlock(void* context) noexcept {
    auto& native = native_of(context);
    KeReleaseSpinLock(&native.lock, native.held_irql);
}

void refuse(void* context, HealthReason reason) noexcept {
    auto& native = native_of(context);
    native.hooks.refuse(native.hooks.context, reason);
}

bool escalate(void* context, HealthReason reason) noexcept {
    auto& native = native_of(context);
    return native.hooks.escalate(native.hooks.context, reason);
}

uint32_t quarantine_vram(void* context, uint64_t vram_offset, uint64_t bytes, uint32_t& owners) noexcept {
    auto& native = native_of(context);
    return native.hooks.quarantine_vram(native.hooks.context, vram_offset, bytes, owners);
}

// PASSIVE_LEVEL (worker). One line per event; values as in gpu_health.hpp.
void log(void*, const HealthEvent& event) noexcept {
    const ULONG a = event.value0;
    const ULONG b = event.value1;
    const ULONG c = event.value2;

    switch (event.kind) {
    case HealthEventKind::hardware_ctf: {
        char message[] = "RX7900: ERROR: GPU HW Critical Temperature Fault (CTF) detected; GPU work refused\n";
        VideoPortDebugPrint(Error, message);
        break;
    }
    case HealthEventKind::thermal_alert: {
        char message[] = "RX7900: thermal alert (temperature above %u C): GFXCLK capped=%u, SW CTF check in 50 ms\n";
        VideoPortDebugPrint(Error, message, b, a);
        break;
    }
    case HealthEventKind::swctf_confirmed: {
        char message[] = "RX7900: ERROR: GPU over temperature range (SW CTF): hotspot %u C, limit %u C, read=%u\n";
        VideoPortDebugPrint(Error, message, a, b, c);
        break;
    }
    case HealthEventKind::swctf_cleared: {
        char message[] = "RX7900: thermal alert cleared: hotspot %u C below %u C; GFXCLK cap held\n";
        VideoPortDebugPrint(Error, message, a, b);
        break;
    }
    case HealthEventKind::thermal_restored: {
        char message[] = "RX7900: hotspot %u C: GFXCLK cap lifted (soft max %u MHz)\n";
        VideoPortDebugPrint(Error, message, a, b);
        break;
    }
    case HealthEventKind::thermal_held: {
        char message[] = "RX7900: hotspot %u C after %u checks: GFXCLK cap stays applied\n";
        VideoPortDebugPrint(Error, message, a, b);
        break;
    }
    case HealthEventKind::under_temperature: {
        char message[] = "RX7900: ERROR: GPU under temperature range detected (%u)\n";
        VideoPortDebugPrint(Error, message, a);
        break;
    }
    case HealthEventKind::throttling: {
        char message[] = "RX7900: WARN: GPU is throttled (throttler status 0x%x, hotspot %u C, %u suppressed)\n";
        VideoPortDebugPrint(Error, message, a, b, c);
        break;
    }
    case HealthEventKind::fan_abnormal: {
        char message[] = "RX7900: fan abnormal: soft CTF limit reduced to %u C (offset %d)\n";
        VideoPortDebugPrint(Error, message, a, static_cast<LONG>(b));
        break;
    }
    case HealthEventKind::fan_recovery: {
        char message[] = "RX7900: fan recovered: soft CTF limit %u C\n";
        VideoPortDebugPrint(Error, message, a);
        break;
    }
    case HealthEventKind::ac_dc: {
        char message[] = "RX7900: switched to %s power (acknowledgement status %u)\n";
        char ac[] = "AC";
        char dc[] = "DC";
        VideoPortDebugPrint(Error, message, a != 0 ? ac : dc, b);
        break;
    }
    case HealthEventKind::smu_failure: {
        char message[] = "RX7900: thermal action SMU status %u at step %u\n";
        VideoPortDebugPrint(Error, message, a, b);
        break;
    }
    case HealthEventKind::ras_athub_fatal: {
        char message[] = "RX7900: uncorrectable hardware error (ERREVENT_ATHUB) detected: vectors %u status %u\n";
        VideoPortDebugPrint(Error, message, a, b);
        break;
    }
    case HealthEventKind::ras_umc_ignored: {
        char message[] = "RX7900: UMC ECC interrupt without UMC RAS support ignored (%u)\n";
        VideoPortDebugPrint(Error, message, a);
        break;
    }
    case HealthEventKind::ras_umc_correctable: {
        char message[] = "RX7900: UMC correctable ECC errors: %u\n";
        VideoPortDebugPrint(Error, message, a);
        break;
    }
    case HealthEventKind::ras_umc_uncorrectable: {
        char message[] = "RX7900: UMC uncorrectable ECC errors: %u, pages retired %u, sessions quarantined %u\n";
        VideoPortDebugPrint(Error, message, a, b, c);
        break;
    }
    case HealthEventKind::ras_umc_unknown: {
        char message[] = "RX7900: UMC ECC interrupt with unreadable error records (%u): treated as uncorrectable\n";
        VideoPortDebugPrint(Error, message, a);
        break;
    }
    case HealthEventKind::ras_umc_page: {
        // umc_v8_10_convert_error_address "Error Address(PA): 0x%llx".
        char message[] = "RX7900: UMC error address (PA) 0x%x%08x, channel %u umc %u node %u\n";
        VideoPortDebugPrint(Error, message, b, a, c & 0xffu, (c >> 8) & 0xffu, c >> 16);
        break;
    }
    case HealthEventKind::ras_umc_undecoded: {
        char message[] = "RX7900: UMC uncorrectable records without usable address %u, overflowed %u, pages dropped %u\n";
        VideoPortDebugPrint(Error, message, a, b, c);
        break;
    }
    case HealthEventKind::ras_umc_uncontained: {
        char message[] = "RX7900: UMC bad page 0x%x%08x not contained by quarantine (%u)\n";
        VideoPortDebugPrint(Error, message, b, a, c);
        break;
    }
    case HealthEventKind::ras_umc_contained: {
        char message[] = "RX7900: UMC uncorrectable records %u contained: %u pages retired, no recovery\n";
        VideoPortDebugPrint(Error, message, a, b);
        break;
    }
    case HealthEventKind::ras_umc_counters: {
        char message[] = "RX7900: UMC ECC error counters armed=%u\n";
        VideoPortDebugPrint(Error, message, a);
        break;
    }
    case HealthEventKind::ras_bad_page_threshold: {
        char message[] = "RX7900: retired bad pages %u reach threshold value %u\n";
        VideoPortDebugPrint(Error, message, a, b);
        break;
    }
    case HealthEventKind::ras_gc_fed: {
        char message[] = "RX7900: RLC GC FED interrupt (%u); GC 11.0.0 has no handler\n";
        VideoPortDebugPrint(Error, message, a);
        break;
    }
    case HealthEventKind::page_retire_failed: {
        char message[] = "RX7900: VRAM page retirement failed (allocator status %u)\n";
        VideoPortDebugPrint(Error, message, a);
        break;
    }
    case HealthEventKind::refused: {
        char message[] = "RX7900: ERROR: GPU work refused from now on (reason %u)\n";
        VideoPortDebugPrint(Error, message, a);
        break;
    }
    case HealthEventKind::escalated: {
        char message[] = "RX7900: destructive recovery requested (reason %u)\n";
        VideoPortDebugPrint(Error, message, a);
        break;
    }
    }
}

struct RunCall final {
    GpuHealthNative* native;
    uint64_t delay_us;
};

void run_body(void* argument) noexcept {
    auto& call = *static_cast<RunCall*>(argument);
    call.delay_us = call.native->health.run().delay_us;
}

// System thread routine (PASSIVE_LEVEL): wait for the DPC or the next due
// check, run one action pass under the device serialization, repeat.
VOID NTAPI health_worker(PVOID context) {
    auto& native = native_of(context);
    uint64_t delay_us = 0;

    for (;;) {
        LARGE_INTEGER timeout{};
        timeout.QuadPart = -static_cast<LONGLONG>(delay_us) * 10;
        KeWaitForSingleObject(&native.wake, Executive, KernelMode, FALSE, delay_us != 0 ? &timeout : nullptr);

        if (InterlockedCompareExchange(&native.stopping, 0, 0) != 0) {
            break;
        }

        RunCall call{&native, 0};
        delay_us = native.hooks.run_locked(native.hooks.context, run_body, &call) ? call.delay_us :
                                                                                      blocked_retry_us;
    }

    PsTerminateSystemThread(STATUS_SUCCESS);
}

} // namespace

bool gpu_health_start(GpuHealthNative& native, const hardware::GpuHealthConfig& config,
                      const GpuHealthHooks& hooks) noexcept {
    if (KeGetCurrentIrql() != PASSIVE_LEVEL || hooks.run_locked == nullptr || hooks.refuse == nullptr ||
        hooks.escalate == nullptr || hooks.quarantine_vram == nullptr) {
        return false;
    }

    if (native.initialized) {
        const bool rebound = native.health.rebind(config) == hardware::HealthStatus::ready && native.started;

        if (rebound) {
            KeSetEvent(&native.wake, IO_NO_INCREMENT, FALSE);
        }

        return rebound;
    }

    KeInitializeSpinLock(&native.lock);
    KeInitializeEvent(&native.wake, SynchronizationEvent, FALSE);
    native.hooks = hooks;
    native.stopping = 0;
    const hardware::GpuHealthPlatform platform{&native, now_us, lock, unlock, refuse, escalate,
                                               quarantine_vram, log};

    if (native.health.initialize(config, platform) != hardware::HealthStatus::ready) {
        return false;
    }

    native.initialized = true;
    OBJECT_ATTRIBUTES attributes{};
    InitializeObjectAttributes(&attributes, nullptr, OBJ_KERNEL_HANDLE, nullptr, nullptr);
    HANDLE thread = nullptr;

    if (!NT_SUCCESS(PsCreateSystemThread(&thread, SYNCHRONIZE, &attributes, nullptr, nullptr, health_worker,
                                         &native))) {
        return false;
    }

    PVOID object = nullptr;
    const NTSTATUS referenced = ObReferenceObjectByHandle(thread, SYNCHRONIZE, nullptr, KernelMode, &object,
                                                          nullptr);
    ZwClose(thread);

    if (!NT_SUCCESS(referenced)) {
        // Not joinable: make it exit at once (storage stays resident).
        InterlockedExchange(&native.stopping, 1);
        KeSetEvent(&native.wake, IO_NO_INCREMENT, FALSE);
        return false;
    }

    native.thread = object;
    native.started = true;
    // Events consumed before the worker existed are acted on now.
    KeSetEvent(&native.wake, IO_NO_INCREMENT, FALSE);
    return true;
}

void gpu_health_consume(GpuHealthNative& native, const hardware::IrqDispatchView& view) noexcept {
    if (native.initialized && native.health.consume(view) && native.started) {
        KeSetEvent(&native.wake, IO_NO_INCREMENT, FALSE);
    }
}

void gpu_health_stop(GpuHealthNative& native) noexcept {
    if (!native.started || KeGetCurrentIrql() != PASSIVE_LEVEL) {
        return;
    }

    InterlockedExchange(&native.stopping, 1);
    KeSetEvent(&native.wake, IO_NO_INCREMENT, FALSE);
    KeWaitForSingleObject(native.thread, Executive, KernelMode, FALSE, nullptr);
    ObDereferenceObject(native.thread);
    native.thread = nullptr;
    native.started = false;
}

} // namespace rx7900::reactos

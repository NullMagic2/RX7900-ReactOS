/*
Module: rx7900.reactos.fence_native
Purpose: NT implementations of the FenceManager platform services.
Created: 2026-10-22
Architecture: KSPIN_LOCK, KEVENT, KTIMER/KDPC, KeQueryInterruptTime and
InterlockedCompareExchange64 behind the portable FencePlatform/RingPointerStore
callbacks. No fence or hardware policy here.
*/
#include "fence_native.hpp"

namespace rx7900::reactos {
namespace {

using rx7900::hardware::FenceHangReason;
using rx7900::hardware::IrqVmFault;
using rx7900::hardware::fence_fallback_timeout_us;
using rx7900::hardware::fence_waiter_slots;

// KeQueryInterruptTime counts 100 ns units.
constexpr ULONGLONG interrupt_time_per_microsecond = 10;
// KeSetTimerEx period in milliseconds.
constexpr LONG poll_period_ms = static_cast<LONG>(fence_fallback_timeout_us / 1000u);

FenceNative& native_of(void* context) noexcept {
    return *static_cast<FenceNative*>(context);
}

// <= DISPATCH_LEVEL. The lock is never taken recursively (FenceManager contract),
// so one saved IRQL suffices.
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

uint64_t now_us(void*) noexcept {
    return KeQueryInterruptTime() / interrupt_time_per_microsecond;
}

// Under the FenceManager lock (DISPATCH_LEVEL): KeSetEvent with Wait FALSE.
void signal(void* context, uint32_t slot) noexcept {
    if (slot < fence_waiter_slots) {
        KeSetEvent(&native_of(context).events[slot], IO_NO_INCREMENT, FALSE);
    }
}

void reset(void* context, uint32_t slot) noexcept {
    if (slot < fence_waiter_slots) {
        KeClearEvent(&native_of(context).events[slot]);
    }
}

// PASSIVE_LEVEL without the lock. A negative interval is relative time.
bool block(void* context, uint32_t slot, uint64_t timeout_us) noexcept {
    if (slot >= fence_waiter_slots || KeGetCurrentIrql() != PASSIVE_LEVEL) {
        return false;
    }

    // FenceManager blocks in slices of at most fence_fallback_timeout_us.
    const uint64_t bounded = timeout_us < fence_fallback_timeout_us ? timeout_us :
                                                                       fence_fallback_timeout_us;
    LARGE_INTEGER interval{};
    interval.QuadPart = -static_cast<LONGLONG>(bounded * interrupt_time_per_microsecond);
    return KeWaitForSingleObject(&native_of(context).events[slot], Executive, KernelMode, FALSE,
                                 &interval) == STATUS_SUCCESS;
}

// gmc_v11_0_process_interrupt dev_err lines, already rate limited by the caller.
void report_vm_fault(void*, const IrqVmFault& fault, uint32_t suppressed) noexcept {
    if (suppressed != 0) {
        DbgPrint("RX7900: gmc_v11_0_process_interrupt: %u callbacks suppressed\n", suppressed);
    }

    DbgPrint("RX7900: [%s] page fault (src_id:%u ring:%u vmid:%u pasid:%u) address "
             "0x%08x%08x client %u status 0x%08x read %u cleared %u\n",
             fault.cache_hub == rx7900::hardware::VmHub::mmhub ? "mmhub" : "gfxhub",
             fault.source, fault.ring_id, fault.vmid, fault.pasid,
             static_cast<unsigned>(fault.address >> 32), static_cast<unsigned>(fault.address),
             fault.client, fault.status, fault.status_read ? 1u : 0u,
             fault.status_cleared ? 1u : 0u);
}

// amdgpu_job_timedout's "ring %s timeout, signaled seq=%u, emitted seq=%u".
void report_hang(void*, uint32_t ring, FenceHangReason reason, uint32_t signalled,
                 uint32_t emitted) noexcept {
    DbgPrint("RX7900: ring %u quarantined (reason %u), signaled seq=%u, emitted seq=%u; "
             "explicit mode-1 reset required\n",
             ring, static_cast<unsigned>(reason), signalled, emitted);
}

// KDPC routine at DISPATCH_LEVEL; DeferredContext is the FenceNative.
VOID NTAPI poll_dpc(PKDPC, PVOID context, PVOID, PVOID) {
    auto* native = static_cast<FenceNative*>(context);

    if (native != nullptr && native->fences != nullptr) {
        native->fences->poll();
    }
}

bool store64(void*, uint8_t* address, uint64_t value) noexcept {
    // The writeback slot is caller-validated in bounds; atomicity needs 8-byte
    // alignment, which a misaligned CPU mapping cannot give.
    if (address == nullptr || (reinterpret_cast<ULONG_PTR>(address) & 7u) != 0) {
        return false;
    }

    auto* target = reinterpret_cast<volatile LONG64*>(address);
    const auto desired = static_cast<LONG64>(value);
    // A compare-exchange with a wrong comperand returns the current value
    // atomically; the loop ends when the full 8-byte store succeeded.
    LONG64 expected = InterlockedCompareExchange64(target, 0, 0);

    for (;;) {
        const LONG64 previous = InterlockedCompareExchange64(target, desired, expected);

        if (previous == expected) {
            return true;
        }

        expected = previous;
    }
}

} // namespace

bool fence_native_initialize(FenceNative& native,
                             rx7900::hardware::FenceManager& fences) noexcept {
    if (native.initialized || KeGetCurrentIrql() != PASSIVE_LEVEL) {
        return false;
    }

    KeInitializeSpinLock(&native.lock);

    for (auto& event : native.events) {
        KeInitializeEvent(&event, SynchronizationEvent, FALSE);
    }

    KeInitializeTimerEx(&native.timer, NotificationTimer);
    KeInitializeDpc(&native.timer_dpc, poll_dpc, &native);
    native.fences = &fences;
    native.timer_started = false;
    const rx7900::hardware::FencePlatform platform{&native, lock, unlock, now_us, signal, reset,
                                                   block, report_vm_fault, report_hang};

    if (fences.initialize(platform) != rx7900::hardware::FenceStatus::ready) {
        native.fences = nullptr;
        return false;
    }

    native.initialized = true;
    return true;
}

void fence_native_start_poll(FenceNative& native) noexcept {
    if (!native.initialized || native.timer_started || KeGetCurrentIrql() != PASSIVE_LEVEL) {
        return;
    }

    LARGE_INTEGER due{};
    due.QuadPart = -static_cast<LONGLONG>(fence_fallback_timeout_us * interrupt_time_per_microsecond);
    KeSetTimerEx(&native.timer, due, poll_period_ms, &native.timer_dpc);
    native.timer_started = true;
}

void fence_native_stop_poll(FenceNative& native) noexcept {
    if (!native.timer_started || KeGetCurrentIrql() != PASSIVE_LEVEL) {
        return;
    }

    KeCancelTimer(&native.timer);
    // A DPC already queued or running finishes before storage may be reused.
    KeFlushQueuedDpcs();
    native.timer_started = false;
}

rx7900::hardware::RingPointerStore fence_native_wptr_store() noexcept {
    return {nullptr, store64};
}

} // namespace rx7900::reactos

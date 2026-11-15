/*
Module: rx7900.reactos.fence_native
Purpose: Supply the NT lock, clock, waiter events, fallback poll timer, log sinks
and atomic WPTR store that the portable FenceManager needs.
Created: 2026-10-22
Architecture: Thin native adapter used by the physical rx7900video path; fence
semantics stay in src/hardware/fence. Uses only ntoskrnl Ke/Interlocked APIs.
*/
#pragma once

#include "video_sdk.hpp"

#include "fence.hpp"

namespace rx7900::reactos {

// Device-owned, nonpaged, constructed once (zeroed VideoPort extension storage
// is a valid unstarted state). Every member is owned here; the FenceManager
// pointer is borrowed and outlives this object.
//
// Lifetime: once the poll timer is started, or the FenceManager is reachable
// from the interrupt DPC, this storage must stay resident until
// fence_native_stop_poll returned and no DPC can run. The physical path never
// unloads after the IH service is armed (see video.cpp release_device), so the
// timer then runs for the device lifetime.
struct FenceNative final {
    KSPIN_LOCK lock;
    // Saved by the single lock holder; written only while the lock is held.
    KIRQL held_irql;
    // One auto-reset (SynchronizationEvent) event per FenceManager waiter slot.
    KEVENT events[rx7900::hardware::fence_waiter_slots];
    KTIMER timer;
    KDPC timer_dpc;
    rx7900::hardware::FenceManager* fences;
    bool initialized;
    bool timer_started;
};

// PASSIVE_LEVEL, once, before the IrqService drain consumer can run: initializes
// the spin lock and events, then fences.initialize with this platform.
bool fence_native_initialize(FenceNative& native,
                             rx7900::hardware::FenceManager& fences) noexcept;

// PASSIVE_LEVEL: a periodic KTIMER every fence_fallback_timeout_us whose DPC
// (DISPATCH_LEVEL) runs FenceManager::poll, i.e. the Linux fence fallback timer
// and the drm_sched job timeout. Idempotent.
void fence_native_start_poll(FenceNative& native) noexcept;
// PASSIVE_LEVEL: cancels the timer and flushes queued DPCs (KeFlushQueuedDpcs).
void fence_native_stop_poll(FenceNative& native) noexcept;

// RingPointerStore over InterlockedCompareExchange64 (cmpxchg8b on i386): one
// 8-byte access of an 8-byte aligned writeback slot; refuses misaligned
// addresses. Any IRQL; context unused.
rx7900::hardware::RingPointerStore fence_native_wptr_store() noexcept;

} // namespace rx7900::reactos

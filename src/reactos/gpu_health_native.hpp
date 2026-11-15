/*
Module: rx7900.reactos.gpu_health_native
Purpose: Own the thermal and RAS interrupt actions on the physical miniport: the
IH DPC handoff and a PASSIVE_LEVEL worker thread that runs GpuHealth under the
device serialization.
Created: 2026-10-24
Architecture: Thin native adapter over src/hardware/gpu_health (policy stays
there): KSPIN_LOCK for the DPC mailbox, a KEVENT the DPC sets, one system
thread with a bounded timed wait, and VideoPortDebugPrint for every event. The
device owner (video.cpp) supplies its serialization and its refusal, recovery
and session-quarantine paths through GpuHealthHooks. Uses only ntoskrnl/
VideoPort APIs.
*/
#pragma once

#include "video_sdk.hpp"

#include "gpu_health.hpp"

namespace rx7900::reactos {

// The device owner's side. run_locked runs body(argument) at PASSIVE_LEVEL under
// its device serialization (video.cpp: device_lock, then the process guard) and
// returns false without running it while the device cannot act (D3, no running
// ASIC, teardown). refuse, escalate and quarantine_vram are called only from
// inside run_locked: refuse must stop admission of new GPU work for good;
// escalate requests the destructive mode-1 recovery (false: no recovery path);
// quarantine_vram quarantines every process session owning VRAM at that VRAM
// byte offset range, returns how many were newly quarantined and sets owners to
// every live session owning it (newly or already quarantined).
struct GpuHealthHooks final {
    void* context = nullptr;
    bool (*run_locked)(void* context, void (*body)(void*) noexcept, void* argument) noexcept = nullptr;
    void (*refuse)(void* context, hardware::HealthReason reason) noexcept = nullptr;
    bool (*escalate)(void* context, hardware::HealthReason reason) noexcept = nullptr;
    uint32_t (*quarantine_vram)(void* context, uint64_t vram_offset, uint64_t bytes, uint32_t& owners) noexcept =
        nullptr;
};

// Device-owned, nonpaged, constructed once (zeroed storage is a valid unstarted
// state). Lifetime: once started, the DPC may call gpu_health_consume and the
// worker runs until gpu_health_stop joined it; the physical path never unloads
// after the IH service is armed, so storage stays resident.
struct GpuHealthNative final {
    hardware::GpuHealth health;
    GpuHealthHooks hooks;
    KSPIN_LOCK lock;
    // Saved by the single mailbox-lock holder.
    KIRQL held_irql;
    KEVENT wake;
    // Referenced thread object, joined and dereferenced by gpu_health_stop.
    PVOID thread;
    volatile LONG stopping;
    bool initialized;
    bool started;
};

// PASSIVE_LEVEL after a completed bring-up with interrupt sources enabled: the
// first call initializes the actions over config and starts the worker; later
// calls (resume, restart after a confirmed reset) rebind config and wake the
// worker (UMC error counters are set up again in its next pass). true when the
// actions run.
bool gpu_health_start(GpuHealthNative& native, const hardware::GpuHealthConfig& config,
                      const GpuHealthHooks& hooks) noexcept;
// The IH DPC drain consumer half (DISPATCH_LEVEL or below): counter deltas and
// the ERREVENT_ATHUB status, then the worker is woken (KeSetEvent, no wait).
void gpu_health_consume(GpuHealthNative& native, const hardware::IrqDispatchView& view) noexcept;
// Teardown, PASSIVE_LEVEL, without the device serialization held: stop and
// join the worker. The DPC keeps feeding the (idle) mailbox afterwards.
void gpu_health_stop(GpuHealthNative& native) noexcept;

} // namespace rx7900::reactos

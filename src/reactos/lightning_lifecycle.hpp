/*
Module: lightning.reactos.lifecycle
Purpose: One PnP/power state machine and the ordered owner sequences of the NT5 filter.
Created: 2026-10-25
Architecture: Allocation-free policy shared by lightning_driver.cpp and host tests.
    The LightningNT5 upper filter owns three dependent resources below its
    lifecycle mutex: the native transport (BAR0, DMA common buffers, host
    context), the renderer (per-file host sessions and borrowed views) and the
    recognized display miniport below it (scanout, staging RAM). Every PnP and
    power transition runs through DeviceLifecycle and one of the run_*
    sequences below, so each transition orders those owners the same way:
      bring-up   (START, D0, refused power-down):
                 lower first, then transport, display, renderer and (START
                 only; it survives D3) the control link;
      power-down (D1-D3, system shutdown):
                 display quiesce, renderer, transport, then the lower device;
      stop:      display quiesce, renderer, transport, control link, then lower;
      remove:    control link (and control device), display quiesce, renderer,
                 transport, then the software display abandon (staging lease
                 transfer) before the lower REMOVE;
      surprise:  control link, display abandon, renderer and transport without
                 register access, then lower.
    Containment is a sticky latch separate from the phase: no STOP, SURPRISE,
    REMOVE, START, cancel or power rollback clears it. A contained device never
    receives register access again, never restarts, and REMOVE retains its FDO
    and owned resources (DMA, adapter, borrowed views) until reboot.
    lightning-pnp-veto: the one avoidable route into containment, an orderly
    STOP/REMOVE (or device D1-D3) while a renderer client still maps borrowed
    CPU views that only it can end, is refused at its QUERY instead: the query
    fails before the lower stack sees it, the phase is unchanged and the device
    keeps serving. Surprise removal and an unqueried SET cannot be refused and
    still contain.
*/
#pragma once
#include <stdint.h>

namespace lightning::reactos {

// Lifecycle position of one filter FDO. stop_pending and remove_pending keep
// the phase they interrupted in previous(); settled() reports that phase.
enum class DevicePhase : uint8_t {
    added, started, suspended, stopped, stop_pending, remove_pending, surprise, removed
};

// Identifies the lifecycle step whose failure contained the device; printed
// with the containment line so a VM log names the failed transition.
enum class LifecycleStep : uint32_t {
    start_unwind = 1, start_transport, power_down, power_up, stop, surprise, remove_owners, remove_abandon
};

// NT power-state numbering (potypes.h); lightning_driver.cpp asserts equality.
namespace power_state {
inline constexpr uint32_t system_working = 1;
inline constexpr uint32_t system_hibernate = 5;
inline constexpr uint32_t system_shutdown = 6;
inline constexpr uint32_t device_d0 = 1;
inline constexpr uint32_t device_d3 = 4;
} // namespace power_state

// What the filter does with one power IRP.
//   owned: queued to the precreated PASSIVE power thread (lifecycle sequence);
//   forward: passed down unchanged; invalid: completed STATUS_INVALID_PARAMETER;
//   veto: a QUERY completed STATUS_NOT_SUPPORTED.
enum class PowerRoute : uint8_t { owned, forward, invalid, veto };

// System sleep/hibernate needs a device power policy owner; ReactOS videoprt
// requests no device IRPs, so S1-S4 queries are vetoed and an unqueried S1-S4
// SET is forwarded (device IRPs from a policy owner, if any, still drive the
// device). Shutdown is final: it is admitted and its SET quiesces the device
// through the power thread before the lower stack sees it, because no device
// IRP ever follows on ReactOS. Out-of-range device states are malformed.
constexpr PowerRoute power_route(bool set, bool system, uint32_t state) noexcept {
    if (system) {
        if (state == power_state::system_working || state == power_state::system_shutdown) {
            return set && state == power_state::system_shutdown ? PowerRoute::owned : PowerRoute::forward;
        }

        return set ? PowerRoute::forward : PowerRoute::veto;
    }

    if (!set) {
        return PowerRoute::forward;
    }

    return state >= power_state::device_d0 && state <= power_state::device_d3 ? PowerRoute::owned :
        PowerRoute::invalid;
}

// lightning-pnp-veto: a device QUERY_POWER for D1-D3 while renderer sessions
// still hold borrowed views. The SET would contain (the renderer cannot end a
// CPU borrow), so the query is refused with STATUS_DEVICE_BUSY. System
// queries are never refused for a borrow: shutdown stays admitted (its SET
// contains if a borrow survives process termination) and S1-S4 are already
// vetoed by power_route.
constexpr bool borrow_vetoes_power_query(bool system, uint32_t state, uint32_t borrowed_sessions) noexcept {
    return !system && borrowed_sessions != 0 && state > power_state::device_d0 && state <= power_state::device_d3;
}

// What the filter does with QUERY_STOP / QUERY_REMOVE (DeviceLifecycle::admit_query).
//   admitted: the phase becomes the pending query; the query is forwarded.
//   refused: contained (reboot-only), another query pending, or the device
//     already left; STATUS_DEVICE_BUSY, nothing changes.
//   borrowed: renderer sessions still map borrowed CPU views (ring and blob
//     mappings) that only their process can end, so the STOP/REMOVE would
//     contain. STATUS_DEVICE_BUSY before the lower stack sees the query; the
//     phase is unchanged, so new work stays admitted and the CANCEL the PnP
//     manager sends next only confirms the started phase.
enum class QueryAdmission : uint8_t { admitted, refused, borrowed };

// What REMOVE does with the FDO after the lower REMOVE and the detach decision.
//   delete_device: every owned resource was released; IoDeleteDevice.
//   retirement_channel: the abandoned display may still hold staging leases,
//     or a foreign open (win32k's captured top-of-stack pointer) can still
//     call this filter; the FDO stays attached until the RetirementGate tears
//     it down (never when contained).
//   retain_contained: reboot-only containment keeps the FDO and its retained
//     DMA/adapter/view ownership; DriverUnload stays cleared.
enum class RemoveDisposition : uint8_t { delete_device, retirement_channel, retain_contained };

// Not thread-safe: the filter calls every method under its lifecycle mutex.
class DeviceLifecycle final {
public:
    DevicePhase phase() const noexcept { return phase_; }
    DevicePhase previous() const noexcept { return previous_; }
    bool contained() const noexcept { return contained_; }
    bool pending() const noexcept {
        return phase_ == DevicePhase::stop_pending || phase_ == DevicePhase::remove_pending;
    }

    DevicePhase settled() const noexcept { return pending() ? previous_ : phase_; }

    // Sticky. True only for the first call, so per-image accounting runs once.
    bool contain() noexcept {
        const bool first = !contained_;
        contained_ = true;
        return first;
    }

    // New control-device and foreign display work (open, hardware IOCTLs).
    // A pending query already closes it, as it did before the state machine.
    bool accepts_work() const noexcept { return !contained_ && phase_ == DevicePhase::started; }

    // Teardown steps may touch device registers (MMIO CLOSE/RESET, quiesce
    // publication) only in an owned D0 that was never contained or removed.
    bool hardware_reachable() const noexcept {
        const DevicePhase current = settled();
        return !contained_ && current != DevicePhase::suspended && current != DevicePhase::surprise &&
            current != DevicePhase::removed;
    }

    // START: first start or restart after STOP. A contained device stays
    // contained; a duplicate START never tears a running device down.
    bool admit_start() const noexcept {
        return !contained_ && (phase_ == DevicePhase::added || phase_ == DevicePhase::stopped);
    }

    void started() noexcept { phase_ = DevicePhase::started; }
    void stopped() noexcept { phase_ = DevicePhase::stopped; }
    void surprise_removed() noexcept { phase_ = DevicePhase::surprise; }
    void removed() noexcept { phase_ = DevicePhase::removed; }

    // QUERY_STOP / QUERY_REMOVE. Refused while contained (reboot-only), while
    // another query is pending, or after the device left; vetoed (phase
    // unchanged) while borrowed_sessions renderer sessions hold views that
    // only their owner can end. The caller samples the census under the same
    // lifecycle mutex: once admitted, accepts_work() is false, so no new view
    // can be mapped before the STOP/REMOVE or the CANCEL.
    QueryAdmission admit_query(bool stop, uint32_t borrowed_sessions) noexcept {
        if (contained_ || pending() || phase_ == DevicePhase::surprise || phase_ == DevicePhase::removed) {
            return QueryAdmission::refused;
        }

        if (borrowed_sessions != 0) {
            return QueryAdmission::borrowed;
        }

        previous_ = phase_;
        phase_ = stop ? DevicePhase::stop_pending : DevicePhase::remove_pending;
        return QueryAdmission::admitted;
    }

    // A query with no borrow census (no renderer, host fixtures).
    bool admit_query(bool stop) noexcept { return admit_query(stop, 0) == QueryAdmission::admitted; }

    // The lower stack vetoed our accepted query, or CANCEL arrived: only the
    // matching pending phase returns to the phase it interrupted.
    void end_query(bool stop) noexcept {
        if (phase_ == (stop ? DevicePhase::stop_pending : DevicePhase::remove_pending)) {
            phase_ = previous_;
        }
    }

    // Device power-down (D1-D3) or the final system shutdown.
    bool power_down_due() const noexcept { return !contained_ && settled() == DevicePhase::started; }
    void powered_down() noexcept { settle(DevicePhase::suspended); }
    // D0 completed below, or the lower stack refused a prepared power-down.
    // Nothing resumes once contained or after STOP/SURPRISE/REMOVE moved on.
    bool power_up_due() const noexcept { return !contained_ && settled() == DevicePhase::suspended; }
    void powered_up() noexcept { settle(DevicePhase::started); }

    // channel_needed: the lower device is still referenced and either its
    // display was abandoned (leases) or a foreign open is outstanding.
    RemoveDisposition remove_disposition(bool channel_needed) const noexcept {
        if (channel_needed) {
            return RemoveDisposition::retirement_channel;
        }

        return contained_ ? RemoveDisposition::retain_contained : RemoveDisposition::delete_device;
    }

    // The retirement channel may arm its self-deleting teardown only while the
    // device is not contained; otherwise it stays until reboot.
    bool retirement_teardown_allowed() const noexcept { return !contained_; }

private:
    void settle(DevicePhase phase) noexcept {
        if (pending()) {
            previous_ = phase;
        } else {
            phase_ = phase;
        }
    }

    DevicePhase phase_ = DevicePhase::added;
    DevicePhase previous_ = DevicePhase::added;
    bool contained_ = false;
};

// Owners concept used by the run_* sequences, implemented by the filter over
// its device state and by host fixtures. Every call runs at PASSIVE_LEVEL under
// the lifecycle mutex; bool results are success. hardware=false forbids
// register access (surprise removal, D3, containment).
//   bool transport_start();    bool transport_resume();   bool transport_suspend();
//   bool transport_stop(bool hardware);  bool transport_ready() const;
//   bool transport_contained() const;
//   bool display_discover();   bool display_resume();     bool display_quiesce();
//   bool display_abandon();    bool display_recognized() const;
//   bool display_lost() const; void observe_leases();
//   void renderer_start();     bool renderer_stop(bool hardware);
//   bool publish_control();    void withdraw_control();
//   void contain(LifecycleStep step);   (native containment; calls lifecycle.contain())

enum class StartResult : uint8_t { started, refused, failed };

// IRP_MN_START_DEVICE after the lower stack started. A failure after the
// transport started unwinds in reverse; an uncertain unwind contains.
template<typename Owners>
StartResult run_start(DeviceLifecycle& lifecycle, Owners& owners) noexcept {
    if (!lifecycle.admit_start()) {
        return StartResult::refused;
    }

    bool ok = owners.transport_start();
    ok = ok && owners.display_discover();
    ok = ok && owners.display_resume();

    if (ok) {
        owners.renderer_start();
        lifecycle.started();
        ok = owners.publish_control();
    }

    if (!ok && owners.transport_ready()) {
        lifecycle.stopped();

        if (!owners.display_quiesce() || !owners.renderer_stop(true) || !owners.transport_stop(true)) {
            owners.contain(LifecycleStep::start_unwind);
        }
    }

    if (owners.transport_contained()) {
        owners.contain(LifecycleStep::start_transport);
    }

    return ok ? StartResult::started : StartResult::failed;
}

// Before forwarding D1-D3 or the system shutdown SET. A filter must not fail
// a legitimate SET: an uncertain step contains instead. True when the device
// was prepared, so a lower refusal must roll it back (run_power_up).
template<typename Owners>
bool run_power_down(DeviceLifecycle& lifecycle, Owners& owners) noexcept {
    if (!lifecycle.power_down_due()) {
        return false;
    }

    const bool ok = owners.display_quiesce() && owners.renderer_stop(true) && owners.transport_suspend();
    lifecycle.powered_down();

    if (!ok) {
        owners.contain(LifecycleStep::power_down);
    }

    return ok;
}

// After a lower D0, or after the lower stack refused a prepared power-down.
template<typename Owners>
void run_power_up(DeviceLifecycle& lifecycle, Owners& owners) noexcept {
    if (!lifecycle.power_up_due()) {
        return;
    }

    if (owners.transport_resume() && owners.display_resume()) {
        owners.renderer_start();
        lifecycle.powered_up();
    } else {
        owners.contain(LifecycleStep::power_up);
    }
}

// IRP_MN_STOP_DEVICE. True: forward the STOP. A contained or failed stop
// completes it with the failure and keeps the lower resources untouched.
template<typename Owners>
bool run_stop(DeviceLifecycle& lifecycle, Owners& owners) noexcept {
    const bool hardware = lifecycle.hardware_reachable();
    const bool ok = (!hardware || owners.display_quiesce()) && owners.renderer_stop(hardware) &&
        owners.transport_stop(hardware);
    owners.withdraw_control();
    lifecycle.stopped();

    if (!ok) {
        owners.contain(LifecycleStep::stop);
    }

    return ok;
}

// IRP_MN_SURPRISE_REMOVAL: the function may already be gone. No register
// access; the display client transfers its views off leased staging RAM.
template<typename Owners>
void run_surprise(DeviceLifecycle& lifecycle, Owners& owners) noexcept {
    owners.withdraw_control();
    lifecycle.surprise_removed();
    const bool abandoned = owners.display_abandon();
    const bool renderer = owners.renderer_stop(false);
    const bool transport = renderer && owners.transport_stop(false);

    if (!abandoned || !renderer || !transport) {
        owners.contain(LifecycleStep::surprise);
    }
}

// IRP_MN_REMOVE_DEVICE, the owner part before the remove-lock drain and the
// lower REMOVE. The display is abandoned last (software only), so its staging
// leases transfer to the retirement channel whatever happened before.
template<typename Owners>
void run_remove_owners(DeviceLifecycle& lifecycle, Owners& owners) noexcept {
    const bool hardware = lifecycle.hardware_reachable();
    owners.withdraw_control();

    if ((hardware && !owners.display_quiesce()) || !owners.renderer_stop(hardware) ||
        !owners.transport_stop(hardware)) {
        owners.contain(LifecycleStep::remove_owners);
    }

    lifecycle.removed();

    if (owners.display_recognized() && !owners.display_abandon()) {
        owners.contain(LifecycleStep::remove_abandon);
    } else if (owners.display_lost()) {
        owners.observe_leases();
    }
}

} // namespace lightning::reactos

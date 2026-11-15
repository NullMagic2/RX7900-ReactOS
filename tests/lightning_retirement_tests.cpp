/*
Module: lightning.tests.retirement
Purpose: Exercise the removed-filter retirement gate ordering and ref-counting.
Created: 2026-10-24
Architecture: Drives the portable RetirementGate exactly as lightning_driver.cpp
does - open/close of the win32k device reference and resolve-receipt held-lease
observations after REMOVE - and asserts teardown fires once, only when the last
reference and the last lease are both gone, in every interleaving.
*/
#include "lightning_retirement.hpp"
#include "lightning_lifecycle.hpp"
#include <cstdio>
#include <cstdlib>

namespace {
using namespace lightning::reactos;
unsigned checks = 0;

void expect(bool condition, const char* description) {
    ++checks;
    if (!condition) {
        std::fprintf(stderr, "FAIL %s\n", description);
        std::exit(1);
    }
}

// A tiny model of the driver side: how many times retirement_teardown would
// run for a given sequence of signals. Teardown must run exactly once.
struct Harness final {
    RetirementGate gate{};
    unsigned teardowns = 0;

    void open() { gate.note_open(); }
    void close() {
        const bool last = gate.note_close();
        if (last && gate.begin_teardown()) ++teardowns;
    }
    void resolve(uint32_t held) {
        gate.observe_held_leases(held);
        if (gate.begin_teardown()) ++teardowns;
    }
    void remove() { gate.enter_retirement(); }
};

// win32k opens the device, display client holds one lease; on REMOVE the client
// retires the lease (resolve held=0) and only then win32k closes. Teardown on
// the final close.
void lease_then_close() {
    Harness h;
    h.open();
    h.remove();
    expect(!h.gate.ready_to_delete(), "not ready while open and lease unknown");
    h.resolve(1);
    expect(!h.gate.ready_to_delete(), "not ready while a lease is held");
    h.resolve(0);
    expect(h.gate.ready_to_delete() == false || h.gate.foreign_opens() == 0,
           "lease retired but win32k still referencing keeps it alive");
    expect(h.teardowns == 0, "no teardown while win32k still open");
    h.close();
    expect(h.teardowns == 1, "teardown on final close after leases retired");
    expect(h.gate.torn_down(), "gate latched torn down");
}

// Reverse order: win32k closes first (remove-complete notification), then the
// client retires the last lease. Teardown on the resolve.
void close_then_lease() {
    Harness h;
    h.open();
    h.remove();
    h.resolve(2);
    h.close();
    expect(h.teardowns == 0, "no teardown while leases still held");
    expect(h.gate.ready_to_delete() == false, "not ready: leases held");
    h.resolve(1);
    expect(h.teardowns == 0, "still held");
    h.resolve(0);
    expect(h.teardowns == 1, "teardown when last lease retired after close");
}

// No lease was ever held: teardown as soon as win32k releases after REMOVE.
// Models observe_held_leases(0) arriving via the client's abandon confirmation.
void no_leases() {
    Harness h;
    h.open();
    h.remove();
    h.resolve(0);
    expect(h.teardowns == 0, "win32k still open");
    h.close();
    expect(h.teardowns == 1, "teardown once win32k releases, no leases");
}

// Reference counting: several concurrent win32k opens must all close before the
// device may go. A spurious extra close must not trigger a second teardown.
void nested_opens() {
    Harness h;
    h.open();
    h.open();
    h.open();
    h.remove();
    h.resolve(0);
    expect(!h.gate.note_close(), "a close that leaves opens is not the last");
    h.gate.note_open();
    h.close();
    expect(h.teardowns == 0, "two refs still outstanding");
    h.close();
    expect(h.teardowns == 0, "one ref still outstanding");
    h.close();
    expect(h.teardowns == 1, "teardown on last of three closes");
    h.close(); // spurious / double close
    expect(h.teardowns == 1, "no second teardown on extra close");
    expect(h.gate.foreign_opens() == 0, "open count never goes negative");
}

// Nothing happens before REMOVE: opens/closes/leases without retirement must
// never tear down (the device is live).
void never_before_remove() {
    Harness h;
    h.open();
    h.resolve(0);
    h.close();
    expect(h.teardowns == 0, "no teardown while the device is still live");
    expect(!h.gate.ready_to_delete(), "not retiring: not ready");
}

// A lease that re-appears (held goes 0 then non-zero) before win32k closes must
// not leave the gate stuck 'retired'; only the state at decision time counts.
void lease_reacquired() {
    Harness h;
    h.open();
    h.remove();
    h.resolve(0);
    h.resolve(3); // a new MAP took another lease before teardown
    h.close();
    expect(h.teardowns == 0, "re-acquired lease blocks teardown on close");
    h.resolve(0);
    expect(h.teardowns == 1, "teardown once the new lease is also retired");
}

// Close arriving while still live (open count underflow guard) and begin_teardown
// idempotency under repeated ready checks.
void latch_idempotent() {
    Harness h;
    h.remove();
    h.open();
    h.resolve(0);
    h.close();
    expect(h.teardowns == 1, "single teardown");
    expect(!h.gate.begin_teardown(), "begin_teardown refuses after latch");
    h.resolve(0);
    expect(h.teardowns == 1, "no teardown after already torn down");
}

// lightning_driver.cpp REMOVE since display-unload: the filter asks the
// miniport for its held-lease count right after abandon (observe_retained_leases),
// enters retirement only with its teardown work item reserved, checks the gate at
// once, and counts foreign closes that arrive before REMOVE.
struct RemovalFilter final {
    RetirementGate gate{};
    unsigned teardowns = 0;

    void open() { gate.note_open(); }
    void close() {
        if (gate.note_close() && gate.begin_teardown()) ++teardowns;
    }
    void resolve(uint32_t held) {
        gate.observe_held_leases(held);
        if (gate.begin_teardown()) ++teardowns;
    }
    void remove(uint32_t held_after_abandon, bool work_reserved) {
        gate.observe_held_leases(held_after_abandon);
        if (work_reserved) {
            gate.enter_retirement();
            if (gate.begin_teardown()) ++teardowns;
        }
    }
};

// The primary display: win32k open, one lease. The desktop thread first
// releases the lease (DrvDisableSurface), then drops the file object.
void primary_switch_order() {
    RemovalFilter f;
    f.open();
    f.remove(1, true);
    expect(f.teardowns == 0, "REMOVE with an open and a lease keeps the channel");
    f.resolve(0);
    expect(f.teardowns == 0, "lease retired, win32k still open");
    f.close();
    expect(f.teardowns == 1, "win32k's final close tears the channel down once");
}

// No lease left after abandon and win32k already closed: nothing else would
// ever arrive, so REMOVE itself queues the teardown.
void nothing_outstanding_at_remove() {
    RemovalFilter f;
    f.open();
    f.close();
    expect(f.gate.foreign_opens() == 0, "a close before REMOVE ends its counted open");
    f.remove(0, true);
    expect(f.teardowns == 1, "teardown queued from REMOVE itself");
}

// The seed only answers for leases: an open display still waits for its close.
void seeded_zero_still_waits_for_close() {
    RemovalFilter f;
    f.open();
    f.remove(0, true);
    expect(f.teardowns == 0, "no leases but win32k still holds the display");
    f.close();
    expect(f.teardowns == 1, "teardown on the close");
}

// Without a reserved work item (allocation failure or containment) the channel
// keeps serving retirement but never deletes itself.
void no_work_item_never_tears_down() {
    RemovalFilter f;
    f.open();
    f.remove(0, false);
    f.close();
    f.resolve(0);
    expect(f.teardowns == 0, "an unarmed channel stays until reboot");
    expect(!f.gate.retiring(), "it never entered retirement");
}

// lightning-lifecycle (2026-10-25): REMOVE keeps the channel for any
// outstanding foreign open, not only for an abandoned display. A lower device
// without the private ABI cannot hold staging leases, so REMOVE seeds zero.
void foreign_open_without_display() {
    RemovalFilter f;
    f.open();
    f.remove(0, true);
    expect(f.teardowns == 0, "win32k's open keeps a non-display channel alive");
    f.close();
    expect(f.teardowns == 1, "the last close deletes a non-display channel once");
}

// A failed REMOVE-time observation leaves the lease count unknown: the channel
// must not delete itself on the close alone (the client may still need it to
// release RAM), and the next resolve receipt decides.
void unknown_leases_wait_for_a_receipt() {
    RetirementGate gate{};
    unsigned teardowns = 0;
    gate.note_open();
    gate.enter_retirement();
    expect(!gate.begin_teardown(), "unknown leases and an open: no teardown");

    if (gate.note_close() && gate.begin_teardown()) {
        ++teardowns;
    }

    expect(teardowns == 0 && !gate.leases_retired(), "the close alone never deletes with unknown leases");
    gate.observe_held_leases(0);

    if (gate.begin_teardown()) {
        ++teardowns;
    }

    expect(teardowns == 1, "the re-observation retires the channel once");
}

// The lifecycle disposition that feeds this gate.
void disposition_matches_gate() {
    using namespace lightning::reactos;
    DeviceLifecycle lifecycle{};
    expect(lifecycle.remove_disposition(false) == RemoveDisposition::delete_device, "nothing to retire: delete");
    expect(lifecycle.remove_disposition(true) == RemoveDisposition::retirement_channel, "leases or opens: channel");
    expect(lifecycle.retirement_teardown_allowed(), "an uncontained channel may arm its teardown");
    lifecycle.contain();
    expect(lifecycle.remove_disposition(false) == RemoveDisposition::retain_contained, "contained: retained FDO");
    expect(lifecycle.remove_disposition(true) == RemoveDisposition::retirement_channel,
        "a contained display still serves retirement");
    expect(!lifecycle.retirement_teardown_allowed(), "a contained channel never arms its teardown");
}

} // namespace

int main() {
    lease_then_close();
    close_then_lease();
    no_leases();
    nested_opens();
    never_before_remove();
    lease_reacquired();
    latch_idempotent();
    primary_switch_order();
    nothing_outstanding_at_remove();
    seeded_zero_still_waits_for_close();
    no_work_item_never_tears_down();
    foreign_open_without_display();
    unknown_leases_wait_for_a_receipt();
    disposition_matches_gate();
    std::printf("lightning_retirement_tests: %u checks passed\n", checks);
    return 0;
}

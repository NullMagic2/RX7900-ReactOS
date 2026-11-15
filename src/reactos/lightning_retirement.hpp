/*
Module: lightning.reactos.retirement
Purpose: Decide when a removed Lightning filter may finally delete itself.
Created: 2026-10-24
Architecture: Allocation-free policy shared by the NT5 filter and host tests.
    After PnP REMOVE of a recognized display the filter cannot be deleted while
    win32k still references the top-of-stack device object (its captured
    FileObject) or while the display client still holds staging RAM leases.
    This gate tracks both so lightning_driver.cpp can restore DriverUnload and
    delete the detached filter exactly once, when the last reference drops.
    The miniport (lightning_video.cpp) owns the actual RAM; this only records
    the last observed held-lease count reported back through a resolve receipt.
*/
#pragma once
#include <stdint.h>

namespace lightning::reactos {

// Not thread-safe by itself: the NT5 filter calls every method under its
// per-device lifecycle mutex (Guard), which also serializes the dispatch paths
// that update it. State transitions are monotonic toward teardown.
class RetirementGate final {
public:
    // A recognized-display REMOVE detached the filter but left it alive as the
    // staging retirement channel. Only after this is teardown ever considered.
    void enter_retirement() noexcept { retiring_ = true; }

    bool retiring() const noexcept { return retiring_; }

    // win32k (or any foreign caller) opened/closed the top-of-stack device.
    // Opens before retirement are counted too, so a handle still open at REMOVE
    // keeps the filter alive until its matching close arrives.
    void note_open() noexcept {
        if (foreign_opens_ < 0x7fffffff) ++foreign_opens_;
    }

    // Returns true only on the transition to zero, so the caller runs teardown
    // for the final close and never double-deletes on a spurious extra close.
    bool note_close() noexcept {
        if (foreign_opens_ == 0) return false;
        --foreign_opens_;
        return foreign_opens_ == 0;
    }

    int32_t foreign_opens() const noexcept { return foreign_opens_; }

    // The miniport answered a retirement request; held is its post-operation
    // held-lease count. Zero means no RAM lease is outstanding any more.
    void observe_held_leases(uint32_t held) noexcept {
        leases_retired_ = (held == 0);
    }

    bool leases_retired() const noexcept { return leases_retired_; }

    // The sole deletion predicate. True exactly when the filter has been
    // retired, win32k has released its device reference, and no staging lease
    // remains. The caller must also confirm it has not already torn down.
    bool ready_to_delete() const noexcept {
        return retiring_ && foreign_opens_ == 0 && leases_retired_;
    }

    // Latch so teardown runs once even if several signals arrive together.
    bool begin_teardown() noexcept {
        if (torn_down_ || !ready_to_delete()) return false;
        torn_down_ = true;
        return true;
    }

    bool torn_down() const noexcept { return torn_down_; }

private:
    int32_t foreign_opens_ = 0;
    bool retiring_ = false;
    bool leases_retired_ = false;
    bool torn_down_ = false;
};

} // namespace lightning::reactos

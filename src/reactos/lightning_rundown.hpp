/*
Module: lightning.reactos.rundown
Purpose: Admit entry-point requests only while a channel is open and drain them before teardown.
Created: 2026-10-25
Architecture: Allocation-free policy over one caller-supplied 32-bit atomic cell.
    A request that holds no reference on the device object it runs in (a
    foreign IRP that reached the removed LightningNT5 filter, whose
    extension teardown deletes) is admitted here before it reads any channel
    state, and releases as its last access. Teardown closes admission and
    then waits for the count to drain; until it drains the extension stays.
    The kernel cell wraps InterlockedCompareExchange; host tests use
    std::atomic. Unlike the NT remove lock this never blocks a caller and a
    closed channel never reopens.
*/
#pragma once
#include <stdint.h>

namespace lightning::reactos {

// Cell: int32_t load() const noexcept; int32_t compare_exchange(int32_t
// desired, int32_t expected) noexcept returning the previous value (the
// InterlockedCompareExchange contract). Zero-initialized means inactive.
template<typename Cell>
class Rundown final {
public:
    enum class Phase { inactive, open, closed };

    // Start admitting. Idempotent; false once closed (never reopens).
    bool open() noexcept {
        for (;;) {
            const int32_t observed = cell_.load();

            if ((observed & closed_flag) != 0) {
                return false;
            }

            if ((observed & open_flag) != 0 ||
                cell_.compare_exchange(observed | open_flag, observed) == observed) {
                return true;
            }
        }
    }

    // True: the caller is admitted and must call release exactly once, as its
    // final access to the storage that holds this object. False: inactive,
    // closed or saturated; the caller must not touch the guarded channel.
    bool acquire() noexcept {
        for (;;) {
            const int32_t observed = cell_.load();

            if ((observed & (open_flag | closed_flag)) != open_flag || (observed & count_mask) == count_mask) {
                return false;
            }

            if (cell_.compare_exchange(observed + 1, observed) == observed) {
                return true;
            }
        }
    }

    void release() noexcept {
        for (;;) {
            const int32_t observed = cell_.load();

            if ((observed & count_mask) == 0) {
                // Unbalanced release: never wrap into a false drain.
                return;
            }

            if (cell_.compare_exchange(observed - 1, observed) == observed) {
                return;
            }
        }
    }

    // Refuse every later acquire. Requests already admitted keep running and
    // still release. True only for the call that closed it.
    bool close() noexcept {
        for (;;) {
            const int32_t observed = cell_.load();

            if ((observed & closed_flag) != 0) {
                return false;
            }

            const int32_t closed = (observed & ~open_flag) | closed_flag;

            if (cell_.compare_exchange(closed, observed) == observed) {
                return true;
            }
        }
    }

    Phase phase() const noexcept {
        const int32_t observed = cell_.load();
        return (observed & closed_flag) != 0 ? Phase::closed :
            ((observed & open_flag) != 0 ? Phase::open : Phase::inactive);
    }

    uint32_t active() const noexcept { return static_cast<uint32_t>(cell_.load() & count_mask); }

    // Closed and nothing admitted remains: the guarded storage may retire.
    bool drained() const noexcept {
        const int32_t observed = cell_.load();
        return (observed & closed_flag) != 0 && (observed & count_mask) == 0;
    }

private:
    static constexpr int32_t open_flag = 0x40000000;
    static constexpr int32_t closed_flag = 0x20000000;
    static constexpr int32_t count_mask = 0x1fffffff;
    Cell cell_{};
};

} // namespace lightning::reactos

/*
Module: lightning.reactos.completion
Purpose: Retire bounded resource leases only after observed per-ring GPU fences.
Created: 2026-10-21
Architecture: Allocation-free ledger; queue acknowledgement alone never retires work.
*/
#pragma once

#include "lightning_abi.hpp"

namespace lightning::reactos {

class CompletionLedger final {
public:
    static constexpr size_t capacity = 256;

    // Reserve values without mutation before either hardware submission. The
    // caller holds the native session guard across prepare, submission and commit.
    bool prepare(size_t ring, uint64_t& serial, uint64_t& point) const noexcept {
        if (ring >= QLV_MAX_TIMELINES || count_ == capacity || next_ == UINT64_MAX ||
            issued_[ring] == UINT64_MAX) {
            return false;
        }

        serial = next_;
        point = issued_[ring] + 1;
        return true;
    }

    // Called only after both user work and its genuine sync-only fence were
    // accepted by the host. Failure after hardware mutation requires real reset.
    bool commit(size_t ring, uint64_t serial, uint64_t point) noexcept {
        uint64_t expected_serial = 0;
        uint64_t expected_point = 0;

        if (!prepare(ring, expected_serial, expected_point) || serial != expected_serial || point != expected_point) {
            return false;
        }

        batches_[(head_ + count_) % capacity] = {serial, point, ring};
        issued_[ring] = point;
        ++next_;
        ++count_;
        return true;
    }

    // The native caller supplies an actual validated READ_SYNC value. Independent
    // rings may finish out of order; retire only the globally contiguous prefix.
    bool observe(size_t ring, uint64_t point) noexcept {
        if (ring >= QLV_MAX_TIMELINES || point < completed_[ring] || point > issued_[ring]) {
            return false;
        }

        completed_[ring] = point;

        while (count_ != 0) {
            const auto& first = batches_[head_];

            if (completed_[first.ring] < first.point) {
                break;
            }

            retired_ = first.serial;
            head_ = (head_ + 1) % capacity;
            --count_;
        }

        return true;
    }

    bool pending(uint64_t serial) const noexcept { return serial > retired_; }
    uint64_t latest() const noexcept { return next_ - 1; }
    uint64_t retired() const noexcept { return retired_; }
    size_t count() const noexcept { return count_; }
    uint64_t issued(size_t ring) const noexcept { return ring < QLV_MAX_TIMELINES ? issued_[ring] : 0; }
    uint64_t completed(size_t ring) const noexcept { return ring < QLV_MAX_TIMELINES ? completed_[ring] : 0; }

private:
    struct Batch final { uint64_t serial; uint64_t point; size_t ring; };
    Batch batches_[capacity]{};
    uint64_t issued_[QLV_MAX_TIMELINES]{};
    uint64_t completed_[QLV_MAX_TIMELINES]{};
    uint64_t next_ = 1;
    uint64_t retired_ = 0;
    size_t head_ = 0;
    size_t count_ = 0;
};

} // namespace lightning::reactos

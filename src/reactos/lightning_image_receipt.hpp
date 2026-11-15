/*
Module: lightning.reactos.image_receipt
Purpose: Decide producer sync points and asynchronous image-read receipts without OS calls.
Created: 2026-10-24
Architecture: Pure fixed-width policy shared by the native renderer, the
LightningNT5 upper filter and host tests. The renderer owns the recorded points
and receipts; the filter supplies observed GDI canvas progress; nothing here
waits, allocates or touches a device.
*/
#pragma once
#include <stddef.h>
#include <stdint.h>

namespace lightning::reactos {

// The producer named by a version-2 present: an opaque renderer sync token of
// the presenting session and the value its producing submission signals.
struct ProducerPoint final {
    uint64_t sync = 0;
    uint64_t value = 0;
};

// Recorded when a SUBMIT names a caller sync as a signal point: the private
// ledger serial of that submission and its position on the private per-ring
// timeline. A later SUBMIT signaling the same sync replaces the record.
struct SignalPoint final {
    uint64_t serial = 0;
    uint64_t point = 0;
    uint64_t value = 0;
    uint32_t ring = 0;

    constexpr bool recorded() const noexcept { return serial != 0 && point != 0; }
};

// A present may wait for the named producer alone only when it is a recorded
// signal of this session with the same value, written no later than the image's
// own conservative stamp. Anything else keeps the full drain to image_serial.
constexpr bool producer_usable(const ProducerPoint& producer, const SignalPoint& signal,
                               uint64_t image_serial) noexcept {
    return producer.sync != 0 && signal.recorded() && signal.value == producer.value &&
        signal.serial <= image_serial;
}

// Outstanding asynchronous external copy of one image: the GDI canvas epoch
// and the accepted command sequence that reads it. Zero sequence means none.
struct ImageReceipt final {
    uint32_t mode_generation = 0;
    uint32_t epoch = 0;
    uint32_t sequence = 0;

    constexpr bool pending() const noexcept { return sequence != 0; }
};

// Observed GDI canvas progress from a native coherence query or wait receipt.
struct CanvasProgress final {
    uint32_t epoch = 0;
    uint32_t accepted_sequence = 0;
    uint32_t completed_sequence = 0;
    bool uncertain = false;
};

enum class ReceiptState { retired, pending, uncertain };

// The host executes one canvas's commands in order and completes them as an
// ordered prefix. A canvas epoch changes only after a BIND that first waited
// for every earlier command, or after a reset whose backend release was
// confirmed; both prove the older epoch's reads retired. An uncertain canvas
// can still prove an already observed completion, never a later one.
constexpr ReceiptState decide_receipt(const ImageReceipt& receipt, const CanvasProgress& progress) noexcept {
    if (!receipt.pending() || progress.epoch != receipt.epoch ||
        progress.completed_sequence >= receipt.sequence) {
        return ReceiptState::retired;
    }

    if (progress.uncertain || progress.accepted_sequence < receipt.sequence ||
        progress.completed_sequence > progress.accepted_sequence) {
        return ReceiptState::uncertain;
    }

    return ReceiptState::pending;
}

// A newer accepted copy of the same image supersedes an older one: in-order
// completion of the newer read implies the older one retired. A copy from an
// older epoch than the outstanding one is a protocol inconsistency.
constexpr bool supersede_receipt(const ImageReceipt& current, const ImageReceipt& accepted,
                                 ImageReceipt& output) noexcept {
    if (!accepted.pending()) {
        return false;
    }

    if (current.pending() && (accepted.epoch < current.epoch ||
        (accepted.epoch == current.epoch && accepted.sequence <= current.sequence))) {
        return false;
    }

    output = accepted;
    return true;
}

// Latest signal of recently signaled syncs, bounded per session. A full table
// replaces its oldest record; an evicted or never-recorded sync simply keeps
// the caller's conservative whole-session stamp, so loss is never unsafe.
template <size_t Capacity>
class SignalTable final {
public:
    void record(uint64_t token, const SignalPoint& signal) noexcept {
        if (token == 0 || !signal.recorded()) {
            return;
        }

        Entry* target = &entries_[0];

        for (auto& entry : entries_) {
            if (entry.token == token) {
                target = &entry;
                break;
            }

            if (target->token != 0 && (entry.token == 0 || entry.signal.serial < target->signal.serial)) {
                target = &entry;
            }
        }

        *target = Entry{token, signal};
    }
    const SignalPoint* find(uint64_t token) const noexcept {
        for (const auto& entry : entries_) {
            if (token != 0 && entry.token == token) {
                return &entry.signal;
            }
        }

        return nullptr;
    }
    void erase(uint64_t token) noexcept {
        for (auto& entry : entries_) {
            if (token != 0 && entry.token == token) {
                entry = Entry{};
            }
        }
    }
    void clear() noexcept {
        for (auto& entry : entries_) {
            entry = Entry{};
        }
    }

private:
    struct Entry final {
        uint64_t token = 0;
        SignalPoint signal{};
    };
    Entry entries_[Capacity]{};
};

// Images whose newest accepted copy may still read them, bounded per session.
// A caller that cannot pin keeps the synchronous copy instead.
template <size_t Capacity>
class PinTable final {
public:
    bool can_pin(uint64_t token) const noexcept {
        if (token == 0) {
            return false;
        }

        for (const auto& entry : entries_) {
            if (entry.token == token || entry.token == 0) {
                return true;
            }
        }

        return false;
    }
    bool pin(uint64_t token, const ImageReceipt& receipt) noexcept {
        if (!can_pin(token) || !receipt.pending()) {
            return false;
        }

        Entry* free_entry = nullptr;

        for (auto& entry : entries_) {
            if (entry.token == token) {
                return supersede_receipt(entry.receipt, receipt, entry.receipt);
            }

            if (entry.token == 0 && free_entry == nullptr) {
                free_entry = &entry;
            }
        }

        *free_entry = Entry{token, receipt};
        return true;
    }
    const ImageReceipt* find(uint64_t token) const noexcept {
        for (const auto& entry : entries_) {
            if (token != 0 && entry.token == token) {
                return &entry.receipt;
            }
        }

        return nullptr;
    }
    // Index-based walk for settling; tokens are zero for free entries.
    static constexpr size_t capacity() noexcept { return Capacity; }
    uint64_t token(size_t index) const noexcept { return index < Capacity ? entries_[index].token : 0; }
    void erase(uint64_t token) noexcept {
        for (auto& entry : entries_) {
            if (token != 0 && entry.token == token) {
                entry = Entry{};
            }
        }
    }
    void clear() noexcept {
        for (auto& entry : entries_) {
            entry = Entry{};
        }
    }
    bool empty() const noexcept {
        for (const auto& entry : entries_) {
            if (entry.token != 0) {
                return false;
            }
        }

        return true;
    }

private:
    struct Entry final {
        uint64_t token = 0;
        ImageReceipt receipt{};
    };
    Entry entries_[Capacity]{};
};

} // namespace lightning::reactos

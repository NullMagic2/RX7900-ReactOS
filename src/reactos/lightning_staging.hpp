/*
Module: lightning.reactos.staging
Purpose: Retain cached CPU scanout storage until its last mapped GDI lease ends.
Created: 2026-10-21
Architecture: Allocation-free ownership ledger; native VideoPort allocates bytes.
*/
#pragma once

#include <stdint.h>

namespace lightning::reactos {

// Caller serializes the ledger. Buffers are zeroed by the native owner before
// installation. Retired buffers remain writable by existing GDI surfaces but
// cannot become the active scanout again. No destructor implicitly frees bytes.
class StagingLedger final {
public:
    static constexpr uint32_t capacity = 4;

    bool can_replace() const noexcept {
        for (uint32_t i = 0; i < capacity; ++i) {
            if (slots_[i].address == nullptr || (i == current_ && slots_[i].leases == 0)) return true;
        }
        return false;
    }

    bool install(void* address, uint32_t bytes) noexcept {
        if (address == nullptr || bytes == 0 || current_ != capacity) return false;
        for (const auto& slot : slots_) {
            if (slot.address == address) return false;
        }
        for (uint32_t i = 0; i < capacity; ++i) {
            if (slots_[i].address == nullptr) {
                slots_[i] = {address, bytes, 0, 0, 0};
                current_ = i;
                return true;
            }
        }
        return false;
    }

    // A nonzero generation tags exactly one lease on an untagged slot and is
    // never issued again: it must exceed every earlier issue. initiator is the
    // owned MAP requester's lease (0 for legacy MAP); it is metadata only.
    void* acquire(uint32_t generation=0,uint32_t initiator=0) noexcept {
        if (current_ == capacity || slots_[current_].leases == UINT32_MAX) return nullptr;
        Slot& slot=slots_[current_];
        if (generation!=0) {
            if (slot.generation!=0 || slot.leases!=0 || generation<=issued_) return nullptr;
            slot.generation=generation;
            slot.initiator=initiator;
            issued_=generation;
        }
        ++slots_[current_].leases;
        return slots_[current_].address;
    }

    // Returns false for an unknown address or a repeated/unbalanced unmap.
    bool release(void* address) noexcept {
        for (auto& slot : slots_) {
            if (slot.address == address && address != nullptr && slot.leases != 0) {
                --slot.leases;
                return true;
            }
        }
        return false;
    }

    // Private DDI retirement retries carry their original nonwrapping token.
    // Comparing generations prevents a reused pool address from releasing a
    // newer allocation after a delayed or uncertain unmap acknowledgement.
    bool release_generation(uint32_t generation) noexcept {
        Slot* slot=leased_slot(generation);
        if (slot==nullptr) return false;
        --slot->leases;
        return true;
    }

    // Authoritative classification for idempotent resolve. A generation at or
    // below the watermark that is not leased now was issued and retired, or
    // was a mode-only token that never named RAM; either way nothing is held
    // under it and nothing ever will be. Above the watermark it is unknown.
    enum class LeaseState { held, not_held, unknown };
    LeaseState classify(uint32_t generation,uint32_t& initiator) const noexcept {
        initiator=0;
        if (generation==0 || generation>issued_) return LeaseState::unknown;
        for (const auto& slot:slots_) {
            if (slot.generation==generation && slot.leases!=0) {
                initiator=slot.initiator;
                return LeaseState::held;
            }
        }
        return LeaseState::not_held;
    }

    uint32_t issued_generation() const noexcept { return issued_; }

    // Outstanding leases across every retained slot, saturating.
    uint32_t held_leases() const noexcept {
        uint32_t total=0;
        for (const auto& slot:slots_) {
            total=slot.leases>UINT32_MAX-total ? UINT32_MAX : total+slot.leases;
        }
        return total;
    }

    void retire() noexcept { current_ = capacity; }

    // Detach one unleased retired allocation. The native owner alone frees it.
    void* reclaim() noexcept {
        for (uint32_t i = 0; i < capacity; ++i) {
            auto& slot = slots_[i];
            if (i != current_ && slot.address != nullptr && slot.leases == 0) {
                void* address = slot.address;
                slot = {};
                return address;
            }
        }
        return nullptr;
    }

    void* current() const noexcept {
        return current_ == capacity ? nullptr : slots_[current_].address;
    }

    // A generation credential is meaningful only while its RAM lease exists.
    // Tokens never authorize another slot or a buffer already reclaimed.
    bool leased_generation(uint32_t generation) const noexcept {
        if (generation==0) return false;
        for (const auto& slot:slots_) {
            if (slot.generation==generation && slot.leases!=0) return true;
        }
        return false;
    }

    bool current_leased_generation(uint32_t generation) const noexcept {
        return generation!=0 && current_!=capacity &&
            slots_[current_].generation==generation && slots_[current_].leases!=0;
    }

    uint32_t current_generation() const noexcept {
        return current_==capacity ? 0U : slots_[current_].generation;
    }

    // Compare the already-returned kernel mapping without dereferencing it.
    // Read-only mode inventory cannot authenticate an old same-sized PDEV.
    // A missing, unleased or retired allocation never yields a credential.
    uint32_t current_mapping_generation(const void* address) const noexcept {
        if (current_==capacity || address==nullptr) return 0;
        const Slot& slot=slots_[current_];
        return slot.address==address && slot.leases!=0 ? slot.generation : 0U;
    }

    uint32_t current_bytes() const noexcept {
        return current_ == capacity ? 0 : slots_[current_].bytes;
    }

private:
    struct Slot final {
        void* address = nullptr;
        uint32_t bytes = 0;
        uint32_t leases = 0;
        uint32_t generation = 0;
        uint32_t initiator = 0;
    };

    Slot* leased_slot(uint32_t generation) noexcept {
        if (generation==0) return nullptr;
        for (auto& slot:slots_) {
            if (slot.generation==generation && slot.leases!=0) return &slot;
        }
        return nullptr;
    }

    Slot slots_[capacity]{};
    uint32_t current_ = capacity;
    // Highest generation ever issued. Retained across native restarts with
    // the ledger so a later MAP can never reissue a retired credential.
    uint32_t issued_ = 0;
};

} // namespace lightning::reactos

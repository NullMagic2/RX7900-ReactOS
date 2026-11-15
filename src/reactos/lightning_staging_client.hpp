/*
Module: lightning.reactos.staging_client
Purpose: Retain exact per-PDEV staging retirement credentials across failures.
Created: 2026-10-22
Architecture: Allocation-free display-client ownership; native staging owns RAM.
*/
#pragma once
#include "lightning_staging.hpp"

namespace lightning::reactos {

// Caller serializes the PDEV. Addresses identify only previously returned MAP
// results and are never dereferenced here. Native retirement receives the
// nonwrapping generation, so a delayed retry cannot release a reused address.
class StagingClientLeases final {
public:
    using Release=bool (*)(void* context,uint32_t generation) noexcept;

    bool has_room() const noexcept {
        for (const auto& slot:slots_) {
            if (slot.generation==0) return true;
        }
        return false;
    }

    bool remember(void* address,uint32_t generation) noexcept {
        if (address==nullptr || generation==0) return false;
        for (const auto& slot:slots_) {
            if (slot.address==address) return slot.generation==generation && !slot.retiring;
            if (slot.generation==generation) return false;
        }
        for (auto& slot:slots_) {
            if (slot.generation==0) {
                slot={address,generation,false};
                return true;
            }
        }
        return false;
    }

    // A successful native MAP can be unusable to GDI if its returned layout is
    // malformed. Its acknowledged generation still owns RAM, even without a
    // trustworthy address. Preserve that token until native release confirms.
    bool remember_unpublished(uint32_t generation) noexcept {
        if (generation==0) return false;
        for (const auto& slot:slots_) {
            if (slot.generation==generation) return false;
        }
        for (auto& slot:slots_) {
            if (slot.generation==0) {
                slot={nullptr,generation,true};
                return true;
            }
        }
        return false;
    }

    bool retire(void* address,Release release,void* context) noexcept {
        for (auto& slot:slots_) {
            if (slot.address!=address || address==nullptr) continue;
            slot.retiring=true;
            return release_slot(slot,release,context);
        }
        return false;
    }

    // Surprise-removal transfer: the caller has already moved every engine
    // view off leased RAM, so each remaining credential, active or pending,
    // becomes a generation-only retirement. Drain afterwards.
    void retire_all() noexcept {
        for (auto& slot:slots_) {
            if (slot.generation!=0) slot.retiring=true;
        }
    }

    bool empty() const noexcept {
        for (const auto& slot:slots_) {
            if (slot.generation!=0) return false;
        }
        return true;
    }

    // Failure preserves the exact token and counts against the native bound.
    // Call before another MAP and during PDEV teardown; never retry an address
    // through the legacy unmap ABI after an uncertain completion.
    bool drain(Release release,void* context) noexcept {
        bool complete=true;
        for (auto& slot:slots_) {
            if (slot.retiring && !release_slot(slot,release,context)) complete=false;
        }
        return complete;
    }

private:
    struct Slot final {
        void* address=nullptr;
        uint32_t generation=0;
        bool retiring=false;
    };

    static bool release_slot(Slot& slot,Release release,void* context) noexcept {
        if (release==nullptr || !release(context,slot.generation)) return false;
        slot={};
        return true;
    }

    Slot slots_[StagingLedger::capacity]{};
};

} // namespace lightning::reactos

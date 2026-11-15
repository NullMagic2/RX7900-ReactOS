/*
Module: lightning.reactos.staging_resolve
Purpose: Answer idempotent generation-checked staging release-or-confirm requests.
Created: 2026-10-23
Architecture: Allocation-free policy shared by the miniport and display client;
the native miniport owns the ledger, its mutex and every RAM free.
*/
#pragma once
#include "lightning_present_abi.hpp"
#include "lightning_staging.hpp"

namespace lightning::reactos {

constexpr bool valid_staging_resolve_request(const StagingResolveRequest& request,uint32_t bytes) noexcept {
    return bytes==sizeof(request) && request.size==sizeof(request) &&
        request.version==cpu_scanout_abi_version && request.lease_generation!=0 &&
        (request.operation==staging_resolve_release || request.operation==staging_resolve_query);
}

// Caller holds the ledger's native mutex and has validated the request. A
// release decrements the one lease its generation names, at most once; every
// later request for that generation answers not_held. RAM is reclaimed by the
// caller afterwards. device_lost reports surprise-removal containment.
inline StagingResolveReceipt resolve_staging(StagingLedger& staging,
    const StagingResolveRequest& request,bool device_lost) noexcept {
    StagingResolveReceipt receipt{};
    receipt.size=sizeof(receipt);
    receipt.version=cpu_scanout_abi_version;
    receipt.lease_generation=request.lease_generation;
    uint32_t initiator=0;
    const auto state=staging.classify(request.lease_generation,initiator);

    if (state==StagingLedger::LeaseState::unknown) {
        receipt.outcome=staging_lease_unknown;
    } else if (state==StagingLedger::LeaseState::not_held) {
        receipt.outcome=staging_lease_not_held;
    } else if (request.operation==staging_resolve_release) {
        receipt.outcome=staging.release_generation(request.lease_generation) ?
            staging_lease_released : staging_lease_held;
    } else {
        receipt.outcome=staging_lease_held;
        if (request.initiator_lease!=0 && initiator==request.initiator_lease) {
            receipt.flags|=staging_flag_initiator;
        }
    }

    if (device_lost) receipt.flags|=staging_flag_device_lost;
    receipt.issued_generation=staging.issued_generation();
    receipt.held_leases=staging.held_leases();
    return receipt;
}

// Client check: only an exact echo of this request's generation is an answer.
// A receipt for another generation (reordered, duplicated or stale) is not
// evidence about either lease and must leave both credentials untouched.
constexpr bool valid_staging_resolve_receipt(const StagingResolveReceipt& receipt,
    const StagingResolveRequest& request) noexcept {
    const bool known_outcome=receipt.outcome==staging_lease_released ||
        receipt.outcome==staging_lease_not_held || receipt.outcome==staging_lease_held ||
        receipt.outcome==staging_lease_unknown;
    const bool release_answer=request.operation!=staging_resolve_release ||
        receipt.outcome!=staging_lease_held;
    const bool query_answer=request.operation!=staging_resolve_query ||
        receipt.outcome!=staging_lease_released;
    constexpr uint32_t known_flags=staging_flag_device_lost|staging_flag_initiator;
    return receipt.size==sizeof(receipt) && receipt.version==cpu_scanout_abi_version &&
        receipt.lease_generation==request.lease_generation && known_outcome &&
        release_answer && query_answer && (receipt.flags&~known_flags)==0 &&
        (receipt.outcome==staging_lease_unknown ||
            receipt.lease_generation<=receipt.issued_generation) &&
        receipt.reserved==0;
}

// Debug-only acknowledgement fault plan. Native code consumes it only when
// built with RX7900_LIGHTNING_LEASE_FAULTS and its registry value is nonzero.
// Bits 0-7: release requests applied whose acknowledgement is then withheld.
// Bits 8-15: release requests dropped before the ledger is touched (first).
enum class StagingFault { none, drop_before_apply, lost_after_apply };

class StagingFaultPlan final {
public:
    static constexpr uint32_t count_mask=0xffU;
    static constexpr uint32_t drop_shift=8;

    void configure(uint32_t packed) noexcept {
        lost_after_apply_=packed&count_mask;
        drop_before_apply_=(packed>>drop_shift)&count_mask;
    }

    StagingFault next(uint32_t operation) noexcept {
        if (operation!=staging_resolve_release) return StagingFault::none;

        if (drop_before_apply_!=0) {
            --drop_before_apply_;
            ++injected_;
            return StagingFault::drop_before_apply;
        }

        if (lost_after_apply_!=0) {
            --lost_after_apply_;
            ++injected_;
            return StagingFault::lost_after_apply;
        }

        return StagingFault::none;
    }

    uint32_t injected() const noexcept { return injected_; }

private:
    uint32_t lost_after_apply_=0;
    uint32_t drop_before_apply_=0;
    uint32_t injected_=0;
};

} // namespace lightning::reactos

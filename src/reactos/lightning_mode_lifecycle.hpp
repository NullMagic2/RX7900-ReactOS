/*
Module: lightning.reactos.mode_lifecycle
Purpose: Authenticate PDEV transitions and narrowly authorize failed-mode rollback.
Created: 2026-10-22
Architecture: Uses the existing native staging ledger; owns no second resources.
*/
#pragma once
#include "lightning_present_abi.hpp"
#include "lightning_staging.hpp"

namespace lightning::reactos {

struct ModeRollbackCredential final {
    uint32_t lease_generation;
    uint32_t mode_generation;
};

// Snapshot the current mapped owner immediately before a transition or MAP.
// The lease may predate multiple acknowledged mode tokens; only the latest
// token is a permissible failed-replacement rollback credential.
inline ModeRollbackCredential current_mode_rollback_credential(
    const StagingLedger& staging,uint32_t mode_generation,
    uint32_t initiating_lease=0) noexcept {
    const uint32_t lease=staging.current_generation();
    if (mode_generation==0) return {};
    if (staging.current_leased_generation(lease)) return {lease,mode_generation};
    // A validated owned transition may retry after a failed replacement has
    // released the current slot. Its explicit lease remains the sole owner.
    return staging.leased_generation(initiating_lease) ?
        ModeRollbackCredential{initiating_lease,mode_generation} : ModeRollbackCredential{};
}

constexpr bool valid_mode_owner(const ModeOwnerRequest& owner,uint32_t bytes) noexcept {
    return owner.size==bytes && owner.version==cpu_scanout_abi_version &&
        owner.mode_generation!=0 && owner.lease_generation!=0;
}

// Shared by owned mode transitions and owned MAP. A retained old RAM lease
// cannot displace a different PDEV's mapped replacement even when the mode
// generation has advanced independently of that replacement's lease token.
inline bool current_mode_lease_authorized(const StagingLedger& staging,
    ModeRollbackCredential rollback,uint32_t lease_generation) noexcept {
    const uint32_t active_lease=staging.current_generation();
    if (staging.current_leased_generation(active_lease)) {
        return lease_generation==active_lease;
    }
    return lease_generation==rollback.lease_generation &&
        staging.leased_generation(rollback.lease_generation);
}

// Called with VideoState's mutex held. A retired PDEV may reactivate solely
// after the immediate replacement failed to retain a mapped owner. Keeping a
// RAM lease alive is necessary but insufficient while a new PDEV owns scanout.
inline bool mode_transition_authorized(const StagingLedger& staging,
    uint32_t current_generation,ModeRollbackCredential rollback,
    const ModeTransitionRequest& request) noexcept {
    if (!valid_mode_owner(request.owner,sizeof(request)) || request.enable>1 || request.reserved!=0) return false;
    if (request.owner.mode_generation==current_generation) {
        return current_mode_lease_authorized(staging,rollback,request.owner.lease_generation);
    }
    return request.enable==1 && rollback.mode_generation!=0 &&
        rollback.mode_generation<current_generation &&
        request.owner.lease_generation==rollback.lease_generation &&
        request.owner.mode_generation==rollback.mode_generation &&
        staging.leased_generation(rollback.lease_generation) &&
        (!staging.current_leased_generation(staging.current_generation()) ||
         staging.current_generation()==rollback.lease_generation);
}

// framebuf DrvEnableSurface sends the SDK SET_CURRENT_MODE then MAP with no
// credential, before any lease of its own exists. The native owner therefore
// preauthenticates that pair from its own state: no mapped owner holds the
// current allocation, or that exact owner yielded scanout through its own
// authenticated disable at this mode token (yield) and nothing changed since.
// A stale or concurrent PDEV can never displace an active mapped owner.
inline bool initial_mode_authorized(const StagingLedger& staging,uint32_t current_generation,
    ModeRollbackCredential yield,bool quiesced) noexcept {
    const uint32_t active_lease=staging.current_generation();
    if (!staging.current_leased_generation(active_lease)) return true;
    return quiesced && current_generation!=0 && yield.lease_generation==active_lease &&
        yield.mode_generation==current_generation;
}

// The MAP half is a single ticket bound to the exact token the admitted mode
// set produced; every later mode transition advances it. Without the ticket
// no SDK MAP is ever answered, so no unauthenticated mapping path remains.
constexpr bool initial_map_authorized(uint32_t ticket,uint32_t current_generation,bool quiesced) noexcept {
    return ticket!=0 && ticket==current_generation && !quiesced;
}

// MAP's rollback snapshot. An owned MAP's initiator acknowledged the current
// token, so it is snapshotted afresh. The preauthenticated framebuf MAP's PDEV
// never acknowledged the token its own SET produced: the yielded owner's
// latest acknowledged token, captured by that SET, remains the only one.
inline ModeRollbackCredential map_rollback_credential(const StagingLedger& staging,
    uint32_t mode_generation,uint32_t initiating_lease,bool initial,ModeRollbackCredential prior) noexcept {
    return initial ? prior : current_mode_rollback_credential(staging,mode_generation,initiating_lease);
}

// Power resume restores the owner's intent. A display its mapped owner
// yielded stays blank until that owner or a preauthenticated successor
// enables it; resume never revives a PDEV win32k has disabled.
constexpr bool resume_keeps_yield(ModeRollbackCredential yield,uint32_t current_generation,
    bool quiesced,bool blank_published) noexcept {
    return quiesced && blank_published && yield.mode_generation!=0 &&
        yield.mode_generation==current_generation;
}

// A preauthenticated mode set whose MAP is still pending has no mapped image:
// resume commits it cleared instead of showing the yielded owner's RAM.
constexpr bool resume_preserves(uint32_t initial_map,uint32_t current_generation) noexcept {
    return initial_map==0 || initial_map!=current_generation;
}

} // namespace lightning::reactos

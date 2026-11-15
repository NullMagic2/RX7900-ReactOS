/*
Module: lightning.reactos.gdi_lifecycle
Purpose: Authorize reuse only from an actual monotonic backend retirement token.
Created: 2026-10-21
Architecture: Shared pure policy for the native reset boundary and its tests.
*/
#pragma once
#include "lightning-gdi.h"

namespace lightning::reactos::gdi {
// Shared one-second execution/retirement deadline. Event waits and the legacy
// bounded poller consume the same budget; neither owns a separate timeout.
constexpr uint64_t completion_wait_limit_100ns=10000000ULL;

struct Retirement final {
    uint32_t generation=0;
    uint32_t flags=0;
    uint32_t epoch=0;
    uint32_t mode_generation=0;
    uint32_t accepted_sequence=0;
    uint32_t completed_sequence=0;
    uint32_t status=LIGHTNING_GDI_IDLE;
};

// Called only after our own RESET doorbell. An unchanged generation is valid
// solely for a fresh controller which already proved retirement and owned no
// canvas/transfer. Tokens never wrap. LOST/resetting/poisoned are not release.
constexpr bool retirement_confirmed(uint32_t ownership_token,bool already_retired,
                                    Retirement observed) noexcept {
    return observed.generation!=0 &&
        (observed.generation>ownership_token ||
         (already_retired && observed.generation==ownership_token)) &&
        (observed.flags&LIGHTNING_GDI_FLAG_RETIRED)!=0 &&
        (observed.flags&(LIGHTNING_GDI_FLAG_READY|LIGHTNING_GDI_FLAG_RESETTING|LIGHTNING_GDI_FLAG_POISONED))==0 &&
        observed.status==LIGHTNING_GDI_IDLE &&
        observed.epoch==0 && observed.mode_generation==0 &&
        observed.accepted_sequence==0 && observed.completed_sequence==0;
}

// Delayed host GL readiness must not add an IOCTL to every GDI operation. Use
// elapsed counter ticks without overflow-prone deadline addition. If QPC is
// unavailable, one attempt per256 visible operations remains bounded.
class PromotionSchedule final {
public:
    void configure(uint64_t frequency,uint64_t now) noexcept {
        frequency_=frequency; last_=now; operations_=0;
    }
    bool due(uint64_t now) noexcept {
        if (frequency_!=0) {
            if (now>=last_ && now-last_<frequency_) return false;
            last_=now;
            return true;
        }
        if (++operations_<256U) return false;
        operations_=0;
        return true;
    }
private:
    uint64_t frequency_=0;
    uint64_t last_=0;
    uint32_t operations_=0;
};
} // namespace lightning::reactos::gdi

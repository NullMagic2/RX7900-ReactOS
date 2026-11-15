/*
Module: lightning.reactos.gdi_completion
Purpose: Own the optional completion interrupt, DPC wake and native wait event.
Created: 2026-10-22
Architecture: Nonpaged miniport-extension storage; VideoPort excludes its ISR.
*/
#pragma once

#include "video_sdk.hpp"
#include <stdint.h>

namespace lightning::reactos::gdi {

class CompletionSignal final {
public:
    CompletionSignal() noexcept=default;
    CompletionSignal(const CompletionSignal&)=delete;
    CompletionSignal& operator=(const CompletionSignal&)=delete;
    // Call once after placement construction, before the extension is published.
    // The extension and this object remain owned by VideoPort through ISR/DPC
    // retirement. No external event or callback allocation escapes this owner.
    void initialize(PVOID extension) noexcept;
    // PASSIVE_LEVEL, miniport mutex held. Enable only after HwInitialize proves
    // VideoPort connected the configured IRQ. A missing host capability keeps
    // the existing bounded polling path operational.
    bool enable(PULONG registers,bool connected) noexcept;
    // PASSIVE_LEVEL, miniport mutex held. Excludes the ISR before unmapping and
    // retires the exact private DPC. Surprise loss passes false: no MMIO access.
    void disable(bool hardware_access=true) noexcept;
    // DIRQL, VideoPort interrupt lock held. Touch only the enabled mapped IRQ
    // window and nonpaged storage; never acquire the PASSIVE miniport mutex.
    bool interrupt() noexcept;
    // PASSIVE_LEVEL, miniport mutex held. Clear, recheck the authoritative GPU
    // ledger, then wait. A signaled event is only a reason to reread that ledger.
    bool prepare() noexcept;
    bool wait(uint64_t remaining_100ns) noexcept;
private:
    static BOOLEAN NTAPI enable_synchronized(PVOID context);
    static BOOLEAN NTAPI disable_synchronized(PVOID context);
    static VOID NTAPI completion_dpc(PKDPC dpc,PVOID context,PVOID argument1,PVOID argument2);
    PVOID extension_=nullptr;
    PULONG registers_=nullptr;
    PULONG candidate_=nullptr;
    KEVENT completion_{};
    KDPC dpc_{};
    volatile LONG dpc_references_=0;
    bool initialized_=false;
    bool hardware_access_=false;
};

} // namespace lightning::reactos::gdi

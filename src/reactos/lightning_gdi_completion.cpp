/*
Module: lightning.reactos.gdi_completion
Purpose: Signal execution progress through a real ISR/DPC and retain exact lifetime.
Created: 2026-10-22
Architecture: Reuses VideoPort interrupt synchronization and kernel event/DPC APIs.
*/
#include "lightning_gdi_completion.hpp"
#include "lightning-gdi.h"
#include "lightning_gdi_lifecycle.hpp"

namespace lightning::reactos::gdi {
namespace {
constexpr LONGLONG rundown_retry_100ns=-10000;

PULONG address(PULONG registers,uint32_t offset) noexcept {
    // The miniport validated/mapped the complete control window. These offsets
    // are fixed shared-ABI constants, never values supplied by a caller.
    return registers+offset/sizeof(ULONG);
}
}

void CompletionSignal::initialize(PVOID extension) noexcept {
    if (initialized_ || extension==nullptr) return;
    extension_=extension;
    KeInitializeEvent(&completion_,NotificationEvent,FALSE);
    KeInitializeDpc(&dpc_,completion_dpc,this);
    initialized_=true;
}

BOOLEAN NTAPI CompletionSignal::enable_synchronized(PVOID context) {
    auto& signal=*static_cast<CompletionSignal*>(context);
    auto* registers=signal.candidate_;
    if (registers==nullptr) return FALSE;
    VideoPortWriteRegisterUlong(address(registers,LIGHTNING_GDI_REG_IRQ_ENABLE),0);
    VideoPortWriteRegisterUlong(address(registers,LIGHTNING_GDI_REG_IRQ),LIGHTNING_GDI_IRQ_MASK);
    VideoPortWriteRegisterUlong(address(registers,LIGHTNING_GDI_REG_IRQ_ENABLE),LIGHTNING_GDI_IRQ_MASK);
    if (VideoPortReadRegisterUlong(address(registers,LIGHTNING_GDI_REG_IRQ_ENABLE))!=LIGHTNING_GDI_IRQ_MASK) {
        VideoPortWriteRegisterUlong(address(registers,LIGHTNING_GDI_REG_IRQ_ENABLE),0);
        return FALSE;
    }
    // Publish only while holding the same interrupt lock as HwInterrupt. An
    // interrupt raised by ENABLE waits until this mapping owner is visible.
    signal.registers_=registers;
    return TRUE;
}

bool CompletionSignal::enable(PULONG registers,bool connected) noexcept {
    if (!initialized_ || !connected || registers==nullptr) return false;
    if (registers_==registers) return true;
    if (registers_!=nullptr) return false;
    if ((VideoPortReadRegisterUlong(address(registers,LIGHTNING_GDI_REG_CAPABILITIES))&
        LIGHTNING_GDI_CAP_COMPLETION_IRQ)==0) return false;
    candidate_=registers;
    const bool enabled=VideoPortSynchronizeExecution(extension_,VpMediumPriority,
        enable_synchronized,this)!=FALSE;
    candidate_=nullptr;
    return enabled;
}

BOOLEAN NTAPI CompletionSignal::disable_synchronized(PVOID context) {
    auto& signal=*static_cast<CompletionSignal*>(context);
    auto* registers=signal.registers_;
    signal.registers_=nullptr;
    if (signal.hardware_access_ && registers!=nullptr) {
        VideoPortWriteRegisterUlong(address(registers,LIGHTNING_GDI_REG_IRQ_ENABLE),0);
        VideoPortWriteRegisterUlong(address(registers,LIGHTNING_GDI_REG_IRQ),LIGHTNING_GDI_IRQ_MASK);
    }
    return TRUE;
}

void CompletionSignal::disable(bool hardware_access) noexcept {
    if (!initialized_) return;
    hardware_access_=hardware_access;
    VideoPortSynchronizeExecution(extension_,VpMediumPriority,disable_synchronized,this);
    hardware_access_=false;
    // No ISR can enqueue again after the synchronized mapping removal. Every
    // queued or running DPC drops its own reference as its final storage access,
    // so rundown waits for that instead of removing the DPC. On an SMP kernel
    // ReactOS KeRemoveQueueDpc returns TRUE from an unlocked DpcData read even
    // when another processor has just dequeued the DPC to run it (ke/dpc.c);
    // trusting that result would retire the reference twice. The ISR inserts on
    // its own processor, which drains it when leaving the interrupt. Do not use
    // global KeFlushQueuedDpcs as proof of another processor's running DPC
    // retirement on the pinned ReactOS release either.
    while (InterlockedCompareExchange(&dpc_references_,0,0)!=0) {
        LARGE_INTEGER delay{};
        delay.QuadPart=rundown_retry_100ns;
        KeDelayExecutionThread(KernelMode,FALSE,&delay);
    }
    KeClearEvent(&completion_);
}

VOID NTAPI CompletionSignal::completion_dpc(PKDPC,PVOID context,PVOID,PVOID) {
    auto& signal=*static_cast<CompletionSignal*>(context);
    KeSetEvent(&signal.completion_,IO_NO_INCREMENT,FALSE);
    // Last access to the owner: disable may return after observing zero.
    InterlockedDecrement(&signal.dpc_references_);
}

bool CompletionSignal::interrupt() noexcept {
    if (registers_==nullptr) return false;
    const ULONG causes=VideoPortReadRegisterUlong(address(registers_,LIGHTNING_GDI_REG_IRQ))&
        LIGHTNING_GDI_IRQ_MASK;
    if (causes==0) return false;
    // Acknowledge only this source; Venus and scanout retain separate IRQ owners.
    VideoPortWriteRegisterUlong(address(registers_,LIGHTNING_GDI_REG_IRQ),causes);
    InterlockedIncrement(&dpc_references_);
    if (!KeInsertQueueDpc(&dpc_,nullptr,nullptr)) InterlockedDecrement(&dpc_references_);
    return true;
}

bool CompletionSignal::prepare() noexcept {
    if (registers_==nullptr || VideoPortReadRegisterUlong(address(registers_,LIGHTNING_GDI_REG_IRQ_ENABLE))!=
        LIGHTNING_GDI_IRQ_MASK) return false;
    KeClearEvent(&completion_);
    return true;
}

bool CompletionSignal::wait(uint64_t remaining_100ns) noexcept {
    if (registers_==nullptr || remaining_100ns==0 || remaining_100ns>completion_wait_limit_100ns) return false;
    LARGE_INTEGER timeout{};
    timeout.QuadPart=-static_cast<LONGLONG>(remaining_100ns);
    const NTSTATUS status=KeWaitForSingleObject(&completion_,Executive,KernelMode,FALSE,&timeout);
    return status==STATUS_SUCCESS || status==STATUS_TIMEOUT;
}

} // namespace lightning::reactos::gdi

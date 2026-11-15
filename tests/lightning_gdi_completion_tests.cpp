/*
Module: lightning.tests.gdi_completion
Purpose: Exercise the production ISR/DPC owner with explicit kernel API boundaries.
Created: 2026-10-22
Architecture: Exact SDK types; no GPU execution or kernel scheduling is simulated.
*/
#include "lightning_gdi_completion.hpp"
#include "lightning-gdi.h"
#include <stdio.h>
#include <stdlib.h>

namespace {
using lightning::reactos::gdi::CompletionSignal;
unsigned checks=0,failures=0;
constexpr size_t control_words=LIGHTNING_GDI_END/sizeof(ULONG);
ULONG registers[control_words]{};
unsigned reads=0,writes=0,synchronized_calls=0,queued_calls=0,dpc_calls=0,waits=0,delays=0;
PKDEFERRED_ROUTINE callback=nullptr;
PVOID callback_context=nullptr;
PKDPC queued=nullptr;
bool event_signaled=false,run_on_wait=false,reject_enable=false;
bool remove_fails_once=false,fail_wait=false;
// SMP model of ReactOS KeRemoveQueueDpc (ke/dpc.c): another processor has
// dequeued the DPC to run it, yet the unlocked DpcData read still returns TRUE.
bool remove_reports_running=false;
constexpr unsigned rundown_delay_limit=1000;
LONGLONG last_timeout=0;

void check(bool value,const char* name) {
    ++checks;
    if (!value) {++failures; printf("FAIL %s\n",name);}
}

void run_dpc() {
    if (queued==nullptr || callback==nullptr) return;
    auto* item=queued;
    queued=nullptr;
    ++dpc_calls;
    callback(item,callback_context,nullptr,nullptr);
}

void reset_boundary() {
    for (auto& value:registers) value=0;
    registers[LIGHTNING_GDI_REG_CAPABILITIES/sizeof(ULONG)]=LIGHTNING_GDI_CAP_COMPLETION_IRQ;
    reads=writes=synchronized_calls=queued_calls=dpc_calls=waits=delays=0;
    callback=nullptr;
    callback_context=nullptr;
    queued=nullptr;
    event_signaled=run_on_wait=reject_enable=remove_fails_once=fail_wait=false;
    remove_reports_running=false;
    last_timeout=0;
}

void signal_irq(ULONG causes) {
    registers[LIGHTNING_GDI_REG_IRQ/sizeof(ULONG)]|=causes;
}
}

extern "C" VOID NTAPI KeInitializeEvent(PRKEVENT,EVENT_TYPE type,BOOLEAN state) {
    check(type==NotificationEvent,"production completion uses a notification event");
    event_signaled=state!=FALSE;
}
extern "C" VOID NTAPI KeInitializeDpc(PRKDPC,PKDEFERRED_ROUTINE routine,PVOID context) {
    callback=routine;
    callback_context=context;
}
extern "C" BOOLEAN NTAPI KeInsertQueueDpc(PRKDPC dpc,PVOID,PVOID) {
    ++queued_calls;
    if (queued!=nullptr) return FALSE;
    queued=dpc;
    return TRUE;
}
extern "C" BOOLEAN NTAPI KeRemoveQueueDpc(PRKDPC dpc) {
    if (remove_fails_once) {remove_fails_once=false; return FALSE;}
    if (remove_reports_running && queued==dpc) return TRUE;
    if (queued!=dpc) return FALSE;
    queued=nullptr;
    return TRUE;
}
extern "C" LONG NTAPI KeSetEvent(PRKEVENT,KPRIORITY,BOOLEAN) {
    const LONG previous=event_signaled ? 1 : 0;
    event_signaled=true;
    return previous;
}
extern "C" VOID NTAPI KeClearEvent(PRKEVENT) {event_signaled=false;}
extern "C" NTSTATUS NTAPI KeWaitForSingleObject(PVOID,KWAIT_REASON,KPROCESSOR_MODE,BOOLEAN,
                                                PLARGE_INTEGER timeout) {
    ++waits;
    last_timeout=timeout==nullptr ? 0 : timeout->QuadPart;
    if (run_on_wait) run_dpc();
    return fail_wait ? STATUS_UNSUCCESSFUL : event_signaled ? STATUS_SUCCESS : STATUS_TIMEOUT;
}
extern "C" NTSTATUS NTAPI KeDelayExecutionThread(KPROCESSOR_MODE,BOOLEAN,PLARGE_INTEGER) {
    ++delays;
    if (delays>rundown_delay_limit) {
        // A corrupted reference count never reaches zero: report, do not hang.
        printf("FAIL DPC rundown did not terminate after %u delays\n",delays);
        exit(1);
    }
    run_dpc();
    return STATUS_SUCCESS;
}
extern "C" BOOLEAN NTAPI VideoPortSynchronizeExecution(PVOID,VIDEO_SYNCHRONIZE_PRIORITY priority,
    PMINIPORT_SYNCHRONIZE_ROUTINE routine,PVOID context) {
    ++synchronized_calls;
    check(priority==VpMediumPriority,"mapping transitions exclude the actual VideoPort ISR");
    return routine(context);
}
extern "C" ULONG NTAPI VideoPortReadRegisterUlong(PULONG location) {
    ++reads;
    return *location;
}
extern "C" VOID NTAPI VideoPortWriteRegisterUlong(PULONG location,ULONG value) {
    ++writes;
    if (location==registers+LIGHTNING_GDI_REG_IRQ/sizeof(ULONG)) {*location&=~value; return;}
    *location=reject_enable && location==registers+LIGHTNING_GDI_REG_IRQ_ENABLE/sizeof(ULONG) ? 0 : value;
}

int main() {
    reset_boundary();
    CompletionSignal signal;
    signal.initialize(registers);
    check(!signal.enable(registers,false) && writes==0,"unconnected IRQ keeps the polling path");
    registers[LIGHTNING_GDI_REG_CAPABILITIES/sizeof(ULONG)]=0;
    check(!signal.enable(registers,true) && writes==0,"old host does not receive unknown IRQ writes");
    registers[LIGHTNING_GDI_REG_CAPABILITIES/sizeof(ULONG)]=LIGHTNING_GDI_CAP_COMPLETION_IRQ;
    signal_irq(LIGHTNING_GDI_IRQ_COMPLETED);
    check(signal.enable(registers,true) && registers[LIGHTNING_GDI_REG_IRQ/sizeof(ULONG)]==0 &&
        registers[LIGHTNING_GDI_REG_IRQ_ENABLE/sizeof(ULONG)]==LIGHTNING_GDI_IRQ_MASK,
        "connected optional IRQ masks, clears stale causes and verifies enable");
    const unsigned enabled_writes=writes;
    check(signal.enable(registers,true) && writes==enabled_writes,"same live mapping retains its IRQ owner");
    check(!signal.interrupt() && queued_calls==0,"unrelated shared IRQ does not enqueue work");
    signal_irq(0x80000000UL);
    check(!signal.interrupt() && queued_calls==0,"unknown cause remains outside the GDI IRQ owner");
    signal_irq(LIGHTNING_GDI_IRQ_COMPLETED|LIGHTNING_GDI_IRQ_RETIRED);
    check(signal.interrupt() && queued_calls==1 && queued!=nullptr &&
        registers[LIGHTNING_GDI_REG_IRQ/sizeof(ULONG)]==0x80000000UL && !event_signaled,
        "ISR acknowledges only GDI causes and defers event signaling");
    signal_irq(LIGHTNING_GDI_IRQ_COMPLETED);
    check(signal.interrupt() && queued_calls==2 && queued!=nullptr,
        "coalesced IRQ retains exactly one queued DPC reference");
    check(signal.prepare() && !event_signaled,"wait preparation clears a previous event before ledger recheck");
    run_dpc();
    check(event_signaled && dpc_calls==1,"actual DPC boundary signals the event after ISR return");
    check(signal.wait(12345) && waits==1 && last_timeout==-12345,
        "IRQ between ledger read and sleep remains signaled with the bounded deadline");
    check(signal.prepare() && signal.wait(123) && !event_signaled,
        "kernel timeout is only a reason for the caller to reread its GPU ledger");
    registers[LIGHTNING_GDI_REG_IRQ_ENABLE/sizeof(ULONG)]=0;
    check(!signal.prepare(),"host-side interrupt masking prevents an unarmed blocking wait");
    registers[LIGHTNING_GDI_REG_IRQ_ENABLE/sizeof(ULONG)]=LIGHTNING_GDI_IRQ_MASK;
    fail_wait=true;
    check(!signal.wait(123),"unexpected kernel wait failure is not reported as a valid wake");
    fail_wait=false;
    const unsigned valid_waits=waits;
    check(!signal.wait(0) && !signal.wait(10000001ULL) && waits==valid_waits,
        "zero and overflowing native wait budgets are rejected before kernel call");
    signal_irq(LIGHTNING_GDI_IRQ_LOST);
    signal.interrupt();
    signal.disable();
    check(queued==nullptr && !signal.prepare() &&
        registers[LIGHTNING_GDI_REG_IRQ_ENABLE/sizeof(ULONG)]==0,
        "normal stop retires a queued DPC after excluding and masking the ISR");
    const unsigned retired_reads=reads,retired_writes=writes;
    check(!signal.interrupt() && reads==retired_reads && writes==retired_writes,
        "retired ISR never touches the released mapping");
    check(signal.enable(registers,true),"resume can establish a fresh owner on the retained device");
    signal_irq(LIGHTNING_GDI_IRQ_COMPLETED);
    signal.interrupt();
    remove_fails_once=true;
    const unsigned running_delays=delays;
    signal.disable();
    remove_fails_once=false;
    check(delays==running_delays+1 && queued==nullptr && !signal.prepare(),
        "running DPC retires its final storage reference before stop returns");
    check(signal.enable(registers,true),"SMP race model establishes a fresh owner");
    signal_irq(LIGHTNING_GDI_IRQ_COMPLETED);
    signal.interrupt();
    remove_reports_running=true;
    const unsigned smp_delays=delays,smp_dpcs=dpc_calls;
    signal.disable();
    remove_reports_running=false;
    check(queued==nullptr && dpc_calls==smp_dpcs+1 && delays>smp_delays && !signal.prepare(),
        "stop never returns while another processor still owes its DPC retirement");
    check(signal.enable(registers,true),"owner after the SMP race is independent");
    signal_irq(LIGHTNING_GDI_IRQ_COMPLETED);
    check(signal.interrupt() && queued!=nullptr,"ISR after the SMP race queues one DPC reference");
    signal.disable();
    check(queued==nullptr && !signal.prepare(),
        "reference count survives the SMP race: the next rundown still terminates");
    check(signal.enable(registers,true),"second resume establishes independent interrupt ownership");
    signal_irq(LIGHTNING_GDI_IRQ_COMPLETED);
    signal.interrupt();
    const unsigned surprise_reads=reads,surprise_writes=writes;
    signal.disable(false);
    check(reads==surprise_reads && writes==surprise_writes && !signal.prepare() && queued==nullptr,
        "surprise loss excludes ISR and drains DPC without accessing missing hardware");
    reject_enable=true;
    check(!signal.enable(registers,true) && !signal.prepare(),
        "failed enable readback does not publish a usable event-wait owner");
    signal.disable(false);
    printf("Native completion IRQ CPU boundaries: %u checks, %u failures; GPU and scheduling untested\n",
        checks,failures);
    return failures==0 ? 0 : 1;
}

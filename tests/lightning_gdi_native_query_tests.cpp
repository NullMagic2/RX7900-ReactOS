/*
Module: lightning.tests.native_query
Purpose: Exercise actual native query caching, live ledger checks and reset invalidation.
Created: 2026-10-22
Architecture: Exact SDK/native implementation with bounded register and kernel API fixtures.
*/
#include "video_sdk.hpp"
#include "lightning_gdi_native.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace {
using namespace lightning::reactos::gdi;
unsigned checks=0,failures=0;
unsigned stalls=0,delays=0;
ULONGLONG clock_value=0;
void check(bool condition,const char* name) {
    ++checks;
    if (!condition) {++failures; printf("FAIL %s\n",name);}
}

struct Registers final {
    void* transfer=nullptr;
    uint32_t reads=0;
    uint32_t magic=LIGHTNING_GDI_MAGIC;
    uint32_t capabilities=LIGHTNING_GDI_CAPABILITIES;
    uint32_t reset_generation=1;
    uint32_t epoch=0,generation=0,accepted=0,completed=0,revision=0;
    uint32_t published=0;
    uint32_t flags=LIGHTNING_GDI_FLAG_RETIRED;
    uint32_t status=LIGHTNING_GDI_IDLE;
    uint32_t fail_address=UINT32_MAX;
    uint32_t completion_reads=0;
    uint32_t event_prepares=0,event_waits=0,spurious_wakes=0;
    uint64_t last_wait_budget=0;
    bool complete_on_prepare=false,event_failure=false,event_timeout=false;
    LightningGdiExternalImage external{};
    uint32_t last_flags=0,flushes=0;
};

bool prepare_event(void* opaque) noexcept {
    auto& state=*static_cast<Registers*>(opaque);
    ++state.event_prepares;
    if (state.complete_on_prepare) state.completed=state.accepted;
    return true;
}

bool wait_event(void* opaque,uint64_t remaining_100ns) noexcept {
    auto& state=*static_cast<Registers*>(opaque);
    ++state.event_waits;
    state.last_wait_budget=remaining_100ns;
    if (state.event_failure) return false;
    if (state.event_timeout) clock_value+=remaining_100ns;
    else if (state.spurious_wakes!=0) --state.spurious_wakes;
    else state.completed=state.accepted;
    return true;
}

bool read_register(void* opaque,uint32_t address,uint32_t& value) noexcept {
    auto& state=*static_cast<Registers*>(opaque);
    ++state.reads;
    value=0;
    if (address==state.fail_address) return false;
    switch (address) {
    case LIGHTNING_GDI_REG_MAGIC:value=state.magic;break;
    case LIGHTNING_GDI_REG_VERSION:value=LIGHTNING_GDI_VERSION;break;
    case LIGHTNING_GDI_REG_CAPABILITIES:value=state.capabilities;break;
    case LIGHTNING_GDI_REG_RESET_GENERATION:value=state.reset_generation;break;
    case LIGHTNING_GDI_REG_TRANSFER_BYTES:value=LIGHTNING_GDI_TRANSFER_BYTES;break;
    case LIGHTNING_GDI_REG_MAX_PIXEL_BYTES:value=LIGHTNING_GDI_MAX_PIXEL_BYTES;break;
    case LIGHTNING_GDI_REG_PUBLISHED_SEQUENCE:value=state.published;break;
    case LIGHTNING_GDI_REG_PUBLISHED_EPOCH:value=state.epoch;break;
    case LIGHTNING_GDI_REG_PUBLISHED_MODE_GENERATION:value=state.generation;break;
    case LIGHTNING_GDI_REG_EPOCH:value=state.epoch;break;
    case LIGHTNING_GDI_REG_MODE_GENERATION:value=state.generation;break;
    case LIGHTNING_GDI_REG_ACCEPTED_SEQUENCE:value=state.accepted;break;
    case LIGHTNING_GDI_REG_COMPLETED_SEQUENCE:
        if (state.completion_reads!=0 && --state.completion_reads==0) state.completed=state.accepted;
        value=state.completed;break;
    case LIGHTNING_GDI_REG_CANVAS_REVISION:value=state.revision;break;
    case LIGHTNING_GDI_REG_FLAGS:value=state.flags;break;
    case LIGHTNING_GDI_REG_STATUS:value=state.status;break;
    case LIGHTNING_GDI_REG_RESULT:value=LIGHTNING_GDI_OK;break;
    default:return false;
    }
    return true;
}

bool write_register(void* opaque,uint32_t address,uint32_t value) noexcept {
    auto& state=*static_cast<Registers*>(opaque);
    if (address!=LIGHTNING_GDI_REG_COMMAND) return false;
    if (value==LIGHTNING_GDI_RESET) {
        ++state.reset_generation;
        state.epoch=state.generation=state.accepted=state.completed=state.revision=state.published=0;
        state.flags=LIGHTNING_GDI_FLAG_RETIRED;
        state.status=LIGHTNING_GDI_IDLE;
        return true;
    }
    if (value==LIGHTNING_GDI_FLUSH && (state.capabilities&LIGHTNING_GDI_CAP_EXTERNAL_MAILBOX)!=0) {
        ++state.flushes;
        return true;
    }
    if (value!=LIGHTNING_GDI_SUBMIT || state.transfer==nullptr) return false;
    LightningGdiCommand command{};
    // The fixture owns the complete transfer allocation; native has copied its
    // checked header before issuing this synchronous register operation.
    memcpy(&command,state.transfer,sizeof(command));
    state.last_flags=command.flags;
    if (command.opcode!=LIGHTNING_GDI_BIND && command.opcode!=LIGHTNING_GDI_UPLOAD &&
        command.opcode!=LIGHTNING_GDI_EXTERNAL_IMAGE) return false;
    if (command.opcode==LIGHTNING_GDI_UPLOAD || command.opcode==LIGHTNING_GDI_EXTERNAL_IMAGE) state.revision=command.sequence;
    if (command.opcode==LIGHTNING_GDI_EXTERNAL_IMAGE) memcpy(&state.external,
        static_cast<uint8_t*>(state.transfer)+LIGHTNING_GDI_COMMAND_BYTES,sizeof(state.external));
    state.epoch=command.epoch;
    state.generation=command.mode_generation;
    state.accepted=command.sequence;
    state.flags=LIGHTNING_GDI_FLAG_READY;
    state.status=LIGHTNING_GDI_ACCEPTED;
    return true;
}

class Owner final {
public:
    Owner() noexcept {
        registers.transfer=calloc(1,LIGHTNING_GDI_TRANSFER_BYTES);
        view={&registers,read_register,write_register,registers.transfer,
            LIGHTNING_GDI_TRANSFER_BYTES,staging,sizeof(staging),16,4,4,1};
    }
    ~Owner() {free(registers.transfer);}
    Owner(const Owner&)=delete;
    Owner& operator=(const Owner&)=delete;
    bool bind() noexcept {
        SubmitRequest request{};
        request.size=sizeof(request);
        request.version=native_abi_version;
        auto& command=request.command;
        command.magic=LIGHTNING_GDI_MAGIC;
        command.version=LIGHTNING_GDI_VERSION;
        command.opcode=LIGHTNING_GDI_BIND;
        command.mode_generation=view.mode_generation;
        command.destination={0,0,view.width,view.height};
        command.metadata_bytes=LIGHTNING_GDI_COMMAND_BYTES;
        Receipt receipt{};
        canvas.submit(view,rx7900::ByteView(reinterpret_cast<const uint8_t*>(&request),sizeof(request)),receipt);
        return receipt.outcome==accepted && receipt.accepted_sequence==1 && receipt.epoch==1;
    }
    bool baseline() noexcept {
        if (!bind()) return false;
        registers.completed=registers.accepted;
        SubmitRequest request{};
        request.size=sizeof(request);
        request.version=native_abi_version;
        request.command.magic=LIGHTNING_GDI_MAGIC;
        request.command.version=LIGHTNING_GDI_VERSION;
        request.command.opcode=LIGHTNING_GDI_UPLOAD;
        request.command.mode_generation=view.mode_generation;
        request.command.destination=request.command.batch={0,0,4,4};
        request.command.metadata_bytes=LIGHTNING_GDI_COMMAND_BYTES;
        request.command.pixel_bytes=64;
        request.command.flags=LIGHTNING_GDI_UPLOAD_BEGIN|LIGHTNING_GDI_UPLOAD_END;
        Receipt result{};
        canvas.submit(view,rx7900::ByteView(reinterpret_cast<const uint8_t*>(&request),sizeof(request)),result);
        registers.completed=registers.accepted;
        return result.outcome==accepted && result.accepted_sequence==2;
    }
    void external_submit(Receipt& result,uint32_t version=native_abi_version,
                         uint32_t opcode=LIGHTNING_GDI_EXTERNAL_IMAGE,uint32_t flags=0) noexcept {
        struct ExternalRequest final {SubmitRequest request; LightningGdiExternalImage external;};
        static_assert(sizeof(ExternalRequest)==sizeof(SubmitRequest)+LIGHTNING_GDI_EXTERNAL_IMAGE_BYTES,
            "Exact descriptor immediately follows the command");
        ExternalRequest input{};
        input.request.size=sizeof(input);
        input.request.version=version;
        auto& command=input.request.command;
        command.magic=LIGHTNING_GDI_MAGIC;
        command.version=LIGHTNING_GDI_VERSION;
        command.opcode=opcode;
        command.flags=flags;
        command.mode_generation=view.mode_generation;
        command.destination={0,0,4,4};
        command.metadata_bytes=LIGHTNING_GDI_COMMAND_BYTES+LIGHTNING_GDI_EXTERNAL_IMAGE_BYTES;
        input.external={LIGHTNING_GDI_EXTERNAL_IMAGE_BYTES,LIGHTNING_GDI_EXTERNAL_IMAGE_VERSION,
            11,22,33,4,4,16,0};
        canvas.submit(view,rx7900::ByteView(reinterpret_cast<const uint8_t*>(&input),sizeof(input)),result);
    }
    NativeCanvas canvas{};
    Registers registers{};
    NativeView view{};
private:
    uint32_t staging[16]{};
};
}

extern "C" ULONGLONG NTAPI KeQueryInterruptTime() {return ++clock_value;}
extern "C" NTSTATUS NTAPI KeDelayExecutionThread(KPROCESSOR_MODE,BOOLEAN,PLARGE_INTEGER) {
    ++delays;
    clock_value+=10000;
    return STATUS_SUCCESS;
}
extern "C" VOID NTAPI VideoPortStallExecution(ULONG microseconds) {
    ++stalls;
    clock_value+=static_cast<ULONGLONG>(microseconds)*10;
}
extern "C" VOID NTAPI VideoPortMoveMemory(PVOID destination,PVOID source,ULONG bytes) {
    memmove(destination,source,bytes);
}

int main() {
    Owner owner;
    check(owner.registers.transfer!=nullptr,"real bounded transfer allocation");
    if (owner.registers.transfer==nullptr) return 1;
    Information information{};
    check(owner.canvas.query(owner.view,information,true) && owner.registers.reads==6 &&
        information.capabilities==LIGHTNING_GDI_CAPABILITIES,"cold coherence query validates the fixed contract");
    owner.registers.reads=0;
    check(owner.canvas.query(owner.view,information,true) && owner.registers.reads==2,
        "warm unbound coherence query still reads live capabilities and reset generation");
    check(owner.bind(),"actual native bind establishes an uncompleted ledger");
    owner.registers.reads=0;
    check(owner.canvas.query(owner.view,information,true) && owner.registers.reads==8 &&
        information.accepted_sequence==1 && information.completed_sequence==0,
        "warm active coherence query preserves all six exact ledger reads");
    owner.registers.completed=owner.registers.published=1;
    owner.registers.reads=0;
    check(owner.canvas.query(owner.view,information) && owner.registers.reads==15 &&
        information.published_sequence==1 && information.published_epoch==1 &&
        information.published_mode_generation==1,"full statistics query retains actual publication receipts");
    owner.registers.reads=0;
    check(owner.canvas.query(owner.view,information,true) && owner.registers.reads==8 &&
        information.completed_sequence==1 && information.published_sequence==0 &&
        information.published_epoch==0 && information.published_mode_generation==0,
        "coherence-only result never advertises omitted publication proof");
    owner.registers.capabilities=0;
    check(owner.canvas.query(owner.view,information,true) && information.capabilities==0,
        "cached fixed contract cannot hide capability withdrawal");
    owner.registers.capabilities=LIGHTNING_GDI_CAPABILITIES;
    check(owner.canvas.reset(owner.view),"actual reset retirement succeeds before cache reuse");
    owner.registers.reads=0;
    owner.registers.magic=0;
    check(owner.canvas.query(owner.view,information,true) && owner.registers.reads==6 &&
        information.capabilities==0,"reset invalidates fixed contract before the next fast query");

    Owner failed;
    check(failed.bind(),"second native ledger binds");
    check(failed.canvas.query(failed.view,information,true),"second fixed contract validates");
    failed.registers.status=LIGHTNING_GDI_LOST;
    check(failed.canvas.query(failed.view,information,true) && information.quarantined==1 &&
        failed.canvas.quarantined(),"fast query observes LOST and quarantines without hiding failure");
    failed.registers.reads=0;
    check(failed.canvas.query(failed.view,information,true) && failed.registers.reads>=6,
        "quarantine invalidates the previously cached fixed contract");
    Owner quick;
    check(quick.bind(),"quick completion ledger binds");
    quick.registers.completion_reads=5;
    stalls=delays=0;
    check(quick.canvas.drain(quick.view) && stalls==4 && delays==0 && !quick.canvas.quarantined(),
        "short completion observes the actual ledger without sleeping a scheduler tick");
    Owner slow;
    check(slow.bind(),"slow completion ledger binds");
    slow.registers.completion_reads=12;
    stalls=delays=0;
    check(slow.canvas.drain(slow.view) && stalls==8 && delays==3 && !slow.canvas.quarantined(),
        "slow completion yields after the bounded polling budget");
    Owner lost;
    check(lost.bind(),"never-completing ledger binds");
    stalls=delays=0;
    check(!lost.canvas.drain(lost.view) && stalls==8 && delays<=1000 && lost.canvas.quarantined(),
        "expired real completion retains quarantine rather than fabricating success");
    Owner interrupt;
    check(interrupt.bind(),"interrupt completion ledger binds");
    interrupt.view.prepare_completion=prepare_event;
    interrupt.view.wait_completion=wait_event;
    stalls=delays=0;
    check(interrupt.canvas.drain(interrupt.view) && stalls==8 && delays==0 &&
        interrupt.registers.event_prepares==2 && interrupt.registers.event_waits==1 &&
        interrupt.registers.last_wait_budget!=0 && interrupt.registers.last_wait_budget<=10000000ULL,
        "connected completion waits on its event and then verifies the real ledger");
    Owner race;
    check(race.bind(),"arm-window race ledger binds");
    race.view.prepare_completion=prepare_event;
    race.view.wait_completion=wait_event;
    race.registers.complete_on_prepare=true;
    stalls=delays=0;
    check(race.canvas.drain(race.view) && stalls==8 && delays==0 &&
        race.registers.event_prepares==1 && race.registers.event_waits==0,
        "completion during event clear is caught by the mandatory ledger recheck");
    Owner spurious;
    check(spurious.bind(),"spurious wake ledger binds");
    spurious.view.prepare_completion=prepare_event;
    spurious.view.wait_completion=wait_event;
    spurious.registers.spurious_wakes=3;
    stalls=delays=0;
    check(spurious.canvas.drain(spurious.view) && stalls==8 && delays==0 &&
        spurious.registers.event_waits==4 && spurious.registers.completed==1,
        "unrelated wakes never advance completion without an observed fence prefix");
    Owner timed_out;
    check(timed_out.bind(),"event-timeout ledger binds");
    timed_out.view.prepare_completion=prepare_event;
    timed_out.view.wait_completion=wait_event;
    timed_out.registers.event_timeout=true;
    stalls=delays=0;
    check(!timed_out.canvas.drain(timed_out.view) && stalls==8 && delays==0 &&
        timed_out.registers.event_waits==1 && timed_out.canvas.quarantined(),
        "IRQ timeout retains uncertain GPU ownership under the same absolute deadline");
    Owner failed_wait;
    check(failed_wait.bind(),"failed kernel-event ledger binds");
    failed_wait.view.prepare_completion=prepare_event;
    failed_wait.view.wait_completion=wait_event;
    failed_wait.registers.event_failure=true;
    check(!failed_wait.canvas.drain(failed_wait.view) && failed_wait.canvas.quarantined(),
        "unexpected kernel-event failure quarantines rather than fabricating completion");
    Owner external;
    external.registers.capabilities|=LIGHTNING_GDI_CAP_EXTERNAL_IMAGE;
    check(external.baseline(),"external-image fixture establishes a real native baseline ledger");
    external.view.prepare_completion=prepare_event;
    external.view.wait_completion=wait_event;
    Receipt external_receipt{};
    external.external_submit(external_receipt);
    check(external_receipt.outcome==accepted && external_receipt.accepted_sequence==3 &&
        external_receipt.completed_sequence==3 && external.registers.event_waits==1 &&
        !external.canvas.quarantined(),
        "external submit cannot release its caller lease until the actual native fence prefix completes");
    check(external.registers.external.context==11 && external.registers.external.resource==22 &&
        external.registers.external.lease==33 && external.registers.external.stride==16,
        "native transfer contains the exact validated external descriptor rather than stale bytes");
    Owner external_timeout;
    external_timeout.registers.capabilities|=LIGHTNING_GDI_CAP_EXTERNAL_IMAGE;
    check(external_timeout.baseline(),"external timeout fixture establishes a baseline");
    external_timeout.view.prepare_completion=prepare_event;
    external_timeout.view.wait_completion=wait_event;
    external_timeout.registers.event_timeout=true;
    external_timeout.external_submit(external_receipt);
    check(external_receipt.outcome==uncertain && external_receipt.accepted_sequence==3 &&
        external_receipt.completed_sequence==2 && external_timeout.canvas.quarantined(),
        "uncertain external GPU reads return quarantine and preserve incomplete source ownership");
    // async-present: a filter-authorized version-2 external image returns at
    // acceptance; its read stays outstanding for the filter's per-image pin.
    Owner asynchronous;
    asynchronous.registers.capabilities|=LIGHTNING_GDI_CAP_EXTERNAL_IMAGE;
    check(asynchronous.baseline(),"async external fixture establishes a baseline");
    asynchronous.view.prepare_completion=prepare_event;
    asynchronous.view.wait_completion=wait_event;
    const unsigned async_waits=asynchronous.registers.event_waits;
    asynchronous.external_submit(external_receipt,native_async_abi_version);
    check(external_receipt.outcome==accepted && external_receipt.result==LIGHTNING_GDI_OK &&
        external_receipt.accepted_sequence==3 && external_receipt.completed_sequence==2 &&
        asynchronous.registers.event_waits==async_waits && !asynchronous.canvas.quarantined(),
        "version-2 external image returns at acceptance with its read still outstanding");
    check(asynchronous.registers.external.context==11 && asynchronous.registers.external.resource==22 &&
        asynchronous.registers.external.lease==33,"version 2 sends the unchanged canonical descriptor");
    WaitRequest async_wait{sizeof(async_wait),native_abi_version,asynchronous.view.mode_generation,3};
    Receipt async_completion{};
    asynchronous.canvas.wait(asynchronous.view,async_wait,async_completion);
    check(async_completion.outcome==accepted && async_completion.completed_sequence==3 &&
        asynchronous.registers.event_waits==async_waits+1,
        "the filter's later wait observes the read's actual ordered completion");
    Owner wrong_opcode;
    check(wrong_opcode.baseline(),"version-2 opcode fixture establishes a baseline");
    const uint32_t accepted_before=wrong_opcode.registers.accepted;
    SubmitRequest fill{};
    fill.size=sizeof(fill);
    fill.version=native_async_abi_version;
    fill.command.magic=LIGHTNING_GDI_MAGIC;
    fill.command.version=LIGHTNING_GDI_VERSION;
    fill.command.opcode=LIGHTNING_GDI_FILL;
    fill.command.mode_generation=wrong_opcode.view.mode_generation;
    fill.command.destination={0,0,4,4};
    fill.command.metadata_bytes=LIGHTNING_GDI_COMMAND_BYTES;
    fill.command.color=0x00ff00ffU;
    wrong_opcode.canvas.submit(wrong_opcode.view,
        rx7900::ByteView(reinterpret_cast<const uint8_t*>(&fill),sizeof(fill)),external_receipt);
    check(external_receipt.outcome==declined && wrong_opcode.registers.accepted==accepted_before &&
        !wrong_opcode.canvas.quarantined(),
        "version 2 is legal only for an external image: a valid version-2 fill is declined before any host access");
    Owner future;
    future.registers.capabilities|=LIGHTNING_GDI_CAP_EXTERNAL_IMAGE;
    check(future.baseline(),"unknown-version fixture establishes a baseline");
    future.external_submit(external_receipt,native_async_abi_version+1);
    check(external_receipt.outcome==declined && future.registers.accepted==accepted_before,
        "an unknown native submit version is declined before any host access");
    // present-coalesce: with a mailbox host only a version-2 copy is marked
    // replaceable, and a native wait for it rings FLUSH exactly once.
    Owner mailbox;
    mailbox.registers.capabilities|=LIGHTNING_GDI_CAP_EXTERNAL_IMAGE|LIGHTNING_GDI_CAP_EXTERNAL_MAILBOX;
    check(mailbox.baseline() && mailbox.canvas.query(mailbox.view,information,true) &&
        (information.capabilities&LIGHTNING_GDI_CAP_EXTERNAL_MAILBOX)!=0,"mailbox fixture validates its host contract");
    mailbox.view.prepare_completion=prepare_event;
    mailbox.view.wait_completion=wait_event;
    mailbox.external_submit(external_receipt,native_async_abi_version,LIGHTNING_GDI_EXTERNAL_IMAGE,
        LIGHTNING_GDI_EXTERNAL_REPLACEABLE);
    check(external_receipt.outcome==declined && mailbox.registers.accepted==2 && mailbox.registers.flushes==0,
        "a caller cannot set the latest-wins flag itself");
    mailbox.external_submit(external_receipt,native_async_abi_version);
    check(external_receipt.outcome==accepted && external_receipt.accepted_sequence==3 &&
        mailbox.registers.last_flags==LIGHTNING_GDI_EXTERNAL_REPLACEABLE && mailbox.registers.flushes==0,
        "a version-2 copy is replaceable on a mailbox host and returns at acceptance");
    mailbox.external_submit(external_receipt,native_async_abi_version);
    WaitRequest earlier_wait{sizeof(earlier_wait),native_abi_version,mailbox.view.mode_generation,3};
    mailbox.canvas.wait(mailbox.view,earlier_wait,async_completion);
    check(external_receipt.accepted_sequence==4 && async_completion.outcome==accepted && mailbox.registers.flushes==0,
        "waiting for an incomplete copy before the held one never releases the held copy");
    mailbox.external_submit(external_receipt,native_async_abi_version);
    WaitRequest held_wait{sizeof(held_wait),native_abi_version,mailbox.view.mode_generation,5};
    mailbox.canvas.wait(mailbox.view,held_wait,async_completion);
    check(async_completion.outcome==accepted && async_completion.completed_sequence==5 &&
        mailbox.registers.flushes==1 && !mailbox.canvas.quarantined(),
        "a wait covering the held copy rings FLUSH once, then observes its real completion");
    mailbox.external_submit(external_receipt);
    check(external_receipt.outcome==accepted && mailbox.registers.last_flags==0 && mailbox.registers.flushes==1,
        "a synchronous version-1 copy is never replaceable and its wait needs no FLUSH");
    mailbox.external_submit(external_receipt,native_async_abi_version);
    const uint32_t replaceable=external_receipt.accepted_sequence;
    mailbox.external_submit(external_receipt);
    WaitRequest drained{sizeof(drained),native_abi_version,mailbox.view.mode_generation,replaceable};
    mailbox.canvas.wait(mailbox.view,drained,async_completion);
    check(async_completion.outcome==accepted && mailbox.registers.flushes==1,
        "a later accepted command already makes the copy older; no FLUSH is needed");
    Owner plain_host;
    plain_host.registers.capabilities|=LIGHTNING_GDI_CAP_EXTERNAL_IMAGE;
    check(plain_host.baseline() && plain_host.canvas.query(plain_host.view,information,true),
        "non-mailbox fixture validates its host contract");
    plain_host.view.prepare_completion=prepare_event;
    plain_host.view.wait_completion=wait_event;
    plain_host.external_submit(external_receipt,native_async_abi_version);
    WaitRequest plain_wait{sizeof(plain_wait),native_abi_version,plain_host.view.mode_generation,3};
    plain_host.canvas.wait(plain_host.view,plain_wait,async_completion);
    check(external_receipt.outcome==accepted && plain_host.registers.last_flags==0 &&
        async_completion.outcome==accepted && plain_host.registers.flushes==0,
        "an older host never receives the flag or the FLUSH doorbell");
    // LZDoom exit regression: a mode change resets the host canvas (its epoch
    // reads zero again) and the next BIND must not reuse the old canvas epoch,
    // or an async image receipt of the old canvas looks like a future read.
    Owner remode;
    check(remode.baseline(),"mode-change fixture binds its first canvas");
    remode.external_submit(external_receipt,native_async_abi_version);
    const uint32_t old_epoch=external_receipt.epoch;
    const uint32_t old_sequence=external_receipt.accepted_sequence;
    remode.registers.completed=remode.registers.accepted;
    check(old_epoch==1 && old_sequence==3 && remode.canvas.reset(remode.view) && remode.registers.epoch==0,
        "the host epoch reads zero after the mode-change reset");
    ++remode.view.mode_generation;
    SubmitRequest rebind{};
    rebind.size=sizeof(rebind);
    rebind.version=native_abi_version;
    rebind.command.magic=LIGHTNING_GDI_MAGIC;
    rebind.command.version=LIGHTNING_GDI_VERSION;
    rebind.command.opcode=LIGHTNING_GDI_BIND;
    rebind.command.mode_generation=remode.view.mode_generation;
    rebind.command.destination={0,0,remode.view.width,remode.view.height};
    rebind.command.metadata_bytes=LIGHTNING_GDI_COMMAND_BYTES;
    Receipt rebound{};
    remode.canvas.submit(remode.view,rx7900::ByteView(reinterpret_cast<const uint8_t*>(&rebind),sizeof(rebind)),
        rebound);
    check(rebound.outcome==accepted && rebound.accepted_sequence==1 && rebound.epoch==old_epoch+1 &&
        remode.registers.epoch==old_epoch+1, "a BIND after a reset takes an epoch above every earlier canvas");
    check(remode.canvas.query(remode.view,information,true) && information.epoch!=old_epoch &&
        information.accepted_sequence<old_sequence,
        "the old canvas's receipt epoch no longer matches, so it retires instead of looking like a future read");
    printf("Native GDI query: %u checks, %u failures; GPU and kernel scheduling are not simulated\n",checks,failures);
    return failures==0 ? 0 : 1;
}

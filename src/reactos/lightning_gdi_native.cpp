/*
Module: lightning.reactos.gdi_native
Purpose: Snapshot owned staging rows and acknowledge only real GPU completion.
Created: 2026-10-21
Architecture: Uses the canonical shared command/transfer policy, not Venus IO.
License: GPL-2.0-or-later.
*/
#include "video_sdk.hpp"
#include "lightning_gdi_native.hpp"
#include "lightning_gdi_lifecycle.hpp"

namespace lightning::reactos::gdi {
namespace {
namespace core = ::lightning::gdi;
constexpr uint32_t wait_attempts = 1000;
constexpr uint32_t fast_poll_attempts = 8;

bool prepare_completion(const NativeView& view,uint32_t attempt) noexcept {
    return attempt>=fast_poll_attempts && view.prepare_completion!=nullptr &&
        view.wait_completion!=nullptr && view.prepare_completion(view.context);
}

bool completion_pause(const NativeView& view,uint32_t attempt,bool armed,
                      uint64_t remaining_100ns) noexcept {
    // PASSIVE_LEVEL miniport path. A sub-millisecond fence must not acquire a
    // full scheduler-tick delay. Bound the initial polling budget to 200us;
    // genuinely slow/lost devices then yield under the existing 1s deadline.
    if (attempt<fast_poll_attempts) {
        VideoPortStallExecution(25);
        return true;
    }
    if (armed) return view.wait_completion(view.context,remaining_100ns);
    LARGE_INTEGER delay{};
    delay.QuadPart=-10000;
    return NT_SUCCESS(KeDelayExecutionThread(KernelMode,FALSE,&delay));
}

uint64_t ticket(core::Mode mode,uint32_t sequence) noexcept {
    return (static_cast<uint64_t>(mode.epoch)<<32)|sequence;
}

bool accessible(const NativeView& view) noexcept {
    return view.read!=nullptr && view.write!=nullptr && view.transfer!=nullptr &&
        view.transfer_bytes==LIGHTNING_GDI_TRANSFER_BYTES && view.staging!=nullptr &&
        view.width!=0 && view.height!=0 && view.mode_generation!=0 &&
        static_cast<uint64_t>(view.width)*LIGHTNING_GDI_PIXEL_BYTES<=view.stride &&
        static_cast<uint64_t>(view.stride)*view.height<=view.staging_bytes;
}

bool valid_stage_region(const NativeView& view,uint32_t x,uint32_t y,
                        LightningDisplayRect rectangle) noexcept {
    return rectangle.width!=0 && rectangle.height!=0 && x<view.width && y<view.height &&
        rectangle.width<=view.width-x && rectangle.height<=view.height-y;
}

} // namespace

void NativeCanvas::poison() noexcept {
    contract_validated_=false;
    canvas_.poison();
    if (transfer_.phase()!=core::LeasePhase::free) transfer_.uncertain();
}

void NativeCanvas::abandon() noexcept { poison(); }

bool NativeCanvas::quarantined() const noexcept {
    return canvas_.poisoned() || transfer_.phase()==core::LeasePhase::quarantined;
}

bool NativeCanvas::drain(const NativeView& view) noexcept {
    return canvas_.accepted_sequence()==0 && !quarantined() ? true :
        wait_sequence(view,canvas_.accepted_sequence());
}

bool NativeCanvas::reset(const NativeView& view) noexcept {
    contract_validated_=false;
    if (view.read==nullptr || view.write==nullptr) return false;
    uint32_t previous=0;
    uint32_t flags=0;
    uint32_t epoch=0;
    uint32_t generation=0;
    uint32_t accepted_sequence=0;
    uint32_t completed_sequence=0;
    if (!view.read(view.context,LIGHTNING_GDI_REG_RESET_GENERATION,previous) ||
        !view.read(view.context,LIGHTNING_GDI_REG_FLAGS,flags)) return false;
    // Fresh never-used controllers are already retired and deliberately do not
    // create a GL context merely to reset it. Local uncertainty still requires
    // our own reset and a generation change; no restart erases that evidence.
    const bool owned=has_canvas() || quarantined() || transfer_.phase()!=core::LeasePhase::free;
    const bool was_retired=(flags&LIGHTNING_GDI_FLAG_RETIRED)!=0 && !owned;
    const uint32_t ownership_token=owned && ownership_reset_generation_!=0 ? ownership_reset_generation_ : previous;
    if (!view.write(view.context,LIGHTNING_GDI_REG_COMMAND,LIGHTNING_GDI_RESET)) {
        poison(); return false;
    }
    const uint64_t begin=KeQueryInterruptTime();
    for (uint32_t attempt=0;attempt<wait_attempts;++attempt) {
        const bool armed=prepare_completion(view,attempt);
        uint32_t current=0;
        uint32_t status=0;
        if (!view.read(view.context,LIGHTNING_GDI_REG_FLAGS,flags) ||
            !view.read(view.context,LIGHTNING_GDI_REG_RESET_GENERATION,current) ||
            !view.read(view.context,LIGHTNING_GDI_REG_EPOCH,epoch) ||
            !view.read(view.context,LIGHTNING_GDI_REG_MODE_GENERATION,generation) ||
            !view.read(view.context,LIGHTNING_GDI_REG_ACCEPTED_SEQUENCE,accepted_sequence) ||
            !view.read(view.context,LIGHTNING_GDI_REG_COMPLETED_SEQUENCE,completed_sequence)) break;
        if (!view.read(view.context,LIGHTNING_GDI_REG_STATUS,status)) break;
        if (retirement_confirmed(ownership_token,was_retired,
            {current,flags,epoch,generation,accepted_sequence,completed_sequence,status})) {
            canvas_.retire_after_backend_release();
            transfer_.retire_after_backend_release();
            readback_sequence_=0;
            replaceable_sequence_=0;
            ownership_reset_generation_=0;
            return true;
        }
        const uint64_t now=KeQueryInterruptTime();
        if (now<begin || now-begin>=completion_wait_limit_100ns) break;
        if (!completion_pause(view,attempt,armed,completion_wait_limit_100ns-(now-begin))) break;
    }
    poison();
    return false;
}

bool NativeCanvas::snapshot(const NativeView& view,void* destination,uint32_t bytes) noexcept {
    if (!accessible(view) || destination==nullptr ||
        static_cast<uint64_t>(view.stride)*view.height>bytes || !drain(view) ||
        !canvas_.readback_ready()) return false;
    NativeView target=view;
    target.staging=destination;
    target.staging_bytes=bytes;
    core::StripeCursor stripes;
    if (!stripes.begin({0,0,view.width,view.height},view.width,view.height)) return false;
    while (!stripes.done()) {
        LightningDisplayRect stripe{};
        uint32_t row_bytes=0;
        uint32_t pixel_bytes=0;
        if (!stripes.current(stripe) ||
            !core::packed_transfer_size(stripe.width,stripe.height,row_bytes,pixel_bytes)) return false;
        SubmitRequest request{};
        request.size=sizeof(request);
        request.version=native_abi_version;
        request.command.magic=LIGHTNING_GDI_MAGIC;
        request.command.version=LIGHTNING_GDI_VERSION;
        request.command.opcode=LIGHTNING_GDI_READBACK;
        request.command.mode_generation=view.mode_generation;
        request.command.destination=stripe;
        request.command.pixel_bytes=pixel_bytes;
        request.command.metadata_bytes=sizeof(request.command);
        Receipt result{};
        submit(target,rx7900::ByteView(reinterpret_cast<const uint8_t*>(&request),sizeof(request)),result);
        if (result.outcome!=accepted || !stripes.advance(stripe)) return false;
    }
    return true;
}

void NativeCanvas::receipt(const NativeView& view,uint32_t outcome,uint32_t result,
                            Receipt& output) const noexcept {
    output={sizeof(Receipt),native_abi_version,outcome,result,view.mode_generation,
        canvas_.mode().epoch,canvas_.accepted_sequence(),canvas_.completed_sequence(),
        canvas_.canvas_revision()};
}

bool NativeCanvas::refresh(const NativeView& view) noexcept {
    if (!accessible(view) || quarantined()) return false;
    if (canvas_.accepted_sequence()==0) return true;
    uint32_t epoch=0;
    uint32_t generation=0;
    uint32_t accepted_sequence=0;
    uint32_t completed_sequence=0;
    uint32_t status=0;
    uint32_t revision=0;
    if (!view.read(view.context,LIGHTNING_GDI_REG_EPOCH,epoch) ||
        !view.read(view.context,LIGHTNING_GDI_REG_MODE_GENERATION,generation) ||
        !view.read(view.context,LIGHTNING_GDI_REG_ACCEPTED_SEQUENCE,accepted_sequence) ||
        !view.read(view.context,LIGHTNING_GDI_REG_COMPLETED_SEQUENCE,completed_sequence) ||
        !view.read(view.context,LIGHTNING_GDI_REG_STATUS,status) ||
        !view.read(view.context,LIGHTNING_GDI_REG_CANVAS_REVISION,revision) ||
        status==LIGHTNING_GDI_LOST || epoch!=canvas_.mode().epoch ||
        generation!=canvas_.mode().generation || accepted_sequence!=canvas_.accepted_sequence() ||
        completed_sequence>accepted_sequence || revision!=canvas_.canvas_revision()) {
        poison();
        return false;
    }
    // Host completion is an ordered prefix. Retire each exact portable ledger
    // ticket only after observing that prefix, including its actual GPU fence.
    while (canvas_.completed_sequence()<completed_sequence) {
        const uint32_t sequence=canvas_.completed_sequence()+1U;
        if (canvas_.completed(sequence,ticket(canvas_.mode(),sequence))!=core::Status::validated) {
            poison();
            return false;
        }
    }
    return true;
}

bool NativeCanvas::wait_sequence(const NativeView& view,uint32_t sequence) noexcept {
    if (sequence>canvas_.accepted_sequence()) return false;
    const uint64_t begin=KeQueryInterruptTime();
    for (uint32_t attempt=0;attempt<wait_attempts;++attempt) {
        // Clear then recheck under the same immutable epoch/mode ownership.
        // An IRQ between that read and the wait leaves the event signaled.
        const bool armed=prepare_completion(view,attempt);
        if (!refresh(view)) return false;
        if (canvas_.completed_sequence()>=sequence) return true;
        // present-coalesce: a mailbox host may hold the newest latest-wins copy
        // until its display wants a frame; this waiter needs it now. FLUSH
        // consumes no sequence, so a failed write only leaves the bounded hold.
        if (attempt==0 && replaceable_sequence_!=0 && sequence>=replaceable_sequence_ &&
            replaceable_sequence_==canvas_.accepted_sequence())
            (void)view.write(view.context,LIGHTNING_GDI_REG_COMMAND,LIGHTNING_GDI_FLUSH);
        const uint64_t now=KeQueryInterruptTime();
        if (now<begin || now-begin>=completion_wait_limit_100ns) break;
        if (!completion_pause(view,attempt,armed,completion_wait_limit_100ns-(now-begin))) break;
    }
    poison();
    return false;
}

bool NativeCanvas::query(const NativeView& view,Information& output,bool coherence_only) noexcept {
    Information candidate{};
    candidate.size=sizeof(candidate);
    candidate.version=native_abi_version;
    candidate.magic=LIGHTNING_GDI_MAGIC;
    candidate.width=view.width;
    candidate.height=view.height;
    candidate.mode_generation=view.mode_generation;
    candidate.quarantined=quarantined() ? 1U : 0U;
    if (!accessible(view)) return false;
    uint32_t magic=0;
    uint32_t version=0;
    uint32_t transfer_bytes=0;
    uint32_t maximum_pixels=0;
    if (!coherence_only || !contract_validated_) {
        if (!view.read(view.context,LIGHTNING_GDI_REG_MAGIC,magic) ||
            !view.read(view.context,LIGHTNING_GDI_REG_VERSION,version) ||
            !view.read(view.context,LIGHTNING_GDI_REG_TRANSFER_BYTES,transfer_bytes) ||
            !view.read(view.context,LIGHTNING_GDI_REG_MAX_PIXEL_BYTES,maximum_pixels)) return false;
        contract_validated_=magic==LIGHTNING_GDI_MAGIC && version==LIGHTNING_GDI_VERSION &&
            transfer_bytes==LIGHTNING_GDI_TRANSFER_BYTES && maximum_pixels==LIGHTNING_GDI_MAX_PIXEL_BYTES;
    }
    if (!view.read(view.context,LIGHTNING_GDI_REG_CAPABILITIES,candidate.capabilities) ||
        !view.read(view.context,LIGHTNING_GDI_REG_RESET_GENERATION,candidate.reset_generation)) return false;
    capabilities_=candidate.capabilities;
    if (!coherence_only &&
        (!view.read(view.context,LIGHTNING_GDI_REG_PUBLISHED_SEQUENCE,candidate.published_sequence) ||
         !view.read(view.context,LIGHTNING_GDI_REG_PUBLISHED_EPOCH,candidate.published_epoch) ||
         !view.read(view.context,LIGHTNING_GDI_REG_PUBLISHED_MODE_GENERATION,candidate.published_mode_generation))) return false;
    if (!contract_validated_) candidate.capabilities=0;
    if (canvas_.accepted_sequence()!=0 && !refresh(view)) candidate.quarantined=1;
    candidate.active=canvas_.mode().generation==view.mode_generation &&
        canvas_.publication_ready() && !quarantined() ? 1U : 0U;
    candidate.epoch=canvas_.mode().epoch;
    candidate.accepted_sequence=canvas_.accepted_sequence();
    candidate.completed_sequence=canvas_.completed_sequence();
    candidate.canvas_revision=canvas_.canvas_revision();
    output=candidate;
    return true;
}

bool NativeCanvas::copy_upload(const NativeView& view,const SubmitRequest& request) noexcept {
    if (request.command.opcode==LIGHTNING_GDI_RASTER) {
        const uint32_t bytes=request.command.pixel_bytes;
        // The typed descriptor was validated before this call. Its compact
        // bundle is assembled solely in our current owned RAM staging lease.
        // No submitted pointer/offset can select another kernel allocation.
        if (request.stage_source_x!=0 || request.stage_source_y!=0 ||
            bytes>view.staging_bytes || bytes>LIGHTNING_GDI_MAX_PIXEL_BYTES) return false;
        if (bytes!=0) VideoPortMoveMemory(
            static_cast<uint8_t*>(view.transfer)+LIGHTNING_GDI_PIXEL_OFFSET,view.staging,bytes);
        return true;
    }
    const auto& rectangle=request.command.destination;
    uint32_t row_bytes=0;
    uint32_t bytes=0;
    if (!valid_stage_region(view,request.stage_source_x,request.stage_source_y,rectangle) ||
        !core::packed_transfer_size(rectangle.width,rectangle.height,row_bytes,bytes) ||
        bytes!=request.command.pixel_bytes) return false;
    for (uint32_t row=0;row<rectangle.height;++row) {
        const size_t source=static_cast<size_t>(request.stage_source_y+row)*view.stride+
            static_cast<size_t>(request.stage_source_x)*LIGHTNING_GDI_PIXEL_BYTES;
        VideoPortMoveMemory(static_cast<uint8_t*>(view.transfer)+LIGHTNING_GDI_PIXEL_OFFSET+
            static_cast<size_t>(row)*row_bytes,static_cast<uint8_t*>(view.staging)+source,row_bytes);
    }
    return true;
}

bool NativeCanvas::copy_readback(const NativeView& view,LightningDisplayRect rectangle) noexcept {
    uint32_t row_bytes=0;
    uint32_t bytes=0;
    if (!valid_stage_region(view,rectangle.x,rectangle.y,rectangle) ||
        !core::packed_transfer_size(rectangle.width,rectangle.height,row_bytes,bytes)) return false;
    KeMemoryBarrier();
    for (uint32_t row=0;row<rectangle.height;++row) {
        const size_t destination=static_cast<size_t>(rectangle.y+row)*view.stride+
            static_cast<size_t>(rectangle.x)*LIGHTNING_GDI_PIXEL_BYTES;
        VideoPortMoveMemory(static_cast<uint8_t*>(view.staging)+destination,
            static_cast<uint8_t*>(view.transfer)+LIGHTNING_GDI_PIXEL_OFFSET+
            static_cast<size_t>(row)*row_bytes,row_bytes);
    }
    return true;
}

void NativeCanvas::submit(const NativeView& view,rx7900::ByteView input,Receipt& output) noexcept {
    receipt(view,declined,LIGHTNING_GDI_BAD_PACKET,output);
    SubmitRequest request{};
    if (!accessible(view) || input.size()<sizeof(request)) return;
    // METHOD_BUFFERED input is copied before the shared system output buffer
    // receives its receipt. No unaligned wire pointer is reinterpreted as a type.
    auto* request_bytes=reinterpret_cast<uint8_t*>(&request);
    for (size_t i=0;i<sizeof(request);++i) {
        if (!input.read_u8(i,request_bytes[i])) return;
    }
    uint32_t metadata_bytes=0;
    constexpr size_t metadata_offset=offsetof(SubmitRequest,command);
    // async-present: version 2 exists only for a filter-authorized external
    // image; it skips the final source-read wait below and nothing else.
    const bool async_external=request.version==native_async_abi_version &&
        request.command.opcode==LIGHTNING_GDI_EXTERNAL_IMAGE;
    if (request.size!=input.size() || (request.version!=native_abi_version && !async_external) ||
        !core::metadata_size(request.command.opcode,request.command.clip_count,metadata_bytes) ||
        request.command.metadata_bytes!=metadata_bytes ||
        metadata_bytes>sizeof(metadata_snapshot_) ||
        input.size()!=metadata_offset+metadata_bytes ||
        request.command.mode_generation!=view.mode_generation || request.command.epoch!=0 ||
        request.command.sequence!=0) return;
    for (uint32_t index=0;index<metadata_bytes;++index) {
        if (!input.read_u8(metadata_offset+index,metadata_snapshot_[index])) return;
    }
    auto& command=command_snapshot_;
    if (core::decode_command(metadata_snapshot_,metadata_bytes,command)!=core::Status::validated) return;
    // The latest-wins flag is owned here, never by the caller.
    if (command.header.opcode==LIGHTNING_GDI_EXTERNAL_IMAGE && command.header.flags!=0) return;
    if (quarantined() || !refresh(view)) {
        receipt(view,uncertain,LIGHTNING_GDI_GPU_FAILURE,output);
        return;
    }
    auto& candidate=bind_candidate_;
    core::CanvasState* policy=&canvas_;
    if (command.header.opcode==LIGHTNING_GDI_BIND) {
        // This unpublished candidate has no independent hardware ownership.
        // Reset it in its serialized device storage, without placing a
        // second fence ledger on the already nested display kernel stack.
        candidate=core::CanvasState{};
        if (!wait_sequence(view,canvas_.accepted_sequence()) ||
            transfer_.phase()!=core::LeasePhase::free) {
            receipt(view,uncertain,LIGHTNING_GDI_GPU_FAILURE,output);
            return;
        }
        uint32_t host_epoch=0;
        if (!view.read(view.context,LIGHTNING_GDI_REG_EPOCH,host_epoch)) {
            receipt(view,declined,LIGHTNING_GDI_STALE_MODE,output);
            return;
        }
        const uint32_t floor=host_epoch>epoch_floor_ ? host_epoch : epoch_floor_;
        if (floor==UINT32_MAX) {
            receipt(view,declined,LIGHTNING_GDI_STALE_MODE,output);
            return;
        }
        const core::Mode mode{view.mode_generation,floor+1U,view.width,view.height};
        if (candidate.configure(mode)!=core::Status::validated) {
            receipt(view,uncertain,LIGHTNING_GDI_GPU_FAILURE,output);
            return;
        }
        policy=&candidate;
    }
    command.header.epoch=policy->mode().epoch;
    command.header.sequence=policy->next_sequence();
    // present-coalesce: only a version-2 copy releases its source through the
    // ordered receipt instead of a wait below, so only it may be held or
    // superseded by a host that negotiated the mailbox (lightning-gdi.h).
    const bool replaceable=async_external && contract_validated_ &&
        (capabilities_&LIGHTNING_GDI_CAP_EXTERNAL_MAILBOX)!=0;
    if (replaceable) command.header.flags|=LIGHTNING_GDI_EXTERNAL_REPLACEABLE;
    if (command.header.opcode==LIGHTNING_GDI_READBACK) command.header.reference=policy->canvas_revision();
    if (command.header.opcode==LIGHTNING_GDI_BEGIN_GROUP) command.header.reference=command.header.sequence;
    if ((command.header.opcode==LIGHTNING_GDI_UPLOAD || command.header.opcode==LIGHTNING_GDI_BLEND_UPLOAD) &&
        (command.header.flags&LIGHTNING_GDI_UPLOAD_BEGIN)!=0) command.header.reference=command.header.sequence;
    auto valid=policy->validate(command);
    if (valid==core::Status::busy && wait_sequence(view,canvas_.accepted_sequence())) valid=policy->validate(command);
    if (valid!=core::Status::validated) {
        receipt(view,declined,static_cast<uint32_t>(valid),output);
        return;
    }
    const bool readback=command.header.opcode==LIGHTNING_GDI_READBACK;
    const bool upload=command.header.opcode==LIGHTNING_GDI_UPLOAD ||
        command.header.opcode==LIGHTNING_GDI_BLEND_UPLOAD ||
        command.header.opcode==LIGHTNING_GDI_RASTER ||
        (command.header.opcode==LIGHTNING_GDI_ROP3 &&
         command.header.flags==LIGHTNING_GDI_ROP_SOURCE_UPLOAD);
    if ((!upload && (request.stage_source_x!=0 || request.stage_source_y!=0)) ||
        !transfer_.reserve(policy->mode(),command.header.sequence,readback)) return;
    if (upload && !copy_upload(view,request)) {
        transfer_.rejected(policy->mode(),command.header.sequence);
        return;
    }
    VideoPortMoveMemory(view.transfer,&command.header,sizeof(command.header));
    const uint32_t clip_offset=command.header.metadata_bytes-
        command.header.clip_count*LIGHTNING_GDI_CLIP_BYTES;
    if (command.header.opcode==LIGHTNING_GDI_RASTER) VideoPortMoveMemory(
        static_cast<uint8_t*>(view.transfer)+LIGHTNING_GDI_COMMAND_BYTES,
        &command.raster,sizeof(command.raster));
    if (command.header.opcode==LIGHTNING_GDI_GRADIENT) VideoPortMoveMemory(
        static_cast<uint8_t*>(view.transfer)+LIGHTNING_GDI_COMMAND_BYTES,
        &command.gradient,sizeof(command.gradient));
    if (command.header.opcode==LIGHTNING_GDI_EXTERNAL_IMAGE) VideoPortMoveMemory(
        static_cast<uint8_t*>(view.transfer)+LIGHTNING_GDI_COMMAND_BYTES,
        &command.external,sizeof(command.external));
    if (command.header.clip_count!=0) VideoPortMoveMemory(
        static_cast<uint8_t*>(view.transfer)+clip_offset,command.clips,
        command.header.clip_count*sizeof(LightningDisplayRect));
    KeMemoryBarrier();
    uint32_t result=0;
    uint32_t observed_sequence=0;
    uint32_t observed_epoch=0;
    uint32_t observed_generation=0;
    // Capture the monotonic token before a first submission can possibly own
    // backend work. This also covers an uncertain BIND acceptance. A later
    // already-retired host must prove a token newer than this exact ownership.
    if (command.header.opcode==LIGHTNING_GDI_BIND &&
        (!view.read(view.context,LIGHTNING_GDI_REG_RESET_GENERATION,ownership_reset_generation_) ||
         ownership_reset_generation_==0)) {
        transfer_.rejected(policy->mode(),command.header.sequence);
        receipt(view,declined,LIGHTNING_GDI_NOT_READY,output);
        return;
    }
    if (!view.write(view.context,LIGHTNING_GDI_REG_COMMAND,LIGHTNING_GDI_SUBMIT) ||
        !view.read(view.context,LIGHTNING_GDI_REG_RESULT,result) ||
        !view.read(view.context,LIGHTNING_GDI_REG_ACCEPTED_SEQUENCE,observed_sequence) ||
        !view.read(view.context,LIGHTNING_GDI_REG_EPOCH,observed_epoch) ||
        !view.read(view.context,LIGHTNING_GDI_REG_MODE_GENERATION,observed_generation)) {
        poison();
        receipt(view,uncertain,LIGHTNING_GDI_GPU_FAILURE,output);
        return;
    }
    if (result!=LIGHTNING_GDI_OK) {
        if (observed_sequence!=canvas_.accepted_sequence() || !transfer_.rejected(policy->mode(),command.header.sequence) ||
            canvas_.batch_open()) poison();
        receipt(view,quarantined() ? uncertain : declined,result,output);
        return;
    }
    if (observed_sequence!=command.header.sequence || observed_epoch!=command.header.epoch ||
        observed_generation!=view.mode_generation ||
        policy->accepted(command,ticket(policy->mode(),command.header.sequence))!=core::Status::validated ||
        !transfer_.accepted(policy->mode(),command.header.sequence)) {
        poison();
        receipt(view,uncertain,LIGHTNING_GDI_GPU_FAILURE,output);
        return;
    }
    if (policy==&candidate) {
        canvas_=candidate;
        epoch_floor_=canvas_.mode().epoch;
        replaceable_sequence_=0;
    }
    if (replaceable) replaceable_sequence_=command.header.sequence;
    if (readback) {
        readback_sequence_=command.header.sequence;
        uint32_t acknowledged=0;
        if (!wait_sequence(view,readback_sequence_) ||
            !view.read(view.context,LIGHTNING_GDI_REG_READBACK_SEQUENCE,acknowledged) ||
            acknowledged!=readback_sequence_ || !transfer_.completed(canvas_.mode(),readback_sequence_) ||
            !copy_readback(view,command.header.destination) || !transfer_.copied(canvas_.mode(),readback_sequence_)) {
            poison();
            receipt(view,uncertain,LIGHTNING_GDI_GPU_FAILURE,output);
            return;
        }
    }
    // The upper filter pins the registered source under its per-file guard for
    // this entire synchronous IOCTL. Returning before the actual GPU source
    // read retires would open a resource-CLOSE race between two IOCTLs.
    // A version-2 request instead returns at acceptance: the filter keeps its
    // own per-image pin until the ordered completion counter passes this
    // sequence, and the receipt below reports the observed prefix only.
    if (command.header.opcode==LIGHTNING_GDI_EXTERNAL_IMAGE && !async_external &&
        !wait_sequence(view,command.header.sequence)) {
        poison();
        receipt(view,uncertain,LIGHTNING_GDI_GPU_FAILURE,output);
        return;
    }
    receipt(view,accepted,LIGHTNING_GDI_OK,output);
}

void NativeCanvas::wait(const NativeView& view,const WaitRequest& request,Receipt& output) noexcept {
    if (request.size!=sizeof(request) || request.version!=native_abi_version ||
        request.mode_generation!=view.mode_generation || request.sequence>canvas_.accepted_sequence()) {
        receipt(view,declined,LIGHTNING_GDI_BAD_PACKET,output);
        return;
    }
    const bool complete=wait_sequence(view,request.sequence);
    receipt(view,complete ? accepted : uncertain,complete ? LIGHTNING_GDI_OK : LIGHTNING_GDI_GPU_FAILURE,output);
}

} // namespace lightning::reactos::gdi

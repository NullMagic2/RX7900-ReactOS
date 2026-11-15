/*
Module: lightning.tests.gdi_client
Purpose: Exercise production GDI ownership, receipts and power-cache recovery.
Created: 2026-10-21
Architecture: Real SDK/client/core; bounded fake IO and engine bitmap boundaries.
No native PnP, GPU execution or console publication is simulated as successful.
*/
#include "framebuf.h"
#include "lightning_gdi_client.hpp"
#include "lightning_display.hpp"
#include "brush_snapshot.hpp"
#include "lightning_engine_fixture.hpp"
#include "engine_bitmap.hpp"
#include <stdio.h>
#include <string.h>

namespace {
using namespace lightning::reactos;
using namespace lightning::reactos::gdi;
namespace core=lightning::gdi;
unsigned checks=0;
unsigned failures=0;
uint64_t clock_ticks=0;

void expect(bool condition,const char* description) {
    ++checks;
    if (!condition) {++failures; printf("FAIL %s\n",description);}
}

struct Boundary final {
    core::CanvasState canvas{};
    uint32_t generation=7;
    uint32_t reset_generation=1;
    uint32_t capabilities=LIGHTNING_GDI_CAPABILITIES;
    bool offline=false;
    bool decline=false;
    bool bad_wait=false;
    unsigned bad_receipt=0;
    unsigned submissions=0;
    unsigned restores=0;
    unsigned full_queries=0;
    unsigned coherence_queries=0;
    unsigned waits=0;
    unsigned readbacks=0;
    unsigned cpu_fills=0;
    unsigned gradients=0;
    unsigned decline_gradient=0;
    bool legacy_query_input=false;
    // async-present: a current native stack answers receipt probes/queries;
    // hold_external keeps a version-2 copy accepted but not yet completed.
    bool async_stack=false;
    bool hold_external=false;
    unsigned receipt_probes=0;
    unsigned receipt_queries=0;
    uint32_t receipt_status=external_receipt_retired;
    uint32_t last_submit_version=0;
    uint32_t held_sequence=0;
    uint64_t held_ticket=0;
    uint64_t last_receipt_lease=0;
    LightningGdiCommand last_command{};
    LightningGdiRasterOperands last_raster{};
    LightningGdiGradientOperands last_gradient{};
    LightningGdiExternalImage last_external{};
    uint8_t last_payload[64]{};
    uint8_t cached_pixels[16]{};
    uint8_t staging_pixels[64]{};
    uint8_t retained_pixels[64]{};
    uint32_t staging_bytes=sizeof(staging_pixels);
    void reset() noexcept {
        canvas.retire_after_backend_release();
        ++reset_generation;
    }
} boundary;

template<typename T>
DWORD output_value(PVOID output,DWORD bytes,LPDWORD returned,const T& value) {
    if (output==nullptr || bytes!=sizeof(value)) return 1;
    memcpy(output,&value,sizeof(value));
    *returned=sizeof(value);
    return 0;
}

Receipt receipt(uint32_t outcome,uint32_t result) {
    return {sizeof(Receipt),native_abi_version,outcome,result,boundary.generation,
        boundary.canvas.mode().epoch,boundary.canvas.accepted_sequence(),
        boundary.canvas.completed_sequence(),boundary.canvas.canvas_revision()};
}

SURFOBJ bitmap(void* pixels) {
    SURFOBJ surface{};
    surface.iType=STYPE_BITMAP;
    surface.iBitmapFormat=BMF_32BPP;
    surface.sizlBitmap={2,2};
    surface.lDelta=8;
    surface.pvScan0=pixels;
    return surface;
}

PDEV device() {
    PDEV result{};
    result.hDriver=reinterpret_cast<HANDLE>(static_cast<uintptr_t>(1));
    result.ScreenWidth=2;
    result.ScreenHeight=2;
    result.ScreenDelta=8;
    result.ScreenPtr=boundary.staging_pixels;
    return result;
}

bool start(ClientCanvas& client,PDEV& pdev) {
    return client.initialize(&pdev)==Attempt::performed && client.upload_cached({0,0,2,2});
}
} // namespace

extern "C" VOID APIENTRY EngQueryPerformanceCounter(LONGLONG* value) {*value=clock_ticks;}
extern "C" VOID APIENTRY EngQueryPerformanceFrequency(LONGLONG* value) {*value=100;}
extern "C" ULONG APIENTRY CLIPOBJ_cEnumStart(CLIPOBJ*,BOOL,ULONG,ULONG,ULONG) {return 0;}
extern "C" BOOL APIENTRY CLIPOBJ_bEnum(CLIPOBJ*,ULONG,ULONG*) {return FALSE;}
extern "C" ULONG APIENTRY XLATEOBJ_iXlate(XLATEOBJ*,ULONG value) {
    return (value&0xff00U)|((value&0xffU)<<16)|((value>>16)&0xffU);
}
extern "C" BOOL APIENTRY EngBitBlt(SURFOBJ*,SURFOBJ*,SURFOBJ*,CLIPOBJ*,XLATEOBJ*,RECTL*,
    POINTL*,POINTL*,BRUSHOBJ*,POINTL*,ROP4) {++boundary.cpu_fills; return FALSE;}
extern "C" PVOID APIENTRY BRUSHOBJ_pvGetRbrush(BRUSHOBJ* brush) {return brush==nullptr ? nullptr : brush->pvRbrush;}

extern "C" DWORD APIENTRY EngDeviceIoControl(HANDLE,DWORD code,PVOID input,DWORD bytes,
    PVOID output,DWORD output_bytes,LPDWORD returned) {
    *returned=0;
    if (code==ioctl_query_cpu_scanout || code==ioctl_query_owned_staging) {
        if (code==ioctl_query_owned_staging) {
            if (input==nullptr || bytes!=sizeof(VIDEO_MEMORY)) return 1;
            VIDEO_MEMORY mapping{};
            memcpy(&mapping,input,sizeof(mapping));
            if (mapping.RequestedVirtualAddress!=boundary.staging_pixels) return 1;
        }
        CpuScanoutInformation information{};
        information.size=sizeof(information);
        information.version=cpu_scanout_abi_version;
        information.magic=LIGHTNING_DISPLAY_MAGIC;
        information.width=information.height=2;
        information.stride=8;
        information.mapped_bytes=boundary.staging_bytes;
        information.capabilities=LIGHTNING_DISPLAY_CAPABILITIES;
        information.mode_generation=boundary.generation;
        information.enabled=boundary.offline ? 0U : 1U;
        return output_value(output,output_bytes,returned,information);
    }
    if (boundary.offline) return 1;
    if (code==ioctl_query) {
        bool coherence=false;
        if (bytes!=0 || input!=nullptr) {
            QueryRequest query{};
            if (input==nullptr || bytes!=sizeof(query)) return 1;
            memcpy(&query,input,sizeof(query));
            if (query.size!=sizeof(query) || query.version!=native_abi_version ||
                query.flags!=query_coherence_only || query.reserved!=0) return 1;
            ++boundary.coherence_queries;
            coherence=!boundary.legacy_query_input;
        } else ++boundary.full_queries;
        Information information{};
        information.size=sizeof(information);
        information.version=native_abi_version;
        information.magic=LIGHTNING_GDI_MAGIC;
        information.capabilities=boundary.capabilities;
        information.active=boundary.canvas.publication_ready() ? 1U : 0U;
        information.width=information.height=2;
        information.mode_generation=boundary.generation;
        information.epoch=boundary.canvas.mode().epoch;
        information.accepted_sequence=boundary.canvas.accepted_sequence();
        information.completed_sequence=boundary.canvas.completed_sequence();
        information.canvas_revision=boundary.canvas.canvas_revision();
        information.reset_generation=boundary.reset_generation;
        if (!coherence && boundary.canvas.publication_ready()) {
            information.published_sequence=boundary.canvas.canvas_revision();
            information.published_epoch=boundary.canvas.mode().epoch;
            information.published_mode_generation=boundary.generation;
        }
        return output_value(output,output_bytes,returned,information);
    }
    if (code==ioctl_wait_cpu_scanout) return 0;
    if (code==ioctl_restore) {
        RestoreRequest request{};
        if (input==nullptr || bytes!=sizeof(request)) return 1;
        memcpy(&request,input,sizeof(request));
        if (request.mode_generation!=boundary.generation || boundary.canvas.accepted_sequence()!=0) return 1;
        memcpy(boundary.staging_pixels,boundary.retained_pixels,sizeof(boundary.staging_pixels));
        ++boundary.restores;
        return 0;
    }
    if (code==ioctl_external_receipt) {
        ExternalReceiptRequest request{};
        if (!boundary.async_stack || input==nullptr || bytes!=sizeof(request)) return 1;
        memcpy(&request,input,sizeof(request));
        if (request.size!=sizeof(request) || request.version!=native_abi_version) return 1;
        request.lease==0 ? ++boundary.receipt_probes : ++boundary.receipt_queries;
        boundary.last_receipt_lease=request.lease;
        const ExternalReceipt reply{sizeof(reply),native_abi_version,
            request.lease==0 ? external_receipt_retired : boundary.receipt_status,external_feature_async};
        return output_value(output,output_bytes,returned,reply);
    }
    if (code==ioctl_wait) {
        ++boundary.waits;
        if (boundary.held_sequence!=0) {
            // The native wait observes the held read's actual completion.
            boundary.canvas.completed(boundary.held_sequence,boundary.held_ticket);
            boundary.held_sequence=0;
        }
        auto result=receipt(accepted,LIGHTNING_GDI_OK);
        if (boundary.bad_wait) ++result.completed_sequence;
        return output_value(output,output_bytes,returned,result);
    }
    if (code!=ioctl_submit || input==nullptr || bytes<sizeof(SubmitRequest)) return 1;
    ++boundary.submissions;
    SubmitRequest request{};
    memcpy(&request,input,sizeof(request));
    boundary.last_submit_version=request.version;
    if (request.version!=native_abi_version &&
        (request.version!=native_async_abi_version || request.command.opcode!=LIGHTNING_GDI_EXTERNAL_IMAGE)) return 1;
    if (request.command.mode_generation!=boundary.generation) return 1;
    if (boundary.decline) return output_value(output,output_bytes,returned,receipt(declined,LIGHTNING_GDI_NOT_READY));
    core::Command command{};
    constexpr uint32_t prefix=sizeof(SubmitRequest)-sizeof(LightningGdiCommand);
    if (request.size!=bytes || bytes<prefix ||
        core::decode_command(static_cast<const uint8_t*>(input)+prefix,bytes-prefix,command)!=core::Status::validated) return 1;
    if (command.header.opcode==LIGHTNING_GDI_BIND) {
        if (boundary.canvas.configure({boundary.generation,1,2,2})!=core::Status::validated) return 1;
    }
    command.header.epoch=boundary.canvas.mode().epoch;
    command.header.sequence=boundary.canvas.next_sequence();
    boundary.last_command=command.header;
    if (command.header.opcode==LIGHTNING_GDI_READBACK) command.header.reference=boundary.canvas.canvas_revision();
    if (command.header.opcode==LIGHTNING_GDI_BEGIN_GROUP) command.header.reference=command.header.sequence;
    if (command.header.opcode==LIGHTNING_GDI_UPLOAD && (command.header.flags&LIGHTNING_GDI_UPLOAD_BEGIN)!=0)
        command.header.reference=command.header.sequence;
    if (command.header.opcode==LIGHTNING_GDI_RASTER) boundary.last_raster=command.raster;
    if (command.header.opcode==LIGHTNING_GDI_EXTERNAL_IMAGE) {
        // Version 2 carries the producer point in context/resource on input.
        if ((request.version==native_abi_version &&
             (command.external.context!=0 || command.external.resource!=0)) || command.external.lease==0) return 1;
        boundary.last_external=command.external;
        // A CPU-only stand-in for the separately tested native filter lease
        // lookup. Production code alone validates current process ownership.
        command.external.context=UINT64_C(0x5544330000000001);
        command.external.resource=UINT64_C(0x5544330000000002);
    }
    if (command.header.opcode==LIGHTNING_GDI_GRADIENT) {
        ++boundary.gradients;
        if (boundary.decline_gradient==boundary.gradients)
            return output_value(output,output_bytes,returned,receipt(declined,LIGHTNING_GDI_NOT_READY));
        boundary.last_gradient=command.gradient;
    }
    if (command.header.opcode==LIGHTNING_GDI_READBACK) ++boundary.readbacks;
    if (command.header.opcode==LIGHTNING_GDI_RASTER && command.header.pixel_bytes<=sizeof(boundary.last_payload))
        memcpy(boundary.last_payload,boundary.staging_pixels,command.header.pixel_bytes);
    const uint64_t ticket=(uint64_t{command.header.epoch}<<32)|command.header.sequence;
    const bool hold=boundary.hold_external && request.version==native_async_abi_version;
    if (boundary.canvas.accepted(command,ticket)!=core::Status::validated ||
        (!hold && boundary.canvas.completed(command.header.sequence,ticket)!=core::Status::validated)) return 1;
    if (hold) {
        boundary.held_sequence=command.header.sequence;
        boundary.held_ticket=ticket;
    }
    auto result=receipt(accepted,LIGHTNING_GDI_OK);
    if (boundary.bad_receipt==1) ++result.epoch;
    if (boundary.bad_receipt==2) ++result.accepted_sequence;
    if (boundary.bad_receipt==3) result.completed_sequence=result.accepted_sequence+1;
    if (boundary.bad_receipt==4) ++result.canvas_revision;
    if (boundary.bad_receipt==5) result.outcome=uncertain;
    if (boundary.bad_receipt==6 && result.completed_sequence!=0) --result.completed_sequence;
    return output_value(output,output_bytes,returned,result);
}

int main() {
    {
        uint32_t pixels[16]{};
        for (uint32_t index=0;index<16;++index) pixels[index]=0x100U+index;
        SURFOBJ surface=bitmap(pixels);
        surface.sizlBitmap={4,4};
        surface.lDelta=16;
        surface.cjBits=sizeof(pixels);
        CLIPOBJ complex{};
        complex.iDComplexity=DC_COMPLEX;
        EngineBitmap snapshot;
        SURFOBJ* selected=&surface;
        RECTL target{1,1,4,4};
        POINTL origin{0,0};
        expect(snapshot.preserve_clipped_copy(&surface,selected,&complex,target,origin) &&
            selected!=&surface && selected->sizlBitmap.cx==3 && selected->sizlBitmap.cy==3 &&
            snapshot.bytes()==36 && origin.x==0 && origin.y==0,
            "complex overlapping CopyBits fallback snapshots only its immutable source ROI");
        pixels[0]=0xabcdef;
        uint32_t retained=0;
        memcpy(&retained,selected->pvScan0,sizeof(retained));
        expect(retained==0x100,"earlier clipped writes cannot change the retained source for later clips");
        EngineBitmap disjoint;
        selected=&surface;target={2,2,4,4};origin={0,0};
        expect(disjoint.preserve_clipped_copy(&surface,selected,&complex,target,origin) && selected==&surface &&
            disjoint.surface()==nullptr,"disjoint source and destination need no snapshot allocation");
        EngineBitmap simple;
        complex.iDComplexity=DC_RECT;
        selected=&surface;target={1,1,4,4};origin={0,0};
        expect(simple.preserve_clipped_copy(&surface,selected,&complex,target,origin) && selected==&surface,
            "single-rectangle memmove keeps the existing zero-snapshot engine path");
        complex.iDComplexity=DC_COMPLEX;
        EngineBitmap flipped;
        selected=&surface;target={1,4,4,1};origin={0,0};
        expect(flipped.preserve_clipped_copy(&surface,selected,&complex,target,origin) && selected!=&surface &&
            target.top==4 && target.bottom==1,"snapshot preserves vertical inverted copy orientation");
        EngineBitmap cropped_flip;
        selected=&surface;target={1,4,4,1};origin={2,2};
        expect(cropped_flip.preserve_clipped_copy(&surface,selected,&complex,target,origin) && selected!=&surface &&
            selected->sizlBitmap.cx==2 && selected->sizlBitmap.cy==2 &&
            target.top==4 && target.bottom==1 && origin.x==0 && origin.y==0,
            "nonzero-source flipped copy retains engine-owned right/bottom crop geometry");
        const uint32_t expected_roi[]{0x10a,0x10b,0x10e,0x10f};
        expect(memcmp(selected->pvScan0,expected_roi,sizeof(expected_roi))==0,
            "cropped inverted operation retains exact immutable bottom-right source pixels");
        EngineBitmap cropped;
        selected=&surface;target={0,0,3,3};origin={-1,-1};
        expect(!cropped.preserve_clipped_copy(&surface,selected,&complex,target,origin) && selected==&surface &&
            cropped.surface()==nullptr &&
            target.left==0 && target.top==0 && target.right==3 && target.bottom==3 && origin.x==-1 && origin.y==-1,
            "negative source origin fails before allocation or caller geometry changes");
        EngineBitmap occupied;
        expect(occupied.create(1,1,BMF_32BPP,sizeof(uint32_t)),"occupy final bounded engine fixture slot");
        EngineBitmap exhausted;
        selected=&surface;target={1,1,4,4};origin={0,0};
        const RECTL before=target;
        expect(!exhausted.preserve_clipped_copy(&surface,selected,&complex,target,origin) &&
            selected==&surface && target.left==before.left && target.top==before.top && origin.x==0 && origin.y==0,
            "failed snapshot allocation leaves the original destination and source untouched");
    }
    {
        boundary={};
        PDEV stale=device();
        stale.ScreenPtr=boundary.cached_pixels;
        ClientCanvas client;
        expect(client.initialize(&stale)==Attempt::failed && boundary.submissions==0,
            "same-sized stale staging cannot adopt current inventory and submit a replacement BIND");
    }
    {
        boundary={};
        PDEV recovery_pdev=device();
        ClientCanvas recovery;
        expect(start(recovery,recovery_pdev),"mode retry fixture starts from actual client BIND ledger");
        boundary.bad_wait=true;
        expect(!recovery.retire() && recovery.poisoned(),"failed native completion retains uncertain local ownership");
        const unsigned submitted=boundary.submissions;
        expect(recovery.reinitialize_after_mode(&recovery_pdev)==Attempt::failed &&
            boundary.submissions==submitted,"same mode inventory cannot discard a poisoned canvas");
        ++boundary.generation;
        expect(recovery.reinitialize_after_mode(&recovery_pdev)==Attempt::failed,
            "new mode token without actual native canvas retirement is insufficient");
        boundary.reset();
        boundary.bad_wait=false;
        expect(recovery.reinitialize_after_mode(&recovery_pdev)==Attempt::performed && !recovery.poisoned(),
            "strictly newer native retirement permits failed-bind owner to reinitialize");
        expect(recovery.upload_cached({0,0,2,2}) && recovery.retire(),
            "recovered mode establishes the retained full CPU baseline before ordinary use");
        recovery.release_cpu_storage();
    }
    {
        // DrvAssertMode(TRUE) degradation after an accepted native mode commit.
        boundary={};
        PDEV demotion_pdev=device();
        ClientCanvas demoted;
        expect(start(demoted,demotion_pdev),"demotion fixture starts from an actual client BIND ledger");
        boundary.bad_wait=true;
        expect(!demoted.retire() && demoted.poisoned(),"demotion fixture holds a poisoned canvas");
        boundary.bad_wait=false;
        const unsigned submitted=boundary.submissions;
        const unsigned queried=boundary.full_queries+boundary.coherence_queries;
        demoted.demote_to_cpu();
        clock_ticks+=1000;
        expect(!demoted.enabled() && !demoted.poisoned() && demoted.prepare(nullptr,nullptr) &&
            demoted.prepare(nullptr,nullptr),"demoted canvas lets the CPU publish the retained cache");
        expect(boundary.submissions==submitted && boundary.full_queries+boundary.coherence_queries==queried,
            "demotion neither replays nor promotes: no native GPU request follows");
        expect(!demoted.upload_cached({0,0,2,2}) && boundary.submissions==submitted,
            "a demoted canvas accepts no upload");
        boundary.reset();
        ++boundary.generation;
        expect(demoted.reinitialize_after_mode(&demotion_pdev)==Attempt::performed && demoted.enabled(),
            "the next actual mode reinitialization may bind a fresh canvas");
        expect(demoted.retire(),"rebound canvas retires");
        demoted.release_cpu_storage();
    }

    {
        boundary={};
        PDEV external_pdev=device();
        ClientCanvas external_client;
        ClipBatch clip{};
        clip.bounds={0,0,2,2};
        const LightningGdiExternalImage image{sizeof(image),LIGHTNING_GDI_EXTERNAL_IMAGE_VERSION,
            0,0,UINT64_C(0xaabbccdd00000003),2,2,8,0};
        expect(start(external_client,external_pdev),"external fixture initializes ordinary canvas");
        const unsigned submitted=boundary.submissions;
        expect(external_client.external_image(clip,0,0,image)==Attempt::unsupported &&
            boundary.submissions==submitted,"base host never receives optional external-image operation");
        expect(external_client.retire(),"external fixture retires before changing capabilities");
        boundary={};
        boundary.capabilities|=LIGHTNING_GDI_CAP_EXTERNAL_IMAGE;
        ClientCanvas imported;
        expect(start(imported,external_pdev) && imported.external_image(clip,0,0,image)==Attempt::performed &&
            boundary.last_external.lease==image.lease && boundary.last_external.context==0 &&
            boundary.last_external.resource==0 && boundary.last_command.pixel_bytes==0,
            "client preserves64-bit opaque lease and lets native owner resolve identity without pixel upload");
        auto forged=image;
        forged.context=1;
        const unsigned accepted_submissions=boundary.submissions;
        expect(imported.external_image(clip,0,0,forged)==Attempt::unsupported &&
            boundary.submissions==accepted_submissions,"client cannot inject an external native context");
        boundary.bad_receipt=6;
        expect(imported.external_image(clip,0,0,image)==Attempt::failed && imported.poisoned(),
            "external source reuse requires completed read receipt even after accepted submission");
        imported.release_cpu_storage();
    }

    {
        // async-present: revision-2 presents through the production client.
        boundary={};
        boundary.capabilities|=LIGHTNING_GDI_CAP_EXTERNAL_IMAGE;
        PDEV async_pdev=device();
        ClientCanvas legacy;
        ClipBatch clip{};
        clip.bounds={0,0,2,2};
        const LightningGdiExternalImage image{sizeof(image),LIGHTNING_GDI_EXTERNAL_IMAGE_VERSION,
            0,0,UINT64_C(0xaabbccdd00000005),2,2,8,0};
        const ClientCanvas::ExternalOptions named{true,UINT64_C(0x300000007),1};
        bool pending=true;
        expect(start(legacy,async_pdev),"async fixture initializes an ordinary external-image canvas");
        unsigned submitted=boundary.submissions;
        expect(!legacy.async_external() && legacy.external_image(clip,0,0,image,named,pending)==Attempt::unsupported &&
            !pending && boundary.submissions==submitted,
            "an older native stack never receives a version-2 request and keeps version 1");
        boundary.async_stack=true;
        expect(legacy.async_external() && boundary.receipt_probes==1,
            "an unsupported probe is not cached: a later current stack is still detected");
        expect(legacy.retire(),"legacy async fixture retires");

        boundary={};
        boundary.capabilities|=LIGHTNING_GDI_CAP_EXTERNAL_IMAGE;
        boundary.async_stack=true;
        ClientCanvas async;
        expect(start(async,async_pdev) && async.async_external() && async.async_external() &&
            boundary.receipt_probes==1,"native async support is probed once and then cached");
        boundary.hold_external=true;
        submitted=boundary.submissions;
        expect(async.external_image(clip,0,0,image,named,pending)==Attempt::performed && pending &&
            boundary.submissions==submitted+1 && boundary.last_submit_version==native_async_abi_version &&
            boundary.last_external.lease==image.lease && boundary.last_external.resource==named.producer_sync &&
            boundary.last_external.context==named.producer_value && boundary.held_sequence!=0 &&
            !async.poisoned(),
            "accepted asynchronous copy returns before its read retires and names its producer point");
        uint32_t status=external_receipt_failed;
        boundary.receipt_status=external_receipt_pending;
        const unsigned waits_before_query=boundary.waits;
        expect(async.external_receipt(image.lease,status) && status==external_receipt_pending &&
            boundary.last_receipt_lease==image.lease && boundary.waits==waits_before_query,
            "receipt query reports the outstanding read without any GPU wait");
        // The host completes the held read; the native pin is then released.
        expect(boundary.canvas.completed(boundary.held_sequence,boundary.held_ticket)==core::Status::validated,
            "fixture host completes the outstanding read in order");
        boundary.held_sequence=0;
        boundary.receipt_status=external_receipt_retired;
        expect(async.external_receipt(image.lease,status) && status==external_receipt_retired,
            "receipt query reports retirement once the native pin is released");
        expect(!async.external_receipt(0,status) && status==external_receipt_failed,
            "a zero lease is never a receipt query");
        // lightning-overlap: the filter refuses the receipt query while a PnP or
        // power transition holds the stack (DEVICE_NOT_READY, DELETE_PENDING).
        // The caller must keep the image: failure, never a fabricated retirement.
        boundary.offline=true;
        expect(!async.external_receipt(image.lease,status) && status==external_receipt_failed,
            "a receipt query refused by a transitioning stack reports failure, never retirement");
        boundary.offline=false;
        expect(async.external_receipt(image.lease,status) && status==external_receipt_retired,
            "after the transition the same lease is answered again");
        const ClientCanvas::ExternalOptions synchronous{false,UINT64_C(0x300000007),1};
        const unsigned waits=boundary.waits;
        expect(async.external_image(clip,0,0,image,synchronous,pending)==Attempt::performed && !pending &&
            boundary.waits==waits+1 && boundary.held_sequence==0 && !async.poisoned(),
            "a producer-only revision-2 copy waits for its own read without the native guard");
        boundary.hold_external=false;
        expect(async.external_image(clip,0,0,image,named,pending)==Attempt::performed && !pending,
            "an already completed asynchronous read is reported retired at once");
        const ClientCanvas::ExternalOptions half{true,UINT64_C(0x300000007),0};
        submitted=boundary.submissions;
        expect(async.external_image(clip,0,0,image,half,pending)==Attempt::unsupported &&
            boundary.submissions==submitted,"a producer token without its signaled value is refused");
        auto forged=image;
        forged.resource=9;
        expect(async.external_image(clip,0,0,forged,named,pending)==Attempt::unsupported &&
            boundary.submissions==submitted,"callers still cannot name a native identity directly");
        expect(async.external_image(clip,0,0,image)==Attempt::performed &&
            boundary.last_submit_version==native_abi_version,"version 1 keeps its synchronous request");
        boundary.hold_external=true;
        expect(async.external_image(clip,0,0,image)==Attempt::performed && boundary.held_sequence==0,
            "version 1 is never held asynchronously by the native stack");
        boundary.hold_external=false;
        expect(async.retire(),"async fixture retires after its reads completed");
        async.release_cpu_storage();
        legacy.release_cpu_storage();
    }
    PDEV pdev=device();
    boundary={};
    ClientCanvas client;
    SURFOBJ cached=bitmap(boundary.cached_pixels);
    SURFOBJ staging=bitmap(boundary.staging_pixels);
    expect(start(client,pdev),"production client establishes a real canonical BIND/baseline ledger");
    expect(boundary.full_queries==1 && boundary.coherence_queries==0,
        "initialization still performs the complete native protocol query");
    const unsigned submissions=boundary.submissions;
    expect(client.initialize(&pdev)==Attempt::failed && boundary.submissions==submissions,
        "reinitialize cannot discard a live same-generation owner");
    boundary.offline=true;
    expect(!client.prepare(&cached,&staging) && !client.poisoned(),"confirmed D3 defers without inventing GPU uncertainty");
    boundary.offline=false;
    boundary.reset();
    memset(boundary.cached_pixels,0x55,sizeof(boundary.cached_pixels));
    memset(boundary.retained_pixels,0x22,sizeof(boundary.retained_pixels));
    expect(client.prepare(&cached,&staging) && boundary.restores==0 && boundary.staging_pixels[0]==0x55,
        "D0 rebind preserves known CPU writes instead of overwriting them with older native snapshot");
    expect(boundary.full_queries==1 && boundary.coherence_queries==1,
        "live preparation uses typed coherence query without losing reset recovery");
    boundary.legacy_query_input=true;
    const unsigned legacy_queries=boundary.coherence_queries;
    expect(client.prepare(&cached,&staging) && boundary.coherence_queries==legacy_queries+1 && !client.poisoned(),
        "older miniport may ignore optional query input and return full owner information");
    boundary.legacy_query_input=false;
    ClipBatch full{};
    full.bounds={0,0,2,2};
    expect(client.copy(full,0,0)==Attempt::performed,"accepted GPU copy marks cache stale through production path");
    boundary.reset();
    memset(boundary.retained_pixels,0x33,sizeof(boundary.retained_pixels));
    expect(client.prepare(&cached,&staging) && boundary.restores==1 && boundary.cached_pixels[0]==0x33,
        "stale cache uses acknowledged native snapshot before fresh full baseline");
    boundary.decline=true;
    expect(client.fill(full,0,nullptr,nullptr)==Attempt::unsupported && !client.poisoned(),
        "unchanged declined receipt permits synchronized CPU fallback");
    boundary.decline=false;
    expect(client.raster(full,0x55,UINT32_MAX,nullptr,nullptr,{0,0,2,2},nullptr,false,&staging)==Attempt::performed &&
        boundary.last_command.flags==LIGHTNING_GDI_ROP_SOURCE_NONE && boundary.last_command.color==0,
        "source-independent ROP3 uses no invented source or unused pattern operand");
    POINTL source_point{0,0};
    expect(client.raster(full,0x66,0,&cached,nullptr,{0,0,2,2},&source_point,true,&staging)==Attempt::performed &&
        boundary.last_command.flags==LIGHTNING_GDI_ROP_SOURCE_SCREEN,
        "same-canvas Boolean source uses bounded GPU screen snapshot");
    expect(client.raster(full,0x66,0,&cached,nullptr,{0,0,2,2},&source_point,false,&staging)==Attempt::performed &&
        boundary.last_command.flags==LIGHTNING_GDI_ROP_SOURCE_UPLOAD && boundary.last_command.pixel_bytes==16,
        "bitmap Boolean source uses bounded normalized immutable upload");
    Statistics statistics{};
    const unsigned statistics_queries=boundary.full_queries;
    expect(client.statistics(statistics) && statistics.version==statistics_abi_version &&
        statistics.accepted_rasters==3 && boundary.full_queries==statistics_queries+1 &&
        statistics.device.published_sequence==boundary.canvas.canvas_revision(),
        "statistics retains full publication query and counts only actual acknowledged command acceptance");
    uint8_t coverage_pixels[8]{0,255,0,0,127,64,0,0};
    SURFOBJ coverage=bitmap(coverage_pixels);
    coverage.iBitmapFormat=BMF_8BPP;
    coverage.lDelta=4;
    coverage.pvBits=coverage_pixels;
    coverage.cjBits=sizeof(coverage_pixels);
    BRUSHOBJ solid{};
    solid.iSolidColor=0x12345678;
    expect(client.begin_text() && boundary.canvas.group_open(),"text scope owns a real typed logical group");
    const unsigned glyph_allocations=bitmap_creations;
    expect(client.glyph(full,&coverage,{0,0,2,2},{0,0},&solid,&staging)==Attempt::performed &&
        boundary.last_command.opcode==LIGHTNING_GDI_RASTER && boundary.last_command.pixel_bytes==8 &&
        boundary.last_raster.mask_kind==LIGHTNING_GDI_MASK_COVERAGE8 &&
        boundary.last_payload[0]==0 && boundary.last_payload[1]==255 && boundary.last_payload[4]==127,
        "actual glyph command carries normalized coverage without binary thresholding");
    expect(bitmap_creations==glyph_allocations && boundary.last_payload[2]==0 && boundary.last_payload[3]==0 &&
        boundary.last_payload[6]==0 && boundary.last_payload[7]==0,
        "bounded direct coverage staging creates no engine bitmap and initializes row padding");
    expect(!boundary.canvas.publication_ready() && boundary.canvas.readback_ready(),
        "open text group permits exact CPU readback but cannot publish partial string");
    expect(client.end_text() && !boundary.canvas.group_open() && boundary.canvas.publication_ready(),
        "observed END checkpoint closes the logical string publication");
    expect(client.statistics(statistics) && statistics.accepted_glyphs==1,
        "glyph counter grows only after actual accepted coverage command");
    uint8_t padded_coverage[16]{99,98,97,96,10,11,12,95,20,21,22,94,30,31,32,93};
    SURFOBJ cropped=coverage;
    cropped.sizlBitmap={3,3};
    cropped.pvBits=padded_coverage;
    cropped.pvScan0=padded_coverage+4;
    cropped.cjBits=sizeof(padded_coverage);
    expect(client.glyph(full,&cropped,{0,0,2,2},{1,1},&solid,&staging)==Attempt::performed &&
        boundary.last_payload[0]==21 && boundary.last_payload[1]==22 &&
        boundary.last_payload[4]==31 && boundary.last_payload[5]==32 && bitmap_creations==glyph_allocations,
        "cropped coverage honors allocation offset, source origin and pitch without normalizer allocation");
    const unsigned before_truncation=boundary.submissions;
    cropped.cjBits=14;
    expect(client.glyph(full,&cropped,{0,0,2,2},{1,1},&solid,&staging)==Attempt::failed &&
        boundary.submissions==before_truncation && !client.poisoned() && bitmap_creations==glyph_allocations,
        "truncated glyph allocation fails before a source read, GPU submission or unsafe CPU replay");
    SURFOBJ bottom_up=coverage;
    bottom_up.lDelta=-4;
    bottom_up.pvScan0=coverage_pixels+4;
    expect(client.glyph(full,&bottom_up,{0,0,2,2},{0,0},&solid,&staging)==Attempt::performed &&
        boundary.last_payload[0]==127 && boundary.last_payload[1]==64 &&
        boundary.last_payload[4]==0 && boundary.last_payload[5]==255 && bitmap_creations==glyph_allocations+1,
        "negative-pitch glyph keeps existing engine normalization rather than assuming row orientation");
    SURFOBJ alias=coverage;
    alias.pvBits=boundary.staging_pixels;
    alias.pvScan0=boundary.staging_pixels;
    boundary.staging_pixels[0]=19;
    boundary.staging_pixels[1]=23;
    boundary.staging_pixels[4]=29;
    boundary.staging_pixels[5]=31;
    expect(client.glyph(full,&alias,{0,0,2,2},{0,0},&solid,&staging)==Attempt::performed &&
        boundary.last_payload[0]==19 && boundary.last_payload[1]==23 &&
        boundary.last_payload[4]==29 && boundary.last_payload[5]==31 && bitmap_creations==glyph_allocations+2,
        "aliased coverage uses complete engine snapshot before stage padding or rows can mutate source");
    expect(client.begin_text(),"second text scope opens before power retirement");
    const uint32_t retired_group=boundary.canvas.accepted_sequence();
    boundary.reset();
    expect(client.prepare(&cached,&staging) && boundary.canvas.group_open() &&
        boundary.canvas.accepted_sequence()!=retired_group,
        "proven power retirement replaces old group token after restoring full baseline");
    expect(client.end_text() && boundary.canvas.publication_ready(),
        "post-reset END references fresh group instead of retired sequence identity");
    expect(client.begin_text(),"offline text-end fixture begins a real group");
    boundary.offline=true;
    expect(!client.end_text() && !client.poisoned(),"known D3 defers group END without inventing execution uncertainty");
    boundary.offline=false;
    boundary.reset();
    expect(client.prepare(&cached,&staging) && !boundary.canvas.group_open() && client.begin_text() && client.end_text(),
        "D0 retirement invalidates deferred group token before next logical scope");
    expect(client.begin_text(),"surviving-canvas scope begins normally");
    boundary.offline=true;
    expect(!client.end_text() && !client.poisoned(),"offline admission gate retains existing group identity");
    boundary.offline=false;
    expect(client.prepare(&cached,&staging) && !boundary.canvas.group_open(),
        "same-canvas recovery closes deferred END before later live draw");
    uint8_t mono_pixels[8]{0x80,0,0,0,0x40,0,0,0};
    SURFOBJ mono=coverage;
    mono.iBitmapFormat=BMF_1BPP;
    mono.pvScan0=mono_pixels;
    mono.pvBits=mono_pixels;
    POINTL zero{};
    RasterInput masked{};
    masked.source=&cached;
    masked.original={0,0,2,2};
    masked.source_point=&zero;
    masked.mask=&mono;
    masked.mask_point=&zero;
    masked.rop4=0xaacc;
    expect(client.raster_operands(full,masked,&staging)==Attempt::performed &&
        boundary.last_raster.source_bytes==16 && boundary.last_raster.mask_offset==16 &&
        boundary.last_command.pixel_bytes==24 && boundary.last_payload[16]==0x80,
        "masked copy uses compact independently bounded source then MSB-first mask");
    struct Pattern final {BrushSnapshot header; uint32_t pixels[4];} pattern{};
    pattern.header.size=sizeof(pattern);
    pattern.header.width=pattern.header.height=2;
    pattern.header.stride=8;
    pattern.header.pixel_bytes=16;
    pattern.pixels[0]=0x89abcdef;
    BRUSHOBJ tile{};
    tile.iSolidColor=UINT32_MAX;
    tile.pvRbrush=&pattern;
    POINTL negative{-3,-1};
    RasterInput patterned{};
    patterned.original={0,0,2,2};
    patterned.brush=&tile;
    patterned.brush_origin=&negative;
    patterned.rop4=0xf0f0;
    expect(client.raster_operands(full,patterned,&staging)==Attempt::performed &&
        boundary.last_raster.pattern_kind==LIGHTNING_GDI_PATTERN_TILE &&
        boundary.last_raster.pattern_origin_x==1 && boundary.last_raster.pattern_origin_y==1,
        "realized tile uses owned pixels and signed positive-modulo brush origin");
    RasterInput stretched{};
    stretched.source=&cached;
    stretched.same_canvas=true;
    stretched.stretch=true;
    stretched.source_rectangle={0,0,2,2};
    stretched.original={0,0,4,2};
    stretched.rop4=0xcccc;
    expect(client.raster_operands(full,stretched,&staging)==Attempt::performed &&
        boundary.last_raster.stretch_mode==LIGHTNING_GDI_STRETCH_COLORONCOLOR &&
        boundary.last_raster.original_destination.width==4 && boundary.last_raster.source_rect.width==2,
        "stretch retains original destination ratio and whole source ROI after viewport clipping");
    stretched.same_canvas=false;
    expect(client.raster_operands(full,stretched,&staging)==Attempt::performed &&
        boundary.last_raster.source_kind==LIGHTNING_GDI_ROP_SOURCE_UPLOAD && boundary.last_command.pixel_bytes==16,
        "bitmap stretch normalizes entire source ROI rather than destination-sized pixels");
    stretched.original={4,0,0,2};
    const unsigned before_mirror=boundary.submissions;
    expect(client.raster_operands(full,stretched,&staging)==Attempt::unsupported &&
        boundary.submissions==before_mirror,"signed mirror declines before accepting any GPU command");
    stretched.same_canvas=true;
    stretched.original={0,0,4,2};
    stretched.source_rectangle={-1,0,1,2};
    stretched.raster_flags=LIGHTNING_GDI_RASTER_MIRROR_X;
    expect(client.raster_operands(full,stretched,&staging)==Attempt::performed &&
        boundary.last_raster.source_rect.x==0 && boundary.last_raster.source_rect.width==2 &&
        boundary.last_raster.raster_flags==(LIGHTNING_GDI_RASTER_MIRROR_X|LIGHTNING_GDI_RASTER_GUARDED_SOURCE),
        "paired mirror restores engine-adjusted source ROI before immutable snapshot");
    stretched.same_canvas=false;
    boundary.cached_pixels[0]=0xa1;
    expect(client.raster_operands(full,stretched,&staging)==Attempt::performed &&
        boundary.last_raster.source_rect.x==1 && boundary.last_raster.source_stride==12 &&
        boundary.last_raster.source_bytes==24 && boundary.last_payload[0]==0 && boundary.last_payload[4]==0xa1 &&
        boundary.last_raster.raster_flags==(LIGHTNING_GDI_RASTER_MIRROR_X|LIGHTNING_GDI_RASTER_GUARDED_SOURCE|
            LIGHTNING_GDI_RASTER_INVALID_LEFT),"guarded mirror uploads owned initialized missing-edge bytes and logical source offset");
    {
        // DrvStretchBltROP 1bpp mask: the engine samples it at the stretched
        // source texel and draws only where the bit is clear (STRETCH_MASK).
        uint8_t mask_bits[8]{0x80,0,0,0,0,0,0,0};
        SURFOBJ stretch_mask=mono;
        stretch_mask.sizlBitmap={2,2};
        stretch_mask.lDelta=4;
        stretch_mask.pvScan0=mask_bits;
        stretch_mask.pvBits=mask_bits;
        stretch_mask.cjBits=sizeof(mask_bits);
        RasterInput masked_stretch{};
        masked_stretch.source=&cached;
        masked_stretch.stretch=true;
        masked_stretch.source_rectangle={0,0,1,2};
        masked_stretch.original={0,0,2,2};
        masked_stretch.mask=&stretch_mask;
        masked_stretch.rop4=0x33ccU;
        const unsigned before_masked=boundary.submissions;
        expect(client.raster_operands(full,masked_stretch,&staging)==Attempt::performed &&
            boundary.submissions==before_masked+1 &&
            boundary.last_raster.stretch_mode==LIGHTNING_GDI_STRETCH_COLORONCOLOR &&
            boundary.last_raster.mask_kind==LIGHTNING_GDI_MASK_MONO1 &&
            boundary.last_raster.mask_space==LIGHTNING_GDI_MASK_SPACE_DESTINATION &&
            boundary.last_raster.foreground_rop==0xcc && boundary.last_raster.background_rop==0xaa &&
            boundary.last_raster.mask_offset==8 && boundary.last_raster.mask_stride==4 &&
            boundary.last_payload[8]==0 && boundary.last_payload[12]==0xc0,
            "masked stretch submits the engine draw decision with only the ROP4 foreground");
        masked_stretch.raster_flags=LIGHTNING_GDI_RASTER_MIRROR_X;
        stretch_mask.sizlBitmap={1,2};
        const unsigned before_edge=boundary.submissions;
        expect(client.raster_operands(full,masked_stretch,&staging)==Attempt::unsupported &&
            boundary.submissions==before_edge,
            "mask-edge sample where the pinned engines differ stays with the engine");
        masked_stretch.raster_flags=0;
        stretch_mask.sizlBitmap={2,2};
        boundary.capabilities&=~LIGHTNING_GDI_CAP_STRETCH_MASK;
        const bool withdrawn=client.prepare(&cached,&staging);
        const unsigned before_old_host=boundary.submissions;
        expect(withdrawn && client.raster_operands(full,masked_stretch,&staging)==Attempt::unsupported &&
            boundary.submissions==before_old_host,"a host without the stretch-mask capability is never sent one");
        boundary.capabilities|=LIGHTNING_GDI_CAP_STRETCH_MASK;
        expect(client.prepare(&cached,&staging),"restored capability keeps coherent ownership");
    }
    expect(client.transparent(full,&cached,nullptr,{0,0,4,2},{0,0,2,2},0x112233,true,&staging)==Attempt::performed &&
        boundary.last_raster.raster_flags==LIGHTNING_GDI_RASTER_COLOR_KEY &&
        boundary.last_raster.source_kind==LIGHTNING_GDI_ROP_SOURCE_SCREEN &&
        boundary.last_raster.color_key==0x112233 && boundary.last_command.pixel_bytes==0,
        "screen transparency preserves raw key without unnecessary CPU source upload");
    uint8_t indices[8]{0,1,0,0,1,0,0,0};
    SURFOBJ indexed=mono;
    indexed.iBitmapFormat=BMF_8BPP;
    indexed.pvScan0=indices;
    indexed.pvBits=indices;
    indexed.cjBits=sizeof(indices);
    ULONG palette[2]{0xabcdef,0xabcdef};
    XLATEOBJ translation{};
    translation.flXlate=XO_TABLE;
    translation.cEntries=2;
    translation.pulXlate=palette;
    expect(client.transparent(full,&indexed,&translation,{0,0,2,2},{0,0,2,2},1,false,&staging)==Attempt::performed &&
        boundary.last_raster.mask_space==LIGHTNING_GDI_MASK_SPACE_SOURCE &&
        boundary.last_raster.mask_offset==16 && boundary.last_raster.color_key==0 &&
        boundary.last_command.pixel_bytes==24 && boundary.last_payload[16]==0x80 && boundary.last_payload[20]==0x40,
        "noninjective palette translation cannot turn raw-key preserve pixels into draws");
    BLENDOBJ blend{};
    blend.BlendFunction={AC_SRC_OVER,0,128,AC_SRC_ALPHA};
    expect(client.blend(full,&cached,nullptr,{0,0,4,2},{0,0,2,2},&blend,&staging)==Attempt::performed &&
        boundary.last_raster.raster_flags==(LIGHTNING_GDI_RASTER_ALPHA_BLEND|
            LIGHTNING_GDI_RASTER_SOURCE_ALPHA|LIGHTNING_GDI_RASTER_SOURCE_32BPP) &&
        boundary.last_raster.constant_alpha==128,
        "scaled alpha retains source format and per-pixel alpha semantics");
    blend.BlendFunction.AlphaFormat=0;
    expect(client.blend(full,&indexed,&translation,{0,0,4,2},{0,0,2,2},&blend,&staging)==Attempt::performed &&
        boundary.last_raster.raster_flags==LIGHTNING_GDI_RASTER_ALPHA_BLEND,
        "palette expansion does not invent original 32-bit alpha-source semantics");
    ClipBatch right_half{};
    right_half.bounds={1,0,2,2};
    expect(client.blend(right_half,&cached,nullptr,{1,0,2,2},{0,0,1,1},&blend,&staging,true)==Attempt::performed &&
        boundary.last_raster.source_kind==LIGHTNING_GDI_ROP_SOURCE_SCREEN && boundary.last_command.pixel_bytes==0,
        "disjoint same-canvas scaled alpha snapshots bounded GPU source");
    const unsigned before_screen_readbacks=boundary.readbacks;
    memset(boundary.staging_pixels,0xa6,sizeof(boundary.staging_pixels));
    memset(boundary.cached_pixels,0x35,sizeof(boundary.cached_pixels));
    blend.BlendFunction.AlphaFormat=AC_SRC_ALPHA;
    expect(client.blend(right_half,&cached,nullptr,{1,0,2,2},{0,0,1,2},&blend,&staging,true)==Attempt::performed &&
        boundary.last_command.opcode==LIGHTNING_GDI_RASTER && boundary.last_command.pixel_bytes==0 &&
        boundary.last_raster.source_kind==LIGHTNING_GDI_ROP_SOURCE_SCREEN &&
        boundary.last_raster.raster_flags==(LIGHTNING_GDI_RASTER_ALPHA_BLEND|
            LIGHTNING_GDI_RASTER_SOURCE_ALPHA|LIGHTNING_GDI_RASTER_SOURCE_32BPP) &&
        boundary.last_raster.constant_alpha==128 && boundary.readbacks==before_screen_readbacks &&
        boundary.staging_pixels[0]==0xa6,
        "equal-size screen alpha preserves original alpha semantics without CPU readback or staging copy");
    const unsigned before_overlap=boundary.submissions;
    expect(client.blend(full,&cached,nullptr,{0,0,2,2},{0,0,1,1},&blend,&staging,true)==Attempt::unsupported &&
        boundary.submissions==before_overlap,"overlapping alpha declines before any accepted mutation");
    boundary.decline=true;
    expect(!client.synchronize(&cached,&staging) && !client.poisoned(),
        "declined read-only gather keeps stale cache retryable without authorizing CPU access");
    boundary.decline=false;
    expect(client.synchronize(&cached,&staging),"later accepted complete readback retires the stale cache boundary");
    const unsigned before_fill_readbacks=boundary.readbacks;
    const unsigned before_cpu_fills=boundary.cpu_fills;
    memset(boundary.cached_pixels,0x42,sizeof(boundary.cached_pixels));
    expect(client.fill(full,UINT32_MAX,&cached,nullptr)==Attempt::performed &&
        boundary.last_command.opcode==LIGHTNING_GDI_FILL && boundary.last_command.color==UINT32_MAX &&
        boundary.cpu_fills==before_cpu_fills && boundary.cached_pixels[0]==0x42 &&
        boundary.readbacks==before_fill_readbacks,
        "GPU constant fill does not replay a second CPU fill or claim coherent cached pixels");
    expect(client.synchronize(&cached,&staging) && boundary.readbacks>before_fill_readbacks,
        "a later CPU consumer gathers the actual GPU prefix after a fill");
    boundary.capabilities&=~LIGHTNING_GDI_CAP_STRETCH_ALPHA;
    expect(client.prepare(&cached,&staging),"live alpha capability withdrawal retains coherent ownership");
    const unsigned before_missing_alpha=boundary.submissions;
    expect(client.blend(right_half,&cached,nullptr,{1,0,2,2},{0,0,1,2},&blend,&staging,true)==Attempt::unsupported &&
        boundary.submissions==before_missing_alpha && !client.poisoned(),
        "screen alpha without its typed capability declines before stale CPU source access or GPU mutation");
    boundary.capabilities|=LIGHTNING_GDI_CAP_STRETCH_ALPHA;
    expect(client.prepare(&cached,&staging),"live alpha capability restoration needs no fabricated reset");
    boundary.bad_wait=true;
    expect(!client.retire() && client.enabled() && client.poisoned(),"malformed fence completion cannot retire ownership");
    for (unsigned fault=1;fault<=5;++fault) {
        boundary={};
        ClientCanvas malformed;
        expect(start(malformed,pdev),"malformed receipt case begins from a valid production baseline");
        boundary.bad_receipt=fault;
        expect(malformed.copy(full,0,0)==Attempt::failed && malformed.poisoned(),
            "foreign epoch/sequence/revision, impossible completion or uncertainty never authorizes replay");
    }
    boundary={};
    boundary.staging_bytes=16;
    ClientCanvas small;
    expect(start(small,pdev),"small staging fixture owns a valid baseline");
    const unsigned small_submissions=boundary.submissions;
    expect(small.raster_operands(full,masked,&staging)==Attempt::unsupported &&
        !small.poisoned() && boundary.submissions==small_submissions,
        "combined operand overflow declines before GPU acceptance instead of overflowing stage");
    expect(small.begin_text(),"final CPU-flush failure fixture owns logical group");
    boundary.decline=true;
    expect(!small.upload_cached({0,0,2,2}) && !small.poisoned(),"known rejected final CPU upload did not reach GPU canvas");
    small.abort_text();
    boundary.decline=false;
    const unsigned aborted_submissions=boundary.submissions;
    expect(!small.end_text() && !small.prepare(&cached,&staging) && small.poisoned() &&
        boundary.canvas.group_open() && boundary.submissions==aborted_submissions,
        "failed final flush contains publication; neither END nor later prepare can auto-close missing pixels");
    boundary={};
    boundary.capabilities=LIGHTNING_GDI_BASE_CAPABILITIES;
    ClientCanvas base;
    expect(start(base,pdev),"base-only capability retains existing fill/copy/upload acceleration");
    const auto base_submissions=boundary.submissions;
    expect(base.raster(full,0x55,0,nullptr,nullptr,{0,0,2,2},nullptr,false,&staging)==Attempt::unsupported &&
        boundary.submissions==base_submissions,"optional ROP3 never submits to a base-only backend");
    boundary={};
    boundary.capabilities=0;
    ClientCanvas delayed;
    expect(delayed.initialize(&pdev)==Attempt::unsupported,"unready GPU retains recognized native owner generation");
    ++boundary.generation;
    boundary.capabilities=LIGHTNING_GDI_CAPABILITIES;
    clock_ticks+=100;
    expect(!delayed.prepare(&cached,&staging) && delayed.poisoned() && boundary.submissions==0,
        "delayed promotion cannot adopt a replacement PDEV's generation/staging lease");
    boundary={};
    ClientCanvas batched;
    coverage_pixels[0]=193;
    expect(start(batched,pdev) && batched.begin_text(),"disjoint glyph batch owns a real canvas and logical group");
    ClipBatch first{};
    first.bounds={0,0,1,1};
    ClipBatch last{};
    last.bounds={1,1,2,2};
    const unsigned before_batch=boundary.submissions;
    const unsigned before_batch_queries=boundary.coherence_queries;
    expect(batched.queue_glyph(first,&coverage,{0,0,2,2},{0,0},&solid,&staging,&cached)==Attempt::performed &&
        batched.queue_glyph(last,&coverage,{0,0,2,2},{0,0},&solid,&staging,&cached)==Attempt::performed &&
        boundary.submissions==before_batch,"disjoint coverage is copied privately without per-glyph GPU submission");
    expect(boundary.coherence_queries==before_batch_queries,
        "pure owned coverage accumulation needs no per-glyph MMIO coherence query");
    expect(batched.statistics(statistics) && statistics.accepted_glyphs==0,
        "queued glyphs do not count as actual GPU acceptance");
    expect(batched.end_text() && boundary.submissions==before_batch+2 &&
        boundary.last_raster.mask_kind==LIGHTNING_GDI_MASK_COVERAGE8 && boundary.last_payload[0]==193 &&
        boundary.last_payload[1]==0 && boundary.last_payload[4]==0 && boundary.last_payload[5]==64,
        "one coverage draw plus END preserves zero holes between disjoint clipped glyphs");
    expect(batched.statistics(statistics) && statistics.accepted_glyphs==2 && statistics.accepted_rasters==1,
        "aggregate acceptance counts its glyphs while retaining one actual raster submission");
    expect(batched.begin_text() &&
        batched.queue_glyph(first,&coverage,{0,0,2,2},{0,0},&solid,&staging,&cached)==Attempt::performed,
        "overlap fixture retains its first immutable glyph");
    const unsigned before_second=boundary.submissions;
    const unsigned before_overlap_queries=boundary.coherence_queries;
    expect(batched.queue_glyph(first,&coverage,{0,0,2,2},{0,0},&solid,&staging,&cached)==Attempt::performed &&
        boundary.submissions==before_second+1 && batched.end_text(),
        "overlapping coverage flushes in order instead of changing repeated /256 blending");
    expect(boundary.coherence_queries>before_overlap_queries,
        "actual overlapping GPU submission retains live reset and owner preparation");
    expect(batched.begin_text() &&
        batched.queue_glyph(first,&coverage,{0,0,2,2},{0,0},&solid,&staging,&cached)==Attempt::performed,
        "color barrier fixture retains its original solid color");
    BRUSHOBJ other_solid=solid;
    other_solid.iSolidColor^=0x00ffffff;
    const unsigned before_color=boundary.submissions;
    expect(batched.queue_glyph(last,&coverage,{0,0,2,2},{0,0},&other_solid,&staging,&cached)==Attempt::performed &&
        boundary.submissions==before_color+1 && boundary.last_raster.solid_color==solid.iSolidColor && batched.end_text(),
        "different colors remain ordered independent coverage draws");
    expect(batched.begin_text() &&
        batched.queue_glyph(first,&coverage,{0,0,2,2},{0,0},&solid,&staging,&cached)==Attempt::performed,
        "reset replay fixture owns copied unsubmitted coverage");
    boundary.reset();
    expect(batched.prepare(&cached,&staging) && batched.end_text() &&
        boundary.last_payload[0]==193 && boundary.canvas.publication_ready(),
        "proven backend retirement replays retained coverage only after fresh authoritative baseline");
    expect(batched.begin_text() &&
        batched.queue_glyph(first,&coverage,{0,0,2,2},{0,0},&solid,&staging,&cached)==Attempt::performed,
        "unsupported tile glyph fixture queues earlier coverage");
    const unsigned before_tile=boundary.submissions;
    expect(batched.queue_glyph(last,&coverage,{0,0,2,2},{0,0},&tile,&staging,&cached)==Attempt::unsupported &&
        boundary.submissions==before_tile+1 && batched.end_text(),
        "unsupported pattern glyph retires earlier coverage before synchronized engine fallback");
    expect(batched.begin_text() &&
        batched.queue_glyph(full,&coverage,{0,0,2,2},{0,0},&solid,&staging,&cached)==Attempt::performed &&
        batched.synchronize(&cached,&staging) && batched.end_text(),
        "CPU readback includes pending coverage before authorizing cached reads");
    expect(batched.begin_text() &&
        batched.queue_glyph(first,&coverage,{0,0,2,2},{0,0},&solid,&staging,&cached)==Attempt::performed,
        "decline fixture owns handled glyph bytes");
    boundary.decline=true;
    const unsigned before_decline=boundary.submissions;
    expect(!batched.end_text() && batched.poisoned() && boundary.canvas.group_open() &&
        boundary.submissions==before_decline+1,"rejected aggregate contains publication without authorizing replay or END");
    batched.release_cpu_storage();
    boundary={};
    ClientCanvas gradients;
    expect(start(gradients,pdev),"gradient fixture owns an acknowledged baseline");
    ClipBatch gradient_clip{};
    TRIVERTEX vertices[3]{{2,2,0x1234,0x5678,0x9abc,0},{0,0,0x1235,0x5678,0x9abc,0},
        {0,2,0xabcd,0xbcde,0xcdef,0}};
    GRADIENT_RECT rectangles[2]{{0,1},{1,0}};
    const unsigned before_gradient=boundary.submissions;
    expect(gradients.gradient(gradient_clip,nullptr,nullptr,vertices,3,rectangles,2,{0,0,2,2},{99,99},
        GRADIENT_FILL_RECT_H)==Attempt::performed && boundary.submissions==before_gradient+4 &&
        boundary.gradients==2 && !boundary.canvas.group_open(),
        "complete mesh uses two actual gradient commands inside one acknowledged logical group");
    expect(boundary.last_gradient.vertices[0].x==0 && boundary.last_gradient.vertices[1].x==2 &&
        boundary.last_gradient.vertices[0].red_green==0x56789abcU && boundary.last_gradient.vertices[0].blue==0x1235 &&
        boundary.last_gradient.origin_x==0 && boundary.last_gradient.origin_y==0,
        "identity color translation swaps full16 R/B without losing solid-versus-gradient distinctions");
    XLATEOBJ rgb_to_bgr{};
    rgb_to_bgr.iSrcType=PAL_RGB|PAL_BITFIELDS;
    rgb_to_bgr.iDstType=PAL_BGR|PAL_BITFIELDS;
    expect(gradients.gradient(gradient_clip,nullptr,&rgb_to_bgr,vertices,3,rectangles,1,{0,0,2,2},{0,0},
        GRADIENT_FILL_RECT_V)==Attempt::performed && boundary.last_gradient.mode==GRADIENT_FILL_RECT_V &&
        boundary.last_gradient.vertices[0].red_green==0x56781234U && boundary.last_gradient.vertices[0].blue==0x9abc,
        "ordinary bitfield RGB-to-BGR mapping preserves original16-bit gradient components");
    rectangles[1].LowerRight=3;
    const unsigned before_bad_mesh=boundary.submissions;
    expect(gradients.gradient(gradient_clip,nullptr,nullptr,vertices,3,rectangles,2,{0,0,2,2},{0,0},
        GRADIENT_FILL_RECT_H)==Attempt::unsupported && boundary.submissions==before_bad_mesh && !gradients.poisoned(),
        "invalid final mesh index rejects before any earlier gradient or BEGIN is accepted");
    rectangles[1].LowerRight=0;
    rgb_to_bgr.flXlate=XO_DEVICE_ICM;
    expect(gradients.gradient(gradient_clip,nullptr,&rgb_to_bgr,vertices,3,rectangles,1,{0,0,2,2},{0,0},
        GRADIENT_FILL_RECT_H)==Attempt::unsupported && boundary.submissions==before_bad_mesh,
        "unsupported color management cannot silently become a hardware gradient");
    rgb_to_bgr.flXlate=XO_TABLE;
    expect(gradients.gradient(gradient_clip,nullptr,&rgb_to_bgr,vertices,3,rectangles,1,{0,0,2,2},{0,0},
        GRADIENT_FILL_RECT_H)==Attempt::unsupported,"arbitrary palette tables retain exact engine fallback");
    TRIVERTEX triangle_vertices[3]{{-1,-1,0x1201,0x3402,0x5603,0},{1,-1,0x1202,0x3402,0x5603,0},
        {-1,1,0x1203,0x3402,0x5603,0}};
    GRADIENT_TRIANGLE triangle{0,1,2};
    expect(gradients.gradient(gradient_clip,nullptr,nullptr,triangle_vertices,3,&triangle,1,{0,0,2,2},{1,1},
        GRADIENT_FILL_TRIANGLE)==Attempt::performed && boundary.last_gradient.vertices[0].x==-1 &&
        boundary.last_gradient.origin_x==1 && boundary.last_gradient.origin_y==1 &&
        boundary.last_gradient.extents.width==2 && boundary.last_gradient.vertices[0].blue==0x1201,
        "triangle snapshot retains raw vertices, dither origin, original clipping and low color bits");
    triangle.Vertex3=1;
    const unsigned before_degenerate=boundary.submissions;
    expect(gradients.gradient(gradient_clip,nullptr,nullptr,triangle_vertices,3,&triangle,1,{0,0,2,2},{1,1},
        GRADIENT_FILL_TRIANGLE)==Attempt::performed && boundary.submissions==before_degenerate,
        "duplicate-vertex triangle follows the existing engine no-draw path");
    triangle.Vertex3=2;
    expect(gradients.gradient(gradient_clip,nullptr,nullptr,triangle_vertices,3,&triangle,1,{0,0,2,2},{8192,0},
        GRADIENT_FILL_TRIANGLE)==Attempt::unsupported && boundary.submissions==before_degenerate,
        "translated gradient coordinate overflow declines before a hardware group");
    const unsigned before_sync_readbacks=boundary.readbacks;
    const unsigned before_sync_waits=boundary.waits;
    expect(gradients.complete_gpu(&cached,&staging) && boundary.waits==before_sync_waits+1 &&
        boundary.readbacks==before_sync_readbacks,"GPU synchronizer waits the real prefix without forcing CPU readback");
    expect(gradients.synchronize(&cached,&staging) && boundary.readbacks>before_sync_readbacks,
        "GPU completion alone never falsely marks cached pixels coherent");
    expect(gradients.begin_text() &&
        gradients.queue_glyph(full,&coverage,{0,0,2,2},{0,0},&solid,&staging,&cached)==Attempt::performed,
        "synchronization fixture owns a live text group with pending copied coverage");
    const unsigned before_text_sync=boundary.submissions;
    const unsigned before_text_readback=boundary.readbacks;
    expect(gradients.complete_gpu(&cached,&staging) && boundary.submissions==before_text_sync+1 &&
        boundary.canvas.group_open() && !boundary.canvas.publication_ready() && boundary.readbacks==before_text_readback,
        "completion flushes queued glyphs but never ends or publishes a live text scope");
    expect(gradients.end_text(),"explicit text END retains responsibility for group publication");
    gradients.release_cpu_storage();
    boundary.decline_gradient=boundary.gradients+2;
    const unsigned before_partial=boundary.gradients;
    expect(gradients.gradient(gradient_clip,nullptr,nullptr,vertices,3,rectangles,2,{0,0,2,2},{0,0},
        GRADIENT_FILL_RECT_H)==Attempt::failed && gradients.poisoned() && boundary.canvas.group_open() &&
        boundary.gradients==before_partial+2,"later rejected mesh command contains its accepted prefix and forbids CPU replay");
    boundary={};
    ClientCanvas monochrome_glyph;
    expect(start(monochrome_glyph,pdev) && monochrome_glyph.begin_text(),
        "monochrome text fixture owns a real baseline and publication group");
    uint8_t glyph_bits[8]{0x80,0,0,0,0x40,0,0,0};
    SURFOBJ glyph_mask=coverage;
    glyph_mask.iBitmapFormat=BMF_1BPP;
    glyph_mask.pvBits=glyph_mask.pvScan0=glyph_bits;
    glyph_mask.cjBits=sizeof(glyph_bits);
    glyph_mask.sizlBitmap={2,2};
    glyph_mask.lDelta=4;
    ClipBatch mono_clip{};
    mono_clip.bounds={0,0,2,2};
    const unsigned before_mono_glyph=boundary.submissions;
    const auto mono_result=monochrome_glyph.queue_glyph(mono_clip,&glyph_mask,{0,0,2,2},{0,0},&solid,&staging,&cached);
    expect(mono_result==Attempt::performed &&
        boundary.submissions==before_mono_glyph+1 && boundary.last_command.opcode==LIGHTNING_GDI_RASTER &&
        boundary.last_raster.mask_kind==LIGHTNING_GDI_MASK_MONO1 &&
        boundary.last_raster.foreground_rop==0xf0 && boundary.last_raster.background_rop==0xaa &&
        boundary.last_raster.mask_width==2 && boundary.last_raster.mask_height==2 &&
        boundary.last_payload[0]==0x80 && boundary.last_payload[4]==0x40,
        "actual1bpp glyph uses GPU monochrome composition without thresholding or CPU color rendering");
    expect(monochrome_glyph.statistics(statistics) && statistics.accepted_glyphs==1 &&
        statistics.accepted_rasters==1 && statistics.cpu_fallbacks==0,
        "monochrome text increments only real accepted GPU glyphs with zero CPU fallback");
    expect(monochrome_glyph.end_text() && boundary.canvas.publication_ready(),
        "monochrome glyph publication retains the actual END checkpoint");
    monochrome_glyph.release_cpu_storage();
    expect(bitmap_locks==0 && allocations==0,"fixture leaves production engine resources balanced");
    printf("Production GDI client lifecycle: %u checks, %u failures; GPU/native PnP not simulated\n",checks,failures);
    return failures==0 ? 0 : 1;
}

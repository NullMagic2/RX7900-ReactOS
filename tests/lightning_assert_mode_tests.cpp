/*
Module: lightning.tests.assert_mode
Purpose: Prove DrvAssertMode(TRUE) converges under win32k's enable-until-success loop.
Created: 2026-10-25
Architecture: Real framebuffer_shadow.cpp, lightning_display.cpp, staging ledger,
mode lifecycle and resolve policy; the engine and VideoPort StartIO are a bounded
paired model. win32k PDEVOBJ_vEnableDisplay (pdevobj.c) is modeled as a capped
retry loop. GPU offload, native mutexes and host publication are not simulated.
*/
#include "framebuf.h"
#include "lightning_display.hpp"
#include "lightning_mode_lifecycle.hpp"
#include "lightning_staging_resolve.hpp"
#include "lightning_present_abi.hpp"
#include "lightning-display.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace {
using namespace lightning::reactos;
unsigned checks=0;
unsigned failures=0;

void expect(bool condition,const char* description) {
    ++checks;
    if (!condition) {++failures; printf("FAIL %s\n",description);}
}

constexpr uint32_t width=16;
constexpr uint32_t height=16;
constexpr uint32_t stride=width*4;
constexpr uint32_t bytes=stride*height;
constexpr uint32_t ram_count=16;
// Far above enable_attempt_limit: a run that reaches it never converged.
constexpr unsigned win32k_cap=64;

struct Bitmap final {
    SURFOBJ surface{};
    uint8_t storage[bytes]{};
    bool used=false;
};
Bitmap bitmaps[12]{};
unsigned device_surfaces=0;
unsigned allocations=0;
// Full-frame cache requests by pool (lightning_staging_pool.hpp frame_pool).
unsigned nonpaged_frames=0;
unsigned paged_frames=0;

struct Native final {
    StagingLedger ledger{};
    ModeRollbackCredential rollback{};
    uint8_t ram[ram_count][bytes]{};
    uint32_t next_ram=0;
    uint32_t mode_generation=1;
    bool lost=false;
    bool enabled=true;
    bool cursor_defined=false;
    // Persistent faults of one mandatory or optional enable step.
    bool refuse_map=false;
    bool refuse_present=false;
    bool refuse_cursor=false;
    unsigned enable_commits=0;
    unsigned maps=0;
    unsigned presents=0;
    unsigned cursor_shapes=0;

    void reclaim() { while (ledger.reclaim()!=nullptr) {} }
    void* map(uint32_t initiator) {
        if (next_ram==ram_count) return nullptr;
        ledger.retire();
        reclaim();
        if (!ledger.install(ram[next_ram],bytes)) return nullptr;
        void* address=ledger.acquire(mode_generation+1U,initiator);
        if (address==nullptr) return nullptr;
        ++next_ram;
        ++mode_generation;
        ++maps;
        return address;
    }
} native;

CpuScanoutInformation information() {
    CpuScanoutInformation value{};
    value.size=sizeof(value);
    value.version=cpu_scanout_abi_version;
    value.magic=LIGHTNING_DISPLAY_MAGIC;
    value.width=width;
    value.height=height;
    value.stride=stride;
    value.mapped_bytes=bytes;
    value.capabilities=LIGHTNING_DISPLAY_CAPABILITIES;
    value.mode_generation=native.mode_generation;
    value.enabled=native.enabled ? 1U : 0U;
    return value;
}

template<typename T>
DWORD output_value(PVOID output,DWORD output_bytes,LPDWORD returned,const T& value) {
    if (output==nullptr || output_bytes!=sizeof(value)) return 1;
    memcpy(output,&value,sizeof(value));
    *returned=sizeof(value);
    return 0;
}
} // namespace

extern "C" DWORD APIENTRY EngDeviceIoControl(HANDLE,DWORD code,PVOID input,DWORD input_bytes,
    PVOID output,DWORD output_bytes,LPDWORD returned) {
    *returned=0;
    if (code==ioctl_resolve_owned_staging) {
        StagingResolveRequest request{};
        if (input==nullptr || input_bytes!=sizeof(request)) return 1;
        memcpy(&request,input,sizeof(request));
        if (!valid_staging_resolve_request(request,input_bytes)) return 1;
        const StagingResolveReceipt receipt=resolve_staging(native.ledger,request,native.lost);
        native.reclaim();
        return output_value(output,output_bytes,returned,receipt);
    }
    if (code==IOCTL_VIDEO_UNMAP_VIDEO_MEMORY) {
        VIDEO_MEMORY memory{};
        if (input==nullptr || input_bytes!=sizeof(memory)) return 1;
        memcpy(&memory,input,sizeof(memory));
        const bool released=native.ledger.release(memory.RequestedVirtualAddress);
        native.reclaim();
        return released ? 0 : 1;
    }
    if (native.lost) return 1;
    if (code==ioctl_query_cpu_scanout) return output_value(output,output_bytes,returned,information());
    if (code==ioctl_query_owned_staging) {
        VIDEO_MEMORY memory{};
        if (input==nullptr || input_bytes!=sizeof(memory)) return 1;
        memcpy(&memory,input,sizeof(memory));
        if (!native.enabled ||
            native.ledger.current_mapping_generation(memory.RequestedVirtualAddress)!=native.mode_generation) return 1;
        return output_value(output,output_bytes,returned,information());
    }
    if (code==ioctl_wait_cpu_scanout) return 0;
    if (code==ioctl_present_cpu_scanout) {
        DamageRequest request{};
        if (input==nullptr || input_bytes<sizeof(request) || !native.enabled || native.refuse_present) return 1;
        memcpy(&request,input,sizeof(request));
        if (request.mode_generation!=native.mode_generation) return 1;
        ++native.presents;
        return 0;
    }
    if (code==ioctl_cursor_shape) {
        CursorShapeRequest header{};
        if (input==nullptr || input_bytes<sizeof(header)) return 1;
        memcpy(&header,input,sizeof(header));
        if (native.refuse_cursor || header.size!=input_bytes || header.mode_generation!=native.mode_generation ||
            !native.enabled) return 1;
        native.cursor_defined=true;
        ++native.cursor_shapes;
        return 0;
    }
    if (code==ioctl_cursor_move) {
        CursorMoveRequest request{};
        if (input==nullptr || input_bytes!=sizeof(request)) return 1;
        memcpy(&request,input,sizeof(request));
        if (request.mode_generation!=native.mode_generation || (request.visible!=0 && !native.cursor_defined)) return 1;
        return 0;
    }
    if (code==ioctl_transition_owned_mode) {
        ModeTransitionRequest request{};
        if (input==nullptr || input_bytes!=sizeof(request)) return 1;
        memcpy(&request,input,sizeof(request));
        if (!mode_transition_authorized(native.ledger,native.mode_generation,native.rollback,request)) return 1;
        if (request.enable!=0) {
            // set_mode: a full commit that clears and republishes the scanout
            // and hides/undefines the native cursor plane.
            native.rollback=current_mode_rollback_credential(native.ledger,native.mode_generation,
                request.owner.lease_generation);
            ++native.mode_generation;
            ++native.enable_commits;
            native.cursor_defined=false;
        }
        native.enabled=request.enable!=0;
        const ModeTransitionReceipt receipt{sizeof(receipt),cpu_scanout_abi_version,request.owner.mode_generation,
            native.mode_generation,request.owner.mode_index,0,native.enabled ? 1U : 0U,0};
        return output_value(output,output_bytes,returned,receipt);
    }
    if (code==ioctl_map_owned_staging) {
        ModeOwnerRequest owner{};
        if (input==nullptr || input_bytes!=sizeof(owner)) return 1;
        memcpy(&owner,input,sizeof(owner));
        if (native.refuse_map || !valid_mode_owner(owner,sizeof(owner)) || owner.mode_generation!=native.mode_generation ||
            !native.enabled || !current_mode_lease_authorized(native.ledger,native.rollback,owner.lease_generation))
            return 1;
        native.rollback=current_mode_rollback_credential(native.ledger,native.mode_generation,owner.lease_generation);
        void* address=native.map(owner.lease_generation);
        if (address==nullptr) return 1;
        VIDEO_MEMORY_INFORMATION mapping{};
        mapping.VideoRamBase=mapping.FrameBufferBase=address;
        mapping.VideoRamLength=mapping.FrameBufferLength=bytes;
        return output_value(output,output_bytes,returned,mapping);
    }
    if (code==ioctl_query_cursor) {
        CursorStatistics statistics{};
        statistics.size=sizeof(statistics);
        statistics.version=cpu_scanout_abi_version;
        statistics.magic=cursor_statistics_magic;
        statistics.mode_generation=native.mode_generation;
        statistics.defined=native.cursor_defined ? 1U : 0U;
        return output_value(output,output_bytes,returned,statistics);
    }
    return 1;
}

extern "C" PVOID APIENTRY EngAllocMem(ULONG flags,ULONG size,ULONG) {
    if (size==bytes) ++((flags&FL_NONPAGED_MEMORY)!=0 ? nonpaged_frames : paged_frames);
    void* allocation=calloc(1,size);
    if (allocation!=nullptr) ++allocations;
    return allocation;
}

extern "C" VOID APIENTRY EngFreeMem(PVOID allocation) {
    if (allocation!=nullptr) {--allocations; free(allocation);}
}

extern "C" HBITMAP APIENTRY EngCreateBitmap(SIZEL size,LONG delta,ULONG format,FLONG,PVOID bits) {
    // The real engine allocates a bitless bitmap as one PagedPool object.
    if (bits==nullptr && static_cast<uint64_t>(delta)*size.cy==bytes) ++paged_frames;
    if (size.cx<=0 || size.cy<=0 || delta<=0 || static_cast<uint64_t>(delta)*size.cy>bytes) return nullptr;
    for (auto& bitmap:bitmaps) {
        if (bitmap.used) continue;
        bitmap=Bitmap{};
        bitmap.used=true;
        bitmap.surface.iType=STYPE_BITMAP;
        bitmap.surface.sizlBitmap=size;
        bitmap.surface.lDelta=delta;
        bitmap.surface.iBitmapFormat=format;
        bitmap.surface.pvScan0=bits!=nullptr ? bits : bitmap.storage;
        bitmap.surface.pvBits=bitmap.surface.pvScan0;
        bitmap.surface.cjBits=static_cast<ULONG>(static_cast<uint64_t>(delta)*size.cy);
        bitmap.surface.hsurf=reinterpret_cast<HSURF>(&bitmap);
        return reinterpret_cast<HBITMAP>(&bitmap);
    }
    return nullptr;
}

extern "C" SURFOBJ* APIENTRY EngLockSurface(HSURF handle) {
    for (auto& bitmap:bitmaps) {
        if (reinterpret_cast<HSURF>(&bitmap)==handle && bitmap.used) return &bitmap.surface;
    }
    return nullptr;
}

extern "C" VOID APIENTRY EngUnlockSurface(SURFOBJ*) {}

extern "C" BOOL APIENTRY EngDeleteSurface(HSURF handle) {
    for (auto& bitmap:bitmaps) {
        if (reinterpret_cast<HSURF>(&bitmap)==handle && bitmap.used) {bitmap.used=false; return TRUE;}
    }
    if (handle!=nullptr && device_surfaces!=0) {--device_surfaces; return TRUE;}
    return FALSE;
}

extern "C" HSURF APIENTRY EngCreateDeviceSurface(DHSURF,SIZEL,ULONG) {
    ++device_surfaces;
    static uint8_t handles[16];
    return reinterpret_cast<HSURF>(&handles[device_surfaces%16]);
}

extern "C" BOOL APIENTRY EngAssociateSurface(HSURF,HDEV,FLONG) {return TRUE;}

extern "C" BOOL APIENTRY EngCopyBits(SURFOBJ* destination,SURFOBJ* source,CLIPOBJ*,XLATEOBJ*,
    RECTL* rectangle,POINTL* point) {
    if (destination==nullptr || source==nullptr || rectangle==nullptr || point==nullptr ||
        destination->iBitmapFormat!=BMF_32BPP || source->iBitmapFormat!=BMF_32BPP ||
        rectangle->left<0 || rectangle->top<0 || rectangle->right>destination->sizlBitmap.cx ||
        rectangle->bottom>destination->sizlBitmap.cy || point->x<0 || point->y<0 ||
        point->x+rectangle->right-rectangle->left>source->sizlBitmap.cx ||
        point->y+rectangle->bottom-rectangle->top>source->sizlBitmap.cy) return FALSE;
    for (LONG y=rectangle->top;y<rectangle->bottom;++y) {
        memcpy(static_cast<uint8_t*>(destination->pvScan0)+y*destination->lDelta+rectangle->left*4,
            static_cast<const uint8_t*>(source->pvScan0)+(point->y+y-rectangle->top)*source->lDelta+point->x*4,
            static_cast<size_t>(rectangle->right-rectangle->left)*4U);
    }
    return TRUE;
}

extern "C" BOOL APIENTRY EngBitBlt(SURFOBJ*,SURFOBJ*,SURFOBJ*,CLIPOBJ*,XLATEOBJ*,RECTL*,POINTL*,
    POINTL*,BRUSHOBJ*,POINTL*,ROP4) {return FALSE;}
extern "C" VOID APIENTRY EngMovePointer(SURFOBJ*,LONG,LONG,RECTL*) {}
extern "C" HSEMAPHORE APIENTRY EngCreateSemaphore(VOID) {
    static int semaphore;
    return reinterpret_cast<HSEMAPHORE>(&semaphore);
}
extern "C" VOID APIENTRY EngAcquireSemaphore(HSEMAPHORE) {}
extern "C" VOID APIENTRY EngReleaseSemaphore(HSEMAPHORE) {}
extern "C" VOID APIENTRY EngDeleteSemaphore(HSEMAPHORE) {}
extern "C" VOID APIENTRY EngQueryPerformanceCounter(LONGLONG* value) {static LONGLONG clock; *value=++clock;}
extern "C" VOID APIENTRY EngQueryPerformanceFrequency(LONGLONG* value) {*value=1000;}

namespace {
PDEV device{};
HSURF display=nullptr;

bool enable() {
    native=Native{};
    native.rollback=current_mode_rollback_credential(native.ledger,native.mode_generation);
    device=PDEV{};
    device.hDriver=reinterpret_cast<HANDLE>(static_cast<uintptr_t>(1));
    device.hDevEng=reinterpret_cast<HDEV>(static_cast<uintptr_t>(2));
    device.ModeIndex=1;
    device.ScreenWidth=width;
    device.ScreenHeight=height;
    device.ScreenDelta=stride;
    device.BitsPerPixel=32;
    device.ScreenPtr=native.map(0);
    display=FramebufferShadowEnable(&device,bytes);
    return display!=nullptr;
}

void disable() {
    FramebufferShadowDisable(&device);
    VIDEO_MEMORY memory{device.ScreenPtr};
    DWORD returned=0;
    EngDeviceIoControl(device.hDriver,IOCTL_VIDEO_UNMAP_VIDEO_MEMORY,&memory,sizeof(memory),nullptr,0,&returned);
}

SURFOBJ device_surface() {
    SURFOBJ target{};
    target.iType=STYPE_DEVICE;
    target.hsurf=display;
    return target;
}

bool draw() {
    static uint8_t pixels[bytes];
    SURFOBJ source{};
    source.iType=STYPE_BITMAP;
    source.iBitmapFormat=BMF_32BPP;
    source.sizlBitmap={static_cast<LONG>(width),static_cast<LONG>(height)};
    source.lDelta=stride;
    source.pvScan0=pixels;
    SURFOBJ target=device_surface();
    RECTL rectangle{0,0,4,4};
    POINTL origin{0,0};
    return FramebufferShadowCopyBits(&target,&source,nullptr,nullptr,&rectangle,&origin)!=FALSE;
}

bool define_cursor() {
    static uint8_t pixels[]{32,64,96,128, 0,0,0,0, 1,2,3,255, 0,0,0,0};
    SURFOBJ color{};
    color.iType=STYPE_BITMAP;
    color.iBitmapFormat=BMF_32BPP;
    color.sizlBitmap={2,2};
    color.lDelta=8;
    color.pvScan0=pixels;
    SURFOBJ target=device_surface();
    return FramebufferShadowSetPointer(&target,nullptr,&color,nullptr,1,1,5,5,nullptr,SPS_ALPHA)==SPS_ACCEPT_NOEXCLUDE;
}

DisplayModeHealth health() {
    DisplayModeHealth value{};
    SURFOBJ target=device_surface();
    const ULONG returned=FramebufferShadowEscape(&target,query_mode_health_escape,0,nullptr,sizeof(value),&value);
    if (returned!=sizeof(value) || value.size!=sizeof(value) || value.magic!=mode_health_magic ||
        value.version!=mode_health_version) return DisplayModeHealth{};
    return value;
}

// win32k PDEVOBJ_vEnableDisplay: do { ok=DrvAssertMode(TRUE); } while (!ok);
// Returns the call that succeeded, or 0 when the capped loop never returned.
unsigned win32k_enable_display() {
    for (unsigned call=1;call<=win32k_cap;++call) {
        if (FramebufferShadowAssertMode(&device,TRUE)!=FALSE) return call;
    }
    return 0;
}
} // namespace

int main() {
    expect(FramebufferShadowInitialize()!=FALSE,"shadow registry initializes");

    // Ordinary mode switch: one owned disable, one enable.
    expect(enable() && draw(),"PDEV publishes through its leased staging");
    if (sizeof(void*)==8) {
        expect(nonpaged_frames==1 && paged_frames==0,
            "64-bit: the full-frame GDI cache comes from nonpaged pool, never the 32 MB AMD64 paged pool");
    } else {
        expect(paged_frames==1 && nonpaged_frames==0,"32-bit: the full-frame GDI cache keeps the engine's paged bitmap");
    }
    expect(FramebufferShadowAssertMode(&device,FALSE)!=FALSE && !native.enabled,"owned disable yields scanout");
    unsigned commits=native.enable_commits;
    expect(win32k_enable_display()==1 && native.enable_commits==commits+1 && native.enabled,
        "a healthy enable succeeds on its first call with one native commit");
    DisplayModeHealth record=health();
    expect(record.enables==1 && record.failed_attempts==0 && record.flags==0 &&
        record.mode_generation==native.mode_generation,"health records one clean enable");
    DisplayModeHealth previous=record;

    // win32k asserts the PDEV it just swapped in after DrvEnableSurface.
    commits=native.enable_commits;
    const unsigned maps=native.maps;
    const unsigned presents=native.presents;
    expect(FramebufferShadowAssertMode(&device,TRUE)!=FALSE,"enabling the live owner succeeds");
    expect(native.enable_commits==commits && native.maps==maps && native.presents==presents,
        "the live owner is not recommitted, remapped or cleared");
    expect(draw() && native.presents>presents,"the live owner still publishes");
    disable();

    // Persistent mandatory failure: the staging remap is refused.
    expect(enable() && FramebufferShadowAssertMode(&device,FALSE)!=FALSE,"PDEV suspends for a mode change");
    native.refuse_map=true;
    commits=native.enable_commits;
    expect(win32k_enable_display()==enable_attempt_limit,
        "win32k's enable loop ends after the bounded attempt limit");
    expect(native.enable_commits-commits==enable_attempt_limit,
        "each bounded attempt commits the native mode once; no unbounded clear/republish");
    record=health();
    expect((record.flags&mode_health_parked_now)!=0 && (record.flags&mode_health_parked)!=0 &&
        record.failed_attempts-previous.failed_attempts==enable_attempt_limit &&
        record.last_failed_step==static_cast<uint32_t>(EnableStep::remap) && record.last_error==1,
        "health reports the parked PDEV, the failed remap step and the native MAP error");
    unsigned before=native.presents;
    expect(draw() && native.presents==before,"a parked PDEV keeps drawing locally and publishes nothing");
    native.refuse_map=false;
    expect(FramebufferShadowAssertMode(&device,FALSE)!=FALSE && win32k_enable_display()==1,
        "the next owned transition recovers the parked PDEV");
    record=health();
    before=native.presents;
    expect((record.flags&mode_health_parked_now)==0 && (record.flags&mode_health_parked)!=0 &&
        record.consecutive_failures==0 && draw() && native.presents>before,
        "recovered PDEV is visible and publishes; the driver-wide record keeps the earlier park");
    disable();

    // Persistent publication refusal: the CPU flush is mandatory.
    expect(enable() && FramebufferShadowAssertMode(&device,FALSE)!=FALSE,"PDEV suspends before a refused present");
    native.refuse_present=true;
    expect(win32k_enable_display()==enable_attempt_limit,"refused publication also converges");
    record=health();
    expect((record.flags&mode_health_parked_now)!=0 &&
        record.last_failed_step==static_cast<uint32_t>(EnableStep::flush),"health names the failed flush");
    native.refuse_present=false;
    disable();

    // Optional step: the retained hardware cursor cannot be replayed.
    expect(enable() && define_cursor() && native.cursor_shapes==1,"hardware cursor is accepted");
    expect(FramebufferShadowAssertMode(&device,FALSE)!=FALSE,"PDEV with a hardware cursor suspends");
    native.refuse_cursor=true;
    commits=native.enable_commits;
    previous=health();
    expect((previous.flags&mode_health_cursor_dropped)==0,"no cursor was dropped before");
    expect(win32k_enable_display()==1 && native.enable_commits==commits+1,
        "an unreplayable cursor does not fail the enable");
    record=health();
    expect((record.flags&mode_health_cursor_dropped)!=0 && (record.flags&mode_health_parked_now)==0 &&
        record.failed_attempts==previous.failed_attempts && record.enables==previous.enables+1,
        "health records the dropped cursor, not a failed enable");
    before=native.presents;
    expect(draw() && native.presents>before,"the enabled PDEV publishes after the cursor drop");
    native.refuse_cursor=false;
    expect(define_cursor() && native.cursor_shapes==2,"the engine's next pointer shape is accepted again");
    disable();

    // Removal while suspended: the transition can never succeed again.
    expect(enable() && FramebufferShadowAssertMode(&device,FALSE)!=FALSE,"PDEV suspends before removal");
    native.lost=true;
    native.enabled=false;
    native.ledger.retire();
    native.reclaim();
    expect(win32k_enable_display()==enable_attempt_limit,"enable of a removed device converges");
    record=health();
    expect((record.flags&mode_health_parked_now)!=0 &&
        record.last_failed_step==static_cast<uint32_t>(EnableStep::transition) &&
        native.ledger.held_leases()==0,"parked removed PDEV retired every lease");
    disable();

    FramebufferShadowShutdown();
    expect(allocations==0 && device_surfaces==0,"all engine objects retire");
    for (const auto& bitmap:bitmaps) expect(!bitmap.used,"no engine bitmap leaks");
    printf("Assert mode: %u checks, %u failures; GPU offload, native mutexes and host publication not simulated\n",
        checks,failures);
    return failures==0 ? 0 : 1;
}

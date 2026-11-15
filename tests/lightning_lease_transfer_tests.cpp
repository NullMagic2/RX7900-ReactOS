/*
Module: lightning.tests.lease_transfer
Purpose: Exercise production surprise-removal lease transfer and lost releases.
Created: 2026-10-23
Architecture: Real framebuffer_shadow.cpp, lightning_display.cpp, staging ledger
and resolve policy; the engine and VideoPort StartIO are a bounded paired model.
GPU offload, native mutexes and real pool frees are not simulated.
*/
#include "framebuf.h"
#include "lightning_display.hpp"
#include "lightning_mode_lifecycle.hpp"
#include "lightning_staging_resolve.hpp"
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
constexpr uint32_t ram_count=6;

// Engine bitmaps. A bitmap over caller bits aliases that RAM: the transfer
// contract is that no such alias exists when the miniport frees its RAM.
struct Bitmap final {
    SURFOBJ surface{};
    uint8_t storage[bytes]{};
    void* bits=nullptr;
    bool used=false;
};
Bitmap bitmaps[8]{};
unsigned device_surfaces=0;
unsigned locks=0;
unsigned allocations=0;

bool aliased(const void* address) {
    for (const auto& bitmap:bitmaps) {
        if (bitmap.used && bitmap.bits==address) return true;
    }
    return false;
}

// Paired StartIO model over the production ledger and resolve policy.
struct Native final {
    StagingLedger ledger{};
    StagingFaultPlan faults{};
    ModeRollbackCredential rollback{};
    uint8_t ram[ram_count][bytes]{};
    unsigned frees[ram_count]{};
    uint32_t next_ram=0;
    uint32_t mode_generation=1;
    bool lost=false;
    bool enabled=true;
    bool legacy=false;
    // The miniport answers a disable whose quiesce failed: blanking began,
    // so scanout is no longer enabled, and the receipt reports the failure.
    bool fail_disable=false;
    unsigned aliased_frees=0;
    unsigned resolves=0;
    unsigned legacy_releases=0;
    unsigned unmaps=0;
    unsigned presents=0;

    void reclaim() {
        while (void* retired=ledger.reclaim()) {
            if (aliased(retired)) ++aliased_frees;
            for (uint32_t i=0;i<ram_count;++i) {
                if (retired==ram[i]) ++frees[i];
            }
        }
    }

    void* map(uint32_t initiator) {
        if (next_ram==ram_count) return nullptr;
        ledger.retire();
        reclaim();
        if (!ledger.install(ram[next_ram],bytes)) return nullptr;
        void* address=ledger.acquire(mode_generation+1U,initiator);
        if (address==nullptr) return nullptr;
        ++next_ram;
        ++mode_generation;
        return address;
    }

    void abandon() {
        lost=true;
        enabled=false;
        ledger.retire();
        reclaim();
    }

    unsigned freed_total() const {
        unsigned total=0;
        for (const auto count:frees) total+=count;
        return total;
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
        if (native.legacy || input==nullptr || input_bytes!=sizeof(request)) return 1;
        memcpy(&request,input,sizeof(request));
        if (!valid_staging_resolve_request(request,input_bytes)) return 1;
        ++native.resolves;
        const StagingFault fault=native.faults.next(request.operation);
        if (fault==StagingFault::drop_before_apply) return 1;
        const StagingResolveReceipt receipt=resolve_staging(native.ledger,request,native.lost);
        native.reclaim();
        if (fault==StagingFault::lost_after_apply) return 1;
        return output_value(output,output_bytes,returned,receipt);
    }
    if (code==ioctl_release_owned_staging) {
        StagingReleaseRequest request{};
        if (input==nullptr || input_bytes!=sizeof(request)) return 1;
        memcpy(&request,input,sizeof(request));
        ++native.legacy_releases;
        const bool released=native.ledger.release_generation(request.lease_generation);
        native.reclaim();
        return released ? 0 : 1;
    }
    if (code==IOCTL_VIDEO_UNMAP_VIDEO_MEMORY) {
        VIDEO_MEMORY memory{};
        if (input==nullptr || input_bytes!=sizeof(memory)) return 1;
        memcpy(&memory,input,sizeof(memory));
        ++native.unmaps;
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
        if (input==nullptr || input_bytes<sizeof(request) || !native.enabled) return 1;
        memcpy(&request,input,sizeof(request));
        if (request.mode_generation!=native.mode_generation) return 1;
        ++native.presents;
        return 0;
    }
    if (code==ioctl_transition_owned_mode) {
        ModeTransitionRequest request{};
        if (input==nullptr || input_bytes!=sizeof(request)) return 1;
        memcpy(&request,input,sizeof(request));
        if (!mode_transition_authorized(native.ledger,native.mode_generation,native.rollback,request)) return 1;
        if (request.enable!=0) {
            native.rollback=current_mode_rollback_credential(native.ledger,native.mode_generation,
                request.owner.lease_generation);
            ++native.mode_generation;
        }
        native.enabled=request.enable!=0;
        const uint32_t result=request.enable==0 && native.fail_disable ? 170U : 0U;
        const ModeTransitionReceipt receipt{sizeof(receipt),cpu_scanout_abi_version,request.owner.mode_generation,
            native.mode_generation,request.owner.mode_index,result,native.enabled ? 1U : 0U,0};
        return output_value(output,output_bytes,returned,receipt);
    }
    if (code==ioctl_map_owned_staging) {
        ModeOwnerRequest owner{};
        if (input==nullptr || input_bytes!=sizeof(owner)) return 1;
        memcpy(&owner,input,sizeof(owner));
        if (!valid_mode_owner(owner,sizeof(owner)) || owner.mode_generation!=native.mode_generation ||
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
    return 1;
}

extern "C" PVOID APIENTRY EngAllocMem(ULONG,ULONG size,ULONG) {
    void* allocation=calloc(1,size);
    if (allocation!=nullptr) ++allocations;
    return allocation;
}

extern "C" VOID APIENTRY EngFreeMem(PVOID allocation) {
    if (allocation!=nullptr) {--allocations; free(allocation);}
}

extern "C" HBITMAP APIENTRY EngCreateBitmap(SIZEL size,LONG delta,ULONG format,FLONG,PVOID bits) {
    if (size.cx!=static_cast<LONG>(width) || size.cy!=static_cast<LONG>(height) ||
        delta!=static_cast<LONG>(stride) || format!=BMF_32BPP) return nullptr;
    for (auto& bitmap:bitmaps) {
        if (bitmap.used) continue;
        bitmap=Bitmap{};
        bitmap.used=true;
        bitmap.bits=bits;
        bitmap.surface.iType=STYPE_BITMAP;
        bitmap.surface.sizlBitmap=size;
        bitmap.surface.lDelta=delta;
        bitmap.surface.iBitmapFormat=format;
        bitmap.surface.pvScan0=bits!=nullptr ? bits : bitmap.storage;
        bitmap.surface.pvBits=bitmap.surface.pvScan0;
        bitmap.surface.cjBits=bytes;
        bitmap.surface.hsurf=reinterpret_cast<HSURF>(&bitmap);
        return reinterpret_cast<HBITMAP>(&bitmap);
    }
    return nullptr;
}

extern "C" SURFOBJ* APIENTRY EngLockSurface(HSURF handle) {
    for (auto& bitmap:bitmaps) {
        if (reinterpret_cast<HSURF>(&bitmap)==handle && bitmap.used) {++locks; return &bitmap.surface;}
    }
    return nullptr;
}

extern "C" VOID APIENTRY EngUnlockSurface(SURFOBJ*) {--locks;}

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
        rectangle->left<0 || rectangle->top<0 || rectangle->right>static_cast<LONG>(width) ||
        rectangle->bottom>static_cast<LONG>(height) || point->x<0 || point->y<0) return FALSE;
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

// Upstream framebuf DrvEnableSurface: legacy MAP, then the shadow owner.
bool enable(bool legacy=false) {
    native=Native{};
    native.legacy=legacy;
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

// Upstream framebuf DrvDisableSurface tail after the shadow owner retires.
void disable() {
    FramebufferShadowDisable(&device);
    VIDEO_MEMORY memory{device.ScreenPtr};
    DWORD returned=0;
    EngDeviceIoControl(device.hDriver,IOCTL_VIDEO_UNMAP_VIDEO_MEMORY,&memory,sizeof(memory),nullptr,0,&returned);
}

bool draw() {
    static uint8_t pixels[bytes];
    SURFOBJ source{};
    source.iType=STYPE_BITMAP;
    source.iBitmapFormat=BMF_32BPP;
    source.sizlBitmap={static_cast<LONG>(width),static_cast<LONG>(height)};
    source.lDelta=stride;
    source.pvScan0=pixels;
    SURFOBJ target{};
    target.iType=STYPE_DEVICE;
    target.hsurf=display;
    RECTL rectangle{0,0,4,4};
    POINTL origin{0,0};
    return FramebufferShadowCopyBits(&target,&source,nullptr,nullptr,&rectangle,&origin)!=FALSE;
}

bool no_lease_left() {
    return native.ledger.held_leases()==0 && !aliased(native.ram[0]) && !aliased(native.ram[1]) &&
        !aliased(native.ram[2]) && !aliased(native.ram[3]);
}

bool each_freed_once(uint32_t count) {
    for (uint32_t i=0;i<count;++i) {
        if (native.frees[i]!=1) return false;
    }
    return true;
}
} // namespace

int main() {
    expect(FramebufferShadowInitialize()!=FALSE,"shadow registry initializes");

    // Removal while drawing to the active staging lease.
    expect(enable() && aliased(native.ram[0]) && draw() && native.presents>=2,
        "PDEV draws and publishes through its leased staging wrapper");
    native.abandon();
    expect(native.freed_total()==0 && native.ledger.held_leases()==1,"abandon keeps the aliased lease");
    expect(!draw(),"first draw after removal reports the failed publication");
    expect(native.aliased_frees==0 && native.frees[0]==1 && no_lease_left(),
        "drawing removal moves the view first, then frees the lease once");
    const unsigned resolves=native.resolves;
    expect(draw() && native.resolves==resolves,"later drawing continues locally with no native traffic");
    disable();
    expect(native.resolves==resolves && native.legacy_releases==0 && device.ScreenPtr==nullptr &&
        native.frees[0]==1,"teardown neither repeats retirement nor sends a raw-address unmap");

    // Mode change whose old release acknowledgement is lost, then removal.
    expect(enable() && FramebufferShadowAssertMode(&device,FALSE)!=FALSE,"PDEV suspends for a mode change");
    native.faults.configure(1U);
    expect(FramebufferShadowAssertMode(&device,TRUE)!=FALSE && aliased(native.ram[1]) && !aliased(native.ram[0]),
        "refresh publishes the replacement although the old acknowledgement was lost");
    expect(native.frees[0]==1 && native.aliased_frees==0,"old RAM was freed by the applied release after its wrapper");
    expect(FramebufferShadowAssertMode(&device,FALSE)!=FALSE && FramebufferShadowAssertMode(&device,TRUE)!=FALSE,
        "next remap drains the pending token as not-held");
    expect(native.frees[0]==1 && native.frees[1]==1 && aliased(native.ram[2]),"no double free across the lost ack");
    native.abandon();
    expect(!draw() && no_lease_left() && native.aliased_frees==0 && each_freed_once(3),
        "removal after the lost ack retires every remaining lease exactly once");
    disable();
    expect(each_freed_once(3) && native.legacy_releases==0,"teardown is idempotent");

    // A failed disable: win32k keeps the PDEV enabled and never asserts it
    // again, so the owner must not stay suspended over a blank scanout.
    expect(enable(),"PDEV for a failed disable");
    native.fail_disable=true;
    expect(FramebufferShadowAssertMode(&device,FALSE)==FALSE,"failed native disable fails AssertMode(FALSE)");
    native.fail_disable=false;
    const unsigned before=native.presents;
    expect(native.enabled && aliased(native.ram[1]) && !aliased(native.ram[0]),
        "the owner re-enabled its own mode and remapped its staging");
    expect(draw() && native.presents>before,"drawing publishes again after the failed disable");
    expect(FramebufferShadowAssertMode(&device,FALSE)!=FALSE && FramebufferShadowAssertMode(&device,TRUE)!=FALSE,
        "a later mode change proceeds normally");
    disable();
    expect(native.ledger.held_leases()==0 && native.aliased_frees==0,"failed-disable teardown retires every lease");

    // Removal while suspended: no flush exists, AssertMode detects it.
    expect(enable() && FramebufferShadowAssertMode(&device,FALSE)!=FALSE,"PDEV is quiesced");
    native.abandon();
    native.faults.configure(1U<<StagingFaultPlan::drop_shift);
    expect(FramebufferShadowAssertMode(&device,TRUE)==FALSE,"resume after removal fails");
    expect(no_lease_left() && native.aliased_frees==0 && each_freed_once(1),
        "quiesced removal transfers despite a dropped first release");
    disable();

    // Removal while the release channel is down: bounded retry at teardown.
    expect(enable(),"PDEV for a channel failure");
    native.abandon();
    // Three transfer attempts, the teardown unmap and the first teardown
    // drain are dropped; the second bounded teardown drain succeeds.
    native.faults.configure(5U<<StagingFaultPlan::drop_shift);
    expect(!draw() && native.ledger.held_leases()==1 && !aliased(native.ram[0]),
        "views move even when every transfer release is dropped");
    disable();
    expect(no_lease_left() && native.aliased_frees==0 && each_freed_once(1),
        "teardown retries the unconfirmed token after the channel recovers");

    // Removal before the shadow owner binds.
    native=Native{};
    native.abandon();
    device.ScreenPtr=native.map(0);
    expect(device.ScreenPtr==nullptr || FramebufferShadowEnable(&device,bytes)==nullptr,
        "a removed owner never binds a PDEV");

    // Older miniport without resolve: legacy bounded behavior is preserved.
    expect(enable(true),"PDEV on a legacy owner");
    native.abandon();
    expect(!draw() && native.ledger.held_leases()==1,"legacy owner cannot report loss; lease stays bounded");
    disable();
    expect(native.legacy_releases==1 && native.ledger.held_leases()==0 && native.aliased_frees==0,
        "legacy teardown still deletes wrappers before its release");

    FramebufferShadowShutdown();
    expect(locks==0 && allocations==0 && device_surfaces==0,"all engine objects retire");
    for (const auto& bitmap:bitmaps) expect(!bitmap.used,"no engine bitmap leaks");
    printf("Lease transfer: %u checks, %u failures; GPU offload, native mutexes and pool not simulated\n",
        checks,failures);
    return failures==0 ? 0 : 1;
}

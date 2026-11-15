/*
Module: rx7900.reactos.framebuffer_shadow
Purpose: Draw into cached engine memory and flush bounded dirty pixels via GDI.
Created: 2026-10-21
Architecture: Optional framebuf display DDI implementation, no GPU acceleration.
License: GPL-2.0-or-later, matching ReactOS framebuf integration.
ReactOS engine contract inspected at a97c8a2cdcf6a9a7f5cc543951a02ccb7a80d8ca:
win32ss/gdi/eng/engmisc.c, copybits.c, bitblt.c, mouse.c, stubs.c.
This is an original implementation, not the pan driver's global/full-frame copy.
*/
#include "framebuf.h"
#include "framebuffer_shadow.hpp"
#include "framebuffer_shadow_policy.hpp"
#include "framebuffer_shadow_abi.hpp"
#include "lifetime.hpp"
#include "lightning_staging_pool.hpp"
#ifdef RX7900_LIGHTNING_DISPLAY
#include "lightning_display.hpp"
#endif

#include <stdint.h>
#ifdef RX7900_LIGHTNING_GDI_OFFLOAD
#include "lightning_gdi_client.hpp"
#include "framebuffer_text_batch.hpp"
#include "framebuffer_stretch_guard.hpp"
#include "engine_bitmap.hpp"
#include "brush_snapshot.hpp"
#include "lightning-opengl-present.h"
#include "lightning_present_request.hpp"
#include "lightning_opengl_icd.hpp"
#endif

namespace {
using rx7900::reactos::framebuffer::Rectangle;
namespace policy = rx7900::reactos::framebuffer;

struct ShadowState final {
    ShadowState* next = nullptr;
    HSURF display = nullptr;
    HSURF cached_bitmap = nullptr;
    HSURF physical_bitmap = nullptr;
    // Owned cache pixels where engine bitmaps would come from a too small
    // paged pool (lightning_staging_pool.hpp); null: the engine owns them.
    void* cached_pixels = nullptr;
    SURFOBJ* cached = nullptr;
    SURFOBJ* physical = nullptr;
    Rectangle dirty{};
    policy::ShadowStatistics statistics{};
    bool visible = true;
    bool mode_cache_retained=false;
#ifdef RX7900_LIGHTNING_GDI_OFFLOAD
    lightning::reactos::gdi::ClientCanvas gpu{};
    lightning::reactos::gdi::ClipBatch clip_workspace{};
    RX7900_TEXT_BATCH_CONTEXT* text_batch=nullptr;
#endif
#ifdef RX7900_LIGHTNING_DISPLAY
    lightning::reactos::DisplayLink link{};
    // Surprise removal moved the physical view off leased staging RAM.
    bool lease_transferred=false;
    // enable_owned_mode parked this PDEV; cleared by its next successful enable.
    bool parked=false;
#endif
};
static_assert(alignof(ShadowState) <= (sizeof(void*)==4 ? 8U : 16U),
              "EngAllocMem native pool storage alignment");

// Engine lifecycle serializes EnableDriver/DisableDriver. This registry's only
// job is safe ownership lookup for mixed-device DDIs: foreign dhpdev/dhsurf
// pointers are never interpreted as our PDEV. The semaphore holds state alive
// across bitmap operations and serializes draw/flush against surface retirement.
HSEMAPHORE registry_lock = nullptr;
ShadowState* registry = nullptr;
#ifdef RX7900_LIGHTNING_DISPLAY
// Driver-wide DrvAssertMode(TRUE) record (DisplayModeHealth); registry guard.
lightning::reactos::DisplayModeHealth mode_health{};
#endif

class RegistryGuard final {
public:
    RegistryGuard() noexcept { EngAcquireSemaphore(registry_lock); }
    ~RegistryGuard() { EngReleaseSemaphore(registry_lock); }
    RegistryGuard(const RegistryGuard&) = delete;
    RegistryGuard& operator=(const RegistryGuard&) = delete;
};

ShadowState* lookup(SURFOBJ* surface) noexcept {
    if (surface == nullptr || surface->iType == STYPE_BITMAP) return nullptr;
    for (ShadowState* state = registry; state != nullptr; state = state->next) {
        if (state->display == surface->hsurf) return state;
    }
    return nullptr;
}

// Bitmap inputs belong to GDI. Nonbitmap inputs must be owned registered
// surfaces; unsupported foreign devices fail without dereferencing private data
// or recursively dispatching into another driver while the registry is held.
SURFOBJ* resolve(SURFOBJ* surface, ShadowState*& owner) noexcept {
    owner = lookup(surface);
    if (owner != nullptr) return owner->cached;
    return surface == nullptr || surface->iType == STYPE_BITMAP ? surface : nullptr;
}

Rectangle rectangle_value(const RECTL& rectangle) noexcept {
    return {rectangle.left, rectangle.top, rectangle.right, rectangle.bottom};
}

Rectangle dirty_bounds(ShadowState& state, const RECTL& rectangle,
                       CLIPOBJ* clip) noexcept {
    Rectangle dirty = policy::clip(rectangle_value(rectangle),
        state.cached->sizlBitmap.cx, state.cached->sizlBitmap.cy);
    if (clip != nullptr && clip->iDComplexity != DC_TRIVIAL) {
        dirty = policy::intersect(dirty, rectangle_value(clip->rclBounds));
    }
    return dirty;
}

void add_ticks(ShadowState& state, LONGLONG begin, LONGLONG end, bool flush) noexcept {
    if (begin<0 || end<begin || state.statistics.qpc_frequency==0) {
        if (state.statistics.qpc_failures!=UINT64_MAX) ++state.statistics.qpc_failures;
        return;
    }
    const uint64_t elapsed=static_cast<uint64_t>(end-begin);
    uint64_t& total=flush ? state.statistics.flush_ticks : state.statistics.render_ticks;
    total=elapsed>UINT64_MAX-total ? UINT64_MAX : total+elapsed;
    if (flush && elapsed>state.statistics.max_flush_ticks) state.statistics.max_flush_ticks=elapsed;
}

#ifdef RX7900_LIGHTNING_DISPLAY
void transfer_lost_device(ShadowState& state) noexcept;
#endif

bool flush(ShadowState& state) noexcept {
    // GDI can retain/update the cached image while a console owns the adapter.
    // Preserve damage, but never write the physical mapping after AssertMode(FALSE).
    if (!state.visible) {
#ifdef RX7900_LIGHTNING_DISPLAY
        // Bounded retry of credentials left unconfirmed by a removal transfer.
        if (state.lease_transferred && state.link.retirement_pending()) state.link.transfer_lost_leases();
#endif
        return true;
    }
    if (policy::empty(state.dirty)) return true;
#ifdef RX7900_LIGHTNING_GDI_OFFLOAD
    if (!state.gpu.prepare(state.cached,state.physical) || !state.gpu.flush_glyphs()) return false;
#endif
    RECTL rectangle{state.dirty.left, state.dirty.top,
                    state.dirty.right, state.dirty.bottom};
    POINTL origin{rectangle.left, rectangle.top};
    // Both surfaces are plain engine bitmaps without driver hooks. Bounds have
    // been checked against stride, mapping length and both identical dimensions.
    LONGLONG begin=0;
    LONGLONG end=0;
    EngQueryPerformanceCounter(&begin);
    BOOL copied=EngCopyBits(state.physical,state.cached,nullptr,nullptr,&rectangle,&origin);
#ifdef RX7900_LIGHTNING_DISPLAY
    if (copied) {
#ifdef RX7900_LIGHTNING_GDI_OFFLOAD
        if (state.gpu.enabled()) copied=state.gpu.upload_cached(state.dirty) ? TRUE : FALSE;
        else
#endif
            copied=state.link.present(state.dirty) ? TRUE : FALSE;
    }
#endif
    EngQueryPerformanceCounter(&end);
    add_ticks(state,begin,end,true);
    if (!copied) {
#ifdef RX7900_LIGHTNING_DISPLAY
        transfer_lost_device(state);
#endif
        return false;
    }
    if (state.statistics.flush_calls!=UINT64_MAX) ++state.statistics.flush_calls;
    const uint64_t bytes=static_cast<uint64_t>(rectangle.right-rectangle.left)*
        static_cast<uint32_t>(rectangle.bottom-rectangle.top)*4U;
    state.statistics.flush_bytes=bytes>UINT64_MAX-state.statistics.flush_bytes ? UINT64_MAX :
        state.statistics.flush_bytes+bytes;
    state.dirty = {};
    return true;
}

#ifdef RX7900_LIGHTNING_GDI_OFFLOAD
bool readable(ShadowState* state) noexcept {
    return state==nullptr || !state->gpu.enabled() || state->gpu.synchronize(state->cached,state->physical);
}

lightning::reactos::gdi::Attempt offload(ShadowState* target,ShadowState* source_owner,
    SURFOBJ* bitmap_source,CLIPOBJ* clip,XLATEOBJ* translation,RECTL rectangle,
    POINTL* source_point,SURFOBJ* mask,BRUSHOBJ* brush,ROP4 rop,
    POINTL* mask_point=nullptr,POINTL* brush_origin=nullptr) noexcept {
    namespace gpu=lightning::reactos::gdi;
    if (target==nullptr || !target->visible) return gpu::Attempt::unsupported;
    if (!target->gpu.prepare(target->cached,target->physical)) return gpu::Attempt::failed;
    if (!target->gpu.enabled()) return gpu::Attempt::unsupported;
    if (target->gpu.poisoned() || !flush(*target)) return gpu::Attempt::failed;
    gpu::ClipBatch& batch=target->clip_workspace;
    if (!gpu::capture_clip(rectangle,clip,target->cached->sizlBitmap.cx,target->cached->sizlBitmap.cy,batch)) {
        return gpu::Attempt::unsupported;
    }
    uint32_t constant_color=0;
    if (gpu::constant_fill_rop(rop,constant_color))
        return target->gpu.fill(batch,constant_color,target->cached,clip);
    // NT ROP4 repeats the foreground/background ROP3 byte for unmasked copies.
    if (mask==nullptr && rop==0xf0f0U && brush!=nullptr && brush->iSolidColor!=UINT32_MAX) {
        return target->gpu.fill(batch,brush->iSolidColor,target->cached,clip);
    }
    const uint32_t foreground=rop&UINT8_MAX;
    const uint32_t background=(rop>>8)&UINT8_MAX;
    const bool pattern_needed=::lightning::gdi::rop3_uses_pattern(foreground) ||
        ::lightning::gdi::rop3_uses_pattern(background);
    if (mask!=nullptr || foreground!=background ||
        (pattern_needed && brush!=nullptr && brush->iSolidColor==UINT32_MAX)) {
        const bool direct_source=source_owner==target &&
            (translation==nullptr || (translation->flXlate&XO_TRIVIAL)!=0);
        if ((::lightning::gdi::rop3_uses_source(foreground) || ::lightning::gdi::rop3_uses_source(background)) &&
            !direct_source && !readable(source_owner)) return gpu::Attempt::failed;
        gpu::RasterInput input{};
        input.source=bitmap_source;
        input.translation=translation;
        input.original=rectangle;
        input.source_point=source_point;
        input.same_canvas=direct_source;
        input.mask=mask;
        input.mask_point=mask_point;
        input.brush=brush;
        input.brush_origin=brush_origin;
        input.rop4=rop;
        return target->gpu.raster_operands(batch,input,target->physical);
    }
    if (rop!=0xccccU) {
        const uint32_t rop3=rop&UINT8_MAX;
        if (rop!=(rop3|(rop3<<8))) return gpu::Attempt::unsupported;
        if (::lightning::gdi::rop3_uses_pattern(rop3) &&
            (brush==nullptr || brush->iSolidColor==UINT32_MAX)) return gpu::Attempt::unsupported;
        const bool direct_source=source_owner==target &&
            (translation==nullptr || (translation->flXlate&XO_TRIVIAL)!=0);
        if (::lightning::gdi::rop3_uses_source(rop3) && !direct_source && !readable(source_owner))
            return gpu::Attempt::failed;
        return target->gpu.raster(batch,rop3,brush==nullptr ? 0U : brush->iSolidColor,
            bitmap_source,translation,rectangle,source_point,direct_source,target->physical);
    }
    if (bitmap_source==nullptr || source_point==nullptr) return gpu::Attempt::unsupported;
    if (policy::empty(batch.bounds)) return gpu::Attempt::performed;
    if (source_owner==target && (translation==nullptr || (translation->flXlate&XO_TRIVIAL)!=0)) {
        int32_t source_x=0;
        int32_t source_y=0;
        if (!gpu::source_position(rectangle_value(rectangle),batch.bounds,source_point->x,source_point->y,
            bitmap_source->sizlBitmap.cx,bitmap_source->sizlBitmap.cy,source_x,source_y)) return gpu::Attempt::unsupported;
        return target->gpu.copy(batch,static_cast<uint32_t>(source_x),static_cast<uint32_t>(source_y));
    }
    if (!readable(source_owner)) return gpu::Attempt::failed;
    return target->gpu.bitmap(batch,bitmap_source,translation,rectangle,*source_point,
        target->physical,target->cached,clip);
}
#endif

bool physical_bitmap(PPDEV device,void* address,HSURF& bitmap,SURFOBJ*& surface) noexcept {
    const SIZEL size{static_cast<LONG>(device->ScreenWidth),static_cast<LONG>(device->ScreenHeight)};
    bitmap=reinterpret_cast<HSURF>(EngCreateBitmap(size,static_cast<LONG>(device->ScreenDelta),
                                                 BMF_32BPP,BMF_TOPDOWN,address));
    if (bitmap==nullptr) return false;
    surface=EngLockSurface(bitmap);
    if (surface!=nullptr) return true;
    EngDeleteSurface(bitmap);
    bitmap=nullptr;
    return false;
}

#ifdef RX7900_LIGHTNING_DISPLAY
bool remap_staging(ShadowState& state,PPDEV device) noexcept {
    void* address=nullptr;
    uint32_t bytes=0;
    if (!state.link.map_staging(address,bytes)) return false;
    HSURF bitmap=nullptr;
    SURFOBJ* surface=nullptr;
    const uintptr_t base=reinterpret_cast<uintptr_t>(address);
    const uint64_t extent=static_cast<uint64_t>(device->ScreenDelta)*device->ScreenHeight;
    if (address==nullptr || extent>UINTPTR_MAX-base ||
        !policy::valid_layout(device->ScreenWidth,device->ScreenHeight,device->ScreenDelta,bytes) ||
        !physical_bitmap(device,address,bitmap,surface)) {
        state.link.unmap_staging(address);
        return false;
    }
    // Build and authenticate the replacement before retiring the old wrapper.
    // Failure retains its mapped lease for a later owned-mode rollback/retry.
    if (!state.link.connect(device,address)) {
        EngUnlockSurface(surface);
        EngDeleteSurface(bitmap);
        state.link.unmap_staging(address);
        return false;
    }
    void* previous=device->ScreenPtr;
    EngUnlockSurface(state.physical);
    EngDeleteSurface(state.physical_bitmap);
    state.physical=surface;
    state.physical_bitmap=bitmap;
    device->ScreenPtr=address;
    // The replacement is already authenticated and installed. A failed old
    // release retains its exact token for retry; it must not hide the new PDEV.
    // The next MAP drains that token first and refuses further mutation while
    // native retirement remains uncertain.
    state.link.unmap_staging(previous);
    return true;
}
#endif

#ifdef RX7900_LIGHTNING_DISPLAY
// Surprise removal: native scanout no longer reads staging, but the physical
// wrapper and the GDI client's raster path still alias leased RAM. First move
// that view to a private engine bitmap of identical geometry (the client's
// raster path then fails closed on its pvScan0 check), and only then retire
// every owned, candidate and pending credential so the miniport can free the
// RAM. The cached bitmap stays authoritative; GDI keeps drawing unpublished.
void transfer_lost_device(ShadowState& state) noexcept {
    if (state.lease_transferred || state.physical==nullptr || !state.link.device_lost()) return;
    state.visible=false;
#ifdef RX7900_LIGHTNING_GDI_OFFLOAD
    state.gpu.retire();
#endif
    HSURF bitmap=reinterpret_cast<HSURF>(EngCreateBitmap(state.physical->sizlBitmap,
        state.physical->lDelta,BMF_32BPP,BMF_TOPDOWN,nullptr));
    SURFOBJ* surface=bitmap==nullptr ? nullptr : EngLockSurface(bitmap);
    if (surface==nullptr) {
        // Without a private view the leased RAM stays aliased: retain every
        // credential and retry on the next failed flush.
        if (bitmap!=nullptr) EngDeleteSurface(bitmap);
        return;
    }
    EngUnlockSurface(state.physical);
    EngDeleteSurface(state.physical_bitmap);
    state.physical=surface;
    state.physical_bitmap=bitmap;
    state.lease_transferred=true;
    state.link.transfer_lost_leases();
}
#endif

void release(ShadowState& state,void* final_mapping=nullptr) noexcept {
#ifdef RX7900_LIGHTNING_GDI_OFFLOAD
    state.gpu.retire();
    state.gpu.release_cpu_storage();
#endif
    // GDI retires display callbacks before DisableSurface. RegistryGuard also
    // excludes any in-flight driver operations; bitmap locks retire before delete.
    if (state.display != nullptr) EngDeleteSurface(state.display);
    if (state.physical != nullptr) EngUnlockSurface(state.physical);
    if (state.cached != nullptr) EngUnlockSurface(state.cached);
    if (state.physical_bitmap != nullptr) EngDeleteSurface(state.physical_bitmap);
    if (state.cached_bitmap != nullptr) EngDeleteSurface(state.cached_bitmap);
    // The engine bitmap over these pixels is gone; nothing else aliases them.
    if (state.cached_pixels != nullptr) EngFreeMem(state.cached_pixels);
#ifdef RX7900_LIGHTNING_DISPLAY
    // No engine wrapper may retain RAM when its native lease is released.
    // Initial Enable failures leave the final mapping to upstream cleanup;
    // DisableSurface transfers it here and prevents a legacy address retry.
    if (final_mapping!=nullptr) state.link.unmap_staging(final_mapping);
    state.link.retire();
#else
    (void)final_mapping;
#endif
    EngFreeMem(&state);
}
} // namespace

extern "C" BOOL APIENTRY FramebufferShadowInitialize(VOID) {
    if (registry_lock != nullptr) return TRUE;
    registry_lock = EngCreateSemaphore();
    return registry_lock != nullptr;
}

extern "C" VOID APIENTRY FramebufferShadowShutdown(VOID) {
    // Engine lifecycle guarantees all PDEVs/surfaces have retired first.
    if (registry != nullptr || registry_lock == nullptr) return;
    EngDeleteSemaphore(registry_lock);
    registry_lock = nullptr;
}

extern "C" HSURF APIENTRY FramebufferShadowEnable(PPDEV device, ULONG mapped_bytes) {
    if (registry_lock == nullptr || device == nullptr ||
        device->ShadowState != nullptr || device->BitsPerPixel != 32 ||
        device->ScreenPtr == nullptr ||
        !policy::valid_layout(device->ScreenWidth, device->ScreenHeight,
                              device->ScreenDelta, mapped_bytes)) return nullptr;
    const uintptr_t address = reinterpret_cast<uintptr_t>(device->ScreenPtr);
    const uint64_t extent = static_cast<uint64_t>(device->ScreenDelta) * device->ScreenHeight;
    if (extent > UINTPTR_MAX - address) return nullptr;

    void* storage = EngAllocMem(0, sizeof(ShadowState), ALLOC_TAG);
    if (storage == nullptr) return nullptr;
    ShadowState* state = rx7900::reactos::construct_device_state<ShadowState>(storage);
#ifdef RX7900_LIGHTNING_DISPLAY
    if (!state->link.connect(device)) {
        release(*state);
        return nullptr;
    }
#endif
    LONGLONG frequency=0;
    EngQueryPerformanceFrequency(&frequency);
    if (frequency>0) state->statistics.qpc_frequency=static_cast<uint64_t>(frequency);
    const SIZEL size{static_cast<LONG>(device->ScreenWidth),
                     static_cast<LONG>(device->ScreenHeight)};
    const LONG stride = static_cast<LONG>(device->ScreenDelta);
    // An engine-allocated bitmap is one PagedPool object of the full frame.
    if (lightning::reactos::frame_pool(sizeof(void*)) == lightning::reactos::FramePool::nonpaged) {
        state->cached_pixels = EngAllocMem(FL_ZERO_MEMORY | FL_NONPAGED_MEMORY,
            static_cast<ULONG>(extent), ALLOC_TAG);
        if (state->cached_pixels == nullptr) {
            release(*state);
            return nullptr;
        }
    }
    state->cached_bitmap = reinterpret_cast<HSURF>(
        EngCreateBitmap(size, stride, BMF_32BPP, BMF_TOPDOWN, state->cached_pixels));
    if (state->cached_bitmap != nullptr) state->cached = EngLockSurface(state->cached_bitmap);
    physical_bitmap(device,device->ScreenPtr,state->physical_bitmap,state->physical);
    if (state->cached == nullptr || state->physical == nullptr) {
        release(*state);
        return nullptr;
    }
#ifdef RX7900_LIGHTNING_GDI_OFFLOAD
    if (state->gpu.initialize(device)==lightning::reactos::gdi::Attempt::failed) {
        release(*state);
        return nullptr;
    }
#endif
    state->display = EngCreateDeviceSurface(reinterpret_cast<DHSURF>(state), size, BMF_32BPP);
    if (state->display == nullptr ||
        !EngAssociateSurface(state->display, device->hDevEng, HOOK_COPYBITS | HOOK_BITBLT
#ifdef RX7900_LIGHTNING_GDI_OFFLOAD
            | HOOK_ALPHABLEND | HOOK_STRETCHBLTROP | HOOK_TRANSPARENTBLT | HOOK_LINETO | HOOK_PAINT | HOOK_GRADIENTFILL
#endif
            )) {
        release(*state);
        return nullptr;
    }

    RegistryGuard guard;
    state->dirty = {0, 0, size.cx, size.cy};
    if (!flush(*state)) {
        release(*state);
        return nullptr;
    }
    state->next = registry;
    registry = state;
    device->ShadowState = state;
    return state->display;
}

extern "C" VOID APIENTRY FramebufferShadowDisable(PPDEV device) {
    if (registry_lock == nullptr || device == nullptr) return;
    RegistryGuard guard;
    ShadowState** link = &registry;
    while (*link != nullptr && *link != device->ShadowState) link = &(*link)->next;
    if (*link == nullptr) return;
    ShadowState* state = *link;
    *link = state->next;
    device->ShadowState = nullptr;
#ifdef RX7900_LIGHTNING_DISPLAY
    release(*state,device->ScreenPtr);
    device->ScreenPtr=nullptr;
#else
    release(*state);
#endif
}

namespace {
// The registry guard owns callback exclusion and keeps every bitmap/lease alive.
bool refresh_shadow(ShadowState& state,PPDEV device) noexcept {
#ifdef RX7900_LIGHTNING_DISPLAY
    if (!remap_staging(state,device)) {
        transfer_lost_device(state);
        return false;
    }
#endif
#ifdef RX7900_LIGHTNING_GDI_OFFLOAD
    if (state.gpu.reinitialize_after_mode(device)==lightning::reactos::gdi::Attempt::failed) return false;
#endif
    state.dirty={0,0,state.cached->sizlBitmap.cx,state.cached->sizlBitmap.cy};
    state.visible=true;
    if (flush(state)
#ifdef RX7900_LIGHTNING_DISPLAY
        && state.link.restore_cursor()
#endif
        ) {
        state.mode_cache_retained=false;
        return true;
    }
    state.visible=false;
    return false;
}

bool suspend_shadow(ShadowState& state) noexcept {
#ifdef RX7900_LIGHTNING_DISPLAY
    if (!state.link.current_owner(false)) {
        transfer_lost_device(state);
        state.visible=false;
        return false;
    }
    if (!state.link.suspend_cursor()) {state.visible=false; return false;}
#endif
#ifdef RX7900_LIGHTNING_GDI_OFFLOAD
    if (!readable(&state)) {state.visible=false; return false;}
    if (!state.gpu.retire()) {state.visible=false; return false;}
#endif
    state.mode_cache_retained=true;
    state.visible=false;
    return true;
}

#ifdef RX7900_LIGHTNING_DISPLAY
// After this PDEV's own accepted native mode commit. That commit reset any
// native GPU canvas and hid/undefined the native cursor plane, and the cache
// retained at suspend is complete. Only the authenticated staging remap and
// the CPU publication are mandatory; the GPU canvas and the hardware cursor
// are optional and degrade instead of failing the enable.
lightning::reactos::EnableStep refresh_owned_mode(ShadowState& state,PPDEV device) noexcept {
    using lightning::reactos::EnableStep;
    if (!remap_staging(state,device)) {
        transfer_lost_device(state);
        return EnableStep::remap;
    }
#ifdef RX7900_LIGHTNING_GDI_OFFLOAD
    if (state.gpu.reinitialize_after_mode(device)==lightning::reactos::gdi::Attempt::failed) {
        state.gpu.demote_to_cpu();
        mode_health.flags|=lightning::reactos::mode_health_gpu_demoted;
    }
#endif
    const Rectangle full{0,0,state.cached->sizlBitmap.cx,state.cached->sizlBitmap.cy};
    state.dirty=full;
    state.visible=true;
    bool flushed=flush(state);
#ifdef RX7900_LIGHTNING_GDI_OFFLOAD
    // A failed canvas upload leaves the cache untouched. Publish it through
    // the CPU path; the demoted canvas is never read or replayed.
    if (!flushed && !state.lease_transferred && (state.gpu.enabled() || state.gpu.poisoned())) {
        state.gpu.demote_to_cpu();
        mode_health.flags|=lightning::reactos::mode_health_gpu_demoted;
        state.dirty=full;
        state.visible=true;
        flushed=flush(state);
    }
#endif
    if (!flushed) {
        state.visible=false;
        return EnableStep::flush;
    }
    if (!state.link.restore_cursor()) {
        state.link.abandon_cursor_restore();
        mode_health.flags|=lightning::reactos::mode_health_cursor_dropped;
    }
    state.mode_cache_retained=false;
    return EnableStep::none;
}

lightning::reactos::EnableStep attempt_enable(ShadowState& state,PPDEV device) noexcept {
    using lightning::reactos::EnableStep;
    // Suspend/read back only our own active generation. A previously
    // suspended PDEV may request the native owner's narrow rollback path;
    // read-only inventory alone never grants it a replacement token.
    if (state.visible && !suspend_shadow(state)) return EnableStep::suspend;
#ifdef RX7900_LIGHTNING_GDI_OFFLOAD
    if ((state.gpu.enabled() || state.gpu.poisoned()) && !state.mode_cache_retained) return EnableStep::suspend;
#endif
    if (!state.link.transition_mode(device->ModeIndex,true)) {
        // A suspended PDEV performs no flush; removal is detected here.
        transfer_lost_device(state);
        return EnableStep::transition;
    }
    return refresh_owned_mode(state,device);
}

// win32k PDEVOBJ_vEnableDisplay (win32ss/gdi/eng/pdevobj.c, pinned a97c8a2
// and 822e864) repeats DrvAssertMode(TRUE) until it returns TRUE while it
// holds ghsemPDEV and the PDEV lock. A FALSE that cannot change is therefore a
// system-wide GDI hang, and every retry used to recommit, clear and republish
// the whole scanout. Only mandatory steps fail an attempt; after
// enable_attempt_limit consecutive failures the PDEV is parked: TRUE to the
// engine, not visible, nothing published, recorded in DisplayModeHealth. Its
// next owned transition, or teardown, starts over from the retained cache.
bool enable_owned_mode(ShadowState& state,PPDEV device) noexcept {
    namespace lr=lightning::reactos;
    auto& health=mode_health;
    if (health.enables!=UINT32_MAX) ++health.enables;
    // win32k asserts the PDEV it just swapped in after DrvEnableSurface
    // committed and mapped its mode. Recommitting that live owner would only
    // clear its scanout and reset its canvas.
    if (state.visible && state.link.current_owner(true) && state.link.mode_index()==device->ModeIndex) {
        health.consecutive_failures=0;
        health.mode_generation=state.link.mode_generation();
        state.parked=false;
        return true;
    }
    const lr::EnableStep failed=attempt_enable(state,device);
    health.mode_generation=state.link.mode_generation();
    if (failed==lr::EnableStep::none) {
        health.consecutive_failures=0;
        state.parked=false;
        return true;
    }
    if (health.failed_attempts!=UINT32_MAX) ++health.failed_attempts;
    ++health.consecutive_failures;
    health.last_failed_step=static_cast<uint32_t>(failed);
    health.last_error=state.link.last_error();
    // lightningdisp imports win32k only (no DbgPrint): the escape is the log.
    if (health.consecutive_failures<lr::enable_attempt_limit) return false;
    state.visible=false;
    state.parked=true;
    health.flags|=lr::mode_health_parked;
    health.consecutive_failures=0;
    return true;
}
#endif
} // namespace

extern "C" BOOL APIENTRY FramebufferShadowRefresh(PPDEV device) {
    if (registry_lock==nullptr || device==nullptr) return FALSE;
    RegistryGuard guard;
    for (ShadowState* state=registry;state!=nullptr;state=state->next) {
        if (state==device->ShadowState) return refresh_shadow(*state,device) ? TRUE : FALSE;
    }
    return FALSE;
}

extern "C" BOOL APIENTRY FramebufferShadowSuspend(PPDEV device) {
    if (registry_lock==nullptr || device==nullptr) return FALSE;
    RegistryGuard guard;
    for (ShadowState* state=registry;state!=nullptr;state=state->next) {
        if (state==device->ShadowState) return suspend_shadow(*state) ? TRUE : FALSE;
    }
    return FALSE;
}

#ifdef RX7900_LIGHTNING_DISPLAY
extern "C" BOOL APIENTRY FramebufferShadowAssertMode(PPDEV device,BOOL enable) {
    if (registry_lock==nullptr || device==nullptr) return FALSE;
    RegistryGuard guard;
    for (ShadowState* state=registry;state!=nullptr;state=state->next) {
        if (state!=device->ShadowState) continue;
        if (enable!=FALSE) return enable_owned_mode(*state,device) ? TRUE : FALSE;
        // Suspend/read back only our own active generation.
        bool transitioned=!state->visible || suspend_shadow(*state);
#ifdef RX7900_LIGHTNING_GDI_OFFLOAD
        if ((state->gpu.enabled() || state->gpu.poisoned()) && !state->mode_cache_retained) transitioned=false;
#endif
        if (transitioned && !state->link.transition_mode(device->ModeIndex,false)) {
            // A suspended PDEV performs no flush; removal is detected here.
            transfer_lost_device(*state);
            transitioned=false;
        }
        if (!transitioned) {
            // win32k keeps a PDEV whose disable failed enabled and never
            // asserts it again; undo this call's suspend with one owned
            // re-enable under our own token rather than stay frozen.
            if (state->link.transition_mode(device->ModeIndex,true)) refresh_owned_mode(*state,device);
            return FALSE;
        }
        return TRUE;
    }
    return FALSE;
}
#endif

namespace {
#ifdef RX7900_LIGHTNING_GDI_OFFLOAD
bool accelerated_opengl_available(SURFOBJ* surface) noexcept {
    if (registry_lock == nullptr) return false;
    RegistryGuard guard;
    ShadowState* state = lookup(surface);
    lightning::reactos::gdi::Statistics statistics{};
    return state != nullptr && state->visible && state->gpu.statistics(statistics) &&
        statistics.enabled != 0 && statistics.poisoned == 0 &&
        (statistics.device.capabilities & LIGHTNING_GDI_CAP_EXTERNAL_IMAGE) != 0;
}

bool asynchronous_opengl_available(SURFOBJ* surface) noexcept {
    if (registry_lock == nullptr) return false;
    RegistryGuard guard;
    ShadowState* state = lookup(surface);
    return state != nullptr && state->visible && state->gpu.async_external();
}
#endif
}

extern "C" ULONG APIENTRY FramebufferShadowEscape(SURFOBJ* surface, ULONG escape,
    ULONG input_bytes, PVOID input, ULONG output_bytes, PVOID output) {
    (void)input;
#ifdef RX7900_LIGHTNING_GDI_OFFLOAD
    if (escape==QUERYESCSUPPORT && input_bytes==sizeof(uint32_t) && input!=nullptr &&
        registry_lock!=nullptr) {
        uint32_t requested=0;
        memcpy(&requested,input,sizeof(requested));

        if (requested==LIGHTNING_OPENGL_PRESENT_ESCAPE || requested==OPENGL_GETINFO)
            return accelerated_opengl_available(surface) ? 1U : 0U;
        // async-present: revision-2 presents and receipts need the native
        // stack's per-image pins; an older filter/miniport reports none.
        if (requested==LIGHTNING_OPENGL_RECEIPT_ESCAPE)
            return accelerated_opengl_available(surface) && asynchronous_opengl_available(surface) ? 1U : 0U;
    }
    if (escape==LIGHTNING_OPENGL_RECEIPT_ESCAPE && input_bytes==sizeof(LightningOpenGLReceiptQuery) &&
        input!=nullptr && output!=nullptr && output_bytes>=sizeof(LightningOpenGLReceipt) &&
        registry_lock!=nullptr) {
        static_assert(sizeof(LightningOpenGLReceiptQuery)==LIGHTNING_OPENGL_RECEIPT_QUERY_BYTES &&
            sizeof(LightningOpenGLReceipt)==LIGHTNING_OPENGL_RECEIPT_BYTES, "Fixed receipt escape ABI");
        static_assert(lightning::reactos::gdi::external_receipt_failed==LIGHTNING_OPENGL_RECEIPT_FAILED &&
            lightning::reactos::gdi::external_receipt_retired==LIGHTNING_OPENGL_RECEIPT_RETIRED &&
            lightning::reactos::gdi::external_receipt_pending==LIGHTNING_OPENGL_RECEIPT_PENDING,
            "Native and window receipt states agree");
        LightningOpenGLReceiptQuery query{};
        memcpy(&query,input,sizeof(query));
        uint32_t status=LIGHTNING_OPENGL_RECEIPT_FAILED;

        if (query.size!=sizeof(query) || query.version!=LIGHTNING_OPENGL_RECEIPT_VERSION || query.lease==0) return 0;
        {
            // Never waits for the GPU: the native filter only observes it.
            RegistryGuard guard;
            ShadowState* state=lookup(surface);
            if (state==nullptr || !state->gpu.external_receipt(query.lease,status)) return 0;
        }
        const LightningOpenGLReceipt receipt{sizeof(receipt),LIGHTNING_OPENGL_RECEIPT_VERSION,status,0};
        memcpy(output,&receipt,sizeof(receipt));
        return sizeof(receipt);
    }
    if (escape==OPENGL_GETINFO && input_bytes==sizeof(uint32_t) && input!=nullptr &&
        output!=nullptr && output_bytes>=sizeof(lightning::reactos::opengl::DriverInfo)) {
        uint32_t query=UINT32_MAX;
        memcpy(&query,input,sizeof(query));
        if (query!=lightning::reactos::opengl::query_driver_name ||
            !accelerated_opengl_available(surface)) return 0;
        return lightning::reactos::opengl::write_driver_info(output,output_bytes) ?
            sizeof(lightning::reactos::opengl::DriverInfo) : 0U;
    }
    if (escape==RX7900_TEXT_BATCH_QUERY && input_bytes==0 && output!=nullptr &&
        output_bytes>=sizeof(RX7900_TEXT_BATCH_CAPABILITY) && registry_lock!=nullptr) {
        RegistryGuard guard;
        if (lookup(surface)==nullptr) return 0;
        const RX7900_TEXT_BATCH_CAPABILITY capability{sizeof(capability),RX7900_TEXT_BATCH_VERSION,
            RX7900_TEXT_BATCH_MAGIC,sizeof(RX7900_TEXT_BATCH_CONTEXT)};
        memcpy(output,&capability,sizeof(capability));
        return sizeof(capability);
    }
    if (escape==lightning::reactos::gdi::query_statistics_escape && input_bytes==0 && output!=nullptr &&
        output_bytes>=sizeof(lightning::reactos::gdi::Statistics) && registry_lock!=nullptr) {
        RegistryGuard guard;
        ShadowState* state=lookup(surface);
        lightning::reactos::gdi::Statistics result{};
        if (state==nullptr || !state->gpu.statistics(result)) return 0;
        memcpy(output,&result,sizeof(result));
        return sizeof(result);
    }
#endif
#ifdef RX7900_LIGHTNING_DISPLAY
    if (escape==lightning::reactos::query_cursor_escape && input_bytes==0 && output!=nullptr &&
        output_bytes>=sizeof(lightning::reactos::CursorStatistics) && registry_lock!=nullptr) {
        RegistryGuard guard;
        ShadowState* state=lookup(surface);
        lightning::reactos::CursorStatistics result{};
        if (state==nullptr || !state->link.query_cursor(result)) return 0;
        if (!state->visible) result.recovery_required=0;
        memcpy(output,&result,sizeof(result));
        return sizeof(result);
    }
    if (escape==lightning::reactos::query_mode_health_escape && input_bytes==0 && output!=nullptr &&
        output_bytes>=sizeof(lightning::reactos::DisplayModeHealth) && registry_lock!=nullptr) {
        RegistryGuard guard;
        ShadowState* state=lookup(surface);
        if (state==nullptr) return 0;
        lightning::reactos::DisplayModeHealth result=mode_health;
        if (state->parked) result.flags|=lightning::reactos::mode_health_parked_now;
        result.size=sizeof(result);
        result.version=lightning::reactos::mode_health_version;
        result.magic=lightning::reactos::mode_health_magic;
        memcpy(output,&result,sizeof(result));
        return sizeof(result);
    }
    if (escape==lightning::reactos::query_display_escape && input_bytes==0 && output!=nullptr &&
        output_bytes>=sizeof(lightning::reactos::CpuScanoutInformation) && registry_lock!=nullptr) {
        RegistryGuard guard;
        ShadowState* state=lookup(surface);
        lightning::reactos::CpuScanoutInformation result{};
        if (state==nullptr || !state->link.query(result)) return 0;
        memcpy(output,&result,sizeof(result));
        return sizeof(result);
    }
#endif
    if (escape!=policy::query_shadow_escape || input_bytes!=0 || output==nullptr ||
        output_bytes<sizeof(policy::ShadowStatistics) || registry_lock==nullptr) return 0;
    RegistryGuard guard;
    policy::ShadowStatistics result{};
    ShadowState* state=lookup(surface);
    if (state!=nullptr) {
        result=state->statistics;
        result.active=state->visible ? 1U : 0U;
        result.width=state->cached->sizlBitmap.cx;
        result.height=state->cached->sizlBitmap.cy;
        result.cached_stride=state->cached->lDelta;
        result.physical_stride=state->physical->lDelta;
    }
    result.size=sizeof(result);
    result.version=policy::shadow_statistics_version;
    result.magic=policy::shadow_statistics_magic;
    // GDI probes and owns the declared output buffer for this synchronous DDI.
    // Byte copying supports any native buffer alignment and publishes no pointers.
    memcpy(output,&result,sizeof(result));
    return sizeof(result);
}

#ifdef RX7900_LIGHTNING_GDI_OFFLOAD
extern "C" VOID APIENTRY FramebufferShadowNotify(SURFOBJ* surface,ULONG notification,PVOID data) {
    // Preserve ordinary DN_* behavior. Only the positively identified private
    // kernel text contract borrows our cache; user Escape never mutates it.
    if (notification==RX7900_TEXT_BATCH_GLYPH) {
        auto* request=static_cast<RX7900_TEXT_GLYPH_REQUEST*>(data);
        if (registry_lock==nullptr || surface==nullptr || request==nullptr ||
            request->size!=sizeof(*request) || request->version!=RX7900_TEXT_BATCH_VERSION) return;
        auto* context=request->context;
        if (context==nullptr || context->size!=sizeof(*context) || context->version!=RX7900_TEXT_BATCH_VERSION ||
            context->magic!=RX7900_TEXT_BATCH_MAGIC || context->active==0 || context->original!=surface) return;
        // BEGIN owns the semaphore for this private synchronous callback. All
        // CPU glyph writes must reach the GPU before the next GPU glyph.
        ShadowState* state=lookup(surface);
        request->result=RX7900_TEXT_GLYPH_FAILED;
        if (state==nullptr || state!=context->owner || state->text_batch!=context ||
            state->cached!=context->cached || !state->visible) return;
        state->dirty=policy::unite(state->dirty,policy::clip(rectangle_value(context->dirty),
            state->cached->sizlBitmap.cx,state->cached->sizlBitmap.cy));
        if (!flush(*state)) return;
        context->dirty={};
        lightning::reactos::gdi::ClipBatch& batch=state->clip_workspace;
        if (!lightning::reactos::gdi::capture_clip(request->destination,request->clip,
            state->cached->sizlBitmap.cx,state->cached->sizlBitmap.cy,batch)) {
            if (readable(state)) request->result=RX7900_TEXT_GLYPH_UNSUPPORTED;
            return;
        }
        const auto result=state->gpu.queue_glyph(batch,request->mask,request->destination,
            request->mask_origin,request->brush,state->physical,state->cached);
        if (result==lightning::reactos::gdi::Attempt::performed) {
            request->result=RX7900_TEXT_GLYPH_PERFORMED;
        } else if (result==lightning::reactos::gdi::Attempt::unsupported && readable(state)) {
            state->gpu.fallback();
            request->result=RX7900_TEXT_GLYPH_UNSUPPORTED;
        }
        return;
    }
    if (notification!=RX7900_TEXT_BATCH_BEGIN && notification!=RX7900_TEXT_BATCH_END) return;
    auto* context=static_cast<RX7900_TEXT_BATCH_CONTEXT*>(data);
    if (registry_lock==nullptr || context==nullptr || surface==nullptr ||
        context->size!=sizeof(*context) || context->version!=RX7900_TEXT_BATCH_VERSION ||
        context->magic!=RX7900_TEXT_BATCH_MAGIC || context->original!=surface) return;
    if (notification==RX7900_TEXT_BATCH_BEGIN) {
        EngAcquireSemaphore(registry_lock);
        ShadowState* state=lookup(surface);
        if (state==nullptr || !state->visible) {EngReleaseSemaphore(registry_lock); return;}
        if (!state->gpu.prepare(state->cached,state->physical) || !flush(*state)) {
            context->result=FALSE;
            EngReleaseSemaphore(registry_lock);
            return;
        }
        if (!state->gpu.enabled()) {EngReleaseSemaphore(registry_lock); return;}
        if (!state->gpu.begin_text()) {
            context->result=FALSE;
            EngReleaseSemaphore(registry_lock);
            return;
        }
        state->text_batch=context;
        context->owner=state;
        context->cached=state->cached;
        context->active=TRUE;
        // END releases exactly this semaphore after the existing glyph loop.
        return;
    }
    ShadowState* state=lookup(surface);
    if (state==nullptr || state!=context->owner || state->text_batch!=context ||
        state->cached!=context->cached || context->active==0) return;
    const policy::Rectangle attempted=policy::clip(rectangle_value(context->dirty),
        state->cached->sizlBitmap.cx,state->cached->sizlBitmap.cy);
    state->dirty=policy::unite(state->dirty,attempted);
    const bool published=state->gpu.prepare(state->cached,state->physical) &&
        state->gpu.flush_glyphs() && flush(*state);
    if (!published) state->gpu.abort_text();
    const bool closed=published && state->gpu.end_text();
    context->result=context->result!=FALSE && published && closed ? TRUE : FALSE;
    state->text_batch=nullptr;
    context->owner=nullptr;
    context->cached=nullptr;
    context->active=FALSE;
    EngReleaseSemaphore(registry_lock);
}

extern "C" BOOL APIENTRY FramebufferShadowRealizeBrush(BRUSHOBJ* brush,SURFOBJ* target,
    SURFOBJ* pattern,SURFOBJ* mask,XLATEOBJ* translation,ULONG hatch) {
    if (brush==nullptr || brush->pvRbrush!=nullptr || target==nullptr || target->iBitmapFormat!=BMF_32BPP ||
        pattern==nullptr || (mask!=nullptr && mask->iBitmapFormat!=BMF_1BPP)) return FALSE;
    // The engine owns the brush and all borrowed surfaces under its PDEV lock.
    // No registry acquisition here: lazy realization can be requested while
    // the caller already holds that guard. The public realization callback
    // supplies any hatch mask; no engine-private brush layout is inspected.
    lightning::reactos::EngineBitmap normalized;
    lightning::reactos::EngineBitmap normalized_mask;
    if (!normalized.copy(pattern,pattern->sizlBitmap.cx,pattern->sizlBitmap.cy,BMF_32BPP,translation,
        {0,0},LIGHTNING_GDI_MAX_PIXEL_BYTES) || normalized.bytes()>UINT32_MAX-sizeof(lightning::reactos::BrushSnapshot))
        return FALSE;
    if (mask!=nullptr && !normalized_mask.copy(mask,mask->sizlBitmap.cx,mask->sizlBitmap.cy,BMF_1BPP,
        nullptr,{0,0},LIGHTNING_GDI_MAX_PIXEL_BYTES)) return FALSE;
    // The pinned engine supplies the stock hatch bitmap itself as the mask:
    // its zero bits are ink, while our canonical mask selects ink with one.
    // Only transform the owned mask; retain color translation and arbitrary
    // non-hatch masks exactly as supplied by the engine.
    if (mask!=nullptr && !lightning::reactos::normalize_hatch_mask(
        static_cast<uint8_t*>(normalized_mask.surface()->pvScan0),normalized_mask.bytes(),
        normalized_mask.stride(),static_cast<uint32_t>(mask->sizlBitmap.cx),
        static_cast<uint32_t>(mask->sizlBitmap.cy),hatch)) return FALSE;
    const uint64_t total=sizeof(lightning::reactos::BrushSnapshot)+static_cast<uint64_t>(normalized.bytes())+
        normalized_mask.bytes();
    if (total>UINT32_MAX || total>LIGHTNING_GDI_MAX_PIXEL_BYTES) return FALSE;
    const uint32_t bytes=static_cast<uint32_t>(total);
    void* allocation=BRUSHOBJ_pvAllocRbrush(brush,bytes);
    if (allocation==nullptr) return FALSE;
    auto* snapshot=rx7900::reactos::construct_device_state<lightning::reactos::BrushSnapshot>(allocation);
    snapshot->size=bytes;
    snapshot->width=static_cast<uint32_t>(pattern->sizlBitmap.cx);
    snapshot->height=static_cast<uint32_t>(pattern->sizlBitmap.cy);
    snapshot->stride=normalized.stride();
    snapshot->pixel_bytes=normalized.bytes();
    if (mask!=nullptr) {
        snapshot->mask_width=static_cast<uint32_t>(mask->sizlBitmap.cx);
        snapshot->mask_height=static_cast<uint32_t>(mask->sizlBitmap.cy);
        snapshot->mask_stride=normalized_mask.stride();
        snapshot->mask_bytes=normalized_mask.bytes();
    }
    uint32_t offset=sizeof(*snapshot);
    return normalized.append(allocation,bytes,offset) &&
        (mask==nullptr || normalized_mask.append(allocation,bytes,offset)) && offset==bytes;
}
#endif

extern "C" BOOL APIENTRY FramebufferShadowCopyBits(SURFOBJ* target, SURFOBJ* source,
    CLIPOBJ* clip, XLATEOBJ* translation, RECTL* rectangle, POINTL* source_point) {
    if (registry_lock == nullptr || target == nullptr || source == nullptr ||
        rectangle == nullptr || source_point == nullptr) return FALSE;
    RegistryGuard guard;
    ShadowState* destination = nullptr;
    ShadowState* source_owner = nullptr;
    SURFOBJ* bitmap_target = resolve(target, destination);
    SURFOBJ* bitmap_source = resolve(source, source_owner);
    if (bitmap_target == nullptr || bitmap_source == nullptr) return FALSE;
    ShadowState* counted=destination!=nullptr ? destination : source_owner;
    if (counted!=nullptr) {
        if (counted->statistics.copy_calls!=UINT64_MAX) ++counted->statistics.copy_calls;
        if (destination==nullptr && counted->statistics.source_only_calls!=UINT64_MAX) {
            ++counted->statistics.source_only_calls;
        }
    }
    const RECTL requested = *rectangle;
#ifdef RX7900_LIGHTNING_GDI_OFFLOAD
    const auto attempted=offload(destination,source_owner,bitmap_source,clip,translation,requested,
                                 source_point,nullptr,nullptr,0xccccU);
    if (attempted!=lightning::reactos::gdi::Attempt::unsupported) return attempted==lightning::reactos::gdi::Attempt::performed;
    if (!readable(destination) || !readable(source_owner)) return FALSE;
    if (destination!=nullptr && destination->gpu.enabled()) destination->gpu.fallback();
#endif
    RECTL operation = requested;
    POINTL origin = *source_point;
#ifdef RX7900_LIGHTNING_GDI_OFFLOAD
    lightning::reactos::EngineBitmap preserved_source;
    if (!preserved_source.preserve_clipped_copy(bitmap_target,bitmap_source,clip,operation,origin)) return FALSE;
#endif
    LONGLONG begin=0;
    LONGLONG end=0;
    EngQueryPerformanceCounter(&begin);
    const BOOL rendered = EngCopyBits(bitmap_target, bitmap_source, clip,
                                      translation, &operation, &origin);
    EngQueryPerformanceCounter(&end);
    if (counted!=nullptr) add_ticks(*counted,begin,end,false);
    if (destination == nullptr) {
        if (!rendered && counted!=nullptr && counted->statistics.failed_calls!=UINT64_MAX) {
            ++counted->statistics.failed_calls;
        }
        return rendered;
    }
    // Engines may fail after partial drawing. Conservatively flush attempted
    // bounds even on failure; keep them dirty if physical copying also fails.
    destination->dirty = policy::unite(destination->dirty,
                                       dirty_bounds(*destination, requested, clip));
    const bool copied = flush(*destination);
    if ((!rendered || !copied) && destination->statistics.failed_calls!=UINT64_MAX) {
        ++destination->statistics.failed_calls;
    }
    return rendered && copied;
}

#ifdef RX7900_LIGHTNING_DISPLAY
extern "C" ULONG APIENTRY FramebufferShadowSetPointer(SURFOBJ* surface, SURFOBJ* mask,
    SURFOBJ* color, XLATEOBJ* translation, LONG hot_x, LONG hot_y,
    LONG x, LONG y, RECTL* exclusion, FLONG flags) {
    if (surface==nullptr || registry_lock==nullptr) return SPS_DECLINE;
    // Engine software cursor removal may invoke our drawing hooks. Do it before
    // taking the registry guard, so switching to hardware cannot recurse/deadlock.
    EngMovePointer(surface,-1,-1,exclusion);
    bool accepted=false;
    {
        RegistryGuard guard;
        ShadowState* state=lookup(surface);
        if (state!=nullptr && state->visible) {
            accepted=state->link.define_cursor(mask,color,translation,hot_x,hot_y,x,y,flags);
        }
    }
    if (accepted) return SPS_ACCEPT_NOEXCLUDE;
    // IntEngSetPointerShape installs the engine fallback exactly once after a
    // decline. A rejected native shape has already hidden the hardware plane.
    return SPS_DECLINE;
}

extern "C" VOID APIENTRY FramebufferShadowMovePointer(SURFOBJ* surface,LONG x,LONG y,
    RECTL* exclusion) {
    if (surface==nullptr || registry_lock==nullptr) return;
    {
        RegistryGuard guard;
        ShadowState* state=lookup(surface);
        if (state!=nullptr && state->visible && state->link.hardware_cursor()) {
            // A VOID move cannot transfer engine ownership to software. Keep
            // the retained shape and native failure diagnostics; an explicit
            // acknowledged hide contains a failed move without losing it.
            if (!state->link.move_cursor(x,y) && x!=-1) state->link.recover_failed_cursor_move();
        }
    }
    // The engine invokes this old hardware callback while changing to software.
    // It has already drawn the new software cursor, which must not be hidden.
    // Ordinary software movement goes directly to EngMovePointer in win32k.
    (void)exclusion;
}
#endif

static BOOL shadow_bitblt(SURFOBJ* target, SURFOBJ* source,
    SURFOBJ* mask, CLIPOBJ* clip, XLATEOBJ* translation, RECTL* rectangle,
    POINTL* source_point, POINTL* mask_point, BRUSHOBJ* brush,
    POINTL* brush_origin, ROP4 rop,bool try_gpu) {
    ShadowState* destination = nullptr;
    ShadowState* source_owner = nullptr;
    ShadowState* mask_owner = nullptr;
    SURFOBJ* bitmap_target = resolve(target, destination);
    SURFOBJ* bitmap_source = resolve(source, source_owner);
    SURFOBJ* bitmap_mask = resolve(mask, mask_owner);
    if (bitmap_target == nullptr || (source != nullptr && bitmap_source == nullptr) ||
        (mask != nullptr && bitmap_mask == nullptr)) return FALSE;
    ShadowState* counted=destination!=nullptr ? destination : source_owner;
    if (counted!=nullptr) {
        if (counted->statistics.bitblt_calls!=UINT64_MAX) ++counted->statistics.bitblt_calls;
        if (destination==nullptr && counted->statistics.source_only_calls!=UINT64_MAX) {
            ++counted->statistics.source_only_calls;
        }
    }
    const RECTL requested = *rectangle;
#ifdef RX7900_LIGHTNING_GDI_OFFLOAD
    const auto attempted=try_gpu ? offload(destination,source_owner,bitmap_source,clip,translation,requested,
        source_point,bitmap_mask,brush,rop,mask_point,brush_origin) : lightning::reactos::gdi::Attempt::unsupported;
    if (attempted!=lightning::reactos::gdi::Attempt::unsupported) return attempted==lightning::reactos::gdi::Attempt::performed;
    if (!readable(destination) || !readable(source_owner) || !readable(mask_owner)) return FALSE;
    if (destination!=nullptr && destination->gpu.enabled()) destination->gpu.fallback();
#endif
    RECTL operation = requested;
    LONGLONG begin=0;
    LONGLONG end=0;
    EngQueryPerformanceCounter(&begin);
    const BOOL rendered = EngBitBlt(bitmap_target, bitmap_source, bitmap_mask, clip,
        translation, &operation, source_point, mask_point, brush, brush_origin, rop);
    EngQueryPerformanceCounter(&end);
    if (counted!=nullptr) add_ticks(*counted,begin,end,false);
    if (destination == nullptr) {
        if (!rendered && counted!=nullptr && counted->statistics.failed_calls!=UINT64_MAX) {
            ++counted->statistics.failed_calls;
        }
        return rendered;
    }
    destination->dirty = policy::unite(destination->dirty,
                                       dirty_bounds(*destination, requested, clip));
    const bool copied = flush(*destination);
    if ((!rendered || !copied) && destination->statistics.failed_calls!=UINT64_MAX) {
        ++destination->statistics.failed_calls;
    }
    return rendered && copied;
}

extern "C" BOOL APIENTRY FramebufferShadowBitBlt(SURFOBJ* target, SURFOBJ* source,
    SURFOBJ* mask, CLIPOBJ* clip, XLATEOBJ* translation, RECTL* rectangle,
    POINTL* source_point, POINTL* mask_point, BRUSHOBJ* brush,
    POINTL* brush_origin, ROP4 rop) {
    if (registry_lock==nullptr || target==nullptr || rectangle==nullptr) return FALSE;
    RegistryGuard guard;
    return shadow_bitblt(target,source,mask,clip,translation,rectangle,source_point,mask_point,brush,brush_origin,rop,true);
}

#ifdef RX7900_LIGHTNING_GDI_OFFLOAD
extern "C" BOOL APIENTRY FramebufferShadowPaint(SURFOBJ* target,CLIPOBJ* clip,BRUSHOBJ* brush,
    POINTL* origin,MIX mix) {
    if (registry_lock==nullptr || target==nullptr || clip==nullptr || brush==nullptr) return FALSE;
    RegistryGuard guard;
    ShadowState* state=lookup(target);
    if (state==nullptr) return FALSE;
    uint32_t rop=0;
    if (!lightning::reactos::paint_rop4(mix,rop)) return FALSE;
    POINTL effective_origin=origin==nullptr ? POINTL{} : *origin;
    const uint32_t foreground=rop&UINT8_MAX;
    lightning::reactos::BrushPixels mask{};
    if (brush->iSolidColor==UINT32_MAX && foreground!=(rop>>8)) {
        if (lightning::reactos::brush_snapshot(brush)==nullptr) return FALSE;
        lightning::reactos::brush_mask(brush,mask);
    }
    RECTL requested=clip->rclBounds;
    if (mask.pixels==nullptr) {
        // EngBitBlt treats an absent implicit brush mask as all foreground.
        // This normalization applies equally to GPU and cached-engine drawing.
        rop=foreground|(foreground<<8);
        return shadow_bitblt(target,nullptr,nullptr,clip,nullptr,&requested,nullptr,nullptr,brush,&effective_origin,rop,true);
    }
    auto& batch=state->clip_workspace;
    if (!lightning::reactos::gdi::capture_clip(requested,clip,state->cached->sizlBitmap.cx,
        state->cached->sizlBitmap.cy,batch)) return FALSE;
    if (policy::empty(batch.bounds)) return TRUE;
    lightning::reactos::EngineBitmap expanded;
    if (!expanded.create(batch.bounds.right-batch.bounds.left,batch.bounds.bottom-batch.bounds.top,BMF_1BPP,
        LIGHTNING_GDI_MAX_PIXEL_BYTES) || !lightning::reactos::tile_brush_mask(mask,
        static_cast<uint8_t*>(expanded.surface()->pvScan0),expanded.bytes(),expanded.stride(),batch.bounds,
        effective_origin.x,effective_origin.y)) return FALSE;
    // Everything that can decline for source/clip geometry was checked before
    // the first mutation. The same mask feeds GPU and exact cached fallback.
    if (state->visible) {
        if (!state->gpu.prepare(state->cached,state->physical) || !flush(*state)) return FALSE;
        if (state->gpu.enabled()) {
            POINTL zero{};
            lightning::reactos::gdi::RasterInput input{};
            input.original={batch.bounds.left,batch.bounds.top,batch.bounds.right,batch.bounds.bottom};
            input.mask=expanded.surface();
            input.mask_point=&zero;
            input.brush=brush;
            input.brush_origin=&effective_origin;
            input.rop4=rop;
            const auto result=state->gpu.raster_operands(batch,input,state->physical);
            if (result!=lightning::reactos::gdi::Attempt::unsupported)
                return result==lightning::reactos::gdi::Attempt::performed;
        }
    }
    if (!readable(state)) return FALSE;
    const uint32_t count=batch.count==0 ? 1U : batch.count;
    const policy::Rectangle mask_bounds=batch.bounds;
    for (uint32_t index=0;index<count;++index) {
        const auto area=batch.count==0 ? mask_bounds : policy::Rectangle{
            static_cast<int32_t>(batch.rectangles[index].x),static_cast<int32_t>(batch.rectangles[index].y),
            static_cast<int32_t>(batch.rectangles[index].x+batch.rectangles[index].width),
            static_cast<int32_t>(batch.rectangles[index].y+batch.rectangles[index].height)};
        RECTL rectangle{area.left,area.top,area.right,area.bottom};
        POINTL mask_origin{area.left-mask_bounds.left,area.top-mask_bounds.top};
        // The pinned engine does not adjust mask points for clip rectangles.
        // Each prevalidated rect is therefore passed already clipped. Skipping
        // GPU retry also preserves the state-owned clip workspace during this loop.
        if (!shadow_bitblt(target,nullptr,expanded.surface(),nullptr,nullptr,&rectangle,nullptr,&mask_origin,
            brush,&effective_origin,rop,false)) return FALSE;
    }
    return TRUE;
}

extern "C" VOID APIENTRY FramebufferShadowSynchronizeSurface(SURFOBJ* surface,RECTL* rectangle,ULONG flags) {
    (void)rectangle;
    if (registry_lock==nullptr || surface==nullptr) return;
    RegistryGuard guard;
    ShadowState* state=lookup(surface);
    if (state==nullptr || !state->visible) return;
    const bool valid_flags=(flags&~(DSS_TIMER_EVENT|DSS_FLUSH_EVENT))==0;
    if ((!valid_flags || !flush(*state) || !state->gpu.complete_gpu(state->cached,state->physical)) &&
        state->statistics.failed_calls!=UINT64_MAX) ++state->statistics.failed_calls;
}

extern "C" ULONG APIENTRY FramebufferShadowDrawEscape(SURFOBJ* surface,ULONG escape,
    CLIPOBJ* clip,RECTL* window,ULONG input_bytes,PVOID input) {
    static_assert(sizeof(LightningOpenGLPresent)==LIGHTNING_OPENGL_PRESENT_BYTES,
        "Fixed pointer-free window presentation request");
    // Version 1 (synchronous) or revision 2 (producer point, async receipt).
    lightning::reactos::PresentRequest decoded{};
    if (escape!=LIGHTNING_OPENGL_PRESENT_ESCAPE || input==nullptr || window==nullptr || registry_lock==nullptr ||
        !lightning::reactos::decode_present_request(
            rx7900::ByteView(static_cast<const uint8_t*>(input),input_bytes),decoded))
        return LIGHTNING_OPENGL_PRESENT_UNSUPPORTED;
    const LightningOpenGLPresent& request=decoded.present;

    if (request.lease==0 ||
        request.x<0 || request.y<0 || request.width==0 || request.height==0 ||
        request.image_width>LIGHTNING_GDI_MAX_DIMENSION || request.image_height>LIGHTNING_GDI_MAX_DIMENSION ||
        request.width>request.image_width || request.height>request.image_height) return LIGHTNING_OPENGL_PRESENT_UNSUPPORTED;
    const int64_t left=static_cast<int64_t>(window->left)+request.x;
    const int64_t top=static_cast<int64_t>(window->top)+request.y;
    const int64_t right=left+request.width;
    const int64_t bottom=top+request.height;

    if (left<INT32_MIN || top<INT32_MIN || right>INT32_MAX || bottom>INT32_MAX) {
        return LIGHTNING_OPENGL_PRESENT_UNSUPPORTED;
    }
    RegistryGuard guard;
    ShadowState* state=lookup(surface);

    if (state==nullptr || !state->visible || state->cached==nullptr ||
        !state->gpu.prepare(state->cached,state->physical) || !flush(*state)) return LIGHTNING_OPENGL_PRESENT_FAILED;
    if (!state->gpu.enabled()) return LIGHTNING_OPENGL_PRESENT_UNSUPPORTED;
    const RECTL requested{static_cast<LONG>(left),static_cast<LONG>(top),
        static_cast<LONG>(right),static_cast<LONG>(bottom)};

    if (!lightning::reactos::gdi::capture_clip(requested,clip,
        static_cast<uint32_t>(state->cached->sizlBitmap.cx),
        static_cast<uint32_t>(state->cached->sizlBitmap.cy),state->clip_workspace)) {
        return LIGHTNING_OPENGL_PRESENT_UNSUPPORTED;
    }
    const auto bounded=state->clip_workspace.bounds;

    if (policy::empty(bounded)) return LIGHTNING_OPENGL_PRESENT_SUCCESS;
    int32_t source_x=0;
    int32_t source_y=0;

    if (!lightning::reactos::gdi::source_position({requested.left,requested.top,requested.right,requested.bottom},
        bounded,0,0,static_cast<int32_t>(request.image_width),static_cast<int32_t>(request.image_height),
        source_x,source_y)) return LIGHTNING_OPENGL_PRESENT_UNSUPPORTED;
    const LightningGdiExternalImage image{sizeof(image),LIGHTNING_GDI_EXTERNAL_IMAGE_VERSION,
        0,0,request.lease,request.image_width,request.image_height,request.image_stride,request.image_offset};
    bool pending=false;
    const auto result=decoded.revision2 ?
        state->gpu.external_image(state->clip_workspace,static_cast<uint32_t>(source_x),
            static_cast<uint32_t>(source_y),image,
            {decoded.async,decoded.producer.sync,decoded.producer.value},pending) :
        state->gpu.external_image(state->clip_workspace,static_cast<uint32_t>(source_x),
            static_cast<uint32_t>(source_y),image);

    if (result==lightning::reactos::gdi::Attempt::unsupported) return LIGHTNING_OPENGL_PRESENT_UNSUPPORTED;
    // QUEUED: accepted with its source read still outstanding; the caller
    // reuses the image only after the receipt escape reports it retired.
    if (result==lightning::reactos::gdi::Attempt::performed && pending) return LIGHTNING_OPENGL_PRESENT_QUEUED;
    return result==lightning::reactos::gdi::Attempt::performed ?
        LIGHTNING_OPENGL_PRESENT_SUCCESS : LIGHTNING_OPENGL_PRESENT_FAILED;
}

extern "C" BOOL APIENTRY FramebufferShadowGradientFill(SURFOBJ* target,CLIPOBJ* clip,XLATEOBJ* translation,
    TRIVERTEX* vertices,ULONG vertex_count,PVOID mesh,ULONG mesh_count,
    RECTL* extents,POINTL* origin,ULONG mode) {
    if (registry_lock==nullptr || target==nullptr || extents==nullptr || origin==nullptr) return FALSE;
    RegistryGuard guard;
    ShadowState* destination=nullptr;
    SURFOBJ* bitmap_target=resolve(target,destination);
    if (bitmap_target==nullptr) return FALSE;
    if (destination!=nullptr && destination->visible) {
        if (!destination->gpu.prepare(destination->cached,destination->physical) || !flush(*destination)) return FALSE;
        if (destination->gpu.enabled()) {
            const auto result=destination->gpu.gradient(destination->clip_workspace,clip,translation,
                vertices,vertex_count,mesh,mesh_count,*extents,*origin,mode);
            if (result!=lightning::reactos::gdi::Attempt::unsupported)
                return result==lightning::reactos::gdi::Attempt::performed;
        }
    }
    if (!readable(destination)) return FALSE;
    if (destination!=nullptr && destination->gpu.enabled()) destination->gpu.fallback();
    const BOOL rendered=EngGradientFill(bitmap_target,clip,translation,vertices,vertex_count,mesh,mesh_count,extents,origin,mode);
    if (destination==nullptr) return rendered;
    destination->dirty=policy::unite(destination->dirty,dirty_bounds(*destination,*extents,clip));
    const bool copied=flush(*destination);
    if ((!rendered || !copied) && destination->statistics.failed_calls!=UINT64_MAX) ++destination->statistics.failed_calls;
    return rendered && copied;
}

extern "C" BOOL APIENTRY FramebufferShadowStretchBltROP(SURFOBJ* target,SURFOBJ* source,SURFOBJ* mask,
    CLIPOBJ* clip,XLATEOBJ* translation,COLORADJUSTMENT* adjustment,POINTL* brush_origin,
    RECTL* destination_rectangle,RECTL* source_rectangle,POINTL* mask_origin,
    ULONG mode,BRUSHOBJ* brush,ROP4 rop) {
    COLORADJUSTMENT* original_adjustment=nullptr;
    ULONG orientation=0;
    // Unpaired engines erased the mirror flags before reaching this hook.
    // Return to their original fallback without drawing or changing rectangles.
    if (!Rx7900StretchGuardUnwrap(adjustment,&original_adjustment,&orientation)) return FALSE;
    adjustment=original_adjustment;
    if (registry_lock==nullptr || target==nullptr || source==nullptr ||
        destination_rectangle==nullptr || source_rectangle==nullptr) return FALSE;
    RegistryGuard guard;
    ShadowState* destination=nullptr;
    ShadowState* source_owner=nullptr;
    ShadowState* mask_owner=nullptr;
    SURFOBJ* bitmap_target=resolve(target,destination);
    SURFOBJ* bitmap_source=resolve(source,source_owner);
    SURFOBJ* bitmap_mask=mask==nullptr ? nullptr : resolve(mask,mask_owner);
    if (bitmap_target==nullptr || bitmap_source==nullptr || (mask!=nullptr && bitmap_mask==nullptr)) return FALSE;
    const RECTL requested=*destination_rectangle;
    if (destination!=nullptr && destination->visible) {
        if (!destination->gpu.prepare(destination->cached,destination->physical) || !flush(*destination)) return FALSE;
        // The source-pinned COLORONCOLOR engine does not apply pca; it passes
        // normalized pixels to the integer DIB sampler without color adjustment.
        // A masked stretch is admitted only where the engine's rclBounds
        // pre-clip is a no-op (so each CombinedRect is one captured clip), the
        // mask is a plain engine bitmap and a same-canvas source cannot alias
        // the pixels the DIB loop rewrites while it reads.
        const LONG canvas_width=destination->cached->sizlBitmap.cx;
        const LONG canvas_height=destination->cached->sizlBitmap.cy;
        const bool mask_admitted=mask==nullptr || (mask_owner==nullptr && bitmap_mask==mask &&
            requested.left>=0 && requested.top>=0 && requested.left<requested.right &&
            requested.top<requested.bottom && requested.right<=canvas_width && requested.bottom<=canvas_height &&
            (clip==nullptr || (clip->rclBounds.left<=requested.left && clip->rclBounds.top<=requested.top &&
                clip->rclBounds.right>=requested.right && clip->rclBounds.bottom>=requested.bottom)) &&
            (source_owner!=destination || source_rectangle->right<requested.left-1 ||
                source_rectangle->left>requested.right || source_rectangle->bottom<requested.top-1 ||
                source_rectangle->top>requested.bottom));
        if (destination->gpu.enabled() && mask_admitted && mode==COLORONCOLOR) {
            const bool same_canvas=source_owner==destination &&
                (translation==nullptr || (translation->flXlate&XO_TRIVIAL)!=0);
            if (!same_canvas && !readable(source_owner)) return FALSE;
            lightning::reactos::gdi::ClipBatch& batch=destination->clip_workspace;
            if (lightning::reactos::gdi::capture_clip(requested,clip,
                destination->cached->sizlBitmap.cx,destination->cached->sizlBitmap.cy,batch)) {
                lightning::reactos::gdi::RasterInput input{};
                input.source=bitmap_source;
                input.translation=translation;
                input.original=requested;
                input.same_canvas=same_canvas;
                input.stretch=true;
                input.source_rectangle=*source_rectangle;
                input.mask=bitmap_mask;
                input.brush=brush;
                input.brush_origin=brush_origin;
                input.rop4=rop;
                input.raster_flags=((orientation&RX7900_STRETCH_FLIP_X)!=0 ? LIGHTNING_GDI_RASTER_MIRROR_X : 0U) |
                    ((orientation&RX7900_STRETCH_FLIP_Y)!=0 ? LIGHTNING_GDI_RASTER_MIRROR_Y : 0U);
                const auto result=destination->gpu.raster_operands(batch,input,destination->physical);
                if (result!=lightning::reactos::gdi::Attempt::unsupported)
                    return result==lightning::reactos::gdi::Attempt::performed;
            }
        }
    }
    // IntEng retains the original flip state. Its fallback must receive that
    // state when the exact GPU sample domain cannot be represented safely.
    if (orientation!=0) return FALSE;
    if (!readable(destination) || !readable(source_owner) || !readable(mask_owner)) return FALSE;
    if (destination!=nullptr && destination->gpu.enabled()) destination->gpu.fallback();
    RECTL target_rectangle=requested;
    RECTL source_area=*source_rectangle;
    const BOOL rendered=EngStretchBltROP(bitmap_target,bitmap_source,bitmap_mask,clip,translation,
        adjustment,brush_origin,&target_rectangle,&source_area,mask_origin,mode,brush,rop);
    if (destination==nullptr) return rendered;
    RECTL bounds=requested;
    if (bounds.left>bounds.right) {const LONG value=bounds.left; bounds.left=bounds.right; bounds.right=value;}
    if (bounds.top>bounds.bottom) {const LONG value=bounds.top; bounds.top=bounds.bottom; bounds.bottom=value;}
    destination->dirty=policy::unite(destination->dirty,dirty_bounds(*destination,bounds,clip));
    const bool copied=flush(*destination);
    if ((!rendered || !copied) && destination->statistics.failed_calls!=UINT64_MAX) ++destination->statistics.failed_calls;
    return rendered && copied;
}

extern "C" BOOL APIENTRY FramebufferShadowTransparentBlt(SURFOBJ* target,SURFOBJ* source,CLIPOBJ* clip,
    XLATEOBJ* translation,RECTL* destination_rectangle,RECTL* source_rectangle,
    ULONG transparent_color,ULONG reserved) {
    if (registry_lock==nullptr || target==nullptr || source==nullptr || destination_rectangle==nullptr ||
        source_rectangle==nullptr || reserved!=0) return FALSE;
    RegistryGuard guard;
    ShadowState* destination=nullptr;
    ShadowState* source_owner=nullptr;
    SURFOBJ* bitmap_target=resolve(target,destination);
    SURFOBJ* bitmap_source=resolve(source,source_owner);
    if (bitmap_target==nullptr || bitmap_source==nullptr) return FALSE;
    const RECTL requested=*destination_rectangle;
    if (destination!=nullptr && destination->visible) {
        if (!destination->gpu.prepare(destination->cached,destination->physical) || !flush(*destination)) return FALSE;
        if (destination->gpu.enabled()) {
            const bool same_canvas=source_owner==destination &&
                (translation==nullptr || (translation->flXlate&XO_TRIVIAL)!=0);
            if (!same_canvas && !readable(source_owner)) return FALSE;
            auto& batch=destination->clip_workspace;
            if (lightning::reactos::gdi::capture_clip(requested,clip,
                destination->cached->sizlBitmap.cx,destination->cached->sizlBitmap.cy,batch)) {
                const auto result=destination->gpu.transparent(batch,bitmap_source,translation,
                    requested,*source_rectangle,transparent_color,same_canvas,destination->physical);
                if (result!=lightning::reactos::gdi::Attempt::unsupported)
                    return result==lightning::reactos::gdi::Attempt::performed;
            }
        }
    }
    if (!readable(destination) || !readable(source_owner)) return FALSE;
    if (destination!=nullptr && destination->gpu.enabled()) destination->gpu.fallback();
    RECTL target_rectangle=requested;
    RECTL source_area=*source_rectangle;
    const BOOL rendered=EngTransparentBlt(bitmap_target,bitmap_source,clip,translation,
        &target_rectangle,&source_area,transparent_color,reserved);
    if (destination==nullptr) return rendered;
    destination->dirty=policy::unite(destination->dirty,dirty_bounds(*destination,requested,clip));
    const bool copied=flush(*destination);
    if ((!rendered || !copied) && destination->statistics.failed_calls!=UINT64_MAX) ++destination->statistics.failed_calls;
    return rendered && copied;
}

extern "C" BOOL APIENTRY FramebufferShadowAlphaBlend(SURFOBJ* target,SURFOBJ* source,
    CLIPOBJ* clip,XLATEOBJ* translation,RECTL* destination_rectangle,
    RECTL* source_rectangle,BLENDOBJ* blend) {
    if (registry_lock==nullptr || target==nullptr || source==nullptr ||
        destination_rectangle==nullptr || source_rectangle==nullptr || blend==nullptr) return FALSE;
    RegistryGuard guard;
    ShadowState* destination=nullptr;
    ShadowState* source_owner=nullptr;
    SURFOBJ* bitmap_target=resolve(target,destination);
    SURFOBJ* bitmap_source=resolve(source,source_owner);
    if (bitmap_target==nullptr || bitmap_source==nullptr) return FALSE;
    const RECTL requested=*destination_rectangle;
    if (destination!=nullptr && destination->visible &&
        !destination->gpu.prepare(destination->cached,destination->physical)) return FALSE;
    if (destination!=nullptr && destination->visible && destination->gpu.enabled()) {
        const bool direct_source=source_owner==destination &&
            (translation==nullptr || (translation->flXlate&XO_TRIVIAL)!=0);
        // A same-canvas identity source is snapshotted by the GPU executor.
        // CPU normalization alone needs the source cache to become readable.
        if (!flush(*destination) || (!direct_source && !readable(source_owner))) return FALSE;
        lightning::reactos::gdi::ClipBatch& batch=destination->clip_workspace;
        if (lightning::reactos::gdi::capture_clip(requested,clip,
            destination->cached->sizlBitmap.cx,destination->cached->sizlBitmap.cy,batch)) {
            const auto attempt=destination->gpu.blend(batch,bitmap_source,translation,requested,
                *source_rectangle,blend,destination->physical,source_owner==destination);
            if (attempt!=lightning::reactos::gdi::Attempt::unsupported) {
                return attempt==lightning::reactos::gdi::Attempt::performed;
            }
        }
    }
    if (!readable(destination) || !readable(source_owner)) return FALSE;
    if (destination!=nullptr && destination->gpu.enabled()) destination->gpu.fallback();
    RECTL target_rectangle=requested;
    RECTL source_area=*source_rectangle;
    const BOOL rendered=EngAlphaBlend(bitmap_target,bitmap_source,clip,translation,
        &target_rectangle,&source_area,blend);
    if (destination==nullptr) return rendered;
    destination->dirty=policy::unite(destination->dirty,dirty_bounds(*destination,requested,clip));
    const bool copied=flush(*destination);
    if ((!rendered || !copied) && destination->statistics.failed_calls!=UINT64_MAX) ++destination->statistics.failed_calls;
    return rendered && copied;
}
#endif

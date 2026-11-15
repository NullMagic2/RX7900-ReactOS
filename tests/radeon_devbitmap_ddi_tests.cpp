/*
Module: rx7900.tests.radeon_devbitmap_ddi
Purpose: DDI-level check of the device-bitmap bridge against the production radeondisp:
identity reply, DrvCreateDeviceBitmap admission and declines, exact pixels through
DrvCopyBits readback/upload for the bridge's readers and writers, exactly-once
DrvDeleteDeviceBitmap through cleanup and through a dynamic mode change.
Created: 2026-10-24
Architecture: Links the production radeon_display.cpp, radeon_cursor_client.cpp,
radeon_2d_ioctl.cpp (the miniport Dispatcher) and radeon_devbitmap_bridge.cpp. The test
supplies the win32k side: Eng* exports modelled on the pinned win32ss/gdi/eng/surface.c
(SURFACE_AllocSurface fields, EngAssociateSurface hook mask and palette) and
copybits.c (EngCopyBits dispatch to DrvCopyBits for device operands), the bridge's
engine table as the patch's surface.c glue builds it, PDEV device locks, and a
miniport whose 2D backend keeps exact CPU pixels for every GPU address. No kernel,
hardware or VM is involved.
*/
#include <stdarg.h>
#include <windef.h>
#include <wingdi.h>
#include <winddi.h>
#include <winioctl.h>
#include <ntddvdeo.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "radeon_2d_abi.hpp"
#include "radeon_2d_ioctl.hpp"
#include "radeon_devbitmap_bridge.hpp"
#include "radeon_line_identity.hpp"
#include "dib_floodfill_reference.hpp"

namespace {

namespace abi = rx7900::reactos::radeon2d::abi;
namespace r2d = rx7900::reactos::radeon2d;
namespace line = rx7900::reactos::radeon_line;

unsigned checks = 0;
unsigned failures = 0;

void check(bool condition, const char* message) {
    ++checks;
    if (!condition) {
        ++failures;
        printf("FAIL %s\n", message);
    }
}

constexpr FLONG marker_flag = 0x00200000;   // DRIVER_CREATED_SURFACE
constexpr FLONG api_flag = 0x04000000;      // API_BITMAP
constexpr USHORT pool_flag = 0x100;         // BMF_POOLALLOC
constexpr ULONG screen_width = 320;
constexpr ULONG screen_height = 240;
constexpr ULONG screen_pitch = screen_width * 4;
constexpr uint64_t primary_address = 0x10000000ULL;

// ---------------------------------------------------------------------------
// Miniport: the production Dispatcher over a backend with exact CPU pixels.
// ---------------------------------------------------------------------------
struct BackendSurface {
    bool used = false;
    uint64_t address = 0;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t pitch = 0;
    uint8_t* bytes = nullptr;
};

struct Backend {
    uint8_t framebuffer[screen_pitch * screen_height]{};
    BackendSurface surfaces[64]{};
    uint8_t* staging = nullptr;
    uint64_t fence = 0;
    unsigned created = 0;
    unsigned destroyed = 0;
    unsigned uploads = 0;
    unsigned readbacks = 0;
} backend{};

uint8_t* memory_of(const r2d::EngineSurface& surface) {
    if (surface.gpu_address == primary_address) {
        return backend.framebuffer;
    }

    for (BackendSurface& entry : backend.surfaces) {
        if (entry.used && entry.address == surface.gpu_address) {
            return entry.bytes;
        }
    }

    return nullptr;
}

uint32_t* pixel_of(const r2d::EngineSurface& surface, uint32_t x, uint32_t y) {
    return reinterpret_cast<uint32_t*>(memory_of(surface) + static_cast<size_t>(y) * surface.pitch_bytes) + x;
}

r2d::BackendStatus finish(uint64_t& fence) {
    fence = ++backend.fence;
    return r2d::BackendStatus::ok;
}

r2d::BackendStatus be_fill(void*, const r2d::EngineSurface& target, uint32_t color, const r2d::EngineRect* rects,
                           uint32_t count, uint64_t& fence) noexcept {
    for (uint32_t index = 0; index < count; ++index) {
        for (uint32_t y = 0; y < rects[index].height; ++y) {
            for (uint32_t x = 0; x < rects[index].width; ++x) {
                *pixel_of(target, rects[index].x + x, rects[index].y + y) = color;
            }
        }
    }

    return finish(fence);
}

r2d::BackendStatus be_copy(void*, const r2d::EngineSurface& target, const r2d::EngineSurface& source,
                           const r2d::CopyRect* rects, uint32_t count, uint64_t& fence) noexcept {
    static uint32_t row[4096];
    for (uint32_t index = 0; index < count; ++index) {
        const r2d::CopyRect& rect = rects[index];
        for (uint32_t step = 0; step < rect.destination.height; ++step) {
            // memmove semantics per rectangle: walk rows away from the overlap.
            const bool down = rect.destination.y <= rect.source_y;
            const uint32_t y = down ? step : rect.destination.height - 1 - step;
            memcpy(row, pixel_of(source, rect.source_x, rect.source_y + y), rect.destination.width * 4U);
            memcpy(pixel_of(target, rect.destination.x, rect.destination.y + y), row, rect.destination.width * 4U);
        }
    }

    return finish(fence);
}

r2d::BackendStatus be_upload(void*, const r2d::EngineSurface& target, const r2d::StagingBuffer& staging,
                             uint32_t pitch, const r2d::TransferRect* rects, uint32_t count,
                             uint64_t& fence) noexcept {
    ++backend.uploads;
    const uint8_t* base = reinterpret_cast<const uint8_t*>(static_cast<uintptr_t>(staging.cpu_address));
    for (uint32_t index = 0; index < count; ++index) {
        for (uint32_t y = 0; y < rects[index].surface.height; ++y) {
            memcpy(pixel_of(target, rects[index].surface.x, rects[index].surface.y + y),
                   base + rects[index].staging_offset + static_cast<size_t>(y) * pitch, rects[index].surface.width * 4U);
        }
    }

    return finish(fence);
}

r2d::BackendStatus be_readback(void*, const r2d::EngineSurface& source, const r2d::StagingBuffer& staging,
                               uint32_t pitch, const r2d::TransferRect* rects, uint32_t count,
                               uint64_t& fence) noexcept {
    ++backend.readbacks;
    uint8_t* base = reinterpret_cast<uint8_t*>(static_cast<uintptr_t>(staging.cpu_address));
    for (uint32_t index = 0; index < count; ++index) {
        for (uint32_t y = 0; y < rects[index].surface.height; ++y) {
            memcpy(base + rects[index].staging_offset + static_cast<size_t>(y) * pitch,
                   pixel_of(source, rects[index].surface.x, rects[index].surface.y + y), rects[index].surface.width * 4U);
        }
    }

    return finish(fence);
}

r2d::BackendStatus be_surface_create(void*, uint32_t width, uint32_t height, uint32_t format,
                                     r2d::EngineSurface& output, uint32_t& allocation) noexcept {
    for (uint32_t index = 0; index < 64; ++index) {
        BackendSurface& entry = backend.surfaces[index];
        if (!entry.used) {
            entry.used = true;
            entry.width = width;
            entry.height = height;
            entry.pitch = (width * 4U + 255U) & ~255U;
            entry.address = 0x100000000ULL + index * 0x1000000ULL;
            // Zeroed, as the production backend clears new surfaces.
            entry.bytes = static_cast<uint8_t*>(calloc(entry.pitch, height));
            output = {entry.address, width, height, entry.pitch, format};
            allocation = index + 1;
            ++backend.created;
            return r2d::BackendStatus::ok;
        }
    }

    return r2d::BackendStatus::rejected;
}

void be_surface_destroy(void*, uint32_t allocation) noexcept {
    BackendSurface& entry = backend.surfaces[allocation - 1];
    free(entry.bytes);
    entry = BackendSurface{};
    ++backend.destroyed;
}

r2d::BackendStatus be_staging_create(void*, uint32_t bytes, r2d::StagingBuffer& output) noexcept {
    uint8_t* memory = static_cast<uint8_t*>(calloc(1, bytes));
    output.gpu_address = 0x8000000ULL + backend.created * 0x100000ULL;
    output.cpu_address = reinterpret_cast<uintptr_t>(memory);
    output.bytes = bytes;
    output.allocation = 1;
    return r2d::BackendStatus::ok;
}

void be_staging_destroy(void*, const r2d::StagingBuffer& staging) noexcept {
    free(reinterpret_cast<void*>(static_cast<uintptr_t>(staging.cpu_address)));
}

r2d::BackendStatus be_sync(void*, uint64_t) noexcept { return r2d::BackendStatus::ok; }
uint64_t be_completed(void*) noexcept { return backend.fence; }

r2d::BackendStatus be_keep(void*, uint32_t allocation, r2d::EngineSurface& output) noexcept {
    const BackendSurface& entry = backend.surfaces[allocation - 1];
    output = {entry.address, entry.width, entry.height, entry.pitch, abi::format_xrgb8888};
    return r2d::BackendStatus::ok;
}

r2d::Dispatcher dispatcher;

void configure_miniport() {
    r2d::Blit2dBackend table{};
    table.fill = be_fill;
    table.copy = be_copy;
    table.upload = be_upload;
    table.readback = be_readback;
    table.surface_create = be_surface_create;
    table.surface_destroy = be_surface_destroy;
    table.staging_create = be_staging_create;
    table.staging_destroy = be_staging_destroy;
    table.sync = be_sync;
    table.completed = be_completed;
    table.surface_evict = be_keep;
    table.surface_restore = be_keep;
    const r2d::EngineSurface primary{primary_address, screen_width, screen_height, screen_pitch,
                                     abi::format_xrgb8888};
    check(dispatcher.configure(table, primary, 64U * 1024U, 0x1234U), "the production dispatcher is configured");
}

// ---------------------------------------------------------------------------
// win32k model: GDI surfaces and PDEVs.
// ---------------------------------------------------------------------------
struct GdiSurface {
    SURFOBJ so{};
    bool used = false;
    bool deleted = false;
    int refs = 0;
    FLONG flags = 0;
    void* palette = nullptr;
    bool engine_bits = false;   // bits inside the object (freed with it)
    bool adopted = false;
};

GdiSurface gdi[256];

struct EnginePdev {
    DHPDEV dhpdev = nullptr;
    GdiSurface* surface = nullptr;
    DEVINFO devinfo{};
    GDIINFO gdiinfo{};
    PFN functions[INDEX_LAST]{};
    int lock_depth = 0;
    long refs = 1;
    bool disabled = false;
    ULONG acceleration = 0;
};

EnginePdev pdevs[2];
HANDLE const miniport = reinterpret_cast<HANDLE>(static_cast<uintptr_t>(0x4d50));
unsigned unlocked_driver_calls = 0;
unsigned palette_count = 0;

GdiSurface* allocate_gdi() {
    for (GdiSurface& surface : gdi) {
        if (!surface.used) {
            surface = GdiSurface{};
            surface.used = true;
            surface.so.hsurf = reinterpret_cast<HSURF>(&surface);
            return &surface;
        }
    }

    return nullptr;
}

GdiSurface& gdi_of(SURFOBJ* so) {
    return *reinterpret_cast<GdiSurface*>(reinterpret_cast<uint8_t*>(so) - offsetof(GdiSurface, so));
}

GdiSurface* gdi_of(HSURF handle) {
    GdiSurface* surface = reinterpret_cast<GdiSurface*>(handle);
    return surface >= gdi && surface < gdi + 256 && surface->used && !surface->deleted ? surface : nullptr;
}

EnginePdev& pdev_of(HDEV hdev) { return *reinterpret_cast<EnginePdev*>(hdev); }
HDEV hdev_of(EnginePdev& pdev) { return reinterpret_cast<HDEV>(&pdev); }

// DRVFN entries are generic PFNs; each is called through its INDEX_Drv*
// prototype (the void(*)() step is the conventional function-pointer cast).
template <typename Function>
Function ddi(PFN function) {
    return reinterpret_cast<Function>(reinterpret_cast<void (*)()>(function));
}

ULONG bpp_of(ULONG format) {
    return format == BMF_32BPP ? 32 : format == BMF_24BPP ? 24 : format == BMF_16BPP ? 16 :
           format == BMF_8BPP ? 8 : format == BMF_4BPP ? 4 : 1;
}

ULONG stride_of(LONG width, ULONG format) {
    return ((static_cast<ULONG>(width) * bpp_of(format) + 31U) & ~31U) >> 3;
}

// SURFACE_AllocSurface (pinned surface.c) for STYPE_BITMAP and opaque types.
GdiSurface* alloc_surface(USHORT type, SIZEL size, ULONG format, FLONG fjBitmap, LONG width, PVOID bits) {
    GdiSurface* surface = allocate_gdi();
    SURFOBJ& so = surface->so;
    so.sizlBitmap = size;
    so.iBitmapFormat = format;
    so.iType = type;
    so.fjBitmap = static_cast<USHORT>(fjBitmap);
    // Caller bits with a width keep that (re-aligned) width, as the engine does.
    const ULONG cj_width = bits != nullptr && width != 0 ?
        stride_of((width * 8) / static_cast<LONG>(bpp_of(format)), format) : stride_of(size.cx, format);
    so.cjBits = cj_width * size.cy;
    if (type == STYPE_BITMAP) {
        if (bits != nullptr) {
            so.pvBits = bits;
        } else {
            so.pvBits = calloc(1, so.cjBits);
            surface->engine_bits = true;
        }

        if (fjBitmap & BMF_TOPDOWN) {
            so.pvScan0 = so.pvBits;
            so.lDelta = static_cast<LONG>(cj_width);
        } else {
            so.pvScan0 = static_cast<uint8_t*>(so.pvBits) + so.cjBits - cj_width;
            so.lDelta = -static_cast<LONG>(cj_width);
        }
    }

    surface->refs = 1;
    return surface;
}

void free_surface(GdiSurface& surface);

void dereference(GdiSurface& surface) {
    --surface.refs;
    if (surface.deleted && surface.refs == 0) {
        free_surface(surface);
    }
}

void delete_object(GdiSurface& surface) {
    surface.deleted = true;
    dereference(surface);
}

uint32_t* engine_pixel(SURFOBJ& so, LONG x, LONG y) {
    return reinterpret_cast<uint32_t*>(static_cast<uint8_t*>(so.pvScan0) + static_cast<intptr_t>(y) * so.lDelta) + x;
}

bool device_operand(SURFOBJ* so) { return so != nullptr && so->iType != STYPE_BITMAP; }

} // namespace

// ---------------------------------------------------------------------------
// Eng* exports (the subset radeondisp imports).
// ---------------------------------------------------------------------------
extern "C" PVOID APIENTRY EngAllocMem(ULONG flags, ULONG bytes, ULONG) {
    return (flags & FL_ZERO_MEMORY) != 0 ? calloc(1, bytes) : malloc(bytes);
}

extern "C" VOID APIENTRY EngFreeMem(PVOID memory) { free(memory); }

extern "C" HBITMAP APIENTRY EngCreateBitmap(SIZEL size, LONG width, ULONG format, FLONG flags, PVOID bits) {
    GdiSurface* surface = alloc_surface(STYPE_BITMAP, size, format, flags, width, bits);
    surface->flags = api_flag;
    surface->refs = 0;
    return reinterpret_cast<HBITMAP>(surface->so.hsurf);
}

extern "C" HBITMAP APIENTRY EngCreateDeviceBitmap(DHSURF dhsurf, SIZEL size, ULONG format) {
    GdiSurface* surface = alloc_surface(STYPE_DEVBITMAP, size, format, 0, 0, nullptr);
    surface->so.dhsurf = dhsurf;
    surface->refs = 0;
    return reinterpret_cast<HBITMAP>(surface->so.hsurf);
}

extern "C" HSURF APIENTRY EngCreateDeviceSurface(DHSURF dhsurf, SIZEL size, ULONG format) {
    GdiSurface* surface = alloc_surface(STYPE_DEVICE, size, format, 0, 0, nullptr);
    surface->so.dhsurf = dhsurf;
    surface->refs = 0;
    return surface->so.hsurf;
}

extern "C" BOOL APIENTRY EngAssociateSurface(HSURF handle, HDEV hdev, FLONG hooks) {
    GdiSurface* surface = gdi_of(handle);
    if (surface == nullptr) {
        return FALSE;
    }

    EnginePdev& pdev = pdev_of(hdev);
    surface->so.hdev = hdev;
    surface->so.dhpdev = pdev.dhpdev;
    surface->flags = (surface->flags & ~HOOK_FLAGS) | (hooks & HOOK_FLAGS);
    surface->palette = pdev.devinfo.hpalDefault;
    return TRUE;
}

extern "C" SURFOBJ* APIENTRY EngLockSurface(HSURF handle) {
    GdiSurface* surface = gdi_of(handle);
    if (surface == nullptr) {
        return nullptr;
    }

    ++surface->refs;
    return &surface->so;
}

extern "C" VOID APIENTRY EngUnlockSurface(SURFOBJ* so) {
    if (so != nullptr) {
        dereference(gdi_of(so));
    }
}

extern "C" BOOL APIENTRY EngDeleteSurface(HSURF handle) {
    GdiSurface* surface = gdi_of(handle);
    if (surface == nullptr) {
        return FALSE;
    }

    ++surface->refs;
    delete_object(*surface);
    return TRUE;
}

extern "C" HSEMAPHORE APIENTRY EngCreateSemaphore(VOID) {
    return reinterpret_cast<HSEMAPHORE>(calloc(1, sizeof(int)));
}

extern "C" VOID APIENTRY EngAcquireSemaphore(HSEMAPHORE semaphore) { ++*reinterpret_cast<int*>(semaphore); }
extern "C" VOID APIENTRY EngReleaseSemaphore(HSEMAPHORE semaphore) { --*reinterpret_cast<int*>(semaphore); }
extern "C" VOID APIENTRY EngDeleteSemaphore(HSEMAPHORE semaphore) { free(semaphore); }

extern "C" HPALETTE APIENTRY EngCreatePalette(ULONG, ULONG, ULONG*, FLONG, FLONG, FLONG) {
    ++palette_count;
    return reinterpret_cast<HPALETTE>(static_cast<uintptr_t>(0x7000U + palette_count * 16U));
}

extern "C" BOOL APIENTRY EngDeletePalette(HPALETTE) { return TRUE; }

extern "C" DWORD APIENTRY EngDeviceIoControl(HANDLE device, DWORD code, LPVOID input, DWORD input_bytes,
                                             LPVOID output, DWORD output_bytes, LPDWORD returned) {
    *returned = 0;
    if (device != miniport) {
        return ERROR_INVALID_HANDLE;
    }

    switch (code) {
    case IOCTL_VIDEO_QUERY_NUM_AVAIL_MODES: {
        VIDEO_NUM_MODES modes{};
        modes.NumModes = 1;
        modes.ModeInformationLength = sizeof(VIDEO_MODE_INFORMATION);
        memcpy(output, &modes, sizeof(modes));
        *returned = sizeof(modes);
        return 0;
    }
    case IOCTL_VIDEO_QUERY_AVAIL_MODES: {
        VIDEO_MODE_INFORMATION mode{};
        mode.Length = sizeof(mode);
        mode.ModeIndex = 0;
        mode.VisScreenWidth = screen_width;
        mode.VisScreenHeight = screen_height;
        mode.ScreenStride = screen_pitch;
        mode.NumberOfPlanes = 1;
        mode.BitsPerPlane = 32;
        mode.Frequency = 60;
        mode.RedMask = 0x00ff0000U;
        mode.GreenMask = 0x0000ff00U;
        mode.BlueMask = 0x000000ffU;
        mode.AttributeFlags = VIDEO_MODE_GRAPHICS | VIDEO_MODE_COLOR;
        memcpy(output, &mode, sizeof(mode));
        *returned = sizeof(mode);
        return 0;
    }
    case IOCTL_VIDEO_SET_CURRENT_MODE:
    case IOCTL_VIDEO_UNMAP_VIDEO_MEMORY:
    case IOCTL_VIDEO_RESET_DEVICE:
        return 0;
    case IOCTL_VIDEO_MAP_VIDEO_MEMORY: {
        VIDEO_MEMORY_INFORMATION mapping{};
        mapping.VideoRamBase = backend.framebuffer;
        mapping.VideoRamLength = sizeof(backend.framebuffer);
        mapping.FrameBufferBase = backend.framebuffer;
        mapping.FrameBufferLength = sizeof(backend.framebuffer);
        memcpy(output, &mapping, sizeof(mapping));
        *returned = sizeof(mapping);
        return 0;
    }
    default:
        break;
    }

    if (!abi::is_radeon_2d_ioctl(code)) {
        return ERROR_NOT_SUPPORTED;
    }

    const r2d::DispatchResult result =
        dispatcher.dispatch(code, rx7900::ByteView(static_cast<const uint8_t*>(input), input_bytes), output_bytes);
    if (result.status != r2d::DispatchStatus::success) {
        return ERROR_INVALID_PARAMETER;
    }

    const rx7900::ByteView reply = dispatcher.reply();
    uint8_t* bytes = static_cast<uint8_t*>(output);
    for (uint32_t index = 0; index < result.reply_bytes; ++index) {
        reply.read_u8(index, bytes[index]);
    }

    *returned = result.reply_bytes;
    return 0;
}

// EngCopyBits (pinned copybits.c): a device operand dispatches to its driver's
// DrvCopyBits; two engine bitmaps copy rows (32bpp, trivial translation).
extern "C" BOOL APIENTRY EngCopyBits(SURFOBJ* target, SURFOBJ* source, CLIPOBJ* clip, XLATEOBJ* translation,
                                     RECTL* rect, POINTL* origin) {
    if (device_operand(target) || device_operand(source)) {
        SURFOBJ* device = device_operand(target) ? target : source;
        EnginePdev& pdev = pdev_of(device->hdev);
        if ((gdi_of(device).flags & HOOK_COPYBITS) == 0) {
            return FALSE;
        }

        if (pdev.lock_depth <= 0) {
            ++unlocked_driver_calls;
        }

        return ddi<PFN_DrvCopyBits>(pdev.functions[INDEX_DrvCopyBits])(target, source, clip,
                                                                                   translation, rect, origin);
    }

    if (translation != nullptr && (translation->flXlate & XO_TRIVIAL) == 0) {
        return FALSE;
    }

    RECTL area = *rect;
    if (clip != nullptr && clip->iDComplexity != DC_TRIVIAL) {
        area.left = area.left > clip->rclBounds.left ? area.left : clip->rclBounds.left;
        area.top = area.top > clip->rclBounds.top ? area.top : clip->rclBounds.top;
        area.right = area.right < clip->rclBounds.right ? area.right : clip->rclBounds.right;
        area.bottom = area.bottom < clip->rclBounds.bottom ? area.bottom : clip->rclBounds.bottom;
    }

    for (LONG y = area.top; y < area.bottom; ++y) {
        for (LONG x = area.left; x < area.right; ++x) {
            *engine_pixel(*target, x, y) =
                *engine_pixel(*source, origin->x + x - rect->left, origin->y + y - rect->top);
        }
    }

    return TRUE;
}

extern "C" BOOL APIENTRY EngBitBlt(SURFOBJ*, SURFOBJ*, SURFOBJ*, CLIPOBJ*, XLATEOBJ*, RECTL*, POINTL*, POINTL*,
                                   BRUSHOBJ*, POINTL*, ROP4) { return FALSE; }
extern "C" BOOL APIENTRY EngAlphaBlend(SURFOBJ*, SURFOBJ*, CLIPOBJ*, XLATEOBJ*, RECTL*, RECTL*, BLENDOBJ*) {
    return FALSE;
}
extern "C" BOOL APIENTRY EngTransparentBlt(SURFOBJ*, SURFOBJ*, CLIPOBJ*, XLATEOBJ*, RECTL*, RECTL*, ULONG, ULONG) {
    return FALSE;
}
extern "C" BOOL APIENTRY EngGradientFill(SURFOBJ*, CLIPOBJ*, XLATEOBJ*, TRIVERTEX*, ULONG, PVOID, ULONG, RECTL*,
                                         POINTL*, ULONG) { return FALSE; }
extern "C" BOOL APIENTRY EngLineTo(SURFOBJ*, CLIPOBJ*, BRUSHOBJ*, LONG, LONG, LONG, LONG, RECTL*, MIX) {
    return FALSE;
}
extern "C" BOOL APIENTRY EngStretchBlt(SURFOBJ*, SURFOBJ*, SURFOBJ*, CLIPOBJ*, XLATEOBJ*, COLORADJUSTMENT*, POINTL*,
                                       RECTL*, RECTL*, POINTL*, ULONG) { return FALSE; }
extern "C" BOOL APIENTRY EngFillPath(SURFOBJ*, PATHOBJ*, CLIPOBJ*, BRUSHOBJ*, POINTL*, MIX, FLONG) { return FALSE; }
extern "C" ULONG APIENTRY CLIPOBJ_cEnumStart(CLIPOBJ*, BOOL, ULONG, ULONG, ULONG) { return 0; }
extern "C" BOOL APIENTRY CLIPOBJ_bEnum(CLIPOBJ*, ULONG, ULONG* output) {
    output[0] = 0;
    return FALSE;
}
extern "C" PVOID APIENTRY BRUSHOBJ_pvAllocRbrush(BRUSHOBJ* brush, ULONG bytes) {
    brush->pvRbrush = calloc(1, bytes);
    return brush->pvRbrush;
}
extern "C" PVOID APIENTRY BRUSHOBJ_pvGetRbrush(BRUSHOBJ* brush) { return brush->pvRbrush; }
extern "C" ULONG APIENTRY XLATEOBJ_iXlate(XLATEOBJ*, ULONG color) { return color; }
extern "C" ULONG APIENTRY XLATEOBJ_cGetPalette(XLATEOBJ*, ULONG, ULONG, ULONG*) { return 0; }
extern "C" VOID APIENTRY PATHOBJ_vEnumStart(PATHOBJ*) {}
extern "C" BOOL APIENTRY PATHOBJ_bEnum(PATHOBJ*, PATHDATA* data) {
    memset(data, 0, sizeof(*data));
    return FALSE;
}

namespace {

// ---------------------------------------------------------------------------
// The bridge's engine table, as the patch's surface.c glue builds it.
// ---------------------------------------------------------------------------
unsigned registry_depth = 0;

BOOL APIENTRY lock_registry() {
    ++registry_depth;
    return TRUE;
}

VOID APIENTRY unlock_registry() { --registry_depth; }
VOID APIENTRY lock_device(HDEV hdev) { ++pdev_of(hdev).lock_depth; }
VOID APIENTRY unlock_device(HDEV hdev) { --pdev_of(hdev).lock_depth; }
VOID APIENTRY reference_device(HDEV hdev) { ++pdev_of(hdev).refs; }
VOID APIENTRY release_device(HDEV hdev) { --pdev_of(hdev).refs; }

BOOL APIENTRY describe_device(HDEV hdev, RX7900_DEVBITMAP_DEVICE* device) {
    EnginePdev& pdev = pdev_of(hdev);
    *device = {};
    device->dhpdev = pdev.dhpdev;
    device->primary = pdev.surface != nullptr ? &pdev.surface->so : nullptr;
    device->palette = pdev.devinfo.hpalDefault;
    device->display = TRUE;
    device->disabled = pdev.disabled;
    device->acceleration_level = pdev.acceleration;
    device->escape = ddi<PFN_DrvEscape>(pdev.functions[INDEX_DrvEscape]);
    device->create = ddi<PFN_DrvCreateDeviceBitmap>(pdev.functions[INDEX_DrvCreateDeviceBitmap]);
    device->destroy = ddi<PFN_DrvDeleteDeviceBitmap>(pdev.functions[INDEX_DrvDeleteDeviceBitmap]);
    device->copy = ddi<PFN_DrvCopyBits>(pdev.functions[INDEX_DrvCopyBits]);
    return TRUE;
}

SURFOBJ* APIENTRY lock_bitmap(HBITMAP handle) { return EngLockSurface(reinterpret_cast<HSURF>(handle)); }
VOID APIENTRY unlock_bitmap(SURFOBJ* so) { EngUnlockSurface(so); }
VOID APIENTRY delete_bitmap(SURFOBJ* so) { delete_object(gdi_of(so)); }
FLONG APIENTRY read_flags(SURFOBJ* so) { return gdi_of(so).flags; }
VOID APIENTRY update_flags(SURFOBJ* so, FLONG set, FLONG clear) {
    gdi_of(so).flags = (gdi_of(so).flags | set) & ~clear;
}
PVOID APIENTRY palette_of(SURFOBJ* so) { return gdi_of(so).palette; }
VOID APIENTRY adopt_bitmap(SURFOBJ* so) { gdi_of(so).adopted = true; }

SURFOBJ* APIENTRY create_shadow(SURFOBJ* like) {
    GdiSurface* surface = alloc_surface(STYPE_BITMAP, like->sizlBitmap, like->iBitmapFormat, 0, 0, nullptr);
    surface->flags = api_flag;
    surface->palette = gdi_of(like).palette;
    return &surface->so;
}

BOOL APIENTRY copy_bits(SURFOBJ* target, SURFOBJ* source, RECTL* rect, POINTL* origin) {
    return EngCopyBits(target, source, nullptr, nullptr, rect, origin);
}

PVOID APIENTRY allocate_bits(ULONG bytes) { return calloc(1, bytes); }
VOID APIENTRY free_bits(PVOID bits) { free(bits); }
ULONG unique = 1000;
ULONG APIENTRY next_unique() { return ++unique; }

const RX7900_DEVBITMAP_ENGINE engine = {
    HOOK_FLAGS, marker_flag, api_flag, pool_flag,
    lock_registry, unlock_registry, lock_device, unlock_device, reference_device, release_device,
    describe_device, lock_bitmap, unlock_bitmap, delete_bitmap, read_flags, update_flags, palette_of,
    adopt_bitmap, create_shadow, copy_bits, allocate_bits, free_bits, next_unique,
};

// SURFACE_vCleanup of the patch.
void free_surface(GdiSurface& surface) {
    Rx7900DevBitmapCleanup(&engine, &surface.so);
    if (surface.so.pvBits != nullptr && ((surface.so.fjBitmap & pool_flag) != 0 || surface.engine_bits)) {
        free(surface.so.pvBits);
    }

    surface.used = false;
}

// ---------------------------------------------------------------------------
// PDEV lifecycle through the production DRVFN table.
// ---------------------------------------------------------------------------
DRVENABLEDATA driver{};

PFN function(ULONG index) {
    for (ULONG entry = 0; entry < driver.c; ++entry) {
        if (driver.pdrvfn[entry].iFunc == index) {
            return driver.pdrvfn[entry].pfn;
        }
    }

    return nullptr;
}

bool enable_pdev(EnginePdev& pdev) {
    pdev = EnginePdev{};
    for (ULONG index = 0; index < INDEX_LAST; ++index) {
        pdev.functions[index] = function(index);
    }

    DEVMODEW mode{};
    mode.dmSize = sizeof(mode);
    HSURF patterns[HS_DDI_MAX]{};
    pdev.dhpdev = ddi<PFN_DrvEnablePDEV>(pdev.functions[INDEX_DrvEnablePDEV])(
        &mode, nullptr, HS_DDI_MAX, patterns, sizeof(GDIINFO), reinterpret_cast<ULONG*>(&pdev.gdiinfo),
        sizeof(DEVINFO), &pdev.devinfo, hdev_of(pdev), nullptr, miniport);
    if (pdev.dhpdev == nullptr) {
        return false;
    }

    ddi<PFN_DrvCompletePDEV>(pdev.functions[INDEX_DrvCompletePDEV])(pdev.dhpdev, hdev_of(pdev));
    HSURF primary = ddi<PFN_DrvEnableSurface>(pdev.functions[INDEX_DrvEnableSurface])(pdev.dhpdev);
    pdev.surface = gdi_of(primary);
    if (pdev.surface != nullptr) {
        ++pdev.surface->refs;
    }

    return pdev.surface != nullptr;
}

void disable_pdev(EnginePdev& pdev) {
    ddi<PFN_DrvDisableSurface>(pdev.functions[INDEX_DrvDisableSurface])(pdev.dhpdev);
    ddi<PFN_DrvDisablePDEV>(pdev.functions[INDEX_DrvDisablePDEV])(pdev.dhpdev);
    pdev.dhpdev = nullptr;
}

HBITMAP create(EnginePdev& pdev, LONG width, LONG height) {
    return Rx7900DevBitmapCreate(&engine, hdev_of(pdev), SIZEL{width, height}, BMF_32BPP, nullptr);
}

BOOL fill(EnginePdev& pdev, SURFOBJ* target, RECTL rect, ULONG color) {
    BRUSHOBJ brush{};
    brush.iSolidColor = color;
    lock_device(hdev_of(pdev));
    const BOOL done = ddi<PFN_DrvBitBlt>(pdev.functions[INDEX_DrvBitBlt])(
        target, nullptr, nullptr, nullptr, nullptr, &rect, nullptr, nullptr, &brush, nullptr, 0xf0f0);
    unlock_device(hdev_of(pdev));
    return done;
}

uint32_t expected_pixel(LONG x, LONG y) {
    if (x >= 10 && x < 70 && y >= 5 && y < 50) {
        return 0x00336699U;
    }

    return x >= 40 && y >= 40 ? 0x00ffeeddU : 0;
}

bool matches_expected(SURFOBJ& so, LONG width, LONG height) {
    for (LONG y = 0; y < height; ++y) {
        for (LONG x = 0; x < width; ++x) {
            if (*engine_pixel(so, x, y) != expected_pixel(x, y)) {
                return false;
            }
        }
    }

    return true;
}

// ---------------------------------------------------------------------------
// Cases.
// ---------------------------------------------------------------------------
void identity_reply(EnginePdev& pdev) {
    PFN_DrvEscape escape = ddi<PFN_DrvEscape>(pdev.functions[INDEX_DrvEscape]);
    line::Identity reply{};
    uint32_t requested = line::flag_device_bitmaps;
    check(escape(&pdev.surface->so, line::identity_escape, sizeof(requested), &requested, sizeof(reply), &reply) ==
                  sizeof(reply) &&
              line::admits_requested(reply, line::flag_device_bitmaps, line::flag_device_bitmaps),
          "radeondisp reports flag_device_bitmaps to a request naming it");
    reply = {};
    check(escape(&pdev.surface->so, line::identity_escape, 0, nullptr, sizeof(reply), &reply) == sizeof(reply) &&
              (reply.flags & line::flag_device_bitmaps) == 0 && line::admits(reply, 0),
          "an input-free (version-1) query never carries the device-bitmap bit");
    requested = line::flag_styled_stroke;
    check(escape(&pdev.surface->so, line::identity_escape, sizeof(requested), &requested, sizeof(reply), &reply) ==
                  sizeof(reply) &&
              (reply.flags & line::flag_device_bitmaps) == 0,
          "the stroke bridge's request never receives the device-bitmap bit");
}

void bitmap_lifecycle(EnginePdev& pdev) {
    const unsigned created_before = backend.created;
    const HBITMAP handle = create(pdev, 96, 80);
    GdiSurface* surface = gdi_of(reinterpret_cast<HSURF>(handle));
    check(handle != nullptr && surface != nullptr && surface->so.iType == STYPE_DEVBITMAP &&
              (surface->flags & marker_flag) != 0 && (surface->flags & (HOOK_BITBLT | HOOK_COPYBITS)) ==
                  (HOOK_BITBLT | HOOK_COPYBITS) && surface->palette == pdev.devinfo.hpalDefault &&
              backend.created == created_before + 1 && pdev.refs == 2 && pdev.lock_depth == 0,
          "the production DrvCreateDeviceBitmap result passes the bridge: a VRAM surface, hooked, PDEV palette");
    if (surface == nullptr) {
        return;
    }

    SURFOBJ& so = surface->so;
    check(fill(pdev, &so, RECTL{10, 5, 70, 50}, 0x00336699U), "DrvBitBlt fills the device bitmap on the GPU");

    // An engine bitmap uploaded into the lower right corner (DrvCopyBits upload).
    GdiSurface& source = *alloc_surface(STYPE_BITMAP, SIZEL{96, 80}, BMF_32BPP, BMF_TOPDOWN, 0, nullptr);
    for (LONG y = 0; y < 80; ++y) {
        for (LONG x = 0; x < 96; ++x) {
            *engine_pixel(source.so, x, y) = 0x00ffeeddU;
        }
    }

    // SetBitmapBits-style store of a region through the bridge (upload).
    RECTL patch_area{70, 40, 96, 80};
    const unsigned uploads_before = backend.uploads;
    check(Rx7900DevBitmapStore(&engine, &so, &source.so, &patch_area) && backend.uploads > uploads_before,
          "the bridge stores rows through a DrvCopyBits GPU upload");
    RECTL lower{40, 50, 70, 80};
    check(Rx7900DevBitmapStore(&engine, &so, &source.so, &lower), "a second stored region lands exactly");
    delete_object(source);

    // GetBitmapBits/GetDIBits/BITMAP_CopyBitmap readers: whole readback.
    RECTL whole{0, 0, 96, 80};
    const unsigned readbacks_before = backend.readbacks;
    const unsigned destroyed_early = backend.destroyed;
    SURFOBJ* shadow = Rx7900DevBitmapShadow(&engine, &so, &whole);
    check(backend.readbacks > readbacks_before && backend.destroyed == destroyed_early,
          "the readback is a GPU transfer: the device bitmap stays resident (not demoted)");
    check(shadow != nullptr && matches_expected(*shadow, 96, 80) && unlocked_driver_calls == 0,
          "a shadow read through the production DrvCopyBits holds exactly the GPU pixels");
    if (shadow != nullptr) {
        Rx7900DevBitmapDiscard(&engine, shadow);
    }

    const HBITMAP copy = Rx7900DevBitmapKeep(&engine, Rx7900DevBitmapShadow(&engine, &so, &whole));
    GdiSurface* copied = gdi_of(reinterpret_cast<HSURF>(copy));
    check(copied != nullptr && copied->so.iType == STYPE_BITMAP && (copied->flags & HOOK_FLAGS) == 0 &&
              matches_expected(copied->so, 96, 80),
          "BITMAP_CopyBitmap of a device bitmap is an exact unhooked engine bitmap");
    if (copied != nullptr) {
        ++copied->refs;
        delete_object(*copied);
    }

    // Partial readback (GetBitmapBits of three rows).
    RECTL rows{};
    RECTL partial{};
    Rx7900DevBitmapRows(96 * 4 * 2 + 8, so.sizlBitmap, BMF_32BPP, &rows, &partial);
    SURFOBJ* part = Rx7900DevBitmapShadow(&engine, &so, &rows);
    bool rows_exact = part != nullptr && rows.bottom == 3;
    for (LONG y = 0; rows_exact && y < 80; ++y) {
        for (LONG x = 0; x < 96; ++x) {
            const uint32_t want = y < 3 ? expected_pixel(x, y) : 0;
            rows_exact = rows_exact && *engine_pixel(*part, x, y) == want;
        }
    }

    check(rows_exact, "a partial readback fills exactly the requested rows");
    if (part != nullptr) {
        Rx7900DevBitmapDiscard(&engine, part);
    }

    // DeleteObject: one DrvDeleteDeviceBitmap, the VRAM surface is destroyed.
    const unsigned destroyed_before = backend.destroyed;
    ++surface->refs;
    delete_object(*surface);
    check(backend.destroyed == destroyed_before + 1 && pdev.refs == 1 && !surface->used,
          "deleting the bitmap calls DrvDeleteDeviceBitmap once and frees its VRAM surface");
}

void declines(EnginePdev& pdev) {
    const unsigned created_before = backend.created;
    check(create(pdev, 16, 16) == nullptr && create(pdev, 4, 400) == nullptr && backend.created == created_before,
          "radeondisp refuses small bitmaps: GDI keeps engine bitmaps");
    check(Rx7900DevBitmapCreate(&engine, hdev_of(pdev), SIZEL{96, 96}, BMF_24BPP, nullptr) == nullptr,
          "a format other than the PDEV's is never offered");
    HBITMAP handles[abi::max_session_surfaces + 2]{};
    unsigned created = 0;
    for (HBITMAP& handle : handles) {
        handle = create(pdev, 64, 64);
        created += handle != nullptr ? 1U : 0U;
    }

    check(created > 0 && created < abi::max_session_surfaces + 2 && pdev.lock_depth == 0,
          "beyond the session surface budget radeondisp declines and GDI keeps engine bitmaps");
    const unsigned destroyed_before = backend.destroyed;
    for (HBITMAP handle : handles) {
        if (handle != nullptr) {
            GdiSurface& surface = *gdi_of(reinterpret_cast<HSURF>(handle));
            ++surface.refs;
            delete_object(surface);
        }
    }

    check(backend.destroyed == destroyed_before + created && pdev.refs == 1,
          "every admitted bitmap is deleted exactly once");
    pdev.acceleration = 1;
    check(create(pdev, 64, 64) == nullptr, "acceleration level 1 keeps engine bitmaps with the production driver");
    pdev.acceleration = 0;
}

// NtGdiExtFloodFill on the opaque radeondisp primary (pvScan0 NULL): the guard's
// shadow, filled through the production DrvCopyBits, equals the pinned DIB fill
// of an equivalent engine bitmap.
namespace flood = rx7900::tests::floodfill;

SURFOBJ* APIENTRY flood_shadow(SURFOBJ* like) {
    // SURFACE_AllocSurface(STYPE_BITMAP, ...) as the fillshap.c glue.
    return &alloc_surface(STYPE_BITMAP, like->sizlBitmap, like->iBitmapFormat, 0, 0, nullptr)->so;
}

VOID APIENTRY flood_discard(SURFOBJ* shadow) { delete_object(gdi_of(shadow)); }

BOOL APIENTRY flood_fill(SURFOBJ* target, BRUSHOBJ* brush, RECTL* rect, POINTL* origin, ULONG color, UINT type) {
    return flood::fill_solid(*target, *brush, *rect, *origin, color, type);
}

const RX7900_FLOODFILL_ENGINE flood_engine = {flood_shadow, flood_discard, copy_bits, flood_fill};

uint32_t maze(LONG x, LONG y) {
    if ((x % 23 == 11 && y % 31 != 15) || (y % 19 == 9 && x % 41 != 20)) {
        return 0x00ffffffU;
    }

    return 0;
}

void primary_flood_fill(EnginePdev& pdev) {
    SURFOBJ& primary = pdev.surface->so;
    check(primary.iType == STYPE_DEVICE && primary.pvScan0 == nullptr,
          "the radeondisp primary is opaque: the pinned DIB flood fill would read a NULL pvScan0");
    struct Case {
        RECTL rect;
        POINTL origin;
        ULONG color;
        UINT type;
        const char* name;
    } const cases[] = {
        {{0, 0, 320, 240}, {3, 3}, 0, FLOODFILLSURFACE, "a surface fill of the whole opaque primary equals the DIB fill"},
        {{30, 20, 200, 170}, {31, 21}, 0, FLOODFILLSURFACE, "a surface fill inside a region box equals the DIB fill"},
        {{0, 0, 320, 240}, {40, 40}, 0x00ffffffU, FLOODFILLBORDER, "a border fill of the opaque primary equals the DIB fill"},
    };
    for (const Case& item : cases) {
        for (LONG y = 0; y < static_cast<LONG>(screen_height); ++y) {
            for (LONG x = 0; x < static_cast<LONG>(screen_width); ++x) {
                reinterpret_cast<uint32_t*>(backend.framebuffer)[y * screen_width + x] = maze(x, y);
            }
        }

        GdiSurface& reference = *alloc_surface(STYPE_BITMAP, SIZEL{static_cast<LONG>(screen_width),
                                               static_cast<LONG>(screen_height)}, BMF_32BPP, BMF_TOPDOWN, 0, nullptr);
        memcpy(reference.so.pvBits, backend.framebuffer, sizeof(backend.framebuffer));
        BRUSHOBJ brush{};
        brush.iSolidColor = 0x00c08040U;
        const BOOL expected = flood::fill_solid(reference.so, brush, item.rect, item.origin, item.color, item.type);
        const unsigned before = unlocked_driver_calls;
        // DC_vPrepareDCsForBlit holds the display DC's PDEV device lock.
        lock_device(hdev_of(pdev));
        const BOOL done = Rx7900FloodFillOpaque(&flood_engine, &primary, gdi_of(&primary).flags, &brush, &item.rect,
                                                &item.origin, item.color, item.type);
        unlock_device(hdev_of(pdev));
        unsigned changed = 0;
        for (LONG y = 0; y < static_cast<LONG>(screen_height); ++y) {
            for (LONG x = 0; x < static_cast<LONG>(screen_width); ++x) {
                changed += reinterpret_cast<uint32_t*>(backend.framebuffer)[y * screen_width + x] != maze(x, y);
            }
        }

        check(done == expected && memcmp(backend.framebuffer, reference.so.pvBits, sizeof(backend.framebuffer)) == 0 &&
                  changed != 0 && unlocked_driver_calls == before,
              item.name);
        delete_object(reference);
    }

    BRUSHOBJ brush{};
    const RECTL whole{0, 0, 320, 240};
    const POINTL origin{3, 3};
    check(!Rx7900FloodFillOpaque(&flood_engine, &primary, 0, &brush, &whole, &origin, 0, FLOODFILLSURFACE),
          "an opaque target without CopyBits/BitBlt hooks fails instead of crashing");
}

// The pinned PDEVOBJ_bSwitchMode order: disable the old display, create the new
// PDEV, PDEVOBJ_bDynamicModeChange (the bridge converts first), then release
// the old driver instance.
void mode_change() {
    EnginePdev& persistent = pdevs[0];
    HBITMAP kept[3]{};
    for (HBITMAP& handle : kept) {
        handle = create(persistent, 96, 80);
        GdiSurface* surface = gdi_of(reinterpret_cast<HSURF>(handle));
        if (surface != nullptr) {
            fill(persistent, &surface->so, RECTL{10, 5, 70, 50}, 0x00336699U);
            GdiSurface& source = *alloc_surface(STYPE_BITMAP, SIZEL{96, 80}, BMF_32BPP, BMF_TOPDOWN, 0, nullptr);
            for (LONG y = 0; y < 80; ++y) {
                for (LONG x = 0; x < 96; ++x) {
                    *engine_pixel(source.so, x, y) = 0x00ffeeddU;
                }
            }

            RECTL a{70, 40, 96, 80};
            RECTL b{40, 50, 70, 80};
            Rx7900DevBitmapStore(&engine, &surface->so, &source.so, &a);
            Rx7900DevBitmapStore(&engine, &surface->so, &source.so, &b);
            delete_object(source);
        }
    }

    check(kept[0] != nullptr && kept[1] != nullptr && kept[2] != nullptr && persistent.refs == 4,
          "three device bitmaps live on the switching PDEV");
    PFN_DrvAssertMode assert_mode = ddi<PFN_DrvAssertMode>(persistent.functions[INDEX_DrvAssertMode]);
    lock_device(hdev_of(persistent));
    check(assert_mode(persistent.dhpdev, FALSE), "the old display is disabled first");
    persistent.disabled = true;
    check(create(persistent, 96, 80) == nullptr, "a disabled PDEV keeps engine bitmaps");
    check(enable_pdev(pdevs[1]), "the new driver instance is enabled while the old one exists");
    const unsigned destroyed_before = backend.destroyed;
    const bool converted = Rx7900DevBitmapConvertDevice(&engine, hdev_of(persistent)) &&
                           Rx7900DevBitmapConvertDevice(&engine, hdev_of(pdevs[1]));
    check(converted && backend.destroyed == destroyed_before + 3 && persistent.refs == 1 &&
              unlocked_driver_calls == 0,
          "every device bitmap converts before the switch: read back, DrvDeleteDeviceBitmap once each");
    bool exact = true;
    for (HBITMAP handle : kept) {
        GdiSurface* surface = gdi_of(reinterpret_cast<HSURF>(handle));
        exact = exact && surface != nullptr && surface->so.iType == STYPE_BITMAP &&
                (surface->flags & (HOOK_FLAGS | marker_flag)) == 0 && surface->so.fjBitmap == pool_flag &&
                matches_expected(surface->so, 96, 80);
    }

    check(exact, "the converted bitmaps are engine DDBs holding exactly the device pixels");

    // PDEVOBJ_bDynamicModeChange swaps the instances; the old one is released.
    EnginePdev old = persistent;
    persistent.dhpdev = pdevs[1].dhpdev;
    persistent.surface = pdevs[1].surface;
    persistent.devinfo = pdevs[1].devinfo;
    persistent.disabled = false;
    ddi<PFN_DrvCompletePDEV>(persistent.functions[INDEX_DrvCompletePDEV])(persistent.dhpdev,
                                                                                       hdev_of(persistent));
    unlock_device(hdev_of(persistent));
    pdevs[1] = EnginePdev{};
    disable_pdev(old);
    check(assert_mode(persistent.dhpdev, TRUE), "the new instance takes the display");

    // The persistent PDEV now creates device bitmaps on the new instance.
    const HBITMAP fresh = create(persistent, 96, 80);
    check(fresh != nullptr && gdi_of(reinterpret_cast<HSURF>(fresh))->so.dhpdev == persistent.dhpdev,
          "after the switch new compatible bitmaps belong to the new driver instance");
    if (fresh != nullptr) {
        GdiSurface& surface = *gdi_of(reinterpret_cast<HSURF>(fresh));
        ++surface.refs;
        delete_object(surface);
    }

    const unsigned destroyed_after = backend.destroyed;
    for (HBITMAP handle : kept) {
        GdiSurface* surface = gdi_of(reinterpret_cast<HSURF>(handle));
        if (surface != nullptr) {
            ++surface->refs;
            delete_object(*surface);
        }
    }

    check(backend.destroyed == destroyed_after && persistent.refs == 1,
          "converted bitmaps are freed as engine bitmaps without another driver call");
}

} // namespace

int main() {
    configure_miniport();
    check(DrvEnableDriver(DDI_DRIVER_VERSION_NT5, sizeof(driver), &driver) && driver.c != 0,
          "the production radeondisp DRVFN table is published");
    check(function(INDEX_DrvCreateDeviceBitmap) != nullptr && function(INDEX_DrvDeleteDeviceBitmap) != nullptr &&
              function(INDEX_DrvCopyBits) != nullptr && function(INDEX_DrvEscape) != nullptr,
          "radeondisp exports every function the bridge needs");
    check(enable_pdev(pdevs[0]), "a radeondisp PDEV and primary are enabled on the production dispatcher");
    identity_reply(pdevs[0]);
    bitmap_lifecycle(pdevs[0]);
    declines(pdevs[0]);
    primary_flood_fill(pdevs[0]);
    mode_change();
    check(registry_depth == 0 && pdevs[0].lock_depth == 0, "locks are balanced");
    disable_pdev(pdevs[0]);
    DrvDisableDriver();
    printf("radeon_devbitmap_ddi: %u checks, %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

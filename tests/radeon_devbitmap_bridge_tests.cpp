/*
Module: rx7900.tests.radeon_devbitmap_bridge
Purpose: Check the win32k device-bitmap bridge: identity and device gating, driver
result validation, create/delete pairing (exactly once, through cleanup or
conversion), registry capacity, device-lock discipline, every reader/writer shadow
path against the pinned engine semantics, in-place conversion and declines.
Created: 2026-10-24
Architecture: Production radeon_devbitmap_bridge.cpp and radeon_line_identity.hpp
against a fake engine table (the role of the patch's surface.c glue) and a fake
radeondisp DDI whose DrvEscape answers with the production make_identity. The
GetBitmapBits/SetBitmapBits loops are test-only ports of the pinned
win32ss/gdi/ntgdi/bitmaps.c UnsafeGetBitmapBits/UnsafeSetBitmapBits (identical in
a97c8a2 and 822e864). No kernel, hardware or VM is involved.
*/
#include <stdarg.h>
#include <windef.h>
#include <wingdi.h>
#include <winddi.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "radeon_devbitmap_bridge.hpp"
#include "radeon_line_identity.hpp"
#include "dib_floodfill_reference.hpp"

namespace {

namespace line = rx7900::reactos::radeon_line;

unsigned checks = 0;
unsigned failures = 0;

// The fake engine table (defined below) for SURFACE_vCleanup's bridge call.
const RX7900_DEVBITMAP_ENGINE* engine_table();

void check(bool condition, const char* message) {
    ++checks;
    if (!condition) {
        ++failures;
        printf("FAIL %s\n", message);
    }
}

// Engine constants of the pinned surface.h (values only matter to the fake).
constexpr FLONG engine_hooks = HOOK_FLAGS;
constexpr FLONG marker_flag = 0x00200000;   // DRIVER_CREATED_SURFACE
constexpr FLONG api_flag = 0x04000000;      // API_BITMAP
constexpr USHORT pool_flag = 0x100;         // BMF_POOLALLOC
constexpr FLONG radeon_hooks = HOOK_BITBLT | HOOK_COPYBITS | HOOK_FILLPATH | HOOK_SYNCHRONIZE |
                               HOOK_STRETCHBLT | HOOK_STRETCHBLTROP | HOOK_LINETO;

// ---------------------------------------------------------------------------
// Fake GDI objects.
// ---------------------------------------------------------------------------
struct FakePdev;

// A device bitmap's pixels as the fake driver keeps them (32 bpp, top-down).
struct DeviceStore {
    bool alive = false;
    unsigned deletes = 0;
    LONG width = 0;
    LONG height = 0;
    uint32_t pixels[96 * 96]{};
};

struct FakeSurface {
    SURFOBJ so{};
    bool used = false;
    bool deleted = false;
    bool freed = false;
    int refs = 0;
    FLONG flags = 0;
    void* palette = nullptr;
    bool adopted = false;
    bool own_bits = false;   // bits allocated by create_shadow (freed with the object)
};

constexpr unsigned max_surfaces = 700;
FakeSurface surfaces[max_surfaces];
DeviceStore stores[max_surfaces];

struct FakePdev {
    char instance = 0;   // its address is the DHPDEV
    bool display = true;
    bool disabled = false;
    ULONG acceleration = 0;
    void* palette = nullptr;
    FakeSurface* primary = nullptr;
    PFN_DrvEscape escape = nullptr;
    PFN_DrvCreateDeviceBitmap create = nullptr;
    PFN_DrvDeleteDeviceBitmap destroy = nullptr;
    PFN_DrvCopyBits copy = nullptr;
    int lock_depth = 0;
    int max_depth = 0;
    long refs = 1;
};

FakePdev pdevs[3];
char palette_a = 0;
char palette_b = 0;

struct Counters {
    unsigned creates = 0;
    unsigned deletes = 0;
    unsigned copies = 0;
    unsigned escapes = 0;
    unsigned unlocked_driver_calls = 0;
    unsigned registry_outside_device = 0;
    unsigned double_deletes = 0;
    unsigned bits_allocated = 0;
    unsigned bits_freed = 0;
    unsigned shadows = 0;
    unsigned registry_depth = 0;
    unsigned unique = 100;
} counters{};

// Failure injection.
enum class Escape { current, refuses, short_reply, bad_magic, ignores_request, no_device_bit, unknown_bit };
enum class Create { normal, decline, wrong_type, wrong_size, wrong_format, wrong_hdev, wrong_dhpdev, null_dhsurf,
                    with_bits, missing_hooks, wrong_palette, foreign_handle, already_marked };
Escape escape_mode = Escape::current;
Create create_mode = Create::normal;
bool copy_fails = false;
unsigned shadow_failures_left = 0;
bool shadow_wrong_layout = false;
bool shadow_wrong_format = false;
bool allocate_fails = false;
// A conversion that finishes while a reader waits for the device lock.
SURFOBJ* converted_while_waiting = nullptr;

HBITMAP handle_of(unsigned index) {
    return reinterpret_cast<HBITMAP>(static_cast<uintptr_t>(0x10000U + index * 4U));
}

FakeSurface* from_handle(HBITMAP handle) {
    const uintptr_t value = reinterpret_cast<uintptr_t>(handle);
    if (value < 0x10000U || (value - 0x10000U) % 4U != 0 || (value - 0x10000U) / 4U >= max_surfaces) {
        return nullptr;
    }

    return &surfaces[(value - 0x10000U) / 4U];
}

FakeSurface& fake(SURFOBJ* so) {
    return *reinterpret_cast<FakeSurface*>(reinterpret_cast<uint8_t*>(so) - offsetof(FakeSurface, so));
}

FakePdev& pdev_of(HDEV hdev) { return *reinterpret_cast<FakePdev*>(hdev); }
HDEV hdev_of(FakePdev& pdev) { return reinterpret_cast<HDEV>(&pdev); }
DHPDEV dhpdev_of(FakePdev& pdev) { return reinterpret_cast<DHPDEV>(&pdev.instance); }

FakePdev* pdev_of_dhpdev(DHPDEV dhpdev) {
    for (FakePdev& pdev : pdevs) {
        if (dhpdev_of(pdev) == dhpdev) {
            return &pdev;
        }
    }

    return nullptr;
}

FakeSurface* allocate_surface() {
    for (unsigned index = 0; index < max_surfaces; ++index) {
        if (!surfaces[index].used) {
            surfaces[index] = FakeSurface{};
            surfaces[index].used = true;
            surfaces[index].so.hsurf = reinterpret_cast<HSURF>(handle_of(index));
            return &surfaces[index];
        }
    }

    return nullptr;
}

DeviceStore* store_of(DHSURF dhsurf) {
    for (DeviceStore& store : stores) {
        if (reinterpret_cast<DHSURF>(&store) == dhsurf) {
            return &store;
        }
    }

    return nullptr;
}

// Engine bitmap of the pinned SURFACE_AllocSurface(STYPE_BITMAP, ...) layout.
void make_engine_bitmap(FakeSurface& surface, LONG width, LONG height, ULONG format, bool top_down) {
    const ULONG bpp = format == BMF_32BPP ? 32 : format == BMF_16BPP ? 16 : format == BMF_8BPP ? 8 : 1;
    const ULONG stride = ((width * bpp + 31) & ~31U) >> 3;
    surface.so.iType = STYPE_BITMAP;
    surface.so.sizlBitmap = {width, height};
    surface.so.iBitmapFormat = format;
    surface.so.cjBits = stride * height;
    surface.so.pvBits = calloc(1, surface.so.cjBits);
    surface.own_bits = true;
    surface.so.fjBitmap = top_down ? BMF_TOPDOWN : 0;
    surface.so.lDelta = top_down ? static_cast<LONG>(stride) : -static_cast<LONG>(stride);
    surface.so.pvScan0 = top_down ? surface.so.pvBits :
        static_cast<uint8_t*>(surface.so.pvBits) + surface.so.cjBits - stride;
    surface.flags = api_flag;
}

uint32_t* engine_pixel(SURFOBJ& so, LONG x, LONG y) {
    return reinterpret_cast<uint32_t*>(static_cast<uint8_t*>(so.pvScan0) + static_cast<intptr_t>(y) * so.lDelta) + x;
}

// SURFACE_vCleanup of the patch: the bridge first, then pool bits.
void free_object(FakeSurface& surface) {
    Rx7900DevBitmapCleanup(engine_table(), &surface.so);
    if (surface.so.pvBits != nullptr) {
        if ((surface.so.fjBitmap & pool_flag) != 0) {
            ++counters.bits_freed;
            free(surface.so.pvBits);
        } else if (surface.own_bits) {
            free(surface.so.pvBits);
        }
    }

    surface.freed = true;
    surface.used = false;
}

void dereference(FakeSurface& surface) {
    --surface.refs;
    if (surface.deleted && surface.refs == 0) {
        free_object(surface);
    }
}

// ---------------------------------------------------------------------------
// Fake radeondisp DDI.
// ---------------------------------------------------------------------------
void require_device_lock(FakePdev* pdev) {
    if (pdev == nullptr || pdev->lock_depth <= 0) {
        ++counters.unlocked_driver_calls;
    }
}

ULONG APIENTRY radeon_escape(SURFOBJ* surface, ULONG code, ULONG input_bytes, PVOID input, ULONG output_bytes,
                             PVOID output) {
    ++counters.escapes;
    uint32_t requested = 0;
    if (code != line::identity_escape || !line::decode_request(input_bytes, input, requested) ||
        output_bytes != sizeof(line::Identity) || surface == nullptr) {
        return 0;
    }

    line::Identity reply = line::make_identity(true, true, true, true, requested);
    switch (escape_mode) {
    case Escape::current: break;
    case Escape::refuses: return 0;
    case Escape::short_reply: memcpy(output, &reply, sizeof(reply)); return sizeof(reply) - 4;
    case Escape::bad_magic: reply.magic ^= 1U; break;
    case Escape::ignores_request: reply = line::make_identity(true, true); break;
    case Escape::no_device_bit: reply = line::make_identity(true, true, true, false, requested); break;
    case Escape::unknown_bit: reply.flags |= 0x100U; break;
    }

    memcpy(output, &reply, sizeof(reply));
    return sizeof(reply);
}

HBITMAP APIENTRY radeon_create(DHPDEV dhpdev, SIZEL size, ULONG format) {
    ++counters.creates;
    FakePdev* pdev = pdev_of_dhpdev(dhpdev);
    require_device_lock(pdev);
    if (pdev == nullptr || create_mode == Create::decline || format != BMF_32BPP || size.cx > 96 || size.cy > 96) {
        return nullptr;
    }

    if (create_mode == Create::foreign_handle) {
        return reinterpret_cast<HBITMAP>(static_cast<uintptr_t>(0x3));
    }

    FakeSurface* surface = allocate_surface();
    const unsigned index = static_cast<unsigned>(surface - surfaces);
    DeviceStore& store = stores[index];
    store = DeviceStore{};
    store.alive = true;
    store.width = size.cx;
    store.height = size.cy;
    SURFOBJ& so = surface->so;
    so.dhsurf = reinterpret_cast<DHSURF>(&store);
    so.hdev = hdev_of(*pdev);
    so.dhpdev = dhpdev;
    so.sizlBitmap = size;
    so.iBitmapFormat = format;
    so.iType = STYPE_DEVBITMAP;
    so.cjBits = static_cast<ULONG>(size.cx * 4 * size.cy);   // SURFACE_AllocSurface sets it for any type
    surface->flags = radeon_hooks;
    surface->palette = pdev->palette;
    surface->refs = 0;
    switch (create_mode) {
    case Create::wrong_type: so.iType = STYPE_BITMAP; break;
    case Create::wrong_size: so.sizlBitmap.cx += 1; break;
    case Create::wrong_format: so.iBitmapFormat = BMF_24BPP; break;
    case Create::wrong_hdev: so.hdev = hdev_of(pdevs[2]); break;
    case Create::wrong_dhpdev: so.dhpdev = dhpdev_of(pdevs[2]); break;
    case Create::null_dhsurf: so.dhsurf = nullptr; break;
    case Create::with_bits: so.pvScan0 = store.pixels; break;
    case Create::missing_hooks: surface->flags = HOOK_BITBLT; break;
    case Create::wrong_palette: surface->palette = &palette_b; break;
    case Create::already_marked: surface->flags |= marker_flag; break;
    default: break;
    }

    return handle_of(index);
}

VOID APIENTRY radeon_delete(DHSURF dhsurf) {
    ++counters.deletes;
    DeviceStore* store = store_of(dhsurf);
    // The driver's own PDEV lock: every bridged bitmap here belongs to pdevs[0/1].
    bool locked = false;
    for (FakePdev& pdev : pdevs) {
        locked = locked || pdev.lock_depth > 0;
    }

    if (!locked) {
        ++counters.unlocked_driver_calls;
    }

    if (store == nullptr || !store->alive) {
        ++counters.double_deletes;
        return;
    }

    ++store->deletes;
    store->alive = false;
}

bool device_bitmap(SURFOBJ* so) { return so != nullptr && so->iType == STYPE_DEVBITMAP; }

// DrvCopyBits for a device bitmap operand and an engine 32bpp bitmap: exact
// SRCCOPY of the clipped rectangle; no clip object or translation is expected.
BOOL APIENTRY radeon_copy(SURFOBJ* target, SURFOBJ* source, CLIPOBJ* clip, XLATEOBJ* translation, RECTL* rect,
                          POINTL* origin) {
    ++counters.copies;
    SURFOBJ* device = device_bitmap(target) ? target : source;
    require_device_lock(device != nullptr && device->hdev != nullptr ? &pdev_of(device->hdev) : nullptr);
    if (copy_fails || clip != nullptr || translation != nullptr) {
        return FALSE;
    }

    DeviceStore* store = store_of(device->dhsurf);
    if (store == nullptr || !store->alive) {
        return FALSE;
    }

    for (LONG y = rect->top; y < rect->bottom; ++y) {
        for (LONG x = rect->left; x < rect->right; ++x) {
            const LONG sx = origin->x + x - rect->left;
            const LONG sy = origin->y + y - rect->top;
            if (device_bitmap(target)) {
                store->pixels[y * store->width + x] = *engine_pixel(*source, sx, sy);
            } else {
                *engine_pixel(*target, x, y) = store->pixels[sy * store->width + sx];
            }
        }
    }

    return TRUE;
}

// ---------------------------------------------------------------------------
// Fake engine table (the patch's surface.c adapters).
// ---------------------------------------------------------------------------
BOOL APIENTRY lock_registry() {
    bool any = false;
    for (FakePdev& pdev : pdevs) {
        any = any || pdev.lock_depth > 0;
    }

    if (!any) {
        ++counters.registry_outside_device;
    }

    ++counters.registry_depth;
    return TRUE;
}

VOID APIENTRY unlock_registry() { --counters.registry_depth; }

VOID APIENTRY lock_device(HDEV hdev) {
    FakePdev& pdev = pdev_of(hdev);
    if (converted_while_waiting != nullptr) {
        fake(converted_while_waiting).flags &= ~(marker_flag | engine_hooks);
        converted_while_waiting = nullptr;
    }

    ++pdev.lock_depth;
    if (pdev.lock_depth > pdev.max_depth) {
        pdev.max_depth = pdev.lock_depth;
    }
}

VOID APIENTRY unlock_device(HDEV hdev) { --pdev_of(hdev).lock_depth; }
VOID APIENTRY reference_device(HDEV hdev) { ++pdev_of(hdev).refs; }
VOID APIENTRY release_device(HDEV hdev) { --pdev_of(hdev).refs; }

BOOL APIENTRY describe_device(HDEV hdev, RX7900_DEVBITMAP_DEVICE* device) {
    FakePdev& pdev = pdev_of(hdev);
    *device = {};
    device->dhpdev = dhpdev_of(pdev);
    device->primary = pdev.primary != nullptr ? &pdev.primary->so : nullptr;
    device->palette = pdev.palette;
    device->display = pdev.display;
    device->disabled = pdev.disabled;
    device->acceleration_level = pdev.acceleration;
    device->escape = pdev.escape;
    device->create = pdev.create;
    device->destroy = pdev.destroy;
    device->copy = pdev.copy;
    return TRUE;
}

SURFOBJ* APIENTRY lock_bitmap(HBITMAP handle) {
    FakeSurface* surface = from_handle(handle);
    if (surface == nullptr || !surface->used || surface->deleted) {
        return nullptr;
    }

    ++surface->refs;
    return &surface->so;
}

VOID APIENTRY unlock_bitmap(SURFOBJ* so) { dereference(fake(so)); }

VOID APIENTRY delete_bitmap(SURFOBJ* so) {
    FakeSurface& surface = fake(so);
    surface.deleted = true;
    dereference(surface);
}

FLONG APIENTRY read_flags(SURFOBJ* so) { return fake(so).flags; }

VOID APIENTRY update_flags(SURFOBJ* so, FLONG set, FLONG clear) {
    fake(so).flags = (fake(so).flags | set) & ~clear;
}

PVOID APIENTRY palette_of(SURFOBJ* so) { return fake(so).palette; }
VOID APIENTRY adopt_bitmap(SURFOBJ* so) { fake(so).adopted = true; }

SURFOBJ* APIENTRY create_shadow(SURFOBJ* like) {
    if (shadow_failures_left != 0) {
        --shadow_failures_left;
        return nullptr;
    }

    FakeSurface* surface = allocate_surface();
    make_engine_bitmap(*surface, like->sizlBitmap.cx, like->sizlBitmap.cy - (shadow_wrong_layout ? 1 : 0),
                       shadow_wrong_format ? BMF_16BPP : like->iBitmapFormat, false);
    surface->palette = fake(like).palette;
    surface->refs = 1;
    ++counters.shadows;
    return &surface->so;
}

// EngCopyBits: a device bitmap operand dispatches its PDEV's DrvCopyBits.
BOOL APIENTRY copy_bits(SURFOBJ* target, SURFOBJ* source, RECTL* rect, POINTL* origin) {
    if (device_bitmap(target) || device_bitmap(source)) {
        SURFOBJ* device = device_bitmap(target) ? target : source;
        return pdev_of(device->hdev).copy(target, source, nullptr, nullptr, rect, origin);
    }

    for (LONG y = rect->top; y < rect->bottom; ++y) {
        for (LONG x = rect->left; x < rect->right; ++x) {
            *engine_pixel(*target, x, y) =
                *engine_pixel(*source, origin->x + x - rect->left, origin->y + y - rect->top);
        }
    }

    return TRUE;
}

PVOID APIENTRY allocate_bits(ULONG bytes) {
    if (allocate_fails) {
        return nullptr;
    }

    ++counters.bits_allocated;
    return calloc(1, bytes);
}

VOID APIENTRY free_bits(PVOID bits) {
    ++counters.bits_freed;
    free(bits);
}

ULONG APIENTRY next_unique() { return ++counters.unique; }

const RX7900_DEVBITMAP_ENGINE table = {
    engine_hooks, marker_flag, api_flag, pool_flag,
    lock_registry, unlock_registry, lock_device, unlock_device, reference_device, release_device,
    describe_device, lock_bitmap, unlock_bitmap, delete_bitmap, read_flags, update_flags, palette_of,
    adopt_bitmap, create_shadow, copy_bits, allocate_bits, free_bits, next_unique,
};

const RX7900_DEVBITMAP_ENGINE* engine_table() { return &table; }

const RX7900_DEVBITMAP_ENGINE* const engine = &table;

// ---------------------------------------------------------------------------
// Fixture helpers.
// ---------------------------------------------------------------------------
void reset_pdev(FakePdev& pdev) {
    pdev.display = true;
    pdev.disabled = false;
    pdev.acceleration = 0;
    pdev.palette = &palette_a;
    pdev.escape = radeon_escape;
    pdev.create = radeon_create;
    pdev.destroy = radeon_delete;
    pdev.copy = radeon_copy;
    pdev.lock_depth = 0;
    pdev.max_depth = 0;
    pdev.refs = 1;
    if (pdev.primary == nullptr) {
        pdev.primary = allocate_surface();
        pdev.primary->so.iType = STYPE_DEVICE;
        pdev.primary->so.iBitmapFormat = BMF_32BPP;
        pdev.primary->so.sizlBitmap = {1024, 768};
        pdev.primary->so.hdev = hdev_of(pdev);
        pdev.primary->refs = 1;
    }
}

void reset() {
    for (FakePdev& pdev : pdevs) {
        reset_pdev(pdev);
    }

    counters = Counters{};
    escape_mode = Escape::current;
    create_mode = Create::normal;
    copy_fails = false;
    shadow_failures_left = 0;
    shadow_wrong_layout = false;
    shadow_wrong_format = false;
    allocate_fails = false;
    converted_while_waiting = nullptr;
}

bool balanced() {
    bool ok = counters.registry_depth == 0;
    for (FakePdev& pdev : pdevs) {
        ok = ok && pdev.lock_depth == 0;
    }

    return ok;
}

HBITMAP create(FakePdev& pdev, LONG width = 64, LONG height = 64, void* palette = nullptr,
               ULONG format = BMF_32BPP) {
    return Rx7900DevBitmapCreate(engine, hdev_of(pdev), SIZEL{width, height}, format, palette);
}

// DeleteObject from an application (the object is freed at its last reference).
void delete_object(HBITMAP handle) {
    SURFOBJ* so = lock_bitmap(handle);
    if (so != nullptr) {
        delete_bitmap(so);
    }
}

DeviceStore& store(HBITMAP handle) { return *store_of(from_handle(handle)->so.dhsurf); }

uint32_t pattern(LONG x, LONG y, uint32_t seed) {
    return (static_cast<uint32_t>(x) * 0x9e3779b1U) ^ (static_cast<uint32_t>(y) * 0x85ebca77U) ^ seed;
}

void fill_store(DeviceStore& device, uint32_t seed) {
    for (LONG y = 0; y < device.height; ++y) {
        for (LONG x = 0; x < device.width; ++x) {
            device.pixels[y * device.width + x] = pattern(x, y, seed);
        }
    }
}

// Test-only port of the pinned UnsafeGetBitmapBits (win32ss/gdi/ntgdi/bitmaps.c).
LONG port_get_bits(SURFOBJ& so, ULONG bytes, uint8_t* output) {
    const ULONG width = static_cast<ULONG>(so.sizlBitmap.cx);
    const ULONG height = static_cast<ULONG>(so.sizlBitmap.cy);
    const ULONG bpp = 32;
    const uint8_t* source = static_cast<const uint8_t*>(so.pvScan0);
    uint8_t* target = output;
    const LONG delta_source = so.lDelta;
    const LONG delta_source_abs = labs(delta_source);
    const LONG delta_target = (((width * bpp + 15) & ~15U) >> 3);
    const ULONG source_bytes = height * delta_source_abs;
    bytes = bytes < height * delta_target ? bytes : height * delta_target;
    ULONG source_index = 0;
    ULONG target_index = 0;
    for (ULONG y = 0; y < height; ++y) {
        if (source_index + delta_source_abs > source_bytes || target_index + delta_target > bytes) {
            const LONG left = static_cast<LONG>(source_bytes - source_index < bytes - target_index ?
                                                source_bytes - source_index : bytes - target_index);
            memcpy(target, source, left);
            target_index += left;
            break;
        }

        memcpy(target, source, delta_target);
        source += delta_source;
        target += delta_target;
        source_index += delta_source_abs;
        target_index += delta_target;
    }

    return static_cast<LONG>(target_index);
}

// Test-only port of the pinned UnsafeSetBitmapBits.
LONG port_set_bits(SURFOBJ& so, ULONG bytes, const uint8_t* input) {
    const ULONG width = static_cast<ULONG>(so.sizlBitmap.cx);
    const ULONG height = static_cast<ULONG>(so.sizlBitmap.cy);
    uint8_t* target = static_cast<uint8_t*>(so.pvScan0);
    const uint8_t* source = input;
    const LONG delta_target = so.lDelta;
    const LONG delta_target_abs = labs(delta_target);
    const LONG delta_source = (((width * 32 + 15) & ~15U) >> 3);
    const ULONG target_bytes = delta_target_abs * height;
    const ULONG source_bytes = delta_source * height;
    bytes = bytes < source_bytes ? bytes : source_bytes;
    ULONG source_index = 0;
    ULONG target_index = 0;
    for (ULONG y = 0; y < height; ++y) {
        if (source_index + delta_source > bytes || target_index + delta_target_abs > target_bytes) {
            const LONG left = static_cast<LONG>(bytes - source_index < target_bytes - target_index ?
                                                bytes - source_index : target_bytes - target_index);
            memcpy(target, source, left);
            source_index += left;
            break;
        }

        memcpy(target, source, delta_source);
        source += delta_source;
        target += delta_target;
        source_index += delta_source;
        target_index += delta_target_abs;
    }

    return static_cast<LONG>(source_index);
}

// An engine bitmap holding the same pixels as a device store (the reference).
FakeSurface& engine_copy_of(DeviceStore& device) {
    FakeSurface& surface = *allocate_surface();
    make_engine_bitmap(surface, device.width, device.height, BMF_32BPP, false);
    surface.refs = 1;
    for (LONG y = 0; y < device.height; ++y) {
        for (LONG x = 0; x < device.width; ++x) {
            *engine_pixel(surface.so, x, y) = device.pixels[y * device.width + x];
        }
    }

    return surface;
}

bool same_pixels(DeviceStore& device, SURFOBJ& so) {
    for (LONG y = 0; y < device.height; ++y) {
        for (LONG x = 0; x < device.width; ++x) {
            if (*engine_pixel(so, x, y) != device.pixels[y * device.width + x]) {
                return false;
            }
        }
    }

    return true;
}

void release_engine(FakeSurface& surface) {
    surface.deleted = true;
    dereference(surface);
}

// ---------------------------------------------------------------------------
// Cases.
// ---------------------------------------------------------------------------
void identity_contract() {
    const line::Identity v1 = line::make_identity(true, true, true, true, 0);
    check(v1.flags == line::base_flags, "an input-free query never reports the device-bitmap bit");
    const line::Identity granted = line::make_identity(true, true, false, true, line::flag_device_bitmaps);
    check(line::admits_requested(granted, line::flag_device_bitmaps, line::flag_device_bitmaps),
          "a requested and available device-bitmap capability is admitted");
    check(!line::admits(granted, line::flag_gpu_lines),
          "an old (version-1) kernel refuses a reply carrying the device-bitmap bit");
    check(!line::admits_requested(granted, line::flag_styled_stroke, line::flag_styled_stroke),
          "the stroke bridge never accepts an unrequested device-bitmap bit");
    check(line::make_identity(true, true, true, line::flag_device_bitmaps).flags == v1.flags,
          "the four-argument reply (old driver) never reports device bitmaps");
    check(line::make_identity(true, true, true, false, line::flag_device_bitmaps).flags == v1.flags,
          "an unavailable device-bitmap capability is never reported");
    check((line::flag_device_bitmaps & (line::base_flags | line::flag_styled_stroke | 8U)) == 0,
          "the device-bitmap bit is distinct and leaves bit 8 unassigned");
}

void identity_gating() {
    struct Case {
        Escape mode;
        const char* name;
    } const cases[] = {
        {Escape::refuses, "a refused identity escape keeps an engine bitmap"},
        {Escape::short_reply, "a short identity reply keeps an engine bitmap"},
        {Escape::bad_magic, "a wrong identity magic keeps an engine bitmap"},
        {Escape::ignores_request, "an older radeondisp ignoring the request keeps an engine bitmap"},
        {Escape::no_device_bit, "a radeondisp without offscreen surfaces keeps an engine bitmap"},
        {Escape::unknown_bit, "a reply with an unknown bit keeps an engine bitmap"},
    };
    for (const Case& item : cases) {
        reset();
        escape_mode = item.mode;
        const HBITMAP handle = create(pdevs[0]);
        check(handle == nullptr && counters.escapes == 1 && counters.creates == 0 && balanced(), item.name);
    }

    reset();
    const HBITMAP handle = create(pdevs[0]);
    check(handle != nullptr && counters.escapes == 1 && counters.creates == 1,
          "an identified radeondisp is offered DrvCreateDeviceBitmap once");
    delete_object(handle);
}

void device_gating() {
    struct Case {
        void (*setup)(FakePdev&);
        const char* name;
    } const cases[] = {
        {[](FakePdev& p) { p.acceleration = 1; }, "acceleration level 1 keeps the engine bitmap (pdevobj.c rule)"},
        {[](FakePdev& p) { p.acceleration = 5; }, "acceleration level 5 keeps the engine bitmap"},
        {[](FakePdev& p) { p.create = nullptr; }, "a filtered DrvCreateDeviceBitmap keeps the engine bitmap"},
        {[](FakePdev& p) { p.destroy = nullptr; }, "a driver without DrvDeleteDeviceBitmap is never offered one"},
        {[](FakePdev& p) { p.copy = nullptr; }, "a driver without DrvCopyBits is never offered one"},
        {[](FakePdev& p) { p.escape = nullptr; }, "a driver without DrvEscape is never offered one"},
        {[](FakePdev& p) { p.disabled = true; }, "a disabled PDEV keeps the engine bitmap"},
        {[](FakePdev& p) { p.display = false; }, "a non-display or meta PDEV keeps the engine bitmap"},
        {[](FakePdev& p) { p.palette = nullptr; }, "a PDEV without a palette keeps the engine bitmap"},
    };
    for (const Case& item : cases) {
        reset();
        item.setup(pdevs[0]);
        const HBITMAP handle = create(pdevs[0]);
        check(handle == nullptr && counters.creates == 0 && counters.escapes == 0 && balanced(), item.name);
    }

    reset();
    FakeSurface* primary = pdevs[0].primary;
    pdevs[0].primary = nullptr;
    check(create(pdevs[0]) == nullptr && counters.creates == 0, "a PDEV without its surface keeps the engine bitmap");
    pdevs[0].primary = primary;

    reset();
    check(create(pdevs[0], 64, 64, nullptr, BMF_16BPP) == nullptr && counters.creates == 0,
          "a request in another format than the PDEV's keeps the engine bitmap");
    check(create(pdevs[0], 64, 64, &palette_b) == nullptr && counters.creates == 0,
          "a selected bitmap with a foreign palette keeps the engine bitmap (no driver call)");
    check(create(pdevs[0], 0, 64) == nullptr && create(pdevs[0], 64, -1) == nullptr && counters.escapes == 0,
          "an empty size never reaches the driver");
    const HBITMAP same = create(pdevs[0], 64, 64, &palette_a);
    check(same != nullptr, "a selected bitmap with the PDEV palette may become a device bitmap");
    delete_object(same);

    reset();
    create_mode = Create::decline;
    check(create(pdevs[0]) == nullptr && counters.creates == 1 && counters.deletes == 0 && balanced(),
          "a driver decline means an engine bitmap, as on NT");
}

void result_validation() {
    struct Case {
        Create mode;
        bool deletes;
        const char* name;
    } const cases[] = {
        {Create::wrong_type, false, "a driver result that is not STYPE_DEVBITMAP is deleted and declined"},
        {Create::wrong_size, true, "a device bitmap of another size is deleted once and declined"},
        {Create::wrong_format, true, "a device bitmap of another format is deleted once and declined"},
        {Create::wrong_hdev, true, "a device bitmap associated with another HDEV is deleted once and declined"},
        {Create::wrong_dhpdev, true, "a device bitmap of another DHPDEV is deleted once and declined"},
        {Create::null_dhsurf, false, "a device bitmap without DHSURF is declined"},
        {Create::with_bits, true, "a device bitmap exposing bits is deleted once and declined"},
        {Create::missing_hooks, true, "a device bitmap without BitBlt/CopyBits hooks is deleted once and declined"},
        {Create::wrong_palette, true, "a device bitmap with a foreign palette is deleted once and declined"},
        {Create::already_marked, true, "a result already carrying the bridge marker is deleted once and declined"},
    };
    for (const Case& item : cases) {
        reset();
        create_mode = item.mode;
        const HBITMAP handle = create(pdevs[0]);
        bool freed = true;
        for (FakeSurface& candidate : surfaces) {
            freed = freed && !(candidate.used && candidate.so.iType != STYPE_DEVICE && !candidate.deleted &&
                               candidate.so.dhsurf != nullptr);
        }

        check(handle == nullptr && counters.deletes == (item.deletes ? 1U : 0U) && counters.double_deletes == 0 &&
                  pdevs[0].refs == 1 && freed && balanced(),
              item.name);
    }

    reset();
    create_mode = Create::foreign_handle;
    check(create(pdevs[0]) == nullptr && counters.deletes == 0 && balanced(),
          "a handle that is no bitmap is never interpreted");
}

void create_delete_pairing() {
    reset();
    const HBITMAP handle = create(pdevs[0]);
    FakeSurface& surface = *from_handle(handle);
    check(handle != nullptr && (surface.flags & marker_flag) != 0 && (surface.flags & api_flag) != 0 &&
              surface.adopted && surface.refs == 0 && pdevs[0].refs == 2 && balanced(),
          "a created device bitmap is marked, an API bitmap, process-owned, unlocked and holds its PDEV");
    check(counters.unlocked_driver_calls == 0 && pdevs[0].max_depth >= 1,
          "DrvCreateDeviceBitmap runs under the PDEV device lock");
    check(Rx7900DevBitmapIsBridged(engine, &surface.so) && !Rx7900DevBitmapIsBridged(engine, &pdevs[0].primary->so),
          "only the created bitmap is bridged");

    DeviceStore& device = store(handle);
    delete_object(handle);
    check(device.deletes == 1 && !device.alive && counters.deletes == 1 && pdevs[0].refs == 1 && surface.freed &&
              counters.unlocked_driver_calls == 0 && balanced(),
          "SURFACE_vCleanup calls DrvDeleteDeviceBitmap exactly once under the device lock and releases the PDEV");

    // Deferred deletion: an object still referenced (selected in a DC) is
    // retired at its last dereference, not at DeleteObject.
    reset();
    const HBITMAP held = create(pdevs[0]);
    SURFOBJ* reference = lock_bitmap(held);
    delete_object(held);
    check(counters.deletes == 0 && pdevs[0].refs == 2, "a referenced bitmap is not retired at DeleteObject");
    unlock_bitmap(reference);
    check(counters.deletes == 1 && pdevs[0].refs == 1 && counters.double_deletes == 0,
          "the last dereference (DC deletion, process exit) retires it exactly once");

    // A second cleanup of the same body (impossible in GDI) is still harmless.
    reset();
    const HBITMAP twice = create(pdevs[0]);
    FakeSurface& body = *from_handle(twice);
    Rx7900DevBitmapCleanup(engine, &body.so);
    Rx7900DevBitmapCleanup(engine, &body.so);
    check(counters.deletes == 1 && counters.double_deletes == 0 && pdevs[0].refs == 1,
          "cleanup is idempotent: the marker is cleared with the retirement");
    body.deleted = true;
    body.refs = 1;
    dereference(body);
    check(counters.deletes == 1, "freeing a retired body does not call the driver again");

    // Engine bitmaps never reach the device lock or the driver.
    reset();
    FakeSurface& plain = *allocate_surface();
    make_engine_bitmap(plain, 64, 64, BMF_32BPP, false);
    plain.so.hdev = hdev_of(pdevs[0]);
    plain.refs = 1;
    release_engine(plain);
    check(counters.deletes == 0 && pdevs[0].max_depth == 0 && counters.bits_freed == 0,
          "cleanup of an engine compatible bitmap takes no device lock and calls no driver");
}

void registry_capacity() {
    reset();
    HBITMAP handles[RX7900_DEVBITMAP_CAPACITY + 1]{};
    unsigned created = 0;
    for (unsigned index = 0; index < RX7900_DEVBITMAP_CAPACITY; ++index) {
        handles[index] = create(pdevs[index % 2], 16, 16);
        created += handles[index] != nullptr ? 1U : 0U;
    }

    const unsigned deletes_before = counters.deletes;
    handles[RX7900_DEVBITMAP_CAPACITY] = create(pdevs[0], 16, 16);
    check(created == RX7900_DEVBITMAP_CAPACITY && handles[RX7900_DEVBITMAP_CAPACITY] == nullptr &&
              counters.deletes == deletes_before + 1 && counters.registry_outside_device == 0 && balanced(),
          "a full registry deletes the new driver bitmap once and keeps an engine bitmap");
    delete_object(handles[7]);
    handles[7] = create(pdevs[1], 16, 16);
    check(handles[7] != nullptr, "a retired bitmap frees its registry record");
    for (HBITMAP handle : handles) {
        if (handle != nullptr) {
            delete_object(handle);
        }
    }

    check(counters.deletes == counters.creates && counters.double_deletes == 0 && pdevs[0].refs == 1 &&
              pdevs[1].refs == 1,
          "every created device bitmap is deleted exactly once");
}

void readers_and_writers() {
    reset();
    const HBITMAP handle = create(pdevs[0], 40, 30);
    FakeSurface& surface = *from_handle(handle);
    DeviceStore& device = store(handle);
    fill_store(device, 0x1234U);
    FakeSurface& reference = engine_copy_of(device);

    // GetBitmapBits: rows for the byte count, read back under the lock.
    const ULONG read_sizes[] = {1U, 4U, 159U, 160U, 161U, 2400U, 4800U, 4801U, 100000U};
    for (ULONG bytes : read_sizes) {
        RECTL rows{};
        RECTL partial{};
        Rx7900DevBitmapRows(bytes, surface.so.sizlBitmap, BMF_32BPP, &rows, &partial);
        SURFOBJ* shadow = Rx7900DevBitmapShadow(engine, &surface.so, &rows);
        static uint8_t got[40 * 30 * 4 + 16];
        static uint8_t expected[40 * 30 * 4 + 16];
        memset(got, 0xa5, sizeof(got));
        memset(expected, 0xa5, sizeof(expected));
        const LONG n = port_get_bits(*shadow, bytes, got);
        const LONG m = port_get_bits(reference.so, bytes, expected);
        Rx7900DevBitmapDiscard(engine, shadow);
        char name[96];
        snprintf(name, sizeof(name), "GetBitmapBits of %lu bytes equals the engine bitmap's", bytes);
        check(n == m && memcmp(got, expected, sizeof(got)) == 0 && balanced(), name);
    }

    check(counters.unlocked_driver_calls == 0, "every readback DrvCopyBits holds the device lock");

    // SetBitmapBits: partial row pre-read, touched rows stored, rest unchanged.
    const ULONG write_sizes[] = {3U, 160U, 162U, 1000U, 4800U, 9000U};
    for (ULONG bytes : write_sizes) {
        static uint8_t input[40 * 30 * 4 + 64];
        for (unsigned index = 0; index < sizeof(input); ++index) {
            input[index] = static_cast<uint8_t>(index * 7U + bytes);
        }

        RECTL rows{};
        RECTL partial{};
        Rx7900DevBitmapRows(bytes, surface.so.sizlBitmap, BMF_32BPP, &rows, &partial);
        HDEV locked = Rx7900DevBitmapLock(engine, &surface.so);
        const bool has_partial = partial.right > partial.left;
        SURFOBJ* shadow = Rx7900DevBitmapShadow(engine, &surface.so, has_partial ? &partial : nullptr);
        const LONG n = port_set_bits(*shadow, bytes, input);
        const BOOL stored = Rx7900DevBitmapStore(engine, &surface.so, shadow, &rows);
        Rx7900DevBitmapDiscard(engine, shadow);
        Rx7900DevBitmapUnlock(engine, locked);
        const LONG m = port_set_bits(reference.so, bytes, input);
        char name[96];
        snprintf(name, sizeof(name), "SetBitmapBits of %lu bytes equals the engine bitmap's", bytes);
        check(locked != nullptr && stored && n == m && same_pixels(device, reference.so) && balanced(), name);
    }

    // BITMAP_CopyBitmap (patterns, GetIconInfo): an engine DDB copy.
    RECTL whole{0, 0, 40, 30};
    SURFOBJ* copy = Rx7900DevBitmapShadow(engine, &surface.so, &whole);
    const HBITMAP kept = Rx7900DevBitmapKeep(engine, copy);
    FakeSurface& copied = *from_handle(kept);
    check(kept != nullptr && copied.refs == 0 && copied.so.iType == STYPE_BITMAP && copied.flags == api_flag &&
              copied.palette == surface.palette && copied.so.lDelta < 0 && same_pixels(device, copied.so),
          "BITMAP_CopyBitmap returns an unhooked engine API bitmap with the same pixels and palette");
    delete_object(kept);

    // Flood fill: whole shadow, fill, store the fill rectangle back.
    {
        HDEV locked = Rx7900DevBitmapLock(engine, &surface.so);
        SURFOBJ* shadow = Rx7900DevBitmapShadow(engine, &surface.so, &whole);
        RECTL area{5, 4, 20, 12};
        for (LONG y = area.top; y < area.bottom; ++y) {
            for (LONG x = area.left; x < area.right; ++x) {
                if ((x + y) % 3 == 0) {
                    *engine_pixel(*shadow, x, y) = 0x00ff00ffU;
                    *engine_pixel(reference.so, x, y) = 0x00ff00ffU;
                }
            }
        }

        const BOOL stored = Rx7900DevBitmapStore(engine, &surface.so, shadow, &area);
        Rx7900DevBitmapDiscard(engine, shadow);
        Rx7900DevBitmapUnlock(engine, locked);
        check(stored && same_pixels(device, reference.so) && balanced(),
              "a flood fill on the shadow stores exactly the engine fill result");
    }

    // Store clipping and empty rectangles.
    {
        SURFOBJ* shadow = Rx7900DevBitmapShadow(engine, &surface.so, &whole);
        const unsigned copies = counters.copies;
        RECTL empty{10, 10, 10, 20};
        RECTL outside{-50, -50, -1, -1};
        check(Rx7900DevBitmapStore(engine, &surface.so, shadow, &empty) &&
                  Rx7900DevBitmapStore(engine, &surface.so, shadow, &outside) && counters.copies == copies,
              "an empty or outside store rectangle writes nothing and succeeds");
        RECTL beyond{30, 20, 500, 500};
        *engine_pixel(*shadow, 39, 29) = 0xdeadbeefU;
        check(Rx7900DevBitmapStore(engine, &surface.so, shadow, &beyond) && device.pixels[29 * 40 + 39] == 0xdeadbeefU,
              "a store rectangle is clipped to the bitmap");
        *engine_pixel(reference.so, 39, 29) = 0xdeadbeefU;
        Rx7900DevBitmapDiscard(engine, shadow);
    }

    // Read with nothing requested: zeroed shadow, no driver call.
    {
        const unsigned copies = counters.copies;
        SURFOBJ* shadow = Rx7900DevBitmapShadow(engine, &surface.so, nullptr);
        check(shadow != nullptr && *engine_pixel(*shadow, 3, 3) == 0 && counters.copies == copies,
              "a shadow without a read rectangle is zeroed and calls no driver");
        Rx7900DevBitmapDiscard(engine, shadow);
    }

    // Failures never return wrong pixels.
    copy_fails = true;
    check(Rx7900DevBitmapShadow(engine, &surface.so, &whole) == nullptr && balanced(),
          "a failed readback yields no shadow (the reader fails)");
    SURFOBJ* zero = Rx7900DevBitmapShadow(engine, &surface.so, nullptr);
    check(!Rx7900DevBitmapStore(engine, &surface.so, zero, &whole) && same_pixels(device, reference.so) && balanced(),
          "a failed store reports failure");
    Rx7900DevBitmapDiscard(engine, zero);
    copy_fails = false;
    shadow_failures_left = 1;
    check(Rx7900DevBitmapShadow(engine, &surface.so, &whole) == nullptr, "a failed shadow allocation fails the reader");
    check(counters.unlocked_driver_calls == 0, "every DrvCopyBits of the readers and writers held the device lock");

    release_engine(reference);
    delete_object(handle);
    check(counters.deletes == 1 && counters.double_deletes == 0 && pdevs[0].refs == 1,
          "reader and writer paths leave the pairing unchanged");
}

void lock_contract() {
    reset();
    const HBITMAP handle = create(pdevs[0]);
    FakeSurface& surface = *from_handle(handle);
    HDEV locked = Rx7900DevBitmapLock(engine, &surface.so);
    check(locked == hdev_of(pdevs[0]) && pdevs[0].lock_depth == 1, "a bridged bitmap locks its PDEV");
    Rx7900DevBitmapUnlock(engine, locked);
    FakeSurface& plain = *allocate_surface();
    make_engine_bitmap(plain, 8, 8, BMF_32BPP, true);
    plain.so.hdev = hdev_of(pdevs[0]);
    check(Rx7900DevBitmapLock(engine, &plain.so) == nullptr && pdevs[0].lock_depth == 0,
          "an engine bitmap takes no device lock");
    Rx7900DevBitmapUnlock(engine, nullptr);
    check(balanced(), "unlocking nothing is harmless");
    converted_while_waiting = &surface.so;
    check(Rx7900DevBitmapLock(engine, &surface.so) == nullptr && pdevs[0].lock_depth == 0,
          "a bitmap converted while the lock was awaited is released and treated as an engine bitmap");
    surface.flags |= marker_flag | radeon_hooks;
    plain.refs = 1;
    release_engine(plain);
    delete_object(handle);
}

void conversion() {
    reset();
    const HBITMAP handle = create(pdevs[0], 37, 21);
    FakeSurface& surface = *from_handle(handle);
    DeviceStore& device = store(handle);
    fill_store(device, 0xabcdU);
    FakeSurface& reference = engine_copy_of(device);
    const ULONG unique = surface.so.iUniq;
    check(Rx7900DevBitmapConvert(engine, &surface.so) && balanced(), "a device bitmap converts");
    const ULONG stride = 37 * 4;
    SURFOBJ& so = surface.so;
    check(so.iType == STYPE_BITMAP && so.dhsurf == nullptr && so.dhpdev == nullptr && so.hdev == hdev_of(pdevs[0]) &&
              so.cjBits == stride * 21 && so.lDelta == -static_cast<LONG>(stride) &&
              so.pvScan0 == static_cast<uint8_t*>(so.pvBits) + stride * 20 && so.fjBitmap == pool_flag &&
              so.iUniq != unique,
          "the converted SURFOBJ has the pinned engine DDB layout (bottom-up, pool bits, new iUniq)");
    check((surface.flags & (engine_hooks | marker_flag)) == 0 && (surface.flags & api_flag) != 0,
          "a converted bitmap keeps API_BITMAP and loses every hook and the marker");
    check(memcmp(so.pvBits, reference.so.pvBits, so.cjBits) == 0, "converted bits equal the engine bitmap's bytes");
    check(device.deletes == 1 && pdevs[0].refs == 1 && counters.unlocked_driver_calls == 0,
          "conversion deletes the driver bitmap once under the lock and releases the PDEV");
    check(Rx7900DevBitmapConvert(engine, &so) && device.deletes == 1 && !Rx7900DevBitmapIsBridged(engine, &so),
          "converting an engine bitmap is a no-op");
    HDEV locked = Rx7900DevBitmapLock(engine, &so);
    RECTL whole{0, 0, 37, 21};
    SURFOBJ* shadow = Rx7900DevBitmapShadow(engine, &so, &whole);
    check(locked == nullptr && shadow != nullptr && memcmp(shadow->pvBits, reference.so.pvBits, so.cjBits) == 0,
          "the reader path still works after a concurrent conversion (engine copy)");
    Rx7900DevBitmapDiscard(engine, shadow);
    const unsigned freed = counters.bits_freed;
    delete_object(handle);
    check(device.deletes == 1 && counters.double_deletes == 0 && counters.bits_freed == freed + 1,
          "a converted bitmap is freed by SURFACE_vCleanup's pool path, never deleted twice");
    release_engine(reference);

    // Failure keeps the device bitmap unchanged and tracked.
    struct Failure {
        void (*inject)();
        const char* name;
    } const failures_list[] = {
        {[] { shadow_failures_left = 1; }, "a failed shadow keeps the device bitmap unchanged"},
        {[] { copy_fails = true; }, "a failed readback keeps the device bitmap unchanged"},
        {[] { allocate_fails = true; }, "a failed bit allocation keeps the device bitmap unchanged"},
        {[] { shadow_wrong_layout = true; }, "a shadow of another layout keeps the device bitmap and frees the bits"},
    };
    for (const Failure& item : failures_list) {
        reset();
        const HBITMAP kept = create(pdevs[0], 32, 32);
        FakeSurface& body = *from_handle(kept);
        item.inject();
        const BOOL converted = Rx7900DevBitmapConvert(engine, &body.so);
        copy_fails = false;
        allocate_fails = false;
        shadow_wrong_layout = false;
        check(!converted && body.so.iType == STYPE_DEVBITMAP && body.so.pvBits == nullptr &&
                  Rx7900DevBitmapIsBridged(engine, &body.so) && counters.deletes == 0 && pdevs[0].refs == 2 &&
                  counters.bits_allocated == counters.bits_freed && balanced(),
              item.name);
        delete_object(kept);
        check(counters.deletes == 1 && pdevs[0].refs == 1, "a bitmap whose conversion failed is still deleted once");
    }
}

void mode_change() {
    reset();
    HBITMAP first[5]{};
    HBITMAP second[3]{};
    for (HBITMAP& handle : first) {
        handle = create(pdevs[0], 24, 24);
        fill_store(store(handle), static_cast<uint32_t>(reinterpret_cast<uintptr_t>(handle)));
    }

    for (HBITMAP& handle : second) {
        handle = create(pdevs[1], 24, 24);
    }

    check(pdevs[0].refs == 6 && pdevs[1].refs == 4, "each device bitmap holds one PDEV reference");
    check(Rx7900DevBitmapConvertDevice(engine, hdev_of(pdevs[0])) && balanced(),
          "every device bitmap of the switching PDEV converts");
    bool all_engine = true;
    bool pixels = true;
    for (HBITMAP handle : first) {
        FakeSurface& surface = *from_handle(handle);
        all_engine = all_engine && surface.so.iType == STYPE_BITMAP && !Rx7900DevBitmapIsBridged(engine, &surface.so);
        pixels = pixels && *engine_pixel(surface.so, 5, 7) ==
                               pattern(5, 7, static_cast<uint32_t>(reinterpret_cast<uintptr_t>(handle)));
    }

    bool others = true;
    for (HBITMAP handle : second) {
        others = others && Rx7900DevBitmapIsBridged(engine, &from_handle(handle)->so);
    }

    check(all_engine && pixels && pdevs[0].refs == 1 && counters.deletes == 5,
          "converted bitmaps keep their pixels, each driver bitmap is deleted once, PDEV references drop");
    check(others && pdevs[1].refs == 4, "device bitmaps of another PDEV stay device bitmaps");
    check(pdevs[0].lock_depth == 0 && pdevs[0].max_depth >= 1 && counters.unlocked_driver_calls == 0,
          "conversion of a PDEV holds its device lock for every driver call");
    check(Rx7900DevBitmapConvertDevice(engine, hdev_of(pdevs[0])) && counters.deletes == 5,
          "a second mode change finds nothing to convert");

    // A failure part way leaves the rest unchanged and fails the mode change.
    copy_fails = true;
    check(!Rx7900DevBitmapConvertDevice(engine, hdev_of(pdevs[1])) && balanced(),
          "a failed conversion fails the dynamic mode change");
    copy_fails = false;
    bool kept = true;
    for (HBITMAP handle : second) {
        kept = kept && Rx7900DevBitmapIsBridged(engine, &from_handle(handle)->so);
    }

    check(kept && pdevs[1].refs == 4 && counters.deletes == 5, "unconverted device bitmaps survive a failed switch");
    for (HBITMAP handle : first) {
        delete_object(handle);
    }

    for (HBITMAP handle : second) {
        delete_object(handle);
    }

    check(counters.deletes == 8 && counters.double_deletes == 0 && pdevs[0].refs == 1 && pdevs[1].refs == 1,
          "after conversion and cleanup every driver bitmap was deleted exactly once");
}

// ---------------------------------------------------------------------------
// Opaque flood fill (RX7900_FLOODFILL_OPAQUE).
// ---------------------------------------------------------------------------
namespace flood = rx7900::tests::floodfill;

bool flood_write_fails = false;
unsigned flood_copies = 0;

SURFOBJ* APIENTRY flood_shadow(SURFOBJ* like) { return create_shadow(like); }
VOID APIENTRY flood_discard(SURFOBJ* shadow) { delete_bitmap(shadow); }

BOOL APIENTRY flood_copy(SURFOBJ* target, SURFOBJ* source, RECTL* rect, POINTL* origin) {
    ++flood_copies;
    if (flood_write_fails && device_bitmap(target)) {
        return FALSE;
    }

    return copy_bits(target, source, rect, origin);
}

BOOL APIENTRY flood_fill(SURFOBJ* target, BRUSHOBJ* brush, RECTL* rect, POINTL* origin, ULONG color, UINT type) {
    return flood::fill_solid(*target, *brush, *rect, *origin, color, type);
}

const RX7900_FLOODFILL_ENGINE flood_engine = {flood_shadow, flood_discard, flood_copy, flood_fill};

// A maze of 0x00ffffff walls on 0x00000000, with a few coloured cells, so
// border and surface fills both have real boundaries and leaks. The pinned
// border fill re-pushes already filled pixels and so exhausts its stack after
// oscillating between two of them: the guard must store that partial result.
// The surface fill completes.
uint32_t maze(LONG x, LONG y) {
    if (x % 9 == 4 && y % 13 != 6) {
        return 0x00ffffffU;
    }

    if (y % 11 == 7 && x % 17 != 3) {
        return 0x00ffffffU;
    }

    return (x * 7 + y * 3) % 23 == 0 ? 0x00102030U : 0;
}

void opaque_flood_fill() {
    struct Case {
        RECTL rect;
        POINTL origin;
        ULONG color;
        UINT type;
        const char* name;
    } const cases[] = {
        {{0, 0, 60, 50}, {1, 1}, 0x00ffffffU, FLOODFILLBORDER, "a border fill of the whole bitmap equals the DIB fill"},
        {{0, 0, 60, 50}, {20, 20}, 0, FLOODFILLSURFACE, "a surface fill of the whole bitmap equals the DIB fill"},
        {{10, 8, 41, 33}, {12, 9}, 0x00ffffffU, FLOODFILLBORDER, "a border fill inside a region box equals the DIB fill"},
        {{5, 3, 30, 45}, {6, 4}, 0, FLOODFILLSURFACE, "a surface fill inside a region box equals the DIB fill"},
        {{0, 0, 60, 50}, {4, 0}, 0x00ffffffU, FLOODFILLBORDER, "a fill starting on the border colour fails like the DIB fill"},
        {{-20, -20, 500, 500}, {30, 30}, 0, FLOODFILLSURFACE, "fill bounds beyond the bitmap are clipped"},
    };
    unsigned partial_fills = 0;
    for (const Case& item : cases) {
        reset();
        const HBITMAP handle = create(pdevs[0], 60, 50);
        FakeSurface& surface = *from_handle(handle);
        DeviceStore& device = store(handle);
        for (LONG y = 0; y < 50; ++y) {
            for (LONG x = 0; x < 60; ++x) {
                device.pixels[y * 60 + x] = maze(x, y);
            }
        }

        FakeSurface& reference = engine_copy_of(device);
        BRUSHOBJ brush{};
        brush.iSolidColor = 0x00abcdefU;
        RECTL clipped{item.rect.left > 0 ? item.rect.left : 0, item.rect.top > 0 ? item.rect.top : 0,
                      item.rect.right < 60 ? item.rect.right : 60, item.rect.bottom < 50 ? item.rect.bottom : 50};
        const BOOL expected = flood::fill_solid(reference.so, brush, clipped, item.origin, item.color, item.type);
        // DC_vPrepareDCsForBlit holds the device lock around the fill.
        lock_device(hdev_of(pdevs[0]));
        const BOOL done = Rx7900FloodFillOpaque(&flood_engine, &surface.so, surface.flags, &brush, &item.rect,
                                                &item.origin, item.color, item.type);
        unlock_device(hdev_of(pdevs[0]));
        unsigned changed = 0;
        for (LONG y = 0; y < 50; ++y) {
            for (LONG x = 0; x < 60; ++x) {
                changed += device.pixels[y * 60 + x] != maze(x, y) ? 1U : 0U;
            }
        }

        check(done == expected && same_pixels(device, reference.so) && counters.unlocked_driver_calls == 0 &&
                  balanced() && (item.origin.x == 4 ? changed == 0 : changed > 0),
              item.name);
        partial_fills += !expected && changed != 0 ? 1U : 0U;
        release_engine(reference);
        delete_object(handle);
    }

    check(partial_fills != 0, "a DIB fill stopped part way (stack exhausted) is stored exactly as the engine left it");

    // Declines: nothing read or written, FALSE instead of a NULL pvScan0 crash.
    reset();
    const HBITMAP handle = create(pdevs[0], 60, 50);
    FakeSurface& surface = *from_handle(handle);
    DeviceStore& device = store(handle);
    for (LONG y = 0; y < 50; ++y) {
        for (LONG x = 0; x < 60; ++x) {
            device.pixels[y * 60 + x] = maze(x, y);
        }
    }

    FakeSurface& unchanged = engine_copy_of(device);
    BRUSHOBJ brush{};
    brush.iSolidColor = 0x00abcdefU;
    const RECTL whole{0, 0, 60, 50};
    const POINTL inside{1, 1};
    lock_device(hdev_of(pdevs[0]));
    flood_copies = 0;
    check(!Rx7900FloodFillOpaque(&flood_engine, &surface.so, 0, &brush, &whole, &inside, 0x00ffffffU,
                                 FLOODFILLBORDER) && flood_copies == 0,
          "a target reachable neither by DrvCopyBits nor DrvBitBlt fails without a driver call");
    check(!Rx7900FloodFillOpaque(&flood_engine, &surface.so, HOOK_BITBLT | HOOK_COPYBITS, &brush, &whole,
                                 nullptr, 0x00ffffffU, FLOODFILLBORDER) && flood_copies == 0,
          "a missing origin fails cleanly");
    const POINTL outside{70, 10};
    check(!Rx7900FloodFillOpaque(&flood_engine, &surface.so, surface.flags, &brush, &whole, &outside, 0x00ffffffU,
                                 FLOODFILLBORDER) && flood_copies == 0,
          "an origin outside the fill bounds fails cleanly (no out-of-bitmap read)");
    const RECTL empty{30, 30, 30, 40};
    check(!Rx7900FloodFillOpaque(&flood_engine, &surface.so, surface.flags, &brush, &empty, &inside, 0x00ffffffU,
                                 FLOODFILLBORDER) && flood_copies == 0,
          "empty fill bounds fail cleanly");
    FakeSurface& with_bits = *allocate_surface();
    make_engine_bitmap(with_bits, 8, 8, BMF_32BPP, true);
    with_bits.refs = 1;
    check(!Rx7900FloodFillOpaque(&flood_engine, &with_bits.so, radeon_hooks, &brush, &whole, &inside, 0,
                                 FLOODFILLSURFACE) && flood_copies == 0,
          "a target with bits is left to the original DIB fill");
    release_engine(with_bits);
    shadow_failures_left = 1;
    check(!Rx7900FloodFillOpaque(&flood_engine, &surface.so, surface.flags, &brush, &whole, &inside, 0x00ffffffU,
                                 FLOODFILLBORDER) && flood_copies == 0,
          "a failed shadow allocation fails cleanly");
    shadow_wrong_layout = true;
    check(!Rx7900FloodFillOpaque(&flood_engine, &surface.so, surface.flags, &brush, &whole, &inside, 0x00ffffffU,
                                 FLOODFILLBORDER) && flood_copies == 0,
          "a shadow of another layout fails cleanly");
    shadow_wrong_layout = false;
    shadow_wrong_format = true;
    check(!Rx7900FloodFillOpaque(&flood_engine, &surface.so, surface.flags, &brush, &whole, &inside, 0x00ffffffU,
                                 FLOODFILLBORDER) && flood_copies == 0,
          "a shadow of another format fails cleanly");
    shadow_wrong_format = false;
    copy_fails = true;
    check(!Rx7900FloodFillOpaque(&flood_engine, &surface.so, surface.flags, &brush, &whole, &inside, 0x00ffffffU,
                                 FLOODFILLBORDER) && flood_copies == 1,
          "a failed readback fails without a write");
    copy_fails = false;
    flood_write_fails = true;
    const POINTL open_cell{20, 20};
    check(!Rx7900FloodFillOpaque(&flood_engine, &surface.so, surface.flags, &brush, &whole, &open_cell, 0,
                                 FLOODFILLSURFACE),
          "a failed write-back of a completed fill reports failure");
    flood_write_fails = false;
    check(same_pixels(device, unchanged.so), "no decline changed a pixel");
    unlock_device(hdev_of(pdevs[0]));
    check(balanced(), "declines balance the device and registry locks");
    bool shadows_freed = true;
    for (FakeSurface& candidate : surfaces) {
        shadows_freed = shadows_freed && !(candidate.used && candidate.own_bits && candidate.refs > 0 &&
                                           &candidate != &unchanged);
    }

    check(shadows_freed, "every shadow is deleted on success and failure");
    release_engine(unchanged);
    delete_object(handle);
}

void rows_math() {
    RECTL rows{};
    RECTL partial{};
    const SIZEL size{40, 30};
    check(Rx7900DevBitmapRows(0, size, BMF_32BPP, &rows, &partial) && rows.bottom == 0 && partial.bottom == 0,
          "zero bytes touch no row");
    check(Rx7900DevBitmapRows(1, size, BMF_32BPP, &rows, &partial) && rows.bottom == 1 && partial.top == 0 &&
              partial.bottom == 1,
          "one byte touches a partial first row");
    check(Rx7900DevBitmapRows(160, size, BMF_32BPP, &rows, &partial) && rows.bottom == 1 && partial.bottom == 0,
          "exactly one row has no partial row");
    check(Rx7900DevBitmapRows(161, size, BMF_32BPP, &rows, &partial) && rows.bottom == 2 && partial.top == 1 &&
              partial.bottom == 2 && partial.right == 40,
          "one row plus one byte has the second row partial");
    check(Rx7900DevBitmapRows(1000000, size, BMF_32BPP, &rows, &partial) && rows.bottom == 30 && partial.bottom == 0,
          "more bytes than the bitmap touch every row once");
    // WIDTH_BYTES_ALIGN16: 1 bpp, 20 pixels = 4 bytes; 16 bpp, 7 pixels = 14 bytes.
    check(Rx7900DevBitmapRows(9, SIZEL{20, 5}, BMF_1BPP, &rows, &partial) && rows.bottom == 3 && partial.top == 2,
          "16-bit row alignment of the pinned GetBitmapBits is used (1 bpp)");
    check(Rx7900DevBitmapRows(28, SIZEL{7, 5}, BMF_16BPP, &rows, &partial) && rows.bottom == 2 && partial.bottom == 0,
          "16-bit row alignment of the pinned GetBitmapBits is used (16 bpp)");
    check(!Rx7900DevBitmapRows(8, SIZEL{0, 5}, BMF_32BPP, &rows, &partial) &&
              !Rx7900DevBitmapRows(8, size, 77, &rows, &partial) && rows.bottom == 0,
          "invalid sizes and formats are refused");
}

void engine_table_validation() {
    reset();
    RX7900_DEVBITMAP_ENGINE broken = table;
    broken.copy_bits = nullptr;
    check(Rx7900DevBitmapCreate(&broken, hdev_of(pdevs[0]), SIZEL{64, 64}, BMF_32BPP, nullptr) == nullptr &&
              counters.creates == 0 && !Rx7900DevBitmapConvert(&broken, &pdevs[0].primary->so),
          "an incomplete engine table disables the bridge");
    RX7900_DEVBITMAP_ENGINE overlapping = table;
    overlapping.marker = HOOK_BITBLT;
    FakeSurface& hooked = *allocate_surface();
    hooked.so.iType = STYPE_DEVBITMAP;
    hooked.flags = radeon_hooks;
    check(!Rx7900DevBitmapIsBridged(&overlapping, &hooked.so),
          "a marker overlapping the hook flags never names a hooked surface bridged");
    hooked.used = false;
    check(Rx7900DevBitmapCreate(&overlapping, hdev_of(pdevs[0]), SIZEL{64, 64}, BMF_32BPP, nullptr) == nullptr,
          "a marker overlapping the hook flags disables the bridge");
    check(Rx7900DevBitmapCreate(nullptr, hdev_of(pdevs[0]), SIZEL{64, 64}, BMF_32BPP, nullptr) == nullptr &&
              Rx7900DevBitmapCreate(engine, nullptr, SIZEL{64, 64}, BMF_32BPP, nullptr) == nullptr,
          "a missing engine or HDEV declines");
}

} // namespace

int main() {
    identity_contract();
    identity_gating();
    device_gating();
    result_validation();
    create_delete_pairing();
    registry_capacity();
    readers_and_writers();
    lock_contract();
    conversion();
    mode_change();
    opaque_flood_fill();
    rows_math();
    engine_table_validation();
    check(counters.registry_outside_device == 0, "the registry lock is only taken inside a device lock");
    printf("radeon_devbitmap_bridge: %u checks, %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

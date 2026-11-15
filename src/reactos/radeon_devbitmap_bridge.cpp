/*
Module: rx7900.reactos.radeon_devbitmap_bridge
Purpose: Identity-gated driver device bitmaps for win32k with exact pixel access,
exactly-once deletion and in-place conversion to engine bitmaps.
Created: 2026-10-24
Architecture: Policy only. Engine objects are reached through the glue's
RX7900_DEVBITMAP_ENGINE table and the driver through its DDI functions; no Eng*
import, allocation (other than through the table) or retained caller pointer.
Reuses the read-only radeon_line_identity.hpp escape and request-mask contract.
Locking: a bridged bitmap's state (marker flag, registry record, SURFOBJ layout)
changes only under its PDEV's device lock, which every bridged DDI call also
holds (DC_vPrepareDCsForBlit, the glue's direct readers/writers, creation,
cleanup and conversion). The registry lock nests inside a device lock.
*/
#include <stdarg.h>
#include <windef.h>
#include <wingdi.h>
#include <winddi.h>
#include <stdint.h>
#include <string.h>

#include "radeon_devbitmap_bridge.hpp"
#include "radeon_line_identity.hpp"

namespace {

namespace line = rx7900::reactos::radeon_line;

// Every bridged device bitmap of one PDEV. surface is the SURFOBJ inside the
// private SURFACE, valid until SURFACE_vCleanup returns (which untracks it
// first under the same device lock). destroy is the creating driver's
// DrvDeleteDeviceBitmap, kept so a later PDEV function swap cannot redirect it.
struct Record final {
    SURFOBJ* surface;
    HDEV hdev;
    PFN_DrvDeleteDeviceBitmap destroy;
};

// Zero-initialized static storage: no constructor runs in the kernel image.
Record registry[RX7900_DEVBITMAP_CAPACITY];

// Hooks an engine operation needs on an opaque target or source: EngBitBlt
// reads target and source bits directly (no IntEngEnter), and IntEngEnter
// reaches an opaque surface only through DrvCopyBits.
constexpr FLONG required_hooks = HOOK_BITBLT | HOOK_COPYBITS;

bool usable(const RX7900_DEVBITMAP_ENGINE* engine) noexcept {
    return engine != nullptr && engine->marker != 0 && (engine->marker & engine->hook_flags) == 0 &&
           engine->api_bitmap != 0 && engine->pool_bits != 0 && engine->lock_registry != nullptr &&
           engine->unlock_registry != nullptr && engine->lock_device != nullptr &&
           engine->unlock_device != nullptr && engine->reference_device != nullptr &&
           engine->release_device != nullptr && engine->describe_device != nullptr &&
           engine->lock_bitmap != nullptr && engine->unlock_bitmap != nullptr &&
           engine->delete_bitmap != nullptr && engine->read_flags != nullptr &&
           engine->update_flags != nullptr && engine->palette != nullptr && engine->adopt_bitmap != nullptr &&
           engine->create_shadow != nullptr && engine->copy_bits != nullptr &&
           engine->allocate_bits != nullptr && engine->free_bits != nullptr && engine->next_unique != nullptr;
}

bool marked(const RX7900_DEVBITMAP_ENGINE& engine, SURFOBJ* surface) noexcept {
    return (engine.read_flags(surface) & engine.marker) != 0;
}

uint32_t bits_per_pixel(ULONG format) noexcept {
    switch (format) {
    case BMF_1BPP: return 1;
    case BMF_4BPP: return 4;
    case BMF_8BPP: return 8;
    case BMF_16BPP: return 16;
    case BMF_24BPP: return 24;
    case BMF_32BPP: return 32;
    default: return 0;
    }
}

// Row bytes WIDTH_BYTES_ALIGN32/ALIGN16 of the pinned engine for width pixels.
bool row_bytes(LONG width, ULONG format, uint32_t alignment_bits, uint64_t& bytes) noexcept {
    const uint32_t bpp = bits_per_pixel(format);
    if (bpp == 0 || width <= 0) {
        return false;
    }

    const uint64_t bits = static_cast<uint64_t>(width) * bpp;
    bytes = ((bits + (alignment_bits - 1U)) & ~static_cast<uint64_t>(alignment_bits - 1U)) / 8U;
    return true;
}

// The pinned DDB layout of SURFACE_AllocSurface(STYPE_BITMAP, cx, cy, format, 0,
// 0, 0, NULL): 32-bit aligned rows, bottom-up, stride * height bytes.
struct Layout final {
    ULONG stride = 0;
    ULONG bytes = 0;
};

bool engine_layout(SIZEL size, ULONG format, Layout& layout) noexcept {
    uint64_t stride = 0;
    if (size.cy <= 0 || !row_bytes(size.cx, format, 32, stride)) {
        return false;
    }

    const uint64_t bytes = stride * static_cast<uint64_t>(size.cy);
    // lDelta is a LONG and pvScan0 is pvBits + bytes - stride.
    if (stride == 0 || stride > static_cast<uint64_t>(INT32_MAX) || bytes > UINT32_MAX) {
        return false;
    }

    layout.stride = static_cast<ULONG>(stride);
    layout.bytes = static_cast<ULONG>(bytes);
    return true;
}

// rect clipped to the bitmap; false when nothing remains.
bool clip(const RECTL& rect, SIZEL size, RECTL& output) noexcept {
    output.left = rect.left > 0 ? rect.left : 0;
    output.top = rect.top > 0 ? rect.top : 0;
    output.right = rect.right < size.cx ? rect.right : size.cx;
    output.bottom = rect.bottom < size.cy ? rect.bottom : size.cy;
    return output.left < output.right && output.top < output.bottom;
}

// ---------------------------------------------------------------------------
// Registry (registry lock inside the owner's device lock).
// ---------------------------------------------------------------------------
bool track(const RX7900_DEVBITMAP_ENGINE& engine, SURFOBJ* surface, HDEV hdev,
           PFN_DrvDeleteDeviceBitmap destroy) noexcept {
    if (!engine.lock_registry()) {
        return false;
    }

    Record* free_record = nullptr;
    bool present = false;
    for (Record& record : registry) {
        present = present || record.surface == surface;
        if (free_record == nullptr && record.surface == nullptr) {
            free_record = &record;
        }
    }

    const bool tracked = !present && free_record != nullptr;
    if (tracked) {
        *free_record = {surface, hdev, destroy};
    }

    engine.unlock_registry();
    return tracked;
}

bool untrack(const RX7900_DEVBITMAP_ENGINE& engine, SURFOBJ* surface, Record& output) noexcept {
    output = {};
    if (!engine.lock_registry()) {
        return false;
    }

    bool found = false;
    for (Record& record : registry) {
        if (record.surface == surface) {
            output = record;
            record = {};
            found = true;
            break;
        }
    }

    engine.unlock_registry();
    return found;
}

SURFOBJ* first_of(const RX7900_DEVBITMAP_ENGINE& engine, HDEV hdev) noexcept {
    if (!engine.lock_registry()) {
        return nullptr;
    }

    SURFOBJ* surface = nullptr;
    for (const Record& record : registry) {
        if (record.surface != nullptr && record.hdev == hdev) {
            surface = record.surface;
            break;
        }
    }

    engine.unlock_registry();
    return surface;
}

// ---------------------------------------------------------------------------
// Creation.
// ---------------------------------------------------------------------------

// The PDEV may offer device bitmaps: a display whose driver keeps
// DrvCreateDeviceBitmap at acceleration level 0 (PDEVOBJ_vFilterDriverHooks
// removes it from level 1; the level is checked too because a dynamic mode
// change installs functions filtered at level 0), enabled, with its surface.
bool device_admits(const RX7900_DEVBITMAP_DEVICE& device, ULONG format, PVOID palette) noexcept {
    return device.display && !device.disabled && device.acceleration_level == 0 && device.dhpdev != nullptr &&
           device.primary != nullptr && device.escape != nullptr && device.create != nullptr &&
           device.destroy != nullptr && device.copy != nullptr && device.palette != nullptr &&
           format == device.primary->iBitmapFormat && bits_per_pixel(format) != 0 &&
           (palette == nullptr || palette == device.palette);
}

// One read-only identity query naming the device-bitmap capability.
bool identified(const RX7900_DEVBITMAP_DEVICE& device) noexcept {
    uint32_t requested = line::flag_device_bitmaps;
    line::Identity identity{};
    const ULONG bytes = device.escape(device.primary, line::identity_escape, sizeof(requested), &requested,
                                      sizeof(identity), &identity);
    return bytes == sizeof(identity) &&
           line::admits_requested(identity, line::flag_device_bitmaps, line::flag_device_bitmaps);
}

// The driver's HBITMAP is the opaque device bitmap GDI asked for: every engine
// path then reaches it through DrvBitBlt/DrvCopyBits of this PDEV.
bool accepts(const RX7900_DEVBITMAP_ENGINE& engine, SURFOBJ* surface, HDEV hdev,
             const RX7900_DEVBITMAP_DEVICE& device, SIZEL size, ULONG format) noexcept {
    const FLONG flags = engine.read_flags(surface);
    return surface->iType == STYPE_DEVBITMAP && surface->hdev == hdev && surface->dhpdev == device.dhpdev &&
           surface->dhsurf != nullptr && surface->pvBits == nullptr && surface->pvScan0 == nullptr &&
           surface->sizlBitmap.cx == size.cx && surface->sizlBitmap.cy == size.cy &&
           surface->iBitmapFormat == format && (flags & required_hooks) == required_hooks &&
           (flags & engine.marker) == 0 && engine.palette(surface) == device.palette;
}

HBITMAP create_locked(const RX7900_DEVBITMAP_ENGINE& engine, HDEV hdev, SIZEL size, ULONG format,
                      PVOID palette) noexcept {
    RX7900_DEVBITMAP_DEVICE device{};
    if (!engine.describe_device(hdev, &device) || !device_admits(device, format, palette) || !identified(device)) {
        return nullptr;
    }

    HBITMAP handle = device.create(device.dhpdev, size, format);
    if (handle == nullptr) {
        // The driver declined: GDI creates its engine bitmap, as on NT.
        return nullptr;
    }

    SURFOBJ* surface = engine.lock_bitmap(handle);
    if (surface == nullptr) {
        // Not a GDI bitmap at all; nothing the engine could retire or use.
        return nullptr;
    }

    if (!accepts(engine, surface, hdev, device, size, format) || !track(engine, surface, hdev, device.destroy)) {
        // Never handed out: the driver frees its state once, then the object
        // goes (unmarked, so SURFACE_vCleanup does not call the driver again).
        if (surface->iType == STYPE_DEVBITMAP && surface->dhsurf != nullptr) {
            device.destroy(surface->dhsurf);
        }

        engine.update_flags(surface, 0, engine.marker);
        engine.delete_bitmap(surface);
        return nullptr;
    }

    engine.update_flags(surface, engine.marker | engine.api_bitmap, 0);
    engine.reference_device(hdev);
    engine.adopt_bitmap(surface);
    engine.unlock_bitmap(surface);
    return handle;
}

// ---------------------------------------------------------------------------
// Retirement and conversion (device lock held).
// ---------------------------------------------------------------------------

// The creating driver's DrvDeleteDeviceBitmap of a marked surface. The record
// is authoritative: creation tracks before it marks, so a marker without a
// record (never expected) is cleared without calling any driver.
PFN_DrvDeleteDeviceBitmap retire_record(const RX7900_DEVBITMAP_ENGINE& engine, SURFOBJ* surface) noexcept {
    Record record{};
    return untrack(engine, surface, record) ? record.destroy : nullptr;
}

bool retire_locked(const RX7900_DEVBITMAP_ENGINE& engine, SURFOBJ* surface) noexcept {
    if (!marked(engine, surface)) {
        return false;
    }

    const PFN_DrvDeleteDeviceBitmap destroy = retire_record(engine, surface);
    const DHSURF handle = surface->dhsurf;
    engine.update_flags(surface, 0, engine.marker);
    if (destroy != nullptr && handle != nullptr) {
        destroy(handle);
    }

    return true;
}

// Rows of shadow (same size and format, any orientation) into bits laid out as
// layout (bottom-up). false when the shadow is not that bitmap.
bool copy_rows(const SURFOBJ& shadow, SIZEL size, ULONG format, const Layout& layout, PVOID bits) noexcept {
    const LONG delta = shadow.lDelta;
    const uint64_t magnitude = delta < 0 ? static_cast<uint64_t>(-static_cast<int64_t>(delta)) :
                                           static_cast<uint64_t>(delta);
    if (shadow.pvScan0 == nullptr || shadow.iBitmapFormat != format || shadow.sizlBitmap.cx != size.cx ||
        shadow.sizlBitmap.cy != size.cy || magnitude < layout.stride) {
        return false;
    }

    // Row y of the shadow starts at pvScan0 + y * lDelta (engine contract);
    // row y of the bottom-up layout at bits + (height - 1 - y) * stride. Both
    // stay inside their allocations for 0 <= y < height.
    const uint8_t* source = static_cast<const uint8_t*>(shadow.pvScan0);
    uint8_t* target = static_cast<uint8_t*>(bits);
    for (LONG y = 0; y < size.cy; ++y) {
        const uint8_t* row = source + static_cast<intptr_t>(y) * delta;
        memcpy(target + static_cast<size_t>(size.cy - 1 - y) * layout.stride, row, layout.stride);
    }

    return true;
}

// The SURFOBJ of an engine DDB of the pinned SURFACE_AllocSurface layout.
void morph(SURFOBJ& surface, PVOID bits, const Layout& layout, USHORT pool_bits, ULONG unique) noexcept {
    surface.dhsurf = nullptr;
    surface.dhpdev = nullptr;
    surface.pvBits = bits;
    surface.cjBits = layout.bytes;
    surface.pvScan0 = static_cast<uint8_t*>(bits) + (layout.bytes - layout.stride);
    surface.lDelta = -static_cast<LONG>(layout.stride);
    surface.iUniq = unique;
    surface.fjBitmap = pool_bits;
    surface.iType = STYPE_BITMAP;
}

// Converts a marked surface; released reports the PDEV reference to drop after
// the device lock.
bool convert_locked(const RX7900_DEVBITMAP_ENGINE& engine, SURFOBJ* surface, bool& released) noexcept {
    released = false;
    if (!marked(engine, surface)) {
        Record stale{};
        untrack(engine, surface, stale);
        return true;
    }

    const SIZEL size = surface->sizlBitmap;
    const ULONG format = surface->iBitmapFormat;
    Layout layout{};
    if (!engine_layout(size, format, layout)) {
        return false;
    }

    SURFOBJ* shadow = engine.create_shadow(surface);
    if (shadow == nullptr) {
        return false;
    }

    RECTL whole{0, 0, size.cx, size.cy};
    POINTL origin{0, 0};
    bool copied = engine.copy_bits(shadow, surface, &whole, &origin) != FALSE;
    PVOID bits = copied ? engine.allocate_bits(layout.bytes) : nullptr;
    copied = bits != nullptr && copy_rows(*shadow, size, format, layout, bits);
    engine.delete_bitmap(shadow);
    if (!copied) {
        if (bits != nullptr) {
            engine.free_bits(bits);
        }

        return false;
    }

    const PFN_DrvDeleteDeviceBitmap destroy = retire_record(engine, surface);
    const DHSURF handle = surface->dhsurf;
    morph(*surface, bits, layout, engine.pool_bits, engine.next_unique());
    // Publishes the engine bitmap: a reader seeing no marker sees the layout.
    engine.update_flags(surface, 0, engine.hook_flags | engine.marker);
    if (destroy != nullptr && handle != nullptr) {
        destroy(handle);
    }

    released = true;
    return true;
}

} // namespace

extern "C" BOOL APIENTRY Rx7900DevBitmapIsBridged(const RX7900_DEVBITMAP_ENGINE* engine,
                                                  SURFOBJ* surface) noexcept {
    return usable(engine) && surface != nullptr && marked(*engine, surface) ? TRUE : FALSE;
}

extern "C" HBITMAP APIENTRY Rx7900DevBitmapCreate(const RX7900_DEVBITMAP_ENGINE* engine, HDEV hdev, SIZEL size,
                                                  ULONG format, PVOID palette) noexcept {
    if (!usable(engine) || hdev == nullptr || size.cx <= 0 || size.cy <= 0) {
        return nullptr;
    }

    engine->lock_device(hdev);
    HBITMAP handle = create_locked(*engine, hdev, size, format, palette);
    engine->unlock_device(hdev);
    return handle;
}

extern "C" VOID APIENTRY Rx7900DevBitmapCleanup(const RX7900_DEVBITMAP_ENGINE* engine,
                                                SURFOBJ* surface) noexcept {
    if (!usable(engine) || surface == nullptr || !marked(*engine, surface)) {
        return;
    }

    // A marked surface holds its PDEV reference, so hdev stays valid here.
    HDEV hdev = surface->hdev;
    engine->lock_device(hdev);
    const bool released = retire_locked(*engine, surface);
    engine->unlock_device(hdev);
    if (released) {
        engine->release_device(hdev);
    }
}

extern "C" HDEV APIENTRY Rx7900DevBitmapLock(const RX7900_DEVBITMAP_ENGINE* engine,
                                             SURFOBJ* surface) noexcept {
    if (!usable(engine) || surface == nullptr || !marked(*engine, surface)) {
        return nullptr;
    }

    HDEV hdev = surface->hdev;
    engine->lock_device(hdev);
    if (marked(*engine, surface)) {
        return hdev;
    }

    // Converted meanwhile: an engine bitmap needs no device lock.
    engine->unlock_device(hdev);
    return nullptr;
}

extern "C" VOID APIENTRY Rx7900DevBitmapUnlock(const RX7900_DEVBITMAP_ENGINE* engine, HDEV hdev) noexcept {
    if (usable(engine) && hdev != nullptr) {
        engine->unlock_device(hdev);
    }
}

extern "C" SURFOBJ* APIENTRY Rx7900DevBitmapShadow(const RX7900_DEVBITMAP_ENGINE* engine, SURFOBJ* surface,
                                                   const RECTL* read) noexcept {
    if (!usable(engine) || surface == nullptr) {
        return nullptr;
    }

    SURFOBJ* shadow = engine->create_shadow(surface);
    RECTL rect{};
    if (shadow == nullptr || read == nullptr || !clip(*read, surface->sizlBitmap, rect)) {
        return shadow;
    }

    // The engine copy of a bitmap converted meanwhile is the same EngCopyBits.
    POINTL origin{rect.left, rect.top};
    HDEV locked = Rx7900DevBitmapLock(engine, surface);
    const bool copied = engine->copy_bits(shadow, surface, &rect, &origin) != FALSE;
    Rx7900DevBitmapUnlock(engine, locked);
    if (!copied) {
        engine->delete_bitmap(shadow);
        return nullptr;
    }

    return shadow;
}

extern "C" VOID APIENTRY Rx7900DevBitmapDiscard(const RX7900_DEVBITMAP_ENGINE* engine, SURFOBJ* shadow) noexcept {
    if (usable(engine) && shadow != nullptr) {
        engine->delete_bitmap(shadow);
    }
}

extern "C" HBITMAP APIENTRY Rx7900DevBitmapKeep(const RX7900_DEVBITMAP_ENGINE* engine, SURFOBJ* shadow) noexcept {
    if (!usable(engine) || shadow == nullptr) {
        return nullptr;
    }

    HBITMAP handle = reinterpret_cast<HBITMAP>(shadow->hsurf);
    engine->unlock_bitmap(shadow);
    return handle;
}

extern "C" BOOL APIENTRY Rx7900DevBitmapStore(const RX7900_DEVBITMAP_ENGINE* engine, SURFOBJ* surface,
                                              SURFOBJ* shadow, const RECTL* rect) noexcept {
    if (!usable(engine) || surface == nullptr || shadow == nullptr || rect == nullptr ||
        shadow->sizlBitmap.cx != surface->sizlBitmap.cx || shadow->sizlBitmap.cy != surface->sizlBitmap.cy) {
        return FALSE;
    }

    RECTL area{};
    if (!clip(*rect, surface->sizlBitmap, area)) {
        return TRUE;
    }

    POINTL origin{area.left, area.top};
    HDEV locked = Rx7900DevBitmapLock(engine, surface);
    const BOOL stored = engine->copy_bits(surface, shadow, &area, &origin);
    Rx7900DevBitmapUnlock(engine, locked);
    return stored ? TRUE : FALSE;
}

extern "C" BOOL APIENTRY Rx7900DevBitmapConvert(const RX7900_DEVBITMAP_ENGINE* engine,
                                                SURFOBJ* surface) noexcept {
    if (!usable(engine) || surface == nullptr) {
        return FALSE;
    }

    if (!marked(*engine, surface)) {
        return TRUE;
    }

    HDEV hdev = surface->hdev;
    bool released = false;
    engine->lock_device(hdev);
    const bool converted = convert_locked(*engine, surface, released);
    engine->unlock_device(hdev);
    if (released) {
        engine->release_device(hdev);
    }

    return converted ? TRUE : FALSE;
}

extern "C" BOOL APIENTRY Rx7900DevBitmapConvertDevice(const RX7900_DEVBITMAP_ENGINE* engine,
                                                      HDEV hdev) noexcept {
    if (!usable(engine) || hdev == nullptr) {
        return FALSE;
    }

    // The device lock keeps every listed surface alive (cleanup needs it too);
    // PDEV references are dropped only after it is released.
    engine->lock_device(hdev);
    bool converted = true;
    uint32_t released = 0;
    for (uint32_t pass = 0; pass <= RX7900_DEVBITMAP_CAPACITY && converted; ++pass) {
        SURFOBJ* surface = first_of(*engine, hdev);
        if (surface == nullptr) {
            break;
        }

        bool dropped = false;
        converted = convert_locked(*engine, surface, dropped);
        released += dropped ? 1U : 0U;
    }

    engine->unlock_device(hdev);
    for (uint32_t index = 0; index < released; ++index) {
        engine->release_device(hdev);
    }

    return converted ? TRUE : FALSE;
}

extern "C" BOOL APIENTRY Rx7900DevBitmapRows(ULONG bytes, SIZEL size, ULONG format, RECTL* rows,
                                             RECTL* partial) noexcept {
    if (rows == nullptr || partial == nullptr) {
        return FALSE;
    }

    *rows = {};
    *partial = {};
    uint64_t row = 0;
    if (size.cy <= 0 || !row_bytes(size.cx, format, 16, row) || row == 0) {
        return FALSE;
    }

    // UnsafeSetBitmapBits/UnsafeGetBitmapBits move min(bytes, row * height)
    // bytes, whole rows from the top and then one partial row.
    const uint64_t total = row * static_cast<uint64_t>(size.cy);
    const uint64_t moved = bytes < total ? bytes : total;
    const LONG whole = static_cast<LONG>(moved / row);
    const bool remainder = moved % row != 0;
    const LONG touched = whole + (remainder ? 1 : 0);
    if (touched != 0) {
        *rows = {0, 0, size.cx, touched};
    }

    if (remainder) {
        *partial = {0, whole, size.cx, whole + 1};
    }

    return TRUE;
}

extern "C" BOOL APIENTRY Rx7900FloodFillOpaque(const RX7900_FLOODFILL_ENGINE* engine, SURFOBJ* target, FLONG hooks,
                                               BRUSHOBJ* brush, const RECTL* rect, const POINTL* origin, ULONG color,
                                               UINT fill_type) noexcept {
    if (engine == nullptr || engine->create_shadow == nullptr || engine->delete_shadow == nullptr ||
        engine->copy_bits == nullptr || engine->fill == nullptr || target == nullptr || brush == nullptr ||
        rect == nullptr || origin == nullptr || target->pvScan0 != nullptr ||
        bits_per_pixel(target->iBitmapFormat) == 0 || target->sizlBitmap.cx <= 0 || target->sizlBitmap.cy <= 0 ||
        // EngCopyBits reaches an opaque operand only through DrvCopyBits or
        // DrvBitBlt; without either it would run EngBitBlt on missing bits.
        (hooks & (HOOK_COPYBITS | HOOK_BITBLT)) == 0) {
        return FALSE;
    }

    // The DIB fill reads and writes only pixels inside its rectangle, starting
    // at origin, so the shadow needs exactly that area of the target.
    RECTL area{};
    if (!clip(*rect, target->sizlBitmap, area) || origin->x < area.left || origin->x >= area.right ||
        origin->y < area.top || origin->y >= area.bottom) {
        return FALSE;
    }

    SURFOBJ* shadow = engine->create_shadow(target);
    if (shadow == nullptr) {
        return FALSE;
    }

    if (shadow->pvScan0 == nullptr || shadow->iBitmapFormat != target->iBitmapFormat ||
        shadow->sizlBitmap.cx != target->sizlBitmap.cx || shadow->sizlBitmap.cy != target->sizlBitmap.cy) {
        engine->delete_shadow(shadow);
        return FALSE;
    }

    RECTL read = area;
    POINTL read_origin{area.left, area.top};
    if (!engine->copy_bits(shadow, target, &read, &read_origin)) {
        engine->delete_shadow(shadow);
        return FALSE;
    }

    RECTL fill_rect = area;
    POINTL fill_origin = *origin;
    BOOL filled = engine->fill(shadow, brush, &fill_rect, &fill_origin, color, fill_type) ? TRUE : FALSE;
    RECTL write = area;
    POINTL write_origin{area.left, area.top};
    if (!engine->copy_bits(target, shadow, &write, &write_origin)) {
        filled = FALSE;
    }

    engine->delete_shadow(shadow);
    return filled;
}

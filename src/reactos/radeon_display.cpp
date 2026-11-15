/*
Module: rx7900.reactos.radeon_display
Purpose: NT5 GDI display driver for the physical Radeon RX 7900 adopted pre-OS
scanout, offloading exactly expressible fills and blits on the primary and on
GPU-only offscreen device bitmaps to the rx7900video SDMA 2D IOCTLs and
rendering everything else with the Eng* engine.
Created: 2026-10-22
Architecture: Original C++ DDI (no framebuf sources). The primary is an opaque
device surface (EngCreateDeviceSurface) so the engine can never touch the
framebuffer behind the driver's back: unhooked engine paths reach it only
through DrvCopyBits (IntEngEnter/IntEngLeave), and every CPU access goes
through a GPU barrier first. Offscreen device bitmaps (DrvCreateDeviceBitmap)
are opaque too: GPU resident in a miniport VRAM surface, or demoted to a driver
DIB when the engine needs their pixels. Admission, clip batching order and
bitmap policy live in radeon_display.hpp; the wire format in radeon_2d_abi.hpp;
clip geometry reuses lightning_gdi_geometry.hpp and framebuffer_shadow_policy.hpp
read-only.
ReactOS engine contract inspected at the shared work/reactos tree (a97c8a2):
win32ss/gdi/eng/engmisc.c (IntEngEnter), bitblt.c (IntEngBitBlt source-driver
dispatch), paint.c (EngPaint -> IntEngBitBlt), stubs.c (EngFillPath is
unimplemented, DrvFillPath is never dispatched), stretchblt.c (a hooked
StretchBltROP loses mirroring: only calls the paired stretch guard tagged with
the orientation are drawn, the rest return FALSE to the engine), clip.c
(CLIPOBJ_cEnumStart sorts by CD_* direction; a nonzero cMaxRects also caps
CLIPOBJ_bEnum, so enumeration passes 0 and pages through ENUMRECTS chunks),
ntgdi/bitmaps.c (CreateCompatibleBitmap always creates engine bitmaps: this
engine never calls DrvCreateDeviceBitmap; NT-compatible engines do).
Compute raster (abi version 5): DrvBitBlt/DrvCopyBits beyond the SDMA plans
(general ROP4, realized 8x8 patterns from DrvRealizeBrush, translated 1bpp
sources, 8/16/24bpp sources through staged XLATEOBJ tables, 1bpp masks: the ReactOS mono glyph path IntEngMaskBlt ->
ROP4_MASKPAINT), DrvTransparentBlt, DrvAlphaBlend, DrvGradientFill, DrvLineTo
(paired-kernel-tagged unstyled lines only: EngLineTo ignores MIX, keeps
styles in private brush state and plots iSolidColor for pattern pens),
DrvStretchBltROP (guard-tagged calls: the exact EngStretchBltROP DIB calls,
mirrored, with ROP4, solid or realized tile brush (running PatternX under a
mask), 1bpp mask, negative source extents, translated 8/16/24bpp sources and
GPU snapshots of device windows that leave their surface or meet a
same-surface DIB call, as the engine's zero-filled IntEngEnter temporary) and
DrvStretchBlt (NT engines; ReactOS dispatches only StretchBltROP). The pinned engine
never calls DrvTextOut (glyphs use IntEngMaskBlt), DrvStrokePath (#if 0 in
IntEngLineTo) or DrvCreateDeviceBitmap. An opt-in paired win32k bridge may call
DrvStrokePath for one styled cosmetic segment and kernel-only DrvNotify for
8-bpp coverage glyphs; unsupported calls retain the original engine fallback.
The pointer DDI
(DrvSetPointerShape/DrvMovePointer) is a thin wrapper over radeon_cursor_client
on the primary only; the DRVFN table is sized from its entry list.
*/
#include <stdarg.h>
#include <windef.h>
#include <wingdi.h>
#include <winddi.h>
#include <winioctl.h>
#include <ntddvdeo.h>
#include <string.h>

#include "radeon_display.hpp"
#include "radeon_line_identity.hpp"
#include "radeon_opengl_info.hpp"
#include "radeon_present.hpp"
#include "radeon_text_bridge.hpp"
#include "lightning_gdi_geometry.hpp"
#include "lifetime.hpp"
#include "framebuffer_stretch_guard.hpp"
#include "radeon_cursor_client.hpp"

namespace {

namespace abi = rx7900::reactos::radeon2d::abi;
namespace display = rx7900::reactos::radeon_display;
namespace policy = rx7900::reactos::framebuffer;
using display::Rectangle;
using rx7900::ByteView;
using rx7900::sdma::CommandBuffer;

constexpr ULONG allocation_tag = 0x44325852U; // "RX2D"
// Most compute raster hooks render through Eng* on CPU views when unsupported.
// Styled StrokePath instead returns FALSE to its paired IntEngLineTo caller,
// which follows the original EngLineTo fallback after no GPU mutation.
constexpr FLONG raster_hooks = HOOK_TRANSPARENTBLT | HOOK_ALPHABLEND | HOOK_GRADIENTFILL | HOOK_LINETO |
                               HOOK_STROKEPATH | HOOK_STRETCHBLT | HOOK_STRETCHBLTROP;
constexpr FLONG primary_hooks = HOOK_BITBLT | HOOK_COPYBITS | HOOK_FILLPATH | HOOK_SYNCHRONIZE | raster_hooks;
// Device bitmaps hook the same calls; every other engine path reaches their
// pixels through DrvCopyBits (readback/upload of the touched area).
constexpr FLONG bitmap_hooks = HOOK_BITBLT | HOOK_COPYBITS | HOOK_FILLPATH | HOOK_SYNCHRONIZE | raster_hooks;
static_assert(display::line_mix_tag == RX7900_LINE_MIX_TAG, "Paired kernel line tag");
constexpr ULONG red_mask = 0x00ff0000U;
constexpr ULONG green_mask = 0x0000ff00U;
constexpr ULONG blue_mask = 0x000000ffU;
constexpr uint32_t required_capabilities = abi::capability_fill | abi::capability_copy |
    abi::capability_upload | abi::capability_readback | abi::capability_primary;

// unsupported: nothing was mutated by the GPU for this request (exact fallback
// allowed). surface_lost: likewise, and a named device bitmap lost its VRAM.
enum class Attempt { unsupported, surface_lost, performed, failed };

struct Pdev;

// One offscreen device bitmap, allocated by EngAllocMem and constructed in
// place once. Exactly one of surface (GPU resident) and dib (demoted) is set
// while it is alive; GDI owns handle and deletes it after DrvDeleteDeviceBitmap.
struct DeviceBitmap final {
    DeviceBitmap* next = nullptr;
    Pdev* owner = nullptr;
    HSURF handle = nullptr;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t surface = 0;
    uint64_t bytes = 0;
    HSURF dib_handle = nullptr;
    SURFOBJ* dib = nullptr;
    uint32_t gpu_uses = 0;
};

// Clip enumeration of one operation, in batches of at most abi::max_rects
// rectangles intersected with the operation bounds.
struct ClipCursor final {
    CLIPOBJ* clip = nullptr;
    Rectangle bounds{};
    bool complex = false;
    bool single_pending = false;
    bool more = false;
    bool failed = false;
    ULONG position = 0;
    // CLIPOBJ_bEnum writes the documented ENUMRECTS layout: count + RECTL[].
    struct Enumeration final {
        ULONG count = 0;
        RECTL rectangles[abi::max_rects]{};
    } enumeration{};
};

// One per PDEV, allocated by EngAllocMem and constructed in place once.
struct Pdev final {
    Pdev* next = nullptr;
    bool registered = false;
    HANDLE driver = nullptr;
    HDEV engine = nullptr;
    HPALETTE palette = nullptr;
    ULONG mode_index = 0;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t pitch = 0;
    // Primary device surface handed to GDI, and the engine DIB over the mapped
    // framebuffer that Eng* fallbacks draw into after a GPU barrier.
    HSURF primary = nullptr;
    HSURF bitmap_handle = nullptr;
    SURFOBJ* bitmap = nullptr;
    void* framebuffer = nullptr;
    // Miniport session. staging is the miniport's kernel mapping; it remains
    // valid until close_session and is never accessed after it.
    bool session_open = false;
    bool accelerated = false;
    bool poisoned = false;
    // Destructive-reset recovery (abi v6): the miniport's recovery generation
    // when this session last proved itself, revival queries since poisoning,
    // revival refused for good, a proven-quiescent disabled 2D (exact CPU
    // rendering allowed), whether GDI wants the primary, and lost handles.
    uint32_t recovery_generation = 0;
    uint32_t revival_queries = 0;
    bool revival_closed = false;
    bool gpu_quiescent = false;
    bool primary_wanted = false;
    display::LostSurfaces lost_surfaces{};
    abi::Header session{};
    uint32_t primary_handle = 0;
    uint8_t* staging = nullptr;
    uint32_t staging_bytes = 0;
    uint64_t last_fence = 0;
    uint64_t completed = 0;
    uint64_t half_fence[display::staging_halves]{};
    uint32_t next_half = 0;
    // Device bitmaps of this PDEV and their accounting.
    DeviceBitmap* bitmaps = nullptr;
    uint32_t bitmap_count = 0;
    uint32_t resident_count = 0;
    display::BitmapLimits bitmap_limits{};
    uint64_t resident_bytes = 0;
    // Compute raster codes the miniport advertised for this session.
    uint64_t raster_ops = 0;
    display::Statistics statistics{};
    // Device-owned workspaces: no kilobyte request on the kernel stack.
    ClipCursor clip{};
    abi::RectList batch{};
    abi::RectList moved{};   // A batch translated into the other surface.
    abi::RectList band{};    // One banded transfer rectangle.
    abi::FillCommand fill{};
    abi::CopyCommand copy{};
    abi::TransferCommand transfer{};
    abi::RopCommand rop{};
    abi::TextCommand text{};
    abi::AlphaCommand alpha{};
    abi::LineCommand line{};
    uint32_t style_runs[display::max_engine_style_runs]{};
    abi::GradientCommand gradient{};
    abi::StretchRopCommand stretch_rop{};
    // Guarded stretch planning workspace, the cached snapshot surface (an
    // offscreen surface counted in resident_count/resident_bytes) and the
    // operation serial that marks which staging half already holds this
    // operation's translation table.
    display::StretchRopSetup stretch_setup{};
    display::StretchSourcePlan stretch_plan{};
    uint32_t scratch_surface = 0;
    uint32_t scratch_width = 0;
    uint32_t scratch_height = 0;
    uint64_t scratch_bytes = 0;
    uint32_t stretch_serial = 0;
    uint32_t table_serial[display::staging_halves]{};
    uint8_t request[abi::max_request_bytes]{};
    uint8_t reply[abi::max_reply_bytes]{};
    // RadeonGL present: decoded request and its wire bytes (1072 each), kept off
    // the DDI stack and used only under registry_lock.
    rx7900::reactos::graphics_abi::PresentRequest present_request{};
    uint8_t present_bytes[rx7900::reactos::graphics_abi::present_request_bytes]{};
    // Enable-time descriptions, kept off the kernel stack.
    GDIINFO gdi_info{};
    DEVINFO dev_info{};
    // Hardware cursor client (radeon_cursor_client), driven by the pointer DDI.
    rx7900::reactos::radeon_cursor::PointerClient pointer{};
};
static_assert(alignof(Pdev) <= (sizeof(void*) == 4 ? 8U : 16U), "EngAllocMem pool alignment");
static_assert(alignof(DeviceBitmap) <= (sizeof(void*) == 4 ? 8U : 16U), "EngAllocMem pool alignment");

// Engine lifecycle serializes DrvEnableDriver/DrvDisableDriver. The registry
// identifies our primaries and device bitmaps among foreign device surfaces
// before any private pointer is interpreted, and serializes drawing against
// surface retirement. A PDEV stays registered from its first DrvEnableSurface
// to DrvDisablePDEV, so demoted bitmaps remain usable after DrvDisableSurface.
HSEMAPHORE registry_lock = nullptr;
Pdev* registry = nullptr;
// Engine-owned after DrvEnableDriver; zero-initialized, filled before publication.
// Capacity leaves room for the cursor hooks and later raster entries.
constexpr uint32_t driver_function_capacity = 27;
DRVFN driver_functions[driver_function_capacity]{};

class RegistryGuard final {
public:
    RegistryGuard() noexcept { EngAcquireSemaphore(registry_lock); }
    ~RegistryGuard() { EngReleaseSemaphore(registry_lock); }
    RegistryGuard(const RegistryGuard&) = delete;
    RegistryGuard& operator=(const RegistryGuard&) = delete;
};

// An operand of one DDI call, classified without dereferencing foreign
// private data. kind is computed later against the PDEV doing the work.
struct Operand final {
    SURFOBJ* original = nullptr;
    Pdev* owner = nullptr;          // Our PDEV (primary or device bitmap).
    DeviceBitmap* bitmap = nullptr; // Our device bitmap, resident or demoted.
    bool primary = false;
};

Pdev* registered_pdev(DHPDEV handle) noexcept {
    for (Pdev* device = registry; device != nullptr; device = device->next) {
        if (reinterpret_cast<DHPDEV>(device) == handle) {
            return device;
        }
    }

    return nullptr;
}

DeviceBitmap* owned_bitmap(Pdev& device, DHSURF handle) noexcept {
    for (DeviceBitmap* bitmap = device.bitmaps; bitmap != nullptr; bitmap = bitmap->next) {
        if (reinterpret_cast<DHSURF>(bitmap) == handle) {
            return bitmap;
        }
    }

    return nullptr;
}

// false: a device surface that is not ours (refused without interpretation).
bool classify(SURFOBJ* surface, Operand& output) noexcept {
    output = {};
    output.original = surface;
    if (surface == nullptr || surface->iType == STYPE_BITMAP) {
        return true;
    }

    for (Pdev* device = registry; device != nullptr; device = device->next) {
        if (device->primary != nullptr && device->primary == surface->hsurf) {
            output.owner = device;
            output.primary = true;
            return true;
        }
    }

    if (surface->iType == STYPE_DEVBITMAP) {
        Pdev* device = registered_pdev(surface->dhpdev);
        DeviceBitmap* bitmap = device != nullptr ? owned_bitmap(*device, surface->dhsurf) : nullptr;
        if (bitmap != nullptr) {
            output.owner = device;
            output.bitmap = bitmap;
            return true;
        }
    }

    return false;
}

bool trivial(XLATEOBJ* translation) noexcept {
    return translation == nullptr || (translation->flXlate & XO_TRIVIAL) != 0;
}

bool engine_bitmap32(SURFOBJ* surface) noexcept {
    return surface != nullptr && surface->iType == STYPE_BITMAP && surface->iBitmapFormat == BMF_32BPP &&
           surface->pvScan0 != nullptr && surface->sizlBitmap.cx > 0 && surface->sizlBitmap.cy > 0;
}

// as_resident: classify a demoted bitmap as if it were GPU resident (promotion
// policy); otherwise a demoted bitmap is its 32bpp DIB.
display::SurfaceKind kind_of(const Operand& operand, const Pdev& device, bool as_resident) noexcept {
    if (operand.original == nullptr) {
        return display::SurfaceKind::none;
    }

    if (operand.owner != nullptr && operand.owner != &device) {
        return display::SurfaceKind::other;
    }

    if (operand.primary) {
        return display::SurfaceKind::primary;
    }

    if (operand.bitmap != nullptr) {
        if (operand.bitmap->surface != 0 || (as_resident && operand.bitmap->dib != nullptr)) {
            return display::SurfaceKind::offscreen;
        }

        // A bitmap without either backing (lost with its session and no DIB
        // memory) is never touched again.
        return operand.bitmap->dib != nullptr ? display::SurfaceKind::bitmap32 : display::SurfaceKind::other;
    }

    return engine_bitmap32(operand.original) ? display::SurfaceKind::bitmap32 : display::SurfaceKind::other;
}

display::SurfaceKind kind_with(const Operand& operand, const Pdev& device, XLATEOBJ* translation,
                               bool as_resident) noexcept {
    const display::SurfaceKind kind = kind_of(operand, device, as_resident);
    return kind != display::SurfaceKind::none && !trivial(translation) ? display::SurfaceKind::other : kind;
}

Rectangle value(const RECTL& rectangle) noexcept {
    return {rectangle.left, rectangle.top, rectangle.right, rectangle.bottom};
}

abi::Rect wire(Rectangle rectangle) noexcept {
    return {rectangle.left, rectangle.top, rectangle.right, rectangle.bottom};
}

ULONG enumeration_direction(display::CopyDirection direction) noexcept {
    switch (direction) {
    case display::CopyDirection::left_down: return CD_LEFTDOWN;
    case display::CopyDirection::right_up: return CD_RIGHTUP;
    case display::CopyDirection::left_up: return CD_LEFTUP;
    case display::CopyDirection::right_down:
    default: return CD_RIGHTDOWN;
    }
}

// GDI owns the CLIPOBJ for this synchronous call. false (unsupported) for
// mirrored destinations or unknown complexity; an empty bounds is valid and
// yields no batch.
bool clip_begin(const RECTL& requested, CLIPOBJ* clip, uint32_t width, uint32_t height, ULONG direction,
                ClipCursor& cursor) noexcept {
    cursor.clip = clip;
    cursor.bounds = {};
    cursor.complex = false;
    cursor.single_pending = false;
    cursor.more = false;
    cursor.failed = false;
    cursor.position = 0;
    cursor.enumeration.count = 0;
    if (!lightning::reactos::gdi::draw_bounds(value(requested), static_cast<int32_t>(width),
                                              static_cast<int32_t>(height), cursor.bounds)) {
        return false;
    }

    const BYTE complexity = clip == nullptr ? DC_TRIVIAL : clip->iDComplexity;
    if (complexity != DC_TRIVIAL && complexity != DC_RECT && complexity != DC_COMPLEX) {
        return false;
    }

    if (complexity != DC_TRIVIAL) {
        cursor.bounds = policy::intersect(cursor.bounds, value(clip->rclBounds));
    }

    if (policy::empty(cursor.bounds)) {
        cursor.bounds = {};
        return true;
    }

    if (complexity != DC_COMPLEX) {
        cursor.single_pending = true;
        return true;
    }

    // cMaxRects 0: enumerate everything (a nonzero limit truncates the ReactOS
    // enumeration). The return value is only a count; bEnum is authoritative.
    CLIPOBJ_cEnumStart(clip, FALSE, CT_RECTANGLES, direction, 0);
    cursor.complex = true;
    cursor.more = true;
    return true;
}

// Next batch (up to abi::max_rects nonempty clipped rectangles). false when
// the enumeration is exhausted or failed (cursor.failed).
bool clip_next(ClipCursor& cursor, abi::RectList& batch) noexcept {
    batch.count = 0;
    if (!cursor.complex) {
        if (cursor.single_pending) {
            batch.rects[0] = wire(cursor.bounds);
            batch.count = 1;
            cursor.single_pending = false;
        }

        return batch.count != 0;
    }

    ClipCursor::Enumeration& enumeration = cursor.enumeration;
    while (batch.count < abi::max_rects && !cursor.failed) {
        if (cursor.position >= enumeration.count) {
            if (!cursor.more) {
                break;
            }

            // The engine leaves count untouched when nothing remains. Our
            // Enumeration has the ENUMRECTS layout and capacity, and the byte
            // count passed bounds every engine write.
            enumeration.count = 0;
            cursor.position = 0;
            cursor.more = CLIPOBJ_bEnum(cursor.clip, sizeof(enumeration),
                                        reinterpret_cast<ULONG*>(&enumeration)) != FALSE;
            if (enumeration.count > abi::max_rects || (cursor.more && enumeration.count == 0)) {
                cursor.failed = true;
                batch.count = 0;
            }

            continue;
        }

        display::append_clipped(cursor.bounds, value(enumeration.rectangles[cursor.position]), batch);
        ++cursor.position;
    }

    return batch.count != 0 && !cursor.failed;
}

// Row pointer inside a GDI bitmap. Caller proved 0 <= y < cy and the byte
// span [x * 4, x * 4 + bytes) lies within the row (x + width <= cx, 32bpp).
// lDelta may be negative for bottom-up DIBs; GDI owns pvScan0's allocation.
uint8_t* bitmap_pixel(SURFOBJ* surface, int32_t x, int32_t y) noexcept {
    return static_cast<uint8_t*>(surface->pvScan0) +
           static_cast<intptr_t>(y) * surface->lDelta +
           static_cast<intptr_t>(x) * static_cast<intptr_t>(abi::bytes_per_pixel);
}

// One same-surface SRCCOPY rectangle on a 32bpp DIB with memmove semantics:
// rows run away from the source. Caller proved rect and rect + (dx, dy) inside.
void cpu_copy_rect(SURFOBJ* surface, const abi::Rect& rect, int32_t dx, int32_t dy) noexcept {
    const size_t row_bytes = static_cast<size_t>(rect.right - rect.left) * abi::bytes_per_pixel;
    const int32_t rows = rect.bottom - rect.top;
    for (int32_t step = 0; step < rows; ++step) {
        const int32_t y = dy > 0 ? rect.top + step : rect.bottom - 1 - step;
        memmove(bitmap_pixel(surface, rect.left, y), bitmap_pixel(surface, rect.left + dx, y + dy), row_bytes);
    }
}

bool request_receipt(Pdev& device, uint32_t code, uint32_t bytes, abi::Receipt& receipt) noexcept {
    DWORD returned = 0;
    if (EngDeviceIoControl(device.driver, code, device.request, bytes, device.reply,
                           abi::receipt_bytes, &returned) != 0 || returned != abi::receipt_bytes) {
        return false;
    }

    if (!abi::decode_receipt(ByteView(device.reply, abi::receipt_bytes), receipt)) {
        // A malformed reply cannot prove whether work ran: treat as lost.
        receipt = {};
        receipt.outcome = abi::outcome_lost;
    }

    if (receipt.completed_fence > device.completed) {
        device.completed = receipt.completed_fence;
    }

    // Exactly once per lost handle: remembered for the local fallback path.
    if (receipt.outcome == abi::outcome_surface_lost) {
        display::remember_lost(device.lost_surfaces, receipt.lost_surface);
    }

    return true;
}

void poison(Pdev& device) noexcept {
    device.poisoned = true;
    device.accelerated = false;
    display::count(device.statistics.lost);
}

bool session_usable(const Pdev& device) noexcept {
    return device.session_open && !device.poisoned;
}

bool assert_primary(Pdev& device, bool enable) noexcept;

// A poisoned session after the miniport's destructive-reset recovery: the reset
// proof retired every fence of the session, so a successful sync makes it
// usable again (its lost surfaces answer surface_lost once each). Bounded by
// display::max_revival_queries query IOCTLs per poisoning. true: usable now.
bool revive_session(Pdev& device) noexcept {
    if (!device.poisoned) {
        return device.session_open;
    }

    if (!device.session_open || device.revival_closed || device.gpu_quiescent) {
        return false;
    }

    DWORD returned = 0;
    abi::Capabilities capabilities{};
    if (EngDeviceIoControl(device.driver, abi::ioctl_query, nullptr, 0, device.reply, abi::capabilities_bytes,
                           &returned) != 0 ||
        returned != abi::capabilities_bytes ||
        !abi::decode_capabilities(ByteView(device.reply, abi::capabilities_bytes), capabilities)) {
        ++device.revival_queries;
        device.revival_closed = device.revival_queries >= display::max_revival_queries;
        return false;
    }

    switch (display::revival_decision(capabilities, device.recovery_generation, device.revival_queries)) {
    case display::Revival::wait:
        if (capabilities.recovery_state != abi::recovery_pending) {
            ++device.revival_queries;
        }

        return false;
    case display::Revival::quiescent:
        device.gpu_quiescent = true;
        return false;
    case display::Revival::give_up:
        device.revival_closed = true;
        return false;
    case display::Revival::revive:
        break;
    }

    abi::SyncCommand command{};
    command.session = device.session;
    abi::Receipt receipt{};
    if (!abi::encode_sync(CommandBuffer(device.request, abi::sync_request_bytes), command) ||
        !request_receipt(device, abi::ioctl_sync, abi::sync_request_bytes, receipt) ||
        receipt.outcome != abi::outcome_completed || receipt.completed_fence < device.last_fence) {
        // The miniport did not keep this session: it stays unaccelerated.
        device.revival_closed = true;
        return false;
    }

    device.poisoned = false;
    device.recovery_generation = capabilities.recovery_generation;
    device.revival_queries = 0;
    device.accelerated = device.primary_wanted && assert_primary(device, true);
    device.statistics.accelerated = device.accelerated ? 1U : 0U;
    display::count(device.statistics.syncs);
    return true;
}

// Submit the encoded request in device.request. Untouched GPU (IOCTL error,
// declined or surface_lost receipt) permits an exact engine fallback; lost
// forbids it.
Attempt submit(Pdev& device, uint32_t code, uint32_t bytes, uint64_t& fence) noexcept {
    abi::Receipt receipt{};
    fence = 0;
    if (!request_receipt(device, code, bytes, receipt)) {
        display::count(device.statistics.declined);
        return Attempt::unsupported;
    }

    if (receipt.outcome == abi::outcome_completed) {
        fence = receipt.fence;
        if (fence > device.last_fence) {
            device.last_fence = fence;
        }

        return Attempt::performed;
    }

    if (receipt.outcome == abi::outcome_declined) {
        display::count(device.statistics.declined);
        return Attempt::unsupported;
    }

    if (receipt.outcome == abi::outcome_surface_lost) {
        display::count(device.statistics.declined);
        return Attempt::surface_lost;
    }

    poison(device);
    return Attempt::failed;
}

bool untouched(Attempt attempt) noexcept {
    return attempt == Attempt::unsupported || attempt == Attempt::surface_lost;
}

// Wait until fence (0 = all of this session's work) has retired.
bool sync_fence(Pdev& device, uint64_t fence) noexcept {
    if (device.poisoned && !revive_session(device)) {
        // After a proven reset with 2D off for good, nothing on the GPU can
        // still access memory: CPU access is exact again.
        if (device.gpu_quiescent) {
            return true;
        }

        display::count(device.statistics.sync_failures);
        return false;
    }

    const uint64_t target = fence == 0 ? device.last_fence : fence;
    if (!device.session_open || target <= device.completed) {
        return true;
    }

    abi::SyncCommand command{};
    command.session = device.session;
    command.fence = fence;
    abi::Receipt receipt{};
    if (!abi::encode_sync(CommandBuffer(device.request, abi::sync_request_bytes), command) ||
        !request_receipt(device, abi::ioctl_sync, abi::sync_request_bytes, receipt) ||
        receipt.outcome != abi::outcome_completed || receipt.completed_fence < target) {
        display::count(device.statistics.sync_failures);
        if (receipt.outcome == abi::outcome_lost) {
            poison(device);
        }

        return false;
    }

    display::count(device.statistics.syncs);
    return true;
}

bool assert_primary(Pdev& device, bool enable) noexcept {
    device.primary_wanted = enable;
    if (!device.session_open || device.poisoned) {
        return !enable;
    }

    abi::AssertPrimaryCommand command{};
    command.session = device.session;
    command.enable = enable ? 1U : 0U;
    abi::Receipt receipt{};
    if (!abi::encode_assert_primary(CommandBuffer(device.request, abi::assert_primary_request_bytes),
                                    command) ||
        !request_receipt(device, abi::ioctl_assert_primary, abi::assert_primary_request_bytes, receipt)) {
        return false;
    }

    if (receipt.outcome == abi::outcome_lost) {
        poison(device);
    }

    return receipt.outcome == abi::outcome_completed;
}

// Reserve the next staging half, waiting for the GPU to finish reading or
// writing it. Returns the byte offset of the half or false on barrier failure.
bool acquire_half(Pdev& device, uint32_t& half, uint32_t& offset) noexcept {
    half = device.next_half;
    device.next_half = (device.next_half + 1U) % display::staging_halves;
    if (device.half_fence[half] > device.completed && !sync_fence(device, device.half_fence[half])) {
        return false;
    }

    offset = half * display::staging_half_bytes(device.staging_bytes);
    return true;
}

// ---------------------------------------------------------------------------
// Single requests. Each encodes one batch (at most abi::max_rects rectangles).
// ---------------------------------------------------------------------------
Attempt submit_fill(Pdev& device, uint32_t surface, uint32_t color, const abi::RectList& rects) noexcept {
    abi::FillCommand& command = device.fill;
    command = {};
    command.session = device.session;
    command.surface = surface;
    command.color = color;
    command.rects = rects;
    uint64_t fence = 0;
    if (!abi::encode_fill(CommandBuffer(device.request, abi::fill_request_bytes), command)) {
        return Attempt::unsupported;
    }

    return submit(device, abi::ioctl_fill, abi::fill_request_bytes, fence);
}

Attempt submit_copy(Pdev& device, uint32_t target, uint32_t source, int32_t dx, int32_t dy,
                    const abi::RectList& rects) noexcept {
    abi::CopyCommand& command = device.copy;
    command = {};
    command.session = device.session;
    command.target = target;
    command.source = source;
    command.source_dx = dx;
    command.source_dy = dy;
    command.rects = rects;
    uint64_t fence = 0;
    if (!abi::encode_copy(CommandBuffer(device.request, abi::copy_request_bytes), command)) {
        return Attempt::unsupported;
    }

    return submit(device, abi::ioctl_copy, abi::copy_request_bytes, fence);
}

// Encode and submit one transfer of rects (surface coordinates) whose staging
// image starts at (origin_x, origin_y) = byte offset with the given pitch.
Attempt submit_transfer(Pdev& device, bool upload, uint32_t surface, const abi::RectList& rects,
                        int32_t origin_x, int32_t origin_y, uint32_t offset, uint32_t pitch,
                        uint64_t& fence) noexcept {
    abi::TransferCommand& command = device.transfer;
    command = {};
    command.session = device.session;
    command.surface = surface;
    command.staging_offset = offset;
    command.staging_pitch = pitch;
    command.origin_x = origin_x;
    command.origin_y = origin_y;
    command.rects = rects;
    if (!abi::encode_transfer(CommandBuffer(device.request, abi::transfer_request_bytes), command)) {
        return Attempt::unsupported;
    }

    return submit(device, upload ? abi::ioctl_upload : abi::ioctl_readback,
                  abi::transfer_request_bytes, fence);
}

// Copy rows between a GDI bitmap and staging. area is the rectangle in GPU
// surface coordinates; the bitmap pixel of (x, y) is (x + dx, y + dy).
// Callers proved both spans in bounds (translated_inside and the staging plan).
void copy_rows(Pdev& device, SURFOBJ* bitmap, const abi::Rect& area, int32_t dx, int32_t dy,
               uint32_t offset, uint32_t pitch, int32_t origin_x, int32_t origin_y,
               bool to_staging) noexcept {
    const size_t row_bytes = static_cast<size_t>(area.right - area.left) * abi::bytes_per_pixel;
    for (int32_t y = area.top; y < area.bottom; ++y) {
        uint8_t* staging_row = device.staging + offset +
            static_cast<size_t>(y - origin_y) * pitch +
            static_cast<size_t>(area.left - origin_x) * abi::bytes_per_pixel;
        uint8_t* bitmap_row = bitmap_pixel(bitmap, area.left + dx, y + dy);
        if (to_staging) {
            memcpy(staging_row, bitmap_row, row_bytes);
        } else {
            memcpy(bitmap_row, staging_row, row_bytes);
        }
    }
}

// Upload (bitmap -> GPU surface) or readback (GPU surface -> bitmap) of the
// surface-space rectangle list rects. bitmap pixel = surface pixel + (dx, dy).
// A batch that fits one staging half is one IOCTL; larger work is banded per
// rectangle. A partial GPU transfer followed by an engine fallback is still
// exact: the operation is SRCCOPY from a source the transfer does not modify.
Attempt transfer(Pdev& device, bool upload, uint32_t surface, SURFOBJ* bitmap, const abi::RectList& rects,
                 int32_t dx, int32_t dy) noexcept {
    // The miniport already reported this handle lost (once); it is dead there.
    if (display::is_lost(device.lost_surfaces, surface)) {
        return Attempt::surface_lost;
    }

    const Rectangle bounds = display::list_bounds(rects);
    const uint32_t half_bytes = display::staging_half_bytes(device.staging_bytes);
    const uint32_t bounds_width = static_cast<uint32_t>(bounds.right - bounds.left);
    const uint32_t bounds_height = static_cast<uint32_t>(bounds.bottom - bounds.top);
    uint32_t half = 0;
    uint32_t offset = 0;
    uint64_t fence = 0;
    if (rects.count == 0) {
        return Attempt::unsupported;
    }

    if (display::fits_half(bounds_width, bounds_height, half_bytes)) {
        const uint32_t pitch = bounds_width * abi::bytes_per_pixel;
        if (!acquire_half(device, half, offset)) {
            return Attempt::failed;
        }

        if (upload) {
            for (uint32_t index = 0; index < rects.count; ++index) {
                copy_rows(device, bitmap, rects.rects[index], dx, dy, offset, pitch,
                          bounds.left, bounds.top, true);
            }
        }

        const Attempt result = submit_transfer(device, upload, surface, rects, bounds.left, bounds.top,
                                               offset, pitch, fence);
        if (result != Attempt::performed) {
            return result;
        }

        device.half_fence[half] = fence;
        if (!upload) {
            for (uint32_t index = 0; index < rects.count; ++index) {
                copy_rows(device, bitmap, rects.rects[index], dx, dy, offset, pitch,
                          bounds.left, bounds.top, false);
            }
        }

        return Attempt::performed;
    }

    for (uint32_t index = 0; index < rects.count; ++index) {
        const abi::Rect area = rects.rects[index];
        const uint32_t width = static_cast<uint32_t>(area.right - area.left);
        const uint32_t rows = display::band_rows(width, half_bytes);
        if (rows == 0) {
            // Nothing written by an upload of a later band is undone here, but
            // the operation as a whole is still a pure SRCCOPY of an unmodified
            // source, so the caller's engine fallback stays exact.
            return Attempt::unsupported;
        }

        for (int32_t top = area.top; top < area.bottom;) {
            const int64_t remaining = static_cast<int64_t>(area.bottom) - top;
            const int32_t height = static_cast<int32_t>(remaining < rows ? remaining : rows);
            abi::RectList& band = device.band;
            band = {};
            band.count = 1;
            band.rects[0] = {area.left, top, area.right, top + height};
            const uint32_t pitch = width * abi::bytes_per_pixel;
            if (!acquire_half(device, half, offset)) {
                return Attempt::failed;
            }

            if (upload) {
                copy_rows(device, bitmap, band.rects[0], dx, dy, offset, pitch, area.left, top, true);
            }

            const Attempt result = submit_transfer(device, upload, surface, band, area.left, top, offset,
                                                   pitch, fence);
            if (result != Attempt::performed) {
                return result;
            }

            device.half_fence[half] = fence;
            if (!upload) {
                copy_rows(device, bitmap, band.rects[0], dx, dy, offset, pitch, area.left, top, false);
            }

            top += height;
        }
    }

    return Attempt::performed;
}

// ---------------------------------------------------------------------------
// Device bitmap residency.
// ---------------------------------------------------------------------------
void account_release(Pdev& device, DeviceBitmap& bitmap) noexcept {
    if (bitmap.surface != 0) {
        --device.resident_count;
    }

    device.resident_bytes -= bitmap.bytes;
    bitmap.bytes = 0;
    bitmap.surface = 0;
    device.statistics.live_bitmap_bytes = device.resident_bytes;
}

// Destroy the miniport surface (the miniport proves its last use retired).
void destroy_surface(Pdev& device, DeviceBitmap& bitmap) noexcept {
    if (bitmap.surface == 0) {
        return;
    }

    if (session_usable(device)) {
        abi::SurfaceDestroyCommand command{};
        command.session = device.session;
        command.surface = bitmap.surface;
        abi::Receipt receipt{};
        if (abi::encode_surface_destroy(CommandBuffer(device.request, abi::surface_destroy_request_bytes),
                                        command) &&
            request_receipt(device, abi::ioctl_surface_destroy, abi::surface_destroy_request_bytes, receipt) &&
            receipt.outcome == abi::outcome_lost) {
            poison(device);
        }
    }

    // A closed or poisoned session already gave the storage back (or the
    // miniport quarantined it); the handle is dead either way.
    account_release(device, bitmap);
}

void release_dib(DeviceBitmap& bitmap) noexcept {
    if (bitmap.dib != nullptr) {
        EngUnlockSurface(bitmap.dib);
        bitmap.dib = nullptr;
    }

    if (bitmap.dib_handle != nullptr) {
        EngDeleteSurface(bitmap.dib_handle);
        bitmap.dib_handle = nullptr;
    }
}

// Engine-allocated (zeroed) top-down 32bpp DIB of the bitmap's size.
bool create_dib(DeviceBitmap& bitmap) noexcept {
    const SIZEL size{static_cast<LONG>(bitmap.width), static_cast<LONG>(bitmap.height)};
    bitmap.dib_handle = reinterpret_cast<HSURF>(EngCreateBitmap(size,
        static_cast<LONG>(bitmap.width * abi::bytes_per_pixel), BMF_32BPP, BMF_TOPDOWN, nullptr));
    if (bitmap.dib_handle != nullptr) {
        bitmap.dib = EngLockSurface(bitmap.dib_handle);
    }

    if (bitmap.dib == nullptr || !engine_bitmap32(bitmap.dib)) {
        release_dib(bitmap);
        return false;
    }

    return true;
}

abi::RectList& whole(Pdev& device, const DeviceBitmap& bitmap) noexcept {
    abi::RectList& list = device.moved;
    list = {};
    list.count = 1;
    list.rects[0] = {0, 0, static_cast<int32_t>(bitmap.width), static_cast<int32_t>(bitmap.height)};
    return list;
}

// Move a resident bitmap into a DIB for CPU access: readback of every pixel,
// then the VRAM surface is destroyed. A surface whose contents the miniport
// lost (failed eviction) becomes a zeroed DIB and is counted. false: no CPU
// view could be produced (the bitmap stays resident and unchanged).
bool demote(Pdev& device, DeviceBitmap& bitmap) noexcept {
    if (bitmap.surface == 0) {
        return bitmap.dib != nullptr;
    }

    if (!create_dib(bitmap)) {
        display::count(device.statistics.demotion_failures);
        return false;
    }

    const Attempt result = session_usable(device) ?
        transfer(device, false, bitmap.surface, bitmap.dib, whole(device, bitmap), 0, 0) : Attempt::failed;
    if (result == Attempt::surface_lost) {
        // The miniport lost the VRAM contents (failed eviction): the handle
        // answers surface_lost until destroyed and no byte of the engine-zeroed
        // DIB came from it.
        display::count(device.statistics.surfaces_lost);
    } else if (result != Attempt::performed) {
        release_dib(bitmap);
        display::count(device.statistics.demotion_failures);
        return false;
    }

    destroy_surface(device, bitmap);
    bitmap.gpu_uses = 0;
    display::count(device.statistics.demotions);
    return true;
}

// Create a miniport surface for a bitmap of width x height (accounting and
// budget included). Returns the handle or 0.
uint32_t create_surface(Pdev& device, uint32_t width, uint32_t height, uint64_t& bytes) noexcept {
    abi::SurfaceCreateCommand command{};
    command.session = device.session;
    command.width = width;
    command.height = height;
    command.format = abi::format_xrgb8888;
    DWORD returned = 0;
    abi::SurfaceGrant grant{};
    if (!session_usable(device) ||
        !abi::encode_surface_create(CommandBuffer(device.request, abi::surface_create_request_bytes), command) ||
        EngDeviceIoControl(device.driver, abi::ioctl_surface_create, device.request,
                           abi::surface_create_request_bytes, device.reply, abi::surface_grant_bytes,
                           &returned) != 0 ||
        returned != abi::surface_grant_bytes ||
        !abi::decode_surface_grant(ByteView(device.reply, abi::surface_grant_bytes), grant)) {
        return 0;
    }

    if (grant.width != width || grant.height != height || grant.format != abi::format_xrgb8888 ||
        grant.surface == 0) {
        // Never handed to GDI: give it back immediately.
        DeviceBitmap stray{};
        stray.surface = grant.surface;
        destroy_surface(device, stray);
        return 0;
    }

    bytes = static_cast<uint64_t>(grant.pitch) * grant.height;
    return grant.surface;
}

// Move a demoted bitmap back into a new VRAM surface (upload of every pixel).
// Failure keeps the DIB; nothing observable changes.
bool promote(Pdev& device, DeviceBitmap& bitmap) noexcept {
    bitmap.gpu_uses = 0;
    if (bitmap.dib == nullptr || bitmap.surface != 0 || !session_usable(device) ||
        display::admit_bitmap(static_cast<int32_t>(bitmap.width), static_cast<int32_t>(bitmap.height),
                              device.resident_count, device.resident_bytes, device.bitmap_limits) !=
            display::BitmapAdmission::admit) {
        return false;
    }

    uint64_t bytes = 0;
    const uint32_t surface = create_surface(device, bitmap.width, bitmap.height, bytes);
    if (surface == 0) {
        return false;
    }

    bitmap.surface = surface;
    bitmap.bytes = bytes;
    device.resident_bytes += bytes;
    ++device.resident_count;
    if (transfer(device, true, surface, bitmap.dib, whole(device, bitmap), 0, 0) != Attempt::performed) {
        destroy_surface(device, bitmap);
        return false;
    }

    release_dib(bitmap);
    device.statistics.live_bitmap_bytes = device.resident_bytes;
    display::count(device.statistics.promotions);
    return true;
}

// Promotion policy for a demoted bitmap operand of an operation that would be
// GPU-expressible with it resident.
void consider_promotion(Pdev& device, const Operand& operand, bool expressible) noexcept {
    DeviceBitmap* bitmap = operand.bitmap;
    if (bitmap == nullptr || operand.owner != &device || bitmap->surface != 0) {
        return;
    }

    if (!expressible) {
        bitmap->gpu_uses = 0;
        return;
    }

    if (++bitmap->gpu_uses >= display::promote_after_uses) {
        promote(device, *bitmap);
    }
}

// CPU view of an operand for an Eng* call: the framebuffer DIB after a GPU
// barrier, a demoted device bitmap's DIB, or the engine bitmap itself.
// nullptr: the operand cannot be accessed exactly (the call must fail).
SURFOBJ* cpu_view(const Operand& operand) noexcept {
    if (operand.original == nullptr) {
        return nullptr;
    }

    if (operand.primary) {
        return sync_fence(*operand.owner, 0) ? operand.owner->bitmap : nullptr;
    }

    if (operand.bitmap != nullptr) {
        DeviceBitmap& bitmap = *operand.bitmap;
        if (bitmap.surface != 0) {
            bitmap.gpu_uses = 0;
            return demote(*operand.owner, bitmap) ? bitmap.dib : nullptr;
        }

        bitmap.gpu_uses = 0;
        return bitmap.dib;
    }

    return operand.original;
}

// GPU handle and size of a GPU-surface operand of device (caller classified it).
uint32_t gpu_handle(const Pdev& device, const Operand& operand) noexcept {
    return operand.primary ? device.primary_handle : operand.bitmap->surface;
}

void gpu_size(const Pdev& device, const Operand& operand, uint32_t& width, uint32_t& height) noexcept {
    if (operand.primary) {
        width = device.width;
        height = device.height;
    } else {
        width = operand.bitmap->width;
        height = operand.bitmap->height;
    }
}

// Size of any operand (engine bitmaps from their SURFOBJ; 0 when unusable).
void operand_size(const Pdev& device, const Operand& operand, uint32_t& width, uint32_t& height) noexcept {
    if (operand.primary || operand.bitmap != nullptr) {
        gpu_size(device, operand, width, height);
        return;
    }

    const SURFOBJ* surface = operand.original;
    const bool valid = surface != nullptr && surface->sizlBitmap.cx > 0 && surface->sizlBitmap.cy > 0;
    width = valid ? static_cast<uint32_t>(surface->sizlBitmap.cx) : 0;
    height = valid ? static_cast<uint32_t>(surface->sizlBitmap.cy) : 0;
}

// CPU pixels of an operand that is a 32bpp DIB (engine bitmap or demoted).
SURFOBJ* dib_of(const Operand& operand) noexcept {
    return operand.bitmap != nullptr ? operand.bitmap->dib : operand.original;
}

// ---------------------------------------------------------------------------
// Batched operations.
// ---------------------------------------------------------------------------
void record_batches(Pdev& device, uint32_t batches, uint64_t rects) noexcept {
    if (batches > 1) {
        display::count(device.statistics.complex_clips);
        display::add(device.statistics.clip_batches, batches);
    }

    if (rects > device.statistics.max_clip_rects) {
        device.statistics.max_clip_rects = rects;
    }
}

// True while the clip being executed needs (or needed) more than one request.
bool large_clip(const ClipCursor& cursor, uint32_t batches) noexcept {
    return cursor.complex && (batches != 0 || cursor.more || cursor.position < cursor.enumeration.count);
}

// Batched execution of an operation that does not read what it writes. A
// decline after earlier batches is still an exact fallback (the engine redoes
// the whole operation); every large clip that falls back, including a failed
// enumeration, is counted in clip_fallbacks.
template<typename Submit>
Attempt for_each_batch(Pdev& device, Submit submit_batch) noexcept {
    uint32_t batches = 0;
    uint64_t rects = 0;
    while (clip_next(device.clip, device.batch)) {
        const Attempt result = submit_batch(device.batch);
        if (result != Attempt::performed) {
            if (untouched(result) && large_clip(device.clip, batches)) {
                display::count(device.statistics.clip_fallbacks);
            }

            return result;
        }

        ++batches;
        rects += device.batch.count;
    }

    if (device.clip.failed) {
        display::count(device.statistics.clip_fallbacks);
        return Attempt::unsupported;
    }

    record_batches(device, batches, rects);
    return Attempt::performed;
}

// Same-surface copy. Before anything is submitted the whole enumeration is
// checked to be banded in the copy direction (radeon_display.hpp) and inside
// the surface; then batches run in that order. A decline after earlier
// batches finishes the remaining rectangles on the CPU view in the same order,
// which keeps the snapshot semantics (earlier batches never wrote a later
// source). requested/clip as for clip_begin.
Attempt aliased_copy(Pdev& device, const Operand& surface, const RECTL& requested, CLIPOBJ* clip,
                     int32_t dx, int32_t dy) noexcept {
    uint32_t width = 0;
    uint32_t height = 0;
    gpu_size(device, surface, width, height);
    const display::CopyDirection direction = display::copy_direction(dx, dy);
    const ULONG enumeration = enumeration_direction(direction);
    if (!clip_begin(requested, clip, width, height, enumeration, device.clip)) {
        return Attempt::unsupported;
    }

    display::BandOrder order(direction);
    while (clip_next(device.clip, device.batch)) {
        if (!display::translated_inside(device.batch, dx, dy, width, height)) {
            if (device.clip.complex) {
                display::count(device.statistics.clip_fallbacks);
            }

            return Attempt::unsupported;
        }

        for (uint32_t index = 0; index < device.batch.count && device.clip.complex; ++index) {
            const abi::Rect& rect = device.batch.rects[index];
            order.accept({rect.left, rect.top, rect.right, rect.bottom});
        }
    }

    if (device.clip.failed || order.failed()) {
        display::count(device.statistics.clip_fallbacks);
        return Attempt::unsupported;
    }

    clip_begin(requested, clip, width, height, enumeration, device.clip);
    const uint32_t handle = gpu_handle(device, surface);
    uint32_t batches = 0;
    uint64_t rects = 0;
    while (clip_next(device.clip, device.batch)) {
        const Attempt result = submit_copy(device, handle, handle, dx, dy, device.batch);
        if (result == Attempt::performed) {
            ++batches;
            rects += device.batch.count;
            continue;
        }

        if (batches == 0 || !untouched(result)) {
            if (untouched(result) && large_clip(device.clip, batches)) {
                display::count(device.statistics.clip_fallbacks);
            }

            return result;
        }

        // Earlier batches ran on the GPU: finish this and every later batch
        // on the CPU (barrier or demotion first), in enumeration order.
        SURFOBJ* view = cpu_view(surface);
        if (view == nullptr) {
            return Attempt::failed;
        }

        do {
            for (uint32_t index = 0; index < device.batch.count; ++index) {
                cpu_copy_rect(view, device.batch.rects[index], dx, dy);
            }

            display::add(device.statistics.cpu_finished_rects, device.batch.count);
        } while (clip_next(device.clip, device.batch));

        return device.clip.failed ? Attempt::failed : Attempt::performed;
    }

    if (device.clip.failed) {
        // Unreachable after a successful verification pass of the same CLIPOBJ.
        return batches == 0 ? Attempt::unsupported : Attempt::failed;
    }

    record_batches(device, batches, rects);
    return Attempt::performed;
}

bool needs_primary(const Operand& a, const Operand& b) noexcept {
    return a.primary || b.primary;
}

void count_offscreen(Pdev& device, const Operand& target, const Operand& source) noexcept {
    if ((target.bitmap != nullptr && target.bitmap->surface != 0) ||
        (source.bitmap != nullptr && source.bitmap->surface != 0)) {
        display::count(device.statistics.offscreen_operations);
    }
}

// The single acceleration entry for BitBlt/CopyBits. device is the PDEV whose
// GPU surface is the target (fill/copy/upload) or the source (readback).
Attempt accelerate(Pdev& device, const Operand& target, const Operand& source, CLIPOBJ* clip,
                   XLATEOBJ* translation, const RECTL& rectangle, const POINTL* source_point, bool masked,
                   BRUSHOBJ* brush, uint32_t rop4) noexcept {
    if (!session_usable(device) && !revive_session(device)) {
        return Attempt::unsupported;
    }

    const uint32_t brush_color = brush != nullptr ? brush->iSolidColor : display::no_solid_color;
    // Promotion: would this operation be GPU work with the demoted operands resident?
    const display::BlitDecision hypothetical = display::classify_blit(rop4, masked, brush_color,
        kind_with(target, device, nullptr, true), kind_with(source, device, translation, true));
    consider_promotion(device, target, hypothetical.plan != display::BlitPlan::fallback);
    if (source.bitmap != target.bitmap) {
        consider_promotion(device, source, hypothetical.plan != display::BlitPlan::fallback);
    }

    const display::BlitDecision decision = display::classify_blit(rop4, masked, brush_color,
        kind_with(target, device, nullptr, false), kind_with(source, device, translation, false));
    if (decision.plan == display::BlitPlan::fallback ||
        (needs_primary(target, source) && !device.accelerated)) {
        return Attempt::unsupported;
    }

    uint32_t width = 0;
    uint32_t height = 0;
    operand_size(device, target, width, height);
    if (width == 0 || height == 0) {
        return Attempt::unsupported;
    }

    if (decision.plan == display::BlitPlan::fill) {
        const uint32_t handle = gpu_handle(device, target);
        if (!clip_begin(rectangle, clip, width, height, CD_ANY, device.clip)) {
            return Attempt::unsupported;
        }

        const Attempt result = for_each_batch(device, [&](const abi::RectList& batch) noexcept {
            return submit_fill(device, handle, decision.color, batch);
        });
        if (result == Attempt::performed) {
            display::count(device.statistics.gpu_fills);
            count_offscreen(device, target, source);
        }

        return result;
    }

    if (source_point == nullptr) {
        return Attempt::unsupported;
    }

    // Source pixel = destination pixel + (dx, dy); 64-bit checked narrowing.
    const int64_t wide_dx = static_cast<int64_t>(source_point->x) - rectangle.left;
    const int64_t wide_dy = static_cast<int64_t>(source_point->y) - rectangle.top;
    if (wide_dx < INT32_MIN || wide_dx > INT32_MAX || wide_dy < INT32_MIN || wide_dy > INT32_MAX) {
        return Attempt::unsupported;
    }

    const int32_t dx = static_cast<int32_t>(wide_dx);
    const int32_t dy = static_cast<int32_t>(wide_dy);
    uint32_t source_width = 0;
    uint32_t source_height = 0;
    operand_size(device, source, source_width, source_height);
    if (source_width == 0 || source_height == 0) {
        return Attempt::unsupported;
    }

    Attempt result = Attempt::unsupported;
    if (decision.plan == display::BlitPlan::device_copy) {
        const bool aliased = target.primary ? source.primary : target.bitmap == source.bitmap;
        if (aliased) {
            result = aliased_copy(device, target, rectangle, clip, dx, dy);
        } else if (clip_begin(rectangle, clip, width, height, CD_ANY, device.clip)) {
            const uint32_t target_handle = gpu_handle(device, target);
            const uint32_t source_handle = gpu_handle(device, source);
            result = for_each_batch(device, [&](const abi::RectList& batch) noexcept {
                return display::translated_inside(batch, dx, dy, source_width, source_height) ?
                    submit_copy(device, target_handle, source_handle, dx, dy, batch) : Attempt::unsupported;
            });
        }

        if (result == Attempt::performed) {
            display::count(device.statistics.gpu_copies);
            count_offscreen(device, target, source);
        }

        return result;
    }

    if (!clip_begin(rectangle, clip, width, height, CD_ANY, device.clip)) {
        return Attempt::unsupported;
    }

    if (decision.plan == display::BlitPlan::upload) {
        const uint32_t handle = gpu_handle(device, target);
        SURFOBJ* pixels = dib_of(source);
        result = for_each_batch(device, [&](const abi::RectList& batch) noexcept {
            return display::translated_inside(batch, dx, dy, source_width, source_height) ?
                transfer(device, true, handle, pixels, batch, dx, dy) : Attempt::unsupported;
        });
        if (result == Attempt::performed) {
            display::count(device.statistics.gpu_uploads);
            count_offscreen(device, target, source);
        }

        return result;
    }

    // Readback: the target clip, expressed in source (GPU surface) coordinates.
    const uint32_t handle = gpu_handle(device, source);
    SURFOBJ* pixels = dib_of(target);
    result = for_each_batch(device, [&](const abi::RectList& batch) noexcept {
        if (!display::translated_inside(batch, dx, dy, source_width, source_height)) {
            return Attempt::unsupported;
        }

        abi::RectList& moved = device.moved;
        moved = batch;
        for (uint32_t index = 0; index < moved.count; ++index) {
            abi::Rect& rect = moved.rects[index];
            rect = {rect.left + dx, rect.top + dy, rect.right + dx, rect.bottom + dy};
        }

        return transfer(device, false, handle, pixels, moved, -dx, -dy);
    });
    if (result == Attempt::performed) {
        display::count(device.statistics.gpu_readbacks);
        count_offscreen(device, target, source);
    }

    return result;
}

// Demote every resident bitmap (session close): contents survive as DIBs.
void demote_all(Pdev& device) noexcept {
    for (DeviceBitmap* bitmap = device.bitmaps; bitmap != nullptr; bitmap = bitmap->next) {
        if (bitmap->surface != 0 && !demote(device, *bitmap)) {
            // No readback was possible: the contents are gone with the session.
            if (create_dib(*bitmap)) {
                display::count(device.statistics.surfaces_lost);
            }

            destroy_surface(device, *bitmap);
        }
    }
}

void release_scratch(Pdev& device) noexcept;

void close_session(Pdev& device) noexcept {
    if (!device.session_open) {
        return;
    }

    release_scratch(device);
    demote_all(device);
    sync_fence(device, 0);
    assert_primary(device, false);
    DWORD returned = 0;
    if (abi::encode_close_session(CommandBuffer(device.request, abi::close_session_request_bytes),
                                  device.session)) {
        EngDeviceIoControl(device.driver, abi::ioctl_close_session, device.request,
                           abi::close_session_request_bytes, nullptr, 0, &returned);
    }

    // The miniport frees (or quarantines) staging and surfaces; never touch
    // them again.
    device.session_open = false;
    device.accelerated = false;
    device.staging = nullptr;
    device.staging_bytes = 0;
    device.session = {};
    device.resident_bytes = 0;
    device.resident_count = 0;
    device.statistics.live_bitmap_bytes = 0;
}

// Open a miniport session for this PDEV's mode. Any mismatch leaves the PDEV
// unaccelerated, which is a complete and exact CPU display driver.
void open_session(Pdev& device) noexcept {
    device.accelerated = false;
    DWORD returned = 0;
    abi::Capabilities capabilities{};
    if (EngDeviceIoControl(device.driver, abi::ioctl_query, nullptr, 0, device.reply,
                           abi::capabilities_bytes, &returned) != 0 ||
        returned != abi::capabilities_bytes ||
        !abi::decode_capabilities(ByteView(device.reply, abi::capabilities_bytes), capabilities) ||
        (capabilities.flags & required_capabilities) != required_capabilities ||
        capabilities.max_rects < abi::max_rects || capabilities.primary_width != device.width ||
        capabilities.primary_height != device.height || capabilities.primary_pitch != device.pitch ||
        capabilities.primary_format != abi::format_xrgb8888 ||
        !abi::encode_open_session(CommandBuffer(device.request, abi::open_session_request_bytes))) {
        return;
    }

    abi::SessionGrant grant{};
    if (EngDeviceIoControl(device.driver, abi::ioctl_open_session, device.request,
                           abi::open_session_request_bytes, device.reply, abi::session_grant_bytes,
                           &returned) != 0 ||
        returned != abi::session_grant_bytes ||
        !abi::decode_session_grant(ByteView(device.reply, abi::session_grant_bytes), grant)) {
        return;
    }

    device.session_open = true;
    device.session = grant.session;
    // A fresh session: an earlier session's poisoning does not carry over.
    device.poisoned = false;
    device.recovery_generation = capabilities.recovery_generation;
    device.revival_queries = 0;
    device.revival_closed = false;
    device.gpu_quiescent = false;
    device.lost_surfaces = {};
    device.last_fence = 0;
    device.completed = 0;
    device.half_fence[0] = 0;
    device.half_fence[1] = 0;
    device.next_half = 0;
    device.resident_bytes = 0;
    device.resident_count = 0;
    const uint32_t half = display::staging_half_bytes(grant.staging_bytes);
    if (grant.primary_width != device.width || grant.primary_height != device.height ||
        grant.primary_pitch != device.pitch || grant.primary_format != abi::format_xrgb8888 ||
        (grant.flags & required_capabilities) != required_capabilities ||
        grant.staging_address == 0 || grant.staging_address > UINTPTR_MAX ||
        (grant.staging_bytes & 3U) != 0 ||
        display::band_rows(device.width, half) == 0 ||
        static_cast<uint64_t>(grant.staging_bytes) > UINTPTR_MAX - grant.staging_address) {
        close_session(device);
        return;
    }

    device.primary_handle = grant.primary_surface;
    // The miniport's kernel mapping of its own staging allocation, valid for
    // staging_bytes until close_session. Never derived from caller input.
    device.staging = reinterpret_cast<uint8_t*>(static_cast<uintptr_t>(grant.staging_address));
    device.staging_bytes = grant.staging_bytes;
    // Offscreen bitmaps only with the eviction guarantee: their contents must
    // survive the miniport's power transitions.
    const bool offscreen = (grant.flags & abi::capability_offscreen) != 0 &&
                           (grant.flags & abi::capability_eviction) != 0;
    device.bitmap_limits.max_count = offscreen ?
        (capabilities.max_session_surfaces < abi::max_session_surfaces ? capabilities.max_session_surfaces :
                                                                         abi::max_session_surfaces) : 0;
    device.bitmap_limits.budget_bytes = offscreen ? capabilities.session_surface_bytes : 0;
    device.raster_ops = (grant.flags & abi::capability_raster) != 0 ?
        capabilities.operations & abi::raster_operations : 0;
    device.accelerated = assert_primary(device, true);
}

// ---------------------------------------------------------------------------
// Compute raster operations (abi version 3).
// ---------------------------------------------------------------------------
bool raster_available(const Pdev& device, uint32_t code) noexcept {
    return session_usable(device) && (device.raster_ops & abi::operation_bit(code)) != 0;
}

// A GPU-surface operand of device usable now (primary needs the scanout).
bool gpu_ready(const Pdev& device, const Operand& operand) noexcept {
    return operand.owner == &device && (operand.primary ? device.accelerated :
                                        operand.bitmap != nullptr && operand.bitmap->surface != 0);
}

abi::Operand surface_operand(const Pdev& device, const Operand& operand) noexcept {
    abi::Operand value{};
    value.kind = abi::operand_surface;
    value.handle = gpu_handle(device, operand);
    value.format = abi::format_xrgb8888;
    return value;
}

abi::Operand staging_operand(uint32_t offset, uint32_t width, uint32_t height, uint32_t format) noexcept {
    abi::Operand value{};
    value.kind = abi::operand_staging;
    value.handle = offset;
    value.pitch = width * abi::bytes_per_pixel;   // the engine's pitch >= 4 * width rule
    value.width = width;
    value.height = height;
    value.format = format;
    return value;
}

// Copies a width x height block of a 32bpp DIB at (x, y) into staging (pitch
// 4 * width). Caller proved the block inside the DIB and the staging half.
void stage_bitmap32(Pdev& device, SURFOBJ* bitmap, int32_t x, int32_t y, uint32_t width, uint32_t height,
                    uint32_t offset) noexcept {
    const size_t row = static_cast<size_t>(width) * abi::bytes_per_pixel;
    for (uint32_t index = 0; index < height; ++index) {
        memcpy(device.staging + offset + index * row, bitmap_pixel(bitmap, x, y + static_cast<int32_t>(index)), row);
    }
}

// Copies width x height bits of a 1bpp DIB at (x, y) into staging rows of pitch
// 4 * width (MSB first, bit 0 = pixel x). Caller proved both spans.
void stage_mono(Pdev& device, SURFOBJ* bitmap, int32_t x, int32_t y, uint32_t width, uint32_t height,
                uint32_t offset) noexcept {
    const size_t pitch = static_cast<size_t>(width) * abi::bytes_per_pixel;
    for (uint32_t index = 0; index < height; ++index) {
        // GDI owns pvScan0; row y + index lies inside the bitmap, lDelta may be negative.
        const uint8_t* row = static_cast<const uint8_t*>(bitmap->pvScan0) +
            static_cast<intptr_t>(y + static_cast<int32_t>(index)) * bitmap->lDelta;
        display::copy_mono_bits(row, static_cast<uint32_t>(x), width, device.staging + offset + index * pitch);
    }
}

bool engine_bitmap1(SURFOBJ* surface) noexcept {
    return surface != nullptr && surface->iType == STYPE_BITMAP && surface->iBitmapFormat == BMF_1BPP &&
           surface->pvScan0 != nullptr && surface->sizlBitmap.cx > 0 && surface->sizlBitmap.cy > 0;
}

bool engine_bitmap8(SURFOBJ* surface) noexcept {
    if (surface == nullptr || surface->iType != STYPE_BITMAP || surface->iBitmapFormat != BMF_8BPP ||
        surface->pvScan0 == nullptr || surface->sizlBitmap.cx <= 0 || surface->sizlBitmap.cy <= 0) {
        return false;
    }
    const int64_t pitch = surface->lDelta < 0 ? -static_cast<int64_t>(surface->lDelta) : surface->lDelta;
    return pitch >= surface->sizlBitmap.cx;
}

// One bounded 8-bpp mask rectangle; the compute operand uses the required
// 4*width pitch. The bitmap and staging-half spans were checked by the caller.
void stage_coverage8(Pdev& device, SURFOBJ* bitmap, int32_t x, int32_t y, uint32_t width,
                     uint32_t height, uint32_t offset) noexcept {
    const size_t pitch = static_cast<size_t>(width) * abi::bytes_per_pixel;
    for (uint32_t row = 0; row < height; ++row) {
        const uint8_t* source = static_cast<const uint8_t*>(bitmap->pvScan0) +
            static_cast<intptr_t>(y + static_cast<int32_t>(row)) * bitmap->lDelta + x;
        uint8_t* destination = device.staging + offset + static_cast<size_t>(row) * pitch;
        memcpy(destination, source, width);
        memset(destination + width, 0, pitch - width);
    }
}

// A w x h block at (x, y) inside a cx x cy bitmap.
bool block_inside(int64_t x, int64_t y, int64_t width, int64_t height, int64_t cx, int64_t cy) noexcept {
    return x >= 0 && y >= 0 && width > 0 && height > 0 && x + width <= cx && y + height <= cy;
}

// DrvRealizeBrush tile: 64 BGRX dwords of an 8x8 pattern in target format.
constexpr uint32_t realized_magic = 0x54524452U; // "RDRT"
struct RealizedBrush final {
    uint32_t magic;
    uint32_t tile[64];
};

const RealizedBrush* realized_tile(BRUSHOBJ* brush) noexcept {
    if (brush == nullptr || brush->iSolidColor != display::no_solid_color) {
        return nullptr;
    }

    // Our DrvRealizeBrush created it; GDI keeps it alive with the brush object.
    const auto* realized = static_cast<const RealizedBrush*>(BRUSHOBJ_pvGetRbrush(brush));
    return realized != nullptr && realized->magic == realized_magic ? realized : nullptr;
}

// Submits device.rop (already filled except rects) once per clip batch.
Attempt submit_rop_batches(Pdev& device, uint32_t code) noexcept {
    const uint32_t bytes = code == abi::ioctl_pattern ? abi::pattern_request_bytes : abi::rop_request_bytes;
    return for_each_batch(device, [&](const abi::RectList& batch) noexcept {
        device.rop.rects = batch;
        uint64_t fence = 0;
        if (!abi::encode_rop(CommandBuffer(device.request, bytes), device.rop)) {
            return Attempt::unsupported;
        }

        return submit(device, code, bytes, fence);
    });
}

// 8/16/24bpp engine bitmaps (abi translated formats), 0 for anything else.
uint32_t translated_engine_format(SURFOBJ* surface) noexcept {
    if (surface == nullptr || surface->iType != STYPE_BITMAP || surface->pvScan0 == nullptr ||
        surface->sizlBitmap.cx <= 0 || surface->sizlBitmap.cy <= 0) {
        return 0;
    }

    const uint32_t format = surface->iBitmapFormat == BMF_8BPP ? abi::format_index8 :
                            surface->iBitmapFormat == BMF_16BPP ? abi::format_rgb16 :
                            surface->iBitmapFormat == BMF_24BPP ? abi::format_rgb24 : 0U;
    const int64_t pitch = surface->lDelta < 0 ? -static_cast<int64_t>(surface->lDelta) : surface->lDelta;
    return format != 0 && pitch >= static_cast<int64_t>(surface->sizlBitmap.cx) * display::staged_bytes_per_pixel(format) ?
        format : 0U;
}

// Whether the BITFIELDS masks of one XLATEOBJ palette are 555/565 (or unreadable),
// the RGB16 optimizations win32k derives from them.
bool rgb16_palette(XLATEOBJ* translation, ULONG which, uint32_t type) noexcept {
    if ((type & display::palette_bitfields) == 0) {
        return false;
    }

    ULONG masks[3]{};
    return XLATEOBJ_cGetPalette(translation, which, 3, masks) != 3 ||
           display::rgb16_masks(masks[0], masks[1], masks[2]);
}

// A 24bpp source is staged with three byte-plane tables: only for an OR-linear
// XLATEOBJ (radeon_display.hpp or_linear_translation).
bool or_linear(XLATEOBJ* translation) noexcept {
    if (trivial(translation)) {
        return true;
    }

    return display::or_linear_translation(false, translation->iSrcType, translation->iDstType,
                                          rgb16_palette(translation, XO_SRCBITFIELDS, translation->iSrcType),
                                          rgb16_palette(translation, XO_DESTBITFIELDS, translation->iDstType));
}

// Sampling a 64Ki-entry rgb16 table costs about as much as translating that many
// pixels on the CPU; smaller rgb16 stretches stay with the engine (still exact).
constexpr int64_t min_rgb16_destination_pixels = 4096;

// A translated DrvBitBlt source block at offset of half: the XLATEOBJ table
// (once per half and operation, as stage_stretch_band) and then rows of the
// block's raw pixel bytes at the translated pitch. Caller proved the block
// inside the bitmap and the half.
void stage_translated_block(Pdev& device, SURFOBJ* bits, XLATEOBJ* translation, uint32_t format, int32_t x, int32_t y,
                            uint32_t width, uint32_t height, uint32_t half, uint32_t offset) noexcept {
    const uint32_t table_bytes = abi::staging_table_bytes(format);
    if (device.table_serial[half] != device.stretch_serial) {
        const uint32_t other = (half + 1U) % display::staging_halves;
        const uint32_t half_bytes = display::staging_half_bytes(device.staging_bytes);
        // A table is always at the start of its band, the half's own offset.
        if (device.table_serial[other] == device.stretch_serial) {
            memcpy(device.staging + offset, device.staging + other * half_bytes, table_bytes);
        } else {
            display::write_translation_table(format, [translation](uint32_t value) noexcept {
                return translation != nullptr ? static_cast<uint32_t>(XLATEOBJ_iXlate(translation, value)) : value;
            }, CommandBuffer(device.staging + offset, table_bytes));
        }

        device.table_serial[half] = device.stretch_serial;
    }

    const uint32_t pixel_bytes = display::staged_bytes_per_pixel(format);
    const size_t pitch = static_cast<size_t>(display::staged_pitch(format, width));
    for (uint32_t row = 0; row < height; ++row) {
        // GDI owns pvScan0; row y + row and bytes [x, x + width) * pixel_bytes lie
        // inside the bitmap (block_inside); lDelta may be negative.
        const uint8_t* source = static_cast<const uint8_t*>(bits->pvScan0) +
            static_cast<intptr_t>(y + static_cast<int32_t>(row)) * bits->lDelta +
            static_cast<intptr_t>(x) * static_cast<intptr_t>(pixel_bytes);
        uint8_t* destination = device.staging + offset + table_bytes + row * pitch;
        memcpy(destination, source, static_cast<size_t>(width) * pixel_bytes);
        memset(destination + static_cast<size_t>(width) * pixel_bytes, 0, pitch - static_cast<size_t>(width) * pixel_bytes);
    }
}

// General DrvBitBlt on the compute engine (ROP4, patterns, mono sources,
// 1bpp masks such as the ReactOS mono glyph path IntEngMaskBlt ->
// ROP4_MASKPAINT). Staged operands are uploaded per horizontal band that fits
// one staging half. A destination-reading ROP is not idempotent: a decline
// after an earlier request cannot fall back exactly and fails the call.
Attempt compute_bitblt(Pdev& device, const Operand& target, const Operand& source, SURFOBJ* mask,
                       CLIPOBJ* clip, XLATEOBJ* translation, const RECTL& rectangle, const POINTL* source_point,
                       const POINTL* mask_point, BRUSHOBJ* brush, const POINTL* brush_origin,
                       uint32_t rop4) noexcept {
    const display::SurfaceKind target_kind = kind_with(target, device, nullptr, false);
    if (!gpu_ready(device, target) || !display::gpu_surface(target_kind)) {
        return Attempt::unsupported;
    }

    const bool uses_source = display::rop4_uses_source(rop4);
    display::RasterSource source_kind = display::RasterSource::none;
    uint32_t mono_foreground = 0;
    uint32_t mono_background = 0;
    uint32_t translated_format = 0;
    SURFOBJ* source_bits = nullptr;
    if (uses_source) {
        const display::SurfaceKind kind = kind_with(source, device, translation, false);
        const uint32_t engine_format = source.owner == nullptr ? translated_engine_format(source.original) : 0U;
        const int64_t area = (static_cast<int64_t>(rectangle.right) - rectangle.left) *
                             (static_cast<int64_t>(rectangle.bottom) - rectangle.top);
        if (source.original == nullptr) {
            source_kind = display::RasterSource::unsupported;
        } else if (display::gpu_surface(kind) && gpu_ready(device, source)) {
            source_kind = display::RasterSource::gpu;
        } else if (kind == display::SurfaceKind::bitmap32) {
            source_kind = display::RasterSource::bitmap32;
            source_bits = dib_of(source);
        } else if (source.owner == nullptr && engine_bitmap1(source.original) && translation != nullptr) {
            source_kind = display::RasterSource::mono;
            source_bits = source.original;
            mono_background = XLATEOBJ_iXlate(translation, 0);
            mono_foreground = XLATEOBJ_iXlate(translation, 1);
        } else if (engine_format != 0 && (engine_format != abi::format_rgb24 || or_linear(translation)) &&
                   (engine_format != abi::format_rgb16 || area >= min_rgb16_destination_pixels)) {
            // The generated DIB_32BPP_BitBlt forms translate every raw pixel with
            // XLATEOBJ_iXlate: the staged table is that function.
            source_kind = display::RasterSource::translated;
            source_bits = source.original;
            translated_format = engine_format;
        } else {
            source_kind = display::RasterSource::unsupported;
        }
    }

    const bool masked = mask != nullptr;
    const bool mask_mono = masked && engine_bitmap1(mask) && mask_point != nullptr;
    const bool solid = brush != nullptr && brush->iSolidColor != display::no_solid_color;
    const RealizedBrush* tile = display::rop4_uses_pattern(rop4) && !solid ? realized_tile(brush) : nullptr;
    const uint32_t code = tile != nullptr ? abi::ioctl_pattern : abi::ioctl_rop;
    if (!raster_available(device, code) ||
        !display::compute_rop_admitted(rop4, masked, mask_mono, target_kind, source_kind, solid, tile != nullptr) ||
        (uses_source && source_point == nullptr)) {
        return Attempt::unsupported;
    }

    const Rectangle bounds = value(rectangle);
    const int64_t width = static_cast<int64_t>(bounds.right) - bounds.left;
    const int64_t height = static_cast<int64_t>(bounds.bottom) - bounds.top;
    if (width <= 0 || height <= 0 || width > abi::max_surface_dimension || height > abi::max_surface_dimension ||
        bounds.left < -abi::max_coordinate || bounds.top < -abi::max_coordinate ||
        bounds.right > abi::max_coordinate || bounds.bottom > abi::max_coordinate) {
        return Attempt::unsupported;
    }

    // Every source and mask pixel the operation reads lies inside its bitmap
    // (the engine's DIB code reads them unchecked); a same-surface GPU source
    // must not meet any actual clipped destination pixel (the compute engine
    // has no copy order).
    const int64_t source_x = uses_source ? static_cast<int64_t>(source_point->x) : 0;
    const int64_t source_y = uses_source ? static_cast<int64_t>(source_point->y) : 0;
    if (uses_source) {
        uint32_t cx = 0;
        uint32_t cy = 0;
        operand_size(device, source, cx, cy);
        if (!block_inside(source_x, source_y, width, height, cx, cy)) {
            return Attempt::unsupported;
        }

        const bool same = source_kind == display::RasterSource::gpu &&
            (target.primary ? source.primary : target.bitmap == source.bitmap);
        if (same) {
            uint32_t target_width = 0;
            uint32_t target_height = 0;
            gpu_size(device, target, target_width, target_height);
            if (!clip_begin(rectangle, clip, target_width, target_height, CD_ANY, device.clip)) {
                return Attempt::unsupported;
            }

            const Rectangle reads{static_cast<int32_t>(source_x), static_cast<int32_t>(source_y),
                                  static_cast<int32_t>(source_x + width),
                                  static_cast<int32_t>(source_y + height)};
            while (clip_next(device.clip, device.batch)) {
                for (uint32_t index = 0; index < device.batch.count; ++index) {
                    const abi::Rect& written = device.batch.rects[index];
                    if (!policy::empty(policy::intersect(reads,
                        {written.left, written.top, written.right, written.bottom}))) {
                        return Attempt::unsupported;
                    }
                }
            }

            if (device.clip.failed) {
                return Attempt::unsupported;
            }
        }
    }

    if (mask_mono && !block_inside(mask_point->x, mask_point->y, width, height, mask->sizlBitmap.cx,
                                   mask->sizlBitmap.cy)) {
        return Attempt::unsupported;
    }

    const bool staged_source = source_kind == display::RasterSource::bitmap32 ||
                               source_kind == display::RasterSource::mono ||
                               source_kind == display::RasterSource::translated;
    // A translated source stages its XLATEOBJ table at the start of every band.
    const uint32_t table_bytes = abi::staging_table_bytes(translated_format);
    const uint32_t whole_half = display::staging_half_bytes(device.staging_bytes);
    if (table_bytes >= whole_half) {
        return Attempt::unsupported;
    }

    const uint32_t half_bytes = whole_half - table_bytes;
    if (++device.stretch_serial == 0) {
        device.stretch_serial = 1;
        device.table_serial[0] = device.table_serial[1] = 0;
    }
    // Column strips when even one full-width row of staged source and mask
    // exceeds a staging half: every strip is an independent request with its
    // own source, mask and destination columns (no pixel depends on another).
    const uint32_t strip_columns = display::staged_strip_columns(staged_source, mask_mono,
                                                                 static_cast<uint32_t>(width), half_bytes);
    if (strip_columns == 0) {
        return Attempt::unsupported;
    }

    uint32_t target_width = 0;
    uint32_t target_height = 0;
    gpu_size(device, target, target_width, target_height);
    const bool idempotent = !display::rop4_uses_destination(rop4);
    bool started = false;
    for (int64_t left = bounds.left; left < bounds.right; left += strip_columns) {
        const int64_t right = left + strip_columns < bounds.right ? left + strip_columns : bounds.right;
        const uint32_t w = static_cast<uint32_t>(right - left);
        const uint32_t band_rows = display::staged_rows(staged_source ? w : 0, mask_mono ? w : 0, half_bytes);
        for (int64_t top = bounds.top; top < bounds.bottom; top += band_rows) {
            const int64_t bottom = top + band_rows < bounds.bottom ? top + band_rows : bounds.bottom;
            const uint32_t rows = static_cast<uint32_t>(bottom - top);
            const RECTL band{static_cast<LONG>(left), static_cast<LONG>(top), static_cast<LONG>(right),
                             static_cast<LONG>(bottom)};
            abi::RopCommand& command = device.rop;
            command = {};
            command.session = device.session;
            command.target = gpu_handle(device, target);
            command.rop4 = rop4;
            command.brush_colour = solid ? brush->iSolidColor : 0;
            command.brush_x = brush_origin != nullptr ? brush_origin->x : 0;
            command.brush_y = brush_origin != nullptr ? brush_origin->y : 0;
            command.mono_foreground = mono_foreground;
            command.mono_background = mono_background;
            command.bounds = {band.left, band.top, band.right, band.bottom};
            command.map = {0, 0, 1, 1};
            command.has_pattern = tile != nullptr;
            for (uint32_t index = 0; tile != nullptr && index < 64; ++index) {
                command.pattern[index] = tile->tile[index];
            }

            uint32_t half = 0;
            uint32_t offset = 0;
            const bool staging = staged_source || mask_mono;
            if (staging && !acquire_half(device, half, offset)) {
                return Attempt::failed;
            }

            const int32_t sx = static_cast<int32_t>(source_x + (left - bounds.left));
            const int32_t sy = static_cast<int32_t>(source_y + (top - bounds.top));
            uint32_t cursor = offset;
            if (source_kind == display::RasterSource::gpu) {
                command.source = surface_operand(device, source);
                command.map = {sx, sy, static_cast<int32_t>(w), static_cast<int32_t>(rows)};
            } else if (source_kind == display::RasterSource::translated) {
                stage_translated_block(device, source_bits, translation, translated_format, sx, sy, w, rows, half,
                                       cursor);
                command.source = staging_operand(cursor, w, rows, translated_format);
                command.source.pitch = static_cast<uint32_t>(display::staged_pitch(translated_format, w));
                command.map = {0, 0, static_cast<int32_t>(w), static_cast<int32_t>(rows)};
                cursor += table_bytes + command.source.pitch * rows;
            } else if (staged_source) {
                const bool mono = source_kind == display::RasterSource::mono;
                if (mono) {
                    stage_mono(device, source_bits, sx, sy, w, rows, cursor);
                } else {
                    stage_bitmap32(device, source_bits, sx, sy, w, rows, cursor);
                }

                command.source = staging_operand(cursor, w, rows, mono ? abi::format_mono1 : abi::format_xrgb8888);
                command.map = {0, 0, static_cast<int32_t>(w), static_cast<int32_t>(rows)};
                cursor += w * abi::bytes_per_pixel * rows;
            }

            if (mask_mono) {
                stage_mono(device, mask, static_cast<int32_t>(mask_point->x + (left - bounds.left)),
                           static_cast<int32_t>(mask_point->y + (top - bounds.top)), w, rows, cursor);
                command.mask = staging_operand(cursor, w, rows, abi::format_mono1);
            }

            if (!clip_begin(band, clip, target_width, target_height, CD_ANY, device.clip)) {
                return started && !idempotent ? Attempt::failed : Attempt::unsupported;
            }

            const Attempt result = submit_rop_batches(device, code);
            if (staging) {
                device.half_fence[half] = device.last_fence;
            }

            if (result != Attempt::performed) {
                return started && !idempotent && untouched(result) ? Attempt::failed : result;
            }

            started = true;
        }
    }

    display::count(device.statistics.gpu_raster_rops);
    count_offscreen(device, target, source);
    return Attempt::performed;
}

// One single-rectangle stretch/colour-key/alpha request with a source that is a
// GPU surface or a staged sub-image of a 32bpp DIB. combined: the destination
// rectangle (also the map origin); input: the engine's source rectangle.
// clip: the destination rectangles (inside combined and the target), or null
// for combined itself clipped to the target.
Attempt mapped_request(Pdev& device, uint32_t code, const Operand& target, const Operand& source,
                       const Rectangle& combined, const Rectangle& input, uint32_t flags, uint32_t key,
                       uint32_t constant_alpha, bool source_alpha, const abi::RectList* clip) noexcept {
    const int64_t map_width = static_cast<int64_t>(input.right) - input.left;
    const int64_t map_height = static_cast<int64_t>(input.bottom) - input.top;
    uint32_t target_width = 0;
    uint32_t target_height = 0;
    gpu_size(device, target, target_width, target_height);
    const Rectangle visible = policy::intersect(combined, {0, 0, static_cast<int32_t>(target_width),
                                                           static_cast<int32_t>(target_height)});
    if (map_width <= 0 || map_height <= 0 || map_width > abi::max_surface_dimension ||
        map_height > abi::max_surface_dimension || policy::empty(visible) ||
        static_cast<int64_t>(combined.right) - combined.left > abi::max_surface_dimension ||
        static_cast<int64_t>(combined.bottom) - combined.top > abi::max_surface_dimension) {
        return Attempt::unsupported;
    }

    abi::Operand operand{};
    abi::SourceMap map{input.left, input.top, static_cast<int32_t>(map_width), static_cast<int32_t>(map_height)};
    uint32_t half = 0;
    bool staged = false;
    if (gpu_ready(device, source)) {
        operand = surface_operand(device, source);
    } else {
        // Stage only the source pixels the map reads that exist; pixels mapped
        // outside the bitmap stay outside the staged image too.
        SURFOBJ* bits = dib_of(source);
        Rectangle span{};
        uint32_t offset = 0;
        if (bits == nullptr || !engine_bitmap32(bits)) {
            return Attempt::unsupported;
        }

        if (!display::mapped_span(input.left, input.top, map_width, map_height,
                                  static_cast<uint32_t>(bits->sizlBitmap.cx),
                                  static_cast<uint32_t>(bits->sizlBitmap.cy), span)) {
            return Attempt::performed;   // nothing inside the source: nothing is written
        }

        const uint32_t w = static_cast<uint32_t>(span.right - span.left);
        const uint32_t h = static_cast<uint32_t>(span.bottom - span.top);
        if (static_cast<uint64_t>(w) * h * abi::bytes_per_pixel > display::staging_half_bytes(device.staging_bytes)) {
            return Attempt::unsupported;
        }

        if (!acquire_half(device, half, offset)) {
            return Attempt::failed;
        }

        stage_bitmap32(device, bits, span.left, span.top, w, h, offset);
        operand = staging_operand(offset, w, h, abi::format_xrgb8888);
        map.left -= span.left;
        map.top -= span.top;
        staged = true;
    }

    // device.moved is free here (callers pass device.batch or no list).
    const abi::RectList* list = clip;
    if (list == nullptr) {
        device.moved = {};
        device.moved.count = 1;
        device.moved.rects[0] = wire(visible);
        list = &device.moved;
    }

    const abi::RectList& rects = *list;

    uint64_t fence = 0;
    Attempt result = Attempt::unsupported;
    if (code == abi::ioctl_alpha) {
        abi::AlphaCommand& command = device.alpha;
        command = {};
        command.session = device.session;
        command.target = gpu_handle(device, target);
        command.constant_alpha = constant_alpha;
        command.flags = source_alpha ? abi::alpha_flag_source_alpha : 0;
        command.bounds = wire(combined);
        command.source = operand;
        command.map = map;
        command.rects = rects;
        if (abi::encode_alpha(CommandBuffer(device.request, abi::alpha_request_bytes), command)) {
            result = submit(device, code, abi::alpha_request_bytes, fence);
        }
    } else {
        abi::RopCommand& command = device.rop;
        command = {};
        command.session = device.session;
        command.target = gpu_handle(device, target);
        command.rop4 = 0xccccU;
        command.flags = flags;
        command.colour_key = key;
        command.bounds = wire(combined);
        command.source = operand;
        command.map = map;
        command.rects = rects;
        if (abi::encode_rop(CommandBuffer(device.request, abi::rop_request_bytes), command)) {
            result = submit(device, code, abi::rop_request_bytes, fence);
        }
    }

    if (staged) {
        device.half_fence[half] = fence != 0 ? fence : device.half_fence[half];
    }

    return result;
}

// Iterates the clip rectangles of an engine-style per-rectangle remap: trivial
// clips yield the operation itself, DC_RECT the bounds, DC_COMPLEX the
// enumeration (CD_ANY). visit(rect) returns the Attempt of that rectangle.
template<typename Visit>
Attempt for_each_clip_rect(CLIPOBJ* clip, ClipCursor& cursor, const Rectangle& operation, Visit visit) noexcept {
    const BYTE complexity = clip == nullptr ? DC_TRIVIAL : clip->iDComplexity;
    if (complexity == DC_TRIVIAL) {
        return visit(operation, true);
    }

    if (complexity == DC_RECT) {
        return visit(value(clip->rclBounds), false);
    }

    if (complexity != DC_COMPLEX) {
        return Attempt::unsupported;
    }

    CLIPOBJ_cEnumStart(clip, FALSE, CT_RECTANGLES, CD_ANY, 0);
    cursor.more = true;
    while (cursor.more) {
        cursor.enumeration.count = 0;
        cursor.more = CLIPOBJ_bEnum(clip, sizeof(cursor.enumeration),
                                    reinterpret_cast<ULONG*>(&cursor.enumeration)) != FALSE;
        if (cursor.enumeration.count > abi::max_rects || (cursor.more && cursor.enumeration.count == 0)) {
            return Attempt::unsupported;
        }

        for (ULONG index = 0; index < cursor.enumeration.count; ++index) {
            const Attempt result = visit(value(cursor.enumeration.rectangles[index]), false);
            if (result != Attempt::performed) {
                return result;
            }
        }
    }

    return Attempt::performed;
}

bool well_ordered(const RECTL& rectangle) noexcept {
    return rectangle.left < rectangle.right && rectangle.top < rectangle.bottom;
}

// DrvTransparentBlt: EngTransparentBlt's rectangle arithmetic per clip
// rectangle, SRCCOPY with the 24-bit colour key, dropping lanes outside the
// source (DIB_32BPP_TransparentBlt). Idempotent (writes only source pixels).
Attempt accelerate_transparent(Pdev& device, const Operand& target, const Operand& source, CLIPOBJ* clip,
                               XLATEOBJ* translation, const RECTL& destination, const RECTL& source_rect,
                               ULONG key) noexcept {
    const display::SurfaceKind source_kind = kind_with(source, device, translation, false);
    if (!raster_available(device, abi::ioctl_stretch) || !gpu_ready(device, target) ||
        (!(gpu_ready(device, source) && display::gpu_surface(source_kind)) &&
         source_kind != display::SurfaceKind::bitmap32)) {
        return Attempt::unsupported;
    }

    display::StretchSetup setup{};
    if (!display::transparent_setup(value(destination), value(source_rect), clip != nullptr,
                                    clip != nullptr ? value(clip->rclBounds) : Rectangle{}, setup)) {
        return Attempt::performed;   // the engine draws nothing either
    }

    const bool same = target.primary ? source.primary : (target.bitmap != nullptr && target.bitmap == source.bitmap);
    // The nearest-stretch kernel has no intra-dispatch copy order. A source
    // on the same GPU surface is safe when its complete sampled rectangle is
    // disjoint from the destination; otherwise the engine owns the snapshot.
    if (same && !policy::empty(policy::intersect(value(destination), value(source_rect)))) {
        return Attempt::unsupported;
    }

    const uint32_t flags = abi::rop_flag_colour_key | abi::rop_flag_skip_outside_source;
    const Attempt result = for_each_clip_rect(clip, device.clip, setup.output,
        [&](const Rectangle& rect, bool trivial) noexcept {
            Rectangle combined = setup.output;
            Rectangle input = setup.input;
            if (!trivial && !display::transparent_rect(setup, rect, combined, input)) {
                return Attempt::performed;
            }

            return mapped_request(device, abi::ioctl_stretch, target, source, combined, input, flags, key, 0, false,
                                  nullptr);
        });
    if (result == Attempt::performed) {
        display::count(device.statistics.gpu_transparent);
        count_offscreen(device, target, source);
    }

    return result;
}

// DrvAlphaBlend: EngAlphaBlend's validation and per-rectangle source remap.
// Not idempotent; every rectangle is admitted in a first pass (geometry and
// staging size) before the second pass submits.
Attempt accelerate_alpha(Pdev& device, const Operand& target, const Operand& source, CLIPOBJ* clip,
                         XLATEOBJ* translation, const RECTL& destination, const RECTL& source_rect,
                         const BLENDOBJ* blend) noexcept {
    const display::SurfaceKind source_kind = kind_with(source, device, translation, false);
    if (blend == nullptr || !raster_available(device, abi::ioctl_alpha) || !gpu_ready(device, target) ||
        (!(gpu_ready(device, source) && display::gpu_surface(source_kind)) &&
         source_kind != display::SurfaceKind::bitmap32)) {
        return Attempt::unsupported;
    }

    const BLENDFUNCTION function = blend->BlendFunction;
    Rectangle output = value(destination);
    Rectangle input = value(source_rect);
    output = {output.left < output.right ? output.left : output.right, output.top < output.bottom ? output.top : output.bottom,
              output.left < output.right ? output.right : output.left, output.top < output.bottom ? output.bottom : output.top};
    input = {input.left < input.right ? input.left : input.right, input.top < input.bottom ? input.top : input.bottom,
             input.left < input.right ? input.right : input.left, input.top < input.bottom ? input.bottom : input.top};
    uint32_t source_width = 0;
    uint32_t source_height = 0;
    operand_size(device, source, source_width, source_height);
    const bool same = target.primary ? source.primary : (target.bitmap != nullptr && target.bitmap == source.bitmap);
    // The engine refuses these (and the fallback reproduces the refusal).
    if (function.BlendOp != AC_SRC_OVER || function.BlendFlags != 0 ||
        (function.AlphaFormat & ~AC_SRC_ALPHA) != 0 || input.left < 0 || input.top < 0 ||
        input.right > static_cast<int32_t>(source_width) || input.bottom > static_cast<int32_t>(source_height) ||
        same || policy::empty(output) || policy::empty(input)) {
        return Attempt::unsupported;
    }

    if (clip != nullptr && (clip->rclBounds.left >= clip->rclBounds.right ||
                            clip->rclBounds.top >= clip->rclBounds.bottom)) {
        return Attempt::performed;
    }

    for (uint32_t pass = 0; pass < 2; ++pass) {
        const Attempt result = for_each_clip_rect(clip, device.clip, output,
            [&](const Rectangle& rect, bool trivial) noexcept {
                Rectangle combined = output;
                Rectangle mapped = input;
                if (!trivial && !display::alpha_rect(output, input, rect, combined, mapped)) {
                    return Attempt::performed;
                }

                if (pass == 0) {
                    const int64_t w = static_cast<int64_t>(mapped.right) - mapped.left;
                    const int64_t h = static_cast<int64_t>(mapped.bottom) - mapped.top;
                    return w > 0 && h > 0 && w <= abi::max_surface_dimension && h <= abi::max_surface_dimension &&
                           (gpu_ready(device, source) ||
                            static_cast<uint64_t>(w) * h * abi::bytes_per_pixel <=
                                display::staging_half_bytes(device.staging_bytes)) ?
                        Attempt::performed : Attempt::unsupported;
                }

                const Attempt one = mapped_request(device, abi::ioctl_alpha, target, source, combined, mapped, 0, 0,
                                                   function.SourceConstantAlpha,
                                                   (function.AlphaFormat & AC_SRC_ALPHA) != 0, nullptr);
                return untouched(one) ? Attempt::failed : one;
            });
        if (result != Attempt::performed) {
            return pass == 0 ? Attempt::unsupported : result;
        }
    }

    display::count(device.statistics.gpu_alpha);
    count_offscreen(device, target, source);
    return Attempt::performed;
}

// XLATEOBJ of a gradient: RGB() values to this BGRX device format.
bool bgrx_translation(XLATEOBJ* translation) noexcept {
    const ULONG samples[] = {0x000000ffU, 0x0000ff00U, 0x00ff0000U, 0x00123456U, 0x00000000U};
    for (const ULONG sample : samples) {
        const ULONG expected = ((sample & 0xffU) << 16) | (sample & 0xff00U) | ((sample >> 16) & 0xffU);
        if (XLATEOBJ_iXlate(translation, sample) != expected) {
            return false;
        }
    }

    return true;
}

// FreeType's source-pinned 8-bpp AlphaBltMask path: a single coverage blend
// over one GPU target. Unlike source-copy operations, a second blend after a
// partial batch would change the pixels, so only a complete single request
// is admitted. The win32k caller uses kernel-only DrvNotify after a read-only
// identity query and keeps all borrowed GDI objects alive through this call.
Attempt accelerate_glyph(Pdev& device, const Operand& target,
                         const rx7900::reactos::radeon_text::GlyphRequest& glyph) noexcept {
    SURFOBJ* mask = glyph.mask;
    if (!raster_available(device, abi::ioctl_text) || !gpu_ready(device, target) ||
        !engine_bitmap8(mask) || glyph.brush == nullptr ||
        glyph.rgb_to_device == nullptr || glyph.device_to_rgb == nullptr ||
        (glyph.rgb_to_device->iSrcType & PAL_RGB) == 0 ||
        (glyph.rgb_to_device->iDstType & PAL_BITFIELDS) == 0 ||
        (glyph.device_to_rgb->iSrcType & PAL_BITFIELDS) == 0 ||
        (glyph.device_to_rgb->iDstType & PAL_RGB) == 0 ||
        ((glyph.rgb_to_device->flXlate | glyph.device_to_rgb->flXlate) &
         (XO_TABLE | XO_TO_MONO | XO_FROM_CMYK | XO_DEVICE_ICM | XO_HOST_ICM)) != 0 ||
        (glyph.brush->iSolidColor & 0xff000000U) != 0 ||
        !bgrx_translation(glyph.rgb_to_device) || !bgrx_translation(glyph.device_to_rgb) ||
        (glyph.clip != nullptr && glyph.clip->iDComplexity == DC_COMPLEX)) {
        return Attempt::unsupported;
    }
    const ULONG colour = glyph.brush->iSolidColor;
    const ULONG rgb = ((colour & 0xffU) << 16) | (colour & 0xff00U) | ((colour >> 16) & 0xffU);
    if (XLATEOBJ_iXlate(glyph.device_to_rgb, colour) != rgb ||
        XLATEOBJ_iXlate(glyph.rgb_to_device, rgb) != colour) {
        return Attempt::unsupported;
    }

    uint32_t width = 0;
    uint32_t height = 0;
    gpu_size(device, target, width, height);
    const Rectangle requested = value(glyph.destination);
    if (requested.right <= requested.left || requested.bottom <= requested.top) {
        return Attempt::performed;
    }
    Rectangle visible = policy::clip(requested, static_cast<int32_t>(width), static_cast<int32_t>(height));
    if (glyph.clip != nullptr && glyph.clip->iDComplexity != DC_TRIVIAL) {
        visible = policy::intersect(visible, value(glyph.clip->rclBounds));
    }
    if (policy::empty(visible)) {
        return Attempt::performed;
    }
    const int64_t sx = static_cast<int64_t>(glyph.mask_origin.x) + visible.left - requested.left;
    const int64_t sy = static_cast<int64_t>(glyph.mask_origin.y) + visible.top - requested.top;
    const uint32_t w = static_cast<uint32_t>(visible.right - visible.left);
    const uint32_t h = static_cast<uint32_t>(visible.bottom - visible.top);
    if (!block_inside(sx, sy, w, h, mask->sizlBitmap.cx, mask->sizlBitmap.cy) ||
        !display::fits_half(w, h, display::staging_half_bytes(device.staging_bytes)) ||
        !clip_begin({visible.left, visible.top, visible.right, visible.bottom}, glyph.clip,
                    width, height, CD_ANY, device.clip)) {
        return Attempt::unsupported;
    }
    if (!clip_next(device.clip, device.batch) || device.clip.failed || device.batch.count != 1) {
        return Attempt::unsupported;
    }

    uint32_t half = 0;
    uint32_t offset = 0;
    if (!acquire_half(device, half, offset)) {
        return Attempt::failed;
    }
    stage_coverage8(device, mask, static_cast<int32_t>(sx), static_cast<int32_t>(sy), w, h, offset);
    abi::TextCommand& command = device.text;
    command = {};
    command.session = device.session;
    command.target = gpu_handle(device, target);
    command.mode = abi::text_mode_coverage8;
    command.colour = glyph.brush->iSolidColor;
    command.bounds = wire(visible);
    command.mask = staging_operand(offset, w, h, abi::format_coverage8);
    command.rects = device.batch;
    uint64_t fence = 0;
    if (!abi::encode_text(CommandBuffer(device.request, abi::text_request_bytes), command)) {
        return Attempt::unsupported;
    }
    const Attempt result = submit(device, abi::ioctl_text, abi::text_request_bytes, fence);
    if (result == Attempt::performed) {
        device.half_fence[half] = fence;
        count_offscreen(device, target, Operand{});
    }
    return result;
}

abi::Vertex vertex_of(const TRIVERTEX& vertex, LONG dx, LONG dy) noexcept {
    return {vertex.x + dx, vertex.y + dy, vertex.Red, vertex.Green, vertex.Blue};
}

bool vertex_in_range(const TRIVERTEX& vertex, LONG dx, LONG dy) noexcept {
    const int64_t x = static_cast<int64_t>(vertex.x) + dx;
    const int64_t y = static_cast<int64_t>(vertex.y) + dy;
    return x >= -abi::max_coordinate && x <= abi::max_coordinate && y >= -abi::max_coordinate &&
           y <= abi::max_coordinate;
}

// DrvGradientFill: one request per mesh element and clip batch, in mesh
// order. Gradients overwrite their pixels, so a later decline falls back to
// the whole engine mesh exactly.
Attempt accelerate_gradient(Pdev& device, const Operand& target, CLIPOBJ* clip, XLATEOBJ* translation,
                            const TRIVERTEX* vertices, ULONG vertex_count, const void* mesh, ULONG mesh_count,
                            const RECTL* extents, const POINTL* dither, ULONG mode) noexcept {
    uint32_t width = 0;
    uint32_t height = 0;
    if (!raster_available(device, abi::ioctl_gradient) || !gpu_ready(device, target) || vertices == nullptr ||
        mesh == nullptr || extents == nullptr || !bgrx_translation(translation) ||
        (mode != GRADIENT_FILL_RECT_H && mode != GRADIENT_FILL_RECT_V && mode != GRADIENT_FILL_TRIANGLE)) {
        return Attempt::unsupported;
    }

    gpu_size(device, target, width, height);
    const LONG dx = mode == GRADIENT_FILL_TRIANGLE && dither != nullptr ? dither->x : 0;
    const LONG dy = mode == GRADIENT_FILL_TRIANGLE && dither != nullptr ? dither->y : 0;
    // Admission of the whole mesh before anything is submitted.
    for (ULONG index = 0; index < mesh_count; ++index) {
        if (mode == GRADIENT_FILL_TRIANGLE) {
            // GDI owns mesh_count GRADIENT_TRIANGLE entries for this call.
            const GRADIENT_TRIANGLE& triangle = static_cast<const GRADIENT_TRIANGLE*>(mesh)[index];
            if (triangle.Vertex1 >= vertex_count || triangle.Vertex2 >= vertex_count ||
                triangle.Vertex3 >= vertex_count) {
                return Attempt::unsupported;
            }

            const TRIVERTEX& a = vertices[triangle.Vertex1];
            const TRIVERTEX& b = vertices[triangle.Vertex2];
            const TRIVERTEX& c = vertices[triangle.Vertex3];
            // Shaded and single-colour triangles of any height: the backend
            // plans rows in chunks and applies the matching engine span rules.
            if (!vertex_in_range(a, dx, dy) || !vertex_in_range(b, dx, dy) || !vertex_in_range(c, dx, dy)) {
                return Attempt::unsupported;
            }
        } else {
            const GRADIENT_RECT& rect = static_cast<const GRADIENT_RECT*>(mesh)[index];
            if (rect.UpperLeft >= vertex_count || rect.LowerRight >= vertex_count ||
                !vertex_in_range(vertices[rect.UpperLeft], 0, 0) || !vertex_in_range(vertices[rect.LowerRight], 0, 0)) {
                return Attempt::unsupported;
            }
        }
    }

    for (ULONG index = 0; index < mesh_count; ++index) {
        abi::GradientCommand& command = device.gradient;
        command = {};
        command.session = device.session;
        command.target = gpu_handle(device, target);
        RECTL requested{};
        if (mode == GRADIENT_FILL_TRIANGLE) {
            const GRADIENT_TRIANGLE& triangle = static_cast<const GRADIENT_TRIANGLE*>(mesh)[index];
            command.mode = abi::gradient_triangle;
            command.vertices[0] = vertex_of(vertices[triangle.Vertex1], dx, dy);
            command.vertices[1] = vertex_of(vertices[triangle.Vertex2], dx, dy);
            command.vertices[2] = vertex_of(vertices[triangle.Vertex3], dx, dy);
            if (display::null_triangle(command.vertices[0], command.vertices[1], command.vertices[2])) {
                continue;
            }

            command.extents = {extents->left, extents->top, extents->right, extents->bottom};
            if (!abi::detail::valid_bounds(command.extents)) {
                return Attempt::unsupported;
            }

            requested = *extents;
        } else {
            const GRADIENT_RECT& rect = static_cast<const GRADIENT_RECT*>(mesh)[index];
            const TRIVERTEX& upper = vertices[rect.UpperLeft];
            const TRIVERTEX& lower = vertices[rect.LowerRight];
            command.mode = mode == GRADIENT_FILL_RECT_H ? abi::gradient_rect_h : abi::gradient_rect_v;
            command.vertices[0] = vertex_of(upper, 0, 0);
            command.vertices[1] = vertex_of(lower, 0, 0);
            requested = {upper.x < lower.x ? upper.x : lower.x, upper.y < lower.y ? upper.y : lower.y,
                         upper.x < lower.x ? lower.x : upper.x, upper.y < lower.y ? lower.y : upper.y};
            if (requested.left == requested.right || requested.top == requested.bottom) {
                continue;   // rcSG is empty: nothing intersects it
            }
        }

        if (!clip_begin(requested, clip, width, height, CD_ANY, device.clip)) {
            return Attempt::unsupported;
        }

        const Attempt result = for_each_batch(device, [&](const abi::RectList& batch) noexcept {
            command.rects = batch;
            uint64_t fence = 0;
            return abi::encode_gradient(CommandBuffer(device.request, abi::gradient_request_bytes), command) ?
                submit(device, abi::ioctl_gradient, abi::gradient_request_bytes, fence) : Attempt::unsupported;
        });
        if (result != Attempt::performed) {
            return result;
        }
    }

    display::count(device.statistics.gpu_gradients);
    count_offscreen(device, target, Operand{});
    return Attempt::performed;
}

// DrvLineTo for a paired-kernel-tagged unstyled R2_COPYPEN cosmetic line: the
// eng/lineto.c pixel set (start included, end excluded) of the pen's
// iSolidColor in every clip rectangle, for solid and pattern pens alike (the
// pinned EngLineTo never samples a pattern brush). Idempotent (PATCOPY).
Attempt accelerate_line(Pdev& device, const Operand& target, CLIPOBJ* clip, BRUSHOBJ* brush, LONG x1, LONG y1,
                        LONG x2, LONG y2, const RECTL* bounds, const uint32_t* runs = nullptr,
                        uint32_t run_count = 0) noexcept {
    uint32_t width = 0;
    uint32_t height = 0;
    if (!raster_available(device, abi::ioctl_line) || !gpu_ready(device, target) || brush == nullptr ||
        x1 < -abi::max_coordinate || x1 > abi::max_coordinate || y1 < -abi::max_coordinate ||
        y1 > abi::max_coordinate || x2 < -abi::max_coordinate || x2 > abi::max_coordinate ||
        y2 < -abi::max_coordinate || y2 > abi::max_coordinate) {
        return Attempt::unsupported;
    }

    if (x1 == x2 && y1 == y2) {
        return Attempt::performed;   // EngLineTo draws nothing
    }
    // Styled EngLineTo advances iStyle/lStyleMax once per octant-loop step and
    // never from the clip walk; a step is plotted when the current (banded)
    // clip rectangle contains it, which is exactly when any clip rectangle does,
    // and steps after the walk runs out of rectangles lie outside all of them.
    // So the step-k style bit is clip independent and every clip batch replays
    // the same stream (DC_COMPLEX included).
    gpu_size(device, target, width, height);
    // Every plotted pixel lies in [min, max] on both axes.
    RECTL requested{x1 < x2 ? x1 : x2, y1 < y2 ? y1 : y2, (x1 < x2 ? x2 : x1) + 1, (y1 < y2 ? y2 : y1) + 1};
    if (clip == nullptr && bounds != nullptr) {
        // EngLineTo without a clip object clips to RectBounds.
        const Rectangle limited = policy::intersect(value(requested), value(*bounds));
        requested = {limited.left, limited.top, limited.right, limited.bottom};
        if (policy::empty(limited)) {
            return Attempt::performed;
        }
    }

    abi::LineCommand& command = device.line;
    command = {};
    command.session = device.session;
    command.target = gpu_handle(device, target);
    // A pattern pen too: EngLineTo plots iSolidColor (0xffffffff) for it.
    command.colour = display::engine_line_colour(brush->iSolidColor);
    command.rop3 = 0xf0;
    command.x0 = x1;
    command.y0 = y1;
    command.x1 = x2;
    command.y1 = y2;
    if (run_count != 0) {
        if (!display::engine_style_bits(x1, y1, x2, y2, runs, run_count,
                                        command.style, command.style_bit_count)) {
            return Attempt::unsupported;
        }
        command.flags = abi::line_flag_styled;
    }
    if (!clip_begin(requested, clip, width, height, CD_ANY, device.clip)) {
        return Attempt::unsupported;
    }

    const Attempt result = for_each_batch(device, [&](const abi::RectList& batch) noexcept {
        command.rects = batch;
        uint64_t fence = 0;
        return abi::encode_line(CommandBuffer(device.request, abi::line_request_bytes), command) ?
            submit(device, abi::ioctl_line, abi::line_request_bytes, fence) : Attempt::unsupported;
    });
    if (result == Attempt::performed) {
        display::count(device.statistics.gpu_lines);
        count_offscreen(device, target, Operand{});
    }

    return result;
}

// DrvStretchBlt (NT-compatible engines; ReactOS dispatches only StretchBltROP):
// COLORONCOLOR, no mask, trivial XLATE, both rectangles well ordered (mirrors
// stay engine work), nearest mapping as dib/stretchblt.c. Idempotent.
Attempt accelerate_stretch(Pdev& device, const Operand& target, const Operand& source, SURFOBJ* mask, CLIPOBJ* clip,
                           XLATEOBJ* translation, const RECTL& destination, const RECTL& source_rect,
                           ULONG mode) noexcept {
    const display::SurfaceKind source_kind = kind_with(source, device, translation, false);
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t source_width = 0;
    uint32_t source_height = 0;
    operand_size(device, source, source_width, source_height);
    if (!raster_available(device, abi::ioctl_stretch) || mask != nullptr || mode != COLORONCOLOR ||
        !gpu_ready(device, target) || !well_ordered(destination) || !well_ordered(source_rect) ||
        (!(gpu_ready(device, source) && display::gpu_surface(source_kind)) &&
         source_kind != display::SurfaceKind::bitmap32) ||
        !block_inside(source_rect.left, source_rect.top, static_cast<int64_t>(source_rect.right) - source_rect.left,
                      static_cast<int64_t>(source_rect.bottom) - source_rect.top, source_width, source_height)) {
        return Attempt::unsupported;
    }

    const bool same = target.primary ? source.primary : (target.bitmap != nullptr && target.bitmap == source.bitmap);
    if (same) {
        return Attempt::unsupported;
    }

    gpu_size(device, target, width, height);
    const Rectangle output = value(destination);
    const Rectangle input = value(source_rect);
    if (!clip_begin(destination, clip, width, height, CD_ANY, device.clip)) {
        return Attempt::unsupported;
    }

    // One request per clip batch with the whole-operation map; the source is a
    // GPU surface or staged whole (it must fit one staging half).
    const Attempt result = for_each_batch(device, [&](const abi::RectList& batch) noexcept {
        return mapped_request(device, abi::ioctl_stretch, target, source, output, input,
                              abi::rop_flag_skip_outside_source, 0, 0, false, &batch);
    });
    if (result == Attempt::performed) {
        display::count(device.statistics.gpu_stretches);
        count_offscreen(device, target, source);
    }

    return result;
}


// ---------------------------------------------------------------------------
// Guarded DrvStretchBltROP (abi 0xa17): one request per destination row band of
// each DIB_XXBPP_StretchBlt call of the engine fallback (radeon_display.hpp
// stretch_rop_setup, stretch_rop_call and the shared band planner). A staged
// source or mask holds exactly the samples of its band that exist in the
// engine's sample domain. A device-managed GPU source is read live when its
// IntEngEnter window lies inside the surface and no DIB call of the same
// surface meets it; otherwise from a GPU snapshot of that window, which is the
// engine's own temporary: zeros outside the surface (GDIOBJ_AllocateObject
// zero-fills it), the surface pixels inside, taken before anything is drawn.
// ---------------------------------------------------------------------------

// The cached snapshot surface (a session offscreen surface owned by the PDEV).
void release_scratch(Pdev& device) noexcept {
    if (device.scratch_surface == 0) {
        return;
    }

    DeviceBitmap stray{};
    stray.surface = device.scratch_surface;
    stray.bytes = device.scratch_bytes;
    destroy_surface(device, stray);
    device.scratch_surface = 0;
    device.scratch_width = 0;
    device.scratch_height = 0;
    device.scratch_bytes = 0;
}

// A scratch surface of at least width x height, or 0 (budget, capability).
uint32_t acquire_scratch(Pdev& device, uint32_t width, uint32_t height) noexcept {
    if (device.scratch_surface != 0 && device.scratch_width >= width && device.scratch_height >= height) {
        return device.scratch_surface;
    }

    release_scratch(device);
    // Round up so a run of similar windows reuses one allocation.
    const uint32_t w = (width + 63U) & ~63U;
    const uint32_t h = (height + 63U) & ~63U;
    const uint32_t cw = w > abi::max_surface_dimension ? abi::max_surface_dimension : w;
    const uint32_t ch = h > abi::max_surface_dimension ? abi::max_surface_dimension : h;
    const uint64_t need = display::offscreen_bytes(cw, ch);
    const display::BitmapLimits& limits = device.bitmap_limits;
    if (!session_usable(device) || device.resident_count >= limits.max_count ||
        device.resident_bytes > limits.budget_bytes || need > limits.budget_bytes - device.resident_bytes) {
        return 0;
    }

    uint64_t bytes = 0;
    const uint32_t surface = create_surface(device, cw, ch, bytes);
    if (surface == 0) {
        return 0;
    }

    device.scratch_surface = surface;
    device.scratch_width = cw;
    device.scratch_height = ch;
    device.scratch_bytes = bytes;
    device.resident_bytes += bytes;
    ++device.resident_count;
    device.statistics.live_bitmap_bytes = device.resident_bytes;
    return surface;
}

// The engine's IntEngEnter temporary of the GPU source window domain, on the
// GPU: an SDMA fill of the window outside backed (the surface part) with 0 and
// an SDMA copy of backed. The backend orders both before the stretch reading
// the snapshot, and that stretch before the next fill reuses it. Nothing
// visible changes on a decline. scratch receives the handle.
Attempt snapshot_window(Pdev& device, const Operand& source, const Rectangle& domain, const Rectangle& backed,
                        uint32_t& scratch) noexcept {
    const int64_t width = static_cast<int64_t>(domain.right) - domain.left;
    const int64_t height = static_cast<int64_t>(domain.bottom) - domain.top;
    scratch = 0;
    if (width <= 0 || height <= 0 || width > abi::max_surface_dimension || height > abi::max_surface_dimension) {
        return Attempt::unsupported;
    }

    const uint32_t surface = acquire_scratch(device, static_cast<uint32_t>(width), static_cast<uint32_t>(height));
    if (surface == 0) {
        return Attempt::unsupported;
    }

    // Window minus backed, and backed, in snapshot coordinates.
    abi::RectList& zeros = device.moved;
    zeros = {};
    Rectangle parts[4]{};
    Rectangle inner{};
    zeros.count = display::snapshot_rects(domain, backed, parts, inner);
    for (uint32_t index = 0; index < zeros.count; ++index) {
        zeros.rects[index] = wire(parts[index]);
    }

    if (zeros.count != 0) {
        const Attempt filled = submit_fill(device, surface, 0, zeros);
        if (filled != Attempt::performed) {
            if (filled == Attempt::surface_lost) {
                release_scratch(device);
            }

            return filled == Attempt::failed ? Attempt::failed : Attempt::unsupported;
        }
    }

    if (!policy::empty(inner)) {
        abi::RectList& copied = device.band;
        copied = {};
        copied.count = 1;
        copied.rects[0] = wire(inner);
        const Attempt result = submit_copy(device, surface, gpu_handle(device, source), domain.left, domain.top, copied);
        if (result != Attempt::performed) {
            if (result == Attempt::surface_lost) {
                release_scratch(device);
            }

            return result == Attempt::failed ? Attempt::failed : Attempt::unsupported;
        }
    }

    scratch = surface;
    return Attempt::performed;
}

// The staged source pixels of the operation (engine DIB or demoted bitmap).
struct StretchPixels final {
    SURFOBJ* bits = nullptr;
    XLATEOBJ* translation = nullptr;
    bool mono = false;
};

// Stage one band into the half at base: the translation table (once per half
// and operation), source rows and mask rows.
void stage_stretch_band(Pdev& device, const display::StretchSourcePlan& plan, const StretchPixels& pixels,
                        SURFOBJ* mask, const display::StretchBand& band, uint32_t half, uint32_t base) noexcept {
    if (plan.operand == display::StretchOperand::staged) {
        const display::StretchStage& stage = band.source;
        const uint32_t table_bytes = abi::staging_table_bytes(plan.format);
        if (table_bytes != 0 && device.table_serial[half] != device.stretch_serial) {
            const uint32_t other = (half + 1U) % display::staging_halves;
            const uint32_t half_bytes = display::staging_half_bytes(device.staging_bytes);
            if (device.table_serial[other] == device.stretch_serial) {
                memcpy(device.staging + base + band.table_offset, device.staging + other * half_bytes, table_bytes);
            } else {
                XLATEOBJ* translation = pixels.translation;
                display::write_translation_table(plan.format, [translation](uint32_t value) noexcept {
                    return translation != nullptr ? static_cast<uint32_t>(XLATEOBJ_iXlate(translation, value)) : value;
                }, CommandBuffer(device.staging + base + band.table_offset, table_bytes));
            }

            device.table_serial[half] = device.stretch_serial;
        }

        uint8_t* rows = device.staging + base + stage.offset;
        if (pixels.mono && !stage.zero) {
            stage_mono(device, pixels.bits, static_cast<int32_t>(stage.x), static_cast<int32_t>(stage.y),
                       static_cast<uint32_t>(stage.width), static_cast<uint32_t>(stage.height), base + stage.offset);
        } else {
            const uint32_t pixel_bytes = display::staged_bytes_per_pixel(plan.format);
            SURFOBJ* bits = pixels.bits;
            for (int64_t row = 0; row < stage.height; ++row) {
                // GDI owns pvScan0; stage_source_row reads only backed rows, which lie
                // inside the bitmap; lDelta may be negative.
                display::stage_source_row(plan, stage, row, pixel_bytes, [bits](int32_t sy) noexcept {
                    return static_cast<const uint8_t*>(bits->pvScan0) + static_cast<intptr_t>(sy) * bits->lDelta;
                }, rows + static_cast<size_t>(row) * stage.pitch);
            }
        }
    }

    if (plan.mask_width != 0 && band.mask.height != 0) {
        const display::StretchStage& stage = band.mask;
        stage_mono(device, mask, static_cast<int32_t>(stage.x), static_cast<int32_t>(stage.y),
                   static_cast<uint32_t>(stage.width), static_cast<uint32_t>(stage.height), base + stage.offset);
    }
}

// The operation-wide state of one guarded stretch (device-owned workspace).
struct StretchOperation final {
    const Operand* target = nullptr;
    SURFOBJ* mask = nullptr;
    const RealizedBrush* tile = nullptr;
    BRUSHOBJ* brush = nullptr;
    const POINTL* brush_origin = nullptr;
    uint32_t rop4 = 0;
    bool solid = false;
    bool idempotent = false;
    bool same_surface = false;
    bool snapshot = false;
    bool started = false;
    uint32_t pass = 0;
    StretchPixels pixels{};
    uint32_t mono0 = 0;
    uint32_t mono1 = 0;
    Rectangle surface{};
    uint32_t half_bytes = 0;
};

// One 0xa17 request for band of call: command, staging and submission.
#if defined(_MSC_VER)
__declspec(noinline)
#elif defined(__GNUC__)
__attribute__((noinline))
#endif
Attempt submit_stretch_band(Pdev& device, StretchOperation& operation, const display::StretchCall& call,
                            const display::StretchBand& band) noexcept {
    const display::StretchSourcePlan& plan = device.stretch_plan;
    abi::StretchRopCommand& command = device.stretch_rop;
    const RealizedBrush* tile = operation.tile;
    command = {};
    command.session = device.session;
    command.target = gpu_handle(device, *operation.target);
    command.rop4 = operation.rop4;
    command.flags = tile != nullptr ? abi::stretch_flag_tile : 0U;
    command.brush_colour = tile == nullptr && operation.solid ? operation.brush->iSolidColor : 0U;
    command.brush_x = operation.brush_origin != nullptr ? operation.brush_origin->x : 0;
    command.brush_y = operation.brush_origin != nullptr ? operation.brush_origin->y : 0;
    for (uint32_t index = 0; tile != nullptr && index < 64; ++index) {
        command.pattern[index] = tile->tile[index];
    }

    display::stretch_band_command(device.stretch_setup, call, plan, band, command);
    if (operation.pixels.mono) {
        command.mono_foreground = operation.mono1;
        command.mono_background = operation.mono0;
    }

    uint32_t half = 0;
    uint32_t base = 0;
    const bool staging = band.bytes != 0;
    if (staging) {
        if (!acquire_half(device, half, base)) {
            return Attempt::failed;
        }

        stage_stretch_band(device, plan, operation.pixels, operation.mask, band, half, base);
        if (command.source.kind == abi::operand_staging) {
            command.source.handle += base;
        }

        if (command.mask.kind == abi::operand_staging) {
            command.mask.handle += base;
        }
    }

    uint64_t fence = 0;
    Attempt one = Attempt::unsupported;
    if (abi::encode_stretch_rop(CommandBuffer(device.request, abi::stretch_rop_request_bytes), command)) {
        one = submit(device, abi::ioctl_stretch_rop, abi::stretch_rop_request_bytes, fence);
    }

    if (staging) {
        device.half_fence[half] = fence != 0 ? fence : device.half_fence[half];
    }

    if (one != Attempt::performed) {
        return operation.started && !operation.idempotent && untouched(one) ? Attempt::failed : one;
    }

    operation.started = true;
    return Attempt::performed;
}

// One clip rectangle of the engine's per-rectangle remap: its DIB call, the
// same-surface snapshot decision (pass 0) and its bands. Its own stack frame
// keeps the band planner off the DDI-facing frame.
#if defined(_MSC_VER)
__declspec(noinline)
#elif defined(__GNUC__)
__attribute__((noinline))
#endif
Attempt stretch_rop_clip_rect(Pdev& device, StretchOperation& operation, const Rectangle& rect,
                              bool trivial_clip) noexcept {
    const display::StretchSourcePlan& plan = device.stretch_plan;
    display::StretchCall call{};
    if (!display::stretch_rop_call(device.stretch_setup, trivial_clip ? nullptr : &rect, call)) {
        return Attempt::performed;
    }

    const Rectangle visible = policy::intersect(call.destination, operation.surface);
    if (policy::empty(visible)) {
        return Attempt::performed;
    }

    // The engine snapshots a same-surface source before drawing; the dispatcher
    // refuses a live window meeting the request bounds.
    if (operation.pass == 0 && operation.same_surface && display::stretch_call_meets_window(plan, call)) {
        operation.snapshot = true;
    }

    Attempt band_result = Attempt::performed;
    const display::StretchPlanResult planned = display::stretch_call_bands(device.stretch_setup, call, plan, visible,
        operation.half_bytes, [&](const display::StretchBand& band) noexcept {
            if (operation.pass == 0) {
                return true;
            }

            band_result = submit_stretch_band(device, operation, call, band);
            return band_result == Attempt::performed;
        });
    if (planned == display::StretchPlanResult::unsupported) {
        return operation.started && !operation.idempotent ? Attempt::failed : Attempt::unsupported;
    }

    return planned == display::StretchPlanResult::stopped ? band_result : Attempt::performed;
}

// Keep the guarded stretch planner in its own kernel stack frame. GCC otherwise
// folds its two-pass planner into the DDI entry and triggers the AMD64 DDI
// frame warning; both supported compilers honor this boundary.
#if defined(_MSC_VER)
__declspec(noinline)
#elif defined(__GNUC__)
__attribute__((noinline))
#endif
Attempt accelerate_stretch_rop(Pdev& device, const Operand& target, const Operand& source, SURFOBJ* mask,
                               CLIPOBJ* clip, XLATEOBJ* translation, const POINTL* brush_origin,
                               const RECTL& output, const RECTL& input, BRUSHOBJ* brush, uint32_t rop4,
                               ULONG orientation) noexcept {
    const display::SurfaceKind target_kind = kind_with(target, device, nullptr, false);
    if (!raster_available(device, abi::ioctl_stretch_rop) || !gpu_ready(device, target) || rop4 > 0xffffU) {
        return Attempt::unsupported;
    }

    display::StretchRopSetup& setup = device.stretch_setup;
    const Rectangle clip_bounds = clip != nullptr ? value(clip->rclBounds) : Rectangle{};
    if (!display::stretch_rop_setup(value(output), value(input), (orientation & RX7900_STRETCH_FLIP_X) != 0,
                                    (orientation & RX7900_STRETCH_FLIP_Y) != 0, rop4,
                                    clip != nullptr ? &clip_bounds : nullptr, setup)) {
        return Attempt::performed;   // ROP4_NOOP or degenerate: the engine draws nothing
    }

    // Every product of the engine's LONG mapping stays below 2^31 (extents of at
    // most 16384 either sign, |coordinates| below 2^24).
    const auto coordinate = [](const Rectangle& r) noexcept {
        return r.left >= -abi::max_coordinate && r.top >= -abi::max_coordinate && r.right <= abi::max_coordinate &&
               r.bottom <= abi::max_coordinate;
    };
    const int64_t limit = abi::max_surface_dimension;
    if (!coordinate(setup.output) || setup.destination_width > limit || setup.destination_height > limit ||
        (setup.uses_source && (!coordinate(setup.input) || !coordinate(setup.domain) || setup.source_width < -limit ||
                               setup.source_height < -limit || setup.source_width > limit ||
                               setup.source_height > limit))) {
        return Attempt::unsupported;
    }

    // The source: a GPU surface, or staged pixels of a 32bpp DIB (an engine
    // bitmap read whole, or a demoted device bitmap through its IntEngEnter
    // window), a 1bpp DIB with its two XLATE colours or an 8/16/24bpp DIB with
    // its XLATEOBJ table.
    display::StretchSourcePlan& plan = device.stretch_plan;
    plan = {};
    StretchPixels pixels{};
    display::RasterSource kind = display::RasterSource::none;
    bool device_source = false;
    uint32_t mono0 = 0;
    uint32_t mono1 = 0;
    if (setup.uses_source) {
        SURFOBJ* original = source.original;
        const display::SurfaceKind source_kind = kind_with(source, device, translation, false);
        device_source = original != nullptr && original->iType != STYPE_BITMAP &&
            (original->pvScan0 == nullptr || original->lDelta == 0);
        const uint32_t translated_format = source.owner == nullptr ? translated_engine_format(original) : 0U;
        uint32_t cx = 0;
        uint32_t cy = 0;
        operand_size(device, source, cx, cy);
        kind = display::RasterSource::unsupported;
        if (original == nullptr) {
            kind = display::RasterSource::unsupported;
        } else if (display::gpu_surface(source_kind) && gpu_ready(device, source)) {
            kind = display::RasterSource::gpu;
            plan.operand = display::StretchOperand::surface;
            plan.surface = gpu_handle(device, source);
        } else if (source_kind == display::SurfaceKind::bitmap32) {
            kind = display::RasterSource::bitmap32;
            plan.operand = display::StretchOperand::staged;
            plan.format = abi::format_xrgb8888;
            pixels.bits = dib_of(source);
        } else if (source.owner == nullptr && engine_bitmap1(original) && translation != nullptr) {
            kind = display::RasterSource::mono;
            plan.operand = display::StretchOperand::staged;
            plan.format = abi::format_mono1;
            pixels.bits = original;
            pixels.mono = true;
        } else if (translated_format != 0 &&
                   (translated_format != abi::format_rgb24 || or_linear(translation)) &&
                   (translated_format != abi::format_rgb16 ||
                    setup.destination_width * setup.destination_height >= min_rgb16_destination_pixels)) {
            kind = display::RasterSource::translated;
            plan.operand = display::StretchOperand::staged;
            plan.format = translated_format;
            pixels.bits = original;
            pixels.translation = translation;
        }

        const Rectangle bitmap{0, 0, static_cast<int32_t>(cx), static_cast<int32_t>(cy)};
        plan.domain = device_source ? setup.domain : bitmap;
        plan.backed = policy::intersect(plan.domain, bitmap);
        if (device_source) {
            plan.mask_dx = setup.domain.left;
            plan.mask_dy = setup.domain.top;
        }

        if (policy::empty(plan.domain) || (kind != display::RasterSource::gpu && pixels.bits == nullptr)) {
            kind = display::RasterSource::unsupported;
        }

        if (kind == display::RasterSource::mono) {
            mono0 = XLATEOBJ_iXlate(translation, 0);
            mono1 = XLATEOBJ_iXlate(translation, 1);
        }
    }

    const bool masked = mask != nullptr;
    const bool mask_mono = masked && engine_bitmap1(mask);
    const bool solid = brush != nullptr && brush->iSolidColor != display::no_solid_color;
    const bool tile_needed = display::rop4_uses_pattern(rop4) && brush != nullptr && !solid;
    const RealizedBrush* tile = tile_needed ? realized_tile(brush) : nullptr;
    if (!display::stretch_rop_admitted(rop4, target_kind, kind, masked, mask_mono, tile_needed, tile != nullptr)) {
        return Attempt::unsupported;
    }

    if (mask_mono) {
        plan.mask_width = mask->sizlBitmap.cx;
        plan.mask_height = mask->sizlBitmap.cy;
        plan.running = tile != nullptr;
    }

    StretchOperation operation{};
    operation.target = &target;
    operation.mask = mask;
    operation.tile = tile;
    operation.brush = brush;
    operation.brush_origin = brush_origin;
    operation.rop4 = rop4;
    operation.solid = solid;
    operation.pixels = pixels;
    operation.mono0 = mono0;
    operation.mono1 = mono1;
    uint32_t target_width = 0;
    uint32_t target_height = 0;
    gpu_size(device, target, target_width, target_height);
    operation.surface = {0, 0, static_cast<int32_t>(target_width), static_cast<int32_t>(target_height)};
    operation.half_bytes = display::staging_half_bytes(device.staging_bytes);
    operation.same_surface = kind == display::RasterSource::gpu &&
        (target.primary ? source.primary : target.bitmap == source.bitmap);
    // A live GPU window must lie inside its surface (the dispatcher's rule).
    operation.snapshot = kind == display::RasterSource::gpu && !display::stretch_window_on_surface(plan);
    operation.idempotent = !display::rop3_uses_destination(rop4 & 0xffU);
    if (++device.stretch_serial == 0) {
        device.stretch_serial = 1;
        device.table_serial[0] = device.table_serial[1] = 0;
    }

    for (uint32_t pass = 0; pass < 2; ++pass) {
        if (pass == 1 && operation.snapshot) {
            uint32_t scratch = 0;
            const Attempt taken = snapshot_window(device, source, plan.domain, plan.backed, scratch);
            if (taken != Attempt::performed) {
                return taken;
            }

            plan.surface = scratch;
            plan.origin_x = plan.domain.left;
            plan.origin_y = plan.domain.top;
        }

        operation.pass = pass;
        const Attempt result = for_each_clip_rect(clip, device.clip, setup.output,
            [&device, &operation](const Rectangle& rect, bool trivial_clip) noexcept {
                return stretch_rop_clip_rect(device, operation, rect, trivial_clip);
            });
        if (result != Attempt::performed) {
            return pass == 0 && untouched(result) ? Attempt::unsupported : result;
        }
    }

    display::count(device.statistics.gpu_stretches);
    count_offscreen(device, target, source);
    return Attempt::performed;
}

// ---------------------------------------------------------------------------
// Mode enumeration through the standard VideoPort IOCTLs.
// ---------------------------------------------------------------------------
class ModeList final {
public:
    ModeList() noexcept = default;
    ModeList(const ModeList&) = delete;
    ModeList& operator=(const ModeList&) = delete;
    ~ModeList() {
        if (buffer_ != nullptr) {
            EngFreeMem(buffer_);
        }
    }

    bool load(HANDLE driver) noexcept {
        VIDEO_NUM_MODES modes{};
        DWORD returned = 0;
        if (EngDeviceIoControl(driver, IOCTL_VIDEO_QUERY_NUM_AVAIL_MODES, nullptr, 0, &modes,
                               sizeof(modes), &returned) != 0 || returned != sizeof(modes) ||
            modes.NumModes == 0 || modes.NumModes > 64 ||
            modes.ModeInformationLength < sizeof(VIDEO_MODE_INFORMATION) ||
            modes.ModeInformationLength > 4096) {
            return false;
        }

        const ULONG bytes = modes.NumModes * modes.ModeInformationLength;
        buffer_ = static_cast<uint8_t*>(EngAllocMem(FL_ZERO_MEMORY, bytes, allocation_tag));
        if (buffer_ == nullptr ||
            EngDeviceIoControl(driver, IOCTL_VIDEO_QUERY_AVAIL_MODES, nullptr, 0, buffer_, bytes,
                               &returned) != 0 || returned != bytes) {
            return false;
        }

        count_ = modes.NumModes;
        stride_ = modes.ModeInformationLength;
        return true;
    }

    uint32_t count() const noexcept { return count_; }

    // index < count(); copies the entry to support any buffer alignment.
    VIDEO_MODE_INFORMATION at(uint32_t index) const noexcept {
        VIDEO_MODE_INFORMATION mode{};
        memcpy(&mode, buffer_ + static_cast<size_t>(index) * stride_, sizeof(mode));
        return mode;
    }

private:
    uint8_t* buffer_ = nullptr;
    uint32_t count_ = 0;
    uint32_t stride_ = 0;
};

// The only format this driver renders: one plane, 32bpp XRGB8888, linear
// graphics with a positive pitch that holds the visible width.
bool usable(const VIDEO_MODE_INFORMATION& mode) noexcept {
    return mode.Length != 0 && mode.NumberOfPlanes == 1 && mode.BitsPerPlane == 32 &&
           (mode.AttributeFlags & VIDEO_MODE_GRAPHICS) != 0 && mode.RedMask == red_mask &&
           mode.GreenMask == green_mask && mode.BlueMask == blue_mask &&
           mode.VisScreenWidth != 0 && mode.VisScreenHeight != 0 &&
           mode.VisScreenWidth <= abi::max_surface_dimension &&
           mode.VisScreenHeight <= abi::max_surface_dimension &&
           mode.ScreenStride >= static_cast<uint64_t>(mode.VisScreenWidth) * 4U &&
           (mode.ScreenStride & 3U) == 0;
}

void describe(GDIINFO& info, DEVINFO& devinfo, const VIDEO_MODE_INFORMATION& mode,
              const DEVMODEW& requested) noexcept {
    info.ulVersion = GDI_DRIVER_VERSION;
    info.ulTechnology = DT_RASDISPLAY;
    info.ulHorzSize = mode.XMillimeter;
    info.ulVertSize = mode.YMillimeter;
    info.ulHorzRes = mode.VisScreenWidth;
    info.ulVertRes = mode.VisScreenHeight;
    info.ulPanningHorzRes = mode.VisScreenWidth;
    info.ulPanningVertRes = mode.VisScreenHeight;
    info.cBitsPixel = 32;
    info.cPlanes = 1;
    info.ulVRefresh = mode.Frequency;
    info.ulBltAlignment = 1;
    info.ulLogPixelsX = requested.dmLogPixels;
    info.ulLogPixelsY = requested.dmLogPixels;
    info.flTextCaps = TC_RA_ABLE;
    info.ulDACRed = 8;
    info.ulDACGreen = 8;
    info.ulDACBlue = 8;
    info.ulAspectX = 0x24;
    info.ulAspectY = 0x24;
    info.ulAspectXY = 0x33;
    info.xStyleStep = 1;
    info.yStyleStep = 1;
    info.denStyleStep = 3;
    // sRGB-like default chromaticities (no VIDEO_COLOR_CAPABILITIES query).
    info.ciDevice.Red.x = 6700;
    info.ciDevice.Red.y = 3300;
    info.ciDevice.Green.x = 2100;
    info.ciDevice.Green.y = 7100;
    info.ciDevice.Blue.x = 1400;
    info.ciDevice.Blue.y = 800;
    info.ciDevice.AlignmentWhite.x = 3127;
    info.ciDevice.AlignmentWhite.y = 3290;
    info.ciDevice.RedGamma = 20000;
    info.ciDevice.GreenGamma = 20000;
    info.ciDevice.BlueGamma = 20000;
    info.ulNumColors = static_cast<ULONG>(-1);
    info.ulPrimaryOrder = PRIMARY_ORDER_CBA;
    info.ulHTPatternSize = HT_PATSIZE_4x4_M;
    info.ulHTOutputFormat = HT_FORMAT_32BPP;
    info.flHTFlags = HT_FLAG_ADDITIVE_PRIMS;

    static const LOGFONTW system_font = {16, 7, 0, 0, 700, 0, 0, 0, ANSI_CHARSET,
        OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, DEFAULT_QUALITY, VARIABLE_PITCH | FF_DONTCARE,
        L"System"};
    static const LOGFONTW variable_font = {12, 9, 0, 0, 400, 0, 0, 0, ANSI_CHARSET,
        OUT_DEFAULT_PRECIS, CLIP_STROKE_PRECIS, PROOF_QUALITY, VARIABLE_PITCH | FF_DONTCARE,
        L"MS Sans Serif"};
    static const LOGFONTW fixed_font = {12, 9, 0, 0, 400, 0, 0, 0, ANSI_CHARSET,
        OUT_DEFAULT_PRECIS, CLIP_STROKE_PRECIS, PROOF_QUALITY, FIXED_PITCH | FF_DONTCARE,
        L"Courier"};
    devinfo.flGraphicsCaps = 0;
    // GdiFlush reaches DrvSynchronizeSurface(DSS_FLUSH_EVENT) on engines that
    // implement it, giving applications a real GPU completion barrier.
    // GCAPS2_ALPHACURSOR: SPS_ALPHA shapes reach DrvSetPointerShape, which
    // expresses them as DCN premultiplied cursors or declines them.
    devinfo.flGraphicsCaps2 = GCAPS2_SYNCFLUSH | GCAPS2_ALPHACURSOR;
    devinfo.lfDefaultFont = system_font;
    devinfo.lfAnsiVarFont = variable_font;
    devinfo.lfAnsiFixFont = fixed_font;
    devinfo.iDitherFormat = BMF_32BPP;
}

bool matches(const VIDEO_MODE_INFORMATION& mode, const DEVMODEW& requested) noexcept {
    const bool unspecified = requested.dmPelsWidth == 0 && requested.dmPelsHeight == 0 &&
        requested.dmBitsPerPel == 0 && requested.dmDisplayFrequency == 0;
    return unspecified || (requested.dmPelsWidth == mode.VisScreenWidth &&
        requested.dmPelsHeight == mode.VisScreenHeight && requested.dmBitsPerPel == 32 &&
        (requested.dmDisplayFrequency == 0 || requested.dmDisplayFrequency == mode.Frequency));
}

void unmap(Pdev& device) noexcept {
    if (device.framebuffer == nullptr) {
        return;
    }

    VIDEO_MEMORY memory{};
    memory.RequestedVirtualAddress = device.framebuffer;
    DWORD returned = 0;
    EngDeviceIoControl(device.driver, IOCTL_VIDEO_UNMAP_VIDEO_MEMORY, &memory, sizeof(memory),
                       nullptr, 0, &returned);
    device.framebuffer = nullptr;
}

// DRVFN stores every entry as the generic PFN; the engine calls each through
// its documented INDEX_Drv* prototype. The void(*)() step is the conventional
// generic function-pointer type GCC exempts from -Wcast-function-type.
template<typename Function>
PFN ddi_function(Function function) noexcept {
    return reinterpret_cast<PFN>(reinterpret_cast<void (*)()>(function));
}

void release_surfaces(Pdev& device) noexcept {
    if (device.primary != nullptr) {
        EngDeleteSurface(device.primary);
        device.primary = nullptr;
    }

    if (device.bitmap != nullptr) {
        EngUnlockSurface(device.bitmap);
        device.bitmap = nullptr;
    }

    if (device.bitmap_handle != nullptr) {
        EngDeleteSurface(device.bitmap_handle);
        device.bitmap_handle = nullptr;
    }

    unmap(device);
}


void unlink_bitmap(Pdev& device, DeviceBitmap& bitmap) noexcept {
    DeviceBitmap** link = &device.bitmaps;
    while (*link != nullptr && *link != &bitmap) {
        link = &(*link)->next;
    }

    if (*link != nullptr) {
        *link = bitmap.next;
        --device.bitmap_count;
    }

    bitmap.next = nullptr;
    device.statistics.live_bitmaps = device.bitmap_count;
}

// Registry owner of a GDI surface: our primary or device bitmap.
Pdev* surface_owner(SURFOBJ* surface) noexcept {
    Operand operand{};
    return classify(surface, operand) ? operand.owner : nullptr;
}

// Only a PDEV's primary carries the pointer.
Pdev* pointer_owner(SURFOBJ* surface) noexcept {
    Operand operand{};
    return classify(surface, operand) && operand.primary ? operand.owner : nullptr;
}

} // namespace

// ---------------------------------------------------------------------------
// DDI entry points. GDI calls them at PASSIVE_LEVEL in a win32k thread and
// serializes each PDEV's drawing with its devlock; the registry semaphore
// additionally serializes our state across PDEVs and against retirement.
// ---------------------------------------------------------------------------
extern "C" {

DHPDEV APIENTRY DrvEnablePDEV(DEVMODEW* requested, LPWSTR, ULONG, HSURF*, ULONG caps_bytes,
                              ULONG* caps, ULONG devinfo_bytes, DEVINFO* devinfo, HDEV,
                              LPWSTR, HANDLE driver) {
    if (requested == nullptr || caps == nullptr || devinfo == nullptr || driver == nullptr) {
        return nullptr;
    }

    ModeList modes;
    if (!modes.load(driver)) {
        return nullptr;
    }

    VIDEO_MODE_INFORMATION selected{};
    bool found = false;
    for (uint32_t index = 0; index < modes.count() && !found; ++index) {
        const VIDEO_MODE_INFORMATION mode = modes.at(index);
        if (usable(mode) && matches(mode, *requested)) {
            selected = mode;
            found = true;
        }
    }

    if (!found) {
        return nullptr;
    }

    void* storage = EngAllocMem(FL_ZERO_MEMORY, sizeof(Pdev), allocation_tag);
    if (storage == nullptr) {
        return nullptr;
    }

    Pdev* device = rx7900::reactos::construct_device_state<Pdev>(storage);
    device->driver = driver;
    device->mode_index = selected.ModeIndex;
    device->width = selected.VisScreenWidth;
    device->height = selected.VisScreenHeight;
    device->pitch = selected.ScreenStride;
    device->palette = EngCreatePalette(PAL_BITFIELDS, 0, nullptr, red_mask, green_mask, blue_mask);
    if (device->palette == nullptr) {
        EngFreeMem(device);
        return nullptr;
    }

    GDIINFO& info = device->gdi_info;
    DEVINFO& description = device->dev_info;
    describe(info, description, selected, *requested);
    description.hpalDefault = device->palette;
    // GDI owns both output buffers and declares their byte counts.
    memcpy(caps, &info, caps_bytes < sizeof(info) ? caps_bytes : sizeof(info));
    memcpy(devinfo, &description, devinfo_bytes < sizeof(description) ? devinfo_bytes : sizeof(description));
    return reinterpret_cast<DHPDEV>(device);
}

VOID APIENTRY DrvCompletePDEV(DHPDEV pdev, HDEV engine) {
    reinterpret_cast<Pdev*>(pdev)->engine = engine;
}

VOID APIENTRY DrvDisablePDEV(DHPDEV pdev) {
    Pdev* device = reinterpret_cast<Pdev*>(pdev);
    if (registry_lock != nullptr && device->registered) {
        RegistryGuard guard;
        Pdev** link = &registry;
        while (*link != nullptr && *link != device) {
            link = &(*link)->next;
        }

        if (*link != nullptr) {
            *link = device->next;
        }

        device->next = nullptr;
        device->registered = false;
    }

    // GDI deletes every device bitmap before its PDEV; anything left here was
    // never handed out successfully or GDI broke that rule, so free the DIBs.
    while (device->bitmaps != nullptr) {
        DeviceBitmap* bitmap = device->bitmaps;
        unlink_bitmap(*device, *bitmap);
        release_dib(*bitmap);
        EngFreeMem(bitmap);
    }

    if (device->palette != nullptr) {
        EngDeletePalette(device->palette);
    }

    EngFreeMem(device);
}

HSURF APIENTRY DrvEnableSurface(DHPDEV pdev) {
    if (registry_lock == nullptr) {
        return nullptr;
    }

    Pdev& device = *reinterpret_cast<Pdev*>(pdev);
    ULONG mode = device.mode_index;
    DWORD returned = 0;
    if (EngDeviceIoControl(device.driver, IOCTL_VIDEO_SET_CURRENT_MODE, &mode, sizeof(mode),
                           nullptr, 0, &returned) != 0) {
        return nullptr;
    }

    VIDEO_MEMORY request{};
    VIDEO_MEMORY_INFORMATION mapping{};
    if (EngDeviceIoControl(device.driver, IOCTL_VIDEO_MAP_VIDEO_MEMORY, &request, sizeof(request),
                           &mapping, sizeof(mapping), &returned) != 0 || returned != sizeof(mapping) ||
        mapping.FrameBufferBase == nullptr) {
        return nullptr;
    }

    device.framebuffer = mapping.FrameBufferBase;
    const uintptr_t base = reinterpret_cast<uintptr_t>(mapping.FrameBufferBase);
    const uint64_t extent = static_cast<uint64_t>(device.pitch) * device.height;
    if (!policy::valid_layout(device.width, device.height, device.pitch, mapping.FrameBufferLength) ||
        extent > UINTPTR_MAX - base) {
        release_surfaces(device);
        return nullptr;
    }

    const SIZEL size{static_cast<LONG>(device.width), static_cast<LONG>(device.height)};
    device.bitmap_handle = reinterpret_cast<HSURF>(EngCreateBitmap(size, static_cast<LONG>(device.pitch),
        BMF_32BPP, BMF_TOPDOWN, device.framebuffer));
    if (device.bitmap_handle != nullptr) {
        device.bitmap = EngLockSurface(device.bitmap_handle);
    }

    if (device.bitmap != nullptr) {
        device.primary = EngCreateDeviceSurface(reinterpret_cast<DHSURF>(&device), size, BMF_32BPP);
    }

    if (device.primary == nullptr || !EngAssociateSurface(device.primary, device.engine, primary_hooks)) {
        release_surfaces(device);
        return nullptr;
    }

    RegistryGuard guard;
    if (!device.registered) {
        device.next = registry;
        registry = &device;
        device.registered = true;
    }

    open_session(device);
    device.statistics.accelerated = device.accelerated ? 1U : 0U;
    return device.primary;
}

VOID APIENTRY DrvDisableSurface(DHPDEV pdev) {
    if (registry_lock == nullptr) {
        return;
    }

    Pdev& device = *reinterpret_cast<Pdev*>(pdev);
    RegistryGuard guard;
    rx7900::reactos::radeon_cursor::retire_pointer(device.pointer, device.driver);
    // Device bitmaps become DIBs, then a barrier before the surface returns to
    // GDI; retiring the session lets the miniport prove (or quarantine) the
    // staging and surface lifetimes. The PDEV stays registered for its bitmaps.
    close_session(device);
    release_surfaces(device);
}

BOOL APIENTRY DrvAssertMode(DHPDEV pdev, BOOL enable) {
    if (registry_lock == nullptr) {
        return FALSE;
    }

    Pdev& device = *reinterpret_cast<Pdev*>(pdev);
    DWORD returned = 0;
    RegistryGuard guard;
    if (!enable) {
        // Console or another PDEV takes the scanout: finish our GPU work and
        // release the primary so the miniport refuses further submissions to
        // it. Offscreen bitmaps stay resident: the session and VRAM survive
        // (IOCTL_VIDEO_RESET_DEVICE keeps the adopted mode and the 2D engine).
        if (!sync_fence(device, 0)) {
            return FALSE;
        }

        if (device.session_open) {
            assert_primary(device, false);
        }

        device.primary_wanted = false;
        device.accelerated = false;
        device.statistics.accelerated = 0;
        return EngDeviceIoControl(device.driver, IOCTL_VIDEO_RESET_DEVICE, nullptr, 0, nullptr, 0,
                                  &returned) == 0;
    }

    ULONG mode = device.mode_index;
    if (EngDeviceIoControl(device.driver, IOCTL_VIDEO_SET_CURRENT_MODE, &mode, sizeof(mode),
                           nullptr, 0, &returned) != 0) {
        return FALSE;
    }

    // Remembered even while poisoned: a revived session takes the primary back.
    device.primary_wanted = true;
    device.accelerated = device.session_open && !device.poisoned && assert_primary(device, true);
    device.statistics.accelerated = device.accelerated ? 1U : 0U;
    return TRUE;
}

ULONG APIENTRY DrvGetModes(HANDLE driver, ULONG bytes, DEVMODEW* output) {
    ModeList modes;
    if (driver == nullptr || !modes.load(driver)) {
        return 0;
    }

    uint32_t usable_count = 0;
    for (uint32_t index = 0; index < modes.count(); ++index) {
        usable_count += usable(modes.at(index)) ? 1U : 0U;
    }

    if (output == nullptr) {
        return usable_count * sizeof(DEVMODEW);
    }

    ULONG written = 0;
    for (uint32_t index = 0; index < modes.count(); ++index) {
        const VIDEO_MODE_INFORMATION mode = modes.at(index);
        if (!usable(mode) || bytes - written < sizeof(DEVMODEW)) {
            continue;
        }

        DEVMODEW entry{};
        static const wchar_t name[] = L"radeondisp";
        static_assert(sizeof(name) <= sizeof(entry.dmDeviceName), "Bounded device name");
        memcpy(entry.dmDeviceName, name, sizeof(name));
        entry.dmSpecVersion = DM_SPECVERSION;
        entry.dmDriverVersion = DM_SPECVERSION;
        entry.dmSize = sizeof(DEVMODEW);
        entry.dmBitsPerPel = 32;
        entry.dmPelsWidth = mode.VisScreenWidth;
        entry.dmPelsHeight = mode.VisScreenHeight;
        entry.dmDisplayFrequency = mode.Frequency;
        entry.dmFields = DM_BITSPERPEL | DM_PELSWIDTH | DM_PELSHEIGHT | DM_DISPLAYFREQUENCY |
                         DM_DISPLAYFLAGS;
        // GDI owns bytes of output; the loop condition proves room for one entry.
        memcpy(reinterpret_cast<uint8_t*>(output) + written, &entry, sizeof(entry));
        written += sizeof(DEVMODEW);
    }

    return written;
}

// The pinned engine makes engine bitmaps for CreateCompatibleBitmap; the
// optional win32k device-bitmap bridge (radeon_devbitmap_bridge, identity bit
// flag_device_bitmaps) and NT-compatible engines call this for compatible
// bitmaps of this PDEV, under its device lock. Returning 0 makes GDI create an
// engine bitmap instead, which is how admission refusals, budget exhaustion and
// VRAM pressure punt. The bridge requires the BitBlt/CopyBits hooks: engine
// paths reach the opaque bitmap only through them.
static_assert((bitmap_hooks & (HOOK_BITBLT | HOOK_COPYBITS)) == (HOOK_BITBLT | HOOK_COPYBITS),
              "Device bitmaps hook the calls that reach their pixels");
HBITMAP APIENTRY DrvCreateDeviceBitmap(DHPDEV pdev, SIZEL size, ULONG format) {
    if (registry_lock == nullptr || pdev == nullptr || format != BMF_32BPP) {
        return nullptr;
    }

    RegistryGuard guard;
    Pdev* owner = registered_pdev(pdev);
    if (owner == nullptr || !session_usable(*owner)) {
        return nullptr;
    }

    Pdev& device = *owner;
    if (display::admit_bitmap(size.cx, size.cy, device.resident_count, device.resident_bytes,
                              device.bitmap_limits) != display::BitmapAdmission::admit) {
        display::count(device.statistics.bitmaps_refused);
        return nullptr;
    }

    void* storage = EngAllocMem(FL_ZERO_MEMORY, sizeof(DeviceBitmap), allocation_tag);
    if (storage == nullptr) {
        display::count(device.statistics.bitmaps_refused);
        return nullptr;
    }

    DeviceBitmap* bitmap = rx7900::reactos::construct_device_state<DeviceBitmap>(storage);
    bitmap->owner = &device;
    bitmap->width = static_cast<uint32_t>(size.cx);
    bitmap->height = static_cast<uint32_t>(size.cy);
    uint64_t bytes = 0;
    bitmap->surface = create_surface(device, bitmap->width, bitmap->height, bytes);
    if (bitmap->surface == 0) {
        EngFreeMem(bitmap);
        display::count(device.statistics.bitmaps_refused);
        return nullptr;
    }

    bitmap->bytes = bytes;
    device.resident_bytes += bytes;
    ++device.resident_count;
    HBITMAP handle = EngCreateDeviceBitmap(reinterpret_cast<DHSURF>(bitmap), size, BMF_32BPP);
    if (handle == nullptr ||
        !EngAssociateSurface(reinterpret_cast<HSURF>(handle), device.engine, bitmap_hooks)) {
        if (handle != nullptr) {
            EngDeleteSurface(reinterpret_cast<HSURF>(handle));
        }

        destroy_surface(device, *bitmap);
        EngFreeMem(bitmap);
        display::count(device.statistics.bitmaps_refused);
        return nullptr;
    }

    bitmap->handle = reinterpret_cast<HSURF>(handle);
    bitmap->next = device.bitmaps;
    device.bitmaps = bitmap;
    ++device.bitmap_count;
    device.statistics.live_bitmaps = device.bitmap_count;
    device.statistics.live_bitmap_bytes = device.resident_bytes;
    display::count(device.statistics.bitmaps_created);
    return handle;
}

// Called once per bitmap: by the bridge from SURFACE_vCleanup (GDI frees the
// HBITMAP after this returns) or after it converted the bitmap into an engine
// bitmap through DrvCopyBits (the HBITMAP lives on without this driver).
VOID APIENTRY DrvDeleteDeviceBitmap(DHSURF handle) {
    if (registry_lock == nullptr || handle == nullptr) {
        return;
    }

    RegistryGuard guard;
    for (Pdev* device = registry; device != nullptr; device = device->next) {
        DeviceBitmap* bitmap = owned_bitmap(*device, handle);
        if (bitmap == nullptr) {
            continue;
        }

        unlink_bitmap(*device, *bitmap);
        destroy_surface(*device, *bitmap);
        release_dib(*bitmap);
        device->statistics.live_bitmap_bytes = device->resident_bytes;
        display::count(device->statistics.bitmaps_deleted);
        EngFreeMem(bitmap);
        return;
    }
}

BOOL APIENTRY DrvBitBlt(SURFOBJ* target, SURFOBJ* source, SURFOBJ* mask, CLIPOBJ* clip,
                        XLATEOBJ* translation, RECTL* rectangle, POINTL* source_point,
                        POINTL* mask_point, BRUSHOBJ* brush, POINTL* brush_origin, ROP4 rop) {
    if (registry_lock == nullptr || target == nullptr || rectangle == nullptr) {
        return FALSE;
    }

    RegistryGuard guard;
    Operand target_operand{};
    Operand source_operand{};
    Operand mask_operand{};
    if (!classify(target, target_operand) || !classify(source, source_operand) ||
        !classify(mask, mask_operand)) {
        return FALSE;
    }

    Pdev* device = target_operand.owner != nullptr ? target_operand.owner : source_operand.owner;
    if (device != nullptr) {
        Attempt attempt = accelerate(*device, target_operand, source_operand, clip, translation,
            *rectangle, source_point, mask != nullptr, brush, rop);
        if (untouched(attempt)) {
            // Beyond the SDMA plans: the compute raster engine.
            attempt = compute_bitblt(*device, target_operand, source_operand, mask, clip, translation, *rectangle,
                                     source_point, mask_point, brush, brush_origin, rop);
        }

        if (!untouched(attempt)) {
            return attempt == Attempt::performed ? TRUE : FALSE;
        }

        display::count(device->statistics.fallback_bitblt);
    }

    SURFOBJ* cpu_target = cpu_view(target_operand);
    SURFOBJ* cpu_source = source != nullptr ? cpu_view(source_operand) : nullptr;
    SURFOBJ* cpu_mask = mask != nullptr ? cpu_view(mask_operand) : nullptr;
    if (cpu_target == nullptr || (source != nullptr && cpu_source == nullptr) ||
        (mask != nullptr && cpu_mask == nullptr)) {
        return FALSE;
    }

    RECTL operation = *rectangle;
    return EngBitBlt(cpu_target, cpu_source, cpu_mask, clip, translation, &operation,
                     source_point, mask_point, brush, brush_origin, rop);
}

BOOL APIENTRY DrvCopyBits(SURFOBJ* target, SURFOBJ* source, CLIPOBJ* clip, XLATEOBJ* translation,
                          RECTL* rectangle, POINTL* source_point) {
    if (registry_lock == nullptr || target == nullptr || source == nullptr || rectangle == nullptr ||
        source_point == nullptr) {
        return FALSE;
    }

    RegistryGuard guard;
    Operand target_operand{};
    Operand source_operand{};
    if (!classify(target, target_operand) || !classify(source, source_operand)) {
        return FALSE;
    }

    Pdev* device = target_operand.owner != nullptr ? target_operand.owner : source_operand.owner;
    if (device != nullptr) {
        Attempt attempt = accelerate(*device, target_operand, source_operand, clip, translation,
            *rectangle, source_point, false, nullptr, 0xccccU);
        if (untouched(attempt)) {
            // Translated 1bpp sources (mono expansion) on the compute engine.
            attempt = compute_bitblt(*device, target_operand, source_operand, nullptr, clip, translation, *rectangle,
                                     source_point, nullptr, nullptr, nullptr, 0xccccU);
        }

        if (!untouched(attempt)) {
            return attempt == Attempt::performed ? TRUE : FALSE;
        }

        display::count(device->statistics.fallback_copybits);
    }

    SURFOBJ* cpu_target = cpu_view(target_operand);
    SURFOBJ* cpu_source = cpu_view(source_operand);
    if (cpu_target == nullptr || cpu_source == nullptr) {
        return FALSE;
    }

    RECTL operation = *rectangle;
    POINTL origin = *source_point;
    return EngCopyBits(cpu_target, cpu_source, clip, translation, &operation, &origin);
}

// Not dispatched by the current ReactOS engine (EngFillPath is a stub there);
// provided for NT-compatible engines. A single solid R2_COPYPEN rectangle is
// a GPU fill (in batches for large clips); anything else is passed to
// EngFillPath on the CPU view.
BOOL APIENTRY DrvFillPath(SURFOBJ* surface, PATHOBJ* path, CLIPOBJ* clip, BRUSHOBJ* brush,
                          POINTL* brush_origin, MIX mix, FLONG options) {
    if (registry_lock == nullptr || surface == nullptr || path == nullptr) {
        return FALSE;
    }

    RegistryGuard guard;
    Operand operand{};
    if (!classify(surface, operand) || operand.owner == nullptr) {
        return FALSE;
    }

    Pdev* device = operand.owner;
    const bool gpu = operand.primary ? device->accelerated :
        session_usable(*device) && operand.bitmap->surface != 0;
    if (gpu && brush != nullptr && brush->iSolidColor != display::no_solid_color &&
        (mix & 0xffU) == display::mix_copy_pen) {
        PATHDATA data{};
        PATHOBJ_vEnumStart(path);
        const BOOL more = PATHOBJ_bEnum(path, &data);
        int32_t points[10]{};
        display::Rectangle filled{};
        bool single = !more && data.pptfx != nullptr && (data.count == 4 || data.count == 5) &&
            (data.flags & PD_BEGINSUBPATH) != 0 && (data.flags & PD_ENDSUBPATH) != 0;
        for (ULONG index = 0; single && index < data.count; ++index) {
            // GDI owns data.count POINTFIX entries until the next enumeration.
            points[index * 2] = data.pptfx[index].x;
            points[index * 2 + 1] = data.pptfx[index].y;
        }

        uint32_t width = 0;
        uint32_t height = 0;
        gpu_size(*device, operand, width, height);
        if (single && display::rectangle_from_path(points, data.count, filled)) {
            const RECTL rectangle{filled.left, filled.top, filled.right, filled.bottom};
            if (clip_begin(rectangle, clip, width, height, CD_ANY, device->clip)) {
                const uint32_t handle = gpu_handle(*device, operand);
                const uint32_t color = brush->iSolidColor;
                const Attempt attempt = for_each_batch(*device, [&](const abi::RectList& batch) noexcept {
                    return submit_fill(*device, handle, color, batch);
                });
                if (!untouched(attempt)) {
                    if (attempt == Attempt::performed) {
                        display::count(device->statistics.gpu_fills);
                    }

                    return attempt == Attempt::performed ? TRUE : FALSE;
                }
            }
        }
    }

    display::count(device->statistics.fallback_fillpath);
    SURFOBJ* view = cpu_view(operand);
    if (view == nullptr) {
        return FALSE;
    }

    return EngFillPath(view, path, clip, brush, brush_origin, mix, options);
}

// RGLP from win32k's DrawEscape bridge (reactos-draw-escape.patch): present the
// RadeonGL back buffer into the visible window region. The pure policy is in
// radeon_present.hpp; 0 tells the ICD to use its exact readback path.
ULONG APIENTRY DrvDrawEscape(SURFOBJ* surface, ULONG escape, CLIPOBJ* clip, RECTL* window, ULONG input_bytes,
                             PVOID input) {
    namespace present = rx7900::reactos::radeon_present;
    if (escape != present::gabi::present_escape || registry_lock == nullptr || surface == nullptr ||
        window == nullptr || input == nullptr || input_bytes != present::gabi::present_input_bytes) {
        return 0;
    }

    RegistryGuard guard;
    Pdev* device = surface_owner(surface);
    present::gabi::PresentInput decoded{};
    if (device == nullptr || device->driver == nullptr ||
        !present::gabi::decode_present_input(
            rx7900::ByteView(static_cast<const uint8_t*>(input), input_bytes), decoded)) {
        return 0;
    }

    // radeondisp's own SDMA/compute writes to the primary complete before the
    // miniport's copy rows land (the GdiFlush barrier).
    if (!sync_fence(*device, 0)) {
        return 0;
    }

    present::ClipKind kind = present::ClipKind::missing;
    present::Rect bounds{};
    if (clip != nullptr) {
        bounds = {clip->rclBounds.left, clip->rclBounds.top, clip->rclBounds.right, clip->rclBounds.bottom};
        kind = clip->iDComplexity == DC_TRIVIAL ? present::ClipKind::empty :
               clip->iDComplexity == DC_RECT    ? present::ClipKind::rect :
               clip->iDComplexity == DC_COMPLEX ? present::ClipKind::complex :
                                                  present::ClipKind::unknown;
    }
    const present::Rect target{window->left, window->top, window->right, window->bottom};
    const present::Rect screen{0, 0, static_cast<int32_t>(surface->sizlBitmap.cx),
                               static_cast<int32_t>(surface->sizlBitmap.cy)};

    auto enumerate = [clip](auto&& visit) noexcept {
        struct {
            ULONG count;
            RECTL rects[8];
        } batch;
        CLIPOBJ_cEnumStart(clip, FALSE, CT_RECTANGLES, CD_ANY, 0);
        bool more = true;
        while (more) {
            more = CLIPOBJ_bEnum(clip, sizeof(batch), reinterpret_cast<ULONG*>(&batch)) != FALSE;
            const ULONG count = batch.count <= 8 ? batch.count : 8;
            for (ULONG index = 0; index < count; ++index) {
                const RECTL& r = batch.rects[index];
                visit(present::Rect{r.left, r.top, r.right, r.bottom});
            }
        }
        return true;
    };
    auto send = [device](const present::gabi::PresentRequest& request, uint32_t& outcome) noexcept {
        if (!present::gabi::encode_present_request(request, device->present_bytes, sizeof(device->present_bytes))) {
            return false;
        }
        uint32_t reply[2] = {};
        DWORD returned = 0;
        if (EngDeviceIoControl(device->driver, present::gabi::present_ioctl, device->present_bytes,
                               sizeof(device->present_bytes), reply, sizeof(reply), &returned) != 0 ||
            returned != present::gabi::present_reply_bytes) {
            return false;
        }
        outcome = reply[0];
        return true;
    };
    return present::present(device->present_request, decoded, target, screen, kind, bounds, enumerate, send).value;
}

VOID APIENTRY DrvSynchronizeSurface(SURFOBJ* surface, RECTL*, FLONG) {
    if (registry_lock == nullptr || surface == nullptr) {
        return;
    }

    RegistryGuard guard;
    Pdev* device = surface_owner(surface);
    if (device != nullptr) {
        sync_fence(*device, 0);
    }
}

// Brush realization for the compute pattern path: 8x8 patterns without a
// mask, every pixel translated with the engine's XLATEOBJ exactly as
// EngRealizeBrush converts the pattern to the target format. FALSE leaves the
// brush to the engine realization (BRUSHOBJ_pvGetRbrush then returns NULL and
// DrvBitBlt falls back).
BOOL APIENTRY DrvRealizeBrush(BRUSHOBJ* brush, SURFOBJ* target, SURFOBJ* pattern, SURFOBJ* mask,
                              XLATEOBJ* translation, ULONG) {
    if (registry_lock == nullptr || brush == nullptr || target == nullptr) {
        return FALSE;
    }

    RegistryGuard guard;
    Pdev* device = surface_owner(target);
    const ULONG format = pattern != nullptr ? pattern->iBitmapFormat : 0;
    // Only GPU targets (32bpp BGRX) use the tile; other targets keep the
    // engine's own realization.
    const bool supported = device != nullptr && target->iBitmapFormat == BMF_32BPP && pattern != nullptr &&
        mask == nullptr && pattern->sizlBitmap.cx == 8 &&
        pattern->sizlBitmap.cy == 8 && pattern->pvScan0 != nullptr &&
        (format == BMF_1BPP || format == BMF_4BPP || format == BMF_8BPP || format == BMF_16BPP ||
         format == BMF_24BPP || format == BMF_32BPP) && (translation != nullptr || format == BMF_32BPP);
    if (!supported) {
        if (device != nullptr) {
            display::count(device->statistics.brushes_refused);
        }

        return FALSE;
    }

    // GDI owns the allocation for the brush's lifetime.
    auto* realized = static_cast<RealizedBrush*>(BRUSHOBJ_pvAllocRbrush(brush, sizeof(RealizedBrush)));
    if (realized == nullptr) {
        return FALSE;
    }

    realized->magic = realized_magic;
    for (uint32_t y = 0; y < 8; ++y) {
        // Row y of the 8x8 pattern (lDelta may be negative); bytes per row >= 8 * bpp / 8.
        const uint8_t* row = static_cast<const uint8_t*>(pattern->pvScan0) +
            static_cast<intptr_t>(y) * pattern->lDelta;
        for (uint32_t x = 0; x < 8; ++x) {
            ULONG pixel = 0;
            switch (format) {
            case BMF_1BPP: pixel = (row[0] >> (7U - x)) & 1U; break;
            case BMF_4BPP: pixel = (row[x / 2] >> ((x & 1U) != 0 ? 0U : 4U)) & 0xfU; break;
            case BMF_8BPP: pixel = row[x]; break;
            case BMF_16BPP: pixel = static_cast<ULONG>(row[x * 2]) | (static_cast<ULONG>(row[x * 2 + 1]) << 8); break;
            case BMF_24BPP:
                pixel = static_cast<ULONG>(row[x * 3]) | (static_cast<ULONG>(row[x * 3 + 1]) << 8) |
                        (static_cast<ULONG>(row[x * 3 + 2]) << 16);
                break;
            default:
                pixel = static_cast<ULONG>(row[x * 4]) | (static_cast<ULONG>(row[x * 4 + 1]) << 8) |
                        (static_cast<ULONG>(row[x * 4 + 2]) << 16) | (static_cast<ULONG>(row[x * 4 + 3]) << 24);
                break;
            }

            realized->tile[y * 8 + x] = translation != nullptr ? XLATEOBJ_iXlate(translation, pixel) : pixel;
        }
    }

    if (device != nullptr) {
        display::count(device->statistics.brushes_realized);
    }

    return TRUE;
}

BOOL APIENTRY DrvTransparentBlt(SURFOBJ* target, SURFOBJ* source, CLIPOBJ* clip, XLATEOBJ* translation,
                                RECTL* destination, RECTL* source_rect, ULONG key, ULONG reserved) {
    if (registry_lock == nullptr || target == nullptr || source == nullptr || destination == nullptr ||
        source_rect == nullptr) {
        return FALSE;
    }

    RegistryGuard guard;
    Operand target_operand{};
    Operand source_operand{};
    if (!classify(target, target_operand) || !classify(source, source_operand)) {
        return FALSE;
    }

    Pdev* device = target_operand.owner;
    if (device != nullptr) {
        const Attempt attempt = accelerate_transparent(*device, target_operand, source_operand, clip, translation,
                                                       *destination, *source_rect, key);
        if (!untouched(attempt)) {
            return attempt == Attempt::performed ? TRUE : FALSE;
        }

        display::count(device->statistics.fallback_transparent);
    }

    SURFOBJ* cpu_target = cpu_view(target_operand);
    SURFOBJ* cpu_source = cpu_view(source_operand);
    if (cpu_target == nullptr || cpu_source == nullptr) {
        return FALSE;
    }

    RECTL output = *destination;
    RECTL input = *source_rect;
    return EngTransparentBlt(cpu_target, cpu_source, clip, translation, &output, &input, key, reserved);
}

BOOL APIENTRY DrvAlphaBlend(SURFOBJ* target, SURFOBJ* source, CLIPOBJ* clip, XLATEOBJ* translation,
                            RECTL* destination, RECTL* source_rect, BLENDOBJ* blend) {
    if (registry_lock == nullptr || target == nullptr || source == nullptr || destination == nullptr ||
        source_rect == nullptr || blend == nullptr) {
        return FALSE;
    }

    RegistryGuard guard;
    Operand target_operand{};
    Operand source_operand{};
    if (!classify(target, target_operand) || !classify(source, source_operand)) {
        return FALSE;
    }

    Pdev* device = target_operand.owner;
    if (device != nullptr) {
        const Attempt attempt = accelerate_alpha(*device, target_operand, source_operand, clip, translation,
                                                 *destination, *source_rect, blend);
        if (!untouched(attempt)) {
            return attempt == Attempt::performed ? TRUE : FALSE;
        }

        display::count(device->statistics.fallback_alphablend);
    }

    SURFOBJ* cpu_target = cpu_view(target_operand);
    SURFOBJ* cpu_source = cpu_view(source_operand);
    if (cpu_target == nullptr || cpu_source == nullptr) {
        return FALSE;
    }

    RECTL output = *destination;
    RECTL input = *source_rect;
    return EngAlphaBlend(cpu_target, cpu_source, clip, translation, &output, &input, blend);
}

BOOL APIENTRY DrvGradientFill(SURFOBJ* target, CLIPOBJ* clip, XLATEOBJ* translation, TRIVERTEX* vertices,
                              ULONG vertex_count, PVOID mesh, ULONG mesh_count, RECTL* extents, POINTL* dither,
                              ULONG mode) {
    if (registry_lock == nullptr || target == nullptr) {
        return FALSE;
    }

    RegistryGuard guard;
    Operand target_operand{};
    if (!classify(target, target_operand)) {
        return FALSE;
    }

    Pdev* device = target_operand.owner;
    if (device != nullptr) {
        const Attempt attempt = accelerate_gradient(*device, target_operand, clip, translation, vertices, vertex_count,
                                                    mesh, mesh_count, extents, dither, mode);
        if (!untouched(attempt)) {
            return attempt == Attempt::performed ? TRUE : FALSE;
        }

        display::count(device->statistics.fallback_gradient);
    }

    SURFOBJ* cpu_target = cpu_view(target_operand);
    if (cpu_target == nullptr) {
        return FALSE;
    }

    return EngGradientFill(cpu_target, clip, translation, vertices, vertex_count, mesh, mesh_count, extents, dither,
                           mode);
}

BOOL APIENTRY DrvLineTo(SURFOBJ* target, CLIPOBJ* clip, BRUSHOBJ* brush, LONG x1, LONG y1, LONG x2, LONG y2,
                        RECTL* bounds, MIX mix) {
    if (registry_lock == nullptr || target == nullptr || brush == nullptr) {
        return FALSE;
    }

    RegistryGuard guard;
    Operand target_operand{};
    if (!classify(target, target_operand)) {
        return FALSE;
    }

    Pdev* device = target_operand.owner;
    if (device != nullptr && display::tagged_copy_pen(mix)) {
        const Attempt attempt = accelerate_line(*device, target_operand, clip, brush, x1, y1, x2, y2, bounds);
        if (!untouched(attempt)) {
            return attempt == Attempt::performed ? TRUE : FALSE;
        }
    }

    if (device != nullptr) {
        display::count(device->statistics.fallback_lineto);
    }

    SURFOBJ* cpu_target = cpu_view(target_operand);
    if (cpu_target == nullptr) {
        return FALSE;
    }

    // The engine never sees the paired-kernel tag.
    const MIX original = (mix & display::mix_tag_mask) == display::line_mix_tag ? (mix & ~display::mix_tag_mask) : mix;
    return EngLineTo(cpu_target, clip, brush, x1, y1, x2, y2, bounds, original);
}

// The opt-in pinned IntEngLineTo bridge (radeon_bridge) presents one integer
// cosmetic segment, with the private brush's exact style runs. No path is
// altered until every point, run, clip and target has passed admission;
// engine_style_bits reproduces the decline path's own DIB EngLineTo phase.
// FALSE is the bridge's signal to continue through its original DrvLineTo ->
// EngLineTo path.
BOOL APIENTRY DrvStrokePath(SURFOBJ* target, PATHOBJ* path, CLIPOBJ* clip, XFORMOBJ* transform,
                            BRUSHOBJ* brush, POINTL* brush_origin, LINEATTRS* attributes, MIX mix) {
    static_assert(rx7900::reactos::radeon_line::max_stroke_style_runs <= display::max_engine_style_runs,
                  "Every bridge style run fits the PDEV workspace");
    if (registry_lock == nullptr || target == nullptr || path == nullptr || brush == nullptr ||
        transform != nullptr || brush_origin != nullptr || attributes == nullptr ||
        !display::tagged_copy_pen(mix) || attributes->fl != LA_STYLED ||
        attributes->elWidth.l != 1 || attributes->elStyleState.l != 0 ||
        attributes->cstyle == 0 || attributes->cstyle > display::max_engine_style_runs ||
        attributes->pstyle == nullptr) {
        return FALSE;
    }

    RegistryGuard guard;
    Operand target_operand{};
    if (!classify(target, target_operand) || target_operand.owner == nullptr) {
        return FALSE;
    }
    Pdev& device = *target_operand.owner;
    for (uint32_t index = 0; index < attributes->cstyle; ++index) {
        const LONG run = attributes->pstyle[index].l;
        if (run <= 0 || run > static_cast<LONG>(abi::max_style_bits)) {
            return FALSE;
        }
        device.style_runs[index] = static_cast<uint32_t>(run);
    }

    PATHOBJ_vEnumStart(path);
    PATHDATA points{};
    const BOOL more = PATHOBJ_bEnum(path, &points);
    if (more != FALSE || points.count != 2 || points.pptfx == nullptr ||
        (points.flags & PD_BEGINSUBPATH) == 0 ||
        (points.flags & ~(PD_BEGINSUBPATH | PD_ENDSUBPATH)) != 0) {
        return FALSE;
    }
    const POINTFIX first = points.pptfx[0];
    const POINTFIX second = points.pptfx[1];
    if ((first.x & 15) != 0 || (first.y & 15) != 0 ||
        (second.x & 15) != 0 || (second.y & 15) != 0) {
        return FALSE;
    }
    const LONG x0 = first.x / 16;
    const LONG y0 = first.y / 16;
    const LONG x1 = second.x / 16;
    const LONG y1 = second.y / 16;
    const Attempt result = accelerate_line(device, target_operand, clip, brush, x0, y0, x1, y1,
                                           nullptr, device.style_runs, attributes->cstyle);
    return result == Attempt::performed ? TRUE : FALSE;
}

BOOL APIENTRY DrvStretchBlt(SURFOBJ* target, SURFOBJ* source, SURFOBJ* mask, CLIPOBJ* clip, XLATEOBJ* translation,
                            COLORADJUSTMENT* adjustment, POINTL* halftone_origin, RECTL* destination,
                            RECTL* source_rect, POINTL* mask_point, ULONG mode) {
    if (registry_lock == nullptr || target == nullptr || source == nullptr || destination == nullptr ||
        source_rect == nullptr) {
        return FALSE;
    }

    RegistryGuard guard;
    Operand target_operand{};
    Operand source_operand{};
    Operand mask_operand{};
    if (!classify(target, target_operand) || !classify(source, source_operand) || !classify(mask, mask_operand)) {
        return FALSE;
    }

    Pdev* device = target_operand.owner;
    if (device != nullptr) {
        const Attempt attempt = accelerate_stretch(*device, target_operand, source_operand, mask, clip, translation,
                                                   *destination, *source_rect, mode);
        if (!untouched(attempt)) {
            return attempt == Attempt::performed ? TRUE : FALSE;
        }

        display::count(device->statistics.fallback_stretch);
    }

    SURFOBJ* cpu_target = cpu_view(target_operand);
    SURFOBJ* cpu_source = cpu_view(source_operand);
    SURFOBJ* cpu_mask = mask != nullptr ? cpu_view(mask_operand) : nullptr;
    if (cpu_target == nullptr || cpu_source == nullptr || (mask != nullptr && cpu_mask == nullptr)) {
        return FALSE;
    }

    return EngStretchBlt(cpu_target, cpu_source, cpu_mask, clip, translation, adjustment, halftone_origin,
                         destination, source_rect, mask_point, mode);
}

// Guarded StretchBltROP (paired kernel): only a call whose COLORADJUSTMENT the
// stretch guard tagged is drawn, because only then are the mirror flags known.
// FALSE (counted) leaves the engine's own mirrored EngStretchBltROP fallback,
// which this driver reaches through DrvCopyBits; nothing is drawn before it.
BOOL APIENTRY DrvStretchBltROP(SURFOBJ* target, SURFOBJ* source, SURFOBJ* mask, CLIPOBJ* clip,
                               XLATEOBJ* translation, COLORADJUSTMENT* adjustment, POINTL* brush_origin,
                               RECTL* destination, RECTL* source_rect, POINTL*, ULONG, BRUSHOBJ* brush,
                               ROP4 rop) {
    if (registry_lock == nullptr || target == nullptr || destination == nullptr || source_rect == nullptr) {
        return FALSE;
    }

    RegistryGuard guard;
    Operand target_operand{};
    Operand source_operand{};
    Operand mask_operand{};
    if (!classify(target, target_operand) || target_operand.owner == nullptr) {
        return FALSE;
    }

    Pdev* device = target_operand.owner;
    COLORADJUSTMENT* original = nullptr;
    ULONG orientation = 0;
    Attempt attempt = Attempt::unsupported;
    if (Rx7900StretchGuardUnwrap(adjustment, &original, &orientation) && classify(source, source_operand) &&
        classify(mask, mask_operand) && mask_operand.owner == nullptr) {
        attempt = accelerate_stretch_rop(*device, target_operand, source_operand, mask, clip, translation,
                                         brush_origin, *destination, *source_rect, brush, rop, orientation);
    }

    if (untouched(attempt)) {
        display::count(device->statistics.fallback_stretch);
        return FALSE;
    }

    return attempt == Attempt::performed ? TRUE : FALSE;
}

ULONG APIENTRY DrvEscape(SURFOBJ* surface, ULONG escape, ULONG input_bytes, PVOID input,
                         ULONG output_bytes, PVOID output) {
    namespace line = rx7900::reactos::radeon_line;
    namespace text = rx7900::reactos::radeon_text;
    namespace opengl = rx7900::reactos::radeon_opengl;

    // ReactOS opengl32 (icdload.c) asks QUERYESCSUPPORT for OPENGL_GETINFO, then
    // OPENGL_GETINFO for the ICD name; both are fixed read-only replies.
    namespace present = rx7900::reactos::radeon_present;
    if (escape == opengl::query_escape_support) {
        uint32_t queried = 0;
        if (input != nullptr && input_bytes == sizeof(queried)) {
            memcpy(&queried, input, sizeof(queried));
        }
        return opengl::supports(input, input_bytes) || queried == present::gabi::process_escape ||
                       queried == present::gabi::present_escape ? 1U : 0U;
    }

    // RGLQ: the RadeonGL ICD's process requests, forwarded byte for byte to the
    // miniport process IOCTL, which decodes them and authenticates the calling
    // NT process (DrvEscape runs in the caller's thread). Never parsed here and
    // never forwarded under any other IOCTL code.
    if (escape == present::gabi::process_escape) {
        if (registry_lock == nullptr ||
            !present::process_escape_admitted(input_bytes, output_bytes, input != nullptr, output != nullptr)) {
            return 0;
        }
        HANDLE driver = nullptr;
        {
            RegistryGuard guard;
            Pdev* device = surface_owner(surface);
            driver = device != nullptr ? device->driver : nullptr;
        }
        // The PDEV outlives this call (GDI holds it); the miniport call may wait
        // on GPU work, so other DDI calls are not blocked behind it.
        DWORD returned = 0;
        const bool ok = driver != nullptr &&
                        EngDeviceIoControl(driver, present::pabi::ioctl, input, input_bytes, output,
                                           output_bytes, &returned) == 0;
        return present::process_reply_valid(ok, returned) ? returned : 0;
    }
    if (escape == opengl::opengl_getinfo) {
        return opengl::encode_info(input, input_bytes, output, output_bytes);
    }

    // A read-only positive identity lets the pinned win32k helper discover the
    // private kernel-only Notify callback; no pointers are accepted in Escape.
    if (escape == text::identity_escape) {
        if (input_bytes != 0 || output == nullptr || output_bytes != sizeof(text::Capability) ||
            registry_lock == nullptr) {
            return 0;
        }
        RegistryGuard guard;
        Operand target{};
        if (!classify(surface, target) || target.owner == nullptr ||
            !raster_available(*target.owner, abi::ioctl_text) || !gpu_ready(*target.owner, target)) {
            return 0;
        }
        const text::Capability capability{};
        memcpy(output, &capability, sizeof(capability));
        return sizeof(capability);
    }

    // The paired kernel helpers (framebuffer_stretch_guard, radeon_bridge,
    // radeon_devbitmap_bridge): one fixed capability reply for our own surfaces,
    // so they may tag unstyled lines, guarded stretches, offer styled strokes and
    // compatible bitmaps. The optional input is one request mask; request bits
    // are reported only when named in it.
    if (escape == line::identity_escape) {
        uint32_t requested = 0;
        if (!line::decode_request(input_bytes, input, requested) || output == nullptr ||
            output_bytes != sizeof(line::Identity) || registry_lock == nullptr) {
            return 0;
        }

        RegistryGuard guard;
        const Pdev* device = surface_owner(surface);
        if (device == nullptr) {
            return 0;
        }

        // flag_gpu_stretch: DrvStretchBltROP unwraps the guard and the exact
        // stretch raster operation is available on this session.
        // flag_styled_stroke: DrvStrokePath reuses the line operation's style mask.
        // flag_device_bitmaps: DrvCreateDeviceBitmap can admit offscreen surfaces
        // on this session (the miniport granted them with eviction); a later
        // refusal still lets the bridge keep its engine bitmap.
        const bool lines = raster_available(*device, abi::ioctl_line);
        const bool device_bitmaps = session_usable(*device) && device->bitmap_limits.max_count != 0;
        const line::Identity reply = line::make_identity(lines, raster_available(*device, abi::ioctl_stretch_rop),
                                                         lines, device_bitmaps, requested);
        memcpy(output, &reply, sizeof(reply));
        return sizeof(reply);
    }

    if (escape != display::statistics_escape || input_bytes != 0 || output == nullptr ||
        output_bytes < display::statistics_bytes || registry_lock == nullptr) {
        return 0;
    }

    RegistryGuard guard;
    Pdev* device = surface_owner(surface);
    if (device == nullptr) {
        return 0;
    }

    display::Statistics& statistics = device->statistics;
    statistics.accelerated = device->accelerated ? 1U : 0U;
    statistics.poisoned = device->poisoned ? 1U : 0U;
    statistics.live_bitmaps = device->bitmap_count;
    statistics.live_bitmap_bytes = device->resident_bytes;
    // Encoded into the PDEV's reply workspace (off the kernel stack).
    static_assert(display::statistics_bytes <= sizeof(device->request), "Statistics workspace");
    if (!display::encode_statistics(CommandBuffer(device->request, display::statistics_bytes), statistics)) {
        return 0;
    }

    // GDI probes and owns output_bytes of output for this synchronous call.
    memcpy(output, device->request, display::statistics_bytes);
    return display::statistics_bytes;
}

VOID APIENTRY DrvNotify(SURFOBJ* surface, ULONG notification, PVOID data) {
    namespace text = rx7900::reactos::radeon_text;
    if (notification != text::glyph_notify || registry_lock == nullptr ||
        surface == nullptr || data == nullptr) {
        return;
    }
    auto* glyph = static_cast<text::GlyphRequest*>(data);
    if (glyph->size != sizeof(*glyph) || glyph->version != text::version) {
        return;
    }
    glyph->result = text::unsupported;
    // The pinned IntEngMaskBlt first intersects the glyph with pco->rclBounds.
    // This engine marks only a degenerate (empty) clip DC_TRIVIAL, so the
    // original path draws nothing there: leave it, and a missing clip, to it.
    if (glyph->clip == nullptr || glyph->clip->iDComplexity == DC_TRIVIAL) {
        return;
    }
    RegistryGuard guard;
    Operand target{};
    if (!classify(surface, target) || target.owner == nullptr) {
        return;
    }
    const Attempt attempt = accelerate_glyph(*target.owner, target, *glyph);
    glyph->result = attempt == Attempt::performed ? text::performed :
                    untouched(attempt) ? text::unsupported : text::failed;
}

// Pointer DDI: the logic is in radeon_cursor_client; a decline lets the engine
// draw the pointer (EngSetPointerShape).
ULONG APIENTRY DrvSetPointerShape(SURFOBJ* surface, SURFOBJ* mask, SURFOBJ* color, XLATEOBJ* translation,
                                  LONG hot_x, LONG hot_y, LONG x, LONG y, RECTL* bounds, FLONG flags) {
    if (registry_lock == nullptr) {
        return SPS_DECLINE;
    }

    RegistryGuard guard;
    Pdev* device = pointer_owner(surface);
    return device != nullptr ?
        rx7900::reactos::radeon_cursor::set_pointer_shape(device->pointer, device->driver, mask, color,
                                                          translation, hot_x, hot_y, x, y, bounds, flags) :
        SPS_DECLINE;
}

VOID APIENTRY DrvMovePointer(SURFOBJ* surface, LONG x, LONG y, RECTL* bounds) {
    if (registry_lock == nullptr) {
        return;
    }

    RegistryGuard guard;
    Pdev* device = pointer_owner(surface);
    if (device != nullptr) {
        rx7900::reactos::radeon_cursor::move_pointer(device->pointer, device->driver, x, y, bounds);
    }
}

VOID APIENTRY DrvDisableDriver(VOID) {
    // The engine retires every PDEV and surface before unloading the driver.
    if (registry == nullptr && registry_lock != nullptr) {
        EngDeleteSemaphore(registry_lock);
        registry_lock = nullptr;
    }
}

BOOL APIENTRY DrvEnableDriver(ULONG, ULONG bytes, DRVENABLEDATA* data) {
    if (data == nullptr || bytes < sizeof(DRVENABLEDATA)) {
        return FALSE;
    }

    if (registry_lock == nullptr) {
        registry_lock = EngCreateSemaphore();
        if (registry_lock == nullptr) {
            return FALSE;
        }
    }

    // Filled at run time: function-pointer casts are not constant expressions,
    // and kernel DLL code must not depend on dynamic static initialization.
    // Further entries are appended to this list (the count follows the list).
    const DRVFN entries[] = {
        {INDEX_DrvEnablePDEV, ddi_function(DrvEnablePDEV)},
        {INDEX_DrvCompletePDEV, ddi_function(DrvCompletePDEV)},
        {INDEX_DrvDisablePDEV, ddi_function(DrvDisablePDEV)},
        {INDEX_DrvEnableSurface, ddi_function(DrvEnableSurface)},
        {INDEX_DrvDisableSurface, ddi_function(DrvDisableSurface)},
        {INDEX_DrvAssertMode, ddi_function(DrvAssertMode)},
        {INDEX_DrvGetModes, ddi_function(DrvGetModes)},
        {INDEX_DrvCreateDeviceBitmap, ddi_function(DrvCreateDeviceBitmap)},
        {INDEX_DrvDeleteDeviceBitmap, ddi_function(DrvDeleteDeviceBitmap)},
        {INDEX_DrvBitBlt, ddi_function(DrvBitBlt)},
        {INDEX_DrvCopyBits, ddi_function(DrvCopyBits)},
        {INDEX_DrvFillPath, ddi_function(DrvFillPath)},
        {INDEX_DrvRealizeBrush, ddi_function(DrvRealizeBrush)},
        {INDEX_DrvTransparentBlt, ddi_function(DrvTransparentBlt)},
        {INDEX_DrvAlphaBlend, ddi_function(DrvAlphaBlend)},
        {INDEX_DrvGradientFill, ddi_function(DrvGradientFill)},
        {INDEX_DrvLineTo, ddi_function(DrvLineTo)},
        {INDEX_DrvStrokePath, ddi_function(DrvStrokePath)},
        {INDEX_DrvStretchBlt, ddi_function(DrvStretchBlt)},
        {INDEX_DrvStretchBltROP, ddi_function(DrvStretchBltROP)},
        {INDEX_DrvSynchronizeSurface, ddi_function(DrvSynchronizeSurface)},
        {INDEX_DrvEscape, ddi_function(DrvEscape)},
        {INDEX_DrvDrawEscape, ddi_function(DrvDrawEscape)},
        {INDEX_DrvNotify, ddi_function(DrvNotify)},
        {INDEX_DrvDisableDriver, ddi_function(DrvDisableDriver)},
        {INDEX_DrvSetPointerShape, ddi_function(DrvSetPointerShape)},
        {INDEX_DrvMovePointer, ddi_function(DrvMovePointer)},
    };
    constexpr uint32_t count = sizeof(entries) / sizeof(entries[0]);
    static_assert(count <= driver_function_capacity, "DDI table capacity");
    for (uint32_t index = 0; index < count; ++index) {
        driver_functions[index] = entries[index];
    }

    data->c = count;
    data->pdrvfn = driver_functions;
    data->iDriverVersion = DDI_DRIVER_VERSION_NT5;
    return TRUE;
}

} // extern "C"

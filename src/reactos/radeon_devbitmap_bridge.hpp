/*
Module: rx7900.reactos.radeon_devbitmap_bridge
Purpose: Win32k bridge that makes compatible bitmaps of an identified radeondisp PDEV
driver device bitmaps (DrvCreateDeviceBitmap) with exact engine-wide pixel access,
exactly-once DrvDeleteDeviceBitmap and conversion before a dynamic mode change.
Created: 2026-10-24
Architecture: Compiled into win32k from this tree (RX7900_RADEON_DEVICE_BITMAPS,
reactos-radeon-devbitmap.patch). The patch's C glue (eng/surface.c) supplies one
RX7900_DEVBITMAP_ENGINE table of thin adapters over the private SURFACE/PDEVOBJ
objects; this module owns the policy: identity gating through the read-only
radeon_line_identity.hpp escape (flag_device_bitmaps), validation of the driver
result, a fixed-capacity registry, device-lock discipline, engine shadows for direct
pixel readers/writers and in-place conversion to an engine bitmap. Every decline
leaves the original engine bitmap path. Include after the native GDI SDK types, in
either C glue or C++.
The same module owns the generic flood fill of a target without bits (an opaque
device surface or a device bitmap; RX7900_FLOODFILL_OPAQUE): the pinned
NtGdiExtFloodFill runs the DIB flood fill on the SURFOBJ directly, which reads a
NULL pvScan0. The glue (ntgdi/fillshap.c) supplies an RX7900_FLOODFILL_ENGINE.
*/
#pragma once

// Bridged device bitmaps tracked at once (all PDEVs). A full registry declines
// further creation, so GDI creates engine bitmaps exactly as before.
enum { RX7900_DEVBITMAP_CAPACITY = 256 };

// One PDEV as seen by the bridge, filled by the glue under the PDEV's device
// lock. palette is the palette object EngAssociateSurface gives this PDEV's
// surfaces (compared by identity only). Function pointers are the PDEV's
// current driver functions.
typedef struct _RX7900_DEVBITMAP_DEVICE {
    DHPDEV dhpdev;
    SURFOBJ* primary;
    PVOID palette;
    BOOL display;
    BOOL disabled;
    ULONG acceleration_level;
    PFN_DrvEscape escape;
    PFN_DrvCreateDeviceBitmap create;
    PFN_DrvDeleteDeviceBitmap destroy;
    PFN_DrvCopyBits copy;
} RX7900_DEVBITMAP_DEVICE;

// Engine services. Surfaces are passed as their SURFOBJ; the glue maps them
// back to the private SURFACE. Device locks are the PDEV hsemDevLock (recursive);
// the registry lock is taken only inside a device lock and calls nothing.
typedef struct _RX7900_DEVBITMAP_ENGINE {
    FLONG hook_flags;     // HOOK_FLAGS: hooks a converted bitmap loses
    FLONG marker;         // SURFACE flag naming a bridged device bitmap
    FLONG api_bitmap;     // API_BITMAP
    USHORT pool_bits;     // fjBitmap flag: SURFACE_vCleanup frees pvBits with EngFreeMem
    BOOL (APIENTRY* lock_registry)(VOID);
    VOID (APIENTRY* unlock_registry)(VOID);
    VOID (APIENTRY* lock_device)(HDEV hdev);
    VOID (APIENTRY* unlock_device)(HDEV hdev);
    VOID (APIENTRY* reference_device)(HDEV hdev);
    VOID (APIENTRY* release_device)(HDEV hdev);
    BOOL (APIENTRY* describe_device)(HDEV hdev, RX7900_DEVBITMAP_DEVICE* device);
    // Shared reference by handle; delete_bitmap marks the object deleted and
    // consumes that reference.
    SURFOBJ* (APIENTRY* lock_bitmap)(HBITMAP bitmap);
    VOID (APIENTRY* unlock_bitmap)(SURFOBJ* surface);
    VOID (APIENTRY* delete_bitmap)(SURFOBJ* surface);
    // Atomic access to the SURFACE flags.
    FLONG (APIENTRY* read_flags)(SURFOBJ* surface);
    VOID (APIENTRY* update_flags)(SURFOBJ* surface, FLONG set, FLONG clear);
    PVOID (APIENTRY* palette)(SURFOBJ* surface);
    // Gives a new bitmap to the calling process, selected into no DC.
    VOID (APIENTRY* adopt_bitmap)(SURFOBJ* surface);
    // A referenced, zeroed, bottom-up engine API bitmap with the size, format and
    // palette of like, owned by the calling process (GreCreateBitmapEx layout).
    SURFOBJ* (APIENTRY* create_shadow)(SURFOBJ* like);
    // EngCopyBits with no clip and no translation; it dispatches DrvCopyBits
    // for a device bitmap operand. rect and origin may be modified.
    BOOL (APIENTRY* copy_bits)(SURFOBJ* target, SURFOBJ* source, RECTL* rect, POINTL* origin);
    // EngAllocMem/EngFreeMem storage for a converted bitmap's bits.
    PVOID (APIENTRY* allocate_bits)(ULONG bytes);
    VOID (APIENTRY* free_bits)(PVOID bits);
    // Next SURFOBJ iUniq value.
    ULONG (APIENTRY* next_unique)(VOID);
} RX7900_DEVBITMAP_ENGINE;

// Engine services of the opaque flood fill (ntgdi/fillshap.c glue).
typedef struct _RX7900_FLOODFILL_ENGINE {
    // A referenced, zeroed engine bitmap with the size and format of like
    // (SURFACE_AllocSurface layout); delete_shadow deletes it.
    SURFOBJ* (APIENTRY* create_shadow)(SURFOBJ* like);
    VOID (APIENTRY* delete_shadow)(SURFOBJ* shadow);
    // EngCopyBits with no clip and no translation (DrvCopyBits or DrvBitBlt for
    // an opaque operand). rect and origin may be modified.
    BOOL (APIENTRY* copy_bits)(SURFOBJ* target, SURFOBJ* source, RECTL* rect, POINTL* origin);
    // DIB_XXBPP_FloodFillSolid of the pinned engine.
    BOOL (APIENTRY* fill)(SURFOBJ* target, BRUSHOBJ* brush, RECTL* rect, POINTL* origin, ULONG color,
                          UINT fill_type);
} RX7900_FLOODFILL_ENGINE;

#ifdef __cplusplus
extern "C" {
#define RX7900_DEVBITMAP_NOEXCEPT noexcept
#else
#define RX7900_DEVBITMAP_NOEXCEPT
#endif

// TRUE for a live bridged device bitmap (one atomic flag read; no lock).
BOOL APIENTRY Rx7900DevBitmapIsBridged(const RX7900_DEVBITMAP_ENGINE* engine,
    SURFOBJ* surface) RX7900_DEVBITMAP_NOEXCEPT;

// CreateCompatibleBitmap on the PDEV hdev: under its device lock, a positively
// identified radeondisp is offered DrvCreateDeviceBitmap(size, format). palette
// is the palette the engine bitmap would get, or NULL for the PDEV palette; a
// different palette declines before the driver is called. Returns a process-owned
// bridged HBITMAP holding one PDEV reference, or NULL: the caller then creates
// its engine bitmap unchanged. A driver result the engine cannot use exactly is
// deleted (DrvDeleteDeviceBitmap once) and declined.
HBITMAP APIENTRY Rx7900DevBitmapCreate(const RX7900_DEVBITMAP_ENGINE* engine, HDEV hdev, SIZEL size,
    ULONG format, PVOID palette) RX7900_DEVBITMAP_NOEXCEPT;

// SURFACE_vCleanup: a bridged bitmap not converted before calls DrvDeleteDeviceBitmap
// exactly once under its device lock and releases its PDEV reference.
VOID APIENTRY Rx7900DevBitmapCleanup(const RX7900_DEVBITMAP_ENGINE* engine,
    SURFOBJ* surface) RX7900_DEVBITMAP_NOEXCEPT;

// Device lock for a driver call outside a DC: returns the locked HDEV of a
// bridged bitmap (still bridged while held) or NULL (engine bitmap, no lock).
HDEV APIENTRY Rx7900DevBitmapLock(const RX7900_DEVBITMAP_ENGINE* engine,
    SURFOBJ* surface) RX7900_DEVBITMAP_NOEXCEPT;
VOID APIENTRY Rx7900DevBitmapUnlock(const RX7900_DEVBITMAP_ENGINE* engine, HDEV hdev) RX7900_DEVBITMAP_NOEXCEPT;

// Engine shadow of surface for a direct pixel reader/writer. read (clipped to the
// bitmap) is copied in through DrvCopyBits under the device lock; NULL reads
// nothing (zeroed shadow). The shadow is referenced; Discard deletes it, Keep
// returns its handle and drops the reference (BITMAP_CopyBitmap).
SURFOBJ* APIENTRY Rx7900DevBitmapShadow(const RX7900_DEVBITMAP_ENGINE* engine, SURFOBJ* surface,
    const RECTL* read) RX7900_DEVBITMAP_NOEXCEPT;
VOID APIENTRY Rx7900DevBitmapDiscard(const RX7900_DEVBITMAP_ENGINE* engine, SURFOBJ* shadow) RX7900_DEVBITMAP_NOEXCEPT;
HBITMAP APIENTRY Rx7900DevBitmapKeep(const RX7900_DEVBITMAP_ENGINE* engine, SURFOBJ* shadow) RX7900_DEVBITMAP_NOEXCEPT;

// Writes rect (clipped) of shadow back into surface through DrvCopyBits under
// the device lock. An empty rect writes nothing and succeeds.
BOOL APIENTRY Rx7900DevBitmapStore(const RX7900_DEVBITMAP_ENGINE* engine, SURFOBJ* surface, SURFOBJ* shadow,
    const RECTL* rect) RX7900_DEVBITMAP_NOEXCEPT;

// In-place conversion to an engine bitmap with the pinned DDB layout (bottom-up,
// 32-bit aligned rows, pool bits): readback, DrvDeleteDeviceBitmap once, hooks
// cleared, PDEV reference released. TRUE also for an engine bitmap. FALSE leaves
// the device bitmap unchanged and tracked.
BOOL APIENTRY Rx7900DevBitmapConvert(const RX7900_DEVBITMAP_ENGINE* engine,
    SURFOBJ* surface) RX7900_DEVBITMAP_NOEXCEPT;

// Converts every bridged bitmap of hdev (PDEVOBJ_bDynamicModeChange). FALSE at
// the first failure; earlier bitmaps stay converted, later ones unchanged.
BOOL APIENTRY Rx7900DevBitmapConvertDevice(const RX7900_DEVBITMAP_ENGINE* engine,
    HDEV hdev) RX7900_DEVBITMAP_NOEXCEPT;

// Rows SetBitmapBits/GetBitmapBits move for bytes bytes of a size/format bitmap
// (WIDTH_BYTES_ALIGN16 rows from the top): rows is every row touched, partial
// the one partially written row (empty when the last row is whole). FALSE for
// an unknown format or invalid size (both rectangles empty).
BOOL APIENTRY Rx7900DevBitmapRows(ULONG bytes, SIZEL size, ULONG format, RECTL* rows,
    RECTL* partial) RX7900_DEVBITMAP_NOEXCEPT;

// NtGdiExtFloodFill on a target whose SURFOBJ has no bits (pvScan0 NULL). The
// caller holds the target's device lock (DC_vPrepareDCsForBlit: the display
// DC's PDEV lock, or a bridged bitmap's). rect (the fill bounds, clipped to the
// target) is read into an engine shadow through EngCopyBits, the pinned DIB fill
// runs there and the same rectangle is written back, so every pixel equals the
// DIB fill of an equivalent engine bitmap; pixels the fill leaves are stored
// unchanged. Returns the fill's result (a fill stopped part way still stores
// what the DIB fill left); FALSE without any write when the target cannot be
// read or written through the driver (no CopyBits/BitBlt hook in hooks, unknown
// format, origin outside the clipped rect, shadow or readback failure).
BOOL APIENTRY Rx7900FloodFillOpaque(const RX7900_FLOODFILL_ENGINE* engine, SURFOBJ* target, FLONG hooks,
    BRUSHOBJ* brush, const RECTL* rect, const POINTL* origin, ULONG color, UINT fill_type) RX7900_DEVBITMAP_NOEXCEPT;

#ifdef __cplusplus
}
#endif
#undef RX7900_DEVBITMAP_NOEXCEPT

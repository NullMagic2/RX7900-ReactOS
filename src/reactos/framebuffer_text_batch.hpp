/*
Module: lightning.reactos.text_batch
Purpose: Scope engine glyph rendering into the existing owned cached bitmap.
Created: 2026-10-21
Architecture: Private kernel-only DrvNotify contract; never an IOCTL or mutable
user Escape. Include after native GDI SDK types, in either C glue or C++.
License: GPL-2.0-or-later.
*/
#pragma once

#define RX7900_TEXT_BATCH_VERSION 2U
#define RX7900_TEXT_BATCH_MAGIC 0x3154424cU
#define RX7900_TEXT_BATCH_QUERY 0x4c475451U
#define RX7900_TEXT_BATCH_BEGIN 0x4c475442U
#define RX7900_TEXT_BATCH_END 0x4c475445U
#define RX7900_TEXT_BATCH_GLYPH 0x4c475447U
#define RX7900_TEXT_GLYPH_UNSUPPORTED 0U
#define RX7900_TEXT_GLYPH_PERFORMED 1U
#define RX7900_TEXT_GLYPH_FAILED 2U

typedef struct _RX7900_TEXT_BATCH_CAPABILITY {
    ULONG size;
    ULONG version;
    ULONG magic;
    ULONG context_bytes;
} RX7900_TEXT_BATCH_CAPABILITY;

// Engine-owned stack context. The cached pointer is borrowed solely between a
// successful BEGIN and END; the driver retains its surface registry semaphore.
// owner is driver-private and is never dereferenced by the engine helper.
typedef struct _RX7900_TEXT_BATCH_CONTEXT {
    ULONG size;
    ULONG version;
    ULONG magic;
    ULONG active;
    SURFOBJ* original;
    SURFOBJ* cached;
    PVOID owner;
    PFN_DrvNotify notify;
    RECTL dirty;
    RECTL clip_bounds;
    ULONG clipped;
    ULONG result;
} RX7900_TEXT_BATCH_CONTEXT;

// Kernel stack request, borrowed only during the matching Notify callback.
// It is never accepted through user Escape or serialized into a device IOCTL.
typedef struct _RX7900_TEXT_GLYPH_REQUEST {
    ULONG size;
    ULONG version;
    RX7900_TEXT_BATCH_CONTEXT* context;
    SURFOBJ* mask;
    CLIPOBJ* clip;
    RECTL destination;
    POINTL mask_origin;
    BRUSHOBJ* brush;
    ULONG result;
} RX7900_TEXT_GLYPH_REQUEST;

#ifdef __cplusplus
extern "C" {
#define RX7900_TEXT_NOEXCEPT noexcept
#else
#define RX7900_TEXT_NOEXCEPT
#endif
SURFOBJ* APIENTRY Rx7900TextBatchBegin(RX7900_TEXT_BATCH_CONTEXT* context,
    SURFOBJ* surface,CLIPOBJ* clip,PFN_DrvEscape escape,PFN_DrvNotify notify) RX7900_TEXT_NOEXCEPT;
VOID APIENTRY Rx7900TextBatchRecord(RX7900_TEXT_BATCH_CONTEXT* context,
    const RECTL* rectangle) RX7900_TEXT_NOEXCEPT;
BOOL APIENTRY Rx7900TextBatchEnd(RX7900_TEXT_BATCH_CONTEXT* context) RX7900_TEXT_NOEXCEPT;
ULONG APIENTRY Rx7900TextBatchGlyph(RX7900_TEXT_BATCH_CONTEXT* context,SURFOBJ* mask,
    CLIPOBJ* clip,const RECTL* destination,const POINTL* mask_origin,
    BRUSHOBJ* brush) RX7900_TEXT_NOEXCEPT;
#ifdef __cplusplus
}
#endif
#undef RX7900_TEXT_NOEXCEPT

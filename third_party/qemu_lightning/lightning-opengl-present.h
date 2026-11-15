/*
 * Module: lightning.opengl.window_present_contract
 * Purpose: Share a pointer-free imported-image DrawEscape request.
 * Created: 2026-10-22
 * Architecture: USER/GDI own placement/clipping; native transport owns leases.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_MISC_LIGHTNING_OPENGL_PRESENT_H
#define HW_MISC_LIGHTNING_OPENGL_PRESENT_H
#include <stdint.h>
/* Contract revision of this header. Revision 2 adds the asynchronous request
 * and the receipt query below; the version 1 request keeps its exact meaning
 * and every peer that predates revision 2 keeps using it unchanged.
 */
#define LIGHTNING_OPENGL_PRESENT_CONTRACT_VERSION 2u
#define LIGHTNING_OPENGL_PRESENT_ESCAPE 0x4c474450u
#define LIGHTNING_OPENGL_PRESENT_VERSION 1u
#define LIGHTNING_OPENGL_PRESENT_BYTES 48u
#define LIGHTNING_OPENGL_PRESENT_SUCCESS 1u
#define LIGHTNING_OPENGL_PRESENT_FAILED 0u
#define LIGHTNING_OPENGL_PRESENT_UNSUPPORTED UINT32_MAX
/* x/y are client-relative pixels. The driver derives surface placement and
 * intersects the authoritative GDI clip. Width/height cover the leading source
 * rectangle; scaling, source pointers and application-supplied clips are absent.
 * An opaque lease belongs to the originating process/session and cannot be a
 * native address. SUCCESS means source reads actually retired, not UI vblank.
 */
typedef struct LightningOpenGLPresent {
    uint32_t size;
    uint32_t version;
    uint64_t lease;
    uint32_t image_width;
    uint32_t image_height;
    uint32_t image_stride;
    uint32_t image_offset;
    int32_t x;
    int32_t y;
    uint32_t width;
    uint32_t height;
} LightningOpenGLPresent;

/* Revision 2 asynchronous request: the leading request with size
 * LIGHTNING_OPENGL_PRESENT_ASYNC_BYTES and version
 * LIGHTNING_OPENGL_PRESENT_ASYNC_VERSION. Send it only after
 * QUERYESCSUPPORT reports LIGHTNING_OPENGL_RECEIPT_ESCAPE; an older driver
 * answers UNSUPPORTED without side effects.
 *
 * FLAG_PRODUCER names the producer sync point: the opaque renderer sync token
 * of the same session that the submission writing this image signals, and the
 * value it signals. The driver then waits for that submission only, instead of
 * all GPU work queued so far. Zero or an unknown token keeps the full wait.
 *
 * FLAG_ASYNC lets the driver return QUEUED once the clipped copy is accepted.
 * The lease then stays pinned until the copy's GPU source read retires; the
 * caller must not write the image again before the receipt query reports
 * RETIRED. SUCCESS still means the read already retired (or nothing was
 * visible). The receipt covers GPU execution only: console acceptance, UI
 * consumption and physical scanout remain separate, later events.
 */
#define LIGHTNING_OPENGL_PRESENT_ASYNC_VERSION 2u
#define LIGHTNING_OPENGL_PRESENT_ASYNC_BYTES 72u
#define LIGHTNING_OPENGL_PRESENT_QUEUED 2u
#define LIGHTNING_OPENGL_PRESENT_FLAG_ASYNC 1u
#define LIGHTNING_OPENGL_PRESENT_FLAG_PRODUCER 2u
#define LIGHTNING_OPENGL_PRESENT_FLAGS 3u
typedef struct LightningOpenGLPresentAsync {
    LightningOpenGLPresent present;
    uint32_t flags;
    uint32_t reserved;
    uint64_t producer_sync;
    uint64_t producer_value;
} LightningOpenGLPresentAsync;

/* Revision 2 receipt query through ExtEscape on the presenting window DC.
 * Input is LightningOpenGLReceiptQuery for one lease of the calling process;
 * output is LightningOpenGLReceipt. The call never waits for the GPU. RETIRED
 * also answers a lease with no outstanding asynchronous copy. FAILED means the
 * read can no longer be proven retired (device loss or containment). A zero
 * return means no answer is available now; the caller may ask again.
 */
#define LIGHTNING_OPENGL_RECEIPT_ESCAPE 0x4c474452u
#define LIGHTNING_OPENGL_RECEIPT_VERSION 1u
#define LIGHTNING_OPENGL_RECEIPT_QUERY_BYTES 16u
#define LIGHTNING_OPENGL_RECEIPT_BYTES 16u
#define LIGHTNING_OPENGL_RECEIPT_FAILED 0u
#define LIGHTNING_OPENGL_RECEIPT_RETIRED 1u
#define LIGHTNING_OPENGL_RECEIPT_PENDING 2u
typedef struct LightningOpenGLReceiptQuery {
    uint32_t size;
    uint32_t version;
    uint64_t lease;
} LightningOpenGLReceiptQuery;
typedef struct LightningOpenGLReceipt {
    uint32_t size;
    uint32_t version;
    uint32_t status;
    uint32_t reserved;
} LightningOpenGLReceipt;
#endif

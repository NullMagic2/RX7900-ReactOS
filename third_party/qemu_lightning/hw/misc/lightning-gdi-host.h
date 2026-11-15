/*
 * Module: lightning.gdi.host_boundary
 * Purpose: Execute validated GDI batches in the existing console GL context.
 * Created: 2026-10-21
 * Architecture: Main-loop/current-context API, never a second GPU/device owner.
 * The protocol owns generation/sequence/payload snapshots; this backend owns
 * one canvas, GPU scratch, real GLsync tickets and bounded upload/readback PBOs.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_MISC_LIGHTNING_GDI_HOST_H
#define HW_MISC_LIGHTNING_GDI_HOST_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "hw/misc/lightning-gdi.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct LightningGdiGpu LightningGdiGpu;
/* BQL and the existing console GL context are held. Lookup validates exact
 * owned context/resource/layout, imports DMA if needed, and returns a borrowed
 * texture. It never accepts a guest GL object name. The source remains pinned
 * through the real copy completion; failure cannot authorize source reuse.
 */
typedef bool (*LightningGdiExternalLookup)(void *opaque,
    const LightningGdiExternalImage *image, uint32_t *texture);
#define LIGHTNING_GDI_GPU_UPLOAD_SLOTS 3
typedef enum LightningGdiGpuResult {
    LIGHTNING_GDI_GPU_READY,
    LIGHTNING_GDI_GPU_PENDING,
    LIGHTNING_GDI_GPU_UNAVAILABLE,
    LIGHTNING_GDI_GPU_INVALID,
    LIGHTNING_GDI_GPU_LOST,
    LIGHTNING_GDI_GPU_BUSY,
    LIGHTNING_GDI_GPU_NO_MEMORY,
} LightningGdiGpuResult;

typedef struct LightningGdiGpuImage {
    uint32_t texture;
    uint32_t framebuffer;
    uint32_t width;
    uint32_t height;
    /* QEMU scanout flag, not the guest's logical row numbering. Our native
     * top row occupies GL physical row0, so GTK must reverse source Y while
     * presenting to a bottom-origin window framebuffer: this flag is false. */
    bool y_0_top;
} LightningGdiGpuImage;

typedef struct LightningGdiGpuDiagnostics {
    uint64_t fill_batches;
    uint64_t copy_batches;
    uint64_t external_image_batches;
    uint64_t upload_batches;
    uint64_t blend_batches;
    uint64_t rop3_batches;
    uint64_t raster_batches;
    uint64_t glyph_batches;
    uint64_t gradient_batches;
    uint64_t uploaded_bytes;
    uint64_t upload_staging_busy;
    uint64_t readback_bytes;
    uint64_t observed_fence;
    uint64_t publication_copied_pixels;
    uint64_t publication_copied_bytes;
    uint64_t publication_full_snapshots;
    uint64_t publication_partial_snapshots;
    uint32_t gl_error;
    char renderer[160];
    char vendor[96];
    char render_node[160];
} LightningGdiGpuDiagnostics;

LightningGdiGpu *lightning_gdi_gpu_new(void);
void lightning_gdi_gpu_set_external_lookup(LightningGdiGpu *gpu,
    LightningGdiExternalLookup lookup, void *opaque);
bool lightning_gdi_gpu_external_available(const LightningGdiGpu *gpu);
/* Read-only reuse of the actual EGL/render-node hardware capability gate. */
bool lightning_gdi_gpu_context_hardware(LightningGdiGpuDiagnostics *diagnostics);
/* The existing EGL context must be current for every GPU method. Successful
 * draws return PENDING with an actual ticket; only poll returning READY proves
 * GPU completion. A failed submission can leave uncertain GPU writes: LOST
 * forbids replay/fallback from a stale CPU image.
 */
LightningGdiGpuResult lightning_gdi_gpu_initialize(LightningGdiGpu *gpu,
    uint32_t width, uint32_t height, uint64_t *ticket);
LightningGdiGpuResult lightning_gdi_gpu_fill(LightningGdiGpu *gpu,
    LightningDisplayRect destination, const LightningDisplayRect *clips,
    uint32_t clip_count, uint32_t bgra, uint64_t *ticket);
LightningGdiGpuResult lightning_gdi_gpu_copy(LightningGdiGpu *gpu,
    LightningDisplayRect destination, uint32_t source_x, uint32_t source_y,
    const LightningDisplayRect *clips, uint32_t clip_count, uint64_t *ticket);
/* Completed Vulkan image enters the SAME canvas through window-visible clips.
 * Source rows use top-left logical coordinates and positive linear BGRA8 pitch.
 * No CPU readback, desktop replacement or window placement is inferred here.
 */
LightningGdiGpuResult lightning_gdi_gpu_external_image(LightningGdiGpu *gpu,
    LightningDisplayRect destination, uint32_t source_x, uint32_t source_y,
    const LightningDisplayRect *clips, uint32_t clip_count,
    const LightningGdiExternalImage *image, uint64_t *ticket);
/* Pixels are immutable host-owned packed BGRA8 rows for the destination bbox.
 * The pointer is consumed during this call and never retained. COPY and BLEND
 * snapshot the complete source bbox on the GPU before applying any clips.
 * Upload storage is three bounded unpack buffers with disjoint append ranges.
 * BUSY occurs before mutation if no range fits. A buffer rewinds only after
 * observing its latest use retire; no synchronous GPU wait is used.
 */
LightningGdiGpuResult lightning_gdi_gpu_upload(LightningGdiGpu *gpu,
    LightningDisplayRect destination, const LightningDisplayRect *clips,
    uint32_t clip_count, const uint8_t *pixels, size_t pixel_bytes, uint64_t *ticket);
LightningGdiGpuResult lightning_gdi_gpu_blend(LightningGdiGpu *gpu,
    LightningDisplayRect destination, const LightningDisplayRect *clips,
    uint32_t clip_count, const uint8_t *pixels, size_t pixel_bytes,
    uint32_t constant_alpha, uint32_t flags, uint64_t *ticket);
/* Exact Boolean truth table on all four raw BGRA8 bytes. Source is NONE,
 * same-canvas SCREEN or immutable packed UPLOAD as defined by the shared
 * protocol. Both screen source and required destination are snapshotted once
 * before disjoint clips, preserving overlap. Pattern is a solid raw BGRA word. */
LightningGdiGpuResult lightning_gdi_gpu_rop3(LightningGdiGpu *gpu,
    LightningDisplayRect destination, const LightningDisplayRect *clips,
    uint32_t clip_count, uint32_t source_x, uint32_t source_y,
    const uint8_t *pixels, size_t pixel_bytes, uint32_t source_flags,
    uint32_t pattern_bgra, uint32_t table, uint64_t *ticket);
/* Typed, shared-validated ROP4/mask/tile/coverage, mirrored COLORONCOLOR,
 * source-key transparency and exact integer alpha raster. Shared mapping owns
 * per-clip endpoint rounding in snapshot-local coordinates; source-space key
 * masks follow the same checked sample. Guarded mirrors snapshot the available
 * preceding row/column, exclude unavailable dummy edges before texture fetch,
 * and preserve D for invalid SRCCOPY samples or use S=0 for other ROPs. Payload
 * spans refer only to this immutable host buffer. No pointer is retained after submission. */
LightningGdiGpuResult lightning_gdi_gpu_raster(LightningGdiGpu *gpu,
    LightningDisplayRect destination, const LightningDisplayRect *clips,
    uint32_t clip_count, const LightningGdiRasterOperands *operands,
    const uint8_t *payload, size_t payload_bytes, uint64_t *ticket);
/* Exact pinned ReactOS integer rectangle/triangle recurrence. Operands contain
 * bounded vertices, full16 colors, origin and extents; no pixel payload is read.
 * This uses the same canvas, context, clip validation and actual fence owner. */
LightningGdiGpuResult lightning_gdi_gpu_gradient(LightningGdiGpu *gpu,
    LightningDisplayRect destination, const LightningDisplayRect *clips,
    uint32_t clip_count, const LightningGdiGradientOperands *operands, uint64_t *ticket);
LightningGdiGpuResult lightning_gdi_gpu_poll(LightningGdiGpu *gpu, uint64_t ticket);
LightningGdiGpuResult lightning_gdi_gpu_checkpoint(LightningGdiGpu *gpu, uint64_t *ticket);
LightningGdiGpuResult lightning_gdi_gpu_readback_begin(LightningGdiGpu *gpu,
    LightningDisplayRect rectangle, uint64_t *ticket);
/* No GPU wait is hidden in this API: pending transfers return PENDING before
 * mapping. The writable host buffer must have capacity for packed ROI bytes.
 * On failure its contents are unusable; protocol code must not publish them.
 */
LightningGdiGpuResult lightning_gdi_gpu_readback_copy(LightningGdiGpu *gpu,
    uint64_t ticket, uint8_t *output, size_t capacity);
bool lightning_gdi_gpu_image(const LightningGdiGpu *gpu, LightningGdiGpuImage *image);
/* The caller first orders after actual UI-consumer retirement. This GPU copy
 * into the caller-owned publication FBO returns its own real completion ticket;
 * the mutable primary must never be installed directly as the console texture.
 */
LightningGdiGpuResult lightning_gdi_gpu_copy_for_publication(LightningGdiGpu *gpu,
    uint32_t framebuffer, uint32_t width, uint32_t height, uint64_t *ticket);
/* Disjoint validated same-coordinate regions; empty damage is invalid rather
 * than an implicit full-frame copy. No destination write precedes validation. */
LightningGdiGpuResult lightning_gdi_gpu_copy_regions_for_publication(LightningGdiGpu *gpu,
    uint32_t framebuffer, uint32_t width, uint32_t height,
    const LightningDisplayRect *rectangles, uint32_t count, uint64_t *ticket);
bool lightning_gdi_gpu_diagnostics(const LightningGdiGpu *gpu, LightningGdiGpuDiagnostics *diagnostics);
/* Release requires current owning context and observed GPU retirement. BUSY or
 * LOST retains resources. Only after native context destruction may forget be
 * used: it performs no GL calls and cannot assert GPU quiescence by itself.
 */
LightningGdiGpuResult lightning_gdi_gpu_release(LightningGdiGpu *gpu);
/* Same retirement rules, retaining the C owner's reusable metadata wrapper. */
LightningGdiGpuResult lightning_gdi_gpu_reset(LightningGdiGpu *gpu);
void lightning_gdi_gpu_forget_after_context_destroy(LightningGdiGpu *gpu);

#ifdef __cplusplus
}
#endif
#endif

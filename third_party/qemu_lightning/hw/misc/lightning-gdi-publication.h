/*
 * Module: lightning.gdi.publication_boundary
 * Purpose: Retain immutable console images through real consumer fences.
 * Created: 2026-10-21
 * Architecture: One borrowed current console-copy context; bounded three slots.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_MISC_LIGHTNING_GDI_PUBLICATION_H
#define HW_MISC_LIGHTNING_GDI_PUBLICATION_H
#include "hw/misc/lightning-gdi-controller.h"
#ifdef __cplusplus
extern "C" {
#endif
#define LIGHTNING_GDI_PUBLICATION_CONSUMERS 8

typedef struct LightningGdiPublication LightningGdiPublication;
LightningGdiPublication *lightning_gdi_publication_new(void);
/* Prepare only a real producer sharing token in the existing current context. */
LightningGdiGpuResult lightning_gdi_publication_prepare(LightningGdiPublication *publication);
/* Snapshot is a GPU copy into an independent immutable image. Its private
 * fence survives backend/controller reset and never touches guest storage. */
LightningGdiGpuResult lightning_gdi_publication_begin(LightningGdiPublication *publication,
    LightningGdiGpu *gpu, const LightningGdiFrame *frame);
LightningGdiGpuResult lightning_gdi_publication_poll(LightningGdiPublication *publication);
void lightning_gdi_publication_diagnostics(const LightningGdiPublication *publication, LightningGdiGpuDiagnostics *diagnostics);
void lightning_gdi_publication_error(const LightningGdiPublication *publication, uint32_t *stage, uint32_t *gl_error);
void *lightning_gdi_publication_share_fence(const LightningGdiPublication *publication);
bool lightning_gdi_publication_ready(const LightningGdiPublication *publication,
    LightningGdiGpuImage *image, LightningGdiFrame *frame);
/* Ownership of every returned native GLsync transfers even on failure.
 * Must run after the console switched away from the previous image and after
 * restoring the borrowed copy context. Failure quarantines both images. */
bool lightning_gdi_publication_commit(LightningGdiPublication *publication,
    void *const *fences, size_t count, bool success);
bool lightning_gdi_publication_detach(LightningGdiPublication *publication,
    void *const *fences, size_t count, bool success);
/* Invalidates unpublished snapshots. Actual in-flight copy storage is retained
 * until its private fence signals; no old mode can publish after cancellation. */
void lightning_gdi_publication_cancel(LightningGdiPublication *publication);
bool lightning_gdi_publication_has_current(const LightningGdiPublication *publication);
bool lightning_gdi_publication_pending(const LightningGdiPublication *publication);
/* True only for actual copy/consumer fences needing zero-time GPU polling.
 * A ready immutable image waiting for cadence does not require 1ms wakeups. */
bool lightning_gdi_publication_needs_poll(const LightningGdiPublication *publication);
bool lightning_gdi_publication_idle(const LightningGdiPublication *publication);
/* READY only after actual copy/consumer retirement. Otherwise retains owner. */
LightningGdiGpuResult lightning_gdi_publication_release(LightningGdiPublication *publication);
/* Exceptional final context teardown: delete names without reusing storage.
 * GL's outstanding command/FBO references retain the underlying allocations.
 * Only valid after every consumer switched away (successful detach). */
bool lightning_gdi_publication_delete_detached(LightningGdiPublication *publication);
#ifdef __cplusplus
}
#endif
#endif

/*
 * Module: lightning.gdi.runtime_boundary
 * Purpose: Bind typed GPU work to the existing QEMU main-loop/UI owner.
 * Created: 2026-10-21
 * Architecture: C callbacks contain only QEMU console/context ABI dispatch.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_MISC_LIGHTNING_GDI_RUNTIME_H
#define HW_MISC_LIGHTNING_GDI_RUNTIME_H
#include "hw/misc/lightning-gdi-publication.h"
#ifdef __cplusplus
extern "C" {
#endif

typedef struct LightningGdiRuntime LightningGdiRuntime;
typedef struct LightningGdiUiOps {
    bool (*supported)(void *opaque);
    bool (*make_current)(void *opaque);
    bool (*sharing)(void *opaque, void *producer_fence);
    bool (*publish)(void *opaque, const LightningGdiGpuImage *image,
        const LightningGdiFrame *frame, void *producer_fence, void **fences, size_t capacity, size_t *count,
        uint64_t presentation_serial);
    bool (*detach)(void *opaque, void **fences, size_t capacity, size_t *count);
    /* Optional one-shot first failure; bounded stage strings are implementation constants. */
    void (*failure)(void *opaque, const char *stage, uint32_t result, uint32_t pool_stage, uint32_t pool_gl_error);
    /* Optional level delivery through the existing PCI interrupt aggregate.
     * Acknowledge retriggers another pending source for MSI-X consumers. */
    void (*interrupt)(void *opaque, bool asserted, bool retrigger);
    /* Optional actual-consumer contract. A frame-clock consumer gates the next
     * publication on its actual after-paint receipt, avoiding two pacing clocks. */
    bool (*presentation_supported)(void *opaque);
    bool (*consumer_paced)(void *opaque);
    void (*cancel_presentation)(void *opaque);
} LightningGdiUiOps;
LightningGdiRuntime *lightning_gdi_runtime_new(const LightningGdiUiOps *ops, void *opaque);
/* Main-loop configuration, before admitting external reads. The existing GPU
 * owner preserves this trusted lookup across actual backend resets. */
void lightning_gdi_runtime_set_external_lookup(LightningGdiRuntime *runtime,
    LightningGdiExternalLookup lookup, void *opaque);
bool lightning_gdi_runtime_read(const LightningGdiRuntime *runtime, uint32_t address, uint32_t *value);
bool lightning_gdi_runtime_write(LightningGdiRuntime *runtime, uint32_t address, uint32_t value,
    const uint8_t *bar4, size_t bytes, uint32_t width, uint32_t height);
/* Called on the owning main loop. Returns absolute realtime retry/deadline, or
 * zero when idle. Native driver calls themselves are not preemptible timeouts. */
uint64_t lightning_gdi_runtime_pump(LightningGdiRuntime *runtime, uint64_t now,
    uint32_t refresh_millihz, uint8_t *bar4, size_t bytes);
/* MMIO-safe cancellation: no GL/UI calls. Prevents any old GPU snapshot from
 * publishing after a CPU FULL/DISABLE/mode transition. Main loop retires it. */
void lightning_gdi_runtime_cancel(LightningGdiRuntime *runtime);
/* PCI reset masks and clears interrupt delivery; GPU ownership still retires
 * through cancel/pump and never becomes completion merely from reset. */
void lightning_gdi_runtime_interrupt_reset(LightningGdiRuntime *runtime);
#define LIGHTNING_GDI_CONSUMER_SUCCESS 1u
#define LIGHTNING_GDI_CONSUMER_REJECTED 2u
#define LIGHTNING_GDI_CONSUMER_CANCELLED 3u
#define LIGHTNING_GDI_CONSUMER_REPLAYED 4u
/* Main-loop callback for the exact offered token. This never retires a GPU
 * fence, frees an image, fabricates physical vblank, or acknowledges a new mode. */
void lightning_gdi_runtime_consumed(LightningGdiRuntime *runtime, uint64_t serial, uint32_t result);
bool lightning_gdi_runtime_diagnostics(const LightningGdiRuntime *runtime,
    LightningGdiGpuDiagnostics *diagnostics, uint64_t *published_frames);
/* Main-loop detach first. Returns false retaining uncertain ownership. */
bool lightning_gdi_runtime_release(LightningGdiRuntime *runtime);
#ifdef __cplusplus
}
#endif
#endif

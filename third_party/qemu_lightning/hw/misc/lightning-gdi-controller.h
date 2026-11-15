/*
 * Module: lightning.gdi.controller_boundary
 * Purpose: Connect owned GDI packet snapshots to the existing main-loop GPU owner.
 * Created: 2026-10-21
 * Architecture: BQL serializes all calls; only pump/reset use the current GL context.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_MISC_LIGHTNING_GDI_CONTROLLER_H
#define HW_MISC_LIGHTNING_GDI_CONTROLLER_H
#include "hw/misc/lightning-gdi-host.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct LightningGdiController LightningGdiController;
typedef struct LightningGdiFrame {
    uint32_t mode_generation;
    uint32_t epoch;
    uint32_t revision;
    uint32_t width;
    uint32_t height;
    LightningDisplayRect damage;
    uint32_t damage_count;
    LightningDisplayRect damage_rects[LIGHTNING_GDI_MAX_RECTS];
} LightningGdiFrame;

LightningGdiController *lightning_gdi_controller_new(void);
/* The native owner sets true only after actual GL initialization, shader/FBO
 * validation, real preflight fence completion and reusable-backend reset. */
void lightning_gdi_controller_set_available(LightningGdiController *controller, bool available);
/* Optional same-canvas imported-image path, only after the native DMA lookup
 * is attached to this backend. Does not alter the stable base capabilities. */
void lightning_gdi_controller_set_external_available(LightningGdiController *controller, bool available);
/* Returns false and retains ownership unless reset/context retirement completed. */
bool lightning_gdi_controller_free(LightningGdiController *controller);
bool lightning_gdi_controller_read(const LightningGdiController *controller,
    uint32_t address, uint32_t *value);
/* BAR4 is QEMU-owned mapped RAM, never a guest-supplied host address. Command
 * metadata/pixels are snapshotted before accepted_sequence advances. BIND must
 * match the actual existing scanout dimensions; no pointer is retained here.
 * On a recognized SUBMIT write, C glue schedules its existing main-loop BH.
 */
bool lightning_gdi_controller_write(LightningGdiController *controller,
    uint32_t address, uint32_t value, const uint8_t *bar4, size_t bar4_bytes,
    uint32_t actual_width, uint32_t actual_height);
bool lightning_gdi_controller_needs_work(const LightningGdiController *controller);
/* Existing console GL context must be current on its owning main-loop thread.
 * Real GPU tickets are polled without CPU waits. READBACK completion is published
 * only after bounded PBO pixels are copied to the fixed BAR4 transfer window.
 */
void lightning_gdi_controller_pump(LightningGdiController *controller,
    LightningGdiGpu *gpu, uint8_t *bar4, size_t bar4_bytes);
bool lightning_gdi_controller_frame(const LightningGdiController *controller,
    LightningGdiFrame *frame);
/* Consume only this completed mutation prefix after an actual accepted GPU
 * snapshot. Later completions accumulate separately until its UI receipt. */
bool lightning_gdi_controller_snapshot(LightningGdiController *controller,
    const LightningGdiFrame *frame);
/* Revalidate the exact immutable lease before invoking any UI callback. */
bool lightning_gdi_controller_snapshot_current(const LightningGdiController *controller,
    const LightningGdiFrame *frame);
/* Only after the separate console-copy fence and actual UI publication. Never
 * expose the mutable backend primary as the console texture or claim vblank.
 */
bool lightning_gdi_controller_published(LightningGdiController *controller,
    const LightningGdiFrame *frame, bool success);
/* Reset retains all uncertain work. Pump keeps retiring real tickets then calls
 * reusable backend reset. A false result never authorizes freeing device/GL
 * resources. The backend wrapper is borrowed and is never released by pump.
 */
void lightning_gdi_controller_request_reset(LightningGdiController *controller);
bool lightning_gdi_controller_reset_complete(const LightningGdiController *controller);
/* Exceptional teardown only after the actual owning context was destroyed. */
void lightning_gdi_controller_retire_after_context_destroy(LightningGdiController *controller);

#ifdef __cplusplus
}
#endif
#endif

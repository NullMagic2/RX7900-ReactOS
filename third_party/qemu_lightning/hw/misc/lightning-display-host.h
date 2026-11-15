/*
 * Module: lightning.display.host_boundary
 * Purpose: Expose bounded C++ display state to QEMU's existing C lifetime hooks.
 * Created: 2026-10-21
 * Architecture: Callers hold the BQL. Views remain borrowed until the next
 * submit/reset/free; QEMU must copy them into its separately owned surface.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_MISC_LIGHTNING_DISPLAY_HOST_H
#define HW_MISC_LIGHTNING_DISPLAY_HOST_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "hw/misc/lightning-display.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct LightningDisplayHost LightningDisplayHost;
typedef struct LightningDisplayView {
    const uint8_t *pixels;
    uint32_t width;
    uint32_t height;
    LightningDisplayRect damage;
    bool blank;
} LightningDisplayView;
/* Read-only device-lifetime diagnostics. Reset cancels protocol state but keeps
 * these saturating counters. Byte counts exclude row padding; snapshots count
 * each accepted descriptor read, publications count the copied dirty union.
 */
typedef struct LightningDisplayDiagnostics {
    uint64_t accepted_submissions;
    uint64_t accepted_partial_submissions;
    uint64_t rejected_submissions;
    uint64_t bytes_snapshotted;
    uint64_t bytes_published;
    uint64_t published_frames;
    uint64_t refresh_period_ns;
    uint64_t cursor_publications;
    uint64_t cursor_move_publications;
} LightningDisplayDiagnostics;

LightningDisplayHost *lightning_display_host_new(void);
void lightning_display_host_free(LightningDisplayHost *host);
void lightning_display_host_reset(LightningDisplayHost *host);
bool lightning_display_host_read(const LightningDisplayHost *host, uint32_t address, uint32_t *value);
bool lightning_display_host_write(LightningDisplayHost *host, uint32_t address, uint32_t value);
/* Shared absolute cadence arithmetic for CPU and GPU publication owners. */
uint32_t lightning_display_refresh_period(uint32_t millihz);
uint64_t lightning_display_advance_deadline(uint64_t previous, uint64_t now, uint32_t period);
void lightning_display_host_refresh(LightningDisplayHost *host, uint32_t millihz);
void lightning_display_host_reject(LightningDisplayHost *host, uint32_t result);
bool lightning_display_host_diagnostics(const LightningDisplayHost *host, LightningDisplayDiagnostics *result);
void lightning_display_host_cursor_published(LightningDisplayHost *host, bool shape, bool move);
/* Snapshot descriptors and selected source rows during this call. Source and
 * descriptors must be readable, nonoverlapping with host-owned state, for their
 * supplied byte lengths. Neither pointer is retained. Failure leaves the entire
 * accepted image/prefix unchanged; only RESULT changes. No guest pointer is a
 * host address: the QEMU adapter passes its own mapped RAM-window pointers.
 */
uint32_t lightning_display_host_submit(LightningDisplayHost *host, uint32_t command,
    uint32_t width, uint32_t height, uint32_t stride, uint32_t offset,
    const uint8_t *source, size_t source_bytes, const uint8_t *descriptors,
    size_t descriptor_bytes, uint64_t now_ns);
bool lightning_display_host_pending(const LightningDisplayHost *host);
uint64_t lightning_display_host_deadline(const LightningDisplayHost *host);
bool lightning_display_host_view(const LightningDisplayHost *host, LightningDisplayView *view);
/* Validate the entire separately owned target before writing. Copies only the
 * cumulative dirty region, or the full initialized image on a surface switch.
 * The target must not alias staging; no pointer is retained after this call.
 */
bool lightning_display_host_copy(LightningDisplayHost *host, uint8_t *destination,
    size_t destination_bytes, uint32_t width, uint32_t height, size_t stride,
    bool full, LightningDisplayRect *copied);
/* Call only after main-loop console publication or a real publication failure.
 * Success advances the accepted prefix, failure invalidates the baseline.
 */
void lightning_display_host_complete(LightningDisplayHost *host, bool success, uint64_t now_ns);
/* Return false for background-XOR shapes the console ARGB cursor cannot express.
 * Reuses the existing Rust geometry/conversion contract rather than approximating
 * an inverted pixel. Validation/positions operate on QEMU-owned snapshots only.
 */
bool lightning_display_cursor_exact(uint32_t format, uint32_t width, uint32_t height,
    uint32_t pitch, uint32_t hot_x, uint32_t hot_y, const uint8_t *shape,
    size_t shape_bytes, uint32_t *pixels, size_t pixel_count);
bool lightning_display_cursor_position(uint32_t packed, uint32_t hot_x, uint32_t hot_y,
    int32_t *x, int32_t *y);

/* Independent cursor plane: accepted POSITION is immediate; COMMAND reports
 * the main-loop published enabled state, including fully offscreen shapes.
 * All calls require BQL serialization. A view remains valid until any write.
 * Complete only after console shape/mouse callbacks have returned successfully.
 */
typedef struct LightningCursorHost LightningCursorHost;
typedef struct LightningCursorView {
    uint64_t serial;
    uint32_t width, height, hot_x, hot_y;
    int32_t mouse_x, mouse_y;
    bool visible, shape_changed, mouse_changed;
} LightningCursorView;
typedef struct LightningCursorDiagnostics {
    uint64_t defines, rejects, moves;
    int32_t mouse_x, mouse_y;
    bool shown;
} LightningCursorDiagnostics;
LightningCursorHost *lightning_cursor_host_new(void);
void lightning_cursor_host_free(LightningCursorHost *host);
void lightning_cursor_host_reset(LightningCursorHost *host);
bool lightning_cursor_host_read(const LightningCursorHost *host, uint32_t address, uint32_t *value);
bool lightning_cursor_host_write(LightningCursorHost *host, uint32_t address,
    uint32_t value, const uint8_t *shape, size_t shape_bytes);
void lightning_cursor_host_canvas(LightningCursorHost *host, bool enabled);
bool lightning_cursor_host_view(const LightningCursorHost *host, LightningCursorView *view);
bool lightning_cursor_host_copy(const LightningCursorHost *host, uint64_t serial,
    uint32_t *pixels, size_t count);
bool lightning_cursor_host_complete(LightningCursorHost *host, uint64_t serial);
void lightning_cursor_host_ui_rejected(LightningCursorHost *host);
void lightning_cursor_host_retry(LightningCursorHost *host);
void lightning_cursor_host_detached(LightningCursorHost *host);
/* Host-only acceptance fixture: at most two visible publications are rejected.
 * HIDE can never consume a fault. No guest MMIO exposes this control.
 */
bool lightning_cursor_host_set_faults(LightningCursorHost *host, uint32_t count);
uint32_t lightning_cursor_host_faults(const LightningCursorHost *host);
bool lightning_cursor_host_fault(LightningCursorHost *host, uint64_t serial);
bool lightning_cursor_host_diagnostics(const LightningCursorHost *host, LightningCursorDiagnostics *result);
/* Shared GTK coordinate/pixel adapters. No buffers or UI handles are retained. */
bool lightning_cursor_viewport(int32_t x, int32_t y, uint32_t canvas_width,
    uint32_t canvas_height, uint32_t widget_width, uint32_t widget_height,
    double scale_x, double scale_y, int32_t *window_x, int32_t *window_y);
bool lightning_cursor_canvas_point(double window_x, double window_y, uint32_t canvas_width,
    uint32_t canvas_height, uint32_t widget_width, uint32_t widget_height,
    double scale_x, double scale_y, int32_t *x, int32_t *y);
bool lightning_cursor_rgba(const uint32_t *pixels, size_t count, uint8_t *rgba, size_t bytes);
bool lightning_cursor_scaled_shape(uint32_t width, uint32_t height, uint32_t hot_x,
    uint32_t hot_y, double scale_x, double scale_y, uint32_t maximum_width,
    uint32_t maximum_height, uint32_t *scaled_width, uint32_t *scaled_height,
    uint32_t *scaled_hot_x, uint32_t *scaled_hot_y);

#ifdef __cplusplus
}
#endif
#endif

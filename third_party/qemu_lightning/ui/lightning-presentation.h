/*
 * Module: lightning.ui.presentation_boundary
 * Purpose: Track bounded actual consumer receipts independently of GPU ownership.
 * Created: 2026-10-22
 * Architecture: Main-loop serialized; GTK/console glue owns callbacks and contexts.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef UI_LIGHTNING_PRESENTATION_H
#define UI_LIGHTNING_PRESENTATION_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
#define LIGHTNING_PRESENTATION_CONSUMERS 16u
#define LIGHTNING_PRESENTATION_HISTORY 64u

typedef struct LightningPresentationReceipts LightningPresentationReceipts;
typedef struct LightningPresentationStatus {
    uint64_t offered_serial;
    uint64_t accepted_serial;
    uint64_t consumed_serial;
    uint64_t rejected_serial;
    uint64_t cancelled_serial;
    uint64_t superseded_frames;
    uint32_t active_consumers;
    uint32_t pending_frames;
} LightningPresentationStatus;

LightningPresentationReceipts *lightning_presentation_new(void);
void lightning_presentation_free(LightningPresentationReceipts *owner);
/* IDs never wrap/reuse. A stale callback cannot act as a newly joined listener.
 * Zero rejects an exhausted owner. All calls use the same main-loop owner. */
uint32_t lightning_presentation_join(LightningPresentationReceipts *owner);
void lightning_presentation_leave(LightningPresentationReceipts *owner, uint32_t consumer);
/* Begin before dispatch because an EGL consumer may acknowledge synchronously.
 * Commit only after the console accepted the texture. Neither event by itself
 * establishes consumption. Revisions/epochs remain the graphics device's job.
 * Superseded unconsumed history carries no GPU resource retirement authority. */
bool lightning_presentation_begin(LightningPresentationReceipts *owner,
    uint64_t serial, const uint32_t *consumers, size_t count);
/* Re-enroll the exact still-current accepted texture after a listener change.
 * Consumer IDs and its new rendering callbacks remain independently validated. */
bool lightning_presentation_replay(LightningPresentationReceipts *owner,
    uint64_t serial, const uint32_t *consumers, size_t count);
bool lightning_presentation_commit(LightningPresentationReceipts *owner,
    uint64_t serial, bool accepted);
bool lightning_presentation_consumed(LightningPresentationReceipts *owner,
    uint64_t serial, uint32_t consumer, bool success);
/* Cancellation invalidates pending callbacks but never invents consumption or
 * permits deletion/reuse of any GPU image or consumer synchronization object. */
void lightning_presentation_cancel(LightningPresentationReceipts *owner);
bool lightning_presentation_status(const LightningPresentationReceipts *owner,
    LightningPresentationStatus *status);
#ifdef __cplusplus
}
#endif
#endif

/*
 * Module: lightning.display.contract
 * Purpose: Share the opt-in bounded CPU damage and publication protocol.
 * Created: 2026-10-21
 * Architecture: Supplements the unchanged v1 scanout/cursor registers. The host
 * snapshots all descriptors and pixels before accepting a submission. Accepted
 * means source reads retired; completed means cumulative console publication,
 * never a physical-monitor vblank or every intermediate image being displayed.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_MISC_LIGHTNING_DISPLAY_H
#define HW_MISC_LIGHTNING_DISPLAY_H

#include <stdint.h>

#define LIGHTNING_DISPLAY_MAGIC 0x32504451u
#define LIGHTNING_DISPLAY_VERSION 1u
#define LIGHTNING_DISPLAY_REG_MAGIC 0x360u
#define LIGHTNING_DISPLAY_REG_VERSION 0x364u
#define LIGHTNING_DISPLAY_REG_CAPABILITIES 0x368u
#define LIGHTNING_DISPLAY_REG_RECT_COUNT 0x36cu
#define LIGHTNING_DISPLAY_REG_SUBMITTED_SEQ 0x370u
#define LIGHTNING_DISPLAY_REG_COMPLETED_SEQ 0x374u
#define LIGHTNING_DISPLAY_REG_STATUS 0x378u
#define LIGHTNING_DISPLAY_REG_COMMAND 0x37cu
#define LIGHTNING_DISPLAY_REG_EPOCH 0x380u
#define LIGHTNING_DISPLAY_REG_COMPLETED_EPOCH 0x384u
#define LIGHTNING_DISPLAY_REG_RESULT 0x388u
#define LIGHTNING_DISPLAY_REG_REFRESH_NS 0x38cu
#define LIGHTNING_DISPLAY_REG_ACCEPTED_SEQ 0x390u
#define LIGHTNING_DISPLAY_REG_ACCEPTED_EPOCH 0x394u
#define LIGHTNING_DISPLAY_END 0x398u

#define LIGHTNING_DISPLAY_DAMAGE_OFFSET 0x800u
#define LIGHTNING_DISPLAY_MAX_RECTS 64u
#define LIGHTNING_DISPLAY_RECT_BYTES 16u
#define LIGHTNING_DISPLAY_RECT_X_OFFSET 0u
#define LIGHTNING_DISPLAY_RECT_Y_OFFSET 4u
#define LIGHTNING_DISPLAY_RECT_WIDTH_OFFSET 8u
#define LIGHTNING_DISPLAY_RECT_HEIGHT_OFFSET 12u
#define LIGHTNING_DISPLAY_DAMAGE_BYTES (LIGHTNING_DISPLAY_MAX_RECTS * LIGHTNING_DISPLAY_RECT_BYTES)
#define LIGHTNING_DISPLAY_MAX_PENDING_UPDATES 1024u
#define LIGHTNING_DISPLAY_NANOSECONDS_PER_SECOND 1000000000u

#define LIGHTNING_DISPLAY_COPY_ON_SUBMIT 1u
#define LIGHTNING_DISPLAY_PARTIAL_DAMAGE 2u
#define LIGHTNING_DISPLAY_PACED_UI_COMPLETION 4u
#define LIGHTNING_DISPLAY_COALESCED_UPDATES 8u
#define LIGHTNING_DISPLAY_CAPABILITIES (LIGHTNING_DISPLAY_COPY_ON_SUBMIT | \
    LIGHTNING_DISPLAY_PARTIAL_DAMAGE | LIGHTNING_DISPLAY_PACED_UI_COMPLETION | \
    LIGHTNING_DISPLAY_COALESCED_UPDATES)

#define LIGHTNING_DISPLAY_DISABLE 0u
#define LIGHTNING_DISPLAY_FULL 1u
#define LIGHTNING_DISPLAY_DAMAGE 2u

#define LIGHTNING_DISPLAY_OK 0u
#define LIGHTNING_DISPLAY_BUSY 1u
#define LIGHTNING_DISPLAY_BAD_RECT 2u
#define LIGHTNING_DISPLAY_BAD_SEQUENCE 3u
#define LIGHTNING_DISPLAY_MODE_REQUIRES_FULL 4u
#define LIGHTNING_DISPLAY_NO_MEMORY 5u
#define LIGHTNING_DISPLAY_BAD_GEOMETRY 6u
#define LIGHTNING_DISPLAY_BAD_COMMAND 7u
#define LIGHTNING_DISPLAY_PUBLICATION_FAILED 8u

/* Rectangles use unsigned source coordinates and positive extents. The full
 * list is snapshotted then validated before any pixels or accepted state change.
 * Unknown commands fail. FULL and DISABLE require rect_count zero.
 * Each epoch starts at sequence1; sequences increase without wrapping. After
 * the accepted prefix is completed, a new epoch (previous+1, skipping zero)
 * may start with FULL. Geometry/epoch changes while pending fail BUSY. DISABLE
 * itself is a sequenced, paced blank publication; no later source read occurs.
 * Reset/legacy DISABLE cancels any accepted but unpublished prefix without
 * claiming its publication, and requires a new FULL baseline.
 */
typedef struct LightningDisplayRect {
    uint32_t x;
    uint32_t y;
    uint32_t width;
    uint32_t height;
} LightningDisplayRect;

#endif

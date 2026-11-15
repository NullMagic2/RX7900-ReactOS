/*
 * Module: lightning.gdi.contract
 * Purpose: Bound the shared GPU-canvas command, transfer and fence protocol.
 * Created: 2026-10-21
 * Architecture: Reuses the assigned BAR0/BAR4 and display rectangle contracts.
 * GPU completion and console publication are distinct ownership transitions.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_MISC_LIGHTNING_GDI_H
#define HW_MISC_LIGHTNING_GDI_H

#include <stdint.h>
#include "lightning-memory.h"
#include "lightning-display.h"

#define LIGHTNING_GDI_MAGIC 0x3144474cu
#define LIGHTNING_GDI_VERSION 1u
#define LIGHTNING_GDI_MAX_DIMENSION LIGHTNING_SCANOUT_MAX_DIMENSION
#define LIGHTNING_GDI_PIXEL_BYTES LIGHTNING_SCANOUT_PIXEL_BYTES
#define LIGHTNING_GDI_MAX_RECTS LIGHTNING_DISPLAY_MAX_RECTS
#define LIGHTNING_GDI_MAX_PENDING_FENCES 64u
#define LIGHTNING_GDI_MAX_PENDING_PIXEL_BYTES 67108864u
#define LIGHTNING_GDI_TRANSFER_OFFSET 67108864u
#define LIGHTNING_GDI_TRANSFER_BYTES 16777216u
#define LIGHTNING_GDI_PIXEL_OFFSET 4096u
#define LIGHTNING_GDI_MAX_PIXEL_BYTES (LIGHTNING_GDI_TRANSFER_BYTES - LIGHTNING_GDI_PIXEL_OFFSET)
#define LIGHTNING_GDI_SOURCE_ALPHA 1u
#define LIGHTNING_GDI_SOURCE_32BPP 2u
#define LIGHTNING_GDI_UPLOAD_BEGIN 4u
#define LIGHTNING_GDI_UPLOAD_END 8u
#define LIGHTNING_GDI_ROP_SOURCE_NONE 0u
#define LIGHTNING_GDI_ROP_SOURCE_SCREEN 16u
#define LIGHTNING_GDI_ROP_SOURCE_UPLOAD 32u
#define LIGHTNING_GDI_COMMAND_BYTES 96u
#define LIGHTNING_GDI_RASTER_BYTES 144u
#define LIGHTNING_GDI_GRADIENT_BYTES 88u
#define LIGHTNING_GDI_EXTERNAL_IMAGE_BYTES 48u
#define LIGHTNING_GDI_EXTERNAL_IMAGE_VERSION 1u
#define LIGHTNING_GDI_EXTERNAL_REPLACEABLE 1u
#define LIGHTNING_GDI_GRADIENT_VERSION 1u
#define LIGHTNING_GDI_GRADIENT_RECT_H 0u
#define LIGHTNING_GDI_GRADIENT_RECT_V 1u
#define LIGHTNING_GDI_GRADIENT_TRIANGLE 2u
#define LIGHTNING_GDI_GRADIENT_COORDINATE_LIMIT (2u * LIGHTNING_GDI_MAX_DIMENSION)
#define LIGHTNING_GDI_RASTER_LEGACY_VERSION 2u
#define LIGHTNING_GDI_RASTER_VERSION 3u
#define LIGHTNING_GDI_PATTERN_NONE 0u
#define LIGHTNING_GDI_PATTERN_SOLID 1u
#define LIGHTNING_GDI_PATTERN_TILE 2u
#define LIGHTNING_GDI_MASK_NONE 0u
#define LIGHTNING_GDI_MASK_MONO1 1u
#define LIGHTNING_GDI_MASK_COVERAGE8 2u
#define LIGHTNING_GDI_MASK_SPACE_NONE 0u
#define LIGHTNING_GDI_MASK_SPACE_DESTINATION 1u
#define LIGHTNING_GDI_MASK_SPACE_SOURCE 2u
#define LIGHTNING_GDI_RASTER_MIRROR_X 1u
#define LIGHTNING_GDI_RASTER_MIRROR_Y 2u
#define LIGHTNING_GDI_RASTER_COLOR_KEY 4u
#define LIGHTNING_GDI_RASTER_ALPHA_BLEND 8u
#define LIGHTNING_GDI_RASTER_SOURCE_ALPHA 16u
#define LIGHTNING_GDI_RASTER_SOURCE_32BPP 32u
#define LIGHTNING_GDI_RASTER_GUARDED_SOURCE 64u
#define LIGHTNING_GDI_RASTER_INVALID_LEFT 128u
#define LIGHTNING_GDI_RASTER_INVALID_TOP 256u
#define LIGHTNING_GDI_RASTER_FLAGS 511u
#define LIGHTNING_GDI_STRETCH_NONE 0u
#define LIGHTNING_GDI_STRETCH_COLORONCOLOR 1u
#define LIGHTNING_GDI_CLIP_BYTES LIGHTNING_DISPLAY_RECT_BYTES
#define LIGHTNING_GDI_MAX_METADATA_BYTES (LIGHTNING_GDI_COMMAND_BYTES + LIGHTNING_GDI_RASTER_BYTES + LIGHTNING_GDI_MAX_RECTS * LIGHTNING_GDI_CLIP_BYTES)

#define LIGHTNING_GDI_REG_MAGIC 0x400u
#define LIGHTNING_GDI_REG_VERSION 0x404u
#define LIGHTNING_GDI_REG_CAPABILITIES 0x408u
#define LIGHTNING_GDI_REG_COMMAND 0x40cu
#define LIGHTNING_GDI_REG_STATUS 0x410u
#define LIGHTNING_GDI_REG_RESULT 0x414u
#define LIGHTNING_GDI_REG_ACCEPTED_SEQUENCE 0x418u
#define LIGHTNING_GDI_REG_COMPLETED_SEQUENCE 0x41cu
#define LIGHTNING_GDI_REG_EPOCH 0x420u
#define LIGHTNING_GDI_REG_MODE_GENERATION 0x424u
#define LIGHTNING_GDI_REG_CANVAS_REVISION 0x428u
#define LIGHTNING_GDI_REG_WIDTH 0x42cu
#define LIGHTNING_GDI_REG_HEIGHT 0x430u
#define LIGHTNING_GDI_REG_MAX_PIXEL_BYTES 0x434u
#define LIGHTNING_GDI_REG_READBACK_SEQUENCE 0x438u
#define LIGHTNING_GDI_REG_TRANSFER_BYTES 0x43cu
#define LIGHTNING_GDI_REG_PUBLISHED_SEQUENCE 0x440u
#define LIGHTNING_GDI_REG_PUBLISHED_EPOCH 0x444u
#define LIGHTNING_GDI_REG_PUBLISHED_MODE_GENERATION 0x448u
#define LIGHTNING_GDI_REG_FLAGS 0x44cu
#define LIGHTNING_GDI_REG_RESET_GENERATION 0x450u
/* Optional runtime completion source. Read IRQ returns latched causes; write
 * acknowledges only the supplied bits. ENABLE masks delivery, never the ledger.
 * Old hosts expose no capability and retain the bounded polling contract. */
#define LIGHTNING_GDI_REG_IRQ 0x454u
#define LIGHTNING_GDI_REG_IRQ_ENABLE 0x458u
#define LIGHTNING_GDI_REG_CONSUMED_SEQUENCE 0x45cu
#define LIGHTNING_GDI_REG_CONSUMED_EPOCH 0x460u
#define LIGHTNING_GDI_REG_CONSUMED_MODE_GENERATION 0x464u
#define LIGHTNING_GDI_REG_PRESENTATION_FLAGS 0x468u
#define LIGHTNING_GDI_END 0x46cu
#define LIGHTNING_GDI_PRESENTATION_SUPPORTED 1u
#define LIGHTNING_GDI_PRESENTATION_PENDING 2u
#define LIGHTNING_GDI_PRESENTATION_FAILED 4u
#define LIGHTNING_GDI_PRESENTATION_CONSUMER_CLOCK 8u
#define LIGHTNING_GDI_IRQ_COMPLETED 1u
#define LIGHTNING_GDI_IRQ_RETIRED 2u
#define LIGHTNING_GDI_IRQ_LOST 4u
#define LIGHTNING_GDI_IRQ_MASK (LIGHTNING_GDI_IRQ_COMPLETED | LIGHTNING_GDI_IRQ_RETIRED | LIGHTNING_GDI_IRQ_LOST)
#define LIGHTNING_GDI_FLAG_READY 1u
#define LIGHTNING_GDI_FLAG_RESETTING 2u
#define LIGHTNING_GDI_FLAG_POISONED 4u
#define LIGHTNING_GDI_FLAG_RETIRED 8u
#define LIGHTNING_GDI_SUBMIT 1u
#define LIGHTNING_GDI_RESET 2u
/* EXTERNAL_MAILBOX only: issue a held replaceable copy now; see EXTERNAL_IMAGE. */
#define LIGHTNING_GDI_FLUSH 3u

#define LIGHTNING_GDI_IDLE 0u
#define LIGHTNING_GDI_ACCEPTED 1u
#define LIGHTNING_GDI_REJECTED 2u
#define LIGHTNING_GDI_LOST 3u
#define LIGHTNING_GDI_OK 0u
#define LIGHTNING_GDI_BAD_PACKET 1u
#define LIGHTNING_GDI_STALE_MODE 2u
#define LIGHTNING_GDI_BAD_SEQUENCE 3u
#define LIGHTNING_GDI_BUSY 4u
#define LIGHTNING_GDI_UNSUPPORTED 5u
#define LIGHTNING_GDI_NOT_READY 6u
#define LIGHTNING_GDI_NO_MEMORY 7u
#define LIGHTNING_GDI_GPU_FAILURE 8u
#define LIGHTNING_GDI_BATCH_INCOMPLETE 9u

#define LIGHTNING_GDI_BIND 1u
#define LIGHTNING_GDI_FILL 2u
#define LIGHTNING_GDI_COPY 3u
#define LIGHTNING_GDI_UPLOAD 4u
#define LIGHTNING_GDI_BLEND_UPLOAD 5u
#define LIGHTNING_GDI_READBACK 6u
#define LIGHTNING_GDI_FENCE 7u
#define LIGHTNING_GDI_ROP3 8u
#define LIGHTNING_GDI_RASTER 9u
#define LIGHTNING_GDI_BEGIN_GROUP 10u
#define LIGHTNING_GDI_END_GROUP 11u
#define LIGHTNING_GDI_GRADIENT 12u
#define LIGHTNING_GDI_EXTERNAL_IMAGE 13u

#define LIGHTNING_GDI_CAP_FILL 1u
#define LIGHTNING_GDI_CAP_COPY 2u
#define LIGHTNING_GDI_CAP_UPLOAD 4u
#define LIGHTNING_GDI_CAP_BLEND 8u
#define LIGHTNING_GDI_CAP_READBACK 16u
#define LIGHTNING_GDI_CAP_FENCES 32u
#define LIGHTNING_GDI_CAP_STRIPES 64u
#define LIGHTNING_GDI_CAP_CLIPS 128u
#define LIGHTNING_GDI_CAP_ROP3 256u
#define LIGHTNING_GDI_CAP_RASTER 512u
#define LIGHTNING_GDI_CAP_MONO_MASK 1024u
#define LIGHTNING_GDI_CAP_PATTERN_TILE 2048u
#define LIGHTNING_GDI_CAP_TEXT 4096u
#define LIGHTNING_GDI_CAP_GROUPS 8192u
#define LIGHTNING_GDI_CAP_STRETCH 16384u
#define LIGHTNING_GDI_CAP_MIRROR 32768u
#define LIGHTNING_GDI_CAP_TRANSPARENT 65536u
#define LIGHTNING_GDI_CAP_STRETCH_ALPHA 131072u
#define LIGHTNING_GDI_CAP_GUARDED_SOURCE 262144u
#define LIGHTNING_GDI_CAP_GRADIENT 524288u
/* Runtime/IRQ-adapter capability; deliberately excluded from controller caps. */
#define LIGHTNING_GDI_CAP_COMPLETION_IRQ 1048576u
#define LIGHTNING_GDI_CAP_CONSUMER_RECEIPTS 2097152u
#define LIGHTNING_GDI_CAP_EXTERNAL_IMAGE 4194304u
/* COLORONCOLOR with an engine draw-decision MONO1 mask; see RASTER below. */
#define LIGHTNING_GDI_CAP_STRETCH_MASK 8388608u
/* Latest-wins external-image presentation; host option, outside CAPABILITIES. */
#define LIGHTNING_GDI_CAP_EXTERNAL_MAILBOX 16777216u
#define LIGHTNING_GDI_BASE_CAPABILITIES 255u
#define LIGHTNING_GDI_CAPABILITIES (LIGHTNING_GDI_BASE_CAPABILITIES | LIGHTNING_GDI_CAP_ROP3 | LIGHTNING_GDI_CAP_RASTER | LIGHTNING_GDI_CAP_MONO_MASK | LIGHTNING_GDI_CAP_PATTERN_TILE | LIGHTNING_GDI_CAP_TEXT | LIGHTNING_GDI_CAP_GROUPS | LIGHTNING_GDI_CAP_STRETCH | LIGHTNING_GDI_CAP_MIRROR | LIGHTNING_GDI_CAP_TRANSPARENT | LIGHTNING_GDI_CAP_STRETCH_ALPHA | LIGHTNING_GDI_CAP_GUARDED_SOURCE | LIGHTNING_GDI_CAP_GRADIENT | LIGHTNING_GDI_CAP_STRETCH_MASK)

/* All scalar fields are little-endian uint32, decoded without struct overlays.
 * Exactly metadata_bytes follow at transfer offset0: this header and clip_count
 * canonical rectangles. Packed upload pixels start at fixed PIXEL_OFFSET.
 * src_x/src_y apply to COPY and ROP3 SCREEN; staging source selection is a native owner
 * concern and never a host address. Unused fields and reserved must be zero.
 */
typedef struct LightningGdiCommand {
    uint32_t magic;
    uint32_t version;
    uint32_t opcode;
    uint32_t flags;
    uint32_t mode_generation;
    uint32_t epoch;
    uint32_t sequence;
    uint32_t reference;
    LightningDisplayRect destination;
    uint32_t source_x;
    uint32_t source_y;
    uint32_t color;
    uint32_t constant_alpha;
    uint32_t clip_count;
    uint32_t pixel_bytes;
    LightningDisplayRect batch;
    uint32_t metadata_bytes;
    uint32_t reserved;
} LightningGdiCommand;

typedef struct LightningGdiSignedRect {
    int32_t x;
    int32_t y;
    uint32_t width;
    uint32_t height;
} LightningGdiSignedRect;

/* Imported linear BGRA8 source, never a guest GL name or address. The native
 * filter resolves lease for the current process and fills context/resource and
 * exact registered layout before forwarding a synchronous GDI submission.
 * Host validation checks that context/resource still own the DMA import. Native
 * success waits for the real canvas-copy fence before dropping the source pin.
 * An uncertain copy retains/quarantines its source owner through context loss.
 * With CAP_EXTERNAL_MAILBOX, header flags may hold EXTERNAL_REPLACEABLE, set only
 * by a native owner that releases its source through the ordered completion
 * receipt rather than a wait inside the submission. The host may then keep the
 * newest such copy unissued until the display latches a frame, any later command
 * or FLUSH arrives, or one refresh period passes. When the next accepted command
 * is also REPLACEABLE and its destination/clips write every pixel the held copy
 * would write, the held copy is superseded: its source is never read, it adds no
 * damage, and its sequence completes in order. Completion therefore still means
 * the source read retired or never happened. Publication always shows the result
 * of every accepted command, so no displayed pixel differs from in-order copies.
 * FLUSH consumes no sequence and changes no other state. Without the capability
 * the flag and FLUSH are rejected as before.
 */
typedef struct LightningGdiExternalImage {
    uint32_t size;
    uint32_t version;
    uint64_t context;
    uint64_t resource;
    uint64_t lease;
    uint32_t width;
    uint32_t height;
    uint32_t stride;
    uint32_t offset;
} LightningGdiExternalImage;

/* GRADIENT appends exactly88B before clips, with no packed pixel payload.
 * Each command owns one mesh element; the native owner validates the complete
 * caller mesh before submitting any element. All coordinates and translated
 * coordinates fit the explicit GRADIENT_COORDINATE_LIMIT. Packed red_green
 * retains Red in low16 and Green in high16; blue is zero-extended16. Alpha is
 * ignored by ReactOS and is absent. Full16 RGB equality chooses solid versus
 * gradient triangle coverage; high8 components alone feed color interpolation.
 * RECT_H/V use vertices0/1 in caller order, sorted geometry but unswapped colors,
 * zero vertex2 and zero origin. TRIANGLE retains all three original vertices
 * and dither origin, sorting by(y,x) in the executor. extents is the original
 * signed bounding clip, not a substitute triangle or barycentric interpolation.
 * Exact ReactOS midpoint-error edge/color steps and ordered lower-edge redraw
 * are required. Header destination and clips bound all GPU writes; unused
 * header fields, descriptor reserved and pixel_bytes are zero.
 */
typedef struct LightningGdiGradientVertex {
    int32_t x;
    int32_t y;
    uint32_t red_green;
    uint32_t blue;
} LightningGdiGradientVertex;

typedef struct LightningGdiGradientOperands {
    uint32_t size;
    uint32_t version;
    uint32_t mode;
    uint32_t reserved;
    LightningGdiGradientVertex vertices[3];
    int32_t origin_x;
    int32_t origin_y;
    LightningGdiSignedRect extents;
} LightningGdiGradientOperands;

/* RASTER alone appends this exact144B/version3 descriptor after96B, BEFORE clips.
 * All offsets are relative to the packed payload at PIXEL_OFFSET, never a BAR,
 * pointer, texture name or physical address. Source, mask and tile spans are
 * compact in that order, have exact DWORD-aligned row strides, never overlap,
 * and end exactly at command.pixel_bytes <= MAX_PIXEL_BYTES. An absent operand
 * has zero geometry/span fields; source_kind reuses ROP_SOURCE_* values.
 * Source rect is canvas-bounded SCREEN or normalized (0,0) uploaded BGRA.
 * GUARDED_SOURCE is an optional-capability ordinary mirrored ROP mode. Each
 * mirrored axis adds one owned leading texel to the physical snapshot; logical
 * source_rect x/y are that prefix (1 or 0) for UPLOAD. Stride/bytes describe the
 * exact logical extent plus those prefixes, never arbitrary trailing storage.
 * INVALID_LEFT/TOP are UPLOAD-only declarations of dummy guard pixels outside
 * the actual native source. They require their corresponding mirror prefix.
 * SCREEN keeps its canvas source_rect, declares no invalid flags, and snapshots
 * available leading texels into a padded texture; validity derives from canvas
 * bounds. A mapped sample outside the valid rectangle is never fetched:
 * SRCCOPY preserves D, other Boolean tables consume untranslated S=0, matching
 * ReactOS DIB_XXBPP_StretchBlt. No dummy pixel becomes a translated source read.
 * NONE stretch requires matching extents and zero original_destination.
 * COLORONCOLOR requires its optional capability and source use. Its only
 * destination mask is the STRETCH_MASK form: a destination-sized MONO1 plane
 * holding DIB_XXBPP_StretchBlt's per-pixel draw decision (the native owner
 * samples the 1bpp DDI mask at the mapped source texel; set bit means draw),
 * foreground_rop the ROP4 foreground and background_rop DSTCOPY0xaa, no key or
 * alpha. Other stretch masks are invalid. With that mask a TILE pattern keeps
 * its vertical canvas phase, while its horizontal phase restarts at each clip's
 * left edge and advances only over drawn pixels, as the DIB PatternX counter.
 * original_destination is the signed-origin, positive-extent driver rectangle
 * before final canvas/clip intersection. Source rect covers that whole mapping.
 * For each actual clip, first truncate mapped source endpoints using the full
 * original destination, then sample within that derived per-clip source extent.
 * Derived source extent zero is valid constant sampling. This is not a single
 * global nearest-neighbor formula. MIRROR_X/Y explicitly reverse sampling;
 * they require COLORONCOLOR. Version2 accepts only zero extension words.
 * COLOR_KEY and ALPHA_BLEND are exclusive SRCCOPY source-only operations.
 * COLOR_KEY compares raw source RGB24 with color_key before translation, or
 * consumes a source-sized MONO1/SOURCE mask (bit1 draw, bit0 preserve D).
 * That mask snapshots raw key equality before native source normalization;
 * color_key is zero when the mask is present. ALPHA_BLEND follows ReactOS
 * AC_SRC_OVER integer /255 arithmetic; SOURCE_ALPHA selects per-pixel alpha,
 * SOURCE_32BPP preserves source high-byte semantics. Other source formats use
 * constant_alpha for their resulting source high byte. Unused words are zero.
 * Alpha right/bottom clip endpoints truncate signed distances from the original
 * right/bottom, matching EngAlphaBlend rather than COLORONCOLOR left distances.
 * MONO1 uses destination-sized MSB-first bits, DWORD rows, bit1 foreground and
 * bit0 background. Without a mask, foreground_rop must equal background_rop.
 * TILE is BGRA with positive modulo(canvas coordinate - signed brush origin).
 * Origin normalization must avoid signed32 subtraction overflow.
 * COVERAGE8 is a distinct TEXT-capability mode: destination-sized byte coverage,
 * DWORD rows, sourceNONE, solid pattern, foreground PATCOPY0xf0/background
 * DSTCOPY0xaa. Coverage0 preserves D;255 writes all32 P bits;1..254 computes
 * each RGB byte D + floor(coverage*(P-D)/256), with output high byte zero.
 * It matches ReactOS AlphaBltMask, not /255 premultiplied-alpha blending.
 * Header flags/source_x/source_y/color/constant_alpha/reference/batch are zero.
 */
typedef struct LightningGdiRasterOperands {
    uint32_t size;
    uint32_t version;
    uint32_t source_kind;
    uint32_t pattern_kind;
    LightningDisplayRect source_rect;
    uint32_t source_offset;
    uint32_t source_bytes;
    uint32_t source_stride;
    uint32_t mask_kind;
    uint32_t mask_space;
    uint32_t mask_offset;
    uint32_t mask_bytes;
    uint32_t mask_stride;
    uint32_t mask_width;
    uint32_t mask_height;
    uint32_t pattern_width;
    uint32_t pattern_height;
    int32_t pattern_origin_x;
    int32_t pattern_origin_y;
    uint32_t pattern_offset;
    uint32_t pattern_bytes;
    uint32_t pattern_stride;
    uint32_t solid_color;
    uint32_t foreground_rop;
    uint32_t background_rop;
    uint32_t stretch_mode;
    uint32_t raster_flags;
    uint32_t color_key;
    uint32_t constant_alpha;
    LightningGdiSignedRect original_destination;
} LightningGdiRasterOperands;

/* BIND is sequence1 for the new mode/reset key and has the full canvas as its
 * destination. It initializes owned GPU storage but does not establish a pixel
 * baseline. UPLOAD BEGIN/END brackets a region, with reference=first chunk's
 * sequence and batch=the complete region. Chunks cover batch width in increasing
 * consecutive horizontal stripes. Only a full unclipped baseline permits draws.
 * A single-chunk upload uses both flags; clips are allowed only for an existing
 * baseline and a single complete chunk. No draw/readback/publication is allowed
 * during an incomplete batch. Failed/uncertain partial uploads poison the canvas.
 * READBACK has reference=current canvas revision (last accepted mutation), no
 * clips, and packed ROI bytes; sequence completion includes the actual GPU fence
 * and the host's bounded copy into the transfer window. The guest retains that
 * window until it copies the result. FENCE observes ordered GPU completion only.
 * Each accepted command advances sequence without wrapping; rejected commands
 * consume neither a sequence nor a source lease. No completion is UI publication.
 * ROP3 requires its optional capability in addition to the unchanged base set.
 * Its constant_alpha field is instead an 8-bit Boolean truth table R: each
 * output bit is R[(Pbit<<2)|(Sbit<<1)|Dbit], including the high byte. color is
 * a solid BGRA pattern P, or zero when R does not use P. flags select exactly
 * NONE, SCREEN or UPLOAD. NONE requires source-independent R and zero source
 * coordinates/pixel_bytes. SCREEN requires source-dependent R and a checked
 * equal-size source rectangle. UPLOAD requires source-dependent R, zero source
 * coordinates and exact packed destination-sized BGRA bytes. reference/batch
 * are zero. Source and destination are snapshotted before any clip writes.
 * No mask, bitmap-pattern tile, scaling or partial upload batch is implied.
 * BEGIN_GROUP/END_GROUP use their separate optional capability. BEGIN references
 * its own sequence; END references that same group identifier. All other fields
 * except identity/size/opcode are zero. No nested group or group marker inside
 * an unfinished upload stripe batch is valid. Ordinary commands, readback and
 * fallback upload remain legal inside a group. Both markers receive actual GPU
 * checkpoint tickets; publication waits for END's observed fence, not merely its
 * acceptance. Markers do not change canvas revision. RESET retains the group's
 * uncertain resources until actual backend retirement, as for other commands.
 */

/* Reuse the canonical unsigned top-left rectangle, with positive extents.
 * Every draw validates all nonoverlapping clips before GPU execution. Pixels
 * are byte-exact little-endian BGRA8, without sRGB conversion or filtering.
 * Upload/readback rows are packed width*4; transfers larger than the bounded
 * pixel window use complete-width horizontal stripes. A partial or uncertain
 * upload does not establish a canvas baseline or authorize CPU fallback replay.
 */

#endif

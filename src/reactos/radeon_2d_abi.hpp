/*
Module: rx7900.reactos.radeon_2d_abi
Purpose: Fixed-layout private IOCTL wire format between the physical Radeon GDI
display DLL and the rx7900video miniport for SDMA 2D acceleration.
Created: 2026-10-22
Architecture: Shared by src/reactos/radeon_2d_ioctl (miniport decoder) and
src/reactos/radeon_display (DDI encoder). Pointer-free requests: every object is
named by an opaque miniport-issued handle, every rectangle is validated against
the miniport's own surface sizes and every transfer names an offset inside the
miniport-owned staging allocation of the calling session. The only address that
crosses this boundary is the miniport's kernel mapping of that staging buffer,
which flows out once in SessionGrant and is never accepted as input.

Wire values are little-endian and decoded with ByteView at named offsets; the
value structs below are never overlaid on IOCTL bytes, so host/compiler struct
padding (i386 GCC aligns uint64_t to four bytes) cannot change the ABI.
*/
#pragma once

#include "byte_view.hpp"
#include "sdma.hpp"

#include <stddef.h>
#include <stdint.h>

namespace rx7900::reactos::radeon2d::abi {

// Version 2 (2026-10-22): 96-byte Capabilities with offscreen accounting and the
// implemented-operation mask, outcome_surface_lost, a reserved operation range
// and 48 offscreen surfaces per session. Version 3 (2026-10-22) defines the
// compute raster frames 0xa10..0xa16 and raster operands (surface handles or
// staging regions). Version 4 (2026-10-22) adds 0xa17 (one exact
// EngStretchBltROP DIB call: mirror flags, sample window, ROP4, solid or tile
// brush, stretch mask) and grows LineRequest by a brush origin and 8x8 tile.
// Version 5 (2026-10-23) lets a 0xa17 map extent be negative (the DIB SourceRect
// after the engine's unscaled clip pre-adjustment, sampled with C truncation),
// adds stretch_flag_running_phase (masked tile PatternX) and the translated
// staging formats index8/rgb16/rgb24 with their XLATEOBJ tables for rop and 0xa17 sources.
// Version 6 (2026-10-24) reports destructive-reset recovery: Capabilities grows to
// 112 bytes (recovery generation and state, surfaces lost to and restored after
// resets), Receipt +12 names the surface an outcome_surface_lost refers to, and a
// lost handle answers outcome_surface_lost exactly once (it is dead afterwards).
// Both sides ship together; a peer of another version fails every exact-frame
// decode instead of misreading fields.
constexpr uint32_t version = 6;
constexpr uint32_t magic = 0x44325852U; // "RX2D" in little-endian byte order.

// FILE_DEVICE_VIDEO, device-specific function range (0x800-0xfff), METHOD_BUFFERED,
// FILE_READ_ACCESS | FILE_WRITE_ACCESS. Functions 0xa00.. do not collide with the
// Lightning private range (0x900..0x90c) or any public IOCTL_VIDEO_* code.
constexpr uint32_t file_device_video = 0x23U;
constexpr uint32_t read_write_access = 0x3U;
constexpr uint32_t control_code(uint32_t function) noexcept {
    return (file_device_video << 16) | (read_write_access << 14) | (function << 2);
}

constexpr uint32_t first_function = 0xa00;
constexpr uint32_t ioctl_query = control_code(0xa00);           // 0x0023e800
constexpr uint32_t ioctl_open_session = control_code(0xa01);    // 0x0023e804
constexpr uint32_t ioctl_close_session = control_code(0xa02);   // 0x0023e808
constexpr uint32_t ioctl_assert_primary = control_code(0xa03);  // 0x0023e80c
constexpr uint32_t ioctl_fill = control_code(0xa04);            // 0x0023e810
constexpr uint32_t ioctl_copy = control_code(0xa05);            // 0x0023e814
constexpr uint32_t ioctl_upload = control_code(0xa06);          // 0x0023e818
constexpr uint32_t ioctl_readback = control_code(0xa07);        // 0x0023e81c
constexpr uint32_t ioctl_surface_create = control_code(0xa08);  // 0x0023e820
constexpr uint32_t ioctl_surface_destroy = control_code(0xa09); // 0x0023e824
constexpr uint32_t ioctl_sync = control_code(0xa0a);            // 0x0023e828
static_assert(ioctl_query == 0x0023e800U && ioctl_sync == 0x0023e828U,
              "Documented private Radeon 2D IOCTL codes");

// Compute raster requests (GFX11 compute engine, src/hardware/raster2d): general
// ROP4 with source/mask, realized 8x8 pattern brushes, glyph masks, per-pixel
// alpha, nearest stretch, cosmetic lines and gradients. They are implemented
// only when the miniport's backend has a compute engine: Capabilities.operations
// bit (function - 0xa00) reports each available code, and an unavailable one
// answers ERROR_INVALID_FUNCTION. Codes 0xa18..0xa3f stay reserved.
constexpr uint32_t ioctl_rop = control_code(0xa10);       // 0x0023e840
constexpr uint32_t ioctl_pattern = control_code(0xa11);   // 0x0023e844
constexpr uint32_t ioctl_text = control_code(0xa12);      // 0x0023e848
constexpr uint32_t ioctl_alpha = control_code(0xa13);     // 0x0023e84c
constexpr uint32_t ioctl_stretch = control_code(0xa14);   // 0x0023e850
constexpr uint32_t ioctl_line = control_code(0xa15);      // 0x0023e854
constexpr uint32_t ioctl_gradient = control_code(0xa16);  // 0x0023e858
constexpr uint32_t ioctl_stretch_rop = control_code(0xa17); // 0x0023e85c
static_assert(ioctl_rop == 0x0023e840U && ioctl_gradient == 0x0023e858U &&
              ioctl_stretch_rop == 0x0023e85cU, "Raster IOCTL codes");
// Last function of the private range (0xa00..0xa3f: 64 codes, one mask bit each).
constexpr uint32_t last_function = 0xa3f;
constexpr uint32_t ioctl_last = control_code(last_function);

constexpr bool is_radeon_2d_ioctl(uint32_t code) noexcept {
    return code >= ioctl_query && code <= ioctl_last && (code & 3U) == 0;
}

// Mask bit of a private code (caller proved is_radeon_2d_ioctl).
constexpr uint64_t operation_bit(uint32_t code) noexcept {
    return uint64_t{1} << (((code >> 2) & 0xfffU) - first_function);
}

// Operations implemented by this version (Capabilities.operations).
constexpr uint64_t implemented_operations =
    operation_bit(ioctl_query) | operation_bit(ioctl_open_session) |
    operation_bit(ioctl_close_session) | operation_bit(ioctl_assert_primary) |
    operation_bit(ioctl_fill) | operation_bit(ioctl_copy) | operation_bit(ioctl_upload) |
    operation_bit(ioctl_readback) | operation_bit(ioctl_surface_create) |
    operation_bit(ioctl_surface_destroy) | operation_bit(ioctl_sync);
static_assert(implemented_operations == 0x7ffU, "Eleven implemented operations");

// The compute raster operations (present only with a compute backend).
constexpr uint64_t raster_operations =
    operation_bit(ioctl_rop) | operation_bit(ioctl_pattern) | operation_bit(ioctl_text) |
    operation_bit(ioctl_alpha) | operation_bit(ioctl_stretch) | operation_bit(ioctl_line) |
    operation_bit(ioctl_gradient) | operation_bit(ioctl_stretch_rop);
static_assert(raster_operations == 0xff0000ULL, "Eight raster operations");

constexpr bool is_raster_ioctl(uint32_t code) noexcept {
    return code >= ioctl_rop && code <= ioctl_stretch_rop && (code & 3U) == 0;
}

// Bounded policy shared by both sides. A request carries at most max_rects
// rectangles; larger clip regions are split by the client into several
// requests, which the miniport executes in submission order (the backend
// inserts the read/write barriers between them).
constexpr uint32_t max_rects = 64;
constexpr uint32_t max_sessions = 4;
constexpr uint32_t max_surfaces = 64;          // Includes the primary slot.
constexpr uint32_t max_session_surfaces = 48;  // Offscreen allocations per session.
constexpr uint32_t max_surface_dimension = 16384;
constexpr uint32_t max_staging_pitch = max_surface_dimension * 4U;
constexpr uint32_t format_xrgb8888 = 1;
constexpr uint32_t bytes_per_pixel = 4;
// Default offscreen byte budgets (pitch * height of live offscreen surfaces):
// per session and for the whole device. The device owner may configure lower
// or higher values; Capabilities reports the effective ones.
constexpr uint64_t default_session_surface_bytes = 256ULL << 20;
constexpr uint64_t default_device_surface_bytes = 512ULL << 20;

// Capability flags (Capabilities.flags and SessionGrant.flags).
constexpr uint32_t capability_fill = 1U << 0;
constexpr uint32_t capability_copy = 1U << 1;
constexpr uint32_t capability_upload = 1U << 2;
constexpr uint32_t capability_readback = 1U << 3;
constexpr uint32_t capability_offscreen = 1U << 4;
constexpr uint32_t capability_primary = 1U << 5;
// Offscreen surfaces survive suspend: they are moved to system memory before
// the 2D engine stops and back to VRAM after it restarts (best effort).
constexpr uint32_t capability_eviction = 1U << 6;
// Compute raster requests (Capabilities.operations names the exact codes).
constexpr uint32_t capability_raster = 1U << 8;

// Raster operand formats. Staging operands of every format keep pitch >= 4 *
// width (the compute engine validates every surface as 32-bpp addressable);
// 1-bpp rows are MSB-first, coverage8 is one byte per pixel.
constexpr uint32_t format_mono1 = 2;
constexpr uint32_t format_coverage8 = 3;
// Translated staging sources (0xa10, 0xa11, 0xa14, 0xa17): an XLATEOBJ table of
// staging_table_bytes(format) bytes at the operand offset, then the pixel rows
// (index8 1, rgb16 2, rgb24 3 bytes per pixel; pitch >= that times width). The
// GPU reads S = table[pixel] (rgb24: T0[b0] | T1[b1] | T2[b2], three 256-dword
// tables; the client sends it only for an OR-linear translation).
constexpr uint32_t format_index8 = 4;
constexpr uint32_t format_rgb16 = 5;
constexpr uint32_t format_rgb24 = 6;

constexpr bool translated_format(uint32_t format) noexcept {
    return format == format_index8 || format == format_rgb16 || format == format_rgb24;
}

constexpr uint32_t staging_table_bytes(uint32_t format) noexcept {
    return format == format_index8 ? 256U * 4U : format == format_rgb16 ? 65536U * 4U :
           format == format_rgb24 ? 3U * 256U * 4U : 0U;
}

// Minimum pitch of a staging operand row: every pre-v5 format keeps 4 bytes per
// pixel; translated formats use their own pixel size.
constexpr uint64_t staging_min_pitch(uint32_t format, uint32_t width) noexcept {
    return static_cast<uint64_t>(width) * (format == format_index8 ? 1U : format == format_rgb16 ? 2U :
                                           format == format_rgb24 ? 3U : bytes_per_pixel);
}

// Receipt.outcome. Every structurally valid, authorized request that reached the
// execution stage returns a receipt; an IOCTL error means the GPU was not touched.
constexpr uint32_t outcome_completed = 0; // Accepted (fill/copy/upload) or finished.
constexpr uint32_t outcome_declined = 1;  // Not executed; no GPU mutation occurred.
constexpr uint32_t outcome_lost = 2;      // Uncertain execution; session poisoned.
// Not executed: an offscreen surface named by the request lost its contents
// (eviction could not save it, or a destructive reset erased its VRAM).
// Receipt.lost_surface names it. The miniport answers this exactly once per lost
// handle: the handle is retired with that answer, later requests naming it are
// invalid and its slot is reused only under a new generation. The session is
// not poisoned and no GPU mutation occurred.
constexpr uint32_t outcome_surface_lost = 3;

// Capabilities.recovery_state (version 6). operational: 2D accepts sessions;
// after recovery_generation advanced, sessions that were open survived the
// reset (their in-flight work was resolved by the reset proof). disabled_quiescent:
// a recovery failed after the reset proof (or the adopted scanout no longer
// matched), 2D stays off for the device lifetime and no earlier GPU work can
// still access memory. disabled_unproven: 2D stays off and GPU retirement of
// earlier work is unknown. pending: a fault was detected and dispatch is
// quiesced (every request except query answers not_ready) until the miniport's
// recovery thread finished; ask again later.
constexpr uint32_t recovery_operational = 0;
constexpr uint32_t recovery_disabled_quiescent = 1;
constexpr uint32_t recovery_disabled_unproven = 2;
constexpr uint32_t recovery_pending = 3;

// ---------------------------------------------------------------------------
// Exact byte layouts. Offsets are relative to the start of the IOCTL buffer.
//
// RequestHeader (16): +0 size, +4 version, +8 session, +12 session_key
// Rect (16):          +0 left, +4 top, +8 right, +12 bottom (int32, half-open)
// Capabilities (96):  +0 size, +4 version, +8 magic, +12 flags, +16 max_rects,
//   +20 max_sessions, +24 max_session_surfaces, +28 max_surface_dimension,
//   +32 primary_width, +36 primary_height, +40 primary_pitch, +44 primary_format,
//   +48 staging_bytes, +52 open_sessions, +56 accepted_requests, +60 rejected_requests,
//   +64 session_surface_bytes (uint64 budget), +72 device_surface_bytes (uint64
//   budget), +80 used_surface_bytes (uint64, all sessions), +88 operations (uint64),
//   +96 recovery_generation (completed destructive-reset recoveries),
//   +100 recovery_state, +104 reset_lost_surfaces, +108 reset_restored_surfaces
// OpenSessionRequest (16): +0 size, +4 version, +8 flags (0), +12 reserved (0)
// SessionGrant (56):  +0 size, +4 version, +8 session, +12 session_key,
//   +16 primary_surface, +20 primary_width, +24 primary_height, +28 primary_pitch,
//   +32 primary_format, +36 staging_bytes, +40 flags, +44 reserved,
//   +48 staging_address (uint64, miniport kernel mapping, output only)
// CloseSessionRequest (16): RequestHeader only; no output.
// AssertPrimaryRequest (24): header, +16 enable (0/1), +20 reserved
// FillRequest (1056): header, +16 surface, +20 color, +24 rect_count, +28 reserved,
//   +32 Rect[64] (entries at and above rect_count must be zero)
// CopyRequest (1064): header, +16 target, +20 source, +24 source_dx (int32),
//   +28 source_dy (int32), +32 rect_count, +36 reserved, +40 Rect[64]
//   Destination rectangles; each source rectangle is the destination + (dx, dy).
//   The result equals copying every source rectangle through a temporary.
// TransferRequest (1064, upload and readback): header, +16 surface,
//   +20 staging_offset, +24 staging_pitch, +28 origin_x (int32), +32 origin_y (int32),
//   +36 rect_count, +40 Rect[64]. Staging byte of surface pixel (x, y) is
//   staging_offset + (y - origin_y) * staging_pitch + (x - origin_x) * 4.
// SurfaceCreateRequest (32): header, +16 width, +20 height, +24 format, +28 flags (0)
// SurfaceGrant (32): +0 size, +4 version, +8 surface, +12 width, +16 height,
//   +20 pitch, +24 format, +28 reserved
// SurfaceDestroyRequest (24): header, +16 surface, +20 reserved
// SyncRequest (24): header, +16 fence (uint64; 0 = everything this session submitted)
// Receipt (32): +0 size, +4 version, +8 outcome, +12 lost_surface (the handle of
//   an outcome_surface_lost, zero for every other outcome), +16 fence (uint64),
//   +24 completed_fence (uint64)
// ---------------------------------------------------------------------------
constexpr uint32_t header_bytes = 16;
constexpr uint32_t rect_bytes = 16;
constexpr uint32_t rect_array_bytes = max_rects * rect_bytes;
constexpr uint32_t capabilities_bytes = 112;
constexpr uint32_t open_session_request_bytes = 16;
constexpr uint32_t session_grant_bytes = 56;
constexpr uint32_t close_session_request_bytes = header_bytes;
constexpr uint32_t assert_primary_request_bytes = 24;
constexpr uint32_t fill_rects_offset = 32;
constexpr uint32_t fill_request_bytes = fill_rects_offset + rect_array_bytes;
constexpr uint32_t copy_rects_offset = 40;
constexpr uint32_t copy_request_bytes = copy_rects_offset + rect_array_bytes;
constexpr uint32_t transfer_rects_offset = 40;
constexpr uint32_t transfer_request_bytes = transfer_rects_offset + rect_array_bytes;
constexpr uint32_t surface_create_request_bytes = 32;
constexpr uint32_t surface_grant_bytes = 32;
constexpr uint32_t surface_destroy_request_bytes = 24;
constexpr uint32_t sync_request_bytes = 24;
constexpr uint32_t receipt_bytes = 32;
// Raster frames (version 3); layouts are documented with their codecs below.
constexpr uint32_t operand_bytes = 24;
constexpr uint32_t source_map_bytes = 16;
constexpr uint32_t pattern_tile_bytes = 256;
constexpr uint32_t style_bytes = 256;
constexpr uint32_t max_style_bits = style_bytes * 8U;
constexpr uint32_t rop_rects_offset = 152;
constexpr uint32_t rop_request_bytes = rop_rects_offset + rect_array_bytes;
constexpr uint32_t pattern_request_bytes = rop_request_bytes + pattern_tile_bytes;
constexpr uint32_t text_rects_offset = 88;
constexpr uint32_t text_request_bytes = text_rects_offset + rect_array_bytes;
constexpr uint32_t alpha_rects_offset = 96;
constexpr uint32_t alpha_request_bytes = alpha_rects_offset + rect_array_bytes;
constexpr uint32_t line_rects_offset = 320;
constexpr uint32_t line_brush_offset = line_rects_offset + rect_array_bytes;
constexpr uint32_t line_tile_offset = line_brush_offset + 8;
constexpr uint32_t line_request_bytes = line_tile_offset + pattern_tile_bytes;
constexpr uint32_t gradient_rects_offset = 104;
constexpr uint32_t gradient_request_bytes = gradient_rects_offset + rect_array_bytes;
constexpr uint32_t stretch_rop_rects_offset = 160;
constexpr uint32_t stretch_rop_tile_offset = stretch_rop_rects_offset + rect_array_bytes;
constexpr uint32_t stretch_rop_request_bytes = stretch_rop_tile_offset + pattern_tile_bytes;
static_assert(rop_request_bytes == 1176 && pattern_request_bytes == 1432 &&
              text_request_bytes == 1112 && alpha_request_bytes == 1120 &&
              line_request_bytes == 1608 && gradient_request_bytes == 1128 &&
              stretch_rop_request_bytes == 1440, "Raster frame sizes");
constexpr uint32_t max_request_bytes = line_request_bytes;
constexpr uint32_t max_reply_bytes = capabilities_bytes;
static_assert(fill_request_bytes == 1056 && copy_request_bytes == 1064 &&
              transfer_request_bytes == 1064, "Fixed rectangle-list request sizes");
static_assert(session_grant_bytes <= max_reply_bytes && receipt_bytes <= max_reply_bytes &&
              surface_grant_bytes <= max_reply_bytes, "Bounded reply workspace");

// ---------------------------------------------------------------------------
// Value carriers. Never overlaid on wire bytes.
// ---------------------------------------------------------------------------
struct Rect final {
    int32_t left = 0;
    int32_t top = 0;
    int32_t right = 0;
    int32_t bottom = 0;
};

struct Header final {
    uint32_t session = 0;
    uint32_t session_key = 0;
};

struct Capabilities final {
    uint32_t flags = 0;
    uint32_t max_rects = 0;
    uint32_t max_sessions = 0;
    uint32_t max_session_surfaces = 0;
    uint32_t max_surface_dimension = 0;
    uint32_t primary_width = 0;
    uint32_t primary_height = 0;
    uint32_t primary_pitch = 0;
    uint32_t primary_format = 0;
    uint32_t staging_bytes = 0;
    uint32_t open_sessions = 0;
    uint32_t accepted_requests = 0;
    uint32_t rejected_requests = 0;
    uint64_t session_surface_bytes = 0;
    uint64_t device_surface_bytes = 0;
    uint64_t used_surface_bytes = 0;
    uint64_t operations = 0;
    uint32_t recovery_generation = 0;
    uint32_t recovery_state = recovery_operational;
    uint32_t reset_lost_surfaces = 0;
    uint32_t reset_restored_surfaces = 0;
};

struct SessionGrant final {
    Header session{};
    uint32_t primary_surface = 0;
    uint32_t primary_width = 0;
    uint32_t primary_height = 0;
    uint32_t primary_pitch = 0;
    uint32_t primary_format = 0;
    uint32_t staging_bytes = 0;
    uint32_t flags = 0;
    uint64_t staging_address = 0;
};

struct RectList final {
    uint32_t count = 0;
    Rect rects[max_rects]{};
};

struct FillCommand final {
    Header session{};
    uint32_t surface = 0;
    uint32_t color = 0;
    RectList rects{};
};

struct CopyCommand final {
    Header session{};
    uint32_t target = 0;
    uint32_t source = 0;
    int32_t source_dx = 0;
    int32_t source_dy = 0;
    RectList rects{};
};

struct TransferCommand final {
    Header session{};
    uint32_t surface = 0;
    uint32_t staging_offset = 0;
    uint32_t staging_pitch = 0;
    int32_t origin_x = 0;
    int32_t origin_y = 0;
    RectList rects{};
};

struct AssertPrimaryCommand final {
    Header session{};
    uint32_t enable = 0;
};

struct SurfaceCreateCommand final {
    Header session{};
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t format = 0;
};

struct SurfaceGrant final {
    uint32_t surface = 0;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t pitch = 0;
    uint32_t format = 0;
};

struct SurfaceDestroyCommand final {
    Header session{};
    uint32_t surface = 0;
};

struct SyncCommand final {
    Header session{};
    uint64_t fence = 0;
};

struct Receipt final {
    uint32_t outcome = outcome_declined;
    uint32_t lost_surface = 0;
    uint64_t fence = 0;
    uint64_t completed_fence = 0;
};

// ---------------------------------------------------------------------------
// Checked codec. Decoders validate exact length, size/version fields, reserved
// zeros, list bounds and zero padding. They do not check handles or geometry;
// the miniport dispatcher owns those semantic checks against its own objects.
// Encoders write the complete fixed buffer (including zero padding) or fail.
// ---------------------------------------------------------------------------
namespace detail {

// Two's-complement reinterpretation without implementation-defined conversion.
constexpr int32_t to_signed(uint32_t value) noexcept {
    return value <= 0x7fffffffU ? static_cast<int32_t>(value) :
        -static_cast<int32_t>(~value) - 1;
}

constexpr uint32_t to_unsigned(int32_t value) noexcept {
    return static_cast<uint32_t>(value); // Modular conversion is well defined.
}

inline bool read_i32(ByteView input, size_t offset, int32_t& value) noexcept {
    uint32_t raw = 0;
    if (!input.read_u32(offset, raw)) {
        return false;
    }

    value = to_signed(raw);
    return true;
}

inline bool read_u64(ByteView input, size_t offset, uint64_t& value) noexcept {
    uint32_t low = 0;
    uint32_t high = 0;
    if (!input.read_u32(offset, low) || !input.read_u32(offset + 4, high)) {
        return false;
    }

    value = (static_cast<uint64_t>(high) << 32) | low;
    return true;
}

inline bool write_u64(const sdma::CommandBuffer& output, size_t offset, uint64_t value) noexcept {
    return output.write_u32(offset, static_cast<uint32_t>(value)) &&
           output.write_u32(offset + 4, static_cast<uint32_t>(value >> 32));
}

inline bool zero_fill(const sdma::CommandBuffer& output) noexcept {
    for (size_t offset = 0; offset + 4 <= output.size(); offset += 4) {
        if (!output.write_u32(offset, 0)) {
            return false;
        }
    }

    return (output.size() & 3U) == 0;
}

// Exact length plus the leading size/version pair.
inline bool expect_frame(ByteView input, uint32_t bytes) noexcept {
    uint32_t size = 0;
    uint32_t wire_version = 0;
    return input.size() == bytes && input.read_u32(0, size) && size == bytes &&
           input.read_u32(4, wire_version) && wire_version == version;
}

inline bool decode_header(ByteView input, uint32_t bytes, Header& output) noexcept {
    return expect_frame(input, bytes) && input.read_u32(8, output.session) &&
           input.read_u32(12, output.session_key);
}

inline bool expect_zero(ByteView input, size_t offset) noexcept {
    uint32_t value = 1;
    return input.read_u32(offset, value) && value == 0;
}

inline bool decode_rect(ByteView input, size_t offset, Rect& output) noexcept {
    return read_i32(input, offset, output.left) && read_i32(input, offset + 4, output.top) &&
           read_i32(input, offset + 8, output.right) && read_i32(input, offset + 12, output.bottom);
}

// count was decoded separately; entries beyond it must be all-zero padding.
inline bool decode_rects(ByteView input, size_t offset, uint32_t count, RectList& output) noexcept {
    if (count == 0 || count > max_rects) {
        return false;
    }

    output.count = count;
    for (uint32_t index = 0; index < max_rects; ++index) {
        const size_t position = offset + static_cast<size_t>(index) * rect_bytes;
        if (index < count) {
            if (!decode_rect(input, position, output.rects[index])) {
                return false;
            }
        } else {
            output.rects[index] = {};
            for (size_t word = 0; word < rect_bytes; word += 4) {
                if (!expect_zero(input, position + word)) {
                    return false;
                }
            }
        }
    }

    return true;
}

inline bool begin_frame(const sdma::CommandBuffer& output, uint32_t bytes) noexcept {
    return output.size() == bytes && zero_fill(output) && output.write_u32(0, bytes) &&
           output.write_u32(4, version);
}

inline bool encode_header(const sdma::CommandBuffer& output, uint32_t bytes,
                          const Header& header) noexcept {
    return begin_frame(output, bytes) && output.write_u32(8, header.session) &&
           output.write_u32(12, header.session_key);
}

inline bool encode_rects(const sdma::CommandBuffer& output, size_t offset,
                         const RectList& list) noexcept {
    if (list.count == 0 || list.count > max_rects) {
        return false;
    }

    for (uint32_t index = 0; index < list.count; ++index) {
        const size_t position = offset + static_cast<size_t>(index) * rect_bytes;
        const Rect& rect = list.rects[index];
        if (!output.write_u32(position, to_unsigned(rect.left)) ||
            !output.write_u32(position + 4, to_unsigned(rect.top)) ||
            !output.write_u32(position + 8, to_unsigned(rect.right)) ||
            !output.write_u32(position + 12, to_unsigned(rect.bottom))) {
            return false;
        }
    }

    return true;
}

} // namespace detail

// Requests (decode in the miniport, encode in the DLL).
inline bool decode_open_session(ByteView input) noexcept {
    return detail::expect_frame(input, open_session_request_bytes) &&
           detail::expect_zero(input, 8) && detail::expect_zero(input, 12);
}

inline bool encode_open_session(const sdma::CommandBuffer& output) noexcept {
    return detail::begin_frame(output, open_session_request_bytes);
}

inline bool decode_close_session(ByteView input, Header& output) noexcept {
    return detail::decode_header(input, close_session_request_bytes, output);
}

inline bool encode_close_session(const sdma::CommandBuffer& output, const Header& header) noexcept {
    return detail::encode_header(output, close_session_request_bytes, header);
}

inline bool decode_assert_primary(ByteView input, AssertPrimaryCommand& output) noexcept {
    return detail::decode_header(input, assert_primary_request_bytes, output.session) &&
           input.read_u32(16, output.enable) && output.enable <= 1 &&
           detail::expect_zero(input, 20);
}

inline bool encode_assert_primary(const sdma::CommandBuffer& output,
                                  const AssertPrimaryCommand& command) noexcept {
    return command.enable <= 1 &&
           detail::encode_header(output, assert_primary_request_bytes, command.session) &&
           output.write_u32(16, command.enable);
}

inline bool decode_fill(ByteView input, FillCommand& output) noexcept {
    uint32_t count = 0;
    return detail::decode_header(input, fill_request_bytes, output.session) &&
           input.read_u32(16, output.surface) && input.read_u32(20, output.color) &&
           input.read_u32(24, count) && detail::expect_zero(input, 28) &&
           detail::decode_rects(input, fill_rects_offset, count, output.rects);
}

inline bool encode_fill(const sdma::CommandBuffer& output, const FillCommand& command) noexcept {
    return detail::encode_header(output, fill_request_bytes, command.session) &&
           output.write_u32(16, command.surface) && output.write_u32(20, command.color) &&
           output.write_u32(24, command.rects.count) &&
           detail::encode_rects(output, fill_rects_offset, command.rects);
}

inline bool decode_copy(ByteView input, CopyCommand& output) noexcept {
    uint32_t count = 0;
    return detail::decode_header(input, copy_request_bytes, output.session) &&
           input.read_u32(16, output.target) && input.read_u32(20, output.source) &&
           detail::read_i32(input, 24, output.source_dx) &&
           detail::read_i32(input, 28, output.source_dy) && input.read_u32(32, count) &&
           detail::expect_zero(input, 36) &&
           detail::decode_rects(input, copy_rects_offset, count, output.rects);
}

inline bool encode_copy(const sdma::CommandBuffer& output, const CopyCommand& command) noexcept {
    return detail::encode_header(output, copy_request_bytes, command.session) &&
           output.write_u32(16, command.target) && output.write_u32(20, command.source) &&
           output.write_u32(24, detail::to_unsigned(command.source_dx)) &&
           output.write_u32(28, detail::to_unsigned(command.source_dy)) &&
           output.write_u32(32, command.rects.count) &&
           detail::encode_rects(output, copy_rects_offset, command.rects);
}

inline bool decode_transfer(ByteView input, TransferCommand& output) noexcept {
    uint32_t count = 0;
    return detail::decode_header(input, transfer_request_bytes, output.session) &&
           input.read_u32(16, output.surface) && input.read_u32(20, output.staging_offset) &&
           input.read_u32(24, output.staging_pitch) &&
           detail::read_i32(input, 28, output.origin_x) &&
           detail::read_i32(input, 32, output.origin_y) && input.read_u32(36, count) &&
           detail::decode_rects(input, transfer_rects_offset, count, output.rects);
}

inline bool encode_transfer(const sdma::CommandBuffer& output,
                            const TransferCommand& command) noexcept {
    return detail::encode_header(output, transfer_request_bytes, command.session) &&
           output.write_u32(16, command.surface) && output.write_u32(20, command.staging_offset) &&
           output.write_u32(24, command.staging_pitch) &&
           output.write_u32(28, detail::to_unsigned(command.origin_x)) &&
           output.write_u32(32, detail::to_unsigned(command.origin_y)) &&
           output.write_u32(36, command.rects.count) &&
           detail::encode_rects(output, transfer_rects_offset, command.rects);
}

inline bool decode_surface_create(ByteView input, SurfaceCreateCommand& output) noexcept {
    return detail::decode_header(input, surface_create_request_bytes, output.session) &&
           input.read_u32(16, output.width) && input.read_u32(20, output.height) &&
           input.read_u32(24, output.format) && detail::expect_zero(input, 28);
}

inline bool encode_surface_create(const sdma::CommandBuffer& output,
                                  const SurfaceCreateCommand& command) noexcept {
    return detail::encode_header(output, surface_create_request_bytes, command.session) &&
           output.write_u32(16, command.width) && output.write_u32(20, command.height) &&
           output.write_u32(24, command.format);
}

inline bool decode_surface_destroy(ByteView input, SurfaceDestroyCommand& output) noexcept {
    return detail::decode_header(input, surface_destroy_request_bytes, output.session) &&
           input.read_u32(16, output.surface) && detail::expect_zero(input, 20);
}

inline bool encode_surface_destroy(const sdma::CommandBuffer& output,
                                   const SurfaceDestroyCommand& command) noexcept {
    return detail::encode_header(output, surface_destroy_request_bytes, command.session) &&
           output.write_u32(16, command.surface);
}

inline bool decode_sync(ByteView input, SyncCommand& output) noexcept {
    return detail::decode_header(input, sync_request_bytes, output.session) &&
           detail::read_u64(input, 16, output.fence);
}

inline bool encode_sync(const sdma::CommandBuffer& output, const SyncCommand& command) noexcept {
    return detail::encode_header(output, sync_request_bytes, command.session) &&
           detail::write_u64(output, 16, command.fence);
}

// Replies (encode in the miniport, decode in the DLL).
inline bool encode_capabilities(const sdma::CommandBuffer& output, const Capabilities& value) noexcept {
    return detail::begin_frame(output, capabilities_bytes) && output.write_u32(8, magic) &&
           output.write_u32(12, value.flags) && output.write_u32(16, value.max_rects) &&
           output.write_u32(20, value.max_sessions) &&
           output.write_u32(24, value.max_session_surfaces) &&
           output.write_u32(28, value.max_surface_dimension) &&
           output.write_u32(32, value.primary_width) && output.write_u32(36, value.primary_height) &&
           output.write_u32(40, value.primary_pitch) && output.write_u32(44, value.primary_format) &&
           output.write_u32(48, value.staging_bytes) && output.write_u32(52, value.open_sessions) &&
           output.write_u32(56, value.accepted_requests) &&
           output.write_u32(60, value.rejected_requests) &&
           detail::write_u64(output, 64, value.session_surface_bytes) &&
           detail::write_u64(output, 72, value.device_surface_bytes) &&
           detail::write_u64(output, 80, value.used_surface_bytes) &&
           detail::write_u64(output, 88, value.operations) &&
           output.write_u32(96, value.recovery_generation) && output.write_u32(100, value.recovery_state) &&
           output.write_u32(104, value.reset_lost_surfaces) &&
           output.write_u32(108, value.reset_restored_surfaces);
}

inline bool decode_capabilities(ByteView input, Capabilities& output) noexcept {
    uint32_t wire_magic = 0;
    return detail::expect_frame(input, capabilities_bytes) && input.read_u32(8, wire_magic) &&
           wire_magic == magic && input.read_u32(12, output.flags) &&
           input.read_u32(16, output.max_rects) && input.read_u32(20, output.max_sessions) &&
           input.read_u32(24, output.max_session_surfaces) &&
           input.read_u32(28, output.max_surface_dimension) &&
           input.read_u32(32, output.primary_width) && input.read_u32(36, output.primary_height) &&
           input.read_u32(40, output.primary_pitch) && input.read_u32(44, output.primary_format) &&
           input.read_u32(48, output.staging_bytes) && input.read_u32(52, output.open_sessions) &&
           input.read_u32(56, output.accepted_requests) &&
           input.read_u32(60, output.rejected_requests) &&
           detail::read_u64(input, 64, output.session_surface_bytes) &&
           detail::read_u64(input, 72, output.device_surface_bytes) &&
           detail::read_u64(input, 80, output.used_surface_bytes) &&
           detail::read_u64(input, 88, output.operations) &&
           input.read_u32(96, output.recovery_generation) && input.read_u32(100, output.recovery_state) &&
           output.recovery_state <= recovery_pending &&
           input.read_u32(104, output.reset_lost_surfaces) &&
           input.read_u32(108, output.reset_restored_surfaces);
}

inline bool encode_session_grant(const sdma::CommandBuffer& output, const SessionGrant& value) noexcept {
    return detail::begin_frame(output, session_grant_bytes) &&
           output.write_u32(8, value.session.session) &&
           output.write_u32(12, value.session.session_key) &&
           output.write_u32(16, value.primary_surface) && output.write_u32(20, value.primary_width) &&
           output.write_u32(24, value.primary_height) && output.write_u32(28, value.primary_pitch) &&
           output.write_u32(32, value.primary_format) && output.write_u32(36, value.staging_bytes) &&
           output.write_u32(40, value.flags) && detail::write_u64(output, 48, value.staging_address);
}

inline bool decode_session_grant(ByteView input, SessionGrant& output) noexcept {
    return detail::expect_frame(input, session_grant_bytes) &&
           input.read_u32(8, output.session.session) &&
           input.read_u32(12, output.session.session_key) &&
           input.read_u32(16, output.primary_surface) && input.read_u32(20, output.primary_width) &&
           input.read_u32(24, output.primary_height) && input.read_u32(28, output.primary_pitch) &&
           input.read_u32(32, output.primary_format) && input.read_u32(36, output.staging_bytes) &&
           input.read_u32(40, output.flags) && detail::expect_zero(input, 44) &&
           detail::read_u64(input, 48, output.staging_address);
}

inline bool encode_surface_grant(const sdma::CommandBuffer& output, const SurfaceGrant& value) noexcept {
    return detail::begin_frame(output, surface_grant_bytes) && output.write_u32(8, value.surface) &&
           output.write_u32(12, value.width) && output.write_u32(16, value.height) &&
           output.write_u32(20, value.pitch) && output.write_u32(24, value.format);
}

inline bool decode_surface_grant(ByteView input, SurfaceGrant& output) noexcept {
    return detail::expect_frame(input, surface_grant_bytes) && input.read_u32(8, output.surface) &&
           input.read_u32(12, output.width) && input.read_u32(16, output.height) &&
           input.read_u32(20, output.pitch) && input.read_u32(24, output.format) &&
           detail::expect_zero(input, 28);
}

inline bool encode_receipt(const sdma::CommandBuffer& output, const Receipt& value) noexcept {
    return value.outcome <= outcome_surface_lost &&
           (value.lost_surface == 0 || value.outcome == outcome_surface_lost) &&
           detail::begin_frame(output, receipt_bytes) &&
           output.write_u32(8, value.outcome) && output.write_u32(12, value.lost_surface) &&
           detail::write_u64(output, 16, value.fence) &&
           detail::write_u64(output, 24, value.completed_fence);
}

inline bool decode_receipt(ByteView input, Receipt& output) noexcept {
    return detail::expect_frame(input, receipt_bytes) && input.read_u32(8, output.outcome) &&
           output.outcome <= outcome_surface_lost && input.read_u32(12, output.lost_surface) &&
           (output.lost_surface == 0 || output.outcome == outcome_surface_lost) &&
           detail::read_u64(input, 16, output.fence) &&
           detail::read_u64(input, 24, output.completed_fence);
}


// ---------------------------------------------------------------------------
// Raster frames (version 3). Common pieces:
//
// Operand (24): +0 kind (0 none, 1 surface, 2 staging), +4 handle (surface) or
//   staging_offset (staging), +8 pitch, +12 width, +16 height, +20 format.
//   none: every field zero. surface: pitch/width/height zero, format
//   format_xrgb8888 (the miniport knows the surface). staging: a region of the
//   session's staging allocation; width and height 1..max_surface_dimension,
//   format xrgb8888/mono1/coverage8, offset and pitch multiples of four,
//   pitch >= 4 * width and every byte inside the staging allocation.
// SourceMap (16): +0 left, +4 top (int32), +8 width, +12 height (1..16384). The
//   source pixel of destination (x, y) is (left + floor((x - bounds.left) *
//   width / bounds_width), top + floor((y - bounds.top) * height /
//   bounds_height)); a 1:1 map has width/height equal to the bounds size.
// bounds: the operation rectangle (the map origin and denominator), 1..16384
//   pixels per axis, |coordinates| < 2^24. Every clip rectangle lies inside both
//   bounds and the target surface, nonempty and pairwise disjoint.
//
// RopRequest (1176; 0xa10 rop, 0xa14 stretch) and PatternRequest (1432, 0xa11):
//   header, +16 target, +20 rop4 (<= 0xffff), +24 flags, +28 brush_colour,
//   +32 brush_x, +36 brush_y (int32), +40 mono_foreground, +44 mono_background,
//   +48 colour_key, +52 reserved, +56 bounds, +72 source operand, +96 source map,
//   +112 mask operand (mono1 or none), +136 mask_x, +140 mask_y (int32; mask pixel
//   of bounds.left/top), +144 rect_count, +148 reserved, +152 Rect[64];
//   PatternRequest appends the 8x8 tile (64 BGRX dwords, row-major) at +1176.
//   A staging source may be xrgb8888, mono1 or a translated format (v5).
//   0xa10 requires an identity map (or no source); 0xa14 requires a source.
// TextRequest (1112; 0xa12): header, +16 target, +20 mode (1 mono1 glyph mask:
//   colour where the bit is 1, destination elsewhere; 2 coverage8: blend colour
//   by coverage), +24 colour, +28 reserved, +32 bounds, +48 mask operand (staging,
//   format per mode), +72 mask_x, +76 mask_y, +80 rect_count, +84 reserved,
//   +88 Rect[64].
// AlphaRequest (1120; 0xa13): header, +16 target, +20 constant_alpha (0..255),
//   +24 flags (bit 0 AC_SRC_ALPHA), +28 reserved, +32 bounds, +48 source operand
//   (xrgb8888), +72 source map, +88 rect_count, +92 reserved, +96 Rect[64].
// LineRequest (1608; 0xa15): header, +16 target, +20 colour, +24 rop3 (<= 0xff),
//   +28 flags (bit 0 styled, bit 1 pattern; not both), +32 x0, +36 y0, +40 x1,
//   +44 y1 (int32, start included, end excluded), +48 style_bit_count (styled:
//   1..2048, else 0), +52 reserved, +56 style bits (MSB first; bits at and above
//   the count zero), +312 rect_count, +316 reserved, +320 Rect[64], +1344
//   brush_x, +1348 brush_y (int32), +1352 8x8 tile (64 BGRX dwords). pattern:
//   P of pixel (x, y) is tile[((y - brush_y) & 7) * 8 + ((x - brush_x) & 7)]
//   instead of colour, which is then zero; without it brush and tile are zero.
// GradientRequest (1128; 0xa16): header, +16 target, +20 mode (0 rect H, 1 rect
//   V, 2 triangle), +24 reserved, +28 reserved, +32 extents (triangle: the
//   engine's prclExtents, well ordered; rect: zero), +48 Vertex[3] (16 each: x,
//   y (int32), red | green << 16, blue; rect uses vertices 0 (UpperLeft) and 1
//   (LowerRight) and vertex 2 is zero), +96 rect_count, +100 reserved,
//   +104 Rect[64].
// StretchRopRequest (1440; 0xa17): one dib/stretchblt.c DIB_XXBPP_StretchBlt
//   call of EngStretchBltROP. header, +16 target, +20 rop4 (<= 0xffff), +24 flags
//   (bit 0 mirror x, bit 1 mirror y, bit 2 tile, bit 3 running phase: with a tile
//   and a mask the tile column is ((bounds.left + d) - brush_x) & 7, d = the
//   drawn pixels left of x in the row, as DIB_XXBPP_StretchBlt PatternX),
//   +28 brush_colour, +32 brush_x,
//   +36 brush_y (int32), +40 mono_foreground, +44 mono_background, +48 bounds
//   (the well-ordered DIB DestRect: map origin and denominator), +64 source
//   operand, +88 map (left, top, width, height: the DIB SourceRect in operand
//   coordinates, width/height -16384..16384), +104 window (Rect in operand
//   coordinates: the sample domain), +120 mask operand (mono1 staging or none),
//   +144 mask_x, +148 mask_y (int32), +152 rect_count, +156 reserved,
//   +160 Rect[64], +1184 8x8 tile (64 BGRX dwords; zero without bit 2).
//   Pixel (x, y) of a rectangle samples sx = left + q or, mirrored, left +
//   width - q with q = (x - bounds.left) * width / bounds_width truncated toward
//   zero as C does (sy alike; a negative width walks the other way). Mirror is
//   the DIB-level orientation: the call's flip XOR a negative source extent.
//   With a mask, mask pixel (sx + mask_x, sy + mask_y) outside the mask or set
//   skips the pixel. A sample outside the window reads 0 and skips the
//   pixel when the low ROP3 is SRCCOPY. P is the tile at ((x - brush_x) & 7,
//   (y - brush_y) & 7) or brush_colour; the result is the low ROP3 of P, S, D.
//   The source is present exactly when the ROP4 reads it (mono colours only
//   with a mono1 source; map, window and mask zero without one); a staging
//   source (xrgb8888, mono1, or a translated format: S = its table entry)
//   window is the whole operand; a surface window lies inside it.
// ---------------------------------------------------------------------------
constexpr uint32_t operand_none = 0;
constexpr uint32_t operand_surface = 1;
constexpr uint32_t operand_staging = 2;
constexpr uint32_t rop_flag_colour_key = 1U << 0;
constexpr uint32_t rop_flag_skip_outside_source = 1U << 1;
constexpr uint32_t rop_flags_known = rop_flag_colour_key | rop_flag_skip_outside_source;
constexpr uint32_t text_mode_mono = 1;
constexpr uint32_t text_mode_coverage8 = 2;
constexpr uint32_t alpha_flag_source_alpha = 1U << 0;
constexpr uint32_t line_flag_styled = 1U << 0;
constexpr uint32_t line_flag_pattern = 1U << 1;
constexpr uint32_t stretch_flag_mirror_x = 1U << 0;
constexpr uint32_t stretch_flag_mirror_y = 1U << 1;
constexpr uint32_t stretch_flag_tile = 1U << 2;
constexpr uint32_t stretch_flag_running_phase = 1U << 3;
constexpr uint32_t stretch_flags_known = stretch_flag_mirror_x | stretch_flag_mirror_y | stretch_flag_tile |
                                         stretch_flag_running_phase;
constexpr uint32_t gradient_rect_h = 0;
constexpr uint32_t gradient_rect_v = 1;
constexpr uint32_t gradient_triangle = 2;
// Geometry bound for coordinates that are not checked against a surface.
constexpr int32_t max_coordinate = (1 << 24) - 1;
// Tallest gradient triangle (rows) a compute backend plans; taller ones are
// declined and the client renders them otherwise.
constexpr uint32_t max_triangle_rows = 512;

struct Operand final {
    uint32_t kind = operand_none;
    uint32_t handle = 0;      // surface handle or staging offset
    uint32_t pitch = 0;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t format = 0;
};

struct SourceMap final {
    int32_t left = 0;
    int32_t top = 0;
    int32_t width = 0;
    int32_t height = 0;
};

struct RopCommand final {
    Header session{};
    uint32_t target = 0;
    uint32_t rop4 = 0;
    uint32_t flags = 0;
    uint32_t brush_colour = 0;
    int32_t brush_x = 0;
    int32_t brush_y = 0;
    uint32_t mono_foreground = 0;
    uint32_t mono_background = 0;
    uint32_t colour_key = 0;
    Rect bounds{};
    Operand source{};
    SourceMap map{};
    Operand mask{};
    int32_t mask_x = 0;
    int32_t mask_y = 0;
    bool has_pattern = false;
    uint32_t pattern[64]{};
    RectList rects{};
};

struct TextCommand final {
    Header session{};
    uint32_t target = 0;
    uint32_t mode = 0;
    uint32_t colour = 0;
    Rect bounds{};
    Operand mask{};
    int32_t mask_x = 0;
    int32_t mask_y = 0;
    RectList rects{};
};

struct AlphaCommand final {
    Header session{};
    uint32_t target = 0;
    uint32_t constant_alpha = 0;
    uint32_t flags = 0;
    Rect bounds{};
    Operand source{};
    SourceMap map{};
    RectList rects{};
};

struct LineCommand final {
    Header session{};
    uint32_t target = 0;
    uint32_t colour = 0;
    uint32_t rop3 = 0;
    uint32_t flags = 0;
    int32_t x0 = 0;
    int32_t y0 = 0;
    int32_t x1 = 0;
    int32_t y1 = 0;
    uint32_t style_bit_count = 0;
    uint8_t style[style_bytes]{};
    RectList rects{};
    int32_t brush_x = 0;      // line_flag_pattern only
    int32_t brush_y = 0;
    uint32_t pattern[64]{};
};

struct StretchRopCommand final {
    Header session{};
    uint32_t target = 0;
    uint32_t rop4 = 0;
    uint32_t flags = 0;
    uint32_t brush_colour = 0;
    int32_t brush_x = 0;
    int32_t brush_y = 0;
    uint32_t mono_foreground = 0;
    uint32_t mono_background = 0;
    Rect bounds{};
    Operand source{};
    SourceMap map{};
    Rect window{};
    Operand mask{};
    int32_t mask_x = 0;
    int32_t mask_y = 0;
    RectList rects{};
    uint32_t pattern[64]{};
};

struct Vertex final {
    int32_t x = 0;
    int32_t y = 0;
    uint16_t red = 0;
    uint16_t green = 0;
    uint16_t blue = 0;
};

struct GradientCommand final {
    Header session{};
    uint32_t target = 0;
    uint32_t mode = 0;
    Rect extents{};
    Vertex vertices[3]{};
    RectList rects{};
};

namespace detail {

constexpr bool in_coordinate_range(int32_t value) noexcept {
    return value >= -max_coordinate && value <= max_coordinate;
}

// Well-ordered, 1..max_surface_dimension per axis, inside the coordinate range.
constexpr bool valid_bounds(const Rect& rect) noexcept {
    return in_coordinate_range(rect.left) && in_coordinate_range(rect.top) &&
           in_coordinate_range(rect.right) && in_coordinate_range(rect.bottom) &&
           rect.left < rect.right && rect.top < rect.bottom &&
           static_cast<int64_t>(rect.right) - rect.left <= max_surface_dimension &&
           static_cast<int64_t>(rect.bottom) - rect.top <= max_surface_dimension;
}

inline bool decode_operand(ByteView input, size_t offset, Operand& output) noexcept {
    if (!input.read_u32(offset, output.kind) || !input.read_u32(offset + 4, output.handle) ||
        !input.read_u32(offset + 8, output.pitch) || !input.read_u32(offset + 12, output.width) ||
        !input.read_u32(offset + 16, output.height) || !input.read_u32(offset + 20, output.format)) {
        return false;
    }

    switch (output.kind) {
    case operand_none:
        return output.handle == 0 && output.pitch == 0 && output.width == 0 && output.height == 0 &&
               output.format == 0;
    case operand_surface:
        return output.pitch == 0 && output.width == 0 && output.height == 0 &&
               output.format == format_xrgb8888;
    case operand_staging:
        return output.format >= format_xrgb8888 && output.format <= format_rgb24 &&
               output.width != 0 && output.height != 0 && output.width <= max_surface_dimension &&
               output.height <= max_surface_dimension && (output.handle & 3U) == 0 &&
               (output.pitch & 3U) == 0 && output.pitch <= max_staging_pitch &&
               staging_min_pitch(output.format, output.width) <= output.pitch;
    default:
        return false;
    }
}

inline bool encode_operand(const sdma::CommandBuffer& output, size_t offset, const Operand& value) noexcept {
    return output.write_u32(offset, value.kind) && output.write_u32(offset + 4, value.handle) &&
           output.write_u32(offset + 8, value.pitch) && output.write_u32(offset + 12, value.width) &&
           output.write_u32(offset + 16, value.height) && output.write_u32(offset + 20, value.format);
}

inline bool decode_map(ByteView input, size_t offset, SourceMap& output) noexcept {
    return read_i32(input, offset, output.left) && read_i32(input, offset + 4, output.top) &&
           read_i32(input, offset + 8, output.width) && read_i32(input, offset + 12, output.height) &&
           in_coordinate_range(output.left) && in_coordinate_range(output.top) && output.width > 0 &&
           output.height > 0 && output.width <= static_cast<int32_t>(max_surface_dimension) &&
           output.height <= static_cast<int32_t>(max_surface_dimension);
}

inline bool encode_map(const sdma::CommandBuffer& output, size_t offset, const SourceMap& value) noexcept {
    return output.write_u32(offset, to_unsigned(value.left)) &&
           output.write_u32(offset + 4, to_unsigned(value.top)) &&
           output.write_u32(offset + 8, to_unsigned(value.width)) &&
           output.write_u32(offset + 12, to_unsigned(value.height));
}

inline bool encode_rect_at(const sdma::CommandBuffer& output, size_t offset, const Rect& rect) noexcept {
    return output.write_u32(offset, to_unsigned(rect.left)) &&
           output.write_u32(offset + 4, to_unsigned(rect.top)) &&
           output.write_u32(offset + 8, to_unsigned(rect.right)) &&
           output.write_u32(offset + 12, to_unsigned(rect.bottom));
}

// Every rectangle of the list inside bounds (the surface check is the miniport's).
inline bool rects_within(const RectList& list, const Rect& bounds) noexcept {
    for (uint32_t index = 0; index < list.count && index < max_rects; ++index) {
        const Rect& rect = list.rects[index];
        if (rect.left < bounds.left || rect.top < bounds.top || rect.right > bounds.right ||
            rect.bottom > bounds.bottom) {
            return false;
        }
    }

    return true;
}

} // namespace detail

inline bool decode_rop(ByteView input, bool pattern, RopCommand& output) noexcept {
    uint32_t count = 0;
    const uint32_t bytes = pattern ? pattern_request_bytes : rop_request_bytes;
    if (!detail::decode_header(input, bytes, output.session) || !input.read_u32(16, output.target) ||
        !input.read_u32(20, output.rop4) || !input.read_u32(24, output.flags) ||
        !input.read_u32(28, output.brush_colour) || !detail::read_i32(input, 32, output.brush_x) ||
        !detail::read_i32(input, 36, output.brush_y) || !input.read_u32(40, output.mono_foreground) ||
        !input.read_u32(44, output.mono_background) || !input.read_u32(48, output.colour_key) ||
        !detail::expect_zero(input, 52) || !detail::decode_rect(input, 56, output.bounds) ||
        !detail::decode_operand(input, 72, output.source) || !detail::decode_map(input, 96, output.map) ||
        !detail::decode_operand(input, 112, output.mask) || !detail::read_i32(input, 136, output.mask_x) ||
        !detail::read_i32(input, 140, output.mask_y) || !input.read_u32(144, count) ||
        !detail::expect_zero(input, 148) ||
        !detail::decode_rects(input, rop_rects_offset, count, output.rects)) {
        return false;
    }

    output.has_pattern = pattern;
    for (uint32_t index = 0; index < 64; ++index) {
        output.pattern[index] = 0;
        if (pattern && !input.read_u32(rop_request_bytes + index * 4U, output.pattern[index])) {
            return false;
        }
    }

    const bool mask_ok = output.mask.kind == operand_none ||
        (output.mask.kind == operand_staging && output.mask.format == format_mono1);
    const bool source_ok = output.source.kind == operand_none || output.source.kind == operand_surface ||
        output.source.format == format_xrgb8888 || output.source.format == format_mono1 ||
        translated_format(output.source.format);
    return output.rop4 <= 0xffffU && (output.flags & ~rop_flags_known) == 0 &&
           detail::valid_bounds(output.bounds) && mask_ok && source_ok &&
           detail::in_coordinate_range(output.brush_x) && detail::in_coordinate_range(output.brush_y) &&
           detail::in_coordinate_range(output.mask_x) && detail::in_coordinate_range(output.mask_y) &&
           detail::rects_within(output.rects, output.bounds);
}

inline bool encode_rop(const sdma::CommandBuffer& output, const RopCommand& command) noexcept {
    const uint32_t bytes = command.has_pattern ? pattern_request_bytes : rop_request_bytes;
    if (!detail::encode_header(output, bytes, command.session) || !output.write_u32(16, command.target) ||
        !output.write_u32(20, command.rop4) || !output.write_u32(24, command.flags) ||
        !output.write_u32(28, command.brush_colour) ||
        !output.write_u32(32, detail::to_unsigned(command.brush_x)) ||
        !output.write_u32(36, detail::to_unsigned(command.brush_y)) ||
        !output.write_u32(40, command.mono_foreground) || !output.write_u32(44, command.mono_background) ||
        !output.write_u32(48, command.colour_key) || !detail::encode_rect_at(output, 56, command.bounds) ||
        !detail::encode_operand(output, 72, command.source) || !detail::encode_map(output, 96, command.map) ||
        !detail::encode_operand(output, 112, command.mask) ||
        !output.write_u32(136, detail::to_unsigned(command.mask_x)) ||
        !output.write_u32(140, detail::to_unsigned(command.mask_y)) ||
        !output.write_u32(144, command.rects.count) ||
        !detail::encode_rects(output, rop_rects_offset, command.rects)) {
        return false;
    }

    for (uint32_t index = 0; command.has_pattern && index < 64; ++index) {
        if (!output.write_u32(rop_request_bytes + index * 4U, command.pattern[index])) {
            return false;
        }
    }

    return true;
}

inline bool decode_text(ByteView input, TextCommand& output) noexcept {
    uint32_t count = 0;
    if (!detail::decode_header(input, text_request_bytes, output.session) ||
        !input.read_u32(16, output.target) || !input.read_u32(20, output.mode) ||
        !input.read_u32(24, output.colour) || !detail::expect_zero(input, 28) ||
        !detail::decode_rect(input, 32, output.bounds) || !detail::decode_operand(input, 48, output.mask) ||
        !detail::read_i32(input, 72, output.mask_x) || !detail::read_i32(input, 76, output.mask_y) ||
        !input.read_u32(80, count) || !detail::expect_zero(input, 84) ||
        !detail::decode_rects(input, text_rects_offset, count, output.rects)) {
        return false;
    }

    const uint32_t format = output.mode == text_mode_mono ? format_mono1 :
                            output.mode == text_mode_coverage8 ? format_coverage8 : 0;
    return format != 0 && output.mask.kind == operand_staging && output.mask.format == format &&
           detail::valid_bounds(output.bounds) && detail::in_coordinate_range(output.mask_x) &&
           detail::in_coordinate_range(output.mask_y) && detail::rects_within(output.rects, output.bounds);
}

inline bool encode_text(const sdma::CommandBuffer& output, const TextCommand& command) noexcept {
    return detail::encode_header(output, text_request_bytes, command.session) &&
           output.write_u32(16, command.target) && output.write_u32(20, command.mode) &&
           output.write_u32(24, command.colour) && detail::encode_rect_at(output, 32, command.bounds) &&
           detail::encode_operand(output, 48, command.mask) &&
           output.write_u32(72, detail::to_unsigned(command.mask_x)) &&
           output.write_u32(76, detail::to_unsigned(command.mask_y)) &&
           output.write_u32(80, command.rects.count) &&
           detail::encode_rects(output, text_rects_offset, command.rects);
}

inline bool decode_alpha(ByteView input, AlphaCommand& output) noexcept {
    uint32_t count = 0;
    return detail::decode_header(input, alpha_request_bytes, output.session) &&
           input.read_u32(16, output.target) && input.read_u32(20, output.constant_alpha) &&
           input.read_u32(24, output.flags) && detail::expect_zero(input, 28) &&
           detail::decode_rect(input, 32, output.bounds) && detail::decode_operand(input, 48, output.source) &&
           detail::decode_map(input, 72, output.map) && input.read_u32(88, count) &&
           detail::expect_zero(input, 92) &&
           detail::decode_rects(input, alpha_rects_offset, count, output.rects) &&
           output.constant_alpha <= 255U && (output.flags & ~alpha_flag_source_alpha) == 0 &&
           output.source.kind != operand_none && output.source.format == format_xrgb8888 &&
           detail::valid_bounds(output.bounds) && detail::rects_within(output.rects, output.bounds);
}

inline bool encode_alpha(const sdma::CommandBuffer& output, const AlphaCommand& command) noexcept {
    return detail::encode_header(output, alpha_request_bytes, command.session) &&
           output.write_u32(16, command.target) && output.write_u32(20, command.constant_alpha) &&
           output.write_u32(24, command.flags) && detail::encode_rect_at(output, 32, command.bounds) &&
           detail::encode_operand(output, 48, command.source) && detail::encode_map(output, 72, command.map) &&
           output.write_u32(88, command.rects.count) &&
           detail::encode_rects(output, alpha_rects_offset, command.rects);
}

inline bool decode_line(ByteView input, LineCommand& output) noexcept {
    uint32_t count = 0;
    if (!detail::decode_header(input, line_request_bytes, output.session) ||
        !input.read_u32(16, output.target) || !input.read_u32(20, output.colour) ||
        !input.read_u32(24, output.rop3) || !input.read_u32(28, output.flags) ||
        !detail::read_i32(input, 32, output.x0) || !detail::read_i32(input, 36, output.y0) ||
        !detail::read_i32(input, 40, output.x1) || !detail::read_i32(input, 44, output.y1) ||
        !input.read_u32(48, output.style_bit_count) || !detail::expect_zero(input, 52) ||
        !input.read_u32(312, count) || !detail::expect_zero(input, 316) ||
        !detail::decode_rects(input, line_rects_offset, count, output.rects)) {
        return false;
    }

    const bool styled = (output.flags & line_flag_styled) != 0;
    const bool pattern = (output.flags & line_flag_pattern) != 0;
    if (!detail::read_i32(input, line_brush_offset, output.brush_x) ||
        !detail::read_i32(input, line_brush_offset + 4, output.brush_y)) {
        return false;
    }

    for (uint32_t index = 0; index < 64; ++index) {
        if (!input.read_u32(line_tile_offset + index * 4U, output.pattern[index]) ||
            (!pattern && output.pattern[index] != 0)) {
            return false;
        }
    }

    if (output.rop3 > 0xffU || (output.flags & ~(line_flag_styled | line_flag_pattern)) != 0 ||
        (styled && pattern) ||
        (pattern ? output.colour != 0 || !detail::in_coordinate_range(output.brush_x) ||
                       !detail::in_coordinate_range(output.brush_y) :
                   output.brush_x != 0 || output.brush_y != 0) ||
        (styled ? output.style_bit_count == 0 || output.style_bit_count > max_style_bits :
                  output.style_bit_count != 0) ||
        !detail::in_coordinate_range(output.x0) || !detail::in_coordinate_range(output.y0) ||
        !detail::in_coordinate_range(output.x1) || !detail::in_coordinate_range(output.y1)) {
        return false;
    }

    for (uint32_t index = 0; index < style_bytes; ++index) {
        uint8_t value = 0;
        if (!input.read_u8(56 + index, value)) {
            return false;
        }

        // Bits at and above the count are zero (MSB-first within each byte).
        const uint32_t first_bit = index * 8U;
        const uint32_t valid = output.style_bit_count <= first_bit ? 0U :
            (output.style_bit_count - first_bit >= 8U ? 8U : output.style_bit_count - first_bit);
        const uint32_t allowed = valid == 0 ? 0U : (0xffU << (8U - valid)) & 0xffU;
        if ((value & ~allowed & 0xffU) != 0) {
            return false;
        }

        output.style[index] = value;
    }

    return true;
}

inline bool encode_line(const sdma::CommandBuffer& output, const LineCommand& command) noexcept {
    if (!detail::encode_header(output, line_request_bytes, command.session) ||
        !output.write_u32(16, command.target) || !output.write_u32(20, command.colour) ||
        !output.write_u32(24, command.rop3) || !output.write_u32(28, command.flags) ||
        !output.write_u32(32, detail::to_unsigned(command.x0)) ||
        !output.write_u32(36, detail::to_unsigned(command.y0)) ||
        !output.write_u32(40, detail::to_unsigned(command.x1)) ||
        !output.write_u32(44, detail::to_unsigned(command.y1)) ||
        !output.write_u32(48, command.style_bit_count) || !output.write_u32(312, command.rects.count) ||
        !detail::encode_rects(output, line_rects_offset, command.rects) ||
        !output.write_u32(line_brush_offset, detail::to_unsigned(command.brush_x)) ||
        !output.write_u32(line_brush_offset + 4, detail::to_unsigned(command.brush_y))) {
        return false;
    }

    for (uint32_t index = 0; index < 64; ++index) {
        if (!output.write_u32(line_tile_offset + index * 4U, command.pattern[index])) {
            return false;
        }
    }

    // Style bytes as dwords (little-endian byte order).
    for (uint32_t index = 0; index < style_bytes; index += 4) {
        const uint32_t word = static_cast<uint32_t>(command.style[index]) |
            (static_cast<uint32_t>(command.style[index + 1]) << 8) |
            (static_cast<uint32_t>(command.style[index + 2]) << 16) |
            (static_cast<uint32_t>(command.style[index + 3]) << 24);
        if (!output.write_u32(56 + index, word)) {
            return false;
        }
    }

    return true;
}

inline bool decode_gradient(ByteView input, GradientCommand& output) noexcept {
    uint32_t count = 0;
    if (!detail::decode_header(input, gradient_request_bytes, output.session) ||
        !input.read_u32(16, output.target) || !input.read_u32(20, output.mode) ||
        !detail::expect_zero(input, 24) || !detail::expect_zero(input, 28) ||
        !detail::decode_rect(input, 32, output.extents) || !input.read_u32(96, count) ||
        !detail::expect_zero(input, 100) ||
        !detail::decode_rects(input, gradient_rects_offset, count, output.rects) ||
        output.mode > gradient_triangle) {
        return false;
    }

    const uint32_t used = output.mode == gradient_triangle ? 3U : 2U;
    for (uint32_t index = 0; index < 3; ++index) {
        const size_t offset = 48 + index * 16U;
        Vertex& vertex = output.vertices[index];
        uint32_t red_green = 0;
        uint32_t blue = 0;
        if (!detail::read_i32(input, offset, vertex.x) || !detail::read_i32(input, offset + 4, vertex.y) ||
            !input.read_u32(offset + 8, red_green) || !input.read_u32(offset + 12, blue) || blue > 0xffffU) {
            return false;
        }

        vertex.red = static_cast<uint16_t>(red_green & 0xffffU);
        vertex.green = static_cast<uint16_t>(red_green >> 16);
        vertex.blue = static_cast<uint16_t>(blue);
        if (index >= used ? vertex.x != 0 || vertex.y != 0 || red_green != 0 || blue != 0 :
                            !detail::in_coordinate_range(vertex.x) || !detail::in_coordinate_range(vertex.y)) {
            return false;
        }
    }

    const Rect zero{};
    return output.mode == gradient_triangle ?
        detail::valid_bounds(output.extents) :
        output.extents.left == zero.left && output.extents.top == zero.top &&
            output.extents.right == zero.right && output.extents.bottom == zero.bottom;
}

inline bool encode_gradient(const sdma::CommandBuffer& output, const GradientCommand& command) noexcept {
    if (!detail::encode_header(output, gradient_request_bytes, command.session) ||
        !output.write_u32(16, command.target) || !output.write_u32(20, command.mode) ||
        !detail::encode_rect_at(output, 32, command.extents) || !output.write_u32(96, command.rects.count) ||
        !detail::encode_rects(output, gradient_rects_offset, command.rects)) {
        return false;
    }

    for (uint32_t index = 0; index < 3; ++index) {
        const size_t offset = 48 + index * 16U;
        const Vertex& vertex = command.vertices[index];
        if (!output.write_u32(offset, detail::to_unsigned(vertex.x)) ||
            !output.write_u32(offset + 4, detail::to_unsigned(vertex.y)) ||
            !output.write_u32(offset + 8, static_cast<uint32_t>(vertex.red) |
                                             (static_cast<uint32_t>(vertex.green) << 16)) ||
            !output.write_u32(offset + 12, vertex.blue)) {
            return false;
        }
    }

    return true;
}

namespace detail {

// A stretch map: like decode_map, but a DIB SourceRect may be empty or inverted
// (negative extent) on an axis.
inline bool decode_stretch_map(ByteView input, size_t offset, SourceMap& output) noexcept {
    constexpr int32_t limit = static_cast<int32_t>(max_surface_dimension);
    return read_i32(input, offset, output.left) && read_i32(input, offset + 4, output.top) &&
           read_i32(input, offset + 8, output.width) && read_i32(input, offset + 12, output.height) &&
           in_coordinate_range(output.left) && in_coordinate_range(output.top) && output.width >= -limit &&
           output.height >= -limit && output.width <= limit && output.height <= limit;
}

constexpr bool zero_rect(const Rect& rect) noexcept {
    return rect.left == 0 && rect.top == 0 && rect.right == 0 && rect.bottom == 0;
}

} // namespace detail

inline bool decode_stretch_rop(ByteView input, StretchRopCommand& output) noexcept {
    uint32_t count = 0;
    if (!detail::decode_header(input, stretch_rop_request_bytes, output.session) ||
        !input.read_u32(16, output.target) || !input.read_u32(20, output.rop4) ||
        !input.read_u32(24, output.flags) || !input.read_u32(28, output.brush_colour) ||
        !detail::read_i32(input, 32, output.brush_x) || !detail::read_i32(input, 36, output.brush_y) ||
        !input.read_u32(40, output.mono_foreground) || !input.read_u32(44, output.mono_background) ||
        !detail::decode_rect(input, 48, output.bounds) || !detail::decode_operand(input, 64, output.source) ||
        !detail::read_i32(input, 88, output.map.left) || !detail::read_i32(input, 92, output.map.top) ||
        !detail::read_i32(input, 96, output.map.width) || !detail::read_i32(input, 100, output.map.height) ||
        !detail::decode_rect(input, 104, output.window) || !detail::decode_operand(input, 120, output.mask) ||
        !detail::read_i32(input, 144, output.mask_x) || !detail::read_i32(input, 148, output.mask_y) ||
        !input.read_u32(152, count) || !detail::expect_zero(input, 156) ||
        !detail::decode_rects(input, stretch_rop_rects_offset, count, output.rects)) {
        return false;
    }

    const bool tile = (output.flags & stretch_flag_tile) != 0;
    for (uint32_t index = 0; index < 64; ++index) {
        if (!input.read_u32(stretch_rop_tile_offset + index * 4U, output.pattern[index]) ||
            (!tile && output.pattern[index] != 0)) {
            return false;
        }
    }

    const bool has_source = output.source.kind != operand_none;
    const bool mono = has_source && output.source.kind == operand_staging && output.source.format == format_mono1;
    const bool translated = has_source && output.source.kind == operand_staging &&
                            translated_format(output.source.format);
    const bool has_mask = output.mask.kind != operand_none;
    const bool running = (output.flags & stretch_flag_running_phase) != 0;
    const Rect& window = output.window;
    const bool source_ok = has_source ?
        (output.source.kind == operand_surface || output.source.format == format_xrgb8888 || mono || translated) &&
            detail::decode_stretch_map(input, 88, output.map) && window.left < window.right &&
            window.top < window.bottom && detail::in_coordinate_range(window.left) &&
            detail::in_coordinate_range(window.top) && detail::in_coordinate_range(window.right) &&
            detail::in_coordinate_range(window.bottom) :
        output.map.left == 0 && output.map.top == 0 && output.map.width == 0 && output.map.height == 0 &&
            detail::zero_rect(window);
    const bool mask_ok = has_mask ?
        has_source && output.mask.kind == operand_staging && output.mask.format == format_mono1 &&
            detail::in_coordinate_range(output.mask_x) && detail::in_coordinate_range(output.mask_y) :
        output.mask_x == 0 && output.mask_y == 0;
    return output.rop4 <= 0xffffU && (output.flags & ~stretch_flags_known) == 0 && source_ok && mask_ok &&
           (!running || (tile && has_mask)) &&
           (mono || (output.mono_foreground == 0 && output.mono_background == 0)) &&
           (!tile || output.brush_colour == 0) && detail::in_coordinate_range(output.brush_x) &&
           detail::in_coordinate_range(output.brush_y) && detail::valid_bounds(output.bounds) &&
           detail::rects_within(output.rects, output.bounds);
}

inline bool encode_stretch_rop(const sdma::CommandBuffer& output, const StretchRopCommand& command) noexcept {
    if (!detail::encode_header(output, stretch_rop_request_bytes, command.session) ||
        !output.write_u32(16, command.target) || !output.write_u32(20, command.rop4) ||
        !output.write_u32(24, command.flags) || !output.write_u32(28, command.brush_colour) ||
        !output.write_u32(32, detail::to_unsigned(command.brush_x)) ||
        !output.write_u32(36, detail::to_unsigned(command.brush_y)) ||
        !output.write_u32(40, command.mono_foreground) || !output.write_u32(44, command.mono_background) ||
        !detail::encode_rect_at(output, 48, command.bounds) || !detail::encode_operand(output, 64, command.source) ||
        !detail::encode_map(output, 88, command.map) || !detail::encode_rect_at(output, 104, command.window) ||
        !detail::encode_operand(output, 120, command.mask) ||
        !output.write_u32(144, detail::to_unsigned(command.mask_x)) ||
        !output.write_u32(148, detail::to_unsigned(command.mask_y)) ||
        !output.write_u32(152, command.rects.count) ||
        !detail::encode_rects(output, stretch_rop_rects_offset, command.rects)) {
        return false;
    }

    for (uint32_t index = 0; index < 64; ++index) {
        if (!output.write_u32(stretch_rop_tile_offset + index * 4U, command.pattern[index])) {
            return false;
        }
    }

    return true;
}

} // namespace rx7900::reactos::radeon2d::abi

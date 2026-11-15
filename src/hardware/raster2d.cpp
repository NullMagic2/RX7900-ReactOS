/*
Module: rx7900.hardware.raster2d
Purpose: Compute 2D engine, GDI-exact argument builders and the gradient-triangle
row planner. See raster2d.hpp. GDI semantics reproduce the pinned ReactOS 0.4.16
(822e864) win32ss/gdi engine behaviour (GPL-2.0-or-later; no code copied):
eng/bitblt.c, dib/dib.c DIB_DoRop, dib/stretchblt.c, dib/dib32bpp.c (AlphaBlend,
TransparentBlt), eng/lineto.c, eng/gradient.c.
Created: 2026-10-22
*/
#include "raster2d.hpp"

namespace rx7900::hardware {
namespace {

using namespace gfx11_isa;

bool aligned4(uint64_t value) noexcept { return (value & 3u) == 0; }

bool permits(memory::Access granted, memory::Access needed) noexcept {
    return (static_cast<uint32_t>(granted) & static_cast<uint32_t>(needed)) ==
           static_cast<uint32_t>(needed);
}

bool same_handle(memory::Handle a, memory::Handle b) noexcept {
    return a.slot == b.slot && a.generation == b.generation;
}

uint64_t surface_span(const Raster2dSurface& s) noexcept {
    return uint64_t{s.height} * s.pitch_bytes;
}

// GFX11 compute SH register offsets for SET_SH_REG: (byte address - 0xB000) >> 2.
// Byte addresses are the gfx11.json register map (verified against the pinned
// gc_11_0_0 headers): COMPUTE_PGM_LO 0xB830, PGM_HI 0xB834, RSRC1 0xB848,
// RSRC2 0xB84C, RSRC3 0xB8A0, NUM_THREAD_X 0xB81C, RESOURCE_LIMITS 0xB854,
// DISPATCH_INITIATOR 0xB800, USER_DATA_0 0xB900.
constexpr uint32_t sh_offset(uint32_t byte_address) noexcept { return (byte_address - 0xB000u) >> 2; }
constexpr uint32_t off_pgm_lo = sh_offset(0xB830);
constexpr uint32_t off_pgm_rsrc1 = sh_offset(0xB848);
constexpr uint32_t off_pgm_rsrc3 = sh_offset(0xB8A0);
constexpr uint32_t off_num_thread_x = sh_offset(0xB81C);
constexpr uint32_t off_resource_limits = sh_offset(0xB854);
constexpr uint32_t off_user_data_0 = sh_offset(0xB900);

constexpr uint32_t pm4_set_sh_reg = 0x76;
constexpr uint32_t pm4_dispatch_direct = 0x15;
constexpr uint32_t pm4_shader_type_compute = 1u << 1;
// gfx_v11_0 ring funcs nop: PACKET3(NOP, 0x3FFF) is a header-only one-dword NOP
// (the CP skips no body), the value Linux pads rings and IBs with.
constexpr uint32_t pm4_nop_one_dword = 0xffff1000u;
// nvd.h PACKET3_EVENT_WRITE / PACKET3_ACQUIRE_MEM; EVENT_TYPE CS_PARTIAL_FLUSH (7)
// with EVENT_INDEX 4; GCR_CNTL GLK_INV (7) | GLV_INV (8) | GL1_INV (9).
constexpr uint32_t pm4_event_write = 0x46;
constexpr uint32_t pm4_acquire_mem = 0x58;
constexpr uint32_t event_cs_partial_flush = 7u | (4u << 8);
constexpr uint32_t acquire_poll_interval = 0x0000000au;
constexpr uint32_t gcr_invalidate_l0_l1 = (1u << 7) | (1u << 8) | (1u << 9);

uint32_t packet3(uint32_t opcode, uint32_t count) noexcept {
    return (3u << 30) | ((opcode & 0xffu) << 8) | ((count & 0x3fffu) << 16);
}

// COMPUTE_DISPATCH_INITIATOR: COMPUTE_SHADER_EN | FORCE_START_AT_000 | ORDER_MODE |
// CS_W32_EN (wave32). Matches Mesa radv device->dispatch_initiator with W32 and
// FORCE_START_AT_000 (offsets zero), minus TUNNEL_ENABLE (a high-priority-queue
// no-op not programmed here).
constexpr uint32_t dispatch_initiator = 1u | (1u << 2) | (1u << 6) | (1u << 15);

int32_t sign_mask(bool negative) noexcept { return negative ? -1 : 0; }

void write_map(uint32_t* args, uint32_t base, int32_t dref, int32_t src_base, int32_t magnitude,
               uint32_t divisor, bool negative) noexcept {
    const Raster2dDivisor d = raster2d_divisor(divisor);
    args[base + rop_args::map_dref] = static_cast<uint32_t>(dref);
    args[base + rop_args::map_base] = static_cast<uint32_t>(src_base);
    args[base + rop_args::map_mag] = static_cast<uint32_t>(magnitude);
    args[base + rop_args::map_magic] = d.magic;
    args[base + rop_args::map_shift1] = d.shift1;
    args[base + rop_args::map_shift2] = d.shift2;
    args[base + rop_args::map_neg] = static_cast<uint32_t>(sign_mask(negative));
}

void copy_descriptor(uint32_t* dst, const uint32_t (&src)[4]) noexcept {
    for (uint32_t i = 0; i < 4; ++i) {
        dst[i] = src[i];
    }
}

void write_rop_masks(uint32_t* args, uint32_t base, uint32_t rop3) noexcept {
    uint32_t masks[8];
    raster2d_rop_masks(rop3, masks);

    for (uint32_t i = 0; i < 8; ++i) {
        args[base + i] = masks[i];
    }
}

// Gradient channel ramp parameters (shared by rect and triangle spans).
void gradient_channel_args(uint32_t* args, uint32_t color_index, uint32_t delta_index,
                           uint32_t negative_index, uint32_t channel, int32_t start,
                           int32_t end) noexcept {
    args[color_index + channel] = static_cast<uint32_t>(start);
    args[delta_index + channel] = static_cast<uint32_t>(end >= start ? end - start : start - end);
    args[negative_index + channel] = static_cast<uint32_t>(sign_mask(end < start));
}

// Hazard regions for the overlap refusal (blit2d_regions_overlap compares same
// layouts exactly and other layouts as conservative byte spans).
Raster2dRect surface_area(const Raster2dSurface& s, const Raster2dRect& r) noexcept {
    Raster2dRect area = r;
    area.left = area.left < 0 ? 0 : area.left;
    area.top = area.top < 0 ? 0 : area.top;
    area.right = area.right > static_cast<int32_t>(s.width) ? static_cast<int32_t>(s.width) : area.right;
    area.bottom = area.bottom > static_cast<int32_t>(s.height) ? static_cast<int32_t>(s.height)
                                                                : area.bottom;
    if (area.right < area.left) {
        area.right = area.left;
    }

    if (area.bottom < area.top) {
        area.bottom = area.top;
    }

    return area;
}

Blit2dRegion region(const Raster2dSurface& s, const Raster2dRect& rect) noexcept {
    return {s.gpu_address, s.pitch_bytes, s.height, rect};
}

// Every byte of a byte-addressed (mono1/coverage8) operand.
Blit2dRegion whole_region(const Raster2dSurface& s) noexcept {
    return {s.gpu_address, s.pitch_bytes, s.height,
            {0, 0, static_cast<int32_t>((s.pitch_bytes + 3u) / 4u), static_cast<int32_t>(s.height)}};
}

// The source rows/columns a mapping can read, clamped to the surface; mono1
// sources are treated as whole byte spans.
Blit2dRegion source_region(const Raster2dSurface& s, int32_t left, int32_t top, int32_t width,
                           int32_t height) noexcept {
    if (s.format != Raster2dFormat::bgrx8888 || uint64_t{s.width} * raster2d_bytes_per_pixel > s.pitch_bytes) {
        return whole_region(s);
    }

    const int64_t right = int64_t{left} + (width > 0 ? width : 0);
    const int64_t bottom = int64_t{top} + (height > 0 ? height : 0);
    Raster2dRect rect{left, top, static_cast<int32_t>(right > INT32_MAX ? INT32_MAX : right),
                      static_cast<int32_t>(bottom > INT32_MAX ? INT32_MAX : bottom)};
    return region(s, surface_area(s, rect));
}

// An operation writes only the intersection of its destination rectangle and
// each clip rectangle. Keep the source range conservative, but do not reject a
// same-surface read merely because an unpainted part of the operation overlaps.
bool overlaps_clipped_writes(const Blit2dRegion& read, const Raster2dSurface& destination,
                             const Raster2dRect& bounds, const Raster2dClipList& clip) noexcept {
    const uint32_t count = clip.count == 0 ? 1u : clip.count;
    for (uint32_t index = 0; index < count; ++index) {
        const Raster2dRect& c = clip.count == 0 ? bounds : clip.rects[index];
        const Raster2dRect visible = surface_area(destination,
            {c.left > bounds.left ? c.left : bounds.left,
             c.top > bounds.top ? c.top : bounds.top,
             c.right < bounds.right ? c.right : bounds.right,
             c.bottom < bounds.bottom ? c.bottom : bounds.bottom});
        if (visible.right > visible.left && visible.bottom > visible.top &&
            blit2d_regions_overlap(read, region(destination, visible))) {
            return true;
        }
    }
    return false;
}

// Lowest source coordinate a nearest map can read on one axis, and the number
// of coordinates it spans: base + [0, num) forward, base - [0, num] mirrored
// (floor((d - dref) * num / den) <= num - 1 forward is bounded by num itself
// when mirrored: the first column reads base).
int32_t mapped_low(int32_t base, int32_t num, int32_t sign) noexcept {
    const int64_t low = sign < 0 ? int64_t{base} - num : int64_t{base};
    return static_cast<int32_t>(low < INT32_MIN ? INT32_MIN : low);
}

int32_t mapped_extent(int32_t num, int32_t sign) noexcept {
    const int64_t extent = sign < 0 ? int64_t{num} + 1 : int64_t{num};
    return static_cast<int32_t>(extent > INT32_MAX ? INT32_MAX : extent);
}

uint32_t format_bytes_per_pixel(Raster2dFormat format) noexcept {
    switch (format) {
    case Raster2dFormat::index8: return 1;
    case Raster2dFormat::rgb16: return 2;
    case Raster2dFormat::rgb24: return 3;
    default: return raster2d_bytes_per_pixel;
    }
}

// Bytes a read operand's pixels occupy: every row's pitch except the last,
// whose pixel bytes are rounded up to a dword (the widest kernel load). A
// cropped window (a dispatcher sample window) therefore never lets the raw
// descriptor reach past its surface's last pixel.
uint64_t read_span(const Raster2dSurface& s, Raster2dFormat role) noexcept {
    const uint64_t pixels = role == Raster2dFormat::mono1 ? (uint64_t{s.width} + 7u) / 8u :
                            role == Raster2dFormat::coverage8 || role == Raster2dFormat::index8 ? uint64_t{s.width} :
                            role == Raster2dFormat::rgb16 ? uint64_t{s.width} * 2u :
                            role == Raster2dFormat::rgb24 ? uint64_t{s.width} * 3u :
                                                            uint64_t{s.width} * raster2d_bytes_per_pixel;
    const uint64_t last = (pixels + 3u) & ~uint64_t{3};
    const uint64_t tight = s.height == 0 ? 0 : uint64_t{s.height - 1u} * s.pitch_bytes + last;
    return tight < surface_span(s) ? tight : surface_span(s);
}

// Byte distance from a translated source's table to its first pixel row.
uint64_t translated_offset(const Raster2dRopRequest& r) noexcept {
    return r.source.gpu_address - r.source_table;
}

} // namespace

bool raster2d_translated_format(Raster2dFormat format) noexcept {
    return format == Raster2dFormat::index8 || format == Raster2dFormat::rgb16 || format == Raster2dFormat::rgb24;
}

uint32_t raster2d_table_bytes(Raster2dFormat format) noexcept {
    switch (format) {
    case Raster2dFormat::index8: return rop_args::index8_table_bytes;
    case Raster2dFormat::rgb16: return rop_args::rgb16_table_bytes;
    case Raster2dFormat::rgb24: return rop_args::rgb24_table_bytes;
    default: return 0;
    }
}

void raster2d_rop_source_descriptor(const Raster2dRopRequest& r, uint32_t (&words)[4]) noexcept {
    if (!r.source_monochrome && raster2d_translated_format(r.source.format)) {
        raster2d_buffer_descriptor(r.source_table,
                                   static_cast<uint32_t>(translated_offset(r) + read_span(r.source, r.source.format)), words);
        return;
    }

    raster2d_buffer_descriptor(r.source.gpu_address,
                               static_cast<uint32_t>(read_span(r.source, r.source_monochrome ? Raster2dFormat::mono1 :
                                                                                               Raster2dFormat::bgrx8888)),
                               words);
}

// ------------------------------------------------------------------ builders

void raster2d_build_rop_args(const Raster2dRopRequest& r, const Raster2dRect& rect,
                             const Raster2dDescriptors& d, uint32_t (&args)[64]) noexcept {
    using namespace rop_args;

    for (uint32_t& value : args) {
        value = 0;
    }

    copy_descriptor(args + destination, d.destination);
    copy_descriptor(args + source, d.source);
    copy_descriptor(args + mask, d.mask);
    copy_descriptor(args + pattern, d.pattern);

    args[left] = static_cast<uint32_t>(rect.left);
    args[top] = static_cast<uint32_t>(rect.top);
    args[right] = static_cast<uint32_t>(rect.right);
    args[destination_pitch] = r.destination.pitch_bytes;
    args[source_pitch] = r.source.pitch_bytes;
    args[mask_pitch] = r.mask.pitch_bytes;
    args[solid] = r.brush_colour;

    const uint32_t fg = r.rop4 & 0xffu;
    const uint32_t bg = (r.rop4 >> 8) & 0xffu;
    write_rop_masks(args, foreground, fg);
    write_rop_masks(args, background, bg);

    uint32_t flag_bits = flag_load_destination;

    if (r.has_source) {
        const Raster2dFormat format = r.source_monochrome ? Raster2dFormat::mono1 : r.source.format;
        switch (format) {
        case Raster2dFormat::mono1: flag_bits |= flag_source_mono; break;
        case Raster2dFormat::index8: flag_bits |= flag_source_index8; break;
        case Raster2dFormat::rgb16: flag_bits |= flag_source_rgb16; break;
        case Raster2dFormat::rgb24: flag_bits |= flag_source_rgb24; break;
        default: flag_bits |= flag_source_color; break;
        }

        if (raster2d_translated_format(format)) {
            args[source_offset] = static_cast<uint32_t>(translated_offset(r));
        } else {
            args[mono0] = r.mono_background;
            args[mono1] = r.mono_foreground;
        }

        args[source_width] = r.source.width;
        args[source_height] = r.source.height;
        // dst axis origin is the operation rect, not the clip sub-rect.
        write_map(args, map_x, r.destination_rect.left, r.source_left, r.source_width,
                  static_cast<uint32_t>(r.destination_rect.right - r.destination_rect.left),
                  r.source_sign_x < 0);
        write_map(args, map_y, r.destination_rect.top, r.source_top, r.source_height,
                  static_cast<uint32_t>(r.destination_rect.bottom - r.destination_rect.top),
                  r.source_sign_y < 0);

        if (r.skip_outside_source) {
            flag_bits |= flag_skip_outside_source;
        }
    }

    if (r.has_colour_key) {
        flag_bits |= flag_color_key;
        args[key_mask] = 0x00ffffffu;
        args[key] = r.colour_key & 0x00ffffffu;
    }

    if (r.has_pattern) {
        flag_bits |= flag_pattern_tile;
        args[brush_x] = static_cast<uint32_t>(r.brush_origin.x);
        args[brush_y] = static_cast<uint32_t>(r.brush_origin.y);

        if (r.running_phase && r.has_mask && r.stretch_mask) {
            flag_bits |= flag_pattern_running;
        }
    }

    if (r.has_mask && r.stretch_mask) {
        // The background ROP is never evaluated: its slots carry the mask size.
        flag_bits |= flag_stretch_mask;
        args[mask_dx] = static_cast<uint32_t>(r.mask_origin.x);
        args[mask_dy] = static_cast<uint32_t>(r.mask_origin.y);
        args[stretch_mask_width] = r.mask.width;
        args[stretch_mask_height] = r.mask.height;
    } else if (r.has_mask) {
        flag_bits |= flag_mask;
        args[mask_dx] = static_cast<uint32_t>(r.mask_origin.x - r.destination_rect.left);
        args[mask_dy] = static_cast<uint32_t>(r.mask_origin.y - r.destination_rect.top);
    }

    args[flags] = flag_bits;
}

void raster2d_build_alpha_args(const Raster2dAlphaRequest& r, const Raster2dRect& rect,
                               const Raster2dDescriptors& d, uint32_t (&args)[64]) noexcept {
    using namespace alpha_args;

    for (uint32_t& value : args) {
        value = 0;
    }

    copy_descriptor(args + destination, d.destination);
    copy_descriptor(args + source, d.source);
    args[left] = static_cast<uint32_t>(rect.left);
    args[top] = static_cast<uint32_t>(rect.top);
    args[right] = static_cast<uint32_t>(rect.right);
    args[destination_pitch] = r.destination.pitch_bytes;
    args[source_pitch] = r.source.pitch_bytes;
    args[constant_alpha] = r.constant_alpha;
    args[solid] = r.solid_colour;
    args[source_width] = r.source.width;
    args[source_height] = r.source.height;
    write_map(args, rop_args::map_x, r.destination_rect.left, r.source_left, r.source_width,
              static_cast<uint32_t>(r.destination_rect.right - r.destination_rect.left), false);
    write_map(args, rop_args::map_y, r.destination_rect.top, r.source_top, r.source_height,
              static_cast<uint32_t>(r.destination_rect.bottom - r.destination_rect.top), false);

    uint32_t flag_bits = 0;

    if (r.coverage8) {
        flag_bits |= flag_coverage8;
    } else if (r.source_alpha) {
        flag_bits |= flag_source_alpha;
    }

    args[flags] = flag_bits;
}

void raster2d_build_gradient_rect_args(const Raster2dGradientRectRequest& r,
                                       const Raster2dRect& rect, const Raster2dDescriptors& d,
                                       uint32_t (&args)[64]) noexcept {
    using namespace gradient_args;

    for (uint32_t& value : args) {
        value = 0;
    }

    copy_descriptor(args + destination, d.destination);
    args[left] = static_cast<uint32_t>(rect.left);
    args[top] = static_cast<uint32_t>(rect.top);
    args[right] = static_cast<uint32_t>(rect.right);
    args[destination_pitch] = r.destination.pitch_bytes;
    args[flags] = r.horizontal ? flag_horizontal : 0u;
    args[origin] = static_cast<uint32_t>(r.origin);

    const uint32_t dy = r.divisor == 0 ? 1u : r.divisor;
    args[divisor] = dy;
    args[bias] = dy - 1u - (dy >> 1);
    const Raster2dDivisor div = raster2d_divisor(dy);
    args[magic] = div.magic;
    args[shift1] = div.shift1;
    args[shift2] = div.shift2;

    for (uint32_t c = 0; c < 3; ++c) {
        gradient_channel_args(args, color, delta, negative, c, r.colour0[c] >> 8,
                              r.colour1[c] >> 8);
    }
}

void raster2d_build_line_args(const Raster2dLineRequest& r, const Raster2dDescriptors& d,
                              uint32_t (&args)[64], uint32_t& major_steps) noexcept {
    using namespace line_args;

    for (uint32_t& value : args) {
        value = 0;
    }

    copy_descriptor(args + destination, d.destination);
    copy_descriptor(args + style, d.source);

    const int32_t dx = r.x1 - r.x0;
    const int32_t dy = r.y1 - r.y0;
    const int32_t adx = dx < 0 ? -dx : dx;
    const int32_t ady = dy < 0 ? -dy : dy;
    const bool x_is_major = adx >= ady;
    const uint32_t major = static_cast<uint32_t>(x_is_major ? adx : ady);
    const uint32_t minor = static_cast<uint32_t>(x_is_major ? ady : adx);
    const int32_t xchange = dx < 0 ? -1 : 1;
    const int32_t ychange = dy < 0 ? -1 : 1;
    major_steps = major;

    args[left] = static_cast<uint32_t>(r.clip_bounds.left);
    args[top] = static_cast<uint32_t>(r.clip_bounds.top);
    args[right] = static_cast<uint32_t>(r.clip_bounds.right);
    args[bottom] = static_cast<uint32_t>(r.clip_bounds.bottom);
    args[destination_pitch] = r.destination.pitch_bytes;
    args[first_step] = 0;
    args[end_step] = major;

    args[x_base] = static_cast<uint32_t>(r.x0);
    args[y_base] = static_cast<uint32_t>(r.y0);
    args[x_major] = static_cast<uint32_t>(x_is_major ? xchange : 0);
    args[x_minor] = static_cast<uint32_t>(x_is_major ? 0 : xchange);
    args[y_major] = static_cast<uint32_t>(x_is_major ? 0 : ychange);
    args[y_minor] = static_cast<uint32_t>(x_is_major ? ychange : 0);

    const uint32_t d_major = major == 0 ? 1u : major;
    args[error] = d_major >> 1;
    args[minor_delta] = minor;
    const Raster2dDivisor div = raster2d_divisor(d_major);
    args[magic] = div.magic;
    args[shift1] = div.shift1;
    args[shift2] = div.shift2;
    args[color] = r.colour;

    write_rop_masks(args, rop, r.rop3);

    uint32_t flag_bits = 0;

    if (r.load_destination) {
        flag_bits |= flag_load_destination;
    }

    if (r.styled && r.style_bits != nullptr) {
        flag_bits |= flag_styled;
    } else if (r.has_pattern && r.pattern != nullptr) {
        flag_bits |= flag_pattern;
        args[brush_x] = static_cast<uint32_t>(r.brush_origin.x);
        args[brush_y] = static_cast<uint32_t>(r.brush_origin.y);
    }

    args[flags] = flag_bits;
}

// ------------------------------------------------------- triangle row planner

namespace {

void empty_span(Raster2dGradientSpan& span) noexcept {
    span = {};
    span.low = 1;
    span.high = 0;   // low > high: never covered
    span.divisor = 1;
}

bool smaller(const Raster2dTriVertex& p, const Raster2dTriVertex& q) noexcept {
    return p.y < q.y || (p.y == q.y && p.x < q.x);
}

} // namespace

void Raster2dTrianglePlanner::init_edge(Edge& e, const Raster2dTriVertex& a,
                                        const Raster2dTriVertex& b) noexcept {
    const int32_t ac[3] = {a.red >> 8, a.green >> 8, a.blue >> 8};
    const int32_t bc[3] = {b.red >> 8, b.green >> 8, b.blue >> 8};
    e.x = a.x;
    // INITLINE (shaded) starts one pixel left of the vertex; S_INITLINE does not.
    // The GDI dither origin is already folded into the device coordinates.
    e.sx = solid_ ? a.x : a.x - 1;
    e.dx = b.x > a.x ? b.x - a.x : a.x - b.x;
    e.dy = b.y > a.y ? b.y - a.y : a.y - b.y;
    e.incx = b.x > a.x ? 1 : -1;
    e.ex = -(e.dy >> 1);
    e.destx = b.x;
    e.y0 = a.y;
    e.y1 = b.y;

    for (uint32_t i = 0; i < 3; ++i) {
        e.c[i] = ac[i];
        e.dc[i] = bc[i] > ac[i] ? bc[i] - ac[i] : ac[i] - bc[i];
        e.ec[i] = -(e.dy >> 1);
        e.ic[i] = bc[i] > ac[i] ? 1 : -1;
    }
}

// DOLINE (shaded: STEPCOL then the x walk) or S_DOLINE (x walk only).
void Raster2dTrianglePlanner::step_edge(Edge& e) noexcept {
    if (!solid_) {
        for (uint32_t i = 0; i < 3; ++i) {
            e.ec[i] += e.dc[i];

            while (e.dy != 0 && e.ec[i] > 0) {
                e.c[i] += e.ic[i];
                e.ec[i] -= e.dy;
            }
        }
    }

    e.ex += e.dx;

    while (e.ex > 0 && e.x != e.destx) {
        e.x += e.incx;
        e.sx += e.incx;
        e.ex -= e.dy;
    }
}

void Raster2dTrianglePlanner::fill(const Edge& from, const Edge& to,
                                   Raster2dGradientSpan& span) const noexcept {
    span = {};

    if (solid_) {
        // S_FILLLINE: DIB_HLine over [min, max), skipped when empty.
        const int32_t lo = from.sx < to.sx ? from.sx : to.sx;
        const int32_t hi = from.sx < to.sx ? to.sx : from.sx;

        if (lo >= hi) {
            empty_span(span);
            return;
        }

        span.low = lo;
        span.high = hi - 1;
        span.divisor = 1;

        for (uint32_t c = 0; c < 3; ++c) {
            span.colour[c] = static_cast<uint16_t>(e_[0].c[c]);
        }

        return;
    }

    // FILLLINE: g from sx[from] to sx[to] inclusive, FDOCOL colour DDA.
    const int32_t gx = from.sx <= to.sx ? to.sx - from.sx : from.sx - to.sx;
    span.low = from.sx < to.sx ? from.sx : to.sx;
    span.high = from.sx < to.sx ? to.sx : from.sx;
    span.reverse = from.sx > to.sx;
    span.divisor = gx == 0 ? 1u : static_cast<uint32_t>(gx);
    span.bias = gx == 0 ? 0 : gx - 1 - (gx >> 1);

    for (uint32_t c = 0; c < 3; ++c) {
        span.colour[c] = static_cast<uint16_t>(from.c[c]);
        span.delta[c] = gx == 0 ? 0 : (to.c[c] >= from.c[c] ? to.c[c] - from.c[c] : from.c[c] - to.c[c]);
        span.negative[c] = to.c[c] < from.c[c];
    }
}

void Raster2dTrianglePlanner::start(const Raster2dTriVertex& a, const Raster2dTriVertex& b,
                                    const Raster2dTriVertex& c) noexcept {
    Raster2dTriVertex v1 = a;
    Raster2dTriVertex v2 = b;
    Raster2dTriVertex v3 = c;

    if (smaller(v2, v1)) {
        const Raster2dTriVertex t = v1;
        v1 = v2;
        v2 = t;
    }

    if (smaller(v3, v2)) {
        Raster2dTriVertex t = v2;
        v2 = v3;
        v3 = t;

        if (smaller(v2, v1)) {
            t = v1;
            v1 = v2;
            v2 = t;
        }
    }

    v_[0] = v1;
    v_[1] = v2;
    v_[2] = v3;
    // VCMPCLRS: shaded unless all three vertices share every channel.
    solid_ = v1.red == v2.red && v1.red == v3.red && v1.green == v2.green &&
             v1.green == v3.green && v1.blue == v2.blue && v1.blue == v3.blue;
    init_edge(e_[0], v1, v3);
    init_edge(e_[1], v1, v2);
    init_edge(e_[2], v2, v3);
    top_ = v1.y;
    end_ = v3.y;
    row_ = v1.y;
}

void Raster2dTrianglePlanner::next(Raster2dGradientSpan (&spans)[raster2d_triangle_spans]) noexcept {
    for (auto& span : spans) {
        empty_span(span);
    }

    const int32_t y = row_;
    step_edge(e_[0]);

    if (y >= e_[1].y0 && y <= e_[1].y1) {
        step_edge(e_[1]);
        fill(e_[0], e_[1], spans[0]);
    }

    if (y >= e_[2].y0 && y <= e_[2].y1) {
        if (solid_) {
            step_edge(e_[2]);
            fill(e_[0], e_[2], spans[1]);
        } else {
            fill(e_[0], e_[2], spans[1]);
            step_edge(e_[2]);
            fill(e_[0], e_[2], spans[2]);
        }
    }

    ++row_;
}

void Raster2dTrianglePlanner::skip_to(int32_t row) noexcept {
    Raster2dGradientSpan scratch[raster2d_triangle_spans];

    while (row_ < row && row_ < end_) {
        next(scratch);
    }
}

bool raster2d_plan_triangle(const Raster2dTriVertex& a, const Raster2dTriVertex& b,
                            const Raster2dTriVertex& c, int32_t first_row,
                            Raster2dGradientSpan (*out)[raster2d_triangle_spans], uint32_t capacity,
                            Raster2dRect& extents, uint32_t& count) noexcept {
    count = 0;
    extents = {};

    if (out == nullptr && capacity != 0) {
        return false;
    }

    Raster2dTrianglePlanner planner;
    planner.start(a, b, c);

    if (planner.end() <= planner.top()) {
        return true;   // zero-height triangle: nothing to fill
    }

    const int32_t first = first_row > planner.top() ? first_row : planner.top();
    planner.skip_to(first);
    int32_t min_x = INT32_MAX;
    int32_t max_x = INT32_MIN;

    while (planner.row() < planner.end() && count < capacity) {
        planner.next(out[count]);

        for (const Raster2dGradientSpan& span : out[count]) {
            if (span.low <= span.high) {
                min_x = span.low < min_x ? span.low : min_x;
                max_x = span.high > max_x ? span.high : max_x;
            }
        }

        ++count;
    }

    extents = {max_x >= min_x ? min_x : 0, first, max_x >= min_x ? max_x + 1 : 0,
               first + static_cast<int32_t>(count)};
    return true;
}

bool raster2d_plan_triangle(const Raster2dTriVertex& a, const Raster2dTriVertex& b,
                            const Raster2dTriVertex& c,
                            Raster2dGradientSpan (*out)[raster2d_triangle_spans], uint32_t capacity,
                            Raster2dRect& extents, uint32_t& count) noexcept {
    return raster2d_plan_triangle(a, b, c, INT32_MIN, out, capacity, extents, count);
}

// -------------------------------------------------------------------- engine

void Raster2dEngine::grid(const Raster2dRect& rect, uint32_t& groups_x,
                          uint32_t& groups_y) noexcept {
    const uint32_t width = static_cast<uint32_t>(rect.right - rect.left);
    const uint32_t height = static_cast<uint32_t>(rect.bottom - rect.top);
    groups_x = (width + raster2d_workgroup_threads - 1) / raster2d_workgroup_threads;
    groups_y = height;
}

bool Raster2dEngine::validate_surface(const Raster2dSurface& s,
                                      Raster2dFormat role) const noexcept {
    // The role fixes the format; the bgrx8888 default means "implied by the role".
    if (s.format != role && s.format != Raster2dFormat::bgrx8888) {
        return false;
    }

    if (s.width == 0 || s.height == 0 || s.width > raster2d_max_surface_dimension ||
        s.height > raster2d_max_surface_dimension || s.pitch_bytes == 0 ||
        s.gpu_address > sdma::gpu_virtual_address_max) {
        return false;
    }

    switch (role) {
    case Raster2dFormat::bgrx8888:
        if (!aligned4(s.pitch_bytes) || !aligned4(s.gpu_address) ||
            uint64_t{s.width} * raster2d_bytes_per_pixel > s.pitch_bytes) {
            return false;
        }
        break;
    case Raster2dFormat::mono1:
        if ((uint64_t{s.width} + 7u) / 8u > s.pitch_bytes) {
            return false;
        }
        break;
    case Raster2dFormat::coverage8:
        if (s.width > s.pitch_bytes) {
            return false;
        }
        break;
    case Raster2dFormat::index8:
    case Raster2dFormat::rgb16:
    case Raster2dFormat::rgb24:
        if (s.format != role || !aligned4(s.pitch_bytes) || !aligned4(s.gpu_address) ||
            uint64_t{s.width} * format_bytes_per_pixel(role) > s.pitch_bytes) {
            return false;
        }
        break;
    default:
        return false;
    }

    // NUM_RECORDS of the raw buffer descriptor is 32 bits.
    return surface_span(s) <= UINT32_MAX &&
           surface_span(s) - 1 <= sdma::gpu_virtual_address_max - s.gpu_address;
}

bool Raster2dEngine::surface_touches_engine(const Raster2dSurface& s) const noexcept {
    const uint64_t begin = s.gpu_address;
    const uint64_t end = begin + surface_span(s);
    const uint64_t regions[3][2] = {
        {code_gpu_, code_gpu_ + raster2d_code_bytes},
        {args_gpu_, args_gpu_ + args_bytes_},
        {ib_gpu_, ib_gpu_ + ib_output_.size()},
    };

    for (const auto& region : regions) {
        if (begin < region[1] && region[0] < end) {
            return true;
        }
    }

    return false;
}

void Raster2dEngine::descriptor(const Raster2dSurface& s, uint32_t (&words)[4]) const noexcept {
    raster2d_buffer_descriptor(s.gpu_address, static_cast<uint32_t>(surface_span(s)), words);
}

Raster2dStatus Raster2dEngine::usable() const noexcept {
    if (!initialized_) {
        return Raster2dStatus::not_initialized;
    }

    return report_.faulted ? Raster2dStatus::faulted : Raster2dStatus::success;
}

Raster2dStatus Raster2dEngine::fault(Raster2dStatus reason) noexcept {
    report_.faulted = true;
    report_.last_failure = reason;
    return reason;
}

Raster2dStatus Raster2dEngine::initialize(const Raster2dEngineConfig& config) noexcept {
    if (initialized_) {
        return Raster2dStatus::invalid_argument;
    }

    if (config.manager == nullptr || config.submitter.submit == nullptr ||
        config.submitter.query == nullptr || config.submitter.wait == nullptr ||
        config.ib_count < raster2d_min_ib_slots || config.ib_count > raster2d_max_ib_slots ||
        config.ib_bytes < raster2d_min_ib_bytes || (config.ib_bytes & 31u) != 0 ||
        same_handle(config.code, config.args) || same_handle(config.code, config.ib) ||
        same_handle(config.args, config.ib)) {
        return Raster2dStatus::invalid_argument;
    }

    memory::AllocationInfo code{};
    memory::AllocationInfo args{};
    memory::AllocationInfo ib{};
    const uint64_t ib_total = uint64_t{config.ib_bytes} * config.ib_count;
    const uint64_t args_total = uint64_t{raster2d_args_bytes_total} * config.ib_count;

    if (config.manager->describe(config.code, code) != memory::Status::success || code.retiring ||
        code.backing.cpu_bytes == nullptr || !permits(code.access, memory::Access::read) ||
        code.gpu.virtual_address % raster2d_code_alignment != 0 ||
        code.gpu.byte_length < raster2d_code_bytes ||
        code.backing.cpu_byte_count < raster2d_code_bytes) {
        return Raster2dStatus::invalid_argument;
    }

    if (config.manager->describe(config.args, args) != memory::Status::success || args.retiring ||
        args.backing.cpu_bytes == nullptr || !permits(args.access, memory::Access::read) ||
        args.gpu.virtual_address % raster2d_args_alignment != 0 ||
        args.gpu.byte_length < args_total || args.backing.cpu_byte_count < args_total) {
        return Raster2dStatus::invalid_argument;
    }

    if (config.manager->describe(config.ib, ib) != memory::Status::success || ib.retiring ||
        ib.backing.cpu_bytes == nullptr || !permits(ib.access, memory::Access::read) ||
        (ib.gpu.virtual_address & 31u) != 0 || ib.gpu.byte_length < ib_total ||
        ib.backing.cpu_byte_count < ib_total) {
        return Raster2dStatus::invalid_argument;
    }

    if (config.manager->retain(config.code, memory::Access::read) != memory::Status::success) {
        return Raster2dStatus::invalid_memory;
    }

    if (config.manager->retain(config.args, memory::Access::read) != memory::Status::success) {
        config.manager->unretain(config.code);
        return Raster2dStatus::invalid_memory;
    }

    if (config.manager->retain(config.ib, memory::Access::read) != memory::Status::success) {
        config.manager->unretain(config.code);
        config.manager->unretain(config.args);
        return Raster2dStatus::invalid_memory;
    }

    manager_ = config.manager;
    submitter_ = config.submitter;
    wait_timeout_us_ = config.wait_timeout_us != 0 ? config.wait_timeout_us : blit2d_default_wait_us;
    order_ibs_on_gpu_ = config.order_ibs_on_gpu;
    code_handle_ = config.code;
    args_handle_ = config.args;
    ib_handle_ = config.ib;
    code_gpu_ = code.gpu.virtual_address;
    args_gpu_ = args.gpu.virtual_address;
    args_bytes_ = args_total;
    args_cpu_ = args.backing.cpu_bytes;
    ib_gpu_ = ib.gpu.virtual_address;
    ib_cpu_ = ib.backing.cpu_bytes;
    ib_output_ = sdma::CommandBuffer(ib.backing.cpu_bytes, static_cast<size_t>(ib_total));
    ib_count_ = config.ib_count;
    next_ = 0;
    submit_order_ = 0;
    report_ = {};

    for (uint32_t i = 0; i < ib_count_; ++i) {
        ibs_[i] = {uint64_t{i} * config.ib_bytes, config.ib_bytes / 4u, 0, 0, false};
    }

    sdma::CommandBuffer code_buffer(code.backing.cpu_bytes, raster2d_code_bytes);

    if (!raster2d_build_code(code_buffer, code_image_)) {
        config.manager->unretain(config.code);
        config.manager->unretain(config.args);
        config.manager->unretain(config.ib);
        manager_ = nullptr;
        return fault(Raster2dStatus::invalid_memory);
    }

    report_.code = code_image_;
    initialized_ = true;
    return Raster2dStatus::success;
}

void Raster2dEngine::refresh_completion() noexcept {
    for (uint32_t i = 0; i < ib_count_; ++i) {
        if (!ibs_[i].in_flight) {
            continue;
        }

        const Raster2dFenceState state = submitter_.query(submitter_.context, ibs_[i].sequence);

        if (state == Raster2dFenceState::signalled) {
            ibs_[i].in_flight = false;
            --report_.in_flight;
        } else if (state == Raster2dFenceState::failed) {
            fault(Raster2dStatus::fence_failed);
        }
    }
}

Raster2dStatus Raster2dEngine::wait_slot(IbSlot& slot) noexcept {
    if (!slot.in_flight) {
        return Raster2dStatus::success;
    }

    ++report_.waits;
    const Raster2dFenceState state = submitter_.wait(submitter_.context, slot.sequence,
                                                     wait_timeout_us_);

    if (state == Raster2dFenceState::signalled) {
        slot.in_flight = false;
        --report_.in_flight;
        return Raster2dStatus::success;
    }

    return fault(state == Raster2dFenceState::pending ? Raster2dStatus::fence_timeout
                                                      : Raster2dStatus::fence_failed);
}

Raster2dStatus Raster2dEngine::acquire_slot() noexcept {
    refresh_completion();

    if (report_.faulted) {
        return Raster2dStatus::faulted;
    }

    uint32_t chosen = ib_count_;

    for (uint32_t i = 0; i < ib_count_; ++i) {
        const uint32_t index = (next_ + i) % ib_count_;

        if (!ibs_[index].in_flight) {
            chosen = index;
            break;
        }
    }

    if (chosen == ib_count_) {
        // Every slot in flight: wait for the oldest.
        uint32_t oldest = 0;

        for (uint32_t i = 1; i < ib_count_; ++i) {
            if (ibs_[i].order < ibs_[oldest].order) {
                oldest = i;
            }
        }

        const Raster2dStatus status = wait_slot(ibs_[oldest]);

        if (status != Raster2dStatus::success) {
            return status;
        }

        chosen = oldest;
    }

    next_ = chosen;
    return Raster2dStatus::success;
}

// Begin a new operation: one free IB slot, its dedicated args region cleared.
Raster2dStatus Raster2dEngine::begin_operation(uint32_t dispatch_cap) noexcept {
    const Raster2dStatus status = usable();

    if (status != Raster2dStatus::success) {
        return status;
    }

    if (dispatch_cap == 0 || dispatch_cap > raster2d_max_dispatches) {
        return Raster2dStatus::invalid_argument;
    }

    const Raster2dStatus slot = acquire_slot();

    if (slot != Raster2dStatus::success) {
        return slot;
    }

    open_ = true;
    open_dwords_ = 0;
    open_dispatches_ = 0;
    args_cursor_ = 0;
    return Raster2dStatus::success;
}

Raster2dStatus Raster2dEngine::stage_args(const uint32_t (&args)[64], const uint32_t* tail,
                                          uint32_t tail_dwords, uint64_t& args_gpu) noexcept {
    if (args_cursor_ >= raster2d_max_dispatches) {
        return Raster2dStatus::invalid_argument;
    }

    const uint64_t slot_base = uint64_t{next_} * raster2d_args_bytes_total;
    const uint64_t region = slot_base + uint64_t{args_cursor_} * raster2d_args_region_bytes;
    sdma::CommandBuffer out(args_cpu_, static_cast<size_t>(args_bytes_));

    for (uint32_t i = 0; i < 64; ++i) {
        if (!out.write_u32(static_cast<size_t>(region) + i * 4u, args[i])) {
            return fault(Raster2dStatus::invalid_memory);
        }
    }

    for (uint32_t i = 0; i < tail_dwords; ++i) {
        if (!out.write_u32(static_cast<size_t>(region) + raster2d_args_bytes + i * 4u, tail[i])) {
            return fault(Raster2dStatus::invalid_memory);
        }
    }

    args_gpu = args_gpu_ + region;
    ++args_cursor_;
    return Raster2dStatus::success;
}

Raster2dStatus Raster2dEngine::queue(Raster2dKernel kernel, const Raster2dRect& rect,
                                     uint64_t args_gpu) noexcept {
    uint32_t groups_x = 0;
    uint32_t groups_y = 0;
    grid(rect, groups_x, groups_y);

    if (groups_x == 0 || groups_y == 0) {
        return Raster2dStatus::success;   // empty rectangle contributes nothing
    }

    const Raster2dKernelInfo& info = code_image_.kernels[static_cast<uint32_t>(kernel)];
    const uint64_t pgm = code_gpu_ + info.offset_bytes;
    IbSlot& slot = ibs_[next_];
    const uint64_t slot_base = slot.offset;
    uint32_t offset = open_dwords_;

    auto emit = [&](uint32_t word) {
        if (!ib_output_.write_u32(static_cast<size_t>(slot_base) + offset * 4u, word)) {
            return false;
        }

        ++offset;
        return true;
    };

    bool ok = true;

    // IB prologue, then the full compute state once per IB, then USER_DATA +
    // DISPATCH per rectangle. The prologue orders this IB after every dispatch of
    // earlier IBs on the same compute ring without a CPU fence wait: Mesa
    // gfx10_cs_emit_cache_flush on a compute queue uses EVENT_WRITE
    // CS_PARTIAL_FLUSH (EVENT_INDEX 4: the MEC waits until earlier dispatches have
    // completed) followed by ACQUIRE_MEM with GCR_CNTL GLV_INV | GL1_INV | GLK_INV
    // (RADV_CMD_FLAG_INV_VCACHE | INV_SCACHE) so this IB's waves do not hit stale
    // vector/scalar L0/L1 lines; GL2 is shared by all CUs and needs no action.
    // Linux amdgpu inserts no such wait between kernel IBs: its per-frame
    // ACQUIRE_MEM (gfx_v11_0_emit_mem_sync) only invalidates caches and its
    // RELEASE_MEM end-of-pipe fence does not stall the next frame.
    if (open_dispatches_ == 0 && order_ibs_on_gpu_) {
        ok = emit(packet3(pm4_event_write, 0)) && emit(event_cs_partial_flush) &&
             emit(packet3(pm4_acquire_mem, 6)) && emit(0u) && emit(0xffffffffu) &&
             emit(0x00ffffffu) && emit(0u) && emit(0u) && emit(acquire_poll_interval) &&
             emit(gcr_invalidate_l0_l1);
    }

    if (open_dispatches_ == 0) {
        ok = ok && emit(packet3(pm4_set_sh_reg, 2)) && emit(off_pgm_lo) &&
             emit(static_cast<uint32_t>(pgm >> 8)) &&
             emit(static_cast<uint32_t>(pgm >> 40)) &&
             emit(packet3(pm4_set_sh_reg, 2)) && emit(off_pgm_rsrc1) && emit(info.rsrc1) &&
             emit(info.rsrc2) &&
             emit(packet3(pm4_set_sh_reg, 1)) && emit(off_pgm_rsrc3) && emit(info.rsrc3) &&
             emit(packet3(pm4_set_sh_reg, 1)) && emit(off_resource_limits) &&
             emit(0x00000000u) &&
             emit(packet3(pm4_set_sh_reg, 3)) && emit(off_num_thread_x) &&
             emit(raster2d_workgroup_threads) && emit(1u) && emit(1u);
    }

    ok = ok && emit(packet3(pm4_set_sh_reg, 2)) && emit(off_user_data_0) &&
         emit(static_cast<uint32_t>(args_gpu)) &&
         emit(static_cast<uint32_t>(args_gpu >> 32)) &&
         emit(packet3(pm4_dispatch_direct, 3) | pm4_shader_type_compute) &&
         emit(groups_x) && emit(groups_y) && emit(1u) && emit(dispatch_initiator);

    if (!ok) {
        return fault(Raster2dStatus::invalid_memory);
    }

    open_dwords_ = offset;
    ++open_dispatches_;
    ++report_.dispatches;
    return Raster2dStatus::success;
}

Raster2dStatus Raster2dEngine::submit_open() noexcept {
    if (!open_ || open_dispatches_ == 0) {
        open_ = false;
        ++report_.skipped_empty;
        return Raster2dStatus::success;
    }

    IbSlot& slot = ibs_[next_];
    const uint64_t slot_base = slot.offset;

    // Pad to a multiple of 8 dwords with one-dword NOPs (Gfx11Queues::submit IBs).
    while (open_dwords_ % 8 != 0) {
        if (!ib_output_.write_u32(static_cast<size_t>(slot_base) + open_dwords_ * 4u,
                                  pm4_nop_one_dword)) {
            return fault(Raster2dStatus::invalid_memory);
        }

        ++open_dwords_;
    }

    if (open_dwords_ > slot.capacity_dwords) {
        return fault(Raster2dStatus::invalid_memory);
    }

    uint32_t sequence = 0;
    // ib_offset is the byte offset of the slot inside the IB allocation (the
    // Blit2dSubmitter / Gfx11Submission ib_offset convention).
    const Raster2dSubmitResult result = submitter_.submit(submitter_.context, ib_handle_,
                                                          slot.offset, open_dwords_, sequence);

    if (result == Raster2dSubmitResult::busy) {
        ++report_.busy_retries;
        open_ = false;
        return Raster2dStatus::busy;
    }

    if (result != Raster2dSubmitResult::submitted) {
        open_ = false;
        return fault(Raster2dStatus::submit_failed);
    }

    slot.sequence = sequence;
    slot.order = ++submit_order_;
    slot.in_flight = true;
    ++report_.in_flight;
    ++report_.ib_submissions;
    report_.submitted_dwords += open_dwords_;
    open_ = false;
    next_ = (next_ + 1) % ib_count_;
    return Raster2dStatus::success;
}

Raster2dStatus Raster2dEngine::rop(const Raster2dRopRequest& r,
                                   const Raster2dClipList& clip) noexcept {
    if (clip.count > raster2d_max_rects) {
        return Raster2dStatus::too_many_rects;
    }

    if (r.destination_rect.right < r.destination_rect.left ||
        r.destination_rect.bottom < r.destination_rect.top) {
        return Raster2dStatus::invalid_rect;
    }

    const bool translated = r.has_source && !r.source_monochrome && raster2d_translated_format(r.source.format);
    const Raster2dFormat source_role = r.source_monochrome ? Raster2dFormat::mono1 :
                                       translated ? r.source.format : Raster2dFormat::bgrx8888;

    if ((r.source_sign_x != 1 && r.source_sign_x != -1) ||
        (r.source_sign_y != 1 && r.source_sign_y != -1) ||
        (r.has_source && (r.source_width < 0 || r.source_height < 0)) ||
        (r.stretch_mask && (!r.has_mask || !r.has_source)) ||
        (r.running_phase && (!r.stretch_mask || !r.has_pattern))) {
        return Raster2dStatus::invalid_argument;
    }

    if (!validate_surface(r.destination, Raster2dFormat::bgrx8888) ||
        (r.has_source && !validate_surface(r.source, source_role)) ||
        (r.has_mask && !validate_surface(r.mask, Raster2dFormat::mono1)) ||
        (r.has_pattern && r.pattern == nullptr)) {
        return Raster2dStatus::invalid_surface;
    }

    // One raw descriptor spans the translation table and the pixel rows.
    if (translated && (r.source_table == 0 || !aligned4(r.source_table) || r.source_table > r.source.gpu_address ||
                       translated_offset(r) < raster2d_table_bytes(r.source.format) ||
                       translated_offset(r) > UINT32_MAX - surface_span(r.source))) {
        return Raster2dStatus::invalid_surface;
    }

    if (surface_touches_engine(r.destination)) {
        return Raster2dStatus::engine_memory_overlap;
    }

    {
        const Raster2dRect area = surface_area(r.destination, r.destination_rect);
        const Blit2dRegion write = region(r.destination, area);

        if (r.has_source &&
            overlaps_clipped_writes(source_region(r.source, mapped_low(r.source_left, r.source_width,
                                                                       r.source_sign_x),
                                                  mapped_low(r.source_top, r.source_height,
                                                             r.source_sign_y),
                                                  mapped_extent(r.source_width, r.source_sign_x),
                                                  mapped_extent(r.source_height, r.source_sign_y)),
                                    r.destination, r.destination_rect, clip)) {
            return Raster2dStatus::overlapping_surfaces;
        }

        if (r.has_mask && blit2d_regions_overlap(whole_region(r.mask), write)) {
            return Raster2dStatus::overlapping_surfaces;
        }

        // BltMask reads mask (x + mask_dx, y + mask_dy) for every written pixel;
        // a mask too small for the bounds is refused (the engine refuses it too).
        // The stretch mask rule bounds-checks every sample in the kernel.
        if (r.has_mask && !r.stretch_mask && area.right > area.left && area.bottom > area.top) {
            const int64_t mx0 = int64_t{area.left} + r.mask_origin.x - r.destination_rect.left;
            const int64_t my0 = int64_t{area.top} + r.mask_origin.y - r.destination_rect.top;
            const int64_t mx1 = mx0 + (area.right - area.left);
            const int64_t my1 = my0 + (area.bottom - area.top);

            if (mx0 < 0 || my0 < 0 || mx1 > r.mask.width || my1 > r.mask.height) {
                return Raster2dStatus::out_of_bounds;
            }
        }
    }

    Raster2dDescriptors d{};
    descriptor(r.destination, d.destination);

    if (r.has_source) {
        raster2d_rop_source_descriptor(r, d.source);
    }

    if (r.has_mask) {
        raster2d_buffer_descriptor(r.mask.gpu_address, static_cast<uint32_t>(read_span(r.mask, Raster2dFormat::mono1)),
                                   d.mask);
    }

    // The 8x8 colour tile is staged in the dispatch's args tail; a raw buffer
    // descriptor over those 256 bytes feeds the kernel.
    const uint32_t clip_count = clip.count == 0 ? 1u : clip.count;
    const Raster2dStatus begin = begin_operation(clip_count);

    if (begin != Raster2dStatus::success) {
        return begin;
    }

    for (uint32_t i = 0; i < clip_count; ++i) {
        Raster2dRect rect = clip.count == 0 ? r.destination_rect : clip.rects[i];
        rect.left = rect.left > r.destination_rect.left ? rect.left : r.destination_rect.left;
        rect.top = rect.top > r.destination_rect.top ? rect.top : r.destination_rect.top;
        rect.right = rect.right < r.destination_rect.right ? rect.right : r.destination_rect.right;
        rect.bottom = rect.bottom < r.destination_rect.bottom ? rect.bottom
                                                              : r.destination_rect.bottom;

        if (rect.right <= rect.left || rect.bottom <= rect.top) {
            continue;
        }

        if (rect.left < 0 || rect.top < 0 ||
            static_cast<uint32_t>(rect.right) > r.destination.width ||
            static_cast<uint32_t>(rect.bottom) > r.destination.height) {
            open_ = false;
            return Raster2dStatus::out_of_bounds;
        }

        uint32_t args[64];
        raster2d_build_rop_args(r, rect, d, args);

        if (r.has_pattern && r.pattern != nullptr) {
            // Point the pattern V# at the tile staged after the arg block.
            uint64_t args_gpu = 0;
            const Raster2dStatus staged = stage_args(args, r.pattern, 64, args_gpu);

            if (staged != Raster2dStatus::success) {
                return staged;
            }

            uint32_t tile_words[4];
            raster2d_buffer_descriptor(args_gpu + raster2d_args_bytes, raster2d_pattern_bytes,
                                       tile_words);
            // Rewrite the pattern descriptor in the staged block.
            sdma::CommandBuffer out(args_cpu_, static_cast<size_t>(args_bytes_));
            const uint64_t region = args_gpu - args_gpu_;

            for (uint32_t w = 0; w < 4; ++w) {
                if (!out.write_u32(static_cast<size_t>(region) +
                                       (rop_args::pattern + w) * 4u, tile_words[w])) {
                    return fault(Raster2dStatus::invalid_memory);
                }
            }

            const Raster2dStatus q = queue(Raster2dKernel::rop, rect, args_gpu);

            if (q != Raster2dStatus::success) {
                return q;
            }
        } else {
            uint64_t args_gpu = 0;
            const Raster2dStatus staged = stage_args(args, nullptr, 0, args_gpu);

            if (staged != Raster2dStatus::success) {
                return staged;
            }

            const Raster2dStatus q = queue(Raster2dKernel::rop, rect, args_gpu);

            if (q != Raster2dStatus::success) {
                return q;
            }
        }
    }

    ++report_.operations;
    ++report_.rop_ops;
    return submit_open();
}

Raster2dStatus Raster2dEngine::alpha(const Raster2dAlphaRequest& r,
                                     const Raster2dClipList& clip) noexcept {
    if (clip.count > raster2d_max_rects) {
        return Raster2dStatus::too_many_rects;
    }

    if (r.destination_rect.right < r.destination_rect.left ||
        r.destination_rect.bottom < r.destination_rect.top) {
        return Raster2dStatus::invalid_rect;
    }

    if (!validate_surface(r.destination, Raster2dFormat::bgrx8888) ||
        !validate_surface(r.source, r.coverage8 ? Raster2dFormat::coverage8
                                                : Raster2dFormat::bgrx8888) ||
        r.constant_alpha > 255) {
        return Raster2dStatus::invalid_surface;
    }

    if (surface_touches_engine(r.destination)) {
        return Raster2dStatus::engine_memory_overlap;
    }

    if (blit2d_regions_overlap(r.coverage8 ? whole_region(r.source)
                                           : source_region(r.source, r.source_left, r.source_top,
                                                           r.source_width, r.source_height),
                               region(r.destination, surface_area(r.destination,
                                                                  r.destination_rect)))) {
        return Raster2dStatus::overlapping_surfaces;
    }

    Raster2dDescriptors d{};
    descriptor(r.destination, d.destination);
    raster2d_buffer_descriptor(r.source.gpu_address,
                               static_cast<uint32_t>(read_span(r.source, r.coverage8 ? Raster2dFormat::coverage8 :
                                                                                       Raster2dFormat::bgrx8888)),
                               d.source);

    const uint32_t clip_count = clip.count == 0 ? 1u : clip.count;
    const Raster2dStatus begin = begin_operation(clip_count);

    if (begin != Raster2dStatus::success) {
        return begin;
    }

    for (uint32_t i = 0; i < clip_count; ++i) {
        Raster2dRect rect = clip.count == 0 ? r.destination_rect : clip.rects[i];
        rect.left = rect.left > r.destination_rect.left ? rect.left : r.destination_rect.left;
        rect.top = rect.top > r.destination_rect.top ? rect.top : r.destination_rect.top;
        rect.right = rect.right < r.destination_rect.right ? rect.right : r.destination_rect.right;
        rect.bottom = rect.bottom < r.destination_rect.bottom ? rect.bottom
                                                              : r.destination_rect.bottom;

        if (rect.right <= rect.left || rect.bottom <= rect.top) {
            continue;
        }

        if (rect.left < 0 || rect.top < 0 ||
            static_cast<uint32_t>(rect.right) > r.destination.width ||
            static_cast<uint32_t>(rect.bottom) > r.destination.height) {
            open_ = false;
            return Raster2dStatus::out_of_bounds;
        }

        uint32_t args[64];
        raster2d_build_alpha_args(r, rect, d, args);
        uint64_t args_gpu = 0;
        const Raster2dStatus staged = stage_args(args, nullptr, 0, args_gpu);

        if (staged != Raster2dStatus::success) {
            return staged;
        }

        const Raster2dStatus q = queue(Raster2dKernel::alpha, rect, args_gpu);

        if (q != Raster2dStatus::success) {
            return q;
        }
    }

    ++report_.operations;
    ++report_.alpha_ops;
    return submit_open();
}

Raster2dStatus Raster2dEngine::gradient_rect(const Raster2dGradientRectRequest& r,
                                             const Raster2dClipList& clip) noexcept {
    if (clip.count > raster2d_max_rects) {
        return Raster2dStatus::too_many_rects;
    }

    if (r.destination_rect.right < r.destination_rect.left ||
        r.destination_rect.bottom < r.destination_rect.top ||
        !validate_surface(r.destination, Raster2dFormat::bgrx8888)) {
        return validate_surface(r.destination, Raster2dFormat::bgrx8888)
                   ? Raster2dStatus::invalid_rect : Raster2dStatus::invalid_surface;
    }

    if (surface_touches_engine(r.destination)) {
        return Raster2dStatus::engine_memory_overlap;
    }

    Raster2dDescriptors d{};
    descriptor(r.destination, d.destination);

    const uint32_t clip_count = clip.count == 0 ? 1u : clip.count;
    const Raster2dStatus begin = begin_operation(clip_count);

    if (begin != Raster2dStatus::success) {
        return begin;
    }

    for (uint32_t i = 0; i < clip_count; ++i) {
        Raster2dRect rect = clip.count == 0 ? r.destination_rect : clip.rects[i];
        rect.left = rect.left > r.destination_rect.left ? rect.left : r.destination_rect.left;
        rect.top = rect.top > r.destination_rect.top ? rect.top : r.destination_rect.top;
        rect.right = rect.right < r.destination_rect.right ? rect.right : r.destination_rect.right;
        rect.bottom = rect.bottom < r.destination_rect.bottom ? rect.bottom
                                                              : r.destination_rect.bottom;

        if (rect.right <= rect.left || rect.bottom <= rect.top) {
            continue;
        }

        if (rect.left < 0 || rect.top < 0 ||
            static_cast<uint32_t>(rect.right) > r.destination.width ||
            static_cast<uint32_t>(rect.bottom) > r.destination.height) {
            open_ = false;
            return Raster2dStatus::out_of_bounds;
        }

        uint32_t args[64];
        raster2d_build_gradient_rect_args(r, rect, d, args);
        uint64_t args_gpu = 0;
        const Raster2dStatus staged = stage_args(args, nullptr, 0, args_gpu);

        if (staged != Raster2dStatus::success) {
            return staged;
        }

        const Raster2dStatus q = queue(Raster2dKernel::gradient_rect, rect, args_gpu);

        if (q != Raster2dStatus::success) {
            return q;
        }
    }

    ++report_.operations;
    ++report_.gradient_ops;
    return submit_open();
}

namespace {

// The 48-dword kernel row record (three spans of raster2d_span_dwords).
void span_records(const Raster2dGradientSpan* spans,
                  uint32_t (&rec)[raster2d_triangle_spans * raster2d_span_dwords]) noexcept {
    for (uint32_t s = 0; s < raster2d_triangle_spans; ++s) {
        const Raster2dGradientSpan& span = spans[s];
        const Raster2dDivisor div = raster2d_divisor(span.divisor);
        uint32_t* out = rec + s * raster2d_span_dwords;

        for (uint32_t w = 0; w < raster2d_span_dwords; ++w) {
            out[w] = 0;
        }

        out[triangle_args::span_low] = static_cast<uint32_t>(span.low);
        out[triangle_args::span_high] = static_cast<uint32_t>(span.high);
        out[triangle_args::span_reverse] = span.reverse ? 0xffffffffu : 0u;
        out[triangle_args::span_bias] = static_cast<uint32_t>(span.bias);
        out[triangle_args::span_magic] = div.magic;
        out[triangle_args::span_shift1] = div.shift1;
        out[triangle_args::span_shift2] = div.shift2;

        for (uint32_t c = 0; c < 3; ++c) {
            out[triangle_args::span_color + c] = span.colour[c];
            out[triangle_args::span_delta + c] = static_cast<uint32_t>(span.delta[c]);
            out[triangle_args::span_negative + c] = span.negative[c] ? 0xffffffffu : 0u;
        }
    }
}

} // namespace

// Stages rows [first, last) (at most raster2d_triangle_table_rows) of span records
// into the slot's row table and dispatches each FillRect's part of them; one IB.
Raster2dStatus Raster2dEngine::triangle_chunk(const Raster2dSurface& destination,
                                              const Raster2dRect* fills, uint32_t fill_count,
                                              int32_t first, int32_t last,
                                              Raster2dTrianglePlanner* planner,
                                              const Raster2dGradientSpan (*rows)[raster2d_triangle_spans],
                                              int32_t rows_top) noexcept {
    const Raster2dStatus begin = begin_operation(fill_count == 0 ? 1u : fill_count);

    if (begin != Raster2dStatus::success) {
        return begin;
    }

    const uint64_t table = uint64_t{next_} * raster2d_args_bytes_total + raster2d_triangle_table_offset;
    sdma::CommandBuffer out(args_cpu_, static_cast<size_t>(args_bytes_));

    for (int32_t y = first; y < last; ++y) {
        Raster2dGradientSpan planned[raster2d_triangle_spans];
        const Raster2dGradientSpan* spans = planned;

        if (planner != nullptr) {
            planner->next(planned);
        } else {
            spans = rows[y - rows_top];
        }

        const uint64_t record = table + uint64_t(y - first) * raster2d_triangle_row_bytes;

        uint32_t rec[raster2d_triangle_spans * raster2d_span_dwords];
        span_records(spans, rec);

        for (uint32_t w = 0; w < raster2d_triangle_spans * raster2d_span_dwords; ++w) {
            if (!out.write_u32(static_cast<size_t>(record) + w * 4u, rec[w])) {
                open_ = false;
                return fault(Raster2dStatus::invalid_memory);
            }
        }
    }

    uint32_t descriptor_words[4];
    descriptor(destination, descriptor_words);

    for (uint32_t i = 0; i < fill_count; ++i) {
        const Raster2dRect& f = fills[i];
        const int32_t r0 = f.top > first ? f.top : first;
        const int32_t r1 = f.bottom < last ? f.bottom : last;

        if (f.right <= f.left || r1 <= r0) {
            continue;
        }

        uint32_t args[64] = {};
        copy_descriptor(args + triangle_args::destination, descriptor_words);
        const uint64_t base = args_gpu_ + table + uint64_t(r0 - first) * raster2d_triangle_row_bytes;
        args[triangle_args::left] = static_cast<uint32_t>(f.left);
        args[triangle_args::top] = static_cast<uint32_t>(r0);
        args[triangle_args::right] = static_cast<uint32_t>(f.right);
        args[triangle_args::destination_pitch] = destination.pitch_bytes;
        args[triangle_args::table_low] = static_cast<uint32_t>(base);
        args[triangle_args::table_high] = static_cast<uint32_t>(base >> 32);
        args[triangle_args::row_stride] = raster2d_triangle_row_bytes;
        uint64_t args_gpu = 0;
        const Raster2dStatus staged = stage_args(args, nullptr, 0, args_gpu);

        if (staged != Raster2dStatus::success) {
            open_ = false;
            return staged;
        }

        const Raster2dStatus q = queue(Raster2dKernel::gradient_triangle, {f.left, r0, f.right, r1},
                                       args_gpu);

        if (q != Raster2dStatus::success) {
            return q;
        }
    }

    return submit_open();
}

namespace {

// FillRect list: every clip rectangle (or the whole surface when the list is
// empty) & bounds & the destination surface; empty results dropped.
uint32_t fill_rects(const Raster2dSurface& s, const Raster2dRect& bounds, const Raster2dClipList& clip,
                    Raster2dRect (&fills)[raster2d_max_rects], int32_t& first, int32_t& last) noexcept {
    const Raster2dRect whole{0, 0, static_cast<int32_t>(s.width), static_cast<int32_t>(s.height)};
    const uint32_t count = clip.count == 0 ? 1u : clip.count;
    uint32_t used = 0;
    first = INT32_MAX;
    last = INT32_MIN;

    for (uint32_t i = 0; i < count; ++i) {
        const Raster2dRect c = clip.count == 0 ? whole : clip.rects[i];
        Raster2dRect f{c.left > bounds.left ? c.left : bounds.left, c.top > bounds.top ? c.top : bounds.top,
                       c.right < bounds.right ? c.right : bounds.right,
                       c.bottom < bounds.bottom ? c.bottom : bounds.bottom};
        f = surface_area(s, f);

        if (f.right <= f.left || f.bottom <= f.top) {
            continue;
        }

        fills[used++] = f;
        first = f.top < first ? f.top : first;
        last = f.bottom > last ? f.bottom : last;
    }

    return used;
}

bool clip_valid(const Raster2dClipList& clip) noexcept {
    if (clip.count > raster2d_max_rects) {
        return false;
    }

    for (uint32_t i = 0; i < clip.count; ++i) {
        if (clip.rects[i].right < clip.rects[i].left || clip.rects[i].bottom < clip.rects[i].top) {
            return false;
        }
    }

    return true;
}

} // namespace

Raster2dStatus Raster2dEngine::gradient_triangle(const Raster2dGradientTriangleRequest& r,
                                                 const Raster2dClipList& clip) noexcept {
    const Raster2dStatus state = usable();

    if (state != Raster2dStatus::success) {
        return state;
    }

    if (clip.count > raster2d_max_rects) {
        return Raster2dStatus::too_many_rects;
    }

    if (!validate_surface(r.destination, Raster2dFormat::bgrx8888)) {
        return Raster2dStatus::invalid_surface;
    }

    if (r.rows == nullptr || r.destination_rect.right < r.destination_rect.left ||
        r.destination_rect.bottom < r.destination_rect.top || !clip_valid(clip) ||
        int64_t{r.row_count} != int64_t{r.destination_rect.bottom} - r.destination_rect.top) {
        return Raster2dStatus::invalid_argument;
    }

    if (surface_touches_engine(r.destination)) {
        return Raster2dStatus::engine_memory_overlap;
    }

    // Legacy layout (unchanged contract): one dispatch per row and FillRect, the
    // row's three span records in that dispatch's args-region tail (row_stride 0),
    // so a consumer that only maps each dispatch's args region still sees them.
    // Dispatches beyond one IB continue in the next IB.
    Raster2dRect fills[raster2d_max_rects];
    int32_t first = 0;
    int32_t last = 0;
    const uint32_t count = fill_rects(r.destination, r.destination_rect, clip, fills, first, last);
    uint32_t descriptor_words[4];
    descriptor(r.destination, descriptor_words);

    for (uint32_t i = 0; i < count; ++i) {
        const Raster2dRect& f = fills[i];

        for (int32_t y = f.top; y < f.bottom; ++y) {
            if (!open_ || args_cursor_ >= raster2d_max_dispatches) {
                if (open_) {
                    const Raster2dStatus submitted = submit_open();

                    if (submitted != Raster2dStatus::success) {
                        return submitted;
                    }
                }

                const Raster2dStatus begin = begin_operation(1);

                if (begin != Raster2dStatus::success) {
                    return begin;
                }
            }

            uint32_t tail[raster2d_triangle_spans * raster2d_span_dwords];
            span_records(r.rows[y - r.destination_rect.top], tail);
            uint32_t args[64] = {};
            copy_descriptor(args + triangle_args::destination, descriptor_words);
            args[triangle_args::left] = static_cast<uint32_t>(f.left);
            args[triangle_args::top] = static_cast<uint32_t>(y);
            args[triangle_args::right] = static_cast<uint32_t>(f.right);
            args[triangle_args::destination_pitch] = r.destination.pitch_bytes;
            args[triangle_args::row_stride] = 0;
            uint64_t args_gpu = 0;
            const Raster2dStatus staged = stage_args(args, tail,
                                                     raster2d_triangle_spans * raster2d_span_dwords,
                                                     args_gpu);

            if (staged != Raster2dStatus::success) {
                open_ = false;
                return staged;
            }

            const uint64_t table = args_gpu + raster2d_args_bytes;
            sdma::CommandBuffer out(args_cpu_, static_cast<size_t>(args_bytes_));
            const size_t region = static_cast<size_t>(args_gpu - args_gpu_);

            if (!out.write_u32(region + triangle_args::table_low * 4u, static_cast<uint32_t>(table)) ||
                !out.write_u32(region + triangle_args::table_high * 4u,
                               static_cast<uint32_t>(table >> 32))) {
                open_ = false;
                return fault(Raster2dStatus::invalid_memory);
            }

            const Raster2dStatus q = queue(Raster2dKernel::gradient_triangle, {f.left, y, f.right, y + 1},
                                           args_gpu);

            if (q != Raster2dStatus::success) {
                return q;
            }
        }
    }

    if (open_) {
        const Raster2dStatus submitted = submit_open();

        if (submitted != Raster2dStatus::success) {
            return submitted;
        }
    }

    ++report_.operations;
    ++report_.gradient_ops;
    return Raster2dStatus::success;
}

Raster2dStatus Raster2dEngine::gradient_triangle(const Raster2dTriangleRequest& r,
                                                 const Raster2dClipList& clip) noexcept {
    const Raster2dStatus state = usable();

    if (state != Raster2dStatus::success) {
        return state;
    }

    if (clip.count > raster2d_max_rects) {
        return Raster2dStatus::too_many_rects;
    }

    if (!validate_surface(r.destination, Raster2dFormat::bgrx8888)) {
        return Raster2dStatus::invalid_surface;
    }

    if (r.extents.right < r.extents.left || r.extents.bottom < r.extents.top || !clip_valid(clip)) {
        return Raster2dStatus::invalid_rect;
    }

    for (const Raster2dTriVertex& v : r.vertices) {
        if (v.x < -raster2d_triangle_coordinate_limit || v.x > raster2d_triangle_coordinate_limit ||
            v.y < -raster2d_triangle_coordinate_limit || v.y > raster2d_triangle_coordinate_limit) {
            return Raster2dStatus::invalid_argument;
        }
    }

    if (surface_touches_engine(r.destination)) {
        return Raster2dStatus::engine_memory_overlap;
    }

    const Raster2dTriVertex* v = r.vertices;
    auto same = [](const Raster2dTriVertex& a, const Raster2dTriVertex& b) {
        return a.x == b.x && a.y == b.y;
    };

    // EngGradientFill skips IntEngIsNULLTriangle meshes.
    if (same(v[0], v[1]) || same(v[0], v[2]) || same(v[1], v[2])) {
        return Raster2dStatus::success;
    }

    Raster2dTrianglePlanner planner;
    planner.start(v[0], v[1], v[2]);
    // Rows run from the top vertex while sy < min(v3.y, FillRect.bottom).
    Raster2dRect bounds = r.extents;
    bounds.top = bounds.top > planner.top() ? bounds.top : planner.top();
    bounds.bottom = bounds.bottom < planner.end() ? bounds.bottom : planner.end();
    Raster2dRect fills[raster2d_max_rects];
    int32_t first = 0;
    int32_t last = 0;
    const uint32_t count = bounds.bottom > bounds.top
                               ? fill_rects(r.destination, bounds, clip, fills, first, last) : 0u;

    if (count != 0) {
        planner.skip_to(first);
    }

    for (int32_t row = first; count != 0 && row < last; row += raster2d_triangle_table_rows) {
        const int32_t end = last - row > static_cast<int32_t>(raster2d_triangle_table_rows)
                                ? row + static_cast<int32_t>(raster2d_triangle_table_rows) : last;
        const Raster2dStatus status = triangle_chunk(r.destination, fills, count, row, end, &planner,
                                                     nullptr, 0);

        if (status != Raster2dStatus::success) {
            return status;
        }
    }

    ++report_.operations;
    ++report_.gradient_ops;
    return Raster2dStatus::success;
}

Raster2dStatus Raster2dEngine::line(const Raster2dLineRequest& r) noexcept {
    if (!validate_surface(r.destination, Raster2dFormat::bgrx8888)) {
        return Raster2dStatus::invalid_surface;
    }

    if (r.clip_bounds.right < r.clip_bounds.left || r.clip_bounds.bottom < r.clip_bounds.top ||
        r.clip_bounds.left < 0 || r.clip_bounds.top < 0 ||
        static_cast<uint32_t>(r.clip_bounds.right) > r.destination.width ||
        static_cast<uint32_t>(r.clip_bounds.bottom) > r.destination.height) {
        return Raster2dStatus::invalid_rect;
    }

    if (surface_touches_engine(r.destination)) {
        return Raster2dStatus::engine_memory_overlap;
    }

    // The style bitmap and the tile both live in the dispatch's args tail.
    if (r.has_pattern && (r.pattern == nullptr || r.styled)) {
        return Raster2dStatus::invalid_argument;
    }

    Raster2dDescriptors d{};
    descriptor(r.destination, d.destination);

    uint32_t args[64];
    uint32_t major = 0;
    raster2d_build_line_args(r, d, args, major);

    if (major == 0) {
        return Raster2dStatus::success;   // a zero-length line draws nothing
    }

    const Raster2dStatus begin = begin_operation(1);

    if (begin != Raster2dStatus::success) {
        return begin;
    }

    uint64_t args_gpu = 0;

    if (r.styled && r.style_bits != nullptr) {
        // Stage the style bitmap (one bit per major step) in the tail.
        const uint32_t style_dwords = (major + 31u) / 32u;

        if (style_dwords > raster2d_pattern_bytes / 4u) {
            open_ = false;
            return Raster2dStatus::invalid_argument;
        }

        uint32_t style_words[raster2d_pattern_bytes / 4u];

        for (uint32_t& value : style_words) {
            value = 0;
        }

        for (uint32_t bit = 0; bit < major && bit < r.style_bit_count; ++bit) {
            if ((r.style_bits[bit >> 3] >> (7u - (bit & 7u))) & 1u) {
                style_words[bit >> 5] |= 1u << (bit & 31u);
            }
        }

        const Raster2dStatus staged = stage_args(args, style_words, style_dwords, args_gpu);

        if (staged != Raster2dStatus::success) {
            return staged;
        }

        uint32_t style_desc[4];
        raster2d_buffer_descriptor(args_gpu + raster2d_args_bytes, style_dwords * 4u, style_desc);
        sdma::CommandBuffer out(args_cpu_, static_cast<size_t>(args_bytes_));
        const uint64_t region = args_gpu - args_gpu_;

        for (uint32_t w = 0; w < 4; ++w) {
            out.write_u32(static_cast<size_t>(region) + (line_args::style + w) * 4u, style_desc[w]);
        }
    } else if (r.has_pattern) {
        // Stage the 8x8 tile in the tail and point the pattern V# at it.
        const Raster2dStatus staged = stage_args(args, r.pattern, 64, args_gpu);

        if (staged != Raster2dStatus::success) {
            return staged;
        }

        uint32_t tile_desc[4];
        raster2d_buffer_descriptor(args_gpu + raster2d_args_bytes, raster2d_pattern_bytes, tile_desc);
        sdma::CommandBuffer out(args_cpu_, static_cast<size_t>(args_bytes_));
        const uint64_t region = args_gpu - args_gpu_;

        for (uint32_t w = 0; w < 4; ++w) {
            if (!out.write_u32(static_cast<size_t>(region) + (line_args::pattern + w) * 4u, tile_desc[w])) {
                return fault(Raster2dStatus::invalid_memory);
            }
        }
    } else {
        const Raster2dStatus staged = stage_args(args, nullptr, 0, args_gpu);

        if (staged != Raster2dStatus::success) {
            return staged;
        }
    }

    const uint32_t groups = (major + raster2d_workgroup_threads - 1) / raster2d_workgroup_threads;
    // The line grid is one row of groups over the major-axis steps.
    Raster2dRect grid_rect{0, 0, static_cast<int32_t>(groups * raster2d_workgroup_threads), 1};
    const Raster2dStatus q = queue(Raster2dKernel::line, grid_rect, args_gpu);

    if (q != Raster2dStatus::success) {
        return q;
    }

    ++report_.operations;
    ++report_.line_ops;
    return submit_open();
}

Raster2dStatus Raster2dEngine::flush() noexcept {
    const Raster2dStatus status = usable();

    if (status != Raster2dStatus::success) {
        return status;
    }

    return open_ ? submit_open() : Raster2dStatus::success;
}

Raster2dStatus Raster2dEngine::synchronize() noexcept {
    const Raster2dStatus status = flush();

    if (status != Raster2dStatus::success && status != Raster2dStatus::busy) {
        return status;
    }

    for (uint32_t i = 0; i < ib_count_; ++i) {
        const Raster2dStatus wait = wait_slot(ibs_[i]);

        if (wait != Raster2dStatus::success) {
            return wait;
        }
    }

    return Raster2dStatus::success;
}

Raster2dStatus Raster2dEngine::retire() noexcept {
    const Raster2dStatus status = usable();

    if (status != Raster2dStatus::success) {
        return status;
    }

    refresh_completion();
    return report_.faulted ? Raster2dStatus::faulted : Raster2dStatus::success;
}

void Raster2dEngine::release_leases() noexcept {
    if (manager_ != nullptr) {
        manager_->unretain(code_handle_);
        manager_->unretain(args_handle_);
        manager_->unretain(ib_handle_);
    }

    manager_ = nullptr;
    initialized_ = false;
}

Raster2dStatus Raster2dEngine::shutdown() noexcept {
    if (!initialized_) {
        return Raster2dStatus::not_initialized;
    }

    refresh_completion();

    if (report_.faulted) {
        return Raster2dStatus::faulted;
    }

    if (open_ || report_.in_flight != 0) {
        return Raster2dStatus::busy;
    }

    release_leases();
    return Raster2dStatus::success;
}

Raster2dStatus Raster2dEngine::release_after_reset() noexcept {
    if (!initialized_) {
        return Raster2dStatus::not_initialized;
    }

    for (uint32_t i = 0; i < ib_count_; ++i) {
        ibs_[i].in_flight = false;
    }

    report_.in_flight = 0;
    open_ = false;
    release_leases();
    return Raster2dStatus::success;
}

} // namespace rx7900::hardware

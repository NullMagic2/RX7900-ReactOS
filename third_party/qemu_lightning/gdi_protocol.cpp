/*
Module: lightning.gdi.protocol
Purpose: Validate bounded GPU-canvas commands before any host GPU mutation.
Created: 2026-10-21
Architecture: Reuses RX7900 checked ByteView; no unaligned wire overlays.
License: GPL-2.0-or-later.
*/
#include "gdi_protocol.hpp"
#include "common/byte_view.hpp"

namespace lightning::gdi {
namespace {

bool valid_rectangle(LightningDisplayRect rectangle, uint32_t width, uint32_t height) noexcept {
    return width != 0 && height != 0 && width <= LIGHTNING_GDI_MAX_DIMENSION &&
        height <= LIGHTNING_GDI_MAX_DIMENSION && rectangle.width != 0 && rectangle.height != 0 &&
        rectangle.x <= width && rectangle.y <= height &&
        rectangle.width <= width - rectangle.x && rectangle.height <= height - rectangle.y;
}

bool inside(LightningDisplayRect inner, LightningDisplayRect outer) noexcept {
    return inner.x >= outer.x && inner.y >= outer.y && inner.width != 0 && inner.height != 0 &&
        inner.x - outer.x <= outer.width && inner.y - outer.y <= outer.height &&
        inner.width <= outer.width - (inner.x - outer.x) &&
        inner.height <= outer.height - (inner.y - outer.y);
}

bool overlaps(LightningDisplayRect first, LightningDisplayRect second) noexcept {
    // Only called after both extents were proven within the 4096-pixel canvas.
    return first.x < second.x + second.width && second.x < first.x + first.width &&
        first.y < second.y + second.height && second.y < first.y + first.height;
}

bool same_rectangle(LightningDisplayRect first, LightningDisplayRect second) noexcept {
    return first.x == second.x && first.y == second.y && first.width == second.width &&
        first.height == second.height;
}

bool empty_rectangle(LightningDisplayRect rectangle) noexcept {
    return same_rectangle(rectangle, {});
}

bool read_rectangle(rx7900::ByteView bytes, size_t offset, LightningDisplayRect& result) noexcept {
    return bytes.read_u32(offset + offsetof(LightningDisplayRect, x), result.x) &&
        bytes.read_u32(offset + offsetof(LightningDisplayRect, y), result.y) &&
        bytes.read_u32(offset + offsetof(LightningDisplayRect, width), result.width) &&
        bytes.read_u32(offset + offsetof(LightningDisplayRect, height), result.height);
}

bool valid_mode(Mode mode) noexcept {
    return mode.generation != 0 && mode.epoch != 0 && mode.width != 0 && mode.height != 0 &&
        mode.width <= LIGHTNING_GDI_MAX_DIMENSION && mode.height <= LIGHTNING_GDI_MAX_DIMENSION;
}

bool full_canvas(LightningDisplayRect rectangle, Mode mode) noexcept {
    return same_rectangle(rectangle, {0, 0, mode.width, mode.height});
}

bool rop3_depends(uint32_t table, uint32_t selector) noexcept {
    if (table > 255) { return false; }
    for (uint32_t combination = 0; combination < 8; ++combination) {
        if ((combination & selector) == 0 &&
            (((table >> combination) ^ (table >> (combination | selector))) & 1u) != 0) {
            return true;
        }
    }
    return false;
}

int32_t signed_word(uint32_t word) noexcept {
    return word <= INT32_MAX ? static_cast<int32_t>(word) :
        -1 - static_cast<int32_t>(UINT32_MAX - word);
}

bool read_raster(rx7900::ByteView bytes, RasterOperands& result) noexcept {
    uint32_t words[LIGHTNING_GDI_RASTER_BYTES / sizeof(uint32_t)]{};
    for (size_t index = 0; index < sizeof(words) / sizeof(words[0]); ++index) {
        if (!bytes.read_u32(LIGHTNING_GDI_COMMAND_BYTES + index * sizeof(uint32_t), words[index])) {
            return false;
        }
    }
    result = {words[0], words[1], words[2], words[3], {words[4], words[5], words[6], words[7]},
        words[8], words[9], words[10], words[11], words[12], words[13], words[14], words[15],
        words[16], words[17], words[18], words[19], signed_word(words[20]), signed_word(words[21]),
        words[22], words[23], words[24], words[25], words[26], words[27], words[28],
        words[29], words[30], words[31], {signed_word(words[32]), signed_word(words[33]), words[34], words[35]}};
    return true;
}

bool read_gradient(rx7900::ByteView bytes, GradientOperands& result) noexcept {
    uint32_t words[LIGHTNING_GDI_GRADIENT_BYTES / sizeof(uint32_t)]{};
    for (size_t index = 0; index < sizeof(words) / sizeof(words[0]); ++index) {
        if (!bytes.read_u32(LIGHTNING_GDI_COMMAND_BYTES + index * sizeof(uint32_t), words[index])) {
            return false;
        }
    }
    result.size = words[0];
    result.version = words[1];
    result.mode = words[2];
    result.reserved = words[3];
    for (size_t index = 0; index < 3; ++index) {
        const size_t offset = 4 + index * 4;
        result.vertices[index] = {signed_word(words[offset]), signed_word(words[offset + 1]),
            words[offset + 2], words[offset + 3]};
    }
    result.origin_x = signed_word(words[16]);
    result.origin_y = signed_word(words[17]);
    result.extents = {signed_word(words[18]), signed_word(words[19]), words[20], words[21]};
    return true;
}

void clear_command(Command& output) noexcept {
    // Reset the caller's exclusively owned storage in place. A temporary
    // Command would add a full 1264 bytes to the native kernel call chain.
    output.header = {};
    output.raster = {};
    output.gradient = {};
    output.external = {};
    for (auto& clip : output.clips) {
        clip = {};
    }
}

bool span(uint32_t offset, uint32_t bytes, uint32_t stride, uint32_t expected_stride,
    uint32_t rows, uint32_t& end) noexcept {
    if (offset != end || offset % sizeof(uint32_t) != 0 || stride != expected_stride ||
        bytes == 0 || stride == 0 || bytes != uint64_t{stride} * rows ||
        bytes > LIGHTNING_GDI_MAX_PIXEL_BYTES - end) {
        return false;
    }
    end += bytes;
    return true;
}

bool empty_span(uint32_t offset, uint32_t bytes, uint32_t stride) noexcept {
    return offset == 0 && bytes == 0 && stride == 0;
}

bool inside_signed(LightningDisplayRect inner, LightningGdiSignedRect outer) noexcept {
    return outer.width != 0 && outer.height != 0 && outer.width <= INT32_MAX && outer.height <= INT32_MAX &&
        static_cast<int64_t>(inner.x) >= outer.x && static_cast<int64_t>(inner.y) >= outer.y &&
        static_cast<int64_t>(inner.x) + inner.width <= static_cast<int64_t>(outer.x) + outer.width &&
        static_cast<int64_t>(inner.y) + inner.height <= static_cast<int64_t>(outer.y) + outer.height;
}

} // namespace

static_assert(sizeof(LightningGdiCommand) == LIGHTNING_GDI_COMMAND_BYTES, "Fixed command layout");
static_assert(sizeof(RasterOperands) == LIGHTNING_GDI_RASTER_BYTES, "Fixed typed raster layout");
static_assert(sizeof(GradientOperands) == LIGHTNING_GDI_GRADIENT_BYTES, "Fixed typed gradient layout");
static_assert(sizeof(ExternalImage) == LIGHTNING_GDI_EXTERNAL_IMAGE_BYTES, "Fixed external image layout");
static_assert(offsetof(ExternalImage, context) == 8 && offsetof(ExternalImage, resource) == 16 &&
    offsetof(ExternalImage, lease) == 24 && offsetof(ExternalImage, width) == 32,
    "External image tokens and layout have architecture-independent offsets");
static_assert(LIGHTNING_GDI_GRADIENT_BYTES <= LIGHTNING_GDI_RASTER_BYTES, "Gradient retains existing maximum metadata quota");
static_assert(LIGHTNING_GDI_MAX_METADATA_BYTES <= LIGHTNING_GDI_PIXEL_OFFSET,
              "Command descriptors must not overlap packed pixels");
static_assert(static_cast<uint64_t>(LIGHTNING_GDI_MAX_DIMENSION) * LIGHTNING_GDI_MAX_DIMENSION *
              LIGHTNING_GDI_PIXEL_BYTES <= LIGHTNING_GDI_TRANSFER_OFFSET,
              "Transfer window must not overlap the maximum primary");
static_assert(static_cast<uint64_t>(LIGHTNING_GDI_TRANSFER_OFFSET) + LIGHTNING_GDI_TRANSFER_BYTES <=
              LIGHTNING_MEMORY_ARENA_BYTES, "Transfer window must fit assigned memory");
static_assert(LIGHTNING_DISPLAY_END <= LIGHTNING_GDI_REG_MAGIC && LIGHTNING_GDI_END <=
              LIGHTNING_DISPLAY_DAMAGE_OFFSET, "GDI registers must not overlap display/cursor/damage");

bool validate_clip_batch(uint32_t width, uint32_t height, LightningDisplayRect destination,
                         const LightningDisplayRect* clips, uint32_t count) noexcept {
    if (!valid_rectangle(destination, width, height) || count > LIGHTNING_GDI_MAX_RECTS ||
        (count != 0 && clips == nullptr)) {
        return false;
    }

    for (uint32_t index = 0; index < count; ++index) {
        if (!valid_rectangle(clips[index], width, height) || !inside(clips[index], destination)) {
            return false;
        }

        for (uint32_t previous = 0; previous < index; ++previous) {
            if (overlaps(clips[index], clips[previous])) {
                return false;
            }
        }
    }

    return true;
}

bool packed_transfer_size(uint32_t width, uint32_t height, uint32_t& row_bytes,
                          uint32_t& total_bytes) noexcept {
    row_bytes = 0;
    total_bytes = 0;

    if (width == 0 || height == 0 || width > LIGHTNING_GDI_MAX_DIMENSION ||
        height > LIGHTNING_GDI_MAX_DIMENSION) {
        return false;
    }

    const uint64_t row = static_cast<uint64_t>(width) * LIGHTNING_GDI_PIXEL_BYTES;
    const uint64_t total = row * height;

    if (total > LIGHTNING_GDI_MAX_PIXEL_BYTES) {
        return false;
    }

    row_bytes = static_cast<uint32_t>(row);
    total_bytes = static_cast<uint32_t>(total);
    return true;
}

bool rop3_uses_source(uint32_t table) noexcept { return rop3_depends(table, 2); }
bool rop3_uses_pattern(uint32_t table) noexcept { return rop3_depends(table, 4); }
bool rop3_uses_destination(uint32_t table) noexcept { return rop3_depends(table, 1); }

bool validate_rop3_inputs(uint32_t width, uint32_t height, LightningDisplayRect destination,
    uint32_t source_x, uint32_t source_y, uint32_t source_flags, uint32_t pattern_bgra,
    uint32_t table, uint32_t pixel_bytes) noexcept {
    if (!valid_rectangle(destination, width, height) || table > 255 ||
        (!rop3_uses_pattern(table) && pattern_bgra != 0)) { return false; }
    const bool source = rop3_uses_source(table);
    const bool source_zero = source_x == 0 && source_y == 0;
    if (source_flags == LIGHTNING_GDI_ROP_SOURCE_NONE) {
        return !source && source_zero && pixel_bytes == 0;
    }
    if (!source) { return false; }
    if (source_flags == LIGHTNING_GDI_ROP_SOURCE_SCREEN) {
        return pixel_bytes == 0 && valid_rectangle({source_x, source_y,
            destination.width, destination.height}, width, height);
    }
    uint32_t row = 0, total = 0;
    return source_flags == LIGHTNING_GDI_ROP_SOURCE_UPLOAD && source_zero &&
        packed_transfer_size(destination.width, destination.height, row, total) && pixel_bytes == total;
}

bool metadata_size(uint32_t opcode, uint32_t clip_count, uint32_t& bytes) noexcept {
    bytes = 0;
    if (clip_count > LIGHTNING_GDI_MAX_RECTS) { return false; }
    const uint32_t descriptor = opcode == LIGHTNING_GDI_RASTER ? LIGHTNING_GDI_RASTER_BYTES :
        (opcode == LIGHTNING_GDI_GRADIENT ? LIGHTNING_GDI_GRADIENT_BYTES :
            (opcode == LIGHTNING_GDI_EXTERNAL_IMAGE ? LIGHTNING_GDI_EXTERNAL_IMAGE_BYTES : 0));
    bytes = LIGHTNING_GDI_COMMAND_BYTES + descriptor +
        clip_count * LIGHTNING_GDI_CLIP_BYTES;
    return true;
}

bool validate_external_image(uint32_t width, uint32_t height,
    LightningDisplayRect destination, uint32_t source_x, uint32_t source_y,
    const ExternalImage& image) noexcept {
    const uint64_t row = uint64_t{image.width} * LIGHTNING_GDI_PIXEL_BYTES;
    const uint64_t extent = uint64_t{image.stride} * image.height + image.offset;

    return valid_rectangle(destination, width, height) &&
        image.size == LIGHTNING_GDI_EXTERNAL_IMAGE_BYTES &&
        image.version == LIGHTNING_GDI_EXTERNAL_IMAGE_VERSION && image.context != 0 &&
        image.resource != 0 && image.lease != 0 && image.stride >= row &&
        image.stride % LIGHTNING_GDI_PIXEL_BYTES == 0 &&
        image.offset % LIGHTNING_GDI_PIXEL_BYTES == 0 && extent <= LIGHTNING_MEMORY_ARENA_BYTES &&
        valid_rectangle({source_x, source_y, destination.width, destination.height}, image.width, image.height);
}

bool normalize_pattern_origin(int32_t origin, uint32_t extent, uint32_t& normalized) noexcept {
    normalized = 0;
    if (extent == 0 || extent > LIGHTNING_GDI_MAX_DIMENSION) { return false; }
    const int64_t remainder = static_cast<int64_t>(origin) % extent;
    normalized = static_cast<uint32_t>(remainder < 0 ? remainder + extent : remainder);
    return true;
}

bool coloroncolor_clip_mapping(LightningGdiSignedRect original_destination,
    LightningDisplayRect source, LightningDisplayRect clip, StretchMapping& output) noexcept {
    output = {};
    if (!valid_rectangle(source, LIGHTNING_GDI_MAX_DIMENSION, LIGHTNING_GDI_MAX_DIMENSION) ||
        !valid_rectangle(clip, LIGHTNING_GDI_MAX_DIMENSION, LIGHTNING_GDI_MAX_DIMENSION) ||
        !inside_signed(clip, original_destination)) { return false; }
    const uint64_t left = static_cast<uint64_t>(static_cast<int64_t>(clip.x) - original_destination.x);
    const uint64_t top = static_cast<uint64_t>(static_cast<int64_t>(clip.y) - original_destination.y);
    const uint64_t right = left + clip.width;
    const uint64_t bottom = top + clip.height;
    const uint64_t source_left = source.x + left * source.width / original_destination.width;
    const uint64_t source_top = source.y + top * source.height / original_destination.height;
    const uint64_t source_right = source.x + right * source.width / original_destination.width;
    const uint64_t source_bottom = source.y + bottom * source.height / original_destination.height;
    if (source_left >= uint64_t{source.x} + source.width || source_top >= uint64_t{source.y} + source.height ||
        source_right > uint64_t{source.x} + source.width || source_bottom > uint64_t{source.y} + source.height ||
        source_left > source_right || source_top > source_bottom) { return false; }
    output.source = {static_cast<uint32_t>(source_left), static_cast<uint32_t>(source_top),
        static_cast<uint32_t>(source_right - source_left), static_cast<uint32_t>(source_bottom - source_top)};
    return true;
}

bool validate_gradient_inputs(uint32_t width, uint32_t height, LightningDisplayRect destination,
    const GradientOperands& operands) noexcept {
    constexpr int64_t limit = LIGHTNING_GDI_GRADIENT_COORDINATE_LIMIT;
    const auto bounded = [](int64_t coordinate) noexcept {
        return coordinate >= -static_cast<int64_t>(LIGHTNING_GDI_GRADIENT_COORDINATE_LIMIT) &&
            coordinate <= LIGHTNING_GDI_GRADIENT_COORDINATE_LIMIT;
    };
    const bool triangle = operands.mode == LIGHTNING_GDI_GRADIENT_TRIANGLE;
    const auto extents = operands.extents;
    const int64_t right = static_cast<int64_t>(extents.x) + extents.width;
    const int64_t bottom = static_cast<int64_t>(extents.y) + extents.height;

    if (!valid_rectangle(destination, width, height) || operands.size != LIGHTNING_GDI_GRADIENT_BYTES ||
        operands.version != LIGHTNING_GDI_GRADIENT_VERSION || operands.reserved != 0 ||
        operands.mode > LIGHTNING_GDI_GRADIENT_TRIANGLE ||
        !bounded(operands.origin_x) || !bounded(operands.origin_y) ||
        !bounded(extents.x) || !bounded(extents.y) || !bounded(right) || !bounded(bottom) ||
        extents.width == 0 || extents.height == 0 || extents.width > 2 * limit || extents.height > 2 * limit ||
        !inside_signed(destination, extents)) { return false; }
    const uint32_t vertices = triangle ? 3 : 2;

    for (uint32_t index = 0; index < vertices; ++index) {
        const auto vertex = operands.vertices[index];
        const int64_t x = static_cast<int64_t>(vertex.x) + operands.origin_x;
        const int64_t y = static_cast<int64_t>(vertex.y) + operands.origin_y;

        if (!bounded(vertex.x) || !bounded(vertex.y) || !bounded(x) || !bounded(y) || vertex.blue > UINT16_MAX) {
            return false;
        }
    }
    if (!triangle) {
        const auto unused = operands.vertices[2];
        if (operands.origin_x != 0 || operands.origin_y != 0 || unused.x != 0 || unused.y != 0 ||
            unused.red_green != 0 || unused.blue != 0) { return false; }
        const auto first = operands.vertices[0];
        const auto second = operands.vertices[1];
        const int32_t left = first.x < second.x ? first.x : second.x;
        const int32_t top = first.y < second.y ? first.y : second.y;
        const uint32_t rectangle_width = static_cast<uint32_t>(first.x < second.x ? second.x - first.x : first.x - second.x);
        const uint32_t rectangle_height = static_cast<uint32_t>(first.y < second.y ? second.y - first.y : first.y - second.y);

        if (!inside_signed(destination, {left, top, rectangle_width, rectangle_height})) { return false; }
    }
    return true;
}

bool raster_source_view(const RasterOperands& operands, uint32_t canvas_width,
    uint32_t canvas_height, RasterSourceView& output) noexcept {
    output = {};
    const bool legacy = operands.version == LIGHTNING_GDI_RASTER_LEGACY_VERSION;
    const bool guarded = (operands.raster_flags & LIGHTNING_GDI_RASTER_GUARDED_SOURCE) != 0;
    const bool mirror_x = (operands.raster_flags & LIGHTNING_GDI_RASTER_MIRROR_X) != 0;
    const bool mirror_y = (operands.raster_flags & LIGHTNING_GDI_RASTER_MIRROR_Y) != 0;
    const bool invalid_left = (operands.raster_flags & LIGHTNING_GDI_RASTER_INVALID_LEFT) != 0;
    const bool invalid_top = (operands.raster_flags & LIGHTNING_GDI_RASTER_INVALID_TOP) != 0;
    const auto rectangle = operands.source_rect;

    if (operands.size != LIGHTNING_GDI_RASTER_BYTES ||
        (operands.version != LIGHTNING_GDI_RASTER_VERSION && !legacy) ||
        (operands.raster_flags & ~LIGHTNING_GDI_RASTER_FLAGS) != 0 ||
        (legacy && (operands.raster_flags != 0 || operands.color_key != 0 || operands.constant_alpha != 0)) ||
        rectangle.width == 0 || rectangle.height == 0 ||
        rectangle.width > LIGHTNING_GDI_MAX_DIMENSION || rectangle.height > LIGHTNING_GDI_MAX_DIMENSION ||
        (guarded && ((!mirror_x && !mirror_y) || operands.stretch_mode != LIGHTNING_GDI_STRETCH_COLORONCOLOR ||
            (operands.raster_flags & (LIGHTNING_GDI_RASTER_ALPHA_BLEND | LIGHTNING_GDI_RASTER_COLOR_KEY)) != 0)) ||
        (invalid_left && (!guarded || !mirror_x)) || (invalid_top && (!guarded || !mirror_y))) {
        return false;
    }
    const uint32_t prefix_x = guarded && mirror_x ? 1 : 0;
    const uint32_t prefix_y = guarded && mirror_y ? 1 : 0;
    const uint32_t physical_width = rectangle.width + prefix_x;
    const uint32_t physical_height = rectangle.height + prefix_y;
    RasterSourceView candidate{};
    candidate.physical_width = physical_width;
    candidate.physical_height = physical_height;
    candidate.logical_rect = {prefix_x, prefix_y, rectangle.width, rectangle.height};
    candidate.valid_rect = {0, 0, physical_width, physical_height};

    if (operands.source_kind == LIGHTNING_GDI_ROP_SOURCE_SCREEN) {
        if (invalid_left || invalid_top || !valid_rectangle(rectangle, canvas_width, canvas_height) ||
            !empty_span(operands.source_offset, operands.source_bytes, operands.source_stride)) { return false; }
        const uint32_t available_left = rectangle.x != 0 ? prefix_x : 0;
        const uint32_t available_top = rectangle.y != 0 ? prefix_y : 0;
        candidate.screen_rect = {rectangle.x - available_left, rectangle.y - available_top,
            rectangle.width + available_left, rectangle.height + available_top};
        candidate.valid_rect = {prefix_x - available_left, prefix_y - available_top,
            candidate.screen_rect.width, candidate.screen_rect.height};
    } else if (operands.source_kind == LIGHTNING_GDI_ROP_SOURCE_UPLOAD) {
        const uint64_t row = uint64_t{physical_width} * LIGHTNING_GDI_PIXEL_BYTES;
        const uint64_t bytes = row * physical_height;

        if (rectangle.x != prefix_x || rectangle.y != prefix_y ||
            row != operands.source_stride || bytes != operands.source_bytes ||
            bytes == 0 || bytes > LIGHTNING_GDI_MAX_PIXEL_BYTES) { return false; }
        candidate.valid_rect.x = invalid_left ? prefix_x : 0;
        candidate.valid_rect.y = invalid_top ? prefix_y : 0;
        candidate.valid_rect.width -= candidate.valid_rect.x;
        candidate.valid_rect.height -= candidate.valid_rect.y;
    } else { return false; }
    output = candidate;
    return true;
}

bool raster_clip_mapping(const RasterOperands& operands, LightningDisplayRect clip,
    StretchMapping& output) noexcept {
    output = {};
    RasterSourceView view{};
    if (!raster_source_view(operands, LIGHTNING_GDI_MAX_DIMENSION, LIGHTNING_GDI_MAX_DIMENSION, view)) {
        return false;
    }
    const auto source = view.logical_rect;
    if (!coloroncolor_clip_mapping(operands.original_destination,
            {0, 0, source.width, source.height}, clip, output)) { return false; }
    output.source.x += source.x;
    output.source.y += source.y;
    if ((operands.raster_flags & LIGHTNING_GDI_RASTER_ALPHA_BLEND) != 0) {
        const auto original = operands.original_destination;
        // EngAlphaBlend derives right/bottom from signed distances to the original
        // far edge. C++ division truncates toward zero: these are not floor ratios.
        const int64_t right_distance = static_cast<int64_t>(clip.x) + clip.width -
            (static_cast<int64_t>(original.x) + original.width);
        const int64_t bottom_distance = static_cast<int64_t>(clip.y) + clip.height -
            (static_cast<int64_t>(original.y) + original.height);
        const int64_t right = static_cast<int64_t>(source.x) + source.width +
            right_distance * source.width / original.width;
        const int64_t bottom = static_cast<int64_t>(source.y) + source.height +
            bottom_distance * source.height / original.height;
        if (right < output.source.x || bottom < output.source.y ||
            right > static_cast<int64_t>(source.x) + source.width ||
            bottom > static_cast<int64_t>(source.y) + source.height) {
            output = {};
            return false;
        }
        output.source.width = static_cast<uint32_t>(right - output.source.x);
        output.source.height = static_cast<uint32_t>(bottom - output.source.y);
    }
    if (((operands.raster_flags & LIGHTNING_GDI_RASTER_MIRROR_X) != 0 &&
            output.source.width == 0 && output.source.x == 0) ||
        ((operands.raster_flags & LIGHTNING_GDI_RASTER_MIRROR_Y) != 0 &&
            output.source.height == 0 && output.source.y == 0)) {
        output = {};
        return false;
    }
    return true;
}

bool validate_raster_inputs(uint32_t width, uint32_t height, LightningDisplayRect destination,
    const RasterOperands& operands, uint32_t payload_bytes) noexcept {
    if (!valid_rectangle(destination, width, height) || operands.size != LIGHTNING_GDI_RASTER_BYTES ||
        (operands.version != LIGHTNING_GDI_RASTER_VERSION && operands.version != LIGHTNING_GDI_RASTER_LEGACY_VERSION) ||
        payload_bytes > LIGHTNING_GDI_MAX_PIXEL_BYTES ||
        operands.foreground_rop > 255 || operands.background_rop > 255 ||
        (operands.stretch_mode != LIGHTNING_GDI_STRETCH_NONE && operands.stretch_mode != LIGHTNING_GDI_STRETCH_COLORONCOLOR) ||
        (operands.raster_flags & ~LIGHTNING_GDI_RASTER_FLAGS) != 0) { return false; }
    const bool legacy = operands.version == LIGHTNING_GDI_RASTER_LEGACY_VERSION;
    const bool keyed = (operands.raster_flags & LIGHTNING_GDI_RASTER_COLOR_KEY) != 0;
    const bool alpha = (operands.raster_flags & LIGHTNING_GDI_RASTER_ALPHA_BLEND) != 0;
    const bool mirrored = (operands.raster_flags & (LIGHTNING_GDI_RASTER_MIRROR_X | LIGHTNING_GDI_RASTER_MIRROR_Y)) != 0;
    const bool source_mask = keyed && operands.mask_kind == LIGHTNING_GDI_MASK_MONO1 &&
        operands.mask_space == LIGHTNING_GDI_MASK_SPACE_SOURCE;
    if ((legacy && (operands.raster_flags != 0 || operands.color_key != 0 || operands.constant_alpha != 0)) ||
        (keyed && alpha) || (!keyed && operands.color_key != 0) || operands.color_key > 0x00ffffffu ||
        (!alpha && (operands.constant_alpha != 0 ||
            (operands.raster_flags & (LIGHTNING_GDI_RASTER_SOURCE_ALPHA | LIGHTNING_GDI_RASTER_SOURCE_32BPP)) != 0)) ||
        operands.constant_alpha > 255 || (source_mask && operands.color_key != 0) ||
        ((operands.raster_flags & LIGHTNING_GDI_RASTER_SOURCE_ALPHA) != 0 &&
            (operands.raster_flags & LIGHTNING_GDI_RASTER_SOURCE_32BPP) == 0)) { return false; }
    if ((keyed || alpha) && (operands.foreground_rop != 0xcc || operands.background_rop != 0xcc ||
        (operands.mask_kind != LIGHTNING_GDI_MASK_NONE && !source_mask))) { return false; }
    const bool source = rop3_uses_source(operands.foreground_rop) || rop3_uses_source(operands.background_rop);
    const bool pattern = rop3_uses_pattern(operands.foreground_rop) || rop3_uses_pattern(operands.background_rop);
    const bool stretch = operands.stretch_mode == LIGHTNING_GDI_STRETCH_COLORONCOLOR;
    // STRETCH_MASK: the destination plane already holds the engine's draw
    // decision, so mask-zero pixels preserve D exactly as the DIB skip does.
    const bool stretch_mask = stretch && !keyed && !alpha && operands.mask_kind == LIGHTNING_GDI_MASK_MONO1 &&
        operands.mask_space == LIGHTNING_GDI_MASK_SPACE_DESTINATION && operands.background_rop == 0xaa;
    if ((mirrored || (operands.raster_flags & (LIGHTNING_GDI_RASTER_GUARDED_SOURCE |
            LIGHTNING_GDI_RASTER_INVALID_LEFT | LIGHTNING_GDI_RASTER_INVALID_TOP)) != 0) &&
        (!source || !stretch)) { return false; }
    if (stretch) {
        if (!source || (operands.mask_kind != LIGHTNING_GDI_MASK_NONE && !source_mask && !stretch_mask) ||
            !inside_signed(destination, operands.original_destination)) { return false; }
    } else if (operands.original_destination.x != 0 || operands.original_destination.y != 0 ||
        operands.original_destination.width != 0 || operands.original_destination.height != 0) { return false; }
    uint32_t end = 0;
    if (!source) {
        if (operands.source_kind != LIGHTNING_GDI_ROP_SOURCE_NONE || !empty_rectangle(operands.source_rect) ||
            !empty_span(operands.source_offset, operands.source_bytes, operands.source_stride)) { return false; }
    } else {
        const auto rectangle = operands.source_rect;
        RasterSourceView view{};
        if ((!stretch && (rectangle.width != destination.width || rectangle.height != destination.height)) ||
            !raster_source_view(operands, width, height, view)) { return false; }
        if (operands.source_kind == LIGHTNING_GDI_ROP_SOURCE_UPLOAD &&
            !span(operands.source_offset, operands.source_bytes, operands.source_stride,
                view.physical_width * LIGHTNING_GDI_PIXEL_BYTES, view.physical_height, end)) { return false; }
    }
    if (operands.mask_kind == LIGHTNING_GDI_MASK_NONE) {
        if (operands.mask_space != LIGHTNING_GDI_MASK_SPACE_NONE || operands.mask_width != 0 ||
            operands.mask_height != 0 || !empty_span(operands.mask_offset, operands.mask_bytes, operands.mask_stride) ||
            operands.foreground_rop != operands.background_rop) { return false; }
    } else {
        if (operands.mask_kind != LIGHTNING_GDI_MASK_MONO1 && operands.mask_kind != LIGHTNING_GDI_MASK_COVERAGE8) { return false; }
        const uint32_t mask_width = source_mask ? operands.source_rect.width : destination.width;
        const uint32_t mask_height = source_mask ? operands.source_rect.height : destination.height;
        if ((!source_mask && operands.mask_space != LIGHTNING_GDI_MASK_SPACE_DESTINATION) ||
            operands.mask_width != mask_width || operands.mask_height != mask_height) { return false; }
        const uint32_t row = operands.mask_kind == LIGHTNING_GDI_MASK_MONO1 ?
            ((operands.mask_width + 31) / 32) * sizeof(uint32_t) :
            ((operands.mask_width + 3) / 4) * sizeof(uint32_t);
        if (!span(operands.mask_offset, operands.mask_bytes, operands.mask_stride, row, operands.mask_height, end)) { return false; }
        if (operands.mask_kind == LIGHTNING_GDI_MASK_COVERAGE8 &&
            (operands.source_kind != LIGHTNING_GDI_ROP_SOURCE_NONE || operands.pattern_kind != LIGHTNING_GDI_PATTERN_SOLID ||
                operands.foreground_rop != 0xf0 || operands.background_rop != 0xaa)) { return false; }
    }
    const bool empty_tile = operands.pattern_width == 0 && operands.pattern_height == 0 &&
        operands.pattern_origin_x == 0 && operands.pattern_origin_y == 0 &&
        empty_span(operands.pattern_offset, operands.pattern_bytes, operands.pattern_stride);
    if (!pattern) {
        if (operands.pattern_kind != LIGHTNING_GDI_PATTERN_NONE || operands.solid_color != 0 || !empty_tile) { return false; }
    } else if (operands.pattern_kind == LIGHTNING_GDI_PATTERN_SOLID) {
        if (!empty_tile) { return false; }
    } else if (operands.pattern_kind == LIGHTNING_GDI_PATTERN_TILE) {
        uint32_t row = 0, bytes = 0;
        if (operands.solid_color != 0 || !packed_transfer_size(operands.pattern_width, operands.pattern_height, row, bytes) ||
            !span(operands.pattern_offset, operands.pattern_bytes, operands.pattern_stride, row, operands.pattern_height, end)) { return false; }
    } else { return false; }
    if (stretch) {
        StretchMapping mapped{};
        if (!raster_clip_mapping(operands, destination, mapped)) { return false; }
    }
    return end == payload_bytes;
}

Status decode_command(const uint8_t* metadata, size_t metadata_bytes, Command& output) noexcept {
    clear_command(output);
    const rx7900::ByteView bytes(metadata, metadata_bytes);
    // Metadata is the separate immutable snapshot supplied by the serialized
    // owner. Decode into its output, never a wire overlay or retained view.
    auto& header = output.header;

    if (!bytes.contains(0, LIGHTNING_GDI_COMMAND_BYTES) ||
        !bytes.read_u32(offsetof(LightningGdiCommand, magic), header.magic) ||
        !bytes.read_u32(offsetof(LightningGdiCommand, version), header.version) ||
        !bytes.read_u32(offsetof(LightningGdiCommand, opcode), header.opcode) ||
        !bytes.read_u32(offsetof(LightningGdiCommand, flags), header.flags) ||
        !bytes.read_u32(offsetof(LightningGdiCommand, mode_generation), header.mode_generation) ||
        !bytes.read_u32(offsetof(LightningGdiCommand, epoch), header.epoch) ||
        !bytes.read_u32(offsetof(LightningGdiCommand, sequence), header.sequence) ||
        !bytes.read_u32(offsetof(LightningGdiCommand, reference), header.reference) ||
        !read_rectangle(bytes, offsetof(LightningGdiCommand, destination), header.destination) ||
        !bytes.read_u32(offsetof(LightningGdiCommand, source_x), header.source_x) ||
        !bytes.read_u32(offsetof(LightningGdiCommand, source_y), header.source_y) ||
        !bytes.read_u32(offsetof(LightningGdiCommand, color), header.color) ||
        !bytes.read_u32(offsetof(LightningGdiCommand, constant_alpha), header.constant_alpha) ||
        !bytes.read_u32(offsetof(LightningGdiCommand, clip_count), header.clip_count) ||
        !bytes.read_u32(offsetof(LightningGdiCommand, pixel_bytes), header.pixel_bytes) ||
        !read_rectangle(bytes, offsetof(LightningGdiCommand, batch), header.batch) ||
        !bytes.read_u32(offsetof(LightningGdiCommand, metadata_bytes), header.metadata_bytes) ||
        !bytes.read_u32(offsetof(LightningGdiCommand, reserved), header.reserved)) {
        clear_command(output);
        return Status::bad_packet;
    }

    uint32_t expected_metadata = 0;
    if (header.magic != LIGHTNING_GDI_MAGIC || header.version != LIGHTNING_GDI_VERSION ||
        header.reserved != 0 || header.clip_count > LIGHTNING_GDI_MAX_RECTS ||
        header.metadata_bytes != metadata_bytes || !metadata_size(header.opcode, header.clip_count, expected_metadata) ||
        metadata_bytes != expected_metadata) {
        clear_command(output);
        return Status::bad_packet;
    }
    if (header.opcode == LIGHTNING_GDI_RASTER && !read_raster(bytes, output.raster)) {
        clear_command(output);
        return Status::bad_packet;
    }
    if (header.opcode == LIGHTNING_GDI_GRADIENT && !read_gradient(bytes, output.gradient)) {
        clear_command(output);
        return Status::bad_packet;
    }
    if (header.opcode == LIGHTNING_GDI_EXTERNAL_IMAGE) {
        auto& image = output.external;
        const size_t base = LIGHTNING_GDI_COMMAND_BYTES;

        if (!bytes.read_u32(base + offsetof(ExternalImage, size), image.size) ||
            !bytes.read_u32(base + offsetof(ExternalImage, version), image.version) ||
            !bytes.read_u64(base + offsetof(ExternalImage, context), image.context) ||
            !bytes.read_u64(base + offsetof(ExternalImage, resource), image.resource) ||
            !bytes.read_u64(base + offsetof(ExternalImage, lease), image.lease) ||
            !bytes.read_u32(base + offsetof(ExternalImage, width), image.width) ||
            !bytes.read_u32(base + offsetof(ExternalImage, height), image.height) ||
            !bytes.read_u32(base + offsetof(ExternalImage, stride), image.stride) ||
            !bytes.read_u32(base + offsetof(ExternalImage, offset), image.offset)) {
            clear_command(output);
            return Status::bad_packet;
        }
    }
    const size_t clip_offset = metadata_bytes - header.clip_count * LIGHTNING_GDI_CLIP_BYTES;

    for (uint32_t index = 0; index < header.clip_count; ++index) {
        if (!read_rectangle(bytes, clip_offset + index * LIGHTNING_GDI_CLIP_BYTES,
                output.clips[index])) {
            clear_command(output);
            return Status::bad_packet;
        }
    }

    return Status::validated;
}

Status validate_command(const Command& command, Mode mode) noexcept {
    const auto& header = command.header;
    uint32_t expected_metadata = 0;

    if (!valid_mode(mode) || header.magic != LIGHTNING_GDI_MAGIC || header.version != LIGHTNING_GDI_VERSION ||
        header.reserved != 0 || header.clip_count > LIGHTNING_GDI_MAX_RECTS ||
        !metadata_size(header.opcode, header.clip_count, expected_metadata) || header.metadata_bytes != expected_metadata) {
        return Status::bad_packet;
    }

    if (header.mode_generation != mode.generation || header.epoch != mode.epoch) {
        return Status::stale_mode;
    }

    if (header.sequence == 0) {
        return Status::bad_sequence;
    }

    const bool source_zero = header.source_x == 0 && header.source_y == 0;
    const bool extra_zero = header.color == 0 && header.constant_alpha == 0;

    if (header.opcode == LIGHTNING_GDI_BIND || header.opcode == LIGHTNING_GDI_FENCE ||
        header.opcode == LIGHTNING_GDI_BEGIN_GROUP || header.opcode == LIGHTNING_GDI_END_GROUP) {
        const bool geometry = header.opcode == LIGHTNING_GDI_BIND ? full_canvas(header.destination, mode) :
            empty_rectangle(header.destination);
        const bool reference = header.opcode == LIGHTNING_GDI_BEGIN_GROUP ? header.reference == header.sequence :
            (header.opcode == LIGHTNING_GDI_END_GROUP ? header.reference != 0 : header.reference == 0);
        return geometry && header.flags == 0 && reference && source_zero && extra_zero &&
            header.clip_count == 0 && header.pixel_bytes == 0 && empty_rectangle(header.batch) ?
            Status::validated : Status::bad_packet;
    }

    if (!validate_clip_batch(mode.width, mode.height, header.destination, command.clips, header.clip_count)) {
        return Status::bad_packet;
    }

    switch (header.opcode) {
    case LIGHTNING_GDI_EXTERNAL_IMAGE:
        // REPLACEABLE is format-valid here; its optional host capability gates it.
        return (header.flags & ~LIGHTNING_GDI_EXTERNAL_REPLACEABLE) == 0 && header.reference == 0 &&
            extra_zero && header.pixel_bytes == 0 &&
            empty_rectangle(header.batch) && validate_external_image(mode.width, mode.height,
                header.destination, header.source_x, header.source_y, command.external) ?
            Status::validated : Status::bad_packet;
    case LIGHTNING_GDI_GRADIENT:
        return header.flags == 0 && header.reference == 0 && source_zero && extra_zero &&
            header.pixel_bytes == 0 && empty_rectangle(header.batch) &&
            validate_gradient_inputs(mode.width, mode.height, header.destination, command.gradient) ?
            Status::validated : Status::bad_packet;
    case LIGHTNING_GDI_RASTER:
        if (header.flags != 0 || header.reference != 0 || !source_zero || !extra_zero || !empty_rectangle(header.batch) ||
            !validate_raster_inputs(mode.width, mode.height, header.destination, command.raster, header.pixel_bytes)) {
            return Status::bad_packet;
        }
        if (command.raster.stretch_mode != LIGHTNING_GDI_STRETCH_NONE) {
            StretchMapping mapped{};
            for (uint32_t index = 0; index < header.clip_count; ++index) {
                if (!raster_clip_mapping(command.raster, command.clips[index], mapped)) { return Status::bad_packet; }
            }
        }
        return Status::validated;
    case LIGHTNING_GDI_ROP3:
        {
            return header.reference == 0 && empty_rectangle(header.batch) &&
                validate_rop3_inputs(mode.width, mode.height, header.destination, header.source_x,
                    header.source_y, header.flags, header.color, header.constant_alpha, header.pixel_bytes) ?
                Status::validated : Status::bad_packet;
        }
    case LIGHTNING_GDI_FILL:
        return header.flags == 0 && header.reference == 0 && source_zero && header.constant_alpha == 0 &&
            header.pixel_bytes == 0 && empty_rectangle(header.batch) ? Status::validated : Status::bad_packet;
    case LIGHTNING_GDI_COPY:
        return header.flags == 0 && header.reference == 0 && extra_zero && header.pixel_bytes == 0 &&
            empty_rectangle(header.batch) && valid_rectangle({header.source_x, header.source_y,
                header.destination.width, header.destination.height}, mode.width, mode.height) ?
            Status::validated : Status::bad_packet;
    case LIGHTNING_GDI_UPLOAD:
    case LIGHTNING_GDI_BLEND_UPLOAD:
    case LIGHTNING_GDI_READBACK:
        break;
    default:
        return Status::unsupported;
    }

    uint32_t row_bytes = 0;
    uint32_t total_bytes = 0;

    if (!source_zero || !packed_transfer_size(header.destination.width, header.destination.height,
            row_bytes, total_bytes) || header.pixel_bytes != total_bytes) {
        return Status::bad_packet;
    }

    if (header.opcode == LIGHTNING_GDI_UPLOAD) {
        const uint32_t allowed = LIGHTNING_GDI_UPLOAD_BEGIN | LIGHTNING_GDI_UPLOAD_END;
        const bool one_chunk = (header.flags & allowed) == allowed &&
            same_rectangle(header.destination, header.batch);
        return (header.flags & ~allowed) == 0 && extra_zero && header.reference != 0 &&
            valid_rectangle(header.batch, mode.width, mode.height) && inside(header.destination, header.batch) &&
            header.destination.x == header.batch.x && header.destination.width == header.batch.width &&
            (header.clip_count == 0 || one_chunk) ? Status::validated : Status::bad_packet;
    }

    if (header.opcode == LIGHTNING_GDI_BLEND_UPLOAD) {
        const uint32_t allowed = LIGHTNING_GDI_SOURCE_ALPHA | LIGHTNING_GDI_SOURCE_32BPP;
        return (header.flags & ~allowed) == 0 &&
            ((header.flags & LIGHTNING_GDI_SOURCE_ALPHA) == 0 ||
                (header.flags & LIGHTNING_GDI_SOURCE_32BPP) != 0) &&
            header.constant_alpha <= 255 && header.reference == 0 && header.color == 0 &&
            empty_rectangle(header.batch) ? Status::validated : Status::bad_packet;
    }

    return header.flags == 0 && extra_zero && header.clip_count == 0 && empty_rectangle(header.batch) ?
        Status::validated : Status::bad_packet;
}

bool FillBatch::configure(Mode mode) noexcept {
    *this = FillBatch{};
    if (!valid_mode(mode)) {
        return false;
    }
    mode_ = mode;
    return true;
}

bool FillBatch::append(const Command& command) noexcept {
    const auto& header = command.header;
    if (header.opcode != LIGHTNING_GDI_FILL || validate_command(command, mode_) != Status::validated ||
        (command_count_ != 0 && (header.color != color_ || last_sequence_ == UINT32_MAX ||
            header.sequence != last_sequence_ + 1))) {
        return false;
    }
    const uint32_t added = header.clip_count == 0 ? 1 : header.clip_count;
    if (added > LIGHTNING_GDI_MAX_RECTS - clip_count_) {
        return false;
    }
    for (uint32_t index = 0; index < added; ++index) {
        const auto rectangle = header.clip_count == 0 ? header.destination : command.clips[index];
        for (uint32_t old = 0; old < clip_count_; ++old) {
            if (overlaps(rectangle, clips_[old])) {
                return false;
            }
        }
    }
    // The parser proved each extent within the bounded canvas. Commit only
    // after every clip is checked, preserving the old batch on any rejection.
    for (uint32_t index = 0; index < added; ++index) {
        const auto rectangle = header.clip_count == 0 ? header.destination : command.clips[index];
        if (clip_count_ == 0) {
            destination_ = rectangle;
        } else {
            const uint32_t left = rectangle.x < destination_.x ? rectangle.x : destination_.x;
            const uint32_t top = rectangle.y < destination_.y ? rectangle.y : destination_.y;
            const uint32_t right = rectangle.x + rectangle.width > destination_.x + destination_.width ?
                rectangle.x + rectangle.width : destination_.x + destination_.width;
            const uint32_t bottom = rectangle.y + rectangle.height > destination_.y + destination_.height ?
                rectangle.y + rectangle.height : destination_.y + destination_.height;
            destination_ = {left, top, right - left, bottom - top};
        }
        clips_[clip_count_++] = rectangle;
    }
    color_ = header.color;
    last_sequence_ = header.sequence;
    ++command_count_;
    return true;
}

Status CanvasState::configure(Mode mode) noexcept {
    if (poisoned_) {
        return Status::gpu_failure;
    }

    if (count_ != 0 || upload_open_ || group_open_) {
        return Status::busy;
    }

    if (!valid_mode(mode)) {
        return Status::bad_packet;
    }

    if (mode_.generation == mode.generation && mode_.epoch == mode.epoch) {
        return Status::stale_mode;
    }

    *this = CanvasState{};
    mode_ = mode;
    return Status::validated;
}

uint32_t CanvasState::next_sequence() const noexcept {
    return accepted_ == UINT32_MAX ? 0 : accepted_ + 1;
}

Status CanvasState::validate(const Command& command) const noexcept {
    if (poisoned_) {
        return Status::gpu_failure;
    }

    const auto status = validate_command(command, mode_);

    if (status != Status::validated) {
        return status;
    }

    const auto& header = command.header;

    if (header.sequence != next_sequence()) {
        return Status::bad_sequence;
    }

    if (count_ == LIGHTNING_GDI_MAX_PENDING_FENCES) {
        return Status::busy;
    }

    if (accepted_ == 0 || header.opcode == LIGHTNING_GDI_BIND) {
        return accepted_ == 0 && header.opcode == LIGHTNING_GDI_BIND ? Status::validated : Status::bad_sequence;
    }

    if (upload_open_ && header.opcode != LIGHTNING_GDI_UPLOAD) {
        return Status::batch_incomplete;
    }
    if (header.opcode == LIGHTNING_GDI_BEGIN_GROUP) {
        if (group_open_) { return Status::batch_incomplete; }
        return baseline_accepted_ ? Status::validated : Status::not_ready;
    }
    if (header.opcode == LIGHTNING_GDI_END_GROUP) {
        return group_open_ && header.reference == group_reference_ ? Status::validated : Status::batch_incomplete;
    }

    if (header.opcode == LIGHTNING_GDI_UPLOAD) {
        const bool first = (header.flags & LIGHTNING_GDI_UPLOAD_BEGIN) != 0;
        const bool last = (header.flags & LIGHTNING_GDI_UPLOAD_END) != 0;

        if (first) {
            if (upload_open_ || header.reference != header.sequence || header.destination.y != header.batch.y) {
                return Status::batch_incomplete;
            }

            if (!baseline_accepted_ && (!full_canvas(header.batch, mode_) || header.clip_count != 0)) {
                return Status::not_ready;
            }
        } else if (!upload_open_ || header.reference != upload_reference_ ||
                   !same_rectangle(header.batch, upload_batch_) || header.destination.y != upload_next_y_) {
            return Status::batch_incomplete;
        }

        const uint32_t bottom = header.destination.y + header.destination.height;
        const uint32_t batch_bottom = header.batch.y + header.batch.height;
        return (last ? bottom == batch_bottom : bottom < batch_bottom) ? Status::validated : Status::batch_incomplete;
    }

    if (!baseline_accepted_) {
        return Status::not_ready;
    }

    if (header.opcode == LIGHTNING_GDI_READBACK && header.reference != revision_) {
        return Status::bad_sequence;
    }

    return Status::validated;
}

Status CanvasState::accepted(const Command& command, uint64_t owner_ticket) noexcept {
    const auto status = validate(command);

    if (status != Status::validated) {
        return status;
    }

    if (owner_ticket == 0 || owner_ticket <= last_ticket_) {
        return Status::bad_sequence;
    }

    const auto& header = command.header;
    pending_[(head_ + count_) % LIGHTNING_GDI_MAX_PENDING_FENCES] = {header.sequence, owner_ticket};
    ++count_;
    accepted_ = header.sequence;
    last_ticket_ = owner_ticket;

    if (header.opcode == LIGHTNING_GDI_BEGIN_GROUP) {
        group_open_ = true;
        group_reference_ = header.sequence;
    } else if (header.opcode == LIGHTNING_GDI_END_GROUP) {
        group_open_ = false;
        group_reference_ = 0;
        group_end_sequence_ = header.sequence;
    }

    if (header.opcode == LIGHTNING_GDI_UPLOAD) {
        if ((header.flags & LIGHTNING_GDI_UPLOAD_BEGIN) != 0) {
            upload_batch_ = header.batch;
            upload_reference_ = header.reference;
        }

        upload_next_y_ = header.destination.y + header.destination.height;
        upload_open_ = (header.flags & LIGHTNING_GDI_UPLOAD_END) == 0;

        if (!upload_open_ && full_canvas(header.batch, mode_)) {
            baseline_accepted_ = true;
            baseline_sequence_ = header.sequence;
        }
    }

    if (header.opcode == LIGHTNING_GDI_FILL || header.opcode == LIGHTNING_GDI_COPY ||
        header.opcode == LIGHTNING_GDI_UPLOAD || header.opcode == LIGHTNING_GDI_BLEND_UPLOAD ||
        header.opcode == LIGHTNING_GDI_ROP3 || header.opcode == LIGHTNING_GDI_RASTER ||
        header.opcode == LIGHTNING_GDI_GRADIENT || header.opcode == LIGHTNING_GDI_EXTERNAL_IMAGE) {
        revision_ = header.sequence;
    }

    return Status::validated;
}

Status CanvasState::completed(uint32_t sequence, uint64_t owner_ticket) noexcept {
    if (count_ == 0 || pending_[head_].sequence != sequence || pending_[head_].ticket != owner_ticket) {
        return Status::bad_sequence;
    }

    pending_[head_] = {};
    head_ = (head_ + 1) % LIGHTNING_GDI_MAX_PENDING_FENCES;
    --count_;
    completed_ = sequence;
    return Status::validated;
}

bool CanvasState::readback_ready() const noexcept {
    return !poisoned_ && baseline_accepted_ && !upload_open_ && completed_ >= baseline_sequence_ &&
        completed_ >= revision_;
}

bool CanvasState::publication_ready() const noexcept {
    return readback_ready() && !group_open_ && completed_ >= group_end_sequence_;
}

void CanvasState::retire_after_backend_release() noexcept {
    *this = CanvasState{};
}

bool TransferLease::matches(Mode mode, uint32_t sequence) const noexcept {
    return mode.generation == mode_.generation && mode.epoch == mode_.epoch && mode.width == mode_.width &&
        mode.height == mode_.height && sequence == sequence_;
}

bool TransferLease::reserve(Mode mode, uint32_t sequence, bool readback) noexcept {
    if (phase_ != LeasePhase::free || !valid_mode(mode) || sequence == 0) {
        return false;
    }

    mode_ = mode;
    sequence_ = sequence;
    readback_ = readback;
    phase_ = LeasePhase::awaiting_acceptance;
    return true;
}

bool TransferLease::accepted(Mode mode, uint32_t sequence) noexcept {
    if (phase_ != LeasePhase::awaiting_acceptance || !matches(mode, sequence)) {
        return false;
    }

    if (readback_) {
        phase_ = LeasePhase::awaiting_completion;
    } else {
        *this = TransferLease{};
    }

    return true;
}

bool TransferLease::rejected(Mode mode, uint32_t sequence) noexcept {
    if (phase_ != LeasePhase::awaiting_acceptance || !matches(mode, sequence)) {
        return false;
    }

    *this = TransferLease{};
    return true;
}

bool TransferLease::completed(Mode mode, uint32_t sequence) noexcept {
    if (phase_ != LeasePhase::awaiting_completion || !matches(mode, sequence)) {
        return false;
    }

    phase_ = LeasePhase::readback_ready;
    return true;
}

bool TransferLease::copied(Mode mode, uint32_t sequence) noexcept {
    if (phase_ != LeasePhase::readback_ready || !matches(mode, sequence)) {
        return false;
    }

    *this = TransferLease{};
    return true;
}

void TransferLease::retire_after_backend_release() noexcept {
    *this = TransferLease{};
}

bool StripeCursor::begin(LightningDisplayRect rectangle, uint32_t width, uint32_t height) noexcept {
    *this = StripeCursor{};

    if (!valid_rectangle(rectangle, width, height)) {
        return false;
    }

    rectangle_ = rectangle;
    next_y_ = rectangle.y;
    rows_ = LIGHTNING_GDI_MAX_PIXEL_BYTES / (rectangle.width * LIGHTNING_GDI_PIXEL_BYTES);
    valid_ = rows_ != 0;
    return valid_;
}

bool StripeCursor::current(LightningDisplayRect& rectangle) const noexcept {
    rectangle = {};

    if (!valid_ || done()) {
        return false;
    }

    const uint32_t remaining = rectangle_.y + rectangle_.height - next_y_;
    rectangle = {rectangle_.x, next_y_, rectangle_.width, remaining < rows_ ? remaining : rows_};
    return true;
}

bool StripeCursor::advance(LightningDisplayRect confirmed) noexcept {
    LightningDisplayRect expected{};

    if (!current(expected) || !same_rectangle(expected, confirmed)) {
        return false;
    }

    next_y_ += expected.height;
    return true;
}

} // namespace lightning::gdi

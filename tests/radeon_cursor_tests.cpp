/*
Module: rx7900.tests.radeon_cursor
Purpose: Verify the VideoPort pointer IOCTL wire contract and its DCN image conversion.
Created: 2026-10-22
Architecture: Byte-exact VIDEO_POINTER_ATTRIBUTES/POSITION buffers (ntddvdeo.h
layout) are decoded by radeon_cursor_policy.hpp and converted by
src/display/dcn32_cursor exactly as radeon_cursor_native maps them (one-to-one
format values). Includes truncation, flag and size refusals the miniport must
reject before any state change. Host only; not hardware evidence.
*/
#include <cstring>
#include <memory>
#include <vector>

#include "asic_test_device.hpp"
#include "dcn32_cursor.hpp"
#include "radeon_cursor_policy.hpp"

namespace {

namespace policy = rx7900::reactos::cursor_policy;
using rx7900::ByteView;
using namespace rx7900::display;

rx7900_test::Checks checks;

void expect(bool condition, const char* contract) { checks.expect(condition, contract); }

std::vector<uint8_t> attributes(uint32_t flags, uint32_t width, uint32_t height, uint32_t width_in_bytes,
                                uint32_t enable, int16_t column, int16_t row, const std::vector<uint8_t>& pixels) {
    std::vector<uint8_t> bytes(policy::attributes_header_bytes + pixels.size());
    const rx7900::sdma::CommandBuffer output(bytes.data(), bytes.size());
    output.write_u32(0, flags);
    output.write_u32(4, width);
    output.write_u32(8, height);
    output.write_u32(12, width_in_bytes);
    output.write_u32(16, enable);
    output.write_u32(20, static_cast<uint16_t>(column) | (static_cast<uint32_t>(static_cast<uint16_t>(row)) << 16));
    std::copy(pixels.begin(), pixels.end(), bytes.begin() + policy::attributes_header_bytes);
    return bytes;
}

CursorSource source_of(const policy::PointerShape& shape) {
    CursorSource source{};
    source.format = static_cast<CursorSourceFormat>(shape.format);
    source.width = shape.width;
    source.height = shape.height;
    source.mask = shape.mask;
    source.mask_stride = shape.mask_stride;
    source.color = shape.color;
    source.color_stride = shape.color_stride;
    return source;
}

void decoding() {
    // 16x16 monochrome, WidthInBytes 2 (AND rows then XOR rows).
    std::vector<uint8_t> mono(2 * 16 * 2, 0xff);
    std::fill(mono.begin() + 32, mono.end(), 0x00);
    auto bytes = attributes(policy::flag_mono, 16, 16, 2, 1, -3, 700, mono);
    policy::PointerShape shape{};
    expect(policy::decode_attributes({bytes.data(), bytes.size()}, shape) == policy::DecodeStatus::decoded &&
               shape.format == policy::PointerFormat::monochrome && shape.width == 16 && shape.height == 16 &&
               shape.enable && shape.column == -3 && shape.row == 700 && shape.mask.size() == 64 &&
               shape.mask_stride == 2,
           "monochrome attributes decode with signed Column/Row");

    auto image = std::make_unique<CursorImage>();
    expect(build_cursor_image(source_of(shape), *image) == CursorImageStatus::built &&
               image->mode == CursorMode::premultiplied_alpha && image->pixels[0] == 0,
           "an all-transparent monochrome pointer encodes as transparent ARGB");

    // 32x32 masked color: mask stride 4, color 128 bytes per row.
    std::vector<uint8_t> masked(4 * 32 + 128 * 32, 0);
    masked[4 * 32 + 0] = 0x11;
    masked[4 * 32 + 1] = 0x22;
    masked[4 * 32 + 2] = 0x33;
    bytes = attributes(policy::flag_color, 32, 32, 128, 0, 0, 0, masked);
    expect(policy::decode_attributes({bytes.data(), bytes.size()}, shape) == policy::DecodeStatus::decoded &&
               shape.format == policy::PointerFormat::masked_color && !shape.enable && shape.mask_stride == 4 &&
               shape.mask.size() == 128 && shape.color_stride == 128 && shape.color.size() == 128u * 32u,
           "masked color attributes split into a DWORD-aligned AND mask and 32 bpp rows");
    expect(build_cursor_image(source_of(shape), *image) == CursorImageStatus::built &&
               image->pixels[0] == 0xff332211u,
           "AND 0 makes the color opaque");

    std::vector<uint8_t> argb(64 * 4 * 64, 0x80);
    bytes = attributes(policy::flag_color | policy::flag_premultiplied_alpha, 64, 64, 256, 1, 10, 10, argb);
    expect(policy::decode_attributes({bytes.data(), bytes.size()}, shape) == policy::DecodeStatus::decoded &&
               shape.format == policy::PointerFormat::premultiplied_alpha && shape.color.size() == argb.size() &&
               shape.mask.size() == 0,
           "a 64 x 64 premultiplied pointer decodes without a mask");
    expect(build_cursor_image(source_of(shape), *image) == CursorImageStatus::built &&
               image->mode == CursorMode::premultiplied_alpha && image->pixels[64 * 63 + 63] == 0x80808080u,
           "and reaches the DCN image unchanged");
    bytes = attributes(policy::flag_color | policy::flag_straight_alpha, 64, 64, 256, 1, 10, 10, argb);
    expect(policy::decode_attributes({bytes.data(), bytes.size()}, shape) == policy::DecodeStatus::decoded &&
               shape.format == policy::PointerFormat::straight_alpha,
           "straight alpha decodes");

    // The largest legal request is exactly max_attributes_bytes.
    std::vector<uint8_t> largest(policy::mask_stride(64) * 64 + 256 * 64, 0);
    bytes = attributes(policy::flag_color, 64, 64, 256, 1, 0, 0, largest);
    expect(bytes.size() == policy::max_attributes_bytes &&
               policy::decode_attributes({bytes.data(), bytes.size()}, shape) == policy::DecodeStatus::decoded,
           "a 64 x 64 masked color pointer is the maximum request");
    bytes.push_back(0);
    expect(policy::decode_attributes({bytes.data(), bytes.size()}, shape) == policy::DecodeStatus::oversized,
           "anything longer than the maximum request is refused");
}

void refusals() {
    policy::PointerShape shape{};
    shape.width = 77;
    const std::vector<uint8_t> mono(64, 0);
    auto check = [&](const std::vector<uint8_t>& bytes, policy::DecodeStatus status, const char* contract) {
        expect(policy::decode_attributes({bytes.data(), bytes.size()}, shape) == status && shape.width == 77,
               contract);
    };

    check(attributes(policy::flag_mono | policy::flag_color, 16, 16, 2, 1, 0, 0, mono),
          policy::DecodeStatus::invalid_flags, "mono and color together are refused");
    check(attributes(0, 16, 16, 2, 1, 0, 0, mono), policy::DecodeStatus::invalid_flags, "no format is refused");
    check(attributes(policy::flag_mono | policy::flag_premultiplied_alpha, 16, 16, 2, 1, 0, 0, mono),
          policy::DecodeStatus::invalid_flags, "alpha with a mono pointer is refused");
    check(attributes(policy::flag_color | policy::flag_premultiplied_alpha | policy::flag_straight_alpha, 4, 4, 16, 1,
                     0, 0, mono),
          policy::DecodeStatus::invalid_flags, "both alpha kinds are refused");
    check(attributes(policy::flag_mono | 0x100, 16, 16, 2, 1, 0, 0, mono), policy::DecodeStatus::invalid_flags,
          "unknown flag bits are refused");
    check(attributes(policy::flag_mono | policy::flag_async, 16, 16, 2, 1, 0, 0, mono),
          policy::DecodeStatus::invalid_flags, "asynchronous pointers are not advertised and are refused");
    check(attributes(policy::flag_mono, 16, 16, 2, 2, 0, 0, mono), policy::DecodeStatus::invalid_flags,
          "Enable other than 0/1 is refused");
    check(attributes(policy::flag_mono, 65, 16, 9, 1, 0, 0, mono), policy::DecodeStatus::invalid_size,
          "a width above 64 is refused");
    check(attributes(policy::flag_mono, 16, 0, 2, 1, 0, 0, mono), policy::DecodeStatus::invalid_size,
          "zero height is refused");
    check(attributes(policy::flag_mono, 16, 16, 1, 1, 0, 0, mono), policy::DecodeStatus::invalid_size,
          "a mono stride below ceil(width / 8) is refused");
    check(attributes(policy::flag_color, 16, 16, 60, 1, 0, 0, mono), policy::DecodeStatus::invalid_size,
          "a color stride other than width * 4 is refused");
    check(attributes(policy::flag_mono, 16, 16, 2, 1, 0, 0, std::vector<uint8_t>(63, 0)),
          policy::DecodeStatus::truncated, "a payload one byte short is refused");
    std::vector<uint8_t> header(policy::attributes_header_bytes - 1, 0);
    check(header, policy::DecodeStatus::truncated, "a truncated header is refused");
    check({}, policy::DecodeStatus::truncated, "an empty buffer is refused");
}

void positions() {
    const uint8_t raw[4]{0xfe, 0xff, 0x10, 0x00};
    int16_t column = 0;
    int16_t row = 0;
    expect(policy::decode_position({raw, 4}, column, row) && column == -2 && row == 16,
           "VIDEO_POINTER_POSITION decodes signed SHORTs");
    expect(!policy::decode_position({raw, 3}, column, row) && !policy::decode_position({raw, 0}, column, row),
           "a position buffer of another length is refused");

    expect(policy::top_left(100, 50, 10, 4, column, row) && column == 90 && row == 46,
           "GDI hotspot coordinates become the image top-left");
    expect(policy::top_left(0, 0, 31, 31, column, row) && column == -31 && row == -31,
           "the top-left may be negative (partly offscreen)");
    expect(!policy::top_left(40000, 0, 0, 0, column, row) && !policy::top_left(0, 0, -1, 0, column, row),
           "positions beyond 16 bits or negative hotspots are refused");

    expect(policy::payload_bytes(policy::PointerFormat::monochrome, 32, 32, 4) == 256 &&
               policy::payload_bytes(policy::PointerFormat::masked_color, 32, 32, 128) == 4 * 32 + 128 * 32 &&
               policy::payload_bytes(policy::PointerFormat::premultiplied_alpha, 48, 48, 192) == 192 * 48 &&
               policy::mask_stride(1) == 4 && policy::mask_stride(33) == 8,
           "payload sizes follow the documented layout");
}

} // namespace

int main() {
    decoding();
    refusals();
    positions();
    return checks.finish("radeon_cursor");
}

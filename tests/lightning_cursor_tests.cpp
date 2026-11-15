/*
Module: lightning.tests.cursor
Purpose: Exercise the exact shared native cursor geometry and position policy.
Created: 2026-10-21
Architecture: CPU-only bounds checks; no claim of host cursor publication.
*/
#include <cstdio>
#include <array>
#include <initializer_list>
#include <limits>
#include "lightning_cursor_policy.hpp"
#include "cursor_policy.hpp"

namespace {
using namespace lightning::reactos::cursor;
unsigned checks = 0;
unsigned failures = 0;

void expect(bool condition, const char* description) {
    ++checks;
    if (!condition) {
        ++failures;
        std::fprintf(stderr, "FAIL: %s\n", description);
    }
}

void check_shape_layouts() {
    for (uint32_t width : {1U, 7U, 8U, 9U, 31U, 32U, 33U, 47U, LIGHTNING_SCANOUT_CURSOR_MAX_DIMENSION}) {
        for (uint32_t height : {1U, 2U, 47U, LIGHTNING_SCANOUT_CURSOR_MAX_DIMENSION}) {
            for (uint32_t format : {LIGHTNING_SCANOUT_CURSOR_MONOCHROME, LIGHTNING_SCANOUT_CURSOR_COLOR,
                                   LIGHTNING_SCANOUT_CURSOR_MASKED_COLOR}) {
                const bool mono = format == LIGHTNING_SCANOUT_CURSOR_MONOCHROME;
                const uint32_t pitch = mono ? ((width + 31) / 32) * 4 : width * 4;
                const uint32_t bytes = pitch * height * (mono ? LIGHTNING_SCANOUT_CURSOR_MONOCHROME_PLANES : 1);
                ShapeLayout shape{width, height, pitch, format, width - 1, height - 1, bytes};
                expect(valid_shape(shape), "supported word-aligned shape accepts its exact owned byte extent");
                auto bad = shape;
                bad.hot_x = width;
                expect(!valid_shape(bad), "hotspot immediately outside width is rejected");
                bad = shape;
                bad.hot_y = height;
                expect(!valid_shape(bad), "hotspot immediately outside height is rejected");
                bad = shape;
                --bad.payload_bytes;
                expect(!valid_shape(bad), "one missing payload byte rejects before any native word read");
                bad = shape;
                ++bad.payload_bytes;
                expect(!valid_shape(bad), "trailing unowned byte is not silently accepted");
                bad = shape;
                ++bad.pitch;
                bad.payload_bytes = bad.pitch * height * (mono ? 2 : 1);
                expect(!valid_shape(bad), "unaligned row cannot permit a native four-byte tail read");
                bad = shape;
                bad.pitch = 0;
                bad.payload_bytes = 0;
                expect(!valid_shape(bad), "empty storage cannot represent a nonempty cursor");
            }
        }
    }

    const ShapeLayout mono{8, 8, 4, LIGHTNING_SCANOUT_CURSOR_MONOCHROME, 0, 0, 64};
    auto bad = mono;
    bad.payload_bytes /= 2;
    expect(!valid_shape(bad), "monochrome shape requires both AND and XOR planes");
    for (uint32_t format : {0U, 4U, UINT32_MAX}) {
        bad = mono;
        bad.format = format;
        expect(!valid_shape(bad), "unknown pixel format cannot reach native shape registers");
    }
    for (uint32_t dimension : {0U, LIGHTNING_SCANOUT_CURSOR_MAX_DIMENSION + 1, UINT32_MAX}) {
        bad = mono;
        bad.width = dimension;
        expect(!valid_shape(bad), "zero or excessive width fails before pitch arithmetic");
        bad = mono;
        bad.height = dimension;
        expect(!valid_shape(bad), "zero or excessive height fails before plane arithmetic");
    }
    expect(valid_shape({48, 48, 256, LIGHTNING_SCANOUT_CURSOR_COLOR, 47, 47,
        LIGHTNING_SCANOUT_CURSOR_SHAPE_BYTES}), "legal padded rows may exactly fill the assigned shape window");
    bad = mono;
    bad.payload_bytes = LIGHTNING_SCANOUT_CURSOR_SHAPE_BYTES + 4;
    expect(!valid_shape(bad), "shape bytes cannot escape fixed cursor window");
    bad = mono;
    bad.pitch = UINT32_MAX - 3;
    bad.payload_bytes = 64;
    expect(!valid_shape(bad), "large aligned pitch cannot wrap product into a short buffer");
    expect(!valid_shape({1, 1, 0x80000000U, LIGHTNING_SCANOUT_CURSOR_MONOCHROME, 0, 0, 0}),
        "two-plane size calculation cannot accept a wrapped zero extent");
    expect(!valid_shape({8, 8, 4, LIGHTNING_SCANOUT_CURSOR_COLOR, 0, 0, 32}),
        "color row must contain four bytes per pixel");
}

bool cleared(Position position) {
    return !position.visible && position.left == 0 && position.top == 0 && position.packed == 0;
}

void check_positions() {
    Position position{};
    expect(from_top_left(INT16_MIN, INT16_MAX, true, position) && position.visible && position.packed == 0x7fff8000U,
        "native signed16 extrema pack without changing sign or coordinate order");
    expect(from_top_left(INT16_MAX, INT16_MIN, true, position) && position.packed == 0x80007fffU,
        "opposite signed16 extrema pack without overflow");
    expect(from_hotspot(0, 0, 1, 2, position) && position.left == -1 && position.top == -2 && position.visible &&
        position.packed == 0xfffeffffU, "negative top-left clipping preserves hotspot instead of hiding by result coordinate");
    expect(from_hotspot(-2, -3, 0, 0, position) && position.left == -2 && position.top == -3 && position.visible,
        "offscreen input other than the GDI sentinel remains an ordinary visible-plane position");

    for (int32_t hot = 0; hot < static_cast<int32_t>(LIGHTNING_SCANOUT_CURSOR_MAX_DIMENSION); ++hot) {
        expect(from_hotspot(0, 0, hot, hot, position) && position.left == -hot && position.top == -hot && position.visible,
            "every valid hotspot permits a clipped cursor along top and left screen edges");
        expect(from_hotspot(INT16_MIN + hot, INT16_MIN + hot, hot, hot, position) &&
            position.left == INT16_MIN && position.top == INT16_MIN,
            "hotspot subtraction retains the exact minimum wire position");
        expect(from_hotspot(INT16_MAX + hot, INT16_MAX + hot, hot, hot, position) &&
            position.left == INT16_MAX && position.top == INT16_MAX,
            "GDI hotspot may exceed wire coordinate when resulting top-left remains representable");
        position = {12, 13, 0x000d000cU, true};
        expect(!from_hotspot(INT16_MIN + hot - 1, 0, hot, hot, position) && cleared(position),
            "one position below representable left bound fails and clears stale accepted output");
        position = {12, 13, 0x000d000cU, true};
        expect(!from_hotspot(0, INT16_MAX + hot + 1, hot, hot, position) && cleared(position),
            "one position above representable top bound fails and clears stale accepted output");
    }

    for (int32_t extreme : {INT32_MIN, INT32_MAX}) {
        position = {1, 2, 3, true};
        expect(!from_hotspot(extreme, 0, 47, 47, position) && cleared(position),
            "32-bit extreme input cannot overflow hotspot subtraction or narrow into a valid cursor");
        position = {1, 2, 3, true};
        expect(!from_top_left(0, extreme, true, position) && cleared(position),
            "native input outside signed16 cannot silently truncate");
        expect(from_hotspot(-1, extreme, 47, 47, position) && cleared(position),
            "GDI hide sentinel ignores offscreen Y and produces explicit hidden state");
        expect(from_top_left(extreme, extreme, false, position) && cleared(position),
            "native hide ignores coordinates without performing narrowing");
    }

    for (int32_t invalid : {-1, 48, INT32_MIN, INT32_MAX}) {
        expect(!from_hotspot(0, 0, invalid, 0, position) && cleared(position), "negative or excessive horizontal hotspot is rejected");
        expect(!from_hotspot(0, 0, 0, invalid, position) && cleared(position), "negative or excessive vertical hotspot is rejected");
    }
}

void check_exact_pixels() {
    namespace shared = lightning::cursor;
    std::array<uint8_t, LIGHTNING_SCANOUT_CURSOR_SHAPE_BYTES> pixels{};
    shared::Layout mono{9, 2, 4, 8, 1, shared::Format::monochrome};
    constexpr size_t mono_bytes = 16;
    const auto exact = [&](const shared::Layout& layout, size_t bytes) {
        return shared::exact_shape(layout, LIGHTNING_SCANOUT_CURSOR_MAX_DIMENSION, pixels.data(), bytes);
    };
    expect(exact(mono, mono_bytes), "canonical host checker accepts opaque monochrome black");
    pixels[8] = 0x80;
    expect(exact(mono, mono_bytes), "monochrome white is exactly representable");
    pixels[8] = 0;
    pixels[0] = 0x80;
    expect(exact(mono, mono_bytes), "monochrome AND-only transparency is exact");
    for (uint32_t row = 0; row < mono.height; ++row) {
        for (uint32_t x = 0; x < mono.width; ++x) {
            pixels.fill(0);
            const uint8_t bit = static_cast<uint8_t>(0x80U >> (x % 8));
            pixels[row * mono.pitch + x / 8] = bit;
            pixels[mono.height * mono.pitch + row * mono.pitch + x / 8] = bit;
            expect(!exact(mono, mono_bytes), "background inversion at every actual pixel requests exact software fallback");
        }
    }
    pixels.fill(0xff);
    for (uint32_t row = 0; row < mono.height; ++row) {
        for (uint32_t x = 0; x < mono.width; ++x) {
            pixels[row * mono.pitch + x / 8] &= static_cast<uint8_t>(~(0x80U >> (x % 8)));
        }
    }
    expect(exact(mono, mono_bytes), "unused row padding and trailing bits cannot falsely trigger XOR fallback");
    expect(!exact(mono, mono_bytes - 1), "canonical raw checker rejects missing last AND/XOR plane byte");
    auto invalid = mono;
    invalid.hot_x = mono.width;
    expect(!exact(invalid, mono_bytes), "canonical raw classifier rejects out-of-shape hotspot before scanning");
    invalid = mono;
    invalid.pitch = 5;
    expect(!exact(invalid, pixels.size()), "canonical raw classifier rejects unaligned pitch");
    invalid = mono;
    invalid.height = UINT32_MAX;
    expect(!exact(invalid, pixels.size()), "canonical maximum stops excessive row arithmetic");
    expect(!shared::exact_shape(mono, LIGHTNING_SCANOUT_CURSOR_MAX_DIMENSION, nullptr, mono_bytes),
        "null pixel input cannot be scanned");
    invalid = mono;
    invalid.format = static_cast<shared::Format>(99);
    expect(!exact(invalid, mono_bytes), "unknown internal format cannot be silently classified as ordinary color");

    shared::Layout masked{2, 1, 12, 0, 0, shared::Format::masked_color};
    pixels.fill(0);
    for (uint8_t alpha : {uint8_t{0x80}, uint8_t{0xff}}) {
        for (size_t channel = 0; channel < 3; ++channel) {
            pixels.fill(0);
            pixels[channel] = 1;
            pixels[3] = alpha;
            expect(!exact(masked, masked.pitch), "masked background XOR rejects every nonzero RGB channel at inversion threshold");
        }
        pixels.fill(0);
        pixels[3] = alpha;
        expect(exact(masked, masked.pitch), "masked XOR-zero is exact transparent rather than inversion");
    }
    for (uint8_t alpha : {uint8_t{0}, uint8_t{1}, uint8_t{0x7f}}) {
        pixels.fill(0);
        pixels[0] = 0xff;
        pixels[3] = alpha;
        expect(exact(masked, masked.pitch), "masked replacement RGB below XOR threshold stays exact");
    }
    pixels.fill(0);
    for (size_t byte = 8; byte < masked.pitch; ++byte) { pixels[byte] = 0xff; }
    expect(exact(masked, masked.pitch), "masked color padding pixels are never treated as visible XOR");
    masked.format = shared::Format::color;
    pixels[0] = 0xff;
    pixels[3] = 0x80;
    expect(exact(masked, masked.pitch), "ordinary alpha-color data is not misclassified as masked XOR");
    expect(!exact(masked, masked.pitch - 1), "padded color storage must still contain its whole last row");
}

void check_alpha_and_viewport() {
    namespace shared = lightning::cursor;
    for (uint32_t alpha = 0; alpha <= 255; ++alpha) {
        expect(shared::straight_channel(0, static_cast<uint8_t>(alpha)) == 0,
            "transparent RGB remains zero during premultiplied cursor normalization");
        expect(shared::straight_channel(static_cast<uint8_t>(alpha), static_cast<uint8_t>(alpha)) ==
            (alpha == 0 ? 0 : 255), "maximum valid premultiplied channel maps to saturated straight color");
        expect(shared::straight_channel(static_cast<uint8_t>(alpha), 255) == alpha,
            "opaque alpha cursor channels remain byte-exact");
    }
    expect(shared::straight_channel(128, 128) == 255 && shared::straight_channel(64, 128) == 128 &&
        shared::straight_channel(12, 0) == 0 && shared::straight_channel(255, 1) == 255,
        "alpha conversion uses nearest inverse and bounded clamping for malformed premultiplied input");

    int32_t result = 123;
    expect(shared::viewport_axis(0, 1920, 1920, 1.0, result) && result == 0,
        "native one-to-one viewport preserves origin");
    expect(shared::viewport_axis(1919, 1920, 1920, 1.0, result) && result == 1919,
        "native one-to-one viewport preserves final pixel");
    expect(shared::viewport_axis(500, 1000, 1400, 1.0, result) && result == 700,
        "centered letterboxing adds the same integer offset as GTK input mapping");
    expect(shared::viewport_axis(10, 100, 250, 2.0, result) && result == 45,
        "enlarged viewport applies scale and letterbox together");
    expect(shared::viewport_axis(-10, 100, 50, 0.5, result) && result == -5,
        "reduced viewport retains fully negative offscreen coordinates");
    expect(shared::viewport_axis(-1, 100, 50, 0.5, result) && result == -1,
        "negative fractional position cannot truncate onto visible left edge");
    expect(shared::viewport_axis(100, 100, 50, 0.5, result) && result == 50,
        "position beyond final guest pixel is not clamped onto visible edge");
    for (double scale : {-1.0, 0.0, 1025.0, std::numeric_limits<double>::infinity(),
                         std::numeric_limits<double>::quiet_NaN()}) {
        result = 123;
        expect(!shared::viewport_axis(1, 100, 100, scale, result) && result == 123,
            "invalid or nonfinite viewport cannot overwrite a prior published coordinate");
    }
    expect(!shared::viewport_axis(0, 0, 100, 1.0, result) &&
        !shared::viewport_axis(0, 100, 0, 1.0, result), "empty canvas/widget is not a cursor viewport");
    expect(!shared::viewport_axis(INT32_MAX, 100, 100, 1024.0, result) &&
        !shared::viewport_axis(INT32_MIN, 100, 100, 1024.0, result),
        "viewport scaling cannot overflow either signed coordinate limit");
    expect(!shared::viewport_axis(0, UINT32_MAX, UINT32_MAX, 1.0, result),
        "integer GTK extent must fit its signed native coordinate type");
}

bool near(double actual, double expected) {
    const double difference = actual - expected;
    return difference >= -0.000000001 && difference <= 0.000000001;
}

lightning::cursor::OverlayInput overlay_example() {
    lightning::cursor::OverlayInput input;
    input.hotspot_x = 30;
    input.hotspot_y = 20;
    input.shape_width = 10;
    input.shape_height = 8;
    input.hot_x = 3;
    input.hot_y = 2;
    input.canvas_width = 100;
    input.canvas_height = 80;
    input.widget_width = 200;
    input.widget_height = 200;
    input.device_scale = 1;
    input.scale_x = 1.0;
    input.scale_y = 1.0;
    return input;
}

bool empty_overlay(const lightning::cursor::OverlayPlacement& output) {
    return !output.visible && output.x0 == 0 && output.y0 == 0 && output.x1 == 0 && output.y1 == 0 &&
        output.u0 == 0 && output.v0 == 0 && output.u1 == 0 && output.v1 == 0;
}

void check_overlay_examples() {
    namespace shared = lightning::cursor;
    auto input = overlay_example();
    shared::OverlayPlacement output;
    expect(shared::overlay_placement(input, output) && output.visible && output.framebuffer_width == 200 &&
        output.framebuffer_height == 200 && near(output.x0, 77) && near(output.x1, 87) &&
        near(output.y0, 114) && near(output.y1, 122),
        "hotspot subtraction and centered letterbox produce physical bottom-left quad");
    expect(near(output.u0, 0) && near(output.u1, 1) && near(output.v0, 1) && near(output.v1, 0),
        "top-row-at-v0 upload maps asymmetric source top to upper destination edge");
    input.texture_top_at_v0 = false;
    expect(shared::overlay_placement(input, output) && near(output.v0, 0) && near(output.v1, 1) &&
        near(output.y0, 114) && near(output.y1, 122), "opposite texture origin changes V orientation without flipping screen position");
    input = overlay_example();
    input.device_scale = 2;
    expect(shared::overlay_placement(input, output) && output.framebuffer_width == 400 && output.framebuffer_height == 400 &&
        near(output.x0, 154) && near(output.x1, 174) && near(output.y0, 228) && near(output.y1, 244),
        "HiDPI converts letterbox and shape to framebuffer pixels exactly once");
    input = overlay_example();
    input.hotspot_x = 0;
    input.hotspot_y = 0;
    expect(shared::overlay_placement(input, output) && output.visible && near(output.x0, 50) && near(output.x1, 57) &&
        near(output.y0, 134) && near(output.y1, 140) && near(output.u0, 0.3) && near(output.u1, 1) &&
        near(output.v0, 1) && near(output.v1, 0.25),
        "negative top-left clips actual guest pixels and corresponding cursor UV rather than shifting hotspot");
    input.texture_top_at_v0 = false;
    expect(shared::overlay_placement(input, output) && near(output.v0, 0) && near(output.v1, 0.75),
        "top-clipped cursor UV respects opposite source orientation");
    input = overlay_example();
    input.hotspot_x = 100;
    input.hotspot_y = 80;
    expect(shared::overlay_placement(input, output) && output.visible && near(output.x0, 147) && near(output.x1, 150) &&
        near(output.y0, 60) && near(output.y1, 62) && near(output.u0, 0) && near(output.u1, 0.3) &&
        near(output.v0, 0.25) && near(output.v1, 0), "right-bottom clip retains correct top-left source subsection");
    input = overlay_example();
    input.hotspot_x = -1;
    input.hotspot_y = -1;
    expect(shared::overlay_placement(input, output) && output.visible && near(output.x1 - output.x0, 6) &&
        near(output.y1 - output.y0, 5) && near(output.u0, 0.4) && near(output.v1, 0.375),
        "overlay coordinates have no GDI hide sentinel; explicit plane visibility owns hiding");
    input = overlay_example();
    input.hotspot_x = 20;
    input.hotspot_y = 15;
    input.widget_width = 50;
    input.widget_height = 40;
    input.scale_x = 2;
    input.scale_y = 2;
    expect(shared::overlay_placement(input, output) && output.visible && near(output.x0, 34) && near(output.x1, 50) &&
        near(output.y0, 0) && near(output.y1, 14) && near(output.u1, 0.8) && near(output.v0, 0.875),
        "widget cropping clips an enlarged canvas and source UV before GL submission");
    input = overlay_example();
    input.hotspot_x = 0;
    input.hotspot_y = 0;
    input.widget_width = 50;
    input.widget_height = 40;
    input.scale_x = 0.5;
    input.scale_y = 0.5;
    input.device_scale = 2;
    expect(shared::overlay_placement(input, output) && output.visible && near(output.x0, 0) && near(output.x1, 7) &&
        near(output.y0, 74) && near(output.y1, 80) && near(output.u0, 0.3) && near(output.v1, 0.25),
        "fractional viewport keeps subpixel clipping until physical HiDPI conversion");
    input = overlay_example();
    input.widget_width = 201;
    input.widget_height = 201;
    expect(shared::overlay_placement(input, output) && near(output.x0, 77) && near(output.x1, 87) &&
        near(output.y0, 115) && near(output.y1, 123), "odd letterbox extent follows the existing integer GTK origin");
    input = overlay_example();
    input.scale_x = 2;
    input.scale_y = 0.5;
    input.widget_height = 100;
    expect(shared::overlay_placement(input, output) && near(output.x0, 54) && near(output.x1, 74) &&
        near(output.y0, 57) && near(output.y1, 61), "independent axis scales preserve cursor orientation and canvas alignment");

    for (int32_t outside : {INT32_MIN, -100, 200, INT32_MAX}) {
        input = overlay_example();
        input.hotspot_x = outside;
        expect(shared::overlay_placement(input, output) && empty_overlay(output) &&
            output.framebuffer_width == 200 && output.framebuffer_height == 200,
            "fully offscreen horizontal cursor is a valid hidden overlay with no stale vertices");
        input = overlay_example();
        input.hotspot_y = outside;
        expect(shared::overlay_placement(input, output) && empty_overlay(output),
            "fully offscreen vertical cursor is not a failed movement");
    }
}

void check_overlay_bounds() {
    namespace shared = lightning::cursor;
    shared::OverlayPlacement output;
    for (uint32_t fault = 0; fault < 14; ++fault) {
        auto input = overlay_example();
        switch (fault) {
        case 0: input.shape_width = 0; break;
        case 1: input.shape_height = 0; break;
        case 2: input.hot_x = input.shape_width; break;
        case 3: input.hot_y = input.shape_height; break;
        case 4: input.canvas_width = 0; break;
        case 5: input.canvas_height = UINT32_MAX; break;
        case 6: input.widget_width = UINT32_MAX; break;
        case 7: input.widget_height = 0; break;
        case 8: input.device_scale = 0; break;
        case 9: input.device_scale = UINT32_MAX; break;
        case 10: input.shape_width = UINT32_MAX; break;
        case 11: input.shape_height = UINT32_MAX; break;
        case 12: input.scale_x = std::numeric_limits<double>::quiet_NaN(); break;
        case 13: input.scale_y = std::numeric_limits<double>::infinity(); break;
        default: break;
        }
        output.visible = true;
        output.framebuffer_width = 123;
        output.framebuffer_height = 456;
        output.x0 = 1;
        output.v1 = 1;
        expect(!shared::overlay_placement(input, output) && empty_overlay(output) &&
            output.framebuffer_width == 0 && output.framebuffer_height == 0,
            "invalid overlay geometry clears all old draw coordinates and dimensions");
    }

    for (double scale : {-1.0, 0.0, 1025.0}) {
        auto input = overlay_example();
        input.scale_x = scale;
        expect(!shared::overlay_placement(input, output) && empty_overlay(output), "unsupported horizontal scale never forms a GL quad");
        input = overlay_example();
        input.scale_y = scale;
        expect(!shared::overlay_placement(input, output) && empty_overlay(output), "unsupported vertical scale never forms a GL quad");
    }
    const double scales[]{0.5, 1.0, 2.0};
    const double offsets_x[]{75, 50, 0};
    const double offsets_y[]{80, 60, 20};
    for (uint32_t scale_index = 0; scale_index < 3; ++scale_index) {
        for (uint32_t factor : {1U, 2U}) {
            for (bool top_at_v0 : {false, true}) {
                for (int32_t x : {-10, -1, 0, 3, 49, 99, 100, 110}) {
                    for (int32_t y : {-10, -1, 0, 2, 39, 79, 80, 90}) {
                        auto input = overlay_example();
                        input.hotspot_x = x;
                        input.hotspot_y = y;
                        input.scale_x = scales[scale_index];
                        input.scale_y = scales[scale_index];
                        input.device_scale = factor;
                        input.texture_top_at_v0 = top_at_v0;
                        expect(shared::overlay_placement(input, output), "bounded signed cursor grid always produces valid placement or clipping");
                        if (!output.visible) {
                            expect(empty_overlay(output), "fully clipped grid point has no stale draw rectangle");
                            continue;
                        }
                        expect(output.x0 >= 0 && output.x0 < output.x1 && output.x1 <= output.framebuffer_width &&
                            output.y0 >= 0 && output.y0 < output.y1 && output.y1 <= output.framebuffer_height,
                            "every visible grid quad remains inside the physical framebuffer");
                        expect(output.u0 >= 0 && output.u0 < output.u1 && output.u1 <= 1 &&
                            output.v0 >= 0 && output.v0 <= 1 && output.v1 >= 0 && output.v1 <= 1 &&
                            (top_at_v0 ? output.v0 > output.v1 : output.v0 < output.v1),
                            "all source UVs remain finite, bounded and correctly oriented");
                        const double source_top = (top_at_v0 ? output.v1 : 1 - output.v1) * input.shape_height;
                        const double guest_left = static_cast<double>(x) - input.hot_x + output.u0 * input.shape_width;
                        const double guest_top = static_cast<double>(y) - input.hot_y + source_top;
                        expect(near(output.x0 / factor, offsets_x[scale_index] + guest_left * input.scale_x) &&
                            near((output.framebuffer_height - output.y1) / factor,
                                offsets_y[scale_index] + guest_top * input.scale_y),
                            "destination/source correspondence preserves hotspot through clipping, scale and orientation");
                    }
                }
            }
        }
    }
}

void check_overlay_orientation_samples() {
    namespace shared = lightning::cursor;
    constexpr uint32_t red = 0xffff0000U;
    constexpr uint32_t green = 0xff00ff00U;
    constexpr uint32_t blue = 0xff0000ffU;
    constexpr uint32_t yellow = 0xffffff00U;
    for (bool top_at_v0 : {false, true}) {
        const uint32_t texture_top_at_zero[]{red, green, blue, yellow};
        const uint32_t texture_top_at_one[]{blue, yellow, red, green};
        const uint32_t* pixels = top_at_v0 ? texture_top_at_zero : texture_top_at_one;
        auto input = overlay_example();
        input.shape_width = 2;
        input.shape_height = 2;
        input.hot_x = 0;
        input.hot_y = 0;
        input.device_scale = 2;
        input.texture_top_at_v0 = top_at_v0;
        shared::OverlayPlacement output;
        expect(shared::overlay_placement(input, output) && output.visible,
            "asymmetric color fixture produces a visible physical cursor quad");
        const auto sample = [&](double horizontal, double vertical) {
            const double u = output.u0 + (output.u1 - output.u0) * horizontal;
            const double v = output.v0 + (output.v1 - output.v0) * vertical;
            if (!(u >= 0 && u < 1 && v >= 0 && v < 1)) { return uint32_t{0}; }
            const uint32_t x = static_cast<uint32_t>(u * 2);
            const uint32_t y = static_cast<uint32_t>(v * 2);
            return x < 2 && y < 2 ? pixels[y * 2 + x] : 0;
        };
        expect(sample(0.25, 0.75) == red && sample(0.75, 0.75) == green &&
            sample(0.25, 0.25) == blue && sample(0.75, 0.25) == yellow,
            "both source origins preserve all four asymmetric screen corners after GL Y conversion");
        input.hot_x = 1;
        input.hot_y = 1;
        input.hotspot_x = 0;
        input.hotspot_y = 0;
        expect(shared::overlay_placement(input, output) && output.visible && sample(0.5, 0.5) == yellow,
            "top-left guest clipping selects lower-right source pixel without orientation reversal");
    }
}
} // namespace

int main() {
    check_shape_layouts();
    check_positions();
    check_exact_pixels();
    check_alpha_and_viewport();
    check_overlay_examples();
    check_overlay_bounds();
    check_overlay_orientation_samples();
    std::printf("Lightning cursor bounds: %u checks, %u failures; native/host publication untested\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

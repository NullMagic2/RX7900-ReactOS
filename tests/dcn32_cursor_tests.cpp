/*
Module: rx7900.tests.dcn32_cursor
Purpose: Verify DCN 3.2 cursor image encoding, position arithmetic, clocks and locked register programming.
Created: 2026-10-22
Architecture: Register sequences run through the production RegisterIo path into
the shared synthetic Navi31 register file (tests/asic_test_device.hpp) at the
pinned Linux v6.12 dcn_3_2_0_offset.h indices; expected values are derived by
hand from the Linux functions named in src/display/dcn32_cursor.hpp. A
synthetic ATOM image carries dce_info. This is not hardware evidence.
*/
#include <memory>
#include <vector>

#include "asic_test_device.hpp"
#include "dcn32_cursor.hpp"

namespace {

using namespace rx7900::display;
using rx7900::ByteView;

rx7900_test::Checks checks;

void expect(bool condition, const char* contract) { checks.expect(condition, contract); }

// DMU 3.2.0 segment bases in the shared synthetic discovery (the board's values).
constexpr uint32_t dcn = 0x34c0;
constexpr uint32_t mpc = 0x9000;
constexpr uint32_t hubp_stride = 0xdc;
constexpr uint32_t dpp_stride = 0x16b;
constexpr uint32_t otg_stride = 0x80;

// CEA-861 2160p60 (594 MHz), the user's monitor mode.
constexpr Timing mode_2160{594000, 3840, 4400, 176, 88, 2160, 2250, 8, 10, positive_hsync | positive_vsync};
constexpr CursorStream stream_2160{3840, 2160, 594000, 50000};

struct Write final {
    uint32_t index;
    uint32_t value;
};

class Device final {
public:
    std::unique_ptr<rx7900_test::SimulatedNavi31> device = std::make_unique<rx7900_test::SimulatedNavi31>();
    std::vector<Write> writes;
    std::vector<uint32_t> delays;
    // Fail the write with this ordinal (0-based), or never.
    size_t fail_write = SIZE_MAX;

    uint32_t& reg(uint32_t base, uint32_t index) { return device->reg(base + index); }

    static bool read(void* context, uint32_t offset, uint32_t& value) noexcept {
        auto& self = *static_cast<Device*>(context);
        return rx7900_test::SimulatedNavi31::read(self.device.get(), offset, value);
    }

    static bool write(void* context, uint32_t offset, uint32_t value) noexcept {
        auto& self = *static_cast<Device*>(context);

        if (self.writes.size() == self.fail_write) {
            return false;
        }

        self.writes.push_back({offset / 4, value});
        self.device->reg(offset / 4) = value;
        return true;
    }

    static void delay(void* context, uint32_t microseconds) noexcept {
        static_cast<Device*>(context)->delays.push_back(microseconds);
    }

    rx7900::hardware::RegisterIo io() noexcept { return {this, rx7900_test::aperture_bytes, read, write, delay}; }

    bool wrote(uint32_t base, uint32_t index) const {
        for (const Write& entry : writes) {
            if (entry.index == base + index) {
                return true;
            }
        }

        return false;
    }
};

AdoptedScanout scanout(uint32_t pipe, uint32_t opp, uint32_t otg) {
    AdoptedScanout value{};
    value.pipe = pipe;
    value.opp = opp;
    value.otg = otg;
    value.timing = mode_2160;
    return value;
}

Dcn32CursorRegisters registers(const AdoptedScanout& value) {
    const auto binary = rx7900_test::navi31_discovery();
    Dcn32CursorRegisters output;
    validate_dcn32_cursor_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes, value, output);
    return output;
}

// Synthetic ATOM image: ROM header 2.2 at 0x100, master data table 2.1 at 0x200,
// dce_info (entry 27) at 0x300 with dce_refclk_10khz at +12.
struct Rom final {
    std::vector<uint8_t> bytes = std::vector<uint8_t>(0x400, 0);

    void u16(size_t offset, uint16_t value) {
        bytes.at(offset) = static_cast<uint8_t>(value);
        bytes.at(offset + 1) = static_cast<uint8_t>(value >> 8);
    }

    explicit Rom(uint8_t format = 4, uint16_t refclk_10khz = 10000, uint16_t table_bytes = 0x50) {
        u16(0, 0xaa55);
        u16(0x48, 0x100);
        u16(0x100, 0x30);
        bytes.at(0x102) = 2;
        bytes.at(0x103) = 2;
        rx7900::sdma::CommandBuffer(bytes.data(), bytes.size()).write_u32(0x104, 0x4d4f5441);
        u16(0x100 + 32, 0x200);
        u16(0x200, 4 + 35 * 2);
        bytes.at(0x202) = 2;
        bytes.at(0x203) = 1;
        u16(0x200 + 4 + 27 * 2, 0x300);
        u16(0x300, table_bytes);
        bytes.at(0x302) = format;
        bytes.at(0x303) = 5;
        u16(0x300 + 12, refclk_10khz);
    }

    ByteView view() const { return {bytes.data(), bytes.size()}; }
};

void discovery_and_clocks() {
    const auto binary = rx7900_test::navi31_discovery();
    const ByteView view(binary.data(), binary.size());
    Dcn32CursorRegisters output;

    expect(validate_dcn32_cursor_discovery(view, rx7900_test::aperture_bytes, scanout(0, 0, 0), output) ==
                   rx7900::hardware::DiscoveryStatus::validated &&
               output.valid() && output.pipe() == 0 && output.opp() == 0 && output.otg() == 0,
           "DMU 3.2.0 cursor registers of pipe 0 validate");
    expect(validate_dcn32_cursor_discovery(view, rx7900_test::aperture_bytes, scanout(4, 0, 0), output) ==
                   rx7900::hardware::DiscoveryStatus::invalid_argument && !output.valid(),
           "a pipe beyond the four DCN 3.2 instances is refused");
    expect(validate_dcn32_cursor_discovery(view, (dcn + 0x1000) * 4, scanout(3, 3, 3), output) ==
               rx7900::hardware::DiscoveryStatus::invalid_argument,
           "registers beyond the BAR5 aperture are refused");
    expect(validate_dcn32_cursor_discovery(ByteView(nullptr, 0), rx7900_test::aperture_bytes, scanout(0, 0, 0),
                                           output) != rx7900::hardware::DiscoveryStatus::validated,
           "missing discovery is refused");

    uint32_t khz = 0;
    expect(read_dce_refclk_khz(Rom().view(), khz) && khz == 100000,
           "dce_info v4.x dce_refclk_10khz * 10 is the crystal (100 MHz expected on DCN32)");
    expect(!read_dce_refclk_khz(Rom(3).view(), khz), "a dce_info table of another format is refused");
    expect(!read_dce_refclk_khz(Rom(4, 0).view(), khz), "a zero crystal frequency fails closed");
    expect(!read_dce_refclk_khz(Rom(4, 10000, 12).view(), khz), "a table too short for dce_refclk is refused");

    Device device;
    const auto tokens = registers(scanout(0, 0, 0));
    device.reg(dcn, 0x0525) = 0x00001002u; // timer enabled, REFDIV 2
    expect(read_dchub_refclk_khz(device.io(), tokens, 100000, khz) && khz == 50000,
           "hubbub2_get_dchub_ref_freq: REFDIV 2 halves the 100 MHz reference");
    device.reg(dcn, 0x0525) = 0x00001001u;
    expect(!read_dchub_refclk_khz(device.io(), tokens, 100000, khz),
           "an undivided 100 MHz DCHUB reference is outside 40..60 MHz and refused");
    device.reg(dcn, 0x0525) = 0x00000002u;
    expect(!read_dchub_refclk_khz(device.io(), tokens, 100000, khz), "a stopped DCHUB global timer is refused");
    expect(device.writes.empty(), "clock discovery writes no register");
}

uint32_t at(const CursorImage& image, uint32_t x, uint32_t y) { return image.pixels[y * 64 + x]; }

void image_encoding() {
    auto image = std::make_unique<CursorImage>();

    // 8x2 monochrome: row 0 AND 0x0f XOR 0x33, row 1 AND 0xff XOR 0x00.
    // Row 0 pixel 2 has AND 1 / XOR 1: an inverting pixel.
    std::vector<uint8_t> mono{0x0f, 0xff, 0x33, 0x00};
    CursorSource source{};
    source.format = CursorSourceFormat::monochrome;
    source.width = 8;
    source.height = 2;
    source.mask = ByteView(mono.data(), mono.size());
    source.mask_stride = 1;
    expect(build_cursor_image(source, *image) == CursorImageStatus::built &&
               image->mode == CursorMode::color_1bit_and && image->width == 8 && image->height == 2,
           "a monochrome pointer with an inverting pixel needs the 1-bit-AND mode");
    expect(at(*image, 0, 0) == 0x00000000u && at(*image, 2, 0) == 0x00ffffffu && at(*image, 4, 0) == 0x80000000u &&
               at(*image, 6, 0) == 0x80ffffffu && at(*image, 0, 1) == 0x80000000u &&
               at(*image, 8, 0) == 0x80000000u && at(*image, 63, 63) == 0x80000000u,
           "1-bit-AND words: black, white, transparent, invert; padding is AND 1 with color 0");

    // Without the inverting pixel the same shape is exact premultiplied ARGB.
    mono = {0x0f, 0xff, 0x30, 0x00};
    source.mask = ByteView(mono.data(), mono.size());
    expect(build_cursor_image(source, *image) == CursorImageStatus::built &&
               image->mode == CursorMode::premultiplied_alpha && at(*image, 0, 0) == 0xff000000u &&
               at(*image, 2, 0) == 0xffffffffu && at(*image, 4, 0) == 0 && at(*image, 0, 1) == 0 &&
               at(*image, 9, 0) == 0,
           "a monochrome pointer without inversion is opaque black/white over transparency");

    // 2x1 masked color: pixel 0 AND 0 (opaque red), pixel 1 AND 1 with color 0.
    std::vector<uint8_t> mask{0x40, 0, 0, 0};
    std::vector<uint8_t> color{0x00, 0x00, 0xff, 0x7f, 0x00, 0x00, 0x00, 0x00};
    CursorSource masked{};
    masked.format = CursorSourceFormat::masked_color;
    masked.width = 2;
    masked.height = 1;
    masked.mask = ByteView(mask.data(), mask.size());
    masked.mask_stride = 4;
    masked.color = ByteView(color.data(), color.size());
    masked.color_stride = 8;
    expect(build_cursor_image(masked, *image) == CursorImageStatus::built &&
               image->mode == CursorMode::premultiplied_alpha && at(*image, 0, 0) == 0xffff0000u &&
               at(*image, 1, 0) == 0,
           "mask + color without XOR pixels is exact ARGB; the X byte is ignored");
    color[4] = 0x10;
    expect(build_cursor_image(masked, *image) == CursorImageStatus::built &&
               image->mode == CursorMode::color_1bit_and && at(*image, 0, 0) == 0x00ff0000u &&
               at(*image, 1, 0) == 0x80000010u,
           "an XOR color under AND 1 selects 1-bit-AND (screen XOR color)");

    std::vector<uint8_t> argb{0x10, 0x20, 0x30, 0x40, 0xff, 0xff, 0xff, 0xff};
    CursorSource alpha{};
    alpha.format = CursorSourceFormat::premultiplied_alpha;
    alpha.width = 2;
    alpha.height = 1;
    alpha.color = ByteView(argb.data(), argb.size());
    alpha.color_stride = 8;
    expect(build_cursor_image(alpha, *image) == CursorImageStatus::built &&
               image->mode == CursorMode::premultiplied_alpha && at(*image, 0, 0) == 0x40302010u &&
               at(*image, 1, 0) == 0xffffffffu && at(*image, 2, 0) == 0,
           "premultiplied ARGB is copied unchanged with transparent padding");
    alpha.format = CursorSourceFormat::straight_alpha;
    expect(build_cursor_image(alpha, *image) == CursorImageStatus::built &&
               image->mode == CursorMode::straight_alpha && at(*image, 0, 0) == 0x40302010u,
           "straight alpha selects CURSOR_MODE_COLOR_UN_PRE_MULTIPLIED_ALPHA");

    image->mode = CursorMode::mono;
    image->pixels[0] = 0x12345678u;
    alpha.width = 65;
    expect(build_cursor_image(alpha, *image) == CursorImageStatus::unsupported_size &&
               image->pixels[0] == 0x12345678u && image->mode == CursorMode::mono,
           "a 65-pixel shape exceeds dc->caps.max_cursor_size and leaves the output unchanged");
    alpha.width = 2;
    alpha.height = 2;
    expect(build_cursor_image(alpha, *image) == CursorImageStatus::truncated,
           "a color plane shorter than height rows is refused");
    alpha.height = 1;
    alpha.color_stride = 4;
    expect(build_cursor_image(alpha, *image) == CursorImageStatus::truncated,
           "a color stride below width * 4 is refused");
    source.height = 3;
    expect(build_cursor_image(source, *image) == CursorImageStatus::truncated,
           "a monochrome mask without both planes is refused");
    source.height = 0;
    expect(build_cursor_image(source, *image) == CursorImageStatus::unsupported_size, "zero height is refused");

    // A full 64x64 straight shape fills every word.
    std::vector<uint8_t> full(64 * 64 * 4, 0xaa);
    CursorSource large{};
    large.format = CursorSourceFormat::straight_alpha;
    large.width = 64;
    large.height = 64;
    large.color = ByteView(full.data(), full.size());
    large.color_stride = 256;
    expect(build_cursor_image(large, *image) == CursorImageStatus::built && at(*image, 63, 63) == 0xaaaaaaaau,
           "a 64 x 64 shape fills the whole pitch-64 image");
}

void position_arithmetic() {
    CursorPlacement placement{};

    expect(place_cursor({true, 100, 200}, stream_2160, 64, 64, placement) && placement.enable &&
               placement.x == 100 && placement.y == 200 && placement.hot_x == 0 && placement.hot_y == 0 &&
               placement.dst_x_offset == 100u * 50000u / 594000u,
           "an onscreen top-left programs the position, no hotspot, DST_X_OFFSET = x * refclk / pixclk");
    expect(place_cursor({true, -10, -63}, stream_2160, 64, 64, placement) && placement.enable &&
               placement.x == 0 && placement.y == 0 && placement.hot_x == 10 && placement.hot_y == 63 &&
               placement.dst_x_offset == 0,
           "negative coordinates move into the hotspot (amdgpu_dm min(-v, max - 1))");
    expect(place_cursor({true, -64, 10}, stream_2160, 64, 64, placement) && !placement.enable && placement.x == 0 &&
               placement.hot_x == 0,
           "x <= -max_cursor_width disables with a zeroed position");
    expect(place_cursor({true, 10, -64}, stream_2160, 64, 64, placement) && !placement.enable,
           "y <= -max_cursor_height disables");
    expect(place_cursor({true, 3839, 2159}, stream_2160, 64, 64, placement) && placement.enable &&
               placement.x == 3839 && placement.y == 2159,
           "the last onscreen pixel stays visible");
    expect(place_cursor({true, 3840, 100}, stream_2160, 64, 64, placement) && !placement.enable &&
               placement.x == 3840,
           "beyond the right edge hubp2/dpp1 clear the enable but keep the position");
    expect(place_cursor({true, 100, 2160}, stream_2160, 64, 64, placement) && !placement.enable,
           "beyond the bottom edge disables");
    expect(place_cursor({false, 100, 100}, stream_2160, 64, 64, placement) && !placement.enable &&
               placement.x == 0 && placement.y == 0 && placement.dst_x_offset == 0,
           "a hidden request is the zeroed Linux position");
    expect(place_cursor({true, 20000, 100}, {20000, 2160, 594000, 50000}, 64, 64, placement) &&
               placement.x == (20000u & 0x3fffu) && placement.dst_x_offset == ((20000ull * 50000 / 594000) & 0x1fff),
           "position and DST_X_OFFSET are masked to their 14/13-bit fields as REG_SET does");
    expect(!place_cursor({true, 0, 0}, {3840, 2160, 0, 50000}, 64, 64, placement) &&
               !place_cursor({true, 0, 0}, stream_2160, 65, 64, placement),
           "an unknown pixel clock or an oversized cursor is refused");

    uint32_t field = 9;
    expect(cursor_pitch_field(64, field) && field == 0 && cursor_pitch_field(128, field) && field == 1 &&
               cursor_pitch_field(256, field) && field == 2 && !cursor_pitch_field(32, field),
           "hubp1_get_cursor_pitch fields; other pitches are refused");
    expect(cursor_lines_per_chunk(64, CursorMode::premultiplied_alpha) == 3 &&
               cursor_lines_per_chunk(32, CursorMode::color_1bit_and) == 4 &&
               cursor_lines_per_chunk(128, CursorMode::straight_alpha) == 2 &&
               cursor_lines_per_chunk(256, CursorMode::premultiplied_alpha) == 1 &&
               cursor_lines_per_chunk(64, CursorMode::mono) == 4,
           "hubp2_get_lines_per_chunk: 8 lines for 33..64 pixels, 16 for mono");
    expect(!cursor_uses_mall(64, 64, CursorMode::premultiplied_alpha) &&
               cursor_uses_mall(64, 65, CursorMode::premultiplied_alpha) && !cursor_uses_mall(128, 128, CursorMode::mono) &&
               cursor_uses_mall(1, 65, CursorMode::straight_alpha),
           "USE_MALL_FOR_CURSOR above 16 KiB with the width rounded to 64");
}

void vupdate_keepout() {
    VupdateWindow window{};
    // asic_blank_end = 2250 - (2160 + 8) = 82; vupdate = 82 - 40 + 1 = 43.
    expect(cursor_vupdate_window(mode_2160, 40, window) && window.start_line == 43 && window.end_line == 45,
           "dcn10_get_vupdate_offset_from_vsync / calc_vupdate_position");
    // us_per_line = 4400 * 10000 / 5940000 = 7.
    expect(cursor_lock_delay_microseconds(mode_2160, window, 40) == 3 * 7 + 3 * 7,
           "three lines before VUPDATE: wait until it completes");
    expect(cursor_lock_delay_microseconds(mode_2160, window, 44) == 3 * 7, "inside VUPDATE: wait out the window");
    expect(cursor_lock_delay_microseconds(mode_2160, window, 30) == 0, "more than 70 us before VUPDATE: no stall");
    expect(cursor_lock_delay_microseconds(mode_2160, window, 100) == 0, "front porch after VUPDATE: no stall");
    // vupdate = 82 - 100 + 1 = -17: start = -17 + (0 + 1) * 2250 - 1 = 2232.
    expect(cursor_vupdate_window(mode_2160, 100, window) && window.start_line == 2232 && window.end_line == 2234,
           "a negative VUPDATE offset wraps as Linux computes it");
    Timing zero = mode_2160;
    zero.v_front_porch = 0;
    expect(cursor_vupdate_window(zero, 0, window) && window.start_line == 2250 - 2161 + 1,
           "apply_front_porch_workaround keeps at least one front-porch line");
    Timing bad = mode_2160;
    bad.v_total = 0;
    expect(!cursor_vupdate_window(bad, 0, window), "an empty timing is refused");
}

struct Programmed final {
    Device device;
    AdoptedScanout target;
    Dcn32CursorRegisters tokens;
    Dcn32Cursor cursor;

    Programmed(uint32_t pipe, uint32_t opp, uint32_t otg) : target(scanout(pipe, opp, otg)), tokens(registers(target)) {
        cursor.initialize(tokens, device.io(), mode_2160, stream_2160);
    }

    uint32_t& hubp(uint32_t index) { return device.reg(dcn, index + target.pipe * hubp_stride); }
    uint32_t& dpp(uint32_t index) { return device.reg(dcn, index + target.pipe * dpp_stride); }
    uint32_t& otg(uint32_t index) { return device.reg(dcn, index + target.otg * otg_stride); }
    uint32_t lock_index() const { return mpc + 0x03ab + target.opp * 5; }
};

void register_programming() {
    Programmed state(1, 2, 3);
    state.hubp(0x0678) = 0x00000004u;  // CURSOR_REQ_MODE (preserved by RMW)
    state.hubp(0x05f6) = 0x00000001u;  // USE_MALL_SEL (preserved)
    state.dpp(0x0cf1) = 0x00000084u;   // PIX_INV_MODE | PIXEL_ALPHA_MOD_EN (preserved)
    state.otg(0x1b86) = 40;            // VSTARTUP_START
    state.otg(0x1b4a) = 1000;          // far from VUPDATE: no stall

    expect(state.cursor.set_position({true, 10, 10}) == CursorStatus::ready && state.device.writes.empty(),
           "before attributes the hubp2 guard writes nothing");

    const uint64_t mc = rx7900_test::fb_base + 0x12345000ULL;
    expect(state.cursor.set_attributes({mc, CursorMode::premultiplied_alpha}) == CursorStatus::ready,
           "attributes program");
    const auto& writes = state.device.writes;
    expect(writes.size() >= 2 && writes.front().index == state.lock_index() && writes.front().value == 1 &&
               writes.back().index == state.lock_index() && writes.back().value == 0,
           "program_cursor_attributes holds CUR_VUPDATE_LOCK_SET of the OPP around every write");
    expect(state.hubp(0x067a) == static_cast<uint32_t>(mc >> 32) && state.hubp(0x0679) == static_cast<uint32_t>(mc) &&
               state.hubp(0x067b) == ((64u << 16) | 64u),
           "surface address high/low and a 64 x 64 CURSOR_SIZE");
    expect(state.hubp(0x0678) == ((2u << 8) | (0u << 16) | (3u << 24) | 0x4u),
           "CURSOR_CONTROL: mode premultiplied, pitch 64, 8 lines per chunk, no magnify, REQ_MODE kept");
    expect(state.hubp(0x065b) == 0x300u, "CURSOR_SETTINGS: DST_Y_OFFSET 0, CHUNK_HDL_ADJUST 3");
    expect(state.hubp(0x05f6) == 0x1u, "a 16 KiB cursor does not use MALL; USE_MALL_SEL kept");
    expect(state.dpp(0x0cf1) == ((2u << 4) | 0x84u), "CUR0_MODE premultiplied, no expansion, no degamma ROM");
    expect(state.dpp(0x0cf4) == 0x3c00u, "CUR0_FP_SCALE 1.0 and FP_BIAS 0 (SDR white level)");
    expect(!state.device.wrote(dcn, 0x0cf2 + state.target.pipe * dpp_stride),
           "mono colors are programmed only for CURSOR_MODE_MONO");
    expect(!state.device.wrote(dcn, 0x0678) && !state.device.wrote(dcn, 0x0cf1),
           "pipe 1 registers only: instance 0 is untouched");
    expect(state.device.delays.empty(), "no keepout stall away from VUPDATE");

    state.device.writes.clear();
    expect(state.cursor.set_position({true, -5, 300}) == CursorStatus::ready &&
               (state.hubp(0x0678) & 1u) == 1u && state.hubp(0x067c) == ((0u << 16) | 300u) &&
               state.hubp(0x067d) == ((5u << 16) | 0u) && state.hubp(0x067f) == 0 && (state.dpp(0x0cf1) & 1u) == 1u,
           "a visible position enables HUBP and DPP cursors with the hotspot shift");
    expect(state.device.writes.front().index == state.lock_index() && state.device.writes.back().value == 0,
           "program_cursor_position is locked too");
    expect(state.cursor.report().enabled && state.cursor.report().placement.hot_x == 5, "report follows");

    // Attributes lost (power gating): enabling rewrites them first.
    state.hubp(0x0679) = 0;
    expect(state.cursor.set_position({true, 50, 60}) == CursorStatus::ready &&
               state.hubp(0x0679) == static_cast<uint32_t>(mc) && state.cursor.report().reprogrammed_attributes == 1,
           "CURSOR_SURFACE_ADDRESS reading 0 while enabling reprograms the attributes");

    // Lock keepout: three lines before VUPDATE stalls 42 us.
    state.otg(0x1b4a) = 40;
    expect(state.cursor.set_position({true, 51, 60}) == CursorStatus::ready && state.device.delays.size() == 1 &&
               state.device.delays[0] == 42 && state.cursor.report().lock_delays == 1,
           "delay_cursor_until_vupdate stalls before taking the lock near VUPDATE");

    expect(state.cursor.hide() == CursorStatus::ready && (state.hubp(0x0678) & 1u) == 0 &&
               (state.dpp(0x0cf1) & 1u) == 0 && state.hubp(0x067c) == 0 && state.hubp(0x067d) == 0 &&
               !state.cursor.report().enabled,
           "hide clears both enables with the zeroed position");

    expect(state.cursor.set_attributes({mc, CursorMode::color_1bit_and}) == CursorStatus::ready &&
               ((state.hubp(0x0678) >> 8) & 7u) == 1u && ((state.dpp(0x0cf1) >> 4) & 7u) == 1u,
           "the 1-bit-AND mode reaches CURSOR_MODE and CUR0_MODE");
    expect(state.cursor.set_attributes({mc + 0x100, CursorMode::premultiplied_alpha}) ==
                   CursorStatus::invalid_argument &&
               state.cursor.set_attributes({0, CursorMode::premultiplied_alpha}) == CursorStatus::invalid_argument &&
               state.cursor.set_attributes({1ULL << 48, CursorMode::premultiplied_alpha}) ==
                   CursorStatus::invalid_argument,
           "unaligned, zero or beyond-48-bit addresses are refused");

    state.cursor.forget();
    state.device.writes.clear();
    expect(state.cursor.set_position({true, 1, 1}) == CursorStatus::ready && state.device.writes.empty(),
           "after forget (power loss) nothing is written until attributes are set again");
}

void failures() {
    Programmed state(0, 0, 0);
    state.device.fail_write = 3;
    expect(state.cursor.set_attributes({rx7900_test::fb_base + 0x1000, CursorMode::premultiplied_alpha}) ==
                   CursorStatus::mmio_failure &&
               state.cursor.report().status == CursorStatus::mmio_failure,
           "a failed register write makes the cursor state uncertain");
    state.device.fail_write = SIZE_MAX;
    expect(state.cursor.set_position({true, 0, 0}) == CursorStatus::mmio_failure &&
               state.cursor.set_attributes({rx7900_test::fb_base + 0x1000, CursorMode::premultiplied_alpha}) ==
                   CursorStatus::mmio_failure,
           "every later call refuses after an uncertain write");

    Dcn32Cursor cursor;
    Device device;
    expect(cursor.set_attributes({rx7900_test::fb_base, CursorMode::premultiplied_alpha}) ==
               CursorStatus::uninitialized,
           "an uninitialized cursor refuses");
    expect(cursor.initialize(Dcn32CursorRegisters{}, device.io(), mode_2160, stream_2160) ==
               CursorStatus::invalid_argument,
           "an invalid register token is refused");
    CursorStream mismatch = stream_2160;
    mismatch.width = 1920;
    expect(cursor.initialize(registers(scanout(0, 0, 0)), device.io(), mode_2160, mismatch) ==
               CursorStatus::invalid_argument,
           "a stream that is not the adopted viewport is refused");
}

} // namespace

int main() {
    discovery_and_clocks();
    image_encoding();
    position_arithmetic();
    vupdate_keepout();
    register_programming();
    failures();
    return checks.finish("dcn32_cursor");
}

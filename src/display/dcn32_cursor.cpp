/*
Module: rx7900.display.dcn32_cursor
Purpose: DCN 3.2 cursor image encoding, position arithmetic and locked register programming.
Created: 2026-10-22
Architecture: Allocation-free; register access only through hardware::RegisterIo
and the discovered DMU segment bases. Provenance and notice: dcn32_cursor.hpp.
Copyright 2012-2023 Advanced Micro Devices, Inc.
*/
#include "dcn32_cursor.hpp"

namespace rx7900::display {
namespace {

using hardware::discovery_register_offset;

constexpr uint32_t segment_dcn = 2; // BASE_IDX 2: HUBBUB, HUBP, DPP, OTG
constexpr uint32_t segment_mpc = 3; // BASE_IDX 3: MPC

// dcn_3_2_0_offset.h instance-0 indices and per-instance strides.
constexpr uint32_t hubp_stride = 0xdc;
constexpr uint32_t dpp_stride = 0x16b;
constexpr uint32_t otg_stride = 0x80;
constexpr uint32_t cursor_lock_stride = 5;
constexpr uint32_t cursor_control = 0x0678;           // CURSOR0_0_CURSOR_CONTROL
constexpr uint32_t cursor_surface_address = 0x0679;   // CURSOR0_0_CURSOR_SURFACE_ADDRESS
constexpr uint32_t cursor_surface_high = 0x067a;      // CURSOR0_0_CURSOR_SURFACE_ADDRESS_HIGH
constexpr uint32_t cursor_size = 0x067b;              // CURSOR0_0_CURSOR_SIZE
constexpr uint32_t cursor_position = 0x067c;          // CURSOR0_0_CURSOR_POSITION
constexpr uint32_t cursor_hot_spot = 0x067d;          // CURSOR0_0_CURSOR_HOT_SPOT
constexpr uint32_t cursor_dst_offset = 0x067f;        // CURSOR0_0_CURSOR_DST_OFFSET
constexpr uint32_t cursor_settings = 0x065b;          // HUBPREQ0_CURSOR_SETTINGS
constexpr uint32_t mall_config = 0x05f6;              // HUBP0_DCHUBP_MALL_CONFIG
constexpr uint32_t dpp_cursor_control = 0x0cf1;       // CNVC_CUR0_CURSOR0_CONTROL
constexpr uint32_t dpp_cursor_color0 = 0x0cf2;        // CNVC_CUR0_CURSOR0_COLOR0
constexpr uint32_t dpp_cursor_color1 = 0x0cf3;        // CNVC_CUR0_CURSOR0_COLOR1
constexpr uint32_t dpp_cursor_scale_bias = 0x0cf4;    // CNVC_CUR0_CURSOR0_FP_SCALE_BIAS
constexpr uint32_t cursor_lock_set = 0x03ab;          // CUR_VUPDATE_LOCK_SET0 (MPC)
constexpr uint32_t otg_status_position = 0x1b4a;      // OTG0_OTG_STATUS_POSITION
constexpr uint32_t otg_vstartup_param = 0x1b86;       // OTG0_OTG_VSTARTUP_PARAM
constexpr uint32_t global_timer_control = 0x0525;     // DCHUBBUB_GLOBAL_TIMER_CNTL

// dcn_3_2_0_sh_mask.h fields.
constexpr uint32_t cursor_enable = 0x00000001u;              // CURSOR_ENABLE
constexpr uint32_t cursor_magnify = 0x00000010u;             // CURSOR_2X_MAGNIFY
constexpr uint32_t cursor_mode_mask = 0x00000700u;           // CURSOR_MODE
constexpr uint32_t cursor_mode_shift = 8;
constexpr uint32_t cursor_pitch_mask = 0x00030000u;          // CURSOR_PITCH
constexpr uint32_t cursor_pitch_shift = 16;
constexpr uint32_t cursor_lpc_mask = 0x1f000000u;            // CURSOR_LINES_PER_CHUNK
constexpr uint32_t cursor_lpc_shift = 24;
constexpr uint32_t cursor_high_mask = 0x0000ffffu;           // CURSOR_SURFACE_ADDRESS_HIGH
constexpr uint32_t cursor_height_mask = 0x000001ffu;         // CURSOR_HEIGHT
constexpr uint32_t cursor_width_mask = 0x01ff0000u;          // CURSOR_WIDTH
constexpr uint32_t cursor_width_shift = 16;
constexpr uint32_t cursor_y_mask = 0x00003fffu;              // CURSOR_Y_POSITION
constexpr uint32_t cursor_x_mask = 0x00003fffu;              // CURSOR_X_POSITION (>> 16)
constexpr uint32_t cursor_hot_mask = 0x000000ffu;            // CURSOR_HOT_SPOT_X/Y
constexpr uint32_t cursor_dst_mask = 0x00001fffu;            // CURSOR_DST_X_OFFSET
constexpr uint32_t chunk_hdl_adjust_shift = 8;               // CURSOR0_CHUNK_HDL_ADJUST
constexpr uint32_t use_mall_for_cursor = 0x00000004u;        // USE_MALL_FOR_CURSOR
constexpr uint32_t cur0_enable = 0x00000001u;                // CUR0_ENABLE
constexpr uint32_t cur0_expansion = 0x00000002u;             // CUR0_EXPANSION_MODE
constexpr uint32_t cur0_rom_enable = 0x00000008u;            // CUR0_ROM_EN
constexpr uint32_t cur0_mode_mask = 0x00000070u;             // CUR0_MODE
constexpr uint32_t cur0_mode_shift = 4;
constexpr uint32_t cur0_color_mask = 0x00ffffffu;            // CUR0_COLOR0/1
constexpr uint32_t cur0_scale_mask = 0x0000ffffu;            // CUR0_FP_SCALE
constexpr uint32_t cur0_bias_mask = 0xffff0000u;             // CUR0_FP_BIAS
constexpr uint32_t lock_set = 0x00000001u;                   // CUR_VUPDATE_LOCK_SET
constexpr uint32_t vertical_count_mask = 0x00007fffu;        // OTG_VERT_COUNT
constexpr uint32_t vstartup_mask = 0x000003ffu;              // VSTARTUP_START
constexpr uint32_t timer_refdiv_mask = 0x0000000fu;          // DCHUBBUB_GLOBAL_TIMER_REFDIV
constexpr uint32_t timer_enable = 0x00001000u;               // DCHUBBUB_GLOBAL_TIMER_ENABLE

// hubp32_cursor_set_attributes: CURSOR0_DST_Y_OFFSET 0, CURSOR0_CHUNK_HDL_ADJUST 3.
constexpr uint32_t cursor_settings_value = 3u << chunk_hdl_adjust_shift;
// dcn10_set_cursor_sdr_white_level: 1.0 in the 5/10 float format at sdr_white_level <= 80.
constexpr uint32_t sdr_white_scale = 0x3c00;
// delay_cursor_until_vupdate: a cursor update takes at most about 70 us.
constexpr uint32_t cursor_update_microseconds = 70;
// hubbub2_get_dchub_ref_freq: the DCHUB reference must be about 50 MHz.
constexpr uint32_t dchub_minimum_khz = 40000;
constexpr uint32_t dchub_maximum_khz = 60000;
// atom_master_list_of_data_tables_v2_1 entry 27 is dce_info; dce_refclk_10khz
// follows the 4-byte header, display_caps and bootup_dispclk_10khz.
constexpr size_t dce_info_index = 27;
constexpr size_t dce_refclk_offset = 12;

constexpr uint32_t opaque = 0xff000000u;
constexpr uint32_t and_bit = 0x80000000u;
constexpr uint32_t white = 0x00ffffffu;

bool bit(ByteView plane, uint32_t stride, uint32_t row, uint32_t column) noexcept {
    uint8_t value = 0;
    plane.read_u8(static_cast<size_t>(row) * stride + column / 8, value);
    return (value & (0x80u >> (column % 8))) != 0;
}

uint32_t pixel(ByteView plane, uint32_t stride, uint32_t row, uint32_t column) noexcept {
    uint32_t value = 0;
    plane.read_u32(static_cast<size_t>(row) * stride + static_cast<size_t>(column) * 4, value);
    return value;
}

bool plane_fits(ByteView plane, uint32_t stride, uint32_t minimum_stride, uint32_t rows) noexcept {
    return stride >= minimum_stride && static_cast<uint64_t>(stride) * rows <= plane.size();
}

void fill(CursorImage& image, uint32_t value) noexcept {
    for (uint32_t& word : image.pixels) {
        word = value;
    }
}

template<size_t Count>
bool fits(uint32_t base, const uint32_t (&table)[Count], uint32_t aperture) noexcept {
    uint32_t offset = 0;

    for (const uint32_t reg : table) {
        if (!discovery_register_offset(base, reg, aperture, offset)) {
            return false;
        }
    }

    return true;
}

bool visible(int64_t source_x, int64_t source_y, uint32_t cursor_width, uint32_t cursor_height,
             uint32_t viewport_width, uint32_t viewport_height) noexcept {
    // hubp2_cursor_set_position / dpp1_set_cursor_position edge tests.
    return source_x < static_cast<int64_t>(viewport_width) && source_x + cursor_width > 0 &&
           source_y < static_cast<int64_t>(viewport_height) && source_y + cursor_height > 0;
}

} // namespace

CursorImageStatus build_cursor_image(const CursorSource& source, CursorImage& output) noexcept {
    if (source.width == 0 || source.height == 0 || source.width > dcn32_cursor_max_dimension ||
        source.height > dcn32_cursor_max_dimension) {
        return CursorImageStatus::unsupported_size;
    }

    const uint32_t mask_minimum = (source.width + 7) / 8;
    const uint32_t color_minimum = source.width * 4;
    const uint32_t width = source.width;
    const uint32_t height = source.height;

    switch (source.format) {
    case CursorSourceFormat::monochrome:
        if (!plane_fits(source.mask, source.mask_stride, mask_minimum, height * 2)) {
            return CursorImageStatus::truncated;
        }
        break;
    case CursorSourceFormat::masked_color:
        if (!plane_fits(source.mask, source.mask_stride, mask_minimum, height) ||
            !plane_fits(source.color, source.color_stride, color_minimum, height)) {
            return CursorImageStatus::truncated;
        }
        break;
    case CursorSourceFormat::premultiplied_alpha:
    case CursorSourceFormat::straight_alpha:
        if (!plane_fits(source.color, source.color_stride, color_minimum, height)) {
            return CursorImageStatus::truncated;
        }
        break;
    default:
        return CursorImageStatus::invalid_argument;
    }

    // Screen-dependent pixels decide between exact ARGB and the 1-bit-AND mode.
    bool screen_dependent = false;

    for (uint32_t row = 0; row < height && !screen_dependent; ++row) {
        for (uint32_t column = 0; column < width && !screen_dependent; ++column) {
            if (source.format == CursorSourceFormat::monochrome) {
                screen_dependent = bit(source.mask, source.mask_stride, row, column) &&
                                   bit(source.mask, source.mask_stride, height + row, column);
            } else if (source.format == CursorSourceFormat::masked_color) {
                screen_dependent = bit(source.mask, source.mask_stride, row, column) &&
                                   (pixel(source.color, source.color_stride, row, column) & white) != 0;
            }
        }
    }

    CursorImage& image = output;
    image.width = width;
    image.height = height;

    if (source.format == CursorSourceFormat::premultiplied_alpha ||
        source.format == CursorSourceFormat::straight_alpha) {
        image.mode = source.format == CursorSourceFormat::premultiplied_alpha ? CursorMode::premultiplied_alpha :
                                                                                 CursorMode::straight_alpha;
        fill(image, 0);

        for (uint32_t row = 0; row < height; ++row) {
            for (uint32_t column = 0; column < width; ++column) {
                image.pixels[row * dcn32_cursor_pitch_pixels + column] =
                    pixel(source.color, source.color_stride, row, column);
            }
        }

        return CursorImageStatus::built;
    }

    image.mode = screen_dependent ? CursorMode::color_1bit_and : CursorMode::premultiplied_alpha;
    // Transparent padding: alpha 0, or AND 1 with color 0 (screen unchanged).
    fill(image, screen_dependent ? and_bit : 0u);

    for (uint32_t row = 0; row < height; ++row) {
        for (uint32_t column = 0; column < width; ++column) {
            const bool and_mask = bit(source.mask, source.mask_stride, row, column);
            const uint32_t color = source.format == CursorSourceFormat::monochrome ?
                (bit(source.mask, source.mask_stride, height + row, column) ? white : 0u) :
                (pixel(source.color, source.color_stride, row, column) & white);
            uint32_t word = 0;

            if (screen_dependent) {
                word = (and_mask ? and_bit : 0u) | color;
            } else {
                // AND 1 pixels have color 0 here: transparent; AND 0 is opaque color.
                word = and_mask ? 0u : (opaque | color);
            }

            image.pixels[row * dcn32_cursor_pitch_pixels + column] = word;
        }
    }

    return CursorImageStatus::built;
}

hardware::DiscoveryStatus validate_dcn32_cursor_discovery(ByteView binary, uint32_t aperture,
                                                          const AdoptedScanout& scanout,
                                                          Dcn32CursorRegisters& output) noexcept {
    output = {};

    if (aperture == 0 || (aperture & 3u) != 0 || scanout.pipe >= preos_pipe_count ||
        scanout.opp >= preos_pipe_count || scanout.otg >= preos_pipe_count) {
        return hardware::DiscoveryStatus::invalid_argument;
    }

    hardware::DiscoveredIp dmu;
    const auto status = hardware::discover_ip(binary, {hardware::discovery_dmu_hardware_id, 0, 3, 2, 0, 4}, dmu);

    if (status != hardware::DiscoveryStatus::validated) {
        return status;
    }

    const uint32_t dcn = dmu.segment(segment_dcn);
    const uint32_t mpc = dmu.segment(segment_mpc);
    const uint32_t hubp = scanout.pipe * hubp_stride;
    const uint32_t dpp = scanout.pipe * dpp_stride;
    const uint32_t otg = scanout.otg * otg_stride;
    const uint32_t dcn_registers[]{
        cursor_control + hubp, cursor_surface_address + hubp, cursor_surface_high + hubp,
        cursor_size + hubp, cursor_position + hubp, cursor_hot_spot + hubp, cursor_dst_offset + hubp,
        cursor_settings + hubp, mall_config + hubp, dpp_cursor_control + dpp, dpp_cursor_color0 + dpp,
        dpp_cursor_color1 + dpp, dpp_cursor_scale_bias + dpp, otg_status_position + otg,
        otg_vstartup_param + otg, global_timer_control};
    const uint32_t mpc_registers[]{cursor_lock_set + scanout.opp * cursor_lock_stride};

    if (dcn == 0 || mpc == 0 || !fits(dcn, dcn_registers, aperture) || !fits(mpc, mpc_registers, aperture)) {
        return hardware::DiscoveryStatus::invalid_argument;
    }

    output.dcn_ = dcn;
    output.mpc_ = mpc;
    output.aperture_bytes_ = aperture;
    output.pipe_ = scanout.pipe;
    output.opp_ = scanout.opp;
    output.otg_ = scanout.otg;
    return hardware::DiscoveryStatus::validated;
}

bool read_dce_refclk_khz(ByteView rom, uint32_t& khz) noexcept {
    AtomTable table{};
    uint16_t refclk_10khz = 0;

    if (atom_data_table(rom, dce_info_index, table) != AtomTableStatus::success || table.format != 4 ||
        table.bytes < dce_refclk_offset + sizeof(uint16_t) ||
        !rom.read_u16(table.base + dce_refclk_offset, refclk_10khz) || refclk_10khz == 0) {
        return false;
    }

    khz = static_cast<uint32_t>(refclk_10khz) * 10u;
    return true;
}

bool read_dchub_refclk_khz(const hardware::RegisterIo& io, const Dcn32CursorRegisters& registers,
                           uint32_t dccg_ref_khz, uint32_t& khz) noexcept {
    uint32_t value = 0;

    if (!registers.valid() || io.aperture_bytes != registers.aperture_bytes() || dccg_ref_khz == 0 ||
        !hardware::read_ip_register(io, registers.dcn_, global_timer_control, value) || value == UINT32_MAX ||
        (value & timer_enable) == 0) {
        return false;
    }

    const uint32_t result = (value & timer_refdiv_mask) == 2 ? dccg_ref_khz / 2 : dccg_ref_khz;

    if (result < dchub_minimum_khz || result > dchub_maximum_khz) {
        return false;
    }

    khz = result;
    return true;
}

bool place_cursor(const CursorRequest& request, const CursorStream& stream, uint32_t width,
                  uint32_t height, CursorPlacement& output) noexcept {
    output = {};

    if (stream.width == 0 || stream.height == 0 || stream.pixel_clock_khz == 0 || stream.refclk_khz == 0 ||
        width == 0 || height == 0 || width > dcn32_cursor_pitch_pixels || height > dcn32_cursor_max_dimension) {
        return false;
    }

    constexpr int64_t maximum = dcn32_cursor_max_dimension;
    int64_t x = request.x;
    int64_t y = request.y;

    // amdgpu_dm_plane_get_cursor_position: wholly beyond the top/left edge, or a
    // disabled request, leaves the zeroed (disabled) position.
    if (!request.enable || x <= -maximum || y <= -maximum) {
        return true;
    }

    int64_t hot_x = 0;
    int64_t hot_y = 0;

    if (x < 0) {
        hot_x = -x < maximum - 1 ? -x : maximum - 1;
        x = 0;
    }

    if (y < 0) {
        hot_y = -y < maximum - 1 ? -y : maximum - 1;
        y = 0;
    }

    // dcn10_set_cursor_position: plane dst_rect == src_rect at (0, 0), so the
    // stream-to-plane translation is the identity and x, y stay non-negative.
    // hubp2_cursor_set_position with recout and viewport at (0, 0).
    const int64_t source_x = x - hot_x;
    const int64_t source_y = y - hot_y;
    const uint64_t clamped = source_x >= 0 ? static_cast<uint64_t>(source_x) : 0u;
    const uint64_t destination = clamped * stream.refclk_khz / stream.pixel_clock_khz;

    output.enable = visible(source_x, source_y, width, height, stream.width, stream.height);
    output.x = static_cast<uint32_t>(x) & cursor_x_mask;
    output.y = static_cast<uint32_t>(y) & cursor_y_mask;
    output.hot_x = static_cast<uint32_t>(hot_x) & cursor_hot_mask;
    output.hot_y = static_cast<uint32_t>(hot_y) & cursor_hot_mask;
    output.dst_x_offset = static_cast<uint32_t>(destination & cursor_dst_mask);
    return true;
}

uint32_t cursor_lines_per_chunk(uint32_t width, CursorMode mode) noexcept {
    // CURSOR_LINE_PER_CHUNK_1/2/4/8/16 are field values 0..4.
    constexpr uint32_t lines_16 = 4;

    if (mode == CursorMode::mono) {
        return lines_16;
    }

    if (width >= 1 && width <= 32) {
        return lines_16;
    }

    if (width >= 33 && width <= 64) {
        return 3;
    }

    if (width >= 65 && width <= 128) {
        return 2;
    }

    if (width >= 129 && width <= 256) {
        return 1;
    }

    return lines_16;
}

bool cursor_pitch_field(uint32_t pitch_pixels, uint32_t& field) noexcept {
    switch (pitch_pixels) {
    case 64: field = 0; return true;
    case 128: field = 1; return true;
    case 256: field = 2; return true;
    default: return false;
    }
}

bool cursor_uses_mall(uint32_t width, uint32_t height, CursorMode mode) noexcept {
    uint64_t bytes = static_cast<uint64_t>((width + 63) / 64 * 64) * height;

    if (mode == CursorMode::mono) {
        bytes /= 2;
    } else {
        bytes *= 4;
    }

    return bytes > 16384;
}

bool cursor_vupdate_window(const Timing& timing, uint32_t vstartup_start, VupdateWindow& output) noexcept {
    if (timing.v_total == 0 || timing.height == 0 || timing.height >= timing.v_total) {
        return false;
    }

    // apply_front_porch_workaround: at least one progressive front-porch line.
    const int64_t front_porch = timing.v_front_porch < 1 ? 1 : timing.v_front_porch;
    const int64_t total = timing.v_total;
    const int64_t vesa_sync_start = static_cast<int64_t>(timing.height) + front_porch;
    const int64_t asic_blank_end = total - vesa_sync_start;
    const int64_t vupdate = asic_blank_end - static_cast<int64_t>(vstartup_start) + 1;
    int64_t start = 0;

    if (vupdate >= 0) {
        start = vupdate - (vupdate / total) * total;
    } else {
        start = vupdate + ((-vupdate / total) + 1) * total - 1;
    }

    output.start_line = static_cast<uint32_t>(start);
    output.end_line = static_cast<uint32_t>((start + 2) % total);
    return true;
}

uint32_t cursor_lock_delay_microseconds(const Timing& timing, const VupdateWindow& window,
                                        uint32_t vpos) noexcept {
    if (timing.pixel_clock_khz == 0 || timing.v_total == 0) {
        return 0;
    }

    // Wraparound-free arithmetic exactly as delay_cursor_until_vupdate.
    const uint64_t total = timing.v_total;
    const uint64_t start = window.start_line + total;
    uint64_t end = window.end_line + total;
    const uint64_t position = static_cast<uint64_t>(vpos) + total;
    uint64_t lines = 0;

    if (position <= start) {
        lines = start - position;
    } else if (position > end) {
        return 0;
    }

    // pix_clk_100hz = kHz * 10.
    const uint64_t per_line = static_cast<uint64_t>(timing.h_total) * 10000u /
                              (static_cast<uint64_t>(timing.pixel_clock_khz) * 10u);
    const uint64_t to_vupdate = lines * per_line;

    if (to_vupdate > cursor_update_microseconds) {
        return 0;
    }

    if (end < start) {
        end += total;
    }

    const uint64_t stall = to_vupdate + (end - start + 1) * per_line;
    return stall > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(stall);
}

CursorStatus Dcn32Cursor::initialize(const Dcn32CursorRegisters& registers, const hardware::RegisterIo& io,
                                     const Timing& timing, const CursorStream& stream) noexcept {
    *this = Dcn32Cursor{};

    if (!registers.valid() || !hardware::register_io_valid(io) || io.aperture_bytes != registers.aperture_bytes() ||
        stream.width == 0 || stream.height == 0 || stream.pixel_clock_khz == 0 || stream.refclk_khz == 0 ||
        stream.width != timing.width || stream.height != timing.height ||
        timing.pixel_clock_khz != stream.pixel_clock_khz) {
        report_.status = CursorStatus::invalid_argument;
        return report_.status;
    }

    registers_ = registers;
    io_ = io;
    timing_ = timing;
    stream_ = stream;
    report_.status = CursorStatus::ready;
    return report_.status;
}

bool Dcn32Cursor::read(uint32_t base, uint32_t reg, uint32_t& value) noexcept {
    return hardware::read_ip_register(io_, base, reg, value);
}

bool Dcn32Cursor::write(uint32_t base, uint32_t reg, uint32_t value) noexcept {
    return hardware::write_ip_register(io_, base, reg, value);
}

// REG_UPDATE: read, replace the field bits, write back.
bool Dcn32Cursor::update(uint32_t base, uint32_t reg, uint32_t mask, uint32_t value) noexcept {
    uint32_t current = 0;
    return read(base, reg, current) && write(base, reg, (current & ~mask) | (value & mask));
}

CursorStatus Dcn32Cursor::fail() noexcept {
    report_.status = CursorStatus::mmio_failure;
    return report_.status;
}

// dcn10_cursor_lock -> mpc1_cursor_lock, with delay_cursor_until_vupdate before
// taking the lock. The VUPDATE window comes from OTG_VSTARTUP_PARAM and the
// position from OTG_STATUS_POSITION (dcn10_get_position), both read only.
bool Dcn32Cursor::lock(bool locked) noexcept {
    const uint32_t otg = registers_.otg_ * otg_stride;

    if (locked) {
        uint32_t vstartup = 0;
        uint32_t position = 0;
        VupdateWindow window{};

        if (!read(registers_.dcn_, otg_vstartup_param + otg, vstartup) ||
            !read(registers_.dcn_, otg_status_position + otg, position)) {
            return false;
        }

        if (cursor_vupdate_window(timing_, vstartup & vstartup_mask, window)) {
            const uint32_t stall = cursor_lock_delay_microseconds(timing_, window, position & vertical_count_mask);

            if (stall != 0) {
                io_.delay_microseconds(io_.context, stall);
                ++report_.lock_delays;
            }
        }
    }

    // REG_SET: the whole register is written with only the lock bit.
    return write(registers_.mpc_, cursor_lock_set + registers_.opp_ * cursor_lock_stride,
                 locked ? lock_set : 0u);
}

// hubp32_cursor_set_attributes for the fixed 64 x 64, pitch 64 image.
bool Dcn32Cursor::write_hubp_attributes() noexcept {
    const uint32_t hubp = registers_.pipe_ * hubp_stride;
    const CursorMode mode = report_.mode;
    const uint32_t width = dcn32_cursor_pitch_pixels;
    const uint32_t height = dcn32_cursor_max_dimension;
    uint32_t pitch = 0;
    cursor_pitch_field(dcn32_cursor_pitch_pixels, pitch);
    const uint32_t control = (static_cast<uint32_t>(mode) << cursor_mode_shift) |
                             (pitch << cursor_pitch_shift) |
                             (cursor_lines_per_chunk(width, mode) << cursor_lpc_shift);

    return update(registers_.dcn_, cursor_surface_high + hubp, cursor_high_mask,
                  static_cast<uint32_t>(report_.mc_address >> 32)) &&
           update(registers_.dcn_, cursor_surface_address + hubp, UINT32_MAX,
                  static_cast<uint32_t>(report_.mc_address)) &&
           update(registers_.dcn_, cursor_size + hubp, cursor_width_mask | cursor_height_mask,
                  (width << cursor_width_shift) | height) &&
           update(registers_.dcn_, cursor_control + hubp,
                  cursor_mode_mask | cursor_magnify | cursor_pitch_mask | cursor_lpc_mask, control) &&
           write(registers_.dcn_, cursor_settings + hubp, cursor_settings_value) &&
           update(registers_.dcn_, mall_config + hubp, use_mall_for_cursor,
                  cursor_uses_mall(width, height, mode) ? use_mall_for_cursor : 0u);
}

// dpp3_set_cursor_attributes (ENABLE_CURSOR_DEGAMMA clear: no ROM degamma),
// then dcn10_set_cursor_sdr_white_level through
// dpp1_cnv_set_optional_cursor_attributes (bias, then scale).
bool Dcn32Cursor::write_dpp_attributes() noexcept {
    const uint32_t dpp = registers_.pipe_ * dpp_stride;
    const CursorMode mode = report_.mode;

    if (!update(registers_.dcn_, dpp_cursor_control + dpp, cur0_mode_mask | cur0_expansion | cur0_rom_enable,
                static_cast<uint32_t>(mode) << cur0_mode_shift)) {
        return false;
    }

    if (mode == CursorMode::mono &&
        (!update(registers_.dcn_, dpp_cursor_color0 + dpp, cur0_color_mask, 0) ||
         !update(registers_.dcn_, dpp_cursor_color1 + dpp, cur0_color_mask, UINT32_MAX))) {
        return false;
    }

    return update(registers_.dcn_, dpp_cursor_scale_bias + dpp, cur0_bias_mask, 0) &&
           update(registers_.dcn_, dpp_cursor_scale_bias + dpp, cur0_scale_mask, sdr_white_scale);
}

// hubp2_cursor_set_position then dpp1_set_cursor_position.
bool Dcn32Cursor::write_position(const CursorPlacement& placement) noexcept {
    const uint32_t hubp = registers_.pipe_ * hubp_stride;
    const uint32_t dpp = registers_.pipe_ * dpp_stride;

    if (placement.enable) {
        uint32_t address = 0;

        if (!read(registers_.dcn_, cursor_surface_address + hubp, address)) {
            return false;
        }

        // The surface address was lost (power gating or reset): attributes first.
        if (address == 0) {
            if (!write_hubp_attributes()) {
                return false;
            }

            ++report_.reprogrammed_attributes;
        }
    }

    return update(registers_.dcn_, cursor_control + hubp, cursor_enable, placement.enable ? cursor_enable : 0u) &&
           write(registers_.dcn_, cursor_position + hubp, (placement.x << 16) | placement.y) &&
           write(registers_.dcn_, cursor_hot_spot + hubp, (placement.hot_x << 16) | placement.hot_y) &&
           write(registers_.dcn_, cursor_dst_offset + hubp, placement.dst_x_offset) &&
           update(registers_.dcn_, dpp_cursor_control + dpp, cur0_enable, placement.enable ? cur0_enable : 0u);
}

CursorStatus Dcn32Cursor::set_attributes(const CursorAttributes& attributes) noexcept {
    if (report_.status != CursorStatus::ready) {
        return report_.status == CursorStatus::mmio_failure ? report_.status : CursorStatus::uninitialized;
    }

    if (attributes.mc_address == 0 || (attributes.mc_address % dcn32_cursor_alignment) != 0 ||
        (attributes.mc_address >> 48) != 0 || static_cast<uint32_t>(attributes.mode) > 3) {
        return CursorStatus::invalid_argument;
    }

    report_.mc_address = attributes.mc_address;
    report_.mode = attributes.mode;

    // program_cursor_attributes: lock, set_cursor_attribute, sdr white level, unlock.
    if (!lock(true) || !write_hubp_attributes() || !write_dpp_attributes() || !lock(false)) {
        return fail();
    }

    report_.attributes_programmed = true;
    return CursorStatus::ready;
}

CursorStatus Dcn32Cursor::set_position(const CursorRequest& request) noexcept {
    if (report_.status != CursorStatus::ready) {
        return report_.status == CursorStatus::mmio_failure ? report_.status : CursorStatus::uninitialized;
    }

    request_ = request;
    CursorPlacement placement{};

    if (!place_cursor(request, stream_, dcn32_cursor_pitch_pixels, dcn32_cursor_max_dimension, placement)) {
        return CursorStatus::invalid_argument;
    }

    // hubp2_cursor_set_position guard: nothing before valid attributes exist.
    if (!report_.attributes_programmed) {
        return CursorStatus::ready;
    }

    if (!lock(true) || !write_position(placement) || !lock(false)) {
        return fail();
    }

    report_.placement = placement;
    report_.enabled = placement.enable;
    return CursorStatus::ready;
}

CursorStatus Dcn32Cursor::hide() noexcept {
    // A disabled Linux position is zeroed (amdgpu_dm_update_cursor).
    return set_position({false, 0, 0});
}

void Dcn32Cursor::forget() noexcept {
    report_.attributes_programmed = false;
    report_.enabled = false;
    report_.placement = {};
}

} // namespace rx7900::display

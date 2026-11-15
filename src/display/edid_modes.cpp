/*
Module: rx7900.display.edid_modes
Purpose: Decode EDID timing codes, VIC lists and DisplayID timings and infer range-limit modes into the shared mode catalog.
Created: 2026-10-24
Architecture: Own implementation of the standards listed in edid_modes.hpp;
all reads go through the checked ByteView, all arithmetic is integer and
bounded. The CEA/HDMI VIC timings come from hdmi_stream.
*/
#include "edid_modes.hpp"

#include "hdmi_stream.hpp"

namespace rx7900::display {
namespace {

constexpr uint32_t block_bytes = 128;
constexpr uint32_t maximum_blocks = 8;
constexpr uint32_t descriptor_bytes = 18;
constexpr uint32_t base_descriptor_offset = 54;
constexpr uint32_t base_descriptor_count = 4;
constexpr uint32_t established_offset = 35;
constexpr uint32_t standard_offset = 38;
constexpr uint32_t standard_count = 8;
constexpr uint8_t tag_range_limits = 0xfd;
constexpr uint8_t tag_standard_timings = 0xfa;
constexpr uint8_t tag_cvt_codes = 0xf8;
constexpr uint8_t tag_established_iii = 0xf7;
constexpr uint8_t range_cvt_support = 0x04;
constexpr uint8_t displayid_extension_tag = 0x70;
constexpr uint8_t displayid_type_i_timing = 0x03;
constexpr uint8_t displayid_type_vii_timing = 0x22;
constexpr uint32_t displayid_timing_bytes = 20;
constexpr uint32_t cvt_cell = 8;

struct DmtMode final {
    uint8_t id;
    Timing timing;
};

// VESA DMT 1.0 rev 13 progressive timings by DMT ID, ordered by size as in
// the pinned drm_edid.c table (the interlaced 0x0f is omitted).
constexpr DmtMode dmt_modes[] = {
    {0x01, {31500, 640, 832, 32, 64, 350, 445, 32, 3, positive_hsync}},
    {0x02, {31500, 640, 832, 32, 64, 400, 445, 1, 3, positive_vsync}},
    {0x03, {35500, 720, 936, 36, 72, 400, 446, 1, 3, positive_vsync}},
    {0x04, {25175, 640, 800, 16, 96, 480, 525, 10, 2, 0}},
    {0x05, {31500, 640, 832, 24, 40, 480, 520, 9, 3, 0}},
    {0x06, {31500, 640, 840, 16, 64, 480, 500, 1, 3, 0}},
    {0x07, {36000, 640, 832, 56, 56, 480, 509, 1, 3, 0}},
    {0x08, {36000, 800, 1024, 24, 72, 600, 625, 1, 2, positive_hsync | positive_vsync}},
    {0x09, {40000, 800, 1056, 40, 128, 600, 628, 1, 4, positive_hsync | positive_vsync}},
    {0x0a, {50000, 800, 1040, 56, 120, 600, 666, 37, 6, positive_hsync | positive_vsync}},
    {0x0b, {49500, 800, 1056, 16, 80, 600, 625, 1, 3, positive_hsync | positive_vsync}},
    {0x0c, {56250, 800, 1048, 32, 64, 600, 631, 1, 3, positive_hsync | positive_vsync}},
    {0x0d, {73250, 800, 960, 48, 32, 600, 636, 3, 4, positive_hsync}},
    {0x0e, {33750, 848, 1088, 16, 112, 480, 517, 6, 8, positive_hsync | positive_vsync}},
    {0x10, {65000, 1024, 1344, 24, 136, 768, 806, 3, 6, 0}},
    {0x11, {75000, 1024, 1328, 24, 136, 768, 806, 3, 6, 0}},
    {0x12, {78750, 1024, 1312, 16, 96, 768, 800, 1, 3, positive_hsync | positive_vsync}},
    {0x13, {94500, 1024, 1376, 48, 96, 768, 808, 1, 3, positive_hsync | positive_vsync}},
    {0x14, {115500, 1024, 1184, 48, 32, 768, 813, 3, 4, positive_hsync}},
    {0x15, {108000, 1152, 1600, 64, 128, 864, 900, 1, 3, positive_hsync | positive_vsync}},
    {0x55, {74250, 1280, 1650, 110, 40, 720, 750, 5, 5, positive_hsync | positive_vsync}},
    {0x16, {68250, 1280, 1440, 48, 32, 768, 790, 3, 7, positive_hsync}},
    {0x17, {79500, 1280, 1664, 64, 128, 768, 798, 3, 7, positive_vsync}},
    {0x18, {102250, 1280, 1696, 80, 128, 768, 805, 3, 7, positive_vsync}},
    {0x19, {117500, 1280, 1712, 80, 136, 768, 809, 3, 7, positive_vsync}},
    {0x1a, {140250, 1280, 1440, 48, 32, 768, 813, 3, 7, positive_hsync}},
    {0x1b, {71000, 1280, 1440, 48, 32, 800, 823, 3, 6, positive_hsync}},
    {0x1c, {83500, 1280, 1680, 72, 128, 800, 831, 3, 6, positive_vsync}},
    {0x1d, {106500, 1280, 1696, 80, 128, 800, 838, 3, 6, positive_vsync}},
    {0x1e, {122500, 1280, 1712, 80, 136, 800, 843, 3, 6, positive_vsync}},
    {0x1f, {146250, 1280, 1440, 48, 32, 800, 847, 3, 6, positive_hsync}},
    {0x20, {108000, 1280, 1800, 96, 112, 960, 1000, 1, 3, positive_hsync | positive_vsync}},
    {0x21, {148500, 1280, 1728, 64, 160, 960, 1011, 1, 3, positive_hsync | positive_vsync}},
    {0x22, {175500, 1280, 1440, 48, 32, 960, 1017, 3, 4, positive_hsync}},
    {0x23, {108000, 1280, 1688, 48, 112, 1024, 1066, 1, 3, positive_hsync | positive_vsync}},
    {0x24, {135000, 1280, 1688, 16, 144, 1024, 1066, 1, 3, positive_hsync | positive_vsync}},
    {0x25, {157500, 1280, 1728, 64, 160, 1024, 1072, 1, 3, positive_hsync | positive_vsync}},
    {0x26, {187250, 1280, 1440, 48, 32, 1024, 1084, 3, 7, positive_hsync}},
    {0x27, {85500, 1360, 1792, 64, 112, 768, 795, 3, 6, positive_hsync | positive_vsync}},
    {0x28, {148250, 1360, 1520, 48, 32, 768, 813, 3, 5, positive_hsync}},
    {0x51, {85500, 1366, 1792, 70, 143, 768, 798, 3, 3, positive_hsync | positive_vsync}},
    {0x56, {72000, 1366, 1500, 14, 56, 768, 800, 1, 3, positive_hsync | positive_vsync}},
    {0x29, {101000, 1400, 1560, 48, 32, 1050, 1080, 3, 4, positive_hsync}},
    {0x2a, {121750, 1400, 1864, 88, 144, 1050, 1089, 3, 4, positive_vsync}},
    {0x2b, {156000, 1400, 1896, 104, 144, 1050, 1099, 3, 4, positive_vsync}},
    {0x2c, {179500, 1400, 1912, 104, 152, 1050, 1105, 3, 4, positive_vsync}},
    {0x2d, {208000, 1400, 1560, 48, 32, 1050, 1112, 3, 4, positive_hsync}},
    {0x2e, {88750, 1440, 1600, 48, 32, 900, 926, 3, 6, positive_hsync}},
    {0x2f, {106500, 1440, 1904, 80, 152, 900, 934, 3, 6, positive_vsync}},
    {0x30, {136750, 1440, 1936, 96, 152, 900, 942, 3, 6, positive_vsync}},
    {0x31, {157000, 1440, 1952, 104, 152, 900, 948, 3, 6, positive_vsync}},
    {0x32, {182750, 1440, 1600, 48, 32, 900, 953, 3, 6, positive_hsync}},
    {0x53, {108000, 1600, 1800, 24, 80, 900, 1000, 1, 3, positive_hsync | positive_vsync}},
    {0x33, {162000, 1600, 2160, 64, 192, 1200, 1250, 1, 3, positive_hsync | positive_vsync}},
    {0x34, {175500, 1600, 2160, 64, 192, 1200, 1250, 1, 3, positive_hsync | positive_vsync}},
    {0x35, {189000, 1600, 2160, 64, 192, 1200, 1250, 1, 3, positive_hsync | positive_vsync}},
    {0x36, {202500, 1600, 2160, 64, 192, 1200, 1250, 1, 3, positive_hsync | positive_vsync}},
    {0x37, {229500, 1600, 2160, 64, 192, 1200, 1250, 1, 3, positive_hsync | positive_vsync}},
    {0x38, {268250, 1600, 1760, 48, 32, 1200, 1271, 3, 4, positive_hsync}},
    {0x39, {119000, 1680, 1840, 48, 32, 1050, 1080, 3, 6, positive_hsync}},
    {0x3a, {146250, 1680, 2240, 104, 176, 1050, 1089, 3, 6, positive_vsync}},
    {0x3b, {187000, 1680, 2272, 120, 176, 1050, 1099, 3, 6, positive_vsync}},
    {0x3c, {214750, 1680, 2288, 128, 176, 1050, 1105, 3, 6, positive_vsync}},
    {0x3d, {245500, 1680, 1840, 48, 32, 1050, 1112, 3, 6, positive_hsync}},
    {0x3e, {204750, 1792, 2448, 128, 200, 1344, 1394, 1, 3, positive_vsync}},
    {0x3f, {261000, 1792, 2456, 96, 216, 1344, 1417, 1, 3, positive_vsync}},
    {0x40, {333250, 1792, 1952, 48, 32, 1344, 1423, 3, 4, positive_hsync}},
    {0x41, {218250, 1856, 2528, 96, 224, 1392, 1439, 1, 3, positive_vsync}},
    {0x42, {288000, 1856, 2560, 128, 224, 1392, 1500, 1, 3, positive_vsync}},
    {0x43, {356500, 1856, 2016, 48, 32, 1392, 1474, 3, 4, positive_hsync}},
    {0x52, {148500, 1920, 2200, 88, 44, 1080, 1125, 4, 5, positive_hsync | positive_vsync}},
    {0x44, {154000, 1920, 2080, 48, 32, 1200, 1235, 3, 6, positive_hsync}},
    {0x45, {193250, 1920, 2592, 136, 200, 1200, 1245, 3, 6, positive_vsync}},
    {0x46, {245250, 1920, 2608, 136, 208, 1200, 1255, 3, 6, positive_vsync}},
    {0x47, {281250, 1920, 2624, 144, 208, 1200, 1262, 3, 6, positive_vsync}},
    {0x48, {317000, 1920, 2080, 48, 32, 1200, 1271, 3, 6, positive_hsync}},
    {0x49, {234000, 1920, 2600, 128, 208, 1440, 1500, 1, 3, positive_vsync}},
    {0x4a, {297000, 1920, 2640, 144, 224, 1440, 1500, 1, 3, positive_vsync}},
    {0x4b, {380500, 1920, 2080, 48, 32, 1440, 1525, 3, 4, positive_hsync}},
    {0x54, {162000, 2048, 2250, 26, 80, 1152, 1200, 1, 3, positive_hsync | positive_vsync}},
    {0x4c, {268500, 2560, 2720, 48, 32, 1600, 1646, 3, 6, positive_hsync}},
    {0x4d, {348500, 2560, 3504, 192, 280, 1600, 1658, 3, 6, positive_vsync}},
    {0x4e, {443250, 2560, 3536, 208, 280, 1600, 1672, 3, 6, positive_vsync}},
    {0x4f, {505250, 2560, 3536, 208, 280, 1600, 1682, 3, 6, positive_vsync}},
    {0x50, {552750, 2560, 2720, 48, 32, 1600, 1694, 3, 6, positive_hsync}},
    {0x57, {556744, 4096, 4176, 8, 32, 2160, 2222, 48, 8, positive_hsync}},
    {0x58, {556188, 4096, 4176, 8, 32, 2160, 2222, 48, 8, positive_hsync}},
};

// Established timings I and II (E-EDID 1.4 Table 3.18) in bit order, byte 35
// bit 7 first. width 0: not produced (interlaced 1024x768@87 and the Apple
// 1152x870@75, whose timing no pinned source defines). dmt false: the IBM/
// Apple timing given here.
struct EstablishedMode final {
    uint16_t width;
    uint16_t height;
    uint8_t refresh;
    bool dmt;
    Timing timing;
};

constexpr EstablishedMode established_i_ii[17] = {
    {720, 400, 70, false, {28320, 720, 900, 18, 108, 400, 449, 12, 2, positive_vsync}},
    {720, 400, 88, false, {35500, 720, 900, 18, 108, 400, 449, 21, 2, 0}},
    {640, 480, 60, true, {}},
    {640, 480, 67, false, {30240, 640, 864, 64, 64, 480, 525, 3, 3, 0}},
    {640, 480, 72, true, {}},
    {640, 480, 75, true, {}},
    {800, 600, 56, true, {}},
    {800, 600, 60, true, {}},
    {800, 600, 72, true, {}},
    {800, 600, 75, true, {}},
    {832, 624, 75, false, {57284, 832, 1152, 32, 64, 624, 667, 1, 3, 0}},
    {0, 0, 0, false, {}},
    {1024, 768, 60, true, {}},
    {1024, 768, 70, true, {}},
    {1024, 768, 75, true, {}},
    {1280, 1024, 75, true, {}},
    {0, 0, 0, false, {}},
};

// Established timings III (E-EDID 1.4 Table 3.29), byte 6 bit 7 first; all DMT.
struct MiniMode final {
    uint16_t width;
    uint16_t height;
    uint8_t refresh;
    bool reduced;
};

constexpr MiniMode established_iii[44] = {
    {640, 350, 85, false},   {640, 400, 85, false},   {720, 400, 85, false},   {640, 480, 85, false},
    {848, 480, 60, false},   {800, 600, 85, false},   {1024, 768, 85, false},  {1152, 864, 75, false},
    {1280, 768, 60, true},   {1280, 768, 60, false},  {1280, 768, 75, false},  {1280, 768, 85, false},
    {1280, 960, 60, false},  {1280, 960, 85, false},  {1280, 1024, 60, false}, {1280, 1024, 85, false},
    {1360, 768, 60, false},  {1440, 900, 60, true},   {1440, 900, 60, false},  {1440, 900, 75, false},
    {1440, 900, 85, false},  {1400, 1050, 60, true},  {1400, 1050, 60, false}, {1400, 1050, 75, false},
    {1400, 1050, 85, false}, {1680, 1050, 60, true},  {1680, 1050, 60, false}, {1680, 1050, 75, false},
    {1680, 1050, 85, false}, {1600, 1200, 60, false}, {1600, 1200, 65, false}, {1600, 1200, 70, false},
    {1600, 1200, 75, false}, {1600, 1200, 85, false}, {1792, 1344, 60, false}, {1792, 1344, 75, false},
    {1856, 1392, 60, false}, {1856, 1392, 75, false}, {1920, 1200, 60, true},  {1920, 1200, 60, false},
    {1920, 1200, 75, false}, {1920, 1200, 85, false}, {1920, 1440, 60, false}, {1920, 1440, 75, false},
};

// Sizes tried for CVT range inference (60 Hz and the CVT preferred rate).
constexpr MiniMode cvt_candidates[] = {
    {1280, 720, 60, false},  {1280, 800, 60, false},  {1280, 1024, 60, false}, {1600, 900, 60, false},
    {1600, 1200, 60, false}, {1680, 1050, 60, false}, {1920, 1080, 60, false}, {1920, 1200, 60, false},
    {2560, 1440, 60, false}, {2560, 1600, 60, false}, {3840, 2160, 60, false},
};

uint8_t at(ByteView view, size_t offset) noexcept {
    uint8_t value = 0;
    view.read_u8(offset, value);
    return value;
}

uint32_t le16(ByteView view, size_t offset) noexcept {
    return static_cast<uint32_t>(at(view, offset)) | (static_cast<uint32_t>(at(view, offset + 1)) << 8);
}

// Linux mode_is_rb: the CVT reduced blanking v1 shape.
bool reduced_blanking_shape(const Timing& t) noexcept {
    return t.h_total - t.width == 160 && t.h_front_porch + t.h_sync_width == 80 && t.h_sync_width == 32 &&
           t.v_front_porch == 3;
}

// drm_mode_hsync: kHz, rounded.
uint32_t hsync_khz(const Timing& t) noexcept {
    return t.h_total == 0 ? 0
                          : static_cast<uint32_t>((static_cast<uint64_t>(t.pixel_clock_khz) * 1000u / t.h_total +
                                                   500u) / 1000u);
}

// CVT 1.2 Table 3-2: vertical sync width by aspect ratio of the request.
uint32_t cvt_vsync_lines(uint32_t width, uint32_t height) noexcept {
    const uint64_t w = width;
    const uint64_t h = height;
    if (w * 3 == h * 4) {
        return 4;
    }
    if (w * 9 == h * 16) {
        return 5;
    }
    if (w * 10 == h * 16) {
        return 6;
    }
    if (w * 4 == h * 5 || w * 9 == h * 15) {
        return 7;
    }
    return 10;
}

// CVT range-descriptor aspect bit (byte 14) of a size; 0 when none applies.
uint8_t cvt_aspect_bit(uint32_t width, uint32_t height) noexcept {
    const uint64_t w = width;
    const uint64_t h = height;
    if (w * 3 == h * 4) {
        return 0x80;
    }
    if (w * 9 == h * 16) {
        return 0x40;
    }
    if (w * 10 == h * 16) {
        return 0x20;
    }
    if (w * 4 == h * 5) {
        return 0x10;
    }
    return w * 9 == h * 15 ? 0x08 : 0;
}

class Collector final {
public:
    Collector(ModeCatalog& catalog, EdidModeSource (&sources)[mode_capacity], EdidModeReport& report) noexcept
        : catalog_(catalog), sources_(sources), report_(report) {}

    void add(const Timing& timing, EdidModeSource source) noexcept {
        uint32_t index = 0;
        switch (catalog_.append(timing, index)) {
        case Status::success:
            sources_[index] = source;
            ++report_.added[static_cast<uint32_t>(source)];
            break;
        case Status::duplicate_mode:
            ++report_.duplicates;
            break;
        case Status::capacity_exceeded:
            ++report_.dropped;
            break;
        default:
            ++report_.unsupported;
            break;
        }
    }

    void unsupported() noexcept { ++report_.unsupported; }

    // Linux valid_inferred_mode: no catalog mode of the same size and
    // refresh, and some catalog mode at least this wide and tall.
    bool inferable(const Timing& timing) const noexcept {
        bool covered = false;
        const uint32_t refresh = drm_vrefresh_hz(timing);
        for (uint32_t index = 0; index < catalog_.count(); ++index) {
            Timing mode{};
            if (catalog_.get(index, mode) != Status::success) {
                continue;
            }
            if (mode.width == timing.width && mode.height == timing.height && drm_vrefresh_hz(mode) == refresh) {
                return false;
            }
            covered = covered || (timing.width <= mode.width && timing.height <= mode.height);
        }
        return covered;
    }

private:
    ModeCatalog& catalog_;
    EdidModeSource (&sources_)[mode_capacity];
    EdidModeReport& report_;
};

// The base block facts every decoder below needs.
struct BaseFacts final {
    uint8_t revision = 0;
    bool digital = false;
    // drm_monitor_supports_rb: EDID 1.4 CVT range limits with reduced
    // blanking; before 1.4 every digital sink.
    bool reduced_blanking = false;
};

bool display_descriptor(ByteView base, uint32_t offset, uint8_t& tag) noexcept {
    if (le16(base, offset) != 0 || at(base, offset + 2) != 0) {
        return false;
    }
    tag = at(base, offset + 3);
    return true;
}

// E-EDID 1.4 Table 3.26/3.27 (offset flags in byte 4, CVT data in 11..17).
void parse_range(ByteView base, uint32_t offset, uint8_t revision, EdidRangeLimits& range,
                 Collector& collector) noexcept {
    const uint8_t flags = revision >= 4 ? at(base, offset + 4) : 0;
    const uint8_t vertical = flags & 0x03u;
    const uint8_t horizontal = (flags >> 2) & 0x03u;
    // 01 (minimum offset without maximum offset) is reserved.
    if (vertical == 1 || horizontal == 1 || (flags & 0xf0u) != 0) {
        collector.unsupported();
        return;
    }
    EdidRangeLimits limits{};
    limits.min_vertical_hz = at(base, offset + 5) + (vertical == 3 ? 255u : 0u);
    limits.max_vertical_hz = at(base, offset + 6) + (vertical >= 2 ? 255u : 0u);
    limits.min_horizontal_khz = at(base, offset + 7) + (horizontal == 3 ? 255u : 0u);
    limits.max_horizontal_khz = at(base, offset + 8) + (horizontal >= 2 ? 255u : 0u);
    const uint32_t clock = at(base, offset + 9);
    limits.max_pixel_clock_khz = clock == 0 || clock == 255 ? 0 : clock * 10000u;
    limits.cvt = revision >= 4 && at(base, offset + 10) == range_cvt_support;
    if (limits.cvt) {
        const uint8_t precision = at(base, offset + 12);
        const uint32_t reduction = static_cast<uint32_t>(precision >> 2) * 250u;
        limits.max_pixel_clock_khz = limits.max_pixel_clock_khz > reduction ? limits.max_pixel_clock_khz - reduction : 0;
        limits.max_active_pixels = cvt_cell * (at(base, offset + 13) + 256u * (precision & 0x03u));
        limits.aspect_ratios = at(base, offset + 14) & 0xf8u;
        const uint8_t blanking = at(base, offset + 15);
        limits.reduced_blanking = (blanking & 0x10u) != 0;
        limits.standard_blanking = (blanking & 0x08u) != 0;
        limits.preferred_refresh_hz = at(base, offset + 17);
    }
    if (limits.min_vertical_hz == 0 || limits.min_vertical_hz > limits.max_vertical_hz ||
        limits.min_horizontal_khz == 0 || limits.min_horizontal_khz > limits.max_horizontal_khz) {
        collector.unsupported();
        return;
    }
    limits.valid = true;
    range = limits;
}

bool in_range(const Timing& timing, const EdidRangeLimits& range, bool reduced_ok) noexcept {
    const uint32_t vertical = drm_vrefresh_hz(timing);
    const uint32_t horizontal = hsync_khz(timing);
    return range.valid && range.max_pixel_clock_khz != 0 && vertical >= range.min_vertical_hz &&
           vertical <= range.max_vertical_hz && horizontal >= range.min_horizontal_khz &&
           horizontal <= range.max_horizontal_khz && timing.pixel_clock_khz <= range.max_pixel_clock_khz &&
           (!range.cvt || range.max_active_pixels == 0 || timing.width <= range.max_active_pixels) &&
           (reduced_ok || !reduced_blanking_shape(timing));
}

// E-EDID 1.4 section 3.9 standard timing; Linux drm_mode_std order: DMT with
// reduced blanking when the sink takes it, plain DMT, then CVT for an EDID
// 1.4 continuous-frequency sink. GTF sinks get nothing (GTF is not
// implemented).
void add_standard(uint8_t first, uint8_t second, const BaseFacts& facts, bool continuous,
                  Collector& collector) noexcept {
    // Unused (01 01), reserved (00 xx) and ASCII-space padding (20 20).
    if (first == 0x00 || (first == 0x01 && second == 0x01) || (first == 0x20 && second == 0x20)) {
        return;
    }
    uint32_t width = (static_cast<uint32_t>(first) + 31u) * 8u;
    const uint32_t refresh = (second & 0x3fu) + 60u;
    uint32_t height = 0;
    uint32_t vsync = 0;
    switch (second >> 6) {
    case 0:
        height = facts.revision < 3 ? width : width * 10u / 16u;
        vsync = facts.revision < 3 ? 0u : 6u;
        break;
    case 1: height = width * 3u / 4u; vsync = 4; break;
    case 2: height = width * 4u / 5u; vsync = 7; break;
    default: height = width * 9u / 16u; vsync = 5; break;
    }
    // A standard timing cannot express 1366 (not a multiple of 8): 1360x765
    // and 1368x769 at 60 Hz name the 1366x768 panel timing.
    if (refresh == 60 && ((width == 1360 && height == 765) || (width == 1368 && height == 769))) {
        width = 1366;
        height = 768;
    }
    Timing timing{};
    if ((facts.reduced_blanking && dmt_timing(width, height, refresh, true, timing)) ||
        dmt_timing(width, height, refresh, false, timing) ||
        (facts.revision >= 4 && continuous &&
         cvt_timing(width, height, refresh, false, timing, vsync) == Status::success)) {
        collector.add(timing, EdidModeSource::standard);
        return;
    }
    collector.unsupported();
}

// E-EDID 1.4 section 3.10.3.8 CVT 3-byte timing code.
void add_cvt_code(ByteView base, uint32_t offset, Collector& collector) noexcept {
    const uint8_t lines_low = at(base, offset);
    const uint8_t aspect_lines = at(base, offset + 1);
    const uint8_t rates = at(base, offset + 2);
    if (lines_low == 0 && aspect_lines == 0 && rates == 0) {
        return;
    }
    const uint32_t height = ((static_cast<uint32_t>(lines_low) | ((aspect_lines & 0xf0u) << 4)) + 1u) * 2u;
    uint32_t width = 0;
    uint32_t vsync = 0;
    switch ((aspect_lines >> 2) & 0x03u) {
    case 0: width = height * 4u / 3u; vsync = 4; break;
    case 1: width = height * 16u / 9u; vsync = 5; break;
    case 2: width = height * 16u / 10u; vsync = 6; break;
    default: width = height * 15u / 9u; vsync = 7; break;
    }
    width = width / cvt_cell * cvt_cell;
    if ((aspect_lines & 0x03u) != 0 || (rates & 0x80u) != 0 || (rates & 0x1fu) == 0) {
        collector.unsupported();
        return;
    }
    struct Rate final {
        uint8_t bit;
        uint32_t hz;
        bool reduced;
    };
    constexpr Rate list[5] = {{0x01, 60, true}, {0x10, 50, false}, {0x08, 60, false}, {0x04, 75, false},
                              {0x02, 85, false}};
    for (const Rate& rate : list) {
        if ((rates & rate.bit) == 0) {
            continue;
        }
        Timing timing{};
        if (cvt_timing(width, height, rate.hz, rate.reduced, timing, vsync) == Status::success) {
            collector.add(timing, EdidModeSource::cvt_code);
        } else {
            collector.unsupported();
        }
    }
}

void add_established(const EstablishedMode& mode, Collector& collector) noexcept {
    if (mode.width == 0) {
        collector.unsupported();
        return;
    }
    Timing timing = mode.timing;
    if (mode.dmt && !dmt_timing(mode.width, mode.height, mode.refresh, false, timing)) {
        collector.unsupported();
        return;
    }
    collector.add(timing, EdidModeSource::established);
}

void add_established_iii(ByteView base, uint32_t offset, Collector& collector) noexcept {
    for (uint32_t index = 0; index < 44; ++index) {
        const uint8_t byte = at(base, offset + 6u + index / 8u);
        if ((byte & (0x80u >> (index % 8u))) == 0) {
            continue;
        }
        const MiniMode& mode = established_iii[index];
        Timing timing{};
        if (dmt_timing(mode.width, mode.height, mode.refresh, mode.reduced, timing)) {
            collector.add(timing, EdidModeSource::established);
        } else {
            collector.unsupported();
        }
    }
}

// DisplayID 1.3 section 4.4 / 2.0 section 4.4: 20-byte detailed timing
// descriptors. Byte 3: bit 4 interlaced, bits 6..5 stereo (01 stereo only).
void add_displayid_timing(ByteView section, size_t offset, bool type_vii, Collector& collector) noexcept {
    const uint32_t raw_clock = le16(section, offset) | (static_cast<uint32_t>(at(section, offset + 2)) << 16);
    const uint8_t options = at(section, offset + 3);
    if ((options & 0x10u) != 0 || ((options >> 5) & 0x03u) == 1) {
        collector.unsupported();
        return;
    }
    const uint32_t width = le16(section, offset + 4) + 1u;
    const uint32_t h_blank = le16(section, offset + 6) + 1u;
    const uint32_t h_front = le16(section, offset + 8);
    const uint32_t h_sync = le16(section, offset + 10) + 1u;
    const uint32_t height = le16(section, offset + 12) + 1u;
    const uint32_t v_blank = le16(section, offset + 14) + 1u;
    const uint32_t v_front = le16(section, offset + 16);
    const uint32_t v_sync = le16(section, offset + 18) + 1u;
    Timing timing{};
    timing.pixel_clock_khz = (raw_clock + 1u) * (type_vii ? 1u : 10u);
    timing.width = width;
    timing.h_total = width + h_blank;
    timing.h_front_porch = (h_front & 0x7fffu) + 1u;
    timing.h_sync_width = h_sync;
    timing.height = height;
    timing.v_total = height + v_blank;
    timing.v_front_porch = (v_front & 0x7fffu) + 1u;
    timing.v_sync_width = v_sync;
    timing.flags = ((h_front & 0x8000u) != 0 ? positive_hsync : 0u) | ((v_front & 0x8000u) != 0 ? positive_vsync : 0u);
    collector.add(timing, EdidModeSource::displayid);
}

// One DisplayID section in an EDID extension block (tag 0x70): bytes 1..4
// are the section header (version, payload bytes, product type, extension
// count), the payload follows, then the section checksum; all inside bytes
// 1..126 of the block. A bad section adds nothing.
void add_displayid(ByteView ext, Collector& collector) noexcept {
    const uint32_t payload = at(ext, 2);
    const uint32_t end = 5u + payload; // first byte after the payload: the checksum
    if (end > block_bytes - 2u) {
        collector.unsupported();
        return;
    }
    uint32_t sum = 0;
    for (uint32_t index = 1; index <= end; ++index) {
        sum += at(ext, index);
    }
    if ((sum & 0xffu) != 0) {
        collector.unsupported();
        return;
    }
    uint32_t offset = 5;
    while (offset + 3u <= end) {
        const uint8_t tag = at(ext, offset);
        const uint32_t length = at(ext, offset + 2u);
        if (tag == 0 && length == 0) {
            break; // padding
        }
        if (offset + 3u + length > end) {
            collector.unsupported();
            return;
        }
        if (tag == displayid_type_i_timing || tag == displayid_type_vii_timing) {
            if (length % displayid_timing_bytes != 0) {
                collector.unsupported();
            } else {
                for (uint32_t item = 0; item < length; item += displayid_timing_bytes) {
                    add_displayid_timing(ext, offset + 3u + item, tag == displayid_type_vii_timing, collector);
                }
            }
        }
        offset += 3u + length;
    }
}

void infer_from_range(const EdidRangeLimits& range, const BaseFacts& facts, Collector& collector) noexcept {
    for (const DmtMode& dmt : dmt_modes) {
        if (in_range(dmt.timing, range, facts.reduced_blanking) && collector.inferable(dmt.timing)) {
            collector.add(dmt.timing, EdidModeSource::inferred_dmt);
        }
    }
    if (facts.revision < 4 || !range.cvt) {
        return;
    }
    const bool reduced = facts.reduced_blanking && range.reduced_blanking;
    if (!reduced && !range.standard_blanking) {
        return;
    }
    for (uint32_t pass = 0; pass < 2; ++pass) {
        const uint32_t refresh = pass == 0 ? 60u : range.preferred_refresh_hz;
        if (pass == 1 && (refresh == 0 || refresh == 60)) {
            break;
        }
        for (const MiniMode& candidate : cvt_candidates) {
            Timing timing{};
            if ((range.aspect_ratios & cvt_aspect_bit(candidate.width, candidate.height)) == 0 ||
                cvt_timing(candidate.width, candidate.height, refresh, reduced, timing) != Status::success ||
                !in_range(timing, range, reduced) || !collector.inferable(timing)) {
                continue;
            }
            collector.add(timing, EdidModeSource::inferred_cvt);
        }
    }
}

} // namespace

Status cvt_timing(uint32_t width, uint32_t height, uint32_t refresh_hz, bool reduced_blanking,
                  Timing& output, uint32_t vsync_lines) noexcept {
    if (width < 64 || width > 16384 || height < 64 || height > 8192 || refresh_hz < 24 || refresh_hz > 480 ||
        vsync_lines > 10) {
        return Status::unsupported_timing;
    }
    const uint64_t active = width / cvt_cell * cvt_cell;
    const uint64_t lines = height;
    const uint64_t rate = refresh_hz;
    const uint64_t vsync = vsync_lines != 0 ? vsync_lines : cvt_vsync_lines(width, height);
    Timing timing{};
    if (!reduced_blanking) {
        // H_PERIOD_EST (us) = (1e6 / rate - MIN_VSYNC_BP 550) / (lines + MIN_V_PORCH 3)
        //                   = period_num / period_den.
        const uint64_t period_num = 1000000u - 550u * rate;
        const uint64_t period_den = rate * (lines + 3u);
        // V_SYNC_BP = floor(550 / H_PERIOD_EST) + 1, at least vsync + MIN_V_BPORCH 6.
        uint64_t sync_and_back = 550u * period_den / period_num + 1u;
        if (sync_and_back < vsync + 6u) {
            sync_and_back = vsync + 6u;
        }
        // IDEAL_DUTY_CYCLE (%) = C' 30 - M' 300 * H_PERIOD_EST / 1000
        //                      = duty_num / duty_den, at least 20.
        const int64_t duty_num = static_cast<int64_t>(300u * period_den) - static_cast<int64_t>(3u * period_num);
        const int64_t duty_den = static_cast<int64_t>(10u * period_den);
        uint64_t blank = 0;
        if (duty_num < 20 * duty_den) {
            blank = active * 20u / (80u * 2u * cvt_cell) * (2u * cvt_cell);
        } else {
            const uint64_t numerator = active * static_cast<uint64_t>(duty_num);
            const uint64_t denominator = static_cast<uint64_t>(100 * duty_den - duty_num) * (2u * cvt_cell);
            blank = numerator / denominator * (2u * cvt_cell);
        }
        const uint64_t total = active + blank;
        // ACT_PIXEL_FREQ = 0.25 MHz * floor(total / H_PERIOD_EST / 0.25).
        const uint64_t quarter_mhz = total * period_den * 4u / period_num;
        // H_SYNC = floor(8 % * total / cell) * cell; the sync ends mid-blank.
        const uint64_t hsync = total / 100u * cvt_cell;
        timing.pixel_clock_khz = static_cast<uint32_t>(quarter_mhz * 250u);
        timing.width = static_cast<uint32_t>(active);
        timing.h_total = static_cast<uint32_t>(total);
        timing.h_sync_width = static_cast<uint32_t>(hsync);
        timing.h_front_porch = static_cast<uint32_t>(blank / 2u - hsync);
        timing.height = height;
        timing.v_total = static_cast<uint32_t>(lines + sync_and_back + 3u);
        timing.v_front_porch = 3;
        timing.v_sync_width = static_cast<uint32_t>(vsync);
        timing.flags = positive_vsync;
    } else {
        // H_PERIOD_EST (us) = (1e6 / rate - RB_MIN_V_BLANK 460) / lines.
        const uint64_t period_num = 1000000u - 460u * rate;
        const uint64_t period_den = rate * lines;
        // VBI_LINES = floor(460 / H_PERIOD_EST) + 1, at least RB_V_FPORCH 3 +
        // vsync + MIN_V_BPORCH 6.
        uint64_t vbi = 460u * period_den / period_num + 1u;
        if (vbi < 3u + vsync + 6u) {
            vbi = 3u + vsync + 6u;
        }
        const uint64_t total_v = lines + vbi;
        const uint64_t total_h = active + 160u;
        // ACT_PIXEL_FREQ = 0.25 MHz * floor(rate * total_v * total_h / 1e6 / 0.25).
        const uint64_t quarter_mhz = rate * total_v * total_h * 4u / 1000000u;
        timing.pixel_clock_khz = static_cast<uint32_t>(quarter_mhz * 250u);
        timing.width = static_cast<uint32_t>(active);
        timing.h_total = static_cast<uint32_t>(total_h);
        timing.h_front_porch = 48;
        timing.h_sync_width = 32;
        timing.height = height;
        timing.v_total = static_cast<uint32_t>(total_v);
        timing.v_front_porch = 3;
        timing.v_sync_width = static_cast<uint32_t>(vsync);
        timing.flags = positive_hsync;
    }
    OptcTiming fields{};
    if (prepare_optc_timing(timing, fields) != Status::success) {
        return Status::unsupported_timing;
    }
    output = timing;
    return Status::success;
}

bool dmt_timing(uint32_t width, uint32_t height, uint32_t refresh_hz, bool reduced_blanking,
                Timing& output) noexcept {
    for (const DmtMode& mode : dmt_modes) {
        if (mode.timing.width == width && mode.timing.height == height &&
            drm_vrefresh_hz(mode.timing) == refresh_hz && reduced_blanking_shape(mode.timing) == reduced_blanking) {
            output = mode.timing;
            return true;
        }
    }
    return false;
}

Status append_edid_modes(ByteView edid, ModeCatalog& catalog, EdidModeSource (&sources)[mode_capacity],
                         EdidModeReport& report) noexcept {
    constexpr uint8_t header[8]{0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00};
    if (edid.size() < block_bytes || edid.size() % block_bytes != 0 ||
        edid.size() / block_bytes != static_cast<size_t>(at(edid, 126)) + 1u ||
        edid.size() / block_bytes > maximum_blocks || at(edid, 18) != 1 ||
        (at(edid, 19) != 3 && at(edid, 19) != 4)) {
        return Status::invalid_request;
    }
    for (uint32_t index = 0; index < sizeof(header); ++index) {
        if (at(edid, index) != header[index]) {
            return Status::invalid_request;
        }
    }

    report = EdidModeReport{};
    Collector collector(catalog, sources, report);
    BaseFacts facts{};
    facts.revision = at(edid, 19);
    facts.digital = (at(edid, 20) & 0x80u) != 0;
    report.revision = facts.revision;
    report.continuous_frequency = (at(edid, 24) & 0x01u) != 0;

    // Display descriptors of the base block: range limits first (they
    // decide standard-timing CVT and reduced blanking).
    for (uint32_t index = 0; index < base_descriptor_count; ++index) {
        uint8_t tag = 0;
        const uint32_t offset = base_descriptor_offset + index * descriptor_bytes;
        if (!report.range.valid && display_descriptor(edid, offset, tag) && tag == tag_range_limits) {
            parse_range(edid, offset, facts.revision, report.range, collector);
        }
    }
    facts.reduced_blanking = facts.revision >= 4 ? (report.range.valid && report.range.cvt &&
                                                    report.range.reduced_blanking)
                                                 : facts.digital;

    // add_cvt_modes.
    for (uint32_t index = 0; index < base_descriptor_count; ++index) {
        uint8_t tag = 0;
        const uint32_t offset = base_descriptor_offset + index * descriptor_bytes;
        if (display_descriptor(edid, offset, tag) && tag == tag_cvt_codes) {
            if (at(edid, offset + 5) != 1) {
                collector.unsupported();
                continue;
            }
            for (uint32_t code = 0; code < 4; ++code) {
                add_cvt_code(edid, offset + 6u + code * 3u, collector);
            }
        }
    }

    // add_standard_modes: the eight base slots, then descriptor 0xFA.
    for (uint32_t index = 0; index < standard_count; ++index) {
        add_standard(at(edid, standard_offset + index * 2u), at(edid, standard_offset + index * 2u + 1u), facts,
                     report.continuous_frequency, collector);
    }
    for (uint32_t index = 0; index < base_descriptor_count; ++index) {
        uint8_t tag = 0;
        const uint32_t offset = base_descriptor_offset + index * descriptor_bytes;
        if (display_descriptor(edid, offset, tag) && tag == tag_standard_timings) {
            for (uint32_t code = 0; code < 6; ++code) {
                add_standard(at(edid, offset + 5u + code * 2u), at(edid, offset + 6u + code * 2u), facts,
                             report.continuous_frequency, collector);
            }
        }
    }

    // add_established_modes: I/II bits, then established timings III.
    const uint32_t established = (static_cast<uint32_t>(at(edid, established_offset)) << 16) |
                                 (static_cast<uint32_t>(at(edid, established_offset + 1)) << 8) |
                                 at(edid, established_offset + 2);
    for (uint32_t index = 0; index < 17; ++index) {
        if ((established & (0x800000u >> index)) != 0) {
            add_established(established_i_ii[index], collector);
        }
    }
    for (uint32_t index = 0; index < base_descriptor_count; ++index) {
        uint8_t tag = 0;
        const uint32_t offset = base_descriptor_offset + index * descriptor_bytes;
        if (display_descriptor(edid, offset, tag) && tag == tag_established_iii) {
            add_established_iii(edid, offset, collector);
        }
    }

    // add_cea_modes: VDB short video descriptors, then HDMI VICs.
    CtaVideoList video{};
    if (parse_cta_video(edid, video) == HdmiStatus::success) {
        report.unsupported += video.dropped;
        for (uint32_t index = 0; index < video.vic_count; ++index) {
            Timing timing{};
            if (cea_vic_timing(video.vics[index], timing)) {
                collector.add(timing, EdidModeSource::cta_vic);
            } else {
                collector.unsupported();
            }
        }
        for (uint32_t index = 0; index < video.hdmi_vic_count; ++index) {
            Timing timing{};
            if (hdmi_vic_timing(video.hdmi_vics[index], timing)) {
                collector.add(timing, EdidModeSource::hdmi_vic);
            } else {
                collector.unsupported();
            }
        }
    }

    // add_displayid_detailed_modes.
    for (size_t block = 1; block < edid.size() / block_bytes; ++block) {
        ByteView ext{nullptr, 0};
        if (edid.subview(block * block_bytes, block_bytes, ext) && at(ext, 0) == displayid_extension_tag) {
            add_displayid(ext, collector);
        }
    }

    // add_inferred_modes (continuous frequency only).
    if (report.continuous_frequency && report.range.valid) {
        infer_from_range(report.range, facts, collector);
    }
    return Status::success;
}

} // namespace rx7900::display

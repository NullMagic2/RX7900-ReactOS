/*
Module: rx7900.tests.edid_modes
Purpose: Check EDID mode enumeration beyond detailed timings, the CVT/DMT generators and the VideoPort mode-list policy.
Created: 2026-10-24
Architecture: Host test of src/display/edid_modes and the hdmi_stream VIC
additions. EDIDs are synthesized the way real monitors and TVs are laid out
(an EDID 1.4 DisplayPort monitor with range limits, CVT codes, standard and
established timings; an EDID 1.3 HDMI TV with a CTA-861 extension, Video
Data Block and HDMI VSDB HDMI VICs; a DisplayID 1.3/2.0 extension), pass the
project's strict output::parse_edid and are then widened exactly as the
mode service does. CVT results are checked against the CVT-generated entries
of VESA DMT and published CVT values. The mode-list checks run the real
link-budget, TMDS and representability validators. No hardware access.
*/
#include <cstdio>
#include <cstring>
#include <initializer_list>
#include <vector>

#include "asic_test_device.hpp"
#include "dp_link.hpp"
#include "edid_modes.hpp"
#include "hdmi_stream.hpp"
#include "sink.hpp"

namespace {

using namespace rx7900::display;
using rx7900::ByteView;

rx7900_test::Checks checks;

void expect(bool condition, const char* contract) { checks.expect(condition, contract); }

constexpr uint32_t sync = positive_hsync | positive_vsync;

bool same(const Timing& a, const Timing& b) {
    return a.pixel_clock_khz == b.pixel_clock_khz && a.width == b.width && a.h_total == b.h_total &&
           a.h_front_porch == b.h_front_porch && a.h_sync_width == b.h_sync_width && a.height == b.height &&
           a.v_total == b.v_total && a.v_front_porch == b.v_front_porch && a.v_sync_width == b.v_sync_width &&
           a.flags == b.flags;
}

void print(const char* label, const Timing& t) {
    std::fprintf(stderr, "  %s: %u kHz %ux%u htotal %u hfp %u hs %u vtotal %u vfp %u vs %u flags %u\n", label,
                 t.pixel_clock_khz, t.width, t.height, t.h_total, t.h_front_porch, t.h_sync_width, t.v_total,
                 t.v_front_porch, t.v_sync_width, t.flags);
}

// ---- EDID synthesis ----

struct Edid final {
    std::vector<uint8_t> bytes = std::vector<uint8_t>(128, 0);

    explicit Edid(uint8_t revision, uint8_t input, uint8_t features) {
        for (uint32_t i = 1; i < 7; ++i) {
            bytes[i] = 0xff;
        }
        bytes[8] = 0x4a; // manufacturer "RXT" style id, model, serial: identity is not decoded
        bytes[9] = 0x8b;
        bytes[18] = 1;
        bytes[19] = revision;
        bytes[20] = input;
        bytes[21] = 60; // 60 x 34 cm
        bytes[22] = 34;
        bytes[23] = 120;
        bytes[24] = features;
        for (uint32_t i = 0; i < 8; ++i) {
            bytes[38 + i * 2] = 0x01;
            bytes[39 + i * 2] = 0x01;
        }
        for (uint32_t slot = 0; slot < 4; ++slot) {
            dummy(slot);
        }
    }

    // Dummy descriptor (tag 0x10).
    void dummy(uint32_t slot) {
        const uint32_t o = 54 + slot * 18;
        for (uint32_t i = 0; i < 18; ++i) {
            bytes[o + i] = 0;
        }
        bytes[o + 3] = 0x10;
    }

    static void dtd_into(std::vector<uint8_t>& b, size_t o, const Timing& t) {
        const uint32_t clock = t.pixel_clock_khz / 10;
        const uint32_t hb = t.h_total - t.width;
        const uint32_t vb = t.v_total - t.height;
        b[o + 0] = static_cast<uint8_t>(clock);
        b[o + 1] = static_cast<uint8_t>(clock >> 8);
        b[o + 2] = static_cast<uint8_t>(t.width);
        b[o + 3] = static_cast<uint8_t>(hb);
        b[o + 4] = static_cast<uint8_t>(((t.width >> 8) << 4) | (hb >> 8));
        b[o + 5] = static_cast<uint8_t>(t.height);
        b[o + 6] = static_cast<uint8_t>(vb);
        b[o + 7] = static_cast<uint8_t>(((t.height >> 8) << 4) | (vb >> 8));
        b[o + 8] = static_cast<uint8_t>(t.h_front_porch);
        b[o + 9] = static_cast<uint8_t>(t.h_sync_width);
        b[o + 10] = static_cast<uint8_t>(((t.v_front_porch & 0xf) << 4) | (t.v_sync_width & 0xf));
        b[o + 11] = static_cast<uint8_t>(((t.h_front_porch >> 8) << 6) | ((t.h_sync_width >> 8) << 4) |
                                         ((t.v_front_porch >> 4) << 2) | (t.v_sync_width >> 4));
        b[o + 12] = 0x58; // image size
        b[o + 13] = 0x54;
        b[o + 14] = 0x21;
        b[o + 15] = 0;
        b[o + 16] = 0;
        b[o + 17] = static_cast<uint8_t>(0x18 | ((t.flags & positive_vsync) ? 0x04 : 0) |
                                         ((t.flags & positive_hsync) ? 0x02 : 0));
    }

    void dtd(uint32_t slot, const Timing& t) { dtd_into(bytes, 54 + slot * 18, t); }

    void descriptor(uint32_t slot, uint8_t tag, std::initializer_list<uint8_t> payload) {
        const uint32_t o = 54 + slot * 18;
        for (uint32_t i = 0; i < 18; ++i) {
            bytes[o + i] = 0;
        }
        bytes[o + 3] = tag;
        uint32_t i = 4;
        for (uint8_t v : payload) {
            bytes[o + i++] = v;
        }
    }

    void standard(uint32_t slot, uint8_t a, uint8_t b) {
        bytes[38 + slot * 2] = a;
        bytes[39 + slot * 2] = b;
    }

    // Appends an extension block (tag in bytes[0] of block); returns its base.
    size_t extension(uint8_t tag) {
        const size_t base = bytes.size();
        bytes.resize(base + 128, 0);
        bytes[base] = tag;
        bytes[126] = static_cast<uint8_t>(bytes[126] + 1);
        return base;
    }

    void finish() {
        for (size_t base = 0; base < bytes.size(); base += 128) {
            uint32_t sum = 0;
            for (size_t i = 0; i < 127; ++i) {
                sum += bytes[base + i];
            }
            bytes[base + 127] = static_cast<uint8_t>(0x100u - (sum & 0xffu));
        }
    }

    ByteView view() const { return ByteView(bytes.data(), bytes.size()); }
};

// Parses like the mode service: DTDs through the strict sink parser, then
// every other EDID mode appended behind them.
struct Decoded final {
    rx7900::output::SinkSnapshot sink{};
    EdidModeSource sources[mode_capacity]{};
    EdidModeReport report{};
    Status status = Status::invalid_request;

    explicit Decoded(const Edid& edid) {
        if (rx7900::output::parse_edid(edid.view(), sink) != rx7900::output::Status::success) {
            return;
        }
        status = append_edid_modes(edid.view(), sink.detailed_modes, sources, report);
    }

    uint32_t count() const { return sink.detailed_modes.count(); }
    Timing at(uint32_t i) const {
        Timing t{};
        sink.detailed_modes.get(i, t);
        return t;
    }
    // Index of a mode with this size and rounded refresh from source, or UINT32_MAX.
    uint32_t find(uint32_t w, uint32_t h, uint32_t hz, EdidModeSource source) const {
        for (uint32_t i = 0; i < count(); ++i) {
            const Timing t = at(i);
            if (t.width == w && t.height == h && drm_vrefresh_hz(t) == hz && sources[i] == source) {
                return i;
            }
        }
        return UINT32_MAX;
    }
    uint32_t added(EdidModeSource source) const { return report.added[static_cast<uint32_t>(source)]; }
};

uint32_t hsync_khz(const Timing& t) {
    return static_cast<uint32_t>((static_cast<uint64_t>(t.pixel_clock_khz) * 1000u / t.h_total + 500u) / 1000u);
}

// ---- generators ----

void cvt_and_dmt() {
    Timing t{};
    expect(cvt_timing(1920, 1080, 60, false, t) == Status::success &&
               same(t, {173000, 1920, 2576, 128, 200, 1080, 1120, 3, 5, positive_vsync}),
           "CVT 1920x1080@60 is the published 173.00 MHz 2576x1120 timing");
    expect(cvt_timing(1920, 1080, 60, true, t) == Status::success &&
               same(t, {138500, 1920, 2080, 48, 32, 1080, 1111, 3, 5, positive_hsync}),
           "CVT-RB 1920x1080@60 is 138.50 MHz 2080x1111");
    expect(cvt_timing(1280, 720, 60, false, t) == Status::success && t.pixel_clock_khz == 74500 &&
               t.h_total == 1664 && t.v_total == 748,
           "CVT 1280x720@60 is 74.50 MHz 1664x748");
    expect(cvt_timing(1024, 768, 60, false, t) == Status::success &&
               same(t, {63500, 1024, 1328, 48, 104, 768, 798, 3, 4, positive_vsync}),
           "CVT 1024x768@60 is 63.50 MHz 1328x798 with the 4:3 four-line sync");
    expect(cvt_timing(3840, 2160, 60, true, t) == Status::success && t.pixel_clock_khz == 533250 &&
               t.h_total == 4000 && t.v_total == 2222,
           "CVT-RB 3840x2160@60 is 533.25 MHz 4000x2222");
    expect(cvt_timing(2560, 1440, 60, true, t) == Status::success && t.pixel_clock_khz == 241500 &&
               t.h_total == 2720 && t.v_total == 1481,
           "CVT-RB 2560x1440@60 is 241.50 MHz 2720x1481");
    expect(cvt_timing(1366, 768, 60, false, t) == Status::success && t.width == 1360 && t.v_sync_width == 10,
           "CVT rounds 1366 to the 8-pixel cell and a non-standard aspect takes a 10-line sync");
    // Low rates: the minimum vertical back porch and the 20 % duty-cycle floor apply.
    expect(cvt_timing(640, 480, 24, false, t) == Status::success &&
               same(t, {9250, 640, 800, 16, 64, 480, 493, 3, 4, positive_vsync}),
           "CVT 640x480@24 takes MIN_V_BPORCH (10 sync+back lines) and the 20 % blanking floor");
    Timing untouched{1, 2, 3, 4, 5, 6, 7, 8, 9, 0};
    t = untouched;
    expect(cvt_timing(32, 480, 60, false, t) == Status::unsupported_timing && same(t, untouched) &&
               cvt_timing(640, 480, 10, false, t) == Status::unsupported_timing &&
               cvt_timing(640, 480, 500, true, t) == Status::unsupported_timing && same(t, untouched),
           "CVT refuses sizes and rates outside its range and leaves output alone");

    // VESA DMT entries the DMT standard defines as CVT (and CVT-RB) timings.
    struct Cvt final {
        uint32_t w, h, hz;
        bool rb;
    };
    const Cvt cvt_defined[] = {
        {800, 600, 120, true},   {1024, 768, 120, true},  {1280, 768, 60, true},   {1280, 768, 60, false},
        {1280, 768, 75, false},  {1280, 768, 85, false},  {1280, 768, 120, true},  {1280, 800, 60, true},
        {1280, 800, 60, false},  {1280, 800, 75, false},  {1280, 800, 85, false},  {1280, 800, 120, true},
        {1280, 960, 120, true},  {1280, 1024, 120, true}, {1400, 1050, 60, true},
        {1400, 1050, 60, false}, {1400, 1050, 75, false}, {1400, 1050, 85, false}, {1400, 1050, 120, true},
        {1440, 900, 60, true},   {1440, 900, 60, false},  {1440, 900, 75, false},  {1440, 900, 85, false},
        {1440, 900, 120, true},  {1600, 1200, 120, true}, {1680, 1050, 60, true},  {1680, 1050, 60, false},
        {1680, 1050, 75, false}, {1680, 1050, 85, false}, {1680, 1050, 120, true}, {1792, 1344, 120, true},
        {1856, 1392, 120, true}, {1920, 1200, 60, true},  {1920, 1200, 60, false}, {1920, 1200, 75, false},
        {1920, 1200, 85, false}, {1920, 1200, 120, true}, {1920, 1440, 120, true}, {2560, 1600, 60, true},
        {2560, 1600, 60, false}, {2560, 1600, 75, false}, {2560, 1600, 85, false}, {2560, 1600, 120, true},
    };
    unsigned matched = 0;
    for (const Cvt& c : cvt_defined) {
        Timing generated{};
        Timing table{};
        const bool ok = cvt_timing(c.w, c.h, c.hz, c.rb, generated) == Status::success &&
                        dmt_timing(c.w, c.h, c.hz, c.rb, table) && same(generated, table);
        if (!ok) {
            std::fprintf(stderr, "CVT %ux%u@%u%s differs from DMT\n", c.w, c.h, c.hz, c.rb ? " RB" : "");
            print("cvt", generated);
            print("dmt", table);
        }
        matched += ok ? 1u : 0u;
    }
    expect(matched == sizeof(cvt_defined) / sizeof(cvt_defined[0]),
           "every CVT-defined DMT timing (43, incl. RB at 60 and 120 Hz) is reproduced exactly");
    // DMT 0x28 declares 1360x768 as 16:9 (5-line sync) although 1360:768 is not exact.
    Timing declared{};
    Timing dmt28{};
    expect(cvt_timing(1360, 768, 120, true, declared, 5) == Status::success &&
               dmt_timing(1360, 768, 120, true, dmt28) && same(declared, dmt28) &&
               cvt_timing(1360, 768, 120, true, t) == Status::success && t.v_sync_width == 10,
           "a declared 16:9 aspect gives DMT 0x28's 5-line sync; the derived aspect gives 10");
    expect(cvt_timing(640, 480, 60, false, t, 11) == Status::unsupported_timing, "a sync over 10 lines is refused");

    expect(dmt_timing(1920, 1080, 60, false, t) && same(t, {148500, 1920, 2200, 88, 44, 1080, 1125, 4, 5, sync}),
           "DMT 0x52 1920x1080@60 is the CTA timing with +H +V");
    expect(dmt_timing(1366, 768, 60, false, t) && t.pixel_clock_khz == 85500, "DMT 1366x768@60 is 0x51");
    expect(dmt_timing(4096, 2160, 60, false, t) && t.pixel_clock_khz == 556744,
           "DMT 4096x2160@60 (0x57) is found by its rounded refresh");
    t = untouched;
    expect(!dmt_timing(1234, 567, 60, false, t) && !dmt_timing(1920, 1080, 60, true, t) && same(t, untouched),
           "DMT lookups without a table entry fail and leave output alone");
}

void vic_tables() {
    Timing t{};
    expect(cea_vic_timing(16, t) && same(t, {148500, 1920, 2200, 88, 44, 1080, 1125, 4, 5, sync}),
           "VIC 16 is 1920x1080p60");
    expect(cea_vic_timing(97, t) && t.pixel_clock_khz == 594000 && t.width == 3840 && t.h_total == 4400,
           "VIC 97 is 3840x2160p60");
    expect(cea_vic_timing(1, t) && t.flags == 0 && t.width == 640, "VIC 1 is 640x480 with negative syncs");
    const Timing untouched{1, 2, 3, 4, 5, 6, 7, 8, 9, 0};
    t = untouched;
    expect(!cea_vic_timing(5, t) && !cea_vic_timing(6, t) && !cea_vic_timing(8, t) && !cea_vic_timing(0, t) &&
               !cea_vic_timing(128, t) && !cea_vic_timing(220, t) && same(t, untouched),
           "interlaced (5), pixel-repeated (6, progressive 8) and unknown VICs give no timing");
    expect(hdmi_vic_timing(1, t) && t.pixel_clock_khz == 297000 && t.h_total == 4400 && hdmi_vic_timing(4, t) &&
               t.width == 4096,
           "HDMI VIC 1 is 3840x2160@30 and HDMI VIC 4 is 4096x2160@24");
    t = untouched;
    expect(!hdmi_vic_timing(0, t) && !hdmi_vic_timing(5, t) && same(t, untouched), "HDMI VICs outside 1..4 fail");
}

// ---- an EDID 1.4 DisplayPort monitor ----

Edid dp_monitor() {
    // Digital, 8 bpc, DisplayPort; continuous frequency, preferred native, sRGB.
    Edid e(4, 0xa5, 0x07);
    e.bytes[35] = 0x21; // 640x480@60, 800x600@60
    e.bytes[36] = 0x18; // 1024x768@60 and the interlaced 1024x768@87
    e.bytes[37] = 0x80; // Apple 1152x870@75
    e.standard(0, 0xd1, 0xc0); // 1920x1080@60
    e.standard(1, 0xb3, 0x00); // 1680x1050@60
    e.standard(2, 0x81, 0x80); // 1280x1024@60
    e.standard(3, 0x95, 0x0f); // 1440x900@75
    e.standard(4, 0x8c, 0xc0); // 1368x769@60 -> 1366x768
    Timing native{};
    cvt_timing(2560, 1440, 60, true, native);
    e.dtd(0, native);
    // Range limits: 48-75 Hz, 30-160 kHz, CVT 1.1, 300 MHz - 0.5 MHz, 2560 active
    // pixels, 16:9 + 16:10, preferred 16:9, RB and standard blanking, 60 Hz.
    e.descriptor(1, 0xfd, {0x00, 48, 75, 30, 160, 30, 0x04, 0x11, 0x09, 64, 0x60, 0x38, 0x00, 60});
    // CVT 3-byte codes: 1920x1200 (16:10) at 60 Hz standard and reduced blanking.
    e.descriptor(2, 0xf8, {0x00, 0x01, 0x57, 0x28, 0x29});
    // Established timings III: 848x480@60, 1280x768@60 RB.
    e.descriptor(3, 0xf7, {0x00, 0x0a, 0x08, 0x80, 0x00, 0x00, 0x00, 0x00});
    e.finish();
    return e;
}

void monitor_modes() {
    const Edid e = dp_monitor();
    const Decoded d(e);
    expect(d.status == Status::success && d.sink.detailed_modes.count() > 1 && d.sources[0] == EdidModeSource::detailed,
           "the monitor EDID parses and widens");
    expect(d.report.revision == 4 && d.report.continuous_frequency && d.report.range.valid && d.report.range.cvt &&
               d.report.range.max_pixel_clock_khz == 299500 && d.report.range.max_active_pixels == 2560 &&
               d.report.range.reduced_blanking && d.report.range.standard_blanking &&
               d.report.range.aspect_ratios == 0x60 && d.report.range.preferred_refresh_hz == 60,
           "range limits decode the CVT precision, active-pixel limit, aspects and blanking");
    expect(d.added(EdidModeSource::cvt_code) == 2 && d.find(1920, 1200, 60, EdidModeSource::cvt_code) != UINT32_MAX,
           "CVT codes give 1920x1200@60 standard and reduced blanking");
    Timing t = d.at(d.find(1920, 1200, 60, EdidModeSource::cvt_code));
    expect(t.pixel_clock_khz == 154000, "the first 1920x1200 CVT code mode is the RB one (DMT 0x44, 154 MHz)");
    expect(d.added(EdidModeSource::standard) == 5, "five standard timings");
    const uint32_t s1680 = d.find(1680, 1050, 60, EdidModeSource::standard);
    expect(s1680 != UINT32_MAX && d.at(s1680).pixel_clock_khz == 119000,
           "an RB-capable sink takes the reduced-blanking DMT for a standard timing (1680x1050 RB)");
    const uint32_t s1366 = d.find(1366, 768, 60, EdidModeSource::standard);
    expect(s1366 != UINT32_MAX && d.at(s1366).pixel_clock_khz == 85500,
           "1368x769@60 names the 1366x768 DMT timing");
    const uint32_t s1080 = d.find(1920, 1080, 60, EdidModeSource::standard);
    expect(s1080 != UINT32_MAX && d.at(s1080).pixel_clock_khz == 148500 && d.at(s1080).flags == sync,
           "1920x1080@60 standard timing is DMT 0x52");
    expect(d.find(1440, 900, 75, EdidModeSource::standard) != UINT32_MAX &&
               d.find(1280, 1024, 60, EdidModeSource::standard) != UINT32_MAX,
           "1440x900@75 and 1280x1024@60 standard timings");
    expect(d.added(EdidModeSource::established) == 5 &&
               d.find(640, 480, 60, EdidModeSource::established) != UINT32_MAX &&
               d.find(800, 600, 60, EdidModeSource::established) != UINT32_MAX &&
               d.find(1024, 768, 60, EdidModeSource::established) != UINT32_MAX &&
               d.find(848, 480, 60, EdidModeSource::established) != UINT32_MAX &&
               d.find(1280, 768, 60, EdidModeSource::established) != UINT32_MAX,
           "established I/II and III timings map to DMT");
    expect(d.at(d.find(1280, 768, 60, EdidModeSource::established)).pixel_clock_khz == 68250,
           "established III 1280x768@60 RB is DMT 0x16");
    expect(d.report.unsupported == 2, "1024x768@87 interlaced and Apple 1152x870 are not produced");

    // Inferred modes: in range, inside the native size, never a repeat.
    uint32_t inferred = 0;
    bool ok = true;
    for (uint32_t i = 0; i < d.count(); ++i) {
        if (d.sources[i] != EdidModeSource::inferred_dmt && d.sources[i] != EdidModeSource::inferred_cvt) {
            continue;
        }
        ++inferred;
        const Timing m = d.at(i);
        const uint32_t hz = drm_vrefresh_hz(m);
        ok = ok && hz >= 48 && hz <= 75 && hsync_khz(m) >= 30 && hsync_khz(m) <= 160 &&
             m.pixel_clock_khz <= 299500 && m.width <= 2560 && m.height <= 1440;
        for (uint32_t j = 0; j < i; ++j) {
            const Timing o = d.at(j);
            ok = ok && !(o.width == m.width && o.height == m.height && drm_vrefresh_hz(o) == hz);
        }
    }
    expect(ok && inferred > 10, "inferred DMT modes are in range, inside 2560x1440 and never repeat a mode");
    expect(d.find(1600, 900, 60, EdidModeSource::inferred_dmt) != UINT32_MAX &&
               d.find(800, 600, 56, EdidModeSource::inferred_dmt) != UINT32_MAX &&
               d.find(1280, 720, 60, EdidModeSource::inferred_dmt) != UINT32_MAX,
           "range inference adds DMT 1600x900, 800x600@56 and 1280x720");
    expect(d.find(640, 350, 85, EdidModeSource::inferred_dmt) == UINT32_MAX &&
               d.find(2560, 1600, 60, EdidModeSource::inferred_dmt) == UINT32_MAX &&
               d.find(1280, 768, 120, EdidModeSource::inferred_dmt) == UINT32_MAX,
           "no inference above the vertical range or the native size");
    expect(d.report.duplicates == 0 && d.report.dropped == 0, "no duplicates or capacity drops for the monitor");
}

// ---- an EDID 1.3 HDMI TV with a CTA-861 extension ----

constexpr Timing vic16{148500, 1920, 2200, 88, 44, 1080, 1125, 4, 5, sync};
constexpr Timing vic4{74250, 1280, 1650, 110, 40, 720, 750, 5, 5, sync};

Edid hdmi_tv(bool with_hdmi_vics) {
    Edid e(3, 0x80, 0x0a); // digital, no continuous frequency
    e.bytes[35] = 0x20;    // 640x480@60
    e.dtd(0, vic16);
    e.dtd(1, vic4);
    e.descriptor(2, 0xfd, {0x00, 23, 61, 15, 69, 15, 0x00, 0x0a, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20});
    e.descriptor(3, 0xfc, {0x00, 'R', 'X', ' ', 'T', 'V', 0x0a, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20});
    const size_t c = e.extension(0x02);
    e.bytes[c + 1] = 3;
    e.bytes[c + 3] = 0x71;
    size_t o = c + 4;
    // Video Data Block: 16 (native), 4, 31, 5 (1080i), 97 (2160p60), 6 (480i), 0 (reserved).
    const uint8_t vdb[] = {0x47, 0x90, 0x04, 0x1f, 0x05, 0x61, 0x06, 0x00};
    for (uint8_t v : vdb) {
        e.bytes[o++] = v;
    }
    // HDMI 1.4b VSDB: OUI 00-0C-03, address 1.0.0.0, max TMDS 300 MHz,
    // HDMI_Video_present; HDMI_VIC_LEN 3: VICs 1, 3 and the invalid 9.
    const uint8_t vsdb_vics[] = {0x6d, 0x03, 0x0c, 0x00, 0x10, 0x00, 0x00, 0x3c, 0x20, 0x00, 0x60, 0x01, 0x03, 0x09};
    const uint8_t vsdb_plain[] = {0x67, 0x03, 0x0c, 0x00, 0x10, 0x00, 0x00, 0x3c};
    if (with_hdmi_vics) {
        for (uint8_t v : vsdb_vics) {
            e.bytes[o++] = v;
        }
    } else {
        for (uint8_t v : vsdb_plain) {
            e.bytes[o++] = v;
        }
    }
    e.bytes[c + 2] = static_cast<uint8_t>(o - c);
    // One CTA DTD: 1920x1080@50 (VIC 31 timing).
    Timing vic31{};
    cea_vic_timing(31, vic31);
    Edid::dtd_into(e.bytes, o, vic31);
    e.finish();
    return e;
}

void tv_modes() {
    const Edid e = hdmi_tv(true);
    CtaVideoList video{};
    expect(parse_cta_video(e.view(), video) == HdmiStatus::success && video.vic_count == 6 && video.vics[0] == 16 &&
               video.vics[4] == 97 && video.hdmi_vic_count == 2 && video.hdmi_vics[0] == 1 &&
               video.hdmi_vics[1] == 3 && video.dropped == 2,
           "CTA video list keeps VDB order without the native flag and the HDMI VSDB VIC list");
    HdmiSinkInfo info{};
    expect(parse_hdmi_sink(e.view(), info) == HdmiStatus::success && info.is_hdmi && info.has_hdmi_infoframe &&
               info.max_tmds_clock_khz == 300000,
           "parse_hdmi_sink results are unchanged by the shared walk");

    const Decoded d(e);
    expect(d.status == Status::success && d.sources[0] == EdidModeSource::detailed && same(d.at(0), vic16) &&
               same(d.at(1), vic4),
           "the TV's DTDs stay first at their parse indices");
    expect(d.added(EdidModeSource::cta_vic) == 1 && d.find(3840, 2160, 60, EdidModeSource::cta_vic) != UINT32_MAX,
           "VDB adds 2160p60; 1080p60, 720p60 and 1080p50 were already DTDs");
    expect(d.report.duplicates == 3, "VIC 16, 4 and 31 duplicate the detailed timings exactly");
    expect(d.added(EdidModeSource::hdmi_vic) == 2 && d.find(3840, 2160, 30, EdidModeSource::hdmi_vic) != UINT32_MAX &&
               d.find(3840, 2160, 24, EdidModeSource::hdmi_vic) != UINT32_MAX,
           "HDMI VICs 1 and 3 add 2160p30 and 2160p24");
    expect(d.added(EdidModeSource::established) == 1 && d.added(EdidModeSource::inferred_dmt) == 0 &&
               d.added(EdidModeSource::inferred_cvt) == 0,
           "640x480@60 established; no continuous frequency, so nothing is inferred");
    // VIC 5, VIC 6, VIC 0 (dropped by the list) and HDMI VIC 9.
    expect(d.report.unsupported == 4, "interlaced, pixel-repeated and reserved codes are counted, not guessed");

    // Mode list for an HDMI 1.4 sink: TMDS rules decide, earliest source wins.
    struct Listed final {
        Timing timing;
        uint32_t millihz;
        uint32_t index;
    };
    Listed list[8]{};
    uint32_t count = 1;
    list[0] = {vic16, 60000, UINT32_MAX}; // the GOP mode
    count = select_mode_list(
        d.sink.detailed_modes, count, 8,
        [&](uint32_t slot) { return listed_identity(list[slot].timing, list[slot].millihz); },
        [&](uint32_t slot, uint32_t index, const Timing& timing, uint32_t millihz) {
            TmdsStreamPlan plan{};
            if (plan_tmds_stream(timing, info, plan) != HdmiStatus::success) {
                return false;
            }
            list[slot] = {timing, millihz, index};
            return true;
        });
    bool has_2160p60 = false;
    bool has_2160p30 = false;
    for (uint32_t i = 0; i < count; ++i) {
        has_2160p60 = has_2160p60 || (list[i].timing.height == 2160 && list[i].millihz / 1000 == 60);
        has_2160p30 = has_2160p30 || (list[i].timing.height == 2160 && (list[i].millihz + 500) / 1000 == 30);
    }
    expect(count == 6 && list[1].index == 1 && !has_2160p60 && has_2160p30,
           "TV list: GOP 1080p60 once, 720p, 1080p50, 2160p30/24 and 640x480; 594 MHz exceeds the 300 MHz sink");
    expect(list[2].timing.height == 1080 && (list[2].millihz + 500) / 1000 == 50, "the CTA DTD 1080p50 is listed");

    // A VSDB with progressive and interlaced latency fields before the HDMI_VIC list.
    for (uint8_t length : {uint8_t{15}, uint8_t{14}}) {
        Edid t = hdmi_tv(false);
        const size_t c = 128;
        // tag 3, then OUI, address, flags, 300 MHz, latency + interlaced latency +
        // HDMI_Video_present, 2 + 2 latency bytes, 3D byte, HDMI_VIC_LEN 1, VIC 2.
        const uint8_t payload[] = {0x03, 0x0c, 0x00, 0x10, 0x00, 0x00, 0x3c, 0xe0,
                                   0x10, 0x10, 0x20, 0x20, 0x00, 0x20, 0x02};
        size_t o = c + 4 + 8;
        t.bytes[o++] = static_cast<uint8_t>(0x60 | length);
        for (uint32_t i = 0; i < length; ++i) {
            t.bytes[o++] = payload[i];
        }
        t.bytes[c + 2] = static_cast<uint8_t>(o - c);
        for (size_t i = o; i < c + 127; ++i) {
            t.bytes[i] = 0;
        }
        t.finish();
        CtaVideoList latency{};
        const bool parsed = parse_cta_video(t.view(), latency) == HdmiStatus::success;
        if (length == 15) {
            expect(parsed && latency.hdmi_vic_count == 1 && latency.hdmi_vics[0] == 2 && latency.dropped == 1,
                   "the HDMI_VIC list follows the latency and interlaced latency fields");
        } else {
            expect(parsed && latency.hdmi_vic_count == 0 && latency.dropped == 2,
                   "an HDMI_VIC list running past its block is cut, not read");
        }
    }

    const Edid plain = hdmi_tv(false);
    const Decoded p(plain);
    expect(p.status == Status::success && p.added(EdidModeSource::hdmi_vic) == 0,
           "a VSDB without HDMI_Video_present lists no HDMI VICs");
}

// ---- DisplayID extension ----

// One 20-byte DisplayID detailed timing descriptor (clock in units of clock_units kHz, minus one).
void put_displayid_timing(std::vector<uint8_t>& b, size_t o, const Timing& t, uint32_t clock_units, uint8_t options) {
    auto put16 = [&](size_t at, uint32_t v) {
        b[at] = static_cast<uint8_t>(v);
        b[at + 1] = static_cast<uint8_t>(v >> 8);
    };
    const uint32_t raw = t.pixel_clock_khz / clock_units - 1;
    b[o] = static_cast<uint8_t>(raw);
    b[o + 1] = static_cast<uint8_t>(raw >> 8);
    b[o + 2] = static_cast<uint8_t>(raw >> 16);
    b[o + 3] = options;
    put16(o + 4, t.width - 1);
    put16(o + 6, t.h_total - t.width - 1);
    put16(o + 8, (t.h_front_porch - 1) | ((t.flags & positive_hsync) ? 0x8000u : 0u));
    put16(o + 10, t.h_sync_width - 1);
    put16(o + 12, t.height - 1);
    put16(o + 14, t.v_total - t.height - 1);
    put16(o + 16, (t.v_front_porch - 1) | ((t.flags & positive_vsync) ? 0x8000u : 0u));
    put16(o + 18, t.v_sync_width - 1);
}

void displayid_modes() {
    Edid e(4, 0xa5, 0x06);
    Timing native{};
    cvt_timing(1920, 1080, 60, true, native);
    e.dtd(0, native);
    const size_t x = e.extension(0x70);
    Timing uhd{};
    cvt_timing(3840, 2160, 60, true, uhd);
    Timing five_k{};
    cvt_timing(5120, 2880, 60, true, five_k);
    auto descriptor = [&](size_t o, const Timing& t, uint32_t clock_units, uint8_t options) {
        put_displayid_timing(e.bytes, o, t, clock_units, options);
    };
    // Section: DisplayID 1.3, product type 3 (monitor).
    e.bytes[x + 1] = 0x12;
    e.bytes[x + 3] = 0x03;
    size_t o = x + 5;
    // Type I block: preferred 3840x2160@60 and an interlaced entry.
    e.bytes[o] = 0x03;
    e.bytes[o + 1] = 0x00;
    e.bytes[o + 2] = 40;
    descriptor(o + 3, uhd, 10, 0x80 | 0x04);
    descriptor(o + 23, uhd, 10, 0x10);
    o += 43;
    // Type VII block (1 kHz clock): 5120x2880@60.
    e.bytes[o] = 0x22;
    e.bytes[o + 1] = 0x00;
    e.bytes[o + 2] = 20;
    descriptor(o + 3, five_k, 1, 0x00);
    o += 23;
    // A timing block whose length is not a whole descriptor.
    e.bytes[o] = 0x03;
    e.bytes[o + 2] = 2;
    o += 5;
    e.bytes[x + 2] = static_cast<uint8_t>(o - (x + 5));
    uint32_t sum = 0;
    for (size_t i = x + 1; i < o; ++i) {
        sum += e.bytes[i];
    }
    e.bytes[o] = static_cast<uint8_t>(0x100u - (sum & 0xffu));

    // A section that would run into the EDID block checksum byte is refused even
    // when that byte happens to balance the section sum (append_edid_modes does
    // not check block checksums; parse_edid does).
    {
        Edid z(4, 0xa5, 0x06);
        z.dtd(0, native);
        const size_t q = z.extension(0x70);
        z.bytes[q + 1] = 0x12;
        z.bytes[q + 2] = 122; // header 1..4 + 122 payload: the section checksum would be byte 127
        z.bytes[q + 5] = 0x03;
        z.bytes[q + 7] = 20;
        put_displayid_timing(z.bytes, q + 8, uhd, 10, 0);
        z.finish();
        uint32_t total = 0;
        for (size_t i = q + 1; i < q + 127; ++i) {
            total += z.bytes[i];
        }
        z.bytes[q + 127] = static_cast<uint8_t>(0x100u - (total & 0xffu));
        ModeCatalog catalog{};
        uint32_t index = 0;
        catalog.append(native, index);
        EdidModeSource sources[mode_capacity]{};
        EdidModeReport report{};
        expect(append_edid_modes(z.view(), catalog, sources, report) == Status::success && catalog.count() == 1 &&
                   report.added[static_cast<uint32_t>(EdidModeSource::displayid)] == 0 && report.unsupported == 1,
               "a DisplayID section overlapping the EDID block checksum byte adds nothing");
    }

    // A second DisplayID block whose section checksum is wrong.
    const size_t y = e.extension(0x70);
    e.bytes[y + 1] = 0x12;
    e.bytes[y + 2] = 23;
    e.bytes[y + 5] = 0x03;
    e.bytes[y + 7] = 20;
    Timing other{};
    cvt_timing(2560, 1080, 60, true, other);
    descriptor(y + 8, other, 10, 0);
    uint32_t bad = 0;
    for (size_t i = y + 1; i < y + 5 + 23; ++i) {
        bad += e.bytes[i];
    }
    e.bytes[y + 5 + 23] = static_cast<uint8_t>(0x101u - (bad & 0xffu)); // one off
    e.finish();

    const Decoded d(e);
    expect(d.status == Status::success && d.sink.unsupported_extensions == 2,
           "DisplayID blocks pass the strict EDID parser as extensions it does not decode");
    const uint32_t i_uhd = d.find(3840, 2160, 60, EdidModeSource::displayid);
    const uint32_t i_5k = d.find(5120, 2880, 60, EdidModeSource::displayid);
    expect(i_uhd != UINT32_MAX && same(d.at(i_uhd), uhd), "type I timing (10 kHz units) decodes exactly");
    expect(i_5k != UINT32_MAX && same(d.at(i_5k), five_k), "type VII timing (1 kHz units) decodes exactly");
    expect(d.added(EdidModeSource::displayid) == 2 && d.find(2560, 1080, 60, EdidModeSource::displayid) == UINT32_MAX,
           "the bad-checksum section adds nothing");
    expect(d.report.unsupported == 3, "interlaced entry, partial block and bad section are counted");
}

// ---- range inference variants ----

void range_inference() {
    // EDID 1.4, standard blanking only, 16:9, 49-61 Hz, 15-90 kHz, 230 MHz,
    // preferred refresh 50 Hz; native 1920x1080@60.
    Edid e(4, 0xa5, 0x07);
    e.dtd(0, vic16);
    e.descriptor(1, 0xfd, {0x00, 49, 61, 15, 90, 23, 0x04, 0x11, 0x00, 0x00, 0x40, 0x28, 0x00, 50});
    e.finish();
    const Decoded d(e);
    expect(d.status == Status::success && d.report.range.valid && !d.report.range.reduced_blanking,
           "standard-blanking-only CVT range");
    bool no_rb = true;
    bool inside = true;
    for (uint32_t i = 1; i < d.count(); ++i) {
        const Timing t = d.at(i);
        no_rb = no_rb && !(t.h_total - t.width == 160 && t.h_sync_width == 32 && t.v_front_porch == 3);
        inside = inside && t.width <= 1920 && t.height <= 1080;
    }
    expect(no_rb, "a sink without reduced blanking gets no RB mode");
    expect(inside, "nothing larger than the native 1920x1080 is inferred");
    expect(d.find(1920, 1080, 50, EdidModeSource::inferred_cvt) != UINT32_MAX &&
               d.find(1600, 900, 50, EdidModeSource::inferred_cvt) != UINT32_MAX &&
               d.find(1280, 720, 50, EdidModeSource::inferred_cvt) != UINT32_MAX &&
               d.added(EdidModeSource::inferred_cvt) == 3,
           "CVT inference adds the 16:9 sizes at the preferred 50 Hz");
    Timing t = d.at(d.find(1920, 1080, 50, EdidModeSource::inferred_cvt));
    Timing expected{};
    cvt_timing(1920, 1080, 50, false, expected);
    expect(same(t, expected) && t.pixel_clock_khz == 141500, "CVT 1920x1080@50 is 141.50 MHz");
    expect(d.find(1600, 900, 60, EdidModeSource::inferred_dmt) != UINT32_MAX &&
               d.find(1600, 900, 60, EdidModeSource::inferred_cvt) == UINT32_MAX,
           "a 60 Hz size DMT already covers is not generated again by CVT");

    // EDID 1.3 with a range descriptor but no continuous-frequency bit.
    Edid f(3, 0x80, 0x0a);
    f.dtd(0, vic16);
    f.descriptor(1, 0xfd, {0x00, 49, 61, 15, 90, 23, 0x00, 0x0a, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20});
    f.finish();
    const Decoded g(f);
    expect(g.status == Status::success && g.count() == 1 && g.report.range.valid && !g.report.range.cvt,
           "no continuous frequency: nothing inferred");

    // A reserved offset pattern (01) invalidates the range descriptor.
    Edid h(4, 0xa5, 0x07);
    h.dtd(0, vic16);
    h.descriptor(1, 0xfd, {0x01, 49, 61, 15, 90, 23, 0x04, 0x11, 0x00, 0x00, 0x40, 0x28, 0x00, 50});
    h.finish();
    const Decoded k(h);
    expect(k.status == Status::success && !k.report.range.valid && k.count() == 1 && k.report.unsupported == 1,
           "a reserved range offset pattern is refused, not interpreted");

    // Range offsets (E-EDID 1.4 byte 4 = 10b: maximum vertical rate + 255 Hz only).
    Edid q(4, 0xa5, 0x06);
    q.dtd(0, vic16);
    q.descriptor(1, 0xfd, {0x02, 48, 45, 30, 120, 30, 0x01, 0x0a, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20});
    q.finish();
    const Decoded u(q);
    expect(u.status == Status::success && u.report.range.valid && u.report.range.min_vertical_hz == 48 &&
               u.report.range.max_vertical_hz == 300 && u.report.range.max_horizontal_khz == 120,
           "a maximum-only vertical offset raises the maximum, not the minimum");

    // The maximum pixel clock alone excludes an otherwise covered, in-range DMT mode.
    Edid r(4, 0xa5, 0x07);
    Timing wide{};
    cvt_timing(2560, 1600, 60, true, wide);
    r.dtd(0, wide);
    r.descriptor(1, 0xfd, {0x00, 50, 61, 30, 120, 15, 0x01, 0x0a, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20});
    r.finish();
    const Decoded w(r);
    bool clocked = true;
    for (uint32_t i = 1; i < w.count(); ++i) {
        clocked = clocked && w.at(i).pixel_clock_khz <= 150000;
    }
    expect(w.status == Status::success && w.added(EdidModeSource::inferred_dmt) > 0 && clocked &&
               w.find(1600, 1200, 60, EdidModeSource::inferred_dmt) == UINT32_MAX &&
               w.find(1280, 1024, 60, EdidModeSource::inferred_dmt) != UINT32_MAX,
           "1600x1200@60 (162 MHz) exceeds the 150 MHz range limit; 1280x1024@60 is inferred");

    // A range without a declared pixel clock infers nothing.
    Edid m(4, 0xa5, 0x07);
    m.dtd(0, vic16);
    m.descriptor(1, 0xfd, {0x00, 49, 61, 15, 90, 0, 0x01, 0x0a, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20});
    m.finish();
    const Decoded n(m);
    expect(n.status == Status::success && n.report.range.valid && n.count() == 1,
           "no maximum pixel clock: nothing inferred");
}

// ---- DisplayPort mode list with the real link budget ----

void dp_mode_list() {
    const Edid e = dp_monitor();
    Decoded d(e);
    rx7900::output::LinkCapabilities hbr{};
    expect(rx7900::output::dp_link_budget(0x0a, 2, true, hbr) == rx7900::output::DpProbeStatus::success,
           "HBR x2 budget");
    struct Listed final {
        Timing timing;
        uint32_t millihz;
    };
    std::vector<Listed> list(48);
    uint32_t count = 0;
    count = select_mode_list(
        d.sink.detailed_modes, count, 48,
        [&](uint32_t slot) { return listed_identity(list[slot].timing, list[slot].millihz); },
        [&](uint32_t slot, uint32_t index, const Timing& timing, uint32_t millihz) {
            Timing admitted{};
            if (rx7900::output::prepare_output_mode(d.sink, index, hbr, admitted) != rx7900::output::Status::success) {
                return false;
            }
            list[slot] = {timing, millihz};
            return true;
        });
    bool unique = true;
    bool within = true;
    for (uint32_t i = 0; i < count; ++i) {
        within = within && static_cast<uint64_t>(list[i].timing.pixel_clock_khz) * 24000u <= hbr.usable_payload_bits_per_second;
        for (uint32_t j = 0; j < i; ++j) {
            unique = unique && !(list[j].timing.width == list[i].timing.width &&
                                 list[j].timing.height == list[i].timing.height &&
                                 (list[j].millihz + 500) / 1000 == (list[i].millihz + 500) / 1000);
        }
    }
    expect(count > 10 && unique && within,
           "DP list: every mode fits HBR x2 at 24 bpp and no size/refresh repeats");
    expect(list[0].timing.width != 2560, "the 241.5 MHz native mode does not fit HBR x2 and is skipped");
    uint32_t capped = 0;
    capped = select_mode_list(
        d.sink.detailed_modes, capped, 4, [&](uint32_t slot) { return listed_identity(list[slot].timing, list[slot].millihz); },
        [&](uint32_t slot, uint32_t, const Timing& timing, uint32_t millihz) {
            list[slot] = {timing, millihz};
            return true;
        });
    expect(capped == 4, "the list stops at its capacity");
}

// ---- boundaries ----

void boundaries() {
    const Edid e = dp_monitor();
    ModeCatalog catalog{};
    EdidModeSource sources[mode_capacity]{};
    EdidModeReport report{};
    report.duplicates = 77;
    std::vector<uint8_t> bytes = e.bytes;
    bytes.resize(bytes.size() + 1);
    expect(append_edid_modes(ByteView(bytes.data(), bytes.size()), catalog, sources, report) == Status::invalid_request &&
               catalog.count() == 0 && report.duplicates == 77,
           "a blob that is not whole blocks is refused untouched");
    bytes = e.bytes;
    bytes[0] = 0x01;
    expect(append_edid_modes(ByteView(bytes.data(), bytes.size()), catalog, sources, report) == Status::invalid_request,
           "a bad header is refused");
    bytes = e.bytes;
    bytes[126] = 1;
    expect(append_edid_modes(ByteView(bytes.data(), bytes.size()), catalog, sources, report) == Status::invalid_request,
           "an extension count that does not match the blob is refused");
    expect(append_edid_modes(ByteView(nullptr, 0), catalog, sources, report) == Status::invalid_request,
           "an empty view is refused");

    // CVT codes with reserved bits set, or without a rate, are not interpreted.
    {
        Edid c(4, 0xa5, 0x06);
        c.dtd(0, vic16);
        c.descriptor(1, 0xf8, {0x00, 0x01, 0x57, 0x29, 0x08, 0x57, 0x28, 0x88, 0x57, 0x28, 0x60});
        c.finish();
        const Decoded d(c);
        expect(d.status == Status::success && d.count() == 1 && d.report.unsupported == 3,
               "CVT codes with reserved aspect/rate bits or no rate add nothing");
        c.descriptor(1, 0xf8, {0x00, 0x02, 0x57, 0x28, 0x08});
        c.finish();
        const Decoded v(c);
        expect(v.status == Status::success && v.count() == 1 && v.report.unsupported == 1,
               "an unknown CVT descriptor version is refused");
    }

    // A nearly full catalog: everything beyond capacity is counted as dropped.
    ModeCatalog full{};
    for (uint32_t i = 0; i < 60; ++i) {
        Timing t{};
        cvt_timing(800 + i * 8, 600, 60, false, t);
        uint32_t index = 0;
        full.append(t, index);
    }
    for (uint32_t i = 0; i < mode_capacity; ++i) {
        sources[i] = EdidModeSource::detailed;
    }
    expect(append_edid_modes(e.view(), full, sources, report) == Status::success && full.count() == mode_capacity &&
               report.dropped > 0 && sources[59] == EdidModeSource::detailed &&
               sources[60] != EdidModeSource::detailed,
           "a full catalog drops (and counts) the rest; caller entries keep their source");
}

} // namespace

int main() {
    cvt_and_dmt();
    vic_tables();
    monitor_modes();
    tv_modes();
    displayid_modes();
    range_inference();
    dp_mode_list();
    boundaries();
    return checks.finish("edid_modes");
}

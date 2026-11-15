/*
Module: rx7900.display.edid_modes
Purpose: EDID video modes beyond detailed timing descriptors: CVT codes, standard and established timings, CTA/HDMI VICs, DisplayID detailed timings and range-limit inference.
Created: 2026-10-24
Architecture: Allocation-free display core beside hdmi_stream (whose CEA and
HDMI VIC tables it reuses) and the shared ModeCatalog of modes.hpp. The
decoding is this project's own implementation of the published standards:
VESA E-EDID 1.4 (sections 3.6, 3.8, 3.9, 3.10.3.3/3.10.3.8/3.10.3.9 and
3.10.3.10), VESA DMT 1.0 rev 13 timing values, VESA CVT 1.2 (standard CRT
blanking and reduced blanking v1, progressive, no margins), CTA-861-G 7.5.1
(short video descriptors), HDMI 1.4b 8.3.2 (HDMI_VIC list) and VESA DisplayID
1.3 type I / DisplayID 2.0 type VII detailed timings. Linux v6.12
drm_edid.c (drm_edid_connector_add_modes source order, valid_inferred_mode
and mode_in_range rules, the 1366x768 standard-timing quirk) and drm_modes.c
(drm_cvt_mode) are behavioural references only; no Linux code is used. The
DMT values were transcribed by a scratch script from the pinned drm_edid.c
drm_dmt_modes data (VESA DMT) with DMT 0x52 sync polarity corrected to the
DMT/CTA +H +V. Interlaced, pixel-repeated, GTF-only and stereo timings are
not produced.
*/
#pragma once

#include "byte_view.hpp"
#include "modes.hpp"

namespace rx7900::display {

enum class EdidModeSource : uint8_t {
    detailed,       // EDID/CTA detailed timing descriptors (already in the catalog)
    cvt_code,       // EDID 1.4 CVT 3-byte timing codes (display descriptor 0xF8)
    standard,       // standard timings (bytes 38..53 and display descriptor 0xFA)
    established,    // established timings I/II (bytes 35..37) and III (descriptor 0xF7)
    cta_vic,        // CTA-861 Video Data Block short video descriptors
    hdmi_vic,       // HDMI 1.4b VSDB HDMI_VIC 1..4
    displayid,      // DisplayID type I (tag 0x03) / type VII (tag 0x22) detailed timings
    inferred_dmt,   // DMT modes inside the range limits (continuous frequency sink)
    inferred_cvt    // CVT/CVT-RB modes inside EDID 1.4 CVT range limits
};
constexpr uint32_t edid_mode_source_count = 9;

// The EDID display range limits descriptor (tag 0xFD) as this module uses it.
struct EdidRangeLimits final {
    bool valid = false;
    uint32_t min_vertical_hz = 0;
    uint32_t max_vertical_hz = 0;
    uint32_t min_horizontal_khz = 0;
    uint32_t max_horizontal_khz = 0;
    // 0: not declared.
    uint32_t max_pixel_clock_khz = 0;
    // Byte 10 is the CVT support code (EDID 1.4 only).
    bool cvt = false;
    // CVT: 0 = no limit.
    uint32_t max_active_pixels = 0;
    // CVT byte 14 (bit 7 4:3, 6 16:9, 5 16:10, 4 5:4, 3 15:9).
    uint8_t aspect_ratios = 0;
    bool standard_blanking = false;
    bool reduced_blanking = false;
    uint32_t preferred_refresh_hz = 0;
};

struct EdidModeReport final {
    uint32_t added[edid_mode_source_count]{};
    // Already present in the catalog (from any source).
    uint32_t duplicates = 0;
    // Interlaced, pixel-repeated, GTF-only, unknown codes and malformed
    // descriptors or DisplayID sections; nothing is guessed for them.
    uint32_t unsupported = 0;
    // The catalog was full.
    uint32_t dropped = 0;
    uint8_t revision = 0;
    // Feature byte bit 0: continuous frequency (1.4) / default GTF (1.3).
    bool continuous_frequency = false;
    EdidRangeLimits range{};
};

// Appends, in Linux drm_edid_connector_add_modes order after the detailed
// timings the caller already placed in catalog: CVT codes, standard timings,
// established timings (I, II, III), CTA VICs then HDMI VICs, DisplayID
// detailed timings and, for a continuous-frequency sink with a range limits
// descriptor, inferred DMT and CVT modes. Every mode passes ModeCatalog's
// representability check; exact duplicates are skipped. An inferred mode is
// added only when no catalog mode has the same size and refresh and some
// catalog mode is at least as wide and as tall (Linux valid_inferred_mode),
// and only inside the range limits (vertical/horizontal rates, pixel clock,
// CVT active-pixel limit, reduced blanking only when the sink declares it).
// sources[i] is set for every appended index i; entries below the catalog
// count on entry are untouched. edid must be a whole EDID that
// output::parse_edid accepted (the 128-byte block size, the header and the
// 1.3/1.4 revision are rechecked; checksums are the caller's). Returns
// invalid_request without touching anything for a blob that fails those
// checks. Any IRQL with resident arguments; no allocation; uses well under
// 512 bytes of stack.
Status append_edid_modes(ByteView edid, ModeCatalog& catalog, EdidModeSource (&sources)[mode_capacity],
                         EdidModeReport& report) noexcept;

// VESA CVT 1.2 progressive timing without margins: reduced_blanking false is
// the standard CRT blanking formula (HSync negative, VSync positive),
// true is reduced blanking v1 (160-pixel blank, HSync positive, VSync
// negative). Width is rounded down to the 8-pixel cell. The vertical sync
// width (V_SYNC_RND) is vsync_lines when a source declares the aspect ratio
// (EDID CVT codes and standard timings: 4:3 4, 16:9 5, 16:10 6, 5:4 and 15:9
// 7), else it follows width:height exactly (otherwise 10, as Linux
// drm_cvt_mode does). Exact integer arithmetic; the pixel clock is floored
// to the 0.25 MHz step. unsupported_timing (output unchanged) for sizes or
// rates outside 64..16384 pixels, 64..8192 lines and 24..480 Hz, or a result
// the OTG cannot represent.
Status cvt_timing(uint32_t width, uint32_t height, uint32_t refresh_hz, bool reduced_blanking,
                  Timing& output, uint32_t vsync_lines = 0) noexcept;

// The VESA DMT timing with this size, rounded refresh and blanking style
// (reduced_blanking: the CVT-RB shape of Linux mode_is_rb). First table
// match wins; false (output unchanged) when DMT has none.
bool dmt_timing(uint32_t width, uint32_t height, uint32_t refresh_hz, bool reduced_blanking,
                Timing& output) noexcept;

// VIDEO_MODE_INFORMATION identity of a mode: size and Frequency (the refresh
// rounded to Hz, as the mode service reports it).
struct ListedModeIdentity final {
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t refresh_hz = 0;
};

inline ListedModeIdentity listed_identity(const Timing& timing, uint32_t refresh_millihz) noexcept {
    return {timing.width, timing.height, (refresh_millihz + 500u) / 1000u};
}

// The mode-list policy of the DCN 3.2 VideoPort mode service: catalog order
// (detailed timings, then the append_edid_modes sources in Linux order), one
// list entry per identity with the earliest candidate that validates
// winning, at most capacity entries. count entries are already listed (a
// caller seed such as the GOP mode); identity(i) returns entry i's identity;
// offer(slot, catalog_index, timing, refresh_millihz) validates a candidate
// (link budget, TMDS rules, bandwidth) and, returning true, has written it
// to list entry slot. Returns the new count. Any IRQL the callbacks allow.
template <typename Identity, typename Offer>
uint32_t select_mode_list(const ModeCatalog& catalog, uint32_t count, uint32_t capacity, Identity identity,
                          Offer offer) noexcept {
    for (uint32_t index = 0; index < catalog.count() && count < capacity; ++index) {
        Timing timing{};
        uint32_t millihz = 0;
        if (catalog.get(index, timing) != Status::success || refresh_millihz(timing, millihz) != Status::success) {
            continue;
        }
        const ListedModeIdentity candidate = listed_identity(timing, millihz);
        bool duplicate = false;
        for (uint32_t slot = 0; slot < count && !duplicate; ++slot) {
            const ListedModeIdentity other = identity(slot);
            duplicate = other.width == candidate.width && other.height == candidate.height &&
                        other.refresh_hz == candidate.refresh_hz;
        }
        if (!duplicate && offer(count, index, timing, millihz)) {
            ++count;
        }
    }
    return count;
}

} // namespace rx7900::display

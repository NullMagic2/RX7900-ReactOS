/*
Module: rx7900.tests.hdmi_stream
Purpose: Verify HDMI sink parsing, VIC matching, TMDS limits and info packets against Linux v6.12.
Created: 2026-10-24
Architecture: Host test over src/display/hdmi_stream. Expected packet bytes were
worked out by hand from the pinned Linux sources (dc_resource.c set_avi_info_frame
with the union hdmi_info_packet layout of set_mode_types.h and the enum values of
transform.h/dc_hw_types.h; info_packet.c mod_build_hf_vsif_infopacket and
mod_build_vsc_infopacket; drm_edid.c drm_parse_hdmi_vsdb_video and
drm_parse_hdmi_forum_scds) and are written out literally, not recomputed by the
code under test. EDIDs are synthetic. This is not hardware evidence.
*/
#include <cstdio>
#include <cstring>
#include <vector>

#include "asic_test_device.hpp"
#include "hdmi_stream.hpp"

namespace {

using namespace rx7900::display;

rx7900_test::Checks checks;

void expect(bool condition, const char* contract) { checks.expect(condition, contract); }

constexpr uint32_t sync = positive_hsync | positive_vsync;
constexpr Timing mode_480{25175, 640, 800, 16, 96, 480, 525, 10, 2, 0};
constexpr Timing mode_1080{148500, 1920, 2200, 88, 44, 1080, 1125, 4, 5, sync};
constexpr Timing mode_1080_5994{148352, 1920, 2200, 88, 44, 1080, 1125, 4, 5, sync};
constexpr Timing mode_2160_30{297000, 3840, 4400, 176, 88, 2160, 2250, 8, 10, sync};
constexpr Timing mode_2160{594000, 3840, 4400, 176, 88, 2160, 2250, 8, 10, sync};
constexpr Timing mode_1600{106184, 1600, 1668, 24, 27, 900, 1061, 85, 51, sync};

// Base block (contents irrelevant to the CTA walk) plus one CTA extension built
// from the given data blocks.
std::vector<uint8_t> edid_with(const std::vector<std::vector<uint8_t>>& blocks, uint8_t revision = 3) {
    std::vector<uint8_t> edid(256, 0);
    const uint8_t header[]{0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00};
    std::memcpy(edid.data(), header, sizeof(header));
    edid[126] = 1;
    edid[128] = 0x02;
    edid[129] = revision;
    size_t at = 132;

    for (const auto& block : blocks) {
        std::memcpy(&edid[at], block.data(), block.size());
        at += block.size();
    }

    edid[130] = static_cast<uint8_t>(at - 128);
    return edid;
}

const std::vector<uint8_t> video_db{0x44, 0x90, 0x01, 0x5f, 0x61};
// HDMI 1.x VSDB: OUI 00-0C-03, CEC 1.0.0.0, no DC flags, 300 MHz, HDMI_Video_present.
const std::vector<uint8_t> hdmi_vsdb{0x68, 0x03, 0x0c, 0x00, 0x10, 0x00, 0x00, 0x3c, 0x20};
// HDMI 1.x VSDB without the video byte and max TMDS (length 5).
const std::vector<uint8_t> short_vsdb{0x65, 0x03, 0x0c, 0x00, 0x10, 0x00};
// HF-VSDB: OUI C4-5D-D8, version 1, 600 MHz, SCDC present + LTE_340 scramble.
const std::vector<uint8_t> hf_vsdb{0x67, 0xd8, 0x5d, 0xc4, 0x01, 0x78, 0x88, 0x00};
// HF-VSDB with SCDC but no Max_TMDS_Char_Rate: Linux leaves scrambling unsupported.
const std::vector<uint8_t> hf_vsdb_no_rate{0x67, 0xd8, 0x5d, 0xc4, 0x01, 0x00, 0x80, 0x00};

HdmiSinkInfo parse(const std::vector<uint8_t>& edid) {
    HdmiSinkInfo info{};
    expect(parse_hdmi_sink(rx7900::ByteView(edid.data(), edid.size()), info) == HdmiStatus::success,
           "EDID parses");
    return info;
}

void sink_parsing() {
    const HdmiSinkInfo hdmi2 = parse(edid_with({video_db, hdmi_vsdb, hf_vsdb}));
    expect(hdmi2.is_hdmi && hdmi2.has_hdmi_infoframe, "HDMI VSDB + HF-VSDB: HDMI with infoframes");
    expect(hdmi2.max_tmds_clock_khz == 600000, "HF-VSDB 600 MHz raises the VSDB 300 MHz");
    expect(hdmi2.scdc_supported && hdmi2.scrambling_supported && hdmi2.scrambling_low_rates,
           "SCDC present, scrambling and LTE_340 low rates");
    expect(hdmi2_sink(hdmi2), "SCDC makes an HDMI 2 sink");
    expect(hdmi2.cea_revision == 3, "CEA revision");
    expect(hdmi2.vdb_vics[0] == ((1u << 16) | (1u << 1)) && hdmi2.vdb_vics[2] == (1u << 31) &&
               hdmi2.vdb_vics[3] == (1u << 1),
           "SVD 0x90 is native VIC 16; VICs 1, 95, 97 listed");

    const HdmiSinkInfo hdmi1 = parse(edid_with({video_db, hdmi_vsdb}));
    expect(hdmi1.is_hdmi && hdmi1.has_hdmi_infoframe && hdmi1.max_tmds_clock_khz == 300000 &&
               !hdmi1.scdc_supported && !hdmi2_sink(hdmi1),
           "HDMI 1.4 sink: 300 MHz, no SCDC");

    const HdmiSinkInfo short_one = parse(edid_with({short_vsdb}));
    expect(short_one.is_hdmi && !short_one.has_hdmi_infoframe && short_one.max_tmds_clock_khz == 0,
           "five-byte VSDB: HDMI without a declared TMDS limit or HDMI video");

    const HdmiSinkInfo no_rate = parse(edid_with({hdmi_vsdb, hf_vsdb_no_rate}));
    expect(no_rate.scdc_supported && !no_rate.scrambling_supported && no_rate.max_tmds_clock_khz == 300000,
           "HF-VSDB without a TMDS rate: SCDC but no scrambling, VSDB limit kept");

    const HdmiSinkInfo dvi = parse(edid_with({video_db}));
    expect(!dvi.is_hdmi && !dvi.has_hdmi_infoframe, "no HDMI VSDB: DVI");

    // Revision 1/2 extensions carry no data block collection.
    const HdmiSinkInfo old = parse(edid_with({hdmi_vsdb}, 2));
    expect(!old.is_hdmi && old.cea_revision == 2, "revision 2 CTA extension: no data blocks walked");

    // A block running past the DTD offset ends the walk; earlier blocks count.
    std::vector<uint8_t> overlong = edid_with({video_db, hdmi_vsdb});
    overlong[130] = static_cast<uint8_t>(overlong[130] - 2);
    const HdmiSinkInfo cut = parse(overlong);
    expect(cut.vdb_vics[0] != 0 && !cut.is_hdmi, "an overlong VSDB is ignored, the VDB before it is kept");

    // Y420CMDB with an empty map: every mode is 4:2:0 capable (HDMI 2 sink).
    const HdmiSinkInfo y420 = parse(edid_with({hdmi_vsdb, {0xe1, 0x0f}}));
    expect(y420.ycbcr420 && hdmi2_sink(y420), "empty Y420CMDB marks 4:2:0 support");

    HdmiSinkInfo untouched{};
    untouched.cea_revision = 9;
    const std::vector<uint8_t> partial(200, 0);
    expect(parse_hdmi_sink(rx7900::ByteView(partial.data(), partial.size()), untouched) ==
                   HdmiStatus::invalid_argument &&
               untouched.cea_revision == 9,
           "a partial block is refused with the output unchanged");
}

void vic_matching() {
    expect(match_cea_vic(mode_480) == 1, "640x480@60 is VIC 1");
    expect(match_cea_vic(mode_1080) == 16, "1080p60 is VIC 16");
    expect(match_cea_vic(mode_1080_5994) == 16, "1080p59.94 matches the alternate clock of VIC 16");
    expect(match_cea_vic(mode_2160_30) == 95 && match_hdmi_vic(mode_2160_30) == 1,
           "2160p30 is CEA VIC 95 and HDMI VIC 1");
    expect(match_cea_vic(mode_2160) == 97 && match_hdmi_vic(mode_2160) == 0, "2160p60 is VIC 97, no HDMI VIC");
    expect(match_cea_vic(mode_1600) == 0, "a non-CEA DTD has no VIC");
    Timing negative = mode_1080;
    negative.flags = 0;
    expect(match_cea_vic(negative) == 0, "sync polarity mismatch: no VIC");

    Timing near = mode_1080;
    near.pixel_clock_khz = 148498;
    expect(fixup_detailed_cea_clock(near) == 148500, "a DTD 2 kHz off VIC 16 takes the CEA clock");
    near.pixel_clock_khz = 148355;
    expect(fixup_detailed_cea_clock(near) == 148352, "a DTD 3 kHz off 59.94 takes the alternate clock");
    near.pixel_clock_khz = 148000;
    expect(fixup_detailed_cea_clock(near) == 148000, "a DTD far from every CEA clock is kept");

    expect(drm_vrefresh_hz(mode_1080) == 60 && drm_vrefresh_hz(mode_1080_5994) == 60 &&
               drm_vrefresh_hz(mode_2160_30) == 30,
           "drm_mode_vrefresh rounds to the closest Hz");
}

bool packet_is(const InfoPacket& p, uint8_t hb0, uint8_t hb1, uint8_t hb2, uint8_t hb3,
               const std::vector<uint8_t>& sb) {
    if (!p.valid || p.hb0 != hb0 || p.hb1 != hb1 || p.hb2 != hb2 || p.hb3 != hb3) {
        return false;
    }

    for (size_t i = 0; i < sizeof(p.sb); ++i) {
        const uint8_t want = i < sb.size() ? sb[i] : 0;

        if (p.sb[i] != want) {
            std::fprintf(stderr, "sb[%zu] = 0x%02x, Linux 0x%02x\n", i, p.sb[i], want);
            return false;
        }
    }

    return true;
}

void info_packets() {
    // sb1 = S0_S1 underscan (2) | B0_B1 both bars (3 << 2) | A0 (1 << 4) = 0x1e;
    // sb2 = R0_R3 same as picture (8); sb3 = ITC (0x80); sb4 = VIC; bars:
    // bottom = v_total + 1, right = h_total + 1; sb0 = 0x100 - sum(0x82, 2, 13, sb1..13).
    InfoPacket avi{};
    build_avi_infoframe(mode_1080, 16, 0, avi);
    expect(packet_is(avi, 0x82, 0x02, 0x0d, 0x00,
                     {0xae, 0x1e, 0x08, 0x80, 0x10, 0x00, 0x00, 0x00, 0x66, 0x04, 0x00, 0x00, 0x99, 0x08}),
           "1080p60 AVI infoframe");
    build_avi_infoframe(mode_480, 1, 0, avi);
    expect(packet_is(avi, 0x82, 0x02, 0x0d, 0x00,
                     {0x94, 0x1e, 0x08, 0x80, 0x01, 0x00, 0x00, 0x00, 0x0e, 0x02, 0x00, 0x00, 0x21, 0x03}),
           "640x480 AVI infoframe");
    build_avi_infoframe(mode_2160, 97, 0, avi);
    expect(packet_is(avi, 0x82, 0x02, 0x0d, 0x00,
                     {0x53, 0x1e, 0x08, 0x80, 0x61, 0x00, 0x00, 0x00, 0xcb, 0x08, 0x00, 0x00, 0x31, 0x11}),
           "2160p60 AVI infoframe");
    build_avi_infoframe(mode_2160_30, 95, 1, avi);
    expect(packet_is(avi, 0x82, 0x02, 0x0d, 0x00,
                     {0xb4, 0x1e, 0x08, 0x80, 0x00, 0x00, 0x00, 0x00, 0xcb, 0x08, 0x00, 0x00, 0x31, 0x11}),
           "an HDMI VIC clears the AVI VIC");
    build_avi_infoframe(mode_1080, 200, 0, avi);
    expect(avi.hb1 == 3 && avi.sb[4] == 200, "VIC >= 128 uses AVI version 3");

    InfoPacket vsif{};
    build_hdmi_vsif(mode_2160_30, 1, vsif);
    expect(packet_is(vsif, 0x81, 0x01, 0x05, 0x00, {0x49, 0x03, 0x0c, 0x00, 0x20, 0x01}),
           "HDMI VIC 1 vendor infoframe");
    build_hdmi_vsif(mode_1080, 0, vsif);
    expect(!vsif.valid, "no VSIF for a 2D mode without HDMI VIC");

    InfoPacket vsc{};
    build_dp_vsc(true, vsc);
    std::vector<uint8_t> sb(19, 0);
    sb[17] = 1;
    expect(packet_is(vsc, 0x00, 0x07, 0x05, 0x13, sb), "DP VSC revision 5 colorimetry SDP, RGB 8 bpc sRGB");
    build_dp_vsc(false, vsc);
    expect(!vsc.valid, "no VSC SDP without the DPRX colorimetry capability");
}

void tmds_plans() {
    const HdmiSinkInfo hdmi2 = parse(edid_with({video_db, hdmi_vsdb, hf_vsdb}));
    const HdmiSinkInfo hdmi1 = parse(edid_with({video_db, hdmi_vsdb}));
    const HdmiSinkInfo hdmi1_no_vdb = parse(edid_with({hdmi_vsdb}));
    const HdmiSinkInfo dvi = parse(edid_with({video_db}));
    TmdsStreamPlan plan{};

    expect(plan_tmds_stream(mode_1080, hdmi2, plan) == HdmiStatus::success && plan.signal == TmdsSignal::hdmi &&
               plan.tmds_khz == 148500 && !plan.scramble_above_340 && !plan.lte_340_scramble && plan.vic == 16 &&
               plan.hdmi_vic == 0 && plan.avi.valid && plan.avi.sb[0] == 0xae && !plan.vsif.valid,
           "1080p60 HDMI: VIC 16, AVI, no scrambling");
    expect(plan_tmds_stream(mode_2160, hdmi2, plan) == HdmiStatus::success && plan.scramble_above_340 &&
               plan.vic == 97 && plan.avi.sb[0] == 0x53,
           "2160p60 HDMI 2: scrambled, VIC 97");
    expect(plan_tmds_stream(mode_2160, hdmi1, plan) == HdmiStatus::exceeds_tmds_limit,
           "2160p60 beyond the sink's 300 MHz");
    HdmiSinkInfo no_limit = hdmi1;
    no_limit.max_tmds_clock_khz = 0;
    expect(plan_tmds_stream(mode_2160, no_limit, plan) == HdmiStatus::needs_scrambling,
           "594 MHz needs SCDC scrambling on the sink");
    Timing at_340 = mode_2160_30;
    at_340.pixel_clock_khz = 340000;
    expect(plan_tmds_stream(at_340, no_limit, plan) == HdmiStatus::success && plan.scramble_above_340,
           "exactly 340 MHz scrambles (encoder >= rule) without requiring SCDC");
    Timing above_600 = mode_2160;
    above_600.pixel_clock_khz = 600001;
    expect(plan_tmds_stream(above_600, hdmi2, plan) == HdmiStatus::exceeds_tmds_limit,
           "DCN 3.2 max_hdmi_pixel_clock 600 MHz");

    expect(plan_tmds_stream(mode_2160_30, hdmi1, plan) == HdmiStatus::success && plan.vic == 0 &&
               plan.hdmi_vic == 1 && plan.vsif.valid && plan.vsif.sb[0] == 0x49 && plan.avi.sb[4] == 0,
           "2160p30 on HDMI 1.4: HDMI VIC 1 in the VSIF, AVI VIC 0");
    expect(plan_tmds_stream(mode_2160_30, hdmi1_no_vdb, plan) == HdmiStatus::success && plan.hdmi_vic == 1,
           "HDMI VIC does not need a VDB entry");
    HdmiSinkInfo no_infoframe = hdmi1_no_vdb;
    no_infoframe.has_hdmi_infoframe = false;
    expect(plan_tmds_stream(mode_2160_30, no_infoframe, plan) == HdmiStatus::success && plan.hdmi_vic == 0 &&
               plan.vic == 0 && !plan.vsif.valid,
           "no HDMI video: no HDMI VIC, unlisted VIC 95 dropped on an HDMI 1.4 sink");
    expect(plan_tmds_stream(mode_2160, hdmi2, plan) == HdmiStatus::success && plan.vic == 97,
           "an HDMI 2 sink receives VICs above 64");

    expect(plan_tmds_stream(mode_1080, dvi, plan) == HdmiStatus::success &&
               plan.signal == TmdsSignal::dvi_single_link && !plan.avi.valid && !plan.scramble_above_340,
           "1080p60 DVI single link: no infoframes");
    Timing at_165 = mode_1080;
    at_165.pixel_clock_khz = 165000;
    expect(plan_tmds_stream(at_165, dvi, plan) == HdmiStatus::success, "165 MHz single-link DVI");
    at_165.pixel_clock_khz = 165001;
    expect(plan_tmds_stream(at_165, dvi, plan) == HdmiStatus::exceeds_tmds_limit, "above 165 MHz on DVI");

    TmdsStreamPlan untouched{};
    untouched.tmds_khz = 7;
    Timing broken = mode_1080;
    broken.h_total = broken.width;
    expect(plan_tmds_stream(broken, hdmi2, untouched) == HdmiStatus::unsupported_timing && untouched.tmds_khz == 7,
           "an unprogrammable timing is refused with the output unchanged");
}

} // namespace

int main() {
    sink_parsing();
    vic_matching();
    info_packets();
    tmds_plans();
    return checks.finish("hdmi_stream");
}

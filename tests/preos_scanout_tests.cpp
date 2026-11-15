/*
Module: rx7900.tests.preos_scanout
Purpose: Verify pre-OS DCN3.2 scanout snapshots, adoption rules and reservation coverage.
Created: 2026-10-22
Architecture: GOP-like register images are written into the shared synthetic
Navi31 register file (tests/asic_test_device.hpp) with the pinned Linux v6.12
dcn_3_2_0_offset.h indices, read back through the production RegisterIo path and
decoded. Timings are standard CEA/CVT values; 3840x2160 HDMI matches the
user's monitor as read-only Linux sysfs reports it (card1-HDMI-A-2). This is not
hardware evidence.
*/
#include <memory>

#include "asic_test_device.hpp"
#include "preos_scanout.hpp"

namespace {

using namespace rx7900::display;
using rx7900::hardware::VramLocation;
using rx7900::hardware::VramReservationMap;

rx7900_test::Checks checks;

void expect(bool condition, const char* contract) { checks.expect(condition, contract); }

constexpr uint64_t mib = 1ULL << 20;
constexpr uint64_t window = 256 * mib;
// DMU 3.2.0 segment bases in the shared synthetic discovery (the board's values).
constexpr uint32_t dccg = 0xc0;
constexpr uint32_t dcn = 0x34c0;
constexpr uint32_t mpc = 0x9000;

// dcn_3_2_0_offset.h instance-0 indices and the per-instance strides they show.
constexpr uint32_t otg_stride = 0x80;
constexpr uint32_t hubp_stride = 0xdc;
constexpr uint32_t dscl_stride = 0x16b;
constexpr uint32_t odm_stride = 0x10;
constexpr uint32_t mpcc_stride = 0x15;
constexpr uint32_t vga_control[4]{0x000c, 0x000e, 0x0038, 0x0039};

struct Mode final {
    Timing timing;
    uint32_t pitch_pixels;
};

// CEA-861 1080p60, CVT-RB 1440p60 (241.5 MHz), CEA-861 2160p60 (HDMI 594 MHz).
constexpr Mode mode_1080{{148500, 1920, 2200, 88, 44, 1080, 1125, 4, 5, positive_hsync | positive_vsync}, 1920};
constexpr Mode mode_1440{{241500, 2560, 2720, 48, 32, 1440, 1481, 3, 5, positive_hsync}, 2560};
constexpr Mode mode_2160{{594000, 3840, 4400, 176, 88, 2160, 2250, 8, 10, positive_hsync | positive_vsync}, 3840};

class Gop final {
public:
    std::unique_ptr<rx7900_test::SimulatedNavi31> device = std::make_unique<rx7900_test::SimulatedNavi31>();
    unsigned writes = 0;

    uint32_t& reg(uint32_t base, uint32_t index) { return device->reg(base + index); }
    uint32_t& otg(uint32_t instance, uint32_t index) { return reg(dcn, index + instance * otg_stride); }
    uint32_t& hubp(uint32_t instance, uint32_t index) { return reg(dcn, index + instance * hubp_stride); }
    uint32_t& dscl(uint32_t instance, uint32_t index) { return reg(dcn, index + instance * dscl_stride); }
    uint32_t& mpcc(uint32_t instance, uint32_t index) { return reg(mpc, index + instance * mpcc_stride); }

    Gop() {
        const uint64_t fb = rx7900_test::fb_base >> 24;
        reg(dcn, 0x0475) = static_cast<uint32_t>(fb);
        reg(dcn, 0x0476) = static_cast<uint32_t>(fb + (rx7900_test::memsize_mib >> 4) - 1);
        reg(dcn, 0x0477) = 0x20; // matches the synthetic MMHUB FB_OFFSET
        // Idle MPC trees: every mux and selector disconnected.
        for (uint32_t index = 0; index < 4; ++index) {
            reg(mpc, 0x03d8 + index * 4) = 0xf;
            mpcc(index, 0x0000) = 0xf;
            mpcc(index, 0x0001) = 0xf;
            mpcc(index, 0x0002) = 0xf;
        }
    }

    // What a VBIOS/GOP leaves for one linear ARGB8888 plane: OTG timing as
    // optc1_program_timing writes it, ODM bypass, one MPCC, HUBP/DPP bypass.
    void scanout(uint32_t otg_index, uint32_t pipe, const Mode& mode, uint64_t vram_offset,
                 bool dp_dto) {
        OptcTiming fields{};
        prepare_optc_timing(mode.timing, fields);
        otg(otg_index, 0x1b41) = 0x00010001u;
        otg(otg_index, 0x1b85) = 0x00000101u;
        otg(otg_index, 0x1b2a) = fields.h_total_minus_one;
        otg(otg_index, 0x1b2b) = fields.h_blank_start | (fields.h_blank_end << 16);
        otg(otg_index, 0x1b2c) = fields.h_sync_end << 16;
        otg(otg_index, 0x1b2d) = fields.h_sync_negative;
        otg(otg_index, 0x1b2f) = fields.v_total_minus_one;
        otg(otg_index, 0x1b36) = fields.v_blank_start | (fields.v_blank_end << 16);
        otg(otg_index, 0x1b37) = fields.v_sync_end << 16;
        otg(otg_index, 0x1b38) = fields.v_sync_negative;
        reg(dcn, 0x1acb + otg_index * odm_stride) = otg_index << 16;
        // DCCG: DP DTO with MODULO = DPREFCLK (600 MHz) and PHASE = pixel clock in Hz.
        reg(dccg, 0x0080 + otg_index * 4) = dp_dto ? 0x00000090u : 0;
        reg(dccg, 0x0081 + otg_index * 4) = dp_dto ? mode.timing.pixel_clock_khz * 1000u : 0;
        reg(dccg, 0x0082 + otg_index * 4) = dp_dto ? 600000000u : 0;
        reg(mpc, 0x03d8 + otg_index * 4) = pipe; // OPP otg_index -> MPCC pipe
        mpcc(pipe, 0x0000) = pipe;
        mpcc(pipe, 0x0001) = 0xf;
        mpcc(pipe, 0x0002) = otg_index;
        mpcc(pipe, 0x0003) = 0xff000000u | (0xffu << 16) | (2u << 4) | 2u;
        hubp(pipe, 0x05f4) = 0x00f00001u;
        hubp(pipe, 0x05f3) = otg_index << 4;
        hubp(pipe, 0x05e5) = 8;
        hubp(pipe, 0x05e7) = 0;
        hubp(pipe, 0x05e9) = 0;
        hubp(pipe, 0x05ea) = (mode.timing.height << 16) | mode.timing.width;
        hubp(pipe, 0x0607) = mode.pitch_pixels - 1;
        const uint64_t mc = rx7900_test::fb_base + vram_offset;
        hubp(pipe, 0x060a) = static_cast<uint32_t>(mc);
        hubp(pipe, 0x060b) = static_cast<uint32_t>(mc >> 32);
        hubp(pipe, 0x061a) = 0;
        hubp(pipe, 0x0609) = 0;
        hubp(pipe, 0x066c) = (3u << 22) | (2u << 20) | (1u << 18);
        dscl(pipe, 0x0cfb) = 0;
        dscl(pipe, 0x0d11) = 0;
        dscl(pipe, 0x0d12) = (mode.timing.height << 16) | mode.timing.width;
    }

    // A RegisterIo whose write callback counts and refuses: the snapshot must not write.
    static bool read(void* context, uint32_t offset, uint32_t& value) noexcept {
        auto& self = *static_cast<Gop*>(context);
        return rx7900_test::SimulatedNavi31::read(self.device.get(), offset, value);
    }

    static bool write(void* context, uint32_t, uint32_t) noexcept {
        ++static_cast<Gop*>(context)->writes;
        return false;
    }

    static void delay(void*, uint32_t) noexcept {}

    rx7900::hardware::RegisterIo io() noexcept {
        return {this, rx7900_test::aperture_bytes, read, write, delay};
    }

    PreosDcnRegisters registers() {
        const auto binary = rx7900_test::navi31_discovery();
        PreosDcnRegisters output;
        validate_preos_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes, output);
        return output;
    }

    VramLocation vram() {
        const auto binary = rx7900_test::navi31_discovery();
        rx7900::hardware::NbioRegisters nbio_registers;
        rx7900::hardware::Nbio nbio;
        VramLocation location{};
        rx7900::hardware::validate_nbio_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes,
                                                  nbio_registers);
        nbio.initialize(nbio_registers, device->io());
        nbio.vram_location(location);
        return location;
    }

    PreosStatus decode(AdoptedScanout& scanout, PreosReport& report, uint64_t visible = window) {
        PreosScanoutRegisters snapshot{};

        if (!read_preos_scanout(io(), registers(), snapshot)) {
            report = {};
            return PreosStatus::read_failure;
        }

        return decode_preos_scanout(snapshot, vram(), visible, scanout, report);
    }

    // Linux reservation map from the same registers (read_vbios_fb_bytes).
    VramReservationMap reservations() {
        const auto binary = rx7900_test::navi31_discovery();
        rx7900::hardware::DcnFbRegisters fb;
        uint32_t bytes = 0;
        rx7900::hardware::validate_dcn_fb_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes, fb);
        rx7900::hardware::read_vbios_fb_bytes(device->io(), fb, bytes);
        return map_for(bytes);
    }

    static VramReservationMap map_for(uint32_t fb_bytes) {
        rx7900::hardware::VramReservationInputs inputs{};
        inputs.vram_bytes = static_cast<uint64_t>(rx7900_test::memsize_mib) * mib;
        inputs.visible_vram_bytes = window;
        inputs.capability = rx7900::hardware::atom_capability_two_stage_training;
        inputs.reserved_fb_bytes = 28 * 1024 * 1024;
        inputs.display_hardware = true;
        inputs.vbios_fb_bytes = fb_bytes;
        VramReservationMap map{};
        rx7900::hardware::build_vram_reservation_map(inputs, map);
        return map;
    }
};

bool same_timing(const Timing& a, const Timing& b) {
    return a.pixel_clock_khz == b.pixel_clock_khz && a.width == b.width && a.h_total == b.h_total &&
           a.h_front_porch == b.h_front_porch && a.h_sync_width == b.h_sync_width &&
           a.height == b.height && a.v_total == b.v_total && a.v_front_porch == b.v_front_porch &&
           a.v_sync_width == b.v_sync_width && a.flags == b.flags;
}

void discovery() {
    const auto binary = rx7900_test::navi31_discovery();
    PreosDcnRegisters registers;
    expect(validate_preos_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes, registers) ==
               rx7900::hardware::DiscoveryStatus::validated &&
               registers.segment(1) == dccg && registers.segment(2) == dcn && registers.segment(3) == mpc,
           "DMU 3.2.0 segments 1-3 validated");
    expect(validate_preos_discovery({binary.data(), binary.size()}, 0x14000, registers) !=
               rx7900::hardware::DiscoveryStatus::validated && !registers.valid(),
           "an aperture missing the OTG/MPC registers is refused");
    const auto three = rx7900_test::discovery_with({{271, 3, 2, {0x12, 0xc0, 0x34c0}}});
    expect(validate_preos_discovery({three.data(), three.size()}, rx7900_test::aperture_bytes, registers) !=
               rx7900::hardware::DiscoveryStatus::validated,
           "DMU without the MPC segment refused");
    const auto other = rx7900_test::discovery_with({{271, 3, 1, {0x12, 0xc0, 0x34c0, 0x9000}}});
    expect(validate_preos_discovery({other.data(), other.size()}, rx7900_test::aperture_bytes, registers) !=
               rx7900::hardware::DiscoveryStatus::validated,
           "DCN 3.1 refused");
    Gop gop;
    PreosScanoutRegisters snapshot{};
    expect(!read_preos_scanout(gop.io(), registers, snapshot), "invalid token reads nothing");
}

void typical_modes() {
    {
        Gop gop;
        gop.scanout(0, 0, mode_1080, 0, true);
        AdoptedScanout scanout{};
        PreosReport report{};
        expect(gop.decode(scanout, report) == PreosStatus::adopted && gop.writes == 0,
               "1080p DP GOP adopted without a register write");
        expect(same_timing(scanout.timing, mode_1080.timing) && scanout.refresh_millihz == 60000 &&
                   scanout.clock_source == PreosClockSource::dp_dto,
               "1080p timing decoded exactly; DTO phase gives 148.5 MHz and 60 Hz");
        expect(scanout.stride_bytes == 7680 && scanout.bytes == 8294400 && scanout.vram_offset == 0 &&
                   scanout.mc_address == rx7900_test::fb_base && scanout.otg == 0 && scanout.pipe == 0 &&
                   scanout.secondary_otg_mask == 0 && report.active_otg_mask == 1 && report.primary_otg == 0,
               "1080p surface at the bottom of VRAM, stride 7680");

        PreosReservationCheck check{};
        expect(check_preos_reservations(gop.reservations(), scanout, check) == PreosCoverage::tail_unreserved &&
                   check.tail_offset == 8290304 && check.tail_bytes == 4096,
               "Linux (PITCH-1) reservation leaves the last 1080p page uncovered");
        expect(check_preos_reservations(Gop::map_for(8294400), scanout, check) == PreosCoverage::reserved &&
                   check.tail_bytes == 0,
               "a full-size stolen reservation covers the surface");
    }
    {
        Gop gop;
        gop.scanout(0, 0, mode_1440, 0, false);
        AdoptedScanout scanout{};
        PreosReport report{};
        expect(gop.decode(scanout, report) == PreosStatus::adopted &&
                   scanout.clock_source == PreosClockSource::unknown && scanout.timing.pixel_clock_khz == 0 &&
                   scanout.refresh_millihz == 0 && scanout.timing.width == 2560 && scanout.timing.flags == positive_hsync,
               "1440p over a PHY PLL: geometry adopted, clock unknown");
        uint32_t millihz = 0;
        expect(preos_refresh_from_frames(100, 160, 1000824, 1000000, millihz) && millihz == 59951,
               "60 frame edges in 1.000824 s measure 59.951 Hz");
        expect(apply_preos_measured_refresh(scanout, millihz) && scanout.timing.pixel_clock_khz == 241502 &&
                   scanout.refresh_millihz == 59951,
               "measured refresh derives the pixel clock from the totals");
        PreosReservationCheck check{};
        expect(check_preos_reservations(gop.reservations(), scanout, check) == PreosCoverage::tail_unreserved &&
                   check.tail_offset == 14741504 && check.tail_bytes == 4096,
               "1440p: 9 MiB stolen VGA plus extended, one page short");
    }
    {
        Gop gop;
        gop.scanout(0, 0, mode_2160, 0, false);
        AdoptedScanout scanout{};
        PreosReport report{};
        expect(gop.decode(scanout, report) == PreosStatus::adopted && scanout.stride_bytes == 15360 &&
                   scanout.bytes == 33177600 && same_timing(Timing{0, 3840, 4400, 176, 88, 2160, 2250, 8, 10,
                                                                   positive_hsync | positive_vsync}, scanout.timing),
               "2160p HDMI (the user's monitor) adopted");
        const auto map = gop.reservations();
        PreosReservationCheck check{};
        expect(map.stolen_vga_bytes == 9 * mib && map.stolen_extended_bytes == 2160u * 3839u * 4u - 9 * mib &&
                   check_preos_reservations(map, scanout, check) == PreosCoverage::tail_unreserved &&
                   check.tail_offset == 33169408 && check.tail_bytes == 8192,
               "2160p tail is the 8640-byte undercount rounded to pages");
    }
    {
        Gop gop;
        Mode aligned = mode_2160;
        aligned.pitch_pixels = 4096;
        gop.scanout(0, 0, aligned, 0, true);
        AdoptedScanout scanout{};
        PreosReport report{};
        expect(gop.decode(scanout, report) == PreosStatus::adopted && scanout.stride_bytes == 16384 &&
                   scanout.bytes == 16384ULL * 2160 && scanout.refresh_millihz == 60000,
               "a padded pitch is the stride; 594 MHz DTO gives 60 Hz");
    }
}

// Round trip through the shared OTG field preparation for unusual but valid timings.
void timing_round_trip() {
    const Timing timings[]{
        {25175, 640, 800, 16, 96, 480, 525, 10, 2, 0},
        {108000, 1280, 1688, 48, 112, 1024, 1066, 1, 3, positive_hsync | positive_vsync},
        {533250, 3840, 3920, 48, 32, 2160, 2222, 3, 5, positive_hsync}};

    for (const Timing& timing : timings) {
        Gop gop;
        gop.scanout(0, 0, {timing, timing.width}, 0, true);
        AdoptedScanout scanout{};
        PreosReport report{};
        uint32_t expected = 0;
        refresh_millihz(timing, expected);
        expect(gop.decode(scanout, report) == PreosStatus::adopted && same_timing(scanout.timing, timing) &&
                   scanout.refresh_millihz == expected,
               "decode inverts optc1_program_timing");
    }
}

void refusals() {
    struct Case final {
        const char* contract;
        PreosStatus expected;
        void (*corrupt)(Gop&);
    };

    const Case cases[]{
        {"D1VGA_MODE_ENABLE: legacy VGA", PreosStatus::legacy_vga,
         [](Gop& g) { g.reg(dccg, vga_control[0]) = 0x00000101u; }},
        {"D3VGA_MODE_ENABLE also refuses", PreosStatus::legacy_vga,
         [](Gop& g) { g.reg(dccg, vga_control[2]) = 1; }},
        {"all-ones OTG_CONTROL: device lost", PreosStatus::read_failure,
         [](Gop& g) { g.otg(2, 0x1b41) = 0xffffffffu; }},
        {"master enable without current state", PreosStatus::inconsistent_state,
         [](Gop& g) { g.otg(0, 0x1b41) = 1; }},
        {"no OTG enabled", PreosStatus::no_active_pipe, [](Gop& g) { g.otg(0, 0x1b41) = 0; }},
        {"OTG clock off", PreosStatus::inconsistent_state, [](Gop& g) { g.otg(0, 0x1b85) = 0x1; }},
        {"OTG in soft reset", PreosStatus::inconsistent_state, [](Gop& g) { g.otg(0, 0x1b85) = 0x111; }},
        {"interlaced", PreosStatus::interlaced, [](Gop& g) { g.otg(0, 0x1b44) = 1; }},
        {"OTG pixel division", PreosStatus::unsupported_topology, [](Gop& g) { g.otg(0, 0x1b2e) = 0x00010001u; }},
        {"ODM 2:1 combine", PreosStatus::unsupported_topology,
         [](Gop& g) { g.reg(dcn, 0x1acb) = 0x00100001u; }},
        {"MPC blend tree with a bottom plane", PreosStatus::unsupported_topology,
         [](Gop& g) { g.mpcc(0, 0x0001) = 1; }},
        {"MPCC routed to another OPP", PreosStatus::unsupported_topology, [](Gop& g) { g.mpcc(0, 0x0002) = 1; }},
        {"OPP without an MPCC", PreosStatus::unsupported_topology, [](Gop& g) { g.reg(mpc, 0x03d8) = 0xf; }},
        {"HUBP VTG on another OTG", PreosStatus::unsupported_topology, [](Gop& g) { g.hubp(0, 0x05f3) = 0x10; }},
        {"MPCC bypass mode", PreosStatus::unsupported_topology,
         [](Gop& g) { g.mpcc(0, 0x0003) &= ~3u; }},
        {"blank start before blank end", PreosStatus::inconsistent_timing,
         [](Gop& g) { g.otg(0, 0x1b2b) = 100u | (200u << 16); }},
        {"sync not at the line start", PreosStatus::inconsistent_timing,
         [](Gop& g) { g.otg(0, 0x1b2c) |= 4; }},
        {"sync beyond the back porch", PreosStatus::inconsistent_timing,
         [](Gop& g) { g.otg(0, 0x1b37) = 1000u << 16; }},
        {"composite sync", PreosStatus::inconsistent_timing, [](Gop& g) { g.otg(0, 0x1b2d) |= 0x10000; }},
        {"reserved H_TOTAL bits", PreosStatus::inconsistent_state, [](Gop& g) { g.otg(0, 0x1b2a) |= 0x8000; }},
        {"H_TOTAL shorter than the blank start", PreosStatus::inconsistent_timing,
         [](Gop& g) { g.otg(0, 0x1b2a) = 1930; }},
        {"HUBP blanked", PreosStatus::plane_disabled, [](Gop& g) { g.hubp(0, 0x05f3) |= 1; }},
        {"HUBP clock gated", PreosStatus::plane_disabled, [](Gop& g) { g.hubp(0, 0x05f4) = 0; }},
        {"ARGB2101010", PreosStatus::unsupported_format, [](Gop& g) { g.hubp(0, 0x05e5) = 10; }},
        {"RGB565", PreosStatus::unsupported_format, [](Gop& g) { g.hubp(0, 0x05e5) = 3; }},
        {"ABGR8888 crossbar", PreosStatus::unsupported_format,
         [](Gop& g) { g.hubp(0, 0x066c) = (2u << 22) | (3u << 20); }},
        {"alpha plane", PreosStatus::unsupported_format, [](Gop& g) { g.hubp(0, 0x05e5) |= 0x800; }},
        {"64KB_R_X swizzle", PreosStatus::tiled_or_compressed, [](Gop& g) { g.hubp(0, 0x05e7) = 0x1b; }},
        {"primary DCC", PreosStatus::tiled_or_compressed, [](Gop& g) { g.hubp(0, 0x061a) = 0x2; }},
        {"rotation 90", PreosStatus::rotated_or_mirrored, [](Gop& g) { g.hubp(0, 0x05e5) |= 0x100; }},
        {"horizontal mirror", PreosStatus::rotated_or_mirrored, [](Gop& g) { g.hubp(0, 0x05e5) |= 0x400; }},
        {"720p viewport upscaled by DSCL", PreosStatus::scaled, [](Gop& g) {
             g.hubp(0, 0x05ea) = (720u << 16) | 1280u;
             g.dscl(0, 0x0cfb) = 1;
         }},
        {"DSCL scaling enabled at 1:1", PreosStatus::scaled, [](Gop& g) { g.dscl(0, 0x0cfb) = 1; }},
        {"viewport offset", PreosStatus::scaled, [](Gop& g) { g.hubp(0, 0x05e9) = 8; }},
        {"RECOUT smaller than the active area", PreosStatus::scaled,
         [](Gop& g) { g.dscl(0, 0x0d12) = (1000u << 16) | 1920u; }},
        {"RECOUT offset (underscan border)", PreosStatus::scaled, [](Gop& g) { g.dscl(0, 0x0d11) = 0x00100010u; }},
        {"per-pixel alpha over the background", PreosStatus::alpha_dependent,
         [](Gop& g) { g.mpcc(0, 0x0003) = 2; }},
        {"global alpha below opaque", PreosStatus::alpha_dependent,
         [](Gop& g) { g.mpcc(0, 0x0003) = (0x80u << 16) | (2u << 4) | 2u; }},
        {"TMZ surface", PreosStatus::protected_surface, [](Gop& g) { g.hubp(0, 0x061a) = 1; }},
        {"VMID 1 surface", PreosStatus::gpu_virtual_address, [](Gop& g) { g.hubp(0, 0x0609) = 1; }},
        {"pitch below width", PreosStatus::inconsistent_timing, [](Gop& g) { g.hubp(0, 0x0607) = 1000; }},
        {"address below the FB aperture", PreosStatus::outside_framebuffer,
         [](Gop& g) { g.hubp(0, 0x060b) = 0x7f; }},
        {"address beyond VRAM", PreosStatus::outside_framebuffer, [](Gop& g) { g.hubp(0, 0x060b) = 0x8f; }},
        {"misaligned address", PreosStatus::outside_framebuffer, [](Gop& g) { g.hubp(0, 0x060a) = 0x40; }},
        {"DCN FB base differs from MMHUB", PreosStatus::outside_framebuffer,
         [](Gop& g) { g.reg(dcn, 0x0475) += 1; }},
        {"DCN FB offset differs from MMHUB", PreosStatus::outside_framebuffer,
         [](Gop& g) { g.reg(dcn, 0x0477) = 0; }},
        {"garbage in DCN FB reserved bits", PreosStatus::inconsistent_state,
         [](Gop& g) { g.reg(dcn, 0x0476) |= 0x01000000u; }},
        {"surface beyond the 256 MiB BAR0 window", PreosStatus::not_cpu_visible,
         [](Gop& g) { g.hubp(0, 0x060a) = 0x10000000u; }},
        {"two active OTGs, neither fed by HUBP0", PreosStatus::ambiguous_primary, [](Gop& g) {
             g.scanout(0, 1, mode_1080, 0, true);
             g.scanout(1, 2, mode_1080, 0, true);
             g.hubp(0, 0x05f4) = 0;
             g.mpcc(0, 0x0000) = 0xf;
         }},
        {"one HUBP feeding two OTGs", PreosStatus::unsupported_topology, [](Gop& g) {
             g.scanout(1, 1, mode_1080, 0, true);
             g.mpcc(1, 0x0000) = 0; // MPCC1 also selects DPP0
         }}};

    for (const Case& test : cases) {
        Gop gop;
        gop.scanout(0, 0, mode_1080, 0, true);
        test.corrupt(gop);
        AdoptedScanout scanout{};
        scanout.otg = 77;
        PreosReport report{};
        const PreosStatus status = gop.decode(scanout, report);
        expect(status == test.expected && report.status == status && scanout.otg == 77, test.contract);
    }

    Gop vga;
    vga.scanout(0, 0, mode_1080, 0, true);
    vga.reg(dccg, vga_control[0]) = 1;
    AdoptedScanout scanout{};
    PreosReport report{};
    expect(vga.decode(scanout, report) == PreosStatus::legacy_vga && report.vga_mask == 1 &&
               report.active_otg_mask == 0,
           "VGA text mode is reported before any HUBP state is trusted");

    Gop lost;
    lost.device->reads_ok = false;
    PreosScanoutRegisters snapshot{};
    snapshot.fb_offset = 5;
    expect(!read_preos_scanout(lost.io(), lost.registers(), snapshot) && snapshot.fb_offset == 0,
           "a failed register read yields no snapshot");
}

void accepted_variants() {
    Gop passthrough;
    passthrough.scanout(0, 0, mode_1080, 0, true);
    passthrough.mpcc(0, 0x0003) = 1; // TOP_LAYER_PASSTHROUGH, per-pixel alpha ignored
    passthrough.dscl(0, 0x0cfb) = 6; // DSCL_MODE_DSCL_BYPASS
    passthrough.hubp(0, 0x05f3) |= 0x30000000u; // sticky underflow is informational
    AdoptedScanout scanout{};
    PreosReport report{};
    expect(passthrough.decode(scanout, report) == PreosStatus::adopted && report.underflow_mask == 1,
           "passthrough MPCC and DSCL bypass adopted; underflow reported");

    Gop drr;
    drr.scanout(0, 0, mode_1080, 0, true);
    drr.otg(0, 0x1b33) = 3;
    expect(drr.decode(scanout, report) == PreosStatus::adopted && scanout.variable_refresh &&
               scanout.refresh_millihz == 0 && !apply_preos_measured_refresh(scanout, 60000),
           "variable refresh keeps the refresh unknown");

    Gop hdmi;
    hdmi.scanout(0, 0, mode_1080, 0, true);
    hdmi.reg(dccg, 0x0080) = 0x18; // DTBCLK DTO (DP 2.0) is not a DP DTO pixel clock
    expect(hdmi.decode(scanout, report) == PreosStatus::adopted && scanout.clock_source == PreosClockSource::unknown,
           "DTBCLK DTO leaves the clock unknown");
    hdmi.reg(dccg, 0x0080) = 0x10;
    hdmi.reg(dccg, 0x0081) = 700000000u; // phase above modulo
    expect(hdmi.decode(scanout, report) == PreosStatus::adopted && scanout.timing.pixel_clock_khz == 0,
           "a DTO phase above its modulo is not trusted");

    Gop dual;
    dual.scanout(0, 0, mode_2160, 0, false);
    dual.scanout(1, 1, mode_1080, 64 * mib, true);
    expect(dual.decode(scanout, report) == PreosStatus::adopted && scanout.otg == 0 && scanout.pipe == 0 &&
               scanout.secondary_otg_mask == 2 && report.active_otg_mask == 3 && report.otg_pipe[1] == 1,
           "two displays: HUBP0's pipe adopted, the other reported only");

    Gop swapped;
    swapped.scanout(1, 0, mode_1080, 0, true);
    swapped.scanout(0, 1, mode_1440, 16 * mib, true);
    expect(swapped.decode(scanout, report) == PreosStatus::adopted && scanout.otg == 1 && scanout.pipe == 0 &&
               scanout.opp == 1 && scanout.mpcc == 0 && scanout.secondary_otg_mask == 1,
           "primary follows HUBP0 even on OTG1");

    Gop single;
    single.scanout(2, 2, mode_1080, 0, true);
    expect(single.decode(scanout, report) == PreosStatus::adopted && scanout.pipe == 2,
           "a single active pipe other than HUBP0 decodes");
    PreosReservationCheck check{};
    expect(check_preos_reservations(single.reservations(), scanout, check) == PreosCoverage::unreserved,
           "...but Linux sized the reservation from HUBP0, so it is not reserved");
}

void reservation_consistency() {
    AdoptedScanout scanout{};
    scanout.timing = mode_1080.timing;
    scanout.stride_bytes = 7680;
    scanout.bytes = 8294400;
    PreosReservationCheck check{};

    scanout.vram_offset = 64 * mib;
    expect(check_preos_reservations(Gop::map_for(8294400), scanout, check) == PreosCoverage::unreserved,
           "a surface outside the stolen range is unreserved");
    scanout.vram_offset = 0;
    expect(check_preos_reservations(Gop::map_for(0), scanout, check) == PreosCoverage::unreserved,
           "no stolen reservation at all");
    expect(check_preos_reservations(Gop::map_for(4 * mib), scanout, check) == PreosCoverage::unreserved,
           "a reservation far smaller than the surface (other viewport) is not a tail");
    scanout.vram_offset = 4096;
    expect(check_preos_reservations(Gop::map_for(8294400 + 8192), scanout, check) == PreosCoverage::reserved,
           "a surface inside the stolen range at a nonzero offset is reserved");

    VramReservationMap map = Gop::map_for(1080u * 1919u * 4u);
    scanout.vram_offset = 0;
    rx7900::hardware::VramReservation& extra = map.entries[map.count++];
    extra = {rx7900::hardware::VramReservationKind::firmware_usage, 8290304, 4096, true};
    expect(check_preos_reservations(map, scanout, check) == PreosCoverage::unreserved,
           "a tail owned by another lifetime reservation is refused");
    map.entries[map.count - 1].lifetime = false;
    expect(check_preos_reservations(map, scanout, check) == PreosCoverage::tail_unreserved,
           "a bring-up-only hold over the tail does not count");

    scanout.bytes = 0;
    expect(check_preos_reservations(map, scanout, check) == PreosCoverage::unreserved, "empty surface refused");
    scanout.bytes = 8294400;
    scanout.vram_offset = UINT64_MAX - 100;
    expect(check_preos_reservations(map, scanout, check) == PreosCoverage::unreserved, "wrapping range refused");
}

void comparisons_and_refresh() {
    Gop gop;
    gop.scanout(0, 0, mode_1440, 0, false);
    AdoptedScanout before{};
    AdoptedScanout after{};
    PreosReport report{};
    gop.decode(before, report);
    gop.decode(after, report);
    expect(same_preos_scanout(before, after), "identical snapshots describe the same scanout");
    apply_preos_measured_refresh(after, 59951);
    gop.hubp(0, 0x05f3) |= 0x8; // HUBP_IN_BLANK toggles every frame
    gop.decode(before, report);
    expect(same_preos_scanout(before, after), "measured refresh and volatile status ignored");
    gop.hubp(0, 0x060a) = 0x00100000u;
    gop.decode(after, report);
    expect(!same_preos_scanout(before, after), "a moved surface invalidates adoption");
    gop.hubp(0, 0x060a) = 0;
    gop.otg(0, 0x1b38) = 0; // V sync becomes positive
    gop.decode(after, report);
    expect(!same_preos_scanout(before, after), "a changed sync polarity invalidates adoption");

    uint32_t millihz = 0;
    expect(preos_refresh_from_frames(0x00fffff0u, 0x0000000cu, 466667, 1000000, millihz) && millihz == 60000,
           "24-bit frame counter wrap");
    expect(!preos_refresh_from_frames(5, 5, 1000, 1000000, millihz) &&
               !preos_refresh_from_frames(0, 3, 1000000, 1000000, millihz) &&
               !preos_refresh_from_frames(0, 600, 1000000, 1000000, millihz) &&
               !preos_refresh_from_frames(0, 5000, 10000000, 1000000, millihz) &&
               !preos_refresh_from_frames(0, 60, 1000000, 0, millihz),
           "no frames, 3 Hz, 600 Hz, oversized spans and zero rates refused");
    AdoptedScanout known{};
    gop.otg(0, 0x1b38) = 1;
    gop.reg(dccg, 0x0080) = 0x10;
    gop.reg(dccg, 0x0081) = 241500000u;
    gop.reg(dccg, 0x0082) = 600000000u;
    gop.decode(known, report);
    expect(known.clock_source == PreosClockSource::dp_dto && !apply_preos_measured_refresh(known, 59000) &&
               !apply_preos_measured_refresh(after, 9000),
           "a DTO clock is never overridden; implausible measurements refused");
}

} // namespace

int main() {
    discovery();
    typical_modes();
    timing_round_trip();
    refusals();
    accepted_variants();
    reservation_consistency();
    comparisons_and_refresh();
    return checks.finish("preos_scanout");
}

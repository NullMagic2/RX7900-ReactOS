/*
Module: rx7900.display.preos_scanout
Purpose: Read-only DCN3.2 pre-OS scanout snapshot and adoption decisions.
Created: 2026-10-22
Architecture: Register reads through hardware::RegisterIo; pure decoding reuses
prepare_optc_timing, refresh_millihz and validate_framebuffer; reservation
coverage reads the hardware::VramReservationMap. Allocation-free, no writes.
Copyright 2012-2023 Advanced Micro Devices, Inc. Offsets/masks: Linux v6.12
dcn_3_2_0_offset.h/_sh_mask.h; rules: see preos_scanout.hpp. Notice:
licenses/AMDGPU-DISPLAY.txt.
*/
#include "preos_scanout.hpp"

namespace rx7900::display {
namespace {

using hardware::discovery_register_offset;
using hardware::read_ip_register;

constexpr uint32_t segment_dccg = 1; // BASE_IDX 1
constexpr uint32_t segment_dcn = 2;  // BASE_IDX 2
constexpr uint32_t segment_mpc = 3;  // BASE_IDX 3

// dcn_3_2_0_offset.h, instances 0..3 (dword indices).
constexpr uint32_t vga_control[preos_pipe_count]{0x000c, 0x000e, 0x0038, 0x0039};
constexpr uint32_t pixel_rate_control[preos_pipe_count]{0x0080, 0x0084, 0x0088, 0x008c};
constexpr uint32_t dto_phase[preos_pipe_count]{0x0081, 0x0085, 0x0089, 0x008d};
constexpr uint32_t dto_modulo[preos_pipe_count]{0x0082, 0x0086, 0x008a, 0x008e};
constexpr uint32_t fb_location_base = 0x0475;
constexpr uint32_t fb_location_top = 0x0476;
constexpr uint32_t fb_offset = 0x0477;
constexpr uint32_t otg_control[preos_pipe_count]{0x1b41, 0x1bc1, 0x1c41, 0x1cc1};
constexpr uint32_t otg_clock_control[preos_pipe_count]{0x1b85, 0x1c05, 0x1c85, 0x1d05};
constexpr uint32_t otg_h_total[preos_pipe_count]{0x1b2a, 0x1baa, 0x1c2a, 0x1caa};
constexpr uint32_t otg_h_blank[preos_pipe_count]{0x1b2b, 0x1bab, 0x1c2b, 0x1cab};
constexpr uint32_t otg_h_sync[preos_pipe_count]{0x1b2c, 0x1bac, 0x1c2c, 0x1cac};
constexpr uint32_t otg_h_sync_control[preos_pipe_count]{0x1b2d, 0x1bad, 0x1c2d, 0x1cad};
constexpr uint32_t otg_h_timing_control[preos_pipe_count]{0x1b2e, 0x1bae, 0x1c2e, 0x1cae};
constexpr uint32_t otg_v_total[preos_pipe_count]{0x1b2f, 0x1baf, 0x1c2f, 0x1caf};
constexpr uint32_t otg_v_total_control[preos_pipe_count]{0x1b33, 0x1bb3, 0x1c33, 0x1cb3};
constexpr uint32_t otg_v_blank[preos_pipe_count]{0x1b36, 0x1bb6, 0x1c36, 0x1cb6};
constexpr uint32_t otg_v_sync[preos_pipe_count]{0x1b37, 0x1bb7, 0x1c37, 0x1cb7};
constexpr uint32_t otg_v_sync_control[preos_pipe_count]{0x1b38, 0x1bb8, 0x1c38, 0x1cb8};
constexpr uint32_t otg_interlace_control[preos_pipe_count]{0x1b44, 0x1bc4, 0x1c44, 0x1cc4};
constexpr uint32_t otg_frame_count[preos_pipe_count]{0x1b4c, 0x1bcc, 0x1c4c, 0x1ccc};
constexpr uint32_t odm_data_source_select[preos_pipe_count]{0x1acb, 0x1adb, 0x1aeb, 0x1afb};
constexpr uint32_t hubp_control[preos_pipe_count]{0x05f3, 0x06cf, 0x07ab, 0x0887};
constexpr uint32_t hubp_clock_control[preos_pipe_count]{0x05f4, 0x06d0, 0x07ac, 0x0888};
constexpr uint32_t surface_config[preos_pipe_count]{0x05e5, 0x06c1, 0x079d, 0x0879};
constexpr uint32_t tiling_config[preos_pipe_count]{0x05e7, 0x06c3, 0x079f, 0x087b};
constexpr uint32_t viewport_start[preos_pipe_count]{0x05e9, 0x06c5, 0x07a1, 0x087d};
constexpr uint32_t viewport_dimension[preos_pipe_count]{0x05ea, 0x06c6, 0x07a2, 0x087e};
constexpr uint32_t surface_pitch[preos_pipe_count]{0x0607, 0x06e3, 0x07bf, 0x089b};
constexpr uint32_t address_low[preos_pipe_count]{0x060a, 0x06e6, 0x07c2, 0x089e};
constexpr uint32_t address_high[preos_pipe_count]{0x060b, 0x06e7, 0x07c3, 0x089f};
constexpr uint32_t surface_control[preos_pipe_count]{0x061a, 0x06f6, 0x07d2, 0x08ae};
constexpr uint32_t vmid_settings[preos_pipe_count]{0x0609, 0x06e5, 0x07c1, 0x089d};
constexpr uint32_t hubpret_control[preos_pipe_count]{0x066c, 0x0748, 0x0824, 0x0900};
constexpr uint32_t dscl_mode[preos_pipe_count]{0x0cfb, 0x0e66, 0x0fd1, 0x113c};
constexpr uint32_t recout_start[preos_pipe_count]{0x0d11, 0x0e7c, 0x0fe7, 0x1152};
constexpr uint32_t recout_size[preos_pipe_count]{0x0d12, 0x0e7d, 0x0fe8, 0x1153};
constexpr uint32_t mpc_out_mux[preos_pipe_count]{0x03d8, 0x03dc, 0x03e0, 0x03e4};
constexpr uint32_t mpcc_top_select[preos_pipe_count]{0x0000, 0x0015, 0x002a, 0x003f};
constexpr uint32_t mpcc_bottom_select[preos_pipe_count]{0x0001, 0x0016, 0x002b, 0x0040};
constexpr uint32_t mpcc_opp_id[preos_pipe_count]{0x0002, 0x0017, 0x002c, 0x0041};
constexpr uint32_t mpcc_control[preos_pipe_count]{0x0003, 0x0018, 0x002d, 0x0042};

// dcn_3_2_0_sh_mask.h fields (identical for every instance).
constexpr uint32_t vga_mode_enable = 0x00000001u;          // DnVGA_MODE_ENABLE
constexpr uint32_t otg_master_enable = 0x00000001u;        // OTG_MASTER_EN
constexpr uint32_t otg_current_master_enable = 0x00010000u; // OTG_CURRENT_MASTER_EN_STATE
constexpr uint32_t otg_clock_enable = 0x00000001u;         // OTG_CLOCK_EN
constexpr uint32_t otg_soft_reset = 0x00000010u;           // OTG_SOFT_RESET
constexpr uint32_t otg_clock_on = 0x00000100u;             // OTG_CLOCK_ON
constexpr uint32_t timing_field = 0x00007fffu;             // OTG_H/V_* 15-bit fields
constexpr uint32_t timing_reserved = 0x80008000u;
constexpr uint32_t sync_polarity = 0x00000001u;            // OTG_H/V_SYNC_A_POL
constexpr uint32_t composite_sync = 0x00010000u;           // OTG_COMP_SYNC_A_EN
constexpr uint32_t h_division = 0x00030003u;               // OTG_H_TIMING_DIV_MODE, _CURR
constexpr uint32_t h_division_manual = 0x00000100u;        // OTG_H_TIMING_DIV_MODE_MANUAL
constexpr uint32_t interlace_enable = 0x00000001u;         // OTG_INTERLACE_ENABLE
constexpr uint32_t v_total_select = 0x00000003u;           // OTG_V_TOTAL_MIN_SEL, _MAX_SEL
constexpr uint32_t input_segments = 0x00000003u;           // OPTC_NUM_OF_INPUT_SEGMENT
constexpr uint32_t segment0_source_shift = 16;             // OPTC_SEG0_SRC_SEL
constexpr uint32_t dtbclk_dto_enable = 0x00000008u;        // DTBCLK_DTOn_ENABLE
constexpr uint32_t dp_dto_enable = 0x00000010u;            // DP_DTOn_ENABLE
constexpr uint32_t mpc_select = 0x0000000fu;               // MPC_OUT_MUX, MPCC_*_SEL, MPCC_OPP_ID
constexpr uint32_t mpc_disconnected = 0xf;
constexpr uint32_t mpcc_mode = 0x00000003u;                // MPCC_MODE
constexpr uint32_t mpcc_alpha_mode_shift = 4;              // MPCC_ALPHA_BLND_MODE (2 bits)
constexpr uint32_t mpcc_global_alpha_shift = 16;           // MPCC_GLOBAL_ALPHA (8 bits)
constexpr uint32_t mpcc_top_layer_passthrough = 1;         // MPCC_BLEND_MODE_*
constexpr uint32_t mpcc_top_layer_only = 2;
constexpr uint32_t mpcc_global_alpha_mode = 2;             // MPCC_ALPHA_BLEND_MODE_GLOBAL_ALPHA
constexpr uint32_t hubp_blank = 0x00000001u;               // HUBP_BLANK_EN
constexpr uint32_t hubp_soft_reset = 0x00000004u;          // HUBP_SOFT_RESET
constexpr uint32_t hubp_vtg_shift = 4;                     // HUBP_VTG_SEL (4 bits)
constexpr uint32_t hubp_underflow = 0x70000000u;           // HUBP_UNDERFLOW_STATUS
constexpr uint32_t hubp_clock = 0x00000001u;               // HUBP_CLOCK_ENABLE
constexpr uint32_t pixel_format = 0x0000007fu;             // SURFACE_PIXEL_FORMAT
constexpr uint32_t pixel_format_argb8888 = 8;              // hubp2_program_pixel_format
constexpr uint32_t rotation = 0x00000300u;                 // ROTATION_ANGLE
constexpr uint32_t mirror = 0x00000400u;                   // H_MIRROR_EN
constexpr uint32_t alpha_plane = 0x00000800u;              // ALPHA_PLANE_EN
constexpr uint32_t swizzle = 0x0000001fu;                  // SW_MODE; 0 is DC_SW_LINEAR
constexpr uint32_t surface_tmz = 0x00000001u;              // PRIMARY_SURFACE_TMZ
constexpr uint32_t surface_dcc = 0x00000002u;              // PRIMARY_SURFACE_DCC_EN
constexpr uint32_t vmid = 0x0000000fu;                     // VMID
constexpr uint32_t crossbar_blue_shift = 20;               // CROSSBAR_SRC_CB_B (2 bits)
constexpr uint32_t crossbar_red_shift = 22;                // CROSSBAR_SRC_CR_R (2 bits)
constexpr uint32_t viewport_field = 0x00003fffu;           // PRI_VIEWPORT_*, RECOUT_WIDTH/HEIGHT
constexpr uint32_t recout_start_field = 0x00001fffu;       // RECOUT_START_X/Y
constexpr uint32_t pitch_field = 0x00003fffu;              // PITCH (pitch - 1, pixels)
constexpr uint32_t address_high_field = 0x0000ffffu;       // PRIMARY_SURFACE_ADDRESS_HIGH
constexpr uint32_t dscl_mode_field = 0x00000007u;          // DSCL_MODE
constexpr uint32_t dscl_444_bypass = 0;                    // DSCL_MODE_SCALING_444_BYPASS
constexpr uint32_t dscl_bypass = 6;                        // DSCL_MODE_DSCL_BYPASS
constexpr uint32_t fb_field = 0x00ffffffu;                 // FB_BASE/FB_TOP/FB_OFFSET (16 MiB units)

// DCN surface addresses are 256-byte aligned (hubp2 address registers).
constexpr uint64_t surface_alignment = 256;
constexpr uint64_t page_bytes = 4096;

uint32_t field(uint32_t value, uint32_t shift, uint32_t mask) noexcept {
    return (value >> shift) & mask;
}

uint32_t low15(uint32_t value) noexcept { return value & timing_field; }
uint32_t high15(uint32_t value) noexcept { return (value >> 16) & timing_field; }

struct Reader final {
    const hardware::RegisterIo& io;
    const PreosDcnRegisters& registers;
    bool ok = true;

    uint32_t read(uint32_t segment, uint32_t reg) noexcept {
        uint32_t value = 0;

        if (ok && !read_ip_register(io, registers.segment(segment), reg, value)) {
            ok = false;
        }

        return ok ? value : 0;
    }
};

// Every register the snapshot reads, for the discovery-time aperture check.
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

// optc1_program_timing inverse: H/V_TOTAL = total - 1; sync starts at 0 and ends at
// the sync width; blank start = total - front porch; blank end = blank start - active.
PreosStatus decode_timing(const PreosOtgRegisters& otg, Timing& timing) noexcept {
    if ((otg.h_total & ~timing_field) != 0 || (otg.v_total & ~timing_field) != 0 ||
        (otg.h_blank & timing_reserved) != 0 || (otg.v_blank & timing_reserved) != 0 ||
        (otg.h_sync & timing_reserved) != 0 || (otg.v_sync & timing_reserved) != 0) {
        return PreosStatus::inconsistent_state;
    }

    if ((otg.h_sync_control & composite_sync) != 0) {
        return PreosStatus::inconsistent_timing;
    }

    const uint32_t h_total = otg.h_total + 1;
    const uint32_t v_total = otg.v_total + 1;
    const uint32_t h_blank_start = low15(otg.h_blank);
    const uint32_t h_blank_end = high15(otg.h_blank);
    const uint32_t v_blank_start = low15(otg.v_blank);
    const uint32_t v_blank_end = high15(otg.v_blank);
    const uint32_t h_sync_start = low15(otg.h_sync);
    const uint32_t h_sync_end = high15(otg.h_sync);
    const uint32_t v_sync_start = low15(otg.v_sync);
    const uint32_t v_sync_end = high15(otg.v_sync);

    // Blank end precedes blank start, which precedes the end of the line/frame;
    // the sync pulse starts the line/frame and ends inside the back porch.
    if (h_blank_end >= h_blank_start || h_blank_start >= h_total ||
        v_blank_end >= v_blank_start || v_blank_start >= v_total ||
        h_sync_start != 0 || v_sync_start != 0 || h_sync_end == 0 || v_sync_end == 0 ||
        h_sync_end > h_blank_end || v_sync_end > v_blank_end) {
        return PreosStatus::inconsistent_timing;
    }

    Timing decoded{};
    decoded.width = h_blank_start - h_blank_end;
    decoded.h_total = h_total;
    decoded.h_front_porch = h_total - h_blank_start;
    decoded.h_sync_width = h_sync_end;
    decoded.height = v_blank_start - v_blank_end;
    decoded.v_total = v_total;
    decoded.v_front_porch = v_total - v_blank_start;
    decoded.v_sync_width = v_sync_end;
    decoded.flags = ((otg.h_sync_control & sync_polarity) == 0 ? positive_hsync : 0u) |
                    ((otg.v_sync_control & sync_polarity) == 0 ? positive_vsync : 0u);

    // The shared DCN3.2 representability rules; a placeholder clock is used only
    // for this geometric check when the real clock is not derivable.
    Timing check = decoded;
    check.pixel_clock_khz = 1;
    OptcTiming prepared{};

    if (prepare_optc_timing(check, prepared) != Status::success) {
        return PreosStatus::inconsistent_timing;
    }

    timing = decoded;
    return PreosStatus::adopted;
}

// get_pixel_clk_frequency_100hz: with the DP DTO the VBIOS programs MODULO equal
// to DPREFCLK in Hz, so PHASE is the pixel clock in Hz. The DTO cannot exceed
// its reference. Anything else is a PHY PLL or DTBCLK rate this module cannot see.
void decode_clock(const PreosOtgRegisters& otg, AdoptedScanout& scanout) noexcept {
    scanout.clock_source = PreosClockSource::unknown;
    scanout.timing.pixel_clock_khz = 0;
    scanout.refresh_millihz = 0;

    if ((otg.pixel_rate_control & dp_dto_enable) == 0 ||
        (otg.pixel_rate_control & dtbclk_dto_enable) != 0 || otg.dto_modulo == 0 ||
        otg.dto_phase == 0 || otg.dto_phase > otg.dto_modulo || scanout.variable_refresh) {
        return;
    }

    Timing timing = scanout.timing;
    timing.pixel_clock_khz = otg.dto_phase / 1000u;
    uint32_t refresh = 0;

    if (refresh_millihz(timing, refresh) != Status::success ||
        refresh < preos_minimum_refresh_millihz || refresh > preos_maximum_refresh_millihz) {
        return;
    }

    scanout.clock_source = PreosClockSource::dp_dto;
    scanout.timing.pixel_clock_khz = timing.pixel_clock_khz;
    scanout.refresh_millihz = refresh;
}

// OTG -> ODM segment 0 OPP -> MPC_OUT mux MPCC -> MPCC top DPP, exactly one MPCC
// per tree (mpc1_insert_plane with no plane below).
PreosStatus resolve_pipe(const PreosScanoutRegisters& registers, uint32_t otg, uint32_t& opp,
                         uint32_t& mpcc, uint32_t& pipe) noexcept {
    const uint32_t select = registers.otg[otg].data_source_select;

    if ((select & input_segments) != 0) {
        return PreosStatus::unsupported_topology; // ODM combine
    }

    opp = field(select, segment0_source_shift, mpc_select);

    if (opp >= preos_pipe_count) {
        return PreosStatus::unsupported_topology;
    }

    mpcc = registers.pipe[opp].mpc_out_mux & mpc_select;

    if (mpcc >= preos_pipe_count) {
        return PreosStatus::unsupported_topology;
    }

    const PreosPipeRegisters& tree = registers.pipe[mpcc];
    pipe = tree.mpcc_top_select & mpc_select;

    if (pipe >= preos_pipe_count || (tree.mpcc_bottom_select & mpc_select) != mpc_disconnected ||
        (tree.mpcc_opp_id & mpc_select) != opp) {
        return PreosStatus::unsupported_topology;
    }

    return PreosStatus::adopted;
}

PreosStatus check_otg(const PreosOtgRegisters& otg) noexcept {
    if ((otg.clock_control & (otg_clock_enable | otg_clock_on | otg_soft_reset)) !=
        (otg_clock_enable | otg_clock_on)) {
        return PreosStatus::inconsistent_state;
    }

    if ((otg.interlace_control & interlace_enable) != 0) {
        return PreosStatus::interlaced;
    }

    // optc32_set_odm_bypass: RGB keeps H_TIMING_NO_DIV; any division changes the
    // pixel units of the OTG horizontal fields.
    if ((otg.h_timing_control & (h_division | h_division_manual)) != 0) {
        return PreosStatus::unsupported_topology;
    }

    return PreosStatus::adopted;
}

// HUBP/DPP/MPCC of the adopted pipe, in the PreosStatus order.
PreosStatus check_plane(const PreosPipeRegisters& pipe, const PreosPipeRegisters& tree,
                        uint32_t otg, const Timing& timing) noexcept {
    if ((pipe.hubp_clock_control & hubp_clock) == 0 ||
        (pipe.hubp_control & (hubp_blank | hubp_soft_reset)) != 0) {
        return PreosStatus::plane_disabled;
    }

    if (field(pipe.hubp_control, hubp_vtg_shift, 0xf) != otg) {
        return PreosStatus::unsupported_topology;
    }

    const uint32_t blue = field(pipe.hubpret_control, crossbar_blue_shift, 3);
    const uint32_t red = field(pipe.hubpret_control, crossbar_red_shift, 3);

    // hubp2_program_pixel_format: ARGB8888 is format 8 with CB_B=2, CR_R=3; the
    // swapped crossbar is ABGR8888 (R and B exchanged for GDI).
    if ((pipe.surface_config & pixel_format) != pixel_format_argb8888 ||
        (pipe.surface_config & alpha_plane) != 0 || blue != 2 || red != 3) {
        return PreosStatus::unsupported_format;
    }

    if ((pipe.tiling_config & swizzle) != 0 || (pipe.surface_control & surface_dcc) != 0) {
        return PreosStatus::tiled_or_compressed;
    }

    if ((pipe.surface_config & (rotation | mirror)) != 0) {
        return PreosStatus::rotated_or_mirrored;
    }

    const uint32_t dscl = pipe.dscl_mode & dscl_mode_field;

    if ((pipe.viewport_start & ((viewport_field << 16) | viewport_field)) != 0 ||
        (pipe.viewport_dimension & viewport_field) != timing.width ||
        field(pipe.viewport_dimension, 16, viewport_field) != timing.height ||
        (dscl != dscl_444_bypass && dscl != dscl_bypass) ||
        (pipe.recout_start & ((recout_start_field << 16) | recout_start_field)) != 0 ||
        (pipe.recout_size & viewport_field) != timing.width ||
        field(pipe.recout_size, 16, viewport_field) != timing.height) {
        return PreosStatus::scaled;
    }

    const uint32_t mode = tree.mpcc_control & mpcc_mode;
    const bool opaque = field(tree.mpcc_control, mpcc_alpha_mode_shift, 3) == mpcc_global_alpha_mode &&
                        field(tree.mpcc_control, mpcc_global_alpha_shift, 0xff) == 0xff;

    if (mode != mpcc_top_layer_passthrough && mode != mpcc_top_layer_only) {
        return PreosStatus::unsupported_topology;
    }

    if (mode == mpcc_top_layer_only && !opaque) {
        return PreosStatus::alpha_dependent;
    }

    if ((pipe.surface_control & surface_tmz) != 0) {
        return PreosStatus::protected_surface;
    }

    if ((pipe.vmid_settings & vmid) != 0) {
        return PreosStatus::gpu_virtual_address;
    }

    return PreosStatus::adopted;
}

// hubbub3_init_dchub_sys_ctx with mmhub_read_system_context: DC programs the DCN
// FB aperture from the MMHUB values, so MC addresses mean the same to both.
PreosStatus place_surface(const PreosScanoutRegisters& registers, const PreosPipeRegisters& pipe,
                          const hardware::VramLocation& vram, uint64_t visible,
                          AdoptedScanout& scanout) noexcept {
    if (((registers.fb_location_base | registers.fb_location_top | registers.fb_offset) & ~fb_field) != 0 ||
        (pipe.address_high & ~address_high_field) != 0 || (pipe.surface_pitch & ~((pitch_field << 16) | pitch_field)) != 0) {
        return PreosStatus::inconsistent_state;
    }

    const uint64_t fb_start = static_cast<uint64_t>(registers.fb_location_base) << 24;
    const uint64_t fb_end = (static_cast<uint64_t>(registers.fb_location_top) << 24) | 0xffffffu;
    const uint64_t address = (static_cast<uint64_t>(pipe.address_high) << 32) | pipe.address_low;
    // hubp2_program_size: PITCH holds pitch - 1 in pixels.
    const uint32_t pitch_pixels = (pipe.surface_pitch & pitch_field) + 1;
    const uint32_t stride = pitch_pixels * xrgb8888_bytes_per_pixel;

    if (pitch_pixels < scanout.timing.width) {
        return PreosStatus::inconsistent_timing;
    }

    // Reuse the shared footprint check (stride, alignment, stride x height).
    Timing timing = scanout.timing;
    timing.pixel_clock_khz = 1;
    const uint64_t bytes = static_cast<uint64_t>(stride) * timing.height;
    ScanoutLayout layout{};

    if (validate_framebuffer(timing, {{address}, bytes, 0, stride}, layout) != Status::success ||
        layout.surface_bytes != bytes) {
        return PreosStatus::inconsistent_timing;
    }

    if (vram.bytes == 0 || vram.mc_end < vram.mc_start || fb_start != vram.mc_start ||
        fb_end < vram.mc_end ||
        (static_cast<uint64_t>(registers.fb_offset) << 24) != vram.physical_offset ||
        (address % surface_alignment) != 0 || address < vram.mc_start || address > vram.mc_end ||
        bytes - 1 > vram.mc_end - address) {
        return PreosStatus::outside_framebuffer;
    }

    const uint64_t offset = address - vram.mc_start;

    if (offset >= visible || bytes > visible - offset) {
        return PreosStatus::not_cpu_visible;
    }

    scanout.stride_bytes = stride;
    scanout.mc_address = address;
    scanout.vram_offset = offset;
    scanout.bytes = bytes;
    return PreosStatus::adopted;
}

bool lifetime_stolen(const hardware::VramReservation& entry) noexcept {
    return entry.lifetime && (entry.kind == hardware::VramReservationKind::stolen_vga ||
                              entry.kind == hardware::VramReservationKind::stolen_extended);
}

bool overlap(uint64_t a, uint64_t a_bytes, uint64_t b, uint64_t b_bytes) noexcept {
    return a_bytes != 0 && b_bytes != 0 && a < b + b_bytes && b < a + a_bytes;
}

} // namespace

hardware::DiscoveryStatus validate_preos_discovery(ByteView binary, uint32_t aperture,
                                                   PreosDcnRegisters& output) noexcept {
    output = {};

    if (aperture == 0 || (aperture & 3u) != 0) {
        return hardware::DiscoveryStatus::invalid_argument;
    }

    hardware::DiscoveredIp dmu;
    const auto status = hardware::discover_ip(binary, {hardware::discovery_dmu_hardware_id, 0, 3, 2, 0, 4}, dmu);

    if (status != hardware::DiscoveryStatus::validated) {
        return status;
    }

    const uint32_t dccg = dmu.segment(segment_dccg);
    const uint32_t dcn = dmu.segment(segment_dcn);
    const uint32_t mpc = dmu.segment(segment_mpc);
    const uint32_t hubbub[]{fb_location_base, fb_location_top, fb_offset};

    if (!fits(dccg, vga_control, aperture) || !fits(dccg, pixel_rate_control, aperture) ||
        !fits(dccg, dto_phase, aperture) || !fits(dccg, dto_modulo, aperture) ||
        !fits(dcn, hubbub, aperture) || !fits(dcn, otg_control, aperture) ||
        !fits(dcn, otg_clock_control, aperture) || !fits(dcn, otg_h_total, aperture) ||
        !fits(dcn, otg_h_blank, aperture) || !fits(dcn, otg_h_sync, aperture) ||
        !fits(dcn, otg_h_sync_control, aperture) || !fits(dcn, otg_h_timing_control, aperture) ||
        !fits(dcn, otg_v_total, aperture) || !fits(dcn, otg_v_total_control, aperture) ||
        !fits(dcn, otg_v_blank, aperture) || !fits(dcn, otg_v_sync, aperture) ||
        !fits(dcn, otg_v_sync_control, aperture) || !fits(dcn, otg_interlace_control, aperture) ||
        !fits(dcn, otg_frame_count, aperture) || !fits(dcn, odm_data_source_select, aperture) ||
        !fits(dcn, hubp_control, aperture) || !fits(dcn, hubp_clock_control, aperture) ||
        !fits(dcn, surface_config, aperture) || !fits(dcn, tiling_config, aperture) ||
        !fits(dcn, viewport_start, aperture) || !fits(dcn, viewport_dimension, aperture) ||
        !fits(dcn, surface_pitch, aperture) || !fits(dcn, address_low, aperture) ||
        !fits(dcn, address_high, aperture) || !fits(dcn, surface_control, aperture) ||
        !fits(dcn, vmid_settings, aperture) || !fits(dcn, hubpret_control, aperture) ||
        !fits(dcn, dscl_mode, aperture) || !fits(dcn, recout_start, aperture) ||
        !fits(dcn, recout_size, aperture) || !fits(mpc, mpc_out_mux, aperture) ||
        !fits(mpc, mpcc_top_select, aperture) || !fits(mpc, mpcc_bottom_select, aperture) ||
        !fits(mpc, mpcc_opp_id, aperture) || !fits(mpc, mpcc_control, aperture)) {
        return hardware::DiscoveryStatus::invalid_argument;
    }

    output.dmu_ = dmu;
    output.aperture_bytes_ = aperture;
    return hardware::DiscoveryStatus::validated;
}

bool read_preos_scanout(const hardware::RegisterIo& io, const PreosDcnRegisters& registers,
                        PreosScanoutRegisters& output) noexcept {
    output = {};

    if (!registers.valid() || io.aperture_bytes != registers.aperture_bytes()) {
        return false;
    }

    Reader reader{io, registers};
    PreosScanoutRegisters snapshot{};
    snapshot.fb_location_base = reader.read(segment_dcn, fb_location_base);
    snapshot.fb_location_top = reader.read(segment_dcn, fb_location_top);
    snapshot.fb_offset = reader.read(segment_dcn, fb_offset);

    for (uint32_t index = 0; index < preos_pipe_count; ++index) {
        snapshot.vga_control[index] = reader.read(segment_dccg, vga_control[index]);

        PreosOtgRegisters& otg = snapshot.otg[index];
        otg.control = reader.read(segment_dcn, otg_control[index]);
        otg.clock_control = reader.read(segment_dcn, otg_clock_control[index]);
        otg.h_total = reader.read(segment_dcn, otg_h_total[index]);
        otg.h_blank = reader.read(segment_dcn, otg_h_blank[index]);
        otg.h_sync = reader.read(segment_dcn, otg_h_sync[index]);
        otg.h_sync_control = reader.read(segment_dcn, otg_h_sync_control[index]);
        otg.h_timing_control = reader.read(segment_dcn, otg_h_timing_control[index]);
        otg.v_total = reader.read(segment_dcn, otg_v_total[index]);
        otg.v_blank = reader.read(segment_dcn, otg_v_blank[index]);
        otg.v_sync = reader.read(segment_dcn, otg_v_sync[index]);
        otg.v_sync_control = reader.read(segment_dcn, otg_v_sync_control[index]);
        otg.interlace_control = reader.read(segment_dcn, otg_interlace_control[index]);
        otg.v_total_control = reader.read(segment_dcn, otg_v_total_control[index]);
        otg.data_source_select = reader.read(segment_dcn, odm_data_source_select[index]);
        otg.pixel_rate_control = reader.read(segment_dccg, pixel_rate_control[index]);
        otg.dto_phase = reader.read(segment_dccg, dto_phase[index]);
        otg.dto_modulo = reader.read(segment_dccg, dto_modulo[index]);

        PreosPipeRegisters& pipe = snapshot.pipe[index];
        pipe.hubp_control = reader.read(segment_dcn, hubp_control[index]);
        pipe.hubp_clock_control = reader.read(segment_dcn, hubp_clock_control[index]);
        pipe.surface_config = reader.read(segment_dcn, surface_config[index]);
        pipe.tiling_config = reader.read(segment_dcn, tiling_config[index]);
        pipe.viewport_start = reader.read(segment_dcn, viewport_start[index]);
        pipe.viewport_dimension = reader.read(segment_dcn, viewport_dimension[index]);
        pipe.surface_pitch = reader.read(segment_dcn, surface_pitch[index]);
        pipe.address_low = reader.read(segment_dcn, address_low[index]);
        pipe.address_high = reader.read(segment_dcn, address_high[index]);
        pipe.surface_control = reader.read(segment_dcn, surface_control[index]);
        pipe.vmid_settings = reader.read(segment_dcn, vmid_settings[index]);
        pipe.hubpret_control = reader.read(segment_dcn, hubpret_control[index]);
        pipe.dscl_mode = reader.read(segment_dcn, dscl_mode[index]);
        pipe.recout_start = reader.read(segment_dcn, recout_start[index]);
        pipe.recout_size = reader.read(segment_dcn, recout_size[index]);
        pipe.mpc_out_mux = reader.read(segment_mpc, mpc_out_mux[index]);
        pipe.mpcc_top_select = reader.read(segment_mpc, mpcc_top_select[index]);
        pipe.mpcc_bottom_select = reader.read(segment_mpc, mpcc_bottom_select[index]);
        pipe.mpcc_opp_id = reader.read(segment_mpc, mpcc_opp_id[index]);
        pipe.mpcc_control = reader.read(segment_mpc, mpcc_control[index]);
    }

    if (!reader.ok) {
        return false;
    }

    output = snapshot;
    return true;
}

bool read_preos_frame_count(const hardware::RegisterIo& io, const PreosDcnRegisters& registers,
                            uint32_t otg, uint32_t& frame_count) noexcept {
    uint32_t value = 0;

    if (!registers.valid() || io.aperture_bytes != registers.aperture_bytes() ||
        otg >= preos_pipe_count ||
        !read_ip_register(io, registers.segment(segment_dcn), otg_frame_count[otg], value) ||
        value == UINT32_MAX) {
        return false;
    }

    frame_count = value & 0x00ffffffu; // OTG_FRAME_COUNT
    return true;
}

PreosStatus decode_preos_scanout(const PreosScanoutRegisters& registers,
                                 const hardware::VramLocation& vram, uint64_t visible,
                                 AdoptedScanout& output, PreosReport& report) noexcept {
    report = {};
    report.primary_otg = preos_none;

    for (uint32_t& pipe : report.otg_pipe) {
        pipe = preos_none;
    }

    const auto finish = [&report](PreosStatus status) noexcept {
        report.status = status;
        return status;
    };

    if (visible == 0) {
        return finish(PreosStatus::invalid_argument);
    }

    // Identity registers read as all ones only when the device stopped decoding.
    if (registers.fb_location_base == UINT32_MAX || registers.fb_location_top == UINT32_MAX) {
        return finish(PreosStatus::read_failure);
    }

    for (uint32_t index = 0; index < preos_pipe_count; ++index) {
        if (registers.vga_control[index] == UINT32_MAX || registers.otg[index].control == UINT32_MAX ||
            registers.pipe[index].hubp_control == UINT32_MAX) {
            return finish(PreosStatus::read_failure);
        }

        if ((registers.vga_control[index] & vga_mode_enable) != 0) {
            report.vga_mask |= 1u << index;
        }

        if ((registers.pipe[index].hubp_control & hubp_underflow) != 0) {
            report.underflow_mask |= 1u << index;
        }
    }

    // gmc_v11_0_get_vbios_fb_size treats D1VGA mode as the 9 MiB VGA allocation:
    // the VGA engine scans legacy VGA memory, not a HUBP linear surface.
    if (report.vga_mask != 0) {
        return finish(PreosStatus::legacy_vga);
    }

    for (uint32_t index = 0; index < preos_pipe_count; ++index) {
        const uint32_t control = registers.otg[index].control;
        const bool requested = (control & otg_master_enable) != 0;
        const bool current = (control & otg_current_master_enable) != 0;

        if (requested != current) {
            return finish(PreosStatus::inconsistent_state);
        }

        if (!requested) {
            continue;
        }

        report.active_otg_mask |= 1u << index;
        uint32_t opp = 0;
        uint32_t mpcc = 0;
        uint32_t pipe = 0;

        if (resolve_pipe(registers, index, opp, mpcc, pipe) == PreosStatus::adopted) {
            report.otg_pipe[index] = pipe;
        }
    }

    if (report.active_otg_mask == 0) {
        return finish(PreosStatus::no_active_pipe);
    }

    // One HUBP scanned by two OTGs (two MPCC trees selecting the same DPP).
    for (uint32_t index = 0; index < preos_pipe_count; ++index) {
        for (uint32_t other = index + 1; other < preos_pipe_count; ++other) {
            if (report.otg_pipe[index] != preos_none && report.otg_pipe[index] == report.otg_pipe[other]) {
                return finish(PreosStatus::unsupported_topology);
            }
        }
    }

    // Single active OTG, or the one HUBP0 feeds (the pipe whose size Linux reserves).
    for (uint32_t index = 0; index < preos_pipe_count; ++index) {
        if ((report.active_otg_mask & (1u << index)) == 0) {
            continue;
        }

        if (report.active_otg_mask == (1u << index)) {
            report.primary_otg = index;
            break;
        }

        // Pipes are unique per OTG (checked above), so at most one OTG has HUBP0.
        if (report.otg_pipe[index] == 0) {
            report.primary_otg = index;
        }
    }

    if (report.primary_otg == preos_none) {
        return finish(PreosStatus::ambiguous_primary);
    }

    const uint32_t otg_index = report.primary_otg;
    const PreosOtgRegisters& otg = registers.otg[otg_index];
    AdoptedScanout scanout{};
    scanout.otg = otg_index;
    scanout.secondary_otg_mask = report.active_otg_mask & ~(1u << otg_index);

    PreosStatus status = resolve_pipe(registers, otg_index, scanout.opp, scanout.mpcc, scanout.pipe);

    if (status == PreosStatus::adopted) {
        status = check_otg(otg);
    }

    if (status == PreosStatus::adopted) {
        status = decode_timing(otg, scanout.timing);
    }

    if (status == PreosStatus::adopted) {
        status = check_plane(registers.pipe[scanout.pipe], registers.pipe[scanout.mpcc], otg_index,
                             scanout.timing);
    }

    if (status == PreosStatus::adopted) {
        status = place_surface(registers, registers.pipe[scanout.pipe], vram, visible, scanout);
    }

    if (status != PreosStatus::adopted) {
        return finish(status);
    }

    scanout.variable_refresh = (otg.v_total_control & v_total_select) != 0;
    decode_clock(otg, scanout);
    output = scanout;
    return finish(PreosStatus::adopted);
}

bool preos_refresh_from_frames(uint32_t first, uint32_t last, uint64_t ticks,
                               uint64_t ticks_per_second, uint32_t& millihz) noexcept {
    const uint64_t frames = (static_cast<uint64_t>(last) - first) & 0x00ffffffu;

    // A measurement spans at most a few hundred frames; with frames <= 4096 and
    // ticks_per_second < 2^32 the product frames * ticks_per_second * 1000 < 2^54.
    if (frames == 0 || frames > 4096 || ticks == 0 || ticks_per_second == 0 ||
        ticks_per_second > UINT32_MAX) {
        return false;
    }

    const uint64_t numerator = frames * ticks_per_second * 1000u;
    const uint64_t value = (numerator + ticks / 2) / ticks;

    if (value < preos_minimum_refresh_millihz || value > preos_maximum_refresh_millihz) {
        return false;
    }

    millihz = static_cast<uint32_t>(value);
    return true;
}

bool apply_preos_measured_refresh(AdoptedScanout& scanout, uint32_t millihz) noexcept {
    if (scanout.clock_source != PreosClockSource::unknown || scanout.variable_refresh ||
        millihz < preos_minimum_refresh_millihz || millihz > preos_maximum_refresh_millihz) {
        return false;
    }

    // Totals are at most 2^15 each and millihz below 2^19: the product fits uint64.
    const uint64_t pixel_hz_times_1000 = static_cast<uint64_t>(millihz) * scanout.timing.h_total *
                                         scanout.timing.v_total;
    const uint64_t khz = (pixel_hz_times_1000 + 500000u) / 1000000u;

    if (khz == 0 || khz > UINT32_MAX) {
        return false;
    }

    scanout.timing.pixel_clock_khz = static_cast<uint32_t>(khz);
    scanout.refresh_millihz = millihz;
    return true;
}

bool same_preos_scanout(const AdoptedScanout& a, const AdoptedScanout& b) noexcept {
    const Timing& x = a.timing;
    const Timing& y = b.timing;
    const bool clocks = a.clock_source == b.clock_source &&
                        (a.clock_source != PreosClockSource::dp_dto ||
                         x.pixel_clock_khz == y.pixel_clock_khz);

    return a.otg == b.otg && a.pipe == b.pipe && a.opp == b.opp && a.mpcc == b.mpcc && clocks &&
           x.width == y.width && x.h_total == y.h_total && x.h_front_porch == y.h_front_porch &&
           x.h_sync_width == y.h_sync_width && x.height == y.height && x.v_total == y.v_total &&
           x.v_front_porch == y.v_front_porch && x.v_sync_width == y.v_sync_width &&
           x.flags == y.flags && a.variable_refresh == b.variable_refresh &&
           a.stride_bytes == b.stride_bytes && a.mc_address == b.mc_address &&
           a.vram_offset == b.vram_offset && a.bytes == b.bytes;
}

PreosCoverage check_preos_reservations(const hardware::VramReservationMap& map,
                                       const AdoptedScanout& scanout,
                                       PreosReservationCheck& output) noexcept {
    output = {PreosCoverage::unreserved, 0, 0};
    const uint64_t begin = scanout.vram_offset;
    const uint64_t bytes = scanout.bytes;

    if (bytes == 0 || begin > UINT64_MAX - bytes || map.count > hardware::VramReservationMap::capacity ||
        begin + bytes > UINT64_MAX - (page_bytes - 1)) {
        return output.coverage;
    }

    const uint64_t end = begin + bytes;
    uint64_t cursor = begin;

    // Follow adjacent stolen entries (stolen_vga then stolen_extended); each round
    // advances past one entry, so capacity rounds suffice.
    for (uint32_t round = 0; round < hardware::VramReservationMap::capacity && cursor < end; ++round) {
        bool advanced = false;

        for (uint32_t index = 0; index < map.count; ++index) {
            const hardware::VramReservation& entry = map.entries[index];

            if (lifetime_stolen(entry) && entry.offset <= cursor && cursor - entry.offset < entry.bytes) {
                cursor = entry.offset + entry.bytes;
                advanced = true;
                break;
            }
        }

        if (!advanced) {
            break;
        }
    }

    if (cursor >= end) {
        output.coverage = PreosCoverage::reserved;
        return output.coverage;
    }

    if (cursor == begin || (cursor % page_bytes) != 0) {
        return output.coverage;
    }

    const uint64_t tail_end = (end + page_bytes - 1) & ~(page_bytes - 1);
    const uint64_t tail_bytes = tail_end - cursor;
    // Linux reserves height * PITCH * 4 = bytes - height * 4 (PITCH is pitch - 1).
    const uint64_t undercount = (static_cast<uint64_t>(scanout.timing.height) * xrgb8888_bytes_per_pixel +
                                 page_bytes - 1) & ~(page_bytes - 1);

    if (tail_bytes > undercount + page_bytes || tail_end > map.vram_bytes) {
        return output.coverage;
    }

    for (uint32_t index = 0; index < map.count; ++index) {
        const hardware::VramReservation& entry = map.entries[index];

        if (entry.lifetime && !lifetime_stolen(entry) &&
            overlap(cursor, tail_bytes, entry.offset, entry.bytes)) {
            return output.coverage;
        }
    }

    output = {PreosCoverage::tail_unreserved, cursor, tail_bytes};
    return output.coverage;
}

} // namespace rx7900::display

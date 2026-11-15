/*
Module: rx7900.display.preos_scanout
Purpose: Decide whether the pre-OS (VBIOS/GOP) DCN3.2 scanout can be adopted as a linear framebuffer.
Created: 2026-10-22
Architecture: Read-only register snapshot through the shared hardware::RegisterIo,
then pure decisions that reuse display timing/framebuffer validation and the
hardware VRAM reservation map. Nothing here writes a display register; the
native adapter (src/reactos/preos_scanout_native) owns MMIO, mapping and IOCTLs.
Copyright 2012-2023 Advanced Micro Devices, Inc. Register offsets and fields are
the pinned Linux v6.12 dcn_3_2_0_offset.h/_sh_mask.h values; decode rules follow
optc1_program_timing/optc1_read_otg_state (dcn10_optc.c), optc32_set_odm_bypass,
hubp2_program_pixel_format/rotation/size and hubp3 tiling/DCC (dcn20_hubp.c,
dcn30_hubp.c), mpc1_insert_plane/mpc1_read_mpcc_state (dcn10_mpc.c),
dpp1_dscl_get_dscl_mode/set_recout/set_scl_filter (dcn10_dpp_dscl.c),
hubbub3_init_dchub_sys_ctx (dcn30_hubbub.c), mmhub_read_system_context
(amdgpu_dm.c), get_pixel_clk_frequency_100hz (dce_clock_source.c) and
gmc_v11_0_get_vbios_fb_size (gmc_v11_0.c). Notice: licenses/AMDGPU-DISPLAY.txt.
*/
#pragma once

#include "modes.hpp"
#include "nbio.hpp"
#include "vram_reservations.hpp"

namespace rx7900::display {

// DCN 3.2.0 (Navi31): four OTG/ODM/MPCC/HUBP/DPP instances (dcn32_resource.c).
constexpr uint32_t preos_pipe_count = 4;
// The decoder value for an unresolved pipe or OTG.
constexpr uint32_t preos_none = UINT32_MAX;
// Refresh range accepted from a pixel clock or a frame-counter measurement.
constexpr uint32_t preos_minimum_refresh_millihz = 10000;
constexpr uint32_t preos_maximum_refresh_millihz = 500000;

// Only validate_preos_discovery creates a valid token: the unique unharvested DMU
// instance0 3.2.0 (discovery_dmu_hardware_id) with segments 1 (BASE_IDX 1: VGA,
// DCCG), 2 (BASE_IDX 2: HUBBUB, HUBP, DPP, ODM, OTG) and 3 (BASE_IDX 3: MPC),
// and every snapshot register inside the BAR5 aperture.
class PreosDcnRegisters final {
public:
    PreosDcnRegisters() noexcept = default;
    bool valid() const noexcept { return dmu_.valid() && aperture_bytes_ != 0; }
    uint32_t segment(size_t index) const noexcept { return dmu_.segment(index); }
    uint32_t aperture_bytes() const noexcept { return aperture_bytes_; }

private:
    friend hardware::DiscoveryStatus validate_preos_discovery(ByteView, uint32_t,
                                                               PreosDcnRegisters&) noexcept;
    hardware::DiscoveredIp dmu_{};
    uint32_t aperture_bytes_ = 0;
};

hardware::DiscoveryStatus validate_preos_discovery(ByteView binary, uint32_t register_aperture_bytes,
                                                   PreosDcnRegisters& output) noexcept;

// Raw register values, one array element per instance. No field is interpreted
// while reading; volatile status bits (frame count, underflow, in-blank) are kept
// raw and never compared between snapshots.
struct PreosOtgRegisters final {
    uint32_t control;            // OTG_CONTROL
    uint32_t clock_control;      // OTG_CLOCK_CONTROL
    uint32_t h_total;            // OTG_H_TOTAL
    uint32_t h_blank;            // OTG_H_BLANK_START_END
    uint32_t h_sync;             // OTG_H_SYNC_A
    uint32_t h_sync_control;     // OTG_H_SYNC_A_CNTL
    uint32_t h_timing_control;   // OTG_H_TIMING_CNTL
    uint32_t v_total;            // OTG_V_TOTAL
    uint32_t v_blank;            // OTG_V_BLANK_START_END
    uint32_t v_sync;             // OTG_V_SYNC_A
    uint32_t v_sync_control;     // OTG_V_SYNC_A_CNTL
    uint32_t interlace_control;  // OTG_INTERLACE_CONTROL
    uint32_t v_total_control;    // OTG_V_TOTAL_CONTROL
    uint32_t data_source_select; // ODMn_OPTC_DATA_SOURCE_SELECT
    uint32_t pixel_rate_control; // DCCG OTGn_PIXEL_RATE_CNTL
    uint32_t dto_phase;          // DCCG DP_DTOn_PHASE
    uint32_t dto_modulo;         // DCCG DP_DTOn_MODULO
};

struct PreosPipeRegisters final {
    uint32_t hubp_control;       // HUBPn_DCHUBP_CNTL
    uint32_t hubp_clock_control; // HUBPn_HUBP_CLK_CNTL
    uint32_t surface_config;     // HUBPn_DCSURF_SURFACE_CONFIG
    uint32_t tiling_config;      // HUBPn_DCSURF_TILING_CONFIG
    uint32_t viewport_start;     // HUBPn_DCSURF_PRI_VIEWPORT_START
    uint32_t viewport_dimension; // HUBPn_DCSURF_PRI_VIEWPORT_DIMENSION
    uint32_t surface_pitch;      // HUBPREQn_DCSURF_SURFACE_PITCH
    uint32_t address_low;        // HUBPREQn_DCSURF_PRIMARY_SURFACE_ADDRESS
    uint32_t address_high;       // HUBPREQn_DCSURF_PRIMARY_SURFACE_ADDRESS_HIGH
    uint32_t surface_control;    // HUBPREQn_DCSURF_SURFACE_CONTROL
    uint32_t vmid_settings;      // HUBPREQn_VMID_SETTINGS_0
    uint32_t hubpret_control;    // HUBPRETn_HUBPRET_CONTROL
    uint32_t dscl_mode;          // DSCLn_SCL_MODE
    uint32_t recout_start;       // DSCLn_RECOUT_START
    uint32_t recout_size;        // DSCLn_RECOUT_SIZE
    uint32_t mpc_out_mux;        // MPC_OUTn_MUX (indexed by OPP)
    uint32_t mpcc_top_select;    // MPCCn_MPCC_TOP_SEL (indexed by MPCC)
    uint32_t mpcc_bottom_select; // MPCCn_MPCC_BOT_SEL
    uint32_t mpcc_opp_id;        // MPCCn_MPCC_OPP_ID
    uint32_t mpcc_control;       // MPCCn_MPCC_CONTROL
};

struct PreosScanoutRegisters final {
    uint32_t vga_control[preos_pipe_count]; // D1..D4VGA_CONTROL
    uint32_t fb_location_base;              // DCN_VM_FB_LOCATION_BASE
    uint32_t fb_location_top;               // DCN_VM_FB_LOCATION_TOP
    uint32_t fb_offset;                     // DCN_VM_FB_OFFSET
    PreosOtgRegisters otg[preos_pipe_count];
    PreosPipeRegisters pipe[preos_pipe_count];
};

// PASSIVE_LEVEL, exclusive register owner, before any display programming.
// Register reads only (no index/data window, no write callback). Output is
// zeroed first and complete only on true; false on any access failure.
bool read_preos_scanout(const hardware::RegisterIo& io, const PreosDcnRegisters& registers,
                        PreosScanoutRegisters& output) noexcept;

// OTG_STATUS_FRAME_COUNT (24-bit) of one OTG; read only. For refresh measurement.
bool read_preos_frame_count(const hardware::RegisterIo& io, const PreosDcnRegisters& registers,
                            uint32_t otg, uint32_t& frame_count) noexcept;

enum class PreosStatus {
    adopted,
    invalid_argument,
    // An identity register read all ones: the device is not responding.
    read_failure,
    no_active_pipe,
    // A DnVGA_CONTROL.DnVGA_MODE_ENABLE bit is set: the VGA engine is producing
    // text/planar/palette output from legacy VGA memory. Never adoptable.
    legacy_vga,
    // Several active OTGs and none (or more than one) is sourced by HUBP0.
    ambiguous_primary,
    // ODM combine, OTG pixel division, MPC split/blend trees or mismatched routing.
    unsupported_topology,
    // OTG master-enable request and current state disagree, reserved bits set, or
    // the OTG clock is off/in reset while enabled.
    inconsistent_state,
    interlaced,
    inconsistent_timing,
    // HUBP blanked, soft reset or clock gated: nothing is being fetched.
    plane_disabled,
    // Not SURFACE_PIXEL_FORMAT 8 with the ARGB crossbar, or an alpha plane.
    unsupported_format,
    // SW_MODE != linear or primary DCC enabled.
    tiled_or_compressed,
    rotated_or_mirrored,
    // Viewport offset, viewport/RECOUT not equal to the OTG active area, or DSCL scaling.
    scaled,
    // MPCC per-pixel alpha would let GDI's undefined X byte change the output.
    alpha_dependent,
    // PRIMARY_SURFACE_TMZ: protected memory.
    protected_surface,
    // VMID != 0: the address is a GPU virtual address, not an MC address.
    gpu_virtual_address,
    // DCN FB aperture differs from MMHUB, or the surface is outside local VRAM.
    outside_framebuffer,
    // Some byte of the surface is beyond the CPU-visible BAR0 window.
    not_cpu_visible,
    // Surface not wholly inside the stolen pre-OS reservation (see coverage).
    unreserved,
    // A post-initialization snapshot no longer describes the same scanout.
    invalidated
};

enum class PreosClockSource {
    // DCCG DP DTO enabled: VBIOS programs MODULO = DPREFCLK (Hz), so PHASE is the
    // pixel clock in Hz (get_pixel_clk_frequency_100hz).
    dp_dto,
    // PHY PLL (HDMI/DVI) or DTBCLK DTO: not derivable without clock-manager state.
    unknown
};

// The validated description of the adopted pre-OS scanout. timing is the decoded
// OTG timing; timing.pixel_clock_khz and refresh_millihz are 0 when unknown.
struct AdoptedScanout final {
    uint32_t otg;
    uint32_t pipe; // HUBP/DPP instance
    uint32_t opp;
    uint32_t mpcc;
    Timing timing;
    uint32_t refresh_millihz;
    PreosClockSource clock_source;
    // OTG_V_TOTAL_CONTROL min/max select: variable refresh, refresh left unknown.
    bool variable_refresh;
    // Linear 32 bpp, memory order B, G, R, X (GDI masks 0x00ff0000/0x0000ff00/0x000000ff).
    uint32_t stride_bytes;
    // MC address of the first pixel; vram_offset = mc_address - VramLocation.mc_start,
    // which is also the BAR0 offset. bytes = stride_bytes * height.
    uint64_t mc_address;
    uint64_t vram_offset;
    uint64_t bytes;
    // Other active OTGs (reported, never adopted or touched).
    uint32_t secondary_otg_mask;
};

struct PreosReport final {
    PreosStatus status;
    uint32_t active_otg_mask;
    uint32_t vga_mask;
    uint32_t primary_otg;
    // HUBP/DPP feeding each OTG, preos_none when it could not be resolved.
    uint32_t otg_pipe[preos_pipe_count];
    // Pipes whose HUBP_UNDERFLOW_STATUS is nonzero (sticky; informational).
    uint32_t underflow_mask;
};

// Pure. vram: MMHUB FB_LOCATION/FB_OFFSET and RCC_CONFIG_MEMSIZE (Nbio::vram_location).
// visible_vram_bytes: CPU-reachable BAR0 window (min of BAR0 and VRAM). Decision
// order is the PreosStatus order above; output is written only on adopted, the
// report always. See the .cpp for every field rule.
PreosStatus decode_preos_scanout(const PreosScanoutRegisters& registers,
                                 const hardware::VramLocation& vram, uint64_t visible_vram_bytes,
                                 AdoptedScanout& output, PreosReport& report) noexcept;

// Pure. Refresh from two OTG_STATUS_FRAME_COUNT samples taken at frame edges
// (24-bit wrap) ticks apart at ticks_per_second. False outside the accepted range.
bool preos_refresh_from_frames(uint32_t first_count, uint32_t last_count, uint64_t ticks,
                               uint64_t ticks_per_second, uint32_t& millihz) noexcept;

// Records a measured refresh for a scanout whose pixel clock is unknown and
// derives timing.pixel_clock_khz = refresh * h_total * v_total. Fails (unchanged)
// for an out-of-range value, a known clock or variable refresh.
bool apply_preos_measured_refresh(AdoptedScanout& scanout, uint32_t millihz) noexcept;

// True when two decodes describe the same scanout: pipe routing, timing geometry,
// sync polarity, stride, MC address, VRAM offset and size. Measured refresh is
// ignored; a decoded DTO clock must match.
bool same_preos_scanout(const AdoptedScanout& before, const AdoptedScanout& after) noexcept;

enum class PreosCoverage {
    // Wholly inside lifetime stolen_vga/stolen_extended reservations.
    reserved,
    // Covered from its start, except a page-rounded tail no larger than the
    // Linux pitch undercount: gmc_v11_0_get_vbios_fb_size multiplies the PITCH
    // field (pitch - 1) instead of the pitch, so the last height * 4 bytes (page
    // rounded) can fall outside. The tail meets no other lifetime reservation;
    // the owner must claim it before any allocator can hand it out.
    tail_unreserved,
    unreserved
};

struct PreosReservationCheck final {
    PreosCoverage coverage;
    uint64_t tail_offset;
    uint64_t tail_bytes;
};

// Pure check against the map the allocator is (or would be) initialized from.
PreosCoverage check_preos_reservations(const hardware::VramReservationMap& map,
                                       const AdoptedScanout& scanout,
                                       PreosReservationCheck& output) noexcept;

} // namespace rx7900::display

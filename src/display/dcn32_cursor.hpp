/*
Module: rx7900.display.dcn32_cursor
Purpose: Program the DCN 3.2 hardware cursor of the adopted pre-OS pipe and encode cursor images.
Created: 2026-10-22
Architecture: Register protocol over the shared hardware::RegisterIo, beside the
read-only src/display/preos_scanout snapshot whose AdoptedScanout names the
HUBP/DPP, OPP and OTG instances. Pure image encoding and position arithmetic are
separate functions so host tests reach them without hardware. The native owner
(src/reactos/radeon_cursor_native) supplies MMIO, the VRAM image buffer and its
HDP flush, and serializes every call.
Copyright 2012-2023 Advanced Micro Devices, Inc. Adapted (no source text copied
verbatim) from the pinned Linux v6.12 AMD display core: hubp32_cursor_set_attributes
(dcn32_hubp.c), hubp2_cursor_set_position and hubp2_get_lines_per_chunk
(dcn20_hubp.c), hubp1_get_cursor_pitch (dcn10_hubp.c), dpp3_set_cursor_attributes
(dcn30_dpp.c), dpp1_set_cursor_position and dpp1_cnv_set_optional_cursor_attributes
(dcn10_dpp.c), dcn10_set_cursor_position, dcn10_set_cursor_attribute,
dcn10_set_cursor_sdr_white_level, dcn10_cursor_lock, delay_cursor_until_vupdate,
dcn10_get_vupdate_offset_from_vsync and dcn10_calc_vupdate_position (dcn10_hwseq.c),
mpc1_cursor_lock (dcn10_mpc.c), program_cursor_attributes/program_cursor_position
(dc_stream.c), hubbub2_get_dchub_ref_freq (dcn20_hubbub.c), dccg32_get_dccg_ref_freq
(dcn32_dccg.c), the dce_info crystal frequency of bios_parser2.c, max_cursor_size
(dcn32_resource.c) and amdgpu_dm_plane_get_cursor_position (amdgpu_dm_plane.c).
Register offsets and fields: dcn_3_2_0_offset.h/_sh_mask.h. These files carry the
AMD MIT-style permission notice; see licenses/AMDGPU-DISPLAY.txt.
*/
#pragma once

#include "atom_bios.hpp"
#include "preos_scanout.hpp"

namespace rx7900::display {

// dcn32_resource.c (v6.12): dc->caps.max_cursor_size = 64 on DCN 3.2 (larger
// cursors are disabled there because of SubVP cursor corruption).
constexpr uint32_t dcn32_cursor_max_dimension = 64;
// The image is always a full 64 x 64 surface with transparent padding, exactly the
// configuration Linux programs: dm_check_cursor_fb requires FB width == pitch and
// pitch in {64, 128, 256}; with a 64-pixel maximum that is 64 x 64, pitch 64.
constexpr uint32_t dcn32_cursor_pitch_pixels = 64;
constexpr uint32_t dcn32_cursor_pixels = dcn32_cursor_pitch_pixels * dcn32_cursor_max_dimension;
constexpr uint32_t dcn32_cursor_image_bytes = dcn32_cursor_pixels * 4;
// CURSOR_SURFACE_ADDRESS has no low-order field restriction in the register
// header; Linux cursor BOs are page-aligned VRAM BOs. The same page alignment is
// used for every image slot (and is a multiple of DCN's 256-byte surface rule).
constexpr uint64_t dcn32_cursor_alignment = 4096;

// dc_cursor_color_format, programmed unchanged into CURSOR_MODE and CUR0_MODE.
enum class CursorMode : uint32_t {
    mono = 0,
    color_1bit_and = 1,
    premultiplied_alpha = 2,
    straight_alpha = 3
};

// ---------------------------------------------------------------------------
// Image encoding (pure). A source shape is described by borrowed rows; the
// encoder produces the DCN 32 bpp image (words 0xAARRGGBB, little-endian bytes
// B, G, R, A) and the CURSOR_MODE it must be scanned with.
// ---------------------------------------------------------------------------
enum class CursorSourceFormat : uint32_t {
    // AND plane (height rows) then XOR plane (height rows), 1 bpp MSB first:
    // AND 0 XOR 0 black, AND 0 XOR 1 white, AND 1 XOR 0 transparent,
    // AND 1 XOR 1 inverts the screen (GDI monochrome pointer semantics).
    monochrome,
    // 1 bpp AND mask (height rows, mask_stride) and 32 bpp color rows; the GDI
    // result is (screen AND mask) XOR color. The color X byte is ignored.
    masked_color,
    // 32 bpp B, G, R, A with color premultiplied by alpha (GDI SPS_ALPHA).
    premultiplied_alpha,
    // 32 bpp B, G, R, A with straight (non-premultiplied) color.
    straight_alpha
};

struct CursorSource final {
    CursorSourceFormat format = CursorSourceFormat::monochrome;
    uint32_t width = 0;
    uint32_t height = 0;
    ByteView mask{nullptr, 0};
    uint32_t mask_stride = 0;
    ByteView color{nullptr, 0};
    uint32_t color_stride = 0;
};

enum class CursorImageStatus {
    built,
    invalid_argument,
    // A dimension is zero or above dcn32_cursor_max_dimension.
    unsupported_size,
    // A plane is shorter than its declared rows, or a stride is too small.
    truncated
};

// Device-owned (16 KiB), trivially destructible. pixels[y * pitch + x].
struct CursorImage final {
    CursorMode mode;
    // Source dimensions (informational; the programmed size is always 64 x 64).
    uint32_t width;
    uint32_t height;
    uint32_t pixels[dcn32_cursor_pixels];
};

// Monochrome and masked shapes whose screen-dependent pixels are all
// transparent (AND 1 with XOR/color 0) become exact premultiplied ARGB, the
// mode Linux uses for every cursor (amdgpu_dm_update_cursor). Any pixel that
// inverts or XORs the screen needs CURSOR_MODE_COLOR_1BIT_AND: the AND bit is the
// MSB of the alpha byte and the color occupies the low 24 bits. (That bit
// position is the DCE/DCN 1-bit-AND convention; Linux v6.12 never selects this
// mode, so its encoding needs hardware confirmation.) Alpha shapes are copied
// unchanged in their own mode. Padding outside width x height is transparent
// in the chosen mode. Output is written only on built.
CursorImageStatus build_cursor_image(const CursorSource& source, CursorImage& output) noexcept;

// ---------------------------------------------------------------------------
// Register token and reference clock.
// ---------------------------------------------------------------------------
// Only validate_dcn32_cursor_discovery creates a valid token: the unique DMU
// 3.2.0 instance (segments 2: HUBBUB/HUBP/DPP/OTG and 3: MPC) with every cursor
// register of the given HUBP/DPP pipe, OPP cursor lock and OTG inside the BAR5
// aperture.
class Dcn32CursorRegisters final {
public:
    Dcn32CursorRegisters() noexcept = default;
    bool valid() const noexcept { return dcn_ != 0 && aperture_bytes_ != 0; }
    uint32_t aperture_bytes() const noexcept { return aperture_bytes_; }
    uint32_t pipe() const noexcept { return pipe_; }
    uint32_t opp() const noexcept { return opp_; }
    uint32_t otg() const noexcept { return otg_; }

private:
    friend hardware::DiscoveryStatus validate_dcn32_cursor_discovery(ByteView, uint32_t,
                                                                     const AdoptedScanout&,
                                                                     Dcn32CursorRegisters&) noexcept;
    friend class Dcn32Cursor;
    friend bool read_dchub_refclk_khz(const hardware::RegisterIo&, const Dcn32CursorRegisters&,
                                      uint32_t, uint32_t&) noexcept;
    uint32_t dcn_ = 0;
    uint32_t mpc_ = 0;
    uint32_t aperture_bytes_ = 0;
    uint32_t pipe_ = 0;
    uint32_t opp_ = 0;
    uint32_t otg_ = 0;
};

hardware::DiscoveryStatus validate_dcn32_cursor_discovery(ByteView binary, uint32_t register_aperture_bytes,
                                                          const AdoptedScanout& scanout,
                                                          Dcn32CursorRegisters& output) noexcept;

// bios_parser2 get_firmware_info_v3_4: dccg32_get_dccg_ref_freq takes the
// crystal frequency, which is dce_info (format 4, any content revision)
// dce_refclk_10khz * 10. False for an absent, truncated or other-format table
// or a zero frequency (Linux would carry the zero; this fails closed).
bool read_dce_refclk_khz(ByteView rom, uint32_t& khz) noexcept;

// hubbub2_get_dchub_ref_freq: DCHUBBUB_GLOBAL_TIMER_CNTL REFDIV 2 halves the
// DCCG reference. Linux asserts (and continues) when the global timer is off or
// the result leaves 40..60 MHz; this refuses those states instead. Read only.
bool read_dchub_refclk_khz(const hardware::RegisterIo& io, const Dcn32CursorRegisters& registers,
                           uint32_t dccg_ref_khz, uint32_t& khz) noexcept;

// ---------------------------------------------------------------------------
// Position arithmetic (pure).
// ---------------------------------------------------------------------------
// The stream the cursor is drawn on: the adopted unscaled, unrotated,
// unmirrored plane at (0, 0) whose viewport equals the OTG active area
// (decode_preos_scanout guarantees all of this), and the clocks of
// dc_cursor_mi_param.
struct CursorStream final {
    uint32_t width;
    uint32_t height;
    uint32_t pixel_clock_khz;
    uint32_t refclk_khz;
};

struct CursorRequest final {
    bool enable;
    // Top-left of the cursor image in stream pixels; may be negative or offscreen.
    int32_t x;
    int32_t y;
};

// The register values one hubp2/dpp1 position update writes.
struct CursorPlacement final {
    bool enable;
    uint32_t x;
    uint32_t y;
    uint32_t hot_x;
    uint32_t hot_y;
    uint32_t dst_x_offset;
};

// amdgpu_dm_plane_get_cursor_position (x or y at or beyond minus the maximum
// size disables; a negative coordinate becomes position 0 with hotspot
// min(-v, max - 1)), dcn10_set_cursor_position for rotation 0 without scaling,
// mirroring or translate_by_source, then the hubp2_cursor_set_position and
// dpp1_set_cursor_position visibility tests and CURSOR_DST_X_OFFSET
// (non-negative source offset * refclk / pixel clock, h_scale_ratio 1). Field
// widths are applied as REG_SET masks them. False only for invalid stream
// clocks or sizes.
bool place_cursor(const CursorRequest& request, const CursorStream& stream, uint32_t width,
                  uint32_t height, CursorPlacement& output) noexcept;

// hubp2_get_lines_per_chunk for the 32 bpp and mono modes (field value).
uint32_t cursor_lines_per_chunk(uint32_t width, CursorMode mode) noexcept;
// hubp1_get_cursor_pitch: 64/128/256 pixels -> 0/1/2. Linux logs and falls back
// to 64 for anything else; this returns false instead.
bool cursor_pitch_field(uint32_t pitch_pixels, uint32_t& field) noexcept;
// hubp32_cursor_set_attributes: USE_MALL_FOR_CURSOR when the 64-pixel-rounded
// width x height x bytes-per-pixel exceeds 16 KiB.
bool cursor_uses_mall(uint32_t width, uint32_t height, CursorMode mode) noexcept;

// dcn10_get_vupdate_offset_from_vsync and dcn10_calc_vupdate_position for a
// progressive, borderless timing with the VSTARTUP_START the VBIOS programmed
// (OTG_VSTARTUP_PARAM; Linux programs the same register from its DML value).
struct VupdateWindow final {
    uint32_t start_line;
    uint32_t end_line;
};
bool cursor_vupdate_window(const Timing& timing, uint32_t vstartup_start, VupdateWindow& output) noexcept;

// delay_cursor_until_vupdate: microseconds to stall before taking the cursor
// lock at vertical position vpos (0 when no stall is needed).
uint32_t cursor_lock_delay_microseconds(const Timing& timing, const VupdateWindow& window,
                                        uint32_t vpos) noexcept;

// ---------------------------------------------------------------------------
// Register protocol.
// ---------------------------------------------------------------------------
enum class CursorStatus {
    ready,
    invalid_argument,
    uninitialized,
    // A register access failed: hardware cursor state is uncertain. Every later
    // call refuses until initialize runs again.
    mmio_failure,
    failed
};

struct CursorAttributes final {
    uint64_t mc_address;
    CursorMode mode;
};

// The values last written, for diagnostics and tests.
struct CursorReport final {
    CursorStatus status;
    bool attributes_programmed;
    bool enabled;
    uint64_t mc_address;
    CursorMode mode;
    CursorPlacement placement;
    uint32_t lock_delays;
    uint32_t reprogrammed_attributes;
};

// Device-owned, trivially destructible. Serialized by the native owner at
// PASSIVE_LEVEL (lock stalls use RegisterIo::delay_microseconds). io's context
// outlives the object. Every programming sequence holds the MPC cursor lock of
// the OPP (CUR_VUPDATE_LOCK_SET) so the double-buffered cursor registers take
// effect together at the next VUPDATE, as program_cursor_attributes/position do.
class Dcn32Cursor final {
public:
    Dcn32Cursor() noexcept = default;

    // stream.width/height must equal the adopted viewport; both clocks nonzero.
    // No register is written.
    CursorStatus initialize(const Dcn32CursorRegisters& registers, const hardware::RegisterIo& io,
                            const Timing& timing, const CursorStream& stream) noexcept;

    // program_cursor_attributes: lock, hubp32_cursor_set_attributes (64 x 64,
    // pitch 64), dpp3_set_cursor_attributes (no degamma ROM; mono colors
    // 0/0xffffff), dcn10_set_cursor_sdr_white_level (scale 0x3c00, bias 0),
    // unlock. mc_address must be a nonzero, aligned 48-bit MC address.
    CursorStatus set_attributes(const CursorAttributes& attributes) noexcept;

    // program_cursor_position with place_cursor: lock, hubp2 (reprogramming the
    // attributes when CURSOR_SURFACE_ADDRESS reads 0 while enabling), dpp1,
    // unlock. Before any set_attributes this writes nothing (the hubp2 guard).
    CursorStatus set_position(const CursorRequest& request) noexcept;

    // set_position with a disabled, zeroed request (amdgpu_dm_update_cursor
    // passes a zeroed dc_cursor_position to turn the cursor off).
    CursorStatus hide() noexcept;

    // Forget programmed state without touching registers (after a power loss
    // the hardware registers no longer hold it).
    void forget() noexcept;

    CursorReport report() const noexcept { return report_; }

private:
    bool read(uint32_t segment_base, uint32_t reg, uint32_t& value) noexcept;
    bool write(uint32_t segment_base, uint32_t reg, uint32_t value) noexcept;
    bool update(uint32_t segment_base, uint32_t reg, uint32_t mask, uint32_t value) noexcept;
    bool lock(bool locked) noexcept;
    bool write_hubp_attributes() noexcept;
    bool write_dpp_attributes() noexcept;
    bool write_position(const CursorPlacement& placement) noexcept;
    CursorStatus fail() noexcept;

    Dcn32CursorRegisters registers_{};
    hardware::RegisterIo io_{};
    Timing timing_{};
    CursorStream stream_{};
    CursorRequest request_{};
    CursorReport report_{CursorStatus::uninitialized, false, false, 0, CursorMode::premultiplied_alpha, {}, 0, 0};
};

} // namespace rx7900::display

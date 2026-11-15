/*
Module: rx7900.display.dcn32_clocks
Purpose: Own Navi31 display clocks: SMU13 DALSMC hard minimums, DCCG DTOs and dividers.
Created: 2026-10-24
Architecture: Uses the shared hardware::RegisterIo contract and the discovery-
validated DMU segment1 (DCCG) base. Follows Linux v6.12 clk_mgr/dcn32
(dcn32_clk_mgr.c, dcn32_clk_mgr_smu_msg.c, dalsmc.h), clk_mgr_internal.h,
dccg/dcn20 dccg2_update_dpp_dto, dccg/dcn32 dccg32_set_pixel_rate_div and
dce/dce_clock_source.c dcn31_program_pix_clk / dce112_program_pixel_clk_resync.
The ATOM SetPixelClock half of a TMDS pixel clock lives in dcn32_link_encoder.
Linux AMD register provenance and notice: licenses/AMDGPU-DISPLAY.txt.
*/
#pragma once

#include "dcn32_otg.hpp"
#include "register_io.hpp"

namespace rx7900::display {

// dcn32_smu13_driver_if.h PPCLK_e, the SMU13.0.0 clock indices DALSMC accepts.
enum class SmuClock : uint8_t {
    gfxclk = 0, socclk = 1, uclk = 2, fclk = 3, dclk0 = 4, vclk0 = 5, dclk1 = 6,
    vclk1 = 7, dispclk = 8, dppclk = 9, dprefclk = 10, dcfclk = 11, dtbclk = 12
};

// dalsmc.h message identifiers and response codes.
namespace dalsmc {
constexpr uint32_t version = 0x1;
constexpr uint32_t result_ok = 0x1;
constexpr uint32_t result_failed = 0xff;
constexpr uint32_t result_unknown_command = 0xfe;
constexpr uint32_t result_rejected_prerequisite = 0xfd;
constexpr uint32_t result_rejected_busy = 0xfc;
constexpr uint32_t get_smu_version = 0x2;
constexpr uint32_t get_driver_if_version = 0x3;
constexpr uint32_t get_message_header_version = 0x4;
constexpr uint32_t set_hard_min_by_frequency = 0x9;
constexpr uint32_t get_dpm_frequency_by_index = 0xb;
constexpr uint32_t get_dc_mode_max_dpm_frequency = 0xc;
constexpr uint32_t set_min_deep_sleep_dcfclk = 0xd;
constexpr uint32_t number_of_displays = 0xe;
constexpr uint32_t set_fclk_switch_allow = 0x11;
constexpr uint32_t set_always_wait_dmcub_response = 0x14;
} // namespace dalsmc

enum class ClockResult : uint8_t {
    success,
    invalid_argument,
    // The SMU answered with a non-OK code before anything changed.
    rejected,
    // MP1 discovery base differs from the fixed DAL mailbox indices.
    discovery_mismatch,
    not_initialized,
    // DISPCLK divider 127 transitions need OTG pixel add/drop (dcn-stream).
    unsupported_transition,
    // A message or register write may have taken effect without proof.
    quarantined
};

// DAL-to-PMFW mailbox of dcn32_clk_mgr_smu_msg.c: MSG 0x1628A (MP1 C2PMSG_74),
// ARG 0x16273 (C2PMSG_51), RESP 0x16274 (C2PMSG_52), absolute dword indices that
// are only valid when discovery reports MP1 instance 0 segment 0 at 0x16000.
// Linux polls 10 us x 200000 (2 s). Unlike Linux a busy mailbox before the
// message is a refusal, not an overwrite; a missing response after the message
// write quarantines the mailbox until reset.
class DalSmcMailbox final {
public:
    DalSmcMailbox() noexcept = default;
    DalSmcMailbox(const DalSmcMailbox&) = delete;
    DalSmcMailbox& operator=(const DalSmcMailbox&) = delete;

    ClockResult bind(const hardware::RegisterIo& io, uint32_t mp1_segment0) noexcept;
    // PASSIVE_LEVEL, serialized with every other DAL mailbox user. output is
    // written only on result_ok.
    ClockResult send(uint32_t message, uint32_t argument, uint32_t* output) noexcept;
    bool bound() const noexcept { return bound_; }
    bool quarantined() const noexcept { return quarantined_; }
    uint32_t last_response() const noexcept { return last_response_; }
    // Only after the device owner proved an ASIC reset.
    void reset_proven() noexcept {
        io_ = {};
        last_response_ = 0;
        bound_ = false;
        quarantined_ = false;
    }

private:
    bool wait_response(uint32_t& response) noexcept;

    hardware::RegisterIo io_{};
    uint32_t last_response_ = 0;
    bool bound_ = false;
    bool quarantined_ = false;
};

constexpr uint32_t clock_dpm_levels = 16;
constexpr uint32_t display_pipe_count = 4;

struct ClockLevels final {
    uint32_t mhz[clock_dpm_levels]{};
    uint32_t count = 0;
    // GetDpmFreqByIndex(0xff) bit 31: fine-grained; levels are min and max.
    bool fine_grained = false;
    uint32_t dc_mode_max_mhz = 0;
};

// Linux clk_bw_params subset dcn-stream's bandwidth model reads. valid only
// after init; memory fields come from the caller's ATOM vram_info decode.
struct DisplayClockTable final {
    ClockLevels dcfclk{};
    ClockLevels socclk{};
    ClockLevels dtbclk{};
    ClockLevels dispclk{};
    ClockLevels dppclk{};
    ClockLevels uclk{};
    ClockLevels fclk{};
    uint32_t dentist_vco_khz = 0;
    uint32_t dprefclk_khz = 0;
    uint32_t dccg_ref_khz = 0;
    uint32_t dchub_ref_khz = 0;
    uint32_t smu_version = 0;
    uint32_t driver_if_version = 0;
    uint32_t num_channels = 0;
    uint32_t dram_channel_width_bytes = 0;
    uint32_t vram_type = 0;
    bool memory_info_valid = false;
    bool valid = false;
};

// Linux struct dc_clocks subset, all kHz. dpp_pipe_khz[i] is the DPP DTO
// target of pipe i (0 = pipe unused). display_count feeds NumOfDisplays.
struct DisplayClocks final {
    uint32_t dispclk_khz = 0;
    uint32_t dppclk_khz = 0;
    uint32_t dpp_pipe_khz[display_pipe_count]{};
    uint32_t dcfclk_khz = 0;
    uint32_t dcfclk_deep_sleep_khz = 0;
    uint32_t socclk_khz = 0;
    uint32_t fclk_khz = 0;
    uint32_t dramclk_khz = 0;
    uint32_t ref_dtbclk_khz = 0;
    uint32_t display_count = 0;
    bool p_state_change_support = true;
    bool fclk_p_state_change_support = true;
    bool dtbclk_en = false;
};

struct ClockManagerConfig final {
    hardware::RegisterIo io{};
    // Discovery-validated segment bases: DMU segment1 (DCCG) and MP1 segment0.
    Dcn32RegisterSpace dcn{};
    uint32_t dccg_segment = 0;
    uint32_t mp1_segment0 = 0;
    // Firmware info crystal (dccg/dchub reference) and a VBIOS DP DTO modulo
    // (Hz / 1000) captured from an adopted pre-OS scanout, 0 when unknown.
    uint32_t xtalin_khz = 0;
    uint32_t boot_dprefclk_khz = 0;
    // Value of CLK1_CLK_PLL_REQ read by the owner, 0 when unavailable.
    uint32_t pll_request = 0;
};

// Device-owned, PASSIVE_LEVEL, exclusive display-clock owner. Construct once,
// init once per ASIC epoch. Every operation is bounded; uncertain writes
// quarantine the manager until an owner-proven reset.
class Dcn32ClockManager final {
public:
    Dcn32ClockManager() noexcept = default;
    Dcn32ClockManager(const Dcn32ClockManager&) = delete;
    Dcn32ClockManager& operator=(const Dcn32ClockManager&) = delete;

    // dcn32_clk_mgr_construct + dcn32_init_clocks: SMU version/header checks,
    // DCFCLK, SOCCLK, DTBCLK, DISPCLK, DPPCLK then UCLK/FCLK levels with dc-mode
    // limits, 1950 MHz DISPCLK/DPPCLK caps, VCO and DPREFCLK. No clock changes.
    ClockResult init(const ClockManagerConfig& config) noexcept;
    const DisplayClockTable& table() const noexcept { return table_; }
    void set_memory_info(uint32_t channels, uint32_t width_bytes, uint32_t vram_type) noexcept;

    // dcn32_update_clocks subset (raise first; lowering only with safe_to_lower).
    // applied returns the clocks now in force (dispclk/dppclk after dentist
    // rounding). No watermark table, CAB or dentist WDIVIDER write is issued.
    ClockResult update(const DisplayClocks& requested, bool safe_to_lower,
                       DisplayClocks& applied) noexcept;
    // Reverses update: hard minimums back to the lowest levels, every DPP DTO
    // off, NumOfDisplays 0.
    ClockResult release() noexcept;
    const DisplayClocks& current() const noexcept { return current_; }

    // dcn31_program_pix_clk DP branch for OTG 0..3 (8b/10b only): optimized-rate
    // table first, else PHASE = 100 Hz * 100 and MODULO = DPREFCLK Hz, then
    // DP_DTO0_ENABLE=1, PIPE0_DTO_SRC_SEL=1.
    ClockResult program_dp_dto(uint32_t otg, uint32_t pixel_100hz) noexcept;
    // dcn31_program_pix_clk TMDS branch, before ATOM SetPixelClock: PIPE_DTO_SRC_SEL=0.
    ClockResult select_tmds_source(uint32_t otg) noexcept;
    // dce112_program_pixel_clk_resync for PHY PLL 0..4 (A..E): deep color 0..3.
    ClockResult program_pixel_resync(uint32_t pll, uint32_t deep_color) noexcept;
    // dccg32_set_pixel_rate_div: k1 in {0 (1), 1 (2)}, k2 in {0 (1), 1 (2), 3 (4)}.
    ClockResult set_pixel_rate_div(uint32_t otg, uint32_t k1, uint32_t k2) noexcept;
    // Turns the OTG DP DTO off and the DTO source back to the PHY (0).
    ClockResult clear_pixel_clock(uint32_t otg) noexcept;

    uint32_t dprefclk_khz() const noexcept { return table_.dprefclk_khz; }
    bool quarantined() const noexcept { return quarantined_ || mailbox_.quarantined(); }
    bool initialized() const noexcept { return initialized_; }
    // Only after the device owner proved an ASIC reset: forget every clock
    // state; init must run again for the new epoch.
    void reset_proven() noexcept {
        mailbox_.reset_proven();
        config_ = {};
        table_ = {};
        current_ = {};
        initialized_ = false;
        quarantined_ = false;
    }

private:
    ClockResult init_levels(SmuClock clock, ClockLevels& levels) noexcept;
    ClockResult hard_min(SmuClock clock, uint32_t mhz, uint32_t* granted) noexcept;
    ClockResult send(uint32_t message, uint32_t argument, uint32_t* output) noexcept;
    ClockResult update_dpp_dto(const DisplayClocks& requested, bool safe_to_lower) noexcept;
    ClockResult dccg_update(uint32_t reg, uint32_t field, uint32_t value) noexcept;
    ClockResult dccg_write(uint32_t reg, uint32_t value) noexcept;
    bool ready() const noexcept;

    ClockManagerConfig config_{};
    DalSmcMailbox mailbox_{};
    DisplayClockTable table_{};
    DisplayClocks current_{};
    bool initialized_ = false;
    bool quarantined_ = false;
};

// Linux clk_mgr_internal.h helpers, exposed for the tests and dcn-stream.
uint32_t khz_to_mhz_ceil(uint32_t khz) noexcept;
uint32_t dentist_did_from_divider(uint32_t divider) noexcept;
// DENTIST_DIVIDER_RANGE_SCALE_FACTOR * vco / clock, then clock = 4 * vco / divider.
uint32_t dentist_round_clock(uint32_t vco_khz, uint32_t clock_khz, uint32_t& divider) noexcept;

} // namespace rx7900::display

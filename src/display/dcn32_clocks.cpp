/*
Module: rx7900.display.dcn32_clocks
Purpose: Bounded DALSMC mailbox, DCN3.2 clock table and DCCG DTO programming.
Created: 2026-10-24
Architecture: Linux v6.12 dcn32_clk_mgr*.c order and arithmetic; exact
dcn_3_2_0_offset.h/_sh_mask.h DCCG values (segment BASE_IDX 1).
Copyright 2012-2023 Advanced Micro Devices, Inc.
Linux notice and provenance: licenses/AMDGPU-DISPLAY.txt.
*/
#include "dcn32_clocks.hpp"

namespace rx7900::display {
namespace {

// dcn32_clk_mgr_smu_msg.c mmDAL_* minus the MP1 segment0 base 0x16000.
constexpr uint32_t mp1_segment0_base = 0x16000u;
constexpr uint32_t dal_message = 0x28au;
constexpr uint32_t dal_argument = 0x273u;
constexpr uint32_t dal_response = 0x274u;
constexpr uint32_t dal_poll_us = 10u;
constexpr uint32_t dal_polls = 200000u;

// clk_mgr_internal.h dentist constants.
constexpr uint32_t dentist_scale = 4u;
// dcn32_init_clocks caps DISPCLK/DPPCLK table entries and dc-mode limits.
constexpr uint32_t dispclk_cap_mhz = 1950u;
// dcn32_clk_mgr_construct defaults.
constexpr uint32_t default_dprefclk_khz = 716666u;
constexpr uint32_t default_vco_khz = 4300000u;
constexpr uint32_t dfs_reference_khz = 100000u;
// CLK1_CLK_PLL_REQ FbMult_int (8:0) and FbMult_frac (31:16).
constexpr uint32_t fbmult_int_mask = 0x000001ffu;
constexpr uint32_t fbmult_frac_shift = 16u;
// dcn30_smu_get_dpm_freq_by_index level query index and replies.
constexpr uint32_t level_count_index = 0xffu;
constexpr uint32_t fine_grained_bit = 0x80000000u;
// FclkSwitchAllow_e.
constexpr uint32_t fclk_switch_allow = 1u;
constexpr uint32_t fclk_switch_disallow = 0u;

// dcn_3_2_0_offset.h DCCG (BASE_IDX 1).
constexpr uint32_t otg_pixel_rate_div = 0x006fu;
constexpr uint32_t pixel_rate_cntl[display_pipe_count]{0x0080u, 0x0084u, 0x0088u, 0x008cu};
constexpr uint32_t dp_dto_phase[display_pipe_count]{0x0081u, 0x0085u, 0x0089u, 0x008du};
constexpr uint32_t dp_dto_modulo[display_pipe_count]{0x0082u, 0x0086u, 0x008au, 0x008eu};
constexpr uint32_t dppclk_dto_ctrl = 0x00b6u;
constexpr uint32_t dppclk_dto_param[display_pipe_count]{0x0099u, 0x009au, 0x009bu, 0x009cu};
constexpr uint32_t pixclk_resync[5]{0x0040u, 0x0041u, 0x0042u, 0x0043u, 0x004cu};
// dcn_3_2_0_sh_mask.h DCCG fields.
constexpr uint32_t dp_dto_enable = 0x00000010u;
constexpr uint32_t pipe_dto_source = 0x00003000u;
constexpr uint32_t pipe_dto_source_shift = 12u;
constexpr uint32_t resync_deep_color = 0x00000030u;
constexpr uint32_t resync_deep_color_shift = 4u;
constexpr uint32_t dppclk_dto_modulo_shift = 16u;
constexpr uint32_t dppclk_dto_field = 0xffu;

struct OptimizedRate final {
    uint32_t minimum_khz;
    uint32_t maximum_khz;
    uint32_t target_khz;
    uint32_t multiply;
    uint32_t divide;
};

// dce_clock_source.c video_optimized_pixel_rates (inclusive ranges).
constexpr OptimizedRate optimized_rates[]{
    {25170, 25180, 25200, 1000, 1001},     {59340, 59350, 59400, 1000, 1001},
    {74170, 74180, 74250, 1000, 1001},     {89910, 90000, 90000, 1000, 1001},
    {125870, 125880, 126000, 1000, 1001},  {148350, 148360, 148500, 1000, 1001},
    {167830, 167840, 168000, 1000, 1001},  {222520, 222530, 222750, 1000, 1001},
    {257140, 257150, 257400, 1000, 1001},  {296700, 296710, 297000, 1000, 1001},
    {342850, 342860, 343200, 1000, 1001},  {395600, 395610, 396000, 1000, 1001},
    {409090, 409100, 409500, 1000, 1001},  {445050, 445060, 445500, 1000, 1001},
    {467530, 467540, 468000, 1000, 1001},  {519230, 519240, 519750, 1000, 1001},
    {525970, 525980, 526500, 1000, 1001},  {545450, 545460, 546000, 1000, 1001},
    {593400, 593410, 594000, 1000, 1001},  {623370, 623380, 624000, 1000, 1001},
    {692300, 692310, 693000, 1000, 1001},  {701290, 701300, 702000, 1000, 1001},
    {791200, 791210, 792000, 1000, 1001},  {890100, 890110, 891000, 1000, 1001},
    {1186810, 1186820, 1188000, 1000, 1001}, {27020, 27030, 27000, 1001, 1000},
    {54050, 54060, 54000, 1001, 1000},     {108100, 108110, 108000, 1001, 1000}};

const OptimizedRate* optimized_rate(uint32_t khz) noexcept {
    for (const OptimizedRate& entry : optimized_rates) {
        if (entry.minimum_khz <= khz && khz <= entry.maximum_khz) {
            return &entry;
        }
    }
    return nullptr;
}

uint32_t argument(SmuClock clock, uint32_t value) noexcept {
    return (static_cast<uint32_t>(clock) << 16) | (value & 0xffffu);
}

// clk_mgr_internal.h should_set_clock.
bool should_set(bool safe_to_lower, uint32_t requested, uint32_t current) noexcept {
    return (safe_to_lower && requested < current) || requested > current;
}

// clk_mgr_internal.h should_update_pstate_support.
bool should_update_pstate(bool safe_to_lower, bool requested, bool current) noexcept {
    if (requested == current) {
        return false;
    }
    return requested ? safe_to_lower : !safe_to_lower;
}

uint32_t lowest(const ClockLevels& levels) noexcept {
    return levels.count == 0 ? 0 : levels.mhz[0];
}

uint32_t highest(const ClockLevels& levels) noexcept {
    return levels.count == 0 ? 0 : levels.mhz[levels.count - 1u];
}

} // namespace

uint32_t khz_to_mhz_ceil(uint32_t khz) noexcept {
    return khz / 1000u + (khz % 1000u != 0 ? 1u : 0u);
}

uint32_t dentist_did_from_divider(uint32_t divider) noexcept {
    // clk_mgr dentist_get_did_from_divider ranges (in quarter steps).
    if (divider < 8u) {
        return 0x08u;
    }
    if (divider < 64u) {
        return 0x08u + (divider - 8u);
    }
    if (divider < 128u) {
        return 0x40u + (divider - 64u) / 2u;
    }
    if (divider < 248u) {
        return 0x60u + (divider - 128u) / 4u;
    }
    const uint32_t did = 0x7eu + (divider - 248u) / 264u;
    return did > 0x7fu ? 0x7fu : did;
}

uint32_t dentist_round_clock(uint32_t vco_khz, uint32_t clock_khz, uint32_t& divider) noexcept {
    divider = 0;
    if (vco_khz == 0 || clock_khz == 0) {
        return 0;
    }
    const uint64_t scaled = static_cast<uint64_t>(dentist_scale) * vco_khz;
    const uint64_t value = scaled / clock_khz;
    if (value == 0 || value > UINT32_MAX) {
        return 0;
    }
    divider = static_cast<uint32_t>(value);
    return static_cast<uint32_t>(scaled / divider);
}

ClockResult DalSmcMailbox::bind(const hardware::RegisterIo& io, uint32_t mp1_segment0) noexcept {
    if (bound_ || !hardware::register_io_valid(io)) {
        return ClockResult::invalid_argument;
    }
    if (mp1_segment0 != mp1_segment0_base) {
        return ClockResult::discovery_mismatch;
    }
    uint32_t offset = 0;
    if (!hardware::discovery_register_offset(mp1_segment0, dal_message, io.aperture_bytes, offset)) {
        return ClockResult::invalid_argument;
    }
    io_ = io;
    bound_ = true;
    return ClockResult::success;
}

bool DalSmcMailbox::wait_response(uint32_t& response) noexcept {
    for (uint32_t poll = 0; poll < dal_polls; ++poll) {
        if (!hardware::read_ip_register(io_, mp1_segment0_base, dal_response, response) ||
            response == UINT32_MAX) {
            return false;
        }
        if (response != 0) {
            return true;
        }
        io_.delay_microseconds(io_.context, dal_poll_us);
    }
    return false;
}

ClockResult DalSmcMailbox::send(uint32_t message, uint32_t value, uint32_t* output) noexcept {
    if (!bound_) {
        return ClockResult::not_initialized;
    }
    if (quarantined_) {
        return ClockResult::quarantined;
    }
    uint32_t response = 0;
    // dcn32_smu_send_msg_with_param: wait, clear RESP, ARG, MSG, wait.
    if (!wait_response(response)) {
        return ClockResult::rejected;
    }
    if (!hardware::write_ip_register(io_, mp1_segment0_base, dal_response, 0) ||
        !hardware::write_ip_register(io_, mp1_segment0_base, dal_argument, value) ||
        !hardware::write_ip_register(io_, mp1_segment0_base, dal_message, message)) {
        quarantined_ = true;
        return ClockResult::quarantined;
    }
    if (!wait_response(response)) {
        quarantined_ = true;
        return ClockResult::quarantined;
    }
    last_response_ = response;
    if (response != dalsmc::result_ok) {
        return ClockResult::rejected;
    }
    if (output != nullptr) {
        uint32_t reply = 0;
        if (!hardware::read_ip_register(io_, mp1_segment0_base, dal_argument, reply)) {
            return ClockResult::rejected;
        }
        *output = reply;
    }
    return ClockResult::success;
}

bool Dcn32ClockManager::ready() const noexcept {
    return initialized_ && !quarantined();
}

ClockResult Dcn32ClockManager::send(uint32_t message, uint32_t value, uint32_t* output) noexcept {
    const ClockResult result = mailbox_.send(message, value, output);
    if (result == ClockResult::quarantined) {
        quarantined_ = true;
    }
    return result;
}

ClockResult Dcn32ClockManager::init_levels(SmuClock clock, ClockLevels& levels) noexcept {
    levels = {};
    uint32_t reply = 0;
    ClockResult result = send(dalsmc::get_dpm_frequency_by_index,
                              argument(clock, level_count_index), &reply);
    if (result != ClockResult::success) {
        return result;
    }
    levels.fine_grained = (reply & fine_grained_bit) != 0;
    const uint32_t count = levels.fine_grained ? 2u : (reply & 0xffu);
    if (count > clock_dpm_levels) {
        return ClockResult::rejected;
    }
    for (uint32_t index = 0; index < count; ++index) {
        // dcn32_init_single_clock queries index i for i < num_levels; a
        // fine-grained clock answers index 0 with its minimum, 1 with its maximum.
        uint32_t frequency = 0;
        result = send(dalsmc::get_dpm_frequency_by_index, argument(clock, index), &frequency);
        if (result != ClockResult::success) {
            return result;
        }
        levels.mhz[index] = frequency & 0xffffu;
    }
    levels.count = count;
    uint32_t limit = 0;
    if (send(dalsmc::get_dc_mode_max_dpm_frequency, argument(clock, 0), &limit) ==
        ClockResult::success) {
        levels.dc_mode_max_mhz = limit & 0xffffu;
    }
    return quarantined() ? ClockResult::quarantined : ClockResult::success;
}

ClockResult Dcn32ClockManager::init(const ClockManagerConfig& config) noexcept {
    if (initialized_ || !hardware::register_io_valid(config.io) || !config.dcn.discovery_verified ||
        config.dcn.dmu_major != 3 || config.dcn.dmu_minor != 2 || config.dcn.dmu_revision != 0 ||
        config.dccg_segment == 0 || config.xtalin_khz == 0) {
        return ClockResult::invalid_argument;
    }
    ClockResult result = mailbox_.bind(config.io, config.mp1_segment0);
    if (result != ClockResult::success) {
        return result;
    }
    config_ = config;

    // Filled in place (a local table exceeds the kernel frame budget) and
    // published only through valid/initialized_ on success.
    DisplayClockTable& table = table_;
    table = {};
    uint32_t header = 0;
    result = send(dalsmc::get_smu_version, 0, &table.smu_version);
    if (result == ClockResult::success) {
        result = send(dalsmc::get_driver_if_version, 0, &table.driver_if_version);
    }
    if (result == ClockResult::success) {
        result = send(dalsmc::get_message_header_version, 0, &header);
    }
    if (result != ClockResult::success) {
        return result;
    }
    if (header != dalsmc::version) {
        return ClockResult::rejected;
    }

    struct Entry final {
        SmuClock clock;
        ClockLevels* levels;
    };
    const Entry entries[]{{SmuClock::dcfclk, &table.dcfclk}, {SmuClock::socclk, &table.socclk},
                          {SmuClock::dtbclk, &table.dtbclk}, {SmuClock::dispclk, &table.dispclk},
                          {SmuClock::dppclk, &table.dppclk}, {SmuClock::uclk, &table.uclk},
                          {SmuClock::fclk, &table.fclk}};
    for (const Entry& entry : entries) {
        result = init_levels(entry.clock, *entry.levels);
        if (result != ClockResult::success) {
            return result;
        }
    }
    // dpm_present: DCFCLK, DTBCLK and DISPCLK all report levels.
    if (table.dcfclk.count == 0 || table.dtbclk.count == 0 || table.dispclk.count == 0 ||
        table.dppclk.count == 0 || table.uclk.count == 0) {
        return ClockResult::rejected;
    }
    ClockLevels* const capped[]{&table.dispclk, &table.dppclk};
    for (ClockLevels* levels : capped) {
        for (uint32_t index = 0; index < levels->count; ++index) {
            if (levels->mhz[index] > dispclk_cap_mhz) {
                levels->mhz[index] = dispclk_cap_mhz;
            }
        }
        if (levels->dc_mode_max_mhz > dispclk_cap_mhz) {
            levels->dc_mode_max_mhz = dispclk_cap_mhz;
        }
    }

    // dcn32_get_vco_frequency_from_reg: (FbMult_int + FbMult_frac / 65536) * 100 MHz.
    uint64_t vco = 0;
    if (config.pll_request != 0) {
        const uint64_t fixed = (static_cast<uint64_t>(config.pll_request & fbmult_int_mask) << 16) |
                               (config.pll_request >> fbmult_frac_shift);
        vco = (fixed * dfs_reference_khz) >> 16;
    }
    table.dentist_vco_khz = vco == 0 || vco > UINT32_MAX ? default_vco_khz : static_cast<uint32_t>(vco);
    table.dprefclk_khz = config.boot_dprefclk_khz != 0 ? config.boot_dprefclk_khz : default_dprefclk_khz;
    table.dccg_ref_khz = config.xtalin_khz;
    table.dchub_ref_khz = config.xtalin_khz;
    table.valid = true;
    // Linux starts with P-state support true and every clock unknown (zero).
    current_ = {};
    initialized_ = true;
    return ClockResult::success;
}

void Dcn32ClockManager::set_memory_info(uint32_t channels, uint32_t width_bytes,
                                        uint32_t vram_type) noexcept {
    table_.num_channels = channels;
    table_.dram_channel_width_bytes = width_bytes;
    table_.vram_type = vram_type;
    table_.memory_info_valid = channels != 0 && width_bytes != 0;
}

ClockResult Dcn32ClockManager::hard_min(SmuClock clock, uint32_t mhz, uint32_t* granted) noexcept {
    uint32_t reply = 0;
    const ClockResult result = send(dalsmc::set_hard_min_by_frequency, argument(clock, mhz), &reply);
    if (result == ClockResult::success && granted != nullptr) {
        *granted = reply;
    }
    return result;
}

ClockResult Dcn32ClockManager::dccg_write(uint32_t reg, uint32_t value) noexcept {
    if (!hardware::write_ip_register(config_.io, config_.dccg_segment, reg, value)) {
        quarantined_ = true;
        return ClockResult::quarantined;
    }
    return ClockResult::success;
}

ClockResult Dcn32ClockManager::dccg_update(uint32_t reg, uint32_t field, uint32_t value) noexcept {
    uint32_t current = 0;
    if (!hardware::read_ip_register(config_.io, config_.dccg_segment, reg, current) ||
        current == UINT32_MAX) {
        return ClockResult::rejected;
    }
    return dccg_write(reg, (current & ~field) | (value & field));
}

// dcn32_update_clocks_update_dpp_dto over dccg2_update_dpp_dto: ref_dppclk is
// the global DPPCLK; phase = ceil(255 * pipe / ref), modulo 0xff.
ClockResult Dcn32ClockManager::update_dpp_dto(const DisplayClocks& requested,
                                              bool safe_to_lower) noexcept {
    const uint32_t reference = current_.dppclk_khz;
    for (uint32_t pipe = 0; pipe < display_pipe_count; ++pipe) {
        const uint32_t target = requested.dpp_pipe_khz[pipe];
        const uint32_t previous = current_.dpp_pipe_khz[pipe];
        if (!safe_to_lower && target <= previous) {
            continue;
        }
        if (target == previous) {
            continue;
        }
        const uint32_t enable = 1u << (pipe * 4u);
        ClockResult result = ClockResult::success;
        if (reference != 0 && target != 0) {
            uint64_t phase = (static_cast<uint64_t>(dppclk_dto_field) * target + reference - 1u) /
                             reference;
            if (phase > dppclk_dto_field) {
                phase = dppclk_dto_field;
            }
            result = dccg_write(dppclk_dto_param[pipe],
                                static_cast<uint32_t>(phase) |
                                    (dppclk_dto_field << dppclk_dto_modulo_shift));
            if (result == ClockResult::success) {
                result = dccg_update(dppclk_dto_ctrl, enable, enable);
            }
        } else {
            result = dccg_update(dppclk_dto_ctrl, enable, 0);
        }
        if (result != ClockResult::success) {
            return result;
        }
        current_.dpp_pipe_khz[pipe] = target;
    }
    return ClockResult::success;
}

ClockResult Dcn32ClockManager::update(const DisplayClocks& requested, bool safe_to_lower,
                                      DisplayClocks& applied) noexcept {
    if (!ready()) {
        return quarantined() ? ClockResult::quarantined : ClockResult::not_initialized;
    }
    for (uint32_t pipe = 0; pipe < display_pipe_count; ++pipe) {
        if (requested.dpp_pipe_khz[pipe] > requested.dppclk_khz) {
            return ClockResult::invalid_argument;
        }
    }
    DisplayClocks wanted = requested;
    uint32_t dispclk_divider = 0;
    uint32_t dppclk_divider = 0;
    if (wanted.dispclk_khz != 0) {
        wanted.dispclk_khz = dentist_round_clock(table_.dentist_vco_khz, wanted.dispclk_khz,
                                                 dispclk_divider);
    }
    if (wanted.dppclk_khz != 0) {
        wanted.dppclk_khz = dentist_round_clock(table_.dentist_vco_khz, wanted.dppclk_khz,
                                                dppclk_divider);
    }
    if (wanted.dispclk_khz > dispclk_cap_mhz * 1000u || wanted.dppclk_khz > dispclk_cap_mhz * 1000u ||
        (requested.dispclk_khz != 0 && wanted.dispclk_khz == 0)) {
        return ClockResult::invalid_argument;
    }
    // The 127 divider needs FIFO pixel add/drop on OTG master pipes.
    const uint32_t old_did = current_.dispclk_khz == 0 ? 0 :
        dentist_did_from_divider(static_cast<uint32_t>(
            (static_cast<uint64_t>(dentist_scale) * table_.dentist_vco_khz) / current_.dispclk_khz));
    const uint32_t new_did = wanted.dispclk_khz == 0 ? 0 : dentist_did_from_divider(dispclk_divider);
    if ((old_did == 0x7fu) != (new_did == 0x7fu) && current_.dispclk_khz != 0 &&
        wanted.dispclk_khz != 0) {
        return ClockResult::unsupported_transition;
    }

    ClockResult result = ClockResult::success;
    const bool display_off = wanted.display_count == 0;
    if (display_off == safe_to_lower) {
        result = send(dalsmc::number_of_displays, wanted.display_count, nullptr);
    }
    if (result == ClockResult::success &&
        should_update_pstate(safe_to_lower, wanted.fclk_p_state_change_support,
                             current_.fclk_p_state_change_support)) {
        const bool previous = current_.fclk_p_state_change_support;
        current_.fclk_p_state_change_support = wanted.fclk_p_state_change_support;
        if (wanted.fclk_p_state_change_support) {
            result = send(dalsmc::set_fclk_switch_allow, fclk_switch_allow, nullptr);
        } else if (previous) {
            result = send(dalsmc::set_fclk_switch_allow, fclk_switch_disallow, nullptr);
        }
    }
    if (result == ClockResult::success && should_set(safe_to_lower, wanted.dcfclk_khz,
                                                     current_.dcfclk_khz)) {
        result = hard_min(SmuClock::dcfclk, khz_to_mhz_ceil(wanted.dcfclk_khz), nullptr);
        if (result == ClockResult::success) {
            current_.dcfclk_khz = wanted.dcfclk_khz;
        }
    }
    if (result == ClockResult::success &&
        should_set(safe_to_lower, wanted.dcfclk_deep_sleep_khz, current_.dcfclk_deep_sleep_khz)) {
        result = send(dalsmc::set_min_deep_sleep_dcfclk,
                      khz_to_mhz_ceil(wanted.dcfclk_deep_sleep_khz), nullptr);
        if (result == ClockResult::success) {
            current_.dcfclk_deep_sleep_khz = wanted.dcfclk_deep_sleep_khz;
        }
    }
    if (result == ClockResult::success && should_set(safe_to_lower, wanted.socclk_khz,
                                                     current_.socclk_khz)) {
        current_.socclk_khz = wanted.socclk_khz;
    }
    bool previous_pstate = current_.p_state_change_support;
    if (result == ClockResult::success &&
        should_update_pstate(safe_to_lower, wanted.p_state_change_support,
                             current_.p_state_change_support)) {
        current_.p_state_change_support = wanted.p_state_change_support;
        if (!wanted.p_state_change_support) {
            result = hard_min(SmuClock::uclk, highest(table_.uclk), nullptr);
        }
    }
    if (result == ClockResult::success) {
        // dcn32_smu_wait_for_dmub_ack_mclk(fw_based_mclk_switching = false).
        result = send(dalsmc::set_always_wait_dmcub_response, 0, nullptr);
    }
    bool update_uclk = false;
    if (result == ClockResult::success && should_set(safe_to_lower, wanted.dramclk_khz,
                                                     current_.dramclk_khz)) {
        current_.dramclk_khz = wanted.dramclk_khz;
        update_uclk = true;
    }
    if (result == ClockResult::success && current_.p_state_change_support &&
        (update_uclk || !previous_pstate)) {
        result = hard_min(SmuClock::uclk, khz_to_mhz_ceil(current_.dramclk_khz), nullptr);
    }
    current_.fclk_khz = wanted.fclk_khz;

    // DTBCLK: dtbclk_en off keeps the lowest DTBCLK level as reference.
    if (result == ClockResult::success) {
        const uint32_t dtb_target = wanted.dtbclk_en ? wanted.ref_dtbclk_khz :
                                    lowest(table_.dtbclk) * 1000u;
        if (should_set(safe_to_lower, dtb_target / 1000u, current_.ref_dtbclk_khz / 1000u)) {
            uint32_t granted = 0;
            result = hard_min(SmuClock::dtbclk, khz_to_mhz_ceil(dtb_target), &granted);
            if (result == ClockResult::success) {
                current_.ref_dtbclk_khz = granted * 1000u;
                current_.dtbclk_en = wanted.dtbclk_en;
            }
        }
    }

    bool dpp_lowered = false;
    bool update_dppclk = false;
    bool update_dispclk = false;
    if (result == ClockResult::success && should_set(safe_to_lower, wanted.dppclk_khz,
                                                     current_.dppclk_khz)) {
        dpp_lowered = wanted.dppclk_khz < current_.dppclk_khz;
        current_.dppclk_khz = wanted.dppclk_khz;
        if (!dpp_lowered) {
            result = hard_min(SmuClock::dppclk, current_.dppclk_khz / 1000u, nullptr);
        }
        update_dppclk = true;
    }
    if (result == ClockResult::success && should_set(safe_to_lower, wanted.dispclk_khz,
                                                     current_.dispclk_khz)) {
        current_.dispclk_khz = wanted.dispclk_khz;
        update_dispclk = true;
    }
    if (result == ClockResult::success) {
        if (dpp_lowered) {
            // Lowering: DPP DTO first, then the dentist (DISPCLK), then DPPCLK.
            result = update_dpp_dto(wanted, safe_to_lower);
            if (result == ClockResult::success && current_.dispclk_khz != 0) {
                result = hard_min(SmuClock::dispclk, current_.dispclk_khz / 1000u, nullptr);
            }
            if (result == ClockResult::success) {
                result = hard_min(SmuClock::dppclk, current_.dppclk_khz / 1000u, nullptr);
            }
        } else {
            if ((update_dppclk || update_dispclk) && current_.dispclk_khz != 0) {
                result = hard_min(SmuClock::dispclk, current_.dispclk_khz / 1000u, nullptr);
            }
            if (result == ClockResult::success) {
                result = update_dpp_dto(wanted, safe_to_lower);
            }
        }
    }
    current_.display_count = wanted.display_count;
    applied = current_;
    if (quarantined()) {
        return ClockResult::quarantined;
    }
    return result;
}

ClockResult Dcn32ClockManager::release() noexcept {
    if (!ready()) {
        return quarantined() ? ClockResult::quarantined : ClockResult::not_initialized;
    }
    DisplayClocks floor{};
    floor.dispclk_khz = lowest(table_.dispclk) * 1000u;
    floor.dppclk_khz = lowest(table_.dppclk) * 1000u;
    floor.dcfclk_khz = lowest(table_.dcfclk) * 1000u;
    floor.dramclk_khz = lowest(table_.uclk) * 1000u;
    floor.p_state_change_support = true;
    floor.fclk_p_state_change_support = true;
    DisplayClocks applied{};
    return update(floor, true, applied);
}

ClockResult Dcn32ClockManager::program_dp_dto(uint32_t otg, uint32_t pixel_100hz) noexcept {
    if (!ready()) {
        return quarantined() ? ClockResult::quarantined : ClockResult::not_initialized;
    }
    const uint32_t reference = table_.dprefclk_khz;
    if (otg >= display_pipe_count || pixel_100hz == 0 || reference == 0) {
        return ClockResult::invalid_argument;
    }
    const OptimizedRate* entry = optimized_rate(pixel_100hz / 10u);
    uint64_t phase = 0;
    uint64_t modulo = 0;
    if (entry != nullptr) {
        phase = static_cast<uint64_t>(entry->target_khz) * entry->multiply;
        modulo = static_cast<uint64_t>(reference) * entry->divide;
    } else {
        phase = static_cast<uint64_t>(pixel_100hz) * 100u;
        modulo = static_cast<uint64_t>(reference) * 1000u;
    }
    // The DTO divides the reference: PHASE beyond MODULO cannot be produced.
    if (phase == 0 || phase > modulo || modulo > UINT32_MAX) {
        return ClockResult::invalid_argument;
    }
    ClockResult result = dccg_write(dp_dto_phase[otg], static_cast<uint32_t>(phase));
    if (result == ClockResult::success) {
        result = dccg_write(dp_dto_modulo[otg], static_cast<uint32_t>(modulo));
    }
    if (result == ClockResult::success) {
        result = dccg_update(pixel_rate_cntl[otg], dp_dto_enable | pipe_dto_source,
                             dp_dto_enable | (1u << pipe_dto_source_shift));
    }
    return quarantined() ? ClockResult::quarantined : result;
}

ClockResult Dcn32ClockManager::select_tmds_source(uint32_t otg) noexcept {
    if (!ready()) {
        return quarantined() ? ClockResult::quarantined : ClockResult::not_initialized;
    }
    if (otg >= display_pipe_count) {
        return ClockResult::invalid_argument;
    }
    return dccg_update(pixel_rate_cntl[otg], pipe_dto_source, 0);
}

ClockResult Dcn32ClockManager::program_pixel_resync(uint32_t pll, uint32_t deep_color) noexcept {
    if (!ready()) {
        return quarantined() ? ClockResult::quarantined : ClockResult::not_initialized;
    }
    if (pll >= 5u || deep_color > 3u) {
        return ClockResult::invalid_argument;
    }
    // DCN 3.2 has no PHYPLLA_PIXCLK_DOUBLE_RATE_ENABLE field: deep color only.
    return dccg_update(pixclk_resync[pll], resync_deep_color, deep_color << resync_deep_color_shift);
}

ClockResult Dcn32ClockManager::set_pixel_rate_div(uint32_t otg, uint32_t k1, uint32_t k2) noexcept {
    if (!ready()) {
        return quarantined() ? ClockResult::quarantined : ClockResult::not_initialized;
    }
    if (otg >= display_pipe_count || k1 > 1u || (k2 != 0 && k2 != 1u && k2 != 3u)) {
        return ClockResult::invalid_argument;
    }
    // OTGn_PIXEL_RATE_DIVK1 at bit 3n, DIVK2 at bits 3n+2:3n+1.
    const uint32_t shift = otg * 3u;
    const uint32_t field = 7u << shift;
    const uint32_t value = (k1 | (k2 << 1)) << shift;
    uint32_t current = 0;
    if (!hardware::read_ip_register(config_.io, config_.dccg_segment, otg_pixel_rate_div, current) ||
        current == UINT32_MAX) {
        return ClockResult::rejected;
    }
    if ((current & field) == value) {
        return ClockResult::success;
    }
    return dccg_write(otg_pixel_rate_div, (current & ~field) | value);
}

ClockResult Dcn32ClockManager::clear_pixel_clock(uint32_t otg) noexcept {
    if (!ready()) {
        return quarantined() ? ClockResult::quarantined : ClockResult::not_initialized;
    }
    if (otg >= display_pipe_count) {
        return ClockResult::invalid_argument;
    }
    return dccg_update(pixel_rate_cntl[otg], dp_dto_enable | pipe_dto_source, 0);
}

} // namespace rx7900::display

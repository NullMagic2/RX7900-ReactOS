/*
Module: rx7900.tests.dcn32_clocks
Purpose: Check the DALSMC mailbox, DCN3.2 clock table, update order and DCCG DTOs.
Created: 2026-10-24
Architecture: Synthetic PMFW DAL mailbox and DCCG register model only; expected
values are the Linux v6.12 dcn32_clk_mgr / clk_mgr_internal / dccg2 / dccg32 /
dcn31_program_pix_clk sequences.
*/
#include "dcn32_clocks.hpp"
#include "dcn32_display_model.hpp"

namespace {
using namespace rx7900::display;
using rx7900_test::Checks;
namespace dcn = rx7900_test::dcn;

constexpr uint32_t dal_message = 0x28au;
constexpr uint32_t dal_argument = 0x273u;
constexpr uint32_t dal_response = 0x274u;

struct Smu final {
    // Per PPCLK index: levels in MHz and the fine-grained flag.
    std::vector<uint32_t> levels[13];
    bool fine[13]{};
    std::vector<std::pair<uint32_t, uint32_t>> messages;
    uint32_t header_version = 1;
    bool silent = false;
    uint32_t reject_message = 0;
};

void install(dcn::Model& model, Smu& smu) {
    smu.levels[11] = {300, 600, 1200};          // DCFCLK
    smu.levels[1] = {500, 1000};                // SOCCLK
    smu.levels[12] = {600, 757};                // DTBCLK
    smu.levels[8] = {800, 2000, 2400};          // DISPCLK (capped to 1950)
    smu.levels[9] = {800, 1600};                // DPPCLK
    smu.fine[9] = true;
    smu.levels[2] = {96, 456, 772, 1249};       // UCLK
    smu.levels[3] = {400, 1400};                // FCLK
    model.set(dcn::mp1_segment0, dal_response, 1);
    model.on_write = [&smu](dcn::Model& m, uint32_t offset, uint32_t value) {
        if (offset != dcn::byte_offset(dcn::mp1_segment0, dal_message)) {
            return;
        }
        const uint32_t argument = m.get(dcn::mp1_segment0, dal_argument);
        smu.messages.emplace_back(value, argument);
        if (smu.silent) {
            return;
        }
        if (value == smu.reject_message) {
            m.set(dcn::mp1_segment0, dal_response, dalsmc::result_rejected_prerequisite);
            return;
        }
        uint32_t reply = 0;
        const uint32_t clock = (argument >> 16) & 0xffu;
        const uint32_t index = argument & 0xffffu;
        switch (value) {
        case dalsmc::get_smu_version: reply = 0x004e4000u; break;
        case dalsmc::get_driver_if_version: reply = 0x37u; break;
        case dalsmc::get_message_header_version: reply = smu.header_version; break;
        case dalsmc::get_dpm_frequency_by_index:
            if (clock < 13) {
                const auto& levels = smu.levels[clock];
                if (index == 0xffu) {
                    reply = smu.fine[clock] ? 0x80000000u : static_cast<uint32_t>(levels.size());
                } else if (index < levels.size()) {
                    reply = levels[index];
                }
            }
            break;
        case dalsmc::get_dc_mode_max_dpm_frequency:
            reply = clock < 13 && !smu.levels[clock].empty() ? smu.levels[clock].back() : 0;
            break;
        case dalsmc::set_hard_min_by_frequency: reply = index; break;
        default: reply = 0; break;
        }
        m.set(dcn::mp1_segment0, dal_argument, reply);
        m.set(dcn::mp1_segment0, dal_response, dalsmc::result_ok);
    };
}

ClockManagerConfig config(dcn::Model& model) {
    ClockManagerConfig value{};
    value.io = model.io();
    value.dcn = dcn::space();
    value.dccg_segment = dcn::segment1;
    value.mp1_segment0 = dcn::mp1_segment0;
    value.xtalin_khz = 100000;
    // FbMult_int 42, FbMult_frac 0x8000: 42.5 * 100 MHz.
    value.pll_request = 42u | (0x8000u << 16);
    return value;
}

void check_helpers(Checks& checks) {
    const uint32_t dividers[][2]{{4, 0x08}, {8, 0x08}, {63, 0x3f}, {64, 0x40}, {127, 0x5f},
                                 {128, 0x60}, {247, 0x7d}, {248, 0x7e}, {511, 0x7e},
                                 {512, 0x7f}, {4000, 0x7f}};
    for (const auto& entry : dividers) {
        checks.expect(dentist_did_from_divider(entry[0]) == entry[1], "dentist_get_did_from_divider");
    }
    uint32_t divider = 0;
    checks.expect(dentist_round_clock(4300000, 600000, divider) == 614285 && divider == 28,
                  "dcn32_update_dppclk_dispclk_freq rounding");
    checks.expect(dentist_round_clock(0, 600000, divider) == 0 && divider == 0, "zero VCO refused");
    checks.expect(khz_to_mhz_ceil(1000) == 1 && khz_to_mhz_ceil(1001) == 2 && khz_to_mhz_ceil(0) == 0,
                  "khz_to_mhz_ceil");
}

void check_init(Checks& checks) {
    dcn::Model model;
    Smu smu;
    install(model, smu);
    Dcn32ClockManager clocks;
    checks.expect(clocks.init(config(model)) == ClockResult::success, "clock manager initializes");
    const DisplayClockTable& table = clocks.table();
    checks.expect(table.valid && table.smu_version == 0x004e4000u, "SMU version recorded");
    checks.expect(table.dcfclk.count == 3 && table.dcfclk.mhz[2] == 1200, "DCFCLK levels");
    checks.expect(table.dispclk.count == 3 && table.dispclk.mhz[1] == 1950 &&
                      table.dispclk.mhz[2] == 1950 && table.dispclk.dc_mode_max_mhz == 1950,
                  "DISPCLK capped at 1950 MHz");
    checks.expect(table.dppclk.fine_grained && table.dppclk.count == 2 &&
                      table.dppclk.mhz[0] == 800 && table.dppclk.mhz[1] == 1600,
                  "fine-grained DPPCLK min/max");
    checks.expect(table.uclk.count == 4 && table.fclk.count == 2, "memory clock levels");
    checks.expect(table.dentist_vco_khz == 4250000, "VCO from CLK1_CLK_PLL_REQ");
    checks.expect(table.dprefclk_khz == 716666, "Linux default DPREFCLK");
    checks.expect(table.dccg_ref_khz == 100000 && table.dchub_ref_khz == 100000, "reference clocks");
    checks.expect(!table.memory_info_valid, "memory info needs the ATOM vram_info decode");
    // Version queries first, then DCFCLK, SOCCLK, DTBCLK, DISPCLK, DPPCLK, UCLK, FCLK.
    checks.expect(smu.messages.size() > 4 && smu.messages[0].first == dalsmc::get_smu_version &&
                      smu.messages[3].first == dalsmc::get_dpm_frequency_by_index &&
                      smu.messages[3].second == ((11u << 16) | 0xffu),
                  "dcn32_init_clocks order begins with DCFCLK");
    checks.expect(model.written(dcn::segment1, 0x0081u).empty(), "init changes no DCCG register");
    // Mailbox protocol: RESP cleared, then ARG, then MSG.
    const size_t response = model.write_index(dcn::mp1_segment0, dal_response);
    const size_t argument = model.write_index(dcn::mp1_segment0, dal_argument);
    const size_t message = model.write_index(dcn::mp1_segment0, dal_message);
    checks.expect(response < argument && argument < message, "RESP, ARG, MSG write order");
    checks.expect(clocks.init(config(model)) == ClockResult::invalid_argument, "init once");

    smu.header_version = 2;
    Dcn32ClockManager second;
    checks.expect(second.init(config(model)) == ClockResult::rejected, "header version must be 1");

    dcn::Model moved;
    ClockManagerConfig wrong = config(moved);
    wrong.mp1_segment0 = 0x16200u;
    Dcn32ClockManager third;
    checks.expect(third.init(wrong) == ClockResult::discovery_mismatch && moved.writes.empty(),
                  "MP1 base mismatch touches nothing");

    dcn::Model boot;
    Smu boot_smu;
    install(boot, boot_smu);
    ClockManagerConfig booted = config(boot);
    booted.boot_dprefclk_khz = 600000;
    booted.pll_request = 0;
    Dcn32ClockManager fourth;
    checks.expect(fourth.init(booted) == ClockResult::success &&
                      fourth.table().dprefclk_khz == 600000 &&
                      fourth.table().dentist_vco_khz == 4300000,
                  "boot DPREFCLK and default VCO");
}

void check_mailbox_failures(Checks& checks) {
    dcn::Model model;
    Smu smu;
    install(model, smu);
    smu.silent = true;
    Dcn32ClockManager clocks;
    checks.expect(clocks.init(config(model)) == ClockResult::quarantined && clocks.quarantined(),
                  "unanswered message quarantines");
    checks.expect(model.delayed_us <= 2000000u + 10u, "bounded 2 s DAL poll");

    dcn::Model busy;
    Smu busy_smu;
    install(busy, busy_smu);
    busy.set(dcn::mp1_segment0, dal_response, 0); // previous message never finished
    DalSmcMailbox mailbox;
    checks.expect(mailbox.bind(busy.io(), dcn::mp1_segment0) == ClockResult::success,
                  "mailbox binds");
    checks.expect(mailbox.send(dalsmc::get_smu_version, 0, nullptr) == ClockResult::rejected &&
                      busy.writes.empty() && !mailbox.quarantined(),
                  "busy mailbox refused before any write");

    dcn::Model rejecting;
    Smu rejecting_smu;
    install(rejecting, rejecting_smu);
    rejecting_smu.reject_message = dalsmc::get_smu_version;
    DalSmcMailbox second;
    uint32_t value = 77;
    second.bind(rejecting.io(), dcn::mp1_segment0);
    checks.expect(second.send(dalsmc::get_smu_version, 0, &value) == ClockResult::rejected &&
                      value == 77 && second.last_response() == dalsmc::result_rejected_prerequisite,
                  "non-OK response leaves output unchanged");

    dcn::Model failing;
    Smu failing_smu;
    install(failing, failing_smu);
    failing.fail_write = 1; // ARG write
    DalSmcMailbox third;
    third.bind(failing.io(), dcn::mp1_segment0);
    checks.expect(third.send(dalsmc::get_smu_version, 0, nullptr) == ClockResult::quarantined &&
                      third.quarantined() &&
                      third.send(dalsmc::get_smu_version, 0, nullptr) == ClockResult::quarantined,
                  "failed mailbox write quarantines permanently");
}

size_t message_index(const Smu& smu, uint32_t message, uint32_t argument, size_t start = 0) {
    for (size_t index = start; index < smu.messages.size(); ++index) {
        if (smu.messages[index].first == message && smu.messages[index].second == argument) {
            return index;
        }
    }
    return SIZE_MAX;
}

void check_update(Checks& checks) {
    dcn::Model model;
    Smu smu;
    install(model, smu);
    Dcn32ClockManager clocks;
    clocks.init(config(model));
    smu.messages.clear();
    model.writes.clear();

    DisplayClocks wanted{};
    wanted.dispclk_khz = 600000;
    wanted.dppclk_khz = 600000;
    wanted.dpp_pipe_khz[0] = 300000;
    wanted.dcfclk_khz = 600000;
    wanted.dcfclk_deep_sleep_khz = 8001;
    wanted.dramclk_khz = 772000;
    wanted.display_count = 1;
    DisplayClocks applied{};
    checks.expect(clocks.update(wanted, false, applied) == ClockResult::success, "raise succeeds");
    // 4 * 4250000 / 600000 = 28 -> 607142 kHz.
    checks.expect(applied.dispclk_khz == 607142 && applied.dppclk_khz == 607142,
                  "dispclk/dppclk dentist rounded");
    const size_t dcfclk = message_index(smu, dalsmc::set_hard_min_by_frequency, (11u << 16) | 600u);
    const size_t deep = message_index(smu, dalsmc::set_min_deep_sleep_dcfclk, 9);
    const size_t wait = message_index(smu, dalsmc::set_always_wait_dmcub_response, 0);
    const size_t uclk = message_index(smu, dalsmc::set_hard_min_by_frequency, (2u << 16) | 772u);
    const size_t dtb = message_index(smu, dalsmc::set_hard_min_by_frequency, (12u << 16) | 600u);
    const size_t dpp = message_index(smu, dalsmc::set_hard_min_by_frequency, (9u << 16) | 607u);
    const size_t disp = message_index(smu, dalsmc::set_hard_min_by_frequency, (8u << 16) | 607u);
    // Displays on (enter_display_off false) == safe_to_lower false: NumOfDisplays(1) first.
    checks.expect(message_index(smu, dalsmc::number_of_displays, 1) == 0,
                  "NumOfDisplays when display_off == safe_to_lower");
    checks.expect(dcfclk < deep && deep < wait && wait < uclk && uclk < dtb && dtb < dpp &&
                      dpp < disp,
                  "dcn32_update_clocks raise order");
    // dccg2_update_dpp_dto: phase ceil(255 * 300000 / 607142) = 127, modulo 0xff.
    checks.expect(model.get(dcn::segment1, 0x0099u) == (127u | (0xffu << 16)) &&
                      (model.get(dcn::segment1, 0x00b6u) & 1u) == 1u,
                  "DPP DTO phase/modulo and enable");
    const size_t dto = model.write_index(dcn::segment1, 0x0099u);
    checks.expect(dto != SIZE_MAX, "DPP DTO written after the DISPCLK hard minimum");

    // Lowering without safe_to_lower changes nothing.
    smu.messages.clear();
    model.writes.clear();
    DisplayClocks lower = wanted;
    lower.dispclk_khz = 300000;
    lower.dppclk_khz = 300000;
    lower.dpp_pipe_khz[0] = 150000;
    lower.dcfclk_khz = 300000;
    checks.expect(clocks.update(lower, false, applied) == ClockResult::success &&
                      applied.dispclk_khz == 607142 && applied.dcfclk_khz == 600000 &&
                      model.written(dcn::segment1, 0x0099u).empty(),
                  "no lowering before safe_to_lower");
    smu.messages.clear();
    model.writes.clear();
    checks.expect(clocks.update(lower, true, applied) == ClockResult::success &&
                      applied.dispclk_khz == 303571 && applied.dcfclk_khz == 300000,
                  "lowering with safe_to_lower");
    const size_t lowered_disp = message_index(smu, dalsmc::set_hard_min_by_frequency, (8u << 16) | 303u);
    const size_t lowered_dpp = message_index(smu, dalsmc::set_hard_min_by_frequency, (9u << 16) | 303u);
    checks.expect(lowered_disp != SIZE_MAX && lowered_dpp != SIZE_MAX && lowered_disp < lowered_dpp &&
                      !model.written(dcn::segment1, 0x0099u).empty(),
                  "lowering: DPP DTO, dentist DISPCLK, then DPPCLK");

    // P-state unsupported pins UCLK to the top level (only with !safe_to_lower).
    smu.messages.clear();
    DisplayClocks pinned = lower;
    pinned.p_state_change_support = false;
    checks.expect(clocks.update(pinned, false, applied) == ClockResult::success &&
                      message_index(smu, dalsmc::set_hard_min_by_frequency, (2u << 16) | 1249u) !=
                          SIZE_MAX &&
                      !applied.p_state_change_support,
                  "p_state_change_support false pins UCLK max");

    DisplayClocks invalid = wanted;
    invalid.dpp_pipe_khz[1] = 700000;
    checks.expect(clocks.update(invalid, true, applied) == ClockResult::invalid_argument,
                  "pipe DPP clock above global DPPCLK refused");
    DisplayClocks too_fast = wanted;
    too_fast.dispclk_khz = 2400000;
    checks.expect(clocks.update(too_fast, true, applied) == ClockResult::invalid_argument,
                  "DISPCLK above 1950 MHz refused");

    // Off: DISPCLK through divider 127 needs OTG pixel add/drop.
    DisplayClocks slow = wanted;
    slow.dispclk_khz = 8000; // 4 * 4250000 / 8000 = 2125 -> DID 0x7f
    checks.expect(clocks.update(slow, true, applied) == ClockResult::unsupported_transition,
                  "divider 127 transition refused");

    // Release returns to the lowest levels and turns DPP DTOs off.
    checks.expect(clocks.release() == ClockResult::success &&
                      (model.get(dcn::segment1, 0x00b6u) & 1u) == 0u,
                  "release lowers clocks and disables DPP DTO");
}

void check_dto(Checks& checks) {
    dcn::Model model;
    Smu smu;
    install(model, smu);
    Dcn32ClockManager clocks;
    checks.expect(clocks.program_dp_dto(0, 1485000) == ClockResult::not_initialized,
                  "DTO before init refused");
    clocks.init(config(model));
    model.set(dcn::segment1, 0x0084u, 0x00000008u); // OTG1 DTBCLK DTO bit kept
    checks.expect(clocks.program_dp_dto(1, 1485000) == ClockResult::success, "148.5 MHz DTO");
    checks.expect(model.get(dcn::segment1, 0x0085u) == 148500000u &&
                      model.get(dcn::segment1, 0x0086u) == 716666000u,
                  "PHASE = 100 Hz * 100, MODULO = DPREFCLK Hz");
    checks.expect(model.get(dcn::segment1, 0x0084u) == (0x08u | 0x10u | 0x1000u),
                  "DP_DTO0_ENABLE and PIPE0_DTO_SRC_SEL=1");
    const size_t phase = model.write_index(dcn::segment1, 0x0085u);
    const size_t modulo = model.write_index(dcn::segment1, 0x0086u);
    const size_t control = model.write_index(dcn::segment1, 0x0084u);
    checks.expect(phase < modulo && modulo < control, "PHASE, MODULO, then enable");

    // 148.3516 MHz (1000/1001) uses video_optimized_pixel_rates.
    checks.expect(clocks.program_dp_dto(0, 1483516) == ClockResult::success &&
                      model.get(dcn::segment1, 0x0081u) == 148500000u &&
                      model.get(dcn::segment1, 0x0082u) == 716666u * 1001u,
                  "optimized 1000/1001 rate table");
    checks.expect(clocks.program_dp_dto(0, 8000000) == ClockResult::invalid_argument,
                  "pixel clock above DPREFCLK refused");
    checks.expect(clocks.program_dp_dto(4, 1485000) == ClockResult::invalid_argument, "OTG4 refused");

    checks.expect(clocks.select_tmds_source(1) == ClockResult::success &&
                      (model.get(dcn::segment1, 0x0084u) & 0x3000u) == 0,
                  "TMDS clears PIPE_DTO_SRC_SEL");
    checks.expect(clocks.program_pixel_resync(4, 2) == ClockResult::success &&
                      model.get(dcn::segment1, 0x004cu) == 0x20u,
                  "PHYPLLE deep colour 12 bpc");
    checks.expect(clocks.program_pixel_resync(5, 0) == ClockResult::invalid_argument, "PLL5 refused");
    checks.expect(clocks.set_pixel_rate_div(2, 1, 3) == ClockResult::success &&
                      model.get(dcn::segment1, 0x006fu) == 0x1c0u,
                  "OTG2 pixel rate divider fields");
    const size_t before = model.writes.size();
    checks.expect(clocks.set_pixel_rate_div(2, 1, 3) == ClockResult::success &&
                      model.writes.size() == before,
                  "unchanged divider is not rewritten");
    checks.expect(clocks.set_pixel_rate_div(0, 2, 0) == ClockResult::invalid_argument &&
                      clocks.set_pixel_rate_div(0, 0, 2) == ClockResult::invalid_argument,
                  "invalid divider values refused");
    checks.expect(clocks.clear_pixel_clock(0) == ClockResult::success &&
                      (model.get(dcn::segment1, 0x0080u) & 0x3010u) == 0,
                  "clear_pixel_clock disables the DTO");
    model.fail_write = model.writes.size();
    checks.expect(clocks.program_dp_dto(2, 1485000) == ClockResult::quarantined && clocks.quarantined(),
                  "failed DCCG write quarantines");
    checks.expect(clocks.program_dp_dto(2, 1485000) == ClockResult::quarantined,
                  "quarantine is sticky");
    clocks.reset_proven();
    checks.expect(!clocks.quarantined() && !clocks.initialized(), "reset proof clears the manager");
}

} // namespace

int main() {
    Checks checks;
    check_helpers(checks);
    check_init(checks);
    check_mailbox_failures(checks);
    check_update(checks);
    check_dto(checks);
    return checks.finish("dcn32_clocks");
}

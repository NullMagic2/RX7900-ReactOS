/*
Module: rx7900.tests.dcn32_stream_reference
Purpose: Independent multi-pipe transcription of the Linux v6.12 DC commit, plane update, disable and DPMS sequences for DCN 3.2 (ODM combine, DSC, MPO).
Created: 2026-10-24
Architecture: Test-only. Written from the pinned sources in
work/upstream-dcn-stream (dc.c dc_commit_state_no_check/disable_dangling_plane,
dce110_hwseq.c, dcn10/20/30/31/32_hwseq.c, dcn10/30/32_optc.c, dcn10/20_opp.c,
dcn10/20/30/32_mpc.c, dcn10/20/21/30/32_hubp.c, dcn10/30/32_dpp*.c, dcn20/30/32
hubbub, dcn10/20/30/32 stream encoders, dcn30_vpg.c, dcn30_afmt.c,
dcn32_dccg.c, link_dpms.c, link_hwss_dio.c, dc_hw_sequencer.c, and for DSC
work/upstream-dcn-dsc-odm) without using
the owner's register catalog: every register and field is resolved by its AMD
header name from dcn_3_2_0_offset.h/dcn_3_2_0_sh_mask.h at test time, and the
dc_helper.c REG_SET/REG_UPDATE/REG_GET/REG_WAIT semantics are re-implemented
here (a field absent from the header has mask 0, as an absent Linux mask
entry). It drives the same synthetic model and hooks as the owner so the two
traces compare access for access. Only the deviations the stream owner
documents are applied, each marked DEVIATION at its Linux call site.
*/
#pragma once

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <initializer_list>
#include <map>
#include <regex>
#include <sstream>
#include <string>
#include <utility>

#include "dcn32_stream_model.hpp"

namespace rx7900_test::stream {

struct Headers final {
    std::map<std::string, uint64_t> values;

    bool load(const std::string& directory) {
        for (const char* name : {"/dcn_3_2_0_offset.h", "/dcn_3_2_0_sh_mask.h"}) {
            std::ifstream in(directory + name);
            if (!in) {
                return false;
            }
            const std::regex define(R"(#define\s+(\w+)\s+(0x[0-9a-fA-F]+|\d+)L?\s*$)");
            std::string line;
            while (std::getline(in, line)) {
                if (!line.empty() && line.back() == '\r') {
                    line.pop_back();
                }
                std::smatch m;
                if (std::regex_search(line, m, define)) {
                    values[m[1]] = std::stoull(m[2], nullptr, 0);
                }
            }
        }
        return !values.empty();
    }
    bool has(const std::string& key) const { return values.count(key) != 0; }
    uint64_t at(const std::string& key) const {
        const auto found = values.find(key);
        if (found == values.end()) {
            std::fprintf(stderr, "reference: no header symbol %s\n", key.c_str());
            std::abort();
        }
        return found->second;
    }
};

class Reference final {
public:
    Reference(Model& model, Hooks& hooks, const Headers& headers) : model_(model), hooks_(hooks), h_(headers) {
        io_ = model_.io();
    }

    // ---------------- register helpers (dc_helper.c) ----------------
    struct Reg final {
        std::string name;
        std::string masks; // instance-0 name: Linux takes every instance's masks from it
    };
    struct F final {
        const char* name;
        uint32_t value;
    };

    static Reg reg(const char* format, uint32_t instance = 0) {
        char name[128];
        char masks[128];
        std::snprintf(name, sizeof(name), format, instance);
        std::snprintf(masks, sizeof(masks), format, 0u);
        return {name, masks};
    }
    static Reg plain(const char* name) { return {name, name}; }

    uint32_t addr(const Reg& r) const {
        const uint64_t base_index = h_.at("reg" + r.name + "_BASE_IDX");
        return static_cast<uint32_t>((segment[base_index] + h_.at("reg" + r.name)) * 4u);
    }
    std::pair<uint32_t, uint32_t> field(const Reg& r, const char* f) const {
        const std::string shift = r.masks + "__" + f + "__SHIFT";
        const std::string mask = r.masks + "__" + f + "_MASK";
        if (!h_.has(shift) || !h_.has(mask)) {
            return {0, 0};
        }
        return {static_cast<uint32_t>(h_.at(shift)), static_cast<uint32_t>(h_.at(mask))};
    }
    uint32_t place(uint32_t word, const Reg& r, const F& f) const {
        const auto sm = field(r, f.name);
        return (word & ~sm.second) | ((f.value << sm.first) & sm.second);
    }
    uint32_t READ(const Reg& r) {
        uint32_t v = 0;
        io_.read32(io_.context, addr(r), v);
        return v;
    }
    void WRITE(const Reg& r, uint32_t v) { io_.write32(io_.context, addr(r), v); }
    void SET(const Reg& r, uint32_t init, std::initializer_list<F> fields) {
        uint32_t v = init;
        for (const F& f : fields) {
            v = place(v, r, f);
        }
        WRITE(r, v);
    }
    void UPDATE(const Reg& r, std::initializer_list<F> fields) {
        uint32_t v = READ(r);
        for (const F& f : fields) {
            v = place(v, r, f);
        }
        WRITE(r, v);
    }
    uint32_t GET(const Reg& r, const char* f) {
        const auto sm = field(r, f);
        return (READ(r) & sm.second) >> sm.first;
    }
    bool WAIT(const Reg& r, const char* f, uint32_t condition, uint32_t delay_us, uint32_t tries) {
        const auto sm = field(r, f);
        for (uint32_t i = 0; i <= tries; ++i) {
            if (i != 0) {
                if (delay_us >= 1000) {
                    msleep(delay_us / 1000);
                } else if (delay_us > 0) {
                    udelay(delay_us);
                }
            }
            if (((READ(r) & sm.second) >> sm.first) == condition) {
                return true;
            }
        }
        return false;
    }
    void udelay(uint32_t us) { io_.delay_microseconds(io_.context, us); }
    void msleep(uint32_t ms) { io_.delay_microseconds(io_.context, ms * 1000u); }

    // ---------------- hubbub (dcn32_hubbub.c, dcn10/30 hubbub) ----------------
    static uint32_t convert_and_clamp(uint32_t wm_ns, uint32_t refclk_mhz, uint32_t clamp) {
        uint32_t value = wm_ns * refclk_mhz;
        value /= 1000;
        return value > clamp ? clamp : value;
    }

    void program_watermarks(const rx7900::display::Dcn32BandwidthPlan& bw, bool safe_to_lower) {
        const rx7900::display::Dcn32Watermarks* wm[4] = {&bw.a, &bw.b, &bw.c, &bw.d};
        const char* set_name[4] = {"A", "B", "C", "D"};
        auto program = [&](uint32_t set, uint32_t rx7900::display::Dcn32Watermarks::*member, const char* reg_prefix,
                           uint32_t clamp) {
            const uint32_t requested = wm[set]->*member;
            uint32_t& cached = cache_[set].*member;
            if (safe_to_lower || requested > cached) {
                cached = requested;
                const std::string name = std::string(reg_prefix) + "_" + set_name[set];
                const uint32_t value = clamp != 0 ? convert_and_clamp(requested, refclk_mhz_, clamp) : requested;
                SET(plain(name.c_str()), 0, {{name.c_str(), value}});
            }
        };
        using W = rx7900::display::Dcn32Watermarks;
        // hubbub32_program_urgent_watermarks
        for (uint32_t s = 0; s < 4; ++s) {
            program(s, &W::urgent_ns, "DCHUBBUB_ARB_DATA_URGENCY_WATERMARK", 0x3fff);
            program(s, &W::frac_urg_bw_flip, "DCHUBBUB_ARB_FRAC_URG_BW_FLIP", 0);
            program(s, &W::frac_urg_bw_nom, "DCHUBBUB_ARB_FRAC_URG_BW_NOM", 0);
            program(s, &W::urgent_latency_ns, "DCHUBBUB_ARB_REFCYC_PER_TRIP_TO_MEMORY", 0x3fff);
        }
        // hubbub32_program_stutter_watermarks
        for (uint32_t s = 0; s < 4; ++s) {
            program(s, &W::cstate_enter_plus_exit_ns, "DCHUBBUB_ARB_ALLOW_SR_ENTER_WATERMARK", 0xffff);
            program(s, &W::cstate_exit_ns, "DCHUBBUB_ARB_ALLOW_SR_EXIT_WATERMARK", 0xffff);
        }
        // hubbub32_program_pstate_watermarks: UCLK sets A-D, then FCLK sets A-D.
        for (uint32_t s = 0; s < 4; ++s) {
            program(s, &W::pstate_change_ns, "DCHUBBUB_ARB_UCLK_PSTATE_CHANGE_WATERMARK", 0xffff);
        }
        for (uint32_t s = 0; s < 4; ++s) {
            program(s, &W::fclk_pstate_change_ns, "DCHUBBUB_ARB_FCLK_PSTATE_CHANGE_WATERMARK", 0xffff);
        }
        // hubbub32_program_usr_watermarks
        for (uint32_t s = 0; s < 4; ++s) {
            program(s, &W::usr_retraining_ns, "DCHUBBUB_ARB_USR_RETRAINING_WATERMARK", 0x3fff);
        }
        allow_self_refresh_control(true);
        UPDATE(plain("DCHUBBUB_ARB_USR_RETRAINING_CNTL"), {{"DCHUBBUB_ARB_ALLOW_USR_RETRAINING_FORCE_VALUE", 0},
                                                           {"DCHUBBUB_ARB_ALLOW_USR_RETRAINING_FORCE_ENABLE", 0}});
    }
    void allow_self_refresh_control(bool allow) {
        UPDATE(plain("DCHUBBUB_ARB_DRAM_STATE_CNTL"), {{"DCHUBBUB_ARB_ALLOW_SELF_REFRESH_FORCE_VALUE", 0},
                                                       {"DCHUBBUB_ARB_ALLOW_SELF_REFRESH_FORCE_ENABLE", allow ? 0u : 1u}});
    }
    void program_compbuf_size(uint32_t compbuf_kb, bool safe_to_increase) {
        const uint32_t segments = (compbuf_kb + 64 - 1) / 64;
        if (safe_to_increase || segments <= compbuf_segments_) {
            if (segments > compbuf_segments_) {
                for (uint32_t i = 0; i < 4; ++i) {
                    char reg_name[32];
                    char field_name[32];
                    std::snprintf(reg_name, sizeof(reg_name), "DCHUBBUB_DET%u_CTRL", i);
                    std::snprintf(field_name, sizeof(field_name), "DET%u_SIZE_CURRENT", i);
                    WAIT(plain(reg_name), field_name, det_[i], 1, 100);
                }
            }
            UPDATE(plain("DCHUBBUB_COMPBUF_CTRL"), {{"COMPBUF_SIZE", segments}});
            compbuf_segments_ = segments;
            GET(plain("DCHUBBUB_COMPBUF_CTRL"), "CONFIG_ERROR"); // inside ASSERT(); kept in non-debug builds
        }
    }
    void program_det_size(uint32_t hubp, uint32_t det_kb) {
        const uint32_t segments = (det_kb + 64 - 1) / 64;
        char reg_name[32];
        char field_name[32];
        std::snprintf(reg_name, sizeof(reg_name), "DCHUBBUB_DET%u_CTRL", hubp);
        std::snprintf(field_name, sizeof(field_name), "DET%u_SIZE", hubp);
        UPDATE(plain(reg_name), {{field_name, segments}});
        det_[hubp] = segments;
    }
    void force_pstate_change_control(bool force, bool allow) {
        UPDATE(plain("DCHUBBUB_ARB_DRAM_STATE_CNTL"), {{"DCHUBBUB_ARB_ALLOW_PSTATE_CHANGE_FORCE_VALUE", allow},
                                                       {"DCHUBBUB_ARB_ALLOW_PSTATE_CHANGE_FORCE_ENABLE", force}});
    }

    // ---------------- dcn32_init_hw subset (GOP takeover) ----------------
    void init_hw_subset() {
        // dccg31_init (dccg32_funcs.dccg_init): dccg31_disable_symclk32_se(0..3);
        // dcn32 sets no root_clock_optimization bit (symclk32_le, dpstream, physymclk).
        for (uint32_t i = 0; i < 4; ++i) {
            char src[32];
            char en[32];
            std::snprintf(src, sizeof(src), "SYMCLK32_SE%u_SRC_SEL", i);
            std::snprintf(en, sizeof(en), "SYMCLK32_SE%u_EN", i);
            UPDATE(plain("SYMCLK32_SE_CNTL"), {{src, 0}, {en, 0}});
        }
        // DEVIATION: ODM_MEM_PWR_CTRL3 (enable_mem_low_power.bits.optc) is not written.
        // dcn32_enable_power_gating_plane(hws, true)
        const uint32_t org = GET(plain("DC_IP_REQUEST_CNTL"), "IP_REQUEST_EN");
        if (org == 0) {
            SET(plain("DC_IP_REQUEST_CNTL"), 0, {{"IP_REQUEST_EN", 1}});
        }
        for (const char* d : {"DOMAIN0_PG_CONFIG", "DOMAIN1_PG_CONFIG", "DOMAIN2_PG_CONFIG", "DOMAIN3_PG_CONFIG",
                              "DOMAIN16_PG_CONFIG", "DOMAIN17_PG_CONFIG", "DOMAIN18_PG_CONFIG", "DOMAIN19_PG_CONFIG"}) {
            UPDATE(plain(d), {{"DOMAIN_POWER_FORCEON", 0}});
        }
        if (org == 0) {
            SET(plain("DC_IP_REQUEST_CNTL"), 0, {{"IP_REQUEST_EN", 0}});
        }
        // DEVIATION: blank_all_dp_displays/init_pipes are replaced by the Linux
        // disable path of the adopted pipe in the first commit.
        allow_self_refresh_control(true);
        WRITE(plain("DIO_MEM_PWR_CTRL"), 0);
        // !dc->debug.disable_clock_gate: enable all DCN clock gating.
        WRITE(plain("DCCG_GATE_DISABLE_CNTL"), 0);
        WRITE(plain("DCCG_GATE_DISABLE_CNTL2"), 0);
        UPDATE(plain("DCFCLK_CNTL"), {{"DCFCLK_GATE_DIS", 0}});
        // hubbub32_init_watermarks (not accelerated mode)
        for (const char* n : {"DCHUBBUB_ARB_DATA_URGENCY_WATERMARK", "DCHUBBUB_ARB_FRAC_URG_BW_FLIP",
                              "DCHUBBUB_ARB_FRAC_URG_BW_NOM", "DCHUBBUB_ARB_REFCYC_PER_TRIP_TO_MEMORY",
                              "DCHUBBUB_ARB_ALLOW_SR_ENTER_WATERMARK", "DCHUBBUB_ARB_ALLOW_SR_EXIT_WATERMARK",
                              "DCHUBBUB_ARB_USR_RETRAINING_WATERMARK", "DCHUBBUB_ARB_UCLK_PSTATE_CHANGE_WATERMARK",
                              "DCHUBBUB_ARB_FCLK_PSTATE_CHANGE_WATERMARK"}) {
            const std::string base(n);
            const uint32_t v = READ(plain((base + "_A").c_str()));
            WRITE(plain((base + "_B").c_str()), v);
            WRITE(plain((base + "_C").c_str()), v);
            WRITE(plain((base + "_D").c_str()), v);
        }
        force_pstate_change_control(false, false);
        // dcn32_init_crb
        for (uint32_t i = 0; i < 4; ++i) {
            char reg_name[32];
            char field_name[32];
            std::snprintf(reg_name, sizeof(reg_name), "DCHUBBUB_DET%u_CTRL", i);
            std::snprintf(field_name, sizeof(field_name), "DET%u_SIZE_CURRENT", i);
            det_[i] = GET(plain(reg_name), field_name);
        }
        compbuf_segments_ = GET(plain("DCHUBBUB_COMPBUF_CTRL"), "COMPBUF_SIZE_CURRENT");
        SET(plain("COMPBUF_RESERVED_SPACE"), 0, {{"COMPBUF_RESERVED_SPACE_64B", 8 * 1024 / 32},
                                                 {"COMPBUF_RESERVED_SPACE_ZS", 8 * 1024 / 128}});
        UPDATE(plain("DCHUBBUB_DEBUG_CTRL_0"), {{"DET_DEPTH", 0x47f}});
    }

    // ---------------- dc_state / pipe_ctx (dc_resource.c) ----------------
    // An MPCC of the MPC tree (mpcc->dpp_id, mpcc->mpcc_bot as an index).
    struct Mpcc final {
        int dpp_id = 0xf;
        int bot = -1;
    };
    // plane_res.scl_data as dpp1_dscl_set_scaler_manual_scale caches it.
    struct Scl final {
        bool valid = false;
        rx7900::display::Dcn32Rect recout{};
        rx7900::display::Dcn32Rect viewport{};
        uint32_t h_active = 0;
        uint32_t v_active = 0;
        bool alpha_en = false;
    };
    // The pipe_ctx fields the sequences read. A context is built from a plan's
    // pipe topology (validated against Linux in dcn32_bandwidth_tests).
    struct UpdateFlags final {
        bool enable = false, disable = false, odm = false, global_sync = false, mpcc = false, dppclk = false;
        bool hubp_interdependent = false, hubp_rq_dlg_ttu = false, unbounded_req = false, gamut_remap = false;
        bool scaler = false, viewport = false, det_size = false, plane_changed = false, opp_changed = false;
        bool raw() const {
            return enable || disable || odm || global_sync || mpcc || dppclk || hubp_interdependent ||
                   hubp_rq_dlg_ttu || unbounded_req || gamut_remap || scaler || viewport || det_size ||
                   plane_changed || opp_changed;
        }
    };
    struct PipeCtx final {
        bool stream = false;
        int plane = -1; // plane_state: index into the stream plan's planes
        int top = -1, bottom = -1, prev_odm = -1, next_odm = -1;
        int opp = -1, tg = -1, dsc = -1;
        bool hubp = false; // plane_res.hubp != NULL
        UpdateFlags update_flags{};
        rx7900::display::Dcn32PipeBandwidth dlg_param{}; // pipe_dlg_param + dlg/ttu/rq + det
        rx7900::display::Dcn32Rect viewport{}, recout{};
    };
    struct State final {
        PipeCtx pipe[4]{};
        Dcn32StreamPlan stream{}; // context->streams[0] (valid when has_stream)
        bool has_stream = false;
    };
    static bool is_otg_master(const State& s, int i) {
        return s.pipe[i].prev_odm < 0 && s.pipe[i].top < 0 && s.pipe[i].stream;
    }
    static bool is_opp_head(const State& s, int i) { return s.pipe[i].top < 0 && s.pipe[i].stream; }
    static bool is_dpp_pipe(const State& s, int i) { return s.pipe[i].plane >= 0 && s.pipe[i].stream; }
    static int otg_master_of(const State& s) {
        for (int i = 0; i < 4; ++i) {
            if (is_otg_master(s, i)) {
                return i;
            }
        }
        return -1;
    }
    static int odm_slice_count(const State& s, int master) {
        int n = 1;
        for (int p = s.pipe[master].next_odm; p >= 0; p = s.pipe[p].next_odm) {
            ++n;
        }
        return n;
    }
    State state_from(const Dcn32StreamPlan& plan) const {
        State s{};
        s.stream = plan;
        s.has_stream = true;
        const auto& t = plan.bandwidth.topology;
        auto link = [](uint8_t v) { return v == 0xff ? -1 : static_cast<int>(v); };
        for (int i = 0; i < 4; ++i) {
            const auto& tp = t.pipes[i];
            if (!tp.stream) {
                continue;
            }
            PipeCtx& p = s.pipe[i];
            p.stream = true;
            p.plane = tp.has_plane ? tp.plane : -1;
            p.top = link(tp.top);
            p.bottom = link(tp.bottom);
            p.prev_odm = link(tp.prev_odm);
            p.next_odm = link(tp.next_odm);
            p.opp = tp.opp;
            p.tg = static_cast<int>(plan.pipe);
            p.dsc = tp.dsc ? i : -1;
            p.hubp = true;
            for (uint32_t k = 0; k < plan.bandwidth.pipe_count; ++k) {
                if (plan.bandwidth.pipes[k].pipe == static_cast<uint32_t>(i)) {
                    p.dlg_param = plan.bandwidth.pipes[k];
                }
            }
            p.viewport = tp.viewport;
            p.recout = tp.recout;
        }
        return s;
    }
    // resource_get_odm_slice_src_rect of an OPP head (RGB).
    static void odm_slice_src(const State& s, int opp_head, uint32_t& x, uint32_t& width) {
        int master = opp_head;
        int index = 0;
        while (s.pipe[master].prev_odm >= 0) {
            master = s.pipe[master].prev_odm;
            ++index;
        }
        const uint32_t h_active = s.stream.timing.width;
        const uint32_t count = static_cast<uint32_t>(odm_slice_count(s, master));
        const uint32_t w = h_active / count;
        x = w * static_cast<uint32_t>(index);
        width = s.pipe[opp_head].next_odm < 0 ? h_active - w * (count - 1) : w;
    }
    static int opp_head_of(const State& s, int i) {
        while (s.pipe[i].top >= 0) {
            i = s.pipe[i].top;
        }
        return i;
    }
    const rx7900::display::Dcn32PlaneScanout& plane_of(const State& s, int i) const {
        return s.stream.planes[s.pipe[i].plane < 0 ? 0 : s.pipe[i].plane];
    }

    // Adopt the firmware-lit pipe as the current state (plane + stream).
    void adopt(const Dcn32StreamPlan& firmware) {
        cur_ = State{};
        cur_.has_stream = true;
        cur_.stream = firmware;
        cur_.stream.bandwidth = {};
        PipeCtx& p = cur_.pipe[firmware.pipe];
        p.stream = true;
        p.plane = 0;
        p.opp = static_cast<int>(firmware.pipe);
        p.tg = static_cast<int>(firmware.pipe);
        p.hubp = true;
        firmware_ = true;
        dpms_off_ = false;
        mpcc_[firmware.pipe].dpp_id = static_cast<int>(firmware.pipe);
        mpcc_[firmware.pipe].bot = -1;
        opp_list_[firmware.pipe] = static_cast<int>(firmware.pipe);
        power_gated_[firmware.pipe] = false;
    }

    // ---------------- power gating (dcn32_hwseq.c) ----------------
    void dsc_pg_control(uint32_t dsc, bool power_on) {
        const uint32_t org = GET(plain("DC_IP_REQUEST_CNTL"), "IP_REQUEST_EN");
        if (org == 0) {
            SET(plain("DC_IP_REQUEST_CNTL"), 0, {{"IP_REQUEST_EN", 1}});
        }
        char config[32];
        char status[32];
        std::snprintf(config, sizeof(config), "DOMAIN%u_PG_CONFIG", 16 + dsc);
        std::snprintf(status, sizeof(status), "DOMAIN%u_PG_STATUS", 16 + dsc);
        UPDATE(plain(config), {{"DOMAIN_POWER_GATE", power_on ? 0u : 1u}});
        WAIT(plain(status), "DOMAIN_PGFSM_PWR_STATUS", power_on ? 0u : 2u, 1, 1000);
        if (org == 0) {
            SET(plain("DC_IP_REQUEST_CNTL"), 0, {{"IP_REQUEST_EN", 0}});
        }
    }
    bool dsc_pg_status(uint32_t dsc) { // dcn32_dsc_pg_status
        char status[32];
        std::snprintf(status, sizeof(status), "DOMAIN%u_PG_STATUS", 16 + dsc);
        return GET(plain(status), "DOMAIN_PGFSM_PWR_STATUS") == 0;
    }
    void update_dsc_pg(const State& context, bool safe_to_disable) { // dcn32_update_dsc_pg
        for (uint32_t dsc = 0; dsc < 4; ++dsc) {
            const bool ungated = dsc_pg_status(dsc);
            const bool acquired = context.has_stream && context.pipe[dsc].stream && context.pipe[dsc].dsc >= 0;
            if (acquired) {
                if (!ungated) {
                    dsc_pg_control(dsc, true);
                }
            } else if (safe_to_disable && ungated) {
                dsc_pg_control(dsc, false);
            }
        }
    }
    void hubp_pg_control(uint32_t hubp, bool power_on) {
        char config[32];
        char status[32];
        std::snprintf(config, sizeof(config), "DOMAIN%u_PG_CONFIG", hubp);
        std::snprintf(status, sizeof(status), "DOMAIN%u_PG_STATUS", hubp);
        SET(plain(config), 0, {{"DOMAIN_POWER_GATE", power_on ? 0u : 1u}});
        WAIT(plain(status), "DOMAIN_PGFSM_PWR_STATUS", power_on ? 0u : 2u, 1, 1000);
    }

    // ---------------- OPTC ----------------
    bool is_tg_enabled(uint32_t otg) { return GET(reg("OTG%u_OTG_CONTROL", otg), "OTG_MASTER_EN") != 0; }
    void pipe_control_lock(uint32_t otg, bool lock) {
        if (lock) { // optc3_lock
            UPDATE(reg("OTG%u_OTG_GLOBAL_CONTROL2", otg), {{"OTG_MASTER_UPDATE_LOCK_SEL", otg}});
            SET(reg("OTG%u_OTG_MASTER_UPDATE_LOCK", otg), 0, {{"OTG_MASTER_UPDATE_LOCK", 1}});
            WAIT(reg("OTG%u_OTG_MASTER_UPDATE_LOCK", otg), "UPDATE_LOCK_STATUS", 1, 1, 10);
        } else { // optc1_unlock
            SET(reg("OTG%u_OTG_MASTER_UPDATE_LOCK", otg), 0, {{"OTG_MASTER_UPDATE_LOCK", 0}});
        }
    }
    void interdependent_update_lock(const State& context, bool lock) { // dcn32_interdependent_update_lock
        for (int i = 0; i < 4; ++i) {
            if (!is_otg_master(context, i) || !is_tg_enabled(static_cast<uint32_t>(context.pipe[i].tg))) {
                continue;
            }
            pipe_control_lock(static_cast<uint32_t>(context.pipe[i].tg), lock); // dcn20_pipe_control_lock
        }
    }
    void disable_crtc(uint32_t otg) { // optc32_disable_crtc
        UPDATE(reg("ODM%u_OPTC_DATA_SOURCE_SELECT", otg),
               {{"OPTC_SEG0_SRC_SEL", 0xf}, {"OPTC_SEG1_SRC_SEL", 0xf}, {"OPTC_SEG2_SRC_SEL", 0xf},
                {"OPTC_SEG3_SRC_SEL", 0xf}, {"OPTC_NUM_OF_INPUT_SEGMENT", 0}});
        UPDATE(reg("ODM%u_OPTC_MEMORY_CONFIG", otg), {{"OPTC_MEM_SEL", 0}});
        UPDATE(reg("OTG%u_OTG_CONTROL", otg), {{"OTG_MASTER_EN", 0}});
        UPDATE(reg("VTG%u_CONTROL", otg), {{"VTG0_ENABLE", 0}});
        WAIT(reg("OTG%u_OTG_CLOCK_CONTROL", otg), "OTG_BUSY", 0, 1, 150000);
    }
    void enable_optc_clock(uint32_t otg, bool enable) { // optc1_enable_optc_clock
        if (enable) {
            UPDATE(reg("ODM%u_OPTC_INPUT_CLOCK_CONTROL", otg), {{"OPTC_INPUT_CLK_EN", 1}, {"OPTC_INPUT_CLK_GATE_DIS", 1}});
            WAIT(reg("ODM%u_OPTC_INPUT_CLOCK_CONTROL", otg), "OPTC_INPUT_CLK_ON", 1, 1, 1000);
            UPDATE(reg("OTG%u_OTG_CLOCK_CONTROL", otg), {{"OTG_CLOCK_EN", 1}, {"OTG_CLOCK_GATE_DIS", 1}});
            WAIT(reg("OTG%u_OTG_CLOCK_CONTROL", otg), "OTG_CLOCK_ON", 1, 1, 1000);
        } else {
            if (GET(reg("ODM%u_OPTC_INPUT_GLOBAL_CONTROL", otg), "OPTC_UNDERFLOW_OCCURRED_STATUS") == 1) {
                UPDATE(reg("ODM%u_OPTC_INPUT_GLOBAL_CONTROL", otg), {{"OPTC_UNDERFLOW_CLEAR", 1}});
            }
            UPDATE(reg("OTG%u_OTG_CLOCK_CONTROL", otg), {{"OTG_CLOCK_GATE_DIS", 0}, {"OTG_CLOCK_EN", 0}});
            UPDATE(reg("ODM%u_OPTC_INPUT_CLOCK_CONTROL", otg), {{"OPTC_INPUT_CLK_GATE_DIS", 0}, {"OPTC_INPUT_CLK_EN", 0}});
        }
    }
    void set_odm_bypass(uint32_t otg) { // optc32_set_odm_bypass
        SET(reg("ODM%u_OPTC_DATA_SOURCE_SELECT", otg), 0,
            {{"OPTC_NUM_OF_INPUT_SEGMENT", 0}, {"OPTC_SEG0_SRC_SEL", otg}, {"OPTC_SEG1_SRC_SEL", 0xf},
             {"OPTC_SEG2_SRC_SEL", 0xf}, {"OPTC_SEG3_SRC_SEL", 0xf}});
        UPDATE(reg("OTG%u_OTG_H_TIMING_CNTL", otg), {{"OTG_H_TIMING_DIV_MODE", 0}});
        SET(reg("ODM%u_OPTC_MEMORY_CONFIG", otg), 0, {{"OPTC_MEM_SEL", 0}});
        opp_count_[otg] = 1;
    }
    // optc32_set_odm_combine(opp_id[], opp_cnt, segment_width, last_segment_width)
    void set_odm_combine(uint32_t otg, const int* opp_id, int opp_cnt, int segment_width) {
        uint32_t memory_mask = 0;
        const int h_active = segment_width * opp_cnt;
        const int odm_mem_count = (h_active + 2047) / 2048;
        if (opp_cnt == 4) {
            memory_mask = odm_mem_count <= 2 ? 0x3 : (odm_mem_count <= 4 ? 0xf : 0x3f);
        } else {
            if (odm_mem_count <= 2) {
                memory_mask = 0x1u << (opp_id[0] * 2) | 0x1u << (opp_id[1] * 2);
            } else if (odm_mem_count <= 4) {
                memory_mask = 0x3u << (opp_id[0] * 2) | 0x3u << (opp_id[1] * 2);
            } else {
                memory_mask = 0x77;
            }
        }
        SET(reg("ODM%u_OPTC_MEMORY_CONFIG", otg), 0, {{"OPTC_MEM_SEL", memory_mask}});
        if (opp_cnt == 2) {
            SET(reg("ODM%u_OPTC_DATA_SOURCE_SELECT", otg), 0,
                {{"OPTC_NUM_OF_INPUT_SEGMENT", 1}, {"OPTC_SEG0_SRC_SEL", static_cast<uint32_t>(opp_id[0])},
                 {"OPTC_SEG1_SRC_SEL", static_cast<uint32_t>(opp_id[1])}});
        } else if (opp_cnt == 4) {
            SET(reg("ODM%u_OPTC_DATA_SOURCE_SELECT", otg), 0,
                {{"OPTC_NUM_OF_INPUT_SEGMENT", 3}, {"OPTC_SEG0_SRC_SEL", static_cast<uint32_t>(opp_id[0])},
                 {"OPTC_SEG1_SRC_SEL", static_cast<uint32_t>(opp_id[1])},
                 {"OPTC_SEG2_SRC_SEL", static_cast<uint32_t>(opp_id[2])},
                 {"OPTC_SEG3_SRC_SEL", static_cast<uint32_t>(opp_id[3])}});
        }
        UPDATE(reg("ODM%u_OPTC_WIDTH_CONTROL", otg), {{"OPTC_SEGMENT_WIDTH", static_cast<uint32_t>(segment_width)}});
        UPDATE(reg("OTG%u_OTG_H_TIMING_CNTL", otg), {{"OTG_H_TIMING_DIV_MODE", static_cast<uint32_t>(opp_cnt - 1)}});
        opp_count_[otg] = static_cast<uint32_t>(opp_cnt);
    }
    void set_drr_no_params(uint32_t otg) { // optc32_set_drr with v_total min/max 0 -> optc32_setup_manual_trigger
        UPDATE(reg("OTG%u_OTG_V_TOTAL_CONTROL", otg), {{"OTG_V_TOTAL_MIN_SEL", 1}, {"OTG_V_TOTAL_MAX_SEL", 1},
                                                       {"OTG_FORCE_LOCK_ON_EVENT", 0}, {"OTG_SET_V_TOTAL_MIN_MASK", 2}});
    }
    void set_static_screen_control(uint32_t otg) {
        SET(reg("OTG%u_OTG_STATIC_SCREEN_CONTROL", otg), 0,
            {{"OTG_STATIC_SCREEN_EVENT_MASK", 0}, {"OTG_STATIC_SCREEN_FRAME_COUNT", 2}});
    }
    static uint32_t vfp(const rx7900::display::Timing& t) { return t.v_front_porch < 1 ? 1 : t.v_front_porch; }
    // optc1_program_global_sync(vready_offset, vstartup_start, vupdate_offset, vupdate_width)
    void program_global_sync(uint32_t otg, uint32_t vready, uint32_t vstartup, uint32_t vupdate_offset,
                             uint32_t vupdate_width) {
        if (vstartup == 0) {
            return;
        }
        SET(reg("OTG%u_OTG_VSTARTUP_PARAM", otg), 0, {{"VSTARTUP_START", vstartup}});
        SET(reg("OTG%u_OTG_VUPDATE_PARAM", otg), 0, {{"VUPDATE_OFFSET", vupdate_offset},
                                                     {"VUPDATE_WIDTH", vupdate_width}});
        SET(reg("OTG%u_OTG_VREADY_PARAM", otg), 0, {{"VREADY_OFFSET", vready}});
    }
    void set_vtg_params(uint32_t otg, const rx7900::display::Timing& t, uint32_t vstartup, bool program_fp2) {
        const uint32_t v_init = t.v_total - vfp(t);
        const uint32_t asic_blank_end = v_init - t.height;
        const int32_t line_start = static_cast<int32_t>(asic_blank_end) - static_cast<int32_t>(vstartup) + 1;
        const uint32_t v_fp2 = line_start < 0 ? static_cast<uint32_t>(-line_start) : 0u;
        if (program_fp2) {
            UPDATE(reg("VTG%u_CONTROL", otg), {{"VTG0_FP2", v_fp2}, {"VTG0_VCOUNT_INIT", v_init}});
        } else {
            UPDATE(reg("VTG%u_CONTROL", otg), {{"VTG0_VCOUNT_INIT", v_init}});
        }
    }
    void program_timing(const State& s, int master) { // optc1_program_timing
        const auto& t = s.stream.timing;
        const uint32_t o = static_cast<uint32_t>(s.pipe[master].tg);
        const auto& dlg = s.pipe[master].dlg_param;
        SET(reg("OTG%u_OTG_H_TOTAL", o), 0, {{"OTG_H_TOTAL", t.h_total - 1}});
        UPDATE(reg("OTG%u_OTG_H_SYNC_A", o), {{"OTG_H_SYNC_A_START", 0}, {"OTG_H_SYNC_A_END", t.h_sync_width}});
        const uint32_t hbs = t.h_total - t.h_front_porch;
        UPDATE(reg("OTG%u_OTG_H_BLANK_START_END", o), {{"OTG_H_BLANK_START", hbs}, {"OTG_H_BLANK_END", hbs - t.width}});
        UPDATE(reg("OTG%u_OTG_H_SYNC_A_CNTL", o),
               {{"OTG_H_SYNC_A_POL", (t.flags & rx7900::display::positive_hsync) ? 0u : 1u}});
        const uint32_t v_total = t.v_total - 1;
        SET(reg("OTG%u_OTG_V_TOTAL", o), 0, {{"OTG_V_TOTAL", v_total}});
        // optc3_set_vtotal_min_max without FAMS -> optc1_set_vtotal_min_max
        SET(reg("OTG%u_OTG_V_TOTAL_MAX", o), 0, {{"OTG_V_TOTAL_MAX", v_total}});
        SET(reg("OTG%u_OTG_V_TOTAL_MIN", o), 0, {{"OTG_V_TOTAL_MIN", v_total}});
        UPDATE(reg("OTG%u_OTG_V_SYNC_A", o), {{"OTG_V_SYNC_A_START", 0}, {"OTG_V_SYNC_A_END", t.v_sync_width}});
        const uint32_t vbs = t.v_total - vfp(t);
        UPDATE(reg("OTG%u_OTG_V_BLANK_START_END", o), {{"OTG_V_BLANK_START", vbs}, {"OTG_V_BLANK_END", vbs - t.height}});
        UPDATE(reg("OTG%u_OTG_V_SYNC_A_CNTL", o),
               {{"OTG_V_SYNC_A_POL", (t.flags & rx7900::display::positive_vsync) ? 0u : 1u}});
        const bool dp = s.stream.signal == rx7900::display::StreamSignal::display_port;
        UPDATE(reg("OTG%u_OTG_INTERLACE_CONTROL", o), {{"OTG_INTERLACE_ENABLE", 0}});
        UPDATE(reg("VTG%u_CONTROL", o), {{"VTG0_ENABLE", 0}});
        UPDATE(reg("OTG%u_OTG_CONTROL", o), {{"OTG_START_POINT_CNTL", dp ? 1u : 0u}, {"OTG_FIELD_NUMBER_CNTL", 0}});
        // the OTG master's own pipe_dlg_param
        program_global_sync(o, dlg.vready_offset, dlg.vstartup_start, dlg.vupdate_offset, dlg.vupdate_width);
        set_vtg_params(o, t, dlg.vstartup_start, true);
        UPDATE(reg("ODM%u_OPTC_DATA_FORMAT_CONTROL", o), {{"OPTC_DATA_FORMAT", 0}});
        const uint32_t h_div = opp_count_[o] == 4 ? 3u : (opp_count_[o] == 2 ? 1u : 0u); // optc1->opp_count
        UPDATE(reg("OTG%u_OTG_H_TIMING_CNTL", o), {{"OTG_H_TIMING_DIV_MODE", h_div}});
    }
    void enable_crtc(uint32_t otg) { // optc32_enable_crtc
        UPDATE(reg("ODM%u_OPTC_DATA_SOURCE_SELECT", otg), {{"OPTC_SEG0_SRC_SEL", otg}});
        UPDATE(reg("VTG%u_CONTROL", otg), {{"VTG0_ENABLE", 1}});
        UPDATE(reg("OTG%u_OTG_CONTROL", otg), {{"OTG_DISABLE_POINT_CNTL", 2}, {"OTG_MASTER_EN", 1}});
    }
    void wait_for_state(uint32_t otg, bool vblank) {
        WAIT(reg("OTG%u_OTG_STATUS", otg), vblank ? "OTG_V_BLANK" : "OTG_V_ACTIVE_DISP", 1, 1, 100000);
    }
    void setup_vupdate_interrupt(const State& s, int master) { // dcn20_setup_vupdate_interrupt
        const auto& t = s.stream.timing;
        const int32_t vesa_sync_start = static_cast<int32_t>(t.height + vfp(t));
        const int32_t asic_blank_end = static_cast<int32_t>(t.v_total) - vesa_sync_start;
        int32_t start_line = asic_blank_end - static_cast<int32_t>(s.pipe[master].dlg_param.vstartup_start) + 1;
        if (start_line < 0) {
            start_line = 0;
        }
        SET(reg("OTG%u_OTG_VERTICAL_INTERRUPT2_POSITION", static_cast<uint32_t>(s.pipe[master].tg)), 0,
            {{"OTG_VERTICAL_INTERRUPT2_LINE_START", static_cast<uint32_t>(start_line)}});
    }
    void setup_stereo(const PipeCtx& p) { // dcn10_setup_stereo, no 3D
        // DEVIATION: dc_set_generic_gpio_for_stereo (GPIO service) is not driven.
        const uint32_t opp = static_cast<uint32_t>(p.opp);
        const uint32_t tg = static_cast<uint32_t>(p.tg);
        UPDATE(reg("FMT%u_FMT_CONTROL", opp), {{"FMT_STEREOSYNC_OVERRIDE", 0}});
        UPDATE(reg("OPPBUF%u_OPPBUF_CONTROL", opp), {{"OPPBUF_ACTIVE_WIDTH", 0}});
        UPDATE(reg("OPPBUF%u_OPPBUF_3D_PARAMETERS_0", opp), {{"OPPBUF_3D_VACT_SPACE1_SIZE", 0}});
        SET(reg("OTG%u_OTG_STEREO_CONTROL", tg), 0, {{"OTG_STEREO_EN", 0}});
        SET(reg("OTG%u_OTG_3D_STRUCTURE_CONTROL", tg), 0,
            {{"OTG_3D_STRUCTURE_EN", 0}, {"OTG_3D_STRUCTURE_STEREO_SEL_OVR", 0}});
    }

    // ---------------- OPP ----------------
    void set_disp_pattern_generator(uint32_t opp, bool solid_black, uint32_t width, uint32_t height,
                                    uint32_t offset) { // opp2_set_disp_pattern_generator
        SET(reg("DPG%u_DPG_DIMENSIONS", opp), 0, {{"DPG_ACTIVE_WIDTH", width}, {"DPG_ACTIVE_HEIGHT", height}});
        SET(reg("DPG%u_DPG_OFFSET_SEGMENT", opp), 0, {{"DPG_X_OFFSET", offset}, {"DPG_SEGMENT_WIDTH", 0}});
        if (solid_black) { // CONTROLLER_DP_TEST_PATTERN_SOLID_COLOR with black (0,0,0)
            SET(reg("DPG%u_DPG_COLOUR_B_CB", opp), 0, {{"DPG_COLOUR1_B_CB", 0}, {"DPG_COLOUR0_B_CB", 0}});
            SET(reg("DPG%u_DPG_COLOUR_G_Y", opp), 0, {{"DPG_COLOUR1_G_Y", 0}, {"DPG_COLOUR0_G_Y", 0}});
            SET(reg("DPG%u_DPG_COLOUR_R_CR", opp), 0, {{"DPG_COLOUR1_R_CR", 0}, {"DPG_COLOUR0_R_CR", 0}});
            UPDATE(reg("DPG%u_DPG_CONTROL", opp), {{"DPG_EN", 1}, {"DPG_MODE", 4}});
            SET(reg("DPG%u_DPG_DIMENSIONS", opp), 0, {{"DPG_ACTIVE_WIDTH", width}, {"DPG_ACTIVE_HEIGHT", height}});
        } else { // CONTROLLER_DP_TEST_PATTERN_VIDEOMODE
            WRITE(reg("DPG%u_DPG_CONTROL", opp), 0);
            WRITE(reg("DPG%u_DPG_COLOUR_R_CR", opp), 0);
            WRITE(reg("DPG%u_DPG_COLOUR_G_Y", opp), 0);
            WRITE(reg("DPG%u_DPG_COLOUR_B_CB", opp), 0);
            WRITE(reg("DPG%u_DPG_RAMP_CONTROL", opp), 0);
        }
    }
    void blank_pixel_data(const State& s, int pipe, bool blank) { // dcn20_blank_pixel_data
        int odm_pipe = pipe;
        while (s.pipe[odm_pipe].next_odm >= 0) {
            uint32_t x = 0, width = 0;
            odm_slice_src(s, odm_pipe, x, width);
            set_disp_pattern_generator(static_cast<uint32_t>(s.pipe[odm_pipe].opp), blank, width,
                                       s.stream.timing.height, x);
            odm_pipe = s.pipe[odm_pipe].next_odm;
        }
        uint32_t x = 0, width = 0;
        odm_slice_src(s, odm_pipe, x, width);
        set_disp_pattern_generator(static_cast<uint32_t>(s.pipe[odm_pipe].opp), blank, width, s.stream.timing.height, x);
    }
    bool wait_for_blank_complete(uint32_t opp) { // dcn20_wait_for_blank_complete
        uint32_t counter = 0;
        for (; counter < 1000; ++counter) {
            const uint32_t en = GET(reg("DPG%u_DPG_CONTROL", opp), "DPG_EN");
            const uint32_t pending = GET(reg("DPG%u_DPG_STATUS", opp), "DPG_DOUBLE_BUFFER_PENDING");
            if (!(en == 1 && pending == 1)) {
                break;
            }
            udelay(100);
        }
        if (counter == 1000) {
            return false;
        }
        const uint32_t en = GET(reg("DPG%u_DPG_CONTROL", opp), "DPG_EN"); // REG_GET_2: one read
        const uint32_t pending = GET(reg("DPG%u_DPG_STATUS", opp), "DPG_DOUBLE_BUFFER_PENDING");
        return en == 1 && pending == 0;
    }
    void program_fmt(uint32_t opp) { // opp1_program_fmt
        // resource_build_bit_depth_reduction_params: DITHER_OPTION_SPATIAL8, RGB.
        const Reg b = reg("FMT%u_FMT_BIT_DEPTH_CONTROL", opp);
        const Reg c = reg("FMT%u_FMT_CONTROL", opp);
        UPDATE(b, {{"FMT_TRUNCATE_EN", 0}, {"FMT_TRUNCATE_DEPTH", 0}, {"FMT_TRUNCATE_MODE", 0}});
        UPDATE(b, {{"FMT_SPATIAL_DITHER_EN", 0}, {"FMT_SPATIAL_DITHER_MODE", 0}, {"FMT_SPATIAL_DITHER_DEPTH", 0},
                   {"FMT_TEMPORAL_DITHER_EN", 0}, {"FMT_HIGHPASS_RANDOM_ENABLE", 0}, {"FMT_FRAME_RANDOM_ENABLE", 0},
                   {"FMT_RGB_RANDOM_ENABLE", 0}});
        UPDATE(c, {{"FMT_SPATIAL_DITHER_FRAME_COUNTER_MAX", 0}, {"FMT_SPATIAL_DITHER_FRAME_COUNTER_BIT_SWAP", 0}});
        SET(reg("FMT%u_FMT_DITHER_RAND_R_SEED", opp), 0, {{"FMT_RAND_R_SEED", 0}});
        SET(reg("FMT%u_FMT_DITHER_RAND_G_SEED", opp), 0, {{"FMT_RAND_G_SEED", 0}});
        SET(reg("FMT%u_FMT_DITHER_RAND_B_SEED", opp), 0, {{"FMT_RAND_B_SEED", 0}});
        UPDATE(b, {{"FMT_SPATIAL_DITHER_EN", 1}, {"FMT_SPATIAL_DITHER_MODE", 0}, {"FMT_SPATIAL_DITHER_DEPTH", 1},
                   {"FMT_HIGHPASS_RANDOM_ENABLE", 1}, {"FMT_FRAME_RANDOM_ENABLE", 0}, {"FMT_RGB_RANDOM_ENABLE", 1}});
        // opp1_set_clamping (CLAMPING_FULL_RANGE), opp1_set_pixel_encoding (RGB)
        UPDATE(reg("FMT%u_FMT_CLAMP_CNTL", opp), {{"FMT_CLAMP_DATA_EN", 0}, {"FMT_CLAMP_COLOR_FORMAT", 0}});
        UPDATE(reg("FMT%u_FMT_CLAMP_CNTL", opp), {{"FMT_CLAMP_DATA_EN", 1}, {"FMT_CLAMP_COLOR_FORMAT", 0}});
        UPDATE(c, {{"FMT_PIXEL_ENCODING", 0}, {"FMT_SUBSAMPLING_MODE", 0}, {"FMT_CBCR_BIT_REDUCTION_BYPASS", 0}});
        UPDATE(c, {{"FMT_PIXEL_ENCODING", 0}});
    }
    void set_dyn_expansion(uint32_t opp, rx7900::display::StreamSignal signal) { // opp1_set_dyn_expansion
        UPDATE(reg("FMT%u_FMT_DYNAMIC_EXP_CNTL", opp), {{"FMT_DYNAMIC_EXP_EN", 0}, {"FMT_DYNAMIC_EXP_MODE", 0}});
        // HDMI type A / DP at COLOR_DEPTH_888: 8 -> 12 bit expansion; DVI none.
        if (signal != rx7900::display::StreamSignal::dvi_single) {
            UPDATE(reg("FMT%u_FMT_DYNAMIC_EXP_CNTL", opp), {{"FMT_DYNAMIC_EXP_EN", 1}, {"FMT_DYNAMIC_EXP_MODE", 1}});
        }
    }

    // ---------------- MPC (dcn10_mpc.c, dcn20_mpc.c) ----------------
    int get_mpcc_for_dpp(int opp, int dpp_id) const { // mpc1_get_mpcc_for_dpp
        int tmp = opp_list_[opp];
        while (tmp >= 0) {
            if (mpcc_[tmp].dpp_id == dpp_id) {
                return tmp;
            }
            if (tmp == mpcc_[tmp].bot) {
                break;
            }
            tmp = mpcc_[tmp].bot;
        }
        return -1;
    }
    void remove_mpcc(int opp, int mpcc_id) { // mpc1_remove_mpcc
        bool found = false;
        if (opp_list_[opp] == mpcc_id) {
            found = true;
            if (mpcc_[mpcc_id].bot >= 0) {
                opp_list_[opp] = mpcc_[mpcc_id].bot;
                UPDATE(reg("MPC_OUT%u_MUX", static_cast<uint32_t>(opp)), {{"MPC_OUT_MUX", static_cast<uint32_t>(opp_list_[opp])}});
            } else {
                opp_list_[opp] = -1;
                UPDATE(reg("MPC_OUT%u_MUX", static_cast<uint32_t>(opp)), {{"MPC_OUT_MUX", 0xf}});
            }
        } else {
            int tmp = opp_list_[opp];
            while (tmp >= 0 && mpcc_[tmp].bot != mpcc_id) {
                tmp = mpcc_[tmp].bot;
            }
            if (tmp >= 0 && mpcc_[tmp].bot == mpcc_id) {
                found = true;
                mpcc_[tmp].bot = mpcc_[mpcc_id].bot;
                if (mpcc_[mpcc_id].bot >= 0) {
                    SET(reg("MPCC%u_MPCC_BOT_SEL", static_cast<uint32_t>(tmp)), 0,
                        {{"MPCC_BOT_SEL", static_cast<uint32_t>(mpcc_[mpcc_id].bot)}});
                } else {
                    SET(reg("MPCC%u_MPCC_BOT_SEL", static_cast<uint32_t>(tmp)), 0, {{"MPCC_BOT_SEL", 0xf}});
                    UPDATE(reg("MPCC%u_MPCC_CONTROL", static_cast<uint32_t>(tmp)),
                           {{"MPCC_MODE", 1}}); // MPCC_BLEND_MODE_TOP_LAYER_PASSTHROUGH
                }
            }
        }
        const uint32_t id = static_cast<uint32_t>(mpcc_id);
        SET(reg("MPCC%u_MPCC_TOP_SEL", id), 0, {{"MPCC_TOP_SEL", 0xf}});
        SET(reg("MPCC%u_MPCC_BOT_SEL", id), 0, {{"MPCC_BOT_SEL", 0xf}});
        SET(reg("MPCC%u_MPCC_OPP_ID", id), 0, {{"MPCC_OPP_ID", 0xf}});
        SET(reg("MPCC%u_MPCC_UPDATE_LOCK_SEL", id), 0, {{"MPCC_UPDATE_LOCK_SEL", 0xf}});
        if (found) {
            mpcc_[mpcc_id].dpp_id = 0xf;
            mpcc_[mpcc_id].bot = -1;
        }
    }
    // mpc2_update_blending + dcn10_update_visual_confirm_color -> mpc1_set_bg_color(black)
    void update_blending(const State& s, int pipe) {
        const auto& pl = plane_of(s, pipe);
        uint32_t alpha_mode = 2, pre_multiplied = 0, global_gain = 0xff; // MPCC_ALPHA_BLEND_MODE_GLOBAL_ALPHA
        if (pl.per_pixel_alpha) {
            pre_multiplied = pl.pre_multiplied_alpha ? 1u : 0u;
            if (pl.global_alpha) {
                alpha_mode = 1; // PER_PIXEL_ALPHA_COMBINED_GLOBAL_GAIN
                global_gain = pl.global_alpha_value;
            } else {
                alpha_mode = 0; // PER_PIXEL_ALPHA
            }
        }
        const uint32_t global_alpha = pl.global_alpha ? pl.global_alpha_value : 0xffu;
        const uint32_t m = static_cast<uint32_t>(pipe);
        UPDATE(reg("MPCC%u_MPCC_CONTROL", m),
               {{"MPCC_ALPHA_BLND_MODE", alpha_mode}, {"MPCC_ALPHA_MULTIPLIED_MODE", pre_multiplied},
                {"MPCC_BLND_ACTIVE_OVERLAP_ONLY", 0}, {"MPCC_GLOBAL_ALPHA", global_alpha},
                {"MPCC_GLOBAL_GAIN", global_gain}, {"MPCC_BG_BPC", 4}, {"MPCC_BOT_GAIN_MODE", 0}});
        SET(reg("MPCC%u_MPCC_TOP_GAIN", m), 0, {{"MPCC_TOP_GAIN", 0x1f000}});
        SET(reg("MPCC%u_MPCC_BOT_GAIN_INSIDE", m), 0, {{"MPCC_BOT_GAIN_INSIDE", 0x1f000}});
        SET(reg("MPCC%u_MPCC_BOT_GAIN_OUTSIDE", m), 0, {{"MPCC_BOT_GAIN_OUTSIDE", 0x1f000}});
        int bottommost = pipe;
        while (mpcc_[bottommost].bot >= 0 && mpcc_[bottommost].bot != bottommost) {
            bottommost = mpcc_[bottommost].bot;
        }
        const uint32_t b = static_cast<uint32_t>(bottommost);
        SET(reg("MPCC%u_MPCC_BG_R_CR", b), 0, {{"MPCC_BG_R_CR", 0}});
        SET(reg("MPCC%u_MPCC_BG_G_Y", b), 0, {{"MPCC_BG_G_Y", 0}});
        SET(reg("MPCC%u_MPCC_BG_B_CB", b), 0, {{"MPCC_BG_B_CB", 0}});
    }
    // mpc1_insert_plane(tree of opp, blnd_cfg, sm_cfg NULL, insert_above_mpcc NULL, dpp_id, mpcc_id)
    void insert_plane(const State& s, int pipe, int opp) {
        const uint32_t m = static_cast<uint32_t>(pipe);
        mpcc_[pipe].dpp_id = pipe;
        mpcc_[pipe].bot = -1;
        SET(reg("MPCC%u_MPCC_BOT_SEL", m), 0, {{"MPCC_BOT_SEL", 0xf}});
        UPDATE(reg("MPCC%u_MPCC_CONTROL", m), {{"MPCC_MODE", 2}}); // MPCC_BLEND_MODE_TOP_LAYER_ONLY
        SET(reg("MPCC%u_MPCC_TOP_SEL", m), 0, {{"MPCC_TOP_SEL", m}});
        SET(reg("MPCC%u_MPCC_OPP_ID", m), 0, {{"MPCC_OPP_ID", static_cast<uint32_t>(opp)}});
        SET(reg("MPCC%u_MPCC_UPDATE_LOCK_SEL", m), 0, {{"MPCC_UPDATE_LOCK_SEL", static_cast<uint32_t>(opp)}});
        if (opp_list_[opp] < 0) { // tree->opp_list == insert_above_mpcc (NULL): the toppest
            opp_list_[opp] = pipe;
            UPDATE(reg("MPC_OUT%u_MUX", static_cast<uint32_t>(opp)), {{"MPC_OUT_MUX", m}});
        } else {
            int tmp = opp_list_[opp];
            while (tmp >= 0 && mpcc_[tmp].bot != -1) {
                tmp = mpcc_[tmp].bot;
            }
            SET(reg("MPCC%u_MPCC_BOT_SEL", static_cast<uint32_t>(tmp)), 0, {{"MPCC_BOT_SEL", m}});
            mpcc_[tmp].bot = pipe;
            UPDATE(reg("MPCC%u_MPCC_CONTROL", static_cast<uint32_t>(tmp)), {{"MPCC_MODE", 3}}); // TOP_BOT_BLENDING
        }
        update_blending(s, pipe);
    }
    void update_mpcc(const State& s, int pipe, bool full_update) { // dcn20_update_mpcc
        if (!full_update && !s.pipe[pipe].update_flags.mpcc) {
            update_blending(s, pipe);
            return;
        }
        const int existing = get_mpcc_for_dpp(s.pipe[pipe].opp, pipe);
        if (existing >= 0) {
            remove_mpcc(s.pipe[pipe].opp, existing);
        }
        insert_plane(s, pipe, s.pipe[pipe].opp);
    }

    // ---------------- plane disconnect / disable (dcn10/20 hwseq, hubp2) ----------------
    void plane_atomic_disconnect(const PipeCtx& p, int pipe_idx) { // dcn10_plane_atomic_disconnect
        const int mpcc_to_remove = get_mpcc_for_dpp(p.opp, pipe_idx);
        if (mpcc_to_remove < 0) {
            return; // already reset
        }
        remove_mpcc(p.opp, mpcc_to_remove);
        disconnect_pending_[p.opp][pipe_idx] = true;
    }
    void set_blank(uint32_t hubp, bool blank) { // hubp2_set_blank_regs
        if (blank) {
            const uint32_t v = READ(reg("HUBP%u_DCHUBP_CNTL", hubp));
            if (v != 0) {
                WAIT(reg("HUBP%u_DCHUBP_CNTL", hubp), "HUBP_NO_OUTSTANDING_REQ", 1, 1, 100000);
            }
        }
        UPDATE(reg("HUBP%u_DCHUBP_CNTL", hubp), {{"HUBP_BLANK_EN", blank ? 1u : 0u}, {"HUBP_TTU_DISABLE", 0}});
    }
    void wait_for_mpcc_disconnect(const PipeCtx& p) { // dcn10_wait_for_mpcc_disconnect
        if (p.opp < 0) {
            return;
        }
        for (uint32_t mpcc = 0; mpcc < 4; ++mpcc) {
            if (!disconnect_pending_[p.opp][mpcc]) {
                continue;
            }
            if (p.tg >= 0 && is_tg_enabled(static_cast<uint32_t>(p.tg))) { // mpc2_assert_idle_mpcc
                if (GET(reg("MPCC%u_MPCC_STATUS", mpcc), "MPCC_DISABLED") == 0) {
                    WAIT(reg("MPCC%u_MPCC_STATUS", mpcc), "MPCC_IDLE", 1, 1, 100000);
                }
            }
            disconnect_pending_[p.opp][mpcc] = false;
            set_blank(mpcc, true);
        }
    }
    void plane_atomic_disable(PipeCtx& p, uint32_t pipe) { // dcn20_plane_atomic_disable
        wait_for_mpcc_disconnect(p);
        UPDATE(reg("HUBP%u_DCHUBP_MALL_CONFIG", pipe), {{"USE_MALL_SEL", 0}, {"USE_MALL_FOR_CURSOR", 0}});
        UPDATE(reg("HUBPREQ%u_DCSURF_FLIP_CONTROL2", pipe), {{"SURFACE_GSL_ENABLE", 0}});
        UPDATE(reg("HUBP%u_HUBP_CLK_CNTL", pipe), {{"HUBP_CLOCK_ENABLE", 0}});
        UPDATE(reg("DPP_TOP%u_DPP_CONTROL", pipe), {{"DPP_CLOCK_ENABLE", 0}});
        power_gated_[pipe] = true;
        // dcn10_plane_atomic_power_down (dcn32: no dpp_pg_control, no dpp root clock)
        SET(plain("DC_IP_REQUEST_CNTL"), 0, {{"IP_REQUEST_EN", 1}});
        hubp_pg_control(pipe, false);
        scl_[pipe] = Scl{}; // dpp_reset
        SET(plain("DC_IP_REQUEST_CNTL"), 0, {{"IP_REQUEST_EN", 0}});
        p = PipeCtx{}; // pipe_ctx->stream = NULL, stream_res/plane_res memset, links NULL
    }
    void update_force_pstate(const State& context) { // dcn32_update_force_pstate (no SubVP/FPO)
        for (uint32_t i = 0; i < 4; ++i) {
            if (!context.pipe[i].hubp) {
                continue;
            }
            UPDATE(reg("HUBPREQ%u_UCLK_PSTATE_FORCE", i), {{"DATA_UCLK_PSTATE_FORCE_EN", 0},
                                                           {"DATA_UCLK_PSTATE_FORCE_VALUE", 0}});
            UPDATE(reg("HUBPREQ%u_UCLK_PSTATE_FORCE", i), {{"CURSOR_UCLK_PSTATE_FORCE_EN", 0},
                                                           {"CURSOR_UCLK_PSTATE_FORCE_VALUE", 0}});
        }
    }
    // dc_state_rem_all_planes_for_stream -> resource_remove_dpp_pipes_for_plane_composition
    static void remove_all_planes(State& s) {
        for (int i = 3; i >= 0; --i) {
            PipeCtx& p = s.pipe[i];
            if (p.plane < 0) {
                continue;
            }
            if (p.top >= 0) {
                s.pipe[p.top].bottom = p.bottom;
            }
            if (p.bottom >= 0 && p.top >= 0) {
                s.pipe[p.bottom].top = p.top;
            }
            if (p.top < 0) {
                p.plane = -1;
            } else {
                p = PipeCtx{};
            }
        }
    }
    // disable_dangling_plane for the old stream (planes removed, stream kept).
    void disable_dangling_plane() {
        State dangling = cur_;
        remove_all_planes(dangling);
        interdependent_update_lock(cur_, true);
        program_front_end_for_ctx(dangling, false);
        interdependent_update_lock(cur_, false);
        post_unlock_program_front_end(dangling);
        cur_ = dangling;
    }

    // ---------------- stream encoder / link (dcn10/20/30/32 enc, link_dpms.c) ----------------
    static bool is_dp(const Dcn32StreamPlan& p) { return p.signal == rx7900::display::StreamSignal::display_port; }
    static bool is_hdmi(const Dcn32StreamPlan& p) { return p.signal == rx7900::display::StreamSignal::hdmi; }


    void set_avmute(const Dcn32StreamPlan& p, bool enable) { // dcn30_set_avmute
        if (!is_hdmi(p)) {
            return;
        }
        UPDATE(reg("DIG%u_HDMI_GC", p.engine), {{"HDMI_GC_AVMUTE", enable ? 1u : 0u}});
        if (enable && is_tg_enabled(p.pipe)) {
            wait_for_state(p.pipe, false);
            wait_for_state(p.pipe, true);
            wait_for_state(p.pipe, false);
            wait_for_state(p.pipe, true);
            wait_for_state(p.pipe, false);
        }
    }
    void audio_mute(const Dcn32StreamPlan& p) { // disable_dio_audio_packet -> afmt3_audio_mute_control(true)
        UPDATE(reg("AFMT%u_AFMT_AUDIO_PACKET_CONTROL", p.engine), {{"AFMT_AUDIO_SAMPLE_SEND", 0}});
    }
    void dp_blank(const Dcn32StreamPlan& p) { // enc1_stream_encoder_dp_blank
        const Reg v = reg("DP%u_DP_VID_STREAM_CNTL", p.engine);
        if ((GET(v, "DP_VID_STREAM_ENABLE") & 1u) == 0) {
            return;
        }
        UPDATE(v, {{"DP_VID_STREAM_DIS_DEFER", 2}});
        UPDATE(v, {{"DP_VID_STREAM_ENABLE", 0}});
        WAIT(v, "DP_VID_STREAM_STATUS", 0, 10, 20 * 501);
        UPDATE(reg("DP%u_DP_STEER_FIFO", p.engine), {{"DP_STEER_FIFO_RESET", 1}});
    }
    void stop_dp_info_packets(const Dcn32StreamPlan& p) {
        const Reg s = reg("DP%u_DP_SEC_CNTL", p.engine);
        SET(s, 0, {{"DP_SEC_GSP0_ENABLE", 0}, {"DP_SEC_GSP1_ENABLE", 0}, {"DP_SEC_GSP2_ENABLE", 0},
                   {"DP_SEC_GSP3_ENABLE", 0}, {"DP_SEC_GSP4_ENABLE", 0}, {"DP_SEC_GSP5_ENABLE", 0},
                   {"DP_SEC_GSP6_ENABLE", 0}, {"DP_SEC_GSP7_ENABLE", 0}, {"DP_SEC_MPG_ENABLE", 0},
                   {"DP_SEC_STREAM_ENABLE", 0}});
        if (READ(s) != 0) {
            UPDATE(s, {{"DP_SEC_STREAM_ENABLE", 1}});
        }
    }
    void stop_hdmi_info_packets(const Dcn32StreamPlan& p) { // enc3_stream_encoder_stop_hdmi_info_packets
        const uint32_t e = p.engine;
        const Reg c0 = reg("DIG%u_HDMI_GENERIC_PACKET_CONTROL0", e);
        const Reg c6 = reg("DIG%u_HDMI_GENERIC_PACKET_CONTROL6", e);
        SET(c0, 0, {{"HDMI_GENERIC0_CONT", 0}, {"HDMI_GENERIC0_SEND", 0}, {"HDMI_GENERIC1_CONT", 0}, {"HDMI_GENERIC1_SEND", 0}});
        SET(reg("DIG%u_HDMI_GENERIC_PACKET_CONTROL1", e), 0, {{"HDMI_GENERIC0_LINE", 0}, {"HDMI_GENERIC1_LINE", 0}});
        SET(c0, 0, {{"HDMI_GENERIC2_CONT", 0}, {"HDMI_GENERIC2_SEND", 0}, {"HDMI_GENERIC3_CONT", 0}, {"HDMI_GENERIC3_SEND", 0}});
        SET(reg("DIG%u_HDMI_GENERIC_PACKET_CONTROL2", e), 0, {{"HDMI_GENERIC2_LINE", 0}, {"HDMI_GENERIC3_LINE", 0}});
        SET(c0, 0, {{"HDMI_GENERIC4_CONT", 0}, {"HDMI_GENERIC4_SEND", 0}, {"HDMI_GENERIC5_CONT", 0}, {"HDMI_GENERIC5_SEND", 0}});
        SET(reg("DIG%u_HDMI_GENERIC_PACKET_CONTROL3", e), 0, {{"HDMI_GENERIC4_LINE", 0}, {"HDMI_GENERIC5_LINE", 0}});
        SET(c0, 0, {{"HDMI_GENERIC6_CONT", 0}, {"HDMI_GENERIC6_SEND", 0}, {"HDMI_GENERIC7_CONT", 0}, {"HDMI_GENERIC7_SEND", 0}});
        SET(reg("DIG%u_HDMI_GENERIC_PACKET_CONTROL4", e), 0, {{"HDMI_GENERIC6_LINE", 0}, {"HDMI_GENERIC7_LINE", 0}});
        SET(c6, 0, {{"HDMI_GENERIC8_CONT", 0}, {"HDMI_GENERIC8_SEND", 0}, {"HDMI_GENERIC9_CONT", 0}, {"HDMI_GENERIC9_SEND", 0}});
        SET(reg("DIG%u_HDMI_GENERIC_PACKET_CONTROL7", e), 0, {{"HDMI_GENERIC8_LINE", 0}, {"HDMI_GENERIC9_LINE", 0}});
        SET(c6, 0, {{"HDMI_GENERIC10_CONT", 0}, {"HDMI_GENERIC10_SEND", 0}, {"HDMI_GENERIC11_CONT", 0}, {"HDMI_GENERIC11_SEND", 0}});
        SET(reg("DIG%u_HDMI_GENERIC_PACKET_CONTROL8", e), 0, {{"HDMI_GENERIC10_LINE", 0}, {"HDMI_GENERIC11_LINE", 0}});
        SET(c6, 0, {{"HDMI_GENERIC12_CONT", 0}, {"HDMI_GENERIC12_SEND", 0}, {"HDMI_GENERIC13_CONT", 0}, {"HDMI_GENERIC13_SEND", 0}});
        SET(reg("DIG%u_HDMI_GENERIC_PACKET_CONTROL9", e), 0, {{"HDMI_GENERIC12_LINE", 0}, {"HDMI_GENERIC13_LINE", 0}});
        SET(c6, 0, {{"HDMI_GENERIC14_CONT", 0}, {"HDMI_GENERIC14_SEND", 0}});
        UPDATE(reg("DIG%u_HDMI_GENERIC_PACKET_CONTROL10", e), {{"HDMI_GENERIC14_LINE", 0}});
    }
    void disable_stream(const Dcn32StreamPlan& p) { // dce110_disable_stream
        if (is_hdmi(p)) {
            stop_hdmi_info_packets(p);
            UPDATE(reg("DIG%u_HDMI_CONTROL", p.engine), {{"HDMI_PACKET_GEN_VERSION", 1}, {"HDMI_KEEPOUT_MODE", 1},
                                                         {"HDMI_DEEP_COLOR_ENABLE", 0}, {"HDMI_DATA_SCRAMBLE_EN", 0},
                                                         {"HDMI_CLOCK_CHANNEL_RATE", 0}});
        }
        if (is_dp(p)) {
            stop_dp_info_packets(p);
        }
        audio_mute(p);
        // reset_dio_stream_encoder: set_input_mode(0); DEVIATION: the FE->BE
        // disconnect is the last step of the link owner's disable_link.
        UPDATE(reg("DIG%u_DIG_FIFO_CTRL0", p.engine), {{"DIG_FIFO_OUTPUT_PIXEL_MODE", 0}});
    }
    void link_set_dpms_off(const State& s, int master) {
        const Dcn32StreamPlan& p = s.stream;
        set_avmute(p, true);
        audio_mute(p);
        // DEVIATION: update_psp_stream_config (HDCP) is not called.
        if (is_dp(p)) { // dce110_blank_stream
            dp_blank(p);
            msleep(60);
        }
        if (is_hdmi(p)) { // write_scdc_data(link->ddc, 165000, false)
            hooks_.record_scdc_legacy(p);
        }
        if (is_dp(p)) {
            hooks_.record_disable(p);
            disable_stream(p);
        } else {
            disable_stream(p);
            hooks_.record_disable(p);
        }
        if (p.dsc && is_dp(p)) { // link_set_dsc_enable(false)
            dp_set_dsc_on_rx(false);
            link_set_dsc_on_stream(s, master, false);
        }
    }

    void vpg_update(uint32_t vpg, uint32_t index, const rx7900::display::InfoPacket& packet) {
        WAIT(reg("VPG%u_VPG_GENERIC_STATUS", vpg), "VPG_GENERIC_CONFLICT_OCCURED", 0, 10, 50);
        UPDATE(reg("VPG%u_VPG_GENERIC_STATUS", vpg), {{"VPG_GENERIC_CONFLICT_CLR", 1}});
        UPDATE(reg("VPG%u_VPG_GENERIC_PACKET_ACCESS_CTRL", vpg), {{"VPG_GENERIC_DATA_INDEX", index * 9}});
        SET(reg("VPG%u_VPG_GENERIC_PACKET_DATA", vpg), 0,
            {{"VPG_GENERIC_DATA_BYTE0", packet.hb0}, {"VPG_GENERIC_DATA_BYTE1", packet.hb1},
             {"VPG_GENERIC_DATA_BYTE2", packet.hb2}, {"VPG_GENERIC_DATA_BYTE3", packet.hb3}});
        for (uint32_t i = 0; i < 8; ++i) {
            uint32_t word = 0;
            std::memcpy(&word, &packet.sb[i * 4], 4); // the little-endian uint32_t view Linux writes
            WRITE(reg("VPG%u_VPG_GENERIC_PACKET_DATA", vpg), word);
        }
        char field_name[48];
        std::snprintf(field_name, sizeof(field_name), "VPG_GENERIC%u_IMMEDIATE_UPDATE", index);
        UPDATE(reg("VPG%u_VPG_GSP_IMMEDIATE_UPDATE_CTRL", vpg), {{field_name, 1}});
    }
    void update_hdmi_info_packet(const Dcn32StreamPlan& p, uint32_t index, const rx7900::display::InfoPacket& packet) {
        static const char* const control[15] = {"0", "0", "0", "0", "0", "0", "0", "0", "6", "6", "6", "6", "6", "6", "6"};
        static const char* const line_reg[15] = {"1", "1", "2", "2", "3", "3", "4", "4", "7", "7", "8", "8", "9", "9", "10"};
        uint32_t cont = 0, send = 0, line = 0;
        if (packet.valid) {
            vpg_update(p.engine, index, packet);
            cont = 1;
            send = 1;
            line = 2;
        }
        char reg_name[64];
        char cont_name[32];
        char send_name[32];
        char line_name[32];
        std::snprintf(reg_name, sizeof(reg_name), "DIG%%u_HDMI_GENERIC_PACKET_CONTROL%s", control[index]);
        std::snprintf(cont_name, sizeof(cont_name), "HDMI_GENERIC%u_CONT", index);
        std::snprintf(send_name, sizeof(send_name), "HDMI_GENERIC%u_SEND", index);
        UPDATE(reg(reg_name, p.engine), {{cont_name, cont}, {send_name, send}});
        std::snprintf(reg_name, sizeof(reg_name), "DIG%%u_HDMI_GENERIC_PACKET_CONTROL%s", line_reg[index]);
        std::snprintf(line_name, sizeof(line_name), "HDMI_GENERIC%u_LINE", index);
        UPDATE(reg(reg_name, p.engine), {{line_name, line}});
    }
    void update_info_frame(const Dcn32StreamPlan& p) { // dcn31_update_info_frame
        if (is_hdmi(p)) { // enc3_stream_encoder_update_hdmi_info_packets
            UPDATE(reg("DIG%u_HDMI_DB_CONTROL", p.engine), {{"HDMI_DB_DISABLE", 1}});
            UPDATE(reg("DIG%u_AFMT_CNTL", p.engine), {{"AFMT_AUDIO_CLOCK_EN", 1}});
            const rx7900::display::InfoPacket none{};
            update_hdmi_info_packet(p, 0, p.avi);
            update_hdmi_info_packet(p, 5, none); // hfvsif
            update_hdmi_info_packet(p, 2, none); // gamut
            update_hdmi_info_packet(p, 1, p.vendor);
            update_hdmi_info_packet(p, 3, none); // spd
            update_hdmi_info_packet(p, 4, none); // hdrsmd
            update_hdmi_info_packet(p, 6, none); // vtem
        } else if (is_dp(p)) { // sdp_line_num (no adaptive sync), enc3_stream_encoder_update_dp_info_packets
            if (p.vsc.valid) {
                vpg_update(p.engine, 0, p.vsc);
                vpg_update(p.engine, 1, p.vsc);
                vpg_update(p.engine, 1, p.vsc);
            }
            const Reg s = reg("DP%u_DP_SEC_CNTL", p.engine);
            UPDATE(s, {{"DP_SEC_GSP0_ENABLE", p.vsc.valid ? 1u : 0u}});
            UPDATE(s, {{"DP_SEC_GSP2_ENABLE", 0}});
            UPDATE(s, {{"DP_SEC_GSP3_ENABLE", 0}});
            UPDATE(s, {{"DP_SEC_GSP5_ENABLE", 0}});
            if (READ(s) != 0) {
                UPDATE(s, {{"DP_SEC_STREAM_ENABLE", 1}});
            }
            if (GET(reg("DP%u_DP_SEC_METADATA_TRANSMISSION", p.engine), "DP_SEC_METADATA_PACKET_ENABLE") != 0) {
                UPDATE(s, {{"DP_SEC_STREAM_ENABLE", 1}});
            }
        }
    }
    void dp_set_stream_attribute(const Dcn32StreamPlan& p) { // enc2 -> enc1_stream_encoder_dp_set_stream_attribute
        const auto& t = p.timing;
        const uint32_t e = p.engine;
        uint32_t misc1 = READ(reg("DP%u_DP_MSA_MISC", e));
        misc1 = p.dp_use_vsc ? (misc1 | 0x40u) : (misc1 & ~0x40u);
        UPDATE(reg("DP%u_DP_PIXEL_FORMAT", e), {{"DP_PIXEL_ENCODING", 0}, {"DP_COMPONENT_DEPTH", 1}});
        uint32_t misc0 = 1u << 5; // colorimetry_bpc for COLOR_DEPTH_888
        misc1 &= ~0x80u;          // COLOR_SPACE_SRGB
        SET(reg("DP%u_DP_MSA_COLORIMETRY", e), 0, {{"DP_MSA_MISC0", misc0}});
        WRITE(reg("DP%u_DP_MSA_MISC", e), misc1);
        SET(reg("DP%u_DP_MSA_TIMING_PARAM1", e), 0, {{"DP_MSA_HTOTAL", t.h_total}, {"DP_MSA_VTOTAL", t.v_total}});
        const uint32_t h_blank = t.h_total - t.width;
        const uint32_t h_back_porch = h_blank - t.h_front_porch - t.h_sync_width;
        SET(reg("DP%u_DP_MSA_TIMING_PARAM2", e), 0,
            {{"DP_MSA_HSTART", t.h_sync_width + h_back_porch}, {"DP_MSA_VSTART", t.v_total - t.height - t.v_front_porch}});
        SET(reg("DP%u_DP_MSA_TIMING_PARAM3", e), 0,
            {{"DP_MSA_HSYNCWIDTH", t.h_sync_width},
             {"DP_MSA_HSYNCPOLARITY", (t.flags & rx7900::display::positive_hsync) ? 0u : 1u},
             {"DP_MSA_VSYNCWIDTH", t.v_sync_width},
             {"DP_MSA_VSYNCPOLARITY", (t.flags & rx7900::display::positive_vsync) ? 0u : 1u}});
        SET(reg("DP%u_DP_MSA_TIMING_PARAM4", e), 0, {{"DP_MSA_HWIDTH", t.width}, {"DP_MSA_VHEIGHT", t.height}});
        UPDATE(reg("DP%u_DP_SEC_FRAMING4", e), {{"DP_SST_SDP_SPLITTING", 0}});
    }
    void tmds_attribute_common(const Dcn32StreamPlan& p) {
        // DEVIATION: avoid_vbios_exec_table path (no VBIOS DIGxEncoderControl SETUP).
        UPDATE(reg("DIG%u_DIG_CLOCK_PATTERN", p.engine), {{"DIG_CLOCK_PATTERN", 0x1f}});
        // enc1_stream_encoder_set_stream_attribute_helper (RGB)
        UPDATE(reg("DIG%u_DIG_FE_CNTL", p.engine), {{"TMDS_PIXEL_ENCODING", 0}});
        UPDATE(reg("DIG%u_DIG_FE_CNTL", p.engine), {{"TMDS_COLOR_FORMAT", 0}});
    }
    void hdmi_set_stream_attribute(const Dcn32StreamPlan& p) { // enc32_stream_encoder_hdmi_set_stream_attribute
        const uint32_t e = p.engine;
        tmds_attribute_common(p);
        const Reg c = reg("DIG%u_HDMI_CONTROL", e);
        UPDATE(c, {{"HDMI_PACKET_GEN_VERSION", 1}, {"HDMI_KEEPOUT_MODE", 1}, {"HDMI_DEEP_COLOR_ENABLE", 0},
                   {"HDMI_DATA_SCRAMBLE_EN", 0}, {"HDMI_NO_EXTRA_NULL_PACKET_FILLED", 1}, {"HDMI_CLOCK_CHANNEL_RATE", 0}});
        UPDATE(c, {{"HDMI_DEEP_COLOR_DEPTH", 0}});
        if (p.tmds_khz >= 340000) {
            UPDATE(c, {{"HDMI_DATA_SCRAMBLE_EN", 1}, {"HDMI_CLOCK_CHANNEL_RATE", 1}});
        } else if (p.lte_340_scramble) {
            UPDATE(c, {{"HDMI_DATA_SCRAMBLE_EN", 1}, {"HDMI_CLOCK_CHANNEL_RATE", 0}});
        }
        UPDATE(reg("DIG%u_HDMI_VBI_PACKET_CONTROL", e), {{"HDMI_GC_CONT", 1}, {"HDMI_GC_SEND", 1}, {"HDMI_NULL_SEND", 1}});
        UPDATE(reg("DIG%u_HDMI_VBI_PACKET_CONTROL", e), {{"HDMI_ACP_SEND", 0}});
        UPDATE(reg("DIG%u_HDMI_INFOFRAME_CONTROL0", e), {{"HDMI_AUDIO_INFO_SEND", 1}});
        UPDATE(reg("AFMT%u_AFMT_INFOFRAME_CONTROL0", e), {{"AFMT_AUDIO_INFO_UPDATE", 1}});
        UPDATE(reg("DIG%u_HDMI_INFOFRAME_CONTROL1", e), {{"HDMI_AUDIO_INFO_LINE", 2}});
        UPDATE(reg("DIG%u_HDMI_GC", e), {{"HDMI_GC_AVMUTE", 0}});
    }
    // dcn32_unblank_stream -> enc32_dp_set_odm_combine + dp_unblank
    void dp_unblank(const Dcn32StreamPlan& p, uint32_t pixel_khz, uint32_t ppc) {
        const uint32_t e = p.engine;
        UPDATE(reg("DP%u_DP_PIXEL_FORMAT", e), {{"DP_PIXEL_PER_CYCLE_PROCESSING_MODE", ppc > 1 ? 1u : 0u}});
        const uint32_t n_vid = 0x8000;
        const uint32_t n_multiply = ppc > 1 ? 1u : 0u;
        const uint32_t m_vid = static_cast<uint32_t>(static_cast<uint64_t>(n_vid) * pixel_khz /
                                                     (static_cast<uint64_t>(p.dp_rate_code) * 27000u));
        UPDATE(reg("DP%u_DP_VID_TIMING", e), {{"DP_VID_M_N_GEN_EN", 0}});
        UPDATE(reg("DP%u_DP_VID_N", e), {{"DP_VID_N", n_vid}});
        UPDATE(reg("DP%u_DP_VID_M", e), {{"DP_VID_M", m_vid}});
        UPDATE(reg("DP%u_DP_VID_TIMING", e), {{"DP_VID_M_N_GEN_EN", 1}, {"DP_VID_N_MUL", n_multiply}});
        UPDATE(reg("DP%u_DP_PIXEL_FORMAT", e), {{"DP_PIXEL_PER_CYCLE_PROCESSING_MODE", n_multiply}});
        UPDATE(reg("DP%u_DP_VID_STREAM_CNTL", e), {{"DP_VID_STREAM_ENABLE", 0}});
        WAIT(reg("DP%u_DP_VID_STREAM_CNTL", e), "DP_VID_STREAM_STATUS", 0, 10, 5000);
        UPDATE(reg("DP%u_DP_STEER_FIFO", e), {{"DP_STEER_FIFO_RESET", 1}});
        udelay(10);
        UPDATE(reg("DP%u_DP_STEER_FIFO", e), {{"DP_STEER_FIFO_RESET", 0}});
        WAIT(reg("DIG%u_DIG_FE_CNTL", e), "DIG_SYMCLK_FE_ON", 1, 10, 5000);
        const Reg f = reg("DIG%u_DIG_FIFO_CTRL0", e);
        UPDATE(f, {{"DIG_FIFO_READ_START_LEVEL", 7}});
        UPDATE(f, {{"DIG_FIFO_RESET", 1}});
        WAIT(f, "DIG_FIFO_RESET_DONE", 1, 10, 5000);
        UPDATE(f, {{"DIG_FIFO_RESET", 0}});
        WAIT(f, "DIG_FIFO_RESET_DONE", 0, 10, 5000);
        UPDATE(f, {{"DIG_FIFO_ENABLE", 1}});
        udelay(100);
        UPDATE(reg("DP%u_DP_VID_STREAM_CNTL", e), {{"DP_VID_STREAM_ENABLE", 1}});
    }
    void reset_fifo(uint32_t e, bool reset) { // enc32_reset_fifo
        UPDATE(reg("DIG%u_DIG_FIFO_CTRL0", e), {{"DIG_FIFO_RESET", reset ? 1u : 0u}});
        if (GET(reg("DIG%u_DIG_FE_CNTL", e), "DIG_SYMCLK_FE_ON") != 0) {
            WAIT(reg("DIG%u_DIG_FIFO_CTRL0", e), "DIG_FIFO_RESET_DONE", reset ? 1u : 0u, 10, 5000);
        } else {
            udelay(10);
        }
    }
    void link_set_dpms_on(const State& s, int master) {
        const Dcn32StreamPlan& p = s.stream;
        // DEVIATION: link_enc->setup runs inside the link owner's enable_link.
        UPDATE(reg("OTG%u_OTG_CONTROL", p.pipe), {{"OTG_OUT_MUX", 0}}); // optc3_set_out_mux(OUT_MUX_DIO)
        // setup_dio_stream_attribute: enc1_setup_stereo_sync(tg, false)
        UPDATE(reg("DIG%u_DIG_FE_CNTL", p.engine), {{"DIG_STEREOSYNC_SELECT", p.pipe}});
        UPDATE(reg("DIG%u_DIG_FE_CNTL", p.engine), {{"DIG_STEREOSYNC_GATE_EN", 1}});
        if (is_dp(p)) {
            dp_set_stream_attribute(p);
        } else if (is_hdmi(p)) {
            hdmi_set_stream_attribute(p);
        } else {
            tmds_attribute_common(p); // enc32_stream_encoder_dvi_set_stream_attribute
        }
        update_info_frame(p);
        if (p.dsc && is_dp(p)) {
            link_set_dsc_on_stream(s, master, true); // link_set_dsc_enable(true)
        }
        hooks_.record_enable(p); // enable_link
        // DEVIATION: the second link_enc->setup is omitted (the link owner set it up).
        // dcn20_enable_stream. DEVIATION: dccg32_set_pixel_rate_div here is a
        // no-op re-check (one OTG_PIXEL_RATE_DIV read) that is omitted.
        // setup_dio_stream_encoder (connect_dig_be_to_fe ran in enable_link):
        UPDATE(reg("DIG%u_DIG_FIFO_CTRL0", p.engine), {{"DIG_FIFO_OUTPUT_PIXEL_MODE", p.dio_pix_per_cycle == 2 ? 1u : 0u}});
        UPDATE(reg("DIG%u_DIG_FIFO_CTRL0", p.engine), {{"DIG_FIFO_READ_START_LEVEL", 7}});
        reset_fifo(p.engine, true);
        reset_fifo(p.engine, false);
        UPDATE(reg("DIG%u_DIG_FIFO_CTRL0", p.engine), {{"DIG_FIFO_ENABLE", 1}});
        // dcn30_program_dmdata_engine -> enc2_set_dynamic_metadata(false)
        UPDATE(reg("DME%u_DME_CONTROL", p.engine), {{"METADATA_ENGINE_EN", 0}});
        if (is_dp(p)) {
            UPDATE(reg("DP%u_DP_SEC_METADATA_TRANSMISSION", p.engine), {{"DP_SEC_METADATA_PACKET_ENABLE", 0}});
        } else {
            UPDATE(reg("DIG%u_HDMI_METADATA_PACKET_CONTROL", p.engine), {{"HDMI_METADATA_PACKET_ENABLE", 0}});
            UPDATE(reg("DIG%u_DIG_FE_CNTL", p.engine), {{"DOLBY_VISION_EN", 0}});
        }
        update_info_frame(p);
        if (p.dsc && is_dp(p)) {
            dp_set_dsc_on_rx(true);
            link_set_dsc_pps_packet(s, true);
        }
        if (is_dp(p)) {
            uint32_t pixel_khz = p.timing.pixel_clock_khz;
            uint32_t ppc = p.dio_pix_per_cycle;
            if (s.pipe[master].next_odm >= 0) { // params.opp_cnt > 1
                pixel_khz = (p.timing.pixel_clock_khz * 10u / 2u) / 10u;
                ppc = 2;
            }
            dp_unblank(p, pixel_khz, ppc);
            // enable_stream_features: DP_DOWNSPREAD_CTRL IGNORE_MSA_TIMING_PARAM
            uint8_t old = 0;
            Hooks::dpcd_read(&hooks_, 0x107, old);
            const uint8_t updated = static_cast<uint8_t>((old & 0x7fu) | (p.dp_ignore_msa ? 0x80u : 0u));
            if (updated != old) {
                Hooks::dpcd_write(&hooks_, 0x107, updated);
            }
        }
        // DEVIATION: update_psp_stream_config not called; no audio endpoint.
        if (is_hdmi(p)) {
            UPDATE(reg("DIG%u_HDMI_GC", p.engine), {{"HDMI_GC_AVMUTE", 0}});
        }
    }
    // ---------------- DSC (link_dpms.c, dcn32_hwseq.c, dcn20/30/32 optc and encoder) ----------------
    // The DSC engine blocks (dsc2_set_config's dsc_write_to_registers, dsc2_enable,
    // dsc2_disable, dsc2_disconnect, dsc2_read_state, dsc2_wait_disconnect_pending_clear)
    // are the owner's dcn32_dsc, checked write for write against the unmodified Linux
    // dcn20_dsc.c in dcn32_dsc_tests; this transcription calls them on the same model.
    rx7900::display::Dcn32StreamIo& dsc_io() {
        if (!dsc_io_bound_) {
            dsc_io_.bind(io_, segment[1], segment[2], segment[3]);
            dsc_io_bound_ = true;
        }
        return dsc_io_;
    }
    void optc_set_dsc_config(uint32_t otg, uint32_t mode, uint32_t bytes_per_pixel, uint32_t slice_width) {
        // optc3_set_dsc_config -> optc2_set_dsc_config, then OTG_V_SYNC_MODE 0
        UPDATE(reg("ODM%u_OPTC_DATA_FORMAT_CONTROL", otg), {{"OPTC_DSC_MODE", mode}});
        SET(reg("ODM%u_OPTC_BYTES_PER_PIXEL", otg), 0, {{"OPTC_DSC_BYTES_PER_PIXEL", bytes_per_pixel}});
        UPDATE(reg("ODM%u_OPTC_WIDTH_CONTROL", otg), {{"OPTC_DSC_SLICE_WIDTH", slice_width}});
        UPDATE(reg("OTG%u_OTG_V_SYNC_A_CNTL", otg), {{"OTG_V_SYNC_MODE", 0}});
    }
    void dp_set_dsc_config(uint32_t engine, bool enable) { // enc32_dp_set_dsc_config
        UPDATE(reg("DP%u_DP_DSC_CNTL", engine), {{"DP_DSC_MODE", enable ? 1u : 0u}});
    }
    void dp_set_dsc_pps_info_packet(uint32_t engine, bool enable, const uint8_t* pps) { // enc3_dp_set_dsc_pps_info_packet
        if (enable) {
            UPDATE(reg("DP%u_DP_SEC_CNTL2", engine), {{"DP_SEC_GSP11_PPS", 1}});
            UPDATE(reg("DIG%u_AFMT_CNTL", engine), {{"AFMT_AUDIO_CLOCK_EN", 1}});
            rx7900::display::InfoPacket pps_sdp{};
            pps_sdp.valid = true;
            pps_sdp.hb0 = 0;
            pps_sdp.hb1 = 0x10; // DC_DP_INFOFRAME_TYPE_PPS
            pps_sdp.hb2 = 127;
            pps_sdp.hb3 = 0;
            for (uint32_t i = 0; i < 4; ++i) {
                std::memcpy(pps_sdp.sb, &pps[i * 32], 32);
                vpg_update(engine, 11 + i, pps_sdp); // immediate_update (link_set_dpms_on)
            }
            UPDATE(reg("DP%u_DP_GSP11_CNTL", engine), {{"DP_SEC_GSP11_LINE_NUM", 2}});
            UPDATE(reg("DP%u_DP_MSA_VBID_MISC", engine), {{"DP_VBID6_LINE_REFERENCE", 0}, {"DP_VBID6_LINE_NUM", 3}});
            UPDATE(reg("DP%u_DP_GSP11_CNTL", engine), {{"DP_SEC_GSP11_ENABLE", 1}});
            UPDATE(reg("DP%u_DP_SEC_CNTL", engine), {{"DP_SEC_STREAM_ENABLE", 1}});
        } else {
            UPDATE(reg("DP%u_DP_GSP11_CNTL", engine), {{"DP_SEC_GSP11_ENABLE", 0}});
            UPDATE(reg("DP%u_DP_SEC_CNTL2", engine), {{"DP_SEC_GSP11_PPS", 0}});
        }
    }
    void link_set_dsc_on_stream(const State& s, int master, bool enable) {
        const Dcn32StreamPlan& p = s.stream;
        const uint32_t otg = static_cast<uint32_t>(s.pipe[master].tg);
        // dccg32 has no set_dto_dscclk / set_ref_dscclk.
        if (enable) {
            for (int odm = master; odm >= 0; odm = s.pipe[odm].next_odm) {
                const uint32_t dsc = static_cast<uint32_t>(s.pipe[odm].dsc);
                rx7900::display::dcn32_dsc::write_registers(dsc_io(), dsc, p.dsc_registers); // dsc2_set_config
                rx7900::display::dcn32_dsc::enable(dsc_io(), dsc, static_cast<uint32_t>(s.pipe[odm].opp));
            }
            dp_set_dsc_config(p.engine, true);
            optc_set_dsc_config(otg, 1 /* OPTC_DSC_ENABLED_444 */, p.dsc_optc.bytes_per_pixel, p.dsc_optc.slice_width);
        } else {
            optc_set_dsc_config(otg, 0, 0, 0);
            dp_set_dsc_config(p.engine, false);
            dp_set_dsc_pps_info_packet(p.engine, false, nullptr);
            for (int odm = master; odm >= 0; odm = s.pipe[odm].next_odm) {
                const uint32_t dsc = static_cast<uint32_t>(s.pipe[odm].dsc);
                rx7900::display::dcn32_dsc::disconnect(dsc_io(), dsc);
                rx7900::display::dcn32_dsc::disable(dsc_io(), dsc);
            }
        }
    }
    void dp_set_dsc_on_rx(bool enable) { // dm_helpers_dp_write_dsc_enable: DP_DSC_ENABLE
        Hooks::dpcd_write(&hooks_, 0x160, enable ? 1 : 0);
    }
    void link_set_dsc_pps_packet(const State& s, bool enable) { // link_set_dsc_pps_packet(immediate)
        dp_set_dsc_pps_info_packet(s.stream.engine, enable, s.stream.dsc_pps);
    }
    void update_dsc_on_stream(const State& s, int master) { // dcn32_update_dsc_on_stream(pipe_ctx, true)
        bool fw_en = false;
        rx7900::display::dcn32_dsc::read_state(dsc_io(), static_cast<uint32_t>(s.pipe[master].dsc), fw_en);
        if (!fw_en) {
            return; // "DSC has been disabled"
        }
        for (int odm = master; odm >= 0; odm = s.pipe[odm].next_odm) {
            const uint32_t dsc = static_cast<uint32_t>(s.pipe[odm].dsc);
            rx7900::display::dcn32_dsc::write_registers(dsc_io(), dsc, s.stream.dsc_registers);
            rx7900::display::dcn32_dsc::enable(dsc_io(), dsc, static_cast<uint32_t>(s.pipe[odm].opp));
        }
        optc_set_dsc_config(static_cast<uint32_t>(s.pipe[master].tg), 1, s.stream.dsc_optc.bytes_per_pixel,
                            s.stream.dsc_optc.slice_width);
    }
    void update_odm(const State& s, int master) { // dcn32_update_odm
        int opp_inst[4]{};
        int opp_cnt = 0;
        for (int odm = master; odm >= 0; odm = s.pipe[odm].next_odm) {
            opp_inst[opp_cnt++] = s.pipe[odm].opp;
        }
        const uint32_t otg = static_cast<uint32_t>(s.pipe[master].tg);
        if (opp_cnt > 1) {
            uint32_t x = 0, width = 0;
            odm_slice_src(s, master, x, width);
            set_odm_combine(otg, opp_inst, opp_cnt, static_cast<int>(width));
        } else {
            set_odm_bypass(otg);
        }
        for (int odm = s.pipe[master].next_odm; odm >= 0; odm = s.pipe[odm].next_odm) {
            UPDATE(reg("OPP_PIPE%u_OPP_PIPE_CONTROL", static_cast<uint32_t>(s.pipe[odm].opp)), {{"OPP_PIPE_CLOCK_EN", 1}});
            UPDATE(reg("FMT%u_FMT_422_CONTROL", static_cast<uint32_t>(s.pipe[odm].opp)),
                   {{"FMT_LEFT_EDGE_EXTRA_PIXEL_COUNT", 0}}); // opp2_program_left_edge_extra_pixel, RGB
        }
        if (s.pipe[master].dsc >= 0) {
            update_dsc_on_stream(s, master); // timing.flags.DSC
            const PipeCtx& current_pipe = cur_.pipe[master];
            if (s.pipe[master].next_odm < 0 && current_pipe.next_odm >= 0 && cur_.pipe[current_pipe.next_odm].dsc >= 0) {
                rx7900::display::dcn32_dsc::disconnect(dsc_io(), static_cast<uint32_t>(cur_.pipe[current_pipe.next_odm].dsc));
            }
        }
        if (!is_dpp_pipe(s, master)) {
            blank_pixel_data(s, master, true);
        }
    }

    void reset_back_end_for_pipe() { // dcn20_reset_hw_ctx_wrap -> dcn20_reset_back_end_for_pipe
        const int master = otg_master_of(cur_);
        const Dcn32StreamPlan old = cur_.stream;
        if (!dpms_off_) {
            link_set_dpms_off(cur_, master);
        }
        dpms_off_ = false;
        disable_crtc(old.pipe);
        enable_optc_clock(old.pipe, false);
        set_odm_bypass(old.pipe);
        set_drr_no_params(old.pipe);
        // DEVIATION: the SYMCLK_ON_TX_OFF disable_link_output pass is folded into disable_link.
        Hooks::power_down(&hooks_, old); // cs_power_down
        cur_.pipe[master].stream = false; // pipe_ctx->stream = NULL (OTG master only)
        cur_.has_stream = false;
        firmware_ = false;
    }

    // ---------------- front end ----------------
    void apply_single_controller(const State& s, int master) { // dce110_apply_single_controller_ctx_to_hw
        const Dcn32StreamPlan& p = s.stream;
        program_fmt(static_cast<uint32_t>(s.pipe[master].opp));
        set_dyn_expansion(static_cast<uint32_t>(s.pipe[master].opp), p.signal);
        for (int odm = s.pipe[master].next_odm; odm >= 0; odm = s.pipe[odm].next_odm) {
            set_dyn_expansion(static_cast<uint32_t>(s.pipe[odm].opp), p.signal);
            program_fmt(static_cast<uint32_t>(s.pipe[odm].opp));
        }
        // dcn20_enable_stream_timing. DEVIATION: dccg32_set_pixel_rate_div runs in
        // the link owner's set_pixel_clock after the OPTC clock instead of first.
        int opp_inst[4]{};
        int opp_cnt = 0;
        for (int odm = master; odm >= 0; odm = s.pipe[odm].next_odm) {
            opp_inst[opp_cnt++] = s.pipe[odm].opp;
        }
        if (opp_cnt > 1) {
            uint32_t x = 0, width = 0;
            odm_slice_src(s, master, x, width);
            set_odm_combine(p.pipe, opp_inst, opp_cnt, static_cast<int>(width));
        }
        enable_optc_clock(p.pipe, true);
        hooks_.record_pixel_clock(p); // program_pix_clk (+ pixel rate divider)
        program_timing(s, master);
        // no mpc set_out_rate_control on dcn32
        for (int i = 0; i < opp_cnt; ++i) {
            UPDATE(reg("OPP_PIPE%u_OPP_PIPE_CONTROL", static_cast<uint32_t>(opp_inst[i])), {{"OPP_PIPE_CLOCK_EN", 1}});
            UPDATE(reg("FMT%u_FMT_422_CONTROL", static_cast<uint32_t>(opp_inst[i])), {{"FMT_LEFT_EDGE_EXTRA_PIXEL_COUNT", 0}});
        }
        blank_pixel_data(s, master, true);
        enable_crtc(p.pipe);
        wait_for_blank_complete(static_cast<uint32_t>(s.pipe[master].opp));
        set_drr_no_params(p.pipe);
        set_static_screen_control(p.pipe);
        // back in dce110_apply_single_controller_ctx_to_hw
        setup_vupdate_interrupt(s, master);
        set_drr_no_params(p.pipe);
        set_static_screen_control(p.pipe);
        UPDATE(reg("DIG%u_DIG_FE_CNTL", p.engine), {{"DIG_SOURCE_SELECT", p.pipe}}); // enc1_dig_connect_to_otg
        link_set_dpms_on(s, master);
    }
    void resync_fifo_dccg_dio() { // dcn32_resync_fifo_dccg_dio -> dccg32_trigger_dio_fifo_resync
        const uint32_t rdivider = GET(plain("DENTIST_DISPCLK_CNTL"), "DENTIST_DISPCLK_RDIVIDER");
        if (rdivider != 0) {
            UPDATE(plain("DENTIST_DISPCLK_CNTL"), {{"DENTIST_DISPCLK_WDIVIDER", rdivider}});
        }
    }
    void hubp_setup(const State& s, uint32_t i) { // hubp3_setup
        const auto& bw = s.pipe[i].dlg_param;
        const auto& tm = s.stream.timing;
        WRITE(reg("HUBP%u_HUBPREQ_DEBUG_DB", i), 1u << 8);
        uint32_t vready = 0;
        if (tm.h_total != 0) {
            vready = (bw.vstartup_start - (bw.vready_offset + bw.vupdate_width + bw.vupdate_offset) / tm.h_total) <=
                     s.stream.vblank_end ? 1u : 0u;
        }
        UPDATE(reg("HUBP%u_DCHUBP_CNTL", i), {{"HUBP_VREADY_AT_OR_AFTER_VSYNC", vready}});
        const auto& rq = bw.rq;
        UPDATE(reg("HUBPRET%u_HUBPRET_CONTROL", i), {{"DET_BUF_PLANE1_BASE_ADDRESS", rq.plane1_base_address}});
        SET(reg("HUBPREQ%u_DCN_EXPANSION_MODE", i), 0,
            {{"DRQ_EXPANSION_MODE", rq.drq_expansion_mode}, {"PRQ_EXPANSION_MODE", rq.prq_expansion_mode},
             {"MRQ_EXPANSION_MODE", rq.mrq_expansion_mode}, {"CRQ_EXPANSION_MODE", rq.crq_expansion_mode}});
        const auto& l = rq.rq_regs_l;
        const auto& c = rq.rq_regs_c;
        SET(reg("HUBP%u_DCHUBP_REQ_SIZE_CONFIG", i), 0,
            {{"CHUNK_SIZE", l.chunk_size}, {"MIN_CHUNK_SIZE", l.min_chunk_size}, {"META_CHUNK_SIZE", l.meta_chunk_size},
             {"MIN_META_CHUNK_SIZE", l.min_meta_chunk_size}, {"DPTE_GROUP_SIZE", l.dpte_group_size},
             {"VM_GROUP_SIZE", l.mpte_group_size}, {"SWATH_HEIGHT", l.swath_height},
             {"PTE_ROW_HEIGHT_LINEAR", l.pte_row_height_linear}});
        SET(reg("HUBP%u_DCHUBP_REQ_SIZE_CONFIG_C", i), 0,
            {{"CHUNK_SIZE_C", c.chunk_size}, {"MIN_CHUNK_SIZE_C", c.min_chunk_size},
             {"META_CHUNK_SIZE_C", c.meta_chunk_size}, {"MIN_META_CHUNK_SIZE_C", c.min_meta_chunk_size},
             {"DPTE_GROUP_SIZE_C", c.dpte_group_size}, {"SWATH_HEIGHT_C", c.swath_height},
             {"PTE_ROW_HEIGHT_LINEAR_C", c.pte_row_height_linear}});
        const auto& d = bw.dlg;
        const auto& t = bw.ttu;
        SET(reg("HUBPREQ%u_BLANK_OFFSET_0", i), 0, {{"REFCYC_H_BLANK_END", d.refcyc_h_blank_end},
                                                     {"DLG_V_BLANK_END", d.dlg_vblank_end}});
        SET(reg("HUBPREQ%u_BLANK_OFFSET_1", i), 0, {{"MIN_DST_Y_NEXT_START", d.min_dst_y_next_start}});
        SET(reg("HUBPREQ%u_DST_DIMENSIONS", i), 0, {{"REFCYC_PER_HTOTAL", d.refcyc_per_htotal}});
        SET(reg("HUBPREQ%u_DST_AFTER_SCALER", i), 0, {{"REFCYC_X_AFTER_SCALER", d.refcyc_x_after_scaler},
                                                       {"DST_Y_AFTER_SCALER", d.dst_y_after_scaler}});
        SET(reg("HUBPREQ%u_REF_FREQ_TO_PIX_FREQ", i), 0, {{"REF_FREQ_TO_PIX_FREQ", d.ref_freq_to_pix_freq}});
        SET(reg("HUBPREQ%u_VBLANK_PARAMETERS_1", i), 0, {{"REFCYC_PER_PTE_GROUP_VBLANK_L", d.refcyc_per_pte_group_vblank_l}});
        SET(reg("HUBPREQ%u_NOM_PARAMETERS_0", i), 0, {{"DST_Y_PER_PTE_ROW_NOM_L", d.dst_y_per_pte_row_nom_l}});
        SET(reg("HUBPREQ%u_NOM_PARAMETERS_1", i), 0, {{"REFCYC_PER_PTE_GROUP_NOM_L", d.refcyc_per_pte_group_nom_l}});
        SET(reg("HUBPREQ%u_NOM_PARAMETERS_4", i), 0, {{"DST_Y_PER_META_ROW_NOM_L", d.dst_y_per_meta_row_nom_l}});
        SET(reg("HUBPREQ%u_NOM_PARAMETERS_5", i), 0, {{"REFCYC_PER_META_CHUNK_NOM_L", d.refcyc_per_meta_chunk_nom_l}});
        SET(reg("HUBPREQ%u_PER_LINE_DELIVERY", i), 0, {{"REFCYC_PER_LINE_DELIVERY_L", d.refcyc_per_line_delivery_l},
                                                        {"REFCYC_PER_LINE_DELIVERY_C", d.refcyc_per_line_delivery_c}});
        SET(reg("HUBPREQ%u_VBLANK_PARAMETERS_2", i), 0, {{"REFCYC_PER_PTE_GROUP_VBLANK_C", d.refcyc_per_pte_group_vblank_c}});
        SET(reg("HUBPREQ%u_NOM_PARAMETERS_2", i), 0, {{"DST_Y_PER_PTE_ROW_NOM_C", d.dst_y_per_pte_row_nom_c}});
        SET(reg("HUBPREQ%u_NOM_PARAMETERS_3", i), 0, {{"REFCYC_PER_PTE_GROUP_NOM_C", d.refcyc_per_pte_group_nom_c}});
        SET(reg("HUBPREQ%u_NOM_PARAMETERS_6", i), 0, {{"DST_Y_PER_META_ROW_NOM_C", d.dst_y_per_meta_row_nom_c}});
        SET(reg("HUBPREQ%u_NOM_PARAMETERS_7", i), 0, {{"REFCYC_PER_META_CHUNK_NOM_C", d.refcyc_per_meta_chunk_nom_c}});
        SET(reg("HUBPREQ%u_DCN_TTU_QOS_WM", i), 0, {{"QoS_LEVEL_LOW_WM", t.qos_level_low_wm},
                                                     {"QoS_LEVEL_HIGH_WM", t.qos_level_high_wm}});
        SET(reg("HUBPREQ%u_DCN_SURF0_TTU_CNTL0", i), 0, {{"REFCYC_PER_REQ_DELIVERY", t.refcyc_per_req_delivery_l},
                                                          {"QoS_LEVEL_FIXED", t.qos_level_fixed_l},
                                                          {"QoS_RAMP_DISABLE", t.qos_ramp_disable_l}});
        // Linux programs SURF1/CUR0 through the SURF0 field masks (same layout).
        const Reg surf1{reg("HUBPREQ%u_DCN_SURF1_TTU_CNTL0", i).name, "HUBPREQ0_DCN_SURF0_TTU_CNTL0"};
        SET(surf1, 0, {{"REFCYC_PER_REQ_DELIVERY", t.refcyc_per_req_delivery_c}, {"QoS_LEVEL_FIXED", t.qos_level_fixed_c},
                       {"QoS_RAMP_DISABLE", t.qos_ramp_disable_c}});
        const Reg cur0{reg("HUBPREQ%u_DCN_CUR0_TTU_CNTL0", i).name, "HUBPREQ0_DCN_SURF0_TTU_CNTL0"};
        SET(cur0, 0, {{"REFCYC_PER_REQ_DELIVERY", t.refcyc_per_req_delivery_cur0},
                      {"QoS_LEVEL_FIXED", t.qos_level_fixed_cur0}, {"QoS_RAMP_DISABLE", t.qos_ramp_disable_cur0}});
        SET(reg("HUBPREQ%u_FLIP_PARAMETERS_1", i), 0, {{"REFCYC_PER_PTE_GROUP_FLIP_L", d.refcyc_per_pte_group_flip_l}});
        UPDATE(reg("HUBPREQ%u_DCN_DMDATA_VM_CNTL", i), {{"REFCYC_PER_VM_DMDATA", d.refcyc_per_vm_dmdata}});
    }
    void hubp_setup_interdependent(const State& s, uint32_t i) { // hubp2_setup_interdependent
        const auto& d = s.pipe[i].dlg_param.dlg;
        const auto& t = s.pipe[i].dlg_param.ttu;
        SET(reg("HUBPREQ%u_PREFETCH_SETTINGS", i), 0, {{"DST_Y_PREFETCH", d.dst_y_prefetch}, {"VRATIO_PREFETCH", d.vratio_prefetch}});
        SET(reg("HUBPREQ%u_PREFETCH_SETTINGS_C", i), 0, {{"VRATIO_PREFETCH_C", d.vratio_prefetch_c}});
        SET(reg("HUBPREQ%u_VBLANK_PARAMETERS_0", i), 0, {{"DST_Y_PER_VM_VBLANK", d.dst_y_per_vm_vblank},
                                                          {"DST_Y_PER_ROW_VBLANK", d.dst_y_per_row_vblank}});
        SET(reg("HUBPREQ%u_FLIP_PARAMETERS_0", i), 0, {{"DST_Y_PER_VM_FLIP", d.dst_y_per_vm_flip},
                                                        {"DST_Y_PER_ROW_FLIP", d.dst_y_per_row_flip}});
        SET(reg("HUBPREQ%u_VBLANK_PARAMETERS_3", i), 0, {{"REFCYC_PER_META_CHUNK_VBLANK_L", d.refcyc_per_meta_chunk_vblank_l}});
        SET(reg("HUBPREQ%u_VBLANK_PARAMETERS_4", i), 0, {{"REFCYC_PER_META_CHUNK_VBLANK_C", d.refcyc_per_meta_chunk_vblank_c}});
        SET(reg("HUBPREQ%u_FLIP_PARAMETERS_2", i), 0, {{"REFCYC_PER_META_CHUNK_FLIP_L", d.refcyc_per_meta_chunk_flip_l}});
        SET(reg("HUBPREQ%u_PER_LINE_DELIVERY_PRE", i), 0,
            {{"REFCYC_PER_LINE_DELIVERY_PRE_L", d.refcyc_per_line_delivery_pre_l},
             {"REFCYC_PER_LINE_DELIVERY_PRE_C", d.refcyc_per_line_delivery_pre_c}});
        SET(reg("HUBPREQ%u_DCN_SURF0_TTU_CNTL1", i), 0, {{"REFCYC_PER_REQ_DELIVERY_PRE", t.refcyc_per_req_delivery_pre_l}});
        const Reg surf1{reg("HUBPREQ%u_DCN_SURF1_TTU_CNTL1", i).name, "HUBPREQ0_DCN_SURF0_TTU_CNTL1"};
        SET(surf1, 0, {{"REFCYC_PER_REQ_DELIVERY_PRE", t.refcyc_per_req_delivery_pre_c}});
        const Reg cur0{reg("HUBPREQ%u_DCN_CUR0_TTU_CNTL1", i).name, "HUBPREQ0_DCN_SURF0_TTU_CNTL1"};
        SET(cur0, 0, {{"REFCYC_PER_REQ_DELIVERY_PRE", t.refcyc_per_req_delivery_pre_cur0}});
        const Reg cur1{reg("HUBPREQ%u_DCN_CUR1_TTU_CNTL1", i).name, "HUBPREQ0_DCN_SURF0_TTU_CNTL1"};
        SET(cur1, 0, {{"REFCYC_PER_REQ_DELIVERY_PRE", t.refcyc_per_req_delivery_pre_cur1}});
        SET(reg("HUBPREQ%u_DCN_GLOBAL_TTU_CNTL", i), 0, {{"MIN_TTU_VBLANK", t.min_ttu_vblank},
                                                          {"QoS_LEVEL_FLIP", t.qos_level_flip}});
    }
    void dpp_setup(uint32_t i) { // dpp3_cnv_setup(ARGB8888, EXPANSION_MODE_ZERO, no ICSC, SRGB)
        const Reg f = reg("CNVC_CFG%u_FORMAT_CONTROL", i);
        SET(f, 0, {{"CNVC_BYPASS", 0}, {"FORMAT_EXPANSION_MODE", 1}});
        UPDATE(f, {{"FORMAT_CNV16", 0}});
        UPDATE(f, {{"CNVC_BYPASS_MSB_ALIGN", 0}});
        UPDATE(f, {{"CLAMP_POSITIVE", 0}});
        UPDATE(f, {{"CLAMP_POSITIVE_C", 0}});
        UPDATE(f, {{"FORMAT_CROSSBAR_R", 0}});
        UPDATE(f, {{"FORMAT_CROSSBAR_G", 1}});
        UPDATE(f, {{"FORMAT_CROSSBAR_B", 2}});
        SET(reg("CNVC_CFG%u_CNVC_SURFACE_PIXEL_FORMAT", i), 0, {{"CNVC_SURFACE_PIXEL_FORMAT", 8}, {"CNVC_ALPHA_PLANE_ENABLE", 0}});
        UPDATE(f, {{"ALPHA_EN", 1}}); // FORMAT_CONTROL__ALPHA_EN
        SET(reg("CNVC_CFG%u_PRE_DEALPHA", i), 0, {{"PRE_DEALPHA_EN", 0}, {"PRE_DEALPHA_ABLND_EN", 0}});
        SET(reg("CNVC_CFG%u_PRE_REALPHA", i), 0, {{"PRE_REALPHA_EN", 0}, {"PRE_REALPHA_ABLND_EN", 0}});
        SET(reg("CM%u_CM_POST_CSC_CONTROL", i), 0, {{"CM_POST_CSC_MODE", 0}}); // INPUT_CSC_SELECT_BYPASS
    }
    // plane_res.scl_data as dpp1_dscl_set_scaler_manual_scale's memcmp sees it.
    Scl scl_data(const State& s, int i) const {
        Scl d{};
        d.valid = true;
        d.recout = s.pipe[i].recout;
        d.viewport = s.pipe[i].viewport;
        uint32_t x = 0;
        odm_slice_src(s, opp_head_of(s, i), x, d.h_active);
        d.v_active = s.stream.timing.height;
        d.alpha_en = plane_of(s, i).per_pixel_alpha;
        return d;
    }
    static bool same_scl(const Scl& a, const Scl& b) {
        return a.valid == b.valid && a.recout.x == b.recout.x && a.recout.y == b.recout.y &&
               a.recout.width == b.recout.width && a.recout.height == b.recout.height && a.viewport.x == b.viewport.x &&
               a.viewport.y == b.viewport.y && a.viewport.width == b.viewport.width &&
               a.viewport.height == b.viewport.height && a.h_active == b.h_active && a.v_active == b.v_active &&
               a.alpha_en == b.alpha_en;
    }
    void dpp_set_scaler(const State& s, uint32_t i) { // dpp1_dscl_set_scaler_manual_scale at 1:1
        const Scl d = scl_data(s, static_cast<int>(i));
        if (same_scl(scl_[i], d)) {
            return; // memcmp of the cached scl_data
        }
        scl_[i] = d;
        SET(reg("DSCL%u_DSCL_AUTOCAL", i), 0, {{"AUTOCAL_MODE", 0}, {"AUTOCAL_NUM_PIPE", 0}, {"AUTOCAL_PIPE_ID", 0}});
        SET(reg("DSCL%u_DSCL_CONTROL", i), 0, {{"SCL_BOUNDARY_MODE", 0}});
        SET(reg("DSCL%u_RECOUT_START", i), 0, {{"RECOUT_START_X", static_cast<uint32_t>(d.recout.x)},
                                               {"RECOUT_START_Y", static_cast<uint32_t>(d.recout.y)}});
        SET(reg("DSCL%u_RECOUT_SIZE", i), 0, {{"RECOUT_WIDTH", static_cast<uint32_t>(d.recout.width)},
                                              {"RECOUT_HEIGHT", static_cast<uint32_t>(d.recout.height)}});
        SET(reg("DSCL%u_MPC_SIZE", i), 0, {{"MPC_WIDTH", d.h_active}, {"MPC_HEIGHT", d.v_active}});
        UPDATE(reg("DSCL%u_SCL_MODE", i), {{"DSCL_MODE", 0}}); // DSCL_MODE_SCALING_444_BYPASS
        // dpp1_dscl_set_lb, float format, use_max_lb -> LB_MEMORY_CONFIG_0, 31 partitions
        SET(reg("DSCL%u_LB_DATA_FORMAT", i), 0, {{"INTERLEAVE_EN", 0}, {"ALPHA_EN", d.alpha_en ? 1u : 0u}});
        SET(reg("DSCL%u_LB_MEMORY_CTRL", i), 0, {{"MEMORY_CONFIG", 0}, {"LB_MAX_PARTITIONS", 31}});
    }
    void mem_program_viewport(const State& s, uint32_t i) { // min_set_viewport
        const auto& v = s.pipe[i].viewport;
        const uint32_t x = static_cast<uint32_t>(v.x), y = static_cast<uint32_t>(v.y);
        const uint32_t w = static_cast<uint32_t>(v.width), hgt = static_cast<uint32_t>(v.height);
        SET(reg("HUBP%u_DCSURF_PRI_VIEWPORT_DIMENSION", i), 0, {{"PRI_VIEWPORT_WIDTH", w}, {"PRI_VIEWPORT_HEIGHT", hgt}});
        SET(reg("HUBP%u_DCSURF_PRI_VIEWPORT_START", i), 0, {{"PRI_VIEWPORT_X_START", x}, {"PRI_VIEWPORT_Y_START", y}});
        SET(reg("HUBP%u_DCSURF_SEC_VIEWPORT_DIMENSION", i), 0, {{"SEC_VIEWPORT_WIDTH", w}, {"SEC_VIEWPORT_HEIGHT", hgt}});
        SET(reg("HUBP%u_DCSURF_SEC_VIEWPORT_START", i), 0, {{"SEC_VIEWPORT_X_START", x}, {"SEC_VIEWPORT_Y_START", y}});
        SET(reg("HUBP%u_DCSURF_PRI_VIEWPORT_DIMENSION_C", i), 0, {{"PRI_VIEWPORT_WIDTH_C", w}, {"PRI_VIEWPORT_HEIGHT_C", hgt}});
        SET(reg("HUBP%u_DCSURF_PRI_VIEWPORT_START_C", i), 0, {{"PRI_VIEWPORT_X_START_C", x}, {"PRI_VIEWPORT_Y_START_C", y}});
        SET(reg("HUBP%u_DCSURF_SEC_VIEWPORT_DIMENSION_C", i), 0, {{"SEC_VIEWPORT_WIDTH_C", w}, {"SEC_VIEWPORT_HEIGHT_C", hgt}});
        SET(reg("HUBP%u_DCSURF_SEC_VIEWPORT_START_C", i), 0, {{"SEC_VIEWPORT_X_START_C", x}, {"SEC_VIEWPORT_Y_START_C", y}});
    }
    void gamut_remap_and_output_csc(const State& s, uint32_t i) {
        SET(reg("CM%u_CM_GAMUT_REMAP_CONTROL", i), 0, {{"CM_GAMUT_REMAP_MODE", 0}});         // dpp3 bypass
        SET(reg("MPCC_OGAM%u_MPCC_GAMUT_REMAP_MODE", i), 0, {{"MPCC_GAMUT_REMAP_MODE", 0}}); // mpc3 bypass
        // dcn20_program_output_csc: mpc3_power_on_ogam_lut(mpcc_id = hubp inst),
        // mpc3_set_ocsc_default(hubp->opp_id, SRGB, COEF_A)
        const uint32_t opp = static_cast<uint32_t>(s.pipe[i].opp);
        UPDATE(reg("MPCC%u_MPCC_MEM_PWR_CTRL", i), {{"MPCC_OGAM_MEM_PWR_DIS", 1}});
        WAIT(reg("MPCC%u_MPCC_MEM_PWR_CTRL", i), "MPCC_OGAM_MEM_PWR_STATE", 0, 10, 10);
        WRITE(plain("MPC_OUT_CSC_COEF_FORMAT"), 0);
        SET(reg("MPC_OUT%u_CSC_MODE", opp), 0, {{"MPC_OCSC_MODE", 1}});
        // cm_helper_program_color_matrices from CSC_C11_C12_A to CSC_C33_C34_A,
        // consecutive dwords with the C11/C12 masks (output_csc_matrix SRGB).
        static const uint16_t srgb[12] = {0x2000, 0, 0, 0, 0, 0x2000, 0, 0, 0, 0, 0x2000, 0};
        const Reg first = reg("MPC_OUT%u_CSC_C11_C12_A", opp);
        const Reg last = reg("MPC_OUT%u_CSC_C33_C34_A", opp);
        const auto c11 = field(reg("MPC_OUT%u_CSC_C11_C12_A", 0), "MPC_OCSC_C11_A");
        const auto c12 = field(reg("MPC_OUT%u_CSC_C11_C12_A", 0), "MPC_OCSC_C12_A");
        uint32_t k = 0;
        for (uint32_t offset = addr(first); offset <= addr(last); offset += 4, k += 2) {
            const uint32_t v = ((srgb[k] << c11.first) & c11.second) | ((srgb[k + 1] << c12.first) & c12.second);
            io_.write32(io_.context, offset, v);
        }
    }
    void program_surface_config(const State& s, uint32_t i) { // hubp3_program_surface_config
        const Dcn32StreamPlan& p = s.stream;
        UPDATE(reg("HUBPREQ%u_DCSURF_SURFACE_CONTROL", i) /* hubp3_dcc_control_sienna_cichlid */,
               {{"PRIMARY_SURFACE_DCC_EN", 0}, {"PRIMARY_SURFACE_DCC_IND_BLK", 0}, {"PRIMARY_SURFACE_DCC_IND_BLK_C", 0},
                {"SECONDARY_SURFACE_DCC_EN", 0}, {"SECONDARY_SURFACE_DCC_IND_BLK", 0},
                {"SECONDARY_SURFACE_DCC_IND_BLK_C", 0}});
        auto log2 = [](uint32_t v) {
            uint32_t r = 0;
            while (v > 1) {
                v >>= 1;
                ++r;
            }
            return r;
        };
        UPDATE(reg("HUBP%u_DCSURF_ADDR_CONFIG", i), {{"NUM_PIPES", log2(p.tiling.num_pipes)},
                                                     {"PIPE_INTERLEAVE", p.tiling.pipe_interleave_bytes},
                                                     {"MAX_COMPRESSED_FRAGS", log2(p.tiling.max_compressed_frags)},
                                                     {"NUM_PKRS", log2(p.tiling.num_pkrs)}});
        UPDATE(reg("HUBP%u_DCSURF_TILING_CONFIG", i), {{"SW_MODE", 0}, {"META_LINEAR", 0}, {"PIPE_ALIGNED", 0}});
        UPDATE(reg("HUBPREQ%u_DCSURF_SURFACE_PITCH", i),
               {{"PITCH", plane_of(s, static_cast<int>(i)).pitch_pixels - 1}, {"META_PITCH", 0}});
        UPDATE(reg("HUBP%u_DCSURF_SURFACE_CONFIG", i), {{"ROTATION_ANGLE", 0}, {"H_MIRROR_EN", 0}});
        UPDATE(reg("HUBPRET%u_HUBPRET_CONTROL", i), {{"CROSSBAR_SRC_CB_B", 2}, {"CROSSBAR_SRC_CR_R", 3}});
        UPDATE(reg("HUBP%u_DCSURF_SURFACE_CONFIG", i), {{"SURFACE_PIXEL_FORMAT", 8}});
        power_gated_[i] = false;
    }
    void update_plane_addr(const State& s, uint32_t i) { // hubp3_program_surface_flip_and_addr
        const uint64_t address = plane_of(s, static_cast<int>(i)).surface_address;
        const Reg flip = reg("HUBPREQ%u_DCSURF_FLIP_CONTROL", i);
        UPDATE(flip, {{"SURFACE_FLIP_TYPE", 0}});
        UPDATE(reg("HUBPREQ%u_VMID_SETTINGS_0", i), {{"VMID", 0}});
        UPDATE(flip, {{"SURFACE_FLIP_MODE_FOR_STEREOSYNC", 0}});
        UPDATE(flip, {{"SURFACE_FLIP_IN_STEREOSYNC", 0}});
        UPDATE(reg("HUBPREQ%u_DCSURF_SURFACE_CONTROL", i), {{"PRIMARY_SURFACE_TMZ", 0}, {"PRIMARY_META_SURFACE_TMZ", 0}});
        SET(reg("HUBPREQ%u_DCSURF_PRIMARY_SURFACE_ADDRESS_HIGH", i), 0,
            {{"PRIMARY_SURFACE_ADDRESS_HIGH", static_cast<uint32_t>(address >> 32)}});
        SET(reg("HUBPREQ%u_DCSURF_PRIMARY_SURFACE_ADDRESS", i), 0,
            {{"PRIMARY_SURFACE_ADDRESS", static_cast<uint32_t>(address)}});
        request_address_[i] = address;
    }
    // dcn20_update_dchubp_dpp. full: plane_state->update_flags all set (FULL
    // surface update); stream_all: dc_commit_state_no_check's stream flags.
    void update_dchubp_dpp(const State& s, uint32_t i, bool full, bool stream_all) {
        const UpdateFlags& f = s.pipe[i].update_flags;
        if (f.dppclk) { // dpp1_dppclk_control(false, true)
            UPDATE(reg("DPP_TOP%u_DPP_CONTROL", i), {{"DPPCLK_RATE_CONTROL", 0}, {"DPP_CLOCK_ENABLE", 1}});
        }
        // DEVIATION: dccg2_update_dpp_dto on enable runs in the link owner's update_clocks.
        if (f.hubp_rq_dlg_ttu) {
            UPDATE(reg("HUBP%u_DCHUBP_CNTL", i), {{"HUBP_VTG_SEL", static_cast<uint32_t>(s.pipe[i].tg)}}); // hubp2_vtg_sel
            hubp_setup(s, i);
        }
        if (f.unbounded_req) { // hubp31_set_unbounded_requesting
            const uint32_t unbounded = s.stream.bandwidth.unbounded_req ? 1u : 0u;
            UPDATE(reg("HUBP%u_DCHUBP_CNTL", i), {{"HUBP_UNBOUNDED_REQ_MODE", unbounded}});
            UPDATE(reg("CURSOR0_%u_CURSOR_CONTROL", i), {{"CURSOR_REQ_MODE", unbounded}});
        }
        if (f.hubp_interdependent) {
            hubp_setup_interdependent(s, i);
        }
        if (f.enable || f.plane_changed || full) {
            dpp_setup(i);
        }
        if (f.mpcc || f.plane_changed || full) {
            update_mpcc(s, static_cast<int>(i), full);
        }
        if (f.scaler || full || stream_all) {
            dpp_set_scaler(s, i);
        }
        if (f.viewport) { // context != dc->current_state
            mem_program_viewport(s, i);
        }
        if (f.enable || f.opp_changed || f.plane_changed || stream_all || full) {
            gamut_remap_and_output_csc(s, i);
        }
        if (f.enable || f.plane_changed || f.opp_changed || full) {
            program_surface_config(s, i);
        }
        if (f.enable || f.plane_changed || full) {
            update_plane_addr(s, i);
        }
        if (f.enable) {
            set_blank(i, false);
        }
    }
    void program_pipe(const State& s, uint32_t i, bool full, bool stream_all) { // dcn20_program_pipe
        const PipeCtx& pipe = s.pipe[i];
        const UpdateFlags& f = pipe.update_flags;
        const uint32_t tg = static_cast<uint32_t>(pipe.tg);
        if (is_otg_master(s, static_cast<int>(i))) {
            if (f.enable || f.odm || stream_all) {
                blank_pixel_data(s, static_cast<int>(i), false); // plane visible
            }
        }
        if (f.global_sync && pipe.top < 0 && pipe.prev_odm < 0) {
            // calculate_vready_offset_for_group
            uint32_t vready = pipe.dlg_param.vready_offset;
            for (int o = pipe.bottom; o >= 0; o = s.pipe[o].bottom) {
                vready = std::max(vready, s.pipe[o].dlg_param.vready_offset);
            }
            for (int o = pipe.top; o >= 0; o = s.pipe[o].top) {
                vready = std::max(vready, s.pipe[o].dlg_param.vready_offset);
            }
            for (int o = pipe.next_odm; o >= 0; o = s.pipe[o].next_odm) {
                vready = std::max(vready, s.pipe[o].dlg_param.vready_offset);
            }
            for (int o = pipe.prev_odm; o >= 0; o = s.pipe[o].prev_odm) {
                vready = std::max(vready, s.pipe[o].dlg_param.vready_offset);
            }
            program_global_sync(tg, vready, pipe.dlg_param.vstartup_start, pipe.dlg_param.vupdate_offset,
                                pipe.dlg_param.vupdate_width);
            wait_for_state(tg, false);
            set_vtg_params(tg, s.stream.timing, pipe.dlg_param.vstartup_start, true);
            setup_vupdate_interrupt(s, static_cast<int>(i));
        }
        if (f.odm) {
            update_odm(s, static_cast<int>(i));
        }
        if (f.enable) {
            // dcn20_enable_plane: dcn20_power_on_plane_resources, hubp clock, hubp3_init, OPP clock
            SET(plain("DC_IP_REQUEST_CNTL"), 0, {{"IP_REQUEST_EN", 1}});
            hubp_pg_control(i, true);
            SET(plain("DC_IP_REQUEST_CNTL"), 0, {{"IP_REQUEST_EN", 0}});
            UPDATE(reg("HUBP%u_HUBP_CLK_CNTL", i), {{"HUBP_CLOCK_ENABLE", 1}});
            WRITE(reg("HUBP%u_HUBPREQ_DEBUG", i), 1u << 26);
            UPDATE(reg("OPP_PIPE%u_OPP_PIPE_CONTROL", static_cast<uint32_t>(pipe.opp)), {{"OPP_PIPE_CLOCK_EN", 1}});
            // hubbub32_force_wm_propagate_to_pipes
            SET(plain("DCHUBBUB_ARB_DATA_URGENCY_WATERMARK_A"), 0,
                {{"DCHUBBUB_ARB_DATA_URGENCY_WATERMARK_A", convert_and_clamp(cache_[0].urgent_ns, refclk_mhz_, 0x3fff)}});
        }
        if (f.det_size) {
            program_det_size(i, pipe.dlg_param.det_buffer_size_kb);
        }
        if (f.raw() || full || stream_all) {
            update_dchubp_dpp(s, i, full, stream_all);
        }
        if (pipe.plane >= 0 && (f.enable || full)) {
            UPDATE(reg("CM%u_CM_HDR_MULT_COEF", i), {{"CM_HDR_MULT_COEF", 0x1f000}}); // dcn10_set_hdr_multiplier
        }
        if (pipe.plane >= 0 && (full || f.enable)) {
            // dcn32_set_input_transfer_func (TF_TYPE_BYPASS)
            SET(reg("CNVC_CFG%u_PRE_DEGAM", i), 0, {{"PRE_DEGAM_MODE", 0}, {"PRE_DEGAM_SELECT", 0}});
            UPDATE(reg("CM%u_CM_CONTROL", i), {{"CM_BYPASS", 0}}); // dpp3_enable_cm_block
            SET(reg("CM%u_CM_GAMCOR_CONTROL", i), 0, {{"CM_GAMCOR_MODE", 0}});
            SET(reg("MPCC_MCM%u_MPCC_MCM_1DLUT_CONTROL", i), 0, {{"MPCC_MCM_1DLUT_MODE", 0}});
            SET(reg("MPCC_MCM%u_MPCC_MCM_SHAPER_CONTROL", i), 0, {{"MPCC_MCM_SHAPER_LUT_MODE", 0}});
            UPDATE(reg("MPCC%u_MPCC_MOVABLE_CM_LOCATION_CONTROL", i), {{"MPCC_MOVABLE_CM_LOCATION_CNTL", 0}});
            UPDATE(reg("MPCC_MCM%u_MPCC_MCM_3DLUT_MODE", i), {{"MPCC_MCM_3DLUT_MODE", 0}, {"MPCC_MCM_3DLUT_SIZE", 1}});
        }
        if (f.enable || f.plane_changed || stream_all || (pipe.plane >= 0 && full)) {
            // dcn32_set_output_transfer_func: mpc3_set_output_gamma(NULL)
            SET(reg("MPCC_OGAM%u_MPCC_OGAM_CONTROL", i), 0, {{"MPCC_OGAM_MODE", 0}});
        }
        if (f.enable || f.opp_changed) {
            set_dyn_expansion(static_cast<uint32_t>(pipe.opp), s.stream.signal);
            program_fmt(static_cast<uint32_t>(pipe.opp));
        }
    }
    bool is_flip_pending(uint32_t i) { // hubp2_is_flip_pending
        const uint32_t pending = GET(reg("HUBPREQ%u_DCSURF_FLIP_CONTROL", i), "SURFACE_FLIP_PENDING");
        const uint64_t low = GET(reg("HUBPREQ%u_DCSURF_SURFACE_EARLIEST_INUSE", i), "SURFACE_EARLIEST_INUSE_ADDRESS");
        const uint64_t high = GET(reg("HUBPREQ%u_DCSURF_SURFACE_EARLIEST_INUSE_HIGH", i), "SURFACE_EARLIEST_INUSE_ADDRESS_HIGH");
        return pending != 0 || ((high << 32) | low) != request_address_[i];
    }
    void wait_flip(uint32_t i) {
        for (uint32_t j = 0; j < 100000 && is_flip_pending(i); ++j) {
            udelay(1);
        }
    }
    // dcn20_detect_pipe_changes(old_state = cur_, new_state)
    void detect_pipe_changes(const State& old_state, State& new_state, int i) const {
        const PipeCtx& old_pipe = old_state.pipe[i];
        PipeCtx& new_pipe = new_state.pipe[i];
        new_pipe.update_flags = UpdateFlags{};
        UpdateFlags& f = new_pipe.update_flags;
        if (is_otg_master(new_state, i)) { // resource_is_odm_topology_changed
            bool changed = !is_otg_master(old_state, i);
            int a = i, b = i;
            while (!changed && a >= 0 && b >= 0) {
                if (new_state.pipe[a].opp != old_state.pipe[b].opp ||
                    (new_state.pipe[a].next_odm >= 0) != (old_state.pipe[b].next_odm >= 0)) {
                    changed = true;
                }
                a = new_state.pipe[a].next_odm;
                b = old_state.pipe[b].next_odm;
            }
            f.odm = changed;
        }
        if (old_pipe.plane < 0 && new_pipe.plane < 0) {
            return;
        }
        if (old_pipe.plane < 0 && new_pipe.plane >= 0) {
            f.enable = f.mpcc = f.dppclk = f.hubp_interdependent = f.hubp_rq_dlg_ttu = true;
            f.unbounded_req = f.gamut_remap = f.scaler = f.viewport = f.det_size = true;
            if (new_pipe.top < 0 && new_pipe.prev_odm < 0) {
                f.odm = true;
                f.global_sync = true;
            }
            return;
        }
        if (old_pipe.plane >= 0 && new_pipe.plane < 0) {
            f.disable = true;
            return;
        }
        f.plane_changed = true; // every commit creates new dc_plane_state objects
        const auto& od = old_pipe.dlg_param;
        const auto& nd = new_pipe.dlg_param;
        if (is_otg_master(new_state, i) &&
            (od.vready_offset != nd.vready_offset || od.vstartup_start != nd.vstartup_start ||
             od.vupdate_offset != nd.vupdate_offset || od.vupdate_width != nd.vupdate_width)) {
            f.global_sync = true;
        }
        if (od.det_buffer_size_kb != nd.det_buffer_size_kb) {
            f.det_size = true;
        }
        if (old_pipe.opp != new_pipe.opp) {
            f.opp_changed = true;
        }
        if (old_pipe.opp != new_pipe.opp) { // dpp inst equal (same pipe index)
            f.mpcc = true;
        }
        if (od.dppclk_khz != nd.dppclk_khz) {
            f.dppclk = true;
        }
        if (!same_scl(scl_data(old_state, i), scl_data(new_state, i))) {
            f.scaler = true;
        }
        if (old_pipe.viewport.x != new_pipe.viewport.x || old_pipe.viewport.y != new_pipe.viewport.y ||
            old_pipe.viewport.width != new_pipe.viewport.width || old_pipe.viewport.height != new_pipe.viewport.height) {
            f.viewport = true;
        }
        rx7900::display::Dcn32DlgRegs old_dlg = od.dlg;
        rx7900::display::Dcn32TtuRegs old_ttu = od.ttu;
        const auto& nl = nd.dlg;
        const auto& nt = nd.ttu;
        if (old_dlg.dst_y_prefetch != nl.dst_y_prefetch || old_dlg.vratio_prefetch != nl.vratio_prefetch ||
            old_dlg.vratio_prefetch_c != nl.vratio_prefetch_c || old_dlg.dst_y_per_vm_vblank != nl.dst_y_per_vm_vblank ||
            old_dlg.dst_y_per_row_vblank != nl.dst_y_per_row_vblank || old_dlg.dst_y_per_vm_flip != nl.dst_y_per_vm_flip ||
            old_dlg.dst_y_per_row_flip != nl.dst_y_per_row_flip ||
            old_dlg.refcyc_per_meta_chunk_vblank_l != nl.refcyc_per_meta_chunk_vblank_l ||
            old_dlg.refcyc_per_meta_chunk_vblank_c != nl.refcyc_per_meta_chunk_vblank_c ||
            old_dlg.refcyc_per_meta_chunk_flip_l != nl.refcyc_per_meta_chunk_flip_l ||
            old_dlg.refcyc_per_line_delivery_pre_l != nl.refcyc_per_line_delivery_pre_l ||
            old_dlg.refcyc_per_line_delivery_pre_c != nl.refcyc_per_line_delivery_pre_c ||
            old_ttu.refcyc_per_req_delivery_pre_l != nt.refcyc_per_req_delivery_pre_l ||
            old_ttu.refcyc_per_req_delivery_pre_c != nt.refcyc_per_req_delivery_pre_c ||
            old_ttu.refcyc_per_req_delivery_pre_cur0 != nt.refcyc_per_req_delivery_pre_cur0 ||
            old_ttu.refcyc_per_req_delivery_pre_cur1 != nt.refcyc_per_req_delivery_pre_cur1 ||
            old_ttu.min_ttu_vblank != nt.min_ttu_vblank || old_ttu.qos_level_flip != nt.qos_level_flip) {
            old_dlg.dst_y_prefetch = nl.dst_y_prefetch;
            old_dlg.vratio_prefetch = nl.vratio_prefetch;
            old_dlg.vratio_prefetch_c = nl.vratio_prefetch_c;
            old_dlg.dst_y_per_vm_vblank = nl.dst_y_per_vm_vblank;
            old_dlg.dst_y_per_row_vblank = nl.dst_y_per_row_vblank;
            old_dlg.dst_y_per_vm_flip = nl.dst_y_per_vm_flip;
            old_dlg.dst_y_per_row_flip = nl.dst_y_per_row_flip;
            old_dlg.refcyc_per_meta_chunk_vblank_l = nl.refcyc_per_meta_chunk_vblank_l;
            old_dlg.refcyc_per_meta_chunk_vblank_c = nl.refcyc_per_meta_chunk_vblank_c;
            old_dlg.refcyc_per_meta_chunk_flip_l = nl.refcyc_per_meta_chunk_flip_l;
            old_dlg.refcyc_per_line_delivery_pre_l = nl.refcyc_per_line_delivery_pre_l;
            old_dlg.refcyc_per_line_delivery_pre_c = nl.refcyc_per_line_delivery_pre_c;
            old_ttu.refcyc_per_req_delivery_pre_l = nt.refcyc_per_req_delivery_pre_l;
            old_ttu.refcyc_per_req_delivery_pre_c = nt.refcyc_per_req_delivery_pre_c;
            old_ttu.refcyc_per_req_delivery_pre_cur0 = nt.refcyc_per_req_delivery_pre_cur0;
            old_ttu.refcyc_per_req_delivery_pre_cur1 = nt.refcyc_per_req_delivery_pre_cur1;
            old_ttu.min_ttu_vblank = nt.min_ttu_vblank;
            old_ttu.qos_level_flip = nt.qos_level_flip;
            f.hubp_interdependent = true;
        }
        if (std::memcmp(&old_dlg, &nl, sizeof(old_dlg)) != 0 || std::memcmp(&old_ttu, &nt, sizeof(old_ttu)) != 0 ||
            std::memcmp(&od.rq, &nd.rq, sizeof(od.rq)) != 0) {
            f.hubp_rq_dlg_ttu = true;
        }
        if (old_state.stream.bandwidth.unbounded_req != new_state.stream.bandwidth.unbounded_req) {
            f.unbounded_req = true;
        }
    }
    static int hubp_count(const State& s) {
        int n = 0;
        for (const PipeCtx& p : s.pipe) {
            n += p.plane >= 0 ? 1 : 0;
        }
        return n;
    }
    // dcn20_program_front_end_for_ctx(context); dc->current_state is cur_.
    void program_front_end_for_ctx(State& context, bool full) {
        const bool stream_all = !full; // dc_commit_state_no_check sets every stream flag
        if (hubp_count(cur_) == 0 && hubp_count(context) > 0) {
            force_pstate_change_control(true, false);
            udelay(500);
        }
        for (int i = 0; i < 4; ++i) {
            detect_pipe_changes(cur_, context, i);
        }
        // OTG blank before disabling all front ends
        for (int i = 0; i < 4; ++i) {
            if (context.pipe[i].update_flags.disable && context.pipe[i].top < 0 && context.pipe[i].prev_odm < 0 &&
                context.pipe[i].stream) {
                blank_pixel_data(context, i, true);
            }
        }
        // Disconnect mpcc
        for (int i = 0; i < 4; ++i) {
            if (context.pipe[i].update_flags.disable || context.pipe[i].update_flags.opp_changed) {
                if (context.pipe[i].update_flags.disable) {
                    program_det_size(static_cast<uint32_t>(i), 0); // the current pipe's hubp inst
                }
                plane_atomic_disconnect(cur_.pipe[i], i);
            }
        }
        // update ODM for blanked OTG master pipes
        for (int i = 0; i < 4; ++i) {
            if (is_otg_master(context, i) && !is_dpp_pipe(context, i) && context.pipe[i].update_flags.odm) {
                update_odm(context, i);
            }
        }
        // Program all updated pipes, top pipes first and their bottom pipes in order.
        for (int i = 0; i < 4; ++i) {
            if (context.pipe[i].plane >= 0 && context.pipe[i].top < 0) {
                for (int p = i; p >= 0; p = context.pipe[p].bottom) {
                    program_pipe(context, static_cast<uint32_t>(p), full, stream_all);
                }
            }
        }
    }
    void post_unlock_reset_opp(PipeCtx& opp_head, uint32_t index) { // post_unlock_reset_opp
        wait_for_mpcc_disconnect(opp_head);
        if (opp_head.dsc >= 0) {
            if (dsc_pg_status(static_cast<uint32_t>(opp_head.dsc))) {
                rx7900::display::dcn32_dsc::wait_disconnect_pending_clear(dsc_io(), static_cast<uint32_t>(opp_head.dsc));
                rx7900::display::dcn32_dsc::disable(dsc_io(), static_cast<uint32_t>(opp_head.dsc));
            }
        }
        (void)index;
    }
    void post_unlock_program_front_end(const State& context) { // dcn20_post_unlock_program_front_end
        for (int i = 0; i < 4; ++i) {
            if (is_opp_head(cur_, i) && !is_opp_head(context, i)) {
                post_unlock_reset_opp(cur_.pipe[i], static_cast<uint32_t>(i));
            }
        }
        for (int i = 0; i < 4; ++i) {
            if (context.pipe[i].update_flags.disable) { // dcn20_disable_plane(dc->current_state pipe)
                if (!cur_.pipe[i].hubp || power_gated_[i]) {
                    continue;
                }
                plane_atomic_disable(cur_.pipe[i], static_cast<uint32_t>(i));
            }
        }
        for (int i = 0; i < 4; ++i) {
            const PipeCtx& p = context.pipe[i];
            if (p.plane >= 0 && p.top < 0 && p.update_flags.enable) {
                wait_flip(static_cast<uint32_t>(i));
            }
        }
        for (int i = 0; i < 4; ++i) {
            if (is_otg_master(cur_, i) && is_otg_master(context, i) &&
                odm_slice_count(cur_, i) < odm_slice_count(context, i)) {
                for (uint32_t j = 0; j < 100000; ++j) { // optc32_get_double_buffer_pending
                    if (GET(reg("ODM%u_OPTC_INPUT_GLOBAL_CONTROL", static_cast<uint32_t>(context.pipe[i].tg)),
                            "OPTC_DOUBLE_BUFFER_PENDING") != 1) {
                        break;
                    }
                    udelay(1);
                }
            }
        }
        force_pstate_change_control(false, false);
        update_force_pstate(context);
        // dcn32_program_mall_pipe_config -> dcn32_update_mall_sel (no cursor in MALL)
        for (uint32_t i = 0; i < 4; ++i) {
            if (context.pipe[i].stream && context.pipe[i].plane >= 0 && context.pipe[i].hubp) {
                UPDATE(reg("HUBP%u_DCHUBP_MALL_CONFIG", i), {{"USE_MALL_SEL", context.stream.mall_sel},
                                                              {"USE_MALL_FOR_CURSOR", 0}});
            }
        }
    }
    void wait_for_no_pipes_pending(const State& context) { // hwss_wait_for_no_pipes_pending
        for (uint32_t i = 0; i < 4; ++i) {
            if (context.pipe[i].plane >= 0) {
                wait_flip(i);
            }
        }
    }
    void wait_for_odm_update_pending_complete(const State& context) { // hwss_wait_for_odm_update_pending_complete
        for (int i = 0; i < 4; ++i) {
            if (is_otg_master(context, i)) {
                // optc32_wait_odm_doublebuffer_pending_clear: a field dcn32 does
                // not list (mask 0): the REG_WAIT reads once.
                WAIT(reg("OTG%u_OTG_DOUBLE_BUFFER_CONTROL", static_cast<uint32_t>(context.pipe[i].tg)),
                     "OTG_H_TIMING_DIV_MODE_DB_UPDATE_PENDING", 0, 2, 50000);
            }
        }
        for (int i = 0; i < 4; ++i) { // hwss_wait_for_all_blank_complete
            if (is_opp_head(context, i)) {
                wait_for_blank_complete(static_cast<uint32_t>(context.pipe[i].opp));
            }
        }
    }

    // ---------------- dc_commit_state_no_check ----------------
    rx7900::display::Dcn32StreamClocks clocks_for(const Dcn32StreamPlan* target) const {
        if (target != nullptr) {
            rx7900::display::Dcn32StreamClocks c{target->bandwidth.clocks, target->pipe,
                                                 target->bandwidth.dpp_dppclk_khz, 1};
            if (target->bandwidth.pipe_count > 1) { // plane_res.bw.dppclk_khz of every pipe
                for (uint32_t k = 0; k < target->bandwidth.pipe_count; ++k) {
                    c.dpp_pipe_khz[target->bandwidth.pipes[k].pipe] = target->bandwidth.pipes[k].dppclk_khz;
                }
            }
            return c;
        }
        if (cur_.has_stream && !firmware_) {
            return {cur_.stream.bandwidth.clocks, cur_.stream.pipe, 0, 0};
        }
        return {};
    }
    void prepare_bandwidth(const rx7900::display::Dcn32BandwidthPlan& bw,
                           const rx7900::display::Dcn32StreamClocks& clocks) {
        // dcn32_prepare_bandwidth -> dcn20_prepare_bandwidth, then (no FPO)
        // dc_dmub_srv_p_state_delegate(dc, false, context).
        hooks_.record_clocks(clocks, false);
        program_watermarks(bw, false);
        program_compbuf_size(bw.compbuf_size_kb, false);
        hooks_.record_p_state_delegate(false);
    }
    void optimize_bandwidth(const rx7900::display::Dcn32BandwidthPlan& bw,
                            const rx7900::display::Dcn32StreamClocks& clocks) { // dcn20_optimize_bandwidth
        program_watermarks(bw, true);
        program_compbuf_size(bw.compbuf_size_kb, true);
        hooks_.record_clocks(clocks, true);
    }
    void commit(const Dcn32StreamPlan* target) {
        if (target != nullptr) {
            refclk_mhz_ = target->dchub_refclk_mhz;
        }
        State context{};
        if (target != nullptr) {
            context = state_from(*target);
        }
        // A context without streams keeps the copied bandwidth state of the current one.
        const rx7900::display::Dcn32BandwidthPlan none{};
        const rx7900::display::Dcn32BandwidthPlan bw =
            target != nullptr ? target->bandwidth : (cur_.has_stream && !firmware_ ? cur_.stream.bandwidth : none);
        const rx7900::display::Dcn32StreamClocks clocks = clocks_for(target);
        prepare_bandwidth(bw, clocks);
        update_dsc_pg(context, false);
        if (cur_.has_stream && hubp_count(cur_) > 0) {
            disable_dangling_plane();
        }
        // Program hardware: dc->hwss.wait_for_mpcc_disconnect(new pipes)
        for (const PipeCtx& p : context.pipe) {
            wait_for_mpcc_disconnect(p);
        }
        // dce110_apply_ctx_to_hw -> dcn20_reset_hw_ctx_wrap
        if (cur_.has_stream) {
            reset_back_end_for_pipe();
        }
        if (target != nullptr) {
            const int master = otg_master_of(context);
            apply_single_controller(context, master);
            resync_fifo_dccg_dio();
        }
        // Program all planes within the new context.
        interdependent_update_lock(context, true);
        program_front_end_for_ctx(context, false);
        interdependent_update_lock(context, false);
        post_unlock_program_front_end(context);
        if (target != nullptr) {
            // mode_changed stream: setup_stereo per pipe of the stream, then dc_enable_stereo.
            for (int round = 0; round < 2; ++round) {
                for (const PipeCtx& p : context.pipe) {
                    if (p.stream) {
                        setup_stereo(p);
                    }
                }
            }
            wait_for_no_pipes_pending(context);
            wait_for_odm_update_pending_complete(context);
        }
        optimize_bandwidth(bw, clocks);
        update_dsc_pg(context, true);
        cur_ = context;
        dpms_off_ = false;
    }

    // dc_update_planes_and_stream (update_planes_and_stream_v2) for a FULL
    // surface update of the current stream: commit_planes_for_stream, then
    // dc_post_update_surfaces_to_stream at once (DEVIATION: Linux waits for the
    // next fast update; this owner has none).
    void update_planes(const Dcn32StreamPlan& plan) {
        State context = state_from(plan);
        // commit_planes_for_stream: hwss_process_outstanding_hw_updates(dc->current_state)
        for (int i = 0; i < 4; ++i) {
            if (!cur_.pipe[i].stream || !cur_.pipe[i].hubp) {
                continue;
            }
            for (int opp = 0; opp < 4; ++opp) {
                if (disconnect_pending_[opp][i]) { // mpc2_assert_idle_mpcc
                    if (GET(reg("MPCC%u_MPCC_STATUS", static_cast<uint32_t>(i)), "MPCC_DISABLED") == 0) {
                        WAIT(reg("MPCC%u_MPCC_STATUS", static_cast<uint32_t>(i)), "MPCC_IDLE", 1, 1, 100000);
                    }
                    disconnect_pending_[opp][i] = false;
                    break;
                }
            }
        }
        wait_for_odm_update_pending_complete(cur_);
        program_compbuf_size(cur_.stream.bandwidth.compbuf_size_kb, true); // dcn32_program_outstanding_updates
        refclk_mhz_ = plan.dchub_refclk_mhz;
        const rx7900::display::Dcn32StreamClocks clocks = clocks_for(&plan);
        prepare_bandwidth(plan.bandwidth, clocks);
        update_dsc_pg(context, false);
        interdependent_update_lock(context, true);
        program_front_end_for_ctx(context, true);
        interdependent_update_lock(context, false);
        post_unlock_program_front_end(context);
        // Fire manual trigger only for the bottom plane (addr_update set by the FULL update).
        for (int i = 0; i < 4; ++i) {
            const PipeCtx& p = context.pipe[i];
            if (p.plane < 0 || p.bottom >= 0 || p.next_odm >= 0) {
                continue;
            }
            SET(reg("OTG%u_OTG_TRIGA_MANUAL_TRIG", static_cast<uint32_t>(p.tg)), 0,
                {{"OTG_TRIGA_MANUAL_TRIG", 1}}); // optc2_program_manual_trigger
        }
        cur_ = context;
        // dc_post_update_surfaces_to_stream: is_flip_pending_in_pipes (waited
        // here), disable_plane of free pipes (no HUBP: returns), optimize, DSC PG.
        wait_for_no_pipes_pending(cur_);
        optimize_bandwidth(plan.bandwidth, clocks);
        update_dsc_pg(cur_, true);
    }

    // commit_planes_do_stream_update(dpms_off)
    void dpms(bool on) {
        const int master = otg_master_of(cur_);
        if (!on) {
            link_set_dpms_off(cur_, master);
            dpms_off_ = true;
        } else {
            prepare_bandwidth(cur_.stream.bandwidth, clocks_for(&cur_.stream));
            link_set_dpms_on(cur_, master);
            dpms_off_ = false;
        }
    }

private:
    Model& model_;
    Hooks& hooks_;
    const Headers& h_;
    rx7900::hardware::RegisterIo io_{};
    rx7900::display::Dcn32StreamIo dsc_io_{};
    bool dsc_io_bound_ = false;
    rx7900::display::Dcn32Watermarks cache_[4]{};
    uint32_t det_[4]{};
    uint32_t compbuf_segments_ = 0;
    uint32_t refclk_mhz_ = 0;
    State cur_{};
    bool firmware_ = false;
    bool dpms_off_ = false;
    Mpcc mpcc_[4]{};
    int opp_list_[4]{-1, -1, -1, -1};
    bool disconnect_pending_[4][4]{};
    uint32_t opp_count_[4]{1, 1, 1, 1};
    bool power_gated_[4]{};
    Scl scl_[4]{};
    uint64_t request_address_[4]{};
};

} // namespace rx7900_test::stream

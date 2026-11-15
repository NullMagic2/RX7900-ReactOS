/*
Module: rx7900.tests.dcn32_stream
Purpose: Compare the DCN 3.2 stream owner access for access with the Linux v6.12 reference and test its recovery.
Created: 2026-10-24
Architecture: Host test. Plans come from the real Dcn32Bandwidth (DML32 port,
value-exact against Linux in dcn32_bandwidth_tests) for 640x480, 1920x1080@60
and 3840x2160@60 over DP (HBR2 x4), HDMI and single-link DVI. The owner and the
independent Linux transcription in tests/dcn32_stream_reference.hpp run on two
identical synthetic register models (tests/dcn32_stream_model.hpp); every read,
write, delay and link-owner hook call must match in order and value. After the
Linux sequence the owner may only read back, clear the two sticky underflow
bits and wait one frame. Failure paths (link refusal, register fault, status
timeout, underflow, stalled timing) are checked against the commit contract:
retry from a disabled pipe, then the previous plan, then blank, and quarantine
on anything unconfirmed. dcn-dsc-odm: ODM 2:1 (3840x2160@120 DP HBR3, the
dcn32 power policy) and a handcrafted ODM 4:1 context, DSC with FEC on HBR3
(3840x2160@144, ODM 2:1, 16 bpp) and on HBR2 x2 without ODM, DSC/ODM mode
changes and DPMS, and two-plane (MPO) commits plus plane add/move/alpha/remove
updates are compared the same way (the owner then clears one sticky underflow
bit per HUBP with a plane). This is not hardware evidence.
*/
#include <cstdio>
#include <cstring>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "asic_test_device.hpp"
#include "dcn32_bandwidth.hpp"
#include "dcn32_stream.hpp"
#include "dcn32_stream_model.hpp"
#include "dcn32_stream_reference.hpp"
#include "hdmi_stream.hpp"

namespace {

using namespace rx7900::display;
using namespace rx7900_test::stream;

rx7900_test::Checks checks;

void expect(bool condition, const char* contract) { checks.expect(condition, contract); }

Headers headers;
std::map<uint32_t, std::string> names;

void build_names() {
    for (const auto& entry : headers.values) {
        const std::string& key = entry.first;
        if (key.compare(0, 3, "reg") != 0 || key.size() < 10 || key.find("_BASE_IDX") != std::string::npos) {
            continue;
        }
        const auto base = headers.values.find(key + "_BASE_IDX");
        if (base == headers.values.end() || base->second > 3) {
            continue;
        }
        names[static_cast<uint32_t>((segment[base->second] + entry.second) * 4u)] = key.substr(3);
    }
}

std::string describe(const Op& op) {
    char text[160];
    if (op.kind == Kind::hook) {
        std::snprintf(text, sizeof(text), "H %08x = %u", op.offset, op.value);
    } else if (op.kind == Kind::delay) {
        std::snprintf(text, sizeof(text), "D %u us", op.value);
    } else {
        const auto name = names.find(op.offset);
        std::snprintf(text, sizeof(text), "%c %s = 0x%08x", static_cast<char>(op.kind),
                      name == names.end() ? "?" : name->second.c_str(), op.value);
    }
    return text;
}

// The owner's segment [owner_from, owner_from + reference.size()) must equal
// the reference segment; returns the owner index after it (SIZE_MAX on mismatch).
size_t same_sequence(const char* name, const std::vector<Op>& reference, size_t reference_from,
                     const std::vector<Op>& owner, size_t owner_from) {
    const size_t count = reference.size() - reference_from;
    for (size_t i = 0; i < count; ++i) {
        if (owner_from + i >= owner.size()) {
            std::fprintf(stderr, "%s: owner trace ends at %zu, Linux continues with %s\n", name, i,
                         describe(reference[reference_from + i]).c_str());
            return SIZE_MAX;
        }
        if (owner[owner_from + i] != reference[reference_from + i]) {
            std::fprintf(stderr, "%s: access %zu differs\n", name, i);
            for (size_t k = (i > 4 ? i - 4 : 0); k < i + 4 && k < count && owner_from + k < owner.size(); ++k) {
                std::fprintf(stderr, "  %s %-62s | owner %s\n", k == i ? ">" : " ",
                             describe(reference[reference_from + k]).c_str(), describe(owner[owner_from + k]).c_str());
            }
            return SIZE_MAX;
        }
    }
    return owner_from + count;
}

// After the Linux sequence: reads only, the sticky-underflow clears (the
// OTG's and one per HUBP in hubp_mask; 0 = the OTG's own pipe) and one frame
// wait (the owner's readback and settle check).
bool owner_suffix_only_checks(const std::vector<Op>& owner, size_t from, uint32_t pipe, uint32_t hubp_mask = 0) {
    const uint32_t optc = address(r::OPTC_INPUT_GLOBAL_CONTROL::reg, pipe);
    if (hubp_mask == 0) {
        hubp_mask = 1u << pipe;
    }
    size_t hubps = 0;
    for (uint32_t i = 0; i < 4; ++i) {
        hubps += (hubp_mask >> i) & 1u;
    }
    auto is_hubp = [&](uint32_t offset) {
        for (uint32_t i = 0; i < 4; ++i) {
            if (((hubp_mask >> i) & 1u) != 0 && offset == address(r::DCHUBP_CNTL::reg, i)) {
                return true;
            }
        }
        return false;
    };
    size_t writes = 0;
    size_t delays = 0;
    for (size_t i = from; i < owner.size(); ++i) {
        const Op& op = owner[i];
        if (op.kind == Kind::write) {
            if (op.offset != optc && !is_hubp(op.offset)) {
                std::fprintf(stderr, "unexpected owner write after the Linux sequence: %s\n", describe(op).c_str());
                return false;
            }
            ++writes;
        } else if (op.kind == Kind::delay) {
            ++delays;
        } else if (op.kind == Kind::hook) {
            std::fprintf(stderr, "unexpected hook after the Linux sequence: %s\n", describe(op).c_str());
            return false;
        }
    }
    return writes == 1 + hubps && delays == 1;
}

// ---- plans ----

Dcn32SocInputs table1() {
    Dcn32SocInputs in{};
    in.clocks.levels[0] = {506, 400, 96, 300, 600, 1950, 1950, 810};
    in.clocks.levels[1] = {1100, 1000, 456, 600, 0, 0, 0, 0};
    in.clocks.levels[2] = {1300, 1600, 772, 1000, 0, 0, 0, 0};
    in.clocks.levels[3] = {1564, 2400, 1249, 1200, 0, 0, 0, 0};
    in.clocks.num_dcfclk_levels = 4;
    in.clocks.num_fclk_levels = 4;
    in.clocks.num_memclk_levels = 4;
    in.clocks.num_socclk_levels = 4;
    in.clocks.num_dtbclk_levels = 1;
    in.clocks.num_dispclk_levels = 1;
    in.clocks.num_dppclk_levels = 1;
    in.clocks.dc_mode_limit = {1564, 2400, 1249, 1200, 600, 1950, 1950, 0};
    in.num_chans = 24;
    in.dram_channel_width_bytes = 2;
    in.dentist_vco_khz = 4300000;
    in.dchub_refclk_khz = 100000;
    return in;
}

struct Bandwidth final {
    std::vector<unsigned char> storage = std::vector<unsigned char>(dcn32_bandwidth_workspace_bytes() + 16);
    Dcn32BandwidthWorkspace* workspace = nullptr;
    Dcn32Bandwidth bandwidth;
    Bandwidth() {
        void* raw = storage.data();
        while (reinterpret_cast<uintptr_t>(raw) % 8 != 0) {
            raw = static_cast<unsigned char*>(raw) + 1;
        }
        workspace = construct_dcn32_bandwidth_workspace(raw, dcn32_bandwidth_workspace_bytes());
        bandwidth.configure(*workspace, table1());
    }
};

constexpr uint32_t sync = positive_hsync | positive_vsync;
constexpr Timing mode_480{25175, 640, 800, 16, 96, 480, 525, 10, 2, 0};
constexpr Timing mode_1080{148500, 1920, 2200, 88, 44, 1080, 1125, 4, 5, sync};
constexpr Timing mode_2160{594000, 3840, 4400, 176, 88, 2160, 2250, 8, 10, sync};
constexpr uint64_t surface_base = 0x0000008012340000ull;
constexpr uint32_t max_cab = 64u * 1024u * 1024u;

enum class Sig { dp, hdmi, dvi };

HdmiSinkInfo hdmi2_sink_info() {
    HdmiSinkInfo sink{};
    sink.is_hdmi = true;
    sink.has_hdmi_infoframe = true;
    sink.max_tmds_clock_khz = 600000;
    sink.scdc_supported = true;
    sink.scrambling_supported = true;
    sink.cea_revision = 3;
    return sink;
}

Dcn32StreamRequest request_for(Sig sig, const Timing& t, uint32_t pipe, uint32_t engine, uint64_t address) {
    Dcn32StreamRequest q{};
    q.timing = t;
    q.signal = sig == Sig::dp ? StreamSignal::display_port : (sig == Sig::hdmi ? StreamSignal::hdmi : StreamSignal::dvi_single);
    q.pipe = pipe;
    q.engine = engine;
    q.surface_address = address;
    q.pitch_pixels = (t.width + 63u) & ~63u;
    q.tiling = {32, 256, 1, 8};
    q.dchub_refclk_khz = 100000;
    q.max_cab_allocation_bytes = max_cab;
    if (sig == Sig::dp) {
        q.dp_rate_code = 0x14;
        q.dp_lanes = 4;
        q.dp_use_vsc = t.width == 1920;
    } else {
        HdmiSinkInfo sink = sig == Sig::hdmi ? hdmi2_sink_info() : HdmiSinkInfo{};
        plan_tmds_stream(t, sink, q.tmds);
    }
    return q;
}

bool make_plan(Bandwidth& bw, Sig sig, const Timing& t, uint32_t pipe, uint32_t engine, uint64_t address,
               Dcn32StreamPlan& plan) {
    const Dcn32StreamRequest q = request_for(sig, t, pipe, engine, address);
    Dcn32PlaneRequest pr{};
    pr.timing = t;
    pr.signal = sig == Sig::dp ? Dcn32OutputSignal::display_port : Dcn32OutputSignal::tmds;
    pr.pitch_pixels = q.pitch_pixels;
    pr.otg_instance = pipe;
    Dcn32BandwidthPlan bandwidth{};
    if (bw.bandwidth.validate(pr, bandwidth) != Dcn32BandwidthStatus::ready) {
        return false;
    }
    return build_dcn32_stream_plan(q, bandwidth, plan) == StreamStatus::success;
}

// ---- rigs ----

std::map<uint32_t, uint32_t> reset_state() {
    Model m;
    m.set(r::DENTIST_DISPCLK_CNTL::reg, 0, 0);
    m.set_field(r::DENTIST_DISPCLK_CNTL::reg, 0, r::DENTIST_DISPCLK_CNTL::DENTIST_DISPCLK_RDIVIDER, 0x2c);
    for (uint32_t i = 0; i < 4; ++i) {
        m.set(r::MPCC_TOP_SEL::reg, i, 0xf);
        m.set(r::MPC_OUT_MUX::reg, i, 0xf);
    }
    m.set_field(r::DCHUBBUB_DET0_CTRL::reg, 0, r::DCHUBBUB_DET0_CTRL::DET0_SIZE, 4);
    m.set_field(r::DCHUBBUB_COMPBUF_CTRL::reg, 0, r::DCHUBBUB_COMPBUF_CTRL::COMPBUF_SIZE, 4);
    return m.regs;
}

// What the GOP leaves lit on pipe 0 / DIG A.
std::map<uint32_t, uint32_t> firmware_state(Sig sig) {
    Model m;
    m.regs = reset_state();
    m.set_field(r::OTG_CONTROL::reg, 0, r::OTG_CONTROL::OTG_MASTER_EN, 1);
    m.set_field(r::VTG_CONTROL::reg, 0, r::VTG_CONTROL::VTG0_ENABLE, 1);
    m.set_field(r::OTG_CLOCK_CONTROL::reg, 0, r::OTG_CLOCK_CONTROL::OTG_CLOCK_EN, 1);
    m.set_field(r::OPTC_INPUT_CLOCK_CONTROL::reg, 0, r::OPTC_INPUT_CLOCK_CONTROL::OPTC_INPUT_CLK_EN, 1);
    m.set_field(r::OPTC_DATA_SOURCE_SELECT::reg, 0, r::OPTC_DATA_SOURCE_SELECT::OPTC_SEG0_SRC_SEL, 0);
    m.set(r::MPCC_TOP_SEL::reg, 0, 0);
    m.set(r::MPCC_OPP_ID::reg, 0, 0);
    m.set(r::MPC_OUT_MUX::reg, 0, 0);
    m.set_field(r::HUBP_CLK_CNTL::reg, 0, r::HUBP_CLK_CNTL::HUBP_CLOCK_ENABLE, 1);
    m.set_field(r::DPP_CONTROL::reg, 0, r::DPP_CONTROL::DPP_CLOCK_ENABLE, 1);
    m.set(r::DCSURF_PRIMARY_SURFACE_ADDRESS::reg, 0, 0x00100000);
    m.set_field(r::DIG_FE_CNTL::reg, 0, r::DIG_FE_CNTL::DIG_SOURCE_SELECT, 0);
    m.set_field(r::DIG_FE_CNTL::reg, 0, r::DIG_FE_CNTL::DIG_SYMCLK_FE_ON, 1);
    for (uint32_t d = 0; d < 8; ++d) {
        m.set_field(r::DOMAIN_PG_CONFIG::reg, d, r::DOMAIN_PG_CONFIG::DOMAIN_POWER_FORCEON, 1);
    }
    if (sig == Sig::dp) {
        m.set_field(r::DP_VID_STREAM_CNTL::reg, 0, r::DP_VID_STREAM_CNTL::DP_VID_STREAM_ENABLE, 1);
    }
    m.set(r::DCHUBBUB_ARB_DATA_URGENCY_WATERMARK_A::reg, 0, 0x123);
    m.set(r::DCHUBBUB_ARB_ALLOW_SR_EXIT_WATERMARK_A::reg, 0, 0x456);
    return m.regs;
}

struct OwnerRig final {
    Model model;
    Hooks hooks;
    Dcn32StreamController controller;
    explicit OwnerRig(const std::map<uint32_t, uint32_t>& regs) {
        model.regs = regs;
        hooks.model = &model;
        controller.bind(model.io(), segment[1], segment[2], segment[3], hooks.table());
    }
};

struct ReferenceRig final {
    Model model;
    Hooks hooks;
    Reference linux_ref;
    explicit ReferenceRig(const std::map<uint32_t, uint32_t>& regs) : linux_ref(model, hooks, headers) {
        model.regs = regs;
        hooks.model = &model;
    }
};

Dcn32FirmwarePipe firmware_pipe(Sig sig, const Timing& t) {
    Dcn32FirmwarePipe fw{};
    fw.timing = t;
    fw.signal = sig == Sig::dp ? StreamSignal::display_port : (sig == Sig::hdmi ? StreamSignal::hdmi : StreamSignal::dvi_single);
    fw.pipe = 0;
    fw.engine = 0;
    return fw;
}

Dcn32StreamPlan firmware_plan(const Dcn32FirmwarePipe& fw) {
    Dcn32StreamPlan p{};
    p.timing = fw.timing;
    p.signal = fw.signal;
    p.pipe = fw.pipe;
    p.engine = fw.engine;
    return p;
}

// One step: owner action vs reference action on synchronized models.
struct Pair final {
    std::unique_ptr<OwnerRig> owner;
    std::unique_ptr<ReferenceRig> ref;
    // The pipes with a plane after the step (0: the OTG's pipe only).
    uint32_t hubp_mask = 0;
    explicit Pair(const std::map<uint32_t, uint32_t>& regs)
        : owner(std::make_unique<OwnerRig>(regs)), ref(std::make_unique<ReferenceRig>(regs)) {}

    // Runs both, compares, then hands the owner's register state (including
    // its readback-only suffix effects) to the reference model.
    template <typename OwnerStep, typename ReferenceStep>
    bool step(const char* name, OwnerStep owner_step, ReferenceStep reference_step, bool owner_checks, uint32_t pipe) {
        const size_t owner_from = owner->model.trace.size();
        const size_t ref_from = ref->model.trace.size();
        owner_step(*owner);
        reference_step(*ref);
        const size_t end = same_sequence(name, ref->model.trace, ref_from, owner->model.trace, owner_from);
        bool ok = end != SIZE_MAX;
        if (ok && owner_checks) {
            ok = owner_suffix_only_checks(owner->model.trace, end, pipe, hubp_mask);
        } else if (ok) {
            ok = end == owner->model.trace.size();
            if (!ok) {
                std::fprintf(stderr, "%s: owner has %zu accesses beyond the Linux sequence, first %s\n", name,
                             owner->model.trace.size() - end, describe(owner->model.trace[end]).c_str());
            }
        }
        ref->model.regs = owner->model.regs;
        return ok;
    }
};

// ---- sequence tests ----

void fresh_commits(Bandwidth& bw) {
    struct Case {
        const char* name;
        Sig sig;
        Timing timing;
        uint32_t pipe;
        uint32_t engine;
    };
    const Case cases[] = {
        {"DP 640x480 commit is Linux-exact", Sig::dp, mode_480, 0, 0},
        {"DP 1920x1080@60 commit is Linux-exact", Sig::dp, mode_1080, 1, 2},
        {"DP 3840x2160@60 commit is Linux-exact", Sig::dp, mode_2160, 3, 4},
        {"HDMI 640x480 commit is Linux-exact", Sig::hdmi, mode_480, 0, 1},
        {"HDMI 1920x1080@60 commit is Linux-exact", Sig::hdmi, mode_1080, 2, 3},
        {"HDMI 3840x2160@60 (scrambled) commit is Linux-exact", Sig::hdmi, mode_2160, 1, 0},
        {"DVI 1920x1080@60 commit is Linux-exact", Sig::dvi, mode_1080, 2, 2},
    };
    for (const Case& c : cases) {
        Dcn32StreamPlan plan{};
        expect(make_plan(bw, c.sig, c.timing, c.pipe, c.engine, surface_base, plan), "plan builds");
        Pair pair(reset_state());
        StreamResult result = StreamResult::unchanged;
        const bool same = pair.step(c.name, [&](OwnerRig& o) { result = o.controller.commit(plan); },
                                    [&](ReferenceRig& r) {
                                        r.linux_ref.init_hw_subset();
                                        r.linux_ref.commit(&plan);
                                    },
                                    true, c.pipe);
        expect(same, c.name);
        expect(result == StreamResult::committed && pair.owner->controller.report().attempts == 1, "committed first time");
    }
}

// Boundaries the regular modes do not reach: a DENTIST read divider of 0 (no
// WDIVIDER resync write) and a VSTARTUP exactly at the HUBP VREADY limit.
void edge_commits(Bandwidth& bw) {
    Dcn32StreamPlan plan{};
    expect(make_plan(bw, Sig::dp, mode_1080, 0, 0, surface_base, plan), "edge plan builds");
    std::map<uint32_t, uint32_t> regs = reset_state();
    {
        Model m;
        m.regs = regs;
        m.set(r::DENTIST_DISPCLK_CNTL::reg, 0, 0);
        regs = m.regs;
    }
    const auto& bwp = plan.bandwidth;
    plan.bandwidth.vstartup_start =
        plan.vblank_end + (bwp.vready_offset + bwp.vupdate_width + bwp.vupdate_offset) / plan.timing.h_total;
    // The per-pipe copy of the OTG master's pipe_dlg_param (dcn-dsc-odm).
    plan.bandwidth.pipes[0].vstartup_start = plan.bandwidth.vstartup_start;
    Pair pair(regs);
    const bool same = pair.step("edge commit", [&](OwnerRig& o) { o.controller.commit(plan); },
                                [&](ReferenceRig& r) {
                                    r.linux_ref.init_hw_subset();
                                    r.linux_ref.commit(&plan);
                                },
                                true, 0);
    expect(same, "RDIVIDER 0 and VREADY at its limit are Linux-exact");
    expect(pair.owner->model.field(r::DCHUBP_CNTL::reg, 0, r::DCHUBP_CNTL::HUBP_VREADY_AT_OR_AFTER_VSYNC) == 1,
           "VSTARTUP at the limit sets HUBP_VREADY_AT_OR_AFTER_VSYNC");
}

void mode_changes(Bandwidth& bw) {
    for (Sig sig : {Sig::dp, Sig::hdmi}) {
        Dcn32StreamPlan a{};
        Dcn32StreamPlan b{};
        Dcn32StreamPlan c{};
        expect(make_plan(bw, sig, mode_1080, 1, 1, surface_base, a) &&
                   make_plan(bw, sig, mode_480, 1, 1, surface_base + 0x2000000, b) &&
                   make_plan(bw, sig, mode_2160, 1, 1, surface_base + 0x4000000, c),
               "mode change plans build");
        Pair pair(reset_state());
        bool same = pair.step("first mode", [&](OwnerRig& o) { o.controller.commit(a); },
                              [&](ReferenceRig& r) {
                                  r.linux_ref.init_hw_subset();
                                  r.linux_ref.commit(&a);
                              },
                              true, 1);
        same = same && pair.step("1080p -> 640x480", [&](OwnerRig& o) { o.controller.commit(b); },
                                 [&](ReferenceRig& r) { r.linux_ref.commit(&b); }, true, 1);
        same = same && pair.step("640x480 -> 2160p60", [&](OwnerRig& o) { o.controller.commit(c); },
                                 [&](ReferenceRig& r) { r.linux_ref.commit(&c); }, true, 1);
        same = same && pair.step("disable", [&](OwnerRig& o) { o.controller.disable(); },
                                 [&](ReferenceRig& r) { r.linux_ref.commit(nullptr); }, false, 1);
        expect(same, sig == Sig::dp ? "DP mode changes and disable are Linux-exact"
                                    : "HDMI mode changes and disable are Linux-exact");
        expect(!pair.owner->controller.has_stream() &&
                   pair.owner->model.field(r::OTG_CONTROL::reg, 1, r::OTG_CONTROL::OTG_MASTER_EN) == 0 &&
                   pair.owner->model.field(r::DOMAIN_PG_STATUS::reg, 1, r::DOMAIN_PG_STATUS::DOMAIN_PGFSM_PWR_STATUS) == 2,
               "disable leaves the OTG off and the HUBP power-gated");
    }
}

void takeover(Bandwidth& bw) {
    for (Sig sig : {Sig::dp, Sig::hdmi}) {
        const Dcn32FirmwarePipe fw = firmware_pipe(sig, mode_1080);
        Dcn32StreamPlan plan{};
        expect(make_plan(bw, sig, mode_2160, 0, 0, surface_base, plan), "takeover plan builds");
        Pair pair(firmware_state(sig));
        expect(pair.owner->controller.adopt_firmware(fw) && pair.owner->controller.firmware_current(), "adopt");
        const bool same = pair.step(
            "GOP takeover", [&](OwnerRig& o) { o.controller.commit(plan); },
            [&](ReferenceRig& r) {
                r.linux_ref.init_hw_subset();
                r.linux_ref.adopt(firmware_plan(fw));
                r.linux_ref.commit(&plan);
            },
            true, 0);
        expect(same, sig == Sig::dp ? "DP GOP takeover (init subset + disable + enable) is Linux-exact"
                                    : "HDMI GOP takeover is Linux-exact");
        expect(!pair.owner->controller.firmware_current() && pair.owner->controller.current() != nullptr,
               "the owned plan replaces the firmware state");

        // Disable straight from the firmware state.
        Pair off(firmware_state(sig));
        off.owner->controller.adopt_firmware(fw);
        const bool same_off = off.step(
            "firmware disable", [&](OwnerRig& o) { o.controller.disable(); },
            [&](ReferenceRig& r) {
                r.linux_ref.init_hw_subset();
                r.linux_ref.adopt(firmware_plan(fw));
                r.linux_ref.commit(nullptr);
            },
            false, 0);
        expect(same_off, "disabling the firmware pipe is Linux-exact");
    }
}

void dpms(Bandwidth& bw) {
    for (Sig sig : {Sig::dp, Sig::hdmi}) {
        Dcn32StreamPlan plan{};
        expect(make_plan(bw, sig, mode_1080, 2, 1, surface_base, plan), "dpms plan builds");
        Pair pair(reset_state());
        bool same = pair.step("commit", [&](OwnerRig& o) { o.controller.commit(plan); },
                              [&](ReferenceRig& r) {
                                  r.linux_ref.init_hw_subset();
                                  r.linux_ref.commit(&plan);
                              },
                              true, 2);
        same = same && pair.step("dpms off", [&](OwnerRig& o) { o.controller.set_dpms(false); },
                                 [&](ReferenceRig& r) { r.linux_ref.dpms(false); }, false, 2);
        expect(!pair.owner->controller.dpms_on(), "dpms off recorded");
        same = same && pair.step("dpms on", [&](OwnerRig& o) { o.controller.set_dpms(true); },
                                 [&](ReferenceRig& r) { r.linux_ref.dpms(true); }, false, 2);
        same = same && pair.step("dpms off again", [&](OwnerRig& o) { o.controller.set_dpms(false); },
                                 [&](ReferenceRig& r) { r.linux_ref.dpms(false); }, false, 2);
        // The back end of a DPMS-off stream is reset without a second link_set_dpms_off.
        same = same && pair.step("disable while off", [&](OwnerRig& o) { o.controller.disable(); },
                                 [&](ReferenceRig& r) { r.linux_ref.commit(nullptr); }, false, 2);
        expect(same, sig == Sig::dp ? "DP DPMS off/on/off and disable are Linux-exact"
                                    : "HDMI DPMS off/on/off and disable are Linux-exact");
    }
}

// ---- plan construction ----

void plan_rules(Bandwidth& bw) {
    Dcn32StreamPlan p{};
    expect(make_plan(bw, Sig::dp, mode_480, 0, 0, surface_base, p) && p.pixel_rate_k1 == 0 && p.pixel_rate_k2 == 1 &&
               p.requested_pixel_100hz == 251750 / 2 && p.dio_pix_per_cycle == 2 && p.mall_sel == 2 &&
               p.vblank_end == 525 - 10 - 480 && p.dchub_refclk_mhz == 100 && !p.vsc.valid,
           "DP 640x480: divisible horizontal timing takes the 2-pixel DIG path, MALL for static screen");
    expect(make_plan(bw, Sig::dp, mode_1080, 0, 0, surface_base, p) && p.vsc.valid && p.vsc.hb2 == 5 &&
               p.requested_pixel_100hz == 742500,
           "DP 1080p60 with VSC colorimetry");
    expect(make_plan(bw, Sig::hdmi, mode_2160, 0, 0, surface_base, p) && p.pixel_rate_k2 == 3 &&
               p.requested_pixel_100hz == 5940000 && p.dio_pix_per_cycle == 1 && p.tmds_khz == 594000 &&
               p.avi.valid && p.avi.sb[4] == 97 && !p.vendor.valid,
           "HDMI 2160p60: divide by 4, full pixel clock, AVI VIC 97");
    expect(dcn32_mall_ways_from_bytes(33177600, max_cab) == 8 && dcn32_mall_ways_from_bytes(33177600, 16u << 20) == 32 &&
               dcn32_mall_ways_from_bytes(0, max_cab) == 0 && dcn32_mall_ways_from_bytes(1, 64) == UINT32_MAX,
           "dcn32_calculate_mall_ways_from_bytes");
    Timing odd = mode_1080;
    odd.h_sync_width = 45;
    expect(!dcn32_dp_dig_pixel_rate_div_policy(odd) && dcn32_dp_dig_pixel_rate_div_policy(mode_2160),
           "is_h_timing_divisible_by_2");

    Dcn32BandwidthPlan bandwidth{};
    Dcn32PlaneRequest pr{mode_2160, Dcn32OutputSignal::display_port, 3840, 0};
    bw.bandwidth.validate(pr, bandwidth);
    Dcn32StreamRequest q = request_for(Sig::dp, mode_2160, 0, 0, surface_base);
    q.max_cab_allocation_bytes = 16u << 20;
    expect(build_dcn32_stream_plan(q, bandwidth, p) == StreamStatus::success && p.mall_sel == 0,
           "a surface beyond 16 MALL ways is not cached");
    q.max_cab_allocation_bytes = 32u << 20;
    expect(build_dcn32_stream_plan(q, bandwidth, p) == StreamStatus::success && p.mall_sel == 2,
           "exactly 16 MALL ways are cached");
    q.max_cab_allocation_bytes = max_cab;
    expect(build_dcn32_stream_plan(q, bandwidth, p) == StreamStatus::success && p.mall_sel == 2,
           "2160p60 needs 8 of 16 ways of a 64 MiB CAB");
    struct Bad {
        const char* name;
        void (*mutate)(Dcn32StreamRequest&);
        StreamStatus status;
    };
    const Bad bad[] = {
        {"pitch below width", [](Dcn32StreamRequest& r) { r.pitch_pixels = 3776; }, StreamStatus::invalid_surface},
        {"pitch not 64-pixel aligned", [](Dcn32StreamRequest& r) { r.pitch_pixels = 3904 - 32; }, StreamStatus::invalid_surface},
        {"unaligned surface", [](Dcn32StreamRequest& r) { r.surface_address += 0x80; }, StreamStatus::invalid_surface},
        {"surface above 48 bits", [](Dcn32StreamRequest& r) { r.surface_address = 1ull << 48; }, StreamStatus::invalid_surface},
        {"no surface", [](Dcn32StreamRequest& r) { r.surface_address = 0; }, StreamStatus::invalid_surface},
        {"pipe 4", [](Dcn32StreamRequest& r) { r.pipe = 4; }, StreamStatus::invalid_argument},
        {"engine 5", [](Dcn32StreamRequest& r) { r.engine = 5; }, StreamStatus::invalid_argument},
        {"no DP rate", [](Dcn32StreamRequest& r) { r.dp_rate_code = 0; }, StreamStatus::invalid_argument},
        {"TMDS plan on a DP stream is ignored but HDMI signal without TMDS plan is refused",
         [](Dcn32StreamRequest& r) { r.signal = StreamSignal::hdmi; }, StreamStatus::invalid_argument},
        {"no DCHUB reference clock", [](Dcn32StreamRequest& r) { r.dchub_refclk_khz = 0; }, StreamStatus::invalid_argument},
        {"no CAB size", [](Dcn32StreamRequest& r) { r.max_cab_allocation_bytes = 0; }, StreamStatus::invalid_argument},
        {"zero tiling pipes", [](Dcn32StreamRequest& r) { r.tiling.num_pipes = 0; }, StreamStatus::invalid_argument},
    };
    for (const Bad& b : bad) {
        Dcn32StreamRequest broken = request_for(Sig::dp, mode_2160, 0, 0, surface_base);
        b.mutate(broken);
        Dcn32StreamPlan untouched{};
        untouched.pipe = 77;
        expect(build_dcn32_stream_plan(broken, bandwidth, untouched) == b.status && untouched.pipe == 77, b.name);
    }
    Dcn32BandwidthPlan no_vstartup = bandwidth;
    no_vstartup.vstartup_start = 0;
    expect(build_dcn32_stream_plan(request_for(Sig::dp, mode_2160, 0, 0, surface_base), no_vstartup, p) ==
               StreamStatus::unsupported,
           "a plan without VSTARTUP is unsupported");
}

// ---- failure and recovery ----

uint32_t otg_enabled(OwnerRig& rig, uint32_t pipe) {
    return rig.model.field(r::OTG_CONTROL::reg, pipe, r::OTG_CONTROL::OTG_MASTER_EN);
}

void failures(Bandwidth& bw) {
    Dcn32StreamPlan a{};
    Dcn32StreamPlan b{};
    expect(make_plan(bw, Sig::dp, mode_1080, 0, 0, surface_base, a) &&
               make_plan(bw, Sig::dp, mode_2160, 0, 0, surface_base + 0x4000000, b),
           "failure plans build");

    {   // A refused link on the first attempt: retry from a disabled pipe wins.
        OwnerRig rig(reset_state());
        rig.hooks.enable_script = {LinkCall::refused};
        const StreamResult result = rig.controller.commit(a);
        const StreamReport& report = rig.controller.report();
        expect(result == StreamResult::committed && report.attempts == 2 &&
                   report.first_failure == StreamFailure::link_refused && rig.hooks.enable_calls == 2 &&
                   otg_enabled(rig, 0) == 1 && rig.controller.current() != nullptr,
               "link refusal: one retry from a disabled pipe commits");
    }
    {   // Refused twice with a previous own plan: the previous plan is restored.
        OwnerRig rig(reset_state());
        expect(rig.controller.commit(a) == StreamResult::committed, "baseline commit");
        rig.hooks.enable_script = {LinkCall::success, LinkCall::refused, LinkCall::refused};
        const StreamResult result = rig.controller.commit(b);
        expect(result == StreamResult::restored && rig.controller.report().failure == StreamFailure::link_refused &&
                   rig.controller.current() != nullptr && rig.controller.current()->timing.width == 1920 &&
                   rig.model.field(r::DCSURF_PRIMARY_SURFACE_ADDRESS::reg, 0,
                                   r::DCSURF_PRIMARY_SURFACE_ADDRESS::PRIMARY_SURFACE_ADDRESS) ==
                       static_cast<uint32_t>(surface_base) &&
                   otg_enabled(rig, 0) == 1,
               "two link refusals restore the previous mode and surface");
    }
    {   // Always refused, nothing to restore: blank.
        OwnerRig rig(reset_state());
        rig.hooks.enable_script = {LinkCall::refused, LinkCall::refused};
        const StreamResult result = rig.controller.commit(a);
        // The refusal comes before the plane is enabled, so no plane is ever connected.
        expect(result == StreamResult::blanked && !rig.controller.has_stream() && otg_enabled(rig, 0) == 0 &&
                   rig.model.field(r::OTG_CLOCK_CONTROL::reg, 0, r::OTG_CLOCK_CONTROL::OTG_CLOCK_EN) == 0 &&
                   rig.model.field(r::MPC_OUT_MUX::reg, 0, r::MPC_OUT_MUX::MPC_OUT_MUX) == 0xf &&
                   rig.model.field(r::HUBP_CLK_CNTL::reg, 0, r::HUBP_CLK_CNTL::HUBP_CLOCK_ENABLE) == 0 &&
                   rig.hooks.disable_calls == 2 && !rig.controller.quarantined(),
               "unrecoverable refusal ends blanked: OTG and its clock off, no plane connected, link off");
        expect(rig.controller.commit(a) == StreamResult::committed, "a blanked owner commits again");
    }
    {   // An unconfirmed write quarantines until an owner-proven reset.
        OwnerRig rig(reset_state());
        rig.model.fail_write_at = 40;
        expect(rig.controller.commit(a) == StreamResult::uncertain && rig.controller.quarantined() &&
                   rig.controller.report().failure == StreamFailure::register_fault,
               "a write fault is uncertain and quarantines");
        const size_t before = rig.model.trace.size();
        expect(rig.controller.commit(a) == StreamResult::uncertain && rig.model.trace.size() == before &&
                   rig.controller.disable() == StreamResult::uncertain && rig.model.trace.size() == before,
               "a quarantined owner touches no register");
        rig.model.fail_write_at = SIZE_MAX;
        rig.controller.reset_proven();
        expect(!rig.controller.quarantined() && rig.controller.commit(a) == StreamResult::committed,
               "after reset_proven the owner commits from scratch");
    }
    {   // An uncertain link call quarantines without retry.
        OwnerRig rig(reset_state());
        rig.hooks.enable_script = {LinkCall::uncertain};
        expect(rig.controller.commit(a) == StreamResult::uncertain &&
                   rig.controller.report().failure == StreamFailure::hook_uncertain && rig.hooks.enable_calls == 1,
               "an uncertain link enable quarantines immediately");
    }
    {   // A status that never comes: bounded wait, retry, then blank.
        OwnerRig rig(reset_state());
        rig.model.otg_clock_never_on = true;
        const StreamResult result = rig.controller.commit(a);
        expect(result == StreamResult::blanked && rig.controller.report().first_failure == StreamFailure::register_timeout &&
                   rig.controller.report().attempts == 2,
               "an OTG clock that never turns on times out, retries once and blanks");
        rig.model.otg_clock_never_on = false;
        expect(rig.controller.commit(a) == StreamResult::committed, "the timeout is not sticky");
    }
    {   // Persistent underflow after enable.
        OwnerRig rig(reset_state());
        rig.model.underflow_after_enable = true;
        expect(rig.controller.commit(a) == StreamResult::blanked &&
                   rig.controller.report().first_failure == StreamFailure::underflow,
               "an underflowing stream blanks after one retry");
    }
    {   // A frame counter that does not move.
        OwnerRig rig(reset_state());
        rig.model.frame_frozen = true;
        expect(rig.controller.commit(a) == StreamResult::blanked &&
                   rig.controller.report().first_failure == StreamFailure::timing_stalled,
               "a stalled OTG blanks");
    }
    {   // Health check and recovery of a live stream.
        OwnerRig rig(reset_state());
        expect(rig.controller.commit(a) == StreamResult::committed, "commit before health");
        StreamHealth health{};
        expect(rig.controller.check_health(health) && health.active && health.frame_counter_moving &&
                   !health.optc_underflow && !health.dpg_blanked,
               "a healthy stream reports a moving frame counter");
        rig.model.set_field(r::OPTC_INPUT_GLOBAL_CONTROL::reg, 0,
                            r::OPTC_INPUT_GLOBAL_CONTROL::OPTC_UNDERFLOW_OCCURRED_STATUS, 1);
        expect(rig.controller.check_health(health) && health.optc_underflow &&
                   rig.model.field(r::OPTC_INPUT_GLOBAL_CONTROL::reg, 0,
                                   r::OPTC_INPUT_GLOBAL_CONTROL::OPTC_UNDERFLOW_OCCURRED_STATUS) == 0,
               "an underflow is reported and its sticky bit cleared");
        expect(rig.controller.recover() == StreamResult::committed && rig.controller.report().recoveries == 0,
               "recover re-commits the current plan");
        expect(rig.controller.set_dpms(false) == StreamResult::committed && !rig.controller.check_health(health),
               "no health check while DPMS is off");
    }
    {   // Invalid requests change nothing.
        OwnerRig rig(reset_state());
        Dcn32StreamPlan other = a;
        other.pipe = 2;
        expect(rig.controller.commit(a) == StreamResult::committed, "commit pipe 0");
        const size_t before = rig.model.trace.size();
        expect(rig.controller.commit(other) == StreamResult::unchanged &&
                   rig.controller.report().failure == StreamFailure::invalid_plan && rig.model.trace.size() == before,
               "a plan for another pipe while one is owned is refused before any access");
        Dcn32StreamPlan no_surface = a;
        no_surface.surface_address = 0;
        expect(rig.controller.commit(no_surface) == StreamResult::unchanged && rig.model.trace.size() == before,
               "a plan without surface is refused before any access");
    }
}

} // namespace

// ---- dcn-restore-modes: the Linux steps the first port deferred ----

size_t hook_count(const Model& m, size_t from, uint32_t id) {
    size_t n = 0;
    for (size_t i = from; i < m.trace.size(); ++i) {
        n += (m.trace[i].kind == Kind::hook && m.trace[i].offset == id) ? 1u : 0u;
    }
    return n;
}

size_t first_index(const Model& m, size_t from, Kind kind, uint32_t offset) {
    for (size_t i = from; i < m.trace.size(); ++i) {
        if (m.trace[i].kind == kind && m.trace[i].offset == offset) {
            return i;
        }
    }
    return SIZE_MAX;
}

void linux_steps(Bandwidth& bw) {
    // dcn32_init_hw: dccg31_init and the DCN clock gating of the takeover.
    {
        Dcn32StreamPlan plan{};
        expect(make_plan(bw, Sig::dp, mode_1080, 0, 0, surface_base, plan), "gating plan builds");
        std::map<uint32_t, uint32_t> regs = reset_state();
        Model m;
        m.regs = regs;
        m.set(r::SYMCLK32_SE_CNTL::reg, 0, 0x0000999au);
        m.set(r::DCCG_GATE_DISABLE_CNTL::reg, 0, 0xffffffffu);
        m.set(r::DCCG_GATE_DISABLE_CNTL2::reg, 0, 0xffffffffu);
        m.set(r::DCFCLK_CNTL::reg, 0, 0x80000ff5u);
        regs = m.regs;
        Pair pair(regs);
        const bool same = pair.step("gated commit", [&](OwnerRig& o) { o.controller.commit(plan); },
                                    [&](ReferenceRig& r) {
                                        r.linux_ref.init_hw_subset();
                                        r.linux_ref.commit(&plan);
                                    },
                                    true, 0);
        Model& o = pair.owner->model;
        const size_t symclk = first_index(o, 0, Kind::write, address(r::SYMCLK32_SE_CNTL::reg, 0));
        const size_t ip_request = first_index(o, 0, Kind::read, address(r::DC_IP_REQUEST_CNTL::reg, 0));
        const size_t dio = first_index(o, 0, Kind::write, address(r::DIO_MEM_PWR_CTRL::reg, 0));
        const size_t gate = first_index(o, 0, Kind::write, address(r::DCCG_GATE_DISABLE_CNTL::reg, 0));
        const size_t dcfclk = first_index(o, 0, Kind::write, address(r::DCFCLK_CNTL::reg, 0));
        const size_t watermark = first_index(o, 0, Kind::write, address(r::DCHUBBUB_ARB_DATA_URGENCY_WATERMARK_B::reg, 0));
        expect(same && o.count(Kind::write, address(r::SYMCLK32_SE_CNTL::reg, 0)) == 4 && symclk < ip_request &&
                   dio < gate && gate < dcfclk && dcfclk < watermark,
               "dccg31_init precedes the power-gating plane; clock gating follows DIO_MEM_PWR_CTRL, before init_watermarks");
        expect(o.raw(r::SYMCLK32_SE_CNTL::reg, 0) == 0 && o.raw(r::DCCG_GATE_DISABLE_CNTL::reg, 0) == 0 &&
                   o.raw(r::DCCG_GATE_DISABLE_CNTL2::reg, 0) == 0 && o.raw(r::DCFCLK_CNTL::reg, 0) == 0x00000ff5u,
               "SYMCLK32 SE back to refclk, every DCCG gate enabled, DCFCLK_GATE_DIS cleared (delays kept)");
        expect(hook_count(o, 0, hook_p_state_delegate) == 1 && o.trace[first_index(o, 0, Kind::hook,
                                                                                   hook_p_state_delegate)].value == 0,
               "one dc_dmub_srv_p_state_delegate(false) per prepare_bandwidth");
    }

    // write_scdc_data(165000) at every HDMI link_set_dpms_off, never for DP/DVI.
    for (Sig sig : {Sig::hdmi, Sig::dp, Sig::dvi}) {
        Dcn32StreamPlan a{};
        Dcn32StreamPlan b{};
        expect(make_plan(bw, sig, mode_1080, 1, 2, surface_base, a) &&
                   make_plan(bw, sig, mode_480, 1, 2, surface_base + 0x2000000, b),
               "SCDC plans build");
        Pair pair(reset_state());
        bool same = pair.step("commit", [&](OwnerRig& o) { o.controller.commit(a); },
                              [&](ReferenceRig& r) {
                                  r.linux_ref.init_hw_subset();
                                  r.linux_ref.commit(&a);
                              },
                              true, 1);
        Model& o = pair.owner->model;
        size_t mark = o.trace.size();
        same = same && pair.step("dpms off", [&](OwnerRig& w) { w.controller.set_dpms(false); },
                                 [&](ReferenceRig& r) { r.linux_ref.dpms(false); }, false, 1);
        const size_t scdc_off = hook_count(o, mark, hook_scdc_legacy);
        const size_t scdc_at = first_index(o, mark, Kind::hook, hook_scdc_legacy);
        const size_t disable_at = first_index(o, mark, Kind::hook, hook_disable_link);
        const size_t delegate_off = hook_count(o, mark, hook_p_state_delegate);
        mark = o.trace.size();
        same = same && pair.step("dpms on", [&](OwnerRig& w) { w.controller.set_dpms(true); },
                                 [&](ReferenceRig& r) { r.linux_ref.dpms(true); }, false, 1);
        const size_t delegate_on = hook_count(o, mark, hook_p_state_delegate);
        mark = o.trace.size();
        same = same && pair.step("mode change", [&](OwnerRig& w) { w.controller.commit(b); },
                                 [&](ReferenceRig& r) { r.linux_ref.commit(&b); }, true, 1);
        const size_t scdc_change = hook_count(o, mark, hook_scdc_legacy);
        expect(same, sig == Sig::hdmi ? "HDMI DPMS and mode change with the SCDC legacy write are Linux-exact"
                                      : "DP/DVI DPMS and mode change are Linux-exact");
        const size_t want = sig == Sig::hdmi ? 1u : 0u;
        expect(scdc_off == want && scdc_change == want && (want == 0 || scdc_at < disable_at),
               sig == Sig::hdmi ? "HDMI DPMS off and the old stream of a mode change write SCDC legacy before the link goes down"
                                : "no SCDC write for DP or DVI");
        expect(delegate_off == 0 && delegate_on == 1, "DPMS on (prepare_bandwidth) delegates the p-state, DPMS off not");
        expect(o.count(Kind::hook, hook_scdc_legacy) == 0 || o.trace[scdc_at].value == 2, "SCDC on the stream's engine");
    }

    // Linux ignores both results; only an unconfirmed call quarantines.
    {
        Dcn32StreamPlan plan{};
        expect(make_plan(bw, Sig::hdmi, mode_1080, 0, 0, surface_base, plan), "result plan builds");
        OwnerRig refused(reset_state());
        refused.hooks.delegate_script = {LinkCall::refused, LinkCall::refused, LinkCall::refused};
        refused.hooks.scdc_script = {LinkCall::refused};
        expect(refused.controller.commit(plan) == StreamResult::committed &&
                   refused.controller.set_dpms(false) == StreamResult::committed && !refused.controller.quarantined(),
               "a refused p-state delegate or SCDC write is ignored, as Linux ignores it");
        OwnerRig unsure(reset_state());
        unsure.hooks.delegate_script = {LinkCall::uncertain};
        expect(unsure.controller.commit(plan) == StreamResult::uncertain && unsure.controller.quarantined() &&
                   unsure.controller.report().failure == StreamFailure::hook_uncertain,
               "an unconfirmed DMCUB delegate quarantines");
        OwnerRig scdc(reset_state());
        scdc.hooks.scdc_script = {LinkCall::uncertain};
        expect(scdc.controller.commit(plan) == StreamResult::committed &&
                   scdc.controller.set_dpms(false) == StreamResult::uncertain && scdc.controller.quarantined(),
               "an unknown I2C state at the SCDC write quarantines");
        OwnerRig none(reset_state());
        Dcn32StreamHooks table = none.hooks.table();
        table.p_state_delegate = nullptr;
        table.hdmi_scdc_legacy = nullptr;
        none.controller.unbind();
        none.controller.bind(none.model.io(), segment[1], segment[2], segment[3], table);
        expect(none.controller.commit(plan) == StreamResult::committed &&
                   none.controller.set_dpms(false) == StreamResult::committed &&
                   none.model.count(Kind::hook, hook_p_state_delegate) == 0 &&
                   none.model.count(Kind::hook, hook_scdc_legacy) == 0,
               "without a DMUB service or DDC hook the steps are skipped");
    }
}

// A proven mode-1 reset: the next commit runs the dcn32_init_hw subset again
// (dc_set_power_state D0 -> init_hw in dm_resume) and enables from nothing.
void reset_recommit(Bandwidth& bw) {
    for (Sig sig : {Sig::dp, Sig::hdmi}) {
        Dcn32StreamPlan plan{};
        expect(make_plan(bw, sig, mode_1080, 1, 1, surface_base, plan), "reset plan builds");
        OwnerRig owner(reset_state());
        expect(owner.controller.commit(plan) == StreamResult::committed, "commit before the reset");
        owner.controller.reset_proven();
        owner.model.regs = reset_state(); // what a mode-1 reset leaves
        const size_t from = owner.model.trace.size();
        const StreamResult result = owner.controller.commit(plan);
        ReferenceRig ref(reset_state());
        ref.linux_ref.init_hw_subset();
        ref.linux_ref.commit(&plan);
        const size_t end = same_sequence("re-commit after reset", ref.model.trace, 0, owner.model.trace, from);
        expect(result == StreamResult::committed && end != SIZE_MAX &&
                   owner_suffix_only_checks(owner.model.trace, end, 1) &&
                   hook_count(owner.model, from, hook_disable_link) == 0,
               sig == Sig::dp ? "DP re-commit after a proven reset is the Linux init_hw + commit from an empty state"
                              : "HDMI re-commit after a proven reset is the Linux init_hw + commit from an empty state");
    }
}

// ---- ODM combine, DSC + FEC, two planes (dcn-dsc-odm) ----

constexpr Timing mode_2160p120{1075790, 3840, 3920, 8, 32, 2160, 2287, 113, 8, sync};
constexpr Timing mode_2160p144{1306200, 3840, 3920, 8, 32, 2160, 2314, 140, 8, sync};
constexpr uint8_t hbr2 = 0x14;
constexpr uint8_t hbr3 = 0x1e;
constexpr uint32_t dpcd_dsc_enable_address = 0x160;

struct StreamShape final {
    Timing timing{};
    uint32_t pipe = 0;
    uint32_t engine = 0;
    uint8_t rate = hbr3;
    uint8_t lanes = 4;
    bool allow_odm = false;
    DscConfig dsc{};
    uint32_t plane_count = 0;
    Dcn32PlaneScanout planes[dcn32_max_planes]{};
    const Dcn32Topology* current = nullptr;
};

DscConfig dsc_config(uint32_t slices_h, uint32_t slices_v, uint32_t bpp_x16) {
    DscConfig c{};
    c.num_slices_h = slices_h;
    c.num_slices_v = slices_v;
    c.bits_per_pixel = bpp_x16;
    c.block_pred_enable = true;
    c.linebuf_depth = 13;
    c.version_minor = 2;
    c.is_dp = true;
    return c;
}

Dcn32PlaneScanout scanout(uint64_t address, uint32_t width, uint32_t height, int32_t x, int32_t y) {
    Dcn32PlaneScanout s{};
    s.surface_address = address;
    s.pitch_pixels = (width + 63u) & ~63u;
    s.geometry.surface_width = width;
    s.geometry.surface_height = height;
    s.geometry.dst = Dcn32Rect{x, y, static_cast<int32_t>(width), static_cast<int32_t>(height)};
    return s;
}

Dcn32StreamRequest shaped_request(const StreamShape& s) {
    Dcn32StreamRequest q = request_for(Sig::dp, s.timing, s.pipe, s.engine, surface_base);
    q.dp_rate_code = s.rate;
    q.dp_lanes = s.lanes;
    q.plane_count = s.plane_count;
    for (uint32_t i = 0; i < dcn32_max_planes; ++i) {
        q.planes[i] = s.planes[i];
    }
    q.dsc = s.dsc;
    q.fec = s.dsc.num_slices_h != 0;
    return q;
}

bool shaped_bandwidth(Bandwidth& bw, const StreamShape& s, Dcn32BandwidthPlan& bandwidth) {
    Dcn32PlaneRequest pr{};
    pr.timing = s.timing;
    pr.signal = Dcn32OutputSignal::display_port;
    pr.pitch_pixels = (s.timing.width + 63u) & ~63u;
    pr.otg_instance = s.pipe;
    pr.allow_odm = s.allow_odm;
    pr.dsc_slices_h = s.dsc.num_slices_h;
    pr.dsc_bits_per_pixel_x16 = s.dsc.bits_per_pixel;
    pr.plane_count = s.plane_count;
    for (uint32_t i = 0; i < s.plane_count; ++i) {
        pr.planes[i] = s.planes[i].geometry;
        pr.plane_pitch_pixels[i] = s.planes[i].pitch_pixels;
    }
    pr.current = s.current;
    return bw.bandwidth.validate(pr, bandwidth) == Dcn32BandwidthStatus::ready;
}

bool make_shaped_plan(Bandwidth& bw, const StreamShape& s, Dcn32StreamPlan& plan) {
    Dcn32BandwidthPlan bandwidth{};
    return shaped_bandwidth(bw, s, bandwidth) &&
           build_dcn32_stream_plan(shaped_request(s), bandwidth, plan) == StreamStatus::success;
}

uint32_t plane_pipes(const Dcn32StreamPlan& plan) {
    uint32_t mask = 0;
    for (uint32_t i = 0; i < 4; ++i) {
        mask |= plan.bandwidth.topology.pipes[i].has_plane ? 1u << i : 0u;
    }
    return mask;
}

uint8_t dpcd_value(const Hooks& hooks, uint32_t address) {
    const auto found = hooks.dpcd.find(address);
    return found == hooks.dpcd.end() ? 0xee : found->second;
}

// ODM 4:1 is never chosen by DML for DP 8b/10b on this SoC table; the
// programming is generic, so a handcrafted context checks it: the 2:1 plan's
// pipe parameters on four slices (dcn32_determine_det_override: 18 segments
// over four pipes, four 64 KiB segments each).
bool odm4_plan(const Dcn32StreamPlan& odm2, uint32_t otg, Dcn32StreamPlan& plan) {
    Dcn32BandwidthPlan bandwidth = odm2.bandwidth;
    Dcn32Topology& t = bandwidth.topology;
    t = Dcn32Topology{};
    topology_start_stream(t, otg, false);
    if (!topology_append_plane(t, Dcn32Topology{}, 0) || !topology_add_odm_slices(t, Dcn32Topology{}, 4)) {
        return false;
    }
    Dcn32PlaneGeometry full{odm2.timing.width, odm2.timing.height,
                            Dcn32Rect{0, 0, static_cast<int32_t>(odm2.timing.width),
                                      static_cast<int32_t>(odm2.timing.height)}};
    if (!topology_build_scaling(t, odm2.timing.width, odm2.timing.height, &full, 1)) {
        return false;
    }
    bandwidth.pipe_count = 4;
    bandwidth.odm_slices = 4;
    for (uint32_t k = 0; k < 4; ++k) {
        bandwidth.pipes[k] = odm2.bandwidth.pipes[0];
        bandwidth.pipes[k].pipe = k;
        bandwidth.pipes[k].det_buffer_size_kb = 256;
    }
    bandwidth.det_buffer_size_kb = 256;
    bandwidth.compbuf_size_kb = 1280 - 4 * 256;
    StreamShape s{};
    s.timing = odm2.timing;
    s.pipe = otg;
    return build_dcn32_stream_plan(shaped_request(s), bandwidth, plan) == StreamStatus::success;
}

void odm_commits(Bandwidth& bw) {
    StreamShape s{};
    s.timing = mode_2160p120;
    s.pipe = 2;
    s.engine = 1;
    s.allow_odm = true;
    Dcn32StreamPlan odm{};
    Dcn32StreamPlan one{};
    expect(make_shaped_plan(bw, s, odm) && odm.odm_slices == 2 && odm.bandwidth.pipe_count == 2 &&
               odm.requested_pixel_100hz == mode_2160p120.pixel_clock_khz * 10u / 2u &&
               odm.pixel_rate_k2 == 1 /* DIV_BY_2 */ && odm.dio_pix_per_cycle == 2,
           "3840x2160@120 on DP HBR3 plans ODM 2:1 (half the pixel rate per OPP, DIG two pixels per cycle)");
    expect(make_plan(bw, Sig::dp, mode_1080, 2, 1, surface_base + 0x4000000, one), "one-pipe plan builds");
    const uint32_t second = odm.bandwidth.topology.pipes[2].next_odm;
    Pair pair(reset_state());
    pair.hubp_mask = plane_pipes(odm);
    bool same = pair.step("ODM 2:1 commit", [&](OwnerRig& o) { o.controller.commit(odm); },
                          [&](ReferenceRig& r) {
                              r.linux_ref.init_hw_subset();
                              r.linux_ref.commit(&odm);
                          },
                          true, 2);
    expect(same, "ODM 2:1 commit (3840x2160@120 DP HBR3) is Linux-exact");
    Model& o = pair.owner->model;
    expect(second < 4 && o.field(r::OPTC_DATA_SOURCE_SELECT::reg, 2, r::OPTC_DATA_SOURCE_SELECT::OPTC_NUM_OF_INPUT_SEGMENT) == 1 &&
               o.field(r::OPTC_DATA_SOURCE_SELECT::reg, 2, r::OPTC_DATA_SOURCE_SELECT::OPTC_SEG0_SRC_SEL) == 2 &&
               o.field(r::OPTC_DATA_SOURCE_SELECT::reg, 2, r::OPTC_DATA_SOURCE_SELECT::OPTC_SEG1_SRC_SEL) == second &&
               o.field(r::OTG_H_TIMING_CNTL::reg, 2, r::OTG_H_TIMING_CNTL::OTG_H_TIMING_DIV_MODE) == 1 &&
               o.field(r::OPTC_WIDTH_CONTROL::reg, 2, r::OPTC_WIDTH_CONTROL::OPTC_SEGMENT_WIDTH) == 1920 &&
               o.field(r::DCSURF_PRI_VIEWPORT_START::reg, second, r::DCSURF_PRI_VIEWPORT_START::PRI_VIEWPORT_X_START) == 1920 &&
               o.field(r::DCSURF_PRI_VIEWPORT_DIMENSION::reg, second, r::DCSURF_PRI_VIEWPORT_DIMENSION::PRI_VIEWPORT_WIDTH) == 1920 &&
               o.field(r::MPC_OUT_MUX::reg, second, r::MPC_OUT_MUX::MPC_OUT_MUX) == second &&
               o.field(r::DP_PIXEL_FORMAT::reg, 1, r::DP_PIXEL_FORMAT::DP_PIXEL_PER_CYCLE_PROCESSING_MODE) == 1,
           "two OPP segments of 1920 pixels, the second pipe scans out the right half");
    pair.hubp_mask = plane_pipes(one);
    same = pair.step("ODM 2:1 -> one pipe", [&](OwnerRig& w) { w.controller.commit(one); },
                     [&](ReferenceRig& r) { r.linux_ref.commit(&one); }, true, 2);
    expect(same && o.field(r::OTG_H_TIMING_CNTL::reg, 2, r::OTG_H_TIMING_CNTL::OTG_H_TIMING_DIV_MODE) == 0 &&
               o.field(r::DOMAIN_PG_STATUS::reg, second, r::DOMAIN_PG_STATUS::DOMAIN_PGFSM_PWR_STATUS) == 2,
           "ODM 2:1 -> 1920x1080 (ODM bypass, the second HUBP power-gated) is Linux-exact");
    pair.hubp_mask = plane_pipes(odm);
    same = pair.step("one pipe -> ODM 2:1", [&](OwnerRig& w) { w.controller.commit(odm); },
                     [&](ReferenceRig& r) { r.linux_ref.commit(&odm); }, true, 2);
    expect(same, "1920x1080 -> ODM 2:1 is Linux-exact");
    same = pair.step("DPMS off", [&](OwnerRig& w) { w.controller.set_dpms(false); },
                     [&](ReferenceRig& r) { r.linux_ref.dpms(false); }, false, 2);
    same = same && pair.step("DPMS on", [&](OwnerRig& w) { w.controller.set_dpms(true); },
                             [&](ReferenceRig& r) { r.linux_ref.dpms(true); }, false, 2);
    expect(same, "ODM 2:1 DPMS off/on (unblank at half rate, two pixels per cycle) is Linux-exact");

    // ODM 4:1 (handcrafted context).
    Dcn32StreamPlan odm4{};
    expect(odm4_plan(odm, 0, odm4) && odm4.odm_slices == 4 &&
               odm4.requested_pixel_100hz == mode_2160p120.pixel_clock_khz * 10u / 4u && odm4.pixel_rate_k2 == 1,
           "an ODM 4:1 context plans a quarter of the pixel rate per OPP (k2 DIV_BY_2: the DIG pixel rate divider policy)");
    Pair four(reset_state());
    four.hubp_mask = 0xf;
    same = four.step("ODM 4:1 commit", [&](OwnerRig& w) { w.controller.commit(odm4); },
                     [&](ReferenceRig& r) {
                         r.linux_ref.init_hw_subset();
                         r.linux_ref.commit(&odm4);
                     },
                     true, 0);
    Model& f = four.owner->model;
    expect(same && f.field(r::OPTC_DATA_SOURCE_SELECT::reg, 0, r::OPTC_DATA_SOURCE_SELECT::OPTC_NUM_OF_INPUT_SEGMENT) == 3 &&
               f.field(r::OTG_H_TIMING_CNTL::reg, 0, r::OTG_H_TIMING_CNTL::OTG_H_TIMING_DIV_MODE) == 3 &&
               f.field(r::OPTC_WIDTH_CONTROL::reg, 0, r::OPTC_WIDTH_CONTROL::OPTC_SEGMENT_WIDTH) == 960 &&
               f.field(r::DCSURF_PRI_VIEWPORT_START::reg, 3, r::DCSURF_PRI_VIEWPORT_START::PRI_VIEWPORT_X_START) == 2880,
           "ODM 4:1 commit (four 960-pixel segments) is Linux-exact");
    four.hubp_mask = 0;
    same = four.step("ODM 4:1 disable", [&](OwnerRig& w) { w.controller.disable(); },
                     [&](ReferenceRig& r) { r.linux_ref.commit(nullptr); }, false, 0);
    bool gated = true;
    for (uint32_t i = 0; i < 4; ++i) {
        gated = gated && f.field(r::DOMAIN_PG_STATUS::reg, i, r::DOMAIN_PG_STATUS::DOMAIN_PGFSM_PWR_STATUS) == 2;
    }
    expect(same && gated && f.field(r::OTG_CONTROL::reg, 0, r::OTG_CONTROL::OTG_MASTER_EN) == 0,
           "ODM 4:1 disable is Linux-exact and gates all four HUBPs");
}

void dsc_fec_commits(Bandwidth& bw) {
    // 3840x2160@144 on DP HBR3 x4: DSC 16 bpp, 4 slices, FEC, ODM 2:1 (power policy).
    StreamShape s{};
    s.timing = mode_2160p144;
    s.pipe = 0;
    s.engine = 0;
    s.allow_odm = true;
    s.dsc = dsc_config(4, 20, 256);
    Dcn32StreamPlan dsc{};
    expect(make_shaped_plan(bw, s, dsc) && dsc.dsc && dsc.fec && dsc.odm_slices == 2 &&
               dsc.dsc_optc.slice_width == 960 && dsc.bandwidth.topology.pipes[0].dsc &&
               dsc.bandwidth.topology.pipes[dsc.bandwidth.topology.pipes[0].next_odm].dsc,
           "3840x2160@144 DSC plan: FEC, ODM 2:1, a DSC engine per slice (two 960-pixel slices each)");
    {
        DscRegisterValues values{};
        DscOptcConfig optc{};
        uint8_t whole[dsc_pps_bytes]{};
        expect(dsc_prepare_config(3840, 2160, true, s.dsc, values, optc), "whole-picture PPS prepares");
        dsc_pack_pps(values.pps, whole);
        expect(std::memcmp(whole, dsc.dsc_pps, dsc_pps_bytes) == 0 && dsc.dsc_pps[0] == 0x12 &&
                   ((dsc.dsc_pps[6] << 8) | dsc.dsc_pps[7]) == 2160 && ((dsc.dsc_pps[8] << 8) | dsc.dsc_pps[9]) == 3840,
               "the plan's PPS SDP is the whole picture (DSC 1.2, 3840x2160), as dsc2_get_packed_pps");
    }
    Pair pair(reset_state());
    pair.hubp_mask = plane_pipes(dsc);
    bool same = pair.step("DSC commit", [&](OwnerRig& o) { o.controller.commit(dsc); },
                          [&](ReferenceRig& r) {
                              r.linux_ref.init_hw_subset();
                              r.linux_ref.commit(&dsc);
                          },
                          true, 0);
    expect(same, "DSC + FEC + ODM 2:1 commit (3840x2160@144 DP HBR3) is Linux-exact");
    Model& o = pair.owner->model;
    const uint32_t second = dsc.bandwidth.topology.pipes[0].next_odm;
    expect(dpcd_value(pair.owner->hooks, dpcd_dsc_enable_address) == 1 &&
               o.trace[first_index(o, 0, Kind::hook, hook_link_fec_dsc)].value == ((1u << 24) | 256u) &&
               o.field(r::DP_DSC_CNTL::reg, 0, r::DP_DSC_CNTL::DP_DSC_MODE) == 1 &&
               o.field(r::OPTC_DATA_FORMAT_CONTROL::reg, 0, r::OPTC_DATA_FORMAT_CONTROL::OPTC_DSC_MODE) == 1 &&
               o.field(r::DSCRM_DSC_FORWARD_CONFIG::reg, 0, r::DSCRM_DSC_FORWARD_CONFIG::DSCRM_DSC_FORWARD_EN) == 1 &&
               o.field(r::DSCRM_DSC_FORWARD_CONFIG::reg, second, r::DSCRM_DSC_FORWARD_CONFIG::DSCRM_DSC_OPP_PIPE_SOURCE) == second &&
               o.field(r::DP_GSP11_CNTL::reg, 0, r::DP_GSP11_CNTL::DP_SEC_GSP11_ENABLE) == 1,
           "the sink gets DP_DSC_ENABLE, the link trains with FEC at 16 bpp, encoder/OPTC/both DSCs and the PPS SDP are on");
    // DSC enable precedes enable_link; DP_DSC_ENABLE and the PPS follow it.
    const size_t link_at = first_index(o, 0, Kind::hook, hook_enable_link);
    const size_t dsc_mode_at = first_index(o, 0, Kind::write, address(r::DP_DSC_CNTL::reg, 0));
    const size_t gsp11_at = first_index(o, 0, Kind::write, address(r::DP_GSP11_CNTL::reg, 0));
    expect(dsc_mode_at < link_at && link_at < gsp11_at, "link_set_dpms_on: DSC before enable_link, PPS after it");
    size_t mark = o.trace.size();
    same = pair.step("DSC DPMS off", [&](OwnerRig& w) { w.controller.set_dpms(false); },
                     [&](ReferenceRig& r) { r.linux_ref.dpms(false); }, false, 0);
    expect(same && dpcd_value(pair.owner->hooks, dpcd_dsc_enable_address) == 0 &&
               o.field(r::DP_DSC_CNTL::reg, 0, r::DP_DSC_CNTL::DP_DSC_MODE) == 0 &&
               o.field(r::DSCRM_DSC_FORWARD_CONFIG::reg, 0, r::DSCRM_DSC_FORWARD_CONFIG::DSCRM_DSC_FORWARD_EN) == 0 &&
               first_index(o, mark, Kind::hook, hook_disable_link) <
                   first_index(o, mark, Kind::write, address(r::DP_DSC_CNTL::reg, 0)),
           "DSC DPMS off (link down, then sink and source DSC off) is Linux-exact");
    same = pair.step("DSC DPMS on", [&](OwnerRig& w) { w.controller.set_dpms(true); },
                     [&](ReferenceRig& r) { r.linux_ref.dpms(true); }, false, 0);
    expect(same && dpcd_value(pair.owner->hooks, dpcd_dsc_enable_address) == 1, "DSC DPMS on is Linux-exact");

    // A DSC stream without ODM (2160p60 on HBR2 x2 at 12 bpp), then a DSC
    // reconfiguration with ODM, then an uncompressed mode.
    StreamShape slow{};
    slow.timing = mode_2160;
    slow.pipe = 1;
    slow.engine = 3;
    slow.rate = hbr2;
    slow.lanes = 2;
    slow.dsc = dsc_config(4, 20, 192);
    Dcn32StreamPlan a{};
    expect(make_shaped_plan(bw, slow, a) && a.dsc && a.odm_slices == 1, "2160p60 DSC on HBR2 x2 plans one pipe");
    StreamShape fast = s;
    fast.pipe = 1;
    fast.engine = 3;
    Dcn32StreamPlan b{};
    Dcn32StreamPlan c{};
    expect(make_shaped_plan(bw, fast, b) && make_plan(bw, Sig::dp, mode_1080, 1, 3, surface_base, c),
           "DSC mode-change plans build");
    Pair change(reset_state());
    change.hubp_mask = plane_pipes(a);
    same = change.step("DSC one pipe", [&](OwnerRig& w) { w.controller.commit(a); },
                       [&](ReferenceRig& r) {
                           r.linux_ref.init_hw_subset();
                           r.linux_ref.commit(&a);
                       },
                       true, 1);
    change.hubp_mask = plane_pipes(b);
    same = same && change.step("DSC -> DSC + ODM", [&](OwnerRig& w) { w.controller.commit(b); },
                               [&](ReferenceRig& r) { r.linux_ref.commit(&b); }, true, 1);
    change.hubp_mask = plane_pipes(a);
    same = same && change.step("DSC + ODM -> DSC one pipe", [&](OwnerRig& w) { w.controller.commit(a); },
                               [&](ReferenceRig& r) { r.linux_ref.commit(&a); }, true, 1);
    change.hubp_mask = plane_pipes(b);
    same = same && change.step("DSC one pipe -> DSC + ODM again", [&](OwnerRig& w) { w.controller.commit(b); },
                               [&](ReferenceRig& r) { r.linux_ref.commit(&b); }, true, 1);
    change.hubp_mask = plane_pipes(c);
    same = same && change.step("DSC + ODM -> uncompressed", [&](OwnerRig& w) { w.controller.commit(c); },
                               [&](ReferenceRig& r) { r.linux_ref.commit(&c); }, true, 1);
    change.hubp_mask = 0;
    same = same && change.step("disable", [&](OwnerRig& w) { w.controller.disable(); },
                               [&](ReferenceRig& r) { r.linux_ref.commit(nullptr); }, false, 1);
    bool dsc_gated = true;
    for (uint32_t i = 0; i < 4; ++i) {
        dsc_gated = dsc_gated && change.owner->model.field(r::DOMAIN_PG_STATUS::reg, 4 + i,
                                                             r::DOMAIN_PG_STATUS::DOMAIN_PGFSM_PWR_STATUS) == 2;
    }
    expect(same, "DSC one pipe <-> DSC + ODM, then uncompressed and disable, are Linux-exact");
    expect(dsc_gated && dpcd_value(change.owner->hooks, dpcd_dsc_enable_address) == 0,
           "the DSC power domains end gated and the sink's DSC off");

    // Plan rules: DSC needs DP and FEC; FEC is only used with DSC.
    Dcn32BandwidthPlan bandwidth{};
    expect(shaped_bandwidth(bw, s, bandwidth), "DSC bandwidth validates");
    Dcn32StreamRequest q = shaped_request(s);
    Dcn32StreamPlan out{};
    q.fec = false;
    expect(build_dcn32_stream_plan(q, bandwidth, out) == StreamStatus::unsupported, "DSC without FEC is refused");
    q = shaped_request(s);
    q.dsc.num_slices_h = 3;
    expect(build_dcn32_stream_plan(q, bandwidth, out) == StreamStatus::unsupported,
           "DSC slices not divisible by the ODM slices are refused");
    q = shaped_request(s);
    q.dsc.bits_per_pixel = 95;
    expect(build_dcn32_stream_plan(q, bandwidth, out) == StreamStatus::unsupported,
           "a DSC rate dsc_prepare_config refuses is refused");
    Dcn32StreamPlan plain{};
    expect(make_plan(bw, Sig::dp, mode_1080, 0, 0, surface_base, plain), "plain plan builds");
    q = request_for(Sig::dp, mode_1080, 0, 0, surface_base);
    q.fec = true;
    expect(build_dcn32_stream_plan(q, plain.bandwidth, out) == StreamStatus::unsupported, "FEC without DSC is refused");
    q = request_for(Sig::hdmi, mode_1080, 0, 0, surface_base);
    q.dsc = dsc_config(2, 10, 128);
    q.fec = true;
    expect(build_dcn32_stream_plan(q, plain.bandwidth, out) == StreamStatus::unsupported,
           "DSC on TMDS is refused (HDMI 2.1 FRL DSC is out of scope)");
    // A plane update cannot change DSC: the stream is committed instead.
    OwnerRig owner(reset_state());
    expect(owner.controller.commit(a) == StreamResult::committed &&
               owner.controller.update_planes(b) == StreamResult::unchanged &&
               owner.controller.report().failure == StreamFailure::invalid_plan,
           "update_planes refuses a DSC/ODM change unchanged");
}

void mpo_commits(Bandwidth& bw) {
    const Dcn32PlaneScanout primary = scanout(surface_base, 1920, 1080, 0, 0);
    const Dcn32PlaneScanout overlay = scanout(surface_base + 0x2000000, 640, 360, 100, 200);
    StreamShape one{};
    one.timing = mode_1080;
    one.pipe = 0;
    one.engine = 0;
    one.plane_count = 1;
    one.planes[0] = primary;
    StreamShape two = one;
    two.plane_count = 2;
    two.planes[0] = overlay;
    two.planes[1] = primary;
    Dcn32StreamPlan fresh{};
    expect(make_shaped_plan(bw, two, fresh) && fresh.plane_count == 2 && fresh.bandwidth.pipe_count == 2 &&
               fresh.bandwidth.topology.pipes[0].plane == 0 && fresh.bandwidth.topology.pipes[0].bottom == 1,
           "two planes at 1080p60: the overlay is the top plane on the OTG master, the primary below it");
    {
        Pair pair(reset_state());
        pair.hubp_mask = plane_pipes(fresh);
        const bool same = pair.step("two-plane commit", [&](OwnerRig& o) { o.controller.commit(fresh); },
                                    [&](ReferenceRig& r) {
                                        r.linux_ref.init_hw_subset();
                                        r.linux_ref.commit(&fresh);
                                    },
                                    true, 0);
        Model& o = pair.owner->model;
        expect(same, "two-plane (MPO) commit is Linux-exact");
        expect(o.field(r::MPC_OUT_MUX::reg, 0, r::MPC_OUT_MUX::MPC_OUT_MUX) == 0 &&
                   o.field(r::MPCC_BOT_SEL::reg, 0, r::MPCC_BOT_SEL::MPCC_BOT_SEL) == 1 &&
                   o.field(r::MPCC_BOT_SEL::reg, 1, r::MPCC_BOT_SEL::MPCC_BOT_SEL) == 0xf &&
                   o.field(r::MPCC_CONTROL::reg, 0, r::MPCC_CONTROL::MPCC_MODE) == 3 &&
                   o.field(r::MPCC_OPP_ID::reg, 1, r::MPCC_OPP_ID::MPCC_OPP_ID) == 0 &&
                   o.field(r::RECOUT_START::reg, 0, r::RECOUT_START::RECOUT_START_X) == 100 &&
                   o.field(r::RECOUT_SIZE::reg, 0, r::RECOUT_SIZE::RECOUT_WIDTH) == 640 &&
                   o.field(r::DCSURF_PRIMARY_SURFACE_ADDRESS::reg, 0,
                           r::DCSURF_PRIMARY_SURFACE_ADDRESS::PRIMARY_SURFACE_ADDRESS) ==
                       static_cast<uint32_t>(overlay.surface_address),
               "OPP 0 <- MPCC 0 (overlay, top-bottom blending) <- MPCC 1 (primary, last)");
    }

    // Plane add, move with per-pixel alpha, global alpha, and remove on a live stream.
    Dcn32StreamPlan p1{};
    expect(make_shaped_plan(bw, one, p1), "one-plane plan builds");
    Pair pair(reset_state());
    pair.hubp_mask = plane_pipes(p1);
    bool same = pair.step("one plane", [&](OwnerRig& o) { o.controller.commit(p1); },
                          [&](ReferenceRig& r) {
                              r.linux_ref.init_hw_subset();
                              r.linux_ref.commit(&p1);
                          },
                          true, 0);
    two.current = &p1.bandwidth.topology;
    Dcn32StreamPlan p2{};
    expect(make_shaped_plan(bw, two, p2), "plane add plan builds");
    pair.hubp_mask = plane_pipes(p2);
    StreamResult added = StreamResult::unchanged;
    same = same && pair.step("add overlay", [&](OwnerRig& o) { added = o.controller.update_planes(p2); },
                             [&](ReferenceRig& r) { r.linux_ref.update_planes(p2); }, true, 0);
    expect(same && added == StreamResult::committed, "adding an overlay plane (dc plane add under pipe locks) is Linux-exact");
    StreamShape moved = two;
    moved.current = &p2.bandwidth.topology;
    moved.planes[0] = scanout(surface_base + 0x2000000, 640, 360, 1280, 720);
    moved.planes[0].per_pixel_alpha = true;
    moved.planes[0].pre_multiplied_alpha = true;
    Dcn32StreamPlan p3{};
    expect(make_shaped_plan(bw, moved, p3), "plane move plan builds");
    same = pair.step("move overlay", [&](OwnerRig& o) { o.controller.update_planes(p3); },
                     [&](ReferenceRig& r) { r.linux_ref.update_planes(p3); }, true, 0);
    expect(same && pair.owner->model.field(r::RECOUT_START::reg, 0, r::RECOUT_START::RECOUT_START_X) == 1280,
           "moving the overlay with per-pixel alpha is Linux-exact");
    StreamShape faded = moved;
    faded.current = &p3.bandwidth.topology;
    faded.planes[0].per_pixel_alpha = false;
    faded.planes[0].pre_multiplied_alpha = false;
    faded.planes[0].global_alpha = true;
    faded.planes[0].global_alpha_value = 0x80;
    Dcn32StreamPlan p4{};
    expect(make_shaped_plan(bw, faded, p4), "global alpha plan builds");
    same = pair.step("global alpha", [&](OwnerRig& o) { o.controller.update_planes(p4); },
                     [&](ReferenceRig& r) { r.linux_ref.update_planes(p4); }, true, 0);
    expect(same && pair.owner->model.field(r::MPCC_CONTROL::reg, 0, r::MPCC_CONTROL::MPCC_GLOBAL_ALPHA) == 0x80 &&
               pair.owner->model.field(r::MPCC_CONTROL::reg, 0, r::MPCC_CONTROL::MPCC_ALPHA_BLND_MODE) == 2,
           "a global-alpha overlay is Linux-exact (MPCC global alpha 0x80)");
    StreamShape combined = faded;
    combined.current = &p4.bandwidth.topology;
    combined.planes[0].per_pixel_alpha = true;
    combined.planes[0].global_alpha_value = 0x40;
    Dcn32StreamPlan p4b{};
    expect(make_shaped_plan(bw, combined, p4b), "per-pixel alpha with global gain plan builds");
    same = pair.step("per-pixel alpha with global gain", [&](OwnerRig& o) { o.controller.update_planes(p4b); },
                     [&](ReferenceRig& r) { r.linux_ref.update_planes(p4b); }, true, 0);
    expect(same && pair.owner->model.field(r::MPCC_CONTROL::reg, 0, r::MPCC_CONTROL::MPCC_GLOBAL_GAIN) == 0x40 &&
               pair.owner->model.field(r::MPCC_CONTROL::reg, 0, r::MPCC_CONTROL::MPCC_ALPHA_BLND_MODE) == 1,
           "per-pixel alpha combined with a global gain is Linux-exact");
    one.current = &p4b.bandwidth.topology;
    Dcn32StreamPlan p5{};
    expect(make_shaped_plan(bw, one, p5), "plane remove plan builds");
    pair.hubp_mask = plane_pipes(p5);
    same = pair.step("remove overlay", [&](OwnerRig& o) { o.controller.update_planes(p5); },
                     [&](ReferenceRig& r) { r.linux_ref.update_planes(p5); }, true, 0);
    uint32_t gated_hubp = 4;
    for (uint32_t i = 0; i < 4; ++i) {
        if ((plane_pipes(p4b) & ~plane_pipes(p5) & (1u << i)) != 0) {
            gated_hubp = i;
        }
    }
    expect(same && gated_hubp < 4 &&
               pair.owner->model.field(r::DOMAIN_PG_STATUS::reg, gated_hubp, r::DOMAIN_PG_STATUS::DOMAIN_PGFSM_PWR_STATUS) == 2,
           "removing the overlay (MPCC removed, HUBP disabled after unlock) is Linux-exact");
    pair.hubp_mask = 0;
    same = pair.step("disable", [&](OwnerRig& o) { o.controller.disable(); },
                     [&](ReferenceRig& r) { r.linux_ref.commit(nullptr); }, false, 0);
    expect(same, "disable after plane updates is Linux-exact");

    // A two-plane mode set over a two-plane stream, and the refusals.
    Pair again(reset_state());
    again.hubp_mask = plane_pipes(fresh);
    same = again.step("two planes", [&](OwnerRig& o) { o.controller.commit(fresh); },
                      [&](ReferenceRig& r) {
                          r.linux_ref.init_hw_subset();
                          r.linux_ref.commit(&fresh);
                      },
                      true, 0);
    StreamShape big = two;
    big.timing = mode_2160;
    big.planes[1] = scanout(surface_base, 3840, 2160, 0, 0);
    big.current = &fresh.bandwidth.topology;
    Dcn32StreamPlan p6{};
    expect(make_shaped_plan(bw, big, p6), "2160p60 two-plane plan builds");
    again.hubp_mask = plane_pipes(p6);
    same = same && again.step("two planes -> two planes at 2160p60", [&](OwnerRig& o) { o.controller.commit(p6); },
                              [&](ReferenceRig& r) { r.linux_ref.commit(&p6); }, true, 0);
    expect(same, "a mode change between two-plane streams is Linux-exact");
    expect(again.owner->controller.update_planes(p1) == StreamResult::unchanged,
           "update_planes refuses a plan that changes the timing");
    Dcn32StreamPlan off_screen = p2;
    off_screen.planes[0].geometry.dst.x = 1800;
    Dcn32StreamRequest q = shaped_request(two);
    q.planes[0].geometry.dst.x = 1800;
    expect(build_dcn32_stream_plan(q, p2.bandwidth, off_screen) == StreamStatus::invalid_surface,
           "a plane beyond the active area is refused");
    q = shaped_request(two);
    q.planes[0].global_alpha_value = 0x100;
    expect(build_dcn32_stream_plan(q, p2.bandwidth, off_screen) == StreamStatus::invalid_surface,
           "a global alpha above 0xff is refused");
}

int main() {
#ifdef RX7900_UPSTREAM_DCN_STREAM_DIR
    if (!headers.load(RX7900_UPSTREAM_DCN_STREAM_DIR)) {
        std::fprintf(stderr, "pinned DCN 3.2 headers not found\n");
        return 1;
    }
#else
    std::fprintf(stderr, "RX7900_UPSTREAM_DCN_STREAM_DIR is defined by the build\n");
    return 1;
#endif
    build_names();
    auto bw = std::make_unique<Bandwidth>();
    expect(bw->workspace != nullptr, "bandwidth workspace");
    fresh_commits(*bw);
    edge_commits(*bw);
    mode_changes(*bw);
    takeover(*bw);
    dpms(*bw);
    plan_rules(*bw);
    failures(*bw);
    linux_steps(*bw);
    reset_recommit(*bw);
    odm_commits(*bw);
    dsc_fec_commits(*bw);
    mpo_commits(*bw);
    return checks.finish("dcn32_stream");
}

/*
Module: rx7900.tests.dcn32_stream_model
Purpose: Synthetic Navi31 DCN 3.2 register file with status emulation and an access trace for the stream tests.
Created: 2026-10-24
Architecture: Test-only sparse BAR5 model behind hardware::RegisterIo. Every
read, write, delay and link-owner hook call is appended to one trace so a run
of the stream owner compares access for access with the Linux reference in
tests/dcn32_stream_reference.hpp, which drives the same model. Read-only status
fields are derived on read from the control fields the hardware would follow
(update lock, clock-on, power-gating FSM, MPCC idle, DET/compbuf current, DIG
FIFO reset done, DP stream status, earliest-in-use address, frame counter);
faults, stuck status and underflow are injectable. Segment bases are the
Navi31 DCN_BASE values Linux uses. This is not hardware evidence.
*/
#pragma once

#include <cstdint>
#include <cstdio>
#include <map>
#include <vector>

#include "asic_test_device.hpp"
#include "dcn32_stream.hpp"

namespace rx7900_test::stream {

namespace r = rx7900::display::dcn32_stream_reg;
using rx7900::display::Dcn32StreamClocks;
using rx7900::display::Dcn32StreamHooks;
using rx7900::display::Dcn32StreamPlan;
using rx7900::display::LinkCall;

// DCN_BASE instance 0, segments 0..3 (dcn_3_2_0 IP offsets on Navi31).
constexpr uint32_t segment[4] = {0x00000012u, 0x000000c0u, 0x000034c0u, 0x00009000u};
constexpr uint32_t aperture = 0x00100000u;

inline uint32_t address(const r::Register& reg, uint32_t instance) {
    return (segment[reg.base_index] + reg.offset[instance]) * 4u;
}

inline uint32_t field_value(uint32_t word, r::Field field) { return (word & field.mask) >> field.shift; }
inline uint32_t with_field(uint32_t word, r::Field field, uint32_t value) {
    return (word & ~field.mask) | ((value << field.shift) & field.mask);
}

enum class Kind : char { read = 'R', write = 'W', delay = 'D', hook = 'H' };

struct Op final {
    Kind kind;
    uint32_t offset;
    uint32_t value;
    bool operator==(const Op& other) const {
        return kind == other.kind && offset == other.offset && value == other.value;
    }
    bool operator!=(const Op& other) const { return !(*this == other); }
};

// Hook identifiers recorded in Op::offset (value carries the argument).
enum HookId : uint32_t {
    hook_update_clocks = 0x48000001u,
    hook_clock_dispclk = 0x48000002u,
    hook_clock_dppclk = 0x48000003u,
    hook_clock_dcfclk = 0x48000004u,
    hook_clock_socclk = 0x48000005u,
    hook_clock_dramclk = 0x48000006u,
    hook_clock_fclk = 0x48000007u,
    hook_clock_deep_sleep = 0x48000008u,
    hook_clock_dpp_dto = 0x48000009u,
    hook_clock_displays = 0x4800000au,
    hook_clock_pstate = 0x4800000bu,
    hook_set_pixel_clock = 0x48000010u,
    hook_pixel_divider = 0x48000011u,
    hook_power_down = 0x48000020u,
    hook_enable_link = 0x48000030u,
    hook_link_rate = 0x48000031u,
    hook_disable_link = 0x48000040u,
    hook_dpcd_read = 0x48000050u,
    hook_dpcd_write = 0x48000060u,
    hook_p_state_delegate = 0x48000070u,
    hook_scdc_legacy = 0x48000080u,
    // dcn-dsc-odm: per-pipe DPP DTOs of a multi-pipe context and the FEC/DSC
    // facts of a DP enable (recorded only when present, so one-pipe traces
    // keep their shape).
    hook_clock_dpp_pipe = 0x48000090u,
    hook_link_fec_dsc = 0x480000a0u,
};

class Model final {
public:
    std::map<uint32_t, uint32_t> regs;
    std::vector<Op> trace;
    // Fault injection: index (among writes) of the write that fails, the byte
    // offset whose read fails, a status that never comes, frozen frame counter.
    size_t fail_write_at = SIZE_MAX;
    uint32_t fail_read_offset = UINT32_MAX;
    bool otg_clock_never_on = false;
    bool dp_stream_status_stuck = false;
    bool frame_frozen = false;
    bool underflow_after_enable = false;
    size_t writes = 0;
    uint64_t delayed_us = 0;

    void set(const r::Register& reg, uint32_t instance, uint32_t value) { regs[address(reg, instance)] = value; }
    uint32_t raw(const r::Register& reg, uint32_t instance) const {
        const auto found = regs.find(address(reg, instance));
        return found == regs.end() ? 0u : found->second;
    }
    void set_field(const r::Register& reg, uint32_t instance, r::Field field, uint32_t value) {
        set(reg, instance, with_field(raw(reg, instance), field, value));
    }
    // The value a read returns (stored control fields plus derived status).
    uint32_t value(const r::Register& reg, uint32_t instance) { return read_value(address(reg, instance)); }
    uint32_t field(const r::Register& reg, uint32_t instance, r::Field f) {
        return field_value(value(reg, instance), f);
    }

    rx7900::hardware::RegisterIo io() {
        return {this, aperture, &Model::read32, &Model::write32, &Model::delay};
    }

    void hook(uint32_t id, uint32_t value) { trace.push_back({Kind::hook, id, value}); }

    // Number of trace entries of one kind at one offset.
    size_t count(Kind kind, uint32_t offset) const {
        size_t n = 0;
        for (const Op& op : trace) {
            n += (op.kind == kind && op.offset == offset) ? 1u : 0u;
        }
        return n;
    }

private:
    uint32_t stored(uint32_t offset) const {
        const auto found = regs.find(offset);
        return found == regs.end() ? 0u : found->second;
    }
    uint32_t stored(const r::Register& reg, uint32_t instance) const { return stored(address(reg, instance)); }

    template <typename Body>
    static bool for_instance(const r::Register& reg, uint32_t offset, Body body) {
        for (uint32_t i = 0; i < reg.count; ++i) {
            if (address(reg, i) == offset) {
                body(i);
                return true;
            }
        }
        return false;
    }

    uint32_t read_value(uint32_t offset) {
        uint32_t v = stored(offset);
        for_instance(r::OTG_MASTER_UPDATE_LOCK::reg, offset, [&](uint32_t) {
            namespace L = r::OTG_MASTER_UPDATE_LOCK;
            v = with_field(v, L::UPDATE_LOCK_STATUS, field_value(v, L::OTG_MASTER_UPDATE_LOCK));
        });
        for_instance(r::OTG_CLOCK_CONTROL::reg, offset, [&](uint32_t) {
            namespace C = r::OTG_CLOCK_CONTROL;
            v = with_field(v, C::OTG_CLOCK_ON, otg_clock_never_on ? 0u : field_value(v, C::OTG_CLOCK_EN));
            v = with_field(v, C::OTG_BUSY, 0);
        });
        for_instance(r::OPTC_INPUT_CLOCK_CONTROL::reg, offset, [&](uint32_t) {
            namespace C = r::OPTC_INPUT_CLOCK_CONTROL;
            v = with_field(v, C::OPTC_INPUT_CLK_ON, field_value(v, C::OPTC_INPUT_CLK_EN));
        });
        for_instance(r::DOMAIN_PG_STATUS::reg, offset, [&](uint32_t i) {
            const uint32_t gated = field_value(stored(r::DOMAIN_PG_CONFIG::reg, i), r::DOMAIN_PG_CONFIG::DOMAIN_POWER_GATE);
            v = with_field(v, r::DOMAIN_PG_STATUS::DOMAIN_PGFSM_PWR_STATUS, gated ? 2u : 0u);
        });
        for_instance(r::MPCC_STATUS::reg, offset, [&](uint32_t i) {
            namespace S = r::MPCC_STATUS;
            const bool free = field_value(stored(r::MPCC_TOP_SEL::reg, i), r::MPCC_TOP_SEL::MPCC_TOP_SEL) == 0xf;
            v = with_field(v, S::MPCC_IDLE, free ? 1u : 0u);
            v = with_field(v, S::MPCC_DISABLED, free ? 1u : 0u);
            v = with_field(v, S::MPCC_BUSY, free ? 0u : 1u);
        });
        const r::Register* det[4] = {&r::DCHUBBUB_DET0_CTRL::reg, &r::DCHUBBUB_DET1_CTRL::reg,
                                     &r::DCHUBBUB_DET2_CTRL::reg, &r::DCHUBBUB_DET3_CTRL::reg};
        const r::Field det_size[4] = {r::DCHUBBUB_DET0_CTRL::DET0_SIZE, r::DCHUBBUB_DET1_CTRL::DET1_SIZE,
                                      r::DCHUBBUB_DET2_CTRL::DET2_SIZE, r::DCHUBBUB_DET3_CTRL::DET3_SIZE};
        const r::Field det_current[4] = {
            r::DCHUBBUB_DET0_CTRL::DET0_SIZE_CURRENT, r::DCHUBBUB_DET1_CTRL::DET1_SIZE_CURRENT,
            r::DCHUBBUB_DET2_CTRL::DET2_SIZE_CURRENT, r::DCHUBBUB_DET3_CTRL::DET3_SIZE_CURRENT};
        for (uint32_t i = 0; i < 4; ++i) {
            if (offset == address(*det[i], 0)) {
                v = with_field(v, det_current[i], field_value(v, det_size[i]));
            }
        }
        if (offset == address(r::DCHUBBUB_COMPBUF_CTRL::reg, 0)) {
            namespace C = r::DCHUBBUB_COMPBUF_CTRL;
            v = with_field(v, C::COMPBUF_SIZE_CURRENT, field_value(v, C::COMPBUF_SIZE));
            v = with_field(v, C::CONFIG_ERROR, 0);
        }
        for_instance(r::DIG_FIFO_CTRL0::reg, offset, [&](uint32_t) {
            namespace F = r::DIG_FIFO_CTRL0;
            v = with_field(v, F::DIG_FIFO_RESET_DONE, field_value(v, F::DIG_FIFO_RESET));
        });
        for_instance(r::DP_VID_STREAM_CNTL::reg, offset, [&](uint32_t) {
            namespace V = r::DP_VID_STREAM_CNTL;
            if (!dp_stream_status_stuck) {
                v = with_field(v, V::DP_VID_STREAM_STATUS, field_value(v, V::DP_VID_STREAM_ENABLE));
            }
        });
        for_instance(r::DCSURF_SURFACE_EARLIEST_INUSE::reg, offset, [&](uint32_t i) {
            v = stored(r::DCSURF_PRIMARY_SURFACE_ADDRESS::reg, i);
        });
        for_instance(r::DCSURF_SURFACE_EARLIEST_INUSE_HIGH::reg, offset, [&](uint32_t i) {
            v = stored(r::DCSURF_PRIMARY_SURFACE_ADDRESS_HIGH::reg, i);
        });
        for_instance(r::DCHUBP_CNTL::reg, offset, [&](uint32_t) {
            v = with_field(v, r::DCHUBP_CNTL::HUBP_NO_OUTSTANDING_REQ, 1);
        });
        // A running OTG passes through blank and active within a poll.
        for_instance(r::OTG_STATUS::reg, offset, [&](uint32_t i) {
            const uint32_t on = field_value(stored(r::OTG_CONTROL::reg, i), r::OTG_CONTROL::OTG_MASTER_EN);
            v = with_field(v, r::OTG_STATUS::OTG_V_BLANK, on);
            v = with_field(v, r::OTG_STATUS::OTG_V_ACTIVE_DISP, on);
        });
        return v;
    }

    void after_write(uint32_t offset, uint32_t written) {
        // Sticky underflow: clear bits clear the status; the bits themselves read 0.
        for_instance(r::OPTC_INPUT_GLOBAL_CONTROL::reg, offset, [&](uint32_t i) {
            namespace G = r::OPTC_INPUT_GLOBAL_CONTROL;
            uint32_t v = written;
            if (field_value(written, G::OPTC_UNDERFLOW_CLEAR) != 0) {
                v = with_field(v, G::OPTC_UNDERFLOW_OCCURRED_STATUS, 0);
            }
            v = with_field(v, G::OPTC_UNDERFLOW_CLEAR, 0);
            regs[address(G::reg, i)] = v;
        });
        for_instance(r::DCHUBP_CNTL::reg, offset, [&](uint32_t i) {
            namespace H = r::DCHUBP_CNTL;
            uint32_t v = written;
            if (field_value(written, H::HUBP_UNDERFLOW_CLEAR) != 0) {
                v = with_field(v, H::HUBP_UNDERFLOW_STATUS, 0);
            }
            v = with_field(v, H::HUBP_UNDERFLOW_CLEAR, 0);
            regs[address(H::reg, i)] = v;
        });
    }

    void on_delay(uint32_t microseconds) {
        delayed_us += microseconds;
        for (uint32_t i = 0; i < r::OTG_CONTROL::reg.count; ++i) {
            if (field_value(stored(r::OTG_CONTROL::reg, i), r::OTG_CONTROL::OTG_MASTER_EN) == 0) {
                continue;
            }
            if (!frame_frozen) {
                const uint32_t count = field_value(stored(r::OTG_STATUS_FRAME_COUNT::reg, i),
                                                   r::OTG_STATUS_FRAME_COUNT::OTG_FRAME_COUNT);
                regs[address(r::OTG_STATUS_FRAME_COUNT::reg, i)] =
                    with_field(stored(r::OTG_STATUS_FRAME_COUNT::reg, i), r::OTG_STATUS_FRAME_COUNT::OTG_FRAME_COUNT,
                               count + 1);
            }
            if (underflow_after_enable) {
                regs[address(r::OPTC_INPUT_GLOBAL_CONTROL::reg, i)] =
                    with_field(stored(r::OPTC_INPUT_GLOBAL_CONTROL::reg, i),
                               r::OPTC_INPUT_GLOBAL_CONTROL::OPTC_UNDERFLOW_OCCURRED_STATUS, 1);
            }
        }
    }

    static bool read32(void* context, uint32_t offset, uint32_t& value) noexcept {
        auto& model = *static_cast<Model*>(context);
        if (offset == model.fail_read_offset || offset >= aperture || (offset & 3u) != 0) {
            return false;
        }
        value = model.read_value(offset);
        model.trace.push_back({Kind::read, offset, value});
        return true;
    }

    static bool write32(void* context, uint32_t offset, uint32_t value) noexcept {
        auto& model = *static_cast<Model*>(context);
        if (model.writes++ == model.fail_write_at || offset >= aperture || (offset & 3u) != 0) {
            return false;
        }
        model.trace.push_back({Kind::write, offset, value});
        model.regs[offset] = value;
        model.after_write(offset, value);
        return true;
    }

    static void delay(void* context, uint32_t microseconds) noexcept {
        auto& model = *static_cast<Model*>(context);
        model.trace.push_back({Kind::delay, 0, microseconds});
        model.on_delay(microseconds);
    }
};

// The link owner side of the tests: records every call in the model trace and
// answers from per-hook scripts (default success). enable_link powers the DIG
// FE symbol clock as the link owner's FE->BE connect does; disable_link drops it.
struct Hooks final {
    Model* model = nullptr;
    std::vector<LinkCall> clocks_script;
    std::vector<LinkCall> pixel_script;
    std::vector<LinkCall> enable_script;
    std::vector<LinkCall> disable_script;
    std::vector<LinkCall> delegate_script;
    std::vector<LinkCall> scdc_script;
    size_t clocks_calls = 0;
    size_t pixel_calls = 0;
    size_t enable_calls = 0;
    size_t disable_calls = 0;
    size_t power_down_calls = 0;
    size_t delegate_calls = 0;
    size_t scdc_calls = 0;
    uint8_t downspread = 0;
    // DPCD bytes other than DP_DOWNSPREAD_CTRL (0x107), written by the owner.
    std::map<uint32_t, uint8_t> dpcd;

    static LinkCall next(std::vector<LinkCall>& script, size_t& calls) {
        const size_t index = calls++;
        return index < script.size() ? script[index] : LinkCall::success;
    }

    Dcn32StreamHooks table() {
        Dcn32StreamHooks h{};
        h.context = this;
        h.update_clocks = &Hooks::update_clocks;
        h.set_pixel_clock = &Hooks::set_pixel_clock;
        h.power_down_pixel_clock = &Hooks::power_down;
        h.enable_link = &Hooks::enable_link;
        h.disable_link = &Hooks::disable_link;
        h.dpcd_read = &Hooks::dpcd_read;
        h.dpcd_write = &Hooks::dpcd_write;
        h.p_state_delegate = &Hooks::p_state_delegate;
        h.hdmi_scdc_legacy = &Hooks::hdmi_scdc_legacy;
        return h;
    }

    // The exact recording both the owner and the reference produce.
    void record_clocks(const Dcn32StreamClocks& c, bool safe_to_lower) {
        model->hook(hook_update_clocks, safe_to_lower ? 1u : 0u);
        model->hook(hook_clock_dispclk, c.clocks.dispclk_khz);
        model->hook(hook_clock_dppclk, c.clocks.dppclk_khz);
        model->hook(hook_clock_dcfclk, c.clocks.dcfclk_khz);
        model->hook(hook_clock_socclk, c.clocks.socclk_khz);
        model->hook(hook_clock_dramclk, c.clocks.dramclk_khz);
        model->hook(hook_clock_fclk, c.clocks.fclk_khz);
        model->hook(hook_clock_deep_sleep, c.clocks.dcfclk_deep_sleep_khz);
        model->hook(hook_clock_dpp_dto, (c.pipe << 28) | c.dpp_khz);
        for (uint32_t pipe = 0; pipe < 4; ++pipe) {
            if (c.dpp_pipe_khz[pipe] != 0) {
                model->hook(hook_clock_dpp_pipe, (pipe << 28) | c.dpp_pipe_khz[pipe]);
            }
        }
        model->hook(hook_clock_displays, c.display_count);
        model->hook(hook_clock_pstate, c.clocks.p_state_change_support ? 1u : 0u);
    }
    void record_pixel_clock(const Dcn32StreamPlan& p) {
        model->hook(hook_set_pixel_clock, p.requested_pixel_100hz);
        model->hook(hook_pixel_divider, (p.pipe << 8) | (p.pixel_rate_k1 << 4) | p.pixel_rate_k2);
    }
    void record_enable(const Dcn32StreamPlan& p) {
        model->hook(hook_enable_link, (static_cast<uint32_t>(p.signal) << 8) | p.engine);
        model->hook(hook_link_rate, p.signal == rx7900::display::StreamSignal::display_port
                                        ? (static_cast<uint32_t>(p.dp_rate_code) << 8) | p.dp_lanes
                                        : p.tmds_khz);
        if (p.fec || p.dsc) {
            model->hook(hook_link_fec_dsc, (p.fec ? 1u << 24 : 0u) | (p.dsc ? p.dsc_config.bits_per_pixel : 0u));
        }
        model->set_field(r::DIG_FE_CNTL::reg, p.engine, r::DIG_FE_CNTL::DIG_SYMCLK_FE_ON, 1);
    }
    void record_p_state_delegate(bool should_manage_pstate) {
        model->hook(hook_p_state_delegate, should_manage_pstate ? 1u : 0u);
    }
    void record_scdc_legacy(const Dcn32StreamPlan& p) { model->hook(hook_scdc_legacy, p.engine); }
    void record_disable(const Dcn32StreamPlan& p) {
        model->hook(hook_disable_link, (static_cast<uint32_t>(p.signal) << 8) | p.engine);
        model->set_field(r::DIG_FE_CNTL::reg, p.engine, r::DIG_FE_CNTL::DIG_SYMCLK_FE_ON, 0);
    }

    static LinkCall update_clocks(void* context, const Dcn32StreamClocks& c, bool safe_to_lower) noexcept {
        auto& self = *static_cast<Hooks*>(context);
        self.record_clocks(c, safe_to_lower);
        return next(self.clocks_script, self.clocks_calls);
    }
    static LinkCall set_pixel_clock(void* context, const Dcn32StreamPlan& p) noexcept {
        auto& self = *static_cast<Hooks*>(context);
        self.record_pixel_clock(p);
        return next(self.pixel_script, self.pixel_calls);
    }
    static LinkCall power_down(void* context, const Dcn32StreamPlan& p) noexcept {
        auto& self = *static_cast<Hooks*>(context);
        ++self.power_down_calls;
        self.model->hook(hook_power_down, p.pipe);
        return LinkCall::success;
    }
    static LinkCall enable_link(void* context, const Dcn32StreamPlan& p) noexcept {
        auto& self = *static_cast<Hooks*>(context);
        const LinkCall result = next(self.enable_script, self.enable_calls);
        if (result == LinkCall::success) {
            self.record_enable(p);
        } else {
            self.model->hook(hook_enable_link, 0xdeadu);
        }
        return result;
    }
    static LinkCall disable_link(void* context, const Dcn32StreamPlan& p) noexcept {
        auto& self = *static_cast<Hooks*>(context);
        self.record_disable(p);
        return next(self.disable_script, self.disable_calls);
    }
    static LinkCall p_state_delegate(void* context, bool should_manage_pstate) noexcept {
        auto& self = *static_cast<Hooks*>(context);
        self.record_p_state_delegate(should_manage_pstate);
        return next(self.delegate_script, self.delegate_calls);
    }
    static LinkCall hdmi_scdc_legacy(void* context, const Dcn32StreamPlan& p) noexcept {
        auto& self = *static_cast<Hooks*>(context);
        self.record_scdc_legacy(p);
        return next(self.scdc_script, self.scdc_calls);
    }
    static LinkCall dpcd_read(void* context, uint32_t address, uint8_t& value) noexcept {
        auto& self = *static_cast<Hooks*>(context);
        self.model->hook(hook_dpcd_read, address);
        const auto found = self.dpcd.find(address);
        value = address == 0x107 || found == self.dpcd.end() ? self.downspread : found->second;
        return LinkCall::success;
    }
    static LinkCall dpcd_write(void* context, uint32_t address, uint8_t value) noexcept {
        auto& self = *static_cast<Hooks*>(context);
        self.model->hook(hook_dpcd_write, (address << 8) | value);
        if (address == 0x107) {
            self.downspread = value;
        } else {
            self.dpcd[address] = value;
        }
        return LinkCall::success;
    }
};

} // namespace rx7900_test::stream

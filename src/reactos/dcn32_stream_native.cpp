/*
Module: rx7900.reactos.dcn32_stream_native
Purpose: Native EDID/route discovery, VRAM scanout surfaces and VideoPort mode IOCTLs for the DCN 3.2 stream owner.
Created: 2026-10-24
Architecture: Every hardware decision stays in display::Dcn32StreamController,
display::Dcn32Bandwidth and output::DisplayLinkOwner; this adapter supplies MMIO,
bounded waits, the route lease, the AUX/DDC transports, pinned VRAM and the
VideoPort plumbing. Link-owner results map onto LinkCall: quarantined is
uncertain, every other failure is a refusal. Linux references: amdgpu_dm
(create_stream_for_sink defaults, amdgpu_dm_connector_get_modes; the EDID
modes beyond detailed timings come from display/edid_modes in the
drm_edid_connector_add_modes order), link/protocols/link_dp_capability.c
decide_dp_link_settings (lane count first, then rate), dc_link_dp_bandwidth
(8b/10b without FEC), drm_edid.c drm_do_get_edid (E-DDC segment pointer 0x30,
address 0x50), amdgpu_dm.c dm_suspend/dm_resume GPU-reset path (commit zero
streams, then after dm_dmub_hw_init and dc_set_power_state D0 commit the cached
streams again with their link settings; no detection) for restore().
dcn-dsc-odm (behind DisplayDsc/DisplayOdm/DisplayOverlay): link_detection.c
retrieve_link_cap DSC/FEC capability reads (DP_DSC_SUPPORT 16 bytes,
DP_DSC_BRANCH_OVERALL_THROUGHPUT_0 3 bytes, DP_FEC_CAPABILITY), amdgpu_dm
apply_dsc_policy_for_stream (dc_dsc_compute_config against the verified link
bandwidth with FEC overhead, then decide_dp_link_settings for the compressed
rate), DSC only when the mode does not fit uncompressed; dc_update_planes_and_stream
for the driver-internal overlay plane.
*/
#include "dcn32_stream_native.hpp"

#include "dcn32_cursor.hpp"
#include "hdmi_scdc.hpp"
#include "video_helpers.hpp"

namespace rx7900::reactos {
namespace {

constexpr ULONG workspace_pool_tag = 'dSXR';
constexpr ULONG stall_limit_microseconds = 50;
constexpr uint32_t maximum_wait_microseconds = 100000;
constexpr uint64_t surface_alignment = 64u * 1024u;
constexpr uint32_t pitch_alignment_pixels = 64;    // 256-byte linear pitch
constexpr ULONG clear_chunk_bytes = 1u << 20;
constexpr uint8_t edid_address = 0x50;
constexpr uint8_t edid_segment_address = 0x30;
constexpr uint32_t dprx_feature_enumeration_list = 0x2210;
constexpr uint8_t vsc_sdp_colorimetry_supported = 0x08;
constexpr uint8_t dp_rates[] = {0x06, 0x0a, 0x14, 0x1e};
constexpr uint8_t dp_lane_counts[] = {1, 2, 4};
constexpr uint32_t dpcd_fec_capability = 0x090;
constexpr uint8_t fec_capable_bit = 0x01;
constexpr uint8_t dpcd_revision_14 = 0x14;

namespace r = display::dcn32_stream_reg;

VP_STATUS NTAPI read_dword(PVOID, PVOID context, PWSTR, PVOID data, ULONG length) {
    if (context == nullptr || data == nullptr || length != sizeof(ULONG)) {
        return ERROR_INVALID_PARAMETER;
    }
    *static_cast<ULONG*>(context) = *static_cast<const ULONG*>(data);
    return NO_ERROR;
}

template <size_t N>
bool registry_enabled(PVOID extension, const wchar_t (&value_name)[N]) noexcept {
    wchar_t name[N];
    for (size_t index = 0; index < N; ++index) {
        name[index] = value_name[index];
    }
    ULONG value = 0;
    return VideoPortGetRegistryParameters(extension, name, FALSE, read_dword, &value) == NO_ERROR && value == 1;
}

display::LinkCall link_call(output::LinkResult result) noexcept {
    switch (result) {
    case output::LinkResult::success: return display::LinkCall::success;
    case output::LinkResult::quarantined: return display::LinkCall::uncertain;
    default: return display::LinkCall::refused;
    }
}

uint32_t align_up(uint32_t value, uint32_t alignment) noexcept {
    return (value + alignment - 1u) / alignment * alignment;
}

bool same_timing(const display::Timing& a, const display::Timing& b) noexcept {
    return a.pixel_clock_khz == b.pixel_clock_khz && a.width == b.width && a.h_total == b.h_total &&
           a.h_front_porch == b.h_front_porch && a.h_sync_width == b.h_sync_width && a.height == b.height &&
           a.v_total == b.v_total && a.v_front_porch == b.v_front_porch && a.v_sync_width == b.v_sync_width &&
           a.flags == b.flags;
}

bool same_geometry(const display::Timing& a, const display::Timing& b) noexcept {
    return a.width == b.width && a.h_total == b.h_total && a.h_front_porch == b.h_front_porch &&
           a.h_sync_width == b.h_sync_width && a.height == b.height && a.v_total == b.v_total &&
           a.v_front_porch == b.v_front_porch && a.v_sync_width == b.v_sync_width && a.flags == b.flags;
}

// Runs body with the caller's x87/SSE state saved (Linux DC_FP_START).
template <typename Body>
bool with_fpu(Body body) noexcept {
    KFLOATING_SAVE save{};
    if (!NT_SUCCESS(KeSaveFloatingPointState(&save))) {
        return false;
    }
    const bool result = body();
    KeRestoreFloatingPointState(&save);
    return result;
}

} // namespace

// ---- RegisterIo over BAR5 ----

bool NativeDisplayStream::read32(void* context, uint32_t offset, uint32_t& value) noexcept {
    auto& stream = *static_cast<NativeDisplayStream*>(context);
    return stream.inputs_.hardware != nullptr && read_register(*stream.inputs_.hardware, offset, value);
}

bool NativeDisplayStream::write32(void* context, uint32_t offset, uint32_t value) noexcept {
    auto& stream = *static_cast<NativeDisplayStream*>(context);
    return stream.inputs_.hardware != nullptr && write_register(*stream.inputs_.hardware, offset, value);
}

void NativeDisplayStream::delay(void*, uint32_t microseconds) noexcept {
    if (microseconds > maximum_wait_microseconds) {
        microseconds = maximum_wait_microseconds;
    }
    if (microseconds <= stall_limit_microseconds) {
        KeStallExecutionProcessor(microseconds);
        return;
    }
    LARGE_INTEGER interval{};
    interval.QuadPart = -10LL * static_cast<LONGLONG>(microseconds);
    KeDelayExecutionThread(KernelMode, FALSE, &interval);
}

// ---- link owner hooks ----

output::TmdsKind NativeDisplayStream::tmds_kind(display::StreamSignal signal) const noexcept {
    return signal == display::StreamSignal::hdmi ? output::TmdsKind::hdmi : output::TmdsKind::dvi_single;
}

display::LinkCall NativeDisplayStream::hook_update_clocks(void* context, const display::Dcn32StreamClocks& clocks,
                                                          bool safe_to_lower) noexcept {
    auto& stream = *static_cast<NativeDisplayStream*>(context);
    display::DisplayClocks requested{};
    requested.dispclk_khz = clocks.clocks.dispclk_khz;
    requested.dppclk_khz = clocks.clocks.dppclk_khz;
    if (clocks.pipe < display::display_pipe_count) {
        requested.dpp_pipe_khz[clocks.pipe] = clocks.dpp_khz;
    }
    // dcn-dsc-odm: with ODM or an overlay every pipe of the context has its DTO.
    for (uint32_t pipe = 0; pipe < display::dcn32_stream_pipes && pipe < display::display_pipe_count; ++pipe) {
        if (clocks.dpp_pipe_khz[pipe] != 0) {
            requested.dpp_pipe_khz[pipe] = clocks.dpp_pipe_khz[pipe];
        }
    }
    requested.dcfclk_khz = clocks.clocks.dcfclk_khz;
    requested.dcfclk_deep_sleep_khz = clocks.clocks.dcfclk_deep_sleep_khz;
    requested.socclk_khz = clocks.clocks.socclk_khz;
    requested.fclk_khz = clocks.clocks.fclk_khz;
    requested.dramclk_khz = clocks.clocks.dramclk_khz;
    requested.ref_dtbclk_khz = clocks.clocks.ref_dtbclk_khz;
    requested.display_count = clocks.display_count;
    requested.p_state_change_support = clocks.clocks.p_state_change_support;
    requested.fclk_p_state_change_support = clocks.clocks.fclk_p_state_change_support;
    requested.dtbclk_en = clocks.clocks.dtbclk_en;
    display::DisplayClocks applied{};
    return link_call(stream.inputs_.link->owner().set_display_clocks(requested, safe_to_lower, applied));
}

display::LinkCall NativeDisplayStream::hook_set_pixel_clock(void* context, const display::Dcn32StreamPlan& plan) noexcept {
    auto& stream = *static_cast<NativeDisplayStream*>(context);
    return link_call(stream.inputs_.link->owner().set_pixel_clock(
        stream.lease_, plan.pipe, plan.requested_pixel_100hz, stream.tmds_kind(plan.signal), output::TmdsDepth::bpc8,
        plan.pixel_rate_k1, plan.pixel_rate_k2));
}

display::LinkCall NativeDisplayStream::hook_power_down(void* context, const display::Dcn32StreamPlan& plan) noexcept {
    // dce110_clock_source_power_down: the DP DTO clock source returns at once;
    // a TMDS PHY PLL is powered down (SetPixelClock 0).
    auto& stream = *static_cast<NativeDisplayStream*>(context);
    if (plan.signal == display::StreamSignal::display_port) {
        return display::LinkCall::success;
    }
    return link_call(stream.inputs_.link->owner().clear_pixel_clock(stream.lease_, plan.pipe));
}

display::LinkCall NativeDisplayStream::hook_enable_link(void* context, const display::Dcn32StreamPlan& plan) noexcept {
    auto& stream = *static_cast<NativeDisplayStream*>(context);
    output::DisplayLinkOwner& owner = stream.inputs_.link->owner();
    if (plan.signal == display::StreamSignal::display_port) {
        uint32_t sink_index = display_stream_no_index;
        for (uint32_t index = 0; index < stream.mode_count_; ++index) {
            if (same_timing(stream.modes_[index].timing, plan.timing)) {
                sink_index = stream.modes_[index].sink_mode_index;
                break;
            }
        }
        if (sink_index == display_stream_no_index || stream.aux_io_.transfer == nullptr) {
            return display::LinkCall::refused;
        }
        // disable_link leaves a DP route's DDC pad back in GPIO mode (Linux
        // opens it for every AUX transaction): AUX mode again before training.
        const output::LinkResult opened = owner.prepare_aux(stream.lease_);
        if (opened != output::LinkResult::success) {
            return link_call(opened);
        }
        output::DpTrainingRequest request{plan.dp_rate_code, plan.dp_lanes, sink_index};
        // dcn-dsc-odm: a DSC stream trains with FEC and is admitted compressed.
        request.fec_ready = plan.fec;
        request.dsc_bits_per_pixel_x16 = plan.dsc ? plan.dsc_config.bits_per_pixel : 0u;
        output::DpTrainedLink trained{};
        return link_call(owner.enable_dp_link(stream.lease_, stream.aux_io_, stream.dp_port_, request,
                                              static_cast<uint8_t>(plan.engine), trained));
    }
    output::ScdcReport scdc{};
    return link_call(owner.enable_tmds_link(stream.lease_, plan.tmds_khz, static_cast<uint8_t>(plan.engine),
                                            stream.tmds_kind(plan.signal), plan.lte_340_scramble, scdc));
}

display::LinkCall NativeDisplayStream::hook_disable_link(void* context, const display::Dcn32StreamPlan&) noexcept {
    auto& stream = *static_cast<NativeDisplayStream*>(context);
    return link_call(stream.inputs_.link->owner().disable_link(stream.lease_));
}

display::LinkCall NativeDisplayStream::hook_p_state_delegate(void* context, bool should_manage_pstate) noexcept {
    auto& stream = *static_cast<NativeDisplayStream*>(context);
    return link_call(stream.inputs_.link->owner().p_state_delegate(should_manage_pstate));
}

display::LinkCall NativeDisplayStream::hook_hdmi_scdc_legacy(void* context, const display::Dcn32StreamPlan&) noexcept {
    auto& stream = *static_cast<NativeDisplayStream*>(context);
    return link_call(stream.inputs_.link->owner().hdmi_scdc_legacy(stream.lease_));
}

display::LinkCall NativeDisplayStream::hook_dpcd_read(void* context, uint32_t address, uint8_t& value) noexcept {
    auto& stream = *static_cast<NativeDisplayStream*>(context);
    return stream.aux_io_.transfer != nullptr &&
                   output::dp_aux_read(stream.aux_io_, address, &value, 1) == output::DpProbeStatus::success ?
               display::LinkCall::success : display::LinkCall::refused;
}

display::LinkCall NativeDisplayStream::hook_dpcd_write(void* context, uint32_t address, uint8_t value) noexcept {
    auto& stream = *static_cast<NativeDisplayStream*>(context);
    return stream.aux_io_.transfer != nullptr &&
                   output::dp_aux_write(stream.aux_io_, address, &value, 1) == output::DpProbeStatus::success ?
               display::LinkCall::success : display::LinkCall::refused;
}

// ---- discovery ----

bool NativeDisplayStream::find_firmware_route(uint32_t otg) noexcept {
    // The DIG FE the firmware connected to the adopted OTG, clocked (symclk on).
    const output::Dcn32RouteTable& routes = inputs_.link->owner().routes();
    const uint32_t segment2 = inputs_.preos->registers.segment(2);
    uint32_t found = 0;
    for (uint32_t index = 0; index < routes.count; ++index) {
        const output::Dcn32ConnectorRoute& route = routes.routes[index];
        if (!route.validated || route.dig_engine >= display::dcn32_encoder::engine_count ||
            route.signal == output::LinkSignal::dvi_dual || route.signal == output::LinkSignal::none) {
            continue;
        }
        uint32_t value = 0;
        const uint32_t offset = (segment2 + r::DIG_FE_CNTL::reg.offset[route.dig_engine]) * 4u;
        if (!read32(this, offset, value)) {
            return false;
        }
        if (display::field_of(value, r::DIG_FE_CNTL::DIG_SOURCE_SELECT) == otg &&
            display::field_of(value, r::DIG_FE_CNTL::DIG_SYMCLK_FE_ON) == 1) {
            route_ = route;
            ++found;
        }
    }
    return found == 1;
}

bool NativeDisplayStream::read_tmds_edid() noexcept {
    // drm_do_get_edid over the DC I2C engine (dce_i2c_hw) of the route's DDC line.
    uint32_t xtal = 0;
    if (!display::read_dce_refclk_khz(inputs_.rom, xtal)) {
        return false;
    }
    output::I2cEngineConfig config{};
    config.io = {this, inputs_.hardware->register_bytes, read32, write32, delay};
    config.space = {inputs_.preos->registers.segment(2), inputs_.hardware->register_bytes, 3, 2, 0, true};
    config.dccg_segment = inputs_.preos->registers.segment(1);
    config.xtalin_khz = xtal;
    config.speed_khz = 100;
    uint32_t blocks = 1;
    for (uint32_t block = 0; block < blocks; ++block) {
        uint8_t* destination = &edid_[block * output::edid_block_bytes];
        const uint8_t segment = static_cast<uint8_t>(block >> 1);
        const uint8_t start = static_cast<uint8_t>((block & 1u) * output::edid_block_bytes);
        output::I2cPayload payloads[3]{};
        uint32_t count = 0;
        if (segment != 0) {
            payloads[count++] = {true, edid_segment_address, 1, &segment, nullptr};
        }
        payloads[count++] = {true, edid_address, 1, &start, nullptr};
        payloads[count++] = {false, edid_address, static_cast<uint8_t>(output::edid_block_bytes), nullptr, destination};
        if (output::dcn32_i2c_command(config, route_.ddc_line, payloads, count) != output::I2cResult::success) {
            return false;
        }
        if (block == 0) {
            const uint32_t extensions = destination[126];
            blocks = 1 + (extensions < output::maximum_edid_blocks - 1 ? extensions : output::maximum_edid_blocks - 1);
        }
    }
    edid_bytes_ = blocks * output::edid_block_bytes;
    return true;
}

void NativeDisplayStream::widen_sink_modes(ByteView edid, output::SinkSnapshot& sink) noexcept {
    // The detailed timings keep their probe indices and every other EDID mode
    // is appended behind them, so DpTrainingRequest.sink_mode_index and
    // prepare_output_mode name any advertised mode. A blob the decoder refuses
    // leaves the detailed timings only.
    for (display::EdidModeSource& source : mode_sources_) {
        source = display::EdidModeSource::detailed;
    }
    edid_report_ = display::EdidModeReport{};
    display::append_edid_modes(edid, sink.detailed_modes, mode_sources_, edid_report_);
}

bool NativeDisplayStream::read_sink() noexcept {
    if (route_.signal == output::LinkSignal::display_port) {
        if (!inputs_.link->bind_aux(lease_, hpd_, aux_)) {
            return false;
        }
        aux_io_ = aux_.make_aux_io();
        if (output::probe_dp_port(aux_io_, output::dcn32_default_dp_source(), dp_scratch_, dp_port_) !=
            output::DpProbeStatus::success) {
            return false;
        }
        widen_sink_modes(ByteView(dp_scratch_.edid, dp_port_.sink.block_count * output::edid_block_bytes),
                         dp_port_.sink);
        sink_ = dp_port_.sink;
        // amdgpu_dm: VSC SDP colorimetry with DPCD 1.4+ and DPRX VSC_SDP_COLORIMETRY_SUPPORTED.
        dp_use_vsc_ = false;
        if (dp_port_.receiver.dpcd_revision >= 0x14) {
            uint8_t features = 0;
            if (output::dp_aux_read(aux_io_, dprx_feature_enumeration_list, &features, 1) ==
                output::DpProbeStatus::success) {
                dp_use_vsc_ = (features & vsc_sdp_colorimetry_supported) != 0;
            }
        }
        read_dsc_caps();
        firmware_.signal = display::StreamSignal::display_port;
        return true;
    }
    if (!read_tmds_edid() || output::parse_edid(ByteView(edid_, edid_bytes_), sink_) != output::Status::success ||
        display::parse_hdmi_sink(ByteView(edid_, edid_bytes_), hdmi_) != display::HdmiStatus::success) {
        return false;
    }
    widen_sink_modes(ByteView(edid_, edid_bytes_), sink_);
    // link_detection.c: an HDMI connector without an HDMI VSDB drives DVI.
    firmware_.signal = hdmi_.is_hdmi ? display::StreamSignal::hdmi : display::StreamSignal::dvi_single;
    return true;
}

void NativeDisplayStream::read_dsc_caps() noexcept {
    // retrieve_link_cap (DPCD 1.4+): DSC basic and branch capabilities, then
    // DP_FEC_CAPABILITY. Any failed read leaves DSC off (not a start failure).
    dsc_capable_ = false;
    dsc_sink_ = display::DscSinkCaps{};
    if (!allow_dsc_ || dp_port_.receiver.dpcd_revision < dpcd_revision_14) {
        return;
    }
    uint8_t basic[display::dsc_dpcd_caps_bytes]{};
    uint8_t branch[display::dsc_branch_caps_bytes]{};
    uint8_t fec = 0;
    if (output::dp_aux_read(aux_io_, display::dpcd_dsc_support, basic, sizeof(basic)) !=
            output::DpProbeStatus::success ||
        output::dp_aux_read(aux_io_, display::dpcd_dsc_branch_caps, branch, sizeof(branch)) !=
            output::DpProbeStatus::success ||
        output::dp_aux_read(aux_io_, dpcd_fec_capability, &fec, 1) != output::DpProbeStatus::success) {
        return;
    }
    // amdgpu_dm passes the branch bytes (zero for a non-branch sink): is_dp.
    dsc_capable_ = display::parse_dsc_dpcd(basic, branch, dsc_sink_) && dsc_sink_.is_dsc_supported &&
                   (fec & fec_capable_bit) != 0;
}

bool NativeDisplayStream::configure_bandwidth() noexcept {
    const display::DisplayClockTable* table = inputs_.link->owner().clock_table();
    if (table == nullptr || !table->valid) {
        return false;
    }
    soc_ = display::Dcn32SocInputs{};
    const auto copy = [](const display::ClockLevels& levels, uint32_t display::Dcn32ClockLevel::*member,
                         display::Dcn32SocInputs& soc, uint32_t& count) noexcept {
        count = levels.count < display::dcn32_max_dpm_levels ? levels.count : display::dcn32_max_dpm_levels;
        for (uint32_t index = 0; index < count; ++index) {
            soc.clocks.levels[index].*member = levels.mhz[index];
        }
        soc.clocks.dc_mode_limit.*member = levels.dc_mode_max_mhz;
    };
    using L = display::Dcn32ClockLevel;
    display::Dcn32ClockTable& clocks = soc_.clocks;
    copy(table->dcfclk, &L::dcfclk_mhz, soc_, clocks.num_dcfclk_levels);
    copy(table->fclk, &L::fclk_mhz, soc_, clocks.num_fclk_levels);
    copy(table->uclk, &L::memclk_mhz, soc_, clocks.num_memclk_levels);
    copy(table->socclk, &L::socclk_mhz, soc_, clocks.num_socclk_levels);
    copy(table->dtbclk, &L::dtbclk_mhz, soc_, clocks.num_dtbclk_levels);
    copy(table->dispclk, &L::dispclk_mhz, soc_, clocks.num_dispclk_levels);
    copy(table->dppclk, &L::dppclk_mhz, soc_, clocks.num_dppclk_levels);
    soc_.num_chans = table->memory_info_valid ? table->num_channels : 0;
    soc_.dram_channel_width_bytes = table->memory_info_valid ? table->dram_channel_width_bytes : 0;
    soc_.dentist_vco_khz = table->dentist_vco_khz;
    soc_.dchub_refclk_khz = table->dchub_ref_khz;
    dchub_refclk_khz_ = table->dchub_ref_khz;
    // dcn32_calc_num_avail_chans_for_mall (GC 11.0.0) * mall_size_per_mem_channel.
    max_cab_bytes_ = (soc_.num_chans == 48 ? 32u : 16u) * 4u * 1024u * 1024u;

    if (workspace_storage_ == nullptr) {
        const uint32_t bytes = display::dcn32_bandwidth_workspace_bytes();
        workspace_storage_ = ExAllocatePoolWithTag(NonPagedPool, bytes, workspace_pool_tag);
        if (workspace_storage_ == nullptr) {
            return false;
        }
        workspace_ = display::construct_dcn32_bandwidth_workspace(workspace_storage_, bytes);
        if (workspace_ == nullptr) {
            ExFreePoolWithTag(workspace_storage_, workspace_pool_tag);
            workspace_storage_ = nullptr;
            return false;
        }
    }
    return with_fpu([this]() noexcept {
        return bandwidth_.configure(*workspace_, soc_) == display::Dcn32BandwidthStatus::ready;
    });
}

bool NativeDisplayStream::bandwidth_for(const DisplayStreamMode& mode, uint32_t pitch_pixels,
                                        const DisplayOverlay* overlay) noexcept {
    display::Dcn32PlaneRequest request{};
    request.timing = mode.timing;
    request.signal = mode.signal == display::StreamSignal::display_port ? display::Dcn32OutputSignal::display_port
                                                                          : display::Dcn32OutputSignal::tmds;
    request.pitch_pixels = pitch_pixels;
    request.otg_instance = firmware_.pipe;
    // dcn-dsc-odm: ODM opt-in, the DSC rate, the overlay context and the
    // current pipes (dcn32 free-pipe policies; the firmware state is one pipe).
    request.allow_odm = allow_odm_;
    request.dsc_slices_h = mode.dsc.num_slices_h;
    request.dsc_bits_per_pixel_x16 = mode.dsc.num_slices_h != 0 ? mode.dsc.bits_per_pixel : 0u;
    const display::Dcn32StreamPlan* current = controller_.current();
    request.current = current != nullptr ? &current->bandwidth.topology : nullptr;
    if (overlay != nullptr) {
        request.plane_count = 2;
        request.planes[0] = display::Dcn32PlaneGeometry{
            overlay->width, overlay->height,
            display::Dcn32Rect{overlay->x, overlay->y, static_cast<int32_t>(overlay->width),
                               static_cast<int32_t>(overlay->height)}};
        request.plane_pitch_pixels[0] = overlay->pitch_pixels;
        request.planes[1] = display::Dcn32PlaneGeometry{
            mode.timing.width, mode.timing.height,
            display::Dcn32Rect{0, 0, static_cast<int32_t>(mode.timing.width), static_cast<int32_t>(mode.timing.height)}};
        request.plane_pitch_pixels[1] = pitch_pixels;
    }
    return with_fpu([this, &request]() noexcept {
        return bandwidth_.validate(request, bandwidth_plan_) == display::Dcn32BandwidthStatus::ready;
    });
}

bool NativeDisplayStream::decide_dp_link(DisplayStreamMode& mode) noexcept {
    // decide_dp_link_settings: the lowest rate, and at that rate the fewest
    // lanes, that carries the mode (8b/10b, no FEC: the link owner trains without it).
    const uint8_t max_rate = dp_port_.receiver.maximum_rate_code < 0x1e ? dp_port_.receiver.maximum_rate_code : 0x1e;
    const uint8_t max_lanes = dp_port_.receiver.maximum_lanes < 4 ? dp_port_.receiver.maximum_lanes : 4;
    for (const uint8_t rate : dp_rates) {
        if (rate > max_rate) {
            break;
        }
        for (const uint8_t lanes : dp_lane_counts) {
            if (lanes > max_lanes) {
                break;
            }
            output::LinkCapabilities caps{};
            display::Timing admitted{};
            if (output::dp_link_budget(rate, lanes, true, caps) == output::DpProbeStatus::success &&
                output::prepare_output_mode(sink_, mode.sink_mode_index, caps, admitted) == output::Status::success) {
                mode.dp_rate_code = rate;
                mode.dp_lanes = lanes;
                mode.dsc = display::DscConfig{};
                mode.fec = false;
                return true;
            }
        }
    }
    return decide_dsc_link(mode);
}

bool NativeDisplayStream::decide_dsc_link(DisplayStreamMode& mode) noexcept {
    // apply_dsc_policy_for_stream: dc_dsc_compute_config with the verified link
    // bandwidth (maximum rate and lanes, 8b/10b with FEC), then the lowest link
    // that carries the compressed stream (dc_dsc_stream_bandwidth_in_kbps) and
    // that the link owner admits (compressed payload within 97 percent).
    if (!allow_dsc_ || !dsc_capable_) {
        return false;
    }
    const uint8_t max_rate = dp_port_.receiver.maximum_rate_code < 0x1e ? dp_port_.receiver.maximum_rate_code : 0x1e;
    const uint8_t max_lanes = dp_port_.receiver.maximum_lanes < 4 ? dp_port_.receiver.maximum_lanes : 4;
    const display::DscTiming timing{mode.timing.pixel_clock_khz * 10u, mode.timing.width, mode.timing.height,
                                    mode.timing.h_total, mode.timing.v_total};
    display::DscConfig config{};
    if (!display::dsc_compute_config(dsc_sink_, display::dp_link_bandwidth_kbps(max_rate, max_lanes, true), timing,
                                     config)) {
        return false;
    }
    const uint32_t needed_kbps =
        display::dsc_stream_bandwidth_kbps(timing, config.bits_per_pixel, config.num_slices_h, true);
    const uint64_t payload_bits = static_cast<uint64_t>(mode.timing.pixel_clock_khz) * 1000u * config.bits_per_pixel / 16u;
    for (const uint8_t rate : dp_rates) {
        if (rate > max_rate) {
            break;
        }
        for (const uint8_t lanes : dp_lane_counts) {
            if (lanes > max_lanes) {
                break;
            }
            output::LinkCapabilities caps{};
            if (display::dp_link_bandwidth_kbps(rate, lanes, true) >= needed_kbps &&
                output::dp_link_budget(rate, lanes, true, caps) == output::DpProbeStatus::success &&
                payload_bits <= caps.usable_payload_bits_per_second / 100u * 97u) {
                mode.dp_rate_code = rate;
                mode.dp_lanes = lanes;
                mode.dsc = config;
                mode.fec = true;
                return true;
            }
        }
    }
    return false;
}

bool NativeDisplayStream::dsc_mode_valid(const DisplayStreamMode& mode) noexcept {
    // The stream plan's DSC rules for the validated ODM slice count: slices and
    // width divide by it and both dsc_prepare_config calls (one ODM slice's
    // engine, the whole-picture PPS) accept the configuration.
    const uint32_t odm = bandwidth_plan_.odm_slices == 0 ? 1u : bandwidth_plan_.odm_slices;
    if (mode.dsc.num_slices_h % odm != 0 || mode.timing.width % odm != 0) {
        return false;
    }
    display::DscOptcConfig optc{};
    display::DscConfig engine = mode.dsc;
    engine.num_slices_h = mode.dsc.num_slices_h / odm;
    return display::dsc_prepare_config(mode.timing.width / odm, mode.timing.height, odm > 1, engine, dsc_scratch_,
                                       optc) &&
           display::dsc_prepare_config(mode.timing.width, mode.timing.height, odm > 1, mode.dsc, dsc_scratch_, optc);
}

bool NativeDisplayStream::validate_mode(DisplayStreamMode& mode) noexcept {
    if (display::refresh_millihz(mode.timing, mode.refresh_millihz) != display::Status::success) {
        return false;
    }
    if (mode.signal == display::StreamSignal::display_port) {
        if (mode.sink_mode_index == display_stream_no_index || !decide_dp_link(mode)) {
            return false;
        }
    } else {
        if (display::plan_tmds_stream(mode.timing, hdmi_, mode.tmds) != display::HdmiStatus::success) {
            return false;
        }
        mode.signal = mode.tmds.signal == display::TmdsSignal::hdmi ? display::StreamSignal::hdmi
                                                                     : display::StreamSignal::dvi_single;
    }
    const uint32_t pitch = mode.firmware ? static_cast<uint32_t>(inputs_.preos->scanout.stride_bytes / 4u)
                                         : align_up(mode.timing.width, pitch_alignment_pixels);
    mode.committable = bandwidth_for(mode, pitch) && (mode.dsc.num_slices_h == 0 || dsc_mode_valid(mode));
    return mode.committable;
}

void NativeDisplayStream::build_modes() noexcept {
    mode_count_ = 0;
    const display::AdoptedScanout& scanout = inputs_.preos->scanout;
    DisplayStreamMode& gop = modes_[mode_count_++];
    gop = DisplayStreamMode{};
    gop.firmware = true;
    gop.timing = scanout.timing;
    gop.refresh_millihz = scanout.refresh_millihz;
    gop.signal = firmware_.signal;
    // A TMDS pre-OS clock is not derivable from DCCG: take the pixel clock of a
    // sink timing with the same geometry when one exists.
    for (uint32_t index = 0; index < sink_.detailed_modes.count(); ++index) {
        display::Timing timing{};
        if (sink_.detailed_modes.get(index, timing) != display::Status::success || !same_geometry(timing, gop.timing)) {
            continue;
        }
        if (gop.timing.pixel_clock_khz == 0 || timing.pixel_clock_khz == gop.timing.pixel_clock_khz) {
            gop.timing.pixel_clock_khz = timing.pixel_clock_khz;
            gop.sink_mode_index = index;
            break;
        }
    }
    if (gop.timing.pixel_clock_khz != 0) {
        const uint32_t refresh = gop.refresh_millihz;
        validate_mode(gop);
        if (refresh != 0) {
            gop.refresh_millihz = refresh;
        }
    }
    // One VideoPort mode per size and refresh, EDID source order, validated.
    mode_count_ = display::select_mode_list(
        sink_.detailed_modes, mode_count_, display_stream_mode_capacity,
        [this](uint32_t slot) noexcept {
            return display::listed_identity(modes_[slot].timing, modes_[slot].refresh_millihz);
        },
        [this](uint32_t slot, uint32_t index, const display::Timing& timing, uint32_t) noexcept {
            DisplayStreamMode& mode = modes_[slot];
            mode = DisplayStreamMode{};
            mode.timing = timing;
            mode.signal = firmware_.signal;
            mode.sink_mode_index = index;
            mode.source = mode_sources_[index];
            return validate_mode(mode);
        });
}

DisplayStreamStart NativeDisplayStream::start(const NativeDisplayStreamInputs& inputs) noexcept {
    if (started_ || inputs.hardware == nullptr || inputs.bringup == nullptr || inputs.preos == nullptr ||
        inputs.link == nullptr || inputs.extension == nullptr || inputs.epoch == 0 ||
        KeGetCurrentIrql() != PASSIVE_LEVEL) {
        return start_result_ = DisplayStreamStart::invalid_argument;
    }
    inputs_ = inputs;
    const PreosScanoutNative& preos = *inputs.preos;
    const display::AdoptedScanout& scanout = preos.scanout;
    if (!preos.adopted || scanout.otg != scanout.pipe || scanout.opp != scanout.pipe || scanout.mpcc != scanout.pipe ||
        scanout.pipe >= display::dcn32_stream_pipes || scanout.variable_refresh || (scanout.stride_bytes % 256u) != 0 ||
        (scanout.mc_address & 0xffu) != 0) {
        return start_result_ = DisplayStreamStart::firmware_pipe_unsupported;
    }
    if (!inputs.link->bound() || !inputs.link->owner().bound() || inputs.link->owner().quarantined()) {
        return start_result_ = DisplayStreamStart::link_unavailable;
    }
    const hardware::RegisterIo io{this, inputs.hardware->register_bytes, read32, write32, delay};
    const display::Dcn32StreamHooks hooks{this,           hook_update_clocks,   hook_set_pixel_clock,
                                          hook_power_down, hook_enable_link,     hook_disable_link,
                                          hook_dpcd_read,  hook_dpcd_write,      hook_p_state_delegate,
                                          hook_hdmi_scdc_legacy};
    if (!controller_.bind(io, preos.registers.segment(1), preos.registers.segment(2), preos.registers.segment(3),
                          hooks)) {
        return start_result_ = DisplayStreamStart::controller_unavailable;
    }
    if (!find_firmware_route(scanout.otg) ||
        inputs.link->owner().acquire_route(route_.firmware_path_index, inputs.epoch, lease_) !=
            output::LinkResult::success) {
        controller_.unbind();
        return start_result_ = DisplayStreamStart::route_unavailable;
    }
    lease_held_ = true;
    allow_dsc_ = registry_enabled(inputs.extension, display_dsc_registry_value);
    allow_odm_ = registry_enabled(inputs.extension, display_odm_registry_value);
    allow_overlay_ = registry_enabled(inputs.extension, display_overlay_registry_value);
    overlay_active_ = false;
    if (!inputs.link->bind_hpd(lease_, hpd_) || !read_sink()) {
        return start_result_ = DisplayStreamStart::sink_unavailable;
    }
    bool adopted = false;
    if (inputs.link->owner().adopt_firmware_link(lease_, tmds_kind(firmware_.signal),
                                                 static_cast<uint8_t>(route_.dig_engine), adopted) !=
        output::LinkResult::success) {
        return start_result_ = DisplayStreamStart::route_unavailable;
    }
    if (!configure_bandwidth()) {
        return start_result_ = workspace_storage_ == nullptr ? DisplayStreamStart::workspace_unavailable
                                                              : DisplayStreamStart::clocks_unavailable;
    }
    // gfx_v11_0_gpu_early_init gb_addr_config_fields (amdgpu_dm gfx9 tiling info).
    const uint32_t gb = inputs.bringup->asic.gfx_report().gb_addr_config;
    tiling_.num_pipes = 1u << (gb & 0x7u);
    tiling_.pipe_interleave_bytes = 1u << (8u + ((gb >> 3) & 0x7u));
    tiling_.max_compressed_frags = 1u << ((gb >> 6) & 0x3u);
    tiling_.num_pkrs = 1u << ((gb >> 8) & 0x7u);

    firmware_.timing = scanout.timing;
    firmware_.pipe = scanout.pipe;
    firmware_.engine = route_.dig_engine;
    build_modes();
    if (mode_count_ == 0 || !controller_.adopt_firmware(firmware_)) {
        return start_result_ = DisplayStreamStart::bandwidth_unavailable;
    }
    DisplayStreamSurface& gop = surfaces_[0];
    gop = DisplayStreamSurface{};
    gop.vram_offset = scanout.vram_offset;
    gop.mc_address = scanout.mc_address;
    gop.bytes = scanout.bytes;
    gop.pitch_pixels = scanout.stride_bytes / 4u;
    gop.width = scanout.timing.width;
    gop.height = scanout.timing.height;
    gop.used = true;
    gop.owned = false;
    gop.scanned = true;
    current_surface_ = 0;
    current_mode_ = 0;
    mode_set_ = false;
    started_ = true;
    return start_result_ = DisplayStreamStart::started;
}

// ---- surfaces ----

uint32_t NativeDisplayStream::acquire_surface(uint32_t index) noexcept {
    const DisplayStreamMode& mode = modes_[index];
    if (mode.firmware) {
        return 0;
    }
    for (uint32_t slot = 1; slot < display_stream_surface_capacity; ++slot) {
        DisplayStreamSurface& surface = surfaces_[slot];
        if (surface.used) {
            continue;
        }
        const uint32_t pitch = align_up(mode.timing.width, pitch_alignment_pixels);
        const uint64_t bytes = static_cast<uint64_t>(pitch) * 4u * mode.timing.height;
        const uint64_t rounded = (bytes + surface_alignment - 1) / surface_alignment * surface_alignment;
        // amdgpu_bo_create_kernel-style pinned contiguous CPU-visible VRAM.
        const hardware::VramRequest request{rounded, surface_alignment, hardware::vram_kernel_bo_flags(true)};
        hardware::VramAllocationHandle handle{};
        if (inputs_.bringup->vram_allocator.allocate(request, handle) != hardware::VramStatus::success) {
            return display_stream_no_index;
        }
        hardware::VramAllocationInfo info{};
        if (inputs_.bringup->vram_allocator.describe(handle, info) != hardware::VramStatus::success ||
            !info.contiguous || info.no_cpu_access || info.visible_bytes < rounded) {
            inputs_.bringup->vram_allocator.release(handle);
            return display_stream_no_index;
        }
        surface = DisplayStreamSurface{};
        surface.allocation = handle;
        surface.vram_offset = info.first_byte;
        surface.mc_address = inputs_.preos->vram.mc_start + info.first_byte;
        surface.bytes = bytes;
        surface.pitch_pixels = pitch;
        surface.width = mode.timing.width;
        surface.height = mode.timing.height;
        surface.used = true;
        surface.owned = true;
        return slot;
    }
    return display_stream_no_index;
}

bool NativeDisplayStream::clear_surface(const DisplayStreamSurface& surface) noexcept {
    for (uint64_t done = 0; done < surface.bytes;) {
        const uint64_t remaining = surface.bytes - done;
        const ULONG chunk = remaining < clear_chunk_bytes ? static_cast<ULONG>(remaining) : clear_chunk_bytes;
        PUCHAR view = nullptr;
        if (map_vram_temporary(*inputs_.hardware, inputs_.extension, surface.vram_offset + done, chunk, view) !=
            NO_ERROR) {
            return false;
        }
        VideoPortZeroDeviceMemory(view, chunk);
        unmap_vram_temporary(inputs_.extension, view);
        done += chunk;
    }
    return true;
}

void NativeDisplayStream::release_unused_surfaces() noexcept {
    for (DisplayStreamSurface& surface : surfaces_) {
        if (surface.used && surface.owned && !surface.scanned && surface.mapping == nullptr &&
            inputs_.bringup->vram_allocator.release(surface.allocation) == hardware::VramStatus::success) {
            surface = DisplayStreamSurface{};
        }
    }
}

// ---- commit ----

bool NativeDisplayStream::build_plan(uint32_t mode_index, const DisplayStreamSurface& surface,
                                     const DisplayOverlay* overlay, uint64_t overlay_mc_address) noexcept {
    const DisplayStreamMode& mode = modes_[mode_index];
    request_ = display::Dcn32StreamRequest{};
    request_.timing = mode.timing;
    request_.signal = mode.signal;
    request_.pipe = firmware_.pipe;
    request_.engine = firmware_.engine;
    request_.surface_address = surface.mc_address;
    request_.pitch_pixels = surface.pitch_pixels;
    request_.tiling = tiling_;
    request_.dchub_refclk_khz = dchub_refclk_khz_;
    request_.max_cab_allocation_bytes = max_cab_bytes_;
    request_.dp_rate_code = mode.dp_rate_code;
    request_.dp_lanes = mode.dp_lanes;
    request_.dp_use_vsc = dp_use_vsc_;
    request_.tmds = mode.tmds;
    request_.dsc = mode.dsc;
    request_.fec = mode.fec;
    if (overlay != nullptr) {
        // dc plane order: the overlay (top) first, then the primary.
        display::Dcn32PlaneScanout& top = request_.planes[0];
        top.surface_address = overlay_mc_address;
        top.pitch_pixels = overlay->pitch_pixels;
        top.geometry = display::Dcn32PlaneGeometry{
            overlay->width, overlay->height,
            display::Dcn32Rect{overlay->x, overlay->y, static_cast<int32_t>(overlay->width),
                               static_cast<int32_t>(overlay->height)}};
        top.per_pixel_alpha = overlay->per_pixel_alpha;
        top.pre_multiplied_alpha = overlay->pre_multiplied_alpha;
        top.global_alpha = overlay->global_alpha;
        top.global_alpha_value = overlay->global_alpha_value;
        display::Dcn32PlaneScanout& primary = request_.planes[1];
        primary.surface_address = surface.mc_address;
        primary.pitch_pixels = surface.pitch_pixels;
        primary.geometry = display::Dcn32PlaneGeometry{
            mode.timing.width, mode.timing.height,
            display::Dcn32Rect{0, 0, static_cast<int32_t>(mode.timing.width), static_cast<int32_t>(mode.timing.height)}};
        request_.plane_count = 2;
    }
    return display::build_dcn32_stream_plan(request_, bandwidth_plan_, plan_) == display::StreamStatus::success;
}

VP_STATUS NativeDisplayStream::commit_mode(uint32_t index, bool preserve) noexcept {
    const DisplayStreamMode& mode = modes_[index];
    const uint32_t slot = acquire_surface(index);
    if (slot == display_stream_no_index) {
        return ERROR_NOT_ENOUGH_MEMORY;
    }
    DisplayStreamSurface& surface = surfaces_[slot];
    if ((!preserve && !clear_surface(surface)) || !bandwidth_for(mode, surface.pitch_pixels) ||
        !build_plan(index, surface)) {
        release_unused_surfaces();
        return ERROR_INVALID_PARAMETER;
    }
    const display::StreamResult result = controller_.commit(plan_);
    switch (result) {
    case display::StreamResult::committed:
        if (current_surface_ < display_stream_surface_capacity && current_surface_ != slot) {
            surfaces_[current_surface_].scanned = false;
        }
        surface.scanned = true;
        current_surface_ = slot;
        current_mode_ = index;
        mode_set_ = true;
        overlay_active_ = false;
        ++generation_;
        release_unused_surfaces();
        return NO_ERROR;
    case display::StreamResult::restored:
    case display::StreamResult::unchanged:
        // The previous mode is scanning again; the new surface is dropped.
        release_unused_surfaces();
        return ERROR_INVALID_PARAMETER;
    case display::StreamResult::blanked: {
        if (current_surface_ < display_stream_surface_capacity && current_surface_ != slot) {
            surfaces_[current_surface_].scanned = false;
        }
        current_surface_ = display_stream_no_index;
        overlay_active_ = false;
        ++generation_;
        // Last fallback: the GOP-adopted mode on its firmware surface.
        if (index != 0 && modes_[0].committable && bandwidth_for(modes_[0], surfaces_[0].pitch_pixels) &&
            build_plan(0, surfaces_[0]) && controller_.commit(plan_) == display::StreamResult::committed) {
            surfaces_[0].scanned = true;
            current_surface_ = 0;
            current_mode_ = 0;
        }
        release_unused_surfaces();
        return ERROR_INVALID_PARAMETER;
    }
    default:
        // Quarantined: nothing is released (the engine may still fetch any of them).
        current_surface_ = display_stream_no_index;
        return ERROR_DEVICE_REINITIALIZATION_NEEDED;
    }
}

// ---- IOCTLs ----

bool NativeDisplayStream::is_mode_ioctl(ULONG code) noexcept {
    switch (code) {
    case IOCTL_VIDEO_QUERY_NUM_AVAIL_MODES:
    case IOCTL_VIDEO_QUERY_AVAIL_MODES:
    case IOCTL_VIDEO_QUERY_CURRENT_MODE:
    case IOCTL_VIDEO_SET_CURRENT_MODE:
    case IOCTL_VIDEO_MAP_VIDEO_MEMORY:
    case IOCTL_VIDEO_UNMAP_VIDEO_MEMORY:
    case IOCTL_VIDEO_RESET_DEVICE:
        return true;
    default:
        return false;
    }
}

VP_STATUS NativeDisplayStream::describe(uint32_t index, VIDEO_MODE_INFORMATION& output) const noexcept {
    if (index >= mode_count_) {
        return ERROR_INVALID_PARAMETER;
    }
    const DisplayStreamMode& mode = modes_[index];
    const uint32_t pitch = mode.firmware ? surfaces_[0].pitch_pixels : align_up(mode.timing.width, pitch_alignment_pixels);
    VIDEO_MODE_INFORMATION info{};
    info.Length = static_cast<ULONG>(sizeof(info));
    info.ModeIndex = index;
    info.VisScreenWidth = mode.timing.width;
    info.VisScreenHeight = mode.timing.height;
    info.ScreenStride = pitch * display::xrgb8888_bytes_per_pixel;
    info.NumberOfPlanes = 1;
    info.BitsPerPlane = 32;
    info.Frequency = mode.refresh_millihz != 0 ? (mode.refresh_millihz + 500u) / 1000u : 1u;
    info.NumberRedBits = 8;
    info.NumberGreenBits = 8;
    info.NumberBlueBits = 8;
    info.RedMask = 0x00ff0000u;
    info.GreenMask = 0x0000ff00u;
    info.BlueMask = 0x000000ffu;
    info.AttributeFlags = VIDEO_MODE_GRAPHICS | VIDEO_MODE_COLOR | VIDEO_MODE_NO_OFF_SCREEN;
    info.VideoMemoryBitmapWidth = pitch;
    info.VideoMemoryBitmapHeight = mode.timing.height;
    output = info;
    return NO_ERROR;
}

VP_STATUS NativeDisplayStream::query_modes(VIDEO_REQUEST_PACKET& request) const noexcept {
    const ULONG bytes = mode_count_ * static_cast<ULONG>(sizeof(VIDEO_MODE_INFORMATION));
    if (!has_video_output(request, bytes)) {
        return ERROR_INSUFFICIENT_BUFFER;
    }
    for (uint32_t index = 0; index < mode_count_; ++index) {
        VIDEO_MODE_INFORMATION mode{};
        const VP_STATUS status = describe(index, mode);
        if (status != NO_ERROR) {
            return status;
        }
        auto* destination = static_cast<UCHAR*>(request.OutputBuffer) + index * sizeof(VIDEO_MODE_INFORMATION);
        VideoPortMoveMemory(destination, &mode, static_cast<ULONG>(sizeof(mode)));
    }
    request.StatusBlock->Information = bytes;
    return NO_ERROR;
}

VP_STATUS NativeDisplayStream::set_mode(VIDEO_REQUEST_PACKET& request) noexcept {
    display::ModeRequest mode{};
    const ByteView input(static_cast<const uint8_t*>(request.InputBuffer), request.InputBufferLength);
    if (display::decode_video_mode_request(input, mode) != display::Status::success || mode.mode_index >= mode_count_) {
        return ERROR_INVALID_PARAMETER;
    }
    if (VideoPortGetCurrentIrql() != PASSIVE_LEVEL) {
        return ERROR_INVALID_FUNCTION;
    }
    if (controller_.quarantined()) {
        return ERROR_DEVICE_REINITIALIZATION_NEEDED;
    }
    if (mode.mode_index == current_mode_ && current_surface_ < display_stream_surface_capacity) {
        if (!mode.preserve_contents && !clear_surface(surfaces_[current_surface_])) {
            return ERROR_NOT_ENOUGH_MEMORY;
        }
        mode_set_ = true;
        return NO_ERROR;
    }
    if (!modes_[mode.mode_index].committable) {
        return ERROR_INVALID_PARAMETER;
    }
    return commit_mode(mode.mode_index, mode.preserve_contents);
}

VP_STATUS NativeDisplayStream::map_memory(VIDEO_REQUEST_PACKET& request) noexcept {
    VIDEO_MEMORY input{};
    VIDEO_MEMORY_INFORMATION output{};
    if (!read_video_value(request, input) || input.RequestedVirtualAddress != nullptr) {
        return ERROR_INVALID_PARAMETER;
    }
    if (!has_video_output(request, sizeof(output))) {
        return ERROR_INSUFFICIENT_BUFFER;
    }
    if (!mode_set_ || current_surface_ >= display_stream_surface_capacity || VideoPortGetCurrentIrql() != PASSIVE_LEVEL) {
        return ERROR_BUSY;
    }
    DisplayStreamSurface& surface = surfaces_[current_surface_];
    const HardwareState& hardware = *inputs_.hardware;
    const uint64_t offset = surface.vram_offset;
    const uint64_t bytes = surface.bytes;
    if (surface.mapping != nullptr || (offset & (memory::page_bytes - 1)) != 0 || bytes > MAXULONG ||
        offset > hardware.vram_range_bytes || bytes > hardware.vram_range_bytes - offset || !hardware.acquired) {
        return ERROR_BUSY;
    }
    PHYSICAL_ADDRESS start = hardware.vram_range;
    start.QuadPart += static_cast<LONGLONG>(offset);
    ULONG length = static_cast<ULONG>(bytes);
    ULONG space = VIDEO_MEMORY_SPACE_MEMORY | VIDEO_MEMORY_SPACE_P6CACHE;
    PVOID virtual_address = nullptr;
    const VP_STATUS status = VideoPortMapMemory(inputs_.extension, start, &length, &space, &virtual_address);
    if (status != NO_ERROR || virtual_address == nullptr || length < bytes) {
        if (status == NO_ERROR && virtual_address != nullptr) {
            VideoPortUnmapMemory(inputs_.extension, virtual_address, nullptr);
        }
        return status != NO_ERROR ? status : ERROR_NOT_ENOUGH_MEMORY;
    }
    surface.mapping = virtual_address;
    output.VideoRamBase = virtual_address;
    output.VideoRamLength = length;
    output.FrameBufferBase = virtual_address;
    output.FrameBufferLength = static_cast<ULONG>(bytes);
    return return_video_value(request, output);
}

VP_STATUS NativeDisplayStream::unmap_memory(VIDEO_REQUEST_PACKET& request) noexcept {
    VIDEO_MEMORY input{};
    if (!read_video_value(request, input) || input.RequestedVirtualAddress == nullptr) {
        return ERROR_INVALID_PARAMETER;
    }
    for (DisplayStreamSurface& surface : surfaces_) {
        if (surface.used && surface.mapping == input.RequestedVirtualAddress) {
            const VP_STATUS status = VideoPortUnmapMemory(inputs_.extension, surface.mapping, nullptr);
            if (status == NO_ERROR) {
                surface.mapping = nullptr;
                release_unused_surfaces();
            }
            return status;
        }
    }
    return ERROR_INVALID_PARAMETER;
}

VP_STATUS NativeDisplayStream::start_io(VIDEO_REQUEST_PACKET& request) noexcept {
    if (!started_) {
        return ERROR_DEV_NOT_EXIST;
    }
    switch (request.IoControlCode) {
    case IOCTL_VIDEO_QUERY_NUM_AVAIL_MODES: {
        VIDEO_NUM_MODES modes{};
        modes.NumModes = mode_count_;
        modes.ModeInformationLength = static_cast<ULONG>(sizeof(VIDEO_MODE_INFORMATION));
        return return_video_value(request, modes);
    }
    case IOCTL_VIDEO_QUERY_AVAIL_MODES:
        return query_modes(request);
    case IOCTL_VIDEO_QUERY_CURRENT_MODE: {
        if (current_surface_ >= display_stream_surface_capacity) {
            return ERROR_DEV_NOT_EXIST;
        }
        VIDEO_MODE_INFORMATION mode{};
        const VP_STATUS status = describe(current_mode_, mode);
        return status == NO_ERROR ? return_video_value(request, mode) : status;
    }
    case IOCTL_VIDEO_SET_CURRENT_MODE:
        return set_mode(request);
    case IOCTL_VIDEO_MAP_VIDEO_MEMORY:
        return map_memory(request);
    case IOCTL_VIDEO_UNMAP_VIDEO_MEMORY:
        return unmap_memory(request);
    case IOCTL_VIDEO_RESET_DEVICE:
        // DrvAssertMode(FALSE): the committed mode keeps scanning.
        mode_set_ = false;
        return NO_ERROR;
    default:
        return ERROR_INVALID_FUNCTION;
    }
}

// ---- retargeting, power and recovery ----

bool NativeDisplayStream::scanout(display::AdoptedScanout& output) const noexcept {
    if (!started_ || current_surface_ >= display_stream_surface_capacity) {
        return false;
    }
    const DisplayStreamSurface& surface = surfaces_[current_surface_];
    const DisplayStreamMode& mode = modes_[current_mode_];
    output = inputs_.preos->scanout;
    output.timing = mode.timing;
    output.refresh_millihz = mode.refresh_millihz;
    output.stride_bytes = surface.pitch_pixels * display::xrgb8888_bytes_per_pixel;
    output.mc_address = surface.mc_address;
    output.vram_offset = surface.vram_offset;
    output.bytes = static_cast<uint64_t>(surface.pitch_pixels) * display::xrgb8888_bytes_per_pixel * mode.timing.height;
    output.clock_source = mode.signal == display::StreamSignal::display_port ? display::PreosClockSource::dp_dto
                                                                              : display::PreosClockSource::unknown;
    return true;
}

bool NativeDisplayStream::suspend() noexcept {
    if (!started_) {
        return false;
    }
    // The firmware stream is taken down only when this owner can light the
    // GOP mode again afterwards (otherwise the pre-OS pipe keeps running).
    if (!controller_.has_stream() || (controller_.firmware_current() && !modes_[0].committable)) {
        return true;
    }
    return controller_.disable() == display::StreamResult::committed;
}

void NativeDisplayStream::reset_proven() noexcept {
    if (!started_) {
        return;
    }
    controller_.reset_proven();
    hpd_.retire();
    // The AUX adapter of the reset epoch may bind once more, for a later epoch.
    aux_.reset_proven(inputs_.epoch);
    aux_io_ = {};
    lease_held_ = false;
    lease_ = {};
}

bool NativeDisplayStream::rearm_dp_aux() noexcept {
    if (!aux_.bound()) {
        // bind_aux opens the DDC pad in AUX mode; the adapter checks the new
        // epoch's proof (leased route, initialized encoder, DMCUB running
        // under this owner or held in reset) on every access.
        if (!inputs_.link->bind_aux(lease_, hpd_, aux_)) {
            return false;
        }
        aux_io_ = aux_.make_aux_io();
    } else if (inputs_.link->owner().prepare_aux(lease_) != output::LinkResult::success) {
        // An aborted suspend: same epoch, the disable closed the pad.
        return false;
    }
    // Linux retrains with the link capabilities it detected before the
    // reset; the owner first reads the receiver capability block again and
    // requires the same receiver (a different sink is not trained blindly).
    output::DpReceiverCapabilities receiver{};
    output::LinkCapabilities link{};
    const output::DpReceiverCapabilities& known = dp_port_.receiver;
    return output::probe_dp_receiver(aux_io_, output::dcn32_default_dp_source(), receiver, link) ==
               output::DpProbeStatus::success &&
           receiver.dpcd_revision == known.dpcd_revision && receiver.maximum_rate_code == known.maximum_rate_code &&
           receiver.maximum_lanes == known.maximum_lanes && receiver.enhanced_framing == known.enhanced_framing &&
           receiver.training_pattern_3 == known.training_pattern_3 &&
           receiver.training_aux_interval == known.training_aux_interval &&
           receiver.mst_capable == known.mst_capable && receiver.training_pattern_4 == known.training_pattern_4;
}

display::StreamResult NativeDisplayStream::commit_restored(uint32_t index, uint32_t slot) noexcept {
    if (!modes_[index].committable || !bandwidth_for(modes_[index], surfaces_[slot].pitch_pixels) ||
        !build_plan(index, surfaces_[slot])) {
        return display::StreamResult::unchanged;
    }
    return controller_.commit(plan_);
}

bool NativeDisplayStream::restore(uint64_t epoch) noexcept {
    if (!started_ || current_mode_ >= mode_count_) {
        return false;
    }
    inputs_.epoch = epoch;
    output::DisplayLinkOwner& owner = inputs_.link->owner();
    if (!lease_held_) {
        if (owner.acquire_route(route_.firmware_path_index, epoch, lease_) != output::LinkResult::success) {
            return false;
        }
        lease_held_ = true;
        if (!inputs_.link->bind_hpd(lease_, hpd_)) {
            return false;
        }
    }
    if ((firmware_.signal == display::StreamSignal::display_port && !rearm_dp_aux()) || !configure_bandwidth()) {
        return false;
    }
    const uint32_t slot = current_surface_ < display_stream_surface_capacity ? current_surface_ : 0;
    const uint32_t index = slot == 0 ? 0 : current_mode_;
    display::StreamResult result = commit_restored(index, slot);
    uint32_t restored_slot = slot;
    uint32_t restored_index = index;
    if (result != display::StreamResult::committed && result != display::StreamResult::uncertain && index != 0) {
        // The last fallback of a mode change: the GOP mode on its firmware surface.
        surfaces_[slot].scanned = false;
        result = commit_restored(0, 0);
        restored_slot = 0;
        restored_index = 0;
    }
    ++generation_;
    overlay_active_ = false;
    if (result != display::StreamResult::committed) {
        // Blank, or quarantined (nothing is released: the engine may still fetch).
        current_surface_ = display_stream_no_index;
        return false;
    }
    current_surface_ = restored_slot;
    current_mode_ = restored_index;
    surfaces_[restored_slot].scanned = true;
    release_unused_surfaces();
    return true;
}

bool NativeDisplayStream::check_and_recover() noexcept {
    if (!started_ || controller_.firmware_current() || !controller_.has_stream()) {
        return true;
    }
    display::StreamHealth health{};
    if (controller_.check_health(health) && health.active && health.frame_counter_moving && !health.optc_underflow &&
        !health.hubp_underflow) {
        return true;
    }
    return controller_.recover() == display::StreamResult::committed;
}

bool NativeDisplayStream::retire() noexcept {
    if (!started_) {
        return true;
    }
    if (controller_.has_stream() && !controller_.firmware_current() &&
        controller_.disable() != display::StreamResult::committed) {
        return false;
    }
    for (DisplayStreamSurface& surface : surfaces_) {
        if (surface.mapping != nullptr) {
            return false; // the display driver still holds a view
        }
    }
    for (DisplayStreamSurface& surface : surfaces_) {
        surface.scanned = false;
    }
    release_unused_surfaces();
    if (lease_held_ && inputs_.link->owner().release_route(lease_) != output::LinkResult::success) {
        return false;
    }
    lease_held_ = false;
    hpd_.retire();
    controller_.unbind();
    if (workspace_storage_ != nullptr) {
        ExFreePoolWithTag(workspace_storage_, workspace_pool_tag);
        workspace_storage_ = nullptr;
        workspace_ = nullptr;
    }
    started_ = false;
    return true;
}

// ---- overlay plane (dcn-dsc-odm) ----

VP_STATUS NativeDisplayStream::update_overlay(const DisplayOverlay* overlay) noexcept {
    if (!started_ || !allow_overlay_ || !mode_set_ || current_surface_ >= display_stream_surface_capacity ||
        current_mode_ >= mode_count_ || controller_.current() == nullptr) {
        return ERROR_INVALID_FUNCTION;
    }
    if (KeGetCurrentIrql() != PASSIVE_LEVEL) {
        return ERROR_INVALID_PARAMETER;
    }
    if (controller_.quarantined()) {
        return ERROR_DEVICE_REINITIALIZATION_NEEDED;
    }
    uint64_t overlay_mc = 0;
    if (overlay != nullptr) {
        // Only a live contiguous VRAM allocation covering the scanned bytes.
        hardware::VramAllocationInfo info{};
        const uint64_t bytes = static_cast<uint64_t>(overlay->pitch_pixels) * display::xrgb8888_bytes_per_pixel *
                               overlay->height;
        if (overlay->width == 0 || overlay->height == 0 || overlay->pitch_pixels < overlay->width ||
            inputs_.bringup->vram_allocator.describe(overlay->allocation, info) != hardware::VramStatus::success ||
            !info.contiguous || info.requested_bytes < bytes) {
            return ERROR_INVALID_PARAMETER;
        }
        overlay_mc = inputs_.preos->vram.mc_start + info.first_byte;
    }
    const DisplayStreamSurface& surface = surfaces_[current_surface_];
    if (!bandwidth_for(modes_[current_mode_], surface.pitch_pixels, overlay) ||
        !build_plan(current_mode_, surface, overlay, overlay_mc)) {
        return ERROR_INVALID_PARAMETER;
    }
    switch (controller_.update_planes(plan_)) {
    case display::StreamResult::committed:
        overlay_active_ = overlay != nullptr;
        overlay_ = overlay != nullptr ? *overlay : DisplayOverlay{};
        return NO_ERROR;
    case display::StreamResult::restored:
    case display::StreamResult::unchanged:
        // The previous planes are scanning (again).
        return ERROR_INVALID_PARAMETER;
    case display::StreamResult::blanked:
        surfaces_[current_surface_].scanned = false;
        current_surface_ = display_stream_no_index;
        overlay_active_ = false;
        ++generation_;
        // Last fallback, as for a mode change: the GOP mode on its firmware surface.
        if (modes_[0].committable && bandwidth_for(modes_[0], surfaces_[0].pitch_pixels) &&
            build_plan(0, surfaces_[0]) && controller_.commit(plan_) == display::StreamResult::committed) {
            surfaces_[0].scanned = true;
            current_surface_ = 0;
            current_mode_ = 0;
        }
        release_unused_surfaces();
        return ERROR_INVALID_PARAMETER;
    default:
        current_surface_ = display_stream_no_index;
        return ERROR_DEVICE_REINITIALIZATION_NEEDED;
    }
}

VP_STATUS NativeDisplayStream::set_overlay(const DisplayOverlay& overlay) noexcept {
    return update_overlay(&overlay);
}

VP_STATUS NativeDisplayStream::clear_overlay() noexcept {
    if (!overlay_active_) {
        return NO_ERROR;
    }
    return update_overlay(nullptr);
}

} // namespace rx7900::reactos

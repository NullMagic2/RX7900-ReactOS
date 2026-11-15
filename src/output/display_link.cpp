/*
Module: rx7900.output.display_link
Purpose: Exclusive DCN3.2 link owner: route leases, clocks, PHY, SCDC and DMCUB.
Created: 2026-10-24
Architecture: Sequencing only; register protocols live in the composed modules.
Copyright 2012-2023 Advanced Micro Devices, Inc. (sequence provenance).
Linux notice and provenance: licenses/AMDGPU-OUTPUT.txt.
*/
#include "display_link.hpp"

namespace rx7900::output {
namespace {

// DP_FEC_CONFIGURATION (dcn-dsc-odm: dp_set_fec_ready(false) at disable).
constexpr uint32_t dpcd_fec_configuration = 0x120;

LinkSignal signal_of(TmdsKind kind) noexcept {
    switch (kind) {
    case TmdsKind::hdmi: return LinkSignal::hdmi;
    case TmdsKind::dvi_dual: return LinkSignal::dvi_dual;
    default: return LinkSignal::dvi_single;
    }
}

bool tmds_route(LinkSignal signal) noexcept {
    return signal == LinkSignal::hdmi || signal == LinkSignal::dvi_single ||
           signal == LinkSignal::dvi_dual;
}

LinkResult from_clock(display::ClockResult result) noexcept {
    switch (result) {
    case display::ClockResult::success: return LinkResult::success;
    case display::ClockResult::invalid_argument:
    case display::ClockResult::unsupported_transition: return LinkResult::invalid_argument;
    case display::ClockResult::not_initialized: return LinkResult::not_ready;
    case display::ClockResult::quarantined: return LinkResult::quarantined;
    default: return LinkResult::clock_failed;
    }
}

} // namespace

DpSourceCapabilities dcn32_default_dp_source() noexcept {
    DpSourceCapabilities source{};
    source.maximum_rate_code = 0x1e;
    source.maximum_lanes = 4;
    source.verified = true;
    source.rgb8_verified = true;
    source.enhanced_framing = true;
    source.training_pattern_3 = true;
    source.training_pattern_4 = true;
    return source;
}

bool DisplayLinkOwner::quarantined() const noexcept {
    if (quarantined_ || clocks_.quarantined() || dmub_.quarantined()) {
        return true;
    }
    for (const Dcn32LinkEncoder& encoder : encoders_) {
        if (encoder.quarantined()) {
            return true;
        }
    }
    return false;
}

LinkResult DisplayLinkOwner::bind(const DisplayLinkConfig& config) noexcept {
    if (bound_ || quarantined() || config.catalog == nullptr || config.atom == nullptr ||
        !config.atom->bound() || config.epoch == 0 || !hardware::register_io_valid(config.io) ||
        config.dcn.aperture_bytes != config.io.aperture_bytes) {
        return LinkResult::invalid_argument;
    }
    if (bind_dcn32_routes(*config.catalog, config.dcn, routes_) != RouteStatus::success) {
        return LinkResult::invalid_argument;
    }
    config_ = config;

    display::ClockManagerConfig clock{};
    clock.io = config.io;
    clock.dcn = config.dcn;
    clock.dccg_segment = config.dccg_segment;
    clock.mp1_segment0 = config.mp1_segment0;
    clock.xtalin_khz = config.xtalin_khz;
    clock.boot_dprefclk_khz = config.boot_dprefclk_khz;
    clock.pll_request = config.pll_request;
    const display::ClockResult clocked = clocks_.init(clock);
    if (clocked == display::ClockResult::quarantined) {
        quarantined_ = true;
        return LinkResult::quarantined;
    }
    // A refused table leaves clocks unusable but nothing changed: routes
    // stay bindable for HPD/EDID, no stream can be clocked.
    clocks_ready_ = clocked == display::ClockResult::success;

    if (config.dmcub_firmware.size() != 0) {
        const display::DmubResult started = dmub_.start(config.io, config.dcn, config.dmcub_firmware,
                                                        config.vbios, config.dmub_memory);
        if (started == display::DmubResult::quarantined || started == display::DmubResult::timeout) {
            quarantined_ = true;
            bound_ = true;
            return LinkResult::quarantined;
        }
        dmub_started_ = started == display::DmubResult::success;
    }
    bound_ = true;
    return LinkResult::success;
}

LinkResult DisplayLinkOwner::unbind() noexcept {
    if (!bound_) {
        return LinkResult::not_bound;
    }
    for (const RouteState& state : states_) {
        if (state.leased) {
            return LinkResult::route_unavailable;
        }
    }
    if (quarantined()) {
        return LinkResult::quarantined;
    }
    if (clocks_ready_ && clocks_.release() != display::ClockResult::success) {
        quarantined_ = true;
        return LinkResult::quarantined;
    }
    if (dmub_started_ && dmub_.stop() != display::DmubResult::success) {
        quarantined_ = true;
        return LinkResult::quarantined;
    }
    reset_proven(config_.epoch);
    return LinkResult::success;
}

bool DisplayLinkOwner::lease_current(const RouteLease& lease) const noexcept {
    return bound_ && lease.epoch == config_.epoch && lease.route < routes_.count &&
           states_[lease.route].leased && states_[lease.route].generation == lease.generation;
}

Dcn32LinkEncoder& DisplayLinkOwner::encoder_of(const Dcn32ConnectorRoute& route) noexcept {
    return encoders_[route.transmitter];
}

LinkResult DisplayLinkOwner::from_encoder(EncoderResult result) noexcept {
    switch (result) {
    case EncoderResult::success: return LinkResult::success;
    case EncoderResult::invalid_argument: return LinkResult::invalid_argument;
    case EncoderResult::quarantined: quarantined_ = true; return LinkResult::quarantined;
    case EncoderResult::unsupported_revision:
    case EncoderResult::not_bound: return LinkResult::not_ready;
    default: return LinkResult::route_unavailable;
    }
}

LinkResult DisplayLinkOwner::acquire_route(uint8_t firmware_path_index, uint64_t epoch,
                                           RouteLease& lease) noexcept {
    if (!bound_) {
        return LinkResult::not_bound;
    }
    if (quarantined()) {
        return LinkResult::quarantined;
    }
    if (epoch != config_.epoch) {
        return LinkResult::stale_lease;
    }
    for (uint32_t index = 0; index < routes_.count; ++index) {
        const Dcn32ConnectorRoute& route = routes_.routes[index];
        if (route.firmware_path_index != firmware_path_index) {
            continue;
        }
        RouteState& state = states_[index];
        if (!route.validated || state.leased) {
            return LinkResult::route_unavailable;
        }
        // One lease per transmitter even if several ATOM paths named it.
        for (uint32_t other = 0; other < routes_.count; ++other) {
            if (states_[other].leased && routes_.routes[other].transmitter == route.transmitter) {
                return LinkResult::route_unavailable;
            }
        }
        Dcn32LinkEncoder& encoder = encoder_of(route);
        if (!encoder.bound()) {
            const LinkResult bound = from_encoder(encoder.bind(
                config_.dcn, route, {config_.atom, config_.io, config_.atom_policy}));
            if (bound != LinkResult::success) {
                return bound;
            }
        }
        if (!encoder_initialized_[route.transmitter]) {
            const LinkResult initialized = from_encoder(encoder.hw_init());
            if (initialized != LinkResult::success) {
                return initialized;
            }
            encoder_initialized_[route.transmitter] = true;
        }
        state.leased = true;
        ++state.generation;
        lease = {static_cast<uint8_t>(index), epoch, state.generation};
        return LinkResult::success;
    }
    return LinkResult::route_unavailable;
}

LinkResult DisplayLinkOwner::close_pad(RouteState& state) noexcept {
    if (!state.aux_open) {
        return LinkResult::success;
    }
    if (close_ddc_pad(config_.io, config_.dcn, state.pad) != DdcPadResult::success) {
        quarantined_ = true;
        return LinkResult::quarantined;
    }
    state.aux_open = false;
    return LinkResult::success;
}

LinkResult DisplayLinkOwner::release_route(RouteLease& lease) noexcept {
    if (!lease_current(lease)) {
        return LinkResult::stale_lease;
    }
    RouteState& state = states_[lease.route];
    if (state.link_enabled) {
        const LinkResult disabled = disable_link(lease);
        if (disabled != LinkResult::success) {
            return disabled;
        }
    }
    const LinkResult closed = close_pad(state);
    if (closed != LinkResult::success) {
        return closed;
    }
    state.leased = false;
    lease = {};
    return LinkResult::success;
}

const display::DisplayClockTable* DisplayLinkOwner::clock_table() const noexcept {
    return bound_ && clocks_ready_ && clocks_.table().valid ? &clocks_.table() : nullptr;
}

LinkResult DisplayLinkOwner::set_display_clocks(const display::DisplayClocks& clocks,
                                                bool safe_to_lower,
                                                display::DisplayClocks& applied) noexcept {
    if (!bound_) {
        return LinkResult::not_bound;
    }
    if (!clocks_ready_) {
        return LinkResult::not_ready;
    }
    const LinkResult result = from_clock(clocks_.update(clocks, safe_to_lower, applied));
    quarantined_ = quarantined_ || result == LinkResult::quarantined;
    return result;
}

LinkResult DisplayLinkOwner::set_pixel_clock(const RouteLease& lease, uint32_t otg,
                                             uint32_t pixel_100hz, TmdsKind tmds, TmdsDepth depth,
                                             uint32_t k1, uint32_t k2) noexcept {
    if (!lease_current(lease)) {
        return LinkResult::stale_lease;
    }
    if (!clocks_ready_) {
        return LinkResult::not_ready;
    }
    if (quarantined()) {
        return LinkResult::quarantined;
    }
    const Dcn32ConnectorRoute& route = routes_.routes[lease.route];
    RouteState& state = states_[lease.route];
    LinkResult result = LinkResult::success;
    if (route.signal == LinkSignal::display_port) {
        result = from_clock(clocks_.program_dp_dto(otg, pixel_100hz));
    } else {
        if (!state.pll_held) {
            uint8_t pll = dcn32_route_none;
            for (uint8_t candidate = 0; candidate < dcn32_link_count; ++candidate) {
                if (!pll_used_[candidate]) {
                    pll = candidate;
                    break;
                }
            }
            if (pll == dcn32_route_none) {
                return LinkResult::route_unavailable;
            }
            state.pll = pll;
        }
        result = from_clock(clocks_.select_tmds_source(otg));
        if (result == LinkResult::success) {
            pll_used_[state.pll] = true;
            state.pll_held = true;
            result = from_encoder(encoder_of(route).program_tmds_pll(
                otg, state.pll, signal_of(tmds), pixel_100hz, depth));
        }
        if (result == LinkResult::success) {
            result = from_clock(clocks_.program_pixel_resync(state.pll,
                                                             static_cast<uint32_t>(depth)));
        }
    }
    if (result == LinkResult::success) {
        result = from_clock(clocks_.set_pixel_rate_div(otg, k1, k2));
    }
    quarantined_ = quarantined_ || result == LinkResult::quarantined;
    return result;
}

LinkResult DisplayLinkOwner::clear_pixel_clock(const RouteLease& lease, uint32_t otg) noexcept {
    if (!lease_current(lease)) {
        return LinkResult::stale_lease;
    }
    if (!clocks_ready_) {
        return LinkResult::not_ready;
    }
    const Dcn32ConnectorRoute& route = routes_.routes[lease.route];
    RouteState& state = states_[lease.route];
    LinkResult result = from_clock(clocks_.clear_pixel_clock(otg));
    if (result == LinkResult::success && state.pll_held) {
        result = from_encoder(encoder_of(route).power_down_pll(state.pll));
        if (result == LinkResult::success) {
            pll_used_[state.pll] = false;
            state.pll_held = false;
        }
    }
    quarantined_ = quarantined_ || result == LinkResult::quarantined;
    return result;
}

LinkResult DisplayLinkOwner::prepare_aux(const RouteLease& lease) noexcept {
    if (!lease_current(lease)) {
        return LinkResult::stale_lease;
    }
    if (quarantined()) {
        return LinkResult::quarantined;
    }
    const Dcn32ConnectorRoute& route = routes_.routes[lease.route];
    RouteState& state = states_[lease.route];
    if (route.signal != LinkSignal::display_port) {
        return LinkResult::invalid_argument;
    }
    if (state.aux_open) {
        return LinkResult::success;
    }
    const DdcPadResult opened = open_ddc_pad(config_.io, config_.dcn, route.ddc_line,
                                             DdcMode::aux, state.pad);
    if (opened == DdcPadResult::success) {
        state.aux_open = true;
        return LinkResult::success;
    }
    if (opened == DdcPadResult::state_unknown) {
        state.aux_open = state.pad.valid;
        quarantined_ = true;
        return LinkResult::quarantined;
    }
    return LinkResult::not_ready;
}

LinkResult DisplayLinkOwner::enable_dp_link(const RouteLease& lease, const AuxIo& aux,
                                            const DpPortSnapshot& port,
                                            const DpTrainingRequest& request, uint8_t dig_fe,
                                            DpTrainedLink& trained) noexcept {
    trained = {};
    if (!lease_current(lease)) {
        return LinkResult::stale_lease;
    }
    if (quarantined()) {
        return LinkResult::quarantined;
    }
    const Dcn32ConnectorRoute& route = routes_.routes[lease.route];
    RouteState& state = states_[lease.route];
    if (route.signal != LinkSignal::display_port || dig_fe >= dcn32_link_count ||
        state.link_enabled || aux.transfer == nullptr) {
        return LinkResult::invalid_argument;
    }
    if (!state.aux_open || !config_.dp_source.verified) {
        return LinkResult::not_ready;
    }
    Dcn32LinkEncoder& encoder = encoder_of(route);
    LinkResult result = from_encoder(encoder.setup(LinkSignal::display_port));
    if (result == LinkResult::success) {
        result = from_encoder(encoder.connect_fe(dig_fe, true));
    }
    if (result != LinkResult::success) {
        return result;
    }
    state.dig_fe = dig_fe;
    state.active = LinkSignal::display_port;
    const DpTrainingStatus status = train_dp_sst(aux, encoder.make_phy_io(), config_.dp_source,
                                                 port, request);
    if (status == DpTrainingStatus::trained) {
        state.link_enabled = true;
        trained = {request.rate_code, request.lanes, config_.dp_source.enhanced_framing &&
                                                         port.receiver.enhanced_framing};
        // dcn-dsc-odm: enable_link_dp -> dp_set_fec_enable(true) once 1000 LL
        // codes followed training (7 us at 1-lane RBR).
        if (request.fec_ready) {
            state.fec_ready = true;
            state.fec_aux = aux;
            config_.io.delay_microseconds(config_.io.context, 7);
            result = from_encoder(encoder.fec_set_enable(true));
            if (result != LinkResult::success) {
                return result;
            }
            state.fec_enabled = true;
        }
        return LinkResult::success;
    }
    // dcn-dsc-odm: a failed FEC training leaves FEC_READY set on both ends
    // (dp_set_fec_ready(false) of disable_link_dp, without the PHY off it
    // already had).
    if (request.fec_ready) {
        const uint8_t fec_off = 0;
        dp_aux_write(aux, dpcd_fec_configuration, &fec_off, 1);
        encoder.fec_set_ready(false);
    }
    // train_dp_sst already called finish(false) (PHY disable); undo the FE.
    const LinkResult disconnected = from_encoder(encoder.connect_fe(dig_fe, false));
    state.dig_fe = dcn32_route_none;
    state.active = LinkSignal::none;
    if (status == DpTrainingStatus::state_unknown || encoder.quarantined() ||
        disconnected != LinkResult::success) {
        quarantined_ = true;
        return LinkResult::quarantined;
    }
    return status == DpTrainingStatus::disconnected || status == DpTrainingStatus::auxiliary_failure ?
           LinkResult::sink_failed : LinkResult::training_failed;
}

I2cEngineConfig DisplayLinkOwner::i2c_config() const noexcept {
    I2cEngineConfig config{};
    config.io = config_.io;
    config.space = config_.dcn;
    config.dccg_segment = config_.dccg_segment;
    config.xtalin_khz = config_.xtalin_khz;
    config.speed_khz = 100;
    return config;
}

LinkResult DisplayLinkOwner::enable_tmds_link(const RouteLease& lease, uint32_t tmds_khz,
                                              uint8_t dig_fe, TmdsKind kind,
                                              bool lte_340_scramble, ScdcReport& scdc) noexcept {
    scdc = {};
    if (!lease_current(lease)) {
        return LinkResult::stale_lease;
    }
    if (quarantined()) {
        return LinkResult::quarantined;
    }
    const Dcn32ConnectorRoute& route = routes_.routes[lease.route];
    RouteState& state = states_[lease.route];
    const LinkSignal signal = signal_of(kind);
    if (!tmds_route(route.signal) || dig_fe >= dcn32_link_count || state.link_enabled ||
        tmds_khz == 0 || (signal != LinkSignal::hdmi && tmds_khz > 165000u * 2u) ||
        (signal == LinkSignal::dvi_single && tmds_khz > 165000u)) {
        return LinkResult::invalid_argument;
    }
    if (!state.pll_held) {
        return LinkResult::not_ready;
    }
    Dcn32LinkEncoder& encoder = encoder_of(route);
    LinkResult result = from_encoder(encoder.setup(signal));
    if (result == LinkResult::success) {
        result = from_encoder(encoder.connect_fe(dig_fe, true));
    }
    if (result != LinkResult::success) {
        return result;
    }
    state.dig_fe = dig_fe;
    state.active = signal;
    if (signal == LinkSignal::hdmi) {
        const I2cResult written = hdmi_write_scdc(i2c_config(), route.ddc_line, tmds_khz,
                                                  lte_340_scramble, scdc);
        if (written == I2cResult::state_unknown) {
            quarantined_ = true;
            return LinkResult::quarantined;
        }
        // Linux does not stop on an SCDC failure; a sink that needs scrambling
        // and refused TMDS_CONFIG is reported instead of hidden.
        if (written != I2cResult::success && tmds_khz > hdmi_scrambling_threshold_khz) {
            const LinkResult undone = from_encoder(encoder.connect_fe(dig_fe, false));
            state.dig_fe = dcn32_route_none;
            state.active = LinkSignal::none;
            return undone == LinkResult::success ? LinkResult::sink_failed : undone;
        }
    }
    result = from_encoder(encoder.enable_tmds(signal, tmds_khz));
    if (result != LinkResult::success) {
        return result;
    }
    state.link_enabled = true;
    if (signal == LinkSignal::hdmi &&
        hdmi_read_scdc(i2c_config(), route.ddc_line, scdc) == I2cResult::state_unknown) {
        quarantined_ = true;
        return LinkResult::quarantined;
    }
    return LinkResult::success;
}

LinkResult DisplayLinkOwner::disable_link(const RouteLease& lease) noexcept {
    if (!lease_current(lease)) {
        return LinkResult::stale_lease;
    }
    const Dcn32ConnectorRoute& route = routes_.routes[lease.route];
    RouteState& state = states_[lease.route];
    Dcn32LinkEncoder& encoder = encoder_of(route);
    if (encoder.quarantined()) {
        return LinkResult::quarantined;
    }
    LinkResult result = LinkResult::success;
    if (state.link_enabled) {
        result = from_encoder(encoder.disable(state.active));
        if (result != LinkResult::success) {
            return result;
        }
        state.link_enabled = false;
    }
    // dcn-dsc-odm: disable_link_dp after dp_disable_link_phy:
    // dp_set_fec_enable(false), then dp_set_fec_ready(false) (DPCD, encoder).
    if (state.fec_enabled) {
        result = from_encoder(encoder.fec_set_enable(false));
        if (result != LinkResult::success) {
            return result;
        }
        state.fec_enabled = false;
    }
    if (state.fec_ready) {
        const uint8_t fec_off = 0;
        if (state.fec_aux.transfer != nullptr) {
            dp_aux_write(state.fec_aux, dpcd_fec_configuration, &fec_off, 1);
        }
        result = from_encoder(encoder.fec_set_ready(false));
        if (result != LinkResult::success) {
            return result;
        }
        state.fec_ready = false;
        state.fec_aux = {};
    }
    if (state.dig_fe != dcn32_route_none) {
        result = from_encoder(encoder.connect_fe(state.dig_fe, false));
        if (result != LinkResult::success) {
            return result;
        }
        state.dig_fe = dcn32_route_none;
    }
    state.active = LinkSignal::none;
    return close_pad(state);
}

LinkResult DisplayLinkOwner::adopt_firmware_link(const RouteLease& lease, TmdsKind tmds, uint8_t dig_fe,
                                                 bool& adopted) noexcept {
    adopted = false;
    if (!lease_current(lease)) {
        return LinkResult::stale_lease;
    }
    if (quarantined()) {
        return LinkResult::quarantined;
    }
    const Dcn32ConnectorRoute& route = routes_.routes[lease.route];
    RouteState& state = states_[lease.route];
    if (dig_fe >= dcn32_link_count || state.link_enabled || state.dig_fe != dcn32_route_none) {
        return LinkResult::invalid_argument;
    }
    bool enabled = false;
    if (!encoder_of(route).dig_enabled(enabled)) {
        return LinkResult::not_ready;
    }
    if (!enabled) {
        return LinkResult::success;
    }
    state.link_enabled = true;
    state.active = route.signal == LinkSignal::display_port ? LinkSignal::display_port : signal_of(tmds);
    state.dig_fe = dig_fe;
    adopted = true;
    return LinkResult::success;
}

LinkResult DisplayLinkOwner::dmub_execute(uint8_t (&command)[display::dmub_command_bytes],
                                          bool reply, uint32_t timeout_us) noexcept {
    if (!bound_) {
        return LinkResult::not_bound;
    }
    if (!dmub_.ready()) {
        return dmub_.quarantined() ? LinkResult::quarantined : LinkResult::not_ready;
    }
    switch (dmub_.execute(command, reply, timeout_us)) {
    case display::DmubResult::success: return LinkResult::success;
    case display::DmubResult::invalid_argument: return LinkResult::invalid_argument;
    case display::DmubResult::queue_full: return LinkResult::not_ready;
    default: quarantined_ = true; return LinkResult::quarantined;
    }
}

LinkResult DisplayLinkOwner::p_state_delegate(bool should_manage_pstate) noexcept {
    if (!bound_) {
        return LinkResult::not_bound;
    }
    if (should_manage_pstate) {
        return LinkResult::invalid_argument;
    }
    if (!dmub_started_) {
        return LinkResult::success;
    }
    uint8_t command[display::dmub_command_bytes]{};
    display::dmub_build_fams_setup(false, command);
    return dmub_execute(command, false, display::dmub_command_wait_us);
}

LinkResult DisplayLinkOwner::hdmi_scdc_legacy(const RouteLease& lease) noexcept {
    if (!lease_current(lease)) {
        return LinkResult::stale_lease;
    }
    if (quarantined()) {
        return LinkResult::quarantined;
    }
    const Dcn32ConnectorRoute& route = routes_.routes[lease.route];
    if (!tmds_route(route.signal)) {
        return LinkResult::invalid_argument;
    }
    ScdcReport report{};
    const I2cResult written = hdmi_scdc_legacy_mode(i2c_config(), route.ddc_line, report);
    if (written == I2cResult::state_unknown) {
        quarantined_ = true;
        return LinkResult::quarantined;
    }
    return written == I2cResult::success ? LinkResult::success : LinkResult::sink_failed;
}

bool DisplayLinkOwner::hpd_proof(uint64_t epoch, uint8_t pin, uint32_t segment2,
                                 uint32_t aperture) noexcept {
    if (!bound_ || quarantined() || epoch != config_.epoch ||
        segment2 != config_.dcn.segment2_dword_base || aperture != config_.dcn.aperture_bytes) {
        return false;
    }
    for (uint32_t index = 0; index < routes_.count; ++index) {
        const Dcn32ConnectorRoute& route = routes_.routes[index];
        if (states_[index].leased && route.validated && route.hpd_source == pin) {
            return true;
        }
    }
    return false;
}

bool DisplayLinkOwner::aux_proof(uint64_t epoch, uint8_t engine, uint8_t pin, uint32_t segment2,
                                 uint32_t aperture) noexcept {
    if (!hpd_proof(epoch, pin, segment2, aperture)) {
        return false;
    }
    for (uint32_t index = 0; index < routes_.count; ++index) {
        const Dcn32ConnectorRoute& route = routes_.routes[index];
        const RouteState& state = states_[index];
        if (!state.leased || route.hpd_source != pin || route.aux_engine != engine ||
            route.signal != LinkSignal::display_port || !state.aux_open ||
            !encoder_initialized_[route.transmitter]) {
            continue;
        }
        bool held = false;
        const bool dmub_owned = dmub_started_ ? dmub_.ready() :
                                (display::dmub_held_in_reset(config_.io, config_.dcn, held) && held);
        return dmub_owned && ddc_pad_in_aux_mode(config_.io, config_.dcn, route.ddc_line);
    }
    return false;
}

void DisplayLinkOwner::reset_proven(uint64_t old_epoch) noexcept {
    if (old_epoch != config_.epoch) {
        return;
    }
    for (uint32_t index = 0; index < dcn32_link_count; ++index) {
        encoders_[index].reset_proven();
        encoder_initialized_[index] = false;
        pll_used_[index] = false;
    }
    for (RouteState& state : states_) {
        state.leased = false;
        state.aux_open = false;
        state.link_enabled = false;
        state.pll_held = false;
        state.dig_fe = dcn32_route_none;
        state.active = LinkSignal::none;
        state.pad = {};
        // generation keeps counting so an old lease can never match again.
    }
    clocks_.reset_proven();
    dmub_.reset_proven();
    routes_.clear();
    config_ = {};
    clocks_ready_ = false;
    dmub_started_ = false;
    bound_ = false;
    quarantined_ = false;
}

} // namespace rx7900::output

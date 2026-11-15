/*
Module: rx7900.output.dp_training
Purpose: Fail-closed DP 8b/10b SST training with bounded hardware callbacks.
Created: 2026-10-22
Architecture: Reuses dp_link's AUX retries and output budget; the DCN backend
owns actual PHY clocks, AUX registers, link quiescence and recovery proof.
*/
#include "dp_training.hpp"

namespace rx7900::output {
namespace {

constexpr uint32_t dpcd_link_bw_set = 0x100;
constexpr uint32_t dpcd_training_pattern_set = 0x102;
constexpr uint32_t dpcd_training_lane_set = 0x103;
constexpr uint32_t dpcd_mst_control = 0x111;
constexpr uint32_t dpcd_fec_configuration = 0x120;
constexpr uint8_t fec_ready_bit = 0x01;
constexpr uint32_t dpcd_lane_status = 0x202;
constexpr uint8_t enhanced_framing_bit = 0x80;
constexpr uint8_t mst_enable_bit = 0x01;
constexpr uint8_t scramble_disable_bit = 0x20;
constexpr uint8_t clock_recovery_bit = 0x01;
constexpr uint8_t equalization_bits = 0x07;
constexpr uint8_t interlane_align_bit = 0x01;
constexpr uint8_t voltage_max_bit = 0x04;
constexpr uint8_t preemphasis_max_bit = 0x20;
constexpr uint8_t clock_pattern = 1;
constexpr uint8_t equalization_pattern = 2;
constexpr uint8_t equalization_pattern_3 = 3;
constexpr uint8_t equalization_pattern_4 = 7;
constexpr uint32_t maximum_training_reads = 5;

bool lanes_valid(uint8_t lanes) noexcept {
    return lanes == 1 || lanes == 2 || lanes == 4;
}

uint8_t lane_status(const uint8_t (&status)[6], uint8_t lane) noexcept {
    return static_cast<uint8_t>((status[lane / 2] >> ((lane & 1u) * 4u)) & 0x0fu);
}

bool clock_recovered(const uint8_t (&status)[6], uint8_t lanes) noexcept {
    for (uint8_t lane = 0; lane < lanes; ++lane) {
        if ((lane_status(status, lane) & clock_recovery_bit) == 0) {
            return false;
        }
    }

    return true;
}

bool equalized(const uint8_t (&status)[6], uint8_t lanes) noexcept {
    if ((status[2] & interlane_align_bit) == 0) {
        return false;
    }

    for (uint8_t lane = 0; lane < lanes; ++lane) {
        if ((lane_status(status, lane) & equalization_bits) != equalization_bits) {
            return false;
        }
    }

    return true;
}

void adjust_lanes(const uint8_t (&status)[6], uint8_t lanes,
                  uint8_t (&settings)[4]) noexcept {
    for (uint8_t lane = 0; lane < lanes; ++lane) {
        const uint8_t nibble = static_cast<uint8_t>(
            (status[4 + lane / 2] >> ((lane & 1u) * 4u)) & 0x0fu);
        const uint8_t swing = nibble & 0x03u;
        uint8_t emphasis = static_cast<uint8_t>((nibble >> 2u) & 0x03u);

        if (emphasis > 3u - swing) {
            emphasis = static_cast<uint8_t>(3u - swing);
        }

        settings[lane] = static_cast<uint8_t>(
            swing | (emphasis << 3u) |
            (swing == 3 ? voltage_max_bit : 0u) |
            (emphasis == 3 ? preemphasis_max_bit : 0u));
    }
}

uint32_t training_delay(const DpReceiverCapabilities& receiver,
                        bool clock_phase) noexcept {
    if (receiver.training_aux_interval != 0) {
        return static_cast<uint32_t>(receiver.training_aux_interval) * 4000u;
    }

    return clock_phase ? 100u : 400u;
}

DpTrainingStatus program_pattern(const AuxIo& aux, const DpPhyIo& phy,
                                 uint8_t pattern, const uint8_t (&settings)[4],
                                 uint8_t lanes, bool first_attempt) noexcept {
    // TPS4 carries its own scrambled sequence; TPS1/2/3 disable scrambling.
    const uint8_t sink_pattern = static_cast<uint8_t>(pattern |
        (pattern == equalization_pattern_4 ? 0u : scramble_disable_bit));

    if (first_attempt) {
        if (!phy.pattern(phy.context, pattern)) {
            return DpTrainingStatus::source_failure;
        }

        if (dp_aux_write(aux, dpcd_training_pattern_set, &sink_pattern, 1) !=
            DpProbeStatus::success) {
            return DpTrainingStatus::auxiliary_failure;
        }
    }

    if (!phy.drive(phy.context, settings, lanes)) {
        return DpTrainingStatus::source_failure;
    }

    if (dp_aux_write(aux, dpcd_training_lane_set, settings, lanes) !=
        DpProbeStatus::success) {
        return DpTrainingStatus::auxiliary_failure;
    }

    return DpTrainingStatus::trained;
}

DpTrainingStatus finish_training(const AuxIo& aux, const DpPhyIo& phy,
                                 DpTrainingStatus status) noexcept {
    const uint8_t disable = 0;
    const bool sink_disabled = dp_aux_write(aux, dpcd_training_pattern_set,
                                            &disable, 1) == DpProbeStatus::success;
    const bool source_finished = phy.finish(phy.context,
                                            status == DpTrainingStatus::trained &&
                                            sink_disabled);

    if (!sink_disabled || !source_finished) {
        return DpTrainingStatus::state_unknown;
    }

    return status;
}

// dcn-dsc-odm: admission of a DSC stream. The mode must exist and the sink be
// digital; the compressed rate (pixel clock x bpp / 16, scaled by the 97 percent
// 8b/10b FEC efficiency of link_validation.c when FEC is used) must fit the
// selected budget.
Status prepare_compressed_mode(const SinkSnapshot& sink, uint32_t sink_mode_index,
                               const LinkCapabilities& link, uint32_t bits_per_pixel_x16,
                               bool fec) noexcept {
    display::Timing timing{};

    if (sink.detailed_modes.get(sink_mode_index, timing) != display::Status::success) {
        return Status::mode_not_found;
    }

    if (link.transport != Transport::display_port_sst || link.usable_payload_bits_per_second == 0 ||
        !link.rgb_8bpc_verified || bits_per_pixel_x16 < 6u * 16u || bits_per_pixel_x16 > 24u * 16u ||
        !sink.digital_input) {
        return Status::unsupported_output;
    }

    // u32 kHz x 1000 x 384 fits u64.
    uint64_t required = static_cast<uint64_t>(timing.pixel_clock_khz) * 1000u * bits_per_pixel_x16 / 16u;
    uint64_t usable = link.usable_payload_bits_per_second;

    if (fec) {
        usable = usable / 100u * 97u;
    }

    return required <= usable ? Status::success : Status::unsupported_output;
}

DpTrainingStatus train_phase(const AuxIo& aux, const DpPhyIo& phy,
                             const DpReceiverCapabilities& receiver,
                             uint8_t lanes, uint8_t pattern,
                             uint8_t (&settings)[4], bool clock_phase) noexcept {
    for (uint32_t attempt = 0; attempt < maximum_training_reads; ++attempt) {
        if (!aux.connected(aux.context)) {
            return DpTrainingStatus::disconnected;
        }

        const DpTrainingStatus programmed = program_pattern(aux, phy, pattern,
                                                              settings, lanes,
                                                              attempt == 0);

        if (programmed != DpTrainingStatus::trained) {
            return programmed;
        }

        if (!aux.wait_microseconds(aux.context, training_delay(receiver,
                                                               clock_phase))) {
            return DpTrainingStatus::source_failure;
        }

        uint8_t status[6]{};

        if (dp_aux_read(aux, dpcd_lane_status, status, sizeof(status)) !=
            DpProbeStatus::success) {
            return DpTrainingStatus::auxiliary_failure;
        }

        if (clock_phase && clock_recovered(status, lanes)) {
            return DpTrainingStatus::trained;
        }

        if (!clock_phase) {
            if (!clock_recovered(status, lanes)) {
                return DpTrainingStatus::clock_recovery_failed;
            }

            if (equalized(status, lanes)) {
                return DpTrainingStatus::trained;
            }
        }

        adjust_lanes(status, lanes, settings);
    }

    return clock_phase ? DpTrainingStatus::clock_recovery_failed :
                         DpTrainingStatus::equalization_failed;
}

} // namespace

DpTrainingStatus train_dp_sst(const AuxIo& aux, const DpPhyIo& phy,
                              const DpSourceCapabilities& source,
                              const DpPortSnapshot& port,
                              const DpTrainingRequest& request) noexcept {
    if (aux.connected == nullptr || aux.transfer == nullptr ||
        aux.wait_microseconds == nullptr || phy.configure == nullptr ||
        phy.pattern == nullptr || phy.drive == nullptr || phy.finish == nullptr) {
        return DpTrainingStatus::invalid_argument;
    }

    LinkCapabilities checked_caps{};

    if (!source.verified || !source.rgb8_verified ||
        dp_link_budget(source.maximum_rate_code, source.maximum_lanes, false,
                       checked_caps) != DpProbeStatus::success ||
        dp_link_budget(port.receiver.maximum_rate_code,
                       port.receiver.maximum_lanes, false,
                       checked_caps) != DpProbeStatus::success ||
        !lanes_valid(request.lanes) ||
        request.lanes > source.maximum_lanes ||
        request.lanes > port.receiver.maximum_lanes ||
        request.rate_code > source.maximum_rate_code ||
        request.rate_code > port.receiver.maximum_rate_code ||
        (request.rate_code != 0x06 && request.rate_code != 0x0a &&
         request.rate_code != 0x14 && request.rate_code != 0x1e) ||
        (request.rate_code == 0x1e &&
         (!source.training_pattern_4 || !port.receiver.training_pattern_4)) ||
        port.receiver.training_aux_interval > 4u ||
        port.link.transport != Transport::display_port_sst ||
        !port.link.rgb_8bpc_verified) {
        return DpTrainingStatus::unsupported_link;
    }

    LinkCapabilities selected{};

    if (dp_link_budget(request.rate_code, request.lanes, true, selected) !=
        DpProbeStatus::success) {
        return DpTrainingStatus::unsupported_link;
    }

    display::Timing admitted{};

    if (request.dsc_bits_per_pixel_x16 != 0 ?
            prepare_compressed_mode(port.sink, request.sink_mode_index, selected,
                                    request.dsc_bits_per_pixel_x16, request.fec_ready) != Status::success :
            prepare_output_mode(port.sink, request.sink_mode_index,
                                selected, admitted) != Status::success) {
        return DpTrainingStatus::unsupported_mode;
    }

    if (request.fec_ready && phy.fec_ready == nullptr) {
        return DpTrainingStatus::unsupported_link;
    }

    if (!aux.connected(aux.context)) {
        return DpTrainingStatus::disconnected;
    }

    if (port.receiver.mst_capable) {
        uint8_t mst_control = 0;

        if (dp_aux_read(aux, dpcd_mst_control, &mst_control, 1) !=
            DpProbeStatus::success) {
            return DpTrainingStatus::auxiliary_failure;
        }

        if ((mst_control & mst_enable_bit) != 0) {
            return DpTrainingStatus::unsupported_link;
        }
    }

    const bool enhanced = source.enhanced_framing &&
                          port.receiver.enhanced_framing;
    if (!phy.configure(phy.context, request.rate_code, request.lanes, enhanced)) {
        return finish_training(aux, phy, DpTrainingStatus::source_failure);
    }

    // dp_perform_link_training -> dp_set_fec_ready(true): DPCD first, then the
    // encoder; Linux continues after a failed DPCD write, this owner stops.
    if (request.fec_ready) {
        if (dp_aux_write(aux, dpcd_fec_configuration, &fec_ready_bit, 1) != DpProbeStatus::success) {
            return finish_training(aux, phy, DpTrainingStatus::auxiliary_failure);
        }

        if (!phy.fec_ready(phy.context, true)) {
            return finish_training(aux, phy, DpTrainingStatus::source_failure);
        }
    }

    const uint8_t sink_settings[2]{request.rate_code,
                                   static_cast<uint8_t>(request.lanes |
                                       (enhanced ? enhanced_framing_bit : 0u))};

    if (dp_aux_write(aux, dpcd_link_bw_set, sink_settings, 2) !=
        DpProbeStatus::success) {
        return finish_training(aux, phy, DpTrainingStatus::auxiliary_failure);
    }

    uint8_t lane_settings[4]{};
    DpTrainingStatus status = train_phase(aux, phy, port.receiver, request.lanes,
                                          clock_pattern, lane_settings, true);

    if (status == DpTrainingStatus::trained) {
        const uint8_t pattern = request.rate_code == 0x1e ?
                                equalization_pattern_4 :
                                (port.receiver.training_pattern_3 &&
                                 source.training_pattern_3 ?
                                 equalization_pattern_3 : equalization_pattern);
        status = train_phase(aux, phy, port.receiver, request.lanes, pattern,
                             lane_settings, false);
    }

    if (status == DpTrainingStatus::trained && !aux.connected(aux.context)) {
        status = DpTrainingStatus::disconnected;
    }

    return finish_training(aux, phy, status);
}

} // namespace rx7900::output

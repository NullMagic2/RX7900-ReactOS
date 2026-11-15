/*
Module: rx7900.output.dp_link
Purpose: Safe DP 8b/10b SST capability and EDID transactions.
Created: 2026-10-22
Architecture: Reuses sink::parse_edid and prepare_output_mode; hardware AUX,
HPD, PHY, clock and scanout ownership remain with the native display backend.
Protocol values and defer policy follow Linux v6.12 drm_dp.h/drm_dp_helper.c.
*/
#include "dp_link.hpp"

namespace rx7900::output {
namespace {

constexpr uint32_t dpcd_receiver_base = 0x000;
constexpr uint32_t dpcd_mst_capability = 0x021;
constexpr uint8_t dpcd_minimum_revision = 0x10;
constexpr uint8_t dpcd_maximum_legacy_revision = 0x14;
constexpr uint8_t dpcd_lane_mask = 0x1f;
constexpr uint8_t dpcd_enhanced_frame = 0x80;
constexpr uint8_t dpcd_training_pattern_3 = 0x40;
constexpr uint8_t dpcd_training_pattern_4 = 0x80;
constexpr uint8_t dpcd_downstream_present = 0x01;
constexpr uint8_t dpcd_8b10b_coding = 0x01;
constexpr uint8_t dpcd_mst_enabled = 0x01;
constexpr uint8_t ddc_edid_address = 0x50;
constexpr uint8_t ddc_segment_address = 0x30;
constexpr uint32_t aux_defer_retries = 7;
constexpr uint32_t aux_retry_delay_microseconds = 400;
constexpr uint64_t link_rate_code_bits_per_second = 270000000ULL;
constexpr uint64_t rgb8_bits_per_pixel = 24;
// Reserve 1% beyond the 8b/10b encoding loss for secondary framing overhead.
constexpr uint64_t usable_payload_numerator = 99;
constexpr uint64_t usable_payload_denominator = 100;

bool valid_rate(uint8_t code) noexcept {
    return code == 0x06 || code == 0x0a || code == 0x14 || code == 0x1e;
}

bool valid_lanes(uint8_t lanes) noexcept {
    return lanes == 1 || lanes == 2 || lanes == 4;
}

DpProbeStatus exchange(const AuxIo& io, const AuxRequest& request,
                       uint8_t& transferred) noexcept {
    if (io.transfer == nullptr || io.wait_microseconds == nullptr ||
        request.bytes > dp_aux_payload_bytes ||
        (request.bytes != 0 &&
         (request.operation == AuxOperation::i2c_write ||
          request.operation == AuxOperation::native_write) &&
         request.write_data == nullptr) ||
        (request.bytes != 0 &&
         (request.operation == AuxOperation::native_read ||
          request.operation == AuxOperation::i2c_read) &&
         request.read_data == nullptr)) {
        return DpProbeStatus::invalid_argument;
    }

    for (uint32_t retry = 0; retry <= aux_defer_retries; ++retry) {
        AuxReply reply{};

        if (!io.transfer(io.context, request, reply)) {
            return DpProbeStatus::auxiliary_failure;
        }

        if (reply.bytes > request.bytes) {
            return DpProbeStatus::short_transfer;
        }

        if (reply.result == AuxResult::ack) {
            transferred = reply.bytes;
            return DpProbeStatus::success;
        }

        if (reply.result == AuxResult::nack) {
            return DpProbeStatus::auxiliary_nack;
        }

        if (reply.result != AuxResult::defer) {
            return DpProbeStatus::auxiliary_failure;
        }

        if (retry == aux_defer_retries) {
            break;
        }

        if (!io.wait_microseconds(io.context, aux_retry_delay_microseconds)) {
            return DpProbeStatus::auxiliary_failure;
        }
    }

    return DpProbeStatus::auxiliary_defer_exhausted;
}

class I2cStop final {
public:
    explicit I2cStop(const AuxIo& io) noexcept : io_(io) {}
    I2cStop(const I2cStop&) = delete;
    I2cStop& operator=(const I2cStop&) = delete;

    ~I2cStop() {
        if (active_ && io_.transfer != nullptr) {
            const AuxRequest stop{AuxOperation::i2c_read, ddc_edid_address,
                                  nullptr, nullptr, 0, false};
            AuxReply ignored{};
            io_.transfer(io_.context, stop, ignored);
        }
    }

    void completed() noexcept { active_ = false; }

private:
    const AuxIo& io_;
    bool active_ = true;
};

DpProbeStatus i2c_write_byte(const AuxIo& io, uint8_t slave,
                             uint8_t value) noexcept {
    const AuxRequest request{AuxOperation::i2c_write, slave, &value, nullptr, 1, true};
    uint8_t written = 0;
    const DpProbeStatus status = exchange(io, request, written);

    if (status != DpProbeStatus::success) {
        return status;
    }

    return written == 1 ? DpProbeStatus::success : DpProbeStatus::short_transfer;
}

DpProbeStatus read_edid_block(const AuxIo& io, uint32_t block,
                              uint8_t* destination) noexcept {
    if (destination == nullptr || block >= maximum_edid_blocks) {
        return DpProbeStatus::invalid_argument;
    }

    I2cStop stop(io);

    if (block >= 2) {
        const auto segment_status = i2c_write_byte(io, ddc_segment_address,
                                                   static_cast<uint8_t>(block / 2));

        if (segment_status != DpProbeStatus::success) {
            return segment_status;
        }
    }

    const auto offset_status = i2c_write_byte(io, ddc_edid_address,
                                              static_cast<uint8_t>((block & 1u) *
                                                                   edid_block_bytes));

    if (offset_status != DpProbeStatus::success) {
        return offset_status;
    }

    for (uint32_t offset = 0; offset < edid_block_bytes;) {
        uint8_t scratch[dp_aux_payload_bytes]{};
        const uint8_t requested = static_cast<uint8_t>(
            edid_block_bytes - offset < dp_aux_payload_bytes ?
            edid_block_bytes - offset : dp_aux_payload_bytes);
        const bool more = offset + requested < edid_block_bytes;
        const AuxRequest request{AuxOperation::i2c_read, ddc_edid_address,
                                 nullptr, scratch, requested, more};
        uint8_t received = 0;
        const DpProbeStatus status = exchange(io, request, received);

        if (status != DpProbeStatus::success) {
            return status;
        }

        if (received == 0) {
            return DpProbeStatus::short_transfer;
        }

        for (uint8_t index = 0; index < received; ++index) {
            destination[offset + index] = scratch[index];
        }

        offset += received;

        if (!more && received != requested) {
            // A short final packet left the I2C transaction open. Subsequent
            // packets would require a new MOT request and could skip bytes.
            return DpProbeStatus::short_transfer;
        }
    }

    stop.completed();
    return DpProbeStatus::success;
}

DpProbeStatus read_edid(const AuxIo& io, DpProbeScratch& scratch,
                        SinkSnapshot& output) noexcept {
    DpProbeStatus status = read_edid_block(io, 0, scratch.edid);

    if (status != DpProbeStatus::success) {
        return status;
    }

    const uint32_t blocks = static_cast<uint32_t>(scratch.edid[126]) + 1u;

    if (blocks > maximum_edid_blocks) {
        return DpProbeStatus::invalid_edid;
    }

    for (uint32_t block = 1; block < blocks; ++block) {
        status = read_edid_block(io, block,
                                 scratch.edid + block * edid_block_bytes);

        if (status != DpProbeStatus::success) {
            return status;
        }
    }

    const ByteView view(scratch.edid, blocks * edid_block_bytes);

    // parse_edid commits atomically. Parsing into the caller's unexposed,
    // device-owned candidate avoids a second large kernel-stack snapshot.
    if (parse_edid(view, output) != Status::success) {
        return DpProbeStatus::invalid_edid;
    }

    return DpProbeStatus::success;
}

} // namespace

DpProbeStatus dp_aux_read(const AuxIo& io, uint32_t address,
                          uint8_t* destination, uint8_t bytes) noexcept {
    if (destination == nullptr || bytes == 0 || bytes > dp_aux_payload_bytes ||
        address > 0xfffffu || bytes - 1u > 0xfffffu - address) {
        return DpProbeStatus::invalid_argument;
    }

    uint8_t scratch[dp_aux_payload_bytes]{};
    const AuxRequest request{AuxOperation::native_read, address, nullptr,
                             scratch, bytes, false};
    uint8_t received = 0;
    const DpProbeStatus status = exchange(io, request, received);

    if (status != DpProbeStatus::success) {
        return status;
    }

    if (received != bytes) {
        return DpProbeStatus::short_transfer;
    }

    for (uint8_t index = 0; index < bytes; ++index) {
        destination[index] = scratch[index];
    }

    return DpProbeStatus::success;
}

DpProbeStatus dp_aux_write(const AuxIo& io, uint32_t address,
                           const uint8_t* source, uint8_t bytes) noexcept {
    if (source == nullptr || bytes == 0 || bytes > dp_aux_payload_bytes ||
        address > 0xfffffu || bytes - 1u > 0xfffffu - address) {
        return DpProbeStatus::invalid_argument;
    }

    const AuxRequest request{AuxOperation::native_write, address, source,
                             nullptr, bytes, false};
    uint8_t written = 0;
    const DpProbeStatus status = exchange(io, request, written);

    if (status != DpProbeStatus::success) {
        return status;
    }

    return written == bytes ? DpProbeStatus::success : DpProbeStatus::short_transfer;
}

DpProbeStatus dp_link_budget(uint8_t rate_code, uint8_t lanes,
                             bool rgb8_verified, LinkCapabilities& output) noexcept {
    if (!valid_rate(rate_code) || !valid_lanes(lanes)) {
        return DpProbeStatus::invalid_argument;
    }

    const uint64_t coded_bits = static_cast<uint64_t>(rate_code) *
                                link_rate_code_bits_per_second * lanes;
    const uint64_t payload_bits = (coded_bits / 10u) * 8u;
    const uint64_t usable_bits = (payload_bits / usable_payload_denominator) *
                                 usable_payload_numerator;
    const uint64_t maximum_clock = usable_bits /
                                   (rgb8_bits_per_pixel * 1000u);

    if (maximum_clock == 0 || maximum_clock > UINT32_MAX) {
        return DpProbeStatus::unsupported_receiver;
    }

    output = LinkCapabilities{Transport::display_port_sst,
                              static_cast<uint32_t>(maximum_clock), usable_bits,
                              rgb8_verified};
    return DpProbeStatus::success;
}

DpProbeStatus probe_dp_receiver(const AuxIo& io, const DpSourceCapabilities& source,
                                DpReceiverCapabilities& receiver,
                                LinkCapabilities& link) noexcept {
    if (io.connected == nullptr || io.transfer == nullptr ||
        io.wait_microseconds == nullptr) {
        return DpProbeStatus::invalid_argument;
    }

    if (!source.verified || !valid_rate(source.maximum_rate_code) ||
        !valid_lanes(source.maximum_lanes)) {
        return DpProbeStatus::unsupported_source;
    }

    if (!io.connected(io.context)) {
        return DpProbeStatus::disconnected;
    }

    uint8_t raw[dp_receiver_capability_bytes]{};
    DpProbeStatus status = dp_aux_read(io, dpcd_receiver_base, raw,
                                       dp_receiver_capability_bytes);

    if (status != DpProbeStatus::success) {
        return status;
    }

    const uint8_t revision = raw[0];
    const uint8_t receiver_rate = raw[1];
    const uint8_t receiver_lanes = raw[2] & dpcd_lane_mask;

    if (revision < dpcd_minimum_revision ||
        revision > dpcd_maximum_legacy_revision ||
        !valid_rate(receiver_rate) || !valid_lanes(receiver_lanes) ||
        (raw[5] & dpcd_downstream_present) != 0 ||
        (revision >= 0x12 && (raw[6] & dpcd_8b10b_coding) == 0)) {
        return DpProbeStatus::unsupported_receiver;
    }

    uint8_t mst = 0;

    if (revision >= 0x12) {
        status = dp_aux_read(io, dpcd_mst_capability, &mst, 1);

        if (status != DpProbeStatus::success) {
            return status;
        }
    }

    if (!io.connected(io.context)) {
        return DpProbeStatus::disconnected;
    }

    const uint8_t rate = source.maximum_rate_code < receiver_rate ?
                         source.maximum_rate_code : receiver_rate;
    const uint8_t lanes = source.maximum_lanes < receiver_lanes ?
                          source.maximum_lanes : receiver_lanes;
    LinkCapabilities candidate_link{};
    status = dp_link_budget(rate, lanes, source.rgb8_verified, candidate_link);

    if (status != DpProbeStatus::success) {
        return status;
    }

    receiver = DpReceiverCapabilities{revision, receiver_rate, receiver_lanes,
                                      (raw[2] & dpcd_enhanced_frame) != 0,
                                      (raw[2] & dpcd_training_pattern_3) != 0,
                                      static_cast<uint8_t>(raw[14] & 0x7fu),
                                      (mst & dpcd_mst_enabled) != 0,
                                      (raw[3] & dpcd_training_pattern_4) != 0};
    link = candidate_link;
    return DpProbeStatus::success;
}

DpProbeStatus probe_dp_port(const AuxIo& io, const DpSourceCapabilities& source,
                            DpProbeScratch& scratch, DpPortSnapshot& output) noexcept {
    DpPortSnapshot& candidate = scratch.candidate;
    DpProbeStatus status = probe_dp_receiver(io, source, candidate.receiver,
                                             candidate.link);

    if (status != DpProbeStatus::success) {
        return status;
    }

    status = read_edid(io, scratch, candidate.sink);

    if (status != DpProbeStatus::success) {
        return status;
    }

    if (!io.connected(io.context)) {
        return DpProbeStatus::disconnected;
    }

    // EDID's unknown digital depth is not proof of RGB8. A real native
    // backend can implement a separate sink-specific proof before advertising.
    candidate.link.rgb_8bpc_verified = candidate.link.rgb_8bpc_verified &&
                                        candidate.sink.digital_input &&
                                        candidate.sink.bits_per_channel >= 8;
    bool admitted = false;

    for (uint32_t index = 0; index < candidate.sink.detailed_modes.count(); ++index) {
        display::Timing timing{};

        if (prepare_output_mode(candidate.sink, index, candidate.link,
                                timing) == Status::success) {
            admitted = true;
            break;
        }
    }

    if (!admitted) {
        return DpProbeStatus::no_rgb8_mode;
    }

    output = candidate;
    return DpProbeStatus::success;
}

} // namespace rx7900::output

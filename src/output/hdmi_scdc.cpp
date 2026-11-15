/*
Module: rx7900.output.hdmi_scdc
Purpose: DC_I2C software transactions and HDMI SCDC TMDS configuration.
Created: 2026-10-24
Architecture: Exact v6.12 dcn_3_2_0_offset.h/_sh_mask.h DC_I2C values (BASE_IDX
2) and MICROSECOND_TIME_BASE_DIV (BASE_IDX 1); sequence of dce_i2c_hw.c.
Copyright 2012-2023 Advanced Micro Devices, Inc.
Linux notice and provenance: licenses/AMDGPU-OUTPUT.txt.
*/
#include "hdmi_scdc.hpp"

namespace rx7900::output {
namespace {

constexpr uint32_t i2c_control = 0x1e98u;
constexpr uint32_t i2c_arbitration = 0x1e99u;
constexpr uint32_t i2c_sw_status = 0x1e9bu;
constexpr uint32_t i2c_ddc1_hw_status = 0x1e9cu;
constexpr uint32_t i2c_ddc1_speed = 0x1ea2u;
constexpr uint32_t i2c_ddc1_setup = 0x1ea3u;
constexpr uint32_t i2c_transaction0 = 0x1eaeu;
constexpr uint32_t i2c_data = 0x1eb2u;
constexpr uint32_t microsecond_time_base_div = 0x007bu;

constexpr uint32_t control_go = 0x1u;
constexpr uint32_t control_soft_reset = 0x2u;
constexpr uint32_t control_send_reset = 0x4u;
constexpr uint32_t control_status_reset = 0x8u;
constexpr uint32_t control_ddc_select = 0x700u;
constexpr uint32_t control_transaction_count = 0x300000u;
constexpr uint32_t arbitration_rw_status = 0xcu;
constexpr uint32_t arbitration_no_queued_go = 0x10u;
constexpr uint32_t arbitration_sw_request = 0x100000u;
constexpr uint32_t arbitration_sw_done = 0x200000u;
constexpr uint32_t status_field = 0x3u;
constexpr uint32_t status_done = 0x4u;
constexpr uint32_t status_aborted = 0x10u;
constexpr uint32_t status_timeout = 0x20u;
constexpr uint32_t status_nack = 0x100u;
constexpr uint32_t hw_status_field = 0x3u;
constexpr uint32_t speed_threshold = 0x3u;
constexpr uint32_t speed_start_stop = 0x300u;
constexpr uint32_t speed_prescale = 0xffff0000u;
constexpr uint32_t setup_data_drive_en = 0x1u;
constexpr uint32_t setup_data_drive_sel = 0x2u;
constexpr uint32_t setup_send_reset_length = 0x4u;
constexpr uint32_t setup_enable = 0x40u;
constexpr uint32_t setup_clk_drive_en = 0x80u;
constexpr uint32_t setup_intra_byte_delay = 0xff00u;
constexpr uint32_t setup_intra_transaction_delay = 0xff0000u;
constexpr uint32_t setup_time_limit = 0xff000000u;
constexpr uint32_t transaction_rw = 0x1u;
constexpr uint32_t transaction_stop_on_nack = 0x100u;
constexpr uint32_t transaction_start = 0x1000u;
constexpr uint32_t transaction_stop = 0x2000u;
constexpr uint32_t data_rw = 0x1u;
constexpr uint32_t data_index_write = 0x80000000u;
constexpr uint32_t time_base_div_mask = 0x7fu;
constexpr uint32_t xtal_ref_div_mask = 0x7f00u;

// DC_I2C_STATUS / DC_I2C_REG_RW_CNTL_STATUS values.
constexpr uint32_t status_idle = 0;
constexpr uint32_t status_used_by_sw = 1;
constexpr uint32_t status_used_by_hw = 2;
constexpr uint32_t rw_dmcu_only = 2;
// dce_i2c_hw.h and dcn2_i2c_hw_construct.
constexpr uint32_t timeout_clocks = 32;
constexpr uint32_t setup_time_limit_dcn = 3;
constexpr uint32_t send_reset_length_9 = 9;

enum class Channel : uint8_t { busy, succeeded, failed, timeout, no_response };

class Engine final {
public:
    Engine(const I2cEngineConfig& config, uint8_t line) noexcept
        : config_(config), line_(line) {}

    bool read(uint32_t reg, uint32_t& value) noexcept {
        return hardware::read_ip_register(config_.io, config_.space.segment2_dword_base, reg,
                                          value) &&
               value != UINT32_MAX;
    }
    bool write(uint32_t reg, uint32_t value) noexcept {
        if (!hardware::write_ip_register(config_.io, config_.space.segment2_dword_base, reg,
                                         value)) {
            uncertain_ = true;
            return false;
        }
        return true;
    }
    bool update(uint32_t reg, uint32_t field, uint32_t value) noexcept {
        uint32_t current = 0;
        if (!read(reg, current)) {
            uncertain_ = uncertain_ || touched_;
            return false;
        }
        touched_ = true;
        return write(reg, (current & ~field) | (value & field));
    }

    bool available() noexcept {
        uint32_t hw = 0;
        uint32_t arbitration = 0;
        return read(i2c_ddc1_hw_status + line_, hw) && read(i2c_arbitration, arbitration) &&
               (hw & hw_status_field) != status_used_by_hw &&
               ((arbitration & arbitration_rw_status) >> 2) != rw_dmcu_only;
    }

    bool set_speed(uint32_t speed) noexcept {
        if (speed == 0) {
            return true;
        }
        uint32_t base = 0;
        if (!hardware::read_ip_register(config_.io, config_.dccg_segment,
                                        microsecond_time_base_div, base) ||
            base == UINT32_MAX) {
            uncertain_ = uncertain_ || touched_;
            return false;
        }
        uint32_t xtal_ref_div = (base & xtal_ref_div_mask) >> 8;
        if (xtal_ref_div == 0) {
            xtal_ref_div = 2;
        }
        const uint32_t time_base = base & time_base_div_mask;
        const uint32_t reference = time_base != 0 ? time_base * 1000u :
                                   (config_.xtalin_khz >> 1) * 2u;
        const uint32_t prescale = (reference / xtal_ref_div) / speed;
        const uint32_t value = ((prescale << 16) & speed_prescale) | 2u |
                               ((speed > 50u ? 2u : 1u) << 8);
        return update(i2c_ddc1_speed + 2u * line_,
                      speed_prescale | speed_threshold | speed_start_stop, value);
    }

    bool setup() noexcept {
        if (!update(i2c_arbitration, arbitration_sw_request, arbitration_sw_request) ||
            !set_speed(config_.speed_khz) ||
            !update(i2c_control,
                    control_go | control_soft_reset | control_send_reset | control_status_reset |
                        control_transaction_count | control_ddc_select,
                    control_status_reset | (static_cast<uint32_t>(line_) << 8))) {
            return false;
        }
        const uint32_t setup_value = (setup_time_limit_dcn << 24) |
                                     ((send_reset_length_9 << 2) & setup_send_reset_length) |
                                     setup_enable;
        return update(i2c_ddc1_setup + 2u * line_,
                      setup_time_limit | setup_send_reset_length | setup_enable, setup_value) &&
               update(i2c_arbitration, arbitration_no_queued_go, 0);
    }

    Channel status() noexcept {
        uint32_t value = 0;
        if (!read(i2c_sw_status, value)) {
            return Channel::failed;
        }
        if ((value & status_field) == status_used_by_sw) {
            return Channel::busy;
        }
        if ((value & status_nack) != 0) {
            return Channel::no_response;
        }
        if ((value & status_timeout) != 0) {
            return Channel::timeout;
        }
        if ((value & status_aborted) != 0) {
            return Channel::failed;
        }
        return Channel::succeeded;
    }

    bool hw_busy() noexcept {
        uint32_t value = 0;
        if (!read(i2c_sw_status, value)) {
            return true;
        }
        return (value & status_field) != status_idle && !available();
    }

    // process_transaction: returns false on a refusal, sets executed when this
    // payload closed the batch.
    bool process(const I2cPayload& payload, bool last_payload, bool& executed) noexcept {
        executed = false;
        if (hw_busy()) {
            return false;
        }
        const bool reading = !payload.write;
        // last_transaction: fourth slot, a final write, or any read.
        const bool last = transaction_count_ == 3 || (payload.write && last_payload) || reading;
        uint32_t transaction = transaction_stop_on_nack | transaction_start |
                               (reading ? transaction_rw : 0) |
                               (static_cast<uint32_t>(payload.length) << 16) |
                               (last ? transaction_stop : 0);
        if (!write(i2c_transaction0 + transaction_count_, transaction)) {
            return false;
        }
        const uint32_t address = (static_cast<uint32_t>(payload.address) << 1) | (reading ? 1u : 0u);
        if (transaction_count_ == 0) {
            if (!write(i2c_data, (address << 8) | data_index_write)) {
                return false;
            }
            buffer_used_write_ = 0;
        } else if (!write(i2c_data, address << 8)) {
            return false;
        }
        ++buffer_used_write_;
        uint32_t length = payload.length;
        if (!reading) {
            for (uint32_t index = 0; index < payload.length; ++index) {
                if (!write(i2c_data, static_cast<uint32_t>(payload.write_data[index]) << 8)) {
                    return false;
                }
                ++buffer_used_write_;
            }
            length = 0; // Linux consumes length while pushing bytes.
        }
        ++transaction_count_;
        buffer_used_bytes_ += length + 1u;
        if (!last) {
            return true;
        }
        if (hw_busy()) {
            return false;
        }
        // reset_hw_engine, then execute_transaction.
        if (!update(i2c_control, control_status_reset, control_status_reset) ||
            !update(i2c_ddc1_setup + 2u * line_,
                    setup_data_drive_en | setup_clk_drive_en | setup_data_drive_sel |
                        setup_intra_transaction_delay | setup_intra_byte_delay,
                    0) ||
            !update(i2c_control,
                    control_soft_reset | control_status_reset | control_send_reset | control_go |
                        control_transaction_count,
                    (transaction_count_ - 1u) << 20) ||
            !update(i2c_control, control_go, control_go)) {
            uncertain_ = true;
            return false;
        }
        transaction_count_ = 0;
        buffer_used_bytes_ = 0;
        executed = true;
        return true;
    }

    uint32_t timeout_polls(uint32_t length) const noexcept {
        if (config_.speed_khz == 0) {
            return 0;
        }
        const uint32_t period = (1000u * timeout_clocks) / config_.speed_khz;
        const uint32_t stretches = 1u + (length << 3) + 1u + (buffer_used_bytes_ << 3) +
                                   (transaction_count_ << 1);
        return period * stretches;
    }

    Channel wait(uint32_t polls) noexcept {
        if (polls == 0) {
            return Channel::succeeded;
        }
        Channel result = Channel::busy;
        uint32_t index = 0;
        do {
            result = status();
            if (result != Channel::busy) {
                break;
            }
            config_.io.delay_microseconds(config_.io.context, 1);
            ++index;
        } while (index < polls);
        return result;
    }

    bool reply(I2cPayload& payload) noexcept {
        if (!write(i2c_data, (buffer_used_write_ << 16) | data_rw | data_index_write)) {
            return false;
        }
        for (uint32_t index = 0; index < payload.length; ++index) {
            uint32_t value = 0;
            if (!read(i2c_data, value)) {
                return false;
            }
            payload.read_data[index] = static_cast<uint8_t>((value >> 8) & 0xffu);
        }
        return true;
    }

    bool release() noexcept {
        uint32_t value = 0;
        if (!read(i2c_sw_status, value)) {
            uncertain_ = true;
            return false;
        }
        const bool safe_to_reset = (value & status_field) == status_used_by_sw;
        const bool reset = safe_to_reset ?
            update(i2c_control, control_soft_reset | control_status_reset,
                   control_soft_reset | control_status_reset) :
            update(i2c_control, control_status_reset, control_status_reset);
        // engine_keep_power_up_count is 1: DC_I2C_DDCn_ENABLE stays set.
        return reset && set_speed(config_.speed_khz) &&
               update(i2c_arbitration, arbitration_sw_done | arbitration_sw_request,
                      arbitration_sw_done);
    }

    uint32_t buffer_used_bytes() const noexcept { return buffer_used_bytes_; }
    bool uncertain() const noexcept { return uncertain_; }

private:
    I2cEngineConfig config_;
    uint8_t line_ = 0;
    uint32_t transaction_count_ = 0;
    uint32_t buffer_used_bytes_ = 0;
    uint32_t buffer_used_write_ = 0;
    bool touched_ = false;
    bool uncertain_ = false;
};

I2cResult channel_result(Channel channel) noexcept {
    switch (channel) {
    case Channel::succeeded: return I2cResult::success;
    case Channel::no_response: return I2cResult::no_response;
    case Channel::timeout:
    case Channel::busy: return I2cResult::timeout;
    default: return I2cResult::failed;
    }
}

bool valid_payloads(const I2cPayload* payloads, uint32_t count) noexcept {
    if (payloads == nullptr || count == 0 || count > i2c_maximum_payloads) {
        return false;
    }
    uint32_t bytes = 0;
    for (uint32_t index = 0; index < count; ++index) {
        const I2cPayload& payload = payloads[index];
        if (payload.address > 0x7fu || (payload.length != 0 && (payload.write ?
            payload.write_data == nullptr : payload.read_data == nullptr))) {
            return false;
        }
        bytes += payload.length + 1u;
    }
    return bytes < i2c_buffer_bytes;
}

} // namespace

I2cResult dcn32_i2c_command(const I2cEngineConfig& config, uint8_t ddc_line,
                            I2cPayload* payloads, uint32_t count) noexcept {
    if (!hardware::register_io_valid(config.io) || !config.space.discovery_verified ||
        config.dccg_segment == 0 || config.xtalin_khz == 0 || config.speed_khz == 0 ||
        ddc_line >= dcn32_link_count || !valid_payloads(payloads, count)) {
        return I2cResult::invalid_argument;
    }
    Engine engine(config, ddc_line);
    if (!engine.available()) {
        return I2cResult::busy;
    }
    DdcPadSave pad{};
    const DdcPadResult opened = open_ddc_pad(config.io, config.space, ddc_line, DdcMode::i2c, pad);
    if (opened != DdcPadResult::success) {
        if (pad.valid) {
            return close_ddc_pad(config.io, config.space, pad) == DdcPadResult::success ?
                   I2cResult::busy : I2cResult::state_unknown;
        }
        return opened == DdcPadResult::state_unknown ? I2cResult::state_unknown : I2cResult::busy;
    }

    I2cResult result = I2cResult::success;
    if (!engine.setup() || !engine.set_speed(config.speed_khz)) {
        result = I2cResult::failed;
    }
    for (uint32_t index = 0; result == I2cResult::success && index < count; ++index) {
        I2cPayload& payload = payloads[index];
        if (payload.length >= i2c_buffer_bytes - engine.buffer_used_bytes()) {
            result = I2cResult::invalid_argument;
            break;
        }
        const uint32_t polls = engine.timeout_polls(payload.length + 1u);
        bool executed = false;
        if (!engine.process(payload, index + 1u == count, executed)) {
            result = I2cResult::failed;
            break;
        }
        result = channel_result(engine.wait(polls));
        if (result == I2cResult::success && executed && !payload.write &&
            !engine.reply(payload)) {
            result = I2cResult::failed;
        }
    }

    const bool released = engine.release();
    const bool closed = close_ddc_pad(config.io, config.space, pad) == DdcPadResult::success;
    if (!released || !closed || engine.uncertain()) {
        return I2cResult::state_unknown;
    }
    return result;
}

I2cResult hdmi_write_scdc(const I2cEngineConfig& config, uint8_t ddc_line,
                          uint32_t tmds_khz, bool lte_340_scramble,
                          ScdcReport& report) noexcept {
    report = {};
    uint8_t offset = hdmi_scdc_sink_version;
    uint8_t version = 0;
    I2cPayload query[2]{};
    query[0] = {true, hdmi_scdc_address, 1, &offset, nullptr};
    query[1] = {false, hdmi_scdc_address, 1, nullptr, &version};
    I2cResult result = dcn32_i2c_command(config, ddc_line, query, 2);
    if (result == I2cResult::state_unknown || result == I2cResult::invalid_argument) {
        return result;
    }
    // link_query_ddc_data result is not checked by write_scdc_data: a failed
    // read simply leaves sink_version 0 and skips the source version write.
    report.sink_version = result == I2cResult::success ? version : 0;
    if (report.sink_version == 1) {
        const uint8_t source[2]{hdmi_scdc_source_version, 1};
        I2cPayload write{true, hdmi_scdc_address, 2, source, nullptr};
        result = dcn32_i2c_command(config, ddc_line, &write, 1);
        if (result == I2cResult::state_unknown) {
            return result;
        }
        report.source_version_written = result == I2cResult::success;
    }
    const uint8_t value = tmds_khz > hdmi_scrambling_threshold_khz ? 3 :
                          (lte_340_scramble ? 1 : 0);
    const uint8_t config_bytes[2]{hdmi_scdc_tmds_config, value};
    I2cPayload write{true, hdmi_scdc_address, 2, config_bytes, nullptr};
    result = dcn32_i2c_command(config, ddc_line, &write, 1);
    if (result == I2cResult::success) {
        report.tmds_config = value;
    }
    return result;
}

I2cResult hdmi_read_scdc(const I2cEngineConfig& config, uint8_t ddc_line,
                         ScdcReport& report) noexcept {
    uint8_t offset = hdmi_scdc_tmds_config;
    uint8_t value = 0;
    I2cPayload query[2]{};
    query[0] = {true, hdmi_scdc_address, 1, &offset, nullptr};
    query[1] = {false, hdmi_scdc_address, 1, nullptr, &value};
    I2cResult result = dcn32_i2c_command(config, ddc_line, query, 2);
    if (result != I2cResult::success) {
        return result;
    }
    report.tmds_config = value;
    if ((value & 1u) == 0) {
        return result;
    }
    const uint8_t offsets[2]{hdmi_scdc_scrambler_status, hdmi_scdc_status_flags};
    uint8_t* outputs[2]{&report.scrambler_status, &report.status_flags};
    for (uint32_t index = 0; index < 2; ++index) {
        query[0] = {true, hdmi_scdc_address, 1, &offsets[index], nullptr};
        query[1] = {false, hdmi_scdc_address, 1, nullptr, outputs[index]};
        result = dcn32_i2c_command(config, ddc_line, query, 2);
        if (result != I2cResult::success) {
            return result;
        }
    }
    return result;
}

I2cResult hdmi_scdc_legacy_mode(const I2cEngineConfig& config, uint8_t ddc_line,
                                ScdcReport& report) noexcept {
    return hdmi_write_scdc(config, ddc_line, hdmi_scdc_legacy_tmds_khz, false, report);
}

} // namespace rx7900::output

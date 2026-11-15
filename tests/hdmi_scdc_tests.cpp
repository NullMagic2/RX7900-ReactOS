/*
Module: rx7900.tests.hdmi_scdc
Purpose: Check DC_I2C hardware transactions over a DDC line and HDMI SCDC setup.
Created: 2026-10-24
Architecture: Synthetic DC_I2C engine and SCDC sink on the shared DCN register
model; expected values follow Linux v6.12 dce_i2c_hw.c and link_ddc.c. No
physical GPU access.
*/
#include "dcn32_display_model.hpp"
#include "hdmi_scdc.hpp"

namespace {
using namespace rx7900::output;
using rx7900_test::Checks;
namespace dcn = rx7900_test::dcn;

constexpr uint32_t seg2 = dcn::segment2;
constexpr uint32_t control = 0x1e98u;
constexpr uint32_t arbitration = 0x1e99u;
constexpr uint32_t sw_status = 0x1e9bu;
constexpr uint32_t hw_status1 = 0x1e9cu;
constexpr uint32_t speed1 = 0x1ea2u;
constexpr uint32_t setup1 = 0x1ea3u;
constexpr uint32_t transaction0 = 0x1eaeu;
constexpr uint32_t data = 0x1eb2u;

struct Sink final {
    uint8_t registers[256]{};
    uint8_t pointer = 0;
    uint8_t address = 0x54;
    bool present = true;
    bool hang = false;
    uint32_t transfers = 0;
    std::vector<std::pair<uint8_t, uint8_t>> written;
};

struct Engine final {
    std::vector<uint8_t> buffer = std::vector<uint8_t>(1024, 0);
    uint32_t index = 0;
    bool reading = false;
};

void install(dcn::Model& model, Sink& sink, Engine& engine) {
    model.set(dcn::segment1, 0x007bu, 0x00000264u); // time base 100, XTAL_REF_DIV 2
    model.on_read = [&engine](dcn::Model&, uint32_t offset, uint32_t& value) {
        if (offset != dcn::byte_offset(seg2, data) || !engine.reading) {
            return false;
        }
        value = static_cast<uint32_t>(engine.buffer.at(engine.index++)) << 8;
        return true;
    };
    model.on_write = [&sink, &engine](dcn::Model& m, uint32_t offset, uint32_t value) {
        if (offset == dcn::byte_offset(seg2, data)) {
            if ((value & 0x80000000u) != 0) {
                engine.index = (value >> 16) & 0x3ffu;
            }
            engine.reading = (value & 1u) != 0;
            if (!engine.reading) {
                engine.buffer.at(engine.index++) = static_cast<uint8_t>(value >> 8);
            }
            return;
        }
        if (offset != dcn::byte_offset(seg2, control) || (value & 1u) == 0) {
            return;
        }
        ++sink.transfers;
        // DC_I2C_GO is a trigger: hardware clears it once the batch starts.
        m.values[offset] = value & ~1u;
        if (sink.hang) {
            m.set(seg2, sw_status, 1u); // USED_BY_SW forever
            return;
        }
        const uint32_t count = ((value >> 20) & 3u) + 1u;
        uint32_t cursor = 0;
        uint32_t written_bytes = 0;
        std::vector<uint8_t> reads;
        bool nack = false;
        for (uint32_t t = 0; t < count && !nack; ++t) {
            const uint32_t transaction = m.get(seg2, transaction0 + t);
            const bool read = (transaction & 1u) != 0;
            const uint32_t length = (transaction >> 16) & 0x3ffu;
            const uint8_t address = engine.buffer.at(cursor++);
            ++written_bytes;
            if (!sink.present || (address >> 1) != sink.address || ((address & 1u) != 0) != read) {
                nack = true;
                break;
            }
            if (!read) {
                for (uint32_t byte = 0; byte < length; ++byte) {
                    const uint8_t item = engine.buffer.at(cursor++);
                    ++written_bytes;
                    if (byte == 0) {
                        sink.pointer = item;
                    } else {
                        sink.written.emplace_back(sink.pointer, item);
                        sink.registers[sink.pointer++] = item;
                    }
                }
            } else {
                for (uint32_t byte = 0; byte < length; ++byte) {
                    reads.push_back(sink.registers[sink.pointer++]);
                }
            }
        }
        for (size_t byte = 0; byte < reads.size(); ++byte) {
            engine.buffer.at(written_bytes + byte) = reads[byte];
        }
        m.set(seg2, sw_status, nack ? 0x104u : 0x4u);
    };
}

I2cEngineConfig config(dcn::Model& model) {
    I2cEngineConfig value{};
    value.io = model.io();
    value.space = dcn::space();
    value.dccg_segment = dcn::segment1;
    value.xtalin_khz = 100000;
    value.speed_khz = 100;
    return value;
}

void check_scdc_write(Checks& checks) {
    dcn::Model model;
    Sink sink;
    Engine engine;
    install(model, sink, engine);
    sink.registers[0x01] = 1; // SCDC sink version 1
    const uint8_t line = 1;
    model.set(seg2, 0x28d0u + 4u * line, 0x00000101u); // GPIO masks owned by software
    ScdcReport report{};
    checks.expect(hdmi_write_scdc(config(model), line, 594000, false, report) == I2cResult::success,
                  "SCDC write for 594 MHz");
    checks.expect(report.sink_version == 1 && report.source_version_written && report.tmds_config == 3,
                  "sink version 1 gets source version, TMDS_CONFIG 3");
    checks.expect(sink.written.size() == 2 && sink.written[0] == std::make_pair<uint8_t, uint8_t>(0x02, 1) &&
                      sink.written[1] == std::make_pair<uint8_t, uint8_t>(0x20, 3),
                  "write_scdc_data byte sequence");
    checks.expect(sink.transfers == 3, "query, source version, TMDS config");
    checks.expect(model.get(seg2, speed1 + 2u) == ((500u << 16) | 2u | (2u << 8)),
                  "DC_I2C_DDC2_SPEED: prescale 500, threshold 2, start/stop 2");
    checks.expect(model.get(seg2, setup1 + 2u) == ((3u << 24) | 0x4u | 0x40u),
                  "DC_I2C_DDC2_SETUP: time limit 3, send reset length, enable");
    checks.expect(((model.get(seg2, control) >> 8) & 7u) == line, "DC_I2C_DDC_SELECT");
    checks.expect((model.get(seg2, arbitration) & 0x300000u) == 0x200000u,
                  "engine released (SW_DONE_USING, no request)");
    checks.expect((model.get(seg2, 0x28d0u + 4u * line) & 0x101u) == 0x101u, "DDC pad restored");
    const std::vector<uint32_t> transactions = model.written(seg2, transaction0);
    checks.expect(!transactions.empty() && transactions.front() == (0x100u | 0x1000u | (1u << 16)),
                  "write transaction: STOP_ON_NACK, START, one byte, no STOP before the read");
    checks.expect(model.written(seg2, transaction0 + 1u).front() ==
                      (0x100u | 0x1000u | 1u | (1u << 16) | 0x2000u),
                  "read transaction: RW, STOP");

    dcn::Model low_model;
    Sink low;
    Engine low_engine;
    install(low_model, low, low_engine);
    checks.expect(hdmi_write_scdc(config(low_model), 0, 297000, true, report) == I2cResult::success &&
                      report.sink_version == 0 && !report.source_version_written &&
                      report.tmds_config == 1 && low.written.size() == 1 &&
                      low.written[0] == std::make_pair<uint8_t, uint8_t>(0x20, 1),
                  "version-0 sink with LTE_340MCSC_SCRAMBLE");
    low.written.clear();
    low.registers[0x01] = 2;
    checks.expect(hdmi_write_scdc(config(low_model), 0, 594000, false, report) == I2cResult::success &&
                      report.sink_version == 2 && !report.source_version_written &&
                      low.written.size() == 1,
                  "only a version-1 sink gets SOURCE_VERSION");
    low.registers[0x01] = 0;
    low.written.clear();
    checks.expect(hdmi_write_scdc(config(low_model), 0, 340000, false, report) == I2cResult::success &&
                      low.written.size() == 1 && low.written[0].second == 0,
                  "340 MHz is not above the threshold");
}

void check_scdc_read(Checks& checks) {
    dcn::Model model;
    Sink sink;
    Engine engine;
    install(model, sink, engine);
    sink.registers[0x20] = 3;
    sink.registers[0x21] = 1;
    sink.registers[0x40] = 0x0f;
    ScdcReport report{};
    checks.expect(hdmi_read_scdc(config(model), 0, report) == I2cResult::success &&
                      report.tmds_config == 3 && report.scrambler_status == 1 &&
                      report.status_flags == 0x0f,
                  "read_scdc_data with scrambling");
    sink.registers[0x20] = 0;
    sink.transfers = 0;
    checks.expect(hdmi_read_scdc(config(model), 0, report) == I2cResult::success && sink.transfers == 1,
                  "no scrambler status read when scrambling is off");
}

void check_failures(Checks& checks) {
    {
        dcn::Model model;
        Sink sink;
        Engine engine;
        install(model, sink, engine);
        sink.present = false;
        ScdcReport report{};
        checks.expect(hdmi_write_scdc(config(model), 0, 594000, false, report) == I2cResult::no_response &&
                      report.tmds_config == 0,
                      "absent sink NACK");
    }
    {
        dcn::Model model;
        Sink sink;
        Engine engine;
        install(model, sink, engine);
        model.set(seg2, hw_status1 + 3u, 2u); // DDC4 used by hardware
        uint8_t value = 0;
        I2cPayload read{false, 0x50, 1, nullptr, &value};
        checks.expect(dcn32_i2c_command(config(model), 3, &read, 1) == I2cResult::busy &&
                          model.writes.empty(),
                      "engine used by hardware: refused before any write");
        model.set(seg2, hw_status1 + 3u, 0u);
        model.set(seg2, arbitration, 0x8u); // REG_RW_CNTL_STATUS DMCU_ONLY
        checks.expect(dcn32_i2c_command(config(model), 3, &read, 1) == I2cResult::busy,
                      "DMCU-only arbitration refused");
    }
    {
        dcn::Model model;
        Sink sink;
        Engine engine;
        install(model, sink, engine);
        sink.hang = true;
        uint8_t value = 0;
        I2cPayload read{false, 0x54, 1, nullptr, &value};
        checks.expect(dcn32_i2c_command(config(model), 0, &read, 1) == I2cResult::timeout,
                      "never-completing transfer times out");
        // get_transaction_timeout_hw(2, 100 kHz) = 320 * (1 + 16 + 1) polls of 1 us.
        checks.expect(model.delayed_us >= 5760u && model.delayed_us < 5760u + 3000u,
                      "bounded by the Linux transaction timeout");
        checks.expect((model.get(seg2, control) & 0xau) == 0xau,
                      "release soft-resets an engine still used by software");
    }
    {
        dcn::Model model;
        Sink sink;
        Engine engine;
        install(model, sink, engine);
        model.fail_write = 9;
        uint8_t value = 0;
        I2cPayload read{false, 0x54, 1, nullptr, &value};
        checks.expect(dcn32_i2c_command(config(model), 0, &read, 1) == I2cResult::state_unknown,
                      "failed engine write is state_unknown");
    }
    dcn::Model model;
    uint8_t value = 0;
    I2cPayload too_many[5]{};
    for (I2cPayload& payload : too_many) {
        payload = {false, 0x54, 1, nullptr, &value};
    }
    checks.expect(dcn32_i2c_command(config(model), 0, too_many, 5) == I2cResult::invalid_argument &&
                      dcn32_i2c_command(config(model), 5, too_many, 1) == I2cResult::invalid_argument &&
                      model.writes.empty(),
                  "five payloads or line 5 refused");
    I2cPayload missing{true, 0x54, 2, nullptr, nullptr};
    checks.expect(dcn32_i2c_command(config(model), 0, &missing, 1) == I2cResult::invalid_argument,
                  "write without data refused");
    I2cPayload address{true, 0x80, 0, nullptr, nullptr};
    checks.expect(dcn32_i2c_command(config(model), 0, &address, 1) == I2cResult::invalid_argument,
                  "8-bit address refused");
}

// dcn-restore-modes (2026-10-24, additive): link_set_dpms_off's
// write_scdc_data(165000, false) takes a scrambled sink back to legacy mode.
void check_scdc_legacy(Checks& checks) {
    dcn::Model model;
    Sink sink;
    Engine engine;
    install(model, sink, engine);
    sink.registers[0x01] = 1;
    sink.registers[0x20] = 3; // scrambling + 1/40 from the enable
    model.set(seg2, 0x28d0u, 0x00000101u);
    ScdcReport report{};
    checks.expect(hdmi_scdc_legacy_mode(config(model), 0, report) == I2cResult::success && sink.registers[0x20] == 0 &&
                      report.tmds_config == 0 && report.source_version_written && sink.transfers == 3,
                  "DPMS-off SCDC: sink version query, source version, TMDS_CONFIG 0");
    dcn::Model absent_model;
    Sink absent;
    absent.present = false;
    Engine absent_engine;
    install(absent_model, absent, absent_engine);
    absent_model.set(seg2, 0x28d0u, 0x00000101u);
    checks.expect(hdmi_scdc_legacy_mode(config(absent_model), 0, report) == I2cResult::no_response,
                  "an unplugged sink does not answer (Linux ignores it)");
}

} // namespace

int main() {
    Checks checks;
    check_scdc_write(checks);
    check_scdc_read(checks);
    check_failures(checks);
    check_scdc_legacy(checks);
    return checks.finish("hdmi_scdc");
}

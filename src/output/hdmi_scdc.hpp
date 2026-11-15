/*
Module: rx7900.output.hdmi_scdc
Purpose: Bounded DCN3.2 hardware I2C over a route's DDC line and HDMI 2.0 SCDC setup.
Created: 2026-10-24
Architecture: Reuses the dcn32_route DDC pad ownership (I2C mode) and the shared
hardware::RegisterIo contract. Follows Linux v6.12 dce/dce_i2c_hw.c
(dcn2_i2c_hw_construct, acquire_i2c_hw_engine, setup_engine, set_speed,
process_transaction, execute_transaction, get_channel_status,
process_channel_reply, release_engine), dce_i2c_hw.h constants,
dcn32_resource.c (i2c_speed_in_khz 100) and link/protocols/link_ddc.c
(write_scdc_data, read_scdc_data, HDMI_SCDC_* offsets from the HDMI 2.0 SCDC map)
and link/link_dpms.c link_set_dpms_off (the legacy-mode write at DPMS off).
Linux AMD register provenance and notice: licenses/AMDGPU-OUTPUT.txt.
*/
#pragma once

#include "dcn32_route.hpp"

namespace rx7900::output {

constexpr uint8_t hdmi_scdc_address = 0x54;
constexpr uint8_t hdmi_scdc_sink_version = 0x01;
constexpr uint8_t hdmi_scdc_source_version = 0x02;
constexpr uint8_t hdmi_scdc_tmds_config = 0x20;
constexpr uint8_t hdmi_scdc_scrambler_status = 0x21;
constexpr uint8_t hdmi_scdc_status_flags = 0x40;
// write_scdc_data: TMDS character rates above 340 MHz need 1/40 bit-clock
// ratio plus scrambling (TMDS_CONFIG = 3).
constexpr uint32_t hdmi_scrambling_threshold_khz = 340000;

constexpr uint32_t i2c_maximum_payloads = 4;   // DC_I2C_TRANSACTION0..3
constexpr uint32_t i2c_buffer_bytes = 538;     // I2C_HW_BUFFER_SIZE_DCE100

struct I2cPayload final {
    bool write = false;
    uint8_t address = 0; // 7-bit slave address
    uint8_t length = 0;
    const uint8_t* write_data = nullptr;
    uint8_t* read_data = nullptr;
};

enum class I2cResult : uint8_t {
    success,
    invalid_argument,
    // Engine owned by hardware/DMCU or the pad could not be opened.
    busy,
    no_response,
    timeout,
    failed,
    // A register write was not confirmed or the engine/pad could not be
    // released: owner must quarantine the route until reset.
    state_unknown
};

struct I2cEngineConfig final {
    hardware::RegisterIo io{};
    display::Dcn32RegisterSpace space{};
    // Discovered DMU segment1 (MICROSECOND_TIME_BASE_DIV) base.
    uint32_t dccg_segment = 0;
    // Firmware-info crystal in kHz (dce_i2c_hw reference_frequency = xtal / 2).
    uint32_t xtalin_khz = 0;
    uint32_t speed_khz = 100;
};

// One command on one route's DDC line: opens the pad in I2C mode, claims the
// DC_I2C engine for software, runs up to four payloads as one GO (repeated
// START between them, STOP after the last), reads replies, then releases the
// engine and restores the pad. PASSIVE_LEVEL, exclusive with AUX on the same
// line and with every DC_I2C user. Polls in 1 us steps up to the Linux
// transaction timeout.
I2cResult dcn32_i2c_command(const I2cEngineConfig& config, uint8_t ddc_line,
                            I2cPayload* payloads, uint32_t count) noexcept;

struct ScdcReport final {
    uint8_t sink_version = 0;
    uint8_t tmds_config = 0;
    uint8_t scrambler_status = 0;
    uint8_t status_flags = 0;
    bool source_version_written = false;
};

// write_scdc_data before enabling TMDS output: reads SINK_VERSION, writes
// SOURCE_VERSION 1 for a version-1 sink, then TMDS_CONFIG 3 (> 340 MHz), 1
// (lte_340_scramble) or 0.
I2cResult hdmi_write_scdc(const I2cEngineConfig& config, uint8_t ddc_line,
                          uint32_t tmds_khz, bool lte_340_scramble,
                          ScdcReport& report) noexcept;
// read_scdc_data after enabling: TMDS_CONFIG, then scrambler status and status
// flags when scrambling is enabled.
I2cResult hdmi_read_scdc(const I2cEngineConfig& config, uint8_t ddc_line,
                         ScdcReport& report) noexcept;

// link_set_dpms_off of an HDMI stream ("inform that sink is going to use
// legacy HDMI mode"): write_scdc_data(ddc, 165000, false), the rate the VBIOS
// handles, so TMDS_CONFIG becomes 0 (no scrambling, 1/10 bit-clock ratio).
constexpr uint32_t hdmi_scdc_legacy_tmds_khz = 165000;
I2cResult hdmi_scdc_legacy_mode(const I2cEngineConfig& config, uint8_t ddc_line,
                                ScdcReport& report) noexcept;

} // namespace rx7900::output

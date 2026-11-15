/*
Module: rx7900.output.dp_link
Purpose: Bounded DisplayPort SST receiver and EDID discovery over an owned AUX channel.
Created: 2026-10-22
Architecture: Portable link-side policy. A native DCN owner supplies serialized,
time-bounded AUX/HPD operations and verified source capabilities. This module
does not enable PHY, train a link, program DCN clocks or commit scanout.
Protocol fields follow Linux v6.12 drm_dp.h and drm_dp_helper.c; see
licenses/AMDGPU-OUTPUT.txt for attribution.
*/
#pragma once

#include "sink.hpp"

namespace rx7900::output {

constexpr uint32_t dp_aux_payload_bytes = 16;
constexpr uint32_t dp_receiver_capability_bytes = 16;

enum class AuxOperation : uint8_t {
    native_read,
    native_write,
    i2c_write,
    i2c_read
};

enum class AuxResult : uint8_t {
    ack,
    defer,
    nack,
    transport_failure
};

struct AuxRequest final {
    AuxOperation operation;
    uint32_t address; // DPCD address or 7-bit I2C slave address.
    const uint8_t* write_data;
    uint8_t* read_data;
    uint8_t bytes; // 0..16; zero is a bare I2C address for STOP.
    bool middle_of_transaction;
};

struct AuxReply final {
    AuxResult result = AuxResult::transport_failure;
    uint8_t bytes = 0;
};

struct AuxIo final {
    void* context = nullptr;
    // PASSIVE_LEVEL, exclusively serialized with HPD/link programming. transfer
    // must finish each request within 10 ms and never retain request pointers.
    // For reads, write at most request.bytes into read_data. On ACK, bytes is
    // the actual transfer length. No GPU/MMIO address is accepted from callers.
    bool (*transfer)(void*, const AuxRequest&, AuxReply&) noexcept = nullptr;
    // Wait at least microseconds, with a bounded implementation; required for
    // DEFER. No request pointer survives this callback.
    bool (*wait_microseconds)(void*, uint32_t) noexcept = nullptr;
    // Read the actual connector HPD level before and after discovery.
    bool (*connected)(void*) noexcept = nullptr;
};

enum class DpProbeStatus {
    success,
    invalid_argument,
    disconnected,
    auxiliary_failure,
    auxiliary_nack,
    auxiliary_defer_exhausted,
    short_transfer,
    unsupported_receiver,
    unsupported_source,
    invalid_edid,
    no_rgb8_mode
};

// These are hardware facts, not guesses from a GPU model or PCI ID. Native DCN
// code must only set verified after the encoder/PHY lane map and rate limits have
// been checked against firmware, board wiring and current clock ownership.
struct DpSourceCapabilities final {
    uint8_t maximum_rate_code = 0; // 0x06, 0x0a, 0x14 or 0x1e.
    uint8_t maximum_lanes = 0;     // 1, 2 or 4.
    bool verified = false;
    bool rgb8_verified = false;
    bool enhanced_framing = false;
    bool training_pattern_3 = false;
    bool training_pattern_4 = false;
};

struct DpReceiverCapabilities final {
    uint8_t dpcd_revision = 0;
    uint8_t maximum_rate_code = 0;
    uint8_t maximum_lanes = 0;
    bool enhanced_framing = false;
    bool training_pattern_3 = false;
    uint8_t training_aux_interval = 0;
    // Capability alone does not mean MST is active. A native SST modeset must
    // disable/verify the MST control before using this receiver.
    bool mst_capable = false;
    bool training_pattern_4 = false;
};

struct DpPortSnapshot final {
    DpReceiverCapabilities receiver{};
    SinkSnapshot sink{};
    LinkCapabilities link{};
};

// Device-owned scratch, serialized with the AUX channel. Keeps the full EDID
// blob and candidate snapshot out of a small NT kernel stack. Caller retains
// it through probe_dp_port and never publishes it before success.
struct DpProbeScratch final {
    uint8_t edid[maximum_edid_blocks * edid_block_bytes]{};
    DpPortSnapshot candidate{};
};

// Native DPCD transactions used by discovery and link training. Exactly 1..16
// bytes, wholly inside the 20-bit DPCD space. Read output remains unchanged on
// failure. Writes are not rolled back; the caller owns source/link recovery.
DpProbeStatus dp_aux_read(const AuxIo& io, uint32_t address,
                          uint8_t* destination, uint8_t bytes) noexcept;
DpProbeStatus dp_aux_write(const AuxIo& io, uint32_t address,
                           const uint8_t* source, uint8_t bytes) noexcept;

// Exact 8b/10b payload intersection used by discovery and training. Reserves
// one percent for framing overhead; no DSC/MST/128b132b assumption.
DpProbeStatus dp_link_budget(uint8_t rate_code, uint8_t lanes,
                             bool rgb8_verified, LinkCapabilities& output) noexcept;

// Reads a complete DPCD capability block, rejects branch/128b132b-only
// transport, and intersects the verified source rate/lanes. Output is atomic.
// No mode is advertised until the native ModeBackend can validate clocks,
// bandwidth, training and scanout lifetime at prepare/apply.
DpProbeStatus probe_dp_receiver(const AuxIo& io, const DpSourceCapabilities& source,
                                DpReceiverCapabilities& receiver,
                                LinkCapabilities& link) noexcept;

// Reads at most eight complete 128-byte EDID blocks over I2C-over-AUX, then
// reuses the project's strict EDID decoder and output budget validation. The
// caller owns the AUX channel and the snapshot; output is atomic on failure.
DpProbeStatus probe_dp_port(const AuxIo& io, const DpSourceCapabilities& source,
                            DpProbeScratch& scratch, DpPortSnapshot& output) noexcept;

} // namespace rx7900::output

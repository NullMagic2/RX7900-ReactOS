/*
Module: rx7900.output.sink
Purpose: Discover bounded EDID detailed timings and validate RGB8 output budgets.
Created: 2026-10-21
Architecture: Portable sink side of the display core; transport belongs to miniport.
Linux-derived EDID decoding retains its notice in licenses/AMDGPU-OUTPUT.txt.
*/
#pragma once

#include "modes.hpp"

namespace rx7900::output {

enum class Status {
    success,
    truncated_input,
    malformed_edid,
    checksum_mismatch,
    unsupported_edid,
    mode_not_found,
    unsupported_output,
    capabilities_unavailable
};

constexpr uint32_t edid_block_bytes = 128;
// Explicit bounded policy, not an EDID specification limit. At most 46 DTDs fit.
constexpr uint32_t maximum_edid_blocks = 8;
constexpr uint32_t no_preferred_mode = UINT32_MAX;

// Device-owned value snapshot. Timings are sink candidates, not advertised modes
// until prepare_output_mode and the physical backend's validation both pass.
// parse_edid decodes only base/CTA detailed timings; display::append_edid_modes
// (display/edid_modes) appends CVT codes, standard/established timings, CTA and
// HDMI VICs, DisplayID timings and range-limit modes behind them in a device-owned
// copy (the detailed indices are unchanged). HDR, VRR, DSC, audio and HDMI/DP
// capabilities are not inferred here.
// preferred_mode refers only to the first base DTD when EDID marks it preferred.
struct SinkSnapshot final {
    display::ModeCatalog detailed_modes{};
    uint32_t preferred_mode = no_preferred_mode;
    uint32_t rejected_detailed_modes = 0;
    uint32_t unsupported_extensions = 0;
    uint32_t block_count = 0;
    uint32_t bits_per_channel = 0; // Zero means unknown, never proof of RGB8.
    bool digital_input = false;
};

// Any IRQL with resident buffers; caller serializes snapshot replacement and owns
// input for this call. No bytes/pointers or monitor identity strings are retained.
// Requires an exact whole EDID blob, all declared blocks present and checksummed.
// Only EDID 1.3/1.4 and CTA revisions 1..3 are supported. HF-EEODB is rejected.
// Bad/unsupported individual DTDs are counted and omitted; structural errors leave
// output unchanged. Uses one approximately 3 KiB temporary snapshot on the stack.
Status parse_edid(ByteView input, SinkSnapshot& output) noexcept;

enum class Transport {
    unknown,
    hdmi_tmds,
    display_port_sst
};

// Facts supplied by trusted connector/link discovery, not inferred from EDID DTDs
// or from the GPU marketing name. Backend must resolve ATOM connector routing,
// HPD, DDC/AUX, DPCD/SCDC, encoder/PHY limits and any adapters. The usable payload
// budget is after encoding and reserved transport overhead; raw lane rate is not
// a valid value. The backend must verify RGB 8 bpc end-to-end, including a sink
// with unknown base EDID depth. HDMI FRL, DP MST and compression are unsupported.
struct LinkCapabilities final {
    Transport transport = Transport::unknown;
    uint32_t maximum_pixel_clock_khz = 0;
    uint64_t usable_payload_bits_per_second = 0;
    bool rgb_8bpc_verified = false;
};

// Pure candidate validation, at any IRQL with resident device-owned arguments.
// Returns an existing display::Timing for ModeController::append_mode; this does
// not create a physical connection or commit scanout. Output unchanged on error.
// XRGB8888 scanout uses RGB8 output (24 wire bits/pixel, 32 memory bits/pixel).
// The ModeBackend must still validate DCN clocks/bandwidth, residency and current
// link generation at prepare, then perform/confirm real programming at apply.
// Missing transport/capabilities always fail; there is no physical output backend.
Status prepare_output_mode(const SinkSnapshot& sink, uint32_t sink_mode_index,
                           const LinkCapabilities& link,
                           display::Timing& output) noexcept;

} // namespace rx7900::output

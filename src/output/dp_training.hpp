/*
Module: rx7900.output.dp_training
Purpose: Bounded legacy DisplayPort SST clock-recovery/equalization policy.
Created: 2026-10-22
Architecture: Uses the owned AUX contract from dp_link. A physical DCN backend
must supply PHY/encoder programming, quiescent link ownership and source facts.
No virtual QEMU path or unverified physical output is enabled here.
Protocol fields and status predicates follow Linux v6.12 drm_dp.h and
drm_dp_helper.c; attribution is in licenses/AMDGPU-OUTPUT.txt.
*/
#pragma once

#include "dp_link.hpp"

namespace rx7900::output {

struct DpPhyIo final {
    void* context = nullptr;
    // PASSIVE_LEVEL, exclusive quiescent link owner. Must program the actual
    // source lane clock, PHY and encoder. A false return can leave unknown
    // hardware state; finish(false) must still be safe.
    bool (*configure)(void*, uint8_t rate_code, uint8_t lanes,
                      bool enhanced_framing) noexcept = nullptr;
    // Set a training pattern once per phase. Reprogramming it on each retry
    // restarts some receivers, so lane-drive retries use drive() instead.
    bool (*pattern)(void*, uint8_t pattern) noexcept = nullptr;
    // Program per-lane voltage/pre-emphasis encoded as DPCD TRAINING_LANEn_SET.
    // Neither callback may enable a video stream.
    bool (*drive)(void*, const uint8_t* lane_settings,
                  uint8_t lanes) noexcept = nullptr;
    // Disable source training. true only after sink status was proven and its
    // pattern disabled; false must quiesce/recover. Must never enable scanout.
    bool (*finish)(void*, bool trained) noexcept = nullptr;
    // Optional (dcn-dsc-odm): the encoder half of dp_set_fec_ready
    // (enc2_fec_set_ready). nullptr: the source cannot train with FEC.
    bool (*fec_ready)(void*, bool ready) noexcept = nullptr;
};

struct DpTrainingRequest final {
    uint8_t rate_code;
    uint8_t lanes;
    uint32_t sink_mode_index;
    // dcn-dsc-odm, defaulted so {rate, lanes, index} initializers are unchanged:
    // train with FEC (dp_set_fec_ready: DPCD FEC_CONFIGURATION.FEC_READY, then
    // phy.fec_ready, between the PHY enable and LINK_BW_SET), and admit the
    // mode at this compressed DSC rate (1/16 bpp, FEC overhead included)
    // instead of uncompressed RGB 8 bpc.
    bool fec_ready = false;
    uint32_t dsc_bits_per_pixel_x16 = 0;
};

enum class DpTrainingStatus {
    trained,
    invalid_argument,
    unsupported_link,
    unsupported_mode,
    disconnected,
    auxiliary_failure,
    source_failure,
    clock_recovery_failed,
    equalization_failed,
    // Disable/cleanup could not be confirmed. Owner must quarantine or reset
    // the pipeline; an earlier scanout cannot be reported as known current.
    state_unknown
};

// Trains 8b/10b RBR/HBR/HBR2/HBR3, 1/2/4 lanes. HBR3 requires source and
// sink TPS4 support. At most five status reads in each
// phase, seven AUX DEFER retries per transaction, 16 ms interval per read.
// Requires a complete prior probe, verified source capabilities and real PHY
// callbacks. It does not program DCN clocks, timing, MSA, plane or scanout.
// On success the link is trained but no desktop mode is committed. On failure
// attempts to disable sink and source training; state_unknown means cleanup
// lacked proof. Caller revalidates HPD/DPCD and mode budget before reuse.
DpTrainingStatus train_dp_sst(const AuxIo& aux, const DpPhyIo& phy,
                              const DpSourceCapabilities& source,
                              const DpPortSnapshot& port,
                              const DpTrainingRequest& request) noexcept;

} // namespace rx7900::output

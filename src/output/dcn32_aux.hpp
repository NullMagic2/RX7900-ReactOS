/*
Module: rx7900.output.dcn32_aux
Purpose: Bounded DCN3.2 software AUX engine transactions for one owned route.
Created: 2026-10-22
Architecture: Reuses the DP request/reply contract and DCN register space;
the native owner supplies a configured DDC pin, clock, mapping and lease.
Linux AMD register provenance and notice: licenses/AMDGPU-OUTPUT.txt.
*/
#pragma once

#include "dp_link.hpp"
#include "dcn32_otg.hpp"

namespace rx7900::output {

struct Dcn32AuxIo final {
    void* context = nullptr;
    bool (*read)(void*, uint32_t byte_offset, uint32_t& value) noexcept = nullptr;
    bool (*write)(void*, uint32_t byte_offset, uint32_t value) noexcept = nullptr;
    bool (*wait_microseconds)(void*, uint32_t) noexcept = nullptr;
    bool (*connected)(void*) noexcept = nullptr;
};

// Assertions from the exclusive native display owner, not inferred from BAR5.
struct Dcn32AuxPrerequisites final {
    bool exclusive_register_lease = false;
    bool exact_ddc_route_in_aux_mode = false;
    bool aux_clock_and_phy_ready = false;
    bool dmub_arbitration_owned = false;
};

// One DCN3.2 AUX0..4 engine. The owner keeps callbacks and the BAR5 mapping
// resident at PASSIVE_LEVEL and serializes transfer with reset, DDC GPIO mode,
// DMUB, removal and link changes. It may enable/reset its selected AUX engine
// after those prerequisites are proven, but never enables clocks or a link. Any
// submitted request with uncertain completion permanently quarantines the
// instance until the owner proves an actual device reset and creates a new one.
class Dcn32AuxEngine final {
public:
    Dcn32AuxEngine(Dcn32AuxIo io, display::Dcn32RegisterSpace space,
                   uint8_t engine, uint8_t verified_hpd_pin) noexcept
        : io_(io), space_(space), engine_(engine), hpd_pin_(verified_hpd_pin) {}
    Dcn32AuxEngine(const Dcn32AuxEngine&) = delete;
    Dcn32AuxEngine& operator=(const Dcn32AuxEngine&) = delete;

    bool transfer(const AuxRequest& request, AuxReply& reply,
                  const Dcn32AuxPrerequisites& prerequisites) noexcept;
    bool quarantined() const noexcept { return quarantined_; }

private:
    Dcn32AuxIo io_{};
    display::Dcn32RegisterSpace space_{};
    uint8_t engine_ = 0;
    uint8_t hpd_pin_ = 0;
    bool quarantined_ = false;
};

} // namespace rx7900::output

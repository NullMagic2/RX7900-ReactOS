/*
Module: rx7900.display.dcn32_hpd
Purpose: Read a verified Navi31 DCN3.2 HPD sense bit without changing display state.
Created: 2026-10-22
Architecture: Reuses the bounded DCN register callback and discovery token;
the native owner supplies route, mapping and lifetime proof.
Linux AMD register provenance and notice: licenses/AMDGPU-DISPLAY.txt.
*/
#pragma once

#include "dcn32_otg.hpp"

namespace rx7900::display {

enum class HpdResult : uint8_t {
    connected,
    disconnected,
    unavailable,
    read_failure
};

// The pin is a proven DCN HPD instance index 0..4, not the raw ATOM HPD ID.
// The owner must hold its mapping and register lock for the whole call. No
// writes, polls, interrupt acknowledgements or allocations occur.
HpdResult read_dcn32_hpd(RegisterIo io, const Dcn32RegisterSpace& space,
                         uint8_t verified_pin) noexcept;

} // namespace rx7900::display

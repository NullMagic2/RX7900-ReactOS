/*
Module: rx7900.display.dcn32_otg
Purpose: Program and read back timing fields of a proven disabled DCN3.2 OTG.
Created: 2026-10-21
Architecture: Hardware subcomponent; does not initialize or enable display output.
Linux AMD register provenance and notice: licenses/AMDGPU-DISPLAY.txt.
*/
#pragma once

#include "modes.hpp"

namespace rx7900::display {

struct RegisterIo final {
    void* context = nullptr;
    bool (*read)(void*, uint32_t byte_offset, uint32_t& value) noexcept = nullptr;
    bool (*write)(void*, uint32_t byte_offset, uint32_t value) noexcept = nullptr;
};

// Native owner obtains segment2 from validated IP-discovery DMU3.2.0 register
// bases. It is a BAR5-relative DWORD index, not a PCI address or GPU VA. Native
// callbacks independently check byte offsets against their live owned mapping.
struct Dcn32RegisterSpace final {
    uint32_t segment2_dword_base = 0;
    uint32_t aperture_bytes = 0;
    uint32_t dmu_major = 0;
    uint32_t dmu_minor = 0;
    uint32_t dmu_revision = 0;
    bool discovery_verified = false;
};

// Explicit facts held by a trusted device orchestrator for the entire call,
// including serialized MMIO callbacks. These cannot be established by mapping
// PCI BARs alone. Firmware/ASIC/DMUB, stable display clocks and exclusive control
// must already exist. There is no built-in way to assert these from userspace.
struct DisabledOtgPrerequisites final {
    bool resources_and_initialization_owned = false;
    bool display_clocks_verified = false;
    bool exclusive_register_lock_held = false;
    bool vtg_pipe_link_quiesced = false;
    bool single_pipe_rgb_no_odm = false;
};

enum class OtgResult {
    // Only the ten disabled timing fields were written and read back. No OTG,
    // encoder/PHY, plane or stream was enabled; this is not a physical modeset.
    disabled_fields_verified,
    invalid_timing,
    unavailable,
    unsafe_hardware_state,
    read_failure,
    // A write may have occurred without proof of all requested fields. Instance
    // is permanently blocked; owner must quarantine the pipeline until a real
    // reset/reinitialization confirms safety before constructing another object.
    quarantined
};

class Dcn32Otg final {
public:
    explicit Dcn32Otg(RegisterIo io, Dcn32RegisterSpace space,
                      uint32_t instance) noexcept
        : io_(io), space_(space), instance_(instance) {}
    Dcn32Otg(const Dcn32Otg&) = delete;
    Dcn32Otg& operator=(const Dcn32Otg&) = delete;

    // PASSIVE_LEVEL, exclusive register/firmware ownership. Callbacks/context and
    // owned mapping stay alive. Supports OTG0..3, disabled progressive RGB/no ODM.
    // Validates all addresses and pre-reads fields before the first write, checks
    // actual master/current-enable and clock/reset/busy state before each write,
    // preserves unrelated bits and verifies every programmed field by readback.
    // No polls, allocations, clocks/resets, enable bits or lock registers written.
    // Does not program VTG/global sync, pixel clocks, DML, planes or link training.
    OtgResult program_disabled(const Timing& timing,
                               const DisabledOtgPrerequisites& prerequisites) noexcept;
    bool quarantined() const noexcept { return quarantined_; }

private:
    RegisterIo io_{};
    Dcn32RegisterSpace space_{};
    uint32_t instance_ = 0;
    bool quarantined_ = false;
};

} // namespace rx7900::display

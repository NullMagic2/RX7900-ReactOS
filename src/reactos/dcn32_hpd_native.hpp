/*
Module: rx7900.reactos.dcn32_hpd_native
Purpose: Bind read-only physical HPD sense to the miniport's owned BAR5 mapping.
Created: 2026-10-22
Architecture: The portable register decode is in src/display/dcn32_hpd;
this adapter supplies native mapped-MMIO and lease checks only.
*/
#pragma once

#include "hardware.hpp"
#include "preos_scanout.hpp"
#include "dcn32_hpd.hpp"

namespace rx7900::reactos {

// The device owner serializes this callback with removal, reset, mode changes
// and register release. It proves the same epoch, physical HPD instance,
// discovery segment2 and exclusive register lease on each call; a BAR alone is not
// sufficient. The context must remain alive until retire().
struct NativeHpdProof final {
    void* context = nullptr;
    bool (*current)(void*, uint64_t epoch, uint8_t verified_pin,
                    uint32_t segment2_dword_base,
                    uint32_t aperture_bytes) noexcept = nullptr;
};

// Device-extension-owned read-only lease. Caller must provide a validated
// PreosDcnRegisters token from this card and a separately proven DCN HPD
// instance, not a raw ATOM HPD ID. No active link or mode is inferred.
class NativeDcn32Hpd final {
public:
    NativeDcn32Hpd() noexcept = default;
    NativeDcn32Hpd(const NativeDcn32Hpd&) = delete;
    NativeDcn32Hpd& operator=(const NativeDcn32Hpd&) = delete;
    NativeDcn32Hpd(NativeDcn32Hpd&&) = delete;
    NativeDcn32Hpd& operator=(NativeDcn32Hpd&&) = delete;

    bool bind(HardwareState& hardware, const display::PreosDcnRegisters& registers,
              uint8_t verified_pin, uint64_t epoch, NativeHpdProof proof) noexcept;
    display::HpdResult sample() noexcept;
    void retire() noexcept;
    bool bound() const noexcept { return hardware_ != nullptr; }
    bool matches(const HardwareState& hardware, uint8_t pin, uint64_t epoch,
                 uint32_t segment2_dword_base, uint32_t aperture_bytes) const noexcept {
        return hardware_ == &hardware && pin_ == pin && epoch_ == epoch &&
               space_.segment2_dword_base == segment2_dword_base &&
               space_.aperture_bytes == aperture_bytes;
    }

private:
    HardwareState* hardware_ = nullptr;
    display::Dcn32RegisterSpace space_{};
    NativeHpdProof proof_{};
    uint64_t epoch_ = 0;
    uint8_t pin_ = 0;
};

} // namespace rx7900::reactos

/*
Module: rx7900.reactos.dcn32_hpd_native
Purpose: Checked VideoPort MMIO callback for a proven DCN3.2 HPD instance.
Created: 2026-10-22
Architecture: Read-only native adapter; no PCI address, MMIO offset or GPU
address comes from a user request.
*/
#include "dcn32_hpd_native.hpp"

namespace rx7900::reactos {
namespace {

bool native_read(void* context, uint32_t offset, uint32_t& value) noexcept {
    return read_register(*static_cast<HardwareState*>(context), offset, value);
}

} // namespace

bool NativeDcn32Hpd::bind(HardwareState& hardware,
                          const display::PreosDcnRegisters& registers,
                          uint8_t verified_pin, uint64_t epoch,
                          NativeHpdProof proof) noexcept {
    if (bound() || !hardware.acquired || hardware.registers == nullptr ||
        !registers.valid() || registers.aperture_bytes() != hardware.register_bytes ||
        verified_pin >= 5 || epoch == 0 || proof.current == nullptr) {
        return false;
    }

    // The validated token is unique DMU instance0 3.2.0. Even bind requires
    // the owner to prove the selected pin and current serialized lease.
    if (!proof.current(proof.context, epoch, verified_pin,
                       registers.segment(2), registers.aperture_bytes())) {
        return false;
    }

    hardware_ = &hardware;
    space_ = {registers.segment(2), hardware.register_bytes, 3, 2, 0, true};
    proof_ = proof;
    epoch_ = epoch;
    pin_ = verified_pin;
    return true;
}

display::HpdResult NativeDcn32Hpd::sample() noexcept {
    if (!bound() || !hardware_->acquired || hardware_->registers == nullptr ||
        hardware_->register_bytes != space_.aperture_bytes ||
        !proof_.current(proof_.context, epoch_, pin_,
                        space_.segment2_dword_base, space_.aperture_bytes)) {
        return display::HpdResult::unavailable;
    }

    const display::RegisterIo io{hardware_, native_read, nullptr};
    const auto result = display::read_dcn32_hpd(io, space_, pin_);
    // A reset or removal between the read and result publication invalidates
    // this sample. The owner serializes release, so this check is a guard, not
    // a substitute for the register lease.
    if (!proof_.current(proof_.context, epoch_, pin_,
                        space_.segment2_dword_base, space_.aperture_bytes)) {
        return display::HpdResult::unavailable;
    }
    return result;
}

void NativeDcn32Hpd::retire() noexcept {
    hardware_ = nullptr;
    space_ = {};
    proof_ = {};
    epoch_ = 0;
    pin_ = 0;
}

} // namespace rx7900::reactos

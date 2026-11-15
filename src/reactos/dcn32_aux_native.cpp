/*
Module: rx7900.reactos.dcn32_aux_native
Purpose: Bounded physical AUX callbacks under exact native route ownership.
Created: 2026-10-22
Architecture: The miniport must establish display ownership before binding.
*/
#include "dcn32_aux_native.hpp"
#include "lifetime.hpp"

namespace rx7900::reactos {

#if defined(_MSC_VER)
static_assert(__is_trivially_destructible(output::Dcn32AuxEngine),
#elif defined(__GNUC__)
static_assert(__has_trivial_destructor(output::Dcn32AuxEngine),
#else
#error Native AUX re-arm requires an audited trivial-destructor compiler trait
#endif
              "A re-armed AUX engine replaces the old one in place");

bool NativeDcn32Aux::bind(HardwareState& hardware,
                           const display::PreosDcnRegisters& registers,
                           NativeDcn32Hpd& hpd, uint8_t engine,
                           uint8_t verified_hpd_pin, uint64_t epoch,
                           NativeAuxProof proof) noexcept {
    const bool armed = !ever_bound_ || (rearm_above_epoch_ != 0 && epoch > rearm_above_epoch_);
    if (!armed || bound() || !hardware.acquired || hardware.registers == nullptr ||
        !registers.valid() || registers.aperture_bytes() != hardware.register_bytes ||
        !hpd.matches(hardware, verified_hpd_pin, epoch, registers.segment(2),
                     registers.aperture_bytes()) ||
        engine >= 5 || verified_hpd_pin >= 5 || epoch == 0 ||
        proof.current == nullptr ||
        !proof.current(proof.context, epoch, engine, verified_hpd_pin,
                       registers.segment(2), registers.aperture_bytes())) {
        return false;
    }

    hardware_ = &hardware;
    hpd_ = &hpd;
    proof_ = proof;
    epoch_ = epoch;
    segment2_ = registers.segment(2);
    aperture_ = registers.aperture_bytes();
    engine_index_ = engine;
    pin_ = verified_hpd_pin;
    const output::Dcn32AuxIo io{this, read_callback, write_callback,
                                 wait_callback, connected_callback};
    const display::Dcn32RegisterSpace space{segment2_, aperture_, 3, 2, 0, true};
    // A re-bind after a proven reset ends the previous engine's lifetime by
    // constructing the new one in the same storage (trivially destructible).
    engine_ = new (DeviceStorage{engine_storage_})
        output::Dcn32AuxEngine(io, space, engine, verified_hpd_pin);
    ever_bound_ = true;
    rearm_above_epoch_ = 0;
    return true;
}

void NativeDcn32Aux::reset_proven(uint64_t old_epoch) noexcept {
    if (!ever_bound_ || old_epoch == 0 || old_epoch != epoch_) {
        return;
    }
    hardware_ = nullptr;
    hpd_ = nullptr;
    proof_ = {};
    engine_ = nullptr;
    epoch_ = 0;
    rearm_above_epoch_ = old_epoch;
}

bool NativeDcn32Aux::current() const noexcept {
    return bound() && hardware_->acquired && hardware_->registers != nullptr &&
           hardware_->register_bytes == aperture_ &&
           proof_.current(proof_.context, epoch_, engine_index_, pin_,
                          segment2_, aperture_);
}

bool NativeDcn32Aux::connected() noexcept {
    return current() && hpd_ != nullptr &&
           hpd_->sample() == display::HpdResult::connected && current();
}

bool NativeDcn32Aux::read_callback(void* context, uint32_t offset,
                                    uint32_t& value) noexcept {
    auto& owner = *static_cast<NativeDcn32Aux*>(context);
    return owner.current() &&
           read_register(*owner.hardware_, offset, value);
}

bool NativeDcn32Aux::write_callback(void* context, uint32_t offset,
                                     uint32_t value) noexcept {
    auto& owner = *static_cast<NativeDcn32Aux*>(context);
    return owner.current() &&
           write_register(*owner.hardware_, offset, value);
}

bool NativeDcn32Aux::wait_callback(void* context, uint32_t microseconds) noexcept {
    auto& owner = *static_cast<NativeDcn32Aux*>(context);
    if (!owner.current() || microseconds > 1000) {
        return false;
    }
    if (microseconds != 0) {
        KeStallExecutionProcessor(microseconds);
    }
    return owner.current();
}

bool NativeDcn32Aux::connected_callback(void* context) noexcept {
    return static_cast<NativeDcn32Aux*>(context)->connected();
}

bool NativeDcn32Aux::transfer_callback(void* context,
                                         const output::AuxRequest& request,
                                         output::AuxReply& reply) noexcept {
    auto& owner = *static_cast<NativeDcn32Aux*>(context);
    reply = {};
    if (!owner.current() || !owner.connected() || owner.engine_ == nullptr) {
        return false;
    }
    const output::Dcn32AuxPrerequisites prerequisites{true, true, true, true};
    const bool success = owner.engine_->transfer(request, reply, prerequisites);
    if (!owner.current()) {
        reply = {};
        return false;
    }
    return success;
}

output::AuxIo NativeDcn32Aux::make_aux_io() noexcept {
    if (!bound()) {
        return {};
    }
    return {this, transfer_callback, wait_callback, connected_callback};
}

bool NativeDcn32Aux::retire() noexcept {
    if (!bound() || quarantined()) {
        return false;
    }
    hardware_ = nullptr;
    hpd_ = nullptr;
    proof_ = {};
    return true;
}

bool NativeDcn32Aux::quarantined() const noexcept {
    return engine_ != nullptr && engine_->quarantined();
}

} // namespace rx7900::reactos

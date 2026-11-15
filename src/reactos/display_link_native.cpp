/*
Module: rx7900.reactos.display_link_native
Purpose: Native MMIO, waits, DMUB VRAM window and HPD/AUX proofs for DisplayLinkOwner.
Created: 2026-10-24
Architecture: Thin NT/VideoPort adapter; every protocol decision stays portable.
*/
#include "display_link_native.hpp"

namespace rx7900::reactos {
namespace {

constexpr ULONG stall_limit_microseconds = 50;
// One DAL/AUX/I2C poll step never needs more than the 100 ms DMUB boot wait.
constexpr uint32_t maximum_wait_microseconds = 100000;

} // namespace

bool NativeDisplayLink::read32(void* context, uint32_t offset, uint32_t& value) noexcept {
    auto& link = *static_cast<NativeDisplayLink*>(context);
    return link.hardware_ != nullptr && read_register(*link.hardware_, offset, value);
}

bool NativeDisplayLink::write32(void* context, uint32_t offset, uint32_t value) noexcept {
    auto& link = *static_cast<NativeDisplayLink*>(context);
    return link.hardware_ != nullptr && write_register(*link.hardware_, offset, value);
}

// PASSIVE_LEVEL only: short polls busy-wait, longer waits yield. Both wait at
// least the request (bounded so a corrupt caller cannot sleep unbounded).
void NativeDisplayLink::delay(void*, uint32_t microseconds) noexcept {
    if (microseconds > maximum_wait_microseconds) {
        microseconds = maximum_wait_microseconds;
    }
    if (microseconds <= stall_limit_microseconds) {
        KeStallExecutionProcessor(microseconds);
        return;
    }
    LARGE_INTEGER interval{};
    interval.QuadPart = -10LL * static_cast<LONGLONG>(microseconds);
    KeDelayExecutionThread(KernelMode, FALSE, &interval);
}

bool NativeDisplayLink::dmub_range(uint32_t offset, uint32_t bytes) const noexcept {
    return dmub_cpu_ != nullptr && offset <= dmub_bytes_ && bytes <= dmub_bytes_ - offset;
}

// The DMUB window is an uncached BAR0 view of the caller-reserved VRAM range;
// offsets are checked against its exact length before any pointer is formed.
bool NativeDisplayLink::dmub_write(void* context, uint32_t offset, ByteView source) noexcept {
    auto& link = *static_cast<NativeDisplayLink*>(context);
    if (source.size() > UINT32_MAX ||
        !link.dmub_range(offset, static_cast<uint32_t>(source.size()))) {
        return false;
    }
    volatile UCHAR* target = link.dmub_cpu_ + offset;
    for (size_t index = 0; index < source.size(); ++index) {
        uint8_t value = 0;
        source.read_u8(index, value);
        target[index] = value;
    }
    return true;
}

bool NativeDisplayLink::dmub_read(void* context, uint32_t offset, uint8_t* data,
                                  uint32_t bytes) noexcept {
    auto& link = *static_cast<NativeDisplayLink*>(context);
    if (data == nullptr || !link.dmub_range(offset, bytes)) {
        return false;
    }
    const volatile UCHAR* source = link.dmub_cpu_ + offset;
    for (uint32_t index = 0; index < bytes; ++index) {
        data[index] = source[index];
    }
    return true;
}

bool NativeDisplayLink::dmub_fill(void* context, uint32_t offset, uint32_t bytes) noexcept {
    auto& link = *static_cast<NativeDisplayLink*>(context);
    if (!link.dmub_range(offset, bytes)) {
        return false;
    }
    volatile UCHAR* target = link.dmub_cpu_ + offset;
    for (uint32_t index = 0; index < bytes; ++index) {
        target[index] = 0;
    }
    return true;
}

// dmub_rb_flush_pending: order CPU stores to the uncached window before the
// MMIO pointer write; a read-back forces posted writes out.
bool NativeDisplayLink::dmub_flush(void* context) noexcept {
    auto& link = *static_cast<NativeDisplayLink*>(context);
    if (link.dmub_cpu_ == nullptr || link.dmub_bytes_ == 0) {
        return false;
    }
    KeMemoryBarrier();
    const volatile UCHAR* last = link.dmub_cpu_ + (link.dmub_bytes_ - 1u);
    (void)*last;
    KeMemoryBarrier();
    return true;
}

bool NativeDisplayLink::hpd_current(void* context, uint64_t epoch, uint8_t pin,
                                    uint32_t segment2, uint32_t aperture) noexcept {
    auto& link = *static_cast<NativeDisplayLink*>(context);
    return link.hardware_ != nullptr && link.hardware_->acquired &&
           link.owner_.hpd_proof(epoch, pin, segment2, aperture);
}

bool NativeDisplayLink::aux_current(void* context, uint64_t epoch, uint8_t engine, uint8_t pin,
                                    uint32_t segment2, uint32_t aperture) noexcept {
    auto& link = *static_cast<NativeDisplayLink*>(context);
    return link.hardware_ != nullptr && link.hardware_->acquired &&
           link.owner_.aux_proof(epoch, engine, pin, segment2, aperture);
}

output::LinkResult NativeDisplayLink::bind(const NativeDisplayLinkInputs& inputs) noexcept {
    if (bound() || inputs.hardware == nullptr || !inputs.hardware->acquired ||
        inputs.hardware->registers == nullptr || inputs.registers == nullptr ||
        !inputs.registers->valid() ||
        inputs.registers->aperture_bytes() != inputs.hardware->register_bytes ||
        inputs.catalog == nullptr || inputs.atom == nullptr || inputs.epoch == 0 ||
        KeGetCurrentIrql() != PASSIVE_LEVEL) {
        return output::LinkResult::invalid_argument;
    }
    const bool wants_dmub = inputs.dmcub_firmware.size() != 0;
    if (wants_dmub && (inputs.aperture == nullptr || inputs.dmub_bytes == 0 ||
                       (inputs.dmub_vram_offset & 0xfffu) != 0 ||
                       (inputs.dmub_mc_address & 0xfffu) != 0)) {
        return output::LinkResult::invalid_argument;
    }

    hardware_ = inputs.hardware;
    registers_ = inputs.registers;
    segment2_ = inputs.registers->segment(2);
    aperture_bytes_ = inputs.hardware->register_bytes;
    epoch_ = inputs.epoch;

    output::DisplayLinkConfig config{};
    config.io = {this, aperture_bytes_, read32, write32, delay};
    config.dcn = {segment2_, aperture_bytes_, 3, 2, 0, true};
    config.dccg_segment = inputs.registers->segment(1);
    config.mp1_segment0 = inputs.mp1_segment0;
    config.xtalin_khz = inputs.xtalin_khz;
    config.boot_dprefclk_khz = inputs.boot_dprefclk_khz;
    // CLK1_CLK_PLL_REQ is outside the validated DMU token: Linux's 4.3 GHz
    // fallback VCO is used until a discovered CLK block owner provides it.
    config.pll_request = 0;
    config.catalog = inputs.catalog;
    config.atom = inputs.atom;
    config.dp_source = output::dcn32_default_dp_source();
    config.epoch = inputs.epoch;

    if (wants_dmub) {
        PUCHAR cpu = nullptr;
        void* token = nullptr;
        if (inputs.aperture->map(inputs.dmub_vram_offset, inputs.dmub_bytes, cpu, token) != NO_ERROR ||
            cpu == nullptr) {
            hardware_ = nullptr;
            registers_ = nullptr;
            return output::LinkResult::not_ready;
        }
        aperture_ = inputs.aperture;
        dmub_cpu_ = cpu;
        dmub_token_ = token;
        dmub_bytes_ = inputs.dmub_bytes;
        config.dmcub_firmware = inputs.dmcub_firmware;
        config.vbios = inputs.vbios;
        config.dmub_memory = {this, inputs.dmub_mc_address, inputs.dmub_bytes,
                              dmub_write, dmub_read, dmub_fill, dmub_flush};
    }

    const output::LinkResult result = owner_.bind(config);
    if (result != output::LinkResult::success && !owner_.quarantined()) {
        // Nothing uncertain: drop the window and the binding.
        if (dmub_token_ != nullptr) {
            aperture_->unmap(dmub_token_);
        }
        dmub_cpu_ = nullptr;
        dmub_token_ = nullptr;
        dmub_bytes_ = 0;
        aperture_ = nullptr;
        hardware_ = nullptr;
        registers_ = nullptr;
    }
    // A quarantined owner keeps hardware_ and the DMUB window: the DMCUB may
    // still write its VRAM, so the mapping and reservation stay owned.
    return result;
}

bool NativeDisplayLink::bind_hpd(const output::RouteLease& lease, NativeDcn32Hpd& hpd) noexcept {
    if (!bound() || registers_ == nullptr || lease.epoch != epoch_ ||
        lease.route >= owner_.routes().count) {
        return false;
    }
    const output::Dcn32ConnectorRoute& route = owner_.routes().routes[lease.route];
    // NativeDcn32Hpd::bind runs the proof itself: the lease must be current
    // and the route validated for exactly this HPD source.
    return hpd.bind(*hardware_, *registers_, route.hpd_source, epoch_, {this, hpd_current});
}

bool NativeDisplayLink::bind_aux(const output::RouteLease& lease, NativeDcn32Hpd& hpd,
                                 NativeDcn32Aux& aux) noexcept {
    if (!bound() || registers_ == nullptr || lease.epoch != epoch_ ||
        lease.route >= owner_.routes().count) {
        return false;
    }
    const output::Dcn32ConnectorRoute& route = owner_.routes().routes[lease.route];
    if (route.signal != output::LinkSignal::display_port ||
        !hpd.matches(*hardware_, route.hpd_source, epoch_, segment2_, aperture_bytes_) ||
        owner_.prepare_aux(lease) != output::LinkResult::success) {
        return false;
    }
    // The AUX adapter re-checks the proof (route in AUX mode, encoder AUX
    // initialized, DMCUB owned or in reset) on every register access.
    return aux.bind(*hardware_, *registers_, hpd, route.aux_engine, route.hpd_source, epoch_,
                    {this, aux_current});
}

bool NativeDisplayLink::retire() noexcept {
    if (!bound()) {
        return true;
    }
    if (owner_.bound() && owner_.unbind() != output::LinkResult::success) {
        return false;
    }
    if (dmub_token_ != nullptr && !aperture_->unmap(dmub_token_)) {
        return false;
    }
    dmub_cpu_ = nullptr;
    dmub_token_ = nullptr;
    dmub_bytes_ = 0;
    aperture_ = nullptr;
    hardware_ = nullptr;
    registers_ = nullptr;
    epoch_ = 0;
    return true;
}

void NativeDisplayLink::reset_proven() noexcept {
    owner_.reset_proven(epoch_);
}

} // namespace rx7900::reactos

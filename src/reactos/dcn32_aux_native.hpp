/*
Module: rx7900.reactos.dcn32_aux_native
Purpose: Bind a proven physical AUX route to the owned VideoPort BAR5 mapping.
Created: 2026-10-22
Architecture: No display mode or clock is inferred from this adapter.
*/
#pragma once

#include "hardware.hpp"
#include "preos_scanout.hpp"
#include "dcn32_hpd_native.hpp"
#include "dcn32_aux.hpp"

namespace rx7900::reactos {

// The owner holds an exclusive register/route lease throughout transfer and
// proves exact ATOM DDC GPIO hardware AUX mode, clock/PHY readiness, DMUB
// arbitration ownership and pin routing for this epoch. A successful callback
// is a fact about the current hardware, not a request to initialize it.
struct NativeAuxProof final {
    void* context = nullptr;
    bool (*current)(void*, uint64_t epoch, uint8_t engine, uint8_t hpd_pin,
                    uint32_t segment2_dword_base,
                    uint32_t aperture_bytes) noexcept = nullptr;
};

// Device-extension-owned and one-shot per proven ASIC reset: bind succeeds once,
// and again only after reset_proven(epoch of that binding) for a strictly later
// epoch (each with a fresh HPD binding and owner proof). Neither this adapter
// nor its portable engine creates a mode, sets clocks or changes DDC GPIO. The
// caller must quiesce callbacks before retire/destruction.
class NativeDcn32Aux final {
public:
    NativeDcn32Aux() noexcept = default;
    NativeDcn32Aux(const NativeDcn32Aux&) = delete;
    NativeDcn32Aux& operator=(const NativeDcn32Aux&) = delete;

    bool bind(HardwareState& hardware,
              const display::PreosDcnRegisters& registers,
              NativeDcn32Hpd& hpd, uint8_t engine, uint8_t verified_hpd_pin,
              uint64_t epoch, NativeAuxProof proof) noexcept;
    output::AuxIo make_aux_io() noexcept;
    bool retire() noexcept;
    // PASSIVE_LEVEL, serialized like bind. The device owner proved an ASIC
    // (mode-1) reset that covers the binding of old_epoch: route, proof and
    // engine are dropped (a quarantined engine too, since the hardware it was
    // unsure about was reset) and one new bind for an epoch above old_epoch is
    // allowed. Linux keeps using its dce_aux engines after the GPU-reset
    // dm_resume (dc_set_power_state D0 runs dcn32_init_hw again); here the
    // engine is created anew because its quarantine is sticky by design. Any
    // other epoch is ignored, so an unrelated proof cannot re-arm the adapter.
    void reset_proven(uint64_t old_epoch) noexcept;
    bool quarantined() const noexcept;
    bool bound() const noexcept { return hardware_ != nullptr; }

private:
    static bool transfer_callback(void*, const output::AuxRequest&,
                                  output::AuxReply&) noexcept;
    static bool wait_callback(void*, uint32_t) noexcept;
    static bool connected_callback(void*) noexcept;
    static bool read_callback(void*, uint32_t, uint32_t&) noexcept;
    static bool write_callback(void*, uint32_t, uint32_t) noexcept;
    bool current() const noexcept;
    bool connected() noexcept;

    HardwareState* hardware_ = nullptr;
    NativeDcn32Hpd* hpd_ = nullptr;
    NativeAuxProof proof_{};
    uint64_t epoch_ = 0;
    uint32_t segment2_ = 0;
    uint32_t aperture_ = 0;
    uint8_t engine_index_ = 0;
    uint8_t pin_ = 0;
    bool ever_bound_ = false;
    // The epoch a proven reset retired; 0 when no re-bind is armed.
    uint64_t rearm_above_epoch_ = 0;
    alignas(output::Dcn32AuxEngine)
        unsigned char engine_storage_[sizeof(output::Dcn32AuxEngine)]{};
    output::Dcn32AuxEngine* engine_ = nullptr;
};

} // namespace rx7900::reactos

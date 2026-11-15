/*
Module: rx7900.reactos.display_link_native
Purpose: Bind the portable DCN3.2 DisplayLinkOwner to the miniport's BAR5/BAR0 mappings.
Created: 2026-10-24
Architecture: Thin VideoPort adapter (RX7900_DISPLAY_LINK builds only). It supplies
MMIO, bounded waits and an optional mapped DMUB VRAM window to
output::DisplayLinkOwner, and binds the existing NativeDcn32Hpd/NativeDcn32Aux
adapters to real leased routes through the owner's proofs. It never reads a
user-supplied address, programs nothing at construction and is used only when
video.cpp (root) calls bind after a completed PhysicalAsicInit=1 bring-up with
the DisplayLinkOwner=1 registry value.
*/
#pragma once

#include "dcn32_aux_native.hpp"
#include "dcn32_hpd_native.hpp"
#include "display_link.hpp"
#include "vram_window.hpp"

namespace rx7900::reactos {

// REG_DWORD in the service key; only 1 enables the owner, never implied by
// PhysicalAsicInit, installation, PCI identity or a parsed ROM.
constexpr wchar_t display_link_registry_value[] = L"DisplayLinkOwner";

struct NativeDisplayLinkInputs final {
    HardwareState* hardware = nullptr;
    // Validated DMU 3.2.0 token (segments 1 and 2) of this card's discovery.
    const display::PreosDcnRegisters* registers = nullptr;
    // Discovered MP1 instance 0 segment 0 (must be 0x16000 for the DAL mailbox).
    uint32_t mp1_segment0 = 0;
    const output::ConnectorCatalog* catalog = nullptr;
    // Bound to the card's VBIOS (AsicBringup owns it); serialized by the caller.
    hardware::AtomInterpreter* atom = nullptr;
    uint32_t xtalin_khz = 0;
    uint32_t boot_dprefclk_khz = 0;
    // Optional DMCUB start: PSP-loaded firmware file, VBIOS bytes and a VRAM
    // range the caller reserved (offset/MC address 4 KiB aligned).
    ByteView dmcub_firmware{nullptr, 0};
    ByteView vbios{nullptr, 0};
    VramAperture* aperture = nullptr;
    uint64_t dmub_vram_offset = 0;
    uint64_t dmub_mc_address = 0;
    uint32_t dmub_bytes = 0;
    uint64_t epoch = 0;
};

// Device-extension-owned, PASSIVE_LEVEL, serialized by the device lock with
// reset, power, removal and mode IOCTLs. Construct once (trivially
// destructible); retire before HardwareState/VramAperture release.
class NativeDisplayLink final {
public:
    NativeDisplayLink() noexcept = default;
    NativeDisplayLink(const NativeDisplayLink&) = delete;
    NativeDisplayLink& operator=(const NativeDisplayLink&) = delete;

    output::LinkResult bind(const NativeDisplayLinkInputs& inputs) noexcept;
    output::DisplayLinkOwner& owner() noexcept { return owner_; }
    bool bound() const noexcept { return hardware_ != nullptr; }

    // NativeDcn32Hpd over the leased route's HPD source with the owner's proof.
    bool bind_hpd(const output::RouteLease& lease, NativeDcn32Hpd& hpd) noexcept;
    // NativeDcn32Aux over the leased DP route; prepares the DDC pad in AUX mode
    // first. aux is permanently one-shot (one per route and device lifetime).
    bool bind_aux(const output::RouteLease& lease, NativeDcn32Hpd& hpd,
                  NativeDcn32Aux& aux) noexcept;

    // Unbinds the owner and unmaps the DMUB window when nothing is uncertain;
    // returns false (keeping every mapping) otherwise.
    bool retire() noexcept;
    // The miniport proved an ASIC reset of the bound epoch.
    void reset_proven() noexcept;

private:
    static bool read32(void*, uint32_t, uint32_t&) noexcept;
    static bool write32(void*, uint32_t, uint32_t) noexcept;
    static void delay(void*, uint32_t) noexcept;
    static bool dmub_write(void*, uint32_t, ByteView) noexcept;
    static bool dmub_read(void*, uint32_t, uint8_t*, uint32_t) noexcept;
    static bool dmub_fill(void*, uint32_t, uint32_t) noexcept;
    static bool dmub_flush(void*) noexcept;
    static bool hpd_current(void*, uint64_t, uint8_t, uint32_t, uint32_t) noexcept;
    static bool aux_current(void*, uint64_t, uint8_t, uint8_t, uint32_t, uint32_t) noexcept;
    bool dmub_range(uint32_t offset, uint32_t bytes) const noexcept;

    output::DisplayLinkOwner owner_{};
    HardwareState* hardware_ = nullptr;
    const display::PreosDcnRegisters* registers_ = nullptr;
    VramAperture* aperture_ = nullptr;
    PUCHAR dmub_cpu_ = nullptr;
    void* dmub_token_ = nullptr;
    uint32_t dmub_bytes_ = 0;
    uint32_t segment2_ = 0;
    uint32_t aperture_bytes_ = 0;
    uint64_t epoch_ = 0;
};

} // namespace rx7900::reactos

/*
Module: rx7900.reactos.vbe_modeset
Purpose: After a legacy (CSM) boot, ask the card's VBIOS for a VBE linear 32 bpp
mode so the pre-OS scanout adoption finds a linear DCN3.2 surface.
Created: 2026-10-22
Architecture: Native binding of the ReactOS VideoPort Int10 interface
(VideoPortQueryServices, VideoPortServicesInt10) to the portable VBE sequence in
src/display/vbe, plus a read-only boot-firmware probe of HARDWARE\DESCRIPTION.
The decision to run it belongs to the miniport (VbeModeSet=1 and
PhysicalAsicInit=1); the scanout itself is captured by preos_scanout_native.
*/
#pragma once

#include "preos_scanout_native.hpp"
#include "vbe.hpp"

namespace rx7900::reactos {

// How freeldr started the system, from the ARC tree the kernel copies into
// HKLM\HARDWARE\DESCRIPTION\System: the UEFI loader creates the
// MultifunctionAdapter "UEFI Internal" (freeldr arch/uefi/uefihw.c), the PC BIOS
// loader the MultifunctionAdapter "ISA" (arch/i386/pc/machpc.c).
enum class BootFirmware : uint32_t {
    unknown,
    bios,
    uefi
};

// Reported numerically by the miniport; new values are appended.
enum class VbeModeset : uint32_t {
    mode_set,
    // The capture did not refuse for an active legacy VGA engine: the current
    // scanout is linear already (GOP/VBE) or refused for another reason.
    not_legacy_vga,
    // The loader was not the PC BIOS one; INT 10h has no VBIOS behind it.
    firmware_not_bios,
    invalid_state,
    // VideoPortQueryServices(VideoPortServicesInt10) failed or was incomplete.
    interface_unavailable,
    // Int10AllocateBuffer returned no usable real-mode scratch buffer.
    buffer_unavailable,
    // The VBE sequence refused; report.status says why.
    refused
};

// Device-owned (inside the zeroed device extension, about 5 KiB, trivially
// destructible); serialized by HwInitialize at PASSIVE_LEVEL.
struct VbeModesetNative final {
    display::VbeWorkspace workspace{};
    display::VbeReport report{};
    BootFirmware firmware = BootFirmware::unknown;
    VbeModeset result = VbeModeset::not_legacy_vga;
};

// PASSIVE_LEVEL. Reads Identifier values of at most sixteen
// HARDWARE\DESCRIPTION\System\MultifunctionAdapter\N keys; writes nothing.
BootFirmware detect_boot_firmware() noexcept;

// HwInitialize, PASSIVE_LEVEL, only from the VideoPort open that runs it in the
// CSRSS context (videoprt dispatch.c initializes the V86 Int10 address space
// immediately before calling HwInitialize). Requires the preceding
// capture_preos_scanout to have refused with PreosStatus::legacy_vga, a BIOS
// boot and an acquired BAR0 range. Then runs display::set_vbe_linear_mode
// through the Int10 interface with one real-mode scratch buffer. No GPU
// register is touched here; the VBIOS programs the display engine. On mode_set
// the caller captures the scanout again. state is reset first.
VbeModeset set_vbe_linear_mode(const HardwareState& hardware, PVOID extension, const PreosScanoutNative& preos,
                               VbeModesetNative& state) noexcept;

} // namespace rx7900::reactos

/*
Module: rx7900.reactos.preos_scanout_native
Purpose: Adopt the running pre-OS (VBIOS/GOP) scanout as the physical miniport's single mode.
Created: 2026-10-22
Architecture: Native resource adapter beside AsicBringup. Takes read-only DCN
snapshots through the BAR5 RegisterIo, delegates every decision to
src/display/preos_scanout, and serves the VideoPort mode/mapping IOCTLs for the
adopted surface through BAR0. It never programs the display engine.
*/
#pragma once

#include "asic_bringup.hpp"
#include "preos_scanout.hpp"

namespace rx7900::reactos {

// Reported numerically by the miniport; new values are appended.
enum class PreosAdoption : uint32_t {
    adopted,
    // Pre-initialization snapshot accepted; finalize_* still has to run.
    captured,
    not_captured,
    discovery_unavailable,
    vram_location_unavailable,
    // decode_preos_scanout refused; PreosScanoutNative::report.status says why.
    decode_refused,
    // The Linux reservation map could not be built from ROM/DCN/VRAM inputs.
    reservations_unavailable,
    unreserved,
    // The ASIC sequencer ran and stopped part way: display state is unknown.
    bringup_incomplete,
    // The post-initialization snapshot or VRAM placement differs from the capture.
    invalidated,
    // The unreserved tail could not be claimed in the VRAM allocator.
    tail_claim_failed
};

// Device-owned (inside the zeroed device extension, about 11 KiB, trivially
// destructible). Serialized by the miniport: HwInitialize and the synchronous
// HwStartIO IOCTLs at PASSIVE_LEVEL. HardwareState and the extension outlive it.
struct PreosScanoutNative final {
    // Own discovery copy: the capture runs before run_asic_bringup reads its own.
    hardware::DiscoveryBinary discovery{};
    display::PreosDcnRegisters registers{};
    display::PreosScanoutRegisters snapshot{};
    display::PreosReport report{};
    display::AdoptedScanout scanout{};
    display::PreosReservationCheck coverage{};
    hardware::VramLocation vram{};
    hardware::VramReservationMap reservations{};
    uint64_t visible_bytes = 0;
    PreosAdoption result = PreosAdoption::not_captured;
    bool captured = false;
    bool adopted = false;
    bool mode_set = false;
    // The single VideoPortMapMemory view handed to the display driver; owned by
    // the caller's process until IOCTL_VIDEO_UNMAP_VIDEO_MEMORY returns it.
    PVOID mapping = nullptr;
};

// HwInitialize, PASSIVE_LEVEL, after acquire and acquire_vbios and BEFORE
// run_asic_bringup or any other register/display owner. rom is the validated
// ATOM image. Reads IP discovery through MM_INDEX/MM_DATA (index writes only, as
// read_vram_discovery documents), the MMHUB FB location, the DCN snapshot and
// gmc_v11_0_get_vbios_fb_size; builds the Linux reservation map; measures the
// refresh from OTG_STATUS_FRAME_COUNT when the pixel clock is not derivable
// (about 0.3 s of polling and one yield). No display register is written.
// Returns captured or a refusal; output is reset first.
PreosAdoption capture_preos_scanout(HardwareState& hardware, ByteView rom,
                                    PreosScanoutNative& preos) noexcept;

// PhysicalAsicInit not requested: nothing reprograms MMHUB, VRAM or DCN, and no
// VRAM allocator exists to hand out an unreserved tail. Re-reads the snapshot
// once and requires the same scanout. Returns adopted or a refusal.
PreosAdoption finalize_preos_without_bringup(HardwareState& hardware, PreosScanoutNative& preos) noexcept;

// Directly after run_asic_bringup in the same HwInitialize, PASSIVE_LEVEL, before
// any other VRAM allocation is made. completed: re-reads the MMHUB location and the
// DCN snapshot, requires both unchanged, checks coverage against the map the
// allocator was built from and claims a tail with VramAllocator::reserve_range.
// Results where AsicInit never ran are handled the same way; sequence_failed and
// sequence_quarantined refuse. Returns adopted or a refusal.
PreosAdoption finalize_preos_after_bringup(HardwareState& hardware, AsicBringup& bringup,
                                           BringupResult result, PreosScanoutNative& preos) noexcept;

// IOCTL helpers. All require preos.adopted (ERROR_DEV_NOT_EXIST otherwise),
// validate buffer lengths before writing, return only initialized bytes and
// retain no caller buffer. Mode index 0 is the only mode.
VP_STATUS preos_mode_information(const PreosScanoutNative& preos, VIDEO_MODE_INFORMATION& output) noexcept;
VP_STATUS preos_query_num_modes(const PreosScanoutNative& preos, VIDEO_REQUEST_PACKET& request) noexcept;
VP_STATUS preos_query_modes(const PreosScanoutNative& preos, VIDEO_REQUEST_PACKET& request) noexcept;
VP_STATUS preos_query_current_mode(const PreosScanoutNative& preos, VIDEO_REQUEST_PACKET& request) noexcept;
// PASSIVE_LEVEL. Accepts only mode 0 and programs nothing. Without
// VIDEO_MODE_NO_ZERO_MEMORY the adopted surface (only those bytes) is cleared
// through short-lived uncached BAR0 views.
VP_STATUS preos_set_current_mode(PreosScanoutNative& preos, const HardwareState& hardware,
                                 PVOID extension, VIDEO_REQUEST_PACKET& request) noexcept;
// PASSIVE_LEVEL, after SET_CURRENT_MODE. VideoPortMapMemory of exactly the
// adopted surface pages (BAR0 base + vram_offset), write-combining hint, into
// the requesting context. One mapping at a time.
VP_STATUS preos_map_video_memory(PreosScanoutNative& preos, const HardwareState& hardware,
                                 PVOID extension, VIDEO_REQUEST_PACKET& request) noexcept;
// Releases the mapping returned by preos_map_video_memory (address must match).
VP_STATUS preos_unmap_video_memory(PreosScanoutNative& preos, PVOID extension,
                                   VIDEO_REQUEST_PACKET& request) noexcept;

} // namespace rx7900::reactos

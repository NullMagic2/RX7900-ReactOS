/*
Module: rx7900.reactos.radeon_cursor_native
Purpose: Serve the VideoPort pointer IOCTLs with the DCN 3.2 hardware cursor of the adopted pre-OS pipe.
Created: 2026-10-22
Architecture: Thin native adapter beside PreosScanoutNative and AsicBringup. The
register protocol and image encoding live in src/display/dcn32_cursor, the
IOCTL wire contract in radeon_cursor_policy.hpp. This module owns the cursor
image VRAM allocation (whole-VRAM allocator, CPU-visible pool), its uncached
BAR0 upload and HDP flush, the opt-in, and the power-transition ordering. No
IOCTL carries an address: callers supply only shapes and coordinates.
*/
#pragma once

#include "asic_bringup.hpp"
#include "preos_scanout_native.hpp"
#include "radeon_cursor_policy.hpp"
#include "dcn32_cursor.hpp"

namespace rx7900::reactos {

// Reported numerically by the miniport; new values are appended.
enum class CursorStart : uint32_t {
    started,
    not_attempted,
    // The pre-OS scanout was not adopted, or the ASIC bring-up did not complete.
    scanout_not_adopted,
    asic_not_running,
    discovery_unavailable,
    // ATOM dce_info crystal frequency, DCHUB global timer or the adopted pixel
    // clock is unknown: CURSOR_DST_X_OFFSET cannot be computed.
    clock_unavailable,
    vram_unavailable,
    protocol_refused,
    // A register write failed: cursor state is uncertain and the hardware
    // cursor stays off for the device lifetime.
    hardware_uncertain,
    // After resume the DCN scanout no longer matches the adopted one.
    invalidated
};

// Two 16 KiB image slots: an update writes the slot DCN is not scanning.
constexpr uint64_t cursor_slot_bytes = display::dcn32_cursor_image_bytes;
constexpr uint64_t cursor_allocation_bytes = 2 * cursor_slot_bytes;

// Device-owned (about 33 KiB inside the zeroed device extension, trivially
// destructible). Serialized by the miniport: HwInitialize, the synchronous
// HwStartIO IOCTLs and HwSetPowerState, all at PASSIVE_LEVEL. The VRAM
// allocation is kept for the device lifetime: the display engine may fetch it
// until a confirmed disable, and this miniport has no unload path.
struct CursorNative final {
    display::Dcn32CursorRegisters registers{};
    display::Dcn32Cursor cursor{};
    // The image DCN scans (kept for resume) and the one being built.
    display::CursorImage image{};
    display::CursorImage staging{};
    hardware::VramAllocationHandle allocation{};
    uint64_t mc_address = 0;
    uint32_t slot = 0;
    uint32_t refclk_khz = 0;
    CursorStart result = CursorStart::not_attempted;
    bool started = false;
    bool shape_defined = false;
    // Requested by the display driver (SET_POINTER_ATTR Enable, ENABLE/DISABLE).
    bool visible = false;
    // Between cursor_suspend and cursor_resume: requests only update state.
    bool suspended = false;
    // Between IOCTL_VIDEO_RESET_DEVICE and the next SET_CURRENT_MODE: hidden.
    bool blanked = false;
    int16_t column = 0;
    int16_t row = 0;
    // Device-lifetime owners recorded by start_hardware_cursor.
    HardwareState* hardware = nullptr;
    AsicBringup* bringup = nullptr;
    PVOID extension = nullptr;
};

// HardwareCursor=1 in the service key, PASSIVE_LEVEL. Never implied by
// PhysicalAsicInit or Radeon2D.
bool hardware_cursor_requested(PVOID extension) noexcept;

// HwInitialize, PASSIVE_LEVEL, after finalize_preos_* adopted the scanout and
// run_asic_bringup completed (asic_running). Validates the cursor registers of
// the adopted pipe, reads the DCHUB reference clock (ATOM dce_info and
// DCHUBBUB_GLOBAL_TIMER_CNTL), requires a known pixel clock and allocates the
// two image slots (page aligned, contiguous, CPU visible). Writes no display
// register: the cursor stays as the firmware left it (off) until a shape.
CursorStart start_hardware_cursor(CursorNative& cursor, HardwareState& hardware, AsicBringup& bringup,
                                  const PreosScanoutNative& preos, ByteView rom, PVOID extension) noexcept;

bool is_pointer_ioctl(ULONG code) noexcept;

// HwStartIO for the six pointer IOCTLs (IOCTL_VIDEO_QUERY_POINTER_CAPABILITIES,
// SET_POINTER_ATTR, SET_POINTER_POSITION, ENABLE_POINTER, DISABLE_POINTER,
// QUERY_POINTER_POSITION). ERROR_INVALID_FUNCTION unless started (the display
// driver then uses the engine pointer). Inputs are decoded completely before
// any state change; outputs are written only after their length is checked;
// no caller buffer is retained. A shape that cannot be expressed returns
// ERROR_INVALID_PARAMETER with the previous cursor unchanged.
VP_STATUS cursor_start_io(CursorNative& cursor, VIDEO_REQUEST_PACKET& request) noexcept;

// IOCTL_VIDEO_RESET_DEVICE (console/mode hand-off, DrvAssertMode FALSE): hides
// the hardware cursor but keeps the requested shape, position and visibility.
// A later successful IOCTL_VIDEO_SET_CURRENT_MODE calls cursor_mode_set, which
// shows it again as requested. PASSIVE_LEVEL.
void cursor_reset_device(CursorNative& cursor) noexcept;
void cursor_mode_set(CursorNative& cursor) noexcept;

// HwSetPowerState D0 -> D3, before the ASIC suspends: the cursor is disabled
// under the cursor lock while DCN still runs, then programmed state is
// forgotten. Requests made while suspended only update state.
void cursor_suspend(CursorNative& cursor) noexcept;

// HwSetPowerState D3 -> D0 after resume_asic completed. Re-reads the DCN
// scanout; when it still matches the adopted one the current image is uploaded
// again (VRAM contents are not assumed to survive) and attributes and position
// are reprogrammed. Otherwise the hardware cursor stays off (invalidated).
void cursor_resume(CursorNative& cursor, const PreosScanoutNative& preos) noexcept;

} // namespace rx7900::reactos

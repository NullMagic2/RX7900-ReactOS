/*
Module: rx7900.reactos.radeon_cursor_native
Purpose: Native start, pointer IOCTLs and power ordering of the DCN 3.2 hardware cursor.
Created: 2026-10-22
Architecture: VideoPort glue only; decisions are in src/display/dcn32_cursor and
radeon_cursor_policy.hpp. Register access goes through the checked BAR5
helpers, the image through VramAperture views of a VramAllocator allocation.
*/
#include "radeon_cursor_native.hpp"

#include "video_helpers.hpp"

namespace rx7900::reactos {
namespace {

namespace policy = cursor_policy;
using display::CursorStatus;

constexpr ULONG stall_limit_microseconds = 50;
// VIDEO_POINTER_CAPABILITIES.HWPtrBitmapStart/End: no offscreen pointer memory
// is exposed to the display driver.
constexpr ULONG no_pointer_bitmap = 0xffffffffu;

static_assert(static_cast<uint32_t>(policy::PointerFormat::monochrome) ==
                  static_cast<uint32_t>(display::CursorSourceFormat::monochrome) &&
              static_cast<uint32_t>(policy::PointerFormat::masked_color) ==
                  static_cast<uint32_t>(display::CursorSourceFormat::masked_color) &&
              static_cast<uint32_t>(policy::PointerFormat::premultiplied_alpha) ==
                  static_cast<uint32_t>(display::CursorSourceFormat::premultiplied_alpha) &&
              static_cast<uint32_t>(policy::PointerFormat::straight_alpha) ==
                  static_cast<uint32_t>(display::CursorSourceFormat::straight_alpha),
              "IOCTL pointer formats map one to one onto DCN cursor sources");
static_assert(policy::max_dimension == display::dcn32_cursor_max_dimension, "One cursor size limit");
static_assert(sizeof(VIDEO_POINTER_POSITION) == policy::position_bytes, "VIDEO_POINTER_POSITION layout");
static_assert(sizeof(VIDEO_POINTER_CAPABILITIES) == policy::capabilities_bytes,
              "VIDEO_POINTER_CAPABILITIES layout");
static_assert(FIELD_OFFSET(VIDEO_POINTER_ATTRIBUTES, Pixels) == policy::attributes_header_bytes,
              "VIDEO_POINTER_ATTRIBUTES layout");

bool mmio_read(void* context, uint32_t offset, uint32_t& value) noexcept {
    return read_register(*static_cast<HardwareState*>(context), offset, value);
}

bool mmio_write(void* context, uint32_t offset, uint32_t value) noexcept {
    return write_register(*static_cast<HardwareState*>(context), offset, value);
}

// PASSIVE_LEVEL waits: short stalls spin, longer ones (the cursor-lock VUPDATE
// keepout is at most a few lines) yield.
void wait(void*, uint32_t microseconds) noexcept {
    if (microseconds <= stall_limit_microseconds) {
        KeStallExecutionProcessor(microseconds);
        return;
    }

    LARGE_INTEGER interval{};
    interval.QuadPart = -10LL * static_cast<LONGLONG>(microseconds);
    KeDelayExecutionThread(KernelMode, FALSE, &interval);
}

hardware::RegisterIo register_io(HardwareState& hardware) noexcept {
    return {&hardware, hardware.register_bytes, mmio_read, mmio_write, wait};
}

VP_STATUS NTAPI read_dword(PVOID, PVOID context, PWSTR, PVOID data, ULONG length) {
    if (context == nullptr || data == nullptr || length != sizeof(ULONG)) {
        return ERROR_INVALID_PARAMETER;
    }

    VideoPortMoveMemory(context, data, sizeof(ULONG));
    return NO_ERROR;
}

CursorStart refuse(CursorNative& cursor, CursorStart result) noexcept {
    cursor.started = false;
    cursor.result = result;
    return result;
}

// A register failure leaves the cursor in an unknown state: stop serving it.
VP_STATUS uncertain(CursorNative& cursor) noexcept {
    refuse(cursor, CursorStart::hardware_uncertain);
    return ERROR_DEVICE_REINITIALIZATION_NEEDED;
}

// Copies image into slot of the allocation through a short-lived uncached BAR0
// view, then flushes HDP so the display engine fetches the new bytes
// (hdp_v6_0_flush_hdp; CPU stores through BAR0 are otherwise not visible).
bool upload(CursorNative& cursor, const display::CursorImage& image, uint32_t slot) noexcept {
    PUCHAR view = nullptr;
    void* token = nullptr;
    AsicBringup& bringup = *cursor.bringup;

    if (slot > 1 ||
        bringup.aperture.map_allocation(bringup.vram_allocator, cursor.allocation, view, token) != NO_ERROR) {
        return false;
    }

    // view covers the whole contiguous allocation (cursor_allocation_bytes)
    // until the unmap below; the image is exactly one slot of ULONG words.
    static_assert(sizeof(image.pixels) == cursor_slot_bytes, "One image per slot");
    PUCHAR destination = view + slot * cursor_slot_bytes;

    for (uint32_t index = 0; index < display::dcn32_cursor_pixels; ++index) {
        VideoPortWriteRegisterUlong(reinterpret_cast<PULONG>(destination + index * sizeof(uint32_t)),
                                    image.pixels[index]);
    }

    const bool unmapped = bringup.aperture.unmap(token);
    // AsicInit keeps the Nbio whose HDP remap program_common confirmed; the
    // copy shares its RegisterIo (this device's BAR5 owner).
    hardware::Nbio nbio = bringup.asic.nbio();
    return unmapped && nbio.flush_hdp();
}

display::CursorRequest request_of(const CursorNative& cursor) noexcept {
    return {cursor.visible && cursor.shape_defined && !cursor.blanked, cursor.column, cursor.row};
}

// The requested position and visibility, unless suspended.
VP_STATUS apply_position(CursorNative& cursor) noexcept {
    if (cursor.suspended || !cursor.shape_defined) {
        return NO_ERROR;
    }

    return cursor.cursor.set_position(request_of(cursor)) == CursorStatus::ready ? NO_ERROR : uncertain(cursor);
}

VP_STATUS set_attributes(CursorNative& cursor, VIDEO_REQUEST_PACKET& request) noexcept {
    const ByteView input(static_cast<const uint8_t*>(request.InputBuffer), request.InputBufferLength);
    policy::PointerShape shape{};

    if (policy::decode_attributes(input, shape) != policy::DecodeStatus::decoded) {
        return ERROR_INVALID_PARAMETER;
    }

    display::CursorSource source{};
    source.format = static_cast<display::CursorSourceFormat>(shape.format);
    source.width = shape.width;
    source.height = shape.height;
    source.mask = shape.mask;
    source.mask_stride = shape.mask_stride;
    source.color = shape.color;
    source.color_stride = shape.color_stride;

    // The input is fully consumed here; nothing borrows it afterwards.
    if (display::build_cursor_image(source, cursor.staging) != display::CursorImageStatus::built) {
        return ERROR_INVALID_PARAMETER;
    }

    if (cursor.suspended) {
        cursor.image = cursor.staging;
    } else {
        // Write the slot the display engine is not scanning, then switch to it.
        const uint32_t slot = cursor.shape_defined ? 1u - cursor.slot : 0u;

        if (!upload(cursor, cursor.staging, slot) ||
            cursor.cursor.set_attributes({cursor.mc_address + slot * cursor_slot_bytes, cursor.staging.mode}) !=
                CursorStatus::ready) {
            return uncertain(cursor);
        }

        cursor.image = cursor.staging;
        cursor.slot = slot;
    }

    cursor.shape_defined = true;
    cursor.visible = shape.enable;
    cursor.column = shape.column;
    cursor.row = shape.row;
    return apply_position(cursor);
}

VP_STATUS set_position(CursorNative& cursor, VIDEO_REQUEST_PACKET& request) noexcept {
    const ByteView input(static_cast<const uint8_t*>(request.InputBuffer), request.InputBufferLength);
    int16_t column = 0;
    int16_t row = 0;

    if (!policy::decode_position(input, column, row)) {
        return ERROR_INVALID_PARAMETER;
    }

    if (!cursor.shape_defined) {
        return ERROR_INVALID_FUNCTION;
    }

    cursor.column = column;
    cursor.row = row;
    return apply_position(cursor);
}

VP_STATUS query_capabilities(VIDEO_REQUEST_PACKET& request) noexcept {
    VIDEO_POINTER_CAPABILITIES capabilities{};
    capabilities.Flags = policy::flag_mono | policy::flag_color | policy::flag_premultiplied_alpha |
                         policy::flag_straight_alpha;
    capabilities.MaxWidth = policy::max_dimension;
    capabilities.MaxHeight = policy::max_dimension;
    capabilities.HWPtrBitmapStart = no_pointer_bitmap;
    capabilities.HWPtrBitmapEnd = no_pointer_bitmap;
    return return_video_value(request, capabilities);
}

VP_STATUS query_position(const CursorNative& cursor, VIDEO_REQUEST_PACKET& request) noexcept {
    if (!cursor.shape_defined) {
        return ERROR_INVALID_FUNCTION;
    }

    VIDEO_POINTER_POSITION position{};
    position.Column = cursor.column;
    position.Row = cursor.row;
    return return_video_value(request, position);
}

} // namespace

bool hardware_cursor_requested(PVOID extension) noexcept {
    ULONG value = 0;
    wchar_t name[] = L"HardwareCursor";
    return VideoPortGetRegistryParameters(extension, name, FALSE, read_dword, &value) == NO_ERROR && value == 1;
}

CursorStart start_hardware_cursor(CursorNative& cursor, HardwareState& hardware, AsicBringup& bringup,
                                  const PreosScanoutNative& preos, ByteView rom, PVOID extension) noexcept {
    if (cursor.started || cursor.allocation.slot != UINT32_MAX || KeGetCurrentIrql() != PASSIVE_LEVEL) {
        return cursor.result;
    }

    cursor.result = CursorStart::not_attempted;

    if (!preos.adopted) {
        return refuse(cursor, CursorStart::scanout_not_adopted);
    }

    if (!hardware.acquired || !bringup.vram_allocator.initialized() || bringup.aperture.visible_bytes() == 0) {
        return refuse(cursor, CursorStart::asic_not_running);
    }

    const hardware::RegisterIo io = register_io(hardware);

    if (display::validate_dcn32_cursor_discovery(preos.discovery.bytes(), hardware.register_bytes, preos.scanout,
                                                 cursor.registers) != hardware::DiscoveryStatus::validated) {
        return refuse(cursor, CursorStart::discovery_unavailable);
    }

    uint32_t crystal_khz = 0;
    const display::Timing& timing = preos.scanout.timing;

    if (timing.pixel_clock_khz == 0 || !display::read_dce_refclk_khz(rom, crystal_khz) ||
        !display::read_dchub_refclk_khz(io, cursor.registers, crystal_khz, cursor.refclk_khz)) {
        return refuse(cursor, CursorStart::clock_unavailable);
    }

    // amdgpu_bo_create_kernel-style pinned contiguous BO with CPU access: the
    // allocator places it in visible VRAM (bottom-up), page aligned.
    const hardware::VramRequest request{cursor_allocation_bytes, display::dcn32_cursor_alignment,
                                        hardware::vram_kernel_bo_flags(true)};
    hardware::VramAllocationInfo info{};

    if (bringup.vram_allocator.allocate(request, cursor.allocation) != hardware::VramStatus::success) {
        cursor.allocation = {};
        return refuse(cursor, CursorStart::vram_unavailable);
    }

    if (bringup.vram_allocator.describe(cursor.allocation, info) != hardware::VramStatus::success ||
        !info.contiguous || info.no_cpu_access || info.visible_bytes < cursor_allocation_bytes ||
        (info.first_byte % display::dcn32_cursor_alignment) != 0) {
        bringup.vram_allocator.release(cursor.allocation);
        cursor.allocation = {};
        return refuse(cursor, CursorStart::vram_unavailable);
    }

    const display::CursorStream stream{timing.width, timing.height, timing.pixel_clock_khz, cursor.refclk_khz};

    if (cursor.cursor.initialize(cursor.registers, io, timing, stream) != CursorStatus::ready) {
        bringup.vram_allocator.release(cursor.allocation);
        cursor.allocation = {};
        return refuse(cursor, CursorStart::protocol_refused);
    }

    // MC = VramLocation.mc_start + VRAM offset (the same rule as the scanout).
    cursor.mc_address = preos.vram.mc_start + info.first_byte;
    cursor.hardware = &hardware;
    cursor.bringup = &bringup;
    cursor.extension = extension;
    cursor.slot = 0;
    cursor.shape_defined = false;
    cursor.visible = false;
    cursor.suspended = false;
    cursor.blanked = false;
    cursor.started = true;
    cursor.result = CursorStart::started;
    return cursor.result;
}

bool is_pointer_ioctl(ULONG code) noexcept {
    switch (code) {
    case IOCTL_VIDEO_QUERY_POINTER_CAPABILITIES:
    case IOCTL_VIDEO_SET_POINTER_ATTR:
    case IOCTL_VIDEO_SET_POINTER_POSITION:
    case IOCTL_VIDEO_ENABLE_POINTER:
    case IOCTL_VIDEO_DISABLE_POINTER:
    case IOCTL_VIDEO_QUERY_POINTER_POSITION:
    case IOCTL_VIDEO_QUERY_POINTER_ATTR:
        return true;
    default:
        return false;
    }
}

VP_STATUS cursor_start_io(CursorNative& cursor, VIDEO_REQUEST_PACKET& request) noexcept {
    if (!cursor.started || VideoPortGetCurrentIrql() != PASSIVE_LEVEL) {
        return ERROR_INVALID_FUNCTION;
    }

    switch (request.IoControlCode) {
    case IOCTL_VIDEO_QUERY_POINTER_CAPABILITIES:
        return query_capabilities(request);
    case IOCTL_VIDEO_SET_POINTER_ATTR:
        return set_attributes(cursor, request);
    case IOCTL_VIDEO_SET_POINTER_POSITION:
        return set_position(cursor, request);
    case IOCTL_VIDEO_ENABLE_POINTER:
        if (!cursor.shape_defined) {
            return ERROR_INVALID_FUNCTION;
        }

        cursor.visible = true;
        return apply_position(cursor);
    case IOCTL_VIDEO_DISABLE_POINTER:
        cursor.visible = false;
        return apply_position(cursor);
    case IOCTL_VIDEO_QUERY_POINTER_POSITION:
        return query_position(cursor, request);
    default:
        // QUERY_POINTER_ATTR: the canonical shape is not returned.
        return ERROR_INVALID_FUNCTION;
    }
}

void cursor_reset_device(CursorNative& cursor) noexcept {
    if (!cursor.started) {
        return;
    }

    cursor.blanked = true;
    apply_position(cursor);
}

void cursor_mode_set(CursorNative& cursor) noexcept {
    if (!cursor.started || !cursor.blanked) {
        return;
    }

    cursor.blanked = false;
    apply_position(cursor);
}

void cursor_suspend(CursorNative& cursor) noexcept {
    if (!cursor.started || cursor.suspended) {
        return;
    }

    // Hide while DCN still runs; a failure only stops the hardware cursor.
    if (cursor.cursor.hide() != CursorStatus::ready) {
        uncertain(cursor);
    }

    cursor.cursor.forget();
    cursor.suspended = true;
}

void cursor_resume(CursorNative& cursor, const PreosScanoutNative& preos) noexcept {
    if (!cursor.started || !cursor.suspended || cursor.hardware == nullptr) {
        return;
    }

    const hardware::RegisterIo io = register_io(*cursor.hardware);
    display::PreosScanoutRegisters snapshot{};
    display::AdoptedScanout scanout{};
    display::PreosReport report{};

    if (!preos.adopted || !display::read_preos_scanout(io, preos.registers, snapshot) ||
        display::decode_preos_scanout(snapshot, preos.vram, preos.visible_bytes, scanout, report) !=
            display::PreosStatus::adopted ||
        !display::same_preos_scanout(preos.scanout, scanout)) {
        refuse(cursor, CursorStart::invalidated);
        return;
    }

    cursor.suspended = false;

    if (!cursor.shape_defined) {
        return;
    }

    if (!upload(cursor, cursor.image, cursor.slot) ||
        cursor.cursor.set_attributes({cursor.mc_address + cursor.slot * cursor_slot_bytes, cursor.image.mode}) !=
            CursorStatus::ready) {
        uncertain(cursor);
        return;
    }

    apply_position(cursor);
}

} // namespace rx7900::reactos

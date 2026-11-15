/*
Module: rx7900.reactos.preos_scanout_native
Purpose: Native capture, post-initialization confirmation and IOCTL service of the adopted pre-OS scanout.
Created: 2026-10-22
Architecture: Thin VideoPort adapter; snapshot decoding and adoption rules live in
src/display/preos_scanout, the Linux reservation map in src/hardware. Register
access goes through the checked BAR5 helpers; VRAM through BAR0 VideoPort views.
*/
#include "preos_scanout_native.hpp"

#include "video_helpers.hpp"

namespace rx7900::reactos {
namespace {

using display::PreosStatus;

constexpr ULONG stall_limit_microseconds = 50;
// Frame-edge polling: 50 us steps for at most 100 ms (a 10 Hz frame), then a
// 250 ms yield between the two edges.
constexpr uint32_t edge_poll_microseconds = 50;
constexpr uint32_t edge_poll_limit = 2000;
constexpr LONGLONG measurement_interval_100ns = 2500000;
constexpr ULONG clear_chunk_bytes = 1u << 20;

bool mmio_read(void* context, uint32_t offset, uint32_t& value) noexcept {
    return read_register(*static_cast<HardwareState*>(context), offset, value);
}

// Only read_vram_discovery writes: its MM_INDEX/MM_INDEX_HI window registers.
bool mmio_write(void* context, uint32_t offset, uint32_t value) noexcept {
    return write_register(*static_cast<HardwareState*>(context), offset, value);
}

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

bool same_location(const hardware::VramLocation& a, const hardware::VramLocation& b) noexcept {
    return a.mc_start == b.mc_start && a.mc_end == b.mc_end && a.bytes == b.bytes &&
           a.physical_offset == b.physical_offset;
}

// MMHUB FB_LOCATION/FB_OFFSET and RCC_CONFIG_MEMSIZE through the shared NBIO token.
bool read_location(const hardware::RegisterIo& io, ByteView discovery, hardware::VramLocation& output) noexcept {
    hardware::NbioRegisters registers;
    hardware::Nbio nbio;
    return hardware::validate_nbio_discovery(discovery, io.aperture_bytes, registers) ==
               hardware::DiscoveryStatus::validated &&
           nbio.initialize(registers, io) == hardware::NbioStatus::ready &&
           nbio.vram_location(output) == hardware::NbioStatus::ready;
}

// Busy-polls OTG_STATUS_FRAME_COUNT until it changes; stamps the change.
bool wait_frame_edge(const hardware::RegisterIo& io, const display::PreosDcnRegisters& registers,
                     uint32_t otg, uint32_t& count, LONGLONG& stamp) noexcept {
    uint32_t initial = 0;

    if (!display::read_preos_frame_count(io, registers, otg, initial)) {
        return false;
    }

    for (uint32_t poll = 0; poll < edge_poll_limit; ++poll) {
        KeStallExecutionProcessor(edge_poll_microseconds);
        uint32_t now = 0;

        if (!display::read_preos_frame_count(io, registers, otg, now)) {
            return false;
        }

        if (now != initial) {
            stamp = KeQueryPerformanceCounter(nullptr).QuadPart;
            count = now;
            return true;
        }
    }

    return false;
}

// Read-only refresh measurement between two frame edges about 250 ms apart.
bool measure_refresh(const hardware::RegisterIo& io, const display::PreosDcnRegisters& registers,
                     uint32_t otg, uint32_t& millihz) noexcept {
    LARGE_INTEGER frequency{};
    KeQueryPerformanceCounter(&frequency);
    uint32_t first = 0;
    uint32_t last = 0;
    LONGLONG start = 0;
    LONGLONG end = 0;

    if (frequency.QuadPart <= 0 || !wait_frame_edge(io, registers, otg, first, start)) {
        return false;
    }

    LARGE_INTEGER interval{};
    interval.QuadPart = -measurement_interval_100ns;
    KeDelayExecutionThread(KernelMode, FALSE, &interval);

    return wait_frame_edge(io, registers, otg, last, end) && end > start &&
           display::preos_refresh_from_frames(first, last, static_cast<uint64_t>(end - start),
                                              static_cast<uint64_t>(frequency.QuadPart), millihz);
}

// The Linux v6.12 map exactly as run_asic_bringup's placement builds it: ATOM
// firmwareinfo/vram_usagebyfirmware, gmc_v11_0_get_vbios_fb_size and VRAM size.
bool build_reservations(const hardware::RegisterIo& io, ByteView rom, PreosScanoutNative& preos) noexcept {
    hardware::AtomFirmwareInfo atom{};
    hardware::AtomVramUsage usage{};
    hardware::DcnFbRegisters dcn;
    uint32_t fb_bytes = 0;
    const auto usage_status = hardware::parse_atom_vram_usage(rom, usage);

    if (hardware::parse_atom_firmware_info(rom, atom) != hardware::AtomFirmwareStatus::success ||
        (usage_status != hardware::AtomFirmwareStatus::success &&
         usage_status != hardware::AtomFirmwareStatus::absent) ||
        hardware::validate_dcn_fb_discovery(preos.discovery.bytes(), io.aperture_bytes, dcn) !=
            hardware::DiscoveryStatus::validated ||
        !hardware::read_vbios_fb_bytes(io, dcn, fb_bytes)) {
        return false;
    }

    const hardware::VramReservationInputs inputs{preos.vram.bytes, preos.visible_bytes, atom.capability,
                                                 atom.reserved_fb_bytes, usage, true, fb_bytes};
    return hardware::build_vram_reservation_map(inputs, preos.reservations) ==
           hardware::VramReservationStatus::built;
}

PreosAdoption refuse(PreosScanoutNative& preos, PreosAdoption result) noexcept {
    preos.adopted = false;
    preos.result = result;
    return result;
}

// Fresh MMHUB location and DCN snapshot must describe the captured scanout.
PreosAdoption reconfirm(HardwareState& hardware, PreosScanoutNative& preos) noexcept {
    const hardware::RegisterIo io = register_io(hardware);
    hardware::VramLocation vram{};

    if (!read_location(io, preos.discovery.bytes(), vram) || !same_location(vram, preos.vram)) {
        return PreosAdoption::invalidated;
    }

    display::PreosScanoutRegisters snapshot{};
    display::AdoptedScanout scanout{};
    display::PreosReport report{};

    if (!display::read_preos_scanout(io, preos.registers, snapshot) ||
        display::decode_preos_scanout(snapshot, vram, preos.visible_bytes, scanout, report) !=
            PreosStatus::adopted ||
        !display::same_preos_scanout(preos.scanout, scanout)) {
        preos.report = report;
        return PreosAdoption::invalidated;
    }

    preos.snapshot = snapshot;
    return PreosAdoption::adopted;
}

bool ready(const PreosScanoutNative& preos, VP_STATUS& status) noexcept {
    status = preos.adopted ? NO_ERROR : ERROR_DEV_NOT_EXIST;
    return preos.adopted;
}

} // namespace

PreosAdoption capture_preos_scanout(HardwareState& hardware, ByteView rom,
                                    PreosScanoutNative& preos) noexcept {
    preos.registers = {};
    preos.snapshot = {};
    preos.report = {};
    preos.scanout = {};
    preos.coverage = {};
    preos.vram = {};
    preos.reservations = {};
    preos.visible_bytes = 0;
    preos.captured = false;
    preos.adopted = false;
    preos.mode_set = false;
    preos.result = PreosAdoption::not_captured;

    if (!hardware.acquired || preos.mapping != nullptr || KeGetCurrentIrql() != PASSIVE_LEVEL) {
        return PreosAdoption::not_captured;
    }

    const hardware::RegisterIo io = register_io(hardware);
    hardware::VramDiscoveryReport discovery_report{};

    if (hardware::read_vram_discovery(io, preos.discovery, discovery_report) !=
            hardware::VramDiscoveryStatus::acquired ||
        display::validate_preos_discovery(preos.discovery.bytes(), hardware.register_bytes, preos.registers) !=
            hardware::DiscoveryStatus::validated) {
        return refuse(preos, PreosAdoption::discovery_unavailable);
    }

    if (!read_location(io, preos.discovery.bytes(), preos.vram)) {
        return refuse(preos, PreosAdoption::vram_location_unavailable);
    }

    // gmc_v11_0_mc_init visible_vram_size, whole pages (as VramAperture).
    const uint64_t window = hardware.vram_range_bytes;
    preos.visible_bytes = (window < preos.vram.bytes ? window : preos.vram.bytes) & ~(memory::page_bytes - 1);

    if (!display::read_preos_scanout(io, preos.registers, preos.snapshot)) {
        preos.report.status = PreosStatus::read_failure;
        return refuse(preos, PreosAdoption::decode_refused);
    }

    if (display::decode_preos_scanout(preos.snapshot, preos.vram, preos.visible_bytes, preos.scanout,
                                      preos.report) != PreosStatus::adopted) {
        return refuse(preos, PreosAdoption::decode_refused);
    }

    if (!build_reservations(io, rom, preos)) {
        return refuse(preos, PreosAdoption::reservations_unavailable);
    }

    if (display::check_preos_reservations(preos.reservations, preos.scanout, preos.coverage) ==
        display::PreosCoverage::unreserved) {
        return refuse(preos, PreosAdoption::unreserved);
    }

    // HDMI/DVI PHY PLL rates are not derivable from registers; measure instead.
    // A failed measurement only leaves the refresh unknown.
    if (preos.scanout.clock_source == display::PreosClockSource::unknown && !preos.scanout.variable_refresh) {
        uint32_t millihz = 0;

        if (measure_refresh(io, preos.registers, preos.scanout.otg, millihz)) {
            display::apply_preos_measured_refresh(preos.scanout, millihz);
        }
    }

    preos.captured = true;
    preos.result = PreosAdoption::captured;
    return preos.result;
}

PreosAdoption finalize_preos_without_bringup(HardwareState& hardware, PreosScanoutNative& preos) noexcept {
    if (!preos.captured || preos.adopted || KeGetCurrentIrql() != PASSIVE_LEVEL) {
        return refuse(preos, PreosAdoption::not_captured);
    }

    const PreosAdoption confirmed = reconfirm(hardware, preos);

    if (confirmed != PreosAdoption::adopted) {
        return refuse(preos, confirmed);
    }

    preos.adopted = true;
    preos.result = PreosAdoption::adopted;
    return preos.result;
}

PreosAdoption finalize_preos_after_bringup(HardwareState& hardware, AsicBringup& bringup,
                                           BringupResult result, PreosScanoutNative& preos) noexcept {
    if (!preos.captured || preos.adopted || KeGetCurrentIrql() != PASSIVE_LEVEL) {
        return refuse(preos, PreosAdoption::not_captured);
    }

    switch (result) {
    case BringupResult::completed:
    // These stop before AsicInit::run programs anything (run_asic_bringup order).
    case BringupResult::not_attempted:
    case BringupResult::discovery_unavailable:
    case BringupResult::rom_unavailable:
    case BringupResult::vram_window_unavailable:
    case BringupResult::firmware_unavailable:
    case BringupResult::vram_reserved:
        break;
    default:
        return refuse(preos, PreosAdoption::bringup_incomplete);
    }

    const PreosAdoption confirmed = reconfirm(hardware, preos);

    if (confirmed != PreosAdoption::adopted) {
        return refuse(preos, confirmed);
    }

    // The allocator was initialized from bringup.reservations when it exists.
    const hardware::VramReservationMap& map = bringup.reservations.count != 0 ? bringup.reservations :
                                                                                 preos.reservations;

    if (display::check_preos_reservations(map, preos.scanout, preos.coverage) ==
        display::PreosCoverage::unreserved) {
        return refuse(preos, PreosAdoption::unreserved);
    }

    // amdgpu_vram_mgr_reserve_range: the tail is never handed out once claimed;
    // pending means an allocation already owns those pages.
    if (preos.coverage.coverage == display::PreosCoverage::tail_unreserved &&
        bringup.vram_allocator.initialized() &&
        bringup.vram_allocator.reserve_range(preos.coverage.tail_offset, preos.coverage.tail_bytes) !=
            hardware::VramStatus::success) {
        return refuse(preos, PreosAdoption::tail_claim_failed);
    }

    preos.adopted = true;
    preos.result = PreosAdoption::adopted;
    return preos.result;
}

VP_STATUS preos_mode_information(const PreosScanoutNative& preos, VIDEO_MODE_INFORMATION& output) noexcept {
    VP_STATUS status = NO_ERROR;

    if (!ready(preos, status)) {
        return status;
    }

    const display::AdoptedScanout& scanout = preos.scanout;
    VIDEO_MODE_INFORMATION mode{};
    mode.Length = static_cast<ULONG>(sizeof(mode));
    mode.ModeIndex = 0;
    mode.VisScreenWidth = scanout.timing.width;
    mode.VisScreenHeight = scanout.timing.height;
    mode.ScreenStride = scanout.stride_bytes;
    mode.NumberOfPlanes = 1;
    mode.BitsPerPlane = 32;
    // 1 is the VideoPort "hardware default" refresh when it is not known.
    mode.Frequency = scanout.refresh_millihz != 0 ? (scanout.refresh_millihz + 500u) / 1000u : 1u;
    mode.NumberRedBits = 8;
    mode.NumberGreenBits = 8;
    mode.NumberBlueBits = 8;
    mode.RedMask = 0x00ff0000u;
    mode.GreenMask = 0x0000ff00u;
    mode.BlueMask = 0x000000ffu;
    mode.AttributeFlags = VIDEO_MODE_GRAPHICS | VIDEO_MODE_COLOR | VIDEO_MODE_NO_OFF_SCREEN;
    mode.VideoMemoryBitmapWidth = scanout.stride_bytes / display::xrgb8888_bytes_per_pixel;
    mode.VideoMemoryBitmapHeight = scanout.timing.height;
    // Physical millimeters stay unknown; never invent a monitor size.
    output = mode;
    return NO_ERROR;
}

VP_STATUS preos_query_num_modes(const PreosScanoutNative& preos, VIDEO_REQUEST_PACKET& request) noexcept {
    VIDEO_NUM_MODES modes{};
    modes.NumModes = preos.adopted ? 1u : 0u;
    modes.ModeInformationLength = static_cast<ULONG>(sizeof(VIDEO_MODE_INFORMATION));
    return return_video_value(request, modes);
}

VP_STATUS preos_query_modes(const PreosScanoutNative& preos, VIDEO_REQUEST_PACKET& request) noexcept {
    VIDEO_MODE_INFORMATION mode{};
    const VP_STATUS status = preos_mode_information(preos, mode);
    return status == NO_ERROR ? return_video_value(request, mode) : status;
}

VP_STATUS preos_query_current_mode(const PreosScanoutNative& preos, VIDEO_REQUEST_PACKET& request) noexcept {
    // The adopted mode is current from the start: the pre-OS scanout is running.
    return preos_query_modes(preos, request);
}

VP_STATUS preos_set_current_mode(PreosScanoutNative& preos, const HardwareState& hardware,
                                 PVOID extension, VIDEO_REQUEST_PACKET& request) noexcept {
    VP_STATUS status = NO_ERROR;

    if (!ready(preos, status)) {
        return status;
    }

    display::ModeRequest mode{};
    const ByteView input(static_cast<const uint8_t*>(request.InputBuffer), request.InputBufferLength);

    if (display::decode_video_mode_request(input, mode) != display::Status::success || mode.mode_index != 0) {
        return ERROR_INVALID_PARAMETER;
    }

    if (VideoPortGetCurrentIrql() != PASSIVE_LEVEL) {
        return ERROR_INVALID_FUNCTION;
    }

    // Only the adopted surface bytes are cleared; nothing is programmed.
    for (uint64_t done = 0; !mode.preserve_contents && done < preos.scanout.bytes;) {
        const uint64_t remaining = preos.scanout.bytes - done;
        const ULONG chunk = remaining < clear_chunk_bytes ? static_cast<ULONG>(remaining) : clear_chunk_bytes;
        PUCHAR view = nullptr;
        status = map_vram_temporary(hardware, extension, preos.scanout.vram_offset + done, chunk, view);

        if (status != NO_ERROR) {
            return status;
        }

        // view covers exactly chunk mapped bytes until the unmap below.
        VideoPortZeroDeviceMemory(view, chunk);
        unmap_vram_temporary(extension, view);
        done += chunk;
    }

    preos.mode_set = true;
    return NO_ERROR;
}

VP_STATUS preos_map_video_memory(PreosScanoutNative& preos, const HardwareState& hardware,
                                 PVOID extension, VIDEO_REQUEST_PACKET& request) noexcept {
    VP_STATUS status = NO_ERROR;

    if (!ready(preos, status)) {
        return status;
    }

    VIDEO_MEMORY input{};
    VIDEO_MEMORY_INFORMATION output{};
    const uint64_t offset = preos.scanout.vram_offset;
    const uint64_t bytes = preos.scanout.bytes;

    if (!read_video_value(request, input) || input.RequestedVirtualAddress != nullptr ||
        (offset & (memory::page_bytes - 1)) != 0 || bytes > MAXULONG ||
        offset > hardware.vram_range_bytes || bytes > hardware.vram_range_bytes - offset ||
        static_cast<uint64_t>(hardware.vram_range.QuadPart) > INT64_MAX - offset) {
        return ERROR_INVALID_PARAMETER;
    }

    if (!has_video_output(request, sizeof(output))) {
        return ERROR_INSUFFICIENT_BUFFER;
    }

    if (!preos.mode_set || preos.mapping != nullptr || !hardware.acquired ||
        VideoPortGetCurrentIrql() != PASSIVE_LEVEL) {
        return ERROR_BUSY;
    }

    PHYSICAL_ADDRESS start = hardware.vram_range;
    start.QuadPart += static_cast<LONGLONG>(offset);
    ULONG length = static_cast<ULONG>(bytes);
    // Linear framebuffer, write-combining hint as the ReactOS VBE miniport uses.
    ULONG space = VIDEO_MEMORY_SPACE_MEMORY | VIDEO_MEMORY_SPACE_P6CACHE;
    PVOID virtual_address = nullptr;
    status = VideoPortMapMemory(extension, start, &length, &space, &virtual_address);

    if (status != NO_ERROR || virtual_address == nullptr || length < bytes) {
        if (status == NO_ERROR && virtual_address != nullptr) {
            VideoPortUnmapMemory(extension, virtual_address, nullptr);
        }

        return status != NO_ERROR ? status : ERROR_NOT_ENOUGH_MEMORY;
    }

    preos.mapping = virtual_address;
    output.VideoRamBase = virtual_address;
    output.VideoRamLength = length;
    output.FrameBufferBase = virtual_address;
    output.FrameBufferLength = static_cast<ULONG>(bytes);
    return return_video_value(request, output);
}

VP_STATUS preos_unmap_video_memory(PreosScanoutNative& preos, PVOID extension,
                                   VIDEO_REQUEST_PACKET& request) noexcept {
    VIDEO_MEMORY input{};

    if (!read_video_value(request, input) || input.RequestedVirtualAddress == nullptr ||
        input.RequestedVirtualAddress != preos.mapping) {
        return ERROR_INVALID_PARAMETER;
    }

    const VP_STATUS status = VideoPortUnmapMemory(extension, preos.mapping, nullptr);

    if (status == NO_ERROR) {
        preos.mapping = nullptr;
    }

    return status;
}

} // namespace rx7900::reactos

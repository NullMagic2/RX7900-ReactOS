/*
Module: rx7900.display.vbe
Purpose: Select and set a VESA BIOS Extensions linear 32 bpp mode so that a legacy
(CSM) boot leaves an adoptable DCN3.2 scanout.
Created: 2026-10-22
Architecture: Portable VBE 2.0/3.0 structure parsing, mode choice and the ordered
INT 10h sequence behind one callback table (VbeBios). The native adapter
(src/reactos/vbe_modeset) binds that table to the ReactOS VideoPort Int10
interface; the pre-OS adoption (src/display/preos_scanout) then decides from
the DCN state the VBIOS programmed. EDID is decoded by src/output/sink.
Structure offsets follow the VESA BIOS Extension (VBE) Core Functions Standard
3.0 and VBE/DDC 1.1; no code is copied from another implementation.
*/
#pragma once

#include "byte_view.hpp"
#include "sink.hpp"

namespace rx7900::display {

// Real-mode memory is the first MiB; every VBE far pointer must stay inside it.
constexpr uint32_t vbe_real_mode_limit = 0x100000;
constexpr uint32_t vbe_controller_info_bytes = 512;
constexpr uint32_t vbe_mode_info_bytes = 256;
// Scratch layout inside the caller's real-mode buffer: controller block, then
// the mode block, then one EDID block.
constexpr uint32_t vbe_mode_info_offset = vbe_controller_info_bytes;
constexpr uint32_t vbe_edid_offset = vbe_mode_info_offset + vbe_mode_info_bytes;
constexpr uint32_t vbe_scratch_bytes = vbe_edid_offset + output::edid_block_bytes;
// Bounded policy: at most this many mode list entries are read (VESA lists are
// far shorter); a list without its 0xffff terminator inside the bound is refused.
constexpr uint32_t vbe_mode_list_capacity = 256;
// Minimum VBE version: 2.0 introduced PhysBasePtr and the 4F02 bit 14 request.
constexpr uint16_t vbe_minimum_version = 0x0200;
constexpr uint16_t vbe_maximum_version = 0x0300;
// Accepted geometry: at least the 640x480 boot-video minimum, at most the DCN
// 16384 pixel surface limit.
constexpr uint32_t vbe_minimum_width = 640;
constexpr uint32_t vbe_minimum_height = 480;
constexpr uint32_t vbe_maximum_dimension = 16384;
// Without a usable EDID preferred timing the choice is capped at a resolution
// every VESA DMT monitor supports.
constexpr uint32_t vbe_fallback_width = 1024;
constexpr uint32_t vbe_fallback_height = 768;

// The INT 10h register image the BIOS sees. es:di addresses the scratch buffer.
struct VbeRegisters final {
    uint32_t eax;
    uint32_t ebx;
    uint32_t ecx;
    uint32_t edx;
    uint32_t edi;
    uint16_t es;
};

// Caller-owned BIOS access. buffer_linear is the real-mode linear address of
// buffer_bytes caller-owned bytes (vbe_scratch_bytes at least). call runs one
// INT 10h and returns false only when the BIOS could not be invoked. read and
// write copy bytes between real-mode linear addresses and caller storage; they
// return false on any access failure. All callbacks run synchronously on the
// caller's thread; context outlives the sequence.
struct VbeBios final {
    void* context;
    uint32_t buffer_linear;
    uint32_t buffer_bytes;
    bool (*call)(void* context, VbeRegisters& registers) noexcept;
    bool (*read)(void* context, uint32_t linear, uint8_t* output, uint32_t bytes) noexcept;
    bool (*write)(void* context, uint32_t linear, const uint8_t* input, uint32_t bytes) noexcept;
};

enum class VbeStatus {
    mode_set,
    invalid_argument,
    // A callback reported that the BIOS or real-mode memory was unreachable.
    bios_unavailable,
    // A VBE function returned AX other than 0x004f.
    function_failed,
    // 4F03 reports a linear-framebuffer mode already: nothing is changed.
    already_linear,
    malformed_controller_info,
    unsupported_version,
    malformed_mode_list,
    // No 32 bpp direct-colour linear mode inside BAR0 at or below the preference.
    no_usable_mode,
    // 4F02 failed, or 4F03 afterwards does not report the chosen linear mode;
    // the previous mode was then requested again (see VbeReport::restored).
    set_failed,
    verify_failed
};

// Per-mode refusal reasons, counted in VbeReport::refusals.
enum class VbeModeStatus {
    usable,
    malformed,
    not_supported,       // ModeAttributes bit 0, 1, 3 or 4 clear
    no_linear_buffer,    // ModeAttributes bit 7 clear or PhysBasePtr zero
    unsupported_format,  // not one plane, 32 bpp, direct colour X8R8G8B8
    unsupported_geometry,
    outside_aperture,    // surface not wholly inside BAR0 or VBE TotalMemory
    above_preference
};

constexpr uint32_t vbe_mode_status_count = 8;

struct VbeControllerInfo final {
    uint16_t version;
    // Real-mode linear address of the VideoModePtr list.
    uint32_t mode_list_linear;
    uint64_t total_memory_bytes;
};

// One validated linear mode. Memory order B, G, R, X (GDI 0x00ff0000/0x0000ff00/0x000000ff).
struct VbeModeInfo final {
    uint16_t mode;
    uint32_t width;
    uint32_t height;
    uint32_t stride_bytes;
    // CPU physical address of the first pixel, as the VBIOS reports it.
    uint32_t physical_base;
    uint64_t bytes;
};

enum class VbePreferenceSource {
    edid_preferred,
    fallback
};

struct VbePreference final {
    uint32_t width;
    uint32_t height;
    VbePreferenceSource source;
};

// The CPU-physical BAR0 window the miniport owns; modes must lie inside it.
struct VbeAperture final {
    uint64_t base;
    uint64_t bytes;
};

// Caller-owned storage (about 5 KiB) so that the sequence keeps no large
// buffers on the kernel stack. Contents are scratch; the report is the result.
struct VbeWorkspace final {
    uint8_t controller[vbe_controller_info_bytes];
    uint8_t mode_block[vbe_mode_info_bytes];
    uint8_t list[vbe_mode_list_capacity * 2];
    uint8_t edid[output::maximum_edid_blocks * output::edid_block_bytes];
    output::SinkSnapshot sink;
    uint16_t modes[vbe_mode_list_capacity];
};

struct VbeReport final {
    VbeStatus status;
    // VBE function (0x4f00..0x4f15, or 0 for AH=00h) and AX of the first failure.
    uint16_t failed_function;
    uint16_t failed_ax;
    uint16_t version;
    // 4F03 BX before any change; legacy VGA modes are below 0x100.
    uint16_t previous_mode;
    uint32_t listed_modes;
    uint32_t refusals[vbe_mode_status_count];
    bool edid_read;
    VbePreference preference;
    VbeModeInfo chosen;
    // After set_failed/verify_failed: the previous mode was requested again
    // and the BIOS reported success.
    bool restored;
};

// Pure. block is the 512-byte 4F00 result. Requires "VESA", a version from
// vbe_minimum_version to vbe_maximum_version, a VideoModePtr inside real-mode
// memory and a nonzero TotalMemory (64 KiB units). Output unchanged on failure.
VbeStatus parse_vbe_controller_info(ByteView block, VbeControllerInfo& output) noexcept;

// Pure. list holds the bytes at VideoModePtr. Copies VESA mode numbers
// (0x100..0x1ff) up to the 0xffff terminator into modes and skips legacy or
// reserved-bit entries. malformed_mode_list when no terminator lies within
// list or capacity entries; output count then is zero.
VbeStatus parse_vbe_mode_list(ByteView list, uint16_t* modes, uint32_t capacity, uint32_t& count) noexcept;

// Pure. block is the 256-byte 4F01 result for mode. Checks attributes,
// X8R8G8B8 direct colour, geometry, the stride (LinBytesPerScanLine from VBE
// 3.0, otherwise BytesPerScanLine), page alignment and that the surface lies in
// both the aperture and total_memory_bytes. Output written only on usable.
VbeModeStatus parse_vbe_mode_info(ByteView block, uint16_t version, uint16_t mode,
                                  const VbeAperture& aperture, uint64_t total_memory_bytes,
                                  VbeModeInfo& output) noexcept;

// Pure. Preferred timing of a parsed sink (first base DTD marked preferred,
// progressive); otherwise the fallback cap. Always writes output.
void vbe_preference_from_sink(const output::SinkSnapshot& sink, bool sink_valid,
                              VbePreference& output) noexcept;

// Pure. True when candidate fits the preference and ranks above best: the exact
// preferred resolution first, then the larger area, then the larger width; ties
// keep best. has_best false means there is no current choice.
bool vbe_mode_preferred(const VbeModeInfo& candidate, bool has_best, const VbeModeInfo& best,
                        const VbePreference& preference) noexcept;

// PASSIVE_LEVEL in the native caller; exclusive owner of the display BIOS. In
// order: 4F03 (refuse already_linear), 4F00 with "VBE2", the mode list, VBE/DDC
// 4F15 EDID (optional), 4F01 for every listed mode, then 4F02 with bit 14 for
// the best usable mode and 4F03 to confirm it. Nothing is changed before 4F02;
// on a 4F02 or confirmation failure the previous mode is requested again
// (AH=00h for a legacy mode, 4F02 otherwise). report is always written.
VbeStatus set_vbe_linear_mode(const VbeBios& bios, const VbeAperture& aperture, VbeWorkspace& workspace,
                              VbeReport& report) noexcept;

} // namespace rx7900::display

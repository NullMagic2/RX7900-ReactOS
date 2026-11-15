/*
Module: rx7900.display.modes
Purpose: Validate DCN3.2 timings and owned scanout ranges before hardware access.
Created: 2026-10-21
Architecture: Allocation-free display core shared by the future VideoPort miniport.
The adapted AMD timing checks and fields retain their notice in AMDGPU-DISPLAY.txt.
*/
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "byte_view.hpp"

namespace rx7900::display {

enum class Status {
    success,
    invalid_timing,
    unsupported_timing,
    invalid_framebuffer,
    arithmetic_overflow,
    capacity_exceeded,
    duplicate_mode,
    mode_not_found,
    invalid_request,
    hardware_not_ready,
    hardware_failure,
    state_unknown,
    busy
};

constexpr uint32_t positive_hsync = 1u;
constexpr uint32_t positive_vsync = 2u;
constexpr uint32_t interlaced = 4u;
constexpr uint32_t mode_capacity = 64u;
constexpr uint32_t xrgb8888_bytes_per_pixel = 4u;

// Progressive, borderless RGB timing. Pixel clock is in kHz, lengths in pixels
// or lines. Sink/link/clock/bandwidth support must also be checked by the backend.
struct Timing final {
    uint32_t pixel_clock_khz;
    uint32_t width;
    uint32_t h_total;
    uint32_t h_front_porch;
    uint32_t h_sync_width;
    uint32_t height;
    uint32_t v_total;
    uint32_t v_front_porch;
    uint32_t v_sync_width;
    uint32_t flags;
};

// Prepared field values, never register addresses or a complete hardware script.
// Horizontal/vertical sync start are zero; total_min/max equal v_total_minus_one.
// Global sync, link/PLL, plane, HUBP and bandwidth programming are separate work.
struct OptcTiming final {
    uint32_t h_total_minus_one;
    uint32_t h_blank_start;
    uint32_t h_blank_end;
    uint32_t h_sync_end;
    uint32_t h_sync_negative;
    uint32_t v_total_minus_one;
    uint32_t v_blank_start;
    uint32_t v_blank_end;
    uint32_t v_sync_end;
    uint32_t v_sync_negative;
};

// Pure, callable at any IRQL with resident inputs/outputs. Outputs are unchanged
// on failure. This validates timing-field representability, not monitor support.
Status prepare_optc_timing(const Timing& timing, OptcTiming& output) noexcept;
Status refresh_millihz(const Timing& timing, uint32_t& output) noexcept;

// This is a GPU scanout address, never a CPU pointer, PCI BAR or user address.
// An eventual VRAM/GPUVM allocator must establish residency and mapping lifetime.
struct GpuScanoutAddress final {
    uint64_t value;
};

// Device-owned linear XRGB8888 allocation supplied by trusted miniport code.
// The allocation owner retains it through commit, scanout and hardware retirement.
struct Framebuffer final {
    GpuScanoutAddress allocation_base;
    uint64_t allocation_bytes;
    uint64_t surface_offset;
    uint32_t stride_bytes;
};

struct ScanoutLayout final {
    GpuScanoutAddress surface_address;
    uint64_t surface_bytes;
    uint32_t stride_bytes;
};

// Checks the full stride times height footprint and inclusive GPU address range.
// Does not infer capacity from a PCI BAR or prove that an address is GPU resident.
Status validate_framebuffer(const Timing& timing, const Framebuffer& framebuffer,
                            ScanoutLayout& output) noexcept;

class ModeCatalog final {
public:
    // Caller serializes access. Successful append creates a stable zero-based ID.
    // No default modes are advertised without a discovered sink and backend check.
    Status append(const Timing& timing, uint32_t& mode_index) noexcept;
    Status get(uint32_t mode_index, Timing& output) const noexcept;
    uint32_t count() const noexcept { return count_; }

private:
    Timing modes_[mode_capacity]{};
    uint32_t count_ = 0;
};

struct ModeRequest final {
    uint32_t mode_index;
    bool preserve_contents;
};

// Decode VIDEO_MODE.RequestedMode with the existing checked binary view. The
// caller retains buffer ownership; the result contains no pointer into input.
// VIDEO_MODE_MAP_MEM_LINEAR is accepted because only linear XRGB8888 is supported.
Status decode_video_mode_request(ByteView input, ModeRequest& output) noexcept;

struct ModePlan final {
    uint32_t mode_index;
    Timing timing;
    OptcTiming optc;
    ScanoutLayout scanout;
    bool preserve_contents;
};

enum class ApplyResult {
    // Only after hardware completion and readback confirm the requested scanout.
    committed,
    // Failure with proof that the previous scanout and framebuffer are unchanged.
    unchanged,
    // Hardware may have changed; the old scanout cannot be reported as current.
    indeterminate
};

// Non-owning callbacks/context stay valid for the controller's entire lifetime.
// Callers serialize at PASSIVE_LEVEL; callbacks must not throw or retain plans.
// prepare performs side-effect-free capability/residency validation. apply owns
// synchronization, vblank/timeout, mode programming, completion/readback and any
// requested framebuffer clear. There is deliberately no fake hardware backend.
// If apply returns indeterminate, the coordinator attempts the known previous
// plan with preserve_contents=true; that allocation must still be retained.
struct ModeBackend final {
    void* context = nullptr;
    Status (*prepare)(void* context, const ModePlan& plan) noexcept = nullptr;
    ApplyResult (*apply)(void* context, const ModePlan& plan) noexcept = nullptr;
};

class ModeController final {
public:
    explicit ModeController(ModeBackend backend = {}) noexcept : backend_(backend) {}
    ModeController(const ModeController&) = delete;
    ModeController& operator=(const ModeController&) = delete;

    // Catalog is device-owned. Append only modes that passed sink/link validation.
    Status append_mode(const Timing& timing, uint32_t& mode_index) noexcept;
    uint32_t mode_count() const noexcept { return catalog_.count(); }
    Status query_mode(uint32_t mode_index, Timing& output) const noexcept;

    // Success means a backend confirmed real hardware, never just cached state.
    // Failed prepares keep the old state. An unconfirmed apply invalidates it
    // unless an attempted rollback is confirmed. No caller buffers are retained.
    Status set_mode(const ModeRequest& request, const Framebuffer& framebuffer) noexcept;
    Status current_mode(ModePlan& output) const noexcept;

    // Call after reset, hot-unplug or any external loss of scanout ownership.
    // Invalidating knowledge does not retire or release the backing allocation.
    Status invalidate_current() noexcept;

private:
    ModeCatalog catalog_{};
    ModeBackend backend_{};
    ModePlan current_{};
    bool current_known_ = false;
    bool busy_ = false;
};

} // namespace rx7900::display

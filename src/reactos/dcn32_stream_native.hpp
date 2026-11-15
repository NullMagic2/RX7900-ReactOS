/*
Module: rx7900.reactos.dcn32_stream_native
Purpose: VideoPort mode service that commits EDID modes through the DCN 3.2 stream owner and the link owner.
Created: 2026-10-24
Architecture: Native adapter (RX7900_DISPLAY_STREAM builds only) between
video.cpp, display::Dcn32StreamController/Dcn32Bandwidth and the bound
output::DisplayLinkOwner of NativeDisplayLink. It takes over the adopted
pre-OS pipe (mode 0, the GOP mode, kept as the fallback), reads the sink EDID
(DP: I2C-over-AUX probe; HDMI/DVI: DDC I2C), advertises only modes that the
sink, the link, the TMDS rules and DML32 accept, allocates one pinned
contiguous CPU-visible VRAM surface per mode set, and serves the VideoPort
mode IOCTLs with up to three live surfaces (a ChangeDisplaySettings switch
maps the new surface before the old one is unmapped). Floating-point DML runs
inside KeSaveFloatingPointState. Nothing is programmed at start: the first
register write happens on an explicit IOCTL_VIDEO_SET_CURRENT_MODE of a mode
other than the running one, and only when the service key sets
DisplayModeSet=1 on top of PhysicalAsicInit=1 and DisplayLinkOwner=1.
dcn-dsc-odm sub-switches of DisplayModeSet (REG_DWORD, default 0, only 1
enables): DisplayDsc (DP SST 8b/10b DSC with FEC for modes the link cannot
carry uncompressed, sink DPCD DSC + FEC capable), DisplayOdm (ODM combine 2:1
when DML32 or the dcn32 single-display power policy asks for it) and
DisplayOverlay (the driver-internal second plane of set_overlay; no GDI or
DirectDraw overlay is exposed). The hardware cursor stays on the OTG master's
HUBP: with ODM it covers the left slice only, with an overlay it is not drawn
above the overlay plane.
*/
#pragma once

#include "asic_bringup.hpp"
#include "dcn32_stream.hpp"
#include "display_link_native.hpp"
#include "dp_link.hpp"
#include "edid_modes.hpp"
#include "preos_scanout_native.hpp"
#include "sink.hpp"

namespace rx7900::reactos {

// REG_DWORD in the service key; only 1 enables the stream owner. Never implied by
// PhysicalAsicInit, DisplayLinkOwner, installation or a parsed EDID.
constexpr wchar_t display_mode_set_registry_value[] = L"DisplayModeSet";
// dcn-dsc-odm sub-switches (REG_DWORD, only 1 enables; read at start()).
constexpr wchar_t display_dsc_registry_value[] = L"DisplayDsc";
constexpr wchar_t display_odm_registry_value[] = L"DisplayOdm";
constexpr wchar_t display_overlay_registry_value[] = L"DisplayOverlay";

// The GOP mode plus the distinct (size, refresh) EDID modes that validate.
constexpr uint32_t display_stream_mode_capacity = 48;
constexpr uint32_t display_stream_surface_capacity = 3;
constexpr uint32_t display_stream_no_index = UINT32_MAX;

// Reported numerically by the miniport; new values are appended.
enum class DisplayStreamStart : uint32_t {
    started,
    invalid_argument,
    link_unavailable,
    firmware_pipe_unsupported,
    route_unavailable,
    sink_unavailable,
    clocks_unavailable,
    workspace_unavailable,
    bandwidth_unavailable,
    controller_unavailable
};

struct DisplayStreamMode final {
    display::Timing timing{};
    uint32_t refresh_millihz = 0;
    display::StreamSignal signal = display::StreamSignal::display_port;
    // Index into the sink snapshot's mode catalog (DP training needs it):
    // its detailed timings first, then the other EDID modes (edid_modes).
    uint32_t sink_mode_index = display_stream_no_index;
    display::EdidModeSource source = display::EdidModeSource::detailed;
    uint8_t dp_rate_code = 0;
    uint8_t dp_lanes = 0;
    display::TmdsStreamPlan tmds{};
    // dcn-dsc-odm: dc_dsc_compute_config result (num_slices_h 0: uncompressed);
    // DSC streams train with FEC.
    display::DscConfig dsc{};
    bool fec = false;
    // Mode 0: the adopted pre-OS mode on the stolen firmware surface.
    bool firmware = false;
    // A known pixel clock and link settings: the stream owner can commit it.
    bool committable = false;
};

struct DisplayStreamSurface final {
    hardware::VramAllocationHandle allocation{};
    uint64_t vram_offset = 0;
    uint64_t mc_address = 0;
    uint64_t bytes = 0;
    uint32_t pitch_pixels = 0;
    uint32_t width = 0;
    uint32_t height = 0;
    // VideoPortMapMemory view handed to the display driver (one per surface).
    PVOID mapping = nullptr;
    bool used = false;
    // Allocated here (released when neither scanned nor mapped); the firmware
    // surface belongs to the pre-OS reservation and is never released.
    bool owned = false;
    bool scanned = false;
};

// dcn-dsc-odm: a second plane blended above the primary (MPC top-bottom
// blending, plane order as amdgpu_dm's zpos). allocation is a pinned,
// contiguous VRAM allocation of the caller (inputs.bringup->vram_allocator)
// that stays allocated until clear_overlay() or a mode change removed the
// plane; the scanout covers pitch_pixels * 4 * height bytes of it. 1:1 only
// (no scaling), fully inside the active area, XRGB/ARGB8888 linear.
struct DisplayOverlay final {
    hardware::VramAllocationHandle allocation{};
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t pitch_pixels = 0;
    int32_t x = 0;
    int32_t y = 0;
    bool per_pixel_alpha = false;
    bool pre_multiplied_alpha = false;
    bool global_alpha = false;
    uint8_t global_alpha_value = 0xff;
};

struct NativeDisplayStreamInputs final {
    HardwareState* hardware = nullptr;
    AsicBringup* bringup = nullptr;
    PreosScanoutNative* preos = nullptr;
    NativeDisplayLink* link = nullptr;
    PVOID extension = nullptr;
    ByteView rom{nullptr, 0};
    // Epoch of the bound link owner (route leases name it).
    uint64_t epoch = 0;
};

// Device-extension-owned (trivially destructible), PASSIVE_LEVEL, serialized by
// the device lock with power, recovery, removal and every IOCTL.
class NativeDisplayStream final {
public:
    NativeDisplayStream() noexcept = default;
    NativeDisplayStream(const NativeDisplayStream&) = delete;
    NativeDisplayStream& operator=(const NativeDisplayStream&) = delete;

    DisplayStreamStart start(const NativeDisplayStreamInputs& inputs) noexcept;
    bool active() const noexcept { return started_; }
    DisplayStreamStart start_result() const noexcept { return start_result_; }

    static bool is_mode_ioctl(ULONG code) noexcept;
    // QUERY_NUM_AVAIL_MODES, QUERY_AVAIL_MODES, QUERY_CURRENT_MODE,
    // SET_CURRENT_MODE, MAP_VIDEO_MEMORY, UNMAP_VIDEO_MEMORY, RESET_DEVICE.
    VP_STATUS start_io(VIDEO_REQUEST_PACKET& request) noexcept;

    // The surface the stream owner scans out as a pre-OS-style descriptor
    // (for the cursor, Radeon 2D primary, GL present target and the recovery
    // scanout check). generation() changes on every committed mode change.
    bool scanout(display::AdoptedScanout& output) const noexcept;
    uint32_t generation() const noexcept { return generation_; }
    // The firmware mode is still the one scanning (nothing committed yet).
    bool firmware_current() const noexcept { return controller_.firmware_current(); }

    // Before D3 or a destructive reset: Linux disable path of the stream (link
    // off). false: the stream owner is quarantined.
    bool suspend() noexcept;
    // The display hardware was reset (mode-1 reset proof): every hardware
    // assumption is dropped, the HPD binding is retired and the DP AUX
    // adapter is re-armed for a later epoch; the link owner must be re-bound
    // before restore.
    void reset_proven() noexcept;
    // After the link owner was bound again (new epoch) or a suspend was
    // aborted (same epoch): re-acquire the route, for DP re-bind the AUX
    // adapter under the new epoch's proof and require the same DPCD receiver
    // capabilities, then commit the current mode (Linux dm_resume of the
    // cached state; DP link training from the commit). A failed commit falls
    // back to the GOP mode on its firmware surface, then to blank, as a mode
    // change does. false: the stream stays off (blank) or quarantined.
    bool restore(uint64_t epoch) noexcept;
    // Periodic/explicit health check; recovers (bounded) on underflow or a
    // stalled OTG. true when the stream is healthy or idle.
    bool check_and_recover() noexcept;

    // Release surfaces, the route lease and the workspace when nothing is
    // uncertain; false keeps everything (the hardware may still scan out).
    bool retire() noexcept;

    const display::StreamReport& report() const noexcept { return controller_.report(); }
    const display::EdidModeReport& edid_report() const noexcept { return edid_report_; }
    uint32_t mode_count() const noexcept { return mode_count_; }
    uint32_t current_mode() const noexcept { return current_mode_; }

    // dcn-dsc-odm driver-internal overlay plane (DisplayOverlay=1, a mode set
    // by this owner): Linux dc_update_planes_and_stream of a FULL surface
    // update on the running stream (Dcn32StreamController::update_planes).
    // A mode change, restore or the GOP fallback drops the overlay. PASSIVE_LEVEL
    // under the device lock. NO_ERROR when the planes are scanning out.
    VP_STATUS set_overlay(const DisplayOverlay& overlay) noexcept;
    VP_STATUS clear_overlay() noexcept;
    bool overlay_active() const noexcept { return overlay_active_; }

private:
    static display::LinkCall hook_update_clocks(void*, const display::Dcn32StreamClocks&, bool) noexcept;
    static display::LinkCall hook_set_pixel_clock(void*, const display::Dcn32StreamPlan&) noexcept;
    static display::LinkCall hook_power_down(void*, const display::Dcn32StreamPlan&) noexcept;
    static display::LinkCall hook_enable_link(void*, const display::Dcn32StreamPlan&) noexcept;
    static display::LinkCall hook_disable_link(void*, const display::Dcn32StreamPlan&) noexcept;
    static display::LinkCall hook_dpcd_read(void*, uint32_t, uint8_t&) noexcept;
    static display::LinkCall hook_dpcd_write(void*, uint32_t, uint8_t) noexcept;
    static display::LinkCall hook_p_state_delegate(void*, bool) noexcept;
    static display::LinkCall hook_hdmi_scdc_legacy(void*, const display::Dcn32StreamPlan&) noexcept;
    static bool read32(void*, uint32_t, uint32_t&) noexcept;
    static bool write32(void*, uint32_t, uint32_t) noexcept;
    static void delay(void*, uint32_t) noexcept;

    bool find_firmware_route(uint32_t otg) noexcept;
    bool read_sink() noexcept;
    void read_dsc_caps() noexcept;
    bool decide_dsc_link(DisplayStreamMode& mode) noexcept;
    bool dsc_mode_valid(const DisplayStreamMode& mode) noexcept;
    VP_STATUS update_overlay(const DisplayOverlay* overlay) noexcept;
    bool read_tmds_edid() noexcept;
    void widen_sink_modes(ByteView edid, output::SinkSnapshot& sink) noexcept;
    bool rearm_dp_aux() noexcept;
    display::StreamResult commit_restored(uint32_t index, uint32_t slot) noexcept;
    bool configure_bandwidth() noexcept;
    void build_modes() noexcept;
    bool validate_mode(DisplayStreamMode& mode) noexcept;
    bool decide_dp_link(DisplayStreamMode& mode) noexcept;
    // overlay (dcn-dsc-odm): validate the two-plane context of the running
    // stream (the overlay above the primary of pitch_pixels).
    bool bandwidth_for(const DisplayStreamMode& mode, uint32_t pitch_pixels,
                       const DisplayOverlay* overlay = nullptr) noexcept;
    VP_STATUS describe(uint32_t index, VIDEO_MODE_INFORMATION& output) const noexcept;
    VP_STATUS query_modes(VIDEO_REQUEST_PACKET& request) const noexcept;
    VP_STATUS set_mode(VIDEO_REQUEST_PACKET& request) noexcept;
    VP_STATUS map_memory(VIDEO_REQUEST_PACKET& request) noexcept;
    VP_STATUS unmap_memory(VIDEO_REQUEST_PACKET& request) noexcept;
    VP_STATUS commit_mode(uint32_t index, bool preserve) noexcept;
    uint32_t acquire_surface(uint32_t index) noexcept;
    bool clear_surface(const DisplayStreamSurface& surface) noexcept;
    void release_unused_surfaces() noexcept;
    bool build_plan(uint32_t mode_index, const DisplayStreamSurface& surface,
                    const DisplayOverlay* overlay = nullptr, uint64_t overlay_mc_address = 0) noexcept;
    output::TmdsKind tmds_kind(display::StreamSignal signal) const noexcept;

    NativeDisplayStreamInputs inputs_{};
    display::Dcn32StreamController controller_{};
    display::Dcn32Bandwidth bandwidth_{};
    display::Dcn32BandwidthWorkspace* workspace_ = nullptr;
    void* workspace_storage_ = nullptr;
    display::Dcn32SocInputs soc_{};
    display::Dcn32BandwidthPlan bandwidth_plan_{};
    display::Dcn32StreamRequest request_{};
    display::Dcn32StreamPlan plan_{};
    display::Dcn32FirmwarePipe firmware_{};
    output::RouteLease lease_{};
    output::Dcn32ConnectorRoute route_{};
    NativeDcn32Hpd hpd_{};
    NativeDcn32Aux aux_{};
    output::AuxIo aux_io_{};
    output::DpProbeScratch dp_scratch_{};
    output::DpPortSnapshot dp_port_{};
    output::SinkSnapshot sink_{};
    // Source of every catalog entry of sink_ (detailed first) and the decode report.
    display::EdidModeSource mode_sources_[display::mode_capacity]{};
    display::EdidModeReport edid_report_{};
    display::HdmiSinkInfo hdmi_{};
    uint8_t edid_[output::maximum_edid_blocks * output::edid_block_bytes]{};
    uint32_t edid_bytes_ = 0;
    DisplayStreamMode modes_[display_stream_mode_capacity]{};
    uint32_t mode_count_ = 0;
    DisplayStreamSurface surfaces_[display_stream_surface_capacity]{};
    uint32_t current_mode_ = 0;
    uint32_t current_surface_ = display_stream_no_index;
    uint32_t generation_ = 0;
    uint32_t dchub_refclk_khz_ = 0;
    uint32_t max_cab_bytes_ = 0;
    display::Dcn32SurfaceTiling tiling_{};
    bool dp_use_vsc_ = false;
    bool mode_set_ = false;
    bool started_ = false;
    bool lease_held_ = false;
    DisplayStreamStart start_result_ = DisplayStreamStart::invalid_argument;
    // dcn-dsc-odm: registry sub-switches, sink DSC/FEC capabilities, the
    // overlay plane and a dsc_prepare_config scratch (kept out of the frame).
    bool allow_dsc_ = false;
    bool allow_odm_ = false;
    bool allow_overlay_ = false;
    bool dsc_capable_ = false;
    display::DscSinkCaps dsc_sink_{};
    display::DscRegisterValues dsc_scratch_{};
    DisplayOverlay overlay_{};
    bool overlay_active_ = false;
};

} // namespace rx7900::reactos

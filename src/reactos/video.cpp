/*
Module: rx7900.reactos.video
Purpose: Native VideoPort discovery and checked display-mode IOCTL integration.
Created: 2026-10-21
Architecture: Separate experimental display miniport; diagnostic driver unchanged.
Uses public-domain ReactOS video.h/ntddvdeo.h APIs. No virtual GPU code copied.
This module deliberately refuses initialization until physical DCN/link/scanout
bring-up exists; an opt-in registry value runs PSP/SMU firmware loading first. Registering it is not permission to replace the working driver.
*/
#include "hardware.hpp"
#include "asic_bringup.hpp"
#include "gpu_process.hpp"
#include "vbios.hpp"
#include "preos_scanout_native.hpp"
#include "vbe_modeset.hpp"
#include "radeon_cursor_native.hpp"
#include "atom_connectors.hpp"
#include "irq_service.hpp"
#include "fence_native.hpp"
#include "gpu_health_native.hpp"
#include "lifetime.hpp"
#include "modes.hpp"
#include "video_helpers.hpp"
#ifdef RX7900_RADEON2D_VIDEOPORT
#include "radeon_2d_ioctl.hpp"
#include "../hardware/radeon_2d_backend.hpp"
#endif
#ifdef RX7900_DISPLAY_LINK
#include "display_link_native.hpp"
#include "dcn32_cursor.hpp"
#endif
#ifdef RX7900_DISPLAY_STREAM
#include "dcn32_stream_native.hpp"
#endif

namespace {

using rx7900::display::ModePlan;
using rx7900::display::Status;

PDRIVER_OBJECT physical_driver = nullptr;
PDRIVER_UNLOAD videoport_unload = nullptr;
bool physical_driver_pinned = false;

// Only the inactive driver is unloadable. A running ASIC already retains its
// IH mappings and callbacks; no module may unload while those can execute.
void NTAPI inactive_driver_unload(PDRIVER_OBJECT driver) {
    rx7900::reactos::gpu_process_driver_stop();
    if (videoport_unload != nullptr) videoport_unload(driver);
}

// VideoPort owns and zeroes extension storage. HwFindAdapter constructs this
// trivially destructible C++ object exactly once before any callback uses it.
// All mappings/firmware are explicitly released on unsuccessful bring-up.
struct VideoDevice final {
    rx7900::reactos::HardwareState hardware{};
    rx7900::output::ConnectorCatalog connectors{};
    rx7900::display::ModeController modes{};
    rx7900::display::Framebuffer framebuffer{};
    rx7900::reactos::AsicBringup asic{};
    rx7900::reactos::GpuProcessService processes{};
    // IH6 service of the physical ASIC; armed only after a completed bring-up.
    rx7900::hardware::IrqService irq{};
    // Kernel-ring fence completion fed by the IH DPC; its native lock, events
    // and fallback/TDR poll timer. Initialized before the IH service is armed.
    rx7900::hardware::FenceManager fences{};
    rx7900::reactos::FenceNative fence_native{};
    // Thermal and RAS interrupt actions: fed by the IH DPC after the fence
    // manager, acted on by their own PASSIVE_LEVEL worker under device_lock.
    // Started with the interrupt sources of a completed bring-up.
    rx7900::reactos::GpuHealthNative health{};
    // The firmware's pre-OS scanout adopted as the single display mode
    // (PhysicalAsicInit only; nothing in the display engine is programmed).
    rx7900::reactos::PreosScanoutNative preos{};
    // VBE linear-mode fallback for a legacy (CSM) boot; scratch and report only
    // (VbeModeSet=1 with PhysicalAsicInit=1; the VBIOS programs the display).
    rx7900::reactos::VbeModesetNative vbe{};
    // DCN 3.2 hardware cursor of the adopted pipe (HardwareCursor=1 on top of
    // a running ASIC and adoption); serves the VideoPort pointer IOCTLs.
    rx7900::reactos::CursorNative cursor{};
#ifdef RX7900_DISPLAY_LINK
    // dcn-link ROOT HOOK (applied by dcn-stream): the DCN 3.2 link owner, bound
    // only with DisplayLinkOwner=1 on top of a completed PhysicalAsicInit=1
    // bring-up and pre-OS adoption; its own interpreter instance over the same
    // VBIOS and the epoch of the current binding (a reset rebinds with +1).
    rx7900::reactos::NativeDisplayLink display_link{};
    rx7900::hardware::AtomInterpreter display_atom{};
    uint64_t display_link_epoch = 0;
#endif
#ifdef RX7900_DISPLAY_STREAM
    // DCN 3.2 stream/plane owner (DisplayModeSet=1 on top of the link owner):
    // serves the mode IOCTLs; nothing is programmed until a SET_CURRENT_MODE
    // of a mode other than the running pre-OS one.
    rx7900::reactos::NativeDisplayStream display_stream{};
    uint32_t display_stream_generation = 0;
    // Device-owned re-validation workspace of a committed scanout.
    rx7900::display::PreosScanoutRegisters display_snapshot{};
    rx7900::display::AdoptedScanout display_decoded{};
    rx7900::display::PreosReport display_report{};
#endif
#ifdef RX7900_RADEON2D_VIDEOPORT
    // Private Radeon 2D IOCTLs over the SDMA blit backend, drawing into the
    // adopted scanout (Radeon2D=1 on top of a running ASIC and adoption).
    rx7900::reactos::radeon2d::NativeDispatcher radeon2d{};
    rx7900::hardware::Radeon2dBackend radeon2d_backend{};
    bool radeon2d_enabled = false;
    // Suspended across a power transition: sessions, staging and evicted
    // offscreen surfaces are retained for the resume (Dispatcher::suspend).
    bool radeon2d_parked = false;
    // Radeon2DCompute=1: the backend also drives the GFX11 compute raster
    // engine on kernel compute ring 1 (raster IOCTLs 0xa10..0xa16).
    bool radeon2d_compute = false;
    // The owners the backend was started with (their objects survive a
    // destructive reset; the restarted backend binds the same ones).
    rx7900::hardware::Radeon2dBackendConfig radeon2d_backend_config{};
    // Destructive (mode-1) reset recovery of 2D: bounded, sticky off after a
    // failure. The backend is AsicBringup's surface_owner while 2D runs.
    rx7900::hardware::Radeon2dRecovery radeon2d_recovery{};
    // Device-owned re-validation workspace of the adopted scanout, kept off
    // the kernel stack of the IOCTL or power callback that runs a recovery.
    rx7900::display::PreosScanoutRegisters recovery_snapshot{};
    rx7900::display::AdoptedScanout recovery_scanout{};
    rx7900::display::PreosReport recovery_report{};
    // Recovery worker: a driver-owned system thread (fresh kernel stack) that
    // runs Radeon2dRecovery; detecting paths only request through the gate and
    // signal recovery_wake. Started in HwInitialize once 2D runs.
    rx7900::hardware::Radeon2dRecoveryGate recovery_gate{};
    KEVENT recovery_wake;
    // Referenced thread object (the handle is closed at start); joined with
    // KeWaitForSingleObject and dereferenced by stop_recovery_worker.
    PVOID recovery_thread = nullptr;
    bool recovery_started = false;
    // Set (interlocked, no lock held) by the process service when an exited
    // session's MES removal, or a reconstruction's teardown, stayed uncertain;
    // the worker turns it into a forced recovery request under device_lock.
    volatile LONG process_recovery_requested = 0;
#endif
    // Device serialization of every HwStartIO, HwSetPowerState and the recovery
    // worker, taken before the shared GpuProcessGuard (lock order: device_lock,
    // process service mutex, Radeon 2D dispatcher mutex). Initialized in
    // HwFindAdapter at PASSIVE_LEVEL; never taken at raised IRQL.
    KMUTEX device_lock;
    bool device_lock_ready = false;
    // The VideoPort extension handle (VideoPortSynchronizeExecution context of
    // the recovery worker); set in HwInitialize.
    PVOID extension = nullptr;
    // A recovery reset retired the process service; its rearm
    // (gpu_process_initialize takes the process registry lock before the
    // service lock) runs after the callback released the service lock.
    bool process_rearm_pending = false;
    // The IrqService was detached from the IH rings (on_ih_stopping) and is
    // waiting for the next IH start (arm_interrupts_early resumes it).
    bool irq_parked = false;
    // Fence-interrupt sources of the kernel rings the first bring-up mapped;
    // resume and a recovery restart map the same rings (same discovery and
    // firmware), so the plan is computed once, off later callback stacks.
    rx7900::hardware::IrqSourcePlan irq_plan{0, 0, 0};
    bool resources_owned = false;
    // Set once the IH service is armed: the ISR, DPC and IH rings (inside the
    // BAR0 window) then depend on every mapping for the device lifetime.
    bool interrupts_owned = false;
    bool initialized = false;
    // A completed PhysicalAsicInit bring-up: the power hooks then suspend and
    // resume the ASIC; otherwise they only record the requested state.
    bool asic_running = false;
    // HwSetPowerState bookkeeping: the adapter is in D3 between a completed
    // suspend and a completed resume; a failed transition leaves the ASIC
    // unproven and every later request is refused.
    bool low_power = false;
    bool power_failed = false;
    ULONG power_state = VideoPowerOn;
};

constexpr size_t extension_bytes = sizeof(VideoDevice) + alignof(VideoDevice) - 1u;
static_assert(extension_bytes <= MAXULONG, "VideoPort extension length fits ULONG");
static_assert((alignof(VideoDevice) & (alignof(VideoDevice) - 1u)) == 0,
              "Native object alignment is a power of two");

// VideoPort guarantees pointer alignment, which can be weaker than a C++ type's
// 64-bit member alignment on i386. DriverEntry reserves the full padded extent.
// The OS owns this nonnull allocation and guarantees enough accessible bytes, so
// rounding inside it cannot wrap a valid native address. All VideoPort APIs keep
// receiving the original opaque extension, never this aligned inner pointer.
VideoDevice* device_at(void* extension) noexcept {
    return rx7900::reactos::aligned_video_extension<VideoDevice>(extension);
}

VP_STATUS video_status(Status status) noexcept {
    switch (status) {
    case Status::success: return NO_ERROR;
    case Status::mode_not_found:
    case Status::invalid_request:
    case Status::invalid_timing:
    case Status::unsupported_timing:
    case Status::invalid_framebuffer:
    case Status::arithmetic_overflow: return ERROR_INVALID_PARAMETER;
    case Status::busy: return ERROR_BUSY;
    case Status::hardware_not_ready:
        return ERROR_DEV_NOT_EXIST;
    case Status::state_unknown:
    default: return ERROR_DEVICE_REINITIALIZATION_NEEDED;
    }
}

// RAII holder of VideoDevice::device_lock (PASSIVE_LEVEL; recursive KMUTEX).
class DeviceGuard final {
public:
    explicit DeviceGuard(VideoDevice& device) noexcept
        : lock_(device.device_lock_ready ? &device.device_lock : nullptr) {
        if (lock_ != nullptr) KeWaitForSingleObject(lock_, Executive, KernelMode, FALSE, nullptr);
    }
    ~DeviceGuard() noexcept { if (lock_ != nullptr) KeReleaseMutex(lock_, FALSE); }
    DeviceGuard(const DeviceGuard&) = delete;
    DeviceGuard& operator=(const DeviceGuard&) = delete;
private:
    KMUTEX* lock_;
};

#ifdef RX7900_RADEON2D_VIDEOPORT
void stop_recovery_worker(VideoDevice& device) noexcept;
#endif

void release_device(VideoDevice& device, void* extension) noexcept {
    device.initialized = false;
    device.modes.invalidate_current();
#ifdef RX7900_RADEON2D_VIDEOPORT
    // Joined before any owner it may touch is released.
    stop_recovery_worker(device);
#endif
    // Its actions reach the process service and the SMU: joined first too.
    rx7900::reactos::gpu_health_stop(device.health);

    if (!rx7900::reactos::gpu_process_shutdown(device.processes)) {
        // An uncertain MES removal keeps process/page-table/backing and native
        // device storage resident; release must never infer hardware idleness.
        return;
    }

#ifdef RX7900_DISPLAY_STREAM
    // The stream owner disables its stream and frees its VRAM surfaces first;
    // an uncertain disable keeps every mapping (the engine may still fetch).
    if (!device.display_stream.retire()) {
        return;
    }
#endif
#ifdef RX7900_DISPLAY_LINK
    // dcn-link ROOT HOOK: the link owner retires before the hardware it maps.
    if (!device.display_link.retire()) {
        return;
    }
#endif
    // VideoPort keeps the interrupt connected after a failed HwInitialize and the
    // running ASIC keeps raising IH vectors, so an armed service retains its mappings.
    if (device.resources_owned && !device.interrupts_owned) {
        // CPU views of VRAM pool allocations; GPU-side ownership is unaffected.
        device.asic.aperture.release();
        rx7900::reactos::release(device.hardware, extension);
        device.resources_owned = false;
    }
}

// VideoPort registry callback, PASSIVE_LEVEL. Copies one REG_DWORD only.
VP_STATUS NTAPI read_dword_parameter(PVOID, PVOID context, PWSTR, PVOID data, ULONG length) {
    if (context == nullptr || data == nullptr || length != sizeof(ULONG)) {
        return ERROR_INVALID_PARAMETER;
    }

    VideoPortMoveMemory(context, data, sizeof(ULONG));
    return NO_ERROR;
}

// Physical ASIC initialization runs only when the service key explicitly sets
// PhysicalAsicInit=1. It loads GPU firmware through PSP; it is never implied by
// installation, PCI identity or a successful ROM parse.
bool physical_init_requested(PVOID extension) noexcept {
    ULONG value = 0;
    wchar_t name[] = L"PhysicalAsicInit";
    return VideoPortGetRegistryParameters(extension, name, FALSE, read_dword_parameter, &value) ==
           NO_ERROR && value == 1;
}

// AtomRomOverride=1 restricts VBIOS acquisition to the administrator rx7900.rom.
bool atom_rom_override(PVOID extension) noexcept {
    ULONG value = 0;
    wchar_t name[] = L"AtomRomOverride";
    return VideoPortGetRegistryParameters(extension, name, FALSE, read_dword_parameter, &value) ==
           NO_ERROR && value == 1;
}

// VbeModeSet=1 lets HwInitialize ask the VBIOS (INT 10h, VBE 4F02 with a linear
// framebuffer) for a linear mode when the pre-OS capture finds the legacy VGA
// engine active. Only with PhysicalAsicInit=1; never implied by either alone.
bool vbe_modeset_requested(PVOID extension) noexcept {
    ULONG value = 0;
    wchar_t name[] = L"VbeModeSet";
    return VideoPortGetRegistryParameters(extension, name, FALSE, read_dword_parameter, &value) ==
           NO_ERROR && value == 1;
}

#ifdef RX7900_RADEON2D_VIDEOPORT
// Radeon2D=1 enables the private 2D IOCTLs; never implied by PhysicalAsicInit.
bool radeon2d_requested(PVOID extension) noexcept {
    ULONG value = 0;
    wchar_t name[] = L"Radeon2D";
    return VideoPortGetRegistryParameters(extension, name, FALSE, read_dword_parameter, &value) ==
           NO_ERROR && value == 1;
}

constexpr uint32_t radeon2d_staging_bytes = 1u << 20;

// Radeon2DCompute=1 (only with Radeon2D=1) adds the compute raster engine.
bool radeon2d_compute_requested(PVOID extension) noexcept {
    ULONG value = 0;
    wchar_t name[] = L"Radeon2DCompute";
    return VideoPortGetRegistryParameters(extension, name, FALSE, read_dword_parameter, &value) ==
           NO_ERROR && value == 1;
}

// The kernel compute queues (AsicInit::kernel_queues, submission enabled by
// enable_kernel_submission on the same FenceManager) serve compute ring
// radeon2d_compute_ring; every compute submission runs under the radeon2d
// dispatcher mutex, and nothing else in this miniport submits to that ring.
rx7900::hardware::Radeon2dBackendConfig radeon2d_config(VideoDevice& device) noexcept {
    auto& asic = device.asic.asic;
    return rx7900::hardware::radeon2d_default_config(asic.sdma_rings(), device.fences, asic.gtt_manager(),
                                                     device.asic.vram_allocator, asic.vram_location().mc_start,
                                                     device.radeon2d_compute ? &asic.kernel_queues() : nullptr);
}

rx7900::reactos::radeon2d::EngineSurface radeon2d_primary(const VideoDevice& device) noexcept {
    const auto& scanout = device.preos.scanout;
    return {scanout.mc_address, scanout.timing.width, scanout.timing.height, scanout.stride_bytes,
            rx7900::reactos::radeon2d::abi::format_xrgb8888};
}

// PASSIVE_LEVEL, after kernel submission and the interrupt sources are up.
// Configures the dispatcher on the adopted scanout; any failure leaves 2D off.
void start_radeon2d(VideoDevice& device) noexcept {
    namespace r2d = rx7900::reactos::radeon2d;

    // A failed recovery keeps 2D off for the device lifetime.
    if (device.radeon2d_recovery.disabled()) {
        return;
    }

    device.radeon2d_backend_config = radeon2d_config(device);
    const auto status = device.radeon2d_backend.initialize(device.radeon2d_backend_config);

    if (status != rx7900::hardware::Radeon2dStatus::ready) {
        char message[] = "RX7900: Radeon 2D backend unavailable status=%u\n";
        VideoPortDebugPrint(Error, message, static_cast<unsigned>(status));
        return;
    }

    if (device.radeon2d_compute) {
        const auto report = device.radeon2d_backend.report();
        char message[] = "RX7900: Radeon 2D compute raster engine %s status=%u\n";
        char ready[] = "ready";
        char off[] = "unavailable";
        VideoPortDebugPrint(Error, message, report.compute ? ready : off,
                            static_cast<unsigned>(report.compute_status));
    }

    const r2d::EngineSurface primary = radeon2d_primary(device);

    if (!r2d::native_configure(device.radeon2d, device.radeon2d_backend.table(), primary,
                               radeon2d_staging_bytes)) {
        device.radeon2d_backend.shutdown();
        char message[] = "RX7900: Radeon 2D dispatcher refused the scanout\n";
        VideoPortDebugPrint(Error, message);
        return;
    }

    device.radeon2d_enabled = true;
    // The confirmed-reset window of mode1_reset_asic parks the backend.
    device.asic.surface_owner = &device.radeon2d_backend;
    device.asic.surface_reset_retire = rx7900::hardware::Radeon2dRecovery::retire_after_reset;
}

void unregister_radeon2d(VideoDevice& device) noexcept {
    device.asic.surface_owner = nullptr;
    device.asic.surface_reset_retire = nullptr;
}

void request_recovery(VideoDevice& device, bool forced) noexcept;

// Before a power transition (amdgpu_device_evict_resources, then the IP
// suspend): evict every offscreen surface to GTT and park the backend, keeping
// sessions, staging and handles for resume_radeon2d. When idleness or an
// eviction cannot be proven the transition is refused (false) and 2D keeps
// running: after a fault the recovery worker is asked for a destructive
// recovery, otherwise dispatch resumes in place. Never the full stop: it would free staging the display
// DLL still maps.
bool suspend_radeon2d(VideoDevice& device) noexcept {
    namespace r2d = rx7900::reactos::radeon2d;

    if (!device.radeon2d_enabled) {
        return true;
    }

    if (r2d::native_suspend(device.radeon2d) &&
        device.radeon2d_backend.suspend() == rx7900::hardware::Radeon2dStatus::suspended) {
        device.radeon2d_enabled = false;
        device.radeon2d_parked = true;
        return true;
    }

    if (rx7900::hardware::Radeon2dRecovery::needed(device.radeon2d_backend, device.fences)) {
        char message[] = "RX7900: Radeon 2D eviction faulted, destructive recovery requested\n";
        VideoPortDebugPrint(Error, message);
        request_recovery(device, false);
    } else if (device.radeon2d.dispatcher.suspended() &&
               !r2d::native_resume(device.radeon2d, device.radeon2d_backend.table(), radeon2d_primary(device))) {
        char message[] = "RX7900: Radeon 2D could not resume dispatch after a refused suspend\n";
        VideoPortDebugPrint(Error, message);
    }

    return false;
}

// After a completed resume (or an aborted suspend): restart the parked backend
// on the same memory owners and resume the retained sessions; if that fails,
// drop them (the dispatcher frees their surfaces and staging) and start anew.
void resume_radeon2d(VideoDevice& device, PVOID extension) noexcept {
    namespace r2d = rx7900::reactos::radeon2d;

    if (device.radeon2d_parked) {
        device.radeon2d_parked = false;

        if (device.preos.adopted &&
            device.radeon2d_backend.initialize(device.radeon2d_backend_config) ==
                rx7900::hardware::Radeon2dStatus::ready &&
            r2d::native_resume(device.radeon2d, device.radeon2d_backend.table(), radeon2d_primary(device))) {
            device.radeon2d_enabled = true;
            return;
        }

        char message[] = "RX7900: Radeon 2D resume failed, sessions dropped\n";
        VideoPortDebugPrint(Error, message);
        r2d::native_shutdown(device.radeon2d);

        if (device.radeon2d_backend.shutdown() != rx7900::hardware::Radeon2dStatus::released) {
            return; // Allocations stay quarantined; 2D stays off.
        }

        unregister_radeon2d(device);
    }

    if (device.preos.adopted && radeon2d_requested(extension)) {
        start_radeon2d(device);
    }
}
#endif

void report_bringup(const rx7900::reactos::AsicBringup& asic,
                    rx7900::reactos::BringupResult result) noexcept {
    const auto report = asic.asic.report();
    char message[] = "RX7900: ASIC bring-up result=%u step=%u status=%u boot=%u ring=%u "
                     "smu=%u ucode=%u failed_ucode=%u ifwi=%u training=%u scpm=%u file=0x%08x\n";
    VideoPortDebugPrint(Error, message, static_cast<unsigned>(result),
                        static_cast<unsigned>(report.step), static_cast<unsigned>(report.status),
                        static_cast<unsigned>(report.boot), static_cast<unsigned>(report.ring),
                        static_cast<unsigned>(report.smu), report.loaded_ucode,
                        static_cast<unsigned>(report.failed_ucode_id),
                        asic.discovery_report.ifwi_ready ? 1u : 0u,
                        report.training_requested ? 1u : 0u, report.scpm_enabled ? 1u : 0u,
                        static_cast<unsigned>(asic.last_file_status));
}

// RegisterIo over the resident BAR5 mapping for the interrupt service. Callable at
// any IRQL; the service itself never waits.
bool irq_mmio_read(void* context, uint32_t offset, uint32_t& value) noexcept {
    return rx7900::reactos::read_register(*static_cast<rx7900::reactos::HardwareState*>(context),
                                          offset, value);
}

bool irq_mmio_write(void* context, uint32_t offset, uint32_t value) noexcept {
    return rx7900::reactos::write_register(*static_cast<rx7900::reactos::HardwareState*>(context),
                                           offset, value);
}

void irq_stall(void*, uint32_t microseconds) noexcept {
    VideoPortStallExecution(microseconds);
}

struct SynchronizedCall final {
    bool (*routine)(void*) noexcept;
    void* argument;
};

// KeSynchronizeExecution routine: runs at DIRQL holding the interrupt spinlock.
BOOLEAN NTAPI run_synchronized(PVOID context) {
    const auto& call = *static_cast<const SynchronizedCall*>(context);
    return call.routine(call.argument) ? TRUE : FALSE;
}

// IrqSynchronize::run. VpMediumPriority excludes HwInterrupt through the
// connected interrupt object; the call and its argument live on this stack.
bool synchronize_irq(void* extension, bool (*routine)(void*) noexcept, void* argument) noexcept {
    SynchronizedCall call{routine, argument};
    return VideoPortSynchronizeExecution(extension, VpMediumPriority, run_synchronized, &call) !=
           FALSE;
}

BOOLEAN NTAPI arm_interrupts(PVOID context) {
    static_cast<rx7900::hardware::IrqService*>(context)->arm_locked();
    return TRUE;
}

BOOLEAN NTAPI disarm_interrupts(PVOID context) {
    static_cast<rx7900::hardware::IrqService*>(context)->disarm_locked();
    return TRUE;
}

// Fence-interrupt sources of the kernel rings AsicInit actually mapped/started
// (amdgpu_fence_driver_hw_init enables one per initialized ring). Computed once
// in HwInitialize; kept out of line so its report copies (about 1.9 KiB) never
// join another callback's frame.
#if defined(_MSC_VER)
__declspec(noinline)
#else
__attribute__((noinline))
#endif
rx7900::hardware::IrqSourcePlan source_plan(const rx7900::hardware::AsicInit& asic) noexcept {
    using rx7900::hardware::Mes11QueueType;
    const auto queues = asic.queues_report();
    const auto sdma = asic.sdma_report();
    rx7900::hardware::IrqSourcePlan plan{0, 0, 0};

    for (const auto& ring : queues.rings) {
        if (!ring.mapped) {
            continue;
        }

        if (ring.type == Mes11QueueType::gfx && ring.me == 0 &&
            ring.pipe < rx7900::hardware::irq_gfx_pipes) {
            plan.gfx_pipes |= 1u << ring.pipe;
        } else if (ring.type == Mes11QueueType::compute && ring.me == 1 &&
                   ring.pipe < rx7900::hardware::irq_mec_pipes) {
            plan.compute_pipes |= 1u << ring.pipe;
        }
    }

    for (uint32_t instance = 0; instance < rx7900::hardware::sdma6_instance_count; ++instance) {
        if (sdma.ring_ready[instance]) {
            plan.sdma_instances |= 1u << instance;
        }
    }

    return plan;
}

// PASSIVE_LEVEL in HwInitialize, only after run_asic_bringup completed. Attaches
// the IH6 service, arms the ISR under the interrupt lock, then enables the
// sources Linux enables at fence hw_init and late_init. Returns whether armed.
// Linux amdgpu_irq_init order: the handler exists before IH hw_init completes, so
// AsicInit calls this right after IH6 is enabled (InterruptReady). Attach to the
// enabled IH owner and arm the ISR; source enables follow after bring-up.
bool arm_interrupts_early(VideoDevice& device, PVOID extension) noexcept {
    using namespace rx7900::hardware;
    auto& hardware = device.hardware;

    // VideoPort connects HwInterrupt only for an assigned interrupt resource.
    if (!hardware.interrupt_assigned) {
        char message[] = "RX7900: no interrupt resource; IH service not armed\n";
        VideoPortDebugPrint(Error, message);
        return false;
    }

    // Ih6::set_rptr acknowledges through the IH doorbell callback whose context is
    // the AsicBringup; run_asic_bringup keeps its hardware reference for the device.
    if (device.asic.hardware == nullptr) {
        char message[] = "RX7900: IH doorbell context unavailable; IH service not armed\n";
        VideoPortDebugPrint(Error, message);
        return false;
    }

    // The DPC hands every drain to the fence manager, so it exists before arming.
    if (!device.fence_native.initialized &&
        !rx7900::reactos::fence_native_initialize(device.fence_native, device.fences)) {
        char message[] = "RX7900: fence manager unavailable; IH service not armed\n";
        VideoPortDebugPrint(Error, message);
        return false;
    }

    // Resume: the suspended service rebinds the resumed (zeroed) rings and keeps
    // its counters, which the FenceManager consumer compares against.
    // irq_parked mirrors IrqStatus::suspended without copying the report.
    if (device.irq_parked) {
        if (device.irq.resume(device.asic.asic.interrupts()) != IrqStatus::attached) {
            char message[] = "RX7900: IH service resume failed\n";
            VideoPortDebugPrint(Error, message);
            return false;
        }

        device.irq_parked = false;
        VideoPortSynchronizeExecution(extension, VpMediumPriority, arm_interrupts, &device.irq);
        return true;
    }

    IrqRegisters registers;

    if (validate_irq_discovery(device.asic.discovery.bytes(), hardware.register_bytes,
                               registers) != DiscoveryStatus::validated ||
        device.irq.attach(device.asic.asic.interrupts(),
                          {&hardware, hardware.register_bytes, irq_mmio_read, irq_mmio_write,
                           irq_stall},
                          registers) != IrqStatus::attached) {
        char message[] = "RX7900: IH service attach failed\n";
        VideoPortDebugPrint(Error, message);
        return false;
    }

    VideoPortSynchronizeExecution(extension, VpMediumPriority, arm_interrupts, &device.irq);
    device.interrupts_owned = true;
    return true;
}

struct InterruptHookContext final {
    VideoDevice* device;
    PVOID extension;
};

bool on_ih_ready(void* context, rx7900::hardware::Ih6&) noexcept {
    const auto& hook = *static_cast<const InterruptHookContext*>(context);
    return arm_interrupts_early(*hook.device, hook.extension);
}

// SuspendPlatform::ih_stopping, PASSIVE_LEVEL, right before ih_v6_0_suspend: the
// Linux handler goes quiet at suspend_noirq. Disarm under the interrupt lock,
// let queued DPCs finish, drain what the ISR already acknowledged into the fence
// manager, then detach the service from the rings (counters kept for resume).
// The device's IrqService drain consumer (DPC, DISPATCH_LEVEL): kernel-ring
// fences and faults first (amdgpu_fence_process), then the thermal/RAS mailbox.
void consume_interrupts(void* context, const rx7900::hardware::IrqDispatchView& view) noexcept {
    auto& device = *static_cast<VideoDevice*>(context);
    const auto fences = device.fences.consumer();
    fences.consume(fences.context, view);
    rx7900::reactos::gpu_health_consume(device.health, view);
}

rx7900::hardware::IrqConsumer interrupt_consumer(VideoDevice& device) noexcept {
    return {&device, consume_interrupts};
}

bool on_ih_stopping(void* context) noexcept {
    using rx7900::hardware::IrqStatus;
    const auto& hook = *static_cast<const InterruptHookContext*>(context);
    auto& device = *hook.device;

    if (!device.interrupts_owned) {
        return true;
    }

    VideoPortSynchronizeExecution(hook.extension, VpMediumPriority, disarm_interrupts, &device.irq);
    KeFlushQueuedDpcs();

    // Each drain dispatches at most one bounded batch per ring; the ring holds
    // at most ring_bytes / 32 acknowledged vectors.
    for (uint32_t round = 0; round < rx7900::hardware::ih6_default_ring_bytes / 32; ++round) {
        const auto status = device.irq.suspend();

        if (status == IrqStatus::suspended) {
            device.irq_parked = true;
            return true;
        }

        if (status != IrqStatus::busy) {
            return false;
        }

        device.irq.drain({hook.extension, synchronize_irq}, interrupt_consumer(device));
    }

    return false;
}

void start_gpu_health(VideoDevice& device) noexcept;

// Fence-interrupt and fault sources plus the fence fallback poll, after a
// completed bring-up or resume (power loss cleared every source enable).
void enable_sources_and_poll(VideoDevice& device) noexcept {
    const auto status = device.irq.enable_sources(device.irq_plan);
    char message[] = "RX7900: IH service armed, interrupt sources status=%u\n";
    VideoPortDebugPrint(Error, message, static_cast<unsigned>(status));
    // Fence fallback timer and job timeout poll; interrupts_owned keeps this
    // device storage resident, so the timer runs for the device lifetime.
    rx7900::reactos::fence_native_start_poll(device.fence_native);
    // Thermal/RAS actions over this incarnation's SMU, NBIO and RAS support.
    start_gpu_health(device);
}

// After a completed bring-up: the CP/SDMA/VM fault sources Linux enables in
// fence hw_init, gmc late_init and gfx late_init.
void enable_interrupt_sources(VideoDevice& device) noexcept {
    if (!device.interrupts_owned) {
        return;
    }

    device.irq_plan = source_plan(device.asic.asic);
    enable_sources_and_poll(device);

    // Kernel-ring submission with interrupt-signalled fences (amdgpu_ib_schedule).
    const auto submission = device.asic.asic.enable_kernel_submission(
        device.fences, rx7900::reactos::fence_native_wptr_store());
    char submit_message[] = "RX7900: kernel-ring submission status=%u\n";
    VideoPortDebugPrint(Error, submit_message, static_cast<unsigned>(submission));
}

#ifdef RX7900_DISPLAY_LINK
// dcn-link ROOT HOOK (applied by dcn-stream). DisplayLinkOwner=1 in the service
// key; never implied by PhysicalAsicInit, installation or a parsed ROM.
bool display_link_requested(PVOID extension) noexcept {
    ULONG value = 0;
    wchar_t name[sizeof(rx7900::reactos::display_link_registry_value) / sizeof(wchar_t)];
    for (size_t index = 0; index < sizeof(name) / sizeof(name[0]); ++index) {
        name[index] = rx7900::reactos::display_link_registry_value[index];
    }
    return VideoPortGetRegistryParameters(extension, name, FALSE, read_dword_parameter, &value) ==
           NO_ERROR && value == 1;
}

// PASSIVE_LEVEL, after a completed bring-up, pre-OS adoption and the connector
// parse (or a proven reset and restart): binds the link owner for epoch with
// the discovered MP1 base, a VBIOS interpreter, the dce_info crystal and the
// adopted DP DTO modulo as the boot DPREFCLK. No DMCUB firmware is started.
bool bind_display_link(VideoDevice& device, uint64_t epoch) noexcept {
    namespace hw = rx7900::hardware;
    const rx7900::ByteView rom = rx7900::reactos::atom_rom(device.hardware);
    hw::DiscoveredIp mp1{};
    hw::AtomVramUsage usage{};
    uint32_t xtal = 0;
    if (hw::discover_ip(device.asic.discovery.bytes(), {hw::discovery_mp1_hardware_id, 0, 13, 0, 0, 1}, mp1) !=
            hw::DiscoveryStatus::validated ||
        !rx7900::display::read_dce_refclk_khz(rom, xtal)) {
        return false;
    }
    hw::parse_atom_vram_usage(rom, usage);
    if (device.display_atom.bind(rom, hw::atom_allocate_fb_scratch(usage).scratch_bytes) != hw::AtomStatus::completed) {
        return false;
    }
    const auto& scanout = device.preos.scanout;
    rx7900::reactos::NativeDisplayLinkInputs inputs{};
    inputs.hardware = &device.hardware;
    inputs.registers = &device.preos.registers;
    inputs.mp1_segment0 = mp1.segment(0);
    inputs.catalog = &device.connectors;
    inputs.atom = &device.display_atom;
    inputs.xtalin_khz = xtal;
    inputs.boot_dprefclk_khz = scanout.clock_source == rx7900::display::PreosClockSource::dp_dto &&
                                       scanout.otg < rx7900::display::preos_pipe_count ?
                                   device.preos.snapshot.otg[scanout.otg].dto_modulo / 1000u : 0u;
    inputs.epoch = epoch;
    const auto result = device.display_link.bind(inputs);
    char message[] = "RX7900: display link owner bind result=%u epoch=%u routes=%u\n";
    VideoPortDebugPrint(Error, message, static_cast<unsigned>(result), static_cast<unsigned>(epoch),
                        static_cast<unsigned>(device.display_link.owner().routes().validated_count));
    if (result != rx7900::output::LinkResult::success) {
        return false;
    }
    device.display_link_epoch = epoch;
    return true;
}
#endif

#ifdef RX7900_DISPLAY_STREAM
// DisplayModeSet=1 in the service key (only with DisplayLinkOwner=1).
bool display_mode_set_requested(PVOID extension) noexcept {
    ULONG value = 0;
    wchar_t name[sizeof(rx7900::reactos::display_mode_set_registry_value) / sizeof(wchar_t)];
    for (size_t index = 0; index < sizeof(name) / sizeof(name[0]); ++index) {
        name[index] = rx7900::reactos::display_mode_set_registry_value[index];
    }
    return VideoPortGetRegistryParameters(extension, name, FALSE, read_dword_parameter, &value) ==
           NO_ERROR && value == 1;
}

void start_display_stream(VideoDevice& device, PVOID extension) noexcept {
    const rx7900::reactos::NativeDisplayStreamInputs inputs{&device.hardware, &device.asic, &device.preos,
                                                            &device.display_link, extension,
                                                            rx7900::reactos::atom_rom(device.hardware),
                                                            device.display_link_epoch};
    const auto result = device.display_stream.start(inputs);
    device.display_stream_generation = device.display_stream.generation();
    char message[] = "RX7900: display stream owner start result=%u modes=%u\n";
    VideoPortDebugPrint(Error, message, static_cast<unsigned>(result),
                        static_cast<unsigned>(device.display_stream.mode_count()));
}

// After a committed mode change the consumers of device.preos.scanout follow
// the new surface: a re-validated DCN snapshot of the committed pipe becomes
// the adopted scanout (cursor resume, Radeon 2D primary, recovery scanout
// check) and the GL present target is reconfigured. Radeon 2D is moved to the
// new primary here only outside a recovery (which restores it itself).
void retarget_display(VideoDevice& device, bool move_radeon2d) noexcept {
    const uint32_t generation = device.display_stream.generation();
    if (generation == device.display_stream_generation) {
        return;
    }
    device.display_stream_generation = generation;
    rx7900::display::AdoptedScanout expected{};
    if (!device.display_stream.scanout(expected)) {
        char message[] = "RX7900: display stream blank; scanout consumers keep the previous surface\n";
        VideoPortDebugPrint(Error, message);
        return;
    }
    const rx7900::hardware::RegisterIo io{&device.hardware, device.hardware.register_bytes, irq_mmio_read,
                                          irq_mmio_write, irq_stall};
    const auto& decoded = device.display_decoded;
    const bool revalidated =
        rx7900::display::read_preos_scanout(io, device.preos.registers, device.display_snapshot) &&
        rx7900::display::decode_preos_scanout(device.display_snapshot, device.preos.vram, device.preos.visible_bytes,
                                              device.display_decoded, device.display_report) ==
            rx7900::display::PreosStatus::adopted &&
        decoded.otg == expected.otg && decoded.pipe == expected.pipe && decoded.mc_address == expected.mc_address &&
        decoded.stride_bytes == expected.stride_bytes && decoded.timing.width == expected.timing.width &&
        decoded.timing.height == expected.timing.height;
    device.preos.scanout = revalidated ? decoded : expected;
    char message[] = "RX7900: display stream committed %ux%u revalidated=%u\n";
    VideoPortDebugPrint(Error, message, static_cast<unsigned>(expected.timing.width),
                        static_cast<unsigned>(expected.timing.height), revalidated ? 1u : 0u);
    const auto& scanout = device.preos.scanout;
    const rx7900::reactos::GraphicsScanout target{scanout.mc_address, scanout.timing.width, scanout.timing.height,
                                                  scanout.stride_bytes};
    if (device.asic_running && !rx7900::reactos::gpu_process_configure_scanout(device.processes, &target)) {
        char refused[] = "RX7900: GL present scanout refused after mode change\n";
        VideoPortDebugPrint(Error, refused);
    }
#ifdef RX7900_RADEON2D_VIDEOPORT
    namespace r2d = rx7900::reactos::radeon2d;
    if (move_radeon2d && device.radeon2d_enabled &&
        (!r2d::native_suspend(device.radeon2d) ||
         !r2d::native_resume(device.radeon2d, device.radeon2d_backend.table(), radeon2d_primary(device)))) {
        char moved[] = "RX7900: Radeon 2D could not move to the new primary\n";
        VideoPortDebugPrint(Error, moved);
    }
#else
    (void)move_radeon2d;
#endif
}
#endif

#ifdef RX7900_DISPLAY_LINK
// The display hardware was reset (mode-1 reset proof, also the one closing a
// suspend): the link owner and the stream owner drop every hardware
// assumption (dcn-link ROOT HOOK reset_proven).
void display_reset_proven(VideoDevice& device) noexcept {
#ifdef RX7900_DISPLAY_STREAM
    device.display_stream.reset_proven();
#endif
    if (device.display_link.bound()) {
        device.display_link.reset_proven();
        device.display_link.retire();
    }
}

// After the ASIC runs again: rebind the link owner (next epoch) and commit the
// current mode again (Linux dm_resume of the cached state). Failures leave the
// display off and are only reported.
void display_restart(VideoDevice& device, bool move_radeon2d) noexcept {
    if (device.display_link_epoch == 0 || device.display_link.bound() ||
        !bind_display_link(device, device.display_link_epoch + 1)) {
        return;
    }
#ifdef RX7900_DISPLAY_STREAM
    if (device.display_stream.active()) {
        const bool restored = device.display_stream.restore(device.display_link_epoch);
        char message[] = "RX7900: display stream restore after reset result=%u\n";
        VideoPortDebugPrint(Error, message, restored ? 1u : 0u);
        retarget_display(device, move_radeon2d);
    }
#else
    (void)move_radeon2d;
#endif
}
#endif

#ifdef RX7900_RADEON2D_VIDEOPORT
// Destructive (mode-1) reset recovery of physical 2D. Runs only on the recovery
// worker thread (PASSIVE_LEVEL, its own kernel stack) under device_lock and the
// shared GpuProcessGuard; never on an IOCTL caller's stack, never from the IH
// DPC or the fence poll DPC (those only mark rings hung in the FenceManager,
// which Radeon2dRecovery::needed reads). A Radeon 2D IOCTL that observes a
// fault, or a failed power transition, only requests it (request_recovery). The sequence and its failure semantics
// live in Radeon2dRecovery (src/hardware/radeon_2d_backend); the callbacks
// below bind it to this device's owners.
struct RecoveryContext final {
    VideoDevice* device;
    PVOID extension;
    InterruptHookContext hook;
};

VideoDevice& recovering_device(void* context) noexcept {
    return *static_cast<RecoveryContext*>(context)->device;
}

void recovery_display_quiesce(void* context) noexcept {
    rx7900::reactos::cursor_suspend(recovering_device(context).cursor);
#ifdef RX7900_DISPLAY_STREAM
    // amdgpu_dm dm_suspend in a GPU reset: commit zero streams before the reset.
    recovering_device(context).display_stream.suspend();
#endif
}

void recovery_display_resume(void* context) noexcept {
    auto& device = recovering_device(context);
    rx7900::reactos::cursor_resume(device.cursor, device.preos);
}

// The ISR goes quiet and the IrqService detaches from the IH rings before the
// release frees them; the fence poll stops before the rings detach. A refusal
// re-arms both: the (still unreset) ASIC keeps its interrupt service.
bool recovery_stop_interrupts(void* context) noexcept {
    auto& recovery = *static_cast<RecoveryContext*>(context);
    auto& device = *recovery.device;
    rx7900::reactos::fence_native_stop_poll(device.fence_native);

    if (!device.interrupts_owned || device.irq_parked || on_ih_stopping(&recovery.hook)) {
        return true;
    }

    VideoPortSynchronizeExecution(recovery.extension, VpMediumPriority, arm_interrupts, &device.irq);
    rx7900::reactos::fence_native_start_poll(device.fence_native);
    return false;
}

rx7900::hardware::AsicStatus recovery_reset_and_release(void* context) noexcept {
#ifdef RX7900_DISPLAY_LINK
    auto& device = recovering_device(context);
    const auto status = rx7900::reactos::mode1_reset_asic(device.asic);
    // dcn-link ROOT HOOK: the destructive-reset retire path.
    if (device.asic.asic.reset_proven()) {
        display_reset_proven(device);
    }
    return status;
#else
    return rx7900::reactos::mode1_reset_asic(recovering_device(context).asic);
#endif
}

bool recovery_reset_proven(void* context) noexcept {
    return recovering_device(context).asic.asic.reset_proven();
}

// Bring-up on the retained owners; interrupt_ready resumes the IrqService on
// the new IH rings. Then the sources Linux enables at fence hw_init/late_init,
// the fence poll and kernel-ring submission on the same FenceManager (which
// release_after_reset detached).
rx7900::hardware::AsicStatus recovery_restart(void* context) noexcept {
    using rx7900::hardware::AsicStatus;
    auto& recovery = *static_cast<RecoveryContext*>(context);
    auto& device = *recovery.device;
    const auto status = rx7900::reactos::restart_asic_after_reset(
        device.hardware, rx7900::reactos::atom_rom(device.hardware), {&recovery.hook, on_ih_ready}, device.asic);

    if (status != AsicStatus::completed) {
        return status;
    }

    enable_sources_and_poll(device);
#ifdef RX7900_DISPLAY_LINK
    const auto submission =
        device.asic.asic.enable_kernel_submission(device.fences, rx7900::reactos::fence_native_wptr_store());
    // Link owner rebound and the current mode committed again before the
    // recovery re-validates the scanout (Radeon 2D restores itself after it).
    display_restart(device, false);
    return submission;
#else
    return device.asic.asic.enable_kernel_submission(device.fences, rx7900::reactos::fence_native_wptr_store());
#endif
}

void recovery_rearm_services(void* context) noexcept {
    recovering_device(context).process_rearm_pending = true;
}

// The pre-OS same-scanout checks of finalize_preos_after_bringup against the
// restarted ASIC: unchanged MMHUB placement, and a fresh DCN snapshot that
// decodes to the adopted scanout. Register reads only.
bool recovery_scanout_matches(void* context) noexcept {
    auto& device = recovering_device(context);

    if (!device.preos.adopted) {
        return false;
    }

    const rx7900::hardware::VramLocation vram = device.asic.asic.vram_location();
    const rx7900::hardware::VramLocation& adopted = device.preos.vram;

    if (vram.mc_start != adopted.mc_start || vram.mc_end != adopted.mc_end || vram.bytes != adopted.bytes ||
        vram.physical_offset != adopted.physical_offset) {
        return false;
    }

    const rx7900::hardware::RegisterIo io{&device.hardware, device.hardware.register_bytes, irq_mmio_read,
                                          irq_mmio_write, irq_stall};
    return rx7900::display::read_preos_scanout(io, device.preos.registers, device.recovery_snapshot) &&
           rx7900::display::decode_preos_scanout(device.recovery_snapshot, vram, device.preos.visible_bytes,
                                                 device.recovery_scanout, device.recovery_report) ==
               rx7900::display::PreosStatus::adopted &&
           rx7900::display::same_preos_scanout(device.preos.scanout, device.recovery_scanout);
}

// A recovery is due for running or parked 2D (a power transition parks it).
bool radeon2d_recovery_due(VideoDevice& device) noexcept {
    return (device.radeon2d_enabled || device.radeon2d_parked) && device.asic_running &&
           !device.radeon2d_recovery.disabled() &&
           rx7900::hardware::Radeon2dRecovery::needed(device.radeon2d_backend, device.fences);
}

void report_recovery(const VideoDevice& device, rx7900::hardware::Radeon2dRecoveryResult result) noexcept {
    const auto report = device.radeon2d_recovery.report();
    const auto counters = device.radeon2d.dispatcher.counters();
    char message[] = "RX7900: Radeon 2D recovery result=%u reset=%u restart=%u backend=%u attempts=%u "
                     "recovered=%u failed=%u scanout_mismatches=%u restored=%u lost=%u revived=%u "
                     "disabled=%u quiescent=%u\n";
    VideoPortDebugPrint(Error, message, static_cast<unsigned>(result), static_cast<unsigned>(report.reset_status),
                        static_cast<unsigned>(report.restart_status), static_cast<unsigned>(report.backend_status),
                        report.attempts, report.recovered, report.failed, report.scanout_mismatches,
                        counters.reset_restored, counters.reset_lost, counters.revived_sessions,
                        report.disabled ? 1u : 0u, report.quiescent ? 1u : 0u);
}

// Recovery worker only: PASSIVE_LEVEL, device_lock and GpuProcessGuard held.
// The dispatcher mutex is held across the whole sequence (recursive KMUTEX: the
// process service's reset callbacks take their own mutex, held by this thread).
void recover_radeon2d(VideoDevice& device, PVOID extension) noexcept {
    using rx7900::hardware::Radeon2dRecoveryResult;

    if (VideoPortGetCurrentIrql() != PASSIVE_LEVEL || !device.radeon2d.initialized) {
        return;
    }

    RecoveryContext context{&device, extension, {&device, extension}};
    const rx7900::hardware::Radeon2dRecoveryPlatform platform{
        &context, recovery_display_quiesce, recovery_display_resume, recovery_stop_interrupts,
        recovery_reset_and_release, recovery_reset_proven, recovery_restart, recovery_rearm_services,
        recovery_scanout_matches};
    Radeon2dRecoveryResult result = Radeon2dRecoveryResult::disabled;
    {
        rx7900::reactos::radeon2d::NativeGuard guard(device.radeon2d);
        result = device.radeon2d_recovery.recover(device.radeon2d.dispatcher, device.radeon2d_backend,
                                                  device.radeon2d_backend_config, radeon2d_primary(device),
                                                  platform);
    }

    device.radeon2d_parked = false;
    device.radeon2d_enabled = result == Radeon2dRecoveryResult::recovered;

    switch (result) {
    case Radeon2dRecoveryResult::recovered:
    case Radeon2dRecoveryResult::scanout_mismatch:
    case Radeon2dRecoveryResult::backend_failed:
    case Radeon2dRecoveryResult::second_fault:
        // The ASIC itself runs again (only 2D stays off).
        break;
    case Radeon2dRecoveryResult::restart_failed:
        // Reset proven and released, but no new ASIC incarnation: nothing runs.
        device.asic_running = false;
        break;
    case Radeon2dRecoveryResult::reset_failed:
    case Radeon2dRecoveryResult::limit_reached:
        // A hung or unproven ASIC: no further power transitions.
        device.power_failed = true;
        break;
    default:
        break;
    }

    report_recovery(device, result);
}

// Detecting paths (a Radeon 2D IOCTL, a failed power transition), device_lock
// held: quiesce the dispatcher now (later requests answer not_ready, query says
// pending), mark unproven work uncertain, and wake the worker. Without a worker
// there is no safe place to reset: 2D is disabled (GPU state unproven).
void request_recovery(VideoDevice& device, bool forced) noexcept {
    namespace r2d = rx7900::reactos::radeon2d;

    if (!device.radeon2d.initialized || device.radeon2d_recovery.disabled()) {
        return;
    }

    if (!device.recovery_started) {
        {
            r2d::NativeGuard guard(device.radeon2d);
            device.radeon2d.dispatcher.disable(false);
        }

        device.radeon2d_enabled = false;
        char message[] = "RX7900: Radeon 2D fault without a recovery worker; 2D disabled\n";
        VideoPortDebugPrint(Error, message);
        return;
    }

    if (!device.recovery_gate.request(forced)) {
        return;
    }

    {
        r2d::NativeGuard guard(device.radeon2d);
        auto& dispatcher = device.radeon2d.dispatcher;

        if (dispatcher.configured() && !dispatcher.recovering() && !dispatcher.disabled()) {
            dispatcher.begin_recovery();
        }
    }

    KeSetEvent(&device.recovery_wake, IO_NO_INCREMENT, FALSE);
}

// Recovery worker, after it released the GpuProcessGuard: the process service's own
// confirmed-reset rearm (gpu_process_initialize on the restarted ASIC), which
// reconstructs the live process sessions after the 2D recovery and the AsicInit
// restart completed (order: reset window retires 2D then processes; restart;
// 2D restore; process reconstruction).
void rearm_processes_after_recovery(VideoDevice& device) noexcept {
    if (!device.process_rearm_pending) {
        return;
    }

    device.process_rearm_pending = false;

    if (!rx7900::reactos::gpu_process_initialize(device.processes, device.asic)) {
        char message[] = "RX7900: native process service not rearmed after the reset\n";
        VideoPortDebugPrint(Error, message);
    }
}

// Process service escalation (reaper thread or rearm, no process lock held):
// record the request and wake the worker; never blocks, never takes device_lock.
void request_process_recovery(void* context) noexcept {
    auto& device = *static_cast<VideoDevice*>(context);
    InterlockedExchange(&device.process_recovery_requested, 1);
    KeSetEvent(&device.recovery_wake, IO_NO_INCREMENT, FALSE);
}

// One wake of the worker. false: the worker must exit (teardown).
bool run_requested_recovery(VideoDevice& device) noexcept {
    DeviceGuard device_guard(device);

    if (device.recovery_gate.stopping()) {
        return false;
    }

    // A hung exited process session (MES REMOVE_QUEUE did not complete) has no
    // narrower recovery: MES itself is unresponsive. Force the destructive one.
    // In D3 the request stays recorded; the D0 transition wakes the worker.
    if (!device.low_power && InterlockedExchange(&device.process_recovery_requested, 0) != 0) {
        char message[] = "RX7900: process session removal uncertain, destructive recovery requested\n";
        VideoPortDebugPrint(Error, message);
        request_recovery(device, true);
    }

    {
        rx7900::reactos::GpuProcessGuard process_guard(device.processes);
        const bool due = device.asic_running && !device.radeon2d_recovery.disabled() &&
                         rx7900::hardware::Radeon2dRecovery::needed(device.radeon2d_backend, device.fences);

        if (device.recovery_gate.begin(device.low_power, due)) {
            recover_radeon2d(device, device.extension);
            device.recovery_gate.finish(device.radeon2d_recovery.disabled());
        }
    }

    // The process service rearms outside its own mutex (registry lock first).
    rearm_processes_after_recovery(device);
    return true;
}

// System thread routine (PASSIVE_LEVEL). device outlives the thread: teardown
// joins it (stop_recovery_worker) before releasing anything it touches.
VOID NTAPI recovery_worker(PVOID context) {
    auto& device = *static_cast<VideoDevice*>(context);

    for (;;) {
        KeWaitForSingleObject(&device.recovery_wake, Executive, KernelMode, FALSE, nullptr);

        if (!run_requested_recovery(device)) {
            break;
        }
    }

    PsTerminateSystemThread(STATUS_SUCCESS);
}

// HwInitialize, PASSIVE_LEVEL, after 2D started. false leaves recovery off (a
// later fault disables 2D instead); nothing is left to clean up. Out of line:
// HwInitialize's frame stays what it was.
#if defined(_MSC_VER)
__declspec(noinline)
#else
__attribute__((noinline))
#endif
bool start_recovery_worker(VideoDevice& device) noexcept {
    if (device.recovery_started || KeGetCurrentIrql() != PASSIVE_LEVEL) {
        return device.recovery_started;
    }

    KeInitializeEvent(&device.recovery_wake, SynchronizationEvent, FALSE);
    OBJECT_ATTRIBUTES attributes{};
    InitializeObjectAttributes(&attributes, nullptr, OBJ_KERNEL_HANDLE, nullptr, nullptr);
    HANDLE thread = nullptr;

    if (!NT_SUCCESS(PsCreateSystemThread(&thread, SYNCHRONIZE, &attributes, nullptr, nullptr, recovery_worker,
                                         &device))) {
        char message[] = "RX7900: Radeon 2D recovery worker unavailable\n";
        VideoPortDebugPrint(Error, message);
        return false;
    }

    PVOID object = nullptr;
    const NTSTATUS referenced = ObReferenceObjectByHandle(thread, SYNCHRONIZE, nullptr, KernelMode, &object,
                                                          nullptr);
    ZwClose(thread);

    if (!NT_SUCCESS(referenced)) {
        // Not joinable: tell it to exit at once. Its device storage stays
        // resident (the miniport is pinned once the ASIC runs), so the exiting
        // thread never touches freed memory.
        device.recovery_gate.stop();
        KeSetEvent(&device.recovery_wake, IO_NO_INCREMENT, FALSE);
        char message[] = "RX7900: Radeon 2D recovery worker not joinable; stopped\n";
        VideoPortDebugPrint(Error, message);
        return false;
    }

    device.recovery_thread = object;
    device.recovery_started = true;
    return true;
}

// Teardown, PASSIVE_LEVEL, device_lock not held: refuse new requests, let a
// running recovery finish (the worker holds device_lock meanwhile), wake the
// worker so it observes stopping, then join and close its handle.
void stop_recovery_worker(VideoDevice& device) noexcept {
    if (!device.recovery_started || KeGetCurrentIrql() != PASSIVE_LEVEL) {
        return;
    }

    {
        DeviceGuard guard(device);
        device.recovery_gate.stop();
    }

    // No process escalation targets the stopping worker any more.
    rx7900::reactos::gpu_process_set_recovery_hook(device.processes, nullptr, nullptr);
    KeSetEvent(&device.recovery_wake, IO_NO_INCREMENT, FALSE);
    KeWaitForSingleObject(device.recovery_thread, Executive, KernelMode, FALSE, nullptr);
    ObDereferenceObject(device.recovery_thread);
    device.recovery_thread = nullptr;
    device.recovery_started = false;
}
#endif

// ---- Thermal and RAS interrupt actions (GpuHealthHooks) ----

// The health worker's pass: device_lock, then the process guard (lock order of
// every HwStartIO); skipped while no running ASIC can act (D3, before
// asic_running, after teardown began), retried by the worker later.
bool health_run_locked(void* context, void (*body)(void*) noexcept, void* argument) noexcept {
    auto& device = *static_cast<VideoDevice*>(context);
    DeviceGuard device_guard(device);

    if (!device.asic_running || device.low_power || !device.interrupts_owned) {
        return false;
    }

    rx7900::reactos::GpuProcessGuard process_guard(device.processes);
    body(argument);
    return true;
}

// Critical temperature, or an unrecoverable RAS error without a recovery path:
// every later GPU request is refused (power_failed: the process, present,
// pointer and power paths check it), 2D stops, and no destructive recovery may
// reset an overheated or unproven ASIC any more. device_lock is held.
void health_refuse(void* context, rx7900::hardware::HealthReason reason) noexcept {
    auto& device = *static_cast<VideoDevice*>(context);
    device.power_failed = true;
#ifdef RX7900_RADEON2D_VIDEOPORT
    if (device.radeon2d.initialized) {
        rx7900::reactos::radeon2d::NativeGuard guard(device.radeon2d);
        device.radeon2d.dispatcher.disable(false);
    }

    device.radeon2d_enabled = false;

    if (device.recovery_started) {
        device.recovery_gate.stop();
        KeSetEvent(&device.recovery_wake, IO_NO_INCREMENT, FALSE);
    }
#endif
    char message[] = "RX7900: GPU quiesced: new work refused, reason=%u\n";
    VideoPortDebugPrint(Error, message, static_cast<unsigned>(reason));
}

// amdgpu_ras_reset_gpu: the forced destructive (mode-1) recovery of the worker
// when it runs; false without one (GpuHealth then refuses). device_lock held.
bool health_escalate(void* context, rx7900::hardware::HealthReason) noexcept {
#ifdef RX7900_RADEON2D_VIDEOPORT
    auto& device = *static_cast<VideoDevice*>(context);

    if (device.recovery_started && device.radeon2d.initialized && !device.radeon2d_recovery.disabled()) {
        request_recovery(device, true);
        return device.recovery_gate.pending();
    }
#else
    (void)context;
#endif
    return false;
}

// VRAM byte offset -> VMID0 MC address of this incarnation, then every process
// session owning it is quarantined; owners counts every session owning it.
uint32_t health_quarantine_vram(void* context, uint64_t vram_offset, uint64_t bytes, uint32_t& owners) noexcept {
    auto& device = *static_cast<VideoDevice*>(context);
    const uint64_t mc_start = device.asic.asic.vram_location().mc_start;
    owners = 0;

    if (vram_offset > UINT64_MAX - mc_start) {
        return 0;
    }

    return rx7900::reactos::gpu_process_quarantine_vram(device.processes, mc_start + vram_offset, bytes, &owners);
}

// PASSIVE_LEVEL, from enable_sources_and_poll (bring-up, resume, restart after
// a confirmed reset): the first call starts the worker, later ones rebind.
// Out of line: the config and hooks stay off the callers' frames.
#if defined(_MSC_VER)
__declspec(noinline)
#else
__attribute__((noinline))
#endif
void start_gpu_health(VideoDevice& device) noexcept {
    auto& asic = device.asic.asic;
    const auto& ras = asic.ras_support();
    // UMC 8.10 node/channel layout for the MCA error-record reader; without it
    // an ECC interrupt stays an uncorrectable error at an unknown address.
    rx7900::hardware::UmcTopology umc{};

    if (ras.umc_supported &&
        rx7900::hardware::validate_umc_discovery(device.asic.discovery.bytes(), umc) !=
            rx7900::hardware::DiscoveryStatus::validated) {
        char message[] = "RX7900: UMC 8.10 error-record layout not validated from IP discovery\n";
        VideoPortDebugPrint(Error, message);
    }

    const rx7900::hardware::GpuHealthConfig config{
        {&device.hardware, device.hardware.register_bytes, irq_mmio_read, irq_mmio_write, irq_stall},
        asic.nbio_registers(), ras.hw_enabled, ras.umc_supported, &asic.smu(), &device.asic.vram_allocator, umc};
    const rx7900::reactos::GpuHealthHooks hooks{&device, health_run_locked, health_refuse, health_escalate,
                                                health_quarantine_vram};

    if (!rx7900::reactos::gpu_health_start(device.health, config, hooks)) {
        char message[] = "RX7900: thermal/RAS interrupt actions unavailable\n";
        VideoPortDebugPrint(Error, message);
    }
}


VP_STATUS describe_mode(VideoDevice& device, uint32_t index,
                        VIDEO_MODE_INFORMATION& output) noexcept {
    rx7900::display::Timing timing{};
    Status status = device.modes.query_mode(index, timing);

    if (status != Status::success) {
        return video_status(status);
    }

    uint32_t refresh = 0;
    status = rx7900::display::refresh_millihz(timing, refresh);

    if (status != Status::success) {
        return video_status(status);
    }

    rx7900::display::ScanoutLayout scanout{};
    status = rx7900::display::validate_framebuffer(timing, device.framebuffer, scanout);

    if (status != Status::success) {
        return video_status(status);
    }

    VIDEO_MODE_INFORMATION mode{};
    mode.Length = static_cast<ULONG>(sizeof(mode));
    mode.ModeIndex = index;
    mode.VisScreenWidth = timing.width;
    mode.VisScreenHeight = timing.height;
    mode.ScreenStride = scanout.stride_bytes;
    mode.NumberOfPlanes = 1;
    mode.BitsPerPlane = 32;
    mode.Frequency = refresh / 1000u;
    mode.NumberRedBits = 8;
    mode.NumberGreenBits = 8;
    mode.NumberBlueBits = 8;
    mode.RedMask = 0x00ff0000u;
    mode.GreenMask = 0x0000ff00u;
    mode.BlueMask = 0x000000ffu;
    mode.AttributeFlags = VIDEO_MODE_GRAPHICS | VIDEO_MODE_COLOR | VIDEO_MODE_LINEAR |
                          VIDEO_MODE_NO_OFF_SCREEN;
    mode.VideoMemoryBitmapWidth = scanout.stride_bytes /
                                   rx7900::display::xrgb8888_bytes_per_pixel;
    mode.VideoMemoryBitmapHeight = timing.height;
    // Physical millimeters stay unknown; never invent a monitor size/DPI.
    output = mode;
    return NO_ERROR;
}

bool has_output(const VIDEO_REQUEST_PACKET& request, ULONG bytes) noexcept {
    return rx7900::reactos::has_video_output(request,bytes);
}

// The I/O manager supplies one METHOD_BUFFERED system allocation, possibly
// aliasing InputBuffer. Caller decodes input before any write, checks the whole
// output extent and retains no pointer. MoveMemory handles native alignment.
template<typename T>
VP_STATUS return_value(VIDEO_REQUEST_PACKET& request, T& value) noexcept {
    return rx7900::reactos::return_video_value(request,value);
}

VP_STATUS query_modes(VideoDevice& device, VIDEO_REQUEST_PACKET& request) noexcept {
    const uint32_t count = device.modes.mode_count();

    if (count == 0) {
        return ERROR_DEV_NOT_EXIST;
    }

    // The common catalog is capped at 64, so this product fits native ULONG.
    const ULONG bytes = count * static_cast<ULONG>(sizeof(VIDEO_MODE_INFORMATION));

    if (!has_output(request, bytes)) {
        return ERROR_INSUFFICIENT_BUFFER;
    }

    for (uint32_t index = 0; index < count; ++index) {
        VIDEO_MODE_INFORMATION mode{};
        const VP_STATUS status = describe_mode(device, index, mode);

        if (status != NO_ERROR) {
            return status;
        }

        // Pointer arithmetic is confined to this validated native output span.
        // The kernel owns bytes bytes throughout this synchronous callback.
        auto* destination = static_cast<UCHAR*>(request.OutputBuffer) +
                            index * sizeof(VIDEO_MODE_INFORMATION);
        VideoPortMoveMemory(destination, &mode, static_cast<ULONG>(sizeof(mode)));
    }

    request.StatusBlock->Information = bytes;
    return NO_ERROR;
}

} // namespace

// VideoPort calls at PASSIVE_LEVEL with a valid, correctly aligned extension and
// config. Native hardware services identify/claim the PnP adapter's resources;
// no HAL scan, BAR sizing writes or speculative resource claims occur here.
extern "C" VP_STATUS NTAPI rx7900_find_adapter(PVOID extension, PVOID context,
                                               PWSTR arguments,
                                               PVIDEO_PORT_CONFIG_INFO config,
                                               PUCHAR again) {
    (void)context;
    (void)arguments;

    if (extension == nullptr || config == nullptr || again == nullptr ||
        config->Length < sizeof(VIDEO_PORT_CONFIG_INFO)) {
        return ERROR_INVALID_PARAMETER;
    }

    *again = FALSE;
    VideoDevice* storage = device_at(extension);

    if (storage == nullptr) {
        return ERROR_INVALID_PARAMETER;
    }

    auto* device = rx7900::reactos::construct_device_state<VideoDevice>(storage);
    // PASSIVE_LEVEL: the device mutex exists before any other callback runs.
    KeInitializeMutex(&device->device_lock, 0);
    device->device_lock_ready = true;
    device->extension = extension;
    const VP_STATUS status = rx7900::reactos::acquire(device->hardware, extension, config);

    if (status != NO_ERROR) {
        // acquire also guarantees cleanup of partially acquired resources.
        return status;
    }

    device->resources_owned = true;
    return NO_ERROR; // PCI discovery only; HwInitialize must confirm display setup.
}

extern "C" BOOLEAN NTAPI rx7900_initialize(PVOID extension) {
    if (extension == nullptr || VideoPortGetCurrentIrql() != 0) {
        return FALSE;
    }

    VideoDevice* state = device_at(extension);

    if (state == nullptr) {
        return FALSE;
    }

    auto& device = *state;

    if (!device.resources_owned) {
        return FALSE;
    }

    // amdgpu_get_bios: the card's own VBIOS (legacy shadow, ROM BAR, SMUIO) in
    // Linux order, rx7900.rom last. Card sources touch PCI config and GPU registers,
    // so they run only with the PhysicalAsicInit opt-in; otherwise the file alone.
    const bool physical = physical_init_requested(extension);
    const auto precedence = physical && !atom_rom_override(extension) ?
        rx7900::reactos::VbiosPrecedence::card_first :
        rx7900::reactos::VbiosPrecedence::administrator_file_only;
    rx7900::reactos::VbiosReport vbios{};

    if (!NT_SUCCESS(rx7900::reactos::acquire_vbios(device.hardware, extension, precedence,
                                                   device.asic.discovery, vbios))) {
        char message[] = "RX7900: no valid VBIOS from the card or rx7900.rom\n";
        VideoPortDebugPrint(Error, message);
        release_device(device, extension);
        return FALSE;
    }

    char vbios_message[] = "RX7900: VBIOS source=%u bytes=%u\n";
    VideoPortDebugPrint(Error, vbios_message, static_cast<unsigned>(vbios.source),
                        static_cast<unsigned>(vbios.image_bytes));

    const rx7900::ByteView rom = rx7900::reactos::atom_rom(device.hardware);

    // Snapshot the firmware scanout before ASIC init touches the memory hub;
    // only with the physical opt-in (the capture writes the MM_INDEX window).
    auto captured = physical ? rx7900::reactos::capture_preos_scanout(device.hardware, rom, device.preos)
                             : rx7900::reactos::PreosAdoption::not_captured;

    // Boot firmware from HARDWARE\DESCRIPTION (freeldr UEFI vs PC BIOS loader).
    char firmware_message[] = "RX7900: boot firmware=%u (0 unknown, 1 BIOS, 2 UEFI)\n";
    VideoPortDebugPrint(Error, firmware_message, static_cast<unsigned>(rx7900::reactos::detect_boot_firmware()));

    // Legacy (CSM) boot: the VGA engine scans text/planar memory, which is never
    // adoptable. With the explicit opt-in the VBIOS sets a VBE linear mode; the
    // scanout it programmed is then captured again before ASIC initialization.
    // set_vbe_linear_mode itself refuses unless the capture reported legacy_vga.
    if (physical && vbe_modeset_requested(extension)) {
        const auto vbe = rx7900::reactos::set_vbe_linear_mode(device.hardware, extension, device.preos, device.vbe);
        const auto& report = device.vbe.report;
        char vbe_message[] =
            "RX7900: VBE mode set result=%u status=%u function=0x%x ax=0x%x version=0x%x previous=0x%x "
            "mode=0x%x %ux%u preference=%ux%u source=%u restored=%u\n";
        VideoPortDebugPrint(Error, vbe_message, static_cast<unsigned>(vbe), static_cast<unsigned>(report.status),
                            static_cast<unsigned>(report.failed_function), static_cast<unsigned>(report.failed_ax),
                            static_cast<unsigned>(report.version), static_cast<unsigned>(report.previous_mode),
                            static_cast<unsigned>(report.chosen.mode), static_cast<unsigned>(report.chosen.width),
                            static_cast<unsigned>(report.chosen.height), static_cast<unsigned>(report.preference.width),
                            static_cast<unsigned>(report.preference.height),
                            static_cast<unsigned>(report.preference.source), report.restored ? 1u : 0u);

        if (vbe == rx7900::reactos::VbeModeset::mode_set) {
            captured = rx7900::reactos::capture_preos_scanout(device.hardware, rom, device.preos);
        }
    }

    if (physical) {
        InterruptHookContext hook{&device, extension};
        const auto result = rx7900::reactos::run_asic_bringup(device.hardware, extension, rom,
                                                              {&hook, on_ih_ready}, device.asic);
        report_bringup(device.asic, result);

        // Re-check the scanout against the post-init state before any other
        // VRAM user; refuses adoption on any change or failed bring-up.
        if (captured == rx7900::reactos::PreosAdoption::captured) {
            rx7900::reactos::finalize_preos_after_bringup(device.hardware, device.asic, result, device.preos);
        }

        if (result == rx7900::reactos::BringupResult::completed) {
            enable_interrupt_sources(device);
            device.asic_running = true;
            physical_driver_pinned = true;
            if (physical_driver != nullptr) physical_driver->DriverUnload = nullptr;
            if (!rx7900::reactos::gpu_process_initialize(device.processes, device.asic)) {
                char message[] = "RX7900: native process GPUVM service unavailable\n";
                VideoPortDebugPrint(Error, message);
            } else if (device.preos.adopted) {
                // Kernel GL presents may write only this adopted linear scanout.
                const auto& scanout = device.preos.scanout;
                const rx7900::reactos::GraphicsScanout target{scanout.mc_address, scanout.timing.width,
                                                              scanout.timing.height, scanout.stride_bytes};
                if (!rx7900::reactos::gpu_process_configure_scanout(device.processes, &target)) {
                    char message[] = "RX7900: GL present scanout refused\n";
                    VideoPortDebugPrint(Error, message);
                }
            }
        } else {
            // Without a running ASIC no resume can need the firmware again.
            rx7900::reactos::release_asic_files(device.asic);
        }
    }

    const auto status = rx7900::output::parse_connectors(rom, device.connectors);

    if (status != rx7900::output::FirmwareStatus::success || device.connectors.count == 0) {
        char message[] = "RX7900: usable ATOM connector topology unavailable\n";
        VideoPortDebugPrint(Error, message);
        release_device(device, extension);
        return FALSE;
    }

    char preos_message[] = "RX7900: pre-OS scanout adoption=%u status=%u %ux%u refresh=%u mHz offset=0x%x\n";
    VideoPortDebugPrint(Error, preos_message, static_cast<unsigned>(device.preos.result),
                        static_cast<unsigned>(device.preos.report.status),
                        static_cast<unsigned>(device.preos.scanout.timing.width),
                        static_cast<unsigned>(device.preos.scanout.timing.height),
                        static_cast<unsigned>(device.preos.scanout.refresh_millihz),
                        static_cast<unsigned>(device.preos.scanout.vram_offset));

    // The adopted firmware scanout is the one owned display mode: the miniport
    // reports it and maps its framebuffer; the display engine is not programmed.
    if (device.preos.adopted) {
#ifdef RX7900_DISPLAY_LINK
        // dcn-link ROOT HOOK: DisplayLinkOwner=1 after the completed bring-up,
        // the adoption and the connector parse; then (DisplayModeSet=1) the
        // stream owner, which programs nothing until a mode is set.
        if (device.asic_running && display_link_requested(extension) && bind_display_link(device, 1)) {
#ifdef RX7900_DISPLAY_STREAM
            if (display_mode_set_requested(extension)) {
                start_display_stream(device, extension);
            }
#endif
        }
#endif
#ifdef RX7900_RADEON2D_VIDEOPORT
        rx7900::reactos::radeon2d::native_initialize(device.radeon2d);

        if (device.asic_running && radeon2d_requested(extension)) {
            device.radeon2d_compute = radeon2d_compute_requested(extension);
            start_radeon2d(device);

            // The process service escalates hung exited sessions to this
            // worker's forced destructive recovery (request_process_recovery).
            if (device.radeon2d_enabled && start_recovery_worker(device)) {
                rx7900::reactos::gpu_process_set_recovery_hook(device.processes, &device,
                                                               request_process_recovery);
            }
        }
#endif
        if (device.asic_running && rx7900::reactos::hardware_cursor_requested(extension)) {
            const auto cursor = rx7900::reactos::start_hardware_cursor(device.cursor, device.hardware, device.asic,
                                                                       device.preos, rom, extension);
            char cursor_message[] = "RX7900: hardware cursor result=%u refclk=%u kHz\n";
            VideoPortDebugPrint(Error, cursor_message, static_cast<unsigned>(cursor),
                                static_cast<unsigned>(device.cursor.refclk_khz));
        }

        device.initialized = true;
        return TRUE;
    }

    // ATOM routes are real firmware data, but do not establish ownership of a
    // running stream. Opt-in PSP/SMU firmware loading above is not display
    // bring-up: DMUB start, DCN programming, DDC/AUX transport,
    // link training, clocks/DML and owned VRAM scanout are still absent. There
    // is no physical ModeBackend to install. Do not advertise candidate modes,
    // reuse a boot BAR as scanout, or return TRUE after only parsing firmware.
    char message[] =
        "RX7900: ATOM topology parsed; physical DCN/link/scanout bring-up unavailable\n";
    VideoPortDebugPrint(Error, message);
    release_device(device, extension);
    return FALSE;
}

namespace {

// The body of rx7900_start_io, with the shared GpuProcessGuard held.
void start_io_locked(VideoDevice& device, PVOID extension, PVIDEO_REQUEST_PACKET packet) noexcept {
    packet->StatusBlock->Information = 0;
    VP_STATUS status = ERROR_INVALID_FUNCTION;

    if (packet->IoControlCode == rx7900::reactos::process_abi::ioctl) {
        packet->StatusBlock->Status = !device.low_power && !device.power_failed ?
            rx7900::reactos::gpu_process_start_io(device.processes, *packet) :
            ERROR_DEVICE_REINITIALIZATION_NEEDED;
        return;
    }

    // Kernel-only: radeondisp DrvDrawEscape with win32k's CLIPOBJ rectangles.
    // The device is exclusive to win32k; radeondisp never forwards caller bytes
    // under this code. The process session is still authenticated by NT process.
    if (packet->IoControlCode == rx7900::reactos::graphics_abi::present_ioctl) {
        packet->StatusBlock->Status = !device.low_power && !device.power_failed && device.preos.adopted ?
            rx7900::reactos::gpu_process_present(device.processes, *packet) :
            ERROR_DEVICE_REINITIALIZATION_NEEDED;
        return;
    }

#ifdef RX7900_RADEON2D_VIDEOPORT
    if (rx7900::reactos::radeon2d::abi::is_radeon_2d_ioctl(packet->IoControlCode)) {
        // A fault seen since the last request (fence poll job timeout, CP or
        // VM fault) is handed to the recovery worker first; the dispatcher
        // answers every state (operational, suspended, recovering: query and
        // not_ready, disabled: query only).
        if (!device.low_power && radeon2d_recovery_due(device)) {
            request_recovery(device, false);
        }

        packet->StatusBlock->Status = device.radeon2d.initialized && !device.low_power ?
            rx7900::reactos::radeon2d::native_start_io(device.radeon2d, *packet) :
            ERROR_INVALID_FUNCTION;

        // A fault this request observed (its receipt said lost): only
        // requested here, the reset runs on the worker's own stack.
        if (!device.low_power && radeon2d_recovery_due(device)) {
            request_recovery(device, false);
        }

        return;
    }
#endif

    // Pointer IOCTLs: the hardware cursor when started, otherwise refused so the
    // display driver keeps the engine pointer.
    if (rx7900::reactos::is_pointer_ioctl(packet->IoControlCode)) {
        packet->StatusBlock->Status = device.preos.adopted && !device.power_failed ?
            rx7900::reactos::cursor_start_io(device.cursor, *packet) : ERROR_INVALID_FUNCTION;
        return;
    }

#ifdef RX7900_DISPLAY_STREAM
    // The DCN 3.2 stream owner (DisplayModeSet=1) serves every mode IOCTL once
    // started; the pre-OS branch below stays the path when it is not.
    if (device.display_stream.active() && rx7900::reactos::NativeDisplayStream::is_mode_ioctl(packet->IoControlCode)) {
        const ULONG code = packet->IoControlCode;
        if (device.low_power || device.power_failed) {
            packet->StatusBlock->Status = ERROR_DEVICE_REINITIALIZATION_NEEDED;
            return;
        }
        if (code == IOCTL_VIDEO_SET_CURRENT_MODE) {
            // The cursor is re-validated against the committed pipe afterwards.
            rx7900::reactos::cursor_suspend(device.cursor);
        } else if (code == IOCTL_VIDEO_QUERY_CURRENT_MODE) {
            // Underflow / stalled-OTG check of the committed stream (one frame).
            device.display_stream.check_and_recover();
        }
        status = device.display_stream.start_io(*packet);
        if (code == IOCTL_VIDEO_SET_CURRENT_MODE) {
            retarget_display(device, true);
            rx7900::reactos::cursor_resume(device.cursor, device.preos);
            if (status == NO_ERROR) {
                rx7900::reactos::cursor_mode_set(device.cursor);
            }
        } else if (code == IOCTL_VIDEO_RESET_DEVICE) {
            rx7900::reactos::cursor_reset_device(device.cursor);
        }
#ifdef RX7900_RADEON2D_VIDEOPORT
        // An unconfirmed display write escalates to the destructive recovery,
        // which rebinds the link owner and commits the mode again.
        if (device.display_stream.report().result == rx7900::display::StreamResult::uncertain) {
            request_recovery(device, true);
        }
#endif
        packet->StatusBlock->Status = status;
        return;
    }
#endif

    // The adopted pre-OS mode replaces the ModeController for every mode IOCTL.
    if (device.preos.adopted) {
        switch (packet->IoControlCode) {
        case IOCTL_VIDEO_QUERY_NUM_AVAIL_MODES:
            status = rx7900::reactos::preos_query_num_modes(device.preos, *packet);
            break;
        case IOCTL_VIDEO_QUERY_AVAIL_MODES:
            status = rx7900::reactos::preos_query_modes(device.preos, *packet);
            break;
        case IOCTL_VIDEO_QUERY_CURRENT_MODE:
            status = rx7900::reactos::preos_query_current_mode(device.preos, *packet);
            break;
        case IOCTL_VIDEO_SET_CURRENT_MODE:
            status = rx7900::reactos::preos_set_current_mode(device.preos, device.hardware, extension, *packet);

            if (status == NO_ERROR) {
                rx7900::reactos::cursor_mode_set(device.cursor);
            }

            break;
        case IOCTL_VIDEO_MAP_VIDEO_MEMORY:
            status = rx7900::reactos::preos_map_video_memory(device.preos, device.hardware, extension, *packet);
            break;
        case IOCTL_VIDEO_UNMAP_VIDEO_MEMORY:
            status = rx7900::reactos::preos_unmap_video_memory(device.preos, extension, *packet);
            break;
        case IOCTL_VIDEO_RESET_DEVICE:
            // framebuf's DrvAssertMode(FALSE) needs success; the firmware mode stays.
            device.preos.mode_set = false;
            rx7900::reactos::cursor_reset_device(device.cursor);
            status = NO_ERROR;
            break;
        default:
            break;
        }

        packet->StatusBlock->Status = status;
        return;
    }

    switch (packet->IoControlCode) {
    case IOCTL_VIDEO_QUERY_NUM_AVAIL_MODES: {
        VIDEO_NUM_MODES modes{};
        modes.NumModes = device.initialized ? device.modes.mode_count() : 0;
        modes.ModeInformationLength = static_cast<ULONG>(sizeof(VIDEO_MODE_INFORMATION));
        status = return_value(*packet, modes);
        break;
    }
    case IOCTL_VIDEO_QUERY_AVAIL_MODES:
        status = device.initialized ? query_modes(device, *packet) : ERROR_DEV_NOT_EXIST;
        break;
    case IOCTL_VIDEO_QUERY_CURRENT_MODE: {
        ModePlan current{};
        status = device.initialized ? video_status(device.modes.current_mode(current)) :
                                      ERROR_DEV_NOT_EXIST;

        if (status == NO_ERROR) {
            VIDEO_MODE_INFORMATION mode{};
            status = describe_mode(device, current.mode_index, mode);

            if (status == NO_ERROR) {
                status = return_value(*packet, mode);
            }
        }

        break;
    }
    case IOCTL_VIDEO_SET_CURRENT_MODE: {
        rx7900::display::ModeRequest request{};
        const rx7900::ByteView input(static_cast<const uint8_t*>(packet->InputBuffer),
                                    packet->InputBufferLength);
        status = video_status(rx7900::display::decode_video_mode_request(input, request));

        if (status == NO_ERROR) {
            if (!device.initialized) {
                status = ERROR_DEV_NOT_EXIST;
            } else if (VideoPortGetCurrentIrql() != 0) {
                status = ERROR_INVALID_FUNCTION;
            } else {
                status = video_status(device.modes.set_mode(request, device.framebuffer));
            }
        }

        break;
    }
    case IOCTL_VIDEO_MAP_VIDEO_MEMORY:
    case IOCTL_VIDEO_UNMAP_VIDEO_MEMORY:
    case IOCTL_VIDEO_RESET_DEVICE:
        // Not a successful no-op. Mapping requires a separately owned CPU BAR
        // view of the actual VRAM allocation; reset requires confirmed restore.
        status = ERROR_INVALID_FUNCTION;
        break;
    default:
        break;
    }

    packet->StatusBlock->Status = status;
}

} // namespace

// Serialized synchronous VideoPort IOCTL callback. Mode changes require
// PASSIVE_LEVEL and a hardware-confirmed initialized session. No user mapping
// is returned before an owned CPU-visible scanout mapping service exists.
extern "C" BOOLEAN NTAPI rx7900_start_io(PVOID extension, PVIDEO_REQUEST_PACKET packet) {
    if (extension == nullptr || packet == nullptr || packet->StatusBlock == nullptr) {
        return FALSE;
    }

    VideoDevice* state = device_at(extension);

    if (state == nullptr) {
        return FALSE;
    }

    auto& device = *state;
    if (VideoPortGetCurrentIrql() != PASSIVE_LEVEL) {
        packet->StatusBlock->Status = ERROR_INVALID_FUNCTION;
        packet->StatusBlock->Information = 0;
        return TRUE;
    }

    DeviceGuard device_guard(device);
    {
        rx7900::reactos::GpuProcessGuard process_guard(device.processes);
        start_io_locked(device, extension, packet);
    }

    return TRUE; // Request consumed; operation success is only StatusBlock.Status.
}

// May run at elevated IRQL during bugcheck/shutdown. No pageable firmware work,
// allocation, waiting or unproven register writes are permitted. A text-console
// hardware restore is not implemented, so explicitly decline that operation.
extern "C" BOOLEAN NTAPI rx7900_reset_hw(PVOID extension, ULONG columns, ULONG rows) {
    (void)extension;
    (void)columns;
    (void)rows;
    return FALSE;
}

// HwGetPowerState for the adapter (HwId DISPLAY_ADAPTER_HW_ID; this miniport
// enumerates no child devices). The physical path supports D0 and D3 only, as
// the Linux PCI core drives Navi31 (D0 <-> D3hot/D3cold): VideoPowerOn is D0;
// StandBy, Suspend, Off and Hibernate all take the D3 path. Shutdown is refused.
extern "C" VP_STATUS NTAPI rx7900_get_power(PVOID extension, ULONG hw_id,
                                            PVIDEO_POWER_MANAGEMENT power) {
    if (extension == nullptr || power == nullptr || power->Length < sizeof(*power)) {
        return ERROR_INVALID_PARAMETER;
    }

    if (hw_id != DISPLAY_ADAPTER_HW_ID) {
        return ERROR_INVALID_FUNCTION;
    }

    switch (power->PowerState) {
    case VideoPowerOn:
    case VideoPowerStandBy:
    case VideoPowerSuspend:
    case VideoPowerOff:
    case VideoPowerHibernate:
        return NO_ERROR;
    default:
        return ERROR_INVALID_FUNCTION;
    }
}

namespace {

// The body of rx7900_set_power, with the shared GpuProcessGuard held.
VP_STATUS set_power_locked(VideoDevice& device, PVOID extension, PVIDEO_POWER_MANAGEMENT power) noexcept {
    const bool low_power = power->PowerState != VideoPowerOn;

    if (!device.asic_running || device.low_power == low_power) {
        device.power_state = power->PowerState;
        return NO_ERROR;
    }

    if (device.power_failed) {
        return ERROR_DEVICE_REINITIALIZATION_NEEDED;
    }

    InterruptHookContext hook{&device, extension};
    using rx7900::hardware::AsicStatus;

    if (low_power) {
#ifdef RX7900_RADEON2D_VIDEOPORT
        // A pending or newly seen fault is recovered first (by the worker):
        // the D3 request is refused meanwhile.
        if (radeon2d_recovery_due(device)) {
            request_recovery(device, false);
        }

        if (device.recovery_gate.pending()) {
            return ERROR_BUSY;
        }
#endif
        if (!rx7900::reactos::gpu_process_suspend(device.processes)) {
            return ERROR_BUSY;
        }
#ifdef RX7900_RADEON2D_VIDEOPORT
        // The 2D engine drains, evicts offscreen VRAM to GTT and frees its
        // IBs before the rings detach.
        if (!suspend_radeon2d(device)) {
            rx7900::reactos::gpu_process_resume(device.processes);
            return ERROR_BUSY;
        }
#endif
        // The cursor is disabled while DCN still runs, as Linux turns off the
        // cursor plane in amdgpu_device_suspend's display phase
        // (drm_atomic_helper_suspend) before any IP block suspends.
        rx7900::reactos::cursor_suspend(device.cursor);
#ifdef RX7900_DISPLAY_STREAM
        // drm_atomic_helper_suspend also disables the CRTC: zero streams.
        device.display_stream.suspend();
#endif
        // amdgpu_pmops_suspend/freeze: amdgpu_device_suspend, the suspend_noirq
        // mode-1 reset, pci_save_state; then the fence poll stops (rings detached).
        const auto status = rx7900::reactos::suspend_asic(device.asic, &hook, on_ih_stopping);
        char message[] = "RX7900: suspend status=%u step=%u failure=%u\n";
        VideoPortDebugPrint(Error, message, static_cast<unsigned>(status),
                            static_cast<unsigned>(device.asic.asic.report().step),
                            static_cast<unsigned>(device.asic.asic.report().suspend_failure));

        // A fence that did not drain aborts before any block suspended: still D0.
        if (status == AsicStatus::busy) {
            rx7900::reactos::gpu_process_resume(device.processes);
#ifdef RX7900_DISPLAY_STREAM
            // Nothing was reset: the same lease commits the current mode again.
            if (device.display_stream.active()) {
                device.display_stream.restore(device.display_link_epoch);
                retarget_display(device, false);
            }
#endif
            rx7900::reactos::cursor_resume(device.cursor, device.preos);
#ifdef RX7900_RADEON2D_VIDEOPORT
            resume_radeon2d(device, extension);
#endif
            return ERROR_BUSY;
        }

        if (status != AsicStatus::completed || !device.asic.power_saved) {
#ifdef RX7900_RADEON2D_VIDEOPORT
            // A suspend that failed part way (a hung ring, a block that did not
            // stop, its own reset): the worker recovers back to D0, through the
            // closing reset's proof when it exists. The D3 request fails.
            if (device.radeon2d_parked && device.recovery_started) {
                request_recovery(device, true);
                return ERROR_BUSY;
            }
#endif
            device.power_failed = true;
            return ERROR_DEVICE_REINITIALIZATION_NEEDED;
        }

        rx7900::reactos::fence_native_stop_poll(device.fence_native);
        device.low_power = true;
        device.power_state = power->PowerState;
        return NO_ERROR;
    }

    // amdgpu_pmops_resume/thaw/restore: pci_restore_state, amdgpu_device_resume,
    // then the interrupt sources and the fence poll come back.
    const auto status = rx7900::reactos::resume_asic(device.hardware, rx7900::reactos::atom_rom(device.hardware),
                                                     {&hook, on_ih_ready}, device.asic);
    char message[] = "RX7900: resume status=%u step=%u posted=%u\n";
    VideoPortDebugPrint(Error, message, static_cast<unsigned>(status),
                        static_cast<unsigned>(device.asic.asic.report().step),
                        device.asic.asic.report().posted ? 1u : 0u);

    if (status == AsicStatus::completed) {
        if (!rx7900::reactos::gpu_process_resume(device.processes)) {
#ifdef RX7900_RADEON2D_VIDEOPORT
            // The ASIC resumed but the process queues could not be reattached
            // (now quarantined): as for a resume that failed after programming
            // began, the worker's destructive recovery retires them and
            // reconstructs the live sessions (contents saved at suspend survive).
            if (device.radeon2d_parked && device.recovery_started) {
                device.low_power = false;
                device.power_state = VideoPowerOn;
                request_recovery(device, true);
                return ERROR_DEVICE_REINITIALIZATION_NEEDED;
            }
#endif
            device.power_failed = true;
            return ERROR_DEVICE_REINITIALIZATION_NEEDED;
        }
        enable_sources_and_poll(device);
#ifdef RX7900_DISPLAY_LINK
        // The suspend closed with a mode-1 reset: rebind and commit again
        // (amdgpu_dm dm_resume) before the cursor and 2D re-validate the scanout.
        display_reset_proven(device);
        display_restart(device, false);
#endif
        rx7900::reactos::cursor_resume(device.cursor, device.preos);
#ifdef RX7900_RADEON2D_VIDEOPORT
        resume_radeon2d(device, extension);
#endif
        device.low_power = false;
        device.power_state = VideoPowerOn;
#ifdef RX7900_RADEON2D_VIDEOPORT
        // A request deferred by low power runs now (a process escalation too).
        if (device.recovery_gate.pending() || device.process_recovery_requested != 0) {
            KeSetEvent(&device.recovery_wake, IO_NO_INCREMENT, FALSE);
        }
#endif
        return NO_ERROR;
    }

    // An unposted card (no ATOM asic_init here) stays suspended and retryable.
    if (!device.asic.asic.report().suspended) {
#ifdef RX7900_RADEON2D_VIDEOPORT
        // A resume that failed after programming began: the card is in D0
        // again, unproven; the worker recovers the parked device (its
        // suspended sessions and GTT surfaces) after this callback returns.
        if (device.radeon2d_parked && device.recovery_started) {
            device.low_power = false;
            device.power_state = VideoPowerOn;
            request_recovery(device, true);
            return ERROR_DEVICE_REINITIALIZATION_NEEDED;
        }
#endif
        device.power_failed = true;
    }

    return ERROR_DEVICE_REINITIALIZATION_NEEDED;
}

} // namespace

// HwSetPowerState, PASSIVE_LEVEL, serialized by VideoPort with the other device
// callbacks. Note: ReactOS videoprt (tree a97c8a2c) never calls this hook: its
// IntVideoPortDispatchPower passes FDO power IRPs to the PCI PDO and completes
// child PDO IRPs itself (win32ss/drivers/videoprt/dispatch.c), and only uses the
// hook pointers to classify PnP miniports (videoprt.c, which also requires
// HwGetVideoChildDescriptor, so this legacy-detected miniport is unchanged).
// Windows videoprt calls it before passing D-IRPs down (D3) and after they
// complete (D0); the bus driver owns PMCSR (ReactOS pci.sys completes SET_POWER
// without touching it, drivers/bus/pci/pdo.c), so no D-state register is written
// here. Without a completed PhysicalAsicInit bring-up nothing is touched and the
// state is only recorded.
extern "C" VP_STATUS NTAPI rx7900_set_power(PVOID extension, ULONG hw_id,
                                            PVIDEO_POWER_MANAGEMENT power) {
    const VP_STATUS supported = rx7900_get_power(extension, hw_id, power);

    if (supported != NO_ERROR) {
        return supported;
    }

    VideoDevice* state = device_at(extension);

    if (state == nullptr || VideoPortGetCurrentIrql() != PASSIVE_LEVEL) {
        return ERROR_INVALID_PARAMETER;
    }

    auto& device = *state;
    VP_STATUS status = NO_ERROR;
    {
        // Waits for an in-flight recovery (the worker holds device_lock).
        DeviceGuard device_guard(device);
        rx7900::reactos::GpuProcessGuard process_guard(device.processes);
        status = set_power_locked(device, extension, power);
    }

    return status;
}

// VideoPort DPC at DISPATCH_LEVEL, queued by rx7900_interrupt. Decodes and routes
// one bounded batch per ring, lets the fence manager process EOP/trap/fault
// records of that batch (amdgpu_fence_process in IRQ context on Linux), and
// requeues itself while acknowledged work remains.
extern "C" VOID NTAPI rx7900_interrupt_dpc(PVOID extension, PVOID context) {
    (void)context;
    VideoDevice* state = extension != nullptr ? device_at(extension) : nullptr;

    if (state == nullptr) {
        return;
    }

    if (state->irq.drain({extension, synchronize_irq}, interrupt_consumer(*state)).more_pending) {
        VideoPortQueueDpc(extension, rx7900_interrupt_dpc, nullptr);
    }
}

// DIRQL callback on a possibly shared line. IrqService::isr touches no register
// unless armed after a completed physical bring-up (PhysicalAsicInit=1); it claims
// only pending IH6 ring 0 vectors, acknowledging them, and the DPC drains them.
extern "C" BOOLEAN NTAPI rx7900_interrupt(PVOID extension) {
    if (extension == nullptr) {
        return FALSE;
    }

    VideoDevice* state = device_at(extension);

    if (state == nullptr || !state->irq.isr()) {
        return FALSE;
    }

    VideoPortQueueDpc(extension, rx7900_interrupt_dpc, nullptr);
    return TRUE;
}

extern "C" ULONG NTAPI DriverEntry(PVOID context1, PVOID context2) {
    VIDEO_HW_INITIALIZATION_DATA initialization{};
    initialization.HwInitDataSize = static_cast<ULONG>(sizeof(initialization));
    initialization.AdapterInterfaceType = PCIBus;
    initialization.HwFindAdapter = rx7900_find_adapter;
    initialization.HwInitialize = rx7900_initialize;
    initialization.HwInterrupt = rx7900_interrupt;
    initialization.HwStartIO = rx7900_start_io;
    initialization.HwResetHw = rx7900_reset_hw;
    initialization.HwSetPowerState = rx7900_set_power;
    initialization.HwGetPowerState = rx7900_get_power;
    initialization.HwDeviceExtensionSize = static_cast<ULONG>(extension_bytes);
    const NTSTATUS notify = rx7900::reactos::gpu_process_driver_start();
    if (!NT_SUCCESS(notify)) return static_cast<ULONG>(notify);
    physical_driver = static_cast<PDRIVER_OBJECT>(context1);
    const ULONG status = VideoPortInitialize(context1, context2, &initialization, nullptr);
    if (status != NO_ERROR) {
        rx7900::reactos::gpu_process_driver_stop();
        physical_driver = nullptr;
        return status;
    }
    videoport_unload = physical_driver->DriverUnload;
    physical_driver->DriverUnload = physical_driver_pinned ? nullptr : inactive_driver_unload;
    return status;
}

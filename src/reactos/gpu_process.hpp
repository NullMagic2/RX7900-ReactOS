/*
Module: rx7900.reactos.gpu_process
Purpose: Own actual NT-process sessions, GPUVMs and MES queue lifetimes.
Created: 2026-10-22
Architecture: One device-owned service over the portable GpuProcessCore session
state machine (UserQueues, ProcessMemory, submissions, graphics) and native ASIC
services; serialized PASSIVE_LEVEL operations, process-exit notification, a
PASSIVE_LEVEL reaper thread and the confirmed-reset callbacks of AsicBringup.
*/
#pragma once

#include "asic_bringup.hpp"
#include "gpu_process_core.hpp"
#include "../opengl/gl_shader_gfx11.hpp"

namespace rx7900::reactos {

// No owning STL/kernel runtime. Construct once in the aligned native extension.
// Process objects are explicitly referenced until session retirement. Native
// queue metadata stays driver-owned and is never user mapped. The only submitted
// work is semantic copy/fill encoded by ProcessSubmissions and kernel-built
// graphics IBs from ProcessGraphics (typed draw records, generated shaders and
// allow-listed registers); raw streams fail. Every GpuProcessCore member is used
// only with lock held (or before publication / after the reaper was joined).
struct GpuProcessService final : GpuProcessCore {
    KMUTEX lock{};
    // Dedicated PASSIVE_LEVEL owner of exited sessions: signalled by the exit
    // notification (which itself releases nothing), it evicts their queues
    // through MES after a bounded idle wait, unmaps, destroys the VM and frees
    // every buffer; a hung removal asks for destructive recovery. The thread is
    // stopped and joined before the extension may be freed.
    KEVENT exit_reap_event{};
    HANDLE exit_reap_thread = nullptr;
    bool exit_reap_stop = false;
    AsicBringup* bringup = nullptr;
    // gl-shaders generator state (about 2 KiB), used under the service mutex.
    opengl::shader::GeneratorWorkspace shader_workspace{};
    // Kernel gfx-ring submitter of graphics contexts without a MES gfx queue:
    // AsicInit::kernel_queues and kernel_fences of the current incarnation
    // (refreshed by every gpu_process_initialize, under the service mutex).
    opengl::KernelGraphicsBridge kernel_bridge{};
};

// Shared physical-device serialization: callbacks that touch either Manager
// take this guard, including non-process Radeon2D and power callbacks. NT kernel
// mutexes are recursive, so a guarded VideoPort call may enter a process method
// that takes the same mutex; every acquisition has a matching release. IRQ/DPC
// never acquires this PASSIVE_LEVEL lock. Lock order: video.cpp device_lock,
// then the process registry lock (initialize/notify/shutdown), then this
// service mutex, then the Radeon 2D dispatcher mutex.
class GpuProcessGuard final {
public:
    explicit GpuProcessGuard(GpuProcessService& service) noexcept
        : lock_(service.initialized ? &service.lock : nullptr) {
        if (lock_ != nullptr) KeWaitForSingleObject(lock_, Executive, KernelMode, FALSE, nullptr);
    }
    ~GpuProcessGuard() noexcept { if (lock_ != nullptr) KeReleaseMutex(lock_, FALSE); }
    GpuProcessGuard(const GpuProcessGuard&) = delete;
    GpuProcessGuard& operator=(const GpuProcessGuard&) = delete;
private:
    KMUTEX* lock_;
};

// DriverEntry/unload own callback code lifetime. Call start before publishing
// any extension, stop before the original VideoPort unload callback. A running
// physical ASIC pins the driver module until hardware/IRQ teardown is proven.
NTSTATUS gpu_process_driver_start() noexcept;
void gpu_process_driver_stop() noexcept;

// PASSIVE_LEVEL only. Calls below serialize with process-exit cleanup. Services
// reference only real device-owned Managers and DMA tracking from AsicInit.
// On a service retired by a confirmed reset this is the rearm: it reconstructs
// every live session on the restarted ASIC (process_core_rearm). Takes the
// registry lock, then the service mutex; the caller may hold video.cpp's
// device_lock (lock order above), never the service mutex.
bool gpu_process_initialize(GpuProcessService& service, AsicBringup& bringup) noexcept;
VP_STATUS gpu_process_start_io(GpuProcessService& service, VIDEO_REQUEST_PACKET& packet) noexcept;

// The device owner's destructive-recovery request for an exited session whose
// MES removal stayed uncertain, or a reconstruction whose teardown did. Called
// from the reaper thread or gpu_process_initialize with no process lock held;
// it must only record the request and wake its worker (never block). nullptr
// clears it (the uncertain session then stays quarantined until a reset).
void gpu_process_set_recovery_hook(GpuProcessService& service, void* context,
                                   void (*request)(void*) noexcept) noexcept;

bool gpu_process_configure_scanout(GpuProcessService& service, const GraphicsScanout* scanout) noexcept;
// RAS page retirement (process_core_quarantine_vram), PASSIVE_LEVEL; the caller
// may hold device_lock and the GpuProcessGuard (recursive service mutex).
uint32_t gpu_process_quarantine_vram(GpuProcessService& service, uint64_t mc_address, uint64_t bytes,
                                     uint32_t* owners = nullptr) noexcept;

// graphics_abi::present_ioctl. Only radeondisp DrvDrawEscape issues it, with the
// win32k CLIPOBJ rectangles of the caller's DC; the session is authenticated
// against the executing NT process like every process request.
VP_STATUS gpu_process_present(GpuProcessService& service, VIDEO_REQUEST_PACKET& packet) noexcept;
bool gpu_process_suspend(GpuProcessService& service) noexcept;
bool gpu_process_resume(GpuProcessService& service) noexcept;
// Quarantined hardware resources are retained, never released by inference.
// Device/module storage must remain resident if this returns false.
bool gpu_process_shutdown(GpuProcessService& service) noexcept;
} // namespace rx7900::reactos

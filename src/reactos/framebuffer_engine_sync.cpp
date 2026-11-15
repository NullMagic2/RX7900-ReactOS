/*
Module: lightning.reactos.engine_sync
Purpose: Give the source-pinned primary Lightning GdiFlush an actual GPU barrier.
Created: 2026-10-22
Architecture: Win32k owns the referenced PDEV and mode lock; driver owns fences.
License: GPL-2.0-or-later.
*/
extern "C" {
#include <win32k.h>
}
#include "framebuffer_stretch_guard.hpp"
#include "lightning_present_abi.hpp"
#include "lightning-display.h"

namespace {
// PASSIVE_LEVEL. The reference outlives the mode lock. No list lock is held
// while entering the device lock or its synchronous display callback.
class PrimaryDevice final {
public:
    PrimaryDevice() noexcept : device_(EngpGetPDEV(nullptr)) {
        if (device_!=nullptr) EngAcquireSemaphore(device_->hsemDevLock);
    }
    ~PrimaryDevice() {
        if (device_!=nullptr) {
            EngReleaseSemaphore(device_->hsemDevLock);
            PDEVOBJ_vRelease(device_);
        }
    }
    PPDEVOBJ get() const noexcept {return device_;}
    PrimaryDevice(const PrimaryDevice&)=delete;
    PrimaryDevice& operator=(const PrimaryDevice&)=delete;
private:
    PPDEVOBJ device_;
};
}

extern "C" VOID APIENTRY Rx7900SynchronizePrimary(FLONG flags) noexcept {
    PrimaryDevice owner;
    auto* device=owner.get();
    if (device==nullptr || device->pSurface==nullptr ||
        (device->flFlags&(PDEV_DISABLED|PDEV_DRIVER_PUNTED_CALL))!=0) return;
    auto* surface=&device->pSurface->SurfObj;
    lightning::reactos::CpuScanoutInformation information{};
    if (device->DriverFunctions.Escape==nullptr ||
        device->DriverFunctions.Escape(surface,lightning::reactos::query_display_escape,0,nullptr,
            sizeof(information),&information)!=sizeof(information) ||
        information.size!=sizeof(information) || information.version!=lightning::reactos::cpu_scanout_abi_version ||
        information.magic!=LIGHTNING_DISPLAY_MAGIC || information.enabled!=1 || information.mode_generation==0) return;
    // This private path applies only to the active Lightning primary. Driver
    // completion means the real GPU fence, never a physical-monitor vblank.
    if (device->DriverFunctions.SynchronizeSurface!=nullptr)
        device->DriverFunctions.SynchronizeSurface(surface,nullptr,flags);
}

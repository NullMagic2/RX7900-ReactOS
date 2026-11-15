/*
Module: rx7900.opengl.kernel_graphics_bridge
Purpose: Bind the graphics lease owner to real GFX11 kernel-ring receipts.
Created: 2026-10-23
Architecture: Thin production callback adapter over existing queues and fences;
the native process service (gpu_process.cpp) owns one per device and points it
at AsicInit::kernel_queues/kernel_fences on every bring-up.
*/
#include "physical_submission.hpp"

namespace rx7900::opengl {
namespace {

hardware::Gfx11QueuesStatus submit(void* context,
                                   const hardware::Gfx11Submission& frame,
                                   uint32_t& sequence) noexcept {
    KernelGraphicsBridge& bridge = *static_cast<KernelGraphicsBridge*>(context);
    if (bridge.queues == nullptr || bridge.fences == nullptr) {
        sequence = 0;
        return hardware::Gfx11QueuesStatus::unavailable;
    }
    return bridge.queues->submit(frame, sequence);
}

hardware::FenceStatus query(void* context, uint32_t sequence) noexcept {
    KernelGraphicsBridge& bridge = *static_cast<KernelGraphicsBridge*>(context);
    return bridge.fences == nullptr ? hardware::FenceStatus::unavailable :
        bridge.fences->query(hardware::fence_ring_gfx, sequence);
}

hardware::Gfx11QueuesStatus retire(void* context) noexcept {
    KernelGraphicsBridge& bridge = *static_cast<KernelGraphicsBridge*>(context);
    return bridge.queues == nullptr ? hardware::Gfx11QueuesStatus::unavailable :
        bridge.queues->retire();
}

bool retired_after_reset(void* context) noexcept {
    KernelGraphicsBridge& bridge = *static_cast<KernelGraphicsBridge*>(context);
    // The confirmed-reset window: AsicInit retires its external owners after
    // the reset proof and before Gfx11Queues::release_after_reset.
    if (bridge.reset_confirmed != nullptr && bridge.reset_confirmed(bridge.reset_context)) {
        return true;
    }
    if (bridge.queues == nullptr || bridge.fences == nullptr) {
        return false;
    }
    return bridge.queues->report().status == hardware::Gfx11QueuesStatus::released &&
        bridge.fences->ring_report(hardware::fence_ring_gfx).state ==
            hardware::FenceRingState::detached;
}

} // namespace

GraphicsSubmitter KernelGraphicsBridge::submitter() noexcept {
    return {this, submit, query, retire, retired_after_reset};
}

} // namespace rx7900::opengl

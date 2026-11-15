/*
Module: lightning.reactos.clip_visibility
Purpose: Distinguish an updated empty region from an unbounded trivial clip.
Created: 2026-10-22
Architecture: Reuses framebuffer rectangle policy; borrows only public DDI state.
License: GPL-2.0-or-later.
*/
#include <stdarg.h>
#include <windef.h>
#include <wingdi.h>
#include <winddi.h>
#include "framebuffer_clip_visibility.hpp"
#include "framebuffer_shadow_policy.hpp"

extern "C" VOID APIENTRY Rx7900NormalizeClipRegion(CLIPOBJ* clip) noexcept {
    if (clip == nullptr || clip->iDComplexity != DC_TRIVIAL) {
        return;
    }

    const auto& bounds = clip->rclBounds;
    const rx7900::reactos::framebuffer::Rectangle visible{
        bounds.left, bounds.top, bounds.right, bounds.bottom};
    if (rx7900::reactos::framebuffer::empty(visible)) {
        // DC_TRIVIAL means no clipping to a display driver. This region was
        // explicitly updated, so empty bounds must instead reject all pixels.
        clip->iDComplexity = DC_RECT;
        clip->iFComplexity = FC_RECT;
    }
}

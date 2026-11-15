/*
Module: rx7900.reactos.radeon_text_engine
Purpose: Submit only identity-checked FreeType coverage glyphs to radeondisp.
Created: 2026-10-23
Architecture: The Win32k caller owns and locks all borrowed GDI objects for
this synchronous call. The DDI alone decides if its GPU operation is admissible.
*/
#include <stdarg.h>
#include <windef.h>
#include <wingdi.h>
#include <winddi.h>

#include "radeon_text_engine.hpp"
#include "radeon_text_bridge.hpp"

extern "C" VOID APIENTRY Rx7900RadeonTextBegin(RX7900_RADEON_TEXT_CONTEXT* context,
    SURFOBJ* destination, PFN_DrvEscape escape, PFN_DrvNotify notify) noexcept {
    if (context == nullptr) {
        return;
    }
    *context = {};

    if (destination == nullptr || destination->iType == STYPE_BITMAP ||
        destination->hdev == nullptr || escape == nullptr || notify == nullptr) {
        return;
    }

    rx7900::reactos::radeon_text::Capability capability{};
    const auto bytes = escape(destination,
        rx7900::reactos::radeon_text::identity_escape, 0, nullptr,
        sizeof(capability), &capability);
    if (bytes != sizeof(capability) || capability.size != sizeof(capability) ||
        capability.version != rx7900::reactos::radeon_text::version ||
        capability.magic != rx7900::reactos::radeon_text::identity_magic ||
        capability.notify_code != rx7900::reactos::radeon_text::glyph_notify) {
        return;
    }

    context->destination = destination;
    context->notify = notify;
}

extern "C" ULONG APIENTRY Rx7900RadeonTextGlyph(RX7900_RADEON_TEXT_CONTEXT* context,
    SURFOBJ* mask, CLIPOBJ* clip, XLATEOBJ* rgb_to_device,
    XLATEOBJ* device_to_rgb, const RECTL* destination,
    const POINTL* mask_origin, BRUSHOBJ* brush) noexcept {
    if (context == nullptr || context->destination == nullptr ||
        context->notify == nullptr || mask == nullptr ||
        destination == nullptr || mask_origin == nullptr || brush == nullptr) {
        return RX7900_RADEON_GLYPH_UNSUPPORTED;
    }

    rx7900::reactos::radeon_text::GlyphRequest request{};
    request.mask = mask;
    request.clip = clip;
    request.rgb_to_device = rgb_to_device;
    request.device_to_rgb = device_to_rgb;
    request.destination = *destination;
    request.mask_origin = *mask_origin;
    request.brush = brush;
    context->notify(context->destination,
        rx7900::reactos::radeon_text::glyph_notify, &request);

    if (request.result > rx7900::reactos::radeon_text::failed) {
        return RX7900_RADEON_GLYPH_FAILED;
    }
    return request.result;
}

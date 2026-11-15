/*
Module: lightning.reactos.present_request
Purpose: Decode the version-1 and revision-2 window presentation requests from captured bytes.
Created: 2026-10-24
Architecture: Pure checked decoding of the shared lightning-opengl-present.h
contract for the display driver's DrawEscape and host tests. Geometry against
the window and GDI limits stays with the caller; no pointer is reinterpreted.
*/
#pragma once
#include <stddef.h>
#include "../common/byte_view.hpp"
#include "lightning-opengl-present.h"
#include "lightning_image_receipt.hpp"

namespace lightning::reactos {

struct PresentRequest final {
    LightningOpenGLPresent present{};
    bool revision2 = false;
    bool async = false;
    ProducerPoint producer{};
};

// Exactly one complete request of either version. A revision-2 request names
// only known flags; its producer fields are zero exactly when FLAG_PRODUCER is
// absent. Output is unchanged on failure.
inline bool decode_present_request(rx7900::ByteView input, PresentRequest& output) noexcept {
    static_assert(sizeof(LightningOpenGLPresent) == LIGHTNING_OPENGL_PRESENT_BYTES &&
        sizeof(LightningOpenGLPresentAsync) == LIGHTNING_OPENGL_PRESENT_ASYNC_BYTES,
        "Fixed window presentation request layouts");
    PresentRequest candidate{};
    auto& present = candidate.present;
    uint32_t x = 0;
    uint32_t y = 0;

    if (!input.read_u32(offsetof(LightningOpenGLPresent, size), present.size) ||
        !input.read_u32(offsetof(LightningOpenGLPresent, version), present.version) ||
        !input.read_u64(offsetof(LightningOpenGLPresent, lease), present.lease) ||
        !input.read_u32(offsetof(LightningOpenGLPresent, image_width), present.image_width) ||
        !input.read_u32(offsetof(LightningOpenGLPresent, image_height), present.image_height) ||
        !input.read_u32(offsetof(LightningOpenGLPresent, image_stride), present.image_stride) ||
        !input.read_u32(offsetof(LightningOpenGLPresent, image_offset), present.image_offset) ||
        !input.read_u32(offsetof(LightningOpenGLPresent, x), x) ||
        !input.read_u32(offsetof(LightningOpenGLPresent, y), y) ||
        !input.read_u32(offsetof(LightningOpenGLPresent, width), present.width) ||
        !input.read_u32(offsetof(LightningOpenGLPresent, height), present.height)) {
        return false;
    }

    present.x = static_cast<int32_t>(x);
    present.y = static_cast<int32_t>(y);

    if (present.version == LIGHTNING_OPENGL_PRESENT_VERSION) {
        if (input.size() != LIGHTNING_OPENGL_PRESENT_BYTES || present.size != LIGHTNING_OPENGL_PRESENT_BYTES) {
            return false;
        }

        output = candidate;
        return true;
    }

    uint32_t flags = 0;
    uint32_t reserved = 0;

    if (present.version != LIGHTNING_OPENGL_PRESENT_ASYNC_VERSION ||
        input.size() != LIGHTNING_OPENGL_PRESENT_ASYNC_BYTES ||
        present.size != LIGHTNING_OPENGL_PRESENT_ASYNC_BYTES ||
        !input.read_u32(offsetof(LightningOpenGLPresentAsync, flags), flags) ||
        !input.read_u32(offsetof(LightningOpenGLPresentAsync, reserved), reserved) ||
        !input.read_u64(offsetof(LightningOpenGLPresentAsync, producer_sync), candidate.producer.sync) ||
        !input.read_u64(offsetof(LightningOpenGLPresentAsync, producer_value), candidate.producer.value) ||
        (flags & ~LIGHTNING_OPENGL_PRESENT_FLAGS) != 0 || reserved != 0) {
        return false;
    }

    const bool named = (flags & LIGHTNING_OPENGL_PRESENT_FLAG_PRODUCER) != 0;

    if (named != (candidate.producer.sync != 0) || named != (candidate.producer.value != 0)) {
        return false;
    }

    candidate.revision2 = true;
    candidate.async = (flags & LIGHTNING_OPENGL_PRESENT_FLAG_ASYNC) != 0;
    output = candidate;
    return true;
}

} // namespace lightning::reactos

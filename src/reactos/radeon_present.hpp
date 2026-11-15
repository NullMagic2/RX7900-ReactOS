/*
Module: rx7900.reactos.radeon_present
Purpose: radeondisp policy for the RadeonGL process escape and the clipped GPU present.
Created: 2026-10-23
Architecture: SDK-free and header-only so host tests run the exact DDI logic.
DrvEscape forwards RGLQ bytes unparsed to the miniport process IOCTL (which
authenticates the calling NT process); DrvDrawEscape turns the RGLP input and
win32k's CLIPOBJ into bounded PresentRequest batches. In this engine a DC's
DC_TRIVIAL clip means empty bounds (win32ss/gdi/eng/clip.c IntEngUpdateClipRegion),
so it presents nothing; a missing clip is unsupported and the ICD keeps its exact
readback path.
*/
#pragma once

#include <stdint.h>

#include "gpu_process_abi.hpp"
#include "process_graphics_abi.hpp"

namespace rx7900::reactos::radeon_present {

namespace gabi = rx7900::reactos::graphics_abi;
namespace pabi = rx7900::reactos::process_abi;

constexpr uint32_t request_header_bytes = 64;   // sizeof(process_abi::Request)
constexpr uint32_t reply_header_bytes = 48;     // sizeof(process_abi::Reply)
static_assert(sizeof(pabi::Request) == request_header_bytes && sizeof(pabi::Reply) == reply_header_bytes,
              "Process escape headers");

// RGLQ: only these sizes are forwarded; the bytes themselves are never parsed here.
constexpr bool process_escape_admitted(uint32_t input_bytes, uint32_t output_bytes, bool input, bool output) noexcept {
    return input && output && input_bytes >= request_header_bytes &&
           input_bytes <= request_header_bytes + pabi::bulk_bytes && output_bytes >= reply_header_bytes &&
           output_bytes <= reply_header_bytes + pabi::bulk_bytes;
}

// The miniport answered with at least a full reply header.
constexpr bool process_reply_valid(bool ioctl_ok, uint32_t returned) noexcept {
    return ioctl_ok && returned >= reply_header_bytes;
}

struct Rect final {
    int32_t left = 0;
    int32_t top = 0;
    int32_t right = 0;
    int32_t bottom = 0;
};

// Nonempty intersection, right/bottom exclusive.
constexpr bool intersect(const Rect& a, const Rect& b, Rect& out) noexcept {
    out.left = a.left > b.left ? a.left : b.left;
    out.top = a.top > b.top ? a.top : b.top;
    out.right = a.right < b.right ? a.right : b.right;
    out.bottom = a.bottom < b.bottom ? a.bottom : b.bottom;
    return out.left < out.right && out.top < out.bottom;
}

enum class ClipKind : uint8_t { missing, empty, rect, complex, unknown };

// DrvDrawEscape result: 0 unsupported (the ICD uses exact readback), otherwise
// 1 + the first non-success miniport outcome (1 = every batch succeeded).
struct Result final {
    uint32_t value = 0;
    uint32_t batches = 0;
    uint32_t rects = 0;
};

// Builds and sends PresentRequest batches of at most max_present_rects screen
// rectangles: each clip rectangle intersected with the window and the surface,
// in enumeration order, empties skipped. enumerate(visit) calls visit(Rect) for
// every DC_COMPLEX rectangle and returns false on enumeration failure.
// send(request, outcome) returns false when the IOCTL itself failed (then the
// whole present reports unsupported so the readback path repaints exactly;
// re-copying already presented rows is idempotent).
template <class Enumerate, class Send>
Result present(gabi::PresentRequest& request, const gabi::PresentInput& input, const Rect& window,
               const Rect& surface, ClipKind kind, const Rect& bounds, Enumerate&& enumerate,
               Send&& send) noexcept {
    Result result{};
    Rect visible{};
    if (kind == ClipKind::missing || kind == ClipKind::unknown) {
        return result;
    }
    if (kind == ClipKind::empty || !intersect(window, surface, visible)) {
        result.value = 1;
        return result;
    }

    request = {};
    request.input = input;
    request.window_left = window.left;
    request.window_top = window.top;
    uint32_t first_failure = 0;
    bool failed_ioctl = false;

    auto flush = [&]() noexcept {
        if (request.rect_count == 0 || failed_ioctl) {
            return;
        }
        uint32_t outcome = static_cast<uint32_t>(pabi::Outcome::success);
        if (!send(request, outcome)) {
            failed_ioctl = true;
            return;
        }
        ++result.batches;
        if (first_failure == 0 && outcome != static_cast<uint32_t>(pabi::Outcome::success)) {
            first_failure = outcome;
        }
        request.rect_count = 0;
    };
    auto add = [&](const Rect& clip) noexcept {
        Rect piece{};
        if (failed_ioctl || !intersect(clip, visible, piece)) {
            return;
        }
        auto& slot = request.rects[request.rect_count++];
        slot.left = piece.left;
        slot.top = piece.top;
        slot.right = piece.right;
        slot.bottom = piece.bottom;
        ++result.rects;
        if (request.rect_count == gabi::max_present_rects) {
            flush();
        }
    };

    if (kind == ClipKind::rect) {
        add(bounds);
    } else if (!enumerate(add)) {
        return Result{};
    }
    flush();
    if (failed_ioctl) {
        return Result{};
    }
    result.value = 1 + first_failure;
    return result;
}

} // namespace rx7900::reactos::radeon_present

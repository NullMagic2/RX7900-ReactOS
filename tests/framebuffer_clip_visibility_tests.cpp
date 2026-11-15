/*
Module: lightning.tests.clip_visibility
Purpose: Check explicit empty-region semantics using production public DDI state.
Created: 2026-10-22
Architecture: Calls the same C++ helper linked into the optional Win32k patch.
*/
#include <stdarg.h>
#include <windef.h>
#include <wingdi.h>
#include <winddi.h>
#include <stdint.h>
#include <stdio.h>
#include "framebuffer_clip_visibility.hpp"

namespace {
unsigned checks = 0;
unsigned failures = 0;

void expect(bool condition, const char* description) {
    ++checks;
    if (!condition) {
        ++failures;
        printf("FAIL: %s\n", description);
    }
}
}

int main() {
    Rx7900NormalizeClipRegion(nullptr);
    const RECTL empty_regions[]{
        {0, 0, 0, 0}, {320, 240, 320, 240}, {4, 2, 4, 10},
        {2, 4, 10, 4}, {8, 2, 4, 10}, {2, 8, 10, 4},
        {INT32_MIN, INT32_MAX, INT32_MIN, INT32_MAX}};
    for (const auto& bounds : empty_regions) {
        CLIPOBJ clip{};
        clip.rclBounds = bounds;
        clip.iDComplexity = DC_TRIVIAL;
        clip.iFComplexity = FC_RECT;
        clip.iUniq = 77;
        clip.iMode = TC_RECTANGLES;
        Rx7900NormalizeClipRegion(&clip);
        expect(clip.iDComplexity == DC_RECT && clip.iFComplexity == FC_RECT,
            "explicit empty visible region clips every pixel");
        expect(clip.rclBounds.left == bounds.left && clip.rclBounds.top == bounds.top &&
            clip.rclBounds.right == bounds.right && clip.rclBounds.bottom == bounds.bottom &&
            clip.iUniq == 77 && clip.iMode == TC_RECTANGLES,
            "normalization preserves rectangle and unrelated clip state");
        Rx7900NormalizeClipRegion(&clip);
        expect(clip.iDComplexity == DC_RECT, "empty normalization is idempotent");
    }

    const RECTL visible_regions[]{
        {0, 0, 300, 300}, {-10, -20, 1, 1},
        {INT32_MIN, INT32_MIN, INT32_MAX, INT32_MAX}};
    const BYTE complexities[]{DC_TRIVIAL, DC_RECT, DC_COMPLEX};
    for (BYTE complexity : complexities) {
        for (const auto& bounds : visible_regions) {
            CLIPOBJ clip{};
            clip.rclBounds = bounds;
            clip.iDComplexity = complexity;
            clip.iFComplexity = FC_COMPLEX;
            Rx7900NormalizeClipRegion(&clip);
            expect(clip.iDComplexity == complexity && clip.iFComplexity == FC_COMPLEX,
                "nonempty region and genuine unbounded sentinel retain semantics");
        }
    }

    CLIPOBJ complex_empty{};
    complex_empty.iDComplexity = DC_COMPLEX;
    complex_empty.iFComplexity = FC_COMPLEX;
    Rx7900NormalizeClipRegion(&complex_empty);
    expect(complex_empty.iDComplexity == DC_COMPLEX && complex_empty.iFComplexity == FC_COMPLEX,
        "existing complex enumeration remains owned by Win32k");
    printf("Clip visibility: %u checks, %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

/*
Module: lightning.reactos.line_coverage
Purpose: Produce bounded cosmetic-line coverage for the existing GPU mask path.
Created: 2026-10-22
Architecture: ReactOS lineto.c integer coverage; GPU owns brush/ROP composition.
License: GPL-2.0-or-later, derived from ReactOS win32ss/gdi/eng/lineto.c.
*/
#pragma once
#include "framebuffer_shadow_policy.hpp"
#include <stddef.h>
#include <stdint.h>

namespace lightning::reactos {
struct LinePoint final { int32_t x=0,y=0; };
using LineRectangle=rx7900::reactos::framebuffer::Rectangle;
struct LineMask final {
    uint8_t* pixels=nullptr;
    size_t bytes=0;
    uint32_t stride=0;
    LineRectangle bounds{};
};

// Clip the conservative integer endpoint bounds to the supported canvas.
// Empty lines succeed with empty bounds; no unbounded endpoint subtraction.
bool line_bounds(LinePoint first,LinePoint last,uint32_t width,uint32_t height,
    LineRectangle& output) noexcept;
// The caller exclusively owns the complete positive-pitch DWORD mono plane.
// Validate all bounds before clearing/writing it. Work is bounded by the visible
// major-axis extent, even for INT32_MIN/INT32_MAX endpoints. Endpoint is excluded.
bool line_coverage(LineMask mask,LinePoint first,LinePoint last) noexcept;
// Convert the foreground ROP2 truth table to a pattern-only ROP3. Mask-zero
// pixels preserve the destination independently of the line's background MIX.
bool line_rop(uint32_t mix,uint32_t& rop4) noexcept;
// EngLineTo writes the brush's raw iSolidColor with DIB_PutPixel/HLine/VLine
// and never reads MIX, a realization or a brush origin. An unrealized pattern
// brush (iSolidColor 0xffffffff, eng/engbrush.c) therefore paints all32 bits:
// WHITENESS under line coverage, DSTCOPY elsewhere, independent of MIX.
constexpr uint32_t line_unrealized_pattern_rop4=0xaaffU;
}

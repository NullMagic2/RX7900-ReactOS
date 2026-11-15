/*
Module: rx7900.tests.dib_floodfill_reference
Purpose: Test-only port of the pinned engine flood fill used as the exact reference for
the opaque flood-fill guard.
Created: 2026-10-24
Architecture: C++ port of win32ss/gdi/dib/floodfill.c DIB_XXBPP_FloodFillSolid
(identical in a97c8a2 and 822e864) for 32 bpp SURFOBJs addressed through pvScan0 and
lDelta: the same four-neighbour stack order, rectangle test and "Can't finish
flooding" early stop. Include after the SDK GDI types. Never compiled into a driver.
*/
#pragma once

#include <stdint.h>
#include <stdlib.h>

namespace rx7900::tests::floodfill {

inline uint32_t& pixel(SURFOBJ& surface, LONG x, LONG y) {
    return *(reinterpret_cast<uint32_t*>(static_cast<uint8_t*>(surface.pvScan0) +
                                         static_cast<intptr_t>(y) * surface.lDelta) + x);
}

struct Item {
    ULONG x;
    ULONG y;
};

inline bool in_rect(const RECTL& rect, ULONG x, ULONG y) {
    return static_cast<LONG>(x) >= rect.left && static_cast<LONG>(x) < rect.right &&
           static_cast<LONG>(y) >= rect.top && static_cast<LONG>(y) < rect.bottom;
}

// FLOODFILLBORDER (0) and FLOODFILLSURFACE (1) as wingdi.h; the port keeps the
// unsigned coordinate arithmetic of the original (x - 1 at 0 wraps and fails the
// rectangle test).
inline BOOL fill_solid(SURFOBJ& surface, BRUSHOBJ& brush, const RECTL& rect, const POINTL& origin, ULONG color,
                       UINT type) {
    const ULONG brush_color = brush.iSolidColor;
    ULONG x = static_cast<ULONG>(origin.x);
    ULONG y = static_cast<ULONG>(origin.y);
    const bool surface_mode = type == FLOODFILLSURFACE;
    if (type != FLOODFILLBORDER && type != FLOODFILLSURFACE) {
        return FALSE;
    }

    if (surface_mode ? pixel(surface, x, y) != color : pixel(surface, x, y) == color) {
        return FALSE;
    }

    const ULONG width = static_cast<ULONG>(rect.right - rect.left);
    const ULONG height = static_cast<ULONG>(rect.bottom - rect.top);
    Item* data = static_cast<Item*>(malloc(sizeof(Item) * width * height));
    if (data == nullptr) {
        return FALSE;
    }

    Item* start = data + width * height;
    ULONG length = 0;
    auto add = [&](ULONG ax, ULONG ay) {
        if (!in_rect(rect, ax, ay)) {
            return;
        }

        const uint32_t value = pixel(surface, static_cast<LONG>(ax), static_cast<LONG>(ay));
        if (surface_mode ? value != color : value == color) {
            return;
        }

        --start;
        start->x = ax;
        start->y = ay;
        ++length;
    };

    add(x, y);
    while (length != 0) {
        x = start->x;
        y = start->y;
        ++start;
        --length;
        pixel(surface, static_cast<LONG>(x), static_cast<LONG>(y)) = brush_color;
        if (start - 4 < data) {
            free(data);
            return FALSE;
        }

        add(x, y + 1);
        add(x, y - 1);
        add(x + 1, y);
        add(x - 1, y);
    }

    free(data);
    return TRUE;
}

} // namespace rx7900::tests::floodfill

/*
Module: lightning.reactos.brush_snapshot
Purpose: Describe only this driver's engine-owned canonical brush realization.
Created: 2026-10-21
Architecture: BRUSHOBJ owns allocation lifetime; no engine-private type casts.
*/
#pragma once
#include <stdint.h>
#include "gdi_protocol.hpp"
#include "line_coverage.hpp"

namespace lightning::reactos {
constexpr uint32_t brush_snapshot_magic=0x3142524cU;
struct BrushSnapshot final {
    uint32_t magic=brush_snapshot_magic;
    uint32_t size=0;
    uint32_t width=0;
    uint32_t height=0;
    uint32_t stride=0;
    uint32_t pixel_bytes=0;
    uint32_t mask_width=0;
    uint32_t mask_height=0;
    uint32_t mask_stride=0;
    uint32_t mask_bytes=0;
};

struct BrushPixels final {
    const uint8_t* pixels=nullptr;
    uint32_t width=0;
    uint32_t height=0;
    uint32_t stride=0;
    uint32_t bytes=0;
};

inline const BrushSnapshot* brush_snapshot(BRUSHOBJ* object) noexcept {
    if (object==nullptr || object->iSolidColor!=UINT32_MAX) return nullptr;
    // The engine invokes our DrvRealizeBrush to create this exact native object
    // and keeps it alive under the brush/PDEV lock through this synchronous use.
    const auto* brush=static_cast<const BrushSnapshot*>(BRUSHOBJ_pvGetRbrush(object));
    if (brush==nullptr || brush->magic!=brush_snapshot_magic || brush->width==0 || brush->height==0 ||
        static_cast<uint64_t>(brush->width)*4U!=brush->stride ||
        static_cast<uint64_t>(brush->stride)*brush->height!=brush->pixel_bytes ||
        brush->pixel_bytes>UINT32_MAX-sizeof(BrushSnapshot) || brush->size>LIGHTNING_GDI_MAX_PIXEL_BYTES) return nullptr;
    const uint32_t prefix=sizeof(BrushSnapshot)+brush->pixel_bytes;
    if (brush->mask_bytes==0) {
        if (brush->mask_width!=0 || brush->mask_height!=0 || brush->mask_stride!=0 || brush->size!=prefix)
            return nullptr;
    } else {
        const uint64_t stride=((static_cast<uint64_t>(brush->mask_width)+31U)/32U)*sizeof(uint32_t);
        if (brush->mask_width==0 || brush->mask_height==0 || stride!=brush->mask_stride ||
            stride*brush->mask_height!=brush->mask_bytes || brush->mask_bytes>UINT32_MAX-prefix ||
            brush->size!=prefix+brush->mask_bytes) return nullptr;
    }
    if (brush->size>UINTPTR_MAX-reinterpret_cast<uintptr_t>(brush)) return nullptr;
    return brush;
}

inline bool brush_pixels(BRUSHOBJ* object,BrushPixels& result) noexcept {
    result={};
    const auto* brush=brush_snapshot(object);
    if (brush==nullptr) return false;
    result={reinterpret_cast<const uint8_t*>(brush)+sizeof(BrushSnapshot),brush->width,brush->height,
        brush->stride,brush->pixel_bytes};
    return true;
}

// The optional1bpp mask follows the canonical32bpp tile in the same engine-owned
// realization. It is obtained from DrvRealizeBrush, never from EBRUSHOBJ casts.
inline bool brush_mask(BRUSHOBJ* object,BrushPixels& result) noexcept {
    result={};
    const auto* brush=brush_snapshot(object);
    if (brush==nullptr || brush->mask_bytes==0) return false;
    result={reinterpret_cast<const uint8_t*>(brush)+sizeof(BrushSnapshot)+brush->pixel_bytes,
        brush->mask_width,brush->mask_height,brush->mask_stride,brush->mask_bytes};
    return true;
}

// ReactOS stock hatch bitmaps encode ink with zero bits (eng/engbrush.c),
// while the canonical mask contract selects foreground with one bits. Only
// stock hatch realizations change polarity; arbitrary DDI masks stay intact.
inline bool normalize_hatch_mask(uint8_t* pixels,uint32_t bytes,uint32_t stride,
    uint32_t width,uint32_t height,uint32_t hatch) noexcept {
    if (hatch>=HS_DDI_MAX) return true;
    if (pixels==nullptr || width==0 || height==0 ||
        ((static_cast<uint64_t>(width)+31U)/32U)*sizeof(uint32_t)!=stride ||
        static_cast<uint64_t>(stride)*height!=bytes || bytes>UINTPTR_MAX-reinterpret_cast<uintptr_t>(pixels)) return false;
    for (uint32_t y=0;y<height;++y) {
        auto* row=pixels+static_cast<size_t>(y)*stride;
        for (uint32_t x=0;x<width;++x) row[x/8U]^=static_cast<uint8_t>(0x80U>>(x&7U));
    }
    return true;
}

// Match the engine MIX_TO_ROP4 low-nibble wrapping, including a zero MIX byte.
// line_rop supplies the shared ROP2 truth-table expansion, with no copied table.
inline bool paint_rop4(uint32_t mix,uint32_t& output) noexcept {
    uint32_t foreground=0;
    uint32_t background=0;
    if (!line_rop(((mix-1U)&15U)+1U,foreground) ||
        !line_rop((((mix>>8)-1U)&15U)+1U,background)) return false;
    output=(foreground&UINT8_MAX)|((background&UINT8_MAX)<<8);
    return true;
}

// Classification only: repeat an owned monochrome brush mask at its signed
// brush origin. Existing GPU/engine BitBlt performs every color/ROP operation.
inline bool tile_brush_mask(const BrushPixels& mask,uint8_t* destination,uint32_t bytes,uint32_t stride,
    rx7900::reactos::framebuffer::Rectangle bounds,int32_t origin_x,int32_t origin_y) noexcept {
    if (mask.pixels==nullptr || destination==nullptr || mask.width==0 || mask.height==0 ||
        bounds.left<0 || bounds.top<0 || bounds.right<=bounds.left || bounds.bottom<=bounds.top ||
        mask.bytes>UINTPTR_MAX-reinterpret_cast<uintptr_t>(mask.pixels) ||
        bytes>UINTPTR_MAX-reinterpret_cast<uintptr_t>(destination)) return false;
    const uint64_t source_stride=((static_cast<uint64_t>(mask.width)+31U)/32U)*sizeof(uint32_t);
    const uint64_t width=static_cast<uint64_t>(bounds.right)-bounds.left;
    const uint64_t height=static_cast<uint64_t>(bounds.bottom)-bounds.top;
    if (source_stride!=mask.stride || source_stride*mask.height!=mask.bytes ||
        stride%sizeof(uint32_t)!=0 || width>static_cast<uint64_t>(stride)*8U ||
        static_cast<uint64_t>(stride)*height>bytes || width>LIGHTNING_GDI_MAX_DIMENSION ||
        height>LIGHTNING_GDI_MAX_DIMENSION) return false;
    const uintptr_t source=reinterpret_cast<uintptr_t>(mask.pixels);
    const uintptr_t target=reinterpret_cast<uintptr_t>(destination);
    if (source<target+bytes && target<source+mask.bytes) return false;
    uint32_t phase_x=0;
    uint32_t phase_y=0;
    if (!::lightning::gdi::normalize_pattern_origin(origin_x,mask.width,phase_x) ||
        !::lightning::gdi::normalize_pattern_origin(origin_y,mask.height,phase_y)) return false;
    for (uint32_t y=0;y<height;++y) {
        auto* row=destination+static_cast<size_t>(y)*stride;
        for (uint32_t byte=0;byte<stride;++byte) row[byte]=0;
        const uint32_t source_y=static_cast<uint32_t>((static_cast<uint64_t>(bounds.top)+y+mask.height-phase_y)%mask.height);
        const auto* source_row=mask.pixels+static_cast<size_t>(source_y)*mask.stride;
        for (uint32_t x=0;x<width;++x) {
            const uint32_t source_x=static_cast<uint32_t>((static_cast<uint64_t>(bounds.left)+x+mask.width-phase_x)%mask.width);
            if ((source_row[source_x/8U]&(0x80U>>(source_x&7U)))!=0)
                row[x/8U]|=static_cast<uint8_t>(0x80U>>(x&7U));
        }
    }
    return true;
}
} // namespace lightning::reactos

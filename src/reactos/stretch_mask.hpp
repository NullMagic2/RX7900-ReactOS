/*
Module: lightning.reactos.stretch_mask
Purpose: Evaluate the engine's masked-stretch draw decision as a destination plane.
Created: 2026-10-22
Architecture: Header-only, allocation-free and SDK-independent. The GDI client
owns the SURFOBJ checks and staging; the GPU applies the ROP under this plane
(LIGHTNING_GDI_CAP_STRETCH_MASK). Ported from ReactOS win32ss/gdi/eng/stretchblt.c
EngStretchBltROP (per-clip InputToCombinedRect) and dib/stretchblt.c
DIB_XXBPP_StretchBlt (mask test) at a97c8a2 and 822e864.
License: GPL-2.0-or-later.
*/
#pragma once
#include <stddef.h>
#include <stdint.h>
#include "framebuffer_shadow_policy.hpp"
#include "hw/misc/lightning-gdi.h"

namespace lightning::reactos {

// A borrowed 1bpp DDI mask: MSB-first rows, scan0_offset bytes into bits, signed
// row delta. stretch_mask_plane proves every row it reads lies inside bytes.
struct StretchMaskSource final {
    const uint8_t* bits=nullptr;
    size_t bytes=0;
    size_t scan0_offset=0;
    int64_t delta=0;
    int32_t width=0;
    int32_t height=0;
};

// The well-ordered prclDest/prclSrc pair DrvStretchBltROP received, plus the
// paired stretch-guard orientation. The caller guarantees EngStretchBltROP's
// rclBounds pre-clip is a no-op for output (output inside rclBounds and canvas),
// so every engine CombinedRect equals one supplied clip rectangle.
struct StretchMaskCall final {
    rx7900::reactos::framebuffer::Rectangle output{};
    rx7900::reactos::framebuffer::Rectangle input{};
    bool mirror_x=false;
    bool mirror_y=false;
};

namespace stretch_mask_detail {
inline int64_t sample(int64_t low,int64_t high,int64_t offset,int64_t extent,bool mirror) noexcept {
    const int64_t step=offset*(high-low)/extent;
    return mirror ? high-step : low+step;
}

inline int64_t mapped(int64_t input,int64_t offset,int64_t input_extent,int64_t output_extent) noexcept {
    return input+offset*input_extent/output_extent;
}
}

// Writes the MSB-first plane for bounds (DWORD rows of stride bytes): bit1 where
// DIB_XXBPP_StretchBlt's mask test lets the pixel draw, zero elsewhere and
// outside every clip. Zero clips means bounds itself. Returns false, after
// clearing the plane, when geometry is invalid or any sample lands exactly on
// the mask's right or bottom edge: a97c8a2 skips those samples while 822e864
// reads one bit past the bitmap, so only the engine fallback is exact there.
inline bool stretch_mask_plane(const StretchMaskCall& call,const StretchMaskSource& mask,
    const LightningDisplayRect* clips,uint32_t count,rx7900::reactos::framebuffer::Rectangle bounds,
    uint8_t* plane,size_t bytes,uint32_t stride) noexcept {
    using namespace stretch_mask_detail;
    const int64_t width=static_cast<int64_t>(bounds.right)-bounds.left;
    const int64_t height=static_cast<int64_t>(bounds.bottom)-bounds.top;
    if (plane==nullptr || width<=0 || height<=0 || width>LIGHTNING_GDI_MAX_DIMENSION ||
        height>LIGHTNING_GDI_MAX_DIMENSION || stride%sizeof(uint32_t)!=0 ||
        static_cast<uint64_t>(width)>uint64_t{stride}*8 || uint64_t{stride}*static_cast<uint64_t>(height)>bytes) return false;
    for (size_t index=0;index<bytes;++index) plane[index]=0;
    const int64_t output_width=static_cast<int64_t>(call.output.right)-call.output.left;
    const int64_t output_height=static_cast<int64_t>(call.output.bottom)-call.output.top;
    const int64_t input_width=static_cast<int64_t>(call.input.right)-call.input.left;
    const int64_t input_height=static_cast<int64_t>(call.input.bottom)-call.input.top;
    const uint64_t row_bytes=(static_cast<uint64_t>(mask.width)+7)/8;
    const int64_t magnitude=mask.delta<0 ? -mask.delta : mask.delta;
    if (count>LIGHTNING_GDI_MAX_RECTS || (count!=0 && clips==nullptr) || mask.bits==nullptr ||
        output_width<=0 || output_height<=0 || input_width<=0 || input_height<=0 ||
        output_width>LIGHTNING_GDI_MAX_DIMENSION || output_height>LIGHTNING_GDI_MAX_DIMENSION ||
        input_width>LIGHTNING_GDI_MAX_DIMENSION || input_height>LIGHTNING_GDI_MAX_DIMENSION ||
        mask.width<=0 || mask.height<=0 || static_cast<uint64_t>(magnitude)<row_bytes ||
        mask.scan0_offset>mask.bytes || mask.bytes>UINTPTR_MAX-reinterpret_cast<uintptr_t>(mask.bits)) return false;
    // Prove both extreme rows before any read; intermediate rows lie between.
    const int64_t last_row=static_cast<int64_t>(mask.scan0_offset)+(mask.height-1)*mask.delta;
    if (last_row<0 || static_cast<uint64_t>(last_row)+row_bytes>mask.bytes ||
        mask.scan0_offset+row_bytes>mask.bytes) return false;
    for (uint32_t index=0;index<(count==0 ? 1U : count);++index) {
        const rx7900::reactos::framebuffer::Rectangle clip=count==0 ? bounds :
            rx7900::reactos::framebuffer::Rectangle{static_cast<int32_t>(clips[index].x),
                static_cast<int32_t>(clips[index].y),static_cast<int32_t>(clips[index].x+clips[index].width),
                static_cast<int32_t>(clips[index].y+clips[index].height)};
        const int64_t clip_width=static_cast<int64_t>(clip.right)-clip.left;
        const int64_t clip_height=static_cast<int64_t>(clip.bottom)-clip.top;
        if (clip_width<=0 || clip_height<=0 || clip.left<bounds.left || clip.top<bounds.top ||
            clip.right>bounds.right || clip.bottom>bounds.bottom || clip.left<call.output.left ||
            clip.top<call.output.top || clip.right>call.output.right || clip.bottom>call.output.bottom) {
            for (size_t byte=0;byte<bytes;++byte) plane[byte]=0;
            return false;
        }
        // EngStretchBltROP InputToCombinedRect, then the DIB integer sampler.
        const int64_t left=mapped(call.input.left,clip.left-call.output.left,input_width,output_width);
        const int64_t right=mapped(call.input.left,clip.right-call.output.left,input_width,output_width);
        const int64_t top=mapped(call.input.top,clip.top-call.output.top,input_height,output_height);
        const int64_t bottom=mapped(call.input.top,clip.bottom-call.output.top,input_height,output_height);
        for (int64_t row=0;row<clip_height;++row) {
            const int64_t sy=sample(top,bottom,row,clip_height,call.mirror_y);
            uint8_t* target=plane+static_cast<size_t>(clip.top-bounds.top+row)*stride;
            for (int64_t column=0;column<clip_width;++column) {
                const int64_t sx=sample(left,right,column,clip_width,call.mirror_x);
                if (sx<0 || sy<0 || sx>mask.width || sy>mask.height) continue;
                if (sx==mask.width || sy==mask.height) {
                    for (size_t byte=0;byte<bytes;++byte) plane[byte]=0;
                    return false;
                }
                const auto offset=static_cast<size_t>(static_cast<int64_t>(mask.scan0_offset)+sy*mask.delta)+
                    static_cast<size_t>(sx/8);
                if ((mask.bits[offset]&(0x80U>>(sx%8)))!=0) continue;
                const size_t x=static_cast<size_t>(clip.left-bounds.left+column);
                target[x/8]|=static_cast<uint8_t>(0x80U>>(x%8));
            }
        }
    }
    return true;
}
}

/*
Module: lightning.tests.brush_snapshot
Purpose: Check production brush ownership, Paint MIX and repeated mask phases.
Created: 2026-10-22
Architecture: Actual GDI SDK boundary; no independent color/ROP rasterizer.
*/
#include "framebuf.h"
#include "brush_snapshot.hpp"
#include <stdio.h>
#include <string.h>

namespace {
unsigned checks=0;
unsigned failures=0;
void check(bool condition,const char* description) {
    ++checks;
    if (!condition) {++failures; printf("FAIL %s\n",description);}
}
}

extern "C" PVOID APIENTRY BRUSHOBJ_pvGetRbrush(BRUSHOBJ* brush) {return brush->pvRbrush;}

int main() {
    using namespace lightning::reactos;
    struct Realization final {BrushSnapshot header; uint32_t pixels[6]; uint8_t mask[8];} realization{};
    realization.header.size=sizeof(realization);
    realization.header.width=3;
    realization.header.height=2;
    realization.header.stride=12;
    realization.header.pixel_bytes=sizeof(realization.pixels);
    realization.header.mask_width=3;
    realization.header.mask_height=2;
    realization.header.mask_stride=4;
    realization.header.mask_bytes=sizeof(realization.mask);
    realization.mask[0]=0xa0;
    realization.mask[4]=0x40;
    BRUSHOBJ brush{};
    brush.iSolidColor=UINT32_MAX;
    brush.pvRbrush=&realization;
    BrushPixels pixels{};
    BrushPixels mask{};
    check(brush_pixels(&brush,pixels) && pixels.width==3 && pixels.bytes==24 &&
        brush_mask(&brush,mask) && mask.width==3 && mask.bytes==8 && mask.pixels==realization.mask,
        "pattern and monochrome mask borrow independent checked spans from the same realization owner");
    uint32_t rop=0;
    check(paint_rop4(0x0b0d,rop) && rop==0xaaf0,
        "transparent Paint retains foreground COPYPEN and background NOP");
    check(paint_rop4(0x0d0d,rop) && rop==0xf0f0,"opaque Paint retains equal foreground/background operation");
    check(paint_rop4(0,rop) && rop==0xffff,"zero MIX bytes wrap to WHITE as the actual engine macro does");
    check(paint_rop4(0x1b1d,rop) && rop==0xaaf0,"engine MIX semantics use the low nibble of each byte");
    uint8_t output[20]{};
    memset(output,0xcd,sizeof(output));
    check(tile_brush_mask(mask,output,16,4,{1,2,8,6},-1,-1) && output[16]==0xcd,
        "signed origin and nonzero destination bounds stay within owned output bytes");
    check(output[0]==0x24 && output[4]==0xda && output[8]==0x24 && output[12]==0xda &&
        output[1]==0 && output[2]==0 && output[3]==0,
        "mask rows repeat at the brush phase with MSB-first bits and initialized padding");
    uint8_t shifted[16]{};
    check(tile_brush_mask(mask,shifted,sizeof(shifted),4,{1,2,8,6},2,1) &&
        memcmp(output,shifted,sizeof(shifted))==0,"equivalent signed brush phases produce identical repeated mask bytes");
    memset(shifted,0xaa,sizeof(shifted));
    check(!tile_brush_mask(mask,shifted,15,4,{1,2,8,6},-1,-1) && shifted[0]==0xaa,
        "truncated destination rejects before any mask mutation");
    check(!tile_brush_mask(mask,realization.mask,mask.bytes,mask.stride,{0,0,3,2},0,0),
        "in-place repeated masks cannot mutate their immutable source");
    check(normalize_hatch_mask(realization.mask,sizeof(realization.mask),4,3,2,HS_DIAGCROSS) &&
        realization.mask[0]==0x40 && realization.mask[4]==0xa0 && realization.mask[1]==0 && realization.mask[7]==0,
        "stock hatch zero-bit ink becomes canonical foreground bits without touching padding");
    check(normalize_hatch_mask(realization.mask,sizeof(realization.mask),4,3,2,UINT32_MAX) &&
        realization.mask[0]==0x40 && realization.mask[4]==0xa0,
        "ordinary realized masks retain their original foreground polarity");
    check(!normalize_hatch_mask(realization.mask,7,4,3,2,HS_DIAGCROSS) && realization.mask[0]==0x40,
        "truncated hatch normalization rejects before modifying a row");
    check(normalize_hatch_mask(realization.mask,sizeof(realization.mask),4,3,2,HS_DIAGCROSS) &&
        realization.mask[0]==0xa0 && realization.mask[4]==0x40,"hatch bit normalization preserves every meaningful bit");
    --realization.header.size;
    check(!brush_pixels(&brush,pixels) && !brush_mask(&brush,mask),"truncated native realization exposes neither span");
    ++realization.header.size;
    ++realization.header.mask_stride;
    check(!brush_snapshot(&brush),"malformed word pitch cannot expose a mask");
    --realization.header.mask_stride;
    realization.header.mask_width=0;
    check(!brush_snapshot(&brush),"nonempty mask cannot have zero width");
    realization.header.mask_width=UINT32_MAX;
    check(!brush_snapshot(&brush),"overflowing mask geometry cannot wrap through native span validation");
    realization.header.mask_width=3;
    realization.header.mask_width=realization.header.mask_height=realization.header.mask_stride=realization.header.mask_bytes=0;
    realization.header.size=sizeof(BrushSnapshot)+realization.header.pixel_bytes;
    check(brush_pixels(&brush,pixels) && !brush_mask(&brush,mask),"unmasked pattern realization has no fabricated transparency mask");
    brush.iSolidColor=0x123456;
    check(!brush_snapshot(&brush),"solid brush never interprets an unrelated realization pointer");
    printf("Production brush/MIX/mask policy: %u checks, %u failures\n",checks,failures);
    return failures==0 ? 0 : 1;
}

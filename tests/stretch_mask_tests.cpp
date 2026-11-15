/*
Module: lightning.tests.stretch_mask
Purpose: Prove the masked-stretch GPU contract pixel-exact against the engine port.
Created: 2026-10-22
Architecture: Host C++17. The production stretch_mask plane, shared protocol
validator and per-clip mapping run unchanged; the GPU fragment rule is
modelled from gdi_gpu.cpp (its actual execution is the opt-in --gpu fixture).
The oracle is gdi_engine_reference.hpp (IntEngStretchBlt, EngStretchBltROP,
DIB_XXBPP_StretchBlt), read-only and owned by the radeondisp work.
*/
#include "gdi_engine_reference.hpp"
#include "stretch_mask.hpp"
#include "gdi_protocol.hpp"
#include <cstdio>
#include <vector>

namespace {
namespace ref=gdi_reference;
namespace core=lightning::gdi;
using Rectangle=rx7900::reactos::framebuffer::Rectangle;
unsigned checks=0,failures=0,compared=0,declined_geometry=0,declined_edge=0,declined_rop=0;

void check(bool condition,const char* message) {
    ++checks;
    if (!condition) {++failures;std::printf("FAIL %s\n",message);}
}

struct Random final {
    uint32_t state=0x12345678U;
    uint32_t next() {state=state*1664525U+1013904223U;return state;}
};

constexpr int32_t canvas_width=24,canvas_height=18;
constexpr uint32_t destination_copy=0xaaU,source_copy=0xccU;

struct Case final {
    ref::Rect destination{},source{};
    ref::Clip clip{};
    const ref::Bitmap1* mask=nullptr;
    ref::Brush brush{};
    ref::Point origin{};
    uint32_t rop4=0;
};

Rectangle rectangle(const ref::Rect& value) {return {value.left,value.top,value.right,value.bottom};}

bool inside(const ref::Rect& inner,const ref::Rect& outer) {
    return outer.left<=inner.left && outer.top<=inner.top && outer.right>=inner.right && outer.bottom>=inner.bottom;
}

uint32_t pixel(const std::vector<uint8_t>& bytes,size_t index) {
    return uint32_t{bytes[index]}|(uint32_t{bytes[index+1]}<<8)|(uint32_t{bytes[index+2]}<<16)|(uint32_t{bytes[index+3]}<<24);
}

// Every DIB sample the engine takes, from its own recorded CombinedRect and
// InputToCombinedRect: true when one lands on the mask's right/bottom edge,
// where a97c8a2 (skip) and 822e864 (reads one bit past the bitmap) differ.
bool edge_sample(const std::vector<ref::DibCall>& calls,const ref::Bitmap1& mask) {
    for (const auto& call:calls) {
        ref::Rect d=call.dest;
        const bool mirror_x=d.left>d.right,mirror_y=d.top>d.bottom;
        ref::make_well_ordered(d);
        const int32_t dw=d.right-d.left,dh=d.bottom-d.top;
        const int32_t sw=call.source.right-call.source.left,sh=call.source.bottom-call.source.top;
        for (int32_t y=d.top;y<d.bottom;++y) {
            const int32_t sy=mirror_y ? call.source.bottom-(y-d.top)*sh/dh : call.source.top+(y-d.top)*sh/dh;
            for (int32_t x=d.left;x<d.right;++x) {
                const int32_t sx=mirror_x ? call.source.right-(x-d.left)*sw/dw : call.source.left+(x-d.left)*sw/dw;
                if (sx>=0 && sy>=0 && sx<=mask.cx && sy<=mask.cy && (sx==mask.cx || sy==mask.cy)) return true;
            }
        }
    }
    return false;
}

// Runs one DrvStretchBltROP through the Lightning masked path and the engine.
void run(const Case& test,const ref::Bitmap32& canvas,const ref::Bitmap32& source) {
    const ref::DriverStretch call=ref::int_eng_stretch_blt(test.destination,test.source,test.clip);
    if (call.bitblt || call.nothing) return;
    ref::Bitmap32 engine=canvas;
    std::vector<ref::DibCall> calls;
    const ref::Source engine_source{&source,nullptr,0,0,false};
    ref::eng_stretch_blt_rop(engine,engine_source,test.mask,&test.clip,&test.origin,
        ref::fallback_destination(call),call.input,&test.brush,test.rop4,&calls);
    // framebuffer_shadow admission: the rclBounds pre-clip must be a no-op.
    const ref::Rect output=call.output;
    if (output.left<0 || output.top<0 || output.right>canvas_width || output.bottom>canvas_height ||
        !inside(output,test.clip.bounds)) {++declined_geometry;return;}
    // capture_clip: bounds and canonical clip rectangles.
    ref::Rect bounds=output;
    if (test.clip.kind!=ref::Complexity::trivial && !ref::intersect(bounds,output,test.clip.bounds)) return;
    std::vector<LightningDisplayRect> clips;
    if (test.clip.kind==ref::Complexity::complex) {
        for (const auto& item:test.clip.rects) {
            ref::Rect clipped{};
            if (ref::intersect(clipped,item,bounds)) clips.push_back({uint32_t(clipped.left),uint32_t(clipped.top),
                uint32_t(clipped.right-clipped.left),uint32_t(clipped.bottom-clipped.top)});
        }
        if (clips.empty()) return;
    }
    // lightning_gdi_client raster_operands: guarded upload snapshot.
    const bool mirror_x=call.flip_x,mirror_y=call.flip_y;
    const uint32_t prefix_x=mirror_x ? 1 : 0,prefix_y=mirror_y ? 1 : 0;
    const int32_t extent_x=call.input.right-call.input.left,extent_y=call.input.bottom-call.input.top;
    const int32_t left=call.input.left+int32_t(prefix_x),top=call.input.top+int32_t(prefix_y);
    if (left<0 || top<0 || left+extent_x>source.cx || top+extent_y>source.cy || extent_x<=0 || extent_y<=0) {
        ++declined_geometry;return;
    }
    const bool missing_x=prefix_x!=0 && left==0,missing_y=prefix_y!=0 && top==0;
    const uint32_t physical_width=uint32_t(extent_x)+prefix_x,physical_height=uint32_t(extent_y)+prefix_y;
    LightningGdiRasterOperands operands{};
    operands.size=sizeof(operands);
    operands.version=LIGHTNING_GDI_RASTER_VERSION;
    operands.foreground_rop=test.rop4&0xffU;
    operands.background_rop=destination_copy;
    operands.stretch_mode=LIGHTNING_GDI_STRETCH_COLORONCOLOR;
    operands.raster_flags=(mirror_x ? LIGHTNING_GDI_RASTER_MIRROR_X : 0U)|(mirror_y ? LIGHTNING_GDI_RASTER_MIRROR_Y : 0U);
    if (prefix_x|prefix_y) operands.raster_flags|=LIGHTNING_GDI_RASTER_GUARDED_SOURCE;
    if (missing_x) operands.raster_flags|=LIGHTNING_GDI_RASTER_INVALID_LEFT;
    if (missing_y) operands.raster_flags|=LIGHTNING_GDI_RASTER_INVALID_TOP;
    operands.original_destination={output.left,output.top,uint32_t(output.right-output.left),uint32_t(output.bottom-output.top)};
    operands.source_kind=LIGHTNING_GDI_ROP_SOURCE_UPLOAD;
    operands.source_rect={prefix_x,prefix_y,uint32_t(extent_x),uint32_t(extent_y)};
    operands.source_stride=physical_width*LIGHTNING_GDI_PIXEL_BYTES;
    operands.source_bytes=operands.source_stride*physical_height;
    const uint32_t width=uint32_t(bounds.right-bounds.left),height=uint32_t(bounds.bottom-bounds.top);
    const uint32_t mask_stride=((width+31U)/32U)*4U;
    operands.mask_kind=LIGHTNING_GDI_MASK_MONO1;
    operands.mask_space=LIGHTNING_GDI_MASK_SPACE_DESTINATION;
    operands.mask_offset=operands.source_bytes;
    operands.mask_width=width;
    operands.mask_height=height;
    operands.mask_stride=mask_stride;
    operands.mask_bytes=mask_stride*height;
    uint32_t payload_bytes=operands.mask_offset+operands.mask_bytes;
    const bool pattern_used=core::rop3_uses_pattern(operands.foreground_rop);
    const bool tiled=pattern_used && test.brush.solid==0xffffffffU;
    uint32_t origin_x=0,origin_y=0;
    if (pattern_used && !tiled) {
        operands.pattern_kind=LIGHTNING_GDI_PATTERN_SOLID;
        operands.solid_color=test.brush.solid;
    } else if (tiled) {
        check(core::normalize_pattern_origin(test.origin.x,8,origin_x) &&
            core::normalize_pattern_origin(test.origin.y,8,origin_y),"brush origin normalizes");
        operands.pattern_kind=LIGHTNING_GDI_PATTERN_TILE;
        operands.pattern_width=operands.pattern_height=8;
        operands.pattern_origin_x=int32_t(origin_x);
        operands.pattern_origin_y=int32_t(origin_y);
        operands.pattern_offset=payload_bytes;
        operands.pattern_stride=8*LIGHTNING_GDI_PIXEL_BYTES;
        operands.pattern_bytes=operands.pattern_stride*8;
        payload_bytes+=operands.pattern_bytes;
    }
    std::vector<uint8_t> payload(payload_bytes,0);
    for (uint32_t y=missing_y ? 1 : 0;y<physical_height;++y) for (uint32_t x=missing_x ? 1 : 0;x<physical_width;++x) {
        const uint32_t value=source.get(call.input.left+int32_t(x),call.input.top+int32_t(y));
        for (uint32_t byte=0;byte<4;++byte) payload[(y*physical_width+x)*4+byte]=uint8_t(value>>(8*byte));
    }
    if (tiled) for (uint32_t index=0;index<64;++index)
        for (uint32_t byte=0;byte<4;++byte) payload[operands.pattern_offset+index*4+byte]=uint8_t(test.brush.tile[index]>>(8*byte));
    // Production decision plane from the borrowed 1bpp mask.
    lightning::reactos::StretchMaskSource mask{};
    mask.bits=test.mask->bits.data();
    mask.bytes=test.mask->bits.size();
    mask.delta=test.mask->stride;
    mask.width=test.mask->cx;
    mask.height=test.mask->cy;
    lightning::reactos::StretchMaskCall plane_call{};
    plane_call.output=rectangle(output);
    plane_call.input=rectangle(call.input);
    plane_call.mirror_x=mirror_x;
    plane_call.mirror_y=mirror_y;
    const bool planned=lightning::reactos::stretch_mask_plane(plane_call,mask,clips.data(),uint32_t(clips.size()),
        rectangle(bounds),payload.data()+operands.mask_offset,operands.mask_bytes,mask_stride);
    const bool edge=edge_sample(calls,*test.mask);
    check(planned!=edge,"plane declines exactly when an engine sample lands on the mask edge");
    if (!planned) {++declined_edge;return;}
    const LightningDisplayRect destination{uint32_t(bounds.left),uint32_t(bounds.top),width,height};
    // DIB_XXBPP_StretchBlt evaluates only the foreground; without a source
    // there the client declines (the protocol requires source use).
    const bool admitted=core::validate_raster_inputs(canvas_width,canvas_height,destination,operands,payload_bytes);
    check(admitted==core::rop3_uses_source(operands.foreground_rop),
        "masked stretch operands pass the shared validator exactly when the foreground reads S");
    if (!admitted) {++declined_rop;return;}
    core::RasterSourceView view{};
    check(core::raster_source_view(operands,canvas_width,canvas_height,view),"source view");
    // gdi_gpu.cpp fragment rule, clip by clip.
    ref::Bitmap32 gpu=canvas;
    const uint8_t* plane=payload.data()+operands.mask_offset;
    const auto bit=[&](int32_t x,int32_t y) {
        return (plane[size_t(y)*mask_stride+size_t(x)/8]&(0x80U>>(x&7)))!=0;
    };
    const size_t count=clips.empty() ? 1 : clips.size();
    for (size_t index=0;index<count;++index) {
        const LightningDisplayRect clip=clips.empty() ? destination : clips[index];
        core::StretchMapping mapping{};
        check(core::raster_clip_mapping(operands,clip,mapping),"per-clip mapping");
        for (int32_t y=int32_t(clip.y);y<int32_t(clip.y+clip.height);++y) for (int32_t x=int32_t(clip.x);x<int32_t(clip.x+clip.width);++x) {
            const int32_t px=x-bounds.left,py=y-bounds.top;
            int32_t ox=(x-int32_t(clip.x))*int32_t(mapping.source.width)/int32_t(clip.width);
            int32_t oy=(y-int32_t(clip.y))*int32_t(mapping.source.height)/int32_t(clip.height);
            if (mirror_x) ox=int32_t(mapping.source.width)-1-ox;
            if (mirror_y) oy=int32_t(mapping.source.height)-1-oy;
            const int32_t sx=int32_t(mapping.source.x)+ox,sy=int32_t(mapping.source.y)+oy;
            const bool valid=sx>=int32_t(view.valid_rect.x) && sy>=int32_t(view.valid_rect.y) &&
                sx<int32_t(view.valid_rect.x+view.valid_rect.width) && sy<int32_t(view.valid_rect.y+view.valid_rect.height);
            const uint32_t s=valid ? pixel(payload,(size_t(sy)*physical_width+size_t(sx))*4) : 0;
            const uint32_t d=canvas.get(x,y);
            if (!valid && operands.foreground_rop==source_copy) continue;
            if (!bit(px,py)) continue;
            uint32_t p=operands.solid_color;
            if (tiled) {
                // PatternX restarts at the clip edge and counts drawn pixels.
                int32_t phase_x=int32_t(clip.x);
                for (int32_t column=int32_t(clip.x)-bounds.left;column<px;++column) phase_x+=bit(column,py) ? 1 : 0;
                const int32_t tx=(phase_x-int32_t(origin_x)+8)%8,ty=(y-int32_t(origin_y)+8)%8;
                p=pixel(payload,operands.pattern_offset+size_t(ty*8+tx)*4);
            }
            gpu.put(x,y,ref::do_rop(operands.foreground_rop,d,s,p));
        }
    }
    ++compared;
    check(gpu.px==engine.px,"masked stretch matches EngStretchBltROP/DIB_XXBPP_StretchBlt pixel for pixel");
}
}

int main() {
    Random random{};
    ref::Bitmap32 canvas(canvas_width,canvas_height),source(13,9);
    for (auto& value:canvas.px) value=random.next();
    for (auto& value:source.px) value=random.next();
    std::vector<ref::Bitmap1> masks{ref::Bitmap1(13,9),ref::Bitmap1(10,7),ref::Bitmap1(17,12),ref::Bitmap1(32,9)};
    for (auto& mask:masks) for (auto& byte:mask.bits) byte=uint8_t(random.next()>>13);
    std::vector<uint32_t> tile(64);
    for (auto& value:tile) value=random.next();
    std::vector<ref::Clip> clips(3);
    clips[1].kind=ref::Complexity::rect;
    clips[1].bounds={1,1,22,17};
    clips[2].kind=ref::Complexity::complex;
    clips[2].bounds={0,0,24,18};
    clips[2].rects={{0,0,24,4},{0,4,7,11},{12,4,24,11},{3,11,19,18}};
    constexpr int32_t widths[]{11,-11,4,-4,17,-17};
    constexpr int32_t heights[]{9,-9,3,-3,13,-13};
    constexpr int32_t source_widths[]{6,-6,3,-3};
    constexpr int32_t source_heights[]{4,-4,2,-2};
    constexpr uint32_t rops[]{0xaaccU,0xccaaU,0x66f0U,0xc0c0U,0x5a33U,0xb8ccU};
    const ref::Point origins[]{{0,0},{3,-5},{-13,7}};
    for (int32_t dw:widths) for (int32_t dh:heights) for (int32_t sw:source_widths) for (int32_t sh:source_heights) {
        const int32_t dx=dw<0 ? 21 : 2,dy=dh<0 ? 16 : 1,sx=sw<0 ? 10 : 2,sy=sh<0 ? 7 : 1;
        for (size_t clip=0;clip<clips.size();++clip) for (uint32_t rop:rops) {
            const size_t variant=size_t((dw+dh+sw+sh+int32_t(clip)+int32_t(rop))&0x7fff);
            Case test{};
            test.destination={dx,dy,dx+dw,dy+dh};
            test.source={sx,sy,sx+sw,sy+sh};
            test.clip=clips[clip];
            test.mask=&masks[variant%masks.size()];
            test.origin=origins[variant%3];
            test.brush=variant%2==0 ? ref::Brush{0xffffffffU,tile.data()} : ref::Brush{0x00c0ffeeU,nullptr};
            test.rop4=rop;
            run(test,canvas,source);
        }
    }
    check(compared>1000 && declined_edge>0,"cases cover GPU comparisons and engine-divergent mask edges");
    // Malformed borrowed masks are rejected before any read.
    std::vector<uint8_t> plane(64,0xc5);
    lightning::reactos::StretchMaskSource bad{masks[0].bits.data(),masks[0].bits.size(),0,masks[0].stride,13,9};
    lightning::reactos::StretchMaskCall bad_call{{0,0,8,8},{0,0,4,4},false,false};
    bad.height=100;
    check(!lightning::reactos::stretch_mask_plane(bad_call,bad,nullptr,0,{0,0,8,8},plane.data(),plane.size(),4),
        "mask rows beyond the borrowed bitmap are rejected");
    bad.height=9;
    bad.delta=-int64_t(masks[0].stride);
    check(!lightning::reactos::stretch_mask_plane(bad_call,bad,nullptr,0,{0,0,8,8},plane.data(),plane.size(),4),
        "bottom-up delta cannot walk before the bitmap start");
    bad.scan0_offset=size_t(masks[0].stride)*8;
    check(lightning::reactos::stretch_mask_plane(bad_call,bad,nullptr,0,{0,0,8,8},plane.data(),plane.size(),4),
        "bottom-up mask with its scan0 at the last row is accepted");
    check(!lightning::reactos::stretch_mask_plane(bad_call,bad,nullptr,0,{0,0,8,8},plane.data(),31,4),
        "truncated plane is rejected");
    std::printf("Stretch mask: %u checks, %u failures (%u GPU comparisons; declines: %u geometry, %u mask edge, %u sourceless foreground)\n",
        checks,failures,compared,declined_geometry,declined_edge,declined_rop);
    return failures==0 ? 0 : 1;
}

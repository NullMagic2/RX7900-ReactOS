/*
Module: lightning.tests.line_coverage
Purpose: Check source-pinned octants, clipped huge endpoints and mask boundaries.
Created: 2026-10-22
Architecture: Actual portable geometry; independent iterative small-line oracle.
*/
#include "line_coverage.hpp"
#include "gdi_engine_reference.hpp"
#include <algorithm>
#include <array>
#include <vector>
#include <cstdio>

namespace {
unsigned checks=0,failures=0;
void check(bool condition,const char* message) {
    ++checks;
    if (!condition) {++failures;std::printf("FAIL %s\n",message);}
}
using namespace lightning::reactos;
constexpr int canvas=8;
void reference(std::array<bool,canvas*canvas>& pixels,LinePoint a,LinePoint b) {
    const int dx=b.x>=a.x ? b.x-a.x : a.x-b.x;
    const int dy=b.y>=a.y ? b.y-a.y : a.y-b.y;
    const int major=dx>=dy ? dx : dy;
    int error=major/2,x=a.x,y=a.y;
    for (int step=0;step<major;++step) {
        if (x>=0 && y>=0 && x<canvas && y<canvas) pixels[y*canvas+x]=true;
        if (dx>=dy) {
            x+=b.x>=a.x ? 1 : -1;
            error+=dy;
            if (error>=dx) {y+=b.y>=a.y ? 1 : -1;error-=dx;}
        } else {
            y+=b.y>=a.y ? 1 : -1;
            error+=dx;
            if (error>=dy) {x+=b.x>=a.x ? 1 : -1;error-=dy;}
        }
    }
}

// Unrealized pattern brush (iSolidColor 0xffffffff) through the production
// coverage plus line_unrealized_pattern_rop4, against the EngLineTo port: the
// engine writes raw iSolidColor with no tile, brush origin or MIX.
constexpr int large=20;
void pattern_brush_lines() {
    namespace ref=gdi_reference;
    const std::vector<std::vector<ref::Rect>> regions{
        {{0,0,large,large}},
        {{2,1,17,15}},
        {{0,0,large,5},{0,5,6,12},{11,5,large,12},{4,12,15,large}}};
    constexpr int ends[]{-4,0,3,9,16,23};
    uint32_t mismatches=0,lines=0;
    for (const auto& region:regions) for (int ax:ends) for (int ay:ends) for (int bx:ends) for (int by:ends) {
        std::array<uint32_t,large*large> engine{},gpu{};
        engine.fill(0x01020304U);
        gpu=engine;
        ref::eng_line_to(region,ax,ay,bx,by,[&](int32_t x,int32_t y) {
            if (x>=0 && y>=0 && x<large && y<large) engine[size_t(y)*large+size_t(x)]=0xffffffffU;
        });
        LineRectangle bounds{};
        bool valid=line_bounds({ax,ay},{bx,by},large,large,bounds);
        ref::Rect limit=region.front();
        for (const auto& rect:region) {
            limit.left=std::min(limit.left,rect.left);limit.top=std::min(limit.top,rect.top);
            limit.right=std::max(limit.right,rect.right);limit.bottom=std::max(limit.bottom,rect.bottom);
        }
        if (region.size()>1 || region.front().left!=0)
            bounds=rx7900::reactos::framebuffer::intersect(bounds,{limit.left,limit.top,limit.right,limit.bottom});
        if (!rx7900::reactos::framebuffer::empty(bounds)) {
            std::array<uint8_t,large*4> plane{};
            valid=valid && line_coverage({plane.data(),plane.size(),4,bounds},{ax,ay},{bx,by});
            const uint32_t foreground=line_unrealized_pattern_rop4&0xffU,background=line_unrealized_pattern_rop4>>8;
            for (const auto& rect:region) for (int y=std::max(rect.top,bounds.top);y<std::min(rect.bottom,bounds.bottom);++y)
                for (int x=std::max(rect.left,bounds.left);x<std::min(rect.right,bounds.right);++x) {
                    const int column=x-bounds.left;
                    const bool set=(plane[size_t(y-bounds.top)*4+size_t(column/8)]&(0x80U>>(column%8)))!=0;
                    auto& pixel=gpu[size_t(y)*large+size_t(x)];
                    pixel=ref::do_rop(set ? foreground : background,pixel,0,0x5a5a5a5aU);
                }
        }
        ++lines;
        if (!valid || gpu!=engine) ++mismatches;
    }
    check(lines==3*6*6*6*6 && mismatches==0,
        "pattern-brush lines in all octants and clip regions match EngLineTo's raw 0xffffffff");
    check(line_unrealized_pattern_rop4==0xaaffU,"pattern-brush line ROP ignores MIX and any tile");
}
}
int main() {
    pattern_brush_lines();
    using namespace lightning::reactos;
    constexpr int points[]{-3,0,1,5,12};
    for (int ax:points) for (int ay:points) for (int bx:points) for (int by:points) {
        const LinePoint a{ax,ay},b{bx,by};
        std::array<bool,canvas*canvas> expected{};
        reference(expected,a,b);
        std::array<uint8_t,40> storage{};
        storage.front()=storage.back()=0xc5;
        LineRectangle bounds{};
        bool valid=line_bounds(a,b,canvas,canvas,bounds);
        const bool empty=bounds.left>=bounds.right || bounds.top>=bounds.bottom;
        if (!empty) valid=valid && line_coverage({storage.data()+1,32,4,bounds},a,b);
        for (int y=0;y<canvas;++y) for (int x=0;x<canvas;++x) {
            const bool inside=!empty && x>=bounds.left && x<bounds.right && y>=bounds.top && y<bounds.bottom;
            const bool bit=inside && (storage[1+(y-bounds.top)*4+(x-bounds.left)/8]&
                (0x80U>>((x-bounds.left)%8)))!=0;
            valid=valid && bit==expected[y*canvas+x];
        }
        check(valid && storage.front()==0xc5 && storage.back()==0xc5,"all octants match iterative engine coverage and preserve canaries");
    }
    std::array<uint8_t,32> pixels{};
    const LineRectangle bounds{0,0,canvas,canvas};
    check(line_coverage({pixels.data(),pixels.size(),4,bounds},{INT32_MIN,INT32_MIN},{INT32_MAX,INT32_MAX}) &&
        pixels[0]==0x80 && pixels[4]==0x40 && pixels[28]==1,"huge diagonal visits only visible pixels with exact half-error");
    check(line_coverage({pixels.data(),pixels.size(),4,bounds},{INT32_MAX,INT32_MAX},{INT32_MIN,INT32_MIN}) &&
        pixels[0]==0x80 && pixels[4]==0x40 && pixels[28]==1,"reverse huge diagonal retains visible coverage");
    pixels.fill(0xc5);
    check(!line_coverage({pixels.data(),pixels.size()-1,4,bounds},{0,0},{7,7}) && pixels.front()==0xc5 &&
        pixels.back()==0xc5,"truncated plane rejected before any write");
    check(!line_coverage({pixels.data(),pixels.size(),3,bounds},{0,0},{7,7}) && pixels.front()==0xc5,
        "non-DWORD stride rejected before clearing");
    check(!line_coverage({nullptr,pixels.size(),4,bounds},{0,0},{7,7}),"null storage rejected");
    check(!line_coverage({reinterpret_cast<uint8_t*>(UINTPTR_MAX-8),32,4,bounds},{0,0},{7,7}),
        "wrapping address extent rejected without dereference");
    constexpr uint32_t tables[]{0x00,0x05,0x0a,0x0f,0x50,0x55,0x5a,0x5f,
        0xa0,0xa5,0xaa,0xaf,0xf0,0xf5,0xfa,0xff};
    for (uint32_t index=0;index<16;++index) {
        uint32_t rop=0;
        check(line_rop((index+1)|((index+1)<<8),rop) && rop==(0xaa00U|tables[index]),
            "all ROP2 tables preserve pixels outside line coverage");
    }
    uint32_t rop=99;
    check(!line_rop(0,rop) && rop==0,"invalid ROP2 has no usable output");
    LineRectangle invalid{1,1,2,2};
    check(!line_bounds({0,0},{1,1},0,canvas,invalid) && invalid.right==0,
        "invalid canvas returns empty geometry");
    std::printf("Line coverage: %u checks, %u failures\n",checks,failures);
    return failures==0 ? 0 : 1;
}

/*
Module: lightning.reactos.line_coverage
Purpose: Bound integer line coverage independently of native bitmap ownership.
Created: 2026-10-22
Architecture: Same initial half-error and endpoint exclusion as ReactOS lineto.c.
License: GPL-2.0-or-later.
*/
#include "line_coverage.hpp"
#include "hw/misc/lightning-gdi.h"

namespace lightning::reactos {
namespace {
int64_t minimum(int64_t a,int64_t b) noexcept {return a<b ? a : b;}
int64_t maximum(int64_t a,int64_t b) noexcept {return a>b ? a : b;}
uint64_t distance(int32_t a,int32_t b) noexcept {
    const int64_t value=static_cast<int64_t>(b)-a;
    return static_cast<uint64_t>(value<0 ? -value : value);
}
}

bool line_bounds(LinePoint first,LinePoint last,uint32_t width,uint32_t height,
    LineRectangle& output) noexcept {
    output={};
    if (width==0 || height==0 || width>LIGHTNING_GDI_MAX_DIMENSION || height>LIGHTNING_GDI_MAX_DIMENSION) return false;
    if (first.x==last.x && first.y==last.y) return true;
    const int64_t left=maximum(0,minimum(first.x,last.x));
    const int64_t top=maximum(0,minimum(first.y,last.y));
    const int64_t right=minimum(width,maximum(first.x,last.x)+1);
    const int64_t bottom=minimum(height,maximum(first.y,last.y)+1);
    if (left>=right || top>=bottom) return true;
    output={static_cast<int32_t>(left),static_cast<int32_t>(top),
        static_cast<int32_t>(right),static_cast<int32_t>(bottom)};
    return true;
}

bool line_coverage(LineMask mask,LinePoint first,LinePoint last) noexcept {
    const int64_t width=static_cast<int64_t>(mask.bounds.right)-mask.bounds.left;
    const int64_t height=static_cast<int64_t>(mask.bounds.bottom)-mask.bounds.top;
    if (mask.pixels==nullptr || mask.bounds.left<0 || mask.bounds.top<0 ||
        width<=0 || height<=0 || width>LIGHTNING_GDI_MAX_DIMENSION || height>LIGHTNING_GDI_MAX_DIMENSION ||
        mask.stride%sizeof(uint32_t)!=0 || static_cast<uint64_t>(width)>uint64_t{mask.stride}*8 ||
        uint64_t{mask.stride}*height>mask.bytes ||
        mask.bytes>UINTPTR_MAX-reinterpret_cast<uintptr_t>(mask.pixels)) return false;
    const uint64_t dx=distance(first.x,last.x),dy=distance(first.y,last.y);
    const bool major_x=dx>=dy;
    const uint64_t major=major_x ? dx : dy;
    const uint64_t minor=major_x ? dy : dx;
    const int64_t start=major_x ? first.x : first.y;
    const bool increasing=major_x ? last.x>=first.x : last.y>=first.y;
    const int64_t low=major_x ? mask.bounds.left : mask.bounds.top;
    const int64_t high=major_x ? mask.bounds.right : mask.bounds.bottom;
    const uint64_t begin=static_cast<uint64_t>(minimum(static_cast<int64_t>(major),
        maximum(0,increasing ? low-start : start-high+1)));
    const uint64_t end=static_cast<uint64_t>(minimum(static_cast<int64_t>(major),
        maximum(0,increasing ? high-start : start-low+1)));
    const size_t extent=static_cast<size_t>(mask.stride)*static_cast<size_t>(height);
    for (size_t index=0;index<extent;++index) mask.pixels[index]=0;
    if (major==0) return true;
    // i<=major-1 and both deltas<=UINT32_MAX bound the multiply plus half-error
    // below UINT64_MAX. This is the engine recurrence evaluated at visible i.
    for (uint64_t i=begin;i<end;++i) {
        const uint64_t secondary=(i*minor+major/2)/major;
        const int64_t x=static_cast<int64_t>(first.x)+(last.x>=first.x ? 1 : -1)*
            static_cast<int64_t>(major_x ? i : secondary);
        const int64_t y=static_cast<int64_t>(first.y)+(last.y>=first.y ? 1 : -1)*
            static_cast<int64_t>(major_x ? secondary : i);
        if (x<mask.bounds.left || x>=mask.bounds.right || y<mask.bounds.top || y>=mask.bounds.bottom) continue;
        const size_t column=static_cast<size_t>(x-mask.bounds.left);
        const size_t offset=static_cast<size_t>(y-mask.bounds.top)*mask.stride+column/8;
        mask.pixels[offset]|=static_cast<uint8_t>(0x80U>>(column%8));
    }
    return true;
}

bool line_rop(uint32_t mix,uint32_t& rop4) noexcept {
    rop4=0;
    const uint32_t operation=mix&UINT8_MAX;
    if ((mix&0xffff0000U)!=0 || operation==0 || operation>16) return false;
    const uint32_t truth=operation-1;
    uint32_t table=0;
    for (uint32_t index=0;index<8;++index) {
        const uint32_t pattern_destination=((index>>1)&2U)|(index&1U);
        table|=((truth>>pattern_destination)&1U)<<index;
    }
    constexpr uint32_t preserve_destination=0xaaU;
    rop4=table|(preserve_destination<<8);
    return true;
}
}

/*
Module: lightning.tests.brush_origin
Purpose: Exercise production DC-to-surface origin translation and LONG bounds.
Created: 2026-10-21
Architecture: Actual GDI SDK/POD helper, no DC or GPU emulation.
*/
#include <windef.h>
#include <wingdi.h>
#include <stdint.h>
#include <stdio.h>
#include "framebuffer_brush_origin.hpp"

namespace {
unsigned checks=0;
unsigned failures=0;
void check(bool condition,const char* name) {
    ++checks;
    if (!condition) {++failures; printf("FAIL %s\n",name);}
}
}
int main() {
    const POINTL logical{-5,3};
    const POINTL dc_origin{320,240};
    POINTL translated{99,99};
    check(Rx7900BrushOrigin(&logical,&dc_origin,&translated) && translated.x==315 && translated.y==243,
        "signed DC brush phase becomes absolute surface coordinates exactly once");
    const POINTL reference_origin{};
    POINTL reference{};
    check(Rx7900BrushOrigin(&logical,&reference_origin,&reference) && reference.x==-5 && reference.y==3,
        "memory bitmap retains signed local origin");
    const POINTL local_position{16,296};
    const POINTL actual_brush{local_position.x-5,local_position.y+3};
    check(Rx7900BrushOrigin(&actual_brush,&dc_origin,&translated) &&
        (dc_origin.x+local_position.x-translated.x)==-reference.x &&
        (dc_origin.y+local_position.y-translated.y)==-reference.y,
        "window and memory reference brush phases agree despite distinct DC and destination origins");
    const POINTL restored{7,-9};
    check(Rx7900BrushOrigin(&restored,&dc_origin,&translated) && translated.x==327 && translated.y==231,
        "restored authoritative attributes replace earlier cached origin");
    const LONG values[]{INT32_MIN,INT32_MIN+1,-1,0,1,INT32_MAX-1,INT32_MAX};
    for (LONG x:values) {
        for (LONG offset:values) {
            const POINTL input{x,0},origin{offset,0};
            const int64_t expected=static_cast<int64_t>(x)+offset;
            const bool valid=expected>=INT32_MIN && expected<=INT32_MAX;
            check((Rx7900BrushOrigin(&input,&origin,&translated)!=FALSE)==valid &&
                (valid ? translated.x==expected && translated.y==0 : translated.x==0 && translated.y==0),
                "horizontal extreme sums are checked before narrowing");
            const POINTL vertical{0,x},vertical_origin{0,offset};
            check((Rx7900BrushOrigin(&vertical,&vertical_origin,&translated)!=FALSE)==valid &&
                (valid ? translated.y==expected && translated.x==0 : translated.x==0 && translated.y==0),
                "vertical extreme sums are checked before narrowing");
        }
    }
    POINTL aliased{-5,3};
    check(Rx7900BrushOrigin(&aliased,&dc_origin,&aliased) && aliased.x==315 && aliased.y==243,
        "output aliasing preserves borrowed input until both coordinates are captured");
    check(!Rx7900BrushOrigin(nullptr,&dc_origin,&translated) && translated.x==0 && translated.y==0,
        "null input fails with initialized output");
    check(!Rx7900BrushOrigin(&logical,&dc_origin,nullptr),"null output never dereferenced");
    const LONG extents[]{1,2,3,7,INT32_MAX};
    for (LONG coordinate:values) {
        for (LONG origin:values) {
            for (LONG extent:extents) {
                LONG phase=-1;
                int64_t expected=(static_cast<int64_t>(coordinate)-origin)%extent;
                if (expected<0) expected+=extent;
                check(Rx7900PatternPhase(coordinate,origin,extent,&phase) && phase==expected &&
                    phase>=0 && phase<extent,"all admitted phase extremes stay within the owned pattern");
            }
        }
    }
    LONG phase=-1;
    check(Rx7900PatternPhase(0,3,2,&phase) && phase==1,
        "negative first-row remainder selects valid final pattern row");
    check(Rx7900PatternPhase(static_cast<int64_t>(INT32_MIN)-1,INT32_MAX,3,&phase) && phase==2,
        "bottom-exclusive minus one remains bounded before LONG subtraction");
    check(!Rx7900PatternPhase(0,0,0,&phase) && phase==0,"zero pattern extent rejects before division");
    check(!Rx7900PatternPhase(0,0,-1,&phase) && phase==0,"negative pattern extent rejects before division");
    check(!Rx7900PatternPhase(INT64_MAX,0,3,&phase) && phase==0,
        "out-of-contract coordinate cannot overflow phase subtraction");
    printf("Brush origin boundary: %u checks, %u failures\n",checks,failures);
    return failures==0 ? 0 : 1;
}

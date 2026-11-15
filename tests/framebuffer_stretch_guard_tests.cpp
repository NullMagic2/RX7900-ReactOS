/*
Module: lightning.tests.stretch_guard
Purpose: Check production paired-call admission and original fallback ownership.
Created: 2026-10-21
Architecture: Actual SDK/helper; no engine renderer, native hardware or UI model.
*/
#include <stdarg.h>
#include <windef.h>
#include <wingdi.h>
#include <winddi.h>
#include "framebuffer_stretch_guard.hpp"
#include "lightning_present_abi.hpp"
#include "lightning-display.h"
#include "radeon_line_identity.hpp"
#include <stdio.h>
#include <string.h>

namespace {
unsigned checks=0;
unsigned failures=0;
unsigned queries=0;
unsigned fault=0;
void check(bool condition,const char* message) {
    ++checks;
    if (!condition) {++failures; printf("FAIL %s\n",message);}
}
ULONG APIENTRY query(SURFOBJ*,ULONG code,ULONG input_bytes,PVOID,ULONG output_bytes,PVOID output) {
    ++queries;
    if (code!=lightning::reactos::query_display_escape || input_bytes!=0 ||
        output_bytes!=sizeof(lightning::reactos::CpuScanoutInformation) || fault==1) return 0;
    auto& result=*static_cast<lightning::reactos::CpuScanoutInformation*>(output);
    result={};
    result.size=sizeof(result);
    result.version=lightning::reactos::cpu_scanout_abi_version;
    result.magic=LIGHTNING_DISPLAY_MAGIC;
    result.width=1920;
    result.height=1080;
    result.stride=1920*4;
    result.mapped_bytes=1920*1080*4;
    result.mode_generation=7;
    result.enabled=1;
    if (fault==2) ++result.version;
    if (fault==3) ++result.magic;
    if (fault==4) result.mode_generation=0;
    if (fault==5) result.enabled=0;
    if (fault==6) result.width=UINT32_MAX;
    if (fault==7) --result.stride;
    if (fault==8) --result.mapped_bytes;
    return sizeof(result);
}

// radeondisp: refuses the Lightning query and answers only its identity.
unsigned radeon_fault=0;
bool radeon_stretch=false;
ULONG APIENTRY radeon_query(SURFOBJ*,ULONG code,ULONG input_bytes,PVOID,ULONG output_bytes,PVOID output) {
    namespace line=rx7900::reactos::radeon_line;
    ++queries;
    if (code!=line::identity_escape || input_bytes!=0 || output_bytes!=sizeof(line::Identity) ||
        radeon_fault==1) return 0;
    auto& result=*static_cast<line::Identity*>(output);
    result=line::make_identity(radeon_fault!=2,radeon_stretch);
    if (radeon_fault==3) ++result.magic;
    if (radeon_fault==4) ++result.version;
    if (radeon_fault==5) result.flags|=4U;
    return radeon_fault==6 ? sizeof(result)-1 : sizeof(result);
}
}

int main() {
    SURFOBJ surface{};
    surface.iType=STYPE_DEVICE;
    COLORADJUSTMENT actual{};
    actual.caSize=sizeof(actual);
    actual.caRedGamma=1234;
    RX7900_STRETCH_GUARD context{};
    COLORADJUSTMENT* original=&actual;
    ULONG orientation=UINT32_MAX;
    check(!Rx7900StretchGuardUnwrap(nullptr,&original,&orientation) && original==nullptr && orientation==0,
        "null ordinary adjustment cannot enter paired hook");
    check(!Rx7900StretchGuardUnwrap(&actual,&original,&orientation),
        "ordinary COLORADJUSTMENT remains rejected without reading a larger object");
    check(Rx7900StretchGuardPrepare(nullptr,&surface,query,&actual,FALSE,FALSE)==&actual && queries==0,
        "absent stack context preserves original callback without a probe");
    for (fault=1;fault<=8;++fault) {
        check(Rx7900StretchGuardPrepare(&context,&surface,query,&actual,FALSE,FALSE)==&actual,
            "foreign, stale, inactive or malformed display identity cannot wrap a callback");
    }
    fault=0;
    const RECTL gradient_clip{3,5,9,11};
    ULONG vertex_bytes=0,mesh_bytes=0;
    check(Rx7900GradientCaptureBytes(GRADIENT_FILL_RECT_H,2,1,&vertex_bytes,&mesh_bytes) &&
        vertex_bytes==2*sizeof(TRIVERTEX) && mesh_bytes==sizeof(GRADIENT_RECT),"rectangle capture uses checked byte counts");
    check(Rx7900GradientCaptureBytes(GRADIENT_FILL_TRIANGLE,3,1,&vertex_bytes,&mesh_bytes) &&
        vertex_bytes==3*sizeof(TRIVERTEX) && mesh_bytes==sizeof(GRADIENT_TRIANGLE),"triangle capture uses checked byte counts");
    check(!Rx7900GradientCaptureBytes(GRADIENT_FILL_TRIANGLE,UINT32_MAX,1,&vertex_bytes,&mesh_bytes) &&
        vertex_bytes==0 && mesh_bytes==0,"vertex multiplication overflow clears captured lengths");
    check(!Rx7900GradientCaptureBytes(GRADIENT_FILL_RECT_V,2,UINT32_MAX,&vertex_bytes,&mesh_bytes),
        "mesh multiplication overflow is rejected before capture");
    check(!Rx7900GradientCaptureBytes(GRADIENT_FILL_TRIANGLE,UINT32_MAX/sizeof(TRIVERTEX),2,&vertex_bytes,&mesh_bytes),
        "combined capture overflow is rejected before allocation");
    check(!Rx7900GradientCaptureBytes(3,3,1,&vertex_bytes,&mesh_bytes) &&
        !Rx7900GradientCaptureBytes(0,0,1,&vertex_bytes,&mesh_bytes) &&
        !Rx7900GradientCaptureBytes(0,2,0,&vertex_bytes,&mesh_bytes) &&
        !Rx7900GradientCaptureBytes(0,2,1,nullptr,&mesh_bytes),"invalid capture descriptors cannot authorize an array read");
    check(Rx7900GradientSpanVisible(3,9,5,&gradient_clip) &&
        Rx7900GradientSpanVisible(4,8,10,&gradient_clip),"clipped solid gradient rows remain admissible");
    check(!Rx7900GradientSpanVisible(3,9,4,&gradient_clip) &&
        !Rx7900GradientSpanVisible(3,9,11,&gradient_clip),"solid triangle cannot write above or below its clip");
    check(!Rx7900GradientSpanVisible(9,9,5,&gradient_clip) &&
        !Rx7900GradientSpanVisible(10,9,5,&gradient_clip),"empty and inverted gradient spans cannot become unsigned byte counts");
    check(!Rx7900GradientSpanVisible(INT32_MIN,INT32_MAX,5,&gradient_clip) &&
        !Rx7900GradientSpanVisible(3,9,5,nullptr),"uncontained spans and absent clip cannot authorize DIB writes");
    for (ULONG flags=0;flags<4;++flags) {
        auto* tagged=Rx7900StretchGuardPrepare(&context,&surface,query,&actual,
            (flags&1)!=0,(flags&2)!=0);
        check(tagged!=&actual && Rx7900StretchGuardUnwrap(tagged,&original,&orientation) &&
            original==&actual && orientation==flags && actual.caRedGamma==1234,
            "paired context preserves both original adjustment and exact orientation without mutation");
    }
    auto* tagged=Rx7900StretchGuardPrepare(&context,&surface,query,nullptr,FALSE,FALSE);
    check(Rx7900StretchGuardUnwrap(tagged,&original,&orientation) && original==nullptr && orientation==0,
        "paired positive stretch permits originally absent color correction");
    context.tag.caColorfulness=4;
    check(!Rx7900StretchGuardUnwrap(tagged,&original,&orientation),"unknown orientation bits cannot admit GPU work");
    context.tag.caColorfulness=0;
    ++context.tag.caFlags;
    check(!Rx7900StretchGuardUnwrap(tagged,&original,&orientation),"mismatched private payload cannot unwrap");
    constexpr MIX copy_mix=0x0d0dU;
    check(Rx7900LinePrepare(&surface,query,copy_mix,0)==(copy_mix|RX7900_LINE_MIX_TAG),
        "identified unstyled cosmetic call carries the paired admission marker");
    unsigned line_queries=queries;
    check(Rx7900LinePrepare(&surface,query,copy_mix,2)==copy_mix && queries==line_queries,
        "styled lines keep original engine ownership without a probe");
    check(Rx7900LinePrepare(&surface,query,0x12340d0dU,0)==0x12340d0dU && queries==line_queries,
        "unknown upper MIX bits cannot become an admitted line");
    fault=3;
    check(Rx7900LinePrepare(&surface,query,copy_mix,0)==copy_mix,
        "foreign display line keeps the original MIX");
    fault=0;
    check(Rx7900LinePrepare(&surface,radeon_query,copy_mix,0)==(copy_mix|RX7900_LINE_MIX_TAG),
        "radeondisp with GPU lines receives the paired admission marker");
    line_queries=queries;
    check(Rx7900LinePrepare(&surface,radeon_query,copy_mix,1)==copy_mix && queries==line_queries,
        "radeondisp styled line keeps original engine ownership without a probe");
    for (unsigned mode=1; mode<=6; ++mode) {
        radeon_fault=mode;
        check(Rx7900LinePrepare(&surface,radeon_query,copy_mix,0)==copy_mix,
            "refused, lineless or malformed radeondisp identity keeps the original MIX");
    }
    radeon_fault=0;
    COLORADJUSTMENT* unwrapped=nullptr;
    ULONG radeon_orientation=UINT32_MAX;
    check(Rx7900StretchGuardPrepare(&context,&surface,radeon_query,&actual,TRUE,FALSE)==&actual,
        "radeondisp without the stretch capability keeps the original adjustment");
    radeon_stretch=true;
    auto* radeon_tagged=Rx7900StretchGuardPrepare(&context,&surface,radeon_query,&actual,TRUE,FALSE);
    check(radeon_tagged!=&actual && Rx7900StretchGuardUnwrap(radeon_tagged,&unwrapped,&radeon_orientation) &&
        unwrapped==&actual && radeon_orientation==RX7900_STRETCH_FLIP_X,
        "radeondisp with the stretch capability receives the orientation tag");
    check(Rx7900LinePrepare(&surface,radeon_query,copy_mix,0)==(copy_mix|RX7900_LINE_MIX_TAG),
        "stretch and line capabilities combine");
    radeon_fault=5;
    check(Rx7900StretchGuardPrepare(&context,&surface,radeon_query,&actual,FALSE,TRUE)==&actual,
        "unknown radeondisp capability bits refuse the stretch tag");
    radeon_fault=0;
    radeon_stretch=false;
    surface.iType=STYPE_BITMAP;
    line_queries=queries;
    check(Rx7900LinePrepare(&surface,radeon_query,copy_mix,0)==copy_mix && queries==line_queries,
        "radeondisp plain engine bitmap is never probed");
    surface.iType=STYPE_DEVICE;
    const unsigned before=queries;
    surface.iType=STYPE_BITMAP;
    check(Rx7900LinePrepare(&surface,query,copy_mix,0)==copy_mix && queries==before,
        "plain engine line never probes a driver or receives a private marker");
    check(Rx7900StretchGuardPrepare(&context,&surface,query,&actual,FALSE,FALSE)==&actual && queries==before,
        "plain engine bitmap never receives a driver capability probe");
    printf("Stretch call guard: %u checks, %u failures\n",checks,failures);
    return failures==0 ? 0 : 1;
}

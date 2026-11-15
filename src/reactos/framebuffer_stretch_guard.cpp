/*
Module: lightning.reactos.stretch_guard
Purpose: Admit only paired engine calls without losing stretch mirror flags.
Created: 2026-10-21
Architecture: Reuses read-only display identity; no retained pointers or locks.
License: GPL-2.0-or-later.
*/
#include <stdarg.h>
#include <windef.h>
#include <wingdi.h>
#include <winddi.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "framebuffer_stretch_guard.hpp"
#include "lightning_present_abi.hpp"
#include "lightning-display.h"
#include "lightning-memory.h"
#include "radeon_line_identity.hpp"

namespace {
constexpr WORD tag_flags=RX7900_STRETCH_GUARD_MAGIC&UINT16_MAX;
constexpr WORD tag_illuminant=0xffffU;
constexpr WORD tag_red=RX7900_STRETCH_GUARD_MAGIC>>16;
constexpr WORD tag_green=0x5354U;
constexpr size_t pointer_offset=12;
constexpr size_t orientation_offset=20;
static_assert(sizeof(COLORADJUSTMENT)==24,"Fixed native GDI adjustment boundary");
static_assert(sizeof(RX7900_STRETCH_GUARD)==sizeof(COLORADJUSTMENT),"Never inspect an extended adjustment");
static_assert(pointer_offset+sizeof(uintptr_t)<=orientation_offset,"Bounded native pointer bytes");
static_assert(orientation_offset+sizeof(ULONG)==sizeof(COLORADJUSTMENT),"Bounded orientation bytes");

bool identity(const lightning::reactos::CpuScanoutInformation& value) noexcept {
    return value.size==sizeof(value) && value.version==lightning::reactos::cpu_scanout_abi_version &&
        value.magic==LIGHTNING_DISPLAY_MAGIC && value.mode_generation!=0 && value.enabled==1 &&
        value.width!=0 && value.height!=0 && value.width<=LIGHTNING_SCANOUT_MAX_DIMENSION &&
        value.height<=LIGHTNING_SCANOUT_MAX_DIMENSION &&
        uint64_t{value.width}*sizeof(uint32_t)<=value.stride &&
        uint64_t{value.stride}*value.height<=value.mapped_bytes;
}

// The physical radeondisp display answers its own fixed identity escape.
bool radeon_identity(SURFOBJ* surface,PFN_DrvEscape escape,uint32_t capability) noexcept {
    namespace line=rx7900::reactos::radeon_line;
    line::Identity information{};
    return escape(surface,line::identity_escape,0,nullptr,sizeof(information),&information)==
        sizeof(information) && line::admits(information,capability);
}
}

extern "C" COLORADJUSTMENT* APIENTRY Rx7900StretchGuardPrepare(RX7900_STRETCH_GUARD* context,
    SURFOBJ* surface,PFN_DrvEscape escape,COLORADJUSTMENT* original,
    BOOL horizontal_flip,BOOL vertical_flip) noexcept {
    if (context==nullptr) return original;
    *context={};
    if (surface==nullptr || surface->iType==STYPE_BITMAP || escape==nullptr) return original;
    lightning::reactos::CpuScanoutInformation information{};
    if ((escape(surface,lightning::reactos::query_display_escape,0,nullptr,sizeof(information),&information)!=
         sizeof(information) || !identity(information)) &&
        !radeon_identity(surface,escape,rx7900::reactos::radeon_line::flag_gpu_stretch)) return original;
    context->tag.caSize=sizeof(*context);
    context->tag.caFlags=tag_flags;
    context->tag.caIlluminantIndex=tag_illuminant;
    context->tag.caRedGamma=tag_red;
    context->tag.caGreenGamma=tag_green;
    context->tag.caBlueGamma=RX7900_STRETCH_GUARD_VERSION;
    const ULONG orientation=(horizontal_flip ? RX7900_STRETCH_FLIP_X : 0U) |
        (vertical_flip ? RX7900_STRETCH_FLIP_Y : 0U);
    const uintptr_t original_address=reinterpret_cast<uintptr_t>(original);
    // Native kernel provenance only: IntEng callers pass their locked DC's
    // dclevel.ca, not a user address. Copy pointer representation as bytes;
    // no object overlay or larger-than-COLORADJUSTMENT read is required.
    auto* bytes=reinterpret_cast<uint8_t*>(&context->tag);
    memcpy(bytes+pointer_offset,&original_address,sizeof(original_address));
    memcpy(bytes+orientation_offset,&orientation,sizeof(orientation));
    return &context->tag;
}

extern "C" MIX APIENTRY Rx7900LinePrepare(SURFOBJ* surface,PFN_DrvEscape escape,MIX mix,
    ULONG style_count) noexcept {
    if (style_count!=0 || (mix&0xffff0000U)!=0 || surface==nullptr ||
        surface->iType==STYPE_BITMAP || escape==nullptr) return mix;
    lightning::reactos::CpuScanoutInformation information{};
    if ((escape(surface,lightning::reactos::query_display_escape,0,nullptr,sizeof(information),&information)!=
         sizeof(information) || !identity(information)) &&
        !radeon_identity(surface,escape,rx7900::reactos::radeon_line::flag_gpu_lines)) return mix;
    return mix|RX7900_LINE_MIX_TAG;
}

extern "C" BOOL APIENTRY Rx7900GradientSpanVisible(LONG left,LONG right,LONG row,
    const RECTL* clip) noexcept {
    // The engine supplies an already intersected span. Never let an empty
    // horizontal range become an unsigned byte count in DIB_HLine.
    return clip!=nullptr && left<right && left>=clip->left && right<=clip->right &&
        row>=clip->top && row<clip->bottom;
}

extern "C" BOOL APIENTRY Rx7900GradientCaptureBytes(ULONG mode,ULONG vertices,ULONG meshes,
    ULONG* vertex_bytes,ULONG* mesh_bytes) noexcept {
    if (vertex_bytes==nullptr || mesh_bytes==nullptr) return FALSE;
    *vertex_bytes=0;
    *mesh_bytes=0;
    const ULONG element_bytes=mode==GRADIENT_FILL_TRIANGLE ? sizeof(GRADIENT_TRIANGLE) :
        ((mode==GRADIENT_FILL_RECT_H || mode==GRADIENT_FILL_RECT_V) ? sizeof(GRADIENT_RECT) : 0U);
    if (vertices==0 || meshes==0 || element_bytes==0 ||
        vertices>UINT32_MAX/sizeof(TRIVERTEX) || meshes>UINT32_MAX/element_bytes) return FALSE;
    const ULONG captured_vertices=vertices*sizeof(TRIVERTEX);
    const ULONG captured_meshes=meshes*element_bytes;
    if (captured_vertices>UINT32_MAX-captured_meshes) return FALSE;
    *vertex_bytes=captured_vertices;
    *mesh_bytes=captured_meshes;
    return TRUE;
}

extern "C" BOOL APIENTRY Rx7900StretchGuardUnwrap(COLORADJUSTMENT* tagged,
    COLORADJUSTMENT** original,ULONG* orientation) noexcept {
    if (original==nullptr || orientation==nullptr) return FALSE;
    *original=nullptr;
    *orientation=0;
    // Every access stays within ordinary COLORADJUSTMENT bytes, including
    // forged caSize values. The impossible standard header identifies only
    // this paired kernel contract; it does not authenticate arbitrary input.
    if (tagged==nullptr || tagged->caSize!=sizeof(RX7900_STRETCH_GUARD) || tagged->caFlags!=tag_flags ||
        tagged->caIlluminantIndex!=tag_illuminant || tagged->caRedGamma!=tag_red ||
        tagged->caGreenGamma!=tag_green || tagged->caBlueGamma!=RX7900_STRETCH_GUARD_VERSION) return FALSE;
    uintptr_t original_address=0;
    ULONG flags=0;
    const auto* bytes=reinterpret_cast<const uint8_t*>(tagged);
    memcpy(&original_address,bytes+pointer_offset,sizeof(original_address));
    memcpy(&flags,bytes+orientation_offset,sizeof(flags));
    if ((flags&~(RX7900_STRETCH_FLIP_X|RX7900_STRETCH_FLIP_Y))!=0 ||
        (original_address%alignof(COLORADJUSTMENT))!=0) return FALSE;
    for (size_t offset=pointer_offset+sizeof(original_address);offset<orientation_offset;++offset) {
        if (bytes[offset]!=0) return FALSE;
    }
    *original=reinterpret_cast<COLORADJUSTMENT*>(original_address);
    *orientation=flags;
    return TRUE;
}

/*
Module: lightning.reactos.text_batch
Purpose: Batch existing FreeType glyph rasterization without a second rasterizer.
Created: 2026-10-21
Architecture: Linked into Win32k, not exported. Original GDI coordinates and
clipping are unchanged. The matching display driver owns the cached bitmap.
License: GPL-2.0-or-later.
*/
#include <stdarg.h>
#include <windef.h>
#include <wingdi.h>
#include <winddi.h>
#include "framebuffer_text_batch.hpp"
#include "framebuffer_shadow_policy.hpp"

namespace {
namespace policy=rx7900::reactos::framebuffer;
policy::Rectangle rectangle(const RECTL& value) noexcept {
    return {value.left,value.top,value.right,value.bottom};
}
}

extern "C" SURFOBJ* APIENTRY Rx7900TextBatchBegin(RX7900_TEXT_BATCH_CONTEXT* context,
    SURFOBJ* surface,CLIPOBJ* clip,PFN_DrvEscape escape,PFN_DrvNotify notify) noexcept {
    if (context==nullptr) return surface;
    *context={};
    context->size=sizeof(*context);
    context->version=RX7900_TEXT_BATCH_VERSION;
    context->magic=RX7900_TEXT_BATCH_MAGIC;
    context->original=surface;
    context->result=TRUE;
    if (surface==nullptr || surface->iType==STYPE_BITMAP || escape==nullptr || notify==nullptr) return surface;
    RX7900_TEXT_BATCH_CAPABILITY capability{};
    // Other drivers see only a standard read-only Escape query. No unknown
    // notification is sent until the complete private identity/layout matches.
    if (escape(surface,RX7900_TEXT_BATCH_QUERY,0,nullptr,sizeof(capability),&capability)!=sizeof(capability) ||
        capability.size!=sizeof(capability) || capability.version!=RX7900_TEXT_BATCH_VERSION ||
        capability.magic!=RX7900_TEXT_BATCH_MAGIC || capability.context_bytes!=sizeof(*context)) return surface;
    context->notify=notify;
    if (clip!=nullptr && clip->iDComplexity!=DC_TRIVIAL) {
        context->clipped=TRUE;
        context->clip_bounds=clip->rclBounds;
    }
    notify(surface,RX7900_TEXT_BATCH_BEGIN,context);
    if (context->active==0) return surface;
    if (context->cached==nullptr || context->cached->iType!=STYPE_BITMAP ||
        context->cached->sizlBitmap.cx!=surface->sizlBitmap.cx ||
        context->cached->sizlBitmap.cy!=surface->sizlBitmap.cy) {
        context->result=FALSE;
        Rx7900TextBatchEnd(context);
        return surface;
    }
    return context->cached;
}

extern "C" VOID APIENTRY Rx7900TextBatchRecord(RX7900_TEXT_BATCH_CONTEXT* context,
    const RECTL* attempted) noexcept {
    if (context==nullptr || context->active==0 || context->original==nullptr || attempted==nullptr) return;
    auto dirty=policy::clip(rectangle(*attempted),context->original->sizlBitmap.cx,context->original->sizlBitmap.cy);
    if (context->clipped) dirty=policy::intersect(dirty,rectangle(context->clip_bounds));
    dirty=policy::unite(rectangle(context->dirty),dirty);
    context->dirty={dirty.left,dirty.top,dirty.right,dirty.bottom};
}

extern "C" BOOL APIENTRY Rx7900TextBatchEnd(RX7900_TEXT_BATCH_CONTEXT* context) noexcept {
    if (context==nullptr) return FALSE;
    if (context->active!=0 && context->notify!=nullptr) context->notify(context->original,RX7900_TEXT_BATCH_END,context);
    return context->active==0 && context->result!=FALSE;
}

extern "C" ULONG APIENTRY Rx7900TextBatchGlyph(RX7900_TEXT_BATCH_CONTEXT* context,SURFOBJ* mask,
    CLIPOBJ* clip,const RECTL* destination,const POINTL* mask_origin,BRUSHOBJ* brush) noexcept {
    if (context!=nullptr && context->result==FALSE) return RX7900_TEXT_GLYPH_FAILED;
    if (context==nullptr || context->active==0 || context->notify==nullptr || context->original==nullptr ||
        mask==nullptr || destination==nullptr || mask_origin==nullptr) return RX7900_TEXT_GLYPH_UNSUPPORTED;
    RX7900_TEXT_GLYPH_REQUEST request{};
    request.size=sizeof(request);
    request.version=RX7900_TEXT_BATCH_VERSION;
    request.context=context;
    request.mask=mask;
    request.clip=clip;
    request.destination=*destination;
    request.mask_origin=*mask_origin;
    request.brush=brush;
    context->notify(context->original,RX7900_TEXT_BATCH_GLYPH,&request);
    if (request.result>RX7900_TEXT_GLYPH_FAILED) request.result=RX7900_TEXT_GLYPH_FAILED;
    if (request.result==RX7900_TEXT_GLYPH_FAILED) context->result=FALSE;
    return request.result;
}

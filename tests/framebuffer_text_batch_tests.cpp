/*
Module: lightning.tests.text_batch
Purpose: Check actual SDK helper capability gating and borrowed-surface scopes.
Created: 2026-10-21
Architecture: No desktop/GPU access; callbacks model only the private contract.
*/
#include <stdarg.h>
#include <windef.h>
#include <wingdi.h>
#include <winddi.h>
#include "framebuffer_text_batch.hpp"
#include <stdio.h>

namespace {
unsigned checks=0;
unsigned failures=0;
unsigned probes=0;
unsigned begins=0;
unsigned ends=0;
unsigned kind=0;
ULONG glyph_result=RX7900_TEXT_GLYPH_PERFORMED;
SURFOBJ cached{};

void check(bool value,const char* name) {
    ++checks;
    if (!value) {++failures; printf("FAIL %s\n",name);}
}

ULONG APIENTRY query(SURFOBJ*,ULONG code,ULONG bytes,PVOID,ULONG output_bytes,PVOID output) {
    ++probes;
    if (kind==1 || code!=RX7900_TEXT_BATCH_QUERY || bytes!=0 || output_bytes!=sizeof(RX7900_TEXT_BATCH_CAPABILITY)) return 0;
    auto& capability=*static_cast<RX7900_TEXT_BATCH_CAPABILITY*>(output);
    capability={sizeof(capability),RX7900_TEXT_BATCH_VERSION,RX7900_TEXT_BATCH_MAGIC,
        sizeof(RX7900_TEXT_BATCH_CONTEXT)};
    if (kind==2) ++capability.version;
    if (kind==3) ++capability.context_bytes;
    if (kind==4) ++capability.magic;
    return sizeof(capability);
}

VOID APIENTRY notify(SURFOBJ*,ULONG code,PVOID data) {
    if (code==RX7900_TEXT_BATCH_GLYPH) {
        auto& request=*static_cast<RX7900_TEXT_GLYPH_REQUEST*>(data);
        check(request.size==sizeof(request) && request.version==RX7900_TEXT_BATCH_VERSION &&
            request.context!=nullptr && request.context->active,"glyph request is a bounded live private scope");
        request.result=glyph_result;
        return;
    }
    auto& context=*static_cast<RX7900_TEXT_BATCH_CONTEXT*>(data);
    if (code==RX7900_TEXT_BATCH_BEGIN) {
        ++begins;
        if (kind==5) return;
        context.active=TRUE;
        context.cached=kind==6 ? nullptr : &cached;
    } else if (code==RX7900_TEXT_BATCH_END) {
        ++ends;
        context.active=FALSE;
        context.cached=nullptr;
        if (kind==7) context.result=FALSE;
    } else check(false,"only positively identified private begin/end notifications are emitted");
}
}

int main() {
    SURFOBJ device{};
    device.iType=STYPE_DEVICE;
    device.sizlBitmap={1920,1080};
    cached.iType=STYPE_BITMAP;
    cached.sizlBitmap=device.sizlBitmap;
    RX7900_TEXT_BATCH_CONTEXT context{};
    check(Rx7900TextBatchBegin(nullptr,&device,nullptr,query,notify)==&device,"null context never borrows");
    for (kind=1;kind<=4;++kind) {
        check(Rx7900TextBatchBegin(&context,&device,nullptr,query,notify)==&device,"unsupported/mismatched identity preserves original surface");
        check(begins==0 && ends==0,"foreign/mismatched driver never receives unknown Notify");
        check(Rx7900TextBatchEnd(&context),"declined batching preserves ordinary engine path");
    }
    kind=0;
    const auto before=probes;
    check(Rx7900TextBatchBegin(&context,&cached,nullptr,query,notify)==&cached && probes==before,
          "plain bitmap text does not query any driver");
    CLIPOBJ clip{};
    clip.iDComplexity=DC_RECT;
    clip.rclBounds={20,30,80,90};
    check(Rx7900TextBatchBegin(&context,&device,&clip,query,notify)==&cached && begins==1 && context.active==TRUE,
          "validated identity borrows matching cached bitmap");
    const RECTL first{10,20,50,60};
    const RECTL second{40,50,100,110};
    Rx7900TextBatchRecord(&context,&first);
    Rx7900TextBatchRecord(&context,&second);
    check(context.dirty.left==20 && context.dirty.top==30 && context.dirty.right==80 && context.dirty.bottom==90,
          "glyph dirty union preserves original screen coordinates and clip bounds");
    check(Rx7900TextBatchEnd(&context) && ends==1 && context.active==0,"normal end releases borrowed scope exactly once");
    check(Rx7900TextBatchEnd(&context) && ends==1,"repeated cleanup cannot notify/release twice");
    kind=0;
    check(Rx7900TextBatchBegin(&context,&device,nullptr,query,notify)==&cached,"GPU glyph fixture borrows owner");
    SURFOBJ mask{};
    BRUSHOBJ brush{};
    POINTL origin{};
    check(Rx7900TextBatchGlyph(&context,&mask,nullptr,&first,&origin,&brush)==RX7900_TEXT_GLYPH_PERFORMED &&
        context.dirty.right==0,"accepted GPU glyph never marks stale CPU cache for writeback");
    glyph_result=RX7900_TEXT_GLYPH_UNSUPPORTED;
    check(Rx7900TextBatchGlyph(&context,&mask,nullptr,&first,&origin,&brush)==RX7900_TEXT_GLYPH_UNSUPPORTED &&
        context.result,"known unsupported result preserves exact CPU fallback");
    glyph_result=99;
    check(Rx7900TextBatchGlyph(&context,&mask,nullptr,&first,&origin,&brush)==RX7900_TEXT_GLYPH_FAILED &&
        !context.result,"unknown execution result fails closed instead of replaying glyph");
    check(!Rx7900TextBatchEnd(&context) && !context.active,"failed glyph still releases borrowed scope");
    kind=5;
    check(Rx7900TextBatchBegin(&context,&device,nullptr,query,notify)==&device && !context.active,
          "runtime CPU backend may decline without changing text behavior");
    kind=6;
    const auto before_end=ends;
    check(Rx7900TextBatchBegin(&context,&device,nullptr,query,notify)==&device && ends==before_end+1 && !context.active,
          "malformed borrowed bitmap is ended before fallback");
    check(!Rx7900TextBatchEnd(&context),"malformed borrow remains an explicit failed scope");
    kind=7;
    check(Rx7900TextBatchBegin(&context,&device,nullptr,query,notify)==&cached,"failure fixture begins valid scope");
    check(!Rx7900TextBatchEnd(&context) && !context.active,"failed final upload releases scope and propagates failure");
    printf("Text batch: %u checks, %u failures\n",checks,failures);
    return failures==0 ? 0 : 1;
}

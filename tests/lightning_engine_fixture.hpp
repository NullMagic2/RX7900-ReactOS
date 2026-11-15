/*
Module: lightning.tests.engine_fixture
Purpose: Reuse bounded engine bitmap boundaries across production client fixtures.
Created: 2026-10-21
Architecture: Real SDK types, no general renderer or native hardware model.
*/
#pragma once
#include "framebuf.h"
#include "lightning-memory.h"
#include <stdlib.h>
#include <string.h>

namespace lightning::tests::engine {
inline unsigned bitmap_locks=0;
inline unsigned allocations=0;
inline unsigned bitmap_creations=0;
struct Bitmap final {
    SURFOBJ surface{};
    uint8_t pixels[LIGHTNING_SCANOUT_CURSOR_SHAPE_BYTES]{};
    bool used=false;
};
Bitmap bitmaps[4]{};


} // namespace lightning::tests::engine

using namespace lightning::tests::engine;

extern "C" PVOID APIENTRY EngAllocMem(ULONG,ULONG bytes,ULONG) {
    void* allocation=calloc(1,bytes);
    if (allocation!=nullptr) ++allocations;
    return allocation;
}

extern "C" VOID APIENTRY EngFreeMem(PVOID allocation) {
    if (allocation!=nullptr) {--allocations; free(allocation);}
}

extern "C" HBITMAP APIENTRY EngCreateBitmap(SIZEL size,LONG stride,ULONG format,FLONG,PVOID) {
    if (size.cx<=0 || size.cy<=0 || stride<=0 ||
        static_cast<uint64_t>(stride)*size.cy>sizeof(bitmaps[0].pixels)) return nullptr;
    for (auto& bitmap:bitmaps) {
        if (bitmap.used) continue;
        bitmap={};
        bitmap.used=true;
        bitmap.surface.iType=STYPE_BITMAP;
        bitmap.surface.sizlBitmap=size;
        bitmap.surface.lDelta=stride;
        bitmap.surface.iBitmapFormat=format;
        bitmap.surface.pvScan0=bitmap.pixels;
        bitmap.surface.pvBits=bitmap.pixels;
        bitmap.surface.cjBits=static_cast<ULONG>(static_cast<uint64_t>(stride)*size.cy);
        ++bitmap_creations;
        return reinterpret_cast<HBITMAP>(&bitmap);
    }
    return nullptr;
}

extern "C" SURFOBJ* APIENTRY EngLockSurface(HSURF handle) {
    auto* bitmap=reinterpret_cast<Bitmap*>(handle);
    if (bitmap==nullptr || !bitmap->used) return nullptr;
    ++bitmap_locks;
    return &bitmap->surface;
}

extern "C" VOID APIENTRY EngUnlockSurface(SURFOBJ*) {--bitmap_locks;}

extern "C" BOOL APIENTRY EngDeleteSurface(HSURF handle) {
    auto* bitmap=reinterpret_cast<Bitmap*>(handle);
    if (bitmap==nullptr || !bitmap->used) return FALSE;
    bitmap->used=false;
    return TRUE;
}

extern "C" BOOL APIENTRY EngCopyBits(SURFOBJ* destination,SURFOBJ* source,CLIPOBJ*,XLATEOBJ* translation,
    RECTL* rectangle,POINTL* point) {
    // Only bounded owned bitmap copies and palette expansion are modeled.
    // GPU sampling/composition remains outside this native boundary fixture.
    if (source==nullptr || destination==nullptr || rectangle==nullptr || point==nullptr ||
        (source->iBitmapFormat!=destination->iBitmapFormat &&
         !(source->iBitmapFormat==BMF_8BPP && destination->iBitmapFormat==BMF_32BPP)) ||
        rectangle->left<0 || rectangle->top<0 || point->x<0 || point->y<0 ||
        rectangle->right<=rectangle->left || rectangle->bottom<=rectangle->top ||
        rectangle->right>destination->sizlBitmap.cx || rectangle->bottom>destination->sizlBitmap.cy ||
        static_cast<int64_t>(point->x)+rectangle->right-rectangle->left>source->sizlBitmap.cx ||
        static_cast<int64_t>(point->y)+rectangle->bottom-rectangle->top>source->sizlBitmap.cy) return FALSE;
    const LONG width=rectangle->right-rectangle->left;
    const LONG height=rectangle->bottom-rectangle->top;
    if (source->iBitmapFormat==BMF_1BPP && ((point->x%8)!=0 || (rectangle->left%8)!=0)) return FALSE;
    const size_t bytes=destination->iBitmapFormat==BMF_1BPP ?
        (static_cast<size_t>(width)+7U)/8U : static_cast<size_t>(width)*
        (destination->iBitmapFormat==BMF_8BPP ? 1U : 4U);
    const size_t source_x=source->iBitmapFormat==BMF_1BPP ? static_cast<size_t>(point->x)/8 :
        static_cast<size_t>(point->x)*(source->iBitmapFormat==BMF_8BPP ? 1U : 4U);
    const size_t target_x=destination->iBitmapFormat==BMF_1BPP ? static_cast<size_t>(rectangle->left)/8 :
        static_cast<size_t>(rectangle->left)*(destination->iBitmapFormat==BMF_8BPP ? 1U : 4U);
    for (LONG y=0;y<height;++y) {
        const auto* row=static_cast<const uint8_t*>(source->pvScan0)+
            static_cast<ptrdiff_t>(source->lDelta)*(point->y+y)+source_x;
        auto* target=static_cast<uint8_t*>(destination->pvScan0)+
            static_cast<size_t>(destination->lDelta)*(rectangle->top+y)+target_x;
        if (source->iBitmapFormat==BMF_8BPP && destination->iBitmapFormat==BMF_32BPP) {
            for (LONG x=0;x<width;++x) {
                const uint32_t index=row[x];
                if (translation!=nullptr && translation->pulXlate!=nullptr && index>=translation->cEntries) return FALSE;
                const uint32_t value=translation!=nullptr && translation->pulXlate!=nullptr ?
                    translation->pulXlate[index] : index;
                memcpy(target+static_cast<size_t>(x)*sizeof(value),&value,sizeof(value));
            }
        } else memcpy(target,row,bytes);
    }
    return TRUE;
}



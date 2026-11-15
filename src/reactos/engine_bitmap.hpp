/*
Module: lightning.reactos.engine_bitmap
Purpose: Own bounded temporary GDI bitmaps for canonical operand normalization.
Created: 2026-10-21
Architecture: Existing EngCopyBits supplies translation/orientation; no rasterizer.
Include after native Winddi types. Caller serializes borrowed source access.
*/
#pragma once
#include <stdint.h>
#include <string.h>

namespace lightning::reactos {
class EngineBitmap final {
public:
    // PASSIVE_LEVEL, caller owns the object and its native allocation until
    // destruction. Only a complete positive-pitch engine allocation is exposed.
    bool create(LONG width,LONG height,ULONG format,uint32_t maximum_bytes) noexcept {
        if (bitmap_!=nullptr || width<=0 || height<=0) return false;
        const uint32_t bits=format==BMF_1BPP ? 1U : (format==BMF_8BPP ? 8U : (format==BMF_32BPP ? 32U : 0U));
        if (bits==0) return false;
        const uint64_t pitch=((static_cast<uint64_t>(width)*bits+31U)/32U)*sizeof(uint32_t);
        if (pitch>INT32_MAX) return false;
        const uint64_t extent=pitch*static_cast<uint32_t>(height);
        if (extent>maximum_bytes) return false;
        const SIZEL size{width,height};
        bitmap_=reinterpret_cast<HSURF>(EngCreateBitmap(size,static_cast<LONG>(pitch),format,BMF_TOPDOWN,nullptr));
        if (bitmap_==nullptr) return false;
        surface_=EngLockSurface(bitmap_);
        if (surface_==nullptr || surface_->pvBits==nullptr || surface_->pvScan0!=surface_->pvBits ||
            surface_->lDelta!=static_cast<LONG>(pitch) || extent>surface_->cjBits ||
            extent>UINTPTR_MAX-reinterpret_cast<uintptr_t>(surface_->pvScan0)) return false;
        bytes_=static_cast<uint32_t>(extent);
        return true;
    }

    bool copy(SURFOBJ* source,LONG width,LONG height,ULONG format,XLATEOBJ* translation,
              POINTL origin,uint32_t maximum_bytes) noexcept {
        if (bitmap_!=nullptr || source==nullptr || source->iType!=STYPE_BITMAP ||
            width<=0 || height<=0 || origin.x<0 || origin.y<0 ||
            static_cast<int64_t>(origin.x)+width>source->sizlBitmap.cx ||
            static_cast<int64_t>(origin.y)+height>source->sizlBitmap.cy) return false;
        if (!create(width,height,format,maximum_bytes)) return false;
        RECTL rectangle{0,0,width,height};
        if (!EngCopyBits(surface_,source,nullptr,translation,&rectangle,&origin)) {
            bytes_=0;
            return false;
        }
        return true;
    }

    // The pinned EngCopyBits enumerates complex clips in CD_ANY order. Its
    // per-rectangle memmove cannot preserve a source overwritten by an earlier
    // rectangle. Snapshot only that aliased, overlapping CPU fallback's bounded
    // source ROI; the final engine copy still applies the original XLATE once.
    // The destination is deliberately unchanged: the pinned engine performs
    // its own source-edge clamp before each vertical-flip clip operation.
    bool preserve_clipped_copy(SURFOBJ* destination,SURFOBJ*& source,CLIPOBJ* clip,
        const RECTL& rectangle,POINTL& point) noexcept {
        if (source==nullptr || source!=destination || clip==nullptr || clip->iDComplexity!=DC_COMPLEX)
            return true;
        // The pinned engine clamps the right/bottom edges only. Do not hand a
        // negative origin to an owned snapshot: its prefix is not allocated,
        // even if a caller expected a later clip rectangle to exclude it.
        if (point.x<0 || point.y<0) return false;
        const int64_t width=static_cast<int64_t>(rectangle.right)-rectangle.left;
        const bool flipped=rectangle.top>rectangle.bottom;
        const int64_t top=flipped ? rectangle.bottom : rectangle.top;
        const int64_t height=flipped ? static_cast<int64_t>(rectangle.top)-rectangle.bottom :
            static_cast<int64_t>(rectangle.bottom)-rectangle.top;
        if (width<=0 || height<=0 || source->sizlBitmap.cx<=0 || source->sizlBitmap.cy<=0) return true;
        const int64_t source_left=point.x;
        const int64_t source_top=point.y;
        const int64_t requested_right=static_cast<int64_t>(point.x)+width;
        const int64_t requested_bottom=static_cast<int64_t>(point.y)+height;
        const int64_t source_right=requested_right<source->sizlBitmap.cx ? requested_right : source->sizlBitmap.cx;
        const int64_t source_bottom=requested_bottom<source->sizlBitmap.cy ? requested_bottom : source->sizlBitmap.cy;
        if (source_left>=source_right || source_top>=source_bottom) return true;
        if (rectangle.left>=source_right || source_left>=rectangle.right ||
            top>=source_bottom || source_top>=top+height) return true;
        if (source->iBitmapFormat!=BMF_32BPP || source->cjBits==0) return false;
        const int64_t copied_width=source_right-source_left;
        const int64_t copied_height=source_bottom-source_top;
        const POINTL origin{static_cast<LONG>(source_left),static_cast<LONG>(source_top)};
        if (!copy(source,static_cast<LONG>(copied_width),static_cast<LONG>(copied_height),
            source->iBitmapFormat,nullptr,origin,source->cjBits)) return false;
        point={static_cast<LONG>(point.x-source_left),static_cast<LONG>(point.y-source_top)};
        source=surface_;
        return true;
    }

    SURFOBJ* surface() const noexcept {return surface_;}
    uint32_t bytes() const noexcept {return bytes_;}
    uint32_t stride() const noexcept {return surface_==nullptr ? 0U : static_cast<uint32_t>(surface_->lDelta);}
    bool append(void* destination,uint32_t capacity,uint32_t& offset) const noexcept {
        if (surface_==nullptr || bytes_==0 || destination==nullptr || offset>capacity || bytes_>capacity-offset) return false;
        const uintptr_t address=reinterpret_cast<uintptr_t>(destination);
        if (capacity>UINTPTR_MAX-address) return false;
        // Both spans are owned/validated native buffers. No caller-supplied
        // address is retained, and the destination payload is compact.
        memcpy(static_cast<uint8_t*>(destination)+offset,surface_->pvScan0,bytes_);
        offset+=bytes_;
        return true;
    }

    EngineBitmap() noexcept=default;
    ~EngineBitmap() {
        if (surface_!=nullptr) EngUnlockSurface(surface_);
        if (bitmap_!=nullptr) EngDeleteSurface(bitmap_);
    }
    EngineBitmap(const EngineBitmap&)=delete;
    EngineBitmap& operator=(const EngineBitmap&)=delete;
private:
    HSURF bitmap_=nullptr;
    SURFOBJ* surface_=nullptr;
    uint32_t bytes_=0;
};
} // namespace lightning::reactos

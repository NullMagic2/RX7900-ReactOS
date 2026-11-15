/*
Module: lightning.reactos.gdi_client
Purpose: Offload supported raster operations and synchronize every CPU fallback.
Created: 2026-10-21
Architecture: Reuses EngCopyBits for source normalization/cache readback; no new
software rasterizer. Accepted commands are never replayed after uncertainty.
License: GPL-2.0-or-later.
*/
#include "framebuf.h"
#include "lightning_gdi_client.hpp"
#include "lightning_present_abi.hpp"
#include "lightning_display.hpp"
#include "engine_bitmap.hpp"
#include "brush_snapshot.hpp"
#include "stretch_mask.hpp"

namespace lightning::reactos::gdi {
namespace {
namespace core=::lightning::gdi;
namespace policy=rx7900::reactos::framebuffer;

LightningDisplayRect wire(Rectangle rectangle) noexcept {
    return {static_cast<uint32_t>(rectangle.left),static_cast<uint32_t>(rectangle.top),
        static_cast<uint32_t>(rectangle.right-rectangle.left),
        static_cast<uint32_t>(rectangle.bottom-rectangle.top)};
}

bool control(HANDLE device,uint32_t code,void* input,uint32_t input_bytes,
             void* output,uint32_t output_bytes) noexcept {
    DWORD returned=0;
    return EngDeviceIoControl(device,code,input,input_bytes,output,output_bytes,&returned)==0 && returned==output_bytes;
}

void increment(uint64_t& value) noexcept { if (value!=UINT64_MAX) ++value; }

enum class CoverageCopy { unavailable, copied, invalid };

CoverageCopy append_coverage(SURFOBJ* source,POINTL origin,uint32_t width,uint32_t height,
    void* destination,uint32_t capacity,uint32_t& offset,uint32_t& pitch,uint32_t& bytes) noexcept {
    pitch=0;
    bytes=0;
    if (source==nullptr || source->iType!=STYPE_BITMAP || source->iBitmapFormat!=BMF_8BPP) return CoverageCopy::unavailable;
    if (source->pvScan0==nullptr) return CoverageCopy::invalid;
    if (source->lDelta<=0 || source->pvBits==nullptr) return CoverageCopy::unavailable;
    if (destination==nullptr || width==0 || height==0 || origin.x<0 || origin.y<0 ||
        source->sizlBitmap.cx<=0 || source->sizlBitmap.cy<=0 || source->lDelta<source->sizlBitmap.cx ||
        static_cast<uint64_t>(origin.x)+width>static_cast<uint32_t>(source->sizlBitmap.cx) ||
        static_cast<uint64_t>(origin.y)+height>static_cast<uint32_t>(source->sizlBitmap.cy)) return CoverageCopy::invalid;
    const uintptr_t base=reinterpret_cast<uintptr_t>(source->pvBits);
    const uintptr_t scan=reinterpret_cast<uintptr_t>(source->pvScan0);
    const uintptr_t target=reinterpret_cast<uintptr_t>(destination);
    if (source->cjBits>UINTPTR_MAX-base || capacity>UINTPTR_MAX-target || scan<base || scan-base>source->cjBits)
        return CoverageCopy::invalid;
    const uint64_t source_begin=static_cast<uint64_t>(scan-base)+static_cast<uint64_t>(origin.y)*source->lDelta+origin.x;
    const uint64_t source_end=source_begin+static_cast<uint64_t>(height-1U)*source->lDelta+width;
    const uint64_t row_bytes=(static_cast<uint64_t>(width)+3U)&~3ULL;
    const uint64_t total=row_bytes*height;
    if (source_end>source->cjBits || offset>capacity) return CoverageCopy::invalid;
    if (total>capacity-offset) return CoverageCopy::unavailable;
    // Aliasing operands use the existing temporary engine bitmap snapshot.
    // Otherwise the caller's live glyph and owned native stage are disjoint.
    if (base<target+capacity && target<base+source->cjBits) return CoverageCopy::unavailable;
    auto* target_row=static_cast<uint8_t*>(destination)+offset;
    const auto* source_row=static_cast<const uint8_t*>(source->pvBits)+static_cast<size_t>(source_begin);
    for (uint32_t row=0;row<height;++row) {
        memcpy(target_row,source_row,width);
        memset(target_row+width,0,static_cast<size_t>(row_bytes)-width);
        // Do not form a pointer beyond either allocation after the last row.
        if (row+1U<height) {
            source_row+=source->lDelta;
            target_row+=static_cast<size_t>(row_bytes);
        }
    }
    pitch=static_cast<uint32_t>(row_bytes);
    bytes=static_cast<uint32_t>(total);
    offset+=bytes;
    return CoverageCopy::copied;
}

bool append_key_mask(const EngineBitmap& raw,uint32_t key,void* destination,uint32_t capacity,
    uint32_t& offset,uint32_t& pitch,uint32_t& bytes) noexcept {
    SURFOBJ* surface=raw.surface();
    if (surface==nullptr || surface->iBitmapFormat!=BMF_32BPP || surface->sizlBitmap.cx<=0 ||
        surface->sizlBitmap.cy<=0 || destination==nullptr || offset>capacity) return false;
    const uint32_t width=static_cast<uint32_t>(surface->sizlBitmap.cx);
    const uint32_t height=static_cast<uint32_t>(surface->sizlBitmap.cy);
    const uint64_t row=((static_cast<uint64_t>(width)+31U)/32U)*sizeof(uint32_t);
    const uint64_t total=row*height;
    if (total>capacity-offset || static_cast<uint64_t>(raw.stride())*height>raw.bytes()) return false;
    // The engine snapshot owns normalized raw source indices. Classification
    // creates only a transparency mask; the GPU performs the actual draw.
    auto* mask=static_cast<uint8_t*>(destination)+offset;
    memset(mask,0,static_cast<size_t>(total));
    const auto* pixels=static_cast<const uint8_t*>(surface->pvScan0);
    for (uint32_t y=0;y<height;++y) {
        const auto* source_row=pixels+static_cast<size_t>(y)*raw.stride();
        auto* target_row=mask+static_cast<size_t>(y)*static_cast<size_t>(row);
        for (uint32_t x=0;x<width;++x) {
            uint32_t value=0;
            memcpy(&value,source_row+static_cast<size_t>(x)*sizeof(value),sizeof(value));
            if ((value&0x00ffffffU)!=(key&0x00ffffffU)) target_row[x/8U]|=static_cast<uint8_t>(0x80U>>(x&7U));
        }
    }
    pitch=static_cast<uint32_t>(row);
    bytes=static_cast<uint32_t>(total);
    offset+=bytes;
    return true;
}

// DrvStretchBltROP's 1bpp mask becomes the destination draw-decision plane
// (LIGHTNING_GDI_CAP_STRETCH_MASK). The engine ignores MaskOrigin and samples
// the mask at the stretched source texel; malformed ownership is a failure.
CoverageCopy append_stretch_mask(SURFOBJ* mask,const RasterInput& input,const ClipBatch& clip,
    void* destination,uint32_t capacity,uint32_t& offset,uint32_t& pitch,uint32_t& bytes) noexcept {
    pitch=0;
    bytes=0;
    if (mask==nullptr || mask->iType!=STYPE_BITMAP || mask->iBitmapFormat!=BMF_1BPP) return CoverageCopy::unavailable;
    if (mask->pvScan0==nullptr || mask->pvBits==nullptr) return CoverageCopy::invalid;
    const uintptr_t base=reinterpret_cast<uintptr_t>(mask->pvBits);
    const uintptr_t scan=reinterpret_cast<uintptr_t>(mask->pvScan0);
    if (scan<base || scan-base>mask->cjBits || destination==nullptr || offset>capacity) return CoverageCopy::invalid;
    const uint64_t width=static_cast<uint64_t>(clip.bounds.right-clip.bounds.left);
    const uint64_t height=static_cast<uint64_t>(clip.bounds.bottom-clip.bounds.top);
    const uint64_t row=((width+31U)/32U)*sizeof(uint32_t);
    if (row*height>capacity-offset) return CoverageCopy::unavailable;
    StretchMaskSource source{};
    source.bits=static_cast<const uint8_t*>(mask->pvBits);
    source.bytes=mask->cjBits;
    source.scan0_offset=scan-base;
    source.delta=mask->lDelta;
    source.width=mask->sizlBitmap.cx;
    source.height=mask->sizlBitmap.cy;
    StretchMaskCall call{};
    call.output={input.original.left,input.original.top,input.original.right,input.original.bottom};
    call.input={input.source_rectangle.left,input.source_rectangle.top,
        input.source_rectangle.right,input.source_rectangle.bottom};
    call.mirror_x=(input.raster_flags&LIGHTNING_GDI_RASTER_MIRROR_X)!=0;
    call.mirror_y=(input.raster_flags&LIGHTNING_GDI_RASTER_MIRROR_Y)!=0;
    if (!stretch_mask_plane(call,source,clip.rectangles,clip.count,clip.bounds,
        static_cast<uint8_t*>(destination)+offset,static_cast<size_t>(row*height),static_cast<uint32_t>(row)))
        return CoverageCopy::unavailable;
    pitch=static_cast<uint32_t>(row);
    bytes=static_cast<uint32_t>(row*height);
    offset+=bytes;
    return CoverageCopy::copied;
}

bool copy_source(const ClipBatch& clip,SURFOBJ* source,XLATEOBJ* translation,
                 RECTL original,POINTL point,SURFOBJ* staging) noexcept {
    if (source==nullptr || staging==nullptr || source->iType!=STYPE_BITMAP) return false;
    LONG x=0;
    LONG y=0;
    int32_t checked_x=0;
    int32_t checked_y=0;
    if (!source_position({original.left,original.top,original.right,original.bottom},clip.bounds,
        point.x,point.y,source->sizlBitmap.cx,source->sizlBitmap.cy,checked_x,checked_y)) return false;
    x=checked_x;
    y=checked_y;
    const LONG width=clip.bounds.right-clip.bounds.left;
    const LONG height=clip.bounds.bottom-clip.bounds.top;
    if (width>staging->sizlBitmap.cx || height>staging->sizlBitmap.cy) return false;
    RECTL destination{0,0,width,height};
    POINTL origin{x,y};
    return EngCopyBits(staging,source,nullptr,translation,&destination,&origin)!=FALSE;
}

bool gradient_translation(XLATEOBJ* translation,bool& swap_red_blue) noexcept {
    swap_red_blue=false;
    if (translation==nullptr || translation->flXlate==XO_TRIVIAL) {
        // RGB() writes a COLORREF value unchanged in the identity engine path.
        swap_red_blue=true;
        return true;
    }
    if (translation->flXlate!=0 ||
        (translation->iSrcType&(PAL_RGB|PAL_BGR|PAL_INDEXED))!=PAL_RGB ||
        (translation->iDstType&(PAL_RGB|PAL_BGR|PAL_INDEXED))!=PAL_BGR) return false;
    // Palette flags may include PAL_BITFIELDS. Verify the actual normal RGB
    // channel mapping without admitting tables, ICM or arbitrary transforms.
    return XLATEOBJ_iXlate(translation,0)==0 &&
        XLATEOBJ_iXlate(translation,0x000000ffU)==0x00ff0000U &&
        XLATEOBJ_iXlate(translation,0x0000ff00U)==0x0000ff00U &&
        XLATEOBJ_iXlate(translation,0x00ff0000U)==0x000000ffU &&
        XLATEOBJ_iXlate(translation,0x00ffffffU)==0x00ffffffU;
}

bool gradient_coordinate(int64_t value) noexcept {
    return value>=-static_cast<int64_t>(LIGHTNING_GDI_GRADIENT_COORDINATE_LIMIT) &&
        value<=LIGHTNING_GDI_GRADIENT_COORDINATE_LIMIT;
}

bool gradient_element(const TRIVERTEX* vertices,uint32_t vertex_count,const void* mesh,uint32_t index,
    uint32_t mode,RECTL extents,POINTL origin,bool swap_red_blue,
    LightningGdiGradientOperands& output,RECTL& requested) noexcept {
    output={};
    requested={};
    const bool triangle=mode==GRADIENT_FILL_TRIANGLE;
    GRADIENT_RECT rectangle{};
    GRADIENT_TRIANGLE polygon{};
    // NtGdiGradientFill captured these native arrays before locking the DC.
    // Length arithmetic is checked by the caller; no borrowed pointer escapes.
    const auto* bytes=static_cast<const uint8_t*>(mesh);
    if (triangle) memcpy(&polygon,bytes+static_cast<size_t>(index)*sizeof(polygon),sizeof(polygon));
    else memcpy(&rectangle,bytes+static_cast<size_t>(index)*sizeof(rectangle),sizeof(rectangle));
    output.size=sizeof(output);
    output.version=LIGHTNING_GDI_GRADIENT_VERSION;
    output.mode=mode;
    output.origin_x=triangle ? origin.x : 0;
    output.origin_y=triangle ? origin.y : 0;
    if (!gradient_coordinate(output.origin_x) || !gradient_coordinate(output.origin_y) ||
        !gradient_coordinate(extents.left) || !gradient_coordinate(extents.right) ||
        !gradient_coordinate(extents.top) || !gradient_coordinate(extents.bottom) ||
        extents.right<=extents.left || extents.bottom<=extents.top) return false;
    output.extents={extents.left,extents.top,static_cast<uint32_t>(static_cast<int64_t>(extents.right)-extents.left),
        static_cast<uint32_t>(static_cast<int64_t>(extents.bottom)-extents.top)};
    for (uint32_t corner=0;corner<(triangle ? 3U : 2U);++corner) {
        const uint32_t vertex_index=triangle ?
            (corner==0 ? polygon.Vertex1 : (corner==1 ? polygon.Vertex2 : polygon.Vertex3)) :
            (corner==0 ? rectangle.UpperLeft : rectangle.LowerRight);
        if (vertex_index>=vertex_count) return false;
        TRIVERTEX vertex{};
        memcpy(&vertex,vertices+vertex_index,sizeof(vertex));
        if (!gradient_coordinate(vertex.x) || !gradient_coordinate(vertex.y) ||
            !gradient_coordinate(static_cast<int64_t>(vertex.x)+output.origin_x) ||
            !gradient_coordinate(static_cast<int64_t>(vertex.y)+output.origin_y)) return false;
        const uint32_t red=swap_red_blue ? vertex.Blue : vertex.Red;
        const uint32_t blue=swap_red_blue ? vertex.Red : vertex.Blue;
        output.vertices[corner]={vertex.x,vertex.y,red|(static_cast<uint32_t>(vertex.Green)<<16),blue};
    }
    if (triangle) {
        for (uint32_t first=0;first<3;++first)
            for (uint32_t second=first+1;second<3;++second)
                if (output.vertices[first].x==output.vertices[second].x &&
                    output.vertices[first].y==output.vertices[second].y) return true;
        requested=extents;
    } else {
        const auto first=output.vertices[0];
        const auto second=output.vertices[1];
        requested={first.x<second.x ? first.x : second.x,first.y<second.y ? first.y : second.y,
            first.x>second.x ? first.x : second.x,first.y>second.y ? first.y : second.y};
    }
    return true;
}
} // namespace

bool capture_clip(RECTL requested,CLIPOBJ* clip,uint32_t width,uint32_t height,ClipBatch& output) noexcept {
    ClipBatch& candidate=output;
    candidate.bounds={};
    candidate.count=0;
    if (!draw_bounds({requested.left,requested.top,requested.right,requested.bottom},
        static_cast<int32_t>(width),static_cast<int32_t>(height),candidate.bounds)) return false;
    if (policy::empty(candidate.bounds)) return true;
    if (clip!=nullptr && clip->iDComplexity!=DC_TRIVIAL) candidate.bounds=policy::intersect(candidate.bounds,
        {clip->rclBounds.left,clip->rclBounds.top,clip->rclBounds.right,clip->rclBounds.bottom});
    if (policy::empty(candidate.bounds)) return true;
    if (clip!=nullptr && clip->iDComplexity==DC_COMPLEX) {
        const ULONG count=CLIPOBJ_cEnumStart(clip,FALSE,CT_RECTANGLES,CD_ANY,LIGHTNING_GDI_MAX_RECTS);
        if (count==UINT32_MAX || count>LIGHTNING_GDI_MAX_RECTS) return false;
        ClipBatch::Enumeration& enumeration=candidate.enumeration;
        enumeration.count=0;
        const BOOL more=CLIPOBJ_bEnum(clip,sizeof(enumeration),reinterpret_cast<ULONG*>(&enumeration));
        if (more || enumeration.count>LIGHTNING_GDI_MAX_RECTS) return false;
        for (uint32_t i=0;i<enumeration.count;++i) {
            const auto& rectangle=enumeration.rectangles[i];
            const Rectangle clipped=policy::intersect(candidate.bounds,
                {rectangle.left,rectangle.top,rectangle.right,rectangle.bottom});
            if (!policy::empty(clipped)) candidate.rectangles[candidate.count++]=wire(clipped);
        }
        if (candidate.count==0) candidate.bounds={};
    } else if (clip!=nullptr && clip->iDComplexity!=DC_TRIVIAL && clip->iDComplexity!=DC_RECT) return false;
    if (!policy::empty(candidate.bounds) && !core::validate_clip_batch(width,height,wire(candidate.bounds),
        candidate.rectangles,candidate.count)) return false;
    return true;
}

bool ClientCanvas::valid_information(const Information& information) const noexcept {
    return information.size==sizeof(information) && information.version==native_abi_version &&
        information.magic==LIGHTNING_GDI_MAGIC && information.width==width_ && information.height==height_ &&
        information.mode_generation!=0 && information.active<=1 && information.quarantined<=1 &&
        information.completed_sequence<=information.accepted_sequence &&
        information.canvas_revision<=information.accepted_sequence &&
        ((information.accepted_sequence==0 && information.epoch==0 && information.canvas_revision==0) ||
         (information.accepted_sequence!=0 && information.epoch!=0));
}

bool ClientCanvas::owner_offline() const noexcept {
    CpuScanoutInformation information{};
    return query_display_owner(device_,information) && information.enabled==0 &&
        information.mode_generation==generation_ && information.width==width_ && information.height==height_;
}

Attempt ClientCanvas::initialize(PPDEV device) noexcept {
    // Reinitialization follows a successful Suspend/retire and an acknowledged
    // native mode/MAP transition. Never silently replace a live canvas owner.
    if (active_ || visibility_.poisoned() || !visibility_.readable() || glyphs_.glyph_count!=0) return Attempt::failed;
    release_cpu_storage();
    active_=false;
    visibility_.reinitialize();
    promotable_=false;
    if (device==nullptr) return Attempt::unsupported;
    device_=device->hDriver;
    width_=device->ScreenWidth;
    height_=device->ScreenHeight;
    generation_=0;
    last_sequence_=0;
    last_epoch_=0;
    last_revision_=0;
    text_scope_=false;
    group_sequence_=0;
    LONGLONG frequency=0;
    LONGLONG now=0;
    EngQueryPerformanceFrequency(&frequency);
    EngQueryPerformanceCounter(&now);
    promotion_.configure(frequency>0 && now>=0 ? static_cast<uint64_t>(frequency) : 0,
        now>=0 ? static_cast<uint64_t>(now) : 0);
    Information information{};
    if (!control(device_,ioctl_query,nullptr,0,&information,sizeof(information))) return Attempt::unsupported;
    if (!valid_information(information)) return Attempt::failed;
    generation_=information.mode_generation;
    CpuScanoutInformation owner{};
    if (!query_owned_staging(device_,device->ScreenPtr,owner) || owner.mode_generation!=generation_ || owner.enabled!=1 ||
        owner.width!=width_ || owner.height!=height_ || owner.stride!=device->ScreenDelta ||
        device->ScreenPtr==nullptr || static_cast<uint64_t>(owner.stride)*height_>owner.mapped_bytes) return Attempt::failed;
    staging_base_=device->ScreenPtr;
    staging_bytes_=owner.mapped_bytes;
    if (staging_bytes_>UINTPTR_MAX-reinterpret_cast<uintptr_t>(staging_base_)) return Attempt::failed;
    promotable_=true;
    return bind(information);
}

Attempt ClientCanvas::reinitialize_after_mode(PPDEV device) noexcept {
    if (!active_ && !visibility_.poisoned() && visibility_.readable() && glyphs_.glyph_count==0)
        return initialize(device);
    if (device==nullptr || device->hDriver!=device_) return Attempt::failed;
    CpuScanoutInformation owner{};
    Information observed{};
    if (!query_display_owner(device_,owner) || owner.enabled!=1 || owner.mode_generation<=generation_ ||
        owner.width!=width_ || owner.height!=height_ || owner.stride!=device->ScreenDelta ||
        !control(device_,ioctl_query,nullptr,0,&observed,sizeof(observed)) || !valid_information(observed) ||
        observed.mode_generation!=owner.mode_generation || observed.active!=0 || observed.quarantined!=0 ||
        observed.accepted_sequence!=0 || observed.completed_sequence!=0 || observed.epoch!=0 ||
        observed.canvas_revision!=0 || observed.reset_generation==0 ||
        (reset_generation_!=0 && observed.reset_generation<=reset_generation_)) return Attempt::failed;
    // Native reset proof excludes further reads of prior transfer/glyph bytes.
    // The caller excludes callbacks and owns the retained complete CPU cache.
    release_cpu_storage();
    visibility_.reinitialize();
    active_=false;
    promotable_=false;
    text_scope_=false;
    group_sequence_=0;
    return initialize(device);
}

Attempt ClientCanvas::bind(const Information& information) noexcept {
    if (!valid_information(information) || information.mode_generation!=generation_ ||
        information.quarantined!=0 || information.accepted_sequence!=0) return Attempt::failed;
    if ((information.capabilities&LIGHTNING_GDI_BASE_CAPABILITIES)!=LIGHTNING_GDI_BASE_CAPABILITIES) return Attempt::unsupported;
    if (information.reset_generation==0) return Attempt::failed;
    last_sequence_=0;
    last_epoch_=0;
    last_revision_=0;
    reset_generation_=information.reset_generation;
    capabilities_=information.capabilities;
    LightningGdiCommand command{};
    command.opcode=LIGHTNING_GDI_BIND;
    command.destination={0,0,width_,height_};
    const Attempt result=submit(command,nullptr,0);
    if (result!=Attempt::performed) return result;
    active_=true;
    if (!wait()) return Attempt::failed;
    return Attempt::performed;
}

Attempt ClientCanvas::submit(LightningGdiCommand command,const LightningDisplayRect* clips,
                             uint32_t count,uint32_t source_x,uint32_t source_y,
                             const LightningGdiRasterOperands* operands,
                             const LightningGdiGradientOperands* gradient,
                             const LightningGdiExternalImage* external,
                             uint32_t abi_version) noexcept {
    if (visibility_.poisoned()) return Attempt::failed;
    if (count>LIGHTNING_GDI_MAX_RECTS || (count!=0 && clips==nullptr)) return Attempt::unsupported;
    if ((command.opcode==LIGHTNING_GDI_RASTER)!=(operands!=nullptr) ||
        (command.opcode==LIGHTNING_GDI_GRADIENT)!=(gradient!=nullptr) ||
        (command.opcode==LIGHTNING_GDI_EXTERNAL_IMAGE)!=(external!=nullptr)) return Attempt::unsupported;
    // async-present: version 2 exists only for external images.
    const bool async_external=external!=nullptr && abi_version==native_async_abi_version;
    if (abi_version!=native_abi_version && !async_external) return Attempt::unsupported;
    external_pending_=false;
    Submission& packet=submission_;
    command.magic=LIGHTNING_GDI_MAGIC;
    command.version=LIGHTNING_GDI_VERSION;
    command.mode_generation=generation_;
    command.clip_count=count;
    const uint32_t clip_bytes=count*static_cast<uint32_t>(sizeof(LightningDisplayRect));
    if (!core::metadata_size(command.opcode,count,command.metadata_bytes)) return Attempt::unsupported;
    const uint32_t operand_bytes=operands!=nullptr ? sizeof(*operands) :
        (gradient!=nullptr ? sizeof(*gradient) : (external!=nullptr ? sizeof(*external) : 0U));
    const uint32_t prefix=sizeof(SubmitRequest)-sizeof(LightningGdiCommand);
    packet.request={prefix+command.metadata_bytes,abi_version,source_x,source_y,command};
    if (operands!=nullptr) memcpy(packet.tail,operands,sizeof(*operands));
    if (gradient!=nullptr) memcpy(packet.tail,gradient,sizeof(*gradient));
    if (external!=nullptr) memcpy(packet.tail,external,sizeof(*external));
    if (clip_bytes!=0) memcpy(packet.tail+operand_bytes,clips,clip_bytes);
    Receipt receipt{};
    if (!control(device_,ioctl_submit,&packet,packet.request.size,&receipt,sizeof(receipt)) ||
        receipt.size!=sizeof(receipt) || receipt.version!=native_abi_version || receipt.mode_generation!=generation_ ||
        receipt.outcome==uncertain || receipt.outcome>uncertain) {
        visibility_.poison();
        return Attempt::failed;
    }
    if (receipt.outcome==declined) {
        if (receipt.epoch!=last_epoch_ || receipt.accepted_sequence!=last_sequence_ ||
            receipt.completed_sequence>last_sequence_ || receipt.canvas_revision!=last_revision_) {
            visibility_.poison();
            return Attempt::failed;
        }
        return Attempt::unsupported;
    }
    const bool binding=command.opcode==LIGHTNING_GDI_BIND;
    const bool mutation=command.opcode==LIGHTNING_GDI_FILL || command.opcode==LIGHTNING_GDI_COPY ||
        command.opcode==LIGHTNING_GDI_UPLOAD || command.opcode==LIGHTNING_GDI_BLEND_UPLOAD ||
        command.opcode==LIGHTNING_GDI_ROP3 || command.opcode==LIGHTNING_GDI_RASTER ||
        command.opcode==LIGHTNING_GDI_GRADIENT || command.opcode==LIGHTNING_GDI_EXTERNAL_IMAGE;
    if (receipt.result!=LIGHTNING_GDI_OK || receipt.epoch==0 ||
        (!binding && (receipt.epoch!=last_epoch_ || last_sequence_==UINT32_MAX)) ||
        receipt.accepted_sequence!=(binding ? 1U : last_sequence_+1U) ||
        receipt.completed_sequence>receipt.accepted_sequence ||
        (external!=nullptr && !async_external && receipt.completed_sequence!=receipt.accepted_sequence) ||
        receipt.canvas_revision!=(mutation ? receipt.accepted_sequence : (binding ? 0U : last_revision_))) {
        visibility_.poison();
        return Attempt::failed;
    }
    last_sequence_=receipt.accepted_sequence;
    last_epoch_=receipt.epoch;
    last_revision_=receipt.canvas_revision;
    // The native filter pins a version-2 source until its read retires; the
    // ordered completion prefix here proves it already did, or not yet.
    external_pending_=async_external && receipt.completed_sequence<receipt.accepted_sequence;
    return Attempt::performed;
}

bool ClientCanvas::wait() noexcept {
    WaitRequest request{sizeof(request),native_abi_version,generation_,last_sequence_};
    Receipt receipt{};
    if (visibility_.poisoned() || !control(device_,ioctl_wait,&request,sizeof(request),&receipt,sizeof(receipt)) ||
        receipt.size!=sizeof(receipt) || receipt.version!=native_abi_version || receipt.outcome!=accepted ||
        receipt.result!=LIGHTNING_GDI_OK || receipt.mode_generation!=generation_ ||
        receipt.epoch!=last_epoch_ || receipt.accepted_sequence!=last_sequence_ ||
        receipt.completed_sequence!=last_sequence_ || receipt.canvas_revision!=last_revision_) {
        visibility_.poison();
        return false;
    }
    return true;
}

bool ClientCanvas::ensure_group() noexcept {
    if (visibility_.poisoned()) return false;
    if (!text_scope_ && group_sequence_!=0) {
        if (!flush_glyphs()) return false;
        LightningGdiCommand command{};
        command.opcode=LIGHTNING_GDI_END_GROUP;
        command.reference=group_sequence_;
        if (submit(command,nullptr,0)!=Attempt::performed) {
            visibility_.poison();
            return false;
        }
        group_sequence_=0;
        return true;
    }
    if (!text_scope_ || group_sequence_!=0 || !active_ ||
        (capabilities_&LIGHTNING_GDI_CAP_GROUPS)==0) return true;
    LightningGdiCommand command{};
    command.opcode=LIGHTNING_GDI_BEGIN_GROUP;
    if (submit(command,nullptr,0)!=Attempt::performed) return false;
    group_sequence_=last_sequence_;
    return true;
}

bool ClientCanvas::begin_text() noexcept {
    if (text_scope_ || group_sequence_!=0 || glyphs_.glyph_count!=0 || visibility_.poisoned()) return false;
    text_scope_=true;
    if (ensure_group()) return true;
    text_scope_=false;
    return false;
}

bool ClientCanvas::end_text() noexcept {
    const bool flushed=flush_glyphs();
    text_scope_=false;
    if (!flushed) return false;
    // D3 is a known unavailable owner, not evidence of unknown execution. Keep
    // the token until D0 proves retirement, or closes the surviving group.
    if (group_sequence_!=0 && owner_offline()) return false;
    return ensure_group();
}

void ClientCanvas::abort_text() noexcept {
    // Known CPU dirty bytes did not reach the canvas. Never send END or
    // auto-close this group before the native owner actually retires it.
    text_scope_=false;
    visibility_.poison();
}

bool ClientCanvas::upload(Rectangle rectangle,const LightningDisplayRect* clips,uint32_t count,bool normalized) noexcept {
    const LightningDisplayRect batch=wire(rectangle);
    core::StripeCursor cursor;
    if (!cursor.begin(batch,width_,height_)) return false;
    LightningDisplayRect stripe{};
    if (!cursor.current(stripe) || (count!=0 && stripe.height!=batch.height)) return false;
    uint32_t reference=0;
    bool first=true;
    while (!cursor.done()) {
        if (!cursor.current(stripe)) break;
        uint32_t row_bytes=0;
        uint32_t pixel_bytes=0;
        if (!core::packed_transfer_size(stripe.width,stripe.height,row_bytes,pixel_bytes)) break;
        LightningGdiCommand command{};
        command.opcode=LIGHTNING_GDI_UPLOAD;
        command.destination=stripe;
        command.batch=batch;
        command.reference=reference;
        command.pixel_bytes=pixel_bytes;
        command.flags=(first ? LIGHTNING_GDI_UPLOAD_BEGIN : 0U) |
            (stripe.y+stripe.height==batch.y+batch.height ? LIGHTNING_GDI_UPLOAD_END : 0U);
        const auto result=submit(command,clips,count,normalized ? 0U : stripe.x,
            normalized ? stripe.y-batch.y : stripe.y);
        if (result!=Attempt::performed) {
            if (!first) visibility_.poison();
            return false;
        }
        if (first) reference=last_sequence_;
        first=false;
        if (!cursor.advance(stripe)) break;
    }
    if (!cursor.done()) {visibility_.poison(); return false;}
    increment(statistics_.accepted_uploads);
    return true;
}

bool ClientCanvas::upload_cached(Rectangle rectangle) noexcept {
    // The caller populated staging from its cache. It must flush queued glyphs
    // before that copy, since their snapshot would overwrite these upload bytes.
    return active_ && glyphs_.glyph_count==0 && !visibility_.poisoned() && upload(rectangle,nullptr,0,false);
}

Attempt ClientCanvas::fill(const ClipBatch& clip,uint32_t color,SURFOBJ*,CLIPOBJ*) noexcept {
    if (!active_) return Attempt::unsupported;
    if (!flush_glyphs()) return Attempt::failed;
    if (policy::empty(clip.bounds)) return Attempt::performed;
    LightningGdiCommand command{};
    command.opcode=LIGHTNING_GDI_FILL;
    command.destination=wire(clip.bounds);
    command.color=color;
    const auto result=submit(command,clip.rectangles,clip.count);
    if (result==Attempt::performed) {
        // GPU drawing does not replay the same pixels into the CPU cache.
        // A later actual CPU consumer must gather the observed GPU prefix;
        // bounded stale ownership also preserves reset/D3 snapshot recovery.
        visibility_.gpu_write(clip.bounds);
        increment(statistics_.accepted_fills);
    }
    return result;
}

Attempt ClientCanvas::copy(const ClipBatch& clip,uint32_t source_x,uint32_t source_y) noexcept {
    if (!active_) return Attempt::unsupported;
    if (!flush_glyphs()) return Attempt::failed;
    if (policy::empty(clip.bounds)) return Attempt::performed;
    LightningGdiCommand command{};
    command.opcode=LIGHTNING_GDI_COPY;
    command.destination=wire(clip.bounds);
    command.source_x=source_x;
    command.source_y=source_y;
    const auto result=submit(command,clip.rectangles,clip.count);
    if (result==Attempt::performed) {visibility_.gpu_write(clip.bounds); increment(statistics_.accepted_copies);}
    return result;
}

Attempt ClientCanvas::external_image(const ClipBatch& clip,uint32_t source_x,uint32_t source_y,
    const LightningGdiExternalImage& image) noexcept {
    if (!active_ || (capabilities_&LIGHTNING_GDI_CAP_EXTERNAL_IMAGE)==0) return Attempt::unsupported;
    if (!flush_glyphs()) return Attempt::failed;
    if (policy::empty(clip.bounds)) return Attempt::performed;
    if (image.context!=0 || image.resource!=0 || image.lease==0) return Attempt::unsupported;
    return submit_external(clip,source_x,source_y,image,native_abi_version);
}

Attempt ClientCanvas::external_image(const ClipBatch& clip,uint32_t source_x,uint32_t source_y,
    const LightningGdiExternalImage& image,const ExternalOptions& options,bool& pending) noexcept {
    pending=false;
    if (!active_ || (capabilities_&LIGHTNING_GDI_CAP_EXTERNAL_IMAGE)==0 || !async_external())
        return Attempt::unsupported;
    if (!flush_glyphs()) return Attempt::failed;
    if (policy::empty(clip.bounds)) return Attempt::performed;
    if (image.context!=0 || image.resource!=0 || image.lease==0 ||
        (options.producer_sync==0)!=(options.producer_value==0)) return Attempt::unsupported;
    // Version-2 input: the filter reads the producer point from these two
    // fields and replaces them with the owned image identity.
    LightningGdiExternalImage named=image;
    named.context=options.producer_value;
    named.resource=options.producer_sync;
    const auto result=submit_external(clip,source_x,source_y,named,native_async_abi_version);

    if (result!=Attempt::performed || !external_pending_) return result;
    // A synchronous revision-2 caller (producer point only) still owns the
    // wait for this read, now without the native filter guard held.
    if (!options.async) return wait() ? Attempt::performed : Attempt::failed;
    pending=true;
    return result;
}

Attempt ClientCanvas::submit_external(const ClipBatch& clip,uint32_t source_x,uint32_t source_y,
    const LightningGdiExternalImage& image,uint32_t abi_version) noexcept {
    LightningGdiCommand command{};
    command.opcode=LIGHTNING_GDI_EXTERNAL_IMAGE;
    command.destination=wire(clip.bounds);
    command.source_x=source_x;
    command.source_y=source_y;
    const auto result=submit(command,clip.rectangles,clip.count,0,0,nullptr,nullptr,&image,abi_version);

    if (result==Attempt::performed) {
        // The native filter pins the actual image owner until the GPU read
        // retires: within this synchronous submission for version 1, through
        // its per-image receipt for version 2. CPU cache stays stale.
        visibility_.gpu_write(clip.bounds);
        increment(statistics_.accepted_copies);
    }
    return result;
}

bool ClientCanvas::async_external() noexcept {
    if (async_probe_==0 && device_!=nullptr) {
        ExternalReceiptRequest request{sizeof(request),native_abi_version,0};
        ExternalReceipt reply{};
        const bool answered=control(device_,ioctl_external_receipt,&request,sizeof(request),&reply,sizeof(reply)) &&
            reply.size==sizeof(reply) && reply.version==native_abi_version;
        // A current filter answers; features decide. An older filter forwards
        // the request and its miniport rejects it. Only support is cached: an
        // undecided stack (not yet started, D3) is asked again next time.
        if (answered && (reply.features&external_feature_async)!=0) async_probe_=1;
    }
    return async_probe_==1;
}

bool ClientCanvas::external_receipt(uint64_t lease,uint32_t& status) noexcept {
    status=external_receipt_failed;
    if (device_==nullptr || lease==0 || !async_external()) return false;
    ExternalReceiptRequest request{sizeof(request),native_abi_version,lease};
    ExternalReceipt reply{};
    if (!control(device_,ioctl_external_receipt,&request,sizeof(request),&reply,sizeof(reply)) ||
        reply.size!=sizeof(reply) || reply.version!=native_abi_version ||
        reply.status>external_receipt_pending) return false;
    status=reply.status;
    return true;
}

Attempt ClientCanvas::raster(const ClipBatch& clip,uint32_t rop,uint32_t pattern,SURFOBJ* source,
    XLATEOBJ* translation,RECTL original,POINTL* source_point,bool same_canvas,SURFOBJ* staging) noexcept {
    if (!flush_glyphs()) return Attempt::failed;
    if (!active_ || (capabilities_&LIGHTNING_GDI_CAP_ROP3)==0 || rop>UINT8_MAX) return Attempt::unsupported;
    if (policy::empty(clip.bounds)) return Attempt::performed;
    LightningGdiCommand command{};
    command.opcode=LIGHTNING_GDI_ROP3;
    command.destination=wire(clip.bounds);
    command.color=core::rop3_uses_pattern(rop) ? pattern : 0U;
    command.constant_alpha=rop;
    if (core::rop3_uses_source(rop)) {
        if (source_point==nullptr || source==nullptr) return Attempt::unsupported;
        if (same_canvas && (translation==nullptr || (translation->flXlate&XO_TRIVIAL)!=0)) {
            int32_t x=0;
            int32_t y=0;
            if (!source_position({original.left,original.top,original.right,original.bottom},clip.bounds,
                source_point->x,source_point->y,source->sizlBitmap.cx,source->sizlBitmap.cy,x,y)) return Attempt::unsupported;
            command.flags=LIGHTNING_GDI_ROP_SOURCE_SCREEN;
            command.source_x=static_cast<uint32_t>(x);
            command.source_y=static_cast<uint32_t>(y);
        } else {
            uint32_t row_bytes=0;
            if (!core::packed_transfer_size(command.destination.width,command.destination.height,
                row_bytes,command.pixel_bytes) || !copy_source(clip,source,translation,original,*source_point,staging))
                return Attempt::unsupported;
            command.flags=LIGHTNING_GDI_ROP_SOURCE_UPLOAD;
        }
    }
    const auto result=submit(command,clip.rectangles,clip.count);
    if (result==Attempt::performed) {
        visibility_.gpu_write(clip.bounds);
        increment(statistics_.accepted_rasters);
    }
    return result;
}

Attempt ClientCanvas::raster_operands(const ClipBatch& clip,const RasterInput& input,SURFOBJ* staging) noexcept {
    if (!flush_glyphs()) return Attempt::failed;
    if (!active_ || (capabilities_&LIGHTNING_GDI_CAP_RASTER)==0) return Attempt::unsupported;
    if (policy::empty(clip.bounds)) return Attempt::performed;
    if (staging==nullptr || staging->pvScan0!=staging_base_ || staging->iType!=STYPE_BITMAP ||
        staging->iBitmapFormat!=BMF_32BPP || staging_bytes_==0) return Attempt::failed;
    const uint32_t capacity=staging_bytes_<LIGHTNING_GDI_MAX_PIXEL_BYTES ? staging_bytes_ : LIGHTNING_GDI_MAX_PIXEL_BYTES;
    LightningGdiRasterOperands operands{};
    operands.size=sizeof(operands);
    operands.version=LIGHTNING_GDI_RASTER_VERSION;
    operands.foreground_rop=input.rop4&UINT8_MAX;
    operands.background_rop=(input.rop4>>8)&UINT8_MAX;
    // DIB_XXBPP_StretchBlt evaluates only the ROP4 foreground under its mask;
    // a skipped pixel keeps D, which the shared contract expresses as DSTCOPY.
    constexpr uint32_t destination_copy=0xaaU;
    const bool stretch_mask=input.stretch && input.mask!=nullptr;
    if (stretch_mask) operands.background_rop=destination_copy;
    operands.raster_flags=input.raster_flags;
    operands.color_key=input.color_key;
    operands.constant_alpha=input.constant_alpha;
    if (input.rop4>UINT16_MAX) return Attempt::unsupported;
    const uint32_t mirrors=input.raster_flags&(LIGHTNING_GDI_RASTER_MIRROR_X|LIGHTNING_GDI_RASTER_MIRROR_Y);
    const bool keyed=(input.raster_flags&LIGHTNING_GDI_RASTER_COLOR_KEY)!=0;
    const bool alpha=(input.raster_flags&LIGHTNING_GDI_RASTER_ALPHA_BLEND)!=0;
    const bool guarded=mirrors!=0 && !keyed && !alpha &&
        (capabilities_&LIGHTNING_GDI_CAP_GUARDED_SOURCE)!=0;
    if (guarded) operands.raster_flags|=LIGHTNING_GDI_RASTER_GUARDED_SOURCE;
    if ((mirrors!=0 && (capabilities_&LIGHTNING_GDI_CAP_MIRROR)==0) ||
        (keyed && (capabilities_&LIGHTNING_GDI_CAP_TRANSPARENT)==0) ||
        (alpha && (capabilities_&LIGHTNING_GDI_CAP_STRETCH_ALPHA)==0)) return Attempt::unsupported;
    const bool source_used=core::rop3_uses_source(operands.foreground_rop) || core::rop3_uses_source(operands.background_rop);
    const bool pattern_used=core::rop3_uses_pattern(operands.foreground_rop) || core::rop3_uses_pattern(operands.background_rop);
    const uint32_t width=static_cast<uint32_t>(clip.bounds.right-clip.bounds.left);
    const uint32_t height=static_cast<uint32_t>(clip.bounds.bottom-clip.bounds.top);
    if (input.stretch) {
        const int64_t original_width=static_cast<int64_t>(input.original.right)-input.original.left;
        const int64_t original_height=static_cast<int64_t>(input.original.bottom)-input.original.top;
        if ((capabilities_&LIGHTNING_GDI_CAP_STRETCH)==0 ||
            (stretch_mask && ((capabilities_&(LIGHTNING_GDI_CAP_STRETCH_MASK|LIGHTNING_GDI_CAP_MONO_MASK))!=
                (LIGHTNING_GDI_CAP_STRETCH_MASK|LIGHTNING_GDI_CAP_MONO_MASK) || keyed || alpha || input.coverage)) ||
            (!stretch_mask && operands.foreground_rop!=operands.background_rop) || !source_used ||
            original_width<=0 || original_height<=0 || original_width>LIGHTNING_GDI_MAX_DIMENSION ||
            original_height>LIGHTNING_GDI_MAX_DIMENSION) return Attempt::unsupported;
        operands.stretch_mode=LIGHTNING_GDI_STRETCH_COLORONCOLOR;
        operands.original_destination={input.original.left,input.original.top,
            static_cast<uint32_t>(original_width),static_cast<uint32_t>(original_height)};
    }
    uint32_t offset=0;
    POINTL source_origin{};
    uint32_t source_width=width;
    uint32_t source_height=height;
    EngineBitmap normalized_source;
    EngineBitmap normalized_mask;
    EngineBitmap raw_source;
    if (source_used) {
        int32_t x=0;
        int32_t y=0;
        if (input.source==nullptr) return Attempt::unsupported;
        if (input.stretch) {
            const RECTL source=input.source_rectangle;
            const int64_t extent_x=static_cast<int64_t>(source.right)-source.left;
            const int64_t extent_y=static_cast<int64_t>(source.bottom)-source.top;
            const int64_t left=static_cast<int64_t>(source.left)+((mirrors&LIGHTNING_GDI_RASTER_MIRROR_X)!=0 ? 1 : 0);
            const int64_t top=static_cast<int64_t>(source.top)+((mirrors&LIGHTNING_GDI_RASTER_MIRROR_Y)!=0 ? 1 : 0);
            if (left<0 || top<0 || left+extent_x>input.source->sizlBitmap.cx ||
                top+extent_y>input.source->sizlBitmap.cy || extent_x<=0 || extent_y<=0 ||
                extent_x>LIGHTNING_GDI_MAX_DIMENSION || extent_y>LIGHTNING_GDI_MAX_DIMENSION) return Attempt::unsupported;
            x=static_cast<int32_t>(left);
            y=static_cast<int32_t>(top);
            source_width=static_cast<uint32_t>(extent_x);
            source_height=static_cast<uint32_t>(extent_y);
        } else if (input.source_point==nullptr ||
            !source_position({input.original.left,input.original.top,input.original.right,input.original.bottom},clip.bounds,
                input.source_point->x,input.source_point->y,input.source->sizlBitmap.cx,input.source->sizlBitmap.cy,x,y))
                return Attempt::unsupported;
        source_origin={x,y};
        if (input.same_canvas && (input.translation==nullptr || (input.translation->flXlate&XO_TRIVIAL)!=0)) {
            operands.source_kind=LIGHTNING_GDI_ROP_SOURCE_SCREEN;
            operands.source_rect={static_cast<uint32_t>(x),static_cast<uint32_t>(y),source_width,source_height};
        } else {
            uint32_t prefix_x=0,prefix_y=0;
            if (guarded) {
                prefix_x=(mirrors&LIGHTNING_GDI_RASTER_MIRROR_X)!=0 ? 1U : 0U;
                prefix_y=(mirrors&LIGHTNING_GDI_RASTER_MIRROR_Y)!=0 ? 1U : 0U;
                const uint32_t missing_x=prefix_x!=0 && x==0 ? 1U : 0U;
                const uint32_t missing_y=prefix_y!=0 && y==0 ? 1U : 0U;
                if (!normalized_source.create(static_cast<LONG>(source_width+prefix_x),
                    static_cast<LONG>(source_height+prefix_y),BMF_32BPP,capacity)) return Attempt::unsupported;
                // The whole outgoing span is initialized. Missing edge texels
                // are explicitly invalid in the wire view and never sampled.
                memset(normalized_source.surface()->pvScan0,0,normalized_source.bytes());
                RECTL available{static_cast<LONG>(missing_x),static_cast<LONG>(missing_y),
                    static_cast<LONG>(source_width+prefix_x),static_cast<LONG>(source_height+prefix_y)};
                POINTL origin{x-static_cast<LONG>(prefix_x-missing_x),y-static_cast<LONG>(prefix_y-missing_y)};
                if (!EngCopyBits(normalized_source.surface(),input.source,nullptr,input.translation,&available,&origin))
                    return Attempt::unsupported;
                if (missing_x!=0) operands.raster_flags|=LIGHTNING_GDI_RASTER_INVALID_LEFT;
                if (missing_y!=0) operands.raster_flags|=LIGHTNING_GDI_RASTER_INVALID_TOP;
            } else if (!normalized_source.copy(input.source,static_cast<LONG>(source_width),static_cast<LONG>(source_height),
                BMF_32BPP,input.translation,{x,y},capacity)) return Attempt::unsupported;
            operands.source_kind=LIGHTNING_GDI_ROP_SOURCE_UPLOAD;
            operands.source_rect={prefix_x,prefix_y,source_width,source_height};
            operands.source_offset=offset;
            operands.source_bytes=normalized_source.bytes();
            operands.source_stride=normalized_source.stride();
            if (!normalized_source.append(staging_base_,capacity,offset)) return Attempt::unsupported;
        }
    }
    if (keyed && input.source!=nullptr &&
        (input.source->iBitmapFormat!=BMF_32BPP ||
         (input.translation!=nullptr && (input.translation->flXlate&XO_TRIVIAL)==0))) {
        if ((capabilities_&LIGHTNING_GDI_CAP_MONO_MASK)==0 || input.mask!=nullptr ||
            !raw_source.copy(input.source,static_cast<LONG>(source_width),static_cast<LONG>(source_height),
                BMF_32BPP,nullptr,source_origin,LIGHTNING_GDI_MAX_PIXEL_BYTES)) return Attempt::unsupported;
        operands.mask_kind=LIGHTNING_GDI_MASK_MONO1;
        operands.mask_space=LIGHTNING_GDI_MASK_SPACE_SOURCE;
        operands.mask_offset=offset;
        operands.mask_width=source_width;
        operands.mask_height=source_height;
        operands.color_key=0;
        if (!append_key_mask(raw_source,input.color_key,staging_base_,capacity,offset,
            operands.mask_stride,operands.mask_bytes)) return Attempt::unsupported;
    }
    if (stretch_mask) {
        operands.mask_kind=LIGHTNING_GDI_MASK_MONO1;
        operands.mask_space=LIGHTNING_GDI_MASK_SPACE_DESTINATION;
        operands.mask_offset=offset;
        operands.mask_width=width;
        operands.mask_height=height;
        const auto copied=append_stretch_mask(input.mask,input,clip,staging_base_,capacity,offset,
            operands.mask_stride,operands.mask_bytes);
        if (copied==CoverageCopy::invalid) return Attempt::failed;
        if (copied==CoverageCopy::unavailable) return Attempt::unsupported;
    } else if (input.mask!=nullptr) {
        const uint32_t capability=input.coverage ? LIGHTNING_GDI_CAP_TEXT : LIGHTNING_GDI_CAP_MONO_MASK;
        const ULONG format=input.coverage ? BMF_8BPP : BMF_1BPP;
        int32_t x=0;
        int32_t y=0;
        if ((capabilities_&capability)==0 || input.mask_point==nullptr || input.mask->iBitmapFormat!=format ||
            !source_position({input.original.left,input.original.top,input.original.right,input.original.bottom},clip.bounds,
                input.mask_point->x,input.mask_point->y,input.mask->sizlBitmap.cx,input.mask->sizlBitmap.cy,x,y))
            return Attempt::unsupported;
        operands.mask_kind=input.coverage ? LIGHTNING_GDI_MASK_COVERAGE8 : LIGHTNING_GDI_MASK_MONO1;
        operands.mask_space=LIGHTNING_GDI_MASK_SPACE_DESTINATION;
        operands.mask_offset=offset;
        operands.mask_width=width;
        operands.mask_height=height;
        const auto copied=input.coverage ? append_coverage(input.mask,{x,y},width,height,staging_base_,capacity,
            offset,operands.mask_stride,operands.mask_bytes) : CoverageCopy::unavailable;
        // Malformed source ownership must not authorize an engine fallback
        // that would read the same invalid bytes. No command was submitted.
        if (copied==CoverageCopy::invalid) return Attempt::failed;
        if (copied==CoverageCopy::unavailable) {
            if (!normalized_mask.copy(input.mask,static_cast<LONG>(width),static_cast<LONG>(height),format,nullptr,{x,y},capacity) ||
                !normalized_mask.append(staging_base_,capacity,offset)) return Attempt::unsupported;
            operands.mask_bytes=normalized_mask.bytes();
            operands.mask_stride=normalized_mask.stride();
        }
    } else if (input.coverage) return Attempt::unsupported;
    if (pattern_used) {
        if (input.brush==nullptr) return Attempt::unsupported;
        if (input.brush->iSolidColor!=UINT32_MAX) {
            operands.pattern_kind=LIGHTNING_GDI_PATTERN_SOLID;
            operands.solid_color=input.brush->iSolidColor;
        } else {
            BrushPixels pattern{};
            uint32_t x=0;
            uint32_t y=0;
            if (input.coverage || (capabilities_&LIGHTNING_GDI_CAP_PATTERN_TILE)==0 || input.brush_origin==nullptr ||
                !brush_pixels(input.brush,pattern) || !core::normalize_pattern_origin(input.brush_origin->x,pattern.width,x) ||
                !core::normalize_pattern_origin(input.brush_origin->y,pattern.height,y) ||
                offset>capacity || pattern.bytes>capacity-offset) return Attempt::unsupported;
            operands.pattern_kind=LIGHTNING_GDI_PATTERN_TILE;
            operands.pattern_width=pattern.width;
            operands.pattern_height=pattern.height;
            operands.pattern_origin_x=static_cast<int32_t>(x);
            operands.pattern_origin_y=static_cast<int32_t>(y);
            operands.pattern_offset=offset;
            operands.pattern_bytes=pattern.bytes;
            operands.pattern_stride=pattern.stride;
            memcpy(static_cast<uint8_t*>(staging_base_)+offset,pattern.pixels,pattern.bytes);
            offset+=pattern.bytes;
        }
    }
    const auto destination=wire(clip.bounds);
    if (!core::validate_raster_inputs(width_,height_,destination,operands,offset)) return Attempt::unsupported;
    if (input.stretch) {
        core::StretchMapping mapping{};
        if (clip.count==0) {
            if (!core::raster_clip_mapping(operands,destination,mapping)) return Attempt::unsupported;
        } else {
            for (uint32_t index=0;index<clip.count;++index) {
                if (!core::raster_clip_mapping(operands,clip.rectangles[index],mapping)) return Attempt::unsupported;
            }
        }
    }
    LightningGdiCommand command{};
    command.opcode=LIGHTNING_GDI_RASTER;
    command.destination=destination;
    command.pixel_bytes=offset;
    const auto result=submit(command,clip.rectangles,clip.count,0,0,&operands);
    if (result==Attempt::performed) {
        visibility_.gpu_write(clip.bounds);
        increment(statistics_.accepted_rasters);
    }
    return result;
}

Attempt ClientCanvas::glyph(const ClipBatch& clip,SURFOBJ* mask,RECTL original,POINTL point,
    BRUSHOBJ* brush,SURFOBJ* staging) noexcept {
    if (policy::empty(clip.bounds)) return Attempt::performed;
    if (brush==nullptr || brush->iSolidColor==UINT32_MAX || mask==nullptr ||
        (mask->iBitmapFormat!=BMF_1BPP && mask->iBitmapFormat!=BMF_8BPP)) return Attempt::unsupported;
    RasterInput input{};
    input.original=original;
    input.mask=mask;
    input.mask_point=&point;
    input.coverage=mask->iBitmapFormat==BMF_8BPP;
    input.brush=brush;
    input.rop4=0xaaf0U;
    const auto result=raster_operands(clip,input,staging);
    if (result==Attempt::performed) increment(statistics_.accepted_glyphs);
    return result;
}

Attempt ClientCanvas::queue_glyph(const ClipBatch& clip,SURFOBJ* mask,RECTL original,POINTL point,
    BRUSHOBJ* brush,SURFOBJ* staging,SURFOBJ* cached) noexcept {
    if (cached==nullptr || staging==nullptr || staging->pvScan0!=staging_base_) return Attempt::failed;
    if (mask!=nullptr && mask->pvBits!=nullptr && staging_base_!=nullptr) {
        const uintptr_t source=reinterpret_cast<uintptr_t>(mask->pvBits);
        const uintptr_t target=reinterpret_cast<uintptr_t>(staging_base_);
        if (mask->cjBits>UINTPTR_MAX-source || staging_bytes_>UINTPTR_MAX-target ||
            (source<target+staging_bytes_ && target<source+mask->cjBits)) return Attempt::failed;
    }
    const uint32_t needed=LIGHTNING_GDI_CAP_GROUPS|LIGHTNING_GDI_CAP_RASTER|LIGHTNING_GDI_CAP_TEXT;
    if (!active_ || !text_scope_ || group_sequence_==0 || (capabilities_&needed)!=needed ||
        brush==nullptr || brush->iSolidColor==UINT32_MAX || mask==nullptr ||
        mask->iBitmapFormat!=BMF_8BPP || mask->lDelta<=0 || mask->pvBits==nullptr) {
        if (!prepare(cached,staging) || !flush_glyphs()) return Attempt::failed;
        return glyph(clip,mask,original,point,brush,staging);
    }
    if (visibility_.poisoned()) return Attempt::failed;
    if (policy::empty(clip.bounds)) return Attempt::performed;
    const Rectangle bounded=policy::clip(clip.bounds,width_,height_);
    if (bounded.left!=clip.bounds.left || bounded.top!=clip.bounds.top || bounded.right!=clip.bounds.right ||
        bounded.bottom!=clip.bounds.bottom || clip.count>LIGHTNING_GDI_MAX_RECTS) return Attempt::failed;
    const uint32_t count=clip.count==0 ? 1U : clip.count;
    bool conflict=glyphs_.glyph_count!=0 && (glyphs_.color!=brush->iSolidColor ||
        count>LIGHTNING_GDI_MAX_RECTS-glyphs_.rectangle_count);
    for (uint32_t index=0;index<count;++index) {
        const auto rect=clip.count==0 ? wire(clip.bounds) : clip.rectangles[index];
        const uint64_t right=static_cast<uint64_t>(rect.x)+rect.width;
        const uint64_t bottom=static_cast<uint64_t>(rect.y)+rect.height;
        if (rect.width==0 || rect.height==0 || rect.x<static_cast<uint32_t>(bounded.left) ||
            rect.y<static_cast<uint32_t>(bounded.top) || right>static_cast<uint32_t>(bounded.right) ||
            bottom>static_cast<uint32_t>(bounded.bottom)) return Attempt::failed;
        const Rectangle area{static_cast<int32_t>(rect.x),static_cast<int32_t>(rect.y),
            static_cast<int32_t>(right),static_cast<int32_t>(bottom)};
        for (uint32_t previous=0;previous<glyphs_.rectangle_count;++previous)
            if (!policy::empty(policy::intersect(area,glyphs_.rectangles[previous]))) conflict=true;
        for (uint32_t previous=0;previous<index;++previous) {
            const auto other=clip.rectangles[previous];
            const Rectangle other_area{static_cast<int32_t>(other.x),static_cast<int32_t>(other.y),
                static_cast<int32_t>(other.x+other.width),static_cast<int32_t>(other.y+other.height)};
            if (!policy::empty(policy::intersect(area,other_area))) return Attempt::failed;
        }
    }
    if (conflict && (!prepare(cached,staging) || !flush_glyphs())) return Attempt::failed;
    int32_t source_x=0;
    int32_t source_y=0;
    if (!source_position({original.left,original.top,original.right,original.bottom},clip.bounds,
        point.x,point.y,mask->sizlBitmap.cx,mask->sizlBitmap.cy,source_x,source_y)) return Attempt::failed;
    if (glyphs_.pixels==nullptr) {
        const uint64_t stride=(static_cast<uint64_t>(width_)+3U)&~3ULL;
        const uint64_t bytes=stride*height_;
        if (bytes==0 || bytes>LIGHTNING_GDI_MAX_PIXEL_BYTES) {
            if (!prepare(cached,staging)) return Attempt::failed;
            return glyph(clip,mask,original,point,brush,staging);
        }
        glyphs_.pixels=static_cast<uint8_t*>(EngAllocMem(0,static_cast<ULONG>(bytes),ALLOC_TAG));
        if (glyphs_.pixels==nullptr) {
            if (!prepare(cached,staging)) return Attempt::failed;
            return glyph(clip,mask,original,point,brush,staging);
        }
        glyphs_.stride=static_cast<uint32_t>(stride);
        glyphs_.bytes=static_cast<uint32_t>(bytes);
    }
    const Rectangle combined=policy::unite(glyphs_.bounds,clip.bounds);
    const uint64_t packed_stride=(static_cast<uint64_t>(combined.right-combined.left)+3U)&~3ULL;
    if (packed_stride*static_cast<uint32_t>(combined.bottom-combined.top)>staging_bytes_) {
        if (!prepare(cached,staging) || !flush_glyphs()) return Attempt::failed;
        return glyph(clip,mask,original,point,brush,staging);
    }
    uint32_t offset=0;
    uint32_t pitch=0;
    uint32_t bytes=0;
    const auto copied=append_coverage(mask,{source_x,source_y},bounded.right-bounded.left,bounded.bottom-bounded.top,
        staging_base_,staging_bytes_,offset,pitch,bytes);
    if (copied==CoverageCopy::invalid) return Attempt::failed;
    if (copied==CoverageCopy::unavailable) {
        if (!prepare(cached,staging)) return Attempt::failed;
        return glyph(clip,mask,original,point,brush,staging);
    }
    // Clear only newly exposed union bands. Old coverage remains immutable;
    // holes are exactly zero and therefore preserve every destination byte.
    for (int32_t y=combined.top;y<combined.bottom;++y) {
        auto* row=glyphs_.pixels+static_cast<size_t>(y)*glyphs_.stride;
        if (y<glyphs_.bounds.top || y>=glyphs_.bounds.bottom || policy::empty(glyphs_.bounds)) {
            memset(row+combined.left,0,static_cast<size_t>(combined.right-combined.left));
        } else {
            if (combined.left<glyphs_.bounds.left)
                memset(row+combined.left,0,static_cast<size_t>(glyphs_.bounds.left-combined.left));
            if (combined.right>glyphs_.bounds.right)
                memset(row+glyphs_.bounds.right,0,static_cast<size_t>(combined.right-glyphs_.bounds.right));
        }
    }
    for (uint32_t index=0;index<count;++index) {
        const auto rect=clip.count==0 ? wire(clip.bounds) : clip.rectangles[index];
        for (uint32_t y=0;y<rect.height;++y) {
            const auto* source=static_cast<const uint8_t*>(staging_base_)+
                static_cast<size_t>(rect.y-bounded.top+y)*pitch+(rect.x-bounded.left);
            auto* destination=glyphs_.pixels+static_cast<size_t>(rect.y+y)*glyphs_.stride+rect.x;
            memcpy(destination,source,rect.width);
        }
        glyphs_.rectangles[glyphs_.rectangle_count++]={static_cast<int32_t>(rect.x),static_cast<int32_t>(rect.y),
            static_cast<int32_t>(rect.x+rect.width),static_cast<int32_t>(rect.y+rect.height)};
    }
    glyphs_.bounds=combined;
    glyphs_.color=brush->iSolidColor;
    ++glyphs_.glyph_count;
    return Attempt::performed;
}

bool ClientCanvas::flush_glyphs() noexcept {
    if (glyphs_.glyph_count==0) return !visibility_.poisoned();
    if (!active_ || visibility_.poisoned() || owner_offline()) return false;
    const auto destination=wire(glyphs_.bounds);
    SURFOBJ mask{};
    mask.iType=STYPE_BITMAP;
    mask.iBitmapFormat=BMF_8BPP;
    mask.sizlBitmap={static_cast<LONG>(width_),static_cast<LONG>(height_)};
    mask.lDelta=static_cast<LONG>(glyphs_.stride);
    mask.pvScan0=mask.pvBits=glyphs_.pixels;
    mask.cjBits=glyphs_.bytes;
    LightningGdiRasterOperands operands{};
    operands.size=sizeof(operands);
    operands.version=LIGHTNING_GDI_RASTER_VERSION;
    operands.pattern_kind=LIGHTNING_GDI_PATTERN_SOLID;
    operands.solid_color=glyphs_.color;
    operands.foreground_rop=0xf0U;
    operands.background_rop=0xaaU;
    operands.mask_kind=LIGHTNING_GDI_MASK_COVERAGE8;
    operands.mask_space=LIGHTNING_GDI_MASK_SPACE_DESTINATION;
    operands.mask_width=destination.width;
    operands.mask_height=destination.height;
    uint32_t bytes=0;
    if (append_coverage(&mask,{glyphs_.bounds.left,glyphs_.bounds.top},destination.width,destination.height,
        staging_base_,staging_bytes_,bytes,operands.mask_stride,operands.mask_bytes)!=CoverageCopy::copied ||
        !core::validate_raster_inputs(width_,height_,destination,operands,bytes)) {
        visibility_.poison();
        return false;
    }
    LightningGdiCommand command{};
    command.opcode=LIGHTNING_GDI_RASTER;
    command.destination=destination;
    command.pixel_bytes=bytes;
    if (submit(command,nullptr,0,0,0,&operands)!=Attempt::performed) {
        // The caller already handled these glyphs. A rejected aggregate cannot
        // authorize engine replay or publication of a prefix missing its bytes.
        visibility_.poison();
        return false;
    }
    visibility_.gpu_write(glyphs_.bounds);
    increment(statistics_.accepted_rasters);
    const uint64_t count=glyphs_.glyph_count;
    statistics_.accepted_glyphs=count>UINT64_MAX-statistics_.accepted_glyphs ? UINT64_MAX : statistics_.accepted_glyphs+count;
    glyphs_.bounds={};
    glyphs_.rectangle_count=0;
    glyphs_.glyph_count=0;
    return true;
}

void ClientCanvas::release_cpu_storage() noexcept {
    if (glyphs_.pixels!=nullptr) EngFreeMem(glyphs_.pixels);
    glyphs_={};
}

Attempt ClientCanvas::bitmap(const ClipBatch& clip,SURFOBJ* source,XLATEOBJ* translation,
    RECTL original,POINTL point,SURFOBJ* staging,SURFOBJ* cached,CLIPOBJ* native_clip) noexcept {
    if (!flush_glyphs()) return Attempt::failed;
    if (!active_) return Attempt::unsupported;
    if (policy::empty(clip.bounds)) return Attempt::performed;
    uint32_t row_bytes=0;
    uint32_t bytes=0;
    if (clip.count!=0 && !core::packed_transfer_size(clip.bounds.right-clip.bounds.left,
        clip.bounds.bottom-clip.bounds.top,row_bytes,bytes)) return Attempt::unsupported;
    if (!copy_source(clip,source,translation,original,point,staging)) return Attempt::unsupported;
    if (!upload(clip.bounds,clip.rectangles,clip.count,true)) return visibility_.poisoned() ? Attempt::failed : Attempt::unsupported;
    // Upload bytes are already known CPU pixels. Keep their cache mirror using
    // the existing engine copy rather than forcing readback for the next glyph.
    // Prior stale bounds remain conservative. A failed mirror after acceptance
    // only makes this region stale; it must never cause GPU replay/fallback.
    RECTL destination{clip.bounds.left,clip.bounds.top,clip.bounds.right,clip.bounds.bottom};
    POINTL origin{0,0};
    if (cached==nullptr || !EngCopyBits(cached,staging,native_clip,nullptr,&destination,&origin))
        visibility_.gpu_write(clip.bounds);
    return Attempt::performed;
}

Attempt ClientCanvas::blend(const ClipBatch& clip,SURFOBJ* source,XLATEOBJ* translation,
    RECTL original,RECTL source_rectangle,BLENDOBJ* blend,SURFOBJ* staging,bool same_canvas) noexcept {
    if (!flush_glyphs()) return Attempt::failed;
    if (!active_ || source==nullptr || blend==nullptr) return Attempt::unsupported;
    if (policy::empty(clip.bounds)) return Attempt::performed;
    const auto function=blend->BlendFunction;
    if (function.BlendOp!=AC_SRC_OVER || function.BlendFlags!=0 || (function.AlphaFormat&~AC_SRC_ALPHA)!=0 ||
        (function.AlphaFormat==AC_SRC_ALPHA && source->iBitmapFormat!=BMF_32BPP)) return Attempt::unsupported;
    if (same_canvas && original.left<source_rectangle.right && source_rectangle.left<original.right &&
        original.top<source_rectangle.bottom && source_rectangle.top<original.bottom) return Attempt::unsupported;
    const bool direct_source=same_canvas && (translation==nullptr || (translation->flXlate&XO_TRIVIAL)!=0);
    if (direct_source ||
        static_cast<int64_t>(original.right)-original.left!=static_cast<int64_t>(source_rectangle.right)-source_rectangle.left ||
        static_cast<int64_t>(original.bottom)-original.top!=static_cast<int64_t>(source_rectangle.bottom)-source_rectangle.top) {
        RasterInput input{};
        input.source=source;
        input.translation=translation;
        input.original=original;
        input.same_canvas=same_canvas;
        input.stretch=true;
        input.source_rectangle=source_rectangle;
        input.rop4=0xccccU;
        input.raster_flags=LIGHTNING_GDI_RASTER_ALPHA_BLEND |
            (function.AlphaFormat==AC_SRC_ALPHA ? LIGHTNING_GDI_RASTER_SOURCE_ALPHA : 0U) |
            (source->iBitmapFormat==BMF_32BPP ? LIGHTNING_GDI_RASTER_SOURCE_32BPP : 0U);
        input.constant_alpha=function.SourceConstantAlpha;
        const auto result=raster_operands(clip,input,staging);
        if (result==Attempt::performed) increment(statistics_.accepted_blends);
        return result;
    }
    uint32_t row_bytes=0;
    uint32_t bytes=0;
    if (!core::packed_transfer_size(clip.bounds.right-clip.bounds.left,clip.bounds.bottom-clip.bounds.top,row_bytes,bytes)) return Attempt::unsupported;
    const POINTL point{source_rectangle.left,source_rectangle.top};
    if (!copy_source(clip,source,translation,original,point,staging)) return Attempt::unsupported;
    LightningGdiCommand command{};
    command.opcode=LIGHTNING_GDI_BLEND_UPLOAD;
    command.destination=wire(clip.bounds);
    command.pixel_bytes=bytes;
    command.flags=(function.AlphaFormat==AC_SRC_ALPHA ? LIGHTNING_GDI_SOURCE_ALPHA : 0U) |
        (source->iBitmapFormat==BMF_32BPP ? LIGHTNING_GDI_SOURCE_32BPP : 0U);
    command.constant_alpha=function.SourceConstantAlpha;
    const auto result=submit(command,clip.rectangles,clip.count);
    if (result==Attempt::performed) {visibility_.gpu_write(clip.bounds); increment(statistics_.accepted_blends);}
    return result;
}

Attempt ClientCanvas::transparent(const ClipBatch& clip,SURFOBJ* source,XLATEOBJ* translation,
    RECTL original,RECTL source_rectangle,uint32_t key,bool same_canvas,SURFOBJ* staging) noexcept {
    RasterInput input{};
    input.source=source;
    input.translation=translation;
    input.original=original;
    input.same_canvas=same_canvas;
    input.stretch=true;
    input.source_rectangle=source_rectangle;
    input.rop4=0xccccU;
    input.raster_flags=LIGHTNING_GDI_RASTER_COLOR_KEY;
    input.color_key=key&0x00ffffffU;
    return raster_operands(clip,input,staging);
}

bool ClientCanvas::synchronize(SURFOBJ* cached,SURFOBJ* staging) noexcept {
    if (!prepare(cached,staging)) return false;
    if (!flush_glyphs()) return false;
    if (visibility_.poisoned()) return false;
    if (visibility_.readable()) return true;
    const Rectangle stale=visibility_.stale();
    core::StripeCursor cursor;
    if (!cursor.begin(wire(stale),width_,height_)) return false;
    while (!cursor.done()) {
        LightningDisplayRect stripe{};
        uint32_t row_bytes=0;
        uint32_t bytes=0;
        if (!cursor.current(stripe) || !core::packed_transfer_size(stripe.width,stripe.height,row_bytes,bytes)) break;
        LightningGdiCommand command{};
        command.opcode=LIGHTNING_GDI_READBACK;
        command.destination=stripe;
        command.pixel_bytes=bytes;
        // A declined read-only stripe did not change GPU pixels or publish a
        // partial CPU gather. Keep the stale cache and retry after readiness;
        // only uncertain receipts poison ownership in submit itself.
        if (submit(command,nullptr,0)!=Attempt::performed) return false;
        if (!cursor.advance(stripe)) break;
    }
    if (!cursor.done()) {visibility_.poison(); return false;}
    // Native completed every stripe into retained staging RAM. Publish to the
    // cached bitmap only now, so a partial gather never authorizes CPU fallback.
    RECTL rectangle{stale.left,stale.top,stale.right,stale.bottom};
    POINTL point{stale.left,stale.top};
    if (!EngCopyBits(cached,staging,nullptr,nullptr,&rectangle,&point)) {visibility_.poison(); return false;}
    visibility_.readback_complete();
    increment(statistics_.readbacks);
    return true;
}

bool ClientCanvas::complete_gpu(SURFOBJ* cached,SURFOBJ* staging) noexcept {
    if (!prepare(cached,staging) || !flush_glyphs()) return false;
    // Waiting never completes cache readback or ends a live logical text group.
    return !active_ || wait();
}

Attempt ClientCanvas::gradient(ClipBatch& workspace,CLIPOBJ* clip,XLATEOBJ* translation,
    const TRIVERTEX* vertices,uint32_t vertex_count,const void* mesh,uint32_t mesh_count,
    RECTL extents,POINTL origin,uint32_t mode) noexcept {
    const uint32_t needed=LIGHTNING_GDI_CAP_GRADIENT|LIGHTNING_GDI_CAP_GROUPS;
    if (!active_ || (capabilities_&needed)!=needed || mode>GRADIENT_FILL_TRIANGLE) return Attempt::unsupported;
    if (mesh_count==0) return Attempt::performed;
    const uint32_t mesh_bytes=mode==GRADIENT_FILL_TRIANGLE ? sizeof(GRADIENT_TRIANGLE) : sizeof(GRADIENT_RECT);
    if (vertices==nullptr || mesh==nullptr || vertex_count==0 || vertex_count>UINT32_MAX/sizeof(TRIVERTEX) ||
        mesh_count>UINT32_MAX/mesh_bytes ||
        static_cast<uint64_t>(vertex_count)*sizeof(TRIVERTEX)>UINTPTR_MAX-reinterpret_cast<uintptr_t>(vertices) ||
        static_cast<uint64_t>(mesh_count)*mesh_bytes>UINTPTR_MAX-reinterpret_cast<uintptr_t>(mesh)) return Attempt::unsupported;
    bool swap_red_blue=false;
    if (!gradient_translation(translation,swap_red_blue)) return Attempt::unsupported;
    bool visible=false;
    LightningGdiGradientOperands operands{};
    RECTL requested{};
    for (uint32_t index=0;index<mesh_count;++index) {
        if (!gradient_element(vertices,vertex_count,mesh,index,mode,extents,origin,swap_red_blue,operands,requested))
            return Attempt::unsupported;
        if (policy::empty({requested.left,requested.top,requested.right,requested.bottom})) continue;
        if (!capture_clip(requested,clip,width_,height_,workspace)) return Attempt::unsupported;
        if (policy::empty(workspace.bounds)) continue;
        if (!core::validate_gradient_inputs(width_,height_,wire(workspace.bounds),operands)) return Attempt::unsupported;
        visible=true;
    }
    if (!visible) return Attempt::performed;
    if (!flush_glyphs()) return Attempt::failed;
    const bool own_group=!text_scope_;
    if (own_group && !begin_text()) return Attempt::failed;
    for (uint32_t index=0;index<mesh_count;++index) {
        if (!gradient_element(vertices,vertex_count,mesh,index,mode,extents,origin,swap_red_blue,operands,requested)) {
            abort_text(); return Attempt::failed;
        }
        if (policy::empty({requested.left,requested.top,requested.right,requested.bottom})) continue;
        if (!capture_clip(requested,clip,width_,height_,workspace)) {abort_text(); return Attempt::failed;}
        if (policy::empty(workspace.bounds)) continue;
        if (!core::validate_gradient_inputs(width_,height_,wire(workspace.bounds),operands)) {
            abort_text(); return Attempt::failed;
        }
        LightningGdiCommand command{};
        command.opcode=LIGHTNING_GDI_GRADIENT;
        command.destination=wire(workspace.bounds);
        if (submit(command,workspace.rectangles,workspace.count,0,0,nullptr,&operands)!=Attempt::performed) {
            abort_text(); return Attempt::failed;
        }
        visibility_.gpu_write(workspace.bounds);
        increment(statistics_.accepted_rasters);
    }
    if (own_group && !end_text()) {abort_text(); return Attempt::failed;}
    return Attempt::performed;
}

bool ClientCanvas::prepare(SURFOBJ* cached,SURFOBJ* staging) noexcept {
    if (visibility_.poisoned()) return false;
    if (!active_) {
        if (!promotable_) return true;
        LONGLONG now=0;
        EngQueryPerformanceCounter(&now);
        if (!promotion_.due(now>=0 ? static_cast<uint64_t>(now) : 0)) return true;
        Information candidate{};
        if (!control(device_,ioctl_query,nullptr,0,&candidate,sizeof(candidate))) return true;
        if (!valid_information(candidate) || candidate.mode_generation!=generation_ || candidate.quarantined!=0) {
            visibility_.poison();
            return false;
        }
        if ((candidate.capabilities&LIGHTNING_GDI_BASE_CAPABILITIES)!=LIGHTNING_GDI_BASE_CAPABILITIES) return true;
        // All cached pixels remain authoritative while the CPU backend owns
        // presentation. Drain its real prefix before giving a GPU canvas the
        // console; then establish a full immutable baseline before any draw.
        if (!control(device_,::lightning::reactos::ioctl_wait_cpu_scanout,nullptr,0,nullptr,0)) return false;
        RECTL full{0,0,static_cast<LONG>(width_),static_cast<LONG>(height_)};
        POINTL origin{0,0};
        if (cached==nullptr || staging==nullptr || !EngCopyBits(staging,cached,nullptr,nullptr,&full,&origin)) return false;
        const Attempt result=bind(candidate);
        if (result==Attempt::unsupported) return true;
        if (result==Attempt::failed ||
            !upload({0,0,static_cast<int32_t>(width_),static_cast<int32_t>(height_)},nullptr,0,false)) {
            visibility_.poison(); return false;
        }
        return true;
    }
    Information information{};
    QueryRequest query{sizeof(query),native_abi_version,query_coherence_only,0};
    if (!control(device_,ioctl_query,&query,sizeof(query),&information,sizeof(information))) {
        // The native power gate rejects hardware requests in D3, but its
        // cached owner inventory can distinguish this from unknown execution.
        // No command was submitted; retain stale/cache/dirty ownership for D0.
        if (owner_offline()) return false;
        visibility_.poison(); return false;
    }
    if (!valid_information(information) || information.quarantined!=0 || information.mode_generation!=generation_) {
        visibility_.poison(); return false;
    }
    if (information.reset_generation==reset_generation_) {
        if (information.epoch!=last_epoch_ || information.accepted_sequence!=last_sequence_ ||
            information.canvas_revision!=last_revision_) {
            visibility_.poison(); return false;
        }
        capabilities_=information.capabilities;
        return ensure_group();
    }
    if (information.reset_generation==0 || information.reset_generation<reset_generation_ ||
        information.accepted_sequence!=0 || information.active!=0) {
        visibility_.poison(); return false;
    }
    // Actual retirement invalidates old logical-group sequence identities.
    // A live text scope may start a fresh group only after its new baseline.
    group_sequence_=0;
    // A filter-driven power transition can happen without DrvAssertMode. Its
    // native owner retained the complete fenced primary before GPU retirement.
    // Restore that snapshot before authorizing any CPU read or new GPU draw.
    RECTL full{0,0,static_cast<LONG>(width_),static_cast<LONG>(height_)};
    POINTL origin{0,0};
    if (cached==nullptr || staging==nullptr) {visibility_.poison(); return false;}
    if (!visibility_.readable()) {
        RestoreRequest restore{sizeof(restore),native_abi_version,generation_,0};
        if (!control(device_,ioctl_restore,&restore,sizeof(restore),nullptr,0) ||
            !EngCopyBits(cached,staging,nullptr,nullptr,&full,&origin)) {
            visibility_.poison(); return false;
        }
    }
    // A coherent cache can contain known CPU fallback writes that raced the
    // power gate. Preserve those owned pixels instead of replacing them with
    // the earlier native snapshot; rebuild staging from the complete cache.
    if (!EngCopyBits(staging,cached,nullptr,nullptr,&full,&origin)) {
        visibility_.poison(); return false;
    }
    visibility_.reinitialize();
    if (bind(information)!=Attempt::performed ||
        !upload({0,0,static_cast<int32_t>(width_),static_cast<int32_t>(height_)},nullptr,0,false)) {
        visibility_.poison(); return false;
    }
    return ensure_group();
}

void ClientCanvas::fallback() noexcept {increment(statistics_.cpu_fallbacks);}

bool ClientCanvas::statistics(Statistics& output) noexcept {
    statistics_.size=sizeof(Statistics);
    statistics_.version=statistics_abi_version;
    statistics_.enabled=active_ ? 1U : 0U;
    statistics_.poisoned=visibility_.poisoned() ? 1U : 0U;
    if (active_ && !control(device_,ioctl_query,nullptr,0,&statistics_.device,sizeof(statistics_.device))) return false;
    output=statistics_;
    return true;
}

void ClientCanvas::demote_to_cpu() noexcept {
    release_cpu_storage();
    visibility_.reinitialize();
    active_=false;
    promotable_=false;
    text_scope_=false;
    group_sequence_=0;
    external_pending_=false;
    increment(statistics_.cpu_fallbacks);
}

bool ClientCanvas::retire() noexcept {
    promotable_=false;
    if (text_scope_ || group_sequence_!=0 || glyphs_.glyph_count!=0) return false;
    if (active_ && (visibility_.poisoned() || !wait())) return false;
    active_=false;
    return true;
}

} // namespace lightning::reactos::gdi

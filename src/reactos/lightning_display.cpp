/*
Module: lightning.reactos.display
Purpose: Submit actual dirty regions and exactly representable native cursors.
Created: 2026-10-21
Architecture: Reuses framebuf cached GDI surfaces and the canonical host ABI.
License: GPL-2.0-or-later, matching the reused ReactOS framebuf driver.
*/
#include "framebuf.h"
#include "lightning_display.hpp"
#include "lightning-memory.h"
#include "lightning-display.h"
#include "lightning_cursor_policy.hpp"
#include "engine_bitmap.hpp"
#include "../../third_party/qemu_lightning/cursor_policy.hpp"

namespace lightning::reactos {
namespace {

// Pending retirements are drained at most this many times per MAP, removal
// transfer or teardown. With an authoritative owner a lost acknowledgement
// becomes a final answer on the next attempt; a dead channel stays bounded.
constexpr uint32_t retire_attempts=3;

bool control(HANDLE device, uint32_t code, void* input = nullptr,
             uint32_t input_bytes = 0, void* output = nullptr,
             uint32_t output_bytes = 0) noexcept {
    DWORD returned = 0;
    return EngDeviceIoControl(device, code, input, input_bytes, output,
        output_bytes, &returned) == 0 && returned == output_bytes;
}

// As control, keeping the engine's Win32 error (or ERROR_INVALID_DATA for a
// short reply) for the mode-health record.
bool control_status(HANDLE device, uint32_t code, void* input, uint32_t input_bytes,
                    void* output, uint32_t output_bytes, uint32_t& error) noexcept {
    DWORD returned = 0;
    const DWORD status = EngDeviceIoControl(device, code, input, input_bytes, output,
        output_bytes, &returned);
    error = status != 0 ? status : (returned != output_bytes ? 13U : 0U);
    return error == 0;
}

bool mask_pixel(SURFOBJ& mask, uint32_t x, uint32_t y) noexcept {
    const auto* row = static_cast<const uint8_t*>(mask.pvScan0) +
        static_cast<size_t>(mask.lDelta) * y;
    return (row[x / 8U] & (0x80U >> (x % 8U))) != 0;
}

} // namespace

bool DisplayLink::query(CpuScanoutInformation& information) noexcept {
    return query_display_owner(driver_,information);
}

bool DisplayLink::connect(PPDEV device,void* candidate_staging) noexcept {
    if (device == nullptr || device->hDriver==nullptr) return false;
    if (driver_!=nullptr && driver_!=device->hDriver) return false;
    void* address=candidate_staging!=nullptr ? candidate_staging : device->ScreenPtr;
    CpuScanoutInformation information{};
    if (!query_owned_staging(device->hDriver,address,information) ||
        information.enabled != 1 ||
        information.width != device->ScreenWidth || information.height != device->ScreenHeight ||
        information.stride != device->ScreenDelta || information.mode_generation == 0 ||
        (mode_generation_!=0 && information.mode_generation!=mode_generation_) ||
        !rx7900::reactos::framebuffer::valid_layout(information.width, information.height,
            information.stride, information.mapped_bytes) ||
        !staging_leases_.remember(address,information.mode_generation)) return false;
    driver_ = device->hDriver;
    mode_generation_ = information.mode_generation;
    mode_index_=device->ModeIndex;
    lease_generation_=mode_generation_;
    recovery_required_=false;
    // Negotiate before the first release can be lost: one read-only query of
    // the authenticated lease. A legacy owner fails it and keeps legacy release.
    if (!resolve_supported_) {
        StagingResolveReceipt receipt{};
        resolve_lease(lease_generation_,staging_resolve_query,receipt);
    }
    return true;
}

bool DisplayLink::current_owner(bool require_enabled) noexcept {
    CpuScanoutInformation information{};
    return mode_generation_!=0 && query(information) &&
        information.mode_generation==mode_generation_ && (!require_enabled || information.enabled==1);
}

bool DisplayLink::transition_mode(uint32_t mode_index,bool enable) noexcept {
    if (driver_==nullptr || mode_generation_==0) return false;
    ModeTransitionRequest request{{sizeof(ModeTransitionRequest),cpu_scanout_abi_version,
        mode_generation_,mode_index,lease_generation_},enable ? 1U : 0U,0};
    ModeTransitionReceipt receipt{};
    if (!control_status(driver_,ioctl_transition_owned_mode,&request,sizeof(request),&receipt,sizeof(receipt),
            last_error_) || receipt.size!=sizeof(receipt) || receipt.version!=cpu_scanout_abi_version ||
        receipt.requested_generation!=mode_generation_ || receipt.mode_generation<mode_generation_ ||
        receipt.enabled>1 || receipt.reserved!=0) return false;
    // Even a failed commit can invalidate the old mode. Acknowledging its new
    // token keeps this owner recoverable without borrowing another PDEV's token.
    mode_generation_=receipt.mode_generation;
    mode_index_=receipt.mode_index;
    if (receipt.result!=0) last_error_=receipt.result;
    return receipt.result==0 && receipt.enabled==(enable ? 1U : 0U) && receipt.mode_index==mode_index;
}

bool DisplayLink::map_staging(void*& address, uint32_t& bytes) noexcept {
    address=nullptr;
    bytes=0;
    if (driver_==nullptr || lost_ || mode_generation_==0 || mode_generation_==UINT32_MAX) return false;
    if (!recover_unacknowledged_map()) return false;
    if (!drain_pending() || !staging_leases_.has_room()) return false;
    ModeOwnerRequest request{sizeof(ModeOwnerRequest),cpu_scanout_abi_version,mode_generation_,mode_index_,lease_generation_};
    VIDEO_MEMORY_INFORMATION result{};
    if (!control_status(driver_,ioctl_map_owned_staging,&request,sizeof(request),&result,sizeof(result),
            last_error_)) {
        // A failed status does not prove the MAP was not applied.
        unacknowledged_map_=mode_generation_+1U;
        recover_unacknowledged_map();
        return false;
    }
    // The atomic owned-MAP advances exactly once. Any later mode mutation is
    // rejected by connect rather than silently adopting its read-only inventory.
    ++mode_generation_;
    if (result.FrameBufferBase==nullptr || result.FrameBufferLength==0 ||
        !staging_leases_.remember(result.FrameBufferBase,mode_generation_)) {
        // A malformed successful response cannot publish a bitmap. The only
        // known retirement credential is the acknowledged atomic MAP token.
        // Retain it through uncertain release instead of losing native RAM.
        if (staging_leases_.remember_unpublished(mode_generation_))
            staging_leases_.drain(release_staging_lease,this);
        return false;
    }
    address=result.FrameBufferBase;
    bytes=result.FrameBufferLength;
    return true;
}

bool DisplayLink::unmap_staging(void* address) noexcept {
    return staging_leases_.retire(address,release_staging_lease,this);
}

bool DisplayLink::resolve_lease(uint32_t generation,uint32_t operation,
    StagingResolveReceipt& receipt) noexcept {
    if (driver_==nullptr || generation==0) return false;
    StagingResolveRequest request{sizeof(request),cpu_scanout_abi_version,generation,operation,lease_generation_};
    StagingResolveReceipt candidate{};
    if (!control(driver_,ioctl_resolve_owned_staging,&request,sizeof(request),&candidate,sizeof(candidate)) ||
        !valid_staging_resolve_receipt(candidate,request)) return false;
    resolve_supported_=true;
    receipt=candidate;
    return true;
}

bool DisplayLink::release_staging_lease(void* context,uint32_t generation) noexcept {
    auto& owner=*static_cast<DisplayLink*>(context);
    StagingResolveReceipt receipt{};
    // Every valid release answer is final: released and not_held prove no
    // lease remains under the token (exactly one retirement happened), and
    // unknown proves none was issued, so that token is dropped, never retried.
    if (owner.resolve_lease(generation,staging_resolve_release,receipt)) return true;
    // Once the owner has answered authoritatively, a failure is a lost or
    // refused request. Keep the unchanged token for the next bounded retry.
    if (owner.resolve_supported_) return false;
    // Older miniports: the original release, whose repeat after an applied
    // but unacknowledged release stays a bounded native quarantine.
    StagingReleaseRequest request{sizeof(request),cpu_scanout_abi_version,generation,0};
    return owner.driver_!=nullptr && control(owner.driver_,ioctl_release_owned_staging,&request,sizeof(request));
}

// A failed owned-MAP status is ambiguous: the miniport may have issued the
// lease and advanced the mode before the acknowledgement was lost. Ask about
// that exact next generation. Only a lease issued to this PDEV's own
// initiating credential is adopted, then retired token-only because its
// address was never received. A different requester's lease is untouched.
bool DisplayLink::recover_unacknowledged_map() noexcept {
    if (unacknowledged_map_==0) return true;
    StagingResolveReceipt receipt{};

    if (!resolve_lease(unacknowledged_map_,staging_resolve_query,receipt)) {
        // A legacy owner cannot answer; keep its earlier failed-MAP behavior.
        if (!resolve_supported_) unacknowledged_map_=0;
        return unacknowledged_map_==0;
    }

    if (receipt.outcome==staging_lease_held && (receipt.flags&staging_flag_initiator)!=0) {
        if (!staging_leases_.remember_unpublished(unacknowledged_map_)) return false;
        mode_generation_=unacknowledged_map_;
        staging_leases_.drain(release_staging_lease,this);
    }

    unacknowledged_map_=0;
    return true;
}

bool DisplayLink::device_lost() noexcept {
    if (lost_) return true;
    const uint32_t probe=lease_generation_!=0 ? lease_generation_ : mode_generation_;
    StagingResolveReceipt receipt{};
    return probe!=0 && resolve_lease(probe,staging_resolve_query,receipt) &&
        (receipt.flags&staging_flag_device_lost)!=0;
}

bool DisplayLink::transfer_lost_leases() noexcept {
    lost_=true;
    hardware_cursor_=false;
    recovery_required_=false;
    cursor_={};
    recover_unacknowledged_map();
    staging_leases_.retire_all();
    // Remaining tokens stay pending; PDEV teardown retries them again.
    return drain_pending();
}

bool DisplayLink::drain_pending() noexcept {
    for (uint32_t attempt=0;attempt<retire_attempts;++attempt) {
        if (staging_leases_.drain(release_staging_lease,this)) return true;
    }
    return false;
}

bool DisplayLink::present(rx7900::reactos::framebuffer::Rectangle rectangle) noexcept {
    if (driver_ == nullptr || lost_ || mode_generation_ == 0 || rectangle.left < 0 || rectangle.top < 0 ||
        rectangle.right <= rectangle.left || rectangle.bottom <= rectangle.top) return false;
    struct Request final {
        DamageRequest header;
        LightningDisplayRect rectangle;
    } request{{sizeof(Request), cpu_scanout_abi_version, mode_generation_, 1},
        {static_cast<uint32_t>(rectangle.left), static_cast<uint32_t>(rectangle.top),
         static_cast<uint32_t>(rectangle.right - rectangle.left),
         static_cast<uint32_t>(rectangle.bottom - rectangle.top)}};
    if (control_status(driver_, ioctl_present_cpu_scanout, &request, sizeof(request), nullptr, 0,
            last_error_)) return true;
    // A bounded coalescing queue can fill. Drain the actual accepted prefix and
    // retry once; the dirty owner retains all damage if either operation fails.
    return control(driver_, ioctl_wait_cpu_scanout) &&
        control(driver_, ioctl_present_cpu_scanout, &request, sizeof(request));
}

bool DisplayLink::move_cursor(LONG x, LONG y) noexcept {
    if (!hardware_cursor_) return false;
    if (x==-1) recovery_required_=false;
    cursor::Position position{};
    if (!cursor::from_hotspot(x,y,hot_x_,hot_y_,position)) return false;
    CursorMoveRequest request{sizeof(request), cpu_scanout_abi_version,mode_generation_,
        position.left,position.top,position.visible ? 1U : 0U};
    if (!control(driver_, ioctl_cursor_move, &request, sizeof(request))) return false;
    cursor_.header.x=request.x;
    cursor_.header.y=request.y;
    cursor_.header.visible=request.visible;
    return true;
}

bool DisplayLink::suspend_cursor() noexcept {
    recovery_required_=false;
    if (!hardware_cursor_) return true;
    CursorMoveRequest request{sizeof(request),cpu_scanout_abi_version,mode_generation_,0,0,0};
    return driver_!=nullptr && control(driver_,ioctl_cursor_move,&request,sizeof(request));
}

bool DisplayLink::hide_cursor() noexcept {
    const bool hidden=suspend_cursor();
    if (!hidden) return false;
    hardware_cursor_ = false;
    cursor_.header={};
    return true;
}

bool DisplayLink::restore_cursor() noexcept {
    recovery_required_=false;
    if (!hardware_cursor_) return true;
    cursor_.header.mode_generation=mode_generation_;
    if (cursor_.header.size<sizeof(CursorShapeRequest) ||
        cursor_.header.size>sizeof(cursor_) ||
        !control(driver_,ioctl_cursor_shape,&cursor_,cursor_.header.size)) return false;
    return true;
}

bool DisplayLink::recover_failed_cursor_move() noexcept {
    // The registry caller has already proved a live visible PDEV. Recheck the
    // native mode before signaling the engine: D3/stale cleanup must not ask it
    // to reselect a cursor against an inactive or different canvas.
    CpuScanoutInformation information{};
    if (!hardware_cursor_ || !query(information) || information.enabled!=1 ||
        information.mode_generation!=mode_generation_) return false;
    if (!suspend_cursor()) return false;
    recovery_required_=true;
    return true;
}

void DisplayLink::abandon_cursor_restore() noexcept {
    if (!hardware_cursor_) return;
    (void)suspend_cursor();
    recovery_required_=true;
}

bool DisplayLink::query_cursor(CursorStatistics& statistics) noexcept {
    CursorStatistics candidate{};
    if (driver_==nullptr || !control(driver_,ioctl_query_cursor,nullptr,0,&candidate,sizeof(candidate)) ||
        candidate.size!=sizeof(candidate) || candidate.version!=cpu_scanout_abi_version ||
        candidate.magic!=cursor_statistics_magic) return false;
    candidate.hardware_active=hardware_cursor_ && !recovery_required_ && candidate.defined!=0 &&
        candidate.mode_generation==mode_generation_ ? 1U : 0U;
    candidate.recovery_required=0;
    if (recovery_required_ && candidate.mode_generation==mode_generation_) {
        // A filter-driven power transition need not change PDEV visibility or
        // generation. Its cached inventory still proves whether reselecting
        // against this canvas is currently permitted, without powered MMIO.
        CpuScanoutInformation information{};
        if (query(information) && information.enabled==1 &&
            information.mode_generation==mode_generation_) candidate.recovery_required=1;
    }
    candidate.reserved=0;
    statistics=candidate;
    return true;
}

bool DisplayLink::define_cursor(SURFOBJ* mask, SURFOBJ* color, XLATEOBJ* translation,
    LONG hot_x, LONG hot_y, LONG x, LONG y, FLONG flags) noexcept {
    recovery_required_=false;
    if (!hide_cursor()) return false;
    SURFOBJ* dimensions = color != nullptr ? color : mask;
    if (driver_ == nullptr || dimensions == nullptr || dimensions->sizlBitmap.cx <= 0 ||
        dimensions->sizlBitmap.cy <= 0) return false;
    const LONG width = dimensions->sizlBitmap.cx;
    const LONG height = color != nullptr ? color->sizlBitmap.cy : mask->sizlBitmap.cy / 2;
    if (width > static_cast<LONG>(LIGHTNING_SCANOUT_CURSOR_MAX_DIMENSION) || height <= 0 ||
        height > static_cast<LONG>(LIGHTNING_SCANOUT_CURSOR_MAX_DIMENSION) ||
        hot_x < 0 || hot_y < 0 || hot_x >= width || hot_y >= height ||
        (color == nullptr && mask->sizlBitmap.cy != height * 2)) return false;
    const bool alpha = color != nullptr && (flags & SPS_ALPHA) != 0;
    if (alpha && (color->iBitmapFormat != BMF_32BPP ||
        (translation != nullptr && (translation->flXlate & XO_TRIVIAL) == 0))) return false;
    EngineBitmap normalized_mask;
    EngineBitmap normalized_color;
    const POINTL origin{0,0};
    if (!alpha && (mask == nullptr || mask->iBitmapFormat != BMF_1BPP ||
        !normalized_mask.copy(mask, width, color == nullptr ? height * 2 : height,
                             BMF_1BPP,nullptr,origin,LIGHTNING_SCANOUT_CURSOR_SHAPE_BYTES))) return false;
    if (color != nullptr && !normalized_color.copy(color, width, height, BMF_32BPP,
                                                  alpha ? nullptr : translation,origin,LIGHTNING_SCANOUT_CURSOR_SHAPE_BYTES)) return false;

    const uint32_t pitch = color == nullptr ? static_cast<uint32_t>(((width + 31) / 32) * 4) :
        static_cast<uint32_t>(width * 4);
    const uint32_t bytes = pitch * static_cast<uint32_t>(color == nullptr ? height * 2 : height);
    const uint32_t request_bytes = sizeof(CursorShapeRequest) + bytes;
    auto* allocation = static_cast<uint8_t*>(EngAllocMem(FL_ZERO_MEMORY, request_bytes, ALLOC_TAG));
    if (allocation == nullptr) return false;
    cursor::Position position{};
    bool success=cursor::from_hotspot(x,y,hot_x,hot_y,position);
    if (success) {
        CursorShapeRequest header{request_bytes, cpu_scanout_abi_version,mode_generation_,
            static_cast<uint32_t>(width), static_cast<uint32_t>(height), pitch,
            color == nullptr ? LIGHTNING_SCANOUT_CURSOR_MONOCHROME :
                (alpha ? LIGHTNING_SCANOUT_CURSOR_COLOR : LIGHTNING_SCANOUT_CURSOR_MASKED_COLOR),
            static_cast<uint32_t>(hot_x), static_cast<uint32_t>(hot_y),
            position.left,position.top,position.visible ? 1U : 0U,bytes};
        memcpy(allocation, &header, sizeof(header));
        auto* payload = allocation + sizeof(header);
        SURFOBJ* normalized = color == nullptr ? normalized_mask.surface() : normalized_color.surface();
        memcpy(payload, normalized->pvScan0, bytes);
        if (color != nullptr && !alpha) {
            for (uint32_t row = 0; row < static_cast<uint32_t>(height); ++row) {
                for (uint32_t column = 0; column < static_cast<uint32_t>(width); ++column) {
                    payload[row * pitch + column * 4U + 3U] =
                        mask_pixel(*normalized_mask.surface(), column, row) ? 0x80U : 0U;
                }
            }
        } else if (alpha) {
            // GDI owns premultiplied BGRA. The existing COLOR wire and host
            // cursor API use straight channels; preserve the alpha byte.
            for (uint32_t row=0;row<static_cast<uint32_t>(height);++row) {
                for (uint32_t column=0;column<static_cast<uint32_t>(width);++column) {
                    auto* pixel=payload+row*pitch+column*LIGHTNING_SCANOUT_PIXEL_BYTES;
                    for (uint32_t channel=0;channel<3U;++channel) {
                        pixel[channel]=::lightning::cursor::straight_channel(pixel[channel],pixel[3]);
                    }
                }
            }
        }
        const auto format=color==nullptr ? ::lightning::cursor::Format::monochrome :
            (alpha ? ::lightning::cursor::Format::color : ::lightning::cursor::Format::masked_color);
        success=::lightning::cursor::exact_shape({header.width,header.height,header.pitch,
            header.hot_x,header.hot_y,format},LIGHTNING_SCANOUT_CURSOR_MAX_DIMENSION,payload,bytes);
        if (success) success=control(driver_,ioctl_cursor_shape,allocation,request_bytes);
        if (success) memcpy(&cursor_,allocation,request_bytes);
    }
    EngFreeMem(allocation);
    hardware_cursor_ = success;
    if (success) {
        hot_x_ = hot_x;
        hot_y_ = hot_y;
    }
    return success;
}

void DisplayLink::retire() noexcept {
    recovery_required_=false;
    hide_cursor();
    if (current_owner()) control(driver_, ioctl_wait_cpu_scanout);
    hardware_cursor_=false;
    cursor_={};
    // Retired old/candidate wrappers have already been deleted. Retry their
    // generation-only releases before forgetting this PDEV. ShadowDisable has
    // also retired the final wrapper; an initial Enable failure instead leaves
    // its live ScreenPtr to upstream cleanup. Failed pending releases stay
    // bounded and quarantined in native staging rather than freeing uncertain RAM.
    recover_unacknowledged_map();
    drain_pending();
    staging_leases_={};
    driver_ = nullptr;
    mode_generation_ = 0;
    mode_index_=0;
    lease_generation_=0;
    resolve_supported_=false;
    lost_=false;
    unacknowledged_map_=0;
}

} // namespace lightning::reactos

/*
Module: lightning.reactos.video
Purpose: Own the real Lightning VideoPort display, damage and cursor resources.
Created: 2026-10-21
Architecture: Functional display miniport; the separate transport UpperFilter
owns QLV/DMA. This module never identifies a virtual display as a physical AMD.
License: GPL-2.0-or-later; canonical generated ABI is retained verbatim.
*/
#include "video_sdk.hpp"
#include "pci.hpp"
#include "lifetime.hpp"
#include "video_helpers.hpp"
#include "lightning_scanout.hpp"
#include "lightning_present_abi.hpp"
#include "lightning_staging.hpp"
#include "lightning_staging_resolve.hpp"
#include "lightning_staging_pool.hpp"
#include "lightning_mode_lifecycle.hpp"
#include "lightning_pci_bar.hpp"
#include "lightning_cursor_snapshot.hpp"
#include "lightning_cursor_policy.hpp"
#ifdef RX7900_LIGHTNING_GDI_OFFLOAD
#include "lightning_gdi_native.hpp"
#include "lightning_gdi_completion.hpp"
#endif
#include "modes.hpp"

namespace {
using namespace lightning::reactos;
using rx7900::ByteView;
using rx7900::reactos::aligned_video_extension;
using rx7900::reactos::read_video_value;
using rx7900::reactos::return_video_value;

constexpr uint32_t device_signature=0x3144564cU;
constexpr uint32_t wait_iterations=1000;
constexpr uint32_t wait_microseconds=1000;
constexpr uint32_t initial_mode=1;
constexpr uint32_t control_bytes=LIGHTNING_SCANOUT_CURSOR_SHAPE_OFFSET+LIGHTNING_SCANOUT_CURSOR_SHAPE_BYTES;

struct VideoState final {
    uint32_t signature=0;
    KMUTEX lock{};
    CpuScanout scanout{};
    DisplayCompletion last_completion{};
    StagingLedger staging{};
    PULONG registers=nullptr;
    PVOID framebuffer=nullptr;
    ULONG framebuffer_bytes=0;
    uint32_t desired_mode=initial_mode;
    uint32_t mode_generation=0;
    ModeRollbackCredential rollback{};
    bool initialized=false;
    bool metadata_only=false;
    bool quiesced=true;
    bool blank_published=false;
    bool lost=false;
    CursorShapeSnapshot cursor{};
    CursorStatistics cursor_statistics{};
#ifdef RX7900_LIGHTNING_GDI_OFFLOAD
    lightning::reactos::gdi::NativeCanvas gpu{};
    lightning::reactos::gdi::CompletionSignal gpu_completion{};
    bool irq_configured=false;
    bool irq_connected=false;
    PVOID gpu_transfer=nullptr;
    bool gpu_snapshot=false;
    uint32_t snapshot_generation=0;
#endif
#ifdef RX7900_LIGHTNING_LEASE_FAULTS
    // Debug-only acknowledgement faults, loaded once from LeaseFaultPlan.
    StagingFaultPlan lease_faults{};
    bool lease_faults_loaded=false;
#endif
    // The mapped owner's authenticated disable {lease, mode token}, and the
    // single MAP ticket of the preauthenticated framebuf mode set
    // (lightning_mode_lifecycle.hpp). Every mode-token advance clears both.
    ModeRollbackCredential yield{};
    uint32_t initial_map=0;
    // recover() proved the scanout state unrecoverable; reported once.
    bool scanout_unrecoverable=false;
};

VP_STATUS restore_cursor(VideoState& state) noexcept;
VP_STATUS hide_cursor(VideoState& state,bool retain_visibility) noexcept;

void increment_cursor_counter(uint64_t& counter) noexcept {
    if (counter!=UINT64_MAX) ++counter;
}

VideoState* constructed_state(PVOID extension) noexcept {
    VideoState* state=aligned_video_extension<VideoState>(extension);
    if (state==nullptr) return nullptr;
    // VideoPort can deliver an OPEN before HwFindAdapter constructed our C++
    // state. Read only raw storage until its initialization marker is published;
    // in particular never wait on the zero/uninitialized KMUTEX.
    uint32_t marker=0;
    VideoPortMoveMemory(&marker,state,sizeof(marker));
    KeMemoryBarrier();
    return marker==device_signature ? state : nullptr;
}

class Guard final {
public:
    explicit Guard(VideoState& state) noexcept : state_(state) {
        KeWaitForSingleObject(&state_.lock,Executive,KernelMode,FALSE,nullptr);
    }
    ~Guard() { KeReleaseMutex(&state_.lock,FALSE); }
    Guard(const Guard&)=delete;
    Guard& operator=(const Guard&)=delete;
private:
    VideoState& state_;
};

VP_STATUS port_status(ScanoutStatus status) noexcept {
    switch (status) {
    case ScanoutStatus::success: return NO_ERROR;
    case ScanoutStatus::busy: return ERROR_BUSY;
    case ScanoutStatus::unavailable: return ERROR_DEV_NOT_EXIST;
    case ScanoutStatus::state_unknown:
    case ScanoutStatus::hardware_failure: return ERROR_DEVICE_REINITIALIZATION_NEEDED;
    default: return ERROR_INVALID_PARAMETER;
    }
}

bool register_read(void* context,uint32_t offset,uint32_t& value) noexcept {
    auto& state=*static_cast<VideoState*>(context);
    if (state.lost || state.registers==nullptr || (offset&3U)!=0 || offset>control_bytes-sizeof(ULONG)) return false;
    value=VideoPortReadRegisterUlong(state.registers+offset/sizeof(ULONG));
    return true;
}

bool register_write(void* context,uint32_t offset,uint32_t value) noexcept {
    auto& state=*static_cast<VideoState*>(context);
    // No QLV mailbox or arbitrary register is writable through this miniport.
    const bool scanout=offset>=LIGHTNING_SCANOUT_BASE && offset<LIGHTNING_DISPLAY_END;
    const bool damage=offset>=LIGHTNING_DISPLAY_DAMAGE_OFFSET &&
        offset<LIGHTNING_DISPLAY_DAMAGE_OFFSET+LIGHTNING_DISPLAY_DAMAGE_BYTES;
    const bool cursor=offset>=LIGHTNING_SCANOUT_CURSOR_SHAPE_OFFSET && offset<control_bytes;
    bool permitted=scanout || damage || cursor;
#ifdef RX7900_LIGHTNING_GDI_OFFLOAD
    permitted=permitted || (offset>=LIGHTNING_GDI_REG_MAGIC && offset<LIGHTNING_GDI_END);
#endif
    if (state.lost || state.registers==nullptr || (offset&3U)!=0 || !permitted) return false;
    VideoPortWriteRegisterUlong(state.registers+offset/sizeof(ULONG),value);
    return true;
}

bool clear_framebuffer(void* context,uint32_t bytes) noexcept {
    auto& state=*static_cast<VideoState*>(context);
    if (state.lost || state.framebuffer==nullptr || state.staging.current()==nullptr ||
        bytes>state.framebuffer_bytes || bytes>state.staging.current_bytes()) return false;
    // Existing GDI leases may still refer to staging RAM. Only the device-owned
    // PCI scanout is cleared here; each subsequent MAP gets freshly zeroed RAM.
    VideoPortZeroDeviceMemory(state.framebuffer,bytes);
    return true;
}

void release_maps(VideoState& state,PVOID extension) noexcept {
#ifdef RX7900_LIGHTNING_GDI_OFFLOAD
    state.gpu_completion.disable(!state.lost);
    state.irq_connected=false;
    if (state.gpu_transfer!=nullptr) VideoPortFreeDeviceBase(extension,state.gpu_transfer);
    state.gpu_transfer=nullptr;
    // Only CPU virtual mappings retire here. The QEMU-owned BAR and uncertain
    // transfer ledger are never freed/reused or reset by a native restart.
#endif
    if (state.framebuffer!=nullptr) VideoPortFreeDeviceBase(extension,state.framebuffer);
    if (state.registers!=nullptr) VideoPortFreeDeviceBase(extension,state.registers);
    state.framebuffer=nullptr;
    state.registers=nullptr;
    state.framebuffer_bytes=0;
    state.initialized=false;
    state.blank_published=false;
    state.metadata_only=false;
    state.last_completion={};
    state.quiesced=true;
    state.cursor_statistics.visible=0;
    state.cursor_statistics.defined=0;
    // A retired GDI surface can still write its cached mapping after STOP or
    // removal. Retain every leased RAM allocation; never free under that lease.
    state.staging.retire();
    while (void* retired=state.staging.reclaim()) ExFreePoolWithTag(retired,0x73564c51U);
}

#ifdef RX7900_LIGHTNING_GDI_OFFLOAD
bool prepare_gpu_completion(void* context) noexcept {
    return static_cast<VideoState*>(context)->gpu_completion.prepare();
}

bool wait_gpu_completion(void* context,uint64_t remaining_100ns) noexcept {
    return static_cast<VideoState*>(context)->gpu_completion.wait(remaining_100ns);
}

lightning::reactos::gdi::NativeView gpu_view(VideoState& state) noexcept {
    VirtualMode mode{};
    virtual_mode(state.desired_mode,mode);
    state.gpu_completion.enable(state.registers,state.irq_connected);
    return {&state,register_read,register_write,state.gpu_transfer,
        state.gpu_transfer==nullptr ? 0U : LIGHTNING_GDI_TRANSFER_BYTES,
        state.staging.current(),state.staging.current_bytes(),
        mode.width*LIGHTNING_SCANOUT_PIXEL_BYTES,mode.width,mode.height,state.mode_generation,
        prepare_gpu_completion,wait_gpu_completion};
}
#endif

VP_STATUS new_staging(VideoState& state) noexcept {
    if (!state.staging.can_replace()) return ERROR_BUSY;
    // AMD64 ReactOS has a 32 MB system paged pool (lightning_staging_pool.hpp).
    constexpr POOL_TYPE pool=frame_pool(sizeof(void*))==FramePool::nonpaged ? NonPagedPool : PagedPool;
    void* address=ExAllocatePoolWithTag(pool,state.framebuffer_bytes,0x73564c51U);
    if (address==nullptr) return ERROR_NOT_ENOUGH_MEMORY;
    VideoPortZeroMemory(address,state.framebuffer_bytes);
    state.staging.retire();
    while (void* retired=state.staging.reclaim()) ExFreePoolWithTag(retired,0x73564c51U);
    if (!state.staging.install(address,state.framebuffer_bytes)) {
        ExFreePoolWithTag(address,0x73564c51U);
        return ERROR_DEVICE_REINITIALIZATION_NEEDED;
    }
    return NO_ERROR;
}

bool copy_staging(VideoState& state,VirtualMode mode,const LightningDisplayRect* rectangles,
                  uint32_t count) noexcept {
    if (state.lost || state.framebuffer==nullptr || state.staging.current()==nullptr ||
        !valid_damage(mode,rectangles,count)) return false;
    const uint64_t stride=static_cast<uint64_t>(mode.width)*LIGHTNING_SCANOUT_PIXEL_BYTES;
    if (stride*mode.height>state.framebuffer_bytes || stride*mode.height>state.staging.current_bytes()) return false;
    // The DDI holds its cached surface lock throughout this synchronous IOCTL.
    // Source and destination are disjoint owned mappings. Only the validated
    // rows touch BAR4; the KMUTEX prevents any copy after LOST/quiesce returns.
    for (uint32_t i=0;i<count;++i) {
        const auto& rectangle=rectangles[i];
        for (uint32_t row=0;row<rectangle.height;++row) {
            const size_t offset=static_cast<size_t>((rectangle.y+row)*stride+
                static_cast<uint64_t>(rectangle.x)*LIGHTNING_SCANOUT_PIXEL_BYTES);
            VideoPortMoveMemory(static_cast<uint8_t*>(state.framebuffer)+offset,
                static_cast<uint8_t*>(state.staging.current())+offset,
                rectangle.width*LIGHTNING_SCANOUT_PIXEL_BYTES);
        }
    }
    KeMemoryBarrier();
    return true;
}

VP_STATUS wait_publication(VideoState& state) noexcept {
    const uint64_t begin=KeQueryInterruptTime();
    for (uint32_t attempt=0;attempt<wait_iterations;++attempt) {
        DisplayCompletion completion{};
        const auto status=state.scanout.completion(completion);
        if (status!=ScanoutStatus::success && status!=ScanoutStatus::busy) return port_status(status);
        if (status==ScanoutStatus::success && completion.published) {
            state.last_completion=completion;
            return NO_ERROR;
        }
        const uint64_t now=KeQueryInterruptTime();
        if (now<begin || now-begin>=10000000ULL) break;
        // Serialized PASSIVE_LEVEL mode/lifecycle path only. Yield between
        // polls; no spin at DIRQL and no artificial frame-completion success.
        LARGE_INTEGER delay{};
        delay.QuadPart=-static_cast<LONGLONG>(wait_microseconds)*10;
        KeDelayExecutionThread(KernelMode,FALSE,&delay);
    }
    return ERROR_BUSY;
}

// An uncertain earlier scanout command is resolved by a bounded re-query,
// never by assuming either outcome. Success leaves no mode assumed, so each
// caller then commits a FULL in a new epoch: the replay. Serialized PASSIVE.
VP_STATUS recover_scanout(VideoState& state) noexcept {
    if (!state.scanout.quarantined()) return NO_ERROR;
    if (state.scanout_unrecoverable) return ERROR_DEVICE_REINITIALIZATION_NEEDED;
    const uint64_t begin=KeQueryInterruptTime();
    for (uint32_t attempt=0;attempt<wait_iterations;++attempt) {
        const auto status=state.scanout.recover();
        if (status==ScanoutStatus::success) {
            DbgPrint("LightningVideo: scanout recovered after an uncertain command; replaying mode %lu generation %lu\n",
                state.desired_mode,state.mode_generation);
            return NO_ERROR;
        }
        if (status!=ScanoutStatus::busy) {
            state.scanout_unrecoverable=true;
            DbgPrint("LightningVideo: scanout state unprovable; quarantine is permanent\n");
            return ERROR_DEVICE_REINITIALIZATION_NEEDED;
        }
        const uint64_t now=KeQueryInterruptTime();
        if (now<begin || now-begin>=10000000ULL) break;
        LARGE_INTEGER delay{};
        delay.QuadPart=-static_cast<LONGLONG>(wait_microseconds)*10;
        KeDelayExecutionThread(KernelMode,FALSE,&delay);
    }
    return ERROR_BUSY;
}

VP_STATUS set_mode(VideoState& state,uint32_t index,bool preserve,bool resume=false,
                   uint32_t initiating_lease=0) noexcept {
    if (state.lost || state.metadata_only || state.staging.current()==nullptr) return ERROR_DEV_NOT_EXIST;
    if (!resume && state.mode_generation==UINT32_MAX) return ERROR_DEVICE_REINITIALIZATION_NEEDED;
    VirtualMode mode{};
    // Reject input before changing visibility, cursor or GPU ownership.
    if (virtual_mode(index,mode)!=ScanoutStatus::success ||
        (preserve && index!=state.desired_mode)) return ERROR_INVALID_PARAMETER;
    VP_STATUS status=recover_scanout(state);
    if (status!=NO_ERROR) return status;
    // Once retirement begins, a failed transition must not expose the old
    // owner as active. Its token/retained RAM can authorize an explicit retry.
    state.quiesced=true;
    state.blank_published=false;
    if (state.cursor_statistics.defined!=0 && hide_cursor(state,true)!=NO_ERROR)
        return ERROR_DEVICE_REINITIALIZATION_NEEDED;
#ifdef RX7900_LIGHTNING_GDI_OFFLOAD
    if (state.gpu.has_canvas() || state.gpu.quarantined()) {
        if (preserve && state.gpu.has_canvas()) {
            if (!state.gpu.snapshot(gpu_view(state),state.framebuffer,state.framebuffer_bytes))
                return ERROR_DEVICE_REINITIALIZATION_NEEDED;
            state.gpu_snapshot=true;
            state.snapshot_generation=state.mode_generation;
        }
        // RESET itself proves actual backend retirement. A failed execution
        // drain must not permanently prevent this supported cancellation path;
        // preserve-mode snapshot above already established required pixels.
        if (!state.gpu.reset(gpu_view(state))) return ERROR_DEVICE_REINITIALIZATION_NEEDED;
    }
#endif
    status=wait_publication(state);
    if (status!=NO_ERROR) return status;
    const LightningDisplayRect full{0,0,mode.width,mode.height};
    bool retained_primary=false;
#ifdef RX7900_LIGHTNING_GDI_OFFLOAD
    retained_primary=state.gpu_snapshot && state.snapshot_generation==state.mode_generation && index==state.desired_mode;
#endif
    if (preserve && !retained_primary && !copy_staging(state,mode,&full,1)) return ERROR_DEV_NOT_EXIST;
    if (!resume) {
        state.rollback=current_mode_rollback_credential(state.staging,state.mode_generation,initiating_lease);
        // Geometry/clear can fail after touching scanout. Invalidate before
        // these writes, even if the accepted publication later times out.
        ++state.mode_generation;
        state.yield={};
        state.initial_map=0;
        state.cursor_statistics.defined=0;
        state.cursor.header={};
#ifdef RX7900_LIGHTNING_GDI_OFFLOAD
        state.gpu_snapshot=false;
#endif
    }
    status=port_status(state.scanout.set_mode(index,preserve));
    if (status!=NO_ERROR) return status;
    state.desired_mode=index;
    status=wait_publication(state);
    if (status!=NO_ERROR) return status;
    if (resume && restore_cursor(state)!=NO_ERROR) return ERROR_DEVICE_REINITIALIZATION_NEEDED;
    state.quiesced=false;
    return NO_ERROR;
}

// Caller holds the native mutex and has validated the fixed output before any
// old lease is retired. All callers reuse this one allocation/mapping owner.
// initial: the preauthenticated framebuf MAP, which keeps its SET's rollback.
VP_STATUS map_staging(VideoState& state,VIDEO_MEMORY_INFORMATION& output,
                      uint32_t initiating_lease=0,bool initial=false) noexcept {
#ifdef RX7900_LIGHTNING_GDI_OFFLOAD
    if (state.gpu.quarantined() || state.gpu.has_canvas()) {
        return state.gpu.quarantined() ? ERROR_DEVICE_REINITIALIZATION_NEEDED : ERROR_BUSY;
    }
#endif
    if (state.quiesced || state.mode_generation==UINT32_MAX) return ERROR_DEVICE_REINITIALIZATION_NEEDED;
    if (state.cursor_statistics.defined!=0 && hide_cursor(state,false)!=NO_ERROR)
        return ERROR_DEVICE_REINITIALIZATION_NEEDED;
    state.rollback=map_rollback_credential(state.staging,state.mode_generation,initiating_lease,initial,state.rollback);
    VP_STATUS status=new_staging(state);
    if (status!=NO_ERROR) return status;
    output.VideoRamBase=state.staging.acquire(state.mode_generation+1U,initiating_lease);
    if (output.VideoRamBase==nullptr) return ERROR_BUSY;
    output.VideoRamLength=state.staging.current_bytes();
    output.FrameBufferBase=output.VideoRamBase;
    output.FrameBufferLength=output.VideoRamLength;
    ++state.mode_generation;
    state.yield={};
    state.initial_map=0;
    state.cursor_statistics.defined=0;
    state.cursor.header={};
#ifdef RX7900_LIGHTNING_GDI_OFFLOAD
    state.gpu_snapshot=false;
#endif
    return NO_ERROR;
}

VP_STATUS preinit_power(VideoState& state,bool resume) noexcept {
    if (resume) {
        state.metadata_only=false;
        return NO_ERROR;
    }
    if (state.metadata_only) return NO_ERROR;
    // The filter calls this before the lower PCI stack leaves D0. A recognized
    // adapter can be suspended before its first OPEN, without initializing it.
    const VP_STATUS status=wait_publication(state);
    if (status==NO_ERROR) state.metadata_only=true;
#ifdef RX7900_LIGHTNING_GDI_OFFLOAD
    if (status==NO_ERROR) state.gpu_completion.disable();
#endif
    return status;
}

VP_STATUS quiesce(VideoState& state) noexcept {
    if (state.lost) return ERROR_DEV_NOT_EXIST;
    if (state.quiesced && state.blank_published) return NO_ERROR;
    VP_STATUS status=recover_scanout(state);
    if (status!=NO_ERROR) return status;
    state.quiesced=true;
    state.blank_published=false;
    if (hide_cursor(state,true)!=NO_ERROR) return ERROR_DEVICE_REINITIALIZATION_NEEDED;
#ifdef RX7900_LIGHTNING_GDI_OFFLOAD
    if (state.gpu.has_canvas()) {
        if (!state.gpu.snapshot(gpu_view(state),state.framebuffer,state.framebuffer_bytes))
            return ERROR_DEVICE_REINITIALIZATION_NEEDED;
        state.gpu_snapshot=true;
        state.snapshot_generation=state.mode_generation;
    }
    if ((state.gpu.has_canvas() || state.gpu.quarantined()) && !state.gpu.reset(gpu_view(state)))
        return ERROR_DEVICE_REINITIALIZATION_NEEDED;
#endif
    status=wait_publication(state);
    if (status!=NO_ERROR) return status;
    status=port_status(state.scanout.disable());
    if (status==ERROR_BUSY) {
        status=wait_publication(state);
        if (status==NO_ERROR) status=port_status(state.scanout.disable());
    }
    if (status!=NO_ERROR) return status;
    status=wait_publication(state);
    if (status==NO_ERROR) {
        state.quiesced=true;
        state.blank_published=true;
#ifdef RX7900_LIGHTNING_GDI_OFFLOAD
        state.gpu_completion.disable();
#endif
    }
    return status;
}

// Owner-independent power/filter resume restores intent only: a display its
// mapped owner yielded stays blank, and a preauthenticated mode set whose MAP
// is still pending resumes cleared instead of showing the yielded owner's RAM.
VP_STATUS resume_scanout(VideoState& state) noexcept {
    if (resume_keeps_yield(state.yield,state.mode_generation,state.quiesced,state.blank_published)) return NO_ERROR;
    return set_mode(state,state.desired_mode,resume_preserves(state.initial_map,state.mode_generation),true);
}

VP_STATUS mode_description(VideoState& state,uint32_t index,VIDEO_MODE_INFORMATION& output) noexcept {
    VirtualMode value{};
    const auto status=virtual_mode(index,value);
    if (status!=ScanoutStatus::success) return port_status(status);
    VIDEO_MODE_INFORMATION mode{};
    mode.Length=sizeof(mode);
    mode.ModeIndex=index;
    mode.VisScreenWidth=value.width;
    mode.VisScreenHeight=value.height;
    mode.ScreenStride=value.width*LIGHTNING_SCANOUT_PIXEL_BYTES;
    if (static_cast<uint64_t>(mode.ScreenStride)*value.height>state.framebuffer_bytes) return ERROR_INVALID_PARAMETER;
    mode.NumberOfPlanes=1;
    mode.BitsPerPlane=32;
    mode.Frequency=LIGHTNING_SCANOUT_MODE_REFRESH_HZ;
    mode.NumberRedBits=mode.NumberGreenBits=mode.NumberBlueBits=LIGHTNING_SCANOUT_COLOR_BITS;
    mode.RedMask=0x00ff0000U;
    mode.GreenMask=0x0000ff00U;
    mode.BlueMask=0x000000ffU;
    mode.AttributeFlags=VIDEO_MODE_COLOR|VIDEO_MODE_GRAPHICS|VIDEO_MODE_LINEAR|VIDEO_MODE_NO_OFF_SCREEN;
    mode.VideoMemoryBitmapWidth=value.width;
    mode.VideoMemoryBitmapHeight=value.height;
    output=mode;
    return NO_ERROR;
}

VP_STATUS query_information(VideoState& state,VIDEO_REQUEST_PACKET& request) noexcept {
    DisplayCompletion completion{};
    if (state.metadata_only || (state.quiesced && state.initialized)) {
        // Quiesce captured the completed blank while resources were still in
        // D0. Read-only inventory must not touch powered-down PCI registers.
        completion=state.last_completion;
    } else {
        const auto status=state.scanout.completion(completion);
        if (status!=ScanoutStatus::success) return port_status(status);
    }
    VirtualMode mode{};
    if (virtual_mode(state.desired_mode,mode)!=ScanoutStatus::success) return ERROR_DEV_NOT_EXIST;
    CpuScanoutInformation information{};
    information.size=sizeof(information);
    information.version=cpu_scanout_abi_version;
    information.magic=LIGHTNING_DISPLAY_MAGIC;
    information.width=mode.width;
    information.height=mode.height;
    information.stride=mode.width*LIGHTNING_SCANOUT_PIXEL_BYTES;
    information.mapped_bytes=state.framebuffer_bytes;
    information.capabilities=LIGHTNING_DISPLAY_CAPABILITIES;
    information.mode_generation=state.mode_generation;
    information.refresh_ns=completion.refresh_ns;
    information.accepted_epoch=completion.accepted_epoch;
    information.accepted_sequence=completion.accepted_sequence;
    information.completed_epoch=completion.completed_epoch;
    information.completed_sequence=completion.completed_sequence;
    information.enabled=!state.quiesced;
    return return_video_value(request,information);
}

VP_STATUS submit_damage(VideoState& state,VIDEO_REQUEST_PACKET& request) noexcept {
    DamageRequest header{};
    if (!read_video_value(request,header) || header.size!=request.InputBufferLength ||
        header.version!=cpu_scanout_abi_version || header.mode_generation!=state.mode_generation ||
        header.rectangle_count==0 || header.rectangle_count>LIGHTNING_DISPLAY_MAX_RECTS ||
        header.size!=sizeof(header)+header.rectangle_count*sizeof(LightningDisplayRect)) return ERROR_INVALID_PARAMETER;
    LightningDisplayRect rectangles[LIGHTNING_DISPLAY_MAX_RECTS]{};
    const ByteView input(static_cast<const uint8_t*>(request.InputBuffer),request.InputBufferLength);
    for (uint32_t i=0;i<header.rectangle_count;++i) {
        const size_t offset=sizeof(header)+i*sizeof(LightningDisplayRect);
        if (!input.read_u32(offset,rectangles[i].x) || !input.read_u32(offset+4,rectangles[i].y) ||
            !input.read_u32(offset+8,rectangles[i].width) || !input.read_u32(offset+12,rectangles[i].height)) {
            return ERROR_INVALID_PARAMETER;
        }
    }
    VirtualMode mode{};
    if (virtual_mode(state.desired_mode,mode)!=ScanoutStatus::success) return ERROR_INVALID_PARAMETER;
    // After an uncertain command, replay rather than retry: the recovered
    // FULL snapshots the owner's whole staging, which already holds this damage.
    if (state.scanout.quarantined()) return set_mode(state,state.desired_mode,true,true);
    if (!copy_staging(state,mode,rectangles,header.rectangle_count)) return ERROR_INVALID_PARAMETER;
    return port_status(state.scanout.damage(rectangles,header.rectangle_count));
}

bool wait_cursor_visibility(VideoState& state,bool visible) noexcept {
    const uint64_t begin=KeQueryInterruptTime();
    for (uint32_t attempt=0;attempt<wait_iterations;++attempt) {
        uint32_t command=0;
        uint32_t status=0;
        if (!register_read(&state,LIGHTNING_SCANOUT_REG_CURSOR_COMMAND,command) ||
            (visible && (!register_read(&state,LIGHTNING_SCANOUT_REG_CURSOR_STATUS,status) ||
                         status!=LIGHTNING_SCANOUT_ACTIVE))) return false;
        if (command==(visible ? LIGHTNING_SCANOUT_CURSOR_DEFINE : LIGHTNING_SCANOUT_CURSOR_HIDE)) return true;
        const uint64_t now=KeQueryInterruptTime();
        if (now<begin || now-begin>=10000000ULL) return false;
        // Normal motion on an already visible plane returns on the first
        // read. Only show/hide transitions wait for main-loop publication.
        if (attempt<20U) {
            VideoPortStallExecution(10);
        } else {
            LARGE_INTEGER delay{};
            delay.QuadPart=-10000;
            KeDelayExecutionThread(KernelMode,FALSE,&delay);
        }
    }
    return false;
}

VP_STATUS move_cursor(VideoState& state,const CursorMoveRequest& request) noexcept {
    if (request.size!=sizeof(request) || request.version!=cpu_scanout_abi_version ||
        request.mode_generation!=state.mode_generation || request.visible>1) {
        return ERROR_INVALID_PARAMETER;
    }
    if (request.visible==0) return hide_cursor(state,false);
    cursor::Position position{};
    if (state.cursor_statistics.defined==0 || state.cursor.header.mode_generation!=state.mode_generation ||
        !cursor::from_top_left(request.x,request.y,true,position)) {
        return ERROR_INVALID_PARAMETER;
    }
    uint32_t status=0;
    uint32_t observed_position=0;
    if (!register_write(&state,LIGHTNING_SCANOUT_REG_CURSOR_POSITION,position.packed) ||
        !register_read(&state,LIGHTNING_SCANOUT_REG_CURSOR_STATUS,status) || status!=LIGHTNING_SCANOUT_ACTIVE ||
        !register_read(&state,LIGHTNING_SCANOUT_REG_CURSOR_POSITION,observed_position) ||
        observed_position!=position.packed || !wait_cursor_visibility(state,true)) {
        increment_cursor_counter(state.cursor_statistics.failures);
        return ERROR_DEVICE_REINITIALIZATION_NEEDED;
    }
    state.cursor.header.x=position.left;
    state.cursor.header.y=position.top;
    state.cursor.header.visible=1;
    state.cursor_statistics.x=position.left;
    state.cursor_statistics.y=position.top;
    state.cursor_statistics.visible=1;
    increment_cursor_counter(state.cursor_statistics.moves);
    return NO_ERROR;
}

VP_STATUS define_cursor(VideoState& state,VIDEO_REQUEST_PACKET& request) noexcept {
    CursorShapeRequest header{};
    if (!read_video_value(request,header) || header.size!=request.InputBufferLength ||
        header.version!=cpu_scanout_abi_version ||
        header.mode_generation!=state.mode_generation ||
        header.size!=sizeof(header)+header.shape_bytes) return ERROR_INVALID_PARAMETER;
    cursor::Position position{};
    if (header.visible>1 || !cursor::from_top_left(header.x,header.y,header.visible!=0,position) ||
        !cursor::valid_shape({header.width,header.height,header.pitch,header.format,
                             header.hot_x,header.hot_y,header.shape_bytes})) {
        increment_cursor_counter(state.cursor_statistics.declines);
        return ERROR_INVALID_PARAMETER;
    }
    if (hide_cursor(state,false)!=NO_ERROR) return ERROR_DEVICE_REINITIALIZATION_NEEDED;
    state.cursor_statistics.defined=0;
    const ByteView input(static_cast<const uint8_t*>(request.InputBuffer),request.InputBufferLength);
    for (uint32_t offset=0;offset<header.shape_bytes;offset+=sizeof(uint32_t)) {
        uint32_t word=0;
        if (!input.read_u32(sizeof(header)+offset,word)) return ERROR_INVALID_PARAMETER;
        if (!register_write(&state,LIGHTNING_SCANOUT_CURSOR_SHAPE_OFFSET+offset,word)) {
            increment_cursor_counter(state.cursor_statistics.failures);
            return ERROR_DEVICE_REINITIALIZATION_NEEDED;
        }
    }
    const uint32_t offsets[]{LIGHTNING_SCANOUT_REG_CURSOR_WIDTH,LIGHTNING_SCANOUT_REG_CURSOR_HEIGHT,
        LIGHTNING_SCANOUT_REG_CURSOR_PITCH,LIGHTNING_SCANOUT_REG_CURSOR_HOT_X,
        LIGHTNING_SCANOUT_REG_CURSOR_HOT_Y,LIGHTNING_SCANOUT_REG_CURSOR_FORMAT};
    const uint32_t values[]{header.width,header.height,header.pitch,header.hot_x,header.hot_y,header.format};
    for (uint32_t i=0;i<RTL_NUMBER_OF(offsets);++i) {
        if (!register_write(&state,offsets[i],values[i])) {
            increment_cursor_counter(state.cursor_statistics.failures);
            return ERROR_DEVICE_REINITIALIZATION_NEEDED;
        }
    }
    uint32_t status=0;
    if (!register_write(&state,LIGHTNING_SCANOUT_REG_CURSOR_COMMAND,LIGHTNING_SCANOUT_CURSOR_DEFINE) ||
        !register_read(&state,LIGHTNING_SCANOUT_REG_CURSOR_STATUS,status)) {
        increment_cursor_counter(state.cursor_statistics.failures);
        return ERROR_DEVICE_REINITIALIZATION_NEEDED;
    }
    if (status!=LIGHTNING_SCANOUT_ACTIVE) {
        state.cursor.header={};
        increment_cursor_counter(state.cursor_statistics.declines);
        return hide_cursor(state,false)==NO_ERROR ? ERROR_INVALID_PARAMETER : ERROR_DEVICE_REINITIALIZATION_NEEDED;
    }
    state.cursor.header=header;
    // The validated buffered request owns these bytes for this synchronous
    // call. VideoPort's legacy PVOID source parameter only reads the buffer.
    VideoPortMoveMemory(state.cursor.pixels,static_cast<uint8_t*>(request.InputBuffer)+sizeof(header),header.shape_bytes);
    state.cursor_statistics.defined=1;
    state.cursor_statistics.format=header.format;
    state.cursor_statistics.width=header.width;
    state.cursor_statistics.height=header.height;
    state.cursor_statistics.hot_x=header.hot_x;
    state.cursor_statistics.hot_y=header.hot_y;
    increment_cursor_counter(state.cursor_statistics.definitions);
    const CursorMoveRequest movement{sizeof(CursorMoveRequest),cpu_scanout_abi_version,state.mode_generation,
                                     header.x,header.y,header.visible};
    const VP_STATUS moved=move_cursor(state,movement);
    if (moved!=NO_ERROR) {
        hide_cursor(state,false);
        state.cursor_statistics.defined=0;
        state.cursor.header={};
    }
    return moved;
}

VP_STATUS hide_cursor(VideoState& state,bool retain_visibility) noexcept {
    if (!register_write(&state,LIGHTNING_SCANOUT_REG_CURSOR_COMMAND,LIGHTNING_SCANOUT_CURSOR_HIDE) ||
        !wait_cursor_visibility(state,false)) {
        increment_cursor_counter(state.cursor_statistics.failures);
        return ERROR_DEVICE_REINITIALIZATION_NEEDED;
    }
    if (!retain_visibility) state.cursor.header.visible=0;
    state.cursor_statistics.visible=0;
    increment_cursor_counter(state.cursor_statistics.hides);
    return NO_ERROR;
}

VP_STATUS restore_cursor(VideoState& state) noexcept {
    if (state.cursor_statistics.defined==0) return NO_ERROR;
    if (state.cursor.header.mode_generation!=state.mode_generation) return ERROR_INVALID_PARAMETER;
    // Replay only our retained accepted bytes. The temporary request contains
    // no user pointer and remains under the native PASSIVE_LEVEL guard.
    VIDEO_REQUEST_PACKET request{};
    request.InputBuffer=&state.cursor;
    request.InputBufferLength=state.cursor.header.size;
    return define_cursor(state,request);
}
} // namespace

extern "C" VP_STATUS NTAPI lightning_find_adapter(PVOID extension,PVOID,PWSTR,
    PVIDEO_PORT_CONFIG_INFO config,PUCHAR again) {
    if (extension==nullptr || config==nullptr || again==nullptr ||
        config->Length<sizeof(*config) || config->AdapterInterfaceType!=PCIBus ||
        VideoPortGetCurrentIrql()!=PASSIVE_LEVEL) return ERROR_INVALID_PARAMETER;
    *again=FALSE;
    VideoState* storage=aligned_video_extension<VideoState>(extension);
    if (storage==nullptr) return ERROR_INVALID_PARAMETER;
    uint32_t marker=0;
    VideoPortMoveMemory(&marker,storage,sizeof(marker));
    VideoState* state=storage;
    if (marker==0) {
        state=rx7900::reactos::construct_device_state<VideoState>(storage);
        KeInitializeMutex(&state->lock,0);
#ifdef RX7900_LIGHTNING_GDI_OFFLOAD
        state->gpu_completion.initialize(extension);
#endif
        // Port/PnP serialization and the upper filter's START gate exclude
        // callback publication until construction and mutex initialization end.
        KeMemoryBarrier();
        state->signature=device_signature;
    } else if (marker!=device_signature || !state->quiesced) return ERROR_BUSY;
    Guard guard(*state);
    release_maps(*state,extension);
    state->lost=false;
#ifdef RX7900_LIGHTNING_GDI_OFFLOAD
    state->irq_configured=config->BusInterruptLevel!=0 || config->BusInterruptVector!=0;
#endif
    state->scanout=CpuScanout{};
    VIDEO_ACCESS_RANGE ranges[6]{};
    USHORT vendor=LIGHTNING_MEMORY_VENDOR_ID;
    USHORT device=LIGHTNING_MEMORY_DEVICE_ID;
    ULONG slot=0;
    VP_STATUS status=VideoPortGetAccessRanges(extension,0,nullptr,RTL_NUMBER_OF(ranges),ranges,&vendor,&device,&slot);
    if (status!=NO_ERROR) {
        DbgPrint("LightningVideo: assigned resource query failed %lu\n",status);
        return status;
    }
    DbgPrint("LightningVideo: bus %lu slot %lu assigned ranges\n",config->SystemIoBusNumber,slot);
    for (const auto& range:ranges) {
        DbgPrint("LightningVideo: range %08lx:%08lx length %08lx io %u\n",
            range.RangeStart.HighPart,range.RangeStart.LowPart,range.RangeLength,range.RangeInIoSpace);
    }
    uint8_t bytes[rx7900::pci_header_bytes]{};
    if (VideoPortGetBusData(extension,PCIConfiguration,slot,bytes,0,sizeof(bytes))!=sizeof(bytes)) {
        DbgPrint("LightningVideo: could not read assigned PCI header\n");
        return ERROR_DEV_NOT_EXIST;
    }
    const ByteView config_bytes(bytes,sizeof(bytes));
    uint16_t actual_vendor=0;
    uint16_t actual_device=0;
    uint8_t header_type=0;
    uint32_t bar0=0;
    uint32_t bar4=0;
    uint32_t bar5=0;
    uint64_t framebuffer_bus=0;
    if (!config_bytes.read_u16(rx7900::pci_vendor_offset,actual_vendor) ||
        !config_bytes.read_u16(rx7900::pci_device_offset,actual_device) ||
        !config_bytes.read_u8(rx7900::pci_header_type_offset,header_type) ||
        !config_bytes.read_u32(rx7900::pci_bars_offset,bar0) ||
        !config_bytes.read_u32(rx7900::pci_bars_offset+4*LIGHTNING_MEMORY_BAR_INDEX,bar4) ||
        !config_bytes.read_u32(rx7900::pci_bars_offset+4*(LIGHTNING_MEMORY_BAR_INDEX+1),bar5) ||
        actual_vendor!=vendor || actual_device!=device || (header_type&rx7900::pci_header_type_mask)!=0 ||
        (bar0&0xfU)!=0 || !framebuffer_bar_address(bar4,bar5,framebuffer_bus)) {
        DbgPrint("LightningVideo: rejected PCI identity %04x:%04x header %02x BAR0 %08lx BAR4 %08lx:%08lx\n",
            actual_vendor,actual_device,header_type,bar0,bar5,bar4);
        return ERROR_DEV_NOT_EXIST;
    }
    const uint64_t control_bus=bar0&~0xfU;
    if (control_bus==0 || framebuffer_bus==0) return ERROR_DEV_NOT_EXIST;
    VIDEO_ACCESS_RANGE* control=nullptr;
    VIDEO_ACCESS_RANGE* framebuffer=nullptr;
    for (auto& range:ranges) {
        if (range.RangeInIoSpace!=0 || range.RangeStart.QuadPart<0) continue;
        const uint64_t address=static_cast<uint64_t>(range.RangeStart.QuadPart);
        if (address==control_bus && range.RangeLength>=control_bytes) control=&range;
        if (address==framebuffer_bus &&
            framebuffer_bar_length_valid(bar4,range.RangeLength)) framebuffer=&range;
    }
    const uint32_t map_bytes=maximum_scanout_bytes();
    const uint64_t bus_end_limit=sizeof(void*)==4 ? 0x100000000ULL : UINT64_MAX;
    if (control==nullptr || framebuffer==nullptr || map_bytes==0 ||
        !valid_bus_span(framebuffer_bus,map_bytes,bus_end_limit)) {
        DbgPrint("LightningVideo: assigned BAR range mismatch control %p framebuffer %p bytes %lu\n",
            control,framebuffer,map_bytes);
        return ERROR_DEV_NOT_EXIST;
    }
    state->registers=static_cast<PULONG>(VideoPortGetDeviceBase(extension,control->RangeStart,control_bytes,
                                                              VIDEO_MEMORY_SPACE_MEMORY));
    state->framebuffer=VideoPortGetDeviceBase(extension,framebuffer->RangeStart,map_bytes,
        VIDEO_MEMORY_SPACE_MEMORY|VIDEO_MEMORY_SPACE_P6CACHE);
    if (state->registers==nullptr || state->framebuffer==nullptr) {
        release_maps(*state,extension);
        return ERROR_NOT_ENOUGH_MEMORY;
    }
    state->framebuffer_bytes=map_bytes;
#ifdef RX7900_LIGHTNING_GDI_OFFLOAD
    uint32_t gpu_magic=0;
    constexpr uint64_t transfer_span=static_cast<uint64_t>(LIGHTNING_GDI_TRANSFER_OFFSET)+
        LIGHTNING_GDI_TRANSFER_BYTES;
    if (map_bytes<=LIGHTNING_GDI_TRANSFER_OFFSET &&
        transfer_span<=framebuffer->RangeLength &&
        valid_bus_span(framebuffer_bus,transfer_span,bus_end_limit) &&
        register_read(state,LIGHTNING_GDI_REG_MAGIC,gpu_magic) && gpu_magic==LIGHTNING_GDI_MAGIC) {
        PHYSICAL_ADDRESS transfer_bus=framebuffer->RangeStart;
        transfer_bus.QuadPart+=LIGHTNING_GDI_TRANSFER_OFFSET;
        state->gpu_transfer=VideoPortGetDeviceBase(extension,transfer_bus,LIGHTNING_GDI_TRANSFER_BYTES,
            VIDEO_MEMORY_SPACE_MEMORY|VIDEO_MEMORY_SPACE_P6CACHE);
    }
    uint32_t completion_capabilities=0;
    if (state->gpu_transfer==nullptr ||
        !register_read(state,LIGHTNING_GDI_REG_CAPABILITIES,completion_capabilities) ||
        (completion_capabilities&LIGHTNING_GDI_CAP_COMPLETION_IRQ)==0) {
        // Older hosts retain the bounded polled path and need no unused ISR.
        // VideoPort connects its optional interrupt after FindAdapter returns.
        config->BusInterruptLevel=0;
        config->BusInterruptVector=0;
        state->irq_configured=false;
    }
#endif
    // Native cached RAM is deliberately distinct from the WC PCI mapping. GDI
    // keeps only this RAM lease, so an abandoned adapter receives no late writes.
    status=new_staging(*state);
    if (status!=NO_ERROR) {
        release_maps(*state,extension);
        return status;
    }
    status=port_status(state->scanout.initialize({state,register_read,register_write,clear_framebuffer,map_bytes}));
    if (status!=NO_ERROR) {
        uint32_t scanout_magic=0;
        uint32_t display_magic=0;
        uint32_t capabilities=0;
        register_read(state,LIGHTNING_SCANOUT_REG_MAGIC,scanout_magic);
        register_read(state,LIGHTNING_DISPLAY_REG_MAGIC,display_magic);
        register_read(state,LIGHTNING_DISPLAY_REG_CAPABILITIES,capabilities);
        DbgPrint("LightningVideo: scanout initialization failed %lu magic %08lx display %08lx caps %08lx\n",
            status,scanout_magic,display_magic,capabilities);
    }
    if (status!=NO_ERROR) release_maps(*state,extension);
    return status;
}

namespace {
void publish_hardware_metadata(PVOID extension,const VideoState& state) noexcept {
    // These describe the assigned virtual device and its mapped scanout RAM.
    // Neither the host Radeon model nor dedicated VRAM size is inferred here.
    wchar_t adapter_key[]=L"HardwareInformation.AdapterString";
    wchar_t chip_key[]=L"HardwareInformation.ChipType";
    wchar_t dac_key[]=L"HardwareInformation.DacType";
    wchar_t bios_key[]=L"HardwareInformation.BiosString";
    wchar_t memory_key[]=L"HardwareInformation.MemorySize";
    wchar_t adapter[]=L"QEMU Lightning virtual display";
    wchar_t chip[]=L"Lightning virtual PCI 1234:11F1";
    wchar_t dac[]=L"Virtual 32-bit XRGB scanout";
    wchar_t bios[]=L"QEMU virtual platform";
    ULONG scanout_bytes=state.framebuffer_bytes;
    const VP_STATUS results[]{
        VideoPortSetRegistryParameters(extension,adapter_key,adapter,sizeof(adapter)),
        VideoPortSetRegistryParameters(extension,chip_key,chip,sizeof(chip)),
        VideoPortSetRegistryParameters(extension,dac_key,dac,sizeof(dac)),
        VideoPortSetRegistryParameters(extension,bios_key,bios,sizeof(bios)),
        VideoPortSetRegistryParameters(extension,memory_key,&scanout_bytes,sizeof(scanout_bytes))
    };
    for (const auto result:results) {
        if (result!=NO_ERROR) DbgPrint("LightningVideo: hardware metadata write failed %lu\n",result);
    }
}
} // namespace

extern "C" BOOLEAN NTAPI lightning_initialize(PVOID extension) {
    VideoState* state=constructed_state(extension);
    if (state==nullptr || VideoPortGetCurrentIrql()!=PASSIVE_LEVEL) return FALSE;
    Guard guard(*state);
#ifdef RX7900_LIGHTNING_GDI_OFFLOAD
    // VideoPort connects HwInterrupt after FindAdapter and before Initialize.
    // A host/PCI configuration with no IRQ leaves completion safely polled.
    state->irq_connected=state->irq_configured;
#endif
    if (state->lost || state->metadata_only || !state->scanout.ready()) return FALSE;
    // VideoPort invokes HwInitialize on each OPEN. Diagnostic opens must not
    // clear an existing desktop or take ownership back from AssertMode(FALSE).
    if (state->initialized) return TRUE;
    if (set_mode(*state,initial_mode,false)!=NO_ERROR) {
        release_maps(*state,extension);
        return FALSE;
    }
    state->initialized=true;
    publish_hardware_metadata(extension,*state);
    return TRUE;
}

namespace {
constexpr ULONG staging_pool_tag=0x73564c51U;

void reclaim_staging(VideoState& state) noexcept {
    while (void* retired=state.staging.reclaim()) ExFreePoolWithTag(retired,staging_pool_tag);
}

#ifdef RX7900_LIGHTNING_LEASE_FAULTS
VP_STATUS NTAPI read_lease_fault_plan(PVOID,PVOID context,PWSTR,PVOID data,ULONG bytes) {
    uint32_t packed=0;
    if (data==nullptr || bytes!=sizeof(packed)) return ERROR_INVALID_PARAMETER;
    VideoPortMoveMemory(&packed,data,sizeof(packed));
    static_cast<StagingFaultPlan*>(context)->configure(packed);
    return NO_ERROR;
}
#endif

// Release-or-confirm by generation. Served before the LOST/initialized gate:
// after surprise removal it is the path by which the display client retires
// leases once it has moved its views off them. The mutex is held; the input
// is captured before METHOD_BUFFERED output reuses the system buffer.
VP_STATUS resolve_owned_staging(VideoState& state,PVOID extension,VIDEO_REQUEST_PACKET& request) noexcept {
    StagingResolveRequest input{};
    if (!read_video_value(request,input) ||
        !valid_staging_resolve_request(input,request.InputBufferLength) ||
        !rx7900::reactos::has_video_output(request,sizeof(StagingResolveReceipt))) return ERROR_INVALID_PARAMETER;
#ifdef RX7900_LIGHTNING_LEASE_FAULTS
    if (!state.lease_faults_loaded) {
        wchar_t name[]=L"LeaseFaultPlan";
        state.lease_faults_loaded=true;
        VideoPortGetRegistryParameters(extension,name,FALSE,read_lease_fault_plan,&state.lease_faults);
    }
    const StagingFault fault=state.lease_faults.next(input.operation);
    if (fault==StagingFault::drop_before_apply) {
        DbgPrint("LightningVideo: lease fault drop-before-apply generation %lu injected %lu\n",
            input.lease_generation,state.lease_faults.injected());
        return ERROR_BUSY;
    }
#else
    (void)extension;
#endif
    const StagingResolveReceipt receipt=resolve_staging(state.staging,input,state.lost);
    reclaim_staging(state);
    if (input.operation==staging_resolve_release || state.lost) {
        DbgPrint("LightningVideo: lease resolve generation %lu operation %lu outcome %lu held %lu issued %lu lost %u\n",
            receipt.lease_generation,input.operation,receipt.outcome,receipt.held_leases,
            receipt.issued_generation,state.lost ? 1U : 0U);
    }
#ifdef RX7900_LIGHTNING_LEASE_FAULTS
    if (fault==StagingFault::lost_after_apply) {
        DbgPrint("LightningVideo: lease fault lost-ack-after-apply generation %lu injected %lu\n",
            input.lease_generation,state.lease_faults.injected());
        return ERROR_DEVICE_REINITIALIZATION_NEEDED;
    }
#endif
    return return_video_value(request,receipt);
}
} // namespace

extern "C" BOOLEAN NTAPI lightning_start_io(PVOID extension,PVIDEO_REQUEST_PACKET request) {
    VideoState* state=constructed_state(extension);
    if (request==nullptr || request->StatusBlock==nullptr) return FALSE;
    request->StatusBlock->Information=0;
    if (state==nullptr) {
        request->StatusBlock->Status=ERROR_DEV_NOT_EXIST;
        return TRUE;
    }
    if (VideoPortGetCurrentIrql()!=PASSIVE_LEVEL) {
        request->StatusBlock->Status=ERROR_INVALID_FUNCTION;
        return TRUE;
    }
    Guard guard(*state);
    VP_STATUS status=ERROR_INVALID_FUNCTION;
    // LOST is a software-only interlock, valid even before HwInitialize. Unmap
    // also remains legal after loss so the final RAM lease can be reclaimed.
    if (request->IoControlCode==ioctl_abandon_cpu_scanout) {
#ifdef RX7900_LIGHTNING_GDI_OFFLOAD
        state->gpu_completion.disable(false);
        state->gpu.abandon();
#endif
        state->lost=true;
        state->quiesced=true;
        state->cursor_statistics.visible=0;
        state->cursor_statistics.defined=0;
        // Native scanout and the GPU canvas no longer read staging. RAM no
        // client holds is provably unused and is freed now; leased RAM waits
        // for its display client to move its views off and resolve-release.
        state->staging.retire();
        reclaim_staging(*state);
        DbgPrint("LightningVideo: abandon retains %lu leased staging allocations\n",state->staging.held_leases());
        request->StatusBlock->Status=NO_ERROR;
        return TRUE;
    }
    if (request->IoControlCode==ioctl_resolve_owned_staging) {
        request->StatusBlock->Status=resolve_owned_staging(*state,extension,*request);
        return TRUE;
    }
    if (request->IoControlCode==IOCTL_VIDEO_UNMAP_VIDEO_MEMORY) {
        VIDEO_MEMORY input{};
        status=read_video_value(*request,input) && state->staging.release(input.RequestedVirtualAddress) ?
            NO_ERROR : ERROR_INVALID_PARAMETER;
        while (void* retired=state->staging.reclaim()) ExFreePoolWithTag(retired,0x73564c51U);
        request->StatusBlock->Status=status;
        return TRUE;
    }
    if (request->IoControlCode==ioctl_release_owned_staging) {
        StagingReleaseRequest input{};
        status=request->InputBufferLength==sizeof(input) && request->OutputBufferLength==0 &&
            read_video_value(*request,input) && input.size==sizeof(input) &&
            input.version==cpu_scanout_abi_version && input.reserved==0 &&
            state->staging.release_generation(input.lease_generation) ? NO_ERROR : ERROR_INVALID_PARAMETER;
        while (void* retired=state->staging.reclaim()) ExFreePoolWithTag(retired,0x73564c51U);
        request->StatusBlock->Status=status;
        return TRUE;
    }
    if (request->IoControlCode==ioctl_query_cursor) {
        CursorStatistics statistics=state->cursor_statistics;
        statistics.size=sizeof(statistics);
        statistics.version=cpu_scanout_abi_version;
        statistics.magic=cursor_statistics_magic;
        statistics.mode_generation=state->mode_generation;
        statistics.hardware_active=0;
        statistics.recovery_required=0;
        statistics.reserved=0;
        request->StatusBlock->Status=return_video_value(*request,statistics);
        return TRUE;
    }
    // PnP START precedes the first VideoPort OPEN/HwInitialize. The UpperFilter
    // must recognize its lower device without prematurely initializing scanout.
    if (!state->lost && state->scanout.ready() && request->IoControlCode==ioctl_query_cpu_scanout) {
        request->StatusBlock->Status=query_information(*state,*request);
        return TRUE;
    }
    if (!state->lost && state->scanout.ready() && !state->initialized &&
        (request->IoControlCode==ioctl_quiesce_cpu_scanout || request->IoControlCode==ioctl_resume_cpu_scanout)) {
        request->StatusBlock->Status=preinit_power(*state,request->IoControlCode==ioctl_resume_cpu_scanout);
        return TRUE;
    }
    if (!state->initialized || state->lost) {
        request->StatusBlock->Status=ERROR_DEV_NOT_EXIST;
        return TRUE;
    }
    switch (request->IoControlCode) {
    case ioctl_query_owned_staging: {
        VIDEO_MEMORY input{};
        if (request->InputBufferLength!=sizeof(input) || !read_video_value(*request,input) ||
            state->quiesced || state->mode_generation==0 ||
            state->staging.current_mapping_generation(input.RequestedVirtualAddress)!=state->mode_generation) {
            status=ERROR_INVALID_PARAMETER;
            break;
        }
        // The trusted kernel display caller already received this address
        // through MAP. Equality authenticates its lease; no address is read.
        status=query_information(*state,*request);
        break;
    }
#ifdef RX7900_LIGHTNING_GDI_OFFLOAD
    case lightning::reactos::gdi::ioctl_query: {
        // A quiesce that reset the canvas (an owned yield) still answers the
        // coherence query: the reset epoch proves earlier async reads retired,
        // so the filter can settle receipts between PDEVs. A quiesced canvas
        // that still owns work, and every full query, stay unavailable.
        if (state->quiesced && (request->InputBufferLength==0 || state->gpu.has_canvas() ||
            state->gpu.quarantined())) {status=ERROR_DEV_NOT_EXIST; break;}
        bool coherence_only=false;
        // Capture optional input before METHOD_BUFFERED output replaces the
        // shared system buffer. Empty input preserves the original full query.
        if (request->InputBufferLength!=0) {
            lightning::reactos::gdi::QueryRequest query{};
            if (request->InputBufferLength!=sizeof(query) || !read_video_value(*request,query) ||
                query.size!=sizeof(query) || query.version!=lightning::reactos::gdi::native_abi_version ||
                (query.flags!=lightning::reactos::gdi::query_coherence_only &&
                 // async-present: accepting this flag advertises version-2
                 // external images (NativeCanvas::submit); the answer is unchanged.
                 query.flags!=(lightning::reactos::gdi::query_coherence_only|
                     lightning::reactos::gdi::query_async_external)) || query.reserved!=0) {
                status=ERROR_INVALID_PARAMETER;
                break;
            }
            coherence_only=true;
        }
        lightning::reactos::gdi::Information information{};
        status=state->gpu.query(gpu_view(*state),information,coherence_only) ?
            return_video_value(*request,information) : ERROR_INVALID_FUNCTION;
        break;
    }
    case lightning::reactos::gdi::ioctl_submit: {
        lightning::reactos::gdi::Receipt receipt{};
        if (state->quiesced || !rx7900::reactos::has_video_output(*request,sizeof(receipt))) {
            status=state->quiesced ? ERROR_DEV_NOT_EXIST : ERROR_INSUFFICIENT_BUFFER;
            break;
        }
        state->gpu.submit(gpu_view(*state),ByteView(static_cast<const uint8_t*>(request->InputBuffer),
            request->InputBufferLength),receipt);
        status=return_video_value(*request,receipt);
        break;
    }
    case lightning::reactos::gdi::ioctl_wait: {
        if (state->quiesced) {status=ERROR_DEV_NOT_EXIST; break;}
        lightning::reactos::gdi::WaitRequest wait{};
        lightning::reactos::gdi::Receipt receipt{};
        if (!read_video_value(*request,wait) || !rx7900::reactos::has_video_output(*request,sizeof(receipt))) {
            status=ERROR_INVALID_PARAMETER;
            break;
        }
        state->gpu.wait(gpu_view(*state),wait,receipt);
        status=return_video_value(*request,receipt);
        break;
    }
    case lightning::reactos::gdi::ioctl_restore: {
        lightning::reactos::gdi::RestoreRequest restore{};
        VirtualMode mode{};
        if (state->quiesced || !read_video_value(*request,restore) ||
            request->OutputBufferLength!=0 || restore.size!=sizeof(restore) ||
            restore.version!=lightning::reactos::gdi::native_abi_version || restore.reserved!=0 ||
            restore.mode_generation!=state->mode_generation || !state->gpu_snapshot ||
            state->snapshot_generation!=state->mode_generation || state->gpu.has_canvas() ||
            state->gpu.quarantined() || virtual_mode(state->desired_mode,mode)!=ScanoutStatus::success ||
            state->framebuffer==nullptr || state->staging.current()==nullptr) {
            status=ERROR_DEVICE_REINITIALIZATION_NEEDED; break;
        }
        const uint64_t bytes=static_cast<uint64_t>(mode.width)*mode.height*LIGHTNING_SCANOUT_PIXEL_BYTES;
        if (bytes>state->framebuffer_bytes || bytes>state->staging.current_bytes()) {
            status=ERROR_INVALID_PARAMETER; break;
        }
        KeMemoryBarrier();
        VideoPortMoveMemory(state->staging.current(),state->framebuffer,static_cast<ULONG>(bytes));
        status=NO_ERROR;
        break;
    }
#endif
    case IOCTL_VIDEO_QUERY_NUM_AVAIL_MODES: {
        VIDEO_NUM_MODES modes{LIGHTNING_SCANOUT_MODE_COUNT,sizeof(VIDEO_MODE_INFORMATION)};
        status=return_video_value(*request,modes);
        break;
    }
    case IOCTL_VIDEO_QUERY_AVAIL_MODES: {
        const ULONG required=LIGHTNING_SCANOUT_MODE_COUNT*sizeof(VIDEO_MODE_INFORMATION);
        if (!rx7900::reactos::has_video_output(*request,required)) {status=ERROR_INSUFFICIENT_BUFFER; break;}
        status=NO_ERROR;
        for (uint32_t i=0;i<LIGHTNING_SCANOUT_MODE_COUNT;++i) {
            VIDEO_MODE_INFORMATION mode{};
            status=mode_description(*state,i,mode);
            if (status!=NO_ERROR) break;
            VideoPortMoveMemory(static_cast<UCHAR*>(request->OutputBuffer)+i*sizeof(mode),&mode,sizeof(mode));
        }
        if (status==NO_ERROR) request->StatusBlock->Information=required;
        break;
    }
    case IOCTL_VIDEO_QUERY_CURRENT_MODE: {
        VIDEO_MODE_INFORMATION mode{};
        uint32_t current=0;
        status=port_status(state->scanout.current_mode(current));
        if (status==NO_ERROR) status=mode_description(*state,current,mode);
        if (status==NO_ERROR) status=return_video_value(*request,mode);
        break;
    }
    case IOCTL_VIDEO_SET_CURRENT_MODE: {
        rx7900::display::ModeRequest mode{};
        const auto decoded=rx7900::display::decode_video_mode_request(
            ByteView(static_cast<const uint8_t*>(request->InputBuffer),request->InputBufferLength),mode);
        if (decoded!=rx7900::display::Status::success) {status=ERROR_INVALID_PARAMETER; break;}
        // framebuf DrvEnableSurface's credential-free first request. It never
        // displaces a mapped owner that has not yielded scanout.
        if (!initial_mode_authorized(state->staging,state->mode_generation,state->yield,state->quiesced)) {
            DbgPrint("LightningVideo: refused SDK mode set over mapped owner lease %lu generation %lu\n",
                state->staging.current_generation(),state->mode_generation);
            status=ERROR_BUSY;
            break;
        }
        status=set_mode(*state,mode.mode_index,mode.preserve_contents);
        if (status==NO_ERROR) state->initial_map=state->mode_generation;
        break;
    }
    case ioctl_transition_owned_mode: {
        ModeTransitionRequest transition{};
        if (request->InputBufferLength!=sizeof(transition) ||
            !read_video_value(*request,transition) ||
            !rx7900::reactos::has_video_output(*request,sizeof(ModeTransitionReceipt)) ||
            !mode_transition_authorized(state->staging,state->mode_generation,
                state->rollback,transition) ||
            (transition.enable==0 && transition.owner.mode_index!=state->desired_mode)) {
            status=ERROR_INVALID_PARAMETER;
            break;
        }
        const VP_STATUS changed=transition.enable!=0 ?
            set_mode(*state,transition.owner.mode_index,false,false,transition.owner.lease_generation) :
            quiesce(*state);
        // Only the mapped owner's own successful disable yields scanout to a
        // preauthenticated successor; RESET_DEVICE and power quiesce never do.
        if (transition.enable==0 && changed==NO_ERROR) {
            state->yield={transition.owner.lease_generation,state->mode_generation};
        }
        const ModeTransitionReceipt receipt{sizeof(ModeTransitionReceipt),cpu_scanout_abi_version,
            transition.owner.mode_generation,state->mode_generation,state->desired_mode,
            static_cast<uint32_t>(changed),state->quiesced ? 0U : 1U,0};
        status=return_video_value(*request,receipt);
        break;
    }
    case ioctl_map_owned_staging: {
        ModeOwnerRequest owner{};
        VIDEO_MEMORY_INFORMATION output{};
        if (request->InputBufferLength!=sizeof(owner) || !read_video_value(*request,owner) ||
            !valid_mode_owner(owner,sizeof(owner)) || owner.mode_generation!=state->mode_generation ||
            owner.mode_index!=state->desired_mode ||
            !current_mode_lease_authorized(state->staging,state->rollback,owner.lease_generation) ||
            !rx7900::reactos::has_video_output(*request,sizeof(output))) {
            status=ERROR_INVALID_PARAMETER;
            break;
        }
        status=map_staging(*state,output,owner.lease_generation);
        if (status==NO_ERROR) status=return_video_value(*request,output);
        break;
    }
    case IOCTL_VIDEO_MAP_VIDEO_MEMORY: {
        VIDEO_MEMORY input{};
        VIDEO_MEMORY_INFORMATION output{};
        if (!read_video_value(*request,input) || input.RequestedVirtualAddress!=nullptr) {status=ERROR_INVALID_PARAMETER; break;}
        if (!rx7900::reactos::has_video_output(*request,sizeof(output))) {status=ERROR_INSUFFICIENT_BUFFER; break;}
        // Second half of the preauthenticated framebuf pair; the ticket is
        // consumed even when this MAP fails, so it can never be replayed.
        if (!initial_map_authorized(state->initial_map,state->mode_generation,state->quiesced)) {
            DbgPrint("LightningVideo: refused SDK map without its preauthenticated mode set\n");
            status=ERROR_INVALID_PARAMETER;
            break;
        }
        state->initial_map=0;
        status=map_staging(*state,output,0,true);
        if (status==NO_ERROR) status=return_video_value(*request,output);
        break;
    }

    case IOCTL_VIDEO_RESET_DEVICE:
    case ioctl_quiesce_cpu_scanout: status=quiesce(*state); break;
    case ioctl_resume_cpu_scanout: status=resume_scanout(*state); break;
    case ioctl_wait_cpu_scanout:
        status=state->quiesced ? (state->last_completion.published ? NO_ERROR : ERROR_DEV_NOT_EXIST) :
            wait_publication(*state);
        break;
    case ioctl_query_cpu_scanout: status=query_information(*state,*request); break;
    case ioctl_present_cpu_scanout:
        status=state->quiesced ? ERROR_DEV_NOT_EXIST : submit_damage(*state,*request);
        break;
    case ioctl_cursor_shape:
        status=state->quiesced ? ERROR_DEV_NOT_EXIST : define_cursor(*state,*request);
        break;
    case ioctl_cursor_move: {
        CursorMoveRequest move{};
        status=!state->quiesced && read_video_value(*request,move) ? move_cursor(*state,move) : ERROR_INVALID_PARAMETER;
        break;
    }
    default: break;
    }
    request->StatusBlock->Status=status;
    return TRUE;
}

extern "C" BOOLEAN NTAPI lightning_reset_hw(PVOID,ULONG,ULONG) {
    // Bugcheck/DIRQL cannot wait for paced console publication. Decline the
    // unsupported text-console restore instead of claiming an uncompleted reset.
    return FALSE;
}

#ifdef RX7900_LIGHTNING_GDI_OFFLOAD
extern "C" BOOLEAN NTAPI lightning_interrupt(PVOID extension) {
    VideoState* state=constructed_state(extension);
    return state!=nullptr && state->gpu_completion.interrupt() ? TRUE : FALSE;
}
#endif

extern "C" VP_STATUS NTAPI lightning_get_power(PVOID,ULONG,PVIDEO_POWER_MANAGEMENT power) {
    if (power==nullptr || power->Length<sizeof(*power)) return ERROR_INVALID_PARAMETER;
    switch (power->PowerState) {
    case VideoPowerOn:
    case VideoPowerStandBy:
    case VideoPowerSuspend:
    case VideoPowerOff:
    case VideoPowerHibernate:
    case VideoPowerShutdown:
        return NO_ERROR;
    default:
        return ERROR_INVALID_FUNCTION;
    }
}

extern "C" VP_STATUS NTAPI lightning_set_power(PVOID extension,ULONG,PVIDEO_POWER_MANAGEMENT power) {
    VideoState* state=constructed_state(extension);
    if (state==nullptr || lightning_get_power(extension,0,power)!=NO_ERROR ||
        VideoPortGetCurrentIrql()!=PASSIVE_LEVEL) return ERROR_INVALID_PARAMETER;
    Guard guard(*state);
    if (state->lost) return ERROR_DEV_NOT_EXIST;
    if (!state->initialized) return preinit_power(*state,power->PowerState==VideoPowerOn);
    // The lower PCI stack owns electrical power. All virtual display low-power
    // states publish a blank and retain staged RAM until the lower D0 resume.
    return power->PowerState==VideoPowerOn ? resume_scanout(*state) : quiesce(*state);
}

extern "C" VP_STATUS NTAPI lightning_child(PVOID,PVIDEO_CHILD_ENUM_INFO information,
    PVIDEO_CHILD_TYPE type,PUCHAR descriptor,PULONG id,PULONG unused) {
    if (information==nullptr || type==nullptr || id==nullptr || unused==nullptr ||
        information->Size<sizeof(*information)) return VIDEO_ENUM_INVALID_DEVICE;
    if (information->ChildIndex==0) return VIDEO_ENUM_INVALID_DEVICE;
    if (information->ChildIndex==DISPLAY_ADAPTER_HW_ID) {*type=VideoChip; *unused=0; return VIDEO_ENUM_MORE_DEVICES;}
    if (information->ChildIndex!=1) return VIDEO_ENUM_NO_MORE_DEVICES;
    *type=Monitor;
    *id=LIGHTNING_SCANOUT_TARGET_ID;
    *unused=0;
    if (descriptor!=nullptr && information->ChildDescriptorSize!=0) {
        VideoPortZeroMemory(descriptor,information->ChildDescriptorSize);
    }
    return VIDEO_ENUM_MORE_DEVICES;
}

extern "C" ULONG NTAPI DriverEntry(PVOID context1,PVOID context2) {
    VIDEO_HW_INITIALIZATION_DATA initialization{};
    initialization.HwInitDataSize=sizeof(initialization);
    initialization.AdapterInterfaceType=PCIBus;
    initialization.HwFindAdapter=lightning_find_adapter;
    initialization.HwInitialize=lightning_initialize;
    initialization.HwStartIO=lightning_start_io;
#ifdef RX7900_LIGHTNING_GDI_OFFLOAD
    initialization.HwInterrupt=lightning_interrupt;
#endif
    initialization.HwResetHw=lightning_reset_hw;
    initialization.HwGetPowerState=lightning_get_power;
    initialization.HwSetPowerState=lightning_set_power;
    initialization.HwGetVideoChildDescriptor=lightning_child;
    initialization.HwDeviceExtensionSize=rx7900::reactos::video_extension_bytes<VideoState>;
    return VideoPortInitialize(context1,context2,&initialization,nullptr);
}

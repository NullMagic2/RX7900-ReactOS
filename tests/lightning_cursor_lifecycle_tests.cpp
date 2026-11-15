/*
Module: lightning.tests.cursor_lifecycle
Purpose: Exercise the production display client's retained cursor lifecycle.
Created: 2026-10-21
Architecture: Real ReactOS SDK and lightning_display.cpp; fake engine/IO boundary.
The boundary fixture does not implement the native miniport or GTK publication.
*/
#include "framebuf.h"
#include "lightning_display.hpp"
#include "lightning-display.h"
#include "lightning_engine_fixture.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace {
using namespace lightning::reactos;
unsigned checks=0;
unsigned failures=0;
using namespace lightning::tests::engine;

void expect(bool condition,const char* description) {
    ++checks;
    if (!condition) {++failures; printf("FAIL %s\n",description);}
}

struct Boundary final {
    uint32_t generation=7;
    uint32_t lease_generation=7;
    uint32_t mode_index=1;
    bool enabled=true;
    bool fail_move=false;
    bool fail_hide=false;
    bool invalid_query=false;
    bool fail_transition=false;
    bool fail_mapping=false;
    bool malformed_mapping=false;
    bool invalid_owner_information=false;
    bool fail_release=false;
    PVOID current_mapping=reinterpret_cast<PVOID>(static_cast<uintptr_t>(0x1000));
    unsigned transitions=0;
    unsigned mappings=0;
    unsigned presents=0;
    unsigned waits=0;
    unsigned stale_writes=0;
    unsigned releases=0;
    uint32_t released_generation=0;
    CursorShapeSnapshot shape{};
    CursorStatistics statistics{};
    // Resolve-capable paired miniport: exact issued leases by generation.
    bool resolve_enabled=false;
    bool lost=false;
    bool resolve_fail=false;
    bool resolve_drop_next=false;
    bool resolve_lost_next=false;
    bool resolve_reorder_next=false;
    bool map_ack_lost_next=false;
    bool map_without_lease=false;
    bool map_foreign_initiator=false;
    uint32_t issued=0;
    uint32_t held[8]{};
    uint32_t held_initiator[8]{};
    unsigned resolve_requests=0;
    unsigned resolve_releases=0;
    unsigned legacy_attempts=0;
    unsigned lost_presents=0;
} boundary;

int held_index(uint32_t generation) {
    for (int i=0;i<8;++i) {
        if (generation!=0 && boundary.held[i]==generation) return i;
    }
    return -1;
}

void hold(uint32_t generation,uint32_t initiator) {
    for (int i=0;i<8;++i) {
        if (boundary.held[i]==0) {boundary.held[i]=generation; boundary.held_initiator[i]=initiator; break;}
    }
    if (generation>boundary.issued) boundary.issued=generation;
}

unsigned held_count() {
    unsigned count=0;
    for (const auto generation:boundary.held) count+=generation!=0 ? 1U : 0U;
    return count;
}

template<typename T>
DWORD output_value(PVOID output,DWORD bytes,LPDWORD returned,const T& value) {
    if (output==nullptr || bytes!=sizeof(value)) return 1;
    memcpy(output,&value,sizeof(value));
    *returned=sizeof(value);
    return 0;
}
} // namespace

extern "C" DWORD APIENTRY EngDeviceIoControl(HANDLE,DWORD code,PVOID input,DWORD bytes,
    PVOID output,DWORD output_bytes,LPDWORD returned) {
    *returned=0;
    if (boundary.lost && code!=ioctl_resolve_owned_staging && code!=ioctl_release_owned_staging &&
        code!=ioctl_query_cursor) {
        if (code==ioctl_present_cpu_scanout) ++boundary.lost_presents;
        return 1;
    }
    if (code==ioctl_query_cpu_scanout || code==ioctl_query_owned_staging) {
        if (code==ioctl_query_owned_staging) {
            if (input==nullptr || bytes!=sizeof(VIDEO_MEMORY)) return 1;
            VIDEO_MEMORY mapping{};
            memcpy(&mapping,input,sizeof(mapping));
            if (mapping.RequestedVirtualAddress==nullptr || mapping.RequestedVirtualAddress!=boundary.current_mapping ||
                boundary.lease_generation!=boundary.generation) return 1;
        }
        CpuScanoutInformation information{};
        information.size=sizeof(information);
        information.version=cpu_scanout_abi_version;
        information.magic=boundary.invalid_owner_information ? 0U : LIGHTNING_DISPLAY_MAGIC;
        information.width=1920;
        information.height=1080;
        information.stride=1920*4;
        information.mapped_bytes=1920*1080*4;
        information.capabilities=LIGHTNING_DISPLAY_CAPABILITIES;
        information.mode_generation=boundary.generation;
        information.enabled=boundary.enabled ? 1U : 0U;
        return output_value(output,output_bytes,returned,information);
    }
    if (code==ioctl_query_cursor) {
        auto statistics=boundary.statistics;
        statistics.size=sizeof(statistics);
        statistics.version=cpu_scanout_abi_version;
        statistics.magic=boundary.invalid_query ? 0U : cursor_statistics_magic;
        statistics.mode_generation=boundary.generation;
        return output_value(output,output_bytes,returned,statistics);
    }
    if (code==ioctl_wait_cpu_scanout) {++boundary.waits; return 0;}
    if (code==ioctl_resolve_owned_staging) {
        if (!boundary.resolve_enabled || input==nullptr || bytes!=sizeof(StagingResolveRequest)) return 1;
        StagingResolveRequest request{};
        memcpy(&request,input,sizeof(request));
        ++boundary.resolve_requests;
        if (boundary.resolve_fail || !valid_staging_resolve_request(request,bytes)) return 1;
        if (request.operation==staging_resolve_release && boundary.resolve_drop_next) {
            boundary.resolve_drop_next=false;
            return 1;
        }
        StagingResolveReceipt receipt{sizeof(receipt),cpu_scanout_abi_version,request.lease_generation,0,
            boundary.lost ? staging_flag_device_lost : 0U,boundary.issued,0,0};
        const int index=held_index(request.lease_generation);
        if (request.lease_generation>boundary.issued) {
            receipt.outcome=staging_lease_unknown;
        } else if (index<0) {
            receipt.outcome=staging_lease_not_held;
        } else if (request.operation==staging_resolve_release) {
            boundary.held[index]=0;
            ++boundary.resolve_releases;
            receipt.outcome=staging_lease_released;
        } else {
            receipt.outcome=staging_lease_held;
            if (request.initiator_lease!=0 && boundary.held_initiator[index]==request.initiator_lease)
                receipt.flags|=staging_flag_initiator;
        }
        receipt.held_leases=held_count();
        if (request.operation==staging_resolve_release && boundary.resolve_lost_next) {
            boundary.resolve_lost_next=false;
            return 1;
        }
        if (boundary.resolve_reorder_next) {
            boundary.resolve_reorder_next=false;
            receipt.lease_generation=request.lease_generation+1U;
        }
        return output_value(output,output_bytes,returned,receipt);
    }
    if (code==ioctl_release_owned_staging) {
        ++boundary.legacy_attempts;
        if (input==nullptr || bytes!=sizeof(StagingReleaseRequest) || output_bytes!=0) return 1;
        StagingReleaseRequest request{};
        memcpy(&request,input,sizeof(request));
        if (request.size!=sizeof(request) || request.version!=cpu_scanout_abi_version ||
            request.lease_generation==0 || request.reserved!=0 || boundary.fail_release) return 1;
        if (boundary.resolve_enabled) {
            // The real miniport's legacy path finds no lease after an applied release.
            const int index=held_index(request.lease_generation);
            if (index<0) return 1;
            boundary.held[index]=0;
        }
        ++boundary.releases;
        boundary.released_generation=request.lease_generation;
        return 0;
    }
    if (code==ioctl_transition_owned_mode) {
        if (input==nullptr || bytes!=sizeof(ModeTransitionRequest)) return 1;
        ModeTransitionRequest request{};
        memcpy(&request,input,sizeof(request));
        if (request.owner.size!=sizeof(request) || request.owner.version!=cpu_scanout_abi_version ||
            request.owner.mode_generation!=boundary.generation ||
            request.owner.lease_generation!=boundary.lease_generation || request.enable>1 || request.reserved!=0) {
            ++boundary.stale_writes;
            return 1;
        }
        ++boundary.transitions;
        if (request.enable!=0) ++boundary.generation;
        boundary.mode_index=request.owner.mode_index;
        boundary.enabled=request.enable!=0 && !boundary.fail_transition;
        const ModeTransitionReceipt receipt{sizeof(ModeTransitionReceipt),cpu_scanout_abi_version,
            request.owner.mode_generation,boundary.generation,boundary.mode_index,
            boundary.fail_transition ? 123U : 0U,boundary.enabled ? 1U : 0U,0};
        return output_value(output,output_bytes,returned,receipt);
    }
    if (code==ioctl_map_owned_staging) {
        if (input==nullptr || bytes!=sizeof(ModeOwnerRequest)) return 1;
        ModeOwnerRequest request{};
        memcpy(&request,input,sizeof(request));
        // Resolve mode mirrors current_mode_lease_authorized: an unleased
        // current slot falls back to the initiator's retained rollback lease.
        const bool lease_authorized=!boundary.resolve_enabled ?
            request.lease_generation==boundary.lease_generation :
            (held_index(boundary.lease_generation)>=0 ? request.lease_generation==boundary.lease_generation :
                held_index(request.lease_generation)>=0);
        if (request.size!=sizeof(request) || request.version!=cpu_scanout_abi_version ||
            request.mode_generation!=boundary.generation || request.mode_index!=boundary.mode_index ||
            !lease_authorized || !boundary.enabled || boundary.fail_mapping) return 1;
        ++boundary.generation;
        boundary.lease_generation=boundary.generation;
        ++boundary.mappings;
        VIDEO_MEMORY_INFORMATION mapping{};
        boundary.current_mapping=reinterpret_cast<PVOID>(static_cast<uintptr_t>(
            boundary.resolve_enabled ? 0x10000U+0x1000U*boundary.mappings : 0x2000U));
        if (boundary.resolve_enabled && !boundary.map_without_lease)
            hold(boundary.generation,boundary.map_foreign_initiator ? 0xdeadU : request.lease_generation);
        if (boundary.resolve_enabled && boundary.map_without_lease) {
            // An owner that answered MAP without issuing any lease.
            boundary.issued=boundary.generation-1U;
        }
        if (boundary.map_ack_lost_next) {
            boundary.map_ack_lost_next=false;
            return 1;
        }
        mapping.VideoRamBase=boundary.current_mapping;
        mapping.VideoRamLength=1920*1080*4;
        mapping.FrameBufferBase=boundary.malformed_mapping ? nullptr : mapping.VideoRamBase;
        mapping.FrameBufferLength=mapping.VideoRamLength;
        return output_value(output,output_bytes,returned,mapping);
    }
    if (code==ioctl_present_cpu_scanout) {
        if (input==nullptr || bytes!=sizeof(DamageRequest)+sizeof(LightningDisplayRect) ||
            output_bytes!=0 || !boundary.enabled) return 1;
        DamageRequest request{};
        memcpy(&request,input,sizeof(request));
        if (request.size!=bytes || request.version!=cpu_scanout_abi_version ||
            request.mode_generation!=boundary.generation || request.rectangle_count!=1) {
            ++boundary.stale_writes;
            return 1;
        }
        ++boundary.presents;
        return 0;
    }
    if (!boundary.enabled || input==nullptr) return 1;
    if (code==ioctl_cursor_shape) {
        if (bytes<sizeof(CursorShapeRequest) || bytes>sizeof(boundary.shape)) return 1;
        CursorShapeRequest header{};
        memcpy(&header,input,sizeof(header));
        if (header.mode_generation!=boundary.generation) {++boundary.stale_writes; return 1;}
        if (header.size!=bytes) return 1;
        memcpy(&boundary.shape,input,bytes);
        boundary.statistics.defined=1;
        boundary.statistics.visible=header.visible;
        boundary.statistics.format=header.format;
        boundary.statistics.width=header.width;
        boundary.statistics.height=header.height;
        boundary.statistics.hot_x=header.hot_x;
        boundary.statistics.hot_y=header.hot_y;
        boundary.statistics.x=header.x;
        boundary.statistics.y=header.y;
        ++boundary.statistics.definitions;
        return 0;
    }
    if (code==ioctl_cursor_move) {
        if (bytes!=sizeof(CursorMoveRequest)) return 1;
        CursorMoveRequest move{};
        memcpy(&move,input,sizeof(move));
        if (move.mode_generation!=boundary.generation) {++boundary.stale_writes; return 1;}
        if ((move.visible!=0 && boundary.fail_move) || (move.visible==0 && boundary.fail_hide)) {
            ++boundary.statistics.failures;
            return 1;
        }
        boundary.statistics.visible=move.visible;
        if (move.visible!=0) {
            boundary.statistics.x=move.x;
            boundary.statistics.y=move.y;
            ++boundary.statistics.moves;
        } else {
            ++boundary.statistics.hides;
        }
        return 0;
    }
    return 1;
}

int main() {
    PDEV device{};
    device.hDriver=reinterpret_cast<HANDLE>(static_cast<uintptr_t>(1));
    device.ScreenWidth=1920;
    device.ScreenHeight=1080;
    device.ScreenDelta=1920*4;
    device.ModeIndex=1;
    device.ScreenPtr=boundary.current_mapping;
    DisplayLink link;
    device.ScreenPtr=nullptr;
    expect(!link.connect(&device),"initial connection rejects missing staging mapping");
    device.ScreenPtr=reinterpret_cast<PVOID>(static_cast<uintptr_t>(0x3000));
    expect(!link.connect(&device),"same-sized stale PDEV cannot acquire current inventory credential");
    device.ScreenPtr=boundary.current_mapping;
    boundary.invalid_owner_information=true;
    expect(!link.connect(&device),"owned mapping query still validates returned identity");
    boundary.invalid_owner_information=false;
    expect(link.connect(&device),"production client connects to its exact checked native staging lease");
    uint8_t pixels[]{32,64,96,128, 0,0,0,0, 1,2,3,255, 0,0,0,0};
    SURFOBJ color{};
    color.iType=STYPE_BITMAP;
    color.iBitmapFormat=BMF_32BPP;
    color.sizlBitmap={2,2};
    color.lDelta=8;
    color.pvScan0=pixels;
    expect(link.define_cursor(nullptr,&color,nullptr,1,1,10,20,SPS_ALPHA),"real normalization submits alpha cursor");
    expect(boundary.shape.pixels[0]==64 && boundary.shape.pixels[1]==128 &&
        boundary.shape.pixels[2]==191 && boundary.shape.pixels[3]==128,"production path converts premultiplied alpha once");
    expect(boundary.statistics.x==9 && boundary.statistics.y==19 && boundary.statistics.visible==1,
        "accepted cursor subtracts hotspot exactly once");
    expect(link.suspend_cursor() && link.hardware_cursor() && boundary.statistics.visible==0,
        "suspend hides while retaining the engine-owned hardware lease");
    pixels[0]=0;
    expect(link.restore_cursor() && boundary.statistics.visible==1 && boundary.shape.pixels[0]==64,
        "restore uses owned normalized bytes rather than borrowed source or repeated conversion");
    expect(link.move_cursor(-1,-1) && link.restore_cursor() && boundary.statistics.visible==0,
        "an intentionally hidden cursor stays hidden after replay");
    expect(link.move_cursor(0,0) && boundary.statistics.x==-1 && boundary.statistics.y==-1,
        "negative top-left edge survives hotspot subtraction");
    expect(link.suspend_cursor() && link.transition_mode(device.ModeIndex,false) && !boundary.enabled,
        "owned quiesce authenticates existing mode and hides scanout");
    boundary.fail_transition=true;
    expect(!link.transition_mode(device.ModeIndex,true) && !boundary.enabled,
        "failed publication returns failure while retaining the initiating credential");
    boundary.fail_transition=false;
    expect(link.transition_mode(device.ModeIndex,true) && boundary.enabled,
        "initiator retries a partial mode commit using its acknowledged advanced token");
    void* mapped=reinterpret_cast<void*>(static_cast<uintptr_t>(1));
    uint32_t mapped_bytes=1;
    boundary.fail_mapping=true;
    expect(!link.map_staging(mapped,mapped_bytes) && mapped==nullptr && mapped_bytes==0,
        "failed owned map exposes no stale output pointer or extent");
    boundary.fail_mapping=false;
    expect(link.map_staging(mapped,mapped_bytes) && mapped!=nullptr && mapped_bytes==1920*1080*4 &&
        link.connect(&device,mapped) && link.restore_cursor(),
        "owned remap reconnects exact token and replays retained normalized cursor");
    expect(device.ScreenPtr!=mapped && !link.connect(&device),
        "unpublished old wrapper cannot borrow the accepted candidate credential");
    boundary.fail_release=true;
    expect(!link.unmap_staging(device.ScreenPtr),"failed old-wrapper retirement retains a retry credential");
    device.ScreenPtr=mapped;
    expect(link.connect(&device),"published replacement wrapper authenticates the same owned mapping");
    expect(link.current_owner() && link.present({0,0,16,16}) && boundary.presents==1,
        "uncertain old retirement does not invalidate new PDEV publication");
    void* refused=nullptr;
    uint32_t refused_bytes=0;
    expect(!link.map_staging(refused,refused_bytes) && refused==nullptr && refused_bytes==0,
        "pending failed retirement blocks another native MAP before mutation");
    boundary.fail_release=false;
    expect(boundary.transitions==3 && boundary.mappings==1,
        "mode retry and staging adoption perform bounded explicit operations");
    ++boundary.generation;
    const auto hides=boundary.statistics.hides;
    expect(!link.move_cursor(40,50) && !link.suspend_cursor() && boundary.statistics.hides==hides,
        "old-generation movement and cleanup cannot alter a replacement plane");
    expect(!link.recover_failed_cursor_move(),"stale owner cannot request software recovery");
    expect(!link.connect(&device),"read-only inventory cannot retag stale PDEV cursor ownership");
    const auto transitions=boundary.transitions;
    expect(!link.transition_mode(device.ModeIndex,false) && !link.transition_mode(device.ModeIndex,true) &&
        boundary.transitions==transitions,"stale transition cannot blank or reactivate replacement mode");
    const auto waits=boundary.waits;
    link.retire();
    expect(boundary.waits==waits,"stale retirement does not wait on another PDEV's publication");
    expect(boundary.releases==1 && boundary.released_generation==7,
        "stale PDEV teardown retries only its retained old generation without releasing replacement");
    boundary.lease_generation=boundary.generation;
    expect(!link.hardware_cursor(),"stale retirement releases only its local hardware lease");
    expect(link.connect(&device) && link.define_cursor(nullptr,&color,nullptr,1,1,10,20,SPS_ALPHA),
        "replacement binds explicitly after old owner retires");
    boundary.fail_move=true;
    expect(!link.move_cursor(60,70) && link.recover_failed_cursor_move(),
        "failed live motion requests recovery only after acknowledged hide");
    CursorStatistics statistics{};
    expect(link.query_cursor(statistics) && statistics.recovery_required==1 && statistics.hardware_active==0 &&
        statistics.visible==0,"explicit recovery state is exposed through the real query path");
    expect(link.query_cursor(statistics) && statistics.recovery_required==1,"query does not consume sticky recovery state");
    boundary.enabled=false;
    expect(link.query_cursor(statistics) && statistics.recovery_required==0,"D3 inventory suppresses stale recovery request");
    boundary.enabled=true;
    expect(link.suspend_cursor() && link.query_cursor(statistics) && statistics.recovery_required==0,
        "intentional suspend clears recovery state");
    boundary.fail_move=false;
    expect(link.restore_cursor(),"retained shape can be reselected after successful lifecycle transition");
    boundary.fail_move=true;
    boundary.fail_hide=true;
    expect(!link.move_cursor(80,90) && !link.recover_failed_cursor_move() && link.query_cursor(statistics) &&
        statistics.recovery_required==0,"uncertain hide never authorizes software takeover");
    boundary.fail_hide=false;
    boundary.fail_move=false;
    boundary.invalid_query=true;
    expect(!link.query_cursor(statistics),"foreign query identity cannot authorize owner state");
    boundary.invalid_query=false;
    link.retire();
    expect(!link.hardware_cursor(),"retire drops hardware lease after acknowledged hide");
    expect(link.connect(&device),"new PDEV can bind after prior retirement");
    boundary.malformed_mapping=true;
    boundary.fail_release=true;
    void* malformed_address=reinterpret_cast<void*>(static_cast<uintptr_t>(1));
    uint32_t malformed_bytes=1;
    const unsigned prior_mappings=boundary.mappings;
    expect(!link.map_staging(malformed_address,malformed_bytes) && malformed_address==nullptr &&
        malformed_bytes==0 && boundary.mappings==prior_mappings+1,
        "malformed accepted MAP publishes no PDEV pointer");
    expect(!link.map_staging(malformed_address,malformed_bytes) && boundary.mappings==prior_mappings+1,
        "unconfirmed token-only retirement blocks another MAP");
    boundary.fail_release=false;
    const uint32_t malformed_generation=boundary.generation;
    link.retire();
    expect(boundary.released_generation==malformed_generation,
        "PDEV teardown retries the exact malformed MAP generation");
    // Authoritative resolve owner: every lost-acknowledgement point.
    boundary.malformed_mapping=false;
    boundary.fail_release=false;
    boundary.resolve_enabled=true;
    boundary.lease_generation=boundary.generation;
    boundary.issued=boundary.generation;
    hold(boundary.generation,0);
    boundary.current_mapping=reinterpret_cast<PVOID>(static_cast<uintptr_t>(0x9000));
    device.ScreenPtr=boundary.current_mapping;
    expect(link.connect(&device),"PDEV binds to an authoritative resolve owner");
    const unsigned legacy_releases=boundary.releases;
    const unsigned legacy_attempts=boundary.legacy_attempts;
    void* candidate=nullptr;
    uint32_t candidate_bytes=0;
    expect(link.map_staging(candidate,candidate_bytes) && link.connect(&device,candidate),"candidate mapped");
    boundary.resolve_lost_next=true;
    expect(!link.unmap_staging(device.ScreenPtr) && held_count()==1 && boundary.resolve_releases==1,
        "old release applied but its acknowledgement was lost; only the token is retained");
    device.ScreenPtr=candidate;
    void* next=nullptr;
    uint32_t next_bytes=0;
    expect(link.map_staging(next,next_bytes) && boundary.resolve_releases==1 && held_count()==2,
        "next MAP drains the lost acknowledgement as not-held, without a second release");
    expect(link.connect(&device,next) && link.unmap_staging(device.ScreenPtr) && boundary.resolve_releases==2,
        "ordinary resolve release of the previous wrapper");
    device.ScreenPtr=next;
    expect(link.map_staging(candidate,candidate_bytes) && link.connect(&device,candidate),"another candidate");
    boundary.resolve_drop_next=true;
    expect(!link.unmap_staging(device.ScreenPtr) && held_count()==2 && boundary.resolve_releases==2,
        "release lost before apply keeps both the lease and its token");
    device.ScreenPtr=candidate;
    boundary.resolve_reorder_next=true;
    const unsigned requests_before_reorder=boundary.resolve_requests;
    expect(link.map_staging(next,next_bytes) && boundary.resolve_releases==3 &&
        boundary.resolve_requests==requests_before_reorder+2,
        "a misattributed acknowledgement is distrusted; the bounded retry gets a final not-held answer");
    expect(link.connect(&device,next) && link.unmap_staging(device.ScreenPtr),"the replacement retires the candidate");
    device.ScreenPtr=next;
    expect(boundary.releases==legacy_releases && boundary.legacy_attempts==legacy_attempts,
        "an authoritative owner never receives legacy releases");

    boundary.map_ack_lost_next=true;
    const unsigned held_before_lost_map=held_count();
    expect(!link.map_staging(next,next_bytes) && next==nullptr && held_count()==held_before_lost_map,
        "lost MAP acknowledgement: the client adopts its own lease and releases it token-only");
    void* recovered=nullptr;
    uint32_t recovered_bytes=0;
    expect(link.map_staging(recovered,recovered_bytes) && link.connect(&device,recovered) &&
        link.unmap_staging(device.ScreenPtr),"adopted mode token keeps later MAPs authenticated");
    device.ScreenPtr=recovered;
    boundary.map_ack_lost_next=true;
    boundary.resolve_fail=true;
    const unsigned maps_before_unknown=boundary.mappings;
    expect(!link.map_staging(next,next_bytes) && boundary.mappings==maps_before_unknown+1,
        "unanswerable lost MAP stays pending");
    expect(!link.map_staging(next,next_bytes) && boundary.mappings==maps_before_unknown+1,
        "pending lost MAP blocks further native mutation");
    boundary.resolve_fail=false;
    expect(link.map_staging(next,next_bytes) && boundary.mappings==maps_before_unknown+2 &&
        link.connect(&device,next) && link.unmap_staging(device.ScreenPtr),
        "recovered answer retires the lost MAP lease and MAP proceeds");
    device.ScreenPtr=next;
    boundary.map_without_lease=true;
    boundary.malformed_mapping=true;
    const unsigned releases_before_unknown=boundary.resolve_releases;
    expect(!link.map_staging(candidate,candidate_bytes) && boundary.resolve_releases==releases_before_unknown,
        "token for a lease never issued is answered unknown");
    boundary.map_without_lease=false;
    boundary.malformed_mapping=false;
    expect(link.map_staging(candidate,candidate_bytes) && link.connect(&device,candidate) &&
        link.unmap_staging(device.ScreenPtr),"the dropped unknown token does not block later MAP");
    device.ScreenPtr=candidate;
    expect(!link.device_lost(),"a live authoritative owner is not reported removed");
    boundary.resolve_fail=true;
    expect(!link.device_lost(),"an unanswerable owner is never treated as removed");
    boundary.resolve_fail=false;
    // A lost MAP acknowledgement whose lease another requester owns.
    boundary.map_ack_lost_next=true;
    boundary.map_foreign_initiator=true;
    const unsigned held_before_foreign=held_count();
    const unsigned releases_before_foreign=boundary.resolve_releases;
    expect(!link.map_staging(next,next_bytes) && held_count()==held_before_foreign+1 &&
        boundary.resolve_releases==releases_before_foreign,"a lease issued to another owner is never adopted");
    boundary.map_foreign_initiator=false;
    expect(boundary.releases==legacy_releases,"unknown and failed resolves never fall back to legacy release");

    // Surprise removal with an active lease: transfer releases every token.
    const unsigned held_before_loss=held_count();
    boundary.lost=true;
    expect(!link.present({0,0,8,8}) && link.device_lost(),"removed owner reports authoritative loss");
    const unsigned releases_before_loss=boundary.resolve_releases;
    expect(link.transfer_lost_leases() && boundary.resolve_releases>releases_before_loss &&
        held_count()<held_before_loss && !link.retirement_pending(),"transfer retires the active lease");
    const unsigned lost_presents=boundary.lost_presents;
    expect(!link.present({0,0,8,8}) && boundary.lost_presents==lost_presents,
        "no native publication is attempted after transfer");
    void* none=nullptr;
    uint32_t none_bytes=0;
    const unsigned requests=boundary.resolve_requests;
    expect(!link.map_staging(none,none_bytes) && boundary.resolve_requests==requests,"no MAP after transfer");
    link.retire();
    expect(boundary.resolve_requests==requests && boundary.releases==legacy_releases,
        "teardown after transfer sends no further retirement");
    boundary.lost=false;
    boundary.resolve_enabled=false;
    expect(bitmap_locks==0 && allocations==0,"all production temporary bitmap locks and allocations retire");
    for (const auto& bitmap:bitmaps) expect(!bitmap.used,"all temporary normalized cursor bitmaps are deleted");
    printf("Production display cursor lifecycle: %u checks, %u failures; native D3/GTK not simulated\n",checks,failures);
    return failures==0 ? 0 : 1;
}

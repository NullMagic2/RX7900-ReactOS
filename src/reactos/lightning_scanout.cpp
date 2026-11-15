/*
Module: lightning.reactos.scanout
Purpose: Program and read back real CPU scanout registers without fake output.
Created: 2026-10-21
Architecture: Portable core consumed by VideoPort and failure-injection tests.
License: GPL-2.0-or-later.
Canonical source: Qemu-Lightning-source/hw/misc/lightning.c, 2026-10-21.
SHA256: 28cec9c0a1f7d45df7022036a452084296b7f552478fe815832122d886ee60ed.
Generated lightning-memory.h SHA256:
ebcafb4ce3e01d354d81aa660d500e9b672af55dd8d184736da11f05244fdac5.
*/
#include "lightning_scanout.hpp"

namespace lightning::reactos {
namespace {
constexpr VirtualMode virtual_modes[]{
    {LIGHTNING_SCANOUT_MODE_0_WIDTH,LIGHTNING_SCANOUT_MODE_0_HEIGHT},
    {LIGHTNING_SCANOUT_MODE_1_WIDTH,LIGHTNING_SCANOUT_MODE_1_HEIGHT},
    {LIGHTNING_SCANOUT_MODE_2_WIDTH,LIGHTNING_SCANOUT_MODE_2_HEIGHT},
    {LIGHTNING_SCANOUT_MODE_3_WIDTH,LIGHTNING_SCANOUT_MODE_3_HEIGHT}
};
static_assert(sizeof(virtual_modes)/sizeof(virtual_modes[0]) == LIGHTNING_SCANOUT_MODE_COUNT,
              "Virtual mode table must match the canonical generated protocol");

uint64_t footprint(VirtualMode mode) noexcept {
    return static_cast<uint64_t>(mode.width)*mode.height*LIGHTNING_SCANOUT_PIXEL_BYTES;
}
} // namespace

ScanoutStatus virtual_mode(uint32_t index, VirtualMode& output) noexcept {
    if (index>=LIGHTNING_SCANOUT_MODE_COUNT) return ScanoutStatus::invalid_mode;
    const VirtualMode mode=virtual_modes[index];
    if (mode.width==0 || mode.height==0 || mode.width>LIGHTNING_SCANOUT_MAX_DIMENSION ||
        mode.height>LIGHTNING_SCANOUT_MAX_DIMENSION || footprint(mode)>UINT32_MAX) {
        return ScanoutStatus::invalid_mode;
    }
    output=mode;
    return ScanoutStatus::success;
}

uint32_t maximum_scanout_bytes() noexcept {
    uint64_t maximum=0;
    for (const auto& mode:virtual_modes) {
        const uint64_t bytes=footprint(mode);
        if (bytes>maximum) maximum=bytes;
    }
    return maximum<=UINT32_MAX ? static_cast<uint32_t>(maximum) : 0;
}

ScanoutStatus CpuScanout::initialize(ScanoutAccess access) noexcept {
    if (ready_ || quarantined_) return ScanoutStatus::busy;
    if (access.read==nullptr || access.write==nullptr || access.clear==nullptr) {
        return ScanoutStatus::invalid_access;
    }
    if (maximum_scanout_bytes()==0 || access.reserved_bytes<maximum_scanout_bytes() ||
        access.reserved_bytes>LIGHTNING_MEMORY_ARENA_BYTES) return ScanoutStatus::invalid_backing;
    uint32_t magic=0;
    uint32_t version=0;
    uint32_t resource_low=0;
    uint32_t resource_high=0;
    if (!access.read(access.context,LIGHTNING_SCANOUT_REG_MAGIC,magic) ||
        !access.read(access.context,LIGHTNING_SCANOUT_REG_VERSION,version) ||
        magic!=LIGHTNING_SCANOUT_MAGIC || version!=LIGHTNING_SCANOUT_VERSION) {
        return ScanoutStatus::unavailable;
    }
    if (!access.read(access.context,LIGHTNING_SCANOUT_REG_RESOURCE_LO,resource_low) ||
        !access.read(access.context,LIGHTNING_SCANOUT_REG_RESOURCE_HI,resource_high)) {
        return ScanoutStatus::hardware_failure;
    }
    if (resource_low!=0 || resource_high!=0) return ScanoutStatus::busy;
    uint32_t extension_magic = 0;
    uint32_t extension_version = 0;
    uint32_t capabilities = 0;
    if (!access.read(access.context,LIGHTNING_DISPLAY_REG_MAGIC,extension_magic) ||
        !access.read(access.context,LIGHTNING_DISPLAY_REG_VERSION,extension_version) ||
        !access.read(access.context,LIGHTNING_DISPLAY_REG_CAPABILITIES,capabilities) ||
        extension_magic!=LIGHTNING_DISPLAY_MAGIC || extension_version!=LIGHTNING_DISPLAY_VERSION ||
        (capabilities&LIGHTNING_DISPLAY_CAPABILITIES)!=LIGHTNING_DISPLAY_CAPABILITIES) {
        return ScanoutStatus::unavailable;
    }
    access_=access;
    ready_=true;
    DisplayCompletion initial{};
    if (completion(initial)!=ScanoutStatus::success) return unknown();
    if (!initial.published) {
        ready_=false;
        return ScanoutStatus::busy;
    }
    epoch_=initial.accepted_epoch;
    sequence_=initial.accepted_sequence;
    owned_=true;
    return ScanoutStatus::success;
}

ScanoutStatus CpuScanout::unknown() noexcept {
    current_known_=false;
    quarantined_=true;
    ready_=false;
    return ScanoutStatus::state_unknown;
}

bool CpuScanout::geometry_matches(const VirtualMode& mode) noexcept {
    const uint32_t offsets[]{LIGHTNING_SCANOUT_REG_WIDTH,LIGHTNING_SCANOUT_REG_HEIGHT,
        LIGHTNING_SCANOUT_REG_STRIDE,LIGHTNING_SCANOUT_REG_OFFSET,
        LIGHTNING_SCANOUT_REG_RESOURCE_LO,LIGHTNING_SCANOUT_REG_RESOURCE_HI};
    const uint32_t expected[]{mode.width,mode.height,
        mode.width*LIGHTNING_SCANOUT_PIXEL_BYTES,0,0,0};
    for (uint32_t i=0;i<sizeof(offsets)/sizeof(offsets[0]);++i) {
        uint32_t actual=0;
        if (!access_.read(access_.context,offsets[i],actual) || actual!=expected[i]) return false;
    }
    return true;
}

ScanoutStatus CpuScanout::set_mode(uint32_t index, bool preserve) noexcept {
    if (!ready_ || quarantined_) return ScanoutStatus::unavailable;
    VirtualMode mode{};
    const ScanoutStatus decoded=virtual_mode(index,mode);
    if (decoded!=ScanoutStatus::success) return decoded;
    if (footprint(mode)>access_.reserved_bytes) return ScanoutStatus::invalid_backing;
    DisplayCompletion prior{};
    if (completion(prior)!=ScanoutStatus::success) return unknown();
    if (!prior.published) return ScanoutStatus::busy;
    uint32_t resource_low=0;
    uint32_t resource_high=0;
    if (!access_.read(access_.context,LIGHTNING_SCANOUT_REG_RESOURCE_LO,resource_low) ||
        !access_.read(access_.context,LIGHTNING_SCANOUT_REG_RESOURCE_HI,resource_high)) return unknown();
    if (resource_low!=0 || resource_high!=0) return ScanoutStatus::busy;
    // Clear the owned mapping before committing. A failed clear can have changed
    // bytes visible through a previous CPU scanout: never report the old frame.
    if (!preserve && !access_.clear(access_.context,static_cast<uint32_t>(footprint(mode)))) return unknown();
    const uint32_t offsets[]{LIGHTNING_SCANOUT_REG_WIDTH,LIGHTNING_SCANOUT_REG_HEIGHT,
        LIGHTNING_SCANOUT_REG_STRIDE,LIGHTNING_SCANOUT_REG_OFFSET};
    const uint32_t values[]{mode.width,mode.height,mode.width*LIGHTNING_SCANOUT_PIXEL_BYTES,0};
    for (uint32_t i=0;i<sizeof(offsets)/sizeof(offsets[0]);++i) {
        if (!access_.write(access_.context,offsets[i],values[i])) return unknown();
    }
    if (!geometry_matches(mode)) return unknown();
    const ScanoutStatus submitted=submit(LIGHTNING_DISPLAY_FULL,nullptr,0,true);
    if (submitted!=ScanoutStatus::success) return submitted;
    current_=index;
    current_known_=true;
    return ScanoutStatus::success;
}

ScanoutStatus CpuScanout::present() noexcept {
    if (!ready_ || quarantined_ || !current_known_) return ScanoutStatus::unavailable;
    VirtualMode mode{};
    if (virtual_mode(current_,mode)!=ScanoutStatus::success || !geometry_matches(mode)) return unknown();
    return submit(LIGHTNING_DISPLAY_FULL,nullptr,0,false);
}

ScanoutStatus CpuScanout::damage(const LightningDisplayRect* rectangles, uint32_t count) noexcept {
    if (!ready_ || quarantined_ || !current_known_) return ScanoutStatus::unavailable;
    VirtualMode mode{};
    if (virtual_mode(current_,mode)!=ScanoutStatus::success || !geometry_matches(mode)) return unknown();
    if (!valid_damage(mode,rectangles,count)) return ScanoutStatus::invalid_backing;
    return submit(LIGHTNING_DISPLAY_DAMAGE,rectangles,count,false);
}

bool valid_damage(VirtualMode mode, const LightningDisplayRect* rectangles, uint32_t count) noexcept {
    if (mode.width==0 || mode.height==0 || rectangles==nullptr || count==0 ||
        count>LIGHTNING_DISPLAY_MAX_RECTS) return false;
    for (uint32_t i=0;i<count;++i) {
        const auto& rectangle=rectangles[i];
        if (rectangle.width==0 || rectangle.height==0 || rectangle.x>=mode.width ||
            rectangle.y>=mode.height || rectangle.width>mode.width-rectangle.x ||
            rectangle.height>mode.height-rectangle.y) return false;
    }
    return true;
}

ScanoutStatus CpuScanout::submit(uint32_t command, const LightningDisplayRect* rectangles,
                               uint32_t count, bool new_epoch) noexcept {
    if (sequence_==UINT32_MAX && !new_epoch) {
        DisplayCompletion prior{};
        const auto status=completion(prior);
        if (status!=ScanoutStatus::success) return status;
        if (!prior.published) return ScanoutStatus::busy;
        // The host forbids sequence wrapping within an epoch. Snapshot a full
        // baseline in the next epoch, including a final dirty update if any.
        if (command==LIGHTNING_DISPLAY_DISABLE) {
            const auto full=submit(LIGHTNING_DISPLAY_FULL,nullptr,0,true);
            return full==ScanoutStatus::success ? ScanoutStatus::busy : full;
        }
        new_epoch=true;
        command=LIGHTNING_DISPLAY_FULL;
        rectangles=nullptr;
        count=0;
    }
    const uint32_t next_epoch=new_epoch ? (epoch_==UINT32_MAX ? 1U : epoch_+1U) : epoch_;
    const uint32_t next_sequence=new_epoch ? 1U : sequence_+1U;
    if (next_epoch==0 || next_sequence==0) return ScanoutStatus::invalid_access;
    for (uint32_t i=0;i<count;++i) {
        const uint32_t fields[]{rectangles[i].x,rectangles[i].y,rectangles[i].width,rectangles[i].height};
        for (uint32_t field=0;field<4;++field) {
            if (!access_.write(access_.context,LIGHTNING_DISPLAY_DAMAGE_OFFSET+
                i*LIGHTNING_DISPLAY_RECT_BYTES+field*sizeof(uint32_t),fields[field])) return unknown();
        }
    }
    // From the command write on, the host may have applied it even when its
    // acknowledgement below cannot be read or is inconsistent.
    attempted_=true;
    attempted_epoch_=next_epoch;
    attempted_sequence_=next_sequence;
    if (!access_.write(access_.context,LIGHTNING_DISPLAY_REG_RECT_COUNT,count) ||
        !access_.write(access_.context,LIGHTNING_DISPLAY_REG_EPOCH,next_epoch) ||
        !access_.write(access_.context,LIGHTNING_DISPLAY_REG_SUBMITTED_SEQ,next_sequence) ||
        !access_.write(access_.context,LIGHTNING_DISPLAY_REG_COMMAND,command)) return unknown();
    uint32_t result=0;
    uint32_t accepted_epoch=0;
    uint32_t accepted_sequence=0;
    if (!access_.read(access_.context,LIGHTNING_DISPLAY_REG_RESULT,result) ||
        !access_.read(access_.context,LIGHTNING_DISPLAY_REG_ACCEPTED_EPOCH,accepted_epoch) ||
        !access_.read(access_.context,LIGHTNING_DISPLAY_REG_ACCEPTED_SEQ,accepted_sequence)) return unknown();
    if (result==LIGHTNING_DISPLAY_BUSY && accepted_epoch==epoch_ && accepted_sequence==sequence_) {
        attempted_=false;
        return ScanoutStatus::busy;
    }
    if (result!=LIGHTNING_DISPLAY_OK || accepted_epoch!=next_epoch || accepted_sequence!=next_sequence) return unknown();
    epoch_=next_epoch;
    sequence_=next_sequence;
    attempted_=false;
    // Only an ordinary acceptance after the replay proves the recovered host
    // works again; a replay that keeps failing spends the bounded limit.
    if (!replay_required_) recoveries_=0;
    replay_required_=false;
    dropped_prefix_=false;
    return ScanoutStatus::success;
}

ScanoutStatus CpuScanout::completion(DisplayCompletion& output) noexcept {
    if (!ready_ || quarantined_) return ScanoutStatus::unavailable;
    DisplayCompletion candidate{};
    uint32_t result=0;
    if (!access_.read(access_.context,LIGHTNING_DISPLAY_REG_ACCEPTED_EPOCH,candidate.accepted_epoch) ||
        !access_.read(access_.context,LIGHTNING_DISPLAY_REG_ACCEPTED_SEQ,candidate.accepted_sequence) ||
        !access_.read(access_.context,LIGHTNING_DISPLAY_REG_REFRESH_NS,candidate.refresh_ns) ||
        !access_.read(access_.context,LIGHTNING_DISPLAY_REG_RESULT,result)) return unknown();
    // A recovered failed publication keeps its sticky result until the replay
    // is accepted; any other failed publication is uncertain.
    if ((result==LIGHTNING_DISPLAY_PUBLICATION_FAILED && !dropped_prefix_) || candidate.refresh_ns==0 ||
        candidate.refresh_ns>LIGHTNING_DISPLAY_NANOSECONDS_PER_SECOND) return unknown();
    // PCI reset or another owner must not turn a canceled prefix into a false
    // completed fence by replacing both hardware counters with zero.
    if (owned_ && (candidate.accepted_epoch!=epoch_ || candidate.accepted_sequence!=sequence_)) return unknown();
    for (uint32_t attempt=0;attempt<3;++attempt) {
        uint32_t epoch_again=0;
        if (!access_.read(access_.context,LIGHTNING_DISPLAY_REG_COMPLETED_EPOCH,candidate.completed_epoch) ||
            !access_.read(access_.context,LIGHTNING_DISPLAY_REG_COMPLETED_SEQ,candidate.completed_sequence) ||
            !access_.read(access_.context,LIGHTNING_DISPLAY_REG_COMPLETED_EPOCH,epoch_again)) return unknown();
        if (epoch_again!=candidate.completed_epoch) continue;
        candidate.published=candidate.completed_epoch==candidate.accepted_epoch &&
                            candidate.completed_sequence==candidate.accepted_sequence;
        // published means nothing accepted remains pending. A dropped prefix
        // reports its true completed counters, never the dropped sequence.
        if (dropped_prefix_ && candidate.completed_epoch==dropped_epoch_ &&
            candidate.completed_sequence==dropped_sequence_) candidate.published=true;
        output=candidate;
        return ScanoutStatus::success;
    }
    return ScanoutStatus::busy;
}

ScanoutStatus CpuScanout::disable() noexcept {
    if (!ready_ || quarantined_) return ScanoutStatus::unavailable;
    DisplayCompletion prior{};
    if (completion(prior)!=ScanoutStatus::success) return unknown();
    if (!prior.published) return ScanoutStatus::busy;
    const ScanoutStatus disabled=submit(LIGHTNING_DISPLAY_DISABLE,nullptr,0,false);
    if (disabled!=ScanoutStatus::success) return disabled;
    current_known_=false;
    return ScanoutStatus::success;
}

ScanoutStatus CpuScanout::recover() noexcept {
    if (!quarantined_) return ready_ ? ScanoutStatus::success : ScanoutStatus::unavailable;
    if (access_.read==nullptr || !owned_ || recoveries_>=recovery_limit) return ScanoutStatus::state_unknown;
    // Every failure to prove the state below is final: spend the limit.
    const auto permanent=[this]() noexcept {
        recoveries_=recovery_limit;
        return ScanoutStatus::state_unknown;
    };
    uint32_t magic=0;
    uint32_t version=0;
    uint32_t extension_magic=0;
    uint32_t extension_version=0;
    uint32_t capabilities=0;
    uint32_t resource_low=0;
    uint32_t resource_high=0;
    if (!access_.read(access_.context,LIGHTNING_SCANOUT_REG_MAGIC,magic) ||
        !access_.read(access_.context,LIGHTNING_SCANOUT_REG_VERSION,version) ||
        !access_.read(access_.context,LIGHTNING_DISPLAY_REG_MAGIC,extension_magic) ||
        !access_.read(access_.context,LIGHTNING_DISPLAY_REG_VERSION,extension_version) ||
        !access_.read(access_.context,LIGHTNING_DISPLAY_REG_CAPABILITIES,capabilities) ||
        !access_.read(access_.context,LIGHTNING_SCANOUT_REG_RESOURCE_LO,resource_low) ||
        !access_.read(access_.context,LIGHTNING_SCANOUT_REG_RESOURCE_HI,resource_high) ||
        magic!=LIGHTNING_SCANOUT_MAGIC || version!=LIGHTNING_SCANOUT_VERSION ||
        extension_magic!=LIGHTNING_DISPLAY_MAGIC || extension_version!=LIGHTNING_DISPLAY_VERSION ||
        (capabilities&LIGHTNING_DISPLAY_CAPABILITIES)!=LIGHTNING_DISPLAY_CAPABILITIES) return permanent();
    if (resource_low!=0 || resource_high!=0) return ScanoutStatus::busy;
    uint32_t accepted_epoch=0;
    uint32_t accepted_sequence=0;
    uint32_t refresh_ns=0;
    uint32_t result=0;
    uint32_t status=0;
    uint32_t completed_epoch=0;
    uint32_t completed_sequence=0;
    uint32_t epoch_again=0;
    if (!access_.read(access_.context,LIGHTNING_DISPLAY_REG_ACCEPTED_EPOCH,accepted_epoch) ||
        !access_.read(access_.context,LIGHTNING_DISPLAY_REG_ACCEPTED_SEQ,accepted_sequence) ||
        !access_.read(access_.context,LIGHTNING_DISPLAY_REG_REFRESH_NS,refresh_ns) ||
        !access_.read(access_.context,LIGHTNING_DISPLAY_REG_RESULT,result) ||
        !access_.read(access_.context,LIGHTNING_SCANOUT_REG_STATUS,status) ||
        !access_.read(access_.context,LIGHTNING_DISPLAY_REG_COMPLETED_EPOCH,completed_epoch) ||
        !access_.read(access_.context,LIGHTNING_DISPLAY_REG_COMPLETED_SEQ,completed_sequence) ||
        !access_.read(access_.context,LIGHTNING_DISPLAY_REG_COMPLETED_EPOCH,epoch_again)) return permanent();
    if (epoch_again!=completed_epoch) return ScanoutStatus::busy;
    if (refresh_ns==0 || refresh_ns>LIGHTNING_DISPLAY_NANOSECONDS_PER_SECOND) return permanent();
    // Any other prefix is a host reset (which also canceled GDI and cursor
    // state) or another writer's. Neither is ours to adopt.
    const bool acknowledged=accepted_epoch==epoch_ && accepted_sequence==sequence_;
    const bool applied=attempted_ && accepted_epoch==attempted_epoch_ && accepted_sequence==attempted_sequence_;
    if (!acknowledged && !applied) return permanent();
    const bool published=completed_epoch==accepted_epoch && completed_sequence==accepted_sequence;
    const bool dropped=result==LIGHTNING_DISPLAY_PUBLICATION_FAILED;
    if (!published && (!dropped || status==LIGHTNING_SCANOUT_PENDING)) return ScanoutStatus::busy;
    epoch_=accepted_epoch;
    sequence_=accepted_sequence;
    attempted_=false;
    dropped_prefix_=dropped;
    dropped_epoch_=completed_epoch;
    dropped_sequence_=completed_sequence;
    // unknown() already cleared the mode: only a FULL or a blank may follow.
    replay_required_=true;
    quarantined_=false;
    ready_=true;
    ++recoveries_;
    return ScanoutStatus::success;
}

ScanoutStatus CpuScanout::current_mode(uint32_t& index) const noexcept {
    if (!ready_ || quarantined_ || !current_known_) return ScanoutStatus::unavailable;
    index=current_;
    return ScanoutStatus::success;
}
} // namespace lightning::reactos

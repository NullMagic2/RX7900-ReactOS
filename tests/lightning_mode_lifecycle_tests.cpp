/*
Module: lightning.tests.mode_lifecycle
Purpose: Verify mapped-PDEV isolation and immediate failed-replacement rollback.
Created: 2026-10-22
Architecture: Exercises the production staging ledger and admission policy only.
*/
#include "lightning_mode_lifecycle.hpp"
#include "lightning_scanout.hpp"
#include "lightning_pci_bar.hpp"
#include <cstdio>
#include <cstdlib>

namespace {
using namespace lightning::reactos;
unsigned checks=0;
void expect(bool condition,const char* description) {
    ++checks;
    if (!condition) {
        std::fprintf(stderr,"FAIL %s\n",description);
        std::exit(1);
    }
}
ModeTransitionRequest request(uint32_t generation,uint32_t lease,bool enable=true) {
    return {{sizeof(ModeTransitionRequest),cpu_scanout_abi_version,generation,1,lease},enable ? 1U : 0U,0};
}
bool authorized(const StagingLedger& staging,uint32_t current,uint32_t rollback,
    const ModeTransitionRequest& candidate) {
    return mode_transition_authorized(staging,current,{rollback,rollback},candidate);
}
}

// The token bookkeeping of lightning_video.cpp around the framebuf SDK pair
// (SET_CURRENT_MODE then MAP) and owned transitions, over the production
// ledger and policy. Mode programming and publication are not simulated.
struct Native final {
    StagingLedger staging{};
    uint8_t ram[4][64]{};
    uint32_t next_ram=0;
    uint32_t generation=1;
    ModeRollbackCredential rollback{};
    ModeRollbackCredential yield{};
    uint32_t initial_map=0;
    bool quiesced=false;
    bool blank=false;

    void advance() { ++generation; yield={}; initial_map=0; }
    bool sdk_set() {
        if (!initial_mode_authorized(staging,generation,yield,quiesced)) return false;
        rollback=current_mode_rollback_credential(staging,generation);
        advance();
        quiesced=false;
        blank=false;
        initial_map=generation;
        return true;
    }
    uint32_t map(uint32_t initiator,bool initial) {
        rollback=map_rollback_credential(staging,generation,initiator,initial,rollback);
        staging.retire();
        while (staging.reclaim()!=nullptr) {}
        if (!staging.install(ram[next_ram],sizeof(ram[0])) || staging.acquire(generation+1U,initiator)==nullptr) return 0;
        ++next_ram;
        advance();
        return generation;
    }
    uint32_t sdk_map() {
        if (!initial_map_authorized(initial_map,generation,quiesced)) return 0;
        initial_map=0;
        return map(0,true);
    }
    // Returns the acknowledged token, or 0 when refused.
    uint32_t transition(uint32_t token,uint32_t lease,bool enable) {
        const ModeTransitionRequest request{{sizeof(ModeTransitionRequest),cpu_scanout_abi_version,token,1,lease},
            enable ? 1U : 0U,0};
        if (!mode_transition_authorized(staging,generation,rollback,request)) return 0;
        if (enable) {
            rollback=current_mode_rollback_credential(staging,generation,lease);
            advance();
            quiesced=false;
            blank=false;
        } else {
            quiesced=true;
            blank=true;
            yield={lease,generation};
        }
        return generation;
    }
    uint32_t owned_map(uint32_t token,uint32_t lease) {
        if (token!=generation || quiesced || !current_mode_lease_authorized(staging,rollback,lease)) return 0;
        return map(lease,false);
    }
};

void initial_pdev_pair() {
    Native native;
    // Boot: no mapped owner, so framebuf's first DrvEnableSurface pair is admitted.
    expect(native.sdk_set(),"first PDEV mode set needs no prior lease");
    const uint32_t first=native.sdk_map();
    expect(first!=0 && native.staging.current_leased_generation(first),"first PDEV map consumes its ticket");
    expect(native.sdk_map()==0,"the SDK map ticket is single use");
    expect(!native.sdk_set(),"an active mapped owner cannot be displaced by an SDK mode set");
    uint32_t token=native.transition(native.generation,first,true);
    expect(token!=0,"first PDEV enables with its own token");
    token=native.owned_map(token,first);
    const uint32_t lease=token;
    expect(lease!=0,"first PDEV remaps through its owned credential");

    // ChangeDisplaySettings: old AssertMode(FALSE), new PDEV pair, swap, enable.
    const uint32_t yielded=native.transition(token,lease,false);
    expect(yielded==token && native.quiesced,"old PDEV yields scanout with an authenticated disable");
    expect(!resume_keeps_yield({},native.generation,native.quiesced,native.blank) &&
        resume_keeps_yield(native.yield,native.generation,native.quiesced,native.blank),
        "power resume keeps a yielded display blank");
    expect(native.sdk_map()==0,"no SDK map without its preauthenticated mode set");
    expect(native.sdk_set() && !resume_preserves(native.initial_map,native.generation),
        "yielded scanout admits the successor; resume before its map stays cleared");
    expect(!native.sdk_set(),"the yield is consumed by one successor");
    const uint32_t replacement=native.sdk_map();
    expect(replacement!=0 && resume_preserves(native.initial_map,native.generation),
        "successor maps its own lease; later resume preserves its image");
    expect(native.rollback.lease_generation==lease && native.rollback.mode_generation==yielded,
        "the SDK map keeps the yielded owner's acknowledged token as rollback");
    expect(native.transition(yielded,lease,true)==0,"old PDEV cannot reactivate over the mapped successor");
    expect(native.transition(native.generation,replacement,true)!=0,"successor enables after the PDEV swap");

    // A successor that fails after its map: framebuf unmaps, win32k loops
    // AssertMode(TRUE) on the old PDEV until it succeeds.
    Native failed;
    failed.sdk_set();
    const uint32_t old_lease=failed.sdk_map();
    uint32_t old_token=failed.owned_map(failed.transition(failed.generation,old_lease,true),old_lease);
    const uint32_t old_ram=old_token;
    old_token=failed.transition(old_token,old_ram,false);
    expect(failed.sdk_set(),"successor mode set after the yield");
    const uint32_t doomed=failed.sdk_map();
    expect(doomed!=0 && failed.staging.release_generation(doomed),"failed successor releases its own lease");
    const uint32_t restored=failed.transition(old_token,old_ram,true);
    expect(restored!=0,"old PDEV rolls back with its last acknowledged token");
    expect(failed.owned_map(restored,old_ram)!=0,"rolled-back PDEV remaps on its retained lease");
    const ModeRollbackCredential recomputed=current_mode_rollback_credential(failed.staging,old_token+1U);
    expect(map_rollback_credential(failed.staging,old_token+1U,0,false,{old_ram,old_token}).mode_generation!=old_token &&
        recomputed.mode_generation!=old_token,
        "recomputing at the SDK map would replace the only rollback token the old PDEV holds");

    // Vacancy and stale owners.
    StagingLedger staging;
    uint8_t buffer[64]{};
    expect(initial_mode_authorized(staging,5,{},false),"empty ledger is vacant");
    expect(staging.install(buffer,sizeof(buffer)) && staging.acquire(6)!=nullptr,"mapped owner installed");
    expect(!initial_mode_authorized(staging,6,{},true),"quiesce without a yield (power, RESET_DEVICE) admits no successor");
    expect(!initial_mode_authorized(staging,6,{6,6},false),"a yield is void once scanout is enabled again");
    expect(!initial_mode_authorized(staging,7,{6,6},true),"a yield is void after the token advances");
    expect(!initial_mode_authorized(staging,6,{5,6},true),"another lease's yield cannot free this owner");
    expect(!initial_mode_authorized(staging,0,{6,0},true),"zero tokens never authorize");
    expect(initial_mode_authorized(staging,6,{6,6},true),"exact yield of the current lease admits a successor");
    expect(staging.release_generation(6) && initial_mode_authorized(staging,6,{},false),
        "released owner leaves the scanout vacant");
    expect(!initial_map_authorized(0,0,false) && !initial_map_authorized(6,7,false) &&
        !initial_map_authorized(6,6,true) && initial_map_authorized(6,6,false),
        "map ticket needs its exact token and an enabled scanout");
    expect(resume_preserves(0,6) && resume_preserves(5,6) && !resume_preserves(6,6),
        "only a pending ticket resumes cleared");
    expect(!resume_keeps_yield({6,6},6,false,true) && !resume_keeps_yield({6,6},6,true,false) &&
        !resume_keeps_yield({6,5},6,true,true),"resume revives a display unless its exact yield holds");
}

int main() {
    uint64_t framebuffer=0;
    expect(framebuffer_bar_address(0x80000008U,0,framebuffer) && framebuffer==0x80000000ULL,
        "opt-in 32-bit prefetchable graphics BAR preserves its below-4 GiB address");
    expect(framebuffer_bar_address(0xc000000cU,0xfU,framebuffer) && framebuffer==0xfc0000000ULL,
        "default 64-bit graphics BAR preserves its complete physical address");
    expect(!framebuffer_bar_address(0x80000009U,0,framebuffer) && framebuffer==0,
        "I/O BAR cannot impersonate graphics memory");
    expect(!framebuffer_bar_address(0x80000008U,1,framebuffer) && framebuffer==0,
        "32-bit graphics BAR rejects a nonzero unused high half");
    expect(!framebuffer_bar_address(0,0,framebuffer),
        "missing graphics BAR is rejected");
    expect(framebuffer_bar_length_valid(0x80000008U,low_mmio_graphics_bytes),
        "32-bit graphics BAR requires the negotiated 512 MiB capacity");
    expect(!framebuffer_bar_length_valid(0x80000008U,1024U*1024U*1024U),
        "32-bit graphics BAR rejects the default 1 GiB range");
    expect(framebuffer_bar_length_valid(0xc000000cU,1024U*1024U*1024U),
        "64-bit graphics BAR retains the default 1 GiB capacity");
    expect(!framebuffer_bar_length_valid(0xc000000cU,low_mmio_graphics_bytes),
        "64-bit graphics BAR rejects an unexpectedly truncated range");
    expect(valid_bus_span(0xfffff000ULL,0x1000U,0x100000000ULL),
        "32-bit BAR mapping can end at 4 GiB");
    expect(!valid_bus_span(0xfffff001ULL,0x1000U,0x100000000ULL),
        "32-bit BAR mapping crossing 4 GiB is rejected");
    expect(valid_bus_span(0x100000000ULL,0x1000U,UINT64_MAX),
        "AMD64 BAR mapping above 4 GiB is permitted");
    expect(!valid_bus_span(UINT64_MAX-7U,16U,UINT64_MAX),
        "64-bit BAR mapping cannot wrap the address space");
    expect(!valid_bus_span(0x1000U,0,UINT64_MAX),
        "empty BAR mapping is rejected");
    StagingLedger staging;
    uint8_t buffers[3][64]{};
    expect(!staging.leased_generation(0) && !staging.current_leased_generation(0),"zero is never an owner credential");
    expect(staging.current_mapping_generation(nullptr)==0 && staging.current_mapping_generation(buffers[0])==0,
        "empty ledger cannot authenticate a PDEV mapping");
    expect(staging.install(buffers[0],sizeof(buffers[0])) && staging.acquire(7)==buffers[0],"first PDEV receives generation-tagged RAM");
    expect(staging.current_generation()==7 && staging.current_leased_generation(7),"active lease identity is explicit");
    const ModeRollbackCredential first=current_mode_rollback_credential(staging,7);
    expect(first.lease_generation==7 && first.mode_generation==7,"first mode token snapshots mapped owner");
    const ModeRollbackCredential advanced=current_mode_rollback_credential(staging,8);
    expect(advanced.lease_generation==7 && advanced.mode_generation==8,
        "later acknowledged mode retains lease but advances rollback token");
    expect(staging.current_mapping_generation(buffers[0])==7 &&
        staging.current_mapping_generation(buffers[0]+1)==0 && staging.current_mapping_generation(nullptr)==0,
        "only the exact returned mapping base authenticates the active lease");
    expect(staging.acquire(8)==nullptr && staging.current_generation()==7,"an acquire cannot retag an existing slot");
    expect(authorized(staging,7,0,request(7,7)),"mapped current owner may activate its mode");
    expect(authorized(staging,7,0,request(7,7,false)),"mapped current owner may quiesce its mode");
    expect(!authorized(staging,7,0,request(7,6)),"inventory token alone cannot replace RAM ownership");
    expect(authorized(staging,8,7,request(8,7)),"initiator can retry an acknowledged partially committed mode");
    expect(authorized(staging,8,7,request(7,7)),"immediate previous owner can roll back before replacement maps");
    expect(!authorized(staging,8,7,request(7,7,false)),"retired owner cannot blank a transition");
    staging.retire();
    expect(staging.current_mapping_generation(buffers[0])==0,"retired mapped RAM cannot adopt replacement inventory");
    expect(staging.install(buffers[1],sizeof(buffers[1])) && staging.acquire(9)==buffers[1],"replacement has disjoint leased RAM");
    expect(staging.leased_generation(7) && staging.current_leased_generation(9),"retired RAM remains live without active ownership");
    expect(staging.current_mapping_generation(buffers[0])==0 && staging.current_mapping_generation(buffers[1])==9,
        "same-sized old mapping cannot borrow the replacement generation");
    expect(!authorized(staging,9,7,request(7,7)),"old PDEV cannot reactivate over mapped replacement");
    expect(!authorized(staging,9,7,request(9,7)),"forged current inventory with stale lease cannot bypass replacement");
    expect(!authorized(staging,9,7,request(7,7,false)),"old PDEV cannot reset mapped replacement");
    expect(!mode_transition_authorized(staging,10,advanced,request(10,7)),
        "mode generation advanced beyond active replacement lease cannot admit stale PDEV");
    expect(mode_transition_authorized(staging,10,advanced,request(10,9)),
        "active replacement retains its own mode authority after generation advances");
    expect(!mode_transition_authorized(staging,10,advanced,request(8,7)),
        "old rollback cannot bypass active replacement through differing mode and lease generations");
    expect(!current_mode_lease_authorized(staging,advanced,7) &&
        current_mode_lease_authorized(staging,advanced,9),
        "owned MAP admits only the mapped replacement lease after mode generation advances");
    buffers[0][0]=0xa7;
    expect(buffers[1][0]==0,"stale cached writes do not alias active staging");
    expect(staging.release(buffers[1]) && !staging.current_leased_generation(9),"failed replacement releases its own lease");
    expect(staging.current_mapping_generation(buffers[1])==0,"unleased current allocation cannot authenticate an owner");
    const ModeRollbackCredential retry=current_mode_rollback_credential(staging,9,7);
    expect(retry.lease_generation==7 && retry.mode_generation==9,
        "validated retry preserves initiator lease when current replacement is unleased");
    expect(current_mode_rollback_credential(staging,9).lease_generation==0,
        "owner-independent transition cannot borrow an old lease from unleased inventory");
    expect(mode_transition_authorized(staging,10,retry,request(10,7)),
        "same initiating owner may retry an acknowledged transition with no mapped replacement");
    expect(current_mode_lease_authorized(staging,retry,7) &&
        !current_mode_lease_authorized(staging,{},7),
        "owned MAP requires exact retained initiator when replacement is unleased");
    expect(authorized(staging,9,7,request(7,7)),"only unowned replacement permits prior PDEV rollback");
    expect(!mode_transition_authorized(staging,9,advanced,request(7,7)),
        "older token on the retained lease cannot roll back a later acknowledged mode");
    expect(mode_transition_authorized(staging,9,advanced,request(8,7)),
        "exact latest token may roll back an unowned failed replacement");
    expect(!mode_transition_authorized(staging,9,{},request(8,7)),
        "missing rollback snapshot never authorizes a stale PDEV");
    expect(!authorized(staging,9,6,request(7,7)),"older historical credentials cannot claim immediate rollback");
    expect(!authorized(staging,9,7,request(10,7)),"future generation cannot authorize rollback");
    auto invalid=request(9,7);
    invalid.owner.size--;
    expect(!authorized(staging,9,7,invalid),"truncated transition fails before mutation");
    invalid=request(9,7);
    invalid.owner.version++;
    expect(!authorized(staging,9,7,invalid),"unknown version fails before mutation");
    invalid=request(9,7);
    invalid.enable=2;
    expect(!authorized(staging,9,7,invalid),"nonboolean activation rejected");
    invalid=request(9,7);
    invalid.reserved=1;
    expect(!authorized(staging,9,7,invalid),"unknown flags rejected");
    invalid=request(9,0);
    expect(!authorized(staging,9,7,invalid),"missing lease rejected");
    expect(staging.release(buffers[0]) && staging.reclaim()==buffers[0],"old wrapper release reclaims only its RAM");
    expect(!authorized(staging,9,7,request(7,7)),"reclaimed lease never authorizes recovery");
    expect(!staging.release(buffers[0]),"double retirement cannot create another owner");
    staging.retire();
    expect(staging.reclaim()==buffers[1],"failed current allocation retires after explicit replacement");
    expect(staging.install(buffers[2],sizeof(buffers[2])) && staging.acquire(UINT32_MAX)==buffers[2],"highest nonwrapping token stays a valid existing lease");
    expect(authorized(staging,UINT32_MAX,0,request(UINT32_MAX,UINT32_MAX,false)),"token exhaustion still permits safe quiesce");
    expect(!authorized(staging,0,UINT32_MAX,request(UINT32_MAX,UINT32_MAX)),"reset zero cannot reauthorize an old maximum token");
    expect(!staging.release_generation(0) && !staging.release_generation(7),
        "missing or reclaimed generations cannot retire a newer mapping");
    expect(staging.release_generation(UINT32_MAX) && !staging.release_generation(UINT32_MAX),
        "generation retirement is balanced and cannot repeat");
    staging.retire();
    expect(staging.reclaim()==buffers[2],"acknowledged generation retirement frees only the matching RAM");
    StagingLedger recycled;
    expect(recycled.install(buffers[0],sizeof(buffers[0])) && recycled.acquire(17)==buffers[0] &&
        recycled.release_generation(17),"reusable mapping begins with an acknowledged release");
    recycled.retire();
    expect(recycled.reclaim()==buffers[0] && recycled.install(buffers[0],sizeof(buffers[0])) &&
        recycled.acquire(18)==buffers[0],"reclaimed address can return as a newer staging generation");
    expect(!recycled.release_generation(17) && recycled.current_mapping_generation(buffers[0])==18,
        "delayed release of reused address retains its newer native lease");
    recycled.retire();
    expect(recycled.install(buffers[1],sizeof(buffers[1])) && recycled.acquire(17)==nullptr &&
        recycled.acquire(18)==nullptr && recycled.acquire(19)==buffers[1],
        "a retired generation is never reissued to new RAM");
    initial_pdev_pair();
    std::printf("Mode lifecycle: %u checks PASS; no native mode programming simulated\n",checks);
}

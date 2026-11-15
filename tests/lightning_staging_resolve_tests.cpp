/*
Module: lightning.tests.staging_resolve
Purpose: Fault-inject lost staging release acknowledgements and removal states.
Created: 2026-10-23
Architecture: Production ledger, resolve policy, client leases and removal gate;
a small paired miniport model stands in only for the native mutex and pool.
*/
#include "lightning_staging_resolve.hpp"
#include "lightning_staging_client.hpp"
#include "lightning_display_gate.hpp"
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

constexpr uint32_t unmap_code=0x00230014U;

// Paired miniport model: the real ledger and resolve policy under one owner,
// with the native pool replaced by fixed buffers whose frees are recorded.
struct Miniport final {
    StagingLedger ledger{};
    StagingFaultPlan faults{};
    bool lost=false;
    uint32_t mode_generation=0;
    uint32_t frees=0;
    uint32_t applied_releases=0;
    uint8_t* freed[8]{};

    void reclaim() noexcept {
        while (void* retired=ledger.reclaim()) {
            if (frees<8) freed[frees]=static_cast<uint8_t*>(retired);
            ++frees;
        }
    }

    // Mirrors lightning_video.cpp map_staging: fresh slot, generation+1.
    void* map(uint8_t* buffer,uint32_t initiator) noexcept {
        ledger.retire();
        reclaim();
        if (!ledger.install(buffer,64)) return nullptr;
        void* address=ledger.acquire(mode_generation+1U,initiator);
        if (address!=nullptr) ++mode_generation;
        return address;
    }

    void abandon() noexcept {
        lost=true;
        ledger.retire();
        reclaim();
    }

    // Returns false when no acknowledgement reaches the client.
    bool resolve(const StagingResolveRequest& request,StagingResolveReceipt& receipt) noexcept {
        if (!valid_staging_resolve_request(request,sizeof(request))) return false;
        const StagingFault fault=faults.next(request.operation);
        if (fault==StagingFault::drop_before_apply) return false;
        const uint32_t before=ledger.held_leases();
        receipt=resolve_staging(ledger,request,lost);
        if (ledger.held_leases()<before) ++applied_releases;
        reclaim();
        return fault!=StagingFault::lost_after_apply;
    }
};

struct Client final {
    Miniport* miniport=nullptr;
    uint32_t initiator=0;
    uint32_t requests=0;
    bool reorder_next=false;

    static bool release(void* context,uint32_t generation) noexcept {
        auto& self=*static_cast<Client*>(context);
        ++self.requests;
        StagingResolveRequest request{sizeof(request),cpu_scanout_abi_version,generation,
            staging_resolve_release,self.initiator};
        StagingResolveReceipt receipt{};
        if (!self.miniport->resolve(request,receipt)) return false;
        if (self.reorder_next) {
            // A receipt arriving for another request must not be trusted.
            self.reorder_next=false;
            receipt.lease_generation=generation+1U;
        }
        return valid_staging_resolve_receipt(receipt,request);
    }
};

StagingResolveRequest query(uint32_t generation,uint32_t initiator=0) {
    return {sizeof(StagingResolveRequest),cpu_scanout_abi_version,generation,staging_resolve_query,initiator};
}

StagingResolveRequest release(uint32_t generation) {
    return {sizeof(StagingResolveRequest),cpu_scanout_abi_version,generation,staging_resolve_release,0};
}

void ledger_contract() {
    StagingLedger ledger;
    uint8_t buffers[3][64]{};
    uint32_t initiator=0;
    expect(ledger.classify(1,initiator)==StagingLedger::LeaseState::unknown,"empty ledger issued nothing");
    expect(ledger.install(buffers[0],64) && ledger.acquire()==buffers[0] && ledger.acquire()==buffers[0],
        "legacy untagged MAP keeps shared reference-counted leases");
    expect(ledger.acquire(5)==nullptr,"a referenced untagged slot cannot be retagged");
    expect(ledger.release(buffers[0]) && ledger.release(buffers[0]) && ledger.issued_generation()==0,
        "untagged leases do not consume the generation watermark");
    ledger.retire();
    expect(ledger.reclaim()==buffers[0],"untagged RAM retires normally");
    expect(ledger.install(buffers[1],64) && ledger.acquire(7,3)==buffers[1],"owned MAP tags its single lease");
    expect(ledger.acquire(7)==nullptr && ledger.acquire(8)==nullptr && ledger.held_leases()==1,
        "one generation names exactly one lease");
    expect(ledger.classify(7,initiator)==StagingLedger::LeaseState::held && initiator==3,
        "held lease reports its initiating owner");
    expect(ledger.classify(6,initiator)==StagingLedger::LeaseState::not_held,
        "a mode-only token at or below the watermark never names RAM");
    expect(ledger.classify(8,initiator)==StagingLedger::LeaseState::unknown,"future generation is unknown");
    ledger.retire();
    expect(ledger.install(buffers[2],64) && ledger.acquire(7)==nullptr && ledger.acquire(6)==nullptr,
        "a retired or older generation is never reissued");
    expect(ledger.acquire(9)==buffers[2] && ledger.issued_generation()==9,"watermark advances monotonically");
    expect(ledger.held_leases()==2,"retired and current leases are both counted");
}

void protocol_contract() {
    StagingLedger ledger;
    uint8_t buffer[64]{};
    expect(ledger.install(buffer,64) && ledger.acquire(4,2)==buffer,"lease for protocol checks");
    auto bad=release(4);
    bad.size--;
    expect(!valid_staging_resolve_request(bad,sizeof(bad)),"truncated request rejected");
    bad=release(4);
    bad.version++;
    expect(!valid_staging_resolve_request(bad,sizeof(bad)),"unknown version rejected");
    bad=release(0);
    expect(!valid_staging_resolve_request(bad,sizeof(bad)),"zero generation rejected");
    bad=release(4);
    bad.operation=3;
    expect(!valid_staging_resolve_request(bad,sizeof(bad)),"unknown operation rejected");
    expect(!valid_staging_resolve_request(release(4),sizeof(StagingResolveRequest)+1),"oversized input rejected");

    auto receipt=resolve_staging(ledger,query(4,2),false);
    expect(receipt.outcome==staging_lease_held && (receipt.flags&staging_flag_initiator)!=0 &&
        ledger.held_leases()==1 && valid_staging_resolve_receipt(receipt,query(4,2)),
        "query reports held with its own initiator and changes nothing");
    receipt=resolve_staging(ledger,query(4,9),false);
    expect(receipt.outcome==staging_lease_held && (receipt.flags&staging_flag_initiator)==0,
        "another requester's lease is never attributed to this client");
    receipt=resolve_staging(ledger,release(4),false);
    expect(receipt.outcome==staging_lease_released && ledger.held_leases()==0 &&
        receipt.issued_generation==4 && valid_staging_resolve_receipt(receipt,release(4)),
        "release retires the lease exactly once");
    receipt=resolve_staging(ledger,release(4),false);
    expect(receipt.outcome==staging_lease_not_held && valid_staging_resolve_receipt(receipt,release(4)),
        "duplicate release is a final not-held answer, not a second decrement");
    receipt=resolve_staging(ledger,release(5),true);
    expect(receipt.outcome==staging_lease_unknown && (receipt.flags&staging_flag_device_lost)!=0,
        "never-issued token is unknown and loss is reported");

    auto forged=resolve_staging(ledger,release(4),false);
    forged.lease_generation=3;
    expect(!valid_staging_resolve_receipt(forged,release(4)),"reordered receipt for another generation rejected");
    forged=resolve_staging(ledger,release(4),false);
    forged.outcome=staging_lease_held;
    expect(!valid_staging_resolve_receipt(forged,release(4)),"a release answered held is not a valid answer");
    forged=resolve_staging(ledger,query(4),false);
    forged.outcome=staging_lease_released;
    expect(!valid_staging_resolve_receipt(forged,query(4)),"a query cannot claim to have released");
    forged=resolve_staging(ledger,query(4),false);
    forged.flags|=0x80U;
    expect(!valid_staging_resolve_receipt(forged,query(4)),"unknown receipt flags rejected");
    forged=resolve_staging(ledger,query(4),false);
    forged.issued_generation=3;
    expect(!valid_staging_resolve_receipt(forged,query(4)),"answer above its own watermark rejected");
    forged=resolve_staging(ledger,query(4),false);
    forged.reserved=1;
    expect(!valid_staging_resolve_receipt(forged,query(4)),"nonzero reserved receipt field rejected");
}

void lost_acknowledgements() {
    uint8_t buffers[4][64]{};
    Miniport miniport;
    Client client{&miniport,0};
    StagingClientLeases leases;
    void* first=miniport.map(buffers[0],0);
    expect(first==buffers[0] && leases.remember(first,1),"PDEV holds generation one");

    miniport.faults.configure(1U<<StagingFaultPlan::drop_shift);
    expect(!leases.retire(first,Client::release,&client) && miniport.ledger.held_leases()==1 &&
        miniport.applied_releases==0,"request lost before apply retains the lease and its token");
    expect(leases.drain(Client::release,&client) && miniport.applied_releases==1 && miniport.frees==0,
        "retry after a pre-apply loss releases exactly once");
    miniport.ledger.retire();
    miniport.reclaim();
    expect(miniport.frees==1 && miniport.freed[0]==buffers[0],"retired unleased RAM is freed once");

    void* second=miniport.map(buffers[1],1);
    expect(second==buffers[1] && leases.remember(second,2),"second lease");
    miniport.faults.configure(1U);
    expect(!leases.retire(second,Client::release,&client) && miniport.applied_releases==2 &&
        miniport.ledger.held_leases()==0,"applied release whose acknowledgement is lost keeps only the token");
    expect(leases.drain(Client::release,&client) && miniport.applied_releases==2,
        "retry after a lost acknowledgement resolves not-held without a second release");
    expect(leases.empty(),"the uncertain credential is retired definitively");

    void* third=miniport.map(buffers[2],2);
    expect(third==buffers[2] && leases.remember(third,3),"third lease");
    client.reorder_next=true;
    expect(!leases.retire(third,Client::release,&client) && miniport.applied_releases==3,
        "a misattributed receipt is not trusted even though the release applied");
    expect(leases.drain(Client::release,&client) && miniport.applied_releases==3 && leases.empty(),
        "the reordered case converges through a final not-held answer");

    // Recycled address: buffers[2] retired, returns as a newer generation.
    miniport.ledger.retire();
    miniport.reclaim();
    void* recycled=miniport.map(buffers[2],3);
    expect(recycled==buffers[2] && miniport.mode_generation==4,"freed address returns under generation four");
    StagingResolveReceipt receipt{};
    expect(miniport.resolve(release(3),receipt) && receipt.outcome==staging_lease_not_held &&
        miniport.ledger.held_leases()==1,"a delayed retry of generation three cannot reach the recycled lease");
    expect(leases.remember(recycled,4) && leases.retire(recycled,Client::release,&client) &&
        miniport.ledger.held_leases()==0,"the recycled lease retires only under its own generation");

    // Lost MAP acknowledgement: query recovers the client's own lease only.
    void* lost_map=miniport.map(buffers[3],4);
    expect(lost_map!=nullptr,"MAP applied but its acknowledgement is lost");
    expect(miniport.resolve(query(5,4),receipt) && receipt.outcome==staging_lease_held &&
        (receipt.flags&staging_flag_initiator)!=0,"the initiator recognizes its unacknowledged lease");
    expect(miniport.resolve(query(5,7),receipt) && (receipt.flags&staging_flag_initiator)==0,
        "a different PDEV cannot adopt that lease");
    expect(leases.remember_unpublished(5) && leases.drain(Client::release,&client) &&
        miniport.ledger.held_leases()==0,"adopted token-only lease retires once");
    expect(miniport.resolve(query(6,4),receipt) && receipt.outcome==staging_lease_unknown,
        "a MAP that never applied leaves nothing to adopt");
}

void surprise_removal_states() {
    uint8_t buffers[4][64]{};
    // State 1: removal before any lease exists.
    {
        Miniport miniport;
        miniport.abandon();
        StagingResolveReceipt receipt{};
        expect(miniport.resolve(query(1),receipt) && receipt.outcome==staging_lease_unknown &&
            (receipt.flags&staging_flag_device_lost)!=0 && receipt.held_leases==0,
            "removal before MAP reports loss with nothing held");
    }
    // State 2: one active lease; abandon must not free leased RAM.
    {
        Miniport miniport;
        Client client{&miniport,0};
        StagingClientLeases leases;
        expect(leases.remember(miniport.map(buffers[0],0),1),"active lease before removal");
        miniport.abandon();
        expect(miniport.frees==0 && miniport.ledger.held_leases()==1,
            "abandon retains RAM a wrapper may still alias");
        leases.retire_all();
        expect(leases.drain(Client::release,&client) && miniport.frees==1 && miniport.freed[0]==buffers[0] &&
            leases.empty(),"after its views move, the client's release frees the lease once");
    }
    // State 3: mode change in flight, old wrapper pending after a lost ack and
    // a new candidate active; an unleased retained slot is freed at abandon.
    {
        Miniport miniport;
        Client client{&miniport,0};
        StagingClientLeases leases;
        void* old_mapping=miniport.map(buffers[0],0);
        expect(leases.remember(old_mapping,1),"old PDEV lease");
        void* candidate=miniport.map(buffers[1],1);
        expect(leases.remember(candidate,2),"candidate lease");
        miniport.faults.configure(1U);
        expect(!leases.retire(old_mapping,Client::release,&client),"old release acknowledgement lost");
        expect(miniport.frees==1 && miniport.freed[0]==buffers[0],
            "the applied release freed the retired old RAM despite the lost acknowledgement");
        void* spare=miniport.map(buffers[2],2);
        expect(spare==buffers[2] && miniport.frees==1,"the leased candidate survives the next MAP");
        expect(miniport.ledger.release_generation(3),"spare lease dropped by its owner");
        miniport.abandon();
        expect(miniport.frees==2 && miniport.freed[1]==buffers[2] && miniport.ledger.held_leases()==1,
            "abandon frees unleased retained RAM and keeps the candidate");
        leases.retire_all();
        expect(leases.drain(Client::release,&client) && miniport.frees==3 && miniport.freed[2]==buffers[1],
            "transfer retires pending and candidate credentials, each exactly once");
    }
    // State 4: quiesced/D3 then removal; drop and lost faults during transfer.
    {
        Miniport miniport;
        Client client{&miniport,0};
        StagingClientLeases leases;
        expect(leases.remember(miniport.map(buffers[3],0),1),"quiesced owner lease");
        miniport.abandon();
        leases.retire_all();
        miniport.faults.configure((1U<<StagingFaultPlan::drop_shift)|1U);
        expect(!leases.drain(Client::release,&client) && miniport.ledger.held_leases()==1,
            "a dropped transfer release keeps the lease");
        expect(!leases.drain(Client::release,&client) && miniport.ledger.held_leases()==0 && miniport.frees==1,
            "a lost transfer acknowledgement still freed the RAM once");
        expect(leases.drain(Client::release,&client) && leases.empty() && miniport.frees==1,
            "bounded retry converges without a second free");
        expect(miniport.faults.injected()==2,"both injected faults were consumed");
    }
    // State 5: detached filter after REMOVE admits only kernel retirement.
    expect(retained_retirement_admitted(ioctl_resolve_owned_staging,true,unmap_code) &&
        retained_retirement_admitted(ioctl_release_owned_staging,true,unmap_code) &&
        retained_retirement_admitted(unmap_code,true,unmap_code),"retired filter keeps kernel retirement");
    expect(!retained_retirement_admitted(ioctl_resolve_owned_staging,false,unmap_code),
        "user mode cannot reach retirement after removal");
    expect(!retained_retirement_admitted(ioctl_map_owned_staging,true,unmap_code) &&
        !retained_retirement_admitted(ioctl_present_cpu_scanout,true,unmap_code) &&
        !retained_retirement_admitted(ioctl_query_owned_staging,true,unmap_code),
        "no MAP, presentation or authentication after removal");
    expect(display_admission(DisplayRequest::retirement,true,true,false)==DisplayAdmission::allow,
        "resolve is admitted as retirement while the device is inactive");
}

void fault_plan() {
    StagingFaultPlan plan;
    expect(plan.next(staging_resolve_release)==StagingFault::none,"default plan injects nothing");
    plan.configure((2U<<StagingFaultPlan::drop_shift)|1U);
    expect(plan.next(staging_resolve_query)==StagingFault::none,"queries are never faulted");
    expect(plan.next(staging_resolve_release)==StagingFault::drop_before_apply &&
        plan.next(staging_resolve_release)==StagingFault::drop_before_apply &&
        plan.next(staging_resolve_release)==StagingFault::lost_after_apply &&
        plan.next(staging_resolve_release)==StagingFault::none && plan.injected()==3,
        "drops precede lost acknowledgements and both counts are bounded");
}
} // namespace

int main() {
    ledger_contract();
    protocol_contract();
    lost_acknowledgements();
    surprise_removal_states();
    fault_plan();
    std::printf("Staging resolve: %u checks PASS; native pool and mutex modeled\n",checks);
    return 0;
}

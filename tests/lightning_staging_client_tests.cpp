/*
Module: lightning.tests.staging_client
Purpose: Check bounded PDEV mapping retirement and generation-only retry safety.
Created: 2026-10-22
Architecture: Exercises the production lease owner; no native driver simulated.
*/
#include "lightning_staging_client.hpp"
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
struct Boundary final {
    uint32_t refused=0;
    uint32_t calls=0;
    uint32_t last=0;
    static bool release(void* context,uint32_t generation) noexcept {
        auto& self=*static_cast<Boundary*>(context);
        ++self.calls;
        self.last=generation;
        return generation!=self.refused;
    }
};
}

int main() {
    uint8_t buffers[StagingLedger::capacity+1][64]{};
    StagingClientLeases leases;
    Boundary boundary;
    expect(leases.has_room(),"empty PDEV owns no native mapping credentials");
    expect(!leases.remember(nullptr,1) && !leases.remember(buffers[0],0),"invalid credentials consume no capacity");
    expect(leases.remember(buffers[0],7),"initial native mapping has an exact token");
    expect(leases.remember(buffers[0],7),"reconnect retains the existing single lease");
    expect(!leases.remember(buffers[0],8),"the same address cannot be retagged while its lease exists");
    expect(!leases.remember(buffers[1],7),"one generation cannot authorize two allocations");
    expect(!leases.retire(nullptr,Boundary::release,&boundary) && boundary.calls==0,"unknown retirement cannot reach native release");
    boundary.refused=7;
    expect(!leases.retire(buffers[0],Boundary::release,&boundary) && boundary.last==7,
        "failed old-wrapper unmap retains its exact generation");
    expect(!leases.remember(buffers[0],7),"a retiring mapping cannot be adopted as an active PDEV");
    expect(leases.remember(buffers[1],8),"new wrapper remains separately owned during failed old retirement");
    expect(!leases.drain(Boundary::release,&boundary) && boundary.last==7,
        "retry sends the retained generation instead of a reusable address");
    const uint32_t attempts=boundary.calls;
    expect(!leases.drain(nullptr,&boundary) && boundary.calls==attempts,"absent release service preserves retirement");
    boundary.refused=0;
    expect(leases.drain(Boundary::release,&boundary) && boundary.last==7,"recovered native completion reclaims pending ownership");
    expect(leases.remember(buffers[0],9),"a definitively retired address can be reused with a new generation");
    expect(leases.drain(Boundary::release,&boundary) && boundary.last==7,"active reused mapping is never retried as the old lease");
    expect(leases.retire(buffers[0],Boundary::release,&boundary) && boundary.last==9,"reused address retires only the new token");
    expect(!leases.retire(buffers[0],Boundary::release,&boundary),"double retirement cannot release another generation");
    expect(leases.retire(buffers[1],Boundary::release,&boundary) && boundary.last==8,"replacement retires independently");
    boundary.refused=10;
    expect(leases.remember_unpublished(10),"malformed successful MAP retains its generation without trusting its address");
    expect(!leases.remember(buffers[0],10) && !leases.remember_unpublished(10),
        "unpublished generation cannot be adopted by a later mapping");
    expect(!leases.drain(Boundary::release,&boundary) && boundary.last==10,
        "failed token-only release remains pending without an address");
    boundary.refused=0;
    expect(leases.drain(Boundary::release,&boundary) && boundary.last==10,
        "token-only retirement retries until native acknowledgement");
    for (uint32_t i=0;i<StagingLedger::capacity;++i) {
        expect(leases.remember(buffers[i],i+1),"bounded native slot can be retained");
    }
    expect(!leases.has_room() && !leases.remember(buffers[StagingLedger::capacity],99),"capacity exhaustion rejects before native MAP");
    boundary.refused=1;
    for (uint32_t i=0;i<StagingLedger::capacity;++i) {
        expect(leases.retire(buffers[i],Boundary::release,&boundary)==(i!=0),"each pending owner retires only after its own acknowledgement");
    }
    expect(leases.has_room() && !leases.drain(Boundary::release,&boundary),"one quarantined token does not discard successful cleanup");
    boundary.refused=0;
    expect(leases.drain(Boundary::release,&boundary),"final teardown retry completes all definite retirements");
    expect(leases.empty(),"all credentials retired before the removal transfer cases");
    expect(leases.remember(buffers[0],20) && leases.remember(buffers[1],21),"active and candidate leases");
    boundary.refused=21;
    expect(!leases.retire(buffers[1],Boundary::release,&boundary),"candidate release pending");
    leases.retire_all();
    boundary.refused=0;
    const uint32_t before_transfer=boundary.calls;
    expect(leases.drain(Boundary::release,&boundary) && boundary.calls==before_transfer+2 && leases.empty(),
        "removal transfer retires the active and the pending credential once each");
    expect(leases.drain(Boundary::release,&boundary) && boundary.calls==before_transfer+2,
        "an empty owner sends nothing after transfer");
    std::printf("PDEV staging retirement: %u checks PASS; no native mapping operation simulated\n",checks);
}

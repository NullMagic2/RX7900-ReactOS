/*
Module: lightning.tests.staging_pool
Purpose: Keep full-frame display RAM out of the AMD64 ReactOS paged pool.
Created: 2026-10-25
Architecture: Portable; the production pool policy against the generated mode table.
*/
#include "lightning_staging_pool.hpp"
#include "lightning-memory.h"
#include <cstdio>

namespace {
unsigned checks=0;
unsigned failures=0;
void expect(bool condition,const char* description) {
    ++checks;
    if (!condition) {++failures; std::printf("FAIL %s\n",description);}
}
} // namespace

int main() {
    using namespace lightning::reactos;
    // lightning_video.cpp maps maximum_scanout_bytes() for every staging lease.
    const uint64_t modes[][2]{{LIGHTNING_SCANOUT_MODE_0_WIDTH,LIGHTNING_SCANOUT_MODE_0_HEIGHT},
        {LIGHTNING_SCANOUT_MODE_1_WIDTH,LIGHTNING_SCANOUT_MODE_1_HEIGHT},
        {LIGHTNING_SCANOUT_MODE_2_WIDTH,LIGHTNING_SCANOUT_MODE_2_HEIGHT},
        {LIGHTNING_SCANOUT_MODE_3_WIDTH,LIGHTNING_SCANOUT_MODE_3_HEIGHT}};
    uint64_t frame=0;
    for (const auto& mode:modes) {
        const uint64_t bytes=mode[0]*mode[1]*LIGHTNING_SCANOUT_PIXEL_BYTES;
        if (bytes>frame) frame=bytes;
    }
    expect(frame==1920ull*1080*4,"the largest staging frame is the 1920x1080 mode");
    expect(!mode_change_fits_pool(frame,reactos_amd64_paged_pool_bytes),
        "a mode change's two frames do not fit the 32 MB AMD64 ReactOS paged pool budget");
    expect(frame_pool(8)==FramePool::nonpaged,"64-bit frames come from nonpaged pool");
    expect(frame_pool(4)==FramePool::paged,"i386 keeps its tested paged-pool frames");
    expect(mode_change_fits_pool(frame,512ull*1024*1024),"a large pool keeps the budget check satisfiable");
    expect(!mode_change_fits_pool(0,reactos_amd64_paged_pool_bytes),"an empty frame is not a valid budget");
    std::printf("Lightning staging pool: %u checks, %u failures\n",checks,failures);
    return failures==0 ? 0 : 1;
}

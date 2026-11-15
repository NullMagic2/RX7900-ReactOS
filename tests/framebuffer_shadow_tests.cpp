/*
Module: rx7900.tests.framebuffer_shadow
Purpose: Check mapped extent and dirty-region safety at native GDI boundaries.
Created: 2026-10-21
Architecture: Host-only tests reuse the native framebuffer policy.
*/
#include "reactos/framebuffer_shadow_policy.hpp"
#include "reactos/lightning_gdi_geometry.hpp"
#include "reactos/lightning_gdi_lifecycle.hpp"

#include <cstdio>
#include <cstdint>

namespace {
namespace policy = rx7900::reactos::framebuffer;
unsigned checks = 0;
unsigned failures = 0;

void check(bool result, const char* description) {
    ++checks;
    if (!result) {
        ++failures;
        std::fprintf(stderr, "FAIL: %s\n", description);
    }
}

bool same(policy::Rectangle a, policy::Rectangle b) {
    return a.left == b.left && a.top == b.top && a.right == b.right && a.bottom == b.bottom;
}
}

int main() {
    namespace gpu = lightning::reactos::gdi;
    gpu::PromotionSchedule promotion;
    promotion.configure(1000,50);
    check(!promotion.due(50) && !promotion.due(1049),"host capability probes wait one elapsed second");
    check(promotion.due(1050) && !promotion.due(1051),"one readiness probe cannot become a per-draw IOCTL");
    check(promotion.due(10) && !promotion.due(11),"counter rollback reschedules without unsigned underflow");
    promotion.configure(1000,UINT64_MAX-5U);
    check(!promotion.due(UINT64_MAX),"deadline arithmetic cannot overflow near the counter limit");
    promotion.configure(0,0);
    for (uint32_t i=0;i<255;++i) check(!promotion.due(0),"missing QPC uses a bounded operation budget");
    check(promotion.due(0) && !promotion.due(0),"fallback probes once per256 operations");
    gpu::Retirement retired{5,LIGHTNING_GDI_FLAG_RETIRED,0,0,0,0};
    check(gpu::retirement_confirmed(4,false,retired),"actual newer retired token releases owned GPU work");
    check(!gpu::retirement_confirmed(5,false,retired),"unchanged token cannot release owned work");
    check(gpu::retirement_confirmed(5,true,retired),"fresh never-used retired backend needs no fabricated reset");
    check(!gpu::retirement_confirmed(6,true,retired),"retirement tokens cannot roll backwards");
    check(!gpu::retirement_confirmed(UINT32_MAX,false,{1,LIGHTNING_GDI_FLAG_RETIRED,0,0,0,0}),
          "wrapped retirement token cannot reuse a quarantined transfer");
    check(!gpu::retirement_confirmed(0,true,{0,LIGHTNING_GDI_FLAG_RETIRED,0,0,0,0}),"zero token is not proof");
    check(!gpu::retirement_confirmed(4,false,{5,LIGHTNING_GDI_FLAG_RETIRED,0,0,0,0,LIGHTNING_GDI_LOST}),
          "LOST status never proves actual backend retirement");
    check(!gpu::retirement_confirmed(4,false,{5,LIGHTNING_GDI_FLAG_RETIRED|LIGHTNING_GDI_FLAG_READY,0,0,0,0}),
          "publication-ready state cannot also release a canvas");
    for (uint32_t field=0;field<7;++field) {
        gpu::Retirement unsafe=retired;
        switch (field) {
        case 0: unsafe.flags=LIGHTNING_GDI_FLAG_READY; break;
        case 1: unsafe.flags|=LIGHTNING_GDI_FLAG_RESETTING; break;
        case 2: unsafe.flags|=LIGHTNING_GDI_FLAG_POISONED; break;
        case 3: unsafe.epoch=1; break;
        case 4: unsafe.mode_generation=1; break;
        case 5: unsafe.accepted_sequence=1; break;
        default: unsafe.completed_sequence=1; break;
        }
        check(!gpu::retirement_confirmed(4,false,unsafe),"every remaining owner/reset/fault marker blocks reuse");
    }
    check(policy::valid_layout(1920, 1080, 7680, 8294400), "1080p exact mapping");
    check(policy::valid_layout(1920, 1080, 8192, 8847360), "padded stride mapping");
    check(!policy::valid_layout(1920, 1080, 7680, 8294399), "one-byte-short mapping");
    check(!policy::valid_layout(1920, 1080, 7679, 8294400), "one-byte-short row");
    check(!policy::valid_layout(1920, 1080, 7681, 8295480), "unaligned pitch would be rounded by GDI");
    check(!policy::valid_layout(1, 1, 0x20000000, 0xffffffffU), "GDI pitch bit conversion cannot wrap");
    check(!policy::valid_layout(0, 1080, 7680, 8294400), "zero width");
    check(!policy::valid_layout(1920, 0, 7680, 8294400), "zero height");
    check(!policy::valid_layout(1920, 1080, 0xffffe200U, 0xffffffffU), "negative signed stride");
    check(!policy::valid_layout(0xffffffffU, 2, 4, 8), "width multiplication cannot wrap");
    check(!policy::valid_layout(1, 0xffffffffU, 4, 0xffffffffU), "height multiplication cannot wrap");
    check(!policy::valid_layout(1, 0x20000000U, 4, 0xffffffffU), "engine signed byte extent");
    check(policy::valid_layout(1, 0x1fffffffU, 4, 0xffffffffU), "largest four-byte row product");

    const policy::Rectangle screen{0, 0, 1920, 1080};
    check(same(policy::clip({INT32_MIN, INT32_MIN, INT32_MAX, INT32_MAX}, 1920, 1080), screen),
          "extreme coordinates clip without signed overflow");
    check(same(policy::clip({INT32_MAX, INT32_MAX, INT32_MIN, INT32_MIN}, 1920, 1080), screen),
          "flipped extreme destination clips safely");
    check(same(policy::clip({1900, 1070, 2000, 1200}, 1920, 1080), {1900,1070,1920,1080}),
          "bottom-right dirty edge");
    check(same(policy::clip({-10, -20, 25, 35}, 1920, 1080), {0,0,25,35}), "top-left cursor clip");
    check(same(policy::clip({25, 35, -10, -20}, 1920, 1080), {0,0,25,35}), "mirrored cursor clip");
    check(policy::empty(policy::clip({-40,-30,-20,-10}, 1920, 1080)), "entirely offscreen negative");
    check(policy::empty(policy::clip({2000,1200,3000,1400}, 1920, 1080)), "entirely offscreen positive");
    check(policy::empty(policy::clip({3,4,3,6}, 1920, 1080)), "zero-width operation");
    check(policy::empty(policy::clip({3,4,5,4}, 1920, 1080)), "zero-height operation");

    const policy::Rectangle attempted = policy::clip({10,20,80,90},1920,1080);
    const policy::Rectangle clipped = policy::intersect(attempted, {20,10,70,85});
    check(same(clipped,{20,20,70,85}), "nontrivial GDI clip bounds constrain flush");
    check(policy::empty(policy::intersect(attempted, {100,100,110,110})), "disjoint clip flushes nothing");
    check(same(policy::unite(clipped,{1900,1070,1920,1080}), {20,20,1920,1080}),
          "retained failed dirty pixels merge with later writes");
    check(same(policy::unite({},clipped),clipped), "first dirty operation");
    check(same(policy::unite(clipped,{}),clipped), "empty draw preserves failed flush");

    uint32_t constant_color=0x12345678U;
    check(gpu::constant_fill_rop(0x0000U,constant_color) && constant_color==0,
          "BLACKNESS writes all32bits zero independently of brush/source/mask");
    check(gpu::constant_fill_rop(0xffffU,constant_color) && constant_color==UINT32_MAX,
          "WHITENESS preserves the engine's high alpha byte as well as whiteRGB");
    const uint32_t other_rops[]{0x00ffU,0xff00U,0xccccU,0xf0f0U,0x10000U,UINT32_MAX};
    for (const auto rop : other_rops) {
        check(!gpu::constant_fill_rop(rop,constant_color),
              "mixed/masked/copy/brush/unknownROP cannot become an unconditional fill");
    }

    policy::Rectangle gpu_bounds{};
    check(gpu::draw_bounds({-10,-20,90,80},1920,1080,gpu_bounds) &&
        same(gpu_bounds,{0,0,90,80}),"GPU target clips without unsigned-coordinate wrapping");
    int32_t source_x=-1;
    int32_t source_y=-1;
    check(gpu::source_position({-10,-20,90,80},gpu_bounds,0,0,100,100,source_x,source_y) &&
        source_x==10 && source_y==20,"clipped GPU copy preserves source correspondence");
    check(!gpu::source_position({INT32_MIN,0,1,1},{0,0,1,1},INT32_MAX,0,
        INT32_MAX,1,source_x,source_y),"source offset overflow cannot narrow into a valid GPU coordinate");
    check(!gpu::draw_bounds({10,10,0,0},1920,1080,gpu_bounds),"mirrored GDI copy uses engine fallback");
    check(!gpu::source_position({0,0,100,100},{0,0,100,100},1,0,100,100,source_x,source_y),
          "one-pixel-short source cannot enter offload");
    gpu::CacheVisibility visibility;
    check(visibility.readable(),"fresh cached canvas is CPU-readable");
    visibility.gpu_write({10,20,40,50});
    visibility.gpu_write({30,40,80,90});
    check(!visibility.readable() && same(visibility.stale(),{10,20,80,90}),
          "GPU writes retain complete stale bounds before CPU fallback");
    visibility.poison();
    visibility.readback_complete();
    check(!visibility.readable() && visibility.poisoned() && same(visibility.stale(),{10,20,80,90}),
          "uncertain GPU state cannot authorize CPU replay through a claimed readback");
    visibility.reinitialize();
    visibility.gpu_write({10,20,40,50});
    visibility.readback_complete();
    check(visibility.readable(),"confirmed readback restores cache visibility after explicit reinitialization");

    // Screen-sized and cursor-sized regions exercise realistic clipping edges,
    // both flips and disjoint clipping without ever producing out-of-bounds GDI
    // copy coordinates. Include INT32 endpoints to exercise undefined arithmetic.
    const int32_t values[]{INT32_MIN,-2000,-1,0,1,31,1079,1080,1919,1920,2000,INT32_MAX};
    for (int32_t left : values) {
        for (int32_t right : values) {
            for (int32_t top : values) {
                for (int32_t bottom : values) {
                    const auto result = policy::clip({left,top,right,bottom},1920,1080);
                    check(policy::empty(result) ||
                        (result.left >= 0 && result.top >= 0 && result.right <= 1920 &&
                         result.bottom <= 1080), "all clipped flushes stay inside the mapping");
                }
            }
        }
    }
    std::printf("Framebuffer shadow: %u checks, %u failures\n",checks,failures);
    return failures == 0 ? 0 : 1;
}

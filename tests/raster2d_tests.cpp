/*
Module: rx7900.tests.raster2d
Purpose: Run the generated GFX11 raster2d compute kernels in the shared ISA
interpreter (tests/gfx11_interp.hpp) on synthetic 32-bpp surfaces and compare them
pixel-exact to CPU references of the pinned ReactOS GDI engine semantics: all 256
ROP3 codes over random P/S/D, ROP4 with a 1-bpp mask, 8x8 pattern brushes with an
origin, monochrome and coverage8 glyph expansion, colour-key blits, AlphaBlend
(constant and per-pixel alpha, with rounding), nearest StretchBlt, styled and solid
cosmetic lines, integer gradient rectangles and triangles, mirrored ROP4
stretches (every EngStretchBltROP DIB call of tests/gdi_engine_reference.hpp, all
four flip combinations, clips, masks, tiles, device-source windows) and pattern
lines over all octants against the EngLineTo port. Also decodes the
Raster2dEngine PM4 IBs and checks the exact COMPUTE_* register values and
DISPATCH_DIRECT grid. Host-only; no GPU.
*/
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <memory>
#include <random>
#include <vector>

#include "raster2d.hpp"
#include "gfx11_interp.hpp"
#include "gdi_engine_reference.hpp"

namespace {

using namespace rx7900::hardware;
namespace sdma = rx7900::sdma;
namespace memory = rx7900::memory;

unsigned checks = 0;
unsigned failures = 0;

void expect(bool condition, const char* contract) {
    ++checks;

    if (!condition) {
        ++failures;
        std::fprintf(stderr, "FAIL: %s\n", contract);
    }
}

// ----------------------------------------------------------- kernel harness

// Dispatch one generated kernel over a rectangle onto synthetic memory. The code
// and every surface/args buffer are registered so the interpreter's V#/SMEM reads
// resolve. Returns the destination bytes after the dispatch.
struct Surface final {
    uint64_t gpu;
    uint32_t pitch;
    uint32_t width;
    uint32_t height;
    std::vector<uint8_t> bytes;
};

Surface make_surface(uint64_t gpu, uint32_t width, uint32_t height, uint32_t pitch = 0) {
    Surface s;
    s.gpu = gpu;
    s.width = width;
    s.height = height;
    s.pitch = pitch != 0 ? pitch : width * 4;
    s.bytes.assign(static_cast<size_t>(s.pitch) * height, 0);
    return s;
}

Raster2dSurface view(const Surface& s) {
    return {s.gpu, s.pitch, s.width, s.height};
}

// A full code image generated once.
struct Kernels final {
    std::vector<uint8_t> code;
    Raster2dCodeImage image;

    Kernels() {
        code.assign(raster2d_code_bytes, 0);
        const bool ok = raster2d_build_code(sdma::CommandBuffer(code.data(), code.size()), image);
        if (!ok) { std::fprintf(stderr, "FATAL: kernel generation failed\n"); }
    }
};

// Run a kernel: args block in its own buffer, surfaces registered separately.
void run_kernel(const Kernels& kernels, Raster2dKernel kernel, const uint32_t (&args)[64],
                const uint8_t* tail, uint32_t tail_dwords, const Raster2dRect& rect,
                std::vector<gfx11_interp::Buffer*>& buffers) {
    const Raster2dKernelInfo& info = kernels.image.kernels[static_cast<uint32_t>(kernel)];

    gfx11_interp::Buffer code_buffer;
    code_buffer.base = 0x80000000ull;
    code_buffer.bytes.assign(kernels.code.begin() + info.offset_bytes,
                             kernels.code.begin() + info.offset_bytes + info.size_bytes);

    gfx11_interp::Buffer args_buffer;
    args_buffer.base = 0x90000000ull;
    args_buffer.bytes.assign(reinterpret_cast<const uint8_t*>(args),
                             reinterpret_cast<const uint8_t*>(args) + 256);

    if (tail != nullptr && tail_dwords != 0) {
        const uint8_t* bytes = reinterpret_cast<const uint8_t*>(tail);
        args_buffer.bytes.insert(args_buffer.bytes.end(), bytes, bytes + tail_dwords * 4u);
    }

    args_buffer.bytes.resize(raster2d_args_region_bytes, 0);

    gfx11_interp::Memory mem;
    mem.buffers.push_back(&args_buffer);

    for (gfx11_interp::Buffer* b : buffers) {
        mem.buffers.push_back(b);
    }

    uint32_t groups_x = 0;
    uint32_t groups_y = 0;
    Raster2dEngine::grid(rect, groups_x, groups_y);

    for (uint32_t gy = 0; gy < groups_y; ++gy) {
        for (uint32_t gx = 0; gx < groups_x; ++gx) {
            for (uint32_t waveidx = 0; waveidx < 2; ++waveidx) {
                gfx11_interp::Wave w;
                w.exec = 0xffffffffu;
                w.sgpr[0] = static_cast<uint32_t>(args_buffer.base);
                w.sgpr[1] = static_cast<uint32_t>(args_buffer.base >> 32);
                w.sgpr[2] = gx;
                w.sgpr[3] = gy;

                for (uint32_t lane = 0; lane < gfx11_interp::lanes; ++lane) {
                    w.vgpr[0][lane] = waveidx * gfx11_interp::lanes + lane;
                }

                gfx11_interp::run(code_buffer.bytes.data(), info.size_bytes / 4u, w, &mem);
            }
        }
    }
}

gfx11_interp::Buffer* as_interp(Surface& s) {
    static std::vector<std::unique_ptr<gfx11_interp::Buffer>> pool;
    auto b = std::make_unique<gfx11_interp::Buffer>();
    b->base = s.gpu;
    b->bytes = s.bytes; // copy in
    gfx11_interp::Buffer* ptr = b.get();
    pool.push_back(std::move(b));
    return ptr;
}

// ------------------------------------------------------------ GDI references

// dib/dib.c DIB_DoRop: per-bit minterm selection by (P,S,D).
uint32_t cpu_rop(uint32_t rop3, uint32_t p, uint32_t s, uint32_t d) {
    uint32_t result = 0;

    for (uint32_t bit = 0; bit < 32; ++bit) {
        const uint32_t pb = (p >> bit) & 1u;
        const uint32_t sb = (s >> bit) & 1u;
        const uint32_t db = (d >> bit) & 1u;
        const uint32_t index = (pb << 2) | (sb << 1) | db;
        result |= ((rop3 >> index) & 1u) << bit;
    }

    return result;
}

uint32_t cpu_div255(uint32_t x) { return x / 255u; }

} // namespace

namespace {

// ---------------------------------------------------------------- ROP tests

void build_rop_descriptors(const Raster2dRopRequest& r, Raster2dDescriptors& d) {
    raster2d_buffer_descriptor(r.destination.gpu_address,
                               r.destination.pitch_bytes * r.destination.height, d.destination);
    if (r.has_source) {
        raster2d_buffer_descriptor(r.source.gpu_address, r.source.pitch_bytes * r.source.height,
                                   d.source);
    }
    if (r.has_mask) {
        raster2d_buffer_descriptor(r.mask.gpu_address, r.mask.pitch_bytes * r.mask.height, d.mask);
    }
}

void test_rop_all_codes(const Kernels& kernels) {
    std::mt19937 rng(12345);
    const uint32_t w = 20;
    const uint32_t h = 4;

    uint32_t mismatches = 0;

    for (uint32_t rop3 = 0; rop3 < 256; ++rop3) {
        Surface dst = make_surface(0x100000, w, h);
        Surface src = make_surface(0x200000, w, h);
        std::vector<uint32_t> pattern(64);
        std::vector<uint32_t> dst_ref(static_cast<size_t>(w) * h);

        for (auto& value : pattern) {
            value = rng();
        }

        for (uint32_t y = 0; y < h; ++y) {
            for (uint32_t x = 0; x < w; ++x) {
                const uint32_t dvalue = rng();
                const uint32_t svalue = rng();
                std::memcpy(dst.bytes.data() + static_cast<size_t>(y) * dst.pitch + x * 4, &dvalue, 4);
                std::memcpy(src.bytes.data() + static_cast<size_t>(y) * src.pitch + x * 4, &svalue, 4);
                const uint32_t pvalue = pattern[(y & 7) * 8 + (x & 7)];
                dst_ref[y * w + x] = cpu_rop(rop3, pvalue, svalue, dvalue);
            }
        }

        Raster2dRopRequest r;
        r.destination = view(dst);
        r.destination_rect = {0, 0, static_cast<int32_t>(w), static_cast<int32_t>(h)};
        r.has_source = true;
        r.source = view(src);
        r.source_left = 0;
        r.source_top = 0;
        r.source_width = static_cast<int32_t>(w);
        r.source_height = static_cast<int32_t>(h);
        r.has_pattern = true;
        r.pattern = pattern.data();
        r.brush_origin = {0, 0};
        r.rop4 = rop3 | (rop3 << 8);

        Raster2dDescriptors d{};
        build_rop_descriptors(r, d);
        uint32_t args[64];
        raster2d_build_rop_args(r, r.destination_rect, d, args);
        // Point the pattern descriptor at the tail tile.
        raster2d_buffer_descriptor(0x90000000ull + raster2d_args_bytes, raster2d_pattern_bytes,
                                   *reinterpret_cast<uint32_t(*)[4]>(args + rop_args::pattern));

        gfx11_interp::Buffer* dstb = as_interp(dst);
        gfx11_interp::Buffer* srcb = as_interp(src);
        std::vector<gfx11_interp::Buffer*> buffers = {dstb, srcb};
        run_kernel(kernels, Raster2dKernel::rop, args,
                   reinterpret_cast<const uint8_t*>(pattern.data()), 64, r.destination_rect, buffers);

        for (uint32_t y = 0; y < h; ++y) {
            for (uint32_t x = 0; x < w; ++x) {
                uint32_t got = 0;
                std::memcpy(&got, dstb->bytes.data() + static_cast<size_t>(y) * dst.pitch + x * 4, 4);
                if (got != dst_ref[y * w + x]) {
                    ++mismatches;
                }
            }
        }
    }

    expect(mismatches == 0, "all 256 ROP3 codes over P/S/D match DIB_DoRop");
}

void test_pattern_origin(const Kernels& kernels) {
    const uint32_t w = 24;
    const uint32_t h = 10;
    Surface dst = make_surface(0x100000, w, h);
    std::vector<uint32_t> pattern(64);

    for (uint32_t i = 0; i < 64; ++i) {
        pattern[i] = 0xff000000u | (i * 0x030507u);
    }

    Raster2dRopRequest r;
    r.destination = view(dst);
    r.destination_rect = {0, 0, static_cast<int32_t>(w), static_cast<int32_t>(h)};
    r.has_pattern = true;
    r.pattern = pattern.data();
    r.brush_origin = {3, 5};
    r.rop4 = 0xf0f0; // PATCOPY

    Raster2dDescriptors d{};
    build_rop_descriptors(r, d);
    uint32_t args[64];
    raster2d_build_rop_args(r, r.destination_rect, d, args);
    raster2d_buffer_descriptor(0x90000000ull + raster2d_args_bytes, raster2d_pattern_bytes,
                               *reinterpret_cast<uint32_t(*)[4]>(args + rop_args::pattern));

    gfx11_interp::Buffer* dstb = as_interp(dst);
    std::vector<gfx11_interp::Buffer*> buffers = {dstb};
    run_kernel(kernels, Raster2dKernel::rop, args,
               reinterpret_cast<const uint8_t*>(pattern.data()), 64, r.destination_rect, buffers);

    uint32_t mismatches = 0;

    for (int32_t y = 0; y < static_cast<int32_t>(h); ++y) {
        for (int32_t x = 0; x < static_cast<int32_t>(w); ++x) {
            const uint32_t px = ((x - 3) & 7);
            const uint32_t py = ((y - 5) & 7);
            uint32_t got = 0;
            std::memcpy(&got, dstb->bytes.data() + static_cast<size_t>(y) * dst.pitch + x * 4, 4);
            if (got != pattern[py * 8 + px]) {
                ++mismatches;
            }
        }
    }

    expect(mismatches == 0, "8x8 pattern brush with origin (PATCOPY)");
}

void test_mono_glyph(const Kernels& kernels) {
    const uint32_t w = 16;
    const uint32_t h = 8;
    Surface dst = make_surface(0x100000, w, h);
    // 1-bpp source: w bits per row, MSB first, pitch = 4 bytes (>= 2).
    Surface src = make_surface(0x200000, 1, h, 4);
    src.width = w; // treat as w-wide 1bpp

    std::mt19937 rng(7);
    std::vector<uint8_t> bits(static_cast<size_t>(4) * h);

    for (auto& b : bits) {
        b = static_cast<uint8_t>(rng());
    }

    std::memcpy(src.bytes.data(), bits.data(), bits.size());

    const uint32_t fg = 0xffeeddccu;
    const uint32_t bg = 0xff112233u;

    Raster2dRopRequest r;
    r.destination = view(dst);
    r.destination_rect = {0, 0, static_cast<int32_t>(w), static_cast<int32_t>(h)};
    r.has_source = true;
    r.source_monochrome = true;
    r.source = view(src);
    r.source.pitch_bytes = 4;
    r.source_width = static_cast<int32_t>(w);
    r.source_height = static_cast<int32_t>(h);
    r.mono_foreground = fg;
    r.mono_background = bg;
    r.rop4 = 0xcccc; // SRCCOPY

    Raster2dDescriptors d{};
    raster2d_buffer_descriptor(dst.gpu, dst.pitch * dst.height, d.destination);
    raster2d_buffer_descriptor(src.gpu, src.pitch * src.height, d.source);
    uint32_t args[64];
    raster2d_build_rop_args(r, r.destination_rect, d, args);

    gfx11_interp::Buffer* dstb = as_interp(dst);
    gfx11_interp::Buffer* srcb = as_interp(src);
    std::vector<gfx11_interp::Buffer*> buffers = {dstb, srcb};
    run_kernel(kernels, Raster2dKernel::rop, args, nullptr, 0, r.destination_rect, buffers);

    uint32_t mismatches = 0;

    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            const uint32_t byte = bits[y * 4 + (x >> 3)];
            const uint32_t set = (byte >> (7u - (x & 7u))) & 1u;
            const uint32_t want = set ? fg : bg;
            uint32_t got = 0;
            std::memcpy(&got, dstb->bytes.data() + static_cast<size_t>(y) * dst.pitch + x * 4, 4);
            if (got != want) {
                ++mismatches;
            }
        }
    }

    expect(mismatches == 0, "monochrome glyph expansion (fg/bg)");
}

void test_colour_key(const Kernels& kernels) {
    const uint32_t w = 12;
    const uint32_t h = 4;
    Surface dst = make_surface(0x100000, w, h);
    Surface src = make_surface(0x200000, w, h);
    const uint32_t key = 0x00ff00ffu; // magenta transparent

    std::mt19937 rng(99);
    std::vector<uint32_t> dref(static_cast<size_t>(w) * h);

    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            uint32_t dvalue = rng() | 0xff000000u;
            uint32_t svalue = (x % 3 == 0) ? (0xff000000u | key) : (rng() & 0x00ffffffu);
            std::memcpy(dst.bytes.data() + static_cast<size_t>(y) * dst.pitch + x * 4, &dvalue, 4);
            std::memcpy(src.bytes.data() + static_cast<size_t>(y) * src.pitch + x * 4, &svalue, 4);
            const bool transparent = (svalue & 0x00ffffffu) == (key & 0x00ffffffu);
            dref[y * w + x] = transparent ? dvalue : svalue;
        }
    }

    Raster2dRopRequest r;
    r.destination = view(dst);
    r.destination_rect = {0, 0, static_cast<int32_t>(w), static_cast<int32_t>(h)};
    r.has_source = true;
    r.source = view(src);
    r.source_width = static_cast<int32_t>(w);
    r.source_height = static_cast<int32_t>(h);
    r.has_colour_key = true;
    r.colour_key = key;
    r.rop4 = 0xcccc;

    Raster2dDescriptors d{};
    build_rop_descriptors(r, d);
    uint32_t args[64];
    raster2d_build_rop_args(r, r.destination_rect, d, args);

    gfx11_interp::Buffer* dstb = as_interp(dst);
    gfx11_interp::Buffer* srcb = as_interp(src);
    std::vector<gfx11_interp::Buffer*> buffers = {dstb, srcb};
    run_kernel(kernels, Raster2dKernel::rop, args, nullptr, 0, r.destination_rect, buffers);

    uint32_t mismatches = 0;

    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            uint32_t got = 0;
            std::memcpy(&got, dstb->bytes.data() + static_cast<size_t>(y) * dst.pitch + x * 4, 4);
            if (got != dref[y * w + x]) {
                ++mismatches;
            }
        }
    }

    expect(mismatches == 0, "colour-key (transparent) blit");
}

void test_stretch(const Kernels& kernels) {
    const uint32_t sw = 4, sh = 3;
    const uint32_t dw = 13, dh = 9;
    Surface dst = make_surface(0x100000, dw, dh);
    Surface src = make_surface(0x200000, sw, sh);

    std::mt19937 rng(5);

    for (auto& b : src.bytes) {
        b = static_cast<uint8_t>(rng());
    }

    Raster2dRopRequest r;
    r.destination = view(dst);
    r.destination_rect = {0, 0, static_cast<int32_t>(dw), static_cast<int32_t>(dh)};
    r.has_source = true;
    r.source = view(src);
    r.source_left = 0;
    r.source_top = 0;
    r.source_width = static_cast<int32_t>(sw);
    r.source_height = static_cast<int32_t>(sh);
    r.rop4 = 0xcccc;

    Raster2dDescriptors d{};
    build_rop_descriptors(r, d);
    uint32_t args[64];
    raster2d_build_rop_args(r, r.destination_rect, d, args);

    gfx11_interp::Buffer* dstb = as_interp(dst);
    gfx11_interp::Buffer* srcb = as_interp(src);
    std::vector<gfx11_interp::Buffer*> buffers = {dstb, srcb};
    run_kernel(kernels, Raster2dKernel::rop, args, nullptr, 0, r.destination_rect, buffers);

    uint32_t mismatches = 0;

    for (uint32_t y = 0; y < dh; ++y) {
        for (uint32_t x = 0; x < dw; ++x) {
            // dib/stretchblt.c nearest: sx = left + x*SrcWidth/DstWidth.
            const uint32_t sx = 0 + x * sw / dw;
            const uint32_t sy = 0 + y * sh / dh;
            uint32_t want = 0;
            std::memcpy(&want, src.bytes.data() + static_cast<size_t>(sy) * src.pitch + sx * 4, 4);
            uint32_t got = 0;
            std::memcpy(&got, dstb->bytes.data() + static_cast<size_t>(y) * dst.pitch + x * 4, 4);
            if (got != want) {
                ++mismatches;
            }
        }
    }

    expect(mismatches == 0, "nearest StretchBlt (COLORONCOLOR) mapping");
}

void test_alpha(const Kernels& kernels, bool source_alpha) {
    const uint32_t w = 10;
    const uint32_t h = 4;
    Surface dst = make_surface(0x100000, w, h);
    Surface src = make_surface(0x200000, w, h);
    const uint32_t ca = source_alpha ? 200 : 160;

    std::mt19937 rng(source_alpha ? 1 : 2);
    std::vector<uint32_t> ref(static_cast<size_t>(w) * h);

    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            const uint32_t dvalue = rng();
            const uint32_t svalue = rng();
            std::memcpy(dst.bytes.data() + static_cast<size_t>(y) * dst.pitch + x * 4, &dvalue, 4);
            std::memcpy(src.bytes.data() + static_cast<size_t>(y) * src.pitch + x * 4, &svalue, 4);

            uint32_t sc[4];
            uint32_t dc[4];

            for (uint32_t c = 0; c < 4; ++c) {
                sc[c] = (svalue >> (8 * c)) & 0xff;
                dc[c] = (dvalue >> (8 * c)) & 0xff;
            }

            // dib/dib32bpp.c DIB_32BPP_AlphaBlend.
            for (uint32_t c = 0; c < 4; ++c) {
                sc[c] = cpu_div255(sc[c] * ca);
            }

            const uint32_t alpha = source_alpha ? sc[3] : ca;
            uint32_t out = 0;

            for (uint32_t c = 0; c < 4; ++c) {
                uint32_t v = cpu_div255(dc[c] * (255 - alpha)) + sc[c];
                v = v > 255 ? 255 : v;
                out |= v << (8 * c);
            }

            ref[y * w + x] = out;
        }
    }

    Raster2dAlphaRequest r;
    r.destination = view(dst);
    r.destination_rect = {0, 0, static_cast<int32_t>(w), static_cast<int32_t>(h)};
    r.source = view(src);
    r.source_width = static_cast<int32_t>(w);
    r.source_height = static_cast<int32_t>(h);
    r.constant_alpha = ca;
    r.source_alpha = source_alpha;

    Raster2dDescriptors d{};
    raster2d_buffer_descriptor(dst.gpu, dst.pitch * dst.height, d.destination);
    raster2d_buffer_descriptor(src.gpu, src.pitch * src.height, d.source);
    uint32_t args[64];
    raster2d_build_alpha_args(r, r.destination_rect, d, args);

    gfx11_interp::Buffer* dstb = as_interp(dst);
    gfx11_interp::Buffer* srcb = as_interp(src);
    std::vector<gfx11_interp::Buffer*> buffers = {dstb, srcb};
    run_kernel(kernels, Raster2dKernel::alpha, args, nullptr, 0, r.destination_rect, buffers);

    uint32_t mismatches = 0;

    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            uint32_t got = 0;
            std::memcpy(&got, dstb->bytes.data() + static_cast<size_t>(y) * dst.pitch + x * 4, 4);
            if (got != ref[y * w + x]) {
                ++mismatches;
            }
        }
    }

    expect(mismatches == 0, source_alpha ? "AlphaBlend AC_SRC_ALPHA (premultiplied)"
                                         : "AlphaBlend SourceConstantAlpha");
}

void test_coverage8(const Kernels& kernels) {
    const uint32_t w = 12;
    const uint32_t h = 3;
    Surface dst = make_surface(0x100000, w, h);
    Surface src = make_surface(0x200000, w, h, w); // 8-bpp, pitch = w
    const uint32_t solid = 0x00123456u;

    std::mt19937 rng(42);
    std::vector<uint32_t> ref(static_cast<size_t>(w) * h);

    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            const uint32_t dvalue = rng() & 0x00ffffffu;
            const uint8_t cov = static_cast<uint8_t>(rng());
            std::memcpy(dst.bytes.data() + static_cast<size_t>(y) * dst.pitch + x * 4, &dvalue, 4);
            src.bytes[static_cast<size_t>(y) * src.pitch + x] = cov;

            // eng/bitblt.c EngMaskBitBlt 8-bpp path.
            uint32_t out = dvalue;

            if (cov == 0xff) {
                out = solid;
            } else if (cov > 0) {
                const int r0 = (solid >> 16) & 0xff, g0 = (solid >> 8) & 0xff, b0 = solid & 0xff;
                const int rd = (dvalue >> 16) & 0xff, gd = (dvalue >> 8) & 0xff, bd = dvalue & 0xff;
                const int rr = (int(cov) * (r0 - rd) >> 8) + rd;
                const int gg = (int(cov) * (g0 - gd) >> 8) + gd;
                const int bb = (int(cov) * (b0 - bd) >> 8) + bd;
                out = (uint32_t(rr & 0xff) << 16) | (uint32_t(gg & 0xff) << 8) | uint32_t(bb & 0xff);
            }

            ref[y * w + x] = out;
        }
    }

    Raster2dAlphaRequest r;
    r.destination = view(dst);
    r.destination_rect = {0, 0, static_cast<int32_t>(w), static_cast<int32_t>(h)};
    r.source = view(src);
    r.source.pitch_bytes = w;
    r.source_width = static_cast<int32_t>(w);
    r.source_height = static_cast<int32_t>(h);
    r.coverage8 = true;
    r.solid_colour = solid;

    Raster2dDescriptors d{};
    raster2d_buffer_descriptor(dst.gpu, dst.pitch * dst.height, d.destination);
    raster2d_buffer_descriptor(src.gpu, src.pitch * src.height, d.source);
    uint32_t args[64];
    raster2d_build_alpha_args(r, r.destination_rect, d, args);

    gfx11_interp::Buffer* dstb = as_interp(dst);
    gfx11_interp::Buffer* srcb = as_interp(src);
    std::vector<gfx11_interp::Buffer*> buffers = {dstb, srcb};
    run_kernel(kernels, Raster2dKernel::alpha, args, nullptr, 0, r.destination_rect, buffers);

    uint32_t mismatches = 0;

    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            uint32_t got = 0;
            std::memcpy(&got, dstb->bytes.data() + static_cast<size_t>(y) * dst.pitch + x * 4, 4);
            if ((got & 0x00ffffffu) != (ref[y * w + x] & 0x00ffffffu)) {
                ++mismatches;
            }
        }
    }

    expect(mismatches == 0, "coverage8 glyph composition (EngMaskBitBlt)");
}

void test_gradient_rect(const Kernels& kernels, bool horizontal) {
    const uint32_t w = 32;
    const uint32_t h = 8;
    Surface dst = make_surface(0x100000, w, h);

    const uint16_t c0[3] = {0x1000, 0x2000, 0x8000};
    const uint16_t c1[3] = {0xf000, 0x4000, 0x0000};
    const uint32_t span = horizontal ? w : h;

    // GDI HVINITCOL/HVSTEPCOL reference.
    auto ramp = [&](uint32_t step, uint32_t channel) -> uint32_t {
        int32_t c = c0[channel] >> 8;
        const int32_t dc = std::abs((c1[channel] >> 8) - c);
        const int32_t dy = static_cast<int32_t>(span);
        int32_t ec = -(dy >> 1);
        const int32_t ic = (c1[channel] >> 8) > c ? 1 : -1;

        for (uint32_t k = 0; k < step; ++k) {
            ec += dc;

            while (ec > 0) {
                c += ic;
                ec -= dy;
            }
        }

        return static_cast<uint32_t>(c & 0xff);
    };

    Raster2dGradientRectRequest r;
    r.destination = view(dst);
    r.destination_rect = {0, 0, static_cast<int32_t>(w), static_cast<int32_t>(h)};
    r.horizontal = horizontal;
    r.origin = 0;
    r.divisor = span;

    for (uint32_t c = 0; c < 3; ++c) {
        r.colour0[c] = c0[c];
        r.colour1[c] = c1[c];
    }

    Raster2dDescriptors d{};
    raster2d_buffer_descriptor(dst.gpu, dst.pitch * dst.height, d.destination);
    uint32_t args[64];
    raster2d_build_gradient_rect_args(r, r.destination_rect, d, args);

    gfx11_interp::Buffer* dstb = as_interp(dst);
    std::vector<gfx11_interp::Buffer*> buffers = {dstb};
    run_kernel(kernels, Raster2dKernel::gradient_rect, args, nullptr, 0, r.destination_rect, buffers);

    uint32_t mismatches = 0;

    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            const uint32_t step = horizontal ? x : y;
            const uint32_t want = (ramp(step, 0) << 16) | (ramp(step, 1) << 8) | ramp(step, 2);
            uint32_t got = 0;
            std::memcpy(&got, dstb->bytes.data() + static_cast<size_t>(y) * dst.pitch + x * 4, 4);
            if ((got & 0x00ffffffu) != want) {
                ++mismatches;
            }
        }
    }

    expect(mismatches == 0, horizontal ? "gradient rect horizontal (HVSTEPCOL)"
                                       : "gradient rect vertical (HVSTEPCOL)");
}

void test_divisor_identity() {
    std::mt19937 rng(321);
    uint32_t bad = 0;

    for (uint32_t trial = 0; trial < 20000; ++trial) {
        const uint32_t divisor = (rng() % 20000) + 1;
        const uint32_t n = rng() % 2000000u;
        const Raster2dDivisor params = raster2d_divisor(divisor);

        if (raster2d_divide(n, params) != n / divisor) {
            ++bad;
        }
    }

    expect(bad == 0, "Granlund-Montgomery divisor matches integer division");
}

void test_div255_range() {
    uint32_t bad = 0;

    for (uint32_t x = 0; x <= 65535; ++x) {
        if (((x * 0x8081u) >> 23) != x / 255u) {
            ++bad;
        }
    }

    expect(bad == 0, "the kernel's x/255 approximation is exact for x in [0, 65535]");
}

void test_line(const Kernels& kernels, int32_t x0, int32_t y0, int32_t x1, int32_t y1,
               const char* name) {
    const uint32_t w = 48;
    const uint32_t h = 48;
    Surface dst = make_surface(0x100000, w, h);
    const uint32_t colour = 0x00abcdefu;

    Raster2dLineRequest r;
    r.destination = view(dst);
    r.clip_bounds = {0, 0, static_cast<int32_t>(w), static_cast<int32_t>(h)};
    r.x0 = x0;
    r.y0 = y0;
    r.x1 = x1;
    r.y1 = y1;
    r.colour = colour;
    r.rop3 = 0xf0; // PATCOPY: write the pen colour

    // eng/lineto.c reference: Bresenham from (x0,y0), endpoint excluded, start
    // included, plotting each major-axis step.
    const int32_t dx = x1 - x0;
    const int32_t dy = y1 - y0;
    const int32_t adx = std::abs(dx);
    const int32_t ady = std::abs(dy);
    const int32_t major = std::max(adx, ady);
    const int32_t minor = std::min(adx, ady);
    const bool x_major = adx >= ady;
    const int32_t xchange = dx < 0 ? -1 : 1;
    const int32_t ychange = dy < 0 ? -1 : 1;

    std::vector<uint8_t> ref(static_cast<size_t>(w) * h, 0);

    for (int32_t k = 0; k < major; ++k) {
        const int32_t m = (static_cast<int32_t>(major >> 1) + k * minor) / (major == 0 ? 1 : major);
        const int32_t px = x_major ? x0 + xchange * k : x0 + xchange * m;
        const int32_t py = x_major ? y0 + ychange * m : y0 + ychange * k;

        if (px >= 0 && px < static_cast<int32_t>(w) && py >= 0 && py < static_cast<int32_t>(h)) {
            ref[static_cast<size_t>(py) * w + px] = 1;
        }
    }

    Raster2dDescriptors d{};
    raster2d_buffer_descriptor(dst.gpu, dst.pitch * dst.height, d.destination);
    uint32_t args[64];
    uint32_t steps = 0;
    raster2d_build_line_args(r, d, args, steps);

    const uint32_t groups = (steps + raster2d_workgroup_threads - 1) / raster2d_workgroup_threads;
    Raster2dRect grid_rect{0, 0, static_cast<int32_t>(groups * raster2d_workgroup_threads), 1};

    gfx11_interp::Buffer* dstb = as_interp(dst);
    std::vector<gfx11_interp::Buffer*> buffers = {dstb};
    run_kernel(kernels, Raster2dKernel::line, args, nullptr, 0, grid_rect, buffers);

    uint32_t mismatches = 0;

    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            uint32_t got = 0;
            std::memcpy(&got, dstb->bytes.data() + static_cast<size_t>(y) * dst.pitch + x * 4, 4);
            const bool drawn = (got & 0x00ffffffu) == colour;
            if (drawn != (ref[y * w + x] != 0)) {
                ++mismatches;
            }
        }
    }

    expect(mismatches == 0, name);
}

void test_gradient_triangle(const Kernels& kernels) {
    const uint32_t w = 40;
    const uint32_t h = 40;
    Surface dst = make_surface(0x100000, w, h);

    Raster2dTriVertex v1{5, 4, 0xff00, 0x0000, 0x0000};
    Raster2dTriVertex v2{34, 12, 0x0000, 0xff00, 0x0000};
    Raster2dTriVertex v3{14, 33, 0x0000, 0x0000, 0xff00};

    Raster2dGradientSpan rows[64][raster2d_triangle_spans];
    Raster2dRect extents{};
    uint32_t count = 0;
    const bool planned = raster2d_plan_triangle(v1, v2, v3, rows, 64, extents, count);
    expect(planned && count > 0, "triangle planner produced rows");

    if (count == 0) {
        return;
    }

    // CPU reference: evaluate the same spans with the kernel's arithmetic.
    auto channel_value = [](const Raster2dGradientSpan& s, int32_t x, uint32_t c) -> uint32_t {
        const uint32_t gx = s.divisor;
        const int32_t step = s.reverse ? (s.high - x) : (x - s.low);
        const Raster2dDivisor div = raster2d_divisor(gx);
        const uint32_t numerator = static_cast<uint32_t>(step) * static_cast<uint32_t>(s.delta[c]) +
                                   static_cast<uint32_t>(s.bias);
        const uint32_t n = raster2d_divide(numerator, div);
        const int32_t value = s.negative[c] ? static_cast<int32_t>(s.colour[c]) - static_cast<int32_t>(n)
                                            : static_cast<int32_t>(s.colour[c]) + static_cast<int32_t>(n);
        return static_cast<uint32_t>(value) & 0xffu;
    };

    std::vector<uint32_t> ref(static_cast<size_t>(w) * h, 0xdeadbeef);

    for (uint32_t row = 0; row < count; ++row) {
        const int32_t y = extents.top + static_cast<int32_t>(row);

        if (y < 0 || y >= static_cast<int32_t>(h)) {
            continue;
        }

        for (int32_t x = 0; x < static_cast<int32_t>(w); ++x) {
            uint32_t value = 0xdeadbeef;

            for (uint32_t s = 0; s < raster2d_triangle_spans; ++s) {
                const Raster2dGradientSpan& span = rows[row][s];

                if (span.low <= span.high && x >= span.low && x <= span.high) {
                    value = (channel_value(span, x, 0) << 16) | (channel_value(span, x, 1) << 8) |
                            channel_value(span, x, 2);
                }
            }

            ref[static_cast<size_t>(y) * w + x] = value;
        }
    }

    // Run one row-dispatch per row through the triangle kernel.
    gfx11_interp::Buffer* dstb = as_interp(dst);

    for (uint32_t row = 0; row < count; ++row) {
        const int32_t y = extents.top + static_cast<int32_t>(row);

        if (y < 0 || y >= static_cast<int32_t>(h)) {
            continue;
        }

        uint32_t args[64];

        for (uint32_t& value : args) {
            value = 0;
        }

        raster2d_buffer_descriptor(dst.gpu, dst.pitch * dst.height,
                                   *reinterpret_cast<uint32_t(*)[4]>(args + triangle_args::destination));
        args[triangle_args::left] = static_cast<uint32_t>(extents.left < 0 ? 0 : extents.left);
        args[triangle_args::top] = static_cast<uint32_t>(y);
        args[triangle_args::right] = static_cast<uint32_t>(extents.right > static_cast<int32_t>(w)
                                                              ? w : static_cast<uint32_t>(extents.right));
        args[triangle_args::destination_pitch] = dst.pitch;
        args[triangle_args::row_stride] = 0;
        {
            uint32_t desc[4];
            raster2d_buffer_descriptor(0x90000000ull + raster2d_args_bytes, raster2d_triangle_row_bytes,
                                       desc);
            args[triangle_args::table_low] = desc[0];
            args[triangle_args::table_high] = desc[1];
        }

        uint32_t tail[raster2d_triangle_spans * raster2d_span_dwords] = {};

        for (uint32_t s = 0; s < raster2d_triangle_spans; ++s) {
            const Raster2dGradientSpan& span = rows[row][s];
            uint32_t* rec = tail + s * raster2d_span_dwords;
            rec[triangle_args::span_low] = static_cast<uint32_t>(span.low);
            rec[triangle_args::span_high] = static_cast<uint32_t>(span.high);
            rec[triangle_args::span_reverse] = span.reverse ? 0xffffffffu : 0u;
            rec[triangle_args::span_bias] = static_cast<uint32_t>(span.bias);
            const Raster2dDivisor div = raster2d_divisor(span.divisor);
            rec[triangle_args::span_magic] = div.magic;
            rec[triangle_args::span_shift1] = div.shift1;
            rec[triangle_args::span_shift2] = div.shift2;

            for (uint32_t c = 0; c < 3; ++c) {
                rec[triangle_args::span_color + c] = span.colour[c];
                rec[triangle_args::span_delta + c] = static_cast<uint32_t>(span.delta[c]);
                rec[triangle_args::span_negative + c] = span.negative[c] ? 0xffffffffu : 0u;
            }
        }

        Raster2dRect rect{args[triangle_args::left] > 0 ? static_cast<int32_t>(args[triangle_args::left]) : 0,
                          y, static_cast<int32_t>(args[triangle_args::right]), y + 1};
        std::vector<gfx11_interp::Buffer*> buffers = {dstb};
        run_kernel(kernels, Raster2dKernel::gradient_triangle, args,
                   reinterpret_cast<const uint8_t*>(tail),
                   raster2d_triangle_spans * raster2d_span_dwords, rect, buffers);
    }

    uint32_t mismatches = 0;
    uint32_t covered = 0;

    for (uint32_t row = 0; row < count; ++row) {
        const int32_t y = extents.top + static_cast<int32_t>(row);

        if (y < 0 || y >= static_cast<int32_t>(h)) {
            continue;
        }

        for (int32_t x = 0; x < static_cast<int32_t>(w); ++x) {
            const uint32_t want = ref[static_cast<size_t>(y) * w + x];

            if (want == 0xdeadbeef) {
                continue;   // outside every span: destination unchanged
            }

            ++covered;
            uint32_t got = 0;
            std::memcpy(&got, dstb->bytes.data() + static_cast<size_t>(y) * dst.pitch + x * 4, 4);

            if ((got & 0x00ffffffu) != (want & 0x00ffffffu)) {
                ++mismatches;
            }
        }
    }

    expect(mismatches == 0 && covered > 0, "gradient triangle spans match the planner");
}

// ----------------------------------------------------- PM4 IB engine checks

// A test memory backend for memory::Manager: hands out one contiguous arena with
// CPU backing. No real GPU mapping; create always succeeds.
struct TestBackend final {
    std::vector<std::vector<uint8_t>> storage;

    static memory::CreateResult create(void* ctx, sdma::GpuAllocation alloc, memory::Access,
                                       bool cpu_visible, memory::Backing& backing) noexcept {
        auto* self = static_cast<TestBackend*>(ctx);
        self->storage.emplace_back(static_cast<size_t>(alloc.byte_length), 0);
        backing.token = self;
        backing.cpu_bytes = cpu_visible ? self->storage.back().data() : nullptr;
        backing.cpu_byte_count = cpu_visible ? self->storage.back().size() : 0;
        return memory::CreateResult::created;
    }

    static bool destroy(void*, const memory::Backing&, sdma::GpuAllocation) noexcept { return true; }
};

struct CapturedSubmit final {
    std::vector<uint32_t> dwords;
    const uint8_t* ib_cpu = nullptr;
    uint32_t sequence = 0;
    bool submitted = false;

    static Raster2dSubmitResult submit(void* ctx, memory::Handle, uint64_t offset, uint32_t count,
                                       uint32_t& sequence) noexcept {
        auto* self = static_cast<CapturedSubmit*>(ctx);
        self->submitted = true;
        sequence = ++self->sequence;
        self->dwords.resize(count);
        for (uint32_t i = 0; i < count; ++i) {
            uint32_t value = 0;
            std::memcpy(&value, self->ib_cpu + offset + i * 4u, 4);
            self->dwords[i] = value;
        }
        return Raster2dSubmitResult::submitted;
    }
    static Raster2dFenceState query(void*, uint32_t) noexcept { return Raster2dFenceState::signalled; }
    static Raster2dFenceState wait(void*, uint32_t, uint64_t) noexcept {
        return Raster2dFenceState::signalled;
    }
};

void test_pm4_ib(const Kernels& kernels) {
    TestBackend backend;
    memory::Manager manager;
    const uint64_t arena_base = 0x10000000ull;
    expect(manager.initialize({arena_base, 64u << 20},
                              {&backend, &TestBackend::create, &TestBackend::destroy}) ==
               memory::Status::success,
           "test manager initializes");

    memory::Handle code_h{};
    memory::Handle args_h{};
    memory::Handle ib_h{};
    const uint32_t ib_count = 2;
    auto page_up = [](uint64_t v) { return (v + 4095u) & ~uint64_t{4095u}; };
    manager.allocate(page_up(raster2d_code_bytes), 4096, memory::Access::read, true, code_h);
    manager.allocate(page_up(uint64_t{raster2d_args_bytes_total} * ib_count), 4096,
                     memory::Access::read, true, args_h);
    manager.allocate(page_up(uint64_t{raster2d_min_ib_bytes} * ib_count), 4096, memory::Access::read,
                     true, ib_h);

    CapturedSubmit capture;
    Raster2dEngine engine;
    Raster2dEngineConfig config{};
    config.manager = &manager;
    config.code = code_h;
    config.args = args_h;
    config.ib = ib_h;
    config.ib_count = ib_count;
    config.ib_bytes = raster2d_min_ib_bytes;
    config.submitter = {&capture, &CapturedSubmit::submit, &CapturedSubmit::query,
                        &CapturedSubmit::wait};
    config.wait_timeout_us = 1000;

    const Raster2dStatus init = engine.initialize(config);
    expect(init == Raster2dStatus::success, "engine initializes and generates kernels");

    memory::AllocationInfo code_info{};
    manager.describe(code_h, code_info);
    memory::AllocationInfo args_info{};
    manager.describe(args_h, args_info);
    memory::AllocationInfo ib_info{};
    manager.describe(ib_h, ib_info);
    capture.ib_cpu = ib_info.backing.cpu_bytes;

    // One fill (solid PATCOPY) over a VRAM destination not overlapping engine memory.
    Raster2dRopRequest r;
    r.destination = {0x40000000ull, 256, 64, 16};
    r.destination_rect = {0, 0, 64, 16};
    r.has_pattern = false;
    r.brush_colour = 0x00c0ffeeu;
    r.rop4 = 0xf0f0;

    Raster2dClipList clip{};
    clip.count = 0;
    const Raster2dStatus op = engine.rop(r, clip);
    expect(op == Raster2dStatus::success && capture.submitted, "rop fill submitted an IB");

    // Decode the captured IB and verify the compute state.
    const auto& dw = capture.dwords;
    bool found_pgm = false;
    bool found_num_thread = false;
    bool found_dispatch = false;
    const uint64_t pgm = code_info.gpu.virtual_address +
                         kernels.image.kernels[static_cast<uint32_t>(Raster2dKernel::rop)].offset_bytes;

    for (size_t i = 0; i + 1 < dw.size();) {
        const uint32_t header = dw[i];
        const uint32_t op_code = (header >> 8) & 0xff;
        const uint32_t count = ((header >> 16) & 0x3fff) + 1;

        if ((header >> 30) == 3 && op_code == 0x76) {         // SET_SH_REG
            const uint32_t reg = dw[i + 1];

            if (reg == ((0xB830u - 0xB000u) >> 2)) {          // COMPUTE_PGM_LO
                found_pgm = dw[i + 2] == static_cast<uint32_t>(pgm >> 8) &&
                            dw[i + 3] == static_cast<uint32_t>(pgm >> 40);
            }

            if (reg == ((0xB81Cu - 0xB000u) >> 2)) {          // COMPUTE_NUM_THREAD_X
                found_num_thread = dw[i + 2] == raster2d_workgroup_threads && dw[i + 3] == 1 &&
                                   dw[i + 4] == 1;
            }

            i += 1 + count;
        } else if ((header >> 30) == 3 && op_code == 0x15) {  // DISPATCH_DIRECT
            found_dispatch = dw[i + 1] == 1 /* 64/64 groups_x */ && dw[i + 2] == 16 &&
                             dw[i + 3] == 1 && (header & (1u << 1)) != 0;
            i += 1 + count;
        } else {
            i += 1 + count;
        }
    }

    expect(found_pgm, "IB SET_SH_REG COMPUTE_PGM_LO/HI = kernel address >> 8 / >> 40");
    expect(found_num_thread, "IB COMPUTE_NUM_THREAD_X/Y/Z = 64/1/1");
    expect(found_dispatch, "IB DISPATCH_DIRECT grid 1x16x1 with compute shader type");

    engine.synchronize();
    engine.shutdown();
}

// ------------------------------------------- end-to-end PM4 IB execution

// A simulated device: a Manager arena with CPU backing for the engine's code, args
// and IB allocations, plus surface buffers. Every submitted IB is decoded and
// executed immediately (SET_SH_REG COMPUTE_PGM_LO/HI and USER_DATA_0/1, then
// DISPATCH_DIRECT runs every workgroup of the kernel at the programmed address),
// so the tests check what the engine actually encodes, not the builders alone.
struct GpuSim final {
    TestBackend backend;
    memory::Manager manager;
    memory::Handle code{}, args{}, ib{};
    uint64_t code_va = 0, args_va = 0;
    uint8_t* code_cpu = nullptr;
    uint8_t* args_cpu = nullptr;
    uint8_t* ib_cpu = nullptr;
    uint64_t code_bytes = 0, args_bytes = 0;
    uint32_t ib_count = 2;
    uint32_t ib_bytes = 0;
    std::vector<gfx11_interp::Buffer*> surfaces;
    std::vector<uint64_t> offsets;
    uint32_t sequence = 0;
    uint32_t dispatches = 0;
    bool prologue_ok = true;
    bool decode_ok = true;
    Raster2dEngine engine;

    bool setup() {
        auto page_up = [](uint64_t v) { return (v + 4095u) & ~uint64_t{4095u}; };
        ib_bytes = raster2d_min_ib_bytes;

        if (manager.initialize({0x10000000ull, 64u << 20}, {&backend, &TestBackend::create,
                                                            &TestBackend::destroy}) !=
                memory::Status::success ||
            manager.allocate(page_up(raster2d_code_bytes), 4096, memory::Access::read, true, code) !=
                memory::Status::success ||
            manager.allocate(page_up(uint64_t{raster2d_args_bytes_total} * ib_count), 4096,
                             memory::Access::read, true, args) != memory::Status::success ||
            manager.allocate(page_up(uint64_t{ib_bytes} * ib_count), 4096, memory::Access::read, true,
                             ib) != memory::Status::success) {
            return false;
        }

        memory::AllocationInfo info{};
        manager.describe(code, info);
        code_va = info.gpu.virtual_address;
        code_cpu = info.backing.cpu_bytes;
        code_bytes = info.gpu.byte_length;
        manager.describe(args, info);
        args_va = info.gpu.virtual_address;
        args_cpu = info.backing.cpu_bytes;
        args_bytes = info.gpu.byte_length;
        manager.describe(ib, info);
        ib_cpu = info.backing.cpu_bytes;

        Raster2dEngineConfig config{};
        config.manager = &manager;
        config.code = code;
        config.args = args;
        config.ib = ib;
        config.ib_count = ib_count;
        config.ib_bytes = ib_bytes;
        config.submitter = {this, &GpuSim::submit, &GpuSim::query, &GpuSim::wait};
        config.wait_timeout_us = 1000;
        config.order_ibs_on_gpu = true;
        return engine.initialize(config) == Raster2dStatus::success;
    }

    void execute(const std::vector<uint32_t>& dw) {
        gfx11_interp::Buffer args_buffer;
        args_buffer.base = args_va;
        args_buffer.bytes.assign(args_cpu, args_cpu + args_bytes);
        gfx11_interp::Memory mem;
        mem.buffers.push_back(&args_buffer);

        for (gfx11_interp::Buffer* b : surfaces) {
            mem.buffers.push_back(b);
        }

        // IB prologue: EVENT_WRITE CS_PARTIAL_FLUSH, then ACQUIRE_MEM GLK|GLV|GL1 inv.
        prologue_ok = prologue_ok && dw.size() > 10 && dw[0] == ((3u << 30) | (0x46u << 8)) &&
                      dw[1] == 0x407u && dw[2] == ((3u << 30) | (0x58u << 8) | (6u << 16)) &&
                      dw[3] == 0 && dw[4] == 0xffffffffu && dw[5] == 0x00ffffffu &&
                      dw[8] == 0x0au && dw[9] == 0x380u;
        uint32_t pgm_lo = 0, pgm_hi = 0, ud0 = 0, ud1 = 0;

        for (size_t i = 0; i < dw.size();) {
            const uint32_t header = dw[i];

            if (header == 0xffff1000u) {   // PACKET3(NOP, 0x3FFF): one dword
                ++i;
                continue;
            }

            if ((header >> 30) != 3) {
                decode_ok = false;
                return;
            }

            const uint32_t op = (header >> 8) & 0xff;
            const uint32_t body = ((header >> 16) & 0x3fff) + 1;

            if (i + 1 + body > dw.size()) {
                decode_ok = false;
                return;
            }

            if (op == 0x76) {
                const uint32_t reg = dw[i + 1];

                for (uint32_t k = 0; k + 1 < body; ++k) {
                    const uint32_t value = dw[i + 2 + k];
                    switch (reg + k) {
                    case 0x20c: pgm_lo = value; break;
                    case 0x20d: pgm_hi = value; break;
                    case 0x240: ud0 = value; break;
                    case 0x241: ud1 = value; break;
                    default: break;
                    }
                }
            } else if (op == 0x15) {
                ++dispatches;
                const uint64_t address = (uint64_t{pgm_lo} << 8) | (uint64_t{pgm_hi} << 40);

                if (address < code_va || address >= code_va + code_bytes || (header & 2u) == 0 ||
                    dw[i + 3] != 1) {
                    decode_ok = false;
                    return;
                }

                const uint8_t* code_ptr = code_cpu + (address - code_va);
                const uint32_t code_dwords = static_cast<uint32_t>((code_va + code_bytes - address) / 4);

                for (uint32_t gy = 0; gy < dw[i + 2]; ++gy) {
                    for (uint32_t gx = 0; gx < dw[i + 1]; ++gx) {
                        for (uint32_t wave = 0; wave < 2; ++wave) {
                            gfx11_interp::Wave w;
                            w.sgpr[0] = ud0;
                            w.sgpr[1] = ud1;
                            w.sgpr[2] = gx;
                            w.sgpr[3] = gy;

                            for (uint32_t lane = 0; lane < gfx11_interp::lanes; ++lane) {
                                // GFX11 packs Y/Z work-item ids above bit 10: put
                                // noise there; the kernels must mask it off.
                                w.vgpr[0][lane] = (wave * gfx11_interp::lanes + lane) | (0x2a5u << 10);
                            }

                            gfx11_interp::run(code_ptr, code_dwords, w, &mem);
                        }
                    }
                }
            } else if (op != 0x46 && op != 0x58) {
                decode_ok = false;
                return;
            }

            i += 1 + body;
        }
    }

    static Raster2dSubmitResult submit(void* ctx, memory::Handle, uint64_t offset, uint32_t count,
                                       uint32_t& seq) noexcept {
        auto* self = static_cast<GpuSim*>(ctx);
        self->offsets.push_back(offset);
        std::vector<uint32_t> dw(count);

        for (uint32_t i = 0; i < count; ++i) {
            std::memcpy(&dw[i], self->ib_cpu + offset + i * 4u, 4);
        }

        self->execute(dw);
        seq = ++self->sequence;
        return Raster2dSubmitResult::submitted;
    }
    static Raster2dFenceState query(void*, uint32_t) noexcept { return Raster2dFenceState::signalled; }
    static Raster2dFenceState wait(void*, uint32_t, uint64_t) noexcept {
        return Raster2dFenceState::signalled;
    }
};

gfx11_interp::Buffer* add_surface(GpuSim& sim, uint64_t base, size_t bytes, uint8_t fill_value) {
    static std::vector<std::unique_ptr<gfx11_interp::Buffer>> pool;
    auto b = std::make_unique<gfx11_interp::Buffer>();
    b->base = base;
    b->bytes.assign(bytes, fill_value);
    gfx11_interp::Buffer* ptr = b.get();
    pool.push_back(std::move(b));
    sim.surfaces.push_back(ptr);
    return ptr;
}

uint32_t read32(const gfx11_interp::Buffer* b, size_t offset) {
    uint32_t value = 0;
    std::memcpy(&value, b->bytes.data() + offset, 4);
    return value;
}

// Direct port of the pinned eng/gradient.c IntEngGradientFillTriangle (both the
// gradient and the single-colour branch, the latter with the pinned build's
// Rx7900GradientSpanVisible DIB_HLine guard), independent of the planner.
void reference_triangle(std::vector<uint32_t>& pixels, int32_t width, int32_t height,
                        const Raster2dTriVertex (&input)[3], const Raster2dRect& extents,
                        const std::vector<Raster2dRect>& clips) {
    Raster2dTriVertex v1 = input[0], v2 = input[1], v3 = input[2];
    auto smaller = [](const Raster2dTriVertex& a, const Raster2dTriVertex& b) {
        return a.y < b.y || (a.y == b.y && a.x < b.x);
    };
    if (smaller(v2, v1)) { std::swap(v1, v2); }
    if (smaller(v3, v2)) { std::swap(v2, v3); if (smaller(v2, v1)) { std::swap(v1, v2); } }
    const bool shaded = !(v1.red == v2.red && v1.red == v3.red && v1.green == v2.green &&
                          v1.green == v3.green && v1.blue == v2.blue && v1.blue == v3.blue);
    const Raster2dTriVertex* a[3] = {&v1, &v1, &v2};
    const Raster2dTriVertex* b[3] = {&v3, &v2, &v3};
    auto put = [&](int32_t x, int32_t y, uint32_t c) {
        if (x >= 0 && x < width && y >= 0 && y < height) { pixels[static_cast<size_t>(y) * width + x] = c; }
    };

    for (const Raster2dRect& clip : clips) {
        Raster2dRect fr{std::max(clip.left, extents.left), std::max(clip.top, extents.top),
                        std::min(clip.right, extents.right), std::min(clip.bottom, extents.bottom)};
        if (fr.right <= fr.left || fr.bottom <= fr.top) { continue; }
        int32_t x[3], sx[3], dx[3], dy[3], incx[3], ex[3], destx[3];
        int32_t c[3][3], dc[3][3], ec[3][3], ic[3][3];

        for (int l = 0; l < 3; ++l) {
            x[l] = a[l]->x;
            sx[l] = a[l]->x - (shaded ? 1 : 0);
            dx[l] = std::abs(b[l]->x - a[l]->x);
            dy[l] = std::abs(b[l]->y - a[l]->y);
            incx[l] = b[l]->x > a[l]->x ? 1 : -1;
            ex[l] = -(dy[l] >> 1);
            destx[l] = b[l]->x;
            const int32_t ac[3] = {a[l]->red >> 8, a[l]->green >> 8, a[l]->blue >> 8};
            const int32_t bc[3] = {b[l]->red >> 8, b[l]->green >> 8, b[l]->blue >> 8};
            for (int k = 0; k < 3; ++k) {
                c[l][k] = ac[k]; dc[l][k] = std::abs(bc[k] - ac[k]); ec[l][k] = -(dy[l] >> 1);
                ic[l][k] = bc[k] > ac[k] ? 1 : -1;
            }
        }

        auto doline = [&](int l) {
            if (shaded) {
                for (int k = 0; k < 3; ++k) {
                    ec[l][k] += dc[l][k];
                    if (dy[l] != 0) { while (ec[l][k] > 0) { c[l][k] += ic[l][k]; ec[l][k] -= dy[l]; } }
                }
            }
            ex[l] += dx[l];
            while (ex[l] > 0 && x[l] != destx[l]) { x[l] += incx[l]; sx[l] += incx[l]; ex[l] -= dy[l]; }
        };
        auto rgb = [](int32_t r, int32_t g, int32_t bl) {
            return (uint32_t(r & 0xff) << 16) | (uint32_t(g & 0xff) << 8) | uint32_t(bl & 0xff);
        };
        int32_t y = v1.y;
        int32_t sy = v1.y;
        const int32_t bt = std::min(v3.y, fr.bottom);

        auto fillline = [&](int f, int t) {
            const bool in_y = !(sy < fr.top || sy >= fr.bottom);
            const int32_t gx = std::abs(sx[t] - sx[f]);
            const int32_t gxi = sx[f] < sx[t] ? 1 : -1;
            int32_t gc[3], gd[3], ge[3], gi[3];
            for (int k = 0; k < 3; ++k) {
                gc[k] = c[f][k]; gd[k] = std::abs(c[t][k] - gc[k]); ge[k] = -(gx >> 1);
                gi[k] = c[t][k] > gc[k] ? 1 : -1;
            }
            const int32_t g_end = sx[t] + gxi;
            for (int32_t g = sx[f]; g != g_end; g += gxi) {
                if (in_y && g >= fr.left && g < fr.right) { put(g, sy, rgb(gc[0], gc[1], gc[2])); }
                for (int k = 0; k < 3; ++k) {
                    ge[k] += gd[k];
                    if (gx != 0) { while (ge[k] > 0) { gc[k] += gi[k]; ge[k] -= gx; } }
                }
            }
        };
        const uint32_t solid = rgb(v1.red >> 8, v1.green >> 8, v1.blue >> 8);
        auto s_fillline = [&](int f, int t) {
            const int32_t l = std::max(std::min(sx[f], sx[t]), fr.left);
            const int32_t r = std::min(std::max(sx[f], sx[t]), fr.right);
            if (l < r && sy >= fr.top && sy < fr.bottom) {
                for (int32_t g = l; g < r; ++g) { put(g, sy, solid); }
            }
        };

        while (sy < bt) {
            if (y >= v1.y && y <= v3.y) { doline(0); }
            if (y >= v1.y && y <= v2.y) { doline(1); shaded ? fillline(0, 1) : s_fillline(0, 1); }
            if (y >= v2.y && y <= v3.y) {
                if (shaded) { fillline(0, 2); doline(2); fillline(0, 2); }
                else { doline(2); s_fillline(0, 2); }
            }
            ++y;
            ++sy;
        }
    }
}

void run_triangle_case(const char* name, const Raster2dTriVertex (&verts)[3], const Raster2dRect& extents,
                       const std::vector<Raster2dRect>& clips, int32_t width, int32_t height) {
    GpuSim sim;
    expect(sim.setup(), "engine set up on the simulated device");
    const uint64_t base = 0x40000000ull;
    const uint32_t pitch = static_cast<uint32_t>(width) * 4u + 64u; // padded pitch
    gfx11_interp::Buffer* dst = add_surface(sim, base, size_t(pitch) * height, 0x5a);

    Raster2dTriangleRequest request{};
    request.destination = {base, pitch, static_cast<uint32_t>(width), static_cast<uint32_t>(height)};
    request.vertices[0] = verts[0];
    request.vertices[1] = verts[1];
    request.vertices[2] = verts[2];
    request.extents = extents;
    Raster2dClipList clip{};
    clip.count = static_cast<uint32_t>(clips.size());
    for (size_t i = 0; i < clips.size(); ++i) { clip.rects[i] = clips[i]; }

    const Raster2dStatus status = sim.engine.gradient_triangle(request, clip);
    std::vector<uint32_t> want(static_cast<size_t>(width) * height, 0x5a5a5a5au);
    std::vector<Raster2dRect> ref_clips = clips;
    if (ref_clips.empty()) { ref_clips.push_back({0, 0, width, height}); }
    reference_triangle(want, width, height, verts, extents, ref_clips);
    uint32_t mismatches = 0;
    uint32_t written = 0;

    for (int32_t y = 0; y < height; ++y) {
        for (int32_t x = 0; x < width; ++x) {
            const uint32_t got = read32(dst, size_t(y) * pitch + size_t(x) * 4);
            const uint32_t expected = want[size_t(y) * width + x];
            written += expected != 0x5a5a5a5au ? 1u : 0u;
            if (got != expected) { ++mismatches; }
        }
    }

    // Padding bytes between width * 4 and pitch stay untouched.
    bool padding_ok = true;
    for (int32_t y = 0; y < height; ++y) {
        for (uint32_t byte = uint32_t(width) * 4u; byte < pitch; ++byte) {
            padding_ok = padding_ok && dst->bytes[size_t(y) * pitch + byte] == 0x5a;
        }
    }

    expect(status == Raster2dStatus::success && sim.decode_ok && sim.prologue_ok && mismatches == 0 &&
               written > 0 && padding_ok, name);
    if (mismatches != 0 || status != Raster2dStatus::success || !padding_ok || !sim.decode_ok) {
        std::fprintf(stderr, "  %s: status %d, %u mismatches of %u written, decode %d prologue %d pad %d\n", name,
                     static_cast<int>(status), mismatches, written, sim.decode_ok, sim.prologue_ok, padding_ok);
    }
}

void test_engine_triangles() {
    const Raster2dTriVertex shaded[3] = {{5, 4, 0xff00, 0x1000, 0x0000}, {70, 22, 0x0000, 0xff00, 0x3300},
                                         {20, 61, 0x2200, 0x0000, 0xff00}};
    const Raster2dTriVertex solid[3] = {{60, 3, 0x8000, 0x4000, 0x2000}, {3, 30, 0x8000, 0x4000, 0x2000},
                                        {44, 58, 0x8000, 0x4000, 0x2000}};
    const std::vector<Raster2dRect> clips = {{0, 0, 40, 30}, {40, 0, 80, 30}, {0, 30, 25, 64},
                                             {30, 34, 80, 64}};
    const Raster2dRect all{-100, -100, 1000, 1000};
    run_triangle_case("engine shaded triangle, unclipped", shaded, all, {}, 80, 64);
    run_triangle_case("engine shaded triangle, 4 clip rects + extents", shaded, {2, 6, 75, 60}, clips, 80, 64);
    run_triangle_case("engine single-colour triangle (S_FILLLINE rules), unclipped", solid, all, {}, 80, 64);
    run_triangle_case("engine single-colour triangle, clipped", solid, {1, 1, 70, 62}, clips, 80, 64);
    // Taller than one row table: spans several IBs/chunks.
    const Raster2dTriVertex tall[3] = {{2, 1, 0xff00, 0x0000, 0x8000}, {29, 300, 0x0000, 0xff00, 0x0000},
                                       {11, 700, 0x0000, 0x2000, 0xff00}};
    run_triangle_case("engine 700-row triangle across row-table chunks", tall, all,
                      {{0, 0, 32, 350}, {0, 350, 32, 720}}, 32, 720);
    // Planner start-row overload matches the full plan's tail.
    Raster2dGradientSpan full[64][raster2d_triangle_spans];
    Raster2dGradientSpan tail[64][raster2d_triangle_spans];
    Raster2dRect e1{}, e2{};
    uint32_t n1 = 0, n2 = 0;
    raster2d_plan_triangle(shaded[0], shaded[1], shaded[2], full, 64, e1, n1);
    raster2d_plan_triangle(shaded[0], shaded[1], shaded[2], 30, tail, 64, e2, n2);
    bool same = e2.top == 30 && n2 + 26 == n1;
    for (uint32_t r = 0; same && r < n2; ++r) {
        for (uint32_t s = 0; s < raster2d_triangle_spans; ++s) {
            same = same && std::memcmp(&full[r + 26][s], &tail[r][s], sizeof(Raster2dGradientSpan)) == 0;
        }
    }
    expect(same, "raster2d_plan_triangle(first_row) equals the full plan from that row");
}

void test_engine_byte_formats() {
    // Monochrome source with a tight pitch (2 bytes for 13 pixels) at an odd
    // address, expanded through the engine; a 1-bpp ROP4 mask with a 3-byte
    // pitch; a coverage8 source with pitch = width = 9 at an odd address.
    GpuSim sim;
    expect(sim.setup(), "engine set up for byte formats");
    const uint64_t dst_base = 0x40000000ull;
    const uint32_t w = 13, h = 5, pitch = w * 4;
    gfx11_interp::Buffer* dst = add_surface(sim, dst_base, size_t(pitch) * h, 0x11);
    const uint64_t mono_base = 0x50000001ull;
    gfx11_interp::Buffer* mono = add_surface(sim, mono_base, 2 * h, 0);
    std::mt19937 rng(17);
    for (auto& byte : mono->bytes) { byte = static_cast<uint8_t>(rng()); }

    Raster2dRopRequest r{};
    r.destination = {dst_base, pitch, w, h};
    r.destination_rect = {0, 0, int32_t(w), int32_t(h)};
    r.has_source = true;
    r.source_monochrome = true;
    r.source = {mono_base, 2, w, h, Raster2dFormat::mono1};
    r.source_width = int32_t(w);
    r.source_height = int32_t(h);
    r.mono_foreground = 0x00fedcbau;
    r.mono_background = 0x00123456u;
    r.rop4 = 0xcccc;
    Raster2dClipList none{};
    const Raster2dStatus mono_status = sim.engine.rop(r, none);
    uint32_t bad = 0;
    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            const uint32_t bit = (mono->bytes[y * 2 + (x >> 3)] >> (7 - (x & 7))) & 1u;
            bad += read32(dst, size_t(y) * pitch + x * 4) != (bit ? 0x00fedcbau : 0x00123456u) ? 1u : 0u;
        }
    }
    expect(mono_status == Raster2dStatus::success && bad == 0 && sim.decode_ok,
           "engine mono1 source with pitch 2 for 13 pixels at an odd address");

    // The legacy form (format left at the bgrx8888 default) is accepted for mono.
    Raster2dRopRequest legacy = r;
    legacy.source.format = Raster2dFormat::bgrx8888;
    expect(sim.engine.rop(legacy, none) == Raster2dStatus::success,
           "mono role with the default format is accepted (backward compatible)");
    Raster2dRopRequest wrong = r;
    wrong.source.format = Raster2dFormat::coverage8;
    expect(sim.engine.rop(wrong, none) == Raster2dStatus::invalid_surface,
           "an explicit format that contradicts the role is refused");
    Raster2dRopRequest narrow = r;
    narrow.source.pitch_bytes = 1;  // 13 pixels need 2 bytes
    expect(sim.engine.rop(narrow, none) == Raster2dStatus::invalid_surface,
           "a mono1 pitch below ceil(width / 8) is refused");

    // ROP4 mask: mask bit 1 -> PATCOPY brush, 0 -> destination (BltMask).
    const uint64_t mask_base = 0x51000003ull;
    gfx11_interp::Buffer* mask = add_surface(sim, mask_base, 3 * 8, 0);
    for (auto& byte : mask->bytes) { byte = static_cast<uint8_t>(rng()); }
    std::vector<uint32_t> before(w * h);
    for (uint32_t i = 0; i < w * h; ++i) { before[i] = read32(dst, size_t(i / w) * pitch + (i % w) * 4); }
    Raster2dRopRequest m{};
    m.destination = {dst_base, pitch, w, h};
    m.destination_rect = {2, 1, 12, 5};
    m.has_mask = true;
    m.mask = {mask_base, 3, 20, 8, Raster2dFormat::mono1};
    m.mask_origin = {5, 2};
    m.brush_colour = 0x00abcdefu;
    m.rop4 = 0xaaf0;  // fg PATCOPY, bg D
    const Raster2dStatus mask_status = sim.engine.rop(m, none);
    bad = 0;
    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            uint32_t want = before[y * w + x];
            if (x >= 2 && x < 12 && y >= 1 && y < 5) {
                const uint32_t mx = 5 + (x - 2), my = 2 + (y - 1);
                if ((mask->bytes[my * 3 + (mx >> 3)] >> (7 - (mx & 7))) & 1u) { want = 0x00abcdefu; }
            }
            bad += read32(dst, size_t(y) * pitch + x * 4) != want ? 1u : 0u;
        }
    }
    expect(mask_status == Raster2dStatus::success && bad == 0, "engine ROP4 with a 1-bpp mask (pitch 3)");
    Raster2dRopRequest small = m;
    small.mask_origin = {12, 2};  // 12 + 10 columns > 20
    expect(sim.engine.rop(small, none) == Raster2dStatus::out_of_bounds,
           "a mask smaller than the masked area is refused");

    // coverage8 with pitch = width = 9 at an odd address.
    const uint64_t cov_base = 0x52000005ull;
    const uint32_t cw = 9, ch = 3;
    gfx11_interp::Buffer* cov = add_surface(sim, cov_base, cw * ch, 0);
    for (auto& byte : cov->bytes) { byte = static_cast<uint8_t>(rng()); }
    cov->bytes[0] = 0; cov->bytes[1] = 0xff;
    for (uint32_t i = 0; i < w * h; ++i) { before[i] = read32(dst, size_t(i / w) * pitch + (i % w) * 4); }
    Raster2dAlphaRequest a{};
    a.destination = {dst_base, pitch, w, h};
    a.destination_rect = {1, 1, 1 + int32_t(cw), 1 + int32_t(ch)};
    a.source = {cov_base, cw, cw, ch, Raster2dFormat::coverage8};
    a.source_left = 0;
    a.source_top = 0;
    a.source_width = int32_t(cw);
    a.source_height = int32_t(ch);
    a.coverage8 = true;
    a.solid_colour = 0x00406080u;
    const Raster2dStatus cov_status = sim.engine.alpha(a, none);
    bad = 0;
    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            uint32_t want = before[y * w + x];
            if (x >= 1 && x < 1 + cw && y >= 1 && y < 1 + ch) {
                const int m8 = cov->bytes[(y - 1) * cw + (x - 1)];
                if (m8 == 0xff) {
                    want = 0x00406080u;
                } else if (m8 > 0) {
                    uint32_t out = 0;
                    for (int k = 0; k < 3; ++k) {
                        const int fg = (0x00406080 >> (8 * k)) & 0xff, bg = (want >> (8 * k)) & 0xff;
                        out |= uint32_t(((m8 * (fg - bg)) >> 8) + bg) << (8 * k);
                    }
                    want = out;
                }
            }
            bad += read32(dst, size_t(y) * pitch + x * 4) != want ? 1u : 0u;
        }
    }
    expect(cov_status == Raster2dStatus::success && bad == 0, "engine coverage8 with pitch 9 at an odd address");

    // Overlap refusal: AlphaBlend from the destination area into itself.
    Raster2dAlphaRequest self{};
    self.destination = {dst_base, pitch, w, h};
    self.destination_rect = {0, 0, 6, 3};
    self.source = {dst_base, pitch, w, h};
    self.source_left = 3;
    self.source_top = 1;
    self.source_width = 6;
    self.source_height = 3;
    expect(sim.engine.alpha(self, none) == Raster2dStatus::overlapping_surfaces,
           "overlapping same-surface AlphaBlend is refused");
    Raster2dRopRequest disjoint{};
    disjoint.destination = {dst_base, pitch, w, h};
    disjoint.destination_rect = {0, 0, 4, 2};
    disjoint.has_source = true;
    disjoint.source = {dst_base, pitch, w, h};
    disjoint.source_left = 6;
    disjoint.source_top = 2;
    disjoint.source_width = 4;
    disjoint.source_height = 2;
    disjoint.rop4 = 0x6666;
    expect(sim.engine.rop(disjoint, none) == Raster2dStatus::success,
           "a same-surface ROP from a disjoint area is accepted");

    // A guarded stretch can cover the source in its un-clipped bounds while
    // painting only a separate window clip. Its live GPU reads are safe only
    // when every admitted clip rectangle stays away from the source domain.
    Raster2dRopRequest clipped = disjoint;
    clipped.destination_rect = {0, 0, 12, 4};
    clipped.rop4 = 0xcccc;
    Raster2dClipList safe_clip{};
    safe_clip.count = 1;
    safe_clip.rects[0] = {0, 0, 4, 2};
    const std::vector<uint8_t> snapshot = dst->bytes;
    const Raster2dStatus clipped_status = sim.engine.rop(clipped, safe_clip);
    bool clipped_exact = clipped_status == Raster2dStatus::success;
    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            const uint32_t source_x = 6 + (x * 4 / 12);
            const size_t offset = static_cast<size_t>(y) * pitch + x * 4;
            const size_t reference = x < 4 && y < 2 ? static_cast<size_t>(2) * pitch + source_x * 4 : offset;
            clipped_exact = clipped_exact &&
                std::memcmp(dst->bytes.data() + offset, snapshot.data() + reference, 4) == 0;
        }
    }
    expect(clipped_exact, "same-surface clipped stretch matches a source snapshot pixel-exact");
    Raster2dClipList unsafe_clip{};
    unsafe_clip.count = 1;
    unsafe_clip.rects[0] = {6, 2, 8, 4};
    expect(sim.engine.rop(clipped, unsafe_clip) == Raster2dStatus::overlapping_surfaces,
           "same-surface stretch refuses a clipped write meeting its source");

    // IB offsets are byte offsets of the slots (slot i at i * ib_bytes).
    bool offsets_ok = !sim.offsets.empty();
    for (size_t i = 0; i < sim.offsets.size(); ++i) {
        offsets_ok = offsets_ok && sim.offsets[i] == uint64_t(i % sim.ib_count) * sim.ib_bytes;
    }
    expect(offsets_ok, "submitted ib_offset is the slot byte offset");
    expect(sim.prologue_ok && sim.decode_ok, "every IB starts with CS_PARTIAL_FLUSH + ACQUIRE_MEM");
}


void test_engine_legacy_rows() {
    // Precomputed-rows overload: 150 rows x 2 clip rectangles = 300 row dispatches,
    // more than one IB holds; the clip list is honoured.
    GpuSim sim;
    expect(sim.setup(), "engine set up for the legacy rows path");
    const int32_t width = 48, height = 160;
    const uint64_t base = 0x40000000ull;
    const uint32_t pitch = uint32_t(width) * 4u;
    gfx11_interp::Buffer* dst = add_surface(sim, base, size_t(pitch) * height, 0x5a);
    const Raster2dTriVertex v[3] = {{3, 2, 0xff00, 0x0000, 0x4000}, {45, 60, 0x0000, 0xff00, 0x0000},
                                    {12, 152, 0x1000, 0x2000, 0xff00}};
    static Raster2dGradientSpan rows[160][raster2d_triangle_spans];
    Raster2dRect ext{};
    uint32_t n = 0;
    raster2d_plan_triangle(v[0], v[1], v[2], rows, 160, ext, n);
    Raster2dGradientTriangleRequest request{};
    request.destination = {base, pitch, uint32_t(width), uint32_t(height)};
    request.destination_rect = {0, ext.top, width, ext.top + int32_t(n)};
    request.rows = rows;
    request.row_count = n;
    Raster2dClipList clip{};
    clip.count = 2;
    clip.rects[0] = {0, 0, 20, 160};
    clip.rects[1] = {25, 30, 48, 140};
    const Raster2dStatus status = sim.engine.gradient_triangle(request, clip);
    std::vector<uint32_t> want(size_t(width) * height, 0x5a5a5a5au);
    reference_triangle(want, width, height, v, {-1000, -1000, 1000, 1000},
                       {clip.rects[0], clip.rects[1]});
    uint32_t bad = 0;
    for (int32_t y = 0; y < height; ++y) {
        for (int32_t x = 0; x < width; ++x) {
            bad += read32(dst, size_t(y) * pitch + size_t(x) * 4) != want[size_t(y) * width + x] ? 1u : 0u;
        }
    }
    expect(status == Raster2dStatus::success && bad == 0 && sim.decode_ok && sim.offsets.size() > 1,
           "legacy rows overload honours clips across several IBs");
}


// ------------------------------------------- mirrored stretch (EngStretchBltROP)

namespace ref = gdi_reference;

struct StretchCase final {
    ref::Rect destination;   // application DestRect (signed extents)
    ref::Rect source;        // application SourceRect
    ref::Clip clip;
    uint32_t rop4;
    bool tile;
    bool mask;
    bool mono;
    bool device;             // device-managed source (IntEngEnter window)
    ref::Point brush_origin;
};

// Writes a 32bpp reference bitmap into an interpreter surface (pitch 4 * cx).
void load_surface(Surface& s, const ref::Bitmap32& b) {
    for (int32_t y = 0; y < b.cy; ++y) {
        std::memcpy(s.bytes.data() + size_t(y) * s.pitch, &b.px[size_t(y) * size_t(b.cx)], size_t(b.cx) * 4);
    }
}

// Runs every DIB call the engine port made through the rop kernel, one dispatch
// per call (clip = the call's destination inside the target), and compares with
// the port's final destination. Returns the number of mismatching pixels.
uint32_t run_stretch_case(const Kernels& kernels, const StretchCase& c, std::mt19937& rng, bool& called,
                          uint32_t& flips) {
    const int32_t tw = 37, th = 29, sw = 23, sh = 19;
    ref::Bitmap32 target(tw, th), source(sw, sh);
    ref::Bitmap1 mono(sw, sh), mask(sw + 3, sh - 2);
    for (auto& v : target.px) v = rng();
    for (auto& v : source.px) v = rng();
    for (int32_t y = 0; y < mono.cy; ++y) for (int32_t x = 0; x < mono.cx; ++x) mono.set(x, y, (rng() & 1) != 0);
    for (int32_t y = 0; y < mask.cy; ++y) for (int32_t x = 0; x < mask.cx; ++x) mask.set(x, y, (rng() % 3) == 0);
    std::vector<uint32_t> tile(64);
    for (auto& v : tile) v = rng();

    const ref::DriverStretch drv = ref::int_eng_stretch_blt(c.destination, c.source, c.clip);
    called = !drv.bitblt && !drv.nothing;
    flips = (drv.flip_x ? 1u : 0u) | (drv.flip_y ? 2u : 0u);
    if (!called) {
        return 0;
    }

    ref::Rect domain = drv.input;
    ref::make_well_ordered(domain);
    if (c.device && ref::rop4_uses_source(c.rop4) &&
        (domain.left < 0 || domain.top < 0 || domain.right > sw || domain.bottom > sh ||
         domain.left >= domain.right || domain.top >= domain.bottom)) {
        // This kernel-level sweep keeps device windows inside the surface; the
        // zero-filled IntEngEnter remainder is covered through the radeondisp plan
        // (radeon_2d_backend_tests planned_stretch_end_to_end).
        called = false;
        return 0;
    }

    ref::Source src{};
    if (c.mono) {
        src.mono = &mono;
        src.mono0 = 0x00112233u;
        src.mono1 = 0x00ccbbaau;
    } else {
        src.colour = &source;
        src.device = c.device;
    }

    ref::Brush brush{};
    brush.solid = c.tile ? 0xffffffffu : 0x00a5c3e1u;
    brush.tile = c.tile ? tile.data() : nullptr;
    ref::Bitmap32 expected = target;
    std::vector<ref::DibCall> calls;
    ref::eng_stretch_blt_rop(expected, src, c.mask ? &mask : nullptr, &c.clip, &c.brush_origin,
                             ref::fallback_destination(drv), drv.input, &brush, c.rop4, &calls);

    Surface dst = make_surface(0x100000, tw, th);
    Surface srcs = make_surface(0x200000, sw, sh);
    Surface mono_s = make_surface(0x300000, sw, sh, mono.stride);
    Surface mask_s = make_surface(0x400000, mask.cx, mask.cy, mask.stride);
    load_surface(dst, target);
    load_surface(srcs, source);
    mono_s.bytes = mono.bits;
    mask_s.bytes = mask.bits;
    gfx11_interp::Buffer* dstb = as_interp(dst);
    std::vector<gfx11_interp::Buffer*> buffers = {dstb, as_interp(srcs), as_interp(mono_s), as_interp(mask_s)};

    for (const ref::DibCall& call : calls) {
        ref::Rect d = call.dest;
        const bool mirror_x = d.right < d.left;
        const bool mirror_y = d.bottom < d.top;
        ref::make_well_ordered(d);
        Raster2dRect visible{std::max(d.left, 0), std::max(d.top, 0), std::min(d.right, tw), std::min(d.bottom, th)};
        if (visible.right <= visible.left || visible.bottom <= visible.top) {
            continue;
        }

        Raster2dRopRequest r;
        r.destination = view(dst);
        r.destination_rect = {d.left, d.top, d.right, d.bottom};
        int32_t origin_x = 0, origin_y = 0;
        if (gdi_reference::rop4_uses_source(c.rop4)) {
            r.has_source = true;
            if (c.mono) {
                r.source = {mono_s.gpu, mono_s.pitch, uint32_t(sw), uint32_t(sh), Raster2dFormat::mono1};
                r.source_monochrome = true;
                r.mono_background = src.mono0;
                r.mono_foreground = src.mono1;
            } else if (c.device) {
                // The IntEngEnter window: a cropped view of the source surface.
                origin_x = domain.left;
                origin_y = domain.top;
                r.source = {srcs.gpu + uint64_t(domain.top) * srcs.pitch + uint64_t(domain.left) * 4, srcs.pitch,
                            uint32_t(domain.right - domain.left), uint32_t(domain.bottom - domain.top)};
            } else {
                r.source = view(srcs);
            }
            const int32_t width = call.source.right - call.source.left;
            const int32_t height = call.source.bottom - call.source.top;
            r.source_left = (mirror_x ? call.source.right : call.source.left) - origin_x;
            r.source_top = (mirror_y ? call.source.bottom : call.source.top) - origin_y;
            r.source_width = width;
            r.source_height = height;
            r.source_sign_x = mirror_x ? -1 : 1;
            r.source_sign_y = mirror_y ? -1 : 1;
            r.skip_outside_source = (c.rop4 & 0xff) == 0xcc;
        }
        r.brush_colour = brush.solid;
        r.has_pattern = c.tile && gdi_reference::rop4_uses_pattern(c.rop4);
        r.pattern = tile.data();
        r.brush_origin = {c.brush_origin.x, c.brush_origin.y};
        if (c.mask) {
            r.has_mask = true;
            r.stretch_mask = true;
            r.mask = {mask_s.gpu, mask_s.pitch, uint32_t(mask.cx), uint32_t(mask.cy), Raster2dFormat::mono1};
            r.mask_origin = {0, 0};   // device windows index the mask in window coordinates too
        }
        r.rop4 = c.rop4;

        Raster2dDescriptors desc{};
        build_rop_descriptors(r, desc);
        uint32_t args[64];
        raster2d_build_rop_args(r, visible, desc, args);
        raster2d_buffer_descriptor(0x90000000ull + raster2d_args_bytes, raster2d_pattern_bytes,
                                   *reinterpret_cast<uint32_t(*)[4]>(args + rop_args::pattern));
        run_kernel(kernels, Raster2dKernel::rop, args, reinterpret_cast<const uint8_t*>(tile.data()), 64,
                   visible, buffers);
    }

    uint32_t mismatches = 0;
    for (int32_t y = 0; y < th; ++y) {
        for (int32_t x = 0; x < tw; ++x) {
            uint32_t got = 0;
            std::memcpy(&got, dstb->bytes.data() + size_t(y) * dst.pitch + size_t(x) * 4, 4);
            mismatches += got != expected.get(x, y) ? 1u : 0u;
        }
    }

    return mismatches;
}

void test_mirrored_stretch(const Kernels& kernels) {
    std::mt19937 rng(0x5a17);
    const ref::Clip trivial{};
    ref::Clip rect{};
    rect.kind = ref::Complexity::rect;
    rect.bounds = {3, 2, 31, 25};
    ref::Clip complex{};
    complex.kind = ref::Complexity::complex;
    complex.bounds = {0, 1, 37, 27};
    complex.rects = {{0, 1, 12, 9}, {15, 1, 37, 9}, {4, 9, 30, 20}, {0, 20, 9, 27}, {11, 20, 26, 27}};
    const ref::Clip* clips[] = {&trivial, &rect, &complex};
    // Source/destination signs give every (FLIP_X, FLIP_Y) pair, including the
    // Case0000/0101/1010 rectangles the engine shifts by one.
    const int32_t signs[4][2] = {{1, 1}, {-1, 1}, {1, -1}, {-1, -1}};
    const uint32_t rops[] = {0xcccc, 0x6666, 0xcc66, 0xf0f0, 0xc0c0, 0x5a5a, 0x8888};
    uint32_t bad[4] = {};
    uint32_t cases[4] = {};
    uint32_t total_bad = 0;
    uint32_t called = 0;
    uint32_t device_cases = 0, mask_cases = 0, tile_cases = 0, mono_cases = 0;

    for (uint32_t trial = 0; trial < 480; ++trial) {
        const int32_t (&ds)[2] = signs[trial % 4];
        const int32_t (&ss)[2] = signs[(trial / 4) % 4];
        StretchCase c{};
        const int32_t dw = 3 + int32_t(rng() % 33), dh = 3 + int32_t(rng() % 27);
        const int32_t sw = 1 + int32_t(rng() % 21), sh = 1 + int32_t(rng() % 17);
        const int32_t dx = int32_t(rng() % 40) - 4, dy = int32_t(rng() % 32) - 4;
        const int32_t sx = int32_t(rng() % 26) - 2, sy = int32_t(rng() % 22) - 2;
        c.destination = ds[0] > 0 ? ref::Rect{dx, 0, dx + dw, 0} : ref::Rect{dx + dw, 0, dx, 0};
        c.destination.top = ds[1] > 0 ? dy : dy + dh;
        c.destination.bottom = ds[1] > 0 ? dy + dh : dy;
        c.source = ss[0] > 0 ? ref::Rect{sx, 0, sx + sw, 0} : ref::Rect{sx + sw, 0, sx, 0};
        c.source.top = ss[1] > 0 ? sy : sy + sh;
        c.source.bottom = ss[1] > 0 ? sy + sh : sy;
        c.clip = *clips[trial % 3];
        c.rop4 = rops[rng() % (sizeof(rops) / sizeof(rops[0]))];
        c.tile = (trial % 5) == 1;
        c.mask = !c.tile && (trial % 7) == 2 && ref::rop4_uses_source(c.rop4);
        c.mono = !c.mask && (trial % 11) == 3;
        c.device = !c.mono && (trial % 3) == 0;
        if (c.device) {
            // A device source window must lie inside the surface.
            c.source = ss[0] > 0 ? ref::Rect{std::max(sx, 1), 0, std::min(sx + sw, 22), 0}
                                 : ref::Rect{std::min(sx + sw, 22), 0, std::max(sx, 1), 0};
            c.source.top = ss[1] > 0 ? std::max(sy, 1) : std::min(sy + sh, 18);
            c.source.bottom = ss[1] > 0 ? std::min(sy + sh, 18) : std::max(sy, 1);
        }
        c.brush_origin = {int32_t(rng() % 19) - 9, int32_t(rng() % 19) - 9};
        bool was_called = false;
        uint32_t flips = 0;
        const uint32_t m = run_stretch_case(kernels, c, rng, was_called, flips);
        if (!was_called) {
            continue;
        }

        if (m != 0 && std::getenv("RASTER2D_DEBUG") != nullptr) {
            std::printf("case %u: dst %d,%d,%d,%d src %d,%d,%d,%d clip %d rop %04x tile %d mask %d mono %d dev %d -> %u\n",
                        trial, c.destination.left, c.destination.top, c.destination.right, c.destination.bottom,
                        c.source.left, c.source.top, c.source.right, c.source.bottom, int(c.clip.kind), c.rop4,
                        c.tile, c.mask, c.mono, c.device, m);
        }

        ++called;
        ++cases[flips];
        bad[flips] += m != 0 ? 1u : 0u;
        total_bad += m;
        device_cases += c.device ? 1u : 0u;
        mask_cases += c.mask ? 1u : 0u;
        tile_cases += c.tile ? 1u : 0u;
        mono_cases += c.mono ? 1u : 0u;
    }

    std::printf("mirrored stretch: %u engine calls (flips 0/x/y/xy %u/%u/%u/%u; device %u, mask %u, tile %u, mono %u), "
                "%u mismatching pixels\n", called, cases[0], cases[1], cases[2], cases[3], device_cases, mask_cases,
                tile_cases, mono_cases, total_bad);
    expect(cases[0] > 20 && cases[1] > 20 && cases[2] > 20 && cases[3] > 20 && device_cases > 10 && mask_cases > 5 &&
               tile_cases > 10 && mono_cases > 5, "mirrored stretch covers every flip pair and operand kind");
    expect(bad[0] == 0, "unmirrored stretch: rop kernel equals EngStretchBltROP/DIB_XXBPP_StretchBlt");
    expect(bad[1] == 0, "FLIP_X stretch: sign -1 x mapping equals the engine fallback");
    expect(bad[2] == 0, "FLIP_Y stretch: sign -1 y mapping equals the engine fallback");
    expect(bad[3] == 0, "FLIP_X|FLIP_Y stretch equals the engine fallback");
}

// One DIB_XXBPP_StretchBlt call through the rop kernel with the dispatch rect
// [rect_left, width) of rows [0, height): returns mismatching pixels there.
uint32_t run_dib_call(const Kernels& kernels, Raster2dRopRequest r, const ref::SourceView& view, const ref::Bitmap1* mask,
                      const std::vector<uint32_t>& tile, ref::Rect dest_rect, ref::Rect source_rect, ref::Point origin,
                      int32_t rect_left, Surface& dst, const ref::Bitmap32& target,
                      std::vector<gfx11_interp::Buffer*> buffers) {
    ref::Bitmap32 expected = target;
    ref::Brush brush{};
    ref::dib_stretch(expected, &view, mask, tile.data(), dest_rect, source_rect, origin, &brush, r.rop4);
    load_surface(dst, target);
    gfx11_interp::Buffer* dstb = as_interp(dst);
    buffers.insert(buffers.begin(), dstb);
    ref::Rect d = dest_rect;
    ref::make_well_ordered(d);
    const Raster2dRect visible{rect_left, d.top, d.right, d.bottom};
    Raster2dDescriptors desc{};
    raster2d_buffer_descriptor(r.destination.gpu_address, r.destination.pitch_bytes * r.destination.height, desc.destination);
    raster2d_rop_source_descriptor(r, desc.source);
    if (r.has_mask) {
        raster2d_buffer_descriptor(r.mask.gpu_address, r.mask.pitch_bytes * r.mask.height, desc.mask);
    }
    uint32_t args[64];
    raster2d_build_rop_args(r, visible, desc, args);
    raster2d_buffer_descriptor(0x90000000ull + raster2d_args_bytes, raster2d_pattern_bytes,
                               *reinterpret_cast<uint32_t(*)[4]>(args + rop_args::pattern));
    run_kernel(kernels, Raster2dKernel::rop, args, reinterpret_cast<const uint8_t*>(tile.data()), 64, visible, buffers);
    uint32_t mismatches = 0;
    for (int32_t y = d.top; y < d.bottom; ++y) {
        for (int32_t x = rect_left; x < d.right; ++x) {
            uint32_t got = 0;
            std::memcpy(&got, dstb->bytes.data() + size_t(y) * dst.pitch + size_t(x) * 4, 4);
            mismatches += got != expected.get(x, y) ? 1u : 0u;
        }
    }
    return mismatches;
}

// DIB_XXBPP_StretchBlt PatternX with a mask over rows wider than two waves and
// one workgroup, dispatched from x = 40 while the DIB call starts at 0: the
// running phase counts drawn pixels from the call's left edge (loop over the
// columns before the wave, v_mbcnt_lo inside it). Also index8/rgb16/rgb24
// translated sources with negative extents (C truncation via the step sign).
void test_running_phase_and_translation(const Kernels& kernels) {
    std::mt19937 rng(0x9a77);
    const int32_t tw = 150, th = 6, sw = 50, sh = 6;
    ref::Bitmap32 target(tw, th), source(sw, sh);
    for (auto& v : target.px) v = rng();
    for (auto& v : source.px) v = rng();
    ref::Bitmap1 mask(sw, sh);
    std::vector<uint32_t> tile(64);
    for (auto& v : tile) v = rng();
    uint32_t running_bad = 0, plain_bad = 0;
    for (uint32_t trial = 0; trial < 12; ++trial) {
        for (int32_t y = 0; y < sh; ++y) for (int32_t x = 0; x < sw; ++x) mask.set(x, y, (rng() % (2 + trial % 3)) == 0);
        Surface dst = make_surface(0x100000, tw, th);
        Surface src = make_surface(0x200000, sw, sh);
        Surface mask_s = make_surface(0x400000, uint32_t(mask.cx), uint32_t(mask.cy), mask.stride);
        load_surface(src, source);
        mask_s.bytes = mask.bits;
        Raster2dRopRequest r;
        r.destination = view(dst);
        r.destination_rect = {0, 0, tw, th};
        r.has_source = true;
        r.source = view(src);
        r.source_left = 0;
        r.source_top = 0;
        r.source_width = sw;
        r.source_height = sh;
        r.has_pattern = true;
        r.pattern = tile.data();
        r.brush_origin = {int32_t(trial) - 5, 3};
        r.has_mask = true;
        r.stretch_mask = true;
        r.running_phase = (trial % 4) != 3;
        r.mask = {mask_s.gpu, mask_s.pitch, uint32_t(mask.cx), uint32_t(mask.cy), Raster2dFormat::mono1};
        r.rop4 = trial % 2 == 0 ? 0xb8b8u : 0xc0c0u;
        ref::SourceView sv{};
        sv.colour = &source;
        sv.cx = sw;
        sv.cy = sh;
        const uint32_t m = run_dib_call(kernels, r, sv, &mask, tile, {0, 0, tw, th}, {0, 0, sw, sh},
                                        {r.brush_origin.x, r.brush_origin.y}, 40, dst, target,
                                        {as_interp(src), as_interp(mask_s)});
        (r.running_phase ? running_bad : plain_bad) += m != 0 ? 1u : 0u;
    }
    expect(running_bad == 0, "masked tile stretch: running PatternX across waves and workgroups equals the DIB code");
    expect(plain_bad == 3, "without the running phase a masked tile differs from the DIB code (the phase matters)");

    // Translated sources, mirrored and with a negative source extent.
    uint32_t translated_bad = 0, translated_runs = 0;
    for (uint32_t format = 0; format < 3; ++format) {
        for (uint32_t trial = 0; trial < 6; ++trial) {
            const uint32_t bpp = format + 1;
            ref::BitmapBytes bytes(23, 9, bpp);
            for (auto& b : bytes.bits) b = uint8_t(rng());
            ref::Xlate xlate{};
            if (format == 0) {
                xlate.kind = ref::Xlate::Kind::table;
                xlate.table.resize(trial % 2 == 0 ? 256 : 180);
                for (auto& v : xlate.table) v = rng();
            } else if (format == 1) {
                xlate.kind = trial % 2 == 0 ? ref::Xlate::Kind::x555_to_bgr : ref::Xlate::Kind::x565_to_bgr;
            } else {
                xlate.kind = trial % 2 == 0 ? ref::Xlate::Kind::rgb_to_bgr : ref::Xlate::Kind::trivial;
            }
            const Raster2dFormat f = format == 0 ? Raster2dFormat::index8 : format == 1 ? Raster2dFormat::rgb16 :
                                                                                         Raster2dFormat::rgb24;
            const uint32_t table_bytes = raster2d_table_bytes(f);
            Surface table = make_surface(0x600000, (table_bytes + bytes.stride * 9 + 3) / 4, 1);
            for (uint32_t v = 0; v < table_bytes / 4; ++v) {
                const uint32_t entry = format == 2 ? xlate((v % 256) << (8 * (v / 256))) : xlate(v);
                std::memcpy(table.bytes.data() + v * 4, &entry, 4);
            }
            std::memcpy(table.bytes.data() + table_bytes, bytes.bits.data(), bytes.bits.size());
            Surface dst = make_surface(0x100000, 40, 12);
            ref::Bitmap32 t2(40, 12);
            for (auto& v : t2.px) v = rng();
            // The DIB SourceRect: inverted (negative extent) on odd trials.
            const ref::Rect sr = trial % 2 == 0 ? ref::Rect{2, 1, 21, 8} : ref::Rect{19, 7, 4, 2};
            const bool mirror_x = trial % 3 == 1;
            const ref::Rect dr = mirror_x ? ref::Rect{37, 0, 0, 12} : ref::Rect{0, 0, 37, 12};
            const int32_t w = sr.right - sr.left, h = sr.bottom - sr.top;
            const bool dib_mirror_x = mirror_x != (w < 0), dib_mirror_y = h < 0;
            Raster2dRopRequest r;
            r.destination = view(dst);
            r.destination_rect = {0, 0, 37, 12};
            r.has_source = true;
            r.source = {table.gpu + table_bytes, bytes.stride, uint32_t(bytes.cx), uint32_t(bytes.cy), f};
            r.source_table = table.gpu;
            r.source_left = dib_mirror_x ? sr.right : sr.left;
            r.source_top = dib_mirror_y ? sr.bottom : sr.top;
            r.source_width = w < 0 ? -w : w;
            r.source_height = h < 0 ? -h : h;
            r.source_sign_x = (dib_mirror_x ? -1 : 1) * (w < 0 ? -1 : 1);
            r.source_sign_y = (dib_mirror_y ? -1 : 1) * (h < 0 ? -1 : 1);
            r.rop4 = trial % 3 == 2 ? 0x6666u : 0xccccu;
            r.skip_outside_source = (r.rop4 & 0xffu) == 0xccu;
            ref::SourceView sv{};
            sv.bytes = &bytes;
            sv.xlate = &xlate;
            sv.cx = bytes.cx;
            sv.cy = bytes.cy;
            translated_bad += run_dib_call(kernels, r, sv, nullptr, tile, dr, sr, {0, 0}, 0, dst, t2, {as_interp(table)}) != 0
                ? 1u : 0u;
            ++translated_runs;
        }
    }
    expect(translated_runs == 18 && translated_bad == 0,
           "index8/rgb16/rgb24 sources translate through their tables exactly (mirrored, negative extents)");
}

// --------------------------------------------------- pattern lines (EngLineTo)

void test_pattern_lines(const Kernels& kernels) {
    std::mt19937 rng(0x11e5);
    const int32_t w = 40, h = 36;
    std::vector<uint32_t> tile(64);
    for (auto& v : tile) v = rng();
    // Banded clip: two bands, the second split in x.
    const std::vector<ref::Rect> clips[2] = {{{0, 0, w, h}}, {{2, 1, 37, 14}, {0, 14, 15, 33}, {19, 14, 40, 33}}};
    uint32_t bad = 0, lines = 0, octants = 0;

    for (uint32_t trial = 0; trial < 96; ++trial) {
        const int32_t x0 = 8 + int32_t(rng() % 24), y0 = 8 + int32_t(rng() % 20);
        const uint32_t octant = trial % 8;
        octants |= 1u << octant;
        const int32_t major = 3 + int32_t(rng() % 26), minor = int32_t(rng() % uint32_t(major + 1));
        int32_t ddx = (octant & 1) ? minor : major, ddy = (octant & 1) ? major : minor;
        if (octant & 2) ddx = -ddx;
        if (octant & 4) ddy = -ddy;
        const int32_t x1 = x0 + ddx, y1 = y0 + ddy;
        const ref::Point origin{int32_t(rng() % 23) - 11, int32_t(rng() % 23) - 11};
        const uint32_t rop3 = (trial % 3) == 0 ? 0xf0 : (trial % 3) == 1 ? 0x5a : 0xa0;
        const std::vector<ref::Rect>& rects = clips[trial % 2];

        ref::Bitmap32 target(w, h);
        for (auto& v : target.px) v = rng();
        ref::Bitmap32 expected = target;
        ref::eng_line_to(rects, x0, y0, x1, y1, [&](int32_t x, int32_t y) {
            const uint32_t p = tile[size_t(((y - origin.y) & 7) * 8 + ((x - origin.x) & 7))];
            expected.put(x, y, ref::do_rop(rop3, expected.get(x, y), 0, p));
        });

        Surface dst = make_surface(0x100000, w, h);
        load_surface(dst, target);
        gfx11_interp::Buffer* dstb = as_interp(dst);
        std::vector<gfx11_interp::Buffer*> buffers = {dstb};
        for (const ref::Rect& clip : rects) {
            Raster2dLineRequest r;
            r.destination = view(dst);
            r.clip_bounds = {clip.left, clip.top, clip.right, clip.bottom};
            r.x0 = x0;
            r.y0 = y0;
            r.x1 = x1;
            r.y1 = y1;
            r.rop3 = rop3;
            r.load_destination = (((rop3 ^ (rop3 >> 1)) & 0x55u) != 0);
            r.has_pattern = true;
            r.pattern = tile.data();
            r.brush_origin = {origin.x, origin.y};
            Raster2dDescriptors d{};
            raster2d_buffer_descriptor(dst.gpu, dst.pitch * dst.height, d.destination);
            uint32_t args[64];
            uint32_t steps = 0;
            raster2d_build_line_args(r, d, args, steps);
            raster2d_buffer_descriptor(0x90000000ull + raster2d_args_bytes, raster2d_pattern_bytes,
                                       *reinterpret_cast<uint32_t(*)[4]>(args + line_args::pattern));
            const uint32_t groups = (steps + raster2d_workgroup_threads - 1) / raster2d_workgroup_threads;
            const Raster2dRect grid_rect{0, 0, int32_t(groups * raster2d_workgroup_threads), 1};
            run_kernel(kernels, Raster2dKernel::line, args, reinterpret_cast<const uint8_t*>(tile.data()), 64,
                       grid_rect, buffers);
        }

        uint32_t mismatches = 0;
        for (int32_t y = 0; y < h; ++y) {
            for (int32_t x = 0; x < w; ++x) {
                uint32_t got = 0;
                std::memcpy(&got, dstb->bytes.data() + size_t(y) * dst.pitch + size_t(x) * 4, 4);
                mismatches += got != expected.get(x, y) ? 1u : 0u;
            }
        }

        ++lines;
        bad += mismatches != 0 ? 1u : 0u;
    }

    expect(octants == 0xffu && lines == 96, "pattern lines cover all eight octants");
    expect(bad == 0, "pattern line kernel: EngLineTo pixel set with tile texel ((x - org.x) & 7, (y - org.y) & 7)");
}

} // namespace

int main() {
    Kernels kernels;
    test_divisor_identity();
    test_div255_range();
    test_rop_all_codes(kernels);
    test_pattern_origin(kernels);
    test_mono_glyph(kernels);
    test_colour_key(kernels);
    test_stretch(kernels);
    test_alpha(kernels, false);
    test_alpha(kernels, true);
    test_coverage8(kernels);
    test_gradient_rect(kernels, true);
    test_gradient_rect(kernels, false);
    test_gradient_triangle(kernels);
    test_line(kernels, 2, 2, 40, 10, "cosmetic line shallow (x-major)");
    test_line(kernels, 4, 4, 10, 40, "cosmetic line steep (y-major)");
    test_line(kernels, 40, 6, 3, 30, "cosmetic line negative slope");
    test_line(kernels, 5, 20, 45, 20, "cosmetic line horizontal");
    test_mirrored_stretch(kernels);
    test_running_phase_and_translation(kernels);
    test_pattern_lines(kernels);
    test_pm4_ib(kernels);
    test_engine_triangles();
    test_engine_byte_formats();
    test_engine_legacy_rows();
    std::printf("raster2d: %u checks, %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

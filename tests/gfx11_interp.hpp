/*
Module: rx7900.tests.gfx11_interp
Purpose: Independent decoder and wave32 interpreter for the GFX11 instruction
subset the raster2d kernels use, for host validation. It decodes the raw dwords
(it shares no tables with the encoder), models SGPRs, 32 VGPR lanes, EXEC, SCC and
the constant/inline operand map, executes SALU/VALU/compare/branch and SMEM scalar
loads and MUBUF raw buffer loads/stores with OOB_SELECT_RAW bounds, and runs the
generated kernels on synthetic memory. No GPU; no OS.

Graphics additions (2026-10-23, OpenGL shader tests): float transcendental VALU and
float compares, m0, s_wqm_b32, s_sendmsg capture, s_buffer_load through a V#, MUBUF
b128 with idxen/offen and GFX11 swizzled/structured addressing, MIMG 2D sampling of
a linear RGBA8 texture decoded from the T#/S# words, lds_param_load from a modelled
per-quad parameter LDS, VINTERP p10/p2 quad-lane semantics, and EXP exec capture.
The NGG/PA/SC/PS draw model at the end of this header (namespace graphics) runs a
complete VS+PS pair over a triangle list. Everything here decodes raw dwords and
descriptor words independently of the encoder and of gl_gfx11_abi.hpp.
*/
#pragma once

#include <climits>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <vector>

#include "sdma.hpp"

namespace gfx11_interp {

constexpr uint32_t lanes = 32;

struct Buffer final {
    uint64_t base = 0;
    std::vector<uint8_t> bytes;
};

struct Memory final {
    std::vector<Buffer*> buffers;
    // Optional reader for addresses no buffer covers (the pipeline model passes
    // the session's page tables); pages it returns are cached read-only copies.
    bool (*fallback)(void* context, uint64_t address, uint8_t* bytes, size_t count) = nullptr;
    void* fallback_context = nullptr;
    std::deque<Buffer> cached;

    Buffer* find(uint64_t address, uint32_t size) noexcept {
        for (Buffer* buffer : buffers) {
            if (address >= buffer->base && address + size <= buffer->base + buffer->bytes.size()) {
                return buffer;
            }
        }

        for (Buffer& buffer : cached) {
            if (address >= buffer.base && address + size <= buffer.base + buffer.bytes.size()) {
                return &buffer;
            }
        }

        if (fallback == nullptr || size > 4096u) {
            return nullptr;
        }

        // Cache the 4 KiB page(s) around the access (two when it straddles).
        const uint64_t first = address & ~uint64_t{4095};
        const uint64_t last = (address + size - 1u) & ~uint64_t{4095};
        Buffer page{};
        page.base = first;
        page.bytes.assign(static_cast<size_t>(last - first + 4096u), 0);
        if (!fallback(fallback_context, first, page.bytes.data(), page.bytes.size())) {
            return nullptr;
        }
        cached.push_back(std::move(page));
        return &cached.back();
    }
};

// One EXP instruction's result per lane, recorded so graphics-shader tests can
// read a pixel shader's MRT color or a vertex shader's position output.
struct ExportRecord final {
    uint32_t target = 0;
    uint32_t enable = 0;
    bool done = false;
    uint32_t data[4][lanes] = {};
    uint32_t exec = 0;          // EXEC when the export issued (graphics model)
    uint32_t sequence = 0;      // program-order event number (exports and messages)
};

// One s_sendmsg with the m0 value at issue (graphics model).
struct MessageRecord final {
    uint32_t id = 0;
    uint32_t m0 = 0;
    uint32_t sequence = 0;
};

// Pixel-shader parameter LDS as the SPI fills it: per quad of the wave, per
// attribute slot and channel, the P0, P10 = P1 - P0 and P20 = P2 - P0 float bits of
// the quad's primitive. lds_param_load returns P0/P10/P20 in quad lanes 0/1/2 when
// m0 equals prim_mask; anything else yields a NaN pattern so a wrong m0 is visible.
struct ParamLds final {
    uint32_t prim_mask = 0;
    uint32_t attributes = 0;
    std::vector<uint32_t> data;   // [quad][attribute][channel][3]

    uint32_t at(uint32_t quad, uint32_t attribute, uint32_t channel, uint32_t term) const noexcept {
        const size_t index = ((static_cast<size_t>(quad) * attributes + attribute) * 4u + channel) * 3u + term;
        return attribute < attributes && index < data.size() ? data[index] : 0x7fc00000u;
    }
};

struct Wave final {
    uint32_t sgpr[128] = {};
    uint32_t vgpr[256][lanes] = {};
    uint32_t exec = 0xffffffffu;
    bool scc = false;
    // EXP outputs captured in program order (bounded; graphics shaders export a
    // few positions/params and one color).
    ExportRecord exports[16] = {};
    uint32_t export_count = 0;
    // Graphics model state.
    MessageRecord messages[8] = {};
    uint32_t message_count = 0;
    uint32_t sequence = 0;
    const ParamLds* lds = nullptr;
    uint32_t unsupported = 0;     // count of graphics encodings the model rejected
};

namespace detail {

inline float as_float(uint32_t bits) noexcept {
    float value = 0;
    std::memcpy(&value, &bits, 4);
    return value;
}

inline uint32_t as_bits(float value) noexcept {
    uint32_t bits = 0;
    std::memcpy(&bits, &value, 4);
    return bits;
}


inline uint32_t read_scalar(const Wave& w, uint32_t code, const uint32_t* literal) noexcept {
    if (code <= 105) {
        return w.sgpr[code];
    }

    if (code == 106) {
        return w.sgpr[106];   // vcc_lo stored in sgpr slot 106
    }

    if (code == 125) {
        return w.sgpr[125];   // m0 stored in sgpr slot 125
    }

    if (code == 124) {
        return 0;             // null
    }

    if (code == 126) {
        return w.exec;        // exec_lo
    }

    if (code >= 128 && code <= 192) {
        return code - 128;    // inline 0..64
    }

    if (code >= 193 && code <= 208) {
        return static_cast<uint32_t>(-static_cast<int32_t>(code - 192)); // -1..-16
    }

    if (code == 255 && literal != nullptr) {
        return *literal;
    }

    return 0;
}

inline uint32_t read_vector(const Wave& w, uint32_t code, uint32_t lane,
                            const uint32_t* literal) noexcept {
    if (code >= 256) {
        return w.vgpr[code - 256][lane];
    }

    return read_scalar(w, code, literal);
}

inline void write_scalar(Wave& w, uint32_t code, uint32_t value) noexcept {
    if (code <= 107) {
        w.sgpr[code] = value;
    } else if (code == 125) {
        w.sgpr[125] = value;  // m0
    } else if (code == 126) {
        w.exec = value;
    }
}

inline bool uses_literal(uint32_t code) noexcept { return code == 255; }

inline uint32_t mulhi(uint32_t a, uint32_t b) noexcept {
    return static_cast<uint32_t>((uint64_t{a} * b) >> 32);
}


// ---- Graphics model helpers ----

inline uint32_t read_u32(Memory* memory, uint64_t address, bool& ok) noexcept {
    uint32_t value = 0;
    Buffer* buffer = memory != nullptr ? memory->find(address, 4) : nullptr;

    if (buffer == nullptr) {
        ok = false;
        return 0;
    }

    std::memcpy(&value, buffer->bytes.data() + (address - buffer->base), 4);
    return value;
}

inline bool write_u32(Memory* memory, uint64_t address, uint32_t value) noexcept {
    Buffer* buffer = memory != nullptr ? memory->find(address, 4) : nullptr;

    if (buffer == nullptr) {
        return false;
    }

    std::memcpy(buffer->bytes.data() + (address - buffer->base), &value, 4);
    return true;
}

// s_buffer_load_b32..b512: sbase is a V# quad; offsets beyond NUM_RECORDS read 0.
inline void smem_buffer_load(Wave& w, uint32_t w0, uint32_t w1, Memory* memory) noexcept {
    const uint32_t op = (w0 >> 18) & 0xff;
    const uint32_t sdata = (w0 >> 6) & 0x7f;
    const uint32_t sbase = (w0 & 0x3f) << 1;
    const uint32_t soffset_code = (w1 >> 25) & 0x7f;
    const uint32_t offset = w1 & 0x1fffffu;
    const uint32_t count = op == 8 ? 1u : op == 9 ? 2u : op == 10 ? 4u : op == 11 ? 8u : 16u;
    const uint64_t base = w.sgpr[sbase] | (uint64_t{w.sgpr[sbase + 1] & 0xffffu} << 32);
    const uint32_t records = w.sgpr[sbase + 2];
    const uint32_t soffset = soffset_code == 124 ? 0u : read_scalar(w, soffset_code, nullptr);

    for (uint32_t i = 0; i < count; ++i) {
        const uint32_t byte = soffset + offset + i * 4u;
        bool ok = true;
        uint32_t value = 0;

        if (byte + 4u <= records) {
            value = read_u32(memory, base + byte, ok);
        }

        w.sgpr[sdata + i] = ok ? value : 0u;
    }
}

// MUBUF with the GFX11 buffer addressing the graphics shaders use: offen or idxen,
// optional swizzle (SWIZZLE_ENABLE element size, INDEX_STRIDE), and the OOB_SELECT
// rules quoted in Mesa ac_descriptors.c for GFX11.
inline void mubuf_graphics(Wave& w, uint32_t w0, uint32_t w1, Memory* memory) noexcept {
    const uint32_t offset = w0 & 0xfff;
    const uint32_t op = (w0 >> 18) & 0x7f;
    const uint32_t vaddr = w1 & 0xff;
    const uint32_t vdata = (w1 >> 8) & 0xff;
    const uint32_t srsrc = ((w1 >> 16) & 0x1f) * 4u;
    const bool offen = ((w1 >> 22) & 1u) != 0;
    const bool idxen = ((w1 >> 23) & 1u) != 0;
    const uint32_t soffset_code = (w1 >> 24) & 0xff;
    const uint32_t soffset = soffset_code == 128 ? 0u : read_scalar(w, soffset_code, nullptr);
    const uint32_t d0 = w.sgpr[srsrc], d1 = w.sgpr[srsrc + 1], d2 = w.sgpr[srsrc + 2],
                   d3 = w.sgpr[srsrc + 3];
    const uint64_t base = d0 | (uint64_t{d1 & 0xffffu} << 32);
    const uint32_t stride = (d1 >> 16) & 0x3fffu;
    const uint32_t swizzle = (d1 >> 30) & 3u;          // 0 off, 1 4B, 2 8B, 3 16B elements
    const uint32_t index_stride = 8u << ((d3 >> 21) & 3u);
    const uint32_t oob = (d3 >> 28) & 3u;
    const uint32_t records = d2;
    const bool store = op == 0x1a || op == 0x1d;
    const uint32_t dwords = (op == 0x17 || op == 0x1d) ? 4u : 1u;
    const uint32_t payload = dwords * 4u;

    for (uint32_t lane = 0; lane < lanes; ++lane) {
        if (!((w.exec >> lane) & 1u)) {
            continue;
        }

        const uint32_t index = idxen ? w.vgpr[vaddr][lane] : 0u;
        const uint32_t voffset = offen ? w.vgpr[vaddr][lane] : 0u;
        const uint32_t inst_offset = voffset + offset;
        bool out_of_bounds = false;

        if (oob == 3) {
            out_of_bounds = (swizzle != 0 && stride != 0)
                                ? (index >= records || inst_offset + payload > stride)
                                : (inst_offset + payload > records);
        } else if (oob == 0) {
            out_of_bounds = index >= records || inst_offset + payload > stride;
        } else if (oob == 1) {
            out_of_bounds = index >= records;
        } else {
            out_of_bounds = records == 0;
        }

        uint64_t buffer_offset = 0;

        if (swizzle != 0) {
            const uint32_t element = 2u << swizzle;    // 4, 8 or 16 bytes
            const uint64_t index_msb = index / index_stride;
            const uint64_t index_lsb = index % index_stride;
            const uint64_t offset_msb = inst_offset / element;
            const uint64_t offset_lsb = inst_offset % element;
            buffer_offset = (index_msb * stride + offset_msb * element) * index_stride +
                            index_lsb * element + offset_lsb;
        } else {
            buffer_offset = uint64_t{index} * stride + inst_offset;
        }

        const uint64_t address = base + soffset + buffer_offset;

        for (uint32_t i = 0; i < dwords; ++i) {
            if (store) {
                if (!out_of_bounds && !write_u32(memory, address + i * 4u, w.vgpr[vdata + i][lane])) {
                    ++w.unsupported;   // a store to unmapped model memory is a test failure
                }
            } else {
                bool ok = true;
                const uint32_t value = out_of_bounds ? 0u : read_u32(memory, address + i * 4u, ok);
                w.vgpr[vdata + i][lane] = ok ? value : 0u;
            }
        }
    }
}

// MIMG image_sample / image_sample_lz on a 2D linear 8_8_8_8_UNORM T# with the S#
// wrap and filter fields; any other descriptor is counted as unsupported. LOD is 0
// (image_sample in this model also samples level 0), using XY_MAG_FILTER.
inline void mimg_sample(Wave& w, uint32_t w0, uint32_t w1, Memory* memory) noexcept {
    const uint32_t op = (w0 >> 18) & 0xff;
    const uint32_t dmask = (w0 >> 8) & 0xf;
    const uint32_t dim = (w0 >> 2) & 7u;
    const uint32_t vaddr = w1 & 0xff;
    const uint32_t vdata = (w1 >> 8) & 0xff;
    const uint32_t srsrc = ((w1 >> 16) & 0x1f) * 4u;
    const uint32_t ssamp = ((w1 >> 26) & 0x1f) * 4u;
    const uint32_t* t = &w.sgpr[srsrc];
    const uint32_t* sm = &w.sgpr[ssamp];
    const uint64_t base = (uint64_t{t[0]} | (uint64_t{t[1] & 0xffu} << 32)) << 8;
    const uint32_t format = (t[1] >> 20) & 0xff;
    const uint32_t width = (((t[1] >> 30) & 3u) | ((t[2] & 0xfffu) << 2)) + 1u;
    const uint32_t height = ((t[2] >> 14) & 0x3fffu) + 1u;
    const uint32_t type = t[3] >> 28;
    const uint32_t sw_mode = (t[3] >> 20) & 0x1fu;
    const uint32_t pitch = ((t[4] & 0x1fffu) | (((t[4] >> 13) & 1u) << 13)) + 1u;
    const uint32_t clamp_x = sm[0] & 7u;
    const uint32_t clamp_y = (sm[0] >> 3) & 7u;
    const uint32_t filter = (sm[2] >> 20) & 3u;

    if ((op != 0x1b && op != 0x1f) || dim != 1 || format != 42 || type != 9 || sw_mode != 0 ||
        pitch < width || (clamp_x != 0 && clamp_x != 2) || (clamp_y != 0 && clamp_y != 2) ||
        filter > 1) {
        ++w.unsupported;
        return;
    }

    auto wrap = [](int64_t coord, uint32_t size, uint32_t mode) -> uint32_t {
        const int64_t n = size;
        if (mode == 0) {
            int64_t m = coord % n;
            return static_cast<uint32_t>(m < 0 ? m + n : m);
        }
        return static_cast<uint32_t>(coord < 0 ? 0 : (coord >= n ? n - 1 : coord));
    };

    for (uint32_t lane = 0; lane < lanes; ++lane) {
        if (!((w.exec >> lane) & 1u)) {
            continue;
        }

        const float s = as_float(w.vgpr[vaddr][lane]);
        const float tc = as_float(w.vgpr[vaddr + 1][lane]);
        float rgba[4] = {0, 0, 0, 0};
        bool ok = true;

        auto texel = [&](uint32_t x, uint32_t y, float out[4]) {
            const uint32_t packed = read_u32(memory, base + (uint64_t{y} * pitch + x) * 4u, ok);
            for (uint32_t c = 0; c < 4; ++c) {
                out[c] = static_cast<float>((packed >> (8 * c)) & 0xffu) / 255.0f;
            }
        };

        const float u = s * static_cast<float>(width);
        const float v = tc * static_cast<float>(height);

        if (filter == 0) {
            texel(wrap(static_cast<int64_t>(std::floor(u)), width, clamp_x),
                  wrap(static_cast<int64_t>(std::floor(v)), height, clamp_y), rgba);
        } else {
            const float fu = u - 0.5f;
            const float fv = v - 0.5f;
            const int64_t x0 = static_cast<int64_t>(std::floor(fu));
            const int64_t y0 = static_cast<int64_t>(std::floor(fv));
            const float ax = fu - static_cast<float>(x0);
            const float ay = fv - static_cast<float>(y0);
            float c00[4], c10[4], c01[4], c11[4];
            texel(wrap(x0, width, clamp_x), wrap(y0, height, clamp_y), c00);
            texel(wrap(x0 + 1, width, clamp_x), wrap(y0, height, clamp_y), c10);
            texel(wrap(x0, width, clamp_x), wrap(y0 + 1, height, clamp_y), c01);
            texel(wrap(x0 + 1, width, clamp_x), wrap(y0 + 1, height, clamp_y), c11);
            for (uint32_t c = 0; c < 4; ++c) {
                const float top = c00[c] + (c10[c] - c00[c]) * ax;
                const float bottom = c01[c] + (c11[c] - c01[c]) * ax;
                rgba[c] = top + (bottom - top) * ay;
            }
        }

        if (!ok) {
            ++w.unsupported;
        }

        // DST_SEL must be X, Y, Z, W (4..7) as the ABI builds it.
        uint32_t out = vdata;

        for (uint32_t c = 0; c < 4; ++c) {
            if (((dmask >> c) & 1u) == 0) {
                continue;
            }

            const uint32_t sel = (t[3] >> (3 * c)) & 7u;
            const float value = sel >= 4 ? rgba[sel - 4] : (sel == 1 ? 1.0f : 0.0f);
            w.vgpr[out++][lane] = as_bits(value);
        }
    }
}

// lds_param_load (op 0) from the modelled parameter LDS; lds_direct_load is not
// used by the generated shaders and is counted as unsupported.
inline void ldsdir(Wave& w, uint32_t w0) noexcept {
    const uint32_t vdst = w0 & 0xff;
    const uint32_t channel = (w0 >> 8) & 3u;
    const uint32_t attribute = (w0 >> 10) & 0x3fu;
    const uint32_t op = (w0 >> 20) & 3u;

    if (op != 0 || w.lds == nullptr) {
        ++w.unsupported;
        return;
    }

    for (uint32_t lane = 0; lane < lanes; ++lane) {
        if (!((w.exec >> lane) & 1u)) {
            continue;
        }

        const uint32_t term = lane & 3u;
        uint32_t value = 0x7fc00000u;

        if (w.sgpr[125] == w.lds->prim_mask && term < 3) {
            value = w.lds->at(lane / 4u, attribute, channel, term);
        }

        w.vgpr[vdst][lane] = value;
    }
}

// v_interp_p10_f32: D = fma(src0[quad lane 1], src1, src2[quad lane 0]);
// v_interp_p2_f32:  D = fma(src0[quad lane 2], src1, src2). Results are computed
// for all lanes first, then written (sources may alias the destination).
inline void vinterp(Wave& w, uint32_t w0, uint32_t w1) noexcept {
    const uint32_t vdst = w0 & 0xff;
    const uint32_t op = (w0 >> 16) & 0x7f;
    const uint32_t s0 = w1 & 0x1ff;
    const uint32_t s1 = (w1 >> 9) & 0x1ff;
    const uint32_t s2 = (w1 >> 18) & 0x1ff;

    if ((op != 0 && op != 1) || s0 < 256 || s1 < 256 || s2 < 256 || (w1 >> 29) != 0 ||
        ((w0 >> 11) & 0x1fu) != 0) {
        ++w.unsupported;
        return;
    }

    uint32_t result[lanes] = {};

    for (uint32_t lane = 0; lane < lanes; ++lane) {
        const uint32_t quad = lane & ~3u;
        const float a = as_float(w.vgpr[s0 - 256][quad + (op == 0 ? 1u : 2u)]);
        const float b = as_float(w.vgpr[s1 - 256][lane]);
        const float c = op == 0 ? as_float(w.vgpr[s2 - 256][quad]) : as_float(w.vgpr[s2 - 256][lane]);
        result[lane] = as_bits(std::fma(a, b, c));
    }

    for (uint32_t lane = 0; lane < lanes; ++lane) {
        if ((w.exec >> lane) & 1u) {
            w.vgpr[vdst][lane] = result[lane];
        }
    }
}

} // namespace detail

// Execute from dword 0 until s_endpgm or the end of the program.
inline void run(const uint8_t* code, uint32_t dword_count, Wave& w, Memory* memory = nullptr) {
    auto word = [&](uint32_t index) -> uint32_t {
        uint32_t value = 0;
        std::memcpy(&value, code + index * 4u, 4);
        return value;
    };

    uint32_t pc = 0;

    while (pc < dword_count) {
        const uint32_t w0 = word(pc);
        uint32_t next = pc + 1;

        auto scalar_literal = [&](uint32_t s0, uint32_t s1) -> const uint32_t* {
            static uint32_t storage;

            if (detail::uses_literal(s0) || detail::uses_literal(s1)) {
                storage = word(pc + 1);
                next = pc + 2;
                return &storage;
            }

            return nullptr;
        };

        if ((w0 >> 31) == 0) {
            // VOP1 / VOPC / VOP2.
            if ((w0 >> 25) == 0x3f) {               // VOP1
                const uint32_t vdst = (w0 >> 17) & 0xff;
                const uint32_t op = (w0 >> 9) & 0xff;
                const uint32_t s0 = w0 & 0x1ff;
                const uint32_t* lit = nullptr;
                static uint32_t storage;

                if (detail::uses_literal(s0)) {
                    storage = word(pc + 1);
                    lit = &storage;
                    next = pc + 2;
                }

                if (op == 0x02) {                    // v_readfirstlane_b32 (SGPR in VDST)
                    uint32_t first = 0;

                    for (uint32_t lane = 0; lane < lanes; ++lane) {
                        if ((w.exec >> lane) & 1u) {
                            first = detail::read_vector(w, s0, lane, lit);
                            break;
                        }
                    }

                    detail::write_scalar(w, vdst, first);
                    pc = next;
                    continue;
                }

                for (uint32_t lane = 0; lane < lanes; ++lane) {
                    if (!((w.exec >> lane) & 1u)) {
                        continue;
                    }

                    const uint32_t a = detail::read_vector(w, s0, lane, lit);
                    uint32_t d = a;

                    switch (op) {
                    case 0x01: d = a; break;                               // v_mov_b32
                    case 0x05: d = detail::as_bits(static_cast<float>(static_cast<int32_t>(a))); break; // cvt_f32_i32
                    case 0x06: d = detail::as_bits(static_cast<float>(a)); break;         // cvt_f32_u32
                    case 0x07: {                                           // cvt_u32_f32 (clamping)
                        const float f = detail::as_float(a);
                        d = !(f > 0) ? 0u : f >= 4294967296.0f ? 0xffffffffu : static_cast<uint32_t>(f);
                        break;
                    }
                    case 0x08: {                                           // cvt_i32_f32 (clamping)
                        const float f = detail::as_float(a);
                        d = f != f ? 0u : f <= -2147483648.0f ? 0x80000000u : f >= 2147483648.0f ? 0x7fffffffu
                                                                          : static_cast<uint32_t>(static_cast<int32_t>(f));
                        break;
                    }
                    case 0x11: case 0x12: case 0x13: case 0x14:            // cvt_f32_ubyte0..3
                        d = detail::as_bits(static_cast<float>((a >> (8u * (op - 0x11u))) & 0xffu));
                        break;
                    case 0x20: {                                           // v_fract_f32
                        const float f = detail::as_float(a);
                        const float r = f - std::floor(f);
                        d = detail::as_bits(r < 1.0f ? r : 0.99999994f);
                        break;
                    }
                    case 0x22: d = detail::as_bits(std::ceil(detail::as_float(a))); break;   // v_ceil_f32
                    case 0x24: d = detail::as_bits(std::floor(detail::as_float(a))); break;  // v_floor_f32
                    case 0x2a: { const float f = detail::as_float(a); d = detail::as_bits(f != 0 ? 1.0f / f : 0.0f); break; } // rcp_f32
                    case 0x37: d = ~a; break;                              // v_not_b32
                    case 0x25: d = detail::as_bits(std::exp2(detail::as_float(a))); break;  // v_exp_f32
                    case 0x27: d = detail::as_bits(std::log2(detail::as_float(a))); break;  // v_log_f32
                    case 0x2e: d = detail::as_bits(1.0f / std::sqrt(detail::as_float(a))); break; // v_rsq_f32
                    case 0x33: d = detail::as_bits(std::sqrt(detail::as_float(a))); break;  // v_sqrt_f32
                    default: d = a; break;
                    }

                    w.vgpr[vdst][lane] = d;
                }
            } else if ((w0 >> 25) == 0x3e) {        // VOPC (unused by kernels)
                next = pc + 1;
            } else {                                 // VOP2
                const uint32_t op = (w0 >> 25) & 0x3f;
                const uint32_t vdst = (w0 >> 17) & 0xff;
                const uint32_t vsrc1 = (w0 >> 9) & 0xff;
                const uint32_t s0 = w0 & 0x1ff;
                const uint32_t* lit = nullptr;
                static uint32_t storage;

                if (detail::uses_literal(s0)) {
                    storage = word(pc + 1);
                    lit = &storage;
                    next = pc + 2;
                }

                for (uint32_t lane = 0; lane < lanes; ++lane) {
                    if (!((w.exec >> lane) & 1u)) {
                        continue;
                    }

                    const uint32_t a = detail::read_vector(w, s0, lane, lit);
                    const uint32_t b = w.vgpr[256 + vsrc1 - 256][lane];
                    uint32_t d = 0;

                    const float fa = detail::as_float(a);
                    const float fb = detail::as_float(b);

                    switch (op) {
                    case 0x03: d = detail::as_bits(fa + fb); break;          // v_add_f32
                    case 0x04: d = detail::as_bits(fa - fb); break;          // v_sub_f32
                    case 0x05: d = detail::as_bits(fb - fa); break;          // v_subrev_f32
                    case 0x07: d = detail::as_bits(fa * fb); break;          // v_mul_legacy_f32
                    case 0x08: d = detail::as_bits(fa * fb); break;          // v_mul_f32
                    case 0x0f: d = detail::as_bits(fa < fb ? fa : fb); break; // v_min_f32
                    case 0x10: d = detail::as_bits(fa > fb ? fa : fb); break; // v_max_f32
                    case 0x0b: d = (a & 0xffffffu) * (b & 0xffffffu); break; // v_mul_u32_u24
                    case 0x12: d = static_cast<int32_t>(a) > static_cast<int32_t>(b) ? a : b; break; // max_i32
                    case 0x13: d = a < b ? a : b; break;                     // v_min_u32
                    case 0x18: d = b << (a & 31u); break;                    // v_lshlrev_b32
                    case 0x19: d = b >> (a & 31u); break;                    // v_lshrrev_b32
                    case 0x1a: d = static_cast<uint32_t>(static_cast<int32_t>(b) >> (a & 31u)); break; // ashrrev
                    case 0x1b: d = a & b; break;                             // v_and_b32
                    case 0x1c: d = a | b; break;                             // v_or_b32
                    case 0x1d: d = a ^ b; break;                             // v_xor_b32
                    case 0x25: d = a + b; break;                             // v_add_nc_u32
                    case 0x26: d = a - b; break;                             // v_sub_nc_u32
                    case 0x27: d = b - a; break;                             // v_subrev_nc_u32
                    default: d = 0; break;
                    }

                    w.vgpr[vdst][lane] = d;
                }
            }

            pc = next;
            continue;
        }

        if ((w0 >> 30) == 2) {
            // Scalar: SOPP / SOP1 / SOPC / SOPK / SOP2.
            const uint32_t group = (w0 >> 23) & 0x1ff;

            if (group == 0x17f) {                    // SOPP
                const uint32_t op = (w0 >> 16) & 0x7f;
                const int32_t simm = static_cast<int16_t>(w0 & 0xffff);

                switch (op) {
                case 0x20: pc = static_cast<uint32_t>(static_cast<int32_t>(pc) + 1 + simm); continue; // s_branch
                case 0x21: if (!w.scc) { pc = static_cast<uint32_t>(static_cast<int32_t>(pc) + 1 + simm); continue; } break;
                case 0x22: if (w.scc) { pc = static_cast<uint32_t>(static_cast<int32_t>(pc) + 1 + simm); continue; } break;
                case 0x25: if (w.exec == 0) { pc = static_cast<uint32_t>(static_cast<int32_t>(pc) + 1 + simm); continue; } break;
                case 0x30: return;                   // s_endpgm
                case 0x36:                           // s_sendmsg (graphics model capture)
                    if (w.message_count < 8) {
                        w.messages[w.message_count++] = {static_cast<uint32_t>(simm) & 0xffffu,
                                                         w.sgpr[125], w.sequence};
                    }
                    ++w.sequence;
                    break;
                default: break;                      // s_nop / s_waitcnt / s_code_end
                }

                pc = pc + 1;
                continue;
            }

            if (group == 0x17d) {                    // SOP1
                const uint32_t sdst = (w0 >> 16) & 0x7f;
                const uint32_t op = (w0 >> 8) & 0xff;
                const uint32_t s0 = w0 & 0xff;
                const uint32_t* lit = scalar_literal(s0, 0);
                const uint32_t a = detail::read_scalar(w, s0, lit);

                if (op == 0x00) {
                    detail::write_scalar(w, sdst, a);
                } else if (op == 0x18) {             // s_bcnt1_i32_b32
                    uint32_t bits = 0;

                    for (uint32_t bit = 0; bit < 32; ++bit) {
                        bits += (a >> bit) & 1u;
                    }

                    detail::write_scalar(w, sdst, bits);
                    w.scc = bits != 0;
                } else if (op == 0x1c) {             // s_wqm_b32: whole-quad mask
                    uint32_t quads = 0;

                    for (uint32_t quad = 0; quad < 8; ++quad) {
                        quads |= ((a >> (quad * 4)) & 0xfu) != 0 ? 0xfu << (quad * 4) : 0u;
                    }

                    detail::write_scalar(w, sdst, quads);
                    w.scc = quads != 0;
                }

                pc = next;
                continue;
            }

            if (group == 0x17e) {                    // SOPC
                const uint32_t op = (w0 >> 16) & 0x7f;
                const uint32_t s1 = (w0 >> 8) & 0xff;
                const uint32_t s0 = w0 & 0xff;
                const uint32_t* lit = scalar_literal(s0, s1);
                const uint32_t a = detail::read_scalar(w, s0, lit);
                const uint32_t b = detail::read_scalar(w, s1, lit);

                switch (op) {
                case 0x00: w.scc = a == b; break;                                    // eq_i32
                case 0x03: w.scc = static_cast<int32_t>(a) >= static_cast<int32_t>(b); break;
                case 0x04: w.scc = static_cast<int32_t>(a) < static_cast<int32_t>(b); break;
                case 0x06: w.scc = a == b; break;                                    // eq_u32
                case 0x07: w.scc = a != b; break;                                    // lg_u32
                case 0x09: w.scc = a >= b; break;                                    // ge_u32
                case 0x0a: w.scc = a < b; break;                                     // lt_u32
                default: break;
                }

                pc = next;
                continue;
            }

            if (((w0 >> 28) & 0xf) == 0xb) {         // SOPK (s_waitcnt_vscnt: nop)
                pc = pc + 1;
                continue;
            }

            // SOP2.
            const uint32_t op = (w0 >> 23) & 0x7f;
            const uint32_t sdst = (w0 >> 16) & 0x7f;
            const uint32_t s1 = (w0 >> 8) & 0xff;
            const uint32_t s0 = w0 & 0xff;
            const uint32_t* lit = scalar_literal(s0, s1);
            const uint32_t a = detail::read_scalar(w, s0, lit);
            const uint32_t b = detail::read_scalar(w, s1, lit);
            uint32_t d = 0;
            bool set_scc = false;
            bool scc_value = false;

            switch (op) {
            case 0x00: { const uint64_t s = uint64_t{a} + b; d = static_cast<uint32_t>(s); set_scc = true; scc_value = (s >> 32) != 0; break; } // add_u32
            case 0x01: d = a - b; set_scc = true; scc_value = a < b; break;           // sub_u32 (borrow)
            case 0x02: d = a + b; break;                                              // add_i32
            case 0x03: d = a - b; break;                                              // sub_i32
            case 0x04: { const uint64_t s = uint64_t{a} + b + (w.scc ? 1u : 0u); d = static_cast<uint32_t>(s); set_scc = true; scc_value = (s >> 32) != 0; break; } // addc_u32
            case 0x08: d = a << (b & 31u); set_scc = true; scc_value = d != 0; break; // lshl_b32
            case 0x0a: d = a >> (b & 31u); set_scc = true; scc_value = d != 0; break; // lshr_b32
            case 0x16: d = a & b; set_scc = true; scc_value = d != 0; break;          // and_b32
            case 0x18: d = a | b; set_scc = true; scc_value = d != 0; break;          // or_b32
            case 0x1a: d = a ^ b; set_scc = true; scc_value = d != 0; break;          // xor_b32
            case 0x2c: d = a * b; break;                                              // mul_i32
            case 0x2d: d = detail::mulhi(a, b); break;                                // mul_hi_u32
            case 0x30: d = w.scc ? a : b; break;                                      // cselect_b32
            default: d = 0; break;
            }

            detail::write_scalar(w, sdst, d);

            if (set_scc) {
                w.scc = scc_value;
            }

            pc = next;
            continue;
        }

        // (w0 >> 30) == 3: SMEM / MUBUF / EXP / VOP3.
        const uint32_t high6 = w0 >> 26;

        if (high6 == 0x3e) {                         // EXP
            const uint32_t w1 = word(pc + 1);
            const uint32_t enable = w0 & 0xf;
            const uint32_t target = (w0 >> 4) & 0x3f;
            const bool done = ((w0 >> 11) & 1u) != 0;

            if (w.export_count < 16) {
                ExportRecord& record = w.exports[w.export_count++];
                record.target = target;
                record.enable = enable;
                record.done = done;
                record.exec = w.exec;
                record.sequence = w.sequence;

                for (uint32_t channel = 0; channel < 4; ++channel) {
                    if (((enable >> channel) & 1u) == 0) {
                        continue;
                    }

                    const uint32_t vsrc = (w1 >> (channel * 8)) & 0xff;

                    for (uint32_t lane = 0; lane < lanes; ++lane) {
                        if ((w.exec >> lane) & 1u) {
                            record.data[channel][lane] = w.vgpr[vsrc][lane];
                        }
                    }
                }
            }

            ++w.sequence;
            pc = pc + 2;
            continue;
        }

        if (high6 == 0x3d && ((w0 >> 18) & 0xff) >= 8) {  // SMEM s_buffer_load through a V#
            detail::smem_buffer_load(w, w0, word(pc + 1), memory);
            pc = pc + 2;
            continue;
        }

        if (high6 == 0x38 && (((w0 >> 18) & 0x7f) == 0x17 || ((w0 >> 18) & 0x7f) == 0x1d ||
                              ((word(pc + 1) >> 23) & 1u) != 0)) {
            detail::mubuf_graphics(w, w0, word(pc + 1), memory);   // b128 / idxen
            pc = pc + 2;
            continue;
        }

        if (high6 == 0x3c) {                         // MIMG sample
            detail::mimg_sample(w, w0, word(pc + 1), memory);
            pc = pc + 2;
            continue;
        }

        if ((w0 >> 24) == 0xce) {                    // LDSDIR
            detail::ldsdir(w, w0);
            pc = pc + 1;
            continue;
        }

        if ((w0 >> 24) == 0xcd) {                    // VINTERP
            detail::vinterp(w, w0, word(pc + 1));
            pc = pc + 2;
            continue;
        }

        if (high6 == 0x3d) {                         // SMEM
            const uint32_t w1 = word(pc + 1);
            const uint32_t op = (w0 >> 18) & 0xff;
            const uint32_t sdata = (w0 >> 6) & 0x7f;
            const uint32_t sbase = (w0 & 0x3f) << 1;
            const uint32_t soffset_code = (w1 >> 25) & 0x7f;
            const int32_t offset = static_cast<int32_t>(w1 << 11) >> 11; // sign-extend 21 bits
            const uint32_t count = op == 0 ? 1u : op == 1 ? 2u : op == 2 ? 4u : op == 3 ? 8u : 16u;
            const uint64_t base = w.sgpr[sbase] | (uint64_t{w.sgpr[sbase + 1]} << 32);
            const uint32_t soffset = soffset_code == 124 ? 0u : detail::read_scalar(w, soffset_code, nullptr);
            const uint64_t address = base + soffset + static_cast<uint32_t>(offset);

            if (memory != nullptr) {
                Buffer* buffer = memory->find(address, count * 4u);

                for (uint32_t i = 0; i < count; ++i) {
                    uint32_t value = 0;

                    if (buffer != nullptr) {
                        std::memcpy(&value, buffer->bytes.data() + (address - buffer->base) + i * 4u, 4);
                    }

                    w.sgpr[sdata + i] = value;
                }
            }

            pc = pc + 2;
            continue;
        }

        if (high6 == 0x38) {                         // MUBUF
            const uint32_t w1 = word(pc + 1);
            const uint32_t offset = w0 & 0xfff;
            const uint32_t op = (w0 >> 18) & 0x7f;
            const uint32_t vaddr = w1 & 0xff;
            const uint32_t vdata = (w1 >> 8) & 0xff;
            const uint32_t srsrc = ((w1 >> 16) & 0x1f) * 4u;
            const uint32_t soffset_code = (w1 >> 24) & 0xff;
            const uint32_t soffset = soffset_code == 128 ? 0u
                                     : detail::read_scalar(w, soffset_code, nullptr);
            const uint64_t base = w.sgpr[srsrc] | (uint64_t{w.sgpr[srsrc + 1] & 0xffffu} << 32);
            const uint32_t records = w.sgpr[srsrc + 2];
            const uint32_t size = op == 0x10 ? 1u : 4u; // buffer_load_u8 else b32
            const bool is_store = op == 0x1a;

            for (uint32_t lane = 0; lane < lanes; ++lane) {
                if (!((w.exec >> lane) & 1u)) {
                    continue;
                }

                const uint32_t boff = w.vgpr[vaddr][lane] + soffset + offset;

                if (boff + size > records) {
                    if (!is_store) {
                        w.vgpr[vdata][lane] = 0;      // OOB load returns zero
                    }

                    continue;                         // OOB store dropped
                }

                if (memory == nullptr) {
                    continue;
                }

                const uint64_t address = base + boff;
                Buffer* buffer = memory->find(address, size);

                if (buffer == nullptr) {
                    if (!is_store) {
                        w.vgpr[vdata][lane] = 0;
                    }

                    continue;
                }

                uint8_t* ptr = buffer->bytes.data() + (address - buffer->base);

                if (is_store) {
                    std::memcpy(ptr, &w.vgpr[vdata][lane], 4);
                } else if (size == 1) {
                    w.vgpr[vdata][lane] = *ptr;
                } else {
                    std::memcpy(&w.vgpr[vdata][lane], ptr, 4);
                }
            }

            pc = pc + 2;
            continue;
        }

        // VOP3 (high6 == 0x35).
        const uint32_t w1 = word(pc + 1);
        const uint32_t op = (w0 >> 16) & 0x3ff;
        const uint32_t vdst = w0 & 0xff;
        const uint32_t sdst = w0 & 0x7f;
        const uint32_t s0 = w1 & 0x1ff;
        const uint32_t s1 = (w1 >> 9) & 0x1ff;
        const uint32_t s2 = (w1 >> 18) & 0x1ff;
        const uint32_t* lit = nullptr;
        static uint32_t storage;

        if (detail::uses_literal(s0) || detail::uses_literal(s1) || detail::uses_literal(s2)) {
            storage = word(pc + 2);
            lit = &storage;
            next = pc + 3;
        } else {
            next = pc + 2;
        }

        const bool is_compare = op >= 0x10 && op <= 0x7e; // VOPC range in VOP3
        uint32_t mask = 0;

        for (uint32_t lane = 0; lane < lanes; ++lane) {
            if (!((w.exec >> lane) & 1u)) {
                continue;
            }

            const uint32_t a = detail::read_vector(w, s0, lane, lit);
            const uint32_t b = detail::read_vector(w, s1, lane, lit);
            const uint32_t c = detail::read_vector(w, s2, lane, lit);

            if (is_compare) {
                bool t = false;

                switch (op) {
                case 0x41: t = static_cast<int32_t>(a) < static_cast<int32_t>(b); break;
                case 0x43: t = static_cast<int32_t>(a) <= static_cast<int32_t>(b); break;
                case 0x44: t = static_cast<int32_t>(a) > static_cast<int32_t>(b); break;
                case 0x46: t = static_cast<int32_t>(a) >= static_cast<int32_t>(b); break;
                case 0x49: t = a < b; break;
                case 0x4a: t = a == b; break;
                case 0x4b: t = a <= b; break;
                case 0x4c: t = a > b; break;
                case 0x4d: t = a != b; break;
                case 0x4e: t = a >= b; break;
                case 0x11: t = detail::as_float(a) < detail::as_float(b); break;    // lt_f32
                case 0x12: t = detail::as_float(a) == detail::as_float(b); break;   // eq_f32
                case 0x13: t = detail::as_float(a) <= detail::as_float(b); break;   // le_f32
                case 0x14: t = detail::as_float(a) > detail::as_float(b); break;    // gt_f32
                case 0x15: t = detail::as_float(a) < detail::as_float(b) ||
                               detail::as_float(a) > detail::as_float(b); break;   // lg_f32
                case 0x16: t = detail::as_float(a) >= detail::as_float(b); break;   // ge_f32
                case 0x1d: t = !(detail::as_float(a) == detail::as_float(b)); break; // neq_f32
                default: break;
                }

                mask |= (t ? 1u : 0u) << lane;
                continue;
            }

            uint32_t d = 0;

            switch (op) {
            case 0x101: d = ((c >> lane) & 1u) ? b : a; break;                 // cndmask_e64
            case 0x20b: d = (a & 0xffffffu) * (b & 0xffffffu) + c; break;       // mad_u32_u24
            case 0x210: { const uint32_t width = c & 0x1f; const uint32_t off = b & 0x1f;
                          d = width == 0 ? 0u : (a >> off) & ((width >= 32 ? 0xffffffffu : (1u << width) - 1u)); break; } // bfe_u32
            case 0x211: { const uint32_t width = c & 0x1f; const uint32_t off = b & 0x1f;
                          if (width == 0) { d = 0; } else { uint32_t f = (a >> off) & ((width >= 32 ? 0xffffffffu : (1u << width) - 1u));
                          const uint32_t sign = 1u << (width - 1u); d = (f ^ sign) - sign; } break; } // bfe_i32
            case 0x212: d = (a & b) | (~a & c); break;                          // bfi_b32
            case 0x256: d = (a << (b & 31u)) | c; break;                        // lshl_or_b32
            case 0x257: d = (a & b) | c; break;                                 // and_or_b32
            case 0x32c: d = a * b; break;                                       // mul_lo_u32
            case 0x32d: d = detail::mulhi(a, b); break;                         // mul_hi_u32
            case 0x31f: {                                                      // mbcnt_lo_u32_b32
                uint32_t below = a & ((1u << lane) - 1u);
                uint32_t bits = 0;

                for (; below != 0; below &= below - 1u) {
                    ++bits;
                }

                d = bits + b;
                break;
            }
            default: d = 0; break;
            }

            w.vgpr[vdst][lane] = d;
        }

        if (is_compare) {
            detail::write_scalar(w, sdst, mask);
        }

        pc = next;
    }
}


// ===========================================================================
// Graphics draw model (2026-10-23). Runs a generated NGG vertex program and pixel
// program over a point, line or triangle list:
//   GE/SPI: subgroups of at most 32 unique vertices and 32 primitives, one wave
//     each, with merged_wave_info (s3), the attribute-ring wave offset (s5, in
//     512-byte units), the user SGPRs from s8, the 16-bit subgroup-local vertex
//     offsets in v0/v1 and the vertex id in v5;
//   NGG checks: one GS_ALLOC_REQ (message 9) whose m0 equals vertices |
//     primitives << 12 before any export, a done primitive export per primitive
//     lane decoding back to the subgroup's indices, a done pos0 export per vertex
//     lane, and the attribute slots read back from the ring at wave base +
//     slot * 512 + lane * 16;
//   PA/SC: the gl_raster viewport transform, face/cull test, coverage rules
//     (triangle edge functions, DDA lines, square points) and perspective-correct
//     barycentrics (i, j) at pixel centres, so coverage matches the oracle;
//     hardware SC uses fixed-point snapping and top-left/diamond rules instead;
//   SPI/PS: 2x2 quads of one primitive, at most 8 per wave, EXEC = covered
//     pixels, v0/v1 = i/j, v2 = front face (all ones / zero), the user SGPRs and
//     the primitive mask after them, and the per-quad parameter LDS (P0, P10,
//     P20; flat slots carry the provoking vertex in P0 with zero deltas);
//   CB: the done MRT0 export's 32_ABGR colour, for lanes in its EXEC, packed to
//     BGRA8 with gl_raster's rounding; no blending, depth or stencil.
// Any protocol violation is counted in DrawStats::errors with the first message.
// ===========================================================================
namespace graphics {

struct Viewport final {
    int32_t x = 0, y = 0, width = 0, height = 0;
};

// One shaded fragment's exports (DrawSetup::outputs): MRT0/MRT1 32_ABGR colours,
// MRTZ depth (SPI_SHADER_32_R x) and stencil test value (UINT16_ABGR x[23:16]).
struct FragmentOutput final {
    int32_t x = 0, y = 0;               // GL window pixel, y up
    uint32_t targets = 0;               // bit 0: MRT0, bit 1: MRT1
    float color[2][4] = {};
    bool depth = false;
    float z = 0;
    bool stencil = false;
    uint32_t stencil_value = 0;
};

struct DrawSetup final {
    const uint8_t* vs_code = nullptr;
    uint32_t vs_dwords = 0;
    const uint8_t* ps_code = nullptr;
    uint32_t ps_dwords = 0;
    uint32_t vs_user[13] = {};          // SPI_SHADER_USER_DATA_GS_0.. (s8..s20)
    uint32_t vs_user_count = 13;
    uint32_t ps_user[32] = {};          // SPI_SHADER_USER_DATA_PS_0.. (s0..)
    uint32_t ps_user_count = 20;
    uint32_t vertices_per_primitive = 3;
    std::vector<uint32_t> indices;      // vertex ids, vertices_per_primitive per primitive
    uint32_t param_count = 1;           // SPI_VS_OUT_CONFIG.VS_EXPORT_COUNT + 1 = NUM_INTERP
    uint32_t flat_mask = 0;             // SPI_PS_INPUT_CNTL_n.FLAT_SHADE
    bool provoking_last = true;         // PA_SU_SC_MODE_CNTL.PROVOKING_VTX_LAST
    bool front_face_input = false;      // SPI_PS_INPUT_ENA.FRONT_FACE_ENA
    bool front_ccw = true;
    bool cull_front = false;
    bool cull_back = false;
    float point_size = 1;
    Viewport viewport{};
    uint64_t ring_base = 0;             // attribute ring address in memory
    uint32_t ring_bytes = 0;
    Memory* memory = nullptr;
    uint32_t width = 0, height = 0;
    uint32_t* framebuffer = nullptr;    // BGRA8, index y * width + x, y up (GL window)
    // SPI_PS_INPUT_ENA; 0 keeps the v3 layout (PERSP_CENTER, FRONT_FACE when
    // front_face_input). The model fills PERSP_CENTER, LINEAR_CENTER,
    // POS_X/Y_FLOAT (hardware rows top-down: height - 1 - y) and FRONT_FACE in
    // the SPI's bit order.
    uint32_t ps_input_ena = 0;
    uint32_t z_format = 0;              // SPI_SHADER_Z_FORMAT for MRTZ decoding
    // Scan-converted window (GL x, y up, max exclusive): the SC's scissor; the
    // default covers the framebuffer.
    int32_t clip_x0 = 0, clip_y0 = 0, clip_x1 = INT32_MAX, clip_y1 = INT32_MAX;
    // When set, every exported fragment is appended here instead of the
    // framebuffer write (the PM4 pipeline model's fixed-function back end).
    std::vector<FragmentOutput>* outputs = nullptr;
};

struct DrawStats final {
    uint32_t subgroups = 0;
    uint32_t alloc_requests = 0;
    uint32_t primitives = 0;
    uint32_t ps_waves = 0;
    uint32_t pixels_written = 0;
    uint32_t errors = 0;
    const char* first_error = nullptr;
};

namespace model {

constexpr uint32_t max_params = 8;

struct Vertex final {
    float clip[4] = {};
    float attr[max_params][4] = {};
};

struct Window final {
    float x = 0, y = 0, inv_w = 0;
};

inline void fail(DrawStats& stats, const char* message) {
    if (stats.first_error == nullptr) {
        stats.first_error = message;
    }
    ++stats.errors;
}

inline uint8_t to_byte(float value) {
    const float clamped = value < 0 ? 0 : (value > 1 ? 1 : value);
    return static_cast<uint8_t>(clamped * 255.0f + 0.5f);
}

inline bool to_window(const Vertex& vertex, const Viewport& viewport, Window& out) {
    const float w = vertex.clip[3];
    if (!(w > 0) && !(w < 0)) {
        return false;
    }
    out.inv_w = 1.0f / w;
    out.x = static_cast<float>(viewport.x) +
            (vertex.clip[0] * out.inv_w + 1.0f) * 0.5f * static_cast<float>(viewport.width);
    out.y = static_cast<float>(viewport.y) +
            (vertex.clip[1] * out.inv_w + 1.0f) * 0.5f * static_cast<float>(viewport.height);
    return std::isfinite(out.x) && std::isfinite(out.y);
}

// One fragment candidate of a primitive: perspective (i, j) and linear (li, lj)
// barycentrics.
struct Fragment final {
    int32_t x = 0, y = 0;
    float i = 0, j = 0;
    float li = 0, lj = 0;
};

// Pixel shading for one primitive: groups fragments into quads, runs the PS waves
// and writes the CB.
inline void shade(const DrawSetup& setup, DrawStats& stats, const Vertex* vertices, uint32_t count,
                  bool front, const std::vector<Fragment>& fragments, uint32_t primitive_number,
                  float (*helper)(const void*, int32_t, int32_t, float*), const void* helper_context) {
    // Quads in raster order of first appearance.
    struct Quad final {
        int32_t qx = 0, qy = 0;
        uint32_t live = 0;
        float i[4] = {}, j[4] = {}, li[4] = {}, lj[4] = {};
    };
    std::vector<Quad> quads;

    for (const Fragment& fragment : fragments) {
        const int32_t qx = fragment.x & ~1, qy = fragment.y & ~1;
        const uint32_t lane = static_cast<uint32_t>((fragment.x & 1) | ((fragment.y & 1) << 1));
        Quad* quad = nullptr;
        for (Quad& existing : quads) {
            if (existing.qx == qx && existing.qy == qy) {
                quad = &existing;
            }
        }
        if (quad == nullptr) {
            quads.push_back({});
            quad = &quads.back();
            quad->qx = qx;
            quad->qy = qy;
            for (uint32_t l = 0; l < 4; ++l) {
                float ij[4] = {0, 0, 0, 0};
                helper(helper_context, qx + static_cast<int32_t>(l & 1u),
                       qy + static_cast<int32_t>(l >> 1), ij);
                quad->i[l] = ij[0];
                quad->j[l] = ij[1];
                quad->li[l] = ij[2];
                quad->lj[l] = ij[3];
            }
        }
        quad->live |= 1u << lane;
        quad->i[lane] = fragment.i;
        quad->j[lane] = fragment.j;
        quad->li[lane] = fragment.li;
        quad->lj[lane] = fragment.lj;
    }

    // SPI input VGPRs in bit order (PERSP_CENTER 2, LINEAR_CENTER 2, POS_X_FLOAT,
    // POS_Y_FLOAT, FRONT_FACE).
    const uint32_t ena = setup.ps_input_ena != 0 ? setup.ps_input_ena
                                                 : (1u << 1) | (setup.front_face_input ? 1u << 12 : 0u);
    uint32_t next_vgpr = 0;
    const auto take = [&](uint32_t bit, uint32_t width) -> uint32_t {
        if ((ena & (1u << bit)) == 0) {
            return 0xffu;
        }
        const uint32_t first = next_vgpr;
        next_vgpr += width;
        return first;
    };
    const uint32_t vgpr_persp = take(1, 2);
    const uint32_t vgpr_linear = take(5, 2);
    const uint32_t vgpr_pos_x = take(8, 1);
    const uint32_t vgpr_pos_y = take(9, 1);
    const uint32_t vgpr_front = take(12, 1);
    if ((ena & ~((1u << 1) | (1u << 5) | (1u << 8) | (1u << 9) | (1u << 12))) != 0) {
        fail(stats, "SPI_PS_INPUT_ENA enables an input the model does not provide");
    }

    const uint32_t provoking = setup.provoking_last ? count - 1 : 0;

    for (size_t first = 0; first < quads.size(); first += 8) {
        const uint32_t wave_quads = static_cast<uint32_t>(quads.size() - first < 8 ? quads.size() - first : 8);
        std::vector<Wave> holder(1);
        Wave& w = holder[0];
        ParamLds lds{};
        lds.prim_mask = 0x8000u | (primitive_number & 0x7fffu);
        lds.attributes = setup.param_count;
        lds.data.assign(static_cast<size_t>(wave_quads) * setup.param_count * 4u * 3u, 0u);

        for (uint32_t q = 0; q < wave_quads; ++q) {
            for (uint32_t p = 0; p < setup.param_count; ++p) {
                const bool flat = ((setup.flat_mask >> p) & 1u) != 0;
                for (uint32_t c = 0; c < 4; ++c) {
                    float p0 = flat ? vertices[provoking].attr[p][c] : vertices[0].attr[p][c];
                    float p10 = (!flat && count > 1) ? vertices[1].attr[p][c] - vertices[0].attr[p][c] : 0.0f;
                    float p20 = (!flat && count > 2) ? vertices[2].attr[p][c] - vertices[0].attr[p][c] : 0.0f;
                    const size_t index = ((static_cast<size_t>(q) * setup.param_count + p) * 4u + c) * 3u;
                    lds.data[index] = detail::as_bits(p0);
                    lds.data[index + 1] = detail::as_bits(p10);
                    lds.data[index + 2] = detail::as_bits(p20);
                }
            }
        }

        uint32_t live = 0;

        for (uint32_t q = 0; q < wave_quads; ++q) {
            const Quad& quad = quads[first + q];
            live |= quad.live << (q * 4);
            for (uint32_t l = 0; l < 4; ++l) {
                const uint32_t lane = q * 4 + l;
                const int32_t px = quad.qx + static_cast<int32_t>(l & 1u);
                const int32_t py = quad.qy + static_cast<int32_t>(l >> 1);
                if (vgpr_persp != 0xffu) {
                    w.vgpr[vgpr_persp][lane] = detail::as_bits(quad.i[l]);
                    w.vgpr[vgpr_persp + 1][lane] = detail::as_bits(quad.j[l]);
                }
                if (vgpr_linear != 0xffu) {
                    w.vgpr[vgpr_linear][lane] = detail::as_bits(quad.li[l]);
                    w.vgpr[vgpr_linear + 1][lane] = detail::as_bits(quad.lj[l]);
                }
                if (vgpr_pos_x != 0xffu) {
                    w.vgpr[vgpr_pos_x][lane] = detail::as_bits(static_cast<float>(px) + 0.5f);
                }
                if (vgpr_pos_y != 0xffu) {
                    w.vgpr[vgpr_pos_y][lane] =
                        detail::as_bits(static_cast<float>(static_cast<int32_t>(setup.height) - 1 - py) + 0.5f);
                }
                if (vgpr_front != 0xffu) {
                    w.vgpr[vgpr_front][lane] = front ? 0xffffffffu : 0u;
                }
            }
        }

        for (uint32_t k = 0; k < setup.ps_user_count; ++k) {
            w.sgpr[k] = setup.ps_user[k];
        }

        w.sgpr[setup.ps_user_count] = lds.prim_mask;
        w.lds = &lds;
        w.exec = live;
        run(setup.ps_code, setup.ps_dwords, w, setup.memory);
        ++stats.ps_waves;

        if (w.unsupported != 0) {
            fail(stats, "pixel program used an encoding or descriptor the model rejects");
        }

        if (setup.outputs != nullptr) {
            // MRT0, MRT1 and MRTZ; the done export's EXEC names the fragments.
            const ExportRecord* target[3] = {nullptr, nullptr, nullptr};
            const ExportRecord* done = nullptr;
            for (uint32_t e = 0; e < w.export_count; ++e) {
                const ExportRecord& record = w.exports[e];
                const int slot = record.target == 0 ? 0 : record.target == 1 ? 1 : record.target == 8 ? 2 : -1;
                if (slot < 0) {
                    fail(stats, "pixel wave exported to a target the model does not decode");
                    continue;
                }
                if ((record.exec & ~live) != 0) {
                    fail(stats, "pixel export EXEC includes helper lanes");
                }
                target[slot] = &record;
                done = record.done ? &record : done;
            }
            if (done == nullptr) {
                fail(stats, "pixel wave ended without a done export");
                continue;
            }
            for (uint32_t q = 0; q < wave_quads; ++q) {
                for (uint32_t l = 0; l < 4; ++l) {
                    const uint32_t lane = q * 4 + l;
                    if (!((done->exec >> lane) & 1u) || done->enable == 0) {
                        continue;
                    }
                    FragmentOutput out{};
                    out.x = quads[first + q].qx + static_cast<int32_t>(l & 1u);
                    out.y = quads[first + q].qy + static_cast<int32_t>(l >> 1);
                    if (out.x < 0 || out.y < 0 || out.x >= static_cast<int32_t>(setup.width) ||
                        out.y >= static_cast<int32_t>(setup.height)) {
                        continue;
                    }
                    for (uint32_t m = 0; m < 2; ++m) {
                        const ExportRecord* c = target[m];
                        if (c == nullptr || c->enable == 0 || !((c->exec >> lane) & 1u)) {
                            continue;
                        }
                        if (c->enable != 0xfu) {
                            fail(stats, "MRT export is not four channels (32_ABGR)");
                        }
                        out.targets |= 1u << m;
                        for (uint32_t ch = 0; ch < 4; ++ch) {
                            out.color[m][ch] = detail::as_float(c->data[ch][lane]);
                        }
                    }
                    const ExportRecord* z = target[2];
                    if (z != nullptr && z->enable != 0 && ((z->exec >> lane) & 1u)) {
                        if (setup.z_format == 1) {                  // SPI_SHADER_32_R
                            out.depth = true;
                            out.z = detail::as_float(z->data[0][lane]);
                        } else if (setup.z_format == 7) {           // UINT16_ABGR: x[23:16]
                            out.stencil = true;
                            out.stencil_value = (z->data[0][lane] >> 16) & 0xffu;
                        } else {
                            fail(stats, "MRTZ export with an SPI_SHADER_Z_FORMAT the model does not decode");
                        }
                    }
                    setup.outputs->push_back(out);
                    ++stats.pixels_written;
                }
            }
            continue;
        }

        const ExportRecord* color = nullptr;

        for (uint32_t e = 0; e < w.export_count; ++e) {
            if (w.exports[e].target == 0 && w.exports[e].done) {
                color = &w.exports[e];
            }
        }

        if (color == nullptr) {
            fail(stats, "pixel wave ended without a done MRT0 export");
            continue;
        }

        if ((color->exec & ~live) != 0) {
            fail(stats, "MRT0 export EXEC includes helper lanes");
        }

        if (color->enable == 0) {
            continue;                               // every pixel killed
        }

        if (color->enable != 0xfu) {
            fail(stats, "MRT0 export is not four channels (32_ABGR)");
        }

        for (uint32_t q = 0; q < wave_quads; ++q) {
            for (uint32_t l = 0; l < 4; ++l) {
                const uint32_t lane = q * 4 + l;
                if (!((color->exec >> lane) & 1u)) {
                    continue;
                }
                const int32_t x = quads[first + q].qx + static_cast<int32_t>(l & 1u);
                const int32_t y = quads[first + q].qy + static_cast<int32_t>(l >> 1);
                if (x < 0 || y < 0 || x >= static_cast<int32_t>(setup.width) ||
                    y >= static_cast<int32_t>(setup.height)) {
                    continue;
                }
                const uint32_t packed = (uint32_t{to_byte(detail::as_float(color->data[3][lane]))} << 24) |
                                        (uint32_t{to_byte(detail::as_float(color->data[0][lane]))} << 16) |
                                        (uint32_t{to_byte(detail::as_float(color->data[1][lane]))} << 8) |
                                        uint32_t{to_byte(detail::as_float(color->data[2][lane]))};
                setup.framebuffer[static_cast<size_t>(y) * setup.width + x] = packed;
                ++stats.pixels_written;
            }
        }
    }
}

struct TriangleContext final {
    Window a, b, c;
    float inv_area;
};

// Perspective-correct (i, j) of the pixel centre (x, y) for the triangle.
inline float triangle_ij(const void* context, int32_t x, int32_t y, float* ij) {
    const TriangleContext& t = *static_cast<const TriangleContext*>(context);
    const float px = static_cast<float>(x) + 0.5f;
    const float py = static_cast<float>(y) + 0.5f;
    const float w0 = ((t.b.x - px) * (t.c.y - py) - (t.b.y - py) * (t.c.x - px)) * t.inv_area;
    const float w1 = ((t.c.x - px) * (t.a.y - py) - (t.c.y - py) * (t.a.x - px)) * t.inv_area;
    const float w2 = 1.0f - w0 - w1;
    const float denom = w0 * t.a.inv_w + w1 * t.b.inv_w + w2 * t.c.inv_w;
    ij[0] = denom != 0 ? (w1 * t.b.inv_w) / denom : 0.0f;
    ij[1] = denom != 0 ? (w2 * t.c.inv_w) / denom : 0.0f;
    ij[2] = w1;
    ij[3] = w2;
    return w2;
}

inline float zero_ij(const void*, int32_t, int32_t, float* ij) {
    ij[0] = 0;
    ij[1] = 0;
    ij[2] = 0;
    ij[3] = 0;
    return 0;
}

inline void raster_primitive(const DrawSetup& setup, DrawStats& stats, const Vertex* v, uint32_t count,
                             uint32_t number) {
    Window win[3];

    for (uint32_t k = 0; k < count; ++k) {
        if (!to_window(v[k], setup.viewport, win[k])) {
            return;
        }
    }

    std::vector<Fragment> fragments;
    const int32_t width = static_cast<int32_t>(setup.width) < setup.clip_x1 ? static_cast<int32_t>(setup.width)
                                                                             : setup.clip_x1;
    const int32_t height = static_cast<int32_t>(setup.height) < setup.clip_y1 ? static_cast<int32_t>(setup.height)
                                                                               : setup.clip_y1;
    const int32_t first_x = setup.clip_x0 > 0 ? setup.clip_x0 : 0;
    const int32_t first_y = setup.clip_y0 > 0 ? setup.clip_y0 : 0;

    if (count == 3) {
        const float area = (win[1].x - win[0].x) * (win[2].y - win[0].y) -
                           (win[1].y - win[0].y) * (win[2].x - win[0].x);
        // Window y grows upward here, so a counter-clockwise triangle has a
        // positive signed area (the PA FACE convention for GL).
        const bool front = setup.front_ccw ? (area > 0) : (area < 0);

        if ((front && setup.cull_front) || (!front && setup.cull_back) || area == 0 || !std::isfinite(area)) {
            return;
        }

        TriangleContext t{win[0], win[1], win[2], 1.0f / area};
        const float min_xf = std::fmin(std::fmin(win[0].x, win[1].x), win[2].x);
        const float max_xf = std::fmax(std::fmax(win[0].x, win[1].x), win[2].x);
        const float min_yf = std::fmin(std::fmin(win[0].y, win[1].y), win[2].y);
        const float max_yf = std::fmax(std::fmax(win[0].y, win[1].y), win[2].y);
        int32_t min_x = static_cast<int32_t>(std::floor(min_xf));
        int32_t max_x = static_cast<int32_t>(std::ceil(max_xf));
        int32_t min_y = static_cast<int32_t>(std::floor(min_yf));
        int32_t max_y = static_cast<int32_t>(std::ceil(max_yf));
        min_x = min_x < first_x ? first_x : min_x;
        min_y = min_y < first_y ? first_y : min_y;
        max_x = max_x > width ? width : max_x;
        max_y = max_y > height ? height : max_y;

        for (int32_t y = min_y; y < max_y; ++y) {
            for (int32_t x = min_x; x < max_x; ++x) {
                const float px = static_cast<float>(x) + 0.5f;
                const float py = static_cast<float>(y) + 0.5f;
                const float w0 = ((t.b.x - px) * (t.c.y - py) - (t.b.y - py) * (t.c.x - px)) * t.inv_area;
                const float w1 = ((t.c.x - px) * (t.a.y - py) - (t.c.y - py) * (t.a.x - px)) * t.inv_area;
                const float w2 = 1.0f - w0 - w1;
                if (w0 < 0 || w1 < 0 || w2 < 0) {
                    continue;
                }
                Fragment fragment{x, y, 0, 0, 0, 0};
                float ij[4];
                triangle_ij(&t, x, y, ij);
                fragment.i = ij[0];
                fragment.j = ij[1];
                fragment.li = ij[2];
                fragment.lj = ij[3];
                fragments.push_back(fragment);
            }
        }

        shade(setup, stats, v, count, front, fragments, number, triangle_ij, &t);
        return;
    }

    if (count == 2) {
        const float dx = win[1].x - win[0].x;
        const float dy = win[1].y - win[0].y;
        const float adx = std::fabs(dx), ady = std::fabs(dy);
        const int32_t steps = static_cast<int32_t>((adx > ady ? adx : ady) + 1.0f);

        for (int32_t step = 0; steps > 0 && step <= steps; ++step) {
            const float t = static_cast<float>(step) / static_cast<float>(steps);
            const int32_t x = static_cast<int32_t>(std::floor(win[0].x + dx * t));
            const int32_t y = static_cast<int32_t>(std::floor(win[0].y + dy * t));
            if (x < first_x || y < first_y || x >= width || y >= height) {
                continue;
            }
            // Perspective-correct position along the line as the line barycentric i.
            const float a0 = win[0].inv_w * (1 - t);
            const float a1 = win[1].inv_w * t;
            const float denom = a0 + a1;
            // LINEAR_CENTER carries the DDA position t itself.
            fragments.push_back({x, y, denom != 0 ? a1 / denom : 0.0f, 0.0f, t, 0.0f});
        }

        shade(setup, stats, v, count, true, fragments, number, zero_ij, nullptr);
        return;
    }

    const int32_t half = static_cast<int32_t>(setup.point_size * 0.5f);
    const int32_t cx = static_cast<int32_t>(std::floor(win[0].x));
    const int32_t cy = static_cast<int32_t>(std::floor(win[0].y));

    for (int32_t y = cy - half; y <= cy + half; ++y) {
        for (int32_t x = cx - half; x <= cx + half; ++x) {
            if (x >= first_x && y >= first_y && x < width && y < height) {
                fragments.push_back({x, y, 0.0f, 0.0f, 0.0f, 0.0f});
            }
        }
    }

    shade(setup, stats, v, count, true, fragments, number, zero_ij, nullptr);
}

} // namespace model

// Runs the whole draw. Primitives keep submission order through subgroups.
inline DrawStats run_draw(const DrawSetup& setup) {
    DrawStats stats{};
    const uint32_t vpp = setup.vertices_per_primitive;

    if (vpp < 1 || vpp > 3 || setup.param_count == 0 || setup.param_count > model::max_params ||
        setup.indices.size() % vpp != 0 || setup.framebuffer == nullptr || setup.ring_bytes == 0) {
        model::fail(stats, "invalid draw setup");
        return stats;
    }

    const uint32_t ring_slots = setup.ring_bytes / (setup.param_count * 512u);
    const uint32_t prim_total = static_cast<uint32_t>(setup.indices.size() / vpp);
    uint32_t next = 0;
    uint32_t number = 0;

    while (next < prim_total) {
        // GE: grow the subgroup while it fits 32 vertices and 32 primitives.
        std::vector<uint32_t> ids;
        std::vector<uint32_t> local;   // vpp per primitive
        uint32_t prims = 0;

        while (next < prim_total && prims < 32) {
            uint32_t added = 0;
            for (uint32_t k = 0; k < vpp; ++k) {
                const uint32_t id = setup.indices[next * vpp + k];
                bool seen = false;
                for (uint32_t existing : ids) { seen = seen || existing == id; }
                for (uint32_t m = 0; m < k; ++m) { seen = seen || setup.indices[next * vpp + m] == id; }
                added += seen ? 0u : 1u;
            }
            if (ids.size() + added > 32) {
                break;
            }
            for (uint32_t k = 0; k < vpp; ++k) {
                const uint32_t id = setup.indices[next * vpp + k];
                uint32_t slot = static_cast<uint32_t>(ids.size());
                for (uint32_t m = 0; m < ids.size(); ++m) {
                    if (ids[m] == id) { slot = m; }
                }
                if (slot == ids.size()) { ids.push_back(id); }
                local.push_back(slot);
            }
            ++prims;
            ++next;
        }

        const uint32_t verts = static_cast<uint32_t>(ids.size());
        const uint32_t ring_slot = (stats.subgroups % (ring_slots == 0 ? 1 : ring_slots)) * setup.param_count;
        std::vector<Wave> holder(1);
        Wave& w = holder[0];
        w.sgpr[3] = verts | (prims << 8) | (0u << 24) | (1u << 28);
        w.sgpr[5] = ring_slot;

        for (uint32_t k = 0; k < setup.vs_user_count; ++k) {
            w.sgpr[8 + k] = setup.vs_user[k];
        }

        const uint32_t threads = verts > prims ? verts : prims;
        w.exec = threads >= 32 ? 0xffffffffu : (1u << threads) - 1u;

        for (uint32_t lane = 0; lane < prims; ++lane) {
            const uint32_t i0 = local[lane * vpp];
            const uint32_t i1 = vpp > 1 ? local[lane * vpp + 1] : 0u;
            const uint32_t i2 = vpp > 2 ? local[lane * vpp + 2] : 0u;
            w.vgpr[0][lane] = i0 | (i1 << 16);
            w.vgpr[1][lane] = i2;
        }

        for (uint32_t lane = 0; lane < verts; ++lane) {
            w.vgpr[5][lane] = ids[lane];
        }

        run(setup.vs_code, setup.vs_dwords, w, setup.memory);
        ++stats.subgroups;

        if (w.unsupported != 0) {
            model::fail(stats, "vertex program used an encoding the model rejects");
        }

        // GS_ALLOC_REQ before every export, with the subgroup counts.
        uint32_t alloc_sequence = 0xffffffffu;
        for (uint32_t m = 0; m < w.message_count; ++m) {
            if (w.messages[m].id == 9) {
                ++stats.alloc_requests;
                alloc_sequence = w.messages[m].sequence;
                if (w.messages[m].m0 != (verts | (prims << 12))) {
                    model::fail(stats, "GS_ALLOC_REQ m0 payload does not match the subgroup counts");
                }
            }
        }

        if (alloc_sequence == 0xffffffffu) {
            model::fail(stats, "no GS_ALLOC_REQ");
        }

        const ExportRecord* prim = nullptr;
        const ExportRecord* pos = nullptr;

        for (uint32_t e = 0; e < w.export_count; ++e) {
            const ExportRecord& record = w.exports[e];
            if (record.sequence < alloc_sequence) {
                model::fail(stats, "export before GS_ALLOC_REQ");
            }
            if (record.target == 20) { prim = &record; }
            if (record.target == 12) { pos = &record; }
        }

        const uint32_t prim_lanes = prims >= 32 ? 0xffffffffu : (1u << prims) - 1u;
        const uint32_t vert_lanes = verts >= 32 ? 0xffffffffu : (1u << verts) - 1u;

        if (prim == nullptr || !prim->done || (prim->exec & prim_lanes) != prim_lanes || prim->enable != 1u) {
            model::fail(stats, "missing or incomplete primitive export");
            continue;
        }

        if (pos == nullptr || !pos->done || (pos->exec & vert_lanes) != vert_lanes || pos->enable != 0xfu) {
            model::fail(stats, "missing or incomplete pos0 export");
            continue;
        }

        std::vector<model::Vertex> vertices(verts);

        for (uint32_t lane = 0; lane < verts; ++lane) {
            for (uint32_t c = 0; c < 4; ++c) {
                vertices[lane].clip[c] = detail::as_float(pos->data[c][lane]);
            }
            for (uint32_t p = 0; p < setup.param_count; ++p) {
                const uint64_t address = setup.ring_base + uint64_t{ring_slot} * 512u + p * 512u + lane * 16u;
                for (uint32_t c = 0; c < 4; ++c) {
                    bool ok = true;
                    vertices[lane].attr[p][c] = detail::as_float(detail::read_u32(setup.memory, address + c * 4u, ok));
                    if (!ok) {
                        model::fail(stats, "attribute ring read outside the ring");
                    }
                }
            }
        }

        for (uint32_t lane = 0; lane < prims; ++lane) {
            const uint32_t word = prim->data[0][lane];
            if ((word >> 31) != 0) {
                continue;                                    // null primitive
            }
            model::Vertex pv[3];
            for (uint32_t k = 0; k < vpp; ++k) {
                const uint32_t index = (word >> (10 * k)) & 0x1ffu;
                if (index != local[lane * vpp + k] || index >= verts) {
                    model::fail(stats, "primitive export vertex index mismatch");
                }
                pv[k] = vertices[index < verts ? index : 0];
            }
            ++stats.primitives;
            model::raster_primitive(setup, stats, pv, vpp, number++);
        }
    }

    return stats;
}

} // namespace graphics

} // namespace gfx11_interp

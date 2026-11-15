/*
Module: rx7900.tests.gfx11_isa_model
Purpose: Independent decoder and wave32 interpreter for the GFX11 instruction subset
of src/hardware/gfx11_isa, plus a synthetic GPU memory with raw-buffer (V#)
bounds semantics, used to execute the generated raster2d kernels on host.
Created: 2026-10-22
Architecture: Test-only. Decodes the binary dwords (not the encoder's inputs), so
field placement errors surface as wrong behaviour. Enforces the kernel discipline
the hardware needs and the tests cannot otherwise observe: s_waitcnt before use
of loaded registers, the constant-bus limit, no read of unwritten SGPRs, no VALU
after a SALU EXEC write, no SALU write of an SGPR a VALU used as a lane mask
(GFX11 VALU mask-write hazard), raw V# descriptors only, s_waitcnt_vscnt before
s_endpgm and a bounded instruction count. Semantics per AMDGPUAsmGFX11.rst.
*/
#pragma once

#include <cstdint>
#include <cstdio>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace gfx11_model {

struct Region final {
    uint64_t base = 0;
    std::vector<uint8_t> bytes;
};

class Memory final {
public:
    Region& add(uint64_t base, uint64_t size) {
        regions_.push_back(Region{base, std::vector<uint8_t>(static_cast<size_t>(size), 0)});
        return regions_.back();
    }

    Region* find(uint64_t address, uint64_t size) {
        for (Region& region : regions_) {
            if (address >= region.base && address - region.base <= region.bytes.size() &&
                size <= region.bytes.size() - (address - region.base)) {
                return &region;
            }
        }

        return nullptr;
    }

    bool read(uint64_t address, uint32_t size, uint32_t& value) {
        Region* region = find(address, size);

        if (region == nullptr) {
            return false;
        }

        value = 0;

        for (uint32_t i = 0; i < size; ++i) {
            value |= static_cast<uint32_t>(region->bytes[address - region->base + i]) << (8 * i);
        }

        return true;
    }

    bool write(uint64_t address, uint32_t size, uint32_t value) {
        Region* region = find(address, size);

        if (region == nullptr) {
            return false;
        }

        for (uint32_t i = 0; i < size; ++i) {
            region->bytes[address - region->base + i] = static_cast<uint8_t>(value >> (8 * i));
        }

        return true;
    }

    std::vector<Region>& regions() { return regions_; }

private:
    std::vector<Region> regions_;
};

enum class Fmt { sop1, sop2, sopk, sopc, sopp, smem, vop1, vop2, vopc, vop3, mubuf, unknown };

struct Decoded final {
    Fmt format = Fmt::unknown;
    uint32_t op = 0;
    uint32_t words = 1;   // including literal
    uint32_t w0 = 0;
    uint32_t w1 = 0;
    uint32_t literal = 0;
    bool has_literal = false;
};

inline const char* format_name(Fmt f) {
    switch (f) {
    case Fmt::sop1: return "sop1";
    case Fmt::sop2: return "sop2";
    case Fmt::sopk: return "sopk";
    case Fmt::sopc: return "sopc";
    case Fmt::sopp: return "sopp";
    case Fmt::smem: return "smem";
    case Fmt::vop1: return "vop1";
    case Fmt::vop2: return "vop2";
    case Fmt::vopc: return "vopc";
    case Fmt::vop3: return "vop3";
    case Fmt::mubuf: return "mubuf";
    default: return "unknown";
    }
}

// words: at least 3 dwords available from the instruction start (zero padded).
inline Decoded decode(const uint32_t* words) {
    Decoded d{};
    const uint32_t w = words[0];
    d.w0 = w;
    auto literal_if = [&](uint32_t code, uint32_t index) {
        if (code == 255 && !d.has_literal) {
            d.has_literal = true;
            d.literal = words[index];
            d.words = index + 1;
        }
    };

    if ((w >> 23) == 0x17d) {
        d.format = Fmt::sop1;
        d.op = (w >> 8) & 0xff;
        literal_if(w & 0xff, 1);
    } else if ((w >> 23) == 0x17e) {
        d.format = Fmt::sopc;
        d.op = (w >> 16) & 0x7f;
        literal_if(w & 0xff, 1);
        literal_if((w >> 8) & 0xff, 1);
    } else if ((w >> 23) == 0x17f) {
        d.format = Fmt::sopp;
        d.op = (w >> 16) & 0x7f;
    } else if ((w >> 28) == 0xb) {
        d.format = Fmt::sopk;
        d.op = (w >> 23) & 0x1f;
    } else if ((w >> 30) == 2) {
        d.format = Fmt::sop2;
        d.op = (w >> 23) & 0x7f;
        literal_if(w & 0xff, 1);
        literal_if((w >> 8) & 0xff, 1);
    } else if ((w >> 26) == 0x3d) {
        d.format = Fmt::smem;
        d.op = (w >> 18) & 0xff;
        d.w1 = words[1];
        d.words = 2;
    } else if ((w >> 26) == 0x35) {
        d.format = Fmt::vop3;
        d.op = (w >> 16) & 0x3ff;
        d.w1 = words[1];
        d.words = 2;
        literal_if(d.w1 & 0x1ff, 2);
        literal_if((d.w1 >> 9) & 0x1ff, 2);
        literal_if((d.w1 >> 18) & 0x1ff, 2);
    } else if ((w >> 26) == 0x38) {
        d.format = Fmt::mubuf;
        d.op = (w >> 18) & 0xff;
        d.w1 = words[1];
        d.words = 2;
    } else if ((w >> 25) == 0x3f) {
        d.format = Fmt::vop1;
        d.op = (w >> 9) & 0xff;
        literal_if(w & 0x1ff, 1);
    } else if ((w >> 25) == 0x3e) {
        d.format = Fmt::vopc;
        d.op = (w >> 17) & 0xff;
        literal_if(w & 0x1ff, 1);
    } else if ((w >> 31) == 0) {
        d.format = Fmt::vop2;
        d.op = (w >> 25) & 0x3f;
        literal_if(w & 0x1ff, 1);
    }

    return d;
}

struct Dispatch final {
    uint64_t entry = 0;
    uint32_t user_data[2] = {0, 0};
    uint32_t groups_x = 1;
    uint32_t groups_y = 1;
    uint32_t vgprs = 256;
    // Fill v0 bits [31:10] with nonzero noise (GFX11 packed Y/Z ids must be masked).
    bool packed_id_noise = true;
};

class Wave final {
public:
    Wave(Memory& memory, std::set<std::pair<int, uint32_t>>& coverage)
        : memory_(memory), coverage_(coverage) {}

    std::string error;
    uint64_t executed = 0;

    bool run(const Dispatch& dispatch, uint32_t group_x, uint32_t group_y, uint32_t wave) {
        for (uint32_t i = 0; i < 128; ++i) {
            sgpr_[i] = 0;
            written_[i] = false;
            mask_read_[i] = false;
            l
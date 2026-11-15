/*
Module: rx7900.tests.gfx11_header_db
Purpose: Load the pinned AMD GC 11.0.0 register headers and Mesa 24.2.0 register
JSON at test time, verify their SHA-256 against work/upstream-gpuinit/SHA256SUMS,
and answer register/field/enumerant queries by name or absolute dword.
Created: 2026-10-23
Architecture: Test-only and independent of src/opengl/gfx11_registers.hpp: it
parses the upstream text itself, so a transcription error in the catalog, an
emitted value with bits outside every field, or a wrong enumerant fails a test.
The directory comes from the RX7900_UPSTREAM_GPUINIT_DIR compile definition.
*/
#pragma once

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace gfx11_header_db {

// ---- SHA-256 (FIPS 180-4) ----
inline std::string sha256(const std::string& data) {
    static const uint32_t k[64] = {
        0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
        0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
        0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
        0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
        0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
        0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
        0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
        0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};
    uint32_t h[8] = {0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
                     0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
    std::string m = data;
    const uint64_t bit_length = static_cast<uint64_t>(data.size()) * 8;
    m.push_back(static_cast<char>(0x80));
    while (m.size() % 64 != 56) m.push_back(0);
    for (int i = 7; i >= 0; --i) m.push_back(static_cast<char>(bit_length >> (i * 8)));
    const auto rotr = [](uint32_t x, uint32_t n) { return (x >> n) | (x << (32 - n)); };
    for (size_t block = 0; block < m.size(); block += 64) {
        uint32_t w[64];
        for (int i = 0; i < 16; ++i) {
            w[i] = (static_cast<uint32_t>(static_cast<uint8_t>(m[block + i * 4])) << 24) |
                   (static_cast<uint32_t>(static_cast<uint8_t>(m[block + i * 4 + 1])) << 16) |
                   (static_cast<uint32_t>(static_cast<uint8_t>(m[block + i * 4 + 2])) << 8) |
                   static_cast<uint32_t>(static_cast<uint8_t>(m[block + i * 4 + 3]));
        }
        for (int i = 16; i < 64; ++i) {
            const uint32_t s0 = rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
            const uint32_t s1 = rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
            w[i] = w[i - 16] + s0 + w[i - 7] + s1;
        }
        uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4], f = h[5], g = h[6], hh = h[7];
        for (int i = 0; i < 64; ++i) {
            const uint32_t t1 = hh + (rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25)) + ((e & f) ^ (~e & g)) + k[i] + w[i];
            const uint32_t t2 = (rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22)) + ((a & b) ^ (a & c) ^ (b & c));
            hh = g; g = f; f = e; e = d + t1; d = c; c = b; b = a; a = t1 + t2;
        }
        h[0] += a; h[1] += b; h[2] += c; h[3] += d; h[4] += e; h[5] += f; h[6] += g; h[7] += hh;
    }
    char out[65];
    for (int i = 0; i < 8; ++i) std::snprintf(out + i * 8, 9, "%08x", h[i]);
    return std::string(out, 64);
}

inline std::string directory() {
#ifdef RX7900_UPSTREAM_GPUINIT_DIR
    return RX7900_UPSTREAM_GPUINIT_DIR;
#else
    return "";
#endif
}

inline bool read_file(const std::string& path, std::string& out) {
    std::ifstream file(path, std::ios::binary);
    if (!file) return false;
    std::ostringstream text;
    text << file.rdbuf();
    out = text.str();
    return true;
}

// True when relative (as listed in SHA256SUMS) hashes to its pinned value.
inline bool pinned(const std::string& relative, std::string& content) {
    std::string sums;
    if (!read_file(directory() + "/SHA256SUMS", sums) || !read_file(directory() + "/" + relative, content)) {
        return false;
    }
    std::istringstream lines(sums);
    std::string line;
    while (std::getline(lines, line)) {
        if (line.size() > 66 && line.substr(66) == relative) return line.substr(0, 64) == sha256(content);
    }
    return false;
}

struct Field final {
    std::string name;
    uint32_t shift = 0;
    uint32_t mask = 0;
};

struct Register final {
    std::string name;
    uint32_t offset = 0;
    uint32_t base_index = 0;
    std::vector<Field> fields;
    uint32_t absolute() const { return offset + (base_index == 0 ? 0x1260u : 0xa000u); }
    uint32_t mask_union() const {
        uint32_t m = 0;
        for (const auto& f : fields) m |= f.mask;
        return m;
    }
};

class Database final {
public:
    bool load() {
        std::string offsets, masks;
        offsets_pinned_ = pinned("gc_11_0_0_offset.h", offsets);
        masks_pinned_ = pinned("gc_11_0_0_sh_mask.h", masks);
        json_pinned_ = pinned("mesa-24.2.0/gfx11.json", json_);
        pkt3_pinned_ = pinned("mesa-24.2.0/pkt3.json", pkt3_);
        if (offsets.empty() || masks.empty()) return false;
        std::istringstream o(offsets);
        std::string line;
        while (std::getline(o, line)) {
            std::string name;
            uint32_t value = 0;
            if (!define(line, name, value) || name.rfind("reg", 0) != 0) continue;
            name = name.substr(3);
            const std::string suffix = "_BASE_IDX";
            if (name.size() > suffix.size() && name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0) {
                registers_[name.substr(0, name.size() - suffix.size())].base_index = value;
            } else {
                registers_[name].name = name;
                registers_[name].offset = value;
            }
        }
        std::map<std::string, uint32_t> shifts;
        std::istringstream mk(masks);
        while (std::getline(mk, line)) {
            std::string name;
            uint32_t value = 0;
            if (!define(line, name, value)) continue;
            const auto split = name.find("__");
            if (split == std::string::npos) continue;
            const std::string reg = name.substr(0, split);
            std::string rest = name.substr(split + 2);
            if (rest.size() > 7 && rest.compare(rest.size() - 7, 7, "__SHIFT") == 0) {
                shifts[reg + "." + rest.substr(0, rest.size() - 7)] = value;
            } else if (rest.size() > 5 && rest.compare(rest.size() - 5, 5, "_MASK") == 0) {
                const std::string field = rest.substr(0, rest.size() - 5);
                auto found = registers_.find(reg);
                if (found != registers_.end()) {
                    found->second.fields.push_back({field, shifts[reg + "." + field], value});
                }
            }
        }
        for (const auto& entry : registers_) by_absolute_[entry.second.absolute()] = &entry.second;
        return true;
    }

    bool pinned_ok() const { return offsets_pinned_ && masks_pinned_ && json_pinned_ && pkt3_pinned_; }
    const Register* find(const std::string& name) const {
        const auto found = registers_.find(name);
        return found == registers_.end() ? nullptr : &found->second;
    }
    const Register* at(uint32_t absolute) const {
        const auto found = by_absolute_.find(absolute);
        return found == by_absolute_.end() ? nullptr : found->second;
    }
    const Field* field(const std::string& reg, const std::string& name) const {
        const Register* r = find(reg);
        if (r == nullptr) return nullptr;
        for (const auto& f : r->fields) {
            if (f.name == name) return &f;
        }
        return nullptr;
    }
    // Field value of a register word, by header names; -1 when unknown.
    int64_t get(const std::string& reg, const std::string& name, uint32_t word) const {
        const Field* f = field(reg, name);
        return f == nullptr ? -1 : static_cast<int64_t>((word & f->mask) >> f->shift);
    }

    // gfx11.json register_mappings: byte address of an "mm" register, or -1.
    int64_t json_address(const std::string& name) const {
        const std::string key = "\"name\": \"" + name + "\"";
        size_t position = 0;
        while ((position = json_.find(key, position)) != std::string::npos) {
            const size_t object = json_.rfind("\n  {\n", position);
            const size_t map = object == std::string::npos ? std::string::npos :
                               json_.find("\"map\": {\"at\": ", object);
            if (map != std::string::npos && map < position) {
                const size_t start = map + 14;
                const size_t end = json_.find('}', start);
                const std::string mapping = json_.substr(start, end - start);
                if (mapping.find("\"to\": \"mm\"") != std::string::npos) {
                    return std::stoll(mapping);
                }
            }
            position += key.size();
        }
        return -1;
    }

    // Enumerant value from gfx11.json (or pkt3.json when packet is true), or -1.
    int64_t enumerant(const std::string& type, const std::string& entry, bool packet = false) const {
        const std::string& text = packet ? pkt3_ : json_;
        const size_t start = text.find("\"" + type + "\": {");
        if (start == std::string::npos) return -1;
        const size_t end = text.find(']', start);
        const std::string key = "\"name\": \"" + entry + "\", \"value\": ";
        const size_t found = text.find(key, start);
        if (found == std::string::npos || found > end) return -1;
        return std::stoll(text.substr(found + key.size()));
    }

    // Field bit range from a JSON register type ("bits": [lo, hi]), or false.
    bool json_field(const std::string& type, const std::string& name, uint32_t& lo, uint32_t& hi,
                    bool packet = false) const {
        const std::string& text = packet ? pkt3_ : json_;
        const size_t start = text.find("\n  \"" + type + "\": {\n   \"fields\": [");
        if (start == std::string::npos) return false;
        const size_t end = text.find("]\n  }", start);
        const std::string key = "\"name\": \"" + name + "\"";
        size_t found = text.find(key, start);
        while (found != std::string::npos && found < end) {
            const size_t bits = text.rfind("\"bits\": [", found);
            if (bits != std::string::npos && bits > start) {
                std::sscanf(text.c_str() + bits, "\"bits\": [%u, %u]", &lo, &hi);
                return true;
            }
            found = text.find(key, found + 1);
        }
        return false;
    }

    const std::map<std::string, Register>& registers() const { return registers_; }

private:
    static bool define(const std::string& line, std::string& name, uint32_t& value) {
        char macro[256];
        char number[64];
        if (std::sscanf(line.c_str(), "#define %255s %63s", macro, number) != 2) return false;
        std::string digits = number;
        while (!digits.empty() && (digits.back() == 'L' || digits.back() == 'U')) digits.pop_back();
        if (digits.empty()) return false;
        char* end = nullptr;
        const unsigned long parsed = std::strtoul(digits.c_str(), &end, 0);
        if (end == nullptr || *end != '\0') return false;
        name = macro;
        value = static_cast<uint32_t>(parsed);
        return true;
    }

    std::map<std::string, Register> registers_;
    std::map<uint32_t, const Register*> by_absolute_;
    std::string json_;
    std::string pkt3_;
    bool offsets_pinned_ = false;
    bool masks_pinned_ = false;
    bool json_pinned_ = false;
    bool pkt3_pinned_ = false;
};

} // namespace gfx11_header_db

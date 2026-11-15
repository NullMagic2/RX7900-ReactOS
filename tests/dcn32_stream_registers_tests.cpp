/*
Module: rx7900.tests.dcn32_stream_registers
Purpose: Check every DCN 3.2 stream register constant against the pinned AMD headers.
Created: 2026-10-24
Architecture: Host test. It verifies the SHA-256 of the pinned dcn_3_2_0_offset.h
and dcn_3_2_0_sh_mask.h (work/upstream-dcn-stream/SHA256SUMS and the hashes
recorded in src/display/dcn32_stream_registers.hpp), parses both headers and the
catalog source, and checks for every catalog register: each instance header
exists with the recorded dword offset and one common BASE_IDX, the catalog field
list is exactly the header field list (name, shift, mask) of the first instance,
and every other instance has the same layout. Spot checks tie the compiled
constants to the parsed text. Layout facts the stream owner relies on (Linux
reuses SURF0/CUR0 masks for SURF1/CUR1, programs the OCSC as consecutive
registers, DOMAIN0-3/16-19 power gating) are checked explicitly.
*/
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <regex>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "asic_test_device.hpp"
#include "dcn32_stream_registers.hpp"

namespace {

namespace r = rx7900::display::dcn32_stream_reg;

rx7900_test::Checks checks;

void expect(bool condition, const char* contract) { checks.expect(condition, contract); }

std::string slurp(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    std::ostringstream text;
    text << in.rdbuf();
    return text.str();
}

// FIPS 180-4 SHA-256 (test-only, for the pin check).
std::string sha256(const std::string& data) {
    static const uint32_t k[64]{
        0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
        0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
        0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
        0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
        0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
        0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
        0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
        0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};
    uint32_t h[8]{0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
    std::string m = data;
    const uint64_t bits = static_cast<uint64_t>(data.size()) * 8;
    m.push_back(static_cast<char>(0x80));

    while (m.size() % 64 != 56) {
        m.push_back('\0');
    }

    for (int i = 7; i >= 0; --i) {
        m.push_back(static_cast<char>(bits >> (8 * i)));
    }

    const auto rotr = [](uint32_t x, int n) { return (x >> n) | (x << (32 - n)); };

    for (size_t chunk = 0; chunk < m.size(); chunk += 64) {
        uint32_t w[64];

        for (int i = 0; i < 16; ++i) {
            w[i] = (static_cast<uint32_t>(static_cast<uint8_t>(m[chunk + 4 * i])) << 24) |
                   (static_cast<uint32_t>(static_cast<uint8_t>(m[chunk + 4 * i + 1])) << 16) |
                   (static_cast<uint32_t>(static_cast<uint8_t>(m[chunk + 4 * i + 2])) << 8) |
                   static_cast<uint8_t>(m[chunk + 4 * i + 3]);
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
            hh = g;
            g = f;
            f = e;
            e = d + t1;
            d = c;
            c = b;
            b = a;
            a = t1 + t2;
        }

        h[0] += a, h[1] += b, h[2] += c, h[3] += d, h[4] += e, h[5] += f, h[6] += g, h[7] += hh;
    }

    char out[65];

    for (int i = 0; i < 8; ++i) {
        std::snprintf(out + 8 * i, 9, "%08x", h[i]);
    }

    return std::string(out, 64);
}

std::map<std::string, uint64_t> defines(const std::string& text) {
    std::map<std::string, uint64_t> table;
    const std::regex define(R"(#define\s+(\w+)\s+(0x[0-9a-fA-F]+|\d+)L?\s*$)");
    std::istringstream lines(text);
    std::string line;

    while (std::getline(lines, line)) {
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }

        std::smatch m;

        if (std::regex_search(line, m, define)) {
            table[m[1]] = std::stoull(m[2], nullptr, 0);
        }
    }

    return table;
}

struct FieldText final {
    std::string name;
    uint32_t shift = 0;
    uint32_t mask = 0;
    bool has_mask = true;
    bool operator<(const FieldText& other) const {
        return std::tie(shift, mask, name) < std::tie(other.shift, other.mask, other.name);
    }
};

struct RegisterText final {
    std::string name;
    std::vector<std::string> headers;
    uint32_t base = 0;
    uint32_t count = 0;
    std::vector<uint32_t> offsets;
    std::vector<FieldText> fields;
};

std::vector<RegisterText> parse_catalog(const std::string& text) {
    std::vector<RegisterText> registers;
    const std::regex open(R"(^namespace (\w+) \{$)");
    const std::regex headers(R"(^// headers: (.*)$)");
    const std::regex reg(R"(^constexpr Register reg\{(\d+), (\d+), \{([^}]*)\}\};$)");
    const std::regex field(R"(^constexpr Field (\w+)\{(\d+), (0x[0-9a-f]+)\};$)");
    std::istringstream lines(text);
    std::string line;
    RegisterText* current = nullptr;

    while (std::getline(lines, line)) {
        std::smatch m;

        if (std::regex_match(line, m, open) && m[1] != "rx7900") {
            registers.push_back({});
            current = &registers.back();
            current->name = m[1];
        } else if (current != nullptr && std::regex_match(line, m, headers)) {
            std::istringstream names(m[1].str());
            std::string name;

            while (names >> name) {
                current->headers.push_back(name);
            }
        } else if (current != nullptr && std::regex_match(line, m, reg)) {
            current->base = static_cast<uint32_t>(std::stoul(m[1]));
            current->count = static_cast<uint32_t>(std::stoul(m[2]));
            std::istringstream values(m[3].str());
            std::string value;

            while (std::getline(values, value, ',')) {
                current->offsets.push_back(static_cast<uint32_t>(std::stoul(value, nullptr, 0)));
            }
        } else if (current != nullptr && std::regex_match(line, m, field)) {
            current->fields.push_back({m[1], static_cast<uint32_t>(std::stoul(m[2])),
                                       static_cast<uint32_t>(std::stoul(m[3], nullptr, 0)), true});
        }
    }

    return registers;
}

std::vector<FieldText> header_fields(const std::map<std::string, uint64_t>& masks, const std::string& header) {
    std::vector<FieldText> fields;
    const std::string prefix = header + "__";

    for (auto it = masks.lower_bound(prefix); it != masks.end() && it->first.compare(0, prefix.size(), prefix) == 0;
         ++it) {
        const std::string& key = it->first;

        if (key.size() > prefix.size() + 7 && key.compare(key.size() - 7, 7, "__SHIFT") == 0) {
            const std::string name = key.substr(prefix.size(), key.size() - prefix.size() - 7);
            const auto mask = masks.find(prefix + name + "_MASK");
            fields.push_back({name, static_cast<uint32_t>(it->second),
                              mask == masks.end() ? 0u : static_cast<uint32_t>(mask->second), mask != masks.end()});
        }
    }

    std::sort(fields.begin(), fields.end());
    return fields;
}

bool same_layout(std::vector<FieldText> a, std::vector<FieldText> b) {
    if (a.size() != b.size()) {
        return false;
    }

    for (size_t i = 0; i < a.size(); ++i) {
        if (a[i].shift != b[i].shift || a[i].mask != b[i].mask) {
            return false;
        }
    }

    return true;
}

const RegisterText* find(const std::vector<RegisterText>& catalog, const char* name) {
    for (const RegisterText& reg : catalog) {
        if (reg.name == name) {
            return &reg;
        }
    }

    return nullptr;
}

bool compiled_is(const std::vector<RegisterText>& catalog, const char* name, const r::Register& reg) {
    const RegisterText* text = find(catalog, name);

    if (text == nullptr || text->base != reg.base_index || text->count != reg.count) {
        return false;
    }

    for (uint32_t i = 0; i < reg.count; ++i) {
        if (text->offsets[i] != reg.offset[i]) {
            return false;
        }
    }

    return true;
}

bool compiled_field_is(const std::vector<RegisterText>& catalog, const char* name, const char* field,
                       const r::Field& value) {
    const RegisterText* text = find(catalog, name);

    if (text == nullptr) {
        return false;
    }

    for (const FieldText& f : text->fields) {
        if (f.name == field) {
            return f.shift == value.shift && f.mask == value.mask;
        }
    }

    return false;
}

#define COMPILED(NAME) compiled_is(catalog, #NAME, r::NAME::reg)
#define COMPILED_FIELD(NAME, FIELD) compiled_field_is(catalog, #NAME, #FIELD, r::NAME::FIELD)

void run(const std::string& upstream, const std::string& source) {
    const std::string offset_text = slurp(upstream + "/dcn_3_2_0_offset.h");
    const std::string mask_text = slurp(upstream + "/dcn_3_2_0_sh_mask.h");
    const std::string sums = slurp(upstream + "/SHA256SUMS");
    const std::string catalog_text = slurp(source + "/src/display/dcn32_stream_registers.hpp");
    expect(!offset_text.empty() && !mask_text.empty() && !sums.empty() && !catalog_text.empty(),
           "pinned headers, SHA256SUMS and the catalog are readable");

    const std::string offset_sha = sha256(offset_text);
    const std::string mask_sha = sha256(mask_text);
    expect(sha256("abc") == "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad", "SHA-256 vector");
    expect(sums.find(offset_sha + "  dcn_3_2_0_offset.h") != std::string::npos, "offset header matches its pin");
    expect(sums.find(mask_sha + "  dcn_3_2_0_sh_mask.h") != std::string::npos, "mask header matches its pin");
    expect(catalog_text.find("dcn_3_2_0_offset.h  " + offset_sha) != std::string::npos &&
               catalog_text.find("dcn_3_2_0_sh_mask.h " + mask_sha) != std::string::npos,
           "the catalog was generated from the pinned headers");

    const auto offsets = defines(offset_text);
    const auto masks = defines(mask_text);
    const std::vector<RegisterText> catalog = parse_catalog(catalog_text);
    // dcn-stream 259; dcn-restore-modes +4 (dcn32_init_hw DCCG/DCFCLK gating and dccg31_init);
    // dcn-dsc-odm +37 (DSC engines, OPTC DSC/ODM widths, DP DSC and PPS SDP controls) +1
    // (OTG_TRIGA_MANUAL_TRIG of commit_planes_for_stream).
    expect(catalog.size() == 259 + 4 + 37 + 1, "catalog register count");
    unsigned bad = 0;
    std::set<std::string> names;

    for (const RegisterText& reg : catalog) {
        bool ok = names.insert(reg.name).second && reg.count == reg.headers.size() &&
                  reg.count == reg.offsets.size() && reg.count >= 1 && reg.count <= r::max_instances;

        for (size_t i = 0; ok && i < reg.headers.size(); ++i) {
            const auto offset = offsets.find("reg" + reg.headers[i]);
            const auto base = offsets.find("reg" + reg.headers[i] + "_BASE_IDX");
            ok = offset != offsets.end() && base != offsets.end() && offset->second == reg.offsets[i] &&
                 base->second == reg.base;
        }

        if (ok) {
            std::vector<FieldText> fields = reg.fields;
            std::sort(fields.begin(), fields.end());
            const std::vector<FieldText> first = header_fields(masks, reg.headers[0]);
            ok = fields.size() == first.size();

            for (size_t i = 0; ok && i < fields.size(); ++i) {
                ok = fields[i].name == first[i].name && fields[i].shift == first[i].shift &&
                     fields[i].mask == first[i].mask && first[i].has_mask;
            }

            for (size_t i = 1; ok && i < reg.headers.size(); ++i) {
                // An instance the header gives no field for (DSC_TOP1..3_DSC_DEBUG_CONTROL)
                // is programmed through the instance-0 masks, as the Linux SF lists do.
                const std::vector<FieldText> other = header_fields(masks, reg.headers[i]);
                ok = other.empty() || same_layout(first, other);
            }
        }

        if (!ok) {
            std::fprintf(stderr, "register %s differs from the pinned headers\n", reg.name.c_str());
            ++bad;
        }
    }

    expect(bad == 0, "every offset, BASE_IDX, shift and mask matches the pinned headers");
    expect(find(catalog, "HUBPREQ_DEBUG_DB") != nullptr && find(catalog, "HUBPREQ_DEBUG_DB")->fields.empty(),
           "HUBPREQ_DEBUG_DB has no field list (written whole, as Linux does)");

    // Compiled constants equal the parsed text (same file, no stale copy).
    expect(COMPILED(OTG_CONTROL) && COMPILED(OTG_MASTER_UPDATE_LOCK) && COMPILED(OTG_DOUBLE_BUFFER_CONTROL) &&
               COMPILED(HUBP_CLK_CNTL) && COMPILED(DCSURF_PRIMARY_SURFACE_ADDRESS) && COMPILED(MPCC_TOP_SEL) &&
               COMPILED(MPC_OUT_MUX) && COMPILED(DPP_CONTROL) && COMPILED(FMT_CONTROL) &&
               COMPILED(DIG_FE_CNTL) && COMPILED(DP_VID_STREAM_CNTL) && COMPILED(VPG_GENERIC_PACKET_DATA) &&
               COMPILED(DOMAIN_PG_CONFIG) && COMPILED(DENTIST_DISPCLK_CNTL) && COMPILED(DCHUBBUB_COMPBUF_CTRL),
           "compiled register constants match the catalog text");
    expect(COMPILED_FIELD(OTG_CONTROL, OTG_MASTER_EN) && COMPILED_FIELD(HDMI_CONTROL, HDMI_DATA_SCRAMBLE_EN) &&
               COMPILED_FIELD(DP_VID_STREAM_CNTL, DP_VID_STREAM_ENABLE) &&
               COMPILED_FIELD(DCHUBP_CNTL, HUBP_BLANK_EN) && COMPILED_FIELD(VTG_CONTROL, VTG0_ENABLE),
           "compiled field constants match the catalog text");

    // Pipe/instance strides the per-instance tables encode.
    expect(r::OTG_CONTROL::reg.offset[1] - r::OTG_CONTROL::reg.offset[0] == 0x80 &&
               r::OTG_CONTROL::reg.offset[3] - r::OTG_CONTROL::reg.offset[2] == 0x80,
           "OTG instances are 0x80 dwords apart");
    expect(r::DIG_FE_CNTL::reg.count == 5 && r::VPG_GENERIC_STATUS::reg.count == 5 &&
               r::OTG_CONTROL::reg.count == 4 && r::HUBP_CLK_CNTL::reg.count == 4,
           "five DIG/VPG stream engines, four pipes");
    expect(r::DENTIST_DISPCLK_CNTL::reg.base_index == 1 && r::MPC_OUT_MUX::reg.base_index == 3 &&
               r::OTG_CONTROL::reg.base_index == 2,
           "segments: DCCG 1, MPC 3, DCN 2");

    // Linux programs SURF1/CUR1 TTU fields through the SURF0/CUR0 mask entries.
    expect(same_layout(header_fields(masks, "HUBPREQ0_DCN_SURF0_TTU_CNTL0"),
                       header_fields(masks, "HUBPREQ0_DCN_SURF1_TTU_CNTL0")) &&
               same_layout(header_fields(masks, "HUBPREQ0_DCN_SURF0_TTU_CNTL1"),
                           header_fields(masks, "HUBPREQ0_DCN_SURF1_TTU_CNTL1")) &&
               same_layout(header_fields(masks, "HUBPREQ0_DCN_CUR0_TTU_CNTL1"),
                           header_fields(masks, "HUBPREQ0_DCN_CUR1_TTU_CNTL1")),
           "SURF1/CUR1 TTU layouts equal SURF0/CUR0");

    // cm_helper_program_color_matrices writes C11_C12..C33_C34 as consecutive dwords.
    bool consecutive = true;

    for (uint32_t i = 0; i < 4; ++i) {
        const uint32_t c11 = r::MPC_OUT_CSC_C11_C12_A::reg.offset[i];
        consecutive = consecutive && r::MPC_OUT_CSC_C13_C14_A::reg.offset[i] == c11 + 1 &&
                      r::MPC_OUT_CSC_C21_C22_A::reg.offset[i] == c11 + 2 &&
                      r::MPC_OUT_CSC_C23_C24_A::reg.offset[i] == c11 + 3 &&
                      r::MPC_OUT_CSC_C31_C32_A::reg.offset[i] == c11 + 4 &&
                      r::MPC_OUT_CSC_C33_C34_A::reg.offset[i] == c11 + 5;
    }

    expect(consecutive, "OCSC coefficient registers are consecutive per MPC output");
    const RegisterText* pg = find(catalog, "DOMAIN_PG_CONFIG");
    expect(pg != nullptr && pg->headers.size() == 8 && pg->headers[0] == "DOMAIN0_PG_CONFIG" &&
               pg->headers[3] == "DOMAIN3_PG_CONFIG" && pg->headers[4] == "DOMAIN16_PG_CONFIG" &&
               pg->headers[7] == "DOMAIN19_PG_CONFIG",
           "power-gating instances 0-3 are HUBP/DPP DOMAIN0-3, 4-7 are DSC DOMAIN16-19");
}

} // namespace

int main() {
#if defined(RX7900_UPSTREAM_DCN_STREAM_DIR) && defined(RX7900_SOURCE_DIR)
    run(RX7900_UPSTREAM_DCN_STREAM_DIR, RX7900_SOURCE_DIR);
#else
    expect(false, "RX7900_UPSTREAM_DCN_STREAM_DIR and RX7900_SOURCE_DIR are defined by the build");
#endif
    return checks.finish("dcn32_stream_registers");
}

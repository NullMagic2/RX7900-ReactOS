/*
Module: rx7900.tests.umc_v8_10
Purpose: Verify the UMC 8.10 error-record reader: topology from IP discovery
(node count, harvest table, MALL m_half_use), MCA status classification, the
normalized-to-physical address conversion, SMN register traffic, status and
counter clearing, bounded records and failure handling.
Created: 2026-10-24
Architecture: Production Umc810 and Nbio over a synthetic BAR5 whose RSMU
index/data pair addresses a sparse SMN register file. Expected pages were
produced by compiling the pinned Linux v6.12 umc_v8_10.c conversion functions
(umc_v8_10_get_col_bit, umc_v8_10_swizzle_mode_na_to_pa,
umc_v8_10_convert_error_address) and amdgpu_umc_fill_error_record unchanged
against host shims, outside this tree; one vector is also derived by hand
below. Not hardware evidence.
*/
#include <map>
#include <vector>

#include "asic_test_device.hpp"
#include "umc_v8_10.hpp"

namespace {

using namespace rx7900::hardware;
using rx7900_test::IpRecord;

rx7900_test::Checks checks;

void expect(bool condition, const char* contract) { checks.expect(condition, contract); }

// Segment 2 of UMC instance 0 on the user's board (navi31_umc_records).
constexpr uint32_t board_umc_base = 0x0c400000u;
constexpr uint32_t node_distance = 0x800000;
constexpr uint32_t status_register = 0x03c2;
constexpr uint32_t addr_register = 0x03c4;
constexpr uint32_t count_select_register = 0x0328;
constexpr uint32_t count_register = 0x0329;

void put16(std::vector<uint8_t>& bytes, size_t at, uint16_t value) {
    bytes.at(at) = static_cast<uint8_t>(value);
    bytes.at(at + 1) = static_cast<uint8_t>(value >> 8);
}

void put32(std::vector<uint8_t>& bytes, size_t at, uint32_t value) {
    put16(bytes, at, static_cast<uint16_t>(value));
    put16(bytes, at + 2, static_cast<uint16_t>(value >> 16));
}

struct Layout final {
    uint32_t nodes = 6;
    std::vector<std::pair<uint16_t, uint8_t>> harvest{};
    bool harvest_table = true;
    uint32_t half_use = 0;
    bool mall_table = false;
    uint16_t mall_major = 1;
    uint8_t minor = 10;
    uint32_t skip_instance = 0xff;
    uint32_t base_skew = 0;
};

// NBIF/MMHUB for the SMN pair, UMC instances, then a harvest table and a MALL
// v1 table after the IP table (binary size and checksum updated).
std::vector<uint8_t> umc_discovery(const Layout& layout) {
    std::vector<IpRecord> records{
        {rx7900::hardware::discovery_nbif_hardware_id, 4, 3,
         {rx7900_test::nbif_bases[0], rx7900_test::nbif_bases[1], rx7900_test::nbif_bases[2],
          rx7900_test::nbif_bases[3], rx7900_test::nbif_bases[4], rx7900_test::nbif_bases[5]}},
        {rx7900::hardware::discovery_mmhub_hardware_id, 3, 0, {rx7900_test::mmhub_base}},
    };

    for (auto record : rx7900_test::navi31_umc_records(layout.nodes, layout.minor)) {
        if (record.instance == layout.skip_instance) {
            continue;
        }

        if (record.instance == layout.nodes - 1) {
            record.bases.at(2) += layout.base_skew;
        }

        records.push_back(record);
    }

    auto bytes = rx7900_test::discovery_with(records);

    if (layout.harvest_table) {
        const size_t table = bytes.size();
        bytes.resize(table + 8 + 32 * 4, 0);

        for (size_t index = 0; index < layout.harvest.size(); ++index) {
            put16(bytes, table + 8 + index * 4, layout.harvest[index].first);
            bytes.at(table + 8 + index * 4 + 2) = layout.harvest[index].second;
        }

        put16(bytes, 12 + 2 * 8, static_cast<uint16_t>(table));
    }

    if (layout.mall_table) {
        const size_t table = bytes.size();
        bytes.resize(table + 36, 0);
        put32(bytes, table, 0x4c4c414du);
        put16(bytes, table + 4, layout.mall_major);
        put32(bytes, table + 8, 36);
        put32(bytes, table + 20, layout.half_use);
        put16(bytes, 12 + 4 * 8, static_cast<uint16_t>(table));
    }

    put16(bytes, 10, static_cast<uint16_t>(bytes.size()));
    uint16_t sum = 0;

    for (size_t index = 10; index < bytes.size(); ++index) {
        sum = static_cast<uint16_t>(sum + bytes.at(index));
    }

    put16(bytes, 8, sum);
    return bytes;
}

UmcTopology topology_of(const Layout& layout, DiscoveryStatus* status = nullptr) {
    const auto bytes = umc_discovery(layout);
    UmcTopology topology;
    const auto result = validate_umc_discovery({bytes.data(), bytes.size()}, topology);

    if (status != nullptr) {
        *status = result;
    }

    return topology;
}

// SMN byte address of a UMC register of (node, umc, channel), as umc_v8_10.c
// computes it with the board base: (base + reg + offset) * 4.
uint32_t smn(uint32_t node, uint32_t umc, uint32_t channel, uint32_t reg) {
    return (board_umc_base + reg + 0x400u * channel + 0x4000u * umc + node_distance * node) * 4;
}

// BAR5 with the NBIF RSMU index/data pair (segment 1 dwords 0 and 1) over a
// sparse SMN register file; every SMN data access is counted and may fail.
struct SmnModel final {
    std::map<uint32_t, uint32_t> registers;
    std::vector<std::pair<uint32_t, uint32_t>> writes;
    uint32_t index = 0;
    uint32_t accesses = 0;
    uint32_t fail_at = UINT32_MAX;

    static constexpr uint32_t index_offset = rx7900_test::nbif_bases[1] * 4;
    static constexpr uint32_t data_offset = (rx7900_test::nbif_bases[1] + 1) * 4;

    static bool read(void* context, uint32_t offset, uint32_t& value) noexcept {
        auto& self = *static_cast<SmnModel*>(context);

        if (offset == index_offset) {
            value = self.index;
            return true;
        }

        if (offset == data_offset) {
            if (++self.accesses >= self.fail_at) {
                return false;
            }

            const auto found = self.registers.find(self.index);
            value = found == self.registers.end() ? 0 : found->second;
            return true;
        }

        value = 0;
        return true;
    }

    static bool write(void* context, uint32_t offset, uint32_t value) noexcept {
        auto& self = *static_cast<SmnModel*>(context);

        if (offset == index_offset) {
            self.index = value;
            return true;
        }

        if (offset == data_offset) {
            if (++self.accesses >= self.fail_at) {
                return false;
            }

            self.registers[self.index] = value;
            self.writes.emplace_back(self.index, value);
        }

        return true;
    }

    static void delay(void*, uint32_t) noexcept {}

    RegisterIo io() noexcept { return {this, rx7900_test::aperture_bytes, read, write, delay}; }

    void set64(uint32_t address, uint64_t value) {
        registers[address] = static_cast<uint32_t>(value);
        registers[address + 4] = static_cast<uint32_t>(value >> 32);
    }

    uint64_t get64(uint32_t address) const {
        const auto low = registers.find(address);
        const auto high = registers.find(address + 4);
        return (low == registers.end() ? 0 : low->second) |
               (high == registers.end() ? 0 : static_cast<uint64_t>(high->second) << 32);
    }

    unsigned writes_to(uint32_t address) const {
        unsigned count = 0;

        for (const auto& write : writes) {
            count += write.first == address ? 1 : 0;
        }

        return count;
    }
};

struct Reader final {
    SmnModel model{};
    Umc810 umc{};
    UmcTopology topology{};
    bool ready = false;

    explicit Reader(const Layout& layout = {}) {
        const auto bytes = umc_discovery(layout);
        NbioRegisters nbio{};
        ready = validate_nbio_discovery({bytes.data(), bytes.size()}, rx7900_test::aperture_bytes, nbio) ==
                    DiscoveryStatus::validated &&
                validate_umc_discovery({bytes.data(), bytes.size()}, topology) == DiscoveryStatus::validated &&
                umc.initialize(topology, nbio, model.io()) == UmcStatus::ready;
    }

    void error(uint32_t node, uint32_t umc_instance, uint32_t channel, uint64_t status, uint64_t address) {
        model.set64(smn(node, umc_instance, channel, status_register), status);
        model.set64(smn(node, umc_instance, channel, addr_register), address);
    }
};

constexpr uint64_t val = umc_status_val;
constexpr uint64_t addrv = umc_status_addrv;
constexpr uint64_t uecc = umc_status_uecc;
constexpr uint64_t cecc = umc_status_cecc;
constexpr uint64_t deferred = umc_status_deferred;

uint64_t lsb(uint32_t bits) { return static_cast<uint64_t>(bits) << umc_status_addr_lsb_shift; }

void topology() {
    DiscoveryStatus status{};
    const auto board = topology_of({}, &status);
    uint32_t index = 0;
    expect(status == DiscoveryStatus::validated && board.base() == 0x0c400000u && board.node_count() == 6 &&
               board.active_mask() == 0x3fu && board.channel_count() == 24 && board.column_bit() == 13,
           "board layout: six UMC 8.10 nodes, 24 channels, column bit 13, UMC base reg_offset[UMC][0][2]");
    expect(board.channel_index(0, 0, 0, index) && index == 16 && board.channel_index(5, 1, 1, index) &&
               index == 6 && board.channel_index(3, 0, 1, index) && index == 21 &&
               !board.channel_index(6, 0, 0, index) && !board.channel_index(0, 2, 0, index),
           "umc_v8_10_channel_idx_tbl[node][umc][channel], bounded");

    Layout four{};
    four.nodes = 4;
    const auto navi32 = topology_of(four);
    expect(navi32.channel_count() == 16 && navi32.column_bit() == 12 && navi32.channel_index(0, 1, 0, index) &&
               index == 7 && navi32.channel_index(3, 1, 1, index) && index == 4,
           "four nodes select umc_v8_10_channel_idx_tbl_ext0 and 16 channels (column bit 12)");

    Layout harvested{};
    harvested.harvest = {{42, 0}, {discovery_umc_hardware_id, 2}};
    const auto xt = topology_of(harvested);
    expect(xt.node_count() == 6 && xt.active_mask() == 0x3bu && xt.channel_count() == 20 && xt.column_bit() == 13,
           "a harvested UMC leaves the node table but not the active mask; 20 channels");

    Layout half{};
    half.mall_table = true;
    half.half_use = 0x5;
    expect(topology_of(half).channel_count() == 20, "MALL m_half_use: hweight32 * 2 channels fewer");
    half.mall_major = 2;
    expect(topology_of(half).channel_count() == 24, "a MALL_INFO v2 table carries no m_half_use");
    half.mall_major = 1;
    half.half_use = 0x1;
    const auto odd = topology_of(half);
    expect(odd.valid() && odd.channel_count() == 22 && odd.column_bit() == 0,
           "22 channels have no column-bit mapping: valid for counting, no conversion (Linux U32_MAX)");

    Layout terminated{};
    terminated.harvest = {{42, 0}, {0, 0}, {discovery_umc_hardware_id, 1}};
    expect(topology_of(terminated).active_mask() == 0x3fu && topology_of(terminated).channel_count() == 24,
           "harvest entries after the zero hw_id terminator are ignored");

    Layout no_table{};
    no_table.harvest_table = false;
    expect(topology_of(no_table).active_mask() == 0x3fu, "no harvest table: nothing harvested (Linux logs it)");

    Layout gap{};
    gap.skip_instance = 3;
    Layout skew{};
    skew.base_skew = 0x10;
    Layout version{};
    version.minor = 11;
    Layout seven{};
    seven.nodes = 7;
    Layout all{};
    all.nodes = 1;
    all.harvest = {{discovery_umc_hardware_id, 0}};
    Layout instance{};
    instance.harvest = {{discovery_umc_hardware_id, 40}};
    DiscoveryStatus gap_status{}, skew_status{}, version_status{}, seven_status{}, all_status{}, instance_status{};
    topology_of(gap, &gap_status);
    topology_of(skew, &skew_status);
    topology_of(version, &version_status);
    topology_of(seven, &seven_status);
    topology_of(all, &all_status);
    topology_of(instance, &instance_status);
    expect(gap_status == DiscoveryStatus::unsupported && skew_status == DiscoveryStatus::unsupported &&
               version_status == DiscoveryStatus::unsupported && seven_status == DiscoveryStatus::unsupported &&
               all_status == DiscoveryStatus::unsupported && instance_status == DiscoveryStatus::invalid_binary,
           "refused: instance gap, base off UMC_8_NODE_DIST, UMC 8.11, more nodes than the table, no active "
           "node, harvest instance >= 32");

    auto corrupt = umc_discovery({});
    corrupt.at(200) ^= 1;
    UmcTopology rejected;
    expect(validate_umc_discovery({corrupt.data(), corrupt.size()}, rejected) == DiscoveryStatus::invalid_binary &&
               !rejected.valid(),
           "a bad discovery checksum yields no topology");
}

void conversion() {
    const auto board = topology_of({});
    UmcErrorRecord records[umc_v8_10_retire_unit]{};

    // Hand derivation for node 0, umc 0, channel 0 (channel index 16, 24
    // channels, column bit 13), address 0, AddrLsb 0: na = col << 14, so
    // tmp = ((col * 16) * 24 + 16) << 10; PA keeps tmp bits >= 15, puts na
    // bits 9:8 (0) at 13 and tmp bits 14:10 at 12:8. col 0: tmp = 0x4000,
    // PA = 0x0 | (0x10 << 8) = 0x1000. col 1: tmp = (16 * 24 + 16) << 10 =
    // 0x64000, PA = 0x60000 | ((0x190 & 0x1f) << 8) = 0x61000.
    expect(umc_v8_10_convert_error_address(board, 0, 0, 0, val | addrv | uecc, 0, records) &&
               records[0].vram_offset == 0x1000 && records[1].vram_offset == 0x61000 &&
               records[0].normalized == 0 && records[1].normalized == 0x4000 && records[3].normalized == 0xc000 &&
               records[2].channel_index == 16,
           "hand-derived swizzle: address 0 of channel index 16");

    // Linux-generated vectors (see file header).
    struct Vector final {
        uint32_t node;
        uint32_t umc;
        uint32_t channel;
        uint64_t status;
        uint64_t address;
        uint64_t pages[4];
        uint64_t normalized[4];
    };

    const Vector vectors[]{
        {3, 0, 0, 0x8400200036a7deddULL, 0x201fdd96cULL,
         {0x1000ULL, 0x61000ULL, 0xc1000ULL, 0x121000ULL},
         {0x0ULL, 0x4000ULL, 0x8000ULL, 0xc000ULL}},
        {3, 0, 1, 0x8400200004197207ULL, 0x62a1b0e7f4494aULL,
         {0x93f2895bee12000ULL, 0x93f2895bee72000ULL, 0x93f2895beed2000ULL, 0x93f2895bef32000ULL},
         {0x62a1b0e7f40940ULL, 0x62a1b0e7f44940ULL, 0x62a1b0e7f48940ULL, 0x62a1b0e7f4c940ULL}},
        {2, 1, 0, 0x840020000262dc66ULL, 0xa5e325f3af5c6fULL,
         {0xf8d4b8ed86a9000ULL, 0xf8d4b8ed8709000ULL, 0xf8d4b8ed8769000ULL, 0xf8d4b8ed87c9000ULL},
         {0xa5e325f3af1c6cULL, 0xa5e325f3af5c6cULL, 0xa5e325f3af9c6cULL, 0xa5e325f3afdc6cULL}},
        {5, 1, 0, 0x840020000b7ede9cULL, 0x6ef3bac44ULL,
         {0xa66d8b9000ULL, 0xa66d919000ULL, 0xa66d979000ULL, 0xa66d9d9000ULL},
         {0x6ef3b2800ULL, 0x6ef3b6800ULL, 0x6ef3ba800ULL, 0x6ef3be800ULL}},
        {1, 0, 0, 0x8400200004962421ULL, 0x124d05bab1dd0ULL,
         {0x1b7388980ab000ULL, 0x1b73889810b000ULL, 0x1b73889816b000ULL, 0x1b7388981cb000ULL},
         {0x124d05bab1dd0ULL, 0x124d05bab5dd0ULL, 0x124d05bab9dd0ULL, 0x124d05babddd0ULL}},
        {2, 1, 0, 0x8400200017fa4f84ULL, 0xbffc00ab287ad2ULL,
         {0x11ffa01008000000ULL, 0x11ffa01008060000ULL, 0x11ffa010080c0000ULL, 0x11ffa01008120000ULL},
         {0xbffc00ab000000ULL, 0xbffc00ab004000ULL, 0xbffc00ab008000ULL, 0xbffc00ab00c000ULL}},
        {5, 1, 0, 0x840020000b45d615ULL, 0x4e62d4a9cULL,
         {0x7594389000ULL, 0x75943e9000ULL, 0x7594449000ULL, 0x75944a9000ULL},
         {0x4e62d0800ULL, 0x4e62d4800ULL, 0x4e62d8800ULL, 0x4e62dc800ULL}},
        {5, 1, 1, 0x840020000518e890ULL, 0x1a10f0f08dcf77ULL,
         {0x27196968d396000ULL, 0x27196968d3f6000ULL, 0x27196968d456000ULL, 0x27196968d4b6000ULL},
         {0x1a10f0f08d0f60ULL, 0x1a10f0f08d4f60ULL, 0x1a10f0f08d8f60ULL, 0x1a10f0f08dcf60ULL}},
        {3, 0, 1, 0x8400200009e34487ULL, 0xab686fcac7a355ULL,
         {0x1011ca7b02ab5000ULL, 0x1011ca7b02b15000ULL, 0x1011ca7b02b75000ULL, 0x1011ca7b02bd5000ULL},
         {0xab686fcac72200ULL, 0xab686fcac76200ULL, 0xab686fcac7a200ULL, 0xab686fcac7e200ULL}},
        {5, 1, 0, 0x8400200004baa11aULL, 0xd98b1d4fULL,
         {0x14650aa000ULL, 0x146510a000ULL, 0x146516a000ULL, 0x14651ca000ULL},
         {0xd98b1d40ULL, 0xd98b5d40ULL, 0xd98b9d40ULL, 0xd98bdd40ULL}},
    };

    // Node 2 harvested (20 channels, column bit 13) and four nodes (ext0
    // table, 16 channels, column bit 12).
    const Vector harvested_vectors[]{
        {5, 0, 0, 0x840020000354fa88ULL, 0x382d1c6b3ULL,
         {0x463854c000ULL, 0x463859c000ULL, 0x46385ec000ULL, 0x463863c000ULL},
         {0x382d106b0ULL, 0x382d146b0ULL, 0x382d186b0ULL, 0x382d1c6b0ULL}},
        {1, 0, 0, 0x8400200002261121ULL, 0x549e82265b321dULL,
         {0x69c622aff1fd000ULL, 0x69c622aff24d000ULL, 0x69c622aff29d000ULL, 0x69c622aff2ed000ULL},
         {0x549e82265b321cULL, 0x549e82265b721cULL, 0x549e82265bb21cULL, 0x549e82265bf21cULL}},
    };
    const Vector four_node_vectors[]{
        {0, 1, 0, 0x8400200035b5ca3bULL, 0x7246d189aULL,
         {0x0ULL, 0x40000ULL, 0x80000ULL, 0xc0000ULL},
         {0x0ULL, 0x4000ULL, 0x8000ULL, 0xc000ULL}},
        {2, 1, 1, 0x840020000824f3f2ULL, 0xcfcff0159c5b3dULL,
         {0xcfcff0159c1b000ULL, 0xcfcff0159c5b000ULL, 0xcfcff0159c9b000ULL, 0xcfcff0159cdb000ULL},
         {0xcfcff0159c1b00ULL, 0xcfcff0159c5b00ULL, 0xcfcff0159c9b00ULL, 0xcfcff0159cdb00ULL}},
        {2, 1, 0, 0x8400200002be2726ULL, 0xeae68dceff513ULL,
         {0xeae68dcef35000ULL, 0xeae68dcef75000ULL, 0xeae68dcefb5000ULL, 0xeae68dceff5000ULL},
         {0xeae68dcef3510ULL, 0xeae68dcef7510ULL, 0xeae68dcefb510ULL, 0xeae68dceff510ULL}},
    };
    Layout harvested{};
    harvested.harvest = {{discovery_umc_hardware_id, 2}};
    Layout four{};
    four.nodes = 4;
    const auto matches = [](const UmcTopology& topology, const Vector* first, size_t count) {
        bool all = count != 0;

        for (size_t index = 0; index < count; ++index) {
            const Vector& vector = first[index];
            UmcErrorRecord converted[umc_v8_10_retire_unit]{};
            bool ok = umc_v8_10_convert_error_address(topology, vector.node, vector.umc, vector.channel,
                                                      vector.status, vector.address, converted);

            for (uint32_t column = 0; column < umc_v8_10_retire_unit && ok; ++column) {
                ok = converted[column].vram_offset == vector.pages[column] &&
                     converted[column].normalized == vector.normalized[column] &&
                     converted[column].node == vector.node && converted[column].umc == vector.umc &&
                     converted[column].channel == vector.channel;
            }

            all = all && ok;
        }

        return all;
    };
    const bool all = matches(board, vectors, sizeof(vectors) / sizeof(vectors[0])) &&
                     matches(topology_of(harvested), harvested_vectors,
                             sizeof(harvested_vectors) / sizeof(harvested_vectors[0])) &&
                     matches(topology_of(four), four_node_vectors,
                             sizeof(four_node_vectors) / sizeof(four_node_vectors[0]));

    expect(all, "pages and normalized addresses match the pinned Linux conversion for every vector");

    Layout half{};
    half.mall_table = true;
    half.half_use = 0x1;
    expect(!umc_v8_10_convert_error_address(topology_of(half), 0, 0, 0, val | addrv | uecc, 0x1234, records) &&
               !umc_v8_10_convert_error_address(board, 6, 0, 0, val | addrv | uecc, 0x1234, records) &&
               !umc_v8_10_convert_error_address(UmcTopology{}, 0, 0, 0, val | addrv | uecc, 0x1234, records),
           "no column-bit mapping, a node outside the table or no topology: no conversion");
    UmcErrorRecord masked[umc_v8_10_retire_unit]{};
    expect(umc_v8_10_convert_error_address(board, 2, 0, 1, val | addrv | uecc, 0xff00000000123456ULL, records) &&
               umc_v8_10_convert_error_address(board, 2, 0, 1, val | addrv | uecc, 0x123456ULL, masked) &&
               records[0].vram_offset == masked[0].vram_offset && records[3].normalized == masked[3].normalized,
           "only the 56 ErrorAddr bits of MCUMC_ADDRT0 are used");
}

void classification() {
    expect(umc_status_correctable(val | cecc) && !umc_status_correctable(cecc) &&
               !umc_status_uncorrectable(val | cecc),
           "CE: Val and CECC only");
    expect(umc_status_uncorrectable(val | deferred) && umc_status_uncorrectable(val | uecc) &&
               umc_status_uncorrectable(val | umc_status_pcc) && umc_status_uncorrectable(val | umc_status_uc) &&
               umc_status_uncorrectable(val | umc_status_tcc) && !umc_status_uncorrectable(deferred | uecc) &&
               !umc_status_uncorrectable(val | umc_status_poison),
           "UE: Val with Deferred, UECC, PCC, UC or TCC (Poison alone is not counted)");
    expect(umc_status_has_address(val | addrv | uecc) && !umc_status_has_address(val | addrv | deferred) &&
               !umc_status_has_address(val | uecc) && !umc_status_has_address(addrv | uecc),
           "an address is converted only for Val, AddrV and UECC");
}

void query() {
    Reader reader;
    expect(reader.ready, "NBIO SMN pair and UMC topology ready");
    UmcErrorRecord records[umc_v8_10_record_capacity]{};
    auto result = reader.umc.query(records, umc_v8_10_record_capacity);
    expect(!result.access_failed && result.channels == 24 && result.correctable == 0 && result.uncorrectable == 0 &&
               result.records == 0,
           "quiet channels: 24 statuses read, nothing recorded");
    expect(reader.model.writes_to(smn(0, 0, 0, status_register)) == 0 &&
               reader.model.writes_to(smn(5, 1, 1, count_register)) == 1 &&
               reader.model.registers[smn(5, 1, 1, count_register)] == 0,
           "a zero status is not written; every GeccErrCnt is reset to UMC_V8_10_CE_CNT_INIT (0)");

    reader.model.writes.clear();
    // Node 1 umc 0 ch 1: CE. Node 2 umc 1 ch 0: UE with address. Node 4 umc 0
    // ch 0: deferred (poison) without UECC. Node 5 umc 1 ch 1: UE without AddrV.
    reader.error(1, 0, 1, val | cecc | addrv, 0x777);
    reader.error(2, 1, 0, val | addrv | uecc | lsb(6), 0x123456789abULL);
    reader.error(4, 0, 0, val | addrv | deferred, 0x5000);
    reader.error(5, 1, 1, val | uecc, 0x6000);
    reader.model.set64(smn(3, 0, 0, status_register), 0x1);
    result = reader.umc.query(records, umc_v8_10_record_capacity);
    UmcErrorRecord expected[umc_v8_10_retire_unit]{};
    const bool converted = umc_v8_10_convert_error_address(reader.topology, 2, 1, 0, val | addrv | uecc | lsb(6),
                                                           0x123456789abULL, expected);
    bool same = converted && result.records == umc_v8_10_retire_unit;

    for (uint32_t index = 0; same && index < umc_v8_10_retire_unit; ++index) {
        same = records[index].vram_offset == expected[index].vram_offset && records[index].node == 2 &&
               records[index].umc == 1 && records[index].channel == 0;
    }

    expect(result.correctable == 1 && result.uncorrectable == 3 && result.deferred == 1 && result.decoded == 1 &&
               result.undecoded == 2 && same,
           "CE counted; UECC+AddrV converted to four pages; deferred-only and no-AddrV UEs are undecoded");
    expect(reader.model.get64(smn(1, 0, 1, status_register)) == 0 &&
               reader.model.get64(smn(2, 1, 0, status_register)) == 0 &&
               reader.model.get64(smn(4, 0, 0, status_register)) == 0 &&
               reader.model.get64(smn(5, 1, 1, status_register)) == 0 &&
               reader.model.get64(smn(3, 0, 0, status_register)) == 0 &&
               reader.model.writes_to(smn(2, 1, 0, status_register)) == 1 &&
               reader.model.writes_to(smn(2, 1, 0, status_register) + 4) == 1,
           "every nonzero status (also Val=0) is cleared with one 64-bit write (low, then high dword)");

    // Pending-record overflow: seven UECC errors give 28 pages for 24 slots.
    Reader full;
    const uint32_t nodes[7]{0, 0, 1, 2, 3, 4, 5};
    const uint32_t umcs[7]{0, 1, 1, 0, 1, 0, 1};

    for (uint32_t index = 0; index < 7; ++index) {
        full.error(nodes[index], umcs[index], 0, val | addrv | uecc, 0x10000ULL * (index + 1));
    }

    full.error(5, 0, 1, val | addrv | uecc | umc_status_overflow, 0x2000);
    result = full.umc.query(records, umc_v8_10_record_capacity);
    expect(result.uncorrectable == 8 && result.decoded == 8 && result.records == umc_v8_10_record_capacity &&
               result.dropped == 8 && result.overflowed == 1 && result.undecoded == 0,
           "records are bounded by max_ras_err_cnt_per_query: later pages are dropped and counted");
    expect(full.model.get64(smn(5, 0, 1, status_register)) == 0 && full.model.get64(smn(0, 0, 0, status_register)) == 0,
           "dropped records' statuses are still cleared (Linux clears after fill_error_record fails)");

    UmcErrorRecord few[2]{};
    Reader small;
    small.error(0, 0, 0, val | addrv | uecc, 0x40);
    result = small.umc.query(few, 2);
    expect(result.records == 2 && result.dropped == 2, "a smaller caller capacity bounds the records too");

    Reader harvested(Layout{6, {{discovery_umc_hardware_id, 2}}});
    harvested.error(2, 0, 0, val | addrv | uecc, 0x40);
    result = harvested.umc.query(records, umc_v8_10_record_capacity);
    expect(result.channels == 20 && result.uncorrectable == 0 &&
               harvested.model.get64(smn(2, 0, 0, status_register)) != 0,
           "a harvested node is never read (LOOP_UMC_NODE_INST over active_mask)");

    Reader zero;
    zero.error(0, 1, 1, val | addrv | uecc, 0);
    zero.error(1, 1, 1, val | addrv | uecc | lsb(63), UINT64_MAX);
    result = zero.umc.query(records, umc_v8_10_record_capacity);
    expect(result.records == 8 && records[0].normalized == 0 && records[1].normalized == 0x4000 &&
               records[4].normalized == 0 && records[7].normalized == 0xc000,
           "a zero address is a valid address; AddrLsb 63 clears all 56 address bits");
}

void counters_and_failures() {
    Reader reader;
    reader.model.registers[smn(0, 0, 0, count_select_register)] = 0xffffffffu;
    reader.model.registers[smn(3, 1, 0, count_select_register)] = 0x00008000u;
    expect(reader.umc.initialize_counters(), "umc_v8_10_err_cnt_init succeeds");
    expect(reader.model.registers[smn(0, 0, 0, count_select_register)] == 0xffffdfffu &&
               reader.model.registers[smn(3, 1, 0, count_select_register)] == 0x00009000u &&
               reader.model.registers[smn(3, 1, 0, count_register)] == 0 &&
               reader.model.writes.size() == 48,
           "GeccErrInt (bits 13:12) set to 1, other bits kept; GeccErrCnt = 0; 24 channels x 2 writes");

    for (uint32_t fail = 1; fail < 8; ++fail) {
        Reader failing;
        failing.error(0, 0, 0, val | addrv | uecc, 0x40);
        failing.model.fail_at = fail;
        UmcErrorRecord records[umc_v8_10_record_capacity]{};
        const auto result = failing.umc.query(records, umc_v8_10_record_capacity);
        expect(result.access_failed, "any failed SMN access ends the query as failed");
    }

    Umc810 idle;
    UmcErrorRecord records[1]{};
    expect(!idle.ready() && idle.query(records, 1).access_failed && !idle.initialize_counters() &&
               idle.initialize(UmcTopology{}, NbioRegisters{}, RegisterIo{}) == UmcStatus::invalid_argument,
           "an uninitialized reader reads nothing");
    Reader reader2;
    expect(reader2.umc.query(nullptr, 1).access_failed, "a null record array with capacity is refused");
}

} // namespace

int main() {
    topology();
    conversion();
    classification();
    query();
    counters_and_failures();
    return checks.finish("umc_v8_10");
}

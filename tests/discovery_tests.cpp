/*
Module: rx7900.tests.discovery
Purpose: Exercise shared IP traversal, ambiguity, checksums and register arithmetic.
Created: 2026-10-21
Architecture: Synthetic binary fixtures only; no GPU access or device-presence claims.
*/
#include <cstdio>
#include "discovery_test_fixture.hpp"

namespace {

using namespace rx7900::hardware;
using namespace rx7900_test;
using rx7900::ByteView;
using rx7900::sdma::CommandBuffer;
unsigned checks = 0;
unsigned failures = 0;
constexpr DiscoveryQuery gc{discovery_gc_hardware_id, 0, 11, 0, 0, 2};
constexpr DiscoveryQuery mp0{discovery_mp0_hardware_id, 0, 13, 0, 0, 1};

void expect(bool condition, const char* contract) {
    ++checks;

    if (!condition) {
        ++failures;
        std::fprintf(stderr, "FAIL: %s\n", contract);
    }
}

template<size_t Size>
DiscoveryStatus lookup(const std::array<uint8_t, Size>& bytes, DiscoveryQuery query,
                       DiscoveredIp& ip) {
    return discover_ip({bytes.data(), bytes.size()}, query, ip);
}

std::array<uint8_t, 256> multi_ip(bool two_dies) {
    std::array<uint8_t, 256> bytes{};
    const auto one = discovery_binary();

    for (size_t index = 0; index < 160; ++index) {
        bytes.at(index) = one.at(index);
    }

    CommandBuffer output(bytes.data(), bytes.size());
    const uint16_t end = two_dies ? 180 : 176;
    discovery_u16(bytes, 10, end);
    discovery_u16(bytes, 16, static_cast<uint16_t>(end - 60));
    discovery_u16(bytes, 66, static_cast<uint16_t>(end - 60));
    size_t second_ip = 160;

    if (two_dies) {
        discovery_u16(bytes, 72, 2);
        discovery_u16(bytes, 78, 1);
        discovery_u16(bytes, 80, 160);
        discovery_u16(bytes, 160, 1);
        discovery_u16(bytes, 162, 1);
        second_ip = 164;
    } else {
        discovery_u16(bytes, 142, 2);
    }

    discovery_u16(bytes, second_ip, discovery_mp0_hardware_id);
    bytes.at(second_ip + 3) = 2;
    bytes.at(second_ip + 4) = 13;
    output.write_u32(second_ip + 8, 0x100);
    output.write_u32(second_ip + 12, 0x300);
    discovery_checksums(bytes);
    return bytes;
}

void check_binary() {
    DiscoveredIp ip;
    expect(!ip.valid() && ip.segment(0) == 0 && ip.segment(2) == 0,
           "default descriptor cannot authorize register ownership");
    auto bytes = discovery_binary();
    expect(lookup(bytes, gc, ip) == DiscoveryStatus::validated && ip.valid() &&
           ip.segment(0) == 0x1000 && ip.segment(1) == 0x2000,
           "preserve both validated GC segment bases");

    for (size_t size = 0; size < 160; ++size) {
        expect(discover_ip({bytes.data(), size}, gc, ip) == DiscoveryStatus::invalid_binary &&
               !ip.valid() && ip.segment(0) == 0, "truncated binary clears prior token");
    }

    expect(discover_ip({nullptr, SIZE_MAX}, gc, ip) == DiscoveryStatus::invalid_binary,
           "null input is empty even with forged length");
    expect(discover_ip({bytes.data(), bytes.size()}, {11, 0, 11, 0, 0, 0}, ip) ==
           DiscoveryStatus::invalid_argument && !ip.valid(), "zero requested segments rejected");
    expect(discover_ip({bytes.data(), bytes.size()}, {11, 0, 11, 0, 0, 7}, ip) ==
           DiscoveryStatus::invalid_argument, "bounded token cannot hold seven requested segments");
    expect(discover_ip({bytes.data(), bytes.size()}, {11, 0, 11, 0, 0, 3}, ip) ==
           DiscoveryStatus::unsupported && !ip.valid(), "record with two bases cannot supply three");
    bytes.at(159) ^= 1;
    expect(lookup(bytes, gc, ip) == DiscoveryStatus::invalid_binary, "outer checksum must match");
    bytes = discovery_binary();
    bytes.at(14) ^= 1;
    discovery_checksums(bytes, false);
    expect(lookup(bytes, gc, ip) == DiscoveryStatus::invalid_binary,
           "valid outer checksum cannot mask wrong table checksum");
    bytes = discovery_binary();
    discovery_u16(bytes, 16, 99);
    discovery_checksums(bytes);
    expect(lookup(bytes, gc, ip) == DiscoveryStatus::invalid_binary,
           "outer table size must agree when supplied");
    discovery_u16(bytes, 16, 0);
    discovery_checksums(bytes);
    expect(lookup(bytes, gc, ip) == DiscoveryStatus::validated,
           "preserve permitted unspecified outer table size");

    for (const uint16_t dies : {uint16_t{0}, uint16_t{17}, uint16_t{65535}}) {
        bytes = discovery_binary();
        discovery_u16(bytes, 72, dies);
        discovery_checksums(bytes);
        expect(lookup(bytes, gc, ip) == DiscoveryStatus::invalid_binary && !ip.valid(),
               "die count bounded by checked fixed storage");
    }

    for (const size_t field : {size_t{146}, size_t{148}, size_t{149}, size_t{150}, size_t{151}}) {
        bytes = discovery_binary();
        ++bytes.at(field);
        discovery_checksums(bytes);
        const auto wanted = field == 146 ? DiscoveryStatus::invalid_binary : DiscoveryStatus::unsupported;
        expect(lookup(bytes, gc, ip) == wanted && !ip.valid(),
               "instance and complete exact version must match");
    }

    bytes = discovery_binary();
    bytes.at(147) = 1;
    discovery_checksums(bytes);
    expect(lookup(bytes, gc, ip) == DiscoveryStatus::unsupported,
           "required GC segment1 cannot be absent");
    bytes.at(147) = 255;
    discovery_checksums(bytes);
    expect(lookup(bytes, gc, ip) == DiscoveryStatus::invalid_binary,
           "variable base array cannot escape table");
    bytes = discovery_binary(true);
    expect(lookup(bytes, gc, ip) == DiscoveryStatus::validated,
           "IP version4 permits local zero-extended64-bit bases");
    bytes.at(156) = 1;
    discovery_checksums(bytes);
    expect(lookup(bytes, gc, ip) == DiscoveryStatus::unsupported && !ip.valid(),
           "wide remote address cannot truncate to low dword");
    bytes = discovery_binary();
    bytes.at(155) = 0xc0;
    discovery_checksums(bytes);
    expect(lookup(bytes, gc, ip) == DiscoveryStatus::unsupported,
           "local low30-bit base contract rejects XCC extension");

    for (const uint8_t flags : {uint8_t{1}, uint8_t{2}, uint8_t{255}}) {
        bytes = discovery_binary();
        bytes.at(138) = flags;
        discovery_checksums(bytes);
        expect(lookup(bytes, gc, ip) == DiscoveryStatus::unsupported,
               "version3 reserved address flags rejected");
    }

    bytes = discovery_binary();
    discovery_u16(bytes, 64, 2);
    discovery_checksums(bytes);
    expect(lookup(bytes, gc, ip) == DiscoveryStatus::unsupported,
           "unimplemented table generations rejected");
    bytes = discovery_binary();
    bytes.at(139) = 1;
    discovery_checksums(bytes);
    expect(lookup(bytes, gc, ip) == DiscoveryStatus::unsupported,
           "unknown header address metadata rejected");
}

void check_traversal() {
    DiscoveredIp ip;

    for (const bool two_dies : {false, true}) {
        auto bytes = multi_ip(two_dies);
        expect(lookup(bytes, gc, ip) == DiscoveryStatus::validated && ip.segment(1) == 0x2000,
               "GC located without assuming exclusive or first-only table");
        expect(lookup(bytes, mp0, ip) == DiscoveryStatus::validated && ip.segment(0) == 0x100 &&
               ip.segment(1) == 0, "MP0 located across complete die/IP traversal");
        const size_t second = two_dies ? 164 : 160;
        discovery_u16(bytes, second, discovery_gc_hardware_id);
        bytes.at(second + 4) = 11;
        discovery_checksums(bytes);
        expect(lookup(bytes, gc, ip) == DiscoveryStatus::unsupported && !ip.valid(),
               "duplicate target within or across dies is ambiguous");
    }

    auto bytes = multi_ip(true);
    // Repeating the same empty die record is still an overlapping die extent.
    discovery_u16(bytes, 142, 0);
    discovery_u16(bytes, 78, 0);
    discovery_u16(bytes, 80, 140);
    discovery_checksums(bytes);
    expect(lookup(bytes, gc, ip) == DiscoveryStatus::invalid_binary && !ip.valid(),
           "overlapping die extents cannot alias even without matching IPs");
    bytes = multi_ip(true);
    discovery_u16(bytes, 160, 2);
    discovery_checksums(bytes);
    expect(lookup(bytes, gc, ip) == DiscoveryStatus::invalid_binary,
           "die header identity must match directory");
    bytes = multi_ip(true);
    discovery_u16(bytes, 162, 65535);
    discovery_checksums(bytes);
    expect(lookup(bytes, gc, ip) == DiscoveryStatus::invalid_binary && !ip.valid(),
           "malformed trailing die invalidates an earlier matching entry");
    bytes = multi_ip(true);
    discovery_u16(bytes, 80, 179);
    discovery_checksums(bytes);
    expect(lookup(bytes, gc, ip) == DiscoveryStatus::invalid_binary,
           "truncated die header cannot escape table end");
}

void check_register_bounds() {
    uint32_t offset = 0xdeadbeef;
    expect(discovery_register_offset(0x100, 0x91, 0x648, offset) && offset == 0x644,
           "final complete dword fits exactly");
    expect(!discovery_register_offset(0x100, 0x91, 0x644, offset) && offset == 0x644,
           "one dword past aperture leaves output unchanged");
    expect(!discovery_register_offset(UINT32_MAX, UINT32_MAX, UINT32_MAX - 3, offset),
           "base-plus-register addition cannot wrap32bits");
    expect(!discovery_register_offset(0, 0, 3, offset), "unaligned BAR extent rejected");
    expect(!discovery_register_offset(0, 0, 0, offset), "empty BAR cannot contain register");
}

} // namespace

int main() {
    check_binary();
    check_traversal();
    check_register_bounds();
    std::printf("Discovery tests: %u checks, %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

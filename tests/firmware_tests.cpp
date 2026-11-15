/*
Module: rx7900.tests.firmware
Purpose: Check real AMD container layouts and malformed payload boundaries.
Created: 2026-10-21
Architecture: Pure parser tests; private firmware fixtures never execute on GPU.
*/
#include <array>
#include <cstdio>
#include <limits>
#include <vector>

#include "firmware.hpp"
#include "file_support.hpp"

namespace {

using rx7900::ByteView;
using rx7900::hardware::FirmwareImage;
using rx7900::hardware::FirmwareKind;
using rx7900::hardware::FirmwareStatus;
using Bytes = std::array<uint8_t, 512>;
unsigned checks = 0;
unsigned failures = 0;

void expect(bool condition, const char* contract) {
    ++checks;
    if (!condition) {
        ++failures;
        std::fprintf(stderr, "FAIL: %s\n", contract);
    }
}

void put(Bytes& bytes, size_t offset, uint32_t value) {
    for (size_t index = 0; index < sizeof(value); ++index) {
        bytes.at(offset + index) = static_cast<uint8_t>(value >> (index * 8));
    }
}

Bytes fixture(FirmwareKind kind) {
    Bytes bytes{};
    put(bytes, 0, bytes.size());
    put(bytes, 8, kind == FirmwareKind::dmcub_3_2 ? 1 : 2);
    put(bytes, 12, kind == FirmwareKind::sdma_6_0 ? 6 :
                   (kind == FirmwareKind::psp_13_0 ? 13 : 0x00020003));
    put(bytes, 16, 27);
    put(bytes, 20, 256);
    put(bytes, 24, 256);

    if (kind == FirmwareKind::sdma_6_0) {
        put(bytes, 4, 64);
        put(bytes, 36, 128);
        put(bytes, 40, 64);
        put(bytes, 44, 16);
        put(bytes, 48, 384);
        put(bytes, 52, 128);
        put(bytes, 56, 64);
        put(bytes, 60, 16);
    } else if (kind == FirmwareKind::psp_13_0) {
        put(bytes, 4, 36);
        put(bytes, 32, 2);
        put(bytes, 36, 2);
        put(bytes, 40, 27);
        put(bytes, 44, 0);
        put(bytes, 48, 128);
        put(bytes, 52, 1);
        put(bytes, 56, 27);
        put(bytes, 60, 128);
        put(bytes, 64, 128);
    } else {
        put(bytes, 4, 40);
        put(bytes, 32, 256);
        put(bytes, 36, 4096);
    }

    return bytes;
}

void reject(const Bytes& bytes, FirmwareKind kind, const char* reason) {
    FirmwareImage image{};
    image.version = 0xaabbccdd;
    expect(rx7900::hardware::validate_firmware(ByteView(bytes.data(), bytes.size()),
        kind, image) != FirmwareStatus::success, reason);
    expect(image.version == 0xaabbccdd, "failed validation preserves output");
}

void check_layout(FirmwareKind kind) {
    auto bytes = fixture(kind);
    FirmwareImage image{};
    expect(rx7900::hardware::validate_firmware(ByteView(bytes.data(), bytes.size()),
        kind, image) == FirmwareStatus::success, "valid synthetic container");
    expect(image.payload_offset == 256 && image.payload_bytes == 256,
           "preserve checked payload extent");
    expect(image.part_count == (kind == FirmwareKind::dmcub_3_2 ? 1u : 2u),
           "retain engine-specific payloads");

    for (size_t length = 0; length < bytes.size(); ++length) {
        image.version = 0xaabbccdd;
        expect(rx7900::hardware::validate_firmware(ByteView(bytes.data(), length),
            kind, image) != FirmwareStatus::success, "reject every truncated container");
        expect(image.version == 0xaabbccdd, "truncation is transactional");
    }

    for (const size_t field : {size_t{0}, size_t{4}, size_t{20}, size_t{24}}) {
        bytes = fixture(kind);
        put(bytes, field, UINT32_MAX);
        reject(bytes, kind, "reject overflowing file/header/payload field");
    }
    bytes = fixture(kind);
    put(bytes, 8, 0x00010002);
    reject(bytes, kind, "reject unsupported header minor version");
    bytes = fixture(kind);
    put(bytes, 12, 0xffff);
    reject(bytes, kind, "reject wrong IP family");
    bytes = fixture(kind);
    put(bytes, 24, 16);
    reject(bytes, kind, "payload cannot overlap header");
    bytes = fixture(kind);
    put(bytes, 20, 255);
    reject(bytes, kind, "payload must be dword sized");
}

void check_parts() {
    auto bytes = fixture(FirmwareKind::sdma_6_0);
    put(bytes, 48, 300);
    reject(bytes, FirmwareKind::sdma_6_0, "SDMA threads cannot overlap");
    bytes = fixture(FirmwareKind::sdma_6_0);
    put(bytes, 44, UINT32_MAX);
    reject(bytes, FirmwareKind::sdma_6_0, "jump table extent cannot overflow");
    bytes = fixture(FirmwareKind::sdma_6_0);
    put(bytes, 48, 0xfffffffcu);
    reject(bytes, FirmwareKind::sdma_6_0, "control thread offset is a bounded file offset");
    bytes = fixture(FirmwareKind::psp_13_0);
    put(bytes, 32, UINT32_MAX);
    reject(bytes, FirmwareKind::psp_13_0, "PSP descriptor count is bounded");
    bytes = fixture(FirmwareKind::psp_13_0);
    put(bytes, 52, 2);
    reject(bytes, FirmwareKind::psp_13_0, "duplicate PSP types are rejected");
    bytes = fixture(FirmwareKind::psp_13_0);
    put(bytes, 60, 64);
    reject(bytes, FirmwareKind::psp_13_0, "PSP payloads cannot overlap");
    bytes = fixture(FirmwareKind::psp_13_0);
    put(bytes, 60, UINT32_MAX);
    reject(bytes, FirmwareKind::psp_13_0, "PSP relative offset cannot overflow");
    bytes = fixture(FirmwareKind::psp_13_0);
    put(bytes, 52, 3);
    reject(bytes, FirmwareKind::psp_13_0, "PSP requires SOS and system images");
    bytes = fixture(FirmwareKind::dmcub_3_2);
    put(bytes, 32, 260);
    reject(bytes, FirmwareKind::dmcub_3_2, "DMCUB instructions must fit payload");
    bytes = fixture(FirmwareKind::dmcub_3_2);
    put(bytes, 36, UINT32_MAX);
    reject(bytes, FirmwareKind::dmcub_3_2, "DMCUB BSS has an explicit bound");
    FirmwareImage image{};
    expect(rx7900::hardware::validate_firmware(ByteView(nullptr, 1000),
        FirmwareKind::sdma_6_0, image) == FirmwareStatus::truncated_input,
        "null firmware never dereferenced");
}

void check_private_file(const char* path, FirmwareKind kind) {
    std::vector<uint8_t> bytes(rx7900::hardware::maximum_firmware_bytes);
    const bool loaded = rx7900::tools::read_bounded_file(path, bytes);
    expect(loaded, "read private host firmware fixture");

    if (!loaded) {
        return;
    }

    FirmwareImage image{};
    const auto status = rx7900::hardware::validate_firmware(
        ByteView(bytes.data(), bytes.size()), kind, image);
    expect(status == FirmwareStatus::success, "real firmware container validates");
    std::printf("private firmware %s: status=%u parts=%u\n",
        rx7900::hardware::firmware_filename(kind), static_cast<unsigned>(status),
        static_cast<unsigned>(image.part_count));
}

} // namespace

int main(int argc, char** argv) {
    check_layout(FirmwareKind::sdma_6_0);
    check_layout(FirmwareKind::psp_13_0);
    check_layout(FirmwareKind::dmcub_3_2);
    check_parts();

    if (argc == 4) {
        check_private_file(argv[1], FirmwareKind::sdma_6_0);
        check_private_file(argv[2], FirmwareKind::psp_13_0);
        check_private_file(argv[3], FirmwareKind::dmcub_3_2);
    } else if (argc != 1) {
        return 2;
    }

    std::printf("firmware: %u checks, %u failures; no GPU loads\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

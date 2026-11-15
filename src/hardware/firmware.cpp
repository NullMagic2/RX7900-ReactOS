/*
Module: rx7900.hardware.firmware
Purpose: Decode firmware headers through checked binary reads.
Created: 2026-10-21
Architecture: Linux v6.12 amdgpu_ucode.h/amdgpu_psp.c field adaptation.
Copyright 2012 Advanced Micro Devices, Inc.
Permission notice and pinned sources: licenses/AMDGPU-FIRMWARE.txt.
*/
#include "firmware.hpp"

namespace rx7900::hardware {
namespace {

constexpr uint32_t common_header_bytes = 32;
constexpr uint32_t sdma_header_bytes = 64;
constexpr uint32_t psp_header_bytes = 36;
constexpr uint32_t dmcub_header_bytes = 40;
constexpr uint32_t psp_descriptor_bytes = 16;

uint32_t u32(ByteView bytes, size_t offset) noexcept {
    uint32_t value = 0;
    bytes.read_u32(offset, value);
    return value;
}

uint16_t u16(ByteView bytes, size_t offset) noexcept {
    uint16_t value = 0;
    bytes.read_u16(offset, value);
    return value;
}

bool overlaps(uint32_t start, uint32_t bytes, const FirmwarePart& other) noexcept {
    return start < static_cast<uint64_t>(other.offset) + other.bytes &&
           other.offset < static_cast<uint64_t>(start) + bytes;
}

bool append_part(ByteView input, FirmwareImage& image, uint32_t identifier,
                 uint32_t version, uint32_t offset, uint32_t bytes) noexcept {
    if (image.part_count >= maximum_firmware_parts || bytes == 0 ||
        ((offset | bytes) & 3u) != 0 || offset < image.payload_offset ||
        bytes > image.payload_bytes ||
        offset - image.payload_offset > image.payload_bytes - bytes ||
        !input.contains(offset, bytes)) {
        return false;
    }

    for (uint32_t index = 0; index < image.part_count; ++index) {
        if (image.parts[index].identifier == identifier ||
            overlaps(offset, bytes, image.parts[index])) {
            return false;
        }
    }

    image.parts[image.part_count++] = {identifier, version, offset, bytes};
    return true;
}

FirmwareStatus parse_sdma(ByteView input, FirmwareImage& image) noexcept {
    const uint32_t context_size = u32(input, 36);
    const uint32_t control_size = u32(input, 52);
    const uint32_t context_jump = u32(input, 40);
    const uint32_t context_jump_size = u32(input, 44);
    const uint32_t control_jump = u32(input, 56);
    const uint32_t control_jump_size = u32(input, 60);

    if (((context_jump | context_jump_size | control_jump | control_jump_size) & 3u) != 0 ||
        context_jump > context_size || context_jump_size > context_size - context_jump ||
        control_jump > control_size || control_jump_size > control_size - control_jump ||
        !append_part(input, image, 0, image.version, image.payload_offset, context_size) ||
        !append_part(input, image, 1, image.version, u32(input, 48), control_size)) {
        return FirmwareStatus::malformed_image;
    }

    return FirmwareStatus::success;
}

FirmwareStatus parse_psp(ByteView input, FirmwareImage& image) noexcept {
    const uint32_t count = u32(input, 32);

    if (count == 0 || count > maximum_firmware_parts ||
        image.payload_offset < psp_header_bytes ||
        count > (image.payload_offset - psp_header_bytes) / psp_descriptor_bytes) {
        return FirmwareStatus::malformed_image;
    }

    bool has_sos = false;
    bool has_system = false;

    for (uint32_t index = 0; index < count; ++index) {
        const uint32_t descriptor = psp_header_bytes + index * psp_descriptor_bytes;
        const uint32_t type = u32(input, descriptor);
        const uint32_t relative_offset = u32(input, descriptor + 8);

        if (type == 0 || type >= maximum_firmware_parts ||
            relative_offset > image.payload_bytes ||
            !append_part(input, image, type, u32(input, descriptor + 4),
                         image.payload_offset + relative_offset,
                         u32(input, descriptor + 12))) {
            return FirmwareStatus::malformed_image;
        }

        has_sos = has_sos || type == 1;
        has_system = has_system || type == 2;
    }

    return has_sos && has_system ? FirmwareStatus::success : FirmwareStatus::malformed_image;
}

} // namespace

const char* firmware_filename(FirmwareKind kind) noexcept {
    switch (kind) {
    case FirmwareKind::sdma_6_0: return "sdma_6_0_0.bin";
    case FirmwareKind::psp_13_0: return "psp_13_0_0_sos.bin";
    case FirmwareKind::dmcub_3_2: return "dcn_3_2_0_dmcub.bin";
    default: return nullptr;
    }
}

FirmwareStatus validate_firmware(ByteView input, FirmwareKind kind,
                                 FirmwareImage& output) noexcept {
    if (!input.contains(0, common_header_bytes)) {
        return FirmwareStatus::truncated_input;
    }

    const uint32_t file_bytes = u32(input, 0);
    const uint32_t header_bytes = u32(input, 4);
    const uint16_t major = u16(input, 8);
    const uint16_t minor = u16(input, 10);
    const uint16_t ip_major = u16(input, 12);
    const uint16_t ip_minor = u16(input, 14);
    uint32_t required_header = 0;
    uint16_t expected_ip_major = 0;
    uint16_t expected_ip_minor = 0;
    uint16_t expected_major = 2;

    switch (kind) {
    case FirmwareKind::sdma_6_0:
        required_header = sdma_header_bytes;
        expected_ip_major = 6;
        break;
    case FirmwareKind::psp_13_0:
        required_header = psp_header_bytes;
        expected_ip_major = 13;
        break;
    case FirmwareKind::dmcub_3_2:
        required_header = dmcub_header_bytes;
        expected_ip_major = 3;
        expected_ip_minor = 2;
        expected_major = 1;
        break;
    default:
        return FirmwareStatus::unsupported_version;
    }

    if (major != expected_major || minor != 0) {
        return FirmwareStatus::unsupported_version;
    }
    if (ip_major != expected_ip_major || ip_minor != expected_ip_minor) {
        return FirmwareStatus::wrong_ip;
    }
    if (file_bytes > maximum_firmware_bytes || file_bytes != input.size() ||
        header_bytes < required_header || !input.contains(0, header_bytes)) {
        return FirmwareStatus::malformed_image;
    }

    FirmwareImage image{};
    image.kind = kind;
    image.version = u32(input, 16);
    image.payload_bytes = u32(input, 20);
    image.payload_offset = u32(input, 24);
    image.declared_crc32 = u32(input, 28);

    if (image.payload_bytes == 0 || image.payload_offset < header_bytes ||
        ((image.payload_offset | image.payload_bytes) & 3u) != 0 ||
        !input.contains(image.payload_offset, image.payload_bytes)) {
        return FirmwareStatus::malformed_image;
    }

    FirmwareStatus status = FirmwareStatus::malformed_image;

    if (kind == FirmwareKind::sdma_6_0) {
        status = parse_sdma(input, image);
    } else if (kind == FirmwareKind::psp_13_0) {
        status = parse_psp(input, image);
    } else {
        const uint32_t instruction_bytes = u32(input, 32);
        image.dmcub_bss_bytes = u32(input, 36);

        if (image.dmcub_bss_bytes <= maximum_firmware_bytes &&
            append_part(input, image, 0, image.version, image.payload_offset,
                        instruction_bytes)) {
            status = FirmwareStatus::success;
        }
    }

    if (status == FirmwareStatus::success) {
        output = image;
    }

    return status;
}

} // namespace rx7900::hardware

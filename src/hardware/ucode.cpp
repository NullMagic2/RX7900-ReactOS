/*
Module: rx7900.hardware.ucode
Purpose: Decode PSP payload ranges from GC/SMU/SDMA/DMCUB firmware headers.
Created: 2026-10-21
Architecture: Checked offsets into borrowed files; staging belongs to psp_ring.
Copyright 2012, 2014, 2019 Advanced Micro Devices, Inc.; Copyright 2008 Red Hat
Inc., Copyright 2009 Jerome Glisse. Adapted Linux v6.12 amdgpu_ucode.c/.h,
amdgpu_rlc.c, amdgpu_mes.c, amdgpu_gfx.c, amdgpu_sdma.c, gfx_v11_0.c, imu_v11_0.c,
amdgpu_psp.c (psp_get_fw_type, parse_ta_v2_microcode, parse_ta_bin_descriptor) and
psp_gfx_if.h. Notice: licenses/AMDGPU-FIRMWARE.txt.
*/
#include "ucode.hpp"

namespace rx7900::hardware {
namespace {

constexpr uint32_t common_header_bytes = 32;
constexpr uint32_t gfx_v2_header_bytes = 60;
constexpr uint32_t mes_v1_header_bytes = 72;
constexpr uint32_t imu_v1_header_bytes = 48;
constexpr uint32_t smc_v2_header_bytes = 44;
// Linux structure sizes; the real gc_11_0_0_rlc.bin (v2.3) declares 0xcc bytes.
constexpr uint32_t rlc_header_bytes[]{104, 156, 172, 204, 244};

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

struct Expected final {
    uint16_t major;
    uint16_t minimum_minor;
    uint16_t maximum_minor;
    uint16_t ip_major;
    uint16_t ip_minor;
};

UcodeStatus check_common(ByteView file, Expected expected, uint32_t& minor) noexcept {
    if (!file.contains(0, common_header_bytes)) {
        return UcodeStatus::truncated_input;
    }

    minor = u16(file, 10);

    if (u16(file, 8) != expected.major || minor < expected.minimum_minor ||
        minor > expected.maximum_minor) {
        return UcodeStatus::unsupported_version;
    }

    if (u16(file, 12) != expected.ip_major || u16(file, 14) != expected.ip_minor) {
        return UcodeStatus::wrong_ip;
    }

    if (u32(file, 0) != file.size() || file.size() > maximum_firmware_bytes) {
        return UcodeStatus::malformed_image;
    }

    return UcodeStatus::success;
}

} // namespace

const char* ucode_filename(UcodeSource source, uint8_t pci_revision, bool fallback) noexcept {
    switch (source) {
    case UcodeSource::pfp: return "gc_11_0_0_pfp.bin";
    case UcodeSource::me: return "gc_11_0_0_me.bin";
    case UcodeSource::mec: return "gc_11_0_0_mec.bin";
    case UcodeSource::rlc: return pci_revision == 0xce ? "gc_11_0_0_rlc_1.bin" : "gc_11_0_0_rlc.bin";
    case UcodeSource::mes: return fallback ? "gc_11_0_0_mes.bin" : "gc_11_0_0_mes_2.bin";
    case UcodeSource::mes_kiq: return "gc_11_0_0_mes1.bin";
    case UcodeSource::imu: return "gc_11_0_0_imu.bin";
    case UcodeSource::sdma: return firmware_filename(FirmwareKind::sdma_6_0);
    case UcodeSource::smu: return "smu_13_0_0.bin";
    case UcodeSource::dmcub: return firmware_filename(FirmwareKind::dmcub_3_2);
    default: return nullptr;
    }
}

UcodeStatus rs64_start_address(ByteView file, uint32_t& low, uint32_t& high) noexcept {
    uint32_t minor = 0;
    const auto status = check_common(file, {2, 0, 0, 11, 0}, minor);

    if (status != UcodeStatus::success) {
        return status;
    }

    if (!file.contains(0, gfx_v2_header_bytes)) {
        return UcodeStatus::truncated_input;
    }

    low = u32(file, 52);
    high = u32(file, 56);
    return UcodeStatus::success;
}

UcodeStatus parse_ta_descriptors(ByteView file, TaDescriptors& output) noexcept {
    constexpr uint32_t count_offset = 32;
    constexpr uint32_t descriptor_start = 36;
    constexpr uint32_t descriptor_bytes = 16;
    uint32_t minor = 0;
    const auto status = check_common(file, {2, 0, 0, 13, 0}, minor);

    if (status != UcodeStatus::success) {
        return status;
    }

    if (!file.contains(0, descriptor_start)) {
        return UcodeStatus::truncated_input;
    }

    const uint32_t array_offset = u32(file, 24);
    const uint32_t array_bytes = u32(file, 20);
    const uint32_t count = u32(file, count_offset);

    // count < 26 bounds the multiplication below.
    if (count >= ta_max_packaged_binaries || !file.contains(array_offset, array_bytes) ||
        !file.contains(descriptor_start, count * descriptor_bytes) ||
        descriptor_start + count * descriptor_bytes > array_offset) {
        return UcodeStatus::malformed_image;
    }

    TaDescriptors candidate{};

    for (uint32_t index = 0; index < count; ++index) {
        const uint32_t descriptor = descriptor_start + index * descriptor_bytes;
        const uint32_t type = u32(file, descriptor);
        const uint32_t relative = u32(file, descriptor + 8);
        const uint32_t size = u32(file, descriptor + 12);

        if (type == 0 || type >= ta_fw_type::count) {
            continue;
        }

        if (relative > array_bytes || size > array_bytes - relative) {
            return UcodeStatus::malformed_image;
        }

        candidate.binary[type] = {true, u32(file, descriptor + 4), array_offset + relative, size};
    }

    output = candidate;
    return UcodeStatus::success;
}

UcodeStatus find_ta_asd(ByteView file, uint32_t& offset, uint32_t& bytes) noexcept {
    TaDescriptors descriptors{};
    const auto status = parse_ta_descriptors(file, descriptors);

    if (status != UcodeStatus::success) {
        return status;
    }

    const TaBinary asd = descriptors.get(ta_fw_type::asd);

    if (!asd.present || asd.bytes == 0) {
        return UcodeStatus::malformed_image;
    }

    offset = asd.offset;
    bytes = asd.bytes;
    return UcodeStatus::success;
}

UcodeStatus UcodeCatalog::parse(UcodeSource source, ByteView file, Pending& pending) const noexcept {
    bool malformed = false;
    const auto append = [&](uint8_t id, uint8_t type, uint64_t offset, uint32_t bytes,
                            bool optional) noexcept {
        if (bytes == 0 && optional) {
            return;
        }

        if (bytes == 0 || offset > UINT32_MAX || !file.contains(static_cast<size_t>(offset), bytes) ||
            pending.count >= sizeof(pending.entries) / sizeof(pending.entries[0])) {
            malformed = true;
            return;
        }

        pending.entries[pending.count++] = {id, type, source, static_cast<uint32_t>(offset), bytes};
    };
    uint32_t minor = 0;
    UcodeStatus status = UcodeStatus::success;

    switch (source) {
    case UcodeSource::pfp:
    case UcodeSource::me:
    case UcodeSource::mec: {
        // RS64 is selected by the PFP header version 2.0 (gfx_v11_0_init_microcode).
        status = check_common(file, {2, 0, 0, 11, 0}, minor);

        if (status != UcodeStatus::success) {
            return status;
        }

        if (!file.contains(0, gfx_v2_header_bytes)) {
            return UcodeStatus::truncated_input;
        }

        const uint32_t code_offset = u32(file, 24);
        const uint32_t code_bytes = u32(file, 36);
        const uint32_t data_bytes = u32(file, 44);
        const uint32_t data_offset = u32(file, 48);

        if (source == UcodeSource::pfp) {
            append(15, 87, code_offset, code_bytes, false);
            append(18, 90, data_offset, data_bytes, false);
            append(19, 91, data_offset, data_bytes, false);
        } else if (source == UcodeSource::me) {
            append(16, 88, code_offset, code_bytes, false);
            append(20, 92, data_offset, data_bytes, false);
            append(21, 93, data_offset, data_bytes, false);
        } else {
            append(17, 89, code_offset, code_bytes, false);
            append(22, 94, data_offset, data_bytes, false);
            append(23, 95, data_offset, data_bytes, false);
            append(24, 96, data_offset, data_bytes, false);
            append(25, 97, data_offset, data_bytes, false);
        }

        break;
    }
    case UcodeSource::rlc: {
        status = check_common(file, {2, 0, 4, 11, 0}, minor);

        if (status != UcodeStatus::success) {
            return status;
        }

        // Linux reads fields by structure layout, independent of header_size_bytes.
        if (!file.contains(0, rlc_header_bytes[minor])) {
            return UcodeStatus::truncated_input;
        }

        if (minor >= 1) {
            append(41, 22, u32(file, 120), u32(file, 116), true);
            append(42, 20, u32(file, 136), u32(file, 132), true);
            append(43, 21, u32(file, 152), u32(file, 148), true);
        }

        if (minor >= 2) {
            append(44, 26, u32(file, 160), u32(file, 156), true);
            append(45, 48, u32(file, 168), u32(file, 164), true);
        }

        if (minor == 3) {
            append(46, 25, u32(file, 184), u32(file, 180), true);
            append(47, 7, u32(file, 200), u32(file, 196), true);
        }

        if (minor == 4) {
            append(36, 27, u32(file, 208), u32(file, 204), true);
            append(37, 28, u32(file, 216), u32(file, 212), true);
            append(38, 29, u32(file, 224), u32(file, 220), true);
            append(39, 65, u32(file, 232), u32(file, 228), true);
            append(40, 66, u32(file, 240), u32(file, 236), true);
        }

        append(ucode_id::rlc_g, 8, u32(file, 24), u32(file, 20), false);
        break;
    }
    case UcodeSource::mes:
    case UcodeSource::mes_kiq: {
        status = check_common(file, {1, 0, 0, 11, 0}, minor);

        if (status != UcodeStatus::success) {
            return status;
        }

        if (!file.contains(0, mes_v1_header_bytes)) {
            return UcodeStatus::truncated_input;
        }

        const bool scheduler = source == UcodeSource::mes;
        append(scheduler ? 30 : 32, scheduler ? 33 : 81, u32(file, 40), u32(file, 36), false);
        append(scheduler ? 31 : 33, scheduler ? 34 : 82, u32(file, 52), u32(file, 48), false);
        break;
    }
    case UcodeSource::imu: {
        status = check_common(file, {1, 0, 0, 11, 0}, minor);

        if (status != UcodeStatus::success) {
            return status;
        }

        if (!file.contains(0, imu_v1_header_bytes)) {
            return UcodeStatus::truncated_input;
        }

        // Linux uses the array offset and IRAM size, not the per-section offsets.
        const uint64_t base = u32(file, 24);
        const uint32_t iram_bytes = u32(file, 32);
        append(34, 68, base, iram_bytes, false);
        append(35, 69, base + iram_bytes, u32(file, 40), false);
        break;
    }
    case UcodeSource::smu:
        status = check_common(file, {2, 0, 1, 13, 0}, minor);

        if (status != UcodeStatus::success) {
            return status;
        }

        if (!file.contains(0, smc_v2_header_bytes)) {
            return UcodeStatus::truncated_input;
        }

        append(ucode_id::smc, 18, u32(file, 24), u32(file, 20), false);
        break;
    case UcodeSource::sdma:
    case UcodeSource::dmcub: {
        FirmwareImage image{};
        const bool sdma = source == UcodeSource::sdma;
        const auto parsed = validate_firmware(file, sdma ? FirmwareKind::sdma_6_0 :
                                                           FirmwareKind::dmcub_3_2, image);

        switch (parsed) {
        case FirmwareStatus::success: break;
        case FirmwareStatus::truncated_input: return UcodeStatus::truncated_input;
        case FirmwareStatus::unsupported_version: return UcodeStatus::unsupported_version;
        case FirmwareStatus::wrong_ip: return UcodeStatus::wrong_ip;
        default: return UcodeStatus::malformed_image;
        }

        for (uint32_t index = 0; index < image.part_count; ++index) {
            const auto& part = image.parts[index];

            if (sdma && part.identifier <= 1) {
                append(part.identifier == 0 ? 9 : 10, part.identifier == 0 ? 71 : 72,
                       part.offset, part.bytes, false);
            } else if (!sdma && part.identifier == 0) {
                append(61, 51, part.offset, part.bytes, false);
            }
        }

        if (pending.count != (sdma ? 2u : 1u)) {
            malformed = true;
        }

        break;
    }
    default:
        return UcodeStatus::invalid_argument;
    }

    return malformed ? UcodeStatus::malformed_image : UcodeStatus::success;
}

UcodeStatus UcodeCatalog::add(UcodeSource source, ByteView file) noexcept {
    const size_t index = static_cast<size_t>(source);

    if (index >= ucode_source_count || file.size() == 0) {
        return UcodeStatus::invalid_argument;
    }

    if (present_[index]) {
        return UcodeStatus::duplicate_source;
    }

    Pending pending{};
    const auto status = parse(source, file, pending);

    if (status != UcodeStatus::success) {
        return status;
    }

    if (pending.count > maximum_ucode_entries - count_) {
        return UcodeStatus::capacity_exceeded;
    }

    for (size_t candidate = 0; candidate < pending.count; ++candidate) {
        UcodeEntry ignored{};

        if (find(pending.entries[candidate].linux_id, ignored)) {
            return UcodeStatus::malformed_image;
        }
    }

    for (size_t candidate = 0; candidate < pending.count; ++candidate) {
        const UcodeEntry value = pending.entries[candidate];
        size_t position = count_;

        while (position > 0 && entries_[position - 1].linux_id > value.linux_id) {
            entries_[position] = entries_[position - 1];
            --position;
        }

        entries_[position] = value;
        ++count_;
    }

    files_[index] = file;
    present_[index] = true;
    return UcodeStatus::success;
}

void UcodeCatalog::reset() noexcept {
    for (size_t index = 0; index < ucode_source_count; ++index) {
        files_[index] = ByteView(nullptr, 0);
        present_[index] = false;
    }

    for (auto& value : entries_) {
        value = {};
    }

    count_ = 0;
}

bool UcodeCatalog::has(UcodeSource source) const noexcept {
    const size_t index = static_cast<size_t>(source);
    return index < ucode_source_count && present_[index];
}

UcodeEntry UcodeCatalog::entry(size_t index) const noexcept {
    return index < count_ ? entries_[index] : UcodeEntry{};
}

bool UcodeCatalog::payload(const UcodeEntry& value, ByteView& output) const noexcept {
    const size_t index = static_cast<size_t>(value.source);

    return index < ucode_source_count && present_[index] && value.bytes != 0 &&
           files_[index].subview(value.offset, value.bytes, output);
}

ByteView UcodeCatalog::file(UcodeSource source) const noexcept {
    const size_t index = static_cast<size_t>(source);
    return index < ucode_source_count && present_[index] ? files_[index] : ByteView(nullptr, 0);
}

bool UcodeCatalog::find(uint8_t linux_id, UcodeEntry& output) const noexcept {
    for (size_t index = 0; index < count_; ++index) {
        if (entries_[index].linux_id == linux_id) {
            output = entries_[index];
            return true;
        }
    }

    return false;
}

} // namespace rx7900::hardware

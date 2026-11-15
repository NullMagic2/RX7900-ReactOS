/*
Module: rx7900.hardware.vram_discovery
Purpose: Read the discovery reservation through the indirect VRAM window.
Created: 2026-10-21
Architecture: Fixed SOC15 indices used before any discovered base exists.
Copyright 2018 Advanced Micro Devices, Inc.; Copyright 2008 Advanced Micro Devices,
Inc., Copyright 2008 Red Hat Inc., Copyright 2009 Jerome Glisse.
Adapted Linux v6.12 amdgpu_discovery.c/.h and amdgpu_device.c
(amdgpu_device_mm_access). Notice and provenance: licenses/AMDGPU-FIRMWARE.txt.
*/
#include "vram_discovery.hpp"

namespace rx7900::hardware {
namespace {

constexpr uint32_t mm_index = 0x0;
constexpr uint32_t mm_data = 0x1;
constexpr uint32_t mm_index_hi = 0x6;
constexpr uint32_t rcc_config_memsize = 0xde3;
constexpr uint32_t mp0_c2pmsg_33 = 0x16061;
constexpr uint32_t ifwi_ready_bit = 0x80000000u;
constexpr uint32_t ifwi_poll_iterations = 1000;
constexpr uint32_t ifwi_poll_microseconds = 1000;
constexpr uint32_t discovery_signature = 0x28211407;
constexpr uint32_t discovery_header_bytes = 60;

VramDiscoveryStatus finish(VramDiscoveryReport& report, VramDiscoveryStatus status) noexcept {
    report.status = status;
    return status;
}

} // namespace

// amdgpu_device_mm_access read loop: MM_INDEX carries bits 30:0 plus the aperture
// select bit, MM_INDEX_HI the bits above 31 whenever they change. window_high
// starts at UINT32_MAX so the first access always programs MM_INDEX_HI.
bool read_vram_block(const RegisterIo& io, uint64_t position, uint8_t* output, uint32_t bytes,
                     uint32_t& window_high) noexcept {
    if (output == nullptr || (bytes & 3u) != 0 || (position & 3u) != 0 ||
        !register_io_valid(io)) {
        return false;
    }

    for (uint32_t offset = 0; offset < bytes; offset += sizeof(uint32_t)) {
        const uint64_t at = position + offset;
        const uint32_t high = static_cast<uint32_t>(at >> 31);
        uint32_t value = 0;

        if (!write_ip_register(io, 0, mm_index, static_cast<uint32_t>(at) | 0x80000000u)) {
            return false;
        }

        if (high != window_high) {
            if (!write_ip_register(io, 0, mm_index_hi, high)) {
                return false;
            }

            window_high = high;
        }

        // VRAM content may legitimately be all ones; only MMIO failure aborts.
        if (!read_ip_register(io, 0, mm_data, value)) {
            return false;
        }

        // The caller owns bytes writable bytes at output (dword multiple checked).
        for (uint32_t byte = 0; byte < sizeof(value); ++byte) {
            output[offset + byte] = static_cast<uint8_t>(value >> (byte * 8));
        }
    }

    return true;
}

bool write_vram_block(const RegisterIo& io, uint64_t position, ByteView input,
                      uint32_t& window_high) noexcept {
    if ((input.size() & 3u) != 0 || input.size() > UINT32_MAX || (position & 3u) != 0 ||
        !register_io_valid(io)) {
        return false;
    }

    for (uint32_t offset = 0; offset < input.size(); offset += sizeof(uint32_t)) {
        const uint64_t at = position + offset;
        const uint32_t high = static_cast<uint32_t>(at >> 31);
        uint32_t value = 0;

        if (!input.read_u32(offset, value) ||
            !write_ip_register(io, 0, mm_index, static_cast<uint32_t>(at) | 0x80000000u)) {
            return false;
        }

        if (high != window_high) {
            if (!write_ip_register(io, 0, mm_index_hi, high)) {
                return false;
            }

            window_high = high;
        }

        if (!write_ip_register(io, 0, mm_data, value)) {
            return false;
        }
    }

    return true;
}

VramDiscoveryStatus read_vram_discovery(const RegisterIo& io, DiscoveryBinary& output,
                                        VramDiscoveryReport& report) noexcept {
    report = {};
    output.acquired_ = false;

    for (auto& byte : output.bytes_) {
        byte = 0;
    }

    uint32_t probe = 0;

    if (!register_io_valid(io) || !discovery_register_offset(0, mp0_c2pmsg_33,
        io.aperture_bytes, probe)) {
        return finish(report, VramDiscoveryStatus::invalid_argument);
    }

    for (uint32_t iteration = 0; iteration < ifwi_poll_iterations; ++iteration) {
        if (!read_ip_register(io, 0, mp0_c2pmsg_33, report.ifwi_status) ||
            report.ifwi_status == UINT32_MAX) {
            return finish(report, VramDiscoveryStatus::mmio_failure);
        }

        if ((report.ifwi_status & ifwi_ready_bit) != 0) {
            report.ifwi_ready = true;
            break;
        }

        io.delay_microseconds(io.context, ifwi_poll_microseconds);
    }

    if (!read_ip_register(io, 0, rcc_config_memsize, report.memsize_mib) ||
        report.memsize_mib == UINT32_MAX) {
        return finish(report, VramDiscoveryStatus::mmio_failure);
    }

    // Linux falls back to an ACPI system-memory reservation only for zero VRAM
    // (APUs). A dGPU without reported VRAM is unsupported, not reinterpreted.
    report.vram_bytes = static_cast<uint64_t>(report.memsize_mib) << 20;

    if (report.vram_bytes < discovery_tmr_offset) {
        return finish(report, VramDiscoveryStatus::no_vram);
    }

    report.source_offset = report.vram_bytes - discovery_tmr_offset;
    uint32_t window_high = UINT32_MAX;

    if (!read_vram_block(io, report.source_offset, output.bytes_, discovery_tmr_bytes, window_high)) {
        return finish(report, VramDiscoveryStatus::mmio_failure);
    }

    const ByteView candidate(output.bytes_, sizeof(output.bytes_));
    uint32_t signature = 0;
    uint16_t binary_bytes = 0;

    if (!candidate.read_u32(0, signature) || !candidate.read_u16(10, binary_bytes) ||
        signature != discovery_signature || binary_bytes < discovery_header_bytes ||
        binary_bytes > discovery_tmr_bytes) {
        return finish(report, VramDiscoveryStatus::invalid_binary);
    }

    output.acquired_ = true;
    return finish(report, VramDiscoveryStatus::acquired);
}

VramDiscoveryStatus read_psp_runtime_database(const RegisterIo& io, uint64_t vram_bytes,
                                              PspRuntimeDatabase& output) noexcept {
    constexpr uint16_t cookie = 0x0ed5;
    constexpr uint32_t directory_bytes = 4 + 64 * 8;
    constexpr uint16_t maximum_entries = 64;
    constexpr uint32_t type_boot_config = 5;
    constexpr uint32_t type_scpm = 6;
    output = {};

    if (!register_io_valid(io) || vram_bytes < psp_runtime_db_offset) {
        return VramDiscoveryStatus::invalid_argument;
    }

    const uint64_t header = vram_bytes - psp_runtime_db_offset;
    uint8_t bytes[4 + directory_bytes]{};
    uint32_t window_high = UINT32_MAX;

    if (!read_vram_block(io, header, bytes, sizeof(bytes), window_high)) {
        return VramDiscoveryStatus::mmio_failure;
    }

    const ByteView database(bytes, sizeof(bytes));
    uint16_t found_cookie = 0;
    uint16_t count = 0;
    database.read_u16(0, found_cookie);
    database.read_u16(4, count);

    if (found_cookie != cookie || count >= maximum_entries) {
        return VramDiscoveryStatus::acquired;
    }

    output.present = true;

    const auto lookup = [&](uint32_t type, uint32_t minimum, bool& valid, uint32_t& value) noexcept {
        for (uint16_t index = 0; index < count; ++index) {
            const size_t entry = 8 + static_cast<size_t>(index) * 8;
            uint32_t entry_type = 0;
            uint16_t offset = 0;
            uint16_t size = 0;
            database.read_u32(entry, entry_type);
            database.read_u16(entry + 4, offset);
            database.read_u16(entry + 6, size);

            if (entry_type != type) {
                continue;
            }

            uint8_t field[4]{};

            // Linux stops at the first entry of this type; an undersized entry is
            // "not found". Offsets are header-relative and must be dword aligned.
            if (size < minimum || (offset & 3u) != 0) {
                return true;
            }

            if (!read_vram_block(io, header + offset, field, sizeof(field), window_high)) {
                return false;
            }

            ByteView(field, sizeof(field)).read_u32(0, value);
            valid = true;
            return true;
        }

        return true;
    };

    if (!lookup(type_boot_config, 8, output.boot_config_valid, output.boot_config) ||
        !lookup(type_scpm, 4, output.scpm_valid, output.scpm_status)) {
        return VramDiscoveryStatus::mmio_failure;
    }

    return VramDiscoveryStatus::acquired;
}

} // namespace rx7900::hardware

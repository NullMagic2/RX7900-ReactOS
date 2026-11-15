/*
Module: rx7900.hardware.firmware
Purpose: Validate AMD firmware containers before any staging or PSP operation.
Created: 2026-10-21
Architecture: Checked Linux-derived metadata; never authenticates or loads a GPU.
*/
#pragma once

#include "byte_view.hpp"

namespace rx7900::hardware {

enum class FirmwareKind { sdma_6_0, psp_13_0, dmcub_3_2 };
enum class FirmwareStatus {
    success, truncated_input, malformed_image, unsupported_version, wrong_ip
};

constexpr uint32_t maximum_firmware_bytes = 32u * 1024u * 1024u;
constexpr uint32_t maximum_firmware_parts = 12;

struct FirmwarePart final {
    uint32_t identifier;
    uint32_t version;
    uint32_t offset;
    uint32_t bytes;
};

// Offset-only metadata has no borrowed pointers. Keep the validated source
// immutable for its entire staging lifetime. SDMA IDs 0/1 mean context/control;
// PSP IDs retain the Linux psp_fw_type numeric values; DMCUB ID 0 is instruction.
struct FirmwareImage final {
    FirmwareKind kind{};
    uint32_t version = 0;
    uint32_t payload_offset = 0;
    uint32_t payload_bytes = 0;
    uint32_t declared_crc32 = 0;
    uint32_t part_count = 0;
    uint32_t dmcub_bss_bytes = 0;
    FirmwarePart parts[maximum_firmware_parts]{};
};

// Resident immutable input, any IRQL. Output unchanged on failure. Supports
// SDMA header 2.0/IP6.0, PSP SOS header2.0/IP13.0 and DMCUB header1.0/IP3.2.
// Requires exact file length, complete headers, bounded nonoverlapping payloads.
// CRC metadata is retained, not verified: its packaging convention is not the
// standard CRC32 over the full payload. This is not cryptographic authentication.
// PSP must verify signed firmware before any engine is declared initialized.
FirmwareStatus validate_firmware(ByteView input, FirmwareKind kind,
                                 FirmwareImage& output) noexcept;

// Known filenames only; callers never concatenate an untrusted firmware path.
const char* firmware_filename(FirmwareKind kind) noexcept;

} // namespace rx7900::hardware

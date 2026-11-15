/*
Module: rx7900.hardware.ucode
Purpose: Catalog Navi31 PSP-loaded IP firmware payloads in Linux load order.
Created: 2026-10-21
Architecture: Borrowed immutable file views and checked offsets; no staging or MMIO.
firmware.hpp remains the SDMA/PSP/DMCUB container parser.
*/
#pragma once

#include "firmware.hpp"

namespace rx7900::hardware {

// One file per source. GC11.0.0 RS64 CP, RLC, MES scheduler/KIQ and IMU,
// SDMA6.0.0, SMU13.0.0 and DCN3.2 DMCUB, as selected by Linux v6.12 for PSP load.
enum class UcodeSource : uint8_t { pfp, me, mec, rlc, mes, mes_kiq, imu, sdma, smu, dmcub };
constexpr size_t ucode_source_count = 10;
constexpr size_t maximum_ucode_entries = 40;

// Linux AMDGPU_UCODE_ID values used as the load-order key, and the PSP
// GFX_FW_TYPE values that PSP receives. Only the subset Navi31 uses is named.
namespace ucode_id {
constexpr uint8_t rlc_g = 48;
constexpr uint8_t smc = 50;
} // namespace ucode_id

enum class UcodeStatus {
    success,
    invalid_argument,
    duplicate_source,
    truncated_input,
    malformed_image,
    unsupported_version,
    wrong_ip,
    capacity_exceeded
};

struct UcodeEntry final {
    uint8_t linux_id;
    uint8_t psp_type;
    UcodeSource source;
    uint32_t offset;
    uint32_t bytes;
};

// Fixed names below the native firmware directory; never caller-built paths.
// rlc_1 applies to PCI revision 0xCE exactly as Linux. MES scheduler first tries
// the _mes_2 image; fallback selects the original _mes image Linux retries.
const char* ucode_filename(UcodeSource source, uint8_t pci_revision, bool fallback) noexcept;

// psp_13_0_0_ta.bin: ta_firmware_header_v2_0 (common header, ta_fw_bin_count at
// 32, 16-byte psp_fw_bin_desc entries from 36) for IP 13.0.
constexpr const char* psp_ta_filename = "psp_13_0_0_ta.bin";

// Linux enum ta_fw_type (amdgpu_ucode.h).
namespace ta_fw_type {
constexpr uint32_t asd = 1;
constexpr uint32_t xgmi = 2;
constexpr uint32_t ras = 3;
constexpr uint32_t hdcp = 4;
constexpr uint32_t dtm = 5;
constexpr uint32_t rap = 6;
constexpr uint32_t securedisplay = 7;
constexpr uint32_t count = 8;
} // namespace ta_fw_type

// UCODE_MAX_PSP_PACKAGING, measured as 26 from the pinned amdgpu_ucode.h: a
// ta_fw_bin_count at or above it is refused by parse_ta_v2_microcode.
constexpr uint32_t ta_max_packaged_binaries = 26;

// One parse_ta_bin_descriptor result. offset is the file byte offset of the
// binary (descriptor offset_bytes plus ucode_array_offset_bytes). present means a
// descriptor of this type exists; Linux treats bytes == 0 as "not available".
struct TaBinary final {
    bool present = false;
    uint32_t fw_version = 0;
    uint32_t offset = 0;
    uint32_t bytes = 0;
};

// Indexed by ta_fw_type; index 0 (TA_FW_TYPE_UNKOWN) is never present.
struct TaDescriptors final {
    TaBinary binary[ta_fw_type::count]{};

    TaBinary get(uint32_t type) const noexcept {
        return type != 0 && type < ta_fw_type::count ? binary[type] : TaBinary{};
    }
};

// parse_ta_v2_microcode: descriptors in file order, a later descriptor of the same
// type replacing an earlier one, unknown types ignored (Linux only warns). Fails
// closed where Linux does not check: every known-type binary must lie inside the
// declared ucode array, which must lie inside the file, and the descriptor table
// must end before that array. Only header 2.0 for IP 13.0 is accepted (the v1
// layout is not used by PSP 13.0.0). Output unchanged on failure.
UcodeStatus parse_ta_descriptors(ByteView file, TaDescriptors& output) noexcept;

// gfx_firmware_header_v2_0 ucode_start_addr_lo/hi (gfx_v11_0_config_gfx_rs64) of
// a validated RS64 PFP/ME/MEC file. Outputs unchanged on failure.
UcodeStatus rs64_start_address(ByteView file, uint32_t& low, uint32_t& high) noexcept;
// The ASD binary of parse_ta_descriptors as file byte offset and size; an absent
// or zero-sized ASD is malformed_image. Outputs unchanged on failure.
UcodeStatus find_ta_asd(ByteView file, uint32_t& offset, uint32_t& bytes) noexcept;

// Entries stay sorted by Linux ucode ID, so iteration order is psp_load_non_psp_fw
// order. Payload views borrow the added file bytes: every added file must remain
// immutable and alive until the catalog is reset or destroyed. Zero-sized optional
// RLC lists are omitted (Linux fw_load_skip_check). add is all-or-nothing.
class UcodeCatalog final {
public:
    UcodeCatalog() noexcept = default;
    UcodeCatalog(const UcodeCatalog&) = delete;
    UcodeCatalog& operator=(const UcodeCatalog&) = delete;

    UcodeStatus add(UcodeSource source, ByteView file) noexcept;
    void reset() noexcept;
    bool has(UcodeSource source) const noexcept;
    size_t count() const noexcept { return count_; }
    // Index must be below count(); out-of-range returns an empty entry.
    UcodeEntry entry(size_t index) const noexcept;
    bool payload(const UcodeEntry& entry, ByteView& output) const noexcept;
    bool find(uint8_t linux_id, UcodeEntry& output) const noexcept;
    // Borrowed whole file of an added source, empty when absent.
    ByteView file(UcodeSource source) const noexcept;

private:
    struct Pending final {
        UcodeEntry entries[24]{};
        size_t count = 0;
    };

    UcodeStatus parse(UcodeSource source, ByteView file, Pending& pending) const noexcept;

    ByteView files_[ucode_source_count]{
        {nullptr, 0}, {nullptr, 0}, {nullptr, 0}, {nullptr, 0}, {nullptr, 0},
        {nullptr, 0}, {nullptr, 0}, {nullptr, 0}, {nullptr, 0}, {nullptr, 0}};
    bool present_[ucode_source_count]{};
    UcodeEntry entries_[maximum_ucode_entries]{};
    size_t count_ = 0;
};

} // namespace rx7900::hardware

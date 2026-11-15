/*
Module: rx7900.hardware.vbios_rom
Purpose: Linux v6.12 VBIOS acquisition protocols and per-source image validity rules.
Created: 2026-10-22
Architecture: Pure protocol over borrowed bytes, a ROM-window reader and the shared
RegisterIo contract; src/reactos/vbios owns every real mapping and config write.
common/atom_bios stays the only ATOM/PCIR signature parser.
*/
#pragma once

#include "atom_bios.hpp"
#include "nbio.hpp"
#include "sdma.hpp"

namespace rx7900::hardware {

// SMUIO hardware ID (Linux soc15_hw_ip.h SMUIO_HWID). Navi31 reports SMUIO
// 13.0.6 in IP discovery, which Linux binds to smuio_v13_0_6_funcs.
constexpr uint16_t discovery_smuio_hardware_id = 4;

// Linux amdgpu_bios.c AMD_VBIOS_SIGNATURE " 761295520" at offset 0x30. The probe
// reads AMD_VBIOS_SIGNATURE_END + 1 bytes (dword rounded); the length byte at
// offset 2 counts 512-byte units, so a SMUIO read is at most 255 * 512 bytes.
constexpr uint32_t vbios_signature_offset = 0x30;
constexpr uint32_t vbios_signature_bytes = 10;
constexpr uint32_t vbios_probe_bytes = 0x3c;
constexpr uint32_t vbios_length_unit_bytes = 512;
constexpr uint32_t maximum_smuio_rom_bytes = 255u * vbios_length_unit_bytes;
// Linux check_atom_bios minimum candidate size.
constexpr uint32_t minimum_vbios_bytes = 0x49;
// Linux pci_fixup_video legacy shadow window for the boot VGA function.
constexpr uint32_t legacy_rom_shadow_address = 0xc0000;
constexpr uint32_t legacy_rom_shadow_bytes = 0x20000;

enum class VbiosStatus {
    success,
    invalid_argument,
    unsupported_ip,
    mmio_failure,
    // 0x55AA or the AMD " 761295520" signature is missing (Linux returns false).
    no_signature,
    // A declared length is zero, beyond the source window or beyond storage.
    invalid_length,
    insufficient_storage,
    // Linux check_atom_bios rejected the candidate (0x55AA, header, ATOM/MOTA).
    invalid_atom,
    // Project identity rule: the first PCIR image is not this display function.
    identity_mismatch,
    // VFCT: no image header matches this function (Linux "too short #2").
    absent
};

// Linux check_atom_bios on caller-owned bytes, with checked bounds: at least
// 0x49 bytes (the u16 header pointer at 0x48 must be wholly present here),
// 0x55AA at 0, a nonzero header pointer and "ATOM" or "MOTA" four bytes past it,
// wholly inside the candidate. No table is interpreted.
VbiosStatus check_atom_bios(ByteView candidate) noexcept;

// Every candidate, from any source, before it may become the device ROM: Linux
// check_atom_bios over the whole candidate, then the project rule shared with the
// administrator file (validate_atom_image): at most maximum_atom_file_bytes, the
// first PCIR image belongs to vendor/device with display class and a declared
// extent inside the candidate, and that image alone passes the ATOM check.
// image is unchanged on failure. Structure and identity only, not authenticity.
VbiosStatus validate_vbios_image(ByteView candidate, uint16_t vendor, uint16_t device,
                                 AtomImage& image) noexcept;

// atom_rom_header_v2_2 subsystem_vendor_id/subsystem_id (offsets 24/26 of the ROM
// header, which must be at least 28 bytes) equal the function's config-space
// subsystem IDs. Linux does not check this; the native owner requires it for the
// legacy shadow, which is not addressed through this function's own BARs.
VbiosStatus check_atom_subsystem(ByteView image, uint16_t subsystem_vendor,
                                 uint16_t subsystem_device) noexcept;

// Readable expansion-ROM window: a mapped ROM BAR, the legacy shadow, or bytes in
// memory. read32 reads the little-endian dword at an aligned byte offset below
// bytes; false is an access failure. The context outlives every call.
struct RomWindow final {
    const void* context;
    uint32_t bytes;
    bool (*read32)(const void*, uint32_t aligned_offset, uint32_t&) noexcept;
};

// A window over a borrowed ByteView; view must outlive the returned window.
RomWindow byte_view_rom_window(const ByteView& view) noexcept;

// Linux pci_get_rom_size: walk images while each starts with 0x55AA and its PCIR
// structure (u16 pointer at image + 24) carries "PCIR"; stop at the last-image
// indicator (PCIR + 21 bit 7), a zero length (PCIR + 16, 512-byte units), the
// window end or a following image without 0x55AA. Returns the walked extent,
// never more than the window. Unlike Linux, every structure read must lie
// wholly inside the window, otherwise the walk stops as at an invalid image.
// bytes is zero when the first image is invalid; false only on access failure.
bool pci_rom_image_bytes(const RomWindow& window, uint32_t& bytes) noexcept;

// Linux amdgpu_acpi_vfct_bios over a borrowed copy of the whole ACPI VFCT table.
// The ACPI header length must be at least sizeof(UEFI_ACPI_VFCT) = 0x4c and fit
// the view; image headers (28 bytes: bus, device, function u32, vendor, device
// u16, ...) start at VBIOSImageOffset and are walked exactly as Linux does,
// failing on a truncated header or image. The first nonempty image whose
// bus/device/function/vendor/device match is returned after check_atom_bios.
struct VfctTarget final {
    uint32_t bus;
    uint32_t device;
    uint32_t function;
    uint16_t vendor_id;
    uint16_t device_id;
};

VbiosStatus find_vfct_image(ByteView table, const VfctTarget& target, ByteView& image) noexcept;

// Only validate_smuio_rom_discovery creates a valid token: a unique SMUIO
// instance0 13.0.6 with its first segment, ROM_INDEX (0xe4) and ROM_DATA (0xe5)
// at that base wholly inside the register aperture of an already validated NBIO
// token, whose NBIF segment 5 locates REGS_ROM_OFFSET_CTRL (0xcc23).
class SmuioRomRegisters final {
public:
    SmuioRomRegisters() noexcept = default;
    bool valid() const noexcept { return smuio_.valid() && aperture_bytes_ != 0; }
    uint32_t base() const noexcept { return smuio_.segment(0); }
    uint32_t rom_offset_base() const noexcept { return rom_offset_base_; }
    uint32_t aperture_bytes() const noexcept { return aperture_bytes_; }

private:
    friend DiscoveryStatus validate_smuio_rom_discovery(ByteView, const NbioRegisters&,
                                                        SmuioRomRegisters&) noexcept;
    DiscoveredIp smuio_{};
    uint32_t rom_offset_base_ = 0;
    uint32_t aperture_bytes_ = 0;
};

DiscoveryStatus validate_smuio_rom_discovery(ByteView binary, const NbioRegisters& nbio,
                                             SmuioRomRegisters& output) noexcept;

struct SmuioRomReport final {
    VbiosStatus status = VbiosStatus::invalid_argument;
    // REGS_ROM_OFFSET_CTRL.ROM_OFFSET << 17 as written to ROM_INDEX.
    uint32_t rom_offset = 0;
    // AMD_VBIOS_LENGTH(header) dword aligned; zero until the probe passed.
    uint32_t declared_bytes = 0;
};

// Linux amdgpu_read_bios_from_rom with soc21 amdgpu_soc15_read_bios_from_rom.
// Each of the two reads takes ROM_OFFSET from NBIO4.3 REGS_ROM_OFFSET_CTRL
// (segment 5, beyond BAR5, so through the NBIO SMN pair), writes it << 17 to
// ROM_INDEX once and reads ROM_DATA dword by dword; the hardware advances the
// index by four bytes after each data read. The probe reads 0x3c bytes and
// requires 0x55AA plus " 761295520" at 0x30; the full read is header[2] * 512
// bytes into output, which must hold them. Unlike Linux, a zero declared length
// fails before the second read. Linux then runs check_atom_bios over the copy:
// the caller must pass the first bytes of output to validate_vbios_image, which
// starts with that check. bytes is the copied length.
// PASSIVE_LEVEL, exclusive register owner before any ASIC initialization: no
// ISR, DPC or thread may use ROM_INDEX/ROM_DATA or the SMN pair concurrently.
// nbio must be initialized with the same RegisterIo. Output bytes beyond the
// copied length are unspecified; bytes is zero on failure.
SmuioRomReport read_smuio_rom(const RegisterIo& io, const SmuioRomRegisters& smuio, Nbio& nbio,
                              sdma::CommandBuffer output, uint32_t& bytes) noexcept;

} // namespace rx7900::hardware

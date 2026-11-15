/*
Module: rx7900.tests.vbios_rom
Purpose: Verify Linux v6.12 VBIOS acquisition protocols and per-source validity rules.
Created: 2026-10-22
Architecture: Synthetic SMUIO/NBIO register model and synthetic ROM/VFCT bytes;
no GPU, ROM BAR or ACPI table is accessed.
*/
#include <algorithm>
#include <utility>
#include <vector>

#include "asic_test_device.hpp"
#include "vbios_rom.hpp"

namespace {

using namespace rx7900::hardware;
using rx7900::AtomImage;
using rx7900::ByteView;

rx7900_test::Checks checks;

void expect(bool condition, const char* contract) { checks.expect(condition, contract); }

// The user's board sysfs SMUIO listing: instance0 13.0.6 with these bases.
const std::vector<uint32_t> smuio_bases{0x16800, 0x16a00, 0x02401000, 0x03440000};
constexpr uint32_t rom_index_byte = (0x16800u + 0xe4u) * 4;
constexpr uint32_t rom_data_byte = (0x16800u + 0xe5u) * 4;
constexpr uint32_t rsmu_index_byte = 0x14u * 4;
constexpr uint32_t rsmu_data_byte = (0x14u + 1) * 4;
constexpr uint32_t rom_offset_ctrl_smn = (0x4040000u + 0xcc23u) * 4;

std::vector<uint8_t> board_discovery(uint8_t smuio_minor = 0, uint8_t smuio_revision = 6,
                                     std::vector<uint32_t> bases = smuio_bases) {
    return rx7900_test::discovery_with({
        {discovery_nbif_hardware_id, 4, 3,
         {rx7900_test::nbif_bases[0], rx7900_test::nbif_bases[1], rx7900_test::nbif_bases[2],
          rx7900_test::nbif_bases[3], rx7900_test::nbif_bases[4], rx7900_test::nbif_bases[5]}},
        {discovery_mmhub_hardware_id, 3, 0, {rx7900_test::mmhub_base}},
        {discovery_smuio_hardware_id, 13, smuio_minor, std::move(bases), smuio_revision}});
}

void put16(std::vector<uint8_t>& bytes, size_t offset, uint16_t value) {
    bytes.at(offset) = static_cast<uint8_t>(value);
    bytes.at(offset + 1) = static_cast<uint8_t>(value >> 8);
}

void put32(std::vector<uint8_t>& bytes, size_t offset, uint32_t value) {
    put16(bytes, offset, static_cast<uint16_t>(value));
    put16(bytes, offset + 2, static_cast<uint16_t>(value >> 16));
}

struct ImageSpec final {
    uint16_t vendor = 0x1002;
    uint16_t device = 0x744c;
    uint16_t units = 2;
    bool last = true;
    uint8_t code_type = 0;
    bool amd_signature = true;
    const char* atom = "ATOM";
    uint16_t subsystem_vendor = 0x1da2;
    uint16_t subsystem_device = 0x471e;
};

// One option-ROM image: 0x55AA, length byte, AMD signature at 0x30, PCIR at 0x60,
// ATOM ROM header v2.2 at 0x100 (size 0x24) with the subsystem IDs.
std::vector<uint8_t> rom_image(const ImageSpec& spec = {}) {
    std::vector<uint8_t> bytes(static_cast<size_t>(spec.units) * 512, 0x5a);
    put16(bytes, 0, 0xaa55);
    bytes.at(2) = static_cast<uint8_t>(spec.units);
    const char signature[] = " 761295520";

    for (size_t index = 0; spec.amd_signature && index < 10; ++index) {
        bytes.at(0x30 + index) = static_cast<uint8_t>(signature[index]);
    }

    put16(bytes, 0x18, 0x60);
    put32(bytes, 0x60, 0x52494350);
    put16(bytes, 0x64, spec.vendor);
    put16(bytes, 0x66, spec.device);
    put16(bytes, 0x6a, 0x18);
    bytes.at(0x6f) = 3;
    put16(bytes, 0x70, spec.units);
    bytes.at(0x74) = spec.code_type;
    bytes.at(0x75) = spec.last ? 0x80 : 0;
    put16(bytes, 0x48, 0x100);
    put16(bytes, 0x100, 0x24);
    bytes.at(0x102) = 2;
    bytes.at(0x103) = 2;

    for (size_t index = 0; index < 4; ++index) {
        bytes.at(0x104 + index) = static_cast<uint8_t>(spec.atom[index]);
    }

    put16(bytes, 0x118, spec.subsystem_vendor);
    put16(bytes, 0x11a, spec.subsystem_device);
    return bytes;
}

// SMUIO ROM_INDEX/ROM_DATA plus the NBIO RSMU SMN pair, logging every access.
// ROM_INDEX holds a byte address; each ROM_DATA read returns the dword there and
// advances the index by four, as amdgpu_soc15_read_bios_from_rom relies on.
struct SmuioModel final {
    struct Access final {
        bool write;
        uint32_t offset;
        uint32_t value;
        bool operator==(const Access& other) const {
            return write == other.write && offset == other.offset && value == other.value;
        }
    };

    std::vector<uint8_t> flash;
    uint32_t rom_offset_field = 0;
    uint32_t rom_index = 0;
    uint32_t smn_index = 0;
    size_t fail_after = SIZE_MAX;
    std::vector<Access> log;

    bool fault() { return log.size() >= fail_after; }

    static bool read(void* context, uint32_t offset, uint32_t& value) noexcept {
        auto& self = *static_cast<SmuioModel*>(context);

        if (self.fault()) {
            return false;
        }

        value = 0;

        if (offset == rom_data_byte) {
            for (uint32_t byte = 0; byte < 4; ++byte) {
                const size_t at = static_cast<size_t>(self.rom_index) + byte;
                const uint8_t data = at < self.flash.size() ? self.flash[at] : 0xff;
                value |= static_cast<uint32_t>(data) << (byte * 8);
            }

            self.rom_index += 4;
        } else if (offset == rsmu_index_byte) {
            value = self.smn_index;
        } else if (offset == rsmu_data_byte) {
            value = self.smn_index == rom_offset_ctrl_smn ? (self.rom_offset_field | 0xffffff00u) : 0;
        }

        self.log.push_back({false, offset, value});
        return true;
    }

    static bool write(void* context, uint32_t offset, uint32_t value) noexcept {
        auto& self = *static_cast<SmuioModel*>(context);

        if (self.fault()) {
            return false;
        }

        if (offset == rom_index_byte) {
            self.rom_index = value;
        } else if (offset == rsmu_index_byte) {
            self.smn_index = value;
        }

        self.log.push_back({true, offset, value});
        return true;
    }

    static void delay(void*, uint32_t) noexcept {}

    RegisterIo io() { return {this, rx7900_test::aperture_bytes, read, write, delay}; }
};

struct SmuioFixture final {
    std::vector<uint8_t> binary = board_discovery();
    NbioRegisters nbio_registers;
    SmuioRomRegisters smuio;
    SmuioModel model;
    Nbio nbio;

    SmuioFixture() {
        validate_nbio_discovery({binary.data(), binary.size()}, rx7900_test::aperture_bytes,
                                nbio_registers);
        validate_smuio_rom_discovery({binary.data(), binary.size()}, nbio_registers, smuio);
        nbio.initialize(nbio_registers, model.io());
    }
};

// One get_rom_offset (SMN index write, posting read, data read), one ROM_INDEX
// write and dwords ROM_DATA reads.
std::vector<SmuioModel::Access> soc15_sequence(uint32_t rom_offset_field, uint32_t dwords,
                                               const std::vector<uint8_t>& flash) {
    std::vector<SmuioModel::Access> sequence{
        {true, rsmu_index_byte, rom_offset_ctrl_smn},
        {false, rsmu_index_byte, rom_offset_ctrl_smn},
        {false, rsmu_data_byte, rom_offset_field | 0xffffff00u},
        {true, rom_index_byte, (rom_offset_field & 0x7f) << 17}};
    const size_t base = static_cast<size_t>(rom_offset_field & 0x7f) << 17;

    for (uint32_t index = 0; index < dwords; ++index) {
        uint32_t value = 0;

        for (uint32_t byte = 0; byte < 4; ++byte) {
            const size_t at = base + index * 4 + byte;
            value |= static_cast<uint32_t>(at < flash.size() ? flash[at] : 0xff) << (byte * 8);
        }

        sequence.push_back({false, rom_data_byte, value});
    }

    return sequence;
}

void smuio_discovery() {
    SmuioFixture fixture;
    expect(fixture.smuio.valid() && fixture.smuio.base() == 0x16800 &&
           fixture.smuio.rom_offset_base() == 0x4040000 &&
           fixture.smuio.aperture_bytes() == rx7900_test::aperture_bytes,
           "SMUIO 13.0.6 segment 0 and NBIF segment 5 are taken from discovery");

    SmuioRomRegisters output;
    const auto v13_0_0 = board_discovery(0, 0);
    expect(validate_smuio_rom_discovery({v13_0_0.data(), v13_0_0.size()}, fixture.nbio_registers,
                                        output) == DiscoveryStatus::unsupported && !output.valid(),
           "SMUIO 13.0.0 is not assumed to be 13.0.6");
    const auto v13_0_2 = board_discovery(0, 2);
    expect(validate_smuio_rom_discovery({v13_0_2.data(), v13_0_2.size()}, fixture.nbio_registers,
                                        output) == DiscoveryStatus::unsupported,
           "SMUIO 13.0.2 (smuio_v13_0_funcs) is refused");

    const auto absent = rx7900_test::discovery_with({
        {discovery_nbif_hardware_id, 4, 3,
         {0, 0x14, 0xd20, 0x10400, 0x241b000, 0x4040000}},
        {discovery_mmhub_hardware_id, 3, 0, {rx7900_test::mmhub_base}}});
    expect(validate_smuio_rom_discovery({absent.data(), absent.size()}, fixture.nbio_registers,
                                        output) != DiscoveryStatus::validated && !output.valid(),
           "missing SMUIO record is refused");

    const auto beyond = board_discovery(0, 6, {0x3ffff0});
    expect(validate_smuio_rom_discovery({beyond.data(), beyond.size()}, fixture.nbio_registers,
                                        output) == DiscoveryStatus::invalid_argument,
           "ROM_INDEX/ROM_DATA outside BAR5 are refused");

    expect(validate_smuio_rom_discovery({fixture.binary.data(), fixture.binary.size()},
                                        NbioRegisters{}, output) == DiscoveryStatus::invalid_argument,
           "an unvalidated NBIO token is refused");

    auto corrupt = fixture.binary;
    corrupt.at(corrupt.size() - 1) ^= 1;
    expect(validate_smuio_rom_discovery({corrupt.data(), corrupt.size()}, fixture.nbio_registers,
                                        output) == DiscoveryStatus::invalid_binary,
           "discovery checksum failure is refused");
}

void smuio_read_sequence() {
    SmuioFixture fixture;
    const auto flash = rom_image({0x1002, 0x744c, 4});
    fixture.model.flash = flash;
    std::vector<uint8_t> storage(maximum_smuio_rom_bytes, 0);
    uint32_t bytes = 0;
    const auto report = read_smuio_rom(fixture.model.io(), fixture.smuio, fixture.nbio,
                                       {storage.data(), storage.size()}, bytes);
    auto expected = soc15_sequence(0, vbios_probe_bytes / 4, flash);
    const auto full = soc15_sequence(0, 4 * 512 / 4, flash);
    expected.insert(expected.end(), full.begin(), full.end());
    expect(report.status == VbiosStatus::success && bytes == 2048 && report.declared_bytes == 2048 &&
           report.rom_offset == 0, "SMUIO read copies header[2] * 512 bytes");
    expect(fixture.model.log == expected,
           "exact Linux sequence: offset, ROM_INDEX, 15 probe reads, offset, ROM_INDEX, full reads");
    expect(std::equal(flash.begin(), flash.end(), storage.begin()), "copied bytes equal the flash");

    AtomImage image{};
    expect(validate_vbios_image({storage.data(), bytes}, 0x1002, 0x744c, image) ==
           VbiosStatus::success && image.image_bytes == 2048 && image.header.offset == 0x100,
           "SMUIO copy passes the shared validity rules");

    SmuioFixture offset;
    std::vector<uint8_t> shifted(0x40000 + 1024, 0);
    const auto relocated = rom_image({0x1002, 0x744c, 2});
    std::copy(relocated.begin(), relocated.end(), shifted.begin() + 0x40000);
    offset.model.flash = shifted;
    offset.model.rom_offset_field = 2;
    const auto moved = read_smuio_rom(offset.model.io(), offset.smuio, offset.nbio,
                                      {storage.data(), storage.size()}, bytes);
    expect(moved.status == VbiosStatus::success && moved.rom_offset == 0x40000 && bytes == 1024 &&
           offset.model.log.at(3) == SmuioModel::Access{true, rom_index_byte, 0x40000} &&
           std::equal(relocated.begin(), relocated.end(), storage.begin()),
           "REGS_ROM_OFFSET_CTRL.ROM_OFFSET << 17 selects the ROM base");
}

void smuio_read_failures() {
    std::vector<uint8_t> storage(maximum_smuio_rom_bytes, 0);
    uint32_t bytes = 7;

    {
        SmuioFixture fixture;
        fixture.model.flash = rom_image({0x1002, 0x744c, 2, true, 0, false});
        const auto report = read_smuio_rom(fixture.model.io(), fixture.smuio, fixture.nbio,
                                           {storage.data(), storage.size()}, bytes);
        expect(report.status == VbiosStatus::no_signature && bytes == 0 &&
               fixture.model.log.size() == 4 + vbios_probe_bytes / 4,
               "missing AMD signature stops after the probe read");
    }

    {
        SmuioFixture fixture;
        auto flash = rom_image();
        flash.at(0x39) = '1';
        fixture.model.flash = flash;
        expect(read_smuio_rom(fixture.model.io(), fixture.smuio, fixture.nbio,
                              {storage.data(), storage.size()}, bytes).status == VbiosStatus::no_signature,
               "every character of \" 761295520\" is compared");
    }

    {
        SmuioFixture fixture;
        auto flash = rom_image();
        flash.at(0) = 0;
        fixture.model.flash = flash;
        expect(read_smuio_rom(fixture.model.io(), fixture.smuio, fixture.nbio,
                              {storage.data(), storage.size()}, bytes).status == VbiosStatus::no_signature,
               "missing 0x55AA is refused");
    }

    {
        SmuioFixture fixture;
        expect(read_smuio_rom(fixture.model.io(), fixture.smuio, fixture.nbio,
                              {storage.data(), storage.size()}, bytes).status == VbiosStatus::no_signature,
               "all-ones (absent) flash is refused");
    }

    {
        SmuioFixture fixture;
        auto flash = rom_image();
        flash.at(2) = 0;
        fixture.model.flash = flash;
        const auto report = read_smuio_rom(fixture.model.io(), fixture.smuio, fixture.nbio,
                                           {storage.data(), storage.size()}, bytes);
        expect(report.status == VbiosStatus::invalid_length && bytes == 0 &&
               fixture.model.log.size() == 4 + vbios_probe_bytes / 4,
               "zero declared length fails before a second read");
    }

    {
        SmuioFixture fixture;
        fixture.model.flash = rom_image({0x1002, 0x744c, 4});
        std::vector<uint8_t> small(2047, 0);
        const auto report = read_smuio_rom(fixture.model.io(), fixture.smuio, fixture.nbio,
                                           {small.data(), small.size()}, bytes);
        expect(report.status == VbiosStatus::insufficient_storage && report.declared_bytes == 2048 &&
               bytes == 0, "declared length beyond storage is refused");
    }

    {
        SmuioFixture fixture;
        fixture.model.flash = rom_image({0x1002, 0x744c, 255});
        const auto report = read_smuio_rom(fixture.model.io(), fixture.smuio, fixture.nbio,
                                           {storage.data(), storage.size()}, bytes);
        expect(report.status == VbiosStatus::success && bytes == maximum_smuio_rom_bytes,
               "the 255-unit maximum fits maximum_smuio_rom_bytes");
    }

    for (const size_t failure : {size_t{0}, size_t{3}, size_t{10}, size_t{4 + 15 + 2}, size_t{4 + 15 + 4 + 100}}) {
        SmuioFixture fixture;
        fixture.model.flash = rom_image({0x1002, 0x744c, 4});
        fixture.model.fail_after = failure;
        expect(read_smuio_rom(fixture.model.io(), fixture.smuio, fixture.nbio,
                              {storage.data(), storage.size()}, bytes).status == VbiosStatus::mmio_failure &&
               bytes == 0, "MMIO failure at any step is reported");
    }

    SmuioFixture fixture;
    expect(read_smuio_rom(fixture.model.io(), SmuioRomRegisters{}, fixture.nbio,
                          {storage.data(), storage.size()}, bytes).status == VbiosStatus::invalid_argument &&
           fixture.model.log.empty(), "an invalid SMUIO token performs no access");
    RegisterIo other = fixture.model.io();
    other.aperture_bytes = 0x80000;
    expect(read_smuio_rom(other, fixture.smuio, fixture.nbio, {storage.data(), storage.size()},
                          bytes).status == VbiosStatus::invalid_argument,
           "a different register aperture is refused");
    Nbio uninitialized;
    fixture.model.flash = rom_image();
    expect(read_smuio_rom(fixture.model.io(), fixture.smuio, uninitialized,
                          {storage.data(), storage.size()}, bytes).status == VbiosStatus::mmio_failure &&
           fixture.model.log.empty(), "an uninitialized NBIO cannot supply the ROM offset");
}

void validity() {
    AtomImage image{};
    const auto good = rom_image();
    expect(check_atom_bios({good.data(), good.size()}) == VbiosStatus::success, "ATOM accepted");
    expect(validate_vbios_image({good.data(), good.size()}, 0x1002, 0x744c, image) ==
           VbiosStatus::success && image.image_bytes == 1024, "identity and extent accepted");

    const auto mota = rom_image({0x1002, 0x744c, 2, true, 0, true, "MOTA"});
    expect(check_atom_bios({mota.data(), mota.size()}) == VbiosStatus::success,
           "reversed MOTA signature accepted as Linux does");

    const auto bad_atom = rom_image({0x1002, 0x744c, 2, true, 0, true, "ATOX"});
    expect(check_atom_bios({bad_atom.data(), bad_atom.size()}) == VbiosStatus::invalid_atom &&
           validate_vbios_image({bad_atom.data(), bad_atom.size()}, 0x1002, 0x744c, image) ==
           VbiosStatus::invalid_atom, "non-ATOM image refused");

    auto no_rom = good;
    no_rom.at(1) = 0;
    expect(check_atom_bios({no_rom.data(), no_rom.size()}) == VbiosStatus::no_signature,
           "missing 0x55AA refused");

    auto zero_header = good;
    put16(zero_header, 0x48, 0);
    expect(check_atom_bios({zero_header.data(), zero_header.size()}) == VbiosStatus::invalid_atom,
           "zero ROM header pointer refused");

    auto end_header = good;
    put16(end_header, 0x48, static_cast<uint16_t>(end_header.size() - 6));
    expect(check_atom_bios({end_header.data(), end_header.size()}) == VbiosStatus::invalid_atom,
           "ATOM string crossing the end refused (Linux would read past it)");

    auto wrap = good;
    put16(wrap, 0x48, 0xfffe);
    expect(check_atom_bios({wrap.data(), wrap.size()}) == VbiosStatus::invalid_atom,
           "16-bit header pointer plus four does not wrap");

    expect(check_atom_bios({good.data(), 0x48}) == VbiosStatus::invalid_length &&
           check_atom_bios({good.data(), 0x49}) == VbiosStatus::invalid_atom &&
           check_atom_bios({nullptr, 4096}) == VbiosStatus::invalid_length,
           "short and null candidates refused");

    const auto other = rom_image({0x1002, 0x15bf});
    expect(validate_vbios_image({other.data(), other.size()}, 0x1002, 0x744c, image) ==
           VbiosStatus::identity_mismatch, "another device's ATOM image refused");

    auto truncated = rom_image({0x1002, 0x744c, 4});
    truncated.resize(1536);
    expect(validate_vbios_image({truncated.data(), truncated.size()}, 0x1002, 0x744c, image) ==
           VbiosStatus::invalid_length, "PCIR extent beyond the candidate refused");

    std::vector<uint8_t> huge(rx7900::maximum_atom_file_bytes + 4, 0);
    std::copy(good.begin(), good.end(), huge.begin());
    expect(validate_vbios_image({huge.data(), huge.size()}, 0x1002, 0x744c, image) ==
           VbiosStatus::invalid_length, "candidate above 1 MiB refused");

    const auto efi = rom_image({0x1002, 0x744c, 2, true, 7});
    expect(validate_vbios_image({efi.data(), efi.size()}, 0x1002, 0x744c, image) ==
           VbiosStatus::invalid_atom, "unknown PCIR code type refused");

    expect(check_atom_subsystem({good.data(), good.size()}, 0x1da2, 0x471e) == VbiosStatus::success,
           "ROM header subsystem matches the board");
    expect(check_atom_subsystem({good.data(), good.size()}, 0x1da2, 0x471f) ==
           VbiosStatus::identity_mismatch, "ROM header subsystem mismatch refused");
    auto short_header = good;
    put16(short_header, 0x100, 0x1a);
    expect(check_atom_subsystem({short_header.data(), short_header.size()}, 0x1da2, 0x471e) ==
           VbiosStatus::invalid_atom, "ROM header without subsystem fields refused");
}

void rom_walk() {
    uint32_t bytes = 1;
    std::vector<uint8_t> window(0x20000, 0);
    const auto legacy = rom_image({0x1002, 0x744c, 0x80, false});
    const auto efi = rom_image({0x1002, 0x744c, 0x40, true, 3});
    std::copy(legacy.begin(), legacy.end(), window.begin());
    std::copy(efi.begin(), efi.end(), window.begin() + legacy.size());
    const ByteView view(window.data(), window.size());
    expect(pci_rom_image_bytes(byte_view_rom_window(view), bytes) && bytes == 0x18000,
           "legacy plus last EFI image extent");

    const auto trailing = rom_image({0x1002, 0x744c, 0x10, true});
    std::copy(trailing.begin(), trailing.end(), window.begin() + 0x18000);
    expect(pci_rom_image_bytes(byte_view_rom_window(view), bytes) && bytes == 0x18000,
           "the last-image indicator ends the walk even before another valid image");

    put16(window, 0x10000, 0);
    expect(pci_rom_image_bytes(byte_view_rom_window(view), bytes) && bytes == 0x10000,
           "following image without 0x55AA ends the walk");

    std::vector<uint8_t> single(0x20000, 0);
    const auto only = rom_image({0x1002, 0x744c, 0x80, true});
    std::copy(only.begin(), only.end(), single.begin());
    const ByteView single_view(single.data(), single.size());
    expect(pci_rom_image_bytes(byte_view_rom_window(single_view), bytes) && bytes == 0x10000,
           "shadow window holding one last image");

    auto overlong = single;
    put16(overlong, 0x70, 0x200);
    expect(pci_rom_image_bytes(byte_view_rom_window(ByteView(overlong.data(), overlong.size())), bytes) &&
           bytes == 0x20000, "declared length is clamped to the window");

    auto bad_pcir = single;
    put32(bad_pcir, 0x60, 0);
    expect(pci_rom_image_bytes(byte_view_rom_window(ByteView(bad_pcir.data(), bad_pcir.size())), bytes) &&
           bytes == 0, "missing PCIR yields zero");

    auto far_pcir = single;
    put16(far_pcir, 0x18, 0xfff0);
    const ByteView short_view(far_pcir.data(), 0x10000);
    expect(pci_rom_image_bytes(byte_view_rom_window(short_view), bytes) && bytes == 0,
           "PCIR structure beyond the window yields zero");

    auto zero_units = single;
    put16(zero_units, 0x70, 0);
    zero_units.at(0x75) = 0;
    expect(pci_rom_image_bytes(byte_view_rom_window(ByteView(zero_units.data(), zero_units.size())), bytes) &&
           bytes == 0, "zero image length stops the walk");

    std::vector<uint8_t> exact = rom_image({0x1002, 0x744c, 0x80, false});
    expect(pci_rom_image_bytes(byte_view_rom_window(ByteView(exact.data(), exact.size())), bytes) &&
           bytes == 0x10000, "a non-last image ending at the window edge stops without reading past it");

    expect(pci_rom_image_bytes(byte_view_rom_window(ByteView(single.data(), 20)), bytes) && bytes == 0,
           "window smaller than an image header yields zero");

    const RomWindow failing{nullptr, 0x20000,
        [](const void*, uint32_t, uint32_t&) noexcept { return false; }};
    expect(!pci_rom_image_bytes(failing, bytes) && bytes == 0, "access failure is reported");
}

std::vector<uint8_t> vfct_table(const std::vector<std::pair<std::vector<uint32_t>, std::vector<uint8_t>>>& images) {
    std::vector<uint8_t> table(0x4c, 0);
    table.at(0) = 'V';
    table.at(1) = 'F';
    table.at(2) = 'C';
    table.at(3) = 'T';
    put32(table, 0x34, 0x4c);

    for (const auto& entry : images) {
        const size_t at = table.size();
        table.resize(at + 0x1c + entry.second.size(), 0);
        put32(table, at, entry.first.at(0));
        put32(table, at + 4, entry.first.at(1));
        put32(table, at + 8, entry.first.at(2));
        put16(table, at + 0x0c, static_cast<uint16_t>(entry.first.at(3)));
        put16(table, at + 0x0e, static_cast<uint16_t>(entry.first.at(4)));
        put32(table, at + 0x18, static_cast<uint32_t>(entry.second.size()));
        std::copy(entry.second.begin(), entry.second.end(), table.begin() + at + 0x1c);
    }

    put32(table, 4, static_cast<uint32_t>(table.size()));
    return table;
}

bool same_bytes(ByteView view, const std::vector<uint8_t>& expected) {
    if (view.size() != expected.size()) {
        return false;
    }

    for (size_t index = 0; index < expected.size(); ++index) {
        uint8_t value = 0;

        if (!view.read_u8(index, value) || value != expected[index]) {
            return false;
        }
    }

    return true;
}

void vfct() {
    const VfctTarget target{3, 0, 0, 0x1002, 0x744c};
    const auto igpu = rom_image({0x1002, 0x15bf});
    const auto dgpu = rom_image();
    const auto table = vfct_table({{{0x11, 0, 0, 0x1002, 0x15bf}, igpu}, {{3, 0, 0, 0x1002, 0x744c}, dgpu}});
    ByteView image(nullptr, 0);
    expect(find_vfct_image({table.data(), table.size()}, target, image) == VbiosStatus::success &&
           same_bytes(image, dgpu), "matching bus/device/function/vendor/device image returned");

    const auto other_bus = vfct_table({{{4, 0, 0, 0x1002, 0x744c}, dgpu}});
    expect(find_vfct_image({other_bus.data(), other_bus.size()}, target, image) == VbiosStatus::absent,
           "image for another bus is not taken");

    const auto other_function = vfct_table({{{3, 0, 1, 0x1002, 0x744c}, dgpu}});
    const auto other_device = vfct_table({{{3, 1, 0, 0x1002, 0x744c}, dgpu}});
    const auto other_vendor = vfct_table({{{3, 0, 0, 0x1022, 0x744c}, dgpu}});
    const auto other_id = vfct_table({{{3, 0, 0, 0x1002, 0x7448}, dgpu}});
    expect(find_vfct_image({other_function.data(), other_function.size()}, target, image) ==
           VbiosStatus::absent &&
           find_vfct_image({other_device.data(), other_device.size()}, target, image) ==
           VbiosStatus::absent &&
           find_vfct_image({other_vendor.data(), other_vendor.size()}, target, image) ==
           VbiosStatus::absent &&
           find_vfct_image({other_id.data(), other_id.size()}, target, image) == VbiosStatus::absent,
           "function, slot, vendor and device must each match");

    const auto empty = vfct_table({{{3, 0, 0, 0x1002, 0x744c}, {}}, {{3, 0, 0, 0x1002, 0x744c}, dgpu}});
    expect(find_vfct_image({empty.data(), empty.size()}, target, image) == VbiosStatus::success,
           "zero-length matching header is skipped as in Linux");

    auto bad = dgpu;
    bad.at(0x104) = 'X';
    const auto invalid = vfct_table({{{3, 0, 0, 0x1002, 0x744c}, bad}});
    expect(find_vfct_image({invalid.data(), invalid.size()}, target, image) == VbiosStatus::invalid_atom,
           "matching non-ATOM image fails without trying later headers");

    auto truncated_image = table;
    put32(truncated_image, 4, static_cast<uint32_t>(table.size() - 1));
    expect(find_vfct_image({truncated_image.data(), truncated_image.size()}, target, image) ==
           VbiosStatus::invalid_length, "truncated image refused");

    auto truncated_header = vfct_table({});
    truncated_header.resize(0x4c + 0x10, 0);
    put32(truncated_header, 4, static_cast<uint32_t>(truncated_header.size()));
    expect(find_vfct_image({truncated_header.data(), truncated_header.size()}, target, image) ==
           VbiosStatus::invalid_length, "truncated image header refused");

    auto overflow = vfct_table({{{3, 0, 0, 0x1002, 0x744c}, dgpu}});
    put32(overflow, 0x4c + 0x18, 0xffffffffu);
    expect(find_vfct_image({overflow.data(), overflow.size()}, target, image) ==
           VbiosStatus::invalid_length, "image length does not wrap the offset");

    auto short_table = table;
    put32(short_table, 4, 0x4b);
    expect(find_vfct_image({short_table.data(), short_table.size()}, target, image) ==
           VbiosStatus::invalid_length, "table shorter than UEFI_ACPI_VFCT refused");

    expect(find_vfct_image({table.data(), table.size() - 1}, target, image) == VbiosStatus::invalid_length,
           "declared length beyond the copied table refused");

    auto beyond = table;
    put32(beyond, 0x34, static_cast<uint32_t>(table.size()));
    expect(find_vfct_image({beyond.data(), beyond.size()}, target, image) == VbiosStatus::absent,
           "first image offset at the end finds nothing");
}

} // namespace

int main() {
    smuio_discovery();
    smuio_read_sequence();
    smuio_read_failures();
    validity();
    rom_walk();
    vfct();
    return checks.finish("vbios_rom_tests");
}

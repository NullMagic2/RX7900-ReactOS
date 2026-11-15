/*
Module: rx7900.tests.output
Purpose: Verify EDID framing, detailed timings and conservative output budgets.
Created: 2026-10-21
Architecture: Portable tests; no DDC/AUX, register access, VM or hardware changes.
*/
#include <array>
#include <cstdio>

#include "sink.hpp"
#include "atom_connectors.hpp"

namespace {

using namespace rx7900::output;
using rx7900::ByteView;
using rx7900::display::Timing;
using Edid = std::array<uint8_t, 256>;
using AtomRom = std::array<uint8_t, 1024>;
unsigned checks = 0;
unsigned failures = 0;

void expect(bool condition, const char* contract) {
    ++checks;

    if (!condition) {
        ++failures;
        std::fprintf(stderr, "FAIL: %s\n", contract);
    }
}

template <size_t Size>
void checksum(std::array<uint8_t, Size>& bytes, size_t base) {
    uint32_t sum = 0;

    for (size_t index = 0; index < 127; ++index) {
        sum += bytes.at(base + index);
    }

    bytes.at(base + 127) = static_cast<uint8_t>((256u - (sum & 255u)) & 255u);
}

template <size_t Size>
void set_detailed(std::array<uint8_t, Size>& bytes, size_t offset) {
    // Synthetic 1920x1080p60 DTD, 148.5 MHz, +H/+V, no monitor identity.
    constexpr std::array<uint8_t, 18> detailed{{
        0x02, 0x3a, 0x80, 0x18, 0x71, 0x38, 0x2d, 0x40, 0x58,
        0x2c, 0x45, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1e
    }};

    for (size_t index = 0; index < detailed.size(); ++index) {
        bytes.at(offset + index) = detailed.at(index);
    }
}

Edid fixture(bool extension = false) {
    Edid bytes{};

    for (size_t index = 1; index < 7; ++index) {
        bytes.at(index) = 0xff;
    }

    bytes.at(18) = 1;
    bytes.at(19) = 4;
    bytes.at(20) = 0xa0; // Digital, 8 bpc, unspecified interface.
    bytes.at(24) = 0x02;
    bytes.at(126) = extension ? 1 : 0;
    set_detailed(bytes, 54);
    checksum(bytes, 0);

    if (extension) {
        bytes.at(128) = 2;
        bytes.at(129) = 3;
        bytes.at(130) = 4;
        set_detailed(bytes, 132);
        bytes.at(132) = 0x01; // Distinct clock from base DTD.
        checksum(bytes, 128);
    }

    return bytes;
}

Status parse(const Edid& bytes, size_t size, SinkSnapshot& sink) {
    return parse_edid(ByteView(bytes.data(), size), sink);
}

void check_base() {
    Edid bytes = fixture();
    SinkSnapshot sink{};
    expect(parse(bytes, 128, sink) == Status::success && sink.block_count == 1 &&
           sink.digital_input && sink.bits_per_channel == 8 &&
           sink.detailed_modes.count() == 1 && sink.preferred_mode == 0,
           "base EDID yields one preferred RGB8 candidate");
    Timing timing{};
    expect(sink.detailed_modes.get(0, timing) == rx7900::display::Status::success &&
           timing.pixel_clock_khz == 148500 && timing.width == 1920 &&
           timing.h_total == 2200 && timing.h_front_porch == 88 &&
           timing.h_sync_width == 44 && timing.height == 1080 &&
           timing.v_total == 1125 && timing.v_front_porch == 4 &&
           timing.v_sync_width == 5 && timing.flags ==
               (rx7900::display::positive_hsync | rx7900::display::positive_vsync),
           "DTD bitfields decode directly into existing display Timing");

    for (size_t size = 0; size < 128; ++size) {
        sink.block_count = 99;
        expect(parse(bytes, size, sink) == Status::truncated_input && sink.block_count == 99,
               "every short base EDID is rejected atomically");
    }

    expect(parse_edid(ByteView(nullptr, SIZE_MAX), sink) == Status::truncated_input,
           "null input never claims a readable EDID block");
    bytes.at(0) = 1;
    checksum(bytes, 0);
    expect(parse(bytes, 128, sink) == Status::malformed_edid,
           "header corruption is not silently repaired");
    bytes = fixture();
    bytes.at(25) ^= 1;
    expect(parse(bytes, 128, sink) == Status::checksum_mismatch,
           "base checksum corruption is rejected");
    bytes = fixture();
    bytes.at(19) = 2;
    checksum(bytes, 0);
    expect(parse(bytes, 128, sink) == Status::unsupported_edid,
           "unsupported EDID version has an explicit error");
    bytes = fixture();
    bytes.at(20) = 0xf0;
    checksum(bytes, 0);
    expect(parse(bytes, 128, sink) == Status::unsupported_edid,
           "reserved digital depth cannot imply RGB8");
    bytes = fixture();
    bytes.at(19) = 3;
    bytes.at(20) = 0x80;
    bytes.at(24) = 0;
    checksum(bytes, 0);
    expect(parse(bytes, 128, sink) == Status::success && sink.bits_per_channel == 0 &&
           sink.preferred_mode == no_preferred_mode,
           "EDID 1.3 unknown depth and absent preferred flag remain unknown");
}

void check_detailed_rejection() {
    constexpr std::array<uint8_t, 7> unsupported_flags{{0x9e, 0x3e, 0x5e, 0x1f,
                                                     0x16, 0x0e, 0x06}};
    SinkSnapshot sink{};

    for (uint8_t flags : unsupported_flags) {
        Edid bytes = fixture();
        bytes.at(71) = flags;
        checksum(bytes, 0);
        expect(parse(bytes, 128, sink) == Status::success &&
               sink.detailed_modes.count() == 0 && sink.rejected_detailed_modes == 1 &&
               sink.preferred_mode == no_preferred_mode,
               "interlace, stereo and non-separate sync DTDs are not advertised");
    }

    for (size_t offset : std::array<size_t, 2>{{69, 70}}) {
        Edid bytes = fixture();
        bytes.at(offset) = 1;
        checksum(bytes, 0);
        expect(parse(bytes, 128, sink) == Status::success &&
               sink.detailed_modes.count() == 0 && sink.rejected_detailed_modes == 1,
               "unsupported DTD borders are rejected");
    }

    Edid bytes = fixture();
    bytes.at(62) = 255;
    bytes.at(63) = 255;
    checksum(bytes, 0);
    expect(parse(bytes, 128, sink) == Status::success &&
           sink.detailed_modes.count() == 0 && sink.rejected_detailed_modes == 1,
           "porch and pulse exceeding blanking reuse display geometry rejection");
    bytes = fixture();
    set_detailed(bytes, 72);
    checksum(bytes, 0);
    expect(parse(bytes, 128, sink) == Status::success &&
           sink.detailed_modes.count() == 1 && sink.rejected_detailed_modes == 0,
           "duplicate sink timings reuse existing catalog deduplication");
}

void check_extensions() {
    Edid bytes = fixture(true);
    SinkSnapshot sink{};
    expect(parse(bytes, bytes.size(), sink) == Status::success && sink.block_count == 2 &&
           sink.detailed_modes.count() == 2 && sink.preferred_mode == 0,
           "CTA detailed timing adds a distinct sink candidate");

    for (size_t size = 128; size < bytes.size(); ++size) {
        sink.block_count = 99;
        expect(parse(bytes, size, sink) == Status::truncated_input && sink.block_count == 99,
               "every truncated declared CTA block leaves snapshot unchanged");
    }

    bytes.at(160) ^= 1;
    expect(parse(bytes, 256, sink) == Status::checksum_mismatch,
           "each extension checksum is checked");
    bytes = fixture();
    expect(parse(bytes, 256, sink) == Status::malformed_edid,
           "trailing unadvertised blocks are rejected");
    bytes.at(126) = maximum_edid_blocks;
    checksum(bytes, 0);
    expect(parse(bytes, 256, sink) == Status::unsupported_edid,
           "extension count cannot exceed fixed work budget");
    bytes = fixture(true);
    bytes.at(130) = 3;
    checksum(bytes, 128);
    expect(parse(bytes, 256, sink) == Status::malformed_edid,
           "CTA DTD offset cannot overlap its header");
    bytes.at(130) = 128;
    checksum(bytes, 128);
    expect(parse(bytes, 256, sink) == Status::malformed_edid,
           "CTA DTD offset cannot extend beyond checksum");
    bytes = fixture(true);
    bytes.at(130) = 5;
    bytes.at(132) = 0x45;
    checksum(bytes, 128);
    expect(parse(bytes, 256, sink) == Status::malformed_edid,
           "CTA data-block length cannot consume DTD bytes");
    bytes.at(130) = 7;
    bytes.at(132) = 0xe2;
    bytes.at(133) = 0x78;
    bytes.at(134) = 1;
    checksum(bytes, 128);
    expect(parse(bytes, 256, sink) == Status::unsupported_edid,
           "HF-EEODB count overrides are rejected explicitly");
    bytes = fixture(true);
    bytes.at(128) = 0x70;
    checksum(bytes, 128);
    expect(parse(bytes, 256, sink) == Status::success &&
           sink.unsupported_extensions == 1 && sink.detailed_modes.count() == 1,
           "DisplayID is recorded unsupported without fabricated modes");
    bytes = fixture(true);
    bytes.at(129) = 4;
    checksum(bytes, 128);
    expect(parse(bytes, 256, sink) == Status::success && sink.unsupported_extensions == 1 &&
           sink.detailed_modes.count() == 1, "future CTA revision is not decoded speculatively");
    bytes = fixture(true);
    bytes.at(130) = 127;
    checksum(bytes, 128);
    expect(parse(bytes, 256, sink) == Status::success && sink.detailed_modes.count() == 1,
           "CTA checksum byte never becomes a DTD");
    bytes = fixture(true);

    for (size_t offset = 128; offset < bytes.size(); ++offset) {
        bytes.at(offset) = 0;
    }

    expect(parse(bytes, 256, sink) == Status::malformed_edid,
           "all-zero extension is not accepted merely because its checksum sums to zero");
}

void check_output_budget() {
    const Edid bytes = fixture();
    SinkSnapshot sink{};
    expect(parse(bytes, 128, sink) == Status::success, "prepare sink for output validation");
    Timing timing{};
    timing.width = 77;
    LinkCapabilities link{};
    expect(prepare_output_mode(sink, 0, link, timing) == Status::capabilities_unavailable &&
           timing.width == 77, "absent physical capability evidence cannot enable output");
    link = {Transport::hdmi_tmds, 148500, UINT64_C(3564000000), true};
    expect(prepare_output_mode(sink, 0, link, timing) == Status::success &&
           timing.width == 1920, "exact RGB8 clock and usable payload limits are inclusive");
    --link.usable_payload_bits_per_second;
    timing.width = 77;
    expect(prepare_output_mode(sink, 0, link, timing) == Status::unsupported_output &&
           timing.width == 77, "one bit per second short leaves timing output unchanged");
    link.usable_payload_bits_per_second = UINT64_MAX;
    --link.maximum_pixel_clock_khz;
    expect(prepare_output_mode(sink, 0, link, timing) == Status::unsupported_output,
           "pixel clock limit is checked independently of payload capacity");
    link.maximum_pixel_clock_khz = UINT32_MAX;
    link.transport = Transport::display_port_sst;
    expect(prepare_output_mode(sink, 0, link, timing) == Status::success,
           "DP SST accepts separately verified usable payload");
    link.rgb_8bpc_verified = false;
    expect(prepare_output_mode(sink, 0, link, timing) == Status::capabilities_unavailable,
           "digital EDID does not prove RGB8 on the complete link");
    link.rgb_8bpc_verified = true;
    sink.bits_per_channel = 6;
    expect(prepare_output_mode(sink, 0, link, timing) == Status::unsupported_output,
           "explicit six-bit sink is not sent eight-bit output");
    sink.bits_per_channel = 0;
    expect(prepare_output_mode(sink, 0, link, timing) == Status::success,
           "unknown base depth requires independent end-to-end RGB8 verification");
    sink.digital_input = false;
    expect(prepare_output_mode(sink, 0, link, timing) == Status::unsupported_output,
           "analog sink is unsupported by digital-only output policy");
    expect(prepare_output_mode(sink, UINT32_MAX, link, timing) == Status::mode_not_found,
           "untrusted mode index cannot leave the existing catalog bounds");
}

void check_maximum_blob() {
    std::array<uint8_t, maximum_edid_blocks * edid_block_bytes> bytes{};
    const Edid base = fixture();

    for (size_t index = 0; index < edid_block_bytes; ++index) {
        bytes.at(index) = base.at(index);
    }

    uint8_t clock_low = 2;

    for (size_t index = 0; index < 4; ++index) {
        const size_t offset = 54 + 18 * index;
        set_detailed(bytes, offset);
        bytes.at(offset) = clock_low++;
    }

    bytes.at(126) = static_cast<uint8_t>(maximum_edid_blocks - 1);
    checksum(bytes, 0);

    for (size_t block = 1; block < maximum_edid_blocks; ++block) {
        const size_t block_base = block * edid_block_bytes;
        bytes.at(block_base) = 2;
        bytes.at(block_base + 1) = 3;
        bytes.at(block_base + 2) = 4;

        for (size_t index = 0; index < 6; ++index) {
            const size_t offset = block_base + 4 + 18 * index;
            set_detailed(bytes, offset);
            bytes.at(offset) = clock_low++;
        }

        checksum(bytes, block_base);
    }

    SinkSnapshot sink{};
    expect(parse_edid(ByteView(bytes.data(), bytes.size()), sink) == Status::success &&
           sink.block_count == maximum_edid_blocks && sink.detailed_modes.count() == 46 &&
           sink.rejected_detailed_modes == 0,
           "maximum eight-block blob fits all 46 distinct DTDs in the shared catalog");
}

void check_mutations() {
    const Edid original = fixture(true);
    SinkSnapshot sink{};

    // Exhaust every single-byte change while fixing that block's checksum. This
    // traverses DTD high fields, flags, CTA offsets and block lengths under ASan.
    for (size_t offset = 0; offset < original.size(); ++offset) {
        for (uint32_t value = 0; value < 256; ++value) {
            Edid bytes = original;
            bytes.at(offset) = static_cast<uint8_t>(value);
            checksum(bytes, offset < 128 ? 0 : 128);
            const Status status = parse(bytes, bytes.size(), sink);

            if (status == Status::success) {
                expect(sink.detailed_modes.count() <= rx7900::display::mode_capacity,
                       "mutated EDID can never exceed fixed catalog capacity");
            }
        }
    }
}

void check_private_edid(const char* path) {
    // Optional local fixture may contain monitor identity. Never print its bytes
    // or identity; keep it outside source and published deliverables.
    std::FILE* file = std::fopen(path, "rb");
    expect(file != nullptr, "open optional private EDID fixture");

    if (file == nullptr) {
        return;
    }

    std::array<uint8_t, maximum_edid_blocks * edid_block_bytes> bytes{};
    const size_t size = std::fread(bytes.data(), 1, bytes.size(), file);
    const bool too_large = std::fgetc(file) != EOF;
    const bool read_failed = std::ferror(file) != 0;
    std::fclose(file);
    expect(!too_large && !read_failed, "private fixture fits bounded input");

    if (too_large || read_failed) {
        return;
    }

    SinkSnapshot sink{};
    const Status status = parse_edid(ByteView(bytes.data(), size), sink);
    expect(status == Status::success && sink.detailed_modes.count() != 0,
           "real host monitor EDID supplies usable detailed timing candidates");
    std::printf("private EDID: %u blocks, %u detailed candidates, %u rejected, "
                "%u unsupported extensions; no identity retained\n", sink.block_count,
                sink.detailed_modes.count(), sink.rejected_detailed_modes,
                sink.unsupported_extensions);
}

void set_word(AtomRom& rom, size_t offset, uint16_t value) {
    rom.at(offset) = static_cast<uint8_t>(value & 0xffu);
    rom.at(offset + 1) = static_cast<uint8_t>(value >> 8);
}

void set_table(AtomRom& rom, size_t offset, uint16_t bytes,
               uint8_t format, uint8_t revision) {
    set_word(rom, offset, bytes);
    rom.at(offset + 2) = format;
    rom.at(offset + 3) = revision;
}

AtomRom atom_fixture(uint8_t object_revision = 5) {
    AtomRom rom{};
    set_word(rom, 0, 0xaa55);
    set_word(rom, 0x48, 0x80);
    set_table(rom, 0x80, 40, 2, 2);
    rom.at(0x84) = 'A';
    rom.at(0x85) = 'T';
    rom.at(0x86) = 'O';
    rom.at(0x87) = 'M';
    set_word(rom, 0xa0, 0x100);
    set_table(rom, 0x100, 74, 2, 1);
    set_word(rom, 0x11c, 0x300);
    set_word(rom, 0x130, 0x200);
    set_table(rom, 0x200, 50, 1, object_revision);
    rom.at(0x206) = 2; // One connector and one generic path without an encoder.
    set_word(rom, 0x208, 0x310c);
    set_word(rom, 0x20a, 40);
    set_word(rom, 0x20c, 0x2121);
    set_word(rom, 0x214, 0x0080);
    set_word(rom, 0x218, 1);
    rom.at(0x228) = 1;
    rom.at(0x229) = 4;
    rom.at(0x22a) = 0x92;
    rom.at(0x22b) = 0;
    rom.at(0x22c) = 2;
    rom.at(0x22d) = 4;
    rom.at(0x22e) = 1;
    rom.at(0x22f) = 1;
    rom.at(0x230) = 0xff;
    set_table(rom, 0x300, 20, 2, 1);
    set_word(rom, 0x304, 0x2345);
    set_word(rom, 0x306, 1);
    rom.at(0x308) = 3;
    rom.at(0x309) = 4;
    rom.at(0x30a) = 0x92;
    set_word(rom, 0x30c, 0x4321);
    set_word(rom, 0x30e, 5);
    rom.at(0x310) = 5;
    rom.at(0x311) = 6;
    rom.at(0x312) = 1;
    return rom;
}

FirmwareStatus parse_atom(const AtomRom& rom, ConnectorCatalog& connectors) {
    return parse_connectors(ByteView(rom.data(), rom.size()), connectors);
}

void check_atom_connectors() {
    AtomRom rom = atom_fixture();
    ConnectorCatalog connectors{};
    expect(parse_atom(rom, connectors) == FirmwareStatus::success && connectors.count == 1,
           "ATOM1.5 object table enumerates actual connector and skips generic path");
    const Connector connector = connectors.connectors[0];
    expect(connector.kind == ConnectorKind::hdmi_type_a &&
           connector.connector_object_id == 0x310c && connector.encoder_object_id == 0x2121 &&
           connector.device_tag == 0x0080 && connector.firmware_path_index == 0 &&
           connector.has_i2c_record && connector.i2c_id == 0x92 &&
           connector.has_hpd_record && connector.hpd_pin_id == 1 &&
           connector.hpd_plugged_level == 1,
           "firmware connector, encoder, DDC and HPD records retain explicit fields");
    expect(connector.ddc_gpio.found && connector.hpd_gpio.found &&
           connector.ddc_gpio.register_index == 0x12345 &&
           connector.ddc_gpio.bit_shift == 3 && connector.ddc_gpio.mask_bit_shift == 4 &&
           connector.hpd_gpio.register_index == 0x54321 &&
           connector.hpd_gpio.bit_shift == 5,
           "GPIO LUT resolves routing without truncating the 32-bit register index");

    for (size_t size = 0; size < 0x314; ++size) {
        connectors.count = 99;
        const auto status = parse_connectors(ByteView(rom.data(), size), connectors);
        expect(status != FirmwareStatus::success && connectors.count == 99,
               "every missing byte through required GPIO table rejects atomically");
    }

    expect(parse_connectors(ByteView(nullptr, SIZE_MAX), connectors) ==
               FirmwareStatus::truncated_input,
           "null ROM cannot claim a complete firmware table");
    rom = atom_fixture(4);
    set_word(rom, 0x20e, 0x2221);
    expect(parse_atom(rom, connectors) == FirmwareStatus::success &&
           connectors.connectors[0].external_encoder_object_id == 0x2221,
           "ATOM1.4 retains real external encoder routing");
    rom = atom_fixture();
    set_word(rom, 0x11c, 0);
    expect(parse_atom(rom, connectors) == FirmwareStatus::success &&
           !connectors.connectors[0].ddc_gpio.found &&
           !connectors.connectors[0].hpd_gpio.found,
           "absent GPIO table remains unresolved without invented GPIO addresses");
    rom = atom_fixture();
    set_word(rom, 0x20a, 0);
    expect(parse_atom(rom, connectors) == FirmwareStatus::success &&
           !connectors.connectors[0].has_i2c_record &&
           !connectors.connectors[0].has_hpd_record,
           "absent connector records do not imply a connected sink");
    rom = atom_fixture();
    rom.at(0x203) = 6;
    expect(parse_atom(rom, connectors) == FirmwareStatus::unsupported_revision,
           "unknown ATOM object table revision is rejected");
    rom = atom_fixture();
    rom.at(0x206) = connector_capacity + 1;
    expect(parse_atom(rom, connectors) == FirmwareStatus::capacity_exceeded,
           "firmware path count cannot exceed fixed topology capacity");
    rom = atom_fixture();
    set_word(rom, 0x200, 39);
    expect(parse_atom(rom, connectors) == FirmwareStatus::invalid_table,
           "path array must fit the declared object table");
    rom = atom_fixture();
    set_word(rom, 0x20a, 8);
    expect(parse_atom(rom, connectors) == FirmwareStatus::invalid_table,
           "record pointer cannot alias firmware path bytes");
    rom = atom_fixture();
    rom.at(0x229) = 1;
    expect(parse_atom(rom, connectors) == FirmwareStatus::invalid_table,
           "record length cannot be smaller than the record header");
    rom.at(0x229) = 255;
    expect(parse_atom(rom, connectors) == FirmwareStatus::invalid_table,
           "record length cannot escape its declared table");
    rom = atom_fixture();
    rom.at(0x22c) = 1;
    expect(parse_atom(rom, connectors) == FirmwareStatus::invalid_table,
           "duplicate I2C assignments are rejected as ambiguous");
    rom = atom_fixture();
    rom.at(0x22f) = 2;
    expect(parse_atom(rom, connectors) == FirmwareStatus::invalid_table,
           "HPD plugged level must be a single logical state");
    rom = atom_fixture();
    rom.at(0x308) = 32;
    expect(parse_atom(rom, connectors) == FirmwareStatus::invalid_table,
           "GPIO shift cannot address outside a 32-bit register");
    rom = atom_fixture();
    set_word(rom, 0x218, 0x310c);
    set_word(rom, 0x21c, 0x2221);
    expect(parse_atom(rom, connectors) == FirmwareStatus::duplicate_connector,
           "duplicate connector object IDs are rejected");

    const AtomRom original = atom_fixture();

    for (size_t offset = 0; offset < 0x314; ++offset) {
        for (uint32_t value = 0; value < 256; ++value) {
            rom = original;
            rom.at(offset) = static_cast<uint8_t>(value);
            connectors.count = 99;
            const auto status = parse_atom(rom, connectors);
            expect(status == FirmwareStatus::success ? connectors.count <= connector_capacity :
                                                      connectors.count == 99,
                   "mutated ATOM firmware preserves bounds and transactional failure");
        }
    }
}

} // namespace

int main(int argc, char** argv) {
    check_base();
    check_detailed_rejection();
    check_extensions();
    check_output_budget();
    check_maximum_blob();
    check_mutations();
    check_atom_connectors();

    if (argc == 2) {
        check_private_edid(argv[1]);
    }

    std::printf("output: %u checks, %u failures; no hardware operations\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

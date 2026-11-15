/*
Module: rx7900.tests.ucode
Purpose: Verify PSP payload selection, Linux order, types and header bounds.
Created: 2026-10-21
Architecture: Synthetic firmware headers; no AMD firmware payload is embedded.
*/
#include <string_view>
#include <vector>

#include "asic_test_device.hpp"
#include "ucode_test_files.hpp"

namespace {

using namespace rx7900::hardware;
using rx7900::ByteView;
using namespace rx7900_test::ucode_files;

rx7900_test::Checks checks;

void expect(bool condition, const char* contract) { checks.expect(condition, contract); }

void cp_and_order() {
    UcodeCatalog catalog;
    const File pfp = gfx(0x40, 0x20);
    const File me = gfx(0x40, 0x20);
    const File mec = gfx(0x80, 0x40);
    const File rlc_file = rlc(3);
    const File mes_file = mes();
    const File kiq = mes();
    const File imu_file = imu();
    const File smu_file = smu();
    const File sdma_file = sdma();
    const File dmcub_file = dmcub();

    expect(catalog.add(UcodeSource::rlc, rlc_file.view()) == UcodeStatus::success &&
           catalog.add(UcodeSource::dmcub, dmcub_file.view()) == UcodeStatus::success &&
           catalog.add(UcodeSource::mec, mec.view()) == UcodeStatus::success &&
           catalog.add(UcodeSource::pfp, pfp.view()) == UcodeStatus::success &&
           catalog.add(UcodeSource::me, me.view()) == UcodeStatus::success &&
           catalog.add(UcodeSource::mes, mes_file.view()) == UcodeStatus::success &&
           catalog.add(UcodeSource::mes_kiq, kiq.view()) == UcodeStatus::success &&
           catalog.add(UcodeSource::imu, imu_file.view()) == UcodeStatus::success &&
           catalog.add(UcodeSource::smu, smu_file.view()) == UcodeStatus::success &&
           catalog.add(UcodeSource::sdma, sdma_file.view()) == UcodeStatus::success,
           "every Navi31 PSP source decodes");

    const uint8_t expected_ids[]{9, 10, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 30, 31, 32,
                                 33, 34, 35, 41, 43, 44, 45, 46, 47, 48, 50, 61};
    const uint8_t expected_types[]{71, 72, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 33, 34, 81,
                                   82, 68, 69, 22, 21, 26, 48, 25, 7, 8, 18, 51};
    bool ordered = catalog.count() == sizeof(expected_ids);

    for (size_t index = 0; ordered && index < sizeof(expected_ids); ++index) {
        ordered = catalog.entry(index).linux_id == expected_ids[index] &&
                  catalog.entry(index).psp_type == expected_types[index];
    }

    expect(ordered, "entries follow AMDGPU_UCODE_ID order with psp_get_fw_type types");
    expect(catalog.entry(catalog.count()).bytes == 0, "out-of-range entry is empty");

    UcodeEntry entry{};
    ByteView payload(nullptr, 0);
    expect(catalog.find(15, entry) && entry.offset == 0x100 && entry.bytes == 0x40,
           "RS64 PFP code uses ucode_array_offset and ucode_size_bytes");
    expect(catalog.find(19, entry) && entry.offset == 0x140 && entry.bytes == 0x20,
           "PFP P1 stack repeats the shared data section");
    expect(catalog.find(25, entry) && entry.offset == 0x180 && entry.bytes == 0x40,
           "MEC P3 stack decoded");
    expect(catalog.find(35, entry) && entry.offset == 0x180 && entry.bytes == 0x60,
           "IMU data follows the IRAM image from the array offset");
    expect(!catalog.find(42, entry), "zero-sized GPM restore list omitted");
    expect(catalog.find(9, entry) && entry.offset == 0x100 && entry.bytes == 0x20 &&
           catalog.find(10, entry) && entry.offset == 0x120,
           "SDMA thread payloads reuse the SDMA container parser");
    expect(catalog.find(61, entry) && entry.bytes == 0x30, "DMCUB uses inst_const_bytes");
    expect(catalog.find(ucode_id::smc, entry) && catalog.payload(entry, payload) &&
           payload.size() == 0x200, "SMC payload is a borrowed checked view");
    expect(catalog.add(UcodeSource::pfp, pfp.view()) == UcodeStatus::duplicate_source,
           "one file per source");

    catalog.reset();
    expect(catalog.count() == 0 && !catalog.has(UcodeSource::pfp) && !catalog.payload(entry, payload),
           "reset forgets every borrowed file");
}

void rlc_versions() {
    UcodeCatalog catalog;
    const File v4 = rlc(4);
    UcodeEntry entry{};
    expect(catalog.add(UcodeSource::rlc, v4.view()) == UcodeStatus::success &&
           catalog.find(36, entry) && entry.psp_type == 27 && catalog.find(37, entry) &&
           !catalog.find(46, entry), "RLC v2.4 adds tap delays but not v2.3 RLC_P/V");

    catalog.reset();
    const File v0 = rlc(0);
    expect(catalog.add(UcodeSource::rlc, v0.view()) == UcodeStatus::success && catalog.count() == 1,
           "RLC v2.0 provides RLC_G only");

    catalog.reset();
    File v5 = rlc(4);
    v5.u16(10, 5);
    expect(catalog.add(UcodeSource::rlc, v5.view()) == UcodeStatus::unsupported_version,
           "RLC v2.5 is not guessed");

    File truncated(150, 2, 3, 11, 0, 0xcc);
    expect(catalog.add(UcodeSource::rlc, truncated.view()) == UcodeStatus::truncated_input,
           "RLC v2.3 fields must be present");
}

void malformed() {
    UcodeCatalog catalog;
    File legacy = gfx(0x40, 0x20);
    legacy.u16(8, 1);
    expect(catalog.add(UcodeSource::pfp, legacy.view()) == UcodeStatus::unsupported_version,
           "non-RS64 CP firmware is refused");

    File wrong_ip = mes();
    wrong_ip.u16(12, 12);
    expect(catalog.add(UcodeSource::mes, wrong_ip.view()) == UcodeStatus::wrong_ip,
           "GC12 MES is refused");

    File size = smu();
    size.u32(0, 0x401);
    expect(catalog.add(UcodeSource::smu, size.view()) == UcodeStatus::malformed_image,
           "declared size must equal the file");

    File range = gfx(0x40, 0x20);
    range.u32(48, 0x3f0);
    expect(catalog.add(UcodeSource::me, range.view()) == UcodeStatus::malformed_image &&
           !catalog.has(UcodeSource::me) && catalog.count() == 0,
           "out-of-file stack range is refused atomically");

    File zero = imu();
    zero.u32(40, 0);
    expect(catalog.add(UcodeSource::imu, zero.view()) == UcodeStatus::malformed_image,
           "mandatory zero-sized IMU data is refused");

    File overflow = imu();
    overflow.u32(24, 0xfffffff0u);
    expect(catalog.add(UcodeSource::imu, overflow.view()) == UcodeStatus::malformed_image,
           "IMU offset arithmetic cannot wrap");

    const File tiny(16, 2, 0, 11, 0, 60);
    expect(catalog.add(UcodeSource::pfp, {tiny.bytes.data(), 16}) == UcodeStatus::truncated_input,
           "short common header refused");
    expect(catalog.add(UcodeSource::pfp, {nullptr, 0}) == UcodeStatus::invalid_argument,
           "empty input refused");
}

void names_and_ta() {
    expect(std::string_view(ucode_filename(UcodeSource::rlc, 0xc8, false)) == "gc_11_0_0_rlc.bin" &&
           std::string_view(ucode_filename(UcodeSource::rlc, 0xce, false)) == "gc_11_0_0_rlc_1.bin",
           "rlc_1 only for PCI revision 0xCE");
    expect(std::string_view(ucode_filename(UcodeSource::mes, 0, false)) == "gc_11_0_0_mes_2.bin" &&
           std::string_view(ucode_filename(UcodeSource::mes, 0, true)) == "gc_11_0_0_mes.bin",
           "MES scheduler prefers _mes_2 with Linux fallback");
    expect(std::string_view(ucode_filename(UcodeSource::sdma, 0, false)) == "sdma_6_0_0.bin",
           "SDMA name reused from the firmware module");

    const File file = ta();
    uint32_t offset = 0;
    uint32_t bytes = 0;
    expect(find_ta_asd(file.view(), offset, bytes) == UcodeStatus::success &&
           offset == 0x110 && bytes == 0x60, "ASD found after other TA descriptors");

    File bad = ta();
    bad.u32(64, 0x200);
    expect(find_ta_asd(bad.view(), offset, bytes) == UcodeStatus::malformed_image &&
           offset == 0x110, "ASD outside the array refused, output unchanged");
}

void ta_descriptors() {
    const File file = ta_navi31();
    TaDescriptors descriptors{};
    expect(parse_ta_descriptors(file.view(), descriptors) == UcodeStatus::success,
           "Navi31 TA 2.0 image with ASD/HDCP/DTM/RAS parses");
    bool layout = true;

    for (const auto& expected : navi31_ta_layout) {
        const TaBinary binary = descriptors.get(expected.type);
        layout = layout && binary.present && binary.fw_version == expected.version &&
                 binary.offset == 0x100 + expected.relative && binary.bytes == expected.bytes;
    }

    expect(layout, "each binary at ucode_array_offset plus offset_bytes with its version");
    expect(!descriptors.get(ta_fw_type::xgmi).present && !descriptors.get(ta_fw_type::rap).present &&
           !descriptors.get(ta_fw_type::securedisplay).present && !descriptors.get(0).present &&
           !descriptors.get(99).present, "XGMI, RAP and secure display absent; bad types empty");

    uint32_t offset = 0;
    uint32_t bytes = 0;
    expect(find_ta_asd(file.view(), offset, bytes) == UcodeStatus::success && offset == 0x100 &&
           bytes == 0x100, "find_ta_asd reuses the descriptor parser");

    File duplicate = ta_navi31();
    duplicate.u32(36 + 3 * 16, ta_fw_type::hdcp);
    expect(parse_ta_descriptors(duplicate.view(), descriptors) == UcodeStatus::success &&
           descriptors.get(ta_fw_type::hdcp).offset == 0x340 &&
           !descriptors.get(ta_fw_type::ras).present,
           "a later descriptor of the same type replaces the earlier one");

    File unknown = ta_navi31();
    unknown.u32(36 + 2 * 16, 9);
    unknown.u32(36 + 2 * 16 + 12, 0xffffff);
    expect(parse_ta_descriptors(unknown.view(), descriptors) == UcodeStatus::success &&
           !descriptors.get(ta_fw_type::dtm).present, "unknown TA types are ignored unchecked");

    File empty_ras = ta_navi31();
    empty_ras.u32(36 + 3 * 16 + 12, 0);
    expect(parse_ta_descriptors(empty_ras.view(), descriptors) == UcodeStatus::success &&
           descriptors.get(ta_fw_type::ras).present && descriptors.get(ta_fw_type::ras).bytes == 0,
           "zero-sized descriptor is present but unavailable");

    File outside = ta_navi31();
    outside.u32(36 + 3 * 16 + 12, 0x300 - 0x240 + 1);
    TaDescriptors unchanged{};
    unchanged.binary[ta_fw_type::rap].bytes = 7;
    expect(parse_ta_descriptors(outside.view(), unchanged) == UcodeStatus::malformed_image &&
           unchanged.binary[ta_fw_type::rap].bytes == 7,
           "RAS one byte past the ucode array fails closed, output unchanged");

    File many = ta_navi31();
    many.u32(32, ta_max_packaged_binaries);
    expect(parse_ta_descriptors(many.view(), descriptors) == UcodeStatus::malformed_image,
           "ta_fw_bin_count at UCODE_MAX_PSP_PACKAGING (26) refused");

    File limit = ta_navi31();
    limit.u32(32, ta_max_packaged_binaries - 1);
    expect(parse_ta_descriptors(limit.view(), descriptors) == UcodeStatus::malformed_image,
           "25 descriptors overlapping the ucode array refused");

    expect(parse_ta_descriptors(ta_navi31(0).view(), descriptors) == UcodeStatus::success &&
           !descriptors.get(ta_fw_type::asd).present &&
           find_ta_asd(ta_navi31(0).view(), offset, bytes) == UcodeStatus::malformed_image,
           "zero descriptors parse; ASD then absent");

    File v1 = ta_navi31();
    v1.u16(8, 1);
    expect(parse_ta_descriptors(v1.view(), descriptors) == UcodeStatus::unsupported_version,
           "TA header 1.x is not accepted for PSP 13.0.0");
}

} // namespace

int main() {
    cp_and_order();
    rlc_versions();
    malformed();
    names_and_ta();
    ta_descriptors();
    return checks.finish("ucode");
}

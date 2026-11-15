/*
Module: rx7900.tests.radeon_installer
Purpose: Verify radeonctl policy, the INF contract and every installer command,
including crash-at-every-mutation recovery matrices, over the fake system.
Created: 2026-10-24
Architecture: Host ASan/UBSan and GCC13 MinGW (Wine) fixture. Uses
radeon_installer_fake_system.hpp; no registry, file, service or device of the
machine running the test is touched. Optional arguments name real win32k.sys
images and their expected capability masks for read-only scanner evidence.
*/
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "pci.hpp"
#include "radeon_installer_fake_system.hpp"
#if defined(_WIN32)
#include "radeon_win32_system.hpp"
#endif
#include "radeon_opengl_info.hpp"
#include "ucode_test_files.hpp"

namespace {

namespace radeon = rx7900::tools::radeon;
using rx7900_test::radeon_fake::Bytes;
using rx7900_test::radeon_fake::FakeSystem;
using rx7900_test::radeon_fake::Snapshot;

unsigned checks = 0;
unsigned failures = 0;

void expect(bool condition, const char* contract) {
    ++checks;
    if (!condition) {
        ++failures;
        std::printf("FAIL: %s\n", contract);
    }
}

void put16(Bytes& bytes, size_t offset, uint16_t value) {
    bytes.at(offset) = static_cast<uint8_t>(value);
    bytes.at(offset + 1) = static_cast<uint8_t>(value >> 8);
}

void put32(Bytes& bytes, size_t offset, uint32_t value) {
    for (size_t index = 0; index < 4; ++index) {
        bytes.at(offset + index) = static_cast<uint8_t>(value >> (8 * index));
    }
}

// Minimal PE image: DOS stub, PE signature, COFF header and the optional header
// fields check_pe_headers reads. tag distinguishes package versions.
Bytes make_pe(uint16_t machine, radeon::ImageKind kind, uint8_t tag) {
    Bytes bytes(1024, 0);
    put16(bytes, 0, 0x5a4d);
    put32(bytes, 0x3c, 0x80);
    put32(bytes, 0x80, 0x00004550);
    put16(bytes, 0x84, machine);
    put16(bytes, 0x80 + 20, machine == radeon::machine_amd64 ? 0xf0 : 0xe0);
    put16(bytes, 0x80 + 22, static_cast<uint16_t>(0x0102 | (kind == radeon::ImageKind::kernel_driver ? 0 : 0x2000)));
    put16(bytes, 0x80 + 24, machine == radeon::machine_amd64 ? 0x20b : 0x10b);
    put16(bytes, 0x80 + 24 + 68, kind == radeon::ImageKind::user_dll ? 2 : 1);
    for (size_t index = 0x200; index < bytes.size(); ++index) {
        bytes[index] = static_cast<uint8_t>(index * 7 + tag);
    }
    return bytes;
}

void put_package(FakeSystem& fake, uint8_t tag, uint16_t machine = radeon::machine_i386) {
    for (size_t index = 0; index < radeon::component_count; ++index) {
        fake.put_file(fake.packaged(radeon::component_files[index].filename),
                      make_pe(machine, radeon::component_files[index].image,
                              static_cast<uint8_t>(tag + index)));
    }
    const char inf[] = "; radeondisplay.inf test copy\r\n";
    fake.put_file(fake.packaged(radeon::inf_filename), Bytes(inf, inf + sizeof(inf) - 1));
}

Bytes psp_sos_fixture() {
    // Same synthetic PSP SOS 2.0/IP13 layout as tests/firmware_tests.cpp.
    Bytes bytes(512, 0);
    const uint32_t fields[][2]{{0, 512}, {4, 36}, {8, 2}, {12, 13}, {16, 27}, {20, 256}, {24, 256},
                               {32, 2}, {36, 2}, {40, 27}, {44, 0}, {48, 128}, {52, 1}, {56, 27},
                               {60, 128}, {64, 128}};
    for (const auto& field : fields) {
        put32(bytes, field[0], field[1]);
    }
    return bytes;
}

Bytes atom_rom_fixture(uint16_t device) {
    // Same structure as tests/common_tests.cpp check_atom_image.
    Bytes bytes(1024, 0);
    put16(bytes, 0, 0xaa55);
    put16(bytes, 0x18, 0x80);
    put16(bytes, 0x48, 0xc0);
    put32(bytes, 0x80, 0x52494350);
    put16(bytes, 0x84, rx7900::amd_vendor_id);
    put16(bytes, 0x86, device);
    put16(bytes, 0x8a, 0x18);
    bytes.at(0x8f) = 3;
    put16(bytes, 0x90, 1);
    put32(bytes, 0xc4, 0x4d4f5441);
    return bytes;
}

void put_firmware(FakeSystem& fake, bool required_only = false) {
    namespace files = rx7900_test::ucode_files;
    fake.directories.insert(rx7900_test::radeon_fake::fold((fake.drivers() + L"\\rx7900").c_str()));
    fake.put_file(fake.firmware("gc_11_0_0_pfp.bin"), files::gfx(0x40, 0x20).bytes);
    fake.put_file(fake.firmware("gc_11_0_0_me.bin"), files::gfx(0x40, 0x20).bytes);
    fake.put_file(fake.firmware("gc_11_0_0_mec.bin"), files::gfx(0x80, 0x40).bytes);
    fake.put_file(fake.firmware("gc_11_0_0_rlc.bin"), files::rlc(3).bytes);
    fake.put_file(fake.firmware("gc_11_0_0_mes_2.bin"), files::mes().bytes);
    fake.put_file(fake.firmware("gc_11_0_0_mes1.bin"), files::mes().bytes);
    fake.put_file(fake.firmware("gc_11_0_0_imu.bin"), files::imu().bytes);
    fake.put_file(fake.firmware("sdma_6_0_0.bin"), files::sdma().bytes);
    fake.put_file(fake.firmware("smu_13_0_0.bin"), files::smu().bytes);
    fake.put_file(fake.firmware("psp_13_0_0_sos.bin"), psp_sos_fixture());
    if (!required_only) {
        fake.put_file(fake.firmware("dcn_3_2_0_dmcub.bin"), files::dmcub().bytes);
        fake.put_file(fake.firmware("psp_13_0_0_ta.bin"), files::ta_navi31().bytes);
        fake.put_file(fake.firmware("rx7900.rom"), atom_rom_fixture(0x744c));
    }
}

int run(FakeSystem& fake, std::vector<const wchar_t*> arguments) {
    arguments.insert(arguments.begin(), L"radeonctl");
    radeon::CommandLine command{};
    if (!radeon::parse_command_line(static_cast<int>(arguments.size()), arguments.data(), command)) {
        return radeon::exit_code::usage;
    }
    const radeon::SystemOps ops = fake.ops();
    return radeon::run_command(ops, command);
}

bool journal_state(const FakeSystem& fake, radeon::JournalState expected) {
    uint32_t state = 0;
    if (!fake.dword(radeon::journal_key, L"State", state)) {
        return expected == radeon::JournalState::none;
    }
    return state == static_cast<uint32_t>(expected);
}

bool no_transaction_leftovers(const FakeSystem& fake) {
    for (const auto& file : fake.files) {
        if (file.first.find(L".radeonctl-") != std::wstring::npos) {
            bool pending = false;
            for (const auto& rename : fake.pending_renames) {
                pending = pending || rename.first == file.first;
            }
            if (!pending) {
                return false;
            }
        }
    }
    return true;
}

// Null-safe content comparison: an absent file never equals anything.
bool same_bytes(const Bytes* file, const Bytes& expected) {
    return file != nullptr && *file == expected;
}

Bytes contents(const Bytes* file) {
    return file != nullptr ? *file : Bytes();
}

bool live_equals_package(const FakeSystem& fake, size_t component) {
    const Bytes* live = fake.file(fake.live(component));
    const Bytes* packaged = fake.file(fake.packaged(radeon::component_files[component].filename));
    return live != nullptr && packaged != nullptr && *live == *packaged;
}

bool feature_values(const FakeSystem& fake, uint32_t features) {
    for (const auto& rule : radeon::feature_rules) {
        if (rule.registry_value == nullptr) {
            continue;
        }
        uint32_t value = 99;
        if (!fake.dword(radeon::device0_key, rule.registry_value, value) ||
            value != ((features & rule.feature) != 0 ? 1u : 0u)) {
            return false;
        }
    }
    return true;
}

bool icd_registered(const FakeSystem& fake) {
    uint32_t version = 0;
    uint32_t driver_version = 0;
    return fake.string(radeon::opengl_drivers_key, L"Dll") == L"radeonglicd.dll" &&
           fake.dword(radeon::opengl_drivers_key, L"Version", version) && version == 1 &&
           fake.dword(radeon::opengl_drivers_key, L"DriverVersion", driver_version) && driver_version == 1;
}

bool installed(const FakeSystem& fake, uint32_t features) {
    if (fake.devices.empty()) {
        return false;
    }
    const auto& device = fake.devices.front();
    bool files = live_equals_package(fake, 0) && live_equals_package(fake, 1);
    if ((features & radeon::feature_opengl_icd) != 0) {
        files = files && live_equals_package(fake, radeon::icd_component) && icd_registered(fake);
    } else {
        files = files && !fake.has_file(fake.live(radeon::icd_component)) &&
                fake.value(radeon::opengl_drivers_key, L"Dll") == nullptr;
    }
    return journal_state(fake, radeon::JournalState::installed) && device.present &&
           radeon::text_equal_insensitive(device.record.service, radeon::service_name) && fake.service.exists &&
           fake.service.start == 3 && fake.service.type == 1 && files && feature_values(fake, features) &&
           fake.string(radeon::device0_key, L"InstalledDisplayDrivers") == L"radeondisp" &&
           no_transaction_leftovers(fake);
}

bool clean(const FakeSystem& fake, const std::wstring& original_service) {
    bool files = true;
    for (size_t index = 0; index < radeon::component_count; ++index) {
        files = files && !fake.has_file(fake.live(index));
    }
    const bool binding = fake.devices.empty() || !fake.devices.front().present ||
                         original_service == fake.devices.front().record.service;
    return journal_state(fake, radeon::JournalState::none) && fake.keys.count(L"software\\rx7900\\radeoninstaller") == 0 &&
           !fake.service.exists && files && no_transaction_leftovers(fake) &&
           fake.value(radeon::opengl_drivers_key, L"Dll") == nullptr && binding;
}

// ---- Pure policy ----

void sha256_vectors() {
    struct Vector final {
        const char* input;
        const char* digest;
    };
    const Vector vectors[]{
        {"", "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"},
        {"abc", "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"},
        {"abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq",
         "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1"},
    };
    for (const auto& vector : vectors) {
        rx7900::tools::Sha256 hasher;
        hasher.update(reinterpret_cast<const uint8_t*>(vector.input), std::strlen(vector.input));
        char text[rx7900::tools::sha256_hex_characters]{};
        rx7900::tools::sha256_hex(hasher.finish(), text);
        expect(std::strcmp(text, vector.digest) == 0, "SHA-256 matches FIPS 180-4 vectors");
    }

    // One million 'a' bytes, fed in uneven chunks across block boundaries.
    rx7900::tools::Sha256 hasher;
    const std::vector<uint8_t> block(997, 'a');
    size_t total = 0;
    while (total < 1000000) {
        const size_t count = 1000000 - total < block.size() ? 1000000 - total : block.size();
        hasher.update(block.data(), count);
        total += count;
    }
    char text[rx7900::tools::sha256_hex_characters]{};
    rx7900::tools::sha256_hex(hasher.finish(), text);
    expect(std::strcmp(text, "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0") == 0,
           "SHA-256 streaming over uneven chunks");

    rx7900::tools::Sha256Digest parsed{};
    wchar_t wide[rx7900::tools::sha256_hex_characters]{};
    rx7900::tools::sha256_hex(rx7900::tools::Sha256().finish(), wide);
    expect(rx7900::tools::parse_sha256_hex(wide, parsed) && parsed == rx7900::tools::Sha256().finish(),
           "hex digest round-trips");
    wide[5] = L'g';
    expect(!rx7900::tools::parse_sha256_hex(wide, parsed), "non-hex digest text is rejected");
}

void feature_policy() {
    uint32_t requirement = 0;
    expect(radeon::invalid_feature(0, requirement) == 0, "no features is valid (all opt-ins 0)");
    expect(radeon::invalid_feature(radeon::known_features, requirement) == 0, "every feature together is valid");
    expect(radeon::invalid_feature(radeon::feature_radeon_2d, requirement) == radeon::feature_radeon_2d &&
               requirement == radeon::feature_physical_asic_init, "Radeon2D requires PhysicalAsicInit");
    expect(radeon::invalid_feature(radeon::feature_physical_asic_init | radeon::feature_radeon_2d_compute,
                                   requirement) == radeon::feature_radeon_2d_compute &&
               requirement == radeon::feature_radeon_2d, "Radeon2DCompute requires Radeon2D");
    expect(radeon::invalid_feature(radeon::feature_vbe_mode_set, requirement) == radeon::feature_vbe_mode_set,
           "VbeModeSet requires PhysicalAsicInit");
    expect(radeon::invalid_feature(radeon::feature_hardware_cursor, requirement) == radeon::feature_hardware_cursor,
           "HardwareCursor requires PhysicalAsicInit");
    expect(radeon::invalid_feature(radeon::feature_atom_rom_override | radeon::feature_opengl_icd, requirement) == 0,
           "AtomRomOverride and the ICD are independent opt-ins");
    expect(radeon::invalid_feature(1u << 20, requirement) == (1u << 20), "unknown feature bits are rejected");
}

void command_line_policy() {
    radeon::CommandLine command{};
    const wchar_t* install[]{L"radeonctl", L"install", L"--asic", L"--2d", L"--opengl-icd", L"--force-replace",
                             L"vbemp", L"--dry-run"};
    expect(radeon::parse_command_line(8, install, command) && command.command == radeon::Command::install &&
               command.features == (radeon::feature_physical_asic_init | radeon::feature_radeon_2d |
                                    radeon::feature_opengl_icd) &&
               command.force_replace && radeon::text_equal_insensitive(command.force_service, L"vbemp") &&
               command.dry_run, "install options parse into features and the named replacement");
    const wchar_t* repeated[]{L"radeonctl", L"install", L"--asic", L"--asic"};
    expect(!radeon::parse_command_line(4, repeated, command), "repeated options are rejected");
    const wchar_t* missing[]{L"radeonctl", L"install", L"--force-replace"};
    expect(!radeon::parse_command_line(3, missing, command), "--force-replace requires a service name");
    const wchar_t* option_as_name[]{L"radeonctl", L"install", L"--force-replace", L"--asic"};
    expect(!radeon::parse_command_line(4, option_as_name, command), "an option is never taken as a service name");
    const wchar_t* foreign[]{L"radeonctl", L"upgrade", L"--asic"};
    expect(!radeon::parse_command_line(3, foreign, command), "feature flags belong to install only");
    const wchar_t* unknown[]{L"radeonctl", L"bind"};
    expect(!radeon::parse_command_line(2, unknown, command) && command.error != nullptr,
           "unknown commands are rejected and named");
    const wchar_t* status[]{L"radeonctl", L"STATUS"};
    expect(radeon::parse_command_line(2, status, command) && command.command == radeon::Command::status,
           "commands are case-insensitive");
    const wchar_t* copy[]{L"radeonctl", L"firmware-copy", L"E:\\amdgpu", L"--replace"};
    expect(radeon::parse_command_line(4, copy, command) && command.command == radeon::Command::firmware_copy &&
               command.replace_firmware && radeon::text_equal_insensitive(command.source_directory, L"E:\\amdgpu"),
           "firmware-copy takes a directory and --replace");
    const wchar_t* copy_missing[]{L"radeonctl", L"firmware-copy"};
    expect(!radeon::parse_command_line(2, copy_missing, command), "firmware-copy requires a directory");
    const wchar_t* none[]{L"radeonctl"};
    expect(radeon::parse_command_line(1, none, command) && command.command == radeon::Command::help,
           "no arguments prints help");
}

size_t list_characters(const wchar_t* list) {
    size_t position = 0;
    while (list[position] != 0) {
        while (list[position] != 0) {
            ++position;
        }
        ++position;
    }
    return position + 1;
}

void identity_policy() {
    for (const uint16_t device_id : rx7900::navi31_device_ids) {
        wchar_t list[256]{};
        std::swprintf(list, 200, L"PCI\\VEN_1002&DEV_%04X&SUBSYS_471E1DA2&REV_C8", device_id);
        const size_t first = std::wcslen(list) + 1;
        std::swprintf(list + first, 50, L"PCI\\VEN_1002&DEV_%04X&CC_030000", device_id);
        const size_t characters = first + std::wcslen(list + first) + 2;
        uint16_t parsed = 0;
        uint8_t revision = 0;
        wchar_t specific[radeon::instance_characters]{};
        expect(radeon::navi31_identity(list, characters, parsed, revision, specific) && parsed == device_id &&
                   revision == 0xc8 && std::wcscmp(specific, list) == 0,
               "every pci.hpp Navi31 GPU ID is recognized with revision and most specific ID");
    }

    struct Rejected final {
        const wchar_t* list;
        const char* reason;
    };
    const wchar_t usb[] = L"PCI\\VEN_1002&DEV_7446&SUBSYS_471E1DA2\0\0";
    const wchar_t audio[] = L"PCI\\VEN_1002&DEV_AB30\0\0";
    const wchar_t intel[] = L"PCI\\VEN_8086&DEV_744C\0\0";
    const wchar_t longer[] = L"PCI\\VEN_1002&DEV_744C0\0\0";
    const wchar_t lightning[] = L"PCI\\VEN_1234&DEV_11F1\0\0";
    const Rejected rejected[]{{usb, "the card's USB function is not a GPU"},
                              {audio, "the HDMI/DP audio function is not a GPU"},
                              {intel, "another vendor never matches"},
                              {longer, "a longer device token is not a Navi31 ID"},
                              {lightning, "the Lightning virtual device never matches"}};
    for (const auto& item : rejected) {
        uint16_t parsed = 0;
        uint8_t revision = 0;
        wchar_t specific[radeon::instance_characters]{};
        expect(!radeon::navi31_identity(item.list, list_characters(item.list), parsed, revision, specific),
               item.reason);
    }

    const wchar_t valid[] = L"PCI\\VEN_1002&DEV_744C\0\0";
    uint16_t parsed = 0;
    uint8_t revision = 0;
    wchar_t specific[radeon::instance_characters]{};
    for (size_t count = 0; count < list_characters(valid) - 1; ++count) {
        expect(!radeon::navi31_identity(valid, count, parsed, revision, specific),
               "every truncated MULTI_SZ is malformed");
    }
    const wchar_t hidden[] = L"PCI\\VEN_8086&DEV_1234\0\0PCI\\VEN_1002&DEV_744C\0\0";
    expect(!radeon::navi31_identity(hidden, sizeof(hidden) / sizeof(wchar_t) - 1, parsed, revision, specific),
           "IDs hidden after the list terminator are rejected");
    const wchar_t conflicting[] = L"PCI\\VEN_1002&DEV_744C\0PCI\\VEN_1002&DEV_7448\0\0";
    expect(!radeon::navi31_identity(conflicting, list_characters(conflicting), parsed, revision, specific),
           "conflicting Navi31 identities in one list are rejected");
    expect(radeon::multi_string_contains(lightning, list_characters(lightning), radeon::lightning_hardware_id),
           "the Lightning device is recognized for reporting");
}

void binding_policy() {
    radeon::DeviceRecord device{};
    expect(radeon::decide_binding(device, nullptr) == radeon::Binding::bind_unbound, "unbound devices may bind");
    radeon::text_copy(device.service, L"RX7900VIDEO");
    expect(radeon::decide_binding(device, nullptr) == radeon::Binding::already_ours, "our binding is recognized");
    radeon::text_copy(device.service, L"vbemp");
    device.started = true;
    expect(radeon::decide_binding(device, nullptr) == radeon::Binding::refuse_active,
           "an active foreign driver is refused");
    device.started = false;
    expect(radeon::decide_binding(device, nullptr) == radeon::Binding::refuse_inactive,
           "a bound but stopped foreign driver is refused");
    device.started = true;
    expect(radeon::decide_binding(device, L"VBEMP") == radeon::Binding::replace_named,
           "--force-replace with the exact service replaces it");
    expect(radeon::decide_binding(device, L"vga") == radeon::Binding::refuse_mismatch,
           "--force-replace naming another service is refused");
    expect(radeon::decide_binding(device, L"") == radeon::Binding::refuse_active, "an empty name forces nothing");

    radeon::DeviceInventory inventory{};
    size_t index = 99;
    expect(radeon::select_navi31(inventory, index) == radeon::Selection::none_present, "no Navi31 present");
    inventory.count = 1;
    expect(radeon::select_navi31(inventory, index) == radeon::Selection::unique && index == 0, "one Navi31");
    inventory.count = 2;
    expect(radeon::select_navi31(inventory, index) == radeon::Selection::multiple, "two Navi31 are refused");
    inventory.count = 1;
    inventory.omitted = 1;
    expect(radeon::select_navi31(inventory, index) == radeon::Selection::multiple,
           "omitted functions prevent proving uniqueness");
    inventory.count = 0;
    expect(radeon::select_navi31(inventory, index) == radeon::Selection::multiple,
           "only omitted functions are still multiple");

    expect(radeon::effective_parameters_key(false, true) == radeon::device0_key &&
               radeon::effective_parameters_key(true, false) == radeon::device0_key &&
               radeon::effective_parameters_key(true, true) == radeon::profile_device0_key,
           "VideoPort parameter key follows UseNewKey and the hardware-profile copy");

    const wchar_t* system = L"C:\\ReactOS\\system32";
    expect(radeon::service_image_matches(L"\\SystemRoot\\System32\\drivers\\rx7900video.sys", system) &&
               radeon::service_image_matches(L"system32\\DRIVERS\\rx7900video.sys", system) &&
               radeon::service_image_matches(L"C:\\ReactOS\\system32\\drivers\\rx7900video.sys", system) &&
               radeon::service_image_matches(L"\\??\\C:\\ReactOS\\system32\\drivers\\rx7900video.sys", system),
           "SCM and SetupAPI image path forms are ours");
    expect(!radeon::service_image_matches(L"\\SystemRoot\\System32\\drivers\\vbemp.sys", system) &&
               !radeon::service_image_matches(L"D:\\evil\\rx7900video.sys", system) &&
               !radeon::service_image_matches(L"", system),
           "other images are never treated as ours");
}

void pe_policy() {
    const Bytes driver = make_pe(radeon::machine_i386, radeon::ImageKind::kernel_driver, 1);
    uint32_t offset = 0;
    expect(radeon::pe_header_offset(rx7900::ByteView(driver.data(), 64), driver.size(), offset) && offset == 0x80,
           "DOS header locates the PE headers");
    const auto check = [](const Bytes& bytes, uint16_t machine, radeon::ImageKind kind) {
        return radeon::check_pe_headers(rx7900::ByteView(bytes.data() + 0x80, radeon::pe_headers_bytes), machine, kind);
    };
    expect(check(driver, radeon::machine_i386, radeon::ImageKind::kernel_driver) == radeon::PeStatus::valid,
           "native i386 kernel driver accepted");
    expect(check(driver, radeon::machine_amd64, radeon::ImageKind::kernel_driver) == radeon::PeStatus::wrong_machine,
           "an i386 image is refused by an AMD64 installer");
    expect(check(driver, radeon::machine_i386, radeon::ImageKind::kernel_dll) == radeon::PeStatus::not_dll,
           "a driver image is not the display DLL");
    const Bytes icd = make_pe(radeon::machine_i386, radeon::ImageKind::user_dll, 1);
    expect(check(icd, radeon::machine_i386, radeon::ImageKind::kernel_dll) == radeon::PeStatus::wrong_subsystem,
           "a user-mode DLL is never installed as the kernel display DLL");
    expect(check(icd, radeon::machine_i386, radeon::ImageKind::user_dll) == radeon::PeStatus::valid,
           "the ICD is a user-mode DLL");
    const Bytes amd64 = make_pe(radeon::machine_amd64, radeon::ImageKind::kernel_dll, 1);
    expect(check(amd64, radeon::machine_amd64, radeon::ImageKind::kernel_dll) == radeon::PeStatus::valid,
           "AMD64 PE32+ display DLL accepted");
    Bytes broken = driver;
    put32(broken, 0x3c, 0x10);
    expect(!radeon::pe_header_offset(rx7900::ByteView(broken.data(), 64), broken.size(), offset),
           "a header offset inside the DOS header is rejected");
    put32(broken, 0x3c, 0x7f0);
    expect(!radeon::pe_header_offset(rx7900::ByteView(broken.data(), 64), broken.size(), offset),
           "PE headers past the end of the file are rejected");
    broken = driver;
    broken[0x80] = 'X';
    expect(check(broken, radeon::machine_i386, radeon::ImageKind::kernel_driver) == radeon::PeStatus::not_pe,
           "a missing PE signature is rejected");
}

void filter_policy() {
    wchar_t merged[radeon::filter_characters]{};
    size_t characters = 0;
    size_t restored = 0;
    const wchar_t original[] = L"OtherUpper\0\u00c9tendu\0\0";
    const wchar_t current[] = L"Fresh\0\0";
    expect(radeon::merge_missing_filters(original, list_characters(original), current, list_characters(current),
                                         merged, characters, restored) &&
               restored == 2 && characters == list_characters(L"Fresh\0OtherUpper\0\u00c9tendu\0\0") &&
               std::memcmp(merged, L"Fresh\0OtherUpper\0\u00c9tendu\0\0", characters * sizeof(wchar_t)) == 0,
           "missing original filters are appended after the current list, Unicode unchanged");
    const wchar_t present[] = L"otherupper\0\u00c9tendu\0\0";
    expect(radeon::merge_missing_filters(original, list_characters(original), present, list_characters(present),
                                         merged, characters, restored) && restored == 0,
           "case-insensitively present filters are not duplicated");
    expect(radeon::merge_missing_filters(original, list_characters(original), nullptr, 0, merged, characters,
                                         restored) &&
               restored == 2 && std::memcmp(merged, original, characters * sizeof(wchar_t)) == 0,
           "an absent current list receives the original list");
    expect(radeon::merge_missing_filters(nullptr, 0, nullptr, 0, merged, characters, restored) && characters == 0 &&
               restored == 0, "no original filters restore nothing");
    const wchar_t duplicated[] = L"A\0A\0\0";
    expect(radeon::merge_missing_filters(duplicated, list_characters(duplicated), nullptr, 0, merged, characters,
                                         restored) && restored == 1, "a repeated original entry is restored once");
    const wchar_t malformed[] = {L'A', 0, 0, L'B', 0, 0};
    expect(!radeon::merge_missing_filters(malformed, 6, nullptr, 0, merged, characters, restored) && merged[0] == 0,
           "hidden entries make a list malformed");
    for (size_t count = 1; count < list_characters(original); ++count) {
        expect(!radeon::merge_missing_filters(original, count, nullptr, 0, merged, characters, restored),
               "every truncated original list is malformed");
    }
    std::vector<wchar_t> big;
    for (int entry = 0; entry < 200; ++entry) {
        for (wchar_t value : std::wstring(L"Filter")) {
            big.push_back(value);
        }
        big.push_back(static_cast<wchar_t>(L'A' + entry / 26));
        big.push_back(static_cast<wchar_t>(L'A' + entry % 26));
        big.push_back(0);
    }
    big.push_back(0);
    expect(!radeon::merge_missing_filters(big.data(), big.size(), current, list_characters(current), merged,
                                          characters, restored) && characters == 0,
           "a merge that would not fit fails without truncating");
}

void firmware_policy() {
    radeon::FirmwareEntry entries[radeon::firmware_entry_capacity]{};
    const size_t count = radeon::firmware_manifest(0xc8, entries);
    const char* expected[]{"gc_11_0_0_pfp.bin", "gc_11_0_0_me.bin", "gc_11_0_0_mec.bin", "gc_11_0_0_rlc.bin",
                           "gc_11_0_0_mes_2.bin", "gc_11_0_0_mes1.bin", "gc_11_0_0_imu.bin", "sdma_6_0_0.bin",
                           "smu_13_0_0.bin", "psp_13_0_0_sos.bin", "dcn_3_2_0_dmcub.bin", "psp_13_0_0_ta.bin",
                           "rx7900.rom"};
    bool names = count == sizeof(expected) / sizeof(expected[0]);
    for (size_t index = 0; names && index < count; ++index) {
        names = std::strcmp(entries[index].name, expected[index]) == 0;
    }
    expect(names, "firmware manifest names follow the miniport load order");
    expect(entries[4].alternate[0] != 0 && std::strcmp(entries[4].alternate, "gc_11_0_0_mes.bin") == 0,
           "MES accepts the Linux fallback image name");
    size_t required = 0;
    for (size_t index = 0; index < count; ++index) {
        required += entries[index].required ? 1 : 0;
    }
    expect(required == 10 && !entries[10].required && !entries[11].required && !entries[12].required,
           "nine ucode files plus the PSP SOS are required; DMCUB, TA and ROM are optional");
    radeon::firmware_manifest(0xce, entries);
    expect(std::strcmp(entries[3].name, "gc_11_0_0_rlc_1.bin") == 0, "revision 0xCE selects gc_11_0_0_rlc_1.bin");

    namespace files = rx7900_test::ucode_files;
    radeon::firmware_manifest(0xc8, entries);
    const auto verdict = [&](size_t index, const Bytes& bytes, uint16_t device = 0x744c) {
        return radeon::validate_firmware_bytes(entries[index], rx7900::ByteView(bytes.data(), bytes.size()), device);
    };
    expect(verdict(0, files::gfx(0x40, 0x20).bytes) == radeon::FirmwareVerdict::valid &&
               verdict(3, files::rlc(3).bytes) == radeon::FirmwareVerdict::valid &&
               verdict(9, psp_sos_fixture()) == radeon::FirmwareVerdict::valid &&
               verdict(11, files::ta_navi31().bytes) == radeon::FirmwareVerdict::valid &&
               verdict(12, atom_rom_fixture(0x744c)) == radeon::FirmwareVerdict::valid,
           "synthetic containers pass the miniport's validators");
    expect(verdict(0, files::sdma().bytes) == radeon::FirmwareVerdict::malformed,
           "an SDMA container is not accepted as GC PFP");
    expect(verdict(12, atom_rom_fixture(0x744c), 0x7448) == radeon::FirmwareVerdict::wrong_device,
           "a ROM for another Navi31 device is refused for the present device");
    expect(verdict(12, atom_rom_fixture(0x7448), 0) == radeon::FirmwareVerdict::valid,
           "without a device any Navi31 ROM identity is accepted");
    expect(verdict(0, Bytes(31, 0)) == radeon::FirmwareVerdict::too_small, "undersized containers are refused");
    expect(radeon::firmware_maximum_bytes(entries[0]) == 32u * 1024u * 1024u &&
               radeon::firmware_maximum_bytes(entries[12]) == 1024u * 1024u,
           "firmware and ROM size limits equal the native loader's");
}

void scanner_policy() {
    std::string image(5000, '\x90');
    const char* markers[]{"Rx7900StretchGuardPrepare", "Rx7900LinePrepare", "IntEngLineTo", "LightningDrawEscape"};
    size_t position = 100;
    for (const char* marker : markers) {
        image.replace(position, std::strlen(marker), marker);
        position += 1000;
    }
    image.replace(4600, 12, "Rx7900Radeon");  // fragment only

    // Every chunk size, so each marker straddles boundaries at every offset.
    bool all = true;
    for (size_t chunk = 1; chunk <= 64 && all; ++chunk) {
        radeon::Win32kScanner scanner;
        for (size_t offset = 0; offset < image.size(); offset += chunk) {
            const size_t count = image.size() - offset < chunk ? image.size() - offset : chunk;
            scanner.feed(rx7900::ByteView(reinterpret_cast<const uint8_t*>(image.data()) + offset, count));
        }
        all = scanner.symbols_present() && scanner.presence(0) == radeon::PatchPresence::present &&
              scanner.presence(1) == radeon::PatchPresence::absent &&
              scanner.presence(2) == radeon::PatchPresence::absent &&
              scanner.presence(3) == radeon::PatchPresence::present;
    }
    expect(all, "win32k markers are found across every chunk boundary");

    std::string partial(2000, 0);
    partial.replace(10, 12, "IntEngLineTo");
    partial.replace(100, 25, "Rx7900RadeonStrokePrepare");
    radeon::Win32kScanner scanner;
    scanner.feed(rx7900::ByteView(reinterpret_cast<const uint8_t*>(partial.data()), partial.size()));
    expect(scanner.presence(1) == radeon::PatchPresence::partial, "one of two bridge symbols is partial");

    std::string stripped(2000, 0);
    stripped.replace(10, 19, "LightningDrawEscape");
    radeon::Win32kScanner without;
    without.feed(rx7900::ByteView(reinterpret_cast<const uint8_t*>(stripped.data()), stripped.size()));
    expect(!without.symbols_present() && without.presence(3) == radeon::PatchPresence::present &&
               without.presence(0) == radeon::PatchPresence::unknown,
           "without a symbol table absence is unknown, presence still proven");

    std::string overlap(200, 0);
    overlap.replace(5, 24, "IntEngLineTIntEngLineTo");
    radeon::Win32kScanner kmp;
    kmp.feed(rx7900::ByteView(reinterpret_cast<const uint8_t*>(overlap.data()), overlap.size()));
    expect(kmp.symbols_present(), "a repeated partial prefix does not hide a marker");

    // "Rx7900R" is a partial match whose border "R" starts the real marker.
    std::string border(200, 0);
    border.replace(5, 12, "IntEngLineTo");
    border.replace(40, 31, "Rx7900Rx7900RadeonStrokePrepare");
    border.replace(100, 22, "Rx7900RadeonStrokeDraw");
    radeon::Win32kScanner bordered;
    bordered.feed(rx7900::ByteView(reinterpret_cast<const uint8_t*>(border.data()), border.size()));
    expect(bordered.presence(1) == radeon::PatchPresence::present,
           "a marker beginning inside a failed partial match is found (KMP borders)");
}

void replace_policy() {
    rx7900::tools::Sha256Digest target{};
    rx7900::tools::Sha256Digest prior{};
    rx7900::tools::Sha256Digest other{};
    target.bytes[0] = 1;
    prior.bytes[0] = 2;
    other.bytes[0] = 3;
    radeon::ReplaceObservation seen{};
    seen.live = true;
    seen.live_hash = target;
    expect(radeon::decide_replace_recovery(seen, target, prior) == radeon::ReplaceRecovery::complete,
           "a live target is complete");
    seen.live_hash = prior;
    seen.staged = true;
    seen.staged_hash = target;
    expect(radeon::decide_replace_recovery(seen, target, prior) == radeon::ReplaceRecovery::finish_forward,
           "a verified staged target goes forward");
    seen.staged_hash = other;
    expect(radeon::decide_replace_recovery(seen, target, prior) == radeon::ReplaceRecovery::restore_previous &&
               radeon::prior_source(seen, prior) == 1, "a torn staged copy restores the live prior");
    seen.live = false;
    seen.retired = true;
    seen.retired_hash = prior;
    expect(radeon::prior_source(seen, prior) == 2, "a retired prior is used when live is gone");
    seen.retired = false;
    seen.backup_staged = true;
    seen.backup_staged_hash = prior;
    expect(radeon::prior_source(seen, prior) == 3, "the staged backup holds the prior");
    seen.backup_staged = false;
    seen.backup = true;
    seen.backup_hash = prior;
    expect(radeon::prior_source(seen, prior) == 4, "the committed backup holds the prior");
    seen.backup_hash = other;
    expect(radeon::decide_replace_recovery(seen, target, prior) == radeon::ReplaceRecovery::unrecoverable,
           "no surviving bytes is reported, never guessed");
}

// ---- INF and source contract ----

std::string read_text(const std::string& path) {
    std::string text;
    if (FILE* file = std::fopen(path.c_str(), "rb")) {
        char buffer[4096];
        size_t count = 0;
        while ((count = std::fread(buffer, 1, sizeof(buffer), file)) != 0) {
            text.append(buffer, count);
        }
        std::fclose(file);
    }
    return text;
}

std::string section(const std::string& inf, const std::string& name) {
    const std::string header = "[" + name + "]";
    const size_t start = inf.find(header);
    if (start == std::string::npos) {
        return std::string();
    }
    const size_t body = start + header.size();
    const size_t end = inf.find("\n[", body);
    return inf.substr(body, end == std::string::npos ? std::string::npos : end - body);
}

std::string narrow(const wchar_t* text) {
    std::string value;
    for (size_t index = 0; text[index] != 0; ++index) {
        value.push_back(static_cast<char>(text[index]));
    }
    return value;
}

void inf_contract() {
    const std::string root = RX7900_SOURCE_DIR;
    const std::string inf = read_text(root + "/radeondisplay.inf");
    expect(!inf.empty(), "radeondisplay.inf is readable");

    for (const char* models : {"Models.NTx86", "Models.NTamd64"}) {
        const std::string body = section(inf, models);
        bool all = !body.empty();
        size_t lines = 0;
        for (const uint16_t device : rx7900::navi31_device_ids) {
            char id[32];
            std::snprintf(id, sizeof(id), "PCI\\VEN_1002&DEV_%04X\r", device);
            char id_lf[32];
            std::snprintf(id_lf, sizeof(id_lf), "PCI\\VEN_1002&DEV_%04X\n", device);
            all = all && (body.find(id) != std::string::npos || body.find(id_lf) != std::string::npos);
        }
        for (size_t at = body.find("PCI\\"); at != std::string::npos; at = body.find("PCI\\", at + 1)) {
            ++lines;
        }
        expect(all && lines == sizeof(rx7900::navi31_device_ids) / sizeof(rx7900::navi31_device_ids[0]),
               "each model section binds exactly the pci.hpp Navi31 GPU IDs");
    }
    expect(inf.find("%Provider%=Models,NTx86,NTamd64") != std::string::npos, "NTx86 and NTamd64 decorations");

    const std::string service = section(inf, "DisplayService");
    expect(service.find("ServiceType=1") != std::string::npos && service.find("StartType=3") != std::string::npos &&
               service.find("ErrorControl=0") != std::string::npos &&
               service.find("ServiceBinary=%12%\\rx7900video.sys") != std::string::npos,
           "kernel service, demand start, ignore errors, rx7900video.sys image");
    expect(inf.find("StartType=0") == std::string::npos && inf.find("StartType=1") == std::string::npos &&
               inf.find("StartType=2") == std::string::npos, "no boot, system or auto start anywhere");
    for (const char* decoration : {"Install.NT.Services", "Install.NTx86.Services", "Install.NTamd64.Services"}) {
        expect(section(inf, decoration).find("AddService=" + narrow(radeon::service_name) + ",0x00000002,DisplayService") !=
                   std::string::npos, "every decoration associates the rx7900video function service");
    }

    const std::string settings = section(inf, "ServiceSettings");
    for (const auto& rule : radeon::feature_rules) {
        if (rule.registry_value != nullptr) {
            expect(settings.find("HKR,Device0," + narrow(rule.registry_value) + ",0x00010001,0") != std::string::npos,
                   "every miniport opt-in defaults to REG_DWORD 0 in Device0");
        }
    }
    expect(settings.find("HKR,Device0,InstalledDisplayDrivers,0x00010000,\"radeondisp\"") != std::string::npos &&
               section(inf, "DisplaySettings").find("InstalledDisplayDrivers,0x00010000,\"radeondisp\"") !=
                   std::string::npos, "radeondisp is the installed display driver");
    expect(inf.find("UpperFilters") == std::string::npos && inf.find("LowerFilters") == std::string::npos,
           "the INF adds or replaces no filters");
    expect(inf.find("OpenGLDrivers") == std::string::npos && inf.find("radeonglicd") == std::string::npos,
           "the ICD stays an explicit installer opt-in");
    expect(inf.find("DriverVer=10/24/2026," + narrow(radeon::package_version)) != std::string::npos,
           "INF DriverVer equals the installer package version");
    for (size_t index = 0; index < radeon::component_count; ++index) {
        const std::string name = narrow(radeon::component_files[index].filename);
        const bool listed = section(inf, "SourceDisksFiles").find(name + "=1") != std::string::npos;
        expect(listed == radeon::component_files[index].inf_managed,
               "the INF copies exactly the inf_managed components");
    }

    // The registry names must equal the names the miniport actually reads.
    const std::string video = read_text(root + "/src/reactos/video.cpp");
    const std::string cursor = read_text(root + "/src/reactos/radeon_cursor_native.cpp");
    for (const auto& rule : radeon::feature_rules) {
        if (rule.registry_value != nullptr) {
            const std::string literal = "wchar_t name[] = L\"" + narrow(rule.registry_value) + "\"";
            expect(video.find(literal) != std::string::npos || cursor.find(literal) != std::string::npos,
                   "each opt-in name is read by rx7900video");
        }
    }
    const std::string blob = read_text(root + "/src/reactos/driver_blob.cpp");
    expect(blob.find("L\"\\\\SystemRoot\\\\System32\\\\drivers\\\\rx7900\\\\\"") != std::string::npos,
           "firmware directory equals the native loader's prefix");
    expect(std::strcmp(rx7900::reactos::radeon_opengl::driver_name, "RadeonGL") == 0 &&
               std::wstring(radeon::opengl_drivers_key).find(L"OpenGLDrivers\\RadeonGL") != std::wstring::npos,
           "the ICD key is named after the radeondisp OPENGL_GETINFO reply");
    // The generated FIRMWARE.txt names every manifest file the miniport loads.
    const std::string package = read_text(root + "/cmake/driver-package.cmake");
    radeon::FirmwareEntry entries[radeon::firmware_entry_capacity]{};
    const size_t count = radeon::firmware_manifest(0xce, entries);
    for (size_t index = 0; index < count; ++index) {
        expect(package.find(entries[index].name) != std::string::npos &&
                   (entries[index].alternate[0] == 0 || package.find(entries[index].alternate) != std::string::npos),
               "FIRMWARE.txt lists every manifest name and fallback");
    }
    const std::string spec = read_text(root + "/src/opengl/icd/radeonglicd.spec");
    expect(spec.find("DrvValidateVersion") != std::string::npos && spec.find("DrvSetContext") != std::string::npos,
           "radeonglicd exports the entry points opengl32 loads");
}

// ---- Engine scenarios ----

void prepare(FakeSystem& fake, const std::wstring& service = L"", bool started = false) {
    put_package(fake, 1);
    fake.add_device(0x744c, service, started);
}

void lightning_vm() {
    FakeSystem fake;
    put_package(fake, 1);
    fake.lightning_present = true;
    const Snapshot before = fake.snapshot();
    expect(run(fake, {L"install", L"--asic"}) == radeon::exit_code::no_navi31 && fake.mutations == 0 &&
               fake.snapshot() == before, "no Navi31: install changes nothing and exits 4");
    expect(fake.output_contains("No Navi31 GPU") && fake.output_contains("Lightning"),
           "no Navi31 is reported cleanly and the Lightning device is named");
    fake.output.clear();
    expect(run(fake, {L"status"}) == radeon::exit_code::success && fake.mutations == 0 &&
               fake.output_contains("No Navi31 GPU"), "status is read-only and reports no Navi31");
    expect(run(fake, {L"remove"}) == radeon::exit_code::success && clean(fake, L""),
           "remove with nothing installed is a clean no-op");
    expect(run(fake, {L"upgrade"}) == radeon::exit_code::refused && run(fake, {L"rollback"}) == radeon::exit_code::refused,
           "upgrade and rollback require an installation");
}

void fresh_install() {
    FakeSystem fake;
    prepare(fake);
    expect(run(fake, {L"install"}) == radeon::exit_code::restart_required && installed(fake, 0),
           "fresh install binds, copies, writes safe opt-ins and asks for a restart");
    expect(fake.service.start == 3 && fake.service.error_control == 0, "service is demand start, errors ignored");
    expect(!fake.devices.front().record.started && fake.service.running == false,
           "nothing is started by the installer");
    const Snapshot after = fake.snapshot();
    const size_t mutations = fake.mutations;
    expect(run(fake, {L"install"}) == radeon::exit_code::success && installed(fake, 0) && fake.snapshot() == after,
           "repeating install is idempotent and needs no restart");
    (void)mutations;
    fake.output.clear();
    expect(run(fake, {L"status"}) == radeon::exit_code::success && fake.output_contains("equals this package") &&
               fake.output_contains("PhysicalAsicInit") && fake.output_contains("Installer journal: installed"),
           "status reads back files, hashes, opt-ins and journal");
    expect(run(fake, {L"install", L"--dry-run", L"--opengl-icd"}) == radeon::exit_code::success && fake.snapshot() == after,
           "dry run changes nothing");
}

void refusals() {
    {
        FakeSystem fake;
        prepare(fake);
        expect(run(fake, {L"install", L"--2d"}) == radeon::exit_code::refused && fake.mutations == 0,
               "dependency violations change nothing");
        expect(run(fake, {L"install", L"--asic"}) == radeon::exit_code::refused && fake.mutations == 0 &&
                   fake.output_contains("requires every required firmware"),
               "--asic without firmware is refused before any change");
        expect(run(fake, {L"install", L"--rom-override"}) == radeon::exit_code::refused && fake.mutations == 0,
               "--rom-override without a valid ROM is refused");
    }
    {
        FakeSystem fake;
        prepare(fake, L"vbemp", true);
        const Snapshot before = fake.snapshot();
        expect(run(fake, {L"install"}) == radeon::exit_code::refused && fake.snapshot() == before &&
                   fake.output_contains("actively driving"), "an active foreign driver is never replaced implicitly");
        expect(run(fake, {L"install", L"--force-replace", L"vga"}) == radeon::exit_code::refused &&
                   fake.snapshot() == before, "naming a different service is refused");
        expect(run(fake, {L"install", L"--force-replace", L"vbemp"}) == radeon::exit_code::restart_required &&
                   installed(fake, 0), "an exactly named replacement binds");
        uint32_t ignored = 0;
        expect(fake.string(radeon::journal_key, L"PreviousService") == L"vbemp" &&
                   !fake.dword(radeon::journal_key, L"Missing", ignored),
               "the replaced service is recorded for status and removal");
        expect(run(fake, {L"remove"}) == radeon::exit_code::restart_required && clean(fake, L""),
               "removing a replacement leaves the device for re-detection");
        fake.redetect();
        expect(radeon::text_equal_insensitive(fake.devices.front().record.service, L"vbemp"),
               "ReactOS re-detection restores its default driver, not the installer");
    }
    {
        FakeSystem fake;
        prepare(fake);
        fake.add_device(0x744c, L"", false, L"4&1A2B3C4D&0&0020");
        expect(run(fake, {L"install"}) == radeon::exit_code::refused && fake.mutations == 0,
               "two Navi31 GPUs are refused");
    }
    {
        FakeSystem fake;
        prepare(fake);
        fake.files.erase(rx7900_test::radeon_fake::fold(fake.packaged(L"radeondisp.dll").c_str()));
        expect(run(fake, {L"install"}) == radeon::exit_code::failure && fake.mutations == 0,
               "an incomplete package is refused before any change");
        put_package(fake, 1);
        fake.put_file(fake.packaged(L"rx7900video.sys"), make_pe(radeon::machine_amd64, radeon::ImageKind::kernel_driver, 1));
        expect(run(fake, {L"install"}) == radeon::exit_code::failure && fake.mutations == 0,
               "a wrong-architecture package is refused before any change");
    }
}

void features_install() {
    FakeSystem fake;
    prepare(fake);
    put_firmware(fake);
    const uint32_t all = radeon::known_features;
    expect(run(fake, {L"install", L"--asic", L"--rom-override", L"--vbe", L"--2d", L"--2d-compute", L"--cursor",
                      L"--opengl-icd"}) == radeon::exit_code::restart_required && installed(fake, all),
           "every opt-in is written as 1 and the ICD registered when requested");
    expect(run(fake, {L"install", L"--asic"}) == radeon::exit_code::restart_required &&
               installed(fake, radeon::feature_physical_asic_init),
           "reinstall with fewer flags turns the rest off and unregisters the ICD");
    expect(run(fake, {L"install"}) == radeon::exit_code::restart_required && installed(fake, 0),
           "reinstall without flags returns every opt-in to 0");

    // A foreign RadeonGL registration is never removed.
    fake.set_string(radeon::opengl_drivers_key, L"Dll", L"vendorgl.dll");
    expect(run(fake, {L"install"}) == radeon::exit_code::success &&
               fake.string(radeon::opengl_drivers_key, L"Dll") == L"vendorgl.dll",
           "another product's RadeonGL key is preserved");
}

void profile_key() {
    FakeSystem fake;
    prepare(fake);
    put_firmware(fake);
    fake.keys.insert(rx7900_test::radeon_fake::fold(radeon::use_new_key));
    fake.keys.insert(rx7900_test::radeon_fake::fold(radeon::profile_device0_key));
    expect(run(fake, {L"install", L"--asic"}) == radeon::exit_code::restart_required, "install with a profile key");
    uint32_t value = 0;
    expect(fake.dword(radeon::profile_device0_key, L"PhysicalAsicInit", value) && value == 1 &&
               fake.dword(radeon::device0_key, L"PhysicalAsicInit", value) && value == 1,
           "opt-ins reach the key VideoPort actually reads");
}

void filter_preservation() {
    FakeSystem fake;
    prepare(fake);
    auto& record = fake.devices.front().record;
    FakeSystem::set_filters(record, true, {L"VendorUpper", L"Second"});
    FakeSystem::set_filters(record, false, {L"VendorLower"});
    fake.install_clears_filters = true;
    expect(run(fake, {L"install"}) == radeon::exit_code::restart_required && installed(fake, 0),
           "install succeeds when SetupAPI resets filters");
    expect(FakeSystem::filters(fake.devices.front().record, true) == std::vector<std::wstring>({L"VendorUpper", L"Second"}) &&
               FakeSystem::filters(fake.devices.front().record, false) == std::vector<std::wstring>({L"VendorLower"}),
           "other drivers' device filters are restored in order after binding");

    expect(run(fake, {L"remove"}) == radeon::exit_code::restart_required &&
               journal_state(fake, radeon::JournalState::filters_pending), "remove saves the filters it cannot keep");
    expect(run(fake, {L"recover"}) == radeon::exit_code::refused, "recover waits until the GPU is re-detected");
    fake.redetect();
    expect(run(fake, {L"recover"}) == radeon::exit_code::restart_required &&
               FakeSystem::filters(fake.devices.front().record, true) ==
                   std::vector<std::wstring>({L"VendorUpper", L"Second"}) &&
               FakeSystem::filters(fake.devices.front().record, false) == std::vector<std::wstring>({L"VendorLower"}) &&
               journal_state(fake, radeon::JournalState::none) &&
               radeon::text_equal_insensitive(fake.devices.front().record.service, L"vbemp"),
           "after re-detection recover restores the filters on the default driver's devnode");
    expect(run(fake, {L"recover"}) == radeon::exit_code::success, "recover is idempotent afterwards");
}

void upgrade_and_rollback() {
    FakeSystem fake;
    prepare(fake);
    expect(run(fake, {L"install", L"--opengl-icd"}) == radeon::exit_code::restart_required &&
               installed(fake, radeon::feature_opengl_icd), "v1 installed with the ICD");
    std::vector<Bytes> v1;
    for (size_t index = 0; index < radeon::component_count; ++index) {
        v1.push_back(contents(fake.file(fake.live(index))));
    }
    expect(run(fake, {L"upgrade"}) == radeon::exit_code::success, "upgrading to the same package is a no-op");
    put_package(fake, 50);
    expect(run(fake, {L"upgrade"}) == radeon::exit_code::restart_required &&
               installed(fake, radeon::feature_opengl_icd), "upgrade replaces every installed file");
    bool backups = true;
    for (size_t index = 0; index < radeon::component_count; ++index) {
        const Bytes* backup = fake.file(fake.drivers() + L"\\rx7900\\backup\\" + radeon::component_files[index].filename);
        backups = backups && backup != nullptr && *backup == v1[index];
    }
    expect(backups, "the previous version of every file is kept as the backup");
    expect(run(fake, {L"rollback"}) == radeon::exit_code::restart_required &&
               same_bytes(fake.file(fake.live(0)), v1[0]) && same_bytes(fake.file(fake.live(1)), v1[1]) &&
               same_bytes(fake.file(fake.live(2)), v1[2]) && journal_state(fake, radeon::JournalState::installed) &&
               no_transaction_leftovers(fake), "rollback restores the previous version");
    expect(same_bytes(fake.file(fake.drivers() + L"\\rx7900\\backup\\rx7900video.sys"),
                      contents(fake.file(fake.packaged(L"rx7900video.sys")))), "rollback keeps the replaced version as the backup");
    fake.put_file(fake.drivers() + L"\\rx7900\\backup\\radeondisp.dll", Bytes(10, 1));
    expect(run(fake, {L"rollback"}) == radeon::exit_code::refused, "an altered backup is never restored");

    // A loaded display DLL is replaced at the next restart instead.
    FakeSystem locked;
    prepare(locked);
    expect(run(locked, {L"install"}) == radeon::exit_code::restart_required, "install for the locked case");
    locked.locked.insert(rx7900_test::radeon_fake::fold(locked.live(1).c_str()));
    put_package(locked, 70);
    expect(run(locked, {L"upgrade"}) == radeon::exit_code::restart_required && !locked.pending_renames.empty() &&
               live_equals_package(locked, 0) && !live_equals_package(locked, 1),
           "an in-use file is scheduled for replacement at restart");
    uint32_t pending = 0;
    expect(locked.dword(radeon::journal_key, L"RestartPending", pending) && pending == 1, "the restart is recorded");
    locked.restart();
    expect(run(locked, {L"recover"}) == radeon::exit_code::success && installed(locked, 0) &&
               locked.dword(radeon::journal_key, L"RestartPending", pending) && pending == 0,
           "after restart recover confirms the replacement and clears the flag");
}

// The amd64 package (radeonctl x86_64, PE32+ images; the ICD a GUI-subsystem
// PE32+ DLL) through install, status, upgrade, rollback and remove with the ICD.
void amd64_icd_lifecycle() {
    FakeSystem fake;
    fake.machine = radeon::machine_amd64;
    put_package(fake, 1, radeon::machine_amd64);
    fake.add_device(0x744c, L"", false);
    expect(run(fake, {L"install", L"--opengl-icd"}) == radeon::exit_code::restart_required &&
               installed(fake, radeon::feature_opengl_icd), "amd64: install copies the PE32+ ICD and registers RadeonGL");
    const Bytes v1 = contents(fake.file(fake.live(radeon::icd_component)));
    fake.output.clear();
    expect(run(fake, {L"status"}) == radeon::exit_code::success && fake.output_contains("amd64") &&
               fake.output_contains("Version=1 DriverVersion=1"), "amd64: status names the package and the ICD versions");
    put_package(fake, 40, radeon::machine_amd64);
    expect(run(fake, {L"upgrade"}) == radeon::exit_code::restart_required &&
               installed(fake, radeon::feature_opengl_icd), "amd64: upgrade replaces the ICD and keeps it registered");
    expect(run(fake, {L"rollback"}) == radeon::exit_code::restart_required &&
               same_bytes(fake.file(fake.live(radeon::icd_component)), v1) && icd_registered(fake) &&
               journal_state(fake, radeon::JournalState::installed) && no_transaction_leftovers(fake),
           "amd64: rollback restores the previous ICD and its registration");
    expect(run(fake, {L"remove"}) == radeon::exit_code::restart_required && clean(fake, L""),
           "amd64: remove deletes the ICD and the OpenGLDrivers\\RadeonGL key");

    // An i386 ICD (or any i386 image) in the amd64 package changes nothing.
    FakeSystem mixed;
    mixed.machine = radeon::machine_amd64;
    put_package(mixed, 1, radeon::machine_amd64);
    mixed.add_device(0x744c, L"", false);
    mixed.put_file(mixed.packaged(L"radeonglicd.dll"), make_pe(radeon::machine_i386, radeon::ImageKind::user_dll, 9));
    expect(run(mixed, {L"install", L"--opengl-icd"}) == radeon::exit_code::failure && mixed.mutations == 0,
           "amd64: an i386 ICD is refused before any change");
    FakeSystem x86;
    put_package(x86, 1, radeon::machine_amd64);
    x86.add_device(0x744c, L"", false);
    expect(run(x86, {L"install", L"--opengl-icd"}) == radeon::exit_code::failure && x86.mutations == 0,
           "an amd64 package is refused by the i386 radeonctl");
}

void remove_complete() {
    FakeSystem fake;
    prepare(fake);
    put_firmware(fake);
    fake.set_string(L"SYSTEM\\CurrentControlSet\\Services\\vbemp", L"ImagePath", L"system32\\drivers\\vbemp.sys");
    fake.set_string(L"SOFTWARE\\Microsoft\\Windows NT\\CurrentVersion\\OpenGLDrivers\\QemuLightning", L"Dll",
                    L"QemuLightningOpenGL.dll");
    expect(run(fake, {L"install", L"--asic", L"--opengl-icd"}) == radeon::exit_code::restart_required,
           "installed before removal");
    fake.put_file(fake.system + L"\\radeonglicd.dll.unrelated", Bytes(4, 4));
    expect(run(fake, {L"remove"}) == radeon::exit_code::restart_required && clean(fake, L""),
           "remove deletes binding, service, files, ICD registration and journal");
    expect(!fake.has_file(L"C:\\ReactOS\\inf\\oem7.inf"), "the staged OEM INF is uninstalled");
    expect(fake.has_file(fake.firmware("smu_13_0_0.bin")) && fake.has_file(fake.firmware("rx7900.rom")),
           "administrator firmware is retained");
    expect(fake.string(L"SYSTEM\\CurrentControlSet\\Services\\vbemp", L"ImagePath") == L"system32\\drivers\\vbemp.sys" &&
               fake.string(L"SOFTWARE\\Microsoft\\Windows NT\\CurrentVersion\\OpenGLDrivers\\QemuLightning", L"Dll") ==
                   L"QemuLightningOpenGL.dll" && fake.has_file(fake.system + L"\\radeonglicd.dll.unrelated"),
           "other drivers' settings, ICDs and files are untouched");

    FakeSystem modified;
    prepare(modified);
    expect(run(modified, {L"install"}) == radeon::exit_code::restart_required, "installed for the altered case");
    modified.put_file(modified.live(1), Bytes(64, 9));
    expect(run(modified, {L"remove"}) == radeon::exit_code::restart_required &&
               modified.has_file(modified.live(1)) && !modified.has_file(modified.live(0)),
           "a file whose contents are not a recorded version is preserved");
}

// ---- In-place unbinding (pnp_unbind.hpp, shared with lightningnt5ctl) ----

// Minimal HKLM model for unbind_in_place: values by key, keys with subkey
// checks, a mutation log and a failure injected at the Nth mutation.
struct UnbindRegistry final {
    std::map<std::wstring, std::map<std::wstring, std::wstring>> strings;
    std::map<std::wstring, std::map<std::wstring, uint32_t>> dwords;
    std::set<std::wstring> keys;
    std::vector<std::wstring> log;
    size_t fail_at = 0;  // 1-based mutation that fails with fail_error
    uint32_t fail_error = radeon::error::gen_failure;

    static std::wstring folded(const wchar_t* text) { return rx7900_test::radeon_fake::fold(text); }
    bool failing(const std::wstring& entry) {
        log.push_back(entry);
        return fail_at != 0 && log.size() >= fail_at;
    }
    uint32_t read_string(const wchar_t* key, const wchar_t* name, wchar_t* output, size_t capacity) {
        output[0] = 0;
        const auto found = strings.find(folded(key));
        if (found == strings.end() || found->second.count(folded(name)) == 0) {
            return radeon::error::file_not_found;
        }
        const std::wstring& text = found->second.at(folded(name));
        if (text.size() + 1 > capacity) {
            return radeon::error::invalid_data;
        }
        std::memcpy(output, text.c_str(), (text.size() + 1) * sizeof(wchar_t));
        return radeon::error::success;
    }
    uint32_t read_dword(const wchar_t* key, const wchar_t* name, uint32_t& value) {
        const auto found = dwords.find(folded(key));
        if (found == dwords.end() || found->second.count(folded(name)) == 0) {
            return radeon::error::file_not_found;
        }
        value = found->second.at(folded(name));
        return radeon::error::success;
    }
    uint32_t write_dword(const wchar_t* key, const wchar_t* name, uint32_t value) {
        if (failing(L"set " + folded(name))) {
            return fail_error;
        }
        dwords[folded(key)][folded(name)] = value;
        return radeon::error::success;
    }
    uint32_t delete_value(const wchar_t* key, const wchar_t* name) {
        if (failing(L"delete " + folded(name))) {
            return fail_error;
        }
        const bool erased = strings[folded(key)].erase(folded(name)) != 0;
        return erased ? radeon::error::success : radeon::error::file_not_found;
    }
    uint32_t delete_key(const wchar_t* key) {
        const std::wstring target = folded(key);
        if (failing(L"key " + target)) {
            return fail_error;
        }
        for (const auto& other : keys) {
            if (other.size() > target.size() && other.compare(0, target.size(), target) == 0 &&
                other[target.size()] == L'\\') {
                return radeon::error::access_denied;
            }
        }
        return keys.erase(target) != 0 ? radeon::error::success : radeon::error::file_not_found;
    }
    uint32_t flush(const wchar_t*) { return radeon::error::success; }
};

constexpr wchar_t unbind_instance[] = L"PCI\\VEN_1234&DEV_11F1&SUBSYS_11001AF4&REV_00\\3&609b8881&03";
constexpr wchar_t unbind_enum_key[] =
    L"system\\currentcontrolset\\enum\\pci\\ven_1234&dev_11f1&subsys_11001af4&rev_00\\3&609b8881&03";
constexpr wchar_t unbind_class_key[] =
    L"system\\currentcontrolset\\control\\class\\{4d36e97d-e325-11ce-bfc1-08002be10318}\\0018";

// A Lightning display devnode as lightningnt5ctl display-install leaves it on ReactOS.
UnbindRegistry lightning_devnode() {
    UnbindRegistry registry;
    auto& values = registry.strings[unbind_enum_key];
    values[L"service"] = L"LightningVideo";
    values[L"driver"] = L"{4d36e97d-e325-11ce-bfc1-08002be10318}\\0018";
    values[L"class"] = L"System";
    values[L"classguid"] = L"{4d36e97d-e325-11ce-bfc1-08002be10318}";
    values[L"upperfilters"] = L"LightningNT5";
    values[L"hardwareid"] = L"PCI\\VEN_1234&DEV_11F1";
    registry.dwords[unbind_enum_key][L"configflags"] = 0x60;
    registry.keys.insert(unbind_enum_key);
    registry.keys.insert(unbind_class_key);
    registry.keys.insert(std::wstring(unbind_class_key) + L"\\settings");
    return registry;
}

bool lightning_unbound(const UnbindRegistry& registry, bool filters_deleted) {
    const auto& values = registry.strings.at(unbind_enum_key);
    return values.count(L"service") == 0 && values.count(L"driver") == 0 && values.count(L"class") == 0 &&
           values.count(L"classguid") == 0 && (values.count(L"upperfilters") == 0) == filters_deleted &&
           values.count(L"hardwareid") == 1 && registry.dwords.at(unbind_enum_key).at(L"configflags") == 0 &&
           registry.keys.count(unbind_class_key) == 0 && registry.keys.count(unbind_enum_key) == 1;
}

void pnp_unbind_algorithm() {
    using rx7900::UnbindStep;
    for (const bool filters : {true, false}) {
        UnbindRegistry registry = lightning_devnode();
        UnbindStep step = UnbindStep::open_device;
        expect(rx7900::unbind_in_place(registry, unbind_instance, L"lightningvideo", filters, step) ==
                       radeon::error::success && step == UnbindStep::none && lightning_unbound(registry, filters),
               filters ? "unbind: binding, driver key, filters and failed-install marks are removed"
                       : "unbind: the binding is removed and the filters are kept");
        expect(!registry.log.empty() && registry.log.back() == L"delete service",
               "unbind: the Service value is deleted last");
        const size_t mutations = registry.log.size();
        expect(rx7900::unbind_in_place(registry, unbind_instance, L"LightningVideo", filters, step) ==
                       radeon::error::success && registry.log.size() == mutations && step == UnbindStep::none,
               "unbind: an unbound device is left alone");

        // Interrupted at every mutation, a repeated run completes the same state.
        for (size_t point = 1; point <= mutations; ++point) {
            UnbindRegistry crashed = lightning_devnode();
            crashed.fail_at = point;
            rx7900::unbind_in_place(crashed, unbind_instance, L"LightningVideo", filters, step);
            crashed.fail_at = 0;
            const bool completed = rx7900::unbind_in_place(crashed, unbind_instance, L"LightningVideo", filters,
                                                           step) == radeon::error::success &&
                                   lightning_unbound(crashed, filters);
            expect(completed, "unbind: a run interrupted at any mutation is completed by the next one");
        }
    }

    UnbindRegistry foreign = lightning_devnode();
    rx7900::UnbindStep step = rx7900::UnbindStep::none;
    expect(rx7900::unbind_in_place(foreign, unbind_instance, L"rx7900video", true, step) ==
                   radeon::error::access_denied && step == rx7900::UnbindStep::open_device && foreign.log.empty(),
           "unbind: a device bound to another service is refused unchanged");
    UnbindRegistry malformed = lightning_devnode();
    malformed.strings[unbind_enum_key][L"driver"] = L"..\\..\\Services\\vga";
    expect(rx7900::unbind_in_place(malformed, unbind_instance, L"LightningVideo", true, step) ==
                   radeon::error::invalid_data && step == rx7900::UnbindStep::driver_key && malformed.log.empty(),
           "unbind: a Driver value that is not {GUID}\\NNNN deletes nothing");
    for (const wchar_t* shape : {L"{4d36e97d-e325-11ce-bfc1-08002be1031g}\\0018",
                                 L"{4d36e97d-e325-11ce-bfc1\\08002be10318}\\0018",
                                 L"{4d36e97d-e325-11ce-bfc1-08002be10318}\\..\\x"}) {
        UnbindRegistry odd = lightning_devnode();
        odd.strings[unbind_enum_key][L"driver"] = shape;
        expect(rx7900::unbind_in_place(odd, unbind_instance, L"LightningVideo", true, step) ==
                       radeon::error::invalid_data && odd.log.empty(),
               "unbind: a Driver value with a malformed GUID or index deletes nothing");
    }
    UnbindRegistry refused = lightning_devnode();
    refused.fail_at = 3;  // Settings, driver key, then the Driver value
    refused.fail_error = radeon::error::access_denied;
    expect(rx7900::unbind_in_place(refused, unbind_instance, L"LightningVideo", true, step) ==
                   radeon::error::access_denied && step == rx7900::UnbindStep::binding_values,
           "unbind: the failing step is reported");
    wchar_t path[rx7900::pnp_unbind::key_characters]{};
    expect(!rx7900::pnp_unbind::enum_key_path(L"..\\Services", path) &&
               !rx7900::pnp_unbind::enum_key_path(L"\\PCI\\X", path) &&
               !rx7900::pnp_unbind::enum_key_path(L"PCI\\\\X", path) &&
               !rx7900::pnp_unbind::enum_key_path(L"PCI\\X\\", path) &&
               rx7900::pnp_unbind::enum_key_path(unbind_instance, path),
           "unbind: only plain relative instance IDs name an Enum key");
}

// ReactOS 0.4.16 as the installer rig observed it (2026-10-25 VM run): DIF_REMOVE
// fails with ERROR_CALL_NOT_IMPLEMENTED, SetupUninstallOEMInf deletes nothing
// without SUOI_FORCEDELETE and the active display names LIGHTNINGVIDEO.
void reactos_removal() {
    const std::wstring staged_inf = L"C:\\ReactOS\\inf\\oem7.inf";
    {
        FakeSystem fake;
        prepare(fake, L"vga", true);
        fake.reactos_setupapi = true;
        fake.redetect_service = L"vga";
        fake.active_service = L"LightningVideo";
        FakeSystem::set_filters(fake.devices.front().record, true, {L"VendorUpper"});
        expect(run(fake, {L"install", L"--force-replace", L"vga"}) == radeon::exit_code::restart_required &&
                   installed(fake, 0), "ReactOS: install replaces the default vga binding");
        fake.output.clear();
        expect(run(fake, {L"status"}) == radeon::exit_code::success &&
                   fake.output_contains("service LIGHTNINGVIDEO"), "ReactOS: status reports the upper-cased display service");
        fake.output.clear();
        expect(run(fake, {L"remove"}) == radeon::exit_code::restart_required && clean(fake, L"") &&
                   fake.output_contains("Removed the rx7900video binding of") &&
                   fake.output_contains("not implemented here") &&
                   fake.output_contains("Restart ReactOS to complete the removal"),
               "ReactOS: remove unbinds in place when DIF_REMOVE is unimplemented");
        const auto& device = fake.devices.front();
        expect(device.present && device.unbound && device.record.service[0] == 0 && device.inf_path.empty() &&
                   FakeSystem::filters(device.record, true) == std::vector<std::wstring>({L"VendorUpper"}),
               "ReactOS: the devnode keeps its instance and filters, its binding is gone");
        expect(!fake.has_file(staged_inf) && fake.output_contains("Removed the staged driver package oem7.inf"),
               "ReactOS: the staged OEM INF is force-deleted after the no-op uninstall");
        fake.output.clear();
        expect(run(fake, {L"remove"}) == radeon::exit_code::success &&
                   fake.output_contains("Nothing of this package is installed"), "ReactOS: a repeated remove is a no-op");
        fake.restart();
        fake.redetect();
        expect(radeon::text_equal_insensitive(fake.devices.front().record.service, L"vga") &&
                   FakeSystem::filters(fake.devices.front().record, true) == std::vector<std::wstring>({L"VendorUpper"}),
               "ReactOS: after the restart the default driver binds the same devnode with its filters");
        fake.output.clear();
        expect(run(fake, {L"recover"}) == radeon::exit_code::success &&
                   fake.output_contains("No installation is recorded") && run(fake, {L"remove"}) == radeon::exit_code::success &&
                   run(fake, {L"install", L"--dry-run"}) == radeon::exit_code::refused,
               "ReactOS: recover, remove and install --dry-run after removal behave as on Windows");
    }
    {
        // A staged copy that another device's driver key still names is kept.
        FakeSystem fake;
        prepare(fake);
        fake.reactos_setupapi = true;
        expect(run(fake, {L"install"}) == radeon::exit_code::restart_required, "ReactOS: installed for the shared INF");
        auto& other = fake.add_device(0x744c, L"other", false, L"4&1A2B3C4D&0&0020");
        other.inf_path = L"OEM7.INF";
        fake.output.clear();
        expect(run(fake, {L"remove"}) == radeon::exit_code::restart_required && fake.has_file(staged_inf) &&
                   fake.output_contains("still used by") && journal_state(fake, radeon::JournalState::none),
               "ReactOS: an OEM INF still used by another device is never force-deleted");
    }
    {
        // Windows refuses to uninstall an INF a device still uses; that is respected.
        FakeSystem fake;
        prepare(fake);
        expect(run(fake, {L"install"}) == radeon::exit_code::restart_required, "installed for the shared INF");
        fake.add_device(0x744c, L"other", false, L"4&1A2B3C4D&0&0020").inf_path = L"oem7.inf";
        fake.output.clear();
        expect(run(fake, {L"remove"}) == radeon::exit_code::restart_required && fake.has_file(staged_inf) &&
                   fake.output_contains("SetupAPI kept the staged driver package oem7.inf") &&
                   journal_state(fake, radeon::JournalState::none),
               "an OEM INF that SetupAPI refuses to uninstall is kept, never forced");
    }
    {
        // A failing in-place step is named in the message (rig run 2 hit error 31).
        FakeSystem fake;
        prepare(fake);
        fake.reactos_setupapi = true;
        expect(run(fake, {L"install"}) == radeon::exit_code::restart_required, "ReactOS: installed for a failing unbind");
        fake.keys.insert(rx7900_test::radeon_fake::fold(
            (std::wstring(rx7900::pnp_unbind::class_root) + fake.devices.front().driver_key + L"\\Extra").c_str()));
        fake.output.clear();
        expect(run(fake, {L"remove"}) == radeon::exit_code::failure &&
                   fake.output_contains("in-place unbinding, deleting the driver (Control\\Class) key, Win32 error 5") &&
                   journal_state(fake, radeon::JournalState::removing) &&
                   radeon::text_equal_insensitive(fake.devices.front().record.service, radeon::service_name),
               "ReactOS: a failed unbinding step is named and the binding and journal are kept");
        fake.keys.erase(rx7900_test::radeon_fake::fold(
            (std::wstring(rx7900::pnp_unbind::class_root) + fake.devices.front().driver_key + L"\\Extra").c_str()));
        expect(run(fake, {L"recover"}) == radeon::exit_code::restart_required && clean(fake, L""),
               "ReactOS: recover completes the removal once the step can succeed");
    }
    {
        // An upper-cased service name (as ReactOS registry paths spell it) is
        // still recognised: service names are case-insensitive.
        FakeSystem fake;
        prepare(fake, L"VGA", true);
        fake.reactos_setupapi = true;
        expect(run(fake, {L"install", L"--force-replace", L"vga"}) == radeon::exit_code::restart_required,
               "service names compare case-insensitively for --force-replace");
        radeon::text_copy(fake.devices.front().record.service, L"RX7900VIDEO");
        fake.output.clear();
        expect(run(fake, {L"install"}) == radeon::exit_code::success && fake.output_contains("bound to rx7900video"),
               "an upper-cased rx7900video binding is recognised as ours");
        expect(run(fake, {L"remove"}) == radeon::exit_code::restart_required && clean(fake, L"") &&
                   fake.devices.front().unbound, "an upper-cased rx7900video binding is removed");
    }
}

void recovery_repairs() {
    FakeSystem fake;
    prepare(fake);
    put_firmware(fake);
    expect(run(fake, {L"install", L"--asic", L"--opengl-icd"}) == radeon::exit_code::restart_required,
           "installed for repair checks");
    const uint32_t features = radeon::feature_physical_asic_init | radeon::feature_opengl_icd;

    fake.keys.erase(rx7900_test::radeon_fake::fold(radeon::opengl_drivers_key));
    fake.registry.erase(rx7900_test::radeon_fake::fold(radeon::opengl_drivers_key));
    fake.output.clear();
    expect(run(fake, {L"status"}) == radeon::exit_code::success && fake.output_contains("MISSING"),
           "status flags a missing ICD registration");
    expect(run(fake, {L"recover"}) == radeon::exit_code::success && installed(fake, features),
           "recover repairs a missing ICD registration");

    fake.registry[rx7900_test::radeon_fake::fold(radeon::device0_key)].erase(L"physicalasicinit");
    fake.service.start = 0;  // something else made it boot start
    expect(run(fake, {L"recover"}) == radeon::exit_code::restart_required && installed(fake, features),
           "recover resets boot start to demand and rewrites lost opt-ins");

    fake.service = rx7900_test::radeon_fake::Service{};
    expect(run(fake, {L"recover"}) == radeon::exit_code::restart_required && installed(fake, features),
           "recover recreates a missing service of a bound device");

    fake.files.erase(rx7900_test::radeon_fake::fold(fake.live(0).c_str()));
    expect(run(fake, {L"recover"}) == radeon::exit_code::restart_required && installed(fake, features),
           "recover restores a deleted driver file from the matching package");

    // A stale service with no device and no record is removed.
    FakeSystem stale;
    put_package(stale, 1);
    stale.add_device(0x744c, L"vbemp", true);
    stale.service = rx7900_test::radeon_fake::Service{true, 1, 3, 0, false, radeon::service_image_systemroot};
    stale.set_string(radeon::opengl_drivers_key, L"Dll", L"radeonglicd.dll");
    expect(run(stale, {L"recover"}) == radeon::exit_code::success && !stale.service.exists &&
               stale.value(radeon::opengl_drivers_key, L"Dll") == nullptr,
           "recover deletes a stale unbound service and a registration whose DLL is missing");
    stale.service = rx7900_test::radeon_fake::Service{true, 1, 3, 0, false, L"\\SystemRoot\\System32\\drivers\\other.sys"};
    expect(run(stale, {L"recover"}) == radeon::exit_code::success && stale.service.exists,
           "a service of our name with a foreign image is never deleted");
}

// ---- Restart-deferred replacements ----

size_t count_mutations(void (*setup)(FakeSystem&), const std::vector<const wchar_t*>& command);

// Tag of the package build every listed component's live file came from, or
// -1 for a mixed or missing set.
int generation_tag(const FakeSystem& fake, size_t components) {
    const Bytes* first = fake.file(fake.live(0));
    if (first == nullptr || first->size() <= 0x200) {
        return -1;
    }
    const uint8_t tag = static_cast<uint8_t>((*first)[0x200] - static_cast<uint8_t>(0x200 * 7));
    for (size_t index = 0; index < components; ++index) {
        if (!same_bytes(fake.file(fake.live(index)), make_pe(radeon::machine_i386, radeon::component_files[index].image,
                                                             static_cast<uint8_t>(tag + index)))) {
            return -1;
        }
    }
    return tag;
}

// The journal records exactly the live files and nothing queued.
bool journal_settled(const FakeSystem& fake, size_t components) {
    for (size_t index = 0; index < components; ++index) {
        const std::wstring name = radeon::component_files[index].filename;
        const Bytes* live = fake.file(fake.live(index));
        if (live == nullptr || fake.value(radeon::journal_key, (L"Pending." + name).c_str()) != nullptr) {
            return false;
        }
        rx7900::tools::Sha256 hasher;
        hasher.update(live->data(), live->size());
        wchar_t text[rx7900::tools::sha256_hex_characters]{};
        rx7900::tools::sha256_hex(hasher.finish(), text);
        if (fake.string(radeon::journal_key, (L"Installed." + name).c_str()) != text) {
            return false;
        }
    }
    uint32_t pending = 1;
    return journal_state(fake, radeon::JournalState::installed) &&
           fake.dword(radeon::journal_key, L"RestartPending", pending) && pending == 0 && no_transaction_leftovers(fake);
}

void setup_deferred_upgrade(FakeSystem& fake) {
    prepare(fake);
    run(fake, {L"install"});
    fake.locked.insert(rx7900_test::radeon_fake::fold(fake.live(1).c_str()));
    put_package(fake, 70);
    run(fake, {L"upgrade"});
}

// Reproducer (installer-guest-rig): rollback after an upgrade whose file was
// replaced only at the next restart left a mixed set after the restart.
void rollback_after_deferred_upgrade() {
    FakeSystem locked;
    prepare(locked);
    expect(run(locked, {L"install"}) == radeon::exit_code::restart_required, "repro: install");
    const Bytes v1_video = contents(locked.file(locked.live(0)));
    const Bytes v1_disp = contents(locked.file(locked.live(1)));
    locked.locked.insert(rx7900_test::radeon_fake::fold(locked.live(1).c_str()));
    put_package(locked, 70);
    expect(run(locked, {L"upgrade"}) == radeon::exit_code::restart_required && !locked.pending_renames.empty(),
           "repro: upgrade defers radeondisp.dll");
    expect(run(locked, {L"upgrade"}) == radeon::exit_code::refused && run(locked, {L"install"}) == radeon::exit_code::refused,
           "upgrade and install refuse while a replacement is queued for the restart");
    put_package(locked, 1);
    expect(run(locked, {L"upgrade"}) == radeon::exit_code::refused,
           "an upgrade whose live files already match would still mix generations: refused while queued");
    put_package(locked, 70);
    const int rolled = run(locked, {L"rollback"});
    expect(rolled == radeon::exit_code::restart_required && !locked.has_file(locked.live(1) + L".radeonctl-new"),
           "rollback cancels the queued replacement by deleting its source");
    locked.restart();
    const int recovered = run(locked, {L"recover"});
    const bool video_v1 = same_bytes(locked.file(locked.live(0)), v1_video);
    const bool disp_v1 = same_bytes(locked.file(locked.live(1)), v1_disp);
    expect(recovered == radeon::exit_code::success && video_v1 && disp_v1 && journal_settled(locked, 2),
           "repro: after rollback and restart both files are the previous version");
    const Bytes* backup = locked.file(locked.drivers() + L"\\rx7900\\backup\\radeondisp.dll");
    expect(same_bytes(backup, contents(locked.file(locked.packaged(L"radeondisp.dll")))),
           "the cancelled generation is kept as the backup");
    expect(run(locked, {L"rollback"}) == radeon::exit_code::restart_required && generation_tag(locked, 2) == 70 &&
               journal_settled(locked, 2), "rolling back again (nothing locked) returns to the upgraded set");

    // An interrupted rollback in its staging phase keeps the queued copy and
    // its single queued rename: only that transaction's own copies go away.
    const size_t total = count_mutations(setup_deferred_upgrade, {L"rollback"});
    size_t staging_points = 0;
    for (size_t point = 1; point <= total; ++point) {
        FakeSystem fake;
        setup_deferred_upgrade(fake);
        fake.mutations = 0;
        fake.crash_at = point;
        run(fake, {L"rollback"});
        fake.crashed = false;
        fake.crash_at = 0;
        if (!journal_state(fake, radeon::JournalState::replace_staging)) {
            continue;
        }
        ++staging_points;
        run(fake, {L"recover"});
        expect(same_bytes(fake.file(fake.live(1) + L".radeonctl-new"), contents(fake.file(fake.packaged(L"radeondisp.dll")))) &&
                   fake.pending_renames.size() == 1, "staging recovery never touches an earlier queued replacement");
    }
    expect(staging_points != 0, "the staging phase of the cancelling rollback was interrupted at least once");

    // Every file deferred: rollback must not report "nothing to roll back".
    FakeSystem all;
    prepare(all);
    run(all, {L"install"});
    all.locked.insert(rx7900_test::radeon_fake::fold(all.live(0).c_str()));
    all.locked.insert(rx7900_test::radeon_fake::fold(all.live(1).c_str()));
    put_package(all, 70);
    expect(run(all, {L"upgrade"}) == radeon::exit_code::restart_required && generation_tag(all, 2) == 1,
           "an upgrade of loaded files leaves them live until the restart");
    expect(run(all, {L"rollback"}) == radeon::exit_code::restart_required, "rollback of a fully deferred upgrade acts");
    all.restart();
    expect(run(all, {L"recover"}) == radeon::exit_code::success && generation_tag(all, 2) == 1 &&
               journal_settled(all, 2), "after the restart the whole set is the previous version");

    // A lost queued rename (the OS failed it) is a mixed set; recover completes
    // the recorded generation from the matching package.
    FakeSystem lost;
    setup_deferred_upgrade(lost);
    lost.pending_renames.clear();
    lost.restart();
    expect(generation_tag(lost, 2) == -1, "a lost rename leaves a mixed set");
    expect(run(lost, {L"recover"}) == radeon::exit_code::restart_required && generation_tag(lost, 2) == 70 &&
               journal_settled(lost, 2), "recover completes a mixed set to the recorded generation");

    // Without any copy of the recorded generation, the whole verified backup
    // generation is restored instead.
    FakeSystem orphan;
    setup_deferred_upgrade(orphan);
    orphan.pending_renames.clear();
    orphan.restart();
    orphan.files.erase(rx7900_test::radeon_fake::fold((orphan.live(1) + L".radeonctl-new").c_str()));
    put_package(orphan, 120);
    expect(run(orphan, {L"recover"}) == radeon::exit_code::restart_required && generation_tag(orphan, 2) == 1 &&
               journal_settled(orphan, 2), "an incompletable mixed set returns wholly to the backup generation");

    // Every queued rename lost: the set is wholly the backup generation.
    FakeSystem adopted;
    prepare(adopted);
    run(adopted, {L"install"});
    adopted.locked.insert(rx7900_test::radeon_fake::fold(adopted.live(0).c_str()));
    adopted.locked.insert(rx7900_test::radeon_fake::fold(adopted.live(1).c_str()));
    put_package(adopted, 70);
    run(adopted, {L"upgrade"});
    adopted.pending_renames.clear();
    adopted.restart();
    expect(run(adopted, {L"recover"}) == radeon::exit_code::success && generation_tag(adopted, 2) == 1 &&
               journal_settled(adopted, 2) && adopted.output_contains("wholly the backup generation"),
           "a set wholly at the backup generation is recorded, not rewritten");
    expect(run(adopted, {L"upgrade"}) == radeon::exit_code::restart_required && journal_state(adopted, radeon::JournalState::installed),
           "after adoption the upgrade can be retried");
}

// The real Win32 boot identity (Wine/ReactOS only): stable within a boot.
void win32_boot_identity() {
#if defined(_WIN32)
    radeon::Win32System state;
    radeon::SystemOps ops{};
    uint64_t first = 0;
    uint64_t second = 0;
    expect(radeon::make_win32_system(state, ops) && ops.boot_id(ops.context, first) == radeon::error::success &&
               ops.boot_id(ops.context, second) == radeon::error::success && first != 0 && first == second,
           "NtQuerySystemInformation boot time identifies the current boot");
#endif
}

// ---- Crash matrices ----

using Setup = void (*)(FakeSystem&);
using Command = std::vector<const wchar_t*>;

size_t count_mutations(Setup setup, const Command& command) {
    FakeSystem fake;
    setup(fake);
    fake.mutations = 0;
    run(fake, command);
    return fake.mutations;
}

struct MatrixResult final {
    size_t points = 0;
    size_t forward = 0;
    size_t backward = 0;
};

template<typename Valid>
MatrixResult crash_matrix(const char* name, Setup setup, const Command& command, bool torn, Valid&& valid) {
    MatrixResult result{};
    const size_t total = count_mutations(setup, command);

    for (size_t point = 1; point <= total; ++point) {
        FakeSystem fake;
        setup(fake);
        fake.mutations = 0;
        fake.crash_at = point;
        fake.torn_copy = torn;
        run(fake, command);
        fake.crashed = false;
        fake.crash_at = 0;
        fake.torn_copy = false;
        const int recovered = run(fake, {L"recover"});
        const int outcome = valid(fake);
        ++result.points;
        result.forward += outcome == 1 ? 1 : 0;
        result.backward += outcome == 2 ? 1 : 0;
        expect(recovered != radeon::exit_code::failure && outcome != 0,
               "every crash point recovers to a complete or clean state");
        if (recovered == radeon::exit_code::failure || outcome == 0) {
            std::printf("  %s crash at mutation %zu/%zu: recover=%d state invalid\n", name, point, total, recovered);
            if (std::getenv("RX7900_INSTALLER_VERBOSE") != nullptr) {
                for (const auto& line : fake.output) {
                    std::printf("    | %s\n", line.c_str());
                }
            }
            continue;
        }
        const Snapshot settled = fake.snapshot();
        run(fake, {L"recover"});
        const bool idempotent = fake.snapshot() == settled;
        expect(idempotent, "recovery is idempotent");
        if (!idempotent) {
            std::printf("  %s crash at mutation %zu: second recover changed state\n", name, point);
        }
    }
    std::printf("%s: %zu crash points, %zu completed forward, %zu rolled back\n", name, result.points,
                result.forward, result.backward);
    expect(result.points == total && total > 3, "the matrix covers every mutation of the command");
    return result;
}

void setup_fresh(FakeSystem& fake) {
    put_package(fake, 1);
    FakeSystem::set_filters(fake.add_device(0x744c, L"", false).record, true, {L"VendorUpper"});
    put_firmware(fake);
}

void setup_current(FakeSystem& fake) {
    setup_fresh(fake);
    run(fake, {L"install", L"--asic", L"--opengl-icd"});
}

void setup_current_reactos(FakeSystem& fake) {
    fake.reactos_setupapi = true;
    setup_current(fake);
}

void setup_installed(FakeSystem& fake) {
    setup_current(fake);
    put_package(fake, 90);
}

void setup_upgraded(FakeSystem& fake) {
    setup_installed(fake);
    run(fake, {L"upgrade"});
}

bool consistent_version(const FakeSystem& fake, uint8_t& tag) {
    // All three live files must come from the same package build.
    const Bytes* first = fake.file(fake.live(0));
    if (first == nullptr || first->size() <= 0x200) {
        return false;
    }
    const uint8_t first_tag = static_cast<uint8_t>((*first)[0x200] - static_cast<uint8_t>(0x200 * 7));
    tag = first_tag;
    for (size_t index = 0; index < radeon::component_count; ++index) {
        const Bytes* live = fake.file(fake.live(index));
        if (live == nullptr ||
            *live != make_pe(radeon::machine_i386, radeon::component_files[index].image,
                             static_cast<uint8_t>(first_tag + index))) {
            return false;
        }
    }
    return true;
}

void crash_matrices() {
    const uint32_t features = radeon::feature_physical_asic_init | radeon::feature_opengl_icd;

    crash_matrix("install", setup_fresh, {L"install", L"--asic", L"--opengl-icd"}, false, [&](FakeSystem& fake) {
        const bool filters = FakeSystem::filters(fake.devices.front().record, true) ==
                             std::vector<std::wstring>({L"VendorUpper"});
        if (installed(fake, features) && filters) {
            return 1;
        }
        if (clean(fake, L"") && filters) {
            // A clean rollback must allow the install to be retried.
            return run(fake, {L"install", L"--asic", L"--opengl-icd"}) == radeon::exit_code::restart_required &&
                           installed(fake, features) ? 2 : 0;
        }
        return 0;
    });

    for (const bool torn : {false, true}) {
        crash_matrix(torn ? "upgrade (torn copies)" : "upgrade", setup_installed, {L"upgrade"}, torn,
                     [&](FakeSystem& fake) {
            uint8_t tag = 0;
            if (!consistent_version(fake, tag) || !journal_state(fake, radeon::JournalState::installed) ||
                !no_transaction_leftovers(fake) || !icd_registered(fake)) {
                return 0;
            }
            // Retrying after recovery reaches the new version.
            const int retried = run(fake, {L"upgrade"});
            return (retried == radeon::exit_code::success || retried == radeon::exit_code::restart_required) &&
                           installed(fake, features) ? (tag == 90 ? 1 : 2) : 0;
        });
    }

    crash_matrix("rollback", setup_upgraded, {L"rollback"}, false, [&](FakeSystem& fake) {
        uint8_t tag = 0;
        if (!consistent_version(fake, tag) || !journal_state(fake, radeon::JournalState::installed) ||
            !no_transaction_leftovers(fake)) {
            return 0;
        }
        return tag == 1 ? 1 : (tag == 90 ? 2 : 0);
    });

    crash_matrix("remove", setup_current, {L"remove"}, false, [&](FakeSystem& fake) {
        // Before the removal journal commits nothing changed: still installed,
        // and a retried remove succeeds. Afterwards removal always continues.
        int direction = 1;
        if (installed(fake, features)) {
            if (run(fake, {L"remove"}) != radeon::exit_code::restart_required) {
                return 0;
            }
            direction = 2;
        }
        const bool removed = !fake.service.exists && !fake.has_file(fake.live(0)) && !fake.has_file(fake.live(1)) &&
                             !fake.has_file(fake.live(2)) && fake.value(radeon::opengl_drivers_key, L"Dll") == nullptr &&
                             no_transaction_leftovers(fake);
        if (!removed) {
            return 0;
        }
        if (journal_state(fake, radeon::JournalState::filters_pending)) {
            fake.redetect();
            run(fake, {L"recover"});
        }
        return journal_state(fake, radeon::JournalState::none) &&
                       FakeSystem::filters(fake.devices.front().record, true) ==
                           std::vector<std::wstring>({L"VendorUpper"}) ? direction : 0;
    });

    // ReactOS: DIF_REMOVE is unimplemented, the binding is cleared in place and
    // the staged INF is force-deleted. Every crash point ends with nothing of
    // ours left, the filters on the devnode and the default driver after restart.
    crash_matrix("remove (ReactOS SetupAPI)", setup_current_reactos, {L"remove"}, false, [&](FakeSystem& fake) {
        int direction = 1;
        if (installed(fake, features)) {
            if (run(fake, {L"remove"}) != radeon::exit_code::restart_required) {
                return 0;
            }
            direction = 2;
        }
        const bool removed = !fake.service.exists && !fake.has_file(fake.live(0)) && !fake.has_file(fake.live(1)) &&
                             !fake.has_file(fake.live(2)) && fake.value(radeon::opengl_drivers_key, L"Dll") == nullptr &&
                             !fake.has_file(L"C:\\ReactOS\\inf\\oem7.inf") && no_transaction_leftovers(fake);
        if (!removed) {
            return 0;
        }
        if (journal_state(fake, radeon::JournalState::filters_pending)) {
            run(fake, {L"recover"});
        }
        if (!journal_state(fake, radeon::JournalState::none)) {
            return 0;
        }
        fake.restart();
        fake.redetect();
        const auto& device = fake.devices.front();
        return device.present && radeon::text_equal_insensitive(device.record.service, L"vbemp") &&
                       FakeSystem::filters(device.record, true) == std::vector<std::wstring>({L"VendorUpper"}) ? direction
                                                                                                             : 0;
    });

    // Deferred replacements: crash anywhere, recover, restart, recover again;
    // the live set is always one whole generation and the record matches it.
    const auto deferred_valid = [&](FakeSystem& fake) {
        fake.restart();
        if (run(fake, {L"recover"}) == radeon::exit_code::failure) {
            return 0;
        }
        const int tag = generation_tag(fake, 2);
        if (!journal_settled(fake, 2) || (tag != 1 && tag != 70)) {
            return 0;
        }
        return tag == 1 ? 1 : 2;
    };
    crash_matrix("rollback after deferred upgrade", setup_deferred_upgrade, {L"rollback"}, false, deferred_valid);
    crash_matrix("deferred upgrade", [](FakeSystem& fake) {
        prepare(fake);
        run(fake, {L"install"});
        fake.locked.insert(rx7900_test::radeon_fake::fold(fake.live(1).c_str()));
        put_package(fake, 70);
    }, {L"upgrade"}, false, [&](FakeSystem& fake) {
        const int outcome = deferred_valid(fake);
        return outcome == 0 ? 0 : (outcome == 2 ? 1 : 2);
    });

    // A failing binding (SetupAPI error) rolls back without a crash.
    FakeSystem failing;
    setup_fresh(failing);
    failing.install_fails = true;
    expect(run(failing, {L"install"}) == radeon::exit_code::failure && clean(failing, L"") &&
               FakeSystem::filters(failing.devices.front().record, true) == std::vector<std::wstring>({L"VendorUpper"}),
           "a refused binding leaves the system clean");
    // A copy that silently returns wrong bytes must never become a live file.
    FakeSystem corrupt;
    setup_installed(corrupt);
    const Bytes before = contents(corrupt.file(corrupt.live(1)));
    corrupt.corrupt_copy = L"radeondisp.dll.radeonctl-new";
    expect(run(corrupt, {L"upgrade"}) == radeon::exit_code::failure && same_bytes(corrupt.file(corrupt.live(1)), before) &&
               no_transaction_leftovers(corrupt) && journal_state(corrupt, radeon::JournalState::installed),
           "a corrupted staged copy aborts the upgrade with the old files intact");
    corrupt.corrupt_copy.clear();
    expect(run(corrupt, {L"upgrade"}) == radeon::exit_code::restart_required &&
               installed(corrupt, radeon::feature_physical_asic_init | radeon::feature_opengl_icd),
           "the upgrade succeeds once copies are faithful");

    FakeSystem empty_bind;
    setup_fresh(empty_bind);
    empty_bind.install_binds_without_files = true;
    expect(run(empty_bind, {L"install"}) == radeon::exit_code::restart_required && installed(empty_bind, 0),
           "a binding that copied no files is completed from the package");
}

// ---- Optional real win32k evidence ----

int scan_images(int argc, char** argv) {
    int failed = 0;
    for (int index = 1; index + 1 < argc; index += 2) {
        FILE* file = std::fopen(argv[index], "rb");
        if (file == nullptr) {
            std::printf("cannot open %s\n", argv[index]);
            ++failed;
            continue;
        }
        radeon::Win32kScanner scanner;
        std::vector<uint8_t> chunk(65536);
        size_t count = 0;
        while ((count = std::fread(chunk.data(), 1, chunk.size(), file)) != 0) {
            scanner.feed(rx7900::ByteView(chunk.data(), count));
        }
        std::fclose(file);
        unsigned mask = 0;
        for (size_t capability = 0; capability < radeon::win32k_capability_count; ++capability) {
            mask |= scanner.presence(capability) == radeon::PatchPresence::present ? 1u << capability : 0u;
        }
        const unsigned expected = static_cast<unsigned>(std::strtoul(argv[index + 1], nullptr, 0));
        std::printf("%s: symbols=%d capability mask 0x%x (expected 0x%x)\n", argv[index],
                    scanner.symbols_present() ? 1 : 0, mask, expected);
        failed += mask == expected ? 0 : 1;
    }
    return failed;
}

} // namespace

// Writes the synthetic package (PE headers only, never executable code) for the
// Wine runs of the real radeonctl binary: radeonctl reads it, never loads it.
int write_package(const char* directory) {
    int failed = 0;
    for (size_t index = 0; index < radeon::component_count; ++index) {
        const Bytes bytes = make_pe(radeon::machine_i386, radeon::component_files[index].image,
                                    static_cast<uint8_t>(1 + index));
        const std::string path = std::string(directory) + "/" + narrow(radeon::component_files[index].filename);
        FILE* file = std::fopen(path.c_str(), "wb");
        failed += file == nullptr || std::fwrite(bytes.data(), 1, bytes.size(), file) != bytes.size() ? 1 : 0;
        if (file != nullptr) {
            std::fclose(file);
        }
    }
    return failed;
}

// --check-package <directory> <i386|amd64>: the built package's PE images
// against the installer's own policy (machine, PE32/PE32+, subsystem, DLL flag).
int check_package(const char* directory, const char* arch) {
    const uint16_t machine = std::strcmp(arch, "amd64") == 0 ? radeon::machine_amd64 : radeon::machine_i386;
    int failed = 0;
    for (size_t index = 0; index < radeon::component_count; ++index) {
        const std::string path = std::string(directory) + "/" + narrow(radeon::component_files[index].filename);
        FILE* file = std::fopen(path.c_str(), "rb");
        Bytes bytes(4096, 0);
        const size_t count = file != nullptr ? std::fread(bytes.data(), 1, bytes.size(), file) : 0;
        long size = 0;
        if (file != nullptr && std::fseek(file, 0, SEEK_END) == 0) {
            size = std::ftell(file);
        }
        if (file != nullptr) {
            std::fclose(file);
        }
        uint32_t offset = 0;
        const rx7900::ByteView dos(bytes.data(), 64);
        bool valid = count >= 64 && size > 0 && radeon::pe_header_offset(dos, static_cast<uint64_t>(size), offset) &&
                     offset + radeon::pe_headers_bytes <= count;
        valid = valid && radeon::check_pe_headers(rx7900::ByteView(bytes.data() + offset, radeon::pe_headers_bytes),
                                                  machine, radeon::component_files[index].image) ==
                             radeon::PeStatus::valid;
        std::printf("%s: %s\n", path.c_str(), valid ? "valid" : "REFUSED");
        failed += valid ? 0 : 1;
    }
    return failed;
}

int main(int argc, char** argv) {
    if (argc == 4 && std::strcmp(argv[1], "--check-package") == 0) {
        return check_package(argv[2], argv[3]) == 0 ? 0 : 1;
    }
    if (argc == 3 && std::strcmp(argv[1], "--write-package") == 0) {
        return write_package(argv[2]) == 0 ? 0 : 1;
    }
    if (argc > 1) {
        return scan_images(argc, argv) == 0 ? 0 : 1;
    }

    sha256_vectors();
    feature_policy();
    command_line_policy();
    identity_policy();
    binding_policy();
    pe_policy();
    filter_policy();
    firmware_policy();
    scanner_policy();
    replace_policy();
    inf_contract();
    lightning_vm();
    fresh_install();
    refusals();
    features_install();
    profile_key();
    filter_preservation();
    upgrade_and_rollback();
    amd64_icd_lifecycle();
    remove_complete();
    pnp_unbind_algorithm();
    reactos_removal();
    recovery_repairs();
    rollback_after_deferred_upgrade();
    win32_boot_identity();
    crash_matrices();

    std::printf("Radeon installer: %u checks, %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

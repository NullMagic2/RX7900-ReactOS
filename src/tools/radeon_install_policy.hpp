/*
Module: rx7900.tools.radeon_install_policy
Purpose: Own every naming, safety and decision rule of the physical Radeon installer.
Created: 2026-10-24
Architecture: Pure C++17 policy with no OS calls, shared by radeonctl (through
radeon_installer and radeon_win32_system) and the host/Wine installer tests.
Device identities come from src/common/pci.hpp, firmware names from
src/hardware/ucode and firmware, ICD versions from radeon_opengl_info.hpp and the
filter merge from src/common/multisz.hpp. Nothing here reads or writes hardware.
*/
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "byte_view.hpp"
#include "sha256.hpp"

namespace rx7900::tools::radeon {

// Win32 numeric error codes are the shared result currency of the engine, its
// Win32 binding and the fake test system. Only the codes the engine inspects.
namespace error {
constexpr uint32_t success = 0;
constexpr uint32_t file_not_found = 2;
constexpr uint32_t path_not_found = 3;
constexpr uint32_t access_denied = 5;
constexpr uint32_t invalid_data = 13;
constexpr uint32_t sharing_violation = 32;
constexpr uint32_t not_supported = 50;
constexpr uint32_t duplicate_name = 52;
constexpr uint32_t device_not_present = 55;
constexpr uint32_t file_exists = 80;
constexpr uint32_t invalid_parameter = 87;
constexpr uint32_t insufficient_buffer = 122;
constexpr uint32_t already_exists = 183;
constexpr uint32_t bad_exe_format = 193;
constexpr uint32_t more_data = 234;
constexpr uint32_t no_more_items = 259;
constexpr uint32_t service_does_not_exist = 1060;
constexpr uint32_t service_exists = 1073;
constexpr uint32_t service_marked_for_delete = 1072;
constexpr uint32_t buffer_overflow = 111;
constexpr uint32_t call_not_implemented = 120;
constexpr uint32_t revision_mismatch = 1306;
constexpr uint32_t read_fault = 30;
constexpr uint32_t gen_failure = 31;
} // namespace error

// Registry value types (winnt.h values).
namespace value_type {
constexpr uint32_t string = 1;       // REG_SZ
constexpr uint32_t dword = 4;        // REG_DWORD
constexpr uint32_t multi_string = 7; // REG_MULTI_SZ
} // namespace value_type

// Process exit codes of radeonctl.
namespace exit_code {
constexpr int success = 0;
constexpr int failure = 1;
constexpr int usage = 2;
constexpr int refused = 3;
constexpr int no_navi31 = 4;
constexpr int restart_required = 3010;
} // namespace exit_code

constexpr size_t path_characters = 260;            // MAX_PATH
constexpr size_t instance_characters = 201;        // MAX_DEVICE_ID_LEN plus NUL
constexpr size_t service_name_characters = 257;    // SCM limit plus NUL
constexpr size_t filter_characters = 1024;         // per filter MULTI_SZ, terminators included
constexpr size_t device_capacity = 4;              // more Navi31 functions are reported, never bound

// Package version; must equal the DriverVer version of radeondisplay.inf.
inline constexpr wchar_t package_version[] = L"0.1.0.0";
inline constexpr wchar_t inf_filename[] = L"radeondisplay.inf";
inline constexpr wchar_t service_name[] = L"rx7900video";
inline constexpr wchar_t service_display_name[] = L"AMD Radeon RX 7900 (Navi31) experimental display";
inline constexpr wchar_t service_image_systemroot[] = L"\\SystemRoot\\System32\\drivers\\rx7900video.sys";
inline constexpr wchar_t service_group[] = L"Video";
inline constexpr wchar_t display_driver_name[] = L"radeondisp";
inline constexpr wchar_t display_class_guid[] = L"{4D36E968-E325-11CE-BFC1-08002BE10318}";
inline constexpr wchar_t lightning_hardware_id[] = L"PCI\\VEN_1234&DEV_11F1";

// SCM values written by the INF and recovery: kernel driver, demand start,
// ignore errors (a failed experimental display must not stop boot).
constexpr uint32_t service_type_kernel = 1;
constexpr uint32_t service_start_demand = 3;
constexpr uint32_t service_error_ignore = 0;

// HKLM-relative registry locations. VideoPort reads miniport parameters from the
// service Device0 key, or from the hardware-profile copy when GraphicsDrivers
// key UseNewKey exists and that copy exists (ReactOS videoprt IntCreateRegistryPath).
inline constexpr wchar_t service_key[] = L"SYSTEM\\CurrentControlSet\\Services\\rx7900video";
inline constexpr wchar_t device0_key[] = L"SYSTEM\\CurrentControlSet\\Services\\rx7900video\\Device0";
inline constexpr wchar_t video_subkey[] = L"SYSTEM\\CurrentControlSet\\Services\\rx7900video\\Video";
inline constexpr wchar_t profile_device0_key[] =
    L"SYSTEM\\CurrentControlSet\\Hardware Profiles\\Current\\System\\CurrentControlSet\\Services\\rx7900video\\Device0";
inline constexpr wchar_t use_new_key[] = L"SYSTEM\\CurrentControlSet\\Control\\GraphicsDrivers\\UseNewKey";
inline constexpr wchar_t opengl_drivers_key[] = L"SOFTWARE\\Microsoft\\Windows NT\\CurrentVersion\\OpenGLDrivers\\RadeonGL";
inline constexpr wchar_t journal_key[] = L"SOFTWARE\\RX7900\\RadeonInstaller";
inline constexpr wchar_t device_map_video_key[] = L"HARDWARE\\DEVICEMAP\\VIDEO";

// Explicit opt-in features. Each miniport value is read by rx7900video as a
// REG_DWORD equal to 1; every other value, including absence, means off.
enum feature : uint32_t {
    feature_physical_asic_init = 1u << 0,
    feature_atom_rom_override = 1u << 1,
    feature_vbe_mode_set = 1u << 2,
    feature_radeon_2d = 1u << 3,
    feature_radeon_2d_compute = 1u << 4,
    feature_hardware_cursor = 1u << 5,
    feature_opengl_icd = 1u << 6,
};
constexpr uint32_t known_features = (1u << 7) - 1;

struct FeatureRule final {
    uint32_t feature;
    const wchar_t* option;          // radeonctl install flag
    const wchar_t* registry_value;  // Device0 REG_DWORD; nullptr when installer-managed
    uint32_t requires_feature;      // 0 when independent
    const char* description;
};

// Names match video.cpp physical_init_requested, atom_rom_override,
// vbe_modeset_requested, radeon2d_requested, radeon2d_compute_requested and
// radeon_cursor_native.cpp hardware_cursor_requested (checked by the host test).
inline constexpr FeatureRule feature_rules[]{
    {feature_physical_asic_init, L"--asic", L"PhysicalAsicInit", 0,
     "PSP/SMU/GFX/MES/SDMA firmware load and ASIC bring-up"},
    {feature_atom_rom_override, L"--rom-override", L"AtomRomOverride", 0,
     "use only the administrator rx7900.rom as the VBIOS"},
    {feature_vbe_mode_set, L"--vbe", L"VbeModeSet", feature_physical_asic_init,
     "ask the VBIOS for a VBE linear mode after a legacy/CSM boot"},
    {feature_radeon_2d, L"--2d", L"Radeon2D", feature_physical_asic_init,
     "private SDMA 2D IOCTLs for radeondisp"},
    {feature_radeon_2d_compute, L"--2d-compute", L"Radeon2DCompute", feature_radeon_2d,
     "GFX11 compute raster engine for radeondisp"},
    {feature_hardware_cursor, L"--cursor", L"HardwareCursor", feature_physical_asic_init,
     "DCN3.2 hardware cursor on the adopted pipe"},
    {feature_opengl_icd, L"--opengl-icd", nullptr, 0,
     "radeonglicd.dll and the OpenGLDrivers\\RadeonGL registration"},
};
constexpr size_t feature_rule_count = sizeof(feature_rules) / sizeof(feature_rules[0]);

// Zero when the set is valid; otherwise the first feature whose prerequisite is
// missing (missing_requirement names it). Unknown bits return them unchanged.
uint32_t invalid_feature(uint32_t features, uint32_t& missing_requirement) noexcept;

// Installed components. inf_managed files are copied by SetupAPI from the INF;
// the ICD is copied by the installer because it is an explicit opt-in.
enum class Destination : uint8_t { system32, drivers };
enum class ImageKind : uint8_t { kernel_driver, kernel_dll, user_dll };

struct ComponentFile final {
    const wchar_t* filename;
    Destination destination;
    ImageKind image;
    bool inf_managed;
    uint32_t required_feature;  // 0 for always-installed components
};

inline constexpr ComponentFile component_files[]{
    {L"rx7900video.sys", Destination::drivers, ImageKind::kernel_driver, true, 0},
    {L"radeondisp.dll", Destination::system32, ImageKind::kernel_dll, true, 0},
    {L"radeonglicd.dll", Destination::system32, ImageKind::user_dll, false, feature_opengl_icd},
};
constexpr size_t component_count = sizeof(component_files) / sizeof(component_files[0]);
constexpr size_t icd_component = 2;

constexpr bool component_selected(size_t index, uint32_t features) noexcept {
    return index < component_count &&
           (component_files[index].required_feature == 0 ||
            (features & component_files[index].required_feature) != 0);
}

// PE identity. machine is IMAGE_FILE_MACHINE_I386 (0x14c) or AMD64 (0x8664).
constexpr uint16_t machine_i386 = 0x014c;
constexpr uint16_t machine_amd64 = 0x8664;
constexpr size_t pe_dos_header_bytes = 64;
constexpr size_t pe_headers_bytes = 24 + 70; // signature, COFF, optional header through Subsystem
constexpr uint32_t pe_maximum_header_offset = 1024u * 1024u;

enum class PeStatus { valid, not_pe, wrong_machine, wrong_subsystem, not_dll };

// The e_lfanew header offset from the first 64 bytes; false when the DOS header
// is malformed or the PE headers would end beyond file_bytes.
bool pe_header_offset(ByteView dos, uint64_t file_bytes, uint32_t& offset) noexcept;

// Checks signature, machine, PE32/PE32+ magic for the machine, DLL flag and
// subsystem (native for kernel images, GUI/console for the user-mode ICD).
PeStatus check_pe_headers(ByteView headers, uint16_t machine, ImageKind kind) noexcept;

// PCI identity of one PnP hardware or compatible ID list (REG_MULTI_SZ
// characters including terminators). Matches "PCI\VEN_1002&DEV_xxxx" prefixes
// whose device is a Navi31 GPU function (pci.hpp navi31_device_ids); the AMD
// USB and HDMI audio functions never match. most_specific receives the first
// list entry; revision the first "&REV_xx" value (0 when absent). Returns false
// for malformed lists or no Navi31 match.
bool navi31_identity(const wchar_t* ids, size_t characters, uint16_t& device_id,
                     uint8_t& revision, wchar_t (&most_specific)[instance_characters]) noexcept;

// True when the list contains the exact (case-insensitive) identifier.
bool multi_string_contains(const wchar_t* ids, size_t characters, const wchar_t* identifier) noexcept;

struct DeviceRecord final {
    wchar_t instance_id[instance_characters]{};
    wchar_t hardware_id[instance_characters]{};  // most specific hardware ID
    uint16_t device_id = 0;
    uint8_t revision = 0;
    wchar_t service[service_name_characters]{};  // empty when unbound
    bool started = false;                        // DN_STARTED
    uint32_t problem = 0;                        // CM_PROB_*, 0 when none
    wchar_t upper_filters[filter_characters]{};  // MULTI_SZ, characters below
    size_t upper_characters = 0;                 // 0 means the value is absent
    wchar_t lower_filters[filter_characters]{};
    size_t lower_characters = 0;
};

struct DeviceInventory final {
    DeviceRecord devices[device_capacity]{};
    size_t count = 0;
    size_t omitted = 0;             // Navi31 functions beyond capacity
    size_t absent = 0;              // recorded Navi31 instances with no attached device (never candidates)
    bool lightning_present = false; // the virtual 1234:11F1 device (never touched)
};

enum class Selection { none_present, unique, multiple };
Selection select_navi31(const DeviceInventory& inventory, size_t& index) noexcept;

// Binding policy. Another driver's binding is replaced only when the caller
// names exactly that service; an active (started) foreign binding is reported.
enum class Binding {
    bind_unbound,      // no functional driver
    already_ours,      // rx7900video is the function driver
    replace_named,     // foreign service named exactly by --force-replace
    refuse_active,     // foreign driver is started and was not named
    refuse_inactive,   // foreign driver is bound but stopped/failed, not named
    refuse_mismatch,   // --force-replace named a different service
};
Binding decide_binding(const DeviceRecord& device, const wchar_t* force_service) noexcept;

// Effective VideoPort parameter key for the rules above.
const wchar_t* effective_parameters_key(bool use_new_key_exists, bool profile_key_exists) noexcept;

// Service ImagePath forms SetupAPI and the SCM produce for our image.
bool service_image_matches(const wchar_t* image_path, const wchar_t* system_directory) noexcept;

// A bare oemNN.inf name as SetupAPI assigns to staged driver packages.
bool oem_inf_name(const wchar_t* name) noexcept;

// Firmware below %SystemRoot%\System32\drivers\rx7900 (driver_blob.cpp).
enum class FirmwareKind : uint8_t { ucode, psp_sos, psp_ta, atom_rom };
struct FirmwareEntry final {
    char name[48]{};
    char alternate[48]{};  // accepted fallback name (MES), empty when none
    FirmwareKind kind = FirmwareKind::ucode;
    uint8_t source = 0;    // hardware::UcodeSource for ucode entries
    bool required = false; // required for PhysicalAsicInit; optional where Linux allows
};
constexpr size_t firmware_entry_capacity = 14;
inline constexpr wchar_t firmware_directory_name[] = L"rx7900";
inline constexpr wchar_t backup_directory_name[] = L"backup";

// Fills the manifest in the miniport's load order for a PCI revision (0xCE
// selects gc_11_0_0_rlc_1.bin as Linux does). Returns the entry count.
size_t firmware_manifest(uint8_t pci_revision, FirmwareEntry (&entries)[firmware_entry_capacity]) noexcept;

enum class FirmwareVerdict { valid, too_small, too_large, malformed, wrong_device };
// Maximum readable size for the entry kind (32 MiB containers, 1 MiB ROM).
uint32_t firmware_maximum_bytes(const FirmwareEntry& entry) noexcept;
// The same structural validators the miniport uses: UcodeCatalog::add for GC,
// RLC, MES, IMU, SDMA, SMU and DMCUB; validate_firmware for the PSP SOS;
// parse_ta_descriptors for the TA; validate_atom_image for rx7900.rom against
// the AMD device ID (any Navi31 ID when device_id is 0). Not authentication.
FirmwareVerdict validate_firmware_bytes(const FirmwareEntry& entry, ByteView bytes,
                                        uint16_t device_id) noexcept;
const char* firmware_verdict_name(FirmwareVerdict verdict) noexcept;

// Read-only win32k capability detection. The optional patches compile named C++
// helpers into win32k; their names remain in the COFF symbol table of ReactOS
// builds. A missing symbol-table probe makes every capability unknown.
enum class PatchPresence { present, absent, partial, unknown };
struct Win32kCapability final {
    const char* name;
    const char* patch;
    const char* markers[2];
    const char* effect;
};
inline constexpr Win32kCapability win32k_capabilities[]{
    {"stretch-guard", "reactos-stretch-guard.patch (RX7900_FRAMEBUFFER_STRETCH_GUARD=ON)",
     {"Rx7900StretchGuardPrepare", "Rx7900LinePrepare"},
     "solid cosmetic lines and DrvStretchBltROP reach radeondisp"},
    {"radeon-stroke-bridge", "reactos-radeon-bridge.patch (RX7900_RADEON_BRIDGE=ON)",
     {"Rx7900RadeonStrokePrepare", "Rx7900RadeonStrokeDraw"},
     "styled cosmetic segments reach radeondisp DrvStrokePath"},
    {"radeon-text-bridge", "reactos-radeon-text-bridge.patch (RX7900_RADEON_TEXT_BRIDGE=ON)",
     {"Rx7900RadeonTextBegin", "Rx7900RadeonTextGlyph"},
     "FreeType coverage glyphs reach radeondisp DrvNotify"},
    {"draw-escape", "reactos-draw-escape.patch (RX7900_FRAMEBUFFER_DRAW_ESCAPE=ON)",
     {"LightningDrawEscape", nullptr},
     "RadeonGL presents through clipped DrvDrawEscape"},
};
constexpr size_t win32k_capability_count = sizeof(win32k_capabilities) / sizeof(win32k_capabilities[0]);
inline constexpr char win32k_symbol_probe[] = "IntEngLineTo";

// Streams a file once and records which fixed markers occur anywhere in it,
// including across chunk boundaries (one Knuth-Morris-Pratt automaton per
// marker, so no bytes are retained between chunks). Markers are the capability
// names above plus the symbol-table probe.
class Win32kScanner final {
public:
    Win32kScanner() noexcept;
    void feed(ByteView chunk) noexcept;
    bool symbols_present() const noexcept;
    PatchPresence presence(size_t capability) const noexcept;

private:
    static constexpr size_t marker_capacity = 2 * win32k_capability_count + 1;
    static constexpr size_t marker_length_capacity = 32;
    const char* markers_[marker_capacity]{};
    size_t lengths_[marker_capacity]{};
    uint8_t failure_[marker_capacity][marker_length_capacity]{};
    size_t matched_[marker_capacity]{};
    bool found_[marker_capacity]{};
    size_t marker_count_ = 0;
};

// Journal phases (HKLM\SOFTWARE\RX7900\RadeonInstaller State).
enum class JournalState : uint32_t {
    none = 0,
    installing = 1,       // binding started; recovery rolls back unless bound to us
    installed = 2,
    replace_staging = 3,  // upgrade/rollback copies staged; live files untouched
    replace_commit = 4,   // live files being swapped; recovery completes or restores
    removing = 5,
    filters_pending = 6,  // removed; preserved device filters await re-detection
};
const char* journal_state_name(JournalState state) noexcept;
bool journal_state_transitional(JournalState state) noexcept;

enum class ReplaceKind : uint32_t { upgrade = 1, rollback = 2 };

// One file of a replace transaction, observed after an interruption. Hashes are
// valid only with their present flag.
struct ReplaceObservation final {
    bool live = false;
    Sha256Digest live_hash{};
    bool staged = false;        // <live>.radeonctl-new
    Sha256Digest staged_hash{};
    bool retired = false;       // <live>.radeonctl-old
    Sha256Digest retired_hash{};
    bool backup = false;        // backup\<name>
    Sha256Digest backup_hash{};
    bool backup_staged = false; // backup\<name>.radeonctl-new
    Sha256Digest backup_staged_hash{};
};

enum class ReplaceRecovery {
    complete,          // live already equals the target
    finish_forward,    // a verified staged target can be moved into place
    restore_previous,  // forward is impossible; a verified prior copy exists
    unrecoverable,     // neither target nor prior bytes remain
};
ReplaceRecovery decide_replace_recovery(const ReplaceObservation& observation,
                                        const Sha256Digest& target,
                                        const Sha256Digest& prior) noexcept;

// Which observed copy holds the prior bytes: 1 live, 2 retired, 3 backup staged,
// 4 backup, 0 none. The live copy is preferred so restoration is a no-op.
int prior_source(const ReplaceObservation& observation, const Sha256Digest& prior) noexcept;

// Command line.
enum class Command { help, status, install, upgrade, rollback, remove, recover, firmware_copy };
struct CommandLine final {
    Command command = Command::help;
    uint32_t features = 0;
    bool dry_run = false;
    bool force_replace = false;
    wchar_t force_service[service_name_characters]{};
    bool replace_firmware = false;
    wchar_t source_directory[path_characters]{};
    const wchar_t* error = nullptr;  // offending argument when parsing fails
};
// Returns false for unknown commands, unknown or repeated options, options not
// valid for the command and overlong arguments. error names the argument.
bool parse_command_line(int argc, const wchar_t* const* argv, CommandLine& output) noexcept;

// Bounded wide-text helpers shared by the engine and its bindings.
size_t text_length(const wchar_t* text, size_t capacity) noexcept;  // capacity when unterminated
bool text_equal_insensitive(const wchar_t* left, const wchar_t* right) noexcept;  // ASCII folding
template<size_t Capacity>
bool text_copy(wchar_t (&output)[Capacity], const wchar_t* text) noexcept {
    output[0] = 0;
    if (text == nullptr) {
        return false;
    }
    for (size_t index = 0; index < Capacity; ++index) {
        output[index] = text[index];
        if (text[index] == 0) {
            return true;
        }
    }
    output[0] = 0;
    return false;
}
template<size_t Capacity>
bool text_append(wchar_t (&output)[Capacity], const wchar_t* text) noexcept {
    const size_t start = text_length(output, Capacity);
    if (start == Capacity || text == nullptr) {
        return false;
    }
    for (size_t index = 0; start + index < Capacity; ++index) {
        output[start + index] = text[index];
        if (text[index] == 0) {
            return true;
        }
    }
    output[start] = 0;
    return false;
}
template<size_t Capacity>
bool text_append_ascii(wchar_t (&output)[Capacity], const char* text) noexcept {
    const size_t start = text_length(output, Capacity);
    if (start == Capacity || text == nullptr) {
        return false;
    }
    for (size_t index = 0; start + index < Capacity; ++index) {
        const auto character = static_cast<unsigned char>(text[index]);
        if (character > 0x7f) {
            output[start] = 0;
            return false;
        }
        output[start + index] = static_cast<wchar_t>(character);
        if (character == 0) {
            return true;
        }
    }
    output[start] = 0;
    return false;
}
// output = directory + "\" + name; false (output empty) when it does not fit.
template<size_t Capacity>
bool path_join(wchar_t (&output)[Capacity], const wchar_t* directory, const wchar_t* name) noexcept {
    if (!text_copy(output, directory) || !text_append(output, L"\\") || !text_append(output, name)) {
        output[0] = 0;
        return false;
    }
    return true;
}

// Merges every entry of original that current lacks into output (current order
// preserved, missing entries appended in original order). Same list rules as
// src/common/multisz.hpp append_filter_name, which needs compile-time service
// names: exact terminated lists, no hidden entries after the terminator, ASCII
// case-insensitive comparison, Unicode names never rewritten. Counts include
// terminators; zero means absent. Returns false for malformed lists or when the
// merge would not fit; output is then empty.
bool merge_missing_filters(const wchar_t* original, size_t original_characters,
                           const wchar_t* current, size_t current_characters,
                           wchar_t (&output)[filter_characters], size_t& output_characters,
                           size_t& restored) noexcept;

} // namespace rx7900::tools::radeon

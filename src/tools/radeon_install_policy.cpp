/*
Module: rx7900.tools.radeon_install_policy
Purpose: Implement the pure Radeon installer naming, safety and decision rules.
Created: 2026-10-24
Architecture: No OS calls. Reuses pci.hpp identities, ucode/firmware/ATOM
validators and the radeon_opengl_info contract through the policy header.
*/
#include "radeon_install_policy.hpp"

#include "atom_bios.hpp"
#include "firmware.hpp"
#include "pci.hpp"
#include "ucode.hpp"

namespace rx7900::tools::radeon {
namespace {

constexpr wchar_t fold(wchar_t value) noexcept {
    return value >= L'A' && value <= L'Z' ? static_cast<wchar_t>(value + (L'a' - L'A')) : value;
}

// Compares count characters of left with a NUL-terminated ASCII prefix.
bool starts_with_insensitive(const wchar_t* text, size_t available, const wchar_t* prefix,
                             size_t& prefix_length) noexcept {
    prefix_length = 0;

    while (prefix[prefix_length] != 0) {
        if (prefix_length >= available || fold(text[prefix_length]) != fold(prefix[prefix_length])) {
            return false;
        }
        ++prefix_length;
    }

    return true;
}

bool hex_digit(wchar_t character, uint32_t& value) noexcept {
    if (character >= L'0' && character <= L'9') {
        value = static_cast<uint32_t>(character - L'0');
        return true;
    }
    if (character >= L'a' && character <= L'f') {
        value = static_cast<uint32_t>(character - L'a' + 10);
        return true;
    }
    if (character >= L'A' && character <= L'F') {
        value = static_cast<uint32_t>(character - L'A' + 10);
        return true;
    }
    return false;
}

bool parse_hex(const wchar_t* text, size_t digits, uint32_t& value) noexcept {
    uint32_t result = 0;

    for (size_t index = 0; index < digits; ++index) {
        uint32_t nibble = 0;

        if (!hex_digit(text[index], nibble)) {
            return false;
        }
        result = (result << 4) | nibble;
    }

    value = result;
    return true;
}

// Visits each entry of a validated MULTI_SZ. Returns false when the list is
// malformed: unterminated, missing its final empty string or hiding entries.
template<typename Visitor>
bool visit_multi_string(const wchar_t* ids, size_t characters, Visitor&& visit) noexcept {
    if (ids == nullptr || characters < 2) {
        return false;
    }

    size_t position = 0;

    while (position < characters) {
        if (ids[position] == 0) {
            for (size_t tail = position; tail < characters; ++tail) {
                if (ids[tail] != 0) {
                    return false;
                }
            }
            return true;
        }

        const size_t first = position;

        while (position < characters && ids[position] != 0) {
            ++position;
        }
        if (position == characters) {
            return false;
        }

        visit(ids + first, position - first);
        ++position;
    }

    return false;
}

bool entry_equal(const wchar_t* entry, size_t length, const wchar_t* other, size_t other_length) noexcept {
    if (length != other_length) {
        return false;
    }
    for (size_t index = 0; index < length; ++index) {
        if (fold(entry[index]) != fold(other[index])) {
            return false;
        }
    }
    return true;
}

bool list_has_entry(const wchar_t* ids, size_t characters, const wchar_t* entry, size_t length) noexcept {
    bool found = false;

    if (characters == 0) {
        return false;
    }

    const bool valid = visit_multi_string(ids, characters, [&](const wchar_t* item, size_t item_length) {
        found = found || entry_equal(item, item_length, entry, length);
    });
    return valid && found;
}

void set_name(char (&output)[48], const char* name) noexcept {
    output[0] = 0;

    if (name == nullptr) {
        return;
    }
    for (size_t index = 0; index < sizeof(output); ++index) {
        output[index] = name[index];
        if (name[index] == 0) {
            return;
        }
    }
    output[0] = 0;
}

} // namespace

size_t text_length(const wchar_t* text, size_t capacity) noexcept {
    if (text == nullptr) {
        return 0;
    }
    for (size_t index = 0; index < capacity; ++index) {
        if (text[index] == 0) {
            return index;
        }
    }
    return capacity;
}

bool text_equal_insensitive(const wchar_t* left, const wchar_t* right) noexcept {
    if (left == nullptr || right == nullptr) {
        return false;
    }

    for (size_t index = 0;; ++index) {
        if (fold(left[index]) != fold(right[index])) {
            return false;
        }
        if (left[index] == 0) {
            return true;
        }
    }
}

uint32_t invalid_feature(uint32_t features, uint32_t& missing_requirement) noexcept {
    missing_requirement = 0;

    if ((features & ~known_features) != 0) {
        return features & ~known_features;
    }

    for (const auto& rule : feature_rules) {
        if ((features & rule.feature) != 0 && rule.requires_feature != 0 &&
            (features & rule.requires_feature) == 0) {
            missing_requirement = rule.requires_feature;
            return rule.feature;
        }
    }

    return 0;
}

bool pe_header_offset(ByteView dos, uint64_t file_bytes, uint32_t& offset) noexcept {
    uint16_t signature = 0;
    uint32_t candidate = 0;

    if (!dos.read_u16(0, signature) || signature != 0x5a4d || !dos.read_u32(0x3c, candidate) ||
        candidate < pe_dos_header_bytes || candidate > pe_maximum_header_offset ||
        file_bytes < pe_headers_bytes || static_cast<uint64_t>(candidate) > file_bytes - pe_headers_bytes) {
        return false;
    }

    offset = candidate;
    return true;
}

PeStatus check_pe_headers(ByteView headers, uint16_t machine, ImageKind kind) noexcept {
    uint32_t signature = 0;
    uint16_t file_machine = 0;
    uint16_t optional_bytes = 0;
    uint16_t characteristics = 0;
    uint16_t magic = 0;
    uint16_t subsystem = 0;

    if (!headers.read_u32(0, signature) || signature != 0x00004550u ||
        !headers.read_u16(4, file_machine) || !headers.read_u16(20, optional_bytes) ||
        !headers.read_u16(22, characteristics) || !headers.read_u16(24, magic) ||
        !headers.read_u16(24 + 68, subsystem) || optional_bytes < 70) {
        return PeStatus::not_pe;
    }

    const uint16_t expected_magic = machine == machine_amd64 ? 0x20b : 0x10b;

    if (file_machine != machine || magic != expected_magic) {
        return PeStatus::wrong_machine;
    }

    constexpr uint16_t image_file_dll = 0x2000;
    constexpr uint16_t subsystem_native = 1;
    constexpr uint16_t subsystem_gui = 2;
    constexpr uint16_t subsystem_console = 3;
    const bool dll = (characteristics & image_file_dll) != 0;

    switch (kind) {
    case ImageKind::kernel_driver:
        return subsystem == subsystem_native ? PeStatus::valid : PeStatus::wrong_subsystem;
    case ImageKind::kernel_dll:
        if (subsystem != subsystem_native) {
            return PeStatus::wrong_subsystem;
        }
        return dll ? PeStatus::valid : PeStatus::not_dll;
    case ImageKind::user_dll:
        if (subsystem != subsystem_gui && subsystem != subsystem_console) {
            return PeStatus::wrong_subsystem;
        }
        return dll ? PeStatus::valid : PeStatus::not_dll;
    }

    return PeStatus::not_pe;
}

bool multi_string_contains(const wchar_t* ids, size_t characters, const wchar_t* identifier) noexcept {
    return identifier != nullptr &&
           list_has_entry(ids, characters, identifier, text_length(identifier, filter_characters));
}

bool navi31_identity(const wchar_t* ids, size_t characters, uint16_t& device_id,
                     uint8_t& revision, wchar_t (&most_specific)[instance_characters]) noexcept {
    bool first = true;
    bool matched = false;
    bool too_long = false;
    uint16_t found_device = 0;
    uint8_t found_revision = 0;
    bool revision_seen = false;
    wchar_t specific[instance_characters]{};

    const bool valid = visit_multi_string(ids, characters, [&](const wchar_t* entry, size_t length) {
        if (first) {
            first = false;
            if (length >= instance_characters) {
                too_long = true;
                return;
            }
            for (size_t index = 0; index < length; ++index) {
                specific[index] = entry[index];
            }
            specific[length] = 0;
        }

        size_t prefix = 0;
        uint32_t vendor = 0;
        uint32_t device = 0;

        if (!starts_with_insensitive(entry, length, L"PCI\\VEN_", prefix) || length < prefix + 4 ||
            !parse_hex(entry + prefix, 4, vendor) || vendor != amd_vendor_id) {
            return;
        }

        size_t device_prefix = 0;

        if (!starts_with_insensitive(entry + prefix + 4, length - prefix - 4, L"&DEV_", device_prefix) ||
            length < prefix + 4 + device_prefix + 4 ||
            !parse_hex(entry + prefix + 4 + device_prefix, 4, device)) {
            return;
        }

        const size_t end = prefix + 4 + device_prefix + 4;

        // The four hexadecimal digits must end the token or be followed by '&'.
        if (end != length && entry[end] != L'&') {
            return;
        }
        if (!is_navi31_device(static_cast<uint16_t>(device))) {
            return;
        }
        if (matched && found_device != device) {
            too_long = true; // conflicting identities are malformed
            return;
        }

        matched = true;
        found_device = static_cast<uint16_t>(device);

        for (size_t position = end; position + 7 <= length && !revision_seen; ++position) {
            size_t revision_prefix = 0;
            uint32_t value = 0;

            if (starts_with_insensitive(entry + position, length - position, L"&REV_", revision_prefix) &&
                position + revision_prefix + 2 <= length &&
                parse_hex(entry + position + revision_prefix, 2, value)) {
                found_revision = static_cast<uint8_t>(value);
                revision_seen = true;
            }
        }
    });

    if (!valid || !matched || too_long) {
        return false;
    }

    device_id = found_device;
    revision = found_revision;
    return text_copy(most_specific, specific);
}

Selection select_navi31(const DeviceInventory& inventory, size_t& index) noexcept {
    if (inventory.count == 0 && inventory.omitted == 0) {
        return Selection::none_present;
    }
    if (inventory.count != 1 || inventory.omitted != 0) {
        return Selection::multiple;
    }

    index = 0;
    return Selection::unique;
}

Binding decide_binding(const DeviceRecord& device, const wchar_t* force_service) noexcept {
    if (device.service[0] == 0) {
        return Binding::bind_unbound;
    }
    if (text_equal_insensitive(device.service, service_name)) {
        return Binding::already_ours;
    }
    if (force_service != nullptr && force_service[0] != 0) {
        return text_equal_insensitive(force_service, device.service) ? Binding::replace_named
                                                                     : Binding::refuse_mismatch;
    }

    return device.started ? Binding::refuse_active : Binding::refuse_inactive;
}

const wchar_t* effective_parameters_key(bool use_new_key_exists, bool profile_key_exists) noexcept {
    return use_new_key_exists && profile_key_exists ? profile_device0_key : device0_key;
}

bool service_image_matches(const wchar_t* image_path, const wchar_t* system_directory) noexcept {
    constexpr const wchar_t* fixed[]{
        L"System32\\drivers\\rx7900video.sys",
        L"\\SystemRoot\\System32\\drivers\\rx7900video.sys",
        L"%SystemRoot%\\System32\\drivers\\rx7900video.sys",
        L"\\??\\%SystemRoot%\\System32\\drivers\\rx7900video.sys",
    };

    if (image_path == nullptr || image_path[0] == 0) {
        return false;
    }
    for (const auto* form : fixed) {
        if (text_equal_insensitive(image_path, form)) {
            return true;
        }
    }

    wchar_t absolute[path_characters]{};
    wchar_t prefixed[path_characters]{};

    if (system_directory != nullptr && text_copy(absolute, system_directory) &&
        text_append(absolute, L"\\drivers\\rx7900video.sys")) {
        if (text_equal_insensitive(image_path, absolute)) {
            return true;
        }
        if (text_copy(prefixed, L"\\??\\") && text_append(prefixed, absolute) &&
            text_equal_insensitive(image_path, prefixed)) {
            return true;
        }
    }

    return false;
}

bool oem_inf_name(const wchar_t* name) noexcept {
    const size_t length = text_length(name, path_characters);

    if (length < 8 || length >= path_characters || fold(name[0]) != L'o' || fold(name[1]) != L'e' ||
        fold(name[2]) != L'm' || !text_equal_insensitive(name + length - 4, L".inf")) {
        return false;
    }
    for (size_t index = 3; index + 4 < length; ++index) {
        if (name[index] < L'0' || name[index] > L'9') {
            return false;
        }
    }
    return true;
}

size_t firmware_manifest(uint8_t pci_revision, FirmwareEntry (&entries)[firmware_entry_capacity]) noexcept {
    using hardware::UcodeSource;
    // The miniport's required order (asic_bringup.cpp), then its optional files.
    constexpr UcodeSource required[]{UcodeSource::pfp, UcodeSource::me, UcodeSource::mec,
        UcodeSource::rlc, UcodeSource::mes, UcodeSource::mes_kiq, UcodeSource::imu,
        UcodeSource::sdma, UcodeSource::smu};
    size_t count = 0;

    for (auto& entry : entries) {
        entry = FirmwareEntry{};
    }
    for (const auto source : required) {
        auto& entry = entries[count++];
        set_name(entry.name, hardware::ucode_filename(source, pci_revision, false));
        if (source == UcodeSource::mes) {
            set_name(entry.alternate, hardware::ucode_filename(source, pci_revision, true));
        }
        entry.kind = FirmwareKind::ucode;
        entry.source = static_cast<uint8_t>(source);
        entry.required = true;
    }

    auto& sos = entries[count++];
    set_name(sos.name, hardware::firmware_filename(hardware::FirmwareKind::psp_13_0));
    sos.kind = FirmwareKind::psp_sos;
    sos.required = true;

    auto& dmcub = entries[count++];
    set_name(dmcub.name, hardware::ucode_filename(UcodeSource::dmcub, pci_revision, false));
    dmcub.kind = FirmwareKind::ucode;
    dmcub.source = static_cast<uint8_t>(UcodeSource::dmcub);

    auto& ta = entries[count++];
    set_name(ta.name, hardware::psp_ta_filename);
    ta.kind = FirmwareKind::psp_ta;

    // The miniport reads the card VBIOS first; the file is the last resort or
    // the only source with AtomRomOverride=1 (driver_blob load_atom_rom).
    auto& rom = entries[count++];
    set_name(rom.name, "rx7900.rom");
    rom.kind = FirmwareKind::atom_rom;
    return count;
}

uint32_t firmware_maximum_bytes(const FirmwareEntry& entry) noexcept {
    return entry.kind == FirmwareKind::atom_rom ? maximum_atom_file_bytes : hardware::maximum_firmware_bytes;
}

FirmwareVerdict validate_firmware_bytes(const FirmwareEntry& entry, ByteView bytes,
                                        uint16_t device_id) noexcept {
    // Native minimums: 32-byte containers, 0x4a-byte ROM (driver_blob.cpp).
    const size_t minimum = entry.kind == FirmwareKind::atom_rom ? 0x4a : 32;

    if (bytes.size() < minimum) {
        return FirmwareVerdict::too_small;
    }
    if (bytes.size() > firmware_maximum_bytes(entry)) {
        return FirmwareVerdict::too_large;
    }

    switch (entry.kind) {
    case FirmwareKind::ucode: {
        hardware::UcodeCatalog catalog;
        return catalog.add(static_cast<hardware::UcodeSource>(entry.source), bytes) ==
                       hardware::UcodeStatus::success ? FirmwareVerdict::valid : FirmwareVerdict::malformed;
    }
    case FirmwareKind::psp_sos: {
        hardware::FirmwareImage image{};
        return hardware::validate_firmware(bytes, hardware::FirmwareKind::psp_13_0, image) ==
                       hardware::FirmwareStatus::success ? FirmwareVerdict::valid : FirmwareVerdict::malformed;
    }
    case FirmwareKind::psp_ta: {
        hardware::TaDescriptors descriptors{};
        return hardware::parse_ta_descriptors(bytes, descriptors) == hardware::UcodeStatus::success ?
                   FirmwareVerdict::valid : FirmwareVerdict::malformed;
    }
    case FirmwareKind::atom_rom: {
        AtomImage image{};
        if (device_id != 0) {
            return validate_atom_image(bytes, amd_vendor_id, device_id, image) == ParseStatus::success ?
                       FirmwareVerdict::valid : FirmwareVerdict::wrong_device;
        }
        for (const uint16_t candidate : navi31_device_ids) {
            if (validate_atom_image(bytes, amd_vendor_id, candidate, image) == ParseStatus::success) {
                return FirmwareVerdict::valid;
            }
        }
        return FirmwareVerdict::wrong_device;
    }
    }

    return FirmwareVerdict::malformed;
}

const char* firmware_verdict_name(FirmwareVerdict verdict) noexcept {
    switch (verdict) {
    case FirmwareVerdict::valid: return "valid container";
    case FirmwareVerdict::too_small: return "too small";
    case FirmwareVerdict::too_large: return "too large";
    case FirmwareVerdict::malformed: return "malformed or wrong IP version";
    case FirmwareVerdict::wrong_device: return "not an ATOM ROM for this Navi31 device";
    }
    return "unknown";
}

Win32kScanner::Win32kScanner() noexcept {
    for (const auto& capability : win32k_capabilities) {
        for (const char* marker : capability.markers) {
            if (marker != nullptr) {
                markers_[marker_count_++] = marker;
            }
        }
    }
    markers_[marker_count_++] = win32k_symbol_probe;

    for (size_t index = 0; index < marker_count_; ++index) {
        size_t length = 0;
        while (length < marker_length_capacity && markers_[index][length] != 0) {
            ++length;
        }
        // Fixed markers are shorter than the capacity; a longer one never matches.
        lengths_[index] = markers_[index][length] == 0 ? length : 0;

        // Classic KMP failure function: failure_[i][k] is the longest proper
        // border of the first k+1 marker bytes.
        size_t border = 0;
        for (size_t position = 1; position < lengths_[index]; ++position) {
            while (border > 0 && markers_[index][position] != markers_[index][border]) {
                border = failure_[index][border - 1];
            }
            if (markers_[index][position] == markers_[index][border]) {
                ++border;
            }
            failure_[index][position] = static_cast<uint8_t>(border);
        }
    }
}

void Win32kScanner::feed(ByteView chunk) noexcept {
    for (size_t offset = 0; offset < chunk.size(); ++offset) {
        uint8_t value = 0;
        chunk.read_u8(offset, value);

        for (size_t marker = 0; marker < marker_count_; ++marker) {
            const size_t length = lengths_[marker];

            if (found_[marker] || length == 0) {
                continue;
            }

            size_t matched = matched_[marker];

            while (matched > 0 && static_cast<uint8_t>(markers_[marker][matched]) != value) {
                matched = failure_[marker][matched - 1];
            }
            if (static_cast<uint8_t>(markers_[marker][matched]) == value) {
                ++matched;
            }
            if (matched == length) {
                found_[marker] = true;
                matched = 0;
            }
            matched_[marker] = matched;
        }
    }
}

bool Win32kScanner::symbols_present() const noexcept {
    return found_[marker_count_ - 1];
}

PatchPresence Win32kScanner::presence(size_t capability) const noexcept {
    if (capability >= win32k_capability_count) {
        return PatchPresence::unknown;
    }

    size_t marker = 0;

    for (size_t index = 0; index < capability; ++index) {
        for (const char* name : win32k_capabilities[index].markers) {
            marker += name != nullptr ? 1 : 0;
        }
    }

    size_t expected = 0;
    size_t seen = 0;

    for (const char* name : win32k_capabilities[capability].markers) {
        if (name != nullptr) {
            ++expected;
            seen += found_[marker++] ? 1 : 0;
        }
    }

    if (seen == expected) {
        return PatchPresence::present;
    }
    if (!symbols_present()) {
        return PatchPresence::unknown;
    }
    return seen == 0 ? PatchPresence::absent : PatchPresence::partial;
}

const char* journal_state_name(JournalState state) noexcept {
    switch (state) {
    case JournalState::none: return "not installed";
    case JournalState::installing: return "interrupted install";
    case JournalState::installed: return "installed";
    case JournalState::replace_staging: return "interrupted file replacement (staging)";
    case JournalState::replace_commit: return "interrupted file replacement (commit)";
    case JournalState::removing: return "interrupted removal";
    case JournalState::filters_pending: return "removed; device filters await re-detection";
    }
    return "unknown";
}

bool journal_state_transitional(JournalState state) noexcept {
    return state == JournalState::installing || state == JournalState::replace_staging ||
           state == JournalState::replace_commit || state == JournalState::removing;
}

int prior_source(const ReplaceObservation& observation, const Sha256Digest& prior) noexcept {
    if (observation.live && observation.live_hash == prior) {
        return 1;
    }
    if (observation.retired && observation.retired_hash == prior) {
        return 2;
    }
    if (observation.backup_staged && observation.backup_staged_hash == prior) {
        return 3;
    }
    if (observation.backup && observation.backup_hash == prior) {
        return 4;
    }
    return 0;
}

ReplaceRecovery decide_replace_recovery(const ReplaceObservation& observation,
                                        const Sha256Digest& target,
                                        const Sha256Digest& prior) noexcept {
    if (observation.live && observation.live_hash == target) {
        return ReplaceRecovery::complete;
    }
    if (observation.staged && observation.staged_hash == target) {
        return ReplaceRecovery::finish_forward;
    }
    return prior_source(observation, prior) != 0 ? ReplaceRecovery::restore_previous
                                                 : ReplaceRecovery::unrecoverable;
}

bool parse_command_line(int argc, const wchar_t* const* argv, CommandLine& output) noexcept {
    output = CommandLine{};

    if (argc < 2 || argv == nullptr || argv[1] == nullptr) {
        output.command = Command::help;
        return argc < 2;
    }

    struct Name final {
        const wchar_t* text;
        Command command;
    };
    constexpr Name commands[]{
        {L"status", Command::status}, {L"install", Command::install},
        {L"upgrade", Command::upgrade}, {L"rollback", Command::rollback},
        {L"remove", Command::remove}, {L"recover", Command::recover},
        {L"firmware-copy", Command::firmware_copy}, {L"help", Command::help},
        {L"--help", Command::help}, {L"/?", Command::help},
    };
    bool known = false;

    for (const auto& name : commands) {
        if (text_equal_insensitive(argv[1], name.text)) {
            output.command = name.command;
            known = true;
        }
    }
    if (!known) {
        output.error = argv[1];
        return false;
    }

    int index = 2;

    if (output.command == Command::firmware_copy) {
        if (argc < 3 || argv[2] == nullptr || argv[2][0] == L'-' ||
            !text_copy(output.source_directory, argv[2])) {
            output.error = argc < 3 ? argv[1] : argv[2];
            return false;
        }
        index = 3;
    }

    for (; index < argc; ++index) {
        const wchar_t* argument = argv[index];
        bool handled = false;

        if (argument == nullptr) {
            return false;
        }
        if (output.command == Command::install) {
            for (const auto& rule : feature_rules) {
                if (text_equal_insensitive(argument, rule.option)) {
                    if ((output.features & rule.feature) != 0) {
                        output.error = argument;
                        return false;
                    }
                    output.features |= rule.feature;
                    handled = true;
                }
            }
            if (!handled && text_equal_insensitive(argument, L"--dry-run") && !output.dry_run) {
                output.dry_run = true;
                handled = true;
            }
            if (!handled && text_equal_insensitive(argument, L"--force-replace") && !output.force_replace) {
                if (index + 1 >= argc || argv[index + 1] == nullptr || argv[index + 1][0] == 0 ||
                    argv[index + 1][0] == L'-' || !text_copy(output.force_service, argv[index + 1])) {
                    output.error = argument;
                    return false;
                }
                output.force_replace = true;
                ++index;
                handled = true;
            }
        }
        if (!handled && output.command == Command::firmware_copy &&
            text_equal_insensitive(argument, L"--replace") && !output.replace_firmware) {
            output.replace_firmware = true;
            handled = true;
        }
        if (!handled) {
            output.error = argument;
            return false;
        }
    }

    return true;
}

bool merge_missing_filters(const wchar_t* original, size_t original_characters,
                           const wchar_t* current, size_t current_characters,
                           wchar_t (&output)[filter_characters], size_t& output_characters,
                           size_t& restored) noexcept {
    output[0] = 0;
    output_characters = 0;
    restored = 0;

    const auto valid_or_absent = [](const wchar_t* list, size_t characters) noexcept {
        return characters == 0 ||
               visit_multi_string(list, characters, [](const wchar_t*, size_t) {});
    };

    if (!valid_or_absent(original, original_characters) || !valid_or_absent(current, current_characters)) {
        return false;
    }

    size_t position = 0;
    bool fits = true;
    const auto append = [&](const wchar_t* entry, size_t length) noexcept {
        if (!fits || position + length + 2 > filter_characters) {
            fits = false;
            return;
        }
        for (size_t index = 0; index < length; ++index) {
            output[position++] = entry[index];
        }
        output[position++] = 0;
    };

    if (current_characters != 0) {
        visit_multi_string(current, current_characters, append);
    }
    if (original_characters != 0) {
        visit_multi_string(original, original_characters, [&](const wchar_t* entry, size_t length) {
            const bool already = current_characters != 0 &&
                                 list_has_entry(current, current_characters, entry, length);
            // An original list may repeat a name; restore it once.
            bool appended = false;
            for (size_t scan = 0; scan < position && !appended;) {
                size_t end = scan;
                while (end < position && output[end] != 0) {
                    ++end;
                }
                appended = entry_equal(output + scan, end - scan, entry, length);
                scan = end + 1;
            }
            if (!already && !appended) {
                append(entry, length);
                restored += fits ? 1 : 0;
            }
        });
    }
    if (!fits) {
        output[0] = 0;
        restored = 0;
        return false;
    }
    if (position == 0) {
        // Both lists absent or empty: nothing to write.
        output_characters = current_characters == 0 && original_characters == 0 ? 0 : 2;
        output[0] = 0;
        output[1] = 0;
        return true;
    }

    output[position++] = 0;
    output_characters = position;
    return true;
}

} // namespace rx7900::tools::radeon

/*
Module: rx7900.tools.radeon_installer
Purpose: Execute the physical Radeon install, status, upgrade, rollback, removal
and recovery commands through the SystemOps seam.
Created: 2026-10-24
Architecture: Platform-neutral engine; every decision is delegated to
radeon_install_policy and every side effect goes through SystemOps. A registry
journal (HKLM\SOFTWARE\RX7900\RadeonInstaller) records each transaction before
its first side effect so an interrupted command can be completed or undone.
*/
#include "radeon_installer.hpp"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#include "radeon_opengl_info.hpp"

namespace rx7900::tools::radeon {
namespace {

using Path = wchar_t[path_characters];
namespace opengl = rx7900::reactos::radeon_opengl;

constexpr wchar_t staged_suffix[] = L".radeonctl-new";
constexpr wchar_t retired_suffix[] = L".radeonctl-old";
constexpr uint32_t hash_chunk_bytes = 16 * 1024;

// Bounded heap buffer for whole-file firmware validation (at most 32 MiB). The
// owner frees it on every exit; views of it never outlive this object.
class HeapBuffer final {
public:
    HeapBuffer() noexcept = default;
    HeapBuffer(const HeapBuffer&) = delete;
    HeapBuffer& operator=(const HeapBuffer&) = delete;
    ~HeapBuffer() noexcept { free(data_); }
    bool allocate(size_t bytes) noexcept {
        free(data_);
        data_ = static_cast<uint8_t*>(malloc(bytes == 0 ? 1 : bytes));
        size_ = data_ != nullptr ? bytes : 0;
        return data_ != nullptr;
    }
    uint8_t* data() noexcept { return data_; }
    size_t size() const noexcept { return size_; }
    ByteView view() const noexcept { return ByteView(data_, size_); }

private:
    uint8_t* data_ = nullptr;
    size_t size_ = 0;
};

struct FileJournal final {
    bool installed = false;
    Sha256Digest installed_hash{};
    bool backup = false;
    Sha256Digest backup_hash{};
    bool target = false;
    Sha256Digest target_hash{};
    bool prior = false;
    Sha256Digest prior_hash{};
    // A replacement queued for the next restart: <live>.radeonctl-new holds
    // pending_hash and a MOVEFILE_DELAY_UNTIL_REBOOT rename onto live exists.
    // installed_hash already names that generation; live still holds the prior.
    bool pending = false;
    Sha256Digest pending_hash{};
};

struct Journal final {
    JournalState state = JournalState::none;
    ReplaceKind operation = ReplaceKind::upgrade;
    uint32_t features = 0;
    bool restart_pending = false;
    // Boot during which the oldest outstanding replacement was queued. A queued
    // copy still present after a later boot means its rename was lost.
    bool queued_boot_known = false;
    uint64_t queued_boot = 0;
    wchar_t instance[instance_characters]{};
    wchar_t hardware_id[instance_characters]{};
    wchar_t previous_service[service_name_characters]{};
    wchar_t inf_name[path_characters]{};
    bool snapshot = false;  // filter snapshot recorded
    wchar_t upper[filter_characters]{};
    size_t upper_characters = 0;
    wchar_t lower[filter_characters]{};
    size_t lower_characters = 0;
    FileJournal files[component_count]{};
};

struct PackageFile final {
    bool present = false;
    uint64_t bytes = 0;
    Sha256Digest hash{};
    PeStatus image = PeStatus::not_pe;
    uint32_t error = error::success;
};

const char* pe_status_name(PeStatus status) noexcept {
    switch (status) {
    case PeStatus::valid: return "valid";
    case PeStatus::not_pe: return "not a PE image";
    case PeStatus::wrong_machine: return "wrong architecture";
    case PeStatus::wrong_subsystem: return "wrong subsystem";
    case PeStatus::not_dll: return "not a DLL";
    }
    return "unknown";
}

const char* binding_name(Binding binding) noexcept {
    switch (binding) {
    case Binding::bind_unbound: return "unbound";
    case Binding::already_ours: return "bound to rx7900video";
    case Binding::replace_named: return "bound to another driver (named for replacement)";
    case Binding::refuse_active: return "actively used by another driver";
    case Binding::refuse_inactive: return "bound to another (stopped) driver";
    case Binding::refuse_mismatch: return "bound to a different driver than --force-replace names";
    }
    return "unknown";
}

class Installer final {
public:
    explicit Installer(const SystemOps& ops) noexcept : ops_(ops) {}

    bool initialize() noexcept;
    int status() noexcept;
    int install(const CommandLine& command) noexcept;
    int upgrade() noexcept;
    int rollback() noexcept;
    int remove() noexcept;
    int recover(bool explicit_command) noexcept;
    int recover_interrupted() noexcept;
    int firmware_copy(const CommandLine& command) noexcept;

private:
    // Output. Host GCC builds check every format string against its arguments.
#if defined(__GNUC__) && !defined(_WIN32)
    void say(const char* format, ...) noexcept __attribute__((format(printf, 2, 3)));
#else
    void say(const char* format, ...) noexcept;
#endif

    // Paths.
    bool live_path(size_t component, Path& output) const noexcept;
    bool package_path(const wchar_t* filename, Path& output) const noexcept;
    bool suffixed(const Path& base, const wchar_t* suffix, Path& output) const noexcept;
    bool backup_path(size_t component, bool staged, Path& output) const noexcept;

    // Files.
    uint32_t hash_file(const wchar_t* path, bool& exists, Sha256Digest& digest) noexcept;
    uint32_t inspect_image(const wchar_t* path, ImageKind kind, uint64_t bytes, PeStatus& status) noexcept;
    uint32_t inspect_package(uint32_t features, PackageFile (&files)[component_count], bool require) noexcept;
    uint32_t read_whole(const wchar_t* path, uint64_t maximum, HeapBuffer& buffer, bool& exists,
                        bool& oversize) noexcept;
    uint32_t verified_copy(const wchar_t* from, const wchar_t* to, const Sha256Digest& expected) noexcept;
    uint32_t place_file(const wchar_t* source, const wchar_t* live, const Sha256Digest& expected,
                        bool& restart, bool* deferred = nullptr) noexcept;
    uint32_t delete_or_schedule(const wchar_t* path, bool& restart) noexcept;
    void delete_if_present(const wchar_t* path) noexcept;

    // Registry helpers.
    uint32_t read_dword(const wchar_t* key, const wchar_t* name, bool& present, uint32_t& value) noexcept;
    uint32_t write_dword(const wchar_t* key, const wchar_t* name, uint32_t value) noexcept;
    template<size_t Capacity>
    uint32_t read_string(const wchar_t* key, const wchar_t* name, bool& present,
                         wchar_t (&output)[Capacity]) noexcept;
    uint32_t write_string(const wchar_t* key, const wchar_t* name, const wchar_t* text) noexcept;
    uint32_t read_multi(const wchar_t* key, const wchar_t* name, bool& present,
                        wchar_t (&output)[filter_characters], size_t& characters) noexcept;
    uint32_t write_multi(const wchar_t* key, const wchar_t* name, const wchar_t* list,
                         size_t characters) noexcept;
    uint32_t delete_value(const wchar_t* key, const wchar_t* name) noexcept;
    bool key_present(const wchar_t* key) noexcept;
    const wchar_t* parameters_key() noexcept;

    // Journal.
    uint32_t load_journal(Journal& journal) noexcept;
    uint32_t save_journal(const Journal& journal) noexcept;  // State written last
    uint32_t clear_journal() noexcept;
    uint32_t write_hash(const wchar_t* prefix, size_t component, bool present, const Sha256Digest& hash) noexcept;

    // Devices and the owned service.
    uint32_t inventory() noexcept;
    void note_step(const wchar_t* step) noexcept;
    const DeviceRecord* find_device(const wchar_t* instance) const noexcept;
    uint32_t ensure_service(bool& changed) noexcept;
    uint32_t remove_owned_service(bool& removed) noexcept;

    // Transactions.
    int finish_install(Journal& journal, const DeviceRecord& device, const PackageFile (&package)[component_count],
                       bool bound_now, bool& restart) noexcept;
    uint32_t restore_filters(const DeviceRecord& device, const Journal& journal, size_t& restored) noexcept;
    uint32_t apply_feature_values(uint32_t features) noexcept;
    uint32_t apply_display_values() noexcept;
    uint32_t apply_icd(uint32_t features, const PackageFile (&package)[component_count], Journal& journal,
                       bool& restart) noexcept;
    bool icd_registration_ours(bool& present) noexcept;
    int run_replace(ReplaceKind kind, uint32_t mask, const Path (&sources)[component_count],
                    const Sha256Digest (&targets)[component_count], Journal& journal) noexcept;
    uint32_t observe(size_t component, ReplaceObservation& observation) noexcept;
    int recover_replace(Journal& journal) noexcept;
    int recover_installing(Journal& journal) noexcept;
    uint32_t remove_binding(const wchar_t* instance, bool& in_place, UnbindStep& failed) noexcept;
    uint32_t remove_oem_inf(const wchar_t* inf_name) noexcept;
    int perform_remove(Journal& journal) noexcept;
    int restore_pending_filters(Journal& journal, bool explicit_command) noexcept;
    int repair_installed(Journal& journal, bool explicit_command) noexcept;
    int repair_generation(Journal& journal, bool explicit_command, bool& restart) noexcept;
    int recover_orphans(bool explicit_command) noexcept;

    // Status sections.
    void status_package() noexcept;
    void status_devices() noexcept;
    void status_service() noexcept;
    void status_files(const Journal& journal) noexcept;
    void status_registry(const Journal& journal) noexcept;
    void status_icd(const Journal& journal) noexcept;
    void status_firmware() noexcept;
    void status_win32k() noexcept;
    void status_display() noexcept;
    bool firmware_ready(uint16_t device_id, uint8_t revision, bool report) noexcept;
    bool rom_ready(uint16_t device_id) noexcept;

    const SystemOps& ops_;
    Path drivers_{};
    Path firmware_{};
    Path backup_{};
    DeviceInventory inventory_{};
};

void Installer::say(const char* format, ...) noexcept {
    char line[1024]{};
    va_list arguments;
    va_start(arguments, format);
#if defined(_WIN32)
    const int written = _vsnprintf(line, sizeof(line) - 1, format, arguments);
#else
    const int written = vsnprintf(line, sizeof(line), format, arguments);
#endif
    va_end(arguments);
    line[sizeof(line) - 1] = 0;
    if (written < 0) {
        line[sizeof(line) - 2] = '~';
    }
    ops_.print(ops_.context, line);
}

bool Installer::initialize() noexcept {
    return ops_.system_directory != nullptr && ops_.inf_directory != nullptr && ops_.package_directory != nullptr &&
           path_join(drivers_, ops_.system_directory, L"drivers") &&
           path_join(firmware_, drivers_, firmware_directory_name) &&
           path_join(backup_, firmware_, backup_directory_name);
}

bool Installer::live_path(size_t component, Path& output) const noexcept {
    if (component >= component_count) {
        return false;
    }
    const auto& file = component_files[component];
    return path_join(output, file.destination == Destination::drivers ? drivers_ : ops_.system_directory,
                     file.filename);
}

bool Installer::package_path(const wchar_t* filename, Path& output) const noexcept {
    return path_join(output, ops_.package_directory, filename);
}

bool Installer::suffixed(const Path& base, const wchar_t* suffix, Path& output) const noexcept {
    return text_copy(output, base) && text_append(output, suffix);
}

bool Installer::backup_path(size_t component, bool staged, Path& output) const noexcept {
    if (component >= component_count || !path_join(output, backup_, component_files[component].filename)) {
        return false;
    }
    return !staged || text_append(output, staged_suffix);
}

uint32_t Installer::hash_file(const wchar_t* path, bool& exists, Sha256Digest& digest) noexcept {
    uint64_t bytes = 0;
    exists = false;
    uint32_t result = ops_.file_size(ops_.context, path, exists, bytes);

    if (result != error::success || !exists) {
        return result;
    }

    Sha256 hasher;
    uint8_t chunk[hash_chunk_bytes];
    uint64_t offset = 0;

    while (offset < bytes) {
        uint32_t received = 0;
        const uint64_t remaining = bytes - offset;
        const uint32_t request = remaining < sizeof(chunk) ? static_cast<uint32_t>(remaining) : sizeof(chunk);
        result = ops_.read_file(ops_.context, path, offset, chunk, request, received);

        if (result != error::success) {
            return result;
        }
        if (received == 0 || received > request) {
            return error::read_fault;
        }
        hasher.update(chunk, received);
        offset += received;
    }

    digest = hasher.finish();
    return error::success;
}

uint32_t Installer::inspect_image(const wchar_t* path, ImageKind kind, uint64_t bytes, PeStatus& status) noexcept {
    uint8_t dos[pe_dos_header_bytes]{};
    uint8_t headers[pe_headers_bytes]{};
    uint32_t received = 0;
    status = PeStatus::not_pe;

    if (bytes < pe_dos_header_bytes) {
        return error::success;
    }

    uint32_t result = ops_.read_file(ops_.context, path, 0, dos, sizeof(dos), received);

    if (result != error::success) {
        return result;
    }

    uint32_t offset = 0;

    if (received != sizeof(dos) || !pe_header_offset(ByteView(dos, sizeof(dos)), bytes, offset)) {
        return error::success;
    }

    result = ops_.read_file(ops_.context, path, offset, headers, sizeof(headers), received);

    if (result != error::success) {
        return result;
    }
    if (received == sizeof(headers)) {
        status = check_pe_headers(ByteView(headers, sizeof(headers)), ops_.machine, kind);
    }
    return error::success;
}

uint32_t Installer::inspect_package(uint32_t features, PackageFile (&files)[component_count], bool require) noexcept {
    uint32_t first_error = error::success;

    for (size_t index = 0; index < component_count; ++index) {
        files[index] = PackageFile{};

        if (!component_selected(index, features)) {
            continue;
        }

        Path path{};
        auto& file = files[index];

        if (!package_path(component_files[index].filename, path)) {
            file.error = error::buffer_overflow;
        } else {
            file.error = ops_.file_size(ops_.context, path, file.present, file.bytes);
        }
        if (file.error == error::success && !file.present) {
            file.error = error::file_not_found;
        }
        if (file.error == error::success) {
            file.error = inspect_image(path, component_files[index].image, file.bytes, file.image);
        }
        if (file.error == error::success && file.image != PeStatus::valid) {
            file.error = error::bad_exe_format;
        }
        if (file.error == error::success) {
            bool exists = false;
            file.error = hash_file(path, exists, file.hash);
        }
        if (file.error != error::success && require) {
            say("Package file %ls: %s (Win32 error %lu).", component_files[index].filename,
                file.present ? pe_status_name(file.image) : "missing", static_cast<unsigned long>(file.error));
            first_error = first_error == error::success ? file.error : first_error;
        }
    }

    Path inf{};
    bool exists = false;
    uint64_t bytes = 0;

    if (!package_path(inf_filename, inf) || ops_.file_size(ops_.context, inf, exists, bytes) != error::success ||
        !exists) {
        if (require) {
            say("Package file %ls is missing.", inf_filename);
        }
        first_error = first_error == error::success ? error::file_not_found : first_error;
    }

    return first_error;
}

uint32_t Installer::read_whole(const wchar_t* path, uint64_t maximum, HeapBuffer& buffer, bool& exists,
                               bool& oversize) noexcept {
    uint64_t bytes = 0;
    oversize = false;
    uint32_t result = ops_.file_size(ops_.context, path, exists, bytes);

    if (result != error::success || !exists) {
        return result;
    }
    if (bytes > maximum) {
        oversize = true;
        return error::success;
    }
    if (!buffer.allocate(static_cast<size_t>(bytes))) {
        return error::insufficient_buffer;
    }

    uint64_t offset = 0;

    while (offset < bytes) {
        const uint64_t remaining = bytes - offset;
        const uint32_t request = remaining < 0x100000u ? static_cast<uint32_t>(remaining) : 0x100000u;
        uint32_t received = 0;
        // The buffer owns bytes writable bytes; offset + request never exceeds it.
        result = ops_.read_file(ops_.context, path, offset, buffer.data() + offset, request, received);
        if (result != error::success) {
            return result;
        }
        if (received == 0 || received > request) {
            return error::read_fault;
        }
        offset += received;
    }
    return error::success;
}

uint32_t Installer::verified_copy(const wchar_t* from, const wchar_t* to, const Sha256Digest& expected) noexcept {
    uint32_t result = ops_.copy_file(ops_.context, from, to);

    if (result != error::success) {
        return result;
    }

    bool exists = false;
    Sha256Digest observed{};
    result = hash_file(to, exists, observed);

    if (result != error::success) {
        return result;
    }
    if (!exists || observed != expected) {
        ops_.delete_file(ops_.context, to);
        return error::invalid_data;
    }
    return error::success;
}

// Places verified bytes at live without ever leaving it absent: stage beside
// it, retire the old file by rename, then rename the staged copy. A file the OS
// refuses to rename (loaded image) is replaced at the next restart instead.
uint32_t Installer::place_file(const wchar_t* source, const wchar_t* live, const Sha256Digest& expected,
                               bool& restart, bool* deferred) noexcept {
    if (deferred != nullptr) {
        *deferred = false;
    }

    Path live_copy{};
    Path staged{};
    Path retired{};

    if (!text_copy(live_copy, live) || !suffixed(live_copy, staged_suffix, staged) ||
        !suffixed(live_copy, retired_suffix, retired)) {
        return error::buffer_overflow;
    }

    uint32_t result = verified_copy(source, staged, expected);

    if (result != error::success) {
        return result;
    }

    bool exists = false;
    uint64_t bytes = 0;
    result = ops_.file_size(ops_.context, live, exists, bytes);

    if (result != error::success) {
        return result;
    }
    if (exists) {
        result = ops_.move_file(ops_.context, live, retired);
        if (result == error::access_denied || result == error::sharing_violation) {
            result = ops_.replace_on_restart(ops_.context, staged, live);
            if (result == error::success) {
                restart = true;
                if (deferred != nullptr) {
                    *deferred = true;
                }
            }
            return result;
        }
        if (result != error::success) {
            return result;
        }
    }

    result = ops_.move_file(ops_.context, staged, live);

    if (result != error::success) {
        if (exists) {
            ops_.move_file(ops_.context, retired, live);
        }
        return result;
    }
    if (exists) {
        delete_or_schedule(retired, restart);
    }
    return error::success;
}

uint32_t Installer::delete_or_schedule(const wchar_t* path, bool& restart) noexcept {
    const uint32_t result = ops_.delete_file(ops_.context, path);

    if (result == error::success || result == error::file_not_found || result == error::path_not_found) {
        return error::success;
    }
    if (result == error::access_denied || result == error::sharing_violation) {
        const uint32_t scheduled = ops_.replace_on_restart(ops_.context, path, nullptr);
        if (scheduled == error::success) {
            restart = true;
        }
        return scheduled;
    }
    return result;
}

void Installer::delete_if_present(const wchar_t* path) noexcept {
    ops_.delete_file(ops_.context, path);
}

uint32_t Installer::read_dword(const wchar_t* key, const wchar_t* name, bool& present, uint32_t& value) noexcept {
    uint32_t type = 0;
    uint32_t bytes = 0;
    uint32_t data = 0;
    present = false;
    const uint32_t result = ops_.read_value(ops_.context, key, name, type, &data, sizeof(data), bytes);

    if (result == error::file_not_found || result == error::path_not_found) {
        return error::success;
    }
    if (result != error::success) {
        return result;
    }
    if (type != value_type::dword || bytes != sizeof(data)) {
        return error::invalid_data;
    }
    present = true;
    value = data;
    return error::success;
}

uint32_t Installer::write_dword(const wchar_t* key, const wchar_t* name, uint32_t value) noexcept {
    return ops_.write_value(ops_.context, key, name, value_type::dword, &value, sizeof(value));
}

template<size_t Capacity>
uint32_t Installer::read_string(const wchar_t* key, const wchar_t* name, bool& present,
                                wchar_t (&output)[Capacity]) noexcept {
    uint32_t type = 0;
    uint32_t bytes = 0;
    present = false;
    output[0] = 0;
    const uint32_t result = ops_.read_value(ops_.context, key, name, type, output,
                                            static_cast<uint32_t>(sizeof(output)), bytes);

    if (result == error::file_not_found || result == error::path_not_found) {
        return error::success;
    }
    if (result != error::success) {
        output[0] = 0;
        return result;
    }

    const size_t characters = bytes / sizeof(wchar_t);

    if (type != value_type::string || bytes % sizeof(wchar_t) != 0 || characters == 0 ||
        characters > Capacity || output[characters - 1] != 0 ||
        text_length(output, characters) != characters - 1) {
        output[0] = 0;
        return error::invalid_data;
    }
    present = true;
    return error::success;
}

uint32_t Installer::write_string(const wchar_t* key, const wchar_t* name, const wchar_t* text) noexcept {
    const size_t characters = text_length(text, 4096);

    if (characters == 4096) {
        return error::invalid_parameter;
    }
    return ops_.write_value(ops_.context, key, name, value_type::string, text,
                            static_cast<uint32_t>((characters + 1) * sizeof(wchar_t)));
}

uint32_t Installer::read_multi(const wchar_t* key, const wchar_t* name, bool& present,
                               wchar_t (&output)[filter_characters], size_t& characters) noexcept {
    uint32_t type = 0;
    uint32_t bytes = 0;
    present = false;
    characters = 0;
    const uint32_t result = ops_.read_value(ops_.context, key, name, type, output,
                                            static_cast<uint32_t>(sizeof(output)), bytes);

    if (result == error::file_not_found || result == error::path_not_found) {
        return error::success;
    }
    if (result != error::success) {
        return result;
    }
    if (type != value_type::multi_string || bytes % sizeof(wchar_t) != 0 || bytes < 2 * sizeof(wchar_t)) {
        return error::invalid_data;
    }
    present = true;
    characters = bytes / sizeof(wchar_t);
    return error::success;
}

uint32_t Installer::write_multi(const wchar_t* key, const wchar_t* name, const wchar_t* list,
                                size_t characters) noexcept {
    return ops_.write_value(ops_.context, key, name, value_type::multi_string, list,
                            static_cast<uint32_t>(characters * sizeof(wchar_t)));
}

uint32_t Installer::delete_value(const wchar_t* key, const wchar_t* name) noexcept {
    const uint32_t result = ops_.delete_value(ops_.context, key, name);
    return result == error::file_not_found || result == error::path_not_found ? error::success : result;
}

bool Installer::key_present(const wchar_t* key) noexcept {
    bool exists = false;
    return ops_.key_exists(ops_.context, key, exists) == error::success && exists;
}

const wchar_t* Installer::parameters_key() noexcept {
    return effective_parameters_key(key_present(use_new_key), key_present(profile_device0_key));
}

// Journal value names for per-file hashes: <prefix>.<filename>.
bool hash_value_name(const wchar_t* prefix, size_t component, wchar_t (&output)[64]) noexcept {
    return text_copy(output, prefix) && text_append(output, L".") &&
           text_append(output, component_files[component].filename);
}

uint32_t Installer::write_hash(const wchar_t* prefix, size_t component, bool present,
                               const Sha256Digest& hash) noexcept {
    wchar_t name[64]{};

    if (!hash_value_name(prefix, component, name)) {
        return error::buffer_overflow;
    }
    if (!present) {
        return delete_value(journal_key, name);
    }

    wchar_t text[sha256_hex_characters]{};
    sha256_hex(hash, text);
    return write_string(journal_key, name, text);
}

uint32_t Installer::load_journal(Journal& journal) noexcept {
    journal = Journal{};

    bool present = false;
    uint32_t value = 0;
    uint32_t result = read_dword(journal_key, L"State", present, value);

    if (result != error::success) {
        return result;
    }
    if (!present) {
        return error::success;
    }
    if (value > static_cast<uint32_t>(JournalState::filters_pending)) {
        return error::invalid_data;
    }
    journal.state = static_cast<JournalState>(value);

    if ((result = read_dword(journal_key, L"Operation", present, value)) != error::success) {
        return result;
    }
    journal.operation = present && value == static_cast<uint32_t>(ReplaceKind::rollback) ? ReplaceKind::rollback
                                                                                          : ReplaceKind::upgrade;
    if ((result = read_dword(journal_key, L"Features", present, value)) != error::success) {
        return result;
    }
    journal.features = present ? value & known_features : 0;
    if ((result = read_dword(journal_key, L"RestartPending", present, value)) != error::success) {
        return result;
    }
    journal.restart_pending = present && value == 1;
    uint32_t boot_low = 0;
    uint32_t boot_high = 0;
    bool low_present = false;
    bool high_present = false;
    if ((result = read_dword(journal_key, L"QueuedBootLow", low_present, boot_low)) != error::success ||
        (result = read_dword(journal_key, L"QueuedBootHigh", high_present, boot_high)) != error::success) {
        return result;
    }
    journal.queued_boot_known = low_present && high_present;
    journal.queued_boot = (static_cast<uint64_t>(boot_high) << 32) | boot_low;

    if ((result = read_string(journal_key, L"DeviceInstance", present, journal.instance)) != error::success ||
        (result = read_string(journal_key, L"HardwareId", present, journal.hardware_id)) != error::success ||
        (result = read_string(journal_key, L"PreviousService", present, journal.previous_service)) != error::success ||
        (result = read_string(journal_key, L"InfName", present, journal.inf_name)) != error::success) {
        return result;
    }

    bool upper = false;
    bool lower = false;

    if ((result = read_multi(journal_key, L"SnapshotUpperFilters", upper, journal.upper,
                             journal.upper_characters)) != error::success ||
        (result = read_multi(journal_key, L"SnapshotLowerFilters", lower, journal.lower,
                             journal.lower_characters)) != error::success) {
        return result;
    }
    if ((result = read_dword(journal_key, L"SnapshotTaken", present, value)) != error::success) {
        return result;
    }
    journal.snapshot = present && value == 1;

    for (size_t index = 0; index < component_count; ++index) {
        struct Slot final {
            const wchar_t* prefix;
            bool* present;
            Sha256Digest* hash;
        };
        auto& file = journal.files[index];
        const Slot slots[]{{L"Installed", &file.installed, &file.installed_hash},
                           {L"Backup", &file.backup, &file.backup_hash},
                           {L"Target", &file.target, &file.target_hash},
                           {L"Prior", &file.prior, &file.prior_hash},
                           {L"Pending", &file.pending, &file.pending_hash}};

        for (const auto& slot : slots) {
            wchar_t name[64]{};
            wchar_t text[sha256_hex_characters]{};
            bool found = false;

            if (!hash_value_name(slot.prefix, index, name)) {
                return error::buffer_overflow;
            }
            if ((result = read_string(journal_key, name, found, text)) != error::success) {
                return result;
            }
            if (found && !parse_sha256_hex(text, *slot.hash)) {
                return error::invalid_data;
            }
            *slot.present = found;
        }
    }

    return error::success;
}

uint32_t Installer::save_journal(const Journal& journal) noexcept {
    uint32_t result = error::success;
    const auto check = [&result](uint32_t value) noexcept {
        if (result == error::success) {
            result = value;
        }
    };
    const auto string_or_delete = [&](const wchar_t* name, const wchar_t* text) noexcept {
        check(text[0] != 0 ? write_string(journal_key, name, text) : delete_value(journal_key, name));
    };

    check(write_dword(journal_key, L"Operation", static_cast<uint32_t>(journal.operation)));
    check(write_dword(journal_key, L"Features", journal.features));
    check(write_dword(journal_key, L"RestartPending", journal.restart_pending ? 1 : 0));
    bool any_pending = false;
    for (const auto& file : journal.files) {
        any_pending = any_pending || file.pending;
    }
    uint64_t boot = journal.queued_boot;
    bool boot_known = journal.queued_boot_known;
    if (any_pending && !boot_known) {
        boot_known = ops_.boot_id(ops_.context, boot) == error::success;
    }
    if (any_pending && boot_known) {
        check(write_dword(journal_key, L"QueuedBootLow", static_cast<uint32_t>(boot)));
        check(write_dword(journal_key, L"QueuedBootHigh", static_cast<uint32_t>(boot >> 32)));
    } else {
        check(delete_value(journal_key, L"QueuedBootLow"));
        check(delete_value(journal_key, L"QueuedBootHigh"));
    }
    check(write_string(journal_key, L"PackageVersion", package_version));
    string_or_delete(L"DeviceInstance", journal.instance);
    string_or_delete(L"HardwareId", journal.hardware_id);
    string_or_delete(L"PreviousService", journal.previous_service);
    string_or_delete(L"InfName", journal.inf_name);
    check(write_dword(journal_key, L"SnapshotTaken", journal.snapshot ? 1 : 0));
    check(journal.snapshot && journal.upper_characters != 0 ?
              write_multi(journal_key, L"SnapshotUpperFilters", journal.upper, journal.upper_characters) :
              delete_value(journal_key, L"SnapshotUpperFilters"));
    check(journal.snapshot && journal.lower_characters != 0 ?
              write_multi(journal_key, L"SnapshotLowerFilters", journal.lower, journal.lower_characters) :
              delete_value(journal_key, L"SnapshotLowerFilters"));

    for (size_t index = 0; index < component_count; ++index) {
        const auto& file = journal.files[index];
        check(write_hash(L"Installed", index, file.installed, file.installed_hash));
        check(write_hash(L"Backup", index, file.backup, file.backup_hash));
        check(write_hash(L"Target", index, file.target, file.target_hash));
        check(write_hash(L"Prior", index, file.prior, file.prior_hash));
        check(write_hash(L"Pending", index, file.pending, file.pending_hash));
    }

    // The state is the commit point: it is written only after every value it
    // refers to, so a reader never sees a state without its data.
    check(write_dword(journal_key, L"State", static_cast<uint32_t>(journal.state)));
    return result;
}

uint32_t Installer::clear_journal() noexcept {
    // Remove the commit point first so an interrupted clear reads as "none".
    uint32_t result = delete_value(journal_key, L"State");

    constexpr const wchar_t* names[]{L"Operation", L"Features", L"RestartPending", L"QueuedBootLow",
        L"QueuedBootHigh", L"PackageVersion",
        L"DeviceInstance", L"HardwareId", L"PreviousService", L"InfName", L"SnapshotTaken",
        L"SnapshotUpperFilters", L"SnapshotLowerFilters", L"Step"};

    for (const auto* name : names) {
        const uint32_t removed = delete_value(journal_key, name);
        result = result == error::success ? removed : result;
    }
    constexpr const wchar_t* prefixes[]{L"Installed", L"Backup", L"Target", L"Prior", L"Pending"};

    for (size_t index = 0; index < component_count; ++index) {
        for (const auto* prefix : prefixes) {
            const uint32_t removed = write_hash(prefix, index, false, Sha256Digest{});
            result = result == error::success ? removed : result;
        }
    }

    const uint32_t removed = ops_.delete_key(ops_.context, journal_key);
    if (removed != error::success && removed != error::file_not_found && removed != error::path_not_found) {
        result = result == error::success ? removed : result;
    }
    return result;
}

// Progress trail of a transaction in flight. Journal writes are flushed to
// disk, so after an interruption the last step reached can be read back.
void Installer::note_step(const wchar_t* step) noexcept {
    write_string(journal_key, L"Step", step);
}

uint32_t Installer::inventory() noexcept {
    inventory_ = DeviceInventory{};
    return ops_.enumerate_devices(ops_.context, inventory_);
}

const DeviceRecord* Installer::find_device(const wchar_t* instance) const noexcept {
    if (instance != nullptr && instance[0] != 0) {
        for (size_t index = 0; index < inventory_.count; ++index) {
            if (text_equal_insensitive(inventory_.devices[index].instance_id, instance)) {
                return &inventory_.devices[index];
            }
        }
        return nullptr;
    }

    size_t selected = 0;
    return select_navi31(inventory_, selected) == Selection::unique ? &inventory_.devices[selected] : nullptr;
}

uint32_t Installer::ensure_service(bool& changed) noexcept {
    ServiceInfo service{};
    changed = false;
    uint32_t result = ops_.query_service(ops_.context, service);

    if (result != error::success) {
        return result;
    }
    if (!service.exists) {
        // A missing or stale-deleted service of a bound device: recreate exactly.
        if ((result = ops_.create_service(ops_.context)) != error::success) {
            return result;
        }
        changed = true;
        say("Recreated the rx7900video demand-start kernel service.");
        if ((result = ops_.query_service(ops_.context, service)) != error::success) {
            return result;
        }
    }
    if (!service.exists || service.type != service_type_kernel ||
        !service_image_matches(service.image_path, ops_.system_directory)) {
        say("Service rx7900video has a foreign configuration (type %lu, image %ls); it was not modified.",
            static_cast<unsigned long>(service.type), service.image_path);
        return error::invalid_data;
    }
    if (service.start != service_start_demand) {
        // Only ever toward demand start; boot/system/auto start are never set.
        if ((result = ops_.set_demand_start(ops_.context)) != error::success) {
            return result;
        }
        changed = true;
        say("Service rx7900video start type %lu was reset to demand start (3).",
            static_cast<unsigned long>(service.start));
    }
    return error::success;
}

uint32_t Installer::remove_owned_service(bool& removed) noexcept {
    ServiceInfo service{};
    removed = false;
    uint32_t result = ops_.query_service(ops_.context, service);

    if (result != error::success || !service.exists) {
        return result;
    }
    if (service.type != service_type_kernel || !service_image_matches(service.image_path, ops_.system_directory)) {
        say("Service rx7900video points to %ls, not this package's image; it was preserved.", service.image_path);
        return error::success;
    }
    result = ops_.delete_service(ops_.context);
    if (result == error::success || result == error::service_marked_for_delete) {
        removed = true;
        return error::success;
    }
    return result == error::service_does_not_exist ? error::success : result;
}

uint32_t Installer::restore_filters(const DeviceRecord& device, const Journal& journal, size_t& restored) noexcept {
    restored = 0;

    if (!journal.snapshot) {
        return error::success;
    }

    struct List final {
        bool upper;
        const wchar_t* original;
        size_t original_characters;
        const wchar_t* current;
        size_t current_characters;
    };
    const List lists[]{
        {true, journal.upper, journal.upper_characters, device.upper_filters, device.upper_characters},
        {false, journal.lower, journal.lower_characters, device.lower_filters, device.lower_characters}};

    for (const auto& list : lists) {
        wchar_t merged[filter_characters]{};
        size_t characters = 0;
        size_t added = 0;

        if (!merge_missing_filters(list.original, list.original_characters, list.current,
                                   list.current_characters, merged, characters, added)) {
            return error::invalid_data;
        }
        if (added == 0) {
            continue;
        }

        const uint32_t result = ops_.write_filters(ops_.context, device.instance_id, list.upper, merged, characters);
        if (result != error::success) {
            return result;
        }
        restored += added;
    }
    return error::success;
}

uint32_t Installer::apply_display_values() noexcept {
    // Exactly what radeondisplay.inf ServiceSettings writes; repaired when missing.
    constexpr wchar_t drivers[] = L"radeondisp\0";
    uint32_t result = write_multi(device0_key, L"InstalledDisplayDrivers", drivers, sizeof(drivers) / sizeof(wchar_t));

    if (result == error::success) {
        result = write_dword(device0_key, L"VgaCompatible", 0);
    }
    if (result == error::success) {
        result = write_string(device0_key, L"Device Description", service_display_name);
    }
    if (result == error::success) {
        result = write_string(video_subkey, L"Service", service_name);
    }
    return result;
}

uint32_t Installer::apply_feature_values(uint32_t features) noexcept {
    const wchar_t* effective = parameters_key();
    const wchar_t* keys[]{device0_key, effective};
    const size_t key_count = effective == device0_key ? 1 : 2;

    for (size_t key = 0; key < key_count; ++key) {
        for (const auto& rule : feature_rules) {
            if (rule.registry_value == nullptr) {
                continue;
            }
            const uint32_t result = write_dword(keys[key], rule.registry_value, (features & rule.feature) != 0 ? 1 : 0);
            if (result != error::success) {
                return result;
            }
        }
    }

    // Read back the key VideoPort will actually use.
    for (const auto& rule : feature_rules) {
        if (rule.registry_value == nullptr) {
            continue;
        }
        bool present = false;
        uint32_t value = 0;
        const uint32_t result = read_dword(effective, rule.registry_value, present, value);
        if (result != error::success || !present || value != ((features & rule.feature) != 0 ? 1u : 0u)) {
            return result != error::success ? result : error::read_fault;
        }
    }
    return error::success;
}

bool Installer::icd_registration_ours(bool& present) noexcept {
    wchar_t dll[path_characters]{};
    present = false;

    if (read_string(opengl_drivers_key, L"Dll", present, dll) != error::success || !present) {
        return false;
    }
    if (text_equal_insensitive(dll, component_files[icd_component].filename)) {
        return true;
    }

    Path absolute{};
    return path_join(absolute, ops_.system_directory, component_files[icd_component].filename) &&
           text_equal_insensitive(dll, absolute);
}

uint32_t Installer::apply_icd(uint32_t features, const PackageFile (&package)[component_count], Journal& journal,
                              bool& restart) noexcept {
    Path live{};

    if (!live_path(icd_component, live)) {
        return error::buffer_overflow;
    }

    bool exists = false;
    Sha256Digest current{};
    uint32_t result = hash_file(live, exists, current);

    if (result != error::success) {
        return result;
    }

    auto& file = journal.files[icd_component];

    if ((features & feature_opengl_icd) == 0) {
        bool registered = false;
        const bool ours = icd_registration_ours(registered);

        if (registered && ours) {
            if ((result = ops_.delete_key(ops_.context, opengl_drivers_key)) != error::success) {
                return result;
            }
            say("Removed the OpenGLDrivers\\RadeonGL registration (--opengl-icd not requested).");
        } else if (registered) {
            say("OpenGLDrivers\\RadeonGL names another DLL; it was preserved.");
        }

        const bool owned_file = exists && ((file.installed && current == file.installed_hash) ||
                                           (package[icd_component].present && current == package[icd_component].hash));
        if (owned_file) {
            if ((result = delete_or_schedule(live, restart)) != error::success) {
                return result;
            }
        }
        file.installed = false;
        return error::success;
    }

    const auto& source = package[icd_component];

    const bool kept = exists && current != source.hash && file.installed && current == file.installed_hash;

    if (kept) {
        // An installed, recorded ICD of another version: only upgrade replaces it.
        say("Installed radeonglicd.dll differs from this package; it was kept. Run radeonctl upgrade to replace it.");
    } else if (!exists || current != source.hash) {
        Path from{};
        if (!package_path(component_files[icd_component].filename, from)) {
            return error::buffer_overflow;
        }
        bool deferred = false;
        if ((result = place_file(from, live, source.hash, restart, &deferred)) != error::success) {
            return result;
        }
        file.pending = deferred;
        file.pending_hash = source.hash;
        journal.queued_boot_known = journal.queued_boot_known && !deferred;
        journal.restart_pending = journal.restart_pending || deferred;
        say(deferred ? "Queued %ls for the next restart (in use)." : "Installed %ls.", live);
    }

    // OpenGLDrivers\RadeonGL as ReactOS opengl32 icdload.c reads it; Version and
    // DriverVersion equal the radeondisp OPENGL_GETINFO reply.
    if ((result = write_string(opengl_drivers_key, L"Dll", component_files[icd_component].filename)) !=
            error::success ||
        (result = write_dword(opengl_drivers_key, L"Version", opengl::interface_version)) != error::success ||
        (result = write_dword(opengl_drivers_key, L"DriverVersion", opengl::driver_version)) != error::success ||
        (result = write_dword(opengl_drivers_key, L"Flags", 0)) != error::success) {
        return result;
    }

    bool registered = false;
    bool version = false;
    bool driver_version = false;
    uint32_t value = 0;
    uint32_t value2 = 0;

    if (!icd_registration_ours(registered) || !registered ||
        read_dword(opengl_drivers_key, L"Version", version, value) != error::success || !version ||
        value != opengl::interface_version ||
        read_dword(opengl_drivers_key, L"DriverVersion", driver_version, value2) != error::success ||
        !driver_version || value2 != opengl::driver_version) {
        return error::read_fault;
    }

    file.installed = true;
    file.installed_hash = kept ? current : source.hash;
    return error::success;
}

int Installer::finish_install(Journal& journal, const DeviceRecord& device,
                              const PackageFile (&package)[component_count], bool bound_now, bool& restart) noexcept {
    size_t restored = 0;
    uint32_t result = restore_filters(device, journal, restored);

    if (result != error::success) {
        say("Device filter preservation failed (Win32 error %lu).", static_cast<unsigned long>(result));
        return exit_code::failure;
    }
    if (restored != 0) {
        say("Restored %lu device filter entr%s that the binding removed.", static_cast<unsigned long>(restored),
            restored == 1 ? "y" : "ies");
        restart = true;
    }

    note_step(L"service");
    bool service_changed = false;
    if ((result = ensure_service(service_changed)) != error::success) {
        say("The rx7900video service could not be verified (Win32 error %lu).", static_cast<unsigned long>(result));
        return exit_code::failure;
    }
    restart = restart || service_changed;

    for (size_t index = 0; index < component_count; ++index) {
        if (!component_files[index].inf_managed) {
            continue;
        }

        Path live{};
        bool exists = false;
        Sha256Digest current{};

        if (!live_path(index, live) || hash_file(live, exists, current) != error::success) {
            say("Cannot read installed %ls.", component_files[index].filename);
            return exit_code::failure;
        }
        if (exists && current == package[index].hash) {
            journal.files[index].installed = true;
            journal.files[index].installed_hash = current;
            continue;
        }
        if (exists && !bound_now) {
            say("Installed %ls differs from this package; it was kept. Run radeonctl upgrade to replace it.",
                component_files[index].filename);
            journal.files[index].installed = true;
            journal.files[index].installed_hash = current;
            continue;
        }

        // Missing after binding, or SetupAPI left other bytes: place the package copy.
        Path from{};
        bool deferred = false;
        if (!package_path(component_files[index].filename, from) ||
            place_file(from, live, package[index].hash, restart, &deferred) != error::success) {
            say("Cannot place %ls from the package.", component_files[index].filename);
            return exit_code::failure;
        }
        journal.files[index].pending = deferred;
        journal.files[index].pending_hash = package[index].hash;
        journal.queued_boot_known = journal.queued_boot_known && !deferred;
        journal.restart_pending = journal.restart_pending || deferred;
        journal.files[index].installed = true;
        journal.files[index].installed_hash = package[index].hash;
        say("Placed %ls from the package.", live);
    }

    note_step(L"options");
    if ((result = apply_display_values()) != error::success ||
        (result = apply_feature_values(journal.features)) != error::success) {
        say("Writing the rx7900video Device0 values failed (Win32 error %lu).", static_cast<unsigned long>(result));
        return exit_code::failure;
    }
    note_step(L"opengl");
    if ((result = apply_icd(journal.features, package, journal, restart)) != error::success) {
        say("OpenGL ICD installation/registration failed (Win32 error %lu).", static_cast<unsigned long>(result));
        return exit_code::failure;
    }
    note_step(L"recording");

    journal.state = JournalState::installed;
    for (auto& file : journal.files) {
        file.target = false;
        file.prior = false;
    }
    if ((result = save_journal(journal)) != error::success) {
        say("Recording the installed state failed (Win32 error %lu).", static_cast<unsigned long>(result));
        return exit_code::failure;
    }
    return exit_code::success;
}

int Installer::install(const CommandLine& command) noexcept {
    uint32_t requirement = 0;
    const uint32_t invalid = invalid_feature(command.features, requirement);

    if (invalid != 0) {
        const wchar_t* option = L"an unknown option";
        const wchar_t* needed = L"an unknown option";
        for (const auto& rule : feature_rules) {
            option = rule.feature == invalid ? rule.option : option;
            needed = rule.feature == requirement ? rule.option : needed;
        }
        say("Refused: %ls requires %ls. Nothing was changed.", option, needed);
        return exit_code::refused;
    }

    PackageFile package[component_count]{};

    if (inspect_package(command.features, package, true) != error::success) {
        say("The package is incomplete or built for another architecture. Nothing was changed.");
        return exit_code::failure;
    }

    Journal journal{};
    uint32_t result = load_journal(journal);

    if (result != error::success) {
        say("The installer journal is unreadable (Win32 error %lu); run radeonctl status.",
            static_cast<unsigned long>(result));
        return exit_code::failure;
    }
    if (journal.state == JournalState::installed) {
        // Same rule as upgrade: never re-record files over a queued replacement.
        for (size_t index = 0; index < component_count; ++index) {
            if (journal.files[index].pending) {
                say("Install refused: %ls is replaced only at the next restart. Restart ReactOS and run radeonctl "
                    "recover first (radeonctl rollback can cancel that replacement now).",
                    component_files[index].filename);
                return exit_code::refused;
            }
        }
    }
    if (journal.state == JournalState::filters_pending) {
        const int restored = restore_pending_filters(journal, false);
        if (restored != exit_code::success) {
            say("Preserved filters of the previous removal are still pending; run radeonctl recover first.");
            return exit_code::refused;
        }
        journal = Journal{};
    }

    if ((result = inventory()) != error::success) {
        say("Device enumeration failed (Win32 error %lu). Nothing was changed.", static_cast<unsigned long>(result));
        return exit_code::failure;
    }

    size_t selected = 0;
    const Selection selection = select_navi31(inventory_, selected);

    if (inventory_.absent != 0) {
        say("Ignored %lu recorded Navi31 device(s) that are not attached (stale registry entries).",
            static_cast<unsigned long>(inventory_.absent));
    }
    if (selection == Selection::none_present) {
        say("No Navi31 GPU (PCI 1002:7448/744A/744B/744C/745E) is present. Nothing was changed.");
        if (inventory_.lightning_present) {
            say("This system has the QEMU Lightning virtual device (1234:11F1); it is not a Radeon and was not touched.");
        }
        say("Bare metal: install ReactOS on the RX 7900 machine. VM: pass the GPU through with VFIO first.");
        return exit_code::no_navi31;
    }
    if (selection == Selection::multiple) {
        say("Refused: %lu Navi31 GPU functions are present%s; this installer binds exactly one.",
            static_cast<unsigned long>(inventory_.count + inventory_.omitted),
            inventory_.omitted != 0 ? " (some beyond the inventory limit)" : "");
        return exit_code::refused;
    }

    const DeviceRecord& device = inventory_.devices[selected];
    const Binding binding = decide_binding(device, command.force_replace ? command.force_service : nullptr);

    say("Navi31 GPU 1002:%04X revision %02X at %ls: %s.", static_cast<unsigned>(device.device_id),
        static_cast<unsigned>(device.revision), device.instance_id, binding_name(binding));
    if (device.service[0] != 0) {
        say("  current service: %ls, %s, problem %lu.", device.service, device.started ? "started" : "not started",
            static_cast<unsigned long>(device.problem));
    }

    if (binding == Binding::refuse_active || binding == Binding::refuse_inactive) {
        say("Refused: %ls %s this GPU. Re-run with --force-replace %ls to replace that binding explicitly.",
            device.service, binding == Binding::refuse_active ? "is actively driving" : "is bound to",
            device.service);
        say("Replacing the active display driver needs a recoverable disk and a second way to reach the machine.");
        return exit_code::refused;
    }
    if (binding == Binding::refuse_mismatch) {
        say("Refused: --force-replace names %ls but the GPU is bound to %ls. Nothing was changed.",
            command.force_service, device.service);
        return exit_code::refused;
    }

    if ((command.features & feature_physical_asic_init) != 0 &&
        !firmware_ready(device.device_id, device.revision, true)) {
        say("Refused: --asic requires every required firmware file below %ls. Nothing was changed.", firmware_);
        return exit_code::refused;
    }
    if ((command.features & feature_atom_rom_override) != 0 && !rom_ready(device.device_id)) {
        say("Refused: --rom-override requires a valid %ls\\rx7900.rom for 1002:%04X. Nothing was changed.",
            firmware_, static_cast<unsigned>(device.device_id));
        return exit_code::refused;
    }

    const bool bind = binding != Binding::already_ours;

    if (command.dry_run) {
        say("Dry run: would %s, write Device0 opt-ins 0x%02lX%s and record the journal. Nothing was changed.",
            bind ? "bind the GPU to rx7900video through radeondisplay.inf" : "reconfigure the existing binding",
            static_cast<unsigned long>(command.features),
            (command.features & feature_opengl_icd) != 0 ? ", install and register radeonglicd.dll" : "");
        return exit_code::success;
    }

    bool restart = false;

    if (bind) {
        // Journal before the first side effect: binding plus filter snapshot.
        journal = Journal{};
        journal.state = JournalState::installing;
        journal.features = command.features;
        text_copy(journal.instance, device.instance_id);
        text_copy(journal.hardware_id, device.hardware_id);
        text_copy(journal.previous_service, device.service);
        journal.snapshot = true;
        for (size_t index = 0; index < device.upper_characters; ++index) {
            journal.upper[index] = device.upper_filters[index];
        }
        journal.upper_characters = device.upper_characters;
        for (size_t index = 0; index < device.lower_characters; ++index) {
            journal.lower[index] = device.lower_filters[index];
        }
        journal.lower_characters = device.lower_characters;

        if ((result = save_journal(journal)) != error::success) {
            say("Recording the install journal failed (Win32 error %lu). Nothing was changed.",
                static_cast<unsigned long>(result));
            clear_journal();
            return exit_code::failure;
        }

        Path inf{};
        bool driver_restart = false;
        package_path(inf_filename, inf);
        note_step(L"binding");
        result = ops_.install_driver(ops_.context, device.instance_id, device.hardware_id, inf, driver_restart);
        note_step(L"bound");
        restart = restart || driver_restart;

        const uint32_t enumerated = inventory();
        const DeviceRecord* bound = enumerated == error::success ? find_device(journal.instance) : nullptr;

        if (result != error::success || bound == nullptr || !text_equal_insensitive(bound->service, service_name)) {
            say("Binding rx7900video failed (Win32 error %lu); rolling back.", static_cast<unsigned long>(result));
            recover_installing(journal);
            return exit_code::failure;
        }

        wchar_t inf_path[path_characters]{};
        if (ops_.driver_value(ops_.context, bound->instance_id, L"InfPath", inf_path, path_characters) ==
            error::success) {
            text_copy(journal.inf_name, inf_path);
        }
        restart = true;
        say("Bound %ls to rx7900video (%ls).", bound->instance_id, journal.inf_name[0] != 0 ? journal.inf_name : L"INF");
        const int finished = finish_install(journal, *bound, package, true, restart);
        if (finished != exit_code::success) {
            say("Install did not complete; the journal is retained. Run radeonctl recover.");
            return finished;
        }
    } else {
        if (journal.state != JournalState::installed) {
            journal = Journal{};
            text_copy(journal.instance, device.instance_id);
            text_copy(journal.hardware_id, device.hardware_id);
        }
        const bool features_changed = journal.features != command.features;
        journal.features = command.features;
        const int finished = finish_install(journal, device, package, false, restart);
        if (finished != exit_code::success) {
            return finished;
        }
        restart = restart || features_changed;
    }

    say("Installed rx7900video, radeondisp%s. Opt-ins:", (command.features & feature_opengl_icd) != 0 ?
        " and the RadeonGL ICD" : "");
    for (const auto& rule : feature_rules) {
        if (rule.registry_value != nullptr) {
            say("  %ls=%u", rule.registry_value, (command.features & rule.feature) != 0 ? 1u : 0u);
        }
    }
    if ((command.features & feature_physical_asic_init) == 0) {
        say("PhysicalAsicInit=0: the miniport performs no firmware load or ASIC bring-up.");
    }
    if (restart) {
        say("Restart ReactOS to load the new configuration; none was performed.");
        return exit_code::restart_required;
    }
    return exit_code::success;
}

int Installer::recover_installing(Journal& journal) noexcept {
    const uint32_t enumerated = inventory();
    const DeviceRecord* device = enumerated == error::success ? find_device(journal.instance) : nullptr;

    if (device != nullptr && text_equal_insensitive(device->service, service_name)) {
        PackageFile package[component_count]{};

        if (inspect_package(journal.features, package, true) != error::success) {
            say("The interrupted install bound the GPU; complete it from the same package with radeonctl recover.");
            return exit_code::failure;
        }
        bool restart = true;
        say("Completing an interrupted install of %ls.", device->instance_id);
        const int finished = finish_install(journal, *device, package, true, restart);
        return finished == exit_code::success ? exit_code::restart_required : finished;
    }

    // Not bound to us: undo everything the interrupted attempt may have created.
    say("Rolling back an install that did not bind the GPU.");
    PackageFile package[component_count]{};
    inspect_package(known_features, package, false);
    bool restart = false;
    bool removed = false;

    if (remove_owned_service(removed) != error::success) {
        return exit_code::failure;
    }
    if (removed) {
        say("Removed the stale rx7900video service.");
    }

    for (size_t index = 0; index < component_count; ++index) {
        Path live{};
        bool exists = false;
        Sha256Digest current{};

        if (!live_path(index, live) || hash_file(live, exists, current) != error::success || !exists) {
            continue;
        }
        if (package[index].present && current == package[index].hash) {
            delete_or_schedule(live, restart);
        }
        Path staged{};
        if (suffixed(live, staged_suffix, staged)) {
            delete_if_present(staged);
        }
    }

    bool registered = false;
    if (icd_registration_ours(registered) && registered) {
        ops_.delete_key(ops_.context, opengl_drivers_key);
    }
    if (device != nullptr) {
        size_t restored = 0;
        restore_filters(*device, journal, restored);
        if (restored != 0) {
            say("Restored %lu device filter entries.", static_cast<unsigned long>(restored));
        }
    }

    const uint32_t cleared = clear_journal();
    if (cleared != error::success) {
        return exit_code::failure;
    }
    return restart ? exit_code::restart_required : exit_code::success;
}

uint32_t Installer::observe(size_t component, ReplaceObservation& observation) noexcept {
    observation = ReplaceObservation{};
    Path live{};
    Path staged{};
    Path retired{};
    Path backup{};
    Path backup_staged{};

    if (!live_path(component, live) || !suffixed(live, staged_suffix, staged) ||
        !suffixed(live, retired_suffix, retired) || !backup_path(component, false, backup) ||
        !backup_path(component, true, backup_staged)) {
        return error::buffer_overflow;
    }

    struct Slot final {
        const wchar_t* path;
        bool* present;
        Sha256Digest* hash;
    };
    const Slot slots[]{{live, &observation.live, &observation.live_hash},
                       {staged, &observation.staged, &observation.staged_hash},
                       {retired, &observation.retired, &observation.retired_hash},
                       {backup, &observation.backup, &observation.backup_hash},
                       {backup_staged, &observation.backup_staged, &observation.backup_staged_hash}};

    for (const auto& slot : slots) {
        const uint32_t result = hash_file(slot.path, *slot.present, *slot.hash);
        if (result != error::success) {
            return result;
        }
    }
    return error::success;
}

int Installer::run_replace(ReplaceKind kind, uint32_t mask, const Path (&sources)[component_count],
                           const Sha256Digest (&targets)[component_count], Journal& journal) noexcept {
    const char* verb = kind == ReplaceKind::upgrade ? "Upgrade" : "Rollback";

    // A component whose earlier replacement is still queued for the restart
    // keeps its prior bytes live while <live>.radeonctl-new holds the queued
    // generation (FileJournal::pending). It may be replaced again only when the
    // target is the live content, i.e. a rollback of that queued generation:
    // the queued source is then deleted, which cancels the delayed rename
    // (ReactOS smss SmpProcessFileRenames logs a missing source and continues),
    // and its bytes become the backup. Replacing the live file instead would be
    // overwritten again by the stale rename at restart, so any other target is
    // refused here; upgrade refuses earlier while anything is pending.
    for (size_t index = 0; index < component_count; ++index) {
        auto& file = journal.files[index];
        file.target = false;
        file.prior = false;

        if ((mask & (1u << index)) == 0) {
            continue;
        }

        Path live{};
        Path staged{};
        bool exists = false;
        Sha256Digest current{};

        if (!live_path(index, live) || !suffixed(live, staged_suffix, staged) ||
            hash_file(live, exists, current) != error::success || !exists) {
            say("%s refused: installed %ls is missing; run radeonctl recover.", verb, component_files[index].filename);
            return exit_code::failure;
        }
        if (file.pending) {
            bool queued = false;
            Sha256Digest queued_hash{};
            if (current != targets[index]) {
                say("%s refused: %ls has a replacement queued for the next restart. Restart ReactOS and run "
                    "radeonctl recover first.", verb, component_files[index].filename);
                return exit_code::refused;
            }
            if (hash_file(staged, queued, queued_hash) != error::success || !queued ||
                queued_hash != file.pending_hash) {
                say("%s refused: the queued copy of %ls is missing or altered; run radeonctl recover.", verb,
                    component_files[index].filename);
                return exit_code::failure;
            }
            file.prior_hash = file.pending_hash;
        } else {
            file.prior_hash = current;
        }
        file.prior = true;
        file.target = true;
        file.target_hash = targets[index];
    }

    journal.operation = kind;
    journal.state = JournalState::replace_staging;
    uint32_t result = save_journal(journal);

    if (result != error::success || ops_.create_directory(ops_.context, firmware_) != error::success ||
        ops_.create_directory(ops_.context, backup_) != error::success) {
        say("%s could not record its journal or backup directory. Nothing was replaced.", verb);
        recover_replace(journal);
        return exit_code::failure;
    }

    // Stage every target and every prior copy, verified, before any live change.
    for (size_t index = 0; index < component_count; ++index) {
        const auto& file = journal.files[index];

        if (!file.target) {
            continue;
        }

        Path live{};
        Path staged{};
        Path backup_staged{};
        bool staged_ok = live_path(index, live) && suffixed(live, staged_suffix, staged) &&
                         backup_path(index, true, backup_staged);

        if (staged_ok && file.pending) {
            // Cancellation: the queued generation becomes the backup.
            staged_ok = verified_copy(staged, backup_staged, file.prior_hash) == error::success;
        } else if (staged_ok) {
            staged_ok = verified_copy(sources[index], staged, file.target_hash) == error::success &&
                        verified_copy(live, backup_staged, file.prior_hash) == error::success;
        }
        if (!staged_ok) {
            say("%s staging of %ls failed; live files are unchanged.", verb, component_files[index].filename);
            recover_replace(journal);
            return exit_code::failure;
        }
    }

    journal.state = JournalState::replace_commit;
    if ((result = save_journal(journal)) != error::success) {
        recover_replace(journal);
        return exit_code::failure;
    }

    bool restart = false;

    for (size_t index = 0; index < component_count; ++index) {
        auto& file = journal.files[index];

        if (!file.target) {
            continue;
        }

        Path live{};
        Path staged{};
        Path retired{};
        Path backup{};
        Path backup_staged{};
        bool live_moved = false;

        if (!live_path(index, live) || !suffixed(live, staged_suffix, staged) ||
            !suffixed(live, retired_suffix, retired) || !backup_path(index, false, backup) ||
            !backup_path(index, true, backup_staged)) {
            return recover_replace(journal);
        }
        result = ops_.move_file(ops_.context, backup_staged, backup);
        if (result == error::success && file.pending) {
            // Live already holds the target: cancel the queued rename.
            result = ops_.delete_file(ops_.context, staged);
            result = result == error::file_not_found ? error::success : result;
            if (result == error::success) {
                file.pending = false;
            }
        } else if (result == error::success) {
            result = ops_.move_file(ops_.context, live, retired);
            live_moved = result == error::success;
            if (result == error::access_denied || result == error::sharing_violation) {
                // A loaded image: the OS replaces it at the next restart.
                result = ops_.replace_on_restart(ops_.context, staged, live);
                if (result == error::success) {
                    restart = true;
                    file.pending = true;
                    file.pending_hash = file.target_hash;
                    journal.queued_boot_known = false;
                }
            } else if (result == error::success) {
                result = ops_.move_file(ops_.context, staged, live);
            }
        }
        if (result != error::success) {
            say("%s commit of %ls failed (Win32 error %lu); recovering.", verb, component_files[index].filename,
                static_cast<unsigned long>(result));
            recover_replace(journal);
            return exit_code::failure;
        }
        if (live_moved) {
            delete_or_schedule(retired, restart);
        }
    }

    journal.restart_pending = false;
    for (size_t index = 0; index < component_count; ++index) {
        auto& file = journal.files[index];
        journal.restart_pending = journal.restart_pending || file.pending;

        if (!file.target) {
            continue;
        }
        file.installed = true;
        file.installed_hash = file.target_hash;
        file.backup = true;
        file.backup_hash = file.prior_hash;
        file.target = false;
        file.prior = false;
    }
    journal.state = JournalState::installed;
    if ((result = save_journal(journal)) != error::success) {
        say("%s replaced the files but could not record it; run radeonctl recover.", verb);
        return exit_code::failure;
    }

    say("%s complete; the replaced files are kept as the backup for radeonctl rollback.", verb);
    for (size_t index = 0; index < component_count; ++index) {
        if ((mask & (1u << index)) != 0) {
            wchar_t text[sha256_hex_characters]{};
            sha256_hex(journal.files[index].installed_hash, text);
            say("  %ls sha256 %ls%s", component_files[index].filename, text,
                journal.files[index].pending ? " (in use: replaced at the next restart)" : "");
        }
    }
    say("Restart ReactOS to load the replaced driver files; none was performed.");
    return exit_code::restart_required;
}

int Installer::recover_replace(Journal& journal) noexcept {
    const char* verb = journal.operation == ReplaceKind::upgrade ? "upgrade" : "rollback";
    bool restart = false;

    if (journal.state == JournalState::replace_staging) {
        // Live files were never touched: discard this transaction's staged
        // copies only. A queued (pending) source belongs to an earlier,
        // committed replacement and is kept.
        for (size_t index = 0; index < component_count; ++index) {
            auto& file = journal.files[index];
            Path live{};
            Path staged{};
            Path backup_staged{};

            if (file.target && !file.pending && live_path(index, live) && suffixed(live, staged_suffix, staged)) {
                delete_if_present(staged);
            }
            if (file.target && backup_path(index, true, backup_staged)) {
                delete_if_present(backup_staged);
            }
            file.target = false;
            file.prior = false;
        }
        journal.state = JournalState::installed;
        say("Discarded the staged files of an interrupted %s; installed files are unchanged.", verb);
        return save_journal(journal) == error::success ? exit_code::success : exit_code::failure;
    }

    ReplaceObservation observations[component_count]{};
    bool restore = false;

    for (size_t index = 0; index < component_count; ++index) {
        const auto& file = journal.files[index];

        if (!file.target || !file.prior) {
            continue;
        }
        if (observe(index, observations[index]) != error::success) {
            return exit_code::failure;
        }
        if (file.pending) {
            // A cancellation can always go forward (delete the queued source).
            continue;
        }

        const ReplaceRecovery decision = decide_replace_recovery(observations[index], file.target_hash, file.prior_hash);

        if (decision == ReplaceRecovery::unrecoverable) {
            say("Interrupted %s: neither the new nor the previous %ls survives; reinstall with radeonctl install.",
                verb, component_files[index].filename);
            return exit_code::failure;
        }
        restore = restore || decision == ReplaceRecovery::restore_previous;
    }

    for (size_t index = 0; index < component_count; ++index) {
        auto& file = journal.files[index];

        if (!file.target || !file.prior) {
            continue;
        }

        const auto& seen = observations[index];
        Path live{};
        Path staged{};
        Path retired{};
        Path backup{};
        Path backup_staged{};

        if (!live_path(index, live) || !suffixed(live, staged_suffix, staged) ||
            !suffixed(live, retired_suffix, retired) || !backup_path(index, false, backup) ||
            !backup_path(index, true, backup_staged)) {
            return exit_code::failure;
        }

        uint32_t result = error::success;
        bool keep_staged = false;

        if (file.pending) {
            // Interrupted cancellation of a queued replacement.
            if (restore) {
                // The set returns to its prior state: the queued generation
                // must still be in place for the rename queued earlier.
                if (!(seen.staged && seen.staged_hash == file.prior_hash)) {
                    const int source = prior_source(seen, file.prior_hash);
                    const wchar_t* from = source == 3 ? backup_staged : source == 4 ? backup : nullptr;
                    result = from != nullptr ? verified_copy(from, staged, file.prior_hash) : error::file_not_found;
                }
                if (result != error::success) {
                    return exit_code::failure;
                }
                keep_staged = true;
                file.installed = true;
                file.installed_hash = file.prior_hash;
            } else {
                delete_if_present(staged);
                file.pending = false;
                file.installed = true;
                file.installed_hash = file.target_hash;
            }
        } else if (restore) {
            // Every file returns to its prior bytes so the set stays consistent.
            bool deferred = false;
            switch (prior_source(seen, file.prior_hash)) {
            case 1:
                break;
            case 2:
                result = ops_.move_file(ops_.context, retired, live);
                break;
            case 3:
                result = place_file(backup_staged, live, file.prior_hash, restart, &deferred);
                break;
            case 4:
                result = place_file(backup, live, file.prior_hash, restart, &deferred);
                break;
            default:
                result = error::file_not_found;
                break;
            }
            if (result != error::success) {
                return exit_code::failure;
            }
            if (deferred) {
                keep_staged = true;
                file.pending = true;
                file.pending_hash = file.prior_hash;
                journal.queued_boot_known = false;
            }
            file.installed = true;
            file.installed_hash = file.prior_hash;
        } else {
            if (seen.staged && seen.staged_hash == file.target_hash) {
                if (seen.live) {
                    result = ops_.move_file(ops_.context, live, retired);
                }
                if (result == error::access_denied || result == error::sharing_violation) {
                    // Still loaded: queue the rename (a repeated queue entry is
                    // harmless; the second finds no source).
                    result = ops_.replace_on_restart(ops_.context, staged, live);
                    if (result == error::success) {
                        restart = true;
                        keep_staged = true;
                        file.pending = true;
                        file.pending_hash = file.target_hash;
                        journal.queued_boot_known = false;
                    }
                } else if (result == error::success) {
                    result = ops_.move_file(ops_.context, staged, live);
                }
                if (result != error::success) {
                    return exit_code::failure;
                }
            }
            file.installed = true;
            file.installed_hash = file.target_hash;
        }

        // Keep the prior bytes as the backup when a verified copy exists.
        if (seen.backup_staged && seen.backup_staged_hash == file.prior_hash) {
            if (ops_.move_file(ops_.context, backup_staged, backup) == error::success) {
                file.backup = true;
                file.backup_hash = file.prior_hash;
            }
        } else if (seen.backup) {
            file.backup = true;
            file.backup_hash = seen.backup_hash;
        }
        if (!restore && seen.backup && seen.backup_hash == file.prior_hash) {
            file.backup = true;
            file.backup_hash = file.prior_hash;
        }

        if (!keep_staged) {
            delete_if_present(staged);
        }
        delete_if_present(retired);
        delete_if_present(backup_staged);
        file.target = false;
        file.prior = false;
    }

    journal.state = JournalState::installed;
    journal.restart_pending = false;
    for (const auto& file : journal.files) {
        journal.restart_pending = journal.restart_pending || file.pending;
    }
    if (save_journal(journal) != error::success) {
        return exit_code::failure;
    }
    say(restore ? "Restored the previous files of an interrupted %s." : "Completed an interrupted %s.", verb);
    return exit_code::restart_required;
}

int Installer::upgrade() noexcept {
    Journal journal{};

    if (load_journal(journal) != error::success || journal.state != JournalState::installed) {
        say("Upgrade refused: no completed installation is recorded (%s). Use radeonctl install.",
            journal_state_name(journal.state));
        return exit_code::refused;
    }

    // Safety decision: no upgrade on top of a replacement still queued for the
    // restart. Its live bytes are not the recorded generation yet, and a new
    // live replacement would be overwritten by the queued rename. Rollback
    // stays available to cancel the queued generation.
    for (size_t index = 0; index < component_count; ++index) {
        if (journal.files[index].pending) {
            say("Upgrade refused: %ls is replaced only at the next restart. Restart ReactOS and run radeonctl "
                "recover first (radeonctl rollback can cancel that replacement now).",
                component_files[index].filename);
            return exit_code::refused;
        }
    }

    PackageFile package[component_count]{};

    if (inspect_package(journal.features, package, true) != error::success) {
        say("The package is incomplete or built for another architecture. Nothing was changed.");
        return exit_code::failure;
    }

    Path sources[component_count]{};
    Sha256Digest targets[component_count]{};
    uint32_t mask = 0;

    for (size_t index = 0; index < component_count; ++index) {
        if (!component_selected(index, journal.features)) {
            continue;
        }

        Path live{};
        bool exists = false;
        Sha256Digest current{};

        if (!live_path(index, live) || hash_file(live, exists, current) != error::success) {
            return exit_code::failure;
        }
        if (exists && current == package[index].hash) {
            continue;
        }
        if (!package_path(component_files[index].filename, sources[index])) {
            return exit_code::failure;
        }
        targets[index] = package[index].hash;
        mask |= 1u << index;
    }

    if (mask == 0) {
        say("Installed files already equal this package; nothing to upgrade.");
        return exit_code::success;
    }
    return run_replace(ReplaceKind::upgrade, mask, sources, targets, journal);
}

int Installer::rollback() noexcept {
    Journal journal{};

    if (load_journal(journal) != error::success || journal.state != JournalState::installed) {
        say("Rollback refused: no completed installation is recorded (%s).", journal_state_name(journal.state));
        return exit_code::refused;
    }

    Path sources[component_count]{};
    Sha256Digest targets[component_count]{};
    uint32_t mask = 0;

    for (size_t index = 0; index < component_count; ++index) {
        const auto& file = journal.files[index];

        if (!component_selected(index, journal.features) || !file.backup) {
            continue;
        }

        Path backup{};
        Path live{};
        bool exists = false;
        Sha256Digest digest{};

        if (!backup_path(index, false, backup) || hash_file(backup, exists, digest) != error::success ||
            !exists || digest != file.backup_hash) {
            say("Rollback refused: backup %ls is missing or altered.", component_files[index].filename);
            return exit_code::refused;
        }
        if (!live_path(index, live) || hash_file(live, exists, digest) != error::success) {
            return exit_code::failure;
        }

        // Compare the generation that will be live after the next restart
        // (the queued one when a replacement is pending), not the live bytes:
        // a deferred file still shows its old content until then.
        const bool effective_known = file.pending || exists;
        const Sha256Digest effective = file.pending ? file.pending_hash : digest;
        if (effective_known && effective == file.backup_hash) {
            continue;
        }
        text_copy(sources[index], backup);
        targets[index] = file.backup_hash;
        mask |= 1u << index;
    }

    if (mask == 0) {
        say("No backup differs from the installed files; nothing to roll back.");
        return exit_code::success;
    }
    return run_replace(ReplaceKind::rollback, mask, sources, targets, journal);
}

// DIF_REMOVE deletes the devnode. ReactOS 0.4.16 refuses it with
// ERROR_CALL_NOT_IMPLEMENTED before any change (no DIF_REMOVE class-install
// parameter handler; SetupDiRemoveDevice is a stub as well); there the binding
// is cleared in place instead, which the next boot turns into an install of
// ReactOS's default driver on the same devnode. failed stays none for a
// DIF_REMOVE error and names the in-place step otherwise.
uint32_t Installer::remove_binding(const wchar_t* instance, bool& in_place, UnbindStep& failed) noexcept {
    in_place = false;
    failed = UnbindStep::none;
    const uint32_t removed = ops_.remove_device(ops_.context, instance);

    if (removed != error::call_not_implemented) {
        return removed;
    }
    in_place = true;
    return ops_.unbind_device(ops_.context, instance, failed);
}

// SetupUninstallOEMInf without SUOI_FORCEDELETE deletes the copy only when no
// device uses it and otherwise fails (Windows); that refusal is respected.
// ReactOS reports success and deletes nothing, so its next-boot driver search
// would select this package's INF again: a copy that survives a successful
// call is force-deleted unless a present device's driver key still names it.
// Returns an error only when an unused copy cannot be deleted.
uint32_t Installer::remove_oem_inf(const wchar_t* inf_name) noexcept {
    Path copy{};
    bool exists = false;
    uint64_t bytes = 0;

    if (!oem_inf_name(inf_name)) {
        return error::success;  // not a SetupAPI-staged copy; nothing of ours to delete
    }
    if (!path_join(copy, ops_.inf_directory, inf_name)) {
        return error::invalid_parameter;
    }

    uint32_t result = ops_.file_size(ops_.context, copy, exists, bytes);
    if (result != error::success || !exists) {
        return result;  // never staged, or removed by an earlier interrupted run
    }
    const uint32_t unforced = ops_.uninstall_oem_inf(ops_.context, inf_name, false);
    result = ops_.file_size(ops_.context, copy, exists, bytes);
    if (result == error::success && exists && unforced != error::success) {
        say("SetupAPI kept the staged driver package %ls (Win32 error %lu).", inf_name,
            static_cast<unsigned long>(unforced));
        return error::success;
    }
    if (result == error::success && exists) {
        if ((result = inventory()) != error::success) {
            return result;
        }
        for (size_t index = 0; index < inventory_.count; ++index) {
            wchar_t used[path_characters]{};
            if (ops_.driver_value(ops_.context, inventory_.devices[index].instance_id, L"InfPath", used,
                                  path_characters) == error::success &&
                text_equal_insensitive(used, inf_name)) {
                say("The staged driver package %ls is still used by %ls; it was kept.", inf_name,
                    inventory_.devices[index].instance_id);
                return error::success;
            }
        }
        result = ops_.uninstall_oem_inf(ops_.context, inf_name, true);
        if (result == error::success) {
            result = ops_.file_size(ops_.context, copy, exists, bytes);
        }
        if (result == error::success && exists) {
            result = error::access_denied;
        }
    }
    if (result != error::success) {
        say("Deleting the staged driver package %ls failed (Win32 error %lu); the journal is retained.", copy,
            static_cast<unsigned long>(result));
        return result;
    }
    say("Removed the staged driver package %ls.", inf_name);
    return error::success;
}

int Installer::perform_remove(Journal& journal) noexcept {
    PackageFile package[component_count]{};
    inspect_package(known_features, package, false);
    bool restart = false;

    if (inventory() != error::success) {
        say("Device enumeration failed; removal stopped before any change.");
        return exit_code::failure;
    }

    const DeviceRecord* device = find_device(journal.instance);

    if (device == nullptr && journal.instance[0] == 0) {
        device = find_device(nullptr);
    }

    const bool bound = device != nullptr && text_equal_insensitive(device->service, service_name);

    if (journal.state == JournalState::none && !bound) {
        // Nothing recorded and nothing bound: remove only what exists, and
        // write no journal when that is nothing at all.
        ServiceInfo service{};
        bool registered = false;
        const bool icd = icd_registration_ours(registered) && registered;
        bool files = false;
        for (size_t index = 0; index < component_count; ++index) {
            Path live{};
            bool exists = false;
            Sha256Digest current{};
            files = files || (live_path(index, live) && hash_file(live, exists, current) == error::success &&
                              exists && package[index].present && current == package[index].hash);
        }
        if (ops_.query_service(ops_.context, service) == error::success && !service.exists && !icd && !files &&
            !key_present(journal_key)) {
            say("Nothing of this package is installed; no change was made.");
            return exit_code::success;
        }
    }

    if (journal.state != JournalState::removing) {
        if (device != nullptr) {
            text_copy(journal.instance, device->instance_id);
        }
        if (bound) {
            // The device key holding these filters is deleted with the binding.
            journal.snapshot = true;
            for (size_t index = 0; index < device->upper_characters; ++index) {
                journal.upper[index] = device->upper_filters[index];
            }
            journal.upper_characters = device->upper_characters;
            for (size_t index = 0; index < device->lower_characters; ++index) {
                journal.lower[index] = device->lower_filters[index];
            }
            journal.lower_characters = device->lower_characters;
            wchar_t inf_path[path_characters]{};
            if (ops_.driver_value(ops_.context, device->instance_id, L"InfPath", inf_path, path_characters) ==
                error::success) {
                text_copy(journal.inf_name, inf_path);
            }
        }
        journal.state = JournalState::removing;
        if (save_journal(journal) != error::success) {
            say("Recording the removal journal failed. Nothing was changed.");
            return exit_code::failure;
        }
    }

    bool unbound_in_place = false;
    if (bound) {
        UnbindStep failed = UnbindStep::none;
        const uint32_t result = remove_binding(device->instance_id, unbound_in_place, failed);
        if (result != error::success) {
            say("Removing the rx7900video device binding failed (%s%s, Win32 error %lu); the journal is retained.",
                failed == UnbindStep::none ? "DIF_REMOVE" : "in-place unbinding, ",
                failed == UnbindStep::none ? "" : rx7900::unbind_step_description(failed),
                static_cast<unsigned long>(result));
            return exit_code::failure;
        }
        restart = true;
        say("Removed the rx7900video binding of %ls.", journal.instance);
        if (unbound_in_place) {
            say("  SetupAPI device removal is not implemented here; the device keeps its instance and filters, "
                "and its driver key and binding values were deleted instead.");
        }
    } else if (device != nullptr && device->service[0] != 0) {
        say("The GPU is bound to %ls, not rx7900video; that binding was preserved.", device->service);
    }

    bool service_removed = false;
    if (remove_owned_service(service_removed) != error::success) {
        say("Deleting the rx7900video service failed; the journal is retained.");
        return exit_code::failure;
    }
    if (service_removed) {
        say("Deleted the rx7900video service.");
        restart = true;
    }

    bool registered = false;
    if (icd_registration_ours(registered) && registered) {
        if (ops_.delete_key(ops_.context, opengl_drivers_key) != error::success) {
            return exit_code::failure;
        }
        say("Removed the OpenGLDrivers\\RadeonGL registration.");
    } else if (registered) {
        say("OpenGLDrivers\\RadeonGL names another DLL; it was preserved.");
    }

    for (size_t index = 0; index < component_count; ++index) {
        const auto& file = journal.files[index];
        Path live{};
        bool exists = false;
        Sha256Digest current{};

        if (!live_path(index, live) || hash_file(live, exists, current) != error::success) {
            return exit_code::failure;
        }
        if (exists) {
            const bool owned = (file.installed && current == file.installed_hash) ||
                               (file.backup && current == file.backup_hash) ||
                               (package[index].present && current == package[index].hash);
            if (owned) {
                if (delete_or_schedule(live, restart) != error::success) {
                    return exit_code::failure;
                }
                say("Deleted %ls.", live);
            } else {
                say("Preserved %ls: its contents are not a recorded or packaged version.", live);
            }
        }

        Path extra{};
        if (suffixed(live, staged_suffix, extra)) {
            delete_if_present(extra);
        }
        if (suffixed(live, retired_suffix, extra)) {
            delete_or_schedule(extra, restart);
        }
        if (backup_path(index, false, extra)) {
            delete_if_present(extra);
        }
        if (backup_path(index, true, extra)) {
            delete_if_present(extra);
        }
    }

    // Never delete Device0 values of another service; ours went with the service key.
    if (journal.inf_name[0] != 0 && remove_oem_inf(journal.inf_name) != error::success) {
        return exit_code::failure;
    }

    // An in-place unbinding keeps the devnode and with it the filter values.
    const bool filters = journal.snapshot && !unbound_in_place &&
                         (journal.upper_characters > 2 || journal.lower_characters > 2);

    if (filters) {
        Journal pending{};
        pending.state = JournalState::filters_pending;
        text_copy(pending.instance, journal.instance);
        pending.snapshot = true;
        for (size_t index = 0; index < journal.upper_characters; ++index) {
            pending.upper[index] = journal.upper[index];
        }
        pending.upper_characters = journal.upper_characters;
        for (size_t index = 0; index < journal.lower_characters; ++index) {
            pending.lower[index] = journal.lower[index];
        }
        pending.lower_characters = journal.lower_characters;
        // Overwrite in place: State moves from removing straight to
        // filters_pending, so the snapshot is never left without a state.
        if (save_journal(pending) != error::success) {
            return exit_code::failure;
        }
        say("Other drivers' device filters were saved; after ReactOS re-detects the GPU, run radeonctl recover.");
    } else if (clear_journal() != error::success) {
        return exit_code::failure;
    }

    say("Firmware below %ls was retained (administrator-supplied).", firmware_);
    if (restart) {
        say("Restart ReactOS to complete the removal; none was performed. ReactOS then re-detects the GPU with its default driver.");
        return exit_code::restart_required;
    }
    return exit_code::success;
}

int Installer::remove() noexcept {
    Journal journal{};

    if (load_journal(journal) != error::success) {
        say("The installer journal is unreadable; run radeonctl status.");
        return exit_code::failure;
    }
    if (journal.state == JournalState::replace_staging || journal.state == JournalState::replace_commit) {
        const int recovered = recover_replace(journal);
        if (recovered == exit_code::failure) {
            return recovered;
        }
    }
    if (journal.state == JournalState::installing) {
        const int recovered = recover_installing(journal);
        if (recovered == exit_code::failure) {
            return recovered;
        }
        load_journal(journal);
    }
    if (journal.state == JournalState::filters_pending) {
        say("The driver is already removed; preserved filters await re-detection (radeonctl recover).");
        return exit_code::success;
    }
    return perform_remove(journal);
}

int Installer::restore_pending_filters(Journal& journal, bool explicit_command) noexcept {
    if (inventory() != error::success) {
        return exit_code::failure;
    }

    const DeviceRecord* device = find_device(journal.instance);

    if (device == nullptr) {
        if (explicit_command) {
            say("The GPU %ls has not been re-detected yet; restart ReactOS, then run radeonctl recover.",
                journal.instance);
        }
        return exit_code::refused;
    }

    size_t restored = 0;
    const uint32_t result = restore_filters(*device, journal, restored);

    if (result != error::success) {
        say("Restoring preserved device filters failed (Win32 error %lu).", static_cast<unsigned long>(result));
        return exit_code::failure;
    }
    say("Restored %lu preserved device filter entr%s on %ls.", static_cast<unsigned long>(restored),
        restored == 1 ? "y" : "ies", device->instance_id);
    return clear_journal() == error::success ? (restored != 0 ? exit_code::restart_required : exit_code::success)
                                             : exit_code::failure;
}

// The installed files must form one recorded generation as a whole. Each
// component is compared with its recorded installed hash; a component whose
// replacement is still queued (pending, queued copy intact) is waiting for the
// restart. Anything else makes the set mixed: it is completed to the recorded
// generation from verified copies (package, queued copy, backup); when that is
// impossible and the live set is not wholly the backup generation, the whole
// set is restored to the verified backup generation instead. A live set that
// is wholly the backup generation is adopted as installed (no file is written).
int Installer::repair_generation(Journal& journal, bool explicit_command, bool& restart) noexcept {
    PackageFile package[component_count]{};
    inspect_package(journal.features, package, false);

    struct Seen final {
        bool selected = false;
        bool live = false;
        Sha256Digest live_hash{};
        bool staged = false;
        Sha256Digest staged_hash{};
        bool backup = false;  // backup file present and equal to the record
    };
    Seen seen[component_count]{};
    // A queued copy is still waiting only within the boot that queued it. An
    // unknown boot identity is treated as waiting (no write without evidence).
    uint64_t boot = 0;
    const bool same_boot = !journal.queued_boot_known || ops_.boot_id(ops_.context, boot) != error::success ||
                           boot == journal.queued_boot;
    bool mismatch = false;
    bool waiting = false;
    bool changed = false;
    bool all_backup = true;

    for (size_t index = 0; index < component_count; ++index) {
        auto& file = journal.files[index];
        auto& item = seen[index];
        Path live{};
        Path staged{};
        Path backup{};
        Sha256Digest backup_hash{};
        bool backup_present = false;

        if (!component_selected(index, journal.features) || !file.installed) {
            if (file.pending) {
                file.pending = false;
                changed = true;
            }
            continue;
        }
        item.selected = true;
        if (!live_path(index, live) || !suffixed(live, staged_suffix, staged) || !backup_path(index, false, backup) ||
            hash_file(live, item.live, item.live_hash) != error::success ||
            hash_file(staged, item.staged, item.staged_hash) != error::success ||
            hash_file(backup, backup_present, backup_hash) != error::success) {
            return exit_code::failure;
        }
        item.backup = file.backup && backup_present && backup_hash == file.backup_hash;
        all_backup = all_backup && item.backup && item.live && item.live_hash == file.backup_hash;

        if (item.live && item.live_hash == file.installed_hash) {
            if (file.pending) {
                file.pending = false;  // the queued replacement happened
                changed = true;
                delete_if_present(staged);
            }
            continue;
        }
        if (same_boot && file.pending && item.staged && item.staged_hash == file.installed_hash) {
            waiting = true;
            continue;
        }
        if (!same_boot && file.pending) {
            say("The replacement of %ls queued before the last restart did not happen.", live);
        }
        mismatch = true;
    }

    if (mismatch && all_backup) {
        // Wholly the backup generation (for example every queued rename was
        // lost): adopt it as installed and keep the record's set as the backup
        // only where those bytes still exist.
        for (size_t index = 0; index < component_count; ++index) {
            auto& file = journal.files[index];
            if (!seen[index].selected) {
                continue;
            }
            const Sha256Digest previous = file.installed_hash;
            const bool previous_present = (seen[index].staged && seen[index].staged_hash == previous) ||
                                          (package[index].present && package[index].hash == previous);
            file.installed_hash = file.backup_hash;
            file.backup = false;
            if (previous_present) {
                Path backup{};
                Path from{};
                Path live{};
                Path staged{};
                if (backup_path(index, false, backup) && live_path(index, live) && suffixed(live, staged_suffix, staged) &&
                    (seen[index].staged && seen[index].staged_hash == previous ?
                         text_copy(from, staged) : package_path(component_files[index].filename, from)) &&
                    verified_copy(from, backup, previous) == error::success) {
                    file.backup = true;
                    file.backup_hash = previous;
                }
            }
            if (file.pending) {
                Path live{};
                Path staged{};
                if (live_path(index, live) && suffixed(live, staged_suffix, staged)) {
                    delete_if_present(staged);
                }
                file.pending = false;
            }
        }
        say("The installed files are wholly the backup generation; recorded it as installed.");
        changed = true;
        mismatch = false;
    }

    if (mismatch) {
        // Complete the recorded generation when every missing file has a copy.
        Path sources[component_count]{};
        bool forward = true;

        for (size_t index = 0; index < component_count; ++index) {
            const auto& file = journal.files[index];
            const auto& item = seen[index];
            Path live{};
            Path staged{};

            if (!item.selected || (item.live && item.live_hash == file.installed_hash) ||
                (same_boot && file.pending && item.staged && item.staged_hash == file.installed_hash)) {
                continue;
            }
            if (!live_path(index, live) || !suffixed(live, staged_suffix, staged)) {
                return exit_code::failure;
            }
            if (package[index].present && package[index].hash == file.installed_hash) {
                package_path(component_files[index].filename, sources[index]);
            } else if (item.staged && item.staged_hash == file.installed_hash) {
                text_copy(sources[index], staged);
            } else if (item.backup && file.backup_hash == file.installed_hash) {
                backup_path(index, false, sources[index]);
            } else {
                forward = false;
            }
        }

        bool restore_backup = false;
        if (!forward) {
            restore_backup = true;
            for (size_t index = 0; index < component_count; ++index) {
                restore_backup = restore_backup && (!seen[index].selected || seen[index].backup);
            }
            if (!restore_backup) {
                say("The installed files are a mixed set and neither the recorded nor the backup generation can be "
                    "completed; run radeonctl upgrade from the recorded package.");
                return exit_code::failure;
            }
        }

        for (size_t index = 0; index < component_count; ++index) {
            auto& file = journal.files[index];
            const auto& item = seen[index];
            const Sha256Digest want = restore_backup ? file.backup_hash : file.installed_hash;
            Path live{};
            Path staged{};
            Path from{};

            if (!item.selected || !live_path(index, live) || !suffixed(live, staged_suffix, staged)) {
                continue;
            }
            if (restore_backup && file.pending) {
                // The queued generation is superseded: cancel its rename.
                delete_if_present(staged);
                file.pending = false;
            }
            if (item.live && item.live_hash == want) {
                continue;
            }
            if (!restore_backup && same_boot && file.pending && item.staged && item.staged_hash == want) {
                continue;
            }
            if (restore_backup ? !backup_path(index, false, from) : !text_copy(from, sources[index])) {
                return exit_code::failure;
            }
            // place_file stages through <live>.radeonctl-new; copy a queued
            // source aside first so staging does not consume it.
            Path aside{};
            if (text_equal_insensitive(from, staged)) {
                if (!suffixed(live, L".radeonctl-repair", aside) || verified_copy(from, aside, want) != error::success) {
                    return exit_code::failure;
                }
                text_copy(from, aside);
            }
            bool deferred = false;
            const uint32_t result = place_file(from, live, want, restart, &deferred);
            if (aside[0] != 0) {
                delete_if_present(aside);
            }
            if (result != error::success) {
                say("Restoring %ls failed (Win32 error %lu).", live, static_cast<unsigned long>(result));
                return exit_code::failure;
            }
            file.pending = deferred;
            file.pending_hash = want;
            journal.queued_boot_known = journal.queued_boot_known && !deferred;
            say(deferred ? "Queued %ls for replacement at the next restart." : "Restored %ls.", live);
            restart = true;
        }
        if (restore_backup) {
            for (size_t index = 0; index < component_count; ++index) {
                auto& file = journal.files[index];
                if (seen[index].selected) {
                    file.installed_hash = file.backup_hash;
                    file.backup = false;  // the incomplete generation is not a valid backup
                }
            }
            say("The installed files were a mixed set; restored the complete backup generation.");
        }
        changed = true;
    }

    const bool pending_before = journal.restart_pending;
    journal.restart_pending = false;
    for (const auto& file : journal.files) {
        journal.restart_pending = journal.restart_pending || file.pending;
    }
    if ((changed || pending_before != journal.restart_pending) && save_journal(journal) != error::success) {
        return exit_code::failure;
    }
    if (waiting && explicit_command) {
        say("A file replacement is pending the next ReactOS restart.");
    }
    return exit_code::success;
}

int Installer::repair_installed(Journal& journal, bool explicit_command) noexcept {
    if (inventory() != error::success) {
        return exit_code::failure;
    }

    const DeviceRecord* device = find_device(journal.instance);
    const bool bound = device != nullptr && text_equal_insensitive(device->service, service_name);
    int code = exit_code::success;
    bool restart = false;

    // Files first: they are repaired whether or not the GPU is present.
    if (repair_generation(journal, explicit_command, restart) != exit_code::success) {
        return exit_code::failure;
    }

    if (!bound) {
        if (explicit_command) {
            say(device == nullptr ? "The recorded GPU is not present; the installation is kept unchanged." :
                                    "The recorded GPU is no longer bound to rx7900video; run radeonctl install or remove.");
        }
        return exit_code::success;
    }

    bool service_changed = false;
    if (ensure_service(service_changed) != error::success) {
        return exit_code::failure;
    }
    restart = restart || service_changed;

    // Device0 values and opt-ins re-applied from the journal (idempotent).
    if (apply_display_values() != error::success || apply_feature_values(journal.features) != error::success) {
        return exit_code::failure;
    }

    if ((journal.features & feature_opengl_icd) != 0) {
        bool registered = false;
        bool version = false;
        uint32_t value = 0;
        const bool ours = icd_registration_ours(registered);
        const bool current = ours && read_dword(opengl_drivers_key, L"DriverVersion", version, value) == error::success &&
                             version && value == opengl::driver_version;
        if (!current) {
            if (write_string(opengl_drivers_key, L"Dll", component_files[icd_component].filename) != error::success ||
                write_dword(opengl_drivers_key, L"Version", opengl::interface_version) != error::success ||
                write_dword(opengl_drivers_key, L"DriverVersion", opengl::driver_version) != error::success ||
                write_dword(opengl_drivers_key, L"Flags", 0) != error::success) {
                return exit_code::failure;
            }
            say("Repaired the missing or altered OpenGLDrivers\\RadeonGL registration.");
        }
    }

    if (explicit_command && code == exit_code::success) {
        say("Installation verified: service, files, Device0 opt-ins%s.",
            (journal.features & feature_opengl_icd) != 0 ? " and ICD registration" : "");
    }
    return code == exit_code::success && restart ? exit_code::restart_required : code;
}

int Installer::recover_orphans(bool explicit_command) noexcept {
    if (inventory() != error::success) {
        return exit_code::failure;
    }

    // Values without a State are a journal interrupted before its commit
    // point: nothing it describes was started.
    if (key_present(journal_key)) {
        if (clear_journal() != error::success) {
            return exit_code::failure;
        }
        say("Removed an incomplete installer journal; no change it described had started.");
    }

    const DeviceRecord* device = find_device(nullptr);
    const bool bound = device != nullptr && text_equal_insensitive(device->service, service_name);
    ServiceInfo service{};

    if (ops_.query_service(ops_.context, service) != error::success) {
        return exit_code::failure;
    }
    if (service.exists && !bound) {
        if (service.running) {
            say("A stale rx7900video service is running; it was left alone. Restart, then run radeonctl recover.");
        } else {
            bool removed = false;
            if (remove_owned_service(removed) != error::success) {
                return exit_code::failure;
            }
            if (removed) {
                say("Deleted a stale rx7900video service that no device uses.");
            }
        }
    } else if (bound && explicit_command) {
        say("The GPU is bound to rx7900video without an installer record; run radeonctl install to adopt it.");
    }

    bool registered = false;
    const bool ours = icd_registration_ours(registered);
    Path icd{};
    bool exists = false;
    uint64_t bytes = 0;

    if (registered && ours && live_path(icd_component, icd) &&
        ops_.file_size(ops_.context, icd, exists, bytes) == error::success && !exists) {
        if (ops_.delete_key(ops_.context, opengl_drivers_key) != error::success) {
            return exit_code::failure;
        }
        say("Removed a RadeonGL registration whose radeonglicd.dll is missing.");
    }
    if (explicit_command) {
        say("No installation is recorded; nothing else to recover.");
    }
    return exit_code::success;
}

int Installer::recover(bool explicit_command) noexcept {
    Journal journal{};

    if (load_journal(journal) != error::success) {
        say("The installer journal is unreadable; inspect HKLM\\%ls.", journal_key);
        return exit_code::failure;
    }

    switch (journal.state) {
    case JournalState::installing:
        return recover_installing(journal);
    case JournalState::replace_staging:
    case JournalState::replace_commit:
        return recover_replace(journal);
    case JournalState::removing:
        say("Completing an interrupted removal.");
        return perform_remove(journal);
    case JournalState::filters_pending:
        return restore_pending_filters(journal, explicit_command);
    case JournalState::installed:
        return repair_installed(journal, explicit_command);
    case JournalState::none:
        return recover_orphans(explicit_command);
    }
    return exit_code::failure;
}

int Installer::recover_interrupted() noexcept {
    Journal journal{};

    if (load_journal(journal) != error::success) {
        say("The installer journal is unreadable; inspect HKLM\\%ls.", journal_key);
        return exit_code::failure;
    }
    return journal_state_transitional(journal.state) ? recover(false) : exit_code::success;
}

bool Installer::firmware_ready(uint16_t device_id, uint8_t revision, bool report) noexcept {
    FirmwareEntry entries[firmware_entry_capacity]{};
    const size_t count = firmware_manifest(revision, entries);
    bool ready = true;

    for (size_t index = 0; index < count; ++index) {
        const auto& entry = entries[index];

        if (entry.kind == FirmwareKind::atom_rom) {
            continue;
        }

        const char* names[]{entry.name, entry.alternate};
        FirmwareVerdict verdict = FirmwareVerdict::too_small;
        bool found = false;
        const char* used = entry.name;

        for (const char* name : names) {
            if (name[0] == 0 || (found && verdict == FirmwareVerdict::valid)) {
                continue;
            }

            wchar_t wide[48]{};
            Path path{};
            HeapBuffer buffer;
            bool exists = false;
            bool oversize = false;

            if (!text_append_ascii(wide, name) || !path_join(path, firmware_, wide) ||
                read_whole(path, firmware_maximum_bytes(entry), buffer, exists, oversize) != error::success ||
                !exists) {
                continue;
            }
            found = true;
            used = name;
            verdict = oversize ? FirmwareVerdict::too_large :
                                 validate_firmware_bytes(entry, buffer.view(), device_id);
        }

        if (report) {
            say("  %-24s %s%s%s%s%s", used, entry.required ? "required " : "optional ",
                found ? firmware_verdict_name(verdict) : "missing",
                entry.alternate[0] != 0 && !found ? " (" : "",
                entry.alternate[0] != 0 && !found ? entry.alternate : "",
                entry.alternate[0] != 0 && !found ? " is also accepted)" : "");
        }
        if (entry.required && (!found || verdict != FirmwareVerdict::valid)) {
            ready = false;
        }
    }
    return ready;
}

bool Installer::rom_ready(uint16_t device_id) noexcept {
    FirmwareEntry entries[firmware_entry_capacity]{};
    const size_t count = firmware_manifest(0, entries);

    for (size_t index = 0; index < count; ++index) {
        if (entries[index].kind != FirmwareKind::atom_rom) {
            continue;
        }

        wchar_t wide[48]{};
        Path path{};
        HeapBuffer buffer;
        bool exists = false;
        bool oversize = false;

        if (!text_append_ascii(wide, entries[index].name) || !path_join(path, firmware_, wide) ||
            read_whole(path, firmware_maximum_bytes(entries[index]), buffer, exists, oversize) != error::success ||
            !exists || oversize) {
            return false;
        }
        return validate_firmware_bytes(entries[index], buffer.view(), device_id) == FirmwareVerdict::valid;
    }
    return false;
}

void Installer::status_package() noexcept {
    PackageFile package[component_count]{};
    inspect_package(known_features, package, false);
    say("Package %ls (radeonctl %ls, %s):", ops_.package_directory, package_version,
        ops_.machine == machine_amd64 ? "amd64" : "i386");

    for (size_t index = 0; index < component_count; ++index) {
        const auto& file = package[index];
        if (!file.present) {
            say("  %-18ls missing", component_files[index].filename);
            continue;
        }
        wchar_t text[sha256_hex_characters]{};
        sha256_hex(file.hash, text);
        say("  %-18ls %8llu bytes %s sha256 %ls", component_files[index].filename,
            static_cast<unsigned long long>(file.bytes), pe_status_name(file.image), text);
    }
}

void Installer::status_devices() noexcept {
    const uint32_t result = inventory();

    say("Devices:");
    if (result != error::success) {
        say("  enumeration failed (Win32 error %lu)", static_cast<unsigned long>(result));
        return;
    }
    if (inventory_.count == 0) {
        say("  No Navi31 GPU (1002:7448/744A/744B/744C/745E) is present.");
        if (inventory_.lightning_present) {
            say("  QEMU Lightning 1234:11F1 is present; this package never binds it.");
        }
        say("  Expected in a VM without VFIO passthrough of the RX 7900.");
    }
    for (size_t index = 0; index < inventory_.count; ++index) {
        const auto& device = inventory_.devices[index];
        const Binding binding = decide_binding(device, nullptr);
        say("  %ls 1002:%04X rev %02X: %s%s%ls%s, %s, problem %lu", device.instance_id,
            static_cast<unsigned>(device.device_id), static_cast<unsigned>(device.revision),
            binding_name(binding), device.service[0] != 0 ? " (" : "", device.service,
            device.service[0] != 0 ? ")" : "", device.started ? "started" : "not started",
            static_cast<unsigned long>(device.problem));
    }
    if (inventory_.omitted != 0) {
        say("  %lu further Navi31 functions were not listed.", static_cast<unsigned long>(inventory_.omitted));
    }
    if (inventory_.absent != 0) {
        say("  Ignored %lu recorded Navi31 device(s) that are not attached (stale registry entries).",
            static_cast<unsigned long>(inventory_.absent));
    }
}

void Installer::status_service() noexcept {
    ServiceInfo service{};
    const uint32_t result = ops_.query_service(ops_.context, service);

    if (result != error::success) {
        say("Service rx7900video: query failed (Win32 error %lu)", static_cast<unsigned long>(result));
        return;
    }
    if (!service.exists) {
        say("Service rx7900video: absent");
        return;
    }

    bool used = false;
    for (size_t index = 0; index < inventory_.count; ++index) {
        used = used || text_equal_insensitive(inventory_.devices[index].service, service_name);
    }
    say("Service rx7900video: type %lu start %lu%s error %lu %s, image %ls%s%s",
        static_cast<unsigned long>(service.type), static_cast<unsigned long>(service.start),
        service.start == service_start_demand ? " (demand)" : " (NOT demand start)",
        static_cast<unsigned long>(service.error_control), service.running ? "running" : "stopped",
        service.image_path, service_image_matches(service.image_path, ops_.system_directory) ? "" : " (foreign image)",
        used ? "" : " - stale: no present device uses it");
}

void Installer::status_files(const Journal& journal) noexcept {
    PackageFile package[component_count]{};
    inspect_package(known_features, package, false);
    say("Installed files:");

    for (size_t index = 0; index < component_count; ++index) {
        Path live{};
        bool exists = false;
        uint64_t bytes = 0;
        Sha256Digest current{};

        if (!live_path(index, live) || ops_.file_size(ops_.context, live, exists, bytes) != error::success) {
            continue;
        }
        if (!exists) {
            say("  %ls absent", live);
            continue;
        }

        PeStatus image = PeStatus::not_pe;
        bool hashed = false;
        hash_file(live, hashed, current);
        inspect_image(live, component_files[index].image, bytes, image);
        wchar_t text[sha256_hex_characters]{};
        sha256_hex(current, text);
        const auto& file = journal.files[index];
        const char* relation = package[index].present && current == package[index].hash ? "equals this package" :
                               file.installed && current == file.installed_hash ? "recorded version" :
                               file.backup && current == file.backup_hash ? "backup version" : "unrecorded version";
        say("  %ls %llu bytes %s, %s, sha256 %ls", live, static_cast<unsigned long long>(bytes),
            pe_status_name(image), relation, text);

        Path backup{};
        if (backup_path(index, false, backup) && hash_file(backup, exists, current) == error::success && exists) {
            sha256_hex(current, text);
            say("    backup %ls sha256 %ls", backup, text);
        }
    }
}

void Installer::status_registry(const Journal& journal) noexcept {
    const wchar_t* effective = parameters_key();
    say("Miniport opt-ins (VideoPort reads HKLM\\%ls):", effective);

    uint32_t enabled = 0;
    for (const auto& rule : feature_rules) {
        if (rule.registry_value == nullptr) {
            continue;
        }
        bool present = false;
        uint32_t value = 0;
        const uint32_t result = read_dword(effective, rule.registry_value, present, value);
        if (result != error::success) {
            say("  %-18ls unreadable (Win32 error %lu)", rule.registry_value, static_cast<unsigned long>(result));
            continue;
        }
        say("  %-18ls %s%s", rule.registry_value, !present ? "absent (off)" : value == 1 ? "1 (on)" : "off",
            present && value != 0 && value != 1 ? " (non-1 value is off)" : "");
        enabled |= present && value == 1 ? rule.feature : 0;
    }

    uint32_t requirement = 0;
    const uint32_t invalid = invalid_feature(enabled, requirement);
    for (const auto& rule : feature_rules) {
        if (rule.feature == invalid) {
            for (const auto& needed : feature_rules) {
                if (needed.feature == requirement) {
                    say("  Warning: the %ls opt-in is on without %ls; the miniport ignores it.", rule.option,
                        needed.option);
                }
            }
        }
    }
    if (journal.state == JournalState::installed &&
        (enabled & ~feature_opengl_icd) != (journal.features & ~feature_opengl_icd)) {
        say("  Opt-ins differ from the recorded install (0x%02lX); radeonctl recover re-applies the record.",
            static_cast<unsigned long>(journal.features));
    }

    wchar_t drivers[filter_characters]{};
    size_t characters = 0;
    bool present = false;
    read_multi(device0_key, L"InstalledDisplayDrivers", present, drivers, characters);
    say("  InstalledDisplayDrivers: %ls", present ? drivers : L"(absent)");
}

void Installer::status_icd(const Journal& journal) noexcept {
    bool registered = false;
    const bool ours = icd_registration_ours(registered);
    wchar_t dll[path_characters]{};
    bool present = false;
    read_string(opengl_drivers_key, L"Dll", present, dll);
    bool version_present = false;
    bool driver_present = false;
    uint32_t version = 0;
    uint32_t driver_version = 0;
    read_dword(opengl_drivers_key, L"Version", version_present, version);
    read_dword(opengl_drivers_key, L"DriverVersion", driver_present, driver_version);

    if (!registered) {
        say("OpenGL ICD: OpenGLDrivers\\RadeonGL absent%s",
            (journal.features & feature_opengl_icd) != 0 ? " - MISSING; radeonctl recover repairs it" : "");
        return;
    }

    Path file{};
    bool exists = false;
    uint64_t bytes = 0;
    live_path(icd_component, file);
    ops_.file_size(ops_.context, file, exists, bytes);
    const bool versions = version_present && version == opengl::interface_version && driver_present &&
                          driver_version == opengl::driver_version;
    say("OpenGL ICD: RadeonGL Dll=%ls Version=%lu DriverVersion=%lu%s%s%s", dll,
        static_cast<unsigned long>(version), static_cast<unsigned long>(driver_version),
        ours ? "" : " (not this package's DLL)", versions ? "" : " (radeondisp reports Version=1 DriverVersion=1)",
        ours && !exists ? " - radeonglicd.dll missing" : "");
}

void Installer::status_firmware() noexcept {
    const DeviceRecord* device = find_device(nullptr);
    say("Firmware below %ls (read only with PhysicalAsicInit=1):", firmware_);
    firmware_ready(device != nullptr ? device->device_id : 0, device != nullptr ? device->revision : 0, true);

    FirmwareEntry entries[firmware_entry_capacity]{};
    const size_t count = firmware_manifest(0, entries);
    for (size_t index = 0; index < count; ++index) {
        if (entries[index].kind != FirmwareKind::atom_rom) {
            continue;
        }
        wchar_t wide[48]{};
        Path path{};
        HeapBuffer buffer;
        bool exists = false;
        bool oversize = false;
        text_append_ascii(wide, entries[index].name);
        path_join(path, firmware_, wide);
        read_whole(path, firmware_maximum_bytes(entries[index]), buffer, exists, oversize);
        const FirmwareVerdict verdict = !exists ? FirmwareVerdict::too_small :
                                        oversize ? FirmwareVerdict::too_large :
                                        validate_firmware_bytes(entries[index], buffer.view(),
                                                                device != nullptr ? device->device_id : 0);
        say("  %-24s optional %s (required only with AtomRomOverride=1)", entries[index].name,
            exists ? firmware_verdict_name(verdict) : "missing");
    }
}

void Installer::status_win32k() noexcept {
    Path path{};
    bool exists = false;
    uint64_t bytes = 0;

    if (!path_join(path, ops_.system_directory, L"win32k.sys") ||
        ops_.file_size(ops_.context, path, exists, bytes) != error::success || !exists) {
        say("win32k: %ls not readable; optional patch capability unknown.", path);
        return;
    }

    Win32kScanner scanner;
    Sha256 hasher;
    uint8_t chunk[hash_chunk_bytes];
    uint64_t offset = 0;

    while (offset < bytes) {
        uint32_t received = 0;
        const uint64_t remaining = bytes - offset;
        const uint32_t request = remaining < sizeof(chunk) ? static_cast<uint32_t>(remaining) : sizeof(chunk);
        if (ops_.read_file(ops_.context, path, offset, chunk, request, received) != error::success || received == 0 ||
            received > request) {
            say("win32k: read failed at offset %llu.", static_cast<unsigned long long>(offset));
            return;
        }
        scanner.feed(ByteView(chunk, received));
        hasher.update(chunk, received);
        offset += received;
    }

    wchar_t text[sha256_hex_characters]{};
    sha256_hex(hasher.finish(), text);
    say("win32k: %ls %llu bytes sha256 %ls (on-disk image; read only, never replaced)", path,
        static_cast<unsigned long long>(bytes), text);
    if (!scanner.symbols_present()) {
        say("  No symbol table found; patch capability cannot be determined from this image.");
    }
    for (size_t index = 0; index < win32k_capability_count; ++index) {
        const auto presence = scanner.presence(index);
        const char* label = presence == PatchPresence::present ? "present" :
                            presence == PatchPresence::absent ? "absent" :
                            presence == PatchPresence::partial ? "partial (mismatched build)" : "unknown";
        say("  %-20s %-9s %s; %s", win32k_capabilities[index].name, label, win32k_capabilities[index].patch,
            win32k_capabilities[index].effect);
    }
    say("  Enabling a patch requires rebuilding win32k for the exact installed ReactOS release.");
}

void Installer::status_display() noexcept {
    ActiveDisplay active{};
    const uint32_t result = ops_.active_display(ops_.context, active);

    if (result != error::success) {
        say("Active desktop: unknown (Win32 error %lu)", static_cast<unsigned long>(result));
        return;
    }
    const bool radeon = text_equal_insensitive(active.opengl_driver, L"RadeonGL");
    say("Active desktop: \\Device\\Video0 service %ls, OpenGL driver %ls%s",
        active.primary_service[0] != 0 ? active.primary_service : L"(unknown)",
        active.opengl_driver[0] != 0 ? active.opengl_driver : L"(none reported)",
        radeon ? " - radeondisp is the desktop display driver" : "");
}

int Installer::status() noexcept {
    Journal journal{};
    const uint32_t loaded = load_journal(journal);

    say("radeonctl status (read-only; nothing is changed)");
    status_package();
    status_devices();
    status_service();
    status_files(journal);
    status_registry(journal);
    status_icd(journal);
    status_firmware();
    status_win32k();
    status_display();

    if (loaded != error::success) {
        say("Installer journal: unreadable (Win32 error %lu)", static_cast<unsigned long>(loaded));
        return exit_code::failure;
    }
    say("Installer journal: %s, features 0x%02lX%s%s", journal_state_name(journal.state),
        static_cast<unsigned long>(journal.features), journal.restart_pending ? ", restart pending" : "",
        journal_state_transitional(journal.state) ? " - run radeonctl recover" : "");
    if (journal.previous_service[0] != 0) {
        say("  The GPU was bound to %ls before installation.", journal.previous_service);
    }
    return exit_code::success;
}

int Installer::firmware_copy(const CommandLine& command) noexcept {
    const DeviceRecord* device = inventory() == error::success ? find_device(nullptr) : nullptr;
    const uint16_t device_id = device != nullptr ? device->device_id : 0;
    FirmwareEntry entries[firmware_entry_capacity]{};
    const size_t count = firmware_manifest(device != nullptr ? device->revision : 0, entries);
    bool failed = false;
    size_t copied = 0;

    if (ops_.create_directory(ops_.context, firmware_) != error::success) {
        say("Cannot create %ls.", firmware_);
        return exit_code::failure;
    }

    for (size_t index = 0; index < count; ++index) {
        const char* names[]{entries[index].name, entries[index].alternate};

        for (const char* name : names) {
            if (name[0] == 0) {
                continue;
            }

            wchar_t wide[48]{};
            Path source{};
            Path destination{};
            HeapBuffer buffer;
            bool exists = false;
            bool oversize = false;

            if (!text_append_ascii(wide, name) || !path_join(source, command.source_directory, wide) ||
                !path_join(destination, firmware_, wide)) {
                return exit_code::failure;
            }
            if (read_whole(source, firmware_maximum_bytes(entries[index]), buffer, exists, oversize) !=
                    error::success || !exists) {
                continue;
            }

            const FirmwareVerdict verdict = oversize ? FirmwareVerdict::too_large :
                                            validate_firmware_bytes(entries[index], buffer.view(), device_id);
            if (verdict != FirmwareVerdict::valid) {
                say("  %s: %s; not copied.", name, firmware_verdict_name(verdict));
                failed = true;
                continue;
            }

            Sha256 hasher;
            hasher.update(buffer.data(), buffer.size());
            const Sha256Digest digest = hasher.finish();
            bool present = false;
            Sha256Digest current{};

            if (hash_file(destination, present, current) != error::success) {
                return exit_code::failure;
            }
            if (present && current == digest) {
                say("  %s: already present.", name);
                continue;
            }
            if (present && !command.replace_firmware) {
                say("  %s: a different file exists; pass --replace to replace it.", name);
                failed = true;
                continue;
            }

            bool restart = false;
            if (place_file(source, destination, digest, restart) != error::success) {
                say("  %s: copy failed.", name);
                failed = true;
                continue;
            }
            ++copied;
            say("  %s: copied.", name);
        }
    }

    say("Copied %lu firmware file(s) to %ls. No firmware is loaded until PhysicalAsicInit=1 and a restart.",
        static_cast<unsigned long>(copied), firmware_);
    const bool ready = firmware_ready(device_id, device != nullptr ? device->revision : 0, false);
    say(ready ? "Every required firmware file is present and structurally valid." :
                "Some required firmware files are still missing or invalid (see radeonctl status).");
    return failed ? exit_code::failure : exit_code::success;
}

} // namespace

void print_usage(const SystemOps& ops) noexcept {
    const char* lines[]{
        "radeonctl - physical AMD Radeon RX 7900 (Navi31) driver installer for ReactOS",
        "",
        "  radeonctl status                     read-only report of devices, service, files,",
        "                                       hashes, opt-ins, ICD, firmware and win32k",
        "  radeonctl install [options]          bind the one present Navi31 GPU (or reconfigure it)",
        "      --asic          PhysicalAsicInit=1   firmware load and ASIC bring-up",
        "      --rom-override  AtomRomOverride=1    use only drivers\\rx7900\\rx7900.rom",
        "      --vbe           VbeModeSet=1         VBE linear mode after a CSM boot (needs --asic)",
        "      --2d            Radeon2D=1           SDMA 2D acceleration (needs --asic)",
        "      --2d-compute    Radeon2DCompute=1    compute raster engine (needs --2d)",
        "      --cursor        HardwareCursor=1     DCN3.2 hardware cursor (needs --asic)",
        "      --opengl-icd    install radeonglicd.dll and OpenGLDrivers\\RadeonGL",
        "      --force-replace <Service>            replace exactly this existing driver binding",
        "      --dry-run       report the plan without changing anything",
        "  radeonctl upgrade                    replace installed files from this package (backup kept)",
        "  radeonctl rollback                   restore the backed-up previous files",
        "  radeonctl remove                     remove binding, service, files and ICD registration",
        "  radeonctl recover                    finish or undo an interrupted command; repair records",
        "  radeonctl firmware-copy <dir> [--replace]  copy validated firmware into drivers\\rx7900",
        "",
        "Every opt-in defaults to 0. Nothing is started, reset or set to boot start, and no",
        "command reboots. Exit codes: 0 ok, 1 failed, 2 usage, 3 refused, 4 no Navi31 GPU,",
        "3010 restart required.",
    };
    for (const char* line : lines) {
        ops.print(ops.context, line);
    }
}

int run_command(const SystemOps& ops, const CommandLine& command) noexcept {
    Installer installer(ops);

    if (!installer.initialize()) {
        ops.print(ops.context, "System or package path is too long.");
        return exit_code::failure;
    }

    switch (command.command) {
    case Command::help:
        print_usage(ops);
        return exit_code::success;
    case Command::status:
        return installer.status();
    case Command::recover:
        return installer.recover(true);
    case Command::firmware_copy:
        return installer.firmware_copy(command);
    case Command::install:
    case Command::upgrade:
    case Command::rollback:
    case Command::remove:
        break;
    }

    // Complete or undo an interrupted transaction before starting a new one.
    // remove handles its own interrupted states so it can continue removing.
    if (command.command != Command::remove) {
        const int recovered = installer.recover_interrupted();
        if (recovered == exit_code::failure) {
            ops.print(ops.context, "An interrupted installer transaction could not be recovered; see radeonctl status.");
            return exit_code::failure;
        }
    }

    switch (command.command) {
    case Command::install: return installer.install(command);
    case Command::upgrade: return installer.upgrade();
    case Command::rollback: return installer.rollback();
    case Command::remove: return installer.remove();
    default: return exit_code::failure;
    }
}

} // namespace rx7900::tools::radeon

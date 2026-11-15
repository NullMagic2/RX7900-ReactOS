/*
Module: rx7900.tests.guest_installer.rig_tool
Purpose: Guest helper for the radeonctl VM scenario: sanity guard, state
snapshots, step runner with captured output, simulated interruption and host
reporting through the kernel debug log and COM2.
Created: 2026-10-24
Architecture: Runs only inside the disposable ReactOS installer-rig VM (qcow2
overlay, rx7900-stub device). installer_rig.bat drives it; the host checker
work/root-tools/installer-rig/check_results.py parses what it writes. It reads
PnP, registry and file state and never changes a device binding; the only
writes are result files, the simulated-interruption journal values and one
staged file of the installer's own rx7900video.sys. radeonctl itself is
executed unchanged from the ISO package directories.
*/
#include <windows.h>
#include <setupapi.h>
#include <cfgmgr32.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "pci.hpp"
#include "pnp_support.hpp"
#include "sha256.hpp"
#include "windows_support.hpp"

namespace {

using rx7900::DeviceHandle;
using rx7900::DeviceSetHandle;
using rx7900::RegistryHandle;

namespace exit_status {
constexpr int success = 0;
constexpr int failure = 1;
constexpr int usage = 2;
constexpr int guard_refused = 5;
} // namespace exit_status

// Identity the guard requires: the rx7900-stub defaults (the user's board).
constexpr wchar_t stub_hardware_id[] = L"PCI\\VEN_1002&DEV_744C&SUBSYS_471E1DA2&REV_C8";
constexpr wchar_t amd_device_prefix[] = L"PCI\\VEN_1002&DEV_";
constexpr wchar_t lightning_hardware_id[] = L"PCI\\VEN_1234&DEV_11F1";
constexpr wchar_t lightning_service[] = L"LightningVideo";
constexpr wchar_t primary_display_name[] = L"\\\\.\\DISPLAY1";
constexpr wchar_t video_map_key[] = L"HARDWARE\\DEVICEMAP\\VIDEO";
constexpr wchar_t video0_value[] = L"\\Device\\Video0";
constexpr wchar_t results_label[] = L"RIGRESULT";
constexpr wchar_t media_marker[] = L"RIGMEDIA.TXT";

// Installer-owned locations (radeon_install_policy.hpp); read-only here
// except the simulated interruption.
constexpr wchar_t rx7900_service_key[] = L"SYSTEM\\CurrentControlSet\\Services\\rx7900video";
constexpr wchar_t rx7900_device0_key[] = L"SYSTEM\\CurrentControlSet\\Services\\rx7900video\\Device0";
constexpr wchar_t journal_key[] = L"SOFTWARE\\RX7900\\RadeonInstaller";
constexpr wchar_t opengl_key[] = L"SOFTWARE\\Microsoft\\Windows NT\\CurrentVersion\\OpenGLDrivers\\RadeonGL";
constexpr wchar_t session_manager_key[] = L"SYSTEM\\CurrentControlSet\\Control\\Session Manager";
constexpr const wchar_t* device0_values[]{L"PhysicalAsicInit", L"AtomRomOverride", L"VbeModeSet",
                                          L"Radeon2D", L"Radeon2DCompute", L"HardwareCursor"};
constexpr DWORD journal_installed = 2;
constexpr DWORD journal_replace_staging = 3;
constexpr DWORD journal_operation_upgrade = 1;
constexpr wchar_t staged_suffix[] = L".radeonctl-new";
constexpr wchar_t retired_suffix[] = L".radeonctl-old";

// ReactOS installs a newly detected device's default driver (display.inf for
// PCI class 0300) from umpnpmgr after logon; the scenario waits for that to
// finish so radeonctl never races it. CM_PROB_FAILED_INSTALL (28) means no
// driver was found, which also settles the binding.
constexpr DWORD settle_timeout_ms = 3 * 60 * 1000;
constexpr DWORD settle_poll_ms = 2000;
constexpr ULONG problem_failed_install = 28;

// Child processes that run longer than this are terminated and recorded as a hang.
constexpr DWORD step_timeout_ms = 10 * 60 * 1000;
constexpr int timeout_exit_code = -1;
// DbgPrint truncates long strings; keep each mirrored chunk well below it.
constexpr size_t mirror_chunk_bytes = 400;
constexpr size_t line_capacity = 2048;
constexpr size_t id_list_characters = rx7900::maximum_hardware_id_characters;
constexpr size_t text_characters = 512;
constexpr DWORD file_chunk_bytes = 64 * 1024;

// One open, append-only, write-through result file.
class ResultFile final {
public:
    explicit ResultFile(const wchar_t* path) noexcept
        : handle_(CreateFileW(path, FILE_APPEND_DATA, FILE_SHARE_READ, nullptr, OPEN_ALWAYS,
                              FILE_ATTRIBUTE_NORMAL | FILE_FLAG_WRITE_THROUGH, nullptr),
                  INVALID_HANDLE_VALUE) {}
    bool valid() const noexcept { return handle_.valid(); }
    void line(const char* text) noexcept {
        if (!handle_.valid()) {
            return;
        }
        DWORD written = 0;
        WriteFile(handle_.get(), text, static_cast<DWORD>(strlen(text)), &written, nullptr);
        WriteFile(handle_.get(), "\r\n", 2, &written, nullptr);
        FlushFileBuffers(handle_.get());
    }

private:
    DeviceHandle handle_;
};

// Kernel debug log (COM1 host file) and COM2 (second host file), best effort.
class HostMirror final {
public:
    HostMirror() noexcept
        : serial_(CreateFileW(L"\\\\.\\COM2", GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0, nullptr),
                  INVALID_HANDLE_VALUE) {}
    void line(const char* tag, const char* text) noexcept {
        const size_t length = strlen(text);
        size_t offset = 0;

        do {
            char chunk[mirror_chunk_bytes + 64]{};
            const size_t count = length - offset < mirror_chunk_bytes ? length - offset : mirror_chunk_bytes;
            _snprintf(chunk, sizeof(chunk) - 1, "RIGLOG %s|%.*s\n", tag, static_cast<int>(count), text + offset);
            OutputDebugStringA(chunk);
            if (serial_.valid()) {
                DWORD written = 0;
                WriteFile(serial_.get(), chunk, static_cast<DWORD>(strlen(chunk)), &written, nullptr);
            }
            offset += count;
        } while (offset < length);
    }

private:
    DeviceHandle serial_;
};

// Formats into a fixed line, records it in the result file and mirrors it.
class Reporter final {
public:
    Reporter(const wchar_t* path, const char* tag) noexcept : file_(path), tag_(tag) {}
    bool valid() const noexcept { return file_.valid(); }
    void say(const char* format, ...) noexcept {
        char text[line_capacity]{};
        va_list arguments;
        va_start(arguments, format);
        _vsnprintf(text, sizeof(text) - 1, format, arguments);
        va_end(arguments);
        file_.line(text);
        mirror_.line(tag_, text);
        puts(text);
    }

private:
    ResultFile file_;
    HostMirror mirror_;
    const char* tag_;
};

// ASCII view of a wide string for the result files (non-ASCII becomes '?').
template<size_t Capacity>
const char* narrow(const wchar_t* text, char (&output)[Capacity]) noexcept {
    size_t index = 0;
    for (; text != nullptr && text[index] != 0 && index + 1 < Capacity; ++index) {
        output[index] = text[index] < 0x80 ? static_cast<char>(text[index]) : '?';
    }
    output[index] = 0;
    return output;
}

bool starts_with_insensitive(const wchar_t* text, const wchar_t* prefix) noexcept {
    const size_t length = wcslen(prefix);
    return wcslen(text) >= length && _wcsnicmp(text, prefix, length) == 0;
}

// Calls visit(entry) for each entry of a validated REG_MULTI_SZ span.
template<typename Visit>
void each_entry(const wchar_t* list, size_t characters, Visit visit) noexcept {
    size_t position = 0;

    while (position < characters && list[position] != 0) {
        const size_t first = position;
        while (position < characters && list[position] != 0) {
            ++position;
        }
        if (position == characters) {
            return;
        }
        visit(list + first);
        ++position;
    }
}

struct DeviceFacts final {
    wchar_t instance[MAX_DEVICE_ID_LEN + 1]{};
    wchar_t service[rx7900::maximum_filter_characters]{};
    wchar_t upper[rx7900::maximum_filter_characters]{};
    size_t upper_characters = 0;
    wchar_t driver[text_characters]{};
    bool started = false;
    ULONG problem = 0;
};

struct Inventory final {
    size_t navi31 = 0;
    size_t stub = 0;
    size_t lightning = 0;
    DeviceFacts stub_device{};
    DeviceFacts lightning_device{};
    DWORD error = ERROR_SUCCESS;
};

bool read_multi(HDEVINFO devices, SP_DEVINFO_DATA& device, DWORD property, wchar_t* output, DWORD bytes,
                size_t& characters) noexcept {
    DWORD type = 0;
    DWORD returned = 0;
    characters = 0;
    output[0] = 0;

    if (!SetupDiGetDeviceRegistryPropertyW(devices, &device, property, &type, reinterpret_cast<PBYTE>(output),
                                           bytes, &returned) ||
        type != REG_MULTI_SZ || returned < 2 * sizeof(wchar_t) || returned % sizeof(wchar_t) != 0) {
        return false;
    }
    characters = returned / sizeof(wchar_t);
    return true;
}

void read_facts(HDEVINFO devices, SP_DEVINFO_DATA& device, DeviceFacts& facts) noexcept {
    facts = DeviceFacts{};
    DWORD required = 0;
    SetupDiGetDeviceInstanceIdW(devices, &device, facts.instance, MAX_DEVICE_ID_LEN + 1, &required);
    rx7900::pnp_string_property(devices, device, SPDRP_SERVICE, facts.service);
    rx7900::pnp_string_property(devices, device, SPDRP_DRIVER, facts.driver);
    read_multi(devices, device, SPDRP_UPPERFILTERS, facts.upper, sizeof(facts.upper), facts.upper_characters);

    ULONG status = 0;
    ULONG problem = 0;
    if (CM_Get_DevNode_Status(&status, &problem, device.DevInst, 0) == CR_SUCCESS) {
        facts.started = (status & DN_STARTED) != 0;
        facts.problem = (status & DN_HAS_PROBLEM) != 0 ? problem : 0;
    }
}

bool is_navi31_id(const wchar_t* id) noexcept {
    if (!starts_with_insensitive(id, amd_device_prefix)) {
        return false;
    }

    const wchar_t* digits = id + wcslen(amd_device_prefix);
    unsigned value = 0;

    for (int index = 0; index < 4; ++index) {
        const wchar_t character = digits[index];
        const unsigned digit = character >= L'0' && character <= L'9' ? character - L'0' :
                               character >= L'a' && character <= L'f' ? character - L'a' + 10 :
                               character >= L'A' && character <= L'F' ? character - L'A' + 10 : 16;
        if (digit == 16) {
            return false;
        }
        value = value * 16 + digit;
    }
    if (digits[4] != 0 && digits[4] != L'&') {
        return false;
    }
    for (const uint16_t known : rx7900::navi31_device_ids) {
        if (known == value) {
            return true;
        }
    }
    return false;
}

Inventory take_inventory() noexcept {
    Inventory inventory{};
    DeviceSetHandle devices(SetupDiGetClassDevsW(nullptr, nullptr, nullptr, DIGCF_ALLCLASSES | DIGCF_PRESENT),
                            INVALID_HANDLE_VALUE);

    if (!devices.valid()) {
        inventory.error = GetLastError();
        return inventory;
    }

    for (DWORD index = 0; index < rx7900::maximum_pnp_devices; ++index) {
        SP_DEVINFO_DATA device{};
        device.cbSize = sizeof(device);

        if (!SetupDiEnumDeviceInfo(devices.get(), index, &device)) {
            break;
        }

        wchar_t hardware[id_list_characters]{};
        wchar_t compatible[id_list_characters]{};
        size_t hardware_characters = 0;
        size_t compatible_characters = 0;
        read_multi(devices.get(), device, SPDRP_HARDWAREID, hardware, sizeof(hardware), hardware_characters);
        read_multi(devices.get(), device, SPDRP_COMPATIBLEIDS, compatible, sizeof(compatible),
                   compatible_characters);

        bool navi31 = false;
        bool stub = false;
        bool lightning = false;
        const auto inspect = [&](const wchar_t* id) noexcept {
            navi31 = navi31 || is_navi31_id(id);
            stub = stub || _wcsicmp(id, stub_hardware_id) == 0;
            lightning = lightning || _wcsicmp(id, lightning_hardware_id) == 0;
        };
        each_entry(hardware, hardware_characters, inspect);
        each_entry(compatible, compatible_characters, inspect);

        inventory.navi31 += navi31 ? 1 : 0;
        if (stub) {
            ++inventory.stub;
            read_facts(devices.get(), device, inventory.stub_device);
        }
        if (lightning) {
            ++inventory.lightning;
            read_facts(devices.get(), device, inventory.lightning_device);
        }
    }
    return inventory;
}

template<size_t Capacity>
bool registry_string(const wchar_t* key, const wchar_t* name, wchar_t (&output)[Capacity]) noexcept {
    output[0] = 0;
    HKEY raw = nullptr;

    if (RegOpenKeyExW(HKEY_LOCAL_MACHINE, key, 0, KEY_QUERY_VALUE, &raw) != ERROR_SUCCESS) {
        return false;
    }

    RegistryHandle opened(raw);
    DWORD type = 0;
    DWORD bytes = sizeof(output) - sizeof(wchar_t);

    if (RegQueryValueExW(opened.get(), name, nullptr, &type, reinterpret_cast<BYTE*>(output), &bytes) !=
            ERROR_SUCCESS || (type != REG_SZ && type != REG_EXPAND_SZ)) {
        output[0] = 0;
        return false;
    }
    output[Capacity - 1] = 0;
    output[bytes / sizeof(wchar_t) < Capacity ? bytes / sizeof(wchar_t) : Capacity - 1] = 0;
    return true;
}

// Distinguishes absent values from present ones; present is false when the
// key or value is missing or not a REG_DWORD.
bool registry_dword(const wchar_t* key, const wchar_t* name, DWORD& value) noexcept {
    value = 0;
    HKEY raw = nullptr;

    if (RegOpenKeyExW(HKEY_LOCAL_MACHINE, key, 0, KEY_QUERY_VALUE, &raw) != ERROR_SUCCESS) {
        return false;
    }

    RegistryHandle opened(raw);
    DWORD type = 0;
    DWORD bytes = sizeof(value);
    return RegQueryValueExW(opened.get(), name, nullptr, &type, reinterpret_cast<BYTE*>(&value), &bytes) ==
               ERROR_SUCCESS && type == REG_DWORD && bytes == sizeof(value);
}

bool registry_value_present(const wchar_t* key, const wchar_t* name) noexcept {
    HKEY raw = nullptr;

    if (RegOpenKeyExW(HKEY_LOCAL_MACHINE, key, 0, KEY_QUERY_VALUE, &raw) != ERROR_SUCCESS) {
        return false;
    }

    RegistryHandle opened(raw);
    return RegQueryValueExW(opened.get(), name, nullptr, nullptr, nullptr, nullptr) == ERROR_SUCCESS;
}

bool registry_key_present(const wchar_t* key) noexcept {
    HKEY raw = nullptr;

    if (RegOpenKeyExW(HKEY_LOCAL_MACHINE, key, 0, KEY_QUERY_VALUE, &raw) != ERROR_SUCCESS) {
        return false;
    }
    RegCloseKey(raw);
    return true;
}

// Service name a VideoPort registry path names: ...\Services\<name>\DeviceN,
// or the Service value of the sibling Video key of a Control\Video path.
template<size_t Capacity>
void video_path_service(const wchar_t* path, wchar_t (&service)[Capacity]) noexcept {
    service[0] = 0;
    const size_t length = wcslen(path);
    constexpr wchar_t services[] = L"\\services\\";
    const size_t services_length = wcslen(services);

    for (size_t start = 0; start + services_length < length; ++start) {
        if (_wcsnicmp(path + start, services, services_length) == 0) {
            size_t index = 0;
            for (size_t cursor = start + services_length;
                 cursor < length && path[cursor] != L'\\' && index + 1 < Capacity; ++cursor) {
                service[index++] = path[cursor];
            }
            service[index] = 0;
            return;
        }
    }

    constexpr wchar_t machine[] = L"\\Registry\\Machine\\";
    const size_t machine_length = wcslen(machine);

    if (length <= machine_length || _wcsnicmp(path, machine, machine_length) != 0) {
        return;
    }

    wchar_t key[text_characters]{};
    wcsncpy(key, path + machine_length, text_characters - 1);
    size_t end = wcslen(key);
    while (end > 0 && key[end - 1] != L'\\') {
        --end;
    }
    if (end == 0 || end + wcslen(L"Video") >= text_characters) {
        return;
    }
    key[end] = 0;
    wcscat(key, L"Video");
    registry_string(key, L"Service", service);
}

struct DisplayFacts final {
    wchar_t video0[text_characters]{};
    wchar_t video0_service[text_characters]{};
    wchar_t primary_name[text_characters]{};
    wchar_t primary_key[text_characters]{};
    wchar_t primary_service[text_characters]{};   // from the primary's DeviceKey
    wchar_t primary_video[text_characters]{};     // \Device\VideoN win32k names DISPLAY(N+1)
    wchar_t primary_video_service[text_characters]{};
    bool primary_found = false;
};

DisplayFacts display_facts() noexcept {
    DisplayFacts facts{};

    if (registry_string(video_map_key, video0_value, facts.video0)) {
        video_path_service(facts.video0, facts.video0_service);
    }

    for (DWORD index = 0; index < 16; ++index) {
        DISPLAY_DEVICEW device{};
        device.cb = sizeof(device);

        if (!EnumDisplayDevicesW(nullptr, index, &device, 0)) {
            break;
        }
        if ((device.StateFlags & DISPLAY_DEVICE_PRIMARY_DEVICE) != 0) {
            facts.primary_found = true;
            wcsncpy(facts.primary_name, device.DeviceName, text_characters - 1);
            wcsncpy(facts.primary_key, device.DeviceKey, text_characters - 1);
            video_path_service(facts.primary_key, facts.primary_service);

            // win32k names \Device\VideoN as \\.\DISPLAY(N+1) (EngpUpdateGraphicsDeviceList).
            const size_t prefix = wcslen(primary_display_name) - 1;
            const unsigned display = _wcsnicmp(facts.primary_name, primary_display_name, prefix) == 0 ?
                static_cast<unsigned>(_wtoi(facts.primary_name + prefix)) : 0;
            wchar_t path[text_characters]{};
            if (display != 0 &&
                SUCCEEDED(StringCchPrintfW(facts.primary_video, text_characters, L"\\Device\\Video%u", display - 1)) &&
                registry_string(video_map_key, facts.primary_video, path)) {
                video_path_service(path, facts.primary_video_service);
            }
            break;
        }
    }
    return facts;
}

// The primary GDI display device belongs to LightningVideo: its DeviceKey names
// that service, or its DISPLAY(N+1) name maps to a \Device\VideoN that does.
bool lightning_is_active(const DisplayFacts& facts) noexcept {
    return facts.primary_found && (_wcsicmp(facts.primary_service, lightning_service) == 0 ||
                                   _wcsicmp(facts.primary_video_service, lightning_service) == 0);
}

int command_guard(const wchar_t* result_path) {
    Reporter report(result_path, "guard");
    const Inventory inventory = take_inventory();
    const DisplayFacts display = display_facts();
    char a[text_characters]{};
    char b[text_characters]{};
    char c[text_characters]{};
    bool pass = inventory.error == ERROR_SUCCESS;

    report.say("GUARD enumeration error=%lu navi31=%u stub=%u lightning=%u",
               static_cast<unsigned long>(inventory.error), static_cast<unsigned>(inventory.navi31),
               static_cast<unsigned>(inventory.stub), static_cast<unsigned>(inventory.lightning));
    if (inventory.navi31 != 1 || inventory.stub != 1) {
        report.say("GUARD refuse: exactly one Navi31 function, and it must be %s, is required",
                   narrow(stub_hardware_id, a));
        pass = false;
    }

    const auto& lightning = inventory.lightning_device;
    report.say("GUARD lightning instance=%s service=%s started=%u problem=%lu", narrow(lightning.instance, a),
               narrow(lightning.service, b), lightning.started ? 1u : 0u, static_cast<unsigned long>(lightning.problem));
    if (inventory.lightning != 1 || _wcsicmp(lightning.service, lightning_service) != 0 || !lightning.started ||
        lightning.problem != 0) {
        report.say("GUARD refuse: the Lightning device must be present once, bound to LightningVideo and started");
        pass = false;
    }

    char d[text_characters]{};
    report.say("GUARD display video0=%s video0_service=%s primary=%s primary_key=%s", narrow(display.video0, a),
               narrow(display.video0_service, b), narrow(display.primary_name, c), narrow(display.primary_key, d));
    report.say("GUARD display primary_video=%s primary_video_service=%s primary_key_service=%s",
               narrow(display.primary_video, a), narrow(display.primary_video_service, b),
               narrow(display.primary_service, c));
    if (!lightning_is_active(display)) {
        report.say("GUARD refuse: LightningVideo is not the active primary display");
        pass = false;
    }

    report.say(pass ? "GUARD pass" : "GUARD FAIL");
    return pass ? exit_status::success : exit_status::guard_refused;
}

bool hash_file(const wchar_t* path, bool& exists, rx7900::tools::Sha256Digest& digest, uint64_t& bytes) noexcept {
    exists = false;
    bytes = 0;
    DeviceHandle file(CreateFileW(path, GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE, nullptr,
                                  OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr),
                      INVALID_HANDLE_VALUE);

    if (!file.valid()) {
        const DWORD error = GetLastError();
        return error == ERROR_FILE_NOT_FOUND || error == ERROR_PATH_NOT_FOUND;
    }

    exists = true;
    rx7900::tools::Sha256 hasher;
    static uint8_t chunk[file_chunk_bytes];
    DWORD received = 0;

    while (ReadFile(file.get(), chunk, sizeof(chunk), &received, nullptr) && received != 0) {
        hasher.update(chunk, received);
        bytes += received;
    }
    digest = hasher.finish();
    return true;
}

void report_file(Reporter& report, const char* step, const char* key, const wchar_t* path) noexcept {
    bool exists = false;
    uint64_t bytes = 0;
    rx7900::tools::Sha256Digest digest{};

    if (!hash_file(path, exists, digest, bytes)) {
        report.say("SNAP %s %s=unreadable", step, key);
        return;
    }
    if (!exists) {
        report.say("SNAP %s %s=absent", step, key);
        return;
    }

    char hex[rx7900::tools::sha256_hex_characters]{};
    rx7900::tools::sha256_hex(digest, hex);
    report.say("SNAP %s %s=%s", step, key, hex);
}

template<size_t Capacity>
bool join(wchar_t (&output)[Capacity], const wchar_t* directory, const wchar_t* name) noexcept {
    return SUCCEEDED(StringCchPrintfW(output, Capacity, L"%s\\%s", directory, name));
}

// %SystemRoot%\System32, the directory radeonctl's live paths are built from
// (rx7900::installed_driver_path names the probe driver file, not a directory).
bool system_directory(wchar_t (&output)[MAX_PATH]) noexcept {
    const UINT length = GetSystemDirectoryW(output, MAX_PATH);
    return length != 0 && length < MAX_PATH;
}

void report_component(Reporter& report, const char* step, const wchar_t* directory, const wchar_t* name,
                      const wchar_t* backup_directory) noexcept {
    wchar_t path[MAX_PATH]{};
    char narrow_name[MAX_PATH]{};
    char key[MAX_PATH + 32]{};
    narrow(name, narrow_name);

    const struct {
        const wchar_t* suffix;
        const char* label;
    } variants[]{{L"", ""}, {staged_suffix, ".staged"}, {retired_suffix, ".retired"}};

    for (const auto& variant : variants) {
        wchar_t file[MAX_PATH]{};
        if (FAILED(StringCchPrintfW(file, MAX_PATH, L"%s%s", name, variant.suffix)) || !join(path, directory, file)) {
            continue;
        }
        _snprintf(key, sizeof(key) - 1, "file.%s%s", narrow_name, variant.label);
        report_file(report, step, key, path);
    }
    if (join(path, backup_directory, name)) {
        _snprintf(key, sizeof(key) - 1, "backup.%s", narrow_name);
        report_file(report, step, key, path);
    }
}

void report_firmware(Reporter& report, const char* step, const wchar_t* firmware_directory) noexcept {
    wchar_t pattern[MAX_PATH]{};
    WIN32_FIND_DATAW found{};
    unsigned count = 0;
    char names[line_capacity / 2]{};

    if (!join(pattern, firmware_directory, L"*")) {
        return;
    }

    const HANDLE search = FindFirstFileW(pattern, &found);

    if (search != INVALID_HANDLE_VALUE) {
        do {
            if ((found.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) != 0) {
                continue;
            }
            char name[MAX_PATH]{};
            narrow(found.cFileName, name);
            if (strlen(names) + strlen(name) + 2 < sizeof(names)) {
                strcat(names, names[0] != 0 ? "," : "");
                strcat(names, name);
            }
            ++count;
        } while (FindNextFileW(search, &found));
        FindClose(search);
    }
    report.say("SNAP %s firmware.count=%u", step, count);
    report.say("SNAP %s firmware.names=%s", step, names);
}

// True when the first file_chunk_bytes of path mention rx7900video (ASCII or
// UTF-16LE text, case-insensitive): a staged copy of radeondisplay.inf.
bool names_rx7900video(const wchar_t* path) noexcept {
    constexpr char needle[] = "rx7900video";
    constexpr size_t needle_length = sizeof(needle) - 1;
    DeviceHandle file(CreateFileW(path, GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE, nullptr,
                                  OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr),
                      INVALID_HANDLE_VALUE);
    static uint8_t chunk[file_chunk_bytes];
    DWORD received = 0;

    if (!file.valid() || !ReadFile(file.get(), chunk, sizeof(chunk), &received, nullptr)) {
        return false;
    }

    size_t matched = 0;
    for (DWORD index = 0; index < received; ++index) {
        const uint8_t byte = chunk[index];
        if (byte == 0) {
            continue;  // UTF-16LE high bytes of ASCII text
        }
        const char folded = byte >= 'A' && byte <= 'Z' ? static_cast<char>(byte + 32) : static_cast<char>(byte);
        matched = folded == needle[matched] ? matched + 1 : (folded == needle[0] ? 1 : 0);
        if (matched == needle_length) {
            return true;
        }
    }
    return false;
}

// Lists the SetupAPI-staged INF copies (%SystemRoot%\inf\oem*.inf) that name
// rx7900video: removal must leave none, or ReactOS selects one again at boot.
void report_staged_infs(Reporter& report, const char* step) noexcept {
    wchar_t inf[MAX_PATH]{};
    wchar_t pattern[MAX_PATH]{};
    char names[line_capacity / 2]{};
    const UINT length = GetWindowsDirectoryW(inf, MAX_PATH);

    if (length != 0 && length < MAX_PATH && SUCCEEDED(StringCchCatW(inf, MAX_PATH, L"\\inf")) &&
        join(pattern, inf, L"oem*.inf")) {
        WIN32_FIND_DATAW found{};
        const HANDLE search = FindFirstFileW(pattern, &found);

        if (search != INVALID_HANDLE_VALUE) {
            do {
                wchar_t path[MAX_PATH]{};
                char name[MAX_PATH]{};
                if ((found.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) != 0 || !join(path, inf, found.cFileName) ||
                    !names_rx7900video(path)) {
                    continue;
                }
                narrow(found.cFileName, name);
                for (char* cursor = name; *cursor != 0; ++cursor) {
                    *cursor = *cursor >= 'A' && *cursor <= 'Z' ? static_cast<char>(*cursor + 32) : *cursor;
                }
                if (strlen(names) + strlen(name) + 2 < sizeof(names)) {
                    strcat(names, names[0] != 0 ? "," : "");
                    strcat(names, name);
                }
            } while (FindNextFileW(search, &found));
            FindClose(search);
        }
    }
    report.say("SNAP %s oem_inf.rx7900video=%s", step, names);
}

int command_snapshot(const wchar_t* step_wide, const wchar_t* result_path) {
    char step[64]{};
    narrow(step_wide, step);
    Reporter report(result_path, step);
    char a[text_characters]{};
    char b[text_characters]{};

    const Inventory inventory = take_inventory();
    const auto& stub = inventory.stub_device;
    const auto& lightning = inventory.lightning_device;
    report.say("SNAP %s boot.uptime_ms=%lu", step, static_cast<unsigned long>(GetTickCount()));
    report.say("SNAP %s inventory.error=%lu", step, static_cast<unsigned long>(inventory.error));
    report.say("SNAP %s stub.count=%u", step, static_cast<unsigned>(inventory.stub));
    report.say("SNAP %s navi31.count=%u", step, static_cast<unsigned>(inventory.navi31));
    report.say("SNAP %s stub.instance=%s", step, narrow(stub.instance, a));
    report.say("SNAP %s stub.service=%s", step, narrow(stub.service, a));
    report.say("SNAP %s stub.started=%u", step, stub.started ? 1u : 0u);
    report.say("SNAP %s stub.problem=%lu", step, static_cast<unsigned long>(stub.problem));
    report.say("SNAP %s stub.driver=%s", step, narrow(stub.driver, a));
    report.say("SNAP %s lightning.count=%u", step, static_cast<unsigned>(inventory.lightning));
    report.say("SNAP %s lightning.instance=%s", step, narrow(lightning.instance, a));
    report.say("SNAP %s lightning.service=%s", step, narrow(lightning.service, a));
    report.say("SNAP %s lightning.started=%u", step, lightning.started ? 1u : 0u);
    report.say("SNAP %s lightning.problem=%lu", step, static_cast<unsigned long>(lightning.problem));
    report.say("SNAP %s lightning.driver=%s", step, narrow(lightning.driver, a));

    char upper[rx7900::maximum_filter_characters]{};
    each_entry(lightning.upper, lightning.upper_characters, [&](const wchar_t* entry) noexcept {
        char name[text_characters]{};
        narrow(entry, name);
        if (strlen(upper) + strlen(name) + 2 < sizeof(upper)) {
            strcat(upper, upper[0] != 0 ? "," : "");
            strcat(upper, name);
        }
    });
    report.say("SNAP %s lightning.upper=%s", step, upper);

    const DisplayFacts display = display_facts();
    report.say("SNAP %s display.video0_service=%s", step, narrow(display.video0_service, a));
    report.say("SNAP %s display.primary=%s", step, narrow(display.primary_name, a));
    report.say("SNAP %s display.primary_service=%s", step,
               narrow(display.primary_service[0] != 0 ? display.primary_service : display.primary_video_service, a));
    report.say("SNAP %s display.lightning_active=%u", step, lightning_is_active(display) ? 1u : 0u);

    DWORD value = 0;
    const bool service = registry_key_present(rx7900_service_key);
    report.say("SNAP %s service.present=%u", step, service ? 1u : 0u);
    report.say("SNAP %s service.start=%s", step,
               registry_dword(rx7900_service_key, L"Start", value) ? _ultoa(value, a, 10) : "absent");
    wchar_t image[text_characters]{};
    registry_string(rx7900_service_key, L"ImagePath", image);
    report.say("SNAP %s service.image=%s", step, narrow(image, a));
    for (const wchar_t* name : device0_values) {
        report.say("SNAP %s device0.%s=%s", step, narrow(name, b),
                   registry_dword(rx7900_device0_key, name, value) ? _ultoa(value, a, 10) : "absent");
    }

    report.say("SNAP %s journal.state=%s", step,
               registry_dword(journal_key, L"State", value) ? _ultoa(value, a, 10) : "absent");
    report.say("SNAP %s journal.features=%s", step,
               registry_dword(journal_key, L"Features", value) ? _ultoa(value, a, 10) : "absent");
    report.say("SNAP %s journal.restart_pending=%s", step,
               registry_dword(journal_key, L"RestartPending", value) ? _ultoa(value, a, 10) : "absent");
    report.say("SNAP %s journal.key=%u", step, registry_key_present(journal_key) ? 1u : 0u);
    report.say("SNAP %s opengl.radeongl=%u", step, registry_key_present(opengl_key) ? 1u : 0u);
    report.say("SNAP %s pending_renames=%u", step,
               registry_value_present(session_manager_key, L"PendingFileRenameOperations") ? 1u : 0u);

    wchar_t system[MAX_PATH]{};
    wchar_t drivers[MAX_PATH]{};
    wchar_t firmware[MAX_PATH]{};
    wchar_t backup[MAX_PATH]{};

    if (!system_directory(system) || !join(drivers, system, L"drivers") ||
        !join(firmware, drivers, L"rx7900") || !join(backup, firmware, L"backup")) {
        report.say("SNAP %s paths=unresolved", step);
        return exit_status::failure;
    }
    report_component(report, step, drivers, L"rx7900video.sys", backup);
    report_component(report, step, system, L"radeondisp.dll", backup);
    report_component(report, step, system, L"radeonglicd.dll", backup);
    report_firmware(report, step, firmware);
    report_staged_infs(report, step);
    report.say("SNAP %s end", step);
    return exit_status::success;
}

// settle <results.txt>: waits until the stub has a function driver or a failed
// install, so the default-binding install has completed.
int command_settle(const wchar_t* result_path) {
    Reporter report(result_path, "settle");
    const DWORD started = GetTickCount();
    Inventory inventory{};

    for (;;) {
        inventory = take_inventory();
        const bool settled = inventory.stub == 1 && (inventory.stub_device.service[0] != 0 ||
                                                      inventory.stub_device.problem == problem_failed_install);
        if (settled || GetTickCount() - started >= settle_timeout_ms) {
            break;
        }
        Sleep(settle_poll_ms);
    }

    char service[text_characters]{};
    report.say("SETTLE service=%s started=%u problem=%lu waited_ms=%lu", narrow(inventory.stub_device.service, service),
               inventory.stub_device.started ? 1u : 0u, static_cast<unsigned long>(inventory.stub_device.problem),
               static_cast<unsigned long>(GetTickCount() - started));
    return exit_status::success;
}

// Writes a batch fragment with the stub's current binding for the scenario.
int command_stub_vars(const wchar_t* batch_path) {
    const Inventory inventory = take_inventory();
    char service[text_characters]{};
    narrow(inventory.stub_device.service, service);
    for (char* cursor = service; *cursor != 0; ++cursor) {
        // Only service-name characters reach the batch file.
        const bool safe = (*cursor >= 'a' && *cursor <= 'z') || (*cursor >= 'A' && *cursor <= 'Z') ||
                          (*cursor >= '0' && *cursor <= '9') || *cursor == '_' || *cursor == '-';
        *cursor = safe ? *cursor : '_';
    }

    DeviceHandle file(CreateFileW(batch_path, GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr),
                      INVALID_HANDLE_VALUE);

    if (!file.valid()) {
        return exit_status::failure;
    }

    char text[line_capacity]{};
    _snprintf(text, sizeof(text) - 1, "set RIG_STUB_SERVICE=%s\r\nset RIG_STUB_PROBLEM=%lu\r\nset RIG_STUB_STARTED=%u\r\n",
              service, static_cast<unsigned long>(inventory.stub_device.problem),
              inventory.stub_device.started ? 1u : 0u);
    DWORD written = 0;
    return WriteFile(file.get(), text, static_cast<DWORD>(strlen(text)), &written, nullptr) ? exit_status::success
                                                                                           : exit_status::failure;
}

// Locates the RIGRESULT FAT volume and the ISO (by its marker file) and writes
// a batch fragment setting RIG_RESULTS and RIG_MEDIA.
int command_locate(const wchar_t* batch_path) {
    wchar_t results[MAX_PATH]{};
    wchar_t media[MAX_PATH]{};

    for (wchar_t letter = L'C'; letter <= L'Z'; ++letter) {
        wchar_t root[] = {letter, L':', L'\\', 0};
        const UINT type = GetDriveTypeW(root);

        if (type == DRIVE_NO_ROOT_DIR || type == DRIVE_UNKNOWN) {
            continue;
        }

        wchar_t label[MAX_PATH]{};
        if (results[0] == 0 && GetVolumeInformationW(root, label, MAX_PATH, nullptr, nullptr, nullptr, nullptr, 0) &&
            _wcsicmp(label, results_label) == 0) {
            StringCchPrintfW(results, MAX_PATH, L"%c:\\RESULTS", letter);
        }

        wchar_t marker[MAX_PATH]{};
        if (media[0] == 0 && join(marker, root, media_marker) && GetFileAttributesW(marker) != INVALID_FILE_ATTRIBUTES) {
            StringCchPrintfW(media, MAX_PATH, L"%c:", letter);
        }
    }

    DeviceHandle file(CreateFileW(batch_path, GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr),
                      INVALID_HANDLE_VALUE);

    if (!file.valid()) {
        return exit_status::failure;
    }

    char a[MAX_PATH]{};
    char b[MAX_PATH]{};
    char text[line_capacity]{};
    _snprintf(text, sizeof(text) - 1, "set RIG_RESULTS=%s\r\nset RIG_MEDIA=%s\r\n", narrow(results, a), narrow(media, b));
    DWORD written = 0;
    WriteFile(file.get(), text, static_cast<DWORD>(strlen(text)), &written, nullptr);
    return media[0] != 0 ? exit_status::success : exit_status::failure;
}

// Appends one argument to a CreateProcess command line; arguments with blanks or
// quotes are quoted the MSVCRT way, others are passed verbatim.
bool append_argument(wchar_t* line, size_t capacity, const wchar_t* argument) noexcept {
    size_t length = wcslen(line);
    const auto put = [&](wchar_t character) noexcept {
        if (length + 1 >= capacity) {
            return false;
        }
        line[length++] = character;
        line[length] = 0;
        return true;
    };

    if (length != 0 && !put(L' ')) {
        return false;
    }

    const bool quoted = argument[0] == 0 || wcspbrk(argument, L" \t\"") != nullptr;

    if (quoted && !put(L'"')) {
        return false;
    }
    for (const wchar_t* cursor = argument; *cursor != 0; ++cursor) {
        if (*cursor == L'"' && !put(L'\\')) {
            return false;
        }
        if (!put(*cursor)) {
            return false;
        }
    }
    return !quoted || put(L'"');
}

void mirror_file(HostMirror& mirror, const char* tag, const wchar_t* path) noexcept {
    DeviceHandle file(CreateFileW(path, GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING,
                                  FILE_ATTRIBUTE_NORMAL, nullptr),
                      INVALID_HANDLE_VALUE);

    if (!file.valid()) {
        return;
    }

    char line[line_capacity]{};
    size_t used = 0;
    char chunk[4096];
    DWORD received = 0;

    while (ReadFile(file.get(), chunk, sizeof(chunk), &received, nullptr) && received != 0) {
        for (DWORD index = 0; index < received; ++index) {
            const char character = chunk[index];
            if (character == '\n' || used + 1 == sizeof(line)) {
                line[used] = 0;
                mirror.line(tag, line);
                used = 0;
            }
            if (character != '\n' && character != '\r') {
                line[used++] = character;
            }
        }
    }
    if (used != 0) {
        line[used] = 0;
        mirror.line(tag, line);
    }
}

// run <results dir> <step> <executable> [arguments...]: runs the step with
// stdout/stderr in <step>.out/<step>.err and records its exit code.
int command_run(int argc, wchar_t** argv) {
    const wchar_t* directory = argv[2];
    const wchar_t* step_wide = argv[3];
    const wchar_t* executable = argv[4];
    wchar_t results[MAX_PATH]{};
    wchar_t out_path[MAX_PATH]{};
    wchar_t err_path[MAX_PATH]{};
    wchar_t out_name[MAX_PATH]{};
    wchar_t err_name[MAX_PATH]{};
    char step[64]{};
    narrow(step_wide, step);

    if (FAILED(StringCchPrintfW(out_name, MAX_PATH, L"%s.out", step_wide)) ||
        FAILED(StringCchPrintfW(err_name, MAX_PATH, L"%s.err", step_wide)) ||
        !join(results, directory, L"results.txt") || !join(out_path, directory, out_name) ||
        !join(err_path, directory, err_name)) {
        return exit_status::usage;
    }

    Reporter report(results, step);
    static wchar_t command[32768]{};
    command[0] = 0;
    char arguments[line_capacity]{};

    for (int index = 4; index < argc; ++index) {
        if (!append_argument(command, sizeof(command) / sizeof(command[0]), argv[index])) {
            report.say("STEP %s exit=usage reason=command-line-too-long", step);
            return exit_status::usage;
        }
        if (index > 4) {
            char argument[text_characters]{};
            narrow(argv[index], argument);
            if (strlen(arguments) + strlen(argument) + 2 < sizeof(arguments)) {
                strcat(arguments, arguments[0] != 0 ? " " : "");
                strcat(arguments, argument);
            }
        }
    }

    SECURITY_ATTRIBUTES inherit{sizeof(SECURITY_ATTRIBUTES), nullptr, TRUE};
    DeviceHandle out(CreateFileW(out_path, GENERIC_WRITE, FILE_SHARE_READ, &inherit, CREATE_ALWAYS,
                                 FILE_ATTRIBUTE_NORMAL, nullptr),
                     INVALID_HANDLE_VALUE);
    DeviceHandle err(CreateFileW(err_path, GENERIC_WRITE, FILE_SHARE_READ, &inherit, CREATE_ALWAYS,
                                 FILE_ATTRIBUTE_NORMAL, nullptr),
                     INVALID_HANDLE_VALUE);
    DeviceHandle in(CreateFileW(L"NUL", GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE, &inherit, OPEN_EXISTING,
                                FILE_ATTRIBUTE_NORMAL, nullptr),
                    INVALID_HANDLE_VALUE);

    if (!out.valid() || !err.valid()) {
        report.say("STEP %s exit=error reason=cannot-create-output win32=%lu", step,
                   static_cast<unsigned long>(GetLastError()));
        return exit_status::failure;
    }

    char program[MAX_PATH]{};
    report.say("BEGIN %s program=%s args=%s", step, narrow(executable, program), arguments);

    STARTUPINFOW startup{};
    startup.cb = sizeof(startup);
    startup.dwFlags = STARTF_USESTDHANDLES;
    startup.hStdInput = in.valid() ? in.get() : nullptr;
    startup.hStdOutput = out.get();
    startup.hStdError = err.get();
    PROCESS_INFORMATION process{};
    const DWORD started = GetTickCount();

    if (!CreateProcessW(executable, command, nullptr, nullptr, TRUE, 0, nullptr, nullptr, &startup, &process)) {
        report.say("STEP %s exit=error reason=create-process win32=%lu", step,
                   static_cast<unsigned long>(GetLastError()));
        return exit_status::failure;
    }

    DeviceHandle process_handle(process.hProcess);
    DeviceHandle thread_handle(process.hThread);
    const DWORD waited = WaitForSingleObject(process.hProcess, step_timeout_ms);
    long code = timeout_exit_code;

    if (waited == WAIT_OBJECT_0) {
        DWORD exit_code = 0;
        GetExitCodeProcess(process.hProcess, &exit_code);
        code = static_cast<long>(exit_code);
    } else {
        TerminateProcess(process.hProcess, static_cast<UINT>(timeout_exit_code));
    }
    FlushFileBuffers(out.get());
    FlushFileBuffers(err.get());

    HostMirror mirror;
    char out_tag[96]{};
    char err_tag[96]{};
    _snprintf(out_tag, sizeof(out_tag) - 1, "%s out", step);
    _snprintf(err_tag, sizeof(err_tag) - 1, "%s err", step);
    mirror_file(mirror, out_tag, out_path);
    mirror_file(mirror, err_tag, err_path);

    if (waited == WAIT_OBJECT_0) {
        report.say("STEP %s exit=%ld ms=%lu", step, code, static_cast<unsigned long>(GetTickCount() - started));
    } else {
        report.say("STEP %s exit=timeout ms=%lu", step, static_cast<unsigned long>(GetTickCount() - started));
    }
    return exit_status::success;
}

// hold <file> run <results dir> <step> <program> [arguments...]: runs the step
// while this process keeps <file> open without FILE_SHARE_DELETE, the way a
// loaded image refuses renames. radeonctl has no test hook for its
// replace-at-restart path, and ReactOS does not keep loaded driver images
// open, so this is the only way the VM reaches MoveFileEx DELAY_UNTIL_REBOOT.
int command_hold(int argc, wchar_t** argv) {
    DeviceHandle held(CreateFileW(argv[2], GENERIC_READ, FILE_SHARE_READ, nullptr, OPEN_EXISTING,
                                  FILE_ATTRIBUTE_NORMAL, nullptr),
                      INVALID_HANDLE_VALUE);
    const DWORD hold_error = held.valid() ? ERROR_SUCCESS : GetLastError();
    wchar_t results[MAX_PATH]{};
    char path[MAX_PATH]{};
    char step[64]{};
    narrow(argv[2], path);
    narrow(argv[5], step);

    if (join(results, argv[4], L"results.txt")) {
        Reporter report(results, step);
        report.say("HOLD %s %s win32=%lu", step, held.valid() ? path : "FAILED",
                   static_cast<unsigned long>(hold_error));
    }
    // argv[3] is "run"; command_run reads argv[2..] as its own arguments.
    return command_run(argc - 2, argv + 2);
}

// Simulates a crash during upgrade staging without any installer test hook:
// on a completed install (journal State=installed) it stages a copy of the
// live rx7900video.sys under the installer's staged name and writes what
// radeonctl journals before staging (Target/Prior hashes of that component,
// Operation=upgrade), then moves the commit value to replace_staging.
// radeonctl recover must then discard the staged copy and return to installed
// with the live files unchanged. (A staged copy without a Target record is not
// the installer's and recover rightly keeps it: rig run 2, P2-17.)
int command_simulate_interrupt(const wchar_t* result_path) {
    Reporter report(result_path, "interrupt");
    DWORD state = 0;

    if (!registry_dword(journal_key, L"State", state) || state != journal_installed) {
        report.say("INTERRUPT refused: journal State is not installed (%lu)", static_cast<unsigned long>(state));
        return exit_status::failure;
    }

    wchar_t system[MAX_PATH]{};
    wchar_t drivers[MAX_PATH]{};
    wchar_t live[MAX_PATH]{};
    wchar_t staged[MAX_PATH]{};

    if (!system_directory(system) || !join(drivers, system, L"drivers") ||
        !join(live, drivers, L"rx7900video.sys") ||
        FAILED(StringCchPrintfW(staged, MAX_PATH, L"%s%s", live, staged_suffix)) ||
        !CopyFileW(live, staged, FALSE)) {
        report.say("INTERRUPT failed: cannot stage rx7900video.sys (win32 %lu)", static_cast<unsigned long>(GetLastError()));
        return exit_status::failure;
    }

    bool exists = false;
    uint64_t bytes = 0;
    rx7900::tools::Sha256Digest digest{};
    wchar_t hex[rx7900::tools::sha256_hex_characters]{};

    if (!hash_file(live, exists, digest, bytes) || !exists) {
        report.say("INTERRUPT failed: cannot hash rx7900video.sys");
        return exit_status::failure;
    }
    rx7900::tools::sha256_hex(digest, hex);

    HKEY raw = nullptr;
    if (RegOpenKeyExW(HKEY_LOCAL_MACHINE, journal_key, 0, KEY_SET_VALUE, &raw) != ERROR_SUCCESS) {
        report.say("INTERRUPT failed: journal key not writable");
        return exit_status::failure;
    }

    RegistryHandle key(raw);
    const DWORD operation = journal_operation_upgrade;
    const DWORD staging = journal_replace_staging;
    const DWORD hex_bytes = static_cast<DWORD>(sizeof(hex));

    if (RegSetValueExW(key.get(), L"Target.rx7900video.sys", 0, REG_SZ, reinterpret_cast<const BYTE*>(hex),
                       hex_bytes) != ERROR_SUCCESS ||
        RegSetValueExW(key.get(), L"Prior.rx7900video.sys", 0, REG_SZ, reinterpret_cast<const BYTE*>(hex),
                       hex_bytes) != ERROR_SUCCESS ||
        RegSetValueExW(key.get(), L"Operation", 0, REG_DWORD, reinterpret_cast<const BYTE*>(&operation),
                       sizeof(operation)) != ERROR_SUCCESS ||
        RegSetValueExW(key.get(), L"State", 0, REG_DWORD, reinterpret_cast<const BYTE*>(&staging),
                       sizeof(staging)) != ERROR_SUCCESS) {
        report.say("INTERRUPT failed: journal values not written");
        return exit_status::failure;
    }
    RegFlushKey(key.get());
    report.say("INTERRUPT staged rx7900video.sys%s and set journal State=3 (replace_staging)", ".radeonctl-new");
    return exit_status::success;
}

// note <results file> <text...>: records a scenario marker line.
int command_note(int argc, wchar_t** argv) {
    Reporter report(argv[2], "note");
    char text[line_capacity]{};

    for (int index = 3; index < argc; ++index) {
        char word[text_characters]{};
        narrow(argv[index], word);
        if (strlen(text) + strlen(word) + 2 < sizeof(text)) {
            strcat(text, text[0] != 0 ? " " : "");
            strcat(text, word);
        }
    }
    report.say("NOTE %s", text);
    return exit_status::success;
}

void usage() noexcept {
    puts("rigtool guard <results.txt>\n"
         "rigtool snapshot <step> <results.txt>\n"
         "rigtool run <results dir> <step> <program> [arguments]\n"
         "rigtool hold <file> run <results dir> <step> <program> [arguments]\n"
         "rigtool settle <results.txt>\n"
         "rigtool stubvars <out.bat>\n"
         "rigtool locate <out.bat>\n"
         "rigtool simulate-interrupt <results.txt>\n"
         "rigtool note <results.txt> <text>");
}

} // namespace

int wmain(int argc, wchar_t** argv) {
    if (argc >= 3 && _wcsicmp(argv[1], L"guard") == 0) {
        return command_guard(argv[2]);
    }
    if (argc >= 4 && _wcsicmp(argv[1], L"snapshot") == 0) {
        return command_snapshot(argv[2], argv[3]);
    }
    if (argc >= 5 && _wcsicmp(argv[1], L"run") == 0) {
        return command_run(argc, argv);
    }
    if (argc >= 7 && _wcsicmp(argv[1], L"hold") == 0 && _wcsicmp(argv[3], L"run") == 0) {
        return command_hold(argc, argv);
    }
    if (argc >= 3 && _wcsicmp(argv[1], L"settle") == 0) {
        return command_settle(argv[2]);
    }
    if (argc >= 3 && _wcsicmp(argv[1], L"stubvars") == 0) {
        return command_stub_vars(argv[2]);
    }
    if (argc >= 3 && _wcsicmp(argv[1], L"locate") == 0) {
        return command_locate(argv[2]);
    }
    if (argc >= 3 && _wcsicmp(argv[1], L"simulate-interrupt") == 0) {
        return command_simulate_interrupt(argv[2]);
    }
    if (argc >= 3 && _wcsicmp(argv[1], L"note") == 0) {
        return command_note(argc, argv);
    }
    usage();
    return exit_status::usage;
}

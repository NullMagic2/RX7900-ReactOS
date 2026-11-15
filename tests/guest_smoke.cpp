/*
Module: rx7900.tests.guest_smoke
Purpose: Exercise packaged service installation, IOCTLs, reload and removal.
Created: 2026-10-21
Architecture: Runs only in a disposable ReactOS guest; reports on disk and COM2.
*/
#include <windows.h>
#include <strsafe.h>
#include <stdio.h>

#include "windows_support.hpp"

namespace {

constexpr DWORD child_timeout_ms = 30000;
constexpr DWORD output_capacity = 4096;
constexpr size_t command_capacity = MAX_PATH + 64;

// buffer is readable for size bytes for this call; no reference is retained.
bool write_all(HANDLE file, const char* buffer, DWORD size) {
    DWORD offset = 0;

    while (offset < size) {
        DWORD written = 0;

        if (!WriteFile(file, buffer + offset, size - offset, &written, nullptr) ||
            written == 0 || written > size - offset) {
            return false;
        }

        offset += written;
    }

    return true;
}

int run_command(const wchar_t* executable, const wchar_t* command, HANDLE log) {
    wchar_t command_line[command_capacity]{};

    if (FAILED(StringCchPrintfW(command_line, command_capacity, L"\"%s\" %s",
                               executable, command))) {
        return -1;
    }

    STARTUPINFOW startup{};
    startup.cb = sizeof(startup);
    startup.dwFlags = STARTF_USESTDHANDLES;
    startup.hStdOutput = log;
    startup.hStdError = log;
    startup.hStdInput = GetStdHandle(STD_INPUT_HANDLE);
    PROCESS_INFORMATION process{};

    if (!CreateProcessW(executable, command_line, nullptr, nullptr, TRUE, 0,
                        nullptr, nullptr, &startup, &process)) {
        return -1;
    }

    rx7900::DeviceHandle thread(process.hThread);
    rx7900::DeviceHandle child(process.hProcess);

    if (WaitForSingleObject(child.get(), child_timeout_ms) != WAIT_OBJECT_0) {
        return -1;
    }

    DWORD code = 0;

    if (!GetExitCodeProcess(child.get(), &code) || code > 0x7fffffff) {
        return -1;
    }

    return static_cast<int>(code);
}

void relay_log(HANDLE log, HANDLE serial) {
    LARGE_INTEGER beginning{};

    if (!SetFilePointerEx(log, beginning, nullptr, FILE_BEGIN)) {
        return;
    }

    char buffer[output_capacity + 1]{};
    DWORD received = 0;

    while (ReadFile(log, buffer, output_capacity, &received, nullptr) && received != 0) {
        // ReadFile filled at most output_capacity bytes; reserve one terminator.
        buffer[received] = '\0';
        OutputDebugStringA(buffer);

        if (serial != INVALID_HANDLE_VALUE && !write_all(serial, buffer, received)) {
            return;
        }
    }
}

bool schedule_removal_check() {
    wchar_t executable[MAX_PATH]{};
    wchar_t command_line[command_capacity]{};

    if (!rx7900::module_sibling_path(L"rx7900_guest_smoke.exe", executable) ||
        FAILED(StringCchPrintfW(command_line, command_capacity,
            L"\"%s\" --verify-removal", executable))) {
        return false;
    }

    HKEY raw_key = nullptr;
    const LONG opened = RegCreateKeyExW(HKEY_LOCAL_MACHINE,
        L"SOFTWARE\\Microsoft\\Windows\\CurrentVersion\\RunOnce", 0, nullptr,
        0, KEY_SET_VALUE, nullptr, &raw_key, nullptr);
    rx7900::RegistryHandle key(raw_key);

    if (opened != ERROR_SUCCESS) {
        return false;
    }

    size_t characters = 0;

    if (FAILED(StringCchLengthW(command_line, command_capacity, &characters))) {
        return false;
    }

    // REG_SZ consumes the checked, terminated UTF-16 array for this call only.
    return RegSetValueExW(key.get(), L"RX7900RemovalCheck", 0, REG_SZ,
        reinterpret_cast<const BYTE*>(command_line),
        static_cast<DWORD>((characters + 1) * sizeof(wchar_t))) == ERROR_SUCCESS;
}

int verify_removal() {
    wchar_t driver[MAX_PATH]{};
    if (!rx7900::installed_driver_path(driver)) {
        return 1;
    }

    const bool file_absent = GetFileAttributesW(driver) == INVALID_FILE_ATTRIBUTES &&
                             GetLastError() == ERROR_FILE_NOT_FOUND;
    rx7900::ServiceHandle manager(OpenSCManagerW(nullptr, nullptr, SC_MANAGER_CONNECT));

    if (!manager.valid()) {
        return 1;
    }

    rx7900::ServiceHandle service(OpenServiceW(manager.get(), rx7900::service_name,
                                              SERVICE_QUERY_STATUS));
    const bool service_absent = !service.valid() &&
                                GetLastError() == ERROR_SERVICE_DOES_NOT_EXIST;
    rx7900::DeviceHandle device(CreateFileW(rx7900::win32_device_name, GENERIC_READ,
        FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING, 0, nullptr),
        INVALID_HANDLE_VALUE);
    const bool device_absent = !device.valid() && GetLastError() == ERROR_FILE_NOT_FOUND;
    const bool passed = file_absent && service_absent && device_absent;
    char details[128]{};
    StringCchPrintfA(details, sizeof(details),
        "removal: file_absent=%u service_absent=%u device_absent=%u\r\n",
        file_absent ? 1u : 0u, service_absent ? 1u : 0u, device_absent ? 1u : 0u);
    OutputDebugStringA(details);
    const char* result = passed ? "RX7900_REMOVAL_AFTER_REBOOT_PASS\r\n" :
                                  "RX7900_REMOVAL_AFTER_REBOOT_FAIL\r\n";
    rx7900::DeviceHandle log(CreateFileW(L"C:\\rx7900-removal.log", GENERIC_WRITE,
        FILE_SHARE_READ, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr),
        INVALID_HANDLE_VALUE);
    size_t result_size = 0;
    StringCchLengthA(result, 128, &result_size);
    OutputDebugStringA(result);

    if (!log.valid() || !write_all(log.get(), result, static_cast<DWORD>(result_size))) {
        return 1;
    }

    return passed ? 0 : 1;
}

} // namespace

int wmain(int argc, wchar_t** argv) {
    if (argc == 2 && lstrcmpW(argv[1], L"--verify-removal") == 0) {
        return verify_removal();
    }

    SECURITY_ATTRIBUTES security{sizeof(SECURITY_ATTRIBUTES), nullptr, TRUE};
    rx7900::DeviceHandle log(CreateFileW(L"C:\\rx7900-smoke.log",
        GENERIC_WRITE | GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE,
        &security, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr), INVALID_HANDLE_VALUE);
    rx7900::DeviceHandle serial(CreateFileW(L"\\\\.\\COM2", GENERIC_WRITE, 0,
        nullptr, OPEN_EXISTING, 0, nullptr), INVALID_HANDLE_VALUE);

    if (!log.valid()) {
        return 1;
    }

    wchar_t executable[MAX_PATH]{};

    if (!rx7900::module_sibling_path(L"rx7900ctl.exe", executable)) {
        return 1;
    }

    struct Step final {
        const wchar_t* command;
        int expected;
        bool may_need_reboot;
    };

    constexpr Step steps[] = {
        {L"install", 0, false}, {L"start", 0, false}, {L"smoke", 0, false},
        {L"stop", 0, false}, {L"start", 0, false}, {L"smoke", 0, false},
        {L"uninstall", 0, true}, {L"query", 1, false}
    };
    bool passed = true;

    for (const auto& step : steps) {
        const int result = run_command(executable, step.command, log.get());
        char status[128]{};

        if (FAILED(StringCchPrintfA(status, sizeof(status),
                                   "STEP %ls: exit=%d expected=%d%s\r\n",
                                   step.command, result, step.expected,
                                   step.may_need_reboot ? " or 3010" : ""))) {
            return 1;
        }

        size_t length = 0;
        StringCchLengthA(status, sizeof(status), &length);
        write_all(log.get(), status, static_cast<DWORD>(length));
        OutputDebugStringA(status);

        if (serial.valid()) {
            write_all(serial.get(), status, static_cast<DWORD>(length));
        }
        if (result != step.expected &&
            !(step.may_need_reboot && result == ERROR_SUCCESS_REBOOT_REQUIRED)) {
            passed = false;
            run_command(executable, L"uninstall", log.get());
            break;
        }
    }

    if (passed && !schedule_removal_check()) {
        passed = false;
    }

    constexpr char pass[] = "RX7900_GUEST_SMOKE_PASS\r\n";
    constexpr char fail[] = "RX7900_GUEST_SMOKE_FAIL\r\n";
    write_all(log.get(), passed ? pass : fail, sizeof(pass) - 1);
    FlushFileBuffers(log.get());

    relay_log(log.get(), serial.get());

    return passed ? 0 : 1;
}

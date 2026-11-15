/*
Module: rx7900.tools.control
Purpose: Install/remove the demand-start driver and exercise its read-only ABI.
Created: 2026-10-21
Architecture: Native Win32 service control, with scoped handles and fixed paths.
*/
#include <windows.h>
#include <strsafe.h>
#include <stdio.h>

#include "protocol.hpp"
#include "report.hpp"
#include "windows_support.hpp"

namespace {

constexpr wchar_t service_image[] = L"\\SystemRoot\\System32\\drivers\\rx7900probe.sys";
constexpr DWORD service_timeout_ms = 10000;
constexpr DWORD poll_interval_ms = 100;

using rx7900::ServiceHandle;
using rx7900::DeviceHandle;

int report_error(const char* action, DWORD code = GetLastError()) {
    fprintf(stderr, "%s failed: Win32 error %lu\n", action, static_cast<unsigned long>(code));
    return 1;
}

int install_driver(SC_HANDLE manager) {
    ServiceHandle existing(OpenServiceW(manager, rx7900::service_name, SERVICE_QUERY_CONFIG));

    if (existing.valid()) {
        return report_error("Install (service already exists; remove it first)", ERROR_SERVICE_EXISTS);
    }
    if (GetLastError() != ERROR_SERVICE_DOES_NOT_EXIST) {
        return report_error("Check existing service");
    }

    wchar_t source[MAX_PATH]{};
    wchar_t destination[MAX_PATH]{};

    if (!rx7900::module_sibling_path(rx7900::driver_filename, source) ||
        !rx7900::installed_driver_path(destination)) {
        return report_error("Resolve package/system path", ERROR_BAD_PATHNAME);
    }
    const bool copied = CopyFileW(source, destination, TRUE) != FALSE;

    if (!copied) {
        const DWORD error = GetLastError();

        if ((error != ERROR_FILE_EXISTS && error != ERROR_ALREADY_EXISTS) ||
            !rx7900::files_equal(source, destination)) {
            return report_error("Copy driver (existing different files are preserved)", error);
        }
    }

    if (!rx7900::clear_read_only_attribute(destination)) {
        const DWORD error = GetLastError();

        if (copied) {
            DeleteFileW(destination);
        }

        return report_error("Prepare installed driver attributes", error);
    }

    ServiceHandle service(CreateServiceW(manager, rx7900::service_name,
        L"RX 7900 read-only bring-up probe", SERVICE_QUERY_STATUS,
        SERVICE_KERNEL_DRIVER, SERVICE_DEMAND_START, SERVICE_ERROR_NORMAL,
        service_image, nullptr, nullptr, nullptr, nullptr, nullptr));

    if (!service.valid()) {
        const DWORD error = GetLastError();
        // CopyFile with fail-if-exists created this file, so rollback owns it.
        if (copied) {
            DeleteFileW(destination);
        }
        return report_error("Create demand-start driver service", error);
    }

    puts("Installed read-only probe; display driver unchanged. Run: rx7900ctl start");
    return 0;
}

bool owned_service(SC_HANDLE service) {
    // QueryServiceConfig can include trailing strings, so use an aligned bounded
    // byte buffer owned for the entire call and interpretation lifetime.
    alignas(QUERY_SERVICE_CONFIGW) unsigned char storage[4096]{};
    DWORD required = 0;
    auto* config = reinterpret_cast<QUERY_SERVICE_CONFIGW*>(storage);

    if (!QueryServiceConfigW(service, config, sizeof(storage), &required)) {
        return false;
    }

    return config->dwServiceType == SERVICE_KERNEL_DRIVER &&
           config->dwStartType == SERVICE_DEMAND_START &&
           config->lpBinaryPathName != nullptr &&
           lstrcmpiW(config->lpBinaryPathName, service_image) == 0;
}

int wait_for_state(SC_HANDLE service, DWORD target) {
    const DWORD started = GetTickCount();

    do {
        SERVICE_STATUS status{};

        if (!QueryServiceStatus(service, &status)) {
            return report_error("Query service state");
        }
        if (status.dwCurrentState == target) {
            return 0;
        }
        if (target == SERVICE_RUNNING && status.dwCurrentState == SERVICE_STOPPED) {
            return report_error("Driver load", status.dwWin32ExitCode);
        }

        Sleep(poll_interval_ms);
    } while (static_cast<DWORD>(GetTickCount() - started) < service_timeout_ms);

    return report_error("Wait for service state", ERROR_TIMEOUT);
}

int stop_driver(SC_HANDLE service) {
    SERVICE_STATUS status{};

    if (!ControlService(service, SERVICE_CONTROL_STOP, &status) &&
        GetLastError() != ERROR_SERVICE_NOT_ACTIVE) {
        return report_error("Stop driver");
    }

    return wait_for_state(service, SERVICE_STOPPED);
}

int service_command(const wchar_t* command) {
    const bool install = lstrcmpW(command, L"install") == 0;
    ServiceHandle manager(OpenSCManagerW(nullptr, nullptr,
        install ? SC_MANAGER_CONNECT | SC_MANAGER_CREATE_SERVICE : SC_MANAGER_CONNECT));

    if (!manager.valid()) {
        return report_error("Open service manager (administrator rights are required)");
    }
    if (install) {
        return install_driver(manager.get());
    }

    const bool remove = lstrcmpW(command, L"uninstall") == 0;
    const bool start = lstrcmpW(command, L"start") == 0;
    const DWORD access = SERVICE_QUERY_CONFIG | SERVICE_QUERY_STATUS |
                         (start ? SERVICE_START : SERVICE_STOP) | (remove ? DELETE : 0);
    ServiceHandle service(OpenServiceW(manager.get(), rx7900::service_name, access));

    if (!service.valid()) {
        return report_error("Open driver service");
    }
    if (!owned_service(service.get())) {
        return report_error("Refuse to modify a service with different configuration",
                            ERROR_INVALID_DATA);
    }
    if (start) {
        if (!StartServiceW(service.get(), 0, nullptr) &&
            GetLastError() != ERROR_SERVICE_ALREADY_RUNNING) {
            return report_error("Start driver");
        }

        return wait_for_state(service.get(), SERVICE_RUNNING);
    }
    if (stop_driver(service.get()) != 0) {
        return 1;
    }
    if (!remove) {
        puts("Stopped read-only probe.");
        return 0;
    }
    wchar_t destination[MAX_PATH]{};

    if (!rx7900::installed_driver_path(destination)) {
        return report_error("Resolve installed path", ERROR_BAD_PATHNAME);
    }

    const DWORD attributes = GetFileAttributesW(destination);

    if (attributes != INVALID_FILE_ATTRIBUTES &&
        !rx7900::clear_read_only_attribute(destination)) {
        return report_error("Prepare driver file for removal");
    }
    if (attributes == INVALID_FILE_ATTRIBUTES && GetLastError() != ERROR_FILE_NOT_FOUND) {
        return report_error("Inspect installed driver file");
    }
    if (!DeleteService(service.get())) {
        return report_error("Remove driver service");
    }
    if (!DeleteFileW(destination)) {
        const DWORD error = GetLastError();

        if (error == ERROR_FILE_NOT_FOUND) {
            puts("Removed probe service; installed driver file was already absent.");
            return 0;
        }
        if ((error == ERROR_ACCESS_DENIED || error == ERROR_SHARING_VIOLATION) &&
            MoveFileExW(destination, nullptr, MOVEFILE_DELAY_UNTIL_REBOOT)) {
            puts("Removed probe service. Driver file removal is scheduled for reboot.");
            return ERROR_SUCCESS_REBOOT_REQUIRED;
        }

        return report_error("Remove installed driver (service already removed)", error);
    }

    puts("Removed read-only probe service and installed driver.");
    return 0;
}

int query_driver(HANDLE device) {
    rx7900::QueryRequest request{sizeof(rx7900::QueryRequest), rx7900::protocol_version};
    rx7900::Snapshot snapshot{};
    DWORD received = 0;

    if (!DeviceIoControl(device, rx7900::ioctl_query_snapshot,
        &request, sizeof(request), &snapshot,
        sizeof(snapshot), &received, nullptr)) {
        return report_error("Query read-only PCI snapshot");
    }
    if (received != sizeof(snapshot) || snapshot.size != sizeof(snapshot) ||
        snapshot.version != rx7900::protocol_version ||
        snapshot.adapter_count > rx7900::adapter_capacity) {
        return report_error("Validate snapshot ABI", ERROR_INVALID_DATA);
    }

    printf("Read-only snapshot v%u: adapters=%u omitted=%u PCI_functions=%u flags=%u\n",
        static_cast<unsigned>(snapshot.version), static_cast<unsigned>(snapshot.adapter_count),
        static_cast<unsigned>(snapshot.omitted_adapters),
        static_cast<unsigned>(snapshot.pci_functions_seen),
        static_cast<unsigned>(snapshot.feature_flags));

    for (uint32_t index = 0; index < snapshot.adapter_count; ++index) {
        rx7900::print_adapter(snapshot.adapters[index]);
    }
    if (snapshot.adapter_count == 0) {
        puts("No Navi 31 adapter visible. This is expected in a VM without passthrough.");
    }

    puts("This package does not initialize display output or acceleration.");
    return 0;
}

bool rejects_request(HANDLE device, DWORD code, rx7900::QueryRequest& request,
                     DWORD input_size, DWORD output_size) {
    rx7900::Snapshot output{};
    DWORD received = 0;

    if (DeviceIoControl(device, code, &request, input_size, &output, output_size,
                        &received, nullptr)) {
        fputs("Invalid IOCTL request was accepted\n", stderr);
        return false;
    }

    const DWORD error = GetLastError();

    if (received != 0 || (error != ERROR_INSUFFICIENT_BUFFER &&
        error != ERROR_REVISION_MISMATCH && error != ERROR_INVALID_FUNCTION)) {
        report_error("Unexpected rejected-IOCTL result", error);
        return false;
    }

    return true;
}

int device_command(bool smoke) {
    DeviceHandle device(CreateFileW(rx7900::win32_device_name, GENERIC_READ,
        FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING, 0, nullptr),
        INVALID_HANDLE_VALUE);

    if (!device.valid()) {
        return report_error("Open probe device (run rx7900ctl start first)");
    }
    if (query_driver(device.get()) != 0) {
        return 1;
    }
    if (!smoke) {
        return 0;
    }

    rx7900::QueryRequest request{sizeof(rx7900::QueryRequest), rx7900::protocol_version};

    if (!rejects_request(device.get(), rx7900::ioctl_query_snapshot, request, 0,
                         sizeof(rx7900::Snapshot)) ||
        !rejects_request(device.get(), rx7900::ioctl_query_snapshot, request,
                         sizeof(request), sizeof(request)) ||
        !rejects_request(device.get(), rx7900::ioctl_query_snapshot + 4, request,
                         sizeof(request), sizeof(rx7900::Snapshot))) {
        return 1;
    }

    ++request.version;

    if (!rejects_request(device.get(), rx7900::ioctl_query_snapshot, request,
                         sizeof(request), sizeof(rx7900::Snapshot))) {
        return 1;
    }

    puts("PASS: snapshot plus short input, short output, unknown IOCTL and ABI mismatch.");
    return 0;
}

} // namespace

int wmain(int argc, wchar_t** argv) {
    if (argc == 2) {
        const wchar_t* command = argv[1];

        if (lstrcmpW(command, L"query") == 0 || lstrcmpW(command, L"smoke") == 0) {
            return device_command(lstrcmpW(command, L"smoke") == 0);
        }
        if (lstrcmpW(command, L"install") == 0 || lstrcmpW(command, L"start") == 0 ||
            lstrcmpW(command, L"stop") == 0 || lstrcmpW(command, L"uninstall") == 0) {
            return service_command(command);
        }
    }

    puts("RX7900 read-only diagnostic driver installer/client");
    puts("Usage: rx7900ctl install|start|query|smoke|stop|uninstall");
    puts("Run in a matching-architecture ReactOS test guest as administrator.");
    return 2;
}

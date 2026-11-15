/*
Module: rx7900.tools.radeon_win32_system
Purpose: Implement the Radeon installer seam with native Win32/SetupAPI calls.
Created: 2026-10-24
Architecture: Every function translates one SystemOps operation and returns the
Win32 error code. Registry and SetupAPI buffers are bounded and type checked
before use; handles are scoped through windows_support/pnp_support owners.
*/
#include "radeon_win32_system.hpp"

#include <newdev.h>
#include <stdio.h>
#include <strsafe.h>

#include "pnp_support.hpp"
#include "radeon_opengl_info.hpp"

namespace rx7900::tools::radeon {
namespace {

using rx7900::DeviceHandle;
using rx7900::DeviceSetHandle;
using rx7900::RegistryHandle;
using rx7900::ServiceHandle;

constexpr DWORD id_list_characters = 4096;

int release_desktop_dc(HDC dc) noexcept {
    return ReleaseDC(nullptr, dc);
}

uint32_t read_value(void*, const wchar_t* key, const wchar_t* name, uint32_t& type, void* data,
                    uint32_t capacity, uint32_t& bytes) {
    HKEY opened = nullptr;
    LONG status = RegOpenKeyExW(HKEY_LOCAL_MACHINE, key, 0, KEY_QUERY_VALUE, &opened);

    if (status != ERROR_SUCCESS) {
        return static_cast<uint32_t>(status);
    }

    RegistryHandle owner(opened);
    DWORD value_type = 0;
    DWORD size = capacity;
    status = RegQueryValueExW(owner.get(), name, nullptr, &value_type, static_cast<BYTE*>(data), &size);
    type = value_type;
    bytes = size;
    return static_cast<uint32_t>(status);
}

uint32_t write_value(void*, const wchar_t* key, const wchar_t* name, uint32_t type, const void* data,
                     uint32_t bytes) {
    HKEY created = nullptr;
    LONG status = RegCreateKeyExW(HKEY_LOCAL_MACHINE, key, 0, nullptr, REG_OPTION_NON_VOLATILE,
                                  KEY_SET_VALUE | KEY_QUERY_VALUE, nullptr, &created, nullptr);

    if (status != ERROR_SUCCESS) {
        return static_cast<uint32_t>(status);
    }

    RegistryHandle owner(created);
    status = RegSetValueExW(owner.get(), name, 0, type, static_cast<const BYTE*>(data), bytes);

    // The journal is the recovery record: make each of its writes durable.
    if (status == ERROR_SUCCESS && lstrcmpiW(key, journal_key) == 0) {
        status = RegFlushKey(owner.get());
    }
    return static_cast<uint32_t>(status);
}

uint32_t delete_value(void*, const wchar_t* key, const wchar_t* name) {
    HKEY opened = nullptr;
    LONG status = RegOpenKeyExW(HKEY_LOCAL_MACHINE, key, 0, KEY_SET_VALUE, &opened);

    if (status != ERROR_SUCCESS) {
        return static_cast<uint32_t>(status);
    }

    RegistryHandle owner(opened);
    status = RegDeleteValueW(owner.get(), name);
    if (status == ERROR_SUCCESS && lstrcmpiW(key, journal_key) == 0) {
        status = RegFlushKey(owner.get());
    }
    return static_cast<uint32_t>(status);
}

uint32_t key_exists(void*, const wchar_t* key, bool& exists) {
    HKEY opened = nullptr;
    const LONG status = RegOpenKeyExW(HKEY_LOCAL_MACHINE, key, 0, KEY_QUERY_VALUE, &opened);

    exists = status == ERROR_SUCCESS;
    if (exists) {
        RegCloseKey(opened);
        return ERROR_SUCCESS;
    }
    return status == ERROR_FILE_NOT_FOUND || status == ERROR_PATH_NOT_FOUND ? ERROR_SUCCESS
                                                                           : static_cast<uint32_t>(status);
}

uint32_t delete_key(void*, const wchar_t* key) {
    return static_cast<uint32_t>(RegDeleteKeyW(HKEY_LOCAL_MACHINE, key));
}

uint32_t file_size(void*, const wchar_t* path, bool& exists, uint64_t& bytes) {
    WIN32_FILE_ATTRIBUTE_DATA data{};
    exists = false;
    bytes = 0;

    if (!GetFileAttributesExW(path, GetFileExInfoStandard, &data)) {
        const DWORD error = GetLastError();
        return error == ERROR_FILE_NOT_FOUND || error == ERROR_PATH_NOT_FOUND ? ERROR_SUCCESS : error;
    }
    if ((data.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) != 0) {
        return ERROR_FILE_EXISTS;
    }

    exists = true;
    bytes = (static_cast<uint64_t>(data.nFileSizeHigh) << 32) | data.nFileSizeLow;
    return ERROR_SUCCESS;
}

uint32_t read_file(void*, const wchar_t* path, uint64_t offset, void* buffer, uint32_t capacity,
                   uint32_t& received) {
    received = 0;
    DeviceHandle file(CreateFileW(path, GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
                                  nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr),
                      INVALID_HANDLE_VALUE);

    if (!file.valid()) {
        return GetLastError();
    }

    LARGE_INTEGER position{};
    position.QuadPart = static_cast<LONGLONG>(offset);
    DWORD count = 0;

    // The caller owns capacity writable bytes at buffer for this call only.
    if (!SetFilePointerEx(file.get(), position, nullptr, FILE_BEGIN) ||
        !ReadFile(file.get(), buffer, capacity, &count, nullptr)) {
        return GetLastError();
    }
    received = count;
    return ERROR_SUCCESS;
}

uint32_t flush_file(const wchar_t* path) noexcept {
    DeviceHandle file(CreateFileW(path, GENERIC_WRITE, FILE_SHARE_READ, nullptr, OPEN_EXISTING,
                                  FILE_ATTRIBUTE_NORMAL, nullptr),
                      INVALID_HANDLE_VALUE);

    if (!file.valid()) {
        return GetLastError();
    }
    return FlushFileBuffers(file.get()) ? ERROR_SUCCESS : GetLastError();
}

uint32_t copy_file(void*, const wchar_t* from, const wchar_t* to) {
    if (!CopyFileW(from, to, FALSE)) {
        return GetLastError();
    }
    // Package media may be read-only; the installed copy must stay replaceable.
    if (!rx7900::clear_read_only_attribute(to)) {
        return GetLastError();
    }
    return flush_file(to);
}

uint32_t move_file(void*, const wchar_t* from, const wchar_t* to) {
    return MoveFileExW(from, to, MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH) ? ERROR_SUCCESS
                                                                                      : GetLastError();
}

uint32_t replace_on_restart(void*, const wchar_t* from, const wchar_t* to) {
    const DWORD flags = MOVEFILE_DELAY_UNTIL_REBOOT | (to != nullptr ? MOVEFILE_REPLACE_EXISTING : 0);
    return MoveFileExW(from, to, flags) ? ERROR_SUCCESS : GetLastError();
}

uint32_t delete_file(void*, const wchar_t* path) {
    const DWORD attributes = GetFileAttributesW(path);

    if (attributes == INVALID_FILE_ATTRIBUTES) {
        return GetLastError();
    }
    if ((attributes & FILE_ATTRIBUTE_READONLY) != 0 && !rx7900::clear_read_only_attribute(path)) {
        return GetLastError();
    }
    return DeleteFileW(path) ? ERROR_SUCCESS : GetLastError();
}

uint32_t create_directory(void*, const wchar_t* path) {
    if (CreateDirectoryW(path, nullptr)) {
        return ERROR_SUCCESS;
    }

    const DWORD error = GetLastError();
    const DWORD attributes = GetFileAttributesW(path);

    return error == ERROR_ALREADY_EXISTS && attributes != INVALID_FILE_ATTRIBUTES &&
                   (attributes & FILE_ATTRIBUTE_DIRECTORY) != 0 ? ERROR_SUCCESS : error;
}

// QueryServiceConfig returns pointers into this caller-owned bounded buffer.
// Validate the complete pointer/string span before copying.
bool copy_config_string(const BYTE* storage, size_t bytes, const wchar_t* source,
                        wchar_t (&output)[path_characters]) noexcept {
    output[0] = 0;
    if (source == nullptr) {
        return true;
    }

    const uintptr_t begin = reinterpret_cast<uintptr_t>(storage);
    const uintptr_t address = reinterpret_cast<uintptr_t>(source);

    if (address < begin || address >= begin + bytes || (address - begin) % sizeof(wchar_t) != 0) {
        return false;
    }

    const size_t characters = (begin + bytes - address) / sizeof(wchar_t);

    for (size_t index = 0; index < characters && index < path_characters; ++index) {
        output[index] = source[index];
        if (source[index] == 0) {
            return true;
        }
    }
    output[0] = 0;
    return false;
}

uint32_t query_service(void*, ServiceInfo& output) {
    output = ServiceInfo{};
    ServiceHandle manager(OpenSCManagerW(nullptr, nullptr, SC_MANAGER_CONNECT));

    if (!manager.valid()) {
        return GetLastError();
    }

    ServiceHandle service(OpenServiceW(manager.get(), service_name, SERVICE_QUERY_CONFIG | SERVICE_QUERY_STATUS));

    if (!service.valid()) {
        const DWORD error = GetLastError();
        return error == ERROR_SERVICE_DOES_NOT_EXIST ? ERROR_SUCCESS : error;
    }

    alignas(QUERY_SERVICE_CONFIGW) BYTE storage[8192]{};
    auto* config = reinterpret_cast<QUERY_SERVICE_CONFIGW*>(storage);
    DWORD required = 0;

    if (!QueryServiceConfigW(service.get(), config, sizeof(storage), &required)) {
        return GetLastError();
    }
    if (!copy_config_string(storage, sizeof(storage), config->lpBinaryPathName, output.image_path)) {
        return ERROR_INVALID_DATA;
    }

    SERVICE_STATUS status{};
    if (!QueryServiceStatus(service.get(), &status)) {
        return GetLastError();
    }

    output.exists = true;
    output.type = config->dwServiceType;
    output.start = config->dwStartType;
    output.error_control = config->dwErrorControl;
    output.running = status.dwCurrentState != SERVICE_STOPPED;
    return ERROR_SUCCESS;
}

uint32_t create_service(void*) {
    ServiceHandle manager(OpenSCManagerW(nullptr, nullptr, SC_MANAGER_CONNECT | SC_MANAGER_CREATE_SERVICE));

    if (!manager.valid()) {
        return GetLastError();
    }

    ServiceHandle service(CreateServiceW(manager.get(), service_name, service_display_name, SERVICE_QUERY_STATUS,
                                         SERVICE_KERNEL_DRIVER, SERVICE_DEMAND_START, SERVICE_ERROR_IGNORE,
                                         service_image_systemroot, service_group, nullptr, nullptr, nullptr,
                                         nullptr));
    return service.valid() ? ERROR_SUCCESS : GetLastError();
}

uint32_t set_demand_start(void*) {
    ServiceHandle manager(OpenSCManagerW(nullptr, nullptr, SC_MANAGER_CONNECT));

    if (!manager.valid()) {
        return GetLastError();
    }

    ServiceHandle service(OpenServiceW(manager.get(), service_name, SERVICE_CHANGE_CONFIG));

    if (!service.valid()) {
        return GetLastError();
    }
    return ChangeServiceConfigW(service.get(), SERVICE_NO_CHANGE, SERVICE_DEMAND_START, SERVICE_NO_CHANGE,
                                nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr)
               ? ERROR_SUCCESS : GetLastError();
}

uint32_t delete_service(void*) {
    ServiceHandle manager(OpenSCManagerW(nullptr, nullptr, SC_MANAGER_CONNECT));

    if (!manager.valid()) {
        return GetLastError();
    }

    ServiceHandle service(OpenServiceW(manager.get(), service_name, DELETE));

    if (!service.valid()) {
        return GetLastError();
    }
    return DeleteService(service.get()) ? ERROR_SUCCESS : GetLastError();
}

// Reads one REG_MULTI_SZ device property into a bounded list. Absent
// properties report zero characters; oversize or mistyped ones fail.
template<size_t Capacity>
uint32_t multi_property(HDEVINFO devices, SP_DEVINFO_DATA& device, DWORD property, wchar_t (&output)[Capacity],
                        size_t& characters) noexcept {
    DWORD type = 0;
    DWORD bytes = 0;
    characters = 0;
    output[0] = 0;

    if (!SetupDiGetDeviceRegistryPropertyW(devices, &device, property, &type, reinterpret_cast<PBYTE>(output),
                                           sizeof(output), &bytes)) {
        const DWORD error = GetLastError();
        return error == ERROR_INVALID_DATA || error == ERROR_FILE_NOT_FOUND ? ERROR_SUCCESS : error;
    }
    if (type != REG_MULTI_SZ || bytes < 2 * sizeof(wchar_t) || bytes > sizeof(output) ||
        bytes % sizeof(wchar_t) != 0) {
        return ERROR_INVALID_DATA;
    }
    characters = bytes / sizeof(wchar_t);
    return ERROR_SUCCESS;
}

uint32_t enumerate_devices(void*, DeviceInventory& output) {
    output = DeviceInventory{};
    DeviceSetHandle devices(SetupDiGetClassDevsW(nullptr, nullptr, nullptr, DIGCF_ALLCLASSES | DIGCF_PRESENT),
                            INVALID_HANDLE_VALUE);

    if (!devices.valid()) {
        return GetLastError();
    }

    for (DWORD index = 0; index < rx7900::maximum_pnp_devices; ++index) {
        SP_DEVINFO_DATA device{};
        device.cbSize = sizeof(device);

        if (!SetupDiEnumDeviceInfo(devices.get(), index, &device)) {
            const DWORD error = GetLastError();
            return error == ERROR_NO_MORE_ITEMS ? ERROR_SUCCESS : error;
        }

        wchar_t hardware[id_list_characters]{};
        wchar_t compatible[id_list_characters]{};
        size_t hardware_characters = 0;
        size_t compatible_characters = 0;

        if (multi_property(devices.get(), device, SPDRP_HARDWAREID, hardware, hardware_characters) !=
                ERROR_SUCCESS ||
            multi_property(devices.get(), device, SPDRP_COMPATIBLEIDS, compatible, compatible_characters) !=
                ERROR_SUCCESS) {
            continue; // Not a parseable PCI identity; never a binding candidate.
        }

        const bool lightning = multi_string_contains(hardware, hardware_characters, lightning_hardware_id) ||
            multi_string_contains(compatible, compatible_characters, lightning_hardware_id);

        DeviceRecord record{};
        const bool navi31 =
            navi31_identity(hardware, hardware_characters, record.device_id, record.revision, record.hardware_id) ||
            navi31_identity(compatible, compatible_characters, record.device_id, record.revision,
                            record.hardware_id);

        if (!lightning && !navi31) {
            continue;
        }

        // ReactOS returns every recorded device instance for a "present devices"
        // query, attached or not. A record without a live device node is a stale
        // registry entry, such as this card seen earlier at another PCI location;
        // it cannot be bound and must not count as a second GPU.
        ULONG status = 0;
        ULONG problem = 0;
        const CONFIGRET live = CM_Get_DevNode_Status(&status, &problem, device.DevInst, 0);
        if (live == CR_NO_SUCH_DEVINST) {
            output.absent += navi31 ? 1 : 0;
            continue;
        }

        output.lightning_present = output.lightning_present || lightning;
        if (!navi31) {
            continue;
        }
        if (output.count == device_capacity) {
            ++output.omitted;
            continue;
        }

        DWORD required = 0;
        if (!SetupDiGetDeviceInstanceIdW(devices.get(), &device, record.instance_id, instance_characters, &required)) {
            return GetLastError();
        }

        DWORD type = 0;
        DWORD bytes = 0;
        if (SetupDiGetDeviceRegistryPropertyW(devices.get(), &device, SPDRP_SERVICE, &type,
                                              reinterpret_cast<PBYTE>(record.service), sizeof(record.service),
                                              &bytes)) {
            if (type != REG_SZ || bytes < sizeof(wchar_t) || bytes % sizeof(wchar_t) != 0 ||
                record.service[bytes / sizeof(wchar_t) - 1] != 0) {
                return ERROR_INVALID_DATA;
            }
        } else {
            const DWORD error = GetLastError();
            if (error != ERROR_INVALID_DATA && error != ERROR_FILE_NOT_FOUND) {
                return error;
            }
            record.service[0] = 0;
        }

        if (live == CR_SUCCESS) {
            record.started = (status & DN_STARTED) != 0;
            record.problem = (status & DN_HAS_PROBLEM) != 0 ? problem : 0;
        }

        const uint32_t upper = multi_property(devices.get(), device, SPDRP_UPPERFILTERS, record.upper_filters,
                                              record.upper_characters);
        const uint32_t lower = multi_property(devices.get(), device, SPDRP_LOWERFILTERS, record.lower_filters,
                                              record.lower_characters);
        if (upper != ERROR_SUCCESS || lower != ERROR_SUCCESS) {
            // Filters that cannot be read completely can never be preserved.
            return upper != ERROR_SUCCESS ? upper : lower;
        }

        output.devices[output.count++] = record;
    }

    return ERROR_BUFFER_OVERFLOW;
}

// Opens one exact device instance in a new set the caller keeps alive.
uint32_t open_instance(const wchar_t* instance_id, DeviceSetHandle& set, SP_DEVINFO_DATA& device) noexcept {
    if (!set.valid()) {
        return GetLastError();
    }
    device = SP_DEVINFO_DATA{};
    device.cbSize = sizeof(device);
    return SetupDiOpenDeviceInfoW(set.get(), instance_id, nullptr, 0, &device) ? ERROR_SUCCESS : GetLastError();
}

// Installs the package INF on one device instance through the class installer.
// UpdateDriverForPlugAndPlayDevices is not used: ReactOS applies it to every
// recorded instance with the hardware ID, attached or not, so a stale record of
// the card would be bound too. DI_DONOTCALLCONFIGMG keeps the running display
// device untouched; the new driver takes effect at the next system start.
uint32_t install_driver(void*, const wchar_t* instance_id, const wchar_t*, const wchar_t* inf_path,
                        bool& restart) {
    restart = true;

    DeviceSetHandle set(SetupDiCreateDeviceInfoList(nullptr, nullptr), INVALID_HANDLE_VALUE);
    SP_DEVINFO_DATA device{};
    const uint32_t opened = open_instance(instance_id, set, device);

    if (opened != ERROR_SUCCESS) {
        return opened;
    }

    SP_DEVINSTALL_PARAMS_W parameters{};
    parameters.cbSize = sizeof(parameters);
    if (!SetupDiGetDeviceInstallParamsW(set.get(), &device, &parameters)) {
        return GetLastError();
    }
    parameters.Flags |= DI_ENUMSINGLEINF | DI_DONOTCALLCONFIGMG | DI_QUIETINSTALL;
    parameters.FlagsEx |= DI_FLAGSEX_ALLOWEXCLUDEDDRVS;
    if (FAILED(StringCchCopyW(parameters.DriverPath, MAX_PATH, inf_path))) {
        return ERROR_BUFFER_OVERFLOW;
    }
    if (!SetupDiSetDeviceInstallParamsW(set.get(), &device, &parameters) ||
        !SetupDiBuildDriverInfoList(set.get(), &device, SPDIT_COMPATDRIVER)) {
        return GetLastError();
    }

    uint32_t result = ERROR_SUCCESS;
    SP_DRVINFO_DATA_W driver{};
    driver.cbSize = sizeof(driver);
    if (!SetupDiEnumDriverInfoW(set.get(), &device, SPDIT_COMPATDRIVER, 0, &driver)) {
        result = GetLastError();
        result = result == ERROR_NO_MORE_ITEMS ? static_cast<uint32_t>(ERROR_NO_COMPAT_DRIVERS) : result;
    }

    constexpr DI_FUNCTION steps[]{DIF_SELECTBESTCOMPATDRV, DIF_ALLOW_INSTALL, DIF_INSTALLDEVICEFILES,
                                  DIF_REGISTER_COINSTALLERS, DIF_INSTALLINTERFACES, DIF_INSTALLDEVICE};
    for (const DI_FUNCTION step : steps) {
        if (result != ERROR_SUCCESS) {
            break;
        }
        if (!SetupDiCallClassInstaller(step, set.get(), &device)) {
            const DWORD failed = GetLastError();
            result = failed == ERROR_DI_DO_DEFAULT ? static_cast<uint32_t>(ERROR_SUCCESS) : failed;
        }
    }

    SetupDiDestroyDriverInfoList(set.get(), &device, SPDIT_COMPATDRIVER);
    return result;
}

uint32_t remove_device(void*, const wchar_t* instance_id) {
    DeviceSetHandle set(SetupDiCreateDeviceInfoList(nullptr, nullptr), INVALID_HANDLE_VALUE);
    SP_DEVINFO_DATA device{};
    const uint32_t opened = open_instance(instance_id, set, device);

    if (opened != ERROR_SUCCESS) {
        return opened;
    }
    // Defense in depth: only the device whose functional driver is ours.
    if (!rx7900::pnp_service_matches(set.get(), device, service_name, false)) {
        return ERROR_ACCESS_DENIED;
    }

    SP_REMOVEDEVICE_PARAMS parameters{};
    parameters.ClassInstallHeader.cbSize = sizeof(SP_CLASSINSTALL_HEADER);
    parameters.ClassInstallHeader.InstallFunction = DIF_REMOVE;
    parameters.Scope = DI_REMOVEDEVICE_GLOBAL;

    if (!SetupDiSetClassInstallParamsW(set.get(), &device, &parameters.ClassInstallHeader, sizeof(parameters)) ||
        !SetupDiCallClassInstaller(DIF_REMOVE, set.get(), &device)) {
        return GetLastError();
    }
    return ERROR_SUCCESS;
}

// DIF_REMOVE failed with ERROR_CALL_NOT_IMPLEMENTED (ReactOS): the binding is
// undone in place by pnp_support's unbind_pnp_device. The device's filters are
// kept (they may belong to other drivers; the engine records that).
uint32_t unbind_device(void*, const wchar_t* instance_id, UnbindStep& failed) {
    DeviceSetHandle set(SetupDiCreateDeviceInfoList(nullptr, nullptr), INVALID_HANDLE_VALUE);
    SP_DEVINFO_DATA device{};
    failed = UnbindStep::open_device;
    const uint32_t opened = open_instance(instance_id, set, device);

    if (opened != ERROR_SUCCESS) {
        return opened;
    }
    return rx7900::unbind_pnp_device(set.get(), device, service_name, false, failed);
}

uint32_t write_filters(void*, const wchar_t* instance_id, bool upper, const wchar_t* list, size_t characters) {
    DeviceSetHandle set(SetupDiCreateDeviceInfoList(nullptr, nullptr), INVALID_HANDLE_VALUE);
    SP_DEVINFO_DATA device{};
    const uint32_t opened = open_instance(instance_id, set, device);

    if (opened != ERROR_SUCCESS) {
        return opened;
    }
    if (characters < 2 || characters > filter_characters) {
        return ERROR_INVALID_PARAMETER;
    }

    const DWORD property = upper ? SPDRP_UPPERFILTERS : SPDRP_LOWERFILTERS;
    const DWORD bytes = static_cast<DWORD>(characters * sizeof(wchar_t));

    if (!SetupDiSetDeviceRegistryPropertyW(set.get(), &device, property, reinterpret_cast<const BYTE*>(list),
                                           bytes)) {
        return GetLastError();
    }

    // Read the complete property back; a partial write is reported, not trusted.
    wchar_t observed[filter_characters]{};
    size_t observed_characters = 0;
    const uint32_t result = multi_property(set.get(), device, property, observed, observed_characters);

    if (result != ERROR_SUCCESS) {
        return result;
    }
    if (observed_characters != characters) {
        return ERROR_READ_FAULT;
    }
    for (size_t index = 0; index < characters; ++index) {
        if (observed[index] != list[index]) {
            return ERROR_READ_FAULT;
        }
    }
    return ERROR_SUCCESS;
}

uint32_t driver_value(void*, const wchar_t* instance_id, const wchar_t* name, wchar_t* output, size_t capacity) {
    if (output == nullptr || capacity == 0) {
        return ERROR_INVALID_PARAMETER;
    }
    output[0] = 0;

    DeviceSetHandle set(SetupDiCreateDeviceInfoList(nullptr, nullptr), INVALID_HANDLE_VALUE);
    SP_DEVINFO_DATA device{};
    const uint32_t opened = open_instance(instance_id, set, device);

    if (opened != ERROR_SUCCESS) {
        return opened;
    }

    RegistryHandle key(SetupDiOpenDevRegKey(set.get(), &device, DICS_FLAG_GLOBAL, 0, DIREG_DRV, KEY_QUERY_VALUE),
                       reinterpret_cast<HKEY>(INVALID_HANDLE_VALUE));

    if (!key.valid()) {
        return GetLastError();
    }

    DWORD type = 0;
    DWORD bytes = static_cast<DWORD>(capacity * sizeof(wchar_t));
    const LONG status = RegQueryValueExW(key.get(), name, nullptr, &type, reinterpret_cast<BYTE*>(output), &bytes);

    if (status != ERROR_SUCCESS) {
        output[0] = 0;
        return static_cast<uint32_t>(status);
    }

    const size_t characters = bytes / sizeof(wchar_t);

    if ((type != REG_SZ && type != REG_MULTI_SZ) || bytes % sizeof(wchar_t) != 0 || characters == 0 ||
        characters > capacity || output[characters - 1] != 0) {
        output[0] = 0;
        return ERROR_INVALID_DATA;
    }
    return ERROR_SUCCESS;
}

uint32_t uninstall_oem_inf(void*, const wchar_t* inf_name, bool force) {
    // Only a bare oemNN.inf name that SetupAPI itself assigned is accepted.
    if (!oem_inf_name(inf_name)) {
        return ERROR_INVALID_PARAMETER;
    }
    if (force) {
        // A package staged from read-only media keeps FILE_ATTRIBUTE_READONLY, and a
        // forced uninstall that is a bare DeleteFileW (ReactOS) is then refused.
        wchar_t path[MAX_PATH]{};
        const UINT length = GetWindowsDirectoryW(path, MAX_PATH);
        if (length != 0 && length < MAX_PATH && SUCCEEDED(StringCchCatW(path, MAX_PATH, L"\\inf\\")) &&
            SUCCEEDED(StringCchCatW(path, MAX_PATH, inf_name))) {
            const DWORD attributes = GetFileAttributesW(path);
            if (attributes != INVALID_FILE_ATTRIBUTES && (attributes & FILE_ATTRIBUTE_READONLY) != 0 &&
                !SetFileAttributesW(path, attributes & ~static_cast<DWORD>(FILE_ATTRIBUTE_READONLY))) {
                return GetLastError();
            }
        }
    }
    return SetupUninstallOEMInfW(inf_name, force ? SUOI_FORCEDELETE : 0, nullptr) ? ERROR_SUCCESS : GetLastError();
}

uint32_t active_display(void*, ActiveDisplay& output) {
    namespace opengl = rx7900::reactos::radeon_opengl;
    output = ActiveDisplay{};

    rx7900::ScopedHandle<HDC, release_desktop_dc> screen(GetDC(nullptr));

    if (screen.valid()) {
        DWORD query = opengl::opengl_getinfo;

        if (ExtEscape(screen.get(), static_cast<int>(opengl::query_escape_support), sizeof(query),
                      reinterpret_cast<LPCSTR>(&query), 0, nullptr) > 0) {
            DWORD selector = opengl::getinfo_driver_name;
            opengl::OpenglInfo information{};
            const int returned = ExtEscape(screen.get(), static_cast<int>(opengl::opengl_getinfo), sizeof(selector),
                                           reinterpret_cast<LPCSTR>(&selector), sizeof(information),
                                           reinterpret_cast<LPSTR>(&information));
            if (returned > 0) {
                for (size_t index = 0; index + 1 < sizeof(output.opengl_driver) / sizeof(wchar_t) &&
                                       index < opengl::max_path && information.driver_name[index] != 0;
                     ++index) {
                    output.opengl_driver[index] = static_cast<wchar_t>(information.driver_name[index]);
                }
            }
        }
    }

    // \Device\Video0 names the registry path VideoPort registered for the
    // primary adapter: ...\Services\<service>\Device0 or the Control\Video key.
    wchar_t path[path_characters]{};
    uint32_t type = 0;
    uint32_t bytes = 0;

    if (read_value(nullptr, device_map_video_key, L"\\Device\\Video0", type, path, sizeof(path) - sizeof(wchar_t),
                   bytes) != ERROR_SUCCESS || type != REG_SZ) {
        return ERROR_SUCCESS;
    }
    path[path_characters - 1] = 0;

    const size_t length = text_length(path, path_characters);
    constexpr wchar_t services[] = L"\\services\\";
    constexpr size_t services_length = sizeof(services) / sizeof(wchar_t) - 1;

    for (size_t start = 0; start + services_length < length; ++start) {
        size_t matched = 0;
        while (matched < services_length) {
            wchar_t value = path[start + matched];
            value = value >= L'A' && value <= L'Z' ? static_cast<wchar_t>(value + 32) : value;
            if (value != services[matched]) {
                break;
            }
            ++matched;
        }
        if (matched == services_length) {
            size_t index = 0;
            for (size_t cursor = start + services_length;
                 cursor < length && path[cursor] != L'\\' && index + 1 < service_name_characters; ++cursor) {
                output.primary_service[index++] = path[cursor];
            }
            output.primary_service[index] = 0;
            return ERROR_SUCCESS;
        }
    }

    // New-key layout: \Registry\Machine\System\...\Control\Video\{id}\0000 whose
    // sibling Video subkey carries the Service value.
    constexpr wchar_t machine[] = L"\\Registry\\Machine\\";
    constexpr size_t machine_length = sizeof(machine) / sizeof(wchar_t) - 1;
    wchar_t prefix[machine_length + 1]{};

    for (size_t index = 0; index < machine_length && index < length; ++index) {
        prefix[index] = path[index];
    }
    if (length > machine_length && text_equal_insensitive(prefix, machine)) {
        wchar_t key[path_characters]{};
        text_copy(key, path + machine_length);
        size_t end = text_length(key, path_characters);
        while (end > 0 && key[end - 1] != L'\\') {
            --end;
        }
        if (end > 0) {
            key[end] = 0;
            if (text_append(key, L"Video")) {
                read_value(nullptr, key, L"Service", type, output.primary_service,
                           sizeof(output.primary_service) - sizeof(wchar_t), bytes);
                output.primary_service[service_name_characters - 1] = 0;
                if (type != REG_SZ) {
                    output.primary_service[0] = 0;
                }
            }
        }
    }
    return ERROR_SUCCESS;
}

// SYSTEM_TIMEOFDAY_INFORMATION.BootTime (NtQuerySystemInformation class 3),
// resolved from ntdll at run time; ReactOS and Windows NT 5 implement it.
uint32_t boot_id(void*, uint64_t& output) {
    using Query = LONG(WINAPI*)(ULONG, PVOID, ULONG, PULONG);
    const HMODULE ntdll = GetModuleHandleW(L"ntdll.dll");
    const auto query = ntdll != nullptr ?
        reinterpret_cast<Query>(reinterpret_cast<void*>(GetProcAddress(ntdll, "NtQuerySystemInformation"))) : nullptr;

    if (query == nullptr) {
        return ERROR_PROC_NOT_FOUND;
    }

    // BootTime, CurrentTime, TimeZoneBias, TimeZoneId, Reserved and the two
    // bias fields; 48 bytes is the largest NT layout.
    alignas(8) unsigned char information[48]{};
    ULONG returned = 0;
    if (query(3, information, sizeof(information), &returned) < 0 || returned < 8) {
        return ERROR_NOT_SUPPORTED;
    }

    uint64_t value = 0;
    for (size_t index = 0; index < 8; ++index) {
        value |= static_cast<uint64_t>(information[index]) << (8 * index);
    }
    output = value;
    return ERROR_SUCCESS;
}

void print(void*, const char* line) {
    fputs(line, stdout);
    fputc('\n', stdout);
    fflush(stdout);
}

} // namespace

bool make_win32_system(Win32System& state, SystemOps& ops) noexcept {
    const UINT system_length = GetSystemDirectoryW(state.system_directory, MAX_PATH);

    if (system_length == 0 || system_length >= MAX_PATH) {
        return false;
    }

    const UINT windows_length = GetWindowsDirectoryW(state.inf_directory, MAX_PATH);

    if (windows_length == 0 || windows_length >= MAX_PATH || !text_append(state.inf_directory, L"\\inf")) {
        return false;
    }

    const DWORD module_length = GetModuleFileNameW(nullptr, state.package_directory, MAX_PATH);

    if (module_length == 0 || module_length >= MAX_PATH) {
        return false;
    }

    // Keep the directory of the returned, terminated module path.
    DWORD cursor = module_length;
    while (cursor != 0 && state.package_directory[cursor - 1] != L'\\') {
        --cursor;
    }
    if (cursor <= 1) {
        return false;
    }
    state.package_directory[cursor - 1] = 0;

    ops = SystemOps{};
    ops.context = &state;
    ops.read_value = read_value;
    ops.write_value = write_value;
    ops.delete_value = delete_value;
    ops.key_exists = key_exists;
    ops.delete_key = delete_key;
    ops.file_size = file_size;
    ops.read_file = read_file;
    ops.copy_file = copy_file;
    ops.move_file = move_file;
    ops.replace_on_restart = replace_on_restart;
    ops.delete_file = delete_file;
    ops.create_directory = create_directory;
    ops.query_service = query_service;
    ops.create_service = create_service;
    ops.set_demand_start = set_demand_start;
    ops.delete_service = delete_service;
    ops.enumerate_devices = enumerate_devices;
    ops.install_driver = install_driver;
    ops.remove_device = remove_device;
    ops.unbind_device = unbind_device;
    ops.write_filters = write_filters;
    ops.driver_value = driver_value;
    ops.uninstall_oem_inf = uninstall_oem_inf;
    ops.active_display = active_display;
    ops.boot_id = boot_id;
    ops.print = print;
    ops.system_directory = state.system_directory;
    ops.inf_directory = state.inf_directory;
    ops.package_directory = state.package_directory;
#if defined(_M_AMD64) || defined(__x86_64__)
    ops.machine = machine_amd64;
#else
    ops.machine = machine_i386;
#endif
    return true;
}

} // namespace rx7900::tools::radeon

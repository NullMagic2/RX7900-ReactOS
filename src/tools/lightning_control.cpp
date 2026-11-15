/*
Module: rx7900.tools.lightning_control
Purpose: Install the NT5 Lightning PnP transport and query its observed host caps.
Created: 2026-10-21
Architecture: Reuses native PnP/handle helpers and the driver's fixed-width ABI.
*/
#include <windows.h>
#include <newdev.h>
#include <stdio.h>

#include "lightning_abi.hpp"
#include "lightning_present_abi.hpp"
#include "lightning-display.h"
#include "lightning_opengl_icd.hpp"
#include "lightning_control_policy.hpp"
#include "pnp_support.hpp"

namespace {

namespace transport = lightning::reactos;
constexpr wchar_t inf_filename[] = L"lightningnt5.inf";
constexpr wchar_t display_inf_filename[] = L"lightningdisplay.inf";
constexpr wchar_t display_service_name[] = L"LightningVideo";
constexpr wchar_t display_class_guid[] = L"{4D36E968-E325-11CE-BFC1-08002BE10318}";
constexpr int reboot_required_exit = 3010;
// Records written only by display-primary-disable, consumed by display-primary-restore.
constexpr wchar_t primary_rollback_key[] = L"SOFTWARE\\QemuLightning\\DisplayPrimaryRollback";

int report_error(const char* action, DWORD error = GetLastError()) noexcept {
    fprintf(stderr, "%s failed: Win32 error %lu\n", action,
            static_cast<unsigned long>(error));
    return 1;
}

bool nt5_system() noexcept {
    OSVERSIONINFOEXW version{};
    version.dwOSVersionInfoSize = sizeof(version);
    version.dwMajorVersion = 5;
    version.dwPlatformId = VER_PLATFORM_WIN32_NT;
    const DWORD fields = VER_MAJORVERSION | VER_PLATFORMID;
    ULONGLONG conditions = VerSetConditionMask(0, VER_MAJORVERSION, VER_EQUAL);
    conditions = VerSetConditionMask(conditions, VER_PLATFORMID, VER_EQUAL);
    return VerifyVersionInfoW(&version, fields, conditions) != FALSE;
}

int query_capabilities() noexcept {
    rx7900::DeviceHandle device(CreateFileW(transport::win32_device_name,
        GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL, nullptr), INVALID_HANDLE_VALUE);

    if (!device.valid()) {
        return report_error("Open Lightning NT5 transport");
    }

    transport::LightningCapabilities caps{};
    DWORD returned = 0;

    if (!DeviceIoControl(device.get(), transport::ioctl_query_capabilities,
            nullptr, 0, &caps, sizeof(caps), &returned, nullptr)) {
        return report_error("Query Lightning host capabilities");
    }
    // The filter sets exchange batching beside renderer exchange; the shared
    // predicate keeps rejecting every bit of an unknown ABI revision.
    if (!transport::valid_capability_reply(caps, returned)) {
        fprintf(stderr, "Capability reply: %lu bytes, size %lu, version %lu, flags 0x%08lx.\n",
                static_cast<unsigned long>(returned), static_cast<unsigned long>(caps.size),
                static_cast<unsigned long>(caps.version), static_cast<unsigned long>(caps.flags));
        return report_error("Validate capability reply", ERROR_REVISION_MISMATCH);
    }

    printf("Lightning NT5 host capability exchange passed.\n"
           "Venus wire format %lu, Vulkan XML %lu, serialization %lu, protocol %lu.\n"
           "Transport capabilities 0x%08lx. Rendering is not established by this query.\n",
           static_cast<unsigned long>(caps.caps.wire_format_version),
           static_cast<unsigned long>(caps.caps.vk_xml_version),
           static_cast<unsigned long>(caps.caps.command_serialization_spec_version),
           static_cast<unsigned long>(caps.caps.venus_protocol_spec_version),
           static_cast<unsigned long>(caps.transport_capabilities));
    if ((caps.flags & transport::capability_renderer_exchange) != 0) {
        puts("Native renderer exchange is available; an application draw is not yet verified here.");
    }
    if ((caps.flags & transport::capability_exchange_batch) != 0) {
        puts("Batched renderer exchange is available.");
    }
    return 0;
}

// HKLM\HARDWARE\DEVICEMAP\VIDEO "\Device\Video0" names the device win32k binds
// the primary desktop to (lightning_control_policy.hpp video_key_owner).
struct PrimaryDisplay final {
    bool present = false;
    lightning::tools::VideoKeyForm form = lightning::tools::VideoKeyForm::unknown;
    wchar_t path[512]{};
    wchar_t service[MAX_PATH]{};
};

PrimaryDisplay read_primary_display() noexcept {
    PrimaryDisplay result{};
    HKEY opened = nullptr;
    if (RegOpenKeyExW(HKEY_LOCAL_MACHINE, L"HARDWARE\\DEVICEMAP\\VIDEO", 0, KEY_QUERY_VALUE, &opened) != ERROR_SUCCESS) {
        return result;
    }
    rx7900::RegistryHandle key(opened);
    DWORD type = 0;
    DWORD bytes = sizeof(result.path) - sizeof(wchar_t);
    if (RegQueryValueExW(key.get(), L"\\Device\\Video0", nullptr, &type,
            reinterpret_cast<BYTE*>(result.path), &bytes) != ERROR_SUCCESS ||
        type != REG_SZ || bytes % sizeof(wchar_t) != 0 || bytes >= sizeof(result.path)) {
        result.path[0] = 0;
        return result;
    }
    result.path[bytes / sizeof(wchar_t)] = 0;
    result.present = true;
    result.form = lightning::tools::video_key_owner(result.path, result.service, MAX_PATH);
    return result;
}

// Present display-class devices and their services: the only services the
// explicit opt-in may disable. Optionally prints them.
bool display_device_uses_service(const wchar_t* service, bool print) noexcept {
    rx7900::DeviceSetHandle devices(SetupDiGetClassDevsW(nullptr, nullptr, nullptr,
        DIGCF_ALLCLASSES | DIGCF_PRESENT), INVALID_HANDLE_VALUE);
    if (!devices.valid()) return false;
    bool found = false;
    SP_DEVINFO_DATA device{};
    device.cbSize = sizeof(device);
    for (DWORD index = 0; SetupDiEnumDeviceInfo(devices.get(), index, &device); ++index) {
        wchar_t class_guid[64]{};
        wchar_t owner[MAX_PATH]{};
        if (rx7900::pnp_string_property(devices.get(), device, SPDRP_CLASSGUID, class_guid) != ERROR_SUCCESS ||
            lstrcmpiW(class_guid, display_class_guid) != 0) continue;
        if (rx7900::pnp_string_property(devices.get(), device, SPDRP_SERVICE, owner) != ERROR_SUCCESS) owner[0] = 0;
        if (print) {
            wchar_t description[MAX_PATH]{};
            if (rx7900::pnp_string_property(devices.get(), device, SPDRP_DEVICEDESC, description) != ERROR_SUCCESS) {
                lstrcpynW(description, L"(no description)", MAX_PATH);
            }
            printf("Display device: %ls, service %ls.\n", description, owner[0] != 0 ? owner : L"(none)");
        }
        if (service != nullptr && owner[0] != 0 && lightning::tools::equal_insensitive(owner, service)) found = true;
    }
    return found;
}

// Read-only. Exit 0 only when LightningVideo owns \Device\Video0.
int report_primary() noexcept {
    namespace tools = lightning::tools;
    const PrimaryDisplay primary = read_primary_display();
    display_device_uses_service(nullptr, true);
    switch (tools::classify_primary(primary.present, primary.form, primary.service, display_service_name)) {
    case tools::PrimaryOwner::lightning:
        puts("Primary display \\Device\\Video0 is LightningVideo.");
        return 0;
    case tools::PrimaryOwner::other:
        printf("Primary display \\Device\\Video0 is owned by %ls, not LightningVideo: the desktop stays on it.\n"
               "No binding was changed. To make Lightning the desktop explicitly and reversibly, run\n"
               "  lightningnt5ctl display-primary-disable %ls\n"
               "and restart; undo it with lightningnt5ctl display-primary-restore and a restart.\n",
               primary.service, primary.service);
        return 1;
    case tools::PrimaryOwner::unknown:
        printf("Primary display \\Device\\Video0 is %ls; its owning service cannot be proven.\n", primary.path);
        return 1;
    case tools::PrimaryOwner::absent:
        break;
    }
    puts("No primary display \\Device\\Video0 is registered.");
    return 1;
}

bool query_service_start(const wchar_t* name, DWORD& type, DWORD& start) noexcept {
    rx7900::ServiceHandle manager(OpenSCManagerW(nullptr, nullptr, SC_MANAGER_CONNECT));
    if (!manager.valid()) return false;
    rx7900::ServiceHandle service(OpenServiceW(manager.get(), name, SERVICE_QUERY_CONFIG));
    if (!service.valid()) return false;
    alignas(QUERY_SERVICE_CONFIGW) BYTE storage[4096]{};
    auto* configuration = reinterpret_cast<QUERY_SERVICE_CONFIGW*>(storage);
    DWORD required = 0;
    if (!QueryServiceConfigW(service.get(), configuration, sizeof(storage), &required)) return false;
    type = configuration->dwServiceType;
    start = configuration->dwStartType;
    return true;
}

DWORD change_service_start(const wchar_t* name, DWORD start) noexcept {
    rx7900::ServiceHandle manager(OpenSCManagerW(nullptr, nullptr, SC_MANAGER_CONNECT));
    if (!manager.valid()) return GetLastError();
    rx7900::ServiceHandle service(OpenServiceW(manager.get(), name, SERVICE_QUERY_CONFIG | SERVICE_CHANGE_CONFIG));
    if (!service.valid()) return GetLastError();
    if (!ChangeServiceConfigW(service.get(), SERVICE_NO_CHANGE, start, SERVICE_NO_CHANGE,
            nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr)) return GetLastError();
    DWORD type = 0;
    DWORD observed = 0;
    if (!query_service_start(name, type, observed)) return GetLastError();
    return observed == start ? ERROR_SUCCESS : ERROR_REVISION_MISMATCH;
}

// Explicit operator opt-in only (never called by install): disable one named
// competing display miniport so LightningVideo can own \Device\Video0 after a
// restart. The original start type is recorded before the change.
int disable_competing_display(const wchar_t* name) noexcept {
    namespace tools = lightning::tools;
    DWORD type = 0;
    DWORD start = 0;
    if (!query_service_start(name, type, start)) return report_error("Query the named display service");
    const bool display_service = display_device_uses_service(name, false);
    if (start == tools::service_disabled && display_service) {
        printf("%ls is already disabled; no change was made.\n", name);
        return 0;
    }
    if (!tools::competing_display_disable_admissible(name, display_service_name, display_service, type, start)) {
        fprintf(stderr, "%ls is not the service of a present display device other than LightningVideo, "
                        "or is not an enabled kernel driver (type 0x%lx, start %lu).\n",
                name, static_cast<unsigned long>(type), static_cast<unsigned long>(start));
        return report_error("Refuse to disable the named display service", ERROR_ACCESS_DENIED);
    }
    HKEY opened = nullptr;
    LONG status = RegCreateKeyExW(HKEY_LOCAL_MACHINE, primary_rollback_key, 0, nullptr, 0,
        KEY_SET_VALUE | KEY_QUERY_VALUE, nullptr, &opened, nullptr);
    if (status != ERROR_SUCCESS) return report_error("Create the display-primary rollback record", static_cast<DWORD>(status));
    rx7900::RegistryHandle key(opened);
    status = RegSetValueExW(key.get(), name, 0, REG_DWORD, reinterpret_cast<const BYTE*>(&start), sizeof(start));
    if (status != ERROR_SUCCESS) return report_error("Record the original start type", static_cast<DWORD>(status));
    const DWORD changed = change_service_start(name, tools::service_disabled);
    if (changed != ERROR_SUCCESS) {
        // Nothing to roll back when the change did not apply.
        DWORD observed_type = 0;
        DWORD observed = start;
        if (query_service_start(name, observed_type, observed) && observed == start) RegDeleteValueW(key.get(), name);
        return report_error("Disable the named display service", changed);
    }
    printf("Disabled %ls (start %lu -> %lu); the original start type is recorded under HKLM\\%ls.\n"
           "Restart ReactOS for LightningVideo to own \\Device\\Video0; none was performed.\n"
           "Undo: lightningnt5ctl display-primary-restore, then restart.\n",
           name, static_cast<unsigned long>(start), static_cast<unsigned long>(tools::service_disabled),
           primary_rollback_key);
    return reboot_required_exit;
}

int restore_competing_displays() noexcept {
    namespace tools = lightning::tools;
    HKEY opened = nullptr;
    LONG status = RegOpenKeyExW(HKEY_LOCAL_MACHINE, primary_rollback_key, 0, KEY_SET_VALUE | KEY_QUERY_VALUE, &opened);
    if (status == ERROR_FILE_NOT_FOUND) {
        puts("No display-primary rollback is recorded; no change was made.");
        return 0;
    }
    if (status != ERROR_SUCCESS) return report_error("Open the display-primary rollback record", static_cast<DWORD>(status));
    rx7900::RegistryHandle key(opened);
    wchar_t names[16][MAX_PATH]{};
    DWORD count = 0;
    for (DWORD index = 0; count < 16; ++index) {
        DWORD characters = MAX_PATH;
        DWORD type = 0;
        status = RegEnumValueW(key.get(), index, names[count], &characters, nullptr, &type, nullptr, nullptr);
        if (status == ERROR_NO_MORE_ITEMS) break;
        if (status != ERROR_SUCCESS) return report_error("Enumerate the display-primary rollback record", static_cast<DWORD>(status));
        if (type == REG_DWORD && names[count][0] != 0) ++count;
    }
    unsigned restored = 0;
    unsigned refused = 0;
    for (DWORD index = 0; index < count; ++index) {
        const wchar_t* name = names[index];
        DWORD recorded = 0;
        DWORD bytes = sizeof(recorded);
        DWORD type = 0;
        DWORD service_type = 0;
        DWORD current = 0;
        if (RegQueryValueExW(key.get(), name, nullptr, &type, reinterpret_cast<BYTE*>(&recorded), &bytes) != ERROR_SUCCESS ||
            type != REG_DWORD || bytes != sizeof(recorded) || !query_service_start(name, service_type, current) ||
            !tools::display_rollback_admissible(recorded, current)) {
            printf("Left %ls unchanged: recorded start %lu, current start %lu.\n", name,
                   static_cast<unsigned long>(recorded), static_cast<unsigned long>(current));
            ++refused;
            continue;
        }
        const DWORD changed = change_service_start(name, recorded);
        if (changed != ERROR_SUCCESS) {
            fprintf(stderr, "Restore %ls failed: Win32 error %lu\n", name, static_cast<unsigned long>(changed));
            ++refused;
            continue;
        }
        RegDeleteValueW(key.get(), name);
        printf("Restored %ls start type %lu.\n", name, static_cast<unsigned long>(recorded));
        ++restored;
    }
    if (restored != 0) puts("Restart ReactOS to apply the restored display binding; none was performed.");
    return refused != 0 ? 1 : (restored != 0 ? reboot_required_exit : 0);
}

int release_desktop_dc(HDC dc) noexcept {
    return ReleaseDC(nullptr, dc);
}

int query_display() noexcept {
    rx7900::ScopedHandle<HDC, release_desktop_dc> screen(GetDC(nullptr));

    if (!screen.valid()) {
        return report_error("Open desktop display context");
    }

    transport::CpuScanoutInformation information{};
    const int returned = ExtEscape(screen.get(), transport::query_display_escape,
        0, nullptr, sizeof(information), reinterpret_cast<char*>(&information));

    if (returned != sizeof(information) || information.size != sizeof(information) ||
        information.version != transport::cpu_scanout_abi_version ||
        information.magic != LIGHTNING_DISPLAY_MAGIC) {
        report_primary();
        return report_error("Desktop is not using the Lightning display driver", ERROR_NOT_SUPPORTED);
    }

    printf("QEMU Lightning virtual display: %lux%lu, stride %lu, generation %lu.\n"
           "Capabilities 0x%08lx, refresh period %lu ns, enabled %lu.\n"
           "Accepted %lu:%lu, published %lu:%lu.\n"
           "Console publication is distinct from physical monitor vblank.\n",
        static_cast<unsigned long>(information.width), static_cast<unsigned long>(information.height),
        static_cast<unsigned long>(information.stride), static_cast<unsigned long>(information.mode_generation),
        static_cast<unsigned long>(information.capabilities), static_cast<unsigned long>(information.refresh_ns),
        static_cast<unsigned long>(information.enabled), static_cast<unsigned long>(information.accepted_epoch),
        static_cast<unsigned long>(information.accepted_sequence), static_cast<unsigned long>(information.completed_epoch),
        static_cast<unsigned long>(information.completed_sequence));

    transport::DisplayModeHealth health{};
    const int health_bytes = ExtEscape(screen.get(), transport::query_mode_health_escape,
        0, nullptr, sizeof(health), reinterpret_cast<char*>(&health));
    if (health_bytes != sizeof(health) || health.size != sizeof(health) ||
        health.version != transport::mode_health_version || health.magic != transport::mode_health_magic) {
        puts("Lightning mode health: not reported by this display driver.");
        return 0;
    }
    printf("Lightning mode health: enables=%lu failed=%lu consecutive=%lu last-step=%lu flags=0x%08lx "
           "last-error=%lu.\n",
        static_cast<unsigned long>(health.enables), static_cast<unsigned long>(health.failed_attempts),
        static_cast<unsigned long>(health.consecutive_failures), static_cast<unsigned long>(health.last_failed_step),
        static_cast<unsigned long>(health.flags), static_cast<unsigned long>(health.last_error));
    if ((health.flags & transport::mode_health_gpu_demoted) != 0) {
        puts("WARNING: a mode enable demoted the GPU canvas to CPU publication.");
    }
    if ((health.flags & transport::mode_health_cursor_dropped) != 0) {
        puts("WARNING: a mode enable could not restore the hardware cursor.");
    }
    if ((health.flags & transport::mode_health_parked_now) != 0) {
        return report_error("Lightning desktop is parked after failed mode enables; nothing is published",
            ERROR_DEVICE_REINITIALIZATION_NEEDED);
    }
    if ((health.flags & transport::mode_health_parked) != 0) {
        puts("WARNING: an earlier desktop PDEV was parked after failed mode enables.");
    }
    return 0;
}

DWORD verify_opengl_registration() noexcept;

bool frontend_path(lightning::tools::FrontendSource source, wchar_t (&path)[MAX_PATH]) noexcept {
    namespace tools = lightning::tools;
    path[0] = 0;
    bool resolved = false;
    if (source == tools::FrontendSource::registered) {
        // Exactly the absolute Dll opengl32 loads; verified, never guessed.
        const UINT length = GetSystemDirectoryW(path, MAX_PATH);
        resolved = verify_opengl_registration() == ERROR_SUCCESS && length != 0 && length < MAX_PATH &&
            SUCCEEDED(StringCchCatW(path, MAX_PATH, L"\\")) &&
            SUCCEEDED(StringCchCatW(path, MAX_PATH, lightning::reactos::opengl::icd_filename));
    } else {
        resolved = rx7900::module_sibling_path(source == tools::FrontendSource::package ?
            tools::frontend_package_relative : lightning::reactos::opengl::icd_filename, path);
    }
    if (!resolved) return false;
    const DWORD attributes = GetFileAttributesW(path);
    return attributes != INVALID_FILE_ATTRIBUTES && (attributes & FILE_ATTRIBUTE_DIRECTORY) == 0;
}

int check_frontend() noexcept {
    wchar_t path[MAX_PATH]{};
    const char* source_name = nullptr;

    for (const auto source : lightning::tools::frontend_order) {
        if (frontend_path(source, path)) {
            source_name = lightning::tools::frontend_source_name(source);
            break;
        }
    }
    if (source_name == nullptr) {
        return report_error("Resolve the registered or packaged WGL frontend", ERROR_FILE_NOT_FOUND);
    }
    printf("WGL frontend candidate: %ls (%s).\n", path, source_name);

    // Mesa owns process-wide hooks/TLS. Keep the exact loaded module alive until
    // normal process exit; explicit FreeLibrary can deadlock its worker cleanup.
    const HMODULE module = LoadLibraryExW(path, nullptr, LOAD_WITH_ALTERED_SEARCH_PATH);

    if (module == nullptr) {
        return report_error("Load ReactOS WGL frontend");
    }

    HMODULE pinned = nullptr;

    if (!GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_PIN, path, &pinned) || pinned != module) {
        return report_error("Retain exact WGL module for process lifetime");
    }

    constexpr const char* required_exports[]{
        "DrvCreateContext", "DrvSetContext", "DrvSwapBuffers", "DrvValidateVersion"
    };

    for (const auto* name : required_exports) {
        if (GetProcAddress(module, name) == nullptr) {
            return report_error("Check packaged WGL entry points", ERROR_PROC_NOT_FOUND);
        }
    }

    puts("ReactOS WGL frontend loaded; required ICD entry points are present.\n"
         "This checks loading only; no renderer or accelerated frame was created.");
    return 0;
}

bool packaged_file_exists(const wchar_t* filename) noexcept {
    wchar_t path[MAX_PATH]{};

    if (!rx7900::module_sibling_path(filename, path)) {
        SetLastError(ERROR_BAD_PATHNAME);
        return false;
    }

    const DWORD attributes = GetFileAttributesW(path);
    return attributes != INVALID_FILE_ATTRIBUTES && (attributes & FILE_ATTRIBUTE_DIRECTORY) == 0;
}

// Read only the bounded DOS and COFF headers. The package is untrusted until
// this check succeeds, and a wrong-architecture kernel image must never reach
// the PnP update call. The scoped handle owns the file through every exit.
DWORD verify_packaged_machine(const wchar_t* filename) noexcept {
    wchar_t path[MAX_PATH]{};

    if (!rx7900::module_sibling_path(filename, path)) return ERROR_BAD_PATHNAME;
    rx7900::DeviceHandle file(CreateFileW(path, GENERIC_READ, FILE_SHARE_READ,
        nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr), INVALID_HANDLE_VALUE);
    if (!file.valid()) return GetLastError();
    LARGE_INTEGER size{};
    if (!GetFileSizeEx(file.get(), &size)) return GetLastError();
    constexpr DWORD dos_size = 64;
    constexpr DWORD coff_size = 6;
    BYTE dos[dos_size]{};
    DWORD bytes = 0;
    if (size.QuadPart < dos_size || !ReadFile(file.get(), dos, dos_size, &bytes, nullptr)) {
        return ERROR_BAD_EXE_FORMAT;
    }
    if (bytes != dos_size || dos[0] != 'M' || dos[1] != 'Z') return ERROR_BAD_EXE_FORMAT;
    const DWORD offset = static_cast<DWORD>(dos[0x3c]) |
        (static_cast<DWORD>(dos[0x3d]) << 8) |
        (static_cast<DWORD>(dos[0x3e]) << 16) |
        (static_cast<DWORD>(dos[0x3f]) << 24);
    if (offset < dos_size || offset > 1024 * 1024 ||
        static_cast<LONGLONG>(offset) > size.QuadPart - coff_size) return ERROR_BAD_EXE_FORMAT;
    LARGE_INTEGER position{};
    position.QuadPart = offset;
    if (!SetFilePointerEx(file.get(), position, nullptr, FILE_BEGIN)) return GetLastError();
    BYTE coff[coff_size]{};
    if (!ReadFile(file.get(), coff, coff_size, &bytes, nullptr) || bytes != coff_size ||
        coff[0] != 'P' || coff[1] != 'E' || coff[2] != 0 || coff[3] != 0) {
        return ERROR_BAD_EXE_FORMAT;
    }
    const WORD machine = static_cast<WORD>(coff[4] | (coff[5] << 8));
#if defined(_M_AMD64) || defined(__x86_64__)
    return machine == IMAGE_FILE_MACHINE_AMD64 ? ERROR_SUCCESS : ERROR_BAD_EXE_FORMAT;
#elif defined(_M_IX86) || defined(__i386__)
    return machine == IMAGE_FILE_MACHINE_I386 ? ERROR_SUCCESS : ERROR_BAD_EXE_FORMAT;
#else
    return ERROR_NOT_SUPPORTED;
#endif
}

// QueryServiceConfig returns pointers into this caller-owned bounded buffer.
// Validate the complete pointer/string span before any native string function.
bool copy_service_path(const BYTE* storage,size_t bytes,const wchar_t* source,
    wchar_t (&output)[MAX_PATH]) noexcept {
    const uintptr_t begin=reinterpret_cast<uintptr_t>(storage);
    const uintptr_t address=reinterpret_cast<uintptr_t>(source);
    if (source==nullptr || bytes>UINTPTR_MAX-begin || address<begin || address>=begin+bytes ||
        (address-begin)%sizeof(wchar_t)!=0) return false;
    const size_t characters=(begin+bytes-address)/sizeof(wchar_t);
    for (size_t index=0;index<characters && index<MAX_PATH;++index) {
        output[index]=source[index];
        if (source[index]==0) return index!=0;
    }
    return false;
}

DWORD verify_transport_registration() noexcept {
    rx7900::ServiceHandle manager(OpenSCManagerW(nullptr,nullptr,SC_MANAGER_CONNECT));
    if (!manager.valid()) return GetLastError();
    rx7900::ServiceHandle service(OpenServiceW(manager.get(),transport::service_name,SERVICE_QUERY_CONFIG));
    if (!service.valid()) return GetLastError();
    alignas(QUERY_SERVICE_CONFIGW) BYTE storage[4096]{};
    auto* configuration=reinterpret_cast<QUERY_SERVICE_CONFIGW*>(storage);
    DWORD required=0;
    if (!QueryServiceConfigW(service.get(),configuration,sizeof(storage),&required)) return GetLastError();
    if (configuration->dwServiceType!=SERVICE_KERNEL_DRIVER || configuration->dwStartType!=SERVICE_DEMAND_START)
        return ERROR_INVALID_DATA;
    wchar_t registered[MAX_PATH]{};
    if (!copy_service_path(storage,sizeof(storage),configuration->lpBinaryPathName,registered)) return ERROR_INVALID_DATA;
    wchar_t installed[MAX_PATH]{};
    const UINT length=GetSystemDirectoryW(installed,MAX_PATH);
    if (length==0 || length>=MAX_PATH || FAILED(StringCchCatW(installed,MAX_PATH,L"\\drivers\\")) ||
        FAILED(StringCchCatW(installed,MAX_PATH,transport::driver_filename))) return ERROR_BAD_PATHNAME;
    wchar_t relative[MAX_PATH]{};
    wchar_t system_root[MAX_PATH]{};
    if (FAILED(StringCchPrintfW(relative,MAX_PATH,L"system32\\drivers\\%s",transport::driver_filename)) ||
        FAILED(StringCchPrintfW(system_root,MAX_PATH,L"\\SystemRoot\\%s",relative))) return ERROR_BAD_PATHNAME;
    bool matched=lstrcmpiW(registered,installed)==0 || lstrcmpiW(registered,relative)==0 ||
        lstrcmpiW(registered,system_root)==0;
    wchar_t expanded[MAX_PATH]{};
    const DWORD expanded_length=ExpandEnvironmentStringsW(registered,expanded,MAX_PATH);
    if (expanded_length!=0 && expanded_length<=MAX_PATH) matched=matched || lstrcmpiW(expanded,installed)==0;
    if (!matched) return ERROR_BAD_PATHNAME;
    const DWORD attributes=GetFileAttributesW(installed);
    if (attributes==INVALID_FILE_ATTRIBUTES) return GetLastError();
    return (attributes&FILE_ATTRIBUTE_DIRECTORY)==0 ? ERROR_SUCCESS : ERROR_FILE_NOT_FOUND;
}

DWORD inspect_display_metadata(HDEVINFO devices,SP_DEVINFO_DATA& device) noexcept {
    wchar_t class_guid[MAX_PATH]{};
    DWORD result=rx7900::pnp_string_property(devices,device,SPDRP_CLASSGUID,class_guid);
    if (result!=ERROR_SUCCESS) return result;
    if (lstrcmpiW(class_guid,display_class_guid)!=0) return ERROR_INVALID_DATA;
    wchar_t driver[MAX_PATH]{};
    result=rx7900::pnp_string_property(devices,device,SPDRP_DRIVER,driver);
    if (result!=ERROR_SUCCESS) return result;
    constexpr size_t guid_characters=sizeof(display_class_guid)/sizeof(wchar_t)-1;
    size_t length=0;
    if (FAILED(StringCchLengthW(driver,MAX_PATH,&length)) || length<=guid_characters ||
        driver[guid_characters]!=L'\\') return ERROR_INVALID_DATA;
    wchar_t driver_class[sizeof(display_class_guid)/sizeof(wchar_t)]{};
    for (size_t index=0;index<guid_characters;++index) driver_class[index]=driver[index];
    if (lstrcmpiW(driver_class,display_class_guid)!=0) {
        puts("Display class migration metadata is inconsistent: the driver key remains in the old class.\n"
             "This ReactOS platform gap was not rewritten; the working LightningVideo binding is retained.");
    }
    return ERROR_SUCCESS;
}

DWORD verify_opengl_registration() noexcept {
#if defined(_M_IX86) || defined(__i386__) || defined(_M_AMD64) || defined(__x86_64__)
    namespace gl=lightning::reactos::opengl;
    HKEY opened=nullptr;
    const LONG status=RegOpenKeyExW(HKEY_LOCAL_MACHINE,gl::registry_key,0,KEY_QUERY_VALUE,&opened);
    if (status!=ERROR_SUCCESS) return static_cast<DWORD>(status);
    rx7900::RegistryHandle key(opened);
    wchar_t registered[MAX_PATH]{};
    DWORD type=0,bytes=sizeof(registered);
    LONG query=RegQueryValueExW(key.get(),L"Dll",nullptr,&type,
        reinterpret_cast<BYTE*>(registered),&bytes);
    if (query!=ERROR_SUCCESS) return static_cast<DWORD>(query);
    if (type!=REG_SZ || bytes<sizeof(wchar_t) || bytes>sizeof(registered) ||
        bytes%sizeof(wchar_t)!=0 || registered[bytes/sizeof(wchar_t)-1]!=0) return ERROR_INVALID_DATA;
    for (size_t index=0;index+1<bytes/sizeof(wchar_t);++index)
        if (registered[index]==0) return ERROR_INVALID_DATA;
    wchar_t system[MAX_PATH]{};
    const UINT count=GetSystemDirectoryW(system,MAX_PATH);
    if (count==0 || count>=MAX_PATH) return ERROR_BAD_PATHNAME;
    wchar_t expected[MAX_PATH]{};
    if (FAILED(StringCchPrintfW(expected,MAX_PATH,L"%s\\%s",system,gl::icd_filename)) ||
        lstrcmpiW(registered,expected)!=0) return ERROR_BAD_PATHNAME;
    const wchar_t* names[]{L"Version",L"DriverVersion",L"Flags"};
    const DWORD values[]{gl::interface_version,gl::driver_version,0};
    for (size_t index=0;index<sizeof(values)/sizeof(values[0]);++index) {
        DWORD value=0;
        bytes=sizeof(value);
        query=RegQueryValueExW(key.get(),names[index],nullptr,&type,reinterpret_cast<BYTE*>(&value),&bytes);
        if (query!=ERROR_SUCCESS) return static_cast<DWORD>(query);
        if (type!=REG_DWORD || bytes!=sizeof(value) || value!=values[index]) return ERROR_REVISION_MISMATCH;
    }
    DWORD interval=gl::uncapped_swap_interval;
    bytes=sizeof(interval);
    query=RegQueryValueExW(key.get(),gl::default_swap_interval_value,nullptr,&type,
        reinterpret_cast<BYTE*>(&interval),&bytes);
    if (query!=ERROR_SUCCESS) return static_cast<DWORD>(query);
    if (type!=REG_DWORD || bytes!=sizeof(interval) || interval>4) return ERROR_INVALID_DATA;
    for (const auto* filename:gl::runtime_files) {
        if (FAILED(StringCchPrintfW(expected,MAX_PATH,L"%s\\%s",system,filename))) return ERROR_BAD_PATHNAME;
        const DWORD attributes=GetFileAttributesW(expected);
        if (attributes==INVALID_FILE_ATTRIBUTES) return GetLastError();
        if ((attributes&FILE_ATTRIBUTE_DIRECTORY)!=0) return ERROR_FILE_INVALID;
    }
#endif
    return ERROR_SUCCESS;
}

// A successful install must not leave the failed-install marks of an earlier
// default-driver attempt: ReactOS SetupDiInstallDevice never clears them, so
// umpnpmgr would report the working device as CM_PROB_FAILED_INSTALL (28).
DWORD clear_failed_install(HDEVINFO devices,SP_DEVINFO_DATA& device) noexcept {
    constexpr DWORD stale=0x00000040UL|0x00000020UL; // CONFIGFLAG_FAILEDINSTALL|CONFIGFLAG_REINSTALL
    DWORD flags=0,type=0;
    if (!SetupDiGetDeviceRegistryPropertyW(devices,&device,SPDRP_CONFIGFLAGS,&type,
            reinterpret_cast<BYTE*>(&flags),sizeof(flags),nullptr)) {
        return GetLastError()==ERROR_INVALID_DATA ? ERROR_SUCCESS : GetLastError();
    }
    if (type!=REG_DWORD) return ERROR_INVALID_DATA;
    if ((flags&stale)==0) return ERROR_SUCCESS;
    DWORD cleared=flags&~stale;
    if (!SetupDiSetDeviceRegistryPropertyW(devices,&device,SPDRP_CONFIGFLAGS,
            reinterpret_cast<const BYTE*>(&cleared),sizeof(cleared))) return GetLastError();
    DWORD observed=0;
    if (!SetupDiGetDeviceRegistryPropertyW(devices,&device,SPDRP_CONFIGFLAGS,&type,
            reinterpret_cast<BYTE*>(&observed),sizeof(observed),nullptr)) return GetLastError();
    return type==REG_DWORD && observed==cleared ? ERROR_SUCCESS : ERROR_REVISION_MISMATCH;
}

int verify_display_install(bool filter_present_before,bool reboot) noexcept {
    rx7900::DeviceSetHandle devices(SetupDiGetClassDevsW(nullptr,nullptr,nullptr,
        DIGCF_ALLCLASSES|DIGCF_PRESENT),INVALID_HANDLE_VALUE);
    if (!devices.valid()) return report_error("Reopen installed Lightning device");
    SP_DEVINFO_DATA device{};
    DWORD result=rx7900::unique_pnp_device(devices.get(),transport::hardware_id,device);
    if (result!=ERROR_SUCCESS) return report_error("Verify exact installed Lightning PCI device",result);
    if (!rx7900::pnp_service_matches(devices.get(),device,display_service_name,false))
        return report_error("Verify retained LightningVideo functional driver",ERROR_INVALID_DATA);
    result=clear_failed_install(devices.get(),device);
    if (result!=ERROR_SUCCESS) return report_error("Clear stale failed-install device flags",result);
    result=verify_transport_registration();
    if (result!=ERROR_SUCCESS) return report_error("Verify LightningNT5 kernel service and installed binary",result);
    result=inspect_display_metadata(devices.get(),device);
    if (result!=ERROR_SUCCESS) return report_error("Verify installed display class metadata",result);
    bool filter_changed=false;
    result=rx7900::repair_upper_filter(devices.get(),device,transport::service_name,filter_changed);
    if (result!=ERROR_SUCCESS) return report_error("Repair and read back exact Lightning upper-filter list",result);
    result=verify_opengl_registration();
    if (result!=ERROR_SUCCESS) return report_error("Verify installed adapter OpenGL ICD metadata and runtime files",result);
    if (reboot || !filter_present_before || filter_changed) {
        puts("LightningVideo and its registered LightningNT5 upper filter are installed.\n"
             "Restart ReactOS to rebuild the device stack; none was performed.");
        // Report only: another miniport owning \Device\Video0 keeps the desktop.
        report_primary();
        return reboot_required_exit;
    }
    puts("Verified LightningVideo binding and LightningNT5 upper-filter registration.");
    return query_capabilities();
}

int install_driver(bool display = false) noexcept {
    const wchar_t* package_files[]{
        display ? display_inf_filename : inf_filename,
        transport::driver_filename,
        L"lightningvideo.sys",
        L"lightningdisp.dll"
    };
    const size_t package_count = display ? 4 : 2;

    for (size_t index = 0; index < package_count; ++index) {
        if (!packaged_file_exists(package_files[index])) {
            return report_error("Check complete driver package", ERROR_FILE_NOT_FOUND);
        }
    }
    for (size_t index = 1; index < package_count; ++index) {
        const DWORD machine = verify_packaged_machine(package_files[index]);
        if (machine != ERROR_SUCCESS) {
            return report_error("Check native driver image architecture", machine);
        }
    }
#if defined(_M_IX86) || defined(__i386__) || defined(_M_AMD64) || defined(__x86_64__)
    if (display) {
        for (const auto* filename:lightning::reactos::opengl::runtime_files) {
            wchar_t relative[MAX_PATH]{};
            if (FAILED(StringCchPrintfW(relative,MAX_PATH,L"LightningGL\\%s",filename)) ||
                !packaged_file_exists(relative)) return report_error("Check complete installed OpenGL runtime package");
            if (lstrcmpW(filename, lightning::reactos::opengl::manifest_filename) != 0) {
                const DWORD machine = verify_packaged_machine(relative);
                if (machine != ERROR_SUCCESS) return report_error("Check OpenGL runtime image architecture", machine);
            }
        }
    }
#endif

    const DWORD scan = rx7900::rescan_pnp_root();

    if (scan != ERROR_SUCCESS) {
        return report_error("Rescan PnP tree", scan);
    }

    rx7900::DeviceSetHandle devices(SetupDiGetClassDevsW(nullptr, nullptr, nullptr,
        DIGCF_ALLCLASSES | DIGCF_PRESENT), INVALID_HANDLE_VALUE);

    if (!devices.valid()) {
        return report_error("Enumerate present devices");
    }

    SP_DEVINFO_DATA device{};
    const DWORD selected = rx7900::unique_pnp_device(devices.get(),
        transport::hardware_id, device);

    if (selected != ERROR_SUCCESS) {
        return report_error("Select unique Lightning PCI device", selected);
    }
    const bool own_transport = rx7900::pnp_service_matches(devices.get(), device,
        transport::service_name, true);
    const bool own_display = display && rx7900::pnp_service_matches(devices.get(), device,
        display_service_name, false);

    if (!own_transport && !own_display) {
        return report_error("Refuse replacement of another device service", ERROR_ACCESS_DENIED);
    }

    rx7900::PreparedFilterProperty previous_filters{};
    if (display) {
        const DWORD filters=rx7900::prepare_upper_filter(devices.get(),device,
            transport::service_name,previous_filters);
        if (filters!=ERROR_SUCCESS) return report_error("Validate existing Lightning upper-filter list",filters);
    }

    wchar_t inf[MAX_PATH]{};

    if (!rx7900::module_sibling_path(display ? display_inf_filename : inf_filename, inf)) {
        return report_error("Resolve package INF", ERROR_BAD_PATHNAME);
    }

    BOOL reboot = FALSE;

    if (!UpdateDriverForPlugAndPlayDevicesW(nullptr, transport::hardware_id,
                                            inf, 0, &reboot)) {
        return report_error("Install Lightning NT5 PnP driver");
    }

    if (display) return verify_display_install(previous_filters.contains_filter,reboot!=FALSE);

    puts("Installed Lightning NT5 transport for PCI 1234:11F1.");

    if (reboot) {
        puts("Windows requested a restart; none was performed.");
        return reboot_required_exit;
    }

    return query_capabilities();
}

int remove_device(bool display = false) noexcept {
    rx7900::DeviceSetHandle devices(SetupDiGetClassDevsW(nullptr, nullptr, nullptr,
        DIGCF_ALLCLASSES | DIGCF_PRESENT), INVALID_HANDLE_VALUE);

    if (!devices.valid()) {
        return report_error("Enumerate present devices");
    }

    SP_DEVINFO_DATA device{};
    const DWORD selected = rx7900::unique_pnp_device(devices.get(),
        transport::hardware_id, device);

    if (selected != ERROR_SUCCESS) {
        return report_error("Select unique Lightning PCI device", selected);
    }
    if (!rx7900::pnp_service_matches(devices.get(), device,
                                   display ? display_service_name : transport::service_name, false)) {
        return report_error("Refuse removal of another device service", ERROR_ACCESS_DENIED);
    }

    SP_REMOVEDEVICE_PARAMS parameters{};
    parameters.ClassInstallHeader.cbSize = sizeof(SP_CLASSINSTALL_HEADER);
    parameters.ClassInstallHeader.InstallFunction = DIF_REMOVE;
    parameters.Scope = DI_REMOVEDEVICE_GLOBAL;

    if (SetupDiSetClassInstallParamsW(devices.get(), &device,
            &parameters.ClassInstallHeader, sizeof(parameters)) &&
        SetupDiCallClassInstaller(DIF_REMOVE, devices.get(), &device)) {
        puts("Removed the Lightning NT5 device binding. Package files remain available for reinstall.");
        return 0;
    }

    const DWORD removal = GetLastError();

    // ReactOS 0.4.16 refuses DIF_REMOVE with ERROR_CALL_NOT_IMPLEMENTED before
    // any change (no DIF_REMOVE class-install parameter handler; the default
    // SetupDiRemoveDevice is a stub). Undo the binding in place instead, filters
    // included as DIF_REMOVE would; the devnode is re-installed at the next boot.
    if (removal != ERROR_CALL_NOT_IMPLEMENTED) {
        return report_error("Remove Lightning NT5 PnP device", removal);
    }

    rx7900::UnbindStep step = rx7900::UnbindStep::none;
    const DWORD unbound = rx7900::unbind_pnp_device(devices.get(), device,
        display ? display_service_name : transport::service_name, true, step);

    if (unbound != ERROR_SUCCESS) {
        fprintf(stderr, "In-place unbinding failed while %s.\n", rx7900::unbind_step_description(step));
        return report_error("Remove Lightning NT5 PnP device binding in place", unbound);
    }
    puts("SetupAPI device removal is not implemented here; the Lightning NT5 binding, driver key and device "
         "filters were deleted in place. Package files remain available for reinstall.");
    puts("Restart ReactOS to complete the removal; none was performed.");
    return reboot_required_exit;
}

} // namespace

int wmain(int argc, wchar_t** argv) {
    if (argc == 3 && lstrcmpW(argv[1], L"display-primary-disable") == 0 && nt5_system()) {
        return disable_competing_display(argv[2]);
    }
    if (argc != 2) {
        fwprintf(stderr, L"Usage: lightningnt5ctl install|display-install|query|display-query|rescan|remove|display-remove|"
                         L"frontend-check|display-primary|display-primary-restore\n"
                         L"       lightningnt5ctl display-primary-disable <competing display service>\n");
        return 2;
    }
    if (!nt5_system()) {
        return report_error("This package requires an NT5 guest", ERROR_OLD_WIN_VERSION);
    }
    if (lstrcmpW(argv[1], L"query") == 0) {
        return query_capabilities();
    }
    if (lstrcmpW(argv[1], L"display-query") == 0) {
        return query_display();
    }
    if (lstrcmpW(argv[1], L"frontend-check") == 0) {
        return check_frontend();
    }
    if (lstrcmpW(argv[1], L"display-primary") == 0) {
        return report_primary();
    }
    if (lstrcmpW(argv[1], L"display-primary-restore") == 0) {
        return restore_competing_displays();
    }
    if (lstrcmpW(argv[1], L"install") == 0) {
        return install_driver();
    }
    if (lstrcmpW(argv[1], L"display-install") == 0) {
        return install_driver(true);
    }
    if (lstrcmpW(argv[1], L"remove") == 0) {
        return remove_device();
    }
    if (lstrcmpW(argv[1], L"display-remove") == 0) {
        return remove_device(true);
    }
    if (lstrcmpW(argv[1], L"rescan") == 0) {
        const DWORD error = rx7900::rescan_pnp_root();
        return error == ERROR_SUCCESS ? 0 : report_error("Rescan PnP tree", error);
    }

    return report_error("Unknown command", ERROR_INVALID_PARAMETER);
}

/*
Module: rx7900.tools.radeon_installer
Purpose: Run radeonctl install/status/upgrade/rollback/remove/recover over a system seam.
Created: 2026-10-24
Architecture: Platform-neutral C++17 engine. radeon_win32_system binds the seam to
the registry, files, SCM, SetupAPI and GDI; the host and Wine tests bind it to an
in-memory fake with fault injection. Policy decisions live in radeon_install_policy.
*/
#pragma once

#include "pnp_unbind.hpp"
#include "radeon_install_policy.hpp"

namespace rx7900::tools::radeon {

// Read-back of the one service this installer owns (service_name). The seam
// never exposes another service, so no command can modify a foreign driver.
struct ServiceInfo final {
    bool exists = false;
    uint32_t type = 0;
    uint32_t start = 0;
    uint32_t error_control = 0;
    bool running = false;
    wchar_t image_path[path_characters]{};
};

// The desktop's display driver as GDI and VideoPort report it, read-only.
struct ActiveDisplay final {
    wchar_t opengl_driver[64]{};                         // OPENGL_GETINFO name, empty if none
    wchar_t primary_service[service_name_characters]{}; // DEVICEMAP\VIDEO \Device\Video0 owner
};

// Step of SystemOps::unbind_device that failed (pnp_unbind.hpp).
using rx7900::UnbindStep;

// Every operation returns a Win32 error code (error::success on success). Keys
// are relative to HKEY_LOCAL_MACHINE; paths are absolute. Buffers are caller
// owned and valid only for the call. Mutating file operations are write-through.
struct SystemOps final {
    void* context;

    uint32_t (*read_value)(void* context, const wchar_t* key, const wchar_t* name, uint32_t& type,
                           void* data, uint32_t capacity, uint32_t& bytes);
    uint32_t (*write_value)(void* context, const wchar_t* key, const wchar_t* name, uint32_t type,
                            const void* data, uint32_t bytes);  // creates the key
    uint32_t (*delete_value)(void* context, const wchar_t* key, const wchar_t* name);
    uint32_t (*key_exists)(void* context, const wchar_t* key, bool& exists);
    uint32_t (*delete_key)(void* context, const wchar_t* key);  // key without subkeys

    uint32_t (*file_size)(void* context, const wchar_t* path, bool& exists, uint64_t& bytes);
    uint32_t (*read_file)(void* context, const wchar_t* path, uint64_t offset, void* buffer,
                          uint32_t capacity, uint32_t& received);
    uint32_t (*copy_file)(void* context, const wchar_t* from, const wchar_t* to);  // replaces
    uint32_t (*move_file)(void* context, const wchar_t* from, const wchar_t* to);  // replaces
    // MOVEFILE_DELAY_UNTIL_REBOOT; to == nullptr deletes at restart.
    uint32_t (*replace_on_restart)(void* context, const wchar_t* from, const wchar_t* to);
    uint32_t (*delete_file)(void* context, const wchar_t* path);
    uint32_t (*create_directory)(void* context, const wchar_t* path);  // success if present

    uint32_t (*query_service)(void* context, ServiceInfo& output);
    uint32_t (*create_service)(void* context);   // kernel, demand start, our image
    uint32_t (*set_demand_start)(void* context); // never any other start type
    uint32_t (*delete_service)(void* context);

    uint32_t (*enumerate_devices)(void* context, DeviceInventory& output);
    // Installs the package INF on exactly one device instance. The device is
    // not restarted; the new driver loads at the next system start.
    uint32_t (*install_driver)(void* context, const wchar_t* instance_id, const wchar_t* hardware_id,
                               const wchar_t* inf_path, bool& restart);
    // DIF_REMOVE (devnode, driver key and filters are deleted). ReactOS 0.4.16
    // returns error::call_not_implemented: SetupDiSetClassInstallParams has no
    // DIF_REMOVE parameter handler and SetupDiRemoveDevice is a stub.
    uint32_t (*remove_device)(void* context, const wchar_t* instance_id);
    // In-place unbinding where DIF_REMOVE is unimplemented: deletes the
    // device's driver key and its Driver, Class and ClassGUID values, clears
    // the failed-install/reinstall marks and deletes the Service value last.
    // The devnode keeps its instance ID and filters; at the next boot ReactOS
    // installs its default driver. Repeating it after a partial run is safe.
    // On failure, failed names the step; it is none on success.
    uint32_t (*unbind_device)(void* context, const wchar_t* instance_id, UnbindStep& failed);
    uint32_t (*write_filters)(void* context, const wchar_t* instance_id, bool upper,
                              const wchar_t* list, size_t characters);
    // REG_SZ value of the device's software (driver) key, e.g. InfPath.
    uint32_t (*driver_value)(void* context, const wchar_t* instance_id, const wchar_t* name,
                             wchar_t* output, size_t capacity);
    // SetupUninstallOEMInf of a bare oemNN.inf name; force = SUOI_FORCEDELETE.
    uint32_t (*uninstall_oem_inf)(void* context, const wchar_t* inf_name, bool force);

    uint32_t (*active_display)(void* context, ActiveDisplay& output);
    // Identity of the current boot (NT boot time); changes at every restart.
    uint32_t (*boot_id)(void* context, uint64_t& output);

    void (*print)(void* context, const char* line);

    const wchar_t* system_directory;   // %SystemRoot%\System32
    const wchar_t* inf_directory;      // %SystemRoot%\inf (SetupAPI's oemNN.inf copies)
    const wchar_t* package_directory;  // directory holding radeonctl.exe and the package
    uint16_t machine;                  // PE machine the package must match
};

// Executes one parsed command and returns its exit_code value. Mutating commands
// first complete or roll back an interrupted journal. No command starts or
// stops a driver, enables boot start, resets hardware or binds implicitly.
int run_command(const SystemOps& ops, const CommandLine& command) noexcept;

void print_usage(const SystemOps& ops) noexcept;

} // namespace rx7900::tools::radeon

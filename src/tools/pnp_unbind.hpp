/*
Module: rx7900.tools.pnp_unbind
Purpose: Undo a device's function-driver binding in place where SetupAPI cannot
remove the device (ReactOS 0.4.16 DIF_REMOVE fails with ERROR_CALL_NOT_IMPLEMENTED).
Created: 2026-10-25
Architecture: Pure C++17 algorithm over a caller-supplied registry accessor:
pnp_support.hpp binds it to the Win32 registry for radeonctl and lightningnt5ctl,
the radeon installer FakeSystem to its in-memory model. ReactOS umpnpmgr
installs a device at boot unless its Enum key has a Class value or
CONFIGFLAG_FAILEDINSTALL, so removing the binding values makes the next boot
install the best driver on the same devnode.
*/
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace rx7900 {

// Step of unbind_in_place that failed; none on success.
enum class UnbindStep : uint8_t {
    none,
    open_device,     // instance ID, its Enum key or its Service value
    driver_key,      // Control\Class\{GUID}\NNNN and its Settings subkey
    binding_values,  // Enum Driver, Class, ClassGUID (and filters when asked)
    config_flags,    // CONFIGFLAG_FAILEDINSTALL / CONFIGFLAG_REINSTALL
    service_value,   // Enum Service (last)
};

// What the step was doing, for error messages.
inline const char* unbind_step_description(UnbindStep step) noexcept {
    switch (step) {
    case UnbindStep::none: return "no step";
    case UnbindStep::open_device: return "opening the device's Enum key";
    case UnbindStep::driver_key: return "deleting the driver (Control\\Class) key";
    case UnbindStep::binding_values: return "deleting the Driver/Class/ClassGUID values";
    case UnbindStep::config_flags: return "clearing the failed-install ConfigFlags";
    case UnbindStep::service_value: return "deleting the Service value";
    }
    return "unknown step";
}

namespace pnp_unbind {

constexpr uint32_t success = 0;          // ERROR_SUCCESS
constexpr uint32_t file_not_found = 2;   // ERROR_FILE_NOT_FOUND
constexpr uint32_t access_denied = 5;    // ERROR_ACCESS_DENIED
constexpr uint32_t invalid_data = 13;    // ERROR_INVALID_DATA
constexpr uint32_t bad_pathname = 161;   // ERROR_BAD_PATHNAME
constexpr uint32_t configflag_reinstall = 0x00000020;       // CONFIGFLAG_REINSTALL (regstr.h)
constexpr uint32_t configflag_failed_install = 0x00000040;  // CONFIGFLAG_FAILEDINSTALL (regstr.h)
constexpr size_t key_characters = 400;  // Enum prefix + MAX_DEVICE_ID_LEN + NUL fits
inline constexpr wchar_t enum_root[] = L"SYSTEM\\CurrentControlSet\\Enum\\";
inline constexpr wchar_t class_root[] = L"SYSTEM\\CurrentControlSet\\Control\\Class\\";

template<size_t Capacity>
bool append(wchar_t (&output)[Capacity], const wchar_t* text) noexcept {
    size_t length = 0;
    while (length < Capacity && output[length] != 0) {
        ++length;
    }
    for (size_t index = 0; text[index] != 0; ++index) {
        if (length + 1 >= Capacity) {
            output[0] = 0;
            return false;
        }
        output[length++] = text[index];
    }
    output[length] = 0;
    return true;
}

inline bool equal_ascii_insensitive(const wchar_t* left, const wchar_t* right) noexcept {
    const auto fold = [](wchar_t value) noexcept {
        return value >= L'A' && value <= L'Z' ? static_cast<wchar_t>(value + (L'a' - L'A')) : value;
    };
    for (size_t index = 0;; ++index) {
        if (fold(left[index]) != fold(right[index])) {
            return false;
        }
        if (left[index] == 0) {
            return true;
        }
    }
}

// Enum key of a device instance ID as SetupAPI reports it: a relative,
// backslash-separated path without empty or dot components.
inline bool enum_key_path(const wchar_t* instance_id, wchar_t (&path)[key_characters]) noexcept {
    path[0] = 0;
    if (instance_id == nullptr || instance_id[0] == 0 || instance_id[0] == L'\\') {
        return false;
    }
    for (size_t index = 0; instance_id[index] != 0; ++index) {
        const wchar_t value = instance_id[index];
        const bool component_start = index == 0 || instance_id[index - 1] == L'\\';
        if (value < 0x20 || value == L'/' || (value == L'\\' && (instance_id[index + 1] == 0 || component_start)) ||
            (value == L'.' && component_start)) {
            return false;
        }
    }
    return append(path, enum_root) && append(path, instance_id);
}

// The Driver value names the software key as {class GUID}\NNNN below
// Control\Class. Only that exact shape is accepted before a key is deleted.
inline bool driver_key_path(const wchar_t* driver, wchar_t (&path)[key_characters]) noexcept {
    constexpr size_t guid_characters = 38;  // {xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx}
    constexpr size_t index_characters = 4;
    size_t length = 0;

    path[0] = 0;
    while (length <= guid_characters + 1 + index_characters && driver[length] != 0) {
        ++length;
    }
    if (length != guid_characters + 1 + index_characters || driver[0] != L'{' ||
        driver[guid_characters - 1] != L'}' || driver[guid_characters] != L'\\') {
        return false;
    }
    for (size_t index = 1; index + 1 < guid_characters; ++index) {
        const wchar_t value = driver[index];
        const bool dash = index == 9 || index == 14 || index == 19 || index == 24;
        const bool hex = (value >= L'0' && value <= L'9') || (value >= L'a' && value <= L'f') ||
                         (value >= L'A' && value <= L'F');
        if (dash ? value != L'-' : !hex) {
            return false;
        }
    }
    for (size_t index = guid_characters + 1; index < length; ++index) {
        if (driver[index] < L'0' || driver[index] > L'9') {
            return false;
        }
    }
    return append(path, class_root) && append(path, driver);
}

inline uint32_t absent_ok(uint32_t result) noexcept {
    return result == file_not_found ? success : result;
}

} // namespace pnp_unbind

// Registry accessor (keys relative to HKEY_LOCAL_MACHINE, Win32 error codes,
// file_not_found for an absent key or value):
//   uint32_t read_string(const wchar_t* key, const wchar_t* name, wchar_t* output, size_t capacity)
//       REG_SZ only, NUL-terminated within capacity, invalid_data otherwise;
//   uint32_t read_dword(const wchar_t* key, const wchar_t* name, uint32_t& value)   REG_DWORD only;
//   uint32_t write_dword(const wchar_t* key, const wchar_t* name, uint32_t value);
//   uint32_t delete_value(const wchar_t* key, const wchar_t* name);
//   uint32_t delete_key(const wchar_t* key)                key without subkeys;
//   uint32_t flush(const wchar_t* key)                     makes the key's changes durable.
//
// Unbinds the device whose Enum Service value is expected_service: deletes its
// driver key (and the Settings subkey the display class installer adds), its
// Driver, Class and ClassGUID values (UpperFilters/LowerFilters too when
// delete_filters, as DIF_REMOVE would), clears CONFIGFLAG_FAILEDINSTALL and
// CONFIGFLAG_REINSTALL and deletes the Service value last, so an interrupted
// run still reads as bound and is repeated. A device without a Service value
// is already unbound (success); another service is refused (access_denied).
// The running devnode is untouched; the change applies at the next boot.
template<typename Registry>
uint32_t unbind_in_place(Registry& registry, const wchar_t* instance_id, const wchar_t* expected_service,
                         bool delete_filters, UnbindStep& failed) noexcept {
    using namespace pnp_unbind;
    constexpr const wchar_t* binding_values[]{L"Driver", L"Class", L"ClassGUID"};
    constexpr const wchar_t* filter_values[]{L"UpperFilters", L"LowerFilters"};
    wchar_t device_key[key_characters]{};
    wchar_t service[key_characters]{};

    failed = UnbindStep::open_device;
    if (!enum_key_path(instance_id, device_key)) {
        return bad_pathname;
    }

    uint32_t result = registry.read_string(device_key, L"Service", service, key_characters);
    if (result == file_not_found) {
        failed = UnbindStep::none;
        return success;  // already unbound
    }
    if (result != success) {
        return result;
    }
    if (!equal_ascii_insensitive(service, expected_service)) {
        return access_denied;
    }

    failed = UnbindStep::driver_key;
    wchar_t driver[key_characters]{};
    result = registry.read_string(device_key, L"Driver", driver, key_characters);
    if (result == success) {
        wchar_t software[key_characters]{};
        wchar_t settings[key_characters]{};
        if (!driver_key_path(driver, software) || !driver_key_path(driver, settings) ||
            !append(settings, L"\\Settings")) {
            return invalid_data;
        }
        if ((result = absent_ok(registry.delete_key(settings))) != success ||
            (result = absent_ok(registry.delete_key(software))) != success) {
            return result;
        }
    } else if (result != file_not_found) {
        return result;
    }

    failed = UnbindStep::binding_values;
    for (const wchar_t* name : binding_values) {
        if ((result = absent_ok(registry.delete_value(device_key, name))) != success) {
            return result;
        }
    }
    for (size_t index = 0; delete_filters && index < 2; ++index) {
        if ((result = absent_ok(registry.delete_value(device_key, filter_values[index]))) != success) {
            return result;
        }
    }

    failed = UnbindStep::config_flags;
    constexpr uint32_t stale = configflag_failed_install | configflag_reinstall;
    uint32_t flags = 0;
    result = registry.read_dword(device_key, L"ConfigFlags", flags);
    if (result == success && (flags & stale) != 0) {
        result = registry.write_dword(device_key, L"ConfigFlags", flags & ~stale);
    }
    if ((result = absent_ok(result)) != success || (result = registry.flush(device_key)) != success) {
        return result;
    }

    failed = UnbindStep::service_value;
    if ((result = absent_ok(registry.delete_value(device_key, L"Service"))) != success ||
        (result = registry.flush(device_key)) != success) {
        return result;
    }
    failed = UnbindStep::none;
    return success;
}

} // namespace rx7900

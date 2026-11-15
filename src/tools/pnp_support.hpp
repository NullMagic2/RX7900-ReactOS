/*
Module: rx7900.tools.pnp_support
Purpose: Share bounded PnP selection and native device-set ownership.
Created: 2026-10-21
Architecture: Installer tools select an exact hardware ID before any mutation.
*/
#pragma once

#include <windows.h>
#include <setupapi.h>
#include <cfgmgr32.h>
#include "windows_support.hpp"
#include "multisz.hpp"
#include "pnp_unbind.hpp"

namespace rx7900 {

using DeviceSetHandle = ScopedHandle<HDEVINFO, SetupDiDestroyDeviceInfoList>;
constexpr DWORD maximum_pnp_devices = 4096;
constexpr DWORD maximum_hardware_id_characters = 4096;
constexpr DWORD maximum_filter_characters = 4096;

struct PreparedFilterProperty final {
    wchar_t merged[maximum_filter_characters]{};
    DWORD bytes=0;
    bool contains_filter=false;
};

// The selected device set stays alive throughout property preparation/write.
// All registry bytes are bounded and typed before the shared list parser runs.
// Missing lists are admitted; malformed existing lists are never overwritten.
template<size_t FilterLength>
inline DWORD prepare_upper_filter(HDEVINFO devices,SP_DEVINFO_DATA& device,
    const wchar_t (&filter)[FilterLength],PreparedFilterProperty& output) noexcept {
    output={};
    wchar_t original[maximum_filter_characters]{};
    DWORD type=0,bytes=0;
    if (!SetupDiGetDeviceRegistryPropertyW(devices,&device,SPDRP_UPPERFILTERS,&type,
        reinterpret_cast<PBYTE>(original),sizeof(original),&bytes)) {
        const DWORD error=GetLastError();
        if (error!=ERROR_FILE_NOT_FOUND && error!=ERROR_INVALID_DATA) return error;
        if (error==ERROR_INVALID_DATA) {
            // Windows uses INVALID_DATA for absence. Confirm absence rather
            // than overwrite an existing malformed or differently typed list.
            RegistryHandle key(SetupDiOpenDevRegKey(devices,&device,DICS_FLAG_GLOBAL,0,DIREG_DEV,KEY_QUERY_VALUE),
                reinterpret_cast<HKEY>(INVALID_HANDLE_VALUE));
            if (!key.valid()) return GetLastError();
            const LONG query=RegQueryValueExW(key.get(),L"UpperFilters",nullptr,nullptr,nullptr,nullptr);
            if (query!=ERROR_FILE_NOT_FOUND) return query==ERROR_SUCCESS ? ERROR_INVALID_DATA : static_cast<DWORD>(query);
        }
        bytes=0;
    } else if (type!=REG_MULTI_SZ || bytes<2*sizeof(wchar_t) || bytes>sizeof(original) || bytes%sizeof(wchar_t)!=0) {
        return ERROR_INVALID_DATA;
    }
    size_t characters=0;
    const auto status=append_filter_name(original,bytes/sizeof(wchar_t),filter,output.merged,characters);
    if (status==MultiStringMerge::malformed) return ERROR_INVALID_DATA;
    if (status==MultiStringMerge::too_large) return ERROR_INSUFFICIENT_BUFFER;
    output.bytes=static_cast<DWORD>(characters*sizeof(wchar_t));
    output.contains_filter=status==MultiStringMerge::unchanged;
    return ERROR_SUCCESS;
}

// Preserve every observed unknown filter and verify the complete resulting
// property. This never changes the functional Service, class or driver key.
template<size_t FilterLength>
inline DWORD repair_upper_filter(HDEVINFO devices,SP_DEVINFO_DATA& device,
    const wchar_t (&filter)[FilterLength],bool& changed) noexcept {
    changed=false;
    PreparedFilterProperty prepared{};
    DWORD result=prepare_upper_filter(devices,device,filter,prepared);
    if (result!=ERROR_SUCCESS || prepared.contains_filter) return result;
    if (!SetupDiSetDeviceRegistryPropertyW(devices,&device,SPDRP_UPPERFILTERS,
        reinterpret_cast<const BYTE*>(prepared.merged),prepared.bytes)) return GetLastError();
    changed=true;
    PreparedFilterProperty observed{};
    result=prepare_upper_filter(devices,device,filter,observed);
    if (result!=ERROR_SUCCESS) return result;
    return observed.contains_filter && observed.bytes==prepared.bytes &&
        memcmp(observed.merged,prepared.merged,prepared.bytes)==0 ? ERROR_SUCCESS : ERROR_READ_FAULT;
}

template<size_t Capacity>
inline DWORD pnp_string_property(HDEVINFO devices,SP_DEVINFO_DATA& device,DWORD property,
    wchar_t (&output)[Capacity]) noexcept {
    DWORD type=0,bytes=0;
    if (!SetupDiGetDeviceRegistryPropertyW(devices,&device,property,&type,
        reinterpret_cast<PBYTE>(output),sizeof(output),&bytes)) return GetLastError();
    if (type!=REG_SZ || bytes<sizeof(wchar_t) || bytes>sizeof(output) ||
        bytes%sizeof(wchar_t)!=0 || output[bytes/sizeof(wchar_t)-1]!=0) return ERROR_INVALID_DATA;
    for (size_t index=0;index+1<bytes/sizeof(wchar_t);++index) {
        if (output[index]==0) return ERROR_INVALID_DATA;
    }
    return ERROR_SUCCESS;
}

// Only parses the returned native REG_MULTI_SZ span. Every comparison sees a
// validated terminating NUL; malformed/truncated properties never match.
inline bool pnp_id_property_matches(HDEVINFO devices, SP_DEVINFO_DATA& device,
                                   DWORD property, const wchar_t* expected) noexcept {
    wchar_t ids[maximum_hardware_id_characters]{};
    DWORD type = 0;
    DWORD bytes = 0;

    if (!SetupDiGetDeviceRegistryPropertyW(devices, &device, property,
            &type, reinterpret_cast<PBYTE>(ids), sizeof(ids), &bytes) ||
        type != REG_MULTI_SZ || bytes < 2 * sizeof(wchar_t) ||
        bytes > sizeof(ids) || bytes % sizeof(wchar_t) != 0) {
        return false;
    }

    const size_t count = bytes / sizeof(wchar_t);
    bool matched = false;
    size_t position = 0;

    while (position < count) {
        const size_t first = position;

        while (position < count && ids[position] != L'\0') {
            ++position;
        }

        if (position == count) {
            return false;
        }
        if (first == position) {
            return matched;
        }

        matched = matched || lstrcmpiW(ids + first, expected) == 0;
        ++position;
    }

    return false; // The required final empty string was absent.
}

inline bool hardware_id_matches(HDEVINFO devices, SP_DEVINFO_DATA& device,
                                const wchar_t* expected) noexcept {
    // PCI vendor/device-only IDs belong to CompatibleIDs on NT. Use the same
    // exact-match search as UpdateDriverForPlugAndPlayDevices, without prefixes.
    return pnp_id_property_matches(devices, device, SPDRP_HARDWAREID, expected) ||
        pnp_id_property_matches(devices, device, SPDRP_COMPATIBLEIDS, expected);
}

// A unique present device is required. The returned data borrows devices;
// callers keep its scoped set alive throughout installation/removal.
inline DWORD unique_pnp_device(HDEVINFO devices, const wchar_t* hardware_id,
                               SP_DEVINFO_DATA& selected) noexcept {
    bool found = false;

    for (DWORD index = 0; index < maximum_pnp_devices; ++index) {
        SP_DEVINFO_DATA current{};
        current.cbSize = sizeof(current);

        if (!SetupDiEnumDeviceInfo(devices, index, &current)) {
            const DWORD error = GetLastError();
            return error == ERROR_NO_MORE_ITEMS && found ? ERROR_SUCCESS : error;
        }
        if (!hardware_id_matches(devices, current, hardware_id)) {
            continue;
        }
        if (found) {
            return ERROR_DUP_NAME;
        }

        selected = current;
        found = true;
    }

    return ERROR_BUFFER_OVERFLOW;
}

// Empty unbound devices are admitted only for install. A different service is
// never replaced. Storage stays bounded and its reported type/length are checked.
inline bool pnp_service_matches(HDEVINFO devices, SP_DEVINFO_DATA& device,
                                const wchar_t* expected, bool allow_unbound) noexcept {
    wchar_t service[MAX_PATH]{};
    DWORD type = 0;
    DWORD bytes = 0;

    if (!SetupDiGetDeviceRegistryPropertyW(devices, &device, SPDRP_SERVICE, &type,
            reinterpret_cast<PBYTE>(service), sizeof(service), &bytes)) {
        const DWORD error = GetLastError();
        // ReactOS forwards RegQueryValueEx's missing Service value as error 2;
        // Windows reports ERROR_INVALID_DATA for the same unbound property.
        return allow_unbound &&
            (error == ERROR_INVALID_DATA || error == ERROR_FILE_NOT_FOUND);
    }
    if (type != REG_SZ || bytes < sizeof(wchar_t) || bytes > sizeof(service) ||
        bytes % sizeof(wchar_t) != 0 || service[bytes / sizeof(wchar_t) - 1] != L'\0') {
        return false;
    }

    return (allow_unbound && service[0] == L'\0') || lstrcmpiW(service, expected) == 0;
}

// unbind_in_place registry accessor over HKEY_LOCAL_MACHINE. Each call opens
// and closes its own key; reads are bounded and type checked.
struct Win32PnpRegistry final {
    uint32_t read_string(const wchar_t* key, const wchar_t* name, wchar_t* output, size_t capacity) noexcept {
        HKEY raw = nullptr;
        LONG status = RegOpenKeyExW(HKEY_LOCAL_MACHINE, key, 0, KEY_QUERY_VALUE, &raw);
        output[0] = L'\0';
        if (status != ERROR_SUCCESS) {
            return static_cast<uint32_t>(status);
        }
        RegistryHandle opened(raw);
        DWORD type = 0;
        DWORD bytes = static_cast<DWORD>((capacity - 1) * sizeof(wchar_t));
        status = RegQueryValueExW(opened.get(), name, nullptr, &type, reinterpret_cast<BYTE*>(output), &bytes);
        if (status != ERROR_SUCCESS) {
            output[0] = L'\0';
            return status == ERROR_MORE_DATA ? ERROR_INVALID_DATA : static_cast<uint32_t>(status);
        }
        if (type != REG_SZ || bytes % sizeof(wchar_t) != 0) {
            output[0] = L'\0';
            return ERROR_INVALID_DATA;
        }
        output[bytes / sizeof(wchar_t)] = L'\0';
        return ERROR_SUCCESS;
    }
    uint32_t read_dword(const wchar_t* key, const wchar_t* name, uint32_t& value) noexcept {
        HKEY raw = nullptr;
        LONG status = RegOpenKeyExW(HKEY_LOCAL_MACHINE, key, 0, KEY_QUERY_VALUE, &raw);
        value = 0;
        if (status != ERROR_SUCCESS) {
            return static_cast<uint32_t>(status);
        }
        RegistryHandle opened(raw);
        DWORD type = 0;
        DWORD data = 0;
        DWORD bytes = sizeof(data);
        status = RegQueryValueExW(opened.get(), name, nullptr, &type, reinterpret_cast<BYTE*>(&data), &bytes);
        if (status != ERROR_SUCCESS) {
            return static_cast<uint32_t>(status);
        }
        if (type != REG_DWORD || bytes != sizeof(data)) {
            return ERROR_INVALID_DATA;
        }
        value = data;
        return ERROR_SUCCESS;
    }
    uint32_t write_dword(const wchar_t* key, const wchar_t* name, uint32_t value) noexcept {
        HKEY raw = nullptr;
        const LONG status = RegOpenKeyExW(HKEY_LOCAL_MACHINE, key, 0, KEY_SET_VALUE, &raw);
        if (status != ERROR_SUCCESS) {
            return static_cast<uint32_t>(status);
        }
        RegistryHandle opened(raw);
        const DWORD data = value;
        return static_cast<uint32_t>(RegSetValueExW(opened.get(), name, 0, REG_DWORD,
                                                    reinterpret_cast<const BYTE*>(&data), sizeof(data)));
    }
    uint32_t delete_value(const wchar_t* key, const wchar_t* name) noexcept {
        HKEY raw = nullptr;
        const LONG status = RegOpenKeyExW(HKEY_LOCAL_MACHINE, key, 0, KEY_SET_VALUE, &raw);
        if (status != ERROR_SUCCESS) {
            return static_cast<uint32_t>(status);
        }
        RegistryHandle opened(raw);
        return static_cast<uint32_t>(RegDeleteValueW(opened.get(), name));
    }
    uint32_t delete_key(const wchar_t* key) noexcept {
        return static_cast<uint32_t>(RegDeleteKeyW(HKEY_LOCAL_MACHINE, key));
    }
    uint32_t flush(const wchar_t* key) noexcept {
        HKEY raw = nullptr;
        const LONG status = RegOpenKeyExW(HKEY_LOCAL_MACHINE, key, 0, KEY_SET_VALUE, &raw);
        if (status != ERROR_SUCCESS) {
            return static_cast<uint32_t>(status);
        }
        RegistryHandle opened(raw);
        return static_cast<uint32_t>(RegFlushKey(opened.get()));
    }
};

// In-place unbinding of one opened device (pnp_unbind.hpp) where DIF_REMOVE
// fails with ERROR_CALL_NOT_IMPLEMENTED (ReactOS 0.4.16). The device's Enum key
// is written directly, as ReactOS SetupAPI itself does in-process
// (SETUPDI_OpenDevKey): its CM_Set_DevNode_Registry_Property cannot delete a
// value (the RPC Buffer is a [ref] pointer, so NULL fails with CR_FAILURE) and
// SetupDiSetDeviceRegistryProperty writes an empty value instead of deleting.
// On Windows the Enum key is SYSTEM-only; this path is not taken there.
inline DWORD unbind_pnp_device(HDEVINFO devices, SP_DEVINFO_DATA& device, const wchar_t* expected_service,
                               bool delete_filters, UnbindStep& failed) noexcept {
    wchar_t instance[MAX_DEVICE_ID_LEN + 1]{};
    DWORD required = 0;

    failed = UnbindStep::open_device;
    if (!SetupDiGetDeviceInstanceIdW(devices, &device, instance, MAX_DEVICE_ID_LEN + 1, &required)) {
        return GetLastError();
    }
    Win32PnpRegistry registry;
    return unbind_in_place(registry, instance, expected_service, delete_filters, failed);
}

inline DWORD rescan_pnp_root() noexcept {
    DEVINST root = 0;
    CONFIGRET result = CM_Locate_DevNodeW(&root, nullptr, CM_LOCATE_DEVNODE_NORMAL);

    if (result == CR_SUCCESS) {
        result = CM_Reenumerate_DevNode(root, CM_REENUMERATE_SYNCHRONOUS);
    }

    return result == CR_SUCCESS ? ERROR_SUCCESS : ERROR_NOT_READY;
}

} // namespace rx7900

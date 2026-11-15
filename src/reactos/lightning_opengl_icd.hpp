/*
Module: lightning.reactos.opengl_icd
Purpose: Share the pinned NT5 OpenGL driver-selection reply and package identity.
Created: 2026-10-22
Architecture: Matches ReactOS opengl32/icdload.c; contains no caller pointers.
*/
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>

namespace lightning::reactos::opengl {
inline constexpr uint32_t interface_version = 2;
inline constexpr uint32_t driver_version = 0x00020000;
inline constexpr uint32_t query_driver_name = 0;
inline constexpr char16_t driver_name[] = u"QemuLightning";
inline constexpr wchar_t registry_name[] = L"QemuLightning";
inline constexpr wchar_t registry_key[] =
    L"SOFTWARE\\Microsoft\\Windows NT\\CurrentVersion\\OpenGLDrivers\\QemuLightning";
inline constexpr wchar_t default_swap_interval_value[] = L"DefaultSwapInterval";
inline constexpr uint32_t uncapped_swap_interval = 0;
inline constexpr wchar_t icd_filename[] = L"QemuLightningOpenGL.dll";
inline constexpr wchar_t manifest_filename[] = L"QemuLightningVulkan.json";
inline constexpr wchar_t loader_filename[] = L"QemuLightningVulkanLoader.dll";
inline constexpr const wchar_t* runtime_files[]{icd_filename, L"QemuLightningSync.dll",
    L"QemuLightningVulkan.dll", loader_filename, manifest_filename};

struct DriverInfo final {
    uint32_t version;
    uint32_t driver_version;
    uint16_t name[261];
};
static_assert(sizeof(DriverInfo) == 532 && offsetof(DriverInfo, name) == 8,
    "Pinned ReactOS Drv_Opengl_Info ABI");

inline bool write_driver_info(void* output, size_t capacity) noexcept {
    if (output == nullptr || capacity < sizeof(DriverInfo)) return false;
    static_assert(sizeof(driver_name) / sizeof(driver_name[0]) <= 261, "Terminated ICD name fits");
    // Write the engine-owned output directly: a 532-byte automatic temporary
    // would consume unnecessary nested NT5 kernel stack space.
    memset(output, 0, sizeof(DriverInfo));
    auto* bytes = static_cast<uint8_t*>(output);
    memcpy(bytes + offsetof(DriverInfo, version), &interface_version, sizeof(interface_version));
    memcpy(bytes + offsetof(DriverInfo, driver_version), &driver_version, sizeof(driver_version));
    memcpy(bytes + offsetof(DriverInfo, name), driver_name, sizeof(driver_name));
    return true;
}
}

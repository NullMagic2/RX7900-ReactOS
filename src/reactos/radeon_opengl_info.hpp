/*
Module: rx7900.reactos.radeon_opengl_info
Purpose: Name the physical RadeonGL ICD to ReactOS opengl32 through the display driver.
Created: 2026-10-23
Architecture: Header-only, SDK-free reply of the standard OPENGL_GETINFO escape
(ReactOS a97c8a2 dll/opengl/opengl32/icdload.c Drv_Opengl_Info). Shared by
radeondisp DrvEscape, the radeonglicd DrvValidateVersion and host tests. The
versions must equal the OpenGLDrivers\RadeonGL Version/DriverVersion values.
*/
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace rx7900::reactos::radeon_opengl {

constexpr uint32_t query_escape_support = 8;       // QUERYESCSUPPORT (wingdi.h)
constexpr uint32_t opengl_getinfo = 4353;          // OPENGL_GETINFO (winddi.h)
constexpr uint32_t getinfo_driver_name = 0;        // OPENGL_GETINFO_DRVNAME
constexpr uint32_t interface_version = 1;
constexpr uint32_t driver_version = 1;
constexpr uint32_t max_path = 260;

// icdload.c Drv_Opengl_Info: ULONG, ULONG, WCHAR[MAX_PATH + 1].
struct OpenglInfo final {
    uint32_t version;
    uint32_t driver_version;
    uint16_t driver_name[max_path + 1];
};
static_assert(sizeof(OpenglInfo) == 8 + 2 * (max_path + 1) + 2, "Drv_Opengl_Info layout (ULONG-aligned)");
static_assert(offsetof(OpenglInfo, driver_name) == 8, "Drv_Opengl_Info layout");

constexpr char driver_name[] = "RadeonGL";

// QUERYESCSUPPORT: exactly one DWORD naming OPENGL_GETINFO.
inline bool supports(const void* input, uint32_t input_bytes) noexcept {
    uint32_t code = 0;
    if (input == nullptr || input_bytes != sizeof(code)) {
        return false;
    }
    memcpy(&code, input, sizeof(code));
    return code == opengl_getinfo;
}

// OPENGL_GETINFO(OPENGL_GETINFO_DRVNAME): fills output and returns the reply
// size, or 0 when the request is not exactly that query or output is too small.
inline uint32_t encode_info(const void* input, uint32_t input_bytes, void* output, uint32_t output_bytes) noexcept {
    uint32_t selector = UINT32_MAX;
    if (input == nullptr || input_bytes != sizeof(selector) || output == nullptr ||
        output_bytes < sizeof(OpenglInfo)) {
        return 0;
    }
    memcpy(&selector, input, sizeof(selector));
    if (selector != getinfo_driver_name) {
        return 0;
    }

    // Written in place (no 532-byte stack copy inside kernel DrvEscape).
    auto* bytes = static_cast<uint8_t*>(output);
    memset(bytes, 0, sizeof(OpenglInfo));
    memcpy(bytes + offsetof(OpenglInfo, version), &interface_version, sizeof(uint32_t));
    memcpy(bytes + offsetof(OpenglInfo, driver_version), &driver_version, sizeof(uint32_t));
    for (size_t index = 0; driver_name[index] != '\0'; ++index) {
        const uint16_t unit = static_cast<uint16_t>(driver_name[index]);
        memcpy(bytes + offsetof(OpenglInfo, driver_name) + index * sizeof(uint16_t), &unit, sizeof(unit));
    }
    return static_cast<uint32_t>(sizeof(OpenglInfo));
}

} // namespace rx7900::reactos::radeon_opengl

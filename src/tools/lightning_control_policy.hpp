/*
Module: rx7900.tools.lightning_control_policy
Purpose: Portable decisions of lightningnt5ctl's primary-display and frontend checks.
Created: 2026-10-25
Architecture: No Win32 calls; lightning_control.cpp supplies registry/SCM/SetupAPI
observations and performs the only explicit, operator-requested changes.
*/
#pragma once
#include <stddef.h>
#include <stdint.h>

namespace lightning::tools {

// ReactOS videoprt IntVideoPortAddDeviceMapLink (videoprt.c) writes
// HKLM\HARDWARE\DEVICEMAP\VIDEO "\Device\VideoN" = the miniport registry path:
// by default the legacy key
//   \Registry\Machine\System\CurrentControlSet\Services\<Service>\Device<N>
// or "...\Control\Video\{VideoId}\NNNN" when Control\GraphicsDrivers\UseNewKey
// exists (that form names no service).
enum class VideoKeyForm { service, video_id, unknown };

constexpr wchar_t lower(wchar_t value) noexcept {
    return value >= L'A' && value <= L'Z' ? static_cast<wchar_t>(value - L'A' + L'a') : value;
}

// Case-insensitive search of a bounded, terminated string.
inline const wchar_t* find_insensitive(const wchar_t* text, const wchar_t* needle) noexcept {
    if (text == nullptr || needle == nullptr || needle[0] == 0) return nullptr;
    for (const wchar_t* start = text; *start != 0; ++start) {
        size_t index = 0;
        while (needle[index] != 0 && start[index] != 0 && lower(start[index]) == lower(needle[index])) ++index;
        if (needle[index] == 0) return start;
    }
    return nullptr;
}

inline bool equal_insensitive(const wchar_t* left, const wchar_t* right) noexcept {
    if (left == nullptr || right == nullptr) return false;
    size_t index = 0;
    while (left[index] != 0 && right[index] != 0 && lower(left[index]) == lower(right[index])) ++index;
    return left[index] == 0 && right[index] == 0;
}

// Extract <Service> from the legacy form, which must continue "\Device<digits>".
inline VideoKeyForm video_key_owner(const wchar_t* path, wchar_t* service, size_t capacity) noexcept {
    if (service != nullptr && capacity != 0) service[0] = 0;
    if (path == nullptr || service == nullptr || capacity < 2) return VideoKeyForm::unknown;
    if (find_insensitive(path, L"\\Control\\Video\\{") != nullptr) return VideoKeyForm::video_id;
    const wchar_t* marker = find_insensitive(path, L"\\Services\\");
    if (marker == nullptr) return VideoKeyForm::unknown;
    const wchar_t* name = marker + 10;
    size_t length = 0;
    while (name[length] != 0 && name[length] != L'\\') ++length;
    if (length == 0 || length >= capacity || name[length] != L'\\') return VideoKeyForm::unknown;
    const wchar_t* tail = name + length + 1;
    const wchar_t prefix[] = L"device";
    for (size_t index = 0; index < 6; ++index) {
        if (lower(tail[index]) != prefix[index]) return VideoKeyForm::unknown;
    }
    if (tail[6] < L'0' || tail[6] > L'9') return VideoKeyForm::unknown;
    for (size_t index = 7; tail[index] != 0; ++index) {
        if (tail[index] < L'0' || tail[index] > L'9') return VideoKeyForm::unknown;
    }
    for (size_t index = 0; index < length; ++index) service[index] = name[index];
    service[length] = 0;
    return VideoKeyForm::service;
}

enum class PrimaryOwner { lightning, other, unknown, absent };

// \Device\Video0 is the device win32k binds the primary desktop to. Only an
// exact service-form match proves it is ours; the new-key form is unknown.
inline PrimaryOwner classify_primary(bool present, VideoKeyForm form, const wchar_t* service,
                                     const wchar_t* lightning_service) noexcept {
    if (!present) return PrimaryOwner::absent;
    if (form != VideoKeyForm::service) return PrimaryOwner::unknown;
    return equal_insensitive(service, lightning_service) ? PrimaryOwner::lightning : PrimaryOwner::other;
}

constexpr uint32_t service_kernel_driver = 0x00000001;
constexpr uint32_t service_disabled = 4;

// The explicit opt-in changes exactly one competing display miniport: the
// operator named it, a present display-class device uses it, it is not ours,
// and it is a kernel driver whose start type is boot/system/auto/demand.
inline bool competing_display_disable_admissible(const wchar_t* requested, const wchar_t* lightning_service,
    bool display_device_service, uint32_t service_type, uint32_t start) noexcept {
    return requested != nullptr && requested[0] != 0 && display_device_service &&
        !equal_insensitive(requested, lightning_service) &&
        (service_type & service_kernel_driver) != 0 && start < service_disabled;
}

// Rollback restores only what the opt-in changed: the recorded original start
// type, and only while the service is still disabled (no later edit by anyone).
constexpr bool display_rollback_admissible(uint32_t recorded_start, uint32_t current_start) noexcept {
    return recorded_start < service_disabled && current_start == service_disabled;
}

// frontend-check loads what ReactOS opengl32 (icdload.c) would load: the
// registered absolute OpenGLDrivers\QemuLightning Dll once display-install
// verified it; before installation, the package's LightningGL\ copy beside the
// tool; the flat sibling layout last (older app-local packages).
enum class FrontendSource { registered, package, sibling };
constexpr FrontendSource frontend_order[] = {FrontendSource::registered, FrontendSource::package,
                                             FrontendSource::sibling};
constexpr const wchar_t* frontend_package_relative = L"LightningGL\\QemuLightningOpenGL.dll";

constexpr const char* frontend_source_name(FrontendSource source) noexcept {
    return source == FrontendSource::registered ? "registered ICD" :
        (source == FrontendSource::package ? "package LightningGL" : "tool sibling");
}

} // namespace lightning::tools

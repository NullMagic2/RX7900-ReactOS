/*
Module: rx7900.tools.windows_support
Purpose: Share scoped OS handles and bounded package-path resolution.
Created: 2026-10-21
Architecture: Win32 ownership boundaries reused by installer and guest tests.
*/
#pragma once

#include <windows.h>
#include <strsafe.h>
#include <string.h>

#include "protocol.hpp"

namespace rx7900 {

template<typename T, auto close_handle>
class ScopedHandle final {
public:
    explicit ScopedHandle(T value, T invalid = nullptr) noexcept
        : value_(value), invalid_(invalid) {}
    ~ScopedHandle() { if (value_ != invalid_) { close_handle(value_); } }
    ScopedHandle(const ScopedHandle&) = delete;
    ScopedHandle& operator=(const ScopedHandle&) = delete;
    bool valid() const noexcept { return value_ != invalid_; }
    T get() const noexcept { return value_; }

private:
    T value_;
    T invalid_;
};

using ServiceHandle = ScopedHandle<SC_HANDLE, CloseServiceHandle>;
using DeviceHandle = ScopedHandle<HANDLE, CloseHandle>;
using RegistryHandle = ScopedHandle<HKEY, RegCloseKey>;

inline bool installed_driver_path(wchar_t (&path)[MAX_PATH]) {
    const UINT length = GetSystemDirectoryW(path, MAX_PATH);

    return length != 0 && length < MAX_PATH &&
           SUCCEEDED(StringCchCatW(path, MAX_PATH, L"\\drivers\\")) &&
           SUCCEEDED(StringCchCatW(path, MAX_PATH, driver_filename));
}

// CopyFile preserves CD-ROM read-only attributes. Clear only that flag on the
// owned/matched driver file so normal and deferred uninstall can remove it.
inline bool clear_read_only_attribute(const wchar_t* path) {
    const DWORD attributes = GetFileAttributesW(path);

    if (attributes == INVALID_FILE_ATTRIBUTES) {
        return false;
    }
    if ((attributes & FILE_ATTRIBUTE_READONLY) == 0) {
        return true;
    }

    const DWORD writable = attributes & ~FILE_ATTRIBUTE_READONLY;
    return SetFileAttributesW(path, writable == 0 ? FILE_ATTRIBUTE_NORMAL : writable) != FALSE;
}

inline bool module_sibling_path(const wchar_t* filename, wchar_t (&path)[MAX_PATH]) {
    const DWORD length = GetModuleFileNameW(nullptr, path, MAX_PATH);

    if (length == 0 || length >= MAX_PATH) {
        return false;
    }

    // The module API terminated a bounded path. Walk only its returned range.
    for (DWORD cursor = length; cursor != 0; --cursor) {
        if (path[cursor - 1] == L'\\') {
            path[cursor] = L'\0';
            return SUCCEEDED(StringCchCatW(path, MAX_PATH, filename));
        }
    }

    return false;
}

// Compare two OS-owned files without loading either complete file into memory.
// Identical pre-staged driver bytes may be reused; different files are untouched.
inline bool files_equal(const wchar_t* left_path, const wchar_t* right_path) {
    DeviceHandle left(CreateFileW(left_path, GENERIC_READ, FILE_SHARE_READ, nullptr,
        OPEN_EXISTING, 0, nullptr), INVALID_HANDLE_VALUE);
    DeviceHandle right(CreateFileW(right_path, GENERIC_READ, FILE_SHARE_READ, nullptr,
        OPEN_EXISTING, 0, nullptr), INVALID_HANDLE_VALUE);

    if (!left.valid() || !right.valid()) {
        return false;
    }

    LARGE_INTEGER left_size{};
    LARGE_INTEGER right_size{};

    if (!GetFileSizeEx(left.get(), &left_size) ||
        !GetFileSizeEx(right.get(), &right_size) ||
        left_size.QuadPart <= 0 || left_size.QuadPart != right_size.QuadPart) {
        return false;
    }

    constexpr DWORD comparison_bytes = 4096;
    unsigned char left_bytes[comparison_bytes]{};
    unsigned char right_bytes[comparison_bytes]{};

    for (;;) {
        DWORD left_count = 0;
        DWORD right_count = 0;

        if (!ReadFile(left.get(), left_bytes, sizeof(left_bytes), &left_count, nullptr) ||
            !ReadFile(right.get(), right_bytes, sizeof(right_bytes), &right_count, nullptr) ||
            left_count != right_count) {
            return false;
        }
        if (left_count == 0) {
            return true;
        }
        if (memcmp(left_bytes, right_bytes, left_count) != 0) {
            return false;
        }
    }
}

} // namespace rx7900

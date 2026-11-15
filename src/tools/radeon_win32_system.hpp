/*
Module: rx7900.tools.radeon_win32_system
Purpose: Bind the Radeon installer seam to the ReactOS registry, files, SCM,
SetupAPI/newdev and GDI.
Created: 2026-10-24
Architecture: Thin Win32 translation layer under radeon_installer; no policy.
Only the rx7900video service is reachable through it, and PnP mutations act on
one exact device instance or hardware ID chosen by the engine's policy.
*/
#pragma once

#include <windows.h>

#include "radeon_installer.hpp"

namespace rx7900::tools::radeon {

// Owns the directory strings the SystemOps view borrows; keep it alive while
// the ops are used. Single-threaded.
struct Win32System final {
    wchar_t system_directory[MAX_PATH]{};
    wchar_t inf_directory[MAX_PATH]{};
    wchar_t package_directory[MAX_PATH]{};
};

// Resolves %SystemRoot%\System32, %SystemRoot%\inf and the directory of the
// running executable. Returns false when any path does not fit.
bool make_win32_system(Win32System& state, SystemOps& ops) noexcept;

} // namespace rx7900::tools::radeon

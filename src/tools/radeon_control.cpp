/*
Module: rx7900.tools.radeon_control
Purpose: radeonctl, the physical RX 7900 (Navi31) driver installer for ReactOS.
Created: 2026-10-24
Architecture: Thin CLI over radeon_installer (engine) and radeon_win32_system
(OS binding). Separate from the read-only rx7900ctl probe installer and from the
Lightning virtual display installer lightningnt5ctl.
*/
#include <windows.h>
#include <stdio.h>

#include "radeon_win32_system.hpp"

namespace {

// Mutating commands target ReactOS (NT 5.x) only; status also runs elsewhere.
bool nt5_system() noexcept {
    OSVERSIONINFOEXW version{};
    version.dwOSVersionInfoSize = sizeof(version);
    version.dwMajorVersion = 5;
    version.dwPlatformId = VER_PLATFORM_WIN32_NT;
    ULONGLONG conditions = VerSetConditionMask(0, VER_MAJORVERSION, VER_EQUAL);
    conditions = VerSetConditionMask(conditions, VER_PLATFORMID, VER_EQUAL);
    return VerifyVersionInfoW(&version, VER_MAJORVERSION | VER_PLATFORMID, conditions) != FALSE;
}

// A 32-bit radeonctl under WOW64 sees the redirected System32 and registry views
// (an i386 ICD under WOW6432Node, i386 kernel images on an AMD64 system).
bool wow64_process() noexcept {
    using IsWow64ProcessFn = BOOL(WINAPI*)(HANDLE, PBOOL);
    const HMODULE kernel32 = GetModuleHandleW(L"kernel32.dll");
    const FARPROC entry = kernel32 != nullptr ? GetProcAddress(kernel32, "IsWow64Process") : nullptr;
    const auto query = reinterpret_cast<IsWow64ProcessFn>(reinterpret_cast<void*>(entry));
    BOOL wow64 = FALSE;
    return query != nullptr && query(GetCurrentProcess(), &wow64) != FALSE && wow64 != FALSE;
}

} // namespace

int wmain(int argc, wchar_t** argv) {
    namespace radeon = rx7900::tools::radeon;
    radeon::Win32System state;
    radeon::SystemOps ops{};

    if (!radeon::make_win32_system(state, ops)) {
        fputs("Cannot resolve the system or package directory.\n", stderr);
        return radeon::exit_code::failure;
    }

    radeon::CommandLine command{};

    if (!radeon::parse_command_line(argc, argv, command)) {
        if (command.error != nullptr) {
            fprintf(stderr, "Invalid argument: %ls\n", command.error);
        }
        radeon::print_usage(ops);
        return radeon::exit_code::usage;
    }

    const bool mutating = command.command == radeon::Command::install ||
                          command.command == radeon::Command::upgrade ||
                          command.command == radeon::Command::rollback ||
                          command.command == radeon::Command::remove ||
                          command.command == radeon::Command::recover ||
                          command.command == radeon::Command::firmware_copy;

    if (mutating && !command.dry_run && !nt5_system()) {
        fputs("This installer changes only ReactOS (NT 5.x) systems; use radeonctl status here.\n", stderr);
        return radeon::exit_code::refused;
    }
    if (mutating && !command.dry_run && wow64_process()) {
        fputs("This is the i386 radeonctl on a 64-bit system; run radeonctl from the amd64 package.\n", stderr);
        return radeon::exit_code::refused;
    }
    return radeon::run_command(ops, command);
}

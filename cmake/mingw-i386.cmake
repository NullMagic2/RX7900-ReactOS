# Module: rx7900.build.mingw_i386
# Purpose: Select the GCC 13 MinGW cross-toolchain for Windows user-mode checks.
# Created: 2026-10-21
# Architecture: The kernel driver is built by ReactOS instead of this toolchain.
set(CMAKE_SYSTEM_NAME Windows)
set(CMAKE_SYSTEM_PROCESSOR i686)
set(CMAKE_CXX_COMPILER i686-w64-mingw32-g++)
set(CMAKE_RC_COMPILER i686-w64-mingw32-windres)

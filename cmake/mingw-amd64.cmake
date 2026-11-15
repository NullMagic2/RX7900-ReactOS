# Module: rx7900.build.mingw_amd64
# Purpose: Select GCC 13 MinGW for 64-bit Windows user-mode checks.
# Created: 2026-10-21
# Architecture: Native kernel artifacts remain owned by the ReactOS build.
set(CMAKE_SYSTEM_NAME Windows)
set(CMAKE_SYSTEM_PROCESSOR x86_64)
set(CMAKE_CXX_COMPILER x86_64-w64-mingw32-g++)
set(CMAKE_RC_COMPILER x86_64-w64-mingw32-windres)

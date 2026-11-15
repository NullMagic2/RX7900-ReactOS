#!/bin/sh
# Builds a driver package with cmake/driver-package.cmake.
# Usage: ./build.sh -DREACTOS_SOURCE=/path/to/reactos [-DNAME=VALUE ...]
# Defaults: the physical Radeon package, Release, i386, GCC. Later -D options
# override them (DRIVER_PACKAGE, DRIVER_BUILD_TYPE, DRIVER_ARCH, DRIVER_COMPILER,
# DRIVER_JOBS, DRIVER_PACKAGE_DIR). The result is Binaries/radeon-i386.
# Needs cmake, ninja, bison, flex and the i686-w64-mingw32 GCC 13 toolchain.
# Recreated 2026-11-15; the original wrapper was not preserved.
set -eu
root=$(cd "$(dirname "$0")" && pwd)
exec cmake -DDRIVER_ACTION=build -DDRIVER_PACKAGE=radeon -DDRIVER_BUILD_TYPE=Release "$@" \
    -P "$root/cmake/driver-package.cmake"

#!/bin/sh
# Removes the owned build intermediates (Binaries/.build/driver-package).
# Source and published packages are kept. Recreated 2026-11-15.
set -eu
root=$(cd "$(dirname "$0")" && pwd)
exec cmake -DDRIVER_ACTION=clean "$@" -P "$root/cmake/driver-package.cmake"

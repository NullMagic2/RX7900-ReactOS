# Building RX7900-ReactOS

This page is for people who want to build, test or change the drivers. For what the project is and how to install it, see the [README](README.md).

## Source layout

- `src/common`, `src/hardware`, `src/display`, `src/output`, `src/memory`, `src/acceleration`: the portable core (PCI, ATOM BIOS, firmware loading, ASIC bring-up, DCN 3.2 display, 2D engine).
- `src/reactos`: the miniport, the display driver and their ReactOS glue.
- `src/opengl`: the OpenGL driver.
- `src/tools`: `radeonctl`, the Setup wizard and diagnostic tools.
- `tests`: host tests.
- `patches`: optional ReactOS patches (see `patches/README.txt`). The released binaries were built without any of them.
- `third_party/qemu_lightning`: protocol headers for the Qemu-Lightning virtual display, which this tree also has guest drivers for (`lightningvideo`, `lightningdisp`, `lightningnt5`).

`rx7900video.sys`, `radeondisp.dll` and `radeonglicd.dll` in the release were built on 2026-10-24 and 2026-10-25. On 2026-11-15 they were rebuilt from this tree with `./build.sh` inside a pristine ReactOS `a97c8a2` checkout: each rebuilt file is byte-identical to the released one except for its PE timestamp and checksum (5, 5 and 8 bytes). `radeonctl.exe` and `Setup.exe` are from 2026-11-14 and include the installer fixes the board needed.

## Building

Host tests need a C++17 compiler and CMake:

```sh
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build -j4
ctest --test-dir build
```

All 102 tests pass. Six of them read pinned register references (AMD's GC 11.0.0 and DCN 3.2 headers from Linux v6.12 and Mesa 24.2.0's register JSON); the first configuration downloads those six files into `Binaries/upstream` and verifies their SHA-256 (`cmake/test-references.cmake`). `-DRX7900_FETCH_TEST_REFERENCES=OFF` skips the download, and those six tests then fail.

The installer and the wizard build with MinGW (i686-w64-mingw32, GCC 13):

```sh
cmake -S . -B build-win -DCMAKE_TOOLCHAIN_FILE=$PWD/cmake/mingw-i386.cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
cmake --build build-win --target radeonctl rx7900setup radeoncheck
```

`radeonctl.exe` built this way and stripped differs from the released file in six timestamp and checksum bytes. The released `Setup.exe` was built directly with `i686-w64-mingw32-g++ -std=c++17 -O2 -municode -mwindows -static -s`.

The drivers build inside a ReactOS source tree at revision `a97c8a2cdcf6a9a7f5cc543951a02ccb7a80d8ca`:

```sh
./build.sh -DREACTOS_SOURCE=/path/to/reactos -DDRIVER_JOBS=8
```

This needs CMake, ninja, bison, flex and the i686-w64-mingw32 GCC 13 toolchain. It runs `cmake/integrate.cmake`, which copies this project to `drivers/base/rx7900` in the ReactOS tree and adds it to that build, builds `rx7900video`, `radeondisp` and `radeonctl` there and `radeonglicd` with MinGW, and assembles the package in `Binaries/radeon-i386`. `./clean_build.sh` removes the intermediates. `CMakeLists.as-built-native.txt` is the build description the released drivers were first built with. `build.bat` and `clean_build.bat` are the same commands for Windows and were not run.

What was lost and what was recovered: the original wrapper scripts were not preserved and were rewritten. Of the optional ReactOS patches, four were reconstructed from the as-built ReactOS tree (`patches/README.txt`); six could not be, because that tree never had them applied: the device-bitmap bridge, the 822e864 boot cache, PnP query-remove, videoprt removal, win32k display release and ACPI hot-unplug. Their `REACTOS_APPLY_...` options stop with a message saying so. The Lightning (virtual display) package build was not run for this publication.

## How the check program was validated

`RadeonCheck.exe` (`tests/radeon_board_check.cpp`) draws in a small window through GDI, reads the pixels back and compares them with the same drawing made on a memory bitmap; moves and reshapes the mouse pointer and asks whether it looked right; and, after asking, renders a small OpenGL scene and reads it back. Its logic was checked under Wine in true colour (every comparison identical) and it runs to the end in a ReactOS guest. That guest's 16-colour VGA driver cannot read the screen back or present OpenGL, which the report's readback and swap lines show. No report from a real card has been recorded so far.

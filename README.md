# RX7900-ReactOS

Experimental display drivers for the AMD Radeon RX 7900 series (Navi31) on ReactOS i386, with a native installer.

**These drivers are tailor-made for ReactOS and are experimental. Keep a backup of the system you install them on, and a second way to reach the machine.**

## Status

On 2026-11-14 the package was installed with every option on one Radeon RX 7900 XTX (PCI `1002:744C`) in an MSI PRO B650M-P, on ReactOS i386 revision `a97c8a2`. After the restart the driver loaded and the screen resolution could be changed.

That is all that has been observed on real hardware. Not checked: correctness of accelerated 2D drawing, the hardware cursor, OpenGL, display outputs other than the one in use, other cards, other boards. In a later session with the driver installed ReactOS hung when asked to restart; whether the driver is involved is not known. The INF also lists `1002:7448`, `744A`, `744B` and `745E`; none of those was tried.

## Download and installation

Download **RX7900-ReactOS-Standalone-i386.zip** from the [latest release](https://github.com/NullMagic2/RX7900-ReactOS/releases/latest).

1. Extract the ZIP inside **ReactOS i386**.
2. Run **Setup.exe** as administrator, choose the options and let it install.
3. Restart ReactOS.

Setup is a front end for `radeonctl.exe`: it runs `radeonctl` commands and shows their output. It replaces the current display driver binding, copies the AMD firmware from the package's `Firmware` folder and never restarts the machine itself. `radeonctl status`, `rollback`, `remove` and `recover` are described in `INSTALL.txt` inside the package.

| File | Purpose |
|---|---|
| `rx7900video.sys` | VideoPort miniport |
| `radeondisp.dll` | GDI display driver |
| `radeonglicd.dll` | OpenGL installable client driver (optional) |
| `radeondisplay.inf` | Device binding |
| `radeonctl.exe` | Command-line installer with journal, rollback and recovery |
| `Setup.exe` | Graphical front end for `radeonctl.exe` |
| `Firmware/` | Unmodified `amdgpu` firmware from linux-firmware, under AMD's redistribution licence |

The card's own video ROM is not distributed. The driver reads it from the card; `FIRMWARE.txt` in the package explains the optional `rx7900.rom` file for a card whose ROM cannot be read.

## Source

- `src/common`, `src/hardware`, `src/display`, `src/output`, `src/memory`, `src/acceleration`: the portable core (PCI, ATOM BIOS, firmware loading, ASIC bring-up, DCN 3.2 display, 2D engine).
- `src/reactos`: the miniport, the display driver and their ReactOS glue.
- `src/opengl`: the OpenGL driver.
- `src/tools`: `radeonctl`, the Setup wizard and diagnostic tools.
- `tests`: host tests.
- `patches`: optional ReactOS `win32k` patches that let the display driver receive more operations. The released binaries were built without any of them.
- `third_party/qemu_lightning`: protocol headers for the Qemu-Lightning virtual display, which this tree also has guest drivers for (`lightningvideo`, `lightningdisp`, `lightningnt5`).

`rx7900video.sys`, `radeondisp.dll` and `radeonglicd.dll` in the release were built on 2026-10-24 and 2026-10-25; the hashes of their main sources recorded in the package's `BUILD-PROVENANCE.txt` match this tree. `radeonctl.exe` and `Setup.exe` are from 2026-11-14 and include the installer fixes the board needed.

## Build

Host tests need a C++17 compiler and CMake:

```sh
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build -j4
ctest --test-dir build
```

On the tree as published 96 of 102 tests pass. The other six (`opengl_gfx11_registers`, `opengl_gfx11_draw`, `opengl_pixel_paths`, `process_graphics`, `dcn32_stream`, `dcn32_stream_registers`) compare against pinned Linux v6.12 and Mesa register headers that lived outside the project (`../../work/upstream-gpuinit`, `../../work/upstream-dcn-stream`) and are not included.

The installer and the wizard build with MinGW (i686-w64-mingw32, GCC 13):

```sh
cmake -S . -B build-win -DCMAKE_TOOLCHAIN_FILE=$PWD/cmake/mingw-i386.cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
cmake --build build-win --target radeonctl rx7900setup
```

`radeonctl.exe` built this way and stripped differs from the released file in six timestamp and checksum bytes. The released `Setup.exe` was built directly with `i686-w64-mingw32-g++ -std=c++17 -O2 -municode -mwindows -static -s`.

The drivers build inside a ReactOS source tree at revision `a97c8a2cdcf6a9a7f5cc543951a02ccb7a80d8ca`. `cmake -DREACTOS_SOURCE=/path/to/reactos -P cmake/integrate.cmake` copies this project to `drivers/base/rx7900` and adds it to the ReactOS build; configure ReactOS with `-DRX7900_RADEON_2D=ON` and build `rx7900video`, `radeondisp` and `radeonctl`. `cmake/driver-package.cmake` automates this and assembles the package. `CMakeLists.as-built-native.txt` is the build description the released drivers were built with.

Gaps in this publication: the wrapper scripts that drove `driver-package.cmake` and the optional ReactOS patches other than the six in `patches/` (PCI resources, framebuffer shadow, brush origin, clip visibility, device bitmaps, hot-unplug and removal) were not preserved, so the options that need them must stay off. The driver build was not repeated for this publication.

## Licence

The project is distributed under the GNU General Public License, version 2 or (at your option) any later version; see [LICENSE](LICENSE). Code adapted from the Linux `amdgpu` and DRM drivers keeps its MIT-style permission notices, listed with sources and hashes in [THIRD_PARTY.txt](THIRD_PARTY.txt) and `licenses/`. `src/tools/gpu_setup_wizard.cpp` is GPL-2.0-only and shared with [B650M-P-USB-ReactOS](https://github.com/NullMagic2/B650M-P-USB-ReactOS). The firmware in the release package is AMD's, redistributed unmodified in binary form under the licence in `Firmware/LICENSE.amdgpu.txt`; it is not part of this source tree.

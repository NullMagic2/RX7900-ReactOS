# Radeon RX 7900 for ReactOS

Welcome! This project brings the **AMD Radeon RX 7900** family of graphics cards (Navi31) to **[ReactOS](https://reactos.org)**, the free operating system that runs Windows programs and drivers.

ReactOS has no driver of its own for these cards, so it falls back to a basic, generic display mode. The drivers here talk to the card directly, and come with a small setup wizard so that installing them takes a few clicks.

**This is an early, experimental release.** It has run on one card so far. Please keep a backup of the system you install it on, and a second way to reach the machine in case the screen stays dark.

**[Download RX7900-ReactOS-Standalone-i386.zip](https://github.com/NullMagic2/RX7900-ReactOS/releases/download/v0.1.0-alpha.2/RX7900-ReactOS-Standalone-i386.zip)** (pre-release v0.1.0-alpha.2)

## What you get

- **A native display driver.** ReactOS drives the card itself, and you can change the screen resolution.
- **A setup wizard.** Pick the options you want and it does the rest.
- **Optional extras.** Accelerated 2D drawing, a hardware mouse pointer and an OpenGL driver. These are included but not yet proven on a real card; see below.
- **A way back.** The installer keeps a journal and can roll back, remove the driver or recover from an interrupted installation.
- **A check program.** `RadeonCheck.exe` tests drawing, the mouse pointer and OpenGL on your machine and writes a report.

## Is this for you?

It is, if you have a Radeon RX 7900 card, run the 32-bit (i386) version of ReactOS, and enjoy trying things that are still rough around the edges.

The one card tested is a **Radeon RX 7900 XTX** (PCI `1002:744C`). The driver also accepts the IDs `1002:7448`, `744A`, `744B` and `745E`, but nobody has tried those yet. If you do, we would love to hear how it went.

## Getting started

1. Download the ZIP above and extract it inside **ReactOS i386**.
2. Run **Setup.exe** as administrator, choose your options and let it install.
3. Restart ReactOS.

Setup never restarts the computer on its own. It replaces the display driver currently in use and copies the AMD firmware files that come in the package. If something goes wrong, `INSTALL.txt` in the package explains how to check the status, roll back, remove the driver or recover.

Your card's own video ROM is not part of the download. The driver reads it from the card; `FIRMWARE.txt` explains what to do in the rare case where it cannot.

## How far along is it?

**Seen working on a real card**

- The package installs with every option switched on.
- After the restart the driver loads.
- The screen resolution can be changed.

That was on one RX 7900 XTX in an MSI PRO B650M-P, running ReactOS 0.4.16.

**Not checked yet**

- Whether accelerated 2D drawing is correct.
- The hardware mouse pointer.
- OpenGL.
- Display outputs other than the one in use, other cards and other motherboards.

**You can help.** Run `RadeonCheck.exe` from the package. It installs nothing and changes nothing: it draws in a small window, moves the mouse pointer and, after asking you, renders a small OpenGL scene. It writes each step to `RadeonCheck report.txt` before running it, so even if the machine stops, the report shows how far it got. Sharing that report in an issue is the most useful thing you can do for this project right now.

## What is in the package

| File | What it is |
|---|---|
| `Setup.exe` | The setup wizard |
| `radeonctl.exe` | The command-line installer behind the wizard, with rollback and recovery |
| `rx7900video.sys` | The driver that talks to the card |
| `radeondisp.dll` | The driver that draws the ReactOS desktop |
| `radeonglicd.dll` | The OpenGL driver (optional) |
| `radeondisplay.inf` | Tells ReactOS which cards the driver is for |
| `RadeonCheck.exe` | The check program |
| `Firmware/` | AMD's own firmware files for the card, unmodified, under AMD's redistribution licence |

## For developers

The portable core (card bring-up, display, 2D engine) lives in `src/`, with the ReactOS drivers in `src/reactos`, the OpenGL driver in `src/opengl` and the installer and wizard in `src/tools`. The host tests need only a C++17 compiler and CMake:

```sh
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build -j4
ctest --test-dir build
```

**[BUILDING.md](BUILDING.md)** has the rest: the source layout, building the installer and the drivers, how the released files were checked against a rebuild, and the optional ReactOS patches. What was observed on the real card is recorded in [evidence/](evidence).

## Related project

**[B650M-P-USB-ReactOS](https://github.com/NullMagic2/B650M-P-USB-ReactOS)** makes the USB ports of the MSI PRO B650M-P work in ReactOS and lets that board start ReactOS from a USB stick. The two were developed side by side on the same machine.

## What changed in each version

The [Releases page](https://github.com/NullMagic2/RX7900-ReactOS/releases) has the notes for every version.

## Questions and feedback

If something does not work, or you tried this on your own card, please [open an issue](https://github.com/NullMagic2/RX7900-ReactOS/issues). Reports from real hardware are especially welcome.

## Licence

The project is distributed under the GNU General Public License, version 2 or (at your option) any later version; see [LICENSE](LICENSE). Code adapted from the Linux `amdgpu` and DRM drivers keeps its MIT-style permission notices, listed in [THIRD_PARTY.txt](THIRD_PARTY.txt) and `licenses/`. The setup wizard source (`src/tools/gpu_setup_wizard.cpp`) is GPL-2.0-only and shared with B650M-P-USB-ReactOS. The firmware in the release package is AMD's, redistributed unmodified under the licence in `Firmware/LICENSE.amdgpu.txt`; it is not part of this source tree.

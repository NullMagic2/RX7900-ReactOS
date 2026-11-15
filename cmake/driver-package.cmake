# Module: rx7900.build.package
# Purpose: Share native builds and ownership-checked cleanup across both shells.
# Created: 2026-10-21
# Architecture: Intermediates are owned; source and published packages survive clean.
cmake_minimum_required(VERSION 3.17)
get_filename_component(project_root "${CMAKE_CURRENT_LIST_DIR}/.." REALPATH)
set(binaries "${project_root}/Binaries")
set(workspace "${binaries}/.build/driver-package")
# Optional private roots for a candidate build (for example Release) that must
# not replace the shared intermediates or the published Binaries/<arch>
# package. Both default to the shared locations and keep the same ownership
# markers, so clean_build only removes an owned override workspace.
if(DEFINED DRIVER_WORKSPACE)
    get_filename_component(workspace "${DRIVER_WORKSPACE}" ABSOLUTE)
endif()
set(owner_text "RX7900 driver workspace v1\n${project_root}\n")

function(reject_symlinks)
    foreach(path IN LISTS ARGN)
        if(IS_SYMLINK "${path}")
            message(FATAL_ERROR "Refusing symlink build path: ${path}")
        endif()
    endforeach()
endfunction()
reject_symlinks("${binaries}" "${binaries}/.build" "${workspace}"
    "${CMAKE_CURRENT_LIST_DIR}/.driver-package.lock")
file(LOCK "${CMAKE_CURRENT_LIST_DIR}/.driver-package.lock" GUARD PROCESS
    TIMEOUT 0 RESULT_VARIABLE lock_result)
if(NOT lock_result STREQUAL "0")
    message(FATAL_ERROR "Another build/cleanup is running: ${lock_result}")
endif()

if(EXISTS "${workspace}")
    reject_symlinks("${workspace}/.driver-owned" "${workspace}/.in-progress")
    if(NOT EXISTS "${workspace}/.driver-owned")
        message(FATAL_ERROR "Refusing an unowned build directory: ${workspace}")
    endif()
    file(READ "${workspace}/.driver-owned" actual_owner)
    if(NOT actual_owner STREQUAL owner_text)
        message(FATAL_ERROR "Build ownership does not match this project")
    endif()
    if(EXISTS "${workspace}/.in-progress")
        message(FATAL_ERROR "An interrupted or external build may still be running. Preserve the workspace and resolve .in-progress before retrying.")
    endif()
endif()

if(DRIVER_ACTION STREQUAL "clean")
    if(EXISTS "${workspace}")
        file(GLOB_RECURSE owned_entries LIST_DIRECTORIES TRUE "${workspace}/*")
        reject_symlinks(${owned_entries})
        file(REMOVE_RECURSE "${workspace}")
    endif()
    message(STATUS "Owned intermediates removed; source and Binaries/i386 or Binaries/amd64 packages preserved")
    return()
elseif(NOT DRIVER_ACTION STREQUAL "build")
    message(FATAL_ERROR "Use build.sh/build.bat or clean_build.sh/clean_build.bat")
endif()

if(NOT DEFINED DRIVER_ARCH)
    set(DRIVER_ARCH i386)
endif()
if(NOT DRIVER_ARCH MATCHES "^(i386|amd64)$")
    message(FATAL_ERROR "DRIVER_ARCH must be i386 or amd64")
endif()
if(NOT DEFINED DRIVER_COMPILER)
    set(DRIVER_COMPILER gcc)
endif()
if(NOT DRIVER_COMPILER MATCHES "^(gcc|msvc)$")
    message(FATAL_ERROR "DRIVER_COMPILER must be gcc or msvc")
endif()
if(NOT DEFINED RX7900_LIGHTNING_GDI_OFFLOAD)
    # The packaged virtual driver must contain its GDI/OpenGL integration.
    # Runtime capability checks still require the matching Lightning host;
    # this setting does not initialize or bind a physical Radeon adapter.
    set(RX7900_LIGHTNING_GDI_OFFLOAD ON)
endif()
if(NOT RX7900_LIGHTNING_GDI_OFFLOAD MATCHES "^(ON|OFF)$")
    message(FATAL_ERROR "RX7900_LIGHTNING_GDI_OFFLOAD must be ON or OFF")
endif()
# Optional Lightning transport variants, passed to the native build (default OFF).
foreach(option_name RX7900_LIGHTNING_POSTED_SUBMIT RX7900_LIGHTNING_TRANSPORT_TIMING)
    if(NOT DEFINED ${option_name})
        set(${option_name} OFF)
    endif()
    if(NOT ${option_name} MATCHES "^(ON|OFF)$")
        message(FATAL_ERROR "${option_name} must be ON or OFF")
    endif()
endforeach()
if(NOT DEFINED DRIVER_BUILD_TYPE)
    # ReactOS maps Debug to DBG=1/KDBG=1 with OPTIMIZE (default -O1) and
    # Release to DBG=0/KDBG=0 with -O2 -DNDEBUG= (sdk/cmake/config.cmake, gcc.cmake).
    set(DRIVER_BUILD_TYPE Debug)
endif()
if(NOT DRIVER_BUILD_TYPE MATCHES "^(Debug|Release)$")
    message(FATAL_ERROR "DRIVER_BUILD_TYPE must be Debug or Release")
endif()
if(DRIVER_COMPILER STREQUAL "msvc" AND NOT CMAKE_HOST_WIN32)
    message(FATAL_ERROR "Run the MSVC build in a VS2022 developer command prompt")
endif()
if(NOT DEFINED DRIVER_JOBS)
    set(DRIVER_JOBS 6)
endif()
if(NOT DRIVER_JOBS MATCHES "^[1-9][0-9]*$" OR DRIVER_JOBS GREATER 256)
    message(FATAL_ERROR "DRIVER_JOBS must be between 1 and 256")
endif()
if(NOT DEFINED REACTOS_SOURCE AND DEFINED ENV{REACTOS_SOURCE})
    set(REACTOS_SOURCE "$ENV{REACTOS_SOURCE}")
endif()
if(NOT DEFINED REACTOS_SOURCE)
    set(REACTOS_SOURCE "${project_root}/../../work/reactos")
endif()
get_filename_component(REACTOS_SOURCE "${REACTOS_SOURCE}" REALPATH)
if(NOT EXISTS "${REACTOS_SOURCE}/toolchain-${DRIVER_COMPILER}.cmake")
    message(FATAL_ERROR "Pass -DREACTOS_SOURCE=/path/to/reactos with the selected native toolchain")
endif()

# Reuse the workspace's unpacked tools when present, without installing globally.
get_filename_component(task_root "${project_root}/../.." REALPATH)
set(tool_root "${task_root}/work/tooling/root/usr")
if(NOT CMAKE_HOST_WIN32 AND EXISTS "${tool_root}/bin/ninja")
    set(ENV{PATH} "${tool_root}/bin:$ENV{PATH}")
    if(NOT DEFINED ENV{BISON_PKGDATADIR})
        set(ENV{BISON_PKGDATADIR} "${tool_root}/share/bison")
    endif()
    if(NOT DEFINED ENV{M4})
        set(ENV{M4} "${tool_root}/bin/m4")
    endif()
endif()
find_program(driver_ninja ninja REQUIRED)
file(MAKE_DIRECTORY "${workspace}")
file(WRITE "${workspace}/.driver-owned" "${owner_text}")

# Optional physical Radeon package (radeon-installer, 2026-10-24, additive).
# DRIVER_PACKAGE defaults to lightning, whose flow below is unchanged. radeon
# builds rx7900video.sys, radeondisp.dll (RX7900_RADEON_2D=ON) and radeonctl.exe
# natively, radeonglicd.dll with the GCC13 MinGW toolchain, and publishes them in
# Binaries/radeon-<arch> (outside the Lightning Binaries/<arch> directory that
# Lightning publication renames), or DRIVER_PACKAGE_DIR, with its own marker.
if(NOT DEFINED DRIVER_PACKAGE)
    set(DRIVER_PACKAGE lightning)
endif()
if(NOT DRIVER_PACKAGE MATCHES "^(lightning|radeon)$")
    message(FATAL_ERROR "DRIVER_PACKAGE must be lightning or radeon")
endif()
if(DRIVER_PACKAGE STREQUAL "radeon")
    set(radeon_native "${workspace}/native-${DRIVER_ARCH}-radeon")
    set(radeon_mingw "${workspace}/mingw-${DRIVER_ARCH}-radeon")
    set(radeon_staged "${workspace}/package-radeon-${DRIVER_ARCH}")
    set(radeon_previous "${workspace}/previous-radeon-${DRIVER_ARCH}")
    set(radeon_package "${binaries}/radeon-${DRIVER_ARCH}")
    if(DEFINED DRIVER_PACKAGE_DIR)
        get_filename_component(radeon_package "${DRIVER_PACKAGE_DIR}" ABSOLUTE)
    endif()
    set(radeon_owner "${owner_text}radeon-${DRIVER_ARCH}\n")
    reject_symlinks("${radeon_native}" "${radeon_mingw}" "${radeon_staged}" "${radeon_previous}" "${radeon_package}")
    foreach(lightning_root "${binaries}/i386" "${binaries}/amd64")
        get_filename_component(lightning_real "${lightning_root}" ABSOLUTE)
        if(radeon_package STREQUAL lightning_real)
            message(FATAL_ERROR "The Radeon package never replaces the Lightning package ${lightning_root}")
        endif()
    endforeach()
    if(DRIVER_ARCH STREQUAL "amd64")
        set(radeon_toolchain "${project_root}/cmake/mingw-amd64.cmake")
    else()
        set(radeon_toolchain "${project_root}/cmake/mingw-i386.cmake")
    endif()
    set(radeon_config "${REACTOS_SOURCE}\n${DRIVER_COMPILER}\n${DRIVER_ARCH}\n${DRIVER_BUILD_TYPE}\nradeon2d=ON\n")
    if(EXISTS "${radeon_native}/.driver-config")
        reject_symlinks("${radeon_native}/.driver-config")
        file(READ "${radeon_native}/.driver-config" radeon_previous_config)
        if(NOT radeon_previous_config STREQUAL radeon_config)
            message(FATAL_ERROR "Source/compiler changed; clean owned intermediates before rebuilding")
        endif()
    endif()
    file(MAKE_DIRECTORY "${radeon_native}")
    file(WRITE "${radeon_native}/.driver-config" "${radeon_config}")
    file(WRITE "${workspace}/.in-progress" "Build owns this workspace until its subprocesses finish.\n")

    function(radeon_checked description)
        execute_process(COMMAND ${ARGN} RESULT_VARIABLE command_result)
        if(NOT command_result STREQUAL "0")
            file(REMOVE "${workspace}/.in-progress")
            message(FATAL_ERROR "${description} failed (${command_result}); source and existing package retained")
        endif()
    endfunction()

    # Source integration only: no optional OS patch is applied for this package.
    radeon_checked("ReactOS integration" "${CMAKE_COMMAND}" "-DREACTOS_SOURCE=${REACTOS_SOURCE}"
        -P "${CMAKE_CURRENT_LIST_DIR}/integrate.cmake")
    radeon_checked("Native configure" "${CMAKE_COMMAND}" -S "${REACTOS_SOURCE}"
        -B "${radeon_native}" -G Ninja "-DCMAKE_MAKE_PROGRAM=${driver_ninja}"
        "-DCMAKE_TOOLCHAIN_FILE=${REACTOS_SOURCE}/toolchain-${DRIVER_COMPILER}.cmake"
        "-DARCH=${DRIVER_ARCH}" "-DCMAKE_BUILD_TYPE=${DRIVER_BUILD_TYPE}" -DRX7900_RADEON_2D=ON)
    # Generated kernel headers first: the static libraries linked into the
    # miniport do not inherit the module's bugcodes/xdk dependency.
    radeon_checked("Native generated headers" "${CMAKE_COMMAND}" --build "${radeon_native}"
        --target bugcodes xdk --parallel "${DRIVER_JOBS}")
    radeon_checked("Native Radeon driver and installer build" "${CMAKE_COMMAND}" --build "${radeon_native}"
        --target rx7900video radeondisp radeonctl --parallel "${DRIVER_JOBS}")
    # radeonglicd is a user-mode Win32 DLL built by this project's own GCC13
    # MinGW configuration (it is not part of the ReactOS tree build).
    radeon_checked("MinGW ICD configure" "${CMAKE_COMMAND}" -S "${project_root}" -B "${radeon_mingw}"
        -G Ninja "-DCMAKE_MAKE_PROGRAM=${driver_ninja}" "-DCMAKE_TOOLCHAIN_FILE=${radeon_toolchain}"
        -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON)
    radeon_checked("MinGW ICD build" "${CMAKE_COMMAND}" --build "${radeon_mingw}" --target radeonglicd
        --parallel "${DRIVER_JOBS}")

    set(radeon_native_payload rx7900video.sys radeondisp.dll radeonctl.exe)
    reject_symlinks("${radeon_staged}")
    if(EXISTS "${radeon_staged}")
        file(GLOB_RECURSE radeon_stale LIST_DIRECTORIES TRUE "${radeon_staged}/*")
        reject_symlinks(${radeon_stale})
        file(REMOVE_RECURSE "${radeon_staged}")
    endif()
    file(MAKE_DIRECTORY "${radeon_staged}")
    foreach(name IN LISTS radeon_native_payload)
        set(source "${radeon_native}/drivers/base/rx7900/${name}")
        if(NOT EXISTS "${source}")
            file(REMOVE "${workspace}/.in-progress")
            message(FATAL_ERROR "Expected native output missing: ${source}")
        endif()
        radeon_checked("Package ${name}" "${CMAKE_COMMAND}" -E copy "${source}" "${radeon_staged}/${name}")
    endforeach()
    if(NOT EXISTS "${radeon_mingw}/radeonglicd.dll")
        file(REMOVE "${workspace}/.in-progress")
        message(FATAL_ERROR "Expected MinGW output missing: ${radeon_mingw}/radeonglicd.dll")
    endif()
    radeon_checked("Package radeonglicd.dll" "${CMAKE_COMMAND}" -E copy "${radeon_mingw}/radeonglicd.dll"
        "${radeon_staged}/radeonglicd.dll")
    file(COPY "${project_root}/radeondisplay.inf" "${project_root}/THIRD_PARTY.txt"
        "${project_root}/licenses" DESTINATION "${radeon_staged}")
    file(MAKE_DIRECTORY "${radeon_staged}/patches")
    foreach(patch reactos-stretch-guard.patch reactos-radeon-bridge.patch reactos-radeon-text-bridge.patch
            reactos-draw-escape.patch reactos-text-batch.patch reactos-cursor-recovery.patch)
        file(COPY "${project_root}/patches/${patch}" DESTINATION "${radeon_staged}/patches")
    endforeach()

    # Provenance: payload and key source digests, build configuration.
    set(radeon_sums "")
    foreach(name rx7900video.sys radeondisp.dll radeonglicd.dll radeonctl.exe radeondisplay.inf)
        file(SHA256 "${radeon_staged}/${name}" digest)
        string(APPEND radeon_sums "${digest}  ${name}\n")
    endforeach()
    file(WRITE "${radeon_staged}/SHA256SUMS.txt" "${radeon_sums}")
    set(radeon_sources "")
    foreach(source src/reactos/video.cpp src/reactos/radeon_display.cpp src/reactos/radeon_opengl_info.hpp
            src/opengl/icd/icd.cpp src/opengl/icd/radeonglicd.spec src/tools/radeon_control.cpp
            src/tools/radeon_installer.cpp src/tools/radeon_install_policy.cpp src/tools/radeon_win32_system.cpp
            radeondisplay.inf CMakeLists.txt)
        file(SHA256 "${project_root}/${source}" digest)
        string(APPEND radeon_sources "${digest}  ${source}\n")
    endforeach()
    execute_process(COMMAND git -C "${REACTOS_SOURCE}" rev-parse HEAD OUTPUT_VARIABLE reactos_head
        OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET RESULT_VARIABLE reactos_head_result)
    if(NOT reactos_head_result STREQUAL "0")
        set(reactos_head "unknown")
    endif()
    file(WRITE "${radeon_staged}/BUILD-PROVENANCE.txt"
        "RX7900 physical Radeon package (generated by cmake/driver-package.cmake DRIVER_PACKAGE=radeon)\n"
        "Native architecture: ${DRIVER_ARCH}; compiler: ${DRIVER_COMPILER}; build type: ${DRIVER_BUILD_TYPE}\n"
        "Native options: RX7900_RADEON_2D=ON (rx7900video, radeondisp, radeonctl)\n"
        "radeonglicd.dll: this project with ${radeon_toolchain} (GCC13 MinGW, Release, static runtime)\n"
        "Integrated ReactOS source: ${REACTOS_SOURCE} at ${reactos_head}\n\n"
        "Payload SHA-256:\n${radeon_sums}\nSource SHA-256:\n${radeon_sources}")
    file(WRITE "${radeon_staged}/FIRMWARE.txt"
        "Navi31 firmware placement for rx7900video (PhysicalAsicInit=1 only)\n\n"
        "The package contains no AMD firmware. Obtain the amdgpu files from linux-firmware\n"
        "(directory amdgpu/, the AMD redistribution licence in LICENSE.amdgpu) and place them,\n"
        "with exactly these names, in %SystemRoot%\\System32\\drivers\\rx7900\\:\n\n"
        "Required: gc_11_0_0_pfp.bin gc_11_0_0_me.bin gc_11_0_0_mec.bin\n"
        "          gc_11_0_0_rlc.bin (gc_11_0_0_rlc_1.bin on PCI revision CE)\n"
        "          gc_11_0_0_mes_2.bin (or gc_11_0_0_mes.bin) gc_11_0_0_mes1.bin gc_11_0_0_imu.bin\n"
        "          sdma_6_0_0.bin smu_13_0_0.bin psp_13_0_0_sos.bin\n"
        "Optional: dcn_3_2_0_dmcub.bin psp_13_0_0_ta.bin\n"
        "VBIOS:    rx7900.rom is read only as the last resort, or only with AtomRomOverride=1.\n"
        "          Dump it from the same card (for example Linux sysfs rom after 'echo 1 > rom').\n\n"
        "radeonctl firmware-copy <directory> copies exactly these names after the same structural\n"
        "checks the miniport applies; radeonctl status reports each file. The checks are not\n"
        "authentication: the PSP verifies AMD signatures when the firmware is loaded.\n")
    file(WRITE "${radeon_staged}/INSTALL.txt"
        "RX7900 physical Radeon (Navi31) ReactOS development package\n\n"
        "Native architecture: ${DRIVER_ARCH}; compiler: ${DRIVER_COMPILER}; build type: ${DRIVER_BUILD_TYPE}.\n"
        "Contents: rx7900video.sys (VideoPort miniport), radeondisp.dll (GDI display driver),\n"
        "radeonglicd.dll (RadeonGL WGL ICD, opt-in), radeondisplay.inf, radeonctl.exe (installer).\n"
        "Experimental: nothing in this package has run on an RX 7900. Keep a recoverable disk image\n"
        "and a second way to reach the machine (serial/remote) before installing.\n\n"
        "Commands (run as administrator from this directory):\n"
        "  radeonctl status                 read-only report (devices, service, files and SHA-256,\n"
        "                                   opt-ins, ICD, firmware, win32k patch capability)\n"
        "  radeonctl install [opt-ins]      bind the one present Navi31 GPU; all opt-ins default to 0\n"
        "      --asic (PhysicalAsicInit) --rom-override (AtomRomOverride) --vbe (VbeModeSet)\n"
        "      --2d (Radeon2D) --2d-compute (Radeon2DCompute) --cursor (HardwareCursor)\n"
        "      --opengl-icd (radeonglicd.dll + OpenGLDrivers\\RadeonGL) --dry-run\n"
        "      --force-replace <Service>    required to replace an existing driver binding (vbemp etc.)\n"
        "  radeonctl upgrade                replace installed files from a newer package, keeping a backup\n"
        "  radeonctl rollback               restore the backup\n"
        "  radeonctl remove                 remove binding, service, files and ICD registration\n"
        "  radeonctl recover                finish or undo an interrupted command; repair records\n"
        "  radeonctl firmware-copy <dir>    copy validated firmware (see FIRMWARE.txt)\n"
        "No command reboots, starts a driver, resets the GPU or sets boot start. Exit code 3010\n"
        "means restart ReactOS; 4 means no Navi31 GPU is present (for example the Lightning VM).\n\n"
        "Bare metal: install ReactOS on the RX 7900 machine (UEFI/GOP or CSM boot), copy this package,\n"
        "run radeonctl status, place firmware (FIRMWARE.txt), then for example\n"
        "  radeonctl install --force-replace <current service>   (first boot: all opt-ins 0)\n"
        "  restart, verify with radeonctl status, then radeonctl install --asic [--2d ...] and restart.\n"
        "VFIO passthrough: bind the GPU (and its HDMI audio function) to vfio-pci on the Linux host,\n"
        "pass both to the ReactOS VM with the VBIOS (romfile=) if the guest cannot read it, keep an\n"
        "emulated VGA for FreeLoader, then follow the bare-metal steps inside the guest.\n\n"
        "Optional win32k patches (patches/) let radeondisp receive more operations. radeonctl status\n"
        "reports which are present in the installed win32k.sys; it never replaces kernel files.\n"
        "Rebuild win32k for the exact installed ReactOS release to add them.\n")
    file(WRITE "${radeon_staged}/.driver-package-owned" "${radeon_owner}")

    # Publish atomically, keeping the previous package for recovery.
    if(EXISTS "${radeon_package}")
        reject_symlinks("${radeon_package}/.driver-package-owned")
        if(NOT EXISTS "${radeon_package}/.driver-package-owned")
            file(REMOVE "${workspace}/.in-progress")
            message(FATAL_ERROR "Refusing to overwrite an unowned published package")
        endif()
        file(READ "${radeon_package}/.driver-package-owned" package_owner)
        if(NOT package_owner STREQUAL radeon_owner)
            file(REMOVE "${workspace}/.in-progress")
            message(FATAL_ERROR "Published package ownership mismatch")
        endif()
        if(EXISTS "${radeon_previous}")
            reject_symlinks("${radeon_previous}/.driver-package-owned")
            if(NOT EXISTS "${radeon_previous}/.driver-package-owned")
                file(REMOVE "${workspace}/.in-progress")
                message(FATAL_ERROR "Refusing to remove an unowned previous package")
            endif()
            file(READ "${radeon_previous}/.driver-package-owned" previous_owner)
            if(NOT previous_owner STREQUAL radeon_owner)
                file(REMOVE "${workspace}/.in-progress")
                message(FATAL_ERROR "Previous package ownership mismatch")
            endif()
            file(GLOB_RECURSE previous_entries LIST_DIRECTORIES TRUE "${radeon_previous}/*")
            reject_symlinks(${previous_entries})
            file(REMOVE_RECURSE "${radeon_previous}")
        endif()
        radeon_checked("Preserve previous package" "${CMAKE_COMMAND}" -E rename "${radeon_package}"
            "${radeon_previous}")
    endif()
    execute_process(COMMAND "${CMAKE_COMMAND}" -E rename "${radeon_staged}" "${radeon_package}"
        RESULT_VARIABLE published)
    if(NOT published STREQUAL "0")
        if(EXISTS "${radeon_previous}")
            execute_process(COMMAND "${CMAKE_COMMAND}" -E rename "${radeon_previous}" "${radeon_package}")
        endif()
        file(REMOVE "${workspace}/.in-progress")
        message(FATAL_ERROR "Publication failed; previous package restoration attempted")
    endif()
    file(REMOVE "${workspace}/.in-progress")
    message(STATUS "Physical Radeon package: ${radeon_package}")
    return()
endif()

set(native "${workspace}/native-${DRIVER_ARCH}")
reject_symlinks("${native}")
set(config_text "${REACTOS_SOURCE}\n${DRIVER_COMPILER}\n${DRIVER_ARCH}\n")
if(NOT DRIVER_BUILD_TYPE STREQUAL "Debug")
    # A Debug workspace keeps its existing marker; another type never reuses it.
    string(APPEND config_text "${DRIVER_BUILD_TYPE}\n")
endif()
if(RX7900_LIGHTNING_POSTED_SUBMIT OR RX7900_LIGHTNING_TRANSPORT_TIMING)
    # A variant workspace never reuses the default one's objects.
    string(APPEND config_text "posted=${RX7900_LIGHTNING_POSTED_SUBMIT} timing=${RX7900_LIGHTNING_TRANSPORT_TIMING}\n")
endif()
if(EXISTS "${native}/.driver-config")
    reject_symlinks("${native}/.driver-config")
    file(READ "${native}/.driver-config" previous_config)
    if(NOT previous_config STREQUAL config_text)
        message(FATAL_ERROR "Source/compiler changed; clean owned intermediates before rebuilding")
    endif()
endif()
file(MAKE_DIRECTORY "${native}")
file(WRITE "${native}/.driver-config" "${config_text}")
file(WRITE "${workspace}/.in-progress" "Build owns this workspace until its subprocesses finish.\n")

function(run_checked description)
    execute_process(COMMAND ${ARGN} RESULT_VARIABLE command_result)
    if(NOT command_result STREQUAL "0")
        file(REMOVE "${workspace}/.in-progress")
        message(FATAL_ERROR "${description} failed (${command_result}); source and existing package retained")
    endif()
endfunction()
# The removal exercise package opts into the kernel removal patches (ACPI PCI
# hot-unplug, Device Manager query-remove, removed-display release and primary
# switch, videoprt teardown on REMOVE) and the lease-fault knob. All default
# OFF, so an ordinary package build is unchanged.
foreach(removal_patch ACPI_HOTPLUG PNP_QUERY_REMOVE WIN32K_DISPLAY_RELEASE VIDEOPRT_REMOVE)
    if(NOT DEFINED REACTOS_APPLY_${removal_patch})
        set(REACTOS_APPLY_${removal_patch} OFF)
    endif()
endforeach()
if(NOT DEFINED RX7900_LIGHTNING_LEASE_FAULTS)
    set(RX7900_LIGHTNING_LEASE_FAULTS OFF)
endif()
# Each patched kernel image is built beside the drivers for root to install.
set(removal_kernel_targets)
if(REACTOS_APPLY_ACPI_HOTPLUG)
    list(APPEND removal_kernel_targets acpi pci)
endif()
if(REACTOS_APPLY_PNP_QUERY_REMOVE)
    list(APPEND removal_kernel_targets ntoskrnl ntkrnlmp)
endif()
if(REACTOS_APPLY_WIN32K_DISPLAY_RELEASE)
    list(APPEND removal_kernel_targets win32k)
endif()
if(REACTOS_APPLY_VIDEOPRT_REMOVE)
    list(APPEND removal_kernel_targets videoprt)
endif()
run_checked("ReactOS integration" "${CMAKE_COMMAND}"
    "-DREACTOS_SOURCE=${REACTOS_SOURCE}" -DREACTOS_APPLY_FRAMEBUFFER_SHADOW=ON
    "-DREACTOS_APPLY_ACPI_HOTPLUG=${REACTOS_APPLY_ACPI_HOTPLUG}"
    "-DREACTOS_APPLY_PNP_QUERY_REMOVE=${REACTOS_APPLY_PNP_QUERY_REMOVE}"
    "-DREACTOS_APPLY_WIN32K_DISPLAY_RELEASE=${REACTOS_APPLY_WIN32K_DISPLAY_RELEASE}"
    "-DREACTOS_APPLY_VIDEOPRT_REMOVE=${REACTOS_APPLY_VIDEOPRT_REMOVE}"
    -P "${CMAKE_CURRENT_LIST_DIR}/integrate.cmake")
run_checked("Native configure" "${CMAKE_COMMAND}" -S "${REACTOS_SOURCE}"
    -B "${native}" -G Ninja "-DCMAKE_MAKE_PROGRAM=${driver_ninja}"
    "-DCMAKE_TOOLCHAIN_FILE=${REACTOS_SOURCE}/toolchain-${DRIVER_COMPILER}.cmake"
    "-DARCH=${DRIVER_ARCH}" "-DCMAKE_BUILD_TYPE=${DRIVER_BUILD_TYPE}"
    "-DRX7900_LIGHTNING_GDI_OFFLOAD=${RX7900_LIGHTNING_GDI_OFFLOAD}"
    "-DRX7900_LIGHTNING_POSTED_SUBMIT=${RX7900_LIGHTNING_POSTED_SUBMIT}"
    "-DRX7900_LIGHTNING_TRANSPORT_TIMING=${RX7900_LIGHTNING_TRANSPORT_TIMING}"
    "-DRX7900_LIGHTNING_LEASE_FAULTS=${RX7900_LIGHTNING_LEASE_FAULTS}"
    "-DRX7900_ACPI_HOTPLUG=${REACTOS_APPLY_ACPI_HOTPLUG}"
    "-DRX7900_PNP_QUERY_REMOVE=${REACTOS_APPLY_PNP_QUERY_REMOVE}"
    "-DRX7900_WIN32K_DISPLAY_RELEASE=${REACTOS_APPLY_WIN32K_DISPLAY_RELEASE}"
    "-DRX7900_VIDEOPRT_REMOVE=${REACTOS_APPLY_VIDEOPRT_REMOVE}")
run_checked("Native driver and installer build" "${CMAKE_COMMAND}" --build "${native}"
    --target lightningvideo lightningdisp lightningnt5 lightningnt5ctl
        rx7900probe rx7900ctl framebuffer_guest_smoke ${removal_kernel_targets}
    --parallel "${DRIVER_JOBS}")

# A direct Ninja build from an older ReactOS checkout can silently package a
# stale renderer. Verify the exact integrated source and a renderer diagnostic
# in the linked image before publishing any binaries.
set(renderer_source "${project_root}/src/reactos/lightning_renderer.cpp")
set(renderer_integrated "${REACTOS_SOURCE}/drivers/base/rx7900/src/reactos/lightning_renderer.cpp")
file(SHA256 "${renderer_source}" renderer_source_sha256)
file(SHA256 "${renderer_integrated}" renderer_integrated_sha256)
if(NOT renderer_source_sha256 STREQUAL renderer_integrated_sha256)
    file(REMOVE "${workspace}/.in-progress")
    message(FATAL_ERROR "Integrated renderer differs from live source; refusing stale package")
endif()
file(STRINGS "${native}/drivers/base/rx7900/lightningnt5.sys" renderer_diagnostics
    REGEX "LightningNT5: renderer resource cap operation")
if(NOT renderer_diagnostics)
    file(REMOVE "${workspace}/.in-progress")
    message(FATAL_ERROR "Built LightningNT5 lacks the renderer capacity diagnostic; refusing stale package")
endif()

set(staged "${workspace}/package-${DRIVER_ARCH}")
reject_symlinks("${staged}")
file(MAKE_DIRECTORY "${staged}")
set(payload lightningvideo.sys lightningdisp.dll lightningnt5.sys lightningnt5ctl.exe
    rx7900probe.sys rx7900ctl.exe framebuffer_guest_smoke.exe)
foreach(name IN LISTS payload)
    set(source "${native}/drivers/base/rx7900/${name}")
    if(NOT EXISTS "${source}")
        file(REMOVE "${workspace}/.in-progress")
        message(FATAL_ERROR "Expected native output missing: ${source}")
    endif()
    run_checked("Package ${name}" "${CMAKE_COMMAND}" -E copy_if_different "${source}" "${staged}/${name}")
endforeach()
file(COPY "${project_root}/lightningdisplay.inf" "${project_root}/lightningnt5.inf"
    "${project_root}/licenses" "${project_root}/patches" DESTINATION "${staged}")
file(WRITE "${staged}/BUILD-PROVENANCE.txt"
    "Live renderer source SHA-256: ${renderer_source_sha256}\n"
    "Integrated ReactOS source: ${REACTOS_SOURCE}\n"
    "Native architecture: ${DRIVER_ARCH}; compiler: ${DRIVER_COMPILER}\n")
if(NOT DRIVER_BUILD_TYPE STREQUAL "Debug")
    file(APPEND "${staged}/BUILD-PROVENANCE.txt" "Native build type: ${DRIVER_BUILD_TYPE}\n")
endif()
if(DRIVER_ARCH STREQUAL "i386")
    if(NOT DEFINED LIGHTNING_OPENGL_PACKAGE)
        set(LIGHTNING_OPENGL_PACKAGE "${binaries}/${DRIVER_ARCH}/LightningGL")
    endif()
    get_filename_component(LIGHTNING_OPENGL_PACKAGE "${LIGHTNING_OPENGL_PACKAGE}" REALPATH)
    reject_symlinks("${LIGHTNING_OPENGL_PACKAGE}")
    foreach(name QemuLightningOpenGL.dll QemuLightningSync.dll QemuLightningVulkan.dll
            QemuLightningVulkanLoader.dll QemuLightningVulkan.json)
        reject_symlinks("${LIGHTNING_OPENGL_PACKAGE}/${name}")
        if(NOT EXISTS "${LIGHTNING_OPENGL_PACKAGE}/${name}")
            file(REMOVE "${workspace}/.in-progress")
            message(FATAL_ERROR "The i386 installer requires its matched OpenGL runtime: ${LIGHTNING_OPENGL_PACKAGE}/${name}. Set LIGHTNING_OPENGL_PACKAGE to the built runtime package.")
        endif()
    endforeach()
    file(GLOB_RECURSE opengl_entries LIST_DIRECTORIES TRUE "${LIGHTNING_OPENGL_PACKAGE}/*")
    reject_symlinks(${opengl_entries})
    file(COPY "${LIGHTNING_OPENGL_PACKAGE}/" DESTINATION "${staged}/LightningGL")
    string(CONCAT opengl_install_text
        "The i386 display installer also installs its adapter-selected Mesa OpenGL ICD and native Vulkan runtime.\n"
        "It preserves the system opengl32.dll; normal applications use the installed ICD through ReactOS.\n")
else()
    # AMD64 (Amd64_gl_runtime, 2026-10-24): the matched native x64 Venus/Zink
    # runtime is required exactly as on i386, from LIGHTNING_OPENGL_PACKAGE
    # (default Binaries/amd64/LightningGL; build it with lightning/opengl
    # QL_REACTOS_ARCH=amd64). Never promote an interrupted or i386 staging
    # directory merely because it contains the usual LightningGL names: the
    # stage must be clean and every packaged PE image must be PE32+ AMD64,
    # because ReactOS AMD64 has no 32-bit subsystem.
    reject_symlinks("${staged}/LightningGL")
    if(EXISTS "${staged}/LightningGL")
        file(REMOVE "${workspace}/.in-progress")
        message(FATAL_ERROR "AMD64 stage contains an OpenGL runtime from an earlier build; clean owned intermediates before packaging")
    endif()
    if(NOT DEFINED LIGHTNING_OPENGL_PACKAGE)
        set(LIGHTNING_OPENGL_PACKAGE "${binaries}/${DRIVER_ARCH}/LightningGL")
    endif()
    get_filename_component(LIGHTNING_OPENGL_PACKAGE "${LIGHTNING_OPENGL_PACKAGE}" REALPATH)
    reject_symlinks("${LIGHTNING_OPENGL_PACKAGE}")
    foreach(name QemuLightningOpenGL.dll QemuLightningSync.dll QemuLightningVulkan.dll
            QemuLightningVulkanLoader.dll QemuLightningVulkan.json)
        reject_symlinks("${LIGHTNING_OPENGL_PACKAGE}/${name}")
        if(NOT EXISTS "${LIGHTNING_OPENGL_PACKAGE}/${name}")
            file(REMOVE "${workspace}/.in-progress")
            message(FATAL_ERROR "The AMD64 installer requires its matched native AMD64 OpenGL runtime: ${LIGHTNING_OPENGL_PACKAGE}/${name}. Set LIGHTNING_OPENGL_PACKAGE to the built runtime package.")
        endif()
    endforeach()
    file(GLOB_RECURSE opengl_entries LIST_DIRECTORIES TRUE "${LIGHTNING_OPENGL_PACKAGE}/*")
    reject_symlinks(${opengl_entries})
    file(GLOB_RECURSE opengl_images "${LIGHTNING_OPENGL_PACKAGE}/*.dll" "${LIGHTNING_OPENGL_PACKAGE}/*.exe")
    foreach(image IN LISTS opengl_images)
        # Bounded DOS/COFF header read: "PE\0\0" followed by machine 0x8664.
        file(READ "${image}" dos_lfanew HEX OFFSET 60 LIMIT 4)
        string(LENGTH "${dos_lfanew}" dos_lfanew_length)
        set(pe_signature "")
        if(dos_lfanew_length EQUAL 8)
            string(SUBSTRING "${dos_lfanew}" 0 2 b0)
            string(SUBSTRING "${dos_lfanew}" 2 2 b1)
            string(SUBSTRING "${dos_lfanew}" 4 2 b2)
            string(SUBSTRING "${dos_lfanew}" 6 2 b3)
            math(EXPR pe_offset "0x${b3}${b2}${b1}${b0}")
            if(pe_offset GREATER_EQUAL 64 AND pe_offset LESS 1048576)
                file(READ "${image}" pe_signature HEX OFFSET ${pe_offset} LIMIT 6)
            endif()
        endif()
        if(NOT pe_signature STREQUAL "504500006486")
            file(REMOVE "${workspace}/.in-progress")
            message(FATAL_ERROR "AMD64 OpenGL runtime image is not PE32+ AMD64: ${image}")
        endif()
    endforeach()
    file(COPY "${LIGHTNING_OPENGL_PACKAGE}/" DESTINATION "${staged}/LightningGL")
    string(CONCAT opengl_install_text
        "The AMD64 display installer also installs its adapter-selected native x64 Mesa OpenGL ICD and Vulkan runtime.\n"
        "It preserves the system opengl32.dll; normal 64-bit applications use the installed ICD through ReactOS.\n")
endif()
file(WRITE "${staged}/INSTALL.txt"
    "RX7900 / QEMU Lightning ReactOS development package\n\n"
    "Native architecture: ${DRIVER_ARCH}; compiler: ${DRIVER_COMPILER}.\n"
    "Run lightningnt5ctl.exe display-install in the ReactOS test VM for the virtual display.\n"
    "Run lightningnt5ctl.exe display-query to identify the active Lightning desktop.\n"
    "${opengl_install_text}"
    "For an updated OpenGL build, rerun display-install so its registered System32\n"
    "QemuLightningVulkan.dll is replaced; copying only to C:\\LightningGL does not update it.\n"
    "Run lightningnt5ctl.exe display-remove to remove its device binding.\n"
    "The command does not reboot automatically. Keep the prior VM disk for recovery.\n\n"
    "The virtual PCI device is 1234:11F1; this package does not bind a physical Radeon.\n"
    "Configure its QEMU device with scanout=on,gpu-scanout=off.\n"
    "For an offload-enabled package, also use gdi-offload=on and a GTK GL display.\n"
    "Retain BIOS VGA for FreeLoader; select Lightning as the ReactOS desktop adapter.\n"
    "rx7900ctl.exe is the separate read-only physical Radeon diagnostic installer.\n"
    "GPU GDI offload and physical Radeon initialization are still under development.\n"
    "This package was built with RX7900_LIGHTNING_GDI_OFFLOAD=${RX7900_LIGHTNING_GDI_OFFLOAD}.\n"
    "Enabling offload also requires a matching QEMU host with actual GPU capabilities.\n"
    "Older ReactOS images may need the supplied source-pinned PCI/cache corrections.\n"
    "Live cursor failure recovery additionally uses reactos-cursor-recovery.patch in win32k.\n"
    "Opt in with REACTOS_APPLY_CURSOR_RECOVERY during integration, then configure win32k\n"
    "with RX7900_FRAMEBUFFER_CURSOR_RECOVERY=ON and RX7900_FRAMEBUFFER_CURSOR_RECOVERY_SOURCE\n"
    "pointing to this project's src/reactos directory. Rebuild for the exact OS release.\n"
    "GPU glyph batching uses reactos-text-batch.patch, REACTOS_APPLY_TEXT_BATCH and\n"
    "RX7900_FRAMEBUFFER_TEXT_BATCH=ON with RX7900_FRAMEBUFFER_TEXT_BATCH_SOURCE.\n"
    "Guarded COLORONCOLOR mirrors, cosmetic-line admission and primary GdiFlush use\n"
    "reactos-stretch-guard.patch (also checked gradient capture and clipping),\n"
    "REACTOS_APPLY_STRETCH_GUARD and RX7900_FRAMEBUFFER_STRETCH_GUARD=ON with\n"
    "RX7900_FRAMEBUFFER_STRETCH_GUARD_SOURCE. Both source paths select src/reactos.\n"
    "Unpaired mirrors, stretch masks/modes and styled lines retain engine fallback.\n"
    "The gradient and SynchronizeSurface hooks require the paired current driver/host.\n"
    "Window-relative pattern alignment also requires reactos-brush-origin.patch in\n"
    "the matching OS release (REACTOS_APPLY_BRUSH_ORIGIN during source integration).\n"
    "Configure win32k with RX7900_FRAMEBUFFER_BRUSH_ORIGIN=ON and\n"
    "RX7900_FRAMEBUFFER_BRUSH_ORIGIN_SOURCE selecting this project's src/reactos.\n"
    "Do not replace an OS kernel using a binary built from a different source release.\n")
file(WRITE "${staged}/.driver-package-owned" "${owner_text}${DRIVER_ARCH}\n")
set(package "${binaries}/${DRIVER_ARCH}")
if(DEFINED DRIVER_PACKAGE_DIR)
    get_filename_component(package "${DRIVER_PACKAGE_DIR}" ABSOLUTE)
endif()
set(previous "${workspace}/previous-${DRIVER_ARCH}")
reject_symlinks("${package}" "${previous}")
if(EXISTS "${package}")
    reject_symlinks("${package}/.driver-package-owned")
    if(NOT EXISTS "${package}/.driver-package-owned")
        file(REMOVE "${workspace}/.in-progress")
        message(FATAL_ERROR "Refusing to overwrite an unowned published package")
    endif()
    file(READ "${package}/.driver-package-owned" package_owner)
    if(NOT package_owner STREQUAL "${owner_text}${DRIVER_ARCH}\n")
        message(FATAL_ERROR "Published package ownership mismatch")
    endif()
    if(EXISTS "${previous}")
        reject_symlinks("${previous}/.driver-package-owned")
        if(NOT EXISTS "${previous}/.driver-package-owned")
            message(FATAL_ERROR "Refusing to remove an unowned previous package")
        endif()
        file(READ "${previous}/.driver-package-owned" previous_owner)
        if(NOT previous_owner STREQUAL "${owner_text}${DRIVER_ARCH}\n")
            message(FATAL_ERROR "Previous package ownership mismatch")
        endif()
        file(GLOB_RECURSE previous_entries LIST_DIRECTORIES TRUE "${previous}/*")
        reject_symlinks(${previous_entries})
        file(REMOVE_RECURSE "${previous}")
    endif()
    run_checked("Preserve previous package" "${CMAKE_COMMAND}" -E rename "${package}" "${previous}")
endif()
execute_process(COMMAND "${CMAKE_COMMAND}" -E rename "${staged}" "${package}"
    RESULT_VARIABLE published)
if(NOT published STREQUAL "0")
    if(EXISTS "${previous}")
        execute_process(COMMAND "${CMAKE_COMMAND}" -E rename "${previous}" "${package}")
    endif()
    file(REMOVE "${workspace}/.in-progress")
    message(FATAL_ERROR "Publication failed; previous package restoration attempted")
endif()
file(REMOVE "${workspace}/.in-progress")
message(STATUS "Driver + installer package: ${package}")
message(STATUS "Use clean_build.sh or clean_build.bat to remove only owned intermediates")

# Module: rx7900.build.integrate
# Purpose: Add the component to an explicitly selected ReactOS build tree.
# Created: 2026-10-21
# Architecture: Own only the marked component directory and one CMake line.
cmake_minimum_required(VERSION 3.17)

if(NOT DEFINED REACTOS_SOURCE)
    message(FATAL_ERROR "Pass -DREACTOS_SOURCE=/absolute/path/to/reactos")
endif()

get_filename_component(component_root "${CMAKE_CURRENT_LIST_DIR}/.." REALPATH)
get_filename_component(reactos_root "${REACTOS_SOURCE}" REALPATH)
set(parent "${reactos_root}/drivers/base/CMakeLists.txt")
set(destination "${reactos_root}/drivers/base/rx7900")

if(NOT EXISTS "${reactos_root}/sdk/include/xdk/ntddk.template.h" OR NOT EXISTS "${parent}")
    message(FATAL_ERROR "REACTOS_SOURCE must be a ReactOS source checkout")
endif()
if(IS_SYMLINK "${destination}" OR IS_SYMLINK "${parent}")
    message(FATAL_ERROR "Refusing symlink integration targets")
endif()
if(EXISTS "${destination}" AND NOT EXISTS "${destination}/.rx7900-owned")
    message(FATAL_ERROR "Refusing to overwrite an unowned rx7900 directory")
endif()

# A layered patch rewrites the context of the patch beneath it, so that lower
# patch no longer reverse-applies once the layer is present; the applied layer
# proves the lower patch is applied too.
set(rx7900_layers_reactos-text-batch.patch reactos-radeon-text-bridge.patch)

function(apply_checked_patch patch_name description)
    find_program(rx7900_git git REQUIRED)
    set(patch_path "${component_root}/patches/${patch_name}")
    execute_process(COMMAND "${rx7900_git}" apply --reverse --check "${patch_path}"
        WORKING_DIRECTORY "${reactos_root}" RESULT_VARIABLE already_applied
        OUTPUT_QUIET ERROR_QUIET)
    foreach(layer IN LISTS rx7900_layers_${patch_name})
        if(NOT already_applied EQUAL 0)
            execute_process(COMMAND "${rx7900_git}" apply --reverse --check
                "${component_root}/patches/${layer}"
                WORKING_DIRECTORY "${reactos_root}" RESULT_VARIABLE already_applied
                OUTPUT_QUIET ERROR_QUIET)
        endif()
    endforeach()
    if(NOT already_applied EQUAL 0)
        execute_process(COMMAND "${rx7900_git}" apply --check "${patch_path}"
            WORKING_DIRECTORY "${reactos_root}" RESULT_VARIABLE can_apply)
        if(NOT can_apply EQUAL 0)
            message(FATAL_ERROR "${description} patch differs from this source; review it before integration")
        endif()
        execute_process(COMMAND "${rx7900_git}" apply "${patch_path}"
            WORKING_DIRECTORY "${reactos_root}" RESULT_VARIABLE applied)
        if(NOT applied EQUAL 0)
            message(FATAL_ERROR "Could not apply the checked ReactOS ${description} patch")
        endif()
    endif()
endfunction()

# OS changes are explicit opt-ins. Enabling the framebuffer adapter additionally
# requires its source path and RX7900_FRAMEBUFFER_SHADOW in the native build.
if(REACTOS_APPLY_PCI_FIX)
    apply_checked_patch(reactos-pci-resources.patch "PCI resource")
endif()
if(REACTOS_APPLY_FRAMEBUFFER_SHADOW)
    apply_checked_patch(reactos-framebuffer-shadow.patch "cached framebuffer")
    apply_checked_patch(reactos-framebuffer-mode-owner.patch "mapped display owner")
endif()
if(REACTOS_APPLY_TEXT_BATCH)
    apply_checked_patch(reactos-text-batch.patch "kernel text batching")
endif()
if(REACTOS_APPLY_RADEON_TEXT_BRIDGE)
    # The Radeon patch extends the source-pinned FreeType loop after the
    # existing optional text-batch declaration, and its CMake context follows
    # the cursor-recovery and stretch-guard declarations. All stay disabled.
    apply_checked_patch(reactos-text-batch.patch "kernel text batching")
    apply_checked_patch(reactos-cursor-recovery.patch "kernel cursor recovery")
    apply_checked_patch(reactos-stretch-guard.patch "kernel stretch orientation")
    apply_checked_patch(reactos-radeon-text-bridge.patch "Radeon glyph bridge")
endif()
if(REACTOS_APPLY_RADEON_BRIDGE)
    # Paired radeondisp caller bridge (styled DrvStrokePath). Its hunks apply
    # with or without the other optional patches; RX7900_RADEON_BRIDGE stays OFF.
    apply_checked_patch(reactos-radeon-bridge.patch "Radeon caller bridge")
endif()
if(REACTOS_APPLY_RADEON_DEVICE_BITMAPS)
    # Paired radeondisp device-bitmap bridge (DrvCreateDeviceBitmap, exactly-once
    # DrvDeleteDeviceBitmap, engine shadows, mode-change conversion). Its hunks
    # apply with or without the other optional patches, in either order;
    # RX7900_RADEON_DEVICE_BITMAPS stays OFF until the native build enables it.
    apply_checked_patch(reactos-radeon-devbitmap.patch "Radeon device-bitmap bridge")
endif()
if(REACTOS_APPLY_FLOODFILL_OPAQUE)
    # The generic flood-fill guard for targets without bits (an opaque display
    # primary) ships in the same patch with its own RX7900_FLOODFILL_OPAQUE
    # option (default OFF): both edit the same NtGdiExtFloodFill lines.
    apply_checked_patch(reactos-radeon-devbitmap.patch "opaque flood-fill guard")
endif()
if(REACTOS_APPLY_STRETCH_GUARD)
    # The paired GDI kernel patches share ordered optional build declarations.
    # Applying a declaration never enables its feature: every option stays OFF.
    apply_checked_patch(reactos-text-batch.patch "kernel text batching")
    apply_checked_patch(reactos-cursor-recovery.patch "kernel cursor recovery")
    apply_checked_patch(reactos-stretch-guard.patch "kernel stretch orientation")
endif()
if(REACTOS_APPLY_BRUSH_ORIGIN)
    apply_checked_patch(reactos-brush-origin.patch "device brush origin")
endif()
if(REACTOS_APPLY_CURSOR_RECOVERY)
    apply_checked_patch(reactos-cursor-recovery.patch "kernel cursor recovery")
endif()
if(REACTOS_APPLY_DRAW_ESCAPE)
    apply_checked_patch(reactos-draw-escape.patch "window-clipped DrawEscape")
endif()
if(REACTOS_APPLY_CLIP_VISIBILITY)
    apply_checked_patch(reactos-clip-visibility.patch "explicit empty clip visibility")
endif()
if(REACTOS_APPLY_RELEASE_FRAMEBUFFER_CACHE_BOOT)
    apply_checked_patch(reactos-822e864-framebuffer-cache.patch "822e864 boot cache")
endif()
if(REACTOS_APPLY_PNP_QUERY_REMOVE)
    # Device Manager disable as a runtime query-remove/remove (ntoskrnl io/pnpmgr
    # devaction.c + plugplay.c, internal/io.h, ntoskrnl/CMakeLists.txt option).
    # Applies to both pinned trees with or without every other optional patch,
    # in either order. Compiled out until the native build sets
    # RX7900_PNP_QUERY_REMOVE=ON (ntoskrnl and ntkrnlmp); default behaviour is kept.
    apply_checked_patch(reactos-pnp-query-remove.patch "PnP query-and-remove")
endif()
if(REACTOS_APPLY_ACPI_EJECT)
    # Retired 2026-10-24: its ACPI half enabled GPEs without the IRQL-safe
    # AcpiOsExecute and ran _EJ0 with no veto; its win32k half is now separate.
    message(FATAL_ERROR "REACTOS_APPLY_ACPI_EJECT is retired: use REACTOS_APPLY_ACPI_HOTPLUG "
        "and REACTOS_APPLY_WIN32K_DISPLAY_RELEASE")
endif()
if(REACTOS_APPLY_WIN32K_DISPLAY_RELEASE)
    # Opt-in release of a removed display's device-stack references so a
    # removable display miniport/filter can unload; a removed primary first
    # hands the desktop to another display (win32ss eng/device.c/.h, pdevobj.c/.h,
    # ntuser display.c/.h and desktop.c, one option in win32ss/CMakeLists.txt).
    # A videoprt display also needs REACTOS_APPLY_VIDEOPRT_REMOVE for its removal
    # notification. Applies to both pinned trees alone or with every other
    # optional patch, in either order. Compiled out until the native build sets
    # RX7900_WIN32K_DISPLAY_RELEASE=ON; default behaviour is kept.
    apply_checked_patch(reactos-win32k-display-release.patch "removed display release")
endif()
if(REACTOS_APPLY_VIDEOPRT_REMOVE)
    # Opt-in videoprt PnP removal (win32ss/drivers/videoprt dispatch.c, child.c,
    # videoprt.c, videoprt.h, one option in its CMakeLists.txt): REMOVE detaches
    # and deletes the adapter FDO, so a disabled display can restart and its
    # miniport unload. Applies to both pinned trees alone or with every other
    # optional patch, in either order. Compiled out until the native build sets
    # RX7900_VIDEOPRT_REMOVE=ON; default behaviour is kept.
    apply_checked_patch(reactos-videoprt-remove.patch "videoprt PnP removal")
endif()
if(REACTOS_APPLY_ACPI_HOTPLUG)
    # Opt-in ACPI PCI hot-unplug with PnP removal before _EJ0 (acpi.sys GPEs,
    # IRQL-safe AcpiOsExecute, Notify -> IoRequestDeviceEject/_EJ0/_OST;
    # pci.sys removal-state query and vanished-function pruning). Applies to
    # both pinned trees alone or with every other optional patch, in either
    # order. Compiled out until the native build sets RX7900_ACPI_HOTPLUG=ON.
    apply_checked_patch(reactos-acpi-hotplug.patch "ACPI PCI hot-unplug")
endif()

file(MAKE_DIRECTORY "${destination}")
file(WRITE "${destination}/.rx7900-owned" "RX7900-ReactOS build integration v1\n")
# Compare content and timestamp changed destinations at the actual copy time.
# file(COPY) preserves rounded source timestamps; a same-second edit could
# otherwise leave newer Ninja objects containing the previous implementation.
file(GLOB_RECURSE component_files LIST_DIRECTORIES FALSE RELATIVE "${component_root}"
    "${component_root}/src/*" "${component_root}/tests/*" "${component_root}/third_party/*")
list(APPEND component_files CMakeLists.txt)
foreach(component_file IN LISTS component_files)
    get_filename_component(component_directory "${destination}/${component_file}" DIRECTORY)
    file(MAKE_DIRECTORY "${component_directory}")
    configure_file("${component_root}/${component_file}" "${destination}/${component_file}" COPYONLY)
endforeach()
file(READ "${parent}" parent_source)

if(NOT parent_source MATCHES "add_subdirectory\\([ \t]*rx7900[ \t]*\\)")
    file(APPEND "${parent}" "\nadd_subdirectory(rx7900)\n")
endif()

message(STATUS "Integrated diagnostic targets: rx7900probe and rx7900ctl")

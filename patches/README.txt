Optional ReactOS patches (revision a97c8a2cdcf6a9a7f5cc543951a02ccb7a80d8ca)

None is needed to build or install the Radeon package; the released drivers
were built with source integration only. cmake/integrate.cmake applies a patch
when its REACTOS_APPLY_... option is ON. Every feature a patch adds is compiled
out until the matching RX7900_... build option is also set.

Original files (shipped in the driver package since 2026-10-24)
  reactos-cursor-recovery.patch       REACTOS_APPLY_CURSOR_RECOVERY
  reactos-draw-escape.patch           REACTOS_APPLY_DRAW_ESCAPE
  reactos-radeon-bridge.patch         REACTOS_APPLY_RADEON_BRIDGE
  reactos-radeon-text-bridge.patch    REACTOS_APPLY_RADEON_TEXT_BRIDGE
  reactos-stretch-guard.patch         REACTOS_APPLY_STRETCH_GUARD
  reactos-text-batch.patch            REACTOS_APPLY_TEXT_BATCH

Reconstructed on 2026-11-15
  reactos-pci-resources.patch         REACTOS_APPLY_PCI_FIX
  reactos-framebuffer-shadow.patch    REACTOS_APPLY_FRAMEBUFFER_SHADOW
  reactos-brush-origin.patch          REACTOS_APPLY_BRUSH_ORIGIN
  reactos-clip-visibility.patch       REACTOS_APPLY_CLIP_VISIBILITY
  The original files were lost. These were rebuilt by comparing the ReactOS
  tree the drivers were built in with pristine a97c8a2, removing the four
  original patches that tree had applied (text-batch, cursor-recovery,
  stretch-guard, radeon-text-bridge) and splitting the rest by the build
  option each change is guarded by. reactos-framebuffer-shadow.patch also
  carries what used to be reactos-framebuffer-mode-owner.patch; the two could
  not be told apart. Checked: each applies to pristine a97c8a2 alone and after
  the four original patches, and integrate.cmake with PCI_FIX,
  FRAMEBUFFER_SHADOW, RADEON_TEXT_BRIDGE, BRUSH_ORIGIN and CLIP_VISIBILITY on a
  pristine tree reproduces the as-built tree file for file. They are therefore
  equivalent in effect to the lost files, not necessarily identical in text.
  No ReactOS build with these options enabled was made for this publication.

Not recoverable (never applied to the as-built tree)
  reactos-radeon-devbitmap.patch, reactos-822e864-framebuffer-cache.patch,
  reactos-pnp-query-remove.patch, reactos-videoprt-remove.patch,
  reactos-win32k-display-release.patch, reactos-acpi-hotplug.patch.

radeonctl-board-fixes.patch in the driver package is not a ReactOS patch: it
records the installer changes made after the drivers were built, and those
changes are already in this source tree.

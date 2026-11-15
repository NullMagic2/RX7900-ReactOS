/*
Module: lightning.reactos.display_gate
Purpose: Keep foreign display initialization and hardware I/O inside owned D0.
Created: 2026-10-21
Architecture: Native caller holds its lifecycle mutex through lower completion.
*/
#pragma once
#include "lightning_present_abi.hpp"

namespace lightning::reactos {

enum class DisplayRequest { open, inventory, retirement, hardware };
enum class DisplayAdmission { allow, access_denied, not_ready };

// Identity is latched only after the exact lower display ABI succeeds. Inventory
// is pointer-free cached metadata; retirement is kernel-only RAM UNMAP/ABANDON.
// Before recognition no lower control request is assumed safe while inactive.
constexpr DisplayAdmission display_admission(DisplayRequest request, bool recognized,
    bool kernel, bool active) noexcept {
    if (request == DisplayRequest::open) {
        return active ? DisplayAdmission::allow : DisplayAdmission::not_ready;
    }

    if (recognized && !kernel && request != DisplayRequest::inventory) {
        return DisplayAdmission::access_denied;
    }

    if (active || (recognized && request != DisplayRequest::hardware)) {
        return DisplayAdmission::allow;
    }

    return DisplayAdmission::not_ready;
}

// After REMOVE of a recognized display the detached filter is retained only
// as a retirement channel: win32k still holds its top-of-stack pointer, and
// the display client must still return leased staging RAM. Only kernel
// pointer-free staging retirement passes; every other request is refused.
// unmap_code is the SDK IOCTL_VIDEO_UNMAP_VIDEO_MEMORY used by older clients.
constexpr bool retained_retirement_admitted(uint32_t code, bool kernel, uint32_t unmap_code) noexcept {
    return kernel && (code == ioctl_release_owned_staging || code == ioctl_resolve_owned_staging ||
        code == unmap_code);
}

} // namespace lightning::reactos

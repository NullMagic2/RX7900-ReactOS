/*
Module: lightning.reactos.display
Purpose: Connect the reused cached GDI owner to bounded Lightning presentation.
Created: 2026-10-21
Architecture: Included after framebuf.h; no rasterizer or PCI ownership here.
*/
#pragma once
#include "lightning_present_abi.hpp"
#include "framebuffer_shadow_policy.hpp"
#include "lightning_cursor_snapshot.hpp"
#include "lightning_staging_client.hpp"
#include "lightning_staging_resolve.hpp"
#include "lightning-display.h"

namespace lightning::reactos {

inline bool valid_display_owner_information(const CpuScanoutInformation& information) noexcept {
    return information.size==sizeof(information) && information.version==cpu_scanout_abi_version &&
        information.magic==LIGHTNING_DISPLAY_MAGIC && information.capabilities==LIGHTNING_DISPLAY_CAPABILITIES &&
        information.enabled<=1;
}

// Read-only owner inventory remains available from cached native metadata in
// D3. Both display and GDI clients use the same checked identity boundary.
inline bool query_display_owner(HANDLE device,CpuScanoutInformation& information) noexcept {
    CpuScanoutInformation candidate{};
    DWORD returned=0;
    if (device==nullptr || EngDeviceIoControl(device,ioctl_query_cpu_scanout,nullptr,0,
        &candidate,sizeof(candidate),&returned)!=0 || returned!=sizeof(candidate) ||
        !valid_display_owner_information(candidate)) return false;
    information=candidate;
    return true;
}

// Authentication is one synchronous native comparison of the returned MAP
// base to the current leased allocation. No arbitrary pointer is dereferenced.
inline bool query_owned_staging(HANDLE device,void* address,CpuScanoutInformation& information) noexcept {
    VIDEO_MEMORY mapping{address};
    CpuScanoutInformation candidate{};
    DWORD returned=0;
    if (device==nullptr || address==nullptr ||
        EngDeviceIoControl(device,ioctl_query_owned_staging,&mapping,sizeof(mapping),
            &candidate,sizeof(candidate),&returned)!=0 || returned!=sizeof(candidate) ||
        !valid_display_owner_information(candidate)) return false;
    information=candidate;
    return true;
}

// Caller holds the cached surface registry guard. The native device handle is
// borrowed from PDEV; requests are synchronous, bounded and pointer-free.
class DisplayLink final {
public:
    // Initial binding only. Subsequent transitions retain an exact credential;
    // a read-only query must never adopt a replacement PDEV's generation.
    // Initial connection authenticates ScreenPtr; remap supplies its not-yet-
    // published candidate so the old engine wrapper survives query failure.
    bool connect(PPDEV device,void* candidate_staging=nullptr) noexcept;
    bool current_owner(bool require_enabled=true) noexcept;
    bool transition_mode(uint32_t mode_index,bool enable) noexcept;
    bool present(rx7900::reactos::framebuffer::Rectangle damage) noexcept;
    bool query(CpuScanoutInformation& information) noexcept;
    bool map_staging(void*& address, uint32_t& bytes) noexcept;
    bool unmap_staging(void* address) noexcept;
    bool define_cursor(SURFOBJ* mask, SURFOBJ* color, XLATEOBJ* translation,
        LONG hot_x, LONG hot_y, LONG x, LONG y, FLONG flags) noexcept;
    bool move_cursor(LONG x, LONG y) noexcept;
    bool hide_cursor() noexcept;
    bool suspend_cursor() noexcept;
    bool restore_cursor() noexcept;
    // DrvAssertMode(TRUE) after this owner's own native mode commit, which
    // already hid and undefined the native plane: a retained shape that cannot
    // be replayed must not fail the enable. Hide best-effort and request the
    // engine's reselect (query_cursor recovery_required); never report the
    // old plane as visible.
    void abandon_cursor_restore() noexcept;
    uint32_t mode_index() const noexcept { return mode_index_; }
    uint32_t mode_generation() const noexcept { return mode_generation_; }
    // Win32 error of the newest failed transition, owned MAP or present
    // (a failed native transition reports its VP_STATUS); 0 after success.
    uint32_t last_error() const noexcept { return last_error_; }
    bool recover_failed_cursor_move() noexcept;
    bool query_cursor(CursorStatistics& statistics) noexcept;
    void retire() noexcept;
    bool hardware_cursor() const noexcept { return hardware_cursor_; }
    // Authoritative surprise-removal containment report from the native
    // owner. Read-only; false when the owner cannot answer (legacy miniport).
    bool device_lost() noexcept;
    // Caller holds the registry guard and has already moved every engine
    // wrapper off leased RAM. Stops native publication and retires every
    // owned/candidate/pending credential by generation; false keeps the
    // unconfirmed tokens for a later bounded retry.
    bool transfer_lost_leases() noexcept;
    bool retirement_pending() const noexcept { return !staging_leases_.empty(); }
private:
    static bool release_staging_lease(void* context,uint32_t generation) noexcept;
    bool resolve_lease(uint32_t generation,uint32_t operation,StagingResolveReceipt& receipt) noexcept;
    bool recover_unacknowledged_map() noexcept;
    bool drain_pending() noexcept;
    HANDLE driver_ = nullptr;
    StagingClientLeases staging_leases_{};
    uint32_t mode_generation_ = 0;
    uint32_t mode_index_ = 0;
    uint32_t lease_generation_ = 0;
    LONG hot_x_ = 0;
    LONG hot_y_ = 0;
    bool hardware_cursor_ = false;
    bool recovery_required_ = false;
    // Set by the first valid resolve receipt. Afterwards a failed resolve is a
    // lost or refused request and is retried; legacy release is never mixed in.
    bool resolve_supported_ = false;
    bool lost_ = false;
    // Exact next generation of an owned MAP whose acknowledgement failed.
    uint32_t unacknowledged_map_ = 0;
    uint32_t last_error_ = 0;
    CursorShapeSnapshot cursor_{};
};

} // namespace lightning::reactos

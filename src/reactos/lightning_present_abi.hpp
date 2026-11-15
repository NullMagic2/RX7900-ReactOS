/*
Module: lightning.reactos.present_abi
Purpose: Share bounded framebuf-to-VideoPort CPU presentation requests.
Created: 2026-10-21
Architecture: No addresses or register offsets cross this ABI.
*/
#pragma once
#include <stdint.h>

namespace lightning::reactos {
// CTL_CODE(FILE_DEVICE_VIDEO, 0x900/0x901, METHOD_BUFFERED, FILE_ANY_ACCESS).
constexpr uint32_t ioctl_query_cpu_scanout=0x00232400U;
constexpr uint32_t ioctl_present_cpu_scanout=0x00232404U;
constexpr uint32_t ioctl_quiesce_cpu_scanout=0x00232408U;
constexpr uint32_t ioctl_resume_cpu_scanout=0x0023240cU;
constexpr uint32_t ioctl_wait_cpu_scanout=0x00232410U;
constexpr uint32_t ioctl_cursor_shape=0x00232414U;
constexpr uint32_t ioctl_cursor_move=0x00232418U;
// Surprise-removal containment: no hardware accesses, waits or implicit frees.
constexpr uint32_t ioctl_abandon_cpu_scanout=0x0023241cU;
constexpr uint32_t cpu_scanout_abi_version=1;
constexpr uint32_t query_display_escape=0x4c445351U;
// DDI-only transitions authenticate the mapped PDEV before changing a global
// mode. Transport power/PnP retains the existing owner-independent requests.
constexpr uint32_t ioctl_transition_owned_mode=0x00232434U;
constexpr uint32_t ioctl_map_owned_staging=0x00232438U;
// Kernel display handoff only: a standard VIDEO_MEMORY input is compared to
// the current leased allocation, never dereferenced. Returns the existing
// pointer-free CpuScanoutInformation only for that exact mapped owner.
constexpr uint32_t ioctl_query_owned_staging=0x0023243cU;
constexpr uint32_t ioctl_release_owned_staging=0x00232440U;
struct StagingReleaseRequest final {
    uint32_t size;
    uint32_t version;
    uint32_t lease_generation;
    uint32_t reserved;
};
static_assert(sizeof(StagingReleaseRequest)==16,"Pointer-free staging retirement credential");

// Idempotent release-or-confirm by nonwrapping lease generation, additive to
// ABI 1. Each generation names at most one RAM lease and is never reissued,
// so a retry after a lost or ambiguous acknowledgement retires it exactly
// once and can never reach a recycled address's newer lease. A miniport that
// predates this code fails it; its client keeps the bounded legacy release
// retry. Kernel display callers only; no address or pointer crosses it.
constexpr uint32_t ioctl_resolve_owned_staging=0x00232444U;
constexpr uint32_t staging_resolve_release=1;
constexpr uint32_t staging_resolve_query=2;
// Outcomes. released: this request retired the lease. not_held: the
// generation was issued earlier and is no longer leased, final. held: query
// only, the lease exists now. unknown: never issued by this adapter (zero or
// above the issue watermark); a client must drop, never retry, that token.
constexpr uint32_t staging_lease_released=1;
constexpr uint32_t staging_lease_not_held=2;
constexpr uint32_t staging_lease_held=3;
constexpr uint32_t staging_lease_unknown=4;
// device_lost: surprise-removal containment is active; native scanout no
// longer reads staging and every lease awaits only its wrapper retirement.
// initiator: a held lease was issued by an owned MAP whose initiating lease
// equals the request's initiator_lease (lost MAP acknowledgement recovery).
constexpr uint32_t staging_flag_device_lost=1U;
constexpr uint32_t staging_flag_initiator=2U;
struct StagingResolveRequest final {
    uint32_t size;
    uint32_t version;
    uint32_t lease_generation;
    uint32_t operation;
    uint32_t initiator_lease;
};
struct StagingResolveReceipt final {
    uint32_t size;
    uint32_t version;
    uint32_t lease_generation;
    uint32_t outcome;
    uint32_t flags;
    uint32_t issued_generation;
    uint32_t held_leases;
    uint32_t reserved;
};
static_assert(sizeof(StagingResolveRequest)==20,"Pointer-free staging resolve request");
static_assert(sizeof(StagingResolveReceipt)==32,"Pointer-free staging resolve receipt");
struct ModeOwnerRequest final {
    uint32_t size;
    uint32_t version;
    uint32_t mode_generation;
    uint32_t mode_index;
    uint32_t lease_generation;
};
struct ModeTransitionRequest final {
    ModeOwnerRequest owner;
    uint32_t enable;
    uint32_t reserved;
};
// A valid request receives an acknowledgement even when mode programming or
// publication fails. Only that initiating owner receives its advanced token;
// it can retry without adopting a different PDEV through read-only inventory.
struct ModeTransitionReceipt final {
    uint32_t size;
    uint32_t version;
    uint32_t requested_generation;
    uint32_t mode_generation;
    uint32_t mode_index;
    uint32_t result;
    uint32_t enabled;
    uint32_t reserved;
};
static_assert(sizeof(ModeOwnerRequest)==20,"Fixed owner credential");
static_assert(sizeof(ModeTransitionRequest)==28,"Bounded mode transition");
static_assert(sizeof(ModeTransitionReceipt)==32,"Pointer-free transition receipt");

// Read-only cursor inventory. Coordinates/moves describe acknowledged accepted
// motion; visibility changes require actual main-loop plane publication.
// hardware_active is the querying DDI owner's accepted plane lease. Host UI
// publication counters independently observe coalesced cursor movement.
constexpr uint32_t ioctl_query_cursor=0x00232430U;
constexpr uint32_t query_cursor_escape=0x4c435351U;
constexpr uint32_t cursor_statistics_magic=0x5352434cU;
struct CursorStatistics final {
    uint32_t size;
    uint32_t version;
    uint32_t magic;
    uint32_t mode_generation;
    uint32_t defined;
    uint32_t visible;
    uint32_t format;
    uint32_t width;
    uint32_t height;
    uint32_t hot_x;
    uint32_t hot_y;
    int32_t x;
    int32_t y;
    uint32_t hardware_active;
    uint64_t definitions;
    uint64_t moves;
    uint64_t hides;
    uint64_t declines;
    uint64_t failures;
    // DDI-only request to reselect the original engine cursor after a failed
    // live move whose host plane is confirmed hidden. Never a normal hide.
    uint32_t recovery_required;
    uint32_t reserved;
};
static_assert(sizeof(CursorStatistics)==104,"Fixed pointer-free cursor statistics ABI");
struct CpuScanoutInformation final {
    uint32_t size;
    uint32_t version;
    uint32_t magic;
    uint32_t width;
    uint32_t height;
    uint32_t stride;
    uint32_t mapped_bytes;
    uint32_t capabilities;
    uint32_t mode_generation;
    uint32_t refresh_ns;
    uint32_t accepted_epoch;
    uint32_t accepted_sequence;
    uint32_t completed_epoch;
    uint32_t completed_sequence;
    uint32_t enabled;
};
static_assert(sizeof(CpuScanoutInformation)==60,"Fixed private display ABI");

struct DamageRequest final {
    uint32_t size;
    uint32_t version;
    uint32_t mode_generation;
    uint32_t rectangle_count;
    // Exactly rectangle_count canonical LightningDisplayRect values follow.
};
static_assert(sizeof(DamageRequest)==16,"Bounded present header");

struct CursorShapeRequest final {
    uint32_t size;
    uint32_t version;
    uint32_t mode_generation;
    uint32_t width;
    uint32_t height;
    uint32_t pitch;
    uint32_t format;
    uint32_t hot_x;
    uint32_t hot_y;
    int32_t x;
    int32_t y;
    uint32_t visible;
    uint32_t shape_bytes;
    // Exactly shape_bytes bytes of canonical cursor format follow.
};
static_assert(sizeof(CursorShapeRequest)==52,"Bounded cursor header");

struct CursorMoveRequest final {
    uint32_t size;
    uint32_t version;
    uint32_t mode_generation;
    int32_t x;
    int32_t y;
    uint32_t visible;
};
static_assert(sizeof(CursorMoveRequest)==24,"Fixed cursor move ABI");

// Read-only DrvAssertMode(TRUE) health of the display driver (escape only; no
// IOCTL, no native state). win32k repeats a failed AssertMode(TRUE) forever
// under its global PDEV lock (pdevobj.c PDEVOBJ_vEnableDisplay), so the display
// driver never returns a final FALSE there: it degrades the optional GPU canvas
// and hardware cursor, and after enable_attempt_limit consecutive mandatory
// failures it parks the PDEV (TRUE to the engine, nothing published). Counters
// and flags 1/2/4 are driver-wide since load and survive PDEV replacement (each
// mode change creates a new PDEV); parked_now describes the querying PDEV.
constexpr uint32_t query_mode_health_escape=0x4c444d48U;
constexpr uint32_t mode_health_magic=0x484d444cU;
constexpr uint32_t mode_health_version=1;
constexpr uint32_t mode_health_gpu_demoted=1U;
constexpr uint32_t mode_health_cursor_dropped=2U;
constexpr uint32_t mode_health_parked=4U;
constexpr uint32_t mode_health_parked_now=8U;
// Mandatory steps of one enable attempt, in order. last_failed_step names the
// first failed step of the newest failed attempt (none: no attempt failed).
enum class EnableStep : uint32_t { none=0, suspend=1, transition=2, remap=3, flush=4 };
constexpr uint32_t enable_attempt_limit=3;
struct DisplayModeHealth final {
    uint32_t size;
    uint32_t version;
    uint32_t magic;
    uint32_t flags;
    uint32_t enables;
    uint32_t failed_attempts;
    uint32_t consecutive_failures;
    uint32_t last_failed_step;
    uint32_t mode_generation;
    // Win32 error of the native request that failed the newest failed
    // attempt (transition, owned MAP or present); 0 when that step failed
    // locally. ERROR_NOT_ENOUGH_MEMORY (8) at remap: staging allocation.
    uint32_t last_error;
};
static_assert(sizeof(DisplayModeHealth)==40,"Fixed pointer-free mode health ABI");

// WAIT has no input and waits for the accepted prefix with a fixed native
// timeout. QUIESCE drains and publishes a blank before returning; RESUME keeps
// the same assigned resources and performs an owned full mode commit. These
// lifecycle requests are also used by the owning transport UpperFilter before
// forwarding native STOP/REMOVE/power transitions. They publish no DMA handles.
} // namespace lightning::reactos

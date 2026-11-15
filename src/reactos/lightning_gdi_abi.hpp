/*
Module: lightning.reactos.gdi_abi
Purpose: Share pointer-free GDI requests between the DDI and its native owner.
Created: 2026-10-21
Architecture: Canonical host command reused verbatim; native owns sequence,
epoch, transfer-window addresses and staged RAM leases. Kernel callers only.
*/
#pragma once

#include "lightning-gdi.h"

namespace lightning::reactos::gdi {

constexpr uint32_t ioctl_query = 0x00232420U;
constexpr uint32_t ioctl_submit = 0x00232424U;
constexpr uint32_t ioctl_wait = 0x00232428U;
constexpr uint32_t ioctl_restore = 0x0023242cU;
constexpr uint32_t query_statistics_escape = 0x4c474451U;
constexpr uint32_t native_abi_version = 1;
constexpr uint32_t statistics_abi_version = 3;

// Optional input to the existing read-only query. An older miniport ignores
// this input and returns its full Information, preserving compatibility.
constexpr uint32_t query_coherence_only = 1;
struct QueryRequest final {
    uint32_t size;
    uint32_t version;
    uint32_t flags;
    uint32_t reserved;
};
static_assert(sizeof(QueryRequest)==16,"Bounded optional native query request");

struct Information final {
    uint32_t size;
    uint32_t version;
    uint32_t magic;
    uint32_t capabilities;
    uint32_t active;
    uint32_t width;
    uint32_t height;
    uint32_t mode_generation;
    uint32_t epoch;
    uint32_t accepted_sequence;
    uint32_t completed_sequence;
    uint32_t canvas_revision;
    uint32_t quarantined;
    uint32_t reset_generation;
    uint32_t published_sequence;
    uint32_t published_epoch;
    uint32_t published_mode_generation;
};
static_assert(sizeof(Information) == 68, "Fixed native GDI query ABI");
static_assert(sizeof(LightningGdiCommand) == LIGHTNING_GDI_COMMAND_BYTES,
              "Canonical GPU command layout");

struct SubmitRequest final {
    uint32_t size;
    uint32_t version;
    uint32_t stage_source_x;
    uint32_t stage_source_y;
    LightningGdiCommand command;
    // RASTER appends canonical RasterOperands, then all commands append exactly
    // command.clip_count canonical LightningDisplayRect values.
    // Epoch/sequence are zero on input; native fills its own ordered values.
    // Upload source coordinates refer only to the active mapped staging bitmap.
};
static_assert(sizeof(SubmitRequest) == 112, "Fixed bounded native submit header");

constexpr uint32_t declined = 0;
constexpr uint32_t accepted = 1;
constexpr uint32_t uncertain = 2;

// Native checks the complete output extent before touching the GPU. Every valid
// request returns this receipt even on host rejection/uncertain execution, so a
// false I/O completion can never be confused with a safe software replay.
struct Receipt final {
    uint32_t size;
    uint32_t version;
    uint32_t outcome;
    uint32_t result;
    uint32_t mode_generation;
    uint32_t epoch;
    uint32_t accepted_sequence;
    uint32_t completed_sequence;
    uint32_t canvas_revision;
};
static_assert(sizeof(Receipt) == 36, "Fixed native acceptance receipt");

struct WaitRequest final {
    uint32_t size;
    uint32_t version;
    uint32_t mode_generation;
    uint32_t sequence;
};
static_assert(sizeof(WaitRequest) == 16, "Bounded observed GPU fence request");

// Asynchronous external image (async-present, additive). A SubmitRequest with
// this version is legal only for LIGHTNING_GDI_EXTERNAL_IMAGE, only from the
// kernel display client, and only after the receipt probe below reported
// external_feature_async. On input its descriptor carries the optional
// producer sync point: context is the signaled value, resource the renderer
// sync token (zero for none). The upper filter replaces both with the owned
// image identity, keeps version 2 only while it can pin the lease until the
// read retires, and otherwise rewrites version 1 (synchronous). The miniport
// then returns after host acceptance, before completion; the receipt reports
// completed_sequence below accepted_sequence while the read is outstanding.
// No version-2 request ever reaches QEMU; the host command is unchanged.
constexpr uint32_t native_async_abi_version = 2;
// Optional ioctl_query flag, combined with query_coherence_only: a miniport
// that accepts it honors version-2 external images. Older miniports reject
// unknown flags, so the filter keeps every external image synchronous.
constexpr uint32_t query_async_external = 2;

// Kernel-only. The LightningNT5 upper filter answers it itself and never
// forwards it; an older stack fails it, which disables asynchronous presents.
// lease zero is the capability probe. Otherwise the answer covers the newest
// accepted copy of that lease of the calling process: retired also covers a
// lease without an outstanding asynchronous copy. The request never waits for
// GPU execution; failed means the read can no longer be proven retired.
constexpr uint32_t ioctl_external_receipt = 0x00232448U;
constexpr uint32_t external_receipt_failed = 0;
constexpr uint32_t external_receipt_retired = 1;
constexpr uint32_t external_receipt_pending = 2;
constexpr uint32_t external_feature_async = 1;
struct ExternalReceiptRequest final {
    uint32_t size;
    uint32_t version;
    uint64_t lease;
};
struct ExternalReceipt final {
    uint32_t size;
    uint32_t version;
    uint32_t status;
    uint32_t features;
};
static_assert(sizeof(ExternalReceiptRequest) == 16, "Pointer-free external receipt query");
static_assert(sizeof(ExternalReceipt) == 16, "Pointer-free external receipt");

// Copy only the miniport-owned, fully fenced pre-suspend primary snapshot into
// the active retained staging lease. No pointer, arbitrary offset or GPU address.
struct RestoreRequest final {
    uint32_t size;
    uint32_t version;
    uint32_t mode_generation;
    uint32_t reserved;
};
static_assert(sizeof(RestoreRequest)==16,"Bounded retained-primary restore request");

struct Statistics final {
    uint32_t size=sizeof(Statistics);
    uint32_t version=statistics_abi_version;
    uint32_t enabled=0;
    uint32_t poisoned=0;
    Information device{};
    uint32_t reserved=0;
    uint64_t accepted_fills=0;
    uint64_t accepted_copies=0;
    uint64_t accepted_uploads=0;
    uint64_t accepted_blends=0;
    uint64_t readbacks=0;
    uint64_t cpu_fallbacks=0;
    uint64_t accepted_rasters=0;
    uint64_t accepted_glyphs=0;
};
static_assert(sizeof(Statistics)==152,"Fixed read-only GPU hook statistics");

} // namespace lightning::reactos::gdi

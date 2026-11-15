/*
Module: lightning.reactos.gdi_native
Purpose: Serialize bounded GPU commands, transfer leases and real fence readback.
Created: 2026-10-21
Architecture: VideoPort owns the mappings and holds its PASSIVE_LEVEL mutex.
*/
#pragma once

#include "lightning_gdi_abi.hpp"
#include "gdi_protocol.hpp"
#include "byte_view.hpp"

namespace lightning::reactos::gdi {

// All pointers are borrowed kernel mappings owned by the calling miniport.
// No address is supplied by userspace or retained beyond a synchronous method.
struct NativeView final {
    void* context = nullptr;
    bool (*read)(void*,uint32_t,uint32_t&) noexcept = nullptr;
    bool (*write)(void*,uint32_t,uint32_t) noexcept = nullptr;
    void* transfer = nullptr;
    uint32_t transfer_bytes = 0;
    void* staging = nullptr;
    uint32_t staging_bytes = 0;
    uint32_t stride = 0;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t mode_generation = 0;
    // Optional connected completion IRQ. prepare clears the device-owned event;
    // the caller then rechecks the authoritative ledger before sleeping. wait
    // only reports kernel-wait validity, never GPU completion. Both callbacks
    // run at PASSIVE_LEVEL with the same miniport mutex as the mapped view.
    bool (*prepare_completion)(void*) noexcept = nullptr;
    bool (*wait_completion)(void*,uint64_t) noexcept = nullptr;
};

class NativeCanvas final {
public:
    // Coherence queries retain live capabilities, reset and exact GPU-ledger
    // checks. Publication fields are zero; callers needing them use full query.
    bool query(const NativeView& view,Information& output,bool coherence_only=false) noexcept;
    void submit(const NativeView& view,rx7900::ByteView input,Receipt& output) noexcept;
    void wait(const NativeView& view,const WaitRequest& request,Receipt& output) noexcept;
    bool drain(const NativeView& view) noexcept;
    // Snapshot an authoritative complete GPU canvas into a caller-owned bounded
    // destination (the miniport's private primary, never a leased upload buffer).
    bool snapshot(const NativeView& view,void* destination,uint32_t bytes) noexcept;
    bool has_canvas() const noexcept { return canvas_.accepted_sequence()!=0; }
    // Only actual backend retirement permits transfer-window reuse/reset.
    bool reset(const NativeView& view) noexcept;
    // Surprise-removal notification cannot retire GPU access or readback writes.
    void abandon() noexcept;
    bool quarantined() const noexcept;
private:
    bool refresh(const NativeView& view) noexcept;
    bool wait_sequence(const NativeView& view,uint32_t sequence) noexcept;
    bool copy_upload(const NativeView& view,const SubmitRequest& request) noexcept;
    bool copy_readback(const NativeView& view,LightningDisplayRect rectangle) noexcept;
    void receipt(const NativeView& view,uint32_t outcome,uint32_t result,Receipt& output) const noexcept;
    void poison() noexcept;
    // The caller's miniport mutex serializes reuse. Keep the bounded wire
    // snapshot device-owned rather than adding a large NT kernel stack frame.
    uint8_t metadata_snapshot_[LIGHTNING_GDI_MAX_METADATA_BYTES]{};
    ::lightning::gdi::Command command_snapshot_{};
    ::lightning::gdi::CanvasState bind_candidate_{};
    ::lightning::gdi::CanvasState canvas_{};
    ::lightning::gdi::TransferLease transfer_{};
    uint32_t readback_sequence_ = 0;
    uint32_t ownership_reset_generation_ = 0;
    // Newest canvas epoch this device bound. A reset clears the host epoch, so a
    // new BIND starts above both; receipts of an older canvas keep an epoch that
    // can no longer recur (lightning_image_receipt.hpp decide_receipt).
    uint32_t epoch_floor_ = 0;
    // Host capabilities of the last query (meaningful while the contract is
    // validated) and the newest accepted latest-wins copy the host may hold.
    uint32_t capabilities_ = 0;
    uint32_t replaceable_sequence_ = 0;
    bool contract_validated_ = false;
};

} // namespace lightning::reactos::gdi

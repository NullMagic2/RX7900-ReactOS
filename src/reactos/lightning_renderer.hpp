/*
Module: lightning.reactos.renderer
Purpose: Own native per-file Venus sessions, assigned aperture views and GPU leases.
Created: 2026-10-21
Architecture: Device-mutex serialized PASSIVE_LEVEL; mandatory process-exit tracking.
*/
#pragma once

#include "lightning_transport.hpp"
#include "lightning_image_receipt.hpp"

namespace lightning::reactos {

struct NativeImageLease final {
    uint64_t context = 0;
    uint64_t resource = 0;
    ImageLayout layout{};
};

// Installed by the LightningNT5 upper filter, which alone can observe the
// lower GDI canvas. resolve runs under the same FDO guard as every renderer
// call: wait=false only observes progress; wait=true may block for the bounded
// native GDI completion wait. async reports that the lower miniport honors
// asynchronous external images; without it no receipt is ever tracked.
// lightning-pnp-veto: CPU borrows only their owner can end. sessions counts
// renderer sessions of processes not exiting that still map aperture views
// (ring and blob mappings); views counts those mappings. shutdown() contains
// the device exactly when sessions is nonzero, so the filter refuses an
// orderly STOP/REMOVE query for it instead.
struct BorrowCensus final {
    uint32_t sessions = 0;
    uint32_t views = 0;
};

struct ReceiptOwner final {
    void* context = nullptr;
    ReceiptState (*resolve)(void* context, const ImageReceipt& receipt, bool wait) noexcept = nullptr;
    bool async = false;
};

class NativeRenderer final {
public:
    NativeRenderer() noexcept = default;
    NativeRenderer(const NativeRenderer&) = delete;
    NativeRenderer& operator=(const NativeRenderer&) = delete;

    // Driver registers process-exit notification before exposing this device.
    NTSTATUS initialize(NativeTransport& transport) noexcept;
    NTSTATUS open_file(PFILE_OBJECT file) noexcept;
    NTSTATUS close_file(PFILE_OBJECT file) noexcept;
    NTSTATUS exchange_file(PFILE_OBJECT file, PEPROCESS requestor, rx7900::ByteView input,
                          rx7900::sdma::CommandBuffer output, size_t& written) noexcept;
    // ioctl_exchange_batch (lightning_abi.hpp): each packet goes through
    // exchange_file in order while the transport shares one doorbell among
    // the posted entries. written counts the replies' bytes.
    NTSTATUS exchange_batch(PFILE_OBJECT file, PEPROCESS requestor, rx7900::ByteView input,
                           rx7900::sdma::CommandBuffer output, size_t& written) noexcept;
    // Kernel display callers only, under the same FDO guard as exchange_file.
    // Keep that guard through synchronous lower GPU consumption so CLOSE,
    // process exit, reset and resource destruction cannot revoke this lease.
    // A usable producer point (a recorded signal of this session whose private
    // timeline is host-published) limits the wait to that submission; without
    // one the image's conservative stamp is drained as before.
    NTSTATUS resolve_image(uint64_t lease, PEPROCESS process,
                           NativeImageLease& output, const ProducerPoint* producer = nullptr) noexcept;
    // Same guard. The upper filter installs its lower-canvas observer once.
    void set_receipt_owner(const ReceiptOwner& owner) noexcept { receipt_owner_ = owner; }
    bool receipt_owner_installed() const noexcept { return receipt_owner_.resolve != nullptr; }
    bool async_receipts() const noexcept { return receipt_owner_.resolve != nullptr && receipt_owner_.async; }
    // Same guard, after a resolved image's asynchronous copy was accepted: pin
    // the lease until the read retires. Destruction, registration, CLOSE,
    // process exit and recovery first settle it. False: lease no longer owned.
    bool track_receipt(uint64_t lease, PEPROCESS process, const ImageReceipt& receipt) noexcept;
    // Same guard, before forwarding: an asynchronous copy of this lease could be pinned.
    bool can_track_receipt(uint64_t lease, PEPROCESS process) const noexcept;
    // Same guard; never waits. Unknown or unpinned leases report retired.
    ReceiptState query_receipt(uint64_t lease, PEPROCESS process) noexcept;
    // A confirmed native display quiesce/reset retired every accepted read.
    void retire_receipts_after_display_reset() noexcept;
    // Called synchronously before process address-space teardown, under device
    // guard and remove lock. Never attach to a process after its exit callback.
    void process_exit(HANDLE process_id) noexcept;
    NTSTATUS shutdown(bool allow_host = true) noexcept;
    // Same guard; read-only. What shutdown() would find busy right now.
    BorrowCensus borrow_census() const noexcept;
    bool ready() const noexcept { return active_ && !quarantined_; }
    bool quarantined() const noexcept { return quarantined_; }

private:
    struct FileSession;
    FileSession* session(PFILE_OBJECT file) const noexcept;
    static bool lookup_public(void*, uint64_t, ResourceInfo&) noexcept;
    static bool lookup_all(void*, uint64_t, ResourceInfo&) noexcept;
    NTSTATUS call(FileSession&, QLV_HEADER, ExchangeReply&, uint64_t sync = 0, uint64_t point = 0) noexcept;
    NTSTATUS poll(FileSession&) noexcept;
    NTSTATUS poll_ring(FileSession&, size_t ring) noexcept;
    NTSTATUS drain(FileSession&, uint64_t serial) noexcept;
    // Wait for one recorded signal alone. False when that ring's private
    // timeline is not host-published; the caller then keeps its full drain.
    bool producer_published(const FileSession&, const SignalPoint&) const noexcept;
    NTSTATUS wait_signal(FileSession&, const SignalPoint&) noexcept;
    void record_signals(FileSession&, rx7900::ByteView input, uint64_t serial, size_t ring,
                        uint64_t point) noexcept;
    // token zero settles every pinned image of the session.
    NTSTATUS settle_receipts(FileSession&, uint64_t token, bool wait) noexcept;
    NTSTATUS settle_all_receipts() noexcept;
    NTSTATUS close_session(FileSession&) noexcept;
    NTSTATUS revoke_views(FileSession&, bool in_exit = false) noexcept;
    NTSTATUS map_blob(FileSession&, uint64_t token, ExchangeReply&) noexcept;
    NTSTATUS revoke_blob(FileSession&, uint64_t token, bool in_exit = false) noexcept;
    static bool has_views(const FileSession&) noexcept;
    // A live borrow shutdown() must retain: views of a process not exiting.
    static bool unrevocable_borrow(const FileSession&) noexcept;
    void invalidate_session(FileSession&, bool retain_views = false) noexcept;
    void finish_recovery() noexcept;
    NTSTATUS recover_all() noexcept;
    bool token_exists(uint64_t token) const noexcept;
    void release_session(size_t slot) noexcept;

    // Each nonpaged allocation is owned by one slot until clean file cleanup.
    // Failed cleanup deliberately retains the complete allocation until reboot.
    PVOID sessions_[QLV_KERNEL_MAX_CLIENTS]{};
    NativeTransport* transport_ = nullptr;
    PHYSICAL_ADDRESS aperture_base_{};
    uint64_t aperture_bytes_ = 0;
    // Never reset/reuse image identities during the device extension lifetime.
    uint64_t next_image_lease_ = 1;
    ReceiptOwner receipt_owner_{};
    bool recovery_waiting_views_ = false;
    bool active_ = false;
    bool quarantined_ = false;
};

} // namespace lightning::reactos

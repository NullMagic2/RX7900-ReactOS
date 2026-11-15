/*
Module: lightning.reactos.exchange_policy
Purpose: Validate the renderer packet closure against one retained file session.
Created: 2026-10-21
Architecture: Pure C++ policy only; does not publish an IOCTL or submit hardware work.
*/
#pragma once

#include "lightning_protocol.hpp"
#include "lightning_image.hpp"

namespace lightning::reactos {

enum class ExchangeStatus {
    validated, short_buffer, malformed, unsupported_operation, wrong_context,
    unknown_resource, resource_busy, invalid_range, host_error
};

enum class ResourceKind { blob, sync };

struct ResourceInfo final {
    ResourceKind kind = ResourceKind::blob;
    uint64_t byte_count = 0;
    uint32_t blob_flags = 0;
    uint32_t pending_uses = 0;
    uint64_t observed_sync_value = 0;
};

struct ExchangeSession final {
    uint64_t context; // Actual retained per-file host context; zero only before OPEN.
    uint64_t aperture_bytes; // Validated assigned aperture, not caller supplied size.
    void* resource_owner;
    // Resolve only live tokens in this exact session/generation. The native owner
    // holds its guard and lifetime pins through validation, DMA and commit. False
    // means absent/foreign/stale; this callback does not establish native ownership.
    bool (*lookup)(void*, uint64_t token, ResourceInfo&) noexcept;
};

class ValidatedExchange final {
public:
    ValidatedExchange() noexcept = default;
    bool valid() const noexcept { return valid_; }
    const QLV_HEADER& header() const noexcept { return header_; }
    size_t response_capacity() const noexcept { return response_capacity_; }

private:
    friend ExchangeStatus validate_exchange_request(rx7900::ByteView, size_t,
        const ExchangeSession&, ValidatedExchange&) noexcept;
    friend ExchangeStatus validate_exchange_reply(const ValidatedExchange&,
        rx7900::ByteView, struct ExchangeReply&) noexcept;
    QLV_HEADER header_{};
    ResourceInfo resource_{};
    uint64_t aperture_bytes_ = 0;
    size_t response_capacity_ = 0;
    void* resource_owner_ = nullptr;
    bool (*lookup_)(void*, uint64_t, ResourceInfo&) noexcept = nullptr;
    bool valid_ = false;
};

struct ExchangeReply final {
    QLV_HEADER header{};
    QLV_CAPS caps{};
};

// Exact header+payload immutable snapshots only; no native struct overlays. The
// reply capacity is the separately owned output buffer. No views escape. OPEN
// requires no current context; all other operations require exact per-file match.
// Requests use the normal STATUS_UNSUPPORTED sentinel. IMPORT/WATCH are
// unsupported. WAIT is an immediate poll; timeout replies are valid host results.
// SUBMIT accepts queue framing only: it does not establish renderer completion or
// release pending_uses. Native code must reserve/pin resources before submission.
ExchangeStatus validate_exchange_request(rx7900::ByteView packet, size_t reply_capacity,
    const ExchangeSession& session, ValidatedExchange& output) noexcept;

// Validates before writing the complete bounded request snapshot into exclusively
// owned native DMA storage. Input remains stable throughout and must not overlap
// output. Both views are private kernel storage; never probe user pointers here.
// On failure neither output bytes nor a prior validation token remain published.
ExchangeStatus snapshot_exchange_request(rx7900::ByteView packet, size_t reply_capacity,
    const ExchangeSession& session, rx7900::sdma::CommandBuffer dma_output,
    ValidatedExchange& output) noexcept;

// Add one native-owned completion point to a caller SUBMIT in private DMA
// storage. The caller packet remains immutable and its original reply contract
// remains the public one. The output owns no pointers after this call; its
// complete span is checked again against the session before host submission.
// A full sync list or payload must use the existing separate-fence path.
ExchangeStatus snapshot_submit_with_private_fence(rx7900::ByteView packet,
    size_t reply_capacity, const ExchangeSession& session, uint64_t sync,
    uint64_t point, uint8_t* dma_storage, size_t dma_capacity,
    ValidatedExchange& output, size_t& bytes_written) noexcept;

// Hardware completion only makes response bytes available. Successful SUBMIT
// reply is queue acceptance, never a fence. MAP host value must remain zero;
// only a later native process-owned mapping may replace it with a checked VA.
// Non-success wire statuses are host_error (including WAIT timeout), not malformed.
ExchangeStatus validate_exchange_reply(const ValidatedExchange& request,
    rx7900::ByteView packet, ExchangeReply& output) noexcept;

// Write only initialized header+actual payload into METHOD_BUFFERED output.
// Native code may replace a validated MAP value after creating the owned view.
ExchangeStatus encode_exchange_reply(const ExchangeReply& reply,
    rx7900::sdma::CommandBuffer output, size_t& bytes_written) noexcept;
ExchangeStatus unsupported_exchange_reply(rx7900::ByteView packet, uint64_t context,
                                           ExchangeReply& output) noexcept;

struct ApertureMapping final {
    uint64_t context;
    uint64_t resource;
    uint64_t offset;
    uint64_t byte_count;
    uint32_t cache_mode;
};

enum class MappingRelation { disjoint, identical, conflict, invalid };

// Before any process mapping, compare against every retained device-wide mapping.
// Repeated same-session/same-resource maps may reuse only identical extents/cache.
MappingRelation mapping_relation(ApertureMapping candidate, ApertureMapping existing,
                                  uint64_t aperture_bytes) noexcept;

// Pure postcondition for an OS-created UserMode view. Native code supplies the
// actual process user limit and ABI pointer width; user input cannot choose them.
bool valid_user_mapping(uint64_t address, uint64_t byte_count, uint64_t user_limit,
                        uint32_t pointer_bits) noexcept;

} // namespace lightning::reactos

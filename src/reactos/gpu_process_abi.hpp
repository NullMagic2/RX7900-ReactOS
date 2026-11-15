/*
Module: rx7900.reactos.gpu_process_abi
Purpose: Define the bounded opaque native process-memory and queue lifecycle ABI.
Created: 2026-10-22
Architecture: VideoPort captures requests; gpu_process authenticates the executing
NT process. No PID, host pointer, physical address or command stream is accepted.
*/
#pragma once

#include <stdint.h>
#include "../common/byte_view.hpp"

namespace rx7900::reactos::process_abi {
constexpr uint32_t legacy_version = 1;
constexpr uint32_t version = 2;
// Version 3 adds kernel-built graphics (process_graphics_abi.hpp) and bulk
// buffer transfers; every version-1/2 request keeps its exact meaning.
constexpr uint32_t graphics_version = 3;
// Version 4 (2026-10-24) reports destructive-reset recovery and process loss;
// every version-1/2/3 request keeps its exact meaning, and a version-4 request
// accepts exactly the version-3 operations and fields. Additionally:
//  * query returns Reply.bytes = reset generation: the confirmed destructive
//    (mode-1) resets this service has survived (Linux vram_lost_counter; a
//    client compares it as amdgpu_ctx_query2 reports AMDGPU_CTX_QUERY2_FLAGS_VRAMLOST).
//  * buffer_lost: a buffer named by the request (directly or inside a graphics
//    description) lost its VRAM contents in a confirmed reset; Reply.lost_buffer
//    names it. Not executed, no GPU or CPU mutation, never stale data. Answered
//    exactly once per lost buffer: the handle is retired with that answer and a
//    later request naming it is invalid_request; buffer handles never recycle.
//    GTT buffers survive a reset with their contents and GPU virtual address.
//  * work_lost: submission_query/graphics_query of work that was in flight at a
//    confirmed reset; it was neither proven complete nor replayed (Linux
//    amdgpu_userq force-completes such fences with an error). Stable answer.
//    Fence values continue across the reset, so a new sequence never aliases it.
//  * session_lost: the session could not be reconstructed after a reset; every
//    request answers it except close, which retires the session.
// A version-1/2/3 request receives quarantined wherever a version-4 request
// receives one of these, with the same exactly-once buffer retirement.
constexpr uint32_t recovery_version = 4;
constexpr uint32_t bulk_bytes = 256u * 1024u;
constexpr uint32_t copy_bytes = 4096;
// FILE_DEVICE_VIDEO, private function 0xb00, METHOD_BUFFERED, FILE_ANY_ACCESS.
constexpr uint32_t ioctl = (0x23u << 16) | (0xb00u << 2);
enum class Operation : uint32_t {
    query = 0, open = 1, close = 2, allocate = 3, release = 4,
    map = 5, unmap = 6, read = 7, write = 8,
    queue_create = 9, queue_destroy = 10, poll = 11,
    copy = 12, fill = 13, submission_query = 14,
    // Version 3: handle names an opaque gfx queue (graphics_*) or buffer (bulk).
    // graphics_open binds the session's graphics context to that gfx queue;
    // handle 0 binds it to the kernel gfx ring instead (later graphics_submit/
    // query then also name handle 0; refused unavailable without that ring);
    // graphics_submit has bytes = payload length of a graphics_abi SubmitHeader
    // stream and returns the fence sequence in Reply.bytes; graphics_query takes
    // it in offset. buffer_upload/readback move 1..bulk_bytes at offset of an
    // owned buffer through the driver's CPU mapping (refused while leased).
    graphics_open = 15, graphics_submit = 16, graphics_query = 17,
    buffer_upload = 18, buffer_readback = 19
};
enum class Outcome : uint32_t {
    success = 0, invalid_request = 1, unavailable = 2,
    access_denied = 3, busy = 4, exhausted = 5, quarantined = 6,
    // Version 4 only (see recovery_version).
    buffer_lost = 7, work_lost = 8, session_lost = 9
};

// Versions whose requests may name graphics/bulk operations.
inline bool graphics_capable(uint32_t abi_version) noexcept {
    return abi_version == graphics_version || abi_version == recovery_version;
}
// All operations use this fixed, captured descriptor; unused fields must be 0.
// bytes is allocation length or copy length; offset only read/write; address
// only map (GPU virtual address). domain 0 GTT/1 VRAM; access 1 read/2 write/3
// read_write. queue_create accepts engine 0 gfx/1 compute/2 SDMA and returns an
// opaque queue handle. Rings, MQDs, PASIDs and doorbells never cross this ABI.
struct Request final {
    uint32_t size;
    uint32_t abi_version;
    uint32_t operation;
    uint32_t reserved;
    uint64_t session;
    uint64_t handle;
    uint64_t bytes;
    uint64_t offset;
    uint64_t address;
    uint32_t domain;
    uint32_t access;
};
// lost_buffer (offset 40, zero before version 4 and for every other outcome)
// names the buffer of a version-4 buffer_lost answer.
struct Reply final {
    uint32_t size;
    uint32_t abi_version;
    uint32_t outcome;
    uint32_t payload_bytes;
    uint64_t session;
    uint64_t handle;
    uint64_t bytes;
    uint64_t lost_buffer;
};
// Version 2 copy/fill append exactly this semantic descriptor to Request.
// Request.handle selects an opaque owned SDMA queue; no IB or GPUVA is supplied.
// Copy requires value zero. Fill requires source/source_offset zero. Accepted
// work returns its fence sequence in Reply.bytes; submission_query uses that
// sequence in Request.offset and reports success only for actual completion.
struct Transfer final {
    uint64_t source;
    uint64_t destination;
    uint64_t source_offset;
    uint64_t destination_offset;
    uint64_t bytes;
    uint32_t value;
    uint32_t reserved;
};
static_assert(sizeof(Request) == 64, "Process request has fixed layout");
static_assert(sizeof(Reply) == 48, "Process reply has fixed layout");
static_assert(sizeof(Transfer) == 48, "Semantic transfer has fixed layout");

inline bool decode_transfer(ByteView input, Transfer& transfer, bool fill) noexcept {
    transfer = {};
    return input.size() == sizeof(transfer) && input.read_u64(0, transfer.source) &&
        input.read_u64(8, transfer.destination) && input.read_u64(16, transfer.source_offset) &&
        input.read_u64(24, transfer.destination_offset) && input.read_u64(32, transfer.bytes) &&
        input.read_u32(40, transfer.value) && input.read_u32(44, transfer.reserved) &&
        transfer.destination != 0 && transfer.bytes != 0 && transfer.reserved == 0 &&
        (fill ? transfer.source == 0 && transfer.source_offset == 0 :
            transfer.source != 0 && transfer.value == 0);
}
// Decode only already-captured readable request bytes; no unaligned overlays.
inline bool decode_request(ByteView input, Request& request) noexcept {
    request = {};
    return input.size() >= sizeof(request) &&
        input.read_u32(0, request.size) && input.read_u32(4, request.abi_version) &&
        input.read_u32(8, request.operation) && input.read_u32(12, request.reserved) &&
        input.read_u64(16, request.session) && input.read_u64(24, request.handle) &&
        input.read_u64(32, request.bytes) && input.read_u64(40, request.offset) &&
        input.read_u64(48, request.address) && input.read_u32(56, request.domain) &&
        input.read_u32(60, request.access) && request.size == sizeof(request) &&
        (graphics_capable(request.abi_version) || request.abi_version == version ||
         request.abi_version == legacy_version) &&
        request.reserved == 0 && request.operation <= static_cast<uint32_t>(Operation::buffer_readback) &&
        (graphics_capable(request.abi_version) ||
         request.operation <= static_cast<uint32_t>(Operation::submission_query)) &&
        (request.abi_version != legacy_version || request.operation <= static_cast<uint32_t>(Operation::poll));
}

// Version-3 operations: every unused field zero, counts bounded.
inline bool valid_graphics_fields(const Request& r) noexcept {
    const auto op = static_cast<Operation>(r.operation);
    const bool bulk = op == Operation::buffer_upload || op == Operation::buffer_readback;
    if (!graphics_capable(r.abi_version) || r.session == 0 || (bulk && r.handle == 0) ||
        r.address != 0 || r.domain != 0 || r.access != 0) return false;
    switch (op) {
    case Operation::graphics_open: return r.bytes == 0 && r.offset == 0;
    case Operation::graphics_submit: return r.bytes != 0 && r.bytes <= bulk_bytes && r.offset == 0;
    case Operation::graphics_query: return r.bytes == 0 && r.offset != 0 && r.offset <= UINT32_MAX;
    case Operation::buffer_upload:
    case Operation::buffer_readback: return r.bytes != 0 && r.bytes <= bulk_bytes;
    default: return false;
    }
}

inline bool valid_request_fields(const Request& r) noexcept {
    const auto op = static_cast<Operation>(r.operation);
    if (r.operation >= static_cast<uint32_t>(Operation::graphics_open)) return valid_graphics_fields(r);
    const bool buffer = op == Operation::allocate;
    const bool copy = op == Operation::read || op == Operation::write;
    const bool mapping = op == Operation::map;
    const bool create_queue = op == Operation::queue_create;
    const bool query_submission = op == Operation::submission_query;
    const bool transfer = op == Operation::copy || op == Operation::fill;
    if ((!copy && !query_submission && r.offset != 0) || (!mapping && r.address != 0) ||
        (!buffer && !copy && r.bytes != 0) ||
        (!buffer && !create_queue && r.domain != 0) ||
        (!buffer && !mapping && r.access != 0)) return false;
    if ((buffer && r.domain > 1) || (create_queue && r.domain > 2) ||
        ((buffer || mapping) && (r.access == 0 || r.access > 3)) ||
        (copy && (r.bytes == 0 || r.bytes > copy_bytes))) return false;
    if ((transfer || query_submission) &&
        ((r.abi_version != version && !graphics_capable(r.abi_version)) ||
         r.session == 0 || r.handle == 0)) return false;
    if (query_submission && (r.offset == 0 || r.offset > UINT32_MAX)) return false;
    if ((op == Operation::query || op == Operation::open) &&
        (r.session != 0 || r.handle != 0)) return false;
    if ((op == Operation::close || op == Operation::allocate ||
         op == Operation::queue_create || op == Operation::poll) && r.handle != 0) return false;
    return true;
}

} // namespace rx7900::reactos::process_abi

/*
Module: lightning.tests.exchange_policy
Purpose: Exercise renderer wire framing, session isolation, ranges and fence lifetimes.
Created: 2026-10-21
Architecture: Synthetic immutable CPU packets; no renderer or OS mapping success mocked.
*/
#include <algorithm>
#include <array>
#include <cstdio>
#include <vector>
#include "lightning_exchange_policy.hpp"
#include "lightning_completion.hpp"

namespace {

using namespace lightning::reactos;
using rx7900::ByteView;
using rx7900::sdma::CommandBuffer;
constexpr uint64_t context_token = 0x400000000ULL;
constexpr uint64_t blob_token = 0x100000001ULL;
constexpr uint64_t sync_token = 0x200000002ULL;
constexpr uint64_t private_sync_token = 0x300000003ULL;
unsigned checks = 0;
unsigned failures = 0;

void expect(bool condition, const char* description) {
    ++checks;

    if (!condition) {
        ++failures;
        std::fprintf(stderr, "FAIL: %s\n", description);
    }
}

void write_u64(CommandBuffer out, size_t offset, uint64_t value) {
    out.write_u32(offset, static_cast<uint32_t>(value));
    out.write_u32(offset + 4, static_cast<uint32_t>(value >> 32));
}

struct Resources final {
    ResourceInfo blob{ResourceKind::blob, 8192, QLV_MAPPABLE | QLV_COHERENT, 0, 0};
    ResourceInfo sync{ResourceKind::sync, 0, 0, 0, 10};
    ResourceInfo private_sync{ResourceKind::sync, 0, 0, 0, 0};
    unsigned lookups = 0;

    static bool lookup(void* owner, uint64_t token, ResourceInfo& output) noexcept {
        auto& resources = *static_cast<Resources*>(owner);
        ++resources.lookups;

        if (token == blob_token) {
            output = resources.blob;
            return true;
        }

        if (token == sync_token) {
            output = resources.sync;
            return true;
        }

        if (token == private_sync_token) {
            output = resources.private_sync;
            return true;
        }

        return false;
    }

    ExchangeSession session(uint64_t context = context_token) noexcept {
        return {context, QLV_APERTURE_BYTES, this, lookup};
    }
};

QLV_HEADER header(uint16_t operation) {
    QLV_HEADER result{};
    result.magic = QLV_MAGIC;
    result.version = QLV_VERSION;
    result.operation = operation;
    result.context = operation == QLV_OPEN ? 0 : context_token;
    result.status = QLV_STATUS_UNSUPPORTED;
    return result;
}

std::vector<uint8_t> packet(QLV_HEADER h) {
    std::vector<uint8_t> bytes(QLV_HEADER_BYTES + h.payload_bytes);
    CommandBuffer out(bytes.data(), bytes.size());
    out.write_u32(QLV_MAGIC_OFFSET, h.magic);
    out.write_u32(QLV_VERSION_OFFSET, h.version | (static_cast<uint32_t>(h.operation) << 16));
    write_u64(out, QLV_CONTEXT_OFFSET, h.context);
    write_u64(out, QLV_RESOURCE_OFFSET, h.resource);
    write_u64(out, QLV_VALUE_OFFSET, h.value);
    write_u64(out, QLV_BYTES_OFFSET, h.bytes);
    write_u64(out, QLV_OFFSET_OFFSET, h.offset);
    out.write_u32(QLV_STATUS_OFFSET, static_cast<uint32_t>(h.status));
    out.write_u32(QLV_FLAGS_OFFSET, h.flags);
    out.write_u32(QLV_PAYLOAD_BYTES_OFFSET, h.payload_bytes);
    out.write_u32(QLV_RESERVED_OFFSET, h.reserved);
    return bytes;
}

QLV_HEADER ordinary(uint16_t operation) {
    auto h = header(operation);

    switch (operation) {
    case QLV_GET_CAPS: h.bytes = QLV_CAPS_BYTES; break;
    case QLV_CREATE_BLOB:
        h.bytes = 8192;
        h.offset = 0x1234567800000001ULL;
        h.flags = QLV_MAPPABLE | QLV_COHERENT;
        break;
    case QLV_DESTROY_BLOB:
    case QLV_MAP_BLOB:
    case QLV_UNMAP_BLOB: h.resource = blob_token; break;
    case QLV_CREATE_SYNC: h.value = 11; break;
    case QLV_DESTROY_SYNC:
    case QLV_READ_SYNC: h.resource = sync_token; break;
    case QLV_WRITE_SYNC: h.resource = sync_token; h.value = 11; break;
    case QLV_SUBMIT: h.offset = 1; h.bytes = 4; h.payload_bytes = 20; break;
    case QLV_WAIT_SYNCS: h.bytes = 1; h.flags = QLV_WAIT_ANY; h.payload_bytes = 16; break;
    case QLV_FLUSH:
    case QLV_INVALIDATE: h.resource = blob_token; h.offset = 4096; h.bytes = 4096; break;
    default: break;
    }

    return h;
}

std::vector<uint8_t> ordinary_packet(uint16_t operation) {
    auto bytes = packet(ordinary(operation));

    if (operation == QLV_SUBMIT || operation == QLV_WAIT_SYNCS) {
        CommandBuffer out(bytes.data(), bytes.size());
        write_u64(out, 64, sync_token);
        write_u64(out, 72, 11);

        if (operation == QLV_SUBMIT) {
            out.write_u32(80, 0x1234abcd);
        }
    }

    return bytes;
}

ExchangeStatus validate(const std::vector<uint8_t>& bytes, Resources& resources,
                        ValidatedExchange& output, size_t capacity = response_capacity) {
    const bool opening = bytes.size() >= 8 && ByteView(bytes.data(), bytes.size()).size() >= 8 &&
        bytes.at(6) == QLV_OPEN;
    return validate_exchange_request({bytes.data(), bytes.size()}, capacity,
        resources.session(opening ? 0 : context_token), output);
}

void check_closure_and_snapshot() {
    Resources resources;
    ValidatedExchange request;
    expect(!request.valid(), "default request token is not validated");

    for (uint16_t operation = QLV_OPEN; operation <= QLV_INVALIDATE; ++operation) {
        const auto bytes = ordinary_packet(operation);
        expect(validate(bytes, resources, request) == ExchangeStatus::validated && request.valid() &&
               request.header().operation == operation, "renderer closure request accepted with actual session tokens");
        std::vector<uint8_t> dma(bytes.size() + 4, 0xa5);
        const auto status = snapshot_exchange_request({bytes.data(), bytes.size()}, response_capacity,
            resources.session(operation == QLV_OPEN ? 0 : context_token),
            {dma.data(), dma.size()}, request);
        bool equal = true;

        for (size_t index = 0; index < bytes.size(); ++index) {
            equal = equal && dma.at(index) == bytes.at(index);
        }

        expect(status == ExchangeStatus::validated && equal && dma.back() == 0xa5,
               "complete validated immutable packet copied with no trailing DMA write");
    }

    auto bytes = ordinary_packet(QLV_SUBMIT);
    std::vector<uint8_t> dma(bytes.size(), 0xa5);
    expect(snapshot_exchange_request({bytes.data(), bytes.size()}, response_capacity, resources.session(),
           {dma.data(), dma.size() - 1}, request) == ExchangeStatus::short_buffer && !request.valid(),
           "short DMA ownership span clears token before writing");
    bool unchanged = true;

    for (const auto byte : dma) {
        unchanged = unchanged && byte == 0xa5;
    }

    expect(unchanged, "short DMA snapshot leaves all prior bytes intact");
    CommandBuffer out(bytes.data(), bytes.size());
    write_u64(out, 64, 0xdeadbeefdeadbeefULL);
    expect(snapshot_exchange_request({bytes.data(), bytes.size()}, response_capacity, resources.session(),
           {dma.data(), dma.size()}, request) == ExchangeStatus::unknown_resource &&
           dma.front() == 0xa5 && dma.back() == 0xa5, "foreign sync rejects complete snapshot before any store");
}

void check_framing_and_sessions() {
    Resources resources;
    ValidatedExchange request;
    auto bytes = ordinary_packet(QLV_GET_CAPS);

    for (size_t size = 0; size < QLV_HEADER_BYTES; ++size) {
        expect(validate_exchange_request({bytes.data(), size}, response_capacity, resources.session(), request) ==
               ExchangeStatus::short_buffer && !request.valid(), "truncated headers cannot publish request tokens");
    }

    expect(validate_exchange_request({nullptr, SIZE_MAX}, response_capacity, resources.session(), request) ==
           ExchangeStatus::short_buffer, "null packet remains empty despite forged size");

    for (const size_t capacity : {size_t{0}, size_t{63}, size_t{QLV_HEADER_BYTES + QLV_MAX_CAPSET_BYTES + 1}}) {
        expect(validate(bytes, resources, request, capacity) == ExchangeStatus::short_buffer,
               "native reply allocation capacity must fit bounded contract");
    }

    expect(validate(bytes, resources, request, 64) == ExchangeStatus::short_buffer,
           "GET_CAPS requires real224-byte reply storage");
    const auto session = resources.session();
    expect(validate_exchange_request({bytes.data(), bytes.size()}, response_capacity,
           resources.session(context_token + 1), request) == ExchangeStatus::wrong_context,
           "per-file context mismatch cannot borrow another context");
    expect(validate_exchange_request({bytes.data(), bytes.size()}, response_capacity,
           resources.session(0), request) == ExchangeStatus::wrong_context,
           "closed session cannot issue renderer operations");
    bytes = ordinary_packet(QLV_OPEN);
    expect(validate_exchange_request({bytes.data(), bytes.size()}, response_capacity, session, request) ==
           ExchangeStatus::wrong_context, "OPEN cannot replace a live file context");

    for (const uint16_t operation : {uint16_t{0}, uint16_t{QLV_IMPORT_BLOB}, uint16_t{QLV_WATCH_SYNCS},
                                   uint16_t{UINT16_MAX}}) {
        bytes = packet(header(operation));
        expect(validate(bytes, resources, request) == ExchangeStatus::unsupported_operation,
               "optional imports/events are not accidentally forwarded");
    }

    for (const size_t field : {size_t{0}, size_t{4}, size_t{48}, size_t{56}, size_t{60}}) {
        bytes = ordinary_packet(QLV_GET_CAPS);
        bytes.at(field) ^= 1;
        expect(validate(bytes, resources, request) == ExchangeStatus::malformed && !request.valid(),
               "header signature/version/status/payload/reserved fields validated");
    }

    bytes = ordinary_packet(QLV_GET_CAPS);
    auto invalid = resources.session();
    invalid.aperture_bytes = QLV_APERTURE_BYTES + QLV_PAGE_BYTES;
    expect(validate_exchange_request({bytes.data(), bytes.size()}, response_capacity, invalid, request) ==
           ExchangeStatus::malformed, "arbitrary caller size cannot become assigned aperture proof");
}

void check_resources_and_cache() {
    Resources resources;
    ValidatedExchange request;
    auto h = ordinary(QLV_CREATE_BLOB);
    h.bytes = 1;
    expect(validate(packet(h), resources, request) == ExchangeStatus::malformed,
           "host blob creation requires complete pages");
    h.bytes = QLV_APERTURE_BYTES + QLV_PAGE_BYTES;
    expect(validate(packet(h), resources, request) == ExchangeStatus::malformed, "individual blob bound preserved");
    h.bytes = QLV_APERTURE_BYTES;
    expect(validate(packet(h), resources, request) == ExchangeStatus::validated, "largest512MiB blob request accepted");
    h.flags = QLV_COHERENT;
    expect(validate(packet(h), resources, request) == ExchangeStatus::malformed,
           "coherent allocation cannot omit mappable requirement");
    h.flags = UINT32_MAX;
    expect(validate(packet(h), resources, request) == ExchangeStatus::malformed, "unknown blob flags rejected");

    for (const uint16_t operation : {uint16_t{QLV_MAP_BLOB}, uint16_t{QLV_DESTROY_BLOB},
                                   uint16_t{QLV_FLUSH}, uint16_t{QLV_READ_SYNC}}) {
        h = ordinary(operation);
        h.resource = 0;
        expect(validate(packet(h), resources, request) == ExchangeStatus::unknown_resource,
               "zero handle cannot resolve a resource");
        h.resource = UINT64_MAX;
        expect(validate(packet(h), resources, request) == ExchangeStatus::unknown_resource,
               "foreign or stale generation-tagged resource rejected");
        h.resource = operation == QLV_READ_SYNC ? blob_token : sync_token;
        expect(validate(packet(h), resources, request) == ExchangeStatus::unknown_resource,
               "blob and sync token kinds cannot be confused");
    }

    resources.blob.pending_uses = 1;
    expect(validate(ordinary_packet(QLV_DESTROY_BLOB), resources, request) == ExchangeStatus::resource_busy,
           "queued GPU leases prevent resource destruction");
    expect(validate(ordinary_packet(QLV_UNMAP_BLOB), resources, request) == ExchangeStatus::validated,
           "queued GPU reads of a live allocation do not delay CPU view teardown");
    expect(validate(ordinary_packet(QLV_MAP_BLOB), resources, request) == ExchangeStatus::validated,
           "queued GPU reads of a live allocation do not delay a new CPU view");
    resources.blob.pending_uses = 0;
    resources.blob.blob_flags = 0;
    expect(validate(ordinary_packet(QLV_MAP_BLOB), resources, request) == ExchangeStatus::invalid_range,
           "nonmappable host allocation cannot obtain a CPU view");
    resources.blob.blob_flags = QLV_MAPPABLE;

    for (const uint16_t operation : {uint16_t{QLV_FLUSH}, uint16_t{QLV_INVALIDATE}}) {
        h = ordinary(operation);
        h.offset = 8192;
        h.bytes = 0;
        expect(validate(packet(h), resources, request) == ExchangeStatus::validated,
               "empty range at owned allocation end is valid");
        h.bytes = 1;
        expect(validate(packet(h), resources, request) == ExchangeStatus::invalid_range,
               "cache range cannot extend beyond retained allocation");
        h.offset = UINT64_MAX;
        h.bytes = 2;
        expect(validate(packet(h), resources, request) == ExchangeStatus::invalid_range,
               "cache range arithmetic cannot wrap");
    }

    resources.blob.byte_count = UINT64_MAX;
    expect(validate(ordinary_packet(QLV_MAP_BLOB), resources, request) == ExchangeStatus::unknown_resource,
           "malformed native resource metadata also fails closed");
}

void check_submission_and_wait() {
    Resources resources;
    ValidatedExchange request;
    auto h = ordinary(QLV_SUBMIT);
    h.offset = QLV_MAX_SUBMIT_SYNCS + 1;
    expect(validate(packet(h), resources, request) == ExchangeStatus::malformed, "submit sync count bounded before multiply");
    h.offset = UINT64_MAX;
    expect(validate(packet(h), resources, request) == ExchangeStatus::malformed, "wide submit count cannot narrow onx86");
    h = ordinary(QLV_SUBMIT);
    h.value = QLV_MAX_TIMELINES;
    expect(validate(packet(h), resources, request) == ExchangeStatus::malformed, "ring index bounded by host timeline contract");
    h.value = 0;
    h.bytes = 3;
    expect(validate(packet(h), resources, request) == ExchangeStatus::malformed, "commands must be whole32-bit words");
    h.bytes = UINT64_MAX;
    expect(validate(packet(h), resources, request) == ExchangeStatus::malformed, "wide command size cannot wrap/narrow");

    auto bytes = ordinary_packet(QLV_SUBMIT);
    CommandBuffer out(bytes.data(), bytes.size());
    write_u64(out, 72, 9);
    expect(validate(bytes, resources, request) == ExchangeStatus::invalid_range,
           "submission cannot move an observed timeline backwards");
    write_u64(out, 72, 11);
    bytes.pop_back();
    expect(validate(bytes, resources, request) == ExchangeStatus::malformed, "truncated command tail rejects whole packet");

    h = header(QLV_SUBMIT);
    h.offset = QLV_MAX_SUBMIT_SYNCS;
    h.bytes = QLV_MAX_PAYLOAD_BYTES - QLV_MAX_SUBMIT_SYNCS * QLV_SYNC_POINT_BYTES;
    h.payload_bytes = QLV_MAX_PAYLOAD_BYTES;
    bytes = packet(h);
    out = {bytes.data(), bytes.size()};

    for (size_t index = 0; index < QLV_MAX_SUBMIT_SYNCS; ++index) {
        write_u64(out, QLV_HEADER_BYTES + index * QLV_SYNC_POINT_BYTES, sync_token);
        write_u64(out, QLV_HEADER_BYTES + index * QLV_SYNC_POINT_BYTES + 8, 11);
    }

    expect(validate(bytes, resources, request) == ExchangeStatus::validated,
           "maximum16MiB payload with64 valid sync points fits checked arithmetic");
    h = header(QLV_SUBMIT);
    h.offset = 0;
    expect(validate(packet(h), resources, request) == ExchangeStatus::validated,
           "empty sync-only submission still requires a real host fence, not local fake success");

    h = ordinary(QLV_WAIT_SYNCS);
    h.bytes = QLV_MAX_WAIT_SYNCS;
    h.payload_bytes = QLV_MAX_WAIT_SYNCS * QLV_SYNC_POINT_BYTES;
    bytes = packet(h);
    out = {bytes.data(), bytes.size()};

    for (size_t index = 0; index < QLV_MAX_WAIT_SYNCS; ++index) {
        write_u64(out, QLV_HEADER_BYTES + index * QLV_SYNC_POINT_BYTES, sync_token);
        write_u64(out, QLV_HEADER_BYTES + index * QLV_SYNC_POINT_BYTES + 8, 11);
    }

    expect(validate(bytes, resources, request) == ExchangeStatus::validated, "maximum1024-point immediate wait accepted");
    write_u64(out, QLV_HEADER_BYTES + (QLV_MAX_WAIT_SYNCS - 1) * QLV_SYNC_POINT_BYTES, UINT64_MAX);
    expect(validate(bytes, resources, request) == ExchangeStatus::unknown_resource && !request.valid(),
           "unknown trailing sync prevents a partially validated wait");
    h = ordinary(QLV_WAIT_SYNCS);
    h.value = 1;
    expect(validate(packet(h), resources, request) == ExchangeStatus::malformed,
           "blocking host wait cannot deadlock serialized signal mailbox");
    h.value = 0;
    h.bytes = UINT64_MAX;
    expect(validate(packet(h), resources, request) == ExchangeStatus::malformed,
           "wait count cannot overflow sync-point product");
    resources.sync.pending_uses = 1;
    h = ordinary(QLV_WRITE_SYNC);
    h.value = 0;
    expect(validate(packet(h), resources, request) == ExchangeStatus::resource_busy,
           "reset cannot reuse a timeline still retained by pending GPU work");
    expect(validate(ordinary_packet(QLV_DESTROY_SYNC), resources, request) == ExchangeStatus::resource_busy,
           "pending fence retains sync lifetime");
    resources.sync.pending_uses = 0;
    expect(validate(packet(h), resources, request) == ExchangeStatus::validated,
           "idle timeline reset is explicit and permitted");
    h.value = 9;
    expect(validate(packet(h), resources, request) == ExchangeStatus::invalid_range,
           "nonzero sync write cannot move backwards");
}

void check_private_submit_snapshot() {
    Resources resources;
    const auto original = ordinary_packet(QLV_SUBMIT);
    std::vector<uint8_t> dma(original.size() + QLV_SYNC_POINT_BYTES + 4, 0xa5);
    ValidatedExchange fused;
    size_t written = 99;
    const auto status = snapshot_submit_with_private_fence(
        {original.data(), original.size()}, response_capacity, resources.session(),
        private_sync_token, 22, dma.data(), dma.size(), fused, written);
    const ByteView view(dma.data(), written);
    uint64_t value = 0;

    expect(status == ExchangeStatus::validated && fused.valid() &&
        fused.header().offset == 2 && fused.header().payload_bytes == 36 &&
        written == original.size() + QLV_SYNC_POINT_BYTES,
        "fused snapshot validates one extra real host sync in the same SUBMIT");
    expect(view.read_u64(QLV_HEADER_BYTES, value) && value == sync_token &&
        view.read_u64(QLV_HEADER_BYTES + QLV_SYNC_VALUE_OFFSET, value) && value == 11 &&
        view.read_u64(QLV_HEADER_BYTES + QLV_SYNC_POINT_BYTES, value) && value == private_sync_token &&
        view.read_u64(QLV_HEADER_BYTES + QLV_SYNC_POINT_BYTES + QLV_SYNC_VALUE_OFFSET, value) && value == 22,
        "fused snapshot preserves caller sync order before its private lifetime point");
    uint32_t command = 0;
    expect(view.read_u32(QLV_HEADER_BYTES + 2 * QLV_SYNC_POINT_BYTES, command) &&
        command == 0x1234abcd && original == ordinary_packet(QLV_SUBMIT) &&
        dma.at(written) == 0xa5,
        "fused snapshot moves commands without mutating caller bytes or trailing DMA");

    written = 99;
    expect(snapshot_submit_with_private_fence({original.data(), original.size()},
        response_capacity, resources.session(), private_sync_token, 22,
        dma.data(), original.size() + QLV_SYNC_POINT_BYTES - 1, fused, written) ==
        ExchangeStatus::short_buffer && written == 0 && !fused.valid(),
        "short DMA buffer cannot publish a partial fused request");
    expect(snapshot_submit_with_private_fence({original.data(), original.size()},
        response_capacity, resources.session(), UINT64_MAX, 22,
        dma.data(), dma.size(), fused, written) == ExchangeStatus::unknown_resource &&
        written == 0 && !fused.valid(),
        "private sync must belong to the same live native session");

    auto full = header(QLV_SUBMIT);
    full.offset = QLV_MAX_SUBMIT_SYNCS;
    full.payload_bytes = QLV_MAX_SUBMIT_SYNCS * QLV_SYNC_POINT_BYTES;
    auto full_packet = packet(full);
    CommandBuffer out(full_packet.data(), full_packet.size());

    for (size_t index = 0; index < QLV_MAX_SUBMIT_SYNCS; ++index) {
        write_u64(out, QLV_HEADER_BYTES + index * QLV_SYNC_POINT_BYTES, sync_token);
        write_u64(out, QLV_HEADER_BYTES + index * QLV_SYNC_POINT_BYTES + QLV_SYNC_VALUE_OFFSET, 11);
    }

    expect(snapshot_submit_with_private_fence({full_packet.data(), full_packet.size()},
        response_capacity, resources.session(), private_sync_token, 22,
        dma.data(), dma.size(), fused, written) == ExchangeStatus::invalid_range &&
        written == 0,
        "full caller sync list retains separate native lifetime-fence fallback");
}

void check_replies() {
    Resources resources;
    ValidatedExchange request;
    ExchangeReply result;
    auto bytes = ordinary_packet(QLV_MAP_BLOB);
    expect(validate(bytes, resources, request) == ExchangeStatus::validated, "prepare retained blob mapping request");
    auto reply_header = ordinary(QLV_MAP_BLOB);
    reply_header.status = QLV_STATUS_SUCCESS;
    reply_header.bytes = resources.blob.byte_count;
    reply_header.offset = QLV_APERTURE_BYTES - reply_header.bytes;
    reply_header.flags = QLV_MAP_CACHE_WRITE_COMBINED;
    auto response = packet(reply_header);
    expect(validate_exchange_reply(request, {response.data(), response.size()}, result) == ExchangeStatus::validated &&
           result.header.value == 0, "host map supplies bounded aperture offset only, never process VA");

    for (size_t count = 0; count < QLV_HEADER_BYTES; ++count) {
        expect(validate_exchange_reply(request, {response.data(), count}, result) == ExchangeStatus::short_buffer &&
               result.header.context == 0, "truncated reply clears any prior mapping result");
    }

    const auto good_map = reply_header;
    reply_header.value = 0x12340000;
    response = packet(reply_header);
    expect(validate_exchange_reply(request, {response.data(), response.size()}, result) == ExchangeStatus::invalid_range,
           "host-supplied user pointer cannot be published");
    reply_header = good_map;
    reply_header.offset += QLV_PAGE_BYTES;
    response = packet(reply_header);
    expect(validate_exchange_reply(request, {response.data(), response.size()}, result) == ExchangeStatus::invalid_range,
           "map end bounded by actual assigned aperture");
    reply_header = good_map;
    reply_header.bytes = 4096;
    response = packet(reply_header);
    expect(validate_exchange_reply(request, {response.data(), response.size()}, result) == ExchangeStatus::invalid_range,
           "host map must match the retained allocation size");
    reply_header = good_map;
    reply_header.flags = 3;
    response = packet(reply_header);
    expect(validate_exchange_reply(request, {response.data(), response.size()}, result) == ExchangeStatus::invalid_range,
           "unknown cache attributes cannot reach native mapping");

    for (const uint16_t operation : {uint16_t{QLV_CREATE_BLOB}, uint16_t{QLV_CREATE_SYNC}}) {
        bytes = ordinary_packet(operation);
        validate(bytes, resources, request);
        reply_header = ordinary(operation);
        reply_header.status = QLV_STATUS_SUCCESS;
        reply_header.resource = 0;
        response = packet(reply_header);
        expect(validate_exchange_reply(request, {response.data(), response.size()}, result) == ExchangeStatus::malformed,
               "zero created token is not success");
        reply_header.resource = blob_token;
        response = packet(reply_header);
        expect(validate_exchange_reply(request, {response.data(), response.size()}, result) == ExchangeStatus::malformed,
               "created token cannot alias an already retained resource of either kind");
        reply_header.resource = 0x900000009ULL;
        response = packet(reply_header);
        expect(validate_exchange_reply(request, {response.data(), response.size()}, result) == ExchangeStatus::validated &&
               result.header.resource == reply_header.resource, "new64-bit host token preserved for native transactional registration");
    }

    bytes = ordinary_packet(QLV_WAIT_SYNCS);
    validate(bytes, resources, request);
    reply_header = header(QLV_WAIT_SYNCS);
    reply_header.status = QLV_STATUS_TIMEOUT;
    response = packet(reply_header);
    expect(validate_exchange_reply(request, {response.data(), response.size()}, result) == ExchangeStatus::host_error &&
           result.header.status == QLV_STATUS_TIMEOUT, "legitimate immediate wait timeout is not malformed/device loss");
    reply_header.payload_bytes = 4;
    response = packet(reply_header);
    expect(validate_exchange_reply(request, {response.data(), response.size()}, result) == ExchangeStatus::malformed,
           "host error cannot carry unvalidated payload");

    bytes = ordinary_packet(QLV_SUBMIT);
    resources.sync.pending_uses = 7;
    validate(bytes, resources, request);
    reply_header = header(QLV_SUBMIT);
    reply_header.status = QLV_STATUS_SUCCESS;
    response = packet(reply_header);
    expect(validate_exchange_reply(request, {response.data(), response.size()}, result) == ExchangeStatus::validated &&
           resources.sync.pending_uses == 7, "queue acceptance never retires resource completion leases");

    bytes = ordinary_packet(QLV_OPEN);
    validate(bytes, resources, request);
    reply_header = header(QLV_OPEN);
    reply_header.context = context_token;
    reply_header.status = QLV_STATUS_SUCCESS;
    reply_header.flags = QLV_HOST_APERTURE_SIZE;
    reply_header.bytes = QLV_APERTURE_BYTES * 2;
    response = packet(reply_header);
    expect(validate_exchange_reply(request, {response.data(), response.size()}, result) == ExchangeStatus::invalid_range,
           "OPEN host aperture cannot exceed assigned PCI resource");
    reply_header.bytes = QLV_APERTURE_BYTES;
    response = packet(reply_header);
    expect(validate_exchange_reply(request, {response.data(), response.size()}, result) == ExchangeStatus::validated,
           "OPEN host geometry fits assigned aperture");
}

void check_mapping_ownership() {
    const ApertureMapping first{context_token, blob_token, 0, 8192, QLV_MAP_CACHE_CACHED};
    auto other = first;
    expect(mapping_relation(first, other, QLV_APERTURE_BYTES) == MappingRelation::identical,
           "same owner/resource/geometry/cache may reuse existing mapping");
    other.context = context_token + 1;
    expect(mapping_relation(first, other, QLV_APERTURE_BYTES) == MappingRelation::conflict,
           "same aperture bytes cannot alias between different file contexts");
    other = first;
    other.resource = blob_token + 1;
    other.offset = 4096;
    expect(mapping_relation(first, other, QLV_APERTURE_BYTES) == MappingRelation::conflict,
           "partial overlap between different resources rejected");
    other.offset = 8192;
    expect(mapping_relation(first, other, QLV_APERTURE_BYTES) == MappingRelation::disjoint,
           "adjacent complete mappings do not conflict");
    other = first;
    other.cache_mode = QLV_MAP_CACHE_WRITE_COMBINED;
    expect(mapping_relation(first, other, QLV_APERTURE_BYTES) == MappingRelation::conflict,
           "same resource cannot acquire conflicting CPU cache aliases");
    other.offset = UINT64_MAX - 4095;
    expect(mapping_relation(first, other, QLV_APERTURE_BYTES) == MappingRelation::invalid,
           "aperture end overflow rejected before overlap sums");
    expect(valid_user_mapping(0x7ffee000, 8192, 0x7ffeffff, 32),
           "exact NT5 user limit accepts complete x86 map");
    expect(!valid_user_mapping(0x7ffef000, 8192, 0x7ffeffff, 32),
           "user view cannot cross supplied process user boundary");
    expect(!valid_user_mapping(0x100000000ULL, 4096, UINT32_MAX, 32),
           "high userVA bits cannot truncate into32-bit client pointer");
    expect(!valid_user_mapping(0x1000, 4096, UINT64_MAX, 32),
           "native32-bit owner cannot supply an incompatible wide user limit");
    expect(!valid_user_mapping(0, 4096, UINT32_MAX, 32), "null mapping is not successful OS allocation");
    expect(!valid_user_mapping(0x1001, 4096, UINT32_MAX, 32), "unaligned view cannot be aperture MDL mapping");
    expect(valid_user_mapping(0x7fffffffe000ULL, 8192, 0x7fffffffffffULL, 64),
           "shared policy preserves64-bit process pointer bounds");
}

void check_native_reply_framing() {
    ExchangeReply reply;
    reply.header = header(QLV_WAIT_SYNCS);
    reply.header.status = QLV_STATUS_TIMEOUT;
    reply.header.value = UINT64_C(0x1122334455667788);
    std::array<uint8_t, response_capacity + 8> output;
    output.fill(0xa5);
    size_t written = 123;
    expect(encode_exchange_reply(reply, {output.data(), output.size()}, written) == ExchangeStatus::validated &&
           written == QLV_HEADER_BYTES, "normal WAIT timeout is encoded as a complete wire reply");
    const auto expected = packet(reply.header);
    expect(std::equal(expected.begin(), expected.end(), output.begin()), "reply encoding matches generated header offsets exactly");
    expect(output[QLV_HEADER_BYTES] == 0xa5, "header-only reply never publishes capacity tail");
    output.fill(0xa5);
    expect(encode_exchange_reply(reply, {output.data(), QLV_HEADER_BYTES - 1}, written) == ExchangeStatus::short_buffer &&
           written == 0 && output[0] == 0xa5, "short output remains unchanged and publishes no bytes");
    reply.header.status = -1;
    expect(encode_exchange_reply(reply, {output.data(), output.size()}, written) == ExchangeStatus::malformed &&
           written == 0 && output[0] == 0xa5, "negative native error cannot masquerade as wire status");
    reply.header = header(QLV_GET_CAPS);
    reply.header.status = QLV_STATUS_SUCCESS;
    reply.header.payload_bytes = QLV_CAPS_BYTES;
    reply.caps.wire_format_version = 1;
    reply.caps.vk_xml_version = 4210995;
    reply.caps.supports_blob_id_0 = 1;
    reply.caps.extension_mask[QLV_EXTENSION_MASK_WORDS - 1] = 0x89abcdef;
    reply.caps.use_guest_vram = 1;
    expect(encode_exchange_reply(reply, {output.data(), output.size()}, written) == ExchangeStatus::validated &&
           written == response_capacity, "caps reply publishes exact initialized payload length");
    uint32_t word = 0;
    ByteView view(output.data(), written);
    expect(view.read_u32(68, word) && word == 4210995 && view.read_u32(208, word) && word == 0x89abcdef &&
           view.read_u32(220, word) && word == 1 && output[response_capacity] == 0xa5,
           "capset final mask and flags retain exact generated wire layout");

    for (const auto operation : {QLV_IMPORT_BLOB, QLV_WATCH_SYNCS}) {
        auto request = header(static_cast<uint16_t>(operation));
        request.payload_bytes = 16;
        request.value = UINT64_MAX;
        auto bytes = packet(request);
        expect(unsupported_exchange_reply({bytes.data(), bytes.size()}, context_token, reply) == ExchangeStatus::validated &&
               reply.header.status == QLV_STATUS_UNSUPPORTED && reply.header.payload_bytes == 0 && reply.header.value == 0,
               "optional operations return deliberate framed unsupported without dereferencing resource values");
        expect(encode_exchange_reply(reply, {output.data(), output.size()}, written) == ExchangeStatus::validated && written == 64,
               "optional unsupported reply remains a valid header-only native success");
        expect(unsupported_exchange_reply({bytes.data(), bytes.size()}, context_token + 1, reply) == ExchangeStatus::malformed,
               "optional feature rejection still enforces exact file context");
        bytes.pop_back();
        expect(unsupported_exchange_reply({bytes.data(), bytes.size()}, context_token, reply) == ExchangeStatus::malformed,
               "optional feature rejection cannot bypass exact packet framing");
    }
}

void check_completion_retirement() {
    CompletionLedger ledger;
    uint64_t serial = 777;
    uint64_t point = 888;
    expect(!ledger.prepare(QLV_MAX_TIMELINES, serial, point) && serial == 777 && point == 888,
           "invalid ring never mutates reservation outputs");
    expect(ledger.prepare(0, serial, point) && serial == 1 && point == 1 && ledger.latest() == 0,
           "preflight and queue acknowledgement alone cannot publish a resource lease");
    expect(!ledger.commit(0, serial + 1, point) && ledger.count() == 0,
           "stale or forged reservation cannot enter completion ledger");
    expect(ledger.commit(0, serial, point) && ledger.pending(serial) && ledger.retired() == 0,
           "accepted real fence retains first resource lease until observed completion");
    expect(ledger.prepare(1, serial, point) && serial == 2 && point == 1 && ledger.commit(1, serial, point),
           "independent rings use independent timeline points and global ordered serials");
    expect(ledger.observe(1, 1) && ledger.retired() == 0 && ledger.count() == 2 && ledger.pending(1),
           "later ring completion cannot retire unfinished earlier queue work");
    expect(!ledger.observe(0, 2) && !ledger.observe(QLV_MAX_TIMELINES, 0) && ledger.retired() == 0,
           "unissued or invalid ring completion fails without retiring resources");
    expect(ledger.observe(0, 1) && ledger.retired() == 2 && ledger.count() == 0 && !ledger.pending(2),
           "observed earlier fence releases the contiguous already completed prefix");
    expect(!ledger.observe(0, 0) && ledger.retired() == 2,
           "regressing host counter cannot erase confirmed completion history");

    // Repeated fills exercise the bounded circular queue across several wraps.
    for (size_t cycle = 0; cycle < 3; ++cycle) {
        for (size_t index = 0; index < CompletionLedger::capacity; ++index) {
            const size_t ring = index % QLV_MAX_TIMELINES;
            expect(ledger.prepare(ring, serial, point) && ledger.commit(ring, serial, point),
                   "every capacity slot retains its accepted real fence");
        }

        expect(ledger.count() == CompletionLedger::capacity && !ledger.prepare(0, serial, point),
               "full ledger rejects submission before hardware mutation");
        const auto before = ledger.latest();

        for (size_t ring = QLV_MAX_TIMELINES; ring != 0; --ring) {
            expect(ledger.observe(ring - 1, ledger.issued(ring - 1)), "real per-ring observations remain bounded during wrap");
        }

        expect(ledger.count() == 0 && ledger.retired() == before && !ledger.pending(before),
               "reverse ring completion retires exact serial prefix after queue wrap");
    }
}

void check_image_registration() {
    Resources resources;
    ValidatedExchange request;
    auto registration = header(QLV_REGISTER_SCANOUT);
    registration.resource = blob_token;
    registration.value = 1;
    registration.bytes = 16 | (uint64_t{8} << QLV_PACKED_WORD_SHIFT);
    registration.offset = 128 | (uint64_t{64} << QLV_PACKED_WORD_SHIFT);
    registration.flags = QLV_SCANOUT_BGRA8;
    ImageLayout layout;

    expect(image_layout(registration, 1024, layout) && layout.width == 16 &&
        layout.height == 8 && layout.stride == 128 && layout.offset == 64 &&
        layout.format == QLV_SCANOUT_BGRA8, "padded image layout includes offset and exact final row");
    expect(!image_layout(registration, 1023, layout), "image registration rejects final pixel beyond owned blob");
    expect(validate(packet(registration), resources, request) == ExchangeStatus::invalid_range,
        "image export requires an explicitly shareable owned allocation");
    resources.blob.blob_flags |= QLV_SHAREABLE;
    expect(validate(packet(registration), resources, request) == ExchangeStatus::validated,
        "image registration accepts checked shareable per-file resource");
    resources.blob.pending_uses = 1;
    expect(validate(packet(registration), resources, request) == ExchangeStatus::resource_busy,
        "image export cannot precede real producer GPU completion");
    resources.blob.pending_uses = 0;

    for (const uint64_t invalid : {uint64_t{0}, uint64_t{UINT32_MAX} + 1}) {
        auto changed = registration;
        changed.value = invalid;
        expect(validate(packet(changed), resources, request) == ExchangeStatus::malformed,
            "wire allocation marker cannot be absent or truncated");
    }

    for (const uint32_t invalid : {uint32_t{0}, uint32_t{QLV_SCANOUT_MAX_DIMENSION + 1}, UINT32_MAX}) {
        auto changed = registration;
        changed.bytes = invalid | (uint64_t{8} << QLV_PACKED_WORD_SHIFT);
        expect(!image_layout(changed, UINT64_MAX, layout), "zero or oversized image width is rejected");
        changed.bytes = 16 | (static_cast<uint64_t>(invalid) << QLV_PACKED_WORD_SHIFT);
        expect(!image_layout(changed, UINT64_MAX, layout), "zero or oversized image height is rejected");
    }

    for (const uint64_t invalid : {uint64_t{60}, uint64_t{129},
        uint64_t{128} | (uint64_t{65} << QLV_PACKED_WORD_SHIFT)}) {
        auto changed = registration;
        changed.offset = invalid;
        expect(!image_layout(changed, resources.blob.byte_count, layout),
            "short row, unaligned stride or unaligned offset cannot reach an import");
    }

    auto changed = registration;
    changed.flags = QLV_SCANOUT_RGBA8;
    expect(image_layout(changed, resources.blob.byte_count, layout), "wire layout retains supported RGBA identity");
    changed.flags = UINT32_MAX;
    expect(!image_layout(changed, UINT64_MAX, layout), "unknown image format fails closed");
    changed = registration;
    changed.resource = sync_token;
    expect(validate(packet(changed), resources, request) == ExchangeStatus::unknown_resource,
        "sync object cannot be exported as an image");
    changed.resource = blob_token + 1;
    expect(validate(packet(changed), resources, request) == ExchangeStatus::unknown_resource,
        "foreign image resource cannot cross the per-file namespace");

    expect(validate(packet(registration), resources, request) == ExchangeStatus::validated,
        "prepare exact image import reply validation");
    auto response_header = registration;
    response_header.status = QLV_STATUS_SUCCESS;
    response_header.value = 0;
    auto response = packet(response_header);
    ExchangeReply reply;
    expect(validate_exchange_reply(request, {response.data(), response.size()}, reply) == ExchangeStatus::validated,
        "actual host success clears marker and echoes exact checked image identity/layout");

    for (const size_t field : {size_t{QLV_RESOURCE_OFFSET}, size_t{QLV_VALUE_OFFSET},
        size_t{QLV_BYTES_OFFSET}, size_t{QLV_OFFSET_OFFSET}, size_t{QLV_FLAGS_OFFSET}}) {
        auto corrupted = response;
        corrupted.at(field) ^= 1;
        expect(validate_exchange_reply(request, {corrupted.data(), corrupted.size()}, reply) == ExchangeStatus::malformed,
            "changed host image identity or layout cannot publish a native lease");
    }
}

} // namespace

int main() {
    check_closure_and_snapshot();
    check_framing_and_sessions();
    check_resources_and_cache();
    check_submission_and_wait();
    check_private_submit_snapshot();
    check_replies();
    check_mapping_ownership();
    check_native_reply_framing();
    check_completion_retirement();
    check_image_registration();
    std::printf("%u Lightning exchange policy checks; %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

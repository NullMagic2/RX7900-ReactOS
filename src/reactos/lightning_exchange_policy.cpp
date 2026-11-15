/*
Module: lightning.reactos.exchange_policy
Purpose: Bound QLV renderer packets, session tokens, sync points and aperture views.
Created: 2026-10-21
Architecture: Adapted existing QEMU-lightning transport_contract/kernel_policy rules;
adds per-session resource/range checks without exposing native handles or pointers.
*/
#include "lightning_exchange_policy.hpp"

namespace lightning::reactos {
namespace {

static_assert(QLV_COMMAND_WORD_BYTES == sizeof(uint32_t), "Packet snapshot requires32-bit command words");
static_assert(QLV_SYNC_POINT_BYTES == 16 && QLV_SYNC_VALUE_OFFSET == 8,
              "Sync-point decoding requires the pinned pair of64-bit wire fields");

uint32_t u32(rx7900::ByteView bytes, size_t offset) noexcept {
    uint32_t value = 0;
    bytes.read_u32(offset, value);
    return value;
}

uint64_t u64(rx7900::ByteView bytes, size_t offset) noexcept {
    return u32(bytes, offset) | (static_cast<uint64_t>(u32(bytes, offset + 4)) << 32);
}

bool read_header(rx7900::ByteView bytes, QLV_HEADER& header) noexcept {
    if (!bytes.contains(0, QLV_HEADER_BYTES)) {
        return false;
    }

    const uint32_t version_operation = u32(bytes, QLV_VERSION_OFFSET);
    const uint32_t status = u32(bytes, QLV_STATUS_OFFSET);

    if (status > QLV_STATUS_TIMEOUT || u32(bytes, QLV_MAGIC_OFFSET) != QLV_MAGIC ||
        (version_operation & UINT16_MAX) != QLV_VERSION || u32(bytes, QLV_RESERVED_OFFSET) != 0) {
        return false;
    }

    header = {};
    header.magic = static_cast<uint32_t>(QLV_MAGIC);
    header.version = static_cast<uint16_t>(QLV_VERSION);
    header.operation = static_cast<uint16_t>(version_operation >> 16);
    header.context = u64(bytes, QLV_CONTEXT_OFFSET);
    header.resource = u64(bytes, QLV_RESOURCE_OFFSET);
    header.value = u64(bytes, QLV_VALUE_OFFSET);
    header.bytes = u64(bytes, QLV_BYTES_OFFSET);
    header.offset = u64(bytes, QLV_OFFSET_OFFSET);
    header.status = static_cast<int32_t>(status);
    header.flags = u32(bytes, QLV_FLAGS_OFFSET);
    header.payload_bytes = u32(bytes, QLV_PAYLOAD_BYTES_OFFSET);
    return true;
}

bool valid_aperture(uint64_t bytes) noexcept {
    return bytes >= QLV_APERTURE_BYTES && bytes <= QLV_MAX_APERTURE_BYTES && (bytes & (bytes - 1)) == 0;
}

bool ordinary_empty(const QLV_HEADER& header) noexcept {
    return header.payload_bytes == 0 && header.value == 0 && header.bytes == 0 &&
        header.offset == 0 && header.flags == 0;
}

bool resolve(const ExchangeSession& session, uint64_t token, ResourceKind kind,
             ResourceInfo& output) noexcept {
    output = {};

    if (token == 0 || session.lookup == nullptr ||
        !session.lookup(session.resource_owner, token, output) || output.kind != kind) {
        return false;
    }

    return kind != ResourceKind::blob || (output.byte_count != 0 && output.byte_count <= QLV_APERTURE_BYTES &&
        (output.byte_count & (QLV_PAGE_BYTES - 1)) == 0 &&
        (output.blob_flags & ~(QLV_MAPPABLE | QLV_COHERENT | QLV_SHAREABLE)) == 0 &&
        ((output.blob_flags & QLV_COHERENT) == 0 || (output.blob_flags & QLV_MAPPABLE) != 0));
}

ExchangeStatus sync_points(rx7900::ByteView packet, const ExchangeSession& session,
                           size_t count, bool submission) noexcept {
    for (size_t index = 0; index < count; ++index) {
        const size_t offset = QLV_HEADER_BYTES + index * QLV_SYNC_POINT_BYTES;
        const uint64_t token = u64(packet, offset);
        const uint64_t value = u64(packet, offset + QLV_SYNC_VALUE_OFFSET);
        ResourceInfo info;

        if (!resolve(session, token, ResourceKind::sync, info)) {
            return ExchangeStatus::unknown_resource;
        }

        if (submission && value < info.observed_sync_value) {
            return ExchangeStatus::invalid_range;
        }
    }

    return ExchangeStatus::validated;
}

bool valid_mapping(ApertureMapping mapping, uint64_t aperture) noexcept {
    return valid_aperture(aperture) && mapping.context != 0 && mapping.resource != 0 &&
        mapping.byte_count != 0 && mapping.byte_count <= QLV_APERTURE_BYTES &&
        ((mapping.offset | mapping.byte_count) & (QLV_PAGE_BYTES - 1)) == 0 &&
        mapping.offset <= aperture && mapping.byte_count <= aperture - mapping.offset &&
        (mapping.cache_mode == QLV_MAP_CACHE_CACHED || mapping.cache_mode == QLV_MAP_CACHE_WRITE_COMBINED);
}

} // namespace

ExchangeStatus validate_exchange_request(rx7900::ByteView packet, size_t capacity,
    const ExchangeSession& session, ValidatedExchange& output) noexcept {
    output = {};
    QLV_HEADER header{};

    if (!packet.contains(0, QLV_HEADER_BYTES) || packet.size() > QLV_HEADER_BYTES + QLV_MAX_PAYLOAD_BYTES ||
        capacity < QLV_HEADER_BYTES || capacity > QLV_HEADER_BYTES + QLV_MAX_CAPSET_BYTES) {
        return ExchangeStatus::short_buffer;
    }

    if (!valid_aperture(session.aperture_bytes) || !read_header(packet, header) ||
        header.status != QLV_STATUS_UNSUPPORTED ||
        header.payload_bytes != packet.size() - QLV_HEADER_BYTES) {
        return ExchangeStatus::malformed;
    }

    if ((header.operation < QLV_OPEN || header.operation > QLV_INVALIDATE) &&
        header.operation != QLV_REGISTER_SCANOUT) {
        return ExchangeStatus::unsupported_operation;
    }

    if (header.operation == QLV_OPEN ? (session.context != 0 || header.context != 0) :
        (session.context == 0 || header.context != session.context)) {
        return ExchangeStatus::wrong_context;
    }

    ResourceInfo resource;
    ExchangeStatus status = ExchangeStatus::validated;

    switch (header.operation) {
    case QLV_OPEN:
    case QLV_CLOSE:
        if (!ordinary_empty(header) || header.resource != 0) {
            return ExchangeStatus::malformed;
        }
        break;
    case QLV_GET_CAPS:
        if (header.payload_bytes != 0 || header.resource != 0 || header.value != 0 ||
            header.offset != 0 || header.flags != 0 || header.bytes != QLV_CAPS_BYTES) {
            return ExchangeStatus::malformed;
        }

        if (capacity < response_capacity) {
            return ExchangeStatus::short_buffer;
        }
        break;
    case QLV_CREATE_BLOB:
        if (session.lookup == nullptr) {
            return ExchangeStatus::unknown_resource;
        }

        if (header.payload_bytes != 0 || header.resource != 0 || header.value != 0 ||
            header.bytes == 0 || header.bytes > QLV_APERTURE_BYTES ||
            (header.bytes & (QLV_PAGE_BYTES - 1)) != 0 ||
            (header.flags & ~(QLV_MAPPABLE | QLV_COHERENT | QLV_SHAREABLE)) != 0 ||
            ((header.flags & QLV_COHERENT) != 0 && (header.flags & QLV_MAPPABLE) == 0)) {
            return ExchangeStatus::malformed;
        }
        break;
    case QLV_DESTROY_BLOB:
    case QLV_MAP_BLOB:
    case QLV_UNMAP_BLOB:
        if (!ordinary_empty(header)) {
            return ExchangeStatus::malformed;
        }

        if (!resolve(session, header.resource, ResourceKind::blob, resource)) {
            return ExchangeStatus::unknown_resource;
        }

        if (header.operation == QLV_MAP_BLOB && (resource.blob_flags & QLV_MAPPABLE) == 0) {
            return ExchangeStatus::invalid_range;
        }

        // Only destruction requires GPU quiescence. MAP and UNMAP change the CPU
        // view, never the host allocation queued GPU work reads: the native
        // owner revokes the process view before the host UNMAP, the host only
        // retires its aperture publication, and the blob lives until DESTROY.
        if (header.operation == QLV_DESTROY_BLOB && resource.pending_uses != 0) {
            return ExchangeStatus::resource_busy;
        }
        break;
    case QLV_REGISTER_SCANOUT: {
        ImageLayout layout;

        if (header.payload_bytes != 0 || header.value == 0 || header.value > UINT32_MAX) {
            return ExchangeStatus::malformed;
        }

        if (!resolve(session, header.resource, ResourceKind::blob, resource)) {
            return ExchangeStatus::unknown_resource;
        }

        if ((resource.blob_flags & QLV_SHAREABLE) == 0 ||
            !image_layout(header, resource.byte_count, layout)) {
            return ExchangeStatus::invalid_range;
        }

        if (resource.pending_uses != 0) {
            return ExchangeStatus::resource_busy;
        }
        break;
    }
    case QLV_CREATE_SYNC:
        if (session.lookup == nullptr) {
            return ExchangeStatus::unknown_resource;
        }

        if (header.payload_bytes != 0 || header.resource != 0 || header.flags != 0 ||
            header.bytes != 0 || header.offset != 0) {
            return ExchangeStatus::malformed;
        }
        break;
    case QLV_DESTROY_SYNC:
    case QLV_READ_SYNC:
    case QLV_WRITE_SYNC:
        if (header.payload_bytes != 0 || header.flags != 0 || header.bytes != 0 || header.offset != 0 ||
            (header.operation != QLV_WRITE_SYNC && header.value != 0)) {
            return ExchangeStatus::malformed;
        }

        if (!resolve(session, header.resource, ResourceKind::sync, resource)) {
            return ExchangeStatus::unknown_resource;
        }

        if ((header.operation == QLV_DESTROY_SYNC ||
             (header.operation == QLV_WRITE_SYNC && header.value == 0)) && resource.pending_uses != 0) {
            return ExchangeStatus::resource_busy;
        }

        if (header.operation == QLV_WRITE_SYNC && header.value != 0 &&
            header.value < resource.observed_sync_value) {
            return ExchangeStatus::invalid_range;
        }
        break;
    case QLV_SUBMIT:
        if (header.resource != 0 || header.flags != 0 || header.offset > QLV_MAX_SUBMIT_SYNCS ||
            header.value >= QLV_MAX_TIMELINES || header.bytes > QLV_MAX_PAYLOAD_BYTES ||
            (header.bytes & (QLV_COMMAND_WORD_BYTES - 1)) != 0 ||
            header.offset * QLV_SYNC_POINT_BYTES + header.bytes != header.payload_bytes) {
            return ExchangeStatus::malformed;
        }

        status = sync_points(packet, session, static_cast<size_t>(header.offset), true);
        break;
    case QLV_WAIT_SYNCS:
        if (header.resource != 0 || header.offset != 0 || header.value != 0 || header.bytes == 0 ||
            header.bytes > QLV_MAX_WAIT_SYNCS || (header.flags & ~QLV_WAIT_ANY) != 0 ||
            header.bytes * QLV_SYNC_POINT_BYTES != header.payload_bytes) {
            return ExchangeStatus::malformed;
        }

        status = sync_points(packet, session, static_cast<size_t>(header.bytes), false);
        break;
    case QLV_FLUSH:
    case QLV_INVALIDATE:
        if (header.payload_bytes != 0 || header.value != 0 || header.flags != 0) {
            return ExchangeStatus::malformed;
        }

        if (!resolve(session, header.resource, ResourceKind::blob, resource)) {
            return ExchangeStatus::unknown_resource;
        }

        if (header.offset > resource.byte_count || header.bytes > resource.byte_count - header.offset) {
            return ExchangeStatus::invalid_range;
        }
        break;
    default:
        return ExchangeStatus::unsupported_operation;
    }

    if (status != ExchangeStatus::validated) {
        return status;
    }

    output.header_ = header;
    output.resource_ = resource;
    output.aperture_bytes_ = session.aperture_bytes;
    output.response_capacity_ = capacity;
    output.resource_owner_ = session.resource_owner;
    output.lookup_ = session.lookup;
    output.valid_ = true;
    return ExchangeStatus::validated;
}

ExchangeStatus snapshot_exchange_request(rx7900::ByteView packet, size_t capacity,
    const ExchangeSession& session, rx7900::sdma::CommandBuffer dma,
    ValidatedExchange& output) noexcept {
    output = {};
    ValidatedExchange validated;
    const auto status = validate_exchange_request(packet, capacity, session, validated);

    if (status != ExchangeStatus::validated) {
        return status;
    }

    if (dma.size() < packet.size()) {
        return ExchangeStatus::short_buffer;
    }

    // Every allowed payload is a whole command word or sync record; validation
    // above proves four-byte granularity and both complete source/destination spans.
    for (size_t offset = 0; offset < packet.size(); offset += QLV_COMMAND_WORD_BYTES) {
        dma.write_u32(offset, u32(packet, offset));
    }

    output = validated;
    return ExchangeStatus::validated;
}

ExchangeStatus snapshot_submit_with_private_fence(rx7900::ByteView packet,
    size_t capacity, const ExchangeSession& session, uint64_t sync,
    uint64_t point, uint8_t* dma_storage, size_t dma_capacity,
    ValidatedExchange& output, size_t& bytes_written) noexcept {
    output = {};
    bytes_written = 0;
    ValidatedExchange caller;
    const auto status = validate_exchange_request(packet, capacity, session, caller);

    if (status != ExchangeStatus::validated) {
        return status;
    }

    const auto& header = caller.header();

    if (header.operation != QLV_SUBMIT || sync == 0 || point == 0 ||
        header.offset >= QLV_MAX_SUBMIT_SYNCS ||
        header.payload_bytes > QLV_MAX_PAYLOAD_BYTES - QLV_SYNC_POINT_BYTES) {
        return ExchangeStatus::invalid_range;
    }

    const size_t expanded = packet.size() + static_cast<size_t>(QLV_SYNC_POINT_BYTES);

    if (dma_storage == nullptr || dma_capacity < expanded) {
        return ExchangeStatus::short_buffer;
    }

    // Both source and DMA spans are private, disjoint and four-byte aligned
    // by validate_exchange_request. Copy the caller's sync prefix unchanged,
    // insert the private point, then shift only its command words.
    rx7900::sdma::CommandBuffer dma(dma_storage, expanded);
    const size_t prefix = QLV_HEADER_BYTES +
        static_cast<size_t>(header.offset) * static_cast<size_t>(QLV_SYNC_POINT_BYTES);

    for (size_t offset = 0; offset < prefix; offset += QLV_COMMAND_WORD_BYTES) {
        uint32_t word = 0;

        if (!packet.read_u32(offset, word) || !dma.write_u32(offset, word)) {
            return ExchangeStatus::short_buffer;
        }
    }

    const uint32_t sync_low = static_cast<uint32_t>(sync);
    const uint32_t point_low = static_cast<uint32_t>(point);

    if (!dma.write_u32(prefix, sync_low) ||
        !dma.write_u32(prefix + 4, static_cast<uint32_t>(sync >> 32)) ||
        !dma.write_u32(prefix + QLV_SYNC_VALUE_OFFSET, point_low) ||
        !dma.write_u32(prefix + QLV_SYNC_VALUE_OFFSET + 4, static_cast<uint32_t>(point >> 32))) {
        return ExchangeStatus::short_buffer;
    }

    for (size_t offset = prefix; offset < packet.size(); offset += QLV_COMMAND_WORD_BYTES) {
        uint32_t word = 0;

        if (!packet.read_u32(offset, word) ||
            !dma.write_u32(offset + QLV_SYNC_POINT_BYTES, word)) {
            return ExchangeStatus::short_buffer;
        }
    }

    if (!dma.write_u32(QLV_OFFSET_OFFSET, static_cast<uint32_t>(header.offset + 1)) ||
        !dma.write_u32(QLV_OFFSET_OFFSET + 4, 0) ||
        !dma.write_u32(QLV_PAYLOAD_BYTES_OFFSET,
            header.payload_bytes + static_cast<uint32_t>(QLV_SYNC_POINT_BYTES))) {
        return ExchangeStatus::short_buffer;
    }

    const auto fused = validate_exchange_request({dma_storage, expanded}, capacity, session, output);

    if (fused == ExchangeStatus::validated) {
        bytes_written = expanded;
    }

    return fused;
}

ExchangeStatus validate_exchange_reply(const ValidatedExchange& request,
    rx7900::ByteView packet, ExchangeReply& output) noexcept {
    output = {};
    QLV_HEADER reply{};

    if (!request.valid_ || !packet.contains(0, QLV_HEADER_BYTES) ||
        packet.size() > request.response_capacity_) {
        return ExchangeStatus::short_buffer;
    }

    if (!read_header(packet, reply) || reply.operation != request.header_.operation ||
        reply.payload_bytes != packet.size() - QLV_HEADER_BYTES ||
        (reply.operation != QLV_OPEN && reply.context != request.header_.context)) {
        return ExchangeStatus::malformed;
    }

    if (reply.status != QLV_STATUS_SUCCESS) {
        if (reply.payload_bytes != 0) {
            return ExchangeStatus::malformed;
        }

        output.header = reply;
        return ExchangeStatus::host_error;
    }

    if (reply.operation == QLV_OPEN || reply.operation == QLV_CLOSE || reply.operation == QLV_GET_CAPS) {
        Reply basic;

        if (decode_reply(reply.operation, request.header_.context, packet, basic) != ProtocolStatus::valid) {
            return ExchangeStatus::malformed;
        }

        if (reply.operation == QLV_OPEN && (reply.flags & QLV_HOST_APERTURE_SIZE) != 0 &&
            (!valid_aperture(reply.bytes) || reply.bytes > request.aperture_bytes_)) {
            return ExchangeStatus::invalid_range;
        }

        output.caps = basic.caps;
    } else if (reply.payload_bytes != 0) {
        return ExchangeStatus::malformed;
    } else if (reply.operation == QLV_CREATE_BLOB || reply.operation == QLV_CREATE_SYNC) {
        ResourceInfo collision;

        if (reply.resource == 0 || request.lookup_ == nullptr ||
            request.lookup_(request.resource_owner_, reply.resource, collision)) {
            return ExchangeStatus::malformed;
        }
    } else if (reply.resource != request.header_.resource) {
        return ExchangeStatus::malformed;
    } else if (reply.operation == QLV_REGISTER_SCANOUT) {
        // The existing host clears the request's legacy allocation marker.
        // Native code publishes its own opaque process lease after validation.
        if (reply.value != 0 || reply.bytes != request.header_.bytes ||
            reply.offset != request.header_.offset || reply.flags != request.header_.flags) {
            return ExchangeStatus::malformed;
        }
    } else if (reply.operation == QLV_MAP_BLOB) {
        const ApertureMapping mapping{reply.context, reply.resource, reply.offset, reply.bytes, reply.flags};

        if (reply.value != 0 || !valid_mapping(mapping, request.aperture_bytes_) ||
            reply.bytes != request.resource_.byte_count) {
            return ExchangeStatus::invalid_range;
        }
    }

    output.header = reply;
    return ExchangeStatus::validated;
}

MappingRelation mapping_relation(ApertureMapping candidate, ApertureMapping existing,
                                  uint64_t aperture) noexcept {
    if (!valid_mapping(candidate, aperture) || !valid_mapping(existing, aperture)) {
        return MappingRelation::invalid;
    }

    if (candidate.context == existing.context && candidate.resource == existing.resource) {
        return candidate.offset == existing.offset && candidate.byte_count == existing.byte_count &&
            candidate.cache_mode == existing.cache_mode ? MappingRelation::identical : MappingRelation::conflict;
    }

    return candidate.offset < existing.offset + existing.byte_count &&
        existing.offset < candidate.offset + candidate.byte_count ? MappingRelation::conflict : MappingRelation::disjoint;
}

ExchangeStatus encode_exchange_reply(const ExchangeReply& reply,
    rx7900::sdma::CommandBuffer output, size_t& written) noexcept {
    written = 0;
    const auto& h = reply.header;

    if (h.magic != QLV_MAGIC || h.version != QLV_VERSION || h.reserved != 0 ||
        h.operation < QLV_OPEN || h.operation > QLV_WATCH_SYNCS || h.status < 0 ||
        static_cast<uint32_t>(h.status) > QLV_STATUS_TIMEOUT ||
        (h.payload_bytes != 0 && (h.payload_bytes != QLV_CAPS_BYTES ||
            h.operation != QLV_GET_CAPS || h.status != QLV_STATUS_SUCCESS))) {
        return ExchangeStatus::malformed;
    }

    if (output.size() < QLV_HEADER_BYTES + h.payload_bytes) {
        return ExchangeStatus::short_buffer;
    }

    output.write_u32(0, h.magic);
    output.write_u32(4, h.version | (static_cast<uint32_t>(h.operation) << 16));
    const uint64_t fields[]{h.context, h.resource, h.value, h.bytes, h.offset};

    for (size_t index = 0; index < 5; ++index) {
        output.write_u32(8 + index * 8, static_cast<uint32_t>(fields[index]));
        output.write_u32(12 + index * 8, static_cast<uint32_t>(fields[index] >> 32));
    }

    output.write_u32(48, static_cast<uint32_t>(h.status));
    output.write_u32(52, h.flags);
    output.write_u32(56, h.payload_bytes);
    output.write_u32(60, 0);

    if (h.payload_bytes != 0) {
        const auto& caps = reply.caps;
        output.write_u32(64, caps.wire_format_version);
        output.write_u32(68, caps.vk_xml_version);
        output.write_u32(72, caps.command_serialization_spec_version);
        output.write_u32(76, caps.venus_protocol_spec_version);
        output.write_u32(80, caps.supports_blob_id_0);

        for (size_t index = 0; index < QLV_EXTENSION_MASK_WORDS; ++index) {
            output.write_u32(84 + index * 4, caps.extension_mask[index]);
        }

        output.write_u32(212, caps.allow_vk_wait_syncs);
        output.write_u32(216, caps.supports_multiple_timelines);
        output.write_u32(220, caps.use_guest_vram);
    }

    written = QLV_HEADER_BYTES + h.payload_bytes;
    return ExchangeStatus::validated;
}

ExchangeStatus unsupported_exchange_reply(rx7900::ByteView packet, uint64_t context,
                                           ExchangeReply& output) noexcept {
    output = {};
    QLV_HEADER h{};

    if (packet.size() > QLV_HEADER_BYTES + QLV_MAX_PAYLOAD_BYTES || !read_header(packet, h) ||
        h.status != QLV_STATUS_UNSUPPORTED || h.payload_bytes != packet.size() - QLV_HEADER_BYTES ||
        context == 0 || h.context != context ||
        (h.operation != QLV_IMPORT_BLOB && h.operation != QLV_WATCH_SYNCS)) {
        return ExchangeStatus::malformed;
    }

    h.status = QLV_STATUS_UNSUPPORTED;
    h.payload_bytes = 0;
    h.value = 0;
    output.header = h;
    return ExchangeStatus::validated;
}

bool valid_user_mapping(uint64_t address, uint64_t bytes, uint64_t limit,
                        uint32_t pointer_bits) noexcept {
    if ((pointer_bits != 32 && pointer_bits != 64) ||
        (pointer_bits == 32 && limit > UINT32_MAX)) {
        return false;
    }

    return address != 0 && bytes != 0 && bytes <= QLV_APERTURE_BYTES &&
        ((address | bytes) & (QLV_PAGE_BYTES - 1)) == 0 && address <= limit && bytes - 1 <= limit - address;
}

} // namespace lightning::reactos

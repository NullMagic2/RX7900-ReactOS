/*
Module: lightning.reactos.protocol
Purpose: Encode only capability-session requests and validate exact QLV replies.
Created: 2026-10-21
Architecture: Reuses generated constants and project checked byte readers/writers.
*/
#include "lightning_protocol.hpp"

namespace lightning::reactos {
namespace {

bool supported(uint16_t operation) noexcept {
    return operation == QLV_OPEN || operation == QLV_GET_CAPS || operation == QLV_CLOSE;
}

uint32_t u32(rx7900::ByteView view, size_t offset) noexcept {
    uint32_t result = 0;
    view.read_u32(offset, result);
    return result;
}

uint64_t u64(rx7900::ByteView view, size_t offset) noexcept {
    return u32(view, offset) | (static_cast<uint64_t>(u32(view, offset + 4)) << 32);
}

} // namespace

ProtocolStatus encode_request(uint16_t operation, uint64_t context,
                              rx7900::sdma::CommandBuffer output) noexcept {
    if (!supported(operation)) {
        return ProtocolStatus::invalid_operation;
    }

    if ((operation == QLV_OPEN) != (context == 0)) {
        return ProtocolStatus::invalid_context;
    }

    if (output.size() < QLV_HEADER_BYTES) {
        return ProtocolStatus::short_buffer;
    }

    for (size_t offset = 0; offset < QLV_HEADER_BYTES; offset += 4) {
        output.write_u32(offset, 0);
    }

    output.write_u32(QLV_MAGIC_OFFSET, static_cast<uint32_t>(QLV_MAGIC));
    output.write_u32(QLV_VERSION_OFFSET, static_cast<uint32_t>(QLV_VERSION) |
        (static_cast<uint32_t>(operation) << 16));
    output.write_u32(QLV_CONTEXT_OFFSET, static_cast<uint32_t>(context));
    output.write_u32(QLV_CONTEXT_OFFSET + 4, static_cast<uint32_t>(context >> 32));
    output.write_u32(QLV_STATUS_OFFSET, static_cast<uint32_t>(QLV_STATUS_UNSUPPORTED));

    if (operation == QLV_GET_CAPS) {
        output.write_u32(QLV_BYTES_OFFSET, static_cast<uint32_t>(QLV_CAPS_BYTES));
    }

    return ProtocolStatus::valid;
}

ProtocolStatus decode_reply(uint16_t operation, uint64_t context,
                            rx7900::ByteView bytes, Reply& output) noexcept {
    output = {};

    if (!supported(operation)) {
        return ProtocolStatus::invalid_operation;
    }

    if ((operation == QLV_OPEN) != (context == 0)) {
        return ProtocolStatus::invalid_context;
    }

    if (!bytes.contains(0, QLV_HEADER_BYTES) || bytes.size() > response_capacity) {
        return ProtocolStatus::short_buffer;
    }

    const uint32_t framing = u32(bytes, QLV_VERSION_OFFSET);
    const uint32_t status = u32(bytes, QLV_STATUS_OFFSET);
    const uint32_t payload = u32(bytes, QLV_PAYLOAD_BYTES_OFFSET);
    const uint64_t reply_context = u64(bytes, QLV_CONTEXT_OFFSET);

    if (u32(bytes, QLV_MAGIC_OFFSET) != QLV_MAGIC ||
        framing != (QLV_VERSION | (static_cast<uint32_t>(operation) << 16)) ||
        u32(bytes, QLV_RESERVED_OFFSET) != 0 || status > QLV_STATUS_TIMEOUT ||
        bytes.size() - QLV_HEADER_BYTES != payload ||
        (operation != QLV_OPEN && reply_context != context)) {
        return ProtocolStatus::malformed;
    }

    if (status != QLV_STATUS_SUCCESS) {
        if (payload != 0) {
            return ProtocolStatus::malformed;
        }

        output.host_status = status;
        return ProtocolStatus::host_error;
    }

    if ((operation == QLV_OPEN && (reply_context == 0 || payload != 0)) ||
        (operation == QLV_CLOSE && (payload != 0 || u64(bytes, QLV_RESOURCE_OFFSET) != 0)) ||
        (operation == QLV_GET_CAPS && payload != QLV_CAPS_BYTES)) {
        return ProtocolStatus::malformed;
    }

    Reply decoded{};
    decoded.context = reply_context;

    if (operation == QLV_GET_CAPS) {
        constexpr size_t cap = QLV_HEADER_BYTES;
        decoded.caps.wire_format_version = u32(bytes, cap);
        decoded.caps.vk_xml_version = u32(bytes, cap + 4);
        decoded.caps.command_serialization_spec_version = u32(bytes, cap + 8);
        decoded.caps.venus_protocol_spec_version = u32(bytes, cap + 12);
        decoded.caps.supports_blob_id_0 = u32(bytes, cap + 16);

        for (size_t index = 0; index < QLV_EXTENSION_MASK_WORDS; ++index) {
            decoded.caps.extension_mask[index] = u32(bytes, cap + 20 + index * 4);
        }

        decoded.caps.allow_vk_wait_syncs = u32(bytes, cap + 148);
        decoded.caps.supports_multiple_timelines = u32(bytes, cap + 152);
        decoded.caps.use_guest_vram = u32(bytes, cap + 156);
    }

    output = decoded;
    return ProtocolStatus::valid;
}

} // namespace lightning::reactos

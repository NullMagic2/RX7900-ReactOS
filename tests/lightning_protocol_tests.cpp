/*
Module: lightning.tests.protocol
Purpose: Verify the exact QLV capability-session wire bytes and malformed replies.
Created: 2026-10-21
Architecture: Synthetic CPU-only buffers; no native DMA or rendering is simulated.
*/
#include <array>
#include <cstdio>
#include "lightning_protocol.hpp"

namespace {

using namespace lightning::reactos;
using rx7900::ByteView;
using rx7900::sdma::CommandBuffer;
unsigned checks = 0;
unsigned failures = 0;

void expect(bool condition, const char* description) {
    ++checks;

    if (!condition) {
        ++failures;
        std::fprintf(stderr, "FAIL: %s\n", description);
    }
}

uint32_t u32(ByteView bytes, size_t offset) {
    uint32_t value = 0;
    bytes.read_u32(offset, value);
    return value;
}

std::array<uint8_t, response_capacity> reply(uint16_t operation, uint64_t context) {
    std::array<uint8_t, response_capacity> bytes{};
    CommandBuffer out(bytes.data(), bytes.size());
    encode_request(operation, operation == QLV_OPEN ? 0 : context, out);
    out.write_u32(QLV_CONTEXT_OFFSET, static_cast<uint32_t>(context));
    out.write_u32(QLV_CONTEXT_OFFSET + 4, static_cast<uint32_t>(context >> 32));
    out.write_u32(QLV_STATUS_OFFSET, static_cast<uint32_t>(QLV_STATUS_SUCCESS));

    if (operation == QLV_GET_CAPS) {
        out.write_u32(QLV_PAYLOAD_BYTES_OFFSET, static_cast<uint32_t>(QLV_CAPS_BYTES));

        for (size_t offset = 0; offset < QLV_CAPS_BYTES; offset += 4) {
            out.write_u32(QLV_HEADER_BYTES + offset, static_cast<uint32_t>(offset + 1));
        }
    }

    return bytes;
}

void check_encoding() {
    std::array<uint8_t, 68> bytes{};
    const uint16_t operations[]{QLV_OPEN, QLV_GET_CAPS, QLV_CLOSE};
    constexpr uint64_t context = 0xfedcba9876543210ULL;

    for (const auto operation : operations) {
        bytes.fill(0xa5);
        expect(encode_request(operation, operation == QLV_OPEN ? 0 : context,
               {bytes.data(), bytes.size()}) == ProtocolStatus::valid, "encode allowed session operation");
        ByteView encoded(bytes.data(), bytes.size());
        expect(u32(encoded, 0) == 0x4e564c51 && u32(encoded, 4) == (1u | (operation << 16)),
               "exact pinned magic/version/operation words");
        expect(u32(encoded, 8) == (operation == QLV_OPEN ? 0 : 0x76543210) &&
               u32(encoded, 12) == (operation == QLV_OPEN ? 0 : 0xfedcba98),
               "context high bits preserved on32-bit target");
        expect(u32(encoded, 32) == (operation == QLV_GET_CAPS ? 160 : 0) &&
               u32(encoded, 48) == QLV_STATUS_UNSUPPORTED && u32(encoded, 60) == 0,
               "exact request dimensions/status/reserved fields");

        for (size_t offset = 64; offset < bytes.size(); ++offset) {
            expect(bytes.at(offset) == 0xa5, "encoder does not write beyond fixed64-byte request");
        }
    }

    for (size_t capacity = 0; capacity < QLV_HEADER_BYTES; ++capacity) {
        bytes.fill(0xa5);
        expect(encode_request(QLV_OPEN, 0, {bytes.data(), capacity}) == ProtocolStatus::short_buffer,
               "every insufficient request capacity rejected before writing");
        bool unchanged = true;

        for (const auto byte : bytes) {
            unchanged = unchanged && byte == 0xa5;
        }

        expect(unchanged, "request failure preserves complete output");
    }

    expect(encode_request(QLV_OPEN, 1, {bytes.data(), bytes.size()}) == ProtocolStatus::invalid_context,
           "OPEN may not adopt a caller context");
    expect(encode_request(QLV_CLOSE, 0, {bytes.data(), bytes.size()}) == ProtocolStatus::invalid_context,
           "CLOSE needs nonzero owned context");
    expect(encode_request(QLV_GET_CAPS, 0, {bytes.data(), bytes.size()}) == ProtocolStatus::invalid_context,
           "GET_CAPS cannot run without an owned session");

    for (const uint16_t operation : {uint16_t{0}, uint16_t{4}, uint16_t{8}, uint16_t{17}, uint16_t{UINT16_MAX}}) {
        expect(encode_request(operation, 1, {bytes.data(), bytes.size()}) == ProtocolStatus::invalid_operation,
               "raw rendering/mapping/unknown operations have no request API");
    }

    expect(encode_request(QLV_OPEN, 0, {nullptr, SIZE_MAX}) == ProtocolStatus::short_buffer,
           "null request output never becomes writable by forged capacity");
}

void check_replies() {
    constexpr uint64_t context = 0x1234567800000000ULL;
    auto bytes = reply(QLV_OPEN, context);
    Reply decoded{};
    expect(decode_reply(QLV_OPEN, 0, {bytes.data(), 64}, decoded) == ProtocolStatus::valid &&
           decoded.context == context, "OPEN preserves real nonzero64-bit logical context");
    bytes = reply(QLV_CLOSE, context);
    expect(decode_reply(QLV_CLOSE, context, {bytes.data(), 64}, decoded) == ProtocolStatus::valid,
           "CLOSE validates matching real context");
    bytes = reply(QLV_GET_CAPS, context);
    expect(decode_reply(QLV_GET_CAPS, context, {bytes.data(), bytes.size()}, decoded) == ProtocolStatus::valid &&
           decoded.caps.wire_format_version == 1 && decoded.caps.vk_xml_version == 5 &&
           decoded.caps.extension_mask[0] == 21 && decoded.caps.extension_mask[31] == 145 &&
           decoded.caps.allow_vk_wait_syncs == 149 && decoded.caps.supports_multiple_timelines == 153 &&
           decoded.caps.use_guest_vram == 157, "capset fields copied exactly without invented support");

    for (size_t count = 0; count < bytes.size(); ++count) {
        decoded.context = UINT64_MAX;
        decoded.caps.vk_xml_version = UINT32_MAX;
        expect(decode_reply(QLV_GET_CAPS, context, {bytes.data(), count}, decoded) != ProtocolStatus::valid &&
               decoded.context == 0 && decoded.caps.vk_xml_version == 0,
               "every truncated capset rejected and old result cleared");
    }

    std::array<uint8_t, response_capacity + 2> unaligned{};

    for (size_t offset = 0; offset < bytes.size(); ++offset) {
        unaligned.at(offset + 1) = bytes.at(offset);
    }

    expect(decode_reply(QLV_GET_CAPS, context, {unaligned.data() + 1, bytes.size()}, decoded) ==
           ProtocolStatus::valid, "unaligned wire snapshot is decoded without native overlays");
    expect(decode_reply(QLV_GET_CAPS, context, {unaligned.data(), unaligned.size()}, decoded) ==
           ProtocolStatus::short_buffer, "reply count cannot exceed fixed common-buffer capacity");
    expect(decode_reply(QLV_GET_CAPS, context, {nullptr, SIZE_MAX}, decoded) ==
           ProtocolStatus::short_buffer, "null response cannot acquire readable bytes");

    for (const size_t field : {size_t{0}, size_t{4}, size_t{6}, size_t{8}, size_t{12}, size_t{56}, size_t{60}}) {
        bytes = reply(QLV_GET_CAPS, context);
        bytes.at(field) ^= 1;
        expect(decode_reply(QLV_GET_CAPS, context, {bytes.data(), bytes.size()}, decoded) ==
               ProtocolStatus::malformed, "wrong magic/version/operation/context/extent/reserved rejected");
    }

    bytes = reply(QLV_OPEN, 0);
    expect(decode_reply(QLV_OPEN, 0, {bytes.data(), 64}, decoded) == ProtocolStatus::malformed,
           "zero host context never becomes OPEN success");
    bytes = reply(QLV_CLOSE, context);
    CommandBuffer out(bytes.data(), bytes.size());
    out.write_u32(QLV_RESOURCE_OFFSET, 1);
    expect(decode_reply(QLV_CLOSE, context, {bytes.data(), 64}, decoded) == ProtocolStatus::malformed,
           "CLOSE cannot acknowledge an unrelated resource");
}

void check_host_errors() {
    Reply decoded;
    constexpr uint64_t context = 0x100000000ULL;

    for (const uint16_t operation : {uint16_t{QLV_OPEN}, uint16_t{QLV_CLOSE}, uint16_t{QLV_GET_CAPS}}) {
        auto bytes = reply(operation, context);
        CommandBuffer out(bytes.data(), bytes.size());
        out.write_u32(QLV_PAYLOAD_BYTES_OFFSET, 0);

        for (uint32_t status = 1; status <= QLV_STATUS_TIMEOUT; ++status) {
            out.write_u32(QLV_STATUS_OFFSET, status);
            expect(decode_reply(operation, operation == QLV_OPEN ? 0 : context,
                   {bytes.data(), 64}, decoded) == ProtocolStatus::host_error &&
                   decoded.host_status == status && decoded.context == 0,
                   "real host error is distinct from successful session/caps");
        }

        out.write_u32(QLV_STATUS_OFFSET, UINT32_MAX);
        expect(decode_reply(operation, operation == QLV_OPEN ? 0 : context,
               {bytes.data(), 64}, decoded) == ProtocolStatus::malformed,
               "unknown or negative wire status rejected");
        out.write_u32(QLV_STATUS_OFFSET, 1);
        out.write_u32(QLV_PAYLOAD_BYTES_OFFSET, 1);
        expect(decode_reply(operation, operation == QLV_OPEN ? 0 : context,
               {bytes.data(), 65}, decoded) == ProtocolStatus::malformed,
               "host errors may not smuggle response payload");
    }
}

} // namespace

int main() {
    check_encoding();
    check_replies();
    check_host_errors();
    std::printf("%u Lightning protocol checks; %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

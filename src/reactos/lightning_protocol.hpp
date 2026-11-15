/*
Module: lightning.reactos.protocol
Purpose: Bind bounded NT5 capability queries to the existing QLV wire contract.
Created: 2026-10-21
Architecture: Pure C++ bytes/ABI only; native ownership lives in lightning_transport.
*/
#pragma once

#include "byte_view.hpp"
#include "sdma.hpp"
#include "lightning_abi.hpp"

namespace lightning::reactos {

constexpr size_t response_capacity = QLV_HEADER_BYTES + QLV_CAPS_BYTES;

enum class ProtocolStatus { valid, invalid_operation, invalid_context, short_buffer, malformed, host_error };

struct Reply final {
    uint64_t context = 0;
    uint32_t host_status = 0;
    QLV_CAPS caps{};
};

// Resident, private immutable request/reply snapshots only. Encoder accepts just
// OPEN/GET_CAPS/CLOSE and writes exactly64 bytes. Output remains unchanged on
// failure. Replies are decoded without structure overlays or native alignment.
ProtocolStatus encode_request(uint16_t operation, uint64_t context,
                              rx7900::sdma::CommandBuffer output) noexcept;
ProtocolStatus decode_reply(uint16_t operation, uint64_t context,
                            rx7900::ByteView bytes, Reply& output) noexcept;

} // namespace lightning::reactos

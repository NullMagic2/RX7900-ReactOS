/*
Module: rx7900.tests.gpu_process_abi
Purpose: Check the actual native process request boundary and capability fields.
Created: 2026-10-22
Architecture: Uses the production decoder consumed by VideoPort, without NT APIs.
*/
#include "../src/reactos/gpu_process_abi.hpp"
#include <cstddef>
#include <initializer_list>
#include <cstdio>

namespace {
using namespace rx7900::reactos::process_abi;
int checks = 0;
int failures = 0;
void check(bool passed) {
    ++checks;
    if (!passed) ++failures;
}
void put(uint8_t* bytes, uint32_t offset, uint64_t value, uint32_t count) {
    for (uint32_t i = 0; i < count; ++i) bytes[offset + i] = static_cast<uint8_t>(value >> (i * 8));
}
Request base(Operation op) {
    Request request{};
    request.size = sizeof(request);
    request.abi_version = version;
    request.operation = static_cast<uint32_t>(op);
    return request;
}
}
int main() {
    uint8_t bytes[sizeof(Request)]{};
    put(bytes, 0, sizeof(Request), 4);
    put(bytes, 4, version, 4);
    Request request{};
    for (uint32_t size = 0; size < sizeof(Request); ++size)
        check(!decode_request(rx7900::ByteView(bytes, size), request));
    check(!decode_request(rx7900::ByteView(nullptr, sizeof(Request)), request));
    check(decode_request(rx7900::ByteView(bytes, sizeof(bytes)), request));
    check(valid_request_fields(request));
    put(bytes, 16, 0xfedcba9876543210ULL, 8);
    put(bytes, 24, 0x8123456789abcdefULL, 8);
    check(decode_request(rx7900::ByteView(bytes, sizeof(bytes)), request));
    check(request.session == 0xfedcba9876543210ULL && request.handle == 0x8123456789abcdefULL);
    check(!valid_request_fields(request)); // query never accepts capabilities.
    put(bytes, 12, 1, 4);
    check(!decode_request(rx7900::ByteView(bytes, sizeof(bytes)), request));
    put(bytes, 12, 0, 4);
    // The first version past the newest one (recovery_version) is unknown.
    put(bytes, 4, recovery_version + 1, 4);
    check(!decode_request(rx7900::ByteView(bytes, sizeof(bytes)), request));
    put(bytes, 4, 0, 4);
    check(!decode_request(rx7900::ByteView(bytes, sizeof(bytes)), request));
    // Version 4 accepts the version-3 operations with identical field rules.
    for (uint32_t accepted : {graphics_version, recovery_version}) {
        put(bytes, 4, accepted, 4);
        put(bytes, 8, static_cast<uint32_t>(Operation::buffer_readback), 4);
        check(decode_request(rx7900::ByteView(bytes, sizeof(bytes)), request));
        request = base(Operation::graphics_query);
        request.abi_version = accepted;
        request.session = 1; request.handle = 2; request.offset = 3;
        check(valid_request_fields(request));
        request.domain = 1; check(!valid_request_fields(request));
        // Handle 0 names the kernel gfx-ring context for graphics_* only.
        request.domain = 0; request.handle = 0; check(valid_request_fields(request));
        request = base(Operation::graphics_open);
        request.abi_version = accepted;
        request.session = 1; check(valid_request_fields(request));
        request.session = 0; check(!valid_request_fields(request));
        request = base(Operation::buffer_upload);
        request.abi_version = accepted;
        request.session = 1; request.bytes = 4; check(!valid_request_fields(request));
        request.handle = 5; check(valid_request_fields(request));
        request = base(Operation::fill);
        request.abi_version = accepted;
        request.session = 1; request.handle = 2;
        check(valid_request_fields(request));
    }
    put(bytes, 4, recovery_version, 4);
    put(bytes, 8, static_cast<uint32_t>(Operation::buffer_readback) + 1, 4);
    check(!decode_request(rx7900::ByteView(bytes, sizeof(bytes)), request));
    put(bytes, 4, version, 4);
    put(bytes, 8, static_cast<uint32_t>(Operation::graphics_open), 4);
    check(!decode_request(rx7900::ByteView(bytes, sizeof(bytes)), request));
    put(bytes, 8, static_cast<uint32_t>(Operation::query), 4);
    check(graphics_capable(recovery_version) && graphics_capable(graphics_version) &&
          !graphics_capable(version) && !graphics_capable(legacy_version) &&
          !graphics_capable(recovery_version + 1));
    // Additive outcomes and the reply field that names a lost buffer.
    check(static_cast<uint32_t>(Outcome::quarantined) == 6 && static_cast<uint32_t>(Outcome::buffer_lost) == 7 &&
          static_cast<uint32_t>(Outcome::work_lost) == 8 && static_cast<uint32_t>(Outcome::session_lost) == 9);
    static_assert(offsetof(Reply, lost_buffer) == 40, "lost buffer replaces the former reserved field");
    put(bytes, 4, version, 4);
    put(bytes, 8, 15, 4);
    check(!decode_request(rx7900::ByteView(bytes, sizeof(bytes)), request));
    const Operation copies[] = {Operation::read, Operation::write};
    for (auto op : copies) {
        request = base(op);
        request.session = 1; request.handle = 2;
        check(!valid_request_fields(request));
        request.bytes = copy_bytes;
        request.offset = UINT64_MAX; // downstream allocation extent checks own addition.
        check(valid_request_fields(request));
        request.bytes = copy_bytes + 1;
        check(!valid_request_fields(request));
        request.bytes = 1; request.address = 4096;
        check(!valid_request_fields(request));
    }
    request = base(Operation::allocate);
    request.session = 1; request.bytes = 4096; request.access = 3;
    check(valid_request_fields(request));
    request.domain = 2; check(!valid_request_fields(request));
    request.domain = 1; request.access = 4; check(!valid_request_fields(request));
    request.access = 0; check(!valid_request_fields(request));
    request.access = 1; request.handle = 1; check(!valid_request_fields(request));
    request = base(Operation::queue_create); request.session = 1;
    request.domain = 2; check(valid_request_fields(request));
    request.domain = 3; check(!valid_request_fields(request));
    request.domain = 0; request.address = 0x1000; check(!valid_request_fields(request));
    request = base(Operation::map); request.session = 1; request.handle = 2;
    request.access = 3; request.address = 4096; check(valid_request_fields(request));
    request.offset = 1; check(!valid_request_fields(request));
    put(bytes, 8, static_cast<uint32_t>(Operation::copy), 4);
    put(bytes, 4, legacy_version, 4);
    check(!decode_request(rx7900::ByteView(bytes, sizeof(bytes)), request));
    put(bytes, 8, static_cast<uint32_t>(Operation::poll), 4);
    check(decode_request(rx7900::ByteView(bytes, sizeof(bytes)), request));
    request = base(Operation::copy);
    request.session = 1; request.handle = 2;
    check(valid_request_fields(request));
    request.bytes = 48; check(!valid_request_fields(request));
    request = base(Operation::submission_query);
    request.session = 1; request.handle = 2;
    check(!valid_request_fields(request));
    request.offset = UINT32_MAX; check(valid_request_fields(request));
    request.offset = uint64_t{UINT32_MAX} + 1; check(!valid_request_fields(request));
    uint8_t transfer_bytes[sizeof(Transfer)]{};
    put(transfer_bytes, 0, 3, 8);
    put(transfer_bytes, 8, 4, 8);
    put(transfer_bytes, 32, 4096, 8);
    Transfer transfer{};
    for (uint32_t size = 0; size < sizeof(Transfer); ++size)
        check(!decode_transfer({transfer_bytes, size}, transfer, false));
    check(decode_transfer({transfer_bytes, sizeof(transfer_bytes)}, transfer, false));
    check(transfer.source == 3 && transfer.destination == 4 && transfer.bytes == 4096);
    check(!decode_transfer({transfer_bytes, sizeof(transfer_bytes)}, transfer, true));
    put(transfer_bytes, 0, 0, 8);
    put(transfer_bytes, 40, 0xfedcba98, 4);
    check(decode_transfer({transfer_bytes, sizeof(transfer_bytes)}, transfer, true));
    check(!decode_transfer({transfer_bytes, sizeof(transfer_bytes)}, transfer, false));
    put(transfer_bytes, 44, 1, 4);
    check(!decode_transfer({transfer_bytes, sizeof(transfer_bytes)}, transfer, true));
    std::printf("GPU process ABI: %d checks, %d failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

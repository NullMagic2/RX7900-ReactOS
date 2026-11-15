/*
Module: rx7900.tests.acceleration
Purpose: Check Linux SDMA packet bytes, complete submissions and hostile bounds.
Created: 2026-10-21
Architecture: Uses production encoder and shared ByteView; never touches a GPU.
*/
#include <array>
#include <cstdio>

#include "byte_view.hpp"
#include "sdma.hpp"
#include "ring.hpp"
#include "interrupt.hpp"
#include "memory_test_backend.hpp"

namespace {

using namespace rx7900::sdma;
unsigned checks = 0;
unsigned failures = 0;

void expect(bool condition, const char* contract) {
    ++checks;

    if (!condition) {
        ++failures;
        std::fprintf(stderr, "FAIL: %s\n", contract);
    }
}

GpuRange range(uint64_t address, uint64_t size, uint64_t offset = 0) {
    return {{address, size}, offset};
}

template<size_t count>
void expect_words(const uint8_t* bytes, size_t byte_count,
                  const std::array<uint32_t, count>& expected) {
    const rx7900::ByteView view(bytes, byte_count);
    expect(byte_count == count * sizeof(uint32_t), "exact packet stream length");

    for (size_t index = 0; index < count; ++index) {
        uint32_t word = 0;
        expect(view.read_u32(index * sizeof(word), word) && word == expected.at(index),
               "packet dword equals pinned Linux encoding");
    }
}

void check_copy() {
    std::array<uint8_t, 80> bytes{};
    EncodeResult result{};
    CopyRequest request{range(UINT64_C(0x123456000), 32, 3),
                        range(UINT64_C(0xabcdef000), 32, 5), 17};
    expect(encode_copy(request, {bytes.data() + 1, bytes.size() - 1}, result) ==
           EncodeStatus::success, "unaligned CPU output and byte-aligned GPU copy");
    const std::array<uint32_t, 7> expected{
        1, 16, 0, 0x23456003, 1, 0xbcdef005, 0xa};
    expect_words(bytes.data() + 1, result.bytes_written, expected);
    expect(bytes.at(0) == 0 && bytes.at(29) == 0, "copy preserves surrounding bytes");
    expect(bytes.at(13) == 3 && bytes.at(14) == 0x60 && bytes.at(15) == 0x45 &&
           bytes.at(16) == 0x23, "wire bytes are explicitly little-endian");

    request = {range(0x1000, max_transfer_bytes + 1),
               range(UINT64_C(0x100000000), max_transfer_bytes + 1),
               max_transfer_bytes + 1};
    expect(encode_copy(request, {bytes.data(), bytes.size()}, result) ==
           EncodeStatus::success, "split transfer larger than Linux policy limit");
    const std::array<uint32_t, 14> split{
        1, 0x3fffff, 0, 0x1000, 0, 0, 1,
        1, 0, 0, 0x401000, 0, 0x400000, 1};
    expect_words(bytes.data(), result.bytes_written, split);

    request.byte_count = max_transfer_bytes;
    expect(encode_copy(request, {bytes.data(), copy_packet_bytes}, result) ==
           EncodeStatus::success && result.bytes_written == copy_packet_bytes,
           "exactly 4 MiB needs one copy packet");
    request.byte_count = 1;
    request.source = range(gpu_virtual_address_max, 1);
    expect(encode_copy(request, {bytes.data(), bytes.size()}, result) ==
           EncodeStatus::success, "last valid GPU address accepts one-byte copy");
    const std::array<uint32_t, 7> last_byte{1, 0, 0, UINT32_MAX, 0xffff, 0, 1};
    expect_words(bytes.data(), result.bytes_written, last_byte);

    for (size_t capacity = 0; capacity < copy_packet_bytes; ++capacity) {
        bytes.fill(0xa5);
        expect(encode_copy(request, {bytes.data(), capacity}, result) ==
               EncodeStatus::output_too_small && result.bytes_written == 0 &&
               result.bytes_required == copy_packet_bytes, "every short copy output fails");

        for (const auto byte : bytes) {
            expect(byte == 0xa5, "capacity failure writes no partial packet");
        }
    }

    expect(encode_copy(request, {nullptr, SIZE_MAX}, result) ==
           EncodeStatus::output_too_small, "null command output is empty");
    request.byte_count = 0;
    expect(encode_copy(request, {bytes.data(), bytes.size()}, result) ==
           EncodeStatus::invalid_count && result.bytes_required == 0,
           "zero copy count cannot underflow count-minus-one");
    request.byte_count = 2;
    expect(encode_copy(request, {bytes.data(), bytes.size()}, result) ==
           EncodeStatus::invalid_range, "copy crossing allocation end fails");
    request.source = range(UINT64_MAX - 1, 8);
    expect(encode_copy(request, {bytes.data(), bytes.size()}, result) ==
           EncodeStatus::invalid_range, "64-bit GPU range overflow fails");
    request.source = range(gpu_virtual_address_max, 2);
    expect(encode_copy(request, {bytes.data(), bytes.size()}, result) ==
           EncodeStatus::invalid_range, "48-bit allocation overflow fails");
    request.source = range(0x1000, 8, UINT64_MAX);
    expect(encode_copy(request, {bytes.data(), bytes.size()}, result) ==
           EncodeStatus::invalid_range, "untrusted range offset cannot wrap");
    request = {range(0x1000, 8), range(0x1001, 8), 8};
    expect(encode_copy(request, {bytes.data(), bytes.size()}, result) ==
           EncodeStatus::overlapping_ranges, "forward overlap has no memmove semantics");
    request = {range(0x1001, 8), range(0x1000, 8), 8};
    expect(encode_copy(request, {bytes.data(), bytes.size()}, result) ==
           EncodeStatus::overlapping_ranges, "backward overlap rejected");
    request = {range(0x1000, 8), range(0x1008, 8), 8};
    expect(encode_copy(request, {bytes.data(), bytes.size()}, result) ==
           EncodeStatus::success, "adjacent copies do not overlap");
}

void check_fill() {
    std::array<uint8_t, 80> bytes{};
    EncodeResult result{};
    FillRequest request{range(UINT64_C(0x123456000), max_transfer_bytes + 4),
                        max_transfer_bytes + 4, 0xaabbccdd};
    expect(encode_fill(request, {bytes.data(), bytes.size()}, result) ==
           EncodeStatus::success, "split dword fill at 4 MiB");
    const std::array<uint32_t, 10> expected{
        11, 0x23456000, 1, 0xaabbccdd, 0x3fffff,
        11, 0x23856000, 1, 0xaabbccdd, 3};
    expect_words(bytes.data(), result.bytes_written, expected);
    request.byte_count = 0;
    expect(encode_fill(request, {bytes.data(), bytes.size()}, result) ==
           EncodeStatus::invalid_count, "reject zero fill");
    request.byte_count = 3;
    expect(encode_fill(request, {bytes.data(), bytes.size()}, result) ==
           EncodeStatus::invalid_alignment, "reject partial dword fill");
    request.byte_count = 4;
    request.destination.byte_offset = 1;
    expect(encode_fill(request, {bytes.data(), bytes.size()}, result) ==
           EncodeStatus::invalid_alignment, "reject unaligned dword fill address");
    request.destination = range(gpu_virtual_address_max - 3, 4);
    expect(encode_fill(request, {bytes.data(), fill_packet_bytes}, result) ==
           EncodeStatus::success, "fill last address-space dword");
    request.destination = range(0x1000, 4, 4);
    expect(encode_fill(request, {bytes.data(), bytes.size()}, result) ==
           EncodeStatus::invalid_range, "offset at allocation end is not writable");
}

void check_fence() {
    std::array<uint8_t, 80> bytes{};
    EncodeResult result{};
    FenceRequest request{range(UINT64_C(0x1fffffffc), 8),
                         UINT64_C(0x11223344aabbccdd), FenceWidth::bits64, true};
    expect(encode_fence(request, {bytes.data(), bytes.size()}, result) ==
           EncodeStatus::success, "64-bit fence with low-address carry and trap");
    const std::array<uint32_t, 10> expected{
        0x30005, 0xfffffffc, 1, 0xaabbccdd,
        0x30005, 0, 2, 0x11223344, 6, 0};
    expect_words(bytes.data(), result.bytes_written, expected);

    request.width = FenceWidth::bits32;
    expect(encode_fence(request, {bytes.data(), bytes.size()}, result) ==
           EncodeStatus::invalid_count, "32-bit fence rejects sequence truncation");
    request.sequence = UINT32_MAX;
    request.interrupt = false;
    expect(encode_fence(request, {bytes.data(), fence32_packet_bytes}, result) ==
           EncodeStatus::success, "32-bit polling fence needs no trap");
    request.width = static_cast<FenceWidth>(99);
    expect(encode_fence(request, {bytes.data(), bytes.size()}, result) ==
           EncodeStatus::invalid_fence_width, "invalid enum cannot silently narrow fence");
    request.width = FenceWidth::bits64;
    request.destination = range(gpu_virtual_address_max - 3, 4);
    expect(encode_fence(request, {bytes.data(), bytes.size()}, result) ==
           EncodeStatus::invalid_range, "second fence dword cannot escape allocation");
    request.destination = range(0x1001, 8);
    expect(encode_fence(request, {bytes.data(), bytes.size()}, result) ==
           EncodeStatus::invalid_alignment, "unaligned fence is an error, not BUG_ON");
}

void check_submissions() {
    std::array<uint8_t, 80> bytes{};
    EncodeResult result{};
    CopyRequest copy{range(0x1000, 16), range(0x2000, 16), 16};
    FenceRequest fence{range(0x3000, 8), 7, FenceWidth::bits32, false};
    expect(encode_copy_submission(copy, fence, {bytes.data(), bytes.size()}, result) ==
           EncodeStatus::success, "copy and completion fence form one bounded stream");
    const std::array<uint32_t, 11> expected{1, 15, 0, 0x1000, 0, 0x2000, 0,
                                           0x30005, 0x3000, 0, 7};
    expect_words(bytes.data(), result.bytes_written, expected);

    for (size_t capacity = 0; capacity < sizeof(uint32_t) * expected.size(); ++capacity) {
        bytes.fill(0xa5);
        expect(encode_copy_submission(copy, fence, {bytes.data(), capacity}, result) ==
               EncodeStatus::output_too_small && result.bytes_written == 0,
               "insufficient submission space leaves both packets uncommitted");

        for (const auto byte : bytes) {
            expect(byte == 0xa5, "submission capacity failure preserves buffer");
        }
    }

    bytes.fill(0xa5);
    fence.destination = range(0x3001, 8);
    expect(encode_copy_submission(copy, fence, {bytes.data(), bytes.size()}, result) ==
           EncodeStatus::invalid_alignment, "invalid tail fence rejects complete submission");

    for (const auto byte : bytes) {
        expect(byte == 0xa5, "invalid fence must not leave earlier copy commands");
    }

    fence.destination = range(0x1000, 8);
    expect(encode_copy_submission(copy, fence, {bytes.data(), bytes.size()}, result) ==
           EncodeStatus::overlapping_ranges, "fence cannot overwrite live source");
    fence.destination = range(0x2004, 8);
    const FillRequest fill{copy.destination, 16, 0x11223344};
    expect(encode_fill_submission(fill, fence, {bytes.data(), bytes.size()}, result) ==
           EncodeStatus::overlapping_ranges, "fence cannot reside inside filled memory");
    fence.destination = range(0x3000, 8);
    expect(encode_fill_submission(fill, fence, {bytes.data(), bytes.size()}, result) ==
           EncodeStatus::success, "fill submission reuses fence encoding");
    const std::array<uint32_t, 9> fill_expected{
        11, 0x2000, 0, 0x11223344, 15, 0x30005, 0x3000, 0, 7};
    expect_words(bytes.data(), result.bytes_written, fill_expected);

    copy = {range(0, UINT64_C(1) << 47),
            range(UINT64_C(1) << 47, UINT64_C(1) << 47), UINT64_C(1) << 47};
    expect(encode_copy(copy, {bytes.data(), bytes.size()}, result) ==
           EncodeStatus::output_too_small && result.bytes_required ==
           (UINT64_C(1) << 25) * copy_packet_bytes,
           "huge numerically valid copy preflights capacity without looping");
    expect(!CommandBuffer(nullptr, SIZE_MAX).write_u32(0, 1), "null writer rejects stores");
    expect(!CommandBuffer(bytes.data(), bytes.size()).write_u32(SIZE_MAX, 1),
           "writer rejects size_t overflow");
}

struct TestQueue final {
    // Hardware state is supplied explicitly by tests; publish never fabricates
    // a completion fence. This fixture is never a production backend.
    HardwareSnapshot snapshot{0, 0, 0, 0x060000, 0, 0, true, false};
    PublishResult publication = PublishResult::published;
    bool query_ok = true;
    bool stop_ok = true;
    unsigned publishes = 0;
    unsigned stops = 0;

    static bool query(void* context, HardwareSnapshot& result) noexcept {
        auto& self = *static_cast<TestQueue*>(context);
        result = self.snapshot;
        return self.query_ok;
    }

    static PublishResult publish(void* context, uint64_t pointer) noexcept {
        auto& self = *static_cast<TestQueue*>(context);
        ++self.publishes;

        if (self.publication != PublishResult::not_published) {
            self.snapshot.write_pointer_bytes = pointer;
        }

        return self.publication;
    }

    static bool stop(void* context) noexcept {
        auto& self = *static_cast<TestQueue*>(context);
        ++self.stops;

        if (self.stop_ok) {
            self.snapshot.ready = false;
        }

        return self.stop_ok;
    }

    RingBackend backend() noexcept { return {this, query, publish, stop}; }
};

struct QueueFixture final {
    rx7900_test::TestBacking backing{};
    rx7900::memory::Manager memory;
    rx7900::memory::Handle ring_memory{};
    rx7900::memory::Handle fence_memory{};
    rx7900::memory::Handle source{};
    rx7900::memory::Handle destination{};
    std::array<uint8_t, 4096> scratch{};
    TestQueue hardware{};
    Ring ring;

    void initialize() {
        using rx7900::memory::Access;
        using rx7900::memory::Status;
        expect(memory.initialize({0x100000, 0x100000}, backing.backend()) == Status::success,
               "test queue mapped arena initialized");
        expect(memory.allocate(4096, 4096, Access::read_write, true, ring_memory) == Status::success &&
               memory.allocate(4096, 4096, Access::read_write, true, fence_memory) == Status::success &&
               memory.allocate(4096, 4096, Access::read_write, true, source) == Status::success &&
               memory.allocate(4096, 4096, Access::read_write, true, destination) == Status::success,
               "test queue backing allocated");
        const RingConfiguration configuration{ring_memory, fence_memory, 256,
            scratch.data(), scratch.size(), 100, 0, true};
        expect(ring.initialize(memory, configuration, hardware.backend()) == RingStatus::success,
               "verified fresh queue initialized");
    }

    void cleanup() {
        expect(ring.stop() == RingStatus::stopped, "confirmed queue quiescence");
        expect(memory.shutdown() == rx7900::memory::Status::success,
               "queue backing released after DMA stopped");
    }
};

void check_ring_completion() {
    using rx7900::memory::Status;
    QueueFixture fixture;
    fixture.initialize();
    Ticket ticket{};
    expect(fixture.ring.submit_copy({fixture.source, 0}, {fixture.destination, 0}, 16, 10,
                                   ticket) == RingStatus::submitted,
           "doorbell acceptance is submitted, not completed");
    expect(fixture.ring.ticket_status(ticket) == RingStatus::pending,
           "published packet is pending without hardware fence");
    InterruptVector trap{};
    trap.client = 0xa;
    trap.source = 49;
    expect(service_interrupt(trap, 0, fixture.ring, 11) == RingStatus::pending,
           "trap without observed fence cannot complete work");
    rx7900::memory::AllocationInfo info{};
    expect(fixture.memory.describe(fixture.source, info) == Status::success && info.leases == 1,
           "source stays leased after submission");
    expect(fixture.memory.release(fixture.source) == Status::deferred,
           "client release cannot unpin in-flight source");
    expect(fixture.memory.describe(fixture.ring_memory, info) == Status::success,
           "ring CPU mapping remains owned");
    const std::array<uint32_t, 18> commands{
        1, 15, 0, 0x102000, 0, 0x103000, 0,
        17, 0, 0xc3a10000, 0, 0,
        0x30005, 0x101000, 0, 1, 6, 0};
    expect_words(info.backing.cpu_bytes, commands.size() * sizeof(uint32_t), commands);
    const rx7900::ByteView ring_bytes(info.backing.cpu_bytes, 256);

    for (size_t offset = 72; offset < 128; offset += 4) {
        uint32_t value = 1;
        expect(ring_bytes.read_u32(offset, value) && value == 0,
               "ring submission pads to 16 dwords with NOP");
    }

    expect(fixture.hardware.snapshot.write_pointer_bytes == 128,
           "native doorbell receives byte WPTR, not dwords");
    fixture.hardware.snapshot.read_pointer_bytes = 128;
    fixture.hardware.snapshot.completed_sequence = ticket.sequence;
    expect(service_interrupt(trap, 0, fixture.ring, 20) == RingStatus::success &&
           fixture.ring.ticket_status(ticket) == RingStatus::completed,
           "actual observed fence completes queued work");
    expect(fixture.memory.describe(fixture.source, info) == Status::stale_handle,
           "completed fence releases retired source mapping");
    fixture.cleanup();
    expect(fixture.ring.ticket_status(ticket) == RingStatus::invalidated,
           "stopped queue invalidates old generation tickets");
}

void check_ring_wrap_and_space() {
    QueueFixture fixture;
    fixture.initialize();
    Ticket first{};
    Ticket second{};
    Ticket third{};
    Ticket busy{};
    expect(fixture.ring.submit_fill({fixture.destination, 0}, 4, 1, 1, first) ==
           RingStatus::submitted &&
           fixture.ring.submit_fill({fixture.destination, 4}, 4, 2, 2, second) ==
           RingStatus::submitted &&
           fixture.ring.submit_fill({fixture.destination, 8}, 4, 3, 3, third) ==
           RingStatus::submitted, "three aligned fills fit with required ring gap");
    expect(fixture.ring.submit_fill({fixture.destination, 12}, 4, 4, 4, busy) ==
           RingStatus::busy && fixture.hardware.publishes == 3,
           "full ring cannot overwrite unread GPU commands");
    fixture.hardware.snapshot.read_pointer_bytes = 192;
    fixture.hardware.snapshot.completed_sequence = third.sequence;
    expect(fixture.ring.poll(5) == RingStatus::success, "hardware reclaims ring space");
    expect(fixture.ring.submit_copy({fixture.source, 0}, {fixture.destination, 0}, 16, 6,
                                   busy) == RingStatus::submitted &&
           fixture.hardware.snapshot.write_pointer_bytes == 320,
           "copy crosses ring end with monotonic byte counter");
    rx7900::memory::AllocationInfo info{};
    fixture.memory.describe(fixture.ring_memory, info);
    const rx7900::ByteView bytes(info.backing.cpu_bytes, 256);
    uint32_t word = 0;
    expect(bytes.read_u32(192, word) && word == 1, "wrapped copy starts at old WPTR");
    expect(bytes.read_u32(0, word) && word == 6, "TRAP after fence wraps to ring start");
    fixture.cleanup();
}

void check_ring_failures() {
    using rx7900::memory::Status;
    QueueFixture fixture;
    fixture.initialize();
    Ticket ticket{};
    expect(fixture.ring.submit_fill({fixture.ring_memory, 0}, 4, 0, 1, ticket) ==
           RingStatus::invalid_memory && fixture.hardware.publishes == 0,
           "client cannot overwrite privileged command ring");
    expect(fixture.ring.submit_copy({fixture.fence_memory, 0}, {fixture.destination, 0},
                                   4, 2, ticket) == RingStatus::invalid_memory,
           "client cannot use completion storage as source");
    fixture.hardware.publication = PublishResult::not_published;
    expect(fixture.ring.submit_fill({fixture.destination, 0}, 4, 0, 3, ticket) ==
           RingStatus::unavailable && ticket.generation == 0,
           "definitely rejected publication rolls back without ticket");
    rx7900::memory::AllocationInfo info{};
    expect(fixture.memory.describe(fixture.destination, info) == Status::success && info.leases == 0,
           "unpublished command releases leases");
    fixture.hardware.publication = PublishResult::uncertain;
    expect(fixture.ring.submit_fill({fixture.destination, 0}, 4, 0, 4, ticket) ==
           RingStatus::hardware_fault && ticket.sequence == 1,
           "uncertain publication faults and preserves its ticket");
    expect(fixture.memory.release(fixture.destination) == Status::deferred,
           "uncertain hardware ownership pins backing");
    fixture.hardware.stop_ok = false;
    expect(fixture.ring.stop() == RingStatus::hardware_fault &&
           fixture.memory.describe(fixture.destination, info) == Status::success && info.leases == 1,
           "unconfirmed stop cannot free possible DMA target");
    fixture.hardware.stop_ok = true;
    fixture.cleanup();
}

void check_ring_timeout_and_fault() {
    using rx7900::memory::Status;
    QueueFixture fixture;
    fixture.initialize();
    Ticket ticket{};
    expect(fixture.ring.submit_fill({fixture.destination, 0}, 4, 0, 10, ticket) ==
           RingStatus::submitted, "timeout work submitted");
    expect(fixture.ring.poll(9) == RingStatus::invalid_argument,
           "backwards clock cannot shorten or extend timeout");
    expect(fixture.ring.poll(109) == RingStatus::pending,
           "work remains pending before deadline");
    expect(fixture.ring.poll(110) == RingStatus::timeout,
           "absent hardware fence reaches explicit timeout");
    rx7900::memory::AllocationInfo info{};
    expect(fixture.memory.describe(fixture.destination, info) == Status::success && info.leases == 1,
           "timeout preserves possible DMA resources until recovery");
    fixture.cleanup();

    QueueFixture corrupted;
    corrupted.initialize();
    expect(corrupted.ring.submit_fill({corrupted.destination, 0}, 4, 0, 1, ticket) ==
           RingStatus::submitted, "fault work submitted");
    corrupted.hardware.snapshot.completed_sequence = ticket.sequence + 1;
    expect(corrupted.ring.poll(2) == RingStatus::hardware_fault,
           "unsubmitted fence sequence is rejected as hardware corruption");
    corrupted.cleanup();
}

} // namespace

int main() {
    check_copy();
    check_fill();
    check_fence();
    check_submissions();
    check_ring_completion();
    check_ring_wrap_and_space();
    check_ring_failures();
    check_ring_timeout_and_fault();
    std::printf("%u acceleration checks; %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

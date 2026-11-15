/*
Module: lightning.tests.image_receipt
Purpose: Check producer sync points, asynchronous image receipts and the revision-2 present decode.
Created: 2026-10-24
Architecture: Portable host/Wine fixture of the pure lightning_image_receipt.hpp and
lightning_present_request.hpp policies; no OS, device or GPU is simulated.
*/
#include "lightning_image_receipt.hpp"
#include "lightning_present_request.hpp"
#include <stddef.h>
#include <stdio.h>
#include <string.h>

namespace {
using namespace lightning::reactos;
unsigned checks = 0;
unsigned failures = 0;

void check(bool condition, const char* description) {
    ++checks;

    if (!condition) {
        ++failures;
        printf("FAIL %s\n", description);
    }
}

// Wire images of the shared contract built field by field (little endian).
struct Wire final {
    uint8_t bytes[LIGHTNING_OPENGL_PRESENT_ASYNC_BYTES + 8]{};
    size_t size = 0;

    void put32(size_t offset, uint32_t value) {
        for (size_t index = 0; index < 4; ++index) {
            bytes[offset + index] = static_cast<uint8_t>(value >> (8 * index));
        }
    }
    void put64(size_t offset, uint64_t value) {
        put32(offset, static_cast<uint32_t>(value));
        put32(offset + 4, static_cast<uint32_t>(value >> 32));
    }
    rx7900::ByteView view() const { return {bytes, size}; }
};

Wire present_v1() {
    Wire wire;
    wire.size = LIGHTNING_OPENGL_PRESENT_BYTES;
    wire.put32(offsetof(LightningOpenGLPresent, size), LIGHTNING_OPENGL_PRESENT_BYTES);
    wire.put32(offsetof(LightningOpenGLPresent, version), LIGHTNING_OPENGL_PRESENT_VERSION);
    wire.put64(offsetof(LightningOpenGLPresent, lease), UINT64_C(0x1122334455667788));
    wire.put32(offsetof(LightningOpenGLPresent, image_width), 640);
    wire.put32(offsetof(LightningOpenGLPresent, image_height), 480);
    wire.put32(offsetof(LightningOpenGLPresent, image_stride), 2560);
    wire.put32(offsetof(LightningOpenGLPresent, image_offset), 256);
    wire.put32(offsetof(LightningOpenGLPresent, x), static_cast<uint32_t>(-3));
    wire.put32(offsetof(LightningOpenGLPresent, y), 7);
    wire.put32(offsetof(LightningOpenGLPresent, width), 600);
    wire.put32(offsetof(LightningOpenGLPresent, height), 400);
    return wire;
}

Wire present_v2(uint32_t flags, uint64_t sync, uint64_t value) {
    Wire wire = present_v1();
    wire.size = LIGHTNING_OPENGL_PRESENT_ASYNC_BYTES;
    wire.put32(offsetof(LightningOpenGLPresent, size), LIGHTNING_OPENGL_PRESENT_ASYNC_BYTES);
    wire.put32(offsetof(LightningOpenGLPresent, version), LIGHTNING_OPENGL_PRESENT_ASYNC_VERSION);
    wire.put32(offsetof(LightningOpenGLPresentAsync, flags), flags);
    wire.put64(offsetof(LightningOpenGLPresentAsync, producer_sync), sync);
    wire.put64(offsetof(LightningOpenGLPresentAsync, producer_value), value);
    return wire;
}

void check_layout() {
    static_assert(LIGHTNING_OPENGL_PRESENT_CONTRACT_VERSION == 2, "revision-2 contract");
    static_assert(offsetof(LightningOpenGLPresentAsync, present) == 0 &&
        offsetof(LightningOpenGLPresentAsync, flags) == LIGHTNING_OPENGL_PRESENT_BYTES &&
        offsetof(LightningOpenGLPresentAsync, reserved) == 52 &&
        offsetof(LightningOpenGLPresentAsync, producer_sync) == 56 &&
        offsetof(LightningOpenGLPresentAsync, producer_value) == 64, "fixed revision-2 offsets");
    static_assert(offsetof(LightningOpenGLReceiptQuery, lease) == 8 &&
        offsetof(LightningOpenGLReceipt, status) == 8, "fixed receipt offsets");
    check(LIGHTNING_OPENGL_PRESENT_QUEUED != LIGHTNING_OPENGL_PRESENT_SUCCESS &&
        LIGHTNING_OPENGL_PRESENT_QUEUED != LIGHTNING_OPENGL_PRESENT_FAILED &&
        LIGHTNING_OPENGL_PRESENT_QUEUED != LIGHTNING_OPENGL_PRESENT_UNSUPPORTED,
        "QUEUED is distinct from every version-1 result");
    check(LIGHTNING_OPENGL_RECEIPT_ESCAPE != LIGHTNING_OPENGL_PRESENT_ESCAPE,
        "the receipt escape never reaches a version-1 DrawEscape decoder");
}

void check_decode() {
    PresentRequest decoded{};
    check(decode_present_request(present_v1().view(), decoded) && !decoded.revision2 && !decoded.async &&
        decoded.present.lease == UINT64_C(0x1122334455667788) && decoded.present.x == -3 &&
        decoded.present.y == 7 && decoded.present.image_stride == 2560 && decoded.producer.sync == 0,
        "version 1 decodes exactly, including signed placement");
    auto shortened = present_v1();
    shortened.size -= 1;
    PresentRequest untouched{};
    untouched.present.lease = 9;
    check(!decode_present_request(shortened.view(), untouched) && untouched.present.lease == 9,
        "a truncated request fails and leaves the output unchanged");
    auto padded = present_v1();
    padded.size += 4;
    check(!decode_present_request(padded.view(), decoded), "a version-1 request with trailing bytes fails");
    const uint32_t both = LIGHTNING_OPENGL_PRESENT_FLAG_ASYNC | LIGHTNING_OPENGL_PRESENT_FLAG_PRODUCER;
    check(decode_present_request(present_v2(both, 77, 1).view(), decoded) && decoded.revision2 &&
        decoded.async && decoded.producer.sync == 77 && decoded.producer.value == 1 &&
        decoded.present.width == 600, "revision 2 decodes the async flag and producer point");
    check(decode_present_request(present_v2(LIGHTNING_OPENGL_PRESENT_FLAG_ASYNC, 0, 0).view(), decoded) &&
        decoded.async && decoded.producer.sync == 0, "revision 2 without a producer keeps the full wait");
    check(decode_present_request(present_v2(LIGHTNING_OPENGL_PRESENT_FLAG_PRODUCER, 5, 3).view(), decoded) &&
        !decoded.async && decoded.producer.sync == 5, "a producer-only revision 2 stays synchronous");
    check(!decode_present_request(present_v2(LIGHTNING_OPENGL_PRESENT_FLAG_ASYNC, 5, 1).view(), decoded) &&
        !decode_present_request(present_v2(both, 0, 1).view(), decoded) &&
        !decode_present_request(present_v2(both, 5, 0).view(), decoded),
        "producer fields are present exactly when FLAG_PRODUCER names them");
    check(!decode_present_request(present_v2(4, 0, 0).view(), decoded), "unknown flags are refused");
    auto reserved = present_v2(LIGHTNING_OPENGL_PRESENT_FLAG_ASYNC, 0, 0);
    reserved.put32(offsetof(LightningOpenGLPresentAsync, reserved), 1);
    check(!decode_present_request(reserved.view(), decoded), "a nonzero reserved word is refused");
    auto mislabeled = present_v2(LIGHTNING_OPENGL_PRESENT_FLAG_ASYNC, 0, 0);
    mislabeled.put32(offsetof(LightningOpenGLPresent, size), LIGHTNING_OPENGL_PRESENT_BYTES);
    check(!decode_present_request(mislabeled.view(), decoded), "revision 2 must label its own size");
    auto future = present_v2(LIGHTNING_OPENGL_PRESENT_FLAG_ASYNC, 0, 0);
    future.put32(offsetof(LightningOpenGLPresent, version), LIGHTNING_OPENGL_PRESENT_ASYNC_VERSION + 1);
    check(!decode_present_request(future.view(), decoded), "an unknown version is refused");
    auto v1_body = present_v1();
    v1_body.size = LIGHTNING_OPENGL_PRESENT_ASYNC_BYTES;
    check(!decode_present_request(v1_body.view(), decoded), "a version-1 header cannot claim revision-2 length");
}

void check_receipts() {
    const ImageReceipt receipt{7, 3, 10};
    check(decide_receipt({}, {}) == ReceiptState::retired, "no outstanding copy is retired");
    check(decide_receipt(receipt, {3, 12, 9, false}) == ReceiptState::pending, "an unfinished read is pending");
    check(decide_receipt(receipt, {3, 12, 10, false}) == ReceiptState::retired &&
        decide_receipt(receipt, {3, 12, 11, false}) == ReceiptState::retired,
        "the ordered completion prefix retires the read");
    check(decide_receipt(receipt, {4, 1, 0, false}) == ReceiptState::retired &&
        decide_receipt(receipt, {0, 0, 0, false}) == ReceiptState::retired,
        "a new canvas epoch proves the old epoch's reads retired");
    check(decide_receipt(receipt, {3, 12, 9, true}) == ReceiptState::uncertain,
        "an uncertain canvas cannot prove a later completion");
    check(decide_receipt(receipt, {3, 12, 10, true}) == ReceiptState::retired,
        "an already observed completion stays proof on an uncertain canvas");
    check(decide_receipt(receipt, {3, 9, 9, false}) == ReceiptState::uncertain,
        "an accepted prefix below the read is a protocol inconsistency");
    check(decide_receipt(receipt, {3, 9, 12, false}) == ReceiptState::retired,
        "completion beyond the read retires it before any consistency check");
    ImageReceipt output{};
    check(supersede_receipt({}, receipt, output) && output.sequence == 10, "a first copy pins");
    check(supersede_receipt(receipt, {7, 3, 11}, output) && output.sequence == 11, "a newer copy supersedes");
    output = {};
    check(!supersede_receipt(receipt, {7, 3, 10}, output) && !supersede_receipt(receipt, {7, 3, 9}, output) &&
        !supersede_receipt(receipt, {7, 2, 50}, output) && !output.pending(),
        "an older or equal copy never replaces the outstanding one");
    check(supersede_receipt(receipt, {8, 4, 1}, output) && output.epoch == 4,
        "a copy on a newer canvas epoch supersedes");
    check(!supersede_receipt(receipt, {}, output), "a copy without a sequence never pins");
}

void check_producers() {
    const SignalPoint signal{40, 9, 1, 2};
    check(producer_usable({5, 1}, signal, 40) && producer_usable({5, 1}, signal, 41),
        "a recorded producer no later than the image stamp is usable");
    check(!producer_usable({0, 1}, signal, 41) && !producer_usable({5, 2}, signal, 41) &&
        !producer_usable({5, 1}, {}, 41) && !producer_usable({5, 1}, signal, 39),
        "zero, mismatched, unrecorded or later producers keep the full drain");
    SignalTable<3> table;
    table.record(0, signal);
    table.record(11, {});
    check(table.find(0) == nullptr && table.find(11) == nullptr, "zero tokens and empty signals are never recorded");
    table.record(11, {10, 1, 1, 0});
    table.record(12, {20, 2, 1, 0});
    table.record(13, {30, 3, 1, 1});
    table.record(11, {40, 4, 1, 0});
    check(table.find(11) != nullptr && table.find(11)->serial == 40 && table.find(12)->serial == 20,
        "a later signal of the same sync replaces its record in place");
    table.record(14, {50, 5, 1, 0});
    check(table.find(12) == nullptr && table.find(14) != nullptr && table.find(11) != nullptr &&
        table.find(13) != nullptr, "a full table evicts only its oldest record");
    table.erase(13);
    check(table.find(13) == nullptr, "destroyed syncs lose their record");
    table.record(15, {60, 6, 1, 0});
    check(table.find(15) != nullptr && table.find(14) != nullptr, "an erased slot is reused before eviction");
    table.clear();
    check(table.find(11) == nullptr && table.find(15) == nullptr, "invalidation clears every record");
}

void check_pins() {
    PinTable<2> pins;
    check(pins.empty() && !pins.can_pin(0) && pins.can_pin(1), "an empty table can pin any nonzero image");
    check(!pins.pin(1, {}) && pins.empty(), "a copy without a sequence never pins");
    check(pins.pin(1, {7, 3, 10}) && pins.pin(2, {7, 3, 11}) && !pins.can_pin(3) && pins.can_pin(1),
        "a full table refuses new images but still supersedes pinned ones");
    check(!pins.pin(3, {7, 3, 12}) && pins.find(3) == nullptr, "an unpinnable copy leaves no record");
    check(pins.pin(1, {7, 3, 13}) && pins.find(1)->sequence == 13 && !pins.pin(1, {7, 3, 12}) &&
        pins.find(1)->sequence == 13, "pinning keeps only the newest copy of an image");
    unsigned present = 0;

    for (size_t index = 0; index < pins.capacity(); ++index) {
        present += pins.token(index) != 0 ? 1U : 0U;
    }

    check(present == 2 && pins.token(pins.capacity()) == 0, "indexed walk sees exactly the pinned images");
    pins.erase(1);
    check(pins.find(1) == nullptr && pins.can_pin(3) && !pins.empty(), "erasing frees one entry");
    pins.clear();
    check(pins.empty(), "a confirmed display reset clears every pin");
}
} // namespace

int main() {
    check_layout();
    check_decode();
    check_receipts();
    check_producers();
    check_pins();
    printf("Image receipt policy: %u checks, %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

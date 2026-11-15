/*
Module: lightning.tests.native_renderer
Purpose: Exercise production per-file renderer ownership, image leases and real-fence backpressure.
Created: 2026-10-22
Architecture: Exact NT SDK types with bounded OS/host fixtures; no physical GPU or VM access.
*/
#include <stdint.h>
#include <ntifs.h>
#include "lightning_renderer.hpp"
#include "lightning_completion.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace {
using namespace lightning::reactos;
unsigned checks = 0;
unsigned failures = 0;
unsigned allocations = 0;
unsigned references = 0;
unsigned delays = 0;
unsigned resets = 0;
unsigned host_calls = 0;
unsigned host_submits = 0, sync_reads = 0;
uint64_t private_syncs[QLV_MAX_TIMELINES]{};
uint64_t private_points[QLV_MAX_TIMELINES]{};
uint64_t completed_token = 0;
ULONGLONG clock_value = 0;
int process_storage[2]{};
PEPROCESS current_process = reinterpret_cast<PEPROCESS>(&process_storage[0]);
bool complete_on_delay = false;
bool complete_all = false;
bool fuse_requests = false;
unsigned section_handles = 0;
unsigned section_views = 0;
unsigned section_opens = 0;
bool fail_section_open = false;
bool fail_section_map = false;
bool alter_section_size = false;
uint16_t fail_operation = 0;
uint64_t next_context = UINT64_C(0x100000000);
uint64_t next_resource = UINT64_C(0x200000000);
uint64_t aperture_capacity = QLV_APERTURE_BYTES;
uint64_t map_offset = 0;
struct SyncPoint final { uint64_t token = 0; uint64_t value = 0; uint64_t released = 0; };
SyncPoint points[QLV_MAX_SYNCS]{};
// async-present: the fixture can publish private timelines and answer one
// token with a partial value (an earlier point than its latest submission).
bool publish_private = false;
uint64_t partial_token = 0;
uint64_t partial_value = 0;
ReceiptState observe_answer = ReceiptState::pending;
ReceiptState wait_answer = ReceiptState::retired;
unsigned owner_observes = 0, owner_waits = 0;
ImageReceipt last_owner_receipt{};

void check(bool condition, const char* description) {
    ++checks;

    if (!condition) {
        ++failures;
        printf("FAIL %s\n", description);
    }
}

void remember_point(uint64_t token, uint64_t value) noexcept {
    for (auto& point : points) {
        if (point.token == 0 || point.token == token) {
            point.token = token;
            point.value = value;
            return;
        }
    }

    check(false, "fixture sync table remains bounded");
}

QLV_HEADER request(uint16_t operation, uint64_t context) noexcept {
    QLV_HEADER result{};
    result.magic = QLV_MAGIC;
    result.version = QLV_VERSION;
    result.operation = operation;
    result.context = context;
    result.status = QLV_STATUS_UNSUPPORTED;
    return result;
}

NTSTATUS invoke(NativeRenderer& renderer, FILE_OBJECT& file, QLV_HEADER header,
                QLV_HEADER& reply) noexcept {
    uint8_t input[QLV_HEADER_BYTES]{};
    uint8_t output[QLV_HEADER_BYTES + QLV_CAPS_BYTES]{};
    ExchangeReply encoded;
    encoded.header = header;
    size_t written = 0;

    if (encode_exchange_reply(encoded, {input, sizeof(input)}, written) != ExchangeStatus::validated) {
        return STATUS_INVALID_PARAMETER;
    }

    const NTSTATUS status = renderer.exchange_file(&file, current_process,
        {input, written}, {output, sizeof(output)}, written);

    reply = {};

    if (NT_SUCCESS(status) && written >= sizeof(reply)) {
        memcpy(&reply, output, sizeof(reply));
    }

    return status;
}

uint64_t open(NativeRenderer& renderer, FILE_OBJECT& file) noexcept {
    check(NT_SUCCESS(renderer.open_file(&file)), "native file creates process-owned session");
    QLV_HEADER reply;
    check(NT_SUCCESS(invoke(renderer, file, request(QLV_OPEN, 0), reply)) &&
        reply.status == QLV_STATUS_SUCCESS && reply.context != 0 &&
        reply.bytes == aperture_capacity && (reply.flags & QLV_HOST_APERTURE_SIZE) != 0,
        "OPEN publishes actual host context and validated aperture capacity");
    return reply.context;
}

uint64_t blob(NativeRenderer& renderer, FILE_OBJECT& file, uint64_t context) noexcept {
    auto header = request(QLV_CREATE_BLOB, context);
    header.bytes = QLV_PAGE_BYTES;
    header.flags = QLV_SHAREABLE;
    QLV_HEADER reply;
    check(NT_SUCCESS(invoke(renderer, file, header, reply)) && reply.status == QLV_STATUS_SUCCESS,
        "owned shareable image allocation is retained");
    return reply.resource;
}

uint64_t register_image(NativeRenderer& renderer, FILE_OBJECT& file,
                        uint64_t context, uint64_t resource) noexcept {
    auto header = request(QLV_REGISTER_SCANOUT, context);
    header.resource = resource;
    header.value = 1;
    header.bytes = 16 | (UINT64_C(8) << QLV_PACKED_WORD_SHIFT);
    header.offset = 128 | (UINT64_C(64) << QLV_PACKED_WORD_SHIFT);
    header.flags = QLV_SCANOUT_BGRA8;
    QLV_HEADER reply;
    check(NT_SUCCESS(invoke(renderer, file, header, reply)) && reply.status == QLV_STATUS_SUCCESS &&
        reply.value != 0, "actual successful host registration publishes native lease");
    return reply.value;
}

NTSTATUS invoke_payload(NativeRenderer& renderer, FILE_OBJECT& file, QLV_HEADER header,
                        const QLV_SYNC_POINT* syncs, uint32_t count, QLV_HEADER& reply) noexcept {
    uint8_t input[QLV_HEADER_BYTES + 4 * QLV_SYNC_POINT_BYTES]{};
    uint8_t output[QLV_HEADER_BYTES + QLV_CAPS_BYTES]{};
    ExchangeReply encoded;
    encoded.header = header;
    encoded.header.payload_bytes = 0;
    size_t written = 0;

    if (count > 4 || encode_exchange_reply(encoded, {input, sizeof(input)}, written) != ExchangeStatus::validated) {
        return STATUS_INVALID_PARAMETER;
    }

    rx7900::sdma::CommandBuffer packet(input, sizeof(input));
    packet.write_u32(QLV_PAYLOAD_BYTES_OFFSET, static_cast<uint32_t>(count * QLV_SYNC_POINT_BYTES));

    for (uint32_t index = 0; index < count; ++index) {
        const size_t offset = static_cast<size_t>(QLV_HEADER_BYTES + index * QLV_SYNC_POINT_BYTES);
        packet.write_u32(offset, static_cast<uint32_t>(syncs[index].sync));
        packet.write_u32(offset + 4, static_cast<uint32_t>(syncs[index].sync >> 32));
        packet.write_u32(offset + QLV_SYNC_VALUE_OFFSET, static_cast<uint32_t>(syncs[index].value));
        packet.write_u32(offset + QLV_SYNC_VALUE_OFFSET + 4, static_cast<uint32_t>(syncs[index].value >> 32));
    }

    const NTSTATUS status = renderer.exchange_file(&file, current_process,
        {input, static_cast<size_t>(QLV_HEADER_BYTES + count * QLV_SYNC_POINT_BYTES)},
        {output, sizeof(output)}, written);
    reply = {};

    if (NT_SUCCESS(status) && written >= sizeof(reply)) {
        memcpy(&reply, output, sizeof(reply));
    }

    return status;
}

uint64_t user_sync(NativeRenderer& renderer, FILE_OBJECT& file, uint64_t context) noexcept {
    QLV_HEADER reply;
    check(NT_SUCCESS(invoke(renderer, file, request(QLV_CREATE_SYNC, context), reply)) &&
        reply.status == QLV_STATUS_SUCCESS && reply.resource != 0, "caller sync is created");
    return reply.resource;
}

bool submit_ring(NativeRenderer& renderer, FILE_OBJECT& file, uint64_t context, uint32_t ring,
                 uint64_t signal = 0, uint64_t value = 0) noexcept {
    auto header = request(QLV_SUBMIT, context);
    header.value = ring;
    header.offset = signal != 0 ? 1 : 0;
    header.payload_bytes = signal != 0 ? static_cast<uint32_t>(QLV_SYNC_POINT_BYTES) : 0;
    const QLV_SYNC_POINT point{signal, value};
    QLV_HEADER reply;
    return NT_SUCCESS(invoke_payload(renderer, file, header, &point, signal != 0 ? 1 : 0, reply)) &&
        reply.status == QLV_STATUS_SUCCESS;
}

ReceiptState fixture_receipt_owner(void*, const ImageReceipt& receipt, bool wait) noexcept {
    wait ? ++owner_waits : ++owner_observes;
    last_owner_receipt = receipt;
    return wait ? wait_answer : observe_answer;
}

// async-present: a present names its producer submission; only that private
// ring point is waited for, from the published timeline. Every other case
// keeps today's drain to the image's conservative stamp.
void check_producer_points() {
    NativeTransport transport;
    NativeRenderer renderer;
    FILE_OBJECT file{};
    check(NT_SUCCESS(renderer.initialize(transport)), "producer fixture initializes renderer");
    const auto context = open(renderer, file);
    const auto resource = blob(renderer, file, context);
    const auto lease = register_image(renderer, file, context, resource);
    const auto fence = user_sync(renderer, file, context);
    check(submit_ring(renderer, file, context, 0, fence, 1), "producer blit signals the per-image fence");
    const uint64_t producer = private_points[0];
    check(submit_ring(renderer, file, context, 0) && submit_ring(renderer, file, context, 1),
        "the next frame is already queued on both rings before the present");
    publish_private = true;
    complete_all = false;
    completed_token = 0;
    partial_token = private_syncs[0];
    partial_value = producer;
    const ProducerPoint named{fence, 1};
    NativeImageLease image;
    unsigned delays_before = delays;
    const unsigned reads_before = sync_reads;
    check(NT_SUCCESS(renderer.resolve_image(lease, current_process, image, &named)) &&
        image.resource == resource && delays == delays_before && sync_reads == reads_before + 1,
        "present waits only for its own producer point, never for the next frame's work");

    complete_on_delay = true;
    delays_before = delays;
    check(NT_SUCCESS(renderer.resolve_image(lease, current_process, image)) && delays > delays_before,
        "a version-1 present keeps the full drain to the image's latest stamp");
    complete_on_delay = false;
    complete_all = false;
    const ProducerPoint mismatched{fence, 2};
    const ProducerPoint unknown{UINT64_C(0x7777), 1};
    const ProducerPoint* fallbacks[] = {&mismatched, &unknown};

    for (const auto* fallback : fallbacks) {
        check(submit_ring(renderer, file, context, 1), "more independent work is queued");
        complete_on_delay = true;
        delays_before = delays;
        check(NT_SUCCESS(renderer.resolve_image(lease, current_process, image, fallback)) && delays > delays_before,
            "an unrecorded or mismatched producer point keeps the full drain");
        complete_on_delay = false;
        complete_all = false;
    }

    check(submit_ring(renderer, file, context, 0, fence, 1) && submit_ring(renderer, file, context, 1),
        "the fence is signaled again ahead of independent work");
    partial_value = private_points[0];
    publish_private = false;
    complete_on_delay = true;
    delays_before = delays;
    check(NT_SUCCESS(renderer.resolve_image(lease, current_process, image, &named)) && delays > delays_before,
        "an unpublished private timeline keeps the full drain");
    complete_on_delay = false;
    complete_all = false;

    // Fence reset (WRITE_SYNC 0) is busy only while its own signal is pending.
    check(submit_ring(renderer, file, context, 0, fence, 1) && submit_ring(renderer, file, context, 1),
        "next frame signals the fence and queues more work");
    publish_private = true;
    partial_value = private_points[0];
    auto reset = request(QLV_WRITE_SYNC, context);
    reset.resource = fence;
    reset.value = 0;
    QLV_HEADER reply;
    delays_before = delays;
    check(NT_SUCCESS(invoke(renderer, file, reset, reply)) && reply.status == QLV_STATUS_SUCCESS &&
        delays == delays_before, "a fence reset waits for its own signal, not for later queued work");
    check(submit_ring(renderer, file, context, 0, fence, 1) && submit_ring(renderer, file, context, 1),
        "the fence is signaled once more");
    publish_private = false;
    complete_on_delay = true;
    delays_before = delays;
    check(NT_SUCCESS(invoke(renderer, file, reset, reply)) && reply.status == QLV_STATUS_SUCCESS &&
        delays > delays_before, "an unpublished fence reset keeps the full drain");
    complete_on_delay = false;
    complete_all = true;
    partial_token = 0;
    check(NT_SUCCESS(renderer.close_file(&file)) && NT_SUCCESS(renderer.shutdown()),
        "producer-point session retires normally");
    complete_all = false;
}

// frame-waits: the Win32 present worker resets the per-image fence after its
// own wait and before the present escape names that fence as producer. The
// reset must not erase the recorded signal: the present stays targeted while
// the next frame is already queued on both rings.
void check_reset_before_present() {
    NativeTransport transport;
    NativeRenderer renderer;
    FILE_OBJECT file{};
    check(NT_SUCCESS(renderer.initialize(transport)), "reset-before-present fixture initializes renderer");
    const auto context = open(renderer, file);
    const auto resource = blob(renderer, file, context);
    const auto lease = register_image(renderer, file, context, resource);
    const auto fence = user_sync(renderer, file, context);
    check(submit_ring(renderer, file, context, 0, fence, 1), "the blit signals the per-image fence");
    const uint64_t producer = private_points[0];
    check(submit_ring(renderer, file, context, 0) && submit_ring(renderer, file, context, 1),
        "the next frame is queued before the worker presents");
    publish_private = true;
    complete_all = false;
    completed_token = 0;
    partial_token = private_syncs[0];
    partial_value = producer;
    auto reset = request(QLV_WRITE_SYNC, context);
    reset.resource = fence;
    reset.value = 0;
    QLV_HEADER reply;
    unsigned delays_before = delays;
    check(NT_SUCCESS(invoke(renderer, file, reset, reply)) && reply.status == QLV_STATUS_SUCCESS &&
        delays == delays_before, "the worker's fence reset after its own wait does not drain later work");
    const ProducerPoint named{fence, 1};
    NativeImageLease image;
    delays_before = delays;
    check(NT_SUCCESS(renderer.resolve_image(lease, current_process, image, &named)) &&
        image.resource == resource && delays == delays_before,
        "a present naming the reset fence still waits only for its recorded producer point");
    complete_all = true;
    partial_token = 0;
    check(NT_SUCCESS(renderer.close_file(&file)) && NT_SUCCESS(renderer.shutdown()),
        "reset-before-present session retires normally");
    complete_all = false;
}

// async-present: an accepted asynchronous copy pins its image until the
// filter's lower-canvas observer proves the read retired.
void check_receipt_pins() {
    NativeTransport transport;
    NativeRenderer renderer;
    FILE_OBJECT file{};
    check(NT_SUCCESS(renderer.initialize(transport)), "receipt fixture initializes renderer");
    const auto context = open(renderer, file);
    const auto resource = blob(renderer, file, context);
    const auto lease = register_image(renderer, file, context, resource);
    PEPROCESS foreign = reinterpret_cast<PEPROCESS>(&process_storage[1]);
    check(!renderer.can_track_receipt(lease, current_process) &&
        !renderer.track_receipt(lease, current_process, {7, 3, 10}),
        "without a receipt owner every copy stays synchronous");
    renderer.set_receipt_owner({nullptr, fixture_receipt_owner, false});
    check(!renderer.can_track_receipt(lease, current_process),
        "a miniport without version-2 support never pins a lease");
    renderer.set_receipt_owner({nullptr, fixture_receipt_owner, true});
    check(renderer.can_track_receipt(lease, current_process) && !renderer.can_track_receipt(lease, foreign) &&
        !renderer.can_track_receipt(0, current_process), "only the owning process can pin its own lease");
    check(renderer.track_receipt(lease, current_process, {7, 3, 10}) &&
        !renderer.track_receipt(lease, current_process, {7, 3, 9}) &&
        renderer.track_receipt(lease, current_process, {7, 3, 11}),
        "a newer accepted copy supersedes an older one, never the reverse");
    observe_answer = ReceiptState::pending;
    const unsigned waits_before = owner_waits;
    check(renderer.query_receipt(lease, current_process) == ReceiptState::pending &&
        owner_waits == waits_before && last_owner_receipt.sequence == 11,
        "receipt query observes the newest copy without waiting");
    check(renderer.query_receipt(lease, foreign) == ReceiptState::retired,
        "a foreign process learns nothing about another lease");
    observe_answer = ReceiptState::retired;
    check(renderer.query_receipt(lease, current_process) == ReceiptState::retired, "observed retirement unpins");
    const unsigned observes = owner_observes;
    check(renderer.query_receipt(lease, current_process) == ReceiptState::retired && owner_observes == observes,
        "an unpinned lease reports retired without asking the canvas");

    check(renderer.track_receipt(lease, current_process, {7, 3, 12}), "copy pins the image again");
    wait_answer = ReceiptState::retired;
    auto registration = request(QLV_REGISTER_SCANOUT, context);
    registration.resource = resource;
    registration.value = 1;
    registration.bytes = 16 | (UINT64_C(8) << QLV_PACKED_WORD_SHIFT);
    registration.offset = 128 | (UINT64_C(64) << QLV_PACKED_WORD_SHIFT);
    registration.flags = QLV_SCANOUT_BGRA8;
    QLV_HEADER reply;
    unsigned waits = owner_waits;
    check(NT_SUCCESS(invoke(renderer, file, registration, reply)) && reply.status == QLV_STATUS_SUCCESS &&
        owner_waits == waits + 1, "re-registration first waits for the pinned read");
    const auto relisted = reply.value;
    check(renderer.track_receipt(relisted, current_process, {7, 3, 13}), "the new identity can be pinned");
    auto destroy = request(QLV_DESTROY_BLOB, context);
    destroy.resource = resource;
    waits = owner_waits;
    check(NT_SUCCESS(invoke(renderer, file, destroy, reply)) && reply.status == QLV_STATUS_SUCCESS &&
        owner_waits == waits + 1 && !renderer.can_track_receipt(relisted, current_process),
        "destroying a pinned image waits for its read before the host frees it");

    uint64_t leases[17]{};

    for (auto& pinned : leases) {
        pinned = register_image(renderer, file, context, blob(renderer, file, context));
    }

    bool pinned_all = true;

    for (size_t index = 0; index < 16; ++index) {
        pinned_all = pinned_all && renderer.track_receipt(leases[index], current_process, {7, 3, 20});
    }

    check(pinned_all && !renderer.can_track_receipt(leases[16], current_process) &&
        renderer.can_track_receipt(leases[0], current_process),
        "a full pin table keeps further copies synchronous but still supersedes pinned images");
    renderer.retire_receipts_after_display_reset();
    waits = owner_waits;
    check(renderer.query_receipt(leases[0], current_process) == ReceiptState::retired && owner_waits == waits,
        "a confirmed display quiesce/reset retires every pinned read");
    check(renderer.track_receipt(leases[1], current_process, {7, 4, 1}) &&
        renderer.track_receipt(leases[2], current_process, {7, 4, 2}), "two images are pinned again");
    waits = owner_waits;
    check(NT_SUCCESS(renderer.close_file(&file)) && owner_waits == waits + 2,
        "file close settles every pinned read before host CLOSE");

    FILE_OBJECT second{};
    const auto second_context = open(renderer, second);
    const auto second_resource = blob(renderer, second, second_context);
    const auto second_lease = register_image(renderer, second, second_context, second_resource);
    check(renderer.track_receipt(second_lease, current_process, {8, 1, 1}), "second session pins a read");
    observe_answer = ReceiptState::uncertain;
    check(renderer.query_receipt(second_lease, current_process) == ReceiptState::uncertain && renderer.quarantined(),
        "an unprovable read contains the renderer, as an uncertain synchronous copy did");
    observe_answer = ReceiptState::pending;
    wait_answer = ReceiptState::retired;
}

// lightning-overlap: a device power-down or surprise removal overlapping an
// asynchronous read pinned on an image. With host access the filter's
// shutdown settles the pin through the owner's bounded wait before the host
// CLOSE; without host access (surprise) the pin is kept and the renderer is
// contained, never silently dropped.
void check_power_down_pins() {
    {
        NativeTransport transport;
        NativeRenderer renderer;
        FILE_OBJECT file{};
        check(NT_SUCCESS(renderer.initialize(transport)), "power-down pin fixture initializes renderer");
        const auto context = open(renderer, file);
        const auto lease = register_image(renderer, file, context, blob(renderer, file, context));
        renderer.set_receipt_owner({nullptr, fixture_receipt_owner, true});
        check(renderer.track_receipt(lease, current_process, {9, 2, 4}), "power-down fixture pins one read");
        wait_answer = ReceiptState::retired;
        const unsigned waits = owner_waits;
        const unsigned calls = host_calls;
        check(NT_SUCCESS(renderer.shutdown(true)) && owner_waits == waits + 1 && last_owner_receipt.sequence == 4 &&
            !renderer.quarantined() && host_calls > calls,
            "D3 shutdown settles the pinned read through one bounded wait before the host CLOSE");
        check(renderer.query_receipt(lease, current_process) == ReceiptState::retired,
            "after the settled power-down the lease reports retired");
        check(NT_SUCCESS(renderer.close_file(&file)), "power-down fixture file closes after shutdown");
    }
    {
        NativeTransport transport;
        NativeRenderer renderer;
        FILE_OBJECT file{};
        check(NT_SUCCESS(renderer.initialize(transport)), "surprise pin fixture initializes renderer");
        const auto context = open(renderer, file);
        const auto lease = register_image(renderer, file, context, blob(renderer, file, context));
        renderer.set_receipt_owner({nullptr, fixture_receipt_owner, true});
        check(renderer.track_receipt(lease, current_process, {9, 3, 7}), "surprise fixture pins one read");
        const unsigned waits = owner_waits;
        const unsigned calls = host_calls;
        check(renderer.shutdown(false) == STATUS_DEVICE_BUSY && renderer.quarantined() && owner_waits == waits &&
            host_calls == calls,
            "surprise shutdown never waits on or calls a gone device and contains the pinned read");
        check(renderer.close_file(&file) == STATUS_DEVICE_BUSY && owner_waits == waits && host_calls == calls,
            "a contained renderer's close keeps the pinned read's host image");
    }
}

void check_images() {
    NativeTransport transport;
    NativeRenderer renderer;
    FILE_OBJECT first{};
    check(NT_SUCCESS(renderer.initialize(transport)), "native renderer claims transport aperture");
    const auto context = open(renderer, first);
    const auto resource = blob(renderer, first, context);
    const auto lease = register_image(renderer, first, context, resource);
    NativeImageLease image;
    check(NT_SUCCESS(renderer.resolve_image(lease, current_process, image)) && image.context == context &&
        image.resource == resource && image.layout.width == 16 && image.layout.height == 8 &&
        image.layout.offset == 64 && image.layout.stride == 128, "lease resolves exact retained image layout");
    check(!NT_SUCCESS(renderer.resolve_image(0, current_process, image)) && image.context == 0,
        "zero lease clears prior resolution output");
    current_process = reinterpret_cast<PEPROCESS>(&process_storage[1]);
    check(!NT_SUCCESS(renderer.resolve_image(lease, current_process, image)), "foreign process cannot use image lease");
    current_process = reinterpret_cast<PEPROCESS>(&process_storage[0]);
    const auto replacement = register_image(renderer, first, context, resource);
    check(replacement != lease && !NT_SUCCESS(renderer.resolve_image(lease, current_process, image)) &&
        NT_SUCCESS(renderer.resolve_image(replacement, current_process, image)), "reregistration invalidates old lease identity");
    auto registration = request(QLV_REGISTER_SCANOUT, context);
    registration.resource = resource;
    registration.value = 1;
    registration.bytes = 16 | (UINT64_C(8) << QLV_PACKED_WORD_SHIFT);
    registration.offset = 128 | (UINT64_C(64) << QLV_PACKED_WORD_SHIFT);
    registration.flags = QLV_SCANOUT_BGRA8;
    fail_operation = QLV_REGISTER_SCANOUT;
    QLV_HEADER reply;
    check(NT_SUCCESS(invoke(renderer, first, registration, reply)) && reply.status == QLV_STATUS_UNSUPPORTED &&
        !NT_SUCCESS(renderer.resolve_image(replacement, current_process, image)),
        "host registration failure revokes prior potentially replaced import");
    fail_operation = 0;
    const auto third_lease = register_image(renderer, first, context, resource);
    const unsigned before = delays;
    check(NT_SUCCESS(invoke(renderer, first, request(QLV_SUBMIT, context), reply)) &&
        reply.status == QLV_STATUS_SUCCESS, "reused image receives asynchronous producer work");
    complete_on_delay = true;
    check(NT_SUCCESS(renderer.resolve_image(third_lease, current_process, image)) && delays > before,
        "consumer lease waits for latest actual producer fence on every frame");
    complete_on_delay = false;
    complete_all = false;
    auto destroy = request(QLV_DESTROY_BLOB, context);
    destroy.resource = resource;
    check(NT_SUCCESS(invoke(renderer, first, destroy, reply)) &&
        !NT_SUCCESS(renderer.resolve_image(third_lease, current_process, image)), "destroyed resource revokes its image lease");
    check(NT_SUCCESS(renderer.close_file(&first)) && first.FsContext == nullptr,
        "native file cleanup retires actual host session");

    FILE_OBJECT second{};
    const auto second_context = open(renderer, second);
    const auto second_resource = blob(renderer, second, second_context);
    const auto second_lease = register_image(renderer, second, second_context, second_resource);
    check(second_lease > third_lease, "closed file slots never reuse old image identities");
    renderer.process_exit(PsGetCurrentProcessId());
    check(!NT_SUCCESS(renderer.resolve_image(second_lease, current_process, image)),
        "process exit revokes image before process address-space retirement");
    check(NT_SUCCESS(renderer.close_file(&second)), "post-exit file cleanup releases retained process reference");
    check(NT_SUCCESS(renderer.shutdown()), "native renderer shuts down without dangling leases");
}

void check_blob_capacity() {
    NativeTransport transport;
    NativeRenderer renderer;
    FILE_OBJECT file{};
    check(NT_SUCCESS(renderer.initialize(transport)), "blob-capacity fixture initializes native renderer");
    const auto context = open(renderer, file);
    static uint64_t tokens[3072]{};
    auto create = request(QLV_CREATE_BLOB, context);
    create.bytes = QLV_PAGE_BYTES;
    create.flags = QLV_SHAREABLE;
    QLV_HEADER reply;
    size_t created = 0;

    for (; created < 3072; ++created) {
        if (!NT_SUCCESS(invoke(renderer, file, create, reply)) ||
            reply.status != QLV_STATUS_SUCCESS || reply.resource == 0) {
            break;
        }
        tokens[created] = reply.resource;
    }

    check(created == 3072 && tokens[2048] != 0 && tokens[3071] != 0,
        "live native blobs cross the former 2048 limit and fill the bounded new capacity");
    const unsigned calls_at_capacity = host_calls;
    check(NT_SUCCESS(invoke(renderer, file, create, reply)) &&
        reply.status == QLV_STATUS_NO_MEMORY && reply.resource == 0 &&
        host_calls == calls_at_capacity,
        "one past the bounded blob capacity fails before sending an unsafe host request");

    if (created != 0) {
        auto destroy = request(QLV_DESTROY_BLOB, context);
        destroy.resource = tokens[created - 1];
        check(NT_SUCCESS(invoke(renderer, file, destroy, reply)) &&
            reply.status == QLV_STATUS_SUCCESS,
            "destroy retires one live blob at the capacity boundary");
        check(NT_SUCCESS(invoke(renderer, file, create, reply)) &&
            reply.status == QLV_STATUS_SUCCESS && reply.resource != 0,
            "retired blob slot can be reused after the capacity boundary");
    }

    check(NT_SUCCESS(renderer.close_file(&file)) && file.FsContext == nullptr &&
        NT_SUCCESS(renderer.shutdown()),
        "session close retires all live blob rows and its bounded pool allocation");
}

void check_mappings() {
    NativeTransport transport;
    NativeRenderer renderer;
    FILE_OBJECT file{};
    check(NT_SUCCESS(renderer.initialize(transport)), "mapping fixture claims bounded native aperture");
    const auto context = open(renderer, file);
    auto create = request(QLV_CREATE_BLOB, context);
    create.bytes = QLV_PAGE_BYTES;
    create.flags = QLV_MAPPABLE | QLV_COHERENT;
    QLV_HEADER reply;
    check(NT_SUCCESS(invoke(renderer, file, create, reply)), "mapping fixture creates owned mappable allocation");
    auto map = request(QLV_MAP_BLOB, context);
    map.resource = reply.resource;
    check(NT_SUCCESS(invoke(renderer, file, map, reply)) && reply.value == UINT32_C(0x20000000) &&
        section_handles == 1 && section_views == 1, "native owned section publishes exactly one valid user view");
    const unsigned opened = section_opens;
    check(NT_SUCCESS(invoke(renderer, file, map, reply)) && section_opens == opened && section_views == 1,
        "repeat mapping reuses the retained view without another section handle");
    auto unmap = request(QLV_UNMAP_BLOB, context);
    unmap.resource = map.resource;
    check(NT_SUCCESS(invoke(renderer, file, unmap, reply)) && section_handles == 0 && section_views == 0,
        "UNMAP retires the user view before closing its kernel section handle");
    fail_section_open = true;
    check(NT_SUCCESS(invoke(renderer, file, map, reply)) && reply.status == QLV_STATUS_NO_MEMORY &&
        reply.value == 0 && section_handles == 0 && section_views == 0,
        "failed section acquisition never publishes a view or leaks ownership");
    fail_section_open = false;
    fail_section_map = true;
    check(NT_SUCCESS(invoke(renderer, file, map, reply)) && reply.status == QLV_STATUS_NO_MEMORY &&
        reply.value == 0 && section_handles == 0 && section_views == 0,
        "failed view creation closes its retained section handle");
    fail_section_map = false;
    alter_section_size = true;
    check(NT_SUCCESS(invoke(renderer, file, map, reply)) && reply.status == QLV_STATUS_NO_MEMORY &&
        reply.value == 0 && section_handles == 0 && section_views == 0,
        "OS-rounded view exceeding the exact owned span is revoked before publication");
    alter_section_size = false;
    check(NT_SUCCESS(invoke(renderer, file, map, reply)) && section_views == 1,
        "a clean mapping retry succeeds after refused publication");
    renderer.process_exit(PsGetCurrentProcessId());
    check(section_handles == 0 && section_views == 0,
        "process exit revokes all owned section views before address-space deletion");
    check(NT_SUCCESS(renderer.close_file(&file)) && NT_SUCCESS(renderer.shutdown()),
        "post-exit cleanup releases native renderer session ownership");
}

// Ac_throughput (2026-10-23): CPU view changes no longer wait for queued GPU
// work, while destruction of the allocation still observes its private fences.
void check_view_changes_do_not_drain() {
    NativeTransport transport;
    NativeRenderer renderer;
    FILE_OBJECT file{};
    check(NT_SUCCESS(renderer.initialize(transport)), "view-drain fixture initializes native renderer");
    const auto context = open(renderer, file);
    auto create = request(QLV_CREATE_BLOB, context);
    create.bytes = QLV_PAGE_BYTES;
    create.flags = QLV_MAPPABLE | QLV_COHERENT;
    QLV_HEADER reply;
    check(NT_SUCCESS(invoke(renderer, file, create, reply)) && reply.status == QLV_STATUS_SUCCESS,
        "view-drain fixture owns a mappable allocation");
    const uint64_t resource = reply.resource;
    auto map = request(QLV_MAP_BLOB, context);
    map.resource = resource;
    check(NT_SUCCESS(invoke(renderer, file, map, reply)) && section_views == 1,
        "view-drain fixture publishes its first CPU view");
    check(NT_SUCCESS(invoke(renderer, file, request(QLV_SUBMIT, context), reply)) &&
        reply.status == QLV_STATUS_SUCCESS, "queued GPU work may still read the mapped allocation");
    complete_all = false;
    const unsigned delays_before = delays;
    const unsigned reads_before = sync_reads;
    const unsigned resets_before = resets;
    auto unmap = request(QLV_UNMAP_BLOB, context);
    unmap.resource = resource;
    check(NT_SUCCESS(invoke(renderer, file, unmap, reply)) && reply.status == QLV_STATUS_SUCCESS &&
        section_views == 0 && section_handles == 0,
        "UNMAP with pending GPU work retires the CPU view and its section handle");
    check(delays == delays_before && sync_reads == reads_before && resets == resets_before,
        "UNMAP neither polls nor sleeps on unrelated queued GPU work");
    check(NT_SUCCESS(invoke(renderer, file, map, reply)) && reply.status == QLV_STATUS_SUCCESS &&
        reply.value != 0 && section_views == 1,
        "MAP with pending GPU work publishes a fresh CPU view");
    check(delays == delays_before && sync_reads == reads_before && resets == resets_before,
        "MAP neither polls nor sleeps on unrelated queued GPU work");
    check(NT_SUCCESS(invoke(renderer, file, unmap, reply)) && section_views == 0,
        "a remapped view still retires before the host releases its aperture");
    auto destroy = request(QLV_DESTROY_BLOB, context);
    destroy.resource = resource;
    complete_on_delay = true;
    check(NT_SUCCESS(invoke(renderer, file, destroy, reply)) && reply.status == QLV_STATUS_SUCCESS &&
        delays > delays_before && sync_reads > reads_before,
        "DESTROY still waits for the observed private fence of the queued GPU use");
    complete_on_delay = false;
    complete_all = false;
    check(resets == resets_before, "view changes during queued GPU work never reset the device");
    check(NT_SUCCESS(renderer.close_file(&file)) && NT_SUCCESS(renderer.shutdown()),
        "view-drain fixture retires its session normally");
}

// Ac_throughput (2026-10-23): a two- or three-image swapchain registers one
// lease per image; a mode change (new images, old ones destroyed) and a
// close/reopen must leave no old lease resolvable.
void check_multi_image_leases() {
    NativeTransport transport;
    NativeRenderer renderer;
    FILE_OBJECT file{};
    check(NT_SUCCESS(renderer.initialize(transport)), "multi-image fixture initializes native renderer");
    const auto context = open(renderer, file);
    uint64_t old_images[3]{};
    uint64_t old_leases[3]{};
    NativeImageLease image;

    for (size_t index = 0; index < 3; ++index) {
        old_images[index] = blob(renderer, file, context);
        old_leases[index] = register_image(renderer, file, context, old_images[index]);
    }

    check(old_leases[0] != old_leases[1] && old_leases[1] != old_leases[2] &&
        old_leases[0] != old_leases[2], "each swapchain image owns a distinct lease");
    bool exact = true;

    for (size_t index = 0; index < 3; ++index) {
        exact = exact && NT_SUCCESS(renderer.resolve_image(old_leases[index], current_process, image)) &&
            image.resource == old_images[index];
    }

    check(exact, "every image lease resolves to exactly its own retained image");
    uint64_t new_leases[2]{};

    for (auto& lease : new_leases) {
        lease = register_image(renderer, file, context, blob(renderer, file, context));
    }

    QLV_HEADER reply;
    auto destroy = request(QLV_DESTROY_BLOB, context);

    for (const auto resource : old_images) {
        destroy.resource = resource;
        check(NT_SUCCESS(invoke(renderer, file, destroy, reply)) && reply.status == QLV_STATUS_SUCCESS,
            "mode change destroys an old swapchain image");
    }

    bool retired = true;

    for (const auto lease : old_leases) {
        retired = retired && !NT_SUCCESS(renderer.resolve_image(lease, current_process, image));
    }

    check(retired && NT_SUCCESS(renderer.resolve_image(new_leases[0], current_process, image)) &&
        NT_SUCCESS(renderer.resolve_image(new_leases[1], current_process, image)),
        "mode change retires every old lease and keeps the replacement chain");
    check(NT_SUCCESS(renderer.close_file(&file)), "multi-image session closes");
    FILE_OBJECT reopened{};
    const auto reopened_context = open(renderer, reopened);
    const auto fresh = register_image(renderer, reopened, reopened_context,
        blob(renderer, reopened, reopened_context));
    check(fresh > new_leases[1] && !NT_SUCCESS(renderer.resolve_image(new_leases[0], current_process, image)) &&
        !NT_SUCCESS(renderer.resolve_image(new_leases[1], current_process, image)),
        "close and reopen leave no earlier lease resolvable");
    check(NT_SUCCESS(renderer.close_file(&reopened)) && NT_SUCCESS(renderer.shutdown()),
        "reopened multi-image session retires normally");
}

void check_mapped_recovery() {
    NativeTransport transport;
    NativeRenderer renderer;
    FILE_OBJECT file{};
    FILE_OBJECT replacement{};
    check(NT_SUCCESS(renderer.initialize(transport)), "recovery fixture initializes native renderer");
    const auto context = open(renderer, file);
    auto create = request(QLV_CREATE_BLOB, context);
    create.bytes = QLV_PAGE_BYTES;
    create.flags = QLV_MAPPABLE | QLV_COHERENT;
    QLV_HEADER reply;
    check(NT_SUCCESS(invoke(renderer, file, create, reply)), "recovery fixture owns mapped allocation");
    auto map = request(QLV_MAP_BLOB, context);
    map.resource = reply.resource;
    check(NT_SUCCESS(invoke(renderer, file, map, reply)) && section_views == 1, "recovery caller borrows a valid native view");
    fail_operation = QLV_SUBMIT;
    check(NT_SUCCESS(invoke(renderer, file, request(QLV_SUBMIT, context), reply)) &&
        reply.status == QLV_STATUS_UNSUPPORTED, "failed accepted-path response requires actual generation reset");
    fail_operation = 0;
    check(section_views == 1 && section_handles == 1 && !renderer.ready(),
        "reset retains borrowed CPU pointers while blocking new renderer mappings");
    check(!NT_SUCCESS(renderer.open_file(&replacement)) && replacement.FsContext == nullptr,
        "replacement session cannot publish an alias over a retained lost-generation view");
    check(NT_SUCCESS(renderer.close_file(&file)) && section_views == 0 && section_handles == 0 && renderer.ready(),
        "explicit old-owner cleanup retires views and reopens the confirmed replacement generation");
    const auto replacement_context = open(renderer, replacement);
    check(replacement_context != context, "replacement context never reuses lost logical identity");
    check(NT_SUCCESS(renderer.close_file(&replacement)) && NT_SUCCESS(renderer.shutdown()),
        "replacement lifetime completes without leaked section/session ownership");
}

void check_shutdown_retains_borrow(bool allow_host) {
    NativeTransport transport;
    NativeRenderer renderer;
    FILE_OBJECT file{};
    check(NT_SUCCESS(renderer.initialize(transport)), "shutdown fixture initializes owned renderer");
    const auto context = open(renderer, file);
    auto create = request(QLV_CREATE_BLOB, context);
    create.bytes = QLV_PAGE_BYTES;
    create.flags = QLV_MAPPABLE | QLV_COHERENT;
    QLV_HEADER reply;
    check(NT_SUCCESS(invoke(renderer, file, create, reply)), "shutdown fixture owns ring backing");
    auto map = request(QLV_MAP_BLOB, context);
    map.resource = reply.resource;
    check(NT_SUCCESS(invoke(renderer, file, map, reply)) && section_views == 1,
        "shutdown fixture publishes the caller's borrowed ring pointer");
    const unsigned calls = host_calls;
    const unsigned retained_allocations = allocations;
    const unsigned retained_references = references;
    check(renderer.shutdown(allow_host) == STATUS_DEVICE_BUSY && renderer.quarantined() && !renderer.ready(),
        "generic stop or quarantine refuses teardown while a live process borrows ring memory");
    check(section_views == 1 && section_handles == 1 && host_calls == calls,
        "generic shutdown neither unmaps the CPU ring nor destroys its host backing");
    check(renderer.shutdown(false) == STATUS_DEVICE_BUSY && section_views == 1 && host_calls == calls,
        "repeated containment preserves the same borrowed view and closed execution gate");
    check(renderer.close_file(&file) == STATUS_DEVICE_BUSY && file.FsContext == nullptr && section_views == 1,
        "quarantined final handle cleanup retains independent live CPU borrows until process exit");
    renderer.process_exit(PsGetCurrentProcessId());
    check(section_views == 0 && section_handles == 0,
        "true process exit retires quarantined views before address-space destruction");
    check(allocations == retained_allocations && references == retained_references && host_calls == calls,
        "uncertain DMA/session ownership remains retained until reboot after CPU view retirement");
}

void check_backpressure() {
    NativeTransport transport;
    NativeRenderer renderer;
    FILE_OBJECT file{};
    check(NT_SUCCESS(renderer.initialize(transport)), "backpressure fixture initializes native renderer");
    const auto context = open(renderer, file);
    auto header = request(QLV_SUBMIT, context);
    QLV_HEADER reply;
    const unsigned reads_before = sync_reads;
    const unsigned submits_before = host_submits;

    for (size_t index = 0; index < CompletionLedger::capacity; ++index) {
        check(NT_SUCCESS(invoke(renderer, file, header, reply)) && reply.status == QLV_STATUS_SUCCESS,
            "burst submission retains each actual private GPU fence");
    }

    check(sync_reads == reads_before, "below-capacity submissions retain leases without eager host counter reads");
    check(host_submits - submits_before == 2 * CompletionLedger::capacity,
        "lazy polling preserves every user packet and separate genuine lifetime fence");

    const unsigned before = delays;
    complete_on_delay = true;
    check(NT_SUCCESS(invoke(renderer, file, header, reply)) && reply.status == QLV_STATUS_SUCCESS && delays > before,
        "full ledger waits for observed oldest fence instead of returning spurious out of memory");
    complete_on_delay = false;
    complete_all = false;
    check(NT_SUCCESS(renderer.close_file(&file)), "accepted burst session closes through actual host cleanup");
    check(NT_SUCCESS(renderer.shutdown()), "burst cleanup preserves normal shutdown");
}

void check_fused_submission() {
    NativeTransport transport;
    NativeRenderer renderer;
    FILE_OBJECT file{};
    check(NT_SUCCESS(renderer.initialize(transport)), "fused fixture initializes native renderer");
    const auto context = open(renderer, file);
    QLV_HEADER reply;
    const unsigned before = host_submits;
    fuse_requests = true;

    for (unsigned index = 0; index < 2; ++index) {
        check(NT_SUCCESS(invoke(renderer, file, request(QLV_SUBMIT, context), reply)) &&
            reply.status == QLV_STATUS_SUCCESS && reply.offset == 0,
            "fused host acceptance preserves the caller's zero-sync reply");
    }

    fuse_requests = false;
    check(host_submits - before == 2 && private_syncs[0] != 0 &&
        private_points[0] != 0,
        "two caller submits retain two genuine private points with one host request each");
    complete_all = true;
    check(NT_SUCCESS(renderer.close_file(&file)), "fused session drains actual completion on normal close");
    complete_all = false;
    check(NT_SUCCESS(renderer.shutdown()), "fused session retires all normal owners");
}

void check_interleaved_lazy_completion() {
    NativeTransport transport;
    NativeRenderer renderer;
    FILE_OBJECT file{};
    check(NT_SUCCESS(renderer.initialize(transport)), "interleaved completion fixture initializes renderer");
    const auto context = open(renderer, file);
    const auto resource = blob(renderer, file, context);
    const auto lease = register_image(renderer, file, context, resource);
    const unsigned reads_before = sync_reads, submits_before = host_submits;
    bool accepted = true;
    QLV_HEADER reply;
    for (unsigned index = 0; index < 12; ++index) {
        auto header = request(QLV_SUBMIT, context);
        header.value = index % 2;
        accepted = accepted && NT_SUCCESS(invoke(renderer, file, header, reply)) &&
            reply.status == QLV_STATUS_SUCCESS;
    }
    check(accepted && sync_reads == reads_before && host_submits - submits_before == 24,
        "two independent queued rings retain all private fences without eager READ_SYNC");
    check(private_syncs[0] != 0 && private_syncs[1] != 0 && private_syncs[0] != private_syncs[1] &&
        private_points[0] == 6 && private_points[1] == 6,
        "each ring has a distinct real lifetime sync and monotonic issued values");

    // The later ring is complete, but the first global receipt is on ring0.
    // Actual production drain must retain the image until that earlier ring
    // also becomes observed complete, rather than treating the newest ring as
    // proof for the globally ordered prefix.
    complete_all = false;
    completed_token = private_syncs[1];
    complete_on_delay = true;
    const unsigned delays_before = delays;
    NativeImageLease image;
    check(NT_SUCCESS(renderer.resolve_image(lease, current_process, image)) &&
        image.resource == resource && delays > delays_before && sync_reads >= reads_before + 3,
        "out-of-order ring completion cannot prematurely release an imported image source");
    complete_on_delay = false;
    complete_all = false;
    completed_token = 0;
    const unsigned drained_reads = sync_reads;
    auto next = request(QLV_SUBMIT, context);
    check(NT_SUCCESS(invoke(renderer, file, next, reply)) && reply.status == QLV_STATUS_SUCCESS &&
        sync_reads == drained_reads && private_points[0] == 7,
        "a retired prefix accepts the next real point without resetting sequence or eager polling");
    complete_all = true;
    check(NT_SUCCESS(renderer.resolve_image(lease, current_process, image)) &&
        sync_reads == drained_reads + 1, "image use still observes the new private fence before publication");
    check(NT_SUCCESS(renderer.close_file(&file)) && NT_SUCCESS(renderer.shutdown()),
        "interleaved lazy completion retires every normal session owner");
    complete_all = false;
    completed_token = 0;
}

void check_failed_lazy_poll() {
    NativeTransport transport;
    NativeRenderer renderer;
    FILE_OBJECT file{};
    check(NT_SUCCESS(renderer.initialize(transport)), "failed lazy poll fixture initializes renderer");
    const auto context = open(renderer, file);
    const auto resource = blob(renderer, file, context);
    const auto lease = register_image(renderer, file, context, resource);
    QLV_HEADER reply;
    fail_operation = QLV_READ_SYNC;
    const unsigned reads_before = sync_reads, resets_before = resets;
    check(NT_SUCCESS(invoke(renderer, file, request(QLV_SUBMIT, context), reply)) &&
        reply.status == QLV_STATUS_SUCCESS && sync_reads == reads_before,
        "acceptance does not depend on a speculative completion query");
    NativeImageLease image;
    check(renderer.resolve_image(lease, current_process, image) == STATUS_DATA_ERROR &&
        image.context == 0 && resets == resets_before + 1,
        "authoritative drain error still resets the generation without exposing an image");
    fail_operation = 0;
    check(!NT_SUCCESS(renderer.resolve_image(lease, current_process, image)),
        "failed lazy poll cannot resurrect an old image lease");
    check(NT_SUCCESS(renderer.close_file(&file)) && NT_SUCCESS(renderer.shutdown()),
        "confirmed reset after lazy poll failure retires normal ownership");
}

void check_failed_producer() {
    NativeTransport transport;
    NativeRenderer renderer;
    FILE_OBJECT file{};
    check(NT_SUCCESS(renderer.initialize(transport)), "failed producer fixture initializes renderer");
    const auto context = open(renderer, file);
    const auto resource = blob(renderer, file, context);
    const auto lease = register_image(renderer, file, context, resource);
    QLV_HEADER reply;
    check(NT_SUCCESS(invoke(renderer, file, request(QLV_SUBMIT, context), reply)) &&
        reply.status == QLV_STATUS_SUCCESS, "failed producer was accepted but never reported complete");
    const unsigned before = resets;
    NativeImageLease image;
    check(renderer.resolve_image(lease, current_process, image) == STATUS_IO_TIMEOUT &&
        image.context == 0 && resets == before + 1, "producer timeout resets actual generation without publishing consumer source");
    check(!NT_SUCCESS(renderer.resolve_image(lease, current_process, image)),
        "reset generation cannot resurrect old imported image lease");
    check(NT_SUCCESS(renderer.close_file(&file)) && NT_SUCCESS(renderer.shutdown()),
        "confirmed reset allows lost file and process ownership to retire");
}
// Ring_batching: one ioctl_exchange_batch runs its packets as ordinary
// exchanges in order inside one transport batch, replies land one header per
// packet in the shared METHOD_BUFFERED buffer, and a malformed batch does nothing.
unsigned batch_begins = 0, batch_ends = 0, batched_calls = 0;
bool batch_open = false;

size_t append_packet(uint8_t* buffer, size_t capacity, size_t offset, QLV_HEADER header,
                     const QLV_SYNC_POINT* point) noexcept {
    ExchangeReply encoded;
    encoded.header = header;
    encoded.header.payload_bytes = 0;
    size_t written = 0;
    const uint32_t payload = point != nullptr ? static_cast<uint32_t>(QLV_SYNC_POINT_BYTES) : 0;

    if (offset > capacity || capacity - offset < QLV_HEADER_BYTES + payload ||
        encode_exchange_reply(encoded, {buffer + offset, capacity - offset}, written) != ExchangeStatus::validated) {
        check(false, "batch fixture packet fits its buffer");
        return offset;
    }
    rx7900::sdma::CommandBuffer packet(buffer + offset, capacity - offset);
    packet.write_u32(QLV_PAYLOAD_BYTES_OFFSET, payload);
    if (point != nullptr) {
        memcpy(buffer + offset + QLV_HEADER_BYTES, point, sizeof(*point));
    }
    return offset + QLV_HEADER_BYTES + payload;
}

QLV_HEADER batch_reply(const uint8_t* buffer, size_t index) noexcept {
    QLV_HEADER reply{};
    memcpy(&reply, buffer + index * QLV_HEADER_BYTES, sizeof(reply));
    return reply;
}

void check_exchange_batch() {
    NativeTransport transport;
    NativeRenderer renderer;
    FILE_OBJECT file{};
    check(NT_SUCCESS(renderer.initialize(transport)), "batch fixture initializes renderer");
    const auto context = open(renderer, file);
    const auto fence = user_sync(renderer, file, context);
    complete_all = true;
    auto reset = request(QLV_WRITE_SYNC, context);
    reset.resource = fence;
    auto signal = request(QLV_SUBMIT, context);
    signal.offset = 1;
    signal.payload_bytes = static_cast<uint32_t>(QLV_SYNC_POINT_BYTES);
    const QLV_SYNC_POINT point{fence, 7};
    const auto roundtrip = request(QLV_SUBMIT, context);
    uint8_t buffer[(exchange_batch_packets + 1) * (QLV_HEADER_BYTES + QLV_SYNC_POINT_BYTES)]{};
    size_t bytes = append_packet(buffer, sizeof(buffer), 0, reset, nullptr);
    bytes = append_packet(buffer, sizeof(buffer), bytes, signal, &point);
    bytes = append_packet(buffer, sizeof(buffer), bytes, roundtrip, nullptr);
    const unsigned calls = host_calls, submits = host_submits, begins = batch_begins, batched = batched_calls;
    size_t written = 0;
    // METHOD_BUFFERED: the same buffer is input and output.
    check(NT_SUCCESS(renderer.exchange_batch(&file, current_process, {buffer, bytes}, {buffer, sizeof(buffer)}, written)) &&
        written == 3 * QLV_HEADER_BYTES, "a three-packet batch returns one reply header per packet");
    const QLV_HEADER replies[3] = {batch_reply(buffer, 0), batch_reply(buffer, 1), batch_reply(buffer, 2)};
    check(replies[0].operation == QLV_WRITE_SYNC && replies[1].operation == QLV_SUBMIT &&
        replies[2].operation == QLV_SUBMIT && replies[0].status == QLV_STATUS_SUCCESS &&
        replies[1].status == QLV_STATUS_SUCCESS && replies[2].status == QLV_STATUS_SUCCESS &&
        replies[1].offset == 1 && replies[2].offset == 0 && replies[0].context == context,
        "replies are in packet order with each packet's own identity");
    check(host_submits == submits + 4 && private_points[0] != 0 && points[0].token != 0,
        "both SUBMITs and their private fences ran; the signal was parsed from intact bytes");
    bool recorded = false;
    for (const auto& entry : points) { recorded = recorded || (entry.token == fence && entry.value == 7); }
    check(recorded, "the signal point of the middle packet reached the host unchanged");
    check(batch_begins == begins + 1 && batch_ends == batch_begins && !batch_open &&
        batched_calls - batched == host_calls - calls && host_calls > calls,
        "every host exchange of the batch ran inside one transport batch");

    const unsigned before = host_calls, begun = batch_begins;
    auto expect_refused = [&](size_t length, size_t capacity, NTSTATUS expected, const char* text) {
        size_t ignored = 0;
        check(renderer.exchange_batch(&file, current_process, {buffer, length}, {buffer, capacity}, ignored) == expected &&
            ignored == 0 && host_calls == before && batch_begins == begun, text);
    };
    bytes = 0;
    for (uint32_t index = 0; index <= exchange_batch_packets; ++index) {
        bytes = append_packet(buffer, sizeof(buffer), bytes, roundtrip, nullptr);
    }
    expect_refused(bytes, sizeof(buffer), STATUS_INVALID_PARAMETER, "more packets than the batch bound do nothing");
    expect_refused(0, sizeof(buffer), STATUS_INVALID_PARAMETER, "an empty batch does nothing");
    expect_refused(QLV_HEADER_BYTES - 1, sizeof(buffer), STATUS_INVALID_PARAMETER, "a torn header does nothing");
    bytes = append_packet(buffer, sizeof(buffer), 0, roundtrip, nullptr);
    bytes = append_packet(buffer, sizeof(buffer), bytes, signal, &point);
    expect_refused(bytes - 1, sizeof(buffer), STATUS_INVALID_PARAMETER, "a truncated payload does nothing");
    expect_refused(bytes, 2 * QLV_HEADER_BYTES - 1, STATUS_BUFFER_TOO_SMALL, "room for every reply is required");
    auto read = request(QLV_READ_SYNC, context);
    read.resource = fence;
    bytes = append_packet(buffer, sizeof(buffer), 0, read, nullptr);
    bytes = append_packet(buffer, sizeof(buffer), bytes, roundtrip, nullptr);
    expect_refused(bytes, sizeof(buffer), STATUS_INVALID_PARAMETER, "only SUBMIT and WRITE_SYNC may precede the last packet");
    bytes = append_packet(buffer, sizeof(buffer), 0, roundtrip, nullptr);
    bytes = append_packet(buffer, sizeof(buffer), bytes, request(QLV_GET_CAPS, context), nullptr);
    expect_refused(bytes, sizeof(buffer), STATUS_INVALID_PARAMETER, "a reply with payload cannot end a batch");
    {
        // A held packet is at most one ring slot; only the last may be larger.
        static uint8_t large[2 * exchange_batch_packet_bytes + 2 * QLV_HEADER_BYTES]{};
        auto big = request(QLV_SUBMIT, context);
        big.bytes = exchange_batch_packet_bytes;
        big.payload_bytes = exchange_batch_packet_bytes;
        ExchangeReply encoded;
        encoded.header = big;
        encoded.header.payload_bytes = 0;
        size_t first = 0;
        encode_exchange_reply(encoded, {large, sizeof(large)}, first);
        rx7900::sdma::CommandBuffer(large, sizeof(large)).write_u32(QLV_PAYLOAD_BYTES_OFFSET, big.payload_bytes);
        const size_t length = append_packet(large, sizeof(large), QLV_HEADER_BYTES + big.payload_bytes,
            roundtrip, nullptr);
        size_t ignored = 0;
        check(renderer.exchange_batch(&file, current_process, {large, length}, {large, sizeof(large)}, ignored) ==
            STATUS_INVALID_PARAMETER && host_calls == before && batch_begins == begun,
            "a held packet larger than a ring slot does nothing");
    }

    bytes = append_packet(buffer, sizeof(buffer), 0, roundtrip, nullptr);
    bytes = append_packet(buffer, sizeof(buffer), bytes, read, nullptr);
    check(NT_SUCCESS(renderer.exchange_batch(&file, current_process, {buffer, bytes}, {buffer, sizeof(buffer)}, written)) &&
        written == 2 * QLV_HEADER_BYTES && batch_reply(buffer, 1).operation == QLV_READ_SYNC &&
        batch_reply(buffer, 1).value == 7, "a held SUBMIT travels with a READ_SYNC that ends the batch");

    fail_operation = QLV_WRITE_SYNC;
    const unsigned submitted = host_submits;
    bytes = append_packet(buffer, sizeof(buffer), 0, reset, nullptr);
    bytes = append_packet(buffer, sizeof(buffer), bytes, roundtrip, nullptr);
    check(NT_SUCCESS(renderer.exchange_batch(&file, current_process, {buffer, bytes}, {buffer, sizeof(buffer)}, written)) &&
        written == QLV_HEADER_BYTES && batch_reply(buffer, 0).status == QLV_STATUS_UNSUPPORTED &&
        host_submits == submitted && batch_ends == batch_begins,
        "a failed packet stops the batch: later packets never run");
    fail_operation = 0;
    PEPROCESS foreign = reinterpret_cast<PEPROCESS>(&process_storage[1]);
    bytes = append_packet(buffer, sizeof(buffer), 0, roundtrip, nullptr);
    check(renderer.exchange_batch(&file, foreign, {buffer, bytes}, {buffer, sizeof(buffer)}, written) == STATUS_ACCESS_DENIED &&
        written == 0 && batch_ends == batch_begins, "a failed first exchange fails the IOCTL itself");
    check(NT_SUCCESS(renderer.close_file(&file)) && NT_SUCCESS(renderer.shutdown()), "batch session retires normally");
    complete_all = false;
}
} // namespace

namespace lightning::reactos {
void NativeTransport::begin_batch() noexcept {
    check(!batch_open, "transport batches never nest");
    batch_open = true;
    ++batch_begins;
}
void NativeTransport::note_submission_wait(bool) noexcept {}
void NativeTransport::end_batch() noexcept {
    check(batch_open, "every transport batch that ends was begun");
    batch_open = false;
    ++batch_ends;
}
bool NativeTransport::aperture(PHYSICAL_ADDRESS& base, uint64_t& bytes) const noexcept {
    base.QuadPart = 0;
    bytes = aperture_capacity;
    return true;
}
NTSTATUS NativeTransport::claim_renderer() noexcept { ready_ = true; return STATUS_SUCCESS; }
// No shared ring in this fixture: drain keeps its sleep-and-poll path.
bool NativeTransport::await_published(uint64_t, uint64_t, uint64_t) noexcept { return false; }
// async-present: private timelines are published only when the scenario says so.
bool NativeTransport::published(uint64_t, uint64_t token) const noexcept {
    if (!publish_private || token == 0) { return false; }
    for (const auto sync : private_syncs) {
        if (sync == token) { return true; }
    }
    return false;
}
NTSTATUS NativeTransport::recover_renderer() noexcept { ++resets; ready_ = true; return STATUS_SUCCESS; }
NTSTATUS NativeTransport::renderer_exchange(rx7900::ByteView packet, size_t capacity,
    const ExchangeSession& session, ExchangeReply& output,
    const PrivateSubmitFence* fence, bool* fused) noexcept {
    if (fused != nullptr) { *fused = false; }
    uint8_t staged[128]{};
    size_t staged_bytes = 0;
    ValidatedExchange fused_request;

    if (fuse_requests && fence != nullptr &&
        snapshot_submit_with_private_fence(packet, capacity, session,
            fence->sync, fence->point, staged, sizeof(staged), fused_request,
            staged_bytes) == ExchangeStatus::validated) {
        packet = {staged, staged_bytes};
        if (fused != nullptr) { *fused = true; }
    }

    ++host_calls;
    if (batch_open) { ++batched_calls; }
    ValidatedExchange validated;

    if (validate_exchange_request(packet, capacity, session, validated) != ExchangeStatus::validated) {
        return STATUS_INVALID_PARAMETER;
    }

    output = {};
    output.header = validated.header();
    auto& header = output.header;
    header.payload_bytes = 0;
    header.status = header.operation == fail_operation ? QLV_STATUS_UNSUPPORTED : QLV_STATUS_SUCCESS;

    if (header.status != QLV_STATUS_SUCCESS) { return STATUS_SUCCESS; }
    if (header.operation == QLV_SUBMIT) { ++host_submits; }
    if (header.operation == QLV_OPEN) { header.context = next_context++; }
    if (header.operation == QLV_CREATE_BLOB || header.operation == QLV_CREATE_SYNC) {
        header.resource = next_resource++;
    }
    if (header.operation == QLV_SUBMIT && header.offset != 0) {
        uint32_t low = 0;
        uint32_t high = 0;
        const size_t point_offset = QLV_HEADER_BYTES +
            static_cast<size_t>(header.offset - 1) * QLV_SYNC_POINT_BYTES;
        packet.read_u32(point_offset, low);
        packet.read_u32(point_offset + sizeof(low), high);
        const uint64_t token = low | (static_cast<uint64_t>(high) << QLV_PCI_WORD_SHIFT);
        packet.read_u32(point_offset + QLV_SYNC_VALUE_OFFSET, low);
        packet.read_u32(point_offset + QLV_SYNC_VALUE_OFFSET + sizeof(low), high);
        remember_point(token, low | (static_cast<uint64_t>(high) << QLV_PCI_WORD_SHIFT));
        private_syncs[header.value] = token;
        private_points[header.value] = low | (static_cast<uint64_t>(high) << QLV_PCI_WORD_SHIFT);
    }
    if (header.operation == QLV_MAP_BLOB) {
        ResourceInfo resource{};
        check(session.lookup(session.resource_owner, header.resource, resource), "host fixture resolves exact mapped resource");
        header.offset = map_offset;
        header.bytes = resource.byte_count;
        header.flags = QLV_MAP_CACHE_CACHED;
    }
    if (header.operation == QLV_UNMAP_BLOB) {
        check(section_views == 0, "host aperture retirement follows the revoked process view");
    }
    if (header.operation == QLV_REGISTER_SCANOUT) { header.value = 0; }
    if (header.operation == QLV_READ_SYNC) {
        ++sync_reads;
        for (auto& point : points) {
            if (point.token == header.resource) {
                header.value = complete_all || completed_token == point.token ? point.value : 0;
                if (partial_token != 0 && point.token == partial_token && !complete_all) {
                    header.value = partial_value;
                }
                // A host timeline never moves backwards once a value was read.
                if (header.value < point.released) { header.value = point.released; }
                point.released = header.value;
                break;
            }
        }
    }
    if (fused != nullptr && *fused) {
        header.offset -= 1;
    }
    return STATUS_SUCCESS;
}
} // namespace lightning::reactos

extern "C" {
ULONG __cdecl DbgPrint(PCSTR, ...) { return 0; }
VOID NTAPI RtlInitUnicodeString(PUNICODE_STRING target, PCWSTR source) {
    const size_t characters = wcslen(source);
    check(characters < MAXUSHORT / sizeof(WCHAR), "fixture native name fits the counted Unicode ABI");
    target->Length = static_cast<USHORT>(characters * sizeof(WCHAR));
    target->MaximumLength = static_cast<USHORT>(target->Length + sizeof(WCHAR));
    target->Buffer = const_cast<PWSTR>(source);
}
PVOID MmHighestUserAddress = reinterpret_cast<PVOID>(UINT32_C(0x7ffeffff));
PVOID NTAPI ExAllocatePoolWithTag(POOL_TYPE, SIZE_T size, ULONG) {
    void* allocation = calloc(1, size);
    if (allocation != nullptr) { ++allocations; }
    return allocation;
}
VOID NTAPI ExFreePoolWithTag(PVOID allocation, ULONG) { if (allocation != nullptr) { --allocations; free(allocation); } }
LONG_PTR FASTCALL ObfReferenceObject(PVOID) { return ++references; }
LONG_PTR FASTCALL ObfDereferenceObject(PVOID) { return --references; }
PEPROCESS NTAPI IoGetCurrentProcess() { return current_process; }
HANDLE NTAPI PsGetCurrentProcessId() { return reinterpret_cast<HANDLE>(current_process); }
KIRQL NTAPI KeGetCurrentIrql() { return PASSIVE_LEVEL; }
ULONGLONG NTAPI KeQueryInterruptTime() { return ++clock_value; }
NTSTATUS NTAPI KeDelayExecutionThread(KPROCESSOR_MODE, BOOLEAN, PLARGE_INTEGER interval) {
    ++delays;
    clock_value += static_cast<ULONGLONG>(-interval->QuadPart);
    if (complete_on_delay) { complete_all = true; }
    return STATUS_SUCCESS;
}
VOID NTAPI KeStackAttachProcess(PKPROCESS, PRKAPC_STATE) { check(false, "fixture never attaches for an unmapped image"); }
VOID NTAPI KeUnstackDetachProcess(PRKAPC_STATE) { check(false, "fixture never detaches for an unmapped image"); }
NTSTATUS NTAPI ZwOpenSection(PHANDLE handle, ACCESS_MASK access, POBJECT_ATTRIBUTES attributes) {
    check(access == (SECTION_MAP_READ | SECTION_MAP_WRITE) &&
        (attributes->Attributes & OBJ_KERNEL_HANDLE) != 0 &&
        wcscmp(attributes->ObjectName->Buffer, L"\\Device\\PhysicalMemory") == 0,
        "mapping opens only the native physical section through a kernel-only handle");
    if (fail_section_open) { return STATUS_ACCESS_DENIED; }
    *handle = reinterpret_cast<HANDLE>(UINT32_C(0x100));
    ++section_opens;
    ++section_handles;
    return STATUS_SUCCESS;
}
NTSTATUS NTAPI ZwMapViewOfSection(HANDLE, HANDLE process, PVOID* address, ULONG_PTR bits,
    SIZE_T commit, PLARGE_INTEGER offset, PSIZE_T bytes, SECTION_INHERIT inheritance,
    ULONG allocation, ULONG protection) {
    check(section_handles == 1 && process == NtCurrentProcess() && bits == 0 && commit == 0 &&
        offset->QuadPart == static_cast<LONGLONG>(map_offset) && *bytes == QLV_PAGE_BYTES && inheritance == ViewUnmap && allocation == 0 &&
        protection == PAGE_READWRITE, "map uses the exact owned span, current process, checked cache and no inherited child view");
    if (fail_section_map) { return STATUS_INSUFFICIENT_RESOURCES; }
    *address = reinterpret_cast<PVOID>(UINT32_C(0x20000000));
    if (alter_section_size) { *bytes += QLV_PAGE_BYTES; }
    ++section_views;
    return STATUS_SUCCESS;
}
NTSTATUS NTAPI ZwUnmapViewOfSection(HANDLE process, PVOID address) {
    check(process == NtCurrentProcess() && address == reinterpret_cast<PVOID>(UINT32_C(0x20000000)) &&
        section_views == 1 && section_handles == 1, "unmap retains the native section while its exact user view retires");
    --section_views;
    return STATUS_SUCCESS;
}
NTSTATUS NTAPI ZwClose(HANDLE handle) {
    check(handle == reinterpret_cast<HANDLE>(UINT32_C(0x100)) && section_handles == 1 && section_views == 0,
        "section close happens only after its user view is gone");
    --section_handles;
    return STATUS_SUCCESS;
}
}

int main() {
    check_blob_capacity();
    check_images();
    check_mappings();
    aperture_capacity = UINT64_C(1024) * 1024 * 1024;
    map_offset = QLV_APERTURE_BYTES + QLV_PAGE_BYTES;
    check_mappings();
    aperture_capacity = QLV_APERTURE_BYTES;
    map_offset = 0;
    check_backpressure();
    check_fused_submission();
    check_interleaved_lazy_completion();
    check_view_changes_do_not_drain();
    check_multi_image_leases();
    check(resets == 0, "normal image lifecycle and bounded backpressure never reset a healthy device");
    check_failed_producer();
    check(allocations == 0 && references == 0, "all native session allocations and process references retire");
    check(resets == 1, "timed-out producer requires confirmed generation reset");
    check_mapped_recovery();
    check(resets == 2 && allocations == 0 && references == 0 && section_handles == 0 && section_views == 0,
        "mapped recovery performs one additional real reset and retires all normal ownership");
    check_failed_lazy_poll();
    check(allocations == 0 && references == 0 && section_handles == 0 && section_views == 0,
        "lazy poll error recovery leaves no normal session, process or mapping owner behind");

    check_shutdown_retains_borrow(false);
    check_shutdown_retains_borrow(true);
    const unsigned held_allocations = allocations;
    check_producer_points();
    check(allocations == held_allocations, "producer-point fixture retires every native owner it created");
    check_reset_before_present();
    check(allocations == held_allocations, "reset-before-present fixture retires every native owner it created");
    check_exchange_batch();
    check(allocations == held_allocations, "batch fixture retires every native owner it created");
    // Last: its final case deliberately contains (quarantines) the renderer.
    check_receipt_pins();
    // Also deliberately contains its second renderer.
    check_power_down_pins();
    printf("%u native renderer checks; %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

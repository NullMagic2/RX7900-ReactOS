/*
Module: rx7900.tests.radeon_2d_ioctl
Purpose: Check the Radeon 2D IOCTL codec, the miniport dispatcher's handle,
ownership, geometry, staging and fence rules, and the display DLL admission
policy against a fake 2D backend.
Created: 2026-10-22
Architecture: Host-only; exercises src/reactos/radeon_2d_abi.hpp,
radeon_2d_ioctl.cpp and radeon_display.hpp without any NT or GPU dependency.
*/
#include "radeon_2d_ioctl.hpp"
#include "radeon_display.hpp"
#include "gdi_engine_reference.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <random>
#include <vector>

namespace {
namespace r2d = rx7900::reactos::radeon2d;
namespace abi = rx7900::reactos::radeon2d::abi;
namespace display = rx7900::reactos::radeon_display;
using rx7900::ByteView;
using rx7900::sdma::CommandBuffer;

unsigned checks = 0;
unsigned failures = 0;

void check(bool result, const char* description) {
    ++checks;
    if (!result) {
        ++failures;
        std::fprintf(stderr, "FAIL: %s\n", description);
    }
}

// ---------------------------------------------------------------------------
// Fake backend: records every call, issues monotonic fences and lets each test
// choose rejected/uncertain outcomes and whether sync proves retirement.
// ---------------------------------------------------------------------------
struct FakeBackend final {
    r2d::BackendStatus next_status = r2d::BackendStatus::ok;
    r2d::BackendStatus sync_status = r2d::BackendStatus::ok;
    r2d::BackendStatus staging_status = r2d::BackendStatus::ok;
    r2d::BackendStatus evict_status = r2d::BackendStatus::ok;
    r2d::BackendStatus restore_status = r2d::BackendStatus::ok;
    unsigned evictions = 0;
    unsigned restores = 0;
    // Raster callbacks (installed by raster_table): last operation and rects.
    unsigned raster_rops = 0;
    unsigned raster_alphas = 0;
    unsigned raster_lines = 0;
    unsigned raster_gradients = 0;
    r2d::RasterRop last_rop{};
    uint32_t last_pattern0 = 0;
    r2d::RasterAlpha last_alpha{};
    r2d::RasterLine last_line{};
    uint8_t last_style0 = 0;
    uint32_t last_line_pattern0 = 0;
    r2d::RasterGradient last_gradient{};
    // Evictions that succeed before evict_status is reported instead.
    unsigned evict_allowance = UINT32_MAX;
    // Live backend surfaces by allocation id (address moves on evict/restore).
    std::map<uint32_t, r2d::EngineSurface> live;
    bool auto_complete = false;
    uint64_t fence = 0;
    uint64_t completed = 0;
    unsigned fills = 0;
    unsigned copies = 0;
    unsigned uploads = 0;
    unsigned readbacks = 0;
    unsigned syncs = 0;
    unsigned surfaces_created = 0;
    unsigned surfaces_destroyed = 0;
    unsigned staging_created = 0;
    unsigned staging_destroyed = 0;
    uint32_t last_color = 0;
    uint32_t last_pitch = 0;
    uint64_t last_target_address = 0;
    uint64_t last_source_address = 0;
    std::vector<r2d::EngineRect> rects;
    std::vector<r2d::CopyRect> copy_rects;
    std::vector<r2d::TransferRect> transfer_rects;
    uint8_t staging_memory[abi::max_sessions][4096]{};
    uint32_t next_allocation = 1;

    r2d::BackendStatus finish(uint64_t& output) {
        if (next_status != r2d::BackendStatus::ok) {
            output = 0;
            return next_status;
        }

        output = ++fence;
        if (auto_complete) {
            completed = fence;
        }

        return r2d::BackendStatus::ok;
    }
};

FakeBackend& fake(void* context) { return *static_cast<FakeBackend*>(context); }

r2d::BackendStatus fake_fill(void* context, const r2d::EngineSurface& target, uint32_t color,
                             const r2d::EngineRect* rects, uint32_t count, uint64_t& fence) noexcept {
    FakeBackend& backend = fake(context);
    ++backend.fills;
    backend.last_color = color;
    backend.last_target_address = target.gpu_address;
    backend.rects.assign(rects, rects + count);
    return backend.finish(fence);
}

r2d::BackendStatus fake_copy(void* context, const r2d::EngineSurface& target,
                             const r2d::EngineSurface& source, const r2d::CopyRect* rects,
                             uint32_t count, uint64_t& fence) noexcept {
    FakeBackend& backend = fake(context);
    ++backend.copies;
    backend.last_target_address = target.gpu_address;
    backend.last_source_address = source.gpu_address;
    backend.copy_rects.assign(rects, rects + count);
    return backend.finish(fence);
}

r2d::BackendStatus fake_upload(void* context, const r2d::EngineSurface& target,
                               const r2d::StagingBuffer&, uint32_t pitch,
                               const r2d::TransferRect* rects, uint32_t count,
                               uint64_t& fence) noexcept {
    FakeBackend& backend = fake(context);
    ++backend.uploads;
    backend.last_pitch = pitch;
    backend.last_target_address = target.gpu_address;
    backend.transfer_rects.assign(rects, rects + count);
    return backend.finish(fence);
}

r2d::BackendStatus fake_readback(void* context, const r2d::EngineSurface& source,
                                 const r2d::StagingBuffer&, uint32_t pitch,
                                 const r2d::TransferRect* rects, uint32_t count,
                                 uint64_t& fence) noexcept {
    FakeBackend& backend = fake(context);
    ++backend.readbacks;
    backend.last_pitch = pitch;
    backend.last_source_address = source.gpu_address;
    backend.transfer_rects.assign(rects, rects + count);
    return backend.finish(fence);
}

r2d::BackendStatus fake_surface_create(void* context, uint32_t width, uint32_t height, uint32_t format,
                                       r2d::EngineSurface& output, uint32_t& allocation) noexcept {
    FakeBackend& backend = fake(context);
    if (backend.next_status != r2d::BackendStatus::ok) {
        return backend.next_status;
    }

    ++backend.surfaces_created;
    allocation = backend.next_allocation++;
    output = {0x100000000ULL + allocation * 0x1000000ULL, width, height,
              ((width * 4U) + 255U) & ~255U, format};
    backend.live[allocation] = output;
    return r2d::BackendStatus::ok;
}

void fake_surface_destroy(void* context, uint32_t allocation) noexcept {
    ++fake(context).surfaces_destroyed;
    fake(context).live.erase(allocation);
}

// Evicted surfaces move to a "system memory" address range at 0x3'0000'0000.
r2d::BackendStatus fake_surface_evict(void* context, uint32_t allocation, r2d::EngineSurface& output) noexcept {
    FakeBackend& backend = fake(context);
    auto found = backend.live.find(allocation);
    if (backend.evict_allowance == 0 && backend.evict_status == r2d::BackendStatus::ok) {
        return r2d::BackendStatus::rejected;
    }

    if (backend.evict_status != r2d::BackendStatus::ok || found == backend.live.end()) {
        return backend.evict_status == r2d::BackendStatus::ok ? r2d::BackendStatus::rejected :
                                                                backend.evict_status;
    }

    ++backend.evictions;
    if (backend.evict_allowance != UINT32_MAX) {
        --backend.evict_allowance;
    }

    found->second.gpu_address = 0x300000000ULL + allocation * 0x1000000ULL;
    output = found->second;
    return r2d::BackendStatus::ok;
}

r2d::BackendStatus fake_surface_restore(void* context, uint32_t allocation, r2d::EngineSurface& output) noexcept {
    FakeBackend& backend = fake(context);
    auto found = backend.live.find(allocation);
    if (backend.restore_status != r2d::BackendStatus::ok || found == backend.live.end()) {
        return backend.restore_status == r2d::BackendStatus::ok ? r2d::BackendStatus::rejected :
                                                                  backend.restore_status;
    }

    ++backend.restores;
    found->second.gpu_address = 0x100000000ULL + allocation * 0x1000000ULL;
    output = found->second;
    return r2d::BackendStatus::ok;
}

r2d::BackendStatus fake_staging_create(void* context, uint32_t bytes, r2d::StagingBuffer& output) noexcept {
    FakeBackend& backend = fake(context);
    if (backend.staging_status != r2d::BackendStatus::ok || bytes > sizeof(backend.staging_memory[0])) {
        return r2d::BackendStatus::rejected;
    }

    const uint32_t slot = backend.staging_created % abi::max_sessions;
    ++backend.staging_created;
    output.gpu_address = 0x8000000ULL + slot * 0x10000ULL;
    output.cpu_address = reinterpret_cast<uintptr_t>(backend.staging_memory[slot]);
    output.bytes = bytes;
    output.allocation = slot;
    return r2d::BackendStatus::ok;
}

void fake_staging_destroy(void* context, const r2d::StagingBuffer&) noexcept {
    ++fake(context).staging_destroyed;
}

r2d::BackendStatus fake_sync(void* context, uint64_t fence) noexcept {
    FakeBackend& backend = fake(context);
    ++backend.syncs;
    if (backend.sync_status == r2d::BackendStatus::ok && fence > backend.completed) {
        backend.completed = fence;
    }

    return backend.sync_status;
}

uint64_t fake_completed(void* context) noexcept { return fake(context).completed; }

r2d::BackendStatus fake_raster_rop(void* context, const r2d::RasterRop& operation, const r2d::EngineRect* rects,
                                   uint32_t count, uint64_t& fence) noexcept {
    FakeBackend& backend = fake(context);
    ++backend.raster_rops;
    backend.last_rop = operation;
    backend.last_pattern0 = operation.pattern != nullptr ? operation.pattern[0] : 0;
    backend.rects.assign(rects, rects + count);
    return backend.finish(fence);
}

r2d::BackendStatus fake_raster_alpha(void* context, const r2d::RasterAlpha& operation, const r2d::EngineRect* rects,
                                     uint32_t count, uint64_t& fence) noexcept {
    FakeBackend& backend = fake(context);
    ++backend.raster_alphas;
    backend.last_alpha = operation;
    backend.rects.assign(rects, rects + count);
    return backend.finish(fence);
}

r2d::BackendStatus fake_raster_line(void* context, const r2d::RasterLine& operation, const r2d::EngineRect* rects,
                                    uint32_t count, uint64_t& fence) noexcept {
    FakeBackend& backend = fake(context);
    ++backend.raster_lines;
    backend.last_line = operation;
    backend.last_style0 = operation.style != nullptr ? operation.style[0] : 0;
    backend.last_line_pattern0 = operation.pattern != nullptr ? operation.pattern[0] : 0;
    backend.rects.assign(rects, rects + count);
    return backend.finish(fence);
}

r2d::BackendStatus fake_raster_gradient(void* context, const r2d::RasterGradient& operation,
                                        const r2d::EngineRect* rects, uint32_t count, uint64_t& fence) noexcept {
    FakeBackend& backend = fake(context);
    ++backend.raster_gradients;
    backend.last_gradient = operation;
    backend.rects.assign(rects, rects + count);
    return backend.finish(fence);
}

r2d::Blit2dBackend table(FakeBackend& backend);

// The fake table plus every raster callback.
r2d::Blit2dBackend raster_table(FakeBackend& backend) {
    r2d::Blit2dBackend value = table(backend);
    value.raster_operations = abi::raster_operations;
    value.raster_rop = fake_raster_rop;
    value.raster_alpha = fake_raster_alpha;
    value.raster_line = fake_raster_line;
    value.raster_gradient = fake_raster_gradient;
    return value;
}

r2d::Blit2dBackend table(FakeBackend& backend) {
    r2d::Blit2dBackend value{};
    value.context = &backend;
    value.fill = fake_fill;
    value.copy = fake_copy;
    value.upload = fake_upload;
    value.readback = fake_readback;
    value.surface_create = fake_surface_create;
    value.surface_destroy = fake_surface_destroy;
    value.staging_create = fake_staging_create;
    value.staging_destroy = fake_staging_destroy;
    value.sync = fake_sync;
    value.completed = fake_completed;
    value.surface_evict = fake_surface_evict;
    value.surface_restore = fake_surface_restore;
    return value;
}

constexpr uint32_t screen_width = 1920;
constexpr uint32_t screen_height = 1080;
constexpr uint32_t screen_pitch = 7680;
constexpr uint32_t staging_size = 4096;
constexpr uint64_t primary_address = 0x8000000000ULL;

r2d::EngineSurface primary() {
    return {primary_address, screen_width, screen_height, screen_pitch, abi::format_xrgb8888};
}

// ---------------------------------------------------------------------------
// Request helpers (encode exactly like the display DLL does).
// ---------------------------------------------------------------------------
struct Buffer final {
    uint8_t bytes[abi::max_request_bytes]{};
    uint32_t size = 0;
    ByteView view() const { return ByteView(bytes, size); }
};

Buffer open_request() {
    Buffer buffer;
    buffer.size = abi::open_session_request_bytes;
    abi::encode_open_session(CommandBuffer(buffer.bytes, buffer.size));
    return buffer;
}

Buffer close_request(abi::Header header) {
    Buffer buffer;
    buffer.size = abi::close_session_request_bytes;
    abi::encode_close_session(CommandBuffer(buffer.bytes, buffer.size), header);
    return buffer;
}

Buffer assert_request(abi::Header header, bool enable) {
    Buffer buffer;
    buffer.size = abi::assert_primary_request_bytes;
    abi::AssertPrimaryCommand command{};
    command.session = header;
    command.enable = enable ? 1U : 0U;
    abi::encode_assert_primary(CommandBuffer(buffer.bytes, buffer.size), command);
    return buffer;
}

abi::RectList rects(std::initializer_list<abi::Rect> list) {
    abi::RectList value{};
    for (const abi::Rect& rect : list) {
        value.rects[value.count++] = rect;
    }

    return value;
}

Buffer fill_request(abi::Header header, uint32_t surface, uint32_t color, const abi::RectList& list) {
    Buffer buffer;
    buffer.size = abi::fill_request_bytes;
    abi::FillCommand command{};
    command.session = header;
    command.surface = surface;
    command.color = color;
    command.rects = list;
    check(abi::encode_fill(CommandBuffer(buffer.bytes, buffer.size), command), "fill encodes");
    return buffer;
}

Buffer copy_request(abi::Header header, uint32_t target, uint32_t source, int32_t dx, int32_t dy,
                    const abi::RectList& list) {
    Buffer buffer;
    buffer.size = abi::copy_request_bytes;
    abi::CopyCommand command{};
    command.session = header;
    command.target = target;
    command.source = source;
    command.source_dx = dx;
    command.source_dy = dy;
    command.rects = list;
    check(abi::encode_copy(CommandBuffer(buffer.bytes, buffer.size), command), "copy encodes");
    return buffer;
}

Buffer transfer_request(abi::Header header, uint32_t surface, uint32_t offset, uint32_t pitch,
                        int32_t origin_x, int32_t origin_y, const abi::RectList& list) {
    Buffer buffer;
    buffer.size = abi::transfer_request_bytes;
    abi::TransferCommand command{};
    command.session = header;
    command.surface = surface;
    command.staging_offset = offset;
    command.staging_pitch = pitch;
    command.origin_x = origin_x;
    command.origin_y = origin_y;
    command.rects = list;
    check(abi::encode_transfer(CommandBuffer(buffer.bytes, buffer.size), command), "transfer encodes");
    return buffer;
}

Buffer sync_request(abi::Header header, uint64_t fence) {
    Buffer buffer;
    buffer.size = abi::sync_request_bytes;
    abi::SyncCommand command{};
    command.session = header;
    command.fence = fence;
    abi::encode_sync(CommandBuffer(buffer.bytes, buffer.size), command);
    return buffer;
}

Buffer surface_create_request(abi::Header header, uint32_t width, uint32_t height) {
    Buffer buffer;
    buffer.size = abi::surface_create_request_bytes;
    abi::SurfaceCreateCommand command{};
    command.session = header;
    command.width = width;
    command.height = height;
    command.format = abi::format_xrgb8888;
    abi::encode_surface_create(CommandBuffer(buffer.bytes, buffer.size), command);
    return buffer;
}

Buffer surface_destroy_request(abi::Header header, uint32_t surface) {
    Buffer buffer;
    buffer.size = abi::surface_destroy_request_bytes;
    abi::SurfaceDestroyCommand command{};
    command.session = header;
    command.surface = surface;
    abi::encode_surface_destroy(CommandBuffer(buffer.bytes, buffer.size), command);
    return buffer;
}

bool receipt_of(const r2d::Dispatcher& dispatcher, r2d::DispatchResult result, abi::Receipt& receipt) {
    return result.status == r2d::DispatchStatus::success && result.reply_bytes == abi::receipt_bytes &&
           abi::decode_receipt(dispatcher.reply(), receipt);
}

uint32_t outcome(r2d::Dispatcher& dispatcher, uint32_t code, const Buffer& request) {
    abi::Receipt receipt{};
    const auto result = dispatcher.dispatch(code, request.view(), abi::receipt_bytes);
    return receipt_of(dispatcher, result, receipt) ? receipt.outcome : 0xffffffffU;
}

bool open(r2d::Dispatcher& dispatcher, abi::SessionGrant& grant) {
    const auto result = dispatcher.dispatch(abi::ioctl_open_session, open_request().view(),
                                            abi::session_grant_bytes);
    return result.status == r2d::DispatchStatus::success &&
           abi::decode_session_grant(dispatcher.reply(), grant);
}

bool open_primary(r2d::Dispatcher& dispatcher, abi::SessionGrant& grant) {
    return open(dispatcher, grant) &&
           outcome(dispatcher, abi::ioctl_assert_primary, assert_request(grant.session, true)) ==
               abi::outcome_completed;
}

// ---------------------------------------------------------------------------
void codec_tests() {
    check(abi::ioctl_query == 0x0023e800U && abi::ioctl_fill == 0x0023e810U &&
          abi::ioctl_sync == 0x0023e828U, "documented device-specific CTL_CODE values");
    check(abi::is_radeon_2d_ioctl(abi::ioctl_copy) && !abi::is_radeon_2d_ioctl(0x00232420U) &&
          abi::is_radeon_2d_ioctl(abi::ioctl_sync + 4U) && abi::is_radeon_2d_ioctl(abi::ioctl_gradient) &&
          abi::is_radeon_2d_ioctl(abi::ioctl_last) && !abi::is_radeon_2d_ioctl(abi::ioctl_last + 4U) &&
          !abi::is_radeon_2d_ioctl(abi::ioctl_fill + 1U),
          "the private range 0xa00..0xa3f is claimed, nothing beyond it");
    check(abi::ioctl_rop == 0x0023e840U && abi::ioctl_gradient == 0x0023e858U &&
          abi::ioctl_last == 0x0023e8fcU && abi::operation_bit(abi::ioctl_last) == (uint64_t{1} << 63) &&
          (abi::implemented_operations & abi::operation_bit(abi::ioctl_rop)) == 0 &&
          abi::ioctl_stretch_rop == 0x0023e85cU && abi::is_raster_ioctl(abi::ioctl_stretch_rop) &&
          !abi::is_raster_ioctl(abi::ioctl_stretch_rop + 4U) && abi::raster_operations == 0xff0000ULL &&
          abi::version == 6, "raster codes and operation mask (raster bits only with a compute backend)");

    abi::Header header{0x0102U, 0xdeadbeefU};
    abi::RectList list = rects({{1, 2, 3, 4}, {-5, -6, 7, 8}});
    Buffer fill = fill_request(header, 7, 0x80ff00ffU, list);
    abi::FillCommand decoded{};
    check(abi::decode_fill(fill.view(), decoded) && decoded.session.session == 0x0102U &&
          decoded.session.session_key == 0xdeadbeefU && decoded.surface == 7 &&
          decoded.color == 0x80ff00ffU && decoded.rects.count == 2 &&
          decoded.rects.rects[1].left == -5 && decoded.rects.rects[1].top == -6,
          "fill round-trips including signed coordinates");
    check(!abi::decode_fill(ByteView(fill.bytes, fill.size - 1), decoded), "truncated fill rejected");
    Buffer bad = fill;
    bad.bytes[4] = 3;
    check(!abi::decode_fill(bad.view(), decoded), "version 3 frame rejected");
    bad.bytes[4] = 4;
    check(!abi::decode_fill(bad.view(), decoded), "version 4 frame rejected");
    bad.bytes[4] = 5;
    check(!abi::decode_fill(bad.view(), decoded), "version 5 frame rejected");
    bad.bytes[4] = 7;
    check(!abi::decode_fill(bad.view(), decoded), "unknown version rejected");
    bad = fill;
    bad.bytes[0] = 0;
    check(!abi::decode_fill(bad.view(), decoded), "size field must equal the frame");
    bad = fill;
    bad.bytes[28] = 1;
    check(!abi::decode_fill(bad.view(), decoded), "nonzero reserved rejected");
    bad = fill;
    bad.bytes[abi::fill_rects_offset + 2 * abi::rect_bytes + 3] = 1;
    check(!abi::decode_fill(bad.view(), decoded), "nonzero padding after rect_count rejected");
    bad = fill;
    bad.bytes[24] = 0;
    check(!abi::decode_fill(bad.view(), decoded), "zero rectangles rejected");
    bad = fill;
    bad.bytes[24] = 65;
    check(!abi::decode_fill(bad.view(), decoded), "more than 64 rectangles rejected");
    abi::RectList empty{};
    check(!abi::encode_fill(CommandBuffer(bad.bytes, abi::fill_request_bytes), abi::FillCommand{}) &&
          empty.count == 0, "encoder refuses an empty rectangle list");

    Buffer copy = copy_request(header, 1, 2, -3, 4, list);
    abi::CopyCommand copy_decoded{};
    check(abi::decode_copy(copy.view(), copy_decoded) && copy_decoded.source_dx == -3 &&
          copy_decoded.source_dy == 4 && copy_decoded.target == 1 && copy_decoded.source == 2,
          "copy round-trips signed deltas");
    Buffer transfer = transfer_request(header, 3, 64, 256, -1, -2, list);
    abi::TransferCommand transfer_decoded{};
    check(abi::decode_transfer(transfer.view(), transfer_decoded) && transfer_decoded.staging_offset == 64 &&
          transfer_decoded.staging_pitch == 256 && transfer_decoded.origin_x == -1 &&
          transfer_decoded.origin_y == -2, "transfer round-trips");

    abi::Receipt receipt{abi::outcome_lost, 0, 0x1122334455667788ULL, 0x99ULL};
    uint8_t receipt_bytes[abi::receipt_bytes]{};
    abi::Receipt receipt_decoded{};
    check(abi::encode_receipt(CommandBuffer(receipt_bytes, sizeof(receipt_bytes)), receipt) &&
          abi::decode_receipt(ByteView(receipt_bytes, sizeof(receipt_bytes)), receipt_decoded) &&
          receipt_decoded.fence == 0x1122334455667788ULL && receipt_decoded.completed_fence == 0x99ULL &&
          receipt_decoded.outcome == abi::outcome_lost, "receipt round-trips 64-bit fences");
    receipt_bytes[8] = 3;
    check(abi::decode_receipt(ByteView(receipt_bytes, sizeof(receipt_bytes)), receipt_decoded) &&
          receipt_decoded.outcome == abi::outcome_surface_lost, "surface_lost outcome decodes");
    receipt_bytes[8] = 4;
    check(!abi::decode_receipt(ByteView(receipt_bytes, sizeof(receipt_bytes)), receipt_decoded),
          "unknown receipt outcome rejected");
    // Version 6: +12 names the surface of an outcome_surface_lost, nothing else.
    abi::Receipt named{abi::outcome_surface_lost, 0x00000a05U, 0, 7};
    check(abi::encode_receipt(CommandBuffer(receipt_bytes, sizeof(receipt_bytes)), named) &&
          abi::decode_receipt(ByteView(receipt_bytes, sizeof(receipt_bytes)), receipt_decoded) &&
          receipt_decoded.lost_surface == 0x00000a05U && receipt_decoded.outcome == abi::outcome_surface_lost,
          "surface_lost receipt round-trips the lost handle");
    receipt_bytes[8] = static_cast<uint8_t>(abi::outcome_completed);
    check(!abi::decode_receipt(ByteView(receipt_bytes, sizeof(receipt_bytes)), receipt_decoded),
          "a lost handle on any other outcome is refused");
    named.outcome = abi::outcome_declined;
    check(!abi::encode_receipt(CommandBuffer(receipt_bytes, sizeof(receipt_bytes)), named),
          "encoder refuses a lost handle outside surface_lost");

    abi::Capabilities capabilities{};
    capabilities.flags = abi::capability_eviction;
    capabilities.session_surface_bytes = 0x0000000110000000ULL;
    capabilities.device_surface_bytes = 0x0000000220000000ULL;
    capabilities.used_surface_bytes = 0x1234ULL;
    capabilities.operations = abi::implemented_operations;
    capabilities.recovery_generation = 3;
    capabilities.recovery_state = abi::recovery_disabled_quiescent;
    capabilities.reset_lost_surfaces = 0x01020304U;
    capabilities.reset_restored_surfaces = 0x05060708U;
    uint8_t capability_bytes[abi::capabilities_bytes]{};
    abi::Capabilities capabilities_decoded{};
    check(abi::capabilities_bytes == 112 &&
          abi::encode_capabilities(CommandBuffer(capability_bytes, sizeof(capability_bytes)), capabilities) &&
          abi::decode_capabilities(ByteView(capability_bytes, sizeof(capability_bytes)), capabilities_decoded) &&
          capabilities_decoded.session_surface_bytes == 0x0000000110000000ULL &&
          capabilities_decoded.device_surface_bytes == 0x0000000220000000ULL &&
          capabilities_decoded.used_surface_bytes == 0x1234ULL &&
          capabilities_decoded.operations == abi::implemented_operations &&
          capabilities_decoded.recovery_generation == 3 &&
          capabilities_decoded.recovery_state == abi::recovery_disabled_quiescent &&
          capabilities_decoded.reset_lost_surfaces == 0x01020304U &&
          capabilities_decoded.reset_restored_surfaces == 0x05060708U,
          "version 6 capabilities round-trip budgets, operation mask and recovery fields");
    capability_bytes[100] = 4;
    check(!abi::decode_capabilities(ByteView(capability_bytes, sizeof(capability_bytes)), capabilities_decoded),
          "an unknown recovery state is refused");
    check(!abi::decode_capabilities(ByteView(capability_bytes, 96), capabilities_decoded),
          "a version 5 sized capability frame is refused");
    check(!abi::decode_capabilities(ByteView(capability_bytes, 64), capabilities_decoded),
          "a version 1 sized capability frame is refused");

    abi::SessionGrant grant{};
    grant.session = header;
    grant.staging_address = 0xfedcba9876543210ULL;
    uint8_t grant_bytes[abi::session_grant_bytes]{};
    abi::SessionGrant grant_decoded{};
    check(abi::encode_session_grant(CommandBuffer(grant_bytes, sizeof(grant_bytes)), grant) &&
          abi::decode_session_grant(ByteView(grant_bytes, sizeof(grant_bytes)), grant_decoded) &&
          grant_decoded.staging_address == 0xfedcba9876543210ULL, "grant round-trips staging mapping");
    check(abi::detail::to_signed(0xffffffffU) == -1 && abi::detail::to_signed(0x80000000U) == INT32_MIN &&
          abi::detail::to_signed(0x7fffffffU) == INT32_MAX, "two's-complement decode");
}

void configuration_tests() {
    FakeBackend backend;
    r2d::Dispatcher dispatcher;
    auto result = dispatcher.dispatch(abi::ioctl_query, ByteView(nullptr, 0), abi::capabilities_bytes);
    abi::Capabilities capabilities{};
    check(result.status == r2d::DispatchStatus::success &&
          abi::decode_capabilities(dispatcher.reply(), capabilities) && capabilities.flags == 0,
          "unconfigured query reports no capability");
    check(dispatcher.dispatch(abi::ioctl_open_session, open_request().view(), 64).status ==
          r2d::DispatchStatus::not_ready, "unconfigured miniport refuses sessions");
    check(dispatcher.dispatch(0x00232420U, ByteView(nullptr, 0), 64).status ==
          r2d::DispatchStatus::unknown_code, "foreign IOCTL left to the caller");

    r2d::Blit2dBackend incomplete = table(backend);
    incomplete.sync = nullptr;
    check(!dispatcher.configure(incomplete, primary(), staging_size, 1), "incomplete backend rejected");
    r2d::EngineSurface narrow = primary();
    narrow.pitch_bytes = screen_width * 4U - 4U;
    check(!dispatcher.configure(table(backend), narrow, staging_size, 1), "pitch below width rejected");
    r2d::EngineSurface misaligned = primary();
    misaligned.gpu_address += 2;
    check(!dispatcher.configure(table(backend), misaligned, staging_size, 1), "unaligned scanout rejected");
    check(!dispatcher.configure(table(backend), primary(), 4094, 1), "unaligned staging size rejected");
    check(dispatcher.configure(table(backend), primary(), staging_size, 1), "valid configuration");
    check(!dispatcher.configure(table(backend), primary(), staging_size, 1), "double configuration refused");

    check(dispatcher.dispatch(abi::ioctl_query, open_request().view(), 64).status ==
          r2d::DispatchStatus::invalid_parameter, "query takes no input");
    check(dispatcher.dispatch(abi::ioctl_query, ByteView(nullptr, 0), abi::capabilities_bytes - 1U).status ==
          r2d::DispatchStatus::insufficient_buffer, "query output capacity checked");
    result = dispatcher.dispatch(abi::ioctl_query, ByteView(nullptr, 0), abi::capabilities_bytes);
    check(result.status == r2d::DispatchStatus::success &&
          abi::decode_capabilities(dispatcher.reply(), capabilities) &&
          capabilities.primary_width == screen_width && capabilities.primary_pitch == screen_pitch &&
          capabilities.staging_bytes == staging_size && capabilities.max_rects == 64 &&
          (capabilities.flags & abi::capability_eviction) != 0 &&
          capabilities.session_surface_bytes == abi::default_session_surface_bytes &&
          capabilities.device_surface_bytes == abi::default_device_surface_bytes &&
          capabilities.used_surface_bytes == 0 && capabilities.operations == abi::implemented_operations,
          "configured query describes the adopted primary and the offscreen budgets");
    for (uint32_t code : {abi::ioctl_rop, abi::ioctl_text, abi::ioctl_gradient,
                          abi::ioctl_last}) {
        check(dispatcher.dispatch(code, ByteView(nullptr, 0), 64).status == r2d::DispatchStatus::unknown_code,
              "raster or reserved code without a compute backend answers unknown_code");
    }

    r2d::Blit2dBackend no_eviction = table(backend);
    no_eviction.surface_restore = nullptr;
    r2d::Dispatcher second;
    check(!second.configure(no_eviction, primary(), staging_size, 1), "eviction callbacks are required");
    r2d::Limits inverted{};
    inverted.staging_bytes = staging_size;
    inverted.session_surface_bytes = 2;
    inverted.device_surface_bytes = 1;
    check(!second.configure(table(backend), primary(), inverted, 1), "session budget above device budget refused");
}

void session_tests() {
    FakeBackend backend;
    r2d::Dispatcher dispatcher;
    dispatcher.configure(table(backend), primary(), staging_size, 7);
    abi::SessionGrant grants[abi::max_sessions]{};
    bool all = true;
    for (abi::SessionGrant& grant : grants) {
        all = all && open(dispatcher, grant);
    }

    check(all && backend.staging_created == abi::max_sessions, "four sessions with private staging");
    check(grants[0].staging_address == reinterpret_cast<uintptr_t>(backend.staging_memory[0]) &&
          grants[0].staging_bytes == staging_size && grants[0].primary_width == screen_width,
          "grant carries the miniport staging mapping and primary geometry");
    check(grants[0].session.session_key != grants[1].session.session_key &&
          grants[0].session.session != grants[1].session.session, "distinct handles and keys");
    abi::SessionGrant extra{};
    check(!open(dispatcher, extra), "fifth session refused (busy)");
    check(dispatcher.dispatch(abi::ioctl_open_session, open_request().view(), 55).status ==
          r2d::DispatchStatus::insufficient_buffer, "grant capacity checked before allocation");

    abi::Header forged = grants[1].session;
    forged.session_key ^= 1U;
    check(dispatcher.dispatch(abi::ioctl_close_session, close_request(forged).view(), 0).status ==
          r2d::DispatchStatus::invalid_parameter, "wrong session key rejected");
    check(dispatcher.dispatch(abi::ioctl_close_session, close_request(grants[1].session).view(), 0).status ==
          r2d::DispatchStatus::success && backend.staging_destroyed == 1, "close frees idle staging");
    check(dispatcher.dispatch(abi::ioctl_close_session, close_request(grants[1].session).view(), 0).status ==
          r2d::DispatchStatus::invalid_parameter, "closed handle is stale");
    abi::SessionGrant reopened{};
    check(open(dispatcher, reopened) && reopened.session.session != grants[1].session.session &&
          (reopened.session.session & 0xffU) == (grants[1].session.session & 0xffU),
          "reused slot gets a new generation");
    check(outcome(dispatcher, abi::ioctl_fill,
                  fill_request(grants[1].session, grants[1].primary_surface, 0, rects({{0, 0, 1, 1}}))) ==
              0xffffffffU, "stale generation cannot reach the reopened slot");

    // Primary ownership.
    check(outcome(dispatcher, abi::ioctl_assert_primary, assert_request(grants[0].session, true)) ==
          abi::outcome_completed, "first session acquires the primary");
    check(outcome(dispatcher, abi::ioctl_assert_primary, assert_request(grants[2].session, true)) ==
          abi::outcome_declined, "second session cannot steal the primary");
    const auto denied = dispatcher.dispatch(abi::ioctl_fill,
        fill_request(grants[2].session, grants[2].primary_surface, 0, rects({{0, 0, 8, 8}})).view(),
        abi::receipt_bytes);
    check(denied.status == r2d::DispatchStatus::invalid_parameter && backend.fills == 0,
          "non-owner fill on the primary rejected before the backend");
    check(outcome(dispatcher, abi::ioctl_fill,
                  fill_request(grants[0].session, grants[0].primary_surface, 5, rects({{0, 0, 8, 8}}))) ==
              abi::outcome_completed, "owner fills the primary");
    const unsigned syncs = backend.syncs;
    check(outcome(dispatcher, abi::ioctl_assert_primary, assert_request(grants[0].session, false)) ==
              abi::outcome_completed && backend.syncs == syncs + 1, "release waits for queued work");
    check(outcome(dispatcher, abi::ioctl_assert_primary, assert_request(grants[2].session, true)) ==
          abi::outcome_completed, "released primary can be acquired by another session");
    Buffer bad_assert = assert_request(grants[2].session, true);
    bad_assert.bytes[16] = 2;
    check(dispatcher.dispatch(abi::ioctl_assert_primary, bad_assert.view(), 32).status ==
          r2d::DispatchStatus::invalid_parameter, "enable must be 0 or 1");
}

void fill_tests() {
    FakeBackend backend;
    r2d::Dispatcher dispatcher;
    dispatcher.configure(table(backend), primary(), staging_size, 3);
    abi::SessionGrant grant{};
    check(open_primary(dispatcher, grant), "fill session");
    const uint32_t surface = grant.primary_surface;
    const abi::Header session = grant.session;

    struct Case final {
        abi::Rect rect;
        const char* description;
    } invalid[] = {
        {{0, 0, 1921, 10}, "right edge beyond width rejected"},
        {{0, 0, 10, 1081}, "bottom edge beyond height rejected"},
        {{-1, 0, 10, 10}, "negative left rejected"},
        {{5, 5, 5, 10}, "empty width rejected"},
        {{10, 5, 5, 10}, "mirrored rectangle rejected"},
        {{0, INT32_MIN, 10, 10}, "INT32_MIN rejected without overflow"},
        {{0, 0, INT32_MAX, 10}, "INT32_MAX rejected"},
    };
    for (const Case& item : invalid) {
        const auto result = dispatcher.dispatch(abi::ioctl_fill,
            fill_request(session, surface, 1, rects({item.rect})).view(), abi::receipt_bytes);
        check(result.status == r2d::DispatchStatus::invalid_parameter, item.description);
    }

    check(dispatcher.dispatch(abi::ioctl_fill,
              fill_request(session, surface, 1, rects({{0, 0, 10, 10}, {5, 5, 15, 15}})).view(),
              abi::receipt_bytes).status == r2d::DispatchStatus::invalid_parameter,
          "overlapping clip rectangles rejected");
    check(dispatcher.dispatch(abi::ioctl_fill,
              fill_request(session, surface, 1, rects({{0, 0, 10, 10}})).view(), 31).status ==
              r2d::DispatchStatus::insufficient_buffer && backend.fills == 0,
          "receipt capacity checked before execution");
    check(dispatcher.dispatch(abi::ioctl_fill,
              fill_request(session, surface + 1U, 1, rects({{0, 0, 10, 10}})).view(),
              abi::receipt_bytes).status == r2d::DispatchStatus::invalid_parameter,
          "unknown surface handle rejected");
    check(backend.fills == 0, "no invalid fill reached the backend");

    abi::RectList full{};
    for (int32_t index = 0; index < 64; ++index) {
        full.rects[full.count++] = {index * 30, 0, index * 30 + 30, 1080};
    }

    abi::Receipt receipt{};
    auto result = dispatcher.dispatch(abi::ioctl_fill, fill_request(session, surface, 0xff123456U, full).view(),
                                      abi::receipt_bytes);
    check(receipt_of(dispatcher, result, receipt) && receipt.outcome == abi::outcome_completed &&
          receipt.fence == 1 && backend.fills == 1 && backend.rects.size() == 64 &&
          backend.rects[63].x == 1890 && backend.rects[63].width == 30 &&
          backend.rects[63].height == 1080 && backend.last_color == 0xff123456U &&
          backend.last_target_address == primary_address,
          "64 edge rectangles reach the backend exactly, color high byte preserved");

    backend.next_status = r2d::BackendStatus::rejected;
    check(outcome(dispatcher, abi::ioctl_fill, fill_request(session, surface, 0, rects({{0, 0, 2, 2}}))) ==
          abi::outcome_declined, "backend rejection is a decline");
    backend.next_status = r2d::BackendStatus::ok;
    check(outcome(dispatcher, abi::ioctl_fill, fill_request(session, surface, 0, rects({{0, 0, 2, 2}}))) ==
          abi::outcome_completed, "decline does not poison");

    backend.next_status = r2d::BackendStatus::uncertain;
    check(outcome(dispatcher, abi::ioctl_fill, fill_request(session, surface, 0, rects({{0, 0, 2, 2}}))) ==
          abi::outcome_lost, "uncertain submission reports lost");
    backend.next_status = r2d::BackendStatus::ok;
    const unsigned fills = backend.fills;
    check(outcome(dispatcher, abi::ioctl_fill, fill_request(session, surface, 0, rects({{0, 0, 2, 2}}))) ==
              abi::outcome_lost && backend.fills == fills, "poisoned session never reaches the backend");
    check(outcome(dispatcher, abi::ioctl_sync, sync_request(session, 0)) == abi::outcome_lost,
          "poisoned session cannot prove completion");

    backend.sync_status = r2d::BackendStatus::uncertain;
    check(dispatcher.dispatch(abi::ioctl_close_session, close_request(session).view(), 0).status ==
              r2d::DispatchStatus::success && backend.staging_destroyed == 0,
          "unproven retirement quarantines staging instead of freeing it");
    backend.sync_status = r2d::BackendStatus::ok;
    abi::SessionGrant next{};
    bool reopened = true;
    for (uint32_t index = 0; index < abi::max_sessions - 1U; ++index) {
        reopened = reopened && open(dispatcher, next);
    }

    abi::SessionGrant refused{};
    check(reopened && !open(dispatcher, refused), "quarantined session slot is never reused");
}

void copy_tests() {
    FakeBackend backend;
    r2d::Dispatcher dispatcher;
    dispatcher.configure(table(backend), primary(), staging_size, 9);
    abi::SessionGrant grant{};
    check(open_primary(dispatcher, grant), "copy session");
    const uint32_t surface = grant.primary_surface;

    // Scroll down by 10: source = destination - 10. B's source overlaps A's
    // destination, so B must be copied before A regardless of input order.
    const abi::RectList scroll = rects({{0, 10, 100, 60}, {0, 60, 100, 110}});
    check(outcome(dispatcher, abi::ioctl_copy, copy_request(grant.session, surface, surface, 0, -10, scroll)) ==
              abi::outcome_completed && backend.copy_rects.size() == 2 &&
              backend.copy_rects[0].destination.y == 60 && backend.copy_rects[1].destination.y == 10 &&
              backend.copy_rects[0].source_y == 50 && backend.copy_rects[1].source_y == 0,
          "overlapping scroll is ordered so no source is overwritten first");
    const abi::RectList up = rects({{0, 50, 100, 100}, {0, 0, 100, 50}});
    check(outcome(dispatcher, abi::ioctl_copy, copy_request(grant.session, surface, surface, 0, 10, up)) ==
              abi::outcome_completed && backend.copy_rects[0].destination.y == 0 &&
              backend.copy_rects[1].destination.y == 50, "upward scroll ordered top first");
    const abi::RectList sideways = rects({{10, 0, 60, 10}, {60, 0, 110, 10}, {110, 0, 160, 10}});
    check(outcome(dispatcher, abi::ioctl_copy, copy_request(grant.session, surface, surface, -5, 0, sideways)) ==
              abi::outcome_completed && backend.copy_rects[0].destination.x == 110 &&
              backend.copy_rects[1].destination.x == 60 && backend.copy_rects[2].destination.x == 10,
          "rightward move ordered right to left");
    check(outcome(dispatcher, abi::ioctl_copy, copy_request(grant.session, surface, surface, 0, 0, sideways)) ==
              abi::outcome_completed && backend.copy_rects[0].destination.x == 10,
          "self copy needs no reordering");
    check(dispatcher.dispatch(abi::ioctl_copy,
              copy_request(grant.session, surface, surface, 0, -1, rects({{0, 0, 10, 10}})).view(),
              abi::receipt_bytes).status == r2d::DispatchStatus::invalid_parameter,
          "source above the surface rejected");
    check(dispatcher.dispatch(abi::ioctl_copy,
              copy_request(grant.session, surface, surface, 1, 0, rects({{1910, 0, 1920, 10}})).view(),
              abi::receipt_bytes).status == r2d::DispatchStatus::invalid_parameter,
          "source beyond the right edge rejected");
    check(dispatcher.dispatch(abi::ioctl_copy,
              copy_request(grant.session, surface, surface, INT32_MAX, 0, rects({{0, 0, 10, 10}})).view(),
              abi::receipt_bytes).status == r2d::DispatchStatus::invalid_parameter,
          "huge delta rejected without overflow");

    // Offscreen surfaces: ownership is per session.
    auto result = dispatcher.dispatch(abi::ioctl_surface_create,
        surface_create_request(grant.session, 64, 32).view(), abi::surface_grant_bytes);
    abi::SurfaceGrant offscreen{};
    check(result.status == r2d::DispatchStatus::success &&
          abi::decode_surface_grant(dispatcher.reply(), offscreen) && offscreen.width == 64 &&
          offscreen.pitch == 256, "offscreen surface created");
    check(outcome(dispatcher, abi::ioctl_copy,
                  copy_request(grant.session, offscreen.surface, surface, 0, 0, rects({{0, 0, 64, 32}}))) ==
              abi::outcome_completed && backend.last_source_address == primary_address &&
              backend.last_target_address != primary_address, "screen to offscreen copy");
    check(dispatcher.dispatch(abi::ioctl_copy,
              copy_request(grant.session, offscreen.surface, surface, 0, 0, rects({{0, 0, 65, 32}})).view(),
              abi::receipt_bytes).status == r2d::DispatchStatus::invalid_parameter,
          "rectangle checked against the offscreen size");
    abi::SessionGrant other{};
    check(open(dispatcher, other), "second session");
    check(dispatcher.dispatch(abi::ioctl_fill,
              fill_request(other.session, offscreen.surface, 0, rects({{0, 0, 4, 4}})).view(),
              abi::receipt_bytes).status == r2d::DispatchStatus::invalid_parameter,
          "another session cannot touch an offscreen surface");
    check(dispatcher.dispatch(abi::ioctl_surface_destroy,
              surface_destroy_request(other.session, offscreen.surface).view(), abi::receipt_bytes).status ==
              r2d::DispatchStatus::invalid_parameter, "another session cannot destroy it");
    check(dispatcher.dispatch(abi::ioctl_surface_destroy,
              surface_destroy_request(grant.session, surface).view(), abi::receipt_bytes).status ==
              r2d::DispatchStatus::invalid_parameter, "the primary cannot be destroyed");

    const unsigned syncs = backend.syncs;
    check(outcome(dispatcher, abi::ioctl_surface_destroy, surface_destroy_request(grant.session, offscreen.surface)) ==
              abi::outcome_completed && backend.syncs == syncs + 1 && backend.surfaces_destroyed == 1,
          "destroy waits for the surface's last fence before freeing");
    check(dispatcher.dispatch(abi::ioctl_fill,
              fill_request(grant.session, offscreen.surface, 0, rects({{0, 0, 4, 4}})).view(),
              abi::receipt_bytes).status == r2d::DispatchStatus::invalid_parameter,
          "destroyed surface handle is stale");

    unsigned created = 0;
    for (uint32_t index = 0; index < abi::max_session_surfaces + 2U; ++index) {
        created += dispatcher.dispatch(abi::ioctl_surface_create, surface_create_request(other.session, 8, 8).view(),
                                       abi::surface_grant_bytes).status == r2d::DispatchStatus::success ? 1U : 0U;
    }

    check(created == abi::max_session_surfaces, "per-session surface quota enforced");
    check(dispatcher.dispatch(abi::ioctl_surface_create,
              surface_create_request(grant.session, abi::max_surface_dimension + 1U, 8).view(),
              abi::surface_grant_bytes).status == r2d::DispatchStatus::invalid_parameter,
          "oversized surface rejected");
    const unsigned destroyed = backend.surfaces_destroyed;
    dispatcher.dispatch(abi::ioctl_close_session, close_request(other.session).view(), 0);
    check(backend.surfaces_destroyed == destroyed + abi::max_session_surfaces,
          "closing a session frees all of its surfaces");
}

void transfer_tests() {
    FakeBackend backend;
    r2d::Dispatcher dispatcher;
    dispatcher.configure(table(backend), primary(), staging_size, 11);
    abi::SessionGrant grant{};
    check(open_primary(dispatcher, grant), "transfer session");
    const uint32_t surface = grant.primary_surface;
    const abi::Header session = grant.session;

    // Two rectangles inside a 16x8 staging image at offset 1024, pitch 64.
    const abi::RectList list = rects({{100, 200, 104, 202}, {108, 204, 116, 208}});
    check(outcome(dispatcher, abi::ioctl_upload, transfer_request(session, surface, 1024, 64, 100, 200, list)) ==
              abi::outcome_completed && backend.transfer_rects.size() == 2 &&
              backend.transfer_rects[0].staging_offset == 1024 &&
              backend.transfer_rects[1].staging_offset == 1024 + 4 * 64 + 8 * 4 &&
              backend.transfer_rects[1].surface.width == 8 && backend.last_pitch == 64,
          "upload staging offsets computed from origin and pitch");

    struct Case final {
        uint32_t offset;
        uint32_t pitch;
        int32_t origin_x;
        int32_t origin_y;
        abi::Rect rect;
        const char* description;
    } invalid[] = {
        {0, 62, 0, 0, {0, 0, 4, 4}, "pitch must be a multiple of four"},
        {0, 0, 0, 0, {0, 0, 4, 4}, "zero pitch rejected"},
        {2, 64, 0, 0, {0, 0, 4, 4}, "unaligned staging offset rejected"},
        {0, abi::max_staging_pitch + 4U, 0, 0, {0, 0, 4, 4}, "pitch above limit rejected"},
        {0, 64, 1, 0, {0, 0, 4, 4}, "rectangle left of the origin rejected"},
        {0, 64, 0, 1, {0, 0, 4, 4}, "rectangle above the origin rejected"},
        {0, 16, 0, 0, {0, 0, 5, 4}, "row wider than pitch rejected"},
        {0, 64, 0, 0, {0, 0, 16, 65}, "rows beyond staging rejected"},
        {staging_size, 64, 0, 0, {0, 0, 1, 1}, "offset at staging end rejected"},
        {staging_size - 4U, 64, 0, 0, {0, 0, 2, 1}, "last row crossing the end rejected"},
        {0xfffffffcU, 64, 0, 0, {0, 0, 1, 1}, "offset near 2^32 rejected without wrap"},
        {0, 64, INT32_MIN, 0, {0, 0, 1, 1}, "INT32_MIN origin rejected without overflow"},
    };
    for (const Case& item : invalid) {
        const auto result = dispatcher.dispatch(abi::ioctl_upload,
            transfer_request(session, surface, item.offset, item.pitch, item.origin_x, item.origin_y,
                             rects({item.rect})).view(), abi::receipt_bytes);
        check(result.status == r2d::DispatchStatus::invalid_parameter, item.description);
    }

    check(outcome(dispatcher, abi::ioctl_upload,
                  transfer_request(session, surface, staging_size - 64U, 64, 0, 0, rects({{0, 0, 16, 1}}))) ==
              abi::outcome_completed, "exactly the last staging row is accepted");
    check(backend.uploads == 2, "no invalid upload reached the backend");

    const unsigned syncs = backend.syncs;
    check(outcome(dispatcher, abi::ioctl_readback, transfer_request(session, surface, 0, 64, 0, 0, rects({{0, 0, 16, 4}}))) ==
              abi::outcome_completed && backend.readbacks == 1 && backend.syncs == syncs + 1 &&
              backend.last_source_address == primary_address,
          "readback completes only after a backend sync");
    backend.sync_status = r2d::BackendStatus::uncertain;
    check(outcome(dispatcher, abi::ioctl_readback, transfer_request(session, surface, 0, 64, 0, 0, rects({{0, 0, 16, 4}}))) ==
              abi::outcome_lost, "readback without proven completion is lost");
    backend.sync_status = r2d::BackendStatus::ok;

    // Sync semantics on a fresh session.
    dispatcher.dispatch(abi::ioctl_close_session, close_request(session).view(), 0);
    abi::SessionGrant fresh{};
    check(open_primary(dispatcher, fresh), "fresh session");
    check(dispatcher.dispatch(abi::ioctl_sync, sync_request(fresh.session, 1).view(), abi::receipt_bytes).status ==
              r2d::DispatchStatus::invalid_parameter, "sync beyond submitted work rejected");
    check(outcome(dispatcher, abi::ioctl_sync, sync_request(fresh.session, 0)) == abi::outcome_completed,
          "sync with nothing pending completes without waiting");
    outcome(dispatcher, abi::ioctl_fill, fill_request(fresh.session, fresh.primary_surface, 0, rects({{0, 0, 1, 1}})));
    abi::Receipt receipt{};
    const auto result = dispatcher.dispatch(abi::ioctl_sync, sync_request(fresh.session, 0).view(), abi::receipt_bytes);
    check(receipt_of(dispatcher, result, receipt) && receipt.outcome == abi::outcome_completed &&
          receipt.completed_fence >= receipt.fence && receipt.fence == backend.fence,
          "sync of everything reports the retired session fence");

    check(dispatcher.shutdown() && !dispatcher.configured(), "idle shutdown retires everything");
    check(dispatcher.dispatch(abi::ioctl_fill,
              fill_request(fresh.session, fresh.primary_surface, 0, rects({{0, 0, 1, 1}})).view(),
              abi::receipt_bytes).status == r2d::DispatchStatus::not_ready, "no requests after shutdown");
    check(dispatcher.configure(table(backend), primary(), staging_size, 12), "re-adoption after shutdown");
    check(dispatcher.dispatch(abi::ioctl_fill,
              fill_request(fresh.session, fresh.primary_surface, 0, rects({{0, 0, 1, 1}})).view(),
              abi::receipt_bytes).status == r2d::DispatchStatus::invalid_parameter,
          "old session and primary handles stay invalid after re-adoption");
}

void display_policy_tests() {
    using display::BlitPlan;
    using display::SurfaceKind;
    auto plan = [](uint32_t rop, bool masked, uint32_t brush, SurfaceKind target, SurfaceKind source) {
        return display::classify_blit(rop, masked, brush, target, source);
    };
    check(plan(0x0000, false, display::no_solid_color, SurfaceKind::primary, SurfaceKind::none).plan ==
              BlitPlan::fill && plan(0x0000, false, 0, SurfaceKind::primary, SurfaceKind::none).color == 0,
          "BLACKNESS fills zero");
    check(plan(0xffff, false, display::no_solid_color, SurfaceKind::primary, SurfaceKind::none).color ==
              UINT32_MAX, "WHITENESS fills the full 32-bit constant");
    check(plan(0xf0f0, false, 0x00123456U, SurfaceKind::primary, SurfaceKind::none).plan == BlitPlan::fill &&
              plan(0xf0f0, false, 0x00123456U, SurfaceKind::primary, SurfaceKind::none).color == 0x00123456U,
          "solid PATCOPY fills iSolidColor");
    check(plan(0xf0f0, false, display::no_solid_color, SurfaceKind::primary, SurfaceKind::none).plan ==
              BlitPlan::fallback, "pattern brush falls back");
    check(plan(0xcccc, false, 0, SurfaceKind::primary, SurfaceKind::primary).plan == BlitPlan::device_copy,
          "screen SRCCOPY is a GPU copy");
    check(plan(0xcccc, false, 0, SurfaceKind::offscreen, SurfaceKind::primary).plan == BlitPlan::device_copy &&
              plan(0xcccc, false, 0, SurfaceKind::primary, SurfaceKind::offscreen).plan == BlitPlan::device_copy &&
              plan(0xcccc, false, 0, SurfaceKind::offscreen, SurfaceKind::offscreen).plan == BlitPlan::device_copy,
          "screen<->offscreen and offscreen<->offscreen SRCCOPY are GPU copies");
    check(plan(0xf0f0, false, 7, SurfaceKind::offscreen, SurfaceKind::none).plan == BlitPlan::fill &&
              plan(0xcccc, false, 0, SurfaceKind::offscreen, SurfaceKind::bitmap32).plan == BlitPlan::upload &&
              plan(0xcccc, false, 0, SurfaceKind::bitmap32, SurfaceKind::offscreen).plan == BlitPlan::readback,
          "offscreen fill, upload and readback");
    check(plan(0x6666, false, 0, SurfaceKind::offscreen, SurfaceKind::offscreen).plan == BlitPlan::fallback &&
              plan(0xcccc, false, 0, SurfaceKind::bitmap32, SurfaceKind::bitmap32).plan == BlitPlan::fallback,
          "other ROPs and DIB-to-DIB stay engine work");
    check(plan(0xcccc, false, 0, SurfaceKind::primary, SurfaceKind::bitmap32).plan == BlitPlan::upload,
          "32bpp bitmap SRCCOPY uploads");
    check(plan(0xcccc, false, 0, SurfaceKind::bitmap32, SurfaceKind::primary).plan == BlitPlan::readback,
          "screen to bitmap SRCCOPY reads back");
    check(plan(0xcccc, false, 0, SurfaceKind::primary, SurfaceKind::other).plan == BlitPlan::fallback,
          "translated or foreign source falls back");
    check(plan(0xcccc, true, 0, SurfaceKind::primary, SurfaceKind::primary).plan == BlitPlan::fallback,
          "masked blit falls back");
    check(plan(0xccaa, false, 0, SurfaceKind::primary, SurfaceKind::primary).plan == BlitPlan::fallback,
          "distinct foreground/background ROP falls back");
    check(plan(0x6666, false, 0, SurfaceKind::primary, SurfaceKind::primary).plan == BlitPlan::fallback,
          "SRCINVERT falls back");
    check(plan(0x1cccc, false, 0, SurfaceKind::primary, SurfaceKind::primary).plan == BlitPlan::fallback,
          "out-of-range ROP4 falls back");
    check(plan(0x0000, false, 0, SurfaceKind::bitmap32, SurfaceKind::none).plan == BlitPlan::fallback,
          "bitmap targets are never filled by the GPU");

    display::Rectangle rect{};
    const int32_t square[] = {16, 32, 160, 32, 160, 320, 16, 320};
    check(display::rectangle_from_path(square, 4, rect) && rect.left == 1 && rect.top == 2 &&
              rect.right == 10 && rect.bottom == 20, "integer rectangle path accepted half-open");
    const int32_t closed[] = {160, 320, 16, 320, 16, 32, 160, 32, 160, 320};
    check(display::rectangle_from_path(closed, 5, rect) && rect.left == 1 && rect.bottom == 20,
          "explicitly closed reversed rectangle accepted");
    const int32_t fractional[] = {17, 32, 160, 32, 160, 320, 17, 320};
    check(!display::rectangle_from_path(fractional, 4, rect), "fractional FIX vertex falls back");
    const int32_t diagonal[] = {16, 32, 160, 32, 16, 320, 160, 320};
    check(!display::rectangle_from_path(diagonal, 4, rect), "bow-tie path falls back");
    const int32_t degenerate[] = {16, 32, 16, 32, 16, 320, 16, 320};
    check(!display::rectangle_from_path(degenerate, 4, rect), "zero-width path falls back");
    const int32_t open[] = {16, 32, 160, 32, 160, 320, 16, 320, 16, 48};
    check(!display::rectangle_from_path(open, 5, rect), "fifth point must close the figure");
    check(!display::rectangle_from_path(square, 3, rect), "triangle falls back");

    check(display::staging_half_bytes(4096) == 2048 && display::staging_half_bytes(4100) == 2048,
          "halves are dword-aligned");
    check(display::fits_half(16, 32, 2048) && !display::fits_half(16, 33, 2048), "half capacity exact");
    check(display::band_rows(1920, 8U << 20) == 1092 && display::band_rows(1920, 7679) == 0 &&
              display::band_rows(0, 4096) == 0, "band rows");

    display::Statistics statistics{};
    statistics.accelerated = 1;
    statistics.gpu_fills = 3;
    statistics.lost = 0x0102030405060708ULL;
    uint8_t bytes[display::statistics_bytes]{};
    uint32_t size = 0;
    uint32_t low = 0;
    uint32_t high = 0;
    const ByteView view(bytes, sizeof(bytes));
    statistics.live_bitmaps = 9;
    statistics.live_bitmap_bytes = 0x1122334455667788ULL;
    uint32_t version = 0;
    uint32_t live = 0;
    check(display::encode_statistics(CommandBuffer(bytes, sizeof(bytes)), statistics) &&
              view.read_u32(0, size) && size == display::statistics_bytes && size == 328 &&
              view.read_u32(4, version) && version == 3 && view.read_u32(16, live) && live == 9 &&
              view.read_u32(24, low) && low == 3 && view.read_u32(104, low) && view.read_u32(108, high) &&
              low == 0x05060708U && high == 0x01020304U && view.read_u32(216, low) &&
              view.read_u32(220, high) && low == 0x55667788U && high == 0x11223344U,
          "statistics escape layout v3");
    statistics.brushes_refused = 0x0a0b0c0d01020304ULL;
    check(display::encode_statistics(CommandBuffer(bytes, sizeof(bytes)), statistics) &&
              view.read_u32(320, low) && view.read_u32(324, high) && low == 0x01020304U && high == 0x0a0b0c0dU,
          "the last v3 counter (brushes_refused) sits at +320");
    uint32_t count = 0;
    display::count(statistics.gpu_fills);
    statistics.syncs = UINT64_MAX;
    display::count(statistics.syncs);
    check(statistics.gpu_fills == 4 && statistics.syncs == UINT64_MAX && count == 0, "saturating counters");
}


// ---------------------------------------------------------------------------
// Offscreen accounting and the suspend/resume eviction contract.
// ---------------------------------------------------------------------------
uint32_t create_surface(r2d::Dispatcher& dispatcher, abi::Header session, uint32_t width, uint32_t height,
                        r2d::DispatchStatus* status = nullptr) {
    const auto result = dispatcher.dispatch(abi::ioctl_surface_create,
        surface_create_request(session, width, height).view(), abi::surface_grant_bytes);
    abi::SurfaceGrant grant{};
    if (status != nullptr) {
        *status = result.status;
    }

    return result.status == r2d::DispatchStatus::success &&
           abi::decode_surface_grant(dispatcher.reply(), grant) ? grant.surface : 0;
}

abi::Capabilities query(r2d::Dispatcher& dispatcher) {
    abi::Capabilities value{};
    dispatcher.dispatch(abi::ioctl_query, ByteView(nullptr, 0), abi::capabilities_bytes);
    abi::decode_capabilities(dispatcher.reply(), value);
    return value;
}

void budget_tests() {
    FakeBackend backend;
    r2d::Dispatcher dispatcher;
    // 64 x 64 surfaces: pitch 256, 16 KiB each. Session budget 3, device budget 5.
    constexpr uint64_t surface = 256U * 64U;
    r2d::Limits limits{};
    limits.staging_bytes = staging_size;
    limits.session_surface_bytes = 3 * surface;
    limits.device_surface_bytes = 5 * surface;
    check(dispatcher.configure(table(backend), primary(), limits, 3), "configured with explicit budgets");
    abi::SessionGrant first{};
    abi::SessionGrant second{};
    check(open(dispatcher, first) && open(dispatcher, second), "two budget sessions");
    uint32_t handles[3]{};
    for (uint32_t& handle : handles) {
        handle = create_surface(dispatcher, first.session, 64, 64);
    }

    r2d::DispatchStatus status{};
    check(handles[2] != 0 && create_surface(dispatcher, first.session, 64, 64, &status) == 0 &&
          status == r2d::DispatchStatus::busy, "session byte budget refuses the fourth surface (busy)");
    check(query(dispatcher).used_surface_bytes == 3 * surface && query(dispatcher).session_surface_bytes ==
              3 * surface && query(dispatcher).device_surface_bytes == 5 * surface,
          "capabilities report budgets and usage");
    check(create_surface(dispatcher, second.session, 64, 64) != 0 &&
          create_surface(dispatcher, second.session, 64, 64) != 0 &&
          create_surface(dispatcher, second.session, 64, 64, &status) == 0 && status == r2d::DispatchStatus::busy,
          "device byte budget bounds all sessions together");
    const unsigned created = backend.surfaces_created;
    check(create_surface(dispatcher, first.session, 4096, 4096, &status) == 0 &&
          status == r2d::DispatchStatus::busy && backend.surfaces_created == created,
          "an over-budget request never reaches the backend");
    check(outcome(dispatcher, abi::ioctl_surface_destroy, surface_destroy_request(first.session, handles[0])) ==
              abi::outcome_completed && create_surface(dispatcher, first.session, 64, 64) != 0,
          "destroy returns bytes to the budget");
    dispatcher.dispatch(abi::ioctl_close_session, close_request(second.session).view(), 0);
    check(query(dispatcher).used_surface_bytes == 3 * surface, "closing a session returns its bytes");

    // A backend pitch wider than predicted is re-checked after creation.
    FakeBackend wide;
    r2d::Dispatcher tight;
    limits.session_surface_bytes = 60U * 4U * 64U; // Exactly the packed size, not the 256-byte pitch.
    limits.device_surface_bytes = limits.session_surface_bytes;
    check(tight.configure(table(wide), primary(), limits, 4) && open(tight, first), "tight budget session");
    check(create_surface(tight, first.session, 60, 64, &status) == 0 && status == r2d::DispatchStatus::busy &&
          wide.surfaces_destroyed == 1, "exact backend pitch over budget is destroyed and refused");
}

void suspend_tests() {
    FakeBackend backend;
    r2d::Dispatcher dispatcher;
    check(dispatcher.configure(table(backend), primary(), staging_size, 5), "suspend configuration");
    check(!dispatcher.resume(table(backend), primary()), "resume without suspend refused");
    abi::SessionGrant grant{};
    check(open_primary(dispatcher, grant), "suspend session");
    const uint32_t a = create_surface(dispatcher, grant.session, 64, 32);
    const uint32_t b = create_surface(dispatcher, grant.session, 64, 32);
    const uint32_t c = create_surface(dispatcher, grant.session, 64, 32);
    check(a != 0 && b != 0 && c != 0, "three offscreen surfaces");
    check(outcome(dispatcher, abi::ioctl_fill, fill_request(grant.session, a, 1, rects({{0, 0, 64, 32}}))) ==
              abi::outcome_completed, "work before suspend");
    const uint64_t used = query(dispatcher).used_surface_bytes;

    backend.evict_allowance = 2; // c finds no system memory.
    const unsigned syncs = backend.syncs;
    check(dispatcher.suspend() && dispatcher.suspended() && backend.evictions == 2 &&
          backend.syncs == syncs + 1 && backend.surfaces_destroyed == 1,
          "suspend proves idleness, evicts two surfaces and frees the one it cannot save");
    check(dispatcher.counters().evicted == 2 && dispatcher.counters().lost == 1 &&
          query(dispatcher).used_surface_bytes == used - 256U * 32U, "eviction accounting");
    check(dispatcher.dispatch(abi::ioctl_fill, fill_request(grant.session, a, 1, rects({{0, 0, 1, 1}})).view(),
                              abi::receipt_bytes).status == r2d::DispatchStatus::not_ready &&
          query(dispatcher).flags == 0, "no request is dispatched while suspended");
    check(!dispatcher.suspend(), "double suspend refused");

    backend.evict_allowance = UINT32_MAX;
    check(dispatcher.resume(table(backend), primary()) && !dispatcher.suspended() && backend.restores == 2 &&
          dispatcher.counters().restored == 2, "resume restores the evicted surfaces");
    const unsigned fills = backend.fills;
    check(outcome(dispatcher, abi::ioctl_fill, fill_request(grant.session, a, 2, rects({{0, 0, 4, 4}}))) ==
              abi::outcome_completed && backend.last_target_address < 0x300000000ULL,
          "a restored surface is drawn at its VRAM placement with the same handle");
    check(outcome(dispatcher, abi::ioctl_fill, fill_request(grant.session, grant.primary_surface, 3,
                                                            rects({{0, 0, 4, 4}}))) == abi::outcome_completed,
          "same-geometry primary keeps its handle and owner");
    {
        abi::Receipt lost{};
        check(receipt_of(dispatcher, dispatcher.dispatch(abi::ioctl_fill, fill_request(grant.session, c, 2,
                  rects({{0, 0, 4, 4}})).view(), abi::receipt_bytes), lost) &&
              lost.outcome == abi::outcome_surface_lost && lost.lost_surface == c && backend.fills == fills + 2,
              "a lost surface answers surface_lost naming it, without reaching the backend");
    }
    // Version 6: exactly once. The handle died with that answer.
    check(dispatcher.dispatch(abi::ioctl_copy, copy_request(grant.session, a, c, 0, 0, rects({{0, 0, 4, 4}})).view(),
                              abi::receipt_bytes).status == r2d::DispatchStatus::invalid_parameter &&
          dispatcher.dispatch(abi::ioctl_readback, transfer_request(grant.session, c, 0, 64, 0, 0,
                                                                    rects({{0, 0, 4, 4}})).view(),
                              abi::receipt_bytes).status == r2d::DispatchStatus::invalid_parameter,
          "copy and readback of an already reported lost surface are invalid");
    check(outcome(dispatcher, abi::ioctl_sync, sync_request(grant.session, 0)) == abi::outcome_completed,
          "a lost surface does not poison the session");
    const unsigned destroyed = backend.surfaces_destroyed;
    check(dispatcher.dispatch(abi::ioctl_surface_destroy, surface_destroy_request(grant.session, c).view(),
                              abi::receipt_bytes).status == r2d::DispatchStatus::invalid_parameter &&
              backend.surfaces_destroyed == destroyed && dispatcher.counters().lost_reported == 1,
          "destroying a reported lost surface is invalid and frees nothing");

    // Restore refused: the surface stays usable in system memory, and a later
    // resume tries again.
    check(dispatcher.suspend(), "second suspend");
    backend.restore_status = r2d::BackendStatus::rejected;
    check(dispatcher.resume(table(backend), primary()), "resume with no VRAM for restore");
    check(outcome(dispatcher, abi::ioctl_fill, fill_request(grant.session, b, 4, rects({{0, 0, 4, 4}}))) ==
              abi::outcome_completed && backend.last_target_address >= 0x300000000ULL,
          "an unrestored surface keeps working in system memory");
    backend.restore_status = r2d::BackendStatus::ok;
    check(dispatcher.suspend() && dispatcher.resume(table(backend), primary()) &&
          outcome(dispatcher, abi::ioctl_fill, fill_request(grant.session, b, 4, rects({{0, 0, 4, 4}}))) ==
              abi::outcome_completed && backend.last_target_address < 0x300000000ULL,
          "the next resume restores it");

    // Different scanout geometry after resume: the old primary handle dies.
    check(dispatcher.suspend(), "third suspend");
    r2d::EngineSurface smaller = primary();
    smaller.width = 1280;
    smaller.height = 720;
    check(dispatcher.resume(table(backend), smaller), "resume onto a different scanout");
    check(dispatcher.dispatch(abi::ioctl_fill,
              fill_request(grant.session, grant.primary_surface, 0, rects({{0, 0, 1, 1}})).view(),
              abi::receipt_bytes).status == r2d::DispatchStatus::invalid_parameter &&
          query(dispatcher).primary_width == 1280, "old primary handle refused after a geometry change");

    // Unprovable idleness: suspend fails, poisons, and shutdown takes over.
    check(outcome(dispatcher, abi::ioctl_fill, fill_request(grant.session, a, 5, rects({{0, 0, 4, 4}}))) ==
              abi::outcome_completed, "work in flight");
    backend.sync_status = r2d::BackendStatus::uncertain;
    check(!dispatcher.suspend() && dispatcher.suspended(), "suspend reports unprovable idleness");
    check(!dispatcher.shutdown() && !dispatcher.configured(), "shutdown quarantines what suspend could not save");

    FakeBackend failing;
    r2d::Dispatcher uncertain;
    check(uncertain.configure(table(failing), primary(), staging_size, 6) && open(uncertain, grant) &&
          create_surface(uncertain, grant.session, 64, 64) != 0, "uncertain eviction setup");
    failing.evict_status = r2d::BackendStatus::uncertain;
    check(!uncertain.suspend() && failing.surfaces_destroyed == 0,
          "an uncertain eviction keeps the surface (never freed) and fails the suspend");
}

// ---------------------------------------------------------------------------
// Clip lists above 64 rectangles: banded enumeration, batching through the real
// dispatcher and execution of its emitted order with per-rectangle memmove.
// ---------------------------------------------------------------------------
constexpr int32_t grid_width = 160;
constexpr int32_t grid_height = 120;

struct Grid final {
    std::vector<uint32_t> pixels = std::vector<uint32_t>(static_cast<size_t>(grid_width) * grid_height);
    uint32_t& at(int32_t x, int32_t y) { return pixels.at(static_cast<size_t>(y) * grid_width + x); }
};

// One CopyRect with memmove semantics (the backend contract for one rectangle).
void execute(Grid& grid, const r2d::CopyRect& rect) {
    std::vector<uint32_t> temporary;
    for (uint32_t y = 0; y < rect.destination.height; ++y) {
        for (uint32_t x = 0; x < rect.destination.width; ++x) {
            temporary.push_back(grid.at(static_cast<int32_t>(rect.source_x + x),
                                        static_cast<int32_t>(rect.source_y + y)));
        }
    }

    size_t index = 0;
    for (uint32_t y = 0; y < rect.destination.height; ++y) {
        for (uint32_t x = 0; x < rect.destination.width; ++x) {
            grid.at(static_cast<int32_t>(rect.destination.x + x), static_cast<int32_t>(rect.destination.y + y)) =
                temporary[index++];
        }
    }
}

// The ReactOS clip.c comparators (CompareRightDown and friends).
bool enumerated_before(display::CopyDirection direction, const display::Rectangle& a,
                       const display::Rectangle& b) {
    const bool down = display::moves_down(direction);
    const bool right = display::moves_right(direction);
    if (down ? a.top != b.top : a.bottom != b.bottom) {
        return down ? a.top < b.top : a.bottom > b.bottom;
    }

    return right ? a.left < b.left : a.right > b.right;
}

std::vector<display::Rectangle> banded_region(std::mt19937& random, int32_t margin) {
    std::vector<display::Rectangle> region;
    int32_t top = margin;
    while (top < grid_height - margin - 2) {
        const int32_t height = 1 + static_cast<int32_t>(random() % 6);
        const int32_t bottom = std::min(top + height, grid_height - margin);
        int32_t left = margin + static_cast<int32_t>(random() % 3);
        while (left < grid_width - margin - 1) {
            const int32_t width = 1 + static_cast<int32_t>(random() % 5);
            const int32_t right = std::min(left + width, grid_width - margin);
            region.push_back({left, top, right, bottom});
            left = right + 1 + static_cast<int32_t>(random() % 3);
        }

        top = bottom + static_cast<int32_t>(random() % 2);
    }

    return region;
}

void batching_tests() {
    // Band order checker.
    display::BandOrder down_right(display::CopyDirection::right_down);
    check(down_right.accept({0, 0, 2, 2}) && down_right.accept({3, 0, 5, 2}) && down_right.accept({0, 2, 4, 3}) &&
          !down_right.failed(), "banded top-down left-to-right order accepted");
    display::BandOrder overlap(display::CopyDirection::right_down);
    check(overlap.accept({0, 0, 5, 10}) && !overlap.accept({10, 5, 20, 15}) && overlap.failed(),
          "vertically overlapping bands (unbanded clip) refused");
    display::BandOrder backwards(display::CopyDirection::left_up);
    check(backwards.accept({10, 10, 12, 12}) && backwards.accept({4, 10, 6, 12}) &&
          !backwards.accept({7, 10, 9, 12}), "same-band rectangle against the direction refused");
    display::BandOrder ragged(display::CopyDirection::right_down);
    check(ragged.accept({0, 0, 2, 2}) && !ragged.accept({3, 0, 5, 3}), "same top with another bottom refused");
    check(display::copy_direction(1, 1) == display::CopyDirection::right_down &&
          display::copy_direction(-1, 1) == display::CopyDirection::left_down &&
          display::copy_direction(1, -1) == display::CopyDirection::right_up &&
          display::copy_direction(-1, -1) == display::CopyDirection::left_up &&
          display::copy_direction(0, 0) == display::CopyDirection::right_down, "copy direction from the offset");

    abi::RectList list{};
    check(display::append_clipped({0, 0, 10, 10}, {5, 5, 20, 20}, list) && list.count == 1 &&
          list.rects[0].right == 10 && !display::append_clipped({0, 0, 10, 10}, {10, 0, 12, 4}, list) &&
          list.count == 1, "clip intersection and empty skip");
    check(display::list_bounds(rects({{4, 5, 6, 7}, {1, 9, 2, 12}})).left == 1 &&
          display::list_bounds(rects({{4, 5, 6, 7}, {1, 9, 2, 12}})).bottom == 12, "batch bounds");
    check(display::translated_inside(rects({{0, 0, 4, 4}}), 2, 2, 6, 6) &&
          !display::translated_inside(rects({{0, 0, 4, 4}}), 3, 2, 6, 6) &&
          !display::translated_inside(rects({{0, 0, 4, 4}}), -1, 0, 6, 6), "translated bounds");

    // Randomized: banded regions far above 64 rectangles, every direction.
    FakeBackend backend;
    backend.auto_complete = true;
    r2d::Dispatcher dispatcher;
    const r2d::EngineSurface surface{primary_address, static_cast<uint32_t>(grid_width),
                                     static_cast<uint32_t>(grid_height), grid_width * 4U, abi::format_xrgb8888};
    abi::SessionGrant grant{};
    check(dispatcher.configure(table(backend), surface, staging_size, 7) && open_primary(dispatcher, grant),
          "batching session");
    std::mt19937 random(0x2d5eedU);
    unsigned trials = 0;
    unsigned exact = 0;
    unsigned largest = 0;
    unsigned requests = 0;
    unsigned naive_wrong = 0;
    unsigned naive_refused = 0;
    for (unsigned trial = 0; trial < 240; ++trial) {
        constexpr int32_t margin = 12;
        const int32_t dx = static_cast<int32_t>(random() % 25) - 12;
        const int32_t dy = static_cast<int32_t>(random() % 25) - 12;
        std::vector<display::Rectangle> region = banded_region(random, margin);
        const display::CopyDirection direction = display::copy_direction(dx, dy);
        std::sort(region.begin(), region.end(), [&](const display::Rectangle& a, const display::Rectangle& b) {
            return enumerated_before(direction, a, b);
        });

        Grid grid;
        for (int32_t y = 0; y < grid_height; ++y) {
            for (int32_t x = 0; x < grid_width; ++x) {
                grid.at(x, y) = static_cast<uint32_t>(y * 1000 + x);
            }
        }

        Grid reference = grid;
        const Grid snapshot = grid;
        for (const auto& rect : region) {
            for (int32_t y = rect.top; y < rect.bottom; ++y) {
                for (int32_t x = rect.left; x < rect.right; ++x) {
                    reference.at(x, y) = snapshot.pixels.at(static_cast<size_t>(y + dy) * grid_width + x + dx);
                }
            }
        }

        display::BandOrder order(direction);
        bool banded = true;
        for (const auto& rect : region) {
            banded = order.accept(rect) && banded;
        }

        // Batches of at most 64 through the dispatcher, executed in its order.
        const display::Rectangle bounds{0, 0, grid_width, grid_height};
        bool accepted = true;
        size_t next = 0;
        while (next < region.size()) {
            abi::RectList batch{};
            while (next < region.size() && batch.count < abi::max_rects) {
                display::append_clipped(bounds, region[next++], batch);
            }

            accepted = outcome(dispatcher, abi::ioctl_copy,
                               copy_request(grant.session, grant.primary_surface, grant.primary_surface, dx, dy,
                                            batch)) == abi::outcome_completed && accepted;
            ++requests;
            for (const auto& rect : backend.copy_rects) {
                execute(grid, rect);
            }
        }

        ++trials;
        exact += banded && accepted && grid.pixels == reference.pixels ? 1U : 0U;
        largest = std::max<unsigned>(largest, static_cast<unsigned>(region.size()));

        // The same region in a shuffled (unbanded) enumeration: BandOrder must
        // refuse whenever sequential execution would differ from the snapshot.
        std::shuffle(region.begin(), region.end(), random);
        display::BandOrder shuffled(direction);
        bool shuffled_banded = true;
        for (const auto& rect : region) {
            shuffled_banded = shuffled.accept(rect) && shuffled_banded;
        }

        Grid naive = snapshot;
        for (const auto& rect : region) {
            execute(naive, {{static_cast<uint32_t>(rect.left), static_cast<uint32_t>(rect.top),
                             static_cast<uint32_t>(rect.right - rect.left),
                             static_cast<uint32_t>(rect.bottom - rect.top)},
                            static_cast<uint32_t>(rect.left + dx), static_cast<uint32_t>(rect.top + dy)});
        }

        if (naive.pixels != reference.pixels) {
            ++naive_wrong;
            naive_refused += shuffled_banded ? 0U : 1U;
        }
    }

    check(exact == trials, "every banded >64-rectangle scroll equals the snapshot result");
    check(largest > 4 * abi::max_rects && requests > 4 * trials, "regions span many 64-rectangle requests");
    check(naive_wrong > 0 && naive_refused == naive_wrong,
          "every unordered enumeration that would corrupt pixels is refused by BandOrder");
    std::printf("batching: %u trials, largest region %u rectangles, %u requests, %u unordered corruptions refused\n",
                trials, largest, requests, naive_refused);
}

void bitmap_policy_tests() {
    const display::BitmapLimits limits{2, display::offscreen_bytes(256, 256) * 2};
    check(display::admit_bitmap(256, 256, 0, 0, limits) == display::BitmapAdmission::admit,
          "a large bitmap is admitted");
    check(display::admit_bitmap(32, 32, 0, 0, limits) == display::BitmapAdmission::too_small &&
          display::admit_bitmap(4096, 4, 0, 0, limits) == display::BitmapAdmission::too_small,
          "small and strip bitmaps stay engine DIBs");
    check(display::admit_bitmap(0, 64, 0, 0, limits) == display::BitmapAdmission::too_large &&
          display::admit_bitmap(16385, 64, 0, 0, limits) == display::BitmapAdmission::too_large &&
          display::admit_bitmap(-64, 64, 0, 0, limits) == display::BitmapAdmission::too_large,
          "invalid sizes refused");
    check(display::admit_bitmap(256, 256, 2, 0, limits) == display::BitmapAdmission::count_limit,
          "count limit");
    check(display::admit_bitmap(256, 256, 1, display::offscreen_bytes(256, 256) + 1, limits) ==
              display::BitmapAdmission::budget &&
          display::admit_bitmap(256, 256, 1, display::offscreen_bytes(256, 256), limits) ==
              display::BitmapAdmission::admit, "byte budget is exact");
    check(display::offscreen_bytes(65, 10) == 512U * 10U && display::offscreen_bytes(64, 1) == 256U,
          "budget prediction uses the 256-byte pitch granule");
}

// ---------------------------------------------------------------------------
// Compute raster frames (version 3) and their dispatch.
// ---------------------------------------------------------------------------
struct RasterBuffer final {
    std::vector<uint8_t> bytes;
    ByteView view() const { return ByteView(bytes.data(), bytes.size()); }
};

abi::Operand staging_operand(uint32_t offset, uint32_t pitch, uint32_t width, uint32_t height,
                             uint32_t format = abi::format_xrgb8888) {
    abi::Operand value{};
    value.kind = abi::operand_staging;
    value.handle = offset;
    value.pitch = pitch;
    value.width = width;
    value.height = height;
    value.format = format;
    return value;
}

abi::Operand surface_operand(uint32_t handle) {
    abi::Operand value{};
    value.kind = abi::operand_surface;
    value.handle = handle;
    value.format = abi::format_xrgb8888;
    return value;
}

RasterBuffer rop_request(const abi::RopCommand& command) {
    RasterBuffer buffer;
    buffer.bytes.assign(command.has_pattern ? abi::pattern_request_bytes : abi::rop_request_bytes, 0);
    check(abi::encode_rop(CommandBuffer(buffer.bytes.data(), buffer.bytes.size()), command), "rop encodes");
    return buffer;
}

abi::RopCommand basic_rop(abi::Header session, uint32_t target) {
    abi::RopCommand command{};
    command.session = session;
    command.target = target;
    command.rop4 = 0x5a5aU;               // PATINVERT: pattern and destination only
    command.brush_colour = 0x00112233U;
    command.bounds = {10, 10, 40, 30};
    command.map = {0, 0, 1, 1};
    command.rects = rects({{10, 10, 20, 30}, {20, 10, 40, 20}});
    return command;
}

uint32_t raster_outcome(r2d::Dispatcher& dispatcher, uint32_t code, const RasterBuffer& request) {
    abi::Receipt receipt{};
    const auto result = dispatcher.dispatch(code, request.view(), abi::receipt_bytes);
    return receipt_of(dispatcher, result, receipt) ? receipt.outcome : 0xffffffffU;
}

void raster_codec_tests() {
    abi::Header header{0x0102U, 0x55aa55aaU};
    abi::RopCommand rop = basic_rop(header, 9);
    rop.source = staging_operand(64, 128, 32, 20);
    rop.map = {-3, 4, 30, 20};
    rop.mask = staging_operand(4096, 128, 32, 20, abi::format_mono1);
    rop.mask_x = 7;
    rop.mask_y = -2;
    rop.flags = abi::rop_flag_colour_key | abi::rop_flag_skip_outside_source;
    rop.colour_key = 0x00abcdefU;
    rop.brush_x = -9;
    rop.mono_foreground = 1;
    rop.mono_background = 2;
    RasterBuffer bytes = rop_request(rop);
    abi::RopCommand decoded{};
    check(abi::decode_rop(bytes.view(), false, decoded) && decoded.map.left == -3 && decoded.mask_y == -2 &&
              decoded.mask.format == abi::format_mono1 && decoded.source.handle == 64 &&
              decoded.colour_key == 0x00abcdefU && decoded.brush_x == -9 && decoded.rects.count == 2 &&
              !decoded.has_pattern, "rop frame round-trips operands, map and signed origins");
    check(!abi::decode_rop(bytes.view(), true, decoded), "a rop frame is not a pattern frame");
    RasterBuffer bad = bytes;
    bad.bytes[52] = 1;
    check(!abi::decode_rop(bad.view(), false, decoded), "rop reserved word must be zero");
    bad = bytes;
    bad.bytes[24] = 0x80;
    check(!abi::decode_rop(bad.view(), false, decoded), "unknown rop flag refused");
    bad = bytes;
    bad.bytes[22] = 1;  // rop4 0x10000 + ...
    check(!abi::decode_rop(bad.view(), false, decoded), "rop4 above 0xffff refused");
    bad = bytes;
    bad.bytes[72] = 3;  // unknown operand kind
    check(!abi::decode_rop(bad.view(), false, decoded), "unknown operand kind refused");
    bad = bytes;
    bad.bytes[72 + 20] = abi::format_coverage8;
    check(!abi::decode_rop(bad.view(), false, decoded), "coverage8 is never a rop source");
    bad = bytes;
    bad.bytes[72 + 8] = 124;  // pitch 124 < 4 * 32
    check(!abi::decode_rop(bad.view(), false, decoded), "staging pitch below 4 * width refused");
    bad = bytes;
    bad.bytes[96 + 8] = 0;    // map width 0
    check(!abi::decode_rop(bad.view(), false, decoded), "zero map width refused");
    bad = bytes;
    bad.bytes[112 + 20] = abi::format_xrgb8888;
    check(!abi::decode_rop(bad.view(), false, decoded), "a mask must be mono1");
    abi::RopCommand outside = rop;
    outside.rects = rects({{10, 10, 41, 20}});
    check(!abi::decode_rop(rop_request(outside).view(), false, decoded), "clip rectangle outside bounds refused");
    abi::RopCommand huge = rop;
    huge.bounds = {0, 0, 16385, 10};
    huge.rects = rects({{0, 0, 10, 10}});
    check(!abi::decode_rop(rop_request(huge).view(), false, decoded), "bounds wider than 16384 refused");
    abi::RopCommand surface_source = rop;
    surface_source.source = surface_operand(5);
    surface_source.source.width = 1;
    check(!abi::decode_rop(rop_request(surface_source).view(), false, decoded),
          "surface operand carries no geometry");

    abi::RopCommand pattern = basic_rop(header, 9);
    pattern.has_pattern = true;
    for (uint32_t index = 0; index < 64; ++index) {
        pattern.pattern[index] = 0x01000000U + index;
    }

    abi::RopCommand pattern_decoded{};
    check(abi::decode_rop(rop_request(pattern).view(), true, pattern_decoded) && pattern_decoded.has_pattern &&
              pattern_decoded.pattern[63] == 0x0100003fU && abi::pattern_request_bytes == 1432,
          "pattern frame carries the 8x8 tile");

    abi::TextCommand text{};
    text.session = header;
    text.target = 9;
    text.mode = abi::text_mode_coverage8;
    text.colour = 0x00ff8800U;
    text.bounds = {0, 0, 16, 8};
    text.mask = staging_operand(0, 64, 16, 8, abi::format_coverage8);
    text.rects = rects({{0, 0, 16, 8}});
    std::vector<uint8_t> text_bytes(abi::text_request_bytes);
    abi::TextCommand text_decoded{};
    check(abi::encode_text(CommandBuffer(text_bytes.data(), text_bytes.size()), text) &&
              abi::decode_text(ByteView(text_bytes.data(), text_bytes.size()), text_decoded) &&
              text_decoded.mode == abi::text_mode_coverage8 && text_decoded.colour == 0x00ff8800U,
          "text frame round-trips");
    text.mode = abi::text_mode_mono;
    check(abi::encode_text(CommandBuffer(text_bytes.data(), text_bytes.size()), text) &&
              !abi::decode_text(ByteView(text_bytes.data(), text_bytes.size()), text_decoded),
          "mono text needs a mono1 mask");

    abi::AlphaCommand alpha{};
    alpha.session = header;
    alpha.target = 9;
    alpha.constant_alpha = 200;
    alpha.flags = abi::alpha_flag_source_alpha;
    alpha.bounds = {0, 0, 20, 20};
    alpha.source = surface_operand(3);
    alpha.map = {5, 5, 10, 40};
    alpha.rects = rects({{0, 0, 20, 20}});
    std::vector<uint8_t> alpha_bytes(abi::alpha_request_bytes);
    abi::AlphaCommand alpha_decoded{};
    check(abi::encode_alpha(CommandBuffer(alpha_bytes.data(), alpha_bytes.size()), alpha) &&
              abi::decode_alpha(ByteView(alpha_bytes.data(), alpha_bytes.size()), alpha_decoded) &&
              alpha_decoded.constant_alpha == 200 && alpha_decoded.map.height == 40, "alpha frame round-trips");
    alpha.constant_alpha = 256;
    check(abi::encode_alpha(CommandBuffer(alpha_bytes.data(), alpha_bytes.size()), alpha) &&
              !abi::decode_alpha(ByteView(alpha_bytes.data(), alpha_bytes.size()), alpha_decoded),
          "SourceConstantAlpha above 255 refused");

    abi::LineCommand line{};
    line.session = header;
    line.target = 9;
    line.colour = 7;
    line.rop3 = 0xf0;
    line.flags = abi::line_flag_styled;
    line.x0 = -5;
    line.y0 = 3;
    line.x1 = 100;
    line.y1 = 50;
    line.style_bit_count = 12;
    line.style[0] = 0xf0;
    line.style[1] = 0xa0;
    line.rects = rects({{0, 0, 50, 50}});
    std::vector<uint8_t> line_bytes(abi::line_request_bytes);
    abi::LineCommand line_decoded{};
    check(abi::encode_line(CommandBuffer(line_bytes.data(), line_bytes.size()), line) &&
              abi::decode_line(ByteView(line_bytes.data(), line_bytes.size()), line_decoded) &&
              line_decoded.x0 == -5 && line_decoded.style[0] == 0xf0 && line_decoded.style[1] == 0xa0 &&
              line_decoded.style_bit_count == 12, "styled line round-trips its style bits");
    line.style[1] = 0xa8;  // bit 12 is beyond the 12-bit count
    check(abi::encode_line(CommandBuffer(line_bytes.data(), line_bytes.size()), line) &&
              !abi::decode_line(ByteView(line_bytes.data(), line_bytes.size()), line_decoded),
          "style bits above the count must be zero");
    line.style[1] = 0xa0;
    line.flags = 0;
    check(abi::encode_line(CommandBuffer(line_bytes.data(), line_bytes.size()), line) &&
              !abi::decode_line(ByteView(line_bytes.data(), line_bytes.size()), line_decoded),
          "an unstyled line carries no style bits");

    abi::GradientCommand gradient{};
    gradient.session = header;
    gradient.target = 9;
    gradient.mode = abi::gradient_triangle;
    gradient.extents = {0, 0, 64, 64};
    gradient.vertices[0] = {1, 2, 0xff00, 0x1200, 0x3400};
    gradient.vertices[1] = {60, 5, 0x0000, 0xff00, 0x0000};
    gradient.vertices[2] = {30, 63, 0x0000, 0x0000, 0xff00};
    gradient.rects = rects({{0, 0, 64, 64}});
    std::vector<uint8_t> gradient_bytes(abi::gradient_request_bytes);
    abi::GradientCommand gradient_decoded{};
    check(abi::encode_gradient(CommandBuffer(gradient_bytes.data(), gradient_bytes.size()), gradient) &&
              abi::decode_gradient(ByteView(gradient_bytes.data(), gradient_bytes.size()), gradient_decoded) &&
              gradient_decoded.vertices[2].blue == 0xff00 && gradient_decoded.vertices[0].green == 0x1200,
          "gradient triangle frame round-trips 16-bit channels");
    gradient.mode = abi::gradient_rect_h;
    check(abi::encode_gradient(CommandBuffer(gradient_bytes.data(), gradient_bytes.size()), gradient) &&
              !abi::decode_gradient(ByteView(gradient_bytes.data(), gradient_bytes.size()), gradient_decoded),
          "a rect gradient has zero extents and no third vertex");
    gradient.extents = {};
    gradient.vertices[2] = {};
    check(abi::encode_gradient(CommandBuffer(gradient_bytes.data(), gradient_bytes.size()), gradient) &&
              abi::decode_gradient(ByteView(gradient_bytes.data(), gradient_bytes.size()), gradient_decoded),
          "rect gradient frame");
}

void raster_dispatch_tests() {
    FakeBackend backend;
    r2d::Dispatcher dispatcher;
    r2d::Blit2dBackend incomplete = raster_table(backend);
    incomplete.raster_line = nullptr;
    check(!dispatcher.configure(incomplete, primary(), staging_size, 9), "an advertised op needs its callback");
    check(dispatcher.configure(raster_table(backend), primary(), staging_size, 9), "raster configuration");
    const abi::Capabilities capabilities = query(dispatcher);
    check((capabilities.flags & abi::capability_raster) != 0 &&
              capabilities.operations == (abi::implemented_operations | abi::raster_operations),
          "capabilities advertise the compute raster codes");
    abi::SessionGrant grant{};
    abi::SessionGrant other{};
    check(open_primary(dispatcher, grant) && open(dispatcher, other) &&
              (grant.flags & abi::capability_raster) != 0, "raster sessions");
    const uint32_t surface = grant.primary_surface;

    abi::RopCommand rop = basic_rop(grant.session, surface);
    check(raster_outcome(dispatcher, abi::ioctl_rop, rop_request(rop)) == abi::outcome_completed &&
              backend.raster_rops == 1 && backend.rects.size() == 2 && backend.rects[1].width == 20 &&
              !backend.last_rop.source.present && backend.last_rop.pattern == nullptr &&
              backend.last_rop.brush_colour == 0x00112233U, "sourceless ROP reaches the compute backend");

    abi::RopCommand needs_source = rop;
    needs_source.rop4 = 0x6666U;  // SRCINVERT without a source
    check(dispatcher.dispatch(abi::ioctl_rop, rop_request(needs_source).view(), abi::receipt_bytes).status ==
              r2d::DispatchStatus::invalid_parameter, "a ROP that reads the source needs one");
    abi::RopCommand halves = rop;
    halves.rop4 = 0xaaf0U;
    check(dispatcher.dispatch(abi::ioctl_rop, rop_request(halves).view(), abi::receipt_bytes).status ==
              r2d::DispatchStatus::invalid_parameter, "distinct ROP4 halves need a mask");

    abi::RopCommand staged = rop;
    staged.rop4 = 0x6666U;
    staged.source = staging_operand(0, 128, 30, 20);
    staged.map = {0, 0, 30, 20};
    check(raster_outcome(dispatcher, abi::ioctl_rop, rop_request(staged)) == abi::outcome_completed &&
              backend.last_rop.source.present &&
              backend.last_rop.source.surface.gpu_address == 0x8000000ULL &&
              backend.last_rop.source.surface.pitch_bytes == 128, "staging source resolved inside staging");
    abi::RopCommand overflow = staged;
    overflow.source = staging_operand(staging_size - 128 * 19, 128, 30, 20);
    check(dispatcher.dispatch(abi::ioctl_rop, rop_request(overflow).view(), abi::receipt_bytes).status ==
              r2d::DispatchStatus::invalid_parameter, "a staging operand ending past the allocation refused");
    abi::RopCommand stretched = staged;
    stretched.map = {0, 0, 15, 20};
    check(dispatcher.dispatch(abi::ioctl_rop, rop_request(stretched).view(), abi::receipt_bytes).status ==
              r2d::DispatchStatus::invalid_parameter, "0xa10 is 1:1 only");
    check(raster_outcome(dispatcher, abi::ioctl_stretch, rop_request(stretched)) == abi::outcome_completed &&
              backend.last_rop.map.width == 15, "0xa14 carries the stretch map");
    check(dispatcher.dispatch(abi::ioctl_stretch, rop_request(rop).view(), abi::receipt_bytes).status ==
              r2d::DispatchStatus::invalid_parameter, "0xa14 needs a source");

    abi::RopCommand screen = staged;
    screen.source = surface_operand(surface);
    screen.map = {200, 200, 30, 20};
    check(raster_outcome(dispatcher, abi::ioctl_rop, rop_request(screen)) == abi::outcome_completed,
          "disjoint same-surface ROP accepted");
    screen.map = {25, 15, 30, 20};
    const unsigned rops = backend.raster_rops;
    check(raster_outcome(dispatcher, abi::ioctl_rop, rop_request(screen)) == abi::outcome_declined &&
              backend.raster_rops == rops, "overlapping same-surface compute ROP declined (no safe order)");

    abi::RopCommand foreign = staged;
    foreign.session = other.session;
    foreign.target = surface;
    check(dispatcher.dispatch(abi::ioctl_rop, rop_request(foreign).view(), abi::receipt_bytes).status ==
              r2d::DispatchStatus::invalid_parameter, "the primary of another session is refused");

    abi::RopCommand pattern = rop;
    pattern.has_pattern = true;
    pattern.pattern[0] = 0x00c0ffeeU;
    check(raster_outcome(dispatcher, abi::ioctl_pattern, rop_request(pattern)) == abi::outcome_completed &&
              backend.last_pattern0 == 0x00c0ffeeU, "pattern tile reaches the backend");

    abi::TextCommand text{};
    text.session = grant.session;
    text.target = surface;
    text.mode = abi::text_mode_mono;
    text.colour = 0x00010203U;
    text.bounds = {100, 100, 132, 116};
    text.mask = staging_operand(256, 128, 32, 16, abi::format_mono1);
    text.mask_x = 3;
    text.rects = rects({{100, 100, 132, 116}});
    std::vector<uint8_t> text_bytes(abi::text_request_bytes);
    abi::encode_text(CommandBuffer(text_bytes.data(), text_bytes.size()), text);
    abi::Receipt receipt{};
    auto result = dispatcher.dispatch(abi::ioctl_text, ByteView(text_bytes.data(), text_bytes.size()),
                                      abi::receipt_bytes);
    check(receipt_of(dispatcher, result, receipt) && receipt.outcome == abi::outcome_completed &&
              backend.last_rop.rop4 == 0xaaf0U && backend.last_rop.mask.present &&
              backend.last_rop.mask_x == 3 && backend.last_rop.brush_colour == 0x00010203U,
          "mono glyphs become ROP4_MASKPAINT with the solid colour");
    text.mode = abi::text_mode_coverage8;
    text.mask.format = abi::format_coverage8;
    abi::encode_text(CommandBuffer(text_bytes.data(), text_bytes.size()), text);
    result = dispatcher.dispatch(abi::ioctl_text, ByteView(text_bytes.data(), text_bytes.size()), abi::receipt_bytes);
    check(receipt_of(dispatcher, result, receipt) && receipt.outcome == abi::outcome_completed &&
              backend.last_alpha.coverage8 && backend.last_alpha.map.left == 3 && backend.last_alpha.map.width == 32 &&
              backend.last_alpha.colour == 0x00010203U, "coverage8 glyphs become the coverage blend");

    abi::AlphaCommand alpha{};
    alpha.session = grant.session;
    alpha.target = surface;
    alpha.constant_alpha = 128;
    alpha.flags = abi::alpha_flag_source_alpha;
    alpha.bounds = {0, 0, 20, 20};
    alpha.source = staging_operand(0, 80, 20, 20);
    alpha.map = {0, 0, 20, 20};
    alpha.rects = rects({{0, 0, 20, 20}});
    std::vector<uint8_t> alpha_bytes(abi::alpha_request_bytes);
    abi::encode_alpha(CommandBuffer(alpha_bytes.data(), alpha_bytes.size()), alpha);
    result = dispatcher.dispatch(abi::ioctl_alpha, ByteView(alpha_bytes.data(), alpha_bytes.size()), abi::receipt_bytes);
    check(receipt_of(dispatcher, result, receipt) && receipt.outcome == abi::outcome_completed &&
              backend.last_alpha.source_alpha && backend.last_alpha.constant_alpha == 128 &&
              !backend.last_alpha.coverage8, "AlphaBlend reaches the backend");

    abi::LineCommand line{};
    line.session = grant.session;
    line.target = surface;
    line.colour = 0x00ffffffU;
    line.rop3 = 0x5a;
    line.flags = abi::line_flag_styled;
    line.style_bit_count = 3;
    line.style[0] = 0xa0;
    line.x1 = 300;
    line.y1 = 200;
    line.rects = rects({{0, 0, 100, 100}, {100, 0, 200, 100}});
    std::vector<uint8_t> line_bytes(abi::line_request_bytes);
    abi::encode_line(CommandBuffer(line_bytes.data(), line_bytes.size()), line);
    result = dispatcher.dispatch(abi::ioctl_line, ByteView(line_bytes.data(), line_bytes.size()), abi::receipt_bytes);
    check(receipt_of(dispatcher, result, receipt) && receipt.outcome == abi::outcome_completed &&
              backend.last_line.styled && backend.last_style0 == 0xa0 && backend.last_line.rop3 == 0x5a &&
              backend.rects.size() == 2, "styled line and its clip rectangles reach the backend");

    abi::GradientCommand gradient{};
    gradient.session = grant.session;
    gradient.target = surface;
    gradient.mode = abi::gradient_rect_v;
    gradient.vertices[0] = {0, 0, 0xff00, 0, 0};
    gradient.vertices[1] = {50, 80, 0, 0, 0xff00};
    gradient.rects = rects({{0, 0, 50, 80}});
    std::vector<uint8_t> gradient_bytes(abi::gradient_request_bytes);
    abi::encode_gradient(CommandBuffer(gradient_bytes.data(), gradient_bytes.size()), gradient);
    result = dispatcher.dispatch(abi::ioctl_gradient, ByteView(gradient_bytes.data(), gradient_bytes.size()),
                                 abi::receipt_bytes);
    check(receipt_of(dispatcher, result, receipt) && receipt.outcome == abi::outcome_completed &&
              backend.last_gradient.mode == abi::gradient_rect_v && backend.last_gradient.vertices[1].blue == 0xff00,
          "gradient reaches the backend");

    // Offscreen targets and sources; lost surfaces; backend declines.
    const uint32_t offscreen = create_surface(dispatcher, grant.session, 64, 64);
    abi::RopCommand off = rop;
    off.target = offscreen;
    off.bounds = {0, 0, 64, 64};
    off.rects = rects({{0, 0, 64, 64}});
    check(raster_outcome(dispatcher, abi::ioctl_rop, rop_request(off)) == abi::outcome_completed,
          "offscreen ROP target");
    abi::RopCommand from_off = staged;
    from_off.source = surface_operand(offscreen);
    check(raster_outcome(dispatcher, abi::ioctl_rop, rop_request(from_off)) == abi::outcome_completed &&
              backend.last_rop.source.surface.width == 64, "offscreen ROP source");
    const unsigned destroyed_syncs = backend.syncs;
    check(outcome(dispatcher, abi::ioctl_surface_destroy, surface_destroy_request(grant.session, offscreen)) ==
              abi::outcome_completed && backend.syncs >= destroyed_syncs,
          "a surface used by compute work is destroyed after its fence");
    backend.next_status = r2d::BackendStatus::rejected;
    check(raster_outcome(dispatcher, abi::ioctl_rop, rop_request(rop)) == abi::outcome_declined,
          "a backend refusal is a declined receipt");
    backend.next_status = r2d::BackendStatus::uncertain;
    check(raster_outcome(dispatcher, abi::ioctl_rop, rop_request(rop)) == abi::outcome_lost &&
              raster_outcome(dispatcher, abi::ioctl_rop, rop_request(rop)) == abi::outcome_lost,
          "an uncertain compute outcome poisons the session");
    backend.next_status = r2d::BackendStatus::ok;

    // Without raster callbacks every raster code answers unknown_code.
    FakeBackend plain;
    r2d::Dispatcher sdma_only;
    abi::SessionGrant plain_grant{};
    check(sdma_only.configure(table(plain), primary(), staging_size, 10) && open_primary(sdma_only, plain_grant) &&
              sdma_only.dispatch(abi::ioctl_rop, rop_request(basic_rop(plain_grant.session,
                                 plain_grant.primary_surface)).view(), abi::receipt_bytes).status ==
                  r2d::DispatchStatus::unknown_code && (query(sdma_only).flags & abi::capability_raster) == 0,
          "an SDMA-only backend advertises and accepts no raster request");
}

// ---------------------------------------------------------------------------
// Compute raster admission and the engine-arithmetic helpers of the DLL.
// ---------------------------------------------------------------------------
// Direct port of the pinned EngTransparentBlt + DIB_32BPP_TransparentBlt
// source selection (eng/transblt.c, dib/dib32bpp.c): for every destination
// pixel the source pixel it would copy, or (-1, -1) when it is not written.
struct SourcePick final {
    int32_t x = -1;
    int32_t y = -1;
};

using Rect = display::Rectangle;

void reference_dib_pick(std::vector<SourcePick>& picks, int32_t width, const Rect& dest, const Rect& src,
                        int32_t src_cx, int32_t src_cy) {
    const int64_t dh = dest.bottom - dest.top, dw = dest.right - dest.left;
    const int64_t sh = src.bottom - src.top, sw = src.right - src.left;
    for (int32_t y = dest.top; y < dest.bottom; ++y) {
        const int64_t sy = src.top + (y - dest.top) * sh / dh;
        for (int32_t x = dest.left; x < dest.right; ++x) {
            const int64_t sx = src.left + (x - dest.left) * sw / dw;
            if (sx >= 0 && sy >= 0 && src_cx > sx && src_cy > sy) {
                picks[static_cast<size_t>(y) * width + x] = {static_cast<int32_t>(sx), static_cast<int32_t>(sy)};
            }
        }
    }
}

void reference_transparent(std::vector<SourcePick>& picks, int32_t width, Rect dest, Rect src, bool complex,
                           const Rect& bounds, const std::vector<Rect>& rects, int32_t src_cx, int32_t src_cy) {
    Rect input = src;
    Rect output = dest;
    if (output.left < bounds.left) { input.left += bounds.left - output.left; output.left = bounds.left; }
    if (bounds.right < output.right) { input.right -= output.right - bounds.right; output.right = bounds.right; }
    if (output.top < bounds.top) { input.top += bounds.top - output.top; output.top = bounds.top; }
    if (bounds.bottom < output.bottom) { input.bottom -= output.bottom - bounds.bottom; output.bottom = bounds.bottom; }
    if (output.right <= output.left || output.bottom <= output.top) {
        return;
    }

    output = dest;
    const int64_t dh = output.bottom - output.top, dw = output.right - output.left;
    const int64_t sh = input.bottom - input.top, sw = input.right - input.left;
    for (const Rect& clip : complex ? rects : std::vector<Rect>{bounds}) {
        const Rect combined = rx7900::reactos::framebuffer::intersect(output, clip);
        if (rx7900::reactos::framebuffer::empty(combined)) {
            continue;
        }

        Rect icr{};
        icr.top = static_cast<int32_t>(input.top + (combined.top - output.top) * sh / dh);
        icr.bottom = static_cast<int32_t>(input.top + (combined.bottom - output.top) * sh / dh);
        icr.left = static_cast<int32_t>(input.left + (combined.left - output.left) * sw / dw);
        icr.right = static_cast<int32_t>(input.left + (combined.right - output.left) * sw / dw);
        reference_dib_pick(picks, width, combined, icr, src_cx, src_cy);
    }
}

// The kernel's nearest map: src = base + floor((x - bounds.left) * mag / den).
void kernel_pick(std::vector<SourcePick>& picks, int32_t width, const Rect& combined, const Rect& input,
                 int32_t src_cx, int32_t src_cy) {
    const int64_t mw = input.right - input.left, mh = input.bottom - input.top;
    const int64_t dw = combined.right - combined.left, dh = combined.bottom - combined.top;
    for (int32_t y = combined.top; y < combined.bottom; ++y) {
        const int64_t sy = input.top + (y - combined.top) * mh / dh;
        for (int32_t x = combined.left; x < combined.right; ++x) {
            const int64_t sx = input.left + (x - combined.left) * mw / dw;
            if (sx >= 0 && sy >= 0 && sx < src_cx && sy < src_cy) {
                picks[static_cast<size_t>(y) * width + x] = {static_cast<int32_t>(sx), static_cast<int32_t>(sy)};
            }
        }
    }
}

bool same_picks(const std::vector<SourcePick>& a, const std::vector<SourcePick>& b) {
    for (size_t index = 0; index < a.size(); ++index) {
        if (a[index].x != b[index].x || a[index].y != b[index].y) {
            return false;
        }
    }

    return true;
}

void raster_policy_tests() {
    using display::RasterSource;
    using display::SurfaceKind;
    check(display::rop4_uses_source(0xcccc) && !display::rop4_uses_source(0xf0f0) &&
              display::rop4_uses_pattern(0x5a5a) && !display::rop4_uses_pattern(0x6666) &&
              display::rop4_uses_destination(0x5555) && !display::rop4_uses_destination(0xcccc) &&
              display::rop4_uses_destination(0xaaf0) && display::rop4_uses_source(0xccaa),
          "ROP3/ROP4 operand use");
    check(display::compute_rop_admitted(0x6666, false, false, SurfaceKind::primary, RasterSource::bitmap32, false, false),
          "SRCINVERT from a 32bpp bitmap is a compute ROP");
    check(display::compute_rop_admitted(0xaaf0, true, true, SurfaceKind::primary, RasterSource::none, true, false),
          "ROP4_MASKPAINT with a 1bpp mask and solid brush (ReactOS mono glyphs)");
    check(!display::compute_rop_admitted(0xaaf0, false, false, SurfaceKind::primary, RasterSource::none, true, false),
          "distinct ROP4 halves need a mask");
    check(!display::compute_rop_admitted(0xaaf0, true, false, SurfaceKind::primary, RasterSource::none, true, false),
          "only 1bpp masks");
    check(!display::compute_rop_admitted(0x5a5a, false, false, SurfaceKind::primary, RasterSource::none, false, false) &&
              display::compute_rop_admitted(0x5a5a, false, false, SurfaceKind::offscreen, RasterSource::none, false, true),
          "a pattern ROP needs a solid or realized brush");
    check(!display::compute_rop_admitted(0x6666, false, false, SurfaceKind::primary, RasterSource::unsupported, false,
                                         false) &&
              !display::compute_rop_admitted(0x6666, false, false, SurfaceKind::bitmap32, RasterSource::gpu, false, false) &&
              !display::compute_rop_admitted(0x10000, false, false, SurfaceKind::primary, RasterSource::none, true, false),
          "unsupported sources, engine-bitmap targets and ROP4 > 0xffff stay engine work");

    const uint8_t bits[] = {0xb5, 0x3c, 0xff, 0x01};
    uint8_t out[3]{};
    display::copy_mono_bits(bits, 3, 17, out);
    // Bits 3..19 of 10110101 00111100 11111111: 10101 00111100 1111
    check(out[0] == 0xa9 && out[1] == 0xe7 && out[2] == 0x80, "MSB-first bit realignment with zero padding");

    check(display::tagged_copy_pen(display::line_mix_tag | 0x0d0d) && display::tagged_copy_pen(display::line_mix_tag | 0x0d) &&
              !display::tagged_copy_pen(0x0d0d) && !display::tagged_copy_pen(display::line_mix_tag | 0x0606),
          "only tagged R2_COPYPEN lines go to the GPU");
    check(display::staged_rows(1920, 0, 512U << 10) == 68 && display::staged_rows(1920, 1920, 512U << 10) == 34 &&
              display::staged_rows(0, 0, 4096) == abi::max_surface_dimension &&
              display::staged_rows(200000, 0, 4096) == 0, "staging band rows");
    const abi::Vertex a{0, 0, 1, 2, 3};
    const abi::Vertex b{10, 0, 1, 2, 3};
    const abi::Vertex c{0, 600, 9, 2, 3};
    check(display::null_triangle(a, a, c) && !display::null_triangle(a, b, c) && !display::solid_triangle(a, b, c) &&
              display::solid_triangle(a, b, b) && display::triangle_rows(a, b, c) == 600,
          "triangle classification");

    // TransparentBlt: the helpers composed with the kernel map choose exactly
    // the source pixels of the engine for random stretches and clips.
    std::mt19937 random(0x7b17U);
    unsigned exact = 0;
    unsigned cases = 0;
    constexpr int32_t width = 96;
    constexpr int32_t height = 64;
    for (unsigned trial = 0; trial < 400; ++trial) {
        const int32_t dl = static_cast<int32_t>(random() % 40);
        const int32_t dt = static_cast<int32_t>(random() % 30);
        const Rect dest{dl, dt, dl + 1 + static_cast<int32_t>(random() % 50), dt + 1 + static_cast<int32_t>(random() % 30)};
        const int32_t sl = static_cast<int32_t>(random() % 20) - 4;
        const int32_t st = static_cast<int32_t>(random() % 20) - 4;
        const Rect src{sl, st, sl + 1 + static_cast<int32_t>(random() % 60), st + 1 + static_cast<int32_t>(random() % 40)};
        const Rect bounds{static_cast<int32_t>(random() % 30), static_cast<int32_t>(random() % 20),
                          60 + static_cast<int32_t>(random() % 36), 40 + static_cast<int32_t>(random() % 24)};
        std::vector<Rect> clip_rects;
        for (int32_t y = bounds.top; y < bounds.bottom; y += 7) {
            for (int32_t x = bounds.left; x < bounds.right; x += 11) {
                clip_rects.push_back({x, y, std::min(x + 9, bounds.right), std::min(y + 7, bounds.bottom)});
            }
        }

        const bool complex = (trial % 2) == 0;
        std::vector<SourcePick> expected(static_cast<size_t>(width) * height);
        std::vector<SourcePick> ours(static_cast<size_t>(width) * height);
        reference_transparent(expected, width, dest, src, complex, bounds, clip_rects, 40, 30);
        display::StretchSetup setup{};
        if (display::transparent_setup(dest, src, true, bounds, setup)) {
            bool valid = true;
            for (const Rect& clip : complex ? clip_rects : std::vector<Rect>{bounds}) {
                Rect combined{};
                Rect input{};
                if (display::transparent_rect(setup, clip, combined, input)) {
                    valid = valid && input.right > input.left && input.bottom > input.top;
                    if (input.right > input.left && input.bottom > input.top) {
                        kernel_pick(ours, width, combined, input, 40, 30);
                    }
                }
            }

            if (!valid) {
                continue;   // the DLL falls back for a non-positive source span
            }
        }

        ++cases;
        exact += same_picks(expected, ours) ? 1U : 0U;
    }

    check(cases > 300 && exact == cases, "TransparentBlt rectangles + nearest map equal EngTransparentBlt pixels");

    // AlphaBlend: the per-rectangle remap of EngAlphaBlend + DIB map.
    unsigned alpha_exact = 0;
    unsigned alpha_cases = 0;
    for (unsigned trial = 0; trial < 400; ++trial) {
        const int32_t dl = static_cast<int32_t>(random() % 40);
        const int32_t dt = static_cast<int32_t>(random() % 30);
        const Rect output{dl, dt, dl + 1 + static_cast<int32_t>(random() % 50), dt + 1 + static_cast<int32_t>(random() % 30)};
        const int32_t sl = static_cast<int32_t>(random() % 10);
        const int32_t st = static_cast<int32_t>(random() % 10);
        const Rect input{sl, st, sl + 1 + static_cast<int32_t>(random() % 30), st + 1 + static_cast<int32_t>(random() % 20)};
        const Rect clip{static_cast<int32_t>(random() % 50), static_cast<int32_t>(random() % 40),
                        50 + static_cast<int32_t>(random() % 46), 40 + static_cast<int32_t>(random() % 24)};
        Rect combined{};
        Rect source{};
        if (!display::alpha_rect(output, input, clip, combined, source)) {
            continue;
        }

        // Reference: Rect exactly as eng/alphablend.c computes it.
        const int64_t iw = input.right - input.left, ih = input.bottom - input.top;
        const int64_t ow = output.right - output.left, oh = output.bottom - output.top;
        const Rect expected{static_cast<int32_t>(input.left + (combined.left - output.left) * iw / ow),
                            static_cast<int32_t>(input.top + (combined.top - output.top) * ih / oh),
                            static_cast<int32_t>(input.right + (combined.right - output.right) * iw / ow),
                            static_cast<int32_t>(input.bottom + (combined.bottom - output.bottom) * ih / oh)};
        ++alpha_cases;
        alpha_exact += source.left == expected.left && source.top == expected.top && source.right == expected.right &&
                       source.bottom == expected.bottom ? 1U : 0U;
    }

    check(alpha_cases > 100 && alpha_exact == alpha_cases,
          "AlphaBlend per-rectangle source equals eng/alphablend.c (truncating, asymmetric right/bottom)");

    Rect span{};
    check(display::mapped_span(-3, 2, 10, 5, 6, 6, span) && span.left == 0 && span.right == 6 && span.top == 2 &&
              span.bottom == 6 && !display::mapped_span(10, 0, 4, 4, 8, 8, span), "mapped source span clamps to the bitmap");
}

// ---------------------------------------------------------------------------
// Guarded StretchBltROP (0xa17) and pattern lines (abi version 4).
// ---------------------------------------------------------------------------
RasterBuffer stretch_rop_request(const abi::StretchRopCommand& command) {
    RasterBuffer buffer;
    buffer.bytes.assign(abi::stretch_rop_request_bytes, 0);
    check(abi::encode_stretch_rop(CommandBuffer(buffer.bytes.data(), buffer.bytes.size()), command),
          "stretch ROP encodes");
    return buffer;
}

RasterBuffer line_request(const abi::LineCommand& command) {
    RasterBuffer buffer;
    buffer.bytes.assign(abi::line_request_bytes, 0);
    check(abi::encode_line(CommandBuffer(buffer.bytes.data(), buffer.bytes.size()), command), "line encodes");
    return buffer;
}

bool decodes_stretch(const abi::StretchRopCommand& command) {
    std::vector<uint8_t> bytes(abi::stretch_rop_request_bytes);
    abi::StretchRopCommand decoded{};
    return abi::encode_stretch_rop(CommandBuffer(bytes.data(), bytes.size()), command) &&
           abi::decode_stretch_rop(ByteView(bytes.data(), bytes.size()), decoded);
}

void stretch_codec_tests() {
    abi::StretchRopCommand command{};
    command.session = {3, 0x77U};
    command.target = 2;
    command.rop4 = 0x66ccU;
    command.flags = abi::stretch_flag_mirror_x | abi::stretch_flag_tile;
    command.brush_x = -7;
    command.brush_y = 5;
    command.bounds = {-4, 2, 30, 40};
    command.source = surface_operand(2);
    command.map = {3, -2, 0, 17};
    command.window = {1, 0, 24, 20};
    command.mask = staging_operand(64, 32, 8, 6, abi::format_mono1);
    command.mask_x = -9;
    command.mask_y = 11;
    command.rects = rects({{0, 2, 30, 40}});
    for (uint32_t index = 0; index < 64; ++index) {
        command.pattern[index] = 0x01010101U * index;
    }

    std::vector<uint8_t> bytes(abi::stretch_rop_request_bytes);
    abi::StretchRopCommand decoded{};
    check(abi::stretch_rop_request_bytes == 1440 && abi::line_request_bytes == 1608 &&
              abi::max_request_bytes == abi::line_request_bytes, "version 4 frame sizes");
    check(abi::encode_stretch_rop(CommandBuffer(bytes.data(), bytes.size()), command) &&
              abi::decode_stretch_rop(ByteView(bytes.data(), bytes.size()), decoded) && decoded.rop4 == 0x66ccU &&
              decoded.flags == command.flags && decoded.brush_x == -7 && decoded.map.top == -2 &&
              decoded.map.width == 0 && decoded.window.right == 24 && decoded.mask.kind == abi::operand_staging &&
              decoded.mask_x == -9 && decoded.mask_y == 11 && decoded.pattern[63] == 0x3f3f3f3fU &&
              decoded.rects.count == 1, "stretch ROP round-trips (empty map axis, signed origins, tile)");
    auto refused = [&](void (*mutate)(abi::StretchRopCommand&), const char* what) {
        abi::StretchRopCommand bad = command;
        mutate(bad);
        check(!decodes_stretch(bad), what);
    };
    refused([](abi::StretchRopCommand& c) { c.flags |= 16U; }, "unknown stretch flag refused");
    {
        abi::StretchRopCommand running = command;
        running.flags |= abi::stretch_flag_running_phase;
        check(decodes_stretch(running), "running phase with a tile and a mask decodes (v5)");
        abi::StretchRopCommand signed_map = command;
        signed_map.map.width = -16384;
        signed_map.map.height = -1;
        check(decodes_stretch(signed_map), "negative map extents down to -16384 decode (v5)");
    }
    refused([](abi::StretchRopCommand& c) {
        c.flags |= abi::stretch_flag_running_phase;
        c.mask = {};
        c.mask_x = 0;
        c.mask_y = 0;
    }, "running phase needs a mask");
    refused([](abi::StretchRopCommand& c) {
        c.flags = (c.flags | abi::stretch_flag_running_phase) & ~abi::stretch_flag_tile;
        for (uint32_t& texel : c.pattern) {
            texel = 0;
        }
    }, "running phase needs a tile");
    refused([](abi::StretchRopCommand& c) { c.rop4 = 0x10000U; }, "ROP4 above 0xffff refused");
    refused([](abi::StretchRopCommand& c) { c.brush_colour = 1; }, "a tile request carries no solid colour");
    refused([](abi::StretchRopCommand& c) { c.flags &= ~abi::stretch_flag_tile; }, "tile bytes without the flag refused");
    refused([](abi::StretchRopCommand& c) { c.map.width = -16385; }, "map extent below -16384 refused");
    refused([](abi::StretchRopCommand& c) { c.map.height = 16385; }, "map extent above 16384 refused");
    refused([](abi::StretchRopCommand& c) { c.window = {5, 0, 5, 20}; }, "empty window refused");
    refused([](abi::StretchRopCommand& c) { c.mono_foreground = 1; }, "mono colours need a mono1 source");
    refused([](abi::StretchRopCommand& c) { c.mask = staging_operand(64, 32, 8, 6); }, "mask must be mono1");
    refused([](abi::StretchRopCommand& c) { c.source = {}; c.map = {}; c.window = {}; },
            "a mask needs a source");
    refused([](abi::StretchRopCommand& c) { c.source = {}; c.mask = {}; c.mask_x = 0; c.mask_y = 0; },
            "no map or window without a source");
    refused([](abi::StretchRopCommand& c) { c.mask = {}; }, "mask origin zero without a mask");
    refused([](abi::StretchRopCommand& c) { c.bounds = {0, 0, 0, 10}; }, "empty bounds refused");
    refused([](abi::StretchRopCommand& c) { c.rects = rects({{-5, 2, 30, 40}}); }, "rectangle outside bounds refused");
    abi::StretchRopCommand mono = command;
    mono.source = staging_operand(0, 64, 16, 4, abi::format_mono1);
    mono.window = {0, 0, 16, 4};
    mono.mono_foreground = 0x00ffffffU;
    check(decodes_stretch(mono), "a mono1 staging source may carry XLATE colours");

    abi::LineCommand line{};
    line.session = {3, 0x77U};
    line.target = 2;
    line.rop3 = 0xf0;
    line.flags = abi::line_flag_pattern;
    line.x0 = -3;
    line.y0 = 4;
    line.x1 = 30;
    line.y1 = -9;
    line.brush_x = -2;
    line.brush_y = 13;
    line.pattern[5] = 0xabcdef01U;
    line.rects = rects({{0, 0, 10, 10}});
    std::vector<uint8_t> line_bytes(abi::line_request_bytes);
    abi::LineCommand back{};
    check(abi::encode_line(CommandBuffer(line_bytes.data(), line_bytes.size()), line) &&
              abi::decode_line(ByteView(line_bytes.data(), line_bytes.size()), back) && back.brush_x == -2 &&
              back.brush_y == 13 && back.pattern[5] == 0xabcdef01U && back.flags == abi::line_flag_pattern,
          "pattern line round-trips brush origin and tile");
    auto line_refused = [&](void (*mutate)(abi::LineCommand&), const char* what) {
        abi::LineCommand bad = line;
        mutate(bad);
        std::vector<uint8_t> raw(abi::line_request_bytes);
        abi::LineCommand out{};
        check(abi::encode_line(CommandBuffer(raw.data(), raw.size()), bad) &&
                  !abi::decode_line(ByteView(raw.data(), raw.size()), out), what);
    };
    line_refused([](abi::LineCommand& c) { c.colour = 1; }, "a pattern line carries no colour");
    line_refused([](abi::LineCommand& c) { c.flags |= abi::line_flag_styled; c.style_bit_count = 8; c.style[0] = 0x80; },
                 "styled and pattern are exclusive");
    line_refused([](abi::LineCommand& c) { c.flags = 0; }, "brush and tile zero without the pattern flag");
    line_refused([](abi::LineCommand& c) { c.flags = 4; }, "unknown line flag refused");
}

void stretch_dispatch_tests() {
    FakeBackend backend;
    r2d::Dispatcher dispatcher;
    check(dispatcher.configure(raster_table(backend), primary(), staging_size, 9), "stretch configuration");
    abi::SessionGrant grant{};
    check(open_primary(dispatcher, grant), "stretch session");
    const uint32_t surface = grant.primary_surface;

    abi::StretchRopCommand command{};
    command.session = grant.session;
    command.target = surface;
    command.rop4 = 0xccccU;
    command.flags = abi::stretch_flag_mirror_x | abi::stretch_flag_mirror_y;
    command.bounds = {100, 200, 140, 260};
    command.rects = rects({{100, 200, 140, 230}});
    command.source = surface_operand(surface);
    command.window = {5, 6, 25, 30};
    command.map = {7, 8, 10, 12};
    command.mask = staging_operand(256, 64, 9, 7, abi::format_mono1);
    command.mask_x = -3;
    command.mask_y = 2;
    check(raster_outcome(dispatcher, abi::ioctl_stretch_rop, stretch_rop_request(command)) == abi::outcome_completed &&
              backend.last_rop.mirror_x && backend.last_rop.mirror_y && backend.last_rop.stretch_mask &&
              backend.last_rop.source.surface.gpu_address == primary_address + 6ULL * screen_pitch + 5ULL * 4ULL &&
              backend.last_rop.source.surface.width == 20 && backend.last_rop.source.surface.height == 24 &&
              backend.last_rop.source.surface.pitch_bytes == screen_pitch && backend.last_rop.map.left == 2 &&
              backend.last_rop.map.top == 2 && backend.last_rop.map.width == 10 && backend.last_rop.mask_x == 2 &&
              backend.last_rop.mask_y == 8 && backend.last_rop.mask.surface.width == 9 &&
              backend.rects.size() == 1 && backend.rects[0].height == 30 &&
              backend.last_rop.flags == abi::rop_flag_skip_outside_source,
          "0xa17: mirrors, window crop, window-relative map and mask origin reach the backend");

    abi::StretchRopCommand tile = command;
    tile.rop4 = 0xc0c0U;   // MERGECOPY
    tile.flags = abi::stretch_flag_tile;
    tile.brush_x = -1;
    tile.mask = {};
    tile.mask_x = tile.mask_y = 0;
    tile.pattern[0] = 0x00c0ffeeU;
    check(raster_outcome(dispatcher, abi::ioctl_stretch_rop, stretch_rop_request(tile)) == abi::outcome_completed &&
              backend.last_pattern0 == 0x00c0ffeeU && !backend.last_rop.mirror_x && !backend.last_rop.stretch_mask &&
              backend.last_rop.brush_x == -1 && backend.last_rop.rop4 == 0xc0c0U && backend.last_rop.flags == 0,
          "tile brush reaches the backend (no SRCCOPY skip for MERGECOPY)");

    abi::StretchRopCommand patcopy = tile;
    patcopy.rop4 = 0xf0f0U;
    patcopy.source = {};
    patcopy.map = {};
    patcopy.window = {};
    check(raster_outcome(dispatcher, abi::ioctl_stretch_rop, stretch_rop_request(patcopy)) == abi::outcome_completed &&
              !backend.last_rop.source.present, "a sourceless ROP4 stretches without a source operand");

    auto invalid = [&](const abi::StretchRopCommand& bad, const char* what) {
        check(dispatcher.dispatch(abi::ioctl_stretch_rop, stretch_rop_request(bad).view(), abi::receipt_bytes).status ==
                  r2d::DispatchStatus::invalid_parameter, what);
    };
    abi::StretchRopCommand bad = patcopy;
    bad.rop4 = 0xccccU;
    invalid(bad, "a source-reading ROP4 needs a source");
    bad = command;
    bad.rop4 = 0xf0f0U;
    invalid(bad, "a source without a source-reading ROP4 is refused");
    bad = command;
    bad.window = {5, 6, 25, int32_t(screen_height) + 1};
    invalid(bad, "a window leaving the source surface is refused");
    bad = command;
    bad.source = staging_operand(0, 64, 16, 8);
    bad.window = {1, 0, 16, 8};
    bad.mask = {};
    bad.mask_x = bad.mask_y = 0;
    invalid(bad, "a staging window is the whole operand");
    bad.window = {0, 0, 16, 8};
    check(raster_outcome(dispatcher, abi::ioctl_stretch_rop, stretch_rop_request(bad)) == abi::outcome_completed &&
              backend.last_rop.source.surface.gpu_address == 0x8000000ULL, "whole staging window accepted uncropped");
    bad.mask = staging_operand(staging_size - 64, 64, 9, 7, abi::format_mono1);
    bad.mask_x = 1;
    invalid(bad, "a mask ending past the staging allocation is refused");

    // v5 translated sources: the operand offset names the XLATEOBJ table, the
    // rows follow it; a signed map and the running phase reach the backend.
    abi::StretchRopCommand translated = command;
    translated.rop4 = 0xb8b8U;
    translated.flags = abi::stretch_flag_tile | abi::stretch_flag_running_phase | abi::stretch_flag_mirror_y;
    translated.source = staging_operand(0, 16, 13, 5, abi::format_index8);
    translated.window = {0, 0, 13, 5};
    translated.map = {9, 1, -7, 3};
    translated.mask = staging_operand(2048, 64, 9, 7, abi::format_mono1);
    translated.pattern[5] = 0x00abcdefU;
    check(raster_outcome(dispatcher, abi::ioctl_stretch_rop, stretch_rop_request(translated)) == abi::outcome_completed &&
              backend.last_rop.source.table_address == 0x8000000ULL &&
              backend.last_rop.source.surface.gpu_address == 0x8000000ULL + 1024U &&
              backend.last_rop.source.surface.format == abi::format_index8 && backend.last_rop.map.width == -7 &&
              backend.last_rop.running_phase && backend.last_rop.mirror_y && !backend.last_rop.mirror_x,
          "0xa17 index8 source: table address, rows after it, signed map and running phase reach the backend");
    bad = translated;
    bad.source.pitch = 12;
    invalid(bad, "a translated row pitch below its pixel bytes is refused");
    bad = translated;
    bad.source = staging_operand(0, 64, 20, 20, abi::format_rgb24);
    bad.window = {0, 0, 20, 20};
    invalid(bad, "a translated table plus rows past the staging allocation is refused");
    bad = translated;
    bad.source = staging_operand(0, 16, 13, 5, abi::format_coverage8);
    invalid(bad, "a coverage8 operand is no stretch source");
    {
        abi::RopCommand rop{};
        rop.session = grant.session;
        rop.target = surface;
        rop.rop4 = 0xccccU;
        rop.bounds = {0, 0, 13, 5};
        rop.source = staging_operand(0, 16, 13, 5, abi::format_index8);
        rop.map = {0, 0, 13, 5};
        rop.rects = rects({{0, 0, 13, 5}});
        std::vector<uint8_t> raw(abi::rop_request_bytes);
        abi::RopCommand decoded{};
        check(abi::encode_rop(CommandBuffer(raw.data(), raw.size()), rop) &&
                  abi::decode_rop(ByteView(raw.data(), raw.size()), false, decoded) &&
                  decoded.source.format == abi::format_index8,
              "0xa10 carries a translated staging source (DrvBitBlt 8/16/24bpp)");
        rop.source.format = abi::format_coverage8;
        check(abi::encode_rop(CommandBuffer(raw.data(), raw.size()), rop) &&
                  !abi::decode_rop(ByteView(raw.data(), raw.size()), false, decoded),
              "a coverage8 operand is no rop source");
    }

    abi::StretchRopCommand overlap = command;
    overlap.window = {90, 190, 120, 230};
    overlap.map = {95, 195, 20, 20};
    check(raster_outcome(dispatcher, abi::ioctl_stretch_rop, stretch_rop_request(overlap)) == abi::outcome_declined,
          "a same-surface window meeting the bounds is declined");

    abi::LineCommand line{};
    line.session = grant.session;
    line.target = surface;
    line.rop3 = 0x5a;
    line.flags = abi::line_flag_pattern;
    line.x0 = 10;
    line.y0 = 10;
    line.x1 = 50;
    line.y1 = 30;
    line.brush_x = 3;
    line.brush_y = -4;
    line.pattern[0] = 0x00123456U;
    line.rects = rects({{0, 0, 100, 100}});
    check(raster_outcome(dispatcher, abi::ioctl_line, line_request(line)) == abi::outcome_completed &&
              backend.last_line.pattern != nullptr && backend.last_line_pattern0 == 0x00123456U &&
              backend.last_line.brush_x == 3 && backend.last_line.brush_y == -4 && backend.last_line.rop3 == 0x5a,
          "pattern line reaches the backend with its tile and origin");
    line.flags = 0;
    line.brush_x = line.brush_y = 0;
    line.pattern[0] = 0;
    line.colour = 0x00ff00ffU;
    check(raster_outcome(dispatcher, abi::ioctl_line, line_request(line)) == abi::outcome_completed &&
              backend.last_line.pattern == nullptr, "solid line carries no tile");
}

// DLL policy: the guarded StretchBltROP plan equals the engine fallback's DIB
// calls (tests/gdi_engine_reference.hpp port of IntEngStretchBlt and
// EngStretchBltROP), over every flip pair, clip kind and odd sizes.
void stretch_policy_tests() {
    namespace ref = gdi_reference;
    std::mt19937 rng(0x7a17);
    ref::Clip rect_clip{};
    rect_clip.kind = ref::Complexity::rect;
    rect_clip.bounds = {-3, 4, 61, 47};
    ref::Clip complex_clip{};
    complex_clip.kind = ref::Complexity::complex;
    complex_clip.bounds = {0, 0, 64, 50};
    complex_clip.rects = {{0, 0, 20, 12}, {25, 0, 64, 12}, {5, 12, 50, 31}, {0, 31, 33, 50}, {40, 31, 64, 50}};
    ref::Clip dc_trivial{};
    dc_trivial.bounds = {0, 0, 40, 30};   // a DC_TRIVIAL CLIPOBJ with real bounds: the unscaled pre-adjust
    const ref::Clip clips[] = {ref::Clip{}, rect_clip, complex_clip, dc_trivial};
    const uint32_t rops[] = {0xcccc, 0x6666, 0xf0f0, 0xaaaa, 0x5a5a};
    uint32_t cases = 0, exact = 0, flips_seen = 0, noop = 0, negative_extent = 0;

    for (uint32_t trial = 0; trial < 4000; ++trial) {
        const int32_t w = 1 + int32_t(rng() % 70), h = 1 + int32_t(rng() % 60);
        const int32_t sw = 1 + int32_t(rng() % 50), sh = 1 + int32_t(rng() % 40);
        const int32_t x = int32_t(rng() % 80) - 20, y = int32_t(rng() % 70) - 20;
        const int32_t sx = int32_t(rng() % 30) - 5, sy = int32_t(rng() % 30) - 5;
        const bool dneg_x = (rng() & 1) != 0, dneg_y = (rng() & 1) != 0, sneg_x = (rng() & 1) != 0,
                   sneg_y = (rng() & 1) != 0;
        const ref::Rect dest{dneg_x ? x + w : x, dneg_y ? y + h : y, dneg_x ? x : x + w, dneg_y ? y : y + h};
        const ref::Rect src{sneg_x ? sx + sw : sx, sneg_y ? sy + sh : sy, sneg_x ? sx : sx + sw, sneg_y ? sy : sy + sh};
        const ref::Clip& clip = clips[trial % 4];
        const uint32_t rop4 = rops[rng() % 5];
        const ref::DriverStretch drv = ref::int_eng_stretch_blt(dest, src, clip);
        if (drv.bitblt || drv.nothing) {
            continue;
        }

        ref::Bitmap32 target(64, 50), source(60, 50);
        ref::Source source_view{};
        source_view.colour = &source;
        std::vector<ref::DibCall> calls;
        const ref::Point origin{0, 0};
        ref::eng_stretch_blt_rop(target, source_view, nullptr, &clip, &origin, ref::fallback_destination(drv),
                                 drv.input, nullptr, rop4, &calls);

        const Rect bounds{clip.bounds.left, clip.bounds.top, clip.bounds.right, clip.bounds.bottom};
        display::StretchRopSetup setup{};
        const bool drawn = display::stretch_rop_setup({drv.output.left, drv.output.top, drv.output.right, drv.output.bottom},
                                                      {drv.input.left, drv.input.top, drv.input.right, drv.input.bottom},
                                                      drv.flip_x, drv.flip_y, rop4, &bounds, setup);
        std::vector<display::StretchCall> plan;
        if (drawn) {
            auto visit = [&](const Rect* r) {
                display::StretchCall call{};
                if (display::stretch_rop_call(setup, r, call)) {
                    plan.push_back(call);
                }
            };
            if (clip.kind == ref::Complexity::trivial) {
                visit(nullptr);
            } else if (clip.kind == ref::Complexity::rect) {
                visit(&bounds);
            } else {
                for (const ref::Rect& r : clip.rects) {
                    const Rect c{r.left, r.top, r.right, r.bottom};
                    visit(&c);
                }
            }
        }

        noop += rop4 == 0xaaaaU ? 1U : 0U;
        negative_extent += drawn && (setup.source_width < 0 || setup.source_height < 0) ? 1U : 0U;
        bool same = plan.size() == calls.size();
        for (size_t i = 0; same && i < plan.size(); ++i) {
            ref::Rect d = calls[i].dest;
            const bool mx = d.right < d.left, my = d.bottom < d.top;
            ref::make_well_ordered(d);
            const display::StretchCall& p = plan[i];
            same = d.left == p.destination.left && d.top == p.destination.top && d.right == p.destination.right &&
                   d.bottom == p.destination.bottom && calls[i].source.left == p.source.left &&
                   calls[i].source.top == p.source.top && calls[i].source.right == p.source.right &&
                   calls[i].source.bottom == p.source.bottom && mx == setup.flip_x && my == setup.flip_y;
            // The per-axis sample formula equals the DIB code at both ends.
            // DIB_XXBPP_StretchBlt recomputes bLeftToRight from both signs.
            const display::StretchAxis ax = display::stretch_axis_x(p, setup.flip_x);
            const int32_t sw_call = calls[i].source.right - calls[i].source.left;
            const int32_t dw_call = d.right - d.left;
            const bool dib_mirror = mx != (sw_call < 0);
            same = same && ax.mirror == dib_mirror;
            for (int32_t px = d.left; same && px < d.right; px += (dw_call > 3 ? dw_call - 1 : 1)) {
                const int32_t dib = dib_mirror ? calls[i].source.right - (px - d.left) * sw_call / dw_call
                                       : calls[i].source.left + (px - d.left) * sw_call / dw_call;
                same = display::stretch_sample(ax, px) == dib;
            }
        }

        if (drawn) {
            flips_seen |= 1U << ((setup.flip_x ? 1 : 0) | (setup.flip_y ? 2 : 0));
        }

        ++cases;
        exact += same ? 1U : 0U;
    }

    std::printf("stretch policy: %u driver calls, %u exact, %u NOOP, %u negative extents (C-truncated, exact)\n",
                cases, exact, noop, negative_extent);
    check(cases > 2000 && exact == cases && flips_seen == 0xfU && noop > 100 && negative_extent > 50,
          "stretch_rop_setup/stretch_rop_call reproduce every EngStretchBltROP DIB call and orientation");

    display::StretchAxis axis{10, 7, 3, 5, true};
    int64_t low = 0, high = 0;
    display::stretch_samples(axis, 10, 17, low, high);
    int64_t first = 0;
    check(low == 3 + 5 - (6 * 5 / 7) && high == 8 && display::clamp_span(low, high, 0, 6, first) == 2 && first == 4 &&
              display::clamp_span(9, 12, 0, 6, first) == 0, "mirrored sample range and domain clamp");
    using display::RasterSource;
    using display::SurfaceKind;
    check(display::stretch_rop_admitted(0xccccU, SurfaceKind::primary, RasterSource::bitmap32, true, true, false, false) &&
              !display::stretch_rop_admitted(0xf0f0U, SurfaceKind::primary, RasterSource::none, true, true, false, false) &&
              display::stretch_rop_admitted(0xc0c0U, SurfaceKind::primary, RasterSource::gpu, true, true, true, true) &&
              !display::stretch_rop_admitted(0xc0c0U, SurfaceKind::primary, RasterSource::gpu, true, true, true, false) &&
              display::stretch_rop_admitted(0xc0c0U, SurfaceKind::primary, RasterSource::gpu, false, false, true, true) &&
              !display::stretch_rop_admitted(0xf0f0U, SurfaceKind::primary, RasterSource::none, false, false, true, false) &&
              !display::stretch_rop_admitted(0xccccU, SurfaceKind::bitmap32, RasterSource::gpu, false, false, false, false) &&
              !display::stretch_rop_admitted(0xccccU, SurfaceKind::primary, RasterSource::unsupported, false, false,
                                             false, false) &&
              !display::stretch_rop_admitted(0xccccU, SurfaceKind::primary, RasterSource::mono, true, false, false, false),
          "stretch admission: masks need a source and a 1bpp mask; a tile with a mask runs its phase");
    // 24bpp byte-plane tables need an OR-linear XLATEOBJ (pinned xlateobj.c).
    using display::or_linear_translation;
    const uint32_t bgr = display::palette_bgr | display::palette_bitfields;   // radeondisp's own palette
    check(or_linear_translation(true, 0, 0, false, false) &&
              or_linear_translation(false, display::palette_bgr, bgr, false, false) &&
              or_linear_translation(false, display::palette_rgb, bgr, false, false) &&
              or_linear_translation(false, display::palette_bitfields, bgr, false, false) &&
              !or_linear_translation(false, display::palette_bitfields, bgr, true, false) &&
              !or_linear_translation(false, display::palette_indexed, bgr, false, false) &&
              !or_linear_translation(false, display::palette_bgr, display::palette_indexed, false, false) &&
              !or_linear_translation(false, display::palette_bgr | display::palette_monochrome, bgr, false, false) &&
              !or_linear_translation(false, display::palette_bgr, bgr, false, true) &&
              !or_linear_translation(false, 0, bgr, false, false) &&
              display::rgb16_masks(0x7c00, 0x3e0, 0x1f) && display::rgb16_masks(0xf800, 0x7e0, 0x1f) &&
              !display::rgb16_masks(0xff0000, 0xff00, 0xff),
          "OR-linear XLATEOBJ selection: Trivial/RGBtoBGR/ShiftAndMask only, never tables, 555/565 or mono");
    {
        Rect zeros[4]{};
        Rect copy{};
        const uint32_t inside = display::snapshot_rects({10, 20, 30, 40}, {10, 20, 30, 40}, zeros, copy);
        const uint32_t corner = display::snapshot_rects({-5, -3, 15, 7}, {0, 0, 15, 7}, zeros, copy);
        const bool corner_ok = corner == 2 && copy.left == 5 && copy.top == 3 && copy.right == 20 && copy.bottom == 10;
        const uint32_t missed = display::snapshot_rects({400, 10, 420, 30}, {}, zeros, copy);
        check(inside == 0 && corner_ok && missed == 1 && zeros[0].right == 20 && zeros[0].bottom == 20 &&
                  rx7900::reactos::framebuffer::empty(copy),
              "snapshot geometry: zero-fill the window off the surface, copy the rest");
    }
    {
        // 1:1 x sampling over [0, 40) against a 32x20 mask: column 32 is a sample.
        display::StretchRopSetup s{};
        s.output = {0, 0, 40, 10};
        s.input = {0, 0, 40, 10};
        display::StretchCall call{{0, 0, 40, 10}, {0, 0, 40, 10}};
        display::StretchSourcePlan plan{};
        plan.operand = display::StretchOperand::staged;
        plan.format = abi::format_xrgb8888;
        plan.domain = plan.backed = {0, 0, 64, 64};
        plan.mask_width = 32;
        plan.mask_height = 20;
        unsigned visited = 0;
        const auto edge = display::stretch_call_bands(s, call, plan, {0, 0, 40, 10}, 1U << 20,
                                                      [&](const display::StretchBand&) { ++visited; return true; });
        plan.mask_width = 48;
        const auto inside = display::stretch_call_bands(s, call, plan, {0, 0, 40, 10}, 1U << 20,
                                                        [&](const display::StretchBand&) { ++visited; return true; });
        plan.mask_width = 32;
        const auto left_of_edge = display::stretch_call_bands(s, call, plan, {0, 0, 32, 10}, 1U << 20,
                                                              [&](const display::StretchBand&) { ++visited; return true; });
        check(edge == display::StretchPlanResult::unsupported && inside == display::StretchPlanResult::done &&
                  left_of_edge == display::StretchPlanResult::done && visited == 2,
              "a mask sample on column cx (a97c8a2 skips it, 822e864 reads past the row) declines the call");
    }
    check(display::staged_strip_columns(true, true, 16384, 512U << 10) == 16384 &&
              display::staged_strip_columns(true, true, 16384, 4096) == 512 &&
              display::staged_rows(512, 512, 4096) == 1 &&
              display::staged_strip_columns(true, false, 3000, 4096) == 1024 &&
              display::staged_strip_columns(false, false, 3000, 0) == 3000 &&
              display::staged_strip_columns(true, true, 3000, 7) == 0,
          "DrvBitBlt column strips: the widest strip whose staged row fits one half");
    const Rect flipped = display::stretch_fallback_destination({1, 2, 9, 7}, true, false);
    check(flipped.left == 9 && flipped.right == 1 && flipped.top == 2 && display::engine_line_colour(0xffffffffU) ==
              0xffffffffU && display::engine_line_colour(0x00123456U) == 0x00123456U,
          "fallback prclDest swaps per flip; EngLineTo plots iSolidColor for pattern pens");
}
} // namespace


// Destructive-reset recovery against the fake backend: ordering, quiescing,
// per-surface uncertainty, exactly-once lost handles, released quarantine,
// accounting and the sticky disable.
void recovery_tests() {
    FakeBackend backend;
    r2d::Dispatcher dispatcher;
    check(!dispatcher.begin_recovery(), "no recovery before configuration");
    check(dispatcher.configure(table(backend), primary(), staging_size, 9), "recovery configuration");
    check(!dispatcher.complete_recovery(table(backend), primary()), "complete without begin refused");
    abi::SessionGrant grant{};
    check(open_primary(dispatcher, grant), "recovery session");
    const uint32_t idle = create_surface(dispatcher, grant.session, 64, 32);
    const uint32_t written = create_surface(dispatcher, grant.session, 64, 32);
    const uint32_t read_only = create_surface(dispatcher, grant.session, 64, 32);
    check(idle != 0 && written != 0 && read_only != 0, "three surfaces");

    // Two system-memory surfaces (refused restore keeps them there) and one in VRAM.
    check(dispatcher.suspend(), "suspend evicts");
    backend.restore_status = r2d::BackendStatus::rejected;
    check(dispatcher.resume(table(backend), primary()), "resume keeps them in system memory");
    backend.restore_status = r2d::BackendStatus::ok;
    const uint32_t vram = create_surface(dispatcher, grant.session, 64, 32);
    check(vram != 0, "a VRAM surface");

    // Pending work at the hang: a write to written, a read of read_only.
    backend.completed = backend.fence;
    check(outcome(dispatcher, abi::ioctl_fill, fill_request(grant.session, written, 1, rects({{0, 0, 4, 4}}))) ==
              abi::outcome_completed &&
          outcome(dispatcher, abi::ioctl_copy, copy_request(grant.session, grant.primary_surface, read_only, 0, 0,
                                                            rects({{0, 0, 4, 4}}))) == abi::outcome_completed,
          "unretired write and read");
    // A surface destroyed with an unprovable sync (quarantined, handle dead).
    const uint32_t doomed = create_surface(dispatcher, grant.session, 64, 32);
    check(outcome(dispatcher, abi::ioctl_fill, fill_request(grant.session, doomed, 1, rects({{0, 0, 4, 4}}))) ==
              abi::outcome_completed, "work on the doomed surface");
    backend.sync_status = r2d::BackendStatus::uncertain;
    check(outcome(dispatcher, abi::ioctl_surface_destroy, surface_destroy_request(grant.session, doomed)) ==
              abi::outcome_lost, "unprovable destroy quarantines and poisons");
    // A second session closed while its work is unprovable (quarantined slot).
    abi::SessionGrant closing{};
    check(open(dispatcher, closing), "second session");
    const uint32_t closing_surface = create_surface(dispatcher, closing.session, 64, 32);
    check(outcome(dispatcher, abi::ioctl_fill, fill_request(closing.session, closing_surface, 1,
                                                            rects({{0, 0, 4, 4}}))) == abi::outcome_completed &&
          dispatcher.dispatch(abi::ioctl_close_session, close_request(closing.session).view(), 0).status ==
              r2d::DispatchStatus::success, "closing session quarantined");
    backend.sync_status = r2d::BackendStatus::ok;

    check(dispatcher.begin_recovery() && dispatcher.recovering() && !dispatcher.begin_recovery(),
          "begin quiesces once");
    const unsigned fills = backend.fills;
    check(dispatcher.dispatch(abi::ioctl_fill, fill_request(grant.session, idle, 1, rects({{0, 0, 1, 1}})).view(),
                              abi::receipt_bytes).status == r2d::DispatchStatus::not_ready &&
              backend.fills == fills && query(dispatcher).flags == 0 &&
              query(dispatcher).recovery_state == abi::recovery_pending,
          "nothing reaches the backend while recovering; query still answers");
    check(!dispatcher.suspend() && !dispatcher.resume(table(backend), primary()),
          "no power transition in the middle of a recovery");

    const unsigned restores = backend.restores;
    const unsigned destroyed = backend.surfaces_destroyed;
    check(dispatcher.complete_recovery(table(backend), primary()) && !dispatcher.recovering() &&
              dispatcher.recovery_generation() == 1, "recovery completes");
    const auto counters = dispatcher.counters();
    check(backend.restores == restores + 2 && counters.reset_restored == 2 && counters.reset_lost == 2 &&
              counters.revived_sessions == 1 && counters.released_quarantine >= 3 &&
              backend.surfaces_destroyed == destroyed + 1,
          "two exact system-memory surfaces restored (a read is not a write), VRAM and the written one lost");
    const abi::Capabilities caps = query(dispatcher);
    check(caps.recovery_generation == 1 && caps.reset_lost_surfaces == 2 && caps.reset_restored_surfaces == 2 &&
              caps.used_surface_bytes == 2U * 256U * 32U && caps.flags != 0,
          "accounting releases lost and quarantined bytes exactly once");
    check(outcome(dispatcher, abi::ioctl_sync, sync_request(grant.session, 0)) == abi::outcome_completed,
          "the poisoned session is revived by the reset proof");
    abi::Receipt lost{};
    check(receipt_of(dispatcher, dispatcher.dispatch(abi::ioctl_copy, copy_request(grant.session, idle, vram, 0, 0,
              rects({{0, 0, 4, 4}})).view(), abi::receipt_bytes), lost) &&
              lost.outcome == abi::outcome_surface_lost && lost.lost_surface == vram,
          "a copy naming a lost source reports that surface");
    check(dispatcher.dispatch(abi::ioctl_fill, fill_request(grant.session, vram, 1, rects({{0, 0, 1, 1}})).view(),
                              abi::receipt_bytes).status == r2d::DispatchStatus::invalid_parameter,
          "exactly once");
    check(receipt_of(dispatcher, dispatcher.dispatch(abi::ioctl_surface_destroy,
              surface_destroy_request(grant.session, written).view(), abi::receipt_bytes), lost) &&
              lost.outcome == abi::outcome_surface_lost && lost.lost_surface == written &&
              dispatcher.counters().lost_reported == 2,
          "destroy of an unreported lost surface is its one report");
    check(outcome(dispatcher, abi::ioctl_fill, fill_request(grant.session, idle, 1, rects({{0, 0, 1, 1}}))) ==
              abi::outcome_completed &&
          outcome(dispatcher, abi::ioctl_fill, fill_request(grant.session, read_only, 1, rects({{0, 0, 1, 1}}))) ==
              abi::outcome_completed, "surviving handles keep working");
    uint32_t reused = 0;
    for (int index = 0; index < 4 && reused == 0; ++index) {
        const uint32_t handle = create_surface(dispatcher, grant.session, 64, 32);
        reused = (handle & 0xffU) == (vram & 0xffU) || (handle & 0xffU) == (written & 0xffU) ? handle : 0;
    }
    check(reused != 0 && reused != vram && reused != written, "a reused slot gets a new generation");
    abi::SessionGrant third{};
    abi::SessionGrant fourth{};
    abi::SessionGrant fifth{};
    check(open(dispatcher, third) && open(dispatcher, fourth) && open(dispatcher, fifth),
          "the quarantined session slot is reusable after the reset proof");

    // A restore that is uncertain is a second fault; disable is sticky.
    check(dispatcher.suspend(), "suspend again");
    backend.restore_status = r2d::BackendStatus::uncertain;
    check(dispatcher.begin_recovery() && !dispatcher.complete_recovery(table(backend), primary()),
          "an uncertain restore fails the recovery");
    dispatcher.disable(false);
    dispatcher.disable(true);
    const unsigned calls = backend.syncs + backend.fills + backend.staging_destroyed;
    check(dispatcher.disabled() && query(dispatcher).recovery_state == abi::recovery_disabled_unproven &&
              query(dispatcher).flags == 0 &&
              dispatcher.dispatch(abi::ioctl_open_session, open_request().view(),
                                  abi::session_grant_bytes).status == r2d::DispatchStatus::not_ready &&
              !dispatcher.shutdown() && !dispatcher.begin_recovery() &&
              backend.syncs + backend.fills + backend.staging_destroyed == calls,
          "disabled: unproven never upgrades, nothing is dispatched or freed");
}

int main() {
    codec_tests();
    configuration_tests();
    session_tests();
    fill_tests();
    copy_tests();
    transfer_tests();
    display_policy_tests();
    budget_tests();
    suspend_tests();
    recovery_tests();
    batching_tests();
    bitmap_policy_tests();
    raster_codec_tests();
    raster_dispatch_tests();
    raster_policy_tests();
    stretch_codec_tests();
    stretch_dispatch_tests();
    stretch_policy_tests();
    std::printf("Radeon 2D IOCTL: %u checks, %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

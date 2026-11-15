/*
Module: rx7900.tests.process_graphics
Purpose: Exercise the version-3 kernel-built graphics boundary: forged and foreign
handles, insufficient mappings and footprints, out-of-range counts and
enumerants, non-finite floats, invalid shader keys, oversize requests, private
VA protection, lease retention through real fences and the present path.
Created: 2026-10-23
Architecture: Production ProcessGraphics/ProcessMemory/UserQueues/decoders over
the synthetic MES fixture (gfx11_graphics_device). Every kernel-built IB is also
decoded against the pinned GC 11.0.0 headers and the allow-list.
*/
#include <cmath>
#include <cstdio>
#include <functional>
#include <string>

#include "asic_test_device.hpp"
#include "gfx11_graphics_device.hpp"
#include "../src/opengl/gl_context.hpp"

namespace {

using gfx11_graphics_device::Device;
namespace wire = rx7900::reactos::graphics_abi;
namespace pabi = rx7900::reactos::process_abi;
namespace abi = rx7900::opengl::gfx11_abi;
using rx7900::memory::Access;
using rx7900::memory::Status;
using rx7900::reactos::ProcessMemoryDomain;

rx7900_test::Checks checks;
void expect(bool value, const std::string& what) { checks.expect(value, what.c_str()); }

gfx11_header_db::Database& database() {
    static gfx11_header_db::Database db;
    static bool loaded = db.load();
    (void)loaded;
    return db;
}

uint32_t float_bits(float f) { uint32_t b; std::memcpy(&b, &f, 4); return b; }

struct Session final {
    Device device{database(), 64, 64};
    uint64_t color = 0, depth = 0, texture = 0;
    uint32_t width = 32, height = 32;
    bool kernel = false;

    // kernel: the graphics context runs on the kernel gfx ring (handle 0).
    explicit Session(bool kernel_ring = false) : kernel(kernel_ring) {
        uint8_t reply[64];
        uint32_t reply_bytes = 0;
        device.kernel_ring_enabled = kernel_ring;
        // open, queue, graphics_open through the production request decoder.
        call(pabi::Operation::open, 0, 0, 0, 0, 0, 0, reply, reply_bytes);
        if (!kernel_ring) call(pabi::Operation::queue_create, 0, 0, 0, 0, 0, 0, reply, reply_bytes);
        call(pabi::Operation::graphics_open, kernel_ring ? 0 : device.queue_handle, 0, 0, 0, 0, 0, reply,
             reply_bytes);
        color = buffer(rx7900::opengl::gfx11::linear_pitch_pixels(width, rx7900::opengl::gfx11::TargetKind::bgra8) *
                       4u * height, 0x10000000ULL, Access::read_write);
        rx7900::opengl::gfx11::DepthLayout layout{};
        rx7900::opengl::gfx11::depth_layout(width, height, layout);
        depth = buffer(layout.total_bytes, 0x20000000ULL, Access::read_write);
        texture = buffer(64 * 64 * 4, 0x30000000ULL, Access::read);
    }

    uint32_t call(pabi::Operation op, uint64_t handle, uint64_t bytes, uint64_t offset, uint64_t address,
                  uint32_t domain, uint32_t access, uint8_t* reply, uint32_t& reply_bytes) {
        uint8_t request[64];
        wire::Writer w(request, 64, 0);
        w.u32(64); w.u32(pabi::graphics_version); w.u32(static_cast<uint32_t>(op)); w.u32(0);
        w.u64(op == pabi::Operation::open ? 0 : device.session); w.u64(handle); w.u64(bytes); w.u64(offset);
        w.u64(address); w.u32(domain); w.u32(access);
        device.call(request, 64, reply, 64, reply_bytes);
        uint32_t outcome = 0;
        rx7900::ByteView(reply, reply_bytes).read_u32(8, outcome);
        return outcome;
    }

    uint64_t buffer(uint64_t bytes, uint64_t va, Access access) {
        uint64_t handle = 0;
        const uint64_t size = (bytes + 4095) & ~4095ULL;
        if (device.buffers->allocate(size, ProcessMemoryDomain::system, Access::read_write, handle) != Status::success ||
            device.buffers->map(handle, va, access) != Status::success) return 0;
        return handle;
    }

    wire::GraphicsState state(bool textured = false) const {
        wire::GraphicsState s{};
        s.viewport_width = width;
        s.viewport_height = height;
        s.depth_far = float_bits(1.0f);
        s.point_size = float_bits(1.0f);
        s.line_width = float_bits(1.0f);
        s.depth_func = 1;
        s.stencil_func = 7;
        s.stencil_value_mask = 0xff;
        s.stencil_write_mask = 0xff;
        s.blend_src = 1;
        s.logic_op_code = 3;
        s.color_mask = 15;
        s.key = abi::ShaderKey{};
        if (textured) {
            s.texture = texture;
            s.texture_width = 16;
            s.texture_height = 16;
            s.texture_pitch = 64;
            s.key.textured = 1;
        }
        return s;
    }

    // A complete submit payload; mutate lets a case corrupt one field.
    std::vector<uint8_t> payload(const std::function<void(wire::SubmitHeader&, wire::GraphicsState&,
                                                          wire::GraphicsCommand&)>& mutate = nullptr,
                                 bool textured = false, uint32_t vertices = 3) const {
        wire::SubmitHeader h{wire::submit_header_bytes, 1, 1, vertices * 80, color, depth, width, height, 0, 0};
        wire::GraphicsState s = state(textured);
        wire::GraphicsCommand c{0, 3, 0, vertices, 0, 0};
        if (mutate) mutate(h, s, c);
        std::vector<uint8_t> out(wire::submit_header_bytes + wire::state_bytes + wire::command_bytes + h.vertex_bytes);
        wire::encode_header(h, out.data(), out.size());
        wire::encode_state(s, out.data() + wire::submit_header_bytes, wire::state_bytes);
        wire::encode_command(c, out.data() + wire::submit_header_bytes + wire::state_bytes, wire::command_bytes);
        uint8_t* v = out.data() + wire::submit_header_bytes + wire::state_bytes + wire::command_bytes;
        const float clip[3][4] = {{-1, -1, 0, 1}, {1, -1, 0, 1}, {0, 1, 0, 1}};
        for (uint32_t i = 0; i < vertices && (i + 1) * 80 <= h.vertex_bytes; ++i) {
            for (int k = 0; k < 4; ++k) {
                const uint32_t bits = float_bits(clip[i % 3][k]);
                std::memcpy(v + i * 80 + k * 4, &bits, 4);
            }
        }
        return out;
    }

    Status submit(const std::vector<uint8_t>& bytes, uint32_t& sequence) {
        return device.graphics.submit(kernel ? 0 : device.queue.id, rx7900::ByteView(bytes.data(), bytes.size()),
                                      sequence);
    }

    uint32_t leases(uint64_t handle) {
        rx7900::reactos::ProcessBufferInfo info{};
        device.buffers->describe(handle, info);
        return info.leases;
    }
};

void baseline() {
    Session s;
    expect(s.device.ok && s.device.graphics.opened() && s.color != 0 && s.depth != 0,
           "session, gfx queue and graphics context open through the decoder");
    uint32_t sequence = 0;
    const auto bytes = s.payload();
    expect(s.submit(bytes, sequence) == Status::success && sequence != 0, "a valid typed draw submits");
    expect(s.device.graphics.query(s.device.queue.id, sequence) == Status::success, "its real fence retires");
    expect(s.device.ibs == 0 || s.device.model->stats().errors == 0, "the IB executes");
}

void forged_handles() {
    Session s, other;
    uint32_t seq = 0;
    expect(s.submit(s.payload([](auto& h, auto&, auto&) { h.color = 0x7777; }), seq) == Status::stale_handle,
           "forged colour handle refused");
    expect(s.submit(s.payload([&](auto& h, auto&, auto&) { h.color = other.color + 1000; }), seq) ==
           Status::stale_handle, "out-of-range handle refused");
    // A handle value valid in another session names nothing here unless this
    // session owns the same number; use a released handle to be exact.
    uint64_t released = 0;
    s.device.buffers->allocate(4096, ProcessMemoryDomain::system, Access::read_write, released);
    s.device.buffers->release(released);
    expect(s.submit(s.payload([&](auto& h, auto&, auto&) { h.depth = released; }), seq) == Status::stale_handle,
           "released depth handle refused");
    expect(s.submit(s.payload([&](auto&, auto& st, auto&) { st.texture = released; }, true), seq) ==
           Status::stale_handle, "released texture handle refused");
    // Handles are per-session: other's colour handle resolves only in other.
    rx7900::reactos::ProcessBufferInfo info{};
    expect(other.device.buffers->describe(other.color, info) == Status::success &&
           s.device.buffers.get() != other.device.buffers.get(),
           "foreign session buffers live in a separate owner (no cross-session resolution)");
}

void mappings() {
    Session s;
    uint32_t seq = 0;
    const uint64_t unmapped = [&]() {
        uint64_t h = 0;
        s.device.buffers->allocate(65536, ProcessMemoryDomain::system, Access::read_write, h);
        return h;
    }();
    expect(s.submit(s.payload([&](auto& h, auto&, auto&) { h.color = unmapped; }), seq) == Status::access_denied,
           "unmapped colour buffer refused");
    const uint64_t read_only = s.buffer(65536, 0x40000000ULL, Access::read);
    expect(s.submit(s.payload([&](auto& h, auto&, auto&) { h.color = read_only; }), seq) == Status::access_denied,
           "read-only mapping cannot be a render target");
    const uint64_t small = s.buffer(4096, 0x50000000ULL, Access::read_write);
    expect(s.submit(s.payload([&](auto& h, auto&, auto&) { h.color = small; }), seq) == Status::invalid_argument,
           "colour buffer smaller than pitch*height refused");
    const uint64_t misaligned_depth = s.buffer(256 * 1024, 0x60001000ULL, Access::read_write);
    expect(s.submit(s.payload([&](auto& h, auto&, auto&) { h.depth = misaligned_depth; }), seq) ==
           Status::invalid_argument, "depth buffer not 64 KiB aligned refused");
    expect(s.submit(s.payload([&](auto& h, auto&, auto&) { h.depth = h.color; }), seq) != Status::success,
           "colour and depth cannot alias");
    const uint64_t write_texture = s.buffer(65536, 0x70000000ULL, Access::write);
    expect(s.submit(s.payload([&](auto&, auto& st, auto&) { st.texture = write_texture; }, true), seq) ==
           Status::access_denied, "texture needs a readable mapping");
    expect(s.submit(s.payload([&](auto&, auto& st, auto&) { st.texture_height = 4096; st.texture_pitch = 64; }, true),
                    seq) == Status::invalid_argument, "texture footprint beyond its buffer refused");
    expect(s.submit(s.payload([&](auto&, auto& st, auto&) { st.texture_offset = 128; }, true), seq) ==
           Status::invalid_argument, "texture offset must keep 256-byte alignment");
    // Private VAs are not available to user mappings.
    for (uint64_t va : {rx7900::reactos::graphics_job_va, rx7900::reactos::graphics_shader_va,
                        rx7900::reactos::graphics_attribute_va}) {
        uint64_t h = 0;
        s.device.buffers->allocate(4096, ProcessMemoryDomain::system, Access::read_write, h);
        expect(s.device.buffers->map(h, va, Access::read_write) != Status::success,
               "user buffers cannot overlap the kernel IB/shader/attribute-ring mappings");
    }
}

void counts_and_enums() {
    Session s;
    uint32_t seq = 0;
    // cross: only the kernel's cross-field checks (state vs command, target vs
    // flags) refuse it; every other case must already fail a wire decoder.
    struct Case { const char* what; std::function<void(wire::SubmitHeader&, wire::GraphicsState&, wire::GraphicsCommand&)> m; bool cross = false; };
    const Case cases[] = {
        {"zero states", [](auto& h, auto&, auto&) { h.state_count = 0; }},
        {"17 states", [](auto& h, auto&, auto&) { h.state_count = 17; }},
        {"65 commands", [](auto& h, auto&, auto&) { h.command_count = 65; }},
        {"vertex bytes not 16-aligned", [](auto& h, auto&, auto&) { h.vertex_bytes = 3 * 80 + 4; }},
        {"vertex bytes over the maximum", [](auto& h, auto&, auto&) { h.vertex_bytes = wire::max_vertex_bytes + 16; }},
        {"width over 4096", [](auto& h, auto&, auto&) { h.width = 4097; }},
        {"zero height", [](auto& h, auto&, auto&) { h.height = 0; }},
        {"unknown target kind", [](auto& h, auto&, auto&) { h.target_kind = 2; }},
        {"reserved header field", [](auto& h, auto&, auto&) { h.reserved = 1; }},
        {"draw beyond its vertices", [](auto&, auto&, auto& c) { c.first_vertex = 1; }},
        {"count not a primitive multiple", [](auto& h, auto&, auto& c) { h.vertex_bytes = 6 * 80; c.vertex_count = 4; }},
        {"count below one primitive", [](auto&, auto&, auto& c) { c.vertex_count = 2; }},
        {"primitive kind 4", [](auto&, auto&, auto& c) { c.primitive = 4; }},
        {"primitive differs from the key", [](auto&, auto&, auto& c) { c.primitive = 1; c.vertex_count = 3; }, true},
        {"state index out of range", [](auto&, auto&, auto& c) { c.state_index = 1; }},
        {"depth copy flag on a colour target", [](auto&, auto&, auto& c) { c.flags = wire::command_depth_copy; }, true},
        {"unknown command flag", [](auto&, auto&, auto& c) { c.flags = 4; }},
        {"depth func 8", [](auto&, auto& st, auto&) { st.depth_func = 8; }},
        {"stencil op 6", [](auto&, auto& st, auto&) { st.stencil_fail = 6; }},
        {"stencil ref 256", [](auto&, auto& st, auto&) { st.stencil_ref = 256; }},
        {"blend factor 11", [](auto&, auto& st, auto&) { st.blend_src = 11; }},
        {"logic op 16", [](auto&, auto& st, auto&) { st.logic_op_code = 16; }},
        {"colour mask 16", [](auto&, auto& st, auto&) { st.color_mask = 16; }},
        {"cull mode 4", [](auto&, auto& st, auto&) { st.cull = 4; }},
        {"boolean field 2", [](auto&, auto& st, auto&) { st.scissor = 2; }},
        {"NaN point size", [](auto&, auto& st, auto&) { st.point_size = 0x7fc00000u; }},
        {"zero line width", [](auto&, auto& st, auto&) { st.line_width = 0; }},
        {"infinite offset units", [](auto&, auto& st, auto&) { st.offset_units = 0x7f800000u; }},
        {"depth range above 1", [](auto&, auto& st, auto&) { st.depth_far = float_bits(1.5f); }},
        {"negative alpha reference", [](auto&, auto& st, auto&) { st.alpha_ref = float_bits(-0.25f); }},
        {"viewport width over 8192", [](auto&, auto& st, auto&) { st.viewport_width = 8193; }},
        {"object-space key", [](auto&, auto& st, auto&) { st.key.vertex_input = abi::vertex_input_object_space; }},
        {"invalid key flag", [](auto&, auto& st, auto&) { st.key.fog = 2; }},
        {"nonzero key reserved byte", [](auto&, auto& st, auto&) { st.key.reserved[1] = 1; }},
        {"textured key without a texture", [](auto&, auto& st, auto&) { st.key.textured = 1; }},
        {"texture pitch below width", [](auto&, auto& st, auto&) {
             st.texture = 1; st.key.textured = 1; st.texture_width = 65; st.texture_height = 1; st.texture_pitch = 64; }},
        {"texture fields without a texture", [](auto&, auto& st, auto&) { st.wrap_s = 1; }},
    };
    for (const auto& c : cases) {
        const auto bytes = s.payload(c.m);
        expect(s.submit(bytes, seq) == Status::invalid_argument && seq == 0,
               std::string("rejected before any GPU work: ") + c.what);
        // The first line of defence: the wire decoders alone.
        const rx7900::ByteView view(bytes.data(), bytes.size());
        wire::SubmitHeader h{};
        wire::GraphicsState st{};
        wire::GraphicsCommand cmd{};
        const bool decoded = wire::decode_header(view, h) && view.size() == wire::submit_payload_bytes(h) &&
            wire::decode_state(view, wire::submit_header_bytes, st) &&
            wire::decode_command(view, wire::submit_header_bytes + wire::state_bytes * h.state_count, h.state_count,
                                 h.vertex_bytes, cmd);
        expect(decoded == c.cross, std::string(c.cross ? "cross-field case passes the decoders: " :
                                                         "wire decoder alone refuses: ") + c.what);
    }
    auto bytes = s.payload();
    bytes.push_back(0);
    expect(s.submit(bytes, seq) == Status::invalid_argument, "trailing payload bytes refused");
    bytes = s.payload();
    bytes.pop_back();
    expect(s.submit(bytes, seq) == Status::invalid_argument, "truncated payload refused");
    expect(s.submit(std::vector<uint8_t>(16, 0), seq) == Status::invalid_argument, "short header refused");
    expect(s.device.submits == 0 && s.device.fx.queues->queue_report(s.device.queue.id).submissions == 0,
           "no refused request reached the gfx queue");
}

void request_boundary() {
    pabi::Request r{};
    r.size = 64;
    r.abi_version = pabi::graphics_version;
    r.operation = static_cast<uint32_t>(pabi::Operation::graphics_submit);
    r.session = 1;
    r.handle = 2;
    r.bytes = pabi::bulk_bytes + 1;
    expect(!pabi::valid_request_fields(r), "oversize graphics payload refused at the request decoder");
    r.bytes = 128;
    r.address = 0x1000;
    expect(!pabi::valid_request_fields(r), "no version-3 request carries an address");
    r.address = 0;
    r.domain = 1;
    expect(!pabi::valid_request_fields(r), "no version-3 request carries a domain");
    r.domain = 0;
    expect(pabi::valid_request_fields(r), "well-formed graphics_submit header accepted");
    r.abi_version = pabi::version;
    uint8_t bytes[64];
    wire::Writer w(bytes, 64, 0);
    w.u32(64); w.u32(pabi::version); w.u32(16); w.u32(0); w.u64(1); w.u64(2); w.u64(128); w.u64(0); w.u64(0);
    w.u32(0); w.u32(0);
    pabi::Request decoded{};
    expect(!pabi::decode_request(rx7900::ByteView(bytes, 64), decoded), "version-2 requests cannot reach graphics ops");
    r.abi_version = pabi::graphics_version;
    r.operation = static_cast<uint32_t>(pabi::Operation::buffer_readback);
    r.bytes = pabi::bulk_bytes;
    expect(pabi::valid_request_fields(r), "bulk readback up to 256 KiB accepted");
    r.bytes = 0;
    expect(!pabi::valid_request_fields(r), "empty bulk transfer refused");
    r.operation = static_cast<uint32_t>(pabi::Operation::graphics_query);
    r.offset = 0;
    expect(!pabi::valid_request_fields(r), "graphics_query needs a sequence");
    r.operation = 20;
    w = wire::Writer(bytes, 64, 0);
    w.u32(64); w.u32(pabi::graphics_version); w.u32(20); w.u32(0);
    expect(!pabi::decode_request(rx7900::ByteView(bytes, 64), decoded), "unknown operation refused");
}

void allow_list_and_leases() {
    Session s;
    // Delay the gfx queue so the submission stays pending.
    const auto report = s.device.fx.queues->queue_report(s.device.queue.id);
    s.device.fx.engines.hang_doorbell = report.doorbell_index;
    uint32_t sequence = 0;
    expect(s.submit(s.payload(nullptr, true), sequence) == Status::success, "textured draw submits");
    // Decode the kernel-built IB itself from the session VM.
    const uint64_t root = [&]() {
        const auto* frame = s.device.fx.engines.active(report.doorbell_index);
        return frame == nullptr ? 0 : (static_cast<uint64_t>(frame->at(3)) << 32) | frame->at(2);
    }();
    unsigned registers = 0, refused = 0;
    const auto& db = database();
    for (uint64_t at = rx7900::reactos::graphics_job_va;; at += 4) {
        const auto read = [&](uint64_t va) {
            uint32_t v = 0;
            for (int i = 0; i < 4; ++i) {
                const uint8_t* p = s.device.fx.engines.translate(root, va + i, false);
                v |= static_cast<uint32_t>(p != nullptr ? *p : 0) << (8 * i);
            }
            return v;
        };
        const uint32_t header = read(at);
        if ((header >> 30) != 3) break;
        const uint32_t count = ((header >> 16) & 0x3fff) + 1;
        const uint32_t opcode = (header >> 8) & 0xff;
        if (opcode == 0x69 || opcode == 0x76 || opcode == 0x79) {
            const uint32_t base = opcode == 0x69 ? 0xa000 : opcode == 0x76 ? 0x2c00 : 0xc000;
            const uint32_t absolute = base + read(at + 4);
            const uint32_t value = read(at + 8);
            ++registers;
            bool allowed = false;
            for (uint32_t i = 0; i < rx7900::opengl::gfx11_reg::allowed_register_count; ++i)
                allowed = allowed || rx7900::opengl::gfx11_reg::absolute_dword(rx7900::opengl::gfx11_reg::allowed_registers[i]) == absolute;
            const auto* reg = db.at(absolute);
            if (!allowed || reg == nullptr || (value & ~reg->mask_union()) != 0) ++refused;
        }
        if (opcode == 0x46 && read(at + 4) == 46) {}   // FLUSH_AND_INV_CB_META
        at += count * 4;
    }
    expect(registers > 150 && refused == 0,
           "every register in the kernel-built IB (read back through the process page tables) is allow-listed and in-mask");
    rx7900::reactos::ProcessBufferInfo info{};
    s.device.buffers->describe(s.color, info);
    uint8_t byte = 0;
    expect(info.leases == 1 && s.device.buffers->read(s.color, 0, &byte, 1) == Status::busy &&
           s.device.buffers->unmap(s.color) == Status::busy && s.device.buffers->release(s.texture) == Status::busy,
           "colour and texture stay leased (no CPU access, unmap or release) while the fence is pending");
    expect(s.device.graphics.query(s.device.queue.id, sequence + 7) == Status::invalid_argument,
           "an invented future fence is not a receipt");
    expect(s.device.graphics.close_after_idle() == Status::busy, "close refuses while the VM can still run the job");
    wire::PresentRequest present{};
    present.input = {wire::present_input_bytes, wire::version, s.device.session, s.color, s.width, s.height};
    present.rect_count = 1;
    present.rects[0] = {0, 0, 8, 8};
    uint32_t copied = 0;
    expect(s.device.graphics.present(present, copied) == Status::busy && copied == 0,
           "present refuses a buffer whose rendering has not retired");
    s.device.fx.engines.hang_doorbell = UINT32_MAX;
    s.device.fx.engines.execute(report.doorbell_index, s.device.fx.engines.pending.at(report.doorbell_index));
    s.device.fx.queues->poll();
    expect(s.device.graphics.query(s.device.queue.id, sequence) == Status::success, "actual fence completion");
    s.device.buffers->describe(s.color, info);
    expect(info.leases == 0 && s.device.buffers->read(s.color, 0, &byte, 1) == Status::success,
           "completion returns every lease");
}

void present_bounds() {
    Session s;
    wire::PresentRequest present{};
    present.input = {wire::present_input_bytes, wire::version, s.device.session, s.color, s.width, s.height};
    present.window_left = 50;
    present.window_top = 40;
    present.rect_count = 2;
    present.rects[0] = {-100, -100, 1000, 1000};   // clipped to window and scanout
    present.rects[1] = {0, 0, 10, 10};             // outside the window entirely
    uint32_t copied = 0;
    expect(s.device.graphics.present(present, copied) == Status::success && copied == 1,
           "present rectangles are intersected with the drawable and the scanout");
    s.device.run_new_ibs_public();
    const auto& stats = s.device.model->stats();
    expect(stats.dma_rows == 24 && stats.errors == 0,
           "only the 14x24 visible part (64x64 scanout from (50,40)) is copied, row by row");
    unsigned changed = 0, outside = 0;
    for (uint32_t y = 0; y < 64; ++y)
        for (uint32_t x = 0; x < 64; ++x) {
            const bool inside = x >= 50 && y >= 40;
            if (s.device.scanout[y * 64 + x] != 0xff102030u) (inside ? changed : outside) += 1;
        }
    expect(outside == 0, "no scanout pixel outside the clipped rectangle changes");
    present.input.buffer = s.texture;   // read-only mapping is enough to read
    present.input.width = 4096;
    expect(s.device.graphics.present(present, copied) == Status::invalid_argument,
           "present refuses a buffer smaller than the claimed drawable");
    present.input.buffer = 0x4242;
    present.input.width = s.width;
    expect(s.device.graphics.present(present, copied) == Status::stale_handle, "present refuses a forged handle");
    uint8_t encoded[wire::present_request_bytes];
    present.input.buffer = s.color;
    present.rects[0] = {5, 5, 5, 9};
    wire::encode_present_request(present, encoded, sizeof(encoded));
    wire::PresentRequest decoded{};
    expect(!wire::decode_present_request(rx7900::ByteView(encoded, sizeof(encoded)), decoded),
           "empty rectangles are malformed");
    present.rects[0] = {0, 0, 1, 1};
    present.rect_count = wire::max_present_rects + 1;
    wire::encode_present_request(present, encoded, sizeof(encoded));
    expect(!wire::decode_present_request(rx7900::ByteView(encoded, sizeof(encoded)), decoded),
           "more than 64 rectangles per request refused");
}

// The kernel gfx-ring route: the same kernel-built IB as a process-VM job.
void kernel_ring_route() {
    Session s(true);
    auto& ring = s.device.kernel;
    expect(s.device.graphics.opened() && s.device.graphics.kernel_route() && s.device.queue_handle == 0,
           "graphics_open with handle 0 binds the context to the kernel gfx ring (no MES queue)");
    uint32_t sequence = 0;
    expect(s.submit(s.payload(nullptr, true), sequence) == Status::success && sequence == 1,
           "a textured typed draw submits; sequences are session-local");
    const auto vm = s.device.fx.queues->vm_report(s.device.process.pasid);
    expect(ring.jobs.size() == 1 && ring.vmid0_submissions == 0 && ring.jobs[0].pasid == s.device.process.pasid &&
           ring.jobs[0].pd_address == vm.pd_address && ring.jobs[0].ib_va == rx7900::reactos::graphics_job_va,
           "the job names this session's PASID, page directory and the IB's private VA (never VMID 0)");
    expect(s.device.kernel_ibs == 1 && s.device.vm_mismatches == 0 && s.device.ib_faults == 0 &&
           s.device.model->stats().errors == 0 && s.device.model->stats().draws > 0,
           "the ring fetched the IB through the session page tables and the draw executed");
    expect(s.device.graphics.query(0, sequence) == Status::success && ring.ib_leases.empty() &&
           s.leases(s.color) == 0 && s.leases(s.texture) == 0,
           "the actual fence receipt returns the ring's IB lease and every buffer lease");

    // A pending fence keeps everything leased; nothing else is a receipt.
    ring.autosignal = false;
    expect(s.submit(s.payload(), sequence) == Status::success && sequence == 2, "second draw pending");
    uint8_t byte = 0;
    expect(s.leases(s.color) == 1 && s.device.buffers->read(s.color, 0, &byte, 1) == Status::busy &&
           s.device.buffers->unmap(s.color) == Status::busy && ring.ib_leases.size() == 1 &&
           s.device.graphics.kernel_pending(),
           "colour stays leased (no CPU access or unmap) and the ring holds the IB while the fence is pending");
    expect(s.device.graphics.query(0, sequence) == Status::busy &&
           s.device.graphics.query(0, sequence + 7) == Status::invalid_argument &&
           s.device.graphics.query(0, 1) == Status::success && s.device.graphics.query(5, 1) == Status::invalid_argument,
           "pending is busy, an invented future number is refused, a retired one answers success");
    expect(s.device.graphics.close_after_idle() == Status::busy,
           "close refuses while the kernel ring may still read the VM");
    // A second pending draw takes the other job slot: its own IB VA.
    expect(s.submit(s.payload(), sequence) == Status::success && sequence == 3 && ring.jobs.size() == 3 &&
           ring.jobs[2].ib_va == rx7900::reactos::graphics_job_va + rx7900::reactos::graphics_job_bytes &&
           s.submit(s.payload(), sequence) == Status::busy,
           "two jobs in flight use the two job slots (distinct IB VAs); a third waits");
    // No MES queue proves anything about the kernel ring: even with the VM
    // gone the context keeps every lease until the ring's receipt.
    expect(s.device.fx.queues->destroy_process(s.device.process.pasid) == rx7900::hardware::UserQueueStatus::destroyed &&
           s.device.graphics.close_after_idle() == Status::busy && s.leases(s.color) == 2,
           "a destroyed VM is no kernel-ring receipt");
    expect(s.device.graphics.wait_kernel_idle(100) == Status::busy, "a bounded drain times out on a pending job");
    ring.signal();
    expect(s.device.graphics.wait_kernel_idle(100) == Status::success && s.leases(s.color) == 0 &&
           ring.ib_leases.empty(), "the receipt drains both slots");
    return;

}

void kernel_ring_failures() {
    Session s(true);
    auto& ring = s.device.kernel;
    uint32_t sequence = 0;
    expect(s.submit(s.payload(), sequence) == Status::success && sequence == 1, "a first draw retires");
    // Refusals before publication return every lease; an uncertain doorbell keeps them.
    ring.autosignal = true;
    ring.answer = rx7900::hardware::Gfx11QueuesStatus::ring_full;
    expect(s.submit(s.payload(), sequence) == Status::busy && s.leases(s.color) == 0, "ring full: retry, no lease");
    ring.answer = rx7900::hardware::Gfx11QueuesStatus::unavailable;
    expect(s.submit(s.payload(), sequence) == Status::unavailable && s.leases(s.color) == 0 &&
           !s.device.graphics.kernel_pending(), "submission disabled: unavailable, no lease");
    ring.answer = rx7900::hardware::Gfx11QueuesStatus::ring_hung;
    expect(s.submit(s.payload(), sequence) == Status::unavailable && s.leases(s.color) == 0,
           "an already hung ring refuses before writing");
    ring.answer = rx7900::hardware::Gfx11QueuesStatus::doorbell_failure;
    expect(s.submit(s.payload(), sequence) == Status::quarantined && s.leases(s.color) == 1 &&
           s.device.graphics.kernel_quarantined(),
           "a failed doorbell may still run the job: quarantined with every lease kept");
    ring.answer = rx7900::hardware::Gfx11QueuesStatus::running;
    expect(s.submit(s.payload(), sequence) == Status::quarantined && s.device.graphics.poll() == Status::quarantined,
           "a quarantined context submits nothing more");
    uint32_t queue = 9;
    uint32_t lost[rx7900::reactos::graphics_jobs]{};
    expect(s.device.graphics.retire_after_reset(queue, lost) == Status::quarantined && s.leases(s.color) == 1,
           "without the reset proof nothing is released");
    ring.reset_confirmed = true;
    expect(s.device.graphics.retire_after_reset(queue, lost) == Status::success && queue == 0 &&
           (lost[0] == 2 || lost[1] == 2) && s.leases(s.color) == 0,
           "the confirmed reset window retires the uncertain job, reports it lost and returns its leases");
}

void kernel_ring_refusals() {
    // No kernel ring: graphics_open with queue 0 is unavailable (MES route only).
    Session mes;
    rx7900::reactos::ProcessGraphics other;
    auto config = mes.device.graphics_config();
    expect(!mes.device.graphics.kernel_route() && other.open(config, 0) == Status::unavailable,
           "without a kernel-ring submitter queue 0 is refused");
    // The MES route never reaches the kernel ring.
    Session both;
    both.device.kernel_ring_enabled = true;
    uint32_t sequence = 0;
    expect(both.submit(both.payload(), sequence) == Status::success && both.device.kernel.jobs.empty(),
           "a context bound to a MES gfx queue keeps using it");
}

// End to end through the ICD backend: without a MES gfx queue the backend's
// typed draws run on the kernel gfx ring, pixel-identical to gl_raster.
void kernel_ring_backend() {
    using namespace rx7900::opengl;
    constexpr uint32_t size = 32;
    std::vector<uint8_t> reference_storage(raster_storage_bytes(size, size, true));
    SoftwareBackend reference{reference_storage.data(), reference_storage.size(), FramebufferFormat{true, 24, 8}};
    reference.resize(size, size);
    Context reference_context{&reference, default_allocator()};
    Device device{database(), 64, 64};
    device.kernel_ring_enabled = true;
    device.mes_gfx_unavailable = true;
    std::vector<uint8_t> storage(raster_storage_bytes(size, size, true));
    SoftwareBackend fallback{storage.data(), storage.size(), FramebufferFormat{true, 24, 8}};
    std::vector<uint8_t> scratch(gfx11_backend_scratch_bytes);
    Gfx11Backend backend(device, scratch.data(), scratch.size(), fallback);
    expect(backend.initialize() && backend.gpu_active(),
           "ICD backend falls back to the kernel gfx ring when no MES gfx queue can be created");
    backend.resize(size, size);
    Context context{&backend, default_allocator()};
    for (Context* c : {&reference_context, &context}) {
        c->viewport(0, 0, size, size);
        c->clear_color(0, 0, 0.25f, 1);
        c->clear(gl::color_buffer_bit);
        c->begin(gl::triangles);
        c->color(1, 0, 0, 1);
        c->vertex(-0.8f, -0.8f, 0, 1);
        c->color(0, 1, 0, 1);
        c->vertex(0.8f, -0.6f, 0, 1);
        c->color(0, 0, 1, 1);
        c->vertex(0.1f, 0.9f, 0, 1);
        c->end();
        c->finish();
    }
    std::vector<float> a(size * size * 4), b(size * size * 4);
    reference.read(ReadSource::color, gl::back, 0, 0, size, size, a.data());
    backend.read(ReadSource::color, gl::back, 0, 0, size, size, b.data());
    unsigned bad = 0, covered = 0;
    for (size_t p = 0; p < a.size() / 4; ++p) {
        bool differs = false;
        for (int c = 0; c < 4; ++c) differs = differs || std::fabs(a[p * 4 + c] - b[p * 4 + c]) > 2.5f / 255.0f;
        bad += differs ? 1 : 0;
        covered += a[p * 4] != 0 || a[p * 4 + 1] != 0 ? 1 : 0;
    }
    expect(backend.gpu_active() && device.kernel_ibs >= 1 && device.vm_mismatches == 0 && device.ib_faults == 0 &&
           device.model->stats().errors == 0 && covered > 100 && bad <= 8,
           "clear and smooth triangle rendered by kernel-ring IBs match gl_raster");
    std::printf("kernel ring backend: ibs=%u diff=%u covered=%u\n", device.kernel_ibs, bad, covered);
}

} // namespace

// radeongl-complete (2026-10-24): submission layout 4 (v4 keys, stipple,
// texture levels, pixel sources, GL_FRONT_AND_BACK) is validated field by
// field before any lease or GPU work; layout 3 keeps its meaning.
void layout4_validation() {
    Session s;
    const uint64_t color2 = s.buffer(rx7900::opengl::gfx11::linear_pitch_pixels(s.width, rx7900::opengl::gfx11::TargetKind::bgra8) *
                                     4u * s.height, 0x40000000ULL, Access::read_write);
    using Mutate = std::function<void(wire::SubmitHeader&, wire::GraphicsState&, wire::GraphicsCommand&)>;
    // A layout-4 payload; the base state is a pixel-rectangle draw (pixel
    // source = the texture buffer) unless the case changes it.
    const auto payload = [&](const Mutate& mutate) {
        wire::SubmitHeader h{wire::submit_header_v4_bytes, 1, 1, 3 * 80, s.color, s.depth, s.width, s.height, 0, 0};
        h.layout = wire::submit_layout_v4;
        wire::GraphicsState st = s.state(false);
        st.key.pixel_source = abi::pixel_source_rgba8;
        st.pixel_buffer = s.texture;
        st.pixel_columns = 4;
        st.pixel_end_row = 4;
        st.inv_zoom_x = st.inv_zoom_y = float_bits(1.0f);
        wire::GraphicsCommand c{0, 3, 0, 3, 0, 0};
        if (mutate) mutate(h, st, c);
        std::vector<uint8_t> out(wire::submit_payload_bytes(h));
        wire::encode_header(h, out.data(), out.size());
        wire::encode_state_v4(st, out.data() + h.header_bytes, wire::state_v4_bytes);
        wire::encode_command(c, out.data() + h.header_bytes + wire::state_v4_bytes, wire::command_bytes);
        uint8_t* v = out.data() + h.header_bytes + wire::state_v4_bytes + wire::command_bytes;
        const float clip[3][4] = {{-1, -1, 0, 1}, {1, -1, 0, 1}, {0, 1, 0, 1}};
        for (uint32_t i = 0; i < 3; ++i) {
            for (int k = 0; k < 4; ++k) {
                const uint32_t bits = float_bits(clip[i][k]);
                std::memcpy(v + i * 80 + k * 4, &bits, 4);
            }
        }
        return out;
    };
    uint32_t seq = 0;
    expect(s.submit(payload(nullptr), seq) == Status::success && seq != 0,
           "layout 4: a pixel-rectangle draw with a readable image buffer submits");
    expect(s.device.graphics.query(s.device.queue.id, seq) == Status::success && s.leases(s.texture) == 0,
           "layout 4: its image lease returns with the fence");
    const Mutate lines = [](auto& h, auto& st, auto& c) {
        (void)h;
        st.key = abi::ShaderKey{};
        st.key.vertices_per_primitive = 2;
        st.key.line_stipple = 1;
        st.line_stipple_factor = 3;
        st.line_stipple_pattern = 0xf0f0;
        st.pixel_buffer = 0; st.pixel_columns = 0; st.pixel_end_row = 0; st.inv_zoom_x = st.inv_zoom_y = 0;
        c.primitive = 2;
        c.vertex_count = 2;
    };
    expect(s.submit(payload(lines), seq) == Status::success, "layout 4: a stippled line draw submits");
    s.device.graphics.query(s.device.queue.id, seq);
    const Mutate both = [&](auto& h, auto& st, auto&) {
        h.color2 = color2;
        st.key = abi::ShaderKey{};
        st.key.front_and_back = 1;
        st.pixel_buffer = 0; st.pixel_columns = 0; st.pixel_end_row = 0; st.inv_zoom_x = st.inv_zoom_y = 0;
    };
    expect(s.submit(payload(both), seq) == Status::success && s.leases(color2) == 1,
           "layout 4: GL_FRONT_AND_BACK submits and leases the second colour buffer");
    s.device.graphics.query(s.device.queue.id, seq);
    const uint32_t before = s.device.submits;

    struct Case { const char* what; Mutate m; bool cross; };
    const Case cases[] = {
        {"layout field not 4", [](auto& h, auto&, auto&) { h.layout = 3; }, false},
        {"header reserved2", [](auto& h, auto&, auto&) { h.reserved2 = 1; }, false},
        {"second colour equals the first", [&](auto& h, auto&, auto&) { h.color2 = s.color; }, false},
        {"second colour without GL_FRONT_AND_BACK", [&](auto& h, auto&, auto&) { h.color2 = color2; }, true},
        {"GL_FRONT_AND_BACK without a second colour", [](auto&, auto& st, auto&) { st.key.front_and_back = 1; }, true},
        {"stipple factor 0", [&](auto& h, auto& st, auto& c) { lines(h, st, c); st.line_stipple_factor = 0; }, false},
        {"stipple factor 257", [&](auto& h, auto& st, auto& c) { lines(h, st, c); st.line_stipple_factor = 257; }, false},
        {"stipple pattern above 16 bits", [&](auto& h, auto& st, auto& c) { lines(h, st, c); st.line_stipple_pattern = 0x10000; }, false},
        {"stipple fields without the key", [](auto&, auto& st, auto&) { st.line_stipple_factor = 1; }, false},
        {"line stipple on triangles", [](auto&, auto& st, auto&) { st.key.line_stipple = 1; st.line_stipple_factor = 1; }, false},
        {"polygon rows without the key", [](auto&, auto& st, auto&) { st.polygon_stipple[7] = 1; }, false},
        {"pixel source without a buffer", [](auto&, auto& st, auto&) { st.pixel_buffer = 0; }, false},
        {"pixel source with zero columns", [](auto&, auto& st, auto&) { st.pixel_columns = 0; }, false},
        {"pixel rows empty", [](auto&, auto& st, auto&) { st.pixel_first_row = 4; }, false},
        {"pixel rows beyond 2^24", [](auto&, auto& st, auto&) { st.pixel_end_row = (1u << 24) + 1; }, false},
        {"NaN raster position", [](auto&, auto& st, auto&) { st.raster_x = 0x7fc00000u; }, false},
        {"transfer fields without the key", [](auto&, auto& st, auto&) { st.transfer_scale[0] = float_bits(1.0f); }, false},
        {"index shift 32", [](auto&, auto& st, auto&) { st.key.pixel_transfer = 1; st.index_shift_left = 32; }, false},
        {"stencil source with fog", [](auto&, auto& st, auto&) { st.key.pixel_source = abi::pixel_source_stencil8; st.key.fog = 1; }, false},
        {"depth source without a depth buffer", [](auto& h, auto& st, auto&) { h.depth = 0; st.key.pixel_source = abi::pixel_source_depth32; }, true},
        {"pixel source on points", [](auto&, auto& st, auto& c) { st.key.vertices_per_primitive = 1; c.primitive = 1; }, false},
        {"magnification filter without LOD", [](auto&, auto& st, auto&) { st.key.mag_filter = 1; }, false},
        {"LOD filter without a texture", [](auto&, auto& st, auto&) { st.key.min_filter = abi::texture_min_linear; }, false},
        {"two levels for a non-mipmap filter", [&](auto&, auto& st, auto&) {
             st.texture = s.texture; st.key.textured = 1; st.texture_width = 16; st.texture_height = 16;
             st.texture_pitch = 64; st.level_count = 2; st.levels[0] = {16384, 8, 8, 64}; }, false},
        {"mip level outside its texture buffer", [&](auto&, auto& st, auto&) {
             st.texture = s.texture; st.key.textured = 1; st.key.min_filter = abi::texture_min_nearest_mipmap_nearest;
             st.texture_width = 16; st.texture_height = 16; st.texture_pitch = 64; st.level_count = 2;
             st.levels[0] = {61440, 8, 8, 64}; }, true},
        {"unused level fields", [](auto&, auto& st, auto&) { st.levels[9].pitch = 64; }, false},
        {"reserved v4 dword", [](auto&, auto& st, auto&) { st.reserved_v4 = 1; }, false},
        {"nonzero v4 key reserved byte", [](auto&, auto& st, auto&) { st.key.reserved[3] = 1; }, false},
    };
    for (const auto& c : cases) {
        const auto bytes = payload(c.m);
        expect(s.submit(bytes, seq) != Status::success && seq == 0, std::string("layout 4 refuses: ") + c.what);
        const rx7900::ByteView view(bytes.data(), bytes.size());
        wire::SubmitHeader h{};
        wire::GraphicsState st{};
        wire::GraphicsCommand cmd{};
        const bool decoded = wire::decode_header(view, h) && view.size() == wire::submit_payload_bytes(h) &&
            wire::decode_state_v4(view, wire::submit_header_v4_bytes, st) &&
            wire::decode_command(view, wire::submit_header_v4_bytes + wire::state_v4_bytes, 1, h.vertex_bytes, cmd);
        expect(decoded == c.cross, std::string(c.cross ? "cross-field case passes the decoders: " :
                                                         "wire decoder alone refuses: ") + c.what);
    }
    expect(s.device.submits == before && s.leases(s.texture) == 0 && s.leases(color2) == 0,
           "no refused layout-4 request reached the queue or kept a lease");
    // Layout 3 is not layout 4: a 48-byte header with layout-4 states is refused.
    auto mixed = payload(nullptr);
    mixed[0] = static_cast<uint8_t>(wire::submit_header_bytes);
    expect(s.submit(mixed, seq) == Status::invalid_argument, "a layout-3 header never decodes layout-4 states");
}

int main() {
    expect(database().pinned_ok(), "pinned headers load");
    baseline();
    forged_handles();
    mappings();
    counts_and_enums();
    request_boundary();
    allow_list_and_leases();
    present_bounds();
    kernel_ring_route();
    kernel_ring_failures();
    kernel_ring_refusals();
    kernel_ring_backend();
    layout4_validation();
    return checks.finish("process_graphics");
}

/*
Module: rx7900.tests.gfx11_graphics_device
Purpose: A host "device" for the GFX11 OpenGL backend: the production kernel
modules (ProcessMemory, ProcessGraphics, UserQueues, the gl-shaders generator)
over the synthetic MES/GTT/page-table fixture, a request dispatcher that mirrors
gpu_process's version-3 handling, and the gfx11_pipeline_model executing every
kernel-built IB the gfx queue receives.
Created: 2026-10-23
Architecture: Test-only GpuTransport. Requests are decoded with the production
process_abi decoder; IBs are found by walking the real gfx ring the production
UserQueues wrote, then read through the session's real page tables. Present
rectangles stand in for the GDI CLIPOBJ radeondisp supplies. Not hardware
evidence: MES, CP and every shader stage are models. The kernel gfx-ring route
(submission-irq, 2026-10-24) is served by KernelRing in the place of
KernelGraphicsBridge/Gfx11Queues: it accepts only process-VM jobs, fetches the
IB at the job's VA through the page directory the job names (as the VM flush
programs it) and signals fences when the test lets it.
*/
#pragma once

#include <functional>
#include <memory>
#include <vector>

#include "user_queue_fixture.hpp"
#include "gfx11_pipeline_model.hpp"
#include "../src/reactos/process_graphics.hpp"
#include "../src/opengl/gl_gfx11_backend.hpp"
#include "../src/opengl/gl_shader_gfx11.hpp"

namespace gfx11_graphics_device {

using namespace rx7900_test::user_queue_fixture;
namespace pabi = rx7900::reactos::process_abi;
namespace wire = rx7900::reactos::graphics_abi;
using rx7900::memory::Status;

inline bool generate_vertex(void* context, const rx7900::opengl::gfx11_abi::ShaderKey& key,
                            rx7900::sdma::CommandBuffer out, rx7900::opengl::gfx11_abi::ProgramInfo& info) noexcept {
    auto& workspace = *static_cast<rx7900::opengl::shader::GeneratorWorkspace*>(context);
    return rx7900::opengl::shader::generate_vertex_shader(key, out, info, workspace) ==
           rx7900::opengl::shader::Status::success;
}
inline bool generate_pixel(void* context, const rx7900::opengl::gfx11_abi::ShaderKey& key,
                           rx7900::sdma::CommandBuffer out, rx7900::opengl::gfx11_abi::ProgramInfo& info) noexcept {
    auto& workspace = *static_cast<rx7900::opengl::shader::GeneratorWorkspace*>(context);
    return rx7900::opengl::shader::generate_pixel_shader(key, out, info, workspace) ==
           rx7900::opengl::shader::Status::success;
}
inline bool run_direct(void*, bool (*body)(void*) noexcept, void* argument) noexcept { return body(argument); }
inline void no_wait(void*, uint32_t) noexcept {}

// The kernel gfx ring of the KernelGraphicsBridge contract: submit leases the
// IB backing (as Gfx11Queues does), executes the IB through the VM the job
// names, and its fence signals at once (autosignal) or on signal(). Every
// refusal/failure the production queue can answer is injectable.
struct KernelRing final {
    std::function<bool(const rx7900::hardware::Gfx11VmJob&, uint32_t dwords)> execute;
    rx7900::memory::Manager* gtt = nullptr;
    uint32_t emitted = 0;
    uint32_t signalled = 0;
    bool autosignal = true;
    bool hung = false;
    bool reset_confirmed = false;
    rx7900::hardware::Gfx11QueuesStatus answer = rx7900::hardware::Gfx11QueuesStatus::running;
    std::vector<std::pair<uint32_t, rx7900::memory::Handle>> ib_leases;
    std::vector<rx7900::hardware::Gfx11VmJob> jobs;
    unsigned vmid0_submissions = 0;

    static rx7900::hardware::Gfx11QueuesStatus submit(void* context, const rx7900::hardware::Gfx11Submission& frame,
                                                      uint32_t& sequence) noexcept {
        auto& self = *static_cast<KernelRing*>(context);
        sequence = 0;
        if (frame.vm == nullptr) {
            ++self.vmid0_submissions;
            return rx7900::hardware::Gfx11QueuesStatus::invalid_argument;
        }
        if (self.answer != rx7900::hardware::Gfx11QueuesStatus::running) {
            // A doorbell failure after publication: the frame may still run.
            if (self.answer == rx7900::hardware::Gfx11QueuesStatus::doorbell_failure) ++self.emitted;
            return self.answer;
        }
        if (frame.ring != rx7900::hardware::fence_ring_gfx ||
            self.gtt->retain(frame.ib, rx7900::memory::Access::read) != Status::success) {
            return rx7900::hardware::Gfx11QueuesStatus::invalid_memory;
        }
        sequence = ++self.emitted;
        self.ib_leases.emplace_back(sequence, frame.ib);
        self.jobs.push_back(*frame.vm);
        if (self.execute) self.execute(*frame.vm, frame.ib_dwords);
        if (self.autosignal) self.signalled = sequence;
        return rx7900::hardware::Gfx11QueuesStatus::running;
    }
    static rx7900::hardware::FenceStatus query(void* context, uint32_t sequence) noexcept {
        auto& self = *static_cast<KernelRing*>(context);
        if (sequence == 0 || sequence > self.emitted) return rx7900::hardware::FenceStatus::invalid_argument;
        if (sequence <= self.signalled) return rx7900::hardware::FenceStatus::signalled;
        return self.hung ? rx7900::hardware::FenceStatus::ring_hung : rx7900::hardware::FenceStatus::pending;
    }
    static rx7900::hardware::Gfx11QueuesStatus retire(void* context) noexcept {
        auto& self = *static_cast<KernelRing*>(context);
        for (size_t i = 0; i < self.ib_leases.size();) {
            if (self.ib_leases[i].first <= self.signalled) {
                self.gtt->unretain(self.ib_leases[i].second);
                self.ib_leases.erase(self.ib_leases.begin() + static_cast<long>(i));
            } else {
                ++i;
            }
        }
        return rx7900::hardware::Gfx11QueuesStatus::running;
    }
    static bool retired_after_reset(void* context) noexcept {
        auto& self = *static_cast<KernelRing*>(context);
        if (!self.reset_confirmed) return false;
        // The reset proof returns every ring lease (Gfx11Queues::release_after_reset).
        for (const auto& lease : self.ib_leases) self.gtt->unretain(lease.second);
        self.ib_leases.clear();
        return true;
    }
    rx7900::opengl::GraphicsSubmitter submitter() noexcept { return {this, submit, query, retire, retired_after_reset}; }
    void signal() noexcept { signalled = emitted; }
};

class Device final : public rx7900::opengl::GpuTransport {
public:
    // gtt_bytes: model GTT size (0 keeps the fixture default; big-surface tests).
    explicit Device(const gfx11_header_db::Database& db, uint32_t scanout_width = 0, uint32_t scanout_height = 0,
                    uint64_t gtt_bytes = 0)
        : fx(0x00001063u, 2100000, gtt_bytes != 0 ? gtt_bytes : rx7900_test::process_model::gtt_bytes),
          db_(db) {
        ok = fx.up();
        device.generator = {shader_workspace.get(), generate_vertex, generate_pixel};
        device.fpu = {nullptr, run_direct};
        device.wait_us = no_wait;
        rx7900::memory::Handle ring{};
        fx.vram->allocate(65536, 65536, rx7900::memory::Access::read_write, true, ring);
        device.attribute_ring = true;
        device.attribute_ring_mc = fx.info(*fx.vram, ring).gpu.virtual_address;
        device.attribute_ring_per_se_bytes = 65536;
        device.shader_engines = 1;
        attribute_ring.base = rx7900::reactos::graphics_attribute_va;
        attribute_ring.bytes.assign(65536, 0);
        if (scanout_width != 0) {
            rx7900::memory::Handle scan{};
            const uint64_t bytes = static_cast<uint64_t>(scanout_width) * scanout_height * 4;
            fx.vram->allocate((bytes + 4095) & ~4095ULL, 4096, rx7900::memory::Access::read_write, true, scan);
            device.scanout = true;
            device.scanout_mc = fx.info(*fx.vram, scan).gpu.virtual_address;
            device.scanout_width = scanout_width;
            device.scanout_height = scanout_height;
            device.scanout_pitch_bytes = scanout_width * 4;
            scanout.assign(static_cast<size_t>(scanout_width) * scanout_height, 0xff102030u);
        }
        model = std::make_unique<gfx11_pipeline_model::Machine>(db_, port());
        model->host_buffers.push_back(&attribute_ring);
        kernel.gtt = fx.gtt.get();
        kernel.execute = [this](const rx7900::hardware::Gfx11VmJob& job, uint32_t dwords) {
            return run_vm_ib(job, dwords);
        };
    }

    static bool dma_bus(void* context, rx7900::memory::Handle handle, uint64_t& bus) noexcept {
        auto& self = *static_cast<Device*>(context);
        rx7900::memory::AllocationInfo info{};
        if (self.fx.gtt->describe(handle, info) != Status::success) return false;
        bus = bus_base + info.gpu.virtual_address - self.fx.gart.start;
        return true;
    }

    rx7900::reactos::ProcessGraphicsConfig graphics_config() {
        rx7900::reactos::ProcessGraphicsConfig c{};
        c.queues = fx.queues.get();
        c.buffers = buffers.get();
        c.gtt = fx.gtt.get();
        c.pasid = process.pasid;
        c.context = this;
        c.dma_bus = dma_bus;
        c.device = &device;
        c.workspace = workspace.get();
        if (kernel_ring_enabled) c.kernel_ring = kernel.submitter();
        return c;
    }

    // ---- GpuTransport ----
    bool call(const uint8_t* request, uint32_t request_bytes, uint8_t* reply, uint32_t reply_capacity,
              uint32_t& reply_bytes) noexcept override {
        ++calls;
        pabi::Request r{};
        reply_bytes = 0;
        if (!pabi::decode_request(rx7900::ByteView(request, request_bytes), r) || !pabi::valid_request_fields(r)) {
            return false;
        }
        const rx7900::ByteView payload(request + 64, request_bytes - 64);
        uint32_t payload_out = 0;
        uint64_t handle = r.handle, bytes = 0, session_out = r.session;
        const pabi::Outcome outcome = execute(r, payload, reply + 48, reply_capacity - 48, payload_out, handle, bytes,
                                              session_out);
        rx7900::reactos::graphics_abi::Writer w(reply, 48, 0);
        w.u32(48); w.u32(r.abi_version); w.u32(static_cast<uint32_t>(outcome)); w.u32(payload_out);
        w.u64(session_out); w.u64(handle); w.u64(bytes); w.u64(0);
        reply_bytes = 48 + payload_out;
        return true;
    }

    bool present(const uint8_t* input, uint32_t input_bytes, uint32_t& outcome, uint32_t& rects) noexcept override {
        ++presents;
        if (!present_hook) return false;
        wire::PresentRequest request{};
        if (!wire::decode_present_input(rx7900::ByteView(input, input_bytes), request.input)) return false;
        // radeondisp's role: window origin and the CLIPOBJ rectangles.
        request.window_left = window_left;
        request.window_top = window_top;
        request.rect_count = static_cast<uint32_t>(clip.size());
        for (size_t i = 0; i < clip.size(); ++i) request.rects[i] = clip[i];
        uint8_t encoded[wire::present_request_bytes];
        if (!wire::encode_present_request(request, encoded, sizeof(encoded)) ||
            !wire::decode_present_request(rx7900::ByteView(encoded, sizeof(encoded)), request)) return false;
        if (request.input.session != session || !open) {
            outcome = static_cast<uint32_t>(pabi::Outcome::access_denied);
            return true;
        }
        const Status status = graphics.present(request, rects);
        run_new_ibs();
        outcome = static_cast<uint32_t>(status == Status::success ? pabi::Outcome::success :
                                        status == Status::busy ? pabi::Outcome::busy : pabi::Outcome::invalid_request);
        return true;
    }

    void yield() noexcept override { fx.queues->poll(); }

    // ---- State visible to tests ----
    Fixture fx;
    bool ok = false;
    std::unique_ptr<rx7900::opengl::shader::GeneratorWorkspace> shader_workspace =
        std::make_unique<rx7900::opengl::shader::GeneratorWorkspace>();
    rx7900::reactos::GraphicsDevice device{};
    std::unique_ptr<rx7900::reactos::GraphicsWorkspace> workspace = std::make_unique<rx7900::reactos::GraphicsWorkspace>();
    std::unique_ptr<rx7900::reactos::ProcessMemory> buffers = std::make_unique<rx7900::reactos::ProcessMemory>();
    rx7900::reactos::ProcessGraphics graphics;
    Fixture::Process process{};
    Fixture::Gang gang{};
    Fixture::Queue queue{};
    uint64_t session = 0;
    uint64_t queue_handle = 0;
    bool open = false;
    unsigned calls = 0, presents = 0, submits = 0;
    bool present_hook = true;
    int32_t window_left = 0, window_top = 0;
    std::vector<wire::PresentRect> clip;
    std::vector<uint32_t> scanout;
    gfx11_interp::Buffer attribute_ring;
    std::unique_ptr<gfx11_pipeline_model::Machine> model;
    uint64_t ring_position = 0;

private:
    pabi::Outcome outcome_of(Status s) {
        switch (s) {
        case Status::success: return pabi::Outcome::success;
        case Status::access_denied: return pabi::Outcome::access_denied;
        case Status::busy: return pabi::Outcome::busy;
        case Status::exhausted: case Status::out_of_memory: return pabi::Outcome::exhausted;
        case Status::quarantined: return pabi::Outcome::quarantined;
        case Status::unavailable: return pabi::Outcome::unavailable;
        default: return pabi::Outcome::invalid_request;
        }
    }

    pabi::Outcome execute(const pabi::Request& r, rx7900::ByteView payload, uint8_t* out, uint32_t out_capacity,
                          uint32_t& payload_out, uint64_t& handle, uint64_t& bytes, uint64_t& session_out) {
        using Op = pabi::Operation;
        const auto op = static_cast<Op>(r.operation);
        if (op == Op::query) return pabi::Outcome::success;
        if (op == Op::open) {
            if (open) return pabi::Outcome::busy;
            process = fx.process(64, true);
            if (process.status != UserQueueStatus::created ||
                buffers->initialize({fx.queues.get(), process.pasid, fx.vram.get(), fx.gtt.get(), this, dma_bus}) !=
                    Status::success) return pabi::Outcome::unavailable;
            gang = fx.gang(process.pasid);
            open = true;
            session = 0x5e55;
            session_out = session;
            return pabi::Outcome::success;
        }
        if (!open || r.session != session) return pabi::Outcome::access_denied;
        // The kernel gfx-ring route (Request.handle 0), as gpu_process_core.
        const bool kernel_op = r.handle == 0 &&
            (op == Op::graphics_open || op == Op::graphics_submit || op == Op::graphics_query);
        const uint32_t graphics_queue = kernel_op ? 0 : queue.id;
        if (kernel_op && op != Op::graphics_open && !graphics.kernel_route()) return pabi::Outcome::invalid_request;
        if (kernel_op && op == Op::graphics_open && !kernel_ring_enabled) return pabi::Outcome::unavailable;
        switch (op) {
        case Op::queue_create:
            if (r.domain != 0 || queue_handle != 0) return pabi::Outcome::invalid_request;
            if (mes_gfx_unavailable) return pabi::Outcome::unavailable;
            queue = fx.queue(gang.id, Mes11QueueType::gfx, 1ULL << 44, 8192);
            if (queue.status != UserQueueStatus::added) return pabi::Outcome::unavailable;
            queue_handle = 0x9001;
            handle = queue_handle;
            return pabi::Outcome::success;
        case Op::graphics_open:
            if (!kernel_op && r.handle != queue_handle) return pabi::Outcome::invalid_request;
            return outcome_of(graphics.open(graphics_config(), graphics_queue));
        case Op::graphics_submit: {
            if (!kernel_op && r.handle != queue_handle) return pabi::Outcome::invalid_request;
            uint32_t sequence = 0;
            const Status s = graphics.submit(graphics_queue, payload, sequence);
            bytes = sequence;
            if (s == Status::success) {
                ++submits;
                if (!kernel_op) run_new_ibs();
            }
            return outcome_of(s);
        }
        case Op::graphics_query:
            if (!kernel_op && r.handle != queue_handle) return pabi::Outcome::invalid_request;
            bytes = r.offset;
            fx.queues->poll();
            return outcome_of(graphics.query(graphics_queue, static_cast<uint32_t>(r.offset)));
        case Op::queue_destroy:
            if (r.handle != queue_handle || queue_handle == 0) return pabi::Outcome::invalid_request;
            if (fx.queues->remove_queue(queue.id) != UserQueueStatus::removed ||
                graphics.retire_removed_queue(queue.id) != Status::success) return pabi::Outcome::quarantined;
            queue_handle = 0;
            return pabi::Outcome::success;
        case Op::allocate:
            return outcome_of(buffers->allocate(r.bytes, static_cast<rx7900::reactos::ProcessMemoryDomain>(r.domain),
                                                static_cast<rx7900::memory::Access>(r.access), handle));
        case Op::map:
            return outcome_of(buffers->map(r.handle, r.address, static_cast<rx7900::memory::Access>(r.access)));
        case Op::unmap: return outcome_of(buffers->unmap(r.handle));
        case Op::release: return outcome_of(buffers->release(r.handle));
        case Op::buffer_upload:
        case Op::buffer_readback: {
            for (uint32_t done = 0; done < r.bytes;) {
                const uint32_t chunk = static_cast<uint32_t>(r.bytes - done < 4096 ? r.bytes - done : 4096);
                Status s = Status::invalid_argument;
                if (op == Op::buffer_upload) {
                    rx7900::ByteView part(nullptr, 0);
                    if (!payload.subview(done, chunk, part)) return pabi::Outcome::invalid_request;
                    s = buffers->write(r.handle, r.offset + done, part);
                } else {
                    if (r.bytes > out_capacity) return pabi::Outcome::invalid_request;
                    s = buffers->read(r.handle, r.offset + done, out + done, chunk);
                }
                if (s != Status::success) return outcome_of(s);
                done += chunk;
            }
            if (op == Op::buffer_readback) payload_out = static_cast<uint32_t>(r.bytes);
            return pabi::Outcome::success;
        }
        case Op::close:
            return pabi::Outcome::success;
        default:
            return pabi::Outcome::invalid_request;
        }
    }

    uint64_t root() {
        // A kernel-ring job runs with the page directory its VM flush named.
        if (graphics.kernel_route()) return vm_root;
        const auto report = fx.queues->queue_report(queue.id);
        const auto* frame = fx.engines.active(report.doorbell_index);
        return frame == nullptr ? 0 : (static_cast<uint64_t>(frame->at(3)) << 32) | frame->at(2);
    }

    bool translate(uint64_t va, uint8_t* bytes, size_t count, bool write) {
        if (device.scanout && va >= rx7900::reactos::graphics_scanout_va &&
            va + count <= rx7900::reactos::graphics_scanout_va + scanout.size() * 4 +
                (device.scanout_mc & 4095)) {
            const uint64_t offset = va - rx7900::reactos::graphics_scanout_va - (device.scanout_mc & 4095);
            auto* base = reinterpret_cast<uint8_t*>(scanout.data());
            if (write) std::memcpy(base + offset, bytes, count);
            else std::memcpy(bytes, base + offset, count);
            return true;
        }
        const uint64_t r = root();
        for (size_t i = 0; i < count; ++i) {
            uint8_t* cpu = fx.engines.translate(r, va + i, write);
            if (cpu == nullptr) return false;
            if (write) *cpu = bytes[i];
            else bytes[i] = *cpu;
        }
        return true;
    }

    gfx11_pipeline_model::MemoryPort port() {
        return {[this](uint64_t va, uint8_t* out, size_t n) { return translate(va, out, n, false); },
                [this](uint64_t va, const uint8_t* in, size_t n) {
                    return translate(va, const_cast<uint8_t*>(in), n, true);
                }};
    }

public:
    void run_new_ibs_public() { run_new_ibs(); }

    // KernelRing::execute: the job's PASID must be this session's and its page
    // directory the session VM's (amdgpu_gmc_pd_addr); the IB is then fetched
    // at the job VA through those tables, as the CP does in the job VMID.
    bool run_vm_ib(const rx7900::hardware::Gfx11VmJob& job, uint32_t dwords) {
        const auto vm = fx.queues->vm_report(process.pasid);
        if (job.pasid != process.pasid || job.pd_address != vm.pd_address || vm.pd_address == 0) {
            ++vm_mismatches;
            return false;
        }
        vm_root = job.pd_address & 0x0000fffffffff000ULL;
        std::vector<uint8_t> bytes(static_cast<size_t>(dwords) * 4);
        if (!translate(job.ib_va, bytes.data(), bytes.size(), false)) {
            ++ib_faults;
            return false;
        }
        std::vector<uint32_t> words(dwords);
        for (uint32_t i = 0; i < dwords; ++i) {
            words[i] = static_cast<uint32_t>(bytes[i * 4]) | (static_cast<uint32_t>(bytes[i * 4 + 1]) << 8) |
                       (static_cast<uint32_t>(bytes[i * 4 + 2]) << 16) | (static_cast<uint32_t>(bytes[i * 4 + 3]) << 24);
        }
        model->execute(words);
        ++ibs;
        ++kernel_ibs;
        return true;
    }

    KernelRing kernel{};
    bool kernel_ring_enabled = false;
    bool mes_gfx_unavailable = false;
    unsigned kernel_ibs = 0, vm_mismatches = 0;
    uint64_t vm_root = 0;

private:
    // Walk the gfx ring the production UserQueues wrote since the last call and
    // execute each INDIRECT_BUFFER through the pipeline model.
    void run_new_ibs() {
        const auto report = fx.queues->queue_report(queue.id);
        const uint8_t* ring = fx.cpu_of(queue.buffer);
        const uint64_t dwords = report.ring_bytes / 4;
        const auto dw = [&](uint64_t index) {
            const size_t at = static_cast<size_t>(index % dwords) * 4;
            return static_cast<uint32_t>(ring[at]) | (static_cast<uint32_t>(ring[at + 1]) << 8) |
                   (static_cast<uint32_t>(ring[at + 2]) << 16) | (static_cast<uint32_t>(ring[at + 3]) << 24);
        };
        while (ring_position < report.wptr) {
            const uint32_t header = dw(ring_position);
            if (header == 0xffff1000u) { ++ring_position; continue; }
            const uint32_t count = ((header >> 16) & 0x3fff) + 1;
            if (((header >> 8) & 0xff) == 0x3f) {
                const uint64_t ib = dw(ring_position + 1) | (static_cast<uint64_t>(dw(ring_position + 2) & 0xffff) << 32);
                const uint32_t size = dw(ring_position + 3) & 0xfffff;
                std::vector<uint32_t> words(size);
                std::vector<uint8_t> bytes(static_cast<size_t>(size) * 4);
                if (translate(ib, bytes.data(), bytes.size(), false)) {
                    for (uint32_t i = 0; i < size; ++i) {
                        words[i] = static_cast<uint32_t>(bytes[i * 4]) | (static_cast<uint32_t>(bytes[i * 4 + 1]) << 8) |
                                   (static_cast<uint32_t>(bytes[i * 4 + 2]) << 16) |
                                   (static_cast<uint32_t>(bytes[i * 4 + 3]) << 24);
                    }
                    model->execute(words);
                    ++ibs;
                } else {
                    ++ib_faults;
                }
            }
            ring_position += count + 1;
        }
    }

public:
    unsigned ibs = 0, ib_faults = 0;

private:
    const gfx11_header_db::Database& db_;
};

} // namespace gfx11_graphics_device
